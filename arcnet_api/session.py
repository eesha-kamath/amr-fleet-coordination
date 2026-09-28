import asyncio
from math import sqrt

from arcnet_core.robot import RobotSpec
from arcnet_core.grid import Grid
from arcnet_core.obstacle import Obstacle
from arcnet_core.config import SafetyConfig
from arcnet_core.engine import ArcnetEngine
from arcnet_core.simulator import WarehouseSimulator
from arcnet_core.task import Task, TaskStatus
from arcnet_core.motion import MotionCommand
from arcnet_core.scenarios import ScenarioLibrary
from arcnet_core.faults import FaultManager


class SimulationSession:
    def __init__(self):
        self.scenario_id = "default"
        self.running = False
        self.loop_task = None
        self.events = []
        self.faults = FaultManager()
        self.library = ScenarioLibrary()

        self.grid = Grid(20, 12, 1.0)

        self.specs = {
            "R1": self._spec("R1"),
            "R2": self._spec("R2"),
            "R3": self._spec("R3")
        }

        self.paths = {}

        self.reset()

    def _spec(self, robot_id):
        return RobotSpec(
            robot_id,
            "rectangle",
            0.9,
            0.64,
            0.19,
            70,
            0.5,
            0.08,
            1.0,
            0.5,
            0.8,
            0.1
        )

    def _create(self, obstacles=None):
        obstacles = obstacles or []

        self.sim = WarehouseSimulator(
            self.grid,
            obstacles,
            self.specs
        )

        self.engine = ArcnetEngine(
            self.specs,
            self.grid,
            obstacles,
            SafetyConfig()
        )

    def reset(self):
        self.running = False
        self.events.clear()
        self.faults.clear()
        self.paths.clear()

        self._create()

        self.sim.add_robot("R1", 2.5, 3.5)
        self.sim.add_robot("R2", 17.5, 3.5)
        self.sim.add_robot("R3", 10.5, 8.5)

        self.sim.add_task(
            Task("T001", (2, 3), (17, 3), "R1")
        )

        self.sim.add_task(
            Task("T002", (17, 3), (2, 3), "R2")
        )

        self.sim.add_task(
            Task("T003", (10, 8), (10, 2), "R3")
        )

        self.log("SYSTEM", "Simulation reset")

    def scenarios(self):
        result = []

        all_scenarios = self.library.all()

        for scenario_id, builder in all_scenarios.items():
            try:
                data = builder()

                result.append({
                    "id": scenario_id,
                    "name": data.get(
                        "name",
                        scenario_id
                    ),
                    "description": data.get(
                        "description",
                        ""
                    )
                })
            except Exception:
                result.append({
                    "id": scenario_id,
                    "name": scenario_id,
                    "description": ""
                })

        return result

    def load_scenario(self, scenario_id):
        self.running = False
        self.scenario_id = scenario_id
        self.events.clear()
        self.faults.clear()
        self.paths.clear()

        data = self.library.get(scenario_id)

        if not data:
            raise ValueError(
                f"Unknown scenario: {scenario_id}"
            )

        obstacles = list(
            data.get("obstacles", [])
        )

        self._create(obstacles)

        for robot in data.get("robots", []):
            self.sim.add_robot(
                robot["id"],
                robot["x"],
                robot["y"],
                robot.get("theta", 0.0),
                robot.get("battery", 100.0)
            )

        for task in data.get("tasks", []):
            if isinstance(task, Task):
                self.sim.add_task(task)
            else:
                self.sim.add_task(
                    Task(
                        task["id"],
                        tuple(task["start"]),
                        tuple(task["destination"]),
                        task.get("assigned_robot")
                    )
                )

        self.log(
            "SCENARIO",
            f"Loaded {scenario_id}"
        )

    def start(self):
        if self.running:
            return

        self.running = True

        try:
            loop = asyncio.get_running_loop()

            if (
                self.loop_task is None
                or self.loop_task.done()
            ):
                self.loop_task = loop.create_task(
                    self._run_loop()
                )

        except RuntimeError:
            pass

        self.log(
            "SYSTEM",
            "Simulation started"
        )

    def stop(self):
        self.running = False

        for robot in self.sim.robots.values():
            robot.vx = 0.0
            robot.vy = 0.0

        self.log(
            "SYSTEM",
            "Simulation stopped"
        )

    async def _run_loop(self):
        while self.running:
            self.step()
            await asyncio.sleep(0.1)

    def step(self):
        if not self.running:
            return

        self._update_peers()

        commands = {}

        for robot_id, robot in self.sim.robots.items():

            if not robot.active:
                continue

            task = self._get_task(robot_id)

            if not task:
                commands[robot_id] = MotionCommand(
                    0.0,
                    0.0,
                    0.0,
                    True
                )
                continue

            if task.status == TaskStatus.PENDING:
                task.activate()

            goal = task.destination

            current = self.grid.to_cell(
                robot.x,
                robot.y
            )

            if current == goal:
                commands[robot_id] = MotionCommand(
                    0.0,
                    0.0,
                    0.0,
                    True
                )
                continue

            path = self.paths.get(robot_id)

            if (
                not path
                or current not in path
            ):
                path = self.engine.plan_robot(
                    robot_id,
                    current,
                    goal
                )

                self.paths[robot_id] = path

            if not path:
                task.fail()

                self.log(
                    robot_id,
                    f"Task {task.id}: no path"
                )

                commands[robot_id] = MotionCommand(
                    0.0,
                    0.0,
                    0.0,
                    True
                )

                continue

            try:
                index = path.index(current)
            except ValueError:
                index = 0

            if index + 1 >= len(path):
                target = goal
            else:
                target = path[index + 1]

            tx, ty = self.grid.to_position(
                target[0],
                target[1]
            )

            dx = tx - robot.x
            dy = ty - robot.y

            d = sqrt(dx * dx + dy * dy)

            if d < 0.08:
                robot.x = tx
                robot.y = ty

                commands[robot_id] = MotionCommand(
                    0.0,
                    0.0,
                    0.0,
                    True
                )

                continue

            vx = dx / d
            vy = dy / d

            peers = self._get_peers(
                robot_id
            )

            for peer in peers:
                self.engine.set_peer_state(
                    peer["id"],
                    peer["x"],
                    peer["y"],
                    peer["vx"],
                    peer["vy"]
                )

            nearest = self._nearest_distance(
                robot,
                peers
            )

            level, command = (
                self.engine.command_for_distance(
                    robot_id,
                    nearest,
                    vx,
                    vy
                )
            )

            commands[robot_id] = command

        self.sim.step(commands)

        self._check_tasks()
        self._check_safety()

    def _update_peers(self):
        for robot_id, robot in self.sim.robots.items():

            if not robot.active:
                continue

            for other_id, other in self.sim.robots.items():

                if (
                    other_id == robot_id
                    or not other.active
                ):
                    continue

                self.engine.set_peer_state(
                    other_id,
                    other.x,
                    other.y,
                    other.vx,
                    other.vy
                )

    def _get_peers(self, robot_id):
        result = []

        for other_id, robot in self.sim.robots.items():

            if (
                other_id == robot_id
                or not robot.active
            ):
                continue

            result.append({
                "id": other_id,
                "x": robot.x,
                "y": robot.y,
                "vx": robot.vx,
                "vy": robot.vy
            })

        return result

    def _nearest_distance(self, robot, peers):
        if not peers:
            return float("inf")

        best = float("inf")

        for peer in peers:
            dx = robot.x - peer["x"]
            dy = robot.y - peer["y"]

            d = sqrt(
                dx * dx +
                dy * dy
            )

            if d < best:
                best = d

        return best

    def _get_task(self, robot_id):
        for task in self.sim.tasks.values():

            if (
                task.assigned_robot == robot_id
                and not task.completed
                and task.status != TaskStatus.FAILED
            ):
                return task

        return None

    def _check_tasks(self):
        for task in self.sim.tasks.values():

            if (
                task.completed
                or not task.assigned_robot
            ):
                continue

            robot = self.sim.get_robot(
                task.assigned_robot
            )

            if not robot or not robot.active:
                continue

            cell = self.grid.to_cell(
                robot.x,
                robot.y
            )

            if cell == task.destination:
                task.complete()

                self.paths.pop(
                    robot.id,
                    None
                )

                self.log(
                    robot.id,
                    f"Task {task.id} completed"
                )

        if (
            self.sim.tasks
            and all(
                task.completed
                for task in self.sim.tasks.values()
            )
        ):
            self.running = False

            self.log(
                "SYSTEM",
                "All tasks completed"
            )

    def _check_safety(self):
        robots = [
            r for r in self.sim.robots.values()
            if r.active
        ]

        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):

                a = robots[i]
                b = robots[j]

                dx = a.x - b.x
                dy = a.y - b.y

                d = sqrt(
                    dx * dx +
                    dy * dy
                )

                ra = self.specs[a.id].radius
                rb = self.specs[b.id].radius

                if d <= ra + rb:
                    self.log(
                        "SAFETY",
                        f"COLLISION {a.id}/{b.id}"
                    )

    def add_task(self, data):
        task = Task(
            data["id"],
            tuple(data["start"]),
            tuple(data["destination"]),
            data.get("assigned_robot")
        )

        self.sim.add_task(task)

        self.log(
            "TASK",
            f"Added {task.id}"
        )

        return {
            "ok": True,
            "task": task.id
        }

    def assign_task(self, task_id, robot_id):
        task = self.sim.tasks.get(task_id)

        if not task:
            return {
                "ok": False,
                "error": "Task not found"
            }

        if robot_id not in self.sim.robots:
            return {
                "ok": False,
                "error": "Robot not found"
            }

        task.assigned_robot = robot_id
        task.status = TaskStatus.PENDING
        task.completed = False

        self.paths.pop(
            robot_id,
            None
        )

        self.log(
            "TASK",
            f"{task_id} assigned to {robot_id}"
        )

        return {"ok": True}

    def add_fault(self, data):
        kind = data.get("kind")

        if kind == "robot_failure":

            robot_id = data.get("target")

            self.faults.kill_robot(
                robot_id
            )

            robot = self.sim.get_robot(
                robot_id
            )

            if robot:
                self.sim.kill_robot(
                    robot_id
                )

            self.log(
                "FAULT",
                f"Robot {robot_id} failed"
            )

        elif kind == "blocked_cell":

            x = int(data["x"])
            y = int(data["y"])

            self.faults.blocked_cell(
                x,
                y
            )

            obstacle = Obstacle(x, y)

            if obstacle not in self.sim.obstacles:
                self.sim.obstacles.append(
                    obstacle
                )

            self.paths.clear()

            self.log(
                "FAULT",
                f"Blocked cell ({x},{y})"
            )

        elif kind == "wifi_dead_zone":

            x = int(data["x"])
            y = int(data["y"])

            self.faults.wifi_dead_zone(
                x,
                y
            )

            self.log(
                "FAULT",
                f"Wi-Fi dead zone ({x},{y})"
            )

        return {"ok": True}

    def log(self, source, message):
        self.events.append({
            "time": round(
                self.sim.time,
                2
            ),
            "source": source,
            "message": message
        })

        if len(self.events) > 200:
            self.events = self.events[-200:]

    def state(self):
        robots = []

        for robot in self.sim.robots.values():

            task = self._get_task(
                robot.id
            )

            robots.append({
                "id": robot.id,
                "x": robot.x,
                "y": robot.y,
                "theta": robot.theta,
                "vx": robot.vx,
                "vy": robot.vy,
                "battery": robot.battery,
                "active": robot.active,
                "task_id":
                    task.id if task else None
            })

        tasks = []

        for task in self.sim.tasks.values():

            tasks.append({
                "id": task.id,
                "start": list(task.start),
                "destination":
                    list(task.destination),
                "assigned_robot":
                    task.assigned_robot,
                "completed":
                    task.completed,
                "status":
                    task.status.value
            })

        obstacles = [
            {
                "x": obstacle.x,
                "y": obstacle.y
            }
            for obstacle in self.sim.obstacles
        ]

        return {
            "time": round(
                self.sim.time,
                2
            ),
            "running": self.running,
            "scenario": self.scenario_id,
            "grid": {
                "width": self.grid.width,
                "height": self.grid.height,
                "cell_size":
                    self.grid.cell_size
            },
            "robots": robots,
            "tasks": tasks,
            "obstacles": obstacles,
            "events":
                self.events[-100:]
        }