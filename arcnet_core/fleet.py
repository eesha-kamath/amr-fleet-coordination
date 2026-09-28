from math import sqrt

from .motion import MotionCommand
from .metrics import Metrics
from .obstacle import Obstacle


class FleetRunner:

    def __init__(
        self,
        simulator,
        engine,
        max_steps=5000,
        fault_manager=None
    ):
        self.sim = simulator
        self.engine = engine
        self.max_steps = max_steps
        self.fault_manager = fault_manager
        self.metrics = Metrics()
        self.failed_tasks = set()

    def run(self, max_steps=None):
        if max_steps is None:
            max_steps = self.max_steps

        for _ in range(max_steps):

            self._apply_faults()
            self._recover_failed_robots()

            if self._all_tasks_complete():
                self.metrics.completion_time = self.sim.time
                return self.metrics

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

                if task.status.value == "pending":
                    task.activate()

                start = self.sim.grid.to_cell(
                    robot.x,
                    robot.y
                )

                path = self.engine.plan_robot(
                    robot_id,
                    start,
                    task.destination
                )

                if not path:
                    self._handle_plan_failure(task)
                    continue

                self.metrics.record_replan()

                target = path[1] if len(path) > 1 else path[0]

                dx = target[0] - robot.x
                dy = target[1] - robot.y

                d = sqrt(dx * dx + dy * dy)

                if d > 0:
                    vx = dx / d
                    vy = dy / d
                else:
                    vx = 0.0
                    vy = 0.0

                peers = self._get_peers(robot_id)

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

                level, command = self.engine.command_for_distance(
                    robot_id,
                    nearest,
                    vx,
                    vy
                )

                if command.stop:
                    self.metrics.record_stop()
                    self.metrics.event(
                        "robot_stop",
                        time=self.sim.time,
                        robot_id=robot_id
                    )

                commands[robot_id] = command

            self.sim.step(commands)

            self._check_safety()
            self._check_tasks()

        self.metrics.completion_time = self.sim.time
        return self.metrics

    def _apply_faults(self):
        if not self.fault_manager:
            return

        for robot_id in self.fault_manager.failed_robots():
            robot = self.sim.get_robot(robot_id)

            if robot and robot.active:
                self.sim.kill_robot(robot_id)

                self.engine.remove_robot(robot_id)

                self.metrics.event(
                    "robot_failure",
                    time=self.sim.time,
                    robot_id=robot_id
                )

        blocked = self.fault_manager.blocked_cells()

        existing = {
            (o.x, o.y)
            for o in self.sim.obstacles
        }

        for x, y in blocked:
            if (x, y) not in existing:
                self.sim.obstacles.append(
                    Obstacle(x, y, True)
                )

        blocked_set = set(blocked)

        self.sim.obstacles[:] = [
            o
            for o in self.sim.obstacles
            if not o.temporary
            or (o.x, o.y) in blocked_set
        ]

        self.engine.obstacles = self.sim.obstacles

    def _recover_failed_robots(self):
        for task in self.sim.tasks.values():

            if task.completed:
                continue

            robot_id = task.assigned_robot

            if not robot_id:
                continue

            robot = self.sim.get_robot(robot_id)

            if robot and robot.active:
                continue

            task.release()

            self.metrics.event(
                "task_requeued",
                time=self.sim.time,
                task_id=task.id
            )

            self._assign_requeued_task(task)

    def _assign_requeued_task(self, task):
        available = [
            robot_id
            for robot_id, robot in self.sim.robots.items()
            if robot.active
        ]

        if not available:
            return

        best = available[0]

        best_score = self._task_score(
            task,
            best
        )

        for robot_id in available[1:]:

            score = self._task_score(
                task,
                robot_id
            )

            if score < best_score:
                best = robot_id
                best_score = score

        task.assigned_robot = best

        self.metrics.event(
            "task_reassigned",
            time=self.sim.time,
            task_id=task.id,
            robot_id=best
        )

    def _task_score(self, task, robot_id):
        robot = self.sim.get_robot(robot_id)

        if not robot:
            return float("inf")

        dx = robot.x - task.start[0]
        dy = robot.y - task.start[1]

        distance = sqrt(
            dx * dx +
            dy * dy
        )

        battery_penalty = max(
            0.0,
            100.0 - robot.battery
        )

        return distance + battery_penalty * 0.01

    def _handle_plan_failure(self, task):
        if task.id in self.failed_tasks:
            return

        task.fail()

        self.failed_tasks.add(task.id)

        self.metrics.tasks_failed += 1

        self.metrics.event(
            "plan_failure",
            time=self.sim.time,
            task_id=task.id
        )

    def _update_peers(self):
        for robot_id, robot in self.sim.robots.items():

            if not robot.active:
                continue

            for other_id, other in self.sim.robots.items():

                if other_id == robot_id:
                    continue

                if not other.active:
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

            if other_id == robot_id:
                continue

            if not robot.active:
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
                and task.status.value != "failed"
            ):
                return task

        return None

    def _all_tasks_complete(self):
        if not self.sim.tasks:
            return False

        return all(
            task.completed
            for task in self.sim.tasks.values()
        )

    def _check_tasks(self):
        for task in self.sim.tasks.values():

            if task.completed:
                continue

            if not task.assigned_robot:
                continue

            robot = self.sim.get_robot(
                task.assigned_robot
            )

            if not robot or not robot.active:
                continue

            cell = self.sim.grid.to_cell(
                robot.x,
                robot.y
            )

            if cell == task.destination:

                task.complete()

                self.metrics.tasks_completed += 1

                self.metrics.event(
                    "task_completed",
                    time=self.sim.time,
                    task_id=task.id,
                    robot_id=robot.id
                )

    def _check_safety(self):
        robots = [
            r
            for r in self.sim.robots.values()
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

                ra = self.engine.robot_specs[
                    a.id
                ].radius

                rb = self.engine.robot_specs[
                    b.id
                ].radius

                collision_distance = ra + rb

                if d <= collision_distance:

                    self.metrics.record_collision()

                    self.metrics.event(
                        "collision",
                        time=self.sim.time,
                        robot_a=a.id,
                        robot_b=b.id
                    )

                elif d <= collision_distance + 0.2:

                    self.metrics.record_near_miss()

                    self.metrics.event(
                        "near_miss",
                        time=self.sim.time,
                        robot_a=a.id,
                        robot_b=b.id
                    )