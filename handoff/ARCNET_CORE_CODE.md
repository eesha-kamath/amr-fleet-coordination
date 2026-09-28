# ARCNET CORE CODE SNAPSHOT


============================================================
FILE: arcnet_core\allocator.py
============================================================

from abc import ABC, abstractmethod


class TaskAllocator(ABC):

    @abstractmethod
    def assign(self, tasks, robots):
        pass


class ManualAllocator(TaskAllocator):

    def assign(self, tasks, robots):
        return tasks


class CentralAllocator(TaskAllocator):

    def assign(self, tasks, robots):
        available = [
            r
            for r in robots
            if robots[r].active
        ]

        for task in tasks:
            if task.assigned_robot:
                continue

            if not available:
                break

            task.assigned_robot = available[0]

        return tasks


class ContractNetAllocator(TaskAllocator):

    def assign(self, tasks, robots):
        available = [
            r
            for r in robots
            if robots[r].active
        ]

        for task in tasks:
            if task.assigned_robot:
                continue

            if not available:
                break

            best = available[0]
            best_battery = robots[best].battery

            for robot_id in available:
                if robots[robot_id].battery > best_battery:
                    best = robot_id
                    best_battery = robots[robot_id].battery

            task.assigned_robot = best

        return tasks


============================================================
FILE: arcnet_core\arbiter.py
============================================================

from .intent import RobotIntent


class ConflictArbiter:

    def decide(self, a: RobotIntent, b: RobotIntent):

        if a.wait_time != b.wait_time:
            return a if a.wait_time > b.wait_time else b

        if a.logical_clock != b.logical_clock:
            return a if a.logical_clock < b.logical_clock else b

        return a if a.id < b.id else b

    def compare(self, a: RobotIntent, b: RobotIntent):
        winner = self.decide(a, b)

        if winner.id == a.id:
            return 1

        return -1

    def highest_priority(self, intents):
        if not intents:
            return None

        winner = intents[0]

        for intent in intents[1:]:
            winner = self.decide(winner, intent)

        return winner


============================================================
FILE: arcnet_core\avoidance.py
============================================================

from dataclasses import dataclass
from math import sqrt, atan2


@dataclass
class AvoidanceResult:
    vx: float
    vy: float
    omega: float
    changed: bool


class LocalAvoidance:

    def __init__(self, time_horizon=2.0, margin=0.1):
        self.time_horizon = time_horizon
        self.margin = margin

    def avoid(
        self,
        x,
        y,
        vx,
        vy,
        peers
    ):
        rvx = vx
        rvy = vy
        changed = False

        for peer in peers:
            dx = peer["x"] - x
            dy = peer["y"] - y

            d = sqrt(dx * dx + dy * dy)

            if d <= 0:
                continue

            pvx = peer.get("vx", 0.0)
            pvy = peer.get("vy", 0.0)

            rel_vx = pvx - rvx
            rel_vy = pvy - rvy

            closing = dx * rel_vx + dy * rel_vy

            if closing >= 0:
                continue

            closing_speed = -closing / d

            if closing_speed <= 0:
                continue

            t = d / closing_speed

            if t > self.time_horizon:
                continue

            nx = -dy / d
            ny = dx / d

            cross = dx * rel_vy - dy * rel_vx

            if cross > 0:
                nx = -nx
                ny = -ny

            strength = (
                self.time_horizon - t
            ) / self.time_horizon

            shift = max(
                0.1,
                strength
            )

            rvx += nx * shift
            rvy += ny * shift

            changed = True

        speed = sqrt(
            rvx * rvx +
            rvy * rvy
        )

        omega = 0.0

        if speed > 0:
            omega = atan2(rvy, rvx)

        return AvoidanceResult(
            rvx,
            rvy,
            omega,
            changed
        )


============================================================
FILE: arcnet_core\baseline.py
============================================================

from math import sqrt


class StopAndWaitBaseline:

    def __init__(self, distance_threshold):
        self.distance_threshold = distance_threshold

    def command(
        self,
        robot,
        vx,
        vy,
        peers
    ):
        for peer in peers:

            dx = peer.x - robot.x
            dy = peer.y - robot.y

            d = sqrt(
                dx * dx +
                dy * dy
            )

            if d <= self.distance_threshold:
                return 0.0, 0.0

        return vx, vy


============================================================
FILE: arcnet_core\cell.py
============================================================

from dataclasses import dataclass


@dataclass(frozen=True)
class Cell:
    x: int
    y: int

    def multicast_group(self):
        return f"239.1.{self.y % 256}.{self.x % 256}"

    def neighbors(self):
        cells = []

        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                cells.append(
                    Cell(
                        self.x + dx,
                        self.y + dy
                    )
                )

        return cells

    def is_neighbor(self, other):
        return (
            abs(self.x - other.x) <= 1
            and abs(self.y - other.y) <= 1
        )


def cell_from_position(x, y, cell_size):
    return Cell(
        int(x / cell_size),
        int(y / cell_size)
    )


============================================================
FILE: arcnet_core\clock.py
============================================================

class LamportClock:
    def __init__(self):
        self.value = 0

    def tick(self):
        self.value += 1
        return self.value

    def receive(self, other):
        self.value = max(self.value, other) + 1
        return self.value


============================================================
FILE: arcnet_core\config.py
============================================================

from dataclasses import dataclass


@dataclass
class SafetyConfig:
    reaction_latency: float = 0.1
    margin: float = 0.1


@dataclass
class LivenessConfig:
    warning_timeout: float = 1.0
    failure_timeout: float = 3.0
    margin: float = 0.1


@dataclass
class NetworkConfig:
    cell_size: float = 10.0
    heartbeat_frequency: float = 1.0
    local_frequency: float = 10.0


@dataclass
class ArcnetConfig:
    safety: SafetyConfig
    liveness: LivenessConfig
    network: NetworkConfig


============================================================
FILE: arcnet_core\conflict.py
============================================================

from dataclasses import dataclass


@dataclass
class Conflict:
    robot_a: str
    robot_b: str
    position: tuple[int, int]
    time: int
    kind: str


class ConflictDetector:

    def detect(self, robot_id, path, peer_paths):
        conflicts = []

        for peer_id, peer_path in peer_paths.items():

            if peer_id == robot_id:
                continue

            n = min(len(path), len(peer_path))

            for t in range(n):

                if path[t] == peer_path[t]:
                    conflicts.append(
                        Conflict(
                            robot_id,
                            peer_id,
                            path[t],
                            t,
                            "vertex"
                        )
                    )

                if t > 0:
                    if (
                        path[t] == peer_path[t - 1]
                        and
                        path[t - 1] == peer_path[t]
                    ):
                        conflicts.append(
                            Conflict(
                                robot_id,
                                peer_id,
                                path[t],
                                t,
                                "edge"
                            )
                        )

        return conflicts

    def has_conflict(self, robot_id, path, peer_paths):
        return bool(
            self.detect(
                robot_id,
                path,
                peer_paths
            )
        )


============================================================
FILE: arcnet_core\deadlock.py
============================================================

class WaitForGraph:

    def __init__(self):
        self.waiting = {}

    def add_wait(self, robot_id, waiting_for):
        self.waiting[robot_id] = waiting_for

    def remove_wait(self, robot_id):
        self.waiting.pop(robot_id, None)

    def clear(self):
        self.waiting.clear()

    def has_deadlock(self):
        return self.get_cycle() is not None

    def get_cycle(self):
        for start in self.waiting:

            seen = set()
            cur = start

            while cur in self.waiting:

                if cur in seen:
                    cycle = []
                    node = cur

                    while True:
                        cycle.append(node)
                        node = self.waiting[node]

                        if node == cur:
                            break

                    return cycle

                seen.add(cur)
                cur = self.waiting[cur]

        return None

    def choose_victim(self, intents, arbiter):
        cycle = self.get_cycle()

        if not cycle:
            return None

        cycle_intents = [
            intent
            for intent in intents
            if intent.id in cycle
        ]

        if not cycle_intents:
            return None

        winner = arbiter.highest_priority(cycle_intents)

        if not winner:
            return None

        return next(
            intent
            for intent in cycle_intents
            if intent.id != winner.id
        )

    def dependencies(self):
        return dict(self.waiting)


============================================================
FILE: arcnet_core\decision.py
============================================================

from dataclasses import dataclass

from .motion import MotionCommand


@dataclass
class CoordinationDecision:
    robot_id: str
    path: list
    command: MotionCommand
    waiting_for: str | None = None
    conflict: object | None = None
    deadlock: bool = False
    victim: str | None = None
    replan: bool = False


============================================================
FILE: arcnet_core\engine.py
============================================================

from .planner import PathPlanner
from .arbiter import ConflictArbiter
from .conflict import ConflictDetector
from .deadlock import WaitForGraph
from .motion import MotionController
from .safety import get_safety_level, SafetyLevel
from .decision import CoordinationDecision
from .avoidance import LocalAvoidance


class ArcnetEngine:

    def __init__(
        self,
        robot_specs,
        grid,
        obstacles,
        safety_config
    ):
        self.robot_specs = robot_specs
        self.grid = grid
        self.obstacles = obstacles
        self.safety_config = safety_config

        self.planner = PathPlanner()
        self.arbiter = ConflictArbiter()
        self.detector = ConflictDetector()
        self.graph = WaitForGraph()
        self.motion = MotionController()
        self.avoidance = LocalAvoidance(
            margin=safety_config.margin
        )

        self.paths = {}
        self.intents = {}
        self.peer_states = {}

    def set_intent(self, intent):
        self.intents[intent.id] = intent

    def set_path(self, robot_id, path):
        self.paths[robot_id] = path

    def set_peer_state(
        self,
        robot_id,
        x,
        y,
        vx=0.0,
        vy=0.0
    ):
        self.peer_states[robot_id] = {
            "x": x,
            "y": y,
            "vx": vx,
            "vy": vy
        }

    def remove_robot(self, robot_id):
        self.paths.pop(robot_id, None)
        self.intents.pop(robot_id, None)
        self.peer_states.pop(robot_id, None)
        self.graph.remove_wait(robot_id)

    def plan_robot(
        self,
        robot_id,
        start,
        goal
    ):
        peer_paths = {
            rid: path
            for rid, path in self.paths.items()
            if rid != robot_id
        }

        path = self.planner.plan_predictive(
            start,
            goal,
            self.obstacles,
            self.grid,
            peer_paths
        )

        self.paths[robot_id] = path

        return path

    def detect_robot_conflicts(self, robot_id):
        path = self.paths.get(robot_id, [])

        peer_paths = {
            rid: path
            for rid, path in self.paths.items()
            if rid != robot_id
        }

        return self.detector.detect(
            robot_id,
            path,
            peer_paths
        )

    def resolve_conflicts(self, robot_id):
        conflicts = self.detect_robot_conflicts(robot_id)

        if not conflicts:
            self.graph.remove_wait(robot_id)
            return None

        conflict = conflicts[0]

        a = self.intents.get(robot_id)
        b = self.intents.get(conflict.robot_b)

        if not a or not b:
            self.graph.add_wait(
                robot_id,
                conflict.robot_b
            )
            return conflict.robot_b

        winner = self.arbiter.decide(a, b)

        if winner.id == robot_id:
            self.graph.remove_wait(robot_id)
            return None

        self.graph.add_wait(
            robot_id,
            conflict.robot_b
        )

        return conflict.robot_b

    def detect_deadlock(self):
        return self.graph.get_cycle()

    def resolve_deadlock(self):
        cycle = self.detect_deadlock()

        if not cycle:
            return None

        intents = [
            self.intents[rid]
            for rid in cycle
            if rid in self.intents
        ]

        victim = self.graph.choose_victim(
            intents,
            self.arbiter
        )

        if not victim:
            return None

        self.graph.remove_wait(victim.id)

        return victim.id

    def command_for_distance(
        self,
        robot_id,
        distance,
        vx,
        vy,
        omega=0.0
    ):
        robot = self.robot_specs[robot_id]

        level = get_safety_level(
            distance,
            robot,
            self.safety_config
        )

        if level == SafetyLevel.HARD_STOP:
            return level, self.motion.hard_stop()

        if level == SafetyLevel.CAUTION:
            peers = list(self.peer_states.values())

            result = self.avoidance.avoid(
                0.0,
                0.0,
                vx,
                vy,
                peers
            )

            return level, self.motion.normal(
                result.vx,
                result.vy,
                result.omega
            )

        return level, self.motion.normal(
            vx,
            vy,
            omega
        )

    def step(
        self,
        robot_id,
        start,
        goal,
        distance,
        vx,
        vy,
        omega=0.0
    ):
        path = self.plan_robot(
            robot_id,
            start,
            goal
        )

        waiting_for = self.resolve_conflicts(
            robot_id
        )

        deadlock = False
        victim = None
        replan = False

        if self.detect_deadlock():
            deadlock = True

            victim = self.resolve_deadlock()

            if victim == robot_id:
                replan = True
                waiting_for = None

        level, command = self.command_for_distance(
            robot_id,
            distance,
            vx,
            vy,
            omega
        )

        if waiting_for and level != SafetyLevel.HARD_STOP:
            command = self.motion.hard_stop()

        return CoordinationDecision(
            robot_id=robot_id,
            path=path,
            command=command,
            waiting_for=waiting_for,
            deadlock=deadlock,
            victim=victim,
            replan=replan
        )


============================================================
FILE: arcnet_core\events.py
============================================================

from dataclasses import dataclass


@dataclass
class Event:
    time: float
    kind: str
    robot_id: str | None = None
    task_id: str | None = None
    peer_id: str | None = None
    data: dict | None = None


class EventLog:

    def __init__(self):
        self.events = []

    def add(
        self,
        time,
        kind,
        robot_id=None,
        task_id=None,
        peer_id=None,
        data=None
    ):
        self.events.append(
            Event(
                time,
                kind,
                robot_id,
                task_id,
                peer_id,
                data or {}
            )
        )

    def all(self):
        return list(self.events)

    def clear(self):
        self.events.clear()


============================================================
FILE: arcnet_core\faults.py
============================================================

from dataclasses import dataclass


@dataclass
class Fault:
    kind: str
    target: str | None = None
    x: int | None = None
    y: int | None = None
    active: bool = True


class FaultManager:
    def __init__(self):
        self.faults = []

    def add(self, fault):
        self.faults.append(fault)

    def remove(self, fault):
        if fault in self.faults:
            self.faults.remove(fault)

    def clear(self):
        self.faults.clear()

    def kill_robot(self, robot_id):
        self.add(
            Fault(
                "robot_failure",
                target=robot_id
            )
        )

    def wifi_dead_zone(self, x, y):
        self.add(
            Fault(
                "wifi_dead_zone",
                x=x,
                y=y
            )
        )

    def blocked_cell(self, x, y):
        self.add(
            Fault(
                "blocked_cell",
                x=x,
                y=y
            )
        )

    def is_wifi_blocked(self, x, y):
        return any(
            f.active
            and f.kind == "wifi_dead_zone"
            and f.x == x
            and f.y == y
            for f in self.faults
        )

    def blocked_cells(self):
        return [
            (f.x, f.y)
            for f in self.faults
            if f.active
            and f.kind == "blocked_cell"
        ]

    def failed_robots(self):
        return [
            f.target
            for f in self.faults
            if f.active
            and f.kind == "robot_failure"
        ]

    def is_robot_failed(self, robot_id):
        return robot_id in self.failed_robots()


============================================================
FILE: arcnet_core\fleet.py
============================================================

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


============================================================
FILE: arcnet_core\geometry.py
============================================================

from dataclasses import dataclass
from math import sqrt


@dataclass
class Footprint:
    shape: str
    length: float
    width: float

    @classmethod
    def from_robot(cls, robot):
        return cls(
            robot.shape,
            robot.length,
            robot.width
        )

    @property
    def radius(self):
        if self.shape == "circle":
            return self.width / 2

        return sqrt(
            (self.length / 2) ** 2 +
            (self.width / 2) ** 2
        )

    def validate(self):
        if self.shape not in ("rectangle", "circle"):
            raise ValueError("Invalid footprint shape")

        if self.length <= 0 or self.width <= 0:
            raise ValueError(
                "Footprint dimensions must be positive"
            )


def distance(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]

    return sqrt(dx * dx + dy * dy)


def surface_gap(
    a,
    b,
    footprint_a,
    footprint_b
):
    return max(
        0.0,
        distance(a, b)
        - footprint_a.radius
        - footprint_b.radius
    )


============================================================
FILE: arcnet_core\grid.py
============================================================

from dataclasses import dataclass


@dataclass
class Grid:
    width: int
    height: int
    cell_size: float

    def validate(self):
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Grid dimensions must be positive")

        if self.cell_size <= 0:
            raise ValueError("Cell size must be positive")

    def to_cell(self, x, y):
        return int(x / self.cell_size), int(y / self.cell_size)

    def to_position(self, x, y):
        return (
            (x + 0.5) * self.cell_size,
            (y + 0.5) * self.cell_size
        )


============================================================
FILE: arcnet_core\intent.py
============================================================

from dataclasses import dataclass


@dataclass
class RobotIntent:
    id: str
    logical_clock: int
    wait_time: float
    x: float
    y: float
    vx: float
    vy: float
    theta: float


============================================================
FILE: arcnet_core\message.py
============================================================

from dataclasses import dataclass


@dataclass
class RobotMessage:
    sender_id: str
    logical_clock: int
    timestamp: float
    x: float
    y: float
    vx: float
    vy: float
    theta: float
    status: int


============================================================
FILE: arcnet_core\metrics.py
============================================================

from dataclasses import dataclass, field


@dataclass
class Metrics:
    completion_time: float = 0.0
    collisions: int = 0
    near_misses: int = 0
    stops: int = 0
    replans: int = 0
    deadlocks_detected: int = 0
    deadlocks_resolved: int = 0
    tasks_completed: int = 0
    tasks_failed: int = 0
    events: list = field(default_factory=list)

    def record_collision(self):
        self.collisions += 1

    def record_near_miss(self):
        self.near_misses += 1

    def record_stop(self):
        self.stops += 1

    def record_replan(self):
        self.replans += 1

    def record_deadlock(self):
        self.deadlocks_detected += 1

    def record_deadlock_resolution(self):
        self.deadlocks_resolved += 1

    def event(self, kind, **data):
        self.events.append({
            "time": data.pop("time", 0.0),
            "kind": kind,
            **data
        })

    def as_dict(self):
        return {
            "completion_time": self.completion_time,
            "collisions": self.collisions,
            "near_misses": self.near_misses,
            "stops": self.stops,
            "replans": self.replans,
            "deadlocks_detected": self.deadlocks_detected,
            "deadlocks_resolved": self.deadlocks_resolved,
            "tasks_completed": self.tasks_completed,
            "tasks_failed": self.tasks_failed
        }


============================================================
FILE: arcnet_core\motion.py
============================================================

from dataclasses import dataclass

from .safety import SafetyLevel


@dataclass
class MotionCommand:
    vx: float
    vy: float
    omega: float
    stop: bool = False


class MotionController:

    def normal(self, vx, vy, omega=0.0):
        return MotionCommand(
            vx,
            vy,
            omega,
            False
        )

    def caution(self, vx, vy, omega=0.0):
        return MotionCommand(
            vx * 0.5,
            vy * 0.5,
            omega,
            False
        )

    def hard_stop(self):
        return MotionCommand(
            0.0,
            0.0,
            0.0,
            True
        )

    def command(self, level, vx, vy, omega=0.0):
        if level == SafetyLevel.HARD_STOP:
            return self.hard_stop()

        if level == SafetyLevel.CAUTION:
            return self.caution(
                vx,
                vy,
                omega
            )

        return self.normal(
            vx,
            vy,
            omega
        )


def safety_command(level):
    controller = MotionController()

    return controller.command(
        level,
        0.0,
        0.0
    )


============================================================
FILE: arcnet_core\obstacle.py
============================================================

from dataclasses import dataclass


@dataclass(frozen=True)
class Obstacle:
    x: int
    y: int
    temporary: bool = False


def is_blocked(obstacles, x, y):
    return any(o.x == x and o.y == y for o in obstacles)


============================================================
FILE: arcnet_core\peer.py
============================================================

from dataclasses import dataclass
from enum import Enum
from .config import LivenessConfig


class PeerStatus(Enum):
    UNKNOWN = "unknown"
    ACTIVE = "active"
    LEFT_NEIGHBORHOOD = "left_neighborhood"
    SILENT_WARNING = "silent_warning"
    OUT_OF_RANGE = "out_of_range"
    PRESUMED_FAILED = "presumed_failed"


@dataclass
class Peer:
    id: str
    x: float
    y: float
    vx: float
    vy: float
    theta: float
    last_local_broadcast: float
    last_global_heartbeat: float
    max_speed: float = 0.0
    status: PeerStatus = PeerStatus.UNKNOWN


class PeerManager:
    def __init__(self, config):
        self.config = config
        self.peers = {}

    def add(self, peer):
        self.peers[peer.id] = peer

    def update_state(
        self,
        robot_id,
        x,
        y,
        vx,
        vy,
        theta,
        now,
        local=True,
        global_heartbeat=True
    ):
        peer = self.peers.get(robot_id)

        if not peer:
            peer = Peer(
                robot_id,
                x,
                y,
                vx,
                vy,
                theta,
                now,
                now,
                0.0,
                PeerStatus.ACTIVE
            )
            self.peers[robot_id] = peer
        else:
            peer.x = x
            peer.y = y
            peer.vx = vx
            peer.vy = vy
            peer.theta = theta

            if local:
                peer.last_local_broadcast = now

            if global_heartbeat:
                peer.last_global_heartbeat = now

            peer.status = PeerStatus.ACTIVE

        return peer

    def get(self, robot_id):
        return self.peers.get(robot_id)

    def remove(self, robot_id):
        self.peers.pop(robot_id, None)

    def uncertainty_radius(self, robot_id, now):
        peer = self.peers.get(robot_id)

        if not peer:
            return 0.0

        silence = max(
            0.0,
            now - peer.last_local_broadcast
        )

        return silence * peer.max_speed + self.config.margin

    def update(
        self,
        robot_id,
        now,
        local_visible,
        global_visible
    ):
        peer = self.peers.get(robot_id)

        if not peer:
            return None

        local_age = now - peer.last_local_broadcast
        global_age = now - peer.last_global_heartbeat

        if local_visible:
            peer.status = PeerStatus.ACTIVE
        elif global_visible:
            peer.status = PeerStatus.LEFT_NEIGHBORHOOD
        elif (
            local_age > self.config.failure_timeout
            and global_age > self.config.failure_timeout
        ):
            peer.status = PeerStatus.PRESUMED_FAILED
        elif local_age > self.config.warning_timeout:
            peer.status = PeerStatus.SILENT_WARNING

        return peer.status

    def update_all(self, now, local_visible):
        for robot_id in self.peers:
            self.update(
                robot_id,
                now,
                robot_id in local_visible,
                True
            )

    def active_peers(self):
        return [
            p for p in self.peers.values()
            if p.status == PeerStatus.ACTIVE
        ]

    def failed_peers(self):
        return [
            p for p in self.peers.values()
            if p.status == PeerStatus.PRESUMED_FAILED
        ]


============================================================
FILE: arcnet_core\planner.py
============================================================

from heapq import heappush, heappop


class PathPlanner:

    def plan(self, start, goal, obstacles, grid):
        return self._astar(start, goal, obstacles, grid)

    def plan_predictive(
        self,
        start,
        goal,
        obstacles,
        grid,
        peer_paths=None,
        start_time=0
    ):
        peer_paths = peer_paths or {}

        q = []
        first = (start[0], start[1], start_time)

        heappush(q, (0, first))

        came_from = {}
        cost = {first: 0}

        while q:
            _, cur = heappop(q)

            x, y, t = cur

            if (x, y) == goal:
                return self._build_time_path(came_from, cur)

            nt = t + 1

            moves = (
                (x + 1, y),
                (x - 1, y),
                (x, y + 1),
                (x, y - 1),
                (x, y)
            )

            for nx, ny in moves:

                if nx < 0 or ny < 0:
                    continue

                if nx >= grid.width or ny >= grid.height:
                    continue

                if self._blocked(obstacles, nx, ny):
                    continue

                if self._peer_conflict(
                    (nx, ny),
                    (x, y),
                    nt,
                    peer_paths
                ):
                    continue

                nxt = (nx, ny, nt)
                nc = cost[cur] + 1

                if nxt not in cost or nc < cost[nxt]:
                    cost[nxt] = nc

                    h = abs(nx - goal[0]) + abs(ny - goal[1])

                    heappush(
                        q,
                        (nc + h, nxt)
                    )

                    came_from[nxt] = cur

        return []

    def _astar(self, start, goal, obstacles, grid):
        q = []
        heappush(q, (0, start))

        came_from = {}
        cost = {start: 0}

        while q:
            _, cur = heappop(q)

            if cur == goal:
                return self._build_path(came_from, cur)

            x, y = cur

            for nx, ny in (
                (x + 1, y),
                (x - 1, y),
                (x, y + 1),
                (x, y - 1)
            ):
                if nx < 0 or ny < 0:
                    continue

                if nx >= grid.width or ny >= grid.height:
                    continue

                if self._blocked(obstacles, nx, ny):
                    continue

                nc = cost[cur] + 1

                if (nx, ny) not in cost or nc < cost[(nx, ny)]:
                    cost[(nx, ny)] = nc

                    h = abs(nx - goal[0]) + abs(ny - goal[1])

                    heappush(
                        q,
                        (nc + h, (nx, ny))
                    )

                    came_from[(nx, ny)] = cur

        return []

    def _peer_conflict(
        self,
        position,
        previous,
        timestamp,
        peer_paths
    ):
        for path in peer_paths.values():

            if not path:
                continue

            if timestamp < len(path):
                peer_position = path[timestamp]

                if position == peer_position:
                    return True

            if timestamp > 0 and timestamp - 1 < len(path):
                peer_position = path[timestamp - 1]

                if position == peer_position and previous == peer_position:
                    return True

        return False

    def _blocked(self, obstacles, x, y):
        return any(
            o.x == x and o.y == y
            for o in obstacles
        )

    def _build_path(self, came_from, cur):
        path = [cur]

        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)

        path.reverse()
        return path

    def _build_time_path(self, came_from, cur):
        path = []

        while True:
            path.append((cur[0], cur[1]))

            if cur not in came_from:
                break

            cur = came_from[cur]

        path.reverse()
        return path


============================================================
FILE: arcnet_core\robot.py
============================================================

from dataclasses import dataclass
from math import sqrt


@dataclass
class RobotSpec:
    id: str
    shape: str
    length: float
    width: float
    height: float
    mass: float
    wheel_track: float
    wheel_radius: float
    max_speed: float
    acceleration: float
    braking: float
    safety_margin: float

    def validate(self):
        if not self.id:
            raise ValueError("Robot ID is required")

        if self.shape not in ("rectangle", "circle"):
            raise ValueError("Invalid robot shape")

        if self.length <= 0 or self.width <= 0 or self.height <= 0:
            raise ValueError("Robot dimensions must be positive")

        if self.mass <= 0:
            raise ValueError("Mass must be positive")

        if self.wheel_track <= 0 or self.wheel_radius <= 0:
            raise ValueError("Wheel dimensions must be positive")

        if self.max_speed <= 0:
            raise ValueError("Max speed must be positive")

        if self.acceleration <= 0 or self.braking <= 0:
            raise ValueError("Acceleration and braking must be positive")

        if self.safety_margin < 0:
            raise ValueError("Safety margin cannot be negative")

    @property
    def radius(self):
        if self.shape == "circle":
            return self.width / 2

        return sqrt((self.length / 2) ** 2 + (self.width / 2) ** 2)

    @property
    def braking_distance(self):
        return self.max_speed ** 2 / (2 * self.braking)
    


============================================================
FILE: arcnet_core\safety.py
============================================================

from enum import Enum
from .robot import RobotSpec
from .geometry import Footprint
from .config import SafetyConfig


class SafetyLevel(Enum):
    NORMAL = "normal"
    CAUTION = "caution"
    HARD_STOP = "hard_stop"


def hard_stop_distance(robot: RobotSpec, config: SafetyConfig):
    footprint = Footprint.from_robot(robot)

    return (
        footprint.radius
        + robot.braking_distance
        + robot.max_speed * config.reaction_latency
        + config.margin
    )


def get_safety_level(distance, robot: RobotSpec, config: SafetyConfig):
    d = hard_stop_distance(robot, config)

    if distance <= d:
        return SafetyLevel.HARD_STOP

    if distance <= d * 2:
        return SafetyLevel.CAUTION

    return SafetyLevel.NORMAL


============================================================
FILE: arcnet_core\scenario.py
============================================================

from dataclasses import dataclass, field


@dataclass
class Scenario:
    id: str
    name: str
    description: str
    obstacles: list = field(default_factory=list)
    robots: list = field(default_factory=list)
    tasks: list = field(default_factory=list)


class ScenarioRunner:
    def __init__(self, simulator, engine):
        self.sim = simulator
        self.engine = engine

    def load(self, scenario):
        if isinstance(scenario, Scenario):
            data = {
                "obstacles": scenario.obstacles,
                "robots": scenario.robots,
                "tasks": scenario.tasks
            }
        else:
            data = scenario

        self.sim.obstacles.clear()
        self.sim.obstacles.extend(data["obstacles"])

        self.sim.robots.clear()
        self.sim.tasks.clear()

        for robot in data["robots"]:
            self.sim.add_robot(
                robot["id"],
                robot["x"],
                robot["y"],
                robot.get("theta", 0.0),
                robot.get("battery", 100.0)
            )

        for task in data["tasks"]:
            self.sim.add_task(task)

    def kill_robot(self, robot_id):
        self.sim.kill_robot(robot_id)

    def block_cell(self, obstacle):
        if obstacle not in self.sim.obstacles:
            self.sim.obstacles.append(obstacle)

    def unblock_cell(self, obstacle):
        if obstacle in self.sim.obstacles:
            self.sim.obstacles.remove(obstacle)


============================================================
FILE: arcnet_core\scenarios.py
============================================================

from .obstacle import Obstacle
from .task import Task


class ScenarioLibrary:

    def head_on(self):
        return {
            "id": "head_on",
            "name": "Head-on",
            "description": "Two robots approach each other in a shared aisle.",
            "robots": [
                {
                    "id": "R1",
                    "x": 1.5,
                    "y": 2.5
                },
                {
                    "id": "R2",
                    "x": 8.5,
                    "y": 2.5
                }
            ],
            "tasks": [
                Task(
                    "T001",
                    (1, 2),
                    (8, 2),
                    "R1"
                ),
                Task(
                    "T002",
                    (8, 2),
                    (1, 2),
                    "R2"
                )
            ],
            "obstacles": []
        }

    def t_junction(self):
        return {
            "id": "t_junction",
            "name": "T-Junction",
            "description": "Two robots approach a shared junction.",
            "robots": [
                {
                    "id": "R1",
                    "x": 2.5,
                    "y": 5.5
                },
                {
                    "id": "R2",
                    "x": 5.5,
                    "y": 2.5
                }
            ],
            "tasks": [
                Task(
                    "T001",
                    (2, 5),
                    (5, 5),
                    "R1"
                ),
                Task(
                    "T002",
                    (5, 2),
                    (5, 6),
                    "R2"
                )
            ],
            "obstacles": [
                Obstacle(3, 4),
                Obstacle(4, 4)
            ]
        }

    def narrow_aisle(self):
        return {
            "id": "narrow_aisle",
            "name": "Narrow Aisle",
            "description": "Robots share a constrained aisle.",
            "robots": [
                {
                    "id": "R1",
                    "x": 1.5,
                    "y": 4.5
                },
                {
                    "id": "R2",
                    "x": 8.5,
                    "y": 4.5
                }
            ],
            "tasks": [
                Task(
                    "T001",
                    (1, 4),
                    (8, 4),
                    "R1"
                ),
                Task(
                    "T002",
                    (8, 4),
                    (1, 4),
                    "R2"
                )
            ],
            "obstacles": []
        }

    def three_way_choke(self):
        return {
            "id": "three_way_choke",
            "name": "3-Way Choke",
            "description": "Three robots converge on one area.",
            "robots": [
                {
                    "id": "R1",
                    "x": 1.5,
                    "y": 5.5
                },
                {
                    "id": "R2",
                    "x": 5.5,
                    "y": 1.5
                },
                {
                    "id": "R3",
                    "x": 9.5,
                    "y": 5.5
                }
            ],
            "tasks": [
                Task(
                    "T001",
                    (1, 5),
                    (5, 5),
                    "R1"
                ),
                Task(
                    "T002",
                    (5, 1),
                    (5, 5),
                    "R2"
                ),
                Task(
                    "T003",
                    (9, 5),
                    (5, 5),
                    "R3"
                )
            ],
            "obstacles": []
        }

    def four_way_choke(self):
        return {
            "id": "four_way_choke",
            "name": "4-Way Choke",
            "description": "Four robots approach a shared intersection.",
            "robots": [
                {"id": "R1", "x": 1.5, "y": 5.5},
                {"id": "R2", "x": 5.5, "y": 1.5},
                {"id": "R3", "x": 9.5, "y": 5.5},
                {"id": "R4", "x": 5.5, "y": 9.5}
            ],
            "tasks": [
                Task("T001", (1, 5), (9, 5), "R1"),
                Task("T002", (5, 1), (5, 9), "R2"),
                Task("T003", (9, 5), (1, 5), "R3"),
                Task("T004", (5, 9), (5, 1), "R4")
            ],
            "obstacles": []
        }

    def blocked_aisle(self):
        return {
            "id": "blocked_aisle",
            "name": "Blocked Aisle",
            "description": "A route becomes temporarily blocked.",
            "robots": [
                {
                    "id": "R1",
                    "x": 1.5,
                    "y": 2.5
                }
            ],
            "tasks": [
                Task(
                    "T001",
                    (1, 2),
                    (8, 2),
                    "R1"
                )
            ],
            "obstacles": [
                Obstacle(
                    4,
                    2,
                    True
                )
            ]
        }

    def robot_failure(self):
        return {
            "id": "robot_failure",
            "name": "Robot Failure",
            "description": "One robot fails during fleet operation.",
            "robots": [
                {
                    "id": "R1",
                    "x": 2.5,
                    "y": 2.5
                },
                {
                    "id": "R2",
                    "x": 7.5,
                    "y": 2.5
                }
            ],
            "tasks": [
                Task(
                    "T001",
                    (2, 2),
                    (8, 2),
                    "R1"
                ),
                Task(
                    "T002",
                    (7, 2),
                    (2, 2),
                    "R2"
                )
            ],
            "obstacles": []
        }

    def wifi_dead_zone(self):
        return {
            "id": "wifi_dead_zone",
            "name": "Wi-Fi Dead Zone",
            "description": "A spatial region temporarily loses communication.",
            "robots": [
                {
                    "id": "R1",
                    "x": 1.5,
                    "y": 5.5
                },
                {
                    "id": "R2",
                    "x": 8.5,
                    "y": 5.5
                }
            ],
            "tasks": [
                Task(
                    "T001",
                    (1, 5),
                    (8, 5),
                    "R1"
                ),
                Task(
                    "T002",
                    (8, 5),
                    (1, 5),
                    "R2"
                )
            ],
            "obstacles": []
        }

    def all(self):
        return {
            "head_on": self.head_on,
            "t_junction": self.t_junction,
            "narrow_aisle": self.narrow_aisle,
            "three_way_choke": self.three_way_choke,
            "four_way_choke": self.four_way_choke,
            "blocked_aisle": self.blocked_aisle,
            "robot_failure": self.robot_failure,
            "wifi_dead_zone": self.wifi_dead_zone
        }

    def get(self, scenario_id):
        scenarios = self.all()

        if scenario_id not in scenarios:
            raise ValueError(
                f"Unknown scenario: {scenario_id}"
            )

        return scenarios[scenario_id]()


============================================================
FILE: arcnet_core\simulator.py
============================================================

from dataclasses import dataclass
from math import sqrt
from .state import RobotState


@dataclass
class SimConfig:
    dt: float = 0.1


class WarehouseSimulator:
    def __init__(
        self,
        grid,
        obstacles,
        robot_specs,
        config=None
    ):
        self.grid = grid
        self.obstacles = obstacles
        self.robot_specs = robot_specs
        self.config = config or SimConfig()
        self.robots = {}
        self.tasks = {}
        self.time = 0.0

    def add_robot(
        self,
        robot_id,
        x,
        y,
        theta=0.0,
        battery=100.0
    ):
        self.robots[robot_id] = RobotState(
            robot_id,
            x,
            y,
            theta,
            0.0,
            0.0,
            battery
        )

    def add_task(self, task):
        self.tasks[task.id] = task

    def get_robot(self, robot_id):
        return self.robots.get(robot_id)

    def step(self, commands):
        dt = self.config.dt

        for robot_id, command in commands.items():
            robot = self.robots.get(robot_id)

            if not robot or not robot.active:
                continue

            robot.vx = command.vx
            robot.vy = command.vy

            if command.stop:
                robot.vx = 0.0
                robot.vy = 0.0

            robot.x += robot.vx * dt
            robot.y += robot.vy * dt

            if robot.vx != 0 or robot.vy != 0:
                robot.theta = self._heading(
                    robot.vx,
                    robot.vy
                )

            robot.battery = max(
                0.0,
                robot.battery - 0.01
            )

        self.time += dt

    def stop_robot(self, robot_id):
        robot = self.robots.get(robot_id)

        if robot:
            robot.vx = 0.0
            robot.vy = 0.0

    def kill_robot(self, robot_id):
        robot = self.robots.get(robot_id)

        if robot:
            robot.active = False
            robot.vx = 0.0
            robot.vy = 0.0

    def complete_task(self, task_id):
        task = self.tasks.get(task_id)

        if task:
            task.complete()

    def _heading(self, vx, vy):
        from math import atan2
        return atan2(vy, vx)

    def distance(self, a, b):
        dx = a.x - b.x
        dy = a.y - b.y
        return sqrt(dx * dx + dy * dy)


============================================================
FILE: arcnet_core\spatial.py
============================================================

from .cell import Cell, cell_from_position


class SpatialManager:

    def __init__(self, cell_size):
        self.cell_size = cell_size
        self.current_cells = {}

    def get_cell(self, x, y):
        return cell_from_position(
            x,
            y,
            self.cell_size
        )

    def set_robot_position(self, robot_id, x, y):
        old = self.current_cells.get(robot_id)
        new = self.get_cell(x, y)

        self.current_cells[robot_id] = new

        if old is None:
            return {
                "event": "CELL_ENTER",
                "robot_id": robot_id,
                "cell": new
            }

        if old == new:
            return None

        return {
            "event": "CELL_CHANGE",
            "robot_id": robot_id,
            "old_cell": old,
            "new_cell": new
        }

    def remove_robot(self, robot_id):
        self.current_cells.pop(robot_id, None)

    def get_cell_for_robot(self, robot_id):
        return self.current_cells.get(robot_id)

    def get_subscriptions(self, robot_id):
        cell = self.current_cells.get(robot_id)

        if not cell:
            return []

        return cell.neighbors()


============================================================
FILE: arcnet_core\state.py
============================================================

from dataclasses import dataclass


@dataclass
class RobotState:
    id: str
    x: float
    y: float
    theta: float
    vx: float
    vy: float
    battery: float
    task_id: str | None = None
    active: bool = True


============================================================
FILE: arcnet_core\strategy.py
============================================================

from abc import ABC, abstractmethod


class CoordinationStrategy(ABC):

    @abstractmethod
    def plan(self, state, peers):
        pass

    @abstractmethod
    def decide_motion(self, state, peers):
        pass


class StopAndWait(CoordinationStrategy):

    def plan(self, state, peers):
        return []

    def decide_motion(self, state, peers):
        if peers:
            return {
                "vx": 0.0,
                "vy": 0.0,
                "stop": True
            }

        return {
            "vx": state.get("vx", 0.0),
            "vy": state.get("vy", 0.0),
            "stop": False
        }


class PriorityOnly(CoordinationStrategy):

    def plan(self, state, peers):
        return state.get("path", [])

    def decide_motion(self, state, peers):
        if not peers:
            return {
                "vx": state.get("vx", 0.0),
                "vy": state.get("vy", 0.0),
                "stop": False
            }

        my_id = state.get("id", "")

        for peer in peers:
            if peer.get("id", "") < my_id:
                return {
                    "vx": 0.0,
                    "vy": 0.0,
                    "stop": True
                }

        return {
            "vx": state.get("vx", 0.0),
            "vy": state.get("vy", 0.0),
            "stop": False
        }


class ArcnetFull(CoordinationStrategy):

    def plan(self, state, peers):
        return state.get("path", [])

    def decide_motion(self, state, peers):
        return {
            "vx": state.get("vx", 0.0),
            "vy": state.get("vy", 0.0),
            "stop": state.get("stop", False)
        }


STRATEGIES = {
    "stop_and_wait": StopAndWait,
    "priority_only": PriorityOnly,
    "arcnet": ArcnetFull
}


============================================================
FILE: arcnet_core\task.py
============================================================

from dataclasses import dataclass
from enum import Enum


class TaskStatus(Enum):
    PENDING = "pending"
    ACTIVE = "active"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class Task:
    id: str
    start: tuple[int, int]
    destination: tuple[int, int]
    assigned_robot: str | None = None
    completed: bool = False
    status: TaskStatus = TaskStatus.PENDING

    def activate(self):
        if self.status == TaskStatus.PENDING:
            self.status = TaskStatus.ACTIVE

    def complete(self):
        self.completed = True
        self.status = TaskStatus.COMPLETED

    def fail(self):
        self.status = TaskStatus.FAILED
        self.completed = False

    def release(self):
        self.assigned_robot = None
        self.status = TaskStatus.PENDING
        self.completed = False


============================================================
FILE: arcnet_core\time.py
============================================================

from abc import ABC, abstractmethod
import time


class Clock(ABC):
    @abstractmethod
    def now(self):
        pass


class WallClock(Clock):
    def now(self):
        return time.time()


============================================================
FILE: arcnet_core\transport.py
============================================================

from abc import ABC, abstractmethod


class Transport(ABC):
    @abstractmethod
    def send(self, message):
        pass

    @abstractmethod
    def receive(self):
        pass


class InMemoryTransport(Transport):
    def __init__(self):
        self.messages = []

    def send(self, message):
        self.messages.append(message)

    def receive(self):
        messages = self.messages[:]
        self.messages.clear()
        return messages


============================================================
FILE: arcnet_core\validation.py
============================================================

from dataclasses import dataclass
import csv


@dataclass
class ValidationResult:
    scenario: str
    strategy: str
    completion_time: float
    collisions: int
    near_misses: int
    stops: int
    replans: int
    deadlocks: int
    deadlocks_resolved: int
    tasks_completed: int
    tasks_failed: int


class ValidationRunner:

    def __init__(self, factory):
        self.factory = factory

    def run(self, scenario_id, strategy):
        simulator, engine, runner = self.factory(
            scenario_id,
            strategy
        )

        metrics = runner.run()

        return ValidationResult(
            scenario=scenario_id,
            strategy=strategy,
            completion_time=metrics.completion_time,
            collisions=metrics.collisions,
            near_misses=metrics.near_misses,
            stops=metrics.stops,
            replans=metrics.replans,
            deadlocks=metrics.deadlocks_detected,
            deadlocks_resolved=metrics.deadlocks_resolved,
            tasks_completed=metrics.tasks_completed,
            tasks_failed=metrics.tasks_failed
        )

    def compare(self, scenario_id):
        return [
            self.run(
                scenario_id,
                "stop_and_wait"
            ),
            self.run(
                scenario_id,
                "arcnet"
            )
        ]

    def run_many(self, scenario_ids):
        results = []

        for scenario_id in scenario_ids:
            results.extend(
                self.compare(scenario_id)
            )

        return results


def write_results(results, path):
    if not results:
        return

    fields = [
        "scenario",
        "strategy",
        "completion_time",
        "collisions",
        "near_misses",
        "stops",
        "replans",
        "deadlocks",
        "deadlocks_resolved",
        "tasks_completed",
        "tasks_failed"
    ]

    with open(
        path,
        "w",
        newline="",
        encoding="utf-8"
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=fields
        )

        writer.writeheader()

        for result in results:
            writer.writerow({
                field: getattr(
                    result,
                    field
                )
                for field in fields
            })


============================================================
FILE: arcnet_core\__init__.py
============================================================

from .robot import RobotSpec
from .state import RobotState
from .geometry import Footprint, surface_gap
from .safety import SafetyLevel
from .config import SafetyConfig, LivenessConfig, NetworkConfig, ArcnetConfig
from .message import RobotMessage
from .clock import LamportClock
from .peer import Peer, PeerStatus, PeerManager
from .cell import Cell
from .grid import Grid
from .obstacle import Obstacle
from .planner import PathPlanner
from .arbiter import ConflictArbiter
from .intent import RobotIntent
from .deadlock import WaitForGraph
from .motion import MotionCommand, MotionController
from .task import Task, TaskStatus
from .allocator import TaskAllocator, ManualAllocator, CentralAllocator, ContractNetAllocator
from .strategy import CoordinationStrategy
from .transport import Transport
from .time import Clock
from .metrics import Metrics
from .conflict import Conflict, ConflictDetector
from .decision import CoordinationDecision
from .engine import ArcnetEngine
from .spatial import SpatialManager
from .avoidance import LocalAvoidance, AvoidanceResult
from .simulator import WarehouseSimulator, SimConfig
from .baseline import StopAndWaitBaseline
from .fleet import FleetRunner
from .scenario import Scenario, ScenarioRunner
from .faults import Fault, FaultManager
from .scenarios import ScenarioLibrary
from .validation import ValidationResult, ValidationRunner, write_results
from .events import Event, EventLog
