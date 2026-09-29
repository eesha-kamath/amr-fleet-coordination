"""Coordination runtime: one tick function shared by the live dashboard and
the validation runner.

Strategies
----------
arcnet         predictive space-time planning with reservations, Lamport +
               aging priority, priority-aware caution, universal hard stop,
               wait-for-graph deadlock recovery.
stop_and_wait  static shortest path, stop when a robot is ahead inside a fixed
               distance, wait until clear. Fixed id right of way, and a
               timeout reroute so the baseline is not trivially deadlocked.

Both strategies run on the same simulator, the same robots, the same tasks
and are graded by the same independent CollisionJudge.
"""
from dataclasses import dataclass
from math import sqrt, atan2

from .arbiter import ConflictArbiter
from .clock import LamportClock
from .config import SafetyConfig, LivenessConfig
from .deadlock import WaitForGraph
from .geometry import Footprint, footprint_polygon, polygon_gap
from .intent import RobotIntent
from .judge import CollisionJudge
from .metrics import Metrics
from .obstacle import Obstacle
from .planner import PathPlanner
from .safety import SafetyLevel
from .simulator import WarehouseSimulator
from .spacetime import SpaceTimePlanner, ReservationTable
from .task import TaskStatus


@dataclass
class RuntimeConfig:
    strategy: str = "arcnet"
    cruise_speed: float = 0.8
    aging_step: float = 4.0
    replan_period: float = 0.5
    standoff_time: float = 2.0
    near_miss_gap: float = 0.10
    hard_gap: float = 0.05
    caution_band: float = 0.3
    caution_speed_factor: float = 0.5
    baseline_stop_gap: float = 1.0
    baseline_wait_timeout: float = 8.0
    idle_clear_steps: int = 6
    plan_fail_limit: float = 45.0
    max_sim_time: float = 300.0


class Agent:

    def __init__(self, robot_id, spec, cell):
        self.id = robot_id
        self.spec = spec
        self.footprint = Footprint.from_robot(spec)
        self.clock = LamportClock()
        self.req_clock = 0
        self.at = cell
        self.target = None
        self.plan = [cell]
        self.history = [cell]
        self.speed = 0.0
        self.direction = (1.0, 0.0)
        self.wait_time = 0.0
        self.stalled = 0.0
        self.no_plan_time = 0.0
        self.mode = "idle"
        self.safety = SafetyLevel.NORMAL
        self.blocker = None
        self.waiting_for = None
        self.reason = ""
        self.next_decision = 0.0
        self.was_stopped = False
        self.task_id = None
        self.reroute_pending = False
        self.silent = False
        self.known_cell = cell
        self.known_pose = None   # (x, y, vx, vy, theta) at last broadcast
        self.silence_since = None
        self.hold_of = None


class CoordinationRuntime:

    def __init__(
        self,
        grid,
        specs,
        obstacles=None,
        config=None,
        safety_config=None
    ):
        self.grid = grid
        self.specs = specs
        self.config = config or RuntimeConfig()
        self.safety_config = safety_config or SafetyConfig()
        self.liveness_config = LivenessConfig()
        self.obstacles = obstacles if obstacles is not None else []

        self.sim = WarehouseSimulator(grid, self.obstacles, specs)
        self.judge = CollisionJudge(
            self.sim,
            specs,
            self.config.near_miss_gap
        )

        self.arbiter = ConflictArbiter(
            aging_step=self.config.aging_step
        )
        self.st_planner = SpaceTimePlanner()
        self.static_planner = PathPlanner()
        self.graph = WaitForGraph()
        self.metrics = Metrics()

        self.agents = {}
        self.events = []
        self.phase = {}
        self._log_seen = {}
        self._cycle_active = False
        self._finished_at = None
        self.wifi_zones = set()

    # ------------------------------------------------------------------
    # construction
    # ------------------------------------------------------------------
    @property
    def time(self):
        return self.sim.time

    @property
    def tasks(self):
        return self.sim.tasks

    @property
    def robots(self):
        return self.sim.robots

    def cell_occupied_by_robot(self, x, y):
        """True if any active robot's footprint currently overlaps (x, y)."""
        cs = self.grid.cell_size
        square = [
            (x * cs, y * cs),
            ((x + 1) * cs, y * cs),
            ((x + 1) * cs, (y + 1) * cs),
            (x * cs, (y + 1) * cs)
        ]

        for agent in self.agents.values():
            if agent.mode == "failed":
                continue

            robot = self.sim.robots[agent.id]
            poly = footprint_polygon(
                robot.x, robot.y, robot.theta, agent.footprint
            )

            if polygon_gap(poly, square) <= 0.0:
                return agent.id

        return None

    def add_robot(self, robot_id, x, y, theta=0.0, battery=100.0):
        cell = self.grid.to_cell(x, y)
        cx, cy = self.grid.to_position(*cell)

        self.sim.add_robot(robot_id, cx, cy, theta, battery)
        self.agents[robot_id] = Agent(
            robot_id,
            self.specs[robot_id],
            cell
        )

    def add_task(self, task):
        self.sim.add_task(task)

    def assign_task(self, task_id, robot_id):
        task = self.sim.tasks.get(task_id)

        if not task or robot_id not in self.agents:
            return False

        task.assigned_robot = robot_id
        task.status = TaskStatus.PENDING
        task.completed = False
        self.phase.pop(task_id, None)
        self._log(
            "TASK",
            robot_id,
            f"{task_id} assigned to {robot_id}"
        )
        return True

    def block_cell(self, x, y):
        occupant = self.cell_occupied_by_robot(x, y)

        if occupant:
            raise ValueError(
                f"Cannot block ({x},{y}): {occupant} is currently there"
            )

        obstacle = Obstacle(x, y, True)

        if not any(o.x == x and o.y == y for o in self.obstacles):
            self.obstacles.append(obstacle)
            self._log("FAULT", "SYSTEM", f"Cell ({x},{y}) blocked")

    def unblock_cell(self, x, y):
        before = len(self.obstacles)
        self.obstacles[:] = [
            o for o in self.obstacles
            if not (o.x == x and o.y == y and o.temporary)
        ]

        if len(self.obstacles) != before:
            self._log("FAULT", "SYSTEM", f"Cell ({x},{y}) cleared")

    def set_wifi_zones(self, cells):
        """cells: iterable of (x, y) grid cells that block a robot's own
        broadcast while it is inside them. Physics is unaffected; only what
        OTHER robots know about that robot goes stale."""
        self.wifi_zones = set(cells)

    def uncertainty_radius(self, agent, now):
        if agent.silence_since is None:
            return 0.0

        silence = max(0.0, now - agent.silence_since)
        return silence * agent.spec.max_speed

    def kill_robot(self, robot_id):
        agent = self.agents.get(robot_id)

        if not agent or agent.mode == "failed":
            return

        self.sim.kill_robot(robot_id)
        agent.mode = "failed"
        agent.speed = 0.0
        agent.plan = [agent.at]
        self.graph.remove_wait(robot_id)
        self.metrics.event(
            "robot_failure",
            time=self.time,
            robot_id=robot_id
        )
        self._log("FAULT", robot_id, f"{robot_id} failed")
        self._requeue_tasks_of(robot_id)

    # ------------------------------------------------------------------
    # main loop
    # ------------------------------------------------------------------
    def finished(self):
        tasks = list(self.sim.tasks.values())

        if not tasks:
            return False

        return all(
            t.completed or t.status == TaskStatus.FAILED
            for t in tasks
        )

    def run(self, max_time=None):
        limit = self.config.max_sim_time if max_time is None else max_time

        while self.time < limit and not self.finished():
            self.step()

        self.metrics.completion_time = (
            self._finished_at
            if self._finished_at is not None
            else self.time
        )
        return self.metrics

    def step(self):
        dt = self.sim.config.dt
        now = self.time

        alive = [
            a for a in self.agents.values()
            if a.mode != "failed"
        ]
        order = sorted(alive, key=self._priority_key)

        self._activate_tasks(alive)
        self._update_broadcasts(alive, now)

        blocked_cells = {(o.x, o.y) for o in self.obstacles}

        for agent in order:
            if agent.target is not None and agent.target in blocked_cells:
                self._abort_transit(agent)
            elif (
                self.config.strategy == "stop_and_wait"
                and agent.target is not None
                and agent.stalled >= 1.5
            ):
                self._abort_transit(agent)

        for agent in order:
            if agent.target is None and now >= agent.next_decision:
                self._decide(agent, order)

        commands = {}
        for agent in order:
            commands[agent.id] = self._motion(agent, dt)

        self.sim.step(commands)

        for agent in order:
            self._after_motion(agent, dt)

        self._detect_standoffs(order)
        self._judge()

        if self.finished() and self._finished_at is None:
            self._finished_at = self.time
            self.metrics.completion_time = self.time
            self._log("SYSTEM", "SYSTEM", "All tasks finished")

    # ------------------------------------------------------------------
    # tasks
    # ------------------------------------------------------------------
    def _task_for(self, agent):
        for task in self.sim.tasks.values():
            if (
                task.assigned_robot == agent.id
                and not task.completed
                and task.status != TaskStatus.FAILED
            ):
                return task

        return None

    def _update_broadcasts(self, alive, now):
        for agent in alive:
            robot = self.sim.robots[agent.id]
            cell = self.grid.to_cell(robot.x, robot.y)

            if cell in self.wifi_zones:
                if not agent.silent:
                    agent.silent = True
                    agent.silence_since = now
                    self._log(
                        "FAULT",
                        agent.id,
                        f"{agent.id} entered a dead zone, "
                        f"broadcast lost",
                        key=(agent.id, "silent"),
                        min_interval=2.0
                    )
            else:
                if agent.silent:
                    self._log(
                        "FAULT",
                        agent.id,
                        f"{agent.id} regained signal",
                        key=(agent.id, "regain"),
                        min_interval=2.0
                    )

                agent.silent = False
                agent.silence_since = None
                agent.known_cell = cell
                agent.known_pose = (
                    robot.x, robot.y, robot.vx, robot.vy, robot.theta
                )

    def _activate_tasks(self, alive):
        for agent in alive:
            task = self._task_for(agent)

            if not task:
                agent.task_id = None
                continue

            if agent.task_id != task.id:
                agent.task_id = task.id
                agent.clock.tick()
                agent.req_clock = agent.clock.value
                agent.wait_time = 0.0
                agent.no_plan_time = 0.0
                task.activate()

                if task.id not in self.phase:
                    self.phase[task.id] = (
                        "deliver"
                        if agent.at == task.start
                        else "pickup"
                    )

    def _goal(self, agent, task):
        if self.phase.get(task.id) == "pickup":
            return task.start

        return task.destination

    def _requeue_tasks_of(self, robot_id):
        for task in self.sim.tasks.values():
            if task.completed or task.assigned_robot != robot_id:
                continue

            task.release()
            self.phase.pop(task.id, None)
            self.metrics.event(
                "task_requeued",
                time=self.time,
                task_id=task.id
            )
            self._log("TASK", "SYSTEM", f"{task.id} requeued")

            best = self._best_robot_for(task)

            if best:
                task.assigned_robot = best
                self.metrics.event(
                    "task_reassigned",
                    time=self.time,
                    task_id=task.id,
                    robot_id=best
                )
                self._log(
                    "TASK",
                    best,
                    f"{task.id} reassigned to {best}"
                )

    def _best_robot_for(self, task):
        best = None
        best_score = float("inf")

        for agent in self.agents.values():
            if agent.mode == "failed":
                continue

            robot = self.sim.robots[agent.id]
            load = sum(
                1 for t in self.sim.tasks.values()
                if t.assigned_robot == agent.id and not t.completed
            )
            dx = robot.x - task.start[0]
            dy = robot.y - task.start[1]
            score = (
                sqrt(dx * dx + dy * dy)
                + 5.0 * load
                + (100.0 - robot.battery) * 0.05
            )

            if score < best_score:
                best = agent.id
                best_score = score

        return best

    # ------------------------------------------------------------------
    # priority
    # ------------------------------------------------------------------
    def _intent(self, agent):
        robot = self.sim.robots[agent.id]

        return RobotIntent(
            agent.id,
            agent.req_clock,
            agent.wait_time,
            robot.x,
            robot.y,
            robot.vx,
            robot.vy,
            robot.theta
        )

    def _priority_key(self, agent):
        return self.arbiter.priority_key(self._intent(agent))

    def _higher(self, other, agent):
        return self._priority_key(other) < self._priority_key(agent)

    # ------------------------------------------------------------------
    # decisions
    # ------------------------------------------------------------------
    def _occupied_cells(self, agent):
        cells = {agent.at}

        if agent.target is not None:
            cells.add(agent.target)

        return cells

    def _reserve_uncertain(self, agent, table, now, keep_clear=()):
        """A silent robot's last known cell, inflated by how long it has
        been silent. The hard-stop safety tier stays independent of this
        (it always uses live ground truth, as a local sensor would) -- this
        only makes cooperative planning conservative around a robot we can
        no longer hear from."""
        # Growth is capped at LivenessConfig.failure_timeout worth of
        # travel: beyond that point a real system would mark the peer
        # PRESUMED_FAILED and stop expanding the hazard, so we do too.
        bounded_silence = min(
            now - (agent.silence_since or now),
            self.liveness_config.failure_timeout
        )
        radius_m = bounded_silence * agent.spec.max_speed
        cells = int(radius_m // self.grid.cell_size) + 1
        cells = min(cells, 3)
        x, y = agent.known_cell

        for dx in range(-cells, cells + 1):
            for dy in range(-cells, cells + 1):
                cand = (x + dx, y + dy)

                if abs(dx) + abs(dy) <= cells and cand not in keep_clear:
                    table.add_static(cand)

    def _decide(self, agent, order):
        task = self._task_for(agent)
        now = self.time

        if not task:
            agent.mode = "idle"
            agent.waiting_for = None
            agent.reason = "No task"

            if self.config.strategy == "arcnet":
                self._idle_evade(agent, order)

            agent.next_decision = now + self.config.replan_period
            return

        goal = self._goal(agent, task)

        if agent.at == goal:
            self._arrive_at_goal(agent, task)
            return

        if self.config.strategy == "stop_and_wait":
            self._decide_baseline(agent, task, goal, order)
        else:
            self._decide_arcnet(agent, task, goal, order)

    def _decide_arcnet(self, agent, task, goal, order):
        now = self.time
        table = ReservationTable()

        for other in order:
            if other.id == agent.id:
                continue

            if other.silent:
                self._reserve_uncertain(
                    other, table, now, keep_clear=(goal,)
                )
                continue

            for cell in self._occupied_cells(other):
                table.add_present(cell)

            if self._higher(other, agent) and len(other.plan) > 1:
                table.add_path(other.plan)

        for other in self.agents.values():
            if other.mode == "failed":
                for cell in self._occupied_cells(other):
                    table.add_static(cell)

        previous = agent.plan
        plan = self.st_planner.plan(
            agent.at,
            goal,
            self.obstacles,
            self.grid,
            table
        )

        agent.clock.tick()

        for other in order:
            if other.id != agent.id:
                other.clock.receive(agent.clock.value)

        if not plan:
            agent.plan = [agent.at]
            agent.no_plan_time += self.config.replan_period
            agent.mode = "waiting"
            agent.reason = "No conflict-free route within horizon"
            agent.next_decision = now + self.config.replan_period

            if agent.no_plan_time >= self.config.plan_fail_limit:
                self._fail_task(agent, task)

            return

        agent.no_plan_time = 0.0
        agent.plan = plan

        ideal = self._static_path(agent.at, goal)
        next_cell = plan[1] if len(plan) > 1 else agent.at

        if len(previous) > 1 and len(plan) > 1 and previous[1] != plan[1]:
            self.metrics.record_replan()
            self.metrics.event(
                "replan",
                time=now,
                robot_id=agent.id
            )

        if next_cell == agent.at:
            blocker = self._find_blocker(agent, ideal, table, order)
            agent.mode = "waiting"
            agent.waiting_for = blocker
            agent.blocker = blocker

            if blocker:
                agent.reason = f"Yielding to {blocker}"
                self._log(
                    "YIELD",
                    agent.id,
                    f"{agent.id} waits for {blocker} "
                    f"(priority tier "
                    f"{self.arbiter.aging_tier(agent.wait_time)}, "
                    f"clock {agent.req_clock})",
                    key=(agent.id, "yield", blocker),
                    min_interval=3.0
                )
            else:
                agent.reason = "Waiting for a free slot"

            agent.next_decision = now + self.config.replan_period
            return

        detour = len(plan) - 1 > len(ideal) - 1 if ideal else False

        if detour and ideal and next_cell != ideal[1]:
            self._log(
                "REROUTE",
                agent.id,
                f"{agent.id} rerouted around predicted conflict",
                key=(agent.id, "reroute"),
                min_interval=3.0
            )

        ok, blocker = self._clear_to_depart(agent, next_cell)

        if not ok:
            self._hold(agent, blocker, f"Holding: {blocker} too close")
            return

        self._start_move(agent, next_cell)
        agent.waiting_for = None
        agent.blocker = None
        agent.reason = "Following plan"

    def _clear_to_depart(self, agent, next_cell):
        """True if the robot may leave its cell now. Returns (ok, blocker)."""
        robot = self.sim.robots[agent.id]
        cx, cy = self.grid.to_position(*next_cell)
        dx = cx - robot.x
        dy = cy - robot.y
        d = sqrt(dx * dx + dy * dy)

        if d == 0:
            return True, None

        saved = agent.direction
        agent.direction = (dx / d, dy / d)
        cruise = min(agent.spec.max_speed, self.config.cruise_speed)
        level, blocker = self._safety(agent, cruise)
        agent.direction = saved

        return level != SafetyLevel.HARD_STOP, blocker

    def _hold(self, agent, blocker, why):
        agent.mode = "waiting"
        agent.waiting_for = blocker
        agent.blocker = blocker
        agent.reason = why
        agent.next_decision = self.time + self.config.replan_period

    def _decide_baseline(self, agent, task, goal, order):
        """Naive baseline: the lower-id robot always takes the direct
        route and lets the fixed hard-stop zone pause and resume it. The
        higher-id robot, whenever any other live robot is within earshot,
        plans a fresh route around every other robot's CURRENT cell. No
        timers, no sticky state -- recomputed plainly every time this robot
        reaches a waypoint (about once per second), which is what makes it
        eventually terminate instead of creeping into a repeated standoff.
        """
        cfg = self.config
        others = [
            o for o in self.agents.values()
            if o.id != agent.id and o.mode != "failed"
        ]
        nearby = any(
            self._distance(agent, o) < cfg.baseline_stop_gap * 4
            for o in others
        )
        yields_here = nearby and any(
            agent.id > o.id for o in others
        )

        obstacles = list(self.obstacles)

        for other in others:
            if other.mode == "failed":
                for cell in self._occupied_cells(other):
                    obstacles.append(Obstacle(cell[0], cell[1], True))
            elif yields_here:
                # Avoid the winner's whole remaining corridor, not just its
                # current cell -- a single-point dodge just gets re-met a
                # moment later as the winner keeps advancing toward us.
                other_task = self._task_for(other)
                other_goal = (
                    self._goal(other, other_task) if other_task else other.at
                )
                for cell in self._static_path(other.at, other_goal)[:6]:
                    obstacles.append(Obstacle(cell[0], cell[1], True))

        path = self.static_planner.plan(agent.at, goal, obstacles, self.grid)

        if not path or len(path) < 2:
            agent.plan = [agent.at]
            agent.mode = "waiting"
            agent.reason = "No route"
            agent.no_plan_time += cfg.replan_period
            agent.next_decision = self.time + cfg.replan_period

            if agent.no_plan_time >= cfg.plan_fail_limit:
                self._fail_task(agent, task)

            return

        agent.no_plan_time = 0.0
        agent.plan = path

        ok, blocker = self._clear_to_depart(agent, path[1])

        if not ok:
            self._hold(agent, blocker, f"Waiting: {blocker} ahead")

            if agent.wait_time >= cfg.plan_fail_limit:
                self._fail_task(agent, task)

            return

        self._start_move(agent, path[1])
        agent.reason = "Yielding route" if yields_here else "Direct route"

    def _distance(self, agent, other):
        me = self.sim.robots[agent.id]
        ot = self.sim.robots[other.id]
        return sqrt((me.x - ot.x) ** 2 + (me.y - ot.y) ** 2)

    def _start_move(self, agent, next_cell):
        cx, cy = self.grid.to_position(*next_cell)
        robot = self.sim.robots[agent.id]

        dx = cx - robot.x
        dy = cy - robot.y
        d = sqrt(dx * dx + dy * dy)

        if d == 0:
            return

        new_dir = (dx / d, dy / d)

        if (
            abs(new_dir[0] - agent.direction[0]) > 1e-6
            or abs(new_dir[1] - agent.direction[1]) > 1e-6
        ):
            agent.speed = 0.0

        agent.direction = new_dir
        agent.target = next_cell
        agent.mode = "moving"

    def _abort_transit(self, agent):
        """Current target cell just became blocked mid-flight: give up and
        return to the last cell centre so the next decision replans."""
        agent.speed = 0.0
        self._start_move(agent, agent.at)
        agent.mode = "retreating"
        agent.stalled = 0.0

    def _static_path(self, start, goal):
        return self.static_planner.plan(
            start,
            goal,
            self.obstacles,
            self.grid
        )

    def _find_blocker(self, agent, ideal, table, order):
        if ideal and len(ideal) > 1:
            first = ideal[1]

            for other in order:
                if other.id == agent.id:
                    continue

                if first in self._occupied_cells(other):
                    return other.id

                if other.plan and first in other.plan[:4]:
                    return other.id

        for other in order:
            if other.id != agent.id and self._higher(other, agent):
                return other.id

        return None

    @staticmethod
    def _ring(cell):
        x, y = cell
        return {(x, y), (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)}

    def _idle_evade(self, agent, order):
        table = ReservationTable()
        now = self.time

        for other in order:
            if other.id == agent.id:
                continue

            if other.silent:
                self._reserve_uncertain(other, table, now)
                continue

            for cell in self._occupied_cells(other):
                table.add_present(cell)

            if other.mode == "moving" or other.mode == "waiting":
                if len(other.plan) > 1:
                    table.add_path(other.plan)

        for other in self.agents.values():
            if other.mode == "failed":
                for cell in self._occupied_cells(other):
                    table.add_static(cell)

        ring = self._ring(agent.at)
        threatened = any(
            ring & table.occupied(k)
            for k in range(1, self.config.idle_clear_steps + 1)
        )

        if not threatened:
            agent.plan = [agent.at]
            return

        plan = self.st_planner.plan_evade(
            agent.at,
            self.obstacles,
            self.grid,
            table,
            inflate=True
        )

        if plan and len(plan) > 1:
            agent.plan = plan
            self._start_move(agent, plan[1])
            agent.mode = "evading"
            agent.reason = "Clearing the way"
            self._log(
                "YIELD",
                agent.id,
                f"{agent.id} steps aside to clear the route",
                key=(agent.id, "evade"),
                min_interval=3.0
            )

    def _arrive_at_goal(self, agent, task):
        if self.phase.get(task.id) == "pickup":
            self.phase[task.id] = "deliver"
            self._log(
                "TASK",
                agent.id,
                f"{agent.id} picked up {task.id} at {task.start}"
            )
            agent.next_decision = self.time
            return

        task.complete()
        self.metrics.tasks_completed += 1
        self.metrics.event(
            "task_completed",
            time=self.time,
            task_id=task.id,
            robot_id=agent.id
        )
        self._log(
            "TASK",
            agent.id,
            f"{agent.id} completed {task.id}"
        )

        agent.mode = "idle"
        agent.plan = [agent.at]
        agent.wait_time = 0.0
        agent.stalled = 0.0
        agent.task_id = None
        agent.next_decision = self.time

    def _fail_task(self, agent, task):
        task.fail()
        self.metrics.tasks_failed += 1
        self.metrics.event(
            "plan_failure",
            time=self.time,
            task_id=task.id
        )
        self._log(
            "TASK",
            agent.id,
            f"{task.id} failed: no route available"
        )
        agent.mode = "idle"
        agent.task_id = None

    # ------------------------------------------------------------------
    # motion and safety
    # ------------------------------------------------------------------
    def _pose_poly(self, x, y, theta, footprint):
        return footprint_polygon(x, y, theta, footprint)

    def _segment_remaining(self, agent):
        """Distance left to the current target cell centre (0 if parked)."""
        if agent.target is None:
            return 0.0

        robot = self.sim.robots[agent.id]
        tx, ty = self.grid.to_position(*agent.target)

        return sqrt((tx - robot.x) ** 2 + (ty - robot.y) ** 2)

    def _predicted_gap_time(self, agent, other, speed, horizon):
        """Earliest time within the horizon at which the two footprints come
        within the safety margin, if each robot keeps its current velocity
        until the end of its current cell-to-cell segment and then holds.
        Returns None if no such time exists."""
        me = self.sim.robots[agent.id]
        ot = self.sim.robots[other.id]
        dx, dy = agent.direction
        margin = self.config.hard_gap

        if other.mode == "failed":
            ovx = ovy = 0.0
            other_left = 0.0
        else:
            ovx, ovy = ot.vx, ot.vy
            other_left = self._segment_remaining(other)

        other_speed = sqrt(ovx * ovx + ovy * ovy)
        my_left = self._segment_remaining(agent)

        centre = sqrt((me.x - ot.x) ** 2 + (me.y - ot.y) ** 2)
        reach = (
            agent.footprint.radius
            + other.footprint.radius
            + margin
            + min(my_left, speed * horizon)
            + min(other_left, other_speed * horizon)
        )

        if centre > reach:
            return None

        my_theta = atan2(dy, dx)

        if other_speed > 1e-6:
            ot_theta = atan2(ovy, ovx)
            odx, ody = ovx / other_speed, ovy / other_speed
        else:
            ot_theta = ot.theta
            odx = ody = 0.0

        t = 0.0
        step = 0.1

        while t <= horizon + 1e-9:
            my_travel = min(speed * t, my_left)
            other_travel = min(other_speed * t, other_left)

            a_poly = self._pose_poly(
                me.x + dx * my_travel,
                me.y + dy * my_travel,
                my_theta,
                agent.footprint
            )
            b_poly = self._pose_poly(
                ot.x + odx * other_travel,
                ot.y + ody * other_travel,
                ot_theta,
                other.footprint
            )

            if polygon_gap(a_poly, b_poly) <= margin:
                return t

            t += step

        return None

    def _baseline_zone_hit(self, agent, other):
        """Fixed protective zone in front of the robot, as on a lidar."""
        me = self.sim.robots[agent.id]
        ot = self.sim.robots[other.id]
        fp = agent.footprint
        dx, dy = agent.direction
        reach = self.config.baseline_stop_gap

        zone = Footprint(
            "rectangle",
            fp.length + reach,
            fp.width + 2 * 0.3
        )
        poly = footprint_polygon(
            me.x + dx * reach / 2,
            me.y + dy * reach / 2,
            atan2(dy, dx),
            zone
        )
        other_poly = footprint_polygon(
            ot.x,
            ot.y,
            ot.theta,
            other.footprint
        )

        return polygon_gap(poly, other_poly) <= 0.0

    def _safety(self, agent, intended_speed):
        """Returns (level, blocker_id)."""
        cfg = self.config
        scfg = self.safety_config
        spec = agent.spec
        dx, dy = agent.direction

        worst = SafetyLevel.NORMAL
        worst_id = None

        t_hard = (
            intended_speed / spec.braking
            + scfg.reaction_latency
            + 0.2
        )
        t_caution = t_hard + 1.0

        for other in self.agents.values():
            if other.id == agent.id:
                continue

            if cfg.strategy == "stop_and_wait":
                if self._baseline_zone_hit(agent, other):
                    return SafetyLevel.HARD_STOP, other.id

                continue

            hit = self._predicted_gap_time(
                agent,
                other,
                intended_speed,
                t_caution
            )

            if hit is None:
                continue

            if hit <= t_hard:
                return SafetyLevel.HARD_STOP, other.id

            if worst != SafetyLevel.CAUTION and self._higher(other, agent):
                worst = SafetyLevel.CAUTION
                worst_id = other.id

        cs = self.grid.cell_size
        robot = self.sim.robots[agent.id]

        if agent.target is not None:
            tx, ty = self.grid.to_position(*agent.target)
            tcell = (int(tx // cs), int(ty // cs))

            for o in self.obstacles:
                if (o.x, o.y) == tcell:
                    return SafetyLevel.HARD_STOP, f"cell{o.x},{o.y}"

        return worst, worst_id

    def _motion(self, agent, dt):
        from .motion import MotionCommand

        robot = self.sim.robots[agent.id]
        spec = agent.spec

        if agent.target is None:
            agent.speed = 0.0
            agent.safety = SafetyLevel.NORMAL
            return MotionCommand(0.0, 0.0, 0.0, True)

        cruise = min(spec.max_speed, self.config.cruise_speed)
        level, blocker = self._safety(agent, cruise)

        agent.safety = level
        desired = cruise

        if level == SafetyLevel.HARD_STOP:
            desired = 0.0
            agent.blocker = blocker
        elif level == SafetyLevel.CAUTION:
            desired = cruise * self.config.caution_speed_factor
            agent.blocker = blocker
        elif agent.mode == "moving":
            agent.blocker = None

        if desired > agent.speed:
            agent.speed = min(desired, agent.speed + spec.acceleration * dt)
        else:
            agent.speed = max(desired, agent.speed - spec.braking * dt)

        tx, ty = self.grid.to_position(*agent.target)
        dist = sqrt((tx - robot.x) ** 2 + (ty - robot.y) ** 2)
        speed = min(agent.speed, dist / dt)

        dx, dy = agent.direction

        if speed <= 1e-9:
            return MotionCommand(0.0, 0.0, 0.0, True)

        return MotionCommand(dx * speed, dy * speed, 0.0, False)

    def _after_motion(self, agent, dt):
        robot = self.sim.robots[agent.id]

        if agent.target is not None:
            tx, ty = self.grid.to_position(*agent.target)
            dist = sqrt((tx - robot.x) ** 2 + (ty - robot.y) ** 2)

            if dist < 1e-6:
                robot.x = tx
                robot.y = ty
                agent.at = agent.target
                agent.target = None
                agent.history.append(agent.at)
                agent.history = agent.history[-6:]
                agent.wait_time = 0.0
                agent.stalled = 0.0
                agent.blocker = None
                agent.waiting_for = None

                if len(agent.plan) > 1 and agent.plan[0] != agent.at:
                    agent.plan = agent.plan[1:]

                agent.next_decision = self.time

        wants_to_move = (
            agent.target is not None
            or agent.mode == "waiting"
        )
        stopped = (
            wants_to_move
            and abs(robot.vx) + abs(robot.vy) < 1e-6
        )

        if stopped:
            agent.stalled += dt
            agent.wait_time += dt

            if not agent.was_stopped:
                agent.was_stopped = True
                self.metrics.record_stop()
                self.metrics.event(
                    "robot_stop",
                    time=self.time,
                    robot_id=agent.id
                )

                if agent.safety == SafetyLevel.HARD_STOP:
                    self._log(
                        "SAFETY",
                        agent.id,
                        f"{agent.id} HARD STOP: "
                        f"{agent.blocker} ahead",
                        key=(agent.id, "hard", agent.blocker),
                        min_interval=2.0
                    )
        else:
            agent.was_stopped = False

            if agent.mode == "moving":
                agent.stalled = 0.0

    # ------------------------------------------------------------------
    # deadlock handling
    # ------------------------------------------------------------------
    def _detect_standoffs(self, order):
        for agent in order:
            stuck = (
                agent.target is not None
                and agent.safety == SafetyLevel.HARD_STOP
                and agent.blocker
                and agent.stalled >= self.config.standoff_time
            )

            waiting = (
                agent.mode == "waiting"
                and agent.waiting_for
                and agent.stalled >= self.config.standoff_time
            )

            if stuck or waiting:
                self.graph.add_wait(agent.id, agent.blocker)
            else:
                self.graph.remove_wait(agent.id)

        cycle = self.graph.get_cycle()

        if cycle and not self._cycle_active:
            self._cycle_active = True
            self.metrics.record_deadlock()
            self.metrics.event(
                "deadlock",
                time=self.time,
                cycle=list(cycle)
            )
            self._log(
                "DEADLOCK",
                "SYSTEM",
                f"Deadlock cycle {cycle} detected"
            )

        if cycle and self.config.strategy == "arcnet":
            intents = [
                self._intent(self.agents[r])
                for r in cycle
                if r in self.agents
            ]
            victim = self.graph.choose_victim(intents, self.arbiter)

            if victim:
                self._retreat(self.agents[victim.id])
                self.graph.remove_wait(victim.id)

        elif not cycle and self._cycle_active:
            self._cycle_active = False
            self.metrics.record_deadlock_resolution()
            self._log("DEADLOCK", "SYSTEM", "Deadlock resolved")

    def _retreat(self, agent):
        """Victim backs up one cell so the higher priority robot passes."""
        if agent.mode == "retreating":
            return

        candidates = []

        for cell in reversed(agent.history[:-1]):
            if cell != agent.at:
                candidates.append(cell)

        x, y = agent.at

        for cell in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if cell not in candidates:
                candidates.append(cell)

        occupied = set()

        for other in self.agents.values():
            if other.id != agent.id:
                occupied |= self._occupied_cells(other)

        blocked = {(o.x, o.y) for o in self.obstacles}

        for cell in candidates:
            if (
                0 <= cell[0] < self.grid.width
                and 0 <= cell[1] < self.grid.height
                and cell not in blocked
                and cell not in occupied
                and abs(cell[0] - agent.at[0])
                + abs(cell[1] - agent.at[1]) == 1
            ):
                if agent.target is not None:
                    agent.target = None
                    robot = self.sim.robots[agent.id]
                    agent.at = self.grid.to_cell(robot.x, robot.y)

                agent.speed = 0.0
                self._start_move(agent, cell)
                agent.mode = "retreating"
                agent.stalled = 0.0
                agent.wait_time = 0.0
                self._log(
                    "DEADLOCK",
                    agent.id,
                    f"{agent.id} selected as victim, backs up",
                    key=(agent.id, "retreat"),
                    min_interval=3.0
                )
                return

    # ------------------------------------------------------------------
    # judge, log, snapshot
    # ------------------------------------------------------------------
    def _judge(self):
        for kind, a, b, gap in self.judge.check():
            if kind == "collision":
                self.metrics.record_collision()
                self.metrics.event(
                    "collision",
                    time=self.time,
                    robot_a=a,
                    robot_b=b
                )
                self._log("SAFETY", "JUDGE", f"COLLISION {a}/{b}")
            elif kind == "near_miss":
                self.metrics.record_near_miss()
                self.metrics.event(
                    "near_miss",
                    time=self.time,
                    robot_a=a,
                    robot_b=b,
                    gap=round(gap, 3)
                )

    def _log(
        self,
        kind,
        source,
        message,
        key=None,
        min_interval=0.0
    ):
        now = self.time

        if key is not None:
            last = self._log_seen.get(key)

            if last is not None and now - last < min_interval:
                return

            self._log_seen[key] = now

        self.events.append({
            "time": round(now, 2),
            "kind": kind,
            "source": source,
            "message": message
        })

        if len(self.events) > 300:
            self.events = self.events[-300:]

    def snapshot(self):
        robots = []
        cs = self.grid.cell_size

        for agent in self.agents.values():
            robot = self.sim.robots[agent.id]
            task = self._task_for(agent)
            spec = agent.spec

            path = [
                list(self.grid.to_position(*c))
                for c in agent.plan
            ] if agent.mode != "failed" else []

            robots.append({
                "id": agent.id,
                "x": robot.x,
                "y": robot.y,
                "theta": robot.theta,
                "vx": robot.vx,
                "vy": robot.vy,
                "battery": robot.battery,
                "active": robot.active,
                "task_id": task.id if task else None,
                "mode": agent.mode,
                "safety": agent.safety.value,
                "waiting_for": agent.waiting_for,
                "reason": agent.reason,
                "wait_time": round(agent.wait_time, 1),
                "priority_tier": self.arbiter.aging_tier(agent.wait_time),
                "logical_clock": agent.req_clock,
                "cell": list(agent.at),
                "length": spec.length,
                "width": spec.width,
                "path": path
            })

        tasks = [
            {
                "id": t.id,
                "start": list(t.start),
                "destination": list(t.destination),
                "assigned_robot": t.assigned_robot,
                "completed": t.completed,
                "status": t.status.value,
                "phase": self.phase.get(t.id)
            }
            for t in self.sim.tasks.values()
        ]

        return {
            "time": round(self.time, 2),
            "strategy": self.config.strategy,
            "grid": {
                "width": self.grid.width,
                "height": self.grid.height,
                "cell_size": cs
            },
            "robots": robots,
            "tasks": tasks,
            "obstacles": [
                {
                    "x": o.x,
                    "y": o.y,
                    "temporary": o.temporary
                }
                for o in self.obstacles
            ],
            "events": self.events[-100:],
            "metrics": self.metrics.as_dict()
        }