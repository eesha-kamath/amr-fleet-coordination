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