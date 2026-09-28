import unittest

from arcnet_core import (
    RobotSpec,
    Footprint,
    SafetyConfig,
    SafetyLevel,
    Grid,
    Obstacle,
    PathPlanner,
    RobotIntent,
    ConflictArbiter,
    WaitForGraph,
    LivenessConfig,
    Peer,
    PeerManager,
    MotionController,
    PeerStatus
)

from arcnet_core.safety import (
    hard_stop_distance,
    get_safety_level
)


class TestCore(unittest.TestCase):


    def robot(self):
        return RobotSpec(
            id="R1",
            shape="rectangle",
            length=0.9,
            width=0.64,
            height=0.19,
            mass=70,
            wheel_track=0.6,
            wheel_radius=0.1,
            max_speed=1.0,
            acceleration=0.5,
            braking=1.0,
            safety_margin=0.1
        )

    def test_robot(self):
        r = self.robot()
        r.validate()

        self.assertGreater(r.radius, 0)
        self.assertEqual(r.braking_distance, 0.5)

    def test_footprint(self):
        r = self.robot()
        f = Footprint.from_robot(r)

        self.assertEqual(f.length, r.length)
        self.assertEqual(f.width, r.width)

    def test_safety(self):
        r = self.robot()
        s = SafetyConfig(0.1, 0.1)

        d = hard_stop_distance(r, s)

        self.assertGreater(d, 0)

        self.assertEqual(
            get_safety_level(d, r, s),
            SafetyLevel.HARD_STOP
        )

    def test_grid_and_planner(self):
        g = Grid(10, 10, 1.0)
        obstacles = [Obstacle(2, 1)]

        p = PathPlanner().plan(
            (0, 1),
            (4, 1),
            obstacles,
            g
        )

        self.assertTrue(p)
        self.assertNotIn((2, 1), p)

    def test_predictive_planner(self):
        g = Grid(10, 10, 1.0)

        peer_paths = {
            "R2": [
                (2, 0),
                (2, 1),
                (2, 2),
                (2, 3)
            ]
        }

        p = PathPlanner().plan_predictive(
            (0, 1),
            (4, 1),
            [],
            g,
            peer_paths
        )

        self.assertTrue(p)

    def test_arbiter(self):
        a = RobotIntent(
            "R1",
            1,
            2.0,
            0,
            0,
            0,
            0,
            0
        )

        b = RobotIntent(
            "R2",
            2,
            1.0,
            0,
            0,
            0,
            0,
            0
        )

        winner = ConflictArbiter().decide(a, b)

        self.assertEqual(winner.id, "R1")

    def test_deadlock(self):
        g = WaitForGraph()

        g.add_wait("R1", "R2")
        g.add_wait("R2", "R3")
        g.add_wait("R3", "R1")

        self.assertTrue(g.has_deadlock())

        self.assertEqual(
            set(g.get_cycle()),
            {"R1", "R2", "R3"}
        )

    def test_deadlock_victim(self):
        g = WaitForGraph()

        g.add_wait("R1", "R2")
        g.add_wait("R2", "R3")
        g.add_wait("R3", "R1")

        a = RobotIntent("R1", 1, 5, 0, 0, 0, 0, 0)
        b = RobotIntent("R2", 2, 1, 0, 0, 0, 0, 0)
        c = RobotIntent("R3", 3, 2, 0, 0, 0, 0, 0)

        victim = g.choose_victim(
            [a, b, c],
            ConflictArbiter()
        )

        self.assertEqual(victim.id, "R2")

    def test_peer_manager(self):
        config = LivenessConfig(
            warning_timeout=1.0,
            failure_timeout=3.0
        )

        manager = PeerManager(config)

        peer = Peer(
            "R2",
            2,
            2,
            0,
            0,
            0,
            0,
            0
        )

        manager.add(peer)

        status = manager.update(
            "R2",
            0.5,
            True,
            True
        )

        self.assertEqual(
            status,
            PeerStatus.ACTIVE
        )

    def test_motion(self):
        m = MotionController()

        normal = m.command(
            SafetyLevel.NORMAL,
            1.0,
            0.0
        )

        caution = m.command(
            SafetyLevel.CAUTION,
            1.0,
            0.0
        )

        stop = m.command(
            SafetyLevel.HARD_STOP,
            1.0,
            0.0
        )

        self.assertEqual(normal.vx, 1.0)
        self.assertEqual(caution.vx, 0.5)
        self.assertTrue(stop.stop)
        self.assertEqual(stop.vx, 0.0)


    def test_conflict_detector(self):
        from arcnet_core import ConflictDetector

        detector = ConflictDetector()

        path = [
         (0, 0),
         (1, 0),
         (2, 0)
        ]

        peers = {
            "R2": [
            (3, 0),
            (1, 0),
            (2, 1)
            ]
        }

        conflicts = detector.detect(
            "R1",
            path,
            peers
        )

        self.assertTrue(conflicts)
        self.assertEqual(conflicts[0].kind, "vertex")

    def test_engine_safety(self):
        from arcnet_core import ArcnetEngine

        r = self.robot()

        grid = Grid(
            10,
            10,
            1.0
        )

        engine = ArcnetEngine(
            {"R1": r},
            grid,
            [],
            SafetyConfig(0.1, 0.1)
        )

        level, command = engine.command_for_distance(
            "R1",
            0.1,
            1.0,
            0.0
        )

        self.assertEqual(
            level,
            SafetyLevel.HARD_STOP
        )

        self.assertTrue(
            command.stop
        )

    def test_surface_gap(self):
        from arcnet_core import Footprint
        from arcnet_core.geometry import surface_gap

        a = Footprint("circle", 1.0, 1.0)
        b = Footprint("circle", 1.0, 1.0)

        self.assertEqual(
            surface_gap(
                (0, 0),
                (3, 0),
                a,
                b
            ),
            2.0
        )


    def test_surface_overlap(self):
        from arcnet_core import Footprint
        from arcnet_core.geometry import surface_gap

        a = Footprint("circle", 1.0, 1.0)
        b = Footprint("circle", 1.0, 1.0)

        self.assertEqual(
            surface_gap(
                (0, 0),
                (1, 0),
                a,
                b
            ),
            0.0
        )


    def test_uncertainty_radius(self):
        from arcnet_core import Peer, PeerManager, LivenessConfig

        manager = PeerManager(
            LivenessConfig(
                warning_timeout=1.0,
                failure_timeout=3.0,
                margin=0.1
            )
        )

        peer = Peer(
            id="R2",
            x=5.0,
            y=5.0,
            vx=1.0,
            vy=0.0,
            theta=0.0,
            last_local_broadcast=0.0,
            last_global_heartbeat=0.0,
            max_speed=2.0
        )

        manager.add(peer)

        radius = manager.uncertainty_radius(
            "R2",
            2.0
        )

        self.assertAlmostEqual(
            radius,
            4.1
        )


    def test_peer_failure(self):
        from arcnet_core import Peer, PeerManager, LivenessConfig
        from arcnet_core.peer import PeerStatus

        manager = PeerManager(
            LivenessConfig(
                warning_timeout=1.0,
                failure_timeout=3.0,
                margin=0.1
            )
        )

        peer = Peer(
            id="R2",
            x=5.0,
            y=5.0,
            vx=0.0,
            vy=0.0,
            theta=0.0,
            last_local_broadcast=0.0,
            last_global_heartbeat=0.0
        )

        manager.add(peer)

        status = manager.update(
            "R2",
            4.0,
            False,
            False
        )

        self.assertEqual(
            status,
            PeerStatus.PRESUMED_FAILED
        )



    def test_cell_neighbors(self):
        from arcnet_core import Cell

        cell = Cell(2, 3)

        neighbors = cell.neighbors()

        self.assertEqual(len(neighbors), 9)
        self.assertIn(Cell(2, 3), neighbors)
        self.assertIn(Cell(1, 2), neighbors)
        self.assertIn(Cell(3, 4), neighbors)


    def test_cell_position(self):
        from arcnet_core import SpatialManager

        manager = SpatialManager(10.0)

        cell = manager.get_cell(
            25.0,
            35.0
        )

        self.assertEqual(
            cell.x,
            2
        )

        self.assertEqual(
            cell.y,
            3
        )


    def test_cell_change(self):
        from arcnet_core import SpatialManager

        manager = SpatialManager(10.0)

        event = manager.set_robot_position(
            "R1",
            5.0,
            5.0
        )

        self.assertEqual(
            event["event"],
            "CELL_ENTER"
        )

        event = manager.set_robot_position(
            "R1",
            15.0,
            5.0
        )

        self.assertEqual(
            event["event"],
            "CELL_CHANGE"
        )

        self.assertEqual(
            event["old_cell"].x,
            0
        )

        self.assertEqual(
            event["new_cell"].x,
            1
        )


    def test_spatial_subscriptions(self):
        from arcnet_core import SpatialManager

        manager = SpatialManager(10.0)

        manager.set_robot_position(
            "R1",
            25.0,
            35.0
        )

        cells = manager.get_subscriptions("R1")

        self.assertEqual(
            len(cells),
            9
        )

    def test_local_avoidance(self):
        from arcnet_core.avoidance import LocalAvoidance

        avoidance = LocalAvoidance()

        result = avoidance.avoid(
            0.0,
            0.0,
            1.0,
            0.0,
            [
                {
                    "x": 2.0,
                    "y": 0.0,
                    "vx": -1.0,
                    "vy": 0.0
                }
            ]
        )

        self.assertTrue(result.changed)

        self.assertNotEqual(
            result.vy,
            0.0
        )


    def test_local_avoidance_no_conflict(self):
        from arcnet_core.avoidance import LocalAvoidance

        avoidance = LocalAvoidance()

        result = avoidance.avoid(
            0.0,
            0.0,
            1.0,
            0.0,
            [
                {
                    "x": 10.0,
                    "y": 10.0,
                    "vx": 0.0,
                    "vy": 0.0
                }
            ]
        )

        self.assertFalse(result.changed)

        self.assertEqual(
            result.vx,
            1.0
        )

        self.assertEqual(
            result.vy,
            0.0
        )


    def test_engine_avoidance(self):
        from arcnet_core import (
            ArcnetEngine,
            RobotSpec,
            Grid,
            SafetyConfig
        )

        r1 = RobotSpec(
            "R1",
            "circle",
            1.0,
            1.0,
            0.5,
            10.0,
            0.5,
            0.1,
            1.0,
            1.0,
            1.0,
            0.1
        )

        engine = ArcnetEngine(
            {"R1": r1},
            Grid(10, 10, 1.0),
            [],
            SafetyConfig()
        )

        engine.set_peer_state(
            "R2",
            1.5,
            0.0,
            -1.0,
            0.0
        )

        level, command = engine.command_for_distance(
            "R1",
            1.5,
            1.0,
            0.0
        )

        self.assertNotEqual(
            command.vy,
            0.0
        )

    def test_simulator(self):
        from arcnet_core import (
            WarehouseSimulator,
            Grid,
            RobotSpec
        )

        robot = RobotSpec(
            "R1",
            "circle",
            1.0,
            1.0,
            0.5,
            10.0,
            0.5,
            0.1,
            1.0,
            1.0,
            1.0,
            0.1
        )

        sim = WarehouseSimulator(
            Grid(10, 10, 1.0),
            [],
            {"R1": robot}
        )

        sim.add_robot(
            "R1",
            1.0,
            1.0
        )

        from arcnet_core.motion import MotionCommand

        sim.step({
            "R1": MotionCommand(
                1.0,
                0.0,
                0.0
            )
        })

        r = sim.get_robot("R1")

        self.assertAlmostEqual(
            r.x,
            1.1
        )

        self.assertAlmostEqual(
            r.y,
            1.0
        )


    def test_kill_robot(self):
        from arcnet_core import (
            WarehouseSimulator,
            Grid,
            RobotSpec
        )

        robot = RobotSpec(
            "R1",
            "circle",
            1.0,
            1.0,
            0.5,
            10.0,
            0.5,
            0.1,
            1.0,
            1.0,
            1.0,
            0.1
        )

        sim = WarehouseSimulator(
            Grid(10, 10, 1.0),
            [],
            {"R1": robot}
        )

        sim.add_robot(
            "R1",
            1.0,
            1.0
        )

        sim.kill_robot("R1")

        r = sim.get_robot("R1")

        self.assertFalse(r.active)
        self.assertEqual(r.vx, 0.0)
        self.assertEqual(r.vy, 0.0)

    def test_fleet_runner(self):
        from arcnet_core import (
            WarehouseSimulator,
            Grid,
            RobotSpec,
            Task,
            ArcnetEngine,
            SafetyConfig,
            FleetRunner
        )

        robot = RobotSpec(
            "R1",
            "circle",
            1.0,
            1.0,
            0.5,
            10.0,
            0.5,
            0.1,
            1.0,
            1.0,
            1.0,
            0.1
        )

        grid = Grid(
            10,
            10,
            1.0
        )

        sim = WarehouseSimulator(
            grid,
            [],
            {"R1": robot}
        )

        sim.add_robot(
            "R1",
            0.5,
            0.5
        )

        task = Task(
            "T1",
            (0, 0),
            (2, 0),
            "R1"
        )

        sim.add_task(task)

        engine = ArcnetEngine(
            {"R1": robot},
            grid,
            [],
            SafetyConfig()
        )

        runner = FleetRunner(
            sim,
            engine
        )

        metrics = runner.run(
            max_steps=100
        )

        self.assertEqual(
            metrics.tasks_completed,
            1
        )


    def test_scenario_runner(self):
        from arcnet_core import (
            WarehouseSimulator,
            Grid,
            RobotSpec,
            ArcnetEngine,
            SafetyConfig,
            Scenario,
            ScenarioRunner
        )

        robot = RobotSpec(
            "R1",
            "circle",
            1.0,
            1.0,
            0.5,
            10.0,
            0.5,
            0.1,
            1.0,
            1.0,
            1.0,
            0.1
        )

        grid = Grid(10, 10, 1.0)

        sim = WarehouseSimulator(
            grid,
            [],
            {"R1": robot}
        )

        engine = ArcnetEngine(
            {"R1": robot},
            grid,
            sim.obstacles,
            SafetyConfig()
        )

        runner = ScenarioRunner(
            sim,
            engine
        )

        scenario = Scenario(
            "S1",
            "Test",
            "Basic scenario",
            [],
            [
                {
                    "id": "R1",
                    "x": 1.0,
                    "y": 1.0
                }
            ],
            []
        )

        runner.load(scenario)

        self.assertIn(
            "R1",
            sim.robots
        )


    def test_scenario_library(self):
        from arcnet_core import ScenarioLibrary

        library = ScenarioLibrary()

        scenarios = library.all()

        self.assertEqual(
            len(scenarios),
            8
        )

        scenario = library.get(
            "head_on"
        )

        self.assertEqual(
            scenario["id"],
            "head_on"
        )

        self.assertEqual(
            len(scenario["robots"]),
            2
        )


    def test_fault_manager(self):
        from arcnet_core import FaultManager

        manager = FaultManager()

        manager.kill_robot("R2")

        self.assertIn(
            "R2",
            manager.failed_robots()
        )

        manager.wifi_dead_zone(
            2,
            3
        )

        self.assertTrue(
            manager.is_wifi_blocked(
                2,
                3
            )
        )

        manager.blocked_cell(
            4,
            5
        )

        self.assertIn(
            (4, 5),
            manager.blocked_cells()
        )


    def test_task_lifecycle(self):
        from arcnet_core import Task, TaskStatus

        task = Task(
            "T1",
            (0, 0),
            (2, 2),
            "R1"
        )

        self.assertEqual(
            task.status,
            TaskStatus.PENDING
        )

        task.activate()

        self.assertEqual(
            task.status,
            TaskStatus.ACTIVE
        )

        task.complete()

        self.assertEqual(
            task.status,
            TaskStatus.COMPLETED
        )



if __name__ == "__main__":
    unittest.main()