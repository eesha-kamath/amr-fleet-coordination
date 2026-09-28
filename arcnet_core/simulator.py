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