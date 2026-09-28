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