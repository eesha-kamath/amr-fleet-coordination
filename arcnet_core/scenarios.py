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