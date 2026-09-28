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