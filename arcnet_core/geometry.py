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