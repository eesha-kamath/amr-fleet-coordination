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
    