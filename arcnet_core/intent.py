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