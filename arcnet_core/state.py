from dataclasses import dataclass


@dataclass
class RobotState:
    id: str
    x: float
    y: float
    theta: float
    vx: float
    vy: float
    battery: float
    task_id: str | None = None
    active: bool = True