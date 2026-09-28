from dataclasses import dataclass


@dataclass
class RobotMessage:
    sender_id: str
    logical_clock: int
    timestamp: float
    x: float
    y: float
    vx: float
    vy: float
    theta: float
    status: int