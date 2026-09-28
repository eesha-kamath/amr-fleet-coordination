from dataclasses import dataclass

from .motion import MotionCommand


@dataclass
class CoordinationDecision:
    robot_id: str
    path: list
    command: MotionCommand
    waiting_for: str | None = None
    conflict: object | None = None
    deadlock: bool = False
    victim: str | None = None
    replan: bool = False