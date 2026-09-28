from dataclasses import dataclass


@dataclass(frozen=True)
class Obstacle:
    x: int
    y: int
    temporary: bool = False


def is_blocked(obstacles, x, y):
    return any(o.x == x and o.y == y for o in obstacles)