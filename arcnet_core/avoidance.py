from dataclasses import dataclass
from math import sqrt, atan2


@dataclass
class AvoidanceResult:
    vx: float
    vy: float
    omega: float
    changed: bool


class LocalAvoidance:

    def __init__(self, time_horizon=2.0, margin=0.1):
        self.time_horizon = time_horizon
        self.margin = margin

    def avoid(
        self,
        x,
        y,
        vx,
        vy,
        peers
    ):
        rvx = vx
        rvy = vy
        changed = False

        for peer in peers:
            dx = peer["x"] - x
            dy = peer["y"] - y

            d = sqrt(dx * dx + dy * dy)

            if d <= 0:
                continue

            pvx = peer.get("vx", 0.0)
            pvy = peer.get("vy", 0.0)

            rel_vx = pvx - rvx
            rel_vy = pvy - rvy

            closing = dx * rel_vx + dy * rel_vy

            if closing >= 0:
                continue

            closing_speed = -closing / d

            if closing_speed <= 0:
                continue

            t = d / closing_speed

            if t > self.time_horizon:
                continue

            nx = -dy / d
            ny = dx / d

            cross = dx * rel_vy - dy * rel_vx

            if cross > 0:
                nx = -nx
                ny = -ny

            strength = (
                self.time_horizon - t
            ) / self.time_horizon

            shift = max(
                0.1,
                strength
            )

            rvx += nx * shift
            rvy += ny * shift

            changed = True

        speed = sqrt(
            rvx * rvx +
            rvy * rvy
        )

        omega = 0.0

        if speed > 0:
            omega = atan2(rvy, rvx)

        return AvoidanceResult(
            rvx,
            rvy,
            omega,
            changed
        )