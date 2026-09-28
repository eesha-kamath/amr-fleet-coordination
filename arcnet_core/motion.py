from dataclasses import dataclass

from .safety import SafetyLevel


@dataclass
class MotionCommand:
    vx: float
    vy: float
    omega: float
    stop: bool = False


class MotionController:

    def normal(self, vx, vy, omega=0.0):
        return MotionCommand(
            vx,
            vy,
            omega,
            False
        )

    def caution(self, vx, vy, omega=0.0):
        return MotionCommand(
            vx * 0.5,
            vy * 0.5,
            omega,
            False
        )

    def hard_stop(self):
        return MotionCommand(
            0.0,
            0.0,
            0.0,
            True
        )

    def command(self, level, vx, vy, omega=0.0):
        if level == SafetyLevel.HARD_STOP:
            return self.hard_stop()

        if level == SafetyLevel.CAUTION:
            return self.caution(
                vx,
                vy,
                omega
            )

        return self.normal(
            vx,
            vy,
            omega
        )


def safety_command(level):
    controller = MotionController()

    return controller.command(
        level,
        0.0,
        0.0
    )