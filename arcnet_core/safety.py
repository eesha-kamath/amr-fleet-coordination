from enum import Enum
from .robot import RobotSpec
from .geometry import Footprint
from .config import SafetyConfig


class SafetyLevel(Enum):
    NORMAL = "normal"
    CAUTION = "caution"
    HARD_STOP = "hard_stop"


def hard_stop_distance(robot: RobotSpec, config: SafetyConfig):
    footprint = Footprint.from_robot(robot)

    return (
        footprint.radius
        + robot.braking_distance
        + robot.max_speed * config.reaction_latency
        + config.margin
    )


def get_safety_level(distance, robot: RobotSpec, config: SafetyConfig):
    d = hard_stop_distance(robot, config)

    if distance <= d:
        return SafetyLevel.HARD_STOP

    if distance <= d * 2:
        return SafetyLevel.CAUTION

    return SafetyLevel.NORMAL