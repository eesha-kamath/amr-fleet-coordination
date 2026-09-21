"""
motion_controller.py

Pure logic, no ROS dependency. v2: three distinct stop thresholds,
not one -- this is the fix for the mutual-freeze bug:

  - HARD_SAFETY_STOP_DISTANCE: universal, priority-blind, true last
    resort. Nobody's exempt, ever. Pure physical safety net.
  - EMERGENCY_STOP_DISTANCE: the "caution zone". Entering it no longer
    means an automatic stop -- brain_node.py now checks the priority
    tiebreak first. Only the LOSING robot stops here; the winner keeps
    moving, since its own plan already routes around the peer via the
    planner's hard-block. Previously BOTH robots stopped here
    regardless of priority, which is exactly what produced the mutual
    standoff / long freeze seen with robot3.
  - STOP_AND_WAIT_DISTANCE: baseline mode only. Deliberately dumb, no
    priority, no yielding -- this IS the comparison point.
"""

import math
from dataclasses import dataclass

LINEAR_SPEED = 0.4
ANGULAR_GAIN = 1.5
GOAL_TOLERANCE = 0.25
HEADING_SLOWDOWN_ANGLE = 0.6

COLLISION_DISTANCE = 0.45
NEAR_MISS_DISTANCE = 1.0
HARD_SAFETY_STOP_DISTANCE = 0.5
EMERGENCY_STOP_DISTANCE = 0.7
STOP_AND_WAIT_DISTANCE = 1.2

@dataclass
class VelocityCommand:
    linear_x: float
    angular_z: float

def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def _normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def distance_to(cx, cy, tx, ty):
    return math.hypot(tx - cx, ty - cy)

def compute_velocity_command(current_x, current_y, current_yaw, target_x, target_y) -> VelocityCommand:
    dist = distance_to(current_x, current_y, target_x, target_y)
    if dist < GOAL_TOLERANCE:
        return VelocityCommand(linear_x=0.0, angular_z=0.0)
    target_heading = math.atan2(target_y - current_y, target_x - current_x)
    heading_error = _normalize_angle(target_heading - current_yaw)
    angular = ANGULAR_GAIN * heading_error
    linear = 0.0 if abs(heading_error) > HEADING_SLOWDOWN_ANGLE else LINEAR_SPEED
    return VelocityCommand(linear_x=linear, angular_z=angular)

def closest_peer_distance(my_x, my_y, peer_positions):
    best, best_id = None, None
    for rid, (px, py) in peer_positions.items():
        d = distance_to(my_x, my_y, px, py)
        if best is None or d < best:
            best, best_id = d, rid
    return best, best_id

def hard_safety_stop_needed(min_dist):
    return min_dist is not None and min_dist < HARD_SAFETY_STOP_DISTANCE

def emergency_stop_needed(min_dist):
    return min_dist is not None and min_dist < EMERGENCY_STOP_DISTANCE

def stop_and_wait_needed(min_dist):
    return min_dist is not None and min_dist < STOP_AND_WAIT_DISTANCE
