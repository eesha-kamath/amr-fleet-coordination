"""
intent_codec.py
JSON-over-std_msgs/String for /fleet/intent. v2: adds lamport_time,
waiting_for, wait_started_at for priority tiebreaking + deadlock detection.
"""

import json
import time

def encode_intent(robot_id, x, y, goal_x, goal_y, planned_cells, planned_times,
                   battery=100.0, lamport_time=0, waiting_for=None, wait_started_at=None):
    payload = {
        "robot_id": robot_id, "x": x, "y": y,
        "goal_x": goal_x, "goal_y": goal_y,
        "planned_cells": planned_cells, "planned_times": planned_times,
        "battery": battery,
        "lamport_time": lamport_time,
        "waiting_for": waiting_for,
        "wait_started_at": wait_started_at,
        "sent_at": time.time(),
    }
    return json.dumps(payload)

def decode_intent(data_str):
    return json.loads(data_str)

def is_stale(sent_at, now, max_age):
    return (now - sent_at) > max_age
