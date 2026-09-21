"""metrics.py -- CSV logger, one row per robot per completed run."""

import csv, os, time

METRICS_FILE = os.path.expanduser("~/amr_ws/fleet_dev/metrics_log.csv")
FIELDS = ["timestamp", "run_mode", "robot_id", "start_x", "start_y",
          "goal_x", "goal_y", "task_time_sec", "collisions", "near_misses",
          "emergency_stops", "replans", "deadlocks_detected", "deadlocks_broken"]

def log_run(run_mode, robot_id, start_x, start_y, goal_x, goal_y, task_time_sec,
            collisions, near_misses, emergency_stops, replans,
            deadlocks_detected=0, deadlocks_broken=0):
    file_exists = os.path.isfile(METRICS_FILE)
    with open(METRICS_FILE, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        if not file_exists:
            writer.writeheader()
        writer.writerow({
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"), "run_mode": run_mode,
            "robot_id": robot_id, "start_x": f"{start_x:.2f}", "start_y": f"{start_y:.2f}",
            "goal_x": goal_x, "goal_y": goal_y, "task_time_sec": f"{task_time_sec:.2f}",
            "collisions": collisions, "near_misses": near_misses,
            "emergency_stops": emergency_stops, "replans": replans,
            "deadlocks_detected": deadlocks_detected, "deadlocks_broken": deadlocks_broken,
        })
