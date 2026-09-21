"""
Standalone proof: same start/goal, same grid, only difference is whether
peer intent is supplied. If the innovation is real, the path or arrival
time should change when a predicted conflict exists -- not just avoid
literal currently-occupied cells (that's baseline A*, not predictive).
"""

import warehouse_grid as grid
from path_planner import plan_path, PeerIntent


def show(label, result):
    if result is None:
        print(f"{label}: NO PATH FOUND")
        return
    print(f"{label}: {len(result.path_cells)} cells, total_cost={result.total_cost:.1f}, "
          f"arrives goal at t={result.arrival_times[-1]:.1f}s")


start_b = (8, 5)
goal_b = (8, 34)

print("Grid size:", grid.GRID_WIDTH, "x", grid.GRID_HEIGHT)
print("Obstacle cell count:", len(grid.OBSTACLES))
print()

baseline = plan_path(start_b, goal_b, start_time=0.0, robot_id="robot2", peer_intents={})
show("No peer knowledge (baseline A*)", baseline)

conflict_cell = baseline.path_cells[len(baseline.path_cells) // 2] if baseline else (8, 20)
conflict_time = baseline.arrival_times[len(baseline.arrival_times) // 2] if baseline else 15.0

peer_a_intent = PeerIntent(
    robot_id="robot1",
    planned_cells=[conflict_cell],
    planned_times=[conflict_time],
)

print(f"\nRobot1 has broadcast intent: will be at {conflict_cell} at t={conflict_time:.1f}s")
print("(this cell sits on Robot2's baseline shortest path)\n")

predictive = plan_path(start_b, goal_b, start_time=0.0, robot_id="robot2",
                        peer_intents={"robot1": peer_a_intent})
show("With peer intent known (predictive-cost A*)", predictive)

if baseline and predictive:
    same_path = baseline.path_cells == predictive.path_cells
    same_timing = baseline.arrival_times == predictive.arrival_times
    print(f"\nIdentical path AND timing to baseline: {same_path and same_timing}")
    if not (same_path and same_timing):
        print(">>> Planner rerouted or re-timed to avoid predicted future congestion. <<<")
