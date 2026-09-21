"""
path_planner.py

Three layers now:
  1. SOFT (predictive): peer's future intended cell near the same time
     = expensive, not forbidden.
  2. HARD-LIVE: peer's CURRENT physical position = un-passable, always.
  3. HARD-CONTESTED (new): if a peer's claimed future cell+time overlaps
     mine AND that peer currently wins the priority tiebreak (Lamport+
     aging, see conflict_resolver.py), their claim is hard-blocked for
     me too -- I yield. If I win instead, it stays soft-cost only,
     because the peer's own planner will independently reach the same
     conclusion and yield to me. No message exchange needed -- both
     sides compute the same deterministic answer from shared broadcasts.
"""

import heapq
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

import warehouse_grid as grid
from conflict_resolver import PriorityInfo, wins

MOVE_TIME = 1.0
CONGESTION_WINDOW = 2.0
CONGESTION_PENALTY = 15.0
DYNAMIC_BLOCK_RADIUS = 1
DYNAMIC_BLOCK_HORIZON = 2.5

@dataclass
class PeerIntent:
    robot_id: str
    planned_cells: List[Tuple[int, int]]
    planned_times: List[float]
    current_x: float = 0.0
    current_y: float = 0.0

@dataclass
class PlanResult:
    path_cells: List[Tuple[int, int]]
    arrival_times: List[float]
    total_cost: float

def _dynamic_blocked_cells(peer_intents, ignore_robot_id):
    blocked = set()
    for robot_id, intent in peer_intents.items():
        if robot_id == ignore_robot_id:
            continue
        cx, cy = grid.world_to_grid(intent.current_x, intent.current_y)
        for dc in range(-DYNAMIC_BLOCK_RADIUS, DYNAMIC_BLOCK_RADIUS + 1):
            for dr in range(-DYNAMIC_BLOCK_RADIUS, DYNAMIC_BLOCK_RADIUS + 1):
                blocked.add((cx + dc, cy + dr))
    return blocked

def _resolve_step(next_cell, next_time, peer_intents, priorities, my_priority_info, now, robot_id):
    """Returns (hard_blocked: bool, soft_cost: float)."""
    soft_cost = 0.0
    for pid, intent in peer_intents.items():
        if pid == robot_id:
            continue
        for pcell, ptime in zip(intent.planned_cells, intent.planned_times):
            if pcell == next_cell and abs(ptime - next_time) <= CONGESTION_WINDOW:
                overlap = CONGESTION_WINDOW - abs(ptime - next_time)
                peer_info = priorities.get(pid) if priorities else None
                if peer_info is not None and my_priority_info is not None:
                    if not wins(my_priority_info, peer_info, now):
                        return True, 0.0
                soft_cost += CONGESTION_PENALTY * overlap
    return False, soft_cost

def _heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def plan_path(start_cell, goal_cell, start_time, robot_id,
              peer_intents: Optional[Dict[str, PeerIntent]] = None,
              priorities: Optional[Dict[str, PriorityInfo]] = None,
              my_priority_info: Optional[PriorityInfo] = None,
              now: float = 0.0) -> Optional[PlanResult]:
    peer_intents = peer_intents or {}
    dynamic_blocked = _dynamic_blocked_cells(peer_intents, robot_id)

    open_heap = []
    counter = 0
    heapq.heappush(open_heap, (0.0, counter, start_cell, start_time))
    came_from = {}
    g_score = {(start_cell, start_time): 0.0}
    visited = set()

    while open_heap:
        _, _, current_cell, current_time = heapq.heappop(open_heap)
        if (current_cell, current_time) in visited:
            continue
        visited.add((current_cell, current_time))

        if current_cell == goal_cell:
            return _reconstruct(came_from, (current_cell, current_time), g_score)

        for next_cell in grid.neighbors(current_cell):
            next_time = current_time + MOVE_TIME

            if next_cell in dynamic_blocked and (next_time - start_time) < DYNAMIC_BLOCK_HORIZON:
                continue

            hard_blocked, soft_cost = _resolve_step(
                next_cell, next_time, peer_intents, priorities, my_priority_info, now, robot_id)
            if hard_blocked:
                continue

            step_cost = MOVE_TIME + soft_cost
            tentative_g = g_score[(current_cell, current_time)] + step_cost
            key = (next_cell, next_time)

            if key not in g_score or tentative_g < g_score[key]:
                g_score[key] = tentative_g
                came_from[key] = (current_cell, current_time)
                f_score = tentative_g + _heuristic(next_cell, goal_cell)
                counter += 1
                heapq.heappush(open_heap, (f_score, counter, next_cell, next_time))

    return None

def _reconstruct(came_from, end_key, g_score) -> PlanResult:
    path_cells, arrival_times = [], []
    key = end_key
    while key in came_from:
        cell, t = key
        path_cells.append(cell)
        arrival_times.append(t)
        key = came_from[key]
    cell, t = key
    path_cells.append(cell)
    arrival_times.append(t)
    path_cells.reverse()
    arrival_times.reverse()
    return PlanResult(path_cells=path_cells, arrival_times=arrival_times, total_cost=g_score[end_key])
