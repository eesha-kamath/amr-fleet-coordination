"""
conflict_resolver.py

Pure logic, no ROS. Three responsibilities:

1. priority_key() -- deterministic tiebreak: Lamport timestamp (earlier
   claim wins) + robot_id, with AGING so a robot that keeps losing
   ties eventually wins anyway (prevents starvation / convoy effect).
   Every robot computes this identically from shared broadcast state --
   this replaces an explicit request/reply handshake (Ricart-Agrawala
   normally needs one; we don't, because full state is already shared).

2. Wait-for graph + cycle detection -- classic OS/DB deadlock
   detection, run independently by each robot on its own local view.

3. Victim selection -- lowest-priority robot in a detected cycle backs
   off. This is detection+recovery, not prevention: we don't try to
   rule out every possible conflict upfront, we catch a real deadlock
   the moment it forms and break it with certainty.
"""

from dataclasses import dataclass
from typing import Dict, Optional, List, Tuple

AGING_STEP_SECONDS = 3.0
MAX_AGING_STEPS = 20

@dataclass
class PriorityInfo:
    robot_id: str
    lamport_time: int
    wait_started_at: Optional[float]

def priority_key(info: PriorityInfo, now: float) -> Tuple[int, int, str]:
    """Smaller key wins. (-aging_steps, lamport_time, robot_id)."""
    if info.wait_started_at is None:
        aging_steps = 0
    else:
        waited = max(0.0, now - info.wait_started_at)
        aging_steps = min(MAX_AGING_STEPS, int(waited // AGING_STEP_SECONDS))
    return (-aging_steps, info.lamport_time, info.robot_id)

def wins(my_info: PriorityInfo, peer_info: PriorityInfo, now: float) -> bool:
    """True if I have priority over peer (I do NOT yield)."""
    return priority_key(my_info, now) < priority_key(peer_info, now)

def find_cycle_from(start: str, graph: Dict[str, Optional[str]], max_depth: int = 10) -> Optional[List[str]]:
    """Follow start's wait-for chain; if it loops back to start, that's a deadlock cycle."""
    path = [start]
    current = start
    for _ in range(max_depth):
        nxt = graph.get(current)
        if nxt is None:
            return None
        if nxt == start:
            return path
        if nxt in path:
            return None
        path.append(nxt)
        current = nxt
    return None

def pick_deadlock_victim(cycle: List[str], priorities: Dict[str, PriorityInfo], now: float) -> str:
    """Lowest-priority (largest key) robot in the cycle backs off."""
    return max(cycle, key=lambda rid: priority_key(priorities[rid], now))
