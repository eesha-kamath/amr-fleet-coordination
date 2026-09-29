"""Space-time planning with reservations.

A plan is a list of grid cells, one per time step: plan[k] is where the robot
is at time k * step_time. During step k a robot occupies BOTH plan[k] and
plan[k + 1] (it is in transit between them), so two robots can never share a
cell within the same step and can never swap cells. Following robots are kept
two cells apart, which matches the physical stopping distance.
"""
from heapq import heappush, heappop


class ReservationTable:

    def __init__(self):
        self.paths = []          # plans of higher priority robots
        self.present = set()     # cells physically occupied right now (k = 0)
        self.static = set()      # cells blocked forever (dead robots)
        self._cache = {}

    def add_path(self, path):
        if path:
            self.paths.append(list(path))
            self._cache.clear()

    def add_present(self, cell):
        self.present.add(cell)
        self._cache.clear()

    def add_static(self, cell):
        self.static.add(cell)
        self._cache.clear()

    def occupied(self, k):
        cached = self._cache.get(k)

        if cached is not None:
            return cached

        cells = set(self.static)

        if k == 0:
            cells |= self.present

        for path in self.paths:
            if k + 1 < len(path):
                cells.add(path[k])
                cells.add(path[k + 1])
            else:
                cells.add(path[-1])

        self._cache[k] = cells
        return cells

    def last_reserved_step(self, cell):
        """Latest step at which any reserved path touches the cell."""
        last = -1

        for path in self.paths:
            for k, c in enumerate(path):
                if c == cell:
                    last = max(last, k)

        return last

    def first_conflict(self, plan):
        """First step at which the given plan clashes, or None."""
        for k in range(len(plan)):
            mine = {plan[k]}

            if k + 1 < len(plan):
                mine.add(plan[k + 1])

            if mine & self.occupied(k):
                return k

        return None


class SpaceTimePlanner:

    def __init__(self, horizon=60, max_nodes=25000):
        self.horizon = horizon
        self.max_nodes = max_nodes

    def plan(
        self,
        start,
        goal,
        obstacles,
        grid,
        table,
        wait_penalty=0.01
    ):
        blocked = {(o.x, o.y) for o in obstacles}

        def goal_test(cell, k):
            if cell != goal:
                return False

            # After arriving the robot parks here, so nobody may pass later.
            return table.last_reserved_step(cell) < k and \
                   cell not in table.static

        def heuristic(cell):
            return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

        return self._search(
            start,
            goal_test,
            heuristic,
            blocked,
            grid,
            table,
            wait_penalty
        )

    def plan_evade(
        self,
        start,
        obstacles,
        grid,
        table,
        clear_steps=4,
        inflate=False
    ):
        """Nearest cell that stays unreserved for a few steps. With inflate
        the four neighbouring cells must also stay free."""
        blocked = {(o.x, o.y) for o in obstacles}

        def area(cell):
            if not inflate:
                return {cell}

            x, y = cell
            return {(x, y), (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)}

        def goal_test(cell, k):
            zone = area(cell)

            return all(
                not (zone & table.occupied(j))
                for j in range(k, k + clear_steps)
            )

        return self._search(
            start,
            goal_test,
            lambda cell: 0,
            blocked,
            grid,
            table,
            0.01
        )

    def _search(
        self,
        start,
        goal_test,
        heuristic,
        blocked,
        grid,
        table,
        wait_penalty
    ):
        first = (start, 0)
        q = [(heuristic(start), 0.0, 0, first)]
        cost = {first: 0.0}
        came = {}
        counter = 0
        nodes = 0

        while q:
            _, g, _, cur = heappop(q)
            cell, k = cur

            if g > cost.get(cur, float("inf")):
                continue

            if goal_test(cell, k):
                return self._build(came, cur)

            if k >= self.horizon:
                continue

            nodes += 1

            if nodes > self.max_nodes:
                return []

            x, y = cell
            occ = table.occupied(k)

            for nxt in (
                (x, y),
                (x + 1, y),
                (x - 1, y),
                (x, y + 1),
                (x, y - 1)
            ):
                nx, ny = nxt

                if nx < 0 or ny < 0:
                    continue

                if nx >= grid.width or ny >= grid.height:
                    continue

                if nxt in blocked:
                    continue

                # The robot's true current cell (k == 0, the search root) is
                # a fact, not a plan choice, so a reservation over it can
                # never veto leaving or waiting there. Only the destination
                # cell is checked at k == 0.
                if k == 0:
                    if nxt in occ:
                        continue
                elif cell in occ or nxt in occ:
                    continue

                step_cost = 1.0 + (wait_penalty if nxt == cell else 0.0)
                state = (nxt, k + 1)
                new_cost = g + step_cost

                if new_cost < cost.get(state, float("inf")):
                    cost[state] = new_cost
                    came[state] = cur
                    counter += 1

                    heappush(
                        q,
                        (
                            new_cost + heuristic(nxt),
                            new_cost,
                            counter,
                            state
                        )
                    )

        return []

    def _build(self, came, cur):
        path = [cur[0]]

        while cur in came:
            cur = came[cur]
            path.append(cur[0])

        path.reverse()
        return path