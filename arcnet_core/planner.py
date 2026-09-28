from heapq import heappush, heappop


class PathPlanner:

    def plan(self, start, goal, obstacles, grid):
        return self._astar(start, goal, obstacles, grid)

    def plan_predictive(
        self,
        start,
        goal,
        obstacles,
        grid,
        peer_paths=None,
        start_time=0
    ):
        peer_paths = peer_paths or {}

        q = []
        first = (start[0], start[1], start_time)

        heappush(q, (0, first))

        came_from = {}
        cost = {first: 0}

        while q:
            _, cur = heappop(q)

            x, y, t = cur

            if (x, y) == goal:
                return self._build_time_path(came_from, cur)

            nt = t + 1

            moves = (
                (x + 1, y),
                (x - 1, y),
                (x, y + 1),
                (x, y - 1),
                (x, y)
            )

            for nx, ny in moves:

                if nx < 0 or ny < 0:
                    continue

                if nx >= grid.width or ny >= grid.height:
                    continue

                if self._blocked(obstacles, nx, ny):
                    continue

                if self._peer_conflict(
                    (nx, ny),
                    (x, y),
                    nt,
                    peer_paths
                ):
                    continue

                nxt = (nx, ny, nt)
                nc = cost[cur] + 1

                if nxt not in cost or nc < cost[nxt]:
                    cost[nxt] = nc

                    h = abs(nx - goal[0]) + abs(ny - goal[1])

                    heappush(
                        q,
                        (nc + h, nxt)
                    )

                    came_from[nxt] = cur

        return []

    def _astar(self, start, goal, obstacles, grid):
        q = []
        heappush(q, (0, start))

        came_from = {}
        cost = {start: 0}

        while q:
            _, cur = heappop(q)

            if cur == goal:
                return self._build_path(came_from, cur)

            x, y = cur

            for nx, ny in (
                (x + 1, y),
                (x - 1, y),
                (x, y + 1),
                (x, y - 1)
            ):
                if nx < 0 or ny < 0:
                    continue

                if nx >= grid.width or ny >= grid.height:
                    continue

                if self._blocked(obstacles, nx, ny):
                    continue

                nc = cost[cur] + 1

                if (nx, ny) not in cost or nc < cost[(nx, ny)]:
                    cost[(nx, ny)] = nc

                    h = abs(nx - goal[0]) + abs(ny - goal[1])

                    heappush(
                        q,
                        (nc + h, (nx, ny))
                    )

                    came_from[(nx, ny)] = cur

        return []

    def _peer_conflict(
        self,
        position,
        previous,
        timestamp,
        peer_paths
    ):
        for path in peer_paths.values():

            if not path:
                continue

            if timestamp < len(path):
                peer_position = path[timestamp]

                if position == peer_position:
                    return True

            if timestamp > 0 and timestamp - 1 < len(path):
                peer_position = path[timestamp - 1]

                if position == peer_position and previous == peer_position:
                    return True

        return False

    def _blocked(self, obstacles, x, y):
        return any(
            o.x == x and o.y == y
            for o in obstacles
        )

    def _build_path(self, came_from, cur):
        path = [cur]

        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)

        path.reverse()
        return path

    def _build_time_path(self, came_from, cur):
        path = []

        while True:
            path.append((cur[0], cur[1]))

            if cur not in came_from:
                break

            cur = came_from[cur]

        path.reverse()
        return path