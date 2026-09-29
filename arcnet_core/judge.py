"""Independent collision judge.

Reads only simulator ground truth (positions and headings). It never reads
planner output, safety levels or coordination state, so the coordination code
cannot influence what counts as a collision.
"""
from .geometry import Footprint, footprint_polygon, polygon_gap


class CollisionJudge:

    def __init__(self, simulator, specs, near_miss_gap=0.15):
        self.sim = simulator
        self.specs = specs
        self.near_miss_gap = near_miss_gap
        self.state = {}          # pair -> "collision" | "near" | "clear"
        self.min_gap = float("inf")

    def _poly(self, robot):
        spec = self.specs[robot.id]

        return footprint_polygon(
            robot.x,
            robot.y,
            robot.theta,
            Footprint.from_robot(spec)
        )

    def check(self):
        """Returns a list of (kind, a, b, gap) edge-triggered events."""
        events = []
        robots = list(self.sim.robots.values())

        polys = {r.id: self._poly(r) for r in robots}

        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                a = robots[i]
                b = robots[j]

                gap = polygon_gap(polys[a.id], polys[b.id])
                self._update((a.id, b.id), gap, events)

        cs = self.sim.grid.cell_size

        for r in robots:
            for o in self.sim.obstacles:
                square = [
                    (o.x * cs, o.y * cs),
                    ((o.x + 1) * cs, o.y * cs),
                    ((o.x + 1) * cs, (o.y + 1) * cs),
                    (o.x * cs, (o.y + 1) * cs)
                ]

                gap = polygon_gap(polys[r.id], square)

                if gap < self.near_miss_gap:
                    self._update((r.id, f"cell{o.x},{o.y}"), gap, events)
                else:
                    self.state[(r.id, f"cell{o.x},{o.y}")] = "clear"

        return events

    def _update(self, pair, gap, events):
        self.min_gap = min(self.min_gap, gap)
        previous = self.state.get(pair, "clear")

        if gap <= 0.0:
            new = "collision"
        elif gap < self.near_miss_gap:
            new = "near"
        else:
            new = "clear"

        if new != previous:
            if new == "collision":
                events.append(("collision", pair[0], pair[1], gap))
            elif new == "near" and previous == "clear":
                events.append(("near_miss", pair[0], pair[1], gap))

        self.state[pair] = new