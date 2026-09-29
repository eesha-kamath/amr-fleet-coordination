from dataclasses import dataclass
from math import sqrt


@dataclass
class Footprint:
    shape: str
    length: float
    width: float

    @classmethod
    def from_robot(cls, robot):
        return cls(
            robot.shape,
            robot.length,
            robot.width
        )

    @property
    def radius(self):
        if self.shape == "circle":
            return self.width / 2

        return sqrt(
            (self.length / 2) ** 2 +
            (self.width / 2) ** 2
        )

    def validate(self):
        if self.shape not in ("rectangle", "circle"):
            raise ValueError("Invalid footprint shape")

        if self.length <= 0 or self.width <= 0:
            raise ValueError(
                "Footprint dimensions must be positive"
            )


def distance(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]

    return sqrt(dx * dx + dy * dy)


def surface_gap(
    a,
    b,
    footprint_a,
    footprint_b
):
    return max(
        0.0,
        distance(a, b)
        - footprint_a.radius
        - footprint_b.radius
    )


# True oriented-rectangle geometry (exact surface-to-surface gaps)
from math import cos, sin


def footprint_polygon(x, y, theta, footprint):
    """Corners of the footprint as an oriented rectangle (or a 12-gon for a
    circle). Length runs along the heading."""
    if footprint.shape == "circle":
        r = footprint.width / 2
        return [
            (x + r * cos(i * 3.141592653589793 / 6),
             y + r * sin(i * 3.141592653589793 / 6))
            for i in range(12)
        ]

    hl = footprint.length / 2
    hw = footprint.width / 2
    c = cos(theta)
    s = sin(theta)

    corners = ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))

    return [
        (x + px * c - py * s, y + px * s + py * c)
        for px, py in corners
    ]


def _point_segment_distance(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p

    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy

    if length_sq == 0:
        return sqrt((px - ax) ** 2 + (py - ay) ** 2)

    t = ((px - ax) * dx + (py - ay) * dy) / length_sq
    t = max(0.0, min(1.0, t))

    cx = ax + t * dx
    cy = ay + t * dy

    return sqrt((px - cx) ** 2 + (py - cy) ** 2)


def _separated_on_some_axis(poly_a, poly_b):
    """Separating axis test for convex polygons. True means no overlap."""
    for poly in (poly_a, poly_b):
        n = len(poly)

        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]

            ax = y1 - y2
            ay = x2 - x1

            a_vals = [ax * px + ay * py for px, py in poly_a]
            b_vals = [ax * px + ay * py for px, py in poly_b]

            if max(a_vals) < min(b_vals) or max(b_vals) < min(a_vals):
                return True

    return False


def polygon_gap(poly_a, poly_b):
    """Smallest distance between two convex polygons. 0.0 if they overlap."""
    if not _separated_on_some_axis(poly_a, poly_b):
        return 0.0

    best = float("inf")

    for poly_p, poly_q in ((poly_a, poly_b), (poly_b, poly_a)):
        n = len(poly_q)

        for p in poly_p:
            for i in range(n):
                d = _point_segment_distance(
                    p,
                    poly_q[i],
                    poly_q[(i + 1) % n]
                )

                if d < best:
                    best = d

    return best


def footprint_gap(pose_a, footprint_a, pose_b, footprint_b):
    """True surface-to-surface gap. pose = (x, y, theta)."""
    return polygon_gap(
        footprint_polygon(pose_a[0], pose_a[1], pose_a[2], footprint_a),
        footprint_polygon(pose_b[0], pose_b[1], pose_b[2], footprint_b)
    )