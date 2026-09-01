"""
Geometry helpers — the only file that imports shapely.

Keeps geometric truth in one place (the grant's 'single rule-evaluation code
path' invariant, scaled down).
"""
from __future__ import annotations

from shapely.geometry import Point, Polygon

from .models import PolygonFence


def fence_polygon(fence: PolygonFence) -> Polygon:
    return Polygon([(v.x, v.y) for v in fence.vertices])


def point_in_fence(x: float, y: float, up: float, fence: PolygonFence,
                   poly: Polygon | None = None) -> bool:
    """True when the point violates the fence: inside the (margin-buffered)
    polygon AND within its altitude band."""
    if not (fence.altitude_floor_m <= up <= fence.altitude_ceiling_m):
        return False
    poly = poly if poly is not None else fence_polygon(fence)
    return poly.buffer(fence.margin_m).contains(Point(x, y))


def nearest_on_polyline(x: float, y: float,
                        pts: list[tuple[float, float]]
                        ) -> tuple[float, float, float]:
    """(distance, nearest_x, nearest_y) to a polyline - segments, not vertices.

    Measuring to the nearest VERTEX instead is a mistake this project has
    already paid for once: `demo/pedestrians.py` placed its figures that way and
    emptied the first populated flight, because a point halfway along a 48 m
    straight reads as 16 m from the nearest corner while sitting 11 m from the
    line itself. A corridor rule built on vertex distance would fail the same
    way but at safety cost - it would let the vehicle drift out of a corridor
    mid-segment and report it as inside.

    The nearest point comes back with the distance because the corridor repair
    needs the direction home, and recomputing it would be a second chance to
    disagree with the check.
    """
    if not pts:
        return 0.0, x, y
    if len(pts) == 1:
        return ((x - pts[0][0]) ** 2 + (y - pts[0][1]) ** 2) ** 0.5, *pts[0]
    best = (float("inf"), x, y)
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy
        t = 0.0 if L2 < 1e-9 else max(0.0, min(
            1.0, ((x - ax) * vx + (y - ay) * vy) / L2))
        px, py = ax + t * vx, ay + t * vy
        d = ((x - px) ** 2 + (y - py) ** 2) ** 0.5
        if d < best[0]:
            best = (d, px, py)
    return best


def push_out_direction(x: float, y: float, poly: Polygon) -> tuple[float, float]:
    """Unit vector pointing from (x, y) toward the nearest way OUT of poly.
    Used by the repair operator to know which velocity component to remove."""
    p = Point(x, y)
    nearest = poly.exterior.interpolate(poly.exterior.project(p))
    dx, dy = x - nearest.x, y - nearest.y          # boundary -> point vector
    norm = (dx * dx + dy * dy) ** 0.5

    if norm < 1e-9:
        # Dead on the boundary: fall back to "away from centroid".
        c = poly.centroid
        dx, dy = x - c.x, y - c.y
        norm = (dx * dx + dy * dy) ** 0.5 or 1.0
        return dx / norm, dy / norm

    if poly.contains(p):
        # Inside: the way out is TOWARD the nearest boundary point.
        return -dx / norm, -dy / norm
    # Outside: away from the polygon.
    return dx / norm, dy / norm
