"""
Geometry helpers — with guardrail/ir.py, the only guardrail files that import
shapely. (models.py reaches shapely only through `keepout_area` and
`corridor_tube` below, imported lazily by its layer merge, so the models stay
importable without it.)

Keeps geometric truth in one place (the grant's 'single rule-evaluation code
path' invariant, scaled down). ir.py does not add a second rule: it builds
`fence_polygon(f).buffer(f.margin_m)` once per fence and hands that ring back to
`point_in_fence` here, so the containment test itself still lives only in this
file.

A circle_fence is a PolygonFence whose vertices are derived (the 32-gon
circumscribing the circle, models.CircleFence), so every helper here handles it
unchanged.
"""
from __future__ import annotations

import shapely
from shapely.geometry import LineString, Point, Polygon

from .models import PolygonFence
from .planar import point_in_ring_xy, ring_edge_distance, segment_distance  # noqa: F401  (re-exported)

# --------------------------------------------------------------------------- #
# The point and segment measures every reader shares (2026-11-17, WP3-22/X-13)
# --------------------------------------------------------------------------- #
#
# "Is this point inside this ring?", "how far is it from it?", "which part of
# this segment lies inside it?" - the Shield's verdicts (guardrail/shield.py
# zone_breached, ZoneRuler, the segment test) and every reader outside it (the
# RTL leg, the stress harness's polygon measures, the BC pilots' input
# features, the compiler's lint and cost map, the analysis tools) call these
# functions instead of asking shapely themselves.
# tests/test_shieldcore_onepath.py fails when a function outside the library
# (this file, planar.py, shield.py, ir.py) calls a shapely predicate or
# distance, or carries the shape of a hand-written containment or
# point-to-segment test - whatever the function is called.
#
# Each one is the shapely call it wraps, unchanged: the open interior for
# `ring_contains` (a point on the edge is outside, as the Shield has always
# read a zone), the closed set for `area_covers` / `area_meets`.


def ring_contains(ring, x: float, y: float) -> bool:
    """Is (x, y) in the open interior of `ring` (a shapely polygon)? The
    footprint half of a keep-out verdict (point_in_fence adds the band)."""
    return ring.contains(Point(x, y))


def ring_distance(ring, x: float, y: float) -> float:
    """Metres from (x, y) to `ring`: 0 inside it, else to its edge (the
    measure Shield.fence_distance reports)."""
    return ring.distance(Point(x, y))


def boundary_depth(ring, x: float, y: float) -> float:
    """Metres from (x, y) to the boundary of `ring` (every ring, holes
    included): how deep a point inside it is."""
    return ring.boundary.distance(Point(x, y))


def edge_distance(poly, x: float, y: float) -> float:
    """Metres from (x, y) to the OUTER edge of `poly` (its exterior ring),
    inside or out."""
    return poly.exterior.distance(Point(x, y))


def edge_gap(poly, geom) -> float:
    """Metres from the OUTER edge of `poly` to `geom` (a line, a path)."""
    return poly.exterior.distance(geom)


def signed_edge_distance(poly, x: float, y: float) -> float:
    """`edge_distance`, negative inside `poly` (its open interior): the stress
    harness's polygon measure (experiments/sweep_scenarios.py)."""
    p = Point(x, y)
    d = poly.exterior.distance(p)
    return -d if poly.contains(p) else d


def nearest_edge_point(poly, x: float, y: float) -> tuple[float, float]:
    """The point of `poly`'s outer edge nearest (x, y)."""
    p = Point(x, y)
    q = poly.exterior.interpolate(poly.exterior.project(p))
    return q.x, q.y


def nearest_boundary_point(area, x: float, y: float) -> tuple[float, float]:
    """The point of `area`'s boundary (every ring of every part) nearest
    (x, y)."""
    from shapely.ops import nearest_points
    q = nearest_points(area.boundary, Point(x, y))[0]
    return q.x, q.y


def segment_in_ring(ring, x0: float, y0: float, x1: float, y1: float):
    """The part of the straight segment (x0, y0)-(x1, y1) that lies in the
    CLOSED `ring` (shapely geometry; empty when it misses)."""
    return LineString([(x0, y0), (x1, y1)]).intersection(ring)


def enters_interior(ring, geom) -> bool:
    """Does `geom` (a segment, a path) pass through the open interior of
    `ring`? A path that only runs along the edge does not."""
    return ring.relate_pattern(geom, "T********")


def area_covers(area, geom) -> bool:
    """Does the closed `area` hold every point of `geom`?"""
    return area.covers(geom)


def area_meets(area, geom) -> bool:
    """Do the closed `area` and `geom` share at least one point?"""
    return area.intersects(geom)


def area_holds_inside(area, geom) -> bool:
    """Does the open interior of `area` hold `geom` (none of it on the
    edge)?"""
    return area.contains_properly(geom)


def contains_many(area, points):
    """`ring_contains` for a shapely point array (numpy bool array)."""
    return shapely.contains(area, points)


def covers_many(area, geoms):
    """`area_covers` for an array of geometries (numpy bool array)."""
    return shapely.covers(area, geoms)


def meets_many(area, geoms):
    """`area_meets` for an array of geometries (numpy bool array)."""
    return shapely.intersects(area, geoms)


def distance_many(geom, points):
    """Metres from `geom` to each of an array of points (numpy array)."""
    return shapely.distance(geom, points)


def point_distance(geom, x: float, y: float) -> float:
    """Metres from (x, y) to any shapely geometry (a line, a point, a
    union of edges)."""
    return geom.distance(Point(x, y))


def fence_polygon(fence: PolygonFence) -> Polygon:
    return Polygon([(v.x, v.y) for v in fence.vertices])


def keepout_area(fence: PolygonFence) -> Polygon:
    """The area a keep-out fence forbids: its polygon grown by its margin ring
    (a negative margin is clamped to 0 - the lint refuses one anyway). Used by
    the layer merge to decide whether an override is at least as strict."""
    return fence_polygon(fence).buffer(max(fence.margin_m, 0.0))


def corridor_tube(corridor) -> Polygon:
    """The area a keep-in corridor allows: every point within half its width
    of the centerline SEGMENTS (round caps), which is exactly the set the
    Shield's distance-to-segments check accepts."""
    return LineString(corridor.points()).buffer(corridor.half_width_m)


def point_in_fence(x: float, y: float, up: float, fence: PolygonFence,
                   poly: Polygon | None = None,
                   buffered: Polygon | None = None) -> bool:
    """True when the point violates the fence: inside the (margin-buffered)
    polygon AND within its altitude band.

    `buffered` is the margin ring built ONCE (guardrail/ir.py compiles it per
    fence). Without it the ring is rebuilt here on every call - correct, and
    what every caller did until 2026-10-06, but at 30-45 us a call it was most
    of the Shield's time: one 50-rule check could make 384 of these (48 fences x
    the current pose and 7 forecast poses). The rebuild path is
    kept for callers holding only a fence, and gives the same answer because
    buffer() is a pure function of (polygon, margin)."""
    if not (fence.altitude_floor_m <= up <= fence.altitude_ceiling_m):
        return False
    if buffered is None:
        poly = poly if poly is not None else fence_polygon(fence)
        buffered = poly.buffer(fence.margin_m)
    return ring_contains(buffered, x, y)


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
