"""The shapely-free half of the one geometry library (guardrail/geometry.py
re-exports it): even-odd containment and point-to-segment distance on plain
vertex lists, for the modules that must import without shapely
(guardrail/models.py's lint, guardrail/fence_export.py on a rail) and for the
compiler's mission-path measures.

Until 2026-11-17 (cards WP3-22 / X-13) these were four private copies - the
compiler's `_point_in_poly` / `_pt_seg`, fence_export's `_point_in_polygon` /
`_segment_distance` and models' `_point_in_ring` / `_dist_to_ring` - each a
line-for-line twin of the others. They are defined here once and the copies
call them, with the same arithmetic (tests/test_shieldcore_onepath.py holds
the old copies as nulls and checks the answers are bit-identical).
"""
from __future__ import annotations

import math


def point_in_ring_xy(x: float, y: float, verts) -> bool:
    """Even-odd (ray-casting) containment of (x, y) in the closed ring
    `verts` (a sequence of (x, y), not repeated at the end). A point exactly on
    an edge may read either way; a caller that needs the boundary counted
    tests it first (models._point_in_ring)."""
    inside = False
    n = len(verts)
    for i in range(n):
        (x1, y1), (x2, y2) = verts[i], verts[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xc = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xc:
                inside = not inside
    return inside


def segment_distance(px: float, py: float, ax: float, ay: float,
                     bx: float, by: float, eps2: float | None = None) -> float:
    """Metres from (px, py) to the segment a-b (its nearest point, not the
    nearest vertex). A segment whose squared length is below `eps2` (None:
    only an exact repeat, length 0) is read as the point a."""
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    degenerate = L2 == 0 if eps2 is None else L2 < eps2
    t = 0.0 if degenerate else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def ring_edge_distance(x: float, y: float, verts, eps2: float | None = None) -> float:
    """Metres from (x, y) to the nearest edge of the closed ring `verts`
    (inf for an empty ring)."""
    best = float("inf")
    n = len(verts)
    for i in range(n):
        (ax, ay), (bx, by) = verts[i], verts[(i + 1) % n]
        best = min(best, segment_distance(x, y, ax, ay, bx, by, eps2))
    return best
