"""Driving paths for the CityLife cars: lanes, filleted corners, speed limits.

WHY THIS IS OFFLINE

`BP_CityCar` used to steer at a corner waypoint and switch to the next one inside
a radius, turning its body with a fixed-rate `RInterpTo`. The yaw rate therefore
had nothing to do with the path: the car pivoted, or swept wide, and the corner
speed came from the distance to a waypoint rather than from how sharp the turn
was. A car moves along an ARC, turning at v/R, and slows for the arc's radius.

That geometry is computed here, once, in plain Python where it can be tested:
each loop becomes a dense polyline (a point every `spacing` cm) whose corners are
circular fillets, plus a speed limit per point. The Blueprint then only has to
follow points it is given (pure pursuit, see `docs/FINDING-crowd-pedestrians-
and-traffic.md`) and cap its speed at the limit ahead.

COORDINATES

Unreal centimetres in THIS level: X is NORTH and Y is EAST (NED x/y scaled by
100). That is a left-handed view from above, so the right-hand side of a heading
(dX, dY) is (-dY, dX), not (dY, -dX). Getting this backwards is how the lanes
were once documented as "keeping left" while the geometry kept right.

TURNS

A turn toward the car's own kerb (a right turn when driving on the right) is the
tight one: it can hug the corner, R about 7 m. A turn across the oncoming lanes
is the wide one, R about 13 m. Both are checked against the junction box.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

Vec = Tuple[float, float]

LANE_OFFSET_CM = 350.0        # lane centre from the road centre line
CARRIAGEWAY_HALF_CM = 800.0   # kerb to road centre line
JUNCTION_HALF_CM = 1100.0     # junction box half size (22 m road tiles)
CAR_HALF_WIDTH_CM = 90.0
R_NEAR_CM = 700.0             # turn toward your own kerb
R_FAR_CM = 1300.0             # turn across the oncoming lanes
SPACING_CM = 150.0
A_LAT_CMS2 = 180.0            # comfortable lateral acceleration, 1.8 m/s^2
A_BRAKE_CMS2 = 150.0          # comfortable braking, 1.5 m/s^2
V_STRAIGHT_CMS = 1500.0       # "no limit" on a straight; the car's own cap rules


def right_of(d: Vec) -> Vec:
    """Right-hand side of heading d, with X = north and Y = east."""
    return (-d[1], d[0])


def left_of(d: Vec) -> Vec:
    return (d[1], -d[0])


def _unit(a: Vec, b: Vec) -> Vec:
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = math.hypot(dx, dy)
    if n == 0:
        raise ValueError(f"repeated junction {a}")
    return (dx / n, dy / n)


def turns_left(d_in: Vec, d_out: Vec) -> bool:
    """True for a left turn. The cross product's sign flips in this frame:
    east (0,1) then north (1,0) is a LEFT turn and gives -1."""
    z = d_in[0] * d_out[1] - d_in[1] * d_out[0]
    return z < 0


@dataclass
class Path:
    name: str
    pts: List[Vec]
    speed: List[float]            # cm/s limit at each point
    curvature: List[float]        # 1/cm, signed: + left, - right
    radii: List[Tuple[str, float]] = field(default_factory=list)  # per corner
    length_cm: float = 0.0


def lane_corners(junctions: Sequence[Vec], drive_side: str = "right",
                 offset: float = LANE_OFFSET_CM) -> List[Vec]:
    """Where consecutive lane centre lines meet, one per junction."""
    if drive_side not in ("right", "left"):
        raise ValueError(drive_side)
    side = right_of if drive_side == "right" else left_of
    n = len(junctions)
    out = []
    for i in range(n):
        j_prev, j, j_next = junctions[i - 1], junctions[i], junctions[(i + 1) % n]
        d_in, d_out = _unit(j_prev, j), _unit(j, j_next)
        o_in, o_out = side(d_in), side(d_out)
        # Point on the incoming lane line: j + o_in*offset + t*d_in; on the
        # outgoing: j + o_out*offset + s*d_out. For perpendicular legs the
        # intersection is j + o_in*offset + o_out*offset.
        if abs(d_in[0] * d_out[0] + d_in[1] * d_out[1]) > 1e-9:
            raise ValueError(f"junction {j}: legs are not perpendicular")
        out.append((j[0] + (o_in[0] + o_out[0]) * offset,
                    j[1] + (o_in[1] + o_out[1]) * offset))
    return out


def build_loop(name: str, junctions: Sequence[Vec], drive_side: str = "right",
               spacing: float = SPACING_CM, r_near: float = R_NEAR_CM,
               r_far: float = R_FAR_CM, a_lat: float = A_LAT_CMS2,
               a_brake: float = A_BRAKE_CMS2) -> Path:
    """Dense lane-centre polyline around the junctions, corners filleted."""
    n = len(junctions)
    corners = lane_corners(junctions, drive_side)
    dirs = [_unit(junctions[i], junctions[(i + 1) % n]) for i in range(n)]

    # One fillet per corner: tangent points, centre, radius, sign.
    fillets = []
    radii = []
    for i in range(n):
        d_in, d_out = dirs[i - 1], dirs[i]
        left = turns_left(d_in, d_out)
        near = (not left) if drive_side == "right" else left
        r = r_near if near else r_far
        radii.append(("near" if near else "far", r))
        c = corners[i]
        t1 = (c[0] - d_in[0] * r, c[1] - d_in[1] * r)   # 90 deg: tan(45) = 1
        t2 = (c[0] + d_out[0] * r, c[1] + d_out[1] * r)
        inward = left_of(d_in) if left else right_of(d_in)
        centre = (t1[0] + inward[0] * r, t1[1] + inward[1] * r)
        fillets.append((t1, t2, centre, r, left))

    pts: List[Vec] = []
    curv: List[float] = []

    def emit_line(a: Vec, b: Vec):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        k = max(1, int(round(d / spacing)))
        for s in range(k):                       # b is emitted by the next piece
            f = s / k
            pts.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
            curv.append(0.0)

    def emit_arc(t1: Vec, t2: Vec, centre: Vec, r: float, left: bool):
        a0 = math.atan2(t1[1] - centre[1], t1[0] - centre[0])
        a1 = math.atan2(t2[1] - centre[1], t2[0] - centre[0])
        sweep = a1 - a0
        # Take the short way round: a fillet never sweeps more than 90 deg.
        sweep = (sweep + math.pi) % (2 * math.pi) - math.pi
        arc_len = abs(sweep) * r
        k = max(2, int(round(arc_len / spacing)))
        sign = 1.0 if left else -1.0
        for s in range(k):
            ang = a0 + sweep * (s / k)
            pts.append((centre[0] + r * math.cos(ang), centre[1] + r * math.sin(ang)))
            curv.append(sign / r)

    for i in range(n):
        t1, t2, centre, r, left = fillets[i]
        emit_arc(t1, t2, centre, r, left)
        nxt = fillets[(i + 1) % n][0]
        emit_line(t2, nxt)

    speed = speed_profile(pts, curv, a_lat, a_brake)
    length = sum(math.hypot(pts[(i + 1) % len(pts)][0] - pts[i][0],
                            pts[(i + 1) % len(pts)][1] - pts[i][1])
                 for i in range(len(pts)))
    return Path(name=name, pts=pts, speed=speed, curvature=curv,
                radii=radii, length_cm=length)


def speed_profile(pts: Sequence[Vec], curv: Sequence[float],
                  a_lat: float = A_LAT_CMS2, a_brake: float = A_BRAKE_CMS2,
                  v_straight: float = V_STRAIGHT_CMS) -> List[float]:
    """Per-point cap: sqrt(a_lat * R) on arcs, then a braking ramp before them.

    The ramp is a backward pass, v_i <= sqrt(v_{i+1}^2 + 2 a d), and the loop is
    closed, so the pass runs twice round to carry a corner's ramp back across
    the seam.
    """
    n = len(pts)
    v = [min(v_straight, math.sqrt(a_lat / abs(k))) if k else v_straight
         for k in curv]
    for _ in range(2):
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            d = math.hypot(pts[j][0] - pts[i][0], pts[j][1] - pts[i][1])
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2.0 * a_brake * d))
    return v


# ---------------------------------------------------------------- the city

# Junction centres on the level's 82 m grid, in driving order. Loops A and B
# share streets in opposite directions and never cross; C is one block west.
LOOPS: Dict[str, List[Vec]] = {
    "A": [(4100.0, -4100.0), (4100.0, 12300.0), (20500.0, 12300.0), (20500.0, -4100.0)],
    "B": [(4100.0, -4100.0), (12300.0, -4100.0), (12300.0, 4100.0), (4100.0, 4100.0)],
    "C": [(-12300.0, -4100.0), (-12300.0, 4100.0), (-4100.0, 4100.0), (-4100.0, -4100.0)],
}


def build_all(drive_side: str = "right", loops: Dict[str, List[Vec]] = None
              ) -> Dict[str, Path]:
    return {k: build_loop(k, v, drive_side) for k, v in (loops or LOOPS).items()}


def lateral_from_centre(p: Vec, junctions: Sequence[Vec]) -> float:
    """Distance from p to the nearest road centre line of this loop's streets.

    The streets are the lines through consecutive junctions, extended; a point
    inside a junction box is measured to whichever centre line is nearer."""
    best = float("inf")
    n = len(junctions)
    for i in range(n):
        a, b = junctions[i], junctions[(i + 1) % n]
        d = _unit(a, b)
        # perpendicular distance to the infinite line through a along d
        dist = abs((p[0] - a[0]) * d[1] - (p[1] - a[1]) * d[0])
        best = min(best, dist)
    return best


def segments_cross(p: Path, q: Path, tol: float = 50.0) -> List[Vec]:
    """Points where two paths come within `tol` cm of each other."""
    out = []
    qs = q.pts
    for a in p.pts[::2]:
        for b in qs[::2]:
            if abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol:
                out.append(a)
                break
    return out


if __name__ == "__main__":
    import json
    import sys
    side = sys.argv[1] if len(sys.argv) > 1 else "right"
    paths = build_all(side)
    for k, p in paths.items():
        print(f"loop {k}: {len(p.pts)} points, {p.length_cm / 100:.0f} m, corners "
              f"{[f'{t} R{r / 100:.0f}m' for t, r in p.radii]}, min speed "
              f"{min(p.speed):.0f} cm/s")
    if "--json" in sys.argv:
        print(json.dumps({k: {"pts": p.pts, "speed": p.speed}
                          for k, p in paths.items()}))
