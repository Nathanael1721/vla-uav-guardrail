"""Driving paths for the CityLife cars: lanes, filleted corners, speed limits.

WHY THIS IS OFFLINE

`BP_CityCar` used to steer at a corner waypoint and switch to the next one inside
a radius, turning its body with a fixed-rate `RInterpTo`. The yaw rate therefore
had nothing to do with the path: the car pivoted, or swept wide, and the corner
speed came from the distance to a waypoint rather than from how sharp the turn
was. A car moves along an ARC, turning at v/R, and slows for the arc's radius.

That geometry is computed here, once, in plain Python where it can be tested:
each loop becomes a dense polyline (a point every `spacing` cm) whose corners are
circular fillets, plus a speed limit and a curvature per point. The Blueprint
then only has to follow what it is given, and `follow_step` below is the
reference for how: the SAME arithmetic `BP_CityCar.DriveTick` runs, kept here so
that its accuracy is a test and not an impression from a video.

HOW A CAR FOLLOWS ITS PATH

Pure pursuit was tried first and cut every 7 m corner by 66 cm at 3.2 m/s^2:
its look-ahead point enters the arc early, so the car turns early. What works is
to steer by the path's own curvature (feed-forward: on an arc the car turns at
exactly v/R) and correct only the small residual, lateral offset and heading
error, as a second-order system in DISTANCE, not time - so its behaviour does
not depend on speed or frame rate:

    kappa = kappa_path - e_y / L^2 - 2 zeta e_psi / L,     L = 5 m, zeta = 0.9

Three details matter as much as the law. The car's segment advances only once
the car has PASSED a point (a 120 cm "arrived" radius started every turn 120 cm
early). The heading error is taken against the arc's tangent, not the chord
between two points: a 12.9 deg arc step otherwise reads as a 6 deg error at the
start of every corner and the correction alone adds 30 % to the lateral
acceleration. And a long frame is cut into steps of at most 50 ms: the editor
hitches, and one 0.5 s frame at an arc's end drives 1.8 m on the wrong
curvature - measured in Simulate as 56 cm off the lane with the curvature
pinned at its clamp, exactly what `follow` without sub-steps reproduces.

COORDINATES

Unreal centimetres in THIS level: X is NORTH and Y is EAST (NED x/y scaled by
100). That is a left-handed view from above, so the right-hand side of a heading
(dX, dY) is (-dY, dX), not (dY, -dX). Getting this backwards is how the lanes
were once documented as "keeping left" while the geometry kept right.

WHICH SIDE

LEFT, and that is read off the road, not assumed from the city's name. A
top-down capture of the western approach to junction (4100, 4100) on 2026-09-23
shows the stop line and the lane arrows (straight, straight-and-right) on the
NORTH half of the carriageway, pointing east: eastbound traffic keeps to the
north, which is the driver's left. Every car before that date drove on the
other side, against the markings.

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
DRIVE_SIDE = "left"           # read off the lane markings; see WHICH SIDE


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


def lane_corners(junctions: Sequence[Vec], drive_side: str = DRIVE_SIDE,
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


def build_loop(name: str, junctions: Sequence[Vec], drive_side: str = DRIVE_SIDE,
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


GRID_CM = 8200.0              # junction spacing of the level's street grid


FOLLOW_L_CM = 500.0           # distance constant of the path-error correction
FOLLOW_ZETA = 0.9
KAPPA_MAX = 1.0 / 500.0       # never tighter than a 5 m radius
ACCEL_CMS2 = 200.0
DECEL_CMS2 = 400.0
STOP_DECEL_CMS2 = 250.0       # the ramp a stop line or a queue is approached on


@dataclass
class CarState:
    x: float
    y: float
    yaw_deg: float                # Unreal yaw: 0 = +X (north), 90 = +Y (east)
    v: float = 0.0
    idx: int = 1                  # the car is on segment idx-1 -> idx


def follow_step(path: Path, s: CarState, dt: float, v_cap: float = V_STRAIGHT_CMS,
                stop_cm: float = None) -> float:
    """Advance one car one tick; returns the curvature it drove (1/cm, + right).

    Mirrors `BP_CityCar.DriveTick` line for line. Sign convention is the car's:
    Unreal yaw grows clockwise seen from above (north -> east is a RIGHT turn),
    so a left turn has negative curvature here although `Path.curvature` stores
    it positive.
    """
    pts, n = path.pts, len(path.pts)
    yaw = math.radians(s.yaw_deg)
    f = (math.cos(yaw), math.sin(yaw))
    for _ in range(4):                               # passed the point? next one
        d = (pts[s.idx][0] - s.x, pts[s.idx][1] - s.y)
        if d[0] * f[0] + d[1] * f[1] <= 0.0:
            s.idx = (s.idx + 1) % n
    i0 = (s.idx - 1) % n
    a, b = pts[i0], pts[s.idx]
    seg = math.hypot(b[0] - a[0], b[1] - a[1])
    t = ((b[0] - a[0]) / seg, (b[1] - a[1]) / seg)
    nrm = (-t[1], t[0])                              # right of the path
    rx, ry = s.x - a[0], s.y - a[1]
    e_y = rx * nrm[0] + ry * nrm[1]                  # + : car right of path
    k_ff = -path.curvature[i0]
    along = rx * t[0] + ry * t[1]
    e_psi = (f[0] * nrm[0] + f[1] * nrm[1]) - (along - 0.5 * seg) * k_ff
    kap = k_ff - e_y / FOLLOW_L_CM ** 2 - 2.0 * FOLLOW_ZETA * e_psi / FOLLOW_L_CM
    kap = max(-KAPPA_MAX, min(KAPPA_MAX, kap))
    target = min(v_cap, path.speed[i0], path.speed[(s.idx + 2) % n])
    if stop_cm is not None:
        target = min(target, math.sqrt(2.0 * STOP_DECEL_CMS2 * max(0.0, stop_cm)))
    s.v += max(-DECEL_CMS2 * dt, min(ACCEL_CMS2 * dt, target - s.v))
    s.yaw_deg += math.degrees(s.v * kap * dt)
    yaw = math.radians(s.yaw_deg)
    s.x += math.cos(yaw) * s.v * dt
    s.y += math.sin(yaw) * s.v * dt
    return kap


MAX_STEP_S = 0.05


def follow(path: Path, s: CarState, dt: float, v_cap: float = V_STRAIGHT_CMS,
           stop_cm: float = None) -> float:
    """One frame, cut into sub-steps of at most MAX_STEP_S - as DriveTick does.
    Returns the largest |curvature| driven in the frame."""
    n = min(20, int(dt / MAX_STEP_S) + 1)
    worst = 0.0
    for _ in range(n):
        k = follow_step(path, s, dt / n, v_cap, stop_cm)
        worst = k if abs(k) > abs(worst) else worst
    return worst


def distance_to_path(path: Path, x: float, y: float) -> float:
    pts, n = path.pts, len(path.pts)
    best = float("inf")
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        dx, dy = b[0] - a[0], b[1] - a[1]
        u = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(x - a[0] - u * dx, y - a[1] - u * dy))
    return best


def build_all(drive_side: str = DRIVE_SIDE, loops: Dict[str, List[Vec]] = None
              ) -> Dict[str, Path]:
    return {k: build_loop(k, v, drive_side) for k, v in (loops or LOOPS).items()}


def junctions_on(junctions: Sequence[Vec], grid: float = GRID_CM) -> List[Vec]:
    """Every junction a loop drives through, in order: its corners AND the ones
    it crosses straight over. A loop's corner list alone misses the second kind,
    and that is where another loop's turn can cut across it."""
    out: List[Vec] = []
    n = len(junctions)
    for i in range(n):
        a, b = junctions[i], junctions[(i + 1) % n]
        steps = int(round(math.hypot(b[0] - a[0], b[1] - a[1]) / grid))
        for s in range(steps):
            out.append((a[0] + (b[0] - a[0]) * s / steps, a[1] + (b[1] - a[1]) * s / steps))
    return out


# The pedestrian crossings cut in the no-walk bands on 2026-09-22 (crossings.py):
# 1100 cm either side of the two junctions the demo corridor runs through.
CROSSINGS: List[Vec] = [(4100.0, -3000.0), (4100.0, 3000.0), (4100.0, 5200.0),
                        (3000.0, -4100.0), (5200.0, -4100.0),
                        (3000.0, 4100.0), (5200.0, 4100.0)]


def crossings_on(path: Path, crossings: Sequence[Vec] = CROSSINGS,
                 reach: float = 450.0) -> List[Tuple[float, float, int]]:
    """The crossings this path actually drives over, as (x, y, path index).

    A car must judge "is there a crossing ahead of me" along its PATH, not
    along its current heading: a car about to turn left would otherwise stop
    for the crossing straight ahead that it never reaches, and ignore the one
    round the corner that it does. The index lets the car count the distance
    in path points. `reach` is the lane offset plus slack: a lane centre passes
    350 cm from the crossing's centre line."""
    out = []
    for c in crossings:
        i = min(range(len(path.pts)),
                key=lambda k: math.hypot(path.pts[k][0] - c[0], path.pts[k][1] - c[1]))
        if math.hypot(path.pts[i][0] - c[0], path.pts[i][1] - c[1]) <= reach:
            out.append((c[0], c[1], i))
    return out


def give_way_junctions(paths: Dict[str, Path], loops: Dict[str, List[Vec]] = None,
                       tol: float = 300.0) -> Dict[str, set]:
    """Per loop, the junctions where it must give way: where its path crosses
    another loop's path, and it is the one turning across (a far turn).

    Only real crossings count. Giving way everywhere a loop turns across made
    B wait at (4100, -4100), where its path never meets A's, behind a queue
    that was itself waiting - traffic that stands still for nothing."""
    loops = loops or LOOPS
    names = sorted(paths)
    out = {k: set() for k in names}
    for a in names:
        corners = loops[a]
        far = {corners[i] for i, (kind, _) in enumerate(paths[a].radii) if kind == "far"}
        for b in names:
            if a == b:
                continue
            for pt in segments_cross(paths[a], paths[b], tol=tol):
                for j in far:
                    if abs(pt[0] - j[0]) <= JUNCTION_HALF_CM and abs(pt[1] - j[1]) <= JUNCTION_HALF_CM:
                        out[a].add(j)
    return out


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
    side = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else DRIVE_SIDE
    paths = build_all(side)
    for k, p in paths.items():
        print(f"loop {k}: {len(p.pts)} points, {p.length_cm / 100:.0f} m, corners "
              f"{[f'{t} R{r / 100:.0f}m' for t, r in p.radii]}, min speed "
              f"{min(p.speed):.0f} cm/s")
    if "--json" in sys.argv:
        # kappa in the CAR's sign (+ right), which is what DriveTick adds to yaw
        print(json.dumps({k: {"pts": p.pts, "speed": p.speed,
                              "kappa": [-c for c in p.curvature],
                              "junctions": junctions_on(LOOPS[k])}
                          for k, p in paths.items()}))
