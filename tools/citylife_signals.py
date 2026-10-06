"""Fixed-time traffic signals for the CityLife street grid: one clock, every junction.

WHY

The signal heads in CityLife_Day are static meshes. `BP_CityCar` never reads
them: `UpdateEffSpeed` (tools/citylife_mcp/drive_tick.py) lets a car into a
junction whenever no crossing car is inside the box, so the lights show one
thing and the traffic does another - on camera, a car driving through a red
head is exactly the kind of detail that makes the city look fake.

A signal here is a PURE FUNCTION of game time and the junction's grid position,
not an actor: every car evaluates the same few lines of arithmetic on
`GetGameTimeInSeconds` and its junction centre, so there is no event, no shared
state to keep in step and nothing to wire per junction. The Blueprint mirrors
this file (`dsl_constants()` hands it the numbers), and the lamp materials can
be driven by the same function if the heads are ever made to light up.
tools/citylife_traffic_model.py runs the car logic against it; its tests and
tests/test_citylife_signals.py are the record that the plan is sane.

FRAMES

Unreal cm of this level: X = NORTH, Y = EAST (NED metres x 100). Junction
(i, j) of the street grid sits at UE (4100 + 8200 i, 4100 + 8200 j); the demo
corridor's two junctions are (0, -1) = (4100, -4100) and (0, 0) = (4100, 4100).
'NS' is travel along X, 'EW' travel along Y. In `state()` the axis is the
direction the ASKING CAR TRAVELS; in `walk()` it is the direction the traffic
of the ROAD BEING CROSSED runs.

TIMING (seconds)

    per axis   green 20, yellow 3, all-red 2  -> a 25 s half cycle, C = 50 s
    NS         starts at `offset`             green [0, 20) yellow [20, 23) red [23, 50)
    EW         starts at offset + C/2         i.e. EW green while NS is in [25, 45)

so the two axes are never green together and both are red for 2 s at every
change (23-25 and 48-50 of the NS phase). offset = (11 i + 23 j) mod 50, a
deterministic spread so neighbouring junctions do not switch in step: 11 and 23
are coprime with 50, so the junctions of one street take different offsets.
It is NOT a green wave - at 3.2-5.6 m/s an 82 m block takes 15-26 s, and no
single offset step suits every car - it only stops the whole city turning red
at once, which would bunch every loop into one platoon.

PEDESTRIANS

A pedestrian on a zebra stops the traffic of the road it cuts, so they may only
START while that road's traffic is red - and the standard, safest window is
while the traffic running PARALLEL to them has green: then no car on the road
being crossed can be moving towards them. So for a zebra across a road whose
traffic runs along `road_axis`:

    walk    the first WALK_S = 8 s of the OTHER axis' green
    flash   from 8 s until the other axis' yellow ends (23 s): no new starts
    dont    the all-red and the road's own green/yellow/all-red

A figure that steps out at the last walk instant has G - WALK + Y + all-red =
17 s before the road it is on turns green. Kerb to kerb is 1800 cm (the
carriageway is +-800 cm, `CARRIAGEWAY_HALF_CM`; a figure starts 100 cm back on
the pavement), so it needs >= 106 cm/s; the level's roaming figures have a
MaxWalkSpeed of 67-97 cm/s (docs/FINDING-crowd-pedestrians-and-traffic.md), so
a slow one is still on the zebra when its road turns green. That is safe - the
cars' crossing rule still stops for anyone on the zebra - but it costs the
cars their first seconds of green: give the crossing figures >= 110 cm/s.

Cars TURNING on a green meet the walkers on their exit zebra (they walk in
parallel with that green). That is the permissive turn every real junction
has, and it is the car's rule to yield, not the signal's: see
citylife_traffic_model.py.
"""
from __future__ import annotations

import math
from typing import Dict, FrozenSet, Iterable, Tuple

GREEN_S = 20.0
YELLOW_S = 3.0
ALL_RED_S = 2.0
HALF_CYCLE_S = GREEN_S + YELLOW_S + ALL_RED_S     # 25: one axis' share of the cycle
CYCLE_S = 2.0 * HALF_CYCLE_S                      # 50
WALK_S = 8.0                                      # new pedestrian starts, from the parallel green
OFFSET_I = 11                                     # offset = (11 i + 23 j) mod C
OFFSET_J = 23
GRID_ORIGIN_CM = 4100.0                           # junction (0, 0) at UE (4100, 4100)
GRID_CM = 8200.0                                  # citylife_routes.GRID_CM
GRID_TOL_CM = 1.0                                 # a junction centre is ON the grid, to 1 cm

NS, EW = "NS", "EW"
AXES = (NS, EW)
GREEN, YELLOW, RED = "green", "yellow", "red"
WALK, FLASH, DONT = "walk", "flash", "dont"


def other(axis: str) -> str:
    if axis not in AXES:
        raise ValueError(axis)
    return EW if axis == NS else NS


def axis_of(dx: float, dy: float) -> str:
    """Axis of a heading (dX north, dY east). A tie goes to NS; a car at a stop
    line is on a straight and its heading is within a few degrees of an axis."""
    return NS if abs(dx) >= abs(dy) else EW


def grid_index(x_cm: float, y_cm: float, tol_cm: float = GRID_TOL_CM) -> Tuple[int, int]:
    """(i, j) of the junction centred at UE (x_cm, y_cm). Raises ValueError for a
    point that is not a grid junction: a mistyped centre must not silently get
    some other junction's offset."""
    i = round((x_cm - GRID_ORIGIN_CM) / GRID_CM)
    j = round((y_cm - GRID_ORIGIN_CM) / GRID_CM)
    cx, cy = junction_centre(i, j)
    if abs(cx - x_cm) > tol_cm or abs(cy - y_cm) > tol_cm:
        raise ValueError(f"({x_cm}, {y_cm}) is not a junction of the {GRID_CM:.0f} cm grid")
    return int(i), int(j)


def junction_centre(i: int, j: int) -> Tuple[float, float]:
    return (GRID_ORIGIN_CM + GRID_CM * i, GRID_ORIGIN_CM + GRID_CM * j)


def offset_ij(i: int, j: int) -> float:
    """Start of the NS green, seconds into the cycle. Python's % is never
    negative; UE's integer % keeps the dividend's sign, so the DSL must write
    ((11 i + 23 j) % 50 + 50) % 50."""
    return float((OFFSET_I * i + OFFSET_J * j) % int(CYCLE_S))


def offset(jx: float, jy: float) -> float:
    return offset_ij(*grid_index(jx, jy))


def phase_time(t: float, jx: float, jy: float, axis: str) -> float:
    """Seconds since `axis` last turned green at junction (jx, jy), in [0, C)."""
    if axis not in AXES:
        raise ValueError(axis)
    start = offset(jx, jy) + (0.0 if axis == NS else HALF_CYCLE_S)
    return (t - start) % CYCLE_S


def state_of_phase(p: float) -> str:
    """Signal state `p` seconds into an axis' own phase."""
    if p < GREEN_S:
        return GREEN
    if p < GREEN_S + YELLOW_S:
        return YELLOW
    return RED


def state(t: float, jx: float, jy: float, axis: str) -> str:
    """'green' | 'yellow' | 'red' for a car TRAVELLING along `axis` into the
    junction centred at UE (jx, jy) cm, at game time t (s)."""
    return state_of_phase(phase_time(t, jx, jy, axis))


def ped_phase_of(p_other: float) -> str:
    """Pedestrian phase for a zebra, from the time into the OTHER axis' phase
    (the traffic running parallel to the walkers)."""
    if p_other < WALK_S:
        return WALK
    if p_other < GREEN_S + YELLOW_S:
        return FLASH
    return DONT


def ped_phase(t: float, jx: float, jy: float, road_axis: str) -> str:
    """'walk' | 'flash' | 'dont' on a zebra across the road whose traffic runs
    along `road_axis`, at the junction centred at UE (jx, jy) cm."""
    return ped_phase_of(phase_time(t, jx, jy, other(road_axis)))


def walk(t: float, jx: float, jy: float, road_axis: str) -> bool:
    """True while a pedestrian may START across the road whose traffic runs
    along `road_axis`. Only ever true while that road's traffic is red."""
    return ped_phase(t, jx, jy, road_axis) == WALK


def next_walk_start(t: float, jx: float, jy: float, road_axis: str) -> float:
    """The earliest time >= t at which walk() is true: t itself during a walk
    window, else the start of the next one."""
    p = phase_time(t, jx, jy, other(road_axis))
    return t if p < WALK_S else t + (CYCLE_S - p)


def signalled(jx: float, jy: float, exceptions: Iterable[Tuple[int, int]] = frozenset()) -> bool:
    """Every junction of the level's grid has signals unless its (i, j) is in
    `exceptions`; a point off the grid is not a signalled junction."""
    try:
        ij = grid_index(jx, jy)
    except ValueError:
        return False
    return ij not in frozenset(exceptions)


def crossing_road_axis(cx: float, cy: float, jx: float, jy: float) -> str:
    """Axis of the road a zebra cuts. The zebras sit 1100 cm out along a leg
    of the junction: one displaced north/south of the centre is on the NS road."""
    return NS if abs(cx - jx) >= abs(cy - jy) else EW


def nearest_junction(x: float, y: float) -> Tuple[float, float]:
    """Centre of the grid junction nearest a point (UE cm)."""
    i = round((x - GRID_ORIGIN_CM) / GRID_CM)
    j = round((y - GRID_ORIGIN_CM) / GRID_CM)
    return junction_centre(i, j)


def dsl_constants() -> Dict[str, float]:
    """Every number the Blueprint needs, in one place, for rendering into DSL.
    The DSL computes, for a car travelling along `axis` into junction (jx, jy):
        i = round((jx - GRID_ORIGIN_CM) / GRID_CM), j likewise from jy
        off = ((OFFSET_I*i + OFFSET_J*j) % CYCLE + CYCLE) % CYCLE
        p = fmod(t - off - (axis == EW) * HALF_CYCLE_S + 2*CYCLE_S, CYCLE_S)
        green if p < GREEN_S, yellow if p < GREEN_S + YELLOW_S, else red
    (+ 2*CYCLE_S keeps fmod's argument positive: UE's fmod keeps its sign.)"""
    return {
        "GREEN_S": GREEN_S, "YELLOW_S": YELLOW_S, "ALL_RED_S": ALL_RED_S,
        "HALF_CYCLE_S": HALF_CYCLE_S, "CYCLE_S": CYCLE_S, "WALK_S": WALK_S,
        "OFFSET_I": float(OFFSET_I), "OFFSET_J": float(OFFSET_J),
        "GRID_ORIGIN_CM": GRID_ORIGIN_CM, "GRID_CM": GRID_CM,
    }


def dsl_phase(t: float, jx: float, jy: float, axis: str) -> float:
    """phase_time() computed the way dsl_constants() tells the Blueprint to,
    with C-style (sign-keeping) % and fmod - so a test can hold the two equal."""
    i = int(round((jx - GRID_ORIGIN_CM) / GRID_CM))
    j = int(round((jy - GRID_ORIGIN_CM) / GRID_CM))
    c = int(CYCLE_S)
    off = (int(math.fmod(OFFSET_I * i + OFFSET_J * j, c)) + c) % c
    return math.fmod(t - off - (HALF_CYCLE_S if axis == EW else 0.0) + 2 * CYCLE_S, CYCLE_S)


SIGNAL_EXCEPTIONS: FrozenSet[Tuple[int, int]] = frozenset()   # every grid junction is signalled


if __name__ == "__main__":
    for ij in ((0, -1), (0, 0), (1, -1), (0, 1), (1, 1), (2, 1), (2, 0), (2, -1), (-2, -1)):
        x, y = junction_centre(*ij)
        row = "".join("G" if state(t, x, y, NS) == GREEN else
                      "y" if state(t, x, y, NS) == YELLOW else "." for t in range(50))
        print(f"{str(ij):9s} ({x:7.0f}, {y:7.0f}) offset {offset(x, y):4.0f}  NS {row}")
