"""Reference model of the CityLife traffic WITH signals: DriveTick + UpdateEffSpeed in Python.

    python -m tools.citylife_traffic_model                    # 30 min, 3/10/30 fps, both modes
    python -m tools.citylife_traffic_model --minutes 5 --fps 10 --mode legacy

WHY

The car logic runs in one place, the Blueprint DSL of
tools/citylife_mcp/drive_tick.py, and the only way to watch it is a Simulate
session in the editor. The crossing rule took four review rounds there, and
every hole was found by an emulator (scratchpad emu_r4.py), not by watching.
So the signal logic is written HERE first, measured over 30 simulated minutes
at 3, 10 and 30 fps, and the DSL then mirrors this file node for node.

`mode="legacy"` is today's DSL exactly (no signals, give-way stop at
JEdge - 300), so before and after are measured by the same code. It
reproduces both problems this file exists to fix:

  (a) the signal heads are static meshes; the cars ignore them.
  (b) a car waiting at a junction stops with its front half ON the zebra:
      JEdge - 300 puts its centre 1400 cm from the junction centre and its
      front at 1170, inside the band 800-1400 (CROSSINGS sit 1100 +- 300 out).
      Legacy, 30 min at 10 fps: 729 car-seconds, 183 waits. Mostly (631 s) it
      is the BOX rule, not give-way: a platoon leader whose heading passes
      45 deg in a turn counts as "non-parallel", and every car behind it
      stopped at JEdge - 300.

THE NEW RULES (mode="signals"; DriveTick and the crossing rules unchanged)

  signal      tools/citylife_signals.py: state(t, junction, axis of travel),
              t = the frame's game time.
  stop line   STOP_CM = zebra outer edge 1400 + 100 + car half length 230 =
              1730 cm from the junction centre along the approach; d_line =
              (along-distance to the centre) - STOP_CM. The CENTRE at the line
              puts the front 100 cm short of the zebra.
  red         StopCm = min(StopCm, d_line) while d_line > -50; past that the
              car is committed and proceeds.
  yellow      stop only if d_line >= v^2 / (2*250) - 50.
  green       hold at the line (same target) for
                box      a non-parallel car INSIDE the box - except one heading
                         into my exit quarter (the car I follow through the
                         turn) or an oncoming car 7 m past the centre;
                giveway  at a loop's give-way junction (Junctions z = 1): an
                         ONCOMING car 1100-3600 cm out (a permissive turn).
                         Cross traffic is the signal's business; counting it,
                         as the old rule did, holds a turner for a car that is
                         itself waiting at red;
                exit     keep the box clear: a car on my EXIT lane (within
                         250 cm of it, heading within 60 deg of it) from the
                         box entry to EXIT_CLEAR_CM = 1400 + 230 + 650 = 2280
                         past the centre, slower than 150 cm/s. The spec's
                         2200 + 650 + d_to_box leaves the waiting car on the
                         EXIT zebra; and without the speed test (box_rule=
                         "static") a queue discharges one car per ~10 s:
                         5 of loop B's cars stood > 120 s and Car_10's lap
                         went to 290 s;
                exitped  a walker on my exit zebra (+-1000 cm, so one just off
                         the kerb counts; scanned up to 24 path points ahead,
                         since a 13 m turn's exit zebra is ~20 past its line),
                         only in the first WALK_S of my green - while walkers
                         may still start. Holding for the whole green starved
                         loop A's turn at (4100, -4100) for 127 s; after 8 s
                         the car pulls in and the crossing rule stops it at
                         the zebra, which guarantees a turner per cycle.
              A green hold needs the car able to stop at the line at 4 m/s^2
              (d_line >= v^2/800 - 50). If it cannot: box falls back to the old
              target JEdge - 300 (`late_hold`, safety first); giveway, exit and
              exitped commit (an oncoming car that entered the window after I
              started sees me in the box and stops at its own line).
  sneak       a turner held >= 4 s on green by giveway/box goes in the first
              1.5 s of yellow if no oncoming car is unable to stop (d_line_o <
              v_o^2/500 - 50 + 100) and box/exit are clear: every real
              permissive turn relies on it. Without it B's right turn at
              (12300, -4100) waited 146 s behind loop A's platoon. A sneaker
              held again while it can still stop at its line (an oncoming car
              that could not stop turned up) drops the sneak (SneakJ = -1):
              kept, it ignored the red once that car had passed.
  queue       a same-way car within ARC_LEADER_CM = 1300 (was 900), and a car
              up to 90 deg round a bend inside my corridor, are queue leaders:
              the old rule only slowed for the latter (EffSpeed floor
              60 cm/s) and crept to 543-600 cm behind a car waiting in a 13 m
              turn (legacy MinGap 356 cm at 3 fps).
  unsignalled junctions (only with `exceptions`): the old conflict rule with
              d_line for JEdge - 300. It sees crossing traffic only once it is
              IN the box, too late to stop 330 cm further back: with every
              junction excepted, 84 s of ZebraWait and 464 late holds in
              30 min. No junction of the level is unsignalled; do not except
              one without an approach-time rule.

MEASURED (30 simulated minutes, all 24 cars, seed 1, 1 crosser/min/zebra)

                RedViol ZebraWait BoxWait(>2 s)  MinGap MaxStop deadlk Car_10 lap
    signals  3      0     0 s     12 / 66 s       650    63 s     0    248.0 s
    signals 10      0     0 s     14 / 56 s       650    46 s     0    247.5 s
    signals 30      0     0 s     12 / 55 s       650    64 s     0    247.7 s
    legacy   3      -   750 s     65 / 383 s      356    93 s     0    200.8 s
    legacy  10      -   729 s     52 / 370 s      592    89 s     0    200.7 s
    legacy  30      -   633 s     50 / 355 s      638    88 s     0    202.3 s

Every box wait with signals is a turner yielding to walkers inside the box
(max 14 s). Across seeds 1-4 at 1 and 2 crossers/min/zebra and 3/10/30 fps
(24 runs) every gate held: RedViol 0, ZebraWait 0, MinGap >= 649.7 cm,
MaxStop <= 96 s. Signals cost Car_10 ~47 s a lap (+23 %, 8 junctions);
lap times are the mean of laps 2+ (lap 1 starts from rest).

WHAT THE DSL NEEDS BEYOND TODAY'S

  - the signal phase (citylife_signals.dsl_constants, GetGameTimeInSeconds);
  - per car a JExit array parallel to Junctions: `jexit_rows()`;
  - the OTHER car's CurSpeed (exit and sneak rules), read on the BP_CityCar
    with the same `:self _car` target the DSL uses for GetActorLocation - a
    car moved by SetActorLocation reports GetVelocity 0;
  - new members HeldTurn (float), SneakJ (int), and the phase time;
  - `dsl_constants()` below for every number.

FRAMES

Unreal cm of this level: X = NORTH, Y = EAST; yaw 0 = +X, clockwise (90 = +Y);
the right of heading (fx, fy) is (-fy, fx). Times in seconds of game time.
Every car runs UpdateEffSpeed (every other tick) then DriveTick, in name
order, reading the cars before it at their new positions - as UE ticks actors
one after another. A stop line crossed in a frame is judged against the
signal at that frame's time: that is what the car decided on and what a
rendered frame shows.

PEDESTRIANS (crude, deterministic): per zebra a Poisson stream of crossers
(`ped_rate`, seeded), each waiting on the pavement 900 cm from the road centre
until citylife_signals.walk() and then crossing kerb to kerb at a fixed
110-150 cm/s, without looking at cars. Waiting figures are outside the car's
zebra box (+-800 cm), so the 6 s standing-figure rule is carried but idle
here: emu_r4.py tests it. `ped_hits` counts a walker inside a car MOVING above
50 cm/s; a crude figure walking into a car stopped on the zebra is not a hit.

METRICS (`simulate`)

  red_viol    a car's front crossed its stop line (by > 5 cm) while red
  zebra_wait  car-seconds stopped with the body overlapping a zebra band while
              held by a signal/give-way/box/exit rule; zebra_stop_any counts
              every reason but the emergency stop, split in zebra_stop_why
  box_wait    stops inside a junction box longer than 2 s (episodes, seconds)
  min_gap     closest centre distance between any two cars
  max_stop    longest continuous stand-still (< 10 cm/s) of any car
  deadlocks   cars that stood still > 120 s at once
  car10_laps  Car_10's lap times (loop A, 320 cm/s)
"""
from __future__ import annotations

import argparse
import math
import random
import time
from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Optional, Sequence, Tuple

import numpy as np

from tools import citylife_routes as R
from tools import citylife_signals as S
from tools.citylife_mcp.apply_routes import SUBJECT, plan

# --- a junction approach, cm from the junction centre measured along the approach
BOX_HALF_CM = R.JUNCTION_HALF_CM                              # 1100
ZEBRA_CENTRE_CM = 1100.0                                      # CROSSINGS: 1100 out along a leg
ZEBRA_HALF_DEPTH_CM = 300.0                                   # band 800..1400
ZEBRA_OUTER_CM = ZEBRA_CENTRE_CM + ZEBRA_HALF_DEPTH_CM        # 1400
ZEBRA_HALF_SPAN_CM = R.CARRIAGEWAY_HALF_CM                    # 800: kerb to road centre
LINE_GAP_CM = 100.0                                           # stop line to zebra
CAR_HALF_LEN_CM = 230.0
CAR_HALF_WIDTH_CM = R.CAR_HALF_WIDTH_CM                       # 90
STOP_CM = ZEBRA_OUTER_CM + LINE_GAP_CM + CAR_HALF_LEN_CM      # 1730: the CENTRE at the line
COMMIT_CM = 50.0                                              # past the line by this: committed
QUEUE_CM = 650.0                                              # centre to centre behind a car
STOP_DECEL = R.STOP_DECEL_CMS2                                # 250: the stop ramp
MAX_DECEL = R.DECEL_CMS2                                      # 400: DriveTick's braking limit
J_SCAN_NEAR_CM, J_SCAN_FAR_CM = 1100.0, 3600.0                # NextJ window (drive_tick.py)
GIVEWAY_FAR_LEGACY_CM = 3100.0
GIVEWAY_FAR_CM = 3600.0                                       # permissive turn: oncoming window
ONCOMING_PASSED_CM = 700.0                                    # an oncoming car this far past the centre is clear
SNEAK_HELD_S = 4.0                                            # held this long on green by give-way/box ...
SNEAK_LATEST_S = 1.5                                          # ... a turner may go in the first 1.5 s of yellow
SNEAK_MARGIN_CM = 100.0
EXIT_CLEAR_CM = ZEBRA_OUTER_CM + CAR_HALF_LEN_CM + QUEUE_CM   # 2280 past the centre
LANE_HALF_CM = 250.0
ARC_LEADER_CM = 1300.0                                        # same-way car this close counts (legacy 900)
V_SLOW = 150.0                                                # exit rule: "not moving away"
V_STOPPED = 10.0                                              # DriveTick's StopRun threshold
EXIT_XING_BACK_CM = 550.0                                     # zebra on the exit side of NextJ:
EXIT_XING_NEAR_CM = 1500.0                                    # (c-J).fwd > -550, |c-J| < 1500
EXIT_QUARTER = 0.3                                            # box car heading into my exit quarter
EXIT_PED_SPAN_CM = ZEBRA_HALF_SPAN_CM + 200.0                 # exit zebra incl. a figure stepping off the kerb
EXIT_XING_AHEAD_PTS = 24                                      # a far turn's exit zebra is ~20 points past its line
RED_TOL_CM = 5.0                                              # RedViol: front past the line by more than this
BOX_WAIT_S = 2.0
DEADLOCK_S = 120.0

PED_KERB_CM = 900.0                                           # a crosser starts on the pavement
PED_SPEED_CMS = (110.0, 150.0)
PED_RATE_PER_S = 1.0 / 60.0                                   # per zebra

HOLD_REASONS = frozenset({"red", "yellow", "box", "giveway", "exit", "exitped", "late"})


def default_speed(car: str) -> float:
    """SpeedCmS per car. The level's values (390-560, docs/FINDING-citylife-level.md)
    live only in the gitignored level; this spreads the same range
    deterministically by car number. Car_10 is 320 (apply_routes.SUBJECT_SPEED)."""
    n = int(car.split("_")[1])
    return 390.0 + ((7 * n) % 18) * 10.0


# ------------------------------------------------------------------ static data

@dataclass
class JInfo:
    x: float
    y: float
    flag: float                   # 1 where this loop gives way (apply_routes.junction_flags)
    ux_in: float
    uy_in: float
    ux_out: float
    uy_out: float
    ex: float                     # exit lane point level with the centre
    ey: float
    ij: Tuple[int, int]
    signalled: bool
    line_idx: int                 # path point nearest the stop line


@dataclass
class Loop:
    name: str
    path: R.Path
    n: int
    junctions: List[Tuple[float, float, float]]
    crossings: List[Tuple[float, float, int]]
    jinfo: List[JInfo]
    xing_dir: List[Tuple[float, float]]         # path direction at each crossing (OFwd)


def junction_table(loop_junctions: Sequence[Tuple[float, float, float]], pts: Sequence[Tuple[float, float]],
                   exceptions: FrozenSet[Tuple[int, int]] = S.SIGNAL_EXCEPTIONS) -> List[JInfo]:
    """Per junction a loop drives through, in driving order: approach and exit
    directions, the exit lane point level with the centre (for JExit), the stop
    line's path index. Keep-left: a lane is LEFT of its direction of travel."""
    n = len(loop_junctions)
    out = []
    for k, (x, y, flag) in enumerate(loop_junctions):
        px, py, _ = loop_junctions[k - 1]
        nx, ny, _ = loop_junctions[(k + 1) % n]
        ui = _unit(x - px, y - py)
        uo = _unit(nx - x, ny - y)
        lo = R.left_of(uo)
        li = R.left_of(ui)
        lx = x - ui[0] * STOP_CM + li[0] * R.LANE_OFFSET_CM
        ly = y - ui[1] * STOP_CM + li[1] * R.LANE_OFFSET_CM
        line_idx = min(range(len(pts)), key=lambda i: math.hypot(pts[i][0] - lx, pts[i][1] - ly))
        out.append(JInfo(x, y, flag, ui[0], ui[1], uo[0], uo[1],
                         x + lo[0] * R.LANE_OFFSET_CM, y + lo[1] * R.LANE_OFFSET_CM,
                         S.grid_index(x, y), S.signalled(x, y, exceptions), line_idx))
    return out


def _unit(dx: float, dy: float) -> Tuple[float, float]:
    n = math.hypot(dx, dy)
    return (dx / n, dy / n)


def build_loops(data: dict, exceptions: FrozenSet[Tuple[int, int]]) -> Dict[str, Loop]:
    loops = {}
    for k, L in data["loops"].items():
        pts = [tuple(p) for p in L["route"]]
        n = len(pts)
        length = sum(math.hypot(pts[(i + 1) % n][0] - pts[i][0], pts[(i + 1) % n][1] - pts[i][1])
                     for i in range(n))
        path = R.Path(name=k, pts=pts, speed=list(L["speed"]),
                      curvature=[-c for c in L["kappa"]],    # plan() stores the car's sign
                      length_cm=length)
        xs = [(q[0], q[1], int(q[2])) for q in L["crossings"]]
        xd = []
        for (_, _, ci) in xs:
            a, b = pts[ci], pts[(ci + 1) % n]
            xd.append(_unit(b[0] - a[0], b[1] - a[1]))
        juncs = [tuple(j) for j in L["junctions"]]
        loops[k] = Loop(k, path, n, juncs, xs, junction_table(juncs, pts, exceptions), xd)
    return loops


def jexit_rows(data: Optional[dict] = None,
               exceptions: FrozenSet[Tuple[int, int]] = S.SIGNAL_EXCEPTIONS) -> Dict[str, List[Tuple[float, float, float]]]:
    """Per loop, one row per entry of its Junctions array, same order: the exit
    lane point level with the junction centre (UE cm) and the exit heading as
    an Unreal yaw in degrees (0 = +X north, 90 = +Y east). Written per car as
    the JExit Vector array, next to Junctions."""
    data = data or plan()
    loops = build_loops(data, frozenset(exceptions))
    return {k: [(J.ex, J.ey, round(math.degrees(math.atan2(J.uy_out, J.ux_out)), 6)) for J in L.jinfo]
            for k, L in loops.items()}


def dsl_constants() -> Dict[str, float]:
    """Every number the new junction rules use, for rendering into the DSL."""
    names = ("BOX_HALF_CM", "ZEBRA_OUTER_CM", "STOP_CM", "COMMIT_CM", "QUEUE_CM", "STOP_DECEL",
             "MAX_DECEL", "J_SCAN_NEAR_CM", "J_SCAN_FAR_CM", "GIVEWAY_FAR_CM", "GIVEWAY_FAR_LEGACY_CM",
             "ONCOMING_PASSED_CM", "SNEAK_HELD_S", "SNEAK_LATEST_S", "SNEAK_MARGIN_CM", "EXIT_CLEAR_CM",
             "LANE_HALF_CM", "ARC_LEADER_CM", "V_SLOW", "V_STOPPED", "EXIT_XING_BACK_CM",
             "EXIT_XING_NEAR_CM", "EXIT_QUARTER", "EXIT_PED_SPAN_CM", "EXIT_XING_AHEAD_PTS")
    g = globals()
    out = {k: float(g[k]) for k in names}
    out.update({"SIG_" + k: v for k, v in S.dsl_constants().items()})
    return out


# ------------------------------------------------------------------ pedestrians

@dataclass
class Ped:
    start: float
    end: float
    x0: float
    y0: float
    vx: float
    vy: float
    speed: float

    def at(self, t: float) -> Tuple[float, float, float]:
        dt = t - self.start
        return (self.x0 + self.vx * dt, self.y0 + self.vy * dt, self.speed)


def make_peds(t_end: float, seed: int, rate: float, signals: bool,
              exceptions: FrozenSet[Tuple[int, int]] = S.SIGNAL_EXCEPTIONS,
              crossings: Sequence[Tuple[float, float]] = R.CROSSINGS) -> List[Ped]:
    """Poisson crossers per zebra, sorted by start. With signals a crosser
    waits for walk(); without, it starts on arrival."""
    rng = random.Random(seed)
    out = []
    if rate <= 0:
        return out
    for cx, cy in crossings:
        jx, jy = S.nearest_junction(cx, cy)
        road = S.crossing_road_axis(cx, cy, jx, jy)
        ax, ay = (0.0, 1.0) if road == S.NS else (1.0, 0.0)      # walking direction: across
        use_sig = signals and S.signalled(jx, jy, exceptions)
        t = 0.0
        while True:
            t += rng.expovariate(rate)
            if t >= t_end:
                break
            side = rng.choice((-1.0, 1.0))
            v = rng.uniform(*PED_SPEED_CMS)
            start = S.next_walk_start(t, jx, jy, road) if use_sig else t
            dur = 2.0 * PED_KERB_CM / v
            out.append(Ped(start, start + dur, cx - ax * side * PED_KERB_CM, cy - ay * side * PED_KERB_CM,
                           ax * side * v, ay * side * v, v))
    out.sort(key=lambda p: p.start)
    return out


# ------------------------------------------------------------------ cars

@dataclass
class Car:
    name: str
    loop: Loop
    cap: float                    # SpeedCmS
    st: R.CarState
    fx: float = 1.0
    fy: float = 0.0
    eff: float = 0.0              # EffSpeed
    stop: float = 0.0             # StopCm (a new BP float defaults to 0)
    gap: float = 0.0              # GapCm
    ticks: int = 0
    still_run: float = 0.0
    still_c: int = -1
    still_cprev: int = -1
    still_done: int = -1
    why: str = ""                 # what set StopCm at the last update
    dt: float = 0.1               # frame time, for StillRun's 2*Dt
    # measurement
    line_k: int = 0
    prog: int = 0
    stop_run: float = 0.0
    stop_max: float = 0.0
    box_run: float = 0.0
    in_zebra_wait: bool = False
    held_turn: float = 0.0        # HeldTurn: s held on green at a give-way junction
    sneak_j: int = -1             # SneakJ: the junction this car is sneaking through on yellow
    counters: Dict[str, int] = field(default_factory=dict)

    def bump(self, k: str, n: int = 1):
        self.counters[k] = self.counters.get(k, 0) + n


def _set_fwd(car: Car):
    yaw = math.radians(car.st.yaw_deg)
    car.fx, car.fy = math.cos(yaw), math.sin(yaw)


# ------------------------------------------------------------------ UpdateEffSpeed

def update_eff(car: Car, t: float, cars: Sequence[Car], peds: Sequence[Tuple[float, float, float]],
               mode: str, box_rule: str = "moving") -> None:
    """drive_tick.py UpdateEffSpeed, with the new junction rules unless legacy."""
    car.ticks += 1
    if car.ticks % 2:
        return
    L, st = car.loop, car.st
    x, y, v = st.x, st.y, st.v
    fx, fy = car.fx, car.fy
    rx, ry = -fy, fx
    legacy = mode == "legacy"

    in_box, nj, jedge = False, -1, 99999.0
    for k, (jx, jy, _flag) in enumerate(L.junctions):
        dx, dy = jx - x, jy - y
        if abs(dx) < BOX_HALF_CM and abs(dy) < BOX_HALF_CM:
            in_box = True
        ta = dx * fx + dy * fy
        if (ta > J_SCAN_NEAR_CM and ta - J_SCAN_NEAR_CM < jedge and ta < J_SCAN_FAR_CM
                and abs(dx * rx + dy * ry) < BOX_HALF_CM):
            jedge, nj = ta - J_SCAN_NEAR_CM, k
    has_j = nj >= 0
    J = L.jinfo[nj] if has_j else None
    check_j = has_j and not in_box
    sig, ph = None, 0.0
    if check_j and not legacy and J.signalled:
        ph = S.phase_time(t, J.x, J.y, S.axis_of(fx, fy))       # s into MY axis' phase
        sig = S.state_of_phase(ph)

    gap, tmpb = 99999.0, 99999.0
    conflict = box_conf = give = exit_blk = onc_go = False
    for o in cars:
        if o is car:
            continue
        ox, oy = o.st.x, o.st.y
        dx, dy = ox - x, oy - y
        d = math.hypot(dx, dy)
        if d <= 1.0:
            continue
        ofx, ofy = o.fx, o.fy
        ta = dx * fx + dy * fy
        par = ofx * fx + ofy * fy
        if ta > 0.0:
            if par > 0.5:
                if abs(dx * rx + dy * ry) < LANE_HALF_CM or d < (900.0 if legacy else ARC_LEADER_CM):
                    gap = min(gap, ta)
            elif ta < 900.0 and abs(dx * rx + dy * ry) < LANE_HALF_CM:
                if par > 0.0 and not legacy:
                    # a leader more than 60 deg round a bend: the old rule
                    # only slowed for it (EffSpeed, floor 60 cm/s) and crept
                    # to 540-600 cm behind a car waiting in the turn
                    gap = min(gap, ta)
                else:
                    tmpb = min(tmpb, ta)
        if not check_j:
            continue
        ex, ey = ox - J.x, oy - J.y
        inside = abs(ex) < BOX_HALF_CM and abs(ey) < BOX_HALF_CM
        ta2 = -(ex * ofx + ey * ofy)               # the other car's along-distance to the centre
        olat = abs(ex * -ofy + ey * ofx)
        if legacy:
            if par < 0.7:
                if inside:
                    conflict = box_conf = True
                elif J.flag > 0.5 and BOX_HALF_CM < ta2 < GIVEWAY_FAR_LEGACY_CM and olat < BOX_HALF_CM:
                    conflict = give = True
        elif par < 0.7 and inside:
            # A car heading into MY exit quarter (towards the exit, not at me)
            # is the car I follow through a turn, not crossing traffic: the old
            # rule held every car behind a platoon leader whose heading passed
            # 45 deg in the box, and at JEdge - 300 - on the zebra (problem (b)
            # is mostly this). An oncoming car 7 m past the centre has passed
            # every path that crosses its lane.
            if not ((ofx * J.ux_out + ofy * J.uy_out > EXIT_QUARTER and par > -EXIT_QUARTER)
                    or (par < -0.7 and ta2 < -ONCOMING_PASSED_CM)):
                box_conf = True
        elif J.flag > 0.5 and olat < BOX_HALF_CM:
            if J.signalled:
                if par < -0.7 and BOX_HALF_CM < ta2 < GIVEWAY_FAR_CM:     # oncoming: same green
                    give = True
                    vo = o.st.v                                           # ... and it cannot stop
                    if vo > V_STOPPED and ta2 - STOP_CM < vo * vo / (2.0 * STOP_DECEL) - COMMIT_CM + SNEAK_MARGIN_CM:
                        onc_go = True
            elif par < 0.7 and BOX_HALF_CM < ta2 < GIVEWAY_FAR_LEGACY_CM:  # unsignalled: the old window
                give = True
        if not legacy:
            qx, qy = ox - J.ex, oy - J.ey
            along = qx * J.ux_out + qy * J.uy_out
            if (-BOX_HALF_CM < along < EXIT_CLEAR_CM
                    and abs(qx * -J.uy_out + qy * J.ux_out) < LANE_HALF_CM
                    and ofx * J.ux_out + ofy * J.uy_out > 0.5
                    and (box_rule == "static" or o.st.v < V_SLOW)):
                exit_blk = True

    stop, why = 99999.0, ""
    if legacy and conflict:
        stop, why = jedge - 300.0, ("box" if box_conf else "giveway")
        car.bump("yield_ticks")

    # --- crossings: drive_tick.py verbatim (emu_r4.py), plus the exit-zebra flag
    n = L.n
    idx = st.idx
    if car.ticks < 3:
        car.still_done = -1
        car.still_cprev = -1
    still_seen, still_best, ehit, close_hit, exit_ped = False, 99999, False, False, False
    for q, (cx, cy, ci) in enumerate(L.crossings):
        ca = (ci - idx + n) % n
        ca2 = ca if ca <= 17 else 1000 + (n - ca)
        if ci == car.still_done and 17 < ca < n - 4:
            car.still_done = -1
        exempt = ci == car.still_done or (ci == car.still_cprev and car.still_run >= 6.0)
        is_exit = False
        if check_j and not legacy and 3 <= ca <= EXIT_XING_AHEAD_PTS:
            jx_, jy_ = cx - J.x, cy - J.y
            is_exit = jx_ * fx + jy_ * fy > -EXIT_XING_BACK_CM and math.hypot(jx_, jy_) < EXIT_XING_NEAR_CM
        in_win = ca <= 17 or ca >= n - 4
        if not (in_win or is_exit):
            continue
        ofx, ofy = L.xing_dir[q]
        for (px, py, pspd) in peds:
            ex, ey = px - cx, py - cy
            if (is_exit and pspd > 20.0 and abs(ex * ofx + ey * ofy) < ZEBRA_HALF_DEPTH_CM
                    and abs(ey * ofx - ex * ofy) < EXIT_PED_SPAN_CM):
                exit_ped = True
            if not in_win:
                continue
            if abs(ex * ofx + ey * ofy) < ZEBRA_HALF_DEPTH_CM and abs(ey * ofx - ex * ofy) < ZEBRA_HALF_SPAN_CM:
                if pspd < 20.0:
                    still_seen = True
                    if ca2 < still_best:
                        still_best, car.still_c = ca2, ci
                if 3 <= ca <= 17:
                    if pspd > 20.0 or not exempt:
                        s_ = 150.0 * ca - 650.0
                        if s_ < stop:
                            stop, why = s_, "ped"
                        car.bump("ped_ticks")
                else:
                    dx, dy = px - x, py - y
                    ta = dx * fx + dy * fy
                    if -100.0 < ta < 900.0 and abs(dx * rx + dy * ry) < 300.0:
                        if pspd > 20.0 or not exempt:
                            stop, why = 0.0, "estop"
                            ehit = True
                            if pspd > 20.0 and ta < 400.0:
                                close_hit = True
                    if ca <= 2 and v > 50.0:
                        car.bump("ped_viol" if pspd > 20.0 else "ped_pass_still")
    if still_seen:
        if car.still_c != car.still_cprev:
            car.still_run = 0.0
            car.still_cprev = car.still_c
        if v < V_STOPPED:
            car.still_run += 2.0 * car.dt
        if car.still_run >= 6.0:
            car.still_done = car.still_cprev
    else:
        car.still_run = 0.0
        car.still_cprev = -1
    if ehit:
        car.bump("estops")
        if v > 50.0:
            car.bump("ped_corr")
    if close_hit and v > 50.0:
        car.bump("ped_close")

    # --- the new junction rules
    if not check_j or nj != car.sneak_j:
        car.sneak_j = -1
    if not check_j:
        car.held_turn = 0.0
    if not legacy and check_j:
        dline = jedge + J_SCAN_NEAR_CM - STOP_CM
        if dline > -COMMIT_CM:
            # A turner held through the whole green at a give-way junction goes
            # on the first 1.5 s of yellow if no oncoming car is unable to stop
            # (the "sneaker" every real permissive turn relies on): without it
            # loop B's right turn at (12300, -4100) waited 146 s behind loop A.
            sneaking = car.sneak_j == nj
            if (not sneaking and sig == S.YELLOW and J.flag > 0.5 and car.held_turn >= SNEAK_HELD_S
                    and v < V_STOPPED and dline < COMMIT_CM
                    and ph - S.GREEN_S < SNEAK_LATEST_S
                    and not (box_conf or onc_go or exit_blk or exit_ped)):
                sneaking, car.sneak_j = True, nj
                car.bump("sneaks")
            hold = ""
            if sig == S.RED and not sneaking:
                hold = "red"
            elif sig == S.YELLOW and not sneaking and dline >= v * v / (2.0 * STOP_DECEL) - COMMIT_CM:
                hold = "yellow"
            if hold:
                target = dline
            else:
                # exitped only while walkers may still START (the first WALK_S
                # of my green: a turner's exit zebra walks with it); after that
                # the car pulls in behind the last one and the crossing rule
                # stops it at the zebra - holding at the line for the whole
                # green starved loop A's turn at (4100, -4100) for 127 s.
                ped_hold = exit_ped and (sig is None or (sig == S.GREEN and ph < S.WALK_S))
                hold = ("box" if box_conf else "giveway" if (onc_go if sneaking else give) else
                        "exit" if exit_blk else "exitped" if ped_hold else "")
                if hold in ("box", "giveway") and J.flag > 0.5 and sig == S.GREEN:
                    car.held_turn += 2.0 * car.dt
                elif v > V_STOPPED:
                    car.held_turn = 0.0
                target = dline
                if hold and dline < v * v / (2.0 * MAX_DECEL) - COMMIT_CM:
                    if hold == "box":
                        target, hold = jedge - 300.0, "late"      # a car IN the box: the old target
                        car.bump("late_hold")
                    else:
                        # committed: an oncoming car that entered the window
                        # after I started sees me in the box and stops at its
                        # line; exit/exitped are left to the queue and
                        # crossing rules
                        car.bump("late_" + hold)
                        hold = ""
                if sneaking and hold and hold != "late":
                    # held again while it can still stop at the line: the
                    # sneak is over, and from the next update the yellow and
                    # red rules apply. Kept latched, a turner stopped at its
                    # line by an oncoming car that could not stop waited into
                    # the red and then drove through it.
                    car.sneak_j = -1
                    car.bump("sneak_off")
            if hold:
                if hold in ("box", "giveway", "late"):
                    car.bump("yield_ticks")
                if target < stop:
                    stop, why = target, hold
        elif box_conf:
            car.bump("late_hold")                                  # committed into a busy box
            if jedge - 300.0 < stop:
                stop, why = jedge - 300.0, "late"

    car.stop, car.gap, car.why = stop, gap, why
    tmpa = max(0.0, min(1.0, tmpb / (700.0 + 1.8 * v)))
    car.eff = min(car.cap, max(60.0, car.cap * tmpa * tmpa))


# ------------------------------------------------------------------ DriveTick

def drive(car: Car, dt: float, t: float, M: "Metrics") -> None:
    """drive_tick.py DriveTick: citylife_routes.follow_step in <= 50 ms sub-steps,
    StopCm and GapCm shrunk by the distance driven. Measures the stop lines.

    A line crossed in this frame is judged against the signal at the FRAME's
    game time t: UE advances TimeSeconds before actors tick, the car decided
    on the state at t, and a rendered frame shows light and car at the same t.
    Stamping each sub-step with a time inside (t - dt, t] instead flagged
    every car that moved off its line on the first green frame at 3 fps
    (164 in 30 min, all in the last 0.1 s of all-red)."""
    st, L = car.st, car.loop
    nsub = max(1, min(20, int(dt * 20.0) + 1))
    sd = dt / nsub
    n = L.n
    for s in range(nsub):
        i_before = st.idx
        R.follow_step(L.path, st, sd, v_cap=car.eff, stop_cm=min(car.stop, car.gap - QUEUE_CM))
        car.stop -= st.v * sd
        car.gap -= st.v * sd
        car.prog += (st.idx - i_before) % n
        J = L.jinfo[car.line_k]
        along = (J.x - st.x) * J.ux_in + (J.y - st.y) * J.uy_in
        if along < STOP_CM - RED_TOL_CM:
            if J.signalled and M.signals:
                s_ = S.state(t, J.x, J.y, S.axis_of(J.ux_in, J.uy_in))
                if s_ == S.RED:
                    M.red_viol += 1
                    M.red_viol_log.append((round(t, 2), car.name, (J.x, J.y)))
                elif s_ == S.YELLOW:
                    M.yellow_entries += 1
            car.line_k = (car.line_k + 1) % len(L.jinfo)
    _set_fwd(car)


# ------------------------------------------------------------------ measurement

def _rects_overlap(c1, u1, h1, c2, u2, h2) -> bool:
    """Separating-axis test for two rectangles: centre, unit length axis, (half length, half width)."""
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    for ax, ay in (u1, (-u1[1], u1[0]), u2, (-u2[1], u2[0])):
        r1 = h1[0] * abs(u1[0] * ax + u1[1] * ay) + h1[1] * abs(-u1[1] * ax + u1[0] * ay)
        r2 = h2[0] * abs(u2[0] * ax + u2[1] * ay) + h2[1] * abs(-u2[1] * ax + u2[0] * ay)
        if abs(dx * ax + dy * ay) > r1 + r2:
            return False
    return True


@dataclass
class Metrics:
    signals: bool
    red_viol: int = 0
    red_viol_log: List = field(default_factory=list)
    yellow_entries: int = 0
    zebra_wait_s: float = 0.0
    zebra_wait_events: int = 0
    zebra_wait_log: List = field(default_factory=list)
    zebra_stop_any_s: float = 0.0
    zebra_stop_why: Dict[str, float] = field(default_factory=dict)
    box_wait_events: int = 0
    box_wait_s: float = 0.0
    box_wait_max_s: float = 0.0
    box_wait_why: Dict[str, int] = field(default_factory=dict)
    min_gap: float = 1e9
    min_gap_at: Tuple = ()
    red_front_min_cm: float = 1e9      # nearest a car held by a red got its front to the centre
    ped_hits: int = 0


ZEBRAS = [((cx, cy), (1.0, 0.0) if S.crossing_road_axis(cx, cy, *S.nearest_junction(cx, cy)) == S.NS
           else (0.0, 1.0)) for cx, cy in R.CROSSINGS]
ZEBRA_HALF = (ZEBRA_HALF_DEPTH_CM, ZEBRA_HALF_SPAN_CM)
CAR_HALF = (CAR_HALF_LEN_CM, CAR_HALF_WIDTH_CM)


def _on_zebra(car: Car) -> bool:
    c1, u1 = (car.st.x, car.st.y), (car.fx, car.fy)
    for c2, u2 in ZEBRAS:
        if abs(c2[0] - c1[0]) < 1500 and abs(c2[1] - c1[1]) < 1500 and \
                _rects_overlap(c1, u1, CAR_HALF, c2, u2, ZEBRA_HALF):
            return True
    return False


# ------------------------------------------------------------------ the run

def build_cars(data: dict, loops: Dict[str, Loop], speeds: Optional[Dict[str, float]] = None) -> List[Car]:
    cars = []
    for name in sorted(data["cars"]):
        c = data["cars"][name]
        L = loops[c["loop"]]
        cap = c["speed"] if c["speed"] is not None else (speeds or {}).get(name, default_speed(name))
        car = Car(name, L, float(cap), R.CarState(c["x"], c["y"], c["yaw"], 0.0, c["idx"]))
        car.eff = car.cap
        _set_fwd(car)
        car.line_k = min(range(len(L.jinfo)), key=lambda k: (L.jinfo[k].line_idx - c["idx"]) % L.n)
        J = L.jinfo[car.line_k]
        if (J.x - car.st.x) * J.ux_in + (J.y - car.st.y) * J.uy_in < STOP_CM - RED_TOL_CM:
            car.line_k = (car.line_k + 1) % len(L.jinfo)
        cars.append(car)
    return cars


def simulate(minutes: float = 30.0, fps: float = 10.0, mode: str = "signals", seed: int = 1,
             ped_rate: float = PED_RATE_PER_S, box_rule: str = "moving",
             exceptions: FrozenSet[Tuple[int, int]] = S.SIGNAL_EXCEPTIONS,
             speeds: Optional[Dict[str, float]] = None, data: Optional[dict] = None) -> dict:
    """Run all 24 cars for `minutes` of game time at a fixed frame rate.
    mode: 'signals' (the new rules) or 'legacy' (today's DSL). Deterministic."""
    if mode not in ("signals", "legacy"):
        raise ValueError(mode)
    if box_rule not in ("moving", "static"):
        raise ValueError(box_rule)
    frames = int(round(minutes * 60.0 * fps)) if minutes > 0.0 and fps > 0.0 else 0
    if frames < 1:
        # a run of no frames would report MinGap 1e9 and no deadlock: a pass
        # that measured nothing
        raise ValueError(f"minutes={minutes}, fps={fps}: no frame to simulate")
    data = data or plan()
    loops = build_loops(data, frozenset(exceptions))
    cars = build_cars(data, loops, speeds)
    signals = mode == "signals"
    M = Metrics(signals=signals)
    dt = 1.0 / fps
    peds_all = make_peds(minutes * 60.0 + 60.0, seed, ped_rate, signals, frozenset(exceptions))
    boxes = sorted({(j[0], j[1]) for L in loops.values() for j in L.junctions})
    bx = np.array([b[0] for b in boxes])
    by = np.array([b[1] for b in boxes])
    for c in cars:
        c.dt = dt
    subj = next(c for c in cars if c.name == SUBJECT)
    laps, lap_marks = [], []
    active: List[Ped] = []
    p_next = 0
    ncar = len(cars)
    iu = np.triu_indices(ncar, 1)
    for k in range(1, frames + 1):
        t = k * dt                                    # the frame's game time (GetGameTimeInSeconds)
        while p_next < len(peds_all) and peds_all[p_next].start <= t:
            active.append(peds_all[p_next])
            p_next += 1
        active = [p for p in active if p.end > t]
        peds = [p.at(t) for p in active]
        for c in cars:
            update_eff(c, t, cars, peds, mode, box_rule)
            drive(c, dt, t, M)
        # --- measurement, after every car has moved
        P = np.array([(c.st.x, c.st.y) for c in cars])
        d = np.hypot(P[:, None, 0] - P[None, :, 0], P[:, None, 1] - P[None, :, 1])[iu]
        m = int(np.argmin(d))
        if d[m] < M.min_gap:
            M.min_gap = float(d[m])
            M.min_gap_at = (round(t, 2), cars[iu[0][m]].name, cars[iu[1][m]].name)
        for c in cars:
            if c.st.v < V_STOPPED:
                c.stop_run += dt
                c.stop_max = max(c.stop_max, c.stop_run)
                held = c.why in HOLD_REASONS and c.stop <= c.gap - QUEUE_CM
                zeb = _on_zebra(c)
                if zeb and c.why != "estop":
                    M.zebra_stop_any_s += dt
                    w = c.why if held or c.why == "ped" else ("queue" if c.gap - QUEUE_CM <= c.stop else c.why or "eff")
                    M.zebra_stop_why[w] = M.zebra_stop_why.get(w, 0.0) + dt
                if zeb and held:
                    M.zebra_wait_s += dt
                    if not c.in_zebra_wait:
                        M.zebra_wait_events += 1
                        M.zebra_wait_log.append((round(t, 2), c.name, c.why, round(c.st.x), round(c.st.y)))
                    c.in_zebra_wait = True
                else:
                    c.in_zebra_wait = False
                if c.why == "red" and held and c.stop_run > 1.0:
                    front = _front_to_centre(c)
                    if front is not None:
                        M.red_front_min_cm = min(M.red_front_min_cm, front)
                inb = bool(np.any((np.abs(bx - c.st.x) < BOX_HALF_CM) & (np.abs(by - c.st.y) < BOX_HALF_CM)))
                if inb:
                    c.box_run += dt
                    if c.box_run - dt < BOX_WAIT_S <= c.box_run:
                        M.box_wait_events += 1
                        M.box_wait_why[c.why or "queue"] = M.box_wait_why.get(c.why or "queue", 0) + 1
                    if c.box_run >= BOX_WAIT_S:
                        M.box_wait_s += dt
                        M.box_wait_max_s = max(M.box_wait_max_s, c.box_run)
                else:
                    c.box_run = 0.0
            else:
                c.stop_run = 0.0
                c.box_run = 0.0
                c.in_zebra_wait = False
        for (px, py, _s) in peds:
            for c in cars:
                if c.st.v < 50.0:
                    continue                  # a crude figure walking into a STOPPED car
                qx, qy = px - c.st.x, py - c.st.y
                if abs(qx) < 400 and abs(qy) < 400 and \
                        abs(qx * c.fx + qy * c.fy) < CAR_HALF_LEN_CM + 30 and \
                        abs(-qx * c.fy + qy * c.fx) < CAR_HALF_WIDTH_CM + 30:
                    M.ped_hits += 1
        if subj.prog >= subj.loop.n * (len(lap_marks) + 1):
            lap_marks.append(t)
    prev = 0.0
    for tm in lap_marks:
        laps.append(round(tm - prev, 1))
        prev = tm
    tot = {}
    for c in cars:
        for kk, vv in c.counters.items():
            tot[kk] = tot.get(kk, 0) + vv
    worst = max(cars, key=lambda c: c.stop_max)
    dist = {}
    for c in cars:
        dist.setdefault(c.loop.name, []).append(c.prog * c.loop.path.length_cm / c.loop.n
                                                if c.loop.path.length_cm else 0.0)
    return {
        "mode": mode, "fps": fps, "minutes": minutes, "box_rule": box_rule,
        "seed": seed, "ped_rate": ped_rate,
        "red_viol": M.red_viol, "red_viol_log": M.red_viol_log[:10],
        "yellow_entries": M.yellow_entries,
        "zebra_wait_s": round(M.zebra_wait_s, 1), "zebra_wait_events": M.zebra_wait_events,
        "zebra_wait_log": M.zebra_wait_log[:10],
        "zebra_stop_any_s": round(M.zebra_stop_any_s, 1),
        "zebra_stop_why": {k: round(v, 1) for k, v in sorted(M.zebra_stop_why.items())},
        "box_wait_events": M.box_wait_events, "box_wait_s": round(M.box_wait_s, 1),
        "box_wait_max_s": round(M.box_wait_max_s, 1), "box_wait_why": M.box_wait_why,
        "min_gap_cm": round(M.min_gap, 1), "min_gap_at": M.min_gap_at,
        "max_stop_s": round(worst.stop_max, 1), "max_stop_car": worst.name,
        "deadlocks": sorted(c.name for c in cars if c.stop_max > DEADLOCK_S),
        "red_front_min_cm": None if M.red_front_min_cm > 1e8 else round(M.red_front_min_cm, 1),
        "ped_hits": M.ped_hits, "peds": len([p for p in peds_all if p.start < minutes * 60.0]),
        "car10_laps_s": laps,
        "car10_mean_lap_s": round(sum(laps[1:]) / len(laps[1:]), 1) if len(laps) > 1 else None,
        "mean_speed_cms": {k: round(sum(v) / len(v) / (minutes * 60.0), 1) for k, v in sorted(dist.items())},
        "counters": tot,
    }


def _front_to_centre(car: Car) -> Optional[float]:
    """Distance from the car's FRONT to the centre of the junction it faces,
    along its heading, if one is 0-3600 cm ahead within its lane corridor."""
    best = None
    for (jx, jy, _f) in car.loop.junctions:
        dx, dy = jx - car.st.x, jy - car.st.y
        ta = dx * car.fx + dy * car.fy
        if 0.0 < ta < J_SCAN_FAR_CM and abs(-dx * car.fy + dy * car.fx) < BOX_HALF_CM:
            f = ta - CAR_HALF_LEN_CM
            best = f if best is None else min(best, f)
    return best


def _fmt(r: dict) -> str:
    return (f"{r['mode']:7s} fps {r['fps']:4.0f} | RedViol {r['red_viol']:3d} (yellow entries "
            f"{r['yellow_entries']}) | ZebraWait {r['zebra_wait_s']:6.1f} s / {r['zebra_wait_events']} "
            f"(any stop on a zebra {r['zebra_stop_any_s']:.1f} s {r['zebra_stop_why']}) | BoxWait {r['box_wait_events']} x, "
            f"{r['box_wait_s']:.1f} s, max {r['box_wait_max_s']:.1f} s {r['box_wait_why']} | MinGap "
            f"{r['min_gap_cm']:.0f} cm {r['min_gap_at']} | MaxStop {r['max_stop_s']:.1f} s "
            f"({r['max_stop_car']}) | deadlocks {r['deadlocks']} | red-held front >= "
            f"{r['red_front_min_cm']} cm | ped hits {r['ped_hits']} of {r['peds']} crossers | "
            f"Car_10 laps {r['car10_laps_s']} mean {r['car10_mean_lap_s']} | speed {r['mean_speed_cms']} | "
            f"{r['counters']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--minutes", type=float, default=30.0)
    ap.add_argument("--fps", type=float, nargs="+", default=[3.0, 10.0, 30.0])
    ap.add_argument("--mode", nargs="+", default=["signals", "legacy"])
    ap.add_argument("--box-rule", default="moving", choices=["moving", "static"])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--ped-rate", type=float, default=PED_RATE_PER_S)
    a = ap.parse_args()
    data = plan()
    for mode in a.mode:
        for fps in a.fps:
            t0 = time.time()
            r = simulate(a.minutes, fps, mode, a.seed, a.ped_rate, a.box_rule, data=data)
            print(_fmt(r), f"[{time.time() - t0:.0f} s wall]", flush=True)


if __name__ == "__main__":
    main()
