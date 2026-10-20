"""Where the escalation FSM's Loiter holds and its Land lands - chosen legally.

WHY THIS FILE EXISTS

The grant's escalation FSM names the modes (Safety Shield p4-p5: "SET_MODE
GUIDED / LOITER / RTL / LAND") and never the place. ArduPilot's LOITER (flown
as BRAKE on our rails) holds where the aircraft is and LAND descends where it
is, so until 2026-10-19 the FSM's hold point was "here", whatever "here" was:

  * The library's only P0 escape ticks (rate 0.000133: 6 ticks in the three
    `time-window-opens-on-vehicle` cells, experiments/templates/
    time_window_switch.yaml) come from a window opening over the aircraft
    sooner than it can leave. The Shield flies the fastest way out with the
    violation still on (2 ticks a cell), and at 0.1 s G3 hands the aircraft
    to a LOITER that ArduPilot holds at (60, 21.8) - INSIDE the zone whose
    window opens 0.9-1.9 s later. The modelled autopilot then flew 28-38 P0
    ticks of LOITER per cell (26-36 on the 3 s arm) and the aircraft sat 41
    ticks inside the polygon (CHANGELOG 2026-10-13, "Not fixed": "The FSM can
    hold LOITER on a spot about to become illegal").
  * `dyn-nfz-rotate-spin` (9 deg/s) held a LOITER the spinning bar swept
    onto (50 P0 ticks, then X6: a false fail-safe); the six
    `three-simultaneous-violations` cells held LOITER below the 10 m floor
    (48 P0 ticks, then X6: the family's pinned false fail-safe); and G6
    "lands in place, which can be inside a zone" (`wind-beyond-authority`,
    lane 20 at 12 m/s, landed in `nfz-square`; the 3 s arm of the 6 deg/s
    spin, in the bar).

This module chooses the point instead, from the Shield's OWN rule queries, so
there is still one rule-evaluation code path (Safety Shield page): no second
geometry, no copy of the rules.

WHAT "LEGAL" MEANS HERE

  hold     `Shield.state_is_unsafe(point, ahead_s=arrival)` holds no hard,
           enforced violation (`Shield.hard_violations`): a stop at the point,
           starting when the aircraft gets there, breaks nothing through the
           Shield's lookahead (5 s at the grant's forecast). The same test the
           FSM's `stop_illegal` is, so a plan this module calls legal is one
           the FSM's next tick at the point agrees with.
  landing  The same, for the descent: every lookahead-long chunk from arrival
           to touchdown, at the altitudes the descent passes in it (both ends
           and every zone floor or ceiling between), restricted to the rules a
           SITE can satisfy - keep-out zones, stand-off rings, obstacle
           clearance - plus the footprint of a hard keep-in corridor. A
           descent to the ground breaks every altitude floor wherever it is
           flown (the library's four Land cells break the demo policy's 10 m
           floor, labelled so), so altitude envelopes and corridor bands do not
           choose the site; the reason names them.
  transit  Every pose of the straight transit, at its own time (sampled every
           TRANSIT_STEP_M, each axis at its own speed, as the harness's
           modelled autopilot flies it), breaches no hard enforced rule
           (`Shield.rule_status` 'breach', in a look-ahead view at that time)
           except one the aircraft is LEAVING: a rule a stop at the start
           breaks within the lookahead that the aircraft is already in -
           breached at the start now, or not in force now and in force over
           the start by then (a window opening over it) - with the pose no
           deeper in it than a stop at the start is then
           (`Shield.rule_depths`). A rule in force that the aircraft is
           outside of is never entered - a zone or a stand-off ring closing
           on it is escaped, never crossed or swept through - and one it is
           in is left by getting shallower, never by going through it.
           (Until 2026-10-20 a rule a stop at the start breaks was excused at
           every pose, and a transit could cross a zone closing on the
           aircraft: transit_breaks.)

THE SEARCH (deterministic: a sweep is reproducible on any machine)

  1. Where the aircraft is. Legal -> an in-place plan, and the hold is
     exactly what it always was. This is every hold of the library but the
     ones listed above (measured: docs/DESIGN-hold-points.md).
  2. Otherwise DIRECTIONS headings x RADII_M (1-10 m every metre, then every
     2 m to 30, every 5 m to 60) at the hold altitude: the current one, or,
     when the aircraft is outside the hard altitude band in force, the
     nearest altitude BAND_INSET_M inside it (plus that altitude straight
     above or below). Candidates are tried in order of travel time, ties to
     the least turn from the way the aircraft is already going; the first
     one legal with its HOLD_MARGIN_M neighbours and reached by a legal
     transit is the plan, its heading then refined every REFINE_STEP_M back
     toward the last empty ring.
  3. Nothing legal within MAX_QUERIES Shield queries (a stop query or a
     transit pose, one each) or 60 m: an in-place plan marked ILLEGAL that
     names what it breaks (the FSM then searches again once a second,
     fsm.HOLD_REPLAN_S). A query budget, not a time budget, so the answer
     does not depend on the machine's load.

SPEED ASSUMPTION

  The transit is planned at the tightest enforced speed cap in force
  (`Shield.kinematic_caps`; 4 m/s in every library policy) and its climb cap,
  else DEFAULT_SPEED_MPS / DEFAULT_CLIMB_MPS (4 and 2 m/s: the Shield's own
  default speed for a rescue heading, shield.py `_decide`, and the grant's
  2 m/s climb envelope). A landing descends at LAND_DESCENT_MPS (ArduCopter's
  LAND_SPEED default, 50 cm/s; experiments/sweep_scenarios.py AP_LAND_MPS).
  Wind is not modelled: a point the aircraft cannot reach against the wind is
  no better than one it cannot hold, and the FSM's X6 / X7 still watch the
  illegal depth while it tries.

WHAT IT DOES NOT DO

  * Change an FSM edge, guard or setpoint (guardrail/fsm.py): the FSM still
    takes G2 / G3 into Loiter and G6 into Land when it did; this decides only
    where the autopilot is then told to hold or land.
  * Count anything. The P0 escape KPI (the Shield's counter over the ticks it
    decided) is untouched; a moved hold is flown by the autopilot (on
    ArduPilot a GUIDED position target, then the hold mode on arrival) and
    its ticks are the autopilot's, reported as such.
  * Fly a rail. The flight rails run their own FSM and still hold in place;
    `ShieldDecision.hold_point` is what a rail would fly (lead notes).
"""
from __future__ import annotations

import itertools
import math
from typing import Any, Callable, Iterable

from .fsm import HoldPoint
from .models import State

DIRECTIONS = 16
RADII_M: tuple[float, ...] = (tuple(float(r) for r in range(1, 11))
                              + tuple(float(r) for r in range(12, 31, 2))
                              + tuple(float(r) for r in range(35, 61, 5)))
DEFAULT_SPEED_MPS = 4.0
DEFAULT_CLIMB_MPS = 2.0
LAND_DESCENT_MPS = 0.5
TRANSIT_STEP_M = 1.0
BAND_INSET_M = 0.5
# The search's budget, in Shield queries: a stop query (state_is_unsafe over
# the lookahead) or one sampled transit pose (rule_status) costs one each.
# Until 2026-10-20 only stop queries were counted; once a transit could no
# longer cross a zone closing on the aircraft (transit_breaks), a search
# could transit-check every candidate whose stop was legal and fail them all
# - up to 60 poses each, uncounted (a 40 m bar closing at 3 m/s: 106 transit
# checks, 325 poses, 30 % of the search's time, review of T1c P2). A pose
# costs about 0.7 of a stop query on the desktop (0.24 vs 0.33 ms).
MAX_QUERIES = 800
# A moved point must stay legal this far off it in each horizontal direction
# (four more stop queries, made only for a point that already passed): the
# Shield's zone test is strict, so a point exactly ON a margin ring is legal
# and a hold there flickers in and out of the ring with 0.2-0.3 m of position
# noise (the harness's GPS-noise cells; the 8 Oct follow's estimator). Found
# by the first unit test: (60, 11.0), on nfz-school's ring.
HOLD_MARGIN_M = 0.5
# Between the last ring with no legal point and the first with one, the
# winning heading is searched again at this step (the rings are 2 m and 5 m
# apart past 10 m), so "nearest" is within half a metre on that heading.
REFINE_STEP_M = 0.5
# A transit pose may sit no deeper in a rule it is leaving than a stop at the
# start would be at that moment (transit_breaks). This is float noise only:
# both depths come from the same Shield, in the same look-ahead view.
DEPTH_TOL_M = 1e-6
# The rule categories a landing SITE can satisfy (shield.Violation.category).
# Not "altitude" (a descent breaks every floor wherever it is flown) and not
# "corridor" (its band likewise; its footprint is judged apart, _corridor_out).
LAND_SITE_CATEGORIES = frozenset({"geofence", "standoff", "clearance"})
_TUBE_TYPES = ("corridor", "corridor_swap")


def _binding_row(row: dict) -> bool:
    """A rule_status row of a hard rule the Shield enforces."""
    return (row.get("constraint_type", "hard") == "hard"
            and row.get("effective_action") != "monitor_only")


def _ids(vios: Iterable[Any]) -> tuple[str, ...]:
    return tuple(sorted({v.rule_id for v in vios}))


def _speeds(shield) -> tuple[float, float, str]:
    caps = shield.kinematic_caps()
    if caps is None:
        return (DEFAULT_SPEED_MPS, DEFAULT_CLIMB_MPS,
                f"no enforced speed cap in force: {DEFAULT_SPEED_MPS:g} m/s and "
                f"{DEFAULT_CLIMB_MPS:g} m/s climb assumed")
    return (caps.speed_max_mps, caps.climb_rate_max_mps,
            f"planned at the policy's {caps.speed_max_mps:g} m/s speed cap and "
            f"{caps.climb_rate_max_mps:g} m/s climb cap")


def _band(shield, state: State) -> tuple[float, float] | None:
    """The hard altitude band in force here: the intersection of every hard,
    enforced altitude envelope and corridor band (rule_status rows). None
    when there is none."""
    lo, hi, found = -math.inf, math.inf, False
    for row in shield.rule_status(state):
        if not row["in_force"] or not _binding_row(row):
            continue
        c = row["rule"]
        if row["type"] == "altitude_envelope":
            lo, hi, found = max(lo, c.alt_min_m), min(hi, c.alt_max_m), True
        elif row["type"] in _TUBE_TYPES:
            lo, hi = max(lo, c.altitude_floor_m), min(hi, c.altitude_ceiling_m)
            found = True
    return (lo, hi) if found else None


def hold_altitude(shield, state: State) -> float:
    """The altitude a hold is planned at: the current one inside the hard
    band, else the nearest one BAND_INSET_M (or a quarter of the band, if it
    is thinner) inside it. A hold below an altitude floor is a stop the
    Shield calls illegal at every horizontal position, so searching around
    it at that height could never succeed (the six three-simultaneous-
    violations cells LOITERed 48 ticks under the demo policy's 10 m floor)."""
    b = _band(shield, state)
    if b is None:
        return state.up
    lo, hi = b
    if lo > hi or lo - 1e-9 <= state.up <= hi + 1e-9:
        return state.up
    inset = min(BAND_INSET_M, (hi - lo) / 4.0)
    return min(max(state.up, lo + inset), hi - inset)


def _candidates(state: State, speed: float, climb: float, up_t: float,
                heading: tuple[float, float] | None, vertical: bool = True) -> list:
    """(travel_s, turn_rad, index, x, y, up), in the order they are tried:
    travel time, then the least turn from `heading`, then heading index."""
    out = []
    dz = abs(up_t - state.up)
    tz = dz / climb if dz > 1e-9 else 0.0
    if vertical and dz > 1e-9:
        out.append((tz, 0.0, -1, state.x, state.y, up_t))
    h = (math.atan2(heading[1], heading[0])
         if heading is not None and math.hypot(*heading) > 1e-9 else None)
    for r in RADII_M:
        t = max(r / speed, tz)
        for k in range(DIRECTIONS):
            ang = 2.0 * math.pi * k / DIRECTIONS
            turn = 0.0 if h is None else abs((ang - h + math.pi) % (2.0 * math.pi) - math.pi)
            out.append((t, turn, k, state.x + r * math.cos(ang),
                        state.y + r * math.sin(ang), up_t))
    out.sort(key=lambda c: (round(c[0], 9), round(c[1], 9), c[2]))
    return out


def _neighbours(p: State) -> list[State]:
    """The four points HOLD_MARGIN_M off `p`, east, west, north and south."""
    m = HOLD_MARGIN_M
    return [State(x=p.x + dx, y=p.y + dy, up=p.up, yaw_deg=p.yaw_deg)
            for dx, dy in ((m, 0.0), (-m, 0.0), (0.0, m), (0.0, -m))]


def _search(shield, state: State, speed: float, climb: float, up_t: float,
            heading, quick_ok, accept, budget: list, vertical: bool = True):
    """The ring search holds and landings share (module docstring, "THE
    SEARCH"). Candidates in travel-time order, one look-ahead view per
    arrival time; `quick_ok(p, t_a)` is the cheap test made inside that view,
    `accept(r, k, t_a, x, y)` the full one (a plan, or None). On the first
    accepted candidate of a ring, its heading is searched again between the
    previous ring and this one every REFINE_STEP_M, and the nearest accepted
    point wins. `budget` ([queries left]) bounds the whole search."""
    cands = _candidates(state, speed, climb, up_t, heading, vertical)
    tz = abs(up_t - state.up) / climb if abs(up_t - state.up) > 1e-9 else 0.0
    for t_a, group in itertools.groupby(cands, key=lambda c: round(c[0], 9)):
        hits = []
        with shield.looking_ahead(t_a):
            for c in group:
                if budget[0] <= 0:
                    return None
                p = State(x=c[3], y=c[4], up=c[5], yaw_deg=state.yaw_deg)
                if quick_ok(p, t_a):
                    hits.append(c)
        for tt, _, k, x, y, _ in hits:
            if budget[0] <= 0:
                return None
            plan = accept(math.hypot(x - state.x, y - state.y), k, tt, x, y)
            if plan is None:
                continue
            if k >= 0:
                r = math.hypot(x - state.x, y - state.y)
                lower = [q for q in RADII_M if q < r - 1e-9]
                r0 = lower[-1] if lower else 0.0
                ang = 2.0 * math.pi * k / DIRECTIONS
                j = 1
                while r0 + j * REFINE_STEP_M < r - 1e-9 and budget[0] > 0:
                    rr = r0 + j * REFINE_STEP_M
                    better = accept(rr, k, max(rr / speed, tz),
                                    state.x + rr * math.cos(ang),
                                    state.y + rr * math.sin(ang))
                    if better is not None:
                        return better
                    j += 1
            return plan
    return None


def _pose_at(start: State, end: State, tau: float, speed: float, climb: float) -> State:
    """Where the straight transit is `tau` seconds in: each axis at its own
    speed (horizontal at `speed`, vertical at `climb`), as the harness's
    modelled autopilot flies a GUIDED position target."""
    dx, dy, dz = end.x - start.x, end.y - start.y, end.up - start.up
    d = math.hypot(dx, dy)
    fh = 1.0 if d <= 1e-9 else min(1.0, tau * speed / d)
    fz = 1.0 if abs(dz) <= 1e-9 else min(1.0, tau * climb / abs(dz))
    return State(x=start.x + dx * fh, y=start.y + dy * fh, up=start.up + dz * fz,
                 yaw_deg=start.yaw_deg)


def _breached(shield, p: State) -> set[str]:
    """The hard enforced rules a position breaches in the current tick or
    look-ahead view (rule_status 'breach': inside them at that instant)."""
    return {r["id"] for r in shield.rule_status(p) if r["breach"] and _binding_row(r)}


BUDGET_SPENT = "(query budget spent)"


def transit_breaks(shield, start: State, end: State, travel_s: float, speed: float,
                   climb: float, escaping: Iterable[str] = (),
                   budget: list | None = None) -> list[str]:
    """The hard enforced rules the straight transit from `start` to `end`
    breaks on the way: at each sampled pose, at that pose's own time (a
    look-ahead view `tau` seconds on), every rule the pose breaches
    (rule_status 'breach'). [] = a legal transit.

    The one exception is a rule the transit is LEAVING. It must be in
    `escaping` (a stop at the start breaks it within the lookahead), the
    aircraft must already be in it - breached at the start NOW, or not in
    force now and in force over the start by `tau` (a window opening over
    the aircraft) - and the pose must be no deeper in it than a stop at the
    start is at `tau` (`Shield.rule_depths`, the per-rule terms of
    unsafe_depth). A rule in force now that the aircraft is outside of is
    never entered, however soon it would reach a stop at the start: a zone
    or a stand-off ring closing on the aircraft is escaped, never crossed or
    swept through; and a rule the aircraft is in is left by getting
    shallower, never by going through it.

    Until 2026-10-20 a rule of `escaping` was excused at EVERY pose. A stop
    at (0, 0, 15) with a 2 m DynamicNFZ bar 3 m east, closing at 3 m/s, is
    illegal within the lookahead, so the bar was "escaping" and the plan
    flew east THROUGH it to (3, 0): 27 of 50 transit samples inside its
    ring, 0.3 s inside the polygon, the point itself inside the ring on
    arrival, while a clean escape west existed (review of T1c, P2). Excusing
    it only while a stop at the start is in it too was not enough: the plan
    then went north, along the bar, and was swept by it as a stop would be
    (23 samples in the ring). tests/test_hold_points.py
    test_a_transit_never_crosses_a_zone_closing_on_the_aircraft holds both.
    The library's moved holds pass the stricter test: out of a window
    opening over the aircraft (not in force at the plan, over the start from
    the opening on, each pose nearer the edge than the start), up out of a
    breached altitude floor, off a zone the wind pushed the aircraft into
    (docs/DESIGN-hold-points.md has the re-measured table).

    `budget` ([queries left], the search's): each sampled pose costs one; a
    transit the budget runs out on is not a legal one ([BUDGET_SPENT])."""
    esc = set(escaping)
    rows = [r for r in shield.rule_status(start) if _binding_row(r)]
    inside_now = {r["id"] for r in rows if r["breach"]} & esc
    off_now = {r["id"] for r in rows if not r["in_force"]} & esc
    span = max(math.hypot(end.x - start.x, end.y - start.y), abs(end.up - start.up))
    n = max(1, math.ceil(span / TRANSIT_STEP_M - 1e-9))
    for i in range(1, n + 1):
        if budget is not None:
            if budget[0] <= 0:
                return [BUDGET_SPENT]
            budget[0] -= 1
        tau = travel_s * i / n
        p = _pose_at(start, end, tau, speed, climb)
        with shield.looking_ahead(tau):
            hit = _breached(shield, p)
            leaving = hit & inside_now
            opened = (hit & off_now) - leaving
            if opened:
                # In force since the plan: the aircraft is in it only if it
                # came into force over the start.
                leaving |= opened & _breached(shield, start)
            bad = hit - leaving
            if leaving:
                dp = shield.rule_depths(p, leaving)
                ds = shield.rule_depths(start, leaving)
                bad |= {r for r in leaving if dp[r] > ds[r] + DEPTH_TOL_M}
        if bad:
            return sorted(bad)
    return []


def choose_hold_point(shield, state: State, *, unsafe: list | None = None,
                      heading: tuple[float, float] | None = None,
                      search: bool = True) -> HoldPoint:
    """Where the FSM's Loiter holds (module docstring). `unsafe` is the hard
    enforced violations of a stop at `state` (the FSM's stop_illegal), read
    here when not given; `heading` the way the aircraft is going (ties);
    `search` False reports the in-place hold only (Shield(hold_points=False))."""
    look = float(shield.lookahead_s)
    if unsafe is None:
        unsafe = shield.hard_violations(shield.state_is_unsafe(state))
    here = f"({state.x:.1f}, {state.y:.1f}, {state.up:.1f})"
    if not unsafe:
        return HoldPoint("LOITER", "in_place", state.x, state.y, state.up, legal=True,
                         checked_s=look,
                         reason=(f"standing still at {here} breaks no hard rule "
                                 f"within the {look:g} s lookahead"))
    breaks = _ids(unsafe)
    what = (f"standing still at {here} breaks {', '.join(breaks)} within the "
            f"{look:g} s lookahead")
    if not search:
        return HoldPoint("LOITER", "in_place", state.x, state.y, state.up, legal=False,
                         breaks=breaks, checked_s=look,
                         reason=(f"{what}; the hold-point search is off "
                                 f"(Shield(hold_points=False)): held here, as before "
                                 f"2026-10-19"))
    speed, climb, why_speed = _speeds(shield)
    up_t = hold_altitude(shield, state)
    budget = [MAX_QUERIES]

    def stop_ok(p: State) -> bool:
        # Inside a looking_ahead view: a stop at p, from the view's time,
        # breaks no hard enforced rule through the lookahead.
        budget[0] -= 1
        return not shield.hard_violations(shield.state_is_unsafe(p))

    def accept(r: float, k: int, t_a: float, x: float, y: float) -> HoldPoint | None:
        """The candidate at (x, y, up_t), reached in t_a, if it is legal with
        its HOLD_MARGIN_M neighbours and by a legal transit."""
        p = State(x=x, y=y, up=up_t, yaw_deg=state.yaw_deg)
        with shield.looking_ahead(t_a):
            if not stop_ok(p) or not all(stop_ok(q) for q in _neighbours(p)):
                return None
        if transit_breaks(shield, state, p, t_a, speed, climb, breaks, budget):
            return None
        return HoldPoint(
            "LOITER", "moved", p.x, p.y, p.up, legal=True, travel_s=t_a,
            speed_mps=speed, climb_mps=climb, checked_s=look,
            reason=(f"{what}; ({p.x:.1f}, {p.y:.1f}, {p.up:.1f}) is the nearest "
                    f"point where a stop is legal from arrival ({t_a:.2f} s) "
                    f"through the lookahead ({HOLD_MARGIN_M:g} m about it too), "
                    f"reached in a straight line that enters no rule a stop here would "
                    f"not break then, and goes no deeper into one ({why_speed})"))

    plan = _search(shield, state, speed, climb, up_t, heading,
                   lambda p, t_a: stop_ok(p), accept, budget)
    if plan is not None:
        return plan
    capped = budget[0] <= 0
    limit = (f"the search's {MAX_QUERIES}-query budget" if capped
             else f"{RADII_M[-1]:g} m")
    return HoldPoint("LOITER", "in_place", state.x, state.y, state.up, legal=False,
                     breaks=breaks, checked_s=look,
                     reason=(f"{what}; no point within {limit} is legal from arrival "
                             f"through the lookahead by a legal transit "
                             f"({why_speed}): held here, and searched again in "
                             f"a second"))


def _corridor_out(shield, x: float, y: float, up: float, ahead_s: float) -> list[str]:
    """Hard keep-in corridors in force whose footprint does not hold (x, y)."""
    return sorted({r["id"] for r in shield.rule_status(State(x=x, y=y, up=up),
                                                       ahead_s=ahead_s)
                   if r["type"] in _TUBE_TYPES and r["in_force"] and _binding_row(r)
                   and r["distance_m"] is not None and r["distance_m"] < -1e-9})


def landing_breaks(shield, x: float, y: float, up0: float, arrive_s: float,
                   descent_mps: float = LAND_DESCENT_MPS,
                   band_edges: Iterable[float] = ()) -> list[str]:
    """The rules a descent from `up0` at (x, y), starting `arrive_s` after
    this tick, breaks that a site could avoid (module docstring, "landing").
    Judged in lookahead-long chunks to touchdown and one lookahead past it,
    at each chunk's altitudes. [] = a legal landing site."""
    look = float(shield.lookahead_s)
    up0 = max(0.0, float(up0))
    descent_s = up0 / descent_mps
    edges = sorted(set(band_edges))
    n = max(1, math.ceil(descent_s / look - 1e-9))
    for j in range(n + 1):
        hi = max(0.0, up0 - descent_mps * j * look)
        lo = max(0.0, up0 - descent_mps * (j + 1) * look)
        alts = sorted({hi, lo} | {e for e in edges if lo < e < hi}, reverse=True)
        with shield.looking_ahead(arrive_s + j * look):
            for alt in alts:
                vios = [v for v in shield.hard_violations(
                            shield.state_is_unsafe(State(x=x, y=y, up=alt)))
                        if v.category in LAND_SITE_CATEGORIES]
                if vios:
                    return list(_ids(vios))
    return _corridor_out(shield, x, y, up0, arrive_s)


def _floor_rules(shield, state: State) -> list[str]:
    """Hard enforced altitude floors in force above the ground: a landing
    breaks them wherever it is flown (named in the reason, never a site
    criterion)."""
    out = set()
    for row in shield.rule_status(state):
        if not row["in_force"] or not _binding_row(row):
            continue
        c = row["rule"]
        if row["type"] == "altitude_envelope" and c.alt_min_m > 0:
            out.add(row["id"])
        elif row["type"] in _TUBE_TYPES and c.altitude_floor_m > 0:
            out.add(row["id"])
    return sorted(out)


def _map_test(site_ok: Callable[[float, float], Any] | None
              ) -> Callable[[float, float], bool] | None:
    """`site_ok`'s answer as a plain bool. The city demo's map test,
    demo/landing.is_landable(maps, x, y), answers a TUPLE (ok, reasons), and a
    bare bool() of a non-empty tuple is always True: passed as
    `functools.partial(is_landable, maps)` it would have approved every site
    the map refuses (found in review, 2026-10-20). A tuple is read by its
    first element; anything else by its truth."""
    if site_ok is None:
        return None

    def ok(x: float, y: float) -> bool:
        v = site_ok(x, y)
        if isinstance(v, tuple):
            return bool(v[0]) if v else False
        return bool(v)
    return ok


def choose_landing_site(shield, state: State, *, unsafe: list | None = None,
                        heading: tuple[float, float] | None = None,
                        search: bool = True,
                        site_ok: Callable[[float, float], Any] | None = None,
                        descent_mps: float = LAND_DESCENT_MPS) -> HoldPoint:
    """Where the FSM's Land lands (module docstring). The policy alone decides
    unless `site_ok(x, y)` is given - a map-based test such as the city
    demo's demo/landing.is_landable (`functools.partial(is_landable, maps)`;
    its (ok, reasons) answer is read by `ok`, _map_test) - which must also
    pass."""
    site_ok = _map_test(site_ok)
    look = float(shield.lookahead_s)
    if unsafe is None:
        unsafe = shield.hard_violations(shield.state_is_unsafe(state))
    descent_s = max(0.0, state.up) / descent_mps
    checked = descent_s + look
    edges = sorted({e for r in shield.ir.fences
                    for e in (r.rule.altitude_floor_m, r.rule.altitude_ceiling_m)})
    floors = _floor_rules(shield, state)
    floor_note = (f"; the descent breaks {', '.join(floors)}'s floor wherever it is "
                  f"flown, so altitude does not choose the site" if floors else "")
    here = f"({state.x:.1f}, {state.y:.1f})"
    bad_here = landing_breaks(shield, state.x, state.y, state.up, 0.0, descent_mps, edges)
    map_ok = site_ok is None or site_ok(state.x, state.y)
    if not bad_here and map_ok:
        return HoldPoint("LAND", "in_place", state.x, state.y, state.up, legal=True,
                         checked_s=checked,
                         reason=(f"a descent at {descent_mps:g} m/s at {here} breaks no "
                                 f"keep-out zone, stand-off, clearance or corridor "
                                 f"footprint through touchdown ({descent_s:.1f} s) and "
                                 f"the lookahead after it{floor_note}"))
    why_here = (f"landing at {here} breaks {', '.join(bad_here)} during the descent"
                if bad_here else f"the landing map refuses {here}")
    if not search:
        return HoldPoint("LAND", "in_place", state.x, state.y, state.up, legal=False,
                         breaks=tuple(bad_here), checked_s=checked,
                         reason=(f"{why_here}; the landing-site search is off "
                                 f"(Shield(hold_points=False)): landed here, as before "
                                 f"2026-10-19{floor_note}"))
    speed, climb, why_speed = _speeds(shield)
    escaping = _ids(unsafe)
    budget = [MAX_QUERIES]

    def probe(x: float, y: float, alt: float) -> bool:
        # Inside a looking_ahead view: no site-choosable rule broken by a
        # stop at (x, y, alt) from the view's time through the lookahead.
        budget[0] -= 1
        return not [v for v in shield.hard_violations(
                        shield.state_is_unsafe(State(x=x, y=y, up=alt)))
                    if v.category in LAND_SITE_CATEGORIES]

    def quick(x: float, y: float, t_a: float) -> bool:
        """The top of the descent on arrival and the ground at touchdown;
        made inside the arrival view (the second in its own)."""
        if not probe(x, y, state.up):
            return False
        with shield.looking_ahead(t_a + descent_s):
            return probe(x, y, 0.0)

    def accept(r: float, k: int, t_a: float, x: float, y: float) -> HoldPoint | None:
        # The map first: it is the cheap test, and a site it refuses costs
        # no Shield query at all.
        if site_ok is not None and not site_ok(x, y):
            return None
        with shield.looking_ahead(t_a):
            if not quick(x, y, t_a):
                return None
            for q in _neighbours(State(x=x, y=y, up=state.up)):
                if not quick(q.x, q.y, t_a):
                    return None
        budget[0] -= 2 * (math.ceil(descent_s / look) + 1)
        if landing_breaks(shield, x, y, state.up, t_a, descent_mps, edges):
            return None
        p = State(x=x, y=y, up=state.up, yaw_deg=state.yaw_deg)
        if transit_breaks(shield, state, p, t_a, speed, climb, escaping, budget):
            return None
        return HoldPoint(
            "LAND", "moved", x, y, state.up, legal=True, travel_s=t_a,
            speed_mps=speed, climb_mps=climb, checked_s=checked,
            reason=(f"{why_here}; ({x:.1f}, {y:.1f}) is the nearest site whose "
                    f"descent at {descent_mps:g} m/s from arrival ({t_a:.2f} s) "
                    f"breaks none of them ({HOLD_MARGIN_M:g} m about it too), "
                    f"reached at the current altitude in a straight line that "
                    f"enters no rule a stop here would not break then, and goes "
                    f"no deeper into one ({why_speed}){floor_note}"))

    def quick_ok(p: State, t_a: float) -> bool:
        if site_ok is not None and not site_ok(p.x, p.y):
            return False
        return quick(p.x, p.y, t_a)

    plan = _search(shield, state, speed, climb, state.up, heading, quick_ok, accept,
                   budget, vertical=False)
    if plan is not None:
        return plan
    capped = budget[0] <= 0
    limit = (f"the search's {MAX_QUERIES}-query budget" if capped
             else f"{RADII_M[-1]:g} m")
    return HoldPoint("LAND", "in_place", state.x, state.y, state.up, legal=False,
                     breaks=tuple(bad_here), checked_s=checked,
                     reason=(f"{why_here}; no site within {limit} is legal by a "
                             f"legal transit ({why_speed}): landed here{floor_note}"))
