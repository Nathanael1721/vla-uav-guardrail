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
  transit  At EVERY instant of the straight transit (each axis at its own
           speed, as the harness's modelled autopilot flies it), against the
           rules as they are then (a look-ahead view: windows read then,
           moving zones and a moving subject where they will be), no hard
           enforced rule is breached except one the aircraft is LEAVING: a
           rule a stop at the start breaks within the lookahead that the
           aircraft is already in - breached at the start now, or not in
           force now and in force over the start by then (a window opening
           over it) - and that one the pose is no deeper in than a stop at
           the start is then, to LEAVE_TOL_M. A rule in force that the
           aircraft is outside of is never entered - a zone or a stand-off
           ring closing on it is escaped, never crossed or swept through -
           and one it is in is left by getting shallower, never by going
           through it. Between the poses it evaluates, transit_breaks BOUNDS
           each rule's room (Shield.rule_clearances) by how fast the pose
           moves relative to the rule (Shield.rule_motion), so the claim is
           true along the whole line, not at chosen poses; what it cannot
           show clear (within CERT_RES_M of a limit) it refuses.
           (Until 2026-10-20 a rule a stop at the start breaks was excused at
           every pose, and a transit could cross a zone closing on the
           aircraft; then poses were judged 1 m apart and a thin or fast zone
           could pass between two of them: transit_breaks.)

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
  3. Nothing legal within the budget (MAX_QUERIES stop-query units, each
     Shield call charged by what it costs, _charges) or 60 m: an in-place
     plan marked ILLEGAL that names what it breaks and, when the budget
     stopped it, says so (the FSM then searches again once a second,
     fsm.HOLD_REPLAN_S). A count, not a clock, so the answer does not
     depend on the machine's load; sized so the fruitless search stays
     inside the 100 ms tick (constants, "THE SEARCH'S BUDGET").

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
BAND_INSET_M = 0.5
# THE TRANSIT TEST'S GUARANTEE (transit_breaks). Until 2026-10-20 a transit
# was judged at poses 1 m apart along its longer axis, evenly spaced in time,
# so the claim "a zone it is outside of is never entered" held at those poses
# only. Review of T1c (re-review P1) reproduced three ways through: a 1 m bar
# closing at 6 m/s crossed between two samples of a 0.5 s transit (159 of 400
# dense poses in its ring, 79 in the polygon); a climb-dominated transit whose
# horizontal samples, 1.875 m apart, straddled a 0.8 m wall's 1.8 m ring (22
# of 400 in the polygon); and a horizontal leg that ended before the first
# sample went 0.72 m deeper into the zone it was leaving. A fuzz with thin
# zones found 27 of 195 moved holds entering a zone they started outside of.
# Now every rule's room (Shield.rule_clearances) is BOUNDED between two
# evaluated poses: it changes no faster than the pose moves relative to the
# rule (the rule's own motion and turn included, Shield.rule_motion), so an
# interval is clear when that bound stays above the limit, and is halved
# where it does not. The claim holds at every instant of the transit, not at
# chosen poses; what cannot be shown clear is refused.
#
# An interval is refused, not halved again, once the bound's slack - how far
# the room can dip between its two ends - is under CERT_RES_M: the transit
# would pass within a centimetre of a rule's limit, which the search treats
# as touching it. LEAVE_TOL_M is the one tolerance in the claim: a transit
# out of a rule the aircraft is in may be up to that much deeper than a stop
# at the start is at the same instant. It has to be above zero - the transit
# starts AT the start, as deep as the stop, and an oblique way out is
# shallower only to second order - and equal to CERT_RES_M, so the test ends
# (review of T1c: "never goes deeper" held at the samples only).
CERT_RES_M = 0.01
LEAVE_TOL_M = 0.01
# The longest interval judged in one piece before any halving. A rule that
# is in force at neither end of an interval is skipped there, which is sound
# because whether a rule is in force changes only at whole minutes (windows
# are "HH:MM", guardrail/shield.py _Schedule), so no in-force stretch is
# shorter than a minute and none can hide inside 30 s. (A zero-length window
# - start == end - is an instant the Shield's own forecast does not see
# either; _Schedule's docstring.)
MAX_INTERVAL_S = 30.0
# How fast a still rule's horizontal room changes per metre moved is the
# Shield's to say (RuleClearance.h_lip): 1 for a distance to a set, and for
# obstacle clearance the distance field's bound measured from the field
# itself (shield._field_lipschitz). Until 2026-10-20 (evening) this file
# assumed sqrt(2) for the clearance field, which a building face (a jump of
# 1.92 cells per cell) and the off-map branch of Shield._distance_at both
# exceed: 2.72 m per metre on a map with buildings.
#
# What "deeper" means is the Shield's too: rule_depths' term (_depth), so a
# corridor's lateral and vertical excess ADD. Until 2026-10-20 (evening) a
# keep-in rule's depth was taken as its largest single excess, and a transit
# out from under a corridor could drift wider of it while the climb hid the
# drift: 11 of 151 random transits the test passed went up to metres deeper
# by rule_depths (re-review of the T1c fix; tests/test_hold_transit.py).
# THE SEARCH'S BUDGET bounds its time on the tick it runs (the grant's
# 100 ms monitor tick). It is a count, never a clock - a sweep is
# reproducible on any machine - in units of a STOP QUERY in a world with no
# moving zone near, each Shield call charged in proportion to what it costs.
# Measured on the desktop, 2026-10-20, best of 5 (the ratios are re-measured
# by tests/test_hold_transit.py test_the_budget_bounds_the_search_inside_
# the_tick):
#
#   stop query (state_is_unsafe at the grant's 5 s / 0.1 s)   0.12-0.20 ms
#     + per moving zone the search can meet                  +0.16-0.20 ms
#   transit evaluation (rule_clearances of the pose and of
#     the start, every rule)                                  0.04-0.05 ms
#     + per moving zone                                       +0.07-0.08 ms
#
# so a stop query costs 1 + MOVING_ZONE_STOP_COST per moving zone, a transit
# evaluation TRANSIT_EVAL_COST + MOVING_ZONE_EVAL_COST per moving zone
# (_charges counts the zones), and a unit is at most about 0.2 ms where the
# zones are few and broad, up to 0.27 ms with several thin bars near the
# aircraft (static ones too: a stop query tests a zone at every one of its
# 51 forecast poses whenever the point is in the zone's bounding box). At
# MAX_QUERIES = 300 the fruitless search took 29-49 ms in the review's
# worlds (bars closing at 3-6 m/s, the thin wall, a zone opening over the
# aircraft and reaching past 60 m), 20-24 ms at the city demo's 3 s / 0.5 s,
# 16 ms in a courtyard walled by a clearance map, and at most 81 ms in the
# seeded fuzz of tests/test_hold_transit.py (desktop, best of 3, two
# measurements on 2026-10-20). The library's dearest plan spends 176 units
# (time-window-opens-on-vehicle, 42-44 ms).
#
# History (review of T1c): until 2026-10-20 only stop queries were counted;
# then a transit pose counted as a whole stop query, out of 800 - and the
# fruitless search against a bar closing at 3 m/s took 127-135 ms, over the
# tick (re-review P3), and 0.5 s in a fuzz world with three bars near.
MAX_QUERIES = 300
MOVING_ZONE_STOP_COST = 1.0
TRANSIT_EVAL_COST = 0.4
MOVING_ZONE_EVAL_COST = 0.5
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


def _charges(shield, state: State, horizon_s: float) -> tuple[float, float, int]:
    """(a stop query's charge, a transit evaluation's, the moving zones
    counted) for one search from `state` whose queries look up to
    `horizon_s` ahead (MAX_QUERIES, "THE SEARCH'S BUDGET"). A moving zone
    counts when the search can meet it: it turns (a turning zone reaches as
    far as it is long, which the Shield's motion model does not report), or
    its margin ring is no farther than the search's reach plus how far it
    moves within the horizon. Soft and monitor-only zones count too: the
    stop query checks every zone. A moving subject's ring is one distance
    per pose and is not counted."""
    motion = shield.rule_motion()
    if not motion:
        return 1.0, TRANSIT_EVAL_COST, 0
    horizon = float(horizon_s)
    room = shield.rule_clearances(state, motion)
    m = 0
    for rid, (ux, uy, w, _, _) in motion.items():
        c = room.get(rid)
        if c is None or not c.keep_out:
            continue
        if w or c.h <= RADII_M[-1] + math.hypot(ux, uy) * horizon:
            m += 1
    return (1.0 + MOVING_ZONE_STOP_COST * m, TRANSIT_EVAL_COST + MOVING_ZONE_EVAL_COST * m, m)


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
    `accept(r, k, t_a, x, y, checked)` the full one (a plan, or None);
    `checked` is True for a candidate quick_ok already passed, so accept
    does not repeat that test (until 2026-10-20 every hit's own stop query
    was made twice, once here and once in accept, at the same time). On the
    first accepted candidate of a ring, its heading is searched again between
    the previous ring and this one every REFINE_STEP_M, and the nearest
    accepted point wins. `budget` ([stop-query equivalents left]) bounds the
    whole search."""
    cands = _candidates(state, speed, climb, up_t, heading, vertical)
    tz = abs(up_t - state.up) / climb if abs(up_t - state.up) > 1e-9 else 0.0
    # Candidates are judged one at a time, in order: the quick test in a view
    # at the arrival time, then, if it passes, the full one (outside the
    # view: accept reads the current tick too). The first candidate passing
    # both wins, as when a whole travel-time group was quick-tested first
    # and its hits accepted in the same order - but the rest of the group is
    # no longer paid for. Every point within climb time shares the vertical
    # candidate's arrival time, so a hold below the floor (three-
    # simultaneous-violations, 3 m under the 10 m floor) stop-queried 193
    # candidates before taking the first, straight up (2026-10-20). One view
    # serves a group's quick tests until a hit leaves it (a view materialises
    # every moving zone again).
    i, n = 0, len(cands)
    while i < n:
        t_a = round(cands[i][0], 9)
        hit = None
        with shield.looking_ahead(t_a):
            while i < n and round(cands[i][0], 9) == t_a:
                c = cands[i]
                i += 1
                if budget[0] <= 0:
                    return None
                if quick_ok(State(x=c[3], y=c[4], up=c[5], yaw_deg=state.yaw_deg), t_a):
                    hit = c
                    break
        if hit is not None:
            tt, _, k, x, y, _ = hit
            if budget[0] <= 0:
                return None
            plan = accept(math.hypot(x - state.x, y - state.y), k, tt, x, y, True)
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


BUDGET_SPENT = "(query budget spent)"


class _Spent(Exception):
    """The search's budget ran out inside a transit check."""


def _depth(c) -> float:
    """How deep a position is in a rule (its RuleClearance `c`), exactly as
    Shield.rule_depths measures it: a zone's depth inside its margin ring
    while the position is within its band; a keep-in rule's horizontal
    excess PLUS its vertical excess (a corridor wide of its centreline and
    under its floor is as deep as both together, unsafe_depth's term); 0.0
    where the position is not past the rule's limits."""
    if c.keep_out:
        return max(0.0, -c.h) if (c.lo >= 0.0 and c.hi >= 0.0) else 0.0
    return max(0.0, -c.h) + max(0.0, -c.lo, -c.hi)


def transit_breaks(shield, start: State, end: State, travel_s: float, speed: float,
                   climb: float, escaping: Iterable[str] = (),
                   budget: list | None = None,
                   eval_cost: float | None = None) -> list[str]:
    """The hard enforced rules the straight transit from `start` to `end`
    breaks on the way, judged at EVERY instant of it - not at sampled poses
    (module constants, "THE TRANSIT TEST'S GUARANTEE"). [] = a legal transit.

    WHAT IS REQUIRED. For each hard enforced rule with position geometry
    (Shield.rule_clearances), at every instant tau of the transit at which
    the rule is in force, the pose then (_pose_at, each axis at its own
    speed) against the rule then (a look-ahead view tau seconds on: windows
    read then, moving zones and a moving subject where they will be):

      * a rule the aircraft is outside of is never entered (room > 0) - a
        zone or a stand-off ring closing on it is escaped, never crossed or
        swept through;
      * a rule the aircraft is LEAVING is left by getting shallower: the pose
        is no deeper in it than a stop at the start is at that instant, to
        LEAVE_TOL_M; once a stop at the start would be out of it, the pose
        must be out too. "Leaving" means in `escaping` (a stop at the start
        breaks it within the lookahead) AND either breached at the start NOW
        or not in force now (a window that opens over it on the way: until
        it is over the start, the pose may not enter it either).

    HOW. The transit is a sequence of straight pieces in time (the
    horizontal leg and the vertical leg end at different times). On an
    interval inside one piece the pose moves at a constant velocity, and each
    part of a rule's room (RuleClearance) changes no faster than:

      h       the pose's speed RELATIVE to the rule - minus a moving zone's
              or subject's velocity, plus its turn rate times the largest
              distance to its rotation centre over the interval (the norm of
              a linear motion is convex, so that is at an end) - times the
              rule's own RuleClearance.h_lip (1 for a distance to a set, the
              clearance field's measured bound for obstacle clearance);
      lo, hi  linear in time (the pose climbs at a constant rate; floors and
              ceilings stand still), so their minimum is at an end, and a
              keep-in rule's vertical excess max(0, -lo, -hi) is convex, so
              its maximum is;

    and the start's depth in a moving rule no faster than the rule moves
    past it. "Deeper" is Shield.rule_depths' measure (_depth: a corridor's
    lateral and vertical excess add). From the two ends that gives a lower
    bound on the margin over the whole interval (interval_ok, below). An
    interval whose bound clears
    every rule is done; one that does not is halved and both halves judged,
    until an evaluated pose breaks a rule (refused, named) or the bound's
    slack is under CERT_RES_M (refused: it cannot be shown clear). Intervals are
    judged in time order, so a refusal costs only the poses up to it. A rule
    in force at neither end of an interval is not judged there
    (MAX_INTERVAL_S says why that is sound).

    WHAT IT REPLACED. Until 2026-10-20 (morning) a rule of `escaping` was
    excused at EVERY pose: with a 2 m DynamicNFZ bar 3 m east of a legal
    aircraft at (0, 0, 15), closing at 3 m/s, the plan flew east THROUGH it
    to (3, 0) (review of T1c, P2). Then (afternoon) poses were sampled 1 m
    apart along the longer axis, and the claim held only at them (re-review
    P1): a 1 m bar closing at 6 m/s was crossed between two samples of a
    0.5 s transit, a climb-dominated transit's 1.875 m horizontal samples
    straddled a 0.8 m wall's ring, and a horizontal leg that ended before the
    first sample went 0.72 m deeper into the zone it was leaving.
    tests/test_hold_transit.py pins each, and a seeded fuzz of thin, fast
    and turning zones and walking subjects checks the plans 1000 poses a
    transit; tests/test_hold_points.py test_a_transit_never_crosses_a_zone_
    closing_on_the_aircraft keeps the first.

    Cost. A rule whose room cannot reach its limit in the rest of the
    transit, by the same bound from the last whole interval's end, is no
    longer read; and a stop at the start is read again only for a moving
    rule being left (a still one's room at the start does not change).

    `budget` ([stop-query equivalents left], the search's): reading the
    start and each evaluated pose costs `eval_cost` (TRANSIT_EVAL_COST when
    not given; the search passes its world's, _charges); a transit the budget
    runs out on is not a legal one ([BUDGET_SPENT])."""
    T = float(travel_s)
    dx, dy, dz = end.x - start.x, end.y - start.y, end.up - start.up
    d = math.hypot(dx, dy)
    if T <= 1e-12 or (d <= 1e-9 and abs(dz) <= 1e-9):
        return []
    esc = set(escaping)
    cost = TRANSIT_EVAL_COST if eval_cost is None else float(eval_cost)

    def charge() -> None:
        if budget is not None:
            if budget[0] <= 0:
                raise _Spent
            budget[0] -= cost

    try:
        charge()
        types = {r["id"]: r["type"] for r in shield.rule_status(start) if _binding_row(r)}
        c0 = shield.rule_clearances(start, types)
    except _Spent:
        return [BUDGET_SPENT]
    if not c0:
        return []
    leave = frozenset(rid for rid in c0 if rid in esc
                      and (c0[rid].breach or not c0[rid].in_force))
    motion = shield.rule_motion(c0)
    leave_still = {rid: c0[rid] for rid in leave if rid not in motion}
    live = set(c0)                      # the rules still read (Cost, above)
    th = d / speed if d > 1e-9 else 0.0
    tz = abs(dz) / climb if abs(dz) > 1e-9 else 0.0
    vh = (dx / d * speed, dy / d * speed) if d > 1e-9 else (0.0, 0.0)
    vz = math.copysign(climb, dz) if abs(dz) > 1e-9 else 0.0

    def velocity(tm: float) -> tuple[float, float, float]:
        """The pose's (vx, vy, vz) on the piece holding time tm."""
        h = vh if tm < th else (0.0, 0.0)
        return h[0], h[1], (vz if tm < tz else 0.0)

    def rates(rid: str, a: float, b: float, pa: State, pb: State,
              v: tuple[float, float, float]) -> tuple[float, float]:
        """How fast rule `rid`'s horizontal room can change on [a, b]: for
        the pose (moving at v) and for a stop at the start (still)."""
        mv = motion.get(rid)
        if mv is None:
            return math.hypot(v[0], v[1]) * c0[rid].h_lip, 0.0
        ux, uy, w, cx, cy = mv
        lp = math.hypot(v[0] - ux, v[1] - uy)
        ls = math.hypot(ux, uy)
        if w:
            ca = (cx + ux * a, cy + uy * a)
            cb = (cx + ux * b, cy + uy * b)
            lp += abs(w) * max(math.hypot(pa.x - ca[0], pa.y - ca[1]),
                               math.hypot(pb.x - cb[0], pb.y - cb[1]))
            ls += abs(w) * max(math.hypot(start.x - ca[0], start.y - ca[1]),
                               math.hypot(start.x - cb[0], start.y - cb[1]))
        return lp, ls

    def sample(tau: float):
        """(tau, pose, its rule readings, the start's for rules being left)
        - the live rules only, in a view tau seconds on."""
        charge()
        p = _pose_at(start, end, tau, speed, climb)
        ids = sorted(live)
        moving_left = [rid for rid in ids if rid in leave and rid not in leave_still]
        with shield.looking_ahead(tau):
            cp = shield.rule_clearances(p, ids)
            cs = shield.rule_clearances(start, moving_left) if moving_left else {}
        cs.update((rid, c) for rid, c in leave_still.items() if rid in cp)
        return tau, p, cp, cs

    def pose_breaks(s) -> list[str]:
        """The rules the evaluated pose `s` itself breaks."""
        _, _, cp, cs = s
        out = []
        for rid, c in cp.items():
            if not c.in_force:
                continue
            d_s = _depth(cs[rid]) if rid in leave else 0.0
            if d_s > 0.0:
                if _depth(c) > d_s + LEAVE_TOL_M:
                    out.append(rid)          # deeper than a stop at the start
            elif c.breach:
                out.append(rid)
        return out

    def margin_lb(x, y, lbh: float, d_lb: float) -> tuple[float, float]:
        """(a lower bound on one rule's margin over an interval, the part of
        the bound's slack its vertical parts add) from its readings `x`, `y`
        at the ends, `lbh` the horizontal room's lower bound there and
        `d_lb` the start's depth's (0.0: the rule is not being left, or the
        start may be out of it by then - the pose must then stay out). The
        margin is the room (> 0 clear) for a rule not being left, and the
        start's depth minus the pose's (>= -LEAVE_TOL_M clear) for one that
        is. A zone's vertical parts only ever help (out of its band is out of
        it); a keep-in rule's vertical excess is at most its larger end value
        (convex), which overstates it by up to half the ends' difference."""
        if x.keep_out:
            return max(lbh + d_lb, -max(x.lo, y.lo), -max(x.hi, y.hi)), 0.0
        if d_lb <= 0.0:
            return min(lbh, min(x.lo, y.lo), min(x.hi, y.hi)), 0.0
        va, vb = max(0.0, -x.lo, -x.hi), max(0.0, -y.lo, -y.hi)
        return d_lb - max(0.0, -lbh) - max(va, vb), 0.5 * abs(va - vb)

    def interval_ok(sa, sb) -> tuple[list[str], list[str]]:
        """(rules not shown clear on [a, b], those of them at resolution);
        the rules both ends read (one dropped since `a` was read cannot be
        reached after it)."""
        a, pa, ca, csa = sa
        b, pb, cb, csb = sb
        dt = b - a
        v = velocity(0.5 * (a + b))
        unproven, final = [], []
        for rid, y in cb.items():
            x = ca[rid]
            if not (x.in_force or y.in_force):
                continue
            lp, ls = rates(rid, a, b, pa, pb, v)
            lbh = 0.5 * (x.h + y.h - lp * dt) if math.isfinite(x.h) else math.inf
            d_lb = 0.0
            if rid in leave:
                da, db = _depth(csa[rid]), _depth(csb[rid])
                d_lb = max(0.0, 0.5 * (da + db - ls * dt))
            lb, v_slack = margin_lb(x, y, lbh, d_lb)
            ok = lb >= -LEAVE_TOL_M if d_lb > 0.0 else lb > 0.0
            if not ok:
                unproven.append(rid)
                # How far the bound can sit below the ends' own margins: the
                # horizontal rates' share and, for a keep-in rule being left,
                # its vertical excess' spread; halving shrinks both. The
                # other vertical parts are exact at the ends (linear).
                slack = (0.5 * (lp + ls) * dt if math.isfinite(x.h) else 0.0) + v_slack
                if slack <= CERT_RES_M or dt <= 1e-6:
                    final.append(rid)
        return unproven, final

    cuts = sorted({0.0, T} | {t for t in (th, tz) if 1e-9 < t < T - 1e-9})
    at_cut = {t: _pose_at(start, end, t, speed, climb) for t in cuts}
    up_end = at_cut[T].up

    def unreachable(s) -> list[str]:
        """The live rules that cannot reach their limit after sample `s`:
        the same bound as interval_ok, from `s` to the end of the transit
        (the fastest the room can change on any remaining piece, the
        vertical parts at their exact extremes - the climb is monotonic)."""
        b, pb, cb, csb = s
        rem = T - b
        dup = up_end - pb.up
        out = []
        for rid, c in cb.items():
            lp = ls = 0.0
            for a0, b0 in zip(cuts, cuts[1:]):
                if b0 <= b + 1e-12:
                    continue
                a1 = max(a0, b)
                p1 = pb if a1 == b else at_cut[a0]
                r = rates(rid, a1, b0, p1, at_cut[b0], velocity(0.5 * (a1 + b0)))
                lp, ls = max(lp, r[0]), max(ls, r[1])
            d_lb = (max(0.0, _depth(csb[rid]) - ls * rem) if rid in leave else 0.0)
            h_lb = c.h - lp * rem
            # The vertical parts' worst values over the rest, as a second
            # reading for margin_lb: the climb is monotonic, so each is at `s`
            # or at the end - for a zone the most IN its band, for a keep-in
            # rule the least room.
            if c.keep_out:
                ext = c._replace(lo=c.lo + max(0.0, dup), hi=c.hi - min(0.0, dup))
            else:
                ext = c._replace(lo=c.lo + min(0.0, dup), hi=c.hi - max(0.0, dup))
            if margin_lb(c, ext, h_lb, d_lb)[0] > 0.0:
                out.append(rid)
        return out

    times = []
    for a, b in zip(cuts, cuts[1:]):
        n = max(1, math.ceil((b - a) / MAX_INTERVAL_S - 1e-9))
        times += [a + (b - a) * k / n for k in range(1, n)] + [b]
    first = (0.0, start, c0, {rid: c0[rid] for rid in leave})
    live.difference_update(unreachable(first))
    try:
        prev = first
        for t in times:
            if not live:
                break
            stack = [(prev, sample(t))]
            nxt = stack[0][1]
            while stack:
                sa, sb = stack.pop()
                bad = pose_breaks(sb)
                if bad:
                    return sorted(bad)
                unproven, final = interval_ok(sa, sb)
                if final:
                    return sorted(final)
                if unproven:
                    sm = sample(0.5 * (sa[0] + sb[0]))
                    stack.append((sm, sb))
                    stack.append((sa, sm))
            live.difference_update(unreachable(nxt))
            prev = nxt
    except _Spent:
        return [BUDGET_SPENT]
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
    tz = abs(up_t - state.up) / climb
    stop_cost, eval_cost, _ = _charges(shield, state,
                                       max(RADII_M[-1] / speed, tz) + look)

    def stop_ok(p: State) -> bool:
        # Inside a looking_ahead view: a stop at p, from the view's time,
        # breaks no hard enforced rule through the lookahead.
        budget[0] -= stop_cost
        return not shield.hard_violations(shield.state_is_unsafe(p))

    def accept(r: float, k: int, t_a: float, x: float, y: float,
               checked: bool = False) -> HoldPoint | None:
        """The candidate at (x, y, up_t), reached in t_a, if it is legal with
        its HOLD_MARGIN_M neighbours and by a legal transit. `checked`: the
        search already made this point's stop query, in a view at the same
        time (_search's hits), so it is not made again. The transit goes
        before the four neighbour queries: both must pass, so the order
        changes no answer, and a transit that fails usually fails in a pose
        or two where the neighbours cost four stop queries (a bar closing on
        the aircraft makes every point behind it a stop-legal candidate that
        only the transit refuses)."""
        p = State(x=x, y=y, up=up_t, yaw_deg=state.yaw_deg)
        if not checked:
            with shield.looking_ahead(t_a):
                if not stop_ok(p):
                    return None
        if transit_breaks(shield, state, p, t_a, speed, climb, breaks, budget, eval_cost):
            return None
        with shield.looking_ahead(t_a):
            if not all(stop_ok(q) for q in _neighbours(p)):
                return None
        return HoldPoint(
            "LOITER", "moved", p.x, p.y, p.up, legal=True, travel_s=t_a,
            speed_mps=speed, climb_mps=climb, checked_s=look,
            reason=(f"{what}; ({p.x:.1f}, {p.y:.1f}, {p.up:.1f}) is the nearest "
                    f"point where a stop is legal from arrival ({t_a:.2f} s) "
                    f"through the lookahead ({HOLD_MARGIN_M:g} m about it too), "
                    f"reached in a straight line that, at every instant on the "
                    f"way, enters no hard rule the aircraft is outside of and is "
                    f"no deeper (to {LEAVE_TOL_M:g} m) in one it is in than a stop "
                    f"here would be then ({why_speed})"))

    plan = _search(shield, state, speed, climb, up_t, heading,
                   lambda p, t_a: stop_ok(p), accept, budget)
    if plan is not None:
        return plan
    capped = budget[0] <= 0
    limit = (f"the search's budget of {MAX_QUERIES:g} stop queries" if capped
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
    checked_s = descent_s + look
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
                         checked_s=checked_s,
                         reason=(f"a descent at {descent_mps:g} m/s at {here} breaks no "
                                 f"keep-out zone, stand-off, clearance or corridor "
                                 f"footprint through touchdown ({descent_s:.1f} s) and "
                                 f"the lookahead after it{floor_note}"))
    why_here = (f"landing at {here} breaks {', '.join(bad_here)} during the descent"
                if bad_here else f"the landing map refuses {here}")
    if not search:
        return HoldPoint("LAND", "in_place", state.x, state.y, state.up, legal=False,
                         breaks=tuple(bad_here), checked_s=checked_s,
                         reason=(f"{why_here}; the landing-site search is off "
                                 f"(Shield(hold_points=False)): landed here, as before "
                                 f"2026-10-19{floor_note}"))
    speed, climb, why_speed = _speeds(shield)
    escaping = _ids(unsafe)
    budget = [MAX_QUERIES]
    stop_cost, eval_cost, _ = _charges(shield, state, RADII_M[-1] / speed + checked_s)

    def probe(x: float, y: float, alt: float) -> bool:
        # Inside a looking_ahead view: no site-choosable rule broken by a
        # stop at (x, y, alt) from the view's time through the lookahead.
        budget[0] -= stop_cost
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

    def accept(r: float, k: int, t_a: float, x: float, y: float,
               checked: bool = False) -> HoldPoint | None:
        # The map first: it is the cheap test, and a site it refuses costs
        # no Shield query at all. Then, as for a hold (choose_hold_point's
        # accept), the site's own quick test only if the search has not
        # made it (`checked`), and the transit - usually a pose or two -
        # before the neighbours' eight probes and the descent's chunks: all
        # must pass, so the order changes no answer, only what a refused
        # site costs.
        if site_ok is not None and not site_ok(x, y):
            return None
        if not checked:
            with shield.looking_ahead(t_a):
                if not quick(x, y, t_a):
                    return None
        p = State(x=x, y=y, up=state.up, yaw_deg=state.yaw_deg)
        if transit_breaks(shield, state, p, t_a, speed, climb, escaping, budget,
                          eval_cost):
            return None
        with shield.looking_ahead(t_a):
            for q in _neighbours(State(x=x, y=y, up=state.up)):
                if not quick(q.x, q.y, t_a):
                    return None
        budget[0] -= 2 * (math.ceil(descent_s / look) + 1) * stop_cost
        if landing_breaks(shield, x, y, state.up, t_a, descent_mps, edges):
            return None
        return HoldPoint(
            "LAND", "moved", x, y, state.up, legal=True, travel_s=t_a,
            speed_mps=speed, climb_mps=climb, checked_s=checked_s,
            reason=(f"{why_here}; ({x:.1f}, {y:.1f}) is the nearest site whose "
                    f"descent at {descent_mps:g} m/s from arrival ({t_a:.2f} s) "
                    f"breaks none of them ({HOLD_MARGIN_M:g} m about it too), "
                    f"reached at the current altitude in a straight line that, at "
                    f"every instant on the way, enters no hard rule the aircraft is "
                    f"outside of and is no deeper (to {LEAVE_TOL_M:g} m) in one it "
                    f"is in than a stop here would be then ({why_speed}){floor_note}"))

    def quick_ok(p: State, t_a: float) -> bool:
        if site_ok is not None and not site_ok(p.x, p.y):
            return False
        return quick(p.x, p.y, t_a)

    plan = _search(shield, state, speed, climb, state.up, heading, quick_ok, accept,
                   budget, vertical=False)
    if plan is not None:
        return plan
    capped = budget[0] <= 0
    limit = (f"the search's budget of {MAX_QUERIES:g} stop queries" if capped
             else f"{RADII_M[-1]:g} m")
    return HoldPoint("LAND", "in_place", state.x, state.y, state.up, legal=False,
                     breaks=tuple(bad_here), checked_s=checked_s,
                     reason=(f"{why_here}; no site within {limit} is legal by a "
                             f"legal transit ({why_speed}): landed here{floor_note}"))
