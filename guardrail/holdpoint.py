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
    hold LOITER on a spot about to become illegal"). (Since 2026-10-21 those
    cells fly no escape and never reach Loiter: the fastest way out of a
    window opening on the aircraft is judged as the recovery it is,
    Shield._opening_imposed. The hold points still decide the other cells
    below.)
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
  3. Nothing legal within the budget (MAX_QUERIES units of about 0.1 ms,
     each Shield call charged by what it costs in the world it is made in,
     _Ledger) or 60 m: an in-place plan marked ILLEGAL that names what it
     breaks and, when the budget stopped it, says so (the FSM then searches
     again once a second, fsm.HOLD_REPLAN_S). A count, not a clock, so the
     answer does not depend on the machine's load; sized so the fruitless
     search stays inside the 100 ms tick. A candidate inside a rule that
     stands still is dropped by a cheap read before its stop query, so a
     static world is searched to the last ring; moving zones make each
     candidate dearer and the reach shorter (constants, "THE SEARCH'S
     BUDGET", "THE CHEAP TEST" and "What the budget reaches").
  4. Since 2026-11-07 the budget is spent TICK_QUERIES units a tick, the
     search on a thread of its own in the world of the tick it started on
     (constants, "THE PER-TICK BOUND"; HoldSearch, HoldPlanner): the same
     plan, a few ticks later for the dearest searches, so no tick pays for
     a whole search (the hil Orin paid 146.9 ms for one). Since 2026-11-10
     a tick pays for a slice or for Loiter's hand-back guard, never both
     (HoldPlanner.handback): the Orin paid 93-121 ms for the two together.

SPEED ASSUMPTION

  The transit is planned at the tightest enforced speed cap in force at any
  time a transit the search plans can still be flying (`Shield.kinematic_caps`
  read one lookahead at a time, _speeds; 4 m/s in every library policy) and
  its climb cap, so a cap whose window opens on the way is kept from the
  start, else DEFAULT_SPEED_MPS / DEFAULT_CLIMB_MPS (4 and 2 m/s: the Shield's own
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
  * Fly a rail. The flight rails run their own FSM and plan with plan_for
    (through HoldPlanner); guardrail/hold_flight.py turns the plan into what
    a rail sends (since 2026-11-07; until then every rail held in place).
"""
from __future__ import annotations

import math
import threading
from dataclasses import replace
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
# reproducible on any machine - in UNITS of about 0.1 ms of the desktop's
# time (UNIT_MS), each Shield call charged by what it costs in the world it
# is made in (_Ledger). Measured on the desktop, 2026-10-21, best of 5, at the
# grant's 5 s / 0.1 s forecast:
#
#   stop query (state_is_unsafe of one point: its 51-pose
#     forecast, whatever the rules)                         0.074-0.078 ms
#     + per static zone whose margin ring's box holds it   +0.027-0.033 ms
#       (in its band, inside or outside the ring; +0.010-0.012 ms out of it)
#     + per moving zone the search can meet                +0.14-0.20 ms
#     + per altitude envelope                              +0.04 ms
#     + per corridor, per centreline segment               +0.046 ms
#     + a stand-off ring, a clearance map                  +0.005-0.009 ms
#   read of one position (rule_clearances)                 0.002-0.004 ms
#     + per zone it reads                                  +0.016-0.018 ms
#     + per other rule (a corridor the dearest)            +0.0003-0.0035 ms
#     + per moving zone (built at the view's time; a stop
#       query in the same view reuses it)                  +0.04-0.05 ms
#
# charged (_Ledger.stop_cost) STOP_QUERY_COST, + STATIC_ZONE_STOP_COST per
# static zone whose box holds the point (any band, binding or not: the stop
# query tests them all), + MOVING_ZONE_STOP_COST per moving zone the search
# can meet, + the policy's other rules (_STOP_RULE_COST); a read
# (_Ledger.read_cost) READ_COST + ZONE_READ_COST per zone + RULE_READ_COST
# per other rule + MOVING_ZONE_READ_COST per moving zone. A transit
# evaluation is the reads it makes (the pose; the start again for a moving
# rule being left); the start's own reads are made once a search.
# tests/test_hold_transit.py test_the_budget_charges_what_the_search_makes
# holds the charges to the calls, and test_each_charge_bounds_what_its_call_
# costs (wall clock, GUARDRAIL_TIMING_TESTS=1) re-measures them on the
# machine it runs on.
#
# THE CHEAP TEST. Before a candidate's stop query, the rules that stand
# still (no motion: a static zone whose box holds the point, an altitude
# envelope, a corridor, clearance, a stand-off from a still subject) are
# read at the point (_Ledger.breached), in the arrival view: one of them
# breached there, hard and enforced, is a stop the Shield calls illegal at
# t = 0 of its own forecast (each of those checks flags a pose inside it
# that a stop does not leave - a static zone's "not escaping", a corridor's
# "not returning", an altitude's "not climbing", a stand-off's "not
# opening", clearance's "not receding"), so the candidate is dropped without
# one. A moving rule is never read here: a stop inside a zone or ring that
# moves away from it can be legal (the zone's own motion is the escape).
# tests/test_hold_transit.py test_the_cheap_test_drops_only_what_the_stop_
# query_refuses checks it against the stop query in random worlds.
#
# One rule first. The rule that dropped the last candidate is read alone
# first (READ_COST + ZONE_READ_COST for a zone), and the others only if it
# is not breached at the point. Until 2026-10-21 (integration review of
# T1c) every still rule whose box held the candidate was read and charged,
# so a candidate inside k nested zones cost READ_COST + k * ZONE_READ_COST:
# with 8 or 10 squares about the aircraft (half-widths 50 down to 46.5 m)
# the budget was spent on the candidates nearer than the 55 m ring, and the
# hold stayed in place, illegal, where 51.5 m out was legal (54 ms, desktop,
# best of 3); now it moves there in 14 ms (landing 18-19 ms). The answer is
# the same either way (breached or not); only the price falls.
# tests/test_hold_transit.py test_nested_static_zones_cost_one_zone_read_a_
# candidate.
#
# What the budget reaches (2026-10-21, desktop, best of 3; the final
# review's reach world, tests/test_hold_transit.py test_a_static_world_is_
# searched_to_its_edge): a square always-on zone centred on the aircraft is
# left to its nearest legal point at a half-width of 30, 50 and 57.5 m (the
# last ring is 60 m), each search inside the budget. Until then every
# candidate cost a whole stop query whatever it held, 300 of them were the
# budget, and 16 headings over the rings out to 28 m spent it: nothing past
# about 28 m was ever tried (final review of T1c, P1), so a hold that
# needed 30 m was held inside the zone. With moving zones the reach is what
# MAX_QUERIES buys at their price (the 40 m bar closing at 3 m/s is still
# fruitless inside the budget).
#
# What it costs: MAX_QUERIES units is about 60 ms of the desktop's time, and
# the fruitless searches measured stay at or under that whatever the world -
# the review's worlds (bars closing at 3-6 m/s, a zone opening over the
# aircraft and reaching past 60 m, 5 and 20 overlapping zones, 20 stacked
# out-of-band zones over the aircraft) and 20 diamond-shaped zones whose
# boxes hold every candidate (docs/DESIGN-hold-points.md, "The budget", has
# the table). Since One rule first, the overlapping zones and the diamonds'
# core drop every candidate for one zone read and end at the last ring in
# 12-15 ms, where they took 40-54 ms (the 20 zones and the diamonds spending
# the budget); the bars (38-39 ms) and the stacked zones (29 ms) spend
# it, on stop queries. Until 2026-10-21 static zones were charged nothing:
# with 20 overlapping zones the fruitless search took 226 ms, and with 20
# diamonds 907 ms (each stop query tested a diamond at all 51 forecast poses
# of the stop, about 0.45 ms a zone, which shield.py _check_fence now tests
# once).
#
# History (review of T1c): until 2026-10-20 only stop queries were counted;
# then a transit pose counted as a whole stop query, out of 800 - and the
# fruitless search against a bar closing at 3 m/s took 127-135 ms, over the
# tick (re-review P3), and 0.5 s in a fuzz world with three bars near; then
# (until 2026-10-21) 300 units of a stop query each, moving zones charged,
# static ones not.
UNIT_MS = 0.1
MAX_QUERIES = 600
STOP_QUERY_COST = 0.8
STATIC_ZONE_STOP_COST = 0.35
MOVING_ZONE_STOP_COST = 2.0
# Per rule of the policy, by type (a corridor's per centreline segment).
_STOP_RULE_COST = {"altitude_envelope": 0.4, "corridor": 0.5, "corridor_swap": 0.5,
                   "subject_standoff": 0.1, "obstacle_clearance": 0.1}
READ_COST = 0.05
ZONE_READ_COST = 0.2
RULE_READ_COST = 0.05
MOVING_ZONE_READ_COST = 0.5
#
# THE PER-TICK BOUND (since 2026-11-07). A search may spend MAX_QUERIES, about
# 60 ms of the desktop's time, and until then spent them on ONE tick: on the
# hil shakedown (hs02, d186077) the AGX Orin, 3-4x slower on this code than the
# desktop, planned dyn-nfz-rotate-spin's moved hold on a 146.9 ms tick (e2e
# 167.5 ms; its Loiter ticks p50 44 ms), over WP3-13's 100 ms tick budget. A
# search the FSM asks for (HoldPlanner, for the Shield's own escalation and
# for the flight rails) now runs on a thread of its own (HoldSearch) and is
# handed one tick's slice at a time: TICK_QUERIES units of its _Ledger -
# counted, never timed, so a headless sweep stays reproducible, and the two
# threads never run at once, so everything happens in the single-threaded
# order. It plans in the world of the tick it started on (that tick's view:
# its time, rules, clock and moving zones), so the plan it returns is exactly
# the one the whole search would have made on that tick (a stand-off rule's
# live subject excepted: it is read where the rail last set it), and it keeps
# its whole reach: a search that spends all of MAX_QUERIES takes
# ceil(MAX_QUERIES / TICK_QUERIES) ticks. Until it returns nothing new is
# held: on entry the FSM has no plan yet and the aircraft holds where it is;
# on a re-plan the point held is no longer legal, and an in-place plan is
# taken on the search's first tick, so the aircraft holds where it is rather
# than fly on toward that point. HoldPlanner(units=None) plans within the
# tick, as before.
#
# 60 since 2026-11-10 (100 from 2026-11-07), and ONE kind of hold work a
# tick: a search slice, or the hand-back guard, never both (HoldPlanner.
# handback). On the hil shakedown hs05 (9eed35b, dyn-nfz-rotate-spin) every
# tick over WP3-13's 100 ms - six, in two flights, 100.7-122.8 ms - was a
# Loiter tick that ran a full 100-unit slice AND the guard. Read from the
# nine flights' rows (tick_ms - shield_ms; a Normal tick's is 7.9 ms at the
# median): a Loiter tick without a search - the guard's ten stop queries and
# hold_point_legal's one - 26.9 ms at the median and 47.6 ms at most; a tick
# with a full slice beside them 55.3 ms and 95.9 ms. So a 100-unit slice
# cost the Orin about 28 ms at the median and up to 49 ms (0.28-0.49 ms a
# unit; the desktop: 0.077 at the median, so 3.6x), and the guard about
# 19 ms and up to 40 ms (the desktop: 4.3 ms at the median, 4.4x). A slice
# tick now costs 60 units and one stop query - about 20 ms, 33 ms at most on
# those figures - and a guard tick what it did: about 40 ms of hold work at
# most either way, where it was 88 ms. A search that spends all of
# MAX_QUERIES takes ceil(MAX_QUERIES / TICK_QUERIES) = 10 ticks (6 before),
# one more when it starts on a tick whose units went to the guard
# (docs/DESIGN-hold-points.md, "The Orin's tick").
TICK_QUERIES = 60
# Loiter's hand-back guard (Shield.handback_unsafe) on any one tick: as many
# samples of the command's flight as fit in this many units at the world's
# dearest stop query (stop_cost_bound), never more than one every
# shield.HANDBACK_STEP_S. Until then always one every HANDBACK_STEP_S through
# the lookahead, whatever a stop query cost (up to 17.5 ms measured on the
# desktop; p50 1.7 ms). Every library world still fits all of them (a stop
# query there is priced at 4 units at most): the bound caps dearer worlds
# only. The price is the ledger's, and in a corridor world a stop query costs
# about 2.3x it (docs/DESIGN-hold-points.md, "The per-tick bound").
HANDBACK_QUERIES = 40
# A search not resumed for this long was abandoned by the loop that ran it (an
# episode that ended mid-search, a node shut down): its thread ends. A live
# loop resumes it every tick, so no result depends on this.
ABANDON_S = 60.0
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


def _speeds(shield, climb_m: float = 0.0) -> tuple[float, float, str]:
    """(speed, climb rate, the reason's words) a transit is planned at: the
    tightest enforced caps in force at any time a transit the search plans
    can still be flying - the longest is RADII_M[-1] at the speed, or
    `climb_m` at the climb rate - else DEFAULT_SPEED_MPS / DEFAULT_CLIMB_MPS.

    Shield.kinematic_caps answers for one lookahead; a transit can be three
    times as long (60 m at 4 m/s, 15 s). Until 2026-10-21 the caps of the
    first lookahead were the plan's, so a tighter hard cap whose window
    opened after it but before arrival was broken by the transit while the
    reason said it entered no rule (final review of T1c, P3: a 1 m/s P0 cap
    opening 5.6 s ahead, the hold 23.5 m off at 4 m/s, 5 of 100 transit
    instants over the cap). Now each lookahead up to the longest transit is
    read in its own look-ahead view (a window opens and closes only on whole
    minutes, shield.py _Schedule, so the views miss none), and a cap that
    tightens the speed lengthens the horizon, read again until nothing
    tighter comes into force.

    The reason words the speed and the climb rate each by where it comes
    from: a cap in force in the first lookahead, one that comes into force
    later, or the default assumed. A later cap no looser than an assumed
    default becomes that axis' source, since it then bounds the transit.
    Until 2026-10-21 (integration review of T1c) one found/later flag worded
    both, so with no cap now and a 5 m/s / 1 m/s cap opening on the way the
    reason called the DEFAULT 4 m/s "the tightest enforced 4 m/s speed cap",
    and with a 4 m/s / 2 m/s cap opening on the way it said "no enforced
    speed cap in force" (tests/test_hold_transit.py test_a_speed_cap_that_
    opens_on_the_way_slows_the_whole_transit asserts each reason whole)."""
    look = max(float(shield.lookahead_s), 1e-3)
    caps = shield.kinematic_caps()
    if caps is not None:
        val = {"speed": caps.speed_max_mps, "climb": caps.climb_rate_max_mps}
        src = {"speed": "now", "climb": "now"}
    else:
        val = {"speed": DEFAULT_SPEED_MPS, "climb": DEFAULT_CLIMB_MPS}
        src = {"speed": "default", "climb": "default"}
    seen = caps is not None             # any enforced cap read, now or later

    def horizon() -> float:
        return max(RADII_M[-1] / val["speed"], abs(climb_m) / val["climb"])

    j = 1
    while j * look < horizon():
        with shield.looking_ahead(j * look):
            cj = shield.kinematic_caps()
        if cj is not None:
            seen = True
            for axis, cap in (("speed", cj.speed_max_mps),
                              ("climb", cj.climb_rate_max_mps)):
                if cap < val[axis] - 1e-12 or (src[axis] == "default"
                                               and cap <= val[axis] + 1e-12):
                    val[axis], src[axis] = min(val[axis], cap), "later"
        j += 1
    by = f"before the longest transit the search plans ends ({horizon():.1f} s)"

    def words(axis: str) -> str:
        v = val[axis]
        if src[axis] == "now":
            return f"{axis} {v:g} m/s: the tightest enforced {axis} cap in force"
        if src[axis] == "later":
            return (f"{axis} {v:g} m/s: the tightest enforced {axis} cap, which comes "
                    f"into force {by}")
        if not seen:
            return f"{axis} {v:g} m/s: assumed, no enforced {axis} cap in force {by}"
        return (f"{axis} {v:g} m/s: assumed, no enforced {axis} cap in force now and "
                f"none at or under {v:g} m/s {by}")
    return val["speed"], val["climb"], f"{words('speed')}; {words('climb')}"


_ZONE_TYPES = ("polygon_fence", "circle_fence", "dynamic_nfz")
# The rule types the cheap test reads, by what a stop there is judged on
# (THE CHEAP TEST); a landing reads only those a SITE can satisfy.
_STILL_TYPES = _ZONE_TYPES + ("altitude_envelope", "subject_standoff",
                              "obstacle_clearance") + _TUBE_TYPES
_SITE_TYPES = _ZONE_TYPES + ("subject_standoff", "obstacle_clearance")


class _Ledger:
    """One search's budget (THE SEARCH'S BUDGET): what is left (`left`, in
    units; `spent`), and what each Shield call costs in this world, read
    once from the start. `horizon_s` is how far ahead the search's queries
    look; `site_only` restricts the cheap test to the rules a landing site
    can satisfy (LAND_SITE_CATEGORIES)."""

    def __init__(self, shield, state: State, horizon_s: float, site_only: bool = False):
        self._left = float(MAX_QUERIES)
        # The search's pacing over monitor ticks (THE PER-TICK BOUND): the
        # HoldSearch running it, or None for a search made within one call.
        self._pacer = getattr(_PACE, "search", None)
        self.motion = shield.rule_motion()
        self.rows = rows = shield.rule_status(state)
        horizon = float(horizon_s)
        # Moving zones the search can meet: it turns (a turning zone reaches
        # as far as it is long, which the Shield's motion model does not
        # report), or its margin ring is no farther than the search's reach
        # plus how far it moves within the horizon. Soft and monitor-only
        # zones count: the stop query checks every zone.
        zone_ids = {r["id"] for r in rows if r["type"] in _ZONE_TYPES}
        moving = [rid for rid in self.motion if rid in zone_ids]
        room = shield.rule_clearances(state, moving) if moving else {}
        self.moving_zones = frozenset(moving)
        self.zones = frozenset(zone_ids)
        # What every stop query costs whatever the point (its forecast, and
        # the rules other than zones; a corridor by its segments).
        self.stop_base = STOP_QUERY_COST
        for r in rows:
            k = _STOP_RULE_COST.get(r["type"], 0.0)
            if r["type"] in _TUBE_TYPES:
                k *= max(1, len(r["rule"].points()) - 1)
            self.stop_base += k
        # The reads just made, charged like any other (the search's transits
        # reuse the start's rows: transit_breaks, `start_reads`).
        self.left -= self.read_cost(r["id"] for r in rows)
        if moving:
            self.left -= self.read_cost(moving)
        self.m = 0
        for rid in moving:
            ux, uy, w, _, _ = self.motion[rid]
            c = room.get(rid)
            if c is not None and (w or c.h <= RADII_M[-1] + math.hypot(ux, uy) * horizon):
                self.m += 1
        recs = {r.rule.id: r for r in shield.ir.fences}
        # Every static zone's ring box (the stop query's index tests them all).
        self.boxes = [recs[rid].bounds for rid in zone_ids
                      if rid in recs and rid not in self.motion]
        kinds = _SITE_TYPES if site_only else _STILL_TYPES
        still = [r for r in rows if _binding_row(r) and r["type"] in kinds
                 and r["id"] not in self.motion]
        self.pre_zones = [(r["id"], recs[r["id"]].bounds) for r in still
                          if r["type"] in _ZONE_TYPES and r["id"] in recs]
        self.pre_rules = [r["id"] for r in still if r["type"] not in _ZONE_TYPES]
        # The rule that dropped the last candidate (breached): read first,
        # alone, at the next one (THE CHEAP TEST, "One rule first").
        self.last_drop: str | None = None
        self.start_reads = None

    @property
    def left(self) -> float:
        return self._left

    @left.setter
    def left(self, v: float) -> None:
        # Every charge of the search goes through here (`ledger.left -= c`),
        # so a search paced by a HoldSearch is paused before it spends more
        # than its tick's slice (HoldSearch.charge).
        if self._pacer is not None and v < self._left:
            self._pacer.charge(self._left - v)
        self._left = v

    @property
    def spent(self) -> float:
        return MAX_QUERIES - self.left

    def read_cost(self, ids) -> float:
        """What one rule_clearances read of these rules is charged."""
        ids = list(ids)
        z = sum(1 for r in ids if r in self.zones)
        return (READ_COST + ZONE_READ_COST * z + RULE_READ_COST * (len(ids) - z)
                + MOVING_ZONE_READ_COST * sum(1 for r in ids if r in self.moving_zones))

    def stop_cost(self, p: State) -> float:
        """What a stop query at p is charged."""
        n = sum(1 for x0, y0, x1, y1 in self.boxes
                if x0 <= p.x <= x1 and y0 <= p.y <= y1)
        return self.stop_base + STATIC_ZONE_STOP_COST * n + MOVING_ZONE_STOP_COST * self.m

    def breached(self, shield, p: State) -> bool:
        """THE CHEAP TEST, in the caller's view: is a still, hard, enforced
        rule breached at p? The rule that dropped the last candidate is read
        first, alone, when it applies at p (its box holds p, or it is not a
        zone), and the others only if it is not breached there; each read is
        charged as what it reads. Of several rules breached, the one p is
        deepest in (_depth; ties to the first read) is kept for the next
        candidate: it is the likeliest to hold that one too."""
        ids = self.pre_rules + [rid for rid, (x0, y0, x1, y1) in self.pre_zones
                                if x0 <= p.x <= x1 and y0 <= p.y <= y1]
        if not ids:
            return False
        last = self.last_drop
        if last is not None and last in ids:
            self.left -= self.read_cost([last])
            c = shield.rule_clearances(p, [last]).get(last)
            if c is not None and c.breach:
                return True
            ids = [rid for rid in ids if rid != last]
            if not ids:
                return False
        self.left -= self.read_cost(ids)
        got = shield.rule_clearances(p, ids)
        hit = [rid for rid in ids if rid in got and got[rid].breach]
        if not hit:
            return False
        self.last_drop = max(hit, key=lambda rid: _depth(got[rid]))
        return True

    def stop_ok(self, shield, p: State, categories=None) -> bool:
        """The cheap test, then the stop query: no hard enforced violation
        of a stop at p from the view's time through the lookahead (of the
        given categories only, when given)."""
        if self.breached(shield, p):
            return False
        self.left -= self.stop_cost(p)
        vios = shield.hard_violations(shield.state_is_unsafe(p))
        if categories is not None:
            vios = [v for v in vios if v.category in categories]
        return not vios


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
            heading, quick_ok, accept, ledger: "_Ledger", vertical: bool = True):
    """The ring search holds and landings share (module docstring, "THE
    SEARCH"). Candidates in travel-time order, one look-ahead view per
    arrival time; `quick_ok(p, t_a)` is the cheap test made inside that view,
    `accept(r, k, t_a, x, y, checked)` the full one (a plan, or None);
    `checked` is True for a candidate quick_ok already passed, so accept
    does not repeat that test (until 2026-10-20 every hit's own stop query
    was made twice, once here and once in accept, at the same time). On the
    first accepted candidate of a ring, its heading is searched again between
    the previous ring and this one every REFINE_STEP_M, and the nearest
    accepted point wins. `ledger` (_Ledger: the units left) bounds the whole
    search."""
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
                if ledger.left <= 0:
                    return None
                if quick_ok(State(x=c[3], y=c[4], up=c[5], yaw_deg=state.yaw_deg), t_a):
                    hit = c
                    break
        if hit is not None:
            tt, _, k, x, y, _ = hit
            if ledger.left <= 0:
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
                while r0 + j * REFINE_STEP_M < r - 1e-9 and ledger.left > 0:
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
# How a plan's reason names the budget when it stopped the search.
BUDGET_NOTE = (f"the search's budget of {MAX_QUERIES:g} units (about "
               f"{MAX_QUERIES * UNIT_MS:g} ms of the desktop's time)")


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
                   ledger: "_Ledger | None" = None) -> list[str]:
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

    `ledger` (the search's _Ledger, "THE SEARCH'S BUDGET"): each read is
    charged as what it reads (_Ledger.read_cost); the start's own reads -
    every rule's status and the binding rules' rooms, the same for every
    transit of a search from it - are made and charged once a search and
    kept on the ledger. A transit the budget runs out on is not a legal one
    ([BUDGET_SPENT])."""
    T = float(travel_s)
    dx, dy, dz = end.x - start.x, end.y - start.y, end.up - start.up
    d = math.hypot(dx, dy)
    if T <= 1e-12 or (d <= 1e-9 and abs(dz) <= 1e-9):
        return []
    esc = set(escaping)

    def charge(ids) -> None:
        if ledger is not None:
            if ledger.left <= 0:
                raise _Spent
            ledger.left -= ledger.read_cost(ids)

    try:
        if ledger is not None and ledger.start_reads is not None:
            types, c0 = ledger.start_reads
        else:
            rows = ledger.rows if ledger is not None else shield.rule_status(start)
            types = {r["id"]: r["type"] for r in rows if _binding_row(r)}
            charge(types)
            c0 = shield.rule_clearances(start, types)
            if ledger is not None:
                ledger.start_reads = (types, c0)
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
        p = _pose_at(start, end, tau, speed, climb)
        ids = sorted(live)
        moving_left = [rid for rid in ids if rid in leave and rid not in leave_still]
        charge(ids)
        if moving_left:
            charge(moving_left)
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


def time_varying(shield) -> bool:
    """Can a rule's geometry or force change with time in this Shield's
    world: a moving zone or a moving subject (Shield.rule_motion), or a rule
    with a window, a switch or a swap (its rule clock's `timed`)? Where
    nothing can, a stop legal through one lookahead stays legal for as long
    as it is held, and the hold horizon (choose_hold_point, `hold_s`) asks
    nothing more of a point."""
    clock = getattr(shield, "rule_clock", None)
    timed = True if clock is None else bool(getattr(clock, "timed", True))
    return timed or bool(shield.rule_motion())


def choose_hold_point(shield, state: State, *, unsafe: list | None = None,
                      heading: tuple[float, float] | None = None,
                      search: bool = True, hold_s: float | None = None) -> HoldPoint:
    """Where the FSM's Loiter holds (module docstring). `unsafe` is the hard
    enforced violations of a stop at `state` (the FSM's stop_illegal), read
    here when not given; `heading` the way the aircraft is going (ties);
    `search` False reports the in-place hold only (Shield(hold_points=False)).

    `hold_s` (since 2026-11-07; fsm.EscalationFSM.hold_horizon_s, FSMConfig
    hold_horizon "loiter"): how long from now the Loiter can last - the
    hold must stay legal from arrival until then and one lookahead past it,
    at its HOLD_MARGIN_M neighbours too, judged in lookahead-long views
    (`held`). Until then a point was judged through one lookahead from
    arrival only, so under a moving zone the nearest such point was legal
    for barely that long: dyn-nfz-rotate-spin on the lagging vehicles re-
    planned every 0.5-1.4 s ahead of the spinning bar, the aircraft never
    stood anywhere a stop was legal for T, and X6 sent it home (a false
    fail-safe by the rubric's N4). None, or a world where nothing can change
    with time (time_varying), plans exactly as before. A hold in place that
    is legal through the lookahead but would not stay legal for the Loiter is
    moved to a point that does; when the search finds no such point (none
    within reach, or the budget spent), the plan is the one the lookahead
    alone gives - the plan before this change - so no hold is worse than it
    was. The lookahead's own search runs first and keeps its whole budget;
    the longer one gets what it left (candidates it refused are not paid
    for twice), so a search still costs at most MAX_QUERIES."""
    look = float(shield.lookahead_s)
    if unsafe is None:
        unsafe = shield.hard_violations(shield.state_is_unsafe(state))
    here = f"({state.x:.1f}, {state.y:.1f}, {state.up:.1f})"
    span = (float(hold_s) if (search and hold_s is not None and float(hold_s) > 0.0
                              and time_varying(shield)) else None)
    stay = None
    if not unsafe:
        stay = HoldPoint("LOITER", "in_place", state.x, state.y, state.up, legal=True,
                         checked_s=look,
                         reason=(f"standing still at {here} breaks no hard rule "
                                 f"within the {look:g} s lookahead"))
        if span is None:
            return stay
    breaks = _ids(unsafe)
    what = (f"standing still at {here} breaks {', '.join(breaks)} within the "
            f"{look:g} s lookahead" if unsafe else
            f"standing still at {here} is legal through the {look:g} s lookahead, "
            f"not for the {span:.1f} s the Loiter can last")
    if not search:
        return HoldPoint("LOITER", "in_place", state.x, state.y, state.up, legal=False,
                         breaks=breaks, checked_s=look,
                         reason=(f"{what}; the hold-point search is off "
                                 f"(Shield(hold_points=False)): held here, as before "
                                 f"2026-10-19"))
    up_t = hold_altitude(shield, state)
    speed, climb, why_speed = _speeds(shield, up_t - state.up)
    tz = abs(up_t - state.up) / climb
    ledger = _Ledger(shield, state, max(RADII_M[-1] / speed, tz) + look + (span or 0.0))

    def stop_ok(p: State) -> bool:
        # Inside a looking_ahead view: a stop at p, from the view's time,
        # breaks no hard enforced rule through the lookahead (the cheap
        # test first, THE CHEAP TEST).
        return ledger.stop_ok(shield, p)

    def views(t_a: float) -> int:
        """How many lookahead-long views after the first a hold reached at
        t_a needs, so that the stops judged from arrival cover the Loiter's
        span and one lookahead past it."""
        return int(math.ceil(max(0.0, span - t_a) / look - 1e-9)) if span else 0

    def held(p: State, t_a: float, neighbours: bool = True,
             point: bool = True) -> bool:
        """THE HOLD HORIZON: a stop at p (then at its HOLD_MARGIN_M
        neighbours) is legal in every lookahead-long view from t_a + look to
        the end of the Loiter's span - the first view, from arrival, the
        caller has judged. The point's views first: most points fail there,
        for a fifth of the neighbours' cost."""
        n = views(t_a)
        for pts in (([p] if point else []), (_neighbours(p) if neighbours else [])):
            if not pts:
                continue
            for j in range(1, n + 1):
                if ledger.left <= 0:
                    return False
                with shield.looking_ahead(t_a + j * look):
                    if not all(stop_ok(q) for q in pts):
                        return False
        return True

    def held_note(t_a: float) -> str:
        return (f"; it stays legal for as long as the Loiter can last ({span:.1f} s "
                f"from now) and one lookahead past it: checked "
                f"{(views(t_a) + 1) * look:g} s from arrival, the {HOLD_MARGIN_M:g} m "
                f"about it too")

    if stay is not None and held(state, 0.0):
        return replace(stay, checked_s=(views(0.0) + 1) * look,
                       reason=stay.reason + held_note(0.0))

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
        if transit_breaks(shield, state, p, t_a, speed, climb, breaks, ledger):
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

    # Candidates a search refused, by (x, y, up, arrival): the same query in
    # the same view gives the same answer, so the hold horizon's search does
    # not pay for them again.
    refused: set = set()

    def key(x: float, y: float, up: float, t_a: float) -> tuple:
        return (round(x, 6), round(y, 6), round(up, 6), round(t_a, 9))

    def quick(p: State, t_a: float) -> bool:
        if key(p.x, p.y, p.up, t_a) in refused:
            return False
        ok = stop_ok(p)
        if not ok:
            refused.add(key(p.x, p.y, p.up, t_a))
        return ok

    def accept_once(r: float, k: int, t_a: float, x: float, y: float,
                    checked: bool = False) -> HoldPoint | None:
        if key(x, y, up_t, t_a) in refused:
            return None
        plan = accept(r, k, t_a, x, y, checked)
        if plan is None:
            refused.add(key(x, y, up_t, t_a))
        return plan

    plan = stay
    if plan is None:
        # The lookahead's search, exactly as before this change.
        plan = _search(shield, state, speed, climb, up_t, heading, quick,
                       accept_once, ledger)
        if plan is not None and span is not None:
            if held(State(x=plan.x, y=plan.y, up=plan.up, yaw_deg=state.yaw_deg),
                    plan.travel_s):
                return replace(plan, checked_s=(views(plan.travel_s) + 1) * look,
                               reason=plan.reason + held_note(plan.travel_s))
            refused.add(key(plan.x, plan.y, plan.up, plan.travel_s))
    if span is not None and (plan is not None or stay is not None):
        # THE HOLD HORIZON's search, on what the first one left.
        def accept_held(r: float, k: int, t_a: float, x: float, y: float,
                        checked: bool = False) -> HoldPoint | None:
            # The point's own later views before its transit and neighbours:
            # under a sweep most candidates near the aircraft are legal for a
            # lookahead and fail there, for a third of the cost.
            if key(x, y, up_t, t_a) in refused:
                return None
            p = State(x=x, y=y, up=up_t, yaw_deg=state.yaw_deg)
            if checked and not held(p, t_a, neighbours=False):
                refused.add(key(x, y, up_t, t_a))
                return None
            got = accept_once(r, k, t_a, x, y, checked)
            if got is None:
                return None
            if not held(p, t_a, point=not checked):
                refused.add(key(got.x, got.y, got.up, t_a))
                return None
            return replace(got, checked_s=(views(t_a) + 1) * look,
                           reason=got.reason + held_note(t_a))

        better = _search(shield, state, speed, climb, up_t, heading, quick,
                         accept_held, ledger)
        if better is not None:
            return better
        limit = BUDGET_NOTE if ledger.left <= 0 else f"{RADII_M[-1]:g} m"
        return replace(plan, reason=(
            f"{plan.reason}; no point within {limit} stays legal for the "
            f"{span:.1f} s the Loiter can last: the lookahead's plan"))
    if plan is not None:
        return plan
    limit = BUDGET_NOTE if ledger.left <= 0 else f"{RADII_M[-1]:g} m"
    return HoldPoint("LOITER", "in_place", state.x, state.y, state.up, legal=False,
                     breaks=breaks, checked_s=look,
                     reason=(f"{what}; no point within {limit} is legal from arrival "
                             f"through the lookahead by a legal transit "
                             f"({why_speed}): held here, and searched again in "
                             f"a second"))


def hold_point_legal(shield, hold: HoldPoint | None, t: float) -> bool | None:
    """Is a Loiter's hold POINT still legal at monitor time `t`: a stop there,
    from the plan's arrival (now, once it is due), breaks no hard enforced
    rule through the lookahead (fsm.TickInput.hold_legal)? None without a
    plan. One stop query. The Shield's own check (Shield._hold_legal) and the
    flight rails' (guardrail/hold_flight.py) are this function."""
    if hold is None:
        return None
    t0 = hold.t_s if hold.t_s is not None else t
    ahead = max(0.0, t0 + hold.travel_s - t)
    p = State(x=hold.x, y=hold.y, up=hold.up)
    return not shield.hard_violations(shield.state_is_unsafe(p, ahead_s=ahead))


def plan_for(shield, fsm, state: State, *, unsafe: list | None = None,
             heading: tuple[float, float] | None = None, search: bool = True,
             site_ok=None) -> HoldPoint:
    """The plan a flight rail running its own FSM gives `fsm` when it asks
    (`fsm.needs_hold`): its Loiter's hold point (choose_hold_point, judged
    for the Loiter's remaining time under FSMConfig.hold_horizon "loiter")
    or its Land's site (choose_landing_site), as the Shield's own escalation
    plans them (Shield._plan_hold). `unsafe` is the hard enforced violations
    of a stop at the position (the FSM's stop_illegal); `search` False holds
    in place, as every rail did until 2026-11-07. A search that raises
    becomes an in-place plan marked illegal, never an exception in the
    rail's 10 Hz loop."""
    from .fsm import FSMState                  # local: no cycle at import
    land = fsm.state is FSMState.LAND
    unsafe = shield.hard_violations(unsafe if unsafe is not None
                                    else shield.state_is_unsafe(state)) or []
    state = State(x=state.x, y=state.y, up=state.up, yaw_deg=state.yaw_deg)
    try:
        if land:
            return choose_landing_site(shield, state, unsafe=unsafe, heading=heading,
                                       search=search, site_ok=site_ok)
        return choose_hold_point(shield, state, unsafe=unsafe, heading=heading,
                                 search=search, hold_s=fsm.hold_horizon_s())
    except Exception as e:                          # noqa: BLE001 - see docstring
        return HoldPoint("LAND" if land else "LOITER", "in_place", state.x, state.y,
                         state.up, legal=False,
                         breaks=tuple(sorted({v.rule_id for v in unsafe})),
                         reason=(f"hold-point search failed ({type(e).__name__}: {e}); "
                                 f"holding where the aircraft is, legality not "
                                 f"established"))


# The HoldSearch a search runs under, on the search's own thread (_Ledger
# reads it when it is made).
_PACE = threading.local()


class _Cancelled(BaseException):
    """A HoldSearch dropped before it ended. Not an Exception: no `except
    Exception` in the search's code (plan_for, Shield._plan_hold) may turn it
    into a plan."""


class HoldSearch:
    """One hold-point search spread over monitor ticks (THE PER-TICK BOUND).

    `run()` - the whole search: Shield._plan_hold, plan_for - runs on a thread
    of its own in `world` (a Shield tick: its time, rules, clock and moving
    zones) and only while step() waits for it: each step() lets it charge up
    to `units` of its _Ledger, then returns. A charge that would overrun the
    slice waits for the next step (one charge larger than a slice is made
    alone). The caller's thread and the search never run at once, so every
    query is made in the order the single-threaded search makes it. `result`
    is what run() returned, `error` what it raised; `ticks` and `spent` (units)
    what it took; `cancelled` if it was dropped (cancel(), or ABANDON_S without
    a step)."""

    def __init__(self, shield, run: Callable[[], Any], world, units: float):
        self.shield, self.world, self.units = shield, world, float(units)
        self.result: Any = None
        self.error: Exception | None = None
        self.done = False
        self.cancelled = False
        self.ticks = 0
        self.spent = 0.0
        self._used = 0.0
        self._go = threading.Semaphore(0)
        self._back = threading.Semaphore(0)
        self._thread = threading.Thread(target=self._main, args=(run,), daemon=True,
                                        name="hold-search")
        self._thread.start()

    def _main(self, run: Callable[[], Any]) -> None:
        resumed = self._go.acquire(timeout=ABANDON_S)
        _PACE.search = self
        self.shield._local.tick = self.world
        try:
            if not resumed or self.cancelled:
                raise _Cancelled
            self.result = run()
        except _Cancelled:
            self.cancelled = True
        except Exception as e:                      # noqa: BLE001 - step() reports it
            self.error = e
        finally:
            _PACE.search = None
            self.shield._local.tick = None
            self.done = True
            self._back.release()

    def charge(self, units: float) -> None:
        """On the search's thread (_Ledger.left): `units` are about to be
        spent. Past this tick's slice, wait for the next step()."""
        if self._used > 0.0 and self._used + units > self.units + 1e-9:
            self._back.release()
            if not self._go.acquire(timeout=ABANDON_S):
                self.cancelled = True
            if self.cancelled:
                raise _Cancelled
        self._used += units
        self.spent += units

    def step(self) -> bool:
        """Run the search for one tick's slice. True once it has ended."""
        if self.done:
            return True
        self.ticks += 1
        self._used = 0.0
        self._go.release()
        self._back.acquire()
        return self.done

    def cancel(self) -> None:
        """Drop the search: its thread ends now (at its next charge)."""
        if self.done:
            return
        self.cancelled = True
        self._go.release()
        self._back.acquire()


class HoldPlanner:
    """The FSM's hold plans under THE PER-TICK BOUND, for whoever runs the FSM:
    the Shield's own escalation (Shield._escalate) and the flight rails that
    run their own (sitl/run_sitl_demo.py, sitl/ros2_shield_node.py).

    Call tick() once a tick, after the FSM stepped. When the FSM asks for a
    plan (`needs_hold`) a search starts (HoldSearch, `run`: the whole plan, as
    the caller made it before) in this tick's world, and gets `units` (default
    TICK_QUERIES) of its ledger each tick; tick() returns the plan on the tick
    the search ends, for the caller to give the FSM (`hold_at`). Until then it
    returns None and nothing new is held: on entry the FSM has no plan, which
    every rail and the harness hold in place; on a re-plan (the FSM still
    holds a point that is no longer legal) it returns, on the search's first
    tick, an in-place plan (`here`, legal unless `unsafe`: the hard enforced
    violations of a stop there, the FSM's stop_illegal), so the aircraft holds
    where it is rather than fly on toward that point. A search whose FSM
    state ended (Loiter left, Land entered, a terminal state) is dropped, and
    the new state's plan searched for. `units` None plans within the tick, as
    before 2026-11-07. `last`: the ticks and units the last plan returned
    took (None for an in-place plan taken meanwhile).

    ONE OF THE TWO A TICK (since 2026-11-10; TICK_QUERIES has the rig's
    numbers). Loiter's hand-back guard (Shield.handback_unsafe, up to
    HANDBACK_QUERIES units) and a search slice are never both paid on one
    tick. Ask the guard through handback(), before the FSM steps: while a
    search runs it samples nothing and answers True - the hand-back is
    refused until the plan is in hand, the conservative side (Loiter's
    "acceptable" clock starts again when the search ends) - and when it did
    sample, a search that starts on that same tick (a re-plan: the point held
    is no longer legal) takes its first slice on the next tick; the in-place
    plan a re-plan holds meanwhile is still returned at once. `guard`: what
    the last handback() did ("ran", "search": refused for the search, None:
    not asked since the last tick()); `slice`: the units the last tick()'s
    search slice spent (0 without one).

    The guard itself is two checks under FSMConfig.handback_request
    "at_speed" (2026-11-10): the path (Shield.handback_unsafe: the Shield's
    action flown from here, a stop legal at every sample) and the request
    (Shield.handback_untrusted: its repair judged from the hold point at the
    speed it asks for, one more decision of the Shield's). One of them a
    tick, the request first, and the other's last answer stands; until both
    have answered in a run of guard ticks the hand-back is refused. In a
    world where nothing else moves (no timed rule, no moving zone, no tracked
    subject) the request's answer is asked again only when the request (to
    1 cm/s), the hold point or the rule set changed; where something does
    move the two take turns, tick by tick; the request is never asked on two
    ticks running. A tick's hold work is then a slice, or the path's
    samples, or one decision: never two of them (`checked`: "path",
    "request")."""

    def __init__(self, units: float | None = TICK_QUERIES):
        self.units = units
        self.search: HoldSearch | None = None
        self.last: dict | None = None
        self.guard: str | None = None
        self.checked: str | None = None     # which check the last handback() made
        self.slice = 0.0            # units the last tick()'s search slice spent
        self._state = None
        self._replan = False
        self._held = False          # the re-plan's in-place plan was given
        self._waited = 0            # ticks a new search waited for the guard
        self._path: bool | None = None      # the path check's last answer
        self._request: bool | None = None   # the request check's last answer
        self._req_key = self._req_rt = None     # what that answer was asked about
        self._request_last = False          # the last guard tick asked it

    def handback(self, shield, state: State, action, *, fsm=None,
                 raw=None) -> tuple[bool, bool | None]:
        """Loiter's hand-back guard under the per-tick bound (class
        docstring): (handback_unsafe, handback_untrusted) for the FSM's
        TickInput. (True, None) while a search runs, nothing sampled. With
        `fsm` on handback_request "at_speed" and the request `raw`, the two
        checks in turn; otherwise the path check alone, as until 2026-11-10."""
        self.checked = None
        if self.units is not None and self.search is not None:
            self.guard = "search"
            return True, None
        self.guard = "ran"
        if (fsm is None or raw is None
                or getattr(fsm.config, "handback_request", None) != "at_speed"):
            self.checked = "path"
            return bool(shield.handback_unsafe(state, action)), None
        hold = fsm.hold
        at = (state if hold is None else
              State(x=hold.x, y=hold.y, up=hold.up, yaw_deg=state.yaw_deg))
        # The request check's answer stands while what it was asked about
        # cannot have changed: the same request (to 1 cm/s) from the same
        # point under the same rule set, in a world where nothing else moves
        # (Shield.world_is_still: no timed rule, no moving zone, no tracked
        # subject). One decision of the Shield's costs as much as the tick's
        # own (the desktop: 2 ms at the median, 16-22 ms for a request deep
        # into a zone), and a Loiter that runs to its timeout would make the
        # same one 150 times. Where the world moves the two checks take
        # turns, tick by tick: an answer kept for 0.5 s there handed
        # dyn-nfz-rotate-spin at 9 deg/s back 0.4 s late, into the bar's next
        # sweep (measured 2026-11-10, the ideal vehicle).
        key = (round(raw.vx, 2), round(raw.vy, 2), round(raw.vz_up, 2),
               round(at.x, 2), round(at.y, 2), round(at.up, 2))
        fresh = (self._request is not None and key == self._req_key
                 and shield._rt is self._req_rt and shield.world_is_still())
        if not fresh and not self._request_last:
            self._request = bool(shield.handback_untrusted(raw, at, fsm=fsm))
            self._req_key, self._req_rt = key, shield._rt
            self._request_last = True
            self.checked = "request"
        else:
            # The path's turn: every tick the request's answer stands, and
            # the tick after a request check whatever changed (never two
            # request checks running, or a request that changes every tick
            # would leave the path unsampled).
            self._path = bool(shield.handback_unsafe(state, action))
            self._request_last = False
            self.checked = "path"
        return (True if self._path is None else self._path), self._request

    def reset(self) -> None:
        """Drop any search (a new episode, the FSM reset)."""
        if self.search is not None:
            self.search.cancel()
        self.search, self._state, self.last = None, None, None

    def tick(self, shield, fsm, run: Callable[[], HoldPoint], *, here: State,
             unsafe: list, world=None) -> HoldPoint | None:
        guard_ran, self.guard = self.guard == "ran", None
        if not guard_ran:
            # A run of guard ticks ended: its answers are not the next one's.
            self._path = self._request = None
            self._req_key = self._req_rt = None
            self._request_last = False
        self.slice = 0.0
        s = self.search
        if s is not None and (fsm.terminal or fsm.state is not self._state):
            self.reset()
            s = None
        if s is None:
            if not fsm.needs_hold:
                return None
            if self.units is None:
                self.last = None
                return run()
            if world is None:
                world = shield._tick_ctx() or shield._new_tick(None)
            self._state = fsm.state
            self._replan = fsm.hold is not None
            self._held, self._waited = False, 0
            try:
                s = self.search = HoldSearch(shield, run, world, self.units)
            except RuntimeError:                    # a thread that cannot start
                self.search, self.last = None, None
                return run()                        # the whole search, as before
            if guard_ran:
                # This tick's units went to the hand-back guard: the search's
                # first slice is the next tick's (ONE OF THE TWO A TICK).
                self._waited = 1
                return self._meanwhile(shield, fsm, here, unsafe)
        from .fsm import FSMState                  # local: as plan_for
        done = s.step()
        self.slice = round(s._used, 3)
        if not done:
            return self._meanwhile(shield, fsm, here, unsafe)
        self.search = None
        self.last = {"ticks": s.ticks, "units": round(s.spent, 3)}
        if self._waited:
            self.last["waited"] = self._waited
        if s.cancelled:
            return None
        if s.error is not None:
            land = fsm.state is FSMState.LAND
            return HoldPoint("LAND" if land else "LOITER", "in_place", here.x, here.y,
                             here.up, legal=False, breaks=_ids(unsafe),
                             reason=(f"hold-point search failed ({type(s.error).__name__}: "
                                     f"{s.error}); holding where the aircraft is, legality "
                                     f"not established"))
        return s.result

    def _meanwhile(self, shield, fsm, here: State, unsafe: list) -> HoldPoint | None:
        """What is held while the search runs: nothing new, except, once, the
        in-place plan of a re-plan (the point held is no longer legal)."""
        from .fsm import FSMState                  # local: as plan_for
        if (not self._held and self._replan and fsm.hold is not None
                and fsm.state is FSMState.LOITER):
            self._held = True
            self.last = None
            return self._in_place(shield, here, unsafe)
        return None

    def _in_place(self, shield, here: State, unsafe: list) -> HoldPoint:
        look = float(shield.lookahead_s)
        at = f"({here.x:.1f}, {here.y:.1f}, {here.up:.1f})"
        breaks = _ids(unsafe)
        now = (f"breaks {', '.join(breaks)}" if breaks else "breaks no hard rule")
        return HoldPoint(
            "LOITER", "in_place", here.x, here.y, here.up, legal=not breaks,
            breaks=breaks, checked_s=look,
            reason=(f"the point held is no longer legal: held at {at} while the "
                    f"hold-point search runs ({self.units:g} units a tick, the "
                    f"per-tick bound); standing still here {now} within the "
                    f"{look:g} s lookahead"))


def stop_cost_bound(shield) -> float:
    """What one stop query in this Shield's world is charged at most (the
    _Ledger's cost model with every moving zone near and every static zone's
    box holding the point): the price Shield.handback_unsafe sizes its
    samples by (HANDBACK_QUERIES). Rule types and motion only: no forecast."""
    rows = shield.rule_status(State(x=0.0, y=0.0, up=0.0))
    motion = shield.rule_motion()
    cost = STOP_QUERY_COST
    for r in rows:
        k = _STOP_RULE_COST.get(r["type"], 0.0)
        if r["type"] in _TUBE_TYPES:
            k *= max(1, len(r["rule"].points()) - 1)
        cost += k
        if r["type"] in _ZONE_TYPES:
            cost += MOVING_ZONE_STOP_COST if r["id"] in motion else STATIC_ZONE_STOP_COST
    return cost


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
    ledger = _Ledger(shield, state, RADII_M[-1] / speed + checked_s, site_only=True)

    def probe(x: float, y: float, alt: float) -> bool:
        # Inside a looking_ahead view: no site-choosable rule broken by a
        # stop at (x, y, alt) from the view's time through the lookahead
        # (the cheap test first, on the rules a site can satisfy).
        return ledger.stop_ok(shield, State(x=x, y=y, up=alt), LAND_SITE_CATEGORIES)

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
        if transit_breaks(shield, state, p, t_a, speed, climb, escaping, ledger):
            return None
        with shield.looking_ahead(t_a):
            for q in _neighbours(State(x=x, y=y, up=state.up)):
                if not quick(q.x, q.y, t_a):
                    return None
        # The descent's chunks: at most two altitudes and the zone edges
        # between them per lookahead-long chunk (counted as two), and the
        # corridor footprint's rule_status.
        ledger.left -= (2 * (math.ceil(descent_s / look) + 1) * ledger.stop_cost(p)
                        + ledger.read_cost(r["id"] for r in ledger.rows))
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
                   ledger, vertical=False)
    if plan is not None:
        return plan
    limit = BUDGET_NOTE if ledger.left <= 0 else f"{RADII_M[-1]:g} m"
    return HoldPoint("LAND", "in_place", state.x, state.y, state.up, legal=False,
                     breaks=tuple(bad_here), checked_s=checked_s,
                     reason=(f"{why_here}; no site within {limit} is legal by a "
                             f"legal transit ({why_speed}): landed here{floor_note}"))
