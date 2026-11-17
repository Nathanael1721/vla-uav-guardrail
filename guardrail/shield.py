"""
Safety Shield (mini) — validate a 4-D action against the policy, repair if
possible, brake only as last resort.

Flow per tick (mirrors the grant's Safety Shield node, scaled down):

    check (trend-aware)                "does this action lead somewhere illegal?"
        -> repair operators             kinematic clamp -> altitude fix, then
                                        obstacle clearance <-> geofence
                                        escape/slide ITERATED to a fixed point
        -> re-check the repaired action P0 escape guard
        -> still illegal? recovery      a heading that re-checks clean...
        -> nothing left? BRAKE          ...and a stop only where a stop is legal

Design rule learned the hard way: when the vehicle is ALREADY in violation
(below the altitude floor, inside a zone, inside the clearance ring), the right
output is a RECOVERY action, not a freeze — braking would lock the violation in
place forever. So checks are trend-aware: "in violation but actively correcting"
passes, and BRAKE is itself re-checked before it is ever emitted (a standstill
inside the clearance ring is not a fail-safe, it IS the deadlock).

The repair operators are COUPLED, so the chain is a fixed-point iteration, not a
pipeline: the geofence anti-stall tangent can leave the clearance ring violated,
and the clearance push can aim into a fence. Running each once leaves whatever
the LAST operator produced un-repaired — which the P0 guard then brakes on.

Guarantee: the action this module RETURNS never makes things worse — it is
either predicted-clean, actively recovering, or a full stop.

ENFORCEMENT (2026-10-07; docs/DESIGN-shield-enforcement.md has the whole story)

  * Every filter() tick feeds the escalation state machine (guardrail/fsm.py,
    docs/DESIGN-escalation-fsm.md). What the Shield EMITS does not depend on
    that state: `decision.emitted` is the repair stack's output, exactly as
    before, for every caller that has no autopilot whose mode it could change.
    The FSM's verdict rides beside it: `fsm_state_before/after`, the mode to
    request (`set_mode`, edge-triggered), and `command`, the action to stream
    this tick (the emitted action, a stop, or nothing while the autopilot owns
    the aircraft). The conservative cap theta is applied by the FSM, by
    default to each repair's own `magnitude_m` / `axis` (the grant's audit
    fields: the depth of the forecast it corrected, as in the p5 audit
    example). `theta_horizon_s=fsm.THETA_HORIZON_S` selects the one-period
    reading instead (the setpoint change over one 0.1 s control period; a PI
    question, docs/DESIGN-escalation-fsm.md). A caller
    that flies this verdict reports the autopilot's side through
    `filter(..., rtl_failed=, home_reached=, landed=)`; the flight rails run
    their own FSM instead (guardrail/replay.py, `rail_shield`).
  * A rule's `violation_action`, hard/soft type and priority change what the
    Shield does: `monitor_only` is recorded and never repaired; `brake`,
    `loiter`, `RTL` and `land` skip projection and stop (where a stop is
    legal); a soft rule is enforced but capped at `brake`, and is the first
    thing the repair chain gives up when it cannot satisfy every rule, lowest
    priority first. A hard rule is never given up.
  * The lookahead is the grant's 5 s at 0.1 s (50 future poses). Each pose is
    judged against the rules in force at ITS OWN time, read from the injected
    clock, on both sides of the instant (t - eps, t + eps with eps = dt / 2),
    so a window ending 17:30 ends at 17:30:00, not at 17:30:59.
  * Mid-flight events are the grant's three hot-applicable classes only:
    dynamic_nfz (spawn, move, translate, rotate, scale, expire; motion is
    enforced), time_window_switch and corridor_swap. Each is checked against
    the DSL's own consistency lint (it may not add a finding) and against the
    layer rule (it may not relax a HARD rule of another layer; REST events are
    mission-layer), bumps the generation and swaps the compiled runtime in one
    assignment. A moving zone
    is skipped only when the box it can sweep over the horizon misses the
    forecast. A polygon_fence is locked once the mission has started (the
    first tick) and is refused with `LockedRuleClass`; see `as_dynamic_nfz`
    for the migration.
  * `rule_status(state)` reports each rule's distance, state and breach from
    this module's own geometry, so the on-screen panel and the controller's
    FenceGuard read the one rule checker instead of keeping their own.
  * The forecast starts from the COMMANDED velocity unless the State carries
    the aircraft's MEASURED velocity (2026-10-19, docs/DESIGN-own-velocity-
    forecast.md): then each pose flies the setpoints already sent
    (note_flown) for the measured reaction time and turns towards the
    candidate at the measured acceleration limit (guardrail/response.py,
    `velocity_response`). That forecast only ADDS: every rule is still judged
    on the commanded forecast from the position too (`_predict`, 2026-10-20),
    so nothing the commanded forecast refused is emitted clean. A rule the
    aircraft's momentum already carries it
    into is judged from where a stop would end (`_committed`), one its
    position already breaks exactly as before, and the clearance rule asks
    whether a stop after one control period still ends clear. The position
    questions (state_is_unsafe, unsafe_depth, the FSM's stop_illegal) ignore
    the velocity. Without one, every decision is byte-identical (18,000
    seeded decisions against c77c77d, tests/test_own_velocity_forecast.py).
  * A BODY-frame action (`filter_body`, the grant's WP3-04 / ARCH-04 input
    frame) can be forecast along its ARC as well (2026-11-17, `arc=True` or
    the constructor's `arc_forecast`; off by default): its forward / right
    velocity held while the heading turns at its yaw rate, judged BESIDE
    its world direction held from t = 0, never in its place (THE ARC ONLY
    ADDS, `_predict`). A zero yaw rate is forecast bit-identically either
    way (tests/test_body_arc_forecast.py).
  * Since 2026-10-12 the repair stack runs as guardrail/repair.py's operator
    library: the chain above, decision for decision, each call recorded
    (`ShieldDecision.trace`, the audit record's `repair_passes`), with the
    grant's first-feasible selection, PathRepair and a forecast segment check
    behind constructor flags, all off by default. A moving zone's polygon is
    built only when its anchor box meets the forecast (guardrail/ir.py
    `anchor_reach`).
  * ONE RULE-EVALUATION CODE PATH and the grant's node architecture
    (2026-11-03, cards WP3-22 / X-13 / WP3-23). This module is the violation
    checker library every reader calls: the rule clock (RuleClock: windows,
    switches, swaps), the per-position verdicts (zone_breached,
    corridor_breached, the band tests, ZoneRuler) and the way out of a zone
    (escape_direction, invariant to the ring's vertex order). The Prefix
    Compiler, the fence export, the RTL hand-over, the dwell count and the
    city controller's FenceGuard ask these instead of keeping their own
    (tests/test_shieldcore_onepath.py fails if a second one appears). Each
    filter() tick admits its action and the action's predicted trajectory
    into the sliding-window buffer (SlidingWindow, the last 50) before
    anything is checked, and the checker reads the trajectory from there.
    Static zones are queried through the IR's STRtree; moving dynamic_nfz
    zones through an STRtree of their reach boxes rebuilt per tick
    (_Tick.moving_near).
"""
from __future__ import annotations

import math
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass, replace
from datetime import timedelta
from functools import lru_cache
from typing import Any, Literal, NamedTuple

import numpy as np
import shapely
from pydantic import BaseModel, Field
from shapely.geometry import LineString, Point

from . import fsm as _fsm
from . import repair as _repair
from . import response as _response
from . import terrain as _terrain
from .frames import from_body as _from_body, to_body as _to_body
from .geometry import nearest_on_polyline, point_in_fence, push_out_direction
from .ir import FENCE_TYPES, FenceRecord, PolicyIR, anchor_reach, compile_fence, is_fence
from .projection import ring_order
from .models import (
    DAYS,
    DEFAULT_LAYER,
    Action4D,
    AltitudeEnvelope,
    Corridor,
    CorridorSwap,
    DistanceEnvelope,
    DynamicNFZ,
    EnvelopeClearance,
    KinematicEnvelope,
    ObstacleClearance,
    Policy,
    PolygonFence,
    State,
    SubjectStandoff,
    TimeWindowSwitch,
    _not_looser,
)

BRAKE =Action4D(vx=0.0, vy=0.0, vz_up=0.0, yaw_rate=0.0)

# The smallest |yaw rate| (rad/s) the arc forecast turns a body-frame action
# along (Shield._arc_rate); below it the straight forecast is used. At 1e-9
# rad/s the arc leaves the line by v * w * t^2 / 2 = 5e-8 m after 5 s at
# 4 m/s, far under every tolerance, and a subnormal rate can no longer reach
# the arc's arithmetic (review 2026-11-17).
ARC_MIN_RATE = 1e-9


def _arc_offset(vx: float, vy: float, w: float, t: float) -> tuple[float, float]:
    """(dNorth, dEast) after `t` seconds of a body-frame velocity whose world
    value at t = 0 is (vx, vy), while the heading turns at `w` rad/s
    (clockwise positive, as State.yaw_deg and Action4D.yaw_rate are): the
    velocity at time s is (vx, vy) rotated by w*s, and this is its integral,
    exactly. `w` must be non-zero; w -> 0 is the straight (vx*t, vy*t).

    Forward is (cos psi, sin psi) in (North, East) (frames.from_body), so a
    rotation by theta maps (n, e) to (n cos - e sin, n sin + e cos); its
    integral over [0, t] is ((n sin wt + e (cos wt - 1)) / w,
    (n (1 - cos wt) + e sin wt) / w). Turning right (w > 0) from North
    drifts East.

    Near w * t = 0 (|w t| < 1e-3) the two quotients sin(wt) / w and
    (1 - cos wt) / w are taken from their series in w t instead (review
    2026-11-17: a subnormal w such as 5e-324 underflowed w * t, and the
    division then read x = 0.0 m at 0.2 s where the line is at 0.8 m). Any
    w, zero included, then gives the straight (vx t, vy t) in the limit."""
    x = w * t
    if abs(x) < 1e-3:
        x2 = x * x
        s_w = t * (1.0 - x2 / 6.0 + x2 * x2 / 120.0)             # sin(x) / w
        c_w = t * x * (0.5 - x2 / 24.0 + x2 * x2 / 720.0)         # (1 - cos x) / w
    else:
        s_w, c_w = math.sin(x) / w, (1.0 - math.cos(x)) / w
    return vx * s_w - vy * c_w, vx * c_w + vy * s_w


def _field_name(c) -> str:
    """Which map a clearance rule is measured on: "buildings" (the obstacle
    map: every obstacle_clearance) or "roads" (a distance_envelope over
    roads, models.RoadClearance). WP1-07, 2026-11-03."""
    return getattr(c, "MAP", "buildings")

# A subject closer than this (metres, horizontally) is directly under or over
# the aircraft: the line to it, and with it "closing" and "opening", is not
# defined. The stand-off check and its recovery treat it as one case.
_STANDOFF_COINCIDENT_M = 1e-6

# The grant's numbers (Safety Shield page, "Locked design choices"): a 5 s
# lookahead "at 10 Hz = 50 future poses", and a 10 Hz monitor. Callers that pass
# their own lookahead keep it; these are the defaults.
LOOKAHEAD_S = 5.0
LOOKAHEAD_DT_S = 0.1
MONITOR_PERIOD_S = 0.1

# Policy DSL page, "Mid-flight update model": "Three constraint classes can be
# hot-applied mid-mission: dynamic_nfz, time_window_switch, corridor_swap. All
# other classes are locked at mission start."
HOT_APPLICABLE = ("dynamic_nfz", "time_window_switch", "corridor_swap")

# Rule types this Shield enforces. Anything else a policy declares is refused
# at construction rather than flown with the rule silently ignored.
ENFORCED_TYPES = frozenset({
    "polygon_fence", "circle_fence", "dynamic_nfz", "altitude_envelope",
    "kinematic_envelope", "obstacle_clearance", "subject_standoff", "corridor",
    "time_window_switch", "corridor_swap",
    # Since 2026-11-03 (WP1-07): enforced as its runtime rule (models.
    # DistanceEnvelope.runtime_rule), see _build_runtime.
    "distance_envelope"})

# Breach actions that skip projection and stop. The PI reference's shield.py
# skips projection only when the WORST violation's action is RTL or land ("Rules
# whose action is a direct fail-safe skip projection entirely"), emits the raw
# action and lets its FSM change mode; it still repairs for brake and loiter.
# Stopping for brake and loiter too is this project's reading (follow-up #112,
# docs/DESIGN-escalation-fsm.md), and so is stopping rather than passing the
# raw action through while the mode change is requested.
_STOP_ACTIONS = ("brake", "loiter", "RTL", "land")

# The escape from a zone whose window opened on the aircraft leaves at the
# speed cap (Shield._repair_geofence). When the cap's escape cannot be flown -
# judged as flown, out of the ring and then braked to a stop
# (Shield._shaped_escape), it still breaks another hard rule -
# Shield._floor_the_escape retries it slower, down to the in-force floor
# min(2.0, cap) in steps of
# this much (or (cap - floor) / _OPENED_RETRY_STEPS when that is coarser, so
# a high cap costs no more re-runs), then with the in-force floor itself
# (_IN_FORCE_FLOOR, the shape a zone the aircraft entered by itself gets).
_OPENED_RETRY_MPS = 0.25
_OPENED_RETRY_STEPS = 8
_IN_FORCE_FLOOR = "in-force floor"


class LockedRuleClass(ValueError):
    """A mid-flight change to a rule class the grant locks at mission start."""


class HotApplyRefused(ValueError):
    """A hot-apply event that names no rule, a duplicate id, or a wrong target."""


class RuleIdConflict(HotApplyRefused):
    """The event would create a second rule with an id already in the policy."""


class UnknownRule(HotApplyRefused):
    """The event names a rule id the policy does not hold."""


class LayerRelaxation(HotApplyRefused):
    """The event would relax a HARD rule written in another layer (grant,
    Policy DSL page, "Layered authoring model": a mission-layer rule cannot
    relax a higher-priority hard constraint from regulation). The same rule
    models.merge_layers applies at ingest, applied to every event."""


MIGRATION_NOTE = (
    "polygon_fence is locked at mission start (Policy DSL page, 'Mid-flight "
    "update model': only dynamic_nfz, time_window_switch and corridor_swap may be "
    "hot-applied). Send the zone as a dynamic_nfz instead - "
    "shield.hot_apply(as_dynamic_nfz(fence)) keeps its id, vertices, band, margin, "
    "priority and breach action, with no motion; over REST, POST /dynamic_nfz. "
    "See docs/DESIGN-shield-enforcement.md, 'Migrating a polygon_fence hot-apply'.")


def as_dynamic_nfz(fence: PolygonFence, motion: dict | None = None) -> DynamicNFZ:
    """The dynamic_nfz a mid-flight polygon_fence stands for: same id,
    vertices, altitude band, margin, priority, hard/soft type and breach action.
    A circle_fence keeps its derived 32-gon as the vertices.

    This is the one-line migration for every caller that used to hot-apply a
    PolygonFence: `shield.hot_apply(as_dynamic_nfz(fence))`."""
    data = {"id": fence.id, "type": "dynamic_nfz",
            "vertices": [v.model_dump(exclude_none=True) for v in fence.vertices],
            "altitude_floor_m": fence.altitude_floor_m,
            "altitude_ceiling_m": fence.altitude_ceiling_m,
            "margin_m": fence.margin_m, "constraint_type": fence.constraint_type,
            "priority": fence.priority, "violation_action": fence.violation_action}
    for k in ("valid_time", "scope", "layer", "locked_by", "altitude_ref"):
        v = getattr(fence, k, None)
        if v is not None:
            data[k] = v.model_dump() if hasattr(v, "model_dump") else v
    if motion is not None:
        data["motion"] = motion
    return DynamicNFZ.model_validate(data)


# --------------------------------------------------------------------------- #
# Time windows, to the second
# --------------------------------------------------------------------------- #
#
# models.Recurrence.active_at compares whole minutes with an inclusive end, so a
# window written to end at 17:30 stays in force until 17:30:59. The Shield
# evaluates the same fields to the microsecond instead: in force on [start,
# end], closed, so 17:30:00.000 is the last instant. "23:59" (the DSL's default
# end) and "24:00" are read as the END of the day: as an instant, 23:59:00 would
# open a one-minute hole every night in every all-day schedule - a P0 rule
# switched off by its own default. Day attribution is models.py's: the
# instant's weekday must be listed (a window that wraps past midnight is in
# force after midnight only if that day is listed too).

_DAY_S = 86400.0


@lru_cache(maxsize=512)
def _hhmm_s(hhmm: str, end: bool = False) -> float:
    h, _, m = hhmm.partition(":")
    s = (int(h) * 60 + int(m)) * 60.0
    if end and s >= (23 * 60 + 59) * 60.0:
        return _DAY_S
    return s


def _tod_s(when) -> float:
    return (when.hour * 3600 + when.minute * 60 + when.second
            + when.microsecond / 1e6)


def recurrence_active(rec, when) -> bool:
    """Is the weekly schedule `rec` (models.Recurrence) in force at the instant
    `when`, to the microsecond? See the block comment above."""
    if DAYS[when.weekday()] not in rec.days:
        return False
    tod = _tod_s(when)
    a, b = _hhmm_s(rec.start_time), _hhmm_s(rec.end_time, end=True)
    if a <= b:
        return a <= tod <= b
    return tod >= a or tod <= b            # wraps past midnight


def _recurrence_edge_within(rec, t0, t1) -> bool:
    """Could `rec` change state anywhere in [t0, t1]? (Over-reporting is safe:
    it only sends the caller down the exact, slower path.)"""
    span = (t1 - t0).total_seconds()
    if span >= _DAY_S:
        return True
    s0 = _tod_s(t0)
    for e in (_hhmm_s(rec.start_time), _hhmm_s(rec.end_time, end=True), 0.0):
        for e2 in (e, e + _DAY_S):
            if s0 <= e2 <= s0 + span:
                return True
    return False


def _window_of(rule):
    """A rule's own weekly schedule (models.Recurrence), or None when it has
    none (no `valid_time`, or one without a recurrence: always in force). Read
    by attribute, so a duck-typed rule without the field counts as always."""
    vt = getattr(rule, "valid_time", None)
    return None if vt is None else getattr(vt, "recurrence", None)


_SWITCH = "time_window_switch"
_SWAP = "corridor_swap"


class RuleClock:
    """THE rule clock: which rules are in force at an instant. The one
    implementation in the system (grant, Safety Shield page, "Violation
    checking": "there is exactly one rule-evaluation code path").

    Built once per generation from the rule list (the Shield keeps one on its
    runtime, `Shield.rule_clock`). Every reader asks this class:

      * the Shield's per-pose schedule (`_Schedule`, both sides of each
        forecast instant) asks `enforced`;
      * the Prefix Compiler's time filter (compiler._RuleClock, a subclass
        that adds only its own sampling and wording) asks `on`, `holders`
        and `minute_on`;
      * the autopilot fence export (guardrail/fence_export.py) asks
        `off_switches`, `has_window` and `on`;
      * the dwell count of every rail (guardrail/replay.zone_dwell) asks the
        flight's Shield's own clock.

    Until 2026-11-03 the compiler held a second copy of `on` ("the ~20
    lines of rule logic only", kept equal to the Shield by tests/test_csp.py)
    and the fence export a third reading of which switches turn a zone off
    (the review of 29 Oct, card WP3-22 / X-13).

    The semantics are the Shield's, to the microsecond (block comment above):

      * no clock (x None) counts as in force ("absent means active");
      * a corridor_swap holds while its own window holds, and switches its
        target off meanwhile;
      * of the time_window_switches on a rule whose windows hold at x, the
        LAST in policy order decides (events accumulate in order); a switch
        whose window cannot be read (no clock) turns a rule ON but never OFF.

    A time_window_switch is an event, not geometry: `on` says whether it is
    ACTING (its own window holds), `enforced` says False (the monitor never
    judges a pose against it). Rules are read by attribute (`type`,
    `valid_time`, `target_id`, `active`), so the duck-typed policies the
    fence export accepts read the same way.
    """
    __slots__ = ("switches_on", "swaps_on", "timed", "recurrences")

    def __init__(self, constraints):
        cons = tuple(constraints)
        sw: dict = {}
        sp: dict = {}
        for c in cons:
            kind = getattr(c, "type", None)
            if kind == _SWITCH:
                sw.setdefault(c.target_id, []).append(c)
            elif kind == _SWAP:
                sp.setdefault(c.target_id, []).append(c)
        # target id -> the switches / swaps acting on it, in policy order.
        self.switches_on: dict = {k: tuple(v) for k, v in sw.items()}
        self.swaps_on: dict = {k: tuple(v) for k, v in sp.items()}
        # Ids whose in-force state can change with the clock, and every
        # weekly schedule in the rule set (the tick's fast path, _Schedule).
        timed = {c.id for c in cons if _window_of(c) is not None}
        timed |= set(sw) | set(sp)
        timed |= {c.id for c in cons if getattr(c, "type", None) == _SWAP}
        self.timed = frozenset(timed)
        self.recurrences = tuple(r for r in (_window_of(c) for c in cons) if r is not None)

    @classmethod
    def of(cls, policy) -> "RuleClock":
        """The clock of `policy`'s rules (a Policy, or anything with a
        `constraints` list)."""
        return cls(getattr(policy, "constraints", ()) or ())

    @staticmethod
    def _base(rule, x):
        """The rule's own window at instant x: True, False, or None when it
        has a window and there is no clock to read it."""
        rec = _window_of(rule)
        if rec is None:
            return True
        if x is None:
            return None
        return recurrence_active(rec, x)

    def on(self, rule, x) -> bool:
        """Is `rule` in force at instant x (None: no clock)? For a switch or a
        swap: is it acting (its own window holds or cannot be read)."""
        kind = rule.type
        if kind == _SWITCH or kind == _SWAP:
            return self._base(rule, x) is not False
        for sp in self.swaps_on.get(rule.id, ()):
            if self._base(sp, x) is True:
                return False
        sws = self.switches_on.get(rule.id, ())
        if sws:
            if any(s.active and self._base(s, x) is None for s in sws):
                return True
            known = [s.active for s in sws if self._base(s, x) is True]
            if known:
                return known[-1]
        return self._base(rule, x) is not False

    def enforced(self, rule, x) -> bool:
        """`on`, for the monitor: a time_window_switch is never judged as
        geometry (it acts through its target)."""
        if rule.type == _SWITCH:
            return False
        return self.on(rule, x)

    def involved(self, rule) -> bool:
        """Is `rule` a switch or a swap, or the target of one?"""
        return (rule.type in (_SWITCH, _SWAP)
                or rule.id in self.switches_on or rule.id in self.swaps_on)

    def holders(self, rule, x) -> list[str]:
        """The ids of the swaps and the switch holding `rule` OFF at x."""
        out = [sp.id for sp in self.swaps_on.get(rule.id, ()) if self._base(sp, x) is True]
        sws = [s for s in self.switches_on.get(rule.id, ()) if self._base(s, x) is True]
        if sws and not sws[-1].active:
            out.append(sws[-1].id)
        return out

    def off_switches(self, rule_id: str) -> list[str]:
        """The ids of the time_window_switches that can turn `rule_id` OFF
        (`active: false`), in policy order. A switch that only holds a rule
        ON adds no off-time to it."""
        return [s.id for s in self.switches_on.get(rule_id, ())
                if getattr(s, "active", True) is False]

    @staticmethod
    def has_window(rule) -> bool:
        """Does `rule` carry a weekly window of its own?"""
        return _window_of(rule) is not None

    @classmethod
    def minute_on(cls, rule, x) -> bool:
        """The rule's OWN window read at the start of x's minute: exactly
        models.ConstraintBase.active_at's minute-grained answer (whole
        minutes, inclusive end: a window ending 17:30 holds until 17:30:59),
        which the Prefix Compiler's time filter has always used for a rule
        no switch or swap touches - wider than `on` at a window's end, so a
        CSP never leaves out a rule the Shield enforces. Read through
        `_base`, so the one schedule reader answers it; None (no clock)
        counts as in force."""
        if x is None:
            return True
        return cls._base(rule, x.replace(second=0, microsecond=0)) is not False


# --------------------------------------------------------------------------- #
# The way out of a zone, whatever order its vertices are stored in
# --------------------------------------------------------------------------- #
#
# geometry.push_out_direction takes the nearest point of the zone's boundary
# from shapely's `project`, which returns the FIRST nearest point along the
# ring. Where two or more edges are exactly as near - the middle of a square,
# a point on its diagonal's bisector - that is decided by where the ring
# starts and which way it winds, not by the zone. Since IR schema 3.0.0
# (2026-11-02) every ring is stored in canonical order (positive winding,
# smallest point first), which moved that choice: the library cell
# dyn-nfz-spawn-on-top@size_m=40 (a 40 m zone spawned centred ON the
# aircraft) took a side edge where it had taken the forward one, the pilot's
# forward command stopped counting as escaping, and the episode ended in
# Loiter with the escape incomplete.
#
# The Shield's answer must depend on the zone, not on how it is written. So
# every escape direction the Shield uses comes from `escape_direction`: the
# same direction as push_out_direction wherever the nearest boundary point is
# unique (bit for bit: push_out_direction's own result is returned), and in an
# exact tie (nearest points within _TIE_M of each other in distance, in
# different directions) the tied direction most aligned with the heading the
# caller judges - the commanded velocity, relative to a moving zone - so a
# command already leaving through one of the equally near edges is escaping.
# With no heading (a hover), or two tied directions equally aligned with it,
# the first of them along the ring in its CANONICAL order (positive winding,
# smallest point first: projection.ring_order, recomputed here from the
# coordinates, so a ring stored in any other order answers the same). That is
# the choice the canonical rings have made since 2026-11-02 for a hover in a
# tie (tests/test_hold_points.py pins one), now a property of the zone and not
# of its storage. Once the aircraft has moved off the tie the nearest point is
# unique again, so no memory is needed.

# Two boundary points whose distances differ by less than this are a tie: far
# above the rounding of the distances (about 1e-14 m at the library's 200 m
# coordinates), far below anything a position can be measured to.
_TIE_M = 1e-9


def escape_direction(x: float, y: float, poly, vx: float = 0.0,
                     vy: float = 0.0) -> tuple[float, float]:
    """The unit vector from (x, y) towards the nearest way OUT of `poly`
    (inside it) or away from it (outside), as geometry.push_out_direction,
    with an exact tie between equally near boundary points broken by the
    heading (vx, vy) and then by the ring's canonical order - never by the
    order the ring is stored in (block comment above)."""
    base = push_out_direction(x, y, poly)
    pts = [(float(px), float(py)) for px, py in list(poly.exterior.coords)[:-1]]
    if len(pts) < 3:
        return base
    pts = [pts[i] for i in ring_order(pts)]
    ring = pts + pts[:1]
    best = math.inf
    near: list[tuple[float, float, float]] = []
    for (ax, ay), (bx, by) in zip(ring, ring[1:]):
        ex, ey = bx - ax, by - ay
        L2 = ex * ex + ey * ey
        s = 0.0 if L2 <= 0.0 else max(0.0, min(1.0, ((x - ax) * ex + (y - ay) * ey) / L2))
        nx, ny = ax + s * ex, ay + s * ey
        d = math.hypot(x - nx, y - ny)
        near.append((d, nx, ny))
        if d < best:
            best = d
    if best < 1e-9:
        return base                       # on the boundary: push_out_direction's own fallback
    inside = poly.contains(Point(x, y))
    dirs: list[tuple[float, float]] = []
    for d, nx, ny in near:
        if d > best + _TIE_M:
            continue
        ux, uy = (nx - x) / d, (ny - y) / d
        if not inside:
            ux, uy = -ux, -uy
        if all(ux * px + uy * py < 1.0 - 1e-12 for px, py in dirs):
            dirs.append((ux, uy))
    if len(dirs) < 2:
        return base                       # one nearest point (or one direction): no tie
    h = math.hypot(vx, vy)
    if h > 1e-9 and math.isfinite(h):
        top = max(ux * vx + uy * vy for ux, uy in dirs) / h
        dirs = [(ux, uy) for ux, uy in dirs if (ux * vx + uy * vy) / h >= top - 1e-9]
    return dirs[0]                        # the first along the canonical ring


# --------------------------------------------------------------------------- #
# Per-position rule tests: the violation checker's point verdicts
# --------------------------------------------------------------------------- #
#
# "Does this point break this rule?" and "how far is it from this zone?" are
# answered by these functions and nowhere else (grant, Safety Shield page,
# "Violation checking": "there is exactly one rule-evaluation code path in
# the system"). The monitor's own views call them (rule_status,
# rule_clearances, zones_now, fence_distance, the corridor check), and so do
# the readers outside it: the Prefix Compiler's action check
# (compiler.action_violations), its relevance facts and its lint, the cost
# map's band, the RTL hand-over (guardrail/rtl_path.py) and the city
# controller (demo/follow_vlm.py FenceGuard, through `ZoneRuler`). The
# comparisons are the ones the monitor always made: a zone's band with its
# ends included, the interior of its margin ring (geometry.point_in_fence on
# the ring compiled once in the IR), a corridor's half-width and band with
# 1e-9 m of slack. Until 2026-11-03 compiler.action_violations judged a
# corridor without that slack, and rtl_path, FenceGuard and the lint each
# made these comparisons themselves (card WP3-22).

_CORRIDOR_SLACK_M = 1e-9


def band_holds(lo: float, hi: float, up: float) -> bool:
    """Is altitude `up` within [lo, hi], ends included: a keep-out zone's
    band test (geometry.point_in_fence's first comparison)."""
    return lo <= up <= hi


def in_zone_band(zone, up: float) -> bool:
    """Is altitude `up` within keep-out zone `zone`'s band?"""
    return band_holds(zone.altitude_floor_m, zone.altitude_ceiling_m, up)


def zone_breached(zone, x: float, y: float, up: float, polygon=None,
                  buffered=None) -> bool:
    """Does the point (x, y, up) break keep-out zone `zone`: inside its margin
    ring AND within its band? `buffered` is the ring compiled once in the IR
    (FenceRecord.buffered); without it the ring is built here."""
    return point_in_fence(x, y, up, zone, polygon, buffered)


def corridor_offset(corridor, x: float, y: float) -> float:
    """Metres from (x, y) to a corridor's centreline (its segments)."""
    return nearest_on_polyline(x, y, corridor.points())[0]


def corridor_band_outside(corridor, up: float) -> tuple[bool, bool]:
    """(below the corridor's floor, above its ceiling) at altitude `up`."""
    return (up < corridor.altitude_floor_m - _CORRIDOR_SLACK_M,
            up > corridor.altitude_ceiling_m + _CORRIDOR_SLACK_M)


def corridor_wide(corridor, offset: float) -> bool:
    """Is a lateral `offset` from the centreline outside the corridor?"""
    return offset > corridor.half_width_m + _CORRIDOR_SLACK_M


def corridor_in_band(corridor, up: float) -> bool:
    below, above = corridor_band_outside(corridor, up)
    return not (below or above)


def corridor_breached(corridor, x: float, y: float, up: float) -> bool:
    """Does the point (x, y, up) break keep-in corridor `corridor`: wider
    than its half-width or outside its band?"""
    return corridor_wide(corridor, corridor_offset(corridor, x, y)) or \
        not corridor_in_band(corridor, up)


class ZoneRuler:
    """Distances from points to the margin rings of a set of compiled zones:
    `Shield.zone_ruler()`, the zones in force at one tick, read once and
    asked for as many points as the caller needs (FenceGuard asks for a
    dozen per control tick). The distance is the one Shield.fence_distance
    reports: 0 inside a ring, inf with no zone."""
    __slots__ = ("records", "_rings")

    def __init__(self, records):
        self.records = tuple(records)
        self._rings = tuple(r.buffered for r in self.records)

    @classmethod
    def of_rings(cls, rings) -> "ZoneRuler":
        """The same measure over margin rings given directly (shapely
        polygons), for a caller that is handed zones rather than reading
        them from a Shield (a test that puts a fence where it needs one)."""
        ruler = cls(())
        ruler._rings = tuple(rings)
        return ruler

    def __bool__(self) -> bool:
        return bool(self._rings)

    def __len__(self) -> int:
        return len(self._rings)

    def distance(self, x: float, y: float) -> float:
        p = Point(x, y)
        return min((g.distance(p) for g in self._rings), default=math.inf)


def _sanitise(a: Action4D) -> tuple[Action4D, list[str]]:
    """Force every channel finite, returning the names of the ones that were not.

    NaN fails EVERY comparison, so an action carrying one sails through _check()
    with zero violations and out through the untouched-passthrough branch of
    filter() — the guardrail fails OPEN, which is the one direction it must never
    fail. Infinity is no better: the speed clamp scales it by cap/hypot, and
    inf * 0.0 is NaN, so a bounded repair operator manufactures the poison.

    This is reachable from a real pilot, not just from a fuzzer. servo() sizes
    forward speed from the detector's box width, so a zero-width box is one
    division away from NaN, and a detector that fails mid-flight is a normal
    event rather than an exotic one.

    Fail-safe reading: a non-finite command is not a command. The channel goes to
    zero, and the caller sees a violation, so it is never silent.
    """
    bad = [n for n, v in (("vx", a.vx), ("vy", a.vy),
                          ("vz_up", a.vz_up), ("yaw_rate", a.yaw_rate))
           if not math.isfinite(v)]
    if not bad:
        return a, bad
    return Action4D(
        vx=a.vx if math.isfinite(a.vx) else 0.0,
        vy=a.vy if math.isfinite(a.vy) else 0.0,
        vz_up=a.vz_up if math.isfinite(a.vz_up) else 0.0,
        yaw_rate=a.yaw_rate if math.isfinite(a.yaw_rate) else 0.0,
    ), bad

# Chamfer(1, sqrt2) OVER-estimates true Euclidean distance by at most 8.24%
# (worst case at atan(sqrt2-1) = 22.5 deg). Over-estimating clearance is the
# UNSAFE direction — it would report "further from the wall than we are" — so
# every distance is scaled by 1/1.0824. The field is therefore a conservative
# lower bound on the true distance, never an optimistic one.
_CHAMFER_CORR = 1.0 / 1.08239

# Floor on the clearance speed taper: the repair SLOWS, it never stops.
# Stopping is the brake's job (and the brake is a deliberate, audited event).
_CLEAR_MIN_SCALE = 0.25

# Nominal horizontal deceleration (m/s^2) used to SIZE the clearance forecast.
# The clearance question is "can I still stop before the ring?", NOT "where does
# a 3 s straight line end up". Scanning the full geofence horizon at constant
# velocity turns min_clearance_m into an effective standoff of
# min_clearance_m + v * lookahead_s (5 m -> 17 m at 4 m/s), so the Shield ends up
# vetoing every planned route whose corridor is narrower than that — including
# routes its own planner produced.
_CLEAR_DECEL_MPS2 = 3.0

# How often the repair chain is re-run before giving up. The operators are
# COUPLED (the geofence anti-stall tangent can break clearance; the clearance
# push can aim into a fence), so the chain is iterated to a fixed point instead
# of being treated as a one-way pipeline whose last stage is never re-repaired.
_REPAIR_PASSES = 3

# Loiter's hand-back guard (Shield.handback_unsafe, since 2026-11-07): the
# straight flight of the request is sampled this often through the lookahead,
# a stop asked at each sample (ten stop queries a Loiter tick at the grant's
# 5 s, made only while the position itself is legal).
HANDBACK_STEP_S = 0.5

# How the Shield recovers from a position that is already illegal (Shield's
# recovery_law, WP3-17):
#   "cap_all"    the default since 2026-11-06: every recovery at its cap.
#                The altitude band as "cap"; GeofenceEscape from a static zone
#                at the speed cap (was the reference's fixed 2.0 m/s) through
#                the path a zone whose window opened on the aircraft already
#                took - judged as flown, out of the ring and then a stop
#                (_shaped_escape), slower only where the cap's stop breaks
#                another rule (_opened_retries), reported to the FSM if none
#                can be flown (_escape_shortfall); StandoffRecover at the speed
#                cap (was (ring - d) / L floored at 0.5 m/s); CorridorReturn at
#                the fastest inward speed, up to the cap, whose lookahead
#                endpoint stays a re-entry margin short of the far wall (was
#                (d - half) / L floored at 0.5 m/s). A moving zone keeps the
#                floor (no shaped escape for a moving ring).
#   "cap"        2026-10-29 to 2026-11-06: the altitude band at the climb cap,
#                the lateral recoveries on the laws above.
#   "lookahead"  before 2026-10-29: the band in one lookahead too (an
#                exponential approach).
# Each law is a FLOOR (a faster legal escape is never slowed), and recoveries
# stay exempt from theta under every one. Every bundle names its law
# (sweep_scenarios `arm.recovery`), so a run on an older law re-flies on it.
RECOVERY_LAWS = ("cap", "lookahead", "cap_all")
DEFAULT_RECOVERY_LAW = "cap_all"

# The dead reckoning's drift allowance, m/s (Shield(dr_drift_mps=...)), while
# the position is not a valid fix (State.fix_ok False; _position_verdict, the
# FSM's X8). The Shield dead-reckons on the commands it streamed, which is
# exact on the harness's ideal vehicle and not on an aircraft: the measured
# ArduCopter GUIDED response flies a setpoint from 0.45 s after it is sent at
# no more than 1.75 m/s^2 (guardrail/response.py ARDUCOPTER_GUIDED), and a wind
# estimate or a gust moves it where no command did. The allowance lets the
# true position wander from the estimate at this rate since the last valid
# fix: the error the Shield must assume grows by it, and the boundary may be
# that much nearer, so X8 can only come earlier with it, never later. 0.5 m/s
# is a judgement - an eighth of the shipped 4 m/s cap, about the residual
# velocity error of a closed loop that holds a steady command - recorded with
# every X8 reason and per harness episode, so a run on another value is told
# apart; a rail that reports its own position variance should set it from
# that. 0 reproduces the rubric's E3 exactly on the ideal vehicle. Not read
# while the autopilot vouches for its own estimate (State.estimator_ok True,
# 2026-11-12): there the handed pose IS the autopilot's navigation estimate,
# and a dead reckoning on commands is no measure of its error.
DR_DRIFT_MPS = 0.5

# Length of the sliding window of recent ticks. Safety Shield page, node
# architecture: "Sliding-window buffer - last 50 actions + predicted
# trajectory", feeding the violation checker. 50 ticks is 5 s at the 10 Hz
# monitor rate, the same span as the grant's lookahead.
HISTORY_LEN = 50

# Padding (metres) of the forecast box queried against the per-tick R-tree
# of moving zones (_Tick.moving_near): the tree's answer is then a superset of
# the zones whose reach box meets the forecast's, boundary contacts included,
# and the exact test (Shield._reach) decides. A larger pad costs only an
# exact test that was not needed, never an answer.
_INDEX_PAD_M = 1e-6


def _dedupe(repairs: list["Repair"]) -> list["Repair"]:
    """One audit line per distinct fix — the repair chain is iterated, so the
    same operator can legitimately log the same detail more than once."""
    seen, out = set(), []
    for r in repairs:
        key = (r.operator, r.detail)
        if key not in seen:
            seen.add(key)
            out.append(r)
    return out


def _build_distance_field(occ, res: float):
    """Metres from each free cell to the nearest BLOCKED cell centre.

    Pure-numpy two-pass chamfer distance transform (no scipy). Each pass walks
    the rows once, propagating from the previous row (orthogonal cost `res`,
    diagonal cost `res*sqrt(2)`) and then running the in-row propagation as a
    vectorised min-scan, using the identity

        min_{k<=j} d[k] + a*(j-k)  ==  cummin(d - a*j) + a*j

    so the whole thing is O(N) numpy calls instead of O(N^2) python.

    Caveat by construction: distance is to the nearest occupied cell CENTRE,
    so a building face sits up to res/2 closer than the number says. Choose
    min_clearance_m with that quantisation in mind (the maps are res = 2 m).
    """
    occ = np.asarray(occ)
    if occ.ndim != 2:
        raise ValueError(f"obstacle_map['occ'] must be 2-D, got shape {occ.shape}")
    n, m = occ.shape
    a = float(res)                 # orthogonal step cost
    b = a * math.sqrt(2.0)         # diagonal step cost
    big = (n + m + 2) * a          # finite "unreachable" sentinel (no inf math)

    d = np.where(occ > 0, 0.0, big).astype(np.float64)
    idx = np.arange(m, dtype=np.float64)
    a_idx = a * idx

    def _hscan(row):
        """In-row propagation, both directions (vectorised min-scan)."""
        row = np.minimum(row, np.minimum.accumulate(row - a_idx) + a_idx)
        rev = (row + a_idx)[::-1]
        return np.minimum(row, np.minimum.accumulate(rev)[::-1] - a_idx)

    def _diag(prev):
        """min(prev[j-1], prev[j+1]) with `big` outside the grid."""
        left = np.empty_like(prev)
        left[0] = big
        left[1:] = prev[:-1]
        right = np.empty_like(prev)
        right[-1] = big
        right[:-1] = prev[1:]
        return np.minimum(left, right)

    # forward pass: rows top -> bottom
    d[0] = _hscan(d[0])
    for i in range(1, n):
        prev = d[i - 1]
        d[i] = np.minimum(d[i], np.minimum(prev + a, _diag(prev) + b))
        d[i] = _hscan(d[i])
    # backward pass: rows bottom -> top
    for i in range(n - 2, -1, -1):
        nxt = d[i + 1]
        d[i] = np.minimum(d[i], np.minimum(nxt + a, _diag(nxt) + b))
        d[i] = _hscan(d[i])

    return d * _CHAMFER_CORR


def _build_signed_distance_field(occ, res: float):
    """SIGNED distance: positive metres to the nearest obstacle out in free
    space, NEGATIVE metres to the nearest free cell when the point is INSIDE an
    obstacle footprint.

    An unsigned field is exactly 0 across a whole building footprint, so its
    gradient there is flat and "the way out" is undefined. That is not a corner
    case: the 2-D grid treats buildings as infinitely tall columns, so a drone
    flying OVER a building (the demo's ESCAPE mode climbs on purpose) sits on
    those dead cells at any altitude, gets "no direction" back, and freezes —
    the exact deadlock the module docstring forbids. Signing the field gives
    every point inside a building a well-defined shortest way out, so the repair
    always has something to steer by.

    The inside half is deliberately NOT chamfer-corrected: over-estimating how
    deep inside we are is the conservative direction.
    """
    occ = (np.asarray(occ) > 0).astype(np.uint8)
    outside = _build_distance_field(occ, res)
    inside = _build_distance_field(1 - occ, res) / _CHAMFER_CORR
    return outside - inside


def _field_lipschitz(d, res: float) -> float:
    """How many metres `Shield._distance_at` can change per metre the point
    moves, anywhere - in the grid and off it - measured from the field `d`
    itself (RuleClearance.h_lip; the hold-point transit test bounds the
    clearance between two evaluated poses with it, holdpoint.transit_breaks).

    In the grid the field is bilinear in each cell, so its x-slope is at most
    the largest difference between x-neighbours over `res` (lx), likewise y
    (ly), and its gradient at most hypot(lx, ly). Off the grid
    `_distance_at` returns max(v - off, min(v + off, off - res/2)), with v
    the bilinear value at the nearest edge point (it moves along the edge,
    at most max(lx, ly)) and off the distance to it (1 per metre): at most
    hypot(max(lx, ly), 1).

    Until 2026-10-20 (re-review of T1c) the transit test assumed sqrt(2),
    "each axis changes by at most one cell per cell". That holds for the
    outside chamfer field alone (_CHAMFER_CORR < 1) but not across a building
    face, where a free cell reads res * _CHAMFER_CORR and its occupied
    neighbour -res: a jump of 1.92 cells per cell, so 2.72 m per metre in the
    grid; off the grid the `v + off` branch carries that along-edge slope at
    any height (5 m off the map beside a border building the field falls
    from 3.77 to 3.00 m in 0.4 m, at a 3 m limit; 2.17 m per metre), so no
    clearance limit, however large, avoided it. Measured on a 20 x 20 map at
    res 2 m with a building inside and one on the border (200000 random
    probes): slopes up to 2.70 in the grid and 2.17 off it, under this
    function's 2.72 (tests/test_hold_transit.py)."""
    a = np.asarray(d, dtype=np.float64)
    r = float(res)
    lx = float(np.abs(np.diff(a, axis=0)).max()) / r if a.shape[0] > 1 else 0.0
    ly = float(np.abs(np.diff(a, axis=1)).max()) / r if a.shape[1] > 1 else 0.0
    return max(math.hypot(lx, ly), math.hypot(max(lx, ly), 1.0))


class Violation(BaseModel):
    rule_id: str
    category: str            # "kinematic" | "altitude" | "geofence" | "clearance"
    detail: str
    predicted_at_s: float    # how far into the lookahead it happens (0 = now)


class Repair(BaseModel):
    operator: str
    detail: str
    # The grant's audit fields (Safety Shield page, audit record:
    # `repair_attempts[].magnitude_m`). The size of the POSITION correction this
    # operator made, in metres, and on which axis: for a lateral operator the
    # penetration depth of the forecast it repaired into the ring this Shield
    # enforces (a zone's MARGIN ring, the clearance / stand-off radius, the
    # corridor half-width), for a vertical one the altitude error. The PI
    # reference measures a zone's depth into the authored polygon, with no
    # margin; measuring into the margin ring is this project's reading (open PI
    # question, docs/DESIGN-shield-enforcement.md). The escalation FSM applies the
    # conservative cap theta (2.0 m lateral / 0.5 m vertical) to their per-axis
    # sum. None for the kinematic clamps, Sanitise and Brake, which correct a
    # velocity, not a position.
    magnitude_m: float | None = None
    axis: Literal["lateral", "vertical"] | None = None
    # A recovery: the aircraft was ALREADY outside the rule and this operator
    # brings it back. Exempt from theta (the reference's "brake while inside =
    # deadlock"); GeofenceEscape and StandoffRecover are recoveries by name.
    recovery: bool = False


Setpoint = Literal["pass", "brake", "none"]


class ShieldDecision(BaseModel):
    raw: Action4D
    emitted: Action4D
    violations: list[Violation] = []
    repairs: list[Repair] = []
    braked: bool = False
    # The policy this decision was made under: the generation and hash of the
    # compiled runtime the tick read (a hot-apply between ticks cannot restamp
    # a decision already made).
    generation: int | None = None
    policy_hash: str | None = None
    # Soft rules the repair chain gave up to satisfy the hard ones (priority
    # order, lowest first). They stay in `emitted_violations`.
    relaxed: list[str] = []
    # The escalation FSM's verdict for this tick (None when the Shield was
    # built with escalation=False). `command` is what to stream: the emitted
    # action ("pass"), a stop ("brake") or nothing ("none": LOITER / RTL /
    # LAND own the aircraft). `set_mode` is the ArduPilot mode to request NOW,
    # only on the tick the state changes.
    fsm_state_before: str | None = None
    fsm_state_after: str | None = None
    fsm_edge: str | None = None
    set_mode: str | None = None
    setpoint: Setpoint | None = None
    command: Action4D | None = None
    fsm_record: dict | None = None
    # A Shield/FSM contract break (the FSM refused this tick's input). The
    # first one requests LOITER; streaming stops until the episode is reset.
    fsm_fault: str | None = None
    # The Shield's re-check of `emitted` (the repair stack's output), of ANY
    # priority. It is the FLOWN action only when the Shield is on and
    # `setpoint` is "pass" (or None: no FSM). On "brake" a zero action flew
    # (`command`), on "none" nothing did (LOITER / RTL / LAND own the
    # aircraft), and on a Shield-off run the raw action flew. The KPI's escape
    # count comes from the rails' flown re-check (guardrail.replay.
    # flown_fields), not from this field. (Until 2026-10-19 this comment, like
    # its twin in guardrail/audit.py, called it the action actually flown.)
    #
    # `violations` is the check on `raw`, so on its own it cannot answer the
    # P0 escape KPI (limit 0) - "a P0 that was seen and then flown anyway". The escape
    # rate was inferred instead, from whether the Shield had done *something*
    # (see guardrail/kpi.py), and since every branch that produces a violation
    # also appends a Repair, that inference could not return a non-zero number
    # for any log this Shield can generate.
    #
    # Recording the re-check makes the KPI a measured fact rather than an
    # inference, and makes it auditable offline from the artefact alone. Empty
    # is the good case and the overwhelmingly common one: re-checked against
    # every delivered flight, the emitted action violated a P0 rule on zero
    # ticks. Since 2026-10-07 it also lists monitor_only rules (recorded, never
    # repaired) and soft rules given up (`relaxed`), of any priority; filter to
    # P0 for the grant's escape KPI.
    emitted_violations: list[Violation] = []
    # How the decision was reached, for the audit record (guardrail/audit.py)
    # and nothing else: the heading the body frame is taken at (`yaw_deg`),
    # every repair operator considered pass by pass, in every chain run, with
    # its result and the reason for a skip (`passes`, `iterations`,
    # `chain_runs`, `flown_run`), the pass each Repair came from
    # (`repair_iterations`, index for index with `repairs`), the repair
    # mode and flags, and the rules whose violation a window edge inside the
    # lookahead caused (`window_edge`). Excluded from model_dump(), so flight
    # logs, digests and every comparison of dumped decisions are unchanged.
    trace: dict | None = Field(default=None, exclude=True)
    # Where the autopilot holds (FSM in Loiter) or lands (Land): the FSM's
    # current fsm.HoldPoint as a dict, None in every other state (since
    # 2026-10-19; guardrail/holdpoint.py). "in_place" is ArduPilot's own
    # LOITER / BRAKE / LAND where the aircraft is; "moved" asks the
    # integration to fly to (x, y, up) first. The same dict is in
    # `fsm_record["hold_point"]`. Excluded from model_dump() like `trace`, so
    # a dumped decision of any other state is byte for byte what it was.
    hold_point: dict | None = Field(default=None, exclude=True)
    # X8's input as THIS Shield judged it, (reason, rule id), for a rail that
    # runs the escalation FSM itself (guardrail/replay.py rail_shield: a
    # Shield with no FSM of its own): `_position_verdict` on the state it was
    # handed (2026-11-07, the ROS 2 node's /mavros/global_position/raw/fix).
    # Computed only for a state that reports its fix quality (State.fix_ok or
    # pos_sigma_m); None otherwise and on a Shield with its own FSM, which
    # reads the verdict directly. Excluded from model_dump() like `trace`.
    position_lost: tuple[str, str] | None = Field(default=None, exclude=True)

    @property
    def touched(self) -> bool:
        # The repairs too: since 2026-10-19 a legal action from an illegal
        # position can be floored (Shield._floor_the_escape) with nothing
        # violated, and an audit log or HUD keyed on violations alone would
        # not show the Shield changing the command.
        return bool(self.violations) or bool(self.repairs)

    @property
    def escaped(self) -> bool:
        """Some rule is still violated by `emitted`, of ANY priority:
        `emitted_violations` also holds monitor_only rules (never repaired, by
        design) and soft rules given up (`relaxed`). `emitted` is the action
        flown only when the Shield is on and `setpoint` is "pass" (or None: no
        FSM); on "brake" a zero action flew and on "none" nothing did, so this
        is the Shield's own re-check, not the flight's. The grant's P0 escape
        rate is the FLOWN re-check filtered to P0 rules (guardrail.replay.
        flown_fields, guardrail/kpi.py, the policy HUD); the decision does not
        carry priorities."""
        return bool(self.emitted_violations)


class _Pose(NamedTuple):
    """One forecast pose (Shield._predict): State's four fields, unvalidated
    because they are computed from validated floats."""
    x: float
    y: float
    up: float
    yaw_deg: float


class RuleClearance(NamedTuple):
    """One rule's signed room at a position (Shield.rule_clearances): the
    horizontal part `h`, the vertical parts `lo` (up - floor) and `hi`
    (ceiling - up), whether the rule keeps OUT (a zone) or IN (everything
    else), rule_status's in_force and breach, and `h_lip`: how many metres
    `h` can change per metre the position moves with the rule standing still
    (1.0 for a distance to a set - a zone's ring, a corridor's centreline, a
    stand-off's subject - and the distance field's measured bound for
    obstacle clearance, _field_lipschitz)."""
    keep_out: bool
    h: float
    lo: float
    hi: float
    in_force: bool
    breach: bool
    h_lip: float = 1.0

    @property
    def room(self) -> float:
        """> 0: the position does not breach the rule (its parts combined as
        the rule combines them: a zone is breached only inside its ring AND
        within its band, a keep-in rule past any one limit)."""
        if self.keep_out:
            return max(self.h, -self.lo, -self.hi)
        return min(self.h, self.lo, self.hi)


class TickRecord(NamedTuple):
    """One entry of the Shield's sliding window: where the vehicle was, the
    action it was given (`decision.raw`), the predicted trajectory the
    violation checker judged that action on (`trajectory`), and what the
    Shield decided.

    `trajectory` (since 2026-11-03, WP3-23): the forecast of `decision.raw`
    from `state`, as (t, pose) pairs over the lookahead - the poses the
    checker read from the buffer (SlidingWindow). A pose has x, y, up and
    yaw_deg. For a state with a measured velocity it holds two poses per grid
    time, the own-velocity one then the commanded one, exactly as the rules
    walked them (Shield._predict). Until that date the buffer kept no
    trajectory: it was "recomputable", and the checker never read the buffer.
    The forecast of the EMITTED action is still recomputable on demand
    (`Shield.forecast(rec.state, rec.decision.emitted, pipe=rec.pipe)`), with
    `arc=rec.arc` for a tick forecast along the arc.

    `arc` (2026-11-17, WP3-04 / ARCH-04): True when this tick's forecasts
    added the ARC of the action's yaw rate (Shield.filter_body with the arc
    on, a yaw rate over ARC_MIN_RATE; the decision's trace says
    "forecast_arc" too). Its `trajectory` then holds the line's pose(s) then
    the arc's at each grid time, and `forecast(..., arc=True)` rebuilds the
    arc (the plain call, the line). False on every other tick.

    `elapsed_ms` is how long that `filter()` call took, wall clock. The grant
    budgets the monitor at 100 ms per tick (Safety Shield page) and no flight
    rail recorded the Shield's share of it (audit card WP3-13); measuring it
    here, once, means a rail only has to log `history[-1].elapsed_ms` rather
    than wrap its own timer around a call it might later move.

    `pipe` (2026-10-20 review): for a state with a measured velocity the
    forecast also depends on the setpoints already sent (`Shield.note_flown`)
    and on when this tick was (`Shield.pipe`), neither of which `state`
    holds; the tick's reaction segments are stored here so the rebuild above
    is exact. None for a state without a measured velocity, whose forecast
    depends on (state, action) alone, as it always did."""
    state: State
    decision: ShieldDecision
    elapsed_ms: float
    pipe: tuple | None = None
    trajectory: tuple | None = None
    arc: bool = False


class _Admitted(NamedTuple):
    """The action the current tick admitted into the buffer, with its
    predicted trajectory, before the decision on it is made."""
    tick: Any
    state: Any
    action: Any
    key: tuple
    trajectory: tuple


class SlidingWindow:
    """The Shield node's sliding-window buffer (grant, Safety Shield page,
    node architecture: Action subscriber -> "Sliding-window buffer: last 50
    actions + predicted trajectory" -> Violation checker).

    Each filter() tick ADMITS the action it was given, with the predicted
    trajectory of that action from the decision state, before anything is
    checked (`admit`). The checker then READS that trajectory from here: the
    Shield's forecast (`Shield._predict`) of the admitted (state, action) on
    the admitting tick is the buffer's, so the geometric, envelope and
    time-window rules all walk the poses the buffer holds (`trajectory_for`).
    When the decision is made the entry is closed into a TickRecord
    (`close`) and kept with the last HISTORY_LEN - 1 before it, oldest first
    (`records`, the Shield's `history`).

    What reads the older entries: `Shield.history` (the HUD, the audit and
    episode writers, the sweep). No rule of this DSL reads PAST actions, so
    the checker reads the newest entry only; the other 49 are what a rule
    over recent actions (an oscillation rule, say) would read.

    Only the tick that admitted an entry can read its trajectory, and only
    for that (state, action): `advise()` and the look-ahead views admit
    nothing and forecast as before; a repair candidate is another action
    and is forecast as before. The forecast is a pure function of (state,
    action) on a tick, so reading it from here or computing it again gives
    the same poses: every decision is unchanged (tests/test_shieldcore_
    onepath.py; the library sweep and the identity tests)."""
    __slots__ = ("_records", "_head")

    def __init__(self, maxlen: int = HISTORY_LEN):
        self._records: deque[TickRecord] = deque(maxlen=maxlen)
        self._head: _Admitted | None = None

    @staticmethod
    def _key(action) -> tuple:
        return (action.vx, action.vy, action.vz_up, action.yaw_rate)

    def admit(self, tick, state, action, trajectory) -> None:
        """The action subscriber's hand-over: `action` from `state` on
        `tick`, with its predicted trajectory."""
        self._head = _Admitted(tick, state, action, self._key(action), tuple(trajectory))

    @property
    def head(self) -> "_Admitted | None":
        """The entry admitted on the current tick and not yet closed."""
        return self._head

    def trajectory_for(self, tick, state, action) -> tuple | None:
        """The admitted trajectory, when (tick, state, action) is the
        admitted entry's; else None (the caller forecasts it)."""
        h = self._head
        if h is None or h.tick is not tick or h.state is not state:
            return None
        if action is not h.action and self._key(action) != h.key:
            return None
        return h.trajectory

    def close(self, state, decision, elapsed_ms: float, pipe, *,
              arc: bool = False) -> TickRecord:
        """The decision on the admitted entry: kept as a TickRecord."""
        h, self._head = self._head, None
        rec = TickRecord(state, decision, elapsed_ms, pipe,
                         h.trajectory if h is not None else None, bool(arc))
        self._records.append(rec)
        return rec

    def clear(self) -> None:
        self._records.clear()
        self._head = None

    def records(self) -> tuple[TickRecord, ...]:
        return tuple(self._records)

    def __len__(self) -> int:
        return len(self._records)


def _effective_action(c) -> str:
    """What the Shield does about rule `c`: the escalation FSM's own table
    (fsm.RuleHit.effective_action), so the repair stack and the FSM cannot
    disagree. Soft rules are capped at `brake` there."""
    return _fsm.RuleHit(c.id, c.violation_action, c.constraint_type,
                        c.priority).effective_action


@dataclass(frozen=True)
class _Runtime:
    """Everything the monitor derives from the policy, built once per
    generation and swapped in ONE assignment, so a tick running on the control
    thread reads either the old rule set or the new one, never half of each
    (events arrive on the REST thread)."""
    ir: PolicyIR
    constraints: tuple
    kins: tuple
    alts: tuple
    clear: tuple
    standoffs: tuple
    corridors: tuple            # corridors and corridor_swap rules
    clock: RuleClock            # THE rule clock of this generation's rules
    switches_on: dict           # target id -> time_window_switch rules, policy order (clock's)
    swaps_on: dict              # target id -> corridor_swap rules, policy order (clock's)
    eff: dict                   # rule id -> effective breach action
    hard: dict                  # rule id -> hard?
    timed: frozenset            # ids whose in-force state can change with time or events
    recurrences: tuple          # every weekly schedule in the policy
    anchors: dict               # moving dynamic_nfz id -> anchor time (s), None = first tick
    moving_ids: frozenset


def _runtime_rule(c, ground):
    """The rule the runtime enforces for declared rule `c` (2026-11-03): a
    distance_envelope as the runtime rule of its class (WP1-07: buildings and
    roads as clearance rules on the obstacle and road maps, people as a
    stand-off from a tracked person), an MSL band as the AGL band over
    `ground` (X-11, guardrail/terrain.py). Same id, priority, breach action
    and schedule, so every check, repair, status row and audit record names
    the declared rule. Every other rule is itself."""
    if isinstance(c, DistanceEnvelope):
        c = c.runtime_rule()
    if ground is not None and _terrain.is_msl(c):
        c = _terrain.agl_rule(c, ground)
    return c


def _build_runtime(policy: Policy, prev: "_Runtime | None", anchors: dict,
                   ground=None) -> _Runtime:
    rules = [_runtime_rule(c, ground) for c in policy.constraints]
    cons = tuple(rules)
    clock = RuleClock(cons)
    ir = PolicyIR.from_policy(policy, reuse=prev.ir if prev is not None else None,
                              rules=rules)
    moving = frozenset(r.rule.id for r in ir.fences if r.moving)
    return _Runtime(
        ir=ir, constraints=cons,
        kins=tuple(c for c in cons if isinstance(c, KinematicEnvelope)),
        alts=tuple(c for c in cons if isinstance(c, AltitudeEnvelope)),
        clear=tuple(c for c in cons if isinstance(c, ObstacleClearance)),
        standoffs=tuple(c for c in cons if isinstance(c, SubjectStandoff)),
        corridors=tuple(c for c in cons if isinstance(c, (Corridor, CorridorSwap))),
        clock=clock, switches_on=clock.switches_on, swaps_on=clock.swaps_on,
        eff={c.id: _effective_action(c) for c in cons},
        hard={c.id: c.constraint_type == "hard" for c in cons},
        timed=clock.timed, recurrences=clock.recurrences,
        anchors={k: v for k, v in anchors.items() if k in moving}, moving_ids=moving)


class _Schedule:
    """Which rules are in force at each instant of one tick's lookahead.

    A pose at forecast time tau is judged against the rules in force at
    now + tau, and "in force" is checked on BOTH sides of that instant (grant,
    Safety Shield page, Violation checking: "handle the 'boundary instant' by
    checking both sides at t- and t+"), with eps = dt / 2, so every window
    edge inside the horizon is seen from both sides by the pose nearest it.
    The test samples the two instants; it does not intersect the window with
    [tau - eps, tau + eps]. A window shorter than eps can therefore fall
    between the samples, and a zero-length one (start == end, an instant here,
    a whole minute in models.Recurrence.active_at) is never seen (review
    probe, 2026-10-07). The DSL lint does not refuse one yet.

    Fast path: when no schedule in the policy has an edge anywhere in
    [now - eps, now + horizon + eps], every rule's state is constant over the
    tick and is read once. Without a clock every rule is in force, as
    ConstraintBase.active_at has always said ("absent means active").
    """
    __slots__ = ("rt", "when", "eps", "grid", "const", "_cache", "_shared")

    def __init__(self, rt: _Runtime, when, eps: float, grid: tuple):
        self.rt, self.when, self.eps, self.grid = rt, when, eps, grid
        self._cache: dict = {}
        # tau -> in force, per weekly window, shared by every PLAIN rule with
        # that window (`_plain_key`): 50 rules on one schedule read it once
        # per pose instead of 50 times.
        self._shared: dict = {}
        if not rt.timed or when is None:
            self.const = True
        else:
            lo = when - timedelta(seconds=eps)
            hi = when + timedelta(seconds=grid[-1] + eps)
            self.const = not any(_recurrence_edge_within(r, lo, hi)
                                 for r in rt.recurrences)

    def on(self, rule, x) -> bool:
        """Is `rule` enforced at instant x (switches and swaps applied)? The
        runtime's RuleClock answers (`RuleClock.enforced`): the one rule
        clock, which the Prefix Compiler, the fence export and the dwell
        count read too.

        * A corridor_swap rule is enforced while its own window holds; the
          corridor it targets is replaced (off) while that window holds.
        * time_window_switch rules targeting `rule`: of those in force, the
          LAST in policy order decides (events accumulate in order). A switch
          that turns the rule ON but whose window cannot be read (no clock)
          applies; one that turns it OFF does not - nothing that cannot
          establish "this rule is off now" may switch it off.
        * Unknown (no clock) counts as in force, for the same reason. A swap
          of unknown window therefore enforces both corridors, the stricter
          reading."""
        return self.rt.clock.enforced(rule, x)

    def in_force(self, rule, tau: float) -> bool:
        if self.when is None:
            return self.on(rule, None)
        return (self.on(rule, self.when + timedelta(seconds=tau - self.eps))
                or self.on(rule, self.when + timedelta(seconds=tau + self.eps)))

    def mask(self, rule):
        """True (in force over the whole tick), False (never), or a function
        of the forecast time tau."""
        if rule.id not in self.rt.timed:
            return True
        key = id(rule)
        m = self._cache.get(key)
        if m is None:
            if self.const:
                m = self.on(rule, self.when)
            else:
                plain = self._plain_key(rule)
                memo: dict = ({} if plain is None
                              else self._shared.setdefault(plain, {}))

                def m(tau, _rule=rule, _memo=memo):
                    k = round(tau, 9)
                    v = _memo.get(k)
                    if v is None:
                        v = _memo[k] = self.in_force(_rule, tau)
                    return v
            self._cache[key] = m
        return m

    def _plain_key(self, rule):
        """The weekly window of a rule whose in-force state is its own window
        and nothing else (no switch or swap targets it, and it is neither):
        `on` then reads only `valid_time.recurrence`, so two such rules with
        the same window are in force at exactly the same instants. None for
        every other rule."""
        rt = self.rt
        if (isinstance(rule, (TimeWindowSwitch, CorridorSwap))
                or rule.id in rt.switches_on or rule.id in rt.swaps_on):
            return None
        vt = rule.valid_time
        if vt is None or vt.recurrence is None:
            return None
        r = vt.recurrence
        return (r.start_time, r.end_time, tuple(r.days))

    def anywhere(self, rule) -> bool:
        m = self.mask(rule)
        if m is True or m is False:
            return m
        return any(m(t) for t in self.grid)

    def first(self, rule) -> float | None:
        """The earliest forecast time at which `rule` is in force, or None."""
        m = self.mask(rule)
        if m is True:
            return 0.0
        if m is False:
            return None
        return next((t for t in self.grid if m(t)), None)


class _Tick:
    """One monitor tick's view: the runtime it pinned, the schedule, the
    monitor clock, and the rule ids the repair chain is told to leave alone
    (monitor_only rules, and soft rules it has given up)."""
    __slots__ = ("rt", "sched", "t", "skip", "_lists", "_moving", "_swept", "unsafe",
                 "_t_first", "given_up", "chain_runs", "ledger", "ahead", "origin",
                 "committed", "pipe_t", "opened_mps")

    def __init__(self, rt: _Runtime, sched: _Schedule, t: float, t_first: float | None):
        self.rt, self.sched, self.t, self._t_first = rt, sched, t, t_first
        # A LOOK-AHEAD view (Shield.looking_ahead): how many seconds after the
        # monitor tick `origin` this view answers for - its clock, its moving
        # zones' ages and its subject all advanced by that much. 0.0 and None
        # on every real tick, where nothing reads them differently.
        self.ahead = 0.0
        self.origin: "_Tick | None" = None
        self.skip: frozenset = frozenset()
        # The soft rules _relax gave up on this tick: the repair chain skipped
        # them, so a soft speed cap among them clamped nothing (_applied_caps).
        self.given_up: frozenset = frozenset()
        self._lists: dict = {}
        self._moving: dict = {}
        self._swept: dict = {}
        self.unsafe = None
        # How many times the repair stack ran on this tick (1 + one per soft-
        # rule relaxation level tried); the audit record's `chain_runs`. Each
        # run's passes, in order: [(run, passes)], so a run that failed and
        # was retried with soft rules given up stays on the record.
        self.chain_runs = 0
        self.ledger: list = []
        # Shield._committed's answer per measured state on this tick (the
        # stop point and the rules its stop path breaks); empty, and never
        # read, while no state carries a velocity.
        self.committed: dict = {}
        # The pipe clock's reading for this tick (Shield._pipe_now), latched
        # at its first use so every forecast of the tick reads one pipe.
        self.pipe_t: float | None = None
        # How fast the escape from a zone whose window opened on the aircraft
        # leaves on the chain run being made (Shield._repair_geofence): None,
        # the speed cap; a number, that speed; _IN_FORCE_FLOOR, the in-force
        # floor a zone the aircraft entered by itself gets. Set only by
        # Shield._floor_the_escape, for one run of its retries
        # (`_opened_retries`), and None again after it.
        self.opened_mps: "float | str | None" = None

    def rules(self, name: str):
        key = (name, self.skip)
        v = self._lists.get(key)
        if v is None:
            src = getattr(self.rt, name)
            if not self.rt.timed and not self.skip:
                v = src
            else:
                v = tuple(r for r in src if r.id not in self.skip
                          and self.sched.anywhere(r))
            self._lists[key] = v
        return v

    def current(self, rec: FenceRecord) -> FenceRecord:
        """A moving zone's record at this tick's time (cached per tick)."""
        r = self._moving.get(rec.index)
        if r is None:
            r = compile_fence(rec.index, rec.rule, ring=self._ring(rec.rule),
                              multiscale=False)
            self._moving[rec.index] = r
        return r

    def _age(self, rule) -> float:
        a = self.rt.anchors.get(rule.id)
        if a is None:
            a = self._t_first if self._t_first is not None else self.t
        return max(0.0, self.t - a)

    def _ring(self, rule):
        return rule.ring_at(self._age(rule))

    def motion(self, rule):
        """(ux, uy, omega_rad_s, cx, cy) of a moving zone at this tick: its
        velocity, turn rate and current rotation centre; None if it is static."""
        if rule.id not in self.rt.moving_ids:
            return None
        ring = self._ring(rule)
        cx = sum(p[0] for p in ring) / len(ring)
        cy = sum(p[1] for p in ring) / len(ring)
        m = rule.motion
        return (m.vx_mps, m.vy_mps, math.radians(m.yaw_rate_dps), cx, cy)

    def swept(self, rec: FenceRecord, horizon: float) -> tuple:
        """Bounding box of every place the moving zone `rec` (its record at
        this tick) can occupy over the next `horizon` seconds: its margin ring
        translated along its velocity and, if it turns, the disc that ring
        sweeps about the rotation centre (`_to_zone_frame` rotates about the
        same point). A pose outside this box is outside the zone at every
        instant of the horizon, so skipping a zone whose box misses the
        forecast's box changes no answer (cached per tick)."""
        key = (rec.index, horizon)
        b = self._swept.get(key)
        if b is None:
            ux, uy, w, cx, cy = self.motion(rec.rule)
            minx, miny, maxx, maxy = rec.bounds
            if w != 0.0:
                r = max(math.hypot(x - cx, y - cy) for x, y in rec.buffered.exterior.coords)
                minx, miny, maxx, maxy = cx - r, cy - r, cx + r, cy + r
            dx, dy = ux * horizon, uy * horizon
            b = (minx + min(0.0, dx), miny + min(0.0, dy),
                 maxx + max(0.0, dx), maxy + max(0.0, dy))
            self._swept[key] = b
        return b

    def fences(self, ir: PolicyIR) -> list[FenceRecord]:
        if not self.rt.timed and not self.skip and not self.rt.moving_ids:
            return ir.fences
        out = []
        for r in ir.fences:
            if r.rule.id in self.skip or not self.sched.anywhere(r.rule):
                continue
            out.append(self.current(r) if r.moving else r)
        return out

    def anchors(self, ir: PolicyIR) -> list[FenceRecord]:
        """`fences`, but a moving zone is left as its ANCHOR record: nothing is
        materialised. The hot paths materialise a moving zone only once its
        anchor box (`anchor_box`) meets the forecast's."""
        if not self.rt.timed and not self.skip:
            return ir.fences
        return [r for r in ir.fences
                if r.rule.id not in self.skip and self.sched.anywhere(r.rule)]

    def anchor_box(self, rec: FenceRecord, horizon: float) -> tuple:
        """ir.anchor_reach at this tick: a superset of `swept` on the zone's
        materialised record, from its anchor geometry (cached per tick)."""
        key = ("anchor", rec.index, horizon)
        b = self._swept.get(key)
        if b is None:
            b = self._swept[key] = anchor_reach(rec, self._age(rec.rule), horizon)
        return b

    def moving_near(self, ir: PolicyIR, box: tuple, horizon: float) -> set | None:
        """The indices of the MOVING zones whose reach over `horizon` from
        this tick (`anchor_box`, ir.anchor_reach) meets `box`, from an R-tree.

        The grant's geometry queries "hit the IR's R-tree spatial index"
        (Safety Shield page). The IR's STRtree holds the static zones: a
        moving zone has no fixed envelope, so until 2026-11-03 it was left out
        of the tree, returned by every query, and tested one by one. Here the
        moving zones are RE-INDEXED PER TICK: the IR's moving-zone R-tree
        (PolicyIR.moving_index), an STRtree over each one's reach box from its
        anchor geometry, built once per tick and horizon on the first query
        (no polygon is built for it), and queried with the forecast's box.
        The query box is padded by _INDEX_PAD_M, so the tree
        returns a superset of the zones whose box meets it; the caller still
        asks the exact question (Shield._reach), so the zones judged are the
        ones the one-by-one test judged. None for a non-finite `box` (every
        moving zone is then looked at, as the static query does)."""
        if not all(math.isfinite(v) for v in box):
            return None
        key = ("moving_tree", id(ir), horizon)
        index = self._swept.get(key)
        if index is None:
            # The IR's moving-zone R-tree at this tick (ir.PolicyIR.
            # moving_index), each zone aged as `anchor_box` ages it.
            index = self._swept[key] = ir.moving_index(
                lambda r: self._age(r.rule), horizon)
        p = _INDEX_PAD_M
        return index.query(box[0] - p, box[1] - p, box[2] + p, box[3] + p)


def _to_zone_frame(p: State, tau: float, mv) -> tuple[float, float]:
    """A forecast pose at time tau, in the frame of a moving zone at the tick's
    time: undo the zone's translation and rotation over tau, so the pose can be
    tested against the zone's CURRENT polygon. Exact for a rigid motion
    (buffering commutes with it)."""
    ux, uy, w, cx, cy = mv
    rx, ry = p.x - (cx + ux * tau), p.y - (cy + uy * tau)
    th = w * tau
    c, s = math.cos(th), math.sin(th)
    return cx + rx * c + ry * s, cy - rx * s + ry * c


class Shield:
    def __init__(self, policy: Policy, lookahead_s: float = LOOKAHEAD_S,
                 dt: float = LOOKAHEAD_DT_S, obstacle_map: dict | None = None,
                 now=None, escalation: "_fsm.FSMConfig | bool | None" = True,
                 theta_horizon_s: float | None = _fsm.DEFAULT_THETA_HORIZON_S, *,
                 repair_mode: str = "chain", path_repair: bool = False,
                 segment_check: bool = False, repair_order: tuple | None = None,
                 hold_points: bool = True, land_site_ok=None,
                 velocity_response: "_response.VelocityResponse | None" = _response.ARDUCOPTER_GUIDED,
                 pipe_clock=None, recovery_law: str = DEFAULT_RECOVERY_LAW,
                 road_map: dict | None = None, ground=None, require_maps: bool = True,
                 dr_drift_mps: float = DR_DRIFT_MPS, arc_forecast: bool = False):
        """obstacle_map: None, or {"occ": uint8 NxN (1 = blocked), "res": float,
        "ox": float, "oy": float} — the same occupancy grid the global planner
        uses. A policy YAML cannot carry an 80x80 grid, so obstacle_clearance
        rules get their world here. With obstacle_map=None those rules are
        inert and the Shield behaves exactly as before.

        road_map: the same format, 1 = road (the street mask,
        demo/build_street_mask.py): the world of a distance_envelope over
        `roads` (since 2026-11-03, WP1-07), measured exactly as the clearance
        rule measures buildings. A distance_envelope over buildings or roads
        is refused at construction when its map is not given - unlike
        obstacle_clearance, it is never inert. `require_maps=False` builds
        one anyway, the rule then inert: only for a Shield that is asked
        about the clock and not flown (the compiler's in-force queries).

        ground: the ground-elevation source of an MSL rule (guardrail/
        terrain.py, X-11): None refuses every rule with altitude_ref MSL, as
        before 2026-11-03; a FlatGround (a stated elevation, per rail) lets
        the Shield enforce an MSL band as the AGL band it is over that ground.

        now: a zero-argument callable returning the current datetime, used only
        to evaluate `valid_time` windows. INJECTED rather than called inline so
        that a scheduled rule is testable at all - a check that reads the wall
        clock itself can only be tested at the hour the suite happens to run.
        None means "no clock", and with no clock every rule is in force: see
        `ConstraintBase.active_at`, where absent always means active. A flight
        rail passes `now=datetime.now` (or its simulation clock). The clock is
        read ONCE per tick.

        lookahead_s / dt: the forecast, by default the grant's 5 s at 0.1 s.

        escalation: True (the grant's defaults, fsm.FSMConfig()), an
        fsm.FSMConfig, or False / None for no escalation FSM. With an FSM every
        filter() tick feeds it; see ShieldDecision.

        theta_horizon_s: what theta is measured on. None (the default,
        fsm.DEFAULT_THETA_HORIZON_S) applies it to each repair's own
        `magnitude_m`: the forecast depth it corrected, the grant's p5 audit
        field and the PI reference's metric. fsm.THETA_HORIZON_S (0.1 s)
        selects the one-period reading, the setpoint change |dv| over one
        control period (p4's "||action_repaired - action_original||" read
        literally); it is not the default because under the shipped speed
        caps it can never exceed theta (docs/DESIGN-escalation-fsm.md,
        "Theta: two readings, one default"). Another number measures |dv|
        over that horizon.

        repair_mode / path_repair / segment_check: the repair library's modes
        (guardrail/repair.py), all off by default, where the decisions are
        the chain's as they have always been. "first_feasible" is the
        grant's selection (the first operator whose action clears the
        earliest violation within theta wins); `path_repair` adds the grant's
        PathRepair (a body-frame detour waypoint) after the lateral
        projection; `segment_check` also judges the straight segment between
        each pair of consecutive forecast poses against every zone's margin
        ring and every corridor's boundary, so a zone thinner than one
        forecast step is not stepped over. `repair_operators` is this
        Shield's copy of the operator registry.

        repair_order: operator names (registry keys) that replace the
        iterated loop's stack (repair.CHAIN_LOOP) in chain mode, or the
        priority order (repair.FIRST_FEASIBLE) in first-feasible mode - how a
        registered operator of a new name is put to work. None (the default)
        keeps the stacks above. A name the registry does not hold is refused
        here.

        hold_points: where the escalation FSM's Loiter holds and its Land
        lands (since 2026-10-19, guardrail/holdpoint.py). True (the default):
        where the aircraft is when that is legal - every hold that was legal
        before is unchanged - and otherwise the nearest point the Shield's own
        rules find legal from arrival through the lookahead (Loiter) or over
        the whole descent (Land), reached without breaking another rule.
        False: always where the aircraft is, the behaviour before that date,
        still recorded with whether it was legal (for comparisons).
        land_site_ok: optional `(x, y) -> bool`, a map-based landing filter
        (the city demo's demo/landing.py) applied on top of the policy's own
        test; None, the policy alone.

        velocity_response: how the aircraft answers a velocity setpoint
        (guardrail/response.py), used ONLY for a State that carries a
        measured `velocity`: each forecast pose then starts from that
        velocity and converges to the commanded one (`_predict`; the
        commanded forecast is judged beside it, so the velocity only adds), a rule the
        aircraft's own momentum already carries it into is judged from where
        a stop would end (`_committed`), and the clearance rule asks whether
        the aircraft can still stop after flying the action for one control
        period. A State without a velocity - every caller before 2026-10-19
        and every rail with the flag off - is forecast exactly as before.
        The default is the response measured on ArduCopter GUIDED
        (ARDUCOPTER_GUIDED, tools/fit_velocity_response.py); None ignores a
        measured velocity altogether. docs/DESIGN-own-velocity-forecast.md.

        pipe_clock: a zero-argument callable returning the rail's REAL time in
        seconds (monotonic), the clock the setpoints already sent are aged on
        (`note_flown`, `pipe`). None (the default) ages them on the monitor
        clock, filter()'s `t` - right for a caller that passes the real time
        as `t`, and what every caller did before 2026-10-20. A rail that lets
        filter() count its ticks at the nominal 0.1 s while its real ticks
        are longer must pass it (2026-10-20 review): the pas_ap rail's ticks
        are 0.109 s median in every recorded log and up to 0.73 s (14 Oct), so
        on the monitor clock a setpoint is aged ~9 % too young per tick and a
        delayed tick re-applies a setpoint already absorbed into the measured
        velocity. Read only for a state with a measured velocity: the rule
        clock, the sliding window, N-in-T and every decision without a
        velocity are untouched by it.

        recovery_law: how fast the Shield recovers from a position that is
        already illegal (RECOVERY_LAWS has the three laws and their dates).
        "cap_all" (the default since 2026-11-06): every recovery at its cap -
        the altitude band at the smallest kinematic climb cap, held to the
        band's far edge (_recovery_vz), a static zone at the speed cap judged
        as flown (_cap_zones), a stand-off ring at the speed cap, a corridor
        at the fastest inward speed whose lookahead endpoint stays short of
        the far wall. "cap" (2026-10-29 to 2026-11-06): the altitude band at
        its cap, the lateral recoveries on their old floors (2.0 m/s out of a
        zone; one lookahead, at least 0.5 m/s, back into a ring or corridor).
        "lookahead" (before 2026-10-29): the band in one lookahead too, an
        exponential with time constant L. The older two are kept so a bundle
        flown with them re-flies bit for bit and so the laws can be compared
        (WP3-17).

        dr_drift_mps: the dead reckoning's drift allowance (m/s) while the
        position is not a valid fix (State.fix_ok False): how fast the
        Shield lets the aircraft's true position wander from the estimate it
        dead-reckons on its own streamed commands. Read only for such a state
        (`_position_verdict`, the FSM's X8), recorded in every X8 reason and
        by the harness per episode. DR_DRIFT_MPS says why its default.

        arc_forecast: the default of `filter_body`'s `arc` (since
        2026-11-17, WP3-04 / ARCH-04; off by default, so every rail flies as
        before until it asks): a BODY-frame action with a non-zero yaw rate
        is forecast along the arc it flies - its velocity held in the body
        frame while the heading turns at the yaw rate (`_arc_offset`) -
        instead of along its world direction at t = 0. `filter()` (a
        world-frame action, flown as a LOCAL_NED velocity that does not
        turn with the heading) is never arced."""
        if recovery_law not in RECOVERY_LAWS:
            raise ValueError(f"recovery_law must be one of {RECOVERY_LAWS}, "
                             f"got {recovery_law!r}")
        self.recovery_law = recovery_law
        if not (isinstance(dr_drift_mps, (int, float)) and not isinstance(dr_drift_mps, bool)
                and math.isfinite(dr_drift_mps) and dr_drift_mps >= 0):
            raise ValueError(f"dr_drift_mps must be a finite number >= 0, got {dr_drift_mps!r}")
        self.dr_drift_mps = float(dr_drift_mps)
        self.arc_forecast = bool(arc_forecast)
        if repair_mode not in _repair.REPAIR_MODES:
            raise ValueError(f"repair_mode must be one of {_repair.REPAIR_MODES}, "
                             f"got {repair_mode!r}")
        self.repair_mode = repair_mode
        self.path_repair = bool(path_repair)
        self.segment_check = bool(segment_check)
        self.repair_operators: dict = dict(_repair.REGISTRY)
        if repair_order is not None:
            repair_order = tuple(repair_order)
            unknown = [n for n in repair_order if n not in self.repair_operators]
            if not repair_order:
                raise ValueError("repair_order is empty: no operator would run")
            if unknown:
                raise ValueError(f"repair_order names unregistered operator(s) {unknown}; "
                                 f"registered: {sorted(self.repair_operators)}")
        self.repair_order = repair_order
        self.hold_points = bool(hold_points)
        if land_site_ok is not None and not callable(land_site_ok):
            raise TypeError(f"land_site_ok must be callable (x, y) -> bool or None, "
                            f"got {land_site_ok!r}")
        self.land_site_ok = land_site_ok
        if velocity_response is not None and not isinstance(
                velocity_response, _response.VelocityResponse):
            raise TypeError(f"velocity_response must be a guardrail.response."
                            f"VelocityResponse or None, got {velocity_response!r}")
        self.velocity_response = velocity_response
        if pipe_clock is not None and not callable(pipe_clock):
            raise TypeError(f"pipe_clock must be a zero-argument callable or None, "
                            f"got {pipe_clock!r}")
        self._pipe_clock = pipe_clock
        # States whose measured velocity was not finite: ignored (the
        # commanded forecast flew that tick), and counted, as set_subject
        # counts a non-finite subject. A NaN velocity would otherwise make
        # every pose NaN, and NaN loses every comparison - the forecast
        # would fail OPEN (see _sanitise).
        self.n_velocity_nonfinite = 0
        # filter() ticks whose forecast started from a measured velocity.
        self.n_velocity_ticks = 0
        # filter() ticks whose measured velocity was SET ASIDE because the
        # position already broke a hard, enforced rule (`_decision_state`).
        self.n_velocity_set_aside = 0
        # Setpoints the rail said it sent (note_flown with an action): a rail
        # that gives a velocity and never calls note_flown leaves the pipe
        # empty, and the forecast coasts the measured velocity through the
        # reaction time instead of flying what was sent.
        self.n_noted_setpoints = 0
        # What the rail actually SENT the autopilot, tick by tick (note_flown):
        # (pipe-clock time, vx, vy), vx None for a tick it sent nothing. The
        # setpoints of the last reaction time are still being flown when this
        # tick's command starts to act (`pipe`); without this record the
        # forecast assumes the aircraft keeps its measured velocity until
        # then. Read only for a state with a measured velocity.
        self._flown: deque = deque(maxlen=64)
        self._tubes: dict = {}
        # The ground under an MSL rule (X-11, 2026-11-03); None = no MSL rule.
        self._ground = ground
        why = self.unenforceable(policy, ground)
        if why:
            raise ValueError("the Shield does not enforce every rule of "
                             f"{policy.policy_id!r}: " + "; ".join(why))
        self.policy = policy
        self.lookahead_s = lookahead_s
        self.dt = dt
        self._now = now
        # Events are applied under this lock; ticks read the runtime pinned at
        # their start, so they never wait on it.
        self._lock = threading.RLock()
        self._local = threading.local()
        # Compile the fences once (grant: pre-compute at ingest, never rebuild
        # per tick): authored polygon, margin ring and an STRtree over the
        # rings. See guardrail/ir.py. The comment that stood here said the
        # same thing about the polygons, while geometry.point_in_fence went on
        # rebuilding the margin ring on every point test - 384 buffer() calls
        # in one 50-rule check, most of its 14-33 ms.
        dyn = {c.id: None for c in policy.constraints if isinstance(c, DynamicNFZ)}
        self._anchors: dict = dyn
        self._rt = _build_runtime(policy, None, dyn, ground)
        # The sliding-window buffer: the last HISTORY_LEN ticks' actions and
        # predicted trajectories, oldest first, which the checker reads (see
        # SlidingWindow and `history`).
        self._buffer = SlidingWindow(HISTORY_LEN)
        # Monitor clock (seconds): the `t` of each filter() tick, or the tick
        # count x MONITOR_PERIOD_S when the caller passes none.
        self._ticks = 0
        self._t_last: float | None = None
        self._t_first: float | None = None
        # Set by reset_episode(): (last tick's t, {moving zone id: its age}),
        # so a moving zone continues from where the episode left it.
        self._carry: tuple[float, dict] | None = None
        # Every event applied, in order (op, rule, generation, hash, t).
        self.events: list[dict] = []
        # Where the thing we are following is, in world coordinates, plus what
        # kind of thing it is. Supplied by the perception stack once per tick via
        # set_subject(); None means "not currently tracking anything", and every
        # SubjectStandoff rule is inert until it is set.
        self._subject: tuple[float, float] | None = None
        self._subject_class: str | None = None
        # The last subject set_subject() refused as non-finite (x, y, class),
        # until a finite one or None replaces it; and how many it has refused.
        self._subject_rejected: tuple | None = None
        self.n_subject_nonfinite = 0
        # The subject's ground velocity (vx, vy) in the Action4D frame, m/s,
        # when the perception stack supplies one (set_subject(velocity=...)):
        # the stand-off check and its repairs then forecast the subject at
        # constant velocity. None holds it still, as before 2026-10-19.
        # Non-finite velocities refused (the position is kept), counted.
        self._subject_v: tuple[float, float] | None = None
        self.n_subject_velocity_nonfinite = 0
        # Ticks where the escape floor (_floor_the_escape) changed a legal
        # action and was NOT flown - the raw action flew instead - because
        # its result failed the full re-check or carried a repair theta
        # governs. Each such decision's trace says why (`floor_refused`).
        self.n_floor_refused = 0
        # The zones whose time window opened on the aircraft sooner than it
        # could leave (_opening_imposed), remembered from the filter() tick
        # that saw it until the aircraft is out of the zone's ring: their
        # escape flies at the cap (GeofenceEscape, _repair_geofence), before
        # the opening and after it. Kept by filter() only
        # (_remember_openings), cleared by reset_episode().
        self._opened_on: set[str] = set()
        # Position knowledge (State.fix_ok, the FSM's X8; 2026-11-06): the last
        # position given as a valid fix and its time, the dead reckoning
        # since it on what this Shield streamed (None while the fix is
        # valid), and what it streamed on the last tick (t, vx, vy).
        self._pk_good: tuple[float, float, float] | None = None
        self._pk: dict | None = None
        self._pk_cmd: tuple[float, float, float] | None = None

        if escalation is True:
            cfg = _fsm.FSMConfig()
        elif escalation is False or escalation is None:
            cfg = None
        elif isinstance(escalation, _fsm.FSMConfig):
            cfg = escalation
        else:
            raise TypeError(f"escalation must be True, False, None or an FSMConfig, "
                            f"got {escalation!r}")
        self.fsm = _fsm.EscalationFSM(cfg) if cfg is not None else None
        if theta_horizon_s is not None:
            _fsm._check_horizon(theta_horizon_s)
        self.theta_horizon_s = theta_horizon_s
        self._fsm_fault: str | None = None
        # The kinematic caps the last filter() clamped against (applied_caps).
        self._last_caps: "_fsm.KinematicCaps | None" = None

        # Distance field: built ONCE here, never per tick (same rule as the
        # fence polygons). Skipped entirely when nothing needs it.
        self._dist = None
        self._res = self._ox = self._oy = 0.0
        self._dist_lip = 1.0
        if obstacle_map is not None and self._rt.clear:
            self._res = float(obstacle_map["res"])
            self._ox = float(obstacle_map["ox"])
            self._oy = float(obstacle_map["oy"])
            self._dist = _build_signed_distance_field(obstacle_map["occ"], self._res)
            self._dist_lip = _field_lipschitz(self._dist, self._res)
        # The road field of a distance_envelope over `roads` (WP1-07), built
        # the same way from road_map (1 = road): distance to the nearest road.
        self._road = None
        if road_map is not None and any(_field_name(c) == "roads" for c in self._rt.clear):
            rres = float(road_map["res"])
            rd = _build_signed_distance_field(road_map["occ"], rres)
            self._road = (rd, rres, float(road_map["ox"]), float(road_map["oy"]),
                          _field_lipschitz(rd, rres))
        # A distance_envelope is never inert (models.DistanceEnvelope): one
        # over buildings or roads without its map is refused here.
        blind = [f"{c.id}: a distance_envelope over {_field_name(c)} needs "
                 f"{'road_map' if _field_name(c) == 'roads' else 'obstacle_map'} "
                 f"(the Shield has no other source of {_field_name(c)})"
                 for c in self._rt.clear
                 if isinstance(c, EnvelopeClearance) and self._field_for(c) is None]
        if blind and require_maps:
            raise ValueError("the Shield cannot enforce every rule of "
                             f"{policy.policy_id!r}: " + "; ".join(blind))

    @staticmethod
    def unenforceable(policy: Policy, ground=None) -> list[str]:
        """Rules of `policy` this Shield would NOT enforce, with the reason.
        A Shield is never built over them: a Policy object made directly (not
        through a flight loader) reaches here without the loaders' gate.
        An MSL height is enforceable since 2026-11-03 with a ground-elevation
        source (`ground`, guardrail/terrain.py) and refused without one."""
        out = []
        for c in policy.constraints:
            if c.type not in ENFORCED_TYPES:
                out.append(f"{c.id}: {c.type} is declarable in the DSL but the "
                           f"Safety Shield does not enforce it")
            if getattr(c, "altitude_ref", None) == "MSL":
                why = _terrain.ground_problem(ground)
                if why:
                    out.append(f"{c.id}: altitude_ref MSL - {why}")
        return out

    def _field_for(self, c):
        """The distance field clearance rule `c` is measured on: (field, res,
        ox, oy, lipschitz) - the obstacle map for buildings (every
        obstacle_clearance), the road map for a distance_envelope over roads -
        or None when that map was not given."""
        if _field_name(c) == "roads":
            return self._road
        if self._dist is None:
            return None
        return (self._dist, self._res, self._ox, self._oy, self._dist_lip)

    @property
    def ground(self):
        """The ground-elevation source MSL rules are enforced over (None:
        no MSL rule can be enforced)."""
        return self._ground

    # ---------------- which rules are in force right now ---------------- #
    #
    # Exposed as PROPERTIES rather than as a filter at each call site. The rule
    # lists are read from more than twenty places - checks, five repair
    # operators, the speed-cap lookups, the rescue search - and gating only the
    # checks would produce the worst possible split: a rule that raises no
    # violation while its repair operator still bends the action away from it.
    # Making the lists themselves time-aware means every consumer, present and
    # future, sees the same set.
    #
    # A list holds every rule in force SOMEWHERE in the tick's lookahead (so a
    # repair also respects a fence that switches on in three seconds); the
    # checks then judge each forecast pose against the rules in force at that
    # pose's own time (`_mask`). During the repair chain the lists also leave
    # out the rules the chain must not act for (`_Tick.skip`). With no clock,
    # no switch, no swap and no skip they are the stored tuples, so the common
    # path allocates nothing.

    def _tick_ctx(self) -> "_Tick | None":
        return getattr(self._local, "tick", None)

    def _grid(self) -> tuple:
        """The forecast's pose times, exactly as `_predict` steps them (cached
        per lookahead / dt, which a caller may change after construction)."""
        key = (self.lookahead_s, self.dt)
        g = getattr(self, "_grid_cache", None)
        if g is None or g[0] != key:
            out, t, h = [], 0.0, self.lookahead_s
            while True:
                out.append(t)
                if t >= h - 1e-9:
                    break
                t = min(t + self.dt, h)
            g = self._grid_cache = (key, tuple(out))
        return g[1]

    def _new_tick(self, t: float | None, rt: _Runtime | None = None) -> _Tick:
        rt = rt if rt is not None else self._rt
        when = self._now() if (self._now is not None and rt.timed) else None
        if self._now is not None and not rt.timed:
            when = None                         # nothing to read it for
        sched = _Schedule(rt, when, self.dt / 2.0, self._grid())
        if t is None:
            # A read between ticks is at the last tick's time; between
            # reset_episode() and the next tick, at the reset's (every moving
            # zone stays where the last episode left it).
            t = (self._t_last if self._t_last is not None
                 else self._carry[0] if self._carry is not None else 0.0)
        return _Tick(rt, sched, t, self._t_first)

    class _Scope:
        __slots__ = ("sh", "tick", "prev")

        def __init__(self, sh, tick):
            self.sh, self.tick, self.prev = sh, tick, None

        def __enter__(self):
            self.prev = getattr(self.sh._local, "tick", None)
            self.sh._local.tick = self.tick
            return self.tick

        def __exit__(self, *exc):
            self.sh._local.tick = self.prev
            return False

    def _scope(self, tick: _Tick):
        return Shield._Scope(self, tick)

    def _needs_tick(self) -> bool:
        rt = self._rt
        return bool(rt.timed or rt.moving_ids)

    def _list(self, name: str):
        tick = self._tick_ctx()
        if tick is None:
            if not self._needs_tick():
                return getattr(self._rt, name)
            tick = self._new_tick(None)
        return tick.rules(name)

    def _mask(self, rule):
        tick = self._tick_ctx()
        if tick is None:
            if rule.id not in self._rt.timed:
                return True
            tick = self._new_tick(None)
        return tick.sched.mask(rule)

    def _motion_of(self, rule):
        tick = self._tick_ctx()
        if tick is None:
            if rule.id not in self._rt.moving_ids:
                return None
            tick = self._new_tick(None)
        return tick.motion(rule)

    @property
    def _kins(self):
        return self._list("kins")

    @property
    def _alts(self):
        return self._list("alts")

    @property
    def _clear(self):
        return self._list("clear")

    @property
    def _standoffs(self):
        return self._list("standoffs")

    @property
    def _corridors(self):
        return self._list("corridors")

    @property
    def _fences(self) -> list:
        """Same gating, but the entries are (rule, prebuilt polygon) pairs.
        Kept for readers of the old shape; the Shield itself walks
        `_fence_records`, which also carries the margin ring."""
        return [(r.rule, r.polygon) for r in self._fence_records(self._ir)]

    def _fence_records(self, ir: PolicyIR) -> list[FenceRecord]:
        """The compiled fences in force somewhere in this tick's lookahead, in
        policy order, a moving zone at its current position. Reads the clock
        once per tick (once per call outside a tick)."""
        tick = self._tick_ctx()
        if tick is None:
            if not self._needs_tick():
                return ir.fences
            tick = self._new_tick(None)
        return tick.fences(ir)

    def _fence_anchors(self, ir: PolicyIR) -> tuple[list[FenceRecord], "_Tick | None"]:
        """`_fence_records` without materialising the moving zones (each is
        its anchor record), and the tick to materialise them in (`_reach`).
        With no moving zone the two lists are the same, and this one is read
        through `_fence_records`, as it is whenever a subclass overrides that
        method (tests/test_profile_shield_tick.py blinds a Shield that way)."""
        tick = self._tick_ctx()
        rt = tick.rt if tick is not None else self._rt
        if not rt.moving_ids or type(self)._fence_records is not Shield._fence_records:
            return self._fence_records(ir), tick
        if tick is None:
            tick = self._new_tick(None)
        return tick.anchors(ir), tick

    def _reach(self, rec: FenceRecord, tick: "_Tick | None", box: tuple,
               horizon: float) -> FenceRecord | None:
        """The record to judge for `rec` against a forecast in `box`, or None
        when the zone cannot meet it. A static zone is itself. A moving zone
        is asked `_may_reach` twice: first as its ANCHOR record, whose box
        (ir.anchor_reach) needs no polygon, and only if that box meets `box`
        is the zone built at the tick's time and asked again, by its own
        swept box, exactly as before. The anchor box contains the swept box,
        so the zones judged are the same ones; what is saved is building the
        polygon of every zone that cannot be reached. Building all 48 moving
        zones of the 50-rule load was 70 % of a `_check` (2026-10-12
        profile). A subclass whose `_may_reach` says True judges every zone
        at every pose, as it did before."""
        if not rec.moving or tick is None:
            return rec                      # no tick: a static view, as before
        if not self._may_reach(rec, box, horizon):
            return None
        cur = tick.current(rec)
        return cur if self._may_reach(cur, box, horizon) else None

    @staticmethod
    def _fence_candidates(ir: PolicyIR, state: State, poses) -> set[int]:
        """Indices of the fences the forecast `poses` could touch.

        The query box is the bounding box of the current position and every
        forecast pose, so a fence left out cannot contain any point the fence
        rule will test - the index changes what is LOOKED AT, never what is
        found (guardrail/ir.py, "Exactness"). The current position is added
        explicitly because the already-inside test reads `state.x` itself, not
        a pose: today the first pose is t = 0 and the two coincide, but a
        forecast that ever starts one step ahead would otherwise drop the very
        fence the vehicle is sitting in. A NaN anywhere (an infinite vx makes
        the t = 0 pose NaN) falls back to every fence inside the IR. A moving
        zone is always a candidate (ir.py)."""
        xs = [state.x] + [p.x for _, p in poses]
        ys = [state.y] + [p.y for _, p in poses]
        return {r.index for r in ir.fences_for_points(xs, ys)}

    @staticmethod
    def _forecast_box(state: State, poses) -> tuple:
        """(box, horizon): the bounding box of the current position and every
        forecast pose, and the forecast's last time. NaN anywhere gives an
        infinite box (every zone is then looked at, as in `_fence_candidates`)."""
        xs = [state.x] + [p.x for _, p in poses]
        ys = [state.y] + [p.y for _, p in poses]
        horizon = poses[-1][0] if poses else 0.0
        if not all(math.isfinite(v) for v in xs + ys):
            inf = float("inf")
            return (-inf, -inf, inf, inf), horizon
        return (min(xs), min(ys), max(xs), max(ys)), horizon

    def _moving_near(self, ir: PolicyIR, tick: "_Tick | None", box: tuple,
                     horizon: float) -> set | None:
        """The moving zones the forecast in `box` can meet within `horizon`,
        from this tick's R-tree of moving zones (`_Tick.moving_near`): a
        superset of the ones `_reach` keeps, which still decides. None - look
        at every moving zone, as before 2026-11-03 - without a tick or a
        moving zone, for a non-finite box, and for a subclass that answers
        reach its own way (`_reach` / `_may_reach` overridden: the identity
        tests' nulls judge every moving zone, and must keep doing so)."""
        if tick is None or not tick.rt.moving_ids:
            return None
        cls = type(self)
        if cls._reach is not Shield._reach or cls._may_reach is not Shield._may_reach:
            return None
        return tick.moving_near(ir, box, horizon)

    def _may_reach(self, rec: FenceRecord, box: tuple, horizon: float) -> bool:
        """Can the forecast in `box` meet `rec` within `horizon`? Always True
        for a static zone (the STRtree already answered); for a moving one,
        does its swept box (`_Tick.swept`) meet the forecast's box. A moving
        zone cannot go in the tree, and testing every one at every pose cost
        31 ms per check with 48 of them on the 50-rule load
        (tests/test_shield_events.py)."""
        if not rec.moving:
            return True
        tick = self._tick_ctx()
        if tick is None:
            return True
        # An anchor record (compiled from the authored vertices, with its
        # multi-scale ring) is bounded without building it; a record built
        # at the tick's time (no coarse ring) by its own swept box.
        sx0, sy0, sx1, sy1 = (tick.anchor_box(rec, horizon) if rec.coarse
                              else tick.swept(rec, horizon))
        bx0, by0, bx1, by1 = box
        return sx0 <= bx1 and sx1 >= bx0 and sy0 <= by1 and sy1 >= by0

    def forecast(self, state: State, action: Action4D, *,
                 pipe=None, arc: bool = False) -> list[tuple[float, State]]:
        """The predicted trajectory the fence and altitude rules judge: (t, pose)
        pairs over the lookahead. Public so the sliding window's "predicted
        trajectory" can be rebuilt for any entry - see TickRecord.

        For a state with a measured velocity this is the own-velocity
        forecast, one pose per grid time; the rules judge the commanded
        forecast from the same position beside it (`_predict`, "THE
        MEASURED VELOCITY ONLY ADDS"), which is forecast() of the state
        without its velocity. (`own_only` is passed only then: a subclass
        that overrides `_predict` with the old signature, as tests/test_ir.py
        does, still gets the old call.)

        `pipe`: the reaction segments to fly instead of the live ones
        (`Shield.pipe`) - a TickRecord's `pipe`, so the entry's forecast is
        rebuilt exactly whatever was sent since. Ignored without a measured
        velocity.

        `arc`: the ARC of `action` - a world-frame action that was a
        body-frame command, turned along the arc of its yaw rate
        (`_arc_rate`), one pose per grid time (the own-velocity one with a
        measured velocity, as above). A `filter_body(..., arc=True)` tick
        (TickRecord.arc) judged this AND the straight forecast (arc=False)
        beside it (`_predict`, THE ARC ONLY ADDS). False: the straight
        forecast."""
        prev_arc = getattr(self._local, "arc", False)
        self._local.arc = bool(arc)
        try:
            return self._forecast(state, action, pipe)
        finally:
            self._local.arc = prev_arc

    def _forecast(self, state: State, action: Action4D, pipe) -> list[tuple[float, State]]:
        kw = {"own_only": True} if self._velocity_of(state) is not None else {}
        measured = bool(kw)
        if self._arc_rate(action):
            kw["arc_path"] = "arc"           # passed only then (the old signature)
        if pipe is None or not measured:
            return [(t, State(x=p.x, y=p.y, up=p.up, yaw_deg=p.yaw_deg))
                    for t, p in self._predict(state, action, **kw)]
        prev = getattr(self._local, "pipe_override", None)
        self._local.pipe_override = tuple(pipe)
        try:
            # A scope of its own: the per-tick cache (_pipe_pre) must not
            # hand this forecast the live pipe's precomputation, nor keep
            # this one for the live tick.
            with self._scope(self._new_tick(None)):
                return [(t, State(x=p.x, y=p.y, up=p.up, yaw_deg=p.yaw_deg))
                        for t, p in self._predict(state, action, **kw)]
        finally:
            self._local.pipe_override = prev

    @property
    def history(self) -> tuple[TickRecord, ...]:
        """The sliding window: the last HISTORY_LEN ticks through `filter()`,
        oldest first, each with the action given, the predicted trajectory
        the checker judged it on, and the decision (TickRecord).

        The grant's node design puts this buffer in front of the checker, and
        since 2026-11-03 it is there: each tick's action and its predicted
        trajectory are admitted to the buffer first and the checker reads the
        trajectory from it (SlidingWindow). No rule in this DSL reads PAST
        actions, so the older entries are a record - for the HUD, an audit
        writer, a future oscillation rule - without a second copy that could
        disagree with it. Every return path of `filter()` closes its entry,
        the brake and rescue branches included."""
        return self._buffer.records()

    @property
    def applied_caps(self) -> "_fsm.KinematicCaps | None":
        """The speed and climb caps the last `filter()` clamped against
        (`_applied_caps`), or None when it applied none. A rail that runs its
        own FSM passes `caps=shield.applied_caps or False` to
        `fsm.tick_input_from_decision`, so theta's |dv| measure (when a rail
        selects it) starts
        from the action the chain really clamped, not from every cap the
        policy lists (one switched off mid-flight, or `monitor_only`)."""
        return self._last_caps

    @property
    def _ir(self) -> PolicyIR:
        tick = self._tick_ctx()
        return tick.rt.ir if tick is not None else self._rt.ir

    @property
    def ir(self) -> PolicyIR:
        """The compiled fence IR the monitor is querying (read-only view)."""
        return self._ir

    @property
    def rule_clock(self) -> RuleClock:
        """The rule clock of the rule set the monitor is enforcing (this
        generation's; read-only): the one implementation of "is this rule in
        force at that instant", for a reader that needs the answer at an
        instant of its own (guardrail/replay.zone_dwell reads each logged
        point's clock reading through it)."""
        tick = self._tick_ctx()
        return tick.rt.clock if tick is not None else self._rt.clock

    @property
    def mission_started(self) -> bool:
        """True from the first filter() tick: the grant's "mission start", after
        which every rule class but the three hot-applicable ones is locked."""
        return self._ticks > 0

    # ---------------- obstacle distance field ---------------- #

    def _distance_at(self, x: float, y: float, f=None) -> float:
        """Approx. SIGNED metres from world point (x, y) to the nearest mapped
        obstacle — negative inside a building footprint. inf when there is no
        map. Bilinear inside the grid; outside it, a safe lower bound built from
        the distance to the map edge (every obstacle lives inside the map, so
        being far off-map IS clear).

        `f`: another field (`_field_for`: the road map of a distance_envelope
        over roads, WP1-07), read the same way; None, the obstacle map."""
        if f is None:
            d, res, ox, oy = self._dist, self._res, self._ox, self._oy
        else:
            d, res, ox, oy = f[0], f[1], f[2], f[3]
        if d is None:
            return float("inf")
        n, m = d.shape
        fi = (x - ox) / res
        fj = (y - oy) / res
        ci = min(max(fi, 0.0), n - 1.0)
        cj = min(max(fj, 0.0), m - 1.0)
        off = math.hypot((fi - ci) * res, (fj - cj) * res)

        i0 = int(math.floor(ci)); i1 = min(i0 + 1, n - 1); ti = ci - i0
        j0 = int(math.floor(cj)); j1 = min(j0 + 1, m - 1); tj = cj - j0
        v = (float(d[i0, j0]) * (1 - ti) * (1 - tj)
             + float(d[i1, j0]) * ti * (1 - tj)
             + float(d[i0, j1]) * (1 - ti) * tj
             + float(d[i1, j1]) * ti * tj)
        if off > 0.0:
            # |d(q) - d(edge)| <= off (1-Lipschitz), so d(q) lies in
            # [v - off, v + off]. Within that band take the value of a map
            # whose border obstacles extend past the edge exactly as far as
            # they reach into it, and no further: off - res/2 once past a
            # built-on border (the cell's own half-width), v + off while still
            # inside its overhang. Continuous with the inside at off = 0, so
            # the half cell between the last cell centre and the map's edge
            # reads as the building it is part of.
            #
            # It used to apply only where the edge value was positive, and
            # return `v - off` otherwise - so a building on the map BORDER
            # became a wall reaching out to infinity, deeper the further one
            # flew. On CityLife, whose loops run 30-120 m past this 160 m
            # map, that read the entrance of a junction as 35 m inside a
            # building, and the clearance repair - its push sized from that
            # 35 m "depth", so at the 5 m/s cap - replaced a +3 m/s follow
            # with a 5 m/s reversal for up to 187 s (citylife_redcar_ground,
            # _carpolicy); at 12 m those reversals pitched the airframe
            # through its 14 m ceiling (_high). Off the map is UNKNOWN, not
            # solid; `off_map()` reports it, and the cure is a map that
            # covers the flight.
            #
            # The first fix, max(v - off, off), ended the infinite wall but
            # jumped from v (inside a border building) to +off at the last
            # cell centre, so the half cell of building beyond it read as
            # clear and a push "out" through it counted as receding (found in
            # review, 2026-09-24).
            return max(v - off, min(v + off, off - 0.5 * res))
        return v

    @property
    def has_obstacle_map(self) -> bool:
        """Is there a distance field at all? Without one ObstacleClearance
        is inert and `off_map()` is False everywhere - a caller counting
        off-map ticks must say so rather than report 0."""
        return self._dist is not None

    def off_map(self, x: float, y: float) -> bool:
        """True where (x, y) lies outside the obstacle map (more than half a
        cell past its edge). The clearance rule cannot see anything there, so
        a caller should report it rather than let it pass as clear."""
        d = self._dist
        if d is None:
            return False
        n, m = d.shape
        fi = (x - self._ox) / self._res
        fj = (y - self._oy) / self._res
        return not (-0.5 <= fi <= n - 0.5 and -0.5 <= fj <= m - 0.5)

    def _away_dir(self, x: float, y: float, probe_r: float,
                  f=None) -> tuple[float, float]:
        """Unit vector along the local gradient of the distance field, i.e.
        'the way AWAY from the nearest building'. (0, 0) when even the probe
        finds nothing to steer by — the caller then leaves the action alone and
        the P0 re-check decides. `f`: the field (_distance_at)."""
        if (self._dist if f is None else f[0]) is None:
            return 0.0, 0.0
        res = self._res if f is None else f[1]
        h = max(res * 0.5, 1e-3)
        gx = (self._distance_at(x + h, y, f) - self._distance_at(x - h, y, f)) / (2 * h)
        gy = (self._distance_at(x, y + h, f) - self._distance_at(x, y - h, f)) / (2 * h)
        norm = math.hypot(gx, gy)
        if norm > 1e-6:
            return gx / norm, gy / norm
        # Flat field (a symmetric ridge between two buildings, the exact centre
        # of a footprint): probe 8 compass directions and take the one that
        # gains the most distance, WIDENING the radius until something gains.
        # "No direction" must never fall through to a brake — a stationary
        # vehicle never recedes, so the violation would repeat forever.
        here = self._distance_at(x, y, f)
        r0 = max(probe_r, res)
        for mult in (1.0, 2.0, 4.0):
            r = r0 * mult
            best, best_d = (0.0, 0.0), here
            for k in range(8):
                ang = k * math.pi / 4.0
                ux, uy = math.cos(ang), math.sin(ang)
                dv = self._distance_at(x + ux * r, y + uy * r, f)
                if dv > best_d + 1e-9:
                    best, best_d = (ux, uy), dv
            if best != (0.0, 0.0):
                return best
        return 0.0, 0.0

    # ---------------- violation checking ---------------- #

    def _predict(self, state: State, action: Action4D,
                 horizon_s: float | None = None, *, own_only: bool = False,
                 arc_path: str | None = None):
        """The forecast (the grant Shield's 50-pose forecast at the default
        5 s / 0.1 s; a caller may pass a shorter one): the commanded velocity
        held from t = 0 for a State without a measured velocity, as it always
        was; for one with, the measured velocity through the response model
        AND the commanded forecast, two poses per grid time (the own-velocity
        branch below; `own_only` yields the first alone). `horizon_s`
        overrides the default lookahead — the clearance
        rule uses a much shorter, braking-distance-sized one (see
        `_clear_horizon_s`). The final sample always lands exactly ON the
        horizon, so a horizon that is not a whole number of `dt` still gets its
        endpoint checked.

        THE BUFFER IS THE CHECKER'S INPUT (2026-11-03, WP3-23). On the tick
        that admitted an action into the sliding-window buffer
        (SlidingWindow.admit, in filter()), the forecast of that action from
        that state is the buffer's trajectory: it is yielded from there, not
        computed again, so every rule that judges the action walks the
        poses the buffer holds. Any other forecast (a repair candidate, a
        stop, another horizon, the own-velocity path alone, a look-ahead
        view) is computed as before.

        THE ARC (2026-11-17, WP3-04 / ARCH-04). Inside `filter_body(...,
        arc=True)` (`_arc_rate`), an action with a non-zero yaw rate is a
        body-frame command whose velocity turns with the heading: its
        horizontal displacement along the ARC is `_arc_offset` instead of the
        straight `v * t`. THE ARC ONLY ADDS (review 2026-11-17), as the
        measured velocity does: each grid time yields the straight pose(s)
        the Shield always judged FIRST, then the arc's, so a rule is broken
        if EITHER breaks it, at the earlier time. The arc in place of the
        line held a yaw rate for the whole 5 s: a VLA whose yaw output
        alternates sign flies nearly straight while every tick forecast a
        tight circle, and a zone dead ahead was repaired 9-14 m later than on
        the line (the test with a yaw rate alternating each tick). `arc_path`
        "line" or "arc" yields one of the two paths alone (the segment test
        and the clearance rule walk each in order, `_segment_paths`,
        `_check_clearance`; `forecast(..., arc=True)` returns the arc). With
        the arc off, or a yaw rate under ARC_MIN_RATE, every pose is computed
        by the very expressions it always was (bit-identical:
        tests/test_body_arc_forecast.py)."""
        w0 = self._arc_rate(action)
        w = 0.0 if arc_path == "line" else w0
        # Which paths: the line always unless only the arc is asked for; the
        # arc whenever there is one.
        lines = not w or arc_path != "arc"
        if horizon_s is None and not own_only and (arc_path is None or not w0):
            held = self._buffer.trajectory_for(self._tick_ctx(), state, action)
            if held is not None:
                yield from held
                return
        horizon = self.lookahead_s if horizon_s is None else horizon_s
        t = 0.0
        # UNITS. State.yaw_deg is degrees; Action4D.yaw_rate is RADIANS per
        # second (models.py, docs/FINDING-the-contract-disagreed-with-itself-
        # about-yaw.md). Until 2026-10-19 the two were added raw, so every
        # forecast heading turned 57.3x too slowly: forecast() of a 30 dps
        # turn read 2.6 deg at t + 5 s instead of 150. No rule reads a pose's
        # heading, so no decision moved; forecast() is public, though, and is
        # how a TickRecord's predicted trajectory is rebuilt.
        yaw_dps = math.degrees(action.yaw_rate)
        # A _Pose, not a State: the four values are already floats (sums and
        # products of a validated State's and Action4D's), and validating
        # each of 51 poses again was a fifth of a 50-rule filter() (2026-10-12
        # profile). `forecast` still returns States.
        v = self._velocity_of(state)
        if v is not None:
            # OWN-VELOCITY FORECAST (2026-10-19; docs/DESIGN-own-velocity-
            # forecast.md). The pose starts from the MEASURED velocity. For
            # the response's reaction time the aircraft flies what was
            # already sent (`pipe`; with no record of it, it keeps its
            # velocity), then turns towards this action at the response's
            # acceleration limit (guardrail/response.py, exact). Horizontal
            # only: the climb rate stays the commanded one, as below - the
            # fit found the response model forecast altitude worse than that.
            # Same grid, same final sample exactly on the horizon.
            #
            # THE MEASURED VELOCITY ONLY ADDS (2026-10-20). Unless `own_only`,
            # each grid time yields TWO poses: the own-velocity one, then the
            # commanded one the Shield always judged. Every consumer of this
            # generator takes the first pose that breaks its rule (fences,
            # stand-off, corridor) or the extreme over all of them (a zone's
            # depth, the forecast box, the closest approach), so a rule is
            # broken if EITHER forecast breaks it, at the earlier time. The
            # pure own-velocity forecast was less cautious than the old one
            # whenever the aircraft was slower than its command: in the closed
            # loop (tools/own_velocity_closed_loop.py) it streamed 3.2 m/s
            # towards the 14 Oct building from 3.0-3.4 m off it, one tick at
            # a time, each followed by a stop, and the old KPI basis (the
            # emitted action re-checked on the commanded forecast) counted 6
            # P0 escapes there against the old Shield's 0. With both judged,
            # nothing the old Shield refused is emitted clean, and the momentum
            # it could not see is still seen. The clearance rule, whose test
            # reads its forecast in order, judges the two separately
            # (_check_clearance).
            #
            # COST (2026-10-20 review: the 50-rule monitor query had not been
            # timed in this mode). The poses are computed once per tick for
            # each (state, action, horizon, own_only) and replayed to every
            # rule that asks (altitude, each stand-off, the fences, each
            # corridor) - the same poses in the same order, so no answer moves.
            pre = self._pipe_pre(v)
            tick = self._tick_ctx()
            key = None
            if tick is not None:
                key = ("poses", state.x, state.y, state.up, state.yaw_deg, v,
                       action.vx, action.vy, action.vz_up, action.yaw_rate,
                       horizon, bool(own_only), id(pre), w, lines)
                hit = tick.committed.get(key)
                if hit is not None:
                    yield from hit
                    return
            lag, dxl, dyl, vxl, vyl, early = pre
            acc = self.velocity_response.accel_mps2
            cx, cy = action.vx, action.vy
            out = []
            while True:
                up = state.up + action.vz_up * t
                yaw = state.yaw_deg + yaw_dps * t
                if t < lag - 1e-12:
                    dx, dy = early(t)
                    adx, ady = dx, dy
                else:
                    ex, ey, _, _ = _response._approach(vxl, vyl, cx, cy, acc, t - lag)
                    dx, dy = dxl + ex, dyl + ey
                    adx, ady = dx, dy
                    if w:
                        # The arc, from when the command starts to act (t -
                        # lag): the response approaches the command, which
                        # turns from then on; added as the arc's lead over
                        # the straight command. AN APPROXIMATION: this
                        # velocity turns from t - lag while the pose's
                        # heading (yaw_deg) and the commanded arc below turn
                        # from t = 0, so even once the response has converged
                        # its direction trails the commanded arc's by
                        # w * lag for the rest of the horizon. It is judged
                        # beside the commanded arc and both lines, never
                        # alone (THE ARC ONLY ADDS).
                        ax, ay = _arc_offset(cx, cy, w, t - lag)
                        adx += ax - cx * (t - lag)
                        ady += ay - cy * (t - lag)
                if lines:
                    out.append((t, _Pose(x=state.x + dx, y=state.y + dy, up=up, yaw_deg=yaw)))
                if w:
                    out.append((t, _Pose(x=state.x + adx, y=state.y + ady, up=up, yaw_deg=yaw)))
                if not own_only:
                    if lines:
                        out.append((t, _Pose(
                            x=state.x + action.vx * t,
                            y=state.y + action.vy * t,
                            up=state.up + action.vz_up * t,
                            yaw_deg=state.yaw_deg + yaw_dps * t,
                        )))
                    if w:
                        ax, ay = _arc_offset(action.vx, action.vy, w, t)
                        out.append((t, _Pose(x=state.x + ax, y=state.y + ay, up=up,
                                             yaw_deg=yaw)))
                if t >= horizon - 1e-9:
                    break
                t = min(t + self.dt, horizon)
            if key is not None:
                tick.committed[key] = out
            yield from out
            return
        while True:
            if lines:
                yield t, _Pose(
                    x=state.x + action.vx * t,
                    y=state.y + action.vy * t,
                    up=state.up + action.vz_up * t,
                    yaw_deg=state.yaw_deg + yaw_dps * t,
                )
            if w:
                ax, ay = _arc_offset(action.vx, action.vy, w, t)
                yield t, _Pose(
                    x=state.x + ax,
                    y=state.y + ay,
                    up=state.up + action.vz_up * t,
                    yaw_deg=state.yaw_deg + yaw_dps * t,
                )
            if t >= horizon - 1e-9:
                return
            t = min(t + self.dt, horizon)

    def _arc_rate(self, action: Action4D) -> float:
        """The yaw rate (rad/s) the forecast of `action` turns its velocity
        at: the action's own, inside `filter_body(..., arc=True)` (or a
        `forecast(..., arc=True)`), when finite and non-zero; else 0.0, the
        straight forecast every other call gets."""
        if not getattr(self._local, "arc", False):
            return 0.0
        w = action.yaw_rate
        return float(w) if (math.isfinite(w) and abs(w) >= ARC_MIN_RATE) else 0.0

    # ---------------- the own-velocity forecast's helpers ---------------- #

    def _velocity_of(self, state) -> tuple[float, float] | None:
        """The measured horizontal velocity (vx, vy) the forecast starts from,
        or None for the commanded forecast: no velocity on the state (every
        State before 2026-10-19, every forecast pose, a stop point), no
        response model (`velocity_response=None`), or a non-finite value
        (counted once per tick by filter(), never flown on)."""
        vel = getattr(state, "velocity", None)
        if vel is None or self.velocity_response is None:
            return None
        vx, vy = vel[0], vel[1]
        if not (math.isfinite(vx) and math.isfinite(vy)):
            return None
        return float(vx), float(vy)

    def note_flown(self, action: "Action4D | None", t: float | None = None) -> None:
        """Tell the Shield what the rail SENT the autopilot this tick (None:
        nothing - the autopilot held the aircraft, or a fault). Call it after
        the send, once per tick. `t` is the time on the pipe clock: by
        default the `pipe_clock` reading now (the send's real time), or,
        without a pipe clock, the last tick's monitor time (filter()'s).

        The own-velocity forecast reads it (`pipe`): a setpoint sent at t is
        what the aircraft flies from t + reaction_s, so the ones sent in the
        last reaction time are still to come when this tick's command starts
        to act. Without them the forecast assumes the aircraft keeps its
        measured velocity until then, and an aircraft at rest that has just
        been sent 2.9 m/s towards a wall is forecast to sit still for the
        whole reaction time (0.45 s) while it is in fact accelerating (the
        closed loop's second approach, docs/DESIGN-own-velocity-forecast.md).
        It changes nothing for a state without a measured velocity.

        A Shield with no escalation FSM of its own (a rail's, guardrail/
        replay.py rail_shield) also dead-reckons on it while the position is
        not a valid fix (State.fix_ok False, X8's input; 2026-11-07): the
        rail's FSM decided after filter() what flew - a stop, nothing while
        the autopilot holds, or the Shield's action - so the next tick's
        estimate integrates what was SENT on this one (None: 0, the
        autopilot holding in place), not the action filter() emitted. It is
        read only while State.fix_ok is False."""
        if t is None:
            t = (float(self._pipe_clock()) if self._pipe_clock is not None
                 else self._t_last if self._t_last is not None else 0.0)
        t = float(t)
        finite = action is not None and math.isfinite(action.vx) and math.isfinite(action.vy)
        if not finite:
            self._flown.append((t, None, None))
        else:
            self._flown.append((t, float(action.vx), float(action.vy)))
            self.n_noted_setpoints += 1
        if self.fsm is None and self._t_last is not None:
            self._pk_cmd = ((self._t_last, float(action.vx), float(action.vy)) if finite
                            else (self._t_last, 0.0, 0.0))

    def pipe(self, t: float | None = None) -> list:
        """The setpoints already sent that act during the next reaction time,
        as reaction segments (guardrail/response.pipe_segments); [] with no
        record or no response model (the forecast then keeps the measured
        velocity for the reaction time). `t` defaults to this tick's time on
        the pipe clock (`_pipe_now`). Inside `forecast(..., pipe=...)` it is
        the pipe given there (a TickRecord's, for an exact rebuild)."""
        if t is None:
            ov = getattr(self._local, "pipe_override", None)
            if ov is not None:
                return list(ov)
        if self.velocity_response is None or not self._flown:
            return []
        tick = None
        if t is None:
            tick = self._tick_ctx()
            t = self._pipe_now(tick)
        if tick is None:
            return _response.pipe_segments(self._flown, t, self.velocity_response.reaction_s)
        # Once per tick (the forecast asks for it many times a tick); keyed on
        # the record's length and last entry too, so a setpoint noted inside
        # a tick is never missed.
        key = ("pipe", t, len(self._flown), self._flown[-1])
        hit = tick.committed.get(key)
        if hit is None:
            hit = tick.committed[key] = _response.pipe_segments(
                self._flown, t, self.velocity_response.reaction_s)
        return list(hit)

    def _pipe_now(self, tick) -> float:
        """Now on the pipe clock: the `pipe_clock` reading (latched on the
        tick at its first use, so one tick's forecasts all read one pipe), or
        without one the tick's monitor time - the last tick's between ticks."""
        if tick is None:
            if self._pipe_clock is not None:
                return float(self._pipe_clock())
            return self._t_last or 0.0
        if tick.pipe_t is None:
            tick.pipe_t = (float(self._pipe_clock()) if self._pipe_clock is not None
                           else tick.t)
        return tick.pipe_t

    def _pipe_pre(self, v: tuple[float, float]):
        """(reaction_s, dx, dy, vx, vy at the end of the reaction, early(t))
        for measured velocity `v` on this tick: the part of every forecast
        no candidate action can change, computed once per tick. `early(t)`
        gives the displacement at a time inside the reaction."""
        tick = self._tick_ctx()
        segs = self.pipe()
        key = ("pre", v, tuple(segs))
        if tick is not None:
            hit = tick.committed.get(key)
            if hit is not None:
                return hit
        resp = self.velocity_response
        segs = resp.reaction(segs)
        lag = sum(s[0] for s in segs)
        dxl, dyl, vxl, vyl = _response.integrate(v[0], v[1], segs, [lag], resp.accel_mps2)[0]
        memo: dict = {}

        def early(t: float, _segs=segs, _v=v, _a=resp.accel_mps2, _m=memo):
            got = _m.get(t)
            if got is None:
                dx, dy, _, _ = _response.integrate(_v[0], _v[1], _segs, [t], _a)[0]
                got = _m[t] = (dx, dy)
            return got
        out = (lag, dxl, dyl, vxl, vyl, early)
        if tick is not None:
            tick.committed[key] = out
        return out

    @staticmethod
    def _positional(state: State) -> State:
        """`state` without its measured velocity: the POSITION questions
        (state_is_unsafe, unsafe_depth, the FSM's stop_illegal) stay about
        where the aircraft IS, as their callers and the KPIs read them."""
        if getattr(state, "velocity", None) is None:
            return state
        return state.model_copy(update={"velocity": None})

    def _committed(self, state):
        """(stop_state, rule ids) for a state with a measured velocity, else
        None.

        The rules the aircraft's own momentum already carries it into: a
        GUIDED stop commanded NOW (zero velocity, through the response
        model: the reaction, then the braking) would still break them - the
        BRAKE action's check with the own-velocity forecast, which is the
        stop path. No candidate action can avoid that part of the forecast:
        whatever is commanded, the aircraft keeps its velocity for the
        reaction time, and no action brakes the approach harder than a stop.

        Judging such a rule on the own-velocity forecast would refuse every
        action, the Shield would fly its best effort with the violation on
        it, and the grant's P0 escape count would charge the Shield with an
        incursion it could not prevent. So each committed rule is judged as
        if the aircraft were already where the stop would end (`stop_state`,
        with no velocity): the trend-aware "already inside" tests then ask
        what they ask of an aircraft inside the ring or zone - is the
        command leaving it? - and the repairs answer as they would from
        there (ClearanceFix's push, GeofenceEscape, StandoffRecover).
        Recovery starts while the aircraft is still outside. A rule the
        POSITION already breaks is judged as before the change, from where
        the aircraft is with no velocity (`_rule_state`). Every other rule is
        judged on the own-velocity forecast from where it is, and a stop
        (zero) is clean for each of those by construction.

        Returns (stop_state, committed ids, positional_state, ids the position
        breaks). Cached per tick and state."""
        v = self._velocity_of(state)
        if v is None or getattr(self._local, "no_commit", False):
            return None
        tick = self._tick_ctx()
        # The rules the chain leaves alone (tick.skip) are not in the answer,
        # so it is cached per skip set too.
        key = (state.x, state.y, state.up, state.yaw_deg, v,
               tick.skip if tick is not None else None)
        if tick is not None:
            hit = tick.committed.get(key)
            if hit is not None:
                return hit
        # The GUIDED stop: the pipe (or the measured velocity kept) for the
        # reaction, then zero at the response's limit, to the standstill.
        path = _response.stop_path(self.velocity_response, None, state.x, state.y,
                                   v[0], v[1], pipe=self.pipe(), dt=self.dt)
        _, sx, sy = path[-1]
        stop = State(x=sx, y=sy, up=state.up, yaw_deg=state.yaw_deg)
        here = self._positional(state)
        self._local.no_commit = True
        try:
            ids = frozenset(x.rule_id for x in self._check_in(state, BRAKE))
            # Rules the POSITION already breaks are judged exactly as before:
            # from where the aircraft is, on the commanded forecast, so the
            # trend tests ask whether the COMMAND leaves the rule and the
            # recovery floors size the command. On the own-velocity forecast
            # the setpoints already in the pipe would be credited to it: a
            # zero command 0.8 m inside the clearance ring "receded" on the
            # one tick after a 0.5 m/s push, passed, and the escape crawled
            # out at half the floor, push and zero alternating (the closed
            # loop on a slower aircraft, docs/DESIGN-own-velocity-forecast.md).
            inside = frozenset(x.rule_id for x in self._check_in(here, BRAKE))
        finally:
            self._local.no_commit = False
        out = (stop, ids - inside, here, inside)
        if tick is not None:
            tick.committed[key] = out
        return out

    def _rule_state(self, rule, state):
        """The state rule `rule` is judged from: where the aircraft is,
        without its velocity, if its position already breaks the rule; the
        stop point if its momentum carries it into the rule (`_committed`);
        else `state` itself (always `state` without a measured velocity)."""
        com = self._committed(state)
        if com is None:
            return state
        if rule.id in com[3]:
            return com[2]
        if rule.id in com[1]:
            return com[0]
        return state

    def _clear_steps(self, state: State, action: Action4D) -> list:
        """The clearance rule's forecast: "can the aircraft still stop before
        the ring after flying `action`?".

        Commanded forecast (no measured velocity): `action` at constant
        velocity over `_clear_horizon_s` - one control step of reaction plus
        the stopping distance at the nominal 3 m/s^2, expressed as a time at
        the commanded speed. Exactly as before.

        Own-velocity forecast: from the measured velocity, the aircraft flies
        what was already sent for the reaction time (`pipe`), then `action`
        for one control period (dt), then a stop at the response's limit -
        sampled on the grid until it is stationary (the last sample exactly
        at the stop, at most the lookahead). The stop the old form assumed
        at 3 m/s^2 after 0.1 s was 1.5 m shorter than the 14 Oct aircraft's
        from 3.2 m/s.

        Always a single path read in order: for an arced action (`_arc_rate`)
        the commanded forecast here is the LINE, and the arc is judged
        beside it (`_clear_arc_steps`; THE ARC ONLY ADDS). The own-velocity
        "stop after one control period" forecast is not arced: its one
        period of the command turns the velocity by at most w * dt."""
        v = self._velocity_of(state)
        if v is None:
            if self._arc_rate(action):
                return list(self._predict(state, action, self._clear_horizon_s(action),
                                          arc_path="line"))
            return list(self._predict(state, action, self._clear_horizon_s(action)))
        resp = self.velocity_response
        dt, a = self.dt, resp.accel_mps2
        pipe = self.pipe()
        segs = resp.reaction(pipe) + [(dt, action.vx, action.vy), (math.inf, 0.0, 0.0)]
        lag = sum(s[0] for s in resp.reaction(pipe))
        _, _, v1x, v1y = _response.integrate(v[0], v[1], segs, [lag + dt], a)[0]
        horizon = min(self.lookahead_s, lag + dt + math.hypot(v1x, v1y) / a)
        times, t = [], 0.0
        while True:
            times.append(t)
            if t >= horizon - 1e-9:
                break
            t = min(t + dt, horizon)
        yaw_dps = math.degrees(action.yaw_rate)
        return [(t, _Pose(x=state.x + dx, y=state.y + dy, up=state.up + action.vz_up * t,
                          yaw_deg=state.yaw_deg + yaw_dps * t))
                for t, (dx, dy, _, _) in zip(times, _response.integrate(v[0], v[1], segs,
                                                                        times, a))]

    def _clear_horizon_s(self, action: Action4D) -> float:
        """How far ahead the CLEARANCE rule looks: one control step of reaction
        plus the distance still needed to stop, expressed as a time at the
        current speed (which is what `_predict` advances at).

        Using the full geofence lookahead here is what made the rule unflyable:
        a constant-velocity 3 s forecast at 4 m/s puts the effective standoff at
        min_clearance_m + 12 m, so any planned corridor narrower than that trips
        the rule every time the path curves — even though the vehicle would
        follow the curve and never get close."""
        v = math.hypot(action.vx, action.vy)
        if v <= 1e-9:
            return 0.0
        return min(self.lookahead_s, self.dt + v / (2.0 * _CLEAR_DECEL_MPS2))

    def _clear_min_dist(self, state: State, a: Action4D, f=None) -> float:
        """Smallest mapped-obstacle distance `a`'s FORECAST reaches. This is the
        score every clearance repair is judged by: a repair must raise it, never
        lower it.

        The t = 0 sample is excluded on purpose — every candidate action shares
        the present position, so scoring it in makes the number blind exactly
        when the vehicle is already inside the ring and the choice matters most.
        (A standstill has no forecast, so it scores its own position.)
        `f`: the field (_distance_at); None, the obstacle map."""
        if (self._dist if f is None else f[0]) is None:
            return float("inf")
        steps = self._clear_steps(state, a)
        d = min(self._distance_at(p.x, p.y, f) for _, p in (steps[1:] or steps))
        arc = self._clear_arc_steps(state, a)
        if arc is not None:
            # The arc beside the line (THE ARC ONLY ADDS): the nearer of the two.
            d = min(d, min(self._distance_at(p.x, p.y, f) for _, p in (arc[1:] or arc)))
        return d

    def _clear_arc_steps(self, state: State, action: Action4D) -> "list | None":
        """The clearance rule's ARC path for an arced action (`_arc_rate`)
        from a state without a measured velocity: `action` along its arc over
        `_clear_horizon_s`, judged beside the line `_clear_steps` returns.
        None otherwise (no arc, or the own-velocity forecast, whose commanded
        part is judged from the position: `_check_clearance`)."""
        if not self._arc_rate(action) or self._velocity_of(state) is not None:
            return None
        return list(self._predict(state, action, self._clear_horizon_s(action),
                                  arc_path="arc"))

    def _clear_dir_candidates(self, a: Action4D, cap: float):
        """Recovery headings: 12 bearings x 2 speeds, inheriting `a`'s vertical
        and yaw components (those were already repaired by the altitude operator,
        so they must not be re-invented here) — re-clamped so a candidate can
        never be rejected for a kinematic reason it inherited."""
        vz, yr = a.vz_up, a.yaw_rate
        for k in self._kins:
            vz = max(-k.climb_rate_max_mps, min(k.climb_rate_max_mps, vz))
            # UNITS. The policy states the cap in DEGREES per second; the
            # Action4D contract carries yaw_rate in RADIANS per second.
            # These were compared raw, so a 45 dps cap sat at 45 rad/s
            # (2578 dps) and this P1 rule could never fire on any real
            # action. Convert at the boundary, here and at the two other
            # sites below.
            ymax = math.radians(k.yaw_rate_max_dps)
            yr = max(-ymax, min(ymax, yr))
        for k in range(12):
            ang = k * math.pi / 6.0
            ux, uy = math.cos(ang), math.sin(ang)
            for frac in (1.0, 0.5):
                spd = cap * frac
                yield Action4D(vx=ux * spd, vy=uy * spd, vz_up=vz, yaw_rate=yr)

    def _best_clear_dir(self, state: State, a: Action4D, cap: float, f=None):
        """The candidate heading whose clearance forecast keeps the most room,
        ties broken toward the commanded heading. `f`: the field."""
        if (self._dist if f is None else f[0]) is None:
            return None
        best, best_key = None, None
        for cand in self._clear_dir_candidates(a, cap):
            key = (self._clear_min_dist(state, cand, f),
                   cand.vx * a.vx + cand.vy * a.vy)
            if best_key is None or key > best_key:
                best, best_key = cand, key
        return best

    def _rescue(self, state: State, raw: Action4D, a: Action4D, cap: float):
        """Best available recovery when the repair chain did not converge.

        Returns (clean, best): `clean` re-checks with ZERO violations, `best`
        is chosen first among the headings that take the aircraft DEEPER into
        no hard rule it is already inside (`_deepens`), then by clearance, then
        toward the commanded heading. Stopping is not an answer on its own —
        a stationary vehicle inside the clearance ring (or inside a fence) is
        still in violation on the next tick and every tick after it, so a brake
        there is a permanent freeze, not a fail-safe.

        Until 2026-10-08 `best` was ranked by clearance alone, so where the
        clearance push and a stand-off ring disagreed the best effort could fly
        straight at the subject whose ring the aircraft was already inside."""
        clean = clean_key = best = best_key = None
        for cand in self._clear_dir_candidates(a, cap):
            key = (self._clear_min_dist(state, cand) if self._dist is not None else 0.0,
                   cand.vx * raw.vx + cand.vy * raw.vy)
            bkey = (-self._deepens(state, cand),) + key
            if best_key is None or bkey > best_key:
                best, best_key = cand, bkey
            if not self._check(state, cand) and (clean_key is None or key > clean_key):
                clean, clean_key = cand, key
        return clean, best

    def _deepens(self, state: State, a: Action4D) -> int:
        """How many HARD, enforced rules the aircraft is already inside that
        `a` takes it deeper into: a stand-off ring whose subject it closes on,
        a zone it heads further into (relative to the zone, if it moves).
        Directly over a subject every heading opens the range, so none deepens
        that ring. Altitude is not counted: every recovery candidate carries
        the same, already repaired, vertical speed."""
        tick = self._tick_ctx()
        rt = tick.rt if tick is not None else self._rt

        def enforced_hard(rule) -> bool:
            alive = self._mask(rule)
            return (rt.hard.get(rule.id, True)
                    and self._act(rt, rule.id) != "monitor_only"
                    and (alive is True or (alive is not False and alive(0.0))))

        n = 0
        if self._subject is not None:
            sx, sy, svx, svy = self._subject_track()
            d = math.hypot(state.x - sx, state.y - sy)
            if d > _STANDOFF_COINCIDENT_M:
                # Closing RELATIVE to the subject (its own velocity is zero
                # unless set_subject was given one).
                closing = ((a.vx - svx) * (sx - state.x) + (a.vy - svy) * (sy - state.y)) / d
                n += sum(1 for so in self._standoffs
                         if so.binds(self._subject_class) and enforced_hard(so)
                         and self._standoff_inside(so, d) and closing > 1e-9)
        anchors, tick = self._fence_anchors(self._ir)
        here = (state.x, state.y, state.x, state.y)
        for rec in anchors:
            f = rec.rule
            if not enforced_hard(f):
                continue
            rec = self._reach(rec, tick, here, 0.0)
            if rec is None or not point_in_fence(
                    state.x, state.y, state.up, f, rec.polygon, rec.buffered):
                continue
            mv = self._motion_of(f)
            ex, ey = (a.vx - mv[0], a.vy - mv[1]) if mv else (a.vx, a.vy)
            ox, oy = escape_direction(state.x, state.y, rec.polygon, a.vx, a.vy)
            n += int(ex * ox + ey * oy < -1e-9)
        return n

    # ---------------- the monitor, one predicate per rule ---------------- #
    #
    # Split out of a single `_check` on 2026-08-26 so that the REPAIRS can ask
    # the same question the monitor asks, instead of each carrying its own
    # weaker copy of the trend test. Three repair operators had drifted into
    # judging position only, and would clobber an action that was already
    # escaping - measured at six times slower on the standoff rule.
    #
    # `_check` folds these and is unchanged in behaviour, which
    # tests/test_check_contract.py pins by hashing its answers over 28 800
    # seeded samples across every policy in policies/.

    def _check_kinematic(self, k, action: Action4D) -> list[Violation]:
        """A property of the action alone - no state, no forecast."""
        out = []
        # Each cap with the float32 contract's slack (fsm.cap_exceeded, since
        # 2026-11-07; 1e-9 absolute before).
        h_speed = (action.vx ** 2 + action.vy ** 2) ** 0.5
        if _fsm.cap_exceeded(h_speed, k.speed_max_mps):
            out.append(Violation(
                rule_id=k.id, category="kinematic", predicted_at_s=0.0,
                detail=f"h-speed {h_speed:.2f} > max {k.speed_max_mps}"))
        if _fsm.cap_exceeded(abs(action.vz_up), k.climb_rate_max_mps):
            out.append(Violation(
                rule_id=k.id, category="kinematic", predicted_at_s=0.0,
                detail=f"|vz| {abs(action.vz_up):.2f} > max {k.climb_rate_max_mps}"))
        yaw_dps = math.degrees(abs(action.yaw_rate))     # contract is rad/s
        if _fsm.cap_exceeded(yaw_dps, k.yaw_rate_max_dps):
            out.append(Violation(
                rule_id=k.id, category="kinematic", predicted_at_s=0.0,
                detail=f"|yaw_rate| {yaw_dps:.1f} dps > max {k.yaw_rate_max_dps}"))
        return out

    def _check_altitude(self, env, state: State, action: Action4D) -> list[Violation]:
        """Trend-aware. Outside the band but moving back in = OK. A pose is
        judged only if the rule is in force at that pose's time (`_mask`)."""
        alive = self._mask(env)
        if alive is False:
            return []
        # With a measured velocity the own-velocity and commanded poses share
        # every altitude and time (the climb is forecast as commanded in
        # both: _predict), so the own-velocity path alone is walked - the
        # same first breach, at the same time, with the same altitude.
        kw = {"own_only": True} if self._velocity_of(state) is not None else {}
        for t, p in self._predict(state, action, **kw):
            if alive is not True and not alive(t):
                continue
            below = p.up < env.alt_min_m - 1e-9
            above = p.up > env.alt_max_m + 1e-9
            if below and action.vz_up <= 1e-9:
                return [Violation(
                    rule_id=env.id, category="altitude", predicted_at_s=t,
                    detail=f"alt {p.up:.1f}m < floor {env.alt_min_m}m, not climbing")]
            if above and action.vz_up >= -1e-9:
                return [Violation(
                    rule_id=env.id, category="altitude", predicted_at_s=t,
                    detail=f"alt {p.up:.1f}m > ceiling {env.alt_max_m}m, not descending")]
        return []

    def _check_standoff(self, so, state: State, action: Action4D) -> list[Violation]:
        """Trend-aware, same shape as the fence rule.

        Inert with no subject set, which is the honest reading: a standoff rule
        with nothing to stand off from has no opinion. "Already too close but
        opening the range" passes, because the alternative is to raise a
        violation on the very action that is fixing it - and BRAKE inside the
        ring would freeze the aircraft at the distance it must not hold.

        A MOVING subject (set_subject(velocity=...)) is forecast at constant
        velocity, and both tests read the motion RELATIVE to it: the range at
        each pose's time is to where the subject will be then, and "opening"
        is the relative radial rate. Held still - the only reading before
        2026-10-19, and still the default - the grant's 5 s forecast of a
        drone following a car at 4-5 m/s runs 20-25 m ahead into the car's
        ring, so StandoffHold fired on 44 % of the replayed city ticks while
        the range was not closing at all; and a car driving AT a hovering
        drone was never forecast to arrive.
        """
        if self._subject is None or not so.binds(self._subject_class):
            return []
        alive = self._mask(so)
        if alive is False:
            return []
        # From the stop point when the aircraft's momentum already carries it
        # into the ring (`_committed`); `state` itself otherwise, and always
        # without a measured velocity.
        state = self._rule_state(so, state)
        # svx = svy = 0.0 with no velocity, which subtracts and adds exact
        # zeros below: every number, and so every decision, is unchanged.
        sx, sy, svx, svy = self._subject_track()
        d_now = math.hypot(state.x - sx, state.y - sy)
        what = self._subject_class or "subject"

        if (alive is True or alive(0.0)) and self._standoff_inside(so, d_now):
            # ALREADY inside. Judge the trend from the radial velocity, not from
            # predicted positions: _predict yields the current pose as its first
            # sample, where the range has not changed yet, so a comparison
            # against it can never see an opening move and the rule fires on the
            # very action that is recovering. Same shape as the geofence
            # "escaping" test.
            if d_now > _STANDOFF_COINCIDENT_M:
                ux, uy = (sx - state.x) / d_now, (sy - state.y) / d_now
                opening = -((action.vx - svx) * ux + (action.vy - svy) * uy)
            else:
                # Directly over the subject: range 0 has no line to the
                # subject, and the radial component along a zero vector is 0
                # for EVERY action, so no action could ever pass and the
                # Shield flew its best effort with the P0 still on it (5 ticks
                # of the 8 Oct follow; docs/FINDING-the-escape-that-stayed-
                # inside-the-ring.md). From range 0 the range grows at the
                # horizontal speed whatever the heading (the one-sided
                # derivative of |p - s| at p = s), which is also what a
                # one-second probe of the range would read - the trend test the
                # PI reference takes for a zone. (Relative to a moving
                # subject: the speed of the motion relative to it.)
                opening = math.hypot(action.vx - svx, action.vy - svy)
            if opening <= 0.1:
                return [Violation(
                    rule_id=so.id, category="standoff", predicted_at_s=0.0,
                    detail=(f"range {d_now:.1f}m to {what} < min "
                            f"{so.min_range_m}m and not opening"))]
            return []        # while inside, predictive checks are moot

        for t, p in self._predict(state, action):
            if alive is not True and not alive(t):
                continue
            d = math.hypot(p.x - (sx + svx * t), p.y - (sy + svy * t))
            if self._standoff_inside(so, d):
                return [Violation(
                    rule_id=so.id, category="standoff", predicted_at_s=t,
                    detail=(f"range closes to {d:.1f}m from {what} in "
                            f"{t:.1f}s, below min {so.min_range_m}m"))]
        return []

    def _opening_imposed(self, f, ring, state: State) -> float | None:
        """When zone `f` is not in force now and its window opens at forecast
        time t_open (the first pose the check judges it at), with the aircraft
        inside its margin ring and band so deep that NO action within the
        policy's hard, enforced caps can be out of the ring by t_open: that
        t_open. None in every other case (the zone is in force now, opens
        later than the aircraft can leave, never opens in the lookahead, or
        cannot be proved imposed).

        WHY (C2, 2026-10-21). A position made illegal UNDER the aircraft is
        the scenario's doing, not the pilot's, and the monitor has always
        judged it by its trend: inside a zone in force, an action leaving at
        more than 0.1 m/s is not a violation (the inside rule in
        _check_fence), and the repair floors how fast it leaves
        (GeofenceEscape, Shield._floor_the_escape). A window opening on the
        aircraft was the one case where that rule was not applied: until the
        opening the zone is not in force now, so the check took the
        predictive branch, found every forecast pose at t_open inside the
        ring - whatever the action - and called the fastest way out a
        violation. That is the library's three time-window-opens-on-vehicle
        cells (aircraft at (60, 21), 10 m from the ring, 4 m/s cap, window
        opening 1.0-2.0 s in): no action cleared the forecast, the Shield
        flew its best effort (4 m/s north) with the P0 violation still on it,
        and its own re-check counted 2 P0 escapes per cell, the library's
        whole non-zero P0 headline (6 ticks, CHANGELOG 2026-10-13 / -21).
        The same action 0.1 s after the opening, the aircraft inside the zone
        in force, was legal. So when the opening is imposed, the zone is
        judged from t_open by the inside rule (escaping is legal, anything
        else is the violation it always was, word for word): what the KPI
        counts is unchanged ("a P0 seen and flown anyway"); the check no
        longer sees a violation where no legal action existed and the action
        is the recovery the inside rule asks for.

        PROVED, NOT GUESSED. Imposed only when an ideal vehicle at the caps
        cannot leave by t_open: horizontal distance to the ring's boundary
        greater than speed cap x t_open AND the band's top and bottom each
        further than climb cap x t_open. The caps are the kinematic rules that
        are hard, enforced (not monitor_only) and in force over the whole
        tick; with none, nothing is proved and the old judgement stands. The
        ideal vehicle reaches further than any real one, so a real aircraft
        is never called imposed where it could have left. A zone that MOVES
        (dynamic_nfz with motion) keeps the old judgement: its escape is
        relative to the zone, which this does not model, and no library cell
        or rail flies one with a window. A state that still carries a
        MEASURED velocity is not proved either (its forecast is two paths);
        in practice that guard never decides, because an imposed opening
        makes a stop at the position illegal, and the Shield then sets the
        velocity aside for the decision (_decision_state) and for that rule
        (_rule_state), so the zone is judged from the commanded forecast as
        without a velocity (tests/test_window_opens_on.py). Not done for
        corridors (keep-in) or stand-offs: no window opens a stand-off, and a
        corridor that comes into force with the aircraft outside it is a
        separate question.

        FROM A STOP POINT TOO (review of 3660383, P8). _check_fence asks this
        of the state the zone is judged from (_rule_state), and for a zone
        the aircraft's momentum carries it into that is the GUIDED stop point
        (_committed), which carries no velocity: an aircraft 3 m short of the
        ring at 4 m/s stops 3.4 m inside it. There the proof is about the
        stop point - an ideal vehicle standing at it cannot be out of the
        ring by t_open - and the real aircraft, behind it, can do no better:
        no action brakes the approach harder than the stop, so it is inside
        the ring once the zone is in force whatever is commanded, which is
        what _committed already says of a zone in force now. The judgement is
        that one's too: as if the aircraft were where the stop ends, only an
        action leaving passes (reversing out), and carrying on or stopping is
        the pose walk's violation from the stop point, word for word. A stop
        point the ideal vehicle could still get out from by t_open is not
        claimed, and the pose walk judges it from there as before.
        `openings_imposed` asks the same per-rule state, so the trace, the
        harness's anticipated count and the cap's memory (`_opened_on`) name
        the zone exactly when the check applies this rule
        (tests/test_window_opens_on.py)."""
        if ring is None or self._velocity_of(state) is not None:
            return None
        tick = self._tick_ctx()
        if tick is None or f.id not in tick.rt.timed or self._motion_of(f) is not None:
            return None
        alive = tick.sched.mask(f)
        if alive is True or alive is False or alive(0.0):
            return None
        if not point_in_fence(state.x, state.y, state.up, f, None, ring):
            return None
        t_open = tick.sched.first(f)
        if t_open is None or t_open <= 0.0:
            return None
        rt = tick.rt
        caps = [k for k in rt.kins if rt.hard.get(k.id, True)
                and self._act(rt, k.id) != "monitor_only" and tick.sched.mask(k) is True]
        if not caps:
            return None
        v_h = min(k.speed_max_mps for k in caps)
        v_z = min(k.climb_rate_max_mps for k in caps)
        if v_h * t_open >= ring.boundary.distance(Point(state.x, state.y)) - 1e-9:
            return None                    # the fastest straight exit is out in time
        if (v_z * t_open > f.altitude_ceiling_m - state.up
                or v_z * t_open > state.up - f.altitude_floor_m):
            return None                    # ... or over / under the band
        return t_open

    def openings_imposed(self, state: State) -> list[str]:
        """The zones whose window opens on the aircraft sooner than it can
        leave (`_opening_imposed`), at this tick's clock: the Shield sees each
        coming and enforces it before it is in force - a stop there is illegal
        and only an escaping action passes - which is why the decision's trace
        lists them (`window_opens_on`) and the harness counts them as
        anticipated. Empty without a timed zone.

        Each zone is asked from the state _check_fence judges it from
        (_rule_state): the position without its velocity if the position
        breaks it, the stop point if the aircraft's momentum carries it in,
        else `state`. Until the review of 3660383 (P8) this asked `state`
        itself, so with a measured velocity a zone the check judged by the
        opening rule from the stop point was missing from the trace and from
        the anticipated count."""
        tick = self._tick_ctx()
        if tick is None:
            if not self._needs_tick():
                return []
            with self._scope(self._new_tick(None)):
                return self.openings_imposed(state)
        if not tick.rt.timed or tick.sched.const:
            return []
        recs, _ = self._fence_anchors(self._ir)
        return sorted({r.rule.id for r in recs if not r.moving
                       and self._opening_imposed(r.rule, r.buffered,
                                                 self._rule_state(r.rule, state)) is not None})

    def _forget_left_openings(self, state: State) -> None:
        """Keep in `_opened_on` only the zones whose window opened on the
        aircraft that it is still inside: the zone is still in the policy and
        not moving, the position it is judged from is inside its margin ring
        and band, and it is in force now or its opening is still imposed.
        Called by filter() at the start of each tick, before the decision.

        WHY (review of 3660383, P1). _opening_imposed applies until the zone
        opens; from the next tick the zone is in force with the aircraft
        inside it, and the inside rule's escape floor is the in-force one,
        2.0 m/s (min(2.0, cap), GeofenceEscape). For a slow pilot that is the
        speed the aircraft then left at, so a window opening on a 1.5 m/s
        pilot kept it inside the polygon 3 more ticks per library cell than
        7c9c9e6's FSM hold transit, which left at the 4 m/s cap - and with
        the floor at 2.0 before the opening too, 23 more (5.1 s to safe
        against 2.6 s), the KPI scoring the longer stay as no escape at all.
        A zone that opened on the aircraft is the scenario's doing, and the
        aircraft must not stay in it longer than it did when every tick there
        was escalated; so its escape flies at the cap until the aircraft is
        out of the ring, and the Shield has to remember which zones those
        are, because from inside a zone in force nothing in the state says
        whether the aircraft flew in or the window opened on it. A zone the
        aircraft enters by itself, or re-enters after leaving one that
        opened on it, is floored at 2.0 as before."""
        if not self._opened_on:
            return
        recs, _ = self._fence_anchors(self._ir)
        keep = set()
        for r in recs:
            f = r.rule
            if f.id not in self._opened_on or r.moving:
                continue
            st = self._rule_state(f, state)
            if not point_in_fence(st.x, st.y, st.up, f, r.polygon, r.buffered):
                continue
            alive = self._mask(f)
            if (alive is True or (alive is not False and alive(0.0))
                    or self._opening_imposed(f, r.buffered, st) is not None):
                keep.add(f.id)
        self._opened_on = keep

    def _shaped_escape(self, state: State, action: Action4D) -> "list | None":
        """The forecast every rule judges `action` on when it ESCAPES a zone
        whose window opened on the aircraft: straight out of that zone's
        margin ring, then a stop - one forecast step (`dt`) still at the
        speed, then `_CLEAR_DECEL_MPS2` to a standstill, held to the horizon,
        the climb as commanded - on the forecast's own grid. None in every
        other case, and the rules judge the straight forecast (`_predict`)
        as they always have.

        "A zone whose window opened on the aircraft": hard, enforced, not
        moving, the position inside its ring and band, and either in force
        now and remembered (`_opened_on`) or still to open with the opening
        imposed (`_opening_imposed`). Under recovery_law "cap_all" (since
        2026-11-06) also any such zone the aircraft is inside for any other
        reason (Shield._cap_zones_at, for the state judged): every recovery from a
        static zone then flies at the cap, so it is judged the same way. "Escapes": leaving it at more than 0.1
        m/s by the inside rule's own test, at the pose where the zone opens
        for an opening still to come (as _check_fence reads it). The action
        must escape every such zone the aircraft is in, and the aircraft
        must be in no hard, enforced zone in force that it was not opened
        on; its straight path must be out of each ring before the horizon.
        Only for a state without a measured velocity: the commanded
        forecast, which is the one an illegal position is judged on anyway
        (_decision_state sets the velocity aside there).

        WHY (review of 69c27bd, P2). The escape from such a zone flies at the
        speed cap until the aircraft is out of the ring (GeofenceEscape,
        `_opened_on`); a floored escape flies only if it re-checks clean
        against every rule (Shield._floor_the_escape), and the re-check
        judged it on the straight 5 s forecast - the cap held for 20 m, as
        if the aircraft kept it long after it was out. So any hard rule
        within 20 m of the exit refused the cap, the library's own geometry
        among them: the corridor-survey-route edge at x = 80 is 9 m past the
        school zone's ring at x = 71, and a pilot flying 1-3 m/s east (or 1-2
        west) at the opening had its cap refused, its escape slowed to the
        speed whose straight forecast ends at x = 80 - 3.5, 3.0 ... 2.25 m/s
        as it neared the edge - and from 10 m short of it the pilot's own
        forecast broke the corridor and the FSM went Normal -> Brake ->
        Loiter: 16 ticks inside the polygon and 4.0 s to safe on the grant
        arm against 5-6 / 2.8 s on 7c9c9e6 (the review's probe2.py, p2_none).
        A zone or the corridor's end 4-9 m past the north exit cost 12-14
        ticks against 3. Nothing about the aircraft's position after the
        exit came from that forecast: out of the ring a stop is legal again,
        and the next tick judges the next action from there - which is what
        this forecast says. A rule beyond the stopping point therefore no
        longer refuses the cap; a rule inside it does, and then the retries
        (`_opened_retries`) fly the fastest speed whose stop is clean, and
        with none the tick is reported to the FSM (`_escape_shortfall`).

        The braking model is the commanded forecast's own: the clearance
        rule asks "can the aircraft still stop" with one step of reaction and
        3 m/s^2 (`_clear_horizon_s`), and the ideal vehicle the harness flies
        takes the next command on the next tick. From the cap that is 0.4 +
        2.67 m past the exit on the grant arm's 0.1 s step (2.0 + 2.67 on
        the 3 s arm's 0.5 s). The braking starts after the LAST time the
        straight path leaves the ring within the horizon, so a ring the path
        re-enters is never braked in.

        Judged by every fence and corridor (`_check_in`), and by the repairs
        that act on what they find (GeofenceSlide in _repair_geofence,
        CorridorReturn in _repair_corridor), so the operators act on what the
        check raised. Not by the stand-off rule, whose forecast moves the
        subject too, nor by the clearance rule, whose forecast is already a
        stop, nor by the altitude band (the climb is unchanged). Not
        `forecast()`, the public rebuild of the straight forecast. Cached
        per tick.

        Not for an arced action (`_arc_rate`, filter_body with arc): this
        shaping is a straight exit (`_ring_exit_time` is a straight line),
        so such an action is judged on the full 5 s line AND arc instead
        (`_predict`, THE ARC ONLY ADDS). That brings back, for a body
        command with a yaw rate escaping a zone that opened on the
        aircraft, the behaviour of 69c27bd this shaping removed: a hard rule
        within the cap's 5 s reach past the exit refuses the cap, and the
        retries fly the fastest speed that re-checks clean
        (tests/test_body_arc_forecast.py pins it in the library's own
        geometry). Opt-in only: no rail flies the arc by default."""
        if self._velocity_of(state) is not None or self._arc_rate(action):
            return None
        h = math.hypot(action.vx, action.vy)
        if not (math.isfinite(h) and h > 0.1):
            return None                       # nothing escapes at 0.1 m/s or less
        tick = self._tick_ctx()
        cap = self._cap_zones_at(tick, state) if tick is not None else frozenset()
        if tick is None or (not self._opened_on and not cap
                            and (not tick.rt.timed or tick.sched.const)):
            return None                       # no zone can have opened on it
        key = ("shaped", state.x, state.y, state.up, state.yaw_deg, action.vx,
               action.vy, action.vz_up, action.yaw_rate,
               frozenset(self._opened_on) | cap)
        hit = tick.committed.get(key)
        if hit is None:
            hit = tick.committed[key] = (self._shape_escape(tick, state, action, h),)
        return hit[0]

    def _shape_escape(self, tick: _Tick, state: State, action: Action4D,
                      h: float) -> "list | None":
        """_shaped_escape's forecast, uncached (`h`: the horizontal speed)."""
        rt = tick.rt
        exits = []
        # Every compiled zone, whatever the chain is skipping (tick.skip):
        # whether the aircraft is escaping a zone that opened on it is a fact
        # about the position, not about the rules being repaired.
        for r in rt.ir.fences:
            if r.moving:
                continue                      # never proved opened (_opening_imposed)
            f, poly, ring = r.rule, r.polygon, r.buffered
            if not rt.hard.get(f.id, True) or self._act(rt, f.id) == "monitor_only":
                continue
            alive = tick.sched.mask(f)
            if alive is False or not point_in_fence(state.x, state.y, state.up, f, poly, ring):
                continue
            if alive is True or alive(0.0):
                if f.id not in self._opened_on and f.id not in self._cap_zones_at(tick, state):
                    return None               # inside a zone in force it flew into
                ox, oy = escape_direction(state.x, state.y, poly, action.vx, action.vy)
            else:
                t_open = self._opening_imposed(f, ring, state)
                if t_open is None:
                    continue                  # can still leave in time: walked as before
                px, py = state.x + action.vx * t_open, state.y + action.vy * t_open
                if not point_in_fence(px, py, state.up + action.vz_up * t_open, f, poly, ring):
                    return None               # over the cap: walked as before
                ox, oy = escape_direction(px, py, poly, action.vx, action.vy)
            if action.vx * ox + action.vy * oy <= 0.1:
                return None                   # not leaving it: that zone's violation
            t_exit = self._ring_exit_time(ring, state, action, h)
            if t_exit is None:
                return None                   # still inside at the horizon
            exits.append(t_exit)
        if not exits:
            return None
        grid = self._grid()
        t_b = max(exits) + self.dt
        if t_b >= grid[-1] - 1e-9:
            return None                       # no braking inside the horizon
        a = _CLEAR_DECEL_MPS2
        ux, uy = action.vx / h, action.vy / h
        t_stop = h / a
        yaw_dps = math.degrees(action.yaw_rate)
        out = []
        for t in grid:
            if t <= t_b:
                d = h * t
            else:
                s = min(t - t_b, t_stop)
                d = h * t_b + h * s - 0.5 * a * s * s
            out.append((t, _Pose(x=state.x + ux * d, y=state.y + uy * d,
                                 up=state.up + action.vz_up * t,
                                 yaw_deg=state.yaw_deg + yaw_dps * t)))
        return out

    def _ring_exit_time(self, ring, state: State, action: Action4D,
                        h: float) -> float | None:
        """When the straight path of `action` from `state` (inside `ring`)
        crosses the ring's boundary for the LAST time within the forecast's
        horizon, or None if it is still inside the ring at the horizon. The
        last crossing, not the first: on a ring the path leaves and enters
        again, the braking (_shaped_escape) never starts inside it."""
        horizon = self._grid()[-1]
        x1, y1 = state.x + action.vx * horizon, state.y + action.vy * horizon
        if ring.contains(Point(x1, y1)):
            return None
        seg = LineString([(state.x, state.y), (x1, y1)])
        pts = shapely.get_coordinates(seg.intersection(ring.boundary))
        if len(pts) == 0:
            return None
        ux, uy = action.vx / h, action.vy / h
        s = max((px - state.x) * ux + (py - state.y) * uy for px, py in pts)
        return max(s, 0.0) / h

    def _check_fence(self, f, poly, state: State, action: Action4D,
                     poses=None, buffered=None) -> list[Violation]:
        """Trend-aware for the already-inside case.

        `poses` is `action`'s forecast when the caller already has it (`_check`
        computes it once for every fence instead of once per fence), and
        `buffered` the fence's prebuilt margin ring. Both default to computing
        them here, which is what this method always did.

        A pose is judged only if the zone is in force at that pose's time. A
        MOVING dynamic_nfz is passed at its current position; each forecast
        pose is moved into the zone's frame at its own time first
        (`_to_zone_frame`), so a zone drifting into the forecast is caught."""
        alive = self._mask(f)
        if alive is False:
            return []
        # A zone the aircraft's momentum already carries it into is judged
        # from the stop point (`_committed`), on that state's own forecast.
        st = self._rule_state(f, state)
        if st is not state:
            state, poses = st, None
        mv = self._motion_of(f)
        if self.segment_check and poses is None:
            poses = list(self._predict(state, action))
        if (alive is True or alive(0.0)) and point_in_fence(
                state.x, state.y, state.up, f, poly, buffered):
            # Escaping is judged RELATIVE to a moving zone: leaving at 1 m/s a
            # zone that follows at 2 m/s is not leaving it. Where two edges
            # are exactly as near (the zone's middle), the way out is the one
            # the command is heading for (escape_direction), not whichever the
            # ring's vertex order lists first.
            ex, ey = ((action.vx - mv[0], action.vy - mv[1]) if mv
                      else (action.vx, action.vy))
            ox, oy = escape_direction(state.x, state.y, poly, action.vx, action.vy)
            escaping = (ex * ox + ey * oy) > 0.1
            if not escaping:
                return [Violation(
                    rule_id=f.id, category="geofence", predicted_at_s=0.0,
                    detail="currently INSIDE zone and not escaping")]
            return []      # while inside, predictive entry checks are moot
        # A window opening on the aircraft sooner than ANY action can take it
        # out of the ring: the inside rule above, applied from the opening
        # (_opening_imposed says why, and when it does not apply). The pose
        # at t_open is where the aircraft is when the zone comes into force,
        # so "leaving" is read there, with that pose's way out (from the
        # zone's middle the nearest edge is a tie, and the current position's
        # way out is an arbitrary one of the two). An action that is not
        # leaving gets the very violation the pose walk below gives it (the
        # first pose judged, t_open, is inside: same pose, same words), so a
        # stop, a hover or a turn back is judged exactly as before; only the
        # escape stops being one. An action over the cap whose pose at
        # t_open is already out of the ring is walked as before.
        t_open = (self._opening_imposed(f, buffered, state)
                  if alive is not True and not mv else None)
        if t_open is not None:
            if poses is None:
                poses = list(self._predict(state, action))
            p_open = next((p for t, p in poses if abs(t - t_open) < 1e-9), None)
            if p_open is not None and point_in_fence(p_open.x, p_open.y, p_open.up,
                                                     f, poly, buffered):
                ox, oy = escape_direction(p_open.x, p_open.y, poly, action.vx, action.vy)
                if action.vx * ox + action.vy * oy > 0.1:
                    return []      # leaving, as the inside rule asks once it opens
                return [Violation(
                    rule_id=f.id, category="geofence", predicted_at_s=t_open,
                    detail=(f"predicted pos ({p_open.x:.1f},{p_open.y:.1f}) inside NFZ "
                            f"at t+{t_open:.1f}s"))]
        # A pose outside the ring's envelope cannot be inside it: tested with
        # four comparisons before point_in_fence builds a shapely Point
        # (~15 us; most of a 50-rule check, and twice the poses with a
        # measured velocity - 2026-10-20 review). The same answer for every
        # pose: a point on or outside the envelope is not in the interior.
        bx0, by0, bx1, by1 = (buffered.bounds if buffered is not None
                              else (-math.inf, -math.inf, math.inf, math.inf))
        # The last point found OUTSIDE the zone: a pose at exactly the same
        # point (x, y and altitude, in the zone's frame) is outside too, so it
        # is not tested again - point_in_fence is a pure function of them. A
        # stop's forecast is 51 poses at one point; inside a zone's envelope
        # but outside its ring (a diamond's corner), testing each cost about
        # 0.45 ms per zone (desktop, 2026-10-21; 8 ms a stop query with 20 such
        # zones), and the hold-point search makes a stop query per candidate
        # (guardrail/holdpoint.py "THE SEARCH'S BUDGET"). Same answers: the
        # time mask is still read per pose, above.
        miss = None
        for t, p in (poses if poses is not None else self._predict(state, action)):
            if alive is not True and not alive(t):
                continue
            px, py = _to_zone_frame(p, t, mv) if mv else (p.x, p.y)
            if not (bx0 < px < bx1 and by0 < py < by1):
                continue
            here = (px, py, p.up)
            if here == miss:
                continue
            miss = here
            if point_in_fence(px, py, p.up, f, poly, buffered):
                return [Violation(
                    rule_id=f.id, category="geofence", predicted_at_s=t,
                    detail=f"predicted pos ({p.x:.1f},{p.y:.1f}) inside NFZ at t+{t:.1f}s")]
        if self.segment_check:
            ring = buffered if buffered is not None else poly.buffer(f.margin_m)
            hit = self._first_segment_hit(
                self._segment_crossing(f, ring, path, alive, mv)
                for path in self._segment_paths(state, action, poses))
            if hit is not None:
                t_in, _, (x0, y0, t0), (x1, y1, t1) = hit
                return [Violation(
                    rule_id=f.id, category="geofence", predicted_at_s=t_in,
                    detail=(f"predicted segment ({x0:.1f},{y0:.1f})->({x1:.1f},{y1:.1f}) "
                            f"crosses NFZ between t+{t0:.1f}s and t+{t1:.1f}s"))]
        return []

    # ---------------- segment check (Shield(segment_check=True)) ---------------- #
    #
    # The grant's geometric check (Safety Shield page, "Violation checking"):
    # "Position point in NFZ? Predicted trajectory segment crosses polygon /
    # corridor boundary?". The pose checks above answer the first question at
    # each forecast pose. Between two poses the aircraft covers speed x dt -
    # 1.2 m at 12 m/s on the grant's 0.1 s grid, 6 m on the sweep's 0.5 s - and
    # a margin ring thinner than that can lie wholly between them. With the
    # flag on, the straight segment joining each pair of consecutive poses is
    # tested too, in the zone's frame for a moving zone. Off by default: it
    # changes decisions (it finds violations the poses miss).
    #
    # With a measured velocity the forecast is TWO paths (`_predict` yields
    # the own-velocity pose and the commanded pose at each grid time), and a
    # segment test walks each path on its own (`_segment_paths`): consecutive
    # poses of the interleaved stream are the connectors BETWEEN the two
    # forecasts, which neither path flies. Walking them (2026-10-20 review;
    # tests/test_own_velocity_forecast.py, moving North at 4 m/s, commanded
    # East at 4 m/s) refused a 0.1 m square lying between the two forecasts
    # that neither path enters, and missed a 0.02 x 0.2 m strip the
    # own-velocity path crosses between two of its poses - the grant's
    # "trajectory segment crosses polygon" check was silently off for the
    # measured forecast. (A zone crossed within the reaction time is crossed
    # by the stop path too: it is committed, and judged from the stop point,
    # like any rule the momentum already decides - `_committed`.)

    def _segment_paths(self, state, action, poses=None) -> list:
        """The forecast paths a segment test walks, each in time order: the
        one forecast (`poses` when the caller has it) for a state without a
        measured velocity; for one with, the own-velocity path and the
        commanded path from the same position, separately - never the
        interleaved stream `_predict` yields, whose consecutive poses join
        the two forecasts. An arced action (`_arc_rate`) has the line and the
        arc of each (`_predict`, THE ARC ONLY ADDS), each walked on its own
        for the same reason."""
        if self._arc_rate(action):
            if self._velocity_of(state) is None:
                return [list(self._predict(state, action, arc_path=p)) for p in ("line", "arc")]
            pos = self._positional(state)
            return ([list(self._predict(state, action, own_only=True, arc_path=p))
                     for p in ("line", "arc")]
                    + [list(self._predict(pos, action, arc_path=p)) for p in ("line", "arc")])
        if self._velocity_of(state) is None:
            return [poses if poses is not None else list(self._predict(state, action))]
        return [list(self._predict(state, action, own_only=True)),
                list(self._predict(self._positional(state), action))]

    @staticmethod
    def _first_segment_hit(hits):
        """The earliest of the segment tests' hits ((t, ...) tuples or None),
        or None: a rule is broken if either forecast path breaks it, at the
        earlier time (`_predict`, "THE MEASURED VELOCITY ONLY ADDS")."""
        found = [h for h in hits if h is not None]
        return min(found, key=lambda h: h[0]) if found else None

    @staticmethod
    def _segment_crossing(f, ring, poses, alive, mv):
        """The first pair of consecutive forecast poses whose straight segment
        passes through the INTERIOR of `ring` (the same open set the pose
        test's `contains` reads), with the zone in force at either end and
        the segment's altitude span meeting the zone's band. Returns (entry
        time, depth, (x0, y0, t0), (x1, y1, t1)) in world coordinates, or
        None. The entry time interpolates along the segment; `depth` is how
        far inside the ring the segment's inside part reaches at its
        midpoint."""
        if ring is None or ring.is_empty:
            return None
        rx0, ry0, rx1, ry1 = ring.bounds
        lo_band, hi_band = f.altitude_floor_m, f.altitude_ceiling_m
        prev = None
        for t, p in poses:
            on = alive is True or alive(t)
            px, py = _to_zone_frame(p, t, mv) if mv else (p.x, p.y)
            if prev is not None and (prev[4] or on):
                t0, x0, y0, u0, _, wx0, wy0 = prev
                if (max(u0, p.up) >= lo_band and min(u0, p.up) <= hi_band
                        and max(x0, px) >= rx0 and min(x0, px) <= rx1
                        and max(y0, py) >= ry0 and min(y0, py) <= ry1):
                    seg = LineString([(x0, y0), (px, py)])
                    if seg.length > 0.0 and ring.relate_pattern(seg, "T********"):
                        inside = ring.intersection(seg)
                        s = min(seg.project(Point(c)) for c in shapely.get_coordinates(inside))
                        depth = max((ring.boundary.distance(part.interpolate(0.5, normalized=True))
                                     for part in shapely.get_parts(inside)
                                     if part.geom_type == "LineString"), default=0.0)
                        t_in = t0 + (t - t0) * s / seg.length
                        return t_in, depth, (wx0, wy0, t0), (p.x, p.y, t)
            prev = (t, px, py, p.up, on, p.x, p.y)
        return None

    def _corridor_tube(self, c):
        """Corridor `c`'s keep-in tube as a polygon that CONTAINS the true
        tube (points within half_width_m of the centerline): GEOS puts a round
        join's vertices on its circle, so its chords cut inside; buffering by
        half / cos(pi/64) (16 segments a quadrant) moves every chord outside.
        A segment it does not cover leaves the true tube; one that leaves the
        true tube by less than 0.12 % of the half-width is not seen. Built
        once per rule object."""
        key = id(c)
        hit = self._tubes.get(key)
        if hit is None or hit[0] is not c:
            tube = LineString(c.points()).buffer(c.half_width_m / math.cos(math.pi / 64.0),
                                                 quad_segs=16)
            hit = self._tubes[key] = (c, tube)
        return hit[1]

    def _corridor_segment_exit(self, c, poses, alive):
        """The first pair of consecutive forecast poses (the rule in force at
        either end) whose straight segment leaves corridor `c`'s tube - two
        poses inside it on either side of a bend, the line between them
        through open air. Called after the pose checks found every pose
        inside. (exit time, (x0, y0, t0), (x1, y1, t1)) or None."""
        tube = self._corridor_tube(c)
        prev = None
        for t, p in poses:
            on = alive is True or alive(t)
            if prev is not None and (prev[3] or on):
                t0, x0, y0, _ = prev
                seg = LineString([(x0, y0), (p.x, p.y)])
                if seg.length > 0.0 and not tube.covers(seg):
                    out = seg.difference(tube)
                    s = min(seg.project(Point(q)) for q in shapely.get_coordinates(out))
                    return t0 + (t - t0) * s / seg.length, (x0, y0, t0), (p.x, p.y, t)
            prev = (t, p.x, p.y, on)
        return None

    def _corridor_geom(self, c, x: float, y: float):
        """(lateral distance from centerline, unit vector pointing back to it)."""
        d, nx, ny = nearest_on_polyline(x, y, c.points())
        if d < 1e-9:
            return d, (0.0, 0.0)          # on the line; no defined "back"
        return d, ((nx - x) / d, (ny - y) / d)

    def _check_corridor(self, c, state: State, action: Action4D,
                        poses=None) -> list[Violation]:
        """Keep-IN, trend-aware. The mirror image of _check_fence.

        `poses` is the forecast to walk when the caller has it for `state`
        (_check_in passes the shaped escape, _shaped_escape); by default, and
        whenever the rule is judged from another state (the stop point,
        `_rule_state`), `action`'s straight forecast from there.

        A fence is violated by being inside it; a corridor by being outside. The
        trend logic is the same in both: already out but actively coming back
        PASSES, because raising a violation on the recovery action is what
        freezes an aircraft in the state the rule forbids. That reasoning cost
        this project several days on the clearance ring and is not re-derived
        per rule type.

        Lateral position and altitude are judged together but recovered
        separately: being wide of the centerline says nothing about whether the
        climb is right, so an action returning laterally while still sinking
        below the floor is not yet a recovery.
        """
        alive = self._mask(c)
        if alive is False:
            return []
        st = self._rule_state(c, state)         # the stop point, if committed
        if st is not state:
            state, poses = st, None
        pts = c.points()
        half = c.half_width_m
        d_now, (ux, uy) = self._corridor_geom(c, state.x, state.y)
        wide = corridor_wide(c, d_now)
        below, above = corridor_band_outside(c, state.up)

        if (alive is True or alive(0.0)) and (wide or below or above):
            bad = []
            if wide and (action.vx * ux + action.vy * uy) <= 0.1:
                bad.append(f"{d_now:.1f}m off centerline (half-width {half:.1f}m), "
                           f"not returning")
            if below and action.vz_up <= 1e-9:
                bad.append(f"alt {state.up:.1f}m < floor {c.altitude_floor_m}m, "
                           f"not climbing")
            if above and action.vz_up >= -1e-9:
                bad.append(f"alt {state.up:.1f}m > ceiling {c.altitude_ceiling_m}m, "
                           f"not descending")
            if bad:
                return [Violation(rule_id=c.id, category="corridor",
                                  predicted_at_s=0.0,
                                  detail="outside corridor: " + "; ".join(bad))]
            return []          # coming back on every axis that is wrong

        for t, p in (poses if poses is not None else self._predict(state, action)):
            if alive is not True and not alive(t):
                continue
            d, _, _ = nearest_on_polyline(p.x, p.y, pts)
            if corridor_wide(c, d):
                return [Violation(
                    rule_id=c.id, category="corridor", predicted_at_s=t,
                    detail=(f"predicted {d:.1f}m off centerline at t+{t:.1f}s, "
                            f"half-width {half:.1f}m"))]
            if not corridor_in_band(c, p.up):
                return [Violation(
                    rule_id=c.id, category="corridor", predicted_at_s=t,
                    detail=(f"predicted alt {p.up:.1f}m outside corridor band "
                            f"[{c.altitude_floor_m}, {c.altitude_ceiling_m}] "
                            f"at t+{t:.1f}s"))]
        if self.segment_check:
            hit = self._first_segment_hit(
                self._corridor_segment_exit(c, path, alive)
                for path in self._segment_paths(state, action, poses))
            if hit is not None:
                t_out, (x0, y0, t0), (x1, y1, t1) = hit
                return [Violation(
                    rule_id=c.id, category="corridor", predicted_at_s=t_out,
                    detail=(f"predicted segment ({x0:.1f},{y0:.1f})->({x1:.1f},{y1:.1f}) "
                            f"leaves the corridor between t+{t0:.1f}s and t+{t1:.1f}s, "
                            f"half-width {half:.1f}m"))]
        return []

    def _clear_forecast(self, state: State, action: Action4D, f=None):
        """(steps, dists) over the clearance horizon.

        Shared so a repair that has already paid for the forecast does not pay
        again, once per rule per repair pass. `f`: the field (_distance_at).
        """
        steps = self._clear_steps(state, action)
        return steps, [self._distance_at(p.x, p.y, f) for _, p in steps]

    def _check_clearance(self, c, state: State, action: Action4D,
                         steps=None, dists=None) -> list[Violation]:
        """Trend-aware, same shape as the geofence rule. Measured on the
        rule's own map (`_field_for`: buildings, or roads for a
        distance_envelope over roads)."""
        f = self._field_for(c)
        if f is None:
            return []
        alive = self._mask(c)
        if alive is False:
            return []
        if steps is None or dists is None:
            # A caller that passes the forecast passes it for the state the
            # rule is judged from (_check_in, _repair_clearance).
            state = self._rule_state(c, state)
            steps, dists = self._clear_forecast(state, action, f)
        if self._velocity_of(state) is not None:
            # THE MEASURED VELOCITY ONLY ADDS (_predict): judged on the
            # own-velocity "stop after one control period" forecast AND, as
            # before the change, on the commanded one from the same position.
            # Separately, because the test below reads its forecast in order
            # (receding, forgiving); the earlier breach is the answer.
            own = self._clearance_verdict(c, steps, dists, alive)
            old = self._check_clearance(c, self._positional(state), action)
            if own and old:
                return own if own[0].predicted_at_s <= old[0].predicted_at_s else old
            return own or old
        line = self._clearance_verdict(c, steps, dists, alive)
        arc = self._clear_arc_steps(state, action)
        if arc is None:
            return line
        # THE ARC ONLY ADDS (_predict): an arced action is judged on its arc
        # as well as on the line, each in order; the earlier breach is the
        # answer, the line's on a tie.
        av = self._clearance_verdict(c, arc, [self._distance_at(p.x, p.y, f) for _, p in arc],
                                     alive)
        if line and av:
            return line if line[0].predicted_at_s <= av[0].predicted_at_s else av
        return line or av

    def _clearance_verdict(self, c, steps, dists, alive) -> list[Violation]:
        """_check_clearance's test on one forecast (steps, dists)."""
        if alive is not True:
            # Only the samples at which the rule is in force are judged.
            keep = [i for i, (t, _) in enumerate(steps) if alive(t)]
            if not keep:
                return []
            steps, dists = [steps[i] for i in keep], [dists[i] for i in keep]
        d_now = dists[0]
        # "moving away" = distance grows over the FIRST forecast step
        receding = len(dists) > 1 and dists[1] > d_now + 1e-6
        # Being inside the ring is forgiven only while the forecast KEEPS
        # improving. The old code dropped the ENTIRE horizon on the strength of
        # one 0.5 s step, so a drone anywhere inside the ring got a free pass on
        # every obstacle on the map - including forecasts that ended up inside a
        # building.
        forgiving = d_now < c.min_clearance_m and receding
        prev = None
        for (t, p), d in zip(steps, dists):
            if d >= c.min_clearance_m:
                forgiving = False         # out of the ring: normal rules again
                prev = d
                continue
            if forgiving and (prev is None or d > prev + 1e-6):
                prev = d
                continue                  # still actively escaping -> never freeze
            return [Violation(
                rule_id=c.id, category="clearance", predicted_at_s=t,
                detail=(f"obstacle dist {d:.2f}m < min {c.min_clearance_m}m "
                        f"at ({p.x:.1f},{p.y:.1f}) t+{t:.1f}s"))]
        return []

    def _check(self, state: State, action: Action4D) -> list[Violation]:
        """Every rule `action` violates from `state`, earliest per rule.

        Outside a filter() tick, a policy whose rules depend on the clock or
        move gets a tick view for this one call (the clock is read once), and
        so does a state with a measured velocity: its rules share one answer
        to `_committed` (two checks of a stop), one pipe and one set of poses
        per action, which the tick caches. Without the tick each rule asked
        `_committed` afresh: at 50 rules on the packed flights (tools/
        profile_shield_tick.py, arm design_load_velocity, horizon grant,
        1301 ticks) the monitor query with a velocity took median 4.8 /
        p99 18.4 / max 31 ms in the first measurement and 4.19 / 11.8 /
        17.7 ms re-measured pinned to the performance cores (359 ticks over
        the 5 ms budget); with the tick view 0.87 / 1.21 / 1.48 ms
        (docs/DESIGN-own-velocity-forecast.md, "The cost"). An earlier
        version of this docstring said p99 13.9 ms, a figure no recorded
        run gives."""
        if self._tick_ctx() is None and (self._needs_tick()
                                         or self._velocity_of(state) is not None):
            with self._scope(self._new_tick(None)):
                return self._check_in(state, action)
        return self._check_in(state, action)

    def _first_in_force(self, rule) -> float:
        tick = self._tick_ctx()
        t0 = tick.sched.first(rule) if tick is not None else 0.0
        return 0.0 if t0 is None else t0

    @staticmethod
    def _standoff_inside(so, d: float) -> bool:
        """Is range `d` inside `so`'s ring? The one comparison the stand-off
        check, its recovery and rule_status share."""
        return d < so.min_range_m - 1e-9

    def _check_in(self, state: State, action: Action4D) -> list[Violation]:
        found: list[Violation] = []
        for k in self._kins:
            vs = self._check_kinematic(k, action)
            if vs and self._mask(k) is not True:
                # A cap that comes into force later in the lookahead is broken
                # then, not now (a constant-velocity forecast keeps the speed).
                t0 = self._first_in_force(k)
                vs = [v.model_copy(update={"predicted_at_s": t0}) for v in vs]
            found += vs
        for env in self._alts:
            found += self._check_altitude(env, state, action)
        if self._subject is not None:
            for so in self._standoffs:
                found += self._check_standoff(so, state, action)
        ir = self._ir                       # one IR for the whole check, even
        recs, tick = self._fence_anchors(ir)    # if hot_apply swaps it meanwhile
        # An escape from a zone whose window opened on the aircraft is judged
        # as it is flown, out of the ring and then a stop (_shaped_escape says
        # why); None almost always, and every rule walks the straight forecast.
        shaped = self._shaped_escape(state, action)
        if recs:
            # One forecast for every fence (it was rebuilt per fence), and only
            # the fences the forecast's bounding box reaches. Policy order is
            # kept, so the violation list - which test_check_contract.py hashes
            # - comes out in the same order as the old walk over all of them.
            poses = shaped if shaped is not None else list(self._predict(state, action))
            com = self._committed(state)
            if com is not None and (com[1] or com[3]):
                # Some rule is judged from the stop point or from the position
                # without the velocity (_check_fence recomputes that state's
                # forecast): the candidate fences and the box cover every
                # forecast in play, so such a zone is not skipped for lying
                # off the own-velocity one.
                both = list(poses)
                if com[1]:
                    both += list(self._predict(com[0], action))
                if com[3]:
                    both += list(self._predict(com[2], action))
                near = self._fence_candidates(ir, state, both)
                box, horizon = self._forecast_box(state, both)
            else:
                near = self._fence_candidates(ir, state, poses)
                box, horizon = self._forecast_box(state, poses)
            # The static zones come from the IR's R-tree (`near`), the moving
            # ones from this tick's (`moving`, None: every one is looked at).
            moving = self._moving_near(ir, tick, box, horizon)
            for r in recs:
                if r.index not in near:
                    continue
                if r.moving and moving is not None and r.index not in moving:
                    continue
                r = self._reach(r, tick, box, horizon)
                if r is not None:
                    found += self._check_fence(r.rule, r.polygon, state, action,
                                               poses, r.buffered)
        for c in self._corridors:
            found += self._check_corridor(c, state, action, shaped)
        if self._clear:
            # One forecast shared across every clearance rule of one map, as
            # before - one per state judged from, when a rule is judged from
            # the stop point (_committed). A rule whose map was not given is
            # inert (obstacle_clearance; a distance_envelope never gets here
            # without its map, see __init__).
            fc: dict = {}
            other = None
            for c in self._clear:
                f = self._field_for(c)
                if f is None:
                    continue
                key = _field_name(c)
                st = self._rule_state(c, state)
                if st is state:
                    if key not in fc:
                        fc[key] = self._clear_forecast(state, action, f)
                    found += self._check_clearance(c, state, action, *fc[key])
                else:
                    if other is None or other[0] is not st or other[1] != key:
                        other = (st, key, *self._clear_forecast(st, action, f))
                    found += self._check_clearance(c, st, action, other[2], other[3])
        com = self._committed(state)
        if com is not None and com[1]:
            # THE MEASURED VELOCITY ONLY ADDS (_predict), for a rule judged
            # from the stop point too: it is also judged as before the change,
            # from the position on the commanded forecast. A stop and a push
            # away pass that test from a legal position, so the decision keeps
            # its clean fallback; what the old Shield refused stays refused.
            found += [v for v in self._check_in(com[2], action) if v.rule_id in com[1]]

        # dedupe by (rule, category), keep earliest
        seen: dict[tuple, Violation] = {}
        for v in found:
            key = (v.rule_id, v.category)
            if key not in seen or v.predicted_at_s < seen[key].predicted_at_s:
                seen[key] = v
        return list(seen.values())

    # ---------------- repair operators ---------------- #

    def _repair_kinematic(self, a: Action4D, repairs: list[Repair]) -> Action4D:
        # The same 1e-9 tolerance as _check_kinematic. Until 2026-10-12 these
        # tests were strict, so an action another operator had already scaled
        # to the cap, and that came out 4.000000000000001 m/s, was "clamped"
        # to 4.0 and logged as a SpeedClamp repair the check would never have
        # flagged. Whether that happened depended on the platform's floating
        # point: on Windows nfz-head-on logged 82 such repairs, on Linux (the
        # CI runners) none, which is why tests/test_sweep.py's pinned repair
        # counts failed on GitHub only. Since 2026-11-07 the tolerance is the
        # float32 contract's slack, fsm.cap_exceeded, in the check and here.
        vx, vy, vz, yr = a.vx, a.vy, a.vz_up, a.yaw_rate
        for k in self._kins:
            h = (vx ** 2 + vy ** 2) ** 0.5
            if _fsm.cap_exceeded(h, k.speed_max_mps):
                s = k.speed_max_mps / h
                vx, vy = vx * s, vy * s
                repairs.append(Repair(operator="SpeedClamp",
                                      detail=f"h-speed {h:.2f} -> {k.speed_max_mps}"))
            if _fsm.cap_exceeded(abs(vz), k.climb_rate_max_mps):
                new = k.climb_rate_max_mps * (1 if vz > 0 else -1)
                repairs.append(Repair(operator="ClimbClamp", detail=f"vz {vz:.2f} -> {new:.2f}"))
                vz = new
            ymax = math.radians(k.yaw_rate_max_dps)
            if _fsm.cap_exceeded(math.degrees(abs(yr)), k.yaw_rate_max_dps):
                new = ymax * (1 if yr > 0 else -1)
                repairs.append(Repair(
                    operator="YawClamp",
                    detail=f"yaw {math.degrees(yr):.1f} -> {math.degrees(new):.1f} dps"))
                yr = new
        return Action4D(vx=vx, vy=vy, vz_up=vz, yaw_rate=yr)

    @staticmethod
    def _reentry_margin(lo: float, hi: float) -> float:
        """How far INSIDE the band a recovery should aim.

        Aiming at the boundary itself makes the recovery a decaying exponential
        that converges to the edge and never crosses it. Measured on
        `sim_demo_policy` with a 10 m floor: from 3 m the vehicle reaches
        9.10 m in 6 s, 9.88 m in 12 s and 9.99973 m after thirty seconds -
        still below the floor, still in an unsafe position, and it would stay
        there for any length of flight.

        The Shield reported this as healthy the whole time, and by its own
        contract it was: the emitted action climbs, so there is no illegal
        action and the P0 escape rate is 0. What was wrong was the STATE, which
        is exactly the gap `mean time to safe` was added to see - and this is
        the defect the first scenario sweep found.
        """
        return min(1.0, (hi - lo) / 4.0)

    def _recovery_vz(self, up: float, lo: float, hi: float, L: float) -> float:
        """The vertical rate a recovery from OUTSIDE the band [lo, hi] flies:
        the smallest climb cap among the kinematic rules in force, held to
        the rate whose lookahead endpoint stays a re-entry margin short of
        the far edge, and never slower than reaching the aim (a margin
        inside the near edge) in one lookahead.

        Until 2026-10-29 the rate was the last of these alone, (aim - up) /
        L. That makes every recovery a P controller with time constant L:
        from D metres short of the aim it covers the last 2 L metres in
        L ln(2 L) seconds, 11.5 s of the 11.9 s the 30 m cell of
        altitude-recovery-depth took at the grant's 5 s lookahead, against
        about 5.5 s at the 2 m/s cap (experiments/templates/
        altitude_envelope.yaml). A longer forecast then recovered more
        slowly, which no rule asks for. With no kinematic rule there is no
        cap to fly at and the one-lookahead rate stays."""
        m = self._reentry_margin(lo, hi)
        caps = [k.climb_rate_max_mps for k in self._kins]
        if up < lo:
            law = (lo + m - up) / L
            return law if not caps else max(law, min(min(caps), (hi - m - up) / L))
        law = (hi - m - up) / L
        return law if not caps else min(law, max(-min(caps), (lo + m - up) / L))

    def _repair_altitude(self, state: State, a: Action4D, repairs: list[Repair]) -> Action4D:
        """Project vz so the lookahead endpoint lands inside the band.

        Two cases that look alike and are not:

          * ALREADY outside - recover at the climb cap (_recovery_vz), aiming
            a margin INSIDE the band so the recovery actually arrives (see
            _reentry_margin). A command already recovering faster flies as
            it is; a slower one, even one whose endpoint is legal, is made
            to recover at the cap - a repair that only ever leaves faster.
          * inside and about to overshoot - aim at the boundary exactly. There
            is nothing to recover from, and pulling further in would fight a
            legal cruise that happens to sit near the edge of its own envelope.

        recovery_law="lookahead" runs _repair_altitude_lookahead instead.
        """
        if self.recovery_law == "lookahead":
            return self._repair_altitude_lookahead(state, a, repairs)
        vz = a.vz_up
        L = self.lookahead_s
        for env in self._alts:
            end_up = state.up + vz * L
            above, below = state.up > env.alt_max_m, state.up < env.alt_min_m
            if above or below:
                rec = self._recovery_vz(state.up, env.alt_min_m, env.alt_max_m, L)
                # At least `rec` towards the band, and never so fast that the
                # endpoint passes the far edge (that clamp is no recovery).
                if above:
                    far, new = (env.alt_min_m - state.up) / L, min(vz, rec)
                    new = max(new, far)
                else:
                    far, new = (env.alt_max_m - state.up) / L, max(vz, rec)
                    new = min(new, far)
                if abs(new - vz) > 1e-9:
                    to_far = new == far and new != rec
                    edge = (f"ceiling {env.alt_max_m}m" if above != to_far
                            else f"floor {env.alt_min_m}m")
                    repairs.append(Repair(operator="AltitudeFix",
                                          detail=f"vz {vz:.2f} -> {new:.2f} ({edge})",
                                          magnitude_m=abs(new - vz) * L, axis="vertical",
                                          recovery=not to_far))
                    vz = new
            elif end_up > env.alt_max_m:
                new = (env.alt_max_m - state.up) / L
                repairs.append(Repair(operator="AltitudeFix",
                                      detail=f"vz {vz:.2f} -> {new:.2f} (ceiling {env.alt_max_m}m)",
                                      magnitude_m=end_up - env.alt_max_m, axis="vertical",
                                      recovery=False))
                vz = new
            elif end_up < env.alt_min_m:
                new = (env.alt_min_m - state.up) / L
                repairs.append(Repair(operator="AltitudeFix",
                                      detail=f"vz {vz:.2f} -> {new:.2f} (floor {env.alt_min_m}m)",
                                      magnitude_m=env.alt_min_m - end_up, axis="vertical",
                                      recovery=False))
                vz = new
        return Action4D(vx=a.vx, vy=a.vy, vz_up=vz, yaw_rate=a.yaw_rate)

    def _repair_altitude_lookahead(self, state: State, a: Action4D,
                                   repairs: list[Repair]) -> Action4D:
        """_repair_altitude as it was until 2026-10-29 (recovery_law
        "lookahead"), kept line for line so a bundle flown with it re-flies
        bit for bit: outside the band, aim a margin inside and reach it in
        one lookahead, and only when the lookahead endpoint is outside."""
        vz = a.vz_up
        L = self.lookahead_s
        for env in self._alts:
            end_up = state.up + vz * L
            margin = self._reentry_margin(env.alt_min_m, env.alt_max_m)
            if end_up > env.alt_max_m:
                outside = state.up > env.alt_max_m
                tgt = env.alt_max_m - (margin if outside else 0.0)
                new = (tgt - state.up) / L
                repairs.append(Repair(operator="AltitudeFix",
                                      detail=f"vz {vz:.2f} -> {new:.2f} (ceiling {env.alt_max_m}m)",
                                      magnitude_m=end_up - tgt, axis="vertical",
                                      recovery=outside))
                vz = new
            elif end_up < env.alt_min_m:
                outside = state.up < env.alt_min_m
                tgt = env.alt_min_m + (margin if outside else 0.0)
                new = (tgt - state.up) / L
                repairs.append(Repair(operator="AltitudeFix",
                                      detail=f"vz {vz:.2f} -> {new:.2f} (floor {env.alt_min_m}m)",
                                      magnitude_m=tgt - end_up, axis="vertical",
                                      recovery=outside))
                vz = new
        return Action4D(vx=a.vx, vy=a.vy, vz_up=vz, yaw_rate=a.yaw_rate)

    def _repair_clearance(self, state: State, a: Action4D,
                          repairs: list[Repair]) -> Action4D:
        """Steer the HORIZONTAL velocity away from the building the FORECAST
        actually runs into.

        All three moves are driven by the local gradient of the distance field:
        cancel the velocity component pointing INTO the obstacle, taper what is
        left (the mission-progress part) by how deep the incursion is, then add
        an outward push sized to recover the shortfall within one lookahead.

        The gradient is read at the OFFENDING FORECAST POSE, not at the current
        one. When the nearest building right now is not the building the
        forecast hits, "outward" as measured here points straight AT the future
        obstacle: the into-component is negative so it is never cancelled, and
        the push then ACCELERATES the vehicle into the wall it is meant to
        avoid.

        The taper deliberately does NOT touch the push: deeper incursion means
        SLOWER progress but a FIRMER escape, never a limp one. soft_margin_m
        only widens the band the taper ramps over. It never zeroes the action —
        that is the brake's job."""
        if not self._clear:
            return a
        vx, vy = a.vx, a.vy
        cap = min((k.speed_max_mps for k in self._kins), default=4.0)

        outer = state
        for c in self._clear:
            # Each rule on its own map (_field_for: buildings, or roads for a
            # distance_envelope over roads); a rule with no map is inert.
            f = self._field_for(c)
            if f is None:
                continue
            # Judged, and repaired, from the stop point when the aircraft's
            # momentum already carries it into this ring (_committed): the
            # push-out then starts before the aircraft reaches the ring.
            state = self._rule_state(c, outer)
            hard_r = c.min_clearance_m
            soft_r = c.min_clearance_m + c.soft_margin_m
            probe = Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
            steps, dists = self._clear_forecast(state, probe, f)
            alive_c = self._mask(c)
            if (alive_c is not False and self._velocity_of(state) is not None
                    and not self._clearance_verdict(c, steps, dists, alive_c)
                    and self._check_clearance(c, self._positional(state), probe)):
                # THE MEASURED VELOCITY ONLY ADDS (_predict, _check_clearance):
                # the own-velocity forecast is content and the commanded one
                # from the position is not - a command towards the building
                # the aircraft is too slow to have taken up yet. Repaired from
                # the position on the commanded forecast, exactly as before the
                # change; the forecast the repair steers by must be the one
                # that breaks (the first offending pose below).
                state = self._positional(state)
                steps, dists = self._clear_forecast(state, probe, f)

            # Ask the MONITOR, against the mutating vector, rather than
            # re-deriving a weaker condition here. The two had drifted: the
            # monitor forgives an action that keeps improving the forecast,
            # this loop fired on `d_min < hard_r` alone. So an aircraft flying
            # straight away from a wall at 5.00 m/s passed through untouched
            # while 5.01 - two tenths of a percent over the speed cap, enough
            # to drag it into the repair loop - came out at 3.57 m/s with a
            # sideways component it never asked for.
            content = not self._check_clearance(c, state, probe, steps, dists)
            d_min = min(dists)
            if content and d_min >= hard_r:
                continue                       # clear of the ring; nothing to do
            if content:
                # Inside the ring and the monitor is satisfied - but its test is
                # `d > prev + 1e-6` per forecast sample, an epsilon rather than a
                # rate, so a two-centimetre-per-second creep past a building
                # counts as escaping. Declining outright here would leave that
                # crawl in place. Raise the outward component to the recovery
                # speed and leave the tangential alone; never reshape an action
                # the monitor is happy with. (Until 2026-10-19 the passthrough
                # declined it for this branch whenever no other rule was broken:
                # the chain never ran. Shield._floor_the_escape runs it now.)
                oxc, oyc = self._away_dir(state.x, state.y, hard_r, f)
                if oxc == 0.0 and oyc == 0.0:
                    continue
                out_have = vx * oxc + vy * oyc
                need = min(cap, max(0.5, (hard_r - d_min) / max(self.lookahead_s, 1e-6)))
                if out_have < need:
                    vx += oxc * (need - out_have)
                    vy += oyc * (need - out_have)
                    repairs.append(Repair(
                        operator="ClearanceFix",
                        detail=(f"{c.id}: inside {hard_r}m and leaving at "
                                f"{out_have:.2f} m/s -> raised to {need:.2f}"),
                        magnitude_m=max(hard_r - d_min, 0.0), axis="lateral",
                        recovery=True))
                continue

            # the FIRST offending pose is the obstacle we have to steer off; at
            # t = 0 that is the current position, which is the old behaviour.
            hit = steps[next(i for i, d in enumerate(dists) if d < hard_r)][1]
            ox, oy = self._away_dir(hit.x, hit.y, hard_r, f)
            if ox == 0.0 and oy == 0.0:
                ox, oy = self._away_dir(state.x, state.y, hard_r, f)
            if ox == 0.0 and oy == 0.0:
                continue                       # nothing to steer by; rescue decides

            # `before` measured the SAME way as `after`.
            #
            # It used to be `min(dists)`, which includes t = 0, while `after`
            # comes from _clear_min_dist, which excludes it. At (38, 20.5)
            # flying away at 5 m/s that is 0.886 against 3.696 - a 4.2x gap, so
            # `after > before` held automatically and the "never leave it
            # worse" invariant below was suppressed exactly where it mattered.
            before = self._clear_min_dist(state, probe, f)

            out_c = vx * ox + vy * oy          # outward component already commanded
            tx, ty = vx - out_c * ox, vy - out_c * oy      # tangential = mission part
            depth = hard_r - d_min             # how far inside the ring we are
            # taper the tangential (mission) motion, floored so we still move
            scale = max(_CLEAR_MIN_SCALE, 1.0 - depth / max(soft_r, 1e-6))
            push = min(cap, max(0.5, depth / max(self.lookahead_s, 1e-6)))
            # FLOOR, not assign: never slow an escape that is already faster
            # than the recovery this repair would have sized.
            out_n = min(cap, max(out_c, push))
            # The cap eats the TANGENTIAL first - scaling the whole vector
            # would undo the push that was just computed.
            room = math.sqrt(max(cap * cap - out_n * out_n, 0.0))
            t_mag = math.hypot(tx, ty)
            t_scale = min(scale, room / t_mag) if t_mag > 1e-9 else 0.0
            cx = tx * t_scale + ox * out_n
            cy = ty * t_scale + oy * out_n

            # INVARIANT: a repair must never leave the forecast worse than it
            # found it. Where the local gradient is a poor guide (corners, two
            # buildings in play) fall back to the direction search rather than
            # shipping a "fix" that flies further in.
            cand = Action4D(vx=cx, vy=cy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
            after = self._clear_min_dist(state, cand, f)
            if after <= before + 1e-9:
                esc = self._best_clear_dir(state, probe, cap, f)
                esc_d = self._clear_min_dist(state, esc, f) if esc is not None else None
                if esc_d is not None and esc_d > max(after, before):
                    cx, cy, after = esc.vx, esc.vy, esc_d
                elif before >= after:
                    # Nothing on offer improves on the action we were handed, so
                    # ship THAT rather than a "fix" that scores worse. `before`
                    # is by definition its score, which makes this the monotone
                    # answer instead of a guess.
                    cx, cy, after = vx, vy, before
            vx, vy = cx, cy
            repairs.append(Repair(
                operator="ClearanceFix",
                detail=(f"{c.id}: dist {d_min:.2f}m < {hard_r}m -> push "
                        f"({ox:+.2f},{oy:+.2f}) at {out_n:.2f} m/s "
                        f"(commanded {out_c:+.2f}, recovery needs {push:.2f}), "
                        f"tangential x{t_scale:.2f}, forecast {before:.2f}->{after:.2f}m"),
                # The forecast's deepest incursion into the ring; a recovery
                # when the aircraft is inside the ring already.
                magnitude_m=max(depth, 0.0), axis="lateral",
                recovery=dists[0] < hard_r))
        return Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)

    def _repair_geofence(self, state: State, a: Action4D, repairs: list[Repair]) -> Action4D:
        """Two modes:
        - INSIDE a zone  -> GeofenceEscape: fly straight out (recovery).
        - heading INTO a zone -> GeofenceSlide: cancel the into-zone velocity
          component, keep/add a tangent one so the mission keeps moving along
          the edge instead of stalling (anti-stall bias)."""
        vx, vy = a.vx, a.vy
        cap = min((k.speed_max_mps for k in self._kins), default=4.0)

        # The fences the CURRENT (vx, vy) can reach, recomputed whenever an
        # earlier fence in the loop has changed the velocity. A single query up
        # front would be wrong: an escape or slide off one fence can point the
        # forecast at a fence the raw action never reached, and the old walk
        # over every fence would then have repaired against it. Skipping a
        # fence outside the current forecast's box is exactly the old loop's
        # `if not hit: continue`, decided without the per-pose tests.
        ir = self._ir
        near_for = poses = near = box = horizon = moving = None
        # A zone the aircraft's momentum already carries it into is repaired
        # from the stop point, on that state's own forecast (_committed); it
        # is relevant by construction, so the candidate test is not asked.
        com = self._committed(state)
        outer = state
        anchors, tick = self._fence_anchors(ir)
        for rec in anchors:
            if near_for != (vx, vy):
                probe = Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
                # The forecast the check walks (_check_in): an escape from a
                # zone that opened on the aircraft as flown, out of the ring
                # and then a stop (_shaped_escape), so the slide below acts on
                # what the check raises - never on a zone only the straight
                # forecast reaches past the stop.
                poses = self._shaped_escape(outer, probe)
                if poses is None:
                    poses = list(self._predict(outer, probe))
                near = self._fence_candidates(ir, outer, poses)
                box, horizon = self._forecast_box(outer, poses)
                moving = self._moving_near(ir, tick, box, horizon)   # this tick's R-tree
                near_for = (vx, vy)
            st = self._rule_state(rec.rule, outer) if com is not None else outer
            if st is not outer:
                state = st
                probe = Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
                fposes = list(self._predict(state, probe))
                rec = self._reach(rec, tick, *self._forecast_box(state, fposes))
            else:
                state, fposes = outer, poses
                if rec.index not in near:
                    continue
                if rec.moving and moving is not None and rec.index not in moving:
                    continue
                rec = self._reach(rec, tick, box, horizon)
            if rec is None:
                continue
            f, poly, ring = rec.rule, rec.polygon, rec.buffered
            # A moving zone: its velocity, turn rate and centre at this tick.
            # Forecast poses are tested in its frame, and "into" / "out of" it
            # are measured relative to its velocity.
            mv = self._motion_of(f)
            zx, zy = (mv[0], mv[1]) if mv else (0.0, 0.0)
            if point_in_fence(state.x, state.y, state.up, f, poly, ring):
                # The way out the check reads (_check_fence): an exact tie
                # between equally near edges goes to the one the current
                # command is heading for (escape_direction).
                ox, oy = escape_direction(state.x, state.y, poly, vx, vy)
                # A window opening on the aircraft sooner than it can leave
                # (_opening_imposed): the way out is read where the current
                # velocity puts it when the zone comes into force, as
                # _check_fence reads it - from the zone's middle the current
                # position's way out is one arbitrary side of a tie, and
                # flooring towards it reversed a 4 m/s escape the other way
                # into a 2 m/s one.
                t_open = None if mv else self._opening_imposed(f, ring, state)
                if t_open is not None:
                    ox, oy = escape_direction(state.x + vx * t_open,
                                              state.y + vy * t_open, poly, vx, vy)
                # ... and its escape leaves at the CAP, not the in-force 2.0
                # m/s floor below: before the opening (t_open) and after it,
                # until the aircraft is out of the ring (`_opened_on`, kept by
                # filter()). Review of 3660383, P1: with C2's 2.0 floor a
                # 1.5 m/s pilot took 5.1 s to safe from the library cells
                # and stayed 26 / 31 / 36 ticks inside the polygon, where
                # 7c9c9e6 - every tick there escalated, the FSM's hold transit
                # at the 4 m/s cap - took 2.6 s and 3 / 8 / 13, at any pilot
                # speed; the KPI read the longer stay as no escape. Not for a
                # moving zone, which _opening_imposed never proves.
                #
                # At the cap WHERE IT CAN BE FLOWN: a faster escape stops
                # further past the exit, and the cap's stop can reach another
                # hard rule the aircraft's own crawl does not. Then
                # _floor_the_escape asks again with a slower one
                # (`tick.opened_mps`, its retries: review of b1d3aca, P1),
                # down to the in-force floor and that floor's own shape
                # (_IN_FORCE_FLOOR), and escalates if none can be flown. The
                # escape is judged as flown, out of the ring and then a stop
                # (_shaped_escape; review of 69c27bd, P2), not on the cap held
                # for the whole lookahead: that 20 m reached the library's
                # own corridor edge, 9 m past the school zone's ring.
                #
                # Under recovery_law "cap_all" (2026-11-06) the same holds for
                # every static zone the aircraft is inside (`_cap_zones_at`),
                # whatever put it there: the recovery flies at the cap, judged
                # as flown, slower only where the cap's stop breaks another
                # rule - as the altitude recovery climbs at the climb cap since
                # 2026-10-29. A moving zone keeps the floor below (the shaped
                # escape is not defined for a moving ring).
                tick_now = self._tick_ctx()
                try_mps = getattr(tick_now, "opened_mps", None)
                at_cap = (not mv and tick_now is not None
                          and f.id in self._cap_zones_at(tick_now, state)
                          and t_open is None and f.id not in self._opened_on)
                opened = ((t_open is not None or (not mv and f.id in self._opened_on)
                           or at_cap)
                          and try_mps != _IN_FORCE_FLOOR)
                why_cap = ("inside the zone (recovery at the cap)" if at_cap
                           else "window opened on the aircraft")
                if opened and try_mps is not None:
                    cap_out = min(float(try_mps), cap)
                else:
                    cap_out = cap
                depth = ring.boundary.distance(Point(state.x, state.y))
                # FLOOR the exit speed, never assign it.
                #
                # `spd` stays min(2.0, cap): 2.0 is the reference
                # implementation's `escape_speed_mps` default, chosen because
                # GeofenceEscape is a recovery operator exempt from the
                # magnitude cap and should leave at a moderate, envelope-safe
                # speed rather than bolt down an unvalidated straight line.
                #
                # What changed is that it was ASSIGNED over the whole
                # horizontal vector. An aircraft 1 m inside the boundary
                # already leaving at 4.00 m/s passed through untouched, while
                # 4.01 - a quarter of a percent over the speed cap, enough to
                # drag it into the repair loop - was slowed to 2.00 and took
                # twice as long to get out of a P0 zone.
                #
                # Note this is evaluated against the MUTATING (vx, vy): with
                # two overlapping fences the second must see what the first
                # left behind, or the last one silently wins.
                # Against a zone that follows the aircraft outward, the exit
                # speed is raised by the zone's own outward speed.
                spd = cap_out if opened else min(2.0 + max(0.0, zx * ox + zy * oy), cap)
                out_now = vx * ox + vy * oy          # outward speed commanded
                probe = Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
                if opened and not self._check_fence(f, poly, state, probe, fposes, ring):
                    # Escaping, from a zone that opened on the aircraft: the
                    # way out at the full cap. The cap eats the TANGENTIAL
                    # first (as ClearanceFix's push does): raising only the
                    # outward part would leave SpeedClamp to scale the whole
                    # vector back, and 1.5 m/s out with 1.0 across would
                    # leave at 3.3 m/s, not 4. So this one does reshape a
                    # legal action: the mission's sideways progress waits
                    # until the aircraft is out of a P0 zone it never entered.
                    out_n = max(out_now, spd)
                    tx, ty = vx - out_now * ox, vy - out_now * oy
                    t_mag = math.hypot(tx, ty)
                    room = math.sqrt(max(cap * cap - out_n * out_n, 0.0))
                    t_scale = min(1.0, room / t_mag) if t_mag > 1e-9 else 0.0
                    nvx, nvy = tx * t_scale + ox * out_n, ty * t_scale + oy * out_n
                    if abs(nvx - vx) > 1e-9 or abs(nvy - vy) > 1e-9:
                        vx, vy = nvx, nvy
                        repairs.append(Repair(
                            operator="GeofenceEscape",
                            detail=(f"{f.id}: {why_cap}, leaving "
                                    f"at {out_now:.2f} m/s -> straight out at the "
                                    f"{cap:.1f} m/s cap" if spd >= cap - 1e-9 else
                                    f"{f.id}: {why_cap}, leaving "
                                    f"at {out_now:.2f} m/s -> out at {spd:.2f} m/s, "
                                    f"under the {cap:.1f} m/s cap"),
                            magnitude_m=depth, axis="lateral", recovery=True))
                    continue
                if not self._check_fence(f, poly, state, probe, fposes, ring):
                    # The monitor is content, so this action IS escaping - but
                    # it forgives anything above 0.1 m/s, which would leave a
                    # crawl. Raise the outward component to the floor and leave
                    # the tangential alone rather than reshaping a legal action.
                    # Since 2026-10-19 this branch also runs when nothing at all
                    # is violated (Shield._floor_the_escape); before, a 0.15 m/s
                    # exit passed untouched unless another rule happened to be
                    # broken, and left at 2.0 m/s if one was.
                    if out_now < spd:
                        vx += ox * (spd - out_now)
                        vy += oy * (spd - out_now)
                        repairs.append(Repair(
                            operator="GeofenceEscape",
                            detail=(f"{f.id}: inside and leaving at "
                                    f"{out_now:.2f} m/s -> raised to {spd:.1f}"),
                            magnitude_m=depth, axis="lateral", recovery=True))
                    continue
                keep = min(cap, max(out_now, spd))
                vx, vy = ox * keep, oy * keep
                repairs.append(Repair(operator="GeofenceEscape",
                                      detail=(f"{f.id}: inside -> exit at {keep:.1f} m/s "
                                              f"(commanded {out_now:+.2f})"),
                                      magnitude_m=depth, axis="lateral", recovery=True))
                continue

            # `fposes` is the forecast of the current (vx, vy) from the state
            # this zone is judged from: rebuilt above the moment the velocity
            # last changed (`poses`), or for this zone's stop point.
            alive = self._mask(f)
            depth = 0.0
            hit = False
            # The envelope pre-test of _check_fence: the same answer per pose.
            bx0, by0, bx1, by1 = ring.bounds
            for t, p in fposes:
                if alive is not True and not alive(t):
                    continue
                px, py = _to_zone_frame(p, t, mv) if mv else (p.x, p.y)
                if not (bx0 < px < bx1 and by0 < py < by1):
                    continue
                if point_in_fence(px, py, p.up, f, poly, ring):
                    hit = True
                    # Penetration depth of the forecast being repaired: how
                    # far inside the ring its deepest pose lies (the grant's
                    # magnitude_m for a lateral projection).
                    depth = max(depth, ring.boundary.distance(Point(px, py)))
            if not hit and self.segment_check:
                # A ring that lies wholly between two poses: the slide acts on
                # it as on a pose inside, sized by the segment's own depth -
                # each forecast path walked on its own (`_segment_paths`), the
                # deeper crossing when both paths cross.
                segs = [s for s in (
                    self._segment_crossing(f, ring, path, alive, mv)
                    for path in self._segment_paths(
                        state, Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate),
                        fposes)) if s is not None]
                if segs:
                    hit, depth = True, max(s[1] for s in segs)
            if not hit:
                continue

            # unit "away from zone"; a tie between equally near edges is
            # broken by the commanded heading (escape_direction)
            ox, oy = escape_direction(state.x, state.y, poly, vx, vy)
            into = -((vx - zx) * ox + (vy - zy) * oy)             # speed INTO the zone
            if into > 0:
                vx += ox * into                                   # cancel it
                vy += oy * into
                # anti-stall: if nearly nothing is left (head-on approach),
                # push along the zone edge instead of stopping dead.
                h = (vx ** 2 + vy ** 2) ** 0.5
                if h < 0.5:
                    tx, ty = -oy, ox                              # tangent to the edge
                    if a.vx * tx + a.vy * ty < 0:                 # keep the raw action's turn side
                        tx, ty = -tx, -ty
                    spd = min(max(into, 1.0), cap)
                    # ...but not straight into a building. Both signs slide along
                    # the same fence edge, so when the preferred side BREAKS the
                    # clearance rule the other side is free to take. Without this
                    # the anti-stall tangent gets vetoed by the clearance check
                    # that already ran, and the P0 guard brakes. The turn side is
                    # only overridden on an actual violation — a legal tangent
                    # keeps following the raw action's intent.
                    bld = [c for c in self._clear if _field_name(c) == "buildings"]
                    if self._dist is not None and bld:
                        hard = min(c.min_clearance_m for c in bld)
                        pro = Action4D(vx=tx * spd, vy=ty * spd,
                                       vz_up=a.vz_up, yaw_rate=a.yaw_rate)
                        if self._clear_min_dist(state, pro) < hard:
                            con = Action4D(vx=-tx * spd, vy=-ty * spd,
                                           vz_up=a.vz_up, yaw_rate=a.yaw_rate)
                            if self._clear_min_dist(state, con) >= hard:
                                tx, ty = -tx, -ty
                    vx, vy = tx * spd, ty * spd
                repairs.append(Repair(operator="GeofenceSlide",
                                      detail=f"{f.id}: removed {into:.2f} m/s into-zone component",
                                      magnitude_m=depth, axis="lateral"))
        return Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)

    # ---------------- mid-flight policy update ---------------- #

    def set_subject(self, x: float | None, y: float | None = None,
                    subject_class: str | None = None, *,
                    velocity: tuple[float, float] | None = None) -> None:
        """Tell the Shield where the tracked subject is, once per tick.

        The Shield cannot see. SubjectStandoff rules are inert until the
        perception stack supplies this, and calling `set_subject(None)` when the
        target is lost is REQUIRED rather than optional: a stale position would
        have the Shield enforcing a standoff from where the subject used to be,
        which is both wrong and unfalsifiable from the logs.

        `subject_class` is what selects between per-class rules, so a policy can
        hold 10 m from a pedestrian and 5 m from a vehicle.

        `velocity` is the subject's ground velocity (vx, vy), m/s, in the
        same world frame as Action4D's vx / vy, when the estimator has one.
        The stand-off check and its repairs (StandoffHold, StandoffRecover)
        then forecast the subject at that constant velocity and judge the
        aircraft's motion relative to it (`_check_standoff`). None - the
        default - holds the subject still over the lookahead, which is every
        decision exactly as before 2026-10-19: following a car at 4-5 m/s,
        the grant's 5 s forecast put the drone 20-25 m further on while the
        car stood still, so the car's ring was "reached" on 44 % of the
        replayed city ticks with the range not closing. It is per call, like
        the position: a call without it clears it.

        A NON-FINITE position (NaN or inf in either coordinate) is dropped:
        the Shield then has no subject, exactly as after `set_subject(None)`
        (`n_subject_nonfinite` counts the refusals, and rule_status reads the
        stand-off unbound rather than "bound, not breached"). While a binding
        stand-off is HARD and enforced, every filter() tick until a finite one
        (or None) arrives also carries a `subject-finite` contract violation
        naming the stand-off rules that cannot be judged. The FSM reads an id
        that is no policy rule as hard P0 project_fix: it enters Brake (G1,
        the Shield's action still streamed) and a NaN that flickers counts its
        onsets toward N-in-T, exactly as that hard rule flickering in and out
        of breach would (docs/DESIGN-escalation-fsm.md). With only soft or
        monitor_only stand-offs binding, nothing is raised, so a subject the
        Shield cannot place never draws a stronger response than a real
        breach of the rule: until a review of 2026-10-19 the violation was
        raised for those too, and a NaN subject beside a monitor_only
        stand-off took the FSM from Normal to Brake where a real breach of it
        left it in Normal. Until 2026-10-19 a non-finite subject was stored:
        NaN fails every comparison, so each distance test against it came
        out "outside the ring" and the stand-off stopped enforcing, while
        rule_status reported it bound and not breached. A diverged estimator
        is the source (perception's TargetState stays NaN once it is NaN).
        Not an exception: this is called from the 10 Hz loop.

        A NON-FINITE velocity is refused the same way for the same reason (a
        NaN in the forecast fails every comparison), but only the velocity:
        the position is good, so the subject is kept and held still, the
        reading every stand-off had before velocities existed
        (`n_subject_velocity_nonfinite` counts them).
        """
        if x is None or y is None:
            self._subject = None
            self._subject_class = None
            self._subject_v = None
            self._subject_rejected = None
            return
        x, y = float(x), float(y)
        if not (math.isfinite(x) and math.isfinite(y)):
            self._subject = None
            self._subject_class = None
            self._subject_v = None
            self._subject_rejected = (x, y, subject_class)
            self.n_subject_nonfinite += 1
            return
        sv = None
        if velocity is not None:
            vx, vy = (float(c) for c in velocity)
            if math.isfinite(vx) and math.isfinite(vy):
                sv = (vx, vy)
            else:
                self.n_subject_velocity_nonfinite += 1
        self._subject_rejected = None
        self._subject = (x, y)
        self._subject_class = subject_class
        self._subject_v = sv

    def _subject_track(self) -> tuple[float, float, float, float]:
        """(x, y, vx, vy) of the subject (set): its velocity is 0.0, 0.0 when
        none was given, so `x + vx * t` is exactly x and `a.vx - vx` exactly
        a.vx - the stationary arithmetic, number for number."""
        sx, sy = self._subject
        svx, svy = self._subject_v if self._subject_v is not None else (0.0, 0.0)
        tick = self._tick_ctx()
        if tick is not None and tick.ahead:
            # A look-ahead view (looking_ahead): the subject where its
            # constant velocity puts it then. Held still, it does not move.
            return sx + svx * tick.ahead, sy + svy * tick.ahead, svx, svy
        return sx, sy, svx, svy

    def _subject_min_range(self, state: State, a: Action4D) -> float:
        """The forecast's closest approach to the subject, which moves at its
        velocity when it has one: the depth a stand-off repair reports."""
        sx, sy, svx, svy = self._subject_track()
        return min(math.hypot(p.x - (sx + svx * t), p.y - (sy + svy * t))
                   for t, p in self._predict(state, a))

    # Read-only views for the on-screen policy indicator (demo/policy_hud.py),
    # so the HUD shows the stand-off and clearance the Shield is actually
    # enforcing - the estimator's subject, not ground truth - rather than a
    # second computation that could disagree with it.
    @property
    def subject(self) -> tuple[float, float] | None:
        return self._subject

    @property
    def subject_class(self) -> str | None:
        return self._subject_class

    @property
    def subject_velocity(self) -> tuple[float, float] | None:
        """The subject's (vx, vy) the stand-off forecasts with; None = still."""
        return self._subject_v

    def clearance_at(self, x: float, y: float) -> float:
        """Metres to the nearest mapped obstacle as the clearance rule reads it;
        inf without a map."""
        return self._distance_at(x, y)

    @staticmethod
    def _cap_sparing_radial(a: Action4D, ux: float, uy: float,
                            keep: float, cap: float) -> Action4D:
        """Fit under `cap` by spending the TANGENTIAL component, not the escape.

        Scaling the whole horizontal vector is the obvious way to respect the
        speed cap and the wrong one during a recovery: it shrinks the very
        component that is getting the aircraft out. Measured on the standoff
        rule, the end-of-pass SpeedClamp took a 3.00 m/s recovery down to 2.12
        while faithfully preserving 2.12 m/s of *mission* motion - it spent the
        budget on the part that was not urgent.

        `(ux, uy)` is the unit escape direction and `keep` the outward speed
        that must survive. Whatever room the cap leaves goes to the tangential
        part; if `keep` alone exceeds the cap, the escape is clipped to the cap
        and the tangential is dropped entirely.
        """
        keep = min(abs(keep), cap)
        out_v = (a.vx * ux + a.vy * uy)
        tx, ty = a.vx - out_v * ux, a.vy - out_v * uy      # tangential remainder
        t_mag = math.hypot(tx, ty)
        room = math.sqrt(max(cap * cap - keep * keep, 0.0))
        if t_mag > room > 0.0:
            tx, ty = tx * room / t_mag, ty * room / t_mag
        elif room <= 0.0:
            tx = ty = 0.0
        out_keep = max(out_v, keep) if out_v >= 0 else keep
        return Action4D(vx=tx + ux * out_keep, vy=ty + uy * out_keep,
                        vz_up=a.vz_up, yaw_rate=a.yaw_rate)

    def _repair_standoff(self, state: State, a: Action4D,
                         repairs: list["Repair"]) -> Action4D:
        """Remove the closing component of velocity along the line to the subject.

        Not a brake and not a reversal: the tangential component survives, so the
        aircraft can still circle the subject at the held range and keep it in
        frame. Killing the whole velocity would stop the mission to satisfy a
        rule that only objects to one direction of travel.

        Against a MOVING subject (set_subject(velocity=...)) every rate here
        is RELATIVE to it, as in the check: StandoffHold removes the closing
        component of the motion relative to the subject (behind a car that
        is the car's own speed along the line, so the follow keeps pace
        instead of stopping), and StandoffRecover floors the relative
        opening rate. With no velocity the subject's is exactly zero and
        every number is the stationary one.
        """
        if self._subject is None:
            return a
        sx, sy, svx, svy = self._subject_track()
        worst = None
        for so in self._standoffs:
            if so.binds(self._subject_class):
                worst = so if worst is None or so.min_range_m > worst.min_range_m else worst
        if worst is None:
            return a
        # From the stop point when the aircraft's momentum already carries it
        # into the ring (_committed): StandoffRecover then opens the range
        # before the aircraft gets there.
        state = self._rule_state(worst, state)

        dx, dy = sx - state.x, sy - state.y
        d = math.hypot(dx, dy)
        ring = worst.min_range_m
        if d <= _STANDOFF_COINCIDENT_M:
            return self._recover_from_overhead(state, a, ring, d, repairs)
        ux, uy = dx / d, dy / d            # unit vector pointing AT the subject

        # Positive means approaching, relative to the subject.
        closing = (a.vx - svx) * ux + (a.vy - svy) * uy
        # The subject's own speed along that line (> 0: away from the
        # aircraft), and the note the details then carry. 0.0 when still.
        s_u = svx * ux + svy * uy
        rel = (f", relative to the subject moving at {math.hypot(svx, svy):.2f} m/s"
               if svx or svy else "")

        # The REPAIR must trigger on the same criterion the CHECK uses, or it
        # declines to act on the very violation that was raised and the action
        # falls through to the brake - and, inside the ring, to the rescue
        # search, which was measured emitting +5 m/s straight AT the subject.
        # The check is predictive over the full lookahead, so this must be too:
        # at 4 m/s and a 3 s horizon the aircraft commits 12 m ahead of itself.
        breach_ahead = (d - closing * self.lookahead_s) < ring - 1e-9
        if self._velocity_of(state) is not None and not breach_ahead:
            # Own-velocity forecast: the CHECK sees the momentum the straight
            # line on the command above does not (the aircraft keeps closing
            # for the reaction time and while it brakes), so ask the check
            # itself, or this operator would decline the very violation it
            # was called for and the tick would fall through to the brake.
            breach_ahead = bool(self._check_standoff(worst, state, a))

        if d < ring - 1e-9:
            # ALREADY inside. Removing the closing component would leave the
            # range exactly where it is, which the check reads as "not opening" -
            # so the violation would persist every tick and the aircraft would be
            # frozen at a distance the policy forbids. Recovery means opening the
            # range, the same reasoning as the clearance ring's push-out.
            cap = min((k.speed_max_mps for k in self._kins), default=4.0)
            want = min(cap, max(0.5, (ring - d) / max(self.lookahead_s, 1e-3)))
            if self.recovery_law == "cap_all":
                # The default since 2026-11-06 (RECOVERY_LAWS): open the range
                # with the aircraft's own outward speed at the cap, as the
                # altitude band is recovered at the climb cap - the fastest
                # opening the cap allows, so the relative rate is the cap plus
                # the subject's own recession (`s_u`; 0 for a still subject).
                # Radially outward from a point subject the range only grows,
                # so there is no far edge to hold short of. The one-lookahead
                # law above took standoff-reclassified 6.5 s at 0.5-0.65 m/s
                # from 3.25 m inside; at the cap, 0.9 s.
                want = cap + s_u
            # FLOOR the opening rate, never assign it.
            #
            # `-closing` is the opening speed already commanded. Writing the
            # radial component to `want` outright made the repair a CEILING on
            # a legal escape: at 9 m from a pedestrian with a 10 m ring, an
            # action opening at 3.00 m/s passed through untouched, while 3.01 -
            # a third of a percent over the speed cap, enough to drag it into
            # the repair loop - came out at 0.500 m/s. Six times slower escape
            # from a P0 breach, and the KPI could not see it because the result
            # still re-checks clean.
            #
            # It is a floor and not a decline for the opposite reason: the
            # monitor forgives any opening above 0.1 m/s, so a repair that
            # simply stood aside would leave a 0.11 m/s crawl out of a ring the
            # policy forbids near a person. (Until 2026-10-19 this ran only when
            # some other rule had opened the repair loop, so that crawl passed
            # through untouched whenever it was the only thing wrong. A legal
            # action from an illegal position is now floored here too:
            # Shield._floor_the_escape.)
            #
            # `out` is the relative opening rate; the aircraft's own outward
            # speed is `out - s_u`, which the cap bounds, hence `cap + s_u`.
            out = min(cap + s_u, max(want, -closing))
            vx = a.vx - (closing + out) * ux
            vy = a.vy - (closing + out) * uy
            repairs.append(Repair(
                operator="StandoffRecover",
                detail=(f"range {d:.1f}m inside min {ring}m: opening at "
                        f"{out:.2f} m/s (commanded {-closing:+.2f}, "
                        f"recovery needs {want:.2f}){rel}, tangential motion kept"),
                magnitude_m=ring - d, axis="lateral", recovery=True))
            fixed = Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
            keep = out - s_u
            if keep >= 0.0:
                return self._cap_sparing_radial(fixed, ux=-ux, uy=-uy, keep=keep, cap=cap)
            # Behind a subject driving away faster than the opening needed, the
            # aircraft still closes on it, only slower than it recedes: the
            # speed to keep is TOWARD the subject, and the cap-sparing fit is
            # made along that line (never reached with the subject held still,
            # where `out` is at least min(cap, 0.5)).
            return self._cap_sparing_radial(fixed, ux=ux, uy=uy, keep=-keep, cap=cap)

        if closing <= 0.0 or not breach_ahead:
            return a                       # opening already, or no breach coming

        vx, vy = a.vx - closing * ux, a.vy - closing * uy
        # How deep the forecast being repaired goes into the ring: its
        # closest pose to the subject, the same "deepest pose" GeofenceSlide
        # reports for a zone. Until 2026-10-08 this was ring - (d - closing x
        # lookahead), a straight-line range that runs THROUGH the subject and
        # out the other side: 20 of the 25 StandoffHold repairs of the 8 Oct
        # flight reported 5.3-11.0 m into a 5 m ring, which no forecast can
        # reach. It is also what theta reads on the default basis
        # (guardrail/fsm.py DEFAULT_THETA_HORIZON_S). Against a moving subject,
        # the closest approach to where it will be at each pose's time.
        d_min = self._subject_min_range(state, a)
        repairs.append(Repair(
            operator="StandoffHold",
            detail=(f"range {d:.1f}m, closing {closing:.2f} m/s{rel} would breach "
                    f"min {ring}m within {self.lookahead_s:.0f}s: closing "
                    f"component removed, tangential motion kept"),
            magnitude_m=max(0.0, ring - d_min), axis="lateral"))
        return Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)

    def _recover_from_overhead(self, state: State, a: Action4D, ring: float,
                               d: float, repairs: list["Repair"]) -> Action4D:
        """StandoffRecover when the subject is directly below the aircraft.

        There is no line to the subject, so there is no "closing component" to
        remove and no outward direction to push along - and every direction
        opens the range at the full horizontal speed. Until 2026-10-08 this
        operator returned the action unchanged ("no defined line"), the check
        could not see any action opening the range, and the P0 escape guard
        flew a best-effort heading with the violation still on it.

        The heading kept is the one commanded, when there is one: the least
        change that leaves. A hover gets the reverse of the aircraft's
        heading, which backs the subject out in front of a forward camera.
        The speed is floored, like the off-centre recovery, at the rate that
        clears the ring in one lookahead (0.5 m/s at least, the speed cap at
        most), and never lowered.

        Over a MOVING subject the same is done in its frame: the heading and
        speed are those of the motion relative to it, and the subject's
        velocity is added back. That sum can pass the speed cap; the
        SpeedClamp closing the pass then scales it, and the check re-judges
        what is left. With no velocity, exactly the stationary action."""
        cap = min((k.speed_max_mps for k in self._kins), default=4.0)
        want = min(cap, max(0.5, (ring - d) / max(self.lookahead_s, 1e-3)))
        if self.recovery_law == "cap_all":
            want = cap                     # "cap_all" (RECOVERY_LAWS)
        _, _, svx, svy = self._subject_track()
        rx, ry = a.vx - svx, a.vy - svy
        h = math.hypot(rx, ry)
        if h > 1e-6:
            ex, ey = rx / h, ry / h
            how = "along the commanded heading"
        else:
            yaw = math.radians(state.yaw_deg)
            ex, ey = -math.cos(yaw), -math.sin(yaw)
            how = "backing away from the aircraft's heading"
        out = min(cap, max(want, h))
        moving = self._subject_v is not None
        rel = (f", relative to the subject moving at {math.hypot(svx, svy):.2f} m/s"
               if svx or svy else "")
        repairs.append(Repair(
            operator="StandoffRecover",
            detail=(f"range {d:.1f}m: directly over the subject, every heading "
                    f"opens the range; leaving {how} at {out:.2f} m/s "
                    f"(commanded {h:.2f}, recovery needs {want:.2f}){rel}"),
            magnitude_m=ring - d, axis="lateral", recovery=True))
        if not moving:
            return Action4D(vx=ex * out, vy=ey * out, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
        return Action4D(vx=svx + ex * out, vy=svy + ey * out, vz_up=a.vz_up,
                        yaw_rate=a.yaw_rate)

    def _repair_corridor(self, state: State, a: Action4D,
                         repairs: list["Repair"]) -> Action4D:
        """Steer back toward the centerline, keeping the along-corridor motion.

        Same shape as _repair_standoff, and for the same reason: the tangential
        component is the mission. A corridor rule objects to sideways drift, not
        to progress, so scaling the whole horizontal vector would satisfy the
        rule by cancelling the flight - which is why this goes through
        `_cap_sparing_radial` rather than a plain clamp.

        Two cases, and the difference matters:

          * ALREADY outside - recover at a real rate, floored at 0.5 m/s, sized
            to close the gap within one lookahead. FLOORED, never assigned: an
            action already returning faster than that must not be slowed down by
            its own repair. That exact bug (a repair acting as a ceiling on a
            legal escape) was found on the standoff rule, where 3.01 m/s of
            outbound recovery came back as 0.500.

          * still inside but FORECAST to exit - cancel the outward drift, i.e.
            floor the inward rate at zero. The permitted outward rate is
            actually (half - d) / lookahead, slightly more than zero, but
            `_cap_sparing_radial` takes the magnitude of what it is given and
            cannot express "outward, but slower". Holding the lateral position
            is the conservative side of that approximation and still leaves the
            along-corridor component untouched.

        The corridor's ALTITUDE band is repaired here too, not delegated to
        _repair_altitude - that operator only knows `AltitudeEnvelope` rules, so
        a policy carrying a corridor and no envelope had its band checked and
        never fixed. Measured before this branch: below the floor and not
        climbing, the Shield raised the violation, found no operator willing to
        act, fell through to the rescue search and emitted an action that STILL
        violated. A P0 escape manufactured by the rule that was supposed to
        prevent one.
        """
        if not self._corridors:
            return a
        cap = min((k.speed_max_mps for k in self._kins), default=4.0)
        L = max(self.lookahead_s, 1e-3)

        outer = state
        for c in self._corridors:
            state = self._rule_state(c, outer)      # the stop point, if committed
            # Altitude first: it is independent of the lateral geometry and its
            # repair is a straight projection, exactly as for AltitudeEnvelope.
            end_up = state.up + a.vz_up * L
            lo, hi = c.altitude_floor_m, c.altitude_ceiling_m
            band = f"(band [{lo}, {hi}]m)"
            if self.recovery_law == "lookahead":
                if end_up > c.altitude_ceiling_m or end_up < c.altitude_floor_m:
                    # The law until 2026-10-29, line for line: aim a margin
                    # inside when already outside (Shield._reentry_margin)
                    # and get there in one lookahead.
                    m = self._reentry_margin(c.altitude_floor_m, c.altitude_ceiling_m)
                    if end_up > c.altitude_ceiling_m:
                        tgt = c.altitude_ceiling_m - (
                            m if state.up > c.altitude_ceiling_m else 0.0)
                    else:
                        tgt = c.altitude_floor_m + (
                            m if state.up < c.altitude_floor_m else 0.0)
                    new = (tgt - state.up) / L
                    outside = not (c.altitude_floor_m <= state.up <= c.altitude_ceiling_m)
                    repairs.append(Repair(
                        operator="CorridorAltitudeFix",
                        detail=f"{c.id}: vz {a.vz_up:.2f} -> {new:.2f} (band "
                               f"[{c.altitude_floor_m}, {c.altitude_ceiling_m}]m)",
                        magnitude_m=abs(end_up - tgt), axis="vertical", recovery=outside))
                    a = Action4D(vx=a.vx, vy=a.vy, vz_up=new, yaw_rate=a.yaw_rate)
            elif state.up > hi or state.up < lo:
                # Already outside: recover as AltitudeFix does, at the climb
                # cap and aiming a margin inside (Shield._recovery_vz, which
                # sees Shield._reentry_margin); never past the far edge.
                rec = self._recovery_vz(state.up, lo, hi, L)
                far = ((lo if state.up > hi else hi) - state.up) / L
                new = (max(min(a.vz_up, rec), far) if state.up > hi
                       else min(max(a.vz_up, rec), far))
                if abs(new - a.vz_up) > 1e-9:
                    repairs.append(Repair(
                        operator="CorridorAltitudeFix",
                        detail=f"{c.id}: vz {a.vz_up:.2f} -> {new:.2f} {band}",
                        magnitude_m=abs(new - a.vz_up) * L, axis="vertical",
                        recovery=not (new == far and new != rec)))
                    a = Action4D(vx=a.vx, vy=a.vy, vz_up=new, yaw_rate=a.yaw_rate)
            elif end_up > hi or end_up < lo:
                tgt = hi if end_up > hi else lo
                new = (tgt - state.up) / L
                repairs.append(Repair(
                    operator="CorridorAltitudeFix",
                    detail=f"{c.id}: vz {a.vz_up:.2f} -> {new:.2f} {band}",
                    magnitude_m=abs(end_up - tgt), axis="vertical", recovery=False))
                a = Action4D(vx=a.vx, vy=a.vy, vz_up=new, yaw_rate=a.yaw_rate)

            d, (ux, uy) = self._corridor_geom(c, state.x, state.y)
            if ux == 0.0 and uy == 0.0:
                # Dead on the centerline: there is no "back" from here. The
                # direction that matters is the one the FORECAST leaves by, so
                # take the geometry at the lookahead endpoint instead. Skipping
                # (the first version) left a straight sideways departure with no
                # repair at all, and the P0 guard stopped the aircraft mid-
                # mission - safe, but a full stop for a drift the operator could
                # simply have been steered out of.
                _, end = list(self._predict(state, a))[-1]
                d2, (ux, uy) = self._corridor_geom(c, end.x, end.y)
                if ux == 0.0 and uy == 0.0:
                    continue                   # genuinely not moving laterally
            half = c.half_width_m
            inward = a.vx * ux + a.vy * uy     # positive = heading back
            outward = -inward

            if d > half + 1e-9:
                need = max((d - half) / L, 0.5)
                if self.recovery_law == "cap_all":
                    # The default since 2026-11-06 (RECOVERY_LAWS): the
                    # fastest inward speed, up to the cap, whose lookahead
                    # endpoint stays a re-entry margin short of the FAR wall -
                    # the altitude band's far-edge hold (_recovery_vz) - never
                    # below the old law.
                    m = self._reentry_margin(-half, half)
                    need = max(need, min(cap, (d + half - m) / L))
                excursion, recovering = d - half, True
            elif d + outward * L > half + 1e-9:
                need = 0.0
                excursion, recovering = d + outward * L - half, False
            elif self._velocity_of(state) is not None and (
                    ex := max((nearest_on_polyline(p.x, p.y, c.points())[0] - half
                               for _, p in self._predict(state, a)), default=0.0)) > 1e-9:
                # Own-velocity forecast: the aircraft's momentum can carry the
                # forecast out where the straight line on the command does
                # not; read the excursion off the forecast itself, as the
                # check does, so the operator acts on what the check raised.
                need = 0.0
                excursion, recovering = ex, False
            else:
                continue                        # forecast stays inside
            if (not recovering and state is outer
                    and (shaped := self._shaped_escape(state, a)) is not None
                    and not self._check_corridor(c, state, a, shaped)):
                # An escape from a zone that opened on the aircraft, judged
                # as flown - out of the ring, then a stop (_shaped_escape) -
                # stays inside the corridor: the check raises nothing, so
                # there is nothing to cancel. The straight line above would
                # have cut the way out to zero in the library's own geometry
                # (the corridor edge 9 m past the school zone's ring; review
                # of 69c27bd, P2).
                continue
            if inward >= need - 1e-9:
                continue                        # already returning fast enough

            before = (a.vx, a.vy)
            a = self._cap_sparing_radial(a, ux, uy, min(need, cap), cap)
            if (a.vx, a.vy) != before:
                repairs.append(Repair(
                    operator="CorridorReturn",
                    detail=(f"{c.id}: {d:.1f}m off centerline "
                            f"(half-width {half:.1f}m), inward "
                            f"{inward:.2f} -> {need:.2f} m/s"),
                    magnitude_m=excursion, axis="lateral", recovery=recovering))
        return a

    def state_is_unsafe(self, state: State, *, ahead_s: float = 0.0) -> list[Violation]:
        """Is this POSITION illegal, whatever the vehicle does next?

        The test is "would standing still here violate a rule". That separates
        the two things a violation can mean, which are easy to conflate and
        measure completely different quantities:

          * the requested ACTION is illegal - too fast, or aimed at a fence it
            has not reached yet. The Shield repairs it and nothing unsafe ever
            happens.
          * the POSITION is illegal - already inside the zone, already under the
            floor. No choice of action makes this tick safe.

        `p0_violation_escape_rate` is about the first. `mean time to safe` is
        about the second, and computing it from the first inflates it wildly:
        `ros2_shield_on` has 219 present-tense action violations and ZERO unsafe
        positions.

        `filter()` already relies on this idea - it asks `_check(state, BRAKE)`
        before it dares to brake, precisely because stopping inside a clearance
        ring is not a fail-safe. This exposes the same question by name so the
        KPI layer can record it per tick instead of re-deriving it.

        Cross-checked against dwell times computed by an unrelated code path:
        216 unsafe ticks on `sitl_ped_on` against a logged `alt_violation_s` of
        21.6 s, and 41 on `ros2_shield_off` against `nfz_s` 3.7 s.

        Returns the violations, so a caller can say WHICH rule the position
        breaks; truthiness is the common use.

        It judges a stop over the whole lookahead, so "illegal" includes "will
        become illegal within the lookahead" (a window opening over it, a
        zone moving onto it). `ahead_s` (since 2026-10-19) asks the same of a
        stop that STARTS `ahead_s` seconds after this tick (`looking_ahead`):
        "if the aircraft arrived here then, could it hold here through the
        lookahead?" - the escalation FSM's hold-point test
        (guardrail/holdpoint.py). 0.0 is exactly the call it always was.

        A measured velocity on `state` is dropped first (`_positional`): this
        stays the POSITION question - the rails' `unsafe` column, the FSM's
        stop_illegal and the time-to-safe KPI read it so - and never becomes
        "where would a stop from this speed end", which the own-velocity
        forecast asks inside the decision instead (`_committed`).
        """
        if ahead_s:
            with self.looking_ahead(ahead_s):
                return self._check_in(self._positional(state), BRAKE)
        return self._check(self._positional(state), BRAKE)

    def recheck_commanded(self, state: State, action: Action4D) -> list[Violation]:
        """`action` re-checked from the POSITION on the COMMANDED forecast -
        the measured velocity dropped (`_positional`): the basis of every P0
        escape count stored before 2026-10-19. A rail that gives the Shield a
        velocity logs this beside its own re-check (decision.emitted_
        violations, made with the velocity, as the decision was), so a run
        with the own-velocity forecast is comparable like for like with the
        stored ones (2026-10-20 review). An action the Shield emitted clean
        with the velocity re-checks clean here too (the measured velocity
        only adds; the property test in tests/test_own_velocity_forecast.py),
        so this count is never above the velocity re-check's."""
        return self._check(self._positional(state), action)

    def stop_breaks(self, state: State) -> list[str]:
        """The HARD, enforced rules a stop at this POSITION breaks, by id: the
        reading the escalation FSM gives `stop_illegal` (a soft rule is capped
        at brake, a monitor_only rule is not enforced). For a caller judging a
        path pose by pose - the hold's stop path (guardrail/response.py
        `hold_plan`)."""
        if self._tick_ctx() is None and self._needs_tick():
            with self._scope(self._new_tick(None)) as tick:
                return self._stop_breaks_in(tick.rt, state)
        tick = self._tick_ctx()
        return self._stop_breaks_in(tick.rt if tick is not None else self._rt, state)

    def _stop_breaks_in(self, rt: _Runtime, state: State) -> list[str]:
        return sorted({v.rule_id for v in self.state_is_unsafe(state)
                       if rt.hard.get(v.rule_id, True)
                       and self._act(rt, v.rule_id) != "monitor_only"})

    def looking_ahead(self, ahead_s: float):
        """A context (`with shield.looking_ahead(3.0): ...`) in which every rule
        query of this Shield - state_is_unsafe, rule_status, zones_now,
        unsafe_depth, the checks behind them - answers for `ahead_s` seconds
        after the current monitor tick: the rule clock read that much later
        (windows, switches, swaps), each moving zone aged by it, a moving
        subject advanced along its velocity. Nested views are relative to the
        monitor tick, not to each other. Nothing is decided or recorded: the
        sliding window, the FSM and the clock are untouched.

        The one place the Shield reasons about a time other than now. It
        exists so the hold-point search can ask the Shield's own rule code -
        not a second geometry - whether a point reached at a given time is
        legal from then on ("there is exactly one rule-evaluation code path in
        the system", Safety Shield page)."""
        a = float(ahead_s)
        if not math.isfinite(a) or a < 0:
            raise ValueError(f"ahead_s must be a finite number >= 0, got {ahead_s!r}")
        base = self._tick_ctx()
        if base is None:
            base = self._new_tick(None)
        elif base.origin is not None:
            base = base.origin
        when = base.sched.when
        if when is not None:
            when = when + timedelta(seconds=a)
        sched = _Schedule(base.rt, when, self.dt / 2.0, self._grid())
        view = _Tick(base.rt, sched, base.t + a,
                     base._t_first if base._t_first is not None else base.t)
        view.ahead = a
        view.origin = base
        return self._scope(view)

    def hard_violations(self, violations) -> list[Violation]:
        """The violations of HARD rules the Shield enforces (not monitor_only),
        under the rule set of the current tick: the filter the FSM's
        `stop_illegal` applies to state_is_unsafe (a soft rule is capped at
        brake, so a stop that breaks only a soft rule is a response that rule
        allows). A violation of no policy rule (the Shield's own contract
        checks) counts as hard, as fsm.rules_from_policy reads it."""
        tick = self._tick_ctx()
        rt = tick.rt if tick is not None else self._rt
        return [v for v in violations
                if rt.hard.get(v.rule_id, True)
                and self._act(rt, v.rule_id) != "monitor_only"]

    def kinematic_caps(self) -> "_fsm.KinematicCaps | None":
        """The smallest speed and climb caps among the enforced kinematic
        rules in force in this tick's lookahead (not monitor_only), or None:
        the speeds the hold-point search plans a transit at."""
        tick = self._tick_ctx()
        if tick is None:
            with self._scope(self._new_tick(None)):
                return self.kinematic_caps()
        rt = tick.rt
        return _fsm.KinematicCaps.of(k for k in tick.rules("kins")
                                     if self._act(rt, k.id) != "monitor_only")

    def unsafe_depth(self, state: State, violations: list | None = None) -> float:
        """How far this POSITION is from legal, in metres: summed over the
        HARD, enforced rules standing still here breaks (`state_is_unsafe`, or
        the `violations` given), each measured its own way - the altitude
        error outside an envelope or corridor band, the depth inside a zone's
        margin ring, the distance inside a stand-off or clearance ring, the
        excess over a corridor's half-width. 0.0 where a stop is legal.

        The escalation FSM's X6 reads its TREND (fsm.TickInput.unsafe_depth_m):
        a position illegal for T whose depth keeps falling is a recovery that
        is working, not one to abort. On 7 Oct X6 sent climbs from under the
        altitude floor home at 5.0 s while they climbed at the 2 m/s cap.
        A rule this method cannot measure contributes 0, which reads as no
        progress on it - the side that still escalates."""
        if self._tick_ctx() is None and self._needs_tick():
            with self._scope(self._new_tick(None)):
                return self._unsafe_depth_in(state, violations)
        return self._unsafe_depth_in(state, violations)

    def _unsafe_depth_in(self, state: State, violations: list | None) -> float:
        vios = self.state_is_unsafe(state) if violations is None else violations
        tick = self._tick_ctx()
        rt = tick.rt if tick is not None else self._rt
        ids = {v.rule_id for v in vios
               if rt.hard.get(v.rule_id, True)
               and self._act(rt, v.rule_id) != "monitor_only"}
        if not ids:
            return 0.0
        # Summed in the order the terms come (envelopes, stand-offs,
        # clearance, corridors, zones): the same additions, in the same
        # order, as before the terms were split out (2026-10-20), so X6's
        # input is bit for bit what it was.
        total = 0.0
        for _, d in self._depth_terms(state, ids):
            total += d
        return total

    def _depth_terms(self, state: State, ids) -> list[tuple[str, float]]:
        """(rule id, depth) for each rule in `ids` that this method can
        measure at `state`, in unsafe_depth's order; the depth is how far past
        that rule's limit the position is (0.0 when it is not past it). Read
        in the current tick or look-ahead view (looking_ahead): a moving zone
        where it is then, a moving subject advanced along its velocity
        (`_subject_track`, which is the subject itself on every real tick)."""
        x, y, up = state.x, state.y, state.up
        out: list[tuple[str, float]] = []
        for env in self._alts:
            if env.id in ids:
                out.append((env.id, max(0.0, env.alt_min_m - up, up - env.alt_max_m)))
        if self._subject is not None:
            sx, sy = self._subject_track()[:2]
            d = math.hypot(x - sx, y - sy)
            for so in self._standoffs:
                if so.id in ids and so.binds(self._subject_class):
                    out.append((so.id, max(0.0, so.min_range_m - d)))
        here_by: dict = {}
        for c in self._clear:
            f = self._field_for(c) if c.id in ids else None
            if f is None:
                continue
            key = _field_name(c)
            if key not in here_by:
                here_by[key] = self._distance_at(x, y, f)
            out.append((c.id, max(0.0, c.min_clearance_m - here_by[key])))
        for c in self._corridors:
            if c.id in ids:
                d_lat, _ = self._corridor_geom(c, x, y)
                out.append((c.id, max(0.0, d_lat - c.half_width_m)
                            + max(0.0, c.altitude_floor_m - up, up - c.altitude_ceiling_m)))
        anchors, tick = self._fence_anchors(self._ir)
        for rec in anchors:
            f = rec.rule
            if f.id not in ids:
                continue
            if rec.moving and tick is not None:
                rec = tick.current(rec)     # only the zones a violation names
            if point_in_fence(x, y, up, f, rec.polygon, rec.buffered):
                out.append((f.id, rec.buffered.boundary.distance(Point(x, y))))
        return out

    def rule_depths(self, state: State, rule_ids) -> dict[str, float]:
        """How far past each named rule's limit this POSITION is, in metres:
        the per-rule terms `unsafe_depth` sums (the altitude error outside an
        envelope or corridor band, the depth inside a zone's margin ring, the
        distance inside a stand-off or clearance ring, the excess over a
        corridor's half-width), in the current tick or look-ahead view. A
        rule it cannot measure (a speed cap; clearance with no map) reads
        0.0, as in unsafe_depth.

        It is what "deeper" means when the hold-point search tells a transit
        LEAVING a rule from one going deeper into it (guardrail/holdpoint.py
        transit_breaks, since 2026-10-20): the search reads the same terms
        from rule_clearances' parts, which it can bound between two poses,
        and tests/test_hold_transit.py judges every plan with this method."""
        ids = set(rule_ids)
        if self._tick_ctx() is None and self._needs_tick():
            with self._scope(self._new_tick(None)):
                return self.rule_depths(state, ids)
        out = {rid: 0.0 for rid in ids}
        for rid, d in self._depth_terms(state, ids):
            out[rid] = out[rid] + d
        return out

    def rule_clearances(self, state: State, rule_ids=None) -> dict[str, "RuleClearance"]:
        """Each POSITION rule's signed room at `state`, split into the parts a
        transit check can bound separately (guardrail/holdpoint.py,
        transit_breaks), in the current tick or look-ahead view: a moving
        zone where it is then, a moving subject advanced, the rule clock
        read then. `rule_ids` (default: every rule) limits the answer; a rule
        with no position geometry here (a speed cap, a switch, a stand-off
        with no subject it binds, clearance with no map) is left out.

          keep_out  True for a keep-out zone (polygon_fence, dynamic_nfz):
                    breached inside its margin ring AND within its band.
                    False for a keep-in rule (altitude envelope, corridor,
                    stand-off ring, obstacle clearance): breached past any
                    one of its limits.
          h         horizontal room in metres: a zone's SIGNED distance to
                    its margin ring (+ outside, - inside, the depth
                    rule_depths reports for a breach), a corridor's half-width
                    minus the lateral offset, the separation minus a
                    stand-off's minimum range, the clearance minus the
                    minimum; +inf for an altitude envelope.
          lo, hi    vertical room: up - floor and ceiling - up of the rule's
                    altitude band (a zone's band, an envelope's or a
                    corridor's limits); +inf for a rule with none.
          in_force  the rule is in force (both sides of the instant), as
                    rule_status says.
          breach    standing here breaks it: rule_status's own test.

        The room `s` (RuleClearance.room) is max(h, -lo, -hi) for a zone and
        min(h, lo, hi) for a keep-in rule; s > 0 means the position does not
        breach the rule. Every part is the Shield's own number, from the same
        compiled ring, subject track, distance field and corridor geometry
        rule_status and rule_depths read (one rule-evaluation code path,
        Safety Shield page). `h` changes by at most `h_lip` metres per metre
        the position moves (1 for a distance to a set; the clearance field's
        own measured bound, _field_lipschitz) and lo, hi by exactly the
        change in altitude, which is what lets the hold-point search bound a
        rule's room BETWEEN two sampled poses instead of trusting the samples
        (review of T1c, 2026-10-20: a thin bar crossed between two 1 m
        samples). A rule's depth - rule_depths' term - is max(0, -h) for a
        zone within its band, and max(0, -h) + max(0, -lo, -hi) for a keep-in
        rule (a corridor's lateral and vertical excess ADD, as in
        unsafe_depth)."""
        if self._tick_ctx() is None:
            with self._scope(self._new_tick(None)):
                return self.rule_clearances(state, rule_ids)
        tick = self._tick_ctx()
        rt = tick.rt
        want = None if rule_ids is None else set(rule_ids)
        recs = None
        x, y, up = state.x, state.y, state.up
        P = None
        subject, subject_class = self._subject, self._subject_class
        if subject is not None and tick.ahead:
            subject = self._subject_track()[:2]
        out: dict[str, RuleClearance] = {}
        inf = math.inf
        for c in rt.constraints:
            if want is not None and c.id not in want:
                continue
            m = tick.sched.mask(c)
            in_force = m is True or (m is not False and m(0.0))
            if c.type in FENCE_TYPES:
                if recs is None:
                    recs = {r.rule.id: r for r in rt.ir.fences}
                    P = Point(x, y)
                rec = recs[c.id]
                if rec.moving:
                    rec = tick.current(rec)
                if rec.buffered.contains(P):
                    h = -rec.buffered.boundary.distance(P)
                else:
                    h = rec.buffered.distance(P)
                out[c.id] = RuleClearance(
                    True, h, up - c.altitude_floor_m, c.altitude_ceiling_m - up, in_force,
                    in_force and zone_breached(c, x, y, up, rec.polygon, rec.buffered))
            elif c.type == "altitude_envelope":
                below, above = self._alt_outside(c, up)
                out[c.id] = RuleClearance(False, inf, up - c.alt_min_m, c.alt_max_m - up,
                                          in_force, in_force and (below or above))
            elif c.type == "subject_standoff":
                if subject is None or not c.binds(subject_class):
                    continue
                sep = math.hypot(x - subject[0], y - subject[1])
                if not math.isfinite(sep):
                    continue
                out[c.id] = RuleClearance(False, sep - c.min_range_m, inf, inf, in_force,
                                          in_force and self._standoff_inside(c, sep))
            elif c.type == "obstacle_clearance":
                f = self._field_for(c)
                if f is None:
                    continue
                d = self._distance_at(x, y, f)
                if not math.isfinite(d):
                    continue
                out[c.id] = RuleClearance(False, d - c.min_clearance_m, inf, inf, in_force,
                                          in_force and d < c.min_clearance_m, f[4])
            elif c.type in ("corridor", "corridor_swap"):
                off, _ = self._corridor_geom(c, x, y)
                out[c.id] = RuleClearance(
                    False, c.half_width_m - off, up - c.altitude_floor_m,
                    c.altitude_ceiling_m - up, in_force,
                    in_force and (corridor_wide(c, off) or not corridor_in_band(c, up)))
        return out

    def rule_motion(self, rule_ids=None) -> dict[str, tuple[float, float, float, float, float]]:
        """How each rule's geometry moves from this tick on, by the Shield's
        own motion model: (ux, uy, omega_rad_s, cx, cy) - a moving
        dynamic_nfz's velocity, turn rate and current rotation centre
        (`_Tick.motion`, the frame `_to_zone_frame` undoes), and for a
        stand-off from a moving subject the subject's constant velocity
        (omega 0, centre the subject now). A rule whose geometry stands
        still is left out. Read in the current tick or look-ahead view. The
        hold-point search turns it into how fast a rule's room can change
        along a transit (guardrail/holdpoint.py, transit_breaks)."""
        if self._tick_ctx() is None:
            with self._scope(self._new_tick(None)):
                return self.rule_motion(rule_ids)
        tick = self._tick_ctx()
        want = None if rule_ids is None else set(rule_ids)
        out: dict[str, tuple[float, float, float, float, float]] = {}
        for c in tick.rt.constraints:
            if want is not None and c.id not in want:
                continue
            if c.type in FENCE_TYPES:
                mv = tick.motion(c)
                if mv is not None and (mv[0] or mv[1] or mv[2]):
                    out[c.id] = tuple(float(v) for v in mv)
            elif (c.type == "subject_standoff" and self._subject is not None
                  and self._subject_v is not None and c.binds(self._subject_class)):
                sx, sy, svx, svy = self._subject_track()
                if svx or svy:
                    out[c.id] = (float(svx), float(svy), 0.0, float(sx), float(sy))
        return out

    # ---------------- mid-flight events (the grant's hot-apply path) ---------------- #
    #
    # Policy DSL page: the hot path "runs whenever a dynamic_nfz,
    # time_window_switch, or corridor_swap event arrives via the REST endpoint
    # or the Stress Testing harness ... Bump generation + re-sign hash ...
    # re-derive affected spatial-index entries". Every event below builds the
    # new rule list, checks it, then commits in one step (`_commit`): the live
    # Policy gets the new list and generation + 1, and the compiled runtime is
    # rebuilt (unchanged zones keep their compiled rings) and swapped in with
    # one assignment. A tick already running keeps the runtime it pinned.

    def _event_time(self, t: float | None) -> float | None:
        if t is not None:
            return float(t)
        return self._t_last                    # None before the first tick

    def _commit(self, constraints: list, op: str, rule_id: str, rule_type: str,
                t: float | None, anchors: dict | None = None) -> dict:
        ids = Counter(c.id for c in constraints)
        dup = sorted(i for i, n in ids.items() if n > 1)
        if dup:
            raise RuleIdConflict(f"{op} refused: rule id(s) {dup} would be used twice; "
                                 f"audit records and events address rules by id")
        # The patch validator: the DSL's own consistency lint, which every
        # loader runs. An event may not ADD a finding (a switch left pointing
        # at an expired zone, a swap aimed at a fence), or the generation's
        # snapshot could not be loaded again. Findings the policy already had
        # are not this event's; nor is "no rules" (a Shield may start empty
        # and an expire may empty it again).
        cand = self.policy.model_copy(update={"constraints": constraints,
                                              "generation": self.policy.generation + 1})
        new = sorted(set(cand.lint()) - set(self.policy.lint()))
        new = [f for f in new if not f.startswith("the policy has no rules")]
        if new:
            raise HotApplyRefused(f"{op} {rule_id!r} refused: the policy it would leave "
                                  f"fails the DSL lint: " + "; ".join(new))
        # ...and the constructor's gate. A Shield is never BUILT over a rule
        # it would not enforce (`unenforceable`: altitude_ref MSL, a class it
        # does not check), and the lint above has no such test. Until
        # 2026-10-19 this path did not ask either, so a dynamic_nfz,
        # corridor_swap or pre-start polygon_fence with altitude_ref MSL was
        # accepted and then judged as metres above ground: a 120-300 m MSL
        # zone over 100 m ground let an aircraft at 50 m AGL fly through it
        # with no violation, and Shield(shield.policy) then refused the very
        # policy the Shield was flying. Only what the event ADDS is refused,
        # like the lint (the constructor keeps the policy clean, so in
        # practice that is everything).
        bad = sorted(set(Shield.unenforceable(cand, self._ground))
                     - set(Shield.unenforceable(self.policy, self._ground)))
        if bad:
            raise HotApplyRefused(f"{op} {rule_id!r} refused: the Shield would not "
                                  f"enforce it: " + "; ".join(bad))
        # Build first, commit after. Until 2026-10-19 the live Policy took the
        # new list and generation + 1 BEFORE the runtime was built, with no
        # rollback: a build that raised (a dynamic_nfz with margin_m 1e308
        # passes pydantic, the lint and the margin check, then GEOS fails to
        # buffer it) left /health and /policy reporting a rule the runtime
        # did not enforce, every later event failing on the same rule, and a
        # retry refused with RuleIdConflict as though it had been applied.
        # Now a failed build changes nothing and is refused (HotApplyRefused
        # is a ValueError: a 422 over REST, not a 500). `anchors` may be {}
        # (expire of the last moving zone), so it is tested against None.
        anchors = self._anchors if anchors is None else anchors
        try:
            rt = _build_runtime(cand, self._rt, anchors, self._ground)
        except Exception as e:                       # noqa: BLE001 - any build fault
            raise HotApplyRefused(f"{op} {rule_id!r} refused: the runtime could not "
                                  f"compile the policy it would leave: "
                                  f"{type(e).__name__}: {e}") from e
        # In place: callers (the REST app, the audit logger) hold this Policy.
        self.policy.constraints = constraints
        self.policy.generation = cand.generation
        self._anchors = anchors
        self._rt = rt
        ev = {"op": op, "type": rule_type, "rule_id": rule_id,
              "generation": self.policy.generation,
              "policy_hash": self._rt.ir.policy_hash,
              "t": t, "mission_started": self.mission_started}
        self.events.append(ev)
        return ev

    def _find(self, rule_id: str):
        for i, c in enumerate(self.policy.constraints):
            if c.id == rule_id:
                return i, c
        raise UnknownRule(f"no rule {rule_id!r} in {self.policy.policy_id} "
                          f"(generation {self.policy.generation})")

    # The layer rule (grant, Policy DSL page, "Layered authoring model"; the
    # same test models.merge_layers runs at ingest). An event has a layer like
    # any rule: its own `layer` field, absent = mission. An event may relax a
    # rule of its OWN layer (that is what a switch-off or a swap is for), and
    # any soft rule, but never a HARD rule written in another layer: no
    # switch-off, no looser swap, no edit or expiry of the zone. Before
    # 2026-10-07 (review) the only validator was the DSL lint, which has no
    # relaxation check, so a mission-layer switch turned a hard regulation
    # zone off and the aircraft flew into it unrepaired.
    #
    # A merged rule also answers to the lower layers whose hard copy it
    # overrode (models.ConstraintBase.locked_by, written by merge_layers).
    # Until 2026-10-19 only the winning copy's layer counted: a mission layer
    # that legitimately tightened a regulation zone made it a "mission" rule,
    # and a plain REST switch-off (mission layer) then turned the regulation
    # zone off. Such a rule can be relaxed by no single layer's event.

    @staticmethod
    def _cross_layer_hard(tgt, layer: str) -> bool:
        return tgt.constraint_type == "hard" and (
            tgt.effective_layer != layer
            or any(lv != layer for lv in (getattr(tgt, "locked_by", None) or ())))

    @staticmethod
    def _held_by(tgt) -> str:
        """'a hard regulation-layer rule', or for a merged rule
        'a hard mission-layer rule overriding a hard regulation-layer copy'."""
        s = f"a hard {tgt.effective_layer}-layer rule"
        locks = getattr(tgt, "locked_by", None) or ()
        if locks:
            s += f" overriding a hard {'/'.join(locks)}-layer copy"
        return s

    def _guard_zone_edit(self, z, op: str, layer: str) -> None:
        if self._cross_layer_hard(z, layer):
            if getattr(z, "locked_by", None):
                how = ("no single layer's event may relax it; re-merge the layers, "
                       "or spawn a new zone")
            else:
                how = (f"send the event from the {z.effective_layer} layer "
                       f"(layer={z.effective_layer!r}, Python API only), or spawn a "
                       f"new zone")
            raise LayerRelaxation(
                f"{op} {z.id!r} refused: it is {self._held_by(z)} "
                f"and this event is {layer}-layer; moving, reshaping or expiring it "
                f"can relax it, and a rule may not be relaxed from another layer. "
                f"To change it, {how}")

    def hot_apply(self, rule, t: float | None = None) -> dict:
        """Apply one mid-flight event and return its record (op, rule, new
        generation and hash).

        Accepted: a `DynamicNFZ` (spawned: the id must be new; a zone with
        `motion` moves from the event time), a `TimeWindowSwitch` (holds its
        target rule on or off while the switch is in force; it supersedes an
        earlier unscheduled switch on the same target) and a `CorridorSwap`
        (replaces its target corridor while in force; it supersedes an earlier
        unscheduled swap of the same corridor).

        Refused with `LockedRuleClass`: every other class once the mission has
        started (the first filter() tick). Before that, a polygon_fence or
        circle_fence is still part of the mission-start policy and is appended
        as before (generation + 1). See `as_dynamic_nfz` / MIGRATION_NOTE.

        Refused with `LayerRelaxation`: a switch-off of a hard rule of another
        layer than the switch's own (or one that would override another
        layer's switch holding it on), and a swap of a hard corridor of
        another layer that is not provably no looser - the tests
        models.merge_layers runs at ingest. An event's layer is its `layer`
        field, absent = mission."""
        kind = getattr(rule, "type", None)
        with self._lock:
            if kind == "dynamic_nfz":
                return self._spawn(rule, t)
            if kind == "time_window_switch":
                return self._switch(rule, t)
            if kind == "corridor_swap":
                return self._swap(rule, t)
            if kind in ("polygon_fence", "circle_fence") and not self.mission_started:
                return self._commit(list(self.policy.constraints) + [rule], "add_before_start",
                                    rule.id, kind, self._event_time(t))
            what = f"{kind} {getattr(rule, 'id', '?')!r}"
            if kind in ("polygon_fence", "circle_fence"):
                raise LockedRuleClass(f"hot_apply refused {what} after mission start: "
                                      + MIGRATION_NOTE)
            raise LockedRuleClass(
                f"hot_apply refused {what}: only {', '.join(HOT_APPLICABLE)} may be "
                f"hot-applied (Policy DSL page); every other class is locked at "
                f"mission start")

    # -- dynamic_nfz

    def _spawn(self, zone: DynamicNFZ, t: float | None) -> dict:
        if not isinstance(zone, DynamicNFZ):
            raise HotApplyRefused(f"dynamic_nfz event must be a DynamicNFZ, got "
                                  f"{type(zone).__name__}")
        if zone.margin_m < 0:
            raise HotApplyRefused(f"{zone.id}: margin_m {zone.margin_m} shrinks the "
                                  f"zone inward (the DSL lint refuses it)")
        if any(c.id == zone.id for c in self.policy.constraints):
            raise RuleIdConflict(f"dynamic_nfz {zone.id!r}: the id already exists; "
                                 f"move/translate/rotate/scale/expire edit a zone")
        te = self._event_time(t)
        anchors = dict(self._anchors)
        anchors[zone.id] = te
        return self._commit(list(self.policy.constraints) + [zone], "spawn",
                            zone.id, "dynamic_nfz", te, anchors)

    def spawn_nfz(self, zone: DynamicNFZ, t: float | None = None) -> dict:
        """dynamic_nfz "add"."""
        with self._lock:
            return self._spawn(zone, t)

    def _zone_now(self, zone_id: str):
        i, z = self._find(zone_id)
        if not isinstance(z, DynamicNFZ):
            raise LockedRuleClass(f"{zone_id!r} is a {z.type}: only a dynamic_nfz can be "
                                  f"moved, reshaped or expired mid-flight. "
                                  + (MIGRATION_NOTE if is_fence(z) else ""))
        return i, z

    def _edit_zone(self, zone_id: str, op: str, fn, t: float | None,
                   motion=False, layer: str = DEFAULT_LAYER) -> dict:
        with self._lock:
            i, z = self._zone_now(zone_id)
            self._guard_zone_edit(z, op, layer)
            te = self._event_time(t)
            # Re-anchor a moving zone where it is NOW, so the edit applies to
            # the zone the aircraft sees and the motion continues from there.
            ring = z.ring_at(self._zone_age(z, te)) if z.motion is not None else \
                [(v.x, v.y) for v in z.vertices]
            new_ring = fn(ring)
            data = z.model_dump(mode="json", exclude_none=True)
            data["vertices"] = [{"x": float(x), "y": float(y)} for x, y in new_ring]
            if motion is not False:
                if motion is None:
                    data.pop("motion", None)
                else:
                    data["motion"] = motion
            try:
                nz = DynamicNFZ.model_validate(data)
            except Exception as e:                       # pydantic ValidationError
                raise HotApplyRefused(f"{op} {zone_id!r}: {e}") from None
            cons = list(self.policy.constraints)
            cons[i] = nz
            anchors = dict(self._anchors)
            anchors[zone_id] = te
            ev = self._commit(cons, op, zone_id, "dynamic_nfz", te, anchors)
            if self._carry is not None:
                # Re-based where it is now: nothing left to carry for it.
                self._carry[1].pop(zone_id, None)
            return ev

    def zone_ring_at(self, zone_id: str, t: float | None) -> list[tuple[float, float]]:
        """Where dynamic_nfz `zone_id` is at time `t` as THIS Shield places
        it - the ring an edit at `t` re-anchors (_edit_zone): its declared
        motion over its age since its last edit, or since the episode's first
        tick for a zone the policy declared. A caller that only swaps a zone's
        motion passes this ring back to move_nfz, so the zone stays where it
        is. (The stress harness computed it itself until 2026-10-19 and took
        a policy-declared zone's age as 0, which put a moving zone back at its
        authored vertices: review, harness-tools/ring-now-wrong-anchor-for-
        policy-zones.)"""
        with self._lock:
            _, z = self._zone_now(zone_id)
            te = self._event_time(t)
            return (z.ring_at(self._zone_age(z, te)) if z.motion is not None
                    else [(v.x, v.y) for v in z.vertices])

    def _zone_age(self, z, te: float | None) -> float:
        if self._carry is not None and z.id in self._carry[1] and (
                te is None or te < self._carry[0]):
            # Between reset_episode() and the new episode's first tick, on a
            # clock that restarted (or none): the zone is where the last
            # episode left it.
            return self._carry[1][z.id]
        a = self._anchors.get(z.id)
        if a is None:
            a = self._t_first
        if a is None or te is None:
            return 0.0
        return max(0.0, te - a)

    @staticmethod
    def _ring_centre(ring) -> tuple[float, float]:
        # The vertex mean: DynamicNFZ.ring_at rotates about the same point.
        return (sum(p[0] for p in ring) / len(ring), sum(p[1] for p in ring) / len(ring))

    # Every edit below takes `layer`, the layer the event comes from (default
    # mission, the REST channel's). A hard zone of another layer is refused
    # (`_guard_zone_edit`).

    def move_nfz(self, zone_id: str, vertices, t: float | None = None,
                 motion=False, *, layer: str = DEFAULT_LAYER) -> dict:
        """dynamic_nfz "move": new vertices (metres, policy frame). `motion` a
        Motion dict to change it, None to stop the zone, False to keep it."""
        pts = [(float(v["x"]), float(v["y"])) if isinstance(v, dict)
               else (float(v[0]), float(v[1])) for v in vertices]
        return self._edit_zone(zone_id, "move", lambda ring: pts, t, motion, layer)

    def translate_nfz(self, zone_id: str, dx: float, dy: float,
                      t: float | None = None, *, layer: str = DEFAULT_LAYER) -> dict:
        """dynamic_nfz "translate" by (dx north, dy east) metres."""
        return self._edit_zone(zone_id, "translate",
                               lambda ring: [(x + dx, y + dy) for x, y in ring], t,
                               layer=layer)

    def rotate_nfz(self, zone_id: str, angle_deg: float, t: float | None = None, *,
                   layer: str = DEFAULT_LAYER) -> dict:
        """dynamic_nfz "rotate" about its vertex mean; +angle is clockwise seen
        from above (north towards east), the Motion convention."""
        th = math.radians(angle_deg)
        c, s = math.cos(th), math.sin(th)

        def fn(ring):
            cx, cy = self._ring_centre(ring)
            return [(cx + (x - cx) * c - (y - cy) * s, cy + (x - cx) * s + (y - cy) * c)
                    for x, y in ring]
        return self._edit_zone(zone_id, "rotate", fn, t, layer=layer)

    def scale_nfz(self, zone_id: str, factor: float, t: float | None = None, *,
                  layer: str = DEFAULT_LAYER) -> dict:
        """dynamic_nfz "scale" about its vertex mean by `factor` > 0."""
        if not (isinstance(factor, (int, float)) and math.isfinite(factor) and factor > 0):
            raise HotApplyRefused(f"scale {zone_id!r}: factor must be a finite number > 0, "
                                  f"got {factor!r}")

        def fn(ring):
            cx, cy = self._ring_centre(ring)
            return [(cx + (x - cx) * factor, cy + (y - cy) * factor) for x, y in ring]
        return self._edit_zone(zone_id, "scale", fn, t, layer=layer)

    def expire_nfz(self, zone_id: str, t: float | None = None, *,
                   layer: str = DEFAULT_LAYER) -> dict:
        """dynamic_nfz "expire": remove the zone."""
        with self._lock:
            i, z = self._zone_now(zone_id)
            self._guard_zone_edit(z, "expire", layer)
            cons = list(self.policy.constraints)
            del cons[i]
            anchors = {k: v for k, v in self._anchors.items() if k != zone_id}
            return self._commit(cons, "expire", zone_id, "dynamic_nfz",
                                self._event_time(t), anchors)

    # -- time_window_switch

    def _switch(self, sw: TimeWindowSwitch, t: float | None) -> dict:
        if not isinstance(sw, TimeWindowSwitch):
            raise HotApplyRefused(f"time_window_switch event must be a TimeWindowSwitch, "
                                  f"got {type(sw).__name__}")
        _, tgt = self._find(sw.target_id)
        if isinstance(tgt, (TimeWindowSwitch, CorridorSwap)):
            raise HotApplyRefused(f"{sw.id}: targets {tgt.id!r}, a {tgt.type}; switch a "
                                  f"rule, not an event")
        layer = sw.effective_layer
        cons = list(self.policy.constraints)
        if not sw.active and tgt.constraint_type == "hard":
            if self._cross_layer_hard(tgt, layer):
                raise LayerRelaxation(
                    f"{sw.id}: a {layer}-layer switch may not turn off {tgt.id!r}, "
                    f"{self._held_by(tgt)} (Policy DSL page, layered "
                    f"authoring: a rule may not be relaxed from another layer; "
                    f"models.merge_layers refuses the same switch at ingest)")
            # ...nor override another layer's switch that holds it on (the
            # last switch in force decides, so appending would do exactly that).
            held = sorted(c.id for c in cons if isinstance(c, TimeWindowSwitch)
                          and c.target_id == sw.target_id and c.active
                          and c.effective_layer != layer)
            if held:
                raise LayerRelaxation(
                    f"{sw.id}: {tgt.id!r} is held on by {held}, switch(es) of another "
                    f"layer; a {layer}-layer switch may not override them")
        if sw.valid_time is None:
            # An unscheduled switch is a standing command: it supersedes the
            # standing commands before it on the same rule, from its own layer
            # (another layer's rule is not this event's to delete).
            cons = [c for c in cons if not (isinstance(c, TimeWindowSwitch)
                                            and c.target_id == sw.target_id
                                            and c.valid_time is None
                                            and c.effective_layer == layer)]
        if any(c.id == sw.id for c in cons):
            raise RuleIdConflict(f"time_window_switch {sw.id!r}: the id already exists")
        return self._commit(cons + [sw], "switch_on" if sw.active else "switch_off",
                            sw.id, "time_window_switch", self._event_time(t))

    def set_rule_active(self, rule_id: str, active: bool, t: float | None = None,
                        switch_id: str | None = None, *,
                        layer: str = DEFAULT_LAYER) -> dict:
        """Hold `rule_id` on (True) or off (False) from now on: a hot-applied
        time_window_switch with no window of its own, from `layer`."""
        with self._lock:
            sid = switch_id or f"switch-{rule_id}-g{self.policy.generation + 1}"
            # The default layer stays implicit, so the switch (and the policy
            # hash) is the one this method wrote before it took a layer.
            # constraint_type / priority / violation_action are required since
            # IR schema 3.0.0; these are the values they defaulted to. `scope`
            # is required since 4.0.0: a switch held from now on belongs to
            # this mission (what an absent scope meant before).
            sw = TimeWindowSwitch(id=sid, type="time_window_switch",
                                  constraint_type="hard", priority="P0",
                                  violation_action="project_fix", scope="mission",
                                  target_id=rule_id, active=bool(active),
                                  layer=None if layer == DEFAULT_LAYER else layer)
            return self._switch(sw, t)

    # -- corridor_swap

    def _swap(self, sp: CorridorSwap, t: float | None) -> dict:
        if not isinstance(sp, CorridorSwap):
            raise HotApplyRefused(f"corridor_swap event must be a CorridorSwap, got "
                                  f"{type(sp).__name__}")
        _, tgt = self._find(sp.target_id)
        if not isinstance(tgt, Corridor):
            raise HotApplyRefused(f"{sp.id}: corridor_swap targets {tgt.id!r}, a "
                                  f"{tgt.type}, not a corridor (to swap again, target "
                                  f"the original corridor)")
        layer = sp.effective_layer
        if self._cross_layer_hard(tgt, layer):
            # The corridor the swap puts in force must be provably no looser
            # than the hard one it replaces: the test models.merge_layers runs
            # on a swap at ingest, on the same corridor it builds. For a merged
            # corridor (`locked_by`) the winner stands in for the overridden
            # lower copies: it is no looser than each of them, and each check
            # _not_looser makes is transitive.
            as_corr = Corridor(id=tgt.id, type="corridor",
                               constraint_type=sp.constraint_type,
                               priority=sp.priority, violation_action=sp.violation_action,
                               scope=sp.effective_scope,   # compared, never stored
                               valid_time=sp.valid_time, altitude_ref=sp.altitude_ref,
                               centerline=sp.centerline, width_m=sp.width_m,
                               altitude_floor_m=sp.altitude_floor_m,
                               altitude_ceiling_m=sp.altitude_ceiling_m)
            why = _not_looser(as_corr, tgt)
            if why:
                locks = getattr(tgt, "locked_by", None) or ()
                over = f", overriding a hard {'/'.join(locks)}-layer copy," if locks else ""
                raise LayerRelaxation(
                    f"{sp.id}: the swap {why}, relaxing hard {tgt.effective_layer}-layer "
                    f"corridor {tgt.id!r}{over} from the {layer} layer (Policy DSL page, "
                    f"layered authoring; models.merge_layers refuses the same swap)")
        cons = list(self.policy.constraints)
        if sp.valid_time is None:
            cons = [c for c in cons if not (isinstance(c, CorridorSwap)
                                            and c.target_id == sp.target_id
                                            and c.valid_time is None
                                            and c.effective_layer == layer)]
        if any(c.id == sp.id for c in cons):
            raise RuleIdConflict(f"corridor_swap {sp.id!r}: the id already exists")
        return self._commit(cons + [sp], "swap", sp.id, "corridor_swap",
                            self._event_time(t))

    # ---------------- one rule checker: read-only views ---------------- #

    def rule_status(self, state: State, action: Action4D | None = None, *,
                    subject=None, subject_class=None, use_live_subject: bool = True,
                    clearance_m: float | None = None, ahead_s: float = 0.0) -> list[dict]:
        """Each rule's distance, state and breach, from this Shield's own
        geometry, in policy order.

        For the on-screen panel and the controller's FenceGuard, so neither
        keeps a second copy of the rules (grant, Safety Shield page: "there is
        exactly one rule-evaluation code path in the system"). Nothing here
        decides an action; it reports the same tests the monitor runs, at the
        aircraft's position now:

          in_force  the rule is in force now (both sides of the instant)
          bound     it applies to the present situation (a stand-off with a
                    subject it binds, a zone whose band holds the altitude,
                    a clearance rule with a map)
          breach    standing still here breaks it - the test state_is_unsafe
                    runs (for the speed rule: `action` breaks it)
          value     the measured quantity (separation, clearance, altitude,
                    lateral offset, speed, distance to the zone's margin ring)
          distance_m  room left before the limit (negative = past it); for a
                    zone, the distance to its margin ring (0 inside)

        Every row also carries `rule`, the constraint itself, from the same
        snapshot of the rule set as the row (an event landing between two
        calls cannot pair a row with another rule). Zones add `inside` (the
        authored polygon), `in_margin` (the ring, not the polygon), `in_band`,
        `vertices` (at their current position) and `margin_m`. `subject` / `subject_class` override the Shield's own
        subject when `use_live_subject` is False (an offline re-render passes
        the logged estimate); `clearance_m` overrides the distance-field read
        for a caller whose Shield has no map but measured it with the flight's.
        `ahead_s` (since 2026-10-19) answers for `ahead_s` seconds after this
        tick instead (`looking_ahead`): the rules in force then, moving zones
        where they will be, a moving subject advanced; 0.0 is exactly the
        call it always was. The hold-point search (guardrail/holdpoint.py)
        judges a transit's poses with it.
        """
        if ahead_s:
            with self.looking_ahead(ahead_s) as tick:
                return self._rule_status_in(tick, state, action, subject,
                                            subject_class, use_live_subject, clearance_m)
        tick = self._tick_ctx()
        if tick is None:
            with self._scope(self._new_tick(None)) as tick:
                return self._rule_status_in(tick, state, action, subject,
                                            subject_class, use_live_subject, clearance_m)
        return self._rule_status_in(tick, state, action, subject, subject_class,
                                    use_live_subject, clearance_m)

    def _rule_status_in(self, tick, state, action, subject, subject_class,
                        use_live_subject, clearance_m) -> list[dict]:
        rt = tick.rt
        recs = {r.rule.id: r for r in rt.ir.fences}
        if use_live_subject:
            subject, subject_class = self._subject, self._subject_class
            if subject is not None and tick.ahead:
                # A look-ahead view: the subject advanced by its velocity, as
                # the stand-off check reads it (_subject_track).
                subject = self._subject_track()[:2]
        P = Point(state.x, state.y)
        out = []
        for i, c in enumerate(rt.constraints):
            m = tick.sched.mask(c)
            in_force = m is True or (m is not False and m(0.0))
            row = {"index": i, "id": c.id, "type": c.type, "rule": c,
                   "priority": c.priority,
                   "constraint_type": c.constraint_type,
                   "violation_action": c.violation_action,
                   "effective_action": rt.eff.get(c.id), "in_force": in_force,
                   "bound": False, "breach": False, "value": None, "distance_m": None}
            if c.type in FENCE_TYPES:
                rec = recs[c.id]
                if rec.moving:
                    rec = tick.current(rec)
                inside = rec.polygon.contains(P)
                in_ring = rec.buffered.contains(P)
                in_band = in_zone_band(c, state.up)
                d = 0.0 if in_ring else rec.buffered.distance(P)
                row.update(bound=in_force and in_band, inside=inside,
                           in_margin=in_ring and not inside, in_band=in_band,
                           value=d, distance_m=d, margin_m=float(c.margin_m),
                           vertices=[(float(x), float(y)) for x, y in
                                     list(rec.polygon.exterior.coords)[:-1]],
                           breach=in_force and zone_breached(
                               c, state.x, state.y, state.up, rec.polygon, rec.buffered))
            elif c.type == "altitude_envelope":
                below, above = self._alt_outside(c, state.up)
                row.update(bound=in_force, value=state.up,
                           distance_m=min(state.up - c.alt_min_m, c.alt_max_m - state.up),
                           breach=in_force and (below or above))
            elif c.type == "kinematic_envelope":
                if action is not None:
                    spd = math.hypot(action.vx, action.vy)
                    row.update(bound=in_force, value=spd, distance_m=c.speed_max_mps - spd,
                               breach=in_force and bool(self._check_kinematic(c, action)))
            elif c.type == "subject_standoff":
                if subject is not None and c.binds(subject_class):
                    sep = math.hypot(state.x - subject[0], state.y - subject[1])
                    # A non-finite separation (a NaN subject handed in by an
                    # offline re-render) cannot be judged: not bound, rather
                    # than "bound, not breached" on a comparison NaN fails.
                    if math.isfinite(sep):
                        row.update(bound=in_force, value=sep,
                                   distance_m=sep - c.min_range_m,
                                   breach=in_force and self._standoff_inside(c, sep))
            elif c.type == "obstacle_clearance" and _field_name(c) == "roads":
                # A distance_envelope over roads (WP1-07): its own map.
                f = self._field_for(c)
                d = self._distance_at(state.x, state.y, f) if f is not None else None
                if d is not None and math.isfinite(d):
                    row.update(bound=in_force, value=d, distance_m=d - c.min_clearance_m,
                               breach=in_force and d < c.min_clearance_m)
            elif c.type == "obstacle_clearance":
                d = clearance_m if clearance_m is not None else (
                    self._distance_at(state.x, state.y) if self._dist is not None else None)
                row["off_map"] = self.off_map(state.x, state.y)
                if d is not None and math.isfinite(d):
                    row.update(bound=in_force, value=d, distance_m=d - c.min_clearance_m,
                               breach=in_force and d < c.min_clearance_m)
            elif c.type in ("corridor", "corridor_swap"):
                off, _ = self._corridor_geom(c, state.x, state.y)
                in_band = corridor_in_band(c, state.up)
                row.update(bound=in_force, value=off, distance_m=c.half_width_m - off,
                           in_band=in_band, centerline=list(c.points()),
                           width_m=float(c.width_m),
                           breach=in_force and (corridor_wide(c, off) or not in_band))
            elif c.type == "time_window_switch":
                row.update(target_id=c.target_id, active=c.active)
            out.append(row)
        return out

    @staticmethod
    def _alt_outside(env, up: float) -> tuple[bool, bool]:
        """(below the floor, above the ceiling): the altitude check's own
        comparison at a pose."""
        return up < env.alt_min_m - 1e-9, up > env.alt_max_m + 1e-9

    def zones_now(self, up: float | None = None) -> list[FenceRecord]:
        """The keep-out zones in force now, at their current position (moving
        dynamic zones materialised), in policy order. With `up`, only zones
        whose altitude band holds it; without, every zone's footprint (the
        conservative 2-D reading a controller uses)."""
        tick = self._tick_ctx()
        if tick is None:
            with self._scope(self._new_tick(None)) as tick:
                return self._zones_now_in(tick, up)
        return self._zones_now_in(tick, up)

    def _zones_now_in(self, tick, up) -> list[FenceRecord]:
        out = []
        for r in tick.rt.ir.fences:
            m = tick.sched.mask(r.rule)
            if not (m is True or (m is not False and m(0.0))):
                continue
            if up is not None and not in_zone_band(r.rule, up):
                continue
            out.append(tick.current(r) if r.moving else r)
        return out

    def zone_ruler(self, up: float | None = None) -> ZoneRuler:
        """Distances to the margin rings of the zones in force now
        (`zones_now(up)`), read once for as many points as the caller asks
        about: the measure `fence_distance` reports, for a reader that needs
        it at many points in one control tick (demo/follow_vlm.py FenceGuard,
        which until 2026-11-03 measured the rings itself)."""
        return ZoneRuler(self.zones_now(up))

    def fence_distance(self, x: float, y: float, up: float | None = None) -> float:
        """Metres from (x, y) to the nearest in-force zone's margin ring (0
        inside it); inf with no zone. The geometry the zone rule enforces."""
        return self.zone_ruler(up).distance(x, y)

    def enforces_hard(self, rule_id: str) -> bool:
        """Is `rule_id` a HARD rule this Shield enforces (its effective action
        is not monitor_only), in the rule set it is enforcing now? The
        escalation FSM's own table (rt.hard, rt.eff), for a reader that
        selects the rules a fail-safe must respect (guardrail/rtl_path.py)
        instead of re-reading the rule's fields."""
        tick = self._tick_ctx()
        rt = tick.rt if tick is not None else self._rt
        return bool(rt.hard.get(rule_id, False)) and \
            rt.eff.get(rule_id) not in (None, "monitor_only")

    # ---------------- the public entry point ---------------- #

    def filter(self, state: State, raw: Action4D, t: float | None = None, *,
               rtl_failed: bool = False, home_reached: bool = False,
               landed: bool = False, hold_off_m: float | None = None,
               hold_reach_m: float | None = None) -> ShieldDecision:
        """One monitor tick: check, repair, re-check, the brake/rescue
        fallbacks, then the escalation FSM. Every decision, whichever branch
        produced it, goes into the sliding window (`history`) before it is
        returned, with the time the decision took.

        `t` is the monitor clock in seconds (monotonic; the simulation or the
        rail's clock). Without it, ticks are counted at the grant's 10 Hz.

        rtl_failed / home_reached / landed: what the autopilot reports this
        tick, for the FSM's terminal edges (G6, G9, G10, X5). Only a caller
        that flies the Shield's own FSM verdict passes them; the flight rails
        run their own FSM (guardrail/replay.py rail_shield) and leave them.
        hold_off_m / hold_reach_m: in Loiter, how far the aircraft is from the
        point its hold flies to and how far a kept hold carries it (X9, since
        2026-11-11; guardrail/hold_flight.py HoldWatch), from the caller that
        flies the hold; None: not judged."""
        t0 = time.perf_counter()
        tick = self._begin_tick(t)
        if (getattr(state, "velocity", None) is not None and self.velocity_response is not None
                and self._velocity_of(state) is None):
            self.n_velocity_nonfinite += 1
        with self._scope(tick):
            # The state the decision is made from: `state`, or `state` without
            # its measured velocity while the position breaks a hard rule.
            dstate = self._decision_state(tick, state)
            if self._velocity_of(state) is not None:
                if dstate is state:
                    self.n_velocity_ticks += 1
                else:
                    self.n_velocity_set_aside += 1
            self._forget_left_openings(dstate)
            # The action subscriber -> the sliding-window buffer -> the
            # violation checker (grant node architecture): the action (made
            # finite, as _decide makes it) and its predicted trajectory from
            # the decision state go into the buffer first, and every rule
            # that judges this action reads that trajectory (_predict).
            admitted, _ = _sanitise(raw)
            self._buffer.admit(tick, dstate, admitted, self._predict(dstate, admitted))
            decision = self._decide(dstate, raw)
            # Zones this tick saw opening on the aircraft: their escape flies
            # at the cap on every later tick until the aircraft is out of the
            # ring (`_opened_on`).
            self._opened_on.update((decision.trace or {}).get("window_opens_on") or ())
            self._last_caps = self._applied_caps(tick, decision)
            decision = self._escalate(tick, dstate, decision, rtl_failed=rtl_failed,
                                      home_reached=home_reached, landed=landed,
                                      hold_off_m=hold_off_m, hold_reach_m=hold_reach_m)
            # What this tick streams, for the dead reckoning while the
            # position is not a valid fix (State.fix_ok, X8).
            self._note_streamed(tick, decision, dstate)
            # The forecast's inputs the State does not hold (TickRecord.pipe),
            # for a tick that forecast from a measured velocity only.
            pipe = (tuple(self.pipe()) if self._velocity_of(dstate) is not None
                    else None)
            # A tick forecast along the arc (filter_body with arc, a yaw rate
            # over ARC_MIN_RATE): said in the trace, which the audit record
            # copies (`forecast_arc`), and on the TickRecord, so the tick's
            # forecast is rebuilt with forecast(..., arc=True). Absent on
            # every other tick, which is recorded as it always was.
            arced = bool(self._arc_rate(admitted))
            if arced:
                decision.trace = {**(decision.trace or {}), "forecast_arc": True}
        elapsed_ms = (time.perf_counter() - t0) * 1e3
        self._buffer.close(dstate.model_copy(), decision, elapsed_ms, pipe, arc=arced)
        return decision

    def filter_body(self, state: State, body: Action4D, t: float | None = None,
                    *, arc: bool | None = None, **kw) -> tuple[ShieldDecision, Action4D]:
        """One monitor tick with the action in the grant's BODY frame.

        The grant (WP3-04, ARCH-04): "Action input frame is body frame (vx,
        vy, vz, yaw_rate); body->local-NED conversion happens after
        projection, in the MAVLink adapter." `body` carries the body-frame
        action in an Action4D: vx forward, vy right, vz_up up, yaw_rate rad/s
        (as /vla/action does). Returns `(decision, emitted_body)`: the tick's
        ShieldDecision, whose raw and emitted actions are in the world frame
        the logs and the audit record read, and the emitted action back in
        the body frame, for the MAVLink adapter to turn into local NED. That
        conversion is the adapter's alone (`sitl/mavlink_adapter_node.py`,
        frames.to_local_ned).

        The action is turned into the world frame at the state's heading
        (frames.from_body), filtered by `filter`, and turned back at the same
        heading (frames.to_body). Until 2026-11-04 the ROS 2 Shield node made
        those two turns itself around `filter()`; the Shield now takes the
        body frame at its own boundary, and the node turns nothing.

        Why the repair is the body-frame one: the rules are world geometry
        (a fence on the ground, a distance to a point, an altitude band), and
        the repairs are geometric - rotating the action and the scene together
        rotates the repair with them; the speed cap is a norm, and the
        vertical and yaw caps do not involve the heading. So repairing the
        rotated action against the world geometry is the body-frame repair,
        rotated, without rotating every rule's geometry on every tick. One
        choice is not of that kind: a tie between equally near edges with no
        heading to break it goes to the first along the ring's stored
        (world) order (`escape_direction`).

        THE ARC (`arc`, 2026-11-17; None: the Shield's `arc_forecast`, off
        by default). A body-frame command holds its forward / right velocity
        while the heading turns at its yaw rate, so it flies an ARC; with
        `arc` on, the forecasts of this tick that `_predict` makes - the
        admitted action's, each repair candidate's, the re-checks' - add the
        ARC (the heading integrated over the lookahead, `_arc_offset`) to
        the straight line, and a rule is broken if either breaks it (THE ARC
        ONLY ADDS, review 2026-11-17: the arc in place of the line forecast
        a held yaw rate for 5 s, so a jittering yaw output saw a zone dead
        ahead 9-14 m late). A zone the arc reaches but the line misses is
        caught; nothing the line catches is lost. With it off the forecast
        holds the action's world direction from t = 0, as it did before:
        such a command is not forecast along its arc. A yaw rate under
        ARC_MIN_RATE is forecast exactly as before either way (the arc IS
        the line). Every repair's yaw rate is the action's own except the
        kinematic cap's, which clamps it; each candidate is forecast on its
        own yaw rate. The emitted body action's arc is recorded on the tick
        (trace "forecast_arc", the audit record's `forecast_arc`,
        TickRecord.arc).

        WHAT STAYS STRAIGHT inside such a tick (each still judged, never on
        the arc alone): the escape shaping for a zone that opened on the
        aircraft (`_shaped_escape`: off for an arced action, which is then
        judged on the full 5 s line and arc - the 69c27bd behaviour for that
        case, pinned in tests/test_body_arc_forecast.py); `_ring_exit_time`,
        `_opening_imposed` and the inside rule's escape point at the opening
        (`t_open`); the own-velocity "stop after one control period"
        clearance forecast (`_clear_steps` with a measured velocity); the
        repair operators' own geometry (they steer in the world frame).
        Outside this call the Shield's queries (state_is_unsafe,
        recheck_commanded, advise, handback_unsafe) are straight.

        `t` and the other keyword arguments are `filter`'s."""
        raw = _from_body(body.vx, body.vy, body.vz_up, body.yaw_rate, state.yaw_deg)
        on = self.arc_forecast if arc is None else bool(arc)
        prev_arc = getattr(self._local, "arc", False)
        self._local.arc = on
        try:
            decision = self.filter(state, raw, t, **kw)
        finally:
            self._local.arc = prev_arc
        fwd, right, up, yaw_rate = _to_body(decision.emitted, state.yaw_deg)
        return decision, Action4D(vx=fwd, vy=right, vz_up=up, yaw_rate=yaw_rate)

    def _decision_state(self, tick: _Tick, state: State) -> State:
        """The state a decision is made from: `state` itself, except that a
        measured velocity is SET ASIDE while the POSITION already breaks a
        hard, enforced rule (a stop here is illegal: the FSM's stop_illegal).

        Why (2026-10-20 review, the open-loop replay of every recorded flight
        with the Shield's revised numbers, chosen so that their MEDIAN
        braking error at 1.5 s is not short - guardrail/response.py): on
        the Shield-off close run (pas_ap_close_off ticks 733-739) the logged
        aircraft was 0.8-1.0 m inside the building footprint, still flying
        deeper at 1.2-1.8 m/s, 3.8-4.9 m from the car. The building rule,
        broken by the position, was judged as before (`_committed`), but the
        car's stand-off ring was "committed" - a stop would end inside it -
        and judged from the stop point, where only an action OPENING the
        range passes. Leaving the building and opening the range from the
        stop point had no common action, the chain did not converge, and the
        rescue (ClearanceEscape) flew a 5 m/s exit that broke the stand-off
        on the commanded forecast too: 7 P0 escapes where the Shield without
        the velocity found a slow push out that broke nothing. The momentum
        adds constraints; from an illegal position, where the escape must be
        found, they can leave none, and the decision falls to a rescue.

        So while the position is illegal the decision is the commanded
        forecast's, exactly as before the change (byte for byte: the same
        state the old Shield saw) - which is what section 9 of the design
        note already said of the broken rule itself ("while the position is
        already inside a rule, recovery is exactly the old one"), now true of
        every rule. The velocity returns the tick the position is legal.
        Counted (`n_velocity_set_aside`).

        ALL OR NOTHING ACROSS RULES (2026-10-20 re-review). Any broken hard,
        enforced rule sets the velocity aside for EVERY rule, so an unrelated
        breach also drops the momentum the building rule would have seen.
        Replayed open loop on the six recorded pas_ap flights (causal
        velocity, the logged pipe; tools/own_velocity_closed_loop.py's
        replay): on pas_ap_20261008_224440 it is set aside on 310 of 1101
        ticks, 279 of them for `alt-band` alone (seven spans of the follow
        below its 6 m floor, ticks 218-1045 - not take-off or landing; at
        most 0.66 m/s horizontally), 25 for the building, 6 for the
        stand-off; on the other five flights every set-aside tick is the
        building rule itself (12, 720, 78, 739 and 71 ticks). During such an
        alt-band span a fast approach to a building would be judged on the
        commanded forecast alone. Setting aside only the BROKEN rule is not
        the cure: in the 7-escape case above the rule whose committed
        handling left no escape was the stand-off ring, which was not broken.
        A per-rule version would have to set aside the rules whose committed
        judgement conflicts with the escape, which needs evidence of its own
        (the re-review found the closed-loop results unchanged with the
        set-aside disabled, on both windows and all three aircraft) - an open
        item in the design note (section 9), not done here."""
        if self._velocity_of(state) is None:
            return state
        here = self._positional(state)
        prev, tick.skip = tick.skip, frozenset()
        try:
            inside = self._check(here, BRAKE)
        finally:
            tick.skip = prev
        rt = tick.rt
        if any(rt.hard.get(v.rule_id, True) and self._act(rt, v.rule_id) != "monitor_only"
               for v in inside):
            return here
        return state

    def advise(self, state: State, raw: Action4D) -> ShieldDecision:
        """The decision filter() WOULD make for `raw` from `state`, at the last
        tick's time, without making it: nothing goes into the sliding window,
        the monitor clock and the escalation FSM do not move, and the
        floor-refusal counter is left as it was.

        For a rail that must stream something other than the pilot's action
        on a tick whose decision it has already taken - the hold that stops
        clear (guardrail/response.py `hold_plan`): the Shield's answer to a
        stop (`raw` zero) from a state with a measured velocity is a stop
        where the stop ends legal, and the recovery the stop point needs
        where the aircraft's momentum carries it into a rule."""
        n_refused = self.n_floor_refused
        try:
            with self._scope(self._new_tick(None)) as tick:
                return self._decide(self._decision_state(tick, state), raw)
        finally:
            self.n_floor_refused = n_refused

    def _begin_tick(self, t: float | None) -> _Tick:
        if t is None:
            t = 0.0 if self._t_last is None else self._t_last + MONITOR_PERIOD_S
        t = float(t)
        if self._t_first is None:
            self._t_first = t
            if self._carry is not None:
                self._resume_moving_zones(t)
        self._ticks += 1
        self._t_last = t
        return self._new_tick(t)

    def reset_episode(self) -> None:
        """Start a new episode on the same Shield: escalation state, its fault
        latch, the sliding window and the monitor clock, and the mission-start
        lock (a polygon_fence may be added again before the first tick). The
        rule set and its generation are kept (events are part of the policy's
        history), and so is every zone's position: a moving zone continues
        from where the last episode left it.

        Before the fix (2026-10-07 review) a zone spawned at t = 120 s jumped
        back to its spawn position on reset and stood still until the new
        episode's counted clock passed 120 s again."""
        with self._lock:
            if self.fsm is not None:
                self.fsm.reset()
            planner = getattr(self, "_hold_planner", None)
            if planner is not None:
                planner.reset()                 # a search of the last episode's Loiter
            self._fsm_fault = None
            self._last_caps = None
            self._buffer.clear()
            self._flown.clear()
            self._opened_on.clear()
            self._pk_good = self._pk = self._pk_cmd = None
            rt = self._rt
            if rt.moving_ids and self._t_last is not None:
                # Pin every moving zone to an explicit anchor (a policy-authored
                # one is anchored at the first tick, which is about to change)
                # and remember its age; _begin_tick re-anchors it if the next
                # episode's clock starts earlier than this one ended.
                anchors = dict(self._anchors)
                ages = {}
                for zid in rt.moving_ids:
                    a = anchors.get(zid)
                    if a is None:
                        a = anchors[zid] = self._t_first
                    ages[zid] = max(0.0, self._t_last - a)
                self._anchors = anchors
                self._rt = replace(rt, anchors={k: v for k, v in anchors.items()
                                                if k in rt.moving_ids})
                self._carry = (self._t_last, ages)
            self._ticks = 0
            self._t_last = self._t_first = None

    def _resume_moving_zones(self, t: float) -> None:
        """First tick after reset_episode(). On a monotonic clock that went on
        (t at or after the last episode's end) the absolute anchors already
        give each zone its age; on a clock that restarted (the counted clock
        does, at 0) each zone is re-anchored so its age carries on from the
        reset."""
        with self._lock:
            t_prev, ages = self._carry
            self._carry = None
            if t >= t_prev:
                return
            rt = self._rt
            anchors = dict(self._anchors)
            for zid, age in ages.items():
                if zid in anchors:
                    anchors[zid] = t - age
            self._anchors = anchors
            self._rt = replace(rt, anchors={k: v for k, v in anchors.items()
                                            if k in rt.moving_ids})

    # ---- what the policy says to do about a rule

    @staticmethod
    def _act(rt: _Runtime, rule_id: str) -> str:
        # A violation of no policy rule (the Shield's own `action-finite`
        # contract) is a hard P0 repair, as fsm.rules_from_policy reads it.
        return rt.eff.get(rule_id, "project_fix")

    def _stop_illegal(self, tick: _Tick, state: State) -> bool:
        """Would standing still here break a HARD, enforced rule? (A soft rule
        is capped at brake: a stop that breaks only a soft rule is a response
        that rule already allows.) Read once per tick, on the full rule set."""
        if tick.unsafe is None:
            prev, tick.skip = tick.skip, frozenset()
            try:
                tick.unsafe = self._check(state, BRAKE)
            finally:
                tick.skip = prev
        rt = tick.rt
        return any(rt.hard.get(v.rule_id, True)
                   and self._act(rt, v.rule_id) != "monitor_only" for v in tick.unsafe)

    def _stack(self, base: tuple) -> tuple:
        """An operator stack (names): the caller's `repair_order` in place of
        `base` when one was given, with PathRepair when it is switched on."""
        stack = self.repair_order if self.repair_order is not None else base
        if self.path_repair and "PathRepair" not in stack:
            return _repair.with_path_repair(stack)
        return stack

    def _repair_chain(self, tick: _Tick, state: State, raw: Action4D,
                      skip: frozenset, repairs: list, raw_violations=()
                      ) -> tuple[Action4D, list, list, list]:
        """The repair stack, run for every enforced rule but those in `skip`.
        Returns (repaired action, repairs, what it still violates, passes):
        `passes` is [(pass number, [RepairAttempt, ...]), ...], every operator
        considered in every pass (pass 0 runs before the iterated loop); it is
        also kept on the tick's ledger as this run's (`tick.ledger`).

        The default mode runs the operators of guardrail/repair.py in the
        order this method always called the `_repair_*` methods it wraps,
        with the same arguments, so its decisions are the old chain's."""
        prev, tick.skip = tick.skip, frozenset(skip)
        tick.chain_runs += 1
        ops, ir = self.repair_operators, tick.rt.ir
        ctx = _repair.RepairContext(self, state, repairs, raw, tuple(raw_violations),
                                    tuple(raw_violations))
        passes: list = []
        tick.ledger.append((tick.chain_runs, passes))
        try:
            if self.repair_mode == "first_feasible":
                fixed = self._first_feasible(tick, ctx, passes)
            else:
                fixed, pre = raw, []
                for name in _repair.CHAIN_PRE:
                    att = ops[name].repair(fixed, None, ir, ctx=ctx)
                    pre.append(att)
                    fixed = att.action
                passes.append((0, pre))
                # Clearance and geofence are COUPLED: the anti-stall tangent can
                # leave the clearance ring violated and the clearance push can aim
                # into a fence, so the chain is iterated to a fixed point. Running
                # it once as a pipeline leaves whatever the LAST operator produced
                # un-repaired, which is exactly what the P0 guard then brakes on -
                # every tick, forever. The caps close each pass: AltitudeFix
                # never recovers slower than the band in one lookahead, which
                # can overshoot the climb cap on a deep recovery.
                loop = self._stack(_repair.CHAIN_LOOP)
                for k in range(1, _REPAIR_PASSES + 1):
                    ctx.pass_no = k
                    atts = []
                    for name in loop:
                        att = ops[name].repair(fixed, None, ir, ctx=ctx)
                        atts.append(att)
                        fixed = att.action
                    passes.append((k, atts))
                    still = self._check(state, fixed)
                    if not still:
                        break
                    ctx.open_violations, ctx.open_label = tuple(still), f"after pass {k}"
            # P0 escape guard: repaired action must re-check clean.
            blocking = self._check(state, fixed)
        finally:
            tick.skip = prev
        return fixed, repairs, blocking, passes

    def _first_feasible(self, tick: _Tick, ctx, passes: list) -> Action4D:
        """The grant's selection (Safety Shield page: "Tried in priority order.
        First operator that produces a feasible action with magnitude <=
        threshold wins", inside "a bounded loop (default 3 iterations)").

        Each iteration targets the earliest open violation. Operators are
        tried in `repair.FIRST_FEASIBLE` order; one whose class does not cover
        the target is skipped. An attempt is FEASIBLE when the monitor no
        longer reports the target on its action and the repairs the tick
        would then carry are within theta as the FSM reads it: summed per
        axis over this attempt's and every earlier iteration's (2.0 m lateral
        / 0.5 m vertical by default; recoveries and clamps are exempt, as in
        the FSM; the reference checks each attempt alone). The first
        feasible attempt is applied and the next iteration re-checks; when no
        operator is feasible the loop ends and `_decide`'s fallback (rescue,
        then brake where a stop is legal) decides. An attempt that is not
        applied leaves no Repair behind."""
        state, ir, ops = ctx.state, tick.rt.ir, self.repair_operators
        cfg = self.fsm.config if self.fsm is not None else _fsm.FSMConfig()
        order = self._stack(_repair.FIRST_FEASIBLE)
        fixed = ctx.raw
        open_ = self._check(state, fixed)
        ctx.open_violations = tuple(open_)
        for k in range(1, _REPAIR_PASSES + 1):
            if not open_:
                break
            ctx.pass_no = k
            target = min(open_, key=lambda v: v.predicted_at_s)
            atts, won = [], False
            for name in order:
                op = ops[name]
                if target.category not in op.categories:
                    atts.append(_repair.RepairAttempt(
                        name, "skip", fixed,
                        f"not a {target.category} violation ({target.rule_id})", (), k))
                    continue
                trial = list(ctx.repairs)
                att = op.repair(fixed, target, ir, ctx=replace(ctx, repairs=trial))
                if att.result != "ok":
                    atts.append(att)
                    continue
                after = self._check(state, att.action)
                if any(v.rule_id == target.rule_id for v in after):
                    atts.append(replace(att, result="fail",
                                        reason=f"{target.rule_id} still violated: {att.reason}"))
                    continue
                # Theta as the FSM reads it (fsm.magnitude_from_repairs): the
                # per-axis SUM over every repair the tick would carry, the
                # ones already applied in earlier iterations included,
                # deduplicated as the decision's repairs are, with the FSM's
                # own exemptions (theta_governs). Until 2026-10-19 each
                # Repair of this attempt was compared alone, so two 1.5 m
                # slides (two zones, or a slide and a stand-off hold in two
                # iterations) were "within theta" here and 3.0 m over it in
                # the FSM, which then abandoned the projection (X1) the
                # selection had just accepted. (The PI reference checks each
                # attempt alone; it has no FSM to agree with.)
                gov = [r for r in _dedupe(list(ctx.repairs) + list(att.repairs))
                       if r.magnitude_m is not None and _fsm.theta_governs(r)]
                tot = {ax: sum(r.magnitude_m for r in gov
                               if (r.axis == "vertical") == (ax == "vertical"))
                       for ax in ("lateral", "vertical")}
                cap = {"lateral": cfg.theta_lateral_m, "vertical": cfg.theta_vertical_m}
                over = [ax for ax in ("lateral", "vertical") if tot[ax] > cap[ax] + _fsm.EPS]
                if over:
                    ax = over[0]
                    atts.append(replace(att, result="fail", reason=(
                        f"{tot[ax]:.2f} m over theta ({ax}) summed over the tick's "
                        f"repairs, as the FSM reads it: {att.reason}")))
                    continue
                ctx.repairs[:] = trial
                fixed, open_ = att.action, after
                ctx.open_violations, ctx.open_label = tuple(after), f"after pass {k}"
                atts.append(att)
                won = True
                break
            passes.append((k, atts))
            if not won:
                break
        return fixed

    def _relax(self, tick: _Tick, state: State, raw: Action4D, pool: list,
               skip: frozenset, head: list, raw_violations=()):
        """Give up soft rules, lowest priority first, until the hard ones (and
        the soft ones still kept) can all be satisfied (card WP1-12; the grant's
        severity order "hard >> soft, P0 > P1 > P2", Prefix Compiler page).
        Returns (action, repairs, relaxed ids, passes) or None, `passes` being
        the run that succeeded (every run, failed ones too, is on the tick's
        ledger). A hard rule is never given up: when hard rules conflict, the
        fallback below stops or recovers, and the FSM escalates."""
        rt = tick.rt
        prio = {c.id: c.priority for c in rt.constraints}
        soft = {v.rule_id for v in pool if not rt.hard.get(v.rule_id, True)
                and self._act(rt, v.rule_id) != "monitor_only"}
        if not soft:
            return None
        given: set = set()
        for level in ("P2", "P1", "P0"):
            add = {r for r in soft if prio.get(r, "P0") == level}
            if not add:
                continue
            given |= add
            fixed, reps, blocking, passes = self._repair_chain(
                tick, state, raw, skip | frozenset(given), list(head), raw_violations)
            if not blocking:
                broken = {v.rule_id for v in self._check(state, fixed)}
                tick.given_up = frozenset(given)
                return fixed, reps, sorted(given & broken), passes
        return None

    def _window_edge(self, tick: _Tick, violations: list) -> list[str]:
        """Rules whose violation a window edge inside the lookahead caused:
        not in force now (on either side of the instant) yet violated later
        in the forecast, so it comes into force in between. The grant's third
        violation category, "time-window" (Safety Shield page). Empty when no
        schedule has an edge in the horizon (`_Schedule.const`)."""
        rt = tick.rt
        if not rt.timed or tick.sched.const:
            return []
        by_id = {c.id: c for c in rt.constraints}
        out = set()
        for v in violations:
            c = by_id.get(v.rule_id)
            if c is None or c.id not in rt.timed:
                continue
            m = tick.sched.mask(c)
            if m is not True and m is not False and not m(0.0):
                out.add(c.id)
        return sorted(out)

    @staticmethod
    def _traced(d: ShieldDecision, state: State, tick: _Tick, passes: list,
                final: list, edge: list, mode: str, flags: dict,
                first: list = ()) -> ShieldDecision:
        """Attach the audit trace (ShieldDecision.trace) to a decision.

        `passes` is the chain run whose action was flown; the ledger holds
        EVERY run of the tick (`tick.ledger`: a run that failed before soft
        rules were given up too), each pass tagged with its run (1, 2, ...;
        None on the one synthetic pass of a tick where no chain ran).
        `first` holds what ran before the chain (Sanitise), recorded at the
        head of the first pass 0; `final` what came after it (SoftRelax, a
        fallback), recorded in the last pass any run made.

        `iterations` counts the loop passes (pass >= 1) of every run: the
        work done on the tick, 0 on a passthrough. `repair_iterations` is,
        per Repair of the decision (index for index), the pass of the flown
        run that first logged it - `_dedupe` keeps the first of each
        (operator, detail), so the same operator logging a new detail in a
        later pass gets that pass - 0 for Sanitise, None for the last
        resorts (Brake, ClearanceEscape, SoftRelax)."""
        led = [{"run": run, **p} for run, ps in tick.ledger
               for p in _repair.ledger_record(ps)]
        if not led or led[0]["pass"] != 0:
            led.insert(0, {"run": tick.ledger[0][0] if tick.ledger else None,
                           "pass": 0, "attempts": []})
        led[0]["attempts"][:0] = list(first)
        led[-1]["attempts"].extend(final)
        first_pass: dict = {}
        for k, atts in passes:
            for a in atts:
                if a.result == "ok":            # an attempt not applied logged nothing
                    for r in a.repairs:
                        first_pass.setdefault((r.operator, r.detail), k)
        d.trace = {"yaw_deg": float(state.yaw_deg), "mode": mode, **flags,
                   "passes": led,
                   "iterations": sum(1 for p in led if p["pass"] >= 1),
                   "chain_runs": tick.chain_runs,
                   "flown_run": next((run for run, ps in tick.ledger if ps is passes), None),
                   "repair_iterations": [0 if r.operator == "Sanitise"
                                         else first_pass.get((r.operator, r.detail))
                                         for r in d.repairs],
                   "window_edge": edge}
        return d

    def _floor_the_escape(self, tick: _Tick, state: State, raw: Action4D,
                          mode: str, flags: dict, violations: list = (),
                          edge: list = ()) -> "ShieldDecision":
        """The repair floors for an action that violates nothing ENFORCED
        (nothing at all, or only monitor_only rules: `violations`), from a
        position a stop would leave illegal (a hard, enforced rule:
        `_stop_illegal`). Returns the decision to fly: the floored action,
        or `raw` passed through as before.

        The monitors forgive a recovery on its trend: a zone left at more
        than 0.1 m/s, a ring opened at more than 0.1 m/s, ANY climb from
        under a floor, any forecast improvement inside the clearance ring.
        How fast is the repairs' business, and they FLOOR it
        (docs/FINDING-repairs-must-floor-the-escape.md): GeofenceEscape to
        min(2.0, cap) - to the cap itself for a zone whose window opened on
        the aircraft (`_opened_on`) - StandoffRecover and ClearanceFix to the rate that
        clears the ring in one lookahead (0.5 m/s at least), AltitudeFix and
        CorridorAltitudeFix to the climb cap (_recovery_vz; the band in one
        lookahead until 2026-10-29), CorridorReturn to the corridor in one
        lookahead. But the floors
        live in the repair chain, and until 2026-10-19 the chain ran only
        when SOME rule was violated. So the escape speed from a P0 breach
        depended on whether an unrelated rule happened to be broken: 1 m
        inside a zone leaving at 0.15 m/s passed untouched (6.7 s to leave)
        and the same action with a 60 dps yaw breach left at 2.0 m/s
        (0.5 s); a climb of 1e-6 m/s from 5 m under a floor passed every
        tick, forever, with zero violations; and the escape speed was not
        monotonic in the command (0.10 m/s out of a zone emitted 2.0,
        0.11 emitted 0.11). That gating is exactly the "decline" the
        finding measured as unsafe.

        So the chain runs here too - for the rules the POSITION breaks (the
        hard, enforced ones a stop would leave violated) and the speed and
        climb caps that close each pass, and for nothing else. Not the whole
        stack: a projection operator judges by its own model, not the
        monitor's, and on a legal action it can still find something to do
        (on the 8 Oct follow, ticks 777-780, climbing out from under the
        floor at 1.8 m/s, StandoffHold's radial reading took 2.4 m/s of
        closing off an action whose forecast never entered the ring).

        Not every repair of those rules, either: only the FLOORS. Even a kept
        rule's own operators have projection branches, and those judge by
        their own model too: outside a corridor laterally, the band forecast
        is not checked (the check is moot while returning), yet
        CorridorAltitudeFix still cut a legal 1.79 m/s climb to 0.28 (a
        4.55 m vertical projection); on another tick a non-recovery
        CorridorReturn of 3.34 m zeroed a legal vy (review of 2026-10-19,
        72 of 607 floored corridor ticks in a random scan). Nothing was
        violated, so the FSM read those ticks as clean and never applied
        theta to an over-theta projection, while the same action beside a
        yaw breach went Normal -> Brake [X1] on it. So a run is flown only
        if every repair it carries is one theta does not govern
        (`fsm.theta_governs`, read with the stop illegal, as the FSM reads
        this tick): the recovery floors and the speed and climb clamps. A
        run with any projection in it is not flown, and `raw` flies; the
        floors are what was missing, and reshaping a legal action is not.

        Nothing enforced was violated, so nothing enforced is reported (the
        FSM reads the tick as clean); the decision carries the floors'
        repairs, which makes it `touched`. It is flown only if it re-checks
        clean against EVERY rule but the monitor_only ones `raw` already
        broke (those are never repaired, are kept on the decision, and are
        in `emitted_violations` if the floored action still breaks them);
        otherwise `raw`, which broke nothing enforced, flies (a faster
        escape looks further ahead on the clearance forecast and can reach
        an obstacle `raw` did not, or a zone escape can aim at another zone)
        - never a brake or a relaxation, which would be the Shield making a
        legal action worse. A run that changes nothing, or is not flown, is
        taken off the ledger: the tick is the passthrough it always was. A
        run that changed the action and was not flown is counted
        (`n_floor_refused`), and the passthrough's trace says why
        (`floor_refused`): there the crawl the floor was for still flies
        (the altitude and corridor-band monitors forgive any climb, the
        clearance monitor any forecast gain), which is otherwise silent.

        monitor_only violations do not stop it: until a review of
        2026-10-19 such a tick took the monitor_only passthrough before the
        floor was asked, so 1 m inside the zone leaving at 0.15 m/s left at
        2.0 m/s alone, or beside an ENFORCED yaw breach, and at 0.15 beside
        a monitor_only one - the escape speed hanging on an unrelated rule
        again. First-feasible mode has no such floors to apply (its
        operators run only for a target violation, so an unrelated one
        floors nothing there either), so it is not asked.

        A ZONE THAT OPENED ON THE AIRCRAFT (review of b1d3aca, P1). Its
        escape is floored at the speed CAP (`window_opens_on`,
        `window_opened_on`; Shield._repair_geofence says why), and the cap's
        forecast reaches twice as far as the in-force floor's: with another
        hard zone within cap x lookahead (20 m) past the exit, the cap's
        escape broke it, was refused, and the crawl it was meant to floor
        flew as before - unfloored, legal under the inside rule, so the FSM
        stayed Normal and the KPI read 0 P0 escapes. In the library's
        07:29:58 cell with an always-on zone 13 m past the school zone's
        ring, a 0.5 m/s pilot stayed 104 ticks inside the polygon (14.4 s to
        safe) and a 0.2 m/s one never got out (279 ticks, censored), against
        26 ticks / 5.1 s on 3660383 (the 2.0 floor) and 3 / 2.6 s on
        7c9c9e6 (every tick escalated). So the cap's refusal is not the end:
        the run is made again with the escape slower (`_opened_retries`: the
        cap less 0.25 m/s at a time down to min(2.0, cap), then the in-force
        floor's own shape), and the FASTEST that re-checks clean is flown,
        its trace saying what the cap broke (`cap_refused`). If none can be
        flown and the action still leaves such a zone slower than
        min(2.0, cap), the tick is not passed as clean: it is reported as a
        violation of that zone (`_escape_shortfall`, in `violations` and
        `emitted_violations`), so the FSM escalates (Normal -> Brake [X1],
        then G3 into a Loiter whose hold point the search picks against
        every rule) as 7c9c9e6 did on every tick there, and the KPI counts
        what is flown meanwhile - `raw`, which breaks no other rule - as the
        P0 escape it is. Every run stays on the ledger then (`chain_runs`).

        WHAT REFUSES THE CAP (review of 69c27bd, P2). The re-check above
        walked the cap's straight forecast, 4 m/s held for the whole 5 s,
        and so refused the cap for any rule within 20 m of the exit - in the
        library's own geometry too: the corridor-survey-route edge at x = 80
        is 9 m past the school zone's ring at x = 71, and a pilot flying 2
        m/s east at the opening stayed 16 ticks inside the polygon, 4.0 s to
        safe, the escape slowed tick by tick and the FSM taken to a Loiter
        (7c9c9e6: 5 / 2.8 s). Every check of an action escaping such a zone
        now walks the escape as flown - out of the ring, then a stop
        (_shaped_escape) - so the retries and the report above apply only
        when a rule lies inside that stop (0.4 + 2.67 m past the exit from
        the cap on the grant arm's 0.1 s step). The trace keeps its words;
        the sweep counts both per episode (`cap_refused_ticks`,
        `floor_refused_ticks`) beside the P0 rate."""
        rt = tick.rt
        violations = list(violations)
        keep = {v.rule_id for v in tick.unsafe
                if rt.hard.get(v.rule_id, True) and self._act(rt, v.rule_id) != "monitor_only"}
        keep |= {k.id for k in rt.kins if self._act(rt, k.id) != "monitor_only"}
        skip = frozenset(c.id for c in rt.constraints if c.id not in keep)
        # The monitor_only rules `raw` already broke are not the floor's to
        # answer; any other rule broken is (skip is never empty here when
        # there are any: a monitor_only rule is never kept).
        allowed = {v.rule_id for v in violations}
        n_led, n_runs = len(tick.ledger), tick.chain_runs

        def run(setting):
            """One run of the floors, the opened zones' escape at `setting`
            (tick.opened_mps): (action, repairs, passes, its re-check, why it
            may not be flown or None)."""
            tick.opened_mps = setting
            try:
                fixed, repairs, blocking, passes = self._repair_chain(tick, state, raw, skip, [])
            finally:
                tick.opened_mps = None
            repairs = _dedupe(repairs)
            why = None
            after: list = []
            if not repairs or fixed == raw:
                pass                                    # nothing to floor
            elif blocking:
                why = "the floored action breaks " + ", ".join(
                    sorted({v.rule_id for v in blocking}))
            elif any(_fsm.theta_governs(r, stop_illegal=True) for r in repairs):
                why = ("the run carries a projection theta governs: " + ", ".join(
                    r.operator for r in repairs if _fsm.theta_governs(r, stop_illegal=True)))
            else:
                after = self._check(state, fixed) if skip else []
                new = sorted({v.rule_id for v in after if v.rule_id not in allowed})
                if new:
                    why = "the floored action breaks " + ", ".join(new)
            return fixed, repairs, passes, after, why

        fixed, repairs, passes, after, why = run(None)
        refused, flown_at = why, None
        if why is not None:
            for setting in self._opened_retries(flags):
                fixed, repairs, passes, after, why = run(setting)
                if why is None:
                    # Flown - or nothing left to floor: `raw` already leaves
                    # at least this fast, and every slower try is the same.
                    flown_at = setting
                    break
        if why is None and repairs and fixed != raw:
            d = self._traced(
                ShieldDecision(raw=raw, emitted=fixed, repairs=repairs,
                               violations=violations, emitted_violations=list(after)),
                state, tick, passes, [], list(edge), mode, flags)
            if refused is not None:
                at = ("the in-force floor" if flown_at == _IN_FORCE_FLOOR
                      else f"{flown_at:.2f} m/s")
                d.trace["cap_refused"] = f"flown at {at}; at the cap {refused}"
            return d
        short = self._escape_shortfall(state, raw, flags, refused) if refused else []
        if short:
            d = self._traced(ShieldDecision(raw=raw, emitted=raw, violations=violations + short,
                                            emitted_violations=violations + short),
                             state, tick, [], [], list(edge), mode, flags)
            self.n_floor_refused += 1
            d.trace["floor_refused"] = f"raw flown, reported to the FSM: {refused}"
            return d
        del tick.ledger[n_led:]
        tick.chain_runs = n_runs
        d = self._traced(ShieldDecision(raw=raw, emitted=raw, violations=violations,
                                        emitted_violations=list(violations)),
                         state, tick, [], [], list(edge), mode, flags)
        if refused is not None:
            self.n_floor_refused += 1
            d.trace["floor_refused"] = f"raw flown: {refused}"
        return d

    def _cap_zones(self, tick: _Tick, state: State) -> frozenset:
        """Under recovery_law "cap_all" (2026-11-06): the ids of the static,
        hard, enforced zones in force now whose margin ring holds the
        aircraft - every zone whose escape flies at the speed cap through the
        path a zone whose window opened on the aircraft already takes
        (`_repair_geofence`, `_shaped_escape`, `_opened_retries`,
        `_escape_shortfall`, `_slower_opened_escape`), whatever put the
        aircraft there: a zone grown or spawned over it, wind, a stale pose.
        Empty under the other laws. A moving zone is left out: the shaped
        escape is not defined for a moving ring (it is left out of the window
        path for the same reason), so its escape keeps the floor."""
        if self.recovery_law != "cap_all":
            return frozenset()
        rt = tick.rt
        out = set()
        for r in rt.ir.fences:
            if r.moving:
                continue
            f = r.rule
            if not rt.hard.get(f.id, True) or self._act(rt, f.id) == "monitor_only":
                continue
            alive = tick.sched.mask(f)
            if alive is False or not (alive is True or alive(0.0)):
                continue
            if point_in_fence(state.x, state.y, state.up, f, r.polygon, r.buffered):
                out.add(f.id)
        return frozenset(out)

    def _cap_zones_at(self, tick: _Tick, state: State) -> frozenset:
        """`_cap_zones` for `state`, once per tick and state (tick.committed).
        Read wherever an escape is judged - the decision, its repairs, and any
        check made outside a decision (a stop query, a re-check of what was
        emitted) - so an escape from a zone the aircraft is in is judged the
        same way everywhere. (A first version computed the set once, for the
        decision's state only: a re-check made outside the decision walked
        the cap's straight 20 m forecast and refused an escape the decision
        had judged as flown - tests/test_own_velocity_forecast.py's sample.)"""
        if self.recovery_law != "cap_all":
            return frozenset()
        key = ("cap_zones", state.x, state.y, state.up)
        hit = tick.committed.get(key)
        if hit is None:
            hit = tick.committed[key] = self._cap_zones(tick, state)
        return hit

    @staticmethod
    def _cap_ids(flags: dict) -> set:
        """Every zone whose escape flies at the cap on this decision: a window
        that opens or opened on the aircraft, and under "cap_all" every static
        zone it is in (`escape_at_cap`)."""
        return (set(flags.get("window_opens_on") or ()) | set(flags.get("window_opened_on") or ())
                | set(flags.get("escape_at_cap") or ()))

    def _opened_retries(self, flags: dict) -> list:
        """The settings _floor_the_escape re-runs the floors with when the
        escape from a zone that opened on the aircraft (`window_opens_on`,
        `window_opened_on` in `flags`) was refused at the cap: the cap less
        _OPENED_RETRY_MPS at a time (coarser for a high cap: at most
        _OPENED_RETRY_STEPS of them) down to the in-force floor min(2.0, cap),
        fastest first, then _IN_FORCE_FLOOR - the in-force floor's own shape,
        the outward part raised and the tangential kept, as for a zone the
        aircraft entered by itself. Empty without such a zone: nothing else
        is floored at the cap, so a slower try would change nothing.

        Fastest first, each re-checked in full, rather than a bisection: the
        re-check is monotone in the speed for a zone straight ahead, but
        nothing makes it so for every rule, and a bisection could step over
        the speed that passes. Nine re-runs at most, and only on a tick whose
        cap was refused. Under recovery_law "cap_all" a zone the aircraft is
        in for any reason is retried the same way (`escape_at_cap`)."""
        if not self._cap_ids(flags):
            return []
        cap = min((k.speed_max_mps for k in self._kins), default=4.0)
        floor = min(2.0, cap)
        step = max(_OPENED_RETRY_MPS, (cap - floor) / _OPENED_RETRY_STEPS)
        out: list = []
        s = cap - step
        while s > floor + 1e-9:
            out.append(round(s, 6))
            s -= step
        if floor < cap - 1e-9:
            out.append(floor)
        return out + [_IN_FORCE_FLOOR]

    def _escape_shortfall(self, state: State, raw: Action4D, flags: dict,
                          why: str) -> list[Violation]:
        """For each zone that opened on the aircraft (`window_opens_on`,
        `window_opened_on`) whose ring the aircraft is still in and which
        `raw` leaves slower than the in-force floor min(2.0, cap): the
        violation _floor_the_escape reports when no floored escape can be
        flown (`why`), so that the FSM escalates instead of the crawl passing
        as clean (review of b1d3aca, P1). The way out is read as
        _repair_geofence reads it: at the pose where the zone opens, for an
        opening still imposed. Empty when `raw` already leaves every such
        zone at the floor or faster: that is the floor, and it flies as
        before. Under recovery_law "cap_all" the same for a zone the aircraft
        is in for any reason (`escape_at_cap`)."""
        ids = self._cap_ids(flags)
        if not ids:
            return []
        windows = set(flags.get("window_opens_on") or ()) | set(flags.get("window_opened_on") or ())
        cap = min((k.speed_max_mps for k in self._kins), default=4.0)
        floor = min(2.0, cap)
        recs, _ = self._fence_anchors(self._ir)
        out = []
        for r in recs:
            f = r.rule
            if f.id not in ids or r.moving:
                continue
            st = self._rule_state(f, state)
            if not point_in_fence(st.x, st.y, st.up, f, r.polygon, r.buffered):
                continue
            t_open = self._opening_imposed(f, r.buffered, st)
            dt = t_open or 0.0
            ox, oy = escape_direction(st.x + raw.vx * dt, st.y + raw.vy * dt, r.polygon,
                                      raw.vx, raw.vy)
            leaving = raw.vx * ox + raw.vy * oy
            if leaving >= floor - 1e-9:
                continue
            what = ("window opened on the aircraft" if f.id in windows
                    else "inside the zone (recovery at the cap)")
            out.append(Violation(
                rule_id=f.id, category="geofence", predicted_at_s=dt,
                detail=(f"{what}: leaving at {leaving:.2f} m/s, "
                        f"under the {floor:.1f} m/s escape floor, and no floored escape "
                        f"can be flown ({why})")))
        return out

    def _slower_opened_escape(self, state: State, raw: Action4D, flags: dict,
                              enforced: list) -> "tuple[Action4D, Repair] | None":
        """The escape from the one zone that opened on the aircraft
        (`window_opens_on`, `window_opened_on`), at the fastest speed from
        the cap down to the in-force floor min(2.0, cap) that re-checks clean
        (`_opened_retries`' speeds), the tangential kept within the cap -
        for a tick where `raw` is escaping that zone and its every enforced
        violation is another zone's or a corridor's (`enforced`): the escape
        reaches that rule in the forecast. None in every other case, and
        when no such speed re-checks clean (the chain then runs as before,
        and its rescue and the FSM answer). The caller asks it with the
        monitor_only rules skipped, as the rescue is.

        WHY (review of b1d3aca, P1, the other half). The same opens-on cell
        with an always-on zone 18 m north of the aircraft (y 40-46) and the
        goto pilot's own 4 m/s north, which reaches that zone's ring in the
        5 s forecast: the chain's GeofenceSlide cancelled the whole
        northward part and pushed along the zone's edge at 4 m/s, east or
        west by the side the pilot's tiny lateral correction was on - and
        that side flipped every tick, so the aircraft stood at (60, 21) for
        1.7 s before it left (south), 20 ticks inside the polygon and 4.3 s
        to safe (43 / 6.8 s on 3660383, 3 / 2.6 s on 7c9c9e6, where every
        tick there escalated). Each of
        those ticks was legal - before the opening "leaving" is read where
        the zone opens, and an edge-parallel 4 m/s is out by then - so
        nothing escalated. The way out the aircraft was already flying, at
        3.5 m/s, broke nothing. Only geofence and corridor violations are
        answered here: a slower straight escape is what shortens their
        forecast, and the repair says exactly what changed (an action that
        also breaks a speed, yaw or climb cap, an altitude band, a stand-off
        or a clearance goes to the chain).

        Since the review of 69c27bd (P2) the escape is judged as flown, out
        of the ring and then a stop (_shaped_escape): the case above - the
        zone's ring 8 m past the exit - no longer violates at all (the 4
        m/s stops at 34.07, short of y = 39), and this applies only to a
        rule inside the escape's stop (tests/test_window_opens_on.py: the
        ring 2 m past the exit, the 4 m/s slowed to 3.0)."""
        ids = self._cap_ids(flags)
        if not ids or not enforced or any(
                v.rule_id in ids or v.category not in ("geofence", "corridor")
                for v in enforced):
            return None
        windows = set(flags.get("window_opens_on") or ()) | set(flags.get("window_opened_on") or ())
        recs, _ = self._fence_anchors(self._ir)
        inside = []
        for r in recs:
            if r.rule.id not in ids or r.moving:
                continue
            st = self._rule_state(r.rule, state)
            if point_in_fence(st.x, st.y, st.up, r.rule, r.polygon, r.buffered):
                inside.append((r, st))
        if len(inside) != 1:
            return None
        r, st = inside[0]
        dt = self._opening_imposed(r.rule, r.buffered, st) or 0.0
        ox, oy = escape_direction(st.x + raw.vx * dt, st.y + raw.vy * dt, r.polygon,
                                  raw.vx, raw.vy)
        out_now = raw.vx * ox + raw.vy * oy
        if out_now <= 0.1:
            return None                     # not escaping that zone (the check agrees)
        tx, ty = raw.vx - out_now * ox, raw.vy - out_now * oy
        t_mag = math.hypot(tx, ty)
        cap = min((k.speed_max_mps for k in self._kins), default=4.0)
        broke = ", ".join(sorted({v.rule_id for v in enforced}))
        depth = r.buffered.boundary.distance(Point(st.x, st.y))
        for s in [cap] + [x for x in self._opened_retries(flags) if x != _IN_FORCE_FLOOR]:
            room = math.sqrt(max(cap * cap - s * s, 0.0))
            k = min(1.0, room / t_mag) if t_mag > 1e-9 else 0.0
            cand = Action4D(vx=tx * k + ox * s, vy=ty * k + oy * s,
                            vz_up=raw.vz_up, yaw_rate=raw.yaw_rate)
            if self._check(state, cand):
                continue
            what = ("window opened on the aircraft" if r.rule.id in windows
                    else "inside the zone (recovery at the cap)")
            return cand, Repair(
                operator="GeofenceEscape",
                detail=(f"{r.rule.id}: {what}, leaving at "
                        f"{out_now:.2f} m/s -> out at {s:.2f} m/s, the fastest from the "
                        f"{cap:.1f} m/s cap that does not break {broke}"),
                magnitude_m=depth, axis="lateral", recovery=True)
        return None

    def _decide(self, state: State, raw: Action4D) -> ShieldDecision:
        tick = self._tick_ctx()
        if tick is None:                        # called outside filter()
            with self._scope(self._new_tick(None)):
                return self._decide(state, raw)
        rt = tick.rt
        flags = {"path_repair": self.path_repair, "segment_check": self.segment_check,
                 # Zones whose window opens on the aircraft sooner than it can
                 # leave (_opening_imposed): enforced before they are in force
                 # even on a tick whose action, already escaping, breaks
                 # nothing. Empty on almost every tick.
                 "window_opens_on": self.openings_imposed(state),
                 # Zones remembered from an earlier tick as having opened on
                 # the aircraft, not yet left (`_opened_on`): their escape
                 # flies at the cap after the opening too.
                 "window_opened_on": sorted(self._opened_on)}
        # Under recovery_law "cap_all": every static zone the aircraft is in,
        # escaped at the cap through the same path (`_cap_zones_at`). The key
        # is written only when there is one, so every other trace is as it was.
        cap_now = self._cap_zones_at(tick, state)
        if cap_now:
            flags["escape_at_cap"] = sorted(cap_now)
        mode = self.repair_mode
        # Finiteness first: every check below is a comparison, and NaN loses them
        # all, so an unsanitised action would be declared legal and passed
        # straight through. See _sanitise().
        raw, nonfinite = _sanitise(raw)
        violations = self._check(state, raw)
        if nonfinite:
            violations = violations + [Violation(
                rule_id="action-finite", category="contract",
                detail=f"non-finite channel(s) {','.join(nonfinite)} zeroed",
                predicted_at_s=0.0)]
        subject_dropped = None
        if self._subject_rejected is not None:
            # set_subject() dropped a non-finite subject, so the stand-off
            # rules that would bind it judge nothing this tick. Said here, so
            # the tick is not a silent pass (see set_subject). Like
            # action-finite it is no policy rule (fsm.rules_from_policy reads
            # it as a hard P0 repair) and it is answered by Sanitise, the
            # finiteness contract's operator: the FSM refuses a violation
            # with no repair beside it, and a new operator name would be one
            # theta has no magnitude for.
            #
            # Only for the HARD, enforced stand-offs that bind it. The FSM's
            # reading (hard, project_fix) is then never a stronger response
            # than the rule's own: project_fix is the weakest enforced
            # action, and P0 against the rule's priority only orders the
            # citation. For a soft or monitor_only stand-off it would be:
            # until a review of 2026-10-19 a NaN subject beside a
            # monitor_only stand-off took the FSM Normal -> Brake (G1), and a
            # flickering one on to LOITER and RTL by N-in-T, where a real
            # breach of that rule leaves it in Normal. Those are only counted
            # (n_subject_nonfinite) and read unbound by rule_status.
            sx, sy, cls = self._subject_rejected
            ids = [so.id for so in self._standoffs if so.binds(cls)
                   and rt.hard.get(so.id, True) and self._act(rt, so.id) != "monitor_only"]
            if ids:
                subject_dropped = f"subject ({sx}, {sy}) -> dropped"
                violations = violations + [Violation(
                    rule_id="subject-finite", category="contract",
                    detail=(f"non-finite subject ({sx}, {sy}) dropped; stand-off "
                            f"{', '.join(ids)} cannot be judged"),
                    predicted_at_s=0.0)]

        if not violations:
            # Nothing was wrong with it, so the re-check is empty by
            # construction and does not need running again - unless the
            # POSITION is illegal, where a legal action can still be a crawl.
            if self.repair_mode == "chain" and self._stop_illegal(tick, state):
                return self._floor_the_escape(tick, state, raw, mode, flags)
            return self._traced(ShieldDecision(raw=raw, emitted=raw),   # passthrough
                                state, tick, [], [], [], mode, flags)
        edge = self._window_edge(tick, violations)

        enforced = [v for v in violations if self._act(rt, v.rule_id) != "monitor_only"]
        if not enforced:
            # monitor_only rules only: recorded, and the command flies as given.
            # Its re-check is the check above, so a P0 monitor_only rule shows
            # up in the escape count, which is the honest place for it. From
            # an illegal position the escape floor still applies, as with
            # nothing violated: a monitor_only rule is not enforced, so it
            # must not decide how fast the aircraft leaves a hard breach.
            if self.repair_mode == "chain" and self._stop_illegal(tick, state):
                return self._floor_the_escape(tick, state, raw, mode, flags,
                                              violations, edge)
            return self._traced(ShieldDecision(raw=raw, emitted=raw, violations=violations,
                                               emitted_violations=list(violations)),
                                state, tick, [], [], edge, mode, flags)
        monitor = frozenset(v.rule_id for v in violations
                            if self._act(rt, v.rule_id) == "monitor_only")

        def last_resort(op: str, result: str, detail: str) -> dict:
            return {"operator": op, "result": result, "reason": detail,
                    "iterations": None, "repairs": [op]}

        head: list[Repair] = []
        first: list[dict] = []
        final: list[dict] = []
        if nonfinite:
            head.append(Repair(operator="Sanitise",
                               detail=f"{','.join(nonfinite)} -> 0.0"))
            first.append({**last_resort("Sanitise", "ok", head[-1].detail), "iterations": 0})
        if subject_dropped is not None:
            head.append(Repair(operator="Sanitise", detail=subject_dropped))
            first.append({**last_resort("Sanitise", "ok", head[-1].detail), "iterations": 0})

        # A rule whose action is brake / loiter / RTL / land skips projection
        # and stops - where a stop is legal. Where it is not (inside a zone,
        # under the floor), stopping is the deadlock, so the repair stack still
        # recovers and the FSM makes the mode change.
        strongest = max((self._act(rt, v.rule_id) for v in enforced),
                        key=_fsm.action_strength)
        if strongest in _STOP_ACTIONS and not self._stop_illegal(tick, state):
            gov = min((v.rule_id for v in enforced
                       if self._act(rt, v.rule_id) == strongest),
                      key=lambda r: (not rt.hard.get(r, True),
                                     next((c.priority for c in rt.constraints
                                           if c.id == r), "P0"), r))
            reps = head + [Repair(operator="Brake",
                                  detail=f"{gov}: violation_action {strongest} -> "
                                         f"projection skipped, stop")]
            return self._traced(
                ShieldDecision(raw=raw, emitted=BRAKE, violations=violations,
                               repairs=_dedupe(reps), braked=True,
                               emitted_violations=self._check(state, BRAKE)),
                state, tick, [], [last_resort("Brake", "fallback", reps[-1].detail)],
                edge, mode, flags, first)

        # Escaping a zone that opened on the aircraft, at a speed whose
        # forecast reaches ANOTHER zone: the escape slowed until it re-checks
        # clean, before the chain's slide would turn it (_slower_opened_escape
        # says why). In chain mode only, and only for a tick whose every
        # enforced violation is such a zone's.
        if self.repair_mode == "chain" and not head:
            prev, tick.skip = tick.skip, monitor
            try:
                slower = self._slower_opened_escape(state, raw, flags, enforced)
            finally:
                tick.skip = prev
            if slower is not None:
                cand, rep = slower
                reps = [rep]
                return self._traced(
                    ShieldDecision(raw=raw, emitted=cand, violations=violations,
                                   repairs=reps, emitted_violations=self._check(state, cand)),
                    state, tick, [], [last_resort("GeofenceEscape", "ok", rep.detail)],
                    edge, mode, flags, first)

        fixed, repairs, blocking, passes = self._repair_chain(
            tick, state, raw, monitor, list(head), violations)
        relaxed: list[str] = []
        if blocking:
            r = self._relax(tick, state, raw, enforced + blocking, monitor, head, violations)
            if r is not None:
                fixed, repairs, relaxed, passes = r
                blocking = []
                if relaxed:
                    repairs.append(Repair(
                        operator="SoftRelax",
                        detail=(f"soft rule(s) {', '.join(relaxed)} given up so the "
                                f"rest can be satisfied (lowest priority first)")))
                    final.append(last_resort("SoftRelax", "relaxed", repairs[-1].detail))

        if blocking:
            # BRAKE is a fail-safe only where standing still is legal. Inside
            # the clearance ring a zero action leaves d_next == d_now, so the
            # same violation is raised next tick and the vehicle is frozen into
            # the violation instead of recovering from it. Look for a heading
            # that re-checks CLEAN before considering a stop.
            stop_illegal = self._stop_illegal(tick, state)
            clean = best = None
            prev, tick.skip = tick.skip, monitor
            try:
                if any(v.category == "clearance" for v in blocking) or stop_illegal:
                    cap = min((k.speed_max_mps for k in self._kins), default=4.0)
                    clean, best = self._rescue(state, raw, fixed, cap)
            finally:
                tick.skip = prev
            if clean is not None:
                repairs.append(Repair(
                    operator="ClearanceEscape",
                    detail="repair not converged -> recovery heading"))
                fixed = clean
            elif not stop_illegal:
                repairs.append(Repair(operator="Brake", detail="repair not converged -> stop"))
                final.append(last_resort("Brake", "fallback", repairs[-1].detail))
                return self._traced(
                    ShieldDecision(raw=raw, emitted=BRAKE, violations=violations,
                                   repairs=_dedupe(repairs), braked=True,
                                   emitted_violations=self._check(state, BRAKE)),
                    state, tick, passes, final, edge, mode, flags, first)
            elif best is not None:
                repairs.append(Repair(
                    operator="ClearanceEscape",
                    detail="stopping is itself illegal here -> best-effort recovery"))
                fixed = best
            else:
                repairs.append(Repair(operator="Brake", detail="repair not converged -> stop"))
                final.append(last_resort("Brake", "fallback", repairs[-1].detail))
                return self._traced(
                    ShieldDecision(raw=raw, emitted=BRAKE, violations=violations,
                                   repairs=_dedupe(repairs), braked=True,
                                   emitted_violations=self._check(state, BRAKE)),
                    state, tick, passes, final, edge, mode, flags, first)
            final.append(last_resort("ClearanceEscape", "fallback", repairs[-1].detail))

        # One more _check, on the action that is actually leaving the building.
        # In the common case `fixed` already re-checked clean inside the loop
        # above and this is a repeat; in the `best is not None` branch it is the
        # only check that has ever been run against what gets flown.
        return self._traced(
            ShieldDecision(raw=raw, emitted=fixed, violations=violations,
                           repairs=_dedupe(repairs), relaxed=relaxed,
                           emitted_violations=self._check(state, fixed)),
            state, tick, passes, final, edge, mode, flags, first)

    def _applied_caps(self, tick: _Tick, d: ShieldDecision) -> "_fsm.KinematicCaps | None":
        """The speed and climb caps the repair chain clamped against on this
        tick: the kinematic rules in force here (`tick.rules`, which leaves out
        a rule out of its window or switched off), not `monitor_only`, and not
        a soft one the chain gave up (`tick.given_up`, set by _relax). Theta's
        |dv| measure (when a rail selects it) starts from the raw action
        clamped to these; reading the caps off the whole policy instead made
        a raw action over a cap the chain never applied a "decision made under
        other caps", which the FSM refuses (a fault, LOITER, nothing
        streamed). (A first version left out EVERY soft cap once any soft rule
        was given up, so a soft P1 speed cap the chain still clamped to was
        missing while a soft P2 band was relaxed: the FSM then measured the
        clamp as position correction, or refused the tick.)"""
        rt = tick.rt
        return _fsm.KinematicCaps.of(
            k for k in tick.rules("kins")
            if self._act(rt, k.id) != "monitor_only" and k.id not in tick.given_up)

    def _position_verdict(self, tick: _Tick, state: State,
                          raw: "Action4D | None" = None) -> "tuple[str, str] | None":
        """The escalation FSM's X8 input (2026-11-06): (reason, rule id) when
        the position the Shield is handed can no longer certify a hard P0
        zone's margin, else None.

        WHAT A RAIL REPORTS (State.fix_ok, State.pos_sigma_m,
        State.estimator_ok). All None - every rail but the ROS 2 node - and
        this returns None on every tick: the Shield behaves exactly as before
        the fields existed.
          * estimator_ok (2026-11-12), when reported, decides first: it is the
            AUTOPILOT'S OWN VERDICT on the estimate it hands the Shield.
            True: the position is certified - it is the estimate the
            autopilot navigates, returns and fences on, valid by the
            autopilot's own test, also while its GPS is denied (fix_ok False:
            EKF3 dead-reckons on its IMU). No dead reckoning of this Shield's
            is held against it: that would measure the Shield's command model
            against the autopilot's estimator, not the estimate's error (on
            the hil rig the gap was 3.0-3.5 m where the estimate was 0.13-
            0.78 m off the aircraft). False: the autopilot reports no valid
            absolute horizontal position for as long as its own EKF failsafe
            waits; the error is unbounded and no margin certifies anything,
            so every hard, enforced P0 zone in force is beyond it (the
            nearest is named).
          * fix_ok False (with estimator_ok None): the position is not a
            current valid fix (a stale pose, no 3D fix). The aircraft
            is dead-reckoned from the last certified position on what this
            Shield itself streamed since (`_note_streamed`: its command; a
            stop or an in-place hold streams 0, a moved hold in transit its
            plan's speed). The position handed in is wrong by err = |estimate
            - handed| + u, where u = dr_drift_mps x the time since the last
            certified position is the drift the dead reckoning cannot see
            (DR_DRIFT_MPS), and the boundary is taken to be u nearer. This is
            the law of a rail that reports no estimator health - the headless
            harness's frozen fix, a bridge forwarding a stale pose - and of
            the ROS 2 node once the autopilot's report goes stale.
          * pos_sigma_m: the rail's 1-sigma horizontal accuracy. Only when a
            rail reports it (None is "not reported", never "perfect"): err is
            at least 3 sigma, judged at the position and along the request's
            own forecast (the rubric reads noise-E3 against the planned
            route; the Shield's view of the plan is the request).

        THE TEST is the rubric's E3 (docs/RUBRIC-failsafe.md), on the
        Shield's own estimate: a hard P0 zone's margin_m is sized for a
        position-error budget, so once err exceeds it while the zone's
        boundary lies within err of the estimate, the margin no longer
        certifies the rule and no repair computed from the handed position
        can be trusted (p1, "repair is unsafe"). The FSM's X8 then returns
        (RTL, on the autopilot's own estimate); G6 lands it if the return
        fails. Until the error reaches a boundary nothing changes, so a fix
        that comes back in time, or one frozen on a held aircraft (whose
        estimate does not move), costs nothing. With dr_drift_mps 0 the test
        fires on the tick E3 first holds on the ideal vehicle; a larger
        allowance only makes it earlier.

        ONCE A RAIL REPORTS ITS ACCURACY, A MARGIN IS AN ACCURACY
        REQUIREMENT: a 1 m margin certifies a P0 zone only while 3 sigma <=
        1 m (sigma <= 0.33 m, RTK class). With ordinary GNSS accuracy the
        request's forecast passing within 3 sigma of such a zone returns the
        aircraft. Sizing margins to the rail's real (EKF-fused) accuracy is a
        policy decision; no policy was changed with this.

        TRUSTING THE AUTOPILOT'S VERDICT IS NOT A BOUND ON ITS ERROR. While
        estimator_ok is True the estimate's own error is the autopilot's,
        and no Shield action flies on a better one (RTL returns on the same
        estimate). On the hil rig (ArduCopter 4.5.7 SITL, 30 GPS denials)
        EKF3 kept its absolute position for 6.8-6.9 s of each 7 s denial and
        was up to 1.47 m off the aircraft meanwhile - more than a 1 m margin
        - so a hard P0 margin certifies a zone through a denial only if it
        covers the autopilot's dead-reckoning error over its own allowance:
        a margin-sizing question for the policy, as with pos_sigma_m."""
        t = tick.t
        prev, self._pk_cmd = self._pk_cmd, None      # _note_streamed sets it again
        sigma = getattr(state, "pos_sigma_m", None)
        noise = 3.0 * float(sigma) if sigma is not None and sigma > 0 else 0.0
        est = getattr(state, "estimator_ok", None)
        if est is True or (est is None and getattr(state, "fix_ok", None) is not False):
            # Certified: a valid fix (or none reported), or the autopilot
            # vouching for its own estimate. It anchors the dead reckoning,
            # should the certification lapse.
            self._pk_good = (state.x, state.y, t)
            self._pk = None
            if not noise:
                return None
            ex, ey, err, since, drift, plan = state.x, state.y, noise, None, 0.0, raw
        else:
            if self._pk is None:
                gx, gy, gt = (self._pk_good if self._pk_good is not None
                              else (state.x, state.y, t))
                self._pk = {"anchor": (gx, gy), "dx": 0.0, "dy": 0.0, "since": gt}
            pk = self._pk
            if prev is not None and t > prev[0]:
                pk["dx"] += prev[1] * (t - prev[0])
                pk["dy"] += prev[2] * (t - prev[0])
            ex, ey = pk["anchor"][0] + pk["dx"], pk["anchor"][1] + pk["dy"]
            since = pk["since"]
            if est is False:
                return self._estimator_lost(tick, state, t - since)
            drift = self.dr_drift_mps * max(0.0, t - since)
            err = max(math.hypot(ex - state.x, ey - state.y) + drift, noise)
            plan = None
        rt = tick.rt
        p = Point(ex, ey)
        poses = None
        for r in self._zones_now_in(tick, state.up):
            f = r.rule
            if not rt.hard.get(f.id, True) or self._act(rt, f.id) == "monitor_only":
                continue
            if getattr(f, "priority", "P0") != "P0":
                continue
            margin = float(getattr(f, "margin_m", 0.0) or 0.0)
            if not err > margin:
                continue
            dist = max(0.0, r.polygon.distance(p) - drift)      # 0 inside
            if plan is not None and dist > err:
                if poses is None:
                    poses = [Point(q.x, q.y)
                             for _, q in self._predict(state, _sanitise(plan)[0])]
                dist = min([dist] + [r.polygon.distance(q) for q in poses])
            if dist <= err:
                what = (f"no valid fix for {t - since:.1f} s: the position handed in may be "
                        f"{err:.1f} m off (dead reckoning on the streamed commands, drift "
                        f"allowance {self.dr_drift_mps:g} m/s)" if since is not None
                        else f"reported accuracy: 3 sigma = {err:.1f} m")
                return (f"{what}, beyond {f.id}'s {margin:g} m margin, with its "
                        f"boundary within {dist:.1f} m of the estimate - the margin no "
                        f"longer certifies it", f.id)
        return None

    def _estimator_lost(self, tick: _Tick, state: State,
                        age_s: float) -> "tuple[str, str] | None":
        """X8's input when the AUTOPILOT reports its own horizontal position
        invalid (State.estimator_ok False, 2026-11-12; _position_verdict):
        the error is unbounded, so the nearest hard, enforced P0 zone in
        force at this altitude is named; None when there is none (nothing to
        certify). `age_s` is the time since the last certified position."""
        rt = tick.rt
        p = Point(state.x, state.y)
        best = None
        for r in self._zones_now_in(tick, state.up):
            f = r.rule
            if not rt.hard.get(f.id, True) or self._act(rt, f.id) == "monitor_only":
                continue
            if getattr(f, "priority", "P0") != "P0":
                continue
            dist = r.polygon.distance(p)
            if best is None or dist < best[0]:
                best = (dist, f)
        if best is None:
            return None
        dist, f = best
        return (f"the autopilot reports no valid absolute horizontal position (its own "
                f"estimator, {age_s:.1f} s after the last certified position): the "
                f"position handed in certifies no margin - {f.id}'s boundary is "
                f"{dist:.1f} m from it", f.id)

    def _noise_marginal(self, tick: _Tick, state: State, d: ShieldDecision) -> bool:
        """The FSM's onset hysteresis input (TickInput.noise_marginal,
        2026-11-06): with a REPORTED position accuracy (State.pos_sigma_m;
        None, every rail today, is always False), is this tick's request
        clean of every hard zone by less than 3 sigma - a verdict the
        position's own noise could flip? The FSM then does not count the next
        violation of the ring as a new N-in-T onset: N-in-T is for a pilot
        that keeps re-violating (rubric N5), and a ring re-touched inside the
        position's own uncertainty is the same violation seen through noise
        (rubric N2). Measured: 0.3 m of noise along the margin ring of
        gps-noise-within-margin (nightly seeds 1001, 1003) made three onsets
        in 5 s out of repairs of 0.02-0.43 m and sent the aircraft home by
        G2 and G4."""
        sigma = getattr(state, "pos_sigma_m", None)
        if sigma is None or not sigma > 0:
            return False
        rt = tick.rt
        if any(rt.hard.get(v.rule_id, True) and self._act(rt, v.rule_id) != "monitor_only"
               for v in d.violations):
            return False                    # violated: not a clean tick
        zones = [r for r in self._zones_now_in(tick, state.up)
                 if rt.hard.get(r.rule.id, True)
                 and self._act(rt, r.rule.id) != "monitor_only"]
        if not zones:
            return False
        ruler = ZoneRuler(zones)
        raw, _ = _sanitise(d.raw)
        band = 3.0 * float(sigma)
        return any(ruler.distance(p.x, p.y) <= band for _, p in self._predict(state, raw))

    def _note_streamed(self, tick: _Tick, d: ShieldDecision, state: State) -> None:
        """What this tick hands the aircraft, for the dead reckoning of
        _position_verdict (2026-11-06): the command streamed, or with the
        autopilot holding, 0 (in place) or the transit to a moved hold point
        at its plan's speed. Read only while State.fix_ok is False."""
        cmd = getattr(d, "command", None)
        vx = vy = 0.0
        if cmd is not None:
            vx, vy = float(cmd.vx), float(cmd.vy)
        elif self.fsm is not None and self.fsm.state is _fsm.FSMState.LOITER:
            h = self.fsm.hold
            if (h is not None and h.kind == "moved" and h.speed_mps
                    and h.t_s is not None and tick.t < h.t_s + h.travel_s):
                pk = self._pk
                ex = (pk["anchor"][0] + pk["dx"]) if pk else state.x
                ey = (pk["anchor"][1] + pk["dy"]) if pk else state.y
                dx, dy = h.x - ex, h.y - ey
                n = math.hypot(dx, dy)
                if n > 1e-9:
                    vx, vy = dx / n * h.speed_mps, dy / n * h.speed_mps
        if math.isfinite(vx) and math.isfinite(vy):
            self._pk_cmd = (tick.t, vx, vy)

    def _escalate(self, tick: _Tick, state: State, d: ShieldDecision, *,
                  rtl_failed: bool = False, home_reached: bool = False,
                  landed: bool = False, hold_off_m: float | None = None,
                  hold_reach_m: float | None = None) -> ShieldDecision:
        """Feed this tick to the escalation FSM and attach its verdict.

        Theta is applied by the FSM, to the repairs' own magnitude_m
        (theta_horizon_s None, the default), or to the setpoint change |dv| x
        theta_horizon_s when a number is given (the one-period reading). `stop_illegal` is the full
        BRAKE check, which the FSM filters to hard enforced rules, and
        `unsafe_depth_m` how far from legal that position is (X6's progress
        test). A contract
        break (the FSM refuses the input) never stops the control loop: the
        decision carries `fsm_fault`, the first one requests LOITER, and
        nothing is streamed after it until reset_episode()."""
        upd: dict[str, Any] = {"generation": tick.rt.ir.generation,
                               "policy_hash": tick.rt.ir.policy_hash}
        if self.fsm is None:
            upd["command"] = d.emitted
            # A rail that runs THE escalation FSM itself (the ROS 2 node,
            # guardrail/replay.py rail_shield) still needs X8's input, and it
            # is this Shield's to judge: its dead reckoning on what the rail
            # streamed (note_flown) and its zones (2026-11-07). Only for a
            # state that reports its fix quality, so every caller whose
            # states report none - every other rail, and the harness's
            # oracle on the true state - runs exactly as before.
            if (getattr(state, "fix_ok", None) is not None
                    or getattr(state, "pos_sigma_m", None) is not None
                    or getattr(state, "estimator_ok", None) is not None):
                upd["position_lost"] = self._position_verdict(tick, state, d.raw)
            return d.model_copy(update=upd)
        if self._fsm_fault is not None:
            upd.update(fsm_fault=self._fsm_fault, setpoint="none", command=None,
                       fsm_state_before=self.fsm.state.value,
                       fsm_state_after=self.fsm.state.value)
            return d.model_copy(update=upd)
        try:
            self._stop_illegal(tick, state)          # fills tick.unsafe
            unsafe = list(tick.unsafe)
            if self._velocity_of(state) is not None:
                # The FSM's stop_illegal is about the POSITION (fsm.py, "stop_
                # illegal means standing still here breaks a HARD rule"), as
                # on every rail; with a measured velocity tick.unsafe is about
                # where a stop would END (_committed), so it is asked again
                # without the velocity. Its inputs keep their meaning.
                unsafe = self._check(self._positional(state), BRAKE)
            caps = self._applied_caps(tick, d)
            lost = self._position_verdict(tick, state, d.raw)
            # Loiter's hand-back guard (FSMConfig.hold_horizon "loiter",
            # 2026-11-07): only while the position itself is legal - an
            # illegal one is not handed back anyway. Asked through the hold
            # planner since 2026-11-10: a tick pays for the guard or for a
            # search slice, never both, and while a plan is being searched
            # the hand-back is refused unsampled (HoldPlanner.handback).
            # With it, in turn (FSMConfig.handback_request "at_speed"): is the
            # request's repair one to trust from the hold point at the speed
            # it asks for (handback_untrusted).
            planner = self._holds()
            handback, hb_untrusted = (
                planner.handback(self, state, d.emitted, fsm=self.fsm, raw=d.raw)
                if (self.fsm.state is _fsm.FSMState.LOITER and not unsafe
                    and self.fsm.config.hold_horizon == "loiter"
                    and not self.fsm.terminal) else (None, None))
            refused_for_search = planner.guard == "search"
            inp = _fsm.tick_input_from_decision(
                tick.t, d, tick.rt, horizon_s=self.theta_horizon_s,
                stop_illegal=unsafe, rtl_failed=bool(rtl_failed),
                home_reached=bool(home_reached), landed=bool(landed),
                caps=caps if caps is not None else False,
                unsafe_depth_m=self._unsafe_depth_in(state, unsafe),
                hold_legal=self._hold_legal(tick),
                position_lost=lost[0] if lost else None,
                noise_marginal=self._noise_marginal(tick, state, d),
                handback_unsafe=handback, handback_untrusted=hb_untrusted,
                hold_off_m=hold_off_m, hold_reach_m=hold_reach_m)
            out = self.fsm.step(inp)
            rec = out.record
            if refused_for_search:
                # Why this tick's handback_unsafe: nothing was sampled.
                rec = {**rec, "handback_refused": "hold search running"}
            # Loiter or Land was just entered, or the point held is no longer
            # legal (needs_hold): choose where the autopilot holds / lands
            # from this Shield's own rules (guardrail/holdpoint.py). The FSM
            # refuses a plan that contradicts its own input (hold_at), and a
            # refusal is a contract break like any other (the fault below).
            # Planned against `unsafe`, the stop check the FSM was just given
            # (about the POSITION), never tick.unsafe, which with a measured
            # velocity is where a stop would END. Since 2026-11-07 the search
            # gets holdpoint.TICK_QUERIES of its budget a tick, in this tick's
            # world, and the plan is taken on the tick it ends (holdpoint.py,
            # THE PER-TICK BOUND; HoldPlanner): the hil Orin spent 146.9 ms on
            # one planning tick, over WP3-13's 100 ms.
            got = planner.tick(
                self, self.fsm, lambda: self._plan_hold(tick, state, d, unsafe),
                here=self._positional(state), unsafe=self.hard_violations(unsafe or []),
                world=tick)
            if got is not None:
                plan = self.fsm.hold_at(got)
                rec = {**rec, "hold_point": plan.to_dict(), "hold_needed": False,
                       "hold_chosen": True}
                if planner.last is not None and planner.last["ticks"] > 1:
                    rec["hold_search"] = dict(planner.last)   # spread over ticks
            elif planner.search is not None:
                rec = {**rec, "hold_search": {"ticks": planner.search.ticks,
                                              "units": round(planner.search.spent, 3),
                                              "running": True}}
        except ValueError as e:
            self._fsm_fault = str(e)
            upd.update(fsm_fault=self._fsm_fault, set_mode="LOITER", setpoint="none",
                       command=None, fsm_state_before=self.fsm.state.value,
                       fsm_state_after=self.fsm.state.value)
            return d.model_copy(update=upd)
        cmd = (d.emitted if out.setpoint == "pass"
               else BRAKE if out.setpoint == "brake" else None)
        hold = self.fsm.hold if "hold_point" in rec else None
        upd.update(fsm_state_before=out.before.value, fsm_state_after=out.state.value,
                   fsm_edge=out.edge, set_mode=out.set_mode, setpoint=out.setpoint,
                   command=cmd, fsm_record=rec,
                   hold_point=hold.to_dict() if hold is not None else None)
        return d.model_copy(update=upd)

    def handback_unsafe(self, state: State, a: Action4D) -> bool:
        """Would flying `a` (the Shield's action for this request) from the
        position carry the aircraft, within the lookahead, onto a spot where
        a stop is illegal? (fsm.TickInput.handback_unsafe, since 2026-11-07.)
        The straight flight of the command is sampled every HANDBACK_STEP_S
        through the lookahead (fewer, evenly spread, where that many stop
        queries would cost more than holdpoint.HANDBACK_QUERIES units: the
        per-tick bound), and a stop at each sample, from the time the
        aircraft gets there, is asked of this Shield's own rules (a look-ahead
        view, as the hold-point search asks). Read only in Loiter, with the
        position itself legal, under FSMConfig.hold_horizon "loiter": the
        hold is then legal for as long as it is held, and handing GUIDED back
        to a request that flies into a moving zone's path - where the hold
        was keeping the aircraft out - is the breach the harness counts as a
        move of the aircraft's own (dyn-nfz-rotate-spin at 9 deg/s on the
        ideal vehicle, handed back at 8.4 s)."""
        from . import holdpoint as _hp      # local: holdpoint imports this module
        st = self._positional(state)
        if math.hypot(a.vx, a.vy) <= 1e-9 and abs(a.vz_up) <= 1e-9:
            return False
        # At most HANDBACK_QUERIES units a tick (holdpoint.py, THE PER-TICK
        # BOUND, since 2026-11-07): as many samples as fit at this world's
        # dearest stop query (holdpoint.stop_cost_bound), spread evenly
        # through the lookahead when fewer than one every HANDBACK_STEP_S fit.
        full = max(1, int(round(self.lookahead_s / HANDBACK_STEP_S)))
        n = max(1, min(full, int(_hp.HANDBACK_QUERIES // self._stop_cost_bound())))
        step = HANDBACK_STEP_S if n == full else self.lookahead_s / n
        for i in range(1, n + 1):
            tau = i * step
            p = State(x=st.x + a.vx * tau, y=st.y + a.vy * tau, up=st.up + a.vz_up * tau,
                      yaw_deg=st.yaw_deg)
            with self.looking_ahead(tau):
                if self.hard_violations(self.state_is_unsafe(p)):
                    return True
        return False

    def world_is_still(self) -> bool:
        """Does a decision here depend on nothing but the state and the
        action: no rule with a time window, no moving zone, no tracked
        subject? (holdpoint.HoldPlanner keeps the request check's answer on
        it; a hot reload or a spawned zone replaces the rule set, which the
        planner sees for itself.)"""
        rt = self._rt
        return not (rt.timed or rt.moving_ids) and self._subject is None

    def handback_untrusted(self, raw: Action4D, at: State, fsm=None,
                           horizon_s: Any = "shield") -> bool:
        """Would the request `raw`, judged from `at` - the point a Loiter
        holds - at the speed it asks for, need a repair of a hard rule that
        may not be trusted: over theta, or not converged?
        (fsm.TickInput.handback_untrusted, FSMConfig.handback_request
        "at_speed", since 2026-11-10.) The decision this Shield WOULD make
        for it there (`advise`: nothing moves) from a state with no measured
        velocity - the ideal-response forecast, the request flown as asked -
        read by the FSM's own G3 test (EscalationFSM.untrusted_repair) at
        this Shield's theta basis. `fsm`: the FSM whose thresholds judge it
        (a rail's own; default this Shield's), and `horizon_s` the theta
        basis that rail gives its FSM (tick_input_from_decision's; default
        this Shield's own): the request is judged exactly as the rail's G3
        would judge it.

        The tick's own decision judges the request from the held aircraft:
        at rest, where a measured-velocity forecast reaches less far, and
        wherever the hold's settling has it. On the hil smoke sm1110_010
        (corridor-drift-angles 8 m/s, -60 deg) the Loiter's point sat where
        CorridorReturn's repair is 2.0 m, the hold crept back to it from
        0.3 m inside, the repair there read 1.72-1.78 m for T_recover, and
        Loiter handed the unchanged request back 14 times."""
        fsm = fsm if fsm is not None else self.fsm
        if fsm is None:
            return False
        p = State(x=at.x, y=at.y, up=at.up, yaw_deg=at.yaw_deg)
        d = self.advise(p, raw)
        tick = self._tick_ctx()
        rt = tick.rt if tick is not None else self._rt
        basis = self.theta_horizon_s if isinstance(horizon_s, str) else horizon_s
        inp = _fsm.tick_input_from_decision(
            float(self._t_last or 0.0), d, rt, horizon_s=basis,
            stop_illegal=False, caps=self.applied_caps or False)
        return fsm.untrusted_repair(inp)

    def _stop_cost_bound(self) -> float:
        """holdpoint.stop_cost_bound for the rule set in force (cached per
        runtime: it reads rule types and motion only)."""
        from . import holdpoint as _hp      # local: holdpoint imports this module
        tick = self._tick_ctx()
        rt = tick.rt if tick is not None else self._rt
        got = getattr(self, "_stop_cost_cache", None)
        if got is None or got[0] is not rt:
            got = self._stop_cost_cache = (rt, _hp.stop_cost_bound(self))
        return got[1]

    def _holds(self):
        """The escalation FSM's hold planner (holdpoint.HoldPlanner, THE
        PER-TICK BOUND), made on first use."""
        planner = getattr(self, "_hold_planner", None)
        if planner is None:
            from . import holdpoint as _hp  # local: holdpoint imports this module
            planner = self._hold_planner = _hp.HoldPlanner()
        return planner

    def _hold_legal(self, tick: _Tick) -> bool | None:
        """In Loiter: is the FSM's hold POINT still legal - a stop there from
        the plan's arrival time (this tick, once it is due) breaks no hard
        enforced rule through the lookahead? (fsm.TickInput.hold_legal.)
        None outside Loiter or before a point is held. One stop query a
        Loiter tick. The point is judged, not the aircraft: an aircraft
        pushed off a legal hold is X6's and X7's (fsm.py)."""
        from . import holdpoint as _hp      # local: holdpoint imports this module
        h = self.fsm.hold if self.fsm is not None else None
        if h is None or self.fsm.state is not _fsm.FSMState.LOITER:
            return None
        # The flight rails judge their own FSM's point with the same function.
        return _hp.hold_point_legal(self, h, tick.t)

    def _plan_hold(self, tick: _Tick, state: State, d: ShieldDecision,
                   stop_illegal: list | None = None) -> "_fsm.HoldPoint":
        """The FSM's hold point for this tick (it asked: `needs_hold`).

        `unsafe` is the stop check the FSM was given on this tick, its
        stop_illegal (`stop_illegal`; None reads tick.unsafe, the same check
        on a tick with no measured velocity), filtered to hard enforced
        rules. It is about the POSITION: with a measured velocity, _escalate
        asks it again without the velocity, because tick.unsafe then says
        where a stop would END (_committed). Until the merge with the
        own-velocity forecast (2026-10-21) the plan read tick.unsafe, and with
        a velocity it relocated holds the FSM had just called legal - 24 of
        337 seeded Loiter entries on seven policies, the reason saying that
        "standing still" at the position broke a rule it does not break
        (tests/test_hold_points.py test_with_a_measured_velocity_the_hold_
        agrees_with_the_fsms_own_stop_check). The search gets the state
        without its velocity: it judges points (a stop at each, a straight
        transit between them), and the stop's own overshoot from the
        aircraft's momentum is the rail's hold that stops clear
        (guardrail/response.py hold_plan), not the point's. `heading` breaks
        ties between equally near points in favour of the way the aircraft is
        already going (this tick's Shield action, else the pilot's). A search
        that raises - a geometry error, a rule the search cannot read - must
        never stop the 10 Hz loop: it becomes an in-place plan marked illegal
        whose reason names the exception, so the record shows it."""
        from . import holdpoint as _hp      # local: holdpoint imports this module
        mode = "LAND" if self.fsm.state is _fsm.FSMState.LAND else "LOITER"
        unsafe = self.hard_violations((tick.unsafe if stop_illegal is None
                                       else stop_illegal) or [])
        state = self._positional(state)
        heading = None
        for a in (d.emitted, d.raw):
            if math.hypot(a.vx, a.vy) > 0.1:
                heading = (a.vx, a.vy)
                break
        try:
            if mode == "LOITER":
                # How long the hold must stay legal: the Loiter's remaining
                # time under FSMConfig.hold_horizon "loiter" (since
                # 2026-11-07), None (one lookahead, as before) otherwise.
                return _hp.choose_hold_point(self, state, unsafe=unsafe, heading=heading,
                                             search=self.hold_points,
                                             hold_s=self.fsm.hold_horizon_s())
            return _hp.choose_landing_site(self, state, unsafe=unsafe, heading=heading,
                                           search=self.hold_points,
                                           site_ok=self.land_site_ok)
        except Exception as e:                      # noqa: BLE001 - see docstring
            return _fsm.HoldPoint(
                mode, "in_place", state.x, state.y, state.up, legal=False,
                breaks=tuple(sorted({v.rule_id for v in unsafe})),
                reason=(f"hold-point search failed ({type(e).__name__}: {e}); "
                        f"holding where the aircraft is, legality not established"))
