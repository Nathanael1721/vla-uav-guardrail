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
  * Since 2026-10-12 the repair stack runs as guardrail/repair.py's operator
    library: the chain above, decision for decision, each call recorded
    (`ShieldDecision.trace`, the audit record's `repair_passes`), with the
    grant's first-feasible selection, PathRepair and a forecast segment check
    behind constructor flags, all off by default. A moving zone's polygon is
    built only when its anchor box meets the forecast (guardrail/ir.py
    `anchor_reach`).
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
from .geometry import nearest_on_polyline, point_in_fence, push_out_direction
from .ir import FENCE_TYPES, FenceRecord, PolicyIR, anchor_reach, compile_fence, is_fence
from .models import (
    DAYS,
    DEFAULT_LAYER,
    Action4D,
    AltitudeEnvelope,
    Corridor,
    CorridorSwap,
    DynamicNFZ,
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
    "time_window_switch", "corridor_swap"})

# Breach actions that skip projection and stop. The PI reference's shield.py
# skips projection only when the WORST violation's action is RTL or land ("Rules
# whose action is a direct fail-safe skip projection entirely"), emits the raw
# action and lets its FSM change mode; it still repairs for brake and loiter.
# Stopping for brake and loiter too is this project's reading (follow-up #112,
# docs/DESIGN-escalation-fsm.md), and so is stopping rather than passing the
# raw action through while the mode change is requested.
_STOP_ACTIONS = ("brake", "loiter", "RTL", "land")


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
    for k in ("valid_time", "scope", "layer", "altitude_ref"):
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

# Length of the sliding window of recent ticks. Safety Shield page, node
# architecture: "Sliding-window buffer - last 50 actions + predicted
# trajectory", feeding the violation checker. 50 ticks is 5 s at the 10 Hz
# monitor rate, the same span as the grant's lookahead.
HISTORY_LEN = 50


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
    # What is STILL wrong with the action that was actually flown.
    #
    # `violations` is the check on `raw`, so on its own it cannot answer the
    # grant's hard KPI - "a P0 that was seen and then flown anyway". The escape
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

    @property
    def touched(self) -> bool:
        return bool(self.violations)

    @property
    def escaped(self) -> bool:
        """Some rule is still violated by the action that was flown, of ANY
        priority: `emitted_violations` also holds monitor_only rules (never
        repaired, by design) and soft rules given up (`relaxed`). The grant's
        P0 escape rate is this filtered to P0 rules, as guardrail/kpi.py and
        the policy HUD do; the decision does not carry priorities."""
        return bool(self.emitted_violations)


class _Pose(NamedTuple):
    """One forecast pose (Shield._predict): State's four fields, unvalidated
    because they are computed from validated floats."""
    x: float
    y: float
    up: float
    yaw_deg: float


class TickRecord(NamedTuple):
    """One entry of the Shield's sliding window: where the vehicle was and what
    the Shield decided. The predicted trajectory the spec puts beside it is
    recomputable from these two (`Shield.forecast(rec.state,
    rec.decision.emitted)`), so it is not paid for on every tick.

    `elapsed_ms` is how long that `filter()` call took, wall clock. The grant
    budgets the monitor at 100 ms per tick (Safety Shield page) and no flight
    rail recorded the Shield's share of it (audit card WP3-13); measuring it
    here, once, means a rail only has to log `history[-1].elapsed_ms` rather
    than wrap its own timer around a call it might later move."""
    state: State
    decision: ShieldDecision
    elapsed_ms: float


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
    switches_on: dict           # target id -> time_window_switch rules, policy order
    swaps_on: dict              # target id -> corridor_swap rules, policy order
    eff: dict                   # rule id -> effective breach action
    hard: dict                  # rule id -> hard?
    timed: frozenset            # ids whose in-force state can change with time or events
    recurrences: tuple          # every weekly schedule in the policy
    anchors: dict               # moving dynamic_nfz id -> anchor time (s), None = first tick
    moving_ids: frozenset


def _build_runtime(policy: Policy, prev: "_Runtime | None", anchors: dict) -> _Runtime:
    cons = tuple(policy.constraints)
    switches_on: dict = {}
    swaps_on: dict = {}
    for c in cons:
        if isinstance(c, TimeWindowSwitch):
            switches_on.setdefault(c.target_id, []).append(c)
        elif isinstance(c, CorridorSwap):
            swaps_on.setdefault(c.target_id, []).append(c)
    timed = {c.id for c in cons if c.valid_time is not None
             and c.valid_time.recurrence is not None}
    timed |= set(switches_on) | set(swaps_on)
    timed |= {c.id for c in cons if isinstance(c, CorridorSwap)}
    recs = tuple(c.valid_time.recurrence for c in cons
                 if c.valid_time is not None and c.valid_time.recurrence is not None)
    ir = PolicyIR.from_policy(policy, reuse=prev.ir if prev is not None else None)
    moving = frozenset(r.rule.id for r in ir.fences if r.moving)
    return _Runtime(
        ir=ir, constraints=cons,
        kins=tuple(c for c in cons if isinstance(c, KinematicEnvelope)),
        alts=tuple(c for c in cons if isinstance(c, AltitudeEnvelope)),
        clear=tuple(c for c in cons if isinstance(c, ObstacleClearance)),
        standoffs=tuple(c for c in cons if isinstance(c, SubjectStandoff)),
        corridors=tuple(c for c in cons if isinstance(c, (Corridor, CorridorSwap))),
        switches_on={k: tuple(v) for k, v in switches_on.items()},
        swaps_on={k: tuple(v) for k, v in swaps_on.items()},
        eff={c.id: _effective_action(c) for c in cons},
        hard={c.id: c.constraint_type == "hard" for c in cons},
        timed=frozenset(timed), recurrences=recs,
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

    @staticmethod
    def _base(rule, x):
        """The rule's own window at instant x: True, False, or None when it has
        a window and there is no clock to read it."""
        vt = rule.valid_time
        if vt is None or vt.recurrence is None:
            return True
        if x is None:
            return None
        return recurrence_active(vt.recurrence, x)

    def on(self, rule, x) -> bool:
        """Is `rule` enforced at instant x (switches and swaps applied)?

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
        rt = self.rt
        if isinstance(rule, TimeWindowSwitch):
            return False                       # an event, not a geometric rule
        if isinstance(rule, CorridorSwap):
            return self._base(rule, x) is not False
        for sp in rt.swaps_on.get(rule.id, ()):
            if self._base(sp, x) is True:
                return False
        sws = rt.switches_on.get(rule.id, ())
        if sws:
            if any(sw.active and self._base(sw, x) is None for sw in sws):
                return True
            known = [sw.active for sw in sws if self._base(sw, x) is True]
            if known:
                return known[-1]
        return self._base(rule, x) is not False

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
                 "_t_first", "given_up", "chain_runs", "ledger")

    def __init__(self, rt: _Runtime, sched: _Schedule, t: float, t_first: float | None):
        self.rt, self.sched, self.t, self._t_first = rt, sched, t, t_first
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
                 segment_check: bool = False, repair_order: tuple | None = None):
        """obstacle_map: None, or {"occ": uint8 NxN (1 = blocked), "res": float,
        "ox": float, "oy": float} — the same occupancy grid the global planner
        uses. A policy YAML cannot carry an 80x80 grid, so obstacle_clearance
        rules get their world here. With obstacle_map=None those rules are
        inert and the Shield behaves exactly as before.

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
        here."""
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
        self._tubes: dict = {}
        why = self.unenforceable(policy)
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
        self._rt = _build_runtime(policy, None, dyn)
        # The last HISTORY_LEN ticks, oldest first; see `history`.
        self._window: deque[TickRecord] = deque(maxlen=HISTORY_LEN)
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
        if obstacle_map is not None and self._rt.clear:
            self._res = float(obstacle_map["res"])
            self._ox = float(obstacle_map["ox"])
            self._oy = float(obstacle_map["oy"])
            self._dist = _build_signed_distance_field(obstacle_map["occ"], self._res)

    @staticmethod
    def unenforceable(policy: Policy) -> list[str]:
        """Rules of `policy` this Shield would NOT enforce, with the reason.
        A Shield is never built over them: a Policy object made directly (not
        through a flight loader) reaches here without the loaders' gate."""
        out = []
        for c in policy.constraints:
            if c.type not in ENFORCED_TYPES:
                out.append(f"{c.id}: {c.type} is declarable in the DSL but the "
                           f"Safety Shield does not enforce it")
            if getattr(c, "altitude_ref", None) == "MSL":
                out.append(f"{c.id}: altitude_ref MSL - heights are measured above "
                           f"ground and there is no ground-elevation source")
        return out

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

    def forecast(self, state: State, action: Action4D) -> list[tuple[float, State]]:
        """The predicted trajectory the fence and altitude rules judge: (t, pose)
        pairs over the lookahead. Public so the sliding window's "predicted
        trajectory" can be rebuilt for any entry - see TickRecord."""
        return [(t, State(x=p.x, y=p.y, up=p.up, yaw_deg=p.yaw_deg))
                for t, p in self._predict(state, action)]

    @property
    def history(self) -> tuple[TickRecord, ...]:
        """The sliding window: the last HISTORY_LEN ticks through `filter()`,
        oldest first.

        The grant's node design puts this buffer in front of the checker. No
        rule in this DSL reads past actions yet, so today it is a record, not
        an input - it lets a caller (the HUD, an audit writer, a future
        oscillation rule) see what the Shield did over the last 5 s without
        keeping a second copy that could disagree with it. Every return path of
        `filter()` appends, the brake and rescue branches included."""
        return tuple(self._window)

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
    def mission_started(self) -> bool:
        """True from the first filter() tick: the grant's "mission start", after
        which every rule class but the three hot-applicable ones is locked."""
        return self._ticks > 0

    # ---------------- obstacle distance field ---------------- #

    def _distance_at(self, x: float, y: float) -> float:
        """Approx. SIGNED metres from world point (x, y) to the nearest mapped
        obstacle — negative inside a building footprint. inf when there is no
        map. Bilinear inside the grid; outside it, a safe lower bound built from
        the distance to the map edge (every obstacle lives inside the map, so
        being far off-map IS clear)."""
        d = self._dist
        if d is None:
            return float("inf")
        n, m = d.shape
        fi = (x - self._ox) / self._res
        fj = (y - self._oy) / self._res
        ci = min(max(fi, 0.0), n - 1.0)
        cj = min(max(fj, 0.0), m - 1.0)
        off = math.hypot((fi - ci) * self._res, (fj - cj) * self._res)

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
            return max(v - off, min(v + off, off - 0.5 * self._res))
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

    def _away_dir(self, x: float, y: float, probe_r: float) -> tuple[float, float]:
        """Unit vector along the local gradient of the distance field, i.e.
        'the way AWAY from the nearest building'. (0, 0) when even the probe
        finds nothing to steer by — the caller then leaves the action alone and
        the P0 re-check decides."""
        if self._dist is None:
            return 0.0, 0.0
        h = max(self._res * 0.5, 1e-3)
        gx = (self._distance_at(x + h, y) - self._distance_at(x - h, y)) / (2 * h)
        gy = (self._distance_at(x, y + h) - self._distance_at(x, y - h)) / (2 * h)
        norm = math.hypot(gx, gy)
        if norm > 1e-6:
            return gx / norm, gy / norm
        # Flat field (a symmetric ridge between two buildings, the exact centre
        # of a footprint): probe 8 compass directions and take the one that
        # gains the most distance, WIDENING the radius until something gains.
        # "No direction" must never fall through to a brake — a stationary
        # vehicle never recedes, so the violation would repeat forever.
        here = self._distance_at(x, y)
        r0 = max(probe_r, self._res)
        for mult in (1.0, 2.0, 4.0):
            r = r0 * mult
            best, best_d = (0.0, 0.0), here
            for k in range(8):
                ang = k * math.pi / 4.0
                ux, uy = math.cos(ang), math.sin(ang)
                dv = self._distance_at(x + ux * r, y + uy * r)
                if dv > best_d + 1e-9:
                    best, best_d = (ux, uy), dv
            if best != (0.0, 0.0):
                return best
        return 0.0, 0.0

    # ---------------- violation checking ---------------- #

    def _predict(self, state: State, action: Action4D,
                 horizon_s: float | None = None):
        """Constant-velocity forecast (the grant Shield's 50-pose forecast at
        the default 5 s / 0.1 s; a caller may pass a shorter one). `horizon_s`
        overrides the default lookahead — the clearance
        rule uses a much shorter, braking-distance-sized one (see
        `_clear_horizon_s`). The final sample always lands exactly ON the
        horizon, so a horizon that is not a whole number of `dt` still gets its
        endpoint checked."""
        horizon = self.lookahead_s if horizon_s is None else horizon_s
        t = 0.0
        # A _Pose, not a State: the four values are already floats (sums and
        # products of a validated State's and Action4D's), and validating
        # each of 51 poses again was a fifth of a 50-rule filter() (2026-10-12
        # profile). `forecast` still returns States.
        while True:
            yield t, _Pose(
                x=state.x + action.vx * t,
                y=state.y + action.vy * t,
                up=state.up + action.vz_up * t,
                yaw_deg=state.yaw_deg + action.yaw_rate * t,
            )
            if t >= horizon - 1e-9:
                return
            t = min(t + self.dt, horizon)

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

    def _clear_min_dist(self, state: State, a: Action4D) -> float:
        """Smallest mapped-obstacle distance `a`'s FORECAST reaches. This is the
        score every clearance repair is judged by: a repair must raise it, never
        lower it.

        The t = 0 sample is excluded on purpose — every candidate action shares
        the present position, so scoring it in makes the number blind exactly
        when the vehicle is already inside the ring and the choice matters most.
        (A standstill has no forecast, so it scores its own position.)"""
        if self._dist is None:
            return float("inf")
        steps = list(self._predict(state, a, self._clear_horizon_s(a)))
        return min(self._distance_at(p.x, p.y) for _, p in (steps[1:] or steps))

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

    def _best_clear_dir(self, state: State, a: Action4D, cap: float):
        """The candidate heading whose clearance forecast keeps the most room,
        ties broken toward the commanded heading."""
        if self._dist is None:
            return None
        best, best_key = None, None
        for cand in self._clear_dir_candidates(a, cap):
            key = (self._clear_min_dist(state, cand),
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
            sx, sy = self._subject
            d = math.hypot(state.x - sx, state.y - sy)
            if d > _STANDOFF_COINCIDENT_M:
                closing = (a.vx * (sx - state.x) + a.vy * (sy - state.y)) / d
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
            ox, oy = push_out_direction(state.x, state.y, rec.polygon)
            mv = self._motion_of(f)
            ex, ey = (a.vx - mv[0], a.vy - mv[1]) if mv else (a.vx, a.vy)
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
        h_speed = (action.vx ** 2 + action.vy ** 2) ** 0.5
        if h_speed > k.speed_max_mps + 1e-9:
            out.append(Violation(
                rule_id=k.id, category="kinematic", predicted_at_s=0.0,
                detail=f"h-speed {h_speed:.2f} > max {k.speed_max_mps}"))
        if abs(action.vz_up) > k.climb_rate_max_mps + 1e-9:
            out.append(Violation(
                rule_id=k.id, category="kinematic", predicted_at_s=0.0,
                detail=f"|vz| {abs(action.vz_up):.2f} > max {k.climb_rate_max_mps}"))
        yaw_dps = math.degrees(abs(action.yaw_rate))     # contract is rad/s
        if yaw_dps > k.yaw_rate_max_dps + 1e-9:
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
        for t, p in self._predict(state, action):
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
        """
        if self._subject is None or not so.binds(self._subject_class):
            return []
        alive = self._mask(so)
        if alive is False:
            return []
        sx, sy = self._subject
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
                opening = -(action.vx * ux + action.vy * uy)
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
                # PI reference takes for a zone.
                opening = math.hypot(action.vx, action.vy)
            if opening <= 0.1:
                return [Violation(
                    rule_id=so.id, category="standoff", predicted_at_s=0.0,
                    detail=(f"range {d_now:.1f}m to {what} < min "
                            f"{so.min_range_m}m and not opening"))]
            return []        # while inside, predictive checks are moot

        for t, p in self._predict(state, action):
            if alive is not True and not alive(t):
                continue
            d = math.hypot(p.x - sx, p.y - sy)
            if self._standoff_inside(so, d):
                return [Violation(
                    rule_id=so.id, category="standoff", predicted_at_s=t,
                    detail=(f"range closes to {d:.1f}m from {what} in "
                            f"{t:.1f}s, below min {so.min_range_m}m"))]
        return []

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
        mv = self._motion_of(f)
        if self.segment_check and poses is None:
            poses = list(self._predict(state, action))
        if (alive is True or alive(0.0)) and point_in_fence(
                state.x, state.y, state.up, f, poly, buffered):
            ox, oy = push_out_direction(state.x, state.y, poly)
            # Escaping is judged RELATIVE to a moving zone: leaving at 1 m/s a
            # zone that follows at 2 m/s is not leaving it.
            ex, ey = ((action.vx - mv[0], action.vy - mv[1]) if mv
                      else (action.vx, action.vy))
            escaping = (ex * ox + ey * oy) > 0.1
            if not escaping:
                return [Violation(
                    rule_id=f.id, category="geofence", predicted_at_s=0.0,
                    detail="currently INSIDE zone and not escaping")]
            return []      # while inside, predictive entry checks are moot
        for t, p in (poses if poses is not None else self._predict(state, action)):
            if alive is not True and not alive(t):
                continue
            px, py = _to_zone_frame(p, t, mv) if mv else (p.x, p.y)
            if point_in_fence(px, py, p.up, f, poly, buffered):
                return [Violation(
                    rule_id=f.id, category="geofence", predicted_at_s=t,
                    detail=f"predicted pos ({p.x:.1f},{p.y:.1f}) inside NFZ at t+{t:.1f}s")]
        if self.segment_check:
            ring = buffered if buffered is not None else poly.buffer(f.margin_m)
            hit = self._segment_crossing(f, ring, poses, alive, mv)
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

    def _check_corridor(self, c, state: State, action: Action4D) -> list[Violation]:
        """Keep-IN, trend-aware. The mirror image of _check_fence.

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
        pts = c.points()
        half = c.half_width_m
        d_now, (ux, uy) = self._corridor_geom(c, state.x, state.y)
        wide = d_now > half + 1e-9
        below = state.up < c.altitude_floor_m - 1e-9
        above = state.up > c.altitude_ceiling_m + 1e-9

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

        for t, p in self._predict(state, action):
            if alive is not True and not alive(t):
                continue
            d, _, _ = nearest_on_polyline(p.x, p.y, pts)
            if d > half + 1e-9:
                return [Violation(
                    rule_id=c.id, category="corridor", predicted_at_s=t,
                    detail=(f"predicted {d:.1f}m off centerline at t+{t:.1f}s, "
                            f"half-width {half:.1f}m"))]
            if not (c.altitude_floor_m - 1e-9 <= p.up
                    <= c.altitude_ceiling_m + 1e-9):
                return [Violation(
                    rule_id=c.id, category="corridor", predicted_at_s=t,
                    detail=(f"predicted alt {p.up:.1f}m outside corridor band "
                            f"[{c.altitude_floor_m}, {c.altitude_ceiling_m}] "
                            f"at t+{t:.1f}s"))]
        if self.segment_check:
            hit = self._corridor_segment_exit(c, self._predict(state, action), alive)
            if hit is not None:
                t_out, (x0, y0, t0), (x1, y1, t1) = hit
                return [Violation(
                    rule_id=c.id, category="corridor", predicted_at_s=t_out,
                    detail=(f"predicted segment ({x0:.1f},{y0:.1f})->({x1:.1f},{y1:.1f}) "
                            f"leaves the corridor between t+{t0:.1f}s and t+{t1:.1f}s, "
                            f"half-width {half:.1f}m"))]
        return []

    def _clear_forecast(self, state: State, action: Action4D):
        """(steps, dists) over the clearance horizon.

        Shared so a repair that has already paid for the forecast does not pay
        again, once per rule per repair pass.
        """
        steps = list(self._predict(state, action, self._clear_horizon_s(action)))
        return steps, [self._distance_at(p.x, p.y) for _, p in steps]

    def _check_clearance(self, c, state: State, action: Action4D,
                         steps=None, dists=None) -> list[Violation]:
        """Trend-aware, same shape as the geofence rule."""
        if self._dist is None:
            return []
        alive = self._mask(c)
        if alive is False:
            return []
        if steps is None or dists is None:
            steps, dists = self._clear_forecast(state, action)
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
        move gets a tick view for this one call (the clock is read once)."""
        if self._tick_ctx() is None and self._needs_tick():
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
        if recs:
            # One forecast for every fence (it was rebuilt per fence), and only
            # the fences the forecast's bounding box reaches. Policy order is
            # kept, so the violation list - which test_check_contract.py hashes
            # - comes out in the same order as the old walk over all of them.
            poses = list(self._predict(state, action))
            near = self._fence_candidates(ir, state, poses)
            box, horizon = self._forecast_box(state, poses)
            for r in recs:
                if r.index not in near:
                    continue
                r = self._reach(r, tick, box, horizon)
                if r is not None:
                    found += self._check_fence(r.rule, r.polygon, state, action,
                                               poses, r.buffered)
        for c in self._corridors:
            found += self._check_corridor(c, state, action)
        if self._dist is not None and self._clear:
            # One forecast shared across every clearance rule, as before.
            steps, dists = self._clear_forecast(state, action)
            for c in self._clear:
                found += self._check_clearance(c, state, action, steps, dists)

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
        # counts failed on GitHub only.
        vx, vy, vz, yr = a.vx, a.vy, a.vz_up, a.yaw_rate
        for k in self._kins:
            h = (vx ** 2 + vy ** 2) ** 0.5
            if h > k.speed_max_mps + 1e-9:
                s = k.speed_max_mps / h
                vx, vy = vx * s, vy * s
                repairs.append(Repair(operator="SpeedClamp",
                                      detail=f"h-speed {h:.2f} -> {k.speed_max_mps}"))
            if abs(vz) > k.climb_rate_max_mps + 1e-9:
                new = k.climb_rate_max_mps * (1 if vz > 0 else -1)
                repairs.append(Repair(operator="ClimbClamp", detail=f"vz {vz:.2f} -> {new:.2f}"))
                vz = new
            ymax = math.radians(k.yaw_rate_max_dps)
            if math.degrees(abs(yr)) > k.yaw_rate_max_dps + 1e-9:
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

    def _repair_altitude(self, state: State, a: Action4D, repairs: list[Repair]) -> Action4D:
        """Project vz so the lookahead endpoint lands inside the band.

        Two cases that look alike and are not:

          * ALREADY outside - aim a margin INSIDE the band, so the recovery
            actually arrives. See _reentry_margin.
          * inside and about to overshoot - aim at the boundary exactly. There
            is nothing to recover from, and pulling further in would fight a
            legal cruise that happens to sit near the edge of its own envelope.
        """
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
        if self._dist is None or not self._clear:
            return a
        vx, vy = a.vx, a.vy
        cap = min((k.speed_max_mps for k in self._kins), default=4.0)

        for c in self._clear:
            hard_r = c.min_clearance_m
            soft_r = c.min_clearance_m + c.soft_margin_m
            probe = Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
            steps, dists = self._clear_forecast(state, probe)

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
                # the monitor is happy with.
                oxc, oyc = self._away_dir(state.x, state.y, hard_r)
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
            ox, oy = self._away_dir(hit.x, hit.y, hard_r)
            if ox == 0.0 and oy == 0.0:
                ox, oy = self._away_dir(state.x, state.y, hard_r)
            if ox == 0.0 and oy == 0.0:
                continue                       # nothing to steer by; rescue decides

            # `before` measured the SAME way as `after`.
            #
            # It used to be `min(dists)`, which includes t = 0, while `after`
            # comes from _clear_min_dist, which excludes it. At (38, 20.5)
            # flying away at 5 m/s that is 0.886 against 3.696 - a 4.2x gap, so
            # `after > before` held automatically and the "never leave it
            # worse" invariant below was suppressed exactly where it mattered.
            before = self._clear_min_dist(state, probe)

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
            after = self._clear_min_dist(state, cand)
            if after <= before + 1e-9:
                esc = self._best_clear_dir(state, probe, cap)
                esc_d = self._clear_min_dist(state, esc) if esc is not None else None
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
        near_for = poses = near = box = horizon = None
        anchors, tick = self._fence_anchors(ir)
        for rec in anchors:
            if near_for != (vx, vy):
                probe = Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
                poses = list(self._predict(state, probe))
                near = self._fence_candidates(ir, state, poses)
                box, horizon = self._forecast_box(state, poses)
                near_for = (vx, vy)
            if rec.index not in near:
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
                ox, oy = push_out_direction(state.x, state.y, poly)
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
                spd = min(2.0 + max(0.0, zx * ox + zy * oy), cap)
                out_now = vx * ox + vy * oy          # outward speed commanded
                probe = Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate)
                if not self._check_fence(f, poly, state, probe, poses, ring):
                    # The monitor is content, so this action IS escaping - but
                    # it forgives anything above 0.1 m/s, which would leave a
                    # crawl. Raise the outward component to the floor and leave
                    # the tangential alone rather than reshaping a legal action.
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

            # `poses` is the forecast of the current (vx, vy): it was rebuilt
            # above the moment the velocity last changed.
            alive = self._mask(f)
            depth = 0.0
            hit = False
            for t, p in poses:
                if alive is not True and not alive(t):
                    continue
                px, py = _to_zone_frame(p, t, mv) if mv else (p.x, p.y)
                if point_in_fence(px, py, p.up, f, poly, ring):
                    hit = True
                    # Penetration depth of the forecast being repaired: how
                    # far inside the ring its deepest pose lies (the grant's
                    # magnitude_m for a lateral projection).
                    depth = max(depth, ring.boundary.distance(Point(px, py)))
            if not hit and self.segment_check:
                # A ring that lies wholly between two poses: the slide acts on
                # it as on a pose inside, sized by the segment's own depth.
                seg = self._segment_crossing(f, ring, poses, alive, mv)
                if seg is not None:
                    hit, depth = True, seg[1]
            if not hit:
                continue

            ox, oy = push_out_direction(state.x, state.y, poly)   # unit "away from zone"
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
                    if self._dist is not None and self._clear:
                        hard = min(c.min_clearance_m for c in self._clear)
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
                    subject_class: str | None = None) -> None:
        """Tell the Shield where the tracked subject is, once per tick.

        The Shield cannot see. SubjectStandoff rules are inert until the
        perception stack supplies this, and calling `set_subject(None)` when the
        target is lost is REQUIRED rather than optional: a stale position would
        have the Shield enforcing a standoff from where the subject used to be,
        which is both wrong and unfalsifiable from the logs.

        `subject_class` is what selects between per-class rules, so a policy can
        hold 10 m from a pedestrian and 5 m from a vehicle.
        """
        if x is None or y is None:
            self._subject = None
            self._subject_class = None
            return
        self._subject = (float(x), float(y))
        self._subject_class = subject_class

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
        """
        if self._subject is None:
            return a
        sx, sy = self._subject
        worst = None
        for so in self._standoffs:
            if so.binds(self._subject_class):
                worst = so if worst is None or so.min_range_m > worst.min_range_m else worst
        if worst is None:
            return a

        dx, dy = sx - state.x, sy - state.y
        d = math.hypot(dx, dy)
        ring = worst.min_range_m
        if d <= _STANDOFF_COINCIDENT_M:
            return self._recover_from_overhead(state, a, ring, d, repairs)
        ux, uy = dx / d, dy / d            # unit vector pointing AT the subject

        closing = a.vx * ux + a.vy * uy    # positive means approaching

        # The REPAIR must trigger on the same criterion the CHECK uses, or it
        # declines to act on the very violation that was raised and the action
        # falls through to the brake - and, inside the ring, to the rescue
        # search, which was measured emitting +5 m/s straight AT the subject.
        # The check is predictive over the full lookahead, so this must be too:
        # at 4 m/s and a 3 s horizon the aircraft commits 12 m ahead of itself.
        breach_ahead = (d - closing * self.lookahead_s) < ring - 1e-9

        if d < ring - 1e-9:
            # ALREADY inside. Removing the closing component would leave the
            # range exactly where it is, which the check reads as "not opening" -
            # so the violation would persist every tick and the aircraft would be
            # frozen at a distance the policy forbids. Recovery means opening the
            # range, the same reasoning as the clearance ring's push-out.
            cap = min((k.speed_max_mps for k in self._kins), default=4.0)
            want = min(cap, max(0.5, (ring - d) / max(self.lookahead_s, 1e-3)))
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
            # policy forbids near a person.
            out = min(cap, max(want, -closing))
            vx = a.vx - (closing + out) * ux
            vy = a.vy - (closing + out) * uy
            repairs.append(Repair(
                operator="StandoffRecover",
                detail=(f"range {d:.1f}m inside min {ring}m: opening at "
                        f"{out:.2f} m/s (commanded {-closing:+.2f}, "
                        f"recovery needs {want:.2f}), tangential motion kept"),
                magnitude_m=ring - d, axis="lateral", recovery=True))
            return self._cap_sparing_radial(
                Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=a.yaw_rate),
                ux=-ux, uy=-uy, keep=out, cap=cap)

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
        # (guardrail/fsm.py DEFAULT_THETA_HORIZON_S).
        d_min = min(math.hypot(p.x - sx, p.y - sy) for _, p in self._predict(state, a))
        repairs.append(Repair(
            operator="StandoffHold",
            detail=(f"range {d:.1f}m, closing {closing:.2f} m/s would breach "
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
        most), and never lowered."""
        cap = min((k.speed_max_mps for k in self._kins), default=4.0)
        want = min(cap, max(0.5, (ring - d) / max(self.lookahead_s, 1e-3)))
        h = math.hypot(a.vx, a.vy)
        if h > 1e-6:
            ex, ey = a.vx / h, a.vy / h
            how = "along the commanded heading"
        else:
            yaw = math.radians(state.yaw_deg)
            ex, ey = -math.cos(yaw), -math.sin(yaw)
            how = "backing away from the aircraft's heading"
        out = min(cap, max(want, h))
        repairs.append(Repair(
            operator="StandoffRecover",
            detail=(f"range {d:.1f}m: directly over the subject, every heading "
                    f"opens the range; leaving {how} at {out:.2f} m/s "
                    f"(commanded {h:.2f}, recovery needs {want:.2f})"),
            magnitude_m=ring - d, axis="lateral", recovery=True))
        return Action4D(vx=ex * out, vy=ey * out, vz_up=a.vz_up, yaw_rate=a.yaw_rate)

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

        for c in self._corridors:
            # Altitude first: it is independent of the lateral geometry and its
            # repair is a straight projection, exactly as for AltitudeEnvelope.
            end_up = state.up + a.vz_up * L
            if end_up > c.altitude_ceiling_m or end_up < c.altitude_floor_m:
                # Aim a margin inside when already outside, for the same reason
                # AltitudeFix does - see Shield._reentry_margin. Targeting the
                # edge converges on it without ever crossing.
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
                excursion, recovering = d - half, True
            elif d + outward * L > half + 1e-9:
                need = 0.0
                excursion, recovering = d + outward * L - half, False
            else:
                continue                        # forecast stays inside
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

    def state_is_unsafe(self, state: State) -> list[Violation]:
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
        """
        return self._check(state, BRAKE)

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
        x, y, up = state.x, state.y, state.up
        total = 0.0
        for env in self._alts:
            if env.id in ids:
                total += max(0.0, env.alt_min_m - up, up - env.alt_max_m)
        if self._subject is not None:
            sx, sy = self._subject
            d = math.hypot(x - sx, y - sy)
            for so in self._standoffs:
                if so.id in ids and so.binds(self._subject_class):
                    total += max(0.0, so.min_range_m - d)
        if self._dist is not None:
            here = self._distance_at(x, y)
            for c in self._clear:
                if c.id in ids:
                    total += max(0.0, c.min_clearance_m - here)
        for c in self._corridors:
            if c.id in ids:
                d_lat, _ = self._corridor_geom(c, x, y)
                total += (max(0.0, d_lat - c.half_width_m)
                          + max(0.0, c.altitude_floor_m - up, up - c.altitude_ceiling_m))
        anchors, tick = self._fence_anchors(self._ir)
        for rec in anchors:
            f = rec.rule
            if f.id not in ids:
                continue
            if rec.moving and tick is not None:
                rec = tick.current(rec)     # only the zones a violation names
            if point_in_fence(x, y, up, f, rec.polygon, rec.buffered):
                total += rec.buffered.boundary.distance(Point(x, y))
        return total

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
        cand = self.policy.model_copy(update={"constraints": constraints})
        new = sorted(set(cand.lint()) - set(self.policy.lint()))
        new = [f for f in new if not f.startswith("the policy has no rules")]
        if new:
            raise HotApplyRefused(f"{op} {rule_id!r} refused: the policy it would leave "
                                  f"fails the DSL lint: " + "; ".join(new))
        self.policy.constraints = constraints
        self.policy.generation += 1
        if anchors is not None:
            self._anchors = anchors
        self._rt = _build_runtime(self.policy, self._rt, self._anchors)
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

    @staticmethod
    def _cross_layer_hard(tgt, layer: str) -> bool:
        return tgt.constraint_type == "hard" and tgt.effective_layer != layer

    def _guard_zone_edit(self, z, op: str, layer: str) -> None:
        if self._cross_layer_hard(z, layer):
            raise LayerRelaxation(
                f"{op} {z.id!r} refused: it is a hard {z.effective_layer}-layer zone "
                f"and this event is {layer}-layer; moving, reshaping or expiring it "
                f"can relax it, and a rule may not be relaxed from another layer. "
                f"Send the event from the {z.effective_layer} layer "
                f"(layer={z.effective_layer!r}, Python API only), or spawn a new zone")

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
            if tgt.effective_layer != layer:
                raise LayerRelaxation(
                    f"{sw.id}: a {layer}-layer switch may not turn off {tgt.id!r}, a hard "
                    f"{tgt.effective_layer}-layer rule (Policy DSL page, layered "
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
            sw = TimeWindowSwitch(id=sid, type="time_window_switch",
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
            # on a swap at ingest, on the same corridor it builds.
            as_corr = Corridor(id=tgt.id, type="corridor",
                               constraint_type=sp.constraint_type,
                               priority=sp.priority, violation_action=sp.violation_action,
                               valid_time=sp.valid_time, altitude_ref=sp.altitude_ref,
                               centerline=sp.centerline, width_m=sp.width_m,
                               altitude_floor_m=sp.altitude_floor_m,
                               altitude_ceiling_m=sp.altitude_ceiling_m)
            why = _not_looser(as_corr, tgt)
            if why:
                raise LayerRelaxation(
                    f"{sp.id}: the swap {why}, relaxing hard {tgt.effective_layer}-layer "
                    f"corridor {tgt.id!r} from the {layer} layer (Policy DSL page, "
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
                    clearance_m: float | None = None) -> list[dict]:
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
        """
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
                in_band = c.altitude_floor_m <= state.up <= c.altitude_ceiling_m
                d = 0.0 if in_ring else rec.buffered.distance(P)
                row.update(bound=in_force and in_band, inside=inside,
                           in_margin=in_ring and not inside, in_band=in_band,
                           value=d, distance_m=d, margin_m=float(c.margin_m),
                           vertices=[(float(x), float(y)) for x, y in
                                     list(rec.polygon.exterior.coords)[:-1]],
                           breach=in_force and point_in_fence(
                               state.x, state.y, state.up, c, rec.polygon, rec.buffered))
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
                    row.update(bound=in_force, value=sep, distance_m=sep - c.min_range_m,
                               breach=in_force and self._standoff_inside(c, sep))
            elif c.type == "obstacle_clearance":
                d = clearance_m if clearance_m is not None else (
                    self._distance_at(state.x, state.y) if self._dist is not None else None)
                row["off_map"] = self.off_map(state.x, state.y)
                if d is not None and math.isfinite(d):
                    row.update(bound=in_force, value=d, distance_m=d - c.min_clearance_m,
                               breach=in_force and d < c.min_clearance_m)
            elif c.type in ("corridor", "corridor_swap"):
                off, _ = self._corridor_geom(c, state.x, state.y)
                in_band = c.altitude_floor_m - 1e-9 <= state.up <= c.altitude_ceiling_m + 1e-9
                row.update(bound=in_force, value=off, distance_m=c.half_width_m - off,
                           in_band=in_band, centerline=list(c.points()),
                           width_m=float(c.width_m),
                           breach=in_force and (off > c.half_width_m + 1e-9 or not in_band))
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
            if up is not None and not (r.rule.altitude_floor_m <= up
                                       <= r.rule.altitude_ceiling_m):
                continue
            out.append(tick.current(r) if r.moving else r)
        return out

    def fence_distance(self, x: float, y: float, up: float | None = None) -> float:
        """Metres from (x, y) to the nearest in-force zone's margin ring (0
        inside it); inf with no zone. The geometry the zone rule enforces."""
        p = Point(x, y)
        zones = self.zones_now(up)
        return min((r.buffered.distance(p) for r in zones), default=float("inf"))

    # ---------------- the public entry point ---------------- #

    def filter(self, state: State, raw: Action4D, t: float | None = None, *,
               rtl_failed: bool = False, home_reached: bool = False,
               landed: bool = False) -> ShieldDecision:
        """One monitor tick: check, repair, re-check, the brake/rescue
        fallbacks, then the escalation FSM. Every decision, whichever branch
        produced it, goes into the sliding window (`history`) before it is
        returned, with the time the decision took.

        `t` is the monitor clock in seconds (monotonic; the simulation or the
        rail's clock). Without it, ticks are counted at the grant's 10 Hz.

        rtl_failed / home_reached / landed: what the autopilot reports this
        tick, for the FSM's terminal edges (G6, G9, G10, X5). Only a caller
        that flies the Shield's own FSM verdict passes them; the flight rails
        run their own FSM (guardrail/replay.py rail_shield) and leave them."""
        t0 = time.perf_counter()
        tick = self._begin_tick(t)
        with self._scope(tick):
            decision = self._decide(state, raw)
            self._last_caps = self._applied_caps(tick, decision)
            decision = self._escalate(tick, state, decision, rtl_failed=rtl_failed,
                                      home_reached=home_reached, landed=landed)
        elapsed_ms = (time.perf_counter() - t0) * 1e3
        self._window.append(TickRecord(state.model_copy(), decision, elapsed_ms))
        return decision

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
            self._fsm_fault = None
            self._last_caps = None
            self._window.clear()
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
                # every tick, forever. The caps close each pass: AltitudeFix sizes
                # vz to reach the band in one lookahead, which can overshoot the
                # climb cap on a deep recovery.
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
        longer reports the target on its action and every non-recovery repair
        it logged is within theta (the FSM's, 2.0 m lateral / 0.5 m vertical
        by default; recoveries are exempt, as in the reference). The first
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
                over = [r for r in att.repairs
                        if not r.recovery and r.magnitude_m is not None
                        and r.magnitude_m > (cfg.theta_vertical_m if r.axis == "vertical"
                                             else cfg.theta_lateral_m) + 1e-9]
                if over:
                    w = max(over, key=lambda r: r.magnitude_m)
                    atts.append(replace(att, result="fail", reason=(
                        f"{w.operator} {w.magnitude_m:.2f} m over theta ({w.axis}): "
                        f"{att.reason}")))
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

    def _decide(self, state: State, raw: Action4D) -> ShieldDecision:
        tick = self._tick_ctx()
        if tick is None:                        # called outside filter()
            with self._scope(self._new_tick(None)):
                return self._decide(state, raw)
        rt = tick.rt
        flags = {"path_repair": self.path_repair, "segment_check": self.segment_check}
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

        if not violations:
            # Nothing was wrong with it, so the re-check is empty by
            # construction and does not need running again.
            return self._traced(ShieldDecision(raw=raw, emitted=raw),   # passthrough
                                state, tick, [], [], [], mode, flags)
        edge = self._window_edge(tick, violations)

        enforced = [v for v in violations if self._act(rt, v.rule_id) != "monitor_only"]
        if not enforced:
            # monitor_only rules only: recorded, and the command flies as given.
            # Its re-check is the check above, so a P0 monitor_only rule shows
            # up in the escape count, which is the honest place for it.
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

    def _escalate(self, tick: _Tick, state: State, d: ShieldDecision, *,
                  rtl_failed: bool = False, home_reached: bool = False,
                  landed: bool = False) -> ShieldDecision:
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
            return d.model_copy(update=upd)
        if self._fsm_fault is not None:
            upd.update(fsm_fault=self._fsm_fault, setpoint="none", command=None,
                       fsm_state_before=self.fsm.state.value,
                       fsm_state_after=self.fsm.state.value)
            return d.model_copy(update=upd)
        try:
            self._stop_illegal(tick, state)          # fills tick.unsafe
            caps = self._applied_caps(tick, d)
            inp = _fsm.tick_input_from_decision(
                tick.t, d, tick.rt, horizon_s=self.theta_horizon_s,
                stop_illegal=list(tick.unsafe), rtl_failed=bool(rtl_failed),
                home_reached=bool(home_reached), landed=bool(landed),
                caps=caps if caps is not None else False,
                unsafe_depth_m=self._unsafe_depth_in(state, list(tick.unsafe)))
            out = self.fsm.step(inp)
        except ValueError as e:
            self._fsm_fault = str(e)
            upd.update(fsm_fault=self._fsm_fault, set_mode="LOITER", setpoint="none",
                       command=None, fsm_state_before=self.fsm.state.value,
                       fsm_state_after=self.fsm.state.value)
            return d.model_copy(update=upd)
        cmd = (d.emitted if out.setpoint == "pass"
               else BRAKE if out.setpoint == "brake" else None)
        upd.update(fsm_state_before=out.before.value, fsm_state_after=out.state.value,
                   fsm_edge=out.edge, set_mode=out.set_mode, setpoint=out.setpoint,
                   command=cmd, fsm_record=out.record)
        return d.model_copy(update=upd)
