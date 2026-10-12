"""The Shield's escalation state machine: Normal -> Brake -> Loiter -> RTL -> Land.

WHY THIS FILE EXISTS

The grant's Safety Shield has two halves: repair a bad action when the repair
is small and trustworthy, and step up through fail-safes when it is not
(Safety Shield PDF p1: "detect violations -> repair via projection -> trigger
fail-safe (RTL / Land) when repair is unsafe"). Until this module, only the
first half existed. `guardrail/shield.py` had one fallback, a zero-velocity
BRAKE, and nothing ever changed the ArduPilot flight mode. `brakes = 0` on all
five KPI runs, so the "fail-safe trigger correctness" figure in
`guardrail/kpi.py` was the escape rate under another name (audit card WP3-15).

This module is the missing half, as a PURE state machine: no ROS, no MAVLink,
no geometry, no clock. Each monitor tick it takes what the Shield did and what
the autopilot reports, and returns the FSM state, the mode to request, the
setpoint to stream, a reason, and an audit record. That keeps every transition
unit-testable on any machine. Since 2026-10-07 `guardrail/shield.py` owns one
per Shield and feeds it on every filter() tick (ShieldDecision.fsm_state_after,
.set_mode, .command; docs/DESIGN-shield-enforcement.md). The rails' own wiring
is specified call site by call site in `docs/DESIGN-escalation-fsm.md`.

WHAT IS COPIED FROM THE GRANT, EXACTLY

Safety Shield PDF p4, "Escalation FSM" (the mermaid source is
kuanting-vla-uav-guardrail/docs/02-implementation/safety-shield.md):

    Normal --> Brake   violation, repairable, magnitude < theta        [G1]
    Brake  --> Loiter  N-in-T threshold                                [G2]
    Brake  --> Loiter  magnitude > theta (or no operator converged,    [G3]
                       "Conservative cap", p4)
    Loiter --> RTL     N-in-T persists                                 [G4]
    Loiter --> RTL     operator-defined timeout                        [G5]
    RTL    --> Land    RTL fails (battery / blocked path)              [G6]
    Brake  --> Normal  cleared for >= T_recover                        [G7]
    Loiter --> Normal  cleared for >= T_recover                        [G8]
    RTL    --> [*]     home reached                                    [G9]
    Land   --> [*]     landed                                          [G10]
    Normal --> RTL     violation_action == RTL (direct), unrepairable  [G11]
    Normal --> Land    violation_action == Land (direct), unrepairable [G12]

Defaults (p4): N=3, T=5 s, theta = 2.0 m lateral / 0.5 m vertical,
T_recover = 2 s. "All five are config-file overridable per mission profile",
and the Outputs table (p7) asks for a "Python state machine; YAML-overridable
thresholds". `FSMConfig.from_yaml` does that.

The mode each state asks for is p5's "Mode escalation: SET_MODE GUIDED /
LOITER / RTL / LAND". Normal and Brake are both GUIDED: Brake is not an
ArduPilot mode, it is the Shield intervening inside GUIDED.

WHERE THE GRANT IS SILENT, AND WHAT WAS CHOSEN

Each of these is marked `extension` in the audit record and in the transition
table of the design doc, so nobody mistakes a choice of ours for a contract
term.

  X1  Normal -> Brake on an UNTRUSTED repair (no operator converged, or the
      repair is bigger than theta), or on a rule whose action is `brake`.
      The FSM diagram gives Normal no edge for this case. The node diagram
      (p2) routes "Converged? magnitude < theta? -> no" into the Escalation
      FSM, whose first state is Brake. Staying in Normal with a repair we do
      not trust would be the one wrong answer. The setpoint on that tick is a
      full stop, because p4 says the Shield "abandons projection".
  X2  Brake / Loiter -> RTL, and X3 Brake / Loiter -> Land, when a violated
      rule's action is RTL / land. The diagram draws these only from Normal,
      but it cannot mean that a P0 rule demanding RTL is ignored because a
      P2 rule happened to put the FSM in Brake one tick earlier.
  X4  Normal / Brake -> Loiter when a violated rule's action is `loiter`.
      The DSL allows `loiter` (Policy DSL PDF p2) and the FSM diagram never
      says what it does. It goes straight to the state it names, as RTL and
      land do.
  X5  RTL -> [*] when the autopilot reports `landed` without a separate
      `home_reached`. ArduPilot's RTL ends by landing at home, so a landed
      vehicle has finished its RTL, and an FSM stuck in RTL after touchdown
      would be wrong.
  X6  Brake / Loiter -> RTL when the POSITION has been illegal (`stop_illegal`:
      standing still here breaks a hard rule) on every tick for >= T, a
      hard rule is still violated on this tick, and the illegal depth
      (`unsafe_depth_m`) has not been shown to fall at x6_min_escape_mps
      over that T (a least-squares slope over the window, at two standard
      errors; since 2026-10-08 a recovery that is working is left to
      finish, and a window with a tick of no depth counts as no progress).
      Without it, an aircraft held
      inside a P0 zone (wind beyond the speed cap) never escalates: every tick
      gets a recovery operator, recoveries are exempt from theta, a held
      violation is one N-in-T onset, and the FSM sits in Brake forever. That
      breaks p1, "trigger fail-safe (RTL / Land) when repair is unsafe". It
      goes to RTL and skips Loiter, because loitering where a stop is illegal
      holds the aircraft in the violation (the reference's "brake while
      inside = deadlock"). It was NOT done by counting such ticks as N-in-T
      events: at 10 Hz that reaches RTL 0.6 s after any incursion and would
      abort every GeofenceEscape that was about to succeed. T is the grant's
      own persistence window, so a recovery gets the same 5 s the grant gives
      N-in-T.
  X7  Brake / Loiter -> RTL when the position has been illegal for >=
      T_recover, a hard rule is violated on this tick, and the illegal depth
      is GROWING over the last T_recover: a least-squares slope above
      x6_min_escape_mps at two standard errors, and a fitted rise of at least
      x7_min_rise_m. A recovery is losing ground to something the Shield's
      forecast cannot see (wind beyond the speed cap). Ours, since
      2026-10-08; a PI question.

Interpretations of terms the grant uses but does not define:

  * N-in-T counts violation ONSETS by default (a hard-rule violation on a
    tick whose previous tick had none, at most one per onset_clear_s; see
    ONSET_CLEAR_S), not violated ticks. At the grant's
    10 Hz monitor rate, counting ticks means any violation lasting 0.3 s
    escalates. Every sustained repair, such as sliding along a fence, would
    then put the aircraft in LOITER, and projection would be pointless. N=3
    in T=5 s reads naturally as "three separate violations in five seconds",
    the thrash pattern of a pilot that keeps fighting the Shield. The grant
    itself lists the N-in-T defaults as an open question to "adjust against
    stress-run data" (p7). `event_mode="tick"` is kept so that data can
    decide.
  * "N-in-T persists" (Loiter -> RTL) means N new onsets within T AFTER
    entering Loiter. The window is cleared on entry. Without that, the onsets
    that caused the Loiter would fire the RTL on the very next tick.
  * "Cleared for >= T_recover" is measured from the first clean tick, not
    from the last violated one. That is the later of the two, so recovery is
    never early. A Loiter always holds for at least T_recover (see _enter
    for why that needs no extra code).
  * Window edges are INCLUSIVE: an onset exactly T seconds old still counts,
    and exactly T_recover of clearing is enough (p4 writes ">= T_recover").
    Floating-point slack is 1e-9 s.
  * Theta is a cap on the size of a POSITION correction, in metres, checked
    lateral and vertical separately. A repair of exactly theta is accepted:
    p3 says the first operator with "magnitude <= threshold wins", and p4
    escalates only when the change is "> threshold". The diagram's
    "magnitude < theta" is the one place that says otherwise.
    Where the metres come from is an OPEN QUESTION for the PI, and the
    default stays where the grant's own example points until he answers.
    The default (horizon_s=None) reads each operator's own `magnitude_m`
    (`magnitude_from_repairs`): the depth of the forecast it corrected. The
    grant's audit example logs LateralProjection with "magnitude_m": 0.7
    for a raw (5.0, 1.2) emitted as (4.7, -0.3) (Safety Shield p5), a
    velocity change of 1.53 m/s, so its metres are not |dv| over one
    period (0.15 m); p3 has LateralProjection "project the predicted point
    to the nearest feasible point", a position depth; and the PI reference
    (repair.py) escalates a deep head-on dive "rather than projected".
    The alternative, THETA_HORIZON_S (|dv| x one 0.1 s control period,
    `magnitude_from_actions`), reads p4's "||action_repaired -
    action_original||" literally for a 10 Hz setpoint. It is selectable
    and NOT the default: under every shipped envelope (4-5 m/s, 2 m/s
    climb) a clamped command changes by at most 1.0 m / 0.4 m per period,
    so the cap could never fire, which would waive it. The design doc
    ("Theta: two readings, one default") has both sides and the
    measurements.
  * Theta does not govern every operator. Kinematic clamps change a speed,
    not a position. RECOVERY operators are exempt, as in the PI's reference
    (packages/safety-shield/.../repair.py: "Recovery operators (e.g.
    GeofenceEscape for an inside vehicle) are exempt from the magnitude
    cap"). An aircraft already inside a zone needs a correction as large as
    its penetration to get out, so capping it would stop or loiter the
    aircraft INSIDE the zone. Both magnitude sources apply the same
    exemptions (`theta_governs`).
  * The velocity proxy measures the repaired action against the raw action
    AFTER the kinematic clamps (`KinematicCaps`), because shield.py runs the
    clamps first and the position operators on their output. Measured
    against the unclamped raw action, the proxy counted the clamp's delta-v
    as metres of position correction: on the delivered flights every
    theta-governed tick also carried a SpeedClamp, and the one escalation
    at h = 0.1 s was a ClimbClamp. A tick with a clamp and no caps is
    refused, not measured the old way.
  * ClearanceEscape is a RESCUE, not a recovery by construction: shield.py
    runs it when "repair not converged", and that can happen where a stop
    is legal. There it is what p4 calls "no operator converges", so the
    tick is untrusted (`blocked`), the FSM streams the legal stop, and it
    escalates. Only where a stop is illegal is it exempt like
    GeofenceEscape. The reference exempts only GeofenceEscape, which runs
    only for an aircraft already inside, and guardrail/kpi.py already
    counts a rescue apart (`converged_via_rescue`) for this reason.
  * `stop_illegal` means standing still here breaks a HARD, enforced rule
    (Shield.state_is_unsafe, filtered by `tick_input_from_decision`). A soft
    rule is capped at brake, so a stop that breaks only a soft rule is a
    response the soft rule already allows. With `stop_illegal` the FSM never
    streams a stop: the setpoint is the Shield's own action instead of a
    zero, and the state still escalates. The reference calls stopping inside
    a zone the "brake while inside = deadlock" failure.
  * G11 / G12 say "violation_action == RTL (direct), unrepairable". The
    module reads "unrepairable" as "a direct-action rule skips projection",
    as the reference's shield.py does ("Rules whose action is a direct
    fail-safe skip projection entirely"). An RTL rule fires G11 even on a
    tick whose repair would have converged.
  * The operator-defined Loiter timeout has no grant default. 30 s is ours.
    `None` disables it.

The PI's reference ships a Phase-1 slice of this FSM (safety_shield/fsm.py:
Normal, Brake, RTL, Land, with no Loiter, no N-in-T and no T_recover). Where
this module differs from that slice, and why, is listed in the design doc.

HARD / SOFT AND PRIORITY

The grant defines both fields (Policy DSL PDF p2) and orders severity as
"hard >> soft, P0 > P1 > P2" (Prefix Compiler PDF p4). It never says what
the Shield does with them. This module's answer:

  * A SOFT rule is still enforced, but its response is capped at `brake`. A
    soft rule never changes the flight mode, never counts toward N-in-T, and
    never holds the FSM in Loiter. `brake` is the strongest action the grant
    itself pairs with a soft rule (envelope-default in the worked example,
    Policy DSL PDF p4). Loiter, RTL and Land abort the mission, and a
    constraint the mission may bend is not a reason to abort it. A soft rule
    written with `RTL` is capped and flagged (`capped: true`), not silently
    obeyed or silently ignored.
  * PRIORITY never makes the response LIGHTER. The response is the strongest
    action among the enforced violations. Otherwise adding a P0 violation
    to a P2 `land` could turn a landing into a repair. Priority orders the
    record and breaks ties: when two rules ask for the same action, the P0
    rule is the one cited, whatever order the Shield listed them in. It is
    copied to `risk_level`, the grant's auto-label (Stress Testing PDF p5).
    Giving up lower-priority rules first when the repair stack cannot
    satisfy all of them is a repair-chain decision, and it stays in
    shield.py (card WP1-12).
  * `monitor_only` violations are recorded and do nothing else.
  * `repair` (this repo's legacy spelling) is read as the grant's
    `project_fix`. Action names are matched case-insensitively because the
    grant spells the same action `land` (DSL literal, Policy DSL p2) and
    `Land` (FSM diagram, Safety Shield p4).

THE KPI THIS MAKES MEASURABLE

`score_failsafe_triggers` computes the grant's fail-safe trigger correctness
(Stress Testing PDF p6: ">= 99 %, triggered when expected, not when not
expected") from a per-episode `expected_failsafe` label, as
(correct triggers + correct non-triggers) / labelled episodes. It reports the
two zero-skill baselines beside it: a Shield that always triggers and one that
never does. An episode set where every label is the same cannot tell a
working fail-safe from a disabled (or a trigger-happy) one. The correctness
figure is still printed, because it is what was measured, but `meets_target`
and `meets_target_at_95` are None, not True, and a warning names the stub
that would tie. A dashboard that reads `meets_target` sees no pass.
"""
from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

# Floating-point slack for the inclusive time and theta comparisons. Ticks are
# timestamps accumulated in 0.1 s steps, and fifty additions of 0.1 give
# 4.999999999999998, not 5.0. An edge test that depends on that last bit is a
# test of IEEE 754, not of the FSM.
EPS = 1e-9

# The grant's control period. "Monitor rate: 10 Hz, matching the VLA action
# rate" (Safety Shield PDF p1) and "It emits a 10 Hz setpoint that
# ArduPilot's inner loop tracks" (Architecture constraints PDF p1): each
# action, raw or repaired, is in force for one 0.1 s period, then the next
# tick replaces it.
CONTROL_PERIOD_S = 0.1
# Theta's ONE-PERIOD reading (docs/DESIGN-escalation-fsm.md, "Theta: two
# readings, one default"): |dv| x CONTROL_PERIOD_S, p4's "||action_repaired -
# action_original||" read literally for a setpoint flown for one 0.1 s
# period. SELECTABLE, NOT THE DEFAULT. The default is the per-operator
# `magnitude_m` (horizon_s=None, DEFAULT_THETA_HORIZON_S), the reading the
# grant's audit example (p5: magnitude_m 0.7 for a 1.53 m/s change) and the
# PI reference support, until the PI says otherwise. Under the 4-5 m/s caps
# of every shipped policy this reading caps nothing (at most 1.0 m per
# period against theta 2.0 m).
THETA_HORIZON_S = CONTROL_PERIOD_S
# The basis every Shield and rail uses unless told otherwise: None, each
# repair's own magnitude_m. Changing it is a PI decision, recorded in the
# design doc when it is made.
DEFAULT_THETA_HORIZON_S: float | None = None
# X6's progress test: a position that is illegal for T but whose illegal depth
# falls at least this fast (shown at two standard errors of a least-squares
# fit over T) is a recovery that is working, and X6 leaves it alone. The same
# 0.1 m/s the Shield's trend tests and the PI's reference (checker.py
# `_ESCAPE_SPEED_MPS`) use to tell escaping from drift. X7 uses it the other
# way round: a depth growing faster than this, at two standard errors.
X6_MIN_ESCAPE_MPS = 0.1
# X7's floor: the fitted rise of the illegal depth over T_recover must be at
# least this many metres before X7 calls a recovery "losing ground". Two
# depth samples T_recover apart with 0.2-0.3 m of position noise sent working
# 0.3-0.5 m/s recoveries home (review of 2026-10-09: 40-176 of 200 seeded
# runs); a fitted slope with its standard error and this floor do not.
X7_MIN_RISE_M = 0.5
# N-in-T's onset rate limit: a hard violation that starts again less than
# this long after the last COUNTED onset is the same violation flickering at
# the 10 Hz monitor, not a new one. 0 counts every return, as before
# 2026-10-08, when GPS noise of 0.2-0.3 m made a forecast skimming a zone's
# ring violate at 6.1, 6.3 and 6.8 s - three "onsets" in 0.7 s, N-in-T,
# LOITER, then RTL - on a pass the scenario labels within the error budget.
# It is a RATE LIMIT, not a merge: a pilot who keeps re-violating still
# produces one onset per ONSET_CLEAR_S, so N-in-T (3 in 5 s) still catches the
# thrash it exists for. (A first version merged every return after a clean
# spell under 1 s into one onset, and one-tick violations every 0.3-0.9 s for
# 30 s then never left Brake.) It must leave room for N onsets in T,
# (N - 1) x onset_clear_s <= T, or N-in-T could never fire; FSMConfig refuses
# a config that breaks that. Pending the PI (design doc, "Questions for the
# PI").
ONSET_CLEAR_S = 1.0


class FSMState(str, Enum):
    """The grant's five states (Safety Shield PDF p4), spelled as it spells them.

    A str-Enum, not 3.11's StrEnum, because the package must import on 3.10
    (vla-real) as well as 3.11 (vla-drone). Always serialise with `.value`.
    """
    NORMAL = "Normal"
    BRAKE = "Brake"
    LOITER = "Loiter"
    RTL = "RTL"
    LAND = "Land"


# Escalation order, used for `max_state` and nothing that decides a transition.
_STATE_ORDER = (FSMState.NORMAL, FSMState.BRAKE, FSMState.LOITER,
                FSMState.RTL, FSMState.LAND)

# The grant's outcome vocabulary has RTL_triggered and Land_triggered and no
# Loiter_triggered (Stress Testing PDF p5), so a fail-safe "trigger" is entry
# into one of these two. Loiter is an escalation step, reported separately.
FAILSAFE_STATES = (FSMState.RTL, FSMState.LAND)

# SET_MODE per state (Safety Shield PDF p5). Normal and Brake both fly GUIDED.
MODE_FOR = {
    FSMState.NORMAL: "GUIDED",
    FSMState.BRAKE: "GUIDED",
    FSMState.LOITER: "LOITER",
    FSMState.RTL: "RTL",
    FSMState.LAND: "LAND",
}
_GUIDED_STATES = (FSMState.NORMAL, FSMState.BRAKE)

# The grant's six breach actions (Policy DSL PDF p2), weakest first. The order
# IS the escalation order: `brake` sits below `loiter` because it keeps the
# aircraft in GUIDED under the Shield's control; the three after it hand the
# aircraft to the autopilot.
ACTIONS = ("monitor_only", "project_fix", "brake", "loiter", "RTL", "land")
_STRENGTH = {a: i for i, a in enumerate(ACTIONS)}
_ACTION_BY_LOWER = {a.lower(): a for a in ACTIONS}
# This repo's models.py has spelled project_fix as `repair` since July.
ACTION_ALIASES = {"repair": "project_fix"}
# The strongest action a soft rule may cause. See the module docstring.
SOFT_ACTION_CAP = "brake"

CONSTRAINT_TYPES = ("hard", "soft")
PRIORITIES = ("P0", "P1", "P2")
OUTCOMES = ("clean", "repaired", "blocked")
SETPOINTS = ("pass", "brake", "none")
EVENT_MODES = ("onset", "tick")

# Every edge this FSM can take: id -> (from, to, source, label). The design
# doc's transition table lists the same ids, and tests/test_fsm.py drives
# every id at least once.
EDGES: dict[str, tuple[str, str, str, str]] = {
    "G1": ("Normal", "Brake", "grant", "violation, repairable, magnitude <= theta"),
    "G2": ("Brake", "Loiter", "grant", "N-in-T threshold"),
    "G3": ("Brake", "Loiter", "grant",
           "magnitude > theta or no operator converged (conservative cap)"),
    "G4": ("Loiter", "RTL", "grant", "N-in-T persists (N new onsets since entry)"),
    "G5": ("Loiter", "RTL", "grant", "operator-defined timeout"),
    "G6": ("RTL", "Land", "grant", "RTL fails (battery / blocked path)"),
    "G7": ("Brake", "Normal", "grant", "cleared for >= T_recover"),
    "G8": ("Loiter", "Normal", "grant", "cleared (hard rules) for >= T_recover"),
    "G9": ("RTL", "[*]", "grant", "home reached"),
    "G10": ("Land", "[*]", "grant", "landed"),
    "G11": ("Normal", "RTL", "grant", "violation_action == RTL (direct)"),
    "G12": ("Normal", "Land", "grant", "violation_action == land (direct)"),
    "X1": ("Normal", "Brake", "extension",
           "untrusted repair (blocked or > theta) or a `brake` rule"),
    "X2": ("Brake|Loiter", "RTL", "extension", "violation_action == RTL (direct)"),
    "X3": ("Brake|Loiter", "Land", "extension", "violation_action == land (direct)"),
    "X4": ("Normal|Brake", "Loiter", "extension", "violation_action == loiter (direct)"),
    "X5": ("RTL", "[*]", "extension", "landed during RTL without home_reached"),
    "X6": ("Brake|Loiter", "RTL", "extension",
           "position illegal (stop_illegal) on every tick for >= T, hard rule violated, "
           "illegal depth not shown to be falling"),
    "X7": ("Brake|Loiter", "RTL", "extension",
           "position illegal for >= T_recover and its illegal depth GROWING (fitted slope "
           "at two standard errors, and a minimum rise) while the Shield intervenes on a "
           "hard rule (a recovery losing ground)"),
}


def canonical_action(action: str) -> str:
    """The grant's spelling of a breach action, or ValueError.

    Unknown names are REFUSED, not defaulted. A typo such as `rtl_now` that
    quietly became `project_fix` would turn a rule meant to send the aircraft
    home into one that only nudges it.
    """
    if not isinstance(action, str):
        raise ValueError(f"violation_action must be a string, got {action!r}")
    a = ACTION_ALIASES.get(action.strip().lower(), action.strip().lower())
    if a not in _ACTION_BY_LOWER:
        raise ValueError(f"unknown violation_action {action!r}; "
                         f"the grant's six are {ACTIONS} (+ alias 'repair')")
    return _ACTION_BY_LOWER[a]


def action_strength(action: str) -> int:
    return _STRENGTH[canonical_action(action)]


@dataclass(frozen=True)
class RuleHit:
    """One rule the raw action violated this tick, with what the policy says
    to do about it.

    `constraint_type` and `priority` have no defaults ON PURPOSE. models.py
    defaults them to hard/P0, which is the right default for a policy file,
    but a hit that reaches the FSM without them has lost information on the
    way. Resolve hits through `rules_from_policy`, which applies the policy's
    own defaults and marks ids it could not find.
    """
    rule_id: str
    violation_action: str
    constraint_type: str
    priority: str
    # False when the id was not in the policy (e.g. the Shield's own
    # `action-finite` contract check) and conservative defaults were used.
    resolved: bool = True
    # True for a kinematic envelope (speed / climb / yaw caps). Its repairs are
    # clamps: they always converge and theta never judges them, so such a rule
    # can neither make a tick untrusted nor be the rule a theta escalation is
    # attributed to. Without this, a speed-cap rule that happened to sort first
    # was cited as the reason a fence repair broke theta.
    kinematic: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.rule_id, str) or not self.rule_id:
            raise ValueError(f"rule_id must be a non-empty string, got {self.rule_id!r}")
        for flag in ("resolved", "kinematic"):
            if not isinstance(getattr(self, flag), bool):
                raise ValueError(f"{self.rule_id}: {flag} must be a bool, "
                                 f"got {getattr(self, flag)!r}")
        object.__setattr__(self, "violation_action",
                           canonical_action(self.violation_action))
        if self.constraint_type not in CONSTRAINT_TYPES:
            raise ValueError(f"{self.rule_id}: constraint_type {self.constraint_type!r} "
                             f"is not one of {CONSTRAINT_TYPES}")
        if self.priority not in PRIORITIES:
            raise ValueError(f"{self.rule_id}: priority {self.priority!r} "
                             f"is not one of {PRIORITIES}")

    @property
    def hard(self) -> bool:
        return self.constraint_type == "hard"

    @property
    def capped(self) -> bool:
        """A soft rule that asked for more than a soft rule may cause."""
        return (not self.hard
                and _STRENGTH[self.violation_action] > _STRENGTH[SOFT_ACTION_CAP])

    @property
    def effective_action(self) -> str:
        """What the Shield actually does about this rule.

        This is the ONE place the hard/soft rule is applied. shield.py should
        call it to decide whether to enforce a rule at all (monitor_only means
        no repair), so the repair stack and the FSM cannot disagree.
        """
        return SOFT_ACTION_CAP if self.capped else self.violation_action

    def to_dict(self) -> dict[str, Any]:
        return {"rule_id": self.rule_id, "constraint_type": self.constraint_type,
                "priority": self.priority, "violation_action": self.violation_action,
                "effective_action": self.effective_action, "capped": self.capped,
                "resolved": self.resolved, "kinematic": self.kinematic}


def _severity_key(h: RuleHit):
    """Listing order: hard before soft, P0 before P2, stronger action first.

    "hard >> soft, P0 > P1 > P2" is the grant's own severity order (Prefix
    Compiler PDF p4). rule_id last makes the order total, so the record does
    not depend on the order the Shield happened to list violations in.
    """
    return (0 if h.hard else 1, PRIORITIES.index(h.priority),
            -_STRENGTH[h.effective_action], h.rule_id)


def _governing_key(h: RuleHit):
    """Which rule the response is attributed to: strongest action first, then
    severity. The strongest action decides the response, so priority can only
    break ties and can never make the response lighter."""
    return (-_STRENGTH[h.effective_action],) + _severity_key(h)


@dataclass(frozen=True)
class RepairMagnitude:
    """Size of the Shield's correction, in metres (the grant's unit for theta).

    `recovery_exempt` names the recovery operators that fired this tick and
    were left out of the two numbers (see `theta_governs`). It is kept in the
    record so a large escape that theta did not judge stays visible.
    """
    lateral_m: float
    vertical_m: float
    recovery_exempt: tuple = ()

    def __post_init__(self) -> None:
        for name in ("lateral_m", "vertical_m"):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, (int, float)) \
                    or not math.isfinite(v) or v < 0:
                raise ValueError(f"RepairMagnitude.{name} must be a finite number "
                                 f">= 0, got {v!r}")
        ex = tuple(self.recovery_exempt)
        if not all(isinstance(o, str) and o for o in ex):
            raise ValueError(f"recovery_exempt must be operator names, got {ex!r}")
        object.__setattr__(self, "recovery_exempt", ex)

    def exceeds(self, cfg: "FSMConfig") -> bool:
        """p4: escalate when the change is "> threshold". Exactly theta passes."""
        return (self.lateral_m > cfg.theta_lateral_m + EPS
                or self.vertical_m > cfg.theta_vertical_m + EPS)

    def to_dict(self) -> dict[str, Any]:
        return {"lateral_m": round(self.lateral_m, 6),
                "vertical_m": round(self.vertical_m, 6),
                "recovery_exempt": list(self.recovery_exempt)}


def _channel(a: Any, *names: str) -> float:
    for n in names:
        v = a.get(n) if isinstance(a, Mapping) else getattr(a, n, None)
        if v is not None:
            v = float(v)
            if not math.isfinite(v):
                raise ValueError(f"action channel {n} is not finite: {v!r}")
            return v
    raise ValueError(f"action has none of the channels {names}: {a!r}")


def _num_ok(v: Any) -> bool:
    return (not isinstance(v, bool) and isinstance(v, (int, float))
            and math.isfinite(v))


def _check_horizon(horizon_s: Any) -> None:
    if not _num_ok(horizon_s) or horizon_s <= 0:
        raise ValueError(f"horizon_s must be a finite number > 0, got {horizon_s!r}")


def _is_kinematic_constraint(c: Any) -> bool:
    return (getattr(c, "type", None) == "kinematic_envelope"
            or (getattr(c, "speed_max_mps", None) is not None
                and getattr(c, "climb_rate_max_mps", None) is not None))


def _watched_only(c: Any) -> bool:
    """Is this rule's breach action `monitor_only` (recorded, never repaired)?"""
    va = getattr(c, "violation_action", None)
    if va is None:
        return False
    try:
        return canonical_action(va) == "monitor_only"
    except ValueError:
        return False


@dataclass(frozen=True)
class KinematicCaps:
    """The policy's horizontal-speed and climb-rate caps, as shield.py applies them.

    shield.py's repair chain runs `_repair_kinematic` FIRST, on the raw
    action, and the position operators work on its output (guardrail/shield.py
    `_decide`: "fixed = self._repair_kinematic(raw, repairs)" before the
    clearance / standoff / corridor / geofence loop). So the position
    correction is measured from the CLAMPED raw action, not from the raw one.
    With several kinematic rules, each clamp scales onto its own cap in turn,
    which leaves the action on the smallest cap: that is what `from_policy`
    keeps.
    """
    speed_max_mps: float
    climb_rate_max_mps: float

    def __post_init__(self) -> None:
        for name in ("speed_max_mps", "climb_rate_max_mps"):
            v = getattr(self, name)
            if not _num_ok(v) or v <= 0:
                raise ValueError(f"KinematicCaps.{name} must be a finite number > 0, "
                                 f"got {v!r}")
            object.__setattr__(self, name, float(v))

    @classmethod
    def from_policy(cls, policy: Any) -> "KinematicCaps | None":
        """The smallest caps among the policy's ENFORCED kinematic envelopes,
        or None. A `monitor_only` envelope is never clamped by shield.py, so
        it is no cap here either (until 2026-10-09 it was, and a raw action
        over it with a position repair on the tick was refused as a decision
        "made under other caps": an FSM fault, LOITER, nothing streamed)."""
        kins = [c for c in (getattr(policy, "constraints", None) or [])
                if _is_kinematic_constraint(c) and not _watched_only(c)]
        return cls.of(kins)

    @classmethod
    def of(cls, rules: Iterable[Any]) -> "KinematicCaps | None":
        """The smallest caps among `rules` (kinematic envelopes), or None."""
        rules = list(rules)
        if not rules:
            return None
        return cls(min(float(c.speed_max_mps) for c in rules),
                   min(float(c.climb_rate_max_mps) for c in rules))

    def clamp(self, vx: float, vy: float, vz: float) -> tuple[float, float, float]:
        """shield.py's SpeedClamp and ClimbClamp, exactly: scale the horizontal
        vector onto the speed cap, clip |vz| to the climb cap (with the Shield's
        1e-9 tolerance, so a float-rounding excess is left alone)."""
        h = math.hypot(vx, vy)
        if h > self.speed_max_mps + 1e-9:
            s = self.speed_max_mps / h
            vx, vy = vx * s, vy * s
        if abs(vz) > self.climb_rate_max_mps + 1e-9:
            vz = math.copysign(self.climb_rate_max_mps, vz)
        return vx, vy, vz

    def exceeded_by(self, vx: float, vy: float, vz: float, tol: float = 1e-6) -> bool:
        return (math.hypot(vx, vy) > self.speed_max_mps + tol
                or abs(vz) > self.climb_rate_max_mps + tol)

    def to_dict(self) -> dict[str, float]:
        return {"speed_max_mps": self.speed_max_mps,
                "climb_rate_max_mps": self.climb_rate_max_mps}


def magnitude_from_actions(raw: Any, emitted: Any, horizon_s: float,
                           caps: KinematicCaps | None = None) -> RepairMagnitude:
    """Turn a velocity repair into the position correction theta is defined on.

    The grant states the cap as "||action_repaired - action_original|| >
    threshold" with theta in metres (Safety Shield PDF p4). Our actions are
    velocities, so the norm of their difference is in m/s and needs a time to
    become metres. This returns the gap between where the raw and the
    repaired action would put the aircraft after `horizon_s` seconds:
    |dv_horizontal| * h and |dv_up| * h.

    With `caps`, the raw action is first clamped the way shield.py clamps it
    (`KinematicCaps.clamp`), so a speed cap is not counted as a position
    correction. Without caps the whole action change is measured, clamp
    included. `repair_magnitude` refuses that on a tick that has a clamp.

    `horizon_s` has NO DEFAULT, and that is deliberate: a caller passes
    THETA_HORIZON_S (one 0.1 s control period, the selectable one-period
    reading) or another horizon by name. The choice decides whether the cap
    ever fires (docs/DESIGN-escalation-fsm.md has the tables). The default
    basis everywhere is not this function: it is each operator's own
    penetration depth (`magnitude_from_repairs`), the reading of the grant's
    p5 audit example and of the PI's reference (kuanting-vla-uav-guardrail/
    .../repair.py, LateralProjection). A default here would make that
    decision invisibly.

    Accepts Action4D objects or dicts, with `vz_up` (this repo) or `vz`.
    """
    _check_horizon(horizon_s)
    rx, ry = _channel(raw, "vx"), _channel(raw, "vy")
    rz = _channel(raw, "vz_up", "vz")
    if caps is not None:
        rx, ry, rz = caps.clamp(rx, ry, rz)
    dvx = _channel(emitted, "vx") - rx
    dvy = _channel(emitted, "vy") - ry
    dvz = _channel(emitted, "vz_up", "vz") - rz
    return RepairMagnitude(lateral_m=math.hypot(dvx, dvy) * horizon_s,
                           vertical_m=abs(dvz) * horizon_s)


# Repair operators that legitimately carry no position magnitude. The kinematic
# clamps change a velocity, not a position, and move the action onto the cap,
# the smallest change that satisfies the rule. Theta is metres of position
# correction, and the PI's reference never applies it to a speed cap (its
# repair stack has no kinematic operator). Sanitise is the finiteness
# contract, and Brake is the fallback itself. SoftRelax (shield.py) is the
# record that a soft rule was given up; the position change is in the operators
# beside it. Any OTHER operator without a magnitude is refused, so a new
# operator cannot slip past the cap as a zero.
NO_POSITION_MAGNITUDE = frozenset({"SpeedClamp", "ClimbClamp", "YawClamp",
                                   "Sanitise", "Brake", "SoftRelax"})
# The clamps that change vx / vy / vz, and so the proxy's delta-v. YawClamp
# changes only yaw_rate, which the proxy does not measure.
VELOCITY_CLAMPS = frozenset({"SpeedClamp", "ClimbClamp"})
# Recovery operators: they run only when the aircraft is ALREADY inside a zone
# or ring, and move it out. The PI's reference exempts them from theta
# (repair.py, `recovery` operators: "Capping them would re-introduce the
# 'brake while inside = deadlock' failure"), and shield.py's GeofenceEscape is
# commented as exactly that. Both are recovery by construction:
#   GeofenceEscape   inside a no-fly polygon -> exit (shield.py: "INSIDE a zone")
#   StandoffRecover  inside a subject's standoff ring -> open the range
#                    (shield.py: "ALREADY inside")
# Any other operator can declare itself with `recovery: True` on the Repair
# (CorridorReturn should, on the branch where the aircraft is already off the
# corridor).
RECOVERY_OPERATORS = frozenset({"GeofenceEscape", "StandoffRecover"})
# Rescue operators: shield.py's P0 escape guard runs them when "repair not
# converged". That can happen where a stop is legal (a predicted clearance
# breach, with BRAKE legal), and there it is what p4 calls "no operator
# converges": the tick is untrusted and maps to `blocked`
# (`tick_input_from_decision`). Only where a stop is illegal is a rescue a
# recovery, and exempt like GeofenceEscape.
RESCUE_OPERATORS = frozenset({"ClearanceEscape"})
AXES = ("lateral", "vertical")


def _field(obj: Any, name: str) -> Any:
    return obj.get(name) if isinstance(obj, Mapping) else getattr(obj, name, None)


def _is_recovery(r: Any, stop_illegal: bool = False) -> bool:
    """A named recovery operator, a rescue where a stop is illegal, or a repair
    flagged `recovery: True`.

    The flag can only ADD an operator to the list, never remove one. A
    Pydantic `Repair` with `recovery: bool = False` writes False on every
    repair, and if False overrode the names, every GeofenceEscape would be
    capped again without anyone having asked for it.
    """
    flag = _field(r, "recovery")
    if flag is not None and not isinstance(flag, bool):
        raise ValueError(f"repair operator {_field(r, 'operator')!r}: recovery must be "
                         f"a bool, got {flag!r}")
    op = _field(r, "operator")
    return (bool(flag) or op in RECOVERY_OPERATORS
            or (stop_illegal and op in RESCUE_OPERATORS))


def theta_governs(r: Any, stop_illegal: bool = False) -> bool:
    """Is this repair one the theta cap judges?

    No for kinematic clamps, Sanitise and Brake (no position correction), and
    no for recovery operators (exempt, as in the PI's reference), which
    includes a rescue on a tick where a stop is illegal. Yes for everything
    else, including an operator name this module has never seen: a new
    operator is capped until someone decides otherwise.
    """
    return (_field(r, "operator") not in NO_POSITION_MAGNITUDE
            and not _is_recovery(r, stop_illegal))


def magnitude_from_repairs(repairs: Iterable[Any], stop_illegal: bool = False) -> RepairMagnitude:
    """Theta's input read from per-operator `magnitude_m`, the grant's own field.

    The grant's audit record carries one entry per repair attempt with
    `magnitude_m` (Safety Shield PDF p5), and the PI's reference fills it with
    the position correction: penetration depth for a lateral projection,
    altitude error for an altitude clamp. Each repair theta governs needs
    `magnitude_m` (metres, >= 0) and `axis` ("lateral" | "vertical").

    Magnitudes on one axis are SUMMED over the operators that fired this
    tick. Operators act one after another on each other's output, so the sum
    bounds the total correction from above, and an untrusted repair is never
    reported as smaller than it was. (The reference checks each attempt on
    its own. The grant's cap is on the total change, ||repaired - original||,
    p4, which the sum bounds.)

    Recovery operators are left out of the sum and named in
    `recovery_exempt`. A magnitude they carry is still validated.

    Since 2026-10-07 guardrail/shield.py's `Repair` carries both fields on
    every position operator. A repair that arrives without them (an older
    log, another implementation) still raises here on the first position
    operator, and the caller must then choose the velocity proxy explicitly
    (`horizon_s`).
    """
    lat = vert = 0.0
    exempt: list[str] = []
    for r in repairs or []:
        op = _field(r, "operator")
        m = _field(r, "magnitude_m")
        governed = theta_governs(r, stop_illegal)
        recovery = _is_recovery(r, stop_illegal)
        if m is None:
            if not governed:
                if recovery:
                    exempt.append(str(op))
                continue
            raise ValueError(
                f"repair operator {op!r} reports no magnitude_m, so the theta cap "
                "cannot be checked (grant audit schema, Safety Shield PDF p5). "
                "Add magnitude_m/axis to the Repair, or pass horizon_s to use "
                "the velocity proxy")
        axis = _field(r, "axis")
        if axis not in AXES:
            raise ValueError(f"repair operator {op!r}: axis {axis!r} is not one of {AXES}")
        m = float(m)
        if not math.isfinite(m) or m < 0:
            raise ValueError(f"repair operator {op!r}: magnitude_m {m!r} is not finite >= 0")
        if not governed:
            if recovery:
                exempt.append(str(op))
            continue
        if axis == "lateral":
            lat += m
        else:
            vert += m
    return RepairMagnitude(lateral_m=lat, vertical_m=vert,
                           recovery_exempt=tuple(dict.fromkeys(exempt)))


def repair_magnitude(repairs: Iterable[Any], raw: Any = None, emitted: Any = None,
                     horizon_s: float | None = None, caps: KinematicCaps | None = None,
                     stop_illegal: bool = False) -> RepairMagnitude:
    """Theta's input from whichever source the caller chose. Nothing picks one
    silently.

    horizon_s=None  per-operator `magnitude_m` (`magnitude_from_repairs`).
    horizon_s=h     the velocity proxy |dv| * h (`magnitude_from_actions`),
                    applied only when at least one repair this tick is one
                    theta governs. A tick of clamps or recoveries alone is
                    0 m under both sources. On a tick that also has a
                    SpeedClamp or ClimbClamp, `caps` is REQUIRED, and the
                    proxy measures from the clamped raw action. Without caps
                    it would add the clamp's delta-v to the position
                    correction, which on the delivered flights was most of
                    the number.
    """
    repairs = list(repairs or [])
    if horizon_s is None:
        return magnitude_from_repairs(repairs, stop_illegal)
    _check_horizon(horizon_s)
    exempt = tuple(dict.fromkeys(str(_field(r, "operator")) for r in repairs
                                 if _is_recovery(r, stop_illegal)))
    if not any(theta_governs(r, stop_illegal) for r in repairs):
        return RepairMagnitude(0.0, 0.0, recovery_exempt=exempt)
    clamped = any(_field(r, "operator") in VELOCITY_CLAMPS for r in repairs)
    if clamped and caps is None:
        ops = sorted({str(_field(r, "operator")) for r in repairs})
        raise ValueError(
            f"velocity proxy on a tick with a kinematic clamp ({', '.join(ops)}) needs "
            "the policy's caps (KinematicCaps.from_policy): without them the clamp's "
            "delta-v would be counted as metres of position correction")
    if caps is not None and not clamped:
        rx, ry = _channel(raw, "vx"), _channel(raw, "vy")
        rz = _channel(raw, "vz_up", "vz")
        if caps.exceeded_by(rx, ry, rz):
            # shield.py clamps first, so a raw action over the cap always
            # produces a clamp. One that did not was decided under other caps:
            # the policy given is not the one that produced this decision.
            raise ValueError(
                f"raw action exceeds the caps {caps.to_dict()} but no SpeedClamp / "
                "ClimbClamp fired: these caps are not the ones this decision was "
                "made under (wrong policy for this run?)")
    m = magnitude_from_actions(raw, emitted, horizon_s, caps)
    return RepairMagnitude(m.lateral_m, m.vertical_m, recovery_exempt=exempt)


@dataclass(frozen=True)
class TickInput:
    """Everything the FSM needs from one monitor tick.

    outcome   what the Shield did to the raw action:
                clean    passed untouched (no enforced violation),
                repaired a repair that re-checked clean,
                blocked  no repair converged (the Shield braked, or what it
                         emitted still violates a rule).
    violations every rule the RAW action violated, monitor_only and soft
               included, so the record shows them.
    magnitude  required when outcome == "repaired": the theta cap must be
               checkable. A repair of unknown size is never waved through as
               small.
    rtl_failed / home_reached / landed: what the autopilot side reports.
               The node derives them from MAVROS (see the design doc).
    stop_illegal  standing still HERE breaks a hard, enforced rule
               (Shield.state_is_unsafe, filtered to hard enforced rules by
               `tick_input_from_decision`). The FSM then never streams a
               stop, because a zero setpoint inside a zone or a clearance
               ring holds the aircraft in the violation it is meant to
               leave. Held on every tick for T, it is also what X6 escalates
               on.
    unsafe_depth_m  how far the POSITION is from legal, in metres
               (Shield.unsafe_depth: summed over the hard rules a stop here
               breaks), or None when the caller cannot say. X6 and X7 read
               its fitted trend: a depth shown to fall at >=
               x6_min_escape_mps over T is a recovery that is working (no
               X6); one shown to grow over T_recover is losing ground (X7).
               None is read as no progress, and never fires X7.
    """
    t: float
    outcome: str
    violations: tuple = ()
    magnitude: RepairMagnitude | None = None
    rtl_failed: bool = False
    home_reached: bool = False
    landed: bool = False
    note: str = ""
    stop_illegal: bool = False
    unsafe_depth_m: float | None = None

    def __post_init__(self) -> None:
        if isinstance(self.t, bool) or not isinstance(self.t, (int, float)) \
                or not math.isfinite(self.t):
            raise ValueError(f"t must be a finite number of seconds, got {self.t!r}")
        if self.outcome not in OUTCOMES:
            raise ValueError(f"outcome {self.outcome!r} is not one of {OUTCOMES}")
        vios = tuple(self.violations)
        for h in vios:
            if not isinstance(h, RuleHit):
                raise ValueError(f"violations must be RuleHit, got {type(h).__name__}")
        object.__setattr__(self, "violations", vios)
        for flag in ("rtl_failed", "home_reached", "landed", "stop_illegal"):
            if not isinstance(getattr(self, flag), bool):
                raise ValueError(f"{flag} must be a bool, got {getattr(self, flag)!r}")
        if self.magnitude is not None and not isinstance(self.magnitude, RepairMagnitude):
            raise ValueError("magnitude must be a RepairMagnitude or None")
        if self.unsafe_depth_m is not None:
            d = self.unsafe_depth_m
            if not _num_ok(d) or d < 0:
                raise ValueError(f"unsafe_depth_m must be a finite number >= 0 or None, "
                                 f"got {d!r}")
            object.__setattr__(self, "unsafe_depth_m", float(d))
        enforced = [h for h in vios if h.effective_action != "monitor_only"]
        if self.outcome == "repaired" and self.magnitude is None:
            raise ValueError("outcome 'repaired' without a magnitude: the theta cap "
                             "cannot be checked, and an unchecked repair must not "
                             "pass as a small one")
        if self.outcome == "clean" and enforced:
            raise ValueError(
                "outcome 'clean' but enforced rules were violated "
                f"({', '.join(h.rule_id for h in enforced)}): the Shield passed "
                "an action it was meant to repair")
        if self.outcome != "clean" and not enforced:
            raise ValueError(f"outcome {self.outcome!r} with no enforced violation: "
                             "the Shield changed an action no rule asked it to")


@dataclass(frozen=True)
class FSMConfig:
    """The grant's five thresholds (Safety Shield PDF p4) plus our knobs.

    n_violations, window_s, theta_lateral_m, theta_vertical_m, t_recover_s are
    the grant's N, T, theta (lateral / vertical) and T_recover, with the
    grant's defaults. loiter_timeout_s is the "operator-defined timeout" the
    grant names without a default (30 s is ours; None disables it).
    event_mode is how N-in-T counts (see the module docstring).
    x6_min_escape_mps is X6's progress test (ours): a position illegal for T
    whose illegal depth (TickInput.unsafe_depth_m) is shown to fall at least
    this fast (least-squares slope over T, at two standard errors) is a
    working recovery and does not fire X6. A window with a tick of no depth
    is read as no progress, so a caller that reports none keeps X6 as it was.
    X7 uses the same rate the other way, with x7_min_rise_m as its floor.
    onset_clear_s is N-in-T's onset rate limit (ONSET_CLEAR_S; 0 counts
    every return). The three knobs entered the config on 2026-10-08/09, so
    every fsm_config_hash written before then differs from today's for the
    same grant thresholds.
    """
    n_violations: int = 3
    window_s: float = 5.0
    theta_lateral_m: float = 2.0
    theta_vertical_m: float = 0.5
    t_recover_s: float = 2.0
    loiter_timeout_s: float | None = 30.0
    event_mode: str = "onset"
    x6_min_escape_mps: float = X6_MIN_ESCAPE_MPS
    onset_clear_s: float = ONSET_CLEAR_S
    x7_min_rise_m: float = X7_MIN_RISE_M

    # Grant symbols accepted in YAML, so a profile can be written the way p4
    # writes it ("N: 3, T: 5, T_recover: 2").
    ALIASES = {"N": "n_violations", "T": "window_s", "T_recover": "t_recover_s",
               "theta_lateral": "theta_lateral_m", "theta_vertical": "theta_vertical_m",
               "loiter_timeout": "loiter_timeout_s"}

    def __post_init__(self) -> None:
        def num(name, lo, strict):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) \
                    or (v <= lo if strict else v < lo):
                raise ValueError(f"FSMConfig.{name} must be a finite number "
                                 f"{'>' if strict else '>='} {lo}, got {v!r}")
        if isinstance(self.n_violations, bool) or not isinstance(self.n_violations, int) \
                or self.n_violations < 1:
            raise ValueError(f"FSMConfig.n_violations must be an int >= 1, "
                             f"got {self.n_violations!r}")
        num("window_s", 0, True)
        num("theta_lateral_m", 0, True)
        num("theta_vertical_m", 0, True)
        num("t_recover_s", 0, False)
        num("x6_min_escape_mps", 0, False)
        num("onset_clear_s", 0, False)
        num("x7_min_rise_m", 0, False)
        if self.loiter_timeout_s is not None:
            num("loiter_timeout_s", 0, True)
            # A timeout no longer than T_recover means no Loiter can ever
            # recover before it times out. Every Loiter would become an RTL,
            # and the config would quietly delete the G8 edge.
            if self.loiter_timeout_s <= self.t_recover_s:
                raise ValueError(
                    f"loiter_timeout_s ({self.loiter_timeout_s}) must exceed "
                    f"t_recover_s ({self.t_recover_s}), or Loiter can never recover")
        if self.event_mode not in EVENT_MODES:
            raise ValueError(f"FSMConfig.event_mode {self.event_mode!r} "
                             f"is not one of {EVENT_MODES}")
        # At most one counted onset per onset_clear_s: N of them need
        # (N - 1) x onset_clear_s seconds, which must fit in T, or N-in-T (G2,
        # G4) could never fire and the config would quietly delete both edges.
        if (self.event_mode == "onset"
                and (self.n_violations - 1) * self.onset_clear_s > self.window_s + EPS):
            raise ValueError(
                f"onset_clear_s ({self.onset_clear_s}) x (N - 1) = "
                f"{(self.n_violations - 1) * self.onset_clear_s:g} s exceeds T "
                f"({self.window_s}): N onsets could never fall inside T")
        # Canonical numbers, AFTER validation (so True is still refused). YAML
        # "T: 5" arrives as the int 5, and json.dumps writes 5 and 5.0
        # differently, so one set of thresholds had two config hashes.
        for name in ("window_s", "theta_lateral_m", "theta_vertical_m", "t_recover_s",
                     "x6_min_escape_mps", "onset_clear_s", "x7_min_rise_m"):
            object.__setattr__(self, name, float(getattr(self, name)))
        if self.loiter_timeout_s is not None:
            object.__setattr__(self, "loiter_timeout_s", float(self.loiter_timeout_s))

    @classmethod
    def _canonical_layer(cls, layer: Mapping[str, Any], where: str) -> dict[str, Any]:
        """One layer (the base section or one profile) with grant symbols
        resolved to field names. A key given twice INSIDE one layer (`N` and
        `n_violations`) is refused: nobody can tell which one was meant."""
        if not isinstance(layer, Mapping):
            raise ValueError(f"{where} must be a mapping, got {type(layer).__name__}")
        known = {f for f in cls.__dataclass_fields__ if f != "ALIASES"}
        out: dict[str, Any] = {}
        for k, v in layer.items():
            name = cls.ALIASES.get(k, k)
            if name not in known:
                raise ValueError(f"{where}: unknown escalation key {k!r}; known: "
                                 f"{sorted(known)} or grant symbols {sorted(cls.ALIASES)}")
            if name in out:
                raise ValueError(f"{where}: escalation key {name!r} given twice "
                                 f"(alias {k!r})")
            out[name] = v
        return out

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any], profile: str | None = None) -> "FSMConfig":
        """Build from a dict: flat keys, an `escalation:` section, and optional
        `profiles: {name: {...}}` overlays ("overridable per mission profile").

        Unknown keys are REFUSED. A misspelt `t_recovr: 5` that silently kept
        the 2 s default is exactly the zero that looks like a setting.

        Aliases are resolved in each layer separately, then the profile is
        overlaid on the base. So a base written with the grant's `N: 3` can be
        overridden by a profile that says `n_violations: 2`, which is an
        override, not a duplicate.
        """
        if not isinstance(data, Mapping):
            raise ValueError(f"escalation config must be a mapping, got {type(data).__name__}")
        section = data.get("escalation", data)
        if not isinstance(section, Mapping):
            raise ValueError("`escalation:` must be a mapping")
        out = cls._canonical_layer({k: v for k, v in section.items() if k != "profiles"},
                                   "escalation")
        if profile is not None:
            profiles = section.get("profiles") or {}
            if profile not in profiles:
                raise KeyError(f"no escalation profile {profile!r}; "
                               f"have {sorted(profiles)}")
            out.update(cls._canonical_layer(profiles[profile] or {},
                                            f"profile {profile!r}"))
        return cls(**out)

    @classmethod
    def from_yaml(cls, path: str | Path, profile: str | None = None) -> "FSMConfig":
        import yaml  # local: keeps this module importable without PyYAML
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        return cls.from_mapping(data, profile)

    def to_dict(self) -> dict[str, Any]:
        return {f: getattr(self, f) for f in self.__dataclass_fields__ if f != "ALIASES"}

    @property
    def digest(self) -> str:
        """Hash of the thresholds in force, written into every audit record, so
        a trigger count can be traced to the N / T / theta that produced it.
        All 64 hex digits, as policy_hash now keeps (models.py), and over the
        canonical (float) values, so `T: 5` and `T: 5.0` hash the same.
        Computed once per (frozen) config: the Shield writes it every tick."""
        d = self.__dict__.get("_digest")
        if d is None:
            blob = json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))
            d = "sha256:" + hashlib.sha256(blob.encode("utf-8")).hexdigest()
            object.__setattr__(self, "_digest", d)
        return d


@dataclass(frozen=True)
class FSMOutput:
    """What the integration acts on, every tick.

    set_mode  None, or the ArduPilot mode to request NOW (edge-triggered: only
              on the tick the state changes, never re-sent every tick).
    setpoint  "pass"  stream the Shield's emitted action (raw or repaired),
              "brake" stream a zero-velocity setpoint,
              "none"  stream nothing; the autopilot owns the aircraft.
    """
    state: FSMState
    before: FSMState
    transition: str | None
    edge: str | None
    set_mode: str | None
    setpoint: str
    reason: str
    terminal: bool
    record: dict = field(default_factory=dict)


class EscalationFSM:
    """One episode's escalation state. Pure: output depends only on the inputs.

    Call `step()` once per monitor tick with a `TickInput`, or `tick()` with
    keyword arguments. Call `reset()` between episodes.
    """

    def __init__(self, config: FSMConfig | None = None) -> None:
        self.config = config or FSMConfig()
        self.reset()

    def reset(self) -> None:
        self._state = FSMState.NORMAL
        self._terminal = False
        self._terminal_reason: str | None = None
        self._t_last: float | None = None
        self._entered_at: float | None = None
        self._events: list[float] = []          # hard-rule onset times in the window
        self._prev_hard = False
        self._last_onset: float | None = None   # the last COUNTED onset (rate limit)
        self._clean_any_since: float | None = None   # no enforced violation since
        self._clean_hard_since: float | None = None  # no HARD enforced violation since
        self._stuck_since: float | None = None       # stop_illegal on every tick since
        # (t, unsafe_depth_m or None) on the ticks of the current stuck spell,
        # oldest first, pruned to the last sample at least max(T, T_recover)
        # old: the windows X6's and X7's fitted trends are read from.
        self._depths: list[tuple[float, float | None]] = []
        self._n = 0
        self._transitions: list[dict[str, Any]] = []
        self._visited = [FSMState.NORMAL]

    # ---------------------------------------------------------------- state
    @property
    def state(self) -> FSMState:
        return self._state

    @property
    def terminal(self) -> bool:
        return self._terminal

    @property
    def transitions(self) -> list[dict[str, Any]]:
        return [dict(t) for t in self._transitions]

    def failsafe_triggered(self, states: Sequence[FSMState] = FAILSAFE_STATES) -> bool:
        """Did this episode enter a fail-safe state? Default RTL or Land."""
        return any(s in self._visited for s in states)

    def summary(self) -> dict[str, Any]:
        """Per-episode facts the harness needs, including the KPI's `triggered`."""
        land = FSMState.LAND in self._visited
        rtl = FSMState.RTL in self._visited
        return {
            "ticks": self._n,
            "final_state": self._state.value,
            "terminal": self._terminal,
            "terminal_reason": self._terminal_reason,
            "visited": [s.value for s in _STATE_ORDER if s in self._visited],
            "max_state": max(self._visited, key=_STATE_ORDER.index).value,
            "failsafe_triggered": land or rtl,
            "loiter_entered": FSMState.LOITER in self._visited,
            # The grant's outcome vocabulary (Stress Testing PDF p5).
            "outcome_label": ("Land_triggered" if land else
                              "RTL_triggered" if rtl else None),
            "transitions": self.transitions,
            "fsm_config_hash": self.config.digest,
        }

    # ----------------------------------------------------------------- step
    def tick(self, t: float, outcome: str = "clean", violations: Iterable[RuleHit] = (),
             magnitude: RepairMagnitude | None = None, **flags) -> FSMOutput:
        return self.step(TickInput(t=t, outcome=outcome, violations=tuple(violations),
                                   magnitude=magnitude, **flags))

    def run(self, inputs: Iterable[TickInput]) -> list[FSMOutput]:
        return [self.step(i) for i in inputs]

    def step(self, inp: TickInput) -> FSMOutput:
        if not isinstance(inp, TickInput):
            raise ValueError(f"step() takes a TickInput, got {type(inp).__name__}")
        cfg = self.config
        t = float(inp.t)
        if self._t_last is not None and t < self._t_last - EPS:
            raise ValueError(f"time went backwards: {t} after {self._t_last}. The "
                             "N-in-T window and T_recover are both durations and "
                             "need a monotonic clock")
        self._t_last = t
        self._n += 1
        if self._entered_at is None:
            self._entered_at = t
        before = self._state

        hits = sorted(inp.violations, key=_severity_key)
        enforced = [h for h in hits if h.effective_action != "monitor_only"]
        hard_enf = [h for h in enforced if h.hard]
        violated, hard_violated = bool(enforced), bool(hard_enf)
        gov = min(enforced, key=_governing_key) if enforced else None
        strongest = gov.effective_action if gov else None

        theta_exceeded = (inp.magnitude.exceeds(cfg)
                          if inp.outcome == "repaired" else None)
        # The repair cannot be trusted: nothing converged, or it moved the
        # aircraft further than theta allows (Safety Shield PDF p4).
        untrusted = inp.outcome == "blocked" or bool(theta_exceeded)
        # ...and it was a HARD rule being projected, which is what escalates.
        # Not a kinematic rule: its clamps always converge and theta never
        # judges them, so it cannot be the repair that was not trusted.
        projected = [h for h in enforced
                     if h.effective_action == "project_fix" and not h.kinematic]
        hard_untrusted = untrusted and any(h.hard for h in projected)
        if untrusted and strongest == "project_fix" and projected:
            # Attribute an untrusted projection to a rule whose repair theta
            # judges, not to the speed cap that happened to sort first.
            gov = min(projected, key=_governing_key)

        # X6's clock: the position has been illegal on every tick since.
        if not inp.stop_illegal:
            self._stuck_since = None
            self._depths = []
        else:
            if self._stuck_since is None:
                self._stuck_since = t
            self._depths.append((t, inp.unsafe_depth_m))
            # Keep the last sample at least max(T, T_recover) old (the ones
            # the two trends are read from) and everything after it.
            keep = max(cfg.window_s, cfg.t_recover_s)
            while (len(self._depths) > 1
                   and self._depths[1][0] <= t - keep + EPS):
                self._depths.pop(0)
        stuck_s = None if self._stuck_since is None else t - self._stuck_since
        progress = self._x6_progress(t)
        losing = self._x7_losing(t)

        # Clean streaks. Brake recovers when nothing enforced is violated;
        # Loiter when no HARD rule is, so a soft rule can never hold the
        # aircraft in LOITER until the timeout sends it home.
        if violated:
            self._clean_any_since = None
        elif self._clean_any_since is None:
            self._clean_any_since = t
        if hard_violated:
            self._clean_hard_since = None
        elif self._clean_hard_since is None:
            self._clean_hard_since = t

        # N-in-T bookkeeping: hard-rule onsets only, at most one per
        # onset_clear_s. A violation that starts again sooner than that after
        # the last counted onset is the same violation flickering at the
        # 10 Hz monitor; a pilot who keeps re-violating still produces one
        # onset per onset_clear_s, so N-in-T still sees the thrash.
        rising = hard_violated and not self._prev_hard
        if cfg.event_mode == "tick":
            onset = hard_violated
        else:
            onset = rising and (self._last_onset is None
                                or t - self._last_onset >= cfg.onset_clear_s - EPS)
        merged = rising and not onset
        self._prev_hard = hard_violated
        if onset:
            self._last_onset = t
            self._events.append(t)
        self._events = [e for e in self._events if t - e <= cfg.window_s + EPS]
        n_in_t = len(self._events) >= cfg.n_violations

        cleared_any = (None if self._clean_any_since is None
                       else t - self._clean_any_since)
        cleared_hard = (None if self._clean_hard_since is None
                        else t - self._clean_hard_since)

        target, edge, why = self._decide(inp, t, violated, strongest, gov,
                                         untrusted, hard_untrusted, theta_exceeded,
                                         n_in_t, cleared_any, cleared_hard,
                                         hard_violated, stuck_s, progress, losing)

        terminal_now = False
        if edge in ("G9", "G10", "X5"):
            terminal_now = True
            self._terminal = True
            self._terminal_reason = why
            transition = f"{before.value}->[*]"
            set_mode = None
        elif target is not None and target != before:
            transition = f"{before.value}->{target.value}"
            self._enter(target, t)
            set_mode = (MODE_FOR[target]
                        if MODE_FOR[target] != MODE_FOR[before] else None)
        else:
            transition = None
            set_mode = None

        after = self._state
        # A stop the Shield says is illegal here is never streamed. The
        # Shield's own action is: for a blocked tick that is its BRAKE where a
        # stop is legal and its best-effort recovery where it is not.
        withheld = False
        if self._terminal:
            setpoint = "none"
        elif after in _GUIDED_STATES:
            # Pass the Shield's action unless it is a repair we do not trust or
            # a rule asked for a stop: then hold still (p4, "abandons
            # projection").
            wants_stop = violated and (untrusted or strongest == "brake")
            withheld = wants_stop and inp.stop_illegal
            setpoint = "brake" if wants_stop and not withheld else "pass"
        elif before in _GUIDED_STATES:
            # Handing over to an autopilot mode: stop on this tick so the last
            # GUIDED setpoint is not still flying while SET_MODE goes through.
            withheld = inp.stop_illegal
            setpoint = "pass" if withheld else "brake"
        else:
            setpoint = "none"
        if withheld:
            why += ("; stop withheld: standing still here breaks a rule, so the "
                    "Shield's own action is flown")

        if transition:
            self._transitions.append({"tick": self._n, "t_s": round(t, 6),
                                      "from": before.value,
                                      "to": "[*]" if terminal_now else after.value,
                                      "edge": edge, "reason": why})
        reason = (f"{transition} [{edge}]: {why}" if transition
                  else f"{after.value}: {why}")
        record = {
            "fsm_tick": self._n,
            "t_s": round(t, 6),
            "fsm_state_before": before.value,
            "fsm_state_after": after.value,
            "transition": transition,
            "edge": edge if transition else None,
            "edge_source": EDGES[edge][2] if (transition and edge) else None,
            "terminal": self._terminal,
            "set_mode": set_mode,
            "setpoint": setpoint,
            "reason": reason,
            "outcome": inp.outcome,
            "governing_rule": gov.to_dict() if gov else None,
            "risk_level": gov.priority if gov else None,
            "violations": [h.to_dict() for h in hits],
            "magnitude": inp.magnitude.to_dict() if inp.magnitude else None,
            "theta": {"lateral_m": cfg.theta_lateral_m,
                      "vertical_m": cfg.theta_vertical_m},
            "theta_exceeded": theta_exceeded,
            "n_in_t": len(self._events),
            # A hard violation started this tick but within onset_clear_s of
            # the last counted onset: the same violation, not a new one.
            "onset_merged": merged,
            "n_threshold": cfg.n_violations,
            "window_s": cfg.window_s,
            "cleared_s": None if cleared_any is None else round(cleared_any, 6),
            "cleared_hard_s": None if cleared_hard is None else round(cleared_hard, 6),
            "stop_illegal_s": None if stuck_s is None else round(stuck_s, 6),
            "unsafe_depth_m": (None if inp.unsafe_depth_m is None
                               else round(inp.unsafe_depth_m, 6)),
            "x6_progress": progress,
            "x7_losing": losing,
            "flags": {"rtl_failed": inp.rtl_failed, "home_reached": inp.home_reached,
                      "landed": inp.landed, "stop_illegal": inp.stop_illegal},
            "stop_withheld": withheld,
            "note": inp.note,
            "fsm_config_hash": cfg.digest,
        }
        return FSMOutput(state=after, before=before, transition=transition,
                         edge=edge if transition else None, set_mode=set_mode,
                         setpoint=setpoint, reason=reason, terminal=self._terminal,
                         record=record)

    # ------------------------------------------------------------ internals
    def _fit(self, t: float, window_s: float) -> dict[str, Any] | None:
        """Least-squares line through the illegal-depth samples of the last
        `window_s` (from the last sample at or before t - window_s to now).

        None when there is nothing to fit. `slope_mps` is None when a sample
        in the window has no depth (a caller that did not report one): no
        evidence either way. `se_mps` is the slope's standard error from the
        fit's own residuals (0 with fewer than three samples, where it cannot
        be estimated). Two samples T apart, as the first version read the
        trend, let 0.2-0.3 m of position noise decide X7 on its own."""
        if not self._depths:
            return None
        i0 = 0
        for i, s in enumerate(self._depths):
            if s[0] <= t - window_s + EPS:
                i0 = i
            else:
                break
        pts = self._depths[i0:]
        t0 = pts[0][0]
        span = pts[-1][0] - t0
        out: dict[str, Any] = {"from_t_s": round(t0, 6), "span_s": round(span, 6),
                               "n": len(pts), "slope_mps": None, "se_mps": None}
        if any(d is None for _, d in pts) or len(pts) < 2 or span < EPS:
            return out
        n = len(pts)
        tb = sum(ti for ti, _ in pts) / n
        db = sum(di for _, di in pts) / n
        sxx = sum((ti - tb) ** 2 for ti, _ in pts)
        if sxx < EPS * EPS:
            return out
        b = sum((ti - tb) * (di - db) for ti, di in pts) / sxx
        res = sum((di - db - b * (ti - tb)) ** 2 for ti, di in pts)
        se = math.sqrt(res / (n - 2) / sxx) if n > 2 else 0.0
        out.update({"from_m": round(pts[0][1], 6), "now_m": round(pts[-1][1], 6),
                    "slope_mps": round(b, 6), "se_mps": round(se, 6),
                    "_b": b, "_se": se, "_span": span})
        return out

    def _x6_progress(self, t: float) -> dict[str, Any] | None:
        """Is the illegal position getting better? None until the stuck spell
        is T long. Otherwise the depth's fitted slope over the last T, and
        whether its fall rate, less two standard errors, reaches
        x6_min_escape_mps: a recovery that is working. A window with a tick of
        no depth is `progressing: False`: no evidence of progress is not
        evidence of it."""
        cfg = self.config
        if (self._stuck_since is None or not self._depths
                or t - self._stuck_since < cfg.window_s - EPS):
            return None
        fit = self._fit(t, cfg.window_s)
        span = fit["span_s"]
        need = cfg.x6_min_escape_mps * span
        if fit["slope_mps"] is None:
            return {"from_t_s": fit["from_t_s"], "span_s": span, "n": fit["n"],
                    "fell_m": None, "need_m": round(need, 6), "progressing": False}
        b, se = fit.pop("_b"), fit.pop("_se")
        fall = -b * fit.pop("_span")
        shown = -b - 2.0 * se
        return {**fit, "fell_m": round(fall, 6), "need_m": round(need, 6),
                "progressing": bool(shown >= cfg.x6_min_escape_mps - EPS and fall > EPS)}

    def _x7_losing(self, t: float) -> dict[str, Any] | None:
        """Is the illegal position getting WORSE? None until the stuck spell is
        T_recover long, or when a depth in the window is missing (an unknown
        depth never fires X7; X6 still covers it at T). Otherwise the depth's
        fitted slope over the last T_recover, and whether the aircraft is
        losing ground: the slope less two standard errors above
        x6_min_escape_mps AND a fitted rise of at least x7_min_rise_m. The
        Shield is commanding a recovery and the aircraft goes deeper, pushed
        by something its forecast cannot see (wind beyond the speed cap)."""
        cfg = self.config
        if (self._stuck_since is None or not self._depths
                or t - self._stuck_since < max(cfg.t_recover_s, CONTROL_PERIOD_S) - EPS):
            return None
        fit = self._fit(t, cfg.t_recover_s)
        if fit is None or fit["slope_mps"] is None:
            return None
        b, se = fit.pop("_b"), fit.pop("_se")
        rose = b * fit.pop("_span")
        shown = b - 2.0 * se
        return {**fit, "rose_m": round(rose, 6),
                "need_mps": cfg.x6_min_escape_mps, "need_m": cfg.x7_min_rise_m,
                "losing": bool(shown > cfg.x6_min_escape_mps + EPS
                               and rose >= cfg.x7_min_rise_m - EPS)}

    def _enter(self, target: FSMState, t: float) -> None:
        self._state = target
        self._entered_at = t
        if target not in self._visited:
            self._visited.append(target)
        if target == FSMState.LOITER:
            # "N-in-T persists" has to be NEW evidence: the onsets that caused
            # this Loiter would otherwise fire G4 on the next tick.
            self._events = []
            # The clean clock is NOT restarted here, because it never needs to
            # be. Every way into Loiter is either a violated tick (G3, X4, G2
            # on an onset), where the clock is unset, or the first clean tick
            # after the onset that completed N-in-T (G2), where the clock
            # already reads the entry time. So a Loiter always holds for at
            # least T_recover. An earlier version reset it anyway, and a
            # mutation run showed no test could tell: dead code that looked
            # like a safeguard. The property is now pinned by
            # test_loiter_always_holds_for_at_least_t_recover instead.

    def _decide(self, inp: TickInput, t: float, violated: bool, strongest: str | None,
                gov: RuleHit | None, untrusted: bool, hard_untrusted: bool,
                theta_exceeded: bool | None, n_in_t: bool,
                cleared_any: float | None, cleared_hard: float | None,
                hard_violated: bool = False, stuck_s: float | None = None,
                progress: dict | None = None, losing: dict | None = None):
        """Return (target state or None, edge id or None, reason)."""
        cfg = self.config
        S = FSMState
        if self._terminal:
            return None, None, f"terminal ({self._terminal_reason}); no transitions"
        # X6: somewhere a stop is illegal for a whole T, with the Shield still
        # intervening on a hard rule, and the illegal depth NOT falling. The
        # recovery is not getting the aircraft out, and holding (Loiter) would
        # keep it in the violation. A recovery whose depth falls at
        # x6_min_escape_mps or faster is left to finish: on 7 Oct, X6 sent
        # climbs from under the altitude floor and 2 m/s exits from a grown
        # zone home at T, while they were working (docs/DESIGN-escalation-
        # fsm.md, "X6 waits for a recovery that is working").
        working = bool(progress and progress.get("progressing"))
        stuck = (hard_violated and stuck_s is not None
                 and stuck_s >= cfg.window_s - EPS and not working)
        if stuck:
            fell = progress.get("fell_m") if progress else None
            trend = ("; no depth reported, so no progress can be seen" if fell is None
                     else f"; illegal depth fell {fell:.2f} m (fitted) in "
                          f"{progress['span_s']:.2f} s, not shown above the "
                          f"{progress['need_m']:.2f} m a working recovery makes "
                          f"(slope {progress['slope_mps']:+.2f} +/- "
                          f"{2 * progress['se_mps']:.2f} m/s)")
        stuck_why = (f"position illegal for {stuck_s:.2f} s >= T {cfg.window_s:g} s "
                     f"while the Shield intervenes on a hard rule{trend}; holding "
                     f"here keeps the aircraft in the violation" if stuck else "")
        # X7: the position has been illegal for T_recover and is getting
        # DEEPER while the Shield commands its recovery - something the
        # Shield's forecast cannot see (a wind beyond the speed cap) is beating
        # it. Waiting for X6's full T lets it through the zone: the 5 m/s
        # tailwind of `wind-beyond-authority` crosses the 16 m square in about
        # 3.5 s (5.4-8.9 s, lane 15, measured with X6 / X7 off).
        sinking = bool(hard_violated and losing and losing.get("losing"))
        sinking_why = (f"position illegal for {stuck_s:.2f} s and its illegal depth "
                       f"grew {losing['rose_m']:.2f} m (fitted) in "
                       f"{losing['span_s']:.2f} s (slope {losing['slope_mps']:+.2f} "
                       f"+/- {2 * losing['se_mps']:.2f} m/s) while the Shield "
                       f"recovers: the recovery is losing ground"
                       if sinking else "")

        def cite(h: RuleHit | None) -> str:
            if h is None:
                return "no rule"
            s = f"{h.rule_id} ({h.constraint_type} {h.priority} {h.violation_action}"
            if h.capped:
                s += f", capped at {SOFT_ACTION_CAP}: soft rules do not change mode"
            if not h.resolved:
                s += ", not in policy: hard/P0 assumed"
            return s + ")"

        def mag() -> str:
            m = inp.magnitude
            if m is None:
                return "no operator converged" if inp.outcome == "blocked" else ""
            return (f"lat {m.lateral_m:.2f} m / vert {m.vertical_m:.2f} m vs theta "
                    f"{cfg.theta_lateral_m:.2f} / {cfg.theta_vertical_m:.2f} m")

        state = self._state
        direct = {"land": S.LAND, "RTL": S.RTL, "loiter": S.LOITER}

        if state == S.NORMAL:
            if strongest in direct:
                edge = {"land": "G12", "RTL": "G11", "loiter": "X4"}[strongest]
                return direct[strongest], edge, f"direct {strongest} by {cite(gov)}"
            if violated:
                if (inp.outcome == "repaired" and not theta_exceeded
                        and strongest == "project_fix"):
                    return S.BRAKE, "G1", f"{cite(gov)} repaired, {mag()}"
                what = ("rule asks for brake" if strongest == "brake" and not untrusted
                        else ("repair over theta, " + mag()) if theta_exceeded
                        else "no operator converged")
                return S.BRAKE, "X1", f"{cite(gov)}: {what}; projection abandoned"
            return None, None, "clean"

        if state == S.BRAKE:
            # Strongest response first: land, RTL (direct, then X6), loiter.
            if strongest in ("land", "RTL"):
                edge = {"land": "X3", "RTL": "X2"}[strongest]
                return direct[strongest], edge, f"direct {strongest} by {cite(gov)}"
            if stuck:
                return S.RTL, "X6", stuck_why
            if sinking:
                return S.RTL, "X7", sinking_why
            if strongest == "loiter":
                return S.LOITER, "X4", f"direct loiter by {cite(gov)}"
            if hard_untrusted:
                extra = (f"; also N-in-T {len(self._events)} in {cfg.window_s:g} s"
                         if n_in_t else "")
                what = (("repair over theta, " + mag()) if theta_exceeded
                        else "no operator converged")
                return S.LOITER, "G3", f"{cite(gov)}: {what}{extra}"
            if n_in_t:
                return (S.LOITER, "G2", f"N-in-T: {len(self._events)} onsets in "
                        f"{cfg.window_s:g} s >= N={cfg.n_violations}")
            if not violated and cleared_any is not None \
                    and cleared_any >= cfg.t_recover_s - EPS:
                return S.NORMAL, "G7", (f"cleared {cleared_any:.2f} s >= T_recover "
                                        f"{cfg.t_recover_s:g} s")
            if violated:
                return None, None, (f"intervening on {cite(gov)}"
                                    + (f", {mag()}" if inp.magnitude else ""))
            return None, None, (f"cleared {cleared_any:.2f} s of T_recover "
                                f"{cfg.t_recover_s:g} s")

        if state == S.LOITER:
            if strongest in ("land", "RTL"):
                edge = {"land": "X3", "RTL": "X2"}[strongest]
                return direct[strongest], edge, f"direct {strongest} by {cite(gov)}"
            if n_in_t:
                return (S.RTL, "G4", f"N-in-T persists: {len(self._events)} onsets "
                        f"in {cfg.window_s:g} s since Loiter entry")
            if cleared_hard is not None and cleared_hard >= cfg.t_recover_s - EPS:
                return S.NORMAL, "G8", (f"hard rules cleared {cleared_hard:.2f} s >= "
                                        f"T_recover {cfg.t_recover_s:g} s")
            dwell = t - (self._entered_at if self._entered_at is not None else t)
            if cfg.loiter_timeout_s is not None \
                    and dwell >= cfg.loiter_timeout_s - EPS:
                return S.RTL, "G5", (f"loiter timeout: {dwell:.2f} s >= "
                                     f"{cfg.loiter_timeout_s:g} s")
            if stuck:
                return S.RTL, "X6", stuck_why
            if sinking:
                return S.RTL, "X7", sinking_why
            return None, None, (f"holding {dwell:.2f} s"
                                + (f"; {cite(gov)} still violated" if violated else ""))

        if state == S.RTL:
            if inp.home_reached:
                return None, "G9", "home reached"
            if inp.landed:
                return None, "X5", "landed during RTL (home_reached not reported)"
            if inp.rtl_failed:
                return S.LAND, "G6", "RTL failed" + (f": {inp.note}" if inp.note else "")
            return None, None, "returning to launch"

        if state == S.LAND:
            if inp.landed:
                return None, "G10", "landed"
            return None, None, "landing"

        raise AssertionError(f"unreachable state {state!r}")   # pragma: no cover


# --------------------------------------------------------------------------- #
# Integration helpers (duck-typed: no import of shield.py or models.py)
# --------------------------------------------------------------------------- #

def rules_from_policy(policy: Any, rule_ids: Iterable[str],
                      kinematic_ids: Iterable[str] = ()) -> tuple[RuleHit, ...]:
    """Resolve violated rule ids to RuleHits using the loaded policy.

    Missing fields take models.py's own defaults (hard, P0, repair). An id
    the policy does not contain becomes a hard P0 project_fix marked
    `resolved=False`. guardrail/kpi.py makes the same "unknown means P0"
    choice, and the flag keeps the guess visible in the record. Duplicate ids
    (one rule hit at several predicted poses) collapse to one hit.

    A rule is `kinematic` when the policy says it is a kinematic envelope, or
    when the caller saw it raise a `kinematic` violation (`kinematic_ids`),
    which works without a policy too.
    """
    by_id = {getattr(c, "id", None): c
             for c in (getattr(policy, "constraints", None) or [])}
    kin = set(kinematic_ids)
    out, seen = [], set()
    for rid in rule_ids:
        if rid in seen:
            continue
        seen.add(rid)
        c = by_id.get(rid)
        if c is None:
            out.append(RuleHit(rid, "project_fix", "hard", "P0", resolved=False,
                               kinematic=rid in kin))
        else:
            out.append(RuleHit(rid, getattr(c, "violation_action", "repair") or "repair",
                               getattr(c, "constraint_type", "hard") or "hard",
                               getattr(c, "priority", "P0") or "P0",
                               kinematic=_is_kinematic_constraint(c) or rid in kin))
    return tuple(out)


def _hits_from_violations(policy: Any, violations: Iterable[Any]) -> tuple[RuleHit, ...]:
    """RuleHits for Shield violations (objects or log dicts with rule_id and
    category). A `kinematic` category marks the rule kinematic."""
    vios = list(violations or [])
    ids = [str(_field(v, "rule_id") or "") for v in vios]
    kin = {i for i, v in zip(ids, vios) if _field(v, "category") == "kinematic"}
    return rules_from_policy(policy, list(dict.fromkeys(ids)), kin)


def _hard_enforced(hits: Iterable[RuleHit]) -> bool:
    return any(h.hard and h.effective_action != "monitor_only" for h in hits)


def _resolve_stop_illegal(stop_illegal: Any, policy: Any) -> bool | None:
    """`stop_illegal` as the FSM means it: a stop here breaks a HARD enforced rule.

    Accepts a bool (the caller vouches), None (unknown), or what
    `Shield.state_is_unsafe(state)` returns: the violations standing still
    would cause. Those are resolved through the policy and only hard, enforced
    rules count. A soft rule is capped at brake, so a stop that breaks only
    a soft rule is a response the rule already allows.
    """
    if stop_illegal is None or isinstance(stop_illegal, bool):
        return stop_illegal
    if isinstance(stop_illegal, (str, bytes, Mapping)) or not isinstance(stop_illegal, Iterable):
        raise ValueError("stop_illegal must be a bool, None, or the violations "
                         f"Shield.state_is_unsafe returned; got {stop_illegal!r}")
    return _hard_enforced(_hits_from_violations(policy, stop_illegal))


def _rescued(repairs: Iterable[Any]) -> bool:
    return any(_field(r, "operator") in RESCUE_OPERATORS for r in repairs or [])


def _blocked_stop_illegal(braked: bool, emitted_violations: Any) -> bool:
    """What a blocked ShieldDecision already says about stopping here.

    shield.py brakes only "where standing still is legal". When it did not
    brake and what it flew still re-checks dirty, it flew a best-effort
    recovery because the stop itself breaks a rule. When it braked and the
    brake still re-checks dirty (`emitted_violations` is the check on BRAKE),
    there was no legal option at all, and the stop is illegal too. (This
    reading cannot tell a soft rule from a hard one, so it errs towards
    "illegal": the FSM then withholds a stop rather than stream one into a
    violation.)
    """
    return (not braked) or bool(emitted_violations)


_RESCUE_NOTE = "ClearanceEscape where a stop is legal: read as not converged (p4)"


def _still_enforced(policy: Any, emitted_violations: Any, relaxed: Any) -> list:
    """The emitted action's violations that the Shield was meant to enforce.

    Two kinds are left out, both on purpose: a `monitor_only` rule is flown as
    commanded by definition (shield.py records it and never repairs it), and a
    soft rule the repair chain gave up to satisfy the others is listed in the
    decision's `relaxed`. Counting either as a non-converged repair would read
    a decision the Shield made deliberately as one it failed to make."""
    ev = list(emitted_violations or [])
    if not ev:
        return []
    gone = set(relaxed or [])
    ids = [str(_field(v, "rule_id") or "") for v in ev]
    monitor = {h.rule_id for h in rules_from_policy(policy, list(dict.fromkeys(ids)))
               if h.effective_action == "monitor_only"}
    return [v for v, i in zip(ev, ids) if i not in gone and i not in monitor]


def tick_input_from_decision(t: float, decision: Any, policy: Any,
                             horizon_s: float | None = None,
                             *, rtl_failed: bool = False, home_reached: bool = False,
                             landed: bool = False, note: str = "",
                             stop_illegal: Any = None,
                             caps: "KinematicCaps | None | bool" = None,
                             unsafe_depth_m: float | None = None) -> TickInput:
    """Map one `guardrail.shield.ShieldDecision` onto a TickInput.

    blocked   the Shield braked, or its own re-check of the emitted action is
              not clean (the best-effort ClearanceEscape branch), or it
              rescued a non-converged chain with ClearanceEscape where a stop
              was legal (p4: "no operator converges"). Either way no repair
              converged.
    repaired  it appended repairs and the emitted action re-checks clean.
    clean     it did nothing.

    Theta's magnitude has two possible sources, and nothing picks one
    silently (`repair_magnitude`):
      horizon_s=None   per-operator `magnitude_m` (`magnitude_from_repairs`),
                       each operator's forecast penetration: the DEFAULT
                       (DEFAULT_THETA_HORIZON_S) of the Shield and every rail,
                       the reading of the grant's p5 audit example and of
                       the PI's reference.
      horizon_s=THETA_HORIZON_S  the one-period reading: the setpoint
                       change |dv| over one 0.1 s control period
                       (`magnitude_from_actions`), measured from the CLAMPED
                       raw action. Selectable (`--theta-horizon-s 0.1`),
                       pending the PI; under every shipped envelope it can
                       never exceed theta.
      horizon_s=h      the same |dv| proxy at another horizon (analysis).

    caps: None reads them from the policy (`KinematicCaps.from_policy`); a
    caller that knows which caps its Shield clamped against on this tick (the
    Shield itself: a cap out of its time window or switched off is not one)
    passes them, or False when it clamped against none.

    stop_illegal: pass `shield.state_is_unsafe(state)` from the node (the
    list; it is filtered to hard enforced rules here) or a bool. Left as
    None, it is read from a blocked decision (`_blocked_stop_illegal`) and
    taken as False on a repaired one. A ClearanceEscape rescue that
    re-checked clean is the exception: whether it is a recovery or a
    non-converged repair depends on exactly this, so None is refused there.

    unsafe_depth_m: `shield.unsafe_depth(state)` (metres from legal), for
    X6's progress test. None: X6 sees no progress, as before 2026-10-08.
    """
    vios = list(getattr(decision, "violations", None) or [])
    repairs = list(getattr(decision, "repairs", None) or [])
    braked = bool(getattr(decision, "braked", False))
    hits = _hits_from_violations(policy, vios)
    ev = _still_enforced(policy, getattr(decision, "emitted_violations", None),
                         getattr(decision, "relaxed", None))
    if not any(h.effective_action != "monitor_only" for h in hits):
        outcome = "clean"
    elif braked or ev:
        outcome = "blocked"
    elif repairs:
        outcome = "repaired"
    else:
        outcome = "clean"
    stop = _resolve_stop_illegal(stop_illegal, policy)
    if outcome == "repaired" and _rescued(repairs):
        if stop is None:
            raise ValueError(
                "ClearanceEscape re-checked clean, but whether that is a recovery "
                "(a stop here is illegal: exempt from theta) or a non-converged "
                "repair (a stop is legal: p4 abandons projection) depends on "
                "stop_illegal. Pass stop_illegal=shield.state_is_unsafe(state)")
        if not stop:
            outcome = "blocked"
            note = f"{note}; {_RESCUE_NOTE}" if note else _RESCUE_NOTE
    if stop is None:
        stop = outcome == "blocked" and _blocked_stop_illegal(braked, ev)
    mag = None
    if outcome == "repaired":
        if caps is None and horizon_s is not None:
            caps = KinematicCaps.from_policy(policy)
        elif caps is False:
            caps = None
        elif caps is True or not (caps is None or isinstance(caps, KinematicCaps)):
            raise ValueError(f"caps must be KinematicCaps, None or False, got {caps!r}")
        mag = repair_magnitude(repairs, decision.raw, decision.emitted, horizon_s,
                               caps=caps, stop_illegal=stop)
    return TickInput(t=t, outcome=outcome, violations=hits, magnitude=mag,
                     rtl_failed=rtl_failed, home_reached=home_reached,
                     landed=landed, note=note, stop_illegal=bool(stop),
                     unsafe_depth_m=unsafe_depth_m)


# --------------------------------------------------------------------------- #
# Replaying a delivered flight log (what the FSM WOULD have done)
# --------------------------------------------------------------------------- #

def _row_stop_illegal(row: Mapping[str, Any], policy: Any) -> bool | None:
    """`unsafe_rules` / `unsafe`, as tools/rescore_kpis.py reconstructs them
    (Shield.state_is_unsafe per tick), or None when the log has neither."""
    if row.get("unsafe_rules") is not None:
        ids = list(row["unsafe_rules"])
        return _hard_enforced(rules_from_policy(policy, ids))
    if row.get("unsafe") is not None:
        return bool(row["unsafe"])
    return None


def tick_input_from_row(row: Mapping[str, Any], policy: Any = None,
                        horizon_s: float | None = None,
                        caps: KinematicCaps | None = None) -> TickInput:
    """One `flight_log.jsonl` row, in the shape guardrail/kpi.py reads.

    A row with violations and no repair is a SHIELD-OFF control arm. The FSM
    does not run there, so it is refused rather than read as clean. Without a
    policy, every id is taken as hard / P0 / project_fix and marked
    unresolved (a `kinematic` violation category still marks a rule
    kinematic).

    `stop_illegal` comes from the row's `unsafe_rules` / `unsafe` when it has
    them. Delivered logs do not: there, a ClearanceEscape rescue that
    re-checked clean cannot be shown to be a recovery, so it is read as
    non-converged (`blocked`, with a note), the reading that escalates.
    """
    vios = row.get("violations") or []
    hits = _hits_from_violations(policy, vios)
    t = float(row["t"])
    # X6's progress input, when the rail logged it (rows since 2026-10-08).
    depth = row.get("unsafe_depth_m")
    depth = float(depth) if _num_ok(depth) and depth >= 0 else None
    # Read on clean ticks too, as the live Shield passes it on every tick: a
    # clean tick inside a zone keeps X6's clock running (a pilot already
    # leaving), and only a legal position restarts it.
    stop = _row_stop_illegal(row, policy)
    if not hits:
        return TickInput(t=t, outcome="clean", stop_illegal=bool(stop),
                         unsafe_depth_m=depth)
    if not any(h.effective_action != "monitor_only" for h in hits):
        return TickInput(t=t, outcome="clean", violations=hits,
                         stop_illegal=bool(stop), unsafe_depth_m=depth)
    braked = bool(row.get("braked"))
    ev = _still_enforced(policy, row.get("emitted_violations"), row.get("relaxed"))
    if braked or ev:
        return TickInput(t=t, outcome="blocked", violations=hits,
                         stop_illegal=(stop if stop is not None
                                       else _blocked_stop_illegal(braked, ev)),
                         unsafe_depth_m=depth)
    repairs = row.get("repairs")
    if not repairs:
        raise ValueError(f"row t={t}: violations but no repair. This is a shield-off "
                         "control arm, and the escalation FSM does not run there")
    if _rescued(repairs) and not stop:
        why = ("a stop was legal" if stop is False
               else "the log does not record whether a stop was legal")
        return TickInput(t=t, outcome="blocked", violations=hits, stop_illegal=False,
                         note=f"{_RESCUE_NOTE}; {why}", unsafe_depth_m=depth)
    mag = repair_magnitude(repairs, row.get("raw"), row.get("emitted"), horizon_s,
                           caps=caps, stop_illegal=bool(stop))
    return TickInput(t=t, outcome="repaired", violations=hits, magnitude=mag,
                     stop_illegal=bool(stop), unsafe_depth_m=depth)


def replay_rows(rows: Iterable[Mapping[str, Any]], horizon_s: float | None = None,
                config: FSMConfig | None = None, policy: Any = None,
                caps: KinematicCaps | None = None) -> dict[str, Any]:
    """Run a delivered flight's rows through the FSM.

    COUNTERFACTUAL after the first escalation: the real flight never
    escalated, so once the FSM would have asked for LOITER / RTL / LAND, the
    rest of the log is a flight that would not have happened. Only the first
    escalation is a finding, and `max_state_counterfactual` is labelled
    accordingly.

    `caps` defaults to the policy's kinematic caps. The proxy needs them on
    every tick that carries a clamp, so a replay of a clamped flight without
    a policy (or caps) is refused rather than measured with the clamp in.
    """
    if caps is None and policy is not None:
        caps = KinematicCaps.from_policy(policy)
    f = EscalationFSM(config)
    first = None
    edges: dict[str, int] = {}
    n = rep = over = onsets = 0
    governed_n = exempt_only = clamps_only = exempt_recov = 0
    gov_recov = gov_clamp = rescue_blocked = stop_known = stop_ticks = 0
    prev = False
    for row in rows:
        inp = tick_input_from_row(row, policy, horizon_s, caps)
        n += 1
        hard = _hard_enforced(inp.violations)
        onsets += int(hard and not prev)
        prev = hard
        stop_known += int(row.get("unsafe_rules") is not None or row.get("unsafe") is not None)
        stop_ticks += int(inp.stop_illegal)
        rescue_blocked += int(_RESCUE_NOTE in inp.note)
        reps = row.get("repairs") or []
        if inp.outcome == "repaired":
            rep += 1
            over += int(inp.magnitude.exceeds(f.config))
            # Ticks theta never judged are counted, so a low over-theta figure
            # cannot hide that most ticks were never measured against theta.
            if any(theta_governs(r, inp.stop_illegal) for r in reps):
                governed_n += 1
                gov_recov += int(any(_is_recovery(r, inp.stop_illegal) for r in reps))
                gov_clamp += int(any(_field(r, "operator") in VELOCITY_CLAMPS for r in reps))
            else:
                exempt_only += 1
                if any(_is_recovery(r, inp.stop_illegal) for r in reps):
                    exempt_recov += 1
                else:
                    clamps_only += 1
        o = f.step(inp)
        if o.edge:
            edges[o.edge] = edges.get(o.edge, 0) + 1
        if first is None and o.state not in _GUIDED_STATES:
            first = {"tick": row.get("tick", n), "t_s": inp.t, "edge": o.edge,
                     "state": o.state.value, "reason": o.reason}
    return {
        "ticks": n,
        "repaired_ticks": rep,
        "repaired_theta_governed": governed_n,
        "repaired_over_theta": over,
        "repaired_theta_exempt_only": exempt_only,
        "repaired_clamps_only": clamps_only,
        "repaired_exempt_with_recovery": exempt_recov,
        "governed_with_recovery": gov_recov,
        "governed_with_velocity_clamp": gov_clamp,
        "rescue_rows_read_as_blocked": rescue_blocked,
        "rows_recording_stop_illegal": stop_known,
        "stop_illegal_ticks": stop_ticks,
        "hard_onsets": onsets,
        "first_escalation": first,
        "max_state_counterfactual": f.summary()["max_state"],
        # Every edge the FSM took over the whole log. After the first
        # escalation this is counterfactual where the flight's own FSM
        # differed (the log then holds what the OTHER state machine flew).
        "edge_counts_counterfactual": dict(sorted(edges.items())),
        "horizon_s": horizon_s,
        "magnitude_source": ("velocity proxy |dv| * horizon_s, from the clamped raw action"
                             + (" (the one-period reading: one control period)"
                                if horizon_s is not None
                                and abs(horizon_s - THETA_HORIZON_S) < EPS else "")
                             if horizon_s is not None else "per-operator magnitude_m"),
        "kinematic_caps": caps.to_dict() if caps is not None else None,
        "fsm_config_hash": f.config.digest,
    }


def _main(argv: Sequence[str] | None = None) -> int:
    """python -m guardrail.fsm replay demo/out/<run> [--policy P] [--horizon S]
    [--event-mode tick]"""
    import argparse
    import sys
    ap = argparse.ArgumentParser(prog="python -m guardrail.fsm")
    sub = ap.add_subparsers(dest="cmd", required=True)
    rp = sub.add_parser("replay", help="replay a flight_log.jsonl through the FSM")
    rp.add_argument("run_dir", nargs="+")
    rp.add_argument("--horizon", type=float, default=None,
                    help=f"|dv| horizon (s); {THETA_HORIZON_S:g} is the one-period "
                         "reading (one control period; selectable, pending the PI); "
                         "omit for per-operator magnitude_m, the default basis")
    rp.add_argument("--policy", default=None,
                    help="the policy the run flew: resolves hard/soft, priority and "
                         "kinematic rules, and gives the proxy its caps")
    rp.add_argument("--event-mode", default="onset", choices=EVENT_MODES)
    a = ap.parse_args(argv)
    cfg = FSMConfig(event_mode=a.event_mode)
    pol = None
    if a.policy:
        from guardrail import load_policy   # local: fsm.py imports nothing from the package
        pol = load_policy(a.policy)
    for d in a.run_dir:
        try:
            p = Path(d) / "flight_log.jsonl"
            rows = [json.loads(l) for l in p.read_text(encoding="utf-8").splitlines()
                    if l.strip()]
            out = {"run": str(d), **replay_rows(rows, horizon_s=a.horizon, config=cfg,
                                                policy=pol)}
        except (ValueError, OSError) as e:
            # A refusal (no magnitude_m, a clamp with no caps, a shield-off
            # control arm, no log) is the answer, not a crash: say which run
            # and why, and exit non-zero.
            print(f"{d}: refused: {e}", file=sys.stderr)
            return 2
        if pol is not None:
            out["policy"] = str(a.policy)
            out["policy_hash_loaded"] = pol.policy_hash
            run_hash = None
            try:
                run_hash = json.loads((Path(d) / "manifest.json")
                                      .read_text(encoding="utf-8")).get("policy_hash")
            except (OSError, ValueError):
                pass
            out["policy_hash_run"] = run_hash
            # Prefix compare: stored runs carry the legacy 16-hex form. A
            # mismatch is reported, not refused: the caps are also checked
            # against the log tick by tick (repair_magnitude).
            out["policy_hash_matches_run"] = (None if not run_hash else
                                              pol.policy_hash[:len(run_hash)] == run_hash)
        print(json.dumps(out))
    return 0


# --------------------------------------------------------------------------- #
# The KPI: fail-safe trigger correctness, with its nulls
# --------------------------------------------------------------------------- #

_Z95 = 1.959963984540054


def _wilson_low(k: int, n: int, z: float = _Z95) -> float | None:
    if n == 0:
        return None
    p = k / n
    den = 1 + z * z / n
    centre = p + z * z / (2 * n)
    margin = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (centre - margin) / den


def _min_error_free_n(target: float, z: float = _Z95) -> int | None:
    """Smallest n whose all-correct Wilson lower bound reaches the target.

    With k = n the bound is n / (n + z^2), so n >= z^2 * target / (1 - target).
    For the grant's 99 % that is 381 error-free episodes. Below that, a
    perfect run still cannot show the target at 95 % confidence.
    """
    if not 0 < target < 1:
        return None
    return math.ceil(z * z * target / (1 - target) - 1e-12)


def _label(v: Any, name: str, i: int) -> bool | None:
    if v is None or type(v) is bool:
        return v
    # "false" is truthy and 0/1 are not labels; refusing them is cheaper than
    # a correctness figure computed from strings.
    raise ValueError(f"episode {i}: {name} must be True, False or None, got {v!r}")


def score_failsafe_triggers(episodes: Iterable[Any], target: float = 0.99) -> dict[str, Any]:
    """Fail-safe trigger correctness (Stress Testing PDF p6), with its nulls.

        correctness = (correct triggers + correct non-triggers) / labelled episodes

    Each episode is a mapping with `expected_failsafe` and `triggered` (True,
    False or None), or an (expected, triggered) pair. Episodes with either
    value None are not scored, and they are COUNTED (`unlabelled`,
    `not_measured`) so a short denominator is visible.

    The two zero-skill baselines are reported beside it:
      null_always_trigger = expected triggers / labelled   (RTL every episode)
      null_never_trigger  = expected non-triggers / labelled (no fail-safe at all)
    A figure that does not beat both is not evidence of a fail-safe.

    When every scored episode has the same label (`discriminating` False),
    one of the two stubs ties whatever the Shield does. The correctness
    figure is still returned, because it is what was measured, but
    `meets_target` and `meets_target_at_95` are None: 400 episodes of
    (expected False, triggered False) score 1.0 for a Shield with no
    fail-safe at all, and a passing target flag there is the silent success
    this project keeps finding.

    `guardrail/kpi.py` computes something else under this KPI's name: one
    minus the P0 escape rate over P0 ticks. That needs no label, and it reads
    1.0 for a Shield whose fail-safe never fired, because nothing in it can
    count a missed trigger or a false one.
    """
    if not 0 < target <= 1:
        raise ValueError(f"target must be in (0, 1], got {target!r}")
    total = unlabelled = not_measured = 0
    tp = tn = fp = fn = 0
    for i, ep in enumerate(episodes):
        total += 1
        if isinstance(ep, Mapping):
            if "expected_failsafe" not in ep:
                raise ValueError(f"episode {i}: no 'expected_failsafe' key. Label "
                                 "it, or set it to None to exclude it explicitly")
            exp, trig = ep["expected_failsafe"], ep.get("triggered")
        else:
            exp, trig = ep
        exp = _label(exp, "expected_failsafe", i)
        trig = _label(trig, "triggered", i)
        if exp is None:
            unlabelled += 1
            continue
        if trig is None:
            not_measured += 1
            continue
        if exp and trig:
            tp += 1
        elif exp and not trig:
            fn += 1
        elif trig:
            fp += 1
        else:
            tn += 1
    n = tp + tn + fp + fn
    pos, neg = tp + fn, tn + fp
    corr = None if n == 0 else (tp + tn) / n
    null_always = None if n == 0 else pos / n
    null_never = None if n == 0 else neg / n
    lo = _wilson_low(tp + tn, n)
    discriminating = pos > 0 and neg > 0
    warnings = []
    if n == 0:
        warnings.append("no scorable episode: correctness is undefined, not 0 or 1")
    elif pos == 0:
        warnings.append("no episode expects a fail-safe: a Shield whose fail-safe "
                        "never fires scores the same as this one, so the target "
                        "is not judged (meets_target None)")
    elif neg == 0:
        warnings.append("every episode expects a fail-safe: a Shield that triggers "
                        "on every episode scores the same as this one, so the "
                        "target is not judged (meets_target None)")
    if unlabelled or not_measured:
        warnings.append(f"{unlabelled} unlabelled and {not_measured} unmeasured "
                        f"episodes were left out of {total}")
    return {
        "failsafe_trigger_correctness": None if corr is None else round(corr, 6),
        "episodes": total,
        "scored": n,
        "unlabelled": unlabelled,
        "not_measured": not_measured,
        "true_triggers": tp,
        "true_non_triggers": tn,
        "false_triggers": fp,
        "missed_triggers": fn,
        "expected_triggers": pos,
        "expected_non_triggers": neg,
        "null_always_trigger": None if null_always is None else round(null_always, 6),
        "null_never_trigger": None if null_never is None else round(null_never, 6),
        "beats_null": (None if corr is None
                       else corr > max(null_always, null_never) + EPS),
        "discriminating": discriminating,
        "wilson_low_95": None if lo is None else round(lo, 6),
        "target": target,
        # Judged only on a set that can tell a fail-safe from a stub.
        "meets_target": (None if corr is None or not discriminating
                         else corr >= target - EPS),
        "meets_target_at_95": (None if lo is None or not discriminating
                               else lo >= target - EPS),
        "min_error_free_episodes_for_target_at_95": _min_error_free_n(target),
        "warnings": warnings,
    }


if __name__ == "__main__":
    raise SystemExit(_main())
