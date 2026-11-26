"""
Policy DSL - Pydantic models, the YAML loader, the grant's own form, and the
layer merge.

The grant's constraint taxonomy (Policy DSL page) has nine classes. All nine are
declarable here, plus two of this project's own:

    polygon_fence       keep-OUT area (no-fly zone), 2-D polygon + altitude band
    circle_fence        keep-OUT disc: centre + radius + altitude band
    corridor            keep-IN tube: centerline + width + altitude band
    altitude_envelope   min/max height
    kinematic_envelope  speed / climb-rate / turn-rate caps
    distance_envelope   min distance to a typed object class (people, buildings,
                        roads); enforced since 2026-11-03 through the
                        clearance runtime (DistanceEnvelope; people: every
                        known person since 2026-11-18)
    dynamic_nfz         polygon with motion, injected mid-flight
    time_window_switch  switches a referenced rule on/off
    corridor_swap       replaces an active corridor with another
    obstacle_clearance  (ours) min distance from MAPPED buildings
    subject_standoff    (ours) min distance from the tracked subject

A type outside RUNTIME_TYPES would be declarable only: the DSL validates,
hashes, bundles and exports it, and the flight loaders (`load_policy`,
`load_layered`, `bundle.load_bundle`, `bundle.load_for_flight`) refuse it
unless the caller asks for the declaration only (`runtime=False`): a rule the
Shield silently ignores is worse than a refusal. Since 2026-11-03 no type is
(distance_envelope was the last). Refused too: an MSL altitude (the Shield has
no ground-elevation source), and a geographic policy whose frame is derived
rather than stated (the ArduPilot SITL rail puts the vehicle's start, its
home, at the frame origin).
`Policy.flight_problems()` says what and why. The Shield has enforced
dynamic_nfz, time_window_switch and corridor_swap since 2026-10-07
(guardrail/shield.py, ENFORCED_TYPES); since 2026-10-08 the compiler renders
them too (guardrail/templates/), so a policy declaring a switch or a swap
flies. A policy FILE declaring a MOVING dynamic_nfz is still refused for
flight (FLIGHT_HELD_TYPES): the Shield enforces it, but no rail can score
the time spent inside it. A dynamic_nfz that does not move flies from a file
since 2026-11-15 (contract item C-16): the rails count it and offline
analysis, the BC features and the prompt read it with the polygon fences
(`Policy.static_keepout_zones`). distance_envelope, the last declarable-only
type, flies since 2026-11-03.

Every rule may carry `valid_time`, a recurring schedule; that is a FIELD, as in
the grant's ConstraintBase, not a class.

TWO SURFACE FORMS, ONE IR

A rule may be written in the grant's form (Policy DSL page, worked example; the
reference repository's bundles/itri-icl-2026-demo.yaml) or in this project's
earlier form, and both load into the same model:

    grant form                              this project's form (still loads)
    geometry: {vertices, altitude_floor_m,  vertices, altitude_floor_m, ...
               altitude_ceiling_m,          at the rule's top level
               altitude_ref}
    altitude_min_m / altitude_max_m         alt_min_m / alt_max_m
    speed_max / climb_rate_max /            speed_max_mps / climb_rate_max_mps /
    turn_rate_max                           yaw_rate_max_dps
    violation_action: project_fix           violation_action: repair (alias)
    scope (required since IR 4.0.0)         scope (required since IR 4.0.0)
    layer, altitude_ref                     (absent = the grant's defaults)
    issued_at, layers_merged                (absent)

The IR keeps this project's field names, so no stored hash moved. `to_grant_form`
produces the grant's form back.

COORDINATES

WGS84 is canonical (grant: "WGS84 latitude / longitude is canonical ... the DSL
itself never carries projected coordinates"). A geographic point keeps lat/lon in
the IR and its hash; its x/y (metres, x = North, y = East of the policy's frame
origin) are a view, computed when the Shield or the compiler first reads them
and cached per origin (XY; guardrail/projection.py) - since IR schema 3.1.0
(2026-11-03), not at load. Every shipped policy states its `origin` and writes
its points in lat/lon since that date (tools/convert_policies_wgs84.py). A policy
written in metres is the legacy form, labelled by `Policy.point_form`
(POINTS_LEGACY_METRES / POINTS_LEGACY_MIXED): the READ form of old bundles, runs
and lock recipes, which the IR (Policy.model_validate) still parses and hashes
exactly as before. Since 2026-11-26 the DSL ENTRY takes WGS84 only (see "WGS84-only
entry" below): load_policy / policy_from_raw / load_layered refuse a metre
document (ProjectedCoordinatesRefused), and the declared re-reads use
load_legacy_metre_policy; the Shield's event entries and the REST bodies refuse
an absolute position in metres, and our own metre-based callers convert it tool
side with `wgs84_from_metres`. Until then every writer into a policy with a
frame converted metre input at ingest (deviation 1 of the final report). Since
2026-11-18 a write with points into a policy with NO frame origin is refused
(NoFrameOrigin), so no writer produces the legacy form any more. Altitude is
metres above ground by default; `altitude_ref: MSL` is declarable and refused
for flight unless the rail gives a ground (X-11).

docs/DESIGN-policy-dsl-grant-form.md records the decisions and what still
differs from the reference implementation.
"""
from __future__ import annotations

import functools
import hashlib
import json
import math
import typing
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import Annotated, Any, ClassVar, Literal, Union

import yaml
from pydantic import (BaseModel, ConfigDict, Field, PrivateAttr, ValidationInfo,
                      field_validator, model_serializer, model_validator)

from .fsm import action_strength, canonical_action
from .projection import (CONSISTENCY_TOL_M, LocalProjection,
                         check_ring, derive_frame_origin, local_metres,
                         metres_to_wgs84, project_raw, projection_for, quantize_m,
                         ring_order)


# --------------------------------------------------------------------------- #
# Runtime data types (not part of the policy file)
# --------------------------------------------------------------------------- #

class Action4D(BaseModel):
    """The VLA's output — the ONLY thing the Shield accepts. Grant contract."""
    vx: float = 0.0        # m/s, +North
    vy: float = 0.0        # m/s, +East
    vz_up: float = 0.0     # m/s, +up  (NED conversion happens at the sim adapter)
    # RADIANS per second, +clockwise seen from above. Not degrees: this comment
    # said deg/s until 2026-09-07 while every line that ENFORCES the cap read
    # radians (shield.py converts with math.radians/math.degrees at three
    # sites), and the only live producer - demo/follow_vlm.py's servo - emits a
    # radian-scale value. Two of the four adapters believed the comment and
    # applied math.radians() a second time; see
    # docs/FINDING-the-contract-disagreed-with-itself-about-yaw.md.
    yaw_rate: float = 0.0


class State(BaseModel):
    """Minimal vehicle state the Shield needs at each tick."""
    x: float               # m North of the frame origin
    y: float               # m East of the frame origin
    up: float              # m above ground
    yaw_deg: float = 0.0
    # The MEASURED ground velocity (vx North, vy East, vz_up), m/s, in the
    # Action4D frame - the autopilot's own estimate (EKF), never the command.
    # Optional, and None by default, which is every caller before 2026-10-19:
    # the Shield's forecast then starts from the commanded velocity exactly as
    # before, and every decision is byte-identical (tests/
    # test_own_velocity_forecast.py pins 18,000 of them against c77c77d).
    # Given, the Shield forecasts each pose from this velocity converging to
    # the commanded one through its response model (guardrail/response.py,
    # docs/DESIGN-own-velocity-forecast.md), and judges that forecast beside
    # the commanded one, so a velocity can only add: on the 14 Oct ArduPilot close
    # approach the commanded-velocity forecast put the aircraft 1.5 m short of
    # where it went and it ended 1.0 m inside the P0 building margin. The
    # vertical component is recorded but not forecast from (the climb rate is
    # still the commanded one; the design note says why). A non-finite value
    # is not refused here - x and y are not either - but the Shield ignores
    # it and counts it (Shield.n_velocity_nonfinite).
    velocity: tuple[float, float, float] | None = None
    # Since 2026-11-06: is (x, y, up) a CURRENT, VALID position fix? None (the
    # default, every rail on that date): the rail does not say, and the
    # Shield trusts the position exactly as before. True: it is. False: it is
    # not - a stale pose (a bridge forwarding its last sample; the pose's own
    # stamp older than the monitor period), the autopilot reporting no 3D fix
    # (MAVLink GPS_RAW_INT.fix_type < 3; MAVROS /mavros/gpsstatus/gps1/raw or
    # the NavSatFix status of /mavros/global_position/raw/fix). The Shield
    # then dead-reckons on what it streamed since the last valid fix and
    # escalates once the error can no longer be certified by a hard P0 zone's
    # margin (the FSM's X8; docs/RUBRIC-failsafe.md E3;
    # docs/DESIGN-escalation-fsm.md, "X8" has where each rail's signal is) -
    # unless the rail also reports the autopilot's own verdict on its
    # estimate (estimator_ok, below), which then decides.
    fix_ok: bool | None = None
    # Since 2026-11-06: the reported 1-sigma HORIZONTAL accuracy of (x, y), m -
    # GPS_RAW_INT.h_acc (a MAVLink 2 extension, in mm), the NavSatFix
    # position_covariance MAVROS builds from it, or the EKF's own position
    # variance. None (the default): NOT REPORTED - never read as perfect, and
    # nothing changes. Reported, the Shield reads it for the FSM's onset
    # hysteresis (a ring re-touched within 3 sigma is the same violation seen
    # through noise) and for X8 (3 sigma beyond a hard P0 margin near its
    # boundary). Once a rail reports it, every hard P0 margin_m becomes an
    # accuracy requirement: a 1 m margin certifies a zone only while 3 sigma
    # <= 1 m. Sizing margins to a rail's real accuracy is a policy decision.
    pos_sigma_m: float | None = None
    # Since 2026-11-12: the AUTOPILOT'S OWN VERDICT on the horizontal position
    # estimate (x, y), on a rail that hands the Shield the autopilot's
    # navigation solution. None (the default; every rail but the ROS 2 node):
    # not reported, and fix_ok alone decides, exactly as before. True: the
    # autopilot's estimator reports a valid absolute horizontal position
    # (ArduPilot EKF_STATUS_REPORT: EKF_POS_HORIZ_ABS set, EKF_CONST_POS_MODE
    # and EKF_UNINITIALIZED clear, the horizontal-position and velocity test
    # ratios below FS_EKF_THRESH), so (x, y) is the estimate the autopilot
    # itself navigates, returns and fences on, valid by its own test - also
    # while its GPS is denied (fix_ok False: EKF3 dead-reckons on its IMU for
    # up to 7 s, posRetryTimeNoVel_ms). False: it reports no valid absolute
    # horizontal position for as long as its own EKF failsafe waits
    # (ArduCopter ekf_check: 1 s), and the position certifies no zone.
    # Reported, it decides X8's position input before fix_ok does
    # (Shield._position_verdict; docs/DESIGN-escalation-fsm.md, "2026-11-12:
    # X8 on the autopilot's own estimator").
    estimator_ok: bool | None = None

    @model_serializer(mode="wrap")
    def _omit_absent_velocity(self, handler):
        # A State without a velocity dumps exactly as it did before the field
        # existed (demo/real_vla_demo.py writes State.model_dump() into its
        # logs), the same rule Policy.canonical_ir applies to optional fields.
        out = handler(self)
        if isinstance(out, dict) and out.get("velocity", 0) is None:
            out.pop("velocity")
        if isinstance(out, dict) and out.get("fix_ok", 0) is None:
            out.pop("fix_ok")
        if isinstance(out, dict) and out.get("pos_sigma_m", 0) is None:
            out.pop("pos_sigma_m")
        if isinstance(out, dict) and out.get("estimator_ok", 0) is None:
            out.pop("estimator_ok")
        return out


# --------------------------------------------------------------------------- #
# Vocabulary (the grant's ConstraintBase, Policy DSL page)
# --------------------------------------------------------------------------- #

DAYS = ("Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun")

Scope = Literal["global", "mission", "segment", "waypoint"]
Layer = Literal["regulation", "site", "mission"]
AltitudeRef = Literal["AGL", "MSL"]
# The grant's six, exactly (Policy DSL page, ConstraintBase). Until IR schema
# 3.0.0 (2026-11-02) there was a seventh, `repair`: this repository's spelling
# since July of what the grant calls project_fix - the Shield's projection
# repair (guardrail/fsm.py has always read it as project_fix,
# fsm.ACTION_ALIASES). It is still ACCEPTED on input and stored as
# project_fix (LEGACY_ACTION_SPELLINGS), so every policy, bundle and log that
# wrote it still loads; the IR never holds it. A run recorded before 3.0.0
# carries the hash of the old spelling, which `Policy.ir2_hashes` reproduces.
ViolationAction = Literal["monitor_only", "project_fix", "brake", "loiter", "RTL",
                          "land"]
LEGACY_ACTION_SPELLINGS: dict[str, str] = {"repair": "project_fix"}
# What an omitted constraint_type / priority / violation_action MEANT under
# IR schema 2.x, when they had defaults. Not a default any more: used only to
# read a record written then that relied on them (guardrail.bundle.derive, the
# lock's hot-apply recipes), so it is rebuilt exactly as it flew.
IR2_RULE_DEFAULTS: dict[str, str] = {"constraint_type": "hard", "priority": "P0",
                                     "violation_action": "repair"}

# Merge precedence, lowest first (grant: "mission > site > regulation").
LAYER_ORDER: tuple[str, ...] = ("regulation", "site", "mission")
# What an absent `layer` means: the grant's ConstraintBase defaults it to
# "mission". Absent stays absent in the IR (None) so no stored hash moves.
DEFAULT_LAYER = "mission"
# What an absent `scope` MEANT before IR schema 4.0.0 (the reference's default;
# the grant gives scope none). Not a default any more: since 4.0.0 every rule
# states its scope. Kept to read a rule of an earlier era's document, which
# may leave it out (`effective_scope`, `era_context`).
DEFAULT_SCOPE = "mission"

# IR schema 4.0.0 (2026-11-06, WP1-01) made `scope` REQUIRED, as the grant's
# ConstraintBase writes it. Until then every shipped policy, every rule built
# in code and so every artefact recorded before left it out, and such a
# document must still load and hash exactly as it was stored. A reader of a
# STORED artefact says which era the document is from (`era_context`); a
# document validated without that context is today's, and must state scope.
SCOPE_REQUIRED_SINCE = "4.0.0"
# The last IR schema version under which a rule could leave `scope` out. The
# IR shapes of 3.1.0 and 4.0.0 differ in that requirement only, so a policy
# read from an earlier era's document that leaves scope out IS a 3.1.0
# document (Policy.ir_schema_version).
LAST_SCOPE_OPTIONAL = "3.1.0"
_ERA_KEY = "ir_era"


def _semver_tuple(v) -> tuple:
    """(major, minor, patch) of a plain semver string; () when unreadable."""
    try:
        parts = tuple(int(x) for x in str(v).split("-")[0].split("+")[0].split("."))
    except ValueError:
        return ()
    return parts if len(parts) == 3 else ()


def era_context(ir_schema_version: str | None) -> dict:
    """The validation context for a STORED document:

        Policy.model_validate(doc, context=era_context(version))

    `version` is the IR schema version the artefact records - a bundle
    manifest's or a lock recipe's `ir_schema_version` - or None for one that
    records none: a bundle written before 2026-10-06, a lock recipe of IR
    2.x, a replay bundle, a run's policy_g<N>.json. Under a version before
    SCOPE_REQUIRED_SINCE, and under None, a rule may leave `scope` out and it
    stays absent (None), so the document hashes exactly as it did when it was
    recorded. Under SCOPE_REQUIRED_SINCE or later, or a version that cannot
    be read, scope is required, as for a document written today.

    Tolerating None for an artefact that records no version weakens nothing:
    a document that leaves scope out can only be one written before
    SCOPE_REQUIRED_SINCE, and the stored hash it must reproduce (the bundle's,
    the replay index's, the run's) is a hash of exactly that document."""
    return {_ERA_KEY: ir_schema_version}


def scope_may_be_absent(context) -> bool:
    """True when the validation `context` says the document is of an era
    before SCOPE_REQUIRED_SINCE (see era_context)."""
    if not isinstance(context, dict) or _ERA_KEY not in context:
        return False
    v = context[_ERA_KEY]
    if v is None:
        return True
    key = _semver_tuple(v)
    return bool(key) and key < _semver_tuple(SCOPE_REQUIRED_SINCE)

# Every IR model refuses a key it does not know (a misspelt field used to be
# dropped without a word, and its safety limit fell back to a loose default)
# and refuses inf/NaN (an infinite cap passed `gt=0` and capped nothing; a NaN
# band failed every comparison, including the validator's).
_IR_CONFIG = ConfigDict(extra="forbid", allow_inf_nan=False)

_SEMVER = (r"^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)"
           r"(-[0-9A-Za-z-]+(\.[0-9A-Za-z-]+)*)?(\+[0-9A-Za-z-]+(\.[0-9A-Za-z-]+)*)?$")


class Recurrence(BaseModel):
    """A weekly schedule: which days, and a clock window within them.

    `start_time` / `end_time` are "HH:MM" local. A window that ends before it
    starts WRAPS past midnight ("22:00"-"06:00"), which is the common case for a
    night-flight restriction and would otherwise be inexpressible.
    """
    model_config = _IR_CONFIG
    days: list[Literal[DAYS]] = Field(default_factory=lambda: list(DAYS))
    start_time: str = "00:00"
    end_time: str = "23:59"

    @staticmethod
    def _mins(hhmm: str) -> int:
        h, _, m = hhmm.partition(":")
        return int(h) * 60 + int(m)

    @model_validator(mode="after")
    def _parseable(self) -> "Recurrence":
        # Until 2026-10-19 only the TOTAL was range-checked: "10:75" loaded as
        # 11:15, "10:-5" as 09:55, " 7:30" and "+7:30" as 07:30 - each in
        # force at a time the author never wrote, and printed verbatim into
        # the CSP the pilot reads ("Mon 10:75-11:99"). And `days: []` loaded,
        # hashed and flew a rule that is never in force: the Shield reported
        # in_force False and the CSP "(in force no day)", the same "enforcing
        # nothing" Policy.lint refuses for an empty `constraints` list. Here,
        # not in lint, so scenario windows (guardrail/scenario_spec.py) and
        # directly built models are held to it too. A validator, not a Field
        # constraint, so the published JSON schema does not change.
        if not self.days:
            raise ValueError("the schedule names no day, so the rule would never be "
                             "in force (omit valid_time for a rule that always is)")
        for f, end in ((self.start_time, False), (self.end_time, True)):
            # ASCII digits, two each (`\d` would also take other scripts' digits).
            h, sep, m = f.partition(":")
            if not (sep and len(h) == 2 and len(m) == 2
                    and (h + m).isascii() and (h + m).isdigit()):
                raise ValueError(f"time {f!r} is not HH:MM")
            # "24:00" is the end of the day (shield._hhmm_s reads it so), so it
            # may end a window but not start one.
            if not ((int(h) <= 23 and int(m) <= 59) or (end and f == "24:00")):
                raise ValueError(f"time {f!r} out of range (00:00-23:59; 24:00 only "
                                 f"as an end_time)")
        return self

    def active_at(self, when) -> bool:
        """Is this schedule in force at `when` (a datetime)?"""
        if DAYS[when.weekday()] not in self.days:
            return False
        now = when.hour * 60 + when.minute
        a, b = self._mins(self.start_time), self._mins(self.end_time)
        if a <= b:
            return a <= now <= b
        return now >= a or now <= b          # wraps past midnight

    def week_intervals(self) -> list[tuple[int, int]]:
        """The schedule as closed [start, end] minute intervals on one week
        (Mon 00:00 = 0). A window that wraps spills into the next day, and
        Sunday's spills into Monday. Used to decide whether two rules can be
        in force at the same time."""
        a, b = self._mins(self.start_time), self._mins(self.end_time)
        week = 7 * 1440
        out: list[tuple[int, int]] = []
        for d in self.days:
            base = DAYS.index(d) * 1440
            if a <= b:
                out.append((base + a, base + b))
            else:
                out.append((base + a, base + 1440))
                nxt = (base + 1440) % week
                out.append((nxt, nxt + b))
        return out


class ValidTime(BaseModel):
    """When a rule is in force. `recurrence` absent means always."""
    model_config = _IR_CONFIG
    recurrence: Recurrence | None = None

    def active_at(self, when) -> bool:
        return True if self.recurrence is None else self.recurrence.active_at(when)


def _times_overlap(a: "ConstraintBase", b: "ConstraintBase") -> bool:
    """Can rules a and b be in force at the same moment? Conservative: True
    unless both carry recurrences whose weekly intervals never meet."""
    ra = a.valid_time.recurrence if a.valid_time else None
    rb = b.valid_time.recurrence if b.valid_time else None
    if ra is None or rb is None:
        return True
    return any(x0 <= y1 and y0 <= x1
               for x0, x1 in ra.week_intervals() for y0, y1 in rb.week_intervals())


# --------------------------------------------------------------------------- #
# Constraint taxonomy (the policy file surface)
# --------------------------------------------------------------------------- #

def _normalise_rule(cls, data):
    """Turn the grant's surface form of one rule into the IR's field names.

    1. A `geometry:` block (grant form) is lifted to the rule's top level, and
       the reference implementation's geometry defaults are filled in for what
       it omits - altitude band 0-200 m and no margin ring - because that is
       what the same block means to the reference. A key given both inside
       `geometry` and beside it is refused, as is a key `geometry` may not hold.
    2. Grant field names are renamed to this project's (`altitude_min_m` ->
       `alt_min_m`, ...). Both names for one field is refused, not resolved.
    3. A class hook (`_post_normalise`) runs last: circle_fence derives its
       polygon there.
    """
    if not isinstance(data, dict):
        return data
    out = dict(data)
    geo = out.pop("geometry", None)
    rid = out.get("id", "?")
    if geo is not None:
        if not cls.GRANT_GEOMETRY:
            raise ValueError(f"{rid}: a {out.get('type')!r} rule takes no "
                             f"`geometry` block")
        if not isinstance(geo, dict):
            raise ValueError(f"{rid}: `geometry` must be a mapping")
        for k, v in geo.items():
            if k not in cls.GRANT_GEOMETRY:
                raise ValueError(f"{rid}: unknown key {k!r} in `geometry` "
                                 f"(allowed: {sorted(cls.GRANT_GEOMETRY)})")
            if k in out:
                raise ValueError(f"{rid}: {k!r} is given both inside `geometry` "
                                 f"and beside it; give it once")
            out[k] = v
        for k, v in cls.GRANT_DEFAULTS.items():
            out.setdefault(k, v)
    for grant, ours in cls.GRANT_ALIASES.items():
        if grant in out:
            if ours in out:
                raise ValueError(f"{rid}: {grant!r} and {ours!r} name the same "
                                 f"field; give one of them")
            out[ours] = out.pop(grant)
    return cls._post_normalise(out)


class ConstraintBase(BaseModel):
    """The grant's ConstraintBase (Policy DSL page).

    `constraint_type`, `priority` and `violation_action` are REQUIRED, as the
    grant writes them (IR schema 3.0.0, 2026-11-02). Until then they defaulted
    to hard / P0 / `repair`, so a rule that said nothing about how hard it was
    or what a breach does was enforced as the strictest rule with a
    projection repair - a safe guess, but a guess the policy's author never
    made, and the reference implementation guesses differently (priority P1).
    Every shipped policy already wrote all three; every rule built in code now
    states them too.

    `scope` is REQUIRED too, since IR schema 4.0.0 (2026-11-06): the grant
    gives it no default either. It was added on 2026-10-06 as an optional
    field whose absence meant "mission" (`DEFAULT_SCOPE`, the reference's
    default), and no shipped policy wrote it. A rule of a document of an
    earlier era may still leave it out (`era_context`, which the readers of
    stored bundles, replay bundles, runs and lock recipes pass): it stays
    absent in the IR, so that document hashes exactly as it was stored, and
    `effective_scope` reads it as "mission", what it meant then. The model's
    annotation admits None for that reason only; a document validated
    without an earlier era's context that leaves scope out, or writes null,
    is refused. The Shield enforces every rule everywhere, whatever its
    scope; a narrower `scope` (segment, waypoint) is therefore enforced more
    widely than written, never less.

    `layer` was added on 2026-10-06 too. Absent means the grant's default
    ("mission", `DEFAULT_LAYER`) and stays absent in the IR, so a policy that
    does not write it hashes as before.
    """
    model_config = _IR_CONFIG
    id: str = Field(min_length=1)
    constraint_type: Literal["hard", "soft"]
    priority: Literal["P0", "P1", "P2"]
    violation_action: ViolationAction
    # Required (no default) since IR schema 4.0.0; None only in a rule of an
    # earlier era's document (era_context).
    scope: Scope | None
    layer: Layer | None = None
    # Provenance a layer merge leaves behind: the LOWER layers that wrote a
    # hard copy of this rule which this (higher-layer, no looser) copy
    # overrides. merge_layers computes it; absent on every rule nobody
    # overrode, so no stored hash moves. Without it the merged rule carried
    # only its winner's layer, and a mission copy of a regulation rule - an
    # identical one, or the tightening the grant allows - could then be
    # switched off or swapped wide by any mission event, at ingest or over
    # REST (found 2026-10-18). The Shield refuses an event that would relax a
    # hard rule unless the event's layer is the rule's layer AND every layer
    # named here (shield.Shield._cross_layer_hard). Writing it by hand can
    # only add refusals, never remove one - and Policy.lint refuses it on a
    # policy no merge produced (no `layers_merged`), and as an empty list.
    locked_by: list[Layer] | None = None

    # Time windows are a FIELD on every rule, not a constraint type of their
    # own. That is the reference DSL's shape (`policy-dsl.md`): a recurring
    # schedule is something any rule may carry, while `time_window_switch` is a
    # separate hot-applicable EVENT that toggles a rule by reference. Modelling
    # the schedule as a type would have made "this fence applies on weekdays"
    # inexpressible without inventing a link between two rules.
    valid_time: ValidTime | None = None

    # Surface-form tables (see _normalise_rule). ClassVars, not fields.
    GRANT_ALIASES: ClassVar[dict[str, str]] = {}
    GRANT_GEOMETRY: ClassVar[frozenset[str]] = frozenset()
    GRANT_DEFAULTS: ClassVar[dict[str, Any]] = {}

    @model_validator(mode="before")
    @classmethod
    def _grant_form(cls, data):
        return _normalise_rule(cls, data)

    @model_validator(mode="before")
    @classmethod
    def _earlier_era_scope(cls, data, info: ValidationInfo):
        # A rule of an earlier era's document (era_context) may leave `scope`
        # out, as every document before IR schema 4.0.0 could: it stays
        # absent, so the document hashes as it was stored. Without that
        # context a missing scope is pydantic's "Field required".
        if isinstance(data, dict) and "scope" not in data and \
                scope_may_be_absent(info.context):
            return {**data, "scope": None}
        return data

    @field_validator("scope")
    @classmethod
    def _scope_stated(cls, v, info: ValidationInfo):
        if v is None and not scope_may_be_absent(info.context):
            raise ValueError(
                "scope is required (IR schema 4.0.0; the grant's ConstraintBase "
                "gives it no default): one of global, mission, segment, waypoint. "
                "Only a document of an earlier era may leave it out")
        return v

    @classmethod
    def _post_normalise(cls, data: dict) -> dict:
        return data

    @field_validator("violation_action", mode="before")
    @classmethod
    def _legacy_spelling(cls, v):
        # `repair` (this repository's spelling until IR 3.0.0) is read as the
        # grant's project_fix, at this boundary, so nothing downstream - the
        # IR, the CSP, the escalation FSM - ever sees the old name. Exact
        # spelling only: any other unknown name is refused below.
        return LEGACY_ACTION_SPELLINGS.get(v, v) if isinstance(v, str) else v

    @field_validator("violation_action")
    @classmethod
    def _known_to_the_fsm(cls, v: str) -> str:
        # Checked against the escalation FSM's own table so the two cannot
        # drift apart.
        canonical_action(v)
        return v

    @property
    def effective_scope(self) -> str:
        """The rule's scope; for a rule of an earlier era's document that left
        it out, what that meant then ("mission", DEFAULT_SCOPE)."""
        return self.scope or DEFAULT_SCOPE

    @property
    def effective_layer(self) -> str:
        return self.layer or DEFAULT_LAYER

    def active_at(self, when) -> bool:
        """Is this rule in force?

        ABSENT MEANS ACTIVE, and that default is deliberate. A P0 rule that
        silently switches itself off because its window was omitted, malformed
        or misread is the single worst failure this file could enable - the
        Shield would report a clean flight while enforcing nothing. Every path
        that cannot establish "this rule is off right now" must leave it on.
        """
        if self.valid_time is None or when is None:
            return True
        return self.valid_time.active_at(when)


# Serialisation contexts of a point (XY._serialise). STORED: what the canonical
# IR holds - a WGS84 point as lat/lon only. ERA: a WGS84 point with the
# UNROUNDED metres hash scheme v2 and the 16-hex eras stored for it (prior and
# legacy hash forms only). No context: lat/lon plus the local view, the shape
# every consumer of a dump has always seen.
_STORED = {"points": "stored"}
_ERA = {"points": "era"}

# How a policy's points are stored (Policy.point_form, WP1-14). The grant:
# "WGS84 latitude / longitude is canonical ... the DSL itself never carries
# projected coordinates". The two legacy forms are READ (old bundles, runs,
# lock recipes); see Policy.point_form for what can still write them.
# Not to be confused with bundle.FRAME_FORM ("sha256-canonical-v3/local-
# metres"), which names a HASH form in a lock entry's migration record, not
# how a policy's points are stored.
POINTS_WGS84 = "wgs84"
POINTS_NONE = "no-points"
POINTS_LEGACY_METRES = "local-metres (legacy read form)"
POINTS_LEGACY_MIXED = "wgs84+local-metres (legacy read form)"
LEGACY_POINT_FORMS = (POINTS_LEGACY_METRES, POINTS_LEGACY_MIXED)


class XY(BaseModel):
    """A point: WGS84 `lat` / `lon` (canonical), or local metres `x` / `y`
    (x = North, y = East of the policy's frame origin; the legacy form).

    A WGS84 point stores lat/lon only: they are what the canonical IR holds and
    hashes, and the grant forbids the DSL to carry projected coordinates. Its
    `x` / `y` are a VIEW, computed the first time something reads them (the
    Shield, the compiler) from the frame origin the point is bound to - its
    policy's, bound when the policy validates (Policy._bind_frame) - and
    cached per origin (projection.local_metres, rounded to the micrometre).
    Until 2026-11-03 (IR schema 3.1.0) x/y were required fields, filled in for
    every point when the policy loaded.

    A WGS84 point validated on its own (a rule outside any policy) has no
    frame yet: reading its x/y raises, unless it was given metres beside
    lat/lon (a dump; a REST body may not carry them since 2026-11-26), which
    it then keeps as its view until a policy binds it. A model_dump writes
    the view beside lat/lon (as every dump did before 3.1.0); the canonical IR
    does not (`_STORED`).

    `x` and `y` are model fields AND are read through the properties set on
    the class below: the field holds the metres of a metre point (or the
    given view of an unbound WGS84 point), the property answers for both."""
    model_config = _IR_CONFIG
    x: float | None = None
    y: float | None = None
    lat: float | None = Field(default=None, ge=-90.0, le=90.0)
    lon: float | None = Field(default=None, ge=-180.0, le=180.0)
    # The frame origin (lat, lon) a WGS84 point is placed in; None until a
    # policy (or a validation context with a "frame") binds it.
    _frame: tuple[float, float] | None = PrivateAttr(default=None)

    @model_validator(mode="after")
    def _pair(self, info: ValidationInfo) -> "XY":
        if (self.lat is None) != (self.lon is None):
            raise ValueError("lat and lon come as a pair")
        sx, sy = self.__dict__.get("x"), self.__dict__.get("y")
        if (sx is None) != (sy is None):
            raise ValueError("x and y come as a pair")
        if self.lat is None and sx is None:
            raise ValueError("a point is {x, y} metres or {lat, lon}")
        ctx = info.context if isinstance(info.context, dict) else None
        frame = ctx.get("frame") if ctx else None
        if frame is not None and self.lat is not None:
            self._bind(tuple(frame))
        return self

    def _bind(self, frame: tuple[float, float]) -> None:
        """Place this WGS84 point in the frame of `frame` (lat, lon). Metres it
        was given beside lat/lon are checked against that frame (1 um) and
        dropped: from here on its x/y are the frame's view."""
        if self.lat is None:
            return
        sx, sy = self.__dict__.get("x"), self.__dict__.get("y")
        if sx is not None:
            x, y = projection_for(*frame).to_local(self.lat, self.lon)
            dx, dy = abs(sx - x), abs(sy - y)
            if dx > CONSISTENCY_TOL_M or dy > CONSISTENCY_TOL_M:
                raise ValueError(
                    f"point ({self.lat}, {self.lon}) carries x/y that disagree by "
                    f"({dx:.3g}, {dy:.3g}) m with its policy's frame; one point "
                    f"cannot mix frames")
            self.__dict__["x"] = self.__dict__["y"] = None
        self._frame = (float(frame[0]), float(frame[1]))

    def _view(self) -> tuple[float, float] | None:
        """(x, y) of this point in its frame, or None for an unbound WGS84
        point that was given no metres."""
        if self.lat is None:
            return (self.__dict__["x"], self.__dict__["y"])
        priv = self.__pydantic_private__
        frame = priv.get("_frame") if priv else None
        if frame is not None:
            return local_metres(frame, self.lat, self.lon)
        sx = self.__dict__.get("x")
        return None if sx is None else (sx, self.__dict__["y"])

    def local(self, origin: tuple[float, float] | None = None) -> tuple[float, float]:
        """(x North, y East) metres of this point in the frame of `origin`
        (lat, lon), default the frame it is bound to: the lazily computed,
        per-origin cached view (projection.local_metres). A metre point is
        in its policy's frame only, so it refuses another origin."""
        if origin is None:
            v = self._view()
            if v is None:
                raise ValueError(_UNBOUND)
            return v
        if self.lat is None:
            raise ValueError("a metre point has no WGS84 position, so it cannot be "
                             "placed in another frame")
        return local_metres((float(origin[0]), float(origin[1])), self.lat, self.lon)

    @model_serializer(mode="wrap")
    def _serialise(self, handler, info):
        out = handler(self)
        if self.lat is None or not isinstance(out, dict):
            return out
        ctx = info.context if isinstance(info.context, dict) else None
        how = ctx.get("points") if ctx else None
        out.pop("x", None)
        out.pop("y", None)
        if how == "stored":
            return out
        if how == "era":
            frame = (self.__pydantic_private__ or {}).get("_frame")
            if frame is not None:
                x, y = projection_for(*frame).to_local(self.lat, self.lon)
                return {**out, "x": x, "y": y}
        v = self._view()
        if v is not None:
            out["x"], out["y"] = v
        return out

    @property
    def geographic(self) -> bool:
        return self.lat is not None


_UNBOUND = ("a WGS84 point that no policy has placed: its metres depend on the "
            "policy's frame origin. Validate the whole policy (or pass "
            "context={'frame': (lat, lon)}), or give x/y beside lat/lon")


def _coord(i: int, name: str) -> property:
    def get(self: XY) -> float:
        v = self._view()
        if v is None:
            raise ValueError(f"{name}: " + _UNBOUND)
        return v[i]
    return property(get, doc=f"{'North' if i == 0 else 'East'} metres in the "
                             f"point's frame (stored, or the WGS84 point's view)")


# The read side of x / y (see XY's docstring). Set after the class is built:
# pydantic keeps field values in the instance __dict__, and a property on the
# class takes precedence over it, so `point.x` is answered here while
# validation and serialisation read the field.
XY.x = _coord(0, "x")
XY.y = _coord(1, "y")


def _orient(a, b, c) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a, b, p, eps: float) -> bool:
    return (min(a[0], b[0]) - eps <= p[0] <= max(a[0], b[0]) + eps
            and min(a[1], b[1]) - eps <= p[1] <= max(a[1], b[1]) + eps)


def _segments_meet(a, b, c, d, eps: float = 1e-9) -> bool:
    """Do closed segments ab and cd share any point?"""
    o1, o2, o3, o4 = _orient(a, b, c), _orient(a, b, d), _orient(c, d, a), _orient(c, d, b)
    if ((o1 > eps and o2 < -eps) or (o1 < -eps and o2 > eps)) and \
            ((o3 > eps and o4 < -eps) or (o3 < -eps and o4 > eps)):
        return True
    return ((abs(o1) <= eps and _on_segment(a, b, c, eps))
            or (abs(o2) <= eps and _on_segment(a, b, d, eps))
            or (abs(o3) <= eps and _on_segment(c, d, a, eps))
            or (abs(o4) <= eps and _on_segment(c, d, b, eps)))


def _dedupe_ring(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """The ring with consecutive repeats removed, cyclically.

    A vertex written twice in a row adds a zero-length edge and changes no
    area: shapely's is_valid accepts it, GIS exports produce it (a ring written
    closed is the common case), and so does rounding nearby points to 0.1 m.
    Refusing it would turn a valid zone away for its spelling. The fence keeps
    the vertices as written; only this check reads the cleaned ring."""
    out: list[tuple[float, float]] = []
    for p in pts:
        if not out or out[-1] != p:
            out.append(p)
    while len(out) >= 2 and out[0] == out[-1]:
        out.pop()
    return out


def polygon_problem(pts) -> str | None:
    """Why a vertex ring is not a usable keep-out polygon, or None.

    The grant's ingest pipeline names "self-intersection / ring direction /
    holes / coordinate bounds" under geometry normalisation. A bow-tie's inside
    depends on the fill rule, so the zone enforced is not the zone drawn; a
    zero-area ring protects nothing while looking like a rule. Ring direction
    is irrelevant to a containment test and is accepted either way; repeated
    consecutive vertices (a closed ring, a doubled GIS vertex) are read once.

    Pure Python, because models.py does not import shapely. What it is checked
    against is exactly this, and no more: tests/test_policy_dsl_grant_form.py
    compares it with shapely's `is_valid` plus non-zero area on 300 seeded
    random rings, the same rings with a vertex doubled, and every fence in
    policies/, with no disagreement. It is not a proof of agreement with
    shapely on every input.

    Edges are compared after a sweep along x that prunes pairs whose x and y
    ranges cannot meet, so an ordinary n-vertex ring costs about n log n, not
    the n^2 / 2 pair tests of the first version (3.5 s at 2000 vertices).
    """
    pts = _dedupe_ring([(float(x), float(y)) for x, y in pts])
    n = len(pts)
    if n < 3:
        return "a polygon needs at least 3 distinct vertices"
    edges = [(pts[i], pts[(i + 1) % n]) for i in range(n)]
    eps = 1e-9

    def adjacent(i, j):
        return abs(i - j) == 1 or {i, j} == {0, n - 1}

    # Crossings first: a bow-tie's two lobes cancel to zero signed area, and
    # "collinear" would name the wrong defect. Adjacent edges are not compared:
    # they share a vertex by construction. Two adjacent edges that fold back
    # over each other need no test of their own - for n >= 4 the fold puts a
    # vertex on a NON-adjacent edge, which this loop finds, and for n = 3 the
    # ring is collinear, which the area check below finds.
    box = [(min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1]))
           for a, b in edges]
    active: list[int] = []
    for i in sorted(range(n), key=lambda k: box[k][0]):
        x0, x1, y0, y1 = box[i]
        active = [j for j in active if box[j][1] >= x0 - eps]
        for j in active:
            if adjacent(i, j) or box[j][2] > y1 + eps or box[j][3] < y0 - eps:
                continue
            if _segments_meet(*edges[i], *edges[j]):
                a, b = min(i, j), max(i, j)
                return f"edges {a} and {b} cross (the polygon intersects itself)"
        active.append(i)
    area2 = sum(pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1]
                for i in range(n))
    if abs(area2) / 2.0 < 1e-6:
        return "the polygon has no area (its vertices are collinear)"
    return None


def _stored_pair(p) -> tuple[float, float]:
    """The coordinates a point is STORED by: (lat, lon) for a WGS84 point,
    (x, y) metres otherwise; from an XY or from a point dict."""
    if isinstance(p, dict):
        if p.get("lat") is not None and p.get("lon") is not None:
            return (float(p["lat"]), float(p["lon"]))
        return (float(p["x"]), float(p["y"]))
    return (p.lat, p.lon) if p.geographic else (p.x, p.y)


def canonical_ring_order(points: list) -> list:
    """The same ring in its canonical order (IR schema 3.0.0, WP1-02): wound
    with POSITIVE signed area over the stored coordinate pairs, starting at
    the lexicographically smallest one. The grant's ingest pipeline names
    "ring direction" under geometry normalisation; until 2026-11-02 the
    direction was checked (polygon_problem) but kept as written, so the same
    zone written clockwise, or from another vertex, had another policy_hash.

    The stored pairs are (x = North, y = East) for a metre point and
    (lat, lon) for a WGS84 one. Both put north first, and the local
    projection scales each axis by a positive factor, so the winding is the
    same in either; on a north-up map a positive ring runs clockwise. It is
    what shapely's orient(..., sign=1.0) gives for these tuples, and the
    order the grant's worked example and all three of the reference
    implementation's documents already use, so their reference hashes do
    not move.

    Only the ORDER changes: the multiset of vertices is the same, so every
    vertex mean (the derived frame origin, a dynamic_nfz's rotation centre)
    and every vertex the Shield measures from are unchanged. A ring whose
    points are not all in one frame, or with no area, is returned as given
    (the validators refuse both). Where the smallest pair occurs more than
    once (a doubled vertex), the smallest rotation as a whole wins."""
    pts = list(points)
    kinds = {(p.get("lat") is not None) if isinstance(p, dict) else p.geographic
             for p in pts}
    if len(kinds) != 1:
        return pts
    return [pts[i] for i in ring_order([_stored_pair(p) for p in pts])]


class _CanonicalRing:
    """Shared by the keep-out polygons: the ring is stored in its canonical
    order (canonical_ring_order), and the order it was GIVEN in is kept
    beside it, outside the IR (a pydantic private attribute, `_given_vertices`,
    declared on each model), so the hash an IR 2.x artefact recorded for it
    can still be reproduced (Policy.ir2_hashes)."""

    def _canonicalise_ring(self) -> None:
        self._given_vertices = list(self.vertices)
        self.vertices = canonical_ring_order(self.vertices)

    def given_vertices(self) -> list:
        """The vertices in the order this rule was given them."""
        return list(self._given_vertices if self._given_vertices is not None
                    else self.vertices)


def _point_in_ring(x: float, y: float, ring) -> bool:
    """Even-odd containment of (x, y) in a simple ring (boundary counts):
    on an edge, else the library's even-odd test (guardrail/planar.py; a copy
    of its own until 2026-11-17, WP3-22)."""
    from .planar import point_in_ring_xy
    n = len(ring)
    if any(_segments_meet(ring[i], ring[(i + 1) % n], (x, y), (x, y)) for i in range(n)):
        return True
    return point_in_ring_xy(x, y, ring)


def _dist_to_ring(x: float, y: float, ring) -> float:
    from .planar import ring_edge_distance
    return ring_edge_distance(x, y, ring)


class Origin(BaseModel):
    """The anchor a lat/lon policy is projected about. See guardrail/projection.py.

    Optional. The grant's form has no origin and the frame is then derived the
    way the reference derives it (`Policy.frame_origin`)."""
    model_config = _IR_CONFIG
    lat: float = Field(ge=-90.0, le=90.0)
    lon: float = Field(ge=-180.0, le=180.0)


# Fences written in the grant's form fill what the reference fills.
_FENCE_GEOMETRY = frozenset({"vertices", "altitude_floor_m", "altitude_ceiling_m",
                             "altitude_ref"})
_FENCE_GRANT_DEFAULTS = {"altitude_floor_m": 0.0, "altitude_ceiling_m": 200.0,
                         "margin_m": 0.0}
_TUBE_GEOMETRY = frozenset({"centerline", "width_m", "altitude_floor_m",
                            "altitude_ceiling_m", "altitude_ref"})
_TUBE_GRANT_DEFAULTS = {"altitude_floor_m": 0.0, "altitude_ceiling_m": 200.0}


class _Banded:
    """Shared check for every rule with an altitude band."""

    def _check_band(self) -> None:
        if self.altitude_ceiling_m <= self.altitude_floor_m:
            raise ValueError(f"{self.id}: ceiling must be > floor")


class PolygonFence(_Banded, _CanonicalRing, ConstraintBase):
    """Keep-OUT no-fly zone. Violated when a (predicted) position is inside.

    The ring is stored in its canonical order (canonical_ring_order, IR
    schema 3.0.0): the same zone written in another direction or from
    another vertex is the same IR and the same policy_hash."""
    type: Literal["polygon_fence"]
    vertices: list[XY] = Field(min_length=3)
    altitude_floor_m: float = 0.0
    altitude_ceiling_m: float = 1000.0
    margin_m: float = 1.0            # extra safety ring around the polygon
    altitude_ref: AltitudeRef | None = None

    _given_vertices: list | None = PrivateAttr(default=None)

    GRANT_GEOMETRY: ClassVar[frozenset[str]] = _FENCE_GEOMETRY
    GRANT_DEFAULTS: ClassVar[dict[str, Any]] = _FENCE_GRANT_DEFAULTS

    @model_validator(mode="after")
    def _sane_band(self) -> "PolygonFence":
        self._check_band()
        return self

    @model_validator(mode="after")
    def _simple_polygon(self) -> "PolygonFence":
        # A circle_fence's polygon is derived (a regular 32-gon about its
        # centre): never self-intersecting, and for a WGS84 centre not
        # placed until its policy binds it (CircleFence.vertices).
        if self.type == "circle_fence":
            return self
        try:
            why = polygon_problem(check_ring(self.vertices))
        except ValueError as e:
            why = str(e)
        if why:
            raise ValueError(f"{self.id}: {why}")
        self._canonicalise_ring()
        return self

    def ring(self) -> list[tuple[float, float]]:
        return [(v.x, v.y) for v in self.vertices]


class CircleFence(PolygonFence):
    """Keep-OUT disc (grant: "Centre + radius + AGL floor / ceiling").

    A subclass of PolygonFence on purpose: its `vertices` are DERIVED - the
    regular 32-gon circumscribing the circle - so every consumer that handles a
    polygon fence (the Shield, its STRtree, the CSP) enforces a circle without a
    line of new code. Circumscribed, so the polygon contains the whole disc; it
    reaches at most r * (1/cos(pi/32) - 1) = 0.48 % of the radius beyond it,
    the conservative direction for a keep-out zone.

    The derived vertices are not part of the IR (`exclude=True`): the IR and
    its hash hold the centre and radius as written, and the polygon is rebuilt
    at every load. That makes SEGMENTS part of what a stored hash MEANS without
    being part of the hash: changing it changes the enforced zone of every
    circle_fence under an unchanged policy_hash. It is therefore pinned by
    tests/test_policy_dsl_grant_form.py, and changing it requires an
    IR_SCHEMA_VERSION bump."""
    type: Literal["circle_fence"]
    center: XY
    radius_m: float = Field(gt=0)
    vertices: list[XY] = Field(default_factory=list, exclude=True)

    SEGMENTS: ClassVar[int] = 32
    GRANT_ALIASES: ClassVar[dict[str, str]] = {"centre": "center"}
    GRANT_GEOMETRY: ClassVar[frozenset[str]] = frozenset({
        "center", "centre", "radius_m", "altitude_floor_m", "altitude_ceiling_m",
        "altitude_ref"})

    @classmethod
    def _post_normalise(cls, data: dict) -> dict:
        if data.get("vertices"):
            raise ValueError(f"{data.get('id', '?')}: a circle_fence's polygon is "
                             f"derived from center + radius_m; do not write vertices")
        c, r = data.get("center"), data.get("radius_m")
        if (c.get("lat") if isinstance(c, dict) else getattr(c, "lat", None)) is not None:
            # A WGS84 centre: the polygon is derived where the centre is
            # placed, when it is first read (CircleFence.vertices below).
            return data
        try:
            cx = float(c["x"] if isinstance(c, dict) else c.x)
            cy = float(c["y"] if isinstance(c, dict) else c.y)
            r = float(r)
        except (TypeError, ValueError, KeyError, AttributeError):
            return data                  # field validation reports what is wrong
        if not (math.isfinite(cx) and math.isfinite(cy) and math.isfinite(r)) or r <= 0:
            return data
        n = cls.SEGMENTS
        big = r / math.cos(math.pi / n)
        verts = [{"x": cx + big * math.cos(2 * math.pi * k / n),
                  "y": cy + big * math.sin(2 * math.pi * k / n)} for k in range(n)]
        return {**data, "vertices": verts}


@functools.lru_cache(maxsize=4096)
def _circle_ring(cx: float, cy: float, r: float, n: int) -> tuple:
    """The circumscribing n-gon of a circle as metre points, exactly the
    arithmetic of CircleFence._post_normalise. Cached: a WGS84 circle's
    polygon is derived from its centre's view when first read."""
    big = r / math.cos(math.pi / n)
    return tuple(XY(x=cx + big * math.cos(2 * math.pi * k / n),
                    y=cy + big * math.sin(2 * math.pi * k / n)) for k in range(n))


def _circle_vertices(self: CircleFence) -> list:
    v = self.__dict__.get("vertices")
    if v or self.center.lat is None:
        return v                     # a metre centre: derived at validation
    return list(_circle_ring(self.center.x, self.center.y, float(self.radius_m),
                             self.SEGMENTS))


# The read side of a circle's `vertices` (the field is excluded from the IR;
# see XY for the pattern): a WGS84 centre's polygon is derived lazily, in the
# frame the centre is placed in.
CircleFence.vertices = property(_circle_vertices,
                                doc="The derived 32-gon (metre points, policy frame)")


class AltitudeEnvelope(ConstraintBase):
    """Stay between alt_min and alt_max (metres above ground unless MSL)."""
    type: Literal["altitude_envelope"]
    alt_min_m: float
    alt_max_m: float
    altitude_ref: AltitudeRef | None = None

    # The worked example and the reference say altitude_min_m; the taxonomy
    # table says alt_min / alt_max.
    GRANT_ALIASES: ClassVar[dict[str, str]] = {
        "altitude_min_m": "alt_min_m", "altitude_max_m": "alt_max_m",
        "alt_min": "alt_min_m", "alt_max": "alt_max_m"}

    @model_validator(mode="after")
    def _sane(self) -> "AltitudeEnvelope":
        if self.alt_max_m <= self.alt_min_m:
            raise ValueError(f"{self.id}: alt_max must be > alt_min")
        return self


class KinematicEnvelope(ConstraintBase):
    """Caps on how fast the vehicle may move/climb/turn.

    Grant names: speed_max, climb_rate_max, turn_rate_max. Units are this
    project's: m/s, m/s and DEGREES per second for the turn rate."""
    type: Literal["kinematic_envelope"]
    speed_max_mps: float = Field(gt=0)          # horizontal speed cap
    climb_rate_max_mps: float = Field(gt=0)     # |vz| cap
    yaw_rate_max_dps: float = Field(gt=0)

    GRANT_ALIASES: ClassVar[dict[str, str]] = {
        "speed_max": "speed_max_mps", "climb_rate_max": "climb_rate_max_mps",
        "turn_rate_max": "yaw_rate_max_dps"}


class ObstacleClearance(ConstraintBase):
    """Keep at least `min_clearance_m` away from every MAPPED obstacle
    (surveyed buildings in the city occupancy grid).

    Unlike the other three, this rule cannot be evaluated from the policy file
    alone — it needs the occupancy map, which is handed to the Shield at
    construction time (`Shield(..., obstacle_map=...)`). With no map the rule
    is INERT: it never fires and never raises. Distance is measured in the
    horizontal plane only; buildings are treated as infinitely tall columns,
    which is the conservative reading of a 2-D occupancy grid.

    soft_margin_m widens the band the repair operator tapers speed over
    ([0, min_clearance_m + soft_margin_m]); it never raises a violation on its
    own — hence "slows, does not block".

    With subject_standoff this is this project's half of the grant's
    `distance_envelope` (buildings and the followed subject); the grant's own
    class is declarable below as DistanceEnvelope.
    """
    type: Literal["obstacle_clearance"]
    min_clearance_m: float = Field(gt=0)
    soft_margin_m: float = Field(default=0.0, ge=0)


# NOTE (2026-09-08, corrected 2026-10-19): `soft_margin_m` below is declared,
# hashed into policy_hash and set to 2.0 in every flown follow policy. It has
# three readers:
#   * guardrail/repair.py PathRepair._standoff_waypoints - the stand-off detour
#     waypoint sits on a circle of radius min_range_m + max(1.0, soft_margin_m),
#     so it DOES change the Shield's commands, on a Shield built with
#     path_repair=True (the default since 2026-11-17, flown by the ROS 2 KPI
#     rail; the follow rails pin it off, guardrail/replay.py
#     FOLLOW_RAIL_PATH_REPAIR; every run flown before then had it off);
#   * demo/policy_hud.py - the on-screen NEAR status;
#   * _not_looser below - lowering it counts as loosening when layers merge.
# StandoffHold / StandoffRecover do not read it (the speed taper in
# guardrail/shield.py's ObstacleClearance repair uses ObstacleClearance's own
# soft_margin_m). On the rails flown before 2026-11-17 (path_repair off) it
# therefore changed the reproducibility hash and the HUD, not the Shield's
# commands; from then on a stand-off detour reads it. The 2026-09-08
# version of this note said "read by the Shield at no site"; PathRepair
# (2026-10-13) made that false.
#
# Left in place rather than deleted: removing it would change policy_hash on
# every stored run and orphan the two replay bundles, for a field that costs
# nothing. Recorded here so the next reader knows exactly where it takes
# effect, and does not tune it on a path_repair=False rail expecting an outcome.
class SubjectStandoff(ConstraintBase):
    """Keep at least `min_range_m` from the SUBJECT being followed.

    Asked for at the 2026-08-19 review: "hold 10 m from a person, and different
    policies for different objects". Until now that was `--want-range`, a
    command-line flag on the controller - which meant it was not hashed into
    `policy_hash`, not written to the audit log, and not enforced by the Shield.
    It was a setpoint the pilot was asked to aim for, not a rule it was held to,
    and the difference is the whole point of the project.

    Like ObstacleClearance this cannot be evaluated from the policy file alone:
    the Shield has no idea where the subject is. The perception stack supplies it
    once per tick through `Shield.set_subject()`. With no subject set the rule is
    INERT - it never fires and never raises - because a standoff rule with
    nothing to stand off from has no opinion, and inventing one would be worse
    than silence.

    `subject_class` selects which rule applies to what: "pedestrian" binds only
    when the tracked subject is a pedestrian, "*" binds to anything. That is what
    makes "different policies for different objects" expressible rather than a
    single global number.

    Distance is horizontal only, matching ObstacleClearance. Altitude is governed
    by AltitudeEnvelope, and mixing the two would make a rule that a legal climb
    could violate.
    """
    type: Literal["subject_standoff"]
    subject_class: str = "*"
    min_range_m: float = Field(gt=0)
    soft_margin_m: float = Field(default=0.0, ge=0)

    def binds(self, subject_class: str | None) -> bool:
        """Does this rule apply to the subject currently being tracked?"""
        if self.subject_class == "*":
            return True
        if subject_class is None:
            return False
        return self.subject_class.lower() == subject_class.lower()


class _Tube(_Banded):
    """Shared by Corridor and CorridorSwap: a centerline with a width."""

    @property
    def half_width_m(self) -> float:
        return self.width_m / 2.0

    def points(self) -> list[tuple[float, float]]:
        return [(v.x, v.y) for v in self.centerline]


class Corridor(_Tube, ConstraintBase):
    """Stay INSIDE a 3-D corridor: centerline polyline, width, altitude band.

    The inverse of PolygonFence, and the first keep-IN rule in this file. A
    fence says "not here"; a corridor says "only here", which is what a delivery
    or survey mission is actually authorised to do.

    Named in the reference DSL taxonomy (`policy-dsl.md`: "Centerline polyline +
    width + AGL bounds") and implemented in neither repository until now -
    `packages/policy-dsl/src/policy_dsl/models.py` carries only PolygonFence and
    AltitudeEnvelope.

    Distance is measured to the centerline SEGMENTS, never to its vertices; see
    `geometry.nearest_on_polyline` for why that distinction has already cost
    this project one flight.

    The altitude band is part of the corridor rather than delegated to
    AltitudeEnvelope, because a corridor is a tube: "inside the width but above
    the ceiling" is outside the corridor, and expressing that with a separate
    global envelope would make it apply everywhere else too.
    """
    type: Literal["corridor"]
    centerline: list[XY] = Field(min_length=2)
    width_m: float = Field(gt=0)             # full width; half is the tolerance
    altitude_floor_m: float = 0.0
    altitude_ceiling_m: float = 1000.0
    altitude_ref: AltitudeRef | None = None

    GRANT_GEOMETRY: ClassVar[frozenset[str]] = _TUBE_GEOMETRY
    GRANT_DEFAULTS: ClassVar[dict[str, Any]] = _TUBE_GRANT_DEFAULTS

    @model_validator(mode="after")
    def _sane_band(self) -> "Corridor":
        self._check_band()
        return self


class DistanceEnvelope(ConstraintBase):
    """Grant: "Min distance to typed object class (people, buildings, roads)".

    Enforced since 2026-11-03 (WP1-07); until then declarable only, and
    refused by the flight loaders. The Shield enforces it through the runtime
    it already has for each class (`runtime_rule`), under the rule's own id,
    priority, breach action and schedule:

      buildings  the clearance runtime over the OBSTACLE map
                 (Shield(obstacle_map=...)): obstacle_clearance's check and
                 repair, at min_distance_m;
      roads      the same runtime over a ROAD map (Shield(road_map=...), the
                 street mask demo/build_street_mask.py writes): distance to
                 the nearest road cell;
      people     the same runtime over the PEOPLE field (since 2026-11-18,
                 guardrail/people.py): distance to the nearest person the
                 Shield is told about (Shield.set_people) - every one, plus
                 the tracked subject while perception classifies it as a
                 person (PEOPLE_CLASSES) - each a disc as wide as the person
                 may have walked since its fix.

    What it does NOT see, stated rather than implied: buildings and roads
    exist only where the maps cover (off the map is unknown, Shield.off_map),
    and people are only those the rail's source reports (Shield.people_source:
    the detector's person boxes on the perception rail, the simulator's
    pedestrian list on the city sim rails, the declared subject alone on the
    SITL rails and the headless harness, whose worlds have no pedestrians).
    A Shield is never built over an envelope without its source (refused at
    construction: obstacle_map, road_map, people_source), so the rule is
    never inert where it claims to hold.

    PEOPLE BEFORE 2026-11-18 (the recorded limit it closes, WP1-07). The rule
    held min_distance_m from ONE position: the subject a rail passed to
    Shield.set_subject, as a stand-off (PeopleStandoff, now removed), so a
    person beside a tracked car was outside it and a car follow under a
    people envelope never bound it; the detectors were prompted for the
    target alone. No flight had carried a people envelope, so no flown
    decision changed with it."""
    type: Literal["distance_envelope"]
    object_class: Literal["people", "buildings", "roads"]
    min_distance_m: float = Field(gt=0)

    def runtime_rule(self) -> "ConstraintBase":
        """The rule the Shield enforces in this one's place (see the class
        docstring): same id, constraint_type, priority, violation_action,
        scope, layer, merge provenance and schedule."""
        base = dict(id=self.id, constraint_type=self.constraint_type,
                    priority=self.priority, violation_action=self.violation_action,
                    scope=self.scope, layer=self.layer, locked_by=self.locked_by,
                    valid_time=self.valid_time)
        # The scope is carried as it is: absent too, for a rule of an earlier
        # era's document (IR schema 4.0.0 made it required; era_context).
        ctx = era_context(None) if self.scope is None else None
        cls = {"roads": RoadClearance, "people": PeopleClearance}.get(
            self.object_class, EnvelopeClearance)
        return cls.model_validate(dict(type="obstacle_clearance",
                                       min_clearance_m=self.min_distance_m, **base),
                                  context=ctx)


# Subject classes perception reports for a person (demo/follow_vlm.py and the
# PAS rail say "pedestrian"; the COCO detectors "person").
PEOPLE_CLASSES: frozenset[str] = frozenset({"pedestrian", "person", "people", "human"})


class EnvelopeClearance(ObstacleClearance):
    """The clearance runtime of a distance_envelope (DistanceEnvelope.
    runtime_rule; not an IR type, never in a policy). `MAP` names the map
    the Shield measures it on: buildings = the obstacle map."""
    MAP: ClassVar[str] = "buildings"


class RoadClearance(EnvelopeClearance):
    """A distance_envelope over `roads`: measured on the Shield's road map."""
    MAP: ClassVar[str] = "roads"


class PeopleClearance(EnvelopeClearance):
    """A distance_envelope over `people` (since 2026-11-18): measured on the
    Shield's people field (guardrail/people.py PeopleField: every person
    Shield.set_people names, and the tracked subject while it is classified
    as a person, PEOPLE_CLASSES). Until then it was a stand-off from the
    tracked subject alone (PeopleStandoff)."""
    MAP: ClassVar[str] = "people"

    @staticmethod
    def is_person(subject_class: str | None) -> bool:
        return subject_class is not None and subject_class.lower() in PEOPLE_CLASSES


class Motion(BaseModel):
    """How a dynamic_nfz moves after it appears: a constant translation and a
    rotation about its centroid (grant: "Polygon with motion (translate /
    rotate)"). +yaw is clockwise seen from above (North towards East)."""
    model_config = _IR_CONFIG
    vx_mps: float = 0.0          # North
    vy_mps: float = 0.0          # East
    yaw_rate_dps: float = 0.0


class DynamicNFZ(_Banded, _CanonicalRing, ConstraintBase):
    """Grant: hot-applicable "Polygon with motion (translate / rotate) injected
    mid-flight".

    The Shield enforces it, motion included, since 2026-10-07
    (guardrail/shield.py, hot-apply); since 2026-10-08 the compiler renders it
    (RUNTIME_TYPES), so a hot-applied generation holding one compiles a CSP.
    The flight loaders refuse a policy FILE that declares one that MOVES
    (FLIGHT_HELD_TYPES, `moves`): no rail can score the time spent inside a
    moving zone. One without motion flies from a file since 2026-11-15
    (C-16; Policy.static_keepout_zones). `ring_at(t)` is the declared
    motion's geometry."""
    type: Literal["dynamic_nfz"]
    vertices: list[XY] = Field(min_length=3)
    altitude_floor_m: float = 0.0
    altitude_ceiling_m: float = 1000.0
    margin_m: float = 1.0
    altitude_ref: AltitudeRef | None = None
    motion: Motion | None = None         # absent = does not move once spawned

    _given_vertices: list | None = PrivateAttr(default=None)

    GRANT_GEOMETRY: ClassVar[frozenset[str]] = _FENCE_GEOMETRY
    GRANT_DEFAULTS: ClassVar[dict[str, Any]] = _FENCE_GRANT_DEFAULTS

    @model_validator(mode="after")
    def _checks(self) -> "DynamicNFZ":
        self._check_band()
        try:
            why = polygon_problem(check_ring(self.vertices))
        except ValueError as e:
            why = str(e)
        if why:
            raise ValueError(f"{self.id}: {why}")
        # Canonical order (see PolygonFence). ring_at rotates about the vertex
        # MEAN, which reordering does not move, so the motion is unchanged.
        self._canonicalise_ring()
        return self

    @property
    def moves(self) -> bool:
        """A motion that is not zero on every channel (guardrail.ir.is_moving's
        rule: the Shield moves only such a zone)."""
        m = self.motion
        return m is not None and any(abs(v) > 0.0 for v in
                                     (m.vx_mps, m.vy_mps, m.yaw_rate_dps))

    def ring_at(self, t_s: float) -> list[tuple[float, float]]:
        """The polygon t_s seconds after it appeared, in the policy frame."""
        pts = [(v.x, v.y) for v in self.vertices]
        if self.motion is None or t_s == 0:
            return pts
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        th = math.radians(self.motion.yaw_rate_dps * t_s)
        c, s = math.cos(th), math.sin(th)
        dx, dy = self.motion.vx_mps * t_s, self.motion.vy_mps * t_s
        return [(cx + dx + (x - cx) * c - (y - cy) * s,
                 cy + dy + (x - cx) * s + (y - cy) * c) for x, y in pts]


class TimeWindowSwitch(ConstraintBase):
    """Grant: hot-applicable, "Activates / deactivates a referenced rule at
    runtime". While this rule is in force (its own `valid_time`), the rule
    `target_id` is held `active` (true) or off (false).

    Flyable since 2026-10-08 (RUNTIME_TYPES; the Shield evaluates switches
    since 2026-10-07, see DynamicNFZ). The DSL checks that the target exists,
    and the layer merge refuses a switch that turns off a hard rule of
    another layer. The compiler applies switches exactly as the Shield does
    (guardrail/compiler.py `_RuleClock`), so a CSP never leaves out a rule a
    switch holds on."""
    type: Literal["time_window_switch"]
    target_id: str = Field(min_length=1)
    active: bool

    GRANT_ALIASES: ClassVar[dict[str, str]] = {"target": "target_id"}


class CorridorSwap(_Tube, ConstraintBase):
    """Grant: hot-applicable, "Replaces an active corridor with another at
    runtime". `target_id` names the corridor replaced; the rest is the new
    corridor, in the corridor's own fields.

    Flyable since 2026-10-08, as the other two hot-apply classes
    (RUNTIME_TYPES)."""
    type: Literal["corridor_swap"]
    target_id: str = Field(min_length=1)
    centerline: list[XY] = Field(min_length=2)
    width_m: float = Field(gt=0)
    altitude_floor_m: float = 0.0
    altitude_ceiling_m: float = 1000.0
    altitude_ref: AltitudeRef | None = None

    GRANT_ALIASES: ClassVar[dict[str, str]] = {"target": "target_id"}
    GRANT_GEOMETRY: ClassVar[frozenset[str]] = _TUBE_GEOMETRY
    GRANT_DEFAULTS: ClassVar[dict[str, Any]] = _TUBE_GRANT_DEFAULTS

    @model_validator(mode="after")
    def _sane_band(self) -> "CorridorSwap":
        self._check_band()
        return self


Constraint = Annotated[
    Union[PolygonFence, AltitudeEnvelope, KinematicEnvelope, ObstacleClearance,
          SubjectStandoff, Corridor, CircleFence, DistanceEnvelope, DynamicNFZ,
          TimeWindowSwitch, CorridorSwap],
    Field(discriminator="type"),
]

# The rule types the runtime handles: the Shield enforces them and the
# constraint compiler renders them (guardrail/compiler.py; tests/test_csp.py
# demands a template for each). Everything else is declarable only; see
# Policy.unenforced_rules. It must never hold a type the Shield does not
# enforce (shield.ENFORCED_TYPES; checked in
# tests/test_policy_dsl_grant_form.py). The Shield has enforced dynamic_nfz,
# time_window_switch and corridor_swap since 2026-10-07; they joined this set
# on 2026-10-08, in the change that gave the compiler their templates
# (sentence, summary, reason). distance_envelope joined on 2026-11-03 (WP1-07):
# the Shield enforces it through its clearance and stand-off runtime
# (DistanceEnvelope.runtime_rule) and the compiler renders it. An MSL altitude
# is refused separately, by name (unenforced_rules).
# A flight may carry every type here except a rule FLIGHT_HELD_TYPES holds.
RUNTIME_TYPES: frozenset[str] = frozenset({
    "polygon_fence", "circle_fence", "altitude_envelope", "kinematic_envelope",
    "obstacle_clearance", "subject_standoff", "corridor",
    "dynamic_nfz", "time_window_switch", "corridor_swap", "distance_envelope"})
# Runtime types the flight loaders still refuse in a policy FILE, with why
# (Policy.unmeasured_rules). Not a gap in the Shield, which enforces them,
# nor in the compiler, which renders them: a gap in what the flight rails
# MEASURE. Found in review on 2026-10-08, the day dynamic_nfz joined
# RUNTIME_TYPES: every rail except demo/follow_vlm.py computed its zone-dwell
# metric (nfz_s, which guardrail/kpi.compute reads for the mission outcome)
# from policy.by_type(PolygonFence), and a DynamicNFZ is not a PolygonFence,
# so a declared zone would add 0 s of dwell however long the aircraft spent
# in it - a silent zero. The grant's own route for this type, a zone
# "injected mid-flight" (Shield.hot_apply), does not pass through these
# loaders and is unchanged. Since 2026-10-11 every flight rail counts zone
# dwell through guardrail.replay.zone_dwell (polygon, circle and non-moving
# dynamic_nfz), so the rails no longer drop a zone that does not move.
#
# LIFTED FOR A ZONE THAT DOES NOT MOVE (2026-11-15, contract item C-16). The
# other consumers that read polygon fences only - experiments/
# analyze_semantic_ab.py's dwell and overlay, training/features.make_fences
# (the BC pilots' features) and the build_prompt no_fly_zones shorthand -
# read `Policy.static_keepout_zones()` now, which holds every dynamic_nfz
# without motion beside the polygon and circle fences, so such a zone is
# measured everywhere a polygon fence is, and a policy file may declare it.
#
# KEPT FOR A MOVING ZONE (`DynamicNFZ.moves`), because no consumer can place
# it where it was: the Shield places it at its monitor clock (filter()'s t,
# or the tick count x 0.1 s on a rail that passes none - the perception
# rail does - from its anchor, the first tick or the hot-apply), and no rail
# records that clock per trajectory point, so zone_dwell names the zone in
# nfz_moving_not_counted instead of scoring it; the BC features (encode)
# take no time at all; the offline overlay draws one position. Lifting it
# needs each rail to record the Shield's clock per point and zone_dwell to
# score `ring_at(age)` from it. Until then a moving zone flies as the grant
# has it, injected mid-flight (Shield.hot_apply; every rail's REST listener).
FLIGHT_HELD_TYPES: dict[str, str] = {
    "dynamic_nfz": (
        "it moves, and no rail records the Shield's monitor clock per "
        "trajectory point, so the dwell metric (nfz_s, guardrail.replay."
        "zone_dwell) names a moving zone without scoring it and the BC "
        "pilots' features cannot place it: time inside it could score 0 s; "
        "inject it mid-flight (Shield.hot_apply, the REST hot-apply API) "
        "instead, or declare it without motion"),
}


def flight_held(rule) -> bool:
    """Is `rule` held back from a policy FILE (FLIGHT_HELD_TYPES)? A
    dynamic_nfz only while it moves (2026-11-15, C-16)."""
    if rule.type not in FLIGHT_HELD_TYPES:
        return False
    return bool(getattr(rule, "moves", True))
# The grant's nine (Policy DSL page, taxonomy table).
GRANT_TYPES: tuple[str, ...] = (
    "polygon_fence", "circle_fence", "corridor", "altitude_envelope",
    "kinematic_envelope", "distance_envelope", "dynamic_nfz",
    "time_window_switch", "corridor_swap")
# Breach actions the Shield performs itself, whatever rail flies it. Since
# 2026-10-07 the Shield dispatches on the rule's action (guardrail/shield.py,
# ENFORCEMENT): project_fix repairs, monitor_only is recorded and never
# repaired, brake skips projection and stops. Until then it repaired, then
# braked, for every rule. (`repair`, read as project_fix since IR 3.0.0, is
# never stored, so it is not listed.)
SHIELD_ACTS_ON: frozenset[str] = frozenset({"project_fix", "monitor_only", "brake"})
# Mode-changing actions: the Shield skips projection and stops (where a stop
# is legal) and its escalation FSM (guardrail/fsm.py) requests the mode. The
# mode itself is flown only where the rail passes ShieldDecision.set_mode to an
# autopilot, so a flight records it (Policy.runtime_notes). A SOFT rule's
# response is capped at brake by the FSM, and that is recorded too.
MODE_ACTIONS: frozenset[str] = frozenset({"loiter", "RTL", "land"})


class Policy(BaseModel):
    """A validated policy (the 'IR' in miniature)."""
    model_config = _IR_CONFIG
    policy_id: str = Field(min_length=1)
    version: str = Field(default="0.1.0", pattern=_SEMVER)
    generation: int = Field(default=0, ge=0)   # bumps on every mid-flight hot-apply
    # In the grant's worked example and in the reference document; part of the
    # authored document and of its hash when written. A policy that does not
    # write it leaves the issue time to the bundle manifest (guardrail/bundle.py),
    # which never enters the hash.
    issued_at: str | None = None
    # Which layers a merged policy came from (grant worked example).
    layers_merged: list[Layer] | None = None
    # Optional frame anchor. When absent the frame is derived from the geometry
    # the way the reference derives it; see `frame_origin`.
    origin: Origin | None = None
    constraints: list[Constraint]

    # The multi-scale geometry a bundle shipped with this policy (WP2-16,
    # guardrail/ir.py "The bundle ships them"): {rule id: {scale: [[x, y]]}},
    # set by guardrail.bundle.check_bundle once verified; None for a policy
    # loaded from YAML. Not a field: never in the IR, the hash or the schema.
    _shipped_multiscale: dict | None = PrivateAttr(default=None)
    # The frame origin every WGS84 point of this policy was bound to when it
    # validated (_bind_frame): the stated origin, else the derived one.
    # Not a field: never in the IR, the hash or the schema.
    _frame_origin: tuple[float, float] | None = PrivateAttr(default=None)

    @property
    def shipped_multiscale(self) -> dict | None:
        return self._shipped_multiscale

    @model_validator(mode="before")
    @classmethod
    def _project(cls, data):
        # The frames are checked here, for the whole policy (only the policy
        # knows its frame origin). Nothing is projected: a WGS84 point's
        # metres are derived when first read (XY; projection.project_raw).
        return project_raw(data) if isinstance(data, dict) else data

    @model_validator(mode="after")
    def _bind_frame(self) -> "Policy":
        # Every WGS84 point is placed in this policy's frame - bound, not
        # projected: the projection runs when the Shield or the compiler
        # first reads the point's x / y (WP1-14: "ENU / NED projection is
        # computed lazily by the Prefix Compiler and Safety Shield").
        if not self.is_geographic:
            self._frame_origin = None
            return self
        if self.origin is not None:
            fo = (self.origin.lat, self.origin.lon)
        else:
            fo = derive_frame_origin([c.model_dump(mode="json", context=_STORED)
                                      for c in self.constraints])
        self._frame_origin = fo
        for pts in _rule_points(self, all_points=True):
            for v in pts:
                v._bind(fo)
        return self

    @field_validator("issued_at", mode="before")
    @classmethod
    def _timestamp_text(cls, v):
        # PyYAML turns an ISO timestamp into a datetime. The reference keeps the
        # IR as text via isoformat(); so does this, so both hash the same bytes.
        if isinstance(v, datetime):
            return v.isoformat()
        return v

    @field_validator("issued_at")
    @classmethod
    def _iso_8601(cls, v):
        if v is None:
            return v
        try:
            datetime.fromisoformat(v.replace("Z", "+00:00"))
        except ValueError:
            raise ValueError(f"issued_at {v!r} is not an ISO-8601 timestamp") from None
        return v

    # ------------------------------------------------------------------ #
    # The frame
    # ------------------------------------------------------------------ #

    @property
    def is_geographic(self) -> bool:
        """True when any point was written in WGS84."""
        return any(p for p in _rule_points(self))

    @property
    def point_form(self) -> str:
        """How this policy's points are STORED (WP1-14, 2026-11-18):

          POINTS_WGS84          every point lat/lon - the canonical form; every
                                policies/*.yaml since 2026-11-03;
          POINTS_NONE           no point at all (envelopes, stand-off, ...);
          POINTS_LEGACY_METRES  no point in lat/lon: the legacy metre READ form;
          POINTS_LEGACY_MIXED   lat/lon points beside metre points (a snapshot
                                of a generation hot-applied in metres before
                                2026-11-17): the legacy read form as well.

        The two legacy forms keep loading and hashing exactly as before, for
        the pre-3-Nov bundles, the runs flown under them and the lock recipes
        that rebuild them. Since 2026-11-18 nothing WRITES them: an event
        with points on a policy with no frame origin is refused
        (rule_in_wgs84 raises NoFrameOrigin; Shield: HotApplyRefused), as is
        a new bundle.derive on a metre base, tools/geo_to_policy.py ingest of
        metre rules onto a frameless base or of a whole metre document, and
        demo/gen_random_scenario.py --metres (removed). They are only READ
        back: a lock recipe on a metre base or recording `wgs84_writes:
        false` is rebuilt as its run flew (bundle.derive legacy_read). Since
        2026-11-26 the entries (load_policy / policy_from_raw, the REST
        bodies, the Shield's events, a new bundle.derive) refuse metre
        input (ProjectedCoordinatesRefused), our own metre callers convert
        tool side (wgs84_from_metres), and these forms are read only by
        load_legacy_metre_policy / legacy_metre_policy_from_raw; the one
        exception is Shield.wgs84_writes=False, a comparison switch no
        flight path sets, whose result write_bundle and the flight loader
        refuse. The flight loader REFUSES the legacy forms since 2026-11-18
        (bundle.load_for_flight; until then it flew them, labelled; a
        point-free policy with no origin still flies, and refuses every
        event with points); a declared read of a recorded
        flight (legacy_read / VLAGUARD_LEGACY_READ=1) loads them and records
        `point_form`, `point_note`. tools/bundle_census.py lists the bundles
        stored in them and the WGS84 bundle to fly instead.

        This labels how POINTS are stored; it is not bundle.FRAME_FORM, the
        name of a hash form in a lock entry's migration record."""
        flags = list(_rule_points(self))
        if not flags:
            return POINTS_NONE
        if all(flags):
            return POINTS_WGS84
        return POINTS_LEGACY_MIXED if any(flags) else POINTS_LEGACY_METRES

    @property
    def frame_origin(self) -> tuple[float, float] | None:
        """(lat, lon) of the local frame's (0, 0): the stated `origin`, else the
        reference's rule (mean of the first polygon_fence's vertices; then the
        first geographic rule). None for a policy written only in metres."""
        if self.origin is not None:
            return (self.origin.lat, self.origin.lon)
        if self._frame_origin is not None:
            return self._frame_origin
        return derive_frame_origin([c.model_dump(mode="json", context=_STORED)
                                    for c in self.constraints])

    def projection(self) -> LocalProjection | None:
        o = self.frame_origin
        return None if o is None else LocalProjection(*o)

    # ------------------------------------------------------------------ #
    # Identity. See docs/DESIGN-policy-identity.md for the whole story.
    # ------------------------------------------------------------------ #

    def canonical_ir(self) -> dict:
        """The canonicalised IR: every field that carries a value, nothing else,
        with geographic points stored as lat/lon only.

        Fields whose value is None are LEFT OUT, and that is the load-bearing
        decision of this file. Until 2026-10-06 the hash covered
        `model_dump()`, which writes every optional field as `null` - so the
        day `valid_time` (2026-09-01) and `origin` (2026-09-01) were added to
        the schema, every policy on disk changed fingerprint without a single
        byte of YAML changing. 42 of the 76 stored flight manifests and all
        five ArduPilot + MAVROS 2 runs then matched no policy in the repo.

        With None omitted, a new OPTIONAL field (one that defaults to None,
        meaning "absent") cannot move any existing hash: a policy that does not
        use it serialises to the same bytes as before the field existed. That
        is a rule on the schema as much as on this method, and
        tests/test_policy_hash.py enforces it - a new field with a non-None
        default would silently change the meaning of every policy that omits
        it. Since 2026-11-02 every schema change, an optional one included,
        also needs an IR_SCHEMA_VERSION bump (the grant's freeze rule); the
        None-omission is what keeps such a bump from moving stored hashes.

        Defaults that are NOT None stay in the canonical form on purpose: if
        `margin_m` defaulted to 2.0 tomorrow, a policy that omits it would
        fly differently, and its hash must say so.

        A geographic point is stored as {lat, lon}: its x/y are a function of
        the frame, derived at load, and the grant forbids the DSL to carry
        projected coordinates (hash scheme v3, 2026-10-06). A metre point is
        stored as {x, y}, so every metre policy's bytes are what they were.
        """
        # _STORED: a WGS84 point dumps as lat/lon only (XY._serialise), so no
        # point is projected to be hashed. (Until 3.1.0 the dump carried x/y
        # and a walk, _geo_canonical, dropped them again: it doubled the
        # 50-rule hash time, 0.26 -> 0.52 ms.)
        return self.model_dump(mode="json", exclude_none=True, context=_STORED)

    def canonical_bytes(self) -> bytes:
        """The exact bytes `policy_hash` digests - and the bytes a bundle ships
        as `ir.json`, so a reader can check the hash with sha256sum alone."""
        return canonical_json(self.canonical_ir())

    @property
    def ir_schema_version(self) -> str:
        """The IR schema version this document is a document of:
        IR_SCHEMA_VERSION, or LAST_SCOPE_OPTIONAL for a policy read from an
        earlier era's document (era_context) whose rules leave `scope` out -
        what its canonical bytes conform to. A bundle's manifest records it
        (guardrail.bundle.manifest_for), so a reader reads the IR in the era it
        belongs to: recording the writer's version instead would make a
        re-bundled earlier policy (a replay of an old run) unreadable."""
        if any(c.scope is None for c in self.constraints):
            return LAST_SCOPE_OPTIONAL
        return IR_SCHEMA_VERSION

    @property
    def policy_hash(self) -> str:
        """SHA-256 over the canonicalised IR, all 64 hex digits.

        The grant (Policy DSL, "Versioning and signing"): "policy_hash - SHA-256
        over the canonicalised IR (post-merge, pre-sign)", and the reference
        implementation keeps the full digest (vlaguard_common/hashing.py). This
        one was cut to 16 hex until 2026-10-06; `legacy_hashes()` still
        reproduces every shortened form a stored run may carry.

        Every audit record, manifest, CSP and bundle carries this string."""
        return "sha256:" + hashlib.sha256(self.canonical_bytes()).hexdigest()

    @property
    def policy_hash_short(self) -> str:
        """The 16-hex prefix of `policy_hash`, for display and for old artefacts.

        For a metre policy and every schema era up to 2026-08-31 this IS the
        hash those runs recorded: the old 16-hex digest of the None-free dump
        and the new 64-hex digest are the same SHA-256 over the same bytes, one
        truncated. A geographic policy's old short hash is the prefix of its
        `prior_hashes()` entry instead (the v2 form held projected metres).
        """
        return self.policy_hash[:len("sha256:") + 16]

    def legacy_hashes(self) -> dict[str, str]:
        """Every SHORTENED hash an older version of this file would have
        produced for this policy, keyed by the form's name.

        The old hash digested `model_dump()`, i.e. "every field the schema had
        THAT DAY", with None written as null. So the bytes depended on which
        optional fields existed when the run flew, and each schema era has its
        own form. `_LEGACY_ERAS` lists them; adding an era is the only way to
        make another historical form verifiable, and guessing one is not. They
        are computed over the projected view (points as x/y), because that is
        what those eras stored.
        """
        return {name: _legacy16(self, keep_null)
                for name, keep_null in _LEGACY_ERAS.items()}

    def prior_hashes(self) -> dict[str, str]:
        """Full-length hashes of earlier canonical schemes that DIFFER from the
        current one for this policy, keyed by scheme name.

        Only a geographic policy has one: `sha256-canonical-v2` (2026-10-06,
        before this change) hashed the projected metres. For a metre policy v2
        and v3 are the same bytes, so nothing is listed."""
        if not self.is_geographic:
            return {}
        # v2 was one day's scheme (2026-10-06), so its IR is the 2.x shape.
        dump = _metric_view(self._ir2_dump(mode="json", exclude_none=True,
                                           context=_ERA))
        return {PRIOR_SCHEME_V2: "sha256:" + hashlib.sha256(canonical_json(dump)).hexdigest()}

    def _ir2_dump(self, *, mode: str = "json", exclude_none: bool = True,
                  repair: bool = True, context: dict | None = None) -> dict:
        """This policy's dump as IR schema 2.x stored it: every polygon ring in
        the order it was GIVEN (3.0.0 stores it canonically) and, with
        `repair`, the grant's project_fix spelt `repair` (3.0.0 stores only
        the grant's six). Every form recorded before 2026-11-02 digests this
        view, not the current IR. `context` is the points' serialisation
        context (_STORED, _ERA; XY._serialise)."""
        d = self.model_dump(mode=mode, exclude_none=exclude_none, context=context)
        for c, rd in zip(self.constraints, d["constraints"]):
            if isinstance(c, _CanonicalRing) and "vertices" in rd:
                rd["vertices"] = [v.model_dump(mode=mode, exclude_none=exclude_none,
                                               context=context)
                                  for v in c.given_vertices()]
            if repair and rd.get("violation_action") == "project_fix":
                rd["violation_action"] = "repair"
        return d

    def ir2_hashes(self) -> dict[str, str]:
        """The full-length hashes an IR schema 2.x artefact recorded for this
        policy, where they differ from the current one, keyed by form.

        IR 3.0.0 (2026-11-02) moved the canonical bytes two ways: `repair` is
        stored as project_fix, and polygon rings in canonical order. A run,
        bundle or lock entry from before carries the hash of the old bytes.
        Two forms are reproduced: IR2_FORM, every project_fix spelt `repair`
        (what every policy of this repository wrote, and what the old default
        meant), and IR2_FORM_AS_WRITTEN, project_fix kept (a grant-form
        document that wrote it). A 2.x document that mixed both spellings in
        one policy matches neither; none is known. Both use the ring order
        the policy was given, so they reproduce the old hash for a policy
        loaded from the old file or the old artefact, not for one rebuilt
        from a 3.0.0 IR (whose given order is the canonical one).

        Kept apart from `prior_hashes` on purpose: the lock treats a prior
        SCHEME's hash as the same content under a newer encoding, but these
        are another IR, so a policy file still at its old version is reported
        as changed and must raise it (the lock's `migrated_from`)."""
        cur = self.policy_hash
        out: dict[str, str] = {}
        for name, rep in ((IR2_FORM, True), (IR2_FORM_AS_WRITTEN, False)):
            dump = self._ir2_dump(mode="json", exclude_none=True, repair=rep,
                                  context=_STORED)
            h = "sha256:" + hashlib.sha256(canonical_json(dump)).hexdigest()
            if h != cur and h not in out.values():
                out[name] = h
        return out

    def metre_form(self) -> "Policy":
        """This policy as it reads written in local metres about its frame
        origin, with no `origin`: every WGS84 point replaced by its local view
        {x, y} (micrometre-rounded; XY), every ring in the order it was given,
        everything else unchanged.

        It is the form every shipped policy was written in until 2026-11-03.
        tools/convert_policies_wgs84.py wrote each one's lat/lon from its metres
        about a stated origin, and proves for each that the metre form of the
        converted file IS the old file's IR - the same canonical bytes, so the
        same policy_hash, legacy forms and IR 2.x forms. The lock reads an old
        version back through it (bundle.rewind; `migrated_from` with the
        frame migration), so a run flown under the metre version still
        resolves to the file it flew. A policy with an `origin` and no point
        at all (a stand-off or clearance policy) has the form without the
        origin. Raises for a policy with neither: it is in metres already."""
        if not self.is_geographic and self.origin is None:
            raise ValueError(f"{self.policy_id}: no WGS84 point and no origin; the "
                             f"policy is written in metres already")
        d = self._ir2_dump(mode="json", exclude_none=True, repair=False)
        d.pop("origin", None)
        # A document of IR schema 3.0.0 (or earlier): a rule that left `scope`
        # out keeps it absent (era_context); one that states it keeps it.
        return Policy.model_validate(_metric_view(d), context=era_context("3.0.0"))

    def ir3_form(self) -> "Policy":
        """This policy as IR schema 3.x held it with `scope` left out: every
        rule's scope absent (None), nothing else changed - the rules keep the
        ring order they were given, their points the frame they are bound to.

        Every shipped policy, and every rule built in code, left scope out
        until IR schema 4.0.0 (2026-11-06), when the shipped files stated it
        and raised their patch version. Put back at the old version
        (bundle.as_version), this form of the new file IS the old file's IR -
        the same canonical bytes, so the same policy_hash, IR 2.x hashes and
        legacy forms - which is how the lock reads the old version back
        (bundle.rewind, bundle.IR3_FORM) and how a run flown under it still
        resolves. It reconstructs the document the old version was; an
        absent scope meant "mission" then (DEFAULT_SCOPE)."""
        rules = [c.model_copy(update={"scope": None}) for c in self.constraints]
        return self.model_copy(update={"constraints": rules})

    def hash_form(self, recorded: str | None) -> str | None:
        """Which identity form `recorded` is for THIS policy, or None.

        Returns HASH_SCHEME for the current 64-hex hash, the scheme name of a
        prior full-length form, an IR 2.x form (`ir2_hashes`), the name of the
        legacy 16-hex form it matches, or REFERENCE_FORM for the reference
        implementation's hash of the same document (its rings in canonical
        order, or as given). None means the recorded hash is not this policy
        under any form we know - the caller must refuse, not guess.
        """
        if not recorded:
            return None
        if recorded == self.policy_hash:
            return HASH_SCHEME
        if recorded == self.policy_hash_short:
            # The display form, pinned by hand (a scenario's expected_hash).
            # Until IR 3.0.0 it equalled legacy16-exclude-none for every metre
            # policy and was recognised as that; the IR 2.x view now differs.
            return SHORT_FORM
        for name, h in self.prior_hashes().items():
            if recorded == h:
                return name
        for name, h in self.ir2_hashes().items():
            if recorded == h:
                return name
        for name, h in self.legacy_hashes().items():
            if recorded == h:
                return name
        if recorded in (self.reference_hash(), self.reference_hash(given=True)):
            return REFERENCE_FORM
        return None

    def matches_hash(self, recorded: str | None) -> bool:
        """True when `recorded` identifies this policy under any known form."""
        return self.hash_form(recorded) is not None

    def by_type(self, cls) -> list:
        return [c for c in self.constraints if isinstance(c, cls)]

    def static_keepout_zones(self) -> list:
        """The keep-out zones that stay where they are declared, in policy
        order: every polygon_fence and circle_fence (by_type(PolygonFence))
        and every dynamic_nfz without motion (DynamicNFZ.moves). Each has
        `vertices`, the altitude band and `margin_m`, so geometry.fence_polygon
        and point_in_fence take it as they take a polygon fence. The readers
        that took polygon fences only (offline analysis, the BC features, the
        prompt's no_fly_zones) read this since 2026-11-15 (C-16); a moving
        zone is not here: it has no one place (FLIGHT_HELD_TYPES)."""
        return [c for c in self.constraints
                if isinstance(c, PolygonFence)
                or (isinstance(c, DynamicNFZ) and not c.moves)]

    # ------------------------------------------------------------------ #
    # The reference implementation's form (cross-loading, WP1-05)
    # ------------------------------------------------------------------ #

    def reference_problems(self) -> list[str]:
        """Why this policy cannot be written in the reference implementation's
        document form (kuanting-vla-uav-guardrail packages/policy-dsl, its
        Phase-1 slice), or [] when it can."""
        why: list[str] = []
        if self.origin is not None:
            why.append("`origin`: the reference derives its frame and has no such field")
        if self.layers_merged is not None:
            why.append("`layers_merged`: not a field of the reference's PolicyDoc")
        if not self.constraints:
            why.append("no rules: the reference requires at least one")
        for c in self.constraints:
            if c.type == "polygon_fence":
                if not all(v.geographic for v in c.vertices):
                    why.append(f"{c.id}: metre vertices (the reference stores WGS84 only)")
                if c.margin_m != 0.0:
                    why.append(f"{c.id}: margin_m {c.margin_m} (the reference has no margin ring)")
            elif c.type != "altitude_envelope":
                why.append(f"{c.id}: {c.type} is not in the reference's DSL "
                           f"(polygon_fence, altitude_envelope)")
            if c.valid_time is not None:
                why.append(f"{c.id}: valid_time (the reference model has no such "
                           f"field and would drop it without a word)")
            if c.locked_by is not None:
                why.append(f"{c.id}: locked_by (layer-merge provenance; not a field "
                           f"of the reference)")
        return why

    def reference_ir(self, given: bool = False) -> dict | None:
        """This policy exactly as the reference's `PolicyDoc.model_dump(mode=
        "json")` would hold it - the bytes its bundle ships and its hash covers -
        or None when the reference cannot represent it (`reference_problems`).

        Absent scope/layer/altitude_ref are written as the reference's defaults
        ("mission", "mission", "AGL"), because that is what the reference
        stores for them (scope is absent only in a rule of a document from
        before IR schema 4.0.0). Rings are written in their canonical order (the IR's,
        so the document is the same before and after a bundle); `given=True`
        writes them in the order this policy was given them, which is what
        the reference hashed for a document it read itself."""
        if self.reference_problems():
            return None
        rules = []
        for c in self.constraints:
            r = {"id": c.id, "type": c.type, "constraint_type": c.constraint_type,
                 "scope": c.effective_scope, "priority": c.priority,
                 "layer": c.effective_layer,
                 "violation_action": canonical_action(c.violation_action)}
            if c.type == "polygon_fence":
                ring = c.given_vertices() if given else c.vertices
                r["geometry"] = {"vertices": [{"lat": v.lat, "lon": v.lon}
                                              for v in ring],
                                 "altitude_floor_m": c.altitude_floor_m,
                                 "altitude_ceiling_m": c.altitude_ceiling_m,
                                 "altitude_ref": c.altitude_ref or "AGL"}
            else:
                r.update(altitude_min_m=c.alt_min_m, altitude_max_m=c.alt_max_m,
                         altitude_ref=c.altitude_ref or "AGL")
            rules.append(r)
        return {"policy_id": self.policy_id, "version": self.version,
                "generation": self.generation, "issued_at": self.issued_at,
                "constraints": rules}

    def reference_hash(self, given: bool = False) -> str | None:
        """The reference implementation's policy_hash for this document: its
        `policy_hash(doc.model_dump(mode="json"))` (policy_dsl/ir.py build_ir,
        vlaguard_common/hashing.py canonicalize) over the authored document as
        its PolicyDoc holds it - grant field names, `geometry:` blocks, every
        default written out, `issued_at: null` kept. None when the reference
        cannot hold the document at all (`reference_problems`); a hash of a
        document its loader would refuse is a number nobody can check, so none
        is invented. Checked against the reference's own loader, in its own
        3.11 environment, by tools/wp1_roundtrip_kpi.py. `given`: see
        reference_ir."""
        ref = self.reference_ir(given=given)
        if ref is None:
            return None
        return "sha256:" + hashlib.sha256(canonical_json(ref)).hexdigest()

    def reference_delta(self) -> dict | None:
        """What `reference_ir()` writes that this policy does not say, so that
        this policy is rebuilt EXACTLY - same `policy_hash` - from the
        reference's document plus this record (`from_reference`). None when the
        reference cannot represent the policy.

        The reference's form is lossy in two ways only, both about defaults
        (everything else it cannot hold is refused by `reference_problems`):
        an absent `scope` / `layer` / `altitude_ref` is written as "mission" /
        "mission" / "AGL" (since IR schema 4.0.0 a rule always states its
        scope, so only a policy of an earlier era's document has an absent
        one to record), and `repair` is spelt `project_fix`. A
        reference-form bundle ships this record in its manifest
        (guardrail/bundle.py), so the PI's loader verifies the reference hash
        and this project's loader verifies BOTH that and this project's hash of
        the policy it rebuilds (card WP1-05, deviation 10)."""
        if self.reference_problems():
            return None
        rules: dict[str, dict] = {}
        for c in self.constraints:
            d: dict[str, Any] = {}
            absent = [k for k in _REFERENCE_DEFAULTED
                      if k in type(c).model_fields and getattr(c, k) is None]
            if absent:
                d["absent"] = absent
            if c.violation_action != canonical_action(c.violation_action):
                d["violation_action"] = c.violation_action
            if d:
                rules[c.id] = d
        return {"rules": rules}

    @classmethod
    def from_reference(cls, doc: dict, delta: dict | None, *,
                       context: dict | None = None) -> "Policy":
        """The policy a reference-form document plus its `reference_delta`
        stands for. Refuses a delta that does not round-trip: the rebuilt
        policy must write back exactly `doc` (so a delta can only restore an
        absent default or a spelling, never change what a rule means).

        `context`: the era of the bundle the delta came from (era_context of
        its manifest's ir_schema_version). A delta written before IR schema
        4.0.0 may restore `scope` as absent; since 4.0.0 no policy leaves it
        out, so no delta says so, and one that does is refused."""
        import copy
        rules = dict((delta or {}).get("rules") or {})
        out = copy.deepcopy(doc)
        by_id = {r.get("id"): r for r in out.get("constraints") or []
                 if isinstance(r, dict)}
        unknown = sorted(set(rules) - set(by_id))
        if unknown:
            raise ValueError(f"reference delta names rules the document does not "
                             f"have: {unknown}")
        for rid, d in rules.items():
            r = by_id[rid]
            for k in d.get("absent") or []:
                if k not in _REFERENCE_DEFAULTED:
                    raise ValueError(f"{rid}: reference delta may restore only "
                                     f"{list(_REFERENCE_DEFAULTED)} as absent, not {k!r}")
                holder = r["geometry"] if (k == "altitude_ref" and isinstance(
                    r.get("geometry"), dict)) else r
                holder.pop(k, None)
            if "violation_action" in d:
                r["violation_action"] = d["violation_action"]
        pol = cls.model_validate(out, context=context)
        if pol.reference_ir() != canonical_reference_doc(doc):
            raise ValueError("the reference delta does not round-trip: the policy "
                             "it rebuilds does not write back the archived "
                             "reference document")
        return pol

    # ------------------------------------------------------------------ #
    # The DSL's rule-consistency lint (grant ingest pipeline)
    # ------------------------------------------------------------------ #

    def lint(self, resolve_refs: bool = True) -> list[str]:
        """Every problem the grant's "rule consistency lint" stage would name.

        Kept apart from field validation because some callers construct a
        policy deliberately outside these rules (a test fence with a negative
        margin, an empty policy that a hot-apply fills); every DSL entry point -
        load_policy, the layer merge, a bundle read - runs it and refuses on
        any finding. `resolve_refs=False` skips only the check that a switch or
        swap names a rule of THIS document - for one layer of a layered policy,
        whose target may live in another layer; the merge checks the result."""
        out: list[str] = []
        if not self.constraints:
            out.append("the policy has no rules (constraints is empty): it would "
                       "load, hash and pass every flight while enforcing nothing")
        for rid, n in Counter(c.id for c in self.constraints).items():
            if n > 1:
                out.append(f"rule id {rid!r} is used by {n} rules: audit records, "
                           f"KPI priorities and hot-apply address rules by id")
        for c in self.constraints:
            m = getattr(c, "margin_m", 0.0)
            if m < 0:
                out.append(f"{c.id}: margin_m {m} shrinks the zone inward (a "
                           f"negative safety ring can erase it)")
        out += _envelope_conflicts(self)
        out += _geometry_conflicts(self)
        ids = {c.id: c for c in self.constraints}
        for c in self.constraints:
            if isinstance(c, (TimeWindowSwitch, CorridorSwap)):
                tgt = ids.get(c.target_id)
                if tgt is None:
                    if resolve_refs:
                        out.append(f"{c.id}: target_id {c.target_id!r} names no "
                                   f"rule in this policy")
                elif tgt is c:
                    out.append(f"{c.id}: a rule cannot target itself")
                elif isinstance(tgt, (TimeWindowSwitch, CorridorSwap)):
                    out.append(f"{c.id}: targets another {tgt.type} rule; switch "
                               f"or swap a rule, not an event")
                elif isinstance(c, CorridorSwap) and not isinstance(tgt, Corridor):
                    out.append(f"{c.id}: corridor_swap targets {tgt.id!r}, a "
                               f"{tgt.type}, not a corridor")
        if self.layers_merged is not None:
            for c in self.constraints:
                if c.effective_layer not in self.layers_merged:
                    out.append(f"{c.id}: layer {c.effective_layer!r} is not in "
                               f"layers_merged {self.layers_merged}")
        # `locked_by` is merge provenance: merge_layers writes it, on a merged
        # policy (`layers_merged` set), and only as a non-empty list. Until the
        # 2026-10-19 review a hand-written value on an unmerged policy was
        # accepted: it moved policy_hash and made the rule unswitchable from
        # every layer, its own included, while the Shield's refusal spoke of
        # "overriding a hard regulation-layer copy" that no file holds; and
        # `locked_by: []` moved the hash and did nothing at all.
        for c in self.constraints:
            if c.locked_by is None:
                continue
            if self.layers_merged is None:
                out.append(f"{c.id}: locked_by {c.locked_by} on a policy no layer "
                           f"merge produced (no layers_merged): it is the "
                           f"provenance merge_layers records, and written by hand "
                           f"it locks the rule against every layer's event and "
                           f"names an overridden copy that does not exist")
            elif not c.locked_by:
                out.append(f"{c.id}: locked_by [] says nothing (absent means no "
                           f"lower hard copy was overridden) but changes policy_hash")
        return out

    # ------------------------------------------------------------------ #
    # What the runtime can and cannot do with this policy
    # ------------------------------------------------------------------ #

    def unenforced_rules(self, ground=None) -> list[str]:
        """Rules the Safety Shield would NOT enforce, with the reason. A flight
        loader refuses a policy for which this is non-empty.

        An MSL height (X-11) is an OPT-IN: enforced since 2026-11-03 when the
        caller names the ground it flies over (`ground`, guardrail/terrain.py
        FlatGround - the rail's stated elevation, or since 2026-11-17 a
        GridGround DEM) and passes the same to the Shield; refused, as
        before, when it names none."""
        out: list[str] = []
        for c in self.constraints:
            if c.type not in RUNTIME_TYPES:
                out.append(f"{c.id}: {c.type} is declarable in the DSL but the "
                           f"Safety Shield does not enforce it")
            if getattr(c, "altitude_ref", None) == "MSL":
                from .terrain import ground_problem
                why = ground_problem(ground)
                if why:
                    out.append(f"{c.id}: altitude_ref MSL - {why}")
        if ground is not None:
            # A DEM is placed in one frame: refused under a policy whose
            # frame origin differs (terrain.origin_problem, 2026-11-17).
            from .terrain import origin_problem
            why = origin_problem(ground, self.frame_origin)
            if why:
                out.append(f"ground: {why}")
        return out

    def frame_problem(self) -> str | None:
        """Why the flight rails cannot place this policy's frame, or None.

        The ArduPilot SITL rail measures the vehicle in metres about the
        autopilot's home (the EKF origin) - its start - and its MAVLink
        adapter (sitl/mavlink_adapter_node.py) will not upload the zones of a
        policy whose `origin` is not at that home: the policy frame's (0, 0)
        IS the take-off point there. A
        grant-form policy states no origin; its frame is DERIVED from its
        geometry (the reference's rule), so (0, 0) is the centroid of its first
        fence, not the take-off point. Flying it would put every zone in the
        wrong place relative to the vehicle, and nothing would fail. The DSL
        loads it (runtime=False); flight needs the take-off point stated.

        What this gate guarantees is a STATED frame, not a correct one. It
        cannot know where the vehicle really is, and an `origin` written
        inside a keep-out zone passes it (policies/wgs84_taipei.yaml states
        the school yard's own centroid). `start_conflicts` names that case and
        flights record it; it is not refused (see start_conflicts)."""
        if self.origin is not None or not self.is_geographic:
            return None
        lat, lon = self.frame_origin
        return (f"the local frame is derived from the geometry (origin "
                f"{lat:.7f}, {lon:.7f}, the reference's rule), but the SITL rail "
                f"places the vehicle's start (home) at the frame origin. State "
                f"`origin: {{lat, lon}}` = the take-off position to fly it")

    def unmeasured_rules(self) -> list[str]:
        """Rules the Shield enforces but the flight rails would not MEASURE,
        with the reason (FLIGHT_HELD_TYPES; `flight_held`: a dynamic_nfz only
        while it moves, since 2026-11-15). A flight loader refuses a policy
        file for which this is non-empty: a dwell metric that cannot see a
        zone reads 0 s, which is indistinguishable from a clean flight."""
        return [f"{c.id}: {c.type} is enforced by the Shield, but "
                f"{FLIGHT_HELD_TYPES[c.type]}"
                for c in self.constraints if flight_held(c)]

    def flight_problems(self, ground=None) -> list[str]:
        """Everything that keeps this policy off a flight rail: unenforced
        rules, rules the rails would not measure, and an unplaceable frame.
        Empty = flyable as written. `ground`: see unenforced_rules."""
        frame = self.frame_problem()
        return (self.unenforced_rules(ground) + self.unmeasured_rules()
                + ([frame] if frame else []))

    def start_conflicts(self) -> list[str]:
        """Hard keep-out zones whose horizontal footprint (the polygon grown by
        its margin ring) holds the frame origin (0, 0) - with the reason, or
        [] when none does.

        The frame origin is the take-off point on the ArduPilot SITL rail
        (home; see frame_problem) and the point a geographic policy's `origin`
        states. It is NOT the start on every rail: the CityLife (classic
        AirSim) scenarios spawn at (35, -20) in their frame
        (demo/gen_random_scenario.py), and six of the generated policies in
        policies/ have a zone over (0, 0) while keeping clear of that spawn.
        So the note says "a vehicle that starts at the frame origin", and it
        is recorded, never refused.

        Conservative: schedules and altitude bands are ignored, because a
        vehicle that takes off there climbs through every height of the
        column and the clock at take-off is not known here. Pure Python, like
        polygon_problem. The flight loaders record it in `runtime_notes`, and
        tools/geo_to_policy.py refuses a stated take-off point inside a zone
        it creates.

        A dynamic_nfz is checked at its declared position (since 2026-10-08,
        when a policy file carrying one became flyable): a moving zone starts
        there at the Shield's first tick."""
        out = []
        for c in self.constraints:
            if not isinstance(c, (PolygonFence, DynamicNFZ)) or c.constraint_type != "hard":
                continue
            ring = _dedupe_ring([(v.x, v.y) for v in c.vertices])
            if len(ring) < 3:
                continue
            m = max(c.margin_m, 0.0)
            if _point_in_ring(0.0, 0.0, ring) or _dist_to_ring(0.0, 0.0, ring) <= m:
                out.append(
                    f"{c.id}: the frame origin (0, 0) lies inside this hard "
                    f"keep-out zone's footprint (margin {m:g} m, band "
                    f"{c.altitude_floor_m:g}-{c.altitude_ceiling_m:g} m, schedule "
                    f"ignored); a vehicle that starts at the frame origin (the "
                    f"ArduPilot SITL rail: home) starts inside the zone at every "
                    f"height of its band")
        return out

    def runtime_notes(self) -> list[str]:
        """Facts a flight should record beside its results. Not refusals:
        breach actions the Shield does not complete on its own (a mode change
        it can only request, or a soft rule's action capped at brake; see
        MODE_ACTIONS), and a start inside a hard keep-out zone
        (`start_conflicts`)."""
        notes = []
        for c in self.constraints:
            act = c.violation_action
            if act in SHIELD_ACTS_ON:
                continue
            if c.constraint_type == "soft":
                notes.append(f"{c.id}: violation_action {act!r} on a soft rule: the "
                             f"escalation FSM caps a soft rule's response at brake "
                             f"(guardrail/fsm.py), so the Shield stops and requests "
                             f"no mode")
            else:
                notes.append(f"{c.id}: violation_action {act!r}: the Shield skips "
                             f"projection and stops where a stop is legal, and its "
                             f"escalation FSM requests {act}; the mode itself is "
                             f"flown only on a rail that passes ShieldDecision."
                             f"set_mode to the autopilot")
        return notes + self.start_conflicts()

    # ------------------------------------------------------------------ #
    # Producing the grant's own form
    # ------------------------------------------------------------------ #

    def to_grant_form(self, origin: tuple[float, float] | None = None) -> dict:
        """This policy as a grant-form DSL document (Policy DSL page): WGS84
        points, `geometry:` blocks, the grant's field names and spellings, and
        every ConstraintBase field written out.

        Loading the result gives the same rules. Fields the grant has no place
        for are kept so that is true, and `grant_form_extensions()` lists them:
        `origin`, a non-zero `margin_m`, a merged rule's `locked_by`, and this
        project's two own rule types.

        A metre point needs a WGS84 anchor for the frame's (0, 0): the policy's
        own `origin`, else the `origin` argument; with neither this raises,
        because inventing an anchor would put the policy somewhere plausible and
        wrong. The anchor used is written as `origin`, so the produced document
        loads into the same local frame."""
        anchor = self.frame_origin
        has_metric = any(not v.geographic for pts in _rule_points(self, all_points=True)
                         for v in pts)
        if has_metric:
            if self.origin is None and origin is not None:
                anchor = (float(origin[0]), float(origin[1]))
            elif self.origin is None:
                raise ValueError(
                    f"{self.policy_id} is written in local metres and states no "
                    f"origin; the grant form is WGS84. Pass origin=(lat, lon): "
                    f"the WGS84 position of the local frame's (0, 0)")
        proj = LocalProjection(*anchor) if anchor is not None else None

        def ll(v: XY) -> dict:
            if v.geographic:
                return {"lat": v.lat, "lon": v.lon}
            lat, lon = proj.to_latlon(v.x, v.y)
            return {"lat": lat, "lon": lon}

        doc: dict[str, Any] = {"policy_id": self.policy_id, "version": self.version,
                               "generation": self.generation}
        if self.issued_at is not None:
            doc["issued_at"] = self.issued_at
        if self.layers_merged is not None:
            doc["layers_merged"] = list(self.layers_merged)
        if self.origin is not None or has_metric:
            doc["origin"] = {"lat": anchor[0], "lon": anchor[1]}
        doc["constraints"] = [_grant_rule(c, ll) for c in self.constraints]
        return doc

    def grant_form_extensions(self) -> list[str]:
        """What `to_grant_form` keeps that the grant's form does not define."""
        out = []
        if self.origin is not None:
            out.append("origin")
        for c in self.constraints:
            if getattr(c, "margin_m", 0.0) not in (0.0, None):
                out.append(f"{c.id}.margin_m")
            # Layer-merge provenance (ConstraintBase.locked_by): _grant_rule
            # writes it, the grant's form has no such field. Missing from this
            # list until the 2026-10-19 review.
            if c.locked_by is not None:
                out.append(f"{c.id}.locked_by")
            if c.type not in GRANT_TYPES:
                out.append(f"{c.id}: type {c.type}")
        return out


# --------------------------------------------------------------------------- #
# Point views
# --------------------------------------------------------------------------- #

def _rule_points(policy: Policy, all_points: bool = False):
    """Per rule, its point collection (vertices, centerline, or centre). With
    all_points=False only the geographic flags are yielded (for any())."""
    for c in policy.constraints:
        if isinstance(c, CircleFence):
            pts = [c.center]
        elif isinstance(c, (PolygonFence, DynamicNFZ)):
            pts = c.vertices
        elif isinstance(c, (Corridor, CorridorSwap)):
            pts = c.centerline
        else:
            pts = []
        if all_points:
            yield pts
        else:
            for v in pts:
                yield v.geographic


class NoFrameOrigin(ValueError):
    """A rule with points offered to a policy that has no WGS84 frame origin
    (WP1-14, 2026-11-18): a policy written only in metres (the legacy read
    form) or one with no point and no stated `origin`. Its points cannot be
    written in the canonical WGS84 form, and the grant's DSL never carries
    projected coordinates, so the write is refused rather than stored in
    metres. A ValueError, so every writer that already turns a ValueError
    into a refusal (Shield._in_frame: HotApplyRefused, an HTTP 422 on the
    REST endpoint) reports it as one."""


def rule_has_points(rule) -> bool:
    """True for a rule class that places points (a fence, a dynamic zone, a
    corridor or corridor swap); False for envelopes, switches and the rest."""
    return isinstance(rule, (CircleFence, PolygonFence, DynamicNFZ, Corridor,
                             CorridorSwap))


def no_frame_message(what: str) -> str:
    """The one wording every writer refuses a frameless write with."""
    return (f"{what}: the policy it goes into has no WGS84 frame origin (no "
            f"stated `origin` and no point in lat/lon), so its points cannot be "
            f"written in the canonical WGS84 form, and a policy may not carry "
            f"projected coordinates (grant, Policy DSL page; WP1-14). State "
            f"`origin: {{lat, lon}}` in the policy, or fly its WGS84 successor "
            f"bundle (v0.1.2 or later; tools/bundle_census.py lists them); the "
            f"local-metre form is a read form only")


def rule_in_wgs84(rule, origin: tuple[float, float] | None):
    """`rule` with every metre point written in WGS84 about `origin` ((lat,
    lon), the frame origin of the policy the rule goes into) and every point
    bound to that frame.

    No frame (`origin` None: a policy written only in metres, the legacy
    read form, or one with no point and no stated origin): a rule that
    places points raises NoFrameOrigin, since there is no WGS84 position
    for a metre point and a lat/lon point would silently become the frame
    the policy's metres are read in; a rule without points (an envelope, a
    switch) comes back unchanged. Until 2026-11-18 a rule with points came
    back unchanged too, so an event on a metre-only policy stored metres.

    WHY (WP1-14, 2026-11-17). Since 2026-11-03 every stored policy is WGS84
    with a stated origin, but the REST hot-apply endpoint, the Shield's own
    zone edits (move / translate / rotate / scale compute a new metre ring)
    and the harness's spawned zones still WROTE metre points into the live
    policy, so a generation's snapshot - what is stored, hashed and signed -
    carried projected coordinates, against "the DSL itself never carries
    projected coordinates" (grant, Policy DSL page). The Shield now passes
    every rule an event writes through this (Shield._in_frame); so do
    bundle.derive (a lock recipe on a base with a frame) and, in raw form,
    tools/geo_to_policy.py ingest (projection.raw_points_in_wgs84).

    WHAT IS STORED: lat/lon only (the canonical IR drops any metre view), so
    the snapshot and its hash carry no projected coordinate.

    WHAT IS ENFORCED is the projection of what is stored: every point of the
    rule, converted or already lat/lon, is bound to the frame here, so its
    metres are computed lazily from its lat/lon (the micrometre view,
    projection.local_metres) - the same floats a reload of the generation's
    snapshot reads, bit for bit, under the same hash. That removes one
    obstacle to replaying a hot-applied run bit for bit from its stored
    policy_g<N>.json (WP4-11 is still PARTIAL: no flown run has been
    replayed at its revision). Metres given beside lat/lon are checked
    against the frame (1 um) and dropped, as at any load (XY._bind): a rule
    whose x/y disagree with its lat/lon by more than that raises ValueError
    (Shield._in_frame: HotApplyRefused), where until 2026-11-17 evening a
    rule given wholly in lat/lon came back unchanged and its given x/y were
    the live view. For a point on the
    micrometre grid (any value with at most six decimals: every REST body
    and hand-written scenario) the view is exactly the metres given; for a
    computed one (a spawn at an off-grid vehicle position, rotate, scale) it
    is within 0.6 um of them (the micrometre rounding plus the 12th decimal
    of a degree).

    Until the evening of 2026-11-17 the Shield's default (integrate/1117
    only) left converted points UNBOUND, keeping the metres given as their
    live view, so the snapshot, reloaded, enforced up to 0.6 um elsewhere
    under the same hash; binding was opt-in. Making it the only mode moved
    computed rings by up to 0.6 um, which the Shield reacts to at boundary
    ties (an exact tie between a dynamic zone's edges is judged at that
    resolution: shield._TIE_BOUND_M); what that moved in the smoke profile,
    the library and the sampled variants is
    docs/data/wgs84_bind_default_20261117/.

    Each converted point is also checked against the frame: written lat/lon
    that do not project back within CONSISTENCY_TOL_M (1 um) of the metres
    given raise ValueError and are never stored. This is a guard on
    metres_to_wgs84 itself; for any point inside the lat/lon range (which
    XY's own range check enforces first) the 12-decimal round trip is far
    inside it, so it is not expected to fire. Vertices are taken in the
    order they were GIVEN (`given_vertices`) and canonicalised again, as at
    any load - unless they are no longer the stored vertices (a ring
    replaced after validation), when the stored ones are taken."""
    if origin is None:
        if rule_has_points(rule):
            raise NoFrameOrigin(no_frame_message(
                f"{getattr(rule, 'type', type(rule).__name__)} "
                f"{getattr(rule, 'id', '?')!r}"))
        return rule
    fo = (float(origin[0]), float(origin[1]))
    if isinstance(rule, CircleFence):
        fields = {"center": [rule.center]}
    elif isinstance(rule, (PolygonFence, DynamicNFZ)):
        verts = rule.given_vertices()
        given = sorted(_stored_pair(v) for v in verts)
        if given != sorted(_stored_pair(v) for v in rule.vertices):
            verts = list(rule.vertices)
        fields = {"vertices": verts}
    elif isinstance(rule, (Corridor, CorridorSwap)):
        fields = {"centerline": list(rule.centerline)}
    else:
        return rule
    proj = projection_for(*fo)
    data = rule.model_dump(mode="json", exclude_none=True)
    for name, pts in fields.items():
        out = []
        for v in pts:
            if v.geographic:
                # (a rule is all lat/lon or all metres: check_ring refuses a
                # mix; such a point is kept as it was, and bound below)
                out.append(v.model_dump(mode="json", exclude_none=True))
                continue
            x, y = v.x, v.y
            lat, lon = metres_to_wgs84(fo, x, y)
            n, e = proj.to_local(lat, lon)
            if abs(n - x) > CONSISTENCY_TOL_M or abs(e - y) > CONSISTENCY_TOL_M:
                raise ValueError(
                    f"{rule.id}: the metre point ({x}, {y}) written as WGS84 "
                    f"({lat}, {lon}) about {fo} projects back to ({n}, {e}); more "
                    f"than {CONSISTENCY_TOL_M} m away, so it is not stored")
            out.append({"lat": lat, "lon": lon, "x": x, "y": y})
        data[name] = out[0] if name == "center" else out
    ctx = dict(era_context(None)) if getattr(rule, "scope", None) is None else {}
    ctx["frame"] = fo                    # XY._pair binds every point: XY._bind
    return type(rule).model_validate(data, context=ctx)


# --------------------------------------------------------------------------- #
# WGS84-only entry (WP1-14, 2026-11-26)
# --------------------------------------------------------------------------- #
#
# The grant (Policy DSL page): "WGS84 latitude / longitude is canonical ...
# ENU / NED projection is computed lazily by the Prefix Compiler and Safety
# Shield from the WGS84 + AGL fields - the DSL itself never carries projected
# coordinates". Until 2026-11-26 every entry still ACCEPTED an absolute
# position in local metres and converted it at ingest (deviation 1 of the
# final report): load_policy / policy_from_raw read a metre document, the REST
# hot-apply bodies took {x, y} vertices, and the Shield's spawn / move / swap /
# pre-start add took metre rules. Since then every entry takes an absolute
# position in WGS84 lat/lon only and refuses metres (ProjectedCoordinatesRefused
# for a document; Shield HotApplyRefused, HTTP 422 over REST, for an event),
# naming the one tool-side conversion below. A RELATIVE edit stays in metres,
# as in the grant's own scenario events (stress-testing page: width_m,
# height_m, ahead_of_vehicle_m, mps): translate (dx, dy), rotate, scale, and
# the rings the Shield computes from them.
#
# Our own callers that work in local metres (the stress harness's scenario
# events, the harness's REST forwards, tools, demos) convert tool-side through
# `wgs84_from_metres`, about the frame origin of the policy they write into
# (Policy.frame_origin; GET /policy `origin` over REST), before they call.
# The legacy metre READ of a document is `load_legacy_metre_policy` /
# `legacy_metre_policy_from_raw`, used only by the declared re-reads and the
# converters that write WGS84 from it: bundle.load_for_flight legacy_read
# (VLAGUARD_LEGACY_READ=1), tools/bundle_census.py, tools/convert_policies_
# wgs84.py, the bundle export CLI (WGS84 about --origin) and demo/gui_policy.py's
# base read. A run's stored generations (guardrail/replay.py,
# tools/replay_at_revision.py) and a bundle's IR are read through the IR
# (Policy.model_validate) as before; a lock recipe through bundle.derive
# legacy_read.

TOOL_SIDE_CONVERSION = "guardrail.models.wgs84_from_metres"


class ProjectedCoordinatesRefused(ValueError):
    """An absolute position given in local metres (projected coordinates) at a
    DSL, REST, gRPC or in-process entry, which takes WGS84 lat/lon only
    (WP1-14, 2026-11-26). `policy` is the validated policy when a whole
    document was refused (so a caller can name it), else None."""

    def __init__(self, message: str, policy: "Policy | None" = None):
        super().__init__(message)
        self.policy = policy


def projected_input_message(what: str) -> str:
    """The one wording every entry refuses a metre position with."""
    return (f"{what}: an absolute position is given in local metres ({{x, y}}). "
            f"The DSL never carries projected coordinates (grant, Policy DSL page; "
            f"WP1-14): give every vertex / centre / centerline point as "
            f"{{lat, lon}}. A tool that works in metres converts them itself, "
            f"before it sends them, with {TOOL_SIDE_CONVERSION}(<rule or body>, "
            f"<the policy's frame origin>) (the policy's `origin`, GET /policy "
            f"over REST); a whole metre policy file with "
            f"tools/convert_policies_wgs84.py. Relative edits (translate dx/dy, "
            f"rotate, scale) stay in metres")


def rule_metre_points(rule) -> int:
    """How many of `rule`'s points are given in local metres only (no
    lat/lon): 0 for a rule in WGS84 or without points."""
    if isinstance(rule, CircleFence):
        pts = [rule.center]
    elif isinstance(rule, (PolygonFence, DynamicNFZ)):
        pts = rule.vertices
    elif isinstance(rule, (Corridor, CorridorSwap)):
        pts = rule.centerline
    else:
        return 0
    return sum(1 for v in pts if not v.geographic)


def refuse_metre_rule(rule, what: str | None = None) -> None:
    """Raise ProjectedCoordinatesRefused when `rule` gives a point in metres."""
    if rule_metre_points(rule):
        raise ProjectedCoordinatesRefused(projected_input_message(
            what or f"{getattr(rule, 'type', type(rule).__name__)} "
                    f"{getattr(rule, 'id', '?')!r}"))


def wgs84_from_metres(obj, origin: tuple[float, float] | None, *,
                      what: str = "input"):
    """THE tool-side conversion (WP1-14, 2026-11-26): `obj` with every
    absolute metre point (x North, y East of `origin`) written as WGS84
    lat/lon about `origin`, the frame origin of the policy it goes into
    (Policy.frame_origin; GET /policy `origin` over REST).

    `obj` is a rule (a model: models.rule_in_wgs84, which also binds it to
    the frame) or raw JSON - a rule or REST body dict, a list of point dicts
    (the raw twin, as projection.raw_points_in_wgs84 converts it). Every
    point goes through projection.metres_to_wgs84, the conversion the Shield
    and the REST endpoint applied at ingest until 2026-11-26, so a caller
    that converts here writes the same lat/lon, bit for bit, as the ingest
    wrote; and each converted point is checked to project back within
    CONSISTENCY_TOL_M (1 um) of the metres given, as rule_in_wgs84 checks
    it, else ValueError. Points already in lat/lon are kept.

    `origin` None (a policy with no WGS84 frame) and a metre point to
    convert: NoFrameOrigin, as every writer refuses it."""
    is_model = isinstance(obj, BaseModel)
    if origin is None:
        n = rule_metre_points(obj) if is_model else _raw_metre_points(obj)
        if n:
            raise NoFrameOrigin(no_frame_message(what))
        return obj
    if is_model:
        return rule_in_wgs84(obj, origin)
    return _raw_in_wgs84(obj, (float(origin[0]), float(origin[1])))


def _raw_metre_points(node) -> int:
    from .projection import metre_point_count
    return metre_point_count(node)


def _raw_in_wgs84(node, fo: tuple[float, float]):
    """projection.raw_points_in_wgs84 with rule_in_wgs84's 1-um check."""
    from .projection import _is_metric_point, _num
    if isinstance(node, list):
        return [_raw_in_wgs84(v, fo) for v in node]
    if not isinstance(node, dict):
        return node
    if _is_metric_point(node):
        x, y = _num(node["x"], "x"), _num(node["y"], "y")
        lat, lon = metres_to_wgs84(fo, x, y)
        n, e = projection_for(*fo).to_local(lat, lon)
        if abs(n - x) > CONSISTENCY_TOL_M or abs(e - y) > CONSISTENCY_TOL_M:
            raise ValueError(f"the metre point ({x}, {y}) written as WGS84 ({lat}, {lon}) "
                             f"about {fo} projects back to ({n}, {e}); more than "
                             f"{CONSISTENCY_TOL_M} m away, so it is not converted")
        return {**{k: v for k, v in node.items() if k not in ("x", "y", "lat", "lon")},
                "lat": lat, "lon": lon}
    return {k: _raw_in_wgs84(v, fo) for k, v in node.items()}


def _geo_canonical(node):
    """Drop the derived x/y from every point that carries lat/lon."""
    if isinstance(node, list):
        return [_geo_canonical(v) for v in node]
    if isinstance(node, dict):
        if "lat" in node and "lon" in node and "x" in node and "y" in node:
            return {k: v for k, v in node.items() if k not in ("x", "y")}
        return {k: _geo_canonical(v) for k, v in node.items()}
    return node


def canonical_reference_doc(doc):
    """A reference-form document (PolicyDoc dump) with every polygon ring in
    canonical order (canonical_ring_order): what `Policy.reference_ir()`
    writes for the same policy, whatever order the document was written in."""
    import copy
    out = copy.deepcopy(doc)
    for r in (out.get("constraints") or []) if isinstance(out, dict) else []:
        geo = r.get("geometry") if isinstance(r, dict) else None
        if isinstance(geo, dict) and isinstance(geo.get("vertices"), list) and \
                all(isinstance(v, dict) and "lat" in v and "lon" in v
                    for v in geo["vertices"]):
            geo["vertices"] = canonical_ring_order(geo["vertices"])
    return out


def ir2_rule_view(rule: dict) -> dict:
    """One rule of an archived IR (a bundle's ir.json, a policy_g<N>.json) as
    IR schema 3.0.0 stores it: `repair` read as project_fix, a keep-out ring
    in canonical order. For comparing an archived 2.x document with the
    policy rebuilt from it (guardrail.bundle.check_bundle)."""
    out = dict(rule)
    va = out.get("violation_action")
    if isinstance(va, str):
        out["violation_action"] = LEGACY_ACTION_SPELLINGS.get(va, va)
    if out.get("type") in ("polygon_fence", "dynamic_nfz") and \
            isinstance(out.get("vertices"), list) and \
            all(isinstance(v, dict) for v in out["vertices"]):
        out["vertices"] = canonical_ring_order(out["vertices"])
    return out


def _metric_view(node):
    """Drop lat/lon from every point, keeping x/y: what the IR held before
    hash scheme v3 (and what every legacy era hashed)."""
    if isinstance(node, list):
        return [_metric_view(v) for v in node]
    if isinstance(node, dict):
        if "x" in node and "y" in node:
            return {k: v for k, v in node.items() if k not in ("lat", "lon")}
        return {k: _metric_view(v) for k, v in node.items()}
    return node


def _grant_rule(c, ll) -> dict:
    """One rule in the grant's form (see Policy.to_grant_form)."""
    r: dict[str, Any] = {"id": c.id, "type": c.type,
                         "constraint_type": c.constraint_type,
                         "scope": c.effective_scope, "priority": c.priority,
                         "layer": c.effective_layer,
                         "violation_action": canonical_action(c.violation_action)}
    if c.locked_by is not None:            # merge provenance (ConstraintBase)
        r["locked_by"] = list(c.locked_by)
    if c.valid_time is not None:
        r["valid_time"] = c.valid_time.model_dump(mode="json", exclude_none=True)
    ref = getattr(c, "altitude_ref", None) or "AGL"
    band = {}
    if hasattr(c, "altitude_floor_m"):
        band = {"altitude_floor_m": c.altitude_floor_m,
                "altitude_ceiling_m": c.altitude_ceiling_m, "altitude_ref": ref}
    if isinstance(c, CircleFence):
        r["geometry"] = {"center": ll(c.center), "radius_m": c.radius_m, **band}
    elif isinstance(c, (PolygonFence, DynamicNFZ)):
        r["geometry"] = {"vertices": [ll(v) for v in c.vertices], **band}
        if isinstance(c, DynamicNFZ) and c.motion is not None:
            r["motion"] = c.motion.model_dump(mode="json")
    elif isinstance(c, (Corridor, CorridorSwap)):
        r["geometry"] = {"centerline": [ll(v) for v in c.centerline],
                         "width_m": c.width_m, **band}
        if isinstance(c, CorridorSwap):
            r["target_id"] = c.target_id
    elif isinstance(c, AltitudeEnvelope):
        r.update(altitude_min_m=c.alt_min_m, altitude_max_m=c.alt_max_m,
                 altitude_ref=ref)
    elif isinstance(c, KinematicEnvelope):
        r.update(speed_max=c.speed_max_mps, climb_rate_max=c.climb_rate_max_mps,
                 turn_rate_max=c.yaw_rate_max_dps)
    elif isinstance(c, DistanceEnvelope):
        r.update(object_class=c.object_class, min_distance_m=c.min_distance_m)
    elif isinstance(c, TimeWindowSwitch):
        r.update(target_id=c.target_id, active=c.active)
    else:                       # obstacle_clearance, subject_standoff: our own
        own = c.model_dump(mode="json", exclude_none=True)
        for k in ("id", "type", "constraint_type", "scope", "priority", "layer",
                  "locked_by", "violation_action", "valid_time"):
            own.pop(k, None)
        r.update(own)
    m = getattr(c, "margin_m", 0.0)
    if m != 0.0:
        r["margin_m"] = m
    return r


def _envelope_conflicts(policy: Policy) -> list[str]:
    """Hard altitude limits that cannot all hold at once: any two of the hard
    envelopes and hard corridor bands with no height in common while their
    schedules overlap (grant lint: "conflicting hard rules"). Pairwise is
    enough: intervals that meet pairwise share a point (1-D Helly). Rules with
    different altitude references are not compared.

    Two corridors are compared too. The Shield checks every active corridor on
    its own (guardrail/shield.py, `for c in self._corridors`), so the vehicle
    must be inside ALL of them at once; two hard corridors whose bands never
    meet cannot both hold. (Until 2026-10-07 corridor pairs were skipped as
    "different places".) Corridors whose tubes are horizontally disjoint are
    equally unsatisfiable; since 2026-11-02 `_geometry_conflicts` checks that,
    and a corridor buried in keep-out zones."""
    bands = []
    for c in policy.constraints:
        if c.constraint_type != "hard":
            continue
        if isinstance(c, AltitudeEnvelope):
            bands.append((c, c.alt_min_m, c.alt_max_m))
        elif isinstance(c, Corridor):
            bands.append((c, c.altitude_floor_m, c.altitude_ceiling_m))
    out = []
    for i in range(len(bands)):
        for j in range(i + 1, len(bands)):
            (a, a0, a1), (b, b0, b1) = bands[i], bands[j]
            if (getattr(a, "altitude_ref", None) or "AGL") != \
                    (getattr(b, "altitude_ref", None) or "AGL"):
                continue
            if max(a0, b0) > min(a1, b1) and _times_overlap(a, b):
                out.append(f"{a.id} ({a0:g}-{a1:g} m) and {b.id} ({b0:g}-{b1:g} m) "
                           f"are hard altitude limits with no height in common: no "
                           f"flight can satisfy both")
    return out


def _geometry_conflicts(policy: Policy) -> list[str]:
    """Hard rules whose GEOMETRY cannot all hold at once (grant lint:
    "conflicting hard rules"), beyond the altitude bands `_envelope_conflicts`
    compares. Two cases, both added 2026-11-02 (WP1-02):

      * a hard keep-IN tube (a corridor, or the corridor a corridor_swap puts
        in force) lying wholly inside hard keep-out zones that forbid every
        height of its band while both can be in force: every point the
        corridor allows is a point a zone forbids. One zone at a time, with
        any schedules; several zones together only when all of them are
        always in force;
      * two hard keep-in tubes with no point in common while both can be in
        force: the Shield holds the vehicle inside every active corridor at
        once (`_envelope_conflicts` compares their bands).

    Only certain conflicts are named. What a switch or a swap may turn off
    is left out (a switch's target, a swap's target, and a swap against its
    own target), as is a moving dynamic_nfz (it does not stay where it was
    declared), a pair under different altitude references, and anything that
    needs the clock or a map (obstacle clearance, stand-off). The keep-out
    area is the polygon grown by its margin exactly as the Shield builds it
    (geometry.keepout_area); a tube is grown outward by the chord error of
    shapely's round caps, so the tube tested contains every point the
    Shield's distance-to-segments check accepts and the test never reports a
    corridor that has a legal point left. A corridor whose boundary only
    touches a zone's is not reported (the touching points are legal).

    shapely is imported only when the policy has a hard tube and a second
    rule to compare it with, so models.py stays importable without it."""
    switched = {c.target_id for c in policy.constraints if isinstance(c, TimeWindowSwitch)}
    swapped = {c.target_id for c in policy.constraints if isinstance(c, CorridorSwap)}
    tubes = [c for c in policy.constraints
             if isinstance(c, (Corridor, CorridorSwap)) and c.constraint_type == "hard"
             and c.id not in switched and c.id not in swapped]
    zones = [c for c in policy.constraints
             if isinstance(c, (PolygonFence, DynamicNFZ)) and c.constraint_type == "hard"
             and c.id not in switched
             and not (isinstance(c, DynamicNFZ) and c.motion is not None
                      and any((c.motion.vx_mps, c.motion.vy_mps, c.motion.yaw_rate_dps)))]
    if not tubes or (not zones and len(tubes) < 2):
        return []
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    from .geometry import area_holds_inside, area_meets, keepout_area

    def ref(c) -> str:
        return getattr(c, "altitude_ref", None) or "AGL"

    # shapely's buffer puts its arc vertices ON the circle, so its round caps
    # are chords inside the true tube; grown by 1/cos(pi/64) (16 segments a
    # quarter circle) the polygon contains the whole true tube.
    grow = 1.0 / math.cos(math.pi / 64.0)
    tube_area = {t.id: LineString(t.points()).buffer(t.half_width_m * grow + 1e-9)
                 for t in tubes}
    out: list[str] = []
    for t in tubes:
        lo, hi = t.altitude_floor_m, t.altitude_ceiling_m
        cover = [z for z in zones if ref(z) == ref(t)
                 and z.altitude_floor_m <= lo and z.altitude_ceiling_m >= hi]
        named = False
        for z in cover:
            if _times_overlap(t, z) and area_holds_inside(keepout_area(z), tube_area[t.id]):
                out.append(f"hard corridor {t.id} ({lo:g}-{hi:g} m) lies wholly inside "
                           f"hard keep-out zone {z.id} at every height of its band: "
                           f"no point the corridor allows is outside the zone, so "
                           f"no flight can satisfy both")
                named = True
        always = [z for z in cover if z.valid_time is None]
        if not named and len(always) > 1:
            union = unary_union([keepout_area(z) for z in always])
            if area_holds_inside(union, tube_area[t.id]):
                out.append(f"hard corridor {t.id} ({lo:g}-{hi:g} m) lies wholly inside "
                           f"the hard keep-out zones {', '.join(z.id for z in always)} "
                           f"at every height of its band: no point the corridor "
                           f"allows is outside them, so no flight can satisfy all")
    for i in range(len(tubes)):
        for j in range(i + 1, len(tubes)):
            a, b = tubes[i], tubes[j]
            if ref(a) != ref(b) or not _times_overlap(a, b):
                continue
            if isinstance(a, CorridorSwap) and a.target_id == b.id or \
                    isinstance(b, CorridorSwap) and b.target_id == a.id:
                continue
            if not area_meets(tube_area[a.id], tube_area[b.id]):
                out.append(f"hard corridors {a.id} and {b.id} have no point in "
                           f"common: the Shield holds the vehicle inside every "
                           f"active corridor, so no flight can satisfy both")
    return out


# --------------------------------------------------------------------------- #
# Hash forms
# --------------------------------------------------------------------------- #

# Name of the current identity form: SHA-256 over the None-free canonical JSON,
# full digest, geographic points as lat/lon. Recorded in every bundle manifest
# so a reader never has to infer which canonicalisation produced a hash.
# v3 (2026-10-06) differs from v2 only for a policy with WGS84 points: v2
# hashed their projected metres. For every metre policy the bytes are the same.
HASH_SCHEME = "sha256-canonical-v3"
PRIOR_SCHEME_V2 = "sha256-canonical-v2"
# The same v3 canonical bytes of the IR as schema 2.x held it (Policy.
# ir2_hashes): rings as given and `repair` for project_fix, or project_fix as
# written. Every hash recorded before 2026-11-02 is in one of these (or older)
# forms.
IR2_FORM = "sha256-canonical-v3/ir-2.x"
IR2_FORM_AS_WRITTEN = "sha256-canonical-v3/ir-2.x-project_fix"
IR2_FORMS = (IR2_FORM, IR2_FORM_AS_WRITTEN)
# The first 16 hex of the current hash (Policy.policy_hash_short), recognised
# as a hand-written pin; no artefact is written in it.
SHORT_FORM = "sha256-canonical-v3/short16"
# The reference implementation's hash of the same document (see reference_ir).
REFERENCE_FORM = "reference-policy-dsl-0.1"
# Fields the reference writes out when this project leaves them absent
# (Policy.reference_delta): its defaults "mission", "mission", "AGL".
_REFERENCE_DEFAULTED = ("scope", "layer", "altitude_ref")

# The IR schema's own version, independent of any one policy's semver. Bumped
# when a field is added, removed, or has its default changed; recorded in bundle
# manifests and in policies/policy.lock.json. 1.x was the unversioned shape
# that drifted from 2026-08-25 to 2026-09-08 (see docs/DESIGN-policy-identity.md
# for the three changes); 2.0.0 is the first shape whose hash cannot be moved
# by an additive change. 2.1.0 (2026-10-06) added the grant's fields (scope,
# layer, altitude_ref, issued_at, layers_merged, lat/lon on points) - all
# absent by default - and five rule types; no stored hash moved.
# Also bumped by a change to a value that decides enforcement without being
# stored in the IR: CircleFence.SEGMENTS (the derived polygon's vertex count)
# is the one such value today.
#
# THE FREEZE RULE (grant, Policy DSL page, R6: "every later IR-shape change
# carries a semver bump and a migration note"). Since 2026-11-02 EVERY change
# to the shape - additive ones included - needs a bump here, a row in
# docs/IR-MIGRATIONS.md, and `python -m guardrail.bundle lock --update` to
# re-pin the lock (guardrail.bundle.ir_schema_drift / check_lock; tests/
# test_policy_hash.py). Until then a new field defaulting to None was let
# through unbumped, because it moves no stored hash; that is this project's
# rule, not the grant's, and `locked_by` (2385f84, 2026-10-19) went in under
# it. 2.2.0 (2026-11-02) is that change, versioned: no field moved.
#
# 3.0.0 (2026-11-02, WP1-01 / WP1-02): constraint_type, priority and
# violation_action are required (they defaulted to hard / P0 / `repair`);
# violation_action holds the grant's six only (`repair` is read as
# project_fix); polygon rings are stored in canonical order. Every policy
# that spelt `repair`, or wrote a ring in another order, has a new hash: a
# major version, as 2.0.0 was. Old hashes resolve through Policy.ir2_hashes
# and the lock's `migrated_from` (docs/IR-MIGRATIONS.md).
#
# 3.1.0 (2026-11-03, WP1-14): a point's `x` / `y` are optional (they were
# required): a WGS84 point stores lat/lon only and its metres are a view,
# computed when first read from the frame its policy binds it to (XY). Every
# 3.0.0 document still validates and no stored hash moves (the canonical form
# held lat/lon only for a WGS84 point already). The shipped policies were
# converted to WGS84 in the same release - a content change, so each raised
# its version (bundle.FRAME_FORM, tools/convert_policies_wgs84.py).
#
# 4.0.0 (2026-11-06, WP1-01): `scope` is required on every rule, as the
# grant's ConstraintBase writes it (11 pins, '<absent>' -> '<required>'; an
# absent scope meant "mission"). A field a writer must now give is a breaking
# change: a major version, as 3.0.0 was. A document of an earlier era - a
# bundle whose manifest names an older ir_schema_version, a lock recipe
# recorded before, a replay bundle or a run's policy_g<N>.json (formats that
# record no version) - is read in its own era (era_context): scope stays
# absent where it wrote none, so it hashes exactly as stored. The shipped
# policies state scope since this release - a content change, so each raised
# its patch version; the old version is read back as the file with scope
# left out (Policy.ir3_form, bundle.IR3_FORM).
IR_SCHEMA_VERSION = "4.0.0"

# The schema eras the old 16-hex hash went through, by which None-valued keys
# `model_dump()` emitted as null in each. Derived from `git log -- guardrail/
# models.py`, not estimated:
#   up to dba9e48 (2026-08-25)  no optional-None field existed at all
#   4950be1 (2026-09-01)        `valid_time` on every rule (+ its `recurrence`)
#   445bdbe (2026-09-01) ->     `origin` on the policy as well, until 2026-10-06
# A policy that sets every optional field hashes identically in several eras.
# Order decides which name `hash_form` reports then: longest-lived era first
# (five weeks, then ~ten weeks, then a few hours on 2026-09-01).
_LEGACY_ERAS: dict[str, frozenset[str]] = {
    "legacy16-include-defaults": frozenset({"valid_time", "recurrence", "origin"}),
    "legacy16-exclude-none": frozenset(),
    "legacy16-valid-time": frozenset({"valid_time", "recurrence"}),
}


def ir_models(root: type[BaseModel] | None = None) -> list[type[BaseModel]]:
    """Every model reachable from `root` (default Policy) through its fields.

    Walked, not listed. The first version pinned a hand-written tuple of
    classes, so a constraint type added to the `Constraint` union - or a model
    nested inside one - would never have been pinned unless someone also
    remembered to edit the tuple (2026-10-06 review).
    """
    seen: dict[str, type[BaseModel]] = {}

    def visit(tp) -> None:
        if isinstance(tp, type) and issubclass(tp, BaseModel):
            if tp.__name__ not in seen:
                seen[tp.__name__] = tp
                for f in tp.model_fields.values():
                    visit(f.annotation)
            return
        for arg in typing.get_args(tp):
            visit(arg)

    visit(Policy if root is None else root)
    return list(seen.values())


def ir_schema_defaults(root: type[BaseModel] | None = None
                       ) -> dict[str, dict[str, object]]:
    """Every IR model's fields and what an omitted field means.

    `"<required>"` - must be given; `"<absent>"` - defaults to None, so leaving
    it out leaves the canonical bytes unchanged; anything else is the default
    VALUE, which the canonical form writes out and the hash therefore covers.

    policies/policy.lock.json pins this, under the version it was pinned at.
    Since 2026-11-02 no change passes without a schema bump (the grant's R6
    freeze rule): a new field - even one defaulting to `"<absent>"`, the only
    change that provably moves no stored hash -, a new field with a value
    default, a changed default, a removed field and a new MODEL (see
    `ir_models`) are all reported until IR_SCHEMA_VERSION is raised and
    `lock --update` re-pins the lock. Until that date a new `"<absent>"`
    field was allowed unbumped (docs/IR-MIGRATIONS.md, the 2.2.0 row).
    """
    from pydantic_core import PydanticUndefined
    out: dict[str, dict[str, object]] = {}
    for cls in ir_models(root):
        fields: dict[str, object] = {}
        for name, f in cls.model_fields.items():
            if f.default_factory is not None:
                fields[name] = f.default_factory()
            elif f.default is PydanticUndefined:
                fields[name] = "<required>"
            elif f.default is None:
                fields[name] = "<absent>"
            else:
                fields[name] = f.default
        out[cls.__name__] = fields
    return json.loads(json.dumps(out, sort_keys=True))


def canonical_json(obj) -> bytes:
    """Sorted keys, no whitespace, ASCII only - the same SERIALIZER as the
    reference's canonicalize() (vlaguard_common/hashing.py).

    The same serializer is not the same hash. The reference digests
    `doc.model_dump(mode="json")` of the AUTHORED document (policy_dsl/ir.py,
    build_ir), which keeps None as null - `issued_at: null` among them - and
    the grant's field names. This code digests its own IR (`Policy.
    canonical_ir`). So the same policy hashed here and by the reference gives
    different strings; `Policy.reference_hash()` reproduces the reference's
    string for every policy the reference can represent, and `hash_form`
    recognises it (docs/DESIGN-policy-dsl-grant-form.md).
    """
    return json.dumps(obj, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True).encode("ascii")


def _prune_nulls(obj, keep: frozenset[str]):
    """Drop None-valued keys except those named in `keep`, recursively."""
    if isinstance(obj, dict):
        return {k: _prune_nulls(v, keep) for k, v in obj.items()
                if v is not None or k in keep}
    if isinstance(obj, list):
        return [_prune_nulls(v, keep) for v in obj]
    return obj


def _legacy16(policy: "Policy", keep_null: frozenset[str]) -> str:
    """The pre-2026-10-06 hash: 16 hex over `model_dump()` of one schema era.

    The old code used json.dumps' default ensure_ascii=True and the Python-mode
    dump; both are reproduced exactly, because a form that is merely close
    verifies nothing. Fields added since are None for any policy those eras
    could hold, so pruning drops them; points are taken as x/y, as stored then;
    and the IR is the 2.x one (`Policy._ir2_dump`: rings as given, `repair`),
    because no legacy era knew IR 3.0.0.
    """
    # _ERA: a WGS84 point's UNROUNDED metres, what those eras stored.
    dump = _prune_nulls(_metric_view(policy._ir2_dump(mode="python", exclude_none=False,
                                                      context=_ERA)),
                        keep_null)
    canon = json.dumps(dump, sort_keys=True, separators=(",", ":"))
    return "sha256:" + hashlib.sha256(canon.encode()).hexdigest()[:16]


# --------------------------------------------------------------------------- #
# Layered policies (grant: "Layered authoring model")
# --------------------------------------------------------------------------- #

def _strength(action: str) -> int:
    return action_strength(action)


def _not_looser(hi, lo) -> str | None:
    """None when rule `hi` (a higher-precedence layer) is at least as strict as
    `lo`, which it overrides; else why not. What cannot be SHOWN to be at least
    as strict counts as looser: the merge refuses rather than trusts."""
    if hi.type != lo.type:
        return f"changes the rule's type ({lo.type} -> {hi.type})"
    if hi.constraint_type == "soft" and lo.constraint_type == "hard":
        return "turns a hard rule soft"
    if int(hi.priority[1]) > int(lo.priority[1]):
        return f"lowers its priority ({lo.priority} -> {hi.priority})"
    if _strength(hi.violation_action) < _strength(lo.violation_action):
        return (f"weakens its breach action ({lo.violation_action} -> "
                f"{hi.violation_action})")
    if lo.valid_time is None and hi.valid_time is not None:
        return "adds a schedule to a rule that is always in force"
    if lo.valid_time is not None and hi.valid_time is not None and \
            hi.valid_time != lo.valid_time:
        return "changes its schedule (only an identical or no schedule is provably not looser)"
    if (getattr(hi, "altitude_ref", None) or "AGL") != (getattr(lo, "altitude_ref", None) or "AGL"):
        return "changes its altitude reference (AGL/MSL cannot be compared)"
    if isinstance(hi, AltitudeEnvelope):
        if hi.alt_min_m < lo.alt_min_m or hi.alt_max_m > lo.alt_max_m:
            return (f"widens the altitude band ({lo.alt_min_m:g}-{lo.alt_max_m:g} m "
                    f"-> {hi.alt_min_m:g}-{hi.alt_max_m:g} m)")
        return None
    if isinstance(hi, KinematicEnvelope):
        for f in ("speed_max_mps", "climb_rate_max_mps", "yaw_rate_max_dps"):
            if getattr(hi, f) > getattr(lo, f):
                return f"raises {f} ({getattr(lo, f):g} -> {getattr(hi, f):g})"
        return None
    if isinstance(hi, (ObstacleClearance, SubjectStandoff, DistanceEnvelope)):
        for f in ("min_clearance_m", "min_range_m", "min_distance_m", "soft_margin_m"):
            if hasattr(hi, f) and getattr(hi, f) < getattr(lo, f):
                return f"reduces {f} ({getattr(lo, f):g} -> {getattr(hi, f):g})"
        for f in ("subject_class", "object_class"):
            if hasattr(hi, f) and getattr(hi, f) != getattr(lo, f):
                return f"changes {f}"
        return None
    if isinstance(hi, (PolygonFence, Corridor)):
        if isinstance(hi, PolygonFence):   # keep-out: bigger is stricter
            if hi.altitude_floor_m > lo.altitude_floor_m or \
                    hi.altitude_ceiling_m < lo.altitude_ceiling_m:
                return "narrows the fence's altitude band"
        else:                              # keep-in: smaller is stricter
            if hi.altitude_floor_m < lo.altitude_floor_m or \
                    hi.altitude_ceiling_m > lo.altitude_ceiling_m:
                return "widens the corridor's altitude band"
        from .geometry import area_covers, corridor_tube, keepout_area    # lazy: shapely
        if isinstance(hi, PolygonFence):
            if not area_covers(keepout_area(hi).buffer(1e-6), keepout_area(lo)):
                return "shrinks the keep-out area (the new zone does not cover the old)"
        elif not area_covers(corridor_tube(lo).buffer(1e-6), corridor_tube(hi)):
            return "widens the corridor (the new tube is not inside the old)"
        return None
    # dynamic_nfz, time_window_switch, corridor_swap: only an identical rule
    # (bar its layer and merge provenance) is provably not looser.
    a = hi.model_dump(exclude={"layer", "locked_by"})
    b = lo.model_dump(exclude={"layer", "locked_by"})
    return None if a == b else f"changes a {hi.type} rule (only an identical one is allowed)"


def merge_layers(policies: list[Policy], *, policy_id: str | None = None,
                 version: str | None = None) -> Policy:
    """Merge layered policies once, at ingest (grant: "Layered authoring model").

    Each rule's layer is its `layer` field (absent = mission). Precedence is
    mission > site > regulation: a rule id written in more than one layer is
    taken from the highest. A higher layer may tighten a lower layer's HARD
    rule but never relax it - wider band, smaller zone, soft instead of hard,
    lower priority, weaker breach action, a new schedule - and a
    time_window_switch may not turn off another layer's hard rule, nor a
    corridor_swap loosen one. "Another layer's" means ANY layer that wrote a
    hard copy of the target, not only the winning copy's layer. Anything
    that cannot be shown to be at least as strict is refused (ValueError,
    naming every case). A lower layer's soft rule may be relaxed.

    The result is a single policy, hashed post-merge as the grant says, with
    `layers_merged` listing the layers present. A winning rule that overrode
    a lower layer's hard copy records that layer in `locked_by`, so the
    provenance survives the merge and the Shield's hot-apply guard refuses
    the same relaxations at runtime. Its id and version default to
    the LAST policy's (pass them in regulation, site, mission order). Policies
    must agree on `origin`; geographic points are re-projected in the merged
    policy's own frame. Layers that mix metre and lat/lon points must state
    that origin.

    The grant states the precedence twice and the two read differently: the
    authoring model says "mission > site > regulation" and the ingest diagram
    "regulation > site > mission precedence". Both are met here, read as what
    each sentence governs: the HIGHER layer's rule wins a same-id conflict
    (mission over site over regulation), and a LOWER layer's hard rule takes
    precedence over any attempt to relax it (regulation over site over mission).
    """
    if not policies:
        raise ValueError("nothing to merge")
    origins = {None if p.origin is None else (p.origin.lat, p.origin.lon)
               for p in policies}
    if len(origins) > 1:
        raise ValueError(f"the layers state different origins {sorted(map(str, origins))}; "
                         f"metres from two frames cannot be merged")
    if origins == {None}:
        kinds = {v.geographic for p in policies
                 for pts in _rule_points(p, all_points=True) for v in pts}
        if kinds == {True, False}:
            # A metre rule beside lat/lon rules sits in the DERIVED frame (the
            # first polygon_fence's centroid). The merge re-orders rules by
            # layer, which can change which fence comes first and so move
            # every metre rule without a word. Refused until the frame is
            # stated (2026-10-07 review).
            raise ValueError(
                "the layers mix metre and lat/lon points and state no origin: "
                "the metre rules are placed in a frame derived from the "
                "geographic geometry, and the merge can change that frame. "
                "State the same `origin: {lat, lon}` in every layer")
    for p in policies:
        # A layer may name a rule another layer holds (a mission switch for a
        # site curfew): references are resolved on the MERGED policy below.
        bad = p.lint(resolve_refs=False)
        if bad:
            raise ValueError(f"{p.policy_id}: does not pass the DSL lint: " + "; ".join(bad))
    rank = {name: i for i, name in enumerate(LAYER_ORDER)}
    # Validate every candidate rule in ONE frame, so geometry compares fairly.
    every = [c for p in policies for c in p.constraints]
    pool_raw = {"policy_id": "merge-pool", "constraints":
                [_geo_canonical(c.model_dump(mode="json", exclude_none=True)) for c in every]}
    if policies[0].origin is not None:
        pool_raw["origin"] = policies[0].origin.model_dump()
    pool = Policy.model_validate(pool_raw).constraints

    by_id: dict[str, list[tuple[int, int, Any]]] = {}
    for pos, c in enumerate(pool):
        by_id.setdefault(c.id, []).append((rank[c.effective_layer], pos, c))
    problems: list[str] = []
    winners = []
    # Per rule id: every HARD copy written in any layer, and the lock layers
    # an input carried in from an earlier merge (its lower copies are gone).
    # The switch and swap checks below test an event against ALL of them.
    # Until 2026-10-19 they tested the winning copy only, so a mission layer
    # that re-declared a regulation rule (an identical copy passes
    # _not_looser) made it a "mission" rule, and a mission switch then turned
    # it off, or a mission swap widened it 10 m -> 500 m, unrefused.
    hard_copies: dict[str, list[Any]] = {}
    carried: dict[str, set[str]] = {}
    for rid, cands in by_id.items():
        layers = [lv for lv, _, _ in cands]
        dup = [LAYER_ORDER[lv] for lv in set(layers) if layers.count(lv) > 1]
        if dup:
            problems.append(f"{rid}: written twice in layer {dup[0]!r}")
            continue
        cands.sort(key=lambda t: t[0])
        top = cands[-1][2]
        for _, _, lower in cands[:-1]:
            if lower.constraint_type != "hard":
                continue
            why = _not_looser(top, lower)
            if why:
                problems.append(f"{rid}: the {top.effective_layer} layer {why}, "
                                f"relaxing a hard {lower.effective_layer}-layer rule")
        hard_copies[rid] = [c for _, _, c in cands if c.constraint_type == "hard"]
        carried[rid] = {lv for _, _, c in cands for lv in (c.locked_by or ())}
        # The provenance survives the merge (ConstraintBase.locked_by), so the
        # Shield's runtime guard sees the overridden hard copies too. Any value
        # an input carried in is kept: it can only add refusals. (Only an
        # earlier merge's result can carry one: the per-layer lint above
        # refuses locked_by on a layer that names no layers_merged.)
        locks = carried[rid] | {c.effective_layer for _, _, c in cands[:-1]
                                if c.constraint_type == "hard"}
        locks.discard(top.effective_layer)
        if locks:
            top = top.model_copy(update={"locked_by": sorted(locks, key=rank.get)})
        winners.append((cands[-1][0], cands[-1][1], top))
    win_by_id = {c.id: c for _, _, c in winners}
    for _, _, c in winners:
        layer = c.effective_layer
        if isinstance(c, TimeWindowSwitch) and not c.active:
            held = sorted({v.effective_layer for v in hard_copies.get(c.target_id, ())}
                          | carried.get(c.target_id, set()), key=rank.get)
            other = [lv for lv in held if lv != layer]
            if other:
                problems.append(f"{c.id}: a {layer}-layer switch turns off "
                                f"{c.target_id!r}, a hard rule of the "
                                f"{'/'.join(other)} layer")
        if isinstance(c, CorridorSwap):
            tgt = win_by_id.get(c.target_id)
            if tgt is None or not isinstance(tgt, Corridor):
                continue
            # Each hard copy of another layer than the swap's (the swap may
            # relax its own layer's tightening, not what lies beneath it)...
            against = [v for v in hard_copies.get(c.target_id, ())
                       if isinstance(v, Corridor) and v.effective_layer != layer]
            # ...and, for a lock carried in from an earlier merge whose copy is
            # no longer here, the winner in its place (no looser than that
            # copy, so this may refuse more than needed, never less). A winner
            # of another layer is in the list already.
            if tgt.constraint_type == "hard" and tgt.effective_layer == layer and \
                    any(lv != layer for lv in carried.get(c.target_id, ())):
                against.append(tgt)
            as_corr = Corridor(id=tgt.id, type="corridor",
                               constraint_type=c.constraint_type,
                               priority=c.priority, violation_action=c.violation_action,
                               scope=c.effective_scope,   # compared, never stored
                               valid_time=c.valid_time, altitude_ref=c.altitude_ref,
                               centerline=c.centerline, width_m=c.width_m,
                               altitude_floor_m=c.altitude_floor_m,
                               altitude_ceiling_m=c.altitude_ceiling_m)
            for v in against:
                why = _not_looser(as_corr, v)
                if why:
                    problems.append(f"{c.id}: the swap {why}, relaxing hard "
                                    f"{v.effective_layer}-layer corridor {v.id!r}")
    if problems:
        raise ValueError("layer merge refused:\n  - " + "\n  - ".join(problems))
    winners.sort(key=lambda t: (t[0], t[1]))
    # Every layer that went INTO the merge, as the grant's example records it -
    # including one whose every rule was overridden.
    present = sorted({c.effective_layer for c in pool}, key=rank.get)
    last = policies[-1]
    raw = {"policy_id": policy_id or last.policy_id,
           "version": version or last.version,
           "generation": last.generation,
           "layers_merged": present,
           "constraints": [_geo_canonical(c.model_dump(mode="json", exclude_none=True))
                           for _, _, c in winners]}
    if last.origin is not None:
        raw["origin"] = last.origin.model_dump()
    merged = Policy.model_validate(raw)
    bad = merged.lint()
    if bad:
        raise ValueError("the merged policy does not pass the DSL lint: " + "; ".join(bad))
    return merged


def load_layered(paths: list[str | Path], *, policy_id: str | None = None,
                 version: str | None = None, runtime: bool = True,
                 ground=None) -> Policy:
    """Load several policy files and merge them (see merge_layers). Each file
    is validated on its own except for cross-layer references, which are
    resolved on the merged policy. `ground`: see load_policy."""
    layers = []
    for p in paths:
        raw = parse_policy_text(Path(p).read_text(encoding="utf-8"), str(p))
        layer = Policy.model_validate(raw)
        refuse_projected_policy(layer, str(p))           # WP1-14: WGS84 only
        layers.append(layer)
    merged = merge_layers(layers, policy_id=policy_id, version=version)
    if runtime:
        _refuse_unenforced(merged, " + ".join(str(p) for p in paths), ground)
    return merged


# --------------------------------------------------------------------------- #
# Loading
# --------------------------------------------------------------------------- #

class _StrictLoader(yaml.SafeLoader):
    """SafeLoader that refuses a mapping key written twice.

    PyYAML keeps the LAST value without a word, so a reviewer reading the
    first `alt_max_m: 30` would not see the `alt_max_m: 300` that is enforced."""


def _no_duplicate_keys(loader, node, deep=False):
    seen = set()
    for key_node, _ in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            dup = key in seen
        except TypeError:
            continue
        if dup:
            raise ValueError(f"line {key_node.start_mark.line + 1}: key {key!r} is "
                             f"written twice in one mapping; YAML would keep only "
                             f"the last value")
        seen.add(key)
    return loader.construct_mapping(node, deep=deep)


_StrictLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
                              _no_duplicate_keys)


def parse_policy_text(text: str, source: str = "<policy>"):
    """YAML or JSON text -> raw document, refusing duplicate keys."""
    try:
        return yaml.load(text, Loader=_StrictLoader)
    except yaml.YAMLError as exc:
        raise ValueError(f"{source}: not valid YAML/JSON ({exc})") from None


def _refuse_unenforced(pol: Policy, source: str, ground=None) -> None:
    gaps = pol.flight_problems(ground)
    if gaps:
        raise ValueError(
            f"{source}: the Safety Shield cannot enforce this policy as written, "
            f"or the flight rails cannot measure it, so it is not loaded for "
            f"flight:\n  - " + "\n  - ".join(gaps)
            + "\nLoad it with runtime=False to work with it as a declaration "
              "(bundle, schema, export).")


def refuse_projected_policy(pol: Policy, source: str = "<policy>") -> None:
    """Raise ProjectedCoordinatesRefused when `pol` stores a point in local
    metres (Policy.point_form: the legacy read forms; WP1-14, 2026-11-26)."""
    if pol.point_form in LEGACY_POINT_FORMS:
        n = sum(1 for g in _rule_points(pol) if not g)
        raise ProjectedCoordinatesRefused(
            projected_input_message(f"{source} ({pol.policy_id}@{pol.version}): {n} "
                                    f"point(s) stored as {pol.point_form}")
            + ". To READ a recorded document in that form (a re-run of a recorded "
              "flight, a census of old bundles), use models.load_legacy_metre_policy",
            policy=pol)


def policy_from_raw(raw, *, source: str = "<policy>", runtime: bool = True,
                    ground=None, context: dict | None = None) -> Policy:
    """Raw document -> validated, linted Policy. The single DSL entry point
    behind load_policy, the REST ingest and the GeoJSON converter. `ground`:
    see load_policy.

    WGS84 ONLY (WP1-14, 2026-11-26): a document that gives a point in local
    metres ({x, y}; Policy.point_form a legacy form) is refused
    (ProjectedCoordinatesRefused, naming the tool-side conversion). The
    legacy read of such a document is legacy_metre_policy_from_raw.

    `context` is for a reader of a STORED document only (a run's
    policy_g<N>.json): its era, era_context, under which a document written
    before IR schema 4.0.0 may leave `scope` out. Every ingest gives none, so
    every new document states every required field."""
    return _policy_from_raw(raw, source=source, runtime=runtime, ground=ground,
                            context=context, legacy_read=False)


def legacy_metre_policy_from_raw(raw, *, source: str = "<policy>", runtime: bool = True,
                                 ground=None, context: dict | None = None) -> Policy:
    """policy_from_raw that also READS the legacy metre form (a document whose
    points are stored in local metres, Policy.point_form; WP1-14). For the
    declared re-reads of recorded documents only - bundle.load_for_flight
    legacy_read (VLAGUARD_LEGACY_READ=1), tools/replay_at_revision.py,
    tools/bundle_census.py - never for a new document: the DSL never carries
    projected coordinates. A WGS84 or point-free document loads exactly as
    policy_from_raw loads it."""
    return _policy_from_raw(raw, source=source, runtime=runtime, ground=ground,
                            context=context, legacy_read=True)


def _policy_from_raw(raw, *, source: str, runtime: bool, ground, context: dict | None,
                     legacy_read: bool) -> Policy:
    pol = Policy.model_validate(raw, context=context)
    problems = pol.lint()
    if problems:
        raise ValueError(f"{source}: the policy does not pass the DSL lint:\n  - "
                         + "\n  - ".join(problems))
    # After the schema and the lint, so a broken document is refused for its
    # own defect (the WP1 negative corpus), then for its metres.
    if not legacy_read:
        refuse_projected_policy(pol, source)
    if runtime:
        _refuse_unenforced(pol, source, ground)
    return pol


def load_policy(path: str | Path, *, runtime: bool = True, ground=None) -> Policy:
    """YAML/JSON file -> validated Policy. Any schema error raises here, loudly,
    BEFORE flight — never mid-air.

    Accepts the grant's form (geometry blocks, grant field names, lat/lon with
    no origin, issued_at, scope, layer) and this project's earlier form.

    `runtime=True` (the default, and what every flight script gets) also
    refuses what the Shield could not enforce as written - a declarable-only
    type, an MSL altitude, a geographic frame derived rather than stated
    (`Policy.flight_problems`). `runtime=False` loads the declaration as
    written, for bundling, schema checks, export and the grant's own
    documents.

    `ground` (X-11, since 2026-11-03): the ground-elevation source the rail
    flies over (guardrail/terrain.py, e.g. terrain.SITL_CMAC). With one, an
    MSL height is flyable - the caller gives the same ground to the Shield,
    which enforces the MSL band as the AGL band over it; without one (the
    default) an MSL height is refused, as it always was. MSL is the grant's
    opt-in, and so is this.

    `issued_at` is part of the document when written (grant worked example,
    reference PolicyDoc) and so of its hash. Until 2026-10-06 it was refused
    here and kept only in the bundle manifest; a policy that does not write it
    still gets its issue time from the manifest (guardrail/bundle.py).
    """
    p = Path(path)
    raw = parse_policy_text(p.read_text(encoding="utf-8"), str(p))
    return policy_from_raw(raw, source=str(p), runtime=runtime, ground=ground)


def load_legacy_metre_policy(path: str | Path, *, runtime: bool = True,
                             ground=None) -> Policy:
    """load_policy that also READS a document stored in the legacy metre form
    (WP1-14, 2026-11-26; see legacy_metre_policy_from_raw): for the declared
    re-reads of recorded documents only."""
    p = Path(path)
    raw = parse_policy_text(p.read_text(encoding="utf-8"), str(p))
    return legacy_metre_policy_from_raw(raw, source=str(p), runtime=runtime,
                                        ground=ground)
