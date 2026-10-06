"""
Policy DSL (mini) — Pydantic models + YAML loader.

Scaled-down version of the grant's constraint taxonomy. Six constraint classes
(the grant's DSL names nine):

    polygon_fence       keep-OUT area (no-fly zone), 2-D polygon + altitude band
    altitude_envelope   min/max height above ground
    kinematic_envelope  speed / climb-rate / yaw-rate caps
    obstacle_clearance  min distance from MAPPED buildings (needs a runtime map)
    subject_standoff    min distance from the tracked subject (needs perception)
    corridor            keep-IN tube: centerline + width + altitude band

Every rule may also carry `valid_time`, a recurring schedule saying when it is
in force. That is a FIELD rather than a constraint class, matching the reference
DSL - see ConstraintBase.

Coordinates: local meters (x = North, y = East), altitude = meters above ground,
up positive. The real grant DSL uses WGS84 lat/lon; local meters keeps the
prototype simple and matches AirSim's local frame. Swap later = loader change only.
"""
from __future__ import annotations

import hashlib
import json
import typing
from pathlib import Path
from typing import Annotated, Literal, Union

import yaml
from pydantic import BaseModel, Field, model_validator

from .projection import project_raw


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
    x: float               # m North of start
    y: float               # m East of start
    up: float              # m above ground
    yaw_deg: float = 0.0


# --------------------------------------------------------------------------- #
# Constraint taxonomy (the policy file surface)
# --------------------------------------------------------------------------- #

DAYS = ("Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun")


class Recurrence(BaseModel):
    """A weekly schedule: which days, and a clock window within them.

    `start_time` / `end_time` are "HH:MM" local. A window that ends before it
    starts WRAPS past midnight ("22:00"-"06:00"), which is the common case for a
    night-flight restriction and would otherwise be inexpressible.
    """
    days: list[Literal[DAYS]] = Field(default_factory=lambda: list(DAYS))
    start_time: str = "00:00"
    end_time: str = "23:59"

    @staticmethod
    def _mins(hhmm: str) -> int:
        h, _, m = hhmm.partition(":")
        return int(h) * 60 + int(m)

    @model_validator(mode="after")
    def _parseable(self) -> "Recurrence":
        for f in (self.start_time, self.end_time):
            try:
                mins = self._mins(f)
            except ValueError:
                raise ValueError(f"time {f!r} is not HH:MM")
            if not 0 <= mins <= 24 * 60:
                raise ValueError(f"time {f!r} out of range")
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


class ValidTime(BaseModel):
    """When a rule is in force. `recurrence` absent means always."""
    recurrence: Recurrence | None = None

    def active_at(self, when) -> bool:
        return True if self.recurrence is None else self.recurrence.active_at(when)


class ConstraintBase(BaseModel):
    id: str
    constraint_type: Literal["hard", "soft"] = "hard"
    priority: Literal["P0", "P1", "P2"] = "P0"
    violation_action: Literal["repair", "brake"] = "repair"

    # Time windows are a FIELD on every rule, not a constraint type of their
    # own. That is the reference DSL's shape (`policy-dsl.md`): a recurring
    # schedule is something any rule may carry, while `time_window_switch` is a
    # separate hot-applicable EVENT that toggles a rule by reference. Modelling
    # the schedule as a type would have made "this fence applies on weekdays"
    # inexpressible without inventing a link between two rules.
    valid_time: ValidTime | None = None

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


class XY(BaseModel):
    x: float
    y: float


class PolygonFence(ConstraintBase):
    """Keep-OUT no-fly zone. Violated when a (predicted) position is inside."""
    type: Literal["polygon_fence"]
    vertices: list[XY] = Field(min_length=3)
    altitude_floor_m: float = 0.0
    altitude_ceiling_m: float = 1000.0
    margin_m: float = 1.0            # extra safety ring around the polygon

    @model_validator(mode="after")
    def _sane_band(self) -> "PolygonFence":
        if self.altitude_ceiling_m <= self.altitude_floor_m:
            raise ValueError(f"{self.id}: ceiling must be > floor")
        return self


class AltitudeEnvelope(ConstraintBase):
    """Stay between alt_min and alt_max (meters above ground)."""
    type: Literal["altitude_envelope"]
    alt_min_m: float
    alt_max_m: float

    @model_validator(mode="after")
    def _sane(self) -> "AltitudeEnvelope":
        if self.alt_max_m <= self.alt_min_m:
            raise ValueError(f"{self.id}: alt_max must be > alt_min")
        return self


class KinematicEnvelope(ConstraintBase):
    """Caps on how fast the vehicle may move/climb/turn."""
    type: Literal["kinematic_envelope"]
    speed_max_mps: float = Field(gt=0)          # horizontal speed cap
    climb_rate_max_mps: float = Field(gt=0)     # |vz| cap
    yaw_rate_max_dps: float = Field(gt=0)


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
    """
    type: Literal["obstacle_clearance"]
    min_clearance_m: float = Field(gt=0)
    soft_margin_m: float = Field(default=0.0, ge=0)


# NOTE (2026-09-08): `soft_margin_m` below is declared, hashed into
# policy_hash, set to 2.0 in every flown follow policy - and read by NOTHING.
# The Shield reads a soft_margin_m at exactly one site, guardrail/shield.py's
# ObstacleClearance repair, and that is ObstacleClearance's own field. So this
# one is a knob that changes the reproducibility hash and no behaviour.
#
# Left in place rather than deleted: removing it would change policy_hash on
# every stored run and orphan the two replay bundles, for a field that costs
# nothing. Recorded here so the next reader does not spend an afternoon looking
# for where it takes effect, and so nobody tunes it expecting an outcome.
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


class Corridor(ConstraintBase):
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

    @model_validator(mode="after")
    def _sane_band(self) -> "Corridor":
        if self.altitude_ceiling_m <= self.altitude_floor_m:
            raise ValueError(f"{self.id}: ceiling must be > floor")
        return self

    @property
    def half_width_m(self) -> float:
        return self.width_m / 2.0

    def points(self) -> list[tuple[float, float]]:
        return [(v.x, v.y) for v in self.centerline]


Constraint = Annotated[
    Union[PolygonFence, AltitudeEnvelope, KinematicEnvelope, ObstacleClearance,
          SubjectStandoff, Corridor],
    Field(discriminator="type"),
]


class Origin(BaseModel):
    """The anchor a lat/lon policy is projected about. See guardrail/projection.py."""
    lat: float = Field(ge=-90.0, le=90.0)
    lon: float = Field(ge=-180.0, le=180.0)


class Policy(BaseModel):
    """A validated policy bundle (the 'IR' in miniature)."""
    policy_id: str
    version: str = "0.1.0"
    generation: int = 0            # bumps on every mid-flight hot-apply (grant rule)
    # Present only on policies authored in WGS84. Kept on the model rather than
    # discarded after projection so the policy still records where its metres
    # are anchored - a bare local frame with no origin cannot be replayed
    # against a map, and it is part of the hash for the same reason.
    origin: Origin | None = None
    constraints: list[Constraint]

    # ------------------------------------------------------------------ #
    # Identity. See docs/DESIGN-policy-identity.md for the whole story.
    # ------------------------------------------------------------------ #

    def canonical_ir(self) -> dict:
        """The canonicalised IR: every field that carries a value, nothing else.

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
        it, so it is refused until the IR schema version is bumped.

        Defaults that are NOT None stay in the canonical form on purpose: if
        `margin_m` defaulted to 2.0 tomorrow, a policy that omits it would
        fly differently, and its hash must say so.
        """
        return self.model_dump(mode="json", exclude_none=True)

    def canonical_bytes(self) -> bytes:
        """The exact bytes `policy_hash` digests - and the bytes a bundle ships
        as `ir.json`, so a reader can check the hash with sha256sum alone."""
        return canonical_json(self.canonical_ir())

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

        For every schema era up to 2026-08-31 this IS the hash those runs
        recorded: the old 16-hex digest of the None-free dump and the new
        64-hex digest are the same SHA-256 over the same bytes, one truncated.
        """
        return self.policy_hash[:len("sha256:") + 16]

    def legacy_hashes(self) -> dict[str, str]:
        """Every SHORTENED hash an older version of this file would have
        produced for this policy, keyed by the form's name.

        The old hash digested `model_dump()`, i.e. "every field the schema had
        THAT DAY", with None written as null. So the bytes depended on which
        optional fields existed when the run flew, and each schema era has its
        own form. `_LEGACY_ERAS` lists them; adding an era is the only way to
        make another historical form verifiable, and guessing one is not.
        """
        return {name: _legacy16(self, keep_null)
                for name, keep_null in _LEGACY_ERAS.items()}

    def hash_form(self, recorded: str | None) -> str | None:
        """Which identity form `recorded` is for THIS policy, or None.

        Returns HASH_SCHEME for the current 64-hex hash, or the name of the
        legacy form it matches. None means the recorded hash is not this
        policy under any form we know - the caller must refuse, not guess.
        """
        if not recorded:
            return None
        if recorded == self.policy_hash:
            return HASH_SCHEME
        for name, h in self.legacy_hashes().items():
            if recorded == h:
                return name
        return None

    def matches_hash(self, recorded: str | None) -> bool:
        """True when `recorded` identifies this policy under any known form."""
        return self.hash_form(recorded) is not None

    def by_type(self, cls) -> list:
        return [c for c in self.constraints if isinstance(c, cls)]


# --------------------------------------------------------------------------- #
# Hash forms
# --------------------------------------------------------------------------- #

# Name of the current identity form: SHA-256 over the None-free canonical JSON,
# full digest. Recorded in every bundle manifest so a reader never has to infer
# which canonicalisation produced a hash.
HASH_SCHEME = "sha256-canonical-v2"

# The IR schema's own version, independent of any one policy's semver. Bumped
# when a field is added, removed, or has its default changed; recorded in bundle
# manifests and in policies/policy.lock.json. 1.x was the unversioned shape
# that drifted from 2026-08-25 to 2026-09-08 (see docs/DESIGN-policy-identity.md
# for the three changes); 2.0.0 is the first shape whose hash cannot be moved
# by an additive change.
IR_SCHEMA_VERSION = "2.0.0"

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

    policies/policy.lock.json pins this. tests/test_policy_hash.py allows one
    kind of change without a schema bump - a NEW field whose default is
    `"<absent>"` - because that is the only change that provably moves no
    stored hash. A new field with a value default, a changed default, or a
    removed field changes what existing policies mean or how they hash, and
    is refused until IR_SCHEMA_VERSION is raised and the lock re-pinned. A new
    MODEL is reported until `lock --update` pins it (see `ir_models`).
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
    keeps lat/lon. This code digests the projected IR in local metres with
    None-valued fields omitted (`Policy.canonical_ir`). So a policy hashed here
    and the same policy hashed by the reference give different strings, even
    with identical field names. That is a recorded deviation
    (docs/DESIGN-policy-identity.md section 2), not an agreement on bytes.
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
    verifies nothing.
    """
    dump = _prune_nulls(policy.model_dump(), keep_null)
    canon = json.dumps(dump, sort_keys=True, separators=(",", ":"))
    return "sha256:" + hashlib.sha256(canon.encode()).hexdigest()[:16]


def load_policy(path: str | Path) -> Policy:
    """YAML file -> validated Policy. Any schema error raises here, loudly,
    BEFORE flight — never mid-air.

    A policy may be authored in local metres (as every existing one is) or in
    WGS84 lat/lon, which the DSL spec calls canonical. Geographic coordinates
    are projected to the local frame HERE, before validation, so the constraint
    models never learn about two coordinate systems — the swap this file's
    header predicted would be "a loader change only".
    """
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if isinstance(raw, dict) and "issued_at" in raw:
        # The grant's worked DSL example (Policy DSL p.3) and the reference
        # PolicyDoc put issued_at INSIDE the hashed document. Here it is a
        # bundle-manifest field, so re-issuing an unchanged policy never moves
        # its hash (docs/DESIGN-policy-identity.md section 5). Pydantic would
        # drop the key without a word; refusing says where it belongs.
        raise ValueError(
            f"{path}: `issued_at` is not a policy field in this implementation. "
            f"It is recorded in the bundle manifest at issue time - `python -m "
            f"guardrail.bundle {Path(path).name} --issued-at <ISO-8601>` - and "
            f"never enters the policy hash. Remove it from the policy file.")
    return Policy.model_validate(project_raw(raw))
