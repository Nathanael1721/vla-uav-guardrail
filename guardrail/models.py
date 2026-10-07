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
                        roads)                                     [declarable]
    dynamic_nfz         polygon with motion, injected mid-flight   [declarable]
    time_window_switch  switches a referenced rule on/off          [declarable]
    corridor_swap       replaces an active corridor with another   [declarable]
    obstacle_clearance  (ours) min distance from MAPPED buildings
    subject_standoff    (ours) min distance from the tracked subject

[declarable] means the DSL validates, hashes, bundles and exports the rule, and
the flight loaders do not accept it yet (RUNTIME_TYPES). Such a policy is
refused by the flight loaders (`load_policy`, `load_layered`,
`bundle.load_bundle`, `bundle.load_for_flight`) unless the caller asks
for the declaration only (`runtime=False`): a rule the Shield silently ignores
is worse than a refusal. So is a geographic policy whose frame is derived
rather than stated (the ArduPilot SITL rail puts the vehicle's start, its
home, at the frame origin).
`Policy.flight_problems()` says what and why. Since 2026-10-07 the Shield
itself enforces dynamic_nfz, time_window_switch and corridor_swap
(guardrail/shield.py, ENFORCED_TYPES); the loaders still refuse them until the
compiler can render them, see RUNTIME_TYPES.

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
    scope, layer, altitude_ref              (absent = the grant's defaults)
    issued_at, layers_merged                (absent)

The IR keeps this project's field names, so no stored hash moved. `to_grant_form`
produces the grant's form back.

COORDINATES

WGS84 is canonical (grant: "WGS84 latitude / longitude is canonical ... the DSL
itself never carries projected coordinates"). A geographic point keeps lat/lon in
the IR and its hash; x/y (metres, x = North, y = East) are added at load for the
Shield - see guardrail/projection.py. A policy written in metres is the legacy
form and keeps loading, and hashing, exactly as before. Altitude is metres above
ground by default; `altitude_ref: MSL` is declarable and refused for flight
(the Shield has no ground-elevation source).

docs/DESIGN-policy-dsl-grant-form.md records the decisions and what still
differs from the reference implementation.
"""
from __future__ import annotations

import hashlib
import json
import math
import typing
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import Annotated, Any, ClassVar, Literal, Union

import yaml
from pydantic import (BaseModel, ConfigDict, Field, field_validator,
                      model_validator)

from .fsm import action_strength, canonical_action
from .projection import LocalProjection, derive_frame_origin, project_raw


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


# --------------------------------------------------------------------------- #
# Vocabulary (the grant's ConstraintBase, Policy DSL page)
# --------------------------------------------------------------------------- #

DAYS = ("Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun")

Scope = Literal["global", "mission", "segment", "waypoint"]
Layer = Literal["regulation", "site", "mission"]
AltitudeRef = Literal["AGL", "MSL"]
# The grant's six, plus `repair`: this repository's spelling of project_fix
# since July, kept so every stored policy (and its hash) stays as written.
ViolationAction = Literal["repair", "monitor_only", "project_fix", "brake",
                          "loiter", "RTL", "land"]

# Merge precedence, lowest first (grant: "mission > site > regulation").
LAYER_ORDER: tuple[str, ...] = ("regulation", "site", "mission")
# What an absent `scope` / `layer` means. The grant's ConstraintBase defaults
# layer to "mission"; it gives scope no default, and the reference uses
# "mission". Absent stays absent in the IR (None) so no stored hash moves.
DEFAULT_SCOPE = "mission"
DEFAULT_LAYER = "mission"

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

    `scope` and `layer` were added on 2026-10-06. Absent means the grant's
    default ("mission" for both, `DEFAULT_SCOPE` / `DEFAULT_LAYER`) and stays
    absent in the IR, so a policy that does not write them hashes as before.
    The Shield enforces every rule everywhere; a narrower `scope` (segment,
    waypoint) is therefore enforced more widely than written, never less.
    """
    model_config = _IR_CONFIG
    id: str = Field(min_length=1)
    constraint_type: Literal["hard", "soft"] = "hard"
    priority: Literal["P0", "P1", "P2"] = "P0"
    violation_action: ViolationAction = "repair"
    scope: Scope | None = None
    layer: Layer | None = None

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

    @classmethod
    def _post_normalise(cls, data: dict) -> dict:
        return data

    @field_validator("violation_action")
    @classmethod
    def _known_to_the_fsm(cls, v: str) -> str:
        # Checked against the escalation FSM's own table so the two cannot
        # drift apart; the value is STORED AS WRITTEN (`repair` stays `repair`)
        # so the canonical IR, and every stored hash, are unchanged.
        canonical_action(v)
        return v

    @property
    def effective_scope(self) -> str:
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


class XY(BaseModel):
    """A point. x = metres North, y = metres East of the policy's frame origin.

    A point authored in WGS84 also keeps `lat` / `lon`: those are what the
    canonical IR stores and hashes, and x/y are derived from them at load
    (guardrail/projection.py). A metre point leaves lat/lon absent."""
    model_config = _IR_CONFIG
    x: float
    y: float
    lat: float | None = Field(default=None, ge=-90.0, le=90.0)
    lon: float | None = Field(default=None, ge=-180.0, le=180.0)

    @model_validator(mode="before")
    @classmethod
    def _projected_first(cls, data):
        if isinstance(data, dict) and ("x" not in data or "y" not in data):
            has_lat, has_lon = data.get("lat") is not None, data.get("lon") is not None
            if has_lat != has_lon:
                raise ValueError(f"point {data}: lat and lon come as a pair")
            if has_lat:
                raise ValueError(
                    f"point {data} reached the model without being projected. "
                    f"Geographic points are projected by the Policy (it knows the "
                    f"frame origin): validate the whole policy, not a single rule.")
        return data

    @model_validator(mode="after")
    def _pair(self) -> "XY":
        if (self.lat is None) != (self.lon is None):
            raise ValueError("lat and lon come as a pair")
        return self

    @property
    def geographic(self) -> bool:
        return self.lat is not None


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


def _point_in_ring(x: float, y: float, ring) -> bool:
    """Even-odd containment of (x, y) in a simple ring (boundary counts)."""
    inside = False
    n = len(ring)
    for i in range(n):
        (ax, ay), (bx, by) = ring[i], ring[(i + 1) % n]
        if _segments_meet((ax, ay), (bx, by), (x, y), (x, y)):
            return True
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            inside = not inside
    return inside


def _dist_to_ring(x: float, y: float, ring) -> float:
    best = float("inf")
    n = len(ring)
    for i in range(n):
        (ax, ay), (bx, by) = ring[i], ring[(i + 1) % n]
        vx, vy = bx - ax, by - ay
        l2 = vx * vx + vy * vy
        t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / l2))
        best = min(best, math.hypot(x - (ax + t * vx), y - (ay + t * vy)))
    return best


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


class PolygonFence(_Banded, ConstraintBase):
    """Keep-OUT no-fly zone. Violated when a (predicted) position is inside."""
    type: Literal["polygon_fence"]
    vertices: list[XY] = Field(min_length=3)
    altitude_floor_m: float = 0.0
    altitude_ceiling_m: float = 1000.0
    margin_m: float = 1.0            # extra safety ring around the polygon
    altitude_ref: AltitudeRef | None = None

    GRANT_GEOMETRY: ClassVar[frozenset[str]] = _FENCE_GEOMETRY
    GRANT_DEFAULTS: ClassVar[dict[str, Any]] = _FENCE_GRANT_DEFAULTS

    @model_validator(mode="after")
    def _sane_band(self) -> "PolygonFence":
        self._check_band()
        return self

    @model_validator(mode="after")
    def _simple_polygon(self) -> "PolygonFence":
        why = polygon_problem(self.ring())
        if why:
            raise ValueError(f"{self.id}: {why}")
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


# NOTE (2026-09-08): `soft_margin_m` below is declared, hashed into
# policy_hash, set to 2.0 in every flown follow policy - and read by the Shield
# at no site (demo/policy_hud.py reads it, for the on-screen NEAR status only).
# The Shield reads a soft_margin_m at exactly one site, guardrail/shield.py's
# ObstacleClearance repair, and that is ObstacleClearance's own field. So this
# one is a knob that changes the reproducibility hash and the HUD, not the
# Shield's behaviour.
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

    DECLARABLE ONLY. The Shield has no map of people or roads; mapped buildings
    are covered by obstacle_clearance and the followed subject by
    subject_standoff. A policy carrying this rule is refused by the flight
    loaders rather than flown with the rule silently ignored."""
    type: Literal["distance_envelope"]
    object_class: Literal["people", "buildings", "roads"]
    min_distance_m: float = Field(gt=0)


class Motion(BaseModel):
    """How a dynamic_nfz moves after it appears: a constant translation and a
    rotation about its centroid (grant: "Polygon with motion (translate /
    rotate)"). +yaw is clockwise seen from above (North towards East)."""
    model_config = _IR_CONFIG
    vx_mps: float = 0.0          # North
    vy_mps: float = 0.0          # East
    yaw_rate_dps: float = 0.0


class DynamicNFZ(_Banded, ConstraintBase):
    """Grant: hot-applicable "Polygon with motion (translate / rotate) injected
    mid-flight".

    DECLARABLE ONLY for the flight loaders (RUNTIME_TYPES). Since 2026-10-07
    the Shield enforces it, motion included (guardrail/shield.py, hot-apply);
    the loaders keep refusing a policy file that carries one until the
    compiler can render it. `ring_at(t)` is the declared motion's geometry."""
    type: Literal["dynamic_nfz"]
    vertices: list[XY] = Field(min_length=3)
    altitude_floor_m: float = 0.0
    altitude_ceiling_m: float = 1000.0
    margin_m: float = 1.0
    altitude_ref: AltitudeRef | None = None
    motion: Motion | None = None         # absent = does not move once spawned

    GRANT_GEOMETRY: ClassVar[frozenset[str]] = _FENCE_GEOMETRY
    GRANT_DEFAULTS: ClassVar[dict[str, Any]] = _FENCE_GRANT_DEFAULTS

    @model_validator(mode="after")
    def _checks(self) -> "DynamicNFZ":
        self._check_band()
        why = polygon_problem([(v.x, v.y) for v in self.vertices])
        if why:
            raise ValueError(f"{self.id}: {why}")
        return self

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

    DECLARABLE ONLY for the flight loaders (RUNTIME_TYPES; since 2026-10-07
    the Shield evaluates switches, see DynamicNFZ). The DSL checks that the
    target exists, and the layer merge refuses a switch that turns off a hard
    rule of another layer."""
    type: Literal["time_window_switch"]
    target_id: str = Field(min_length=1)
    active: bool

    GRANT_ALIASES: ClassVar[dict[str, str]] = {"target": "target_id"}


class CorridorSwap(_Tube, ConstraintBase):
    """Grant: hot-applicable, "Replaces an active corridor with another at
    runtime". `target_id` names the corridor replaced; the rest is the new
    corridor, in the corridor's own fields.

    DECLARABLE ONLY for the flight loaders, as the other two hot-apply
    classes."""
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

# The rule types a flight may carry. Everything else is declarable only; see
# Policy.unenforced_rules. Two modules key on this set: the flight loaders
# (refuse the rest) and the constraint compiler (guardrail/compiler.py renders
# only these; tests/test_csp.py demands a template for each). It must never
# hold a type the Shield does not enforce (shield.ENFORCED_TYPES; checked in
# tests/test_policy_dsl_grant_form.py). Since 2026-10-07 the Shield also
# enforces dynamic_nfz, time_window_switch and corridor_swap, but the compiler
# has no templates for them, so they stay out of this set - refused by the
# loaders, an over-refusal rather than a rule silently ignored - until the
# templates land; then they are added here in the same change.
RUNTIME_TYPES: frozenset[str] = frozenset({
    "polygon_fence", "circle_fence", "altitude_envelope", "kinematic_envelope",
    "obstacle_clearance", "subject_standoff", "corridor"})
# The grant's nine (Policy DSL page, taxonomy table).
GRANT_TYPES: tuple[str, ...] = (
    "polygon_fence", "circle_fence", "corridor", "altitude_envelope",
    "kinematic_envelope", "distance_envelope", "dynamic_nfz",
    "time_window_switch", "corridor_swap")
# Breach actions the Shield performs itself, whatever rail flies it. Since
# 2026-10-07 the Shield dispatches on the rule's action (guardrail/shield.py,
# ENFORCEMENT): repair / project_fix repair, monitor_only is recorded and never
# repaired, brake skips projection and stops. Until then it repaired, then
# braked, for every rule.
SHIELD_ACTS_ON: frozenset[str] = frozenset({"repair", "project_fix", "monitor_only",
                                            "brake"})
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

    @model_validator(mode="before")
    @classmethod
    def _project(cls, data):
        # Geographic points gain x/y here, once, for the whole policy: only the
        # policy knows its frame origin.
        return project_raw(data) if isinstance(data, dict) else data

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
    def frame_origin(self) -> tuple[float, float] | None:
        """(lat, lon) of the local frame's (0, 0): the stated `origin`, else the
        reference's rule (mean of the first polygon_fence's vertices; then the
        first geographic rule). None for a policy written only in metres."""
        if self.origin is not None:
            return (self.origin.lat, self.origin.lon)
        return derive_frame_origin([c.model_dump() for c in self.constraints])

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
        it, so it is refused until the IR schema version is bumped.

        Defaults that are NOT None stay in the canonical form on purpose: if
        `margin_m` defaulted to 2.0 tomorrow, a policy that omits it would
        fly differently, and its hash must say so.

        A geographic point is stored as {lat, lon}: its x/y are a function of
        the frame, derived at load, and the grant forbids the DSL to carry
        projected coordinates (hash scheme v3, 2026-10-06). A metre point is
        stored as {x, y}, so every metre policy's bytes are what they were.
        """
        dump = self.model_dump(mode="json", exclude_none=True)
        # The walk is skipped for a metre policy: it would change nothing, and
        # the audit logger hashes once per record (it doubled the 50-rule hash
        # time, 0.26 -> 0.52 ms, before this shortcut).
        return _geo_canonical(dump) if self.is_geographic else dump

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
        dump = _metric_view(self.model_dump(mode="json", exclude_none=True))
        return {PRIOR_SCHEME_V2: "sha256:" + hashlib.sha256(canonical_json(dump)).hexdigest()}

    def hash_form(self, recorded: str | None) -> str | None:
        """Which identity form `recorded` is for THIS policy, or None.

        Returns HASH_SCHEME for the current 64-hex hash, the scheme name of a
        prior full-length form, the name of the legacy 16-hex form it matches,
        or REFERENCE_FORM for the reference implementation's hash of the same
        document. None means the recorded hash is not this policy under any
        form we know - the caller must refuse, not guess.
        """
        if not recorded:
            return None
        if recorded == self.policy_hash:
            return HASH_SCHEME
        for name, h in self.prior_hashes().items():
            if recorded == h:
                return name
        for name, h in self.legacy_hashes().items():
            if recorded == h:
                return name
        if recorded == self.reference_hash():
            return REFERENCE_FORM
        return None

    def matches_hash(self, recorded: str | None) -> bool:
        """True when `recorded` identifies this policy under any known form."""
        return self.hash_form(recorded) is not None

    def by_type(self, cls) -> list:
        return [c for c in self.constraints if isinstance(c, cls)]

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
        return why

    def reference_ir(self) -> dict | None:
        """This policy exactly as the reference's `PolicyDoc.model_dump(mode=
        "json")` would hold it - the bytes its bundle ships and its hash covers -
        or None when the reference cannot represent it (`reference_problems`).

        Absent scope/layer/altitude_ref are written as the reference's defaults
        ("mission", "mission", "AGL"), and `repair` in the grant's spelling
        (`project_fix`), because that is what the reference stores for them."""
        if self.reference_problems():
            return None
        rules = []
        for c in self.constraints:
            r = {"id": c.id, "type": c.type, "constraint_type": c.constraint_type,
                 "scope": c.effective_scope, "priority": c.priority,
                 "layer": c.effective_layer,
                 "violation_action": canonical_action(c.violation_action)}
            if c.type == "polygon_fence":
                r["geometry"] = {"vertices": [{"lat": v.lat, "lon": v.lon}
                                              for v in c.vertices],
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

    def reference_hash(self) -> str | None:
        """The reference implementation's policy_hash for this document."""
        ref = self.reference_ir()
        if ref is None:
            return None
        return "sha256:" + hashlib.sha256(canonical_json(ref)).hexdigest()

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
        return out

    # ------------------------------------------------------------------ #
    # What the runtime can and cannot do with this policy
    # ------------------------------------------------------------------ #

    def unenforced_rules(self) -> list[str]:
        """Rules the Safety Shield would NOT enforce, with the reason. A flight
        loader refuses a policy for which this is non-empty."""
        out: list[str] = []
        for c in self.constraints:
            if c.type not in RUNTIME_TYPES:
                out.append(f"{c.id}: {c.type} is declarable in the DSL but the "
                           f"Safety Shield does not enforce it")
            if getattr(c, "altitude_ref", None) == "MSL":
                out.append(f"{c.id}: altitude_ref MSL - the Shield measures height "
                           f"above ground and has no ground-elevation source to "
                           f"convert mean-sea-level heights")
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

    def flight_problems(self) -> list[str]:
        """Everything that keeps this policy off a flight rail: unenforced rules
        and an unplaceable frame. Empty = flyable as written."""
        frame = self.frame_problem()
        return self.unenforced_rules() + ([frame] if frame else [])

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
        it creates."""
        out = []
        for c in self.constraints:
            if not isinstance(c, PolygonFence) or c.constraint_type != "hard":
                continue
            ring = _dedupe_ring(c.ring())
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
        `origin`, a non-zero `margin_m`, and this project's two own rule types.

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


def _geo_canonical(node):
    """Drop the derived x/y from every point that carries lat/lon."""
    if isinstance(node, list):
        return [_geo_canonical(v) for v in node]
    if isinstance(node, dict):
        if "lat" in node and "lon" in node and "x" in node and "y" in node:
            return {k: v for k, v in node.items() if k not in ("x", "y")}
        return {k: _geo_canonical(v) for k, v in node.items()}
    return node


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
                  "violation_action", "valid_time"):
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
    equally unsatisfiable; that needs geometry this pure-Python lint does not
    have, and is not checked."""
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
# The reference implementation's hash of the same document (see reference_ir).
REFERENCE_FORM = "reference-policy-dsl-0.1"

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
IR_SCHEMA_VERSION = "2.1.0"

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
    could hold, so pruning drops them; points are taken as x/y, as stored then.
    """
    dump = _prune_nulls(_metric_view(policy.model_dump()), keep_null)
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
        from .geometry import corridor_tube, keepout_area    # lazy: shapely
        if isinstance(hi, PolygonFence):
            if not keepout_area(hi).buffer(1e-6).covers(keepout_area(lo)):
                return "shrinks the keep-out area (the new zone does not cover the old)"
        elif not corridor_tube(lo).buffer(1e-6).covers(corridor_tube(hi)):
            return "widens the corridor (the new tube is not inside the old)"
        return None
    # dynamic_nfz, time_window_switch, corridor_swap: only an identical rule
    # (bar its layer) is provably not looser.
    a = hi.model_dump(exclude={"layer"})
    b = lo.model_dump(exclude={"layer"})
    return None if a == b else f"changes a {hi.type} rule (only an identical one is allowed)"


def merge_layers(policies: list[Policy], *, policy_id: str | None = None,
                 version: str | None = None) -> Policy:
    """Merge layered policies once, at ingest (grant: "Layered authoring model").

    Each rule's layer is its `layer` field (absent = mission). Precedence is
    mission > site > regulation: a rule id written in more than one layer is
    taken from the highest. A higher layer may tighten a lower layer's HARD
    rule but never relax it - wider band, smaller zone, soft instead of hard,
    lower priority, weaker breach action, a new schedule - and a
    time_window_switch may not turn off another layer's hard rule. Anything
    that cannot be shown to be at least as strict is refused (ValueError,
    naming every case). A lower layer's soft rule may be relaxed.

    The result is a single policy, hashed post-merge as the grant says, with
    `layers_merged` listing the layers present. Its id and version default to
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
        winners.append((cands[-1][0], cands[-1][1], top))
    win_by_id = {c.id: c for _, _, c in winners}
    for _, _, c in winners:
        if isinstance(c, TimeWindowSwitch) and not c.active:
            tgt = win_by_id.get(c.target_id)
            if tgt is not None and tgt.constraint_type == "hard" and \
                    tgt.effective_layer != c.effective_layer:
                problems.append(f"{c.id}: a {c.effective_layer}-layer switch turns off "
                                f"{tgt.id!r}, a hard {tgt.effective_layer}-layer rule")
        if isinstance(c, CorridorSwap):
            tgt = win_by_id.get(c.target_id)
            if tgt is not None and isinstance(tgt, Corridor) and \
                    tgt.constraint_type == "hard" and \
                    tgt.effective_layer != c.effective_layer:
                as_corr = Corridor(id=tgt.id, type="corridor",
                                   constraint_type=c.constraint_type,
                                   priority=c.priority, violation_action=c.violation_action,
                                   valid_time=c.valid_time, altitude_ref=c.altitude_ref,
                                   centerline=c.centerline, width_m=c.width_m,
                                   altitude_floor_m=c.altitude_floor_m,
                                   altitude_ceiling_m=c.altitude_ceiling_m)
                why = _not_looser(as_corr, tgt)
                if why:
                    problems.append(f"{c.id}: the swap {why}, relaxing hard "
                                    f"{tgt.effective_layer}-layer corridor {tgt.id!r}")
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
                 version: str | None = None, runtime: bool = True) -> Policy:
    """Load several policy files and merge them (see merge_layers). Each file
    is validated on its own except for cross-layer references, which are
    resolved on the merged policy."""
    layers = []
    for p in paths:
        raw = parse_policy_text(Path(p).read_text(encoding="utf-8"), str(p))
        layers.append(Policy.model_validate(raw))
    merged = merge_layers(layers, policy_id=policy_id, version=version)
    if runtime:
        _refuse_unenforced(merged, " + ".join(str(p) for p in paths))
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


def _refuse_unenforced(pol: Policy, source: str) -> None:
    gaps = pol.flight_problems()
    if gaps:
        raise ValueError(
            f"{source}: the Safety Shield cannot enforce this policy as written, "
            f"so it is not loaded for flight:\n  - " + "\n  - ".join(gaps)
            + "\nLoad it with runtime=False to work with it as a declaration "
              "(bundle, schema, export).")


def policy_from_raw(raw, *, source: str = "<policy>", runtime: bool = True) -> Policy:
    """Raw document -> validated, linted Policy. The single DSL entry point
    behind load_policy, the REST ingest and the GeoJSON converter."""
    pol = Policy.model_validate(raw)
    problems = pol.lint()
    if problems:
        raise ValueError(f"{source}: the policy does not pass the DSL lint:\n  - "
                         + "\n  - ".join(problems))
    if runtime:
        _refuse_unenforced(pol, source)
    return pol


def load_policy(path: str | Path, *, runtime: bool = True) -> Policy:
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

    `issued_at` is part of the document when written (grant worked example,
    reference PolicyDoc) and so of its hash. Until 2026-10-06 it was refused
    here and kept only in the bundle manifest; a policy that does not write it
    still gets its issue time from the manifest (guardrail/bundle.py).
    """
    p = Path(path)
    raw = parse_policy_text(p.read_text(encoding="utf-8"), str(p))
    return policy_from_raw(raw, source=str(p), runtime=runtime)
