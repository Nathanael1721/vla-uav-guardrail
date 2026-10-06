"""ScenarioSpec: the stress harness's scenario schema, checked when it is loaded.

WHAT THE GRANT ASKS FOR

Stress Testing, "Scenario authoring" (PDF p1-2):

    "Scenarios are declarative YAML, validated through Pydantic at load time.
     Pure-data form keeps them diffable; programmability is layered on through
     parameter sweeps."

and gives the Pydantic surface, copied below field for field, under the
grant's names and in the grant's order:

    class ScenarioSpec(BaseModel):          class ScenarioEvent(BaseModel):
        scenario_id: str                        at_sim_t: float
        template: Literal[four names]           type: Literal[seven names]
        description: str                        payload: dict
        mission: "Mission"
        policy: "PolicyRef"
        events: list["ScenarioEvent"]
        parameter_sweep: dict[str, list]
        expected_kpis: "ExpectedKPIs"

Until 2026-10-06 the sweep read `experiments/scenarios.yaml` with a bare
`yaml.safe_load` into plain dicts (audit card WP4-02). A misspelt key there was
not an error, it was a field nobody read: a gate written as `p0_escape_rate`
instead of `p0_violation_escape_rate` would have been reported "not measured",
but a misspelt `shield: of` or `ticks` silently ran the default. Every model
here is `extra="forbid"`, so a key the schema does not know stops the load.

WHAT THE GRANT LEAVES OPEN, AND WHAT WAS CHOSEN

`Mission`, `PolicyRef` and `ExpectedKPIs` are forward references the grant
never defines. They are filled in from its worked example (`dyn-nfz-translate-
001`): a mission has `task_prompt`, `start_pose` and `target`; a policy
reference has `bundle_path` and `expected_hash`; expected KPIs are values such
as `P0_escape_rate: 0` and strings such as `fail_safe_correctness: ">=0.99"`.
Those names are accepted exactly as written there.

WHAT IS OURS (each one says why it is not in the grant's model)

  * `template` accepts the grant's four names plus EXTENSION_TEMPLATES. The 13
    scenarios the sweep already ran (static no-fly zone, altitude, speed cap,
    non-finite action, corridor, fixed-time curfew, stand-off) fit none of the
    four, and filing them under a grant template they do not exercise would be
    a false label in the KPI report, which groups by template.
  * `mission.pilot`: the headless harness has no VLA, so the "planner output"
    is a deliberately dumb pilot (`goto` or `constant`). `task_prompt` is kept
    and recorded, and nothing reads it - which the harness says, rather than
    pretending a language model saw it.
  * `stress`: the sensor and environment stressors the headless harness can
    simulate (GPS noise, state latency, GPS dropout, wind, start jitter). The
    Safety Shield page's test matrix (PDF p6) names "GPS noise / latency"; the
    grant's worked example sweeps `wind_speed_mps`. Neither has a home in the
    grant's model.
  * `expected_outcome` and `expected_failsafe`: labels. "Fail-safe trigger
    correctness: triggered when expected, not when not expected" (Stress
    Testing, acceptance KPIs) cannot be measured without a statement of when it
    is expected; until now no scenario made one (audit card WP3-15).
  * `expect: known_failure`, `shield`, `ticks`, `dt`, `lookahead_s`: carried
    over from the old library so its behaviour is unchanged. `known_failures`
    is the same marker for single parameter cells of a family.
  * `expect: tracked`: a family that MEASURES how far past its design point
    the Shield degrades (GPS noise beyond the margin, GPS denial, drawn
    gusts). Its gates hold only what must hold whatever the sensor does, so a
    passing episode may still have put the aircraft inside a P0 polygon. The
    sweep reports such an episode as "tracked", never "pass", so the headline
    pass count is never made of runs that entered a no-fly zone.
  * `events` and `parameter_sweep` default to empty lists here; in the grant's
    model they have no default. An absent list and an empty one validate the
    same, and defaulting them keeps the 13 legacy scenarios readable.
  * `parameter_sample`: seeded random draws alongside the grant's cartesian
    `parameter_sweep`. The grant's smoke profile is "~50 scenarios" and the
    nightly "~200"; drawing approach angles from a seeded generator gives real,
    distinct geometries instead of a padded count. The draw for variant k
    depends only on (scenario_id, k), so variant k is the same geometry in
    every run and every profile.

PARAMETERS ARE PLACEHOLDERS, AND BOTH DIRECTIONS ARE CHECKED

The grant calls a sweep "field name -> sweep values". Here a spec refers to a
swept or sampled parameter by writing the string "$name" where the value goes,
and expansion substitutes it. Two failures are refused at load:

  * a "$name" with no value: the scenario cannot be built;
  * a parameter that is swept but referenced nowhere THAT CHANGES THE FLIGHT:
    every cell would be the same episode, and the sweep would report N runs of
    one scenario as N scenarios - the padding this module exists to prevent.
    A placeholder read only by `description`, `mission.task_prompt`, the
    gates, the labels or a known-failure pin does not count as read: nothing
    executes those fields, so cells that differ only there fly the same
    episode (BEHAVIOUR_EXCLUDED below; `duplicate_cells` uses the same list).

A consequence worth stating: the grant's own worked example, written exactly as
the grant writes it (`parameter_sweep: {vehicle_speed_mps: [...],
wind_speed_mps: [...]}` with no reference to either name in the body), is
REFUSED by this loader. The grant's sweep keys address fields implicitly, but
its Mission model never defines a `vehicle_speed_mps` or `wind_speed_mps` field
for them to address, so there is nothing to bind them to; accepting them would
expand 36 identical episodes. The example is accepted once each swept name is
written where its value goes ("$vehicle_speed_mps"); tests/test_scenario_spec.py
holds both forms.

`random_seed` is the one parameter the harness itself consumes; it is never a
placeholder.
"""
from __future__ import annotations

import hashlib
import itertools
import math
import random
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Literal, Union

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

# The grant's outcome vocabulary lives in guardrail.kpi; one copy, not two.
from .kpi import OUTCOMES

# --------------------------------------------------------------------------- #
# vocabularies
# --------------------------------------------------------------------------- #

GRANT_TEMPLATES = ("dynamic_nfz_movement", "time_window_switch",
                   "radius_scaling", "corridor_swap")

# Families the grant's four do not cover. Named for what they exercise, so the
# KPI report's per-template rows say something true.
EXTENSION_TEMPLATES = (
    "static_nfz",                     # a fixed keep-out zone
    "altitude_envelope",              # floor / ceiling
    "kinematic_envelope",             # speed / climb caps
    "action_contract",                # non-finite command
    "corridor_keep_in",               # keep-in tube
    "time_window_static",             # a scheduled rule at a FIXED time of day
    "subject_standoff",               # range from a tracked subject
    # The Safety Shield page's test matrix (PDF p6), one per stressor that is
    # not already a grant template:
    "high_speed_near_edge",
    "three_simultaneous_violations",
    "sensor_degradation",             # "GPS noise / latency"
    "wind_disturbance",               # the grant example's wind_speed_mps
)
TEMPLATES = GRANT_TEMPLATES + EXTENSION_TEMPLATES

GRANT_EVENT_TYPES = ("spawn_polygon_fence", "translate_polygon_fence",
                     "rotate_polygon_fence", "activate_rule", "deactivate_rule",
                     "swap_corridor", "scale_radius")

# Grant KPI names -> the field that measures each one in a sweep result.
#
# The first three are the names the grant's worked example writes under
# `expected_kpis` (Stress Testing p2). The last two are NOT in the worked
# example: they are snake_case for the KPI report's "mean repair count" and
# "mean repair magnitude" columns (Stress Testing p6). A gate is one episode,
# so the mean over episodes is that episode's own value.
#
# `fail_safe_correctness` maps to the LABEL-based field on purpose:
# guardrail.kpi's `failsafe_trigger_correctness` counts P0 ticks the Shield
# acted on, which is the escape rate restated (audit card WP3-15), and mapping
# the grant's name onto it would carry that confusion into every gate.
GRANT_KPI_ALIASES = {
    "P0_escape_rate": "p0_violation_escape_rate",
    "fail_safe_correctness": "failsafe_matches_label",
    "mean_time_to_safe_s": "mean_time_to_safe_s",
    "mean_repair_count": "repair_count",
    "mean_repair_magnitude": "mean_repair_magnitude_mps",
}

# Fields that do not change what is FLOWN. A cell's behaviour is everything
# else; the "is this parameter read" check and the duplicate-cell check both
# look only at behaviour, so cells that differ in prose or in what they assert
# are not counted as different scenarios.
BEHAVIOUR_EXCLUDED = ("scenario_id", "template", "description", "expected_kpis",
                      "expected_outcome", "expected_failsafe", "expect",
                      "known_failures", "parameter_sweep", "parameter_sample")
BEHAVIOUR_EXCLUDED_MISSION = ("task_prompt",)

HARNESS_PARAMS = ("random_seed",)
_PLACEHOLDER = re.compile(r"^\$([A-Za-z_][A-Za-z0-9_]*)$")
_ID = r"^[a-z0-9][a-z0-9._-]*$"


class _Strict(BaseModel):
    """Unknown keys are an error, never a silently ignored field."""
    model_config = ConfigDict(extra="forbid", populate_by_name=True)


# --------------------------------------------------------------------------- #
# mission
# --------------------------------------------------------------------------- #

class Pose(_Strict):
    """A position in the policy's frame, or in WGS84 as the grant's example has it.

    Local metres (x North, y East, up above ground) is the frame every policy in
    this repository is authored in. `lat`/`lon`/`alt_agl_m` are the grant
    example's names; a lat/lon pose is projected about the policy's own origin
    when the episode is built, and refused if the policy has none - a WGS84
    point with no anchor has no local position.
    """
    x: float | None = None
    y: float | None = None
    up: float | None = None
    lat: float | None = None
    lon: float | None = None
    alt_agl_m: float | None = None
    yaw_deg: float = 0.0

    @model_validator(mode="after")
    def _one_frame(self) -> "Pose":
        local = self.x is not None or self.y is not None
        geo = self.lat is not None or self.lon is not None
        if local == geo:
            raise ValueError("a pose needs exactly one of (x, y) or (lat, lon)")
        if local and (self.x is None or self.y is None):
            raise ValueError("a local pose needs both x and y")
        if geo and (self.lat is None or self.lon is None):
            raise ValueError("a WGS84 pose needs both lat and lon")
        if self.up is not None and self.alt_agl_m is not None:
            raise ValueError("give the altitude once: `up` or `alt_agl_m`")
        for v in (self.x, self.y, self.up, self.lat, self.lon, self.alt_agl_m):
            if v is not None and not math.isfinite(v):
                raise ValueError("pose coordinates must be finite")
        return self

    @property
    def altitude(self) -> float | None:
        return self.up if self.up is not None else self.alt_agl_m

    def local(self, policy) -> tuple[float, float]:
        if self.x is not None:
            return float(self.x), float(self.y)
        origin = getattr(policy, "origin", None)
        if origin is None:
            raise ValueError(
                f"pose ({self.lat}, {self.lon}) is WGS84 but policy "
                f"{getattr(policy, 'policy_id', '?')!r} declares no origin to "
                f"project it about")
        from .projection import LocalProjection
        return LocalProjection(origin.lat, origin.lon).to_local(self.lat, self.lon)


class PilotSpec(_Strict):
    """What the stand-in for the VLA asks for. Deliberately dumb: a pilot that
    never asks for anything illegal leaves the Shield nothing to do."""
    type: Literal["goto", "constant"]
    speed: float | None = Field(default=None, gt=0)      # goto, m/s
    action: dict[str, float] | None = None               # constant: Action4D fields

    @model_validator(mode="after")
    def _shape(self) -> "PilotSpec":
        if self.type == "constant":
            if self.action is None:
                raise ValueError("a constant pilot needs `action`")
            bad = set(self.action) - {"vx", "vy", "vz_up", "yaw_rate"}
            if bad:
                raise ValueError(f"unknown action channel(s) {sorted(bad)}")
        if self.type == "goto" and self.action is not None:
            raise ValueError("a goto pilot flies to mission.target; it takes no `action`")
        return self


class Reclassify(_Strict):
    at_s: float = Field(ge=0)
    class_: str = Field(alias="class")


class SubjectSpec(_Strict):
    """A declared subject for stand-off rules, optionally re-labelled mid-run."""
    x: float
    y: float
    class_: str | None = Field(default=None, alias="class")
    reclassify: list[Reclassify] = Field(default_factory=list)


class XY(_Strict):
    x: float
    y: float


class Mission(_Strict):
    """The grant example's mission (task_prompt, start_pose, target), plus the
    pieces a headless run needs and the grant does not name."""
    task_prompt: str = ""
    start_pose: Pose
    target: Pose | None = None
    pilot: PilotSpec
    subject: SubjectSpec | None = None
    # The scenario clock's reading at t = 0. It ADVANCES with simulated time, so
    # a run that starts at 17:30:45 crosses a 17:30 window edge mid-flight -
    # the "time-window switch instant" of the Shield's test matrix. Absent means
    # no clock, and with no clock every scheduled rule is in force
    # (ConstraintBase.active_at: absent always means active).
    clock_start: datetime | None = None
    # Rigid rotation of the whole mission (start, target, subject, a constant
    # action) about `rotate_about` (default: the start position). How one
    # family is flown at many approach angles without hand-writing each.
    rotate_deg: float = 0.0
    rotate_about: XY | None = None

    @model_validator(mode="after")
    def _altitude(self) -> "Mission":
        if self.start_pose.altitude is None:
            raise ValueError("start_pose needs an altitude (`up` or `alt_agl_m`)")
        return self


# --------------------------------------------------------------------------- #
# policy reference
# --------------------------------------------------------------------------- #

class PolicyRef(_Strict):
    """Which rules are in force. `bundle_path` / `expected_hash` are the grant
    example's names; `path` (a policy YAML) is how every scenario in this
    repository has referred to its policy.

    `expected_hash`, when given, is CHECKED: a scenario pinned to a policy
    that has since been edited stops at load instead of measuring a policy its
    author never saw. Both the full and the 16-hex legacy forms are accepted,
    because stored scenarios may carry either.
    """
    path: str | None = None
    bundle_path: str | None = None
    expected_hash: str | None = None

    @model_validator(mode="after")
    def _one_source(self) -> "PolicyRef":
        if (self.path is None) == (self.bundle_path is None):
            raise ValueError("policy needs exactly one of `path` or `bundle_path`")
        return self

    @property
    def source(self) -> str:
        return self.path or self.bundle_path

    def load(self, root: Path):
        """A FRESH Policy object every call. Hot-apply appends to the policy it
        is given, so sharing one across episodes would leak a spawned zone from
        one episode into every later one."""
        p = Path(self.source)
        p = p if p.is_absolute() else Path(root) / p
        if self.path is not None:
            from .models import load_policy
            pol = load_policy(p)
        else:
            from .bundle import load_bundle          # optional: needs cryptography
            pol = load_bundle(p)
        if self.expected_hash:
            have = {pol.policy_hash}
            short = getattr(pol, "policy_hash_short", None)
            if short:
                have.add(short)
            if self.expected_hash not in have:
                raise ValueError(
                    f"policy {self.source} hashes to {pol.policy_hash}, but the "
                    f"scenario pins {self.expected_hash}: the policy changed "
                    f"after the scenario was written")
        return pol


# --------------------------------------------------------------------------- #
# events, stress, expectations
# --------------------------------------------------------------------------- #

class ScenarioEvent(_Strict):
    """Copied from the grant field for field. The headless runner implements
    `spawn_polygon_fence` (through `Shield.hot_apply`); the other six are valid
    vocabulary that the runner REFUSES rather than skips, because the Shield
    has no API to move, remove, toggle or rescale a rule mid-flight."""
    at_sim_t: float = Field(ge=0)
    type: Literal[GRANT_EVENT_TYPES]
    payload: dict


class Stress(_Strict):
    """Stressors the headless harness can simulate without a simulator.

    All randomness is drawn from generators seeded by the episode's
    `random_seed`, one generator per stressor, so adding gusts to a scenario
    does not change the GPS noise sequence of the same seed.

    gps_noise_m      sigma of white Gaussian noise on the x, y and up the
                     Shield SEES (the vehicle moves on its true position).
                     White, not a random walk: the simpler model, and stated
                     as such. (A separate vertical sigma was dropped on
                     2026-10-06: no scenario used it and no test held it.)
    latency_ticks    the Shield sees the state from this many ticks ago.
    wind_speed_mps   constant wind added to the vehicle's velocity, never shown
                     to the Shield (it predicts with the commanded action only).
    wind_dir_deg     direction the wind blows TOWARD, degrees clockwise from
                     North (x), in the MISSION's frame: it turns with
                     mission.rotate_deg, so a family flown from four sides
                     keeps "a tailwind into the zone" a tailwind into the
                     zone. None = drawn from the seed (absolute).
    gust_mps         sigma of a per-tick Gaussian gust on each horizontal axis.
    start_jitter_m   uniform +/- jitter on the start x and y, drawn from the
                     seed (wind-gusts uses it, so its three nightly seeds
                     start from three places).
    gps_dropout_s    [start, end) in simulated seconds: GPS denial, as far as a
                     headless run can model it. The state the Shield sees
                     FREEZES at the last fix for the window - what a bridge
                     that keeps forwarding the last pose hands it - while the
                     vehicle keeps flying. The Shield's State has no
                     fix-quality field, so nothing tells it the pose is stale.
                     The grant names no Shield response to GPS loss; the
                     reference design lists "GPS denial" among the dynamic
                     actors stress scenarios need (docs/03-simulation/
                     layer-requirements.md, L2), which is all this answers.
    """
    gps_noise_m: float = Field(default=0.0, ge=0)
    latency_ticks: int = Field(default=0, ge=0)
    wind_speed_mps: float = Field(default=0.0, ge=0)
    wind_dir_deg: float | None = None
    gust_mps: float = Field(default=0.0, ge=0)
    start_jitter_m: float = Field(default=0.0, ge=0)
    gps_dropout_s: tuple[float, float] | None = None

    @model_validator(mode="after")
    def _dropout_window(self) -> "Stress":
        if self.gps_dropout_s is not None:
            lo, hi = self.gps_dropout_s
            if not (math.isfinite(lo) and math.isfinite(hi) and 0 <= lo < hi):
                raise ValueError("gps_dropout_s must be [start, end) with "
                                 "0 <= start < end")
        return self

    @property
    def degrades_perception(self) -> bool:
        return (self.gps_noise_m > 0 or self.latency_ticks > 0
                or self.gps_dropout_s is not None)

    @property
    def consumes_seed(self) -> bool:
        """Does this episode draw anything from its random_seed? A seed sweep
        over an episode that draws nothing produces identical repeats, and the
        harness reports that instead of counting them as spread."""
        return (self.gps_noise_m > 0
                or self.gust_mps > 0 or self.start_jitter_m > 0
                or (self.wind_speed_mps > 0 and self.wind_dir_deg is None))


Bound = Union[float, int, bool, str, None]


def normalise_expectation(key: str, value: Any) -> dict[str, Any]:
    """One expected-KPI entry -> the gate form {max|min|equals: v}.

    Accepted, as the grant's example writes them and as the old library did:
        0 / true / "pedestrian"   -> equals
        ">=0.99"  "<=2.0"  "==0"  -> min / max / equals
        {max: 0.0} {min: 1} {equals: true}
    Strict inequalities are refused: the gate language is inclusive, and
    silently reading ">0.99" as ">=0.99" would accept the boundary the author
    meant to exclude.
    """
    if isinstance(value, dict):
        bad = set(value) - {"max", "min", "equals"}
        if not value or bad:
            raise ValueError(f"expected_kpis.{key}: use max/min/equals, got {sorted(value)}")
        return dict(value)
    if isinstance(value, str):
        m = re.fullmatch(r"\s*(>=|<=|==|>|<)\s*(.+?)\s*", value)
        if m:
            op, rhs = m.groups()
            if op in (">", "<"):
                raise ValueError(f"expected_kpis.{key}: strict {op!r} is not "
                                 f"expressible; write >= or <=")
            try:
                num = float(rhs)
            except ValueError:
                raise ValueError(f"expected_kpis.{key}: {value!r} has no number")
            return {">=": {"min": num}, "<=": {"max": num}, "==": {"equals": num}}[op]
        return {"equals": value}
    if isinstance(value, (bool, int, float)):
        return {"equals": value}
    raise ValueError(f"expected_kpis.{key}: cannot read {value!r} as an expectation")


class KnownFailureCell(_Strict):
    """One parameter cell of a family pinned as an open defect.

    `expect: known_failure` marks a whole scenario; a family whose defect shows
    in SOME cells needs the same marker per cell, or the choice is between a
    red sweep on every build and dropping the cells that fail - and dropping
    them is hiding them. A pinned cell is reported as a known failure while it
    fails and as FIXED (a failure, on purpose) once it passes; every cell not
    pinned is still held to the family's gates, so a new defect is never
    absorbed by an old pin. `params` must identify the cell: a subset of its
    parameters, `variant` included for sampled families.
    """
    params: dict[str, Any] = Field(min_length=1)
    description: str = Field(min_length=1)

    def matches(self, cell_params: dict[str, Any]) -> bool:
        return all(k in cell_params and cell_params[k] == v
                   for k, v in self.params.items())


class SampleSpec(_Strict):
    """One seeded draw per variant: exactly one of uniform / randint / choice."""
    uniform: tuple[float, float] | None = None
    randint: tuple[int, int] | None = None
    choice: list[Any] | None = None

    @model_validator(mode="after")
    def _one(self) -> "SampleSpec":
        if sum(v is not None for v in (self.uniform, self.randint, self.choice)) != 1:
            raise ValueError("a sampled parameter needs exactly one of "
                             "uniform / randint / choice")
        if self.choice is not None and not self.choice:
            raise ValueError("choice needs at least one value")
        return self

    def draw(self, rng: random.Random):
        if self.uniform is not None:
            return round(rng.uniform(*self.uniform), 3)
        if self.randint is not None:
            return rng.randint(*self.randint)
        return rng.choice(self.choice)


# --------------------------------------------------------------------------- #
# the spec
# --------------------------------------------------------------------------- #

class ScenarioSpec(_Strict):
    # ---- the grant's fields, in the grant's order --------------------------
    scenario_id: str = Field(pattern=_ID)
    template: Literal[TEMPLATES]
    description: str = Field(min_length=1)
    mission: Mission
    policy: PolicyRef
    events: list[ScenarioEvent] = Field(default_factory=list)
    parameter_sweep: dict[str, list] = Field(default_factory=dict)
    expected_kpis: dict[str, Any]
    # ---- ours (see the module docstring for why each exists) ---------------
    expected_outcome: Literal[OUTCOMES] | None = None
    expected_failsafe: bool | None = None
    expect: Literal["pass", "known_failure", "tracked"] = "pass"
    known_failures: list[KnownFailureCell] = Field(default_factory=list)
    shield: bool = True
    stress: Stress = Field(default_factory=Stress)
    parameter_sample: dict[str, SampleSpec] = Field(default_factory=dict)
    ticks: int | None = Field(default=None, gt=0)
    dt: float | None = Field(default=None, gt=0)
    lookahead_s: float | None = Field(default=None, gt=0)

    @model_validator(mode="after")
    def _coherent(self) -> "ScenarioSpec":
        if not self.expected_kpis:
            raise ValueError(f"{self.scenario_id}: no expected_kpis - a scenario "
                             f"with no gate is a demonstration, not a test")
        for k, v in self.expected_kpis.items():
            normalise_expectation(k, v)                 # raises on a bad one
        if self.mission.pilot.type == "goto" and self.mission.target is None:
            raise ValueError(f"{self.scenario_id}: a goto pilot needs mission.target")
        for name, vals in self.parameter_sweep.items():
            if not isinstance(vals, list) or not vals:
                raise ValueError(f"{self.scenario_id}: parameter_sweep.{name} "
                                 f"must be a non-empty list")
        both = set(self.parameter_sweep) & set(self.parameter_sample)
        if both:
            raise ValueError(f"{self.scenario_id}: {sorted(both)} both swept and sampled")
        return self

    @property
    def gates(self) -> dict[str, dict[str, Any]]:
        """expected_kpis in the sweep's gate form, grant names resolved."""
        return {GRANT_KPI_ALIASES.get(k, k): normalise_expectation(k, v)
                for k, v in self.expected_kpis.items()}

    @property
    def declared_seeds(self) -> list[int] | None:
        s = self.parameter_sweep.get("random_seed")
        return [int(v) for v in s] if s else None

    def known_failure_for(self, cell_params: dict[str, Any]) -> str | None:
        """Why this cell is an expected failure, or None if it must pass."""
        if self.expect == "known_failure":
            return "the whole scenario is marked expect: known_failure"
        for pin in self.known_failures:
            if pin.matches(cell_params):
                return pin.description.strip()
        return None


# --------------------------------------------------------------------------- #
# families, cells, episodes
# --------------------------------------------------------------------------- #

def _behaviour(body: dict) -> dict:
    """`body` (a raw scenario mapping or a model dump) minus every field that
    does not change what is flown - see BEHAVIOUR_EXCLUDED."""
    out = {k: v for k, v in body.items() if k not in BEHAVIOUR_EXCLUDED}
    if isinstance(out.get("mission"), dict):
        out["mission"] = {k: v for k, v in out["mission"].items()
                          if k not in BEHAVIOUR_EXCLUDED_MISSION}
    return out


def _placeholders(node, found: set[str]) -> set[str]:
    if isinstance(node, dict):
        for v in node.values():
            _placeholders(v, found)
    elif isinstance(node, list):
        for v in node:
            _placeholders(v, found)
    elif isinstance(node, str):
        m = _PLACEHOLDER.match(node)
        if m:
            found.add(m.group(1))
    return found


def _substitute(node, values: dict[str, Any]):
    if isinstance(node, dict):
        return {k: _substitute(v, values) for k, v in node.items()}
    if isinstance(node, list):
        return [_substitute(v, values) for v in node]
    if isinstance(node, str):
        m = _PLACEHOLDER.match(node)
        if m:
            return values[m.group(1)]
    return node


def _fmt(v) -> str:
    if isinstance(v, float) and v.is_integer():
        return str(int(v))
    if isinstance(v, (list, tuple)):
        # A swept window [6.0, 11.0] reads "6-11" in the id, so `--only` can
        # name it without shell quoting; the exact values are in `params`.
        return "-".join(_fmt(x) for x in v)
    return str(v)


class Cell(BaseModel):
    """One concrete scenario: a family with every parameter bound."""
    model_config = ConfigDict(arbitrary_types_allowed=True)
    episode_id: str
    scenario_id: str
    params: dict[str, Any]
    spec: ScenarioSpec


class Family:
    """A ScenarioSpec as written in the library: possibly with placeholders,
    expanding to one or more Cells."""

    _BODY_EXCLUDE = ("parameter_sweep", "parameter_sample")

    def __init__(self, raw: dict, source: str = "?"):
        if not isinstance(raw, dict):
            raise ValueError(f"{source}: a scenario must be a mapping")
        self.raw = raw
        self.source = source
        self.scenario_id = str(raw.get("scenario_id", "?"))
        sweep = raw.get("parameter_sweep") or {}
        sample = raw.get("parameter_sample") or {}
        if not isinstance(sweep, dict) or not isinstance(sample, dict):
            raise ValueError(f"{self.scenario_id}: parameter_sweep / "
                             f"parameter_sample must be mappings")
        self.sweep = {k: v for k, v in sweep.items() if k not in HARNESS_PARAMS}
        self.sample = {k: SampleSpec.model_validate(v) for k, v in sample.items()}
        body = {k: v for k, v in raw.items() if k not in self._BODY_EXCLUDE}
        written = _placeholders(body, set())
        declared = set(self.sweep) | set(self.sample)
        missing = written - declared
        if missing:
            raise ValueError(f"{self.scenario_id}: placeholder(s) "
                             f"{sorted('$' + m for m in missing)} have no value in "
                             f"parameter_sweep or parameter_sample")
        # Read by something that is FLOWN. A parameter that only reaches the
        # description, the task prompt (nothing headless reads it), a gate or a
        # label makes cells that fly the same episode.
        flown = _placeholders(_behaviour(body), set())
        unused = declared - flown
        if unused:
            only_text = sorted(unused & written)
            raise ValueError(
                f"{self.scenario_id}: parameter(s) {sorted(unused)} are swept but "
                f"nothing in the scenario reads them, so every cell would be the "
                f"same episode"
                + (f" ({only_text} reach only the description, task prompt, "
                   f"gates or labels, which change nothing that is flown)"
                   if only_text else ""))
        for pin in raw.get("known_failures") or []:
            keys = set((pin or {}).get("params") or {})
            stray = keys - declared - ({"variant"} if self.sample else set())
            if stray:
                raise ValueError(f"{self.scenario_id}: known_failures pin names "
                                 f"{sorted(stray)}, which this scenario does not "
                                 f"sweep or sample - it could never match a cell")
        # Validated at LOAD, as the grant requires: build the first cell now so a
        # broken family fails before any episode runs, not halfway through one.
        self.first = self.cells()[0]

    @property
    def template(self) -> str:
        return self.first.spec.template

    def cells(self, sweep_overrides: dict[str, list] | None = None,
              samples: int = 1) -> list[Cell]:
        sweep = dict(self.sweep)
        for k, vals in (sweep_overrides or {}).items():
            if k not in sweep:
                raise ValueError(f"{self.scenario_id}: override for {k!r}, which "
                                 f"this scenario does not sweep "
                                 f"(it sweeps {sorted(sweep) or 'nothing'})")
            if not isinstance(vals, list) or not vals:
                raise ValueError(f"{self.scenario_id}: override {k} must be a "
                                 f"non-empty list")
            sweep[k] = vals
        n_samples = samples if self.sample else 1
        names = list(sweep)
        combos = list(itertools.product(*(sweep[n] for n in names))) or [()]
        out: list[Cell] = []
        for combo in combos:
            for k in range(n_samples):
                values = dict(zip(names, combo))
                if self.sample:
                    rng = random.Random(f"{self.scenario_id}:variant:{k}")
                    for pname in sorted(self.sample):
                        values[pname] = self.sample[pname].draw(rng)
                body = _substitute(self.raw, values)
                # The declared sweep stays on the cell's spec for the record,
                # minus the axes this cell has already fixed.
                spec = ScenarioSpec.model_validate(body)
                id_parts = {n: values[n] for n in names}
                params = dict(values)
                if self.sample:
                    id_parts["variant"] = k
                    params["variant"] = k
                out.append(Cell(episode_id=_episode_id(self.scenario_id, id_parts),
                                scenario_id=self.scenario_id, params=params,
                                spec=spec))
        return out


def _episode_id(sid: str, parts: dict[str, Any]) -> str:
    if not parts:
        return sid
    tail = ",".join(f"{k}={_fmt(parts[k])}" for k in sorted(parts))
    eid = f"{sid}@{tail}"
    if len(eid) > 96:
        # Kept short enough to survive inside a Windows path; the hash keeps it
        # unique and the full parameters are recorded beside it.
        eid = f"{sid}@{hashlib.sha1(tail.encode()).hexdigest()[:10]}"
    return eid


class Defaults(_Strict):
    dt: float = Field(default=0.1, gt=0)
    ticks: int = Field(default=300, gt=0)
    lookahead_s: float = Field(default=3.0, gt=0)


class Library:
    """A scenario library file: `defaults` plus a list of families."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        raw = yaml.safe_load(self.path.read_text(encoding="utf-8")) or {}
        extra = set(raw) - {"defaults", "scenarios"}
        if extra:
            raise ValueError(f"{self.path.name}: unknown top-level key(s) {sorted(extra)}")
        self.defaults = Defaults.model_validate(raw.get("defaults") or {})
        self.families: list[Family] = []
        seen: set[str] = set()
        for i, item in enumerate(raw.get("scenarios") or []):
            fam = Family(item, source=f"{self.path.name}[{i}]")
            if fam.scenario_id in seen:
                raise ValueError(f"{self.path.name}: duplicate scenario_id "
                                 f"{fam.scenario_id!r}")
            seen.add(fam.scenario_id)
            self.families.append(fam)
        if not self.families:
            raise ValueError(f"{self.path.name}: no scenarios")

    def by_id(self) -> dict[str, Family]:
        return {f.scenario_id: f for f in self.families}


class Profile(_Strict):
    """A per-profile sweep config (Stress Testing, Outputs: "Sweep config files
    - YAML, per nightly profile"), and the grant's scheduling row it answers to.

    `expected_cells` is the grant's "~N scenarios" written as the range the
    expansion must land in. experiments/sweep_scenarios.py REFUSES to run a
    full profile whose count falls outside it (only an explicit `--only`
    narrows a profile on purpose), so a profile that quietly shrinks to a
    dozen scenarios cannot keep calling itself the smoke set.
    """
    profile: str = Field(pattern=_ID)
    cadence: str
    grant_scope: str
    grant_sim_speedup: str
    library: str = "experiments/scenarios.yaml"
    seeds: list[int] = Field(min_length=1)
    samples_per_family: int = Field(default=1, ge=1)
    families: Union[Literal["all"], list[str]] = "all"
    sweep_overrides: dict[str, dict[str, list]] = Field(default_factory=dict)
    expected_cells: tuple[int, int]
    note: str = ""

    @model_validator(mode="after")
    def _range(self) -> "Profile":
        lo, hi = self.expected_cells
        if not 0 < lo <= hi:
            raise ValueError("expected_cells must be [lo, hi] with 0 < lo <= hi")
        if len(set(self.seeds)) != len(self.seeds):
            raise ValueError("seeds must be distinct")
        return self


def load_profile(path: str | Path) -> Profile:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    return Profile.model_validate(raw)


class Episode(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    cell: Cell
    seed: int

    @property
    def episode_id(self) -> str:
        return self.cell.episode_id

    def dir_name(self, stamp: str) -> str:
        """The grant's bundle name, `episode-<UTC>--<scenario>--seed<n>`."""
        safe = re.sub(r"[^A-Za-z0-9._=,@-]", "_", self.episode_id)
        return f"episode-{stamp}--{safe}--seed{self.seed}"


def expand(lib: Library, profile: Profile | None = None, seed: int | None = None,
           only: list[str] | None = None) -> tuple[list[Episode], dict[str, Any]]:
    """Every episode a run will execute, plus a record of how the count arose.

    Seeds, highest precedence first: `seed` (the --seed flag, one seed), the
    profile's `seeds`, a spec's own `parameter_sweep.random_seed` (the grant's
    form), and finally [0] - which is what every run used before seeds existed.

    `only` names families or cell ids. It is matched against the cells THIS run
    expands - with the profile's overrides and samples - because a cell id like
    `...@variant=2` exists only when the profile samples three variants. (The
    first version matched against the defaults and silently selected nothing.)
    Every name that matched no cell is listed in `info["only_unmatched"]`, and
    every cell that flies the same episode as an earlier one in
    `info["duplicate_cells"]`; the sweep refuses to run with either non-empty.
    """
    fams = lib.families
    if profile is not None and profile.families != "all":
        known = lib.by_id()
        unknown = [f for f in profile.families if f not in known]
        if unknown:
            raise ValueError(f"profile {profile.profile}: unknown families {unknown}")
        fams = [known[f] for f in profile.families]
    if profile is not None:
        unknown = set(profile.sweep_overrides) - {f.scenario_id for f in fams}
        if unknown:
            raise ValueError(f"profile {profile.profile}: overrides for families "
                             f"not in the run: {sorted(unknown)}")
    only_set = set(only or [])
    matched: set[str] = set()

    episodes: list[Episode] = []
    all_cells: list[Cell] = []
    n_cells = 0
    per_family: dict[str, int] = {}
    for fam in fams:
        overrides = profile.sweep_overrides.get(fam.scenario_id) if profile else None
        samples = profile.samples_per_family if profile else 1
        cells = fam.cells(overrides, samples)
        if only_set:
            keep = []
            for c in cells:
                hit = {fam.scenario_id, c.episode_id} & only_set
                if hit:
                    keep.append(c)
                    matched |= hit
            cells = keep
            if not cells:
                continue
        n_cells += len(cells)
        per_family[fam.scenario_id] = len(cells)
        all_cells.extend(cells)
        for c in cells:
            if seed is not None:
                seeds = [int(seed)]
            elif profile is not None:
                seeds = list(profile.seeds)
            else:
                seeds = c.spec.declared_seeds or [0]
            for s in seeds:
                episodes.append(Episode(cell=c, seed=s))

    info: dict[str, Any] = {"cells": n_cells, "episodes": len(episodes),
                            "cells_per_family": per_family,
                            "only_unmatched": sorted(only_set - matched),
                            "duplicate_cells": [list(p) for p in
                                                duplicate_cells(all_cells, lib.defaults)]}
    if profile is not None:
        lo, hi = profile.expected_cells
        info.update({"profile": profile.profile, "grant_scope": profile.grant_scope,
                     "cadence": profile.cadence,
                     "grant_sim_speedup": profile.grant_sim_speedup,
                     "seeds": [int(seed)] if seed is not None else profile.seeds,
                     "expected_cells": [lo, hi],
                     "cells_within_expected": lo <= n_cells <= hi})
    return episodes, info


def duplicate_cells(cells: list[Cell], defaults: "Defaults | None" = None
                    ) -> list[tuple[str, str]]:
    """Pairs (earlier, later) of cells that FLY THE SAME EPISODE. Non-empty
    means a count is padded.

    Compared on behaviour only (BEHAVIOUR_EXCLUDED): two cells that differ in
    their id, family name, description, task prompt, gates or labels are still
    one flight, and counting them twice is the padding this exists to catch. A
    cell that leaves dt / ticks / lookahead_s to the library defaults is
    compared with the defaults filled in, so writing `ticks: 300` explicitly
    does not make a new scenario either.
    """
    seen: dict[str, str] = {}
    dups = []
    dfl = defaults or Defaults()
    for c in cells:
        body = _behaviour(c.spec.model_dump(mode="json", by_alias=True))
        for k in ("dt", "ticks", "lookahead_s"):
            if body.get(k) is None:
                body[k] = getattr(dfl, k)
        key = repr(sorted(_flatten(body)))
        if key in seen:
            dups.append((seen[key], c.episode_id))
        else:
            seen[key] = c.episode_id
    return dups


def _flatten(node, prefix: str = "") -> list[tuple[str, str]]:
    if isinstance(node, dict):
        out = []
        for k, v in node.items():
            out.extend(_flatten(v, f"{prefix}.{k}"))
        return out
    if isinstance(node, list):
        out = []
        for i, v in enumerate(node):
            out.extend(_flatten(v, f"{prefix}[{i}]"))
        return out
    return [(prefix, repr(node))]
