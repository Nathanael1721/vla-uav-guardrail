"""The stress harness's gRPC control plane: StartEpisode, HotApply, EmitEvent,
GetStatus and EndEpisode over mutual TLS, driving the headless harness.

    python -m guardrail.harness_service gen-pki                 # lab CA + certs
    python -m guardrail.harness_service serve                   # mTLS on :50051
    python -m guardrail.harness_service serve --insecure        # loopback only
    python tools/harness_client.py run --scenario nfz-head-on --seed 1001

WHAT THE GRANT ASKS FOR

Stress Testing, "Locked design choices": "Control plane: gRPC over mTLS. Same
root CA as SROS 2 for cert reuse." Its Outputs table lists "RPC harness -
Python service + gRPC stubs", and its "Service surface (proto-ish)" names the
RPCs. The reference data-flow page adds why gRPC and not ROS 2 services: the
orchestrator "must also speak to non-ROS clients (CI, dashboards, scenario
authoring tools)". Audit cards WP4-05 (no gRPC service) and X-15 (no mTLS, no
shared root CA) recorded that none of it existed.

WHAT THIS IS

A thin transport over code that already runs. `Harness` (no gRPC in it) arms
an episode from a ScenarioSpec cell (guardrail/scenario_spec.py: Family,
episode_arms, Episode) and draws its arm (experiments/sweep_scenarios.py
build_arm), queues mid-flight events, and on EndEpisode flies it through the
sweep's other public functions - run_episode, score, episode_verdict,
episode_manifest, write_bundle, manifest_extras - so an episode started over
gRPC is flown, scored, judged and bundled exactly as the sweep does it. `StressHarnessServicer` maps
that onto the generated stubs (guardrail/protos/harness.proto); `serve()`
puts it on a port, with mTLS or, on loopback only, without.

EVENTS ARE SCHEDULED, NOT LIVE. EndEpisode flies a whole episode in one
run_episode call (about 0.3-0.4 s for 30 simulated seconds). So HotApply and
EmitEvent do not act "now": each carries `at_sim_t`, is validated when it
arrives, and is applied by the runner's own `apply_event` at that simulated
time when EndEpisode flies the episode. The flight is a pure function of
(cell, seed, arm, scheduled events), which is what makes it reproducible. A
live mode (events applied at the episode's current tick while it flies) needs
a steppable runner. That hook exists since 2026-10-11
(experiments/sweep_scenarios.py EpisodeRunner: step, inject, finish - the
integrator run_episode itself uses), but this service does not use it yet;
docs/DESIGN-harness-grpc.md lists the live mode as not built.

HotApply takes the Policy DSL's REST hot-apply body (guardrail/api.py:
dynamic_nfz, time_window_switch, corridor_swap) and runs it through the SAME
handler the REST endpoint runs (api.apply_event) on a dry-run Shield: the
injected policy with this episode's hot-apply bodies applied IN FLIGHT ORDER
(by at_sim_t, then by arrival - the order run_episode applies them), not in
the order they arrived. A body the REST endpoint would refuse at its place in
that order is refused here with the matching status (409 ALREADY_EXISTS, 404
NOT_FOUND, 422 INVALID_ARGUMENT); a body that would make an earlier-acked
hot-apply fail further down the timeline is FAILED_PRECONDITION. (Until 9 Oct
the dry run replayed arrival order: a switch at 5 s sent after its zone at
10 s was acked, then the flight failed at EndEpisode.) An accepted body
becomes the ScenarioEvent the runner applies (spawn_polygon_fence with the
zone's vertices, activate_rule / deactivate_rule, swap_corridor). A field
that ScenarioEvent cannot carry (a zone's `valid_time`, a switch's priority)
is refused UNIMPLEMENTED rather than dropped: a zone that flew without its
schedule would be a different zone.

THE DRY RUN KNOWS ONLY THE HOT-APPLIES. The scenario's own `events:` and the
episode's EmitEvents are applied by the runner against the flying vehicle
(a zone spawned `ahead_of_vehicle_m` needs the aircraft's position), so the
dry run cannot replay them. A HotApply aimed at a rule only they create is
NOT_FOUND (aim an EmitEvent at it instead), and a HotApply that collides with
one of them (the same zone id, say) is acked and then fails at EndEpisode
with FAILED_PRECONDITION. PolicyAck's generation counts hot-applies only, as
of that ack; the flown generations are in EpisodeBundle.applied_events.

LIFECYCLE. EndEpisode is idempotent: a second call returns the same bundle,
and a call that arrives while another call is flying the episode waits for
that flight and returns its bundle (or its failure). An episode left ARMED
with no StartEpisode / HotApply / EmitEvent on it for `armed_idle_ttl_s`
(default one hour) expires: it becomes FAILED with the reason, nothing flies,
and it stops counting against MAX_OPEN_EPISODES, so clients that give up on
an episode cannot wedge a long-running server.

WHAT IT IS NOT

  * Not KPI-grade. The headless backend's manifest names the topology
    `headless-kinematic` and a measured sim_speedup far above 1, so
    guardrail.manifest.is_kpi_grade refuses every episode, and EpisodeBundle
    says so (`kpi_grade`, `kpi_grade_reasons`). The grant's KPI numbers come
    from the hil topology (a Jetson Orin, not built yet).
  * Not the page's whole surface. StartSession / StopSession, InjectPolicy,
    IssueTask and the telemetry / event streams are absent from the .proto
    (gRPC answers UNIMPLEMENTED). The policy comes from the ScenarioSpec, and
    the headless pilot reads no task.
  * Not a scheduler. Episodes are flown one at a time per server (one lock
    around run_episode); the reference's answer to throughput (risk R9) is
    parallel hosts.

STATUS CODES (what a client can rely on)

    INVALID_ARGUMENT     the spec, cell, seed, arm or event fails validation
                         (pydantic's message is passed through); a REST 422
    NOT_FOUND            unknown scenario_id, cell, profile, episode, policy
                         file; a REST 404
    ALREADY_EXISTS       a REST 409 (a rule id already in use)
    OUT_OF_RANGE         an event after the episode's last tick
    PERMISSION_DENIED    a client certificate not on the allow-list; a
                         client-sent scenario whose policy lies outside the
                         server's policies/ or bundles/ folder
    FAILED_PRECONDITION  an event for an episode no longer ARMED (ended,
                         failed or expired); an episode the runner refused in
                         flight (see GetStatus.error); a paraphrase arm this
                         server cannot draw (no LLM client here, or no stored
                         set holding the prompt); a hot-apply that would make
                         an earlier-acked one fail later in the flight; a
                         package or source file missing on the server
    UNIMPLEMENTED        a hot-apply field the headless runner cannot carry
    RESOURCE_EXHAUSTED   too many open episodes, or events in one episode
    UNAVAILABLE          (from gRPC) a TLS handshake that failed: no client
                         certificate, one from another CA, or a plaintext
                         client on the mTLS port

MUTUAL TLS AND THE SHARED ROOT CA

`generate_pki` writes, with `cryptography`, a lab root CA (ECDSA P-256, the
key type SROS 2's keystore uses), a server certificate (SAN localhost and
127.0.0.1 by default, serverAuth) and a client certificate (clientAuth), all
signed by that CA. The keys are unencrypted (a development PKI), so the
default folder is OUTSIDE the checkout, in the user's local data folder
(%LOCALAPPDATA%/vla-guardrail/harness_pki on Windows, else
$XDG_DATA_HOME or ~/.local/share/vla-guardrail/harness_pki): the checkout may
sit in a cloud-synced folder (this one does: OneDrive), and a gitignored file
there is still uploaded. A folder inside OneDrive, Dropbox, Google Drive or
iCloud Drive is refused unless `allow_synced` (--allow-synced). On POSIX the
key files are chmod 600; on Windows chmod cannot do that, and the protection
is the folder's ACL (the per-user profile's). The server requires and
verifies a client certificate (`require_client_auth=True`). The file names
are sros2's own (ca.cert.pem, ca.key.pem), and docs/DESIGN-harness-grpc.md
gives the steps that make the same CA the identity and permissions CA of a
SROS 2 keystore - not run here (no ROS 2 on this host), so the ROS 2 graph is
still unsecured.

Because the same CA will sign every ROS 2 node's identity, a valid chain alone
would let any node's certificate drive the harness. The server therefore also
requires the client certificate's common name to be on an allow-list
(default: harness-client) and answers PERMISSION_DENIED otherwise.

STUBS

guardrail/protos/harness_pb2.py and harness_pb2_grpc.py are generated (see
guardrail/protos/__init__.py for the command) and imported lazily, as are
grpc, protobuf and cryptography: `import guardrail.harness_service` works
without them, and the first call that needs one raises HarnessUnavailable
naming the package.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import importlib.util
import ipaddress
import json
import math
import os
import re
import sys
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import yaml
from pydantic import ValidationError

from .scenario_spec import Episode, Family, Library, ScenarioEvent, episode_arms, load_profile

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PORT = 50051


def _default_pki_dir() -> Path:
    """The user's local (not roaming, not synced) data folder: the PKI's
    private keys must not live in the checkout, which may be cloud-synced."""
    if os.name == "nt" and os.environ.get("LOCALAPPDATA"):
        base = Path(os.environ["LOCALAPPDATA"])
    else:
        base = Path(os.environ.get("XDG_DATA_HOME") or Path.home() / ".local" / "share")
    return base / "vla-guardrail" / "harness_pki"


DEFAULT_PKI_DIR = _default_pki_dir()
DEFAULT_RUNS_DIR = ROOT / "demo" / "out" / "harness"
PROFILES_DIR = ROOT / "experiments" / "profiles"
CA_CN = "VLA Guardrail Lab Root CA"
SERVER_CN = "harness-server"
DEFAULT_CLIENT_CN = "harness-client"
# A client-sent scenario may name a policy only under these folders: the
# server reads the file, and a path elsewhere would let a client make it read
# any YAML on the host.
POLICY_ROOTS = ("policies", "bundles")
MAX_OPEN_EPISODES = 256          # ARMED or RUNNING at once
MAX_EVENTS_PER_EPISODE = 512
MAX_KEPT_FINISHED = 4096         # finished episodes kept for GetStatus / EndEpisode
ARMED_IDLE_TTL_S = 3600.0        # an ARMED episode with no call for this long expires
_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
_PROFILE = re.compile(r"^[a-z0-9][a-z0-9_-]{0,63}$")

# guardrail/api.py's HTTP status -> the gRPC code a client sees for it.
REST_TO_GRPC = {409: "ALREADY_EXISTS", 404: "NOT_FOUND", 422: "INVALID_ARGUMENT"}


class HarnessError(Exception):
    """A refused request: the gRPC status code name and the reason, in words."""

    def __init__(self, code: str, message: str):
        super().__init__(f"{code}: {message}")
        self.code = code
        self.message = message


class HarnessUnavailable(RuntimeError):
    """A package the harness needs (grpcio, protobuf, cryptography) or the
    source tree it drives (experiments/) is not there."""


# --------------------------------------------------------------------------- #
# lazily loaded dependencies
# --------------------------------------------------------------------------- #

_MODS: SimpleNamespace | None = None


def _grpc_modules() -> SimpleNamespace:
    """grpc, protobuf's json_format and the generated stubs, on first use."""
    global _MODS
    if _MODS is None:
        try:
            import grpc
            from google.protobuf import json_format, struct_pb2

            from .protos import harness_pb2, harness_pb2_grpc
        except ImportError as e:
            raise HarnessUnavailable(
                f"the gRPC harness needs grpcio >= 1.84.0 and protobuf >= 7.35.1 "
                f"(the versions guardrail/protos was generated with): {e}") from e
        except Exception as e:                                   # noqa: BLE001
            # The stubs check the installed versions as they import: protobuf
            # raises its runtime_version.VersionError and harness_pb2_grpc a
            # RuntimeError, neither an ImportError. Name them, do not trace.
            raise HarnessUnavailable(
                f"the installed grpcio / protobuf were refused by the generated "
                f"stubs, which need grpcio >= 1.84.0 and protobuf >= 7.35.1 (same "
                f"major): {type(e).__name__}: {e}") from e
        _MODS = SimpleNamespace(grpc=grpc, json_format=json_format, struct_pb2=struct_pb2,
                                pb=harness_pb2, pb_grpc=harness_pb2_grpc)
    return _MODS


def _source_module(name: str, path: Path):
    """A module of the source tree that is not part of the guardrail package
    (experiments/, tools/), loaded by path. pyproject.toml packages only
    guardrail/, so the service runs from a checkout, never from a wheel alone.
    A module already imported under `name` from the same file is reused, so
    the tests and the service share one copy."""
    mod = sys.modules.get(name)
    if mod is not None and Path(getattr(mod, "__file__", "")).resolve() == path.resolve():
        return mod
    if not path.is_file():
        raise HarnessUnavailable(f"{path} is missing: the harness service drives the "
                                 f"source tree's {path.parent.name}/{path.name}")
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    try:
        spec.loader.exec_module(mod)
    except BaseException:
        sys.modules.pop(name, None)
        raise
    return mod


def _sweep():
    return _source_module("sweep_scenarios", ROOT / "experiments" / "sweep_scenarios.py")


def _kpi_report():
    return _source_module("kpi_report", ROOT / "tools" / "kpi_report.py")


def _json_safe(node):
    """NaN and infinities as strings, so json.dumps(allow_nan=False) holds (a
    raw action is NaN by design in the non-finite-action scenarios)."""
    if isinstance(node, dict):
        return {k: _json_safe(v) for k, v in node.items()}
    if isinstance(node, (list, tuple)):
        return [_json_safe(v) for v in node]
    if isinstance(node, float) and not math.isfinite(node):
        return str(node)
    return node


# --------------------------------------------------------------------------- #
# the KPI-report row and its nulls
# --------------------------------------------------------------------------- #

# KpiRow field -> where guardrail.kpi.rollup() keeps it. Each score is followed
# by its null (CONTRIBUTING: every score needs its null).
_ROW_FIELDS: tuple[tuple[str, tuple[str, ...]], ...] = (
    ("p0_escape_rate", ("p0_escape_rate",)),
    ("null_passthrough_p0_escape_rate", ("null_passthrough_p0_escape_rate",)),
    ("fail_safe_correctness", ("failsafe_labelled_episodes", "failsafe_trigger_correctness")),
    ("null_never_trigger", ("failsafe_labelled_episodes", "null_never_trigger")),
    ("null_always_trigger", ("failsafe_labelled_episodes", "null_always_trigger")),
    ("mission_success_rate", ("mission_success_rate",)),
    ("null_hover_mission_success_rate", ("null_hover_mission_success_rate",)),
    ("mean_repair_count", ("mean_repair_count_per_episode",)),
    ("mean_repair_magnitude_mps", ("mean_repair_magnitude_mps",)),
    ("repair_success_rate", ("repair_success_rate",)),
    ("null_always_brake_repair_success_rate", ("null_always_brake_repair_success_rate",)),
    ("mean_time_to_safe_s", ("mean_time_to_safe_s",)),
)
KPI_ROW_FIELDS = tuple(name for name, _ in _ROW_FIELDS)


# Why a figure has no value, when the rollup does not say more precisely.
_UNSET_WHY = {
    "fail_safe_correctness": "no expected_failsafe label, or no escalation FSM ran",
    "null_never_trigger": "no expected_failsafe label, or no escalation FSM ran",
    "null_always_trigger": "no expected_failsafe label, or no escalation FSM ran",
    "mean_repair_magnitude_mps": "no repaired tick (or the Shield was off)",
}
# ... and the rollup's own status word, where it has one.
_UNSET_STATUS = {
    "repair_success_rate": "repair_success_status",
    "null_always_brake_repair_success_rate": "repair_success_status",
    "mean_time_to_safe_s": "time_to_safe_status",
}


def kpi_row(rollup: dict, family: str, cell: str, shield: str) -> dict:
    """One KPI-report row from guardrail.kpi.rollup()'s output. A figure the
    rollup does not carry as a finite number is LEFT OUT and named in `unset`
    with the reason (the rollup's status word where it has one, e.g.
    "never_unsafe" for a time to safe): the proto field stays unset, never 0."""
    row: dict[str, Any] = {"family": family, "cell": cell, "shield": shield,
                           "episodes": int(rollup.get("episodes") or 0)}
    unset: dict[str, str] = {}
    for name, path in _ROW_FIELDS:
        v: Any = rollup
        for part in path:
            v = v.get(part) if isinstance(v, dict) else None
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            status = rollup.get(_UNSET_STATUS.get(name, ""))
            unset[name] = str(status) if status else _UNSET_WHY.get(name, "not measured")
        else:
            row[name] = float(v)
    row["unset"] = unset
    return row


# --------------------------------------------------------------------------- #
# HotApply body -> the ScenarioEvent the headless runner applies
# --------------------------------------------------------------------------- #

# What each runner event can carry, per rule class (sweep_scenarios.apply_event
# builds the rule from exactly these). Anything else the rule sets is refused.
_CARRIED = {
    "dynamic_nfz": {"id", "type", "vertices", "altitude_floor_m", "altitude_ceiling_m",
                    "margin_m", "priority", "constraint_type", "violation_action",
                    "motion"},
    "time_window_switch": {"id", "type", "target_id", "active", "valid_time"},
    "corridor_swap": {"id", "type", "target_id", "centerline", "width_m",
                      "altitude_floor_m", "altitude_ceiling_m", "valid_time"},
}


def _window_of(rule) -> dict | None:
    vt = getattr(rule, "valid_time", None)
    if vt is None or vt.recurrence is None:
        return None
    return vt.recurrence.model_dump(mode="json")


def scenario_event_from_rule(rule, at_sim_t: float) -> dict:
    """The ScenarioEvent (as a dict) that makes the runner apply `rule` - the
    rule a hot-apply body created on the dry-run Shield - at `at_sim_t`.

    Raises HarnessError(UNIMPLEMENTED) when the rule sets a field the runner's
    event cannot carry: flying the rule without it would fly another rule."""
    kind = rule.type
    if kind not in _CARRIED:
        raise HarnessError("INVALID_ARGUMENT", f"{kind!r} is not hot-applicable")
    set_fields = set(rule.model_dump(exclude_defaults=True, exclude_none=True))
    vt = getattr(rule, "valid_time", None)
    if vt is not None and vt.recurrence is None:
        set_fields.discard("valid_time")                 # no recurrence = always
    extra = sorted(set_fields - _CARRIED[kind])
    if extra:
        raise HarnessError(
            "UNIMPLEMENTED",
            f"{kind} {rule.id!r} sets {extra}, which the headless runner's event "
            f"cannot carry (experiments/sweep_scenarios.py apply_event builds the "
            f"rule from {sorted(_CARRIED[kind] - {'type'})}); flying it without "
            f"them would fly a different rule")
    if kind == "dynamic_nfz":
        payload: dict[str, Any] = {
            "id": rule.id,
            "vertices": [{"x": float(v.x), "y": float(v.y)} for v in rule.vertices],
            "altitude_floor_m": float(rule.altitude_floor_m),
            "altitude_ceiling_m": float(rule.altitude_ceiling_m),
            "margin_m": float(rule.margin_m), "priority": rule.priority,
            "constraint_type": rule.constraint_type,
            "violation_action": rule.violation_action}
        if rule.motion is not None:
            payload["motion"] = rule.motion.model_dump(mode="json")
        return {"at_sim_t": at_sim_t, "type": "spawn_polygon_fence", "payload": payload}
    if kind == "time_window_switch":
        payload = {"rule_id": rule.target_id, "id": rule.id}
        win = _window_of(rule)
        if win is not None:
            payload["window"] = win
        return {"at_sim_t": at_sim_t,
                "type": "activate_rule" if rule.active else "deactivate_rule",
                "payload": payload}
    payload = {"id": rule.id, "target_id": rule.target_id,
               "centerline": [{"x": float(v.x), "y": float(v.y)} for v in rule.centerline],
               "width_m": float(rule.width_m),
               "altitude_floor_m": float(rule.altitude_floor_m),
               "altitude_ceiling_m": float(rule.altitude_ceiling_m)}
    win = _window_of(rule)
    if win is not None:
        payload["window"] = win
    return {"at_sim_t": at_sim_t, "type": "swap_corridor", "payload": payload}


def schedule_problems(spec, t_end: float, extra_events=()) -> list[str]:
    """What run_episode would refuse about the schedule, found before anything
    flies: an event after the last tick (it never fires), a windowed event with
    no mission clock (its window would always be in force), a GPS dropout that
    starts after the run. The same three checks run_episode makes."""
    out = []
    for e in list(spec.events) + list(extra_events):
        if e.at_sim_t > t_end + 1e-9:
            out.append(f"{e.type} at {e.at_sim_t} s never fires: the run ends at "
                       f"{t_end:.1f} s")
        if "window" in e.payload and spec.mission.clock_start is None:
            out.append(f"{e.type} with a `window` needs mission.clock_start: with no "
                       f"clock the window is always in force")
    drop = spec.stress.gps_dropout_s
    if drop is not None and drop[0] > t_end + 1e-9:
        out.append(f"gps_dropout_s starts at {drop[0]} s and never happens: the run "
                   f"ends at {t_end:.1f} s")
    return out


# --------------------------------------------------------------------------- #
# the harness itself (no transport)
# --------------------------------------------------------------------------- #

@dataclass
class _Queued:
    index: int
    event: Any                       # ScenarioEvent
    source: str                      # "hot_apply" | "emit_event"
    rule_id: str
    received: dict                   # the request as it arrived


@dataclass
class _HotBody:
    """One accepted hot-apply body and where it sits in the flight."""
    at: float                        # at_sim_t
    index: int                       # its queue index (arrival order)
    body: dict


def _flight_order(bodies) -> list:
    """Hot-apply bodies in the order run_episode applies them: by at_sim_t,
    ties by arrival (its stable sort of the cell's events + the queue)."""
    return sorted(bodies, key=lambda b: (b.at, b.index))


@dataclass
class _Episode:
    episode_id: str
    ep: Any                          # scenario_spec.Episode
    profile: str
    defaults: Any                    # scenario_spec.Defaults
    keep_log: bool
    t_end: float
    policy: Any                      # the injected policy; the dry run starts from it
    policy_hash: str
    request: dict
    client: str
    dry: Any                         # Shield after every hot-apply, in flight order
    arm_record: dict | None = None   # sweep_scenarios.build_arm, drawn at StartEpisode
    hot_bodies: list = field(default_factory=list)       # [_HotBody], arrival order
    queued: list = field(default_factory=list)
    state: str = "ARMED"
    result: dict | None = None
    error: str = ""
    idle_since: float = field(default_factory=time.monotonic)   # last accepted call
    finished_at: float = 0.0
    waiters: int = 0                 # EndEpisode calls waiting on another's flight
    done: threading.Event = field(default_factory=threading.Event)
    lock: threading.Lock = field(default_factory=threading.Lock)


def _finite(v, what: str) -> float:
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise HarnessError("INVALID_ARGUMENT", f"{what} must be a number, got {v!r}") from None
    if not math.isfinite(f):
        raise HarnessError("INVALID_ARGUMENT", f"{what} must be finite, got {v!r}")
    return f


class Harness:
    """Episodes armed, fed events and flown; no gRPC in here (the servicer
    below is a translation of this onto the stubs, and tests drive both)."""

    def __init__(self, runs_dir: str | Path | None = None,
                 library: str | Path | None = None,
                 manifest_base: dict | None = None,
                 armed_idle_ttl_s: float | None = None):
        self.runs_dir = Path(runs_dir) if runs_dir else DEFAULT_RUNS_DIR
        self.armed_idle_ttl_s = float(ARMED_IDLE_TTL_S if armed_idle_ttl_s is None
                                      else armed_idle_ttl_s)
        if not self.armed_idle_ttl_s >= 0:
            raise ValueError(f"armed_idle_ttl_s must be >= 0, got {armed_idle_ttl_s!r}")
        self._S = _sweep()
        self._default_lib = Path(library) if library else Path(self._S.LIB)
        self._libs: dict[Path, Library] = {}
        # Resolved ONCE, at construction: the code this process runs is the
        # code it imported at start, so a checkout changed under a running
        # server must not move the revision its manifests record.
        self._manifest_base = dict(manifest_base or self._S.manifest_base())
        self._episodes: dict[str, _Episode] = {}
        self._lock = threading.Lock()
        self._run_lock = threading.Lock()

    # ---- lookups -------------------------------------------------------- #

    def _library(self, path: str | Path | None) -> Library:
        p = Path(path) if path else self._default_lib
        p = p if p.is_absolute() else ROOT / p
        with self._lock:
            lib = self._libs.get(p)
        if lib is None:
            lib = Library(p)
            with self._lock:
                self._libs[p] = lib
        return lib

    def _profile(self, name: str):
        if not name:
            return None
        if not _PROFILE.match(name):
            raise HarnessError("INVALID_ARGUMENT",
                               f"profile {name!r}: a name such as smoke, nightly or broad")
        p = PROFILES_DIR / f"{name}.yaml"
        if not p.is_file():
            have = sorted(q.stem for q in PROFILES_DIR.glob("*.yaml"))
            raise HarnessError("NOT_FOUND", f"no profile {name!r} (have {have})")
        try:
            return load_profile(p)
        except (ValidationError, ValueError) as e:
            raise HarnessError("INVALID_ARGUMENT", f"profile {name!r}: {e}") from None

    def _family_from_yaml(self, text: str) -> Family:
        try:
            raw = yaml.safe_load(text)
        except yaml.YAMLError as e:
            raise HarnessError("INVALID_ARGUMENT", f"scenario_yaml is not YAML: {e}") from None
        if not isinstance(raw, dict):
            raise HarnessError("INVALID_ARGUMENT",
                               "scenario_yaml must be one ScenarioSpec family (a mapping)")
        if {"scenarios", "templates"} & set(raw):
            raise HarnessError("INVALID_ARGUMENT",
                               "scenario_yaml is a library; send one family")
        try:
            return Family(raw, source="grpc")
        except Exception as e:                                   # noqa: BLE001
            # Whatever Family raises here, the client's text is what failed
            # (pydantic's ValidationError, the loader's ValueError, a TypeError
            # from a list where a mapping belongs).
            raise HarnessError("INVALID_ARGUMENT", f"scenario refused by the schema: "
                                                   f"{type(e).__name__}: {e}") from None

    def _policy_allowed(self, spec) -> None:
        p = Path(spec.policy.source)
        p = (p if p.is_absolute() else ROOT / p).resolve()
        roots = [(ROOT / d).resolve() for d in POLICY_ROOTS]
        if not any(p == r or p.is_relative_to(r) for r in roots):
            raise HarnessError("PERMISSION_DENIED",
                               f"policy {spec.policy.source!r} is outside the server's "
                               f"{' and '.join(d + '/' for d in POLICY_ROOTS)}")

    def _get(self, episode_id: str) -> _Episode:
        with self._lock:
            e = self._episodes.get(episode_id)
        if e is None:
            raise HarnessError("NOT_FOUND", f"no episode {episode_id!r}")
        return e

    def _expire_if_idle(self, e: _Episode, now: float | None = None) -> bool:
        """With e.lock held: an ARMED episode idle for the TTL or longer
        becomes FAILED (nothing flew), so it stops holding an open slot.
        ("Or longer", not "longer": Windows' monotonic clock ticks every
        ~16 ms, and a TTL of 0 must also expire an episode created in the
        same tick.)"""
        now = time.monotonic() if now is None else now
        if e.state != "ARMED" or now - e.idle_since < self.armed_idle_ttl_s:
            return False
        e.state = "FAILED"
        e.error = (f"expired: ARMED and idle for {now - e.idle_since:.0f} s (the "
                   f"server's limit is {self.armed_idle_ttl_s:g} s) with no "
                   f"HotApply, EmitEvent or EndEpisode; nothing flew")
        e.finished_at = now
        e.done.set()
        return True

    def _armed(self, e: _Episode) -> None:
        self._expire_if_idle(e)
        if e.state != "ARMED":
            why = f" ({e.error})" if e.state == "FAILED" and e.error else ""
            raise HarnessError("FAILED_PRECONDITION",
                               f"episode {e.episode_id} is {e.state}{why}; events are "
                               f"accepted only before EndEpisode")

    def _evict_locked(self) -> None:
        """With self._lock held: forget the oldest finished episodes beyond
        MAX_KEPT_FINISHED (their bundles stay on disk)."""
        done = [x for x in self._episodes.values() if x.state in ("ENDED", "FAILED")]
        if len(done) > MAX_KEPT_FINISHED:
            for x in sorted(done, key=lambda x: x.finished_at)[:len(done) - MAX_KEPT_FINISHED]:
                self._episodes.pop(x.episode_id, None)

    def _dry_shield(self, e: _Episode, bodies, new: _HotBody | None = None):
        """A fresh Shield on the injected policy with `bodies` (and `new`)
        applied IN FLIGHT ORDER (_flight_order), through the REST endpoint's
        own handler: the state a hot-apply is validated against. Rebuilt per
        request, so a body refused half-way leaves nothing behind.

        Returns (shield, apply_event's result for `new`, `new`'s rule as it
        stood right after `new` was applied - a later body may change it).
        A refusal of `new` itself carries the REST endpoint's code; a body
        that `new` makes fail further down the timeline is
        FAILED_PRECONDITION, naming it."""
        from . import api
        from .shield import Shield
        sh = Shield(e.policy.model_copy(deep=True), escalation=False)
        res = rule = None
        for b in _flight_order([*bodies, *([new] if new is not None else [])]):
            try:
                out = api.apply_event(sh, dict(b.body))
            except api.ApiError as err:
                if b is new:
                    raise HarnessError(REST_TO_GRPC.get(err.status, "INVALID_ARGUMENT"),
                                       f"refused as the REST hot-apply endpoint refuses it "
                                       f"({err.status}) at {b.at:g} s in flight order: "
                                       f"{err.detail}") from None
                if new is None:
                    # Every body here was accepted in this very order before.
                    raise HarnessError("INTERNAL", f"hot-apply #{b.index} (at {b.at:g} s) "
                                                   f"no longer applies: {err.detail}") from None
                raise HarnessError(
                    "FAILED_PRECONDITION",
                    f"at {new.at:g} s this body would make hot-apply #{b.index} (at "
                    f"{b.at:g} s, already acknowledged) fail as the REST endpoint would "
                    f"({err.status}): {err.detail}") from None
            if b is new:
                res = out
                rule = next(c for c in sh.policy.constraints
                            if c.id == out["applied"]).model_copy(deep=True)
        return sh, res, rule

    # ---- the RPCs ------------------------------------------------------- #

    def start_episode(self, *, scenario_id: str = "", scenario_yaml: str = "",
                      cell_id: str = "", seed: int | None = None, profile: str = "",
                      arm: str = "", keep_flight_log: bool = False,
                      client: str = "") -> dict:
        if bool(scenario_id) == bool(scenario_yaml):
            raise HarnessError("INVALID_ARGUMENT",
                               "give exactly one of scenario_id or scenario_yaml")
        prof = self._profile(profile)
        lib = self._library(prof.library if prof else None)
        if scenario_id:
            fam = lib.by_id().get(scenario_id)
            if fam is None:
                raise HarnessError("NOT_FOUND",
                                   f"no scenario {scenario_id!r} in {lib.path.name}")
        else:
            fam = self._family_from_yaml(scenario_yaml)
        if prof is not None and prof.families != "all" \
                and fam.scenario_id not in prof.families:
            raise HarnessError("INVALID_ARGUMENT", f"{fam.scenario_id} is not one of "
                                                   f"profile {prof.profile}'s families")
        overrides = prof.sweep_overrides.get(fam.scenario_id) if prof else None
        try:
            cells = fam.cells(overrides, prof.samples_per_family if prof else 1)
        except (ValidationError, ValueError) as e:
            raise HarnessError("INVALID_ARGUMENT", str(e)) from None
        ids = [c.episode_id for c in cells]
        if cell_id:
            pick = [c for c in cells if c.episode_id == cell_id]
            if not pick:
                raise HarnessError("NOT_FOUND", f"{fam.scenario_id} has no cell "
                                                f"{cell_id!r} (cells: {ids[:12]}"
                                                f"{' ...' if len(ids) > 12 else ''})")
            cell = pick[0]
        elif len(cells) == 1:
            cell = cells[0]
        else:
            raise HarnessError("INVALID_ARGUMENT",
                               f"{fam.scenario_id} expands to {len(cells)} cells; name "
                               f"one in cell_id (e.g. {ids[:6]})")
        spec = cell.spec
        if scenario_yaml:
            self._policy_allowed(spec)

        declared = spec.declared_seeds
        if seed is not None:
            s = int(seed)
            if prof is not None and s not in prof.seeds:
                raise HarnessError("INVALID_ARGUMENT", f"seed {s} is not one of profile "
                                                       f"{prof.profile}'s seeds {prof.seeds}")
        elif prof is not None:
            if len(prof.seeds) != 1:
                raise HarnessError("INVALID_ARGUMENT", f"profile {prof.profile} has seeds "
                                                       f"{prof.seeds}; name one")
            s = int(prof.seeds[0])
        elif declared and len(declared) > 1:
            raise HarnessError("INVALID_ARGUMENT", f"{fam.scenario_id} declares seeds "
                                                   f"{declared}; name one")
        else:
            s = int(declared[0]) if declared else 0

        pp, prefixes = episode_arms(spec, prof)
        arms = [Episode(cell=cell, seed=s, paraphrase_arm=pa, prefix=pfx,
                        ab=len(prefixes) > 1) for pfx in prefixes for pa in pp]
        if arm:
            chosen = [a for a in arms if a.arm_tag == arm]
            if not chosen:
                raise HarnessError("INVALID_ARGUMENT", f"no arm {arm!r}; this cell's "
                                                       f"arms: {[a.arm_tag for a in arms]}")
            ep = chosen[0]
        else:
            ep = arms[0]

        dt = float(spec.dt or lib.defaults.dt)
        ticks = int(spec.ticks or lib.defaults.ticks)
        t_end = (ticks - 1) * dt
        probs = schedule_problems(spec, t_end)
        if probs:
            raise HarnessError("INVALID_ARGUMENT", "; ".join(probs))
        try:
            policy, _source = spec.policy.load_with_source(ROOT)
        except FileNotFoundError as e:
            raise HarnessError("NOT_FOUND", f"policy: {e}") from None
        except (ValidationError, ValueError, OSError) as e:
            raise HarnessError("INVALID_ARGUMENT",
                               f"policy {spec.policy.source!r}: {e}") from None
        arm_record = self._arm_record(ep)

        eid = "ep-" + uuid.uuid4().hex[:12]
        e = _Episode(episode_id=eid, ep=ep, profile=prof.profile if prof else "",
                     defaults=lib.defaults, keep_log=bool(keep_flight_log), t_end=t_end,
                     policy=policy.model_copy(deep=True), policy_hash=policy.policy_hash,
                     request={"scenario_id": scenario_id or None,
                              "scenario_yaml": scenario_yaml or None,
                              "cell_id": cell_id or None,
                              "seed": None if seed is None else int(seed),
                              "profile": profile or None, "arm": arm or None,
                              "keep_flight_log": bool(keep_flight_log)},
                     client=client, dry=None, arm_record=arm_record)
        e.dry = self._dry_shield(e, [])[0]
        with self._lock:                 # the count and the insert under one lock
            # Idle ARMED episodes expire first. Lock order: self._lock, then an
            # episode's lock (no path takes self._lock while holding e.lock);
            # one that is busy right now is not idle, so it is skipped.
            now = time.monotonic()
            expired = False
            for x in list(self._episodes.values()):
                if x.state == "ARMED" and x.lock.acquire(blocking=False):
                    try:
                        expired |= self._expire_if_idle(x, now)
                    finally:
                        x.lock.release()
            if expired:
                self._evict_locked()
            open_n = sum(1 for x in self._episodes.values() if x.state in ("ARMED", "RUNNING"))
            if open_n >= MAX_OPEN_EPISODES:
                raise HarnessError("RESOURCE_EXHAUSTED", f"{open_n} episodes are open; end "
                                                         f"some first")
            self._episodes[eid] = e
        return self._ref(e)

    def _arm_record(self, ep) -> dict | None:
        """The episode's arm record (its paraphrase draw and prefix key), built
        now rather than at EndEpisode, so a scenario this server cannot draw
        an arm for is refused before an episode exists. A schema-valid
        `paraphrase: {backend: llm:<id>}` needs an LLM client this build does
        not configure, and a `stored` backend needs a stored set holding the
        prompt: both are FAILED_PRECONDITION (the server lacks what the spec
        asks for), with the paraphraser's own words. The draw is a function of
        (cell, seed, arm), so building it early changes nothing that flies."""
        from .paraphraser import ParaphraserError, ProtectedTextError
        try:
            with self._run_lock:                 # the sweep's arm cache is module state
                return self._S.build_arm(ep)
        except ProtectedTextError as err:
            raise HarnessError("INVALID_ARGUMENT", f"arm {ep.arm_tag}: the instruction "
                                                   f"cannot be paraphrased: {err}") from None
        except (ParaphraserError, OSError) as err:
            raise HarnessError("FAILED_PRECONDITION",
                               f"arm {ep.arm_tag}: this server cannot draw its paraphrase "
                               f"({type(err).__name__}: {err}); the canonical arm draws "
                               f"none") from None
        except ValueError as err:
            raise HarnessError("INVALID_ARGUMENT", f"arm {ep.arm_tag}: {err}") from None

    def _ref(self, e: _Episode) -> dict:
        return {"episode_id": e.episode_id, "scenario_id": e.ep.cell.scenario_id,
                "cell_id": e.ep.episode_id, "run_id": e.ep.run_id, "seed": e.ep.seed,
                "arm": e.ep.arm_tag, "profile": e.profile,
                "run_end_sim_t": round(e.t_end, 6), "policy_hash": e.policy_hash}

    def _at(self, e: _Episode, at_sim_t) -> float:
        at = _finite(at_sim_t, "at_sim_t")
        if at < 0:
            raise HarnessError("INVALID_ARGUMENT", f"at_sim_t {at} is before the episode")
        if at > e.t_end + 1e-9:
            raise HarnessError("OUT_OF_RANGE", f"at_sim_t {at} s never fires: episode "
                                               f"{e.episode_id} ends at {e.t_end:.1f} s")
        return at

    def hot_apply(self, episode_id: str, at_sim_t: float, body: dict) -> dict:
        e = self._get(episode_id)
        with e.lock:
            self._armed(e)
            at = self._at(e, at_sim_t)
            if not isinstance(body, dict) or not body:
                raise HarnessError("INVALID_ARGUMENT", "body must be a non-empty object "
                                                       "with a `type`")
            if len(e.queued) >= MAX_EVENTS_PER_EPISODE:
                raise HarnessError("RESOURCE_EXHAUSTED",
                                   f"{len(e.queued)} events queued in {episode_id}")
            new = _HotBody(at=at, index=len(e.queued), body=dict(body))
            # Validated where it will fly (at_sim_t order), not where it arrived.
            scratch, res, rule = self._dry_shield(e, e.hot_bodies, new)
            ev_dict = scenario_event_from_rule(rule, at)
            try:
                ev = ScenarioEvent.model_validate(ev_dict)
            except ValidationError as err:            # a translation bug, not the client's
                raise HarnessError("INTERNAL", f"the translated event does not validate: "
                                               f"{err}") from None
            probs = schedule_problems(e.ep.cell.spec, e.t_end, [ev])
            if probs:
                raise HarnessError("FAILED_PRECONDITION", "; ".join(probs))
            e.dry = scratch
            e.hot_bodies.append(new)
            q = _Queued(index=new.index, event=ev, source="hot_apply", rule_id=rule.id,
                        received={"rpc": "HotApply", "at_sim_t": at, "body": body})
            e.queued.append(q)
            e.idle_since = time.monotonic()
            return {"episode_id": e.episode_id, "queue_index": q.index, "at_sim_t": at,
                    "rule_id": rule.id, "op": res.get("op", ""), "type": res.get("type", ""),
                    "validated_generation": int(res.get("generation", 0)),
                    "validated_policy_hash": res.get("policy_hash", ""),
                    "scenario_event_json": json.dumps(ev.model_dump(mode="json"),
                                                      sort_keys=True)}

    def emit_event(self, episode_id: str, at_sim_t: float, type_: str,
                   payload: dict) -> dict:
        e = self._get(episode_id)
        with e.lock:
            self._armed(e)
            at = self._at(e, at_sim_t)
            if len(e.queued) >= MAX_EVENTS_PER_EPISODE:
                raise HarnessError("RESOURCE_EXHAUSTED",
                                   f"{len(e.queued)} events queued in {episode_id}")
            try:
                ev = ScenarioEvent.model_validate({"at_sim_t": at, "type": type_,
                                                   "payload": dict(payload or {})})
            except ValidationError as err:
                raise HarnessError("INVALID_ARGUMENT", f"ScenarioEvent refused: {err}") from None
            probs = schedule_problems(e.ep.cell.spec, e.t_end, [ev])
            if probs:
                raise HarnessError("FAILED_PRECONDITION", "; ".join(probs))
            p = ev.payload
            rid = str(p.get("id") or p.get("rule_id") or p.get("target_id") or "")
            q = _Queued(index=len(e.queued), event=ev, source="emit_event", rule_id=rid,
                        received={"rpc": "EmitEvent", "at_sim_t": at, "type": type_,
                                  "payload": payload})
            e.queued.append(q)
            e.idle_since = time.monotonic()
            return {"episode_id": e.episode_id, "queue_index": q.index, "at_sim_t": at,
                    "type": ev.type}

    def status(self, episode_id: str) -> dict:
        e = self._get(episode_id)
        with e.lock:
            self._expire_if_idle(e)
            res = e.result or {}
            return {**self._ref(e), "state": e.state, "client": e.client,
                    "events": [{"index": q.index, "at_sim_t": q.event.at_sim_t,
                                "type": q.event.type, "source": q.source,
                                "rule_id": q.rule_id} for q in e.queued],
                    "status": res.get("status", ""), "bundle_path": res.get("bundle_path", ""),
                    "error": e.error}

    def end_episode(self, episode_id: str) -> dict:
        """Fly, score and bundle the episode. Idempotent: an ENDED episode
        returns its bundle again, and a call that finds another call flying
        it waits for that flight and returns the same bundle (or its
        failure) - a client that retries after its deadline expired while it
        queued gets the episode, not an error."""
        e = self._get(episode_id)
        with e.lock:
            self._expire_if_idle(e)
            if e.state == "ENDED":
                return e.result
            if e.state == "FAILED":
                raise HarnessError("FAILED_PRECONDITION",
                                   f"episode {episode_id} failed: {e.error}")
            flying_elsewhere = e.state == "RUNNING"
            if flying_elsewhere:
                e.waiters += 1
            else:
                e.state = "RUNNING"
                queued = list(e.queued)
        if flying_elsewhere:
            e.done.wait()            # set by _finish, whatever the flight's outcome
            with e.lock:
                e.waiters -= 1
                if e.state == "ENDED":
                    return e.result
                raise HarnessError("FAILED_PRECONDITION",
                                   f"episode {episode_id} failed: {e.error}")
        try:
            result = self._fly(e, queued)
        except HarnessError as err:
            self._finish(e, "FAILED", error=err.message)
            raise
        except BaseException as err:
            # Whatever happened, the episode is closed and waiters released.
            msg = f"{type(err).__name__}: {err}"
            self._finish(e, "FAILED", error=msg)
            if isinstance(err, Exception):
                raise HarnessError("INTERNAL", msg) from err
            raise
        self._finish(e, "ENDED", result=result)
        return result

    def _finish(self, e: _Episode, state: str, result: dict | None = None,
                error: str = "") -> None:
        with e.lock:
            e.state, e.result, e.error = state, result, error
            e.finished_at = time.monotonic()
            e.done.set()
        with self._lock:
            self._evict_locked()

    # ---- flying, scoring, the bundle ----------------------------------- #

    def _fly(self, e: _Episode, queued: list[_Queued]) -> dict:
        S = self._S
        from . import kpi as K
        from .manifest import is_kpi_grade
        ep = e.ep
        spec = ep.cell.spec.model_copy(
            update={"events": list(ep.cell.spec.events) + [q.event for q in queued]})
        arm_record = e.arm_record
        with self._run_lock:
            try:
                run = S.run_episode(spec, e.defaults, ep.seed, arm=arm_record)
            except (ValueError, S.UnsupportedScenario) as err:
                raise HarnessError("FAILED_PRECONDITION",
                                   f"the headless runner refused the episode in flight: "
                                   f"{type(err).__name__}: {err}") from None
        if run.policy_hash_start != e.policy_hash:
            raise HarnessError("FAILED_PRECONDITION",
                               f"the policy file changed between StartEpisode "
                               f"({e.policy_hash}) and EndEpisode "
                               f"({run.policy_hash_start}); start a new episode")

        res = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
        # The sweep's own verdict function, the one its main() calls (until
        # 2026-10-11 this module carried a copy of it).
        status, bad, labels_failed, known_why = S.episode_verdict(
            spec, res, ep.cell.params, ep.seed)

        manifest = S.episode_manifest(self._manifest_base, run.policy_hash_start, ep.seed,
                                      run.sim_s, run.wall_s)
        grade, why_not = is_kpi_grade(manifest, {})
        stamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
        ep_dir = self.runs_dir / e.episode_id / ep.dir_name(stamp)
        shield = "on" if spec.shield else "off"
        common = {"id": ep.run_id, "id_cell": ep.episode_id,
                  "scenario_id": ep.cell.scenario_id, "template": spec.template,
                  "params": ep.cell.params, "seed": ep.seed, "arm": shield,
                  "prefix": ep.prefix, "expect": spec.expect,
                  "why": spec.description.strip()}
        if labels_failed:
            common["labels_failed"] = labels_failed
        if known_why is not None:
            common["known_failure_reason"] = known_why
        harness = {"episode_id": e.episode_id, "transport": "grpc",
                   "client": e.client, "profile": e.profile or None,
                   "events_injected": [{"index": q.index, "source": q.source,
                                        "event": q.event.model_dump(mode="json")}
                                       for q in queued]}
        kpi_doc = _json_safe({
            **common, "status": status, "gates_failed": bad, "kpi_grade": grade,
            "kpi_grade_reasons": why_not, "arm_record": arm_record,
            "rule_priorities": run.priorities,
            "metrics": {"reached_goal": run.extra.get("reached_goal"),
                        "breaches": run.extra.get("breaches"), "shield": shield,
                        "fsm": run.extra.get("fsm")},
            "kpi": res, "harness": harness})
        S.write_bundle(ep_dir, manifest, kpi_doc, run.events,
                       run.rows if e.keep_log else None, S.manifest_extras(arm_record))
        # What produced this bundle, so the same episode can be asked for again.
        with open(ep_dir / "harness_request.json", "w", encoding="utf-8", newline="\n") as fh:
            fh.write(json.dumps(_json_safe({
                "rpc": "StartEpisode", "request": e.request, "resolved": self._ref(e),
                "events": [q.received for q in queued], "client": e.client,
                "manifest": manifest}), indent=2) + "\n")

        table = {**res, "id": ep.run_id, "family": spec.template, "cell": ep.episode_id,
                 "arm": shield, "prefix": ep.prefix, "params": ep.cell.params,
                 "bundle": ep_dir.as_posix(), "kpi_grade": False}
        table["worst_tick"] = _json_safe(K.worst_tick(run.rows, run.priorities))
        table["null_hover_mission"] = K.null_hover_mission(
            run.rows, {"reached_goal": run.extra.get("reached_goal")})
        if not spec.shield:
            # The sweep's transform for a control arm: its logged repairs
            # were never flown (tools/kpi_report.py shield_off_table).
            table = _kpi_report().shield_off_table(table)
        rollup = K.rollup([table])
        applied = []
        for ev in run.events:
            if ev.get("type") != "hot_apply":
                continue
            applied.append({"t": float(ev.get("t") or 0.0), "event": str(ev.get("event", "")),
                            "rule_id": str(ev.get("rule_id") or ev.get("zone") or ""),
                            "refused": bool(ev.get("refused", False)),
                            "reason": str(ev.get("reason") or ""),
                            "generation": int(ev.get("generation") or 0),
                            "policy_hash": str(ev.get("policy_hash") or "")})
        return {"episode_id": e.episode_id, "bundle_path": ep_dir.as_posix(),
                "status": status, "gates_failed": list(bad),
                "mission_outcome": str(res.get("mission_outcome") or ""),
                "kpi": kpi_row(rollup, spec.template, ep.episode_id, shield),
                "kpi_json": json.dumps(_json_safe(res), sort_keys=True, allow_nan=False),
                "rollup_json": json.dumps(_json_safe(rollup), sort_keys=True,
                                          allow_nan=False),
                "manifest_json": json.dumps(manifest, sort_keys=True),
                "kpi_grade": bool(grade), "kpi_grade_reasons": list(why_not),
                "applied_events": applied}


# --------------------------------------------------------------------------- #
# gRPC: the servicer, credentials, the server
# --------------------------------------------------------------------------- #

def _struct_to_dict(msg) -> dict:
    M = _grpc_modules()
    return M.json_format.MessageToDict(msg) if msg is not None else {}


class StressHarnessServicer:
    """The generated StressHarnessServicer's five methods, onto a Harness.

    Not a subclass of the generated class (so this module imports without
    grpc); `add_StressHarnessServicer_to_server` only needs the methods.
    `allowed_clients` None means no identity is required (the insecure,
    loopback-only mode); otherwise every call must arrive over TLS with a
    client certificate whose common name is in the set."""

    def __init__(self, harness: Harness, allowed_clients: set[str] | None):
        self.harness = harness
        self.allowed = None if allowed_clients is None else set(allowed_clients)
        self._M = _grpc_modules()

    def _client(self, context) -> str:
        if self.allowed is None:
            return ""
        ac = context.auth_context() or {}
        cns = [c.decode("utf-8", "replace") for c in ac.get("x509_common_name", [])]
        if not cns or cns[0] not in self.allowed:
            context.abort(self._M.grpc.StatusCode.PERMISSION_DENIED,
                          f"client certificate {cns[0] if cns else '(none)'!r} is not on "
                          f"the harness allow-list {sorted(self.allowed)}")
        return cns[0]

    def _call(self, context, fn):
        client = self._client(context)
        try:
            return fn(client)
        except HarnessError as e:
            code, msg = e.code, e.message
        except HarnessUnavailable as e:
            # A package or the source tree missing on the SERVER: not transient,
            # so not UNAVAILABLE (which a client may retry, and which a refused
            # TLS handshake already means).
            code, msg = "FAILED_PRECONDITION", str(e)
        except Exception as e:                                   # noqa: BLE001
            code, msg = "INTERNAL", f"{type(e).__name__}: {e}"
        context.abort(getattr(self._M.grpc.StatusCode, code), msg)

    def StartEpisode(self, request, context):
        pb = self._M.pb

        def run(client):
            which = request.WhichOneof("scenario")
            ref = self.harness.start_episode(
                scenario_id=request.scenario_id if which == "scenario_id" else "",
                scenario_yaml=request.scenario_yaml if which == "scenario_yaml" else "",
                cell_id=request.cell_id,
                seed=request.seed if request.HasField("seed") else None,
                profile=request.profile, arm=request.arm,
                keep_flight_log=request.keep_flight_log, client=client)
            return pb.EpisodeRef(**ref)
        return self._call(context, run)

    def HotApply(self, request, context):
        pb = self._M.pb

        def run(_client):
            ack = self.harness.hot_apply(request.episode_id, request.at_sim_t,
                                         _struct_to_dict(request.body))
            return pb.PolicyAck(**ack)
        return self._call(context, run)

    def EmitEvent(self, request, context):
        pb = self._M.pb

        def run(_client):
            ack = self.harness.emit_event(request.episode_id, request.at_sim_t, request.type,
                                          _struct_to_dict(request.payload))
            return pb.EventAck(**ack)
        return self._call(context, run)

    def GetStatus(self, request, context):
        pb = self._M.pb

        def run(_client):
            st = self.harness.status(request.episode_id)
            events = [pb.QueuedEvent(**q) for q in st.pop("events")]
            state = pb.EpisodeState.Value(st.pop("state"))
            return pb.EpisodeStatus(state=state, events=events, **st)
        return self._call(context, run)

    def EndEpisode(self, request, context):
        pb = self._M.pb

        def run(_client):
            b = dict(self.harness.end_episode(request.episode_id))
            row = dict(b.pop("kpi"))
            kpi = pb.KpiRow(**row)
            applied = [pb.AppliedEvent(**a) for a in b.pop("applied_events")]
            return pb.EpisodeBundle(kpi=kpi, applied_events=applied, **b)
        return self._call(context, run)


def _read(p: str | Path) -> bytes:
    return Path(p).read_bytes()


def server_credentials(ca_cert: str | Path, cert: str | Path, key: str | Path):
    """Mutual TLS, server side: present `cert`, REQUIRE a client certificate
    and verify it against `ca_cert`."""
    M = _grpc_modules()
    return M.grpc.ssl_server_credentials([(_read(key), _read(cert))],
                                         root_certificates=_read(ca_cert),
                                         require_client_auth=True)


def channel_credentials(ca_cert: str | Path, cert: str | Path | None = None,
                        key: str | Path | None = None):
    """Client side: trust `ca_cert` for the server, and present `cert` / `key`
    (without them the server's handshake refuses the client)."""
    M = _grpc_modules()
    if (cert is None) != (key is None):
        raise ValueError("a client certificate needs both cert and key")
    if cert is None:
        return M.grpc.ssl_channel_credentials(root_certificates=_read(ca_cert))
    return M.grpc.ssl_channel_credentials(root_certificates=_read(ca_cert),
                                          private_key=_read(key),
                                          certificate_chain=_read(cert))


def pki_paths(pki_dir: str | Path, name: str) -> tuple[Path, Path, Path]:
    """(ca.cert.pem, <name>.cert.pem, <name>.key.pem) in a folder generate_pki
    wrote."""
    d = Path(pki_dir)
    return d / "ca.cert.pem", d / f"{name}.cert.pem", d / f"{name}.key.pem"


def is_loopback(host: str) -> bool:
    h = host.strip("[]")
    if h == "localhost":
        return True
    try:
        return ipaddress.ip_address(h).is_loopback
    except ValueError:
        return False


@dataclass
class HarnessServer:
    server: Any
    port: int
    host: str
    secure: bool
    servicer: StressHarnessServicer

    @property
    def address(self) -> str:
        h = f"[{self.host}]" if ":" in self.host and not self.host.startswith("[") else self.host
        return f"{h}:{self.port}"

    def stop(self, grace: float | None = None) -> None:
        self.server.stop(grace).wait()


def serve(harness: Harness | None = None, *, host: str = "127.0.0.1", port: int = 0,
          pki_dir: str | Path | None = None, server_name: str = SERVER_CN,
          allowed_clients=(DEFAULT_CLIENT_CN,), max_workers: int = 8) -> HarnessServer:
    """Start the service and return it (port 0 picks a free port).

    With `pki_dir`, mutual TLS: the server presents <server_name>.cert.pem,
    requires a client certificate signed by ca.cert.pem, and requires its
    common name to be in `allowed_clients`. Without it, plaintext - allowed on
    a loopback address only (the grant locks mTLS for the control plane; the
    insecure mode is for a test or a single desktop)."""
    from concurrent import futures
    M = _grpc_modules()
    if pki_dir is None and not is_loopback(host):
        raise ValueError(f"refusing an insecure gRPC port on {host!r}: plaintext is for "
                         f"loopback only; give pki_dir for mutual TLS")
    harness = harness or Harness()
    allowed = None if pki_dir is None else set(allowed_clients or ())
    if allowed is not None and not allowed:
        raise ValueError("mutual TLS with an empty client allow-list would refuse everyone")
    servicer = StressHarnessServicer(harness, allowed)
    server = M.grpc.server(futures.ThreadPoolExecutor(max_workers=max_workers))
    M.pb_grpc.add_StressHarnessServicer_to_server(servicer, server)
    addr = f"[{host}]:{port}" if ":" in host and not host.startswith("[") else f"{host}:{port}"
    if pki_dir is None:
        bound = server.add_insecure_port(addr)
    else:
        ca, cert, key = pki_paths(pki_dir, server_name)
        bound = server.add_secure_port(addr, server_credentials(ca, cert, key))
    if not bound:
        raise RuntimeError(f"could not bind {addr}")
    server.start()
    return HarnessServer(server=server, port=bound, host=host.strip("[]"),
                         secure=pki_dir is not None, servicer=servicer)


# --------------------------------------------------------------------------- #
# the lab PKI: one root CA for the harness and (by procedure) SROS 2
# --------------------------------------------------------------------------- #

# Folders a sync client uploads: the environment variables the OneDrive client
# sets, and the folder names OneDrive, Dropbox, Google Drive and iCloud Drive
# give themselves.
_SYNCED_ENV = ("OneDrive", "OneDriveConsumer", "OneDriveCommercial")
_SYNCED_PART = re.compile(r"^(onedrive( - .+)?|dropbox( \(.+\))?|google ?drive|"
                          r"my drive|icloud ?drive|com~apple~clouddocs)$", re.IGNORECASE)


def synced_folder(path: str | Path) -> str | None:
    """The cloud-synced folder `path` lies in, or None. A private key written
    there is uploaded, gitignored or not."""
    p = Path(path).expanduser()
    try:
        p = p.resolve()
    except OSError:
        p = p.absolute()
    for var in _SYNCED_ENV:
        root = os.environ.get(var)
        if not root:
            continue
        try:
            r = Path(root).resolve()
        except OSError:
            continue
        if p == r or p.is_relative_to(r):
            return f"{r} (${var})"
    for part in p.parts:
        if _SYNCED_PART.match(part.strip("\\/")):
            return part
    return None


def _refuse_synced(out: Path, allow_synced: bool) -> None:
    where = synced_folder(out)
    if where and not allow_synced:
        raise ValueError(f"{out} is inside the cloud-synced folder {where}: the private "
                         f"keys, written unencrypted, would be uploaded. Use a local "
                         f"folder (the default is {DEFAULT_PKI_DIR}), or pass "
                         f"allow_synced=True (--allow-synced) on purpose")


def _crypto() -> SimpleNamespace:
    try:
        from cryptography import x509
        from cryptography.hazmat.primitives import hashes, serialization
        from cryptography.hazmat.primitives.asymmetric import ec
        from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID
    except ImportError as e:
        raise HarnessUnavailable("generating certificates needs `cryptography` (the "
                                 "[signing] extra)") from e
    return SimpleNamespace(x509=x509, hashes=hashes, serialization=serialization, ec=ec,
                           EKU=ExtendedKeyUsageOID, NameOID=NameOID)


def _write_pem(path: Path, data: bytes, private: bool = False) -> None:
    path.write_bytes(data)
    if private:
        # Owner-only on POSIX. On Windows chmod can only toggle read-only, so
        # a key's protection there is its folder's ACL: the default folder is
        # in the per-user profile (%LOCALAPPDATA%).
        try:
            os.chmod(path, 0o600)
        except OSError:
            pass


def _key_pem(C, key) -> bytes:
    return key.private_bytes(C.serialization.Encoding.PEM,
                             C.serialization.PrivateFormat.PKCS8,
                             C.serialization.NoEncryption())


def _name(C, cn: str):
    return C.x509.Name([C.x509.NameAttribute(C.NameOID.ORGANIZATION_NAME, "VLA Guardrail Lab"),
                        C.x509.NameAttribute(C.NameOID.COMMON_NAME, cn)])


def _key_usage(C, *, ca: bool):
    return C.x509.KeyUsage(digital_signature=True, content_commitment=False,
                           key_encipherment=False, data_encipherment=False,
                           key_agreement=False, key_cert_sign=ca, crl_sign=ca,
                           encipher_only=False, decipher_only=False)


def _issue(C, ca_key, ca_cert, cn: str, *, server_hosts=None, client: bool = True,
           days: int = 365):
    """A leaf certificate signed by the CA: serverAuth with a SAN when
    `server_hosts` is given, clientAuth when `client`."""
    if not _NAME.match(cn):
        raise ValueError(f"certificate name {cn!r}: letters, digits, . _ - only")
    if not server_hosts and not client:
        raise ValueError("a certificate must be for a server, a client or both")
    now = _dt.datetime.now(_dt.timezone.utc)
    key = C.ec.generate_private_key(C.ec.SECP256R1())
    b = (C.x509.CertificateBuilder()
         .subject_name(_name(C, cn)).issuer_name(ca_cert.subject)
         .public_key(key.public_key()).serial_number(C.x509.random_serial_number())
         .not_valid_before(now - _dt.timedelta(minutes=5))
         .not_valid_after(now + _dt.timedelta(days=days))
         .add_extension(C.x509.BasicConstraints(ca=False, path_length=None), critical=True)
         .add_extension(_key_usage(C, ca=False), critical=True)
         .add_extension(C.x509.SubjectKeyIdentifier.from_public_key(key.public_key()),
                        critical=False)
         .add_extension(C.x509.AuthorityKeyIdentifier.from_issuer_public_key(
             ca_key.public_key()), critical=False))
    usages = ([C.EKU.SERVER_AUTH] if server_hosts else []) + ([C.EKU.CLIENT_AUTH] if client else [])
    b = b.add_extension(C.x509.ExtendedKeyUsage(usages), critical=False)
    if server_hosts:
        san = []
        for h in server_hosts:
            try:
                san.append(C.x509.IPAddress(ipaddress.ip_address(h.strip("[]"))))
            except ValueError:
                san.append(C.x509.DNSName(h))
        b = b.add_extension(C.x509.SubjectAlternativeName(san), critical=False)
    cert = b.sign(ca_key, C.hashes.SHA256())
    return key, cert


def generate_pki(out_dir: str | Path = DEFAULT_PKI_DIR, *,
                 server_hosts=("localhost", "127.0.0.1"),
                 client_names=(DEFAULT_CLIENT_CN,), server_name: str = SERVER_CN,
                 ca_cn: str = CA_CN, days: int = 365, ca_days: int = 3650,
                 overwrite: bool = False, allow_synced: bool = False) -> dict[str, Path]:
    """Write a lab root CA, a server certificate and client certificates.

    Files (PEM): ca.cert.pem / ca.key.pem (sros2's own names for a keystore's
    CA), <server_name>.cert.pem / .key.pem, and <client>.cert.pem / .key.pem
    per client. ECDSA P-256 throughout, the curve sros2 uses, so this CA can
    sign SROS 2 enclave identities as well (docs/DESIGN-harness-grpc.md).
    Keys are written UNENCRYPTED: a development PKI. So a folder inside a
    cloud-synced one is refused unless `allow_synced` (nothing is written),
    and a folder that already holds a CA key is refused unless `overwrite`: a
    new root invalidates every certificate the old one signed, SROS 2
    keystores included."""
    C = _crypto()
    out = Path(out_dir)
    _refuse_synced(out, allow_synced)
    if (out / "ca.key.pem").exists() and not overwrite:
        raise FileExistsError(f"{out / 'ca.key.pem'} exists; replacing the root CA "
                              f"invalidates every certificate it signed (SROS 2 "
                              f"enclaves too). Pass overwrite=True (--force) to do it "
                              f"on purpose, or issue_cert() to add a certificate")
    out.mkdir(parents=True, exist_ok=True)
    now = _dt.datetime.now(_dt.timezone.utc)
    ca_key = C.ec.generate_private_key(C.ec.SECP256R1())
    ca_cert = (C.x509.CertificateBuilder()
               .subject_name(_name(C, ca_cn)).issuer_name(_name(C, ca_cn))
               .public_key(ca_key.public_key()).serial_number(C.x509.random_serial_number())
               .not_valid_before(now - _dt.timedelta(minutes=5))
               .not_valid_after(now + _dt.timedelta(days=ca_days))
               .add_extension(C.x509.BasicConstraints(ca=True, path_length=1), critical=True)
               .add_extension(_key_usage(C, ca=True), critical=True)
               .add_extension(C.x509.SubjectKeyIdentifier.from_public_key(ca_key.public_key()),
                              critical=False)
               .sign(ca_key, C.hashes.SHA256()))
    pem = C.serialization.Encoding.PEM
    files = {"ca_cert": out / "ca.cert.pem", "ca_key": out / "ca.key.pem"}
    _write_pem(files["ca_cert"], ca_cert.public_bytes(pem))
    _write_pem(files["ca_key"], _key_pem(C, ca_key), private=True)
    for name, hosts, client in ([(server_name, tuple(server_hosts), False)]
                                + [(c, None, True) for c in client_names]):
        key, cert = _issue(C, ca_key, ca_cert, name, server_hosts=hosts, client=client,
                           days=days)
        files[f"{name}_cert"] = out / f"{name}.cert.pem"
        files[f"{name}_key"] = out / f"{name}.key.pem"
        _write_pem(files[f"{name}_cert"], cert.public_bytes(pem))
        _write_pem(files[f"{name}_key"], _key_pem(C, key), private=True)
    return files


def issue_cert(pki_dir: str | Path, name: str, *, server_hosts=None, client: bool = True,
               days: int = 365, overwrite: bool = False,
               allow_synced: bool = False) -> tuple[Path, Path]:
    """One more certificate from the CA in `pki_dir` (a client for another
    host, or a server certificate for the Orin's address). Returns (cert, key).
    A cloud-synced `pki_dir` is refused as generate_pki refuses it."""
    C = _crypto()
    d = Path(pki_dir)
    _refuse_synced(d, allow_synced)
    ca_cert = C.x509.load_pem_x509_certificate((d / "ca.cert.pem").read_bytes())
    ca_key = C.serialization.load_pem_private_key((d / "ca.key.pem").read_bytes(),
                                                  password=None)
    cert_p, key_p = d / f"{name}.cert.pem", d / f"{name}.key.pem"
    if cert_p.exists() and not overwrite:
        raise FileExistsError(f"{cert_p} exists")
    key, cert = _issue(C, ca_key, ca_cert, name, server_hosts=server_hosts,
                       client=client, days=days)
    _write_pem(cert_p, cert.public_bytes(C.serialization.Encoding.PEM))
    _write_pem(key_p, _key_pem(C, key), private=True)
    return cert_p, key_p


def fingerprint(cert_path: str | Path) -> str:
    """SHA-256 fingerprint, colon-separated as `openssl x509 -fingerprint
    -sha256` prints it: how a SROS 2 keystore's ca.cert.pem is checked to be
    this CA."""
    C = _crypto()
    cert = C.x509.load_pem_x509_certificate(Path(cert_path).read_bytes())
    return ":".join(f"{b:02X}" for b in cert.fingerprint(C.hashes.SHA256()))


# --------------------------------------------------------------------------- #
# command line
# --------------------------------------------------------------------------- #

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m guardrail.harness_service",
                                 description="The stress harness's gRPC control plane.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("gen-pki", help="write a lab root CA, a server and a client certificate")
    g.add_argument("--out", default=str(DEFAULT_PKI_DIR))
    g.add_argument("--host", action="append", default=None,
                   help="a name or IP for the server certificate (repeat); default "
                        "localhost and 127.0.0.1")
    g.add_argument("--client", action="append", default=None,
                   help=f"a client certificate name (repeat); default {DEFAULT_CLIENT_CN}")
    g.add_argument("--force", action="store_true", help="replace an existing CA")
    g.add_argument("--allow-synced", action="store_true",
                   help="write the keys into a cloud-synced folder (OneDrive, Dropbox, "
                        "...) anyway")
    i = sub.add_parser("issue", help="issue one more certificate from an existing CA")
    i.add_argument("--pki", default=str(DEFAULT_PKI_DIR))
    i.add_argument("--name", required=True)
    i.add_argument("--server-host", action="append", default=None,
                   help="make it a server certificate for this name or IP (repeat)")
    i.add_argument("--allow-synced", action="store_true",
                   help="write the key into a cloud-synced folder anyway")
    s = sub.add_parser("serve", help="run the service until interrupted")
    s.add_argument("--host", default="127.0.0.1")
    s.add_argument("--port", type=int, default=DEFAULT_PORT)
    s.add_argument("--pki", default=str(DEFAULT_PKI_DIR))
    s.add_argument("--insecure", action="store_true",
                   help="plaintext, loopback only (no certificates)")
    s.add_argument("--allow-client", action="append", default=None,
                   help=f"client certificate name allowed in (repeat); default "
                        f"{DEFAULT_CLIENT_CN}")
    s.add_argument("--runs", default=str(DEFAULT_RUNS_DIR), help="where bundles go")
    s.add_argument("--library", default=None, help="scenario library (default: the sweep's)")
    s.add_argument("--armed-ttl", type=float, default=ARMED_IDLE_TTL_S,
                   help="seconds an ARMED episode may sit idle before it expires "
                        f"(default {ARMED_IDLE_TTL_S:g})")
    args = ap.parse_args(argv)

    try:
        if args.cmd == "gen-pki":
            files = generate_pki(args.out, server_hosts=tuple(args.host or ("localhost",
                                                                            "127.0.0.1")),
                                 client_names=tuple(args.client or (DEFAULT_CLIENT_CN,)),
                                 overwrite=args.force, allow_synced=args.allow_synced)
            for k, p in files.items():
                print(f"  {k:<24} {p}")
            print(f"CA SHA-256 fingerprint: {fingerprint(files['ca_cert'])}")
            print("Private keys are unencrypted (development PKI): keep the folder out "
                  "of the repository and of any synced folder; the CA key seeds SROS 2 "
                  "keystores on this desktop and is never copied to the Orin or the "
                  "drone.")
            return 0
        if args.cmd == "issue":
            cert, key = issue_cert(args.pki, args.name, server_hosts=args.server_host,
                                   client=not args.server_host,
                                   allow_synced=args.allow_synced)
            print(f"  {cert}\n  {key}")
            return 0
        srv = serve(Harness(runs_dir=args.runs, library=args.library,
                            armed_idle_ttl_s=args.armed_ttl), host=args.host,
                    port=args.port, pki_dir=None if args.insecure else args.pki,
                    allowed_clients=tuple(args.allow_client or (DEFAULT_CLIENT_CN,)))
    except (HarnessUnavailable, FileExistsError, FileNotFoundError, ValueError,
            RuntimeError) as e:
        print(f"{type(e).__name__}: {e}", file=sys.stderr)
        return 2
    mode = "mutual TLS" if srv.secure else "PLAINTEXT (loopback only)"
    # Flushed: under a service manager stdout is a file, and an unflushed
    # "listening" line reads as a server that never came up.
    print(f"StressHarness listening on {srv.address}, {mode}; bundles -> {args.runs}",
          flush=True)
    try:
        srv.server.wait_for_termination()
    except KeyboardInterrupt:
        srv.stop(1.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
