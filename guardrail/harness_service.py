"""The stress harness's gRPC control plane over mutual TLS: the Stress Testing
page's ten RPCs (StartSession, StopSession, InjectPolicy, HotApply,
StartEpisode, IssueTask, EmitEvent, EndEpisode, StreamTelemetry,
StreamEvents) plus GetStatus, driving the headless harness.

    python -m guardrail.harness_service gen-pki                 # lab CA + certs
    python -m guardrail.harness_service serve                   # mTLS on :50051
    python -m guardrail.harness_service serve --insecure        # loopback only
    python tools/harness_client.py run --scenario nfz-head-on --seed 1001

WHAT THE GRANT ASKS FOR

Stress Testing, "Locked design choices": "Control plane: gRPC over mTLS. Same
root CA as SROS 2 for cert reuse." Its Outputs table lists "RPC harness -
Python service + gRPC stubs", and its "Service surface (proto-ish)" names the
RPCs and its "Session lifecycle" their order: StartSession(scene, airframe,
seed), InjectPolicy(bundle, expected_hash) -> PolicyAck(policy_hash,
generation=0), IssueTask(prompt), StartEpisode, telemetry and Shield events
streamed while it flies, HotApply mid-flight -> PolicyAck(generation=1),
EndEpisode -> EpisodeBundle, StopSession. The reference data-flow page adds
why gRPC and not ROS 2 services: the orchestrator "must also speak to non-ROS
clients (CI, dashboards, scenario authoring tools)". Audit cards WP4-05 (no
gRPC service) and X-15 (no mTLS, no shared root CA) recorded that none of it
existed.

WHAT THIS IS

A thin transport over code that already runs. `Harness` (no gRPC in it) arms
an episode from a ScenarioSpec cell (guardrail/scenario_spec.py: Family,
episode_arms, Episode), draws its arm (experiments/sweep_scenarios.py
build_arm), takes mid-flight events, and flies it through the sweep's other
public functions - run_episode or, live, EpisodeRunner (step, inject, finish),
then score, episode_verdict, episode_manifest, write_bundle, manifest_extras
(and episode_extras where the sweep has it) - so an episode started over gRPC
is flown, scored, judged and bundled exactly as the sweep does it.
`StressHarnessServicer` maps that onto the generated stubs
(guardrail/protos/harness.proto); `serve()` puts it on a port, with mTLS or,
on loopback only, without.

SESSIONS. A session (StartSession) groups episodes: its injected policy
(InjectPolicy: a bundle or policy YAML under policies/ or bundles/, or the
archive's bytes, pinned to `expected_hash`) replaces each later episode's
scenario policy; its task (IssueTask) is recorded with each; its default seed
is used when an episode names none; and both streams carry its episodes.
`backend` is HEADLESS; SITL is a reserved value that answers UNIMPLEMENTED
until the hil rail drives it. A session may be BOUND to a REST hot-apply
endpoint (guardrail/api.py, e.g. a rail's live Shield): a HotApply addressed
to the session is POSTed to <endpoint>/hot_apply as it is and the endpoint's
answer returned, 409 / 404 / 422 mapped as below; the session's InjectPolicy
checks the endpoint flies that policy (GET /health). Only a loopback endpoint
is accepted unless the server allows its host (`rest_hosts`,
--allow-rest-host): the server makes the request, so an endpoint is a place a
client can make the server send to. No proxy is used for it.

TWO WAYS TO FLY AN EPISODE.

  Scheduled (the default). EndEpisode flies a whole episode in one
  run_episode call (about 0.3-0.4 s for 30 simulated seconds). HotApply and
  EmitEvent carry `at_sim_t`, are validated when they arrive, and are applied
  by the runner's own `apply_event` at that simulated time. The flight is a
  pure function of (cell, seed, arm, scheduled events).

  Live (`StartEpisode(live=true, pace)`). The episode starts flying at once
  on a thread of its own, one EpisodeRunner.step() per tick, paced at `pace`
  simulated seconds per wall second (1.0 real time, 0 unpaced). A HotApply or
  EmitEvent is injected between ticks (EpisodeRunner.inject): at `at_sim_t`,
  or with `now` at the next tick to fly; a time already flown is
  OUT_OF_RANGE. Each injection is recorded with the time it flew at, and the
  runner applies an injected event exactly as the same event written last in
  the scenario's `events:` (its own docstring and tests), so the bundle of a
  live episode re-flies as a scheduled one (tools/rerun_episode.py).
  EndEpisode lets it finish unpaced and returns its bundle; StopSession stops
  it (FAILED, nothing bundled). The runner exposes no per-tick state while
  it flies, so the live telemetry is one clock frame per tick and the flown
  rows follow when the episode ends.

HotApply to an episode takes the Policy DSL's REST hot-apply body
(guardrail/api.py: dynamic_nfz, time_window_switch, corridor_swap) and runs it
through the SAME handler the REST endpoint runs (api.apply_event) on a dry-run
Shield: the injected policy with this episode's hot-apply bodies applied IN
FLIGHT ORDER (by at_sim_t, then by arrival - the order run_episode applies
them), not in the order they arrived. A body the REST endpoint would refuse at
its place in that order is refused here with the matching status (409
ALREADY_EXISTS, 404 NOT_FOUND, 422 INVALID_ARGUMENT); a body that would make
an earlier-acked hot-apply fail further down the timeline is
FAILED_PRECONDITION. (Until 9 Oct the dry run replayed arrival order: a switch
at 5 s sent after its zone at 10 s was acked, then the flight failed at
EndEpisode.) An accepted body becomes the ScenarioEvent the runner applies
(spawn_polygon_fence with the zone's vertices, activate_rule /
deactivate_rule, swap_corridor). A field that ScenarioEvent cannot carry (a
zone's `valid_time`, a switch's priority) is refused UNIMPLEMENTED rather than
dropped: a zone that flew without its schedule would be a different zone.

THE DRY RUN KNOWS ONLY THE HOT-APPLIES. The scenario's own `events:` and the
episode's EmitEvents are applied by the runner against the flying vehicle
(a zone spawned `ahead_of_vehicle_m` needs the aircraft's position), so the
dry run cannot replay them. A HotApply aimed at a rule only they create is
NOT_FOUND (aim an EmitEvent at it instead), and a HotApply that collides with
one of them (the same zone id, say) is acked and then fails in flight with
FAILED_PRECONDITION. PolicyAck's generation counts hot-applies only, as of
that ack; the flown generations are in EpisodeBundle.applied_events.

LIFECYCLE. EndEpisode is idempotent: a second call returns the same bundle,
and a call that arrives while another call is flying the episode waits for
that flight and returns its bundle (or its failure). An episode left ARMED
with no StartEpisode / HotApply / EmitEvent on it for `armed_idle_ttl_s`
(default one hour) expires: it becomes FAILED with the reason, nothing flies,
and it stops counting against MAX_OPEN_EPISODES, so clients that give up on
an episode cannot wedge a long-running server. A session expires the same
way: OPEN, with no call on it (StartEpisode, InjectPolicy, IssueTask, a
session HotApply, a stream poll) for `session_idle_ttl_s` (default one hour)
and no episode of it ARMED or RUNNING, it is stopped as StopSession would
stop it, its event stream says `session_expired` first, and it stops
counting against MAX_OPEN_SESSIONS.

WHAT IT IS NOT

  * Not KPI-grade. The headless backend's manifest names the topology
    `headless-kinematic` and a measured sim_speedup far above 1 (a live
    episode at pace 1.0 measures about 1, and is still headless), so
    guardrail.manifest.is_kpi_grade refuses every episode, and EpisodeBundle
    says so (`kpi_grade`, `kpi_grade_reasons`). The grant's KPI numbers come
    from the hil topology (a Jetson Orin, not built yet).
  * Not a simulator session. StartSession opens no simulator: the headless
    backend has no scene or airframe to load (both are recorded), and the
    SITL backend that would is reserved. IssueTask records the prompt's hash;
    the headless pilot reads no text.
  * Not a scheduler. Scheduled episodes fly one at a time per server (one
    lock around run_episode; a live episode takes it one tick at a time); the
    reference's answer to throughput (risk R9) is parallel hosts.

STATUS CODES (what a client can rely on)

    INVALID_ARGUMENT     the spec, cell, seed, arm, event, session request or
                         policy fails validation (pydantic's message is
                         passed through); a REST 422
    NOT_FOUND            unknown scenario_id, cell, profile, episode, session,
                         policy file; a REST 404
    ALREADY_EXISTS       a REST 409 (a rule id already in use)
    OUT_OF_RANGE         an event after the episode's last tick, or before the
                         next tick a live episode flies
    PERMISSION_DENIED    a client certificate not on the allow-list; a policy
                         (a client-sent scenario's, an InjectPolicy path)
                         outside the server's policies/ or bundles/ folder; a
                         REST endpoint off loopback the server does not allow
    FAILED_PRECONDITION  an event for an episode no longer taking them (ended,
                         failed, expired, or a scheduled one being flown); an
                         episode the runner refused in flight (see
                         GetStatus.error); a paraphrase arm this server cannot
                         draw (no LLM client here, or no stored set holding the
                         prompt); a hot-apply that would make an earlier-acked
                         one fail later in the flight; a session that is
                         stopped, or not bound to a REST endpoint for a
                         session HotApply; an injected bundle that does not
                         hash to `expected_hash`, or an endpoint flying
                         another policy; a package or source file missing on
                         the server
    UNIMPLEMENTED        a hot-apply field the headless runner cannot carry;
                         the reserved SITL backend
    RESOURCE_EXHAUSTED   too many open episodes or sessions, events in one
                         episode, or an InjectPolicy archive too large
    UNAVAILABLE          (from gRPC) a TLS handshake that failed: no client
                         certificate, one from another CA, or a plaintext
                         client on the mTLS port; (from the service) a bound
                         REST endpoint that did not answer, or answered 5xx

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
import hashlib
import importlib.util
import ipaddress
import json
import math
import os
import re
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import yaml
from pydantic import ValidationError

from .scenario_spec import (Episode, Family, Library, PolicyRef, ScenarioEvent,
                            episode_arms, load_profile)

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
SESSION_IDLE_TTL_S = 3600.0      # an OPEN session with no call and nothing armed expires
MAX_OPEN_SESSIONS = 64
MAX_KEPT_SESSIONS = 256          # stopped sessions kept for their streams
MAX_TELEMETRY_FRAMES = 200_000   # per session; frames past it are counted, not kept
MAX_BUNDLE_BYTES = 16 << 20      # an InjectPolicy archive sent as bytes
REST_TIMEOUT_S = 10.0            # a bound REST endpoint's answer
DEFAULT_PACE = 1.0               # a live episode: simulated seconds per wall second
STREAM_POLL_S = 0.5              # a stream checks for a cancelled client this often
BACKENDS = ("headless", "sitl")  # "sitl" is reserved (UNIMPLEMENTED)
TASK_NOTE = ("the headless pilot reads no text: the prompt's SHA-256 is recorded in "
             "the bundle's manifest_extras.json (task_prompt_sha256) and no flown "
             "action depends on it")
LIVE_NO_STATE = ("the steppable runner exposes no per-tick state while it flies; the "
                 "flown row follows when the episode ends")
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
    session_id: str = ""
    policy_ref: Any = None           # the session's injected policy (PolicyRef), else None
    task: dict | None = None         # IssueTask: {"prompt_sha256", "chars", "text", "source"}
    live: bool = False
    pace: float = DEFAULT_PACE
    runner: Any = None               # sweep_scenarios.EpisodeRunner of a live episode
    ticks_flown: int = 0
    wake: threading.Event = field(default_factory=threading.Event)   # cut a live pause short


@dataclass
class _Session:
    session_id: str
    backend: str
    scene: str
    airframe: str
    seed: int | None
    rest_endpoint: str
    label: str
    client: str
    policy: dict | None = None       # {"ref": PolicyRef, "policy_hash", "signature", ...}
    task: dict | None = None
    episodes: list = field(default_factory=list)
    state: str = "OPEN"
    events: list = field(default_factory=list)       # HarnessEvent dicts, in order
    telemetry: list = field(default_factory=list)    # TelemetryFrame dicts, in order
    telemetry_dropped: int = 0
    stopped_at: float = 0.0
    idle_since: float = field(default_factory=time.monotonic)   # last call on it
    cond: threading.Condition = field(default_factory=threading.Condition)


def _act4(a) -> dict:
    """An Action4D dict of floats (NaN where a channel is absent or not a
    number: a proto3 double carries NaN, and a raw action is NaN by design in
    the non-finite-action scenarios)."""
    a = a if isinstance(a, dict) else {}
    out = {}
    for k in ("vx", "vy", "vz_up", "yaw_rate"):
        try:
            out[k] = float(a.get(k))
        except (TypeError, ValueError):
            out[k] = float("nan")
    return out


def _row_frame(e: "_Episode", r: dict) -> dict:
    """A TelemetryFrame from one flown row of a headless episode: the pose,
    the raw command and the action that FLEW (the row's `emitted`), the FSM
    state, the counts. What the row does not carry is unset, with why."""
    unset: dict[str, str] = {}
    fr: dict[str, Any] = {"session_id": e.session_id, "episode_id": e.episode_id,
                          "tick": int(r.get("tick") or 0), "sim_t": float(r.get("t") or 0.0),
                          "live": False, "wall_time": time.time(),
                          "planner_output": _act4(r.get("raw")),
                          "executed": _act4(r.get("emitted")),
                          "fsm_state": str(r.get("fsm_state_after") or ""),
                          "violations": len(r.get("violations") or []),
                          "repairs": len(r.get("repairs") or [])}
    for k in ("x", "y", "up"):
        v = r.get(k)
        if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v):
            fr[k] = float(v)
        else:
            unset[k] = "not in the flown row"
    if isinstance(r.get("unsafe"), bool):
        fr["unsafe"] = r["unsafe"]
    else:
        unset["unsafe"] = "not in the flown row"
    fr["unset"] = unset
    return fr


def check_endpoint(url: str, allowed_hosts=()) -> str:
    """A REST hot-apply endpoint a session may be bound to: http(s), a host,
    loopback unless the server allows the host. Returns it without a
    trailing slash. The server itself sends to it, so an arbitrary URL would
    let a client point the server at any address it can reach."""
    try:
        u = urllib.parse.urlsplit(url)
    except ValueError as e:
        raise HarnessError("INVALID_ARGUMENT", f"rest_endpoint {url!r}: {e}") from None
    if u.scheme not in ("http", "https") or not u.hostname:
        raise HarnessError("INVALID_ARGUMENT", f"rest_endpoint {url!r}: an http(s)://host:port "
                                               f"URL of a guardrail/api.py endpoint")
    if u.query or u.fragment or u.username or u.password:
        raise HarnessError("INVALID_ARGUMENT", f"rest_endpoint {url!r}: no query, fragment "
                                               f"or credentials")
    if not is_loopback(u.hostname) and u.hostname not in set(allowed_hosts or ()):
        raise HarnessError("PERMISSION_DENIED",
                           f"rest_endpoint host {u.hostname!r} is not loopback and not "
                           f"allowed by this server (serve --allow-rest-host)")
    return url.rstrip("/")


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
                 armed_idle_ttl_s: float | None = None,
                 rest_hosts=(), session_idle_ttl_s: float | None = None):
        self.runs_dir = Path(runs_dir) if runs_dir else DEFAULT_RUNS_DIR
        self.armed_idle_ttl_s = float(ARMED_IDLE_TTL_S if armed_idle_ttl_s is None
                                      else armed_idle_ttl_s)
        if not self.armed_idle_ttl_s >= 0:
            raise ValueError(f"armed_idle_ttl_s must be >= 0, got {armed_idle_ttl_s!r}")
        self.session_idle_ttl_s = float(SESSION_IDLE_TTL_S if session_idle_ttl_s is None
                                        else session_idle_ttl_s)
        if not self.session_idle_ttl_s >= 0:
            raise ValueError(f"session_idle_ttl_s must be >= 0, got {session_idle_ttl_s!r}")
        self._S = _sweep()
        self._default_lib = Path(library) if library else Path(self._S.LIB)
        self._libs: dict[Path, Library] = {}
        # Resolved ONCE, at construction: the code this process runs is the
        # code it imported at start, so a checkout changed under a running
        # server must not move the revision its manifests record.
        self._manifest_base = dict(manifest_base or self._S.manifest_base())
        self._episodes: dict[str, _Episode] = {}
        self._sessions: dict[str, _Session] = {}
        # Hosts a session's REST endpoint may name besides loopback.
        self.rest_hosts = frozenset(rest_hosts or ())
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
        """With e.lock held: does the episode take events now? An ARMED one
        does, and a live one while it flies."""
        self._expire_if_idle(e)
        if e.live and e.state == "RUNNING" and e.runner is not None and not e.runner.done:
            return
        if e.state != "ARMED":
            why = f" ({e.error})" if e.state == "FAILED" and e.error else ""
            raise HarnessError("FAILED_PRECONDITION",
                               f"episode {e.episode_id} is {e.state}{why}; events are "
                               f"accepted only before EndEpisode (or while a live "
                               f"episode flies)")

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
                      client: str = "", session_id: str = "", live: bool = False,
                      pace: float | None = None) -> dict:
        if bool(scenario_id) == bool(scenario_yaml):
            raise HarnessError("INVALID_ARGUMENT",
                               "give exactly one of scenario_id or scenario_yaml")
        sess = self._open_session(session_id) if session_id else None
        if live:
            pace = DEFAULT_PACE if pace is None else _finite(pace, "pace")
            if pace < 0:
                raise HarnessError("INVALID_ARGUMENT", f"pace {pace} must be >= 0 (0: unpaced)")
        elif pace is not None:
            raise HarnessError("INVALID_ARGUMENT", "pace applies to a live episode only")
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
        if seed is None and sess is not None and sess.seed is not None:
            seed = sess.seed                     # the session's default seed
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
        # A session's injected policy replaces the scenario's (InjectPolicy).
        policy_ref = sess.policy["ref"] if sess is not None and sess.policy else None
        try:
            policy, _source = (policy_ref or spec.policy).load_with_source(ROOT)
        except FileNotFoundError as e:
            raise HarnessError("NOT_FOUND", f"policy: {e}") from None
        except (ValidationError, ValueError, OSError) as e:
            raise HarnessError("INVALID_ARGUMENT",
                               f"policy {(policy_ref or spec.policy).source!r}: {e}") from None
        arm_record = self._arm_record(ep, policy if policy_ref is not None else None)

        eid = "ep-" + uuid.uuid4().hex[:12]
        e = _Episode(episode_id=eid, ep=ep, profile=prof.profile if prof else "",
                     defaults=lib.defaults, keep_log=bool(keep_flight_log), t_end=t_end,
                     policy=policy.model_copy(deep=True), policy_hash=policy.policy_hash,
                     request={"scenario_id": scenario_id or None,
                              "scenario_yaml": scenario_yaml or None,
                              "cell_id": cell_id or None,
                              "seed": None if seed is None else int(seed),
                              "profile": profile or None, "arm": arm or None,
                              "keep_flight_log": bool(keep_flight_log),
                              "session_id": session_id or None, "live": bool(live),
                              "pace": pace},
                     client=client, dry=None, arm_record=arm_record,
                     session_id=session_id, policy_ref=policy_ref,
                     task=dict(sess.task) if sess is not None and sess.task else None,
                     live=bool(live), pace=float(pace) if live else DEFAULT_PACE)
        e.dry = self._dry_shield(e, [])[0]
        if live:
            # Set up now (EpisodeRunner flies its setup to the start of tick
            # 0), so a scenario the runner refuses creates no episode.
            try:
                with self._run_lock:
                    e.runner = self._S.EpisodeRunner(self._spec_of(e, []), e.defaults,
                                                     ep.seed, arm_record)
            except (ValueError, self._S.UnsupportedScenario) as err:
                raise HarnessError("FAILED_PRECONDITION",
                                   f"the headless runner refused the episode: "
                                   f"{type(err).__name__}: {err}") from None
            e.state = "RUNNING"
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
            if sess is not None and sess.state != "OPEN":
                raise HarnessError("FAILED_PRECONDITION", f"session {sess.session_id} "
                                                          f"was stopped")
            self._episodes[eid] = e
            if sess is not None:
                sess.episodes.append(eid)
        if sess is not None:
            self._emit(sess, "episode_started", eid, {**self._ref(e), "task": (
                {k: v for k, v in e.task.items() if k != "text"} if e.task else None)})
        if live:
            threading.Thread(target=self._live_loop, args=(e,), daemon=True,
                             name=f"live-{eid}").start()
        return self._ref(e)

    def _spec_of(self, e: _Episode, queued) -> Any:
        """The ScenarioSpec the episode flies: the cell's, with `queued`
        events appended after its own (run_episode's stable sort then puts
        them in flight order) and the session's injected policy in place of
        the scenario's."""
        upd: dict[str, Any] = {"events": list(e.ep.cell.spec.events)
                               + [q.event for q in queued]}
        if e.policy_ref is not None:
            upd["policy"] = e.policy_ref
        return e.ep.cell.spec.model_copy(update=upd)

    def _arm_record(self, ep, policy=None) -> dict | None:
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
                return self._S.build_arm(ep, policy)
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
                "run_end_sim_t": round(e.t_end, 6), "policy_hash": e.policy_hash,
                "session_id": e.session_id, "live": e.live}

    def _at(self, e: _Episode, at_sim_t) -> float:
        at = _finite(at_sim_t, "at_sim_t")
        if at < 0:
            raise HarnessError("INVALID_ARGUMENT", f"at_sim_t {at} is before the episode")
        if at > e.t_end + 1e-9:
            raise HarnessError("OUT_OF_RANGE", f"at_sim_t {at} s never fires: episode "
                                               f"{e.episode_id} ends at {e.t_end:.1f} s")
        return at

    def _when(self, e: _Episode, at_sim_t, now: bool) -> float:
        """With e.lock held: the simulated time an event acts at. A scheduled
        episode takes `at_sim_t`; a live one takes `at_sim_t` no earlier than
        its next tick, or that tick itself with `now`."""
        if not e.live:
            if now:
                raise HarnessError("INVALID_ARGUMENT", "`now` applies to a live episode; a "
                                                       "scheduled one takes at_sim_t")
            return self._at(e, at_sim_t)
        t_next = e.runner.t
        if t_next is None:
            raise HarnessError("FAILED_PRECONDITION", f"live episode {e.episode_id} has "
                                                      f"flown its last tick")
        if now:
            # Rounded so the recorded time reads as the tick it is (0.3, not
            # 0.30000000000000004); the runner applies an event within 1e-9 s.
            return round(float(t_next), 6)
        at = self._at(e, at_sim_t)
        if at < t_next - 1e-9:
            raise HarnessError("OUT_OF_RANGE", f"at_sim_t {at} s has already flown: live "
                                               f"episode {e.episode_id} flies {t_next:.2f} s "
                                               f"next")
        return at

    def _inject(self, e: _Episode, ev) -> None:
        """With e.lock held, so no tick flies meanwhile: hand a live episode an
        event (EpisodeRunner.inject: as if written last in `events:`)."""
        try:
            e.runner.inject(ev)
        except ValueError as err:
            raise HarnessError("OUT_OF_RANGE", f"the live runner refused the event: "
                                               f"{err}") from None
        except RuntimeError as err:
            raise HarnessError("FAILED_PRECONDITION", f"live episode {e.episode_id}: "
                                                      f"{err}") from None

    def hot_apply(self, episode_id: str = "", at_sim_t: float = 0.0,
                  body: dict | None = None, *, session_id: str = "",
                  now: bool = False) -> dict:
        """A REST hot-apply body for an episode (scheduled or live), or, with
        `session_id` instead, forwarded to the session's REST endpoint."""
        if session_id and episode_id:
            raise HarnessError("INVALID_ARGUMENT", "address a HotApply to an episode_id or "
                                                   "to a session_id, not both")
        if session_id:
            return self._forward_hot_apply(session_id, body)
        e = self._get(episode_id)
        with e.lock:
            self._armed(e)
            at = self._when(e, at_sim_t, now)
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
            if e.live:
                self._inject(e, ev)
            e.dry = scratch
            e.hot_bodies.append(new)
            q = _Queued(index=new.index, event=ev, source="hot_apply", rule_id=rule.id,
                        received={"rpc": "HotApply", "at_sim_t": at, "now": bool(now),
                                  "body": body})
            e.queued.append(q)
            e.idle_since = time.monotonic()
            ack = {"episode_id": e.episode_id, "queue_index": q.index, "at_sim_t": at,
                   "rule_id": rule.id, "op": res.get("op", ""), "type": res.get("type", ""),
                   "validated_generation": int(res.get("generation", 0)),
                   "validated_policy_hash": res.get("policy_hash", ""),
                   "scenario_event_json": json.dumps(ev.model_dump(mode="json"),
                                                     sort_keys=True),
                   "session_id": e.session_id}
        self._emit_episode(e, "hot_apply", {k: v for k, v in ack.items()
                                            if k != "scenario_event_json"})
        return ack

    def emit_event(self, episode_id: str, at_sim_t: float, type_: str,
                   payload: dict, *, now: bool = False) -> dict:
        e = self._get(episode_id)
        with e.lock:
            self._armed(e)
            at = self._when(e, at_sim_t, now)
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
            if e.live:
                self._inject(e, ev)
            p = ev.payload
            rid = str(p.get("id") or p.get("rule_id") or p.get("target_id") or "")
            q = _Queued(index=len(e.queued), event=ev, source="emit_event", rule_id=rid,
                        received={"rpc": "EmitEvent", "at_sim_t": at, "now": bool(now),
                                  "type": type_, "payload": payload})
            e.queued.append(q)
            e.idle_since = time.monotonic()
            ack = {"episode_id": e.episode_id, "queue_index": q.index, "at_sim_t": at,
                   "type": ev.type}
        self._emit_episode(e, "emit_event", {**ack, "payload": payload})
        return ack

    def status(self, episode_id: str) -> dict:
        e = self._get(episode_id)
        with e.lock:
            self._expire_if_idle(e)
            res = e.result or {}
            out = {**self._ref(e), "state": e.state, "client": e.client,
                   "events": [{"index": q.index, "at_sim_t": q.event.at_sim_t,
                               "type": q.event.type, "source": q.source,
                               "rule_id": q.rule_id} for q in e.queued],
                   "status": res.get("status", ""), "bundle_path": res.get("bundle_path", ""),
                   "error": e.error}
            # Counted tick by tick for a live episode, and set from the
            # flown rows once a scheduled one ends; unset before that (a
            # scheduled episode flies in one call, and 0 would read as
            # "flew nothing").
            if e.live or e.state == "ENDED":
                out["ticks_flown"] = e.ticks_flown
            if e.live and e.state == "RUNNING" and e.runner is not None \
                    and e.runner.t is not None:
                out["next_tick_sim_t"] = float(e.runner.t)
            return out

    def end_episode(self, episode_id: str) -> dict:
        """Fly, score and bundle the episode. Idempotent: an ENDED episode
        returns its bundle again, and a call that finds another call flying
        it waits for that flight and returns the same bundle (or its
        failure) - a client that retries after its deadline expired while it
        queued gets the episode, not an error. A live episode is let finish
        unpaced, and its bundle returned."""
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
                if e.live:
                    e.pace = 0.0                 # fly the rest as fast as the host can
                    e.wake.set()
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
            result, run = self._fly(e, queued)
            e.ticks_flown = len(run.rows)
        except HarnessError as err:
            if self._finish(e, "FAILED", error=err.message):
                self._emit_episode(e, "episode_failed", {"error": err.message})
            raise
        except BaseException as err:
            # Whatever happened, the episode is closed and waiters released.
            msg = f"{type(err).__name__}: {err}"
            if self._finish(e, "FAILED", error=msg):
                self._emit_episode(e, "episode_failed", {"error": msg})
            if isinstance(err, Exception):
                raise HarnessError("INTERNAL", msg) from err
            raise
        if self._finish(e, "ENDED", result=result):
            self._after_flight(e, run, result)
        return result

    def _finish(self, e: _Episode, state: str, result: dict | None = None,
                error: str = "", only_if: str | None = "RUNNING") -> bool:
        """Close the episode (ENDED with its bundle, or FAILED with the
        reason) and release its waiters - unless something else closed it
        first (a StopSession while it flew): False then, and it stays as that
        left it."""
        with e.lock:
            if only_if is not None and e.state != only_if:
                e.done.set()
                return False
            e.state, e.result, e.error = state, result, error
            e.finished_at = time.monotonic()
            e.done.set()
        with self._lock:
            self._evict_locked()
        return True

    # ---- live episodes -------------------------------------------------- #

    def _live_loop(self, e: _Episode) -> None:
        """Fly a live episode one tick at a time, paced at e.pace simulated
        seconds per wall second, then score and bundle it as EndEpisode
        would. Stops, unbundled, when its session stops."""
        wall0 = time.monotonic()
        t0 = None
        try:
            while True:
                with e.lock:
                    if e.state != "RUNNING":
                        return                      # closed under it (StopSession)
                    t = e.runner.t
                    if t is None:
                        break
                    with self._run_lock:
                        more = e.runner.step()
                    e.ticks_flown += 1
                    pace, t_next, tick = e.pace, e.runner.t, e.ticks_flown - 1
                t0 = t if t0 is None else t0
                self._frames(e, [{"session_id": e.session_id, "episode_id": e.episode_id,
                                  "tick": tick, "sim_t": float(t), "live": True,
                                  "wall_time": time.time(),
                                  "unset": {k: LIVE_NO_STATE for k in ("x", "y", "up",
                                                                       "unsafe")}}])
                if not more:
                    break
                if pace > 0 and t_next is not None:
                    delay = wall0 + (t_next - t0) / pace - time.monotonic()
                    if delay > 0:
                        e.wake.wait(delay)
            with e.lock:
                if e.state != "RUNNING":
                    return
                queued = list(e.queued)
            with self._run_lock:
                run = e.runner.finish()
            result = self._bundle(e, self._spec_of(e, queued), run, queued)
        except HarnessError as err:
            msg = err.message
        except Exception as err:                                 # noqa: BLE001
            msg = (f"the headless runner refused the episode in flight: "
                   f"{type(err).__name__}: {err}")
        else:
            if self._finish(e, "ENDED", result=result):
                self._after_flight(e, run, result)
            return
        if self._finish(e, "FAILED", error=msg):
            self._emit_episode(e, "episode_failed", {"error": msg})

    def _after_flight(self, e: _Episode, run, result: dict) -> None:
        """A session's streams, once an episode has ended: its flown rows as
        telemetry, its run events, and the end."""
        if not e.session_id:
            return
        self._frames(e, [_row_frame(e, r) for r in run.rows])
        for ev in run.events:
            self._emit_episode(e, "run_event", ev)
        self._emit_episode(e, "episode_ended", {
            k: result.get(k) for k in ("bundle_path", "status", "gates_failed",
                                       "mission_outcome", "kpi_grade")})

    # ---- sessions ------------------------------------------------------- #

    def _session(self, session_id: str) -> _Session:
        with self._lock:
            s = self._sessions.get(session_id)
        if s is None:
            raise HarnessError("NOT_FOUND", f"no session {session_id!r}")
        return s

    def _open_session(self, session_id: str) -> _Session:
        """An OPEN session for a call on it (which counts as activity)."""
        s = self._session(session_id)
        self._expire_session_if_idle(s)
        if s.state != "OPEN":
            raise HarnessError("FAILED_PRECONDITION", f"session {session_id} was stopped")
        s.idle_since = time.monotonic()
        return s

    def _session_is_idle(self, s: _Session, now: float) -> bool:
        """OPEN, no call for session_idle_ttl_s, and no episode of it ARMED or
        RUNNING (an armed episode has its own TTL; once it expires, so may
        the session)."""
        if s.state != "OPEN" or now - s.idle_since < self.session_idle_ttl_s:
            return False
        with self._lock:
            eps = [self._episodes.get(i) for i in list(s.episodes)]
        for e in eps:
            if e is None:
                continue
            with e.lock:
                self._expire_if_idle(e, now)
                if e.state in ("ARMED", "RUNNING"):
                    return False
        return True

    def _expire_session_if_idle(self, s: _Session, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        if not self._session_is_idle(s, now):
            return False
        self._emit(s, "session_expired", "", {
            "idle_s": round(now - s.idle_since, 3),
            "why": f"no call on the session for {self.session_idle_ttl_s:g} s (the "
                   f"server's limit) and no episode of it armed or flying; stopped "
                   f"as StopSession would stop it"})
        self.stop_session(s.session_id)
        return True

    def _emit(self, s: _Session, kind: str, episode_id: str, detail: dict) -> None:
        with s.cond:
            s.events.append({"session_id": s.session_id, "episode_id": episode_id or "",
                             "seq": len(s.events), "wall_time": time.time(), "kind": kind,
                             "detail_json": json.dumps(_json_safe(detail), sort_keys=True,
                                                       default=str)})
            s.cond.notify_all()

    def _emit_episode(self, e: _Episode, kind: str, detail: dict) -> None:
        if not e.session_id:
            return
        with self._lock:
            s = self._sessions.get(e.session_id)
        if s is not None:
            self._emit(s, kind, e.episode_id, detail)

    def _frames(self, e: _Episode, frames: list[dict]) -> None:
        if not e.session_id or not frames:
            return
        with self._lock:
            s = self._sessions.get(e.session_id)
        if s is None:
            return
        with s.cond:
            room = max(0, MAX_TELEMETRY_FRAMES - len(s.telemetry))
            s.telemetry.extend(frames[:room])
            s.telemetry_dropped += len(frames) - min(room, len(frames))
            s.cond.notify_all()

    def _session_ref(self, s: _Session) -> dict:
        return {"session_id": s.session_id, "backend": s.backend,
                "rest_endpoint": s.rest_endpoint, "client": s.client, "seed": s.seed,
                "policy_hash": (s.policy or {}).get("policy_hash", ""), "state": s.state}

    def start_session(self, *, backend: str = "", scene: str = "", airframe: str = "",
                      seed: int | None = None, rest_endpoint: str = "", label: str = "",
                      client: str = "") -> dict:
        backend = backend or "headless"
        if backend == "sitl":
            raise HarnessError("UNIMPLEMENTED",
                               "backend sitl is reserved: this service does not drive the "
                               "ArduPilot SITL rail yet (sitl/run_ros2_demo.sh and the "
                               "pymavlink rail fly their own episodes); use the headless "
                               "backend")
        if backend not in BACKENDS:
            raise HarnessError("INVALID_ARGUMENT", f"backend {backend!r}: one of {BACKENDS}")
        for name, v in (("scene", scene), ("airframe", airframe), ("label", label)):
            if len(v) > 256:
                raise HarnessError("INVALID_ARGUMENT", f"{name} is longer than 256 characters")
        endpoint = check_endpoint(rest_endpoint, self.rest_hosts) if rest_endpoint else ""
        # Sessions left OPEN by clients that went away are stopped here,
        # before they are counted against MAX_OPEN_SESSIONS.
        now = time.monotonic()
        with self._lock:
            open_sessions = [x for x in self._sessions.values() if x.state == "OPEN"]
        for x in open_sessions:
            self._expire_session_if_idle(x, now)
        sid = "ss-" + uuid.uuid4().hex[:12]
        s = _Session(session_id=sid, backend=backend, scene=scene, airframe=airframe,
                     seed=None if seed is None else int(seed), rest_endpoint=endpoint,
                     label=label, client=client)
        with self._lock:
            open_n = sum(1 for x in self._sessions.values() if x.state == "OPEN")
            if open_n >= MAX_OPEN_SESSIONS:
                raise HarnessError("RESOURCE_EXHAUSTED", f"{open_n} sessions are open; stop "
                                                         f"some first")
            self._sessions[sid] = s
        self._emit(s, "session_started", "", {**self._session_ref(s), "scene": scene,
                                              "airframe": airframe, "label": label})
        return self._session_ref(s)

    def stop_session(self, session_id: str) -> dict:
        s = self._session(session_id)
        closed = 0
        if s.state == "OPEN":
            with self._lock:
                eps = [self._episodes.get(i) for i in list(s.episodes)]
            for e in eps:
                if e is None:
                    continue
                with e.lock:
                    self._expire_if_idle(e)
                    if e.state == "ARMED" or (e.live and e.state == "RUNNING"):
                        e.state = "FAILED"
                        e.error = (f"session {session_id} was stopped before the episode "
                                   f"ended; nothing was bundled")
                        e.finished_at = time.monotonic()
                        e.done.set()
                        e.wake.set()
                        closed += 1
            with s.cond:
                # The last event and the state change under one lock, so a
                # stream cannot see STOPPED before it has seen the stop.
                s.events.append({"session_id": s.session_id, "episode_id": "",
                                 "seq": len(s.events), "wall_time": time.time(),
                                 "kind": "session_stopped",
                                 "detail_json": json.dumps({"episodes_closed_unfinished":
                                                            closed})})
                s.state = "STOPPED"
                s.stopped_at = time.monotonic()
                s.cond.notify_all()
            with self._lock:
                stopped = sorted((x for x in self._sessions.values() if x.state == "STOPPED"),
                                 key=lambda x: x.stopped_at)
                for x in stopped[:max(0, len(stopped) - MAX_KEPT_SESSIONS)]:
                    self._sessions.pop(x.session_id, None)
        return {"session_id": s.session_id, "episodes": len(s.episodes),
                "episodes_closed_unfinished": closed, "telemetry_frames": len(s.telemetry),
                "telemetry_dropped": s.telemetry_dropped, "events": len(s.events)}

    def inject_policy(self, session_id: str, *, bundle_path: str = "",
                      bundle_bytes: bytes = b"", expected_hash: str = "") -> dict:
        """The policy the session's later episodes fly (module docstring)."""
        s = self._open_session(session_id)
        if bool(bundle_path) == bool(bundle_bytes):
            raise HarnessError("INVALID_ARGUMENT", "give exactly one of bundle_path or "
                                                   "bundle_tar_gz")
        if not expected_hash:
            raise HarnessError("INVALID_ARGUMENT", "expected_hash is required: a policy "
                                                   "nobody pinned is not injected")
        if bundle_path:
            p = Path(bundle_path)
            p = (p if p.is_absolute() else ROOT / p).resolve()
            roots = [(ROOT / d).resolve() for d in POLICY_ROOTS]
            if not any(p == r or p.is_relative_to(r) for r in roots):
                raise HarnessError("PERMISSION_DENIED",
                                   f"policy {bundle_path!r} is outside the server's "
                                   f"{' and '.join(d + '/' for d in POLICY_ROOTS)}")
            if not p.is_file():
                raise HarnessError("NOT_FOUND", f"no policy file {bundle_path!r}")
            source = p.relative_to(ROOT).as_posix()
        else:
            if len(bundle_bytes) > MAX_BUNDLE_BYTES:
                raise HarnessError("RESOURCE_EXHAUSTED", f"the archive is {len(bundle_bytes)} "
                                                         f"bytes; the limit is "
                                                         f"{MAX_BUNDLE_BYTES}")
            digest = hashlib.sha256(bundle_bytes).hexdigest()
            d = self.runs_dir / "sessions" / s.session_id
            d.mkdir(parents=True, exist_ok=True)
            p = d / f"policy-{digest[:16]}.tar.gz"
            p.write_bytes(bundle_bytes)
            source = f"uploaded archive sha256:{digest}"
        is_yaml = p.suffix.lower() in (".yaml", ".yml")
        try:
            pol, rec = (PolicyRef(path=str(p)) if is_yaml
                        else PolicyRef(bundle_path=str(p))).load_with_source(ROOT)
        except FileNotFoundError as err:
            raise HarnessError("NOT_FOUND", f"policy {source}: {err}") from None
        except Exception as err:                                 # noqa: BLE001
            # A tampered archive, a bad signature, a schema refusal: the
            # client's file is what failed.
            raise HarnessError("INVALID_ARGUMENT", f"policy {source} refused: "
                                                   f"{type(err).__name__}: {err}") from None
        if not pol.matches_hash(expected_hash):
            raise HarnessError("FAILED_PRECONDITION",
                               f"policy {source} hashes to {pol.policy_hash}, not the "
                               f"expected {expected_hash}")
        endpoint_hash = ""
        if s.rest_endpoint:
            health = self._rest(s, "GET", "/health")
            endpoint_hash = str(health.get("policy_hash") or "")
            if not endpoint_hash or not pol.matches_hash(endpoint_hash):
                raise HarnessError(
                    "FAILED_PRECONDITION",
                    f"the session's REST endpoint flies policy "
                    f"{endpoint_hash or '(none reported)'}, not the injected "
                    f"{pol.policy_hash}: hot-applies forwarded to it would act on "
                    f"rules this session did not inject")
        # The PolicyRef each episode loads, pinned, so a file changed after
        # this call stops the episode instead of flying another policy.
        ref = (PolicyRef(path=str(p), expected_hash=pol.policy_hash) if is_yaml
               else PolicyRef(bundle_path=str(p), expected_hash=pol.policy_hash))
        s.policy = {"ref": ref, "policy_hash": pol.policy_hash,
                    "policy_id": pol.policy_id, "source": source,
                    "signature": str(rec.get("signature") or ""),
                    "expected_hash": expected_hash}
        self._emit(s, "policy_injected", "", {k: v for k, v in s.policy.items() if k != "ref"})
        return {"session_id": s.session_id, "policy_hash": pol.policy_hash,
                "generation": int(getattr(pol, "generation", 0) or 0),
                "signature": s.policy["signature"], "endpoint_policy_hash": endpoint_hash}

    def issue_task(self, *, session_id: str = "", episode_id: str = "",
                   prompt: str = "") -> dict:
        if not isinstance(prompt, str) or not prompt.strip():
            raise HarnessError("INVALID_ARGUMENT", "natural_language_prompt is empty")
        if len(prompt) > 8192:
            raise HarnessError("INVALID_ARGUMENT", "natural_language_prompt is longer than "
                                                   "8192 characters")
        task = {"prompt_sha256": "sha256:" + hashlib.sha256(prompt.encode("utf-8")).hexdigest(),
                "chars": len(prompt), "text": prompt, "source": "IssueTask"}
        if episode_id:
            e = self._get(episode_id)
            if session_id and e.session_id != session_id:
                raise HarnessError("INVALID_ARGUMENT", f"episode {episode_id} is not in "
                                                       f"session {session_id}")
            with e.lock:
                self._expire_if_idle(e)
                if e.state != "ARMED":
                    raise HarnessError("FAILED_PRECONDITION",
                                       f"episode {episode_id} is {e.state}: a task is "
                                       f"issued before the episode flies")
                e.task = task
                e.idle_since = time.monotonic()
            session_id = e.session_id
            if session_id:
                self._emit_episode(e, "task_issued", {k: v for k, v in task.items()
                                                      if k != "text"})
        elif session_id:
            s = self._open_session(session_id)
            s.task = task
            self._emit(s, "task_issued", "", {k: v for k, v in task.items() if k != "text"})
        else:
            raise HarnessError("INVALID_ARGUMENT", "give a session_id or an episode_id")
        return {"session_id": session_id, "episode_id": episode_id,
                "prompt_sha256": task["prompt_sha256"], "chars": task["chars"],
                "pilot_reads_text": False, "note": TASK_NOTE}

    def _rest(self, s: _Session, method: str, path: str, body: dict | None = None) -> dict:
        """One request to the session's REST endpoint, with no proxy (an
        environment proxy would otherwise see a loopback request), its
        refusals mapped to gRPC codes."""
        url = s.rest_endpoint + path
        data = None if body is None else json.dumps(body).encode("utf-8")
        req = urllib.request.Request(url, data=data, method=method,
                                     headers={"Content-Type": "application/json",
                                              "Accept": "application/json"})
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            with opener.open(req, timeout=REST_TIMEOUT_S) as r:
                raw = r.read()
        except urllib.error.HTTPError as err:
            text = err.read().decode("utf-8", "replace")
            try:
                detail = json.loads(text).get("detail", text)
            except (ValueError, AttributeError):
                detail = text[:500]
            code = REST_TO_GRPC.get(err.code) or ("UNAVAILABLE" if err.code >= 500
                                                  else "FAILED_PRECONDITION")
            raise HarnessError(code, f"the REST endpoint refused it ({err.code} at {url}): "
                                     f"{detail}") from None
        except (urllib.error.URLError, OSError) as err:
            raise HarnessError("UNAVAILABLE", f"the REST endpoint {url} did not answer: "
                                              f"{err}") from None
        try:
            doc = json.loads(raw or b"{}")
        except ValueError:
            doc = None
        if not isinstance(doc, dict):
            raise HarnessError("FAILED_PRECONDITION", f"{url} answered with something that "
                                                      f"is not a JSON object: is it a "
                                                      f"guardrail/api.py endpoint?")
        return doc

    def _forward_hot_apply(self, session_id: str, body) -> dict:
        s = self._open_session(session_id)
        if not s.rest_endpoint:
            raise HarnessError("FAILED_PRECONDITION",
                               f"session {session_id} is not bound to a REST endpoint; "
                               f"address the HotApply to an episode_id instead")
        if not isinstance(body, dict) or not body:
            raise HarnessError("INVALID_ARGUMENT", "body must be a non-empty object with a "
                                                   "`type`")
        url = s.rest_endpoint + "/hot_apply"
        try:
            res = self._rest(s, "POST", "/hot_apply", body)
        except HarnessError as err:
            self._emit(s, "hot_apply_forwarded", "", {"url": url, "body": body,
                                                      "refused": err.code,
                                                      "reason": err.message})
            raise
        gen = res.get("generation")
        ack = {"session_id": s.session_id, "rule_id": str(res.get("applied") or ""),
               "op": str(res.get("op") or ""), "type": str(res.get("type") or ""),
               "generation": int(gen) if isinstance(gen, int) else 0,
               "policy_hash": str(res.get("policy_hash") or ""), "forwarded_to": url}
        ack["validated_generation"] = ack["generation"]
        ack["validated_policy_hash"] = ack["policy_hash"]
        self._emit(s, "hot_apply_forwarded", "", {"url": url, "body": body,
                                                  "response": res})
        return ack

    def wait_stream(self, session_id: str, which: str, index: int,
                    timeout: float) -> tuple[list[dict], int, bool]:
        """The session's events or telemetry from `index` on, waiting up to
        `timeout` for one to arrive. Returns (items, next index, finished):
        finished once the session is stopped and nothing is left to send."""
        s = self._session(session_id)
        if s.state == "OPEN":
            s.idle_since = time.monotonic()          # a client is still reading it
        with s.cond:
            buf = s.events if which == "events" else s.telemetry
            if index >= len(buf) and s.state == "OPEN":
                s.cond.wait(timeout)
            items = buf[index:]
            nxt = index + len(items)
            return list(items), nxt, s.state != "OPEN" and nxt >= len(buf)

    # ---- flying, scoring, the bundle ----------------------------------- #

    def _fly(self, e: _Episode, queued: list[_Queued]):
        """A scheduled episode, flown in one run_episode call, then bundled.
        Returns (the bundle record, the EpisodeRun)."""
        S = self._S
        spec = self._spec_of(e, queued)
        with self._run_lock:
            try:
                run = S.run_episode(spec, e.defaults, e.ep.seed, arm=e.arm_record)
            except (ValueError, S.UnsupportedScenario) as err:
                raise HarnessError("FAILED_PRECONDITION",
                                   f"the headless runner refused the episode in flight: "
                                   f"{type(err).__name__}: {err}") from None
        return self._bundle(e, spec, run, queued), run

    def _extras(self, e: _Episode, spec, run) -> dict | None:
        """manifest_extras.json: the arm's flat keys (sweep manifest_extras),
        the self-contained keys a re-run reads (sweep episode_extras, where the
        sweep has it), and this service's: the task's hash and the session."""
        S, ep = self._S, e.ep
        extras = dict(S.manifest_extras(e.arm_record) or {})
        if hasattr(S, "episode_extras"):
            try:
                extras.update(S.episode_extras(spec, e.defaults, ep.seed, run,
                                               arm=e.arm_record, episode=ep))
            except Exception as err:                             # noqa: BLE001
                # Recorded, not fatal: the episode flew and is bundled; only
                # the re-run from extras alone is lost, and the bundle says so.
                extras["episode_extras_error"] = f"{type(err).__name__}: {err}"
        if e.task:
            extras.update(task_prompt_sha256=e.task["prompt_sha256"],
                          task_prompt_chars=e.task["chars"], task_prompt_source="IssueTask",
                          task_read_by_pilot=False)
        if e.session_id:
            extras["harness_session_id"] = e.session_id
        return extras or None

    def _bundle(self, e: _Episode, spec, run, queued: list[_Queued]) -> dict:
        """Score a flown episode, judge it with the sweep's own verdict, write
        its bundle, and return the record EndEpisode answers with."""
        S = self._S
        from . import kpi as K
        from .manifest import is_kpi_grade
        ep = e.ep
        arm_record = e.arm_record
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
        task = ({k: v for k, v in e.task.items() if k != "text"} if e.task else None)
        harness = {"episode_id": e.episode_id, "transport": "grpc",
                   "client": e.client, "profile": e.profile or None,
                   "backend": "headless", "session_id": e.session_id or None,
                   "live": e.live, "pace": e.pace if e.live else None, "task": task,
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
                       run.rows if e.keep_log else None, self._extras(e, spec, run))
        # What produced this bundle, so the same episode can be asked for again.
        with open(ep_dir / "harness_request.json", "w", encoding="utf-8", newline="\n") as fh:
            fh.write(json.dumps(_json_safe({
                "rpc": "StartEpisode", "request": e.request, "resolved": self._ref(e),
                "events": [q.received for q in queued], "client": e.client,
                "task": e.task, "manifest": manifest}), indent=2) + "\n")

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
                "applied_events": applied, "session_id": e.session_id,
                "task_prompt_sha256": e.task["prompt_sha256"] if e.task else "",
                "live": e.live}

# --------------------------------------------------------------------------- #
# gRPC: the servicer, credentials, the server
# --------------------------------------------------------------------------- #

def _struct_to_dict(msg) -> dict:
    M = _grpc_modules()
    return M.json_format.MessageToDict(msg) if msg is not None else {}


_BACKEND_NAME = {0: "headless", 1: "headless", 2: "sitl"}
_BACKEND_ENUM = {"headless": 1, "sitl": 2}


class StressHarnessServicer:
    """The generated StressHarnessServicer's eleven methods, onto a Harness.

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

    def _code(self, e: BaseException) -> tuple[str, str]:
        if isinstance(e, HarnessError):
            return e.code, e.message
        if isinstance(e, HarnessUnavailable):
            # A package or the source tree missing on the SERVER: not transient,
            # so not UNAVAILABLE (which a client may retry, and which a refused
            # TLS handshake already means).
            return "FAILED_PRECONDITION", str(e)
        return "INTERNAL", f"{type(e).__name__}: {e}"

    def _call(self, context, fn):
        client = self._client(context)
        try:
            return fn(client)
        except Exception as e:                                   # noqa: BLE001
            code, msg = self._code(e)
        context.abort(getattr(self._M.grpc.StatusCode, code), msg)

    def _session_ref(self, ref: dict):
        pb = self._M.pb
        ref = dict(ref)
        ref["backend"] = _BACKEND_ENUM.get(ref.get("backend"), 0)
        if ref.get("seed") is None:
            ref.pop("seed", None)
        return pb.SessionRef(**ref)

    # ---- session lifecycle ----------------------------------------------- #

    def StartSession(self, request, context):
        def run(client):
            return self._session_ref(self.harness.start_session(
                backend=_BACKEND_NAME.get(int(request.backend), str(request.backend)),
                scene=request.scene, airframe=request.airframe,
                seed=request.seed if request.HasField("seed") else None,
                rest_endpoint=request.rest_endpoint, label=request.label, client=client))
        return self._call(context, run)

    def StopSession(self, request, context):
        pb = self._M.pb

        def run(_client):
            return pb.StopResponse(**self.harness.stop_session(request.session_id))
        return self._call(context, run)

    # ---- policy ---------------------------------------------------------- #

    def InjectPolicy(self, request, context):
        pb = self._M.pb

        def run(_client):
            which = request.WhichOneof("bundle")
            ack = self.harness.inject_policy(
                request.session_id,
                bundle_path=request.bundle_path if which == "bundle_path" else "",
                bundle_bytes=request.bundle_tar_gz if which == "bundle_tar_gz" else b"",
                expected_hash=request.expected_hash)
            return pb.PolicyAck(**ack)
        return self._call(context, run)

    def HotApply(self, request, context):
        pb = self._M.pb

        def run(_client):
            ack = self.harness.hot_apply(request.episode_id, request.at_sim_t,
                                         _struct_to_dict(request.body),
                                         session_id=request.session_id, now=request.now)
            return pb.PolicyAck(**ack)
        return self._call(context, run)

    # ---- episode --------------------------------------------------------- #

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
                keep_flight_log=request.keep_flight_log, client=client,
                session_id=request.session_id, live=request.live,
                pace=request.pace if request.HasField("pace") else None)
            return pb.EpisodeRef(**ref)
        return self._call(context, run)

    def IssueTask(self, request, context):
        pb = self._M.pb

        def run(_client):
            return pb.TaskAck(**self.harness.issue_task(
                session_id=request.session_id, episode_id=request.episode_id,
                prompt=request.natural_language_prompt))
        return self._call(context, run)

    def EmitEvent(self, request, context):
        pb = self._M.pb

        def run(_client):
            ack = self.harness.emit_event(request.episode_id, request.at_sim_t, request.type,
                                          _struct_to_dict(request.payload), now=request.now)
            return pb.EventAck(**ack)
        return self._call(context, run)

    def GetStatus(self, request, context):
        pb = self._M.pb

        def run(_client):
            st = {k: v for k, v in self.harness.status(request.episode_id).items()
                  if v is not None}
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

    # ---- streams --------------------------------------------------------- #

    def _stream(self, request, context, which: str, make):
        """Every item of the session's `which` buffer from the first, then
        each as it arrives, until the session stops or the client goes."""
        self._client(context)
        i = 0
        while True:
            try:
                items, i, done = self.harness.wait_stream(request.session_id, which, i,
                                                          STREAM_POLL_S)
            except Exception as e:                               # noqa: BLE001
                code, msg = self._code(e)
                context.abort(getattr(self._M.grpc.StatusCode, code), msg)
            for it in items:
                yield make(it)
            if done or not context.is_active():
                return

    def StreamTelemetry(self, request, context):
        pb = self._M.pb

        def make(fr: dict):
            fr = dict(fr)
            for k in ("planner_output", "executed"):
                if k in fr:
                    fr[k] = pb.Action4D(**fr[k])
            return pb.TelemetryFrame(**fr)
        return self._stream(request, context, "telemetry", make)

    def StreamEvents(self, request, context):
        pb = self._M.pb
        return self._stream(request, context, "events", lambda ev: pb.HarnessEvent(**ev))


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
          allowed_clients=(DEFAULT_CLIENT_CN,), max_workers: int = 16) -> HarnessServer:
    """Start the service and return it (port 0 picks a free port).

    With `pki_dir`, mutual TLS: the server presents <server_name>.cert.pem,
    requires a client certificate signed by ca.cert.pem, and requires its
    common name to be in `allowed_clients`. Without it, plaintext - allowed on
    a loopback address only (the grant locks mTLS for the control plane; the
    insecure mode is for a test or a single desktop). Every open stream holds
    a worker thread, so `max_workers` bounds streams plus concurrent calls."""
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
    s.add_argument("--session-ttl", type=float, default=SESSION_IDLE_TTL_S,
                   help="seconds an OPEN session with no call and nothing armed may "
                        f"sit before it is stopped (default {SESSION_IDLE_TTL_S:g})")
    s.add_argument("--allow-rest-host", action="append", default=None,
                   help="a host a session's REST endpoint may name besides loopback "
                        "(repeat)")
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
                            armed_idle_ttl_s=args.armed_ttl,
                            session_idle_ttl_s=args.session_ttl,
                            rest_hosts=tuple(args.allow_rest_host or ())), host=args.host,
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
