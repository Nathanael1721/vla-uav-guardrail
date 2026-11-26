"""The stress harness's gRPC control plane over mutual TLS: the Stress Testing
page's ten RPCs (StartSession, StopSession, InjectPolicy, HotApply,
StartEpisode, IssueTask, EmitEvent, EndEpisode, StreamTelemetry,
StreamEvents) plus GetStatus, driving the headless harness and, on a server
started with a rig (serve --rail), the flight rails.

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
`backend` is HEADLESS, or a flight rail (SITL, HIL: see RAIL SESSIONS below;
UNIMPLEMENTED on a server with no rig for it). A session may be BOUND to a
REST hot-apply endpoint (guardrail/api.py, e.g. a rail's live Shield): a
HotApply addressed to the session is sent to the endpoint by its own route -
POST <endpoint>/hot_apply as it is, or, for an edit of a zone (a dynamic_nfz
body with "op": translate / rotate / scale / move and the zone's "id"),
PATCH <endpoint>/dynamic_nfz/<id> (rest_route, 2026-11-10: /hot_apply cannot
carry one) - and the endpoint's
answer returned, 409 / 404 / 422 mapped as below; the session's InjectPolicy
checks the endpoint flies that policy (GET /health). Only a loopback endpoint
is accepted unless the server allows its host (`rest_hosts`,
--allow-rest-host): the server makes the request, so an endpoint is a place a
client can make the server send to. No proxy is used for it. guardrail/api.py
serves plaintext on loopback only and mutual TLS anywhere else, so an https
endpoint needs the server's own REST TLS identity (`rest_pki_dir`,
serve --rest-pki: the lab CA and a client certificate on the endpoint's
allow-list, harness-client unless --rest-client names another); without one an
https endpoint is refused at StartSession. The endpoint's server certificate
must name the address the session uses (gen-pki --host, issue --server-host).

RAIL SESSIONS (2026-11-10, WP4-05 / WP4-03). A session whose backend is a
flight rail flies its episodes on the server's rig (`Harness(rail=RailRig(
...))`, `serve --rail sitl|hil`), through experiments/sweep_scenarios.py's own
rail backends and nothing else: run_hil with its transport (SITL and the
router in WSL, the scenario's files to the Orin, MAVROS restarted, the
mission container, the run folder fetched and accepted by its own records)
or run_sitl (sitl/run_ros2_demo.sh). One harness session is then one flown,
scored episode with its bundle:

  StartSession(backend, rest_endpoint)  the endpoint is the address the
                 MISSION's hot-apply listener will be started on;
  InjectPolicy   a file both hosts hold; the mission is launched pinned to
                 its hash (nothing flies yet, so GET /health is not asked);
  StartEpisode   LAUNCHES the mission (the scenario's own arguments, plus
                 the listener's flags when there is an endpoint); a scenario
                 the rail cannot fly as written is FAILED_PRECONDITION and
                 nothing is launched; one mission at a time. With
                 `drive_scenario_events` (2026-11-14) the scenario's own
                 events are taken off the mission (no --events) and
                 forwarded by the harness at their mission times
                 (scenario_rest_timeline; one with no REST form refuses the
                 episode, one that never reaches the Shield fails it,
                 unscored);
  HotApply       to the episode: queued for mission time `at_sim_t` and
                 forwarded then to the mission's listener (the ack says
                 `pending`); to the session: forwarded at once. Either is
                 kept, with its generation and the endpoint's receipt, for
                 the episode's harness_events.jsonl. EmitEvent (since
                 2026-11-14) is turned into its REST body
                 (rest_body_of_event) and queued the same way; a form with
                 no REST body (a zone placed against the flying vehicle, a
                 change of motion) is UNIMPLEMENTED;
  EndEpisode     waits for the mission, scores the fetched run as a
                 `--backend hil` campaign does (score, rail_gate_set on the
                 cell's own gates, episode_verdict, scenario_score.json
                 beside a hil run) and writes the bundle under the grant's
                 names: manifest.json (the mission's own), policy_bundle
                 .tar.gz (every generation's IR), episode.bag2/ (the run's
                 bag/, aliased), shield_audit.jsonl, harness_events.jsonl,
                 kpi.json. What the run folder lacks is "not written".

Mission time 0 is when the listener's GET /health first reports
mission_started (_rail_driver says how late that can be). Tested on the fake
rig tests/test_sweep.py flies its campaign on (tests/test_harness_rail.py);
NOT FLOWN: no session has driven the real rig.

PARAPHRASE ARMS (2026-11-07, WP4-10). The reference data-flow page draws the
"Paraphraser service (Q3 deliverable)" in the control plane feeding the
"Orchestrator service", which "issue[s] tasks (and paraphrased variants) to
the VLA", over "gRPC + mTLS"; this harness is that orchestrator. A paraphrase
arm (ppN) is drawn by the paraphraser service the server is bound to
(`Harness(paraphraser=...)`, `serve --paraphraser HOST:PORT`, mutual TLS with
the lab PKI's harness-client certificate - the name the service allows by
default - or plaintext on loopback with --insecure), never in process: the
reference design describes no in-process fallback, so there is none, and a
server bound to no service refuses a paraphrase arm (FAILED_PRECONDITION). The arm record
names the service, the transport, and the validator and store digests the
reply carried (`paraphrase.via`), beside the same ids an in-process draw
would have had. The canonical arm (the templated text itself) and the
protected-prompt check stay in process: they draw nothing. Until this date the
harness drew every arm in process through guardrail.paraphraser.

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
counting against MAX_OPEN_SESSIONS. StopSession lets a scheduled episode that
is already flying finish and WAITS for it: that episode's telemetry rows, run
events and `episode_ended` reach both streams before `session_stopped`, the
last item of each, and StopResponse counts them. The wait is the whole of that
flight as EndEpisode sees it - its turn at the one run lock every session's
scheduled flights share (behind any queued ahead of it), its run_episode and
its bundling - so under contention it is several run_episodes, not one (a
scheduled EndEpisode's median is 1.709 s in docs/DESIGN-harness-grpc.md): a
client should give StopSession a deadline to match.

WHAT IT IS NOT

  * Not KPI-grade. The headless backend's manifest names the topology
    `headless-kinematic` and a measured sim_speedup far above 1 (a live
    episode at pace 1.0 measures about 1, and is still headless), so
    guardrail.manifest.is_kpi_grade refuses every episode, and EpisodeBundle
    says so (`kpi_grade`, `kpi_grade_reasons`). The grant's KPI numbers come
    from the hil topology (a Jetson Orin, not built yet).
  * Not a simulator session on the headless backend. StartSession opens no
    simulator there: it has no scene or airframe to load (both are
    recorded). A rail session's simulator is started per episode by the
    sweep's rail backend, at the scenario's start. IssueTask records the
    prompt's hash; neither the headless pilot nor the rails' VLA stub reads
    text.
  * Not a scheduler. Scheduled episodes fly one at a time per server (one
    lock around run_episode; a live episode takes it one tick at a time); the
    reference's answer to throughput (risk R9) is parallel hosts.

STATUS CODES (what a client can rely on)

    INVALID_ARGUMENT     the spec, cell, seed, arm, event, session request or
                         policy fails validation (pydantic's message is
                         passed through); a NaN or Infinity number in a
                         HotApply body or EmitEvent payload; a REST 422
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
                         draw (bound to no paraphraser service, or one with no
                         LLM client or no stored set holding the prompt); a
                         hot-apply that would make an earlier-acked
                         one fail later in the flight; a session that is
                         stopped, or not bound to a REST endpoint for a
                         session HotApply; an injected bundle that does not
                         hash to `expected_hash`, or an endpoint flying
                         another policy; an https endpoint this server has no
                         TLS identity for, or one whose TLS handshake fails
                         (certificate not the lab CA's, wrong host, ours
                         refused); a package or source file missing on the
                         server
    UNIMPLEMENTED        a hot-apply field the headless runner cannot carry;
                         a rail backend (sitl, hil) on a server with no rig
                         for it; on a rail episode, an EmitEvent with no
                         REST form (with drive_scenario_events, a scenario
                         event with none is FAILED_PRECONDITION)
    RESOURCE_EXHAUSTED   too many open episodes or sessions, events in one
                         episode, or an InjectPolicy archive too large; a
                         rail StartEpisode while the rig flies another
    UNAVAILABLE          (from gRPC) a TLS handshake that failed: no client
                         certificate, one from another CA, or a plaintext
                         client on the mTLS port; (from the service) a bound
                         REST endpoint that did not answer, or answered 5xx
                         (an https endpoint that reset us says to check
                         that its allow-list names our certificate); the
                         bound paraphraser service that did not answer

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

The same root also certifies the keys that sign policy bundles
(`issue_policy_signer`, 2026-11-16): a codeSigning-only leaf over the bundle's
Ed25519 public key, valid as long as the CA is, which guardrail.bundle records
in its trust store and ships in each bundle, so a bundle's `signed_by` names
this CA (`python -m guardrail.bundle issue-cert`).

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

from .models import wgs84_from_metres
from .scenario_spec import (Episode, Family, Library, PolicyRef, ScenarioEvent,  # noqa: F401
                            arm_episodes, episode_arms, load_profile)

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
BACKENDS = ("headless", "sitl", "hil")
# The flight rails: flown by the server's rig (RailRig, serve --rail), through
# experiments/sweep_scenarios.py's own rail backends.
RAIL_BACKENDS = ("sitl", "hil")
# PATCH /dynamic_nfz/{id} (guardrail/api.py apply_nfz_edit): what /hot_apply
# cannot carry.
NFZ_EDIT_OPS = ("move", "translate", "rotate", "scale")
RAIL_ENDPOINT_POLL_S = 0.5       # a rail episode: is the mission's listener up yet?
RAIL_HEALTH_POLL_S = 0.2         # then: has the mission started (GET /health)?
EPISODE_BAG_DIR = "episode.bag2"
MISSION_T0_BASIS = ("this server's mission time 0 is when GET <endpoint>/health first "
                    "reported mission_started (the Shield's first filter() tick), "
                    f"polled every {RAIL_HEALTH_POLL_S:g} s: late by at most one poll "
                    "plus a round trip, never early; `rail.t` is the mission's own time "
                    "for the same event")
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


# The paraphraser service's ParaphraseRecord fields that make
# guardrail.paraphraser.Paraphrase.to_record(): an arm drawn over the wire is
# recorded exactly as one drawn in process was, plus where it came from.
_PARAPHRASE_RECORD = ("paraphrase_id", "source_id", "backend", "index", "seed", "text",
                      "text_sha256", "source_sha256")


@dataclass(frozen=True)
class ServiceParaphrase:
    """One paraphrase arm as the paraphraser service drew it, in the shape the
    sweep's build_arm takes (`text`, `to_record()`, `manifest_extras()`). The
    record adds `via`: the service, the transport, and the validator and store
    digests the reply carried, so an episode bundle names the exact validator
    and stored set its wording came from."""
    record: dict
    via: dict

    @property
    def text(self) -> str:
        return self.record["text"]

    def to_record(self) -> dict:
        out = {k: self.record.get(k) for k in _PARAPHRASE_RECORD}
        out["via"] = dict(self.via)
        return out

    def manifest_extras(self) -> dict:
        r = self.record
        return {"paraphrase_id": r.get("paraphrase_id"),
                "paraphrase_backend": r.get("backend"),
                "paraphrase_index": r.get("index"),
                "paraphrase_seed": r.get("seed"),
                "paraphrase_source_id": r.get("source_id"),
                "paraphrase_text_sha256": r.get("text_sha256"),
                "paraphrase_service": self.via.get("service")}


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
                    "scope", "motion"},
    "time_window_switch": {"id", "type", "target_id", "active", "valid_time"},
    "corridor_swap": {"id", "type", "target_id", "centerline", "width_m",
                      "altitude_floor_m", "altitude_ceiling_m", "valid_time"},
}
# The three ConstraintBase fields every rule states since IR schema 3.0.0. The
# runner's switch and swap events do not carry them: apply_event writes them
# at these values (what they defaulted to before), so a rule setting exactly
# these is carried, and any other value is not. The same for `scope`, which
# every rule states since IR schema 4.0.0: a zone's payload carries it, but
# the switch and swap events do not, and the runner writes "mission" on those
# (a switch or swap applied during an episode belongs to that mission; an
# absent scope meant "mission" before), so a hot-applied switch or swap of
# that scope is carried and one of another scope is not.
_RUNNER_WRITES = {kind: {"constraint_type": "hard", "priority": "P0",
                         "violation_action": "project_fix", "scope": "mission"}
                  for kind in ("time_window_switch", "corridor_swap")}


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
    set_fields -= {k for k, v in _RUNNER_WRITES.get(kind, {}).items()
                   if getattr(rule, k, None) == v}
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
            "violation_action": rule.violation_action, "scope": rule.scope}
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
    backend: str = "headless"        # the session's: headless | sitl | hil
    rail: Any = None                 # _RailFlight of a rail episode


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
    # Flights of its episodes (scheduled or live) that may still append rows
    # or events: StopSession waits for 0 before `session_stopped`.
    flying: int = 0
    cond: threading.Condition = field(default_factory=threading.Condition)   # an RLock


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


# --------------------------------------------------------------------------- #
# A hot-apply body's route on a guardrail/api.py endpoint; the rail rig
# --------------------------------------------------------------------------- #

def rest_route(body) -> tuple[str, str, dict]:
    """(method, path, payload) of a hot-apply body on a guardrail/api.py
    endpoint (2026-11-10, WP4-03).

    The grant's event list is spawn / translate / rotate a fence, activate /
    deactivate a rule, swap a corridor, scale a radius. POST /hot_apply
    carries a spawn (dynamic_nfz), a switch (time_window_switch) and a swap
    (corridor_swap), and nothing else: an EDIT of a zone that exists is the
    endpoint's PATCH /dynamic_nfz/{id}. So a body

        {"type": "dynamic_nfz", "op": "translate", "id": "nfz-1", "dx": 5, "dy": 0}
        {"type": "dynamic_nfz", "op": "rotate",    "id": "nfz-1", "angle_deg": 30}
        {"type": "dynamic_nfz", "op": "scale",     "id": "nfz-1", "factor": 1.5}
        {"type": "dynamic_nfz", "op": "move",      "id": "nfz-1", "vertices": [...]}

    goes as PATCH /dynamic_nfz/nfz-1 with the route's own body (everything
    but `type` and `id`); every other body goes to POST /hot_apply as it is,
    as before this date. Until then such an edit was POSTed to /hot_apply and
    refused there (422: a dynamic_nfz with no vertices), so the harness could
    not move, turn or resize a zone on a flying rail."""
    if not isinstance(body, dict) or not body:
        raise HarnessError("INVALID_ARGUMENT", "body must be a non-empty object with a "
                                               "`type`")
    if body.get("type") == "dynamic_nfz" and body.get("op") in NFZ_EDIT_OPS:
        zid = body.get("id")
        if not isinstance(zid, str) or not zid.strip():
            raise HarnessError("INVALID_ARGUMENT",
                               f"a dynamic_nfz {body['op']} names the zone it edits: `id`")
        if zid in (".", "..") or any(c in zid for c in "/\\") or any(
                ord(c) < 32 for c in zid):
            raise HarnessError("INVALID_ARGUMENT",
                               f"zone id {zid!r} cannot be a path segment of "
                               f"PATCH /dynamic_nfz/<id>")
        payload = {k: v for k, v in body.items() if k not in ("type", "id")}
        return "PATCH", "/dynamic_nfz/" + urllib.parse.quote(zid, safe=""), payload
    return "POST", "/hot_apply", body


# The fields guardrail.scenario_events.apply_event writes on a rule it builds
# from a switch or swap event (IR schema 3.0.0 / 4.0.0's required fields, at
# the values they defaulted to): a REST body has to state them.
_EVENT_RULE_BASE = {"constraint_type": "hard", "priority": "P0",
                    "violation_action": "project_fix", "scope": "mission"}
# Why a ScenarioEvent form has no REST body (rest_body_of_event).
_NO_REST_FORM = {
    "vehicle": ("a zone placed against the flying vehicle (width_m / height_m / "
                "ahead_of_vehicle_m / anchor): only the mission knows where the aircraft "
                "is when the event fires, and a REST body carries vertices; give the "
                "zone its `vertices`"),
    "motion": ("a change of a zone's motion (translate `mps` with a direction or "
               "bearing, rotate `deg_per_s`): the mission restarts it from the ring the "
               "flying Shield holds at that instant (Shield.zone_ring_at), and PATCH "
               "/dynamic_nfz move takes vertices only the mission knows then; use a "
               "step (dx_m / dy_m, angle_deg) or spawn the zone with its `motion`"),
    "expect_refused": ("an event the scenario expects the Shield to refuse "
                       "(expect_refused): the REST endpoint answers a refusal as an "
                       "error, with no `expected` to judge it by (the mission's own "
                       "--events does)"),
}


def rest_body_of_event(ev, *, zone_k: int | None = None,
                       last_zone: str | None = None,
                       origin: tuple[float, float] | None = None) -> dict:
    """The REST hot-apply body (guardrail/api.py) that makes a flying Shield
    apply ScenarioEvent `ev` as the rail's mission node applies the same event
    from its --events file (2026-11-14, WP4-03): guardrail.scenario_events.
    apply_event with shield_t=None, so the Shield stamps both with its own
    last tick. Route it with rest_route.

        spawn_polygon_fence {vertices, ...}      POST /hot_apply dynamic_nfz,
                                                 every field spawn_zone defaults
                                                 written out (P0, hard, repair,
                                                 mission, 0-1000 m, 1 m margin)
        translate_polygon_fence {dx_m, dy_m}     PATCH /dynamic_nfz/<id> translate
        rotate_polygon_fence {angle_deg}         PATCH /dynamic_nfz/<id> rotate
        scale_radius {factor}                    PATCH /dynamic_nfz/<id> scale
        activate_rule / deactivate_rule          POST /hot_apply time_window_switch
        swap_corridor                            POST /hot_apply corridor_swap

    Each builds the rule (or edits the ring) through the same Shield method
    the mission node's call does (spawn_nfz / hot_apply -> _spawn; translate_,
    rotate_, scale_nfz; set_rule_active and hot_apply -> _switch; hot_apply ->
    _swap), with the same ids: a switch with no window and no id gets
    Shield.set_rule_active's default, which the REST handler shares.

    `zone_k` and `last_zone` resolve what the payload leaves out, as the
    mission's EventState would: a spawn with no `id` is dyn-nfz-<k> (k spawns
    before it), an edit with no `id` edits the last zone spawned. None: the
    payload must name it (INVALID_ARGUMENT otherwise).

    WGS84 ONLY (WP1-14, 2026-11-26). A ScenarioEvent's absolute positions
    (a spawn's `vertices`, a swap's `centerline`) are written in local
    metres, the stress library's frame (or in {lat, lon}, passed as given);
    the REST endpoint takes {lat, lon} only. Metres are
    converted here, tool side, about `origin` - the frame origin of the
    policy the mission flies (Policy.frame_origin, what GET /policy states
    as `origin`) - with models.wgs84_from_metres, the conversion the
    endpoint applied at ingest until then, so the zone written is the same.
    An event with such points and no `origin` is FAILED_PRECONDITION.
    Relative edits (translate dx_m / dy_m, rotate, scale) stay in metres.

    Raises HarnessError UNIMPLEMENTED for a form with no REST body (a zone
    placed against the vehicle, a change of motion, an expected refusal:
    _NO_REST_FORM says why), INVALID_ARGUMENT for a payload neither the
    mission nor the endpoint could apply."""
    t, p = ev.type, dict(ev.payload or {})
    if p.get("expect_refused"):
        raise HarnessError("UNIMPLEMENTED", f"{t}: " + _NO_REST_FORM["expect_refused"])

    def zone_id(default: str | None, what: str) -> str:
        zid = p.get("id") or default
        if not isinstance(zid, str) or not zid.strip():
            raise HarnessError("INVALID_ARGUMENT", f"{t}: name the zone it {what} (`id`): "
                                                   f"nothing here says which it is")
        return zid

    def num(key: str, default=None) -> float:
        v = p.get(key, default)
        if v is None:
            raise HarnessError("INVALID_ARGUMENT", f"{t}: `{key}` is required")
        return _finite(v, f"{t} {key}")

    def point(v, what: str) -> dict:
        # {lat, lon} as given (WGS84), else {x, y} / [x, y] local metres.
        if isinstance(v, dict) and v.get("lat") is not None and v.get("lon") is not None:
            return {"lat": _finite(v["lat"], f"{t} {what} lat"),
                    "lon": _finite(v["lon"], f"{t} {what} lon")}
        x, y = (v["x"], v["y"]) if isinstance(v, dict) else (v[0], v[1])
        return {"x": _finite(x, f"{t} {what} x"), "y": _finite(y, f"{t} {what} y")}

    def latlon(pts: list[dict], what: str) -> list[dict]:
        if not any("x" in v for v in pts):
            return pts                                 # given in WGS84 already
        if origin is None:
            raise HarnessError("FAILED_PRECONDITION",
                               f"{t}: its {what} are local metres and the REST "
                               f"endpoint takes {{lat, lon}} only (WP1-14); no frame "
                               f"origin was given to convert them about (the policy's "
                               f"`origin`)")
        try:
            return wgs84_from_metres(pts, origin, what=f"{t} {what}")
        except ValueError as err:
            raise HarnessError("INVALID_ARGUMENT", f"{t}: {err}") from None

    if t == "spawn_polygon_fence":
        if "vertices" not in p:
            raise HarnessError("UNIMPLEMENTED", f"{t}: " + _NO_REST_FORM["vehicle"])
        verts = p["vertices"]
        if not isinstance(verts, list) or len(verts) < 3:
            raise HarnessError("INVALID_ARGUMENT", f"{t}: vertices must be a list of at "
                                                   f"least three {{x, y}} points")
        try:
            pts = [point(v, "vertex") for v in verts]
        except (KeyError, TypeError, IndexError):
            raise HarnessError("INVALID_ARGUMENT", f"{t}: every vertex is {{lat, lon}} or "
                                                   f"{{x, y}} in metres") from None
        body: dict[str, Any] = {
            "type": "dynamic_nfz",
            "id": zone_id(None if zone_k is None else f"dyn-nfz-{zone_k}", "spawns"),
            "vertices": latlon(pts, "vertices"),
            "priority": p.get("priority", "P0"),
            "constraint_type": p.get("constraint_type", "hard"),
            "violation_action": p.get("violation_action", "repair"),
            "scope": p.get("scope", "mission"),
            "altitude_floor_m": num("altitude_floor_m", 0.0),
            "altitude_ceiling_m": num("altitude_ceiling_m", 1000.0),
            "margin_m": num("margin_m", 1.0)}
        if p.get("motion") is not None:
            body["motion"] = dict(p["motion"])
        return body
    if t in ("translate_polygon_fence", "rotate_polygon_fence", "scale_radius"):
        if (t == "translate_polygon_fence" and "mps" in p) or \
                (t == "rotate_polygon_fence" and "deg_per_s" in p):
            raise HarnessError("UNIMPLEMENTED", f"{t}: " + _NO_REST_FORM["motion"])
        zid = zone_id(last_zone, "edits")
        if t == "translate_polygon_fence":
            return {"type": "dynamic_nfz", "op": "translate", "id": zid,
                    "dx": num("dx_m", 0.0), "dy": num("dy_m", 0.0)}
        if t == "rotate_polygon_fence":
            return {"type": "dynamic_nfz", "op": "rotate", "id": zid,
                    "angle_deg": num("angle_deg")}
        return {"type": "dynamic_nfz", "op": "scale", "id": zid, "factor": num("factor")}
    if t in ("activate_rule", "deactivate_rule"):
        active = t == "activate_rule"
        rid = p.get("rule_id")
        if not isinstance(rid, str) or not rid:
            raise HarnessError("INVALID_ARGUMENT", f"{t}: `rule_id` is required")
        body = {"type": "time_window_switch", **_EVENT_RULE_BASE, "target_id": rid,
                "active": active}
        win = p.get("window")
        if win is None:
            if p.get("id"):
                body["id"] = p["id"]
        else:
            body["id"] = p.get("id") or f"sw-{rid}-{'on' if active else 'off'}"
            body["valid_time"] = {"recurrence": dict(win)}
        return body
    if t == "swap_corridor":
        tgt = p.get("target_id")
        if not isinstance(tgt, str) or not tgt:
            raise HarnessError("INVALID_ARGUMENT", f"{t}: `target_id` is required")
        cl = [point(v, "centerline") for v in p.get("centerline") or []]
        body = {"type": "corridor_swap", **_EVENT_RULE_BASE, "id": p.get("id") or f"swap-{tgt}",
                "target_id": tgt, "centerline": latlon(cl, "centerline"),
                "width_m": num("width_m")}
        for k in ("altitude_floor_m", "altitude_ceiling_m"):
            if k in p:
                body[k] = num(k)
        if p.get("window") is not None:
            body["valid_time"] = {"recurrence": dict(p["window"])}
        return body
    raise HarnessError("INVALID_ARGUMENT", f"event type {t!r} has no REST hot-apply form")


def scenario_rest_timeline(spec, origin: tuple[float, float] | None = None) -> list[dict]:
    """A scenario's own events as the REST forwards a harness-driven rail
    episode makes (StartEpisode drive_scenario_events, 2026-11-14): in the
    order the mission applies them (by at_sim_t, ties as written: run_episode's
    and the node's stable sort), each with its body (rest_body_of_event) and
    route (rest_route). Each entry: {index, at_sim_t, type, payload, body,
    method, path}.

    `origin` is the frame origin the events' metre positions are converted
    about (rest_body_of_event); None reads it from the scenario's own policy
    (spec.policy), the policy the mission flies.

    Raises FAILED_PRECONDITION naming every event that has no REST form (the
    scenario then flies only with the mission's own --events), and
    INVALID_ARGUMENT when the scenario has no events to drive."""
    order = sorted(enumerate(spec.events), key=lambda x: x[1].at_sim_t)
    if origin is None and any(_event_has_points(ev) for ev in spec.events):
        origin = _policy_origin(spec)
    k, last = 0, None
    out, problems = [], []
    for i, ev in order:
        try:
            body = rest_body_of_event(ev, zone_k=k, last_zone=last, origin=origin)
        except HarnessError as err:
            problems.append(f"#{i} {ev.type} at {ev.at_sim_t:g} s ({err.code}: {err.message})")
            body = None
        if ev.type == "spawn_polygon_fence":
            last = (body or {}).get("id") or ev.payload.get("id") or f"dyn-nfz-{k}"
            k += 1
        if body is not None:
            method, path, _ = rest_route(body)
            out.append({"index": i, "at_sim_t": float(ev.at_sim_t), "type": ev.type,
                        "payload": ev.model_dump(mode="json")["payload"], "body": body,
                        "method": method, "path": path})
    if problems:
        raise HarnessError("FAILED_PRECONDITION",
                           f"the harness cannot drive {spec.scenario_id}'s own events over "
                           f"REST: " + "; ".join(problems) + " (fly it with the mission's "
                           f"own --events: drive_scenario_events off)")
    if not out:
        raise HarnessError("INVALID_ARGUMENT", f"{spec.scenario_id} has no events of its own "
                                               f"for the harness to drive")
    return out


def _event_has_points(ev) -> bool:
    """Does ScenarioEvent `ev` give absolute positions (a spawn's vertices, a
    swap's centerline), which a REST body carries as lat/lon?"""
    p = ev.payload or {}
    return ((ev.type == "spawn_polygon_fence" and "vertices" in p)
            or (ev.type == "swap_corridor" and bool(p.get("centerline"))))


def _policy_origin(spec) -> tuple[float, float] | None:
    """The frame origin of the policy scenario `spec` flies (None: it has
    none, or it does not load)."""
    try:
        return spec.policy.load(ROOT).frame_origin
    except Exception:                                       # noqa: BLE001
        return None


def flown_start_policy_hash(manifest: dict | None, held: dict) -> str | None:
    """The policy a rail mission STARTED with, to check against the episode's. The
    run's policy_g0.json is that policy byte for byte (the Shield node writes it at
    its start); manifest.json's policy_hash names the policy the mission ENDED with,
    which for a mission whose zones were edited by hot-apply is a later generation
    (hh01_001, 14 Nov: g0 sha256:b665..., the manifest's sha256:cd8a... = g2 after
    a spawn and a rotate). The manifest is read only when no g0 file came back."""
    if 0 in held:
        return "sha256:" + hashlib.sha256(held[0]).hexdigest()
    return (manifest or {}).get("policy_hash")


def rail_hotapply_args(endpoint: str, rig) -> list[str]:
    """The Shield node's hot-apply listener flags for a rail session's
    `endpoint` ([] when it names none: the mission then takes no REST event).
    Off loopback the node serves mutual TLS from the rig's PKI folder, as the
    node sees it (RailRig.hotapply_pki, hotapply_server_cert,
    hotapply_clients). Harness._rail_hot_args and the dry run of
    tools/harness_hil_episode.py both use it."""
    if not endpoint:
        return []
    u = urllib.parse.urlsplit(endpoint)
    local = is_loopback(u.hostname)
    args = [f"--hotapply-port={u.port}"]
    host = rig.hotapply_host or ("" if local else u.hostname)
    if host:
        args.append(f"--hotapply-host={host}")
    if not local or u.scheme == "https":
        if not rig.hotapply_pki:
            raise HarnessError("FAILED_PRECONDITION",
                               f"rest_endpoint {endpoint} needs mutual TLS on the mission's "
                               f"listener, and this server's rig names no PKI folder for "
                               f"it (serve --rail-hotapply-pki)")
        args.append(f"--hotapply-pki={rig.hotapply_pki}")
        if rig.hotapply_server_cert:
            args.append(f"--hotapply-server-cert={rig.hotapply_server_cert}")
        args += [f"--hotapply-client={cn}" for cn in rig.hotapply_clients]
    return args


@dataclass
class RailRig:
    """The rig a server flies rail sessions on (`Harness(rail=...)`, `serve
    --rail`): nothing of its own, only where experiments/sweep_scenarios.py's
    rail backends are pointed.

    `hil`: the sweep's run_hil through its transport (`transport(HilConfig)`,
    default the sweep's make_hil_transport: SITL and the router in WSL, the
    mission container on the Orin over ssh, the run folder fetched into
    `store`). `sitl`: the sweep's run_sitl (`sitl_runner`, default it), whose
    run folder is `sitl_out`/<tag> (default the checkout's demo/out). Tests
    hand in the fakes tests/test_sweep.py flies its hil campaign on."""
    backends: tuple = ("hil",)
    hil_config: Any = None           # sweep HilConfig; None: its defaults
    transport: Any = None            # factory(HilConfig) -> transport
    sitl_runner: Any = None          # callable like sweep run_sitl
    sitl_out: str | Path | None = None
    # Fetched hil run folders; None: the hil config's `store`, else
    # <runs>/rail_runs.
    store: str | Path | None = None
    campaign: str = ""               # prefixes every mission tag
    controls: bool = False           # also fly Shield-off control arms
    # The mission's hot-apply listener, when a session names an endpoint: the
    # address it binds ("": the endpoint's host), and for an endpoint off
    # loopback the lab PKI folder AS THE NODE SEES IT, its server
    # certificate's name ("": harness-server) and the client names it admits
    # (empty: the node's default, harness-client).
    hotapply_host: str = ""
    hotapply_pki: str = ""
    hotapply_server_cert: str = ""
    hotapply_clients: tuple = ()


@dataclass
class _RailFlight:
    """A rail episode's flight: what it was launched with, the hot-applies
    waiting for their mission time, and every forward made into it."""
    spec: Any                        # the ScenarioSpec the rail flies
    hot_args: list                   # the node's hot-apply listener flags ([]: none)
    endpoint: str = ""
    tag: str = ""
    phase: str = "launching"
    # Who applies the scenario's own events (2026-11-14): "mission" (its
    # --events file, as a campaign flies it) or "harness" (taken off the
    # mission and forwarded over REST at their mission times; `timeline` is
    # scenario_rest_timeline's list, `spec` then has no events).
    events_via: str = "mission"
    timeline: list = field(default_factory=list)
    # (at | None, index, body, source, extra), unsent; source: episode (a
    # HotApply), emit_event, scenario.
    scheduled: list = field(default_factory=list)
    forwarded: list = field(default_factory=list)   # records, in the order sent
    not_sent: list = field(default_factory=list)
    t0: float | None = None          # time.monotonic() at mission time 0
    t0_wall: float | None = None
    endpoint_policy_hash: str = ""
    endpoint_problem: str = ""
    max_s: float = 0.0               # the rail's own time cap (rail_max_s)
    over: threading.Event = field(default_factory=threading.Event)
    cond: threading.Condition = field(default_factory=threading.Condition)
    driver: Any = None


def alias_tree(src: str | Path, dst: str | Path) -> dict:
    """`src`'s files under `dst`, by hard link where the filesystem allows
    (same volume: no second copy of a bag), else copied. Returns {"how":
    "hard links" | "copies" | "hard links and copies", "files": [{"path",
    "bytes", "sha256"}]}, the digests read from `dst` so the alias is
    checkable against the run folder."""
    import shutil
    src, dst = Path(src), Path(dst)
    linked = copied = 0
    files = []
    for f in sorted(p for p in src.rglob("*") if p.is_file()):
        out = dst / f.relative_to(src)
        out.parent.mkdir(parents=True, exist_ok=True)
        try:
            os.link(f, out)
            linked += 1
        except OSError:
            shutil.copy2(f, out)
            copied += 1
        h = hashlib.sha256()
        with open(out, "rb") as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b""):
                h.update(chunk)
        files.append({"path": f.relative_to(src).as_posix(), "bytes": out.stat().st_size,
                      "sha256": "sha256:" + h.hexdigest()})
    how = ("hard links" if linked and not copied else "copies" if copied and not linked
           else "hard links and copies" if linked else "nothing")
    return {"how": how, "files": files}


# --------------------------------------------------------------------------- #
# The grant's episode-bundle files (Stress Testing, "Episode bundle")
# --------------------------------------------------------------------------- #

POLICY_BUNDLE_FILE = "policy_bundle.tar.gz"
POLICY_BUNDLE_INDEX = "policy_bundle.json"
POLICY_BUNDLE_KIND = "vla-guardrail-episode-policy"
# Why the headless backend writes no episode.bag2/ (kpi.json harness.bundle_files).
NO_EPISODE_BAG = ("not written: the headless backend runs no ROS 2 graph, so there is "
                  "nothing for rosbag2 to record (a ROS 2 rail run records its bag in "
                  "its run folder, bag/)")


REPLAY_INPUT_FILES = ("metrics.json", "trajectory.json")


def bundle_replay_inputs(run_dir: str | Path, ep_dir: str | Path) -> dict:
    """What a replay of a rail episode reads beyond the grant's six names,
    put beside them so the bundle alone replays (WP4-11,
    tools/replay_at_revision.py; until 2026-11-18 a rail session's bundle
    named its run folder instead): metrics.json (the Shield's forecast, arm
    and subject), trajectory.json (the position's own doubles, which
    flight_log.jsonl rounds to 1 mm) and the run's signed
    `<tag>.replay.tar.gz`, each byte for byte (the archive as a hard link
    when the two folders share a volume). Returns {name: what was done}; a
    file the run lacks is said, not made up."""
    import shutil
    src, dst = Path(run_dir), Path(ep_dir)
    out: dict[str, str] = {}
    for name in REPLAY_INPUT_FILES:
        if (src / name).is_file():
            shutil.copyfile(src / name, dst / name)
            out[name] = "written: the run's, byte for byte"
        else:
            out[name] = f"not written: the run folder {src.name} has no {name}"
    tars = sorted(src.glob("*.replay.tar.gz"))
    if tars:
        t = tars[0]
        how = link_or_copy(t, dst / t.name)
        out[t.name] = f"written: the run's signed replay bundle, as {how}"
    else:
        out["<tag>.replay.tar.gz"] = (f"not written: the run folder {src.name} has no "
                                      f"replay bundle")
    return out


def link_or_copy(src: str | Path, dst: str | Path) -> str:
    """`src` at `dst` byte for byte: a hard link where the filesystem allows
    (same volume: no second copy of a flight's largest files), else a copy.
    An existing `dst` is replaced. Returns "a hard link" or "a copy"."""
    import shutil
    src, dst = Path(src), Path(dst)
    if dst.exists():
        dst.unlink()
    try:
        os.link(src, dst)
        return "a hard link"
    except OSError:
        shutil.copyfile(src, dst)
        return "a copy"


def read_held_generations(run_dir: str | Path) -> dict[int, bytes]:
    """{generation: bytes} of every policy_g<N>.json a rail run folder kept:
    the canonical IR the Shield node wrote at each generation it flew."""
    held: dict[int, bytes] = {}
    for f in sorted(Path(run_dir).glob("policy_g*.json")):
        tail = f.stem[len("policy_g"):]
        if tail.isdigit():
            held[int(tail)] = f.read_bytes()
    return held


def rail_bag_and_audit(run_dir: str | Path, ep_dir: str | Path, *,
                       link_audit: bool = False) -> tuple[dict, dict | None]:
    """Two of the grant's six bundle names from a rail run folder (Stress
    Testing, "Episode bundle"): episode.bag2/ (the run's rosbag2 directory,
    which the rail records as bag/ and keeps under that name, aliased by
    alias_tree) and shield_audit.jsonl (the run's, byte for byte: a copy, or
    with `link_audit` a hard link where the volume allows). Returns
    ({name: what was done, or why not}, alias_tree's record of the bag or
    None). Shared by the gRPC Harness's rail session and the stress sweep's
    rail campaigns (experiments/sweep_scenarios.write_rail_episode_bundle)."""
    import shutil
    run_dir, ep_dir = Path(run_dir), Path(ep_dir)
    files: dict[str, str] = {}
    bag_info = None
    bag = run_dir / "bag"
    if bag.is_dir() and any(p.is_file() for p in bag.rglob("*")):
        bag_info = alias_tree(bag, ep_dir / EPISODE_BAG_DIR)
        bag_info["source"] = bag.as_posix()
        files[EPISODE_BAG_DIR + "/"] = (
            f"written: the run's rosbag2 directory (recorded as bag/, which the run "
            f"folder keeps) under the grant's name, {len(bag_info['files'])} file(s) "
            f"as {bag_info['how']}")
    else:
        files[EPISODE_BAG_DIR + "/"] = (
            f"not written: the run folder {run_dir.name} has no bag/ (the mission "
            f"recorded none: VLAGUARD_BAG=0, or rosbag2 did not start)")
    audit = next((run_dir / n for n in ("shield_audit.jsonl", "audit.jsonl")
                  if (run_dir / n).is_file()), None)
    if audit is None:
        files["shield_audit.jsonl"] = (f"not written: the run folder {run_dir.name} "
                                       f"has no shield_audit.jsonl")
    elif link_audit:
        how = link_or_copy(audit, ep_dir / "shield_audit.jsonl")
        files["shield_audit.jsonl"] = (f"written: the run's {audit.name}, byte for byte, "
                                       f"as {how}")
    else:
        shutil.copyfile(audit, ep_dir / "shield_audit.jsonl")
        files["shield_audit.jsonl"] = f"written: the run's {audit.name}, byte for byte"
    return files, bag_info


def _blob_is(blob: bytes, recorded: str) -> bool:
    """Is `blob` the canonical IR the hash `recorded` names? Its SHA-256, or
    (an older run's shortened hash) the policy it parses to matching it.
    The blob is a recorded generation, read in the form it was recorded in
    (an older run's may hold local metres; WP1-14)."""
    if "sha256:" + hashlib.sha256(blob).hexdigest() == recorded:
        return True
    try:
        from .models import legacy_metre_policy_from_raw
        return bool(legacy_metre_policy_from_raw(json.loads(blob), source="generation",
                                                 runtime=False).matches_hash(recorded))
    except Exception:                                            # noqa: BLE001
        return False


def write_policy_bundle(ep_dir: str | Path, source: str | Path, start_policy,
                        final_policy, generations: list[dict],
                        held: dict | None = None) -> dict:
    """`policy_bundle.tar.gz` in `ep_dir`: "the exact bundle that was active
    (incl. all hot-apply generations)" (Stress Testing, "Episode bundle").
    Returns its index.

    Members (gzip and tar times 0, so the same episode gives the same bytes):

      injected/<file>      the policy file the episode was given, byte for
                           byte: the signed bundle, an uploaded archive or
                           the YAML (`source`);
      generations/policy_g<N>.json
                           the canonical IR of each generation this function
                           holds a policy for: `start_policy` (generation 0,
                           as injected) and `final_policy` (the flying
                           Shield's at the end), each only where its
                           generation number AND hash are the ones the
                           episode recorded for that generation; and, since
                           2026-11-10, `held`: {generation: canonical IR
                           bytes} of every generation the run kept (the
                           headless runner's EpisodeRun.generations, a rail
                           run's policy_g<N>.json files), each a member only
                           when its bytes hash to the recorded hash;
      policy_bundle.json   the index: every generation the episode flew
                           (`generations`, harness events: t, event, rule,
                           policy_hash) with its IR member or null, the
                           injected file's SHA-256, every member's SHA-256,
                           and `complete` - false, with `missing` saying
                           which, when a generation's IR is not a member.

    Until 2026-11-10 nothing held a generation between the first and the
    last (the headless runner's hot_apply events recorded its hash only), so
    with two or more hot-applies the ones between were in the index by hash
    alone. With `held` they are members; a generation nothing holds the IR of
    is still named by its hash, and the index says so rather than leave the
    gap unmarked."""
    import gzip
    import io
    import tarfile
    d = Path(ep_dir)
    d.mkdir(parents=True, exist_ok=True)
    members: dict[str, bytes] = {}
    src = Path(source)
    injected = None
    if src.is_file():
        injected = "injected/" + src.name
        members[injected] = src.read_bytes()
    gens = []
    for g in generations:
        n, h = g.get("generation"), g.get("policy_hash")
        ir = None
        blob = (held or {}).get(n)
        if blob is not None and h and _blob_is(bytes(blob), h):
            ir = f"generations/policy_g{n}.json"
            members[ir] = bytes(blob)
        for pol in (() if ir else (start_policy, final_policy)):
            if (pol is not None and h and n == getattr(pol, "generation", None)
                    and pol.matches_hash(h)):
                ir = f"generations/policy_g{n}.json"
                members[ir] = pol.canonical_bytes()
                break
        gens.append({"generation": n, "policy_hash": h, "t": g.get("t"),
                     "event": g.get("event"), "rule_id": g.get("rule_id"), "ir": ir})
    missing = [g["generation"] for g in gens if g["ir"] is None]
    why = []
    if injected is None:
        why.append(f"the injected policy file {str(source)!r} is no longer on disk")
    if missing:
        why.append(f"generation(s) {missing}: no IR held for it that hashes to the "
                   f"recorded policy_hash; named by that hash")
    index = {"kind": POLICY_BUNDLE_KIND, "version": 1,
             "injected": {"source": str(source), "member": injected,
                          "sha256": ("sha256:" + hashlib.sha256(members[injected]).hexdigest()
                                     if injected else None)},
             "generations": gens,
             "complete": not why, "missing": "; ".join(why) or None,
             "members": {k: "sha256:" + hashlib.sha256(v).hexdigest()
                         for k, v in sorted(members.items())}}
    members[POLICY_BUNDLE_INDEX] = (json.dumps(_json_safe(index), indent=2, sort_keys=True)
                                    + "\n").encode()
    with open(d / POLICY_BUNDLE_FILE, "wb") as fh:
        with gzip.GzipFile(fileobj=fh, mode="wb", mtime=0, filename="") as gz:
            with tarfile.open(fileobj=gz, mode="w") as tar:
                for name, data in sorted(members.items()):
                    info = tarfile.TarInfo(name=name)
                    info.size = len(data)
                    info.mtime = 0
                    tar.addfile(info, io.BytesIO(data))
    return index


class Harness:
    """Episodes armed, fed events and flown; no gRPC in here (the servicer
    below is a translation of this onto the stubs, and tests drive both)."""

    def __init__(self, runs_dir: str | Path | None = None,
                 library: str | Path | None = None,
                 manifest_base: dict | None = None,
                 armed_idle_ttl_s: float | None = None,
                 rest_hosts=(), session_idle_ttl_s: float | None = None,
                 rest_pki_dir: str | Path | None = None,
                 rest_client_name: str = DEFAULT_CLIENT_CN,
                 paraphraser=None, rail: "RailRig | None" = None):
        # The paraphraser service every paraphrase arm is drawn from
        # (2026-11-07, WP4-10; module docstring, PARAPHRASE ARMS): a
        # guardrail.paraphraser_service.ParaphraserClient, or anything whose
        # `paraphrase(text, n, seed, backend)` answers its dicts. None: this
        # server draws no paraphrase arm (the canonical arm draws none).
        self.paraphraser = paraphraser
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
        # The identity this server presents to an https REST endpoint: the lab
        # CA to trust it by, and a client certificate for its common-name
        # allow-list (guardrail/api.py serves plaintext on loopback only and
        # mutual TLS everywhere else). Until 2026-10-19 the forward had none,
        # so an https endpoint - every rail's off-loopback hot-apply port -
        # failed every call as UNAVAILABLE (CERTIFICATE_VERIFY_FAILED, or a
        # dropped cert-less client), a code that tells a client to retry.
        self.rest_client_name = None
        self._rest_tls = None
        if rest_pki_dir is not None:
            import ssl
            ca, cert, key = pki_paths(rest_pki_dir, rest_client_name)
            for f in (ca, cert, key):
                if not f.is_file():
                    raise FileNotFoundError(
                        f"{f} is missing: the REST client identity is written by "
                        f"gen-pki (or issue --name {rest_client_name})")
            ctx = ssl.create_default_context(cafile=str(ca))
            ctx.load_cert_chain(str(cert), str(key))
            self._rest_tls, self.rest_client_name = ctx, rest_client_name
        self._lock = threading.Lock()
        self._run_lock = threading.Lock()
        # The rig this server flies rail episodes on (RailRig; None: it flies
        # none, and a sitl / hil session is UNIMPLEMENTED here). One mission
        # at a time: the rig is one aircraft.
        self.rail = rail
        self._rail_busy = threading.Lock()
        self._rail_k = 0
        self._rail_campaign = ((rail.campaign if rail is not None else "")
                               or "h" + _dt.datetime.now(_dt.timezone.utc).strftime(
                                   "%m%dT%H%M"))
        if rail is not None:
            bad = sorted(set(rail.backends) - set(RAIL_BACKENDS))
            if bad or not rail.backends:
                raise ValueError(f"RailRig.backends {rail.backends!r}: some of "
                                 f"{RAIL_BACKENDS}")
            if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,23}", self._rail_campaign):
                raise ValueError(f"RailRig.campaign {self._rail_campaign!r} must be a "
                                 f"short plain name (it prefixes every mission tag)")

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
                      pace: float | None = None,
                      drive_scenario_events: bool = False) -> dict:
        if bool(scenario_id) == bool(scenario_yaml):
            raise HarnessError("INVALID_ARGUMENT",
                               "give exactly one of scenario_id or scenario_yaml")
        sess = self._open_session(session_id) if session_id else None
        rail = sess is not None and sess.backend in RAIL_BACKENDS
        if rail and (live or pace is not None):
            raise HarnessError("INVALID_ARGUMENT",
                               f"a {sess.backend} rail episode flies in real time on the "
                               f"rig: `live` and `pace` are the headless backend's")
        if drive_scenario_events and not rail:
            raise HarnessError("INVALID_ARGUMENT",
                               "drive_scenario_events is a rail episode's (a sitl or hil "
                               "session): the headless runner applies a scenario's own "
                               "events itself")
        if live:
            pace = DEFAULT_PACE if pace is None else _finite(pace, "pace")
            if pace < 0:
                raise HarnessError("INVALID_ARGUMENT", f"pace {pace} must be >= 0 (0: unpaced)")
        elif pace is not None:
            raise HarnessError("INVALID_ARGUMENT", "pace applies to a live episode only")
        r = self._resolve(scenario_id=scenario_id, scenario_yaml=scenario_yaml,
                          cell_id=cell_id, seed=seed, profile=profile, arm=arm, sess=sess)
        prof, lib, spec, ep, t_end = r.prof, r.lib, r.spec, r.ep, r.t_end
        # A session's injected policy replaces the scenario's (InjectPolicy).
        policy_ref = sess.policy["ref"] if sess is not None and sess.policy else None
        try:
            policy, _source = (policy_ref or spec.policy).load_with_source(ROOT)
        except FileNotFoundError as e:
            raise HarnessError("NOT_FOUND", f"policy: {e}") from None
        except (ValidationError, ValueError, OSError) as e:
            raise HarnessError("INVALID_ARGUMENT",
                               f"policy {(policy_ref or spec.policy).source!r}: {e}") from None
        arm_record = self._arm_record(ep, policy if policy_ref is not None else None,
                                      defaults=lib.defaults)
        flight = (self._rail_prepare(sess, spec, ep, lib.defaults,
                                     drive=bool(drive_scenario_events)) if rail else None)

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
                              "pace": pace,
                              "drive_scenario_events": bool(drive_scenario_events)},
                     client=client, dry=None, arm_record=arm_record,
                     session_id=session_id, policy_ref=policy_ref,
                     task=dict(sess.task) if sess is not None and sess.task else None,
                     live=bool(live), pace=float(pace) if live else DEFAULT_PACE)
        return self._open_episode(e, sess, rail=rail, flight=flight, live=bool(live))

    def _resolve(self, *, scenario_id: str, scenario_yaml: str, cell_id: str,
                 seed: int | None, profile: str, arm: str, sess) -> SimpleNamespace:
        """StartEpisode's request resolved to what flies: the profile, the
        library, the cell, its spec, the seed, the arm (scenario_spec.Episode)
        and the run's last tick, with every refusal start_episode gives about
        them. Shared with plan_rail_episode, so a dry run resolves exactly what
        StartEpisode would."""
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

        # The sweep's own arms of this cell and seed (scenario_spec.
        # arm_episodes): one per prefix and paraphrase arm and, under a
        # prefix-consumer profile (2026-11-16), each with its consumer and
        # the A/A repeat of a cell the profile names ("prefix-on,aa1").
        try:
            arms = arm_episodes(cell, s, prof)
        except ValueError as e:
            raise HarnessError("INVALID_ARGUMENT", str(e)) from None
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
        return SimpleNamespace(prof=prof, lib=lib, fam=fam, cell=cell, spec=spec, ep=ep,
                               t_end=t_end)

    def _open_episode(self, e: _Episode, sess, *, rail: bool, flight, live: bool) -> dict:
        """start_episode's second half: the episode set up (a live runner, a
        rail flight and the scenario events it drives), counted, registered
        and started. Nothing is held or launched when it raises."""
        eid, ep, arm_record = e.episode_id, e.ep, e.arm_record
        e.dry = self._dry_shield(e, [])[0]
        if rail:
            e.backend, e.rail = sess.backend, flight
            e.state = "RUNNING"          # launched below, once the rig is taken
            # A harness-driven scenario (drive_scenario_events): its own
            # events are queued as forwards at their mission times, before
            # the mission is launched, so none can be missed.
            for item in flight.timeline:
                idx = len(e.queued)
                e.queued.append(_Queued(
                    index=idx, source="scenario",
                    event=SimpleNamespace(at_sim_t=item["at_sim_t"], type=item["type"]),
                    rule_id=str(item["body"].get("id") or item["body"].get("target_id") or ""),
                    received={"rpc": "StartEpisode", "drive_scenario_events": True,
                              "at_sim_t": item["at_sim_t"],
                              "scenario_event": {"index": item["index"], "type": item["type"],
                                                 "payload": item["payload"]},
                              "body": item["body"]}))
                flight.scheduled.append((item["at_sim_t"], idx, dict(item["body"]), "scenario",
                                         {"scenario_event": item["type"],
                                          "scenario_event_index": item["index"]}))
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
            # The rig is taken last, in the step that creates the episode, so
            # no refusal above leaves it held; _rail_loop releases it.
            if rail and not self._rail_busy.acquire(blocking=False):
                raise HarnessError("RESOURCE_EXHAUSTED",
                                   "the rig is flying another episode's mission: one at "
                                   "a time (EndEpisode that one first)")
            self._episodes[eid] = e
            if sess is not None:
                sess.episodes.append(eid)
        if sess is not None:
            self._emit(sess, "episode_started", eid, {**self._ref(e), "task": (
                {k: v for k, v in e.task.items() if k != "text"} if e.task else None)})
        if live:
            threading.Thread(target=self._live_loop, args=(e,), daemon=True,
                             name=f"live-{eid}").start()
        if rail:
            threading.Thread(target=self._rail_loop, args=(e,), daemon=True,
                             name=f"rail-{eid}").start()
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

    def _arm_record(self, ep, policy=None, defaults=None) -> dict | None:
        """The episode's arm record (its paraphrase draw and prefix key), built
        now rather than at EndEpisode, so a scenario this server cannot draw
        an arm for is refused before an episode exists. A paraphrase arm is
        drawn by the paraphraser service this server is bound to
        (`_service_draw`, since 2026-11-07), never in process; what the
        service cannot draw - a `paraphrase: {backend: llm:<id>}` it has no
        client for, a `stored` source it holds no set of - is
        FAILED_PRECONDITION with the service's own words, as it was when the
        draw was in process. The draw is a function of (cell, seed, arm), so
        building it early changes nothing that flies.

        A prefix-consumer episode's record (2026-11-16: the sweep's
        `pilot_reads_prefix`, `consumer`, `repeat`) is the sweep's too, with
        the CSP its pilot flies compiled here for `defaults` (the library's);
        a scenario that pilot cannot fly (mid-flight events, a policy that is
        not static) is INVALID_ARGUMENT, before an episode exists."""
        drawn = None
        if ep.paraphrase_arm is not None and ep.paraphrase_arm >= 0:
            drawn = self._service_draw(ep, policy)
        try:
            with self._run_lock:                 # the sweep's arm cache is module state
                return self._S.build_arm(ep, policy, drawn=drawn, defaults=defaults)
        except OSError as err:
            raise HarnessError("FAILED_PRECONDITION",
                               f"arm {ep.arm_tag}: {type(err).__name__}: {err}") from None
        except ValueError as err:
            raise HarnessError("INVALID_ARGUMENT", f"arm {ep.arm_tag}: {err}") from None

    def _service_draw(self, ep, policy=None) -> "ServiceParaphrase":
        """Paraphrase arm `ep.paraphrase_arm` of this episode's seed, drawn by
        the paraphraser service (module docstring, PARAPHRASE ARMS). The text
        sent is what the arm rewords (the sweep's instruction_text: the task
        prompt or the rule summary); AerialVLA's protected prompt is refused
        here first, as the scenario's mistake (INVALID_ARGUMENT), before any
        call; an unbound server, or a service that does not answer, draws
        nothing."""
        spec = ep.cell.spec
        pol = policy if policy is not None else spec.policy.load(ROOT)
        text, _ = self._S.instruction_text(spec, pol)
        from .paraphraser import protected_reason   # a check on the text: no draw
        why = protected_reason(text)
        if why:
            raise HarnessError("INVALID_ARGUMENT", f"arm {ep.arm_tag}: the instruction "
                                                   f"cannot be paraphrased: {why}")
        client = self.paraphraser
        if client is None:
            raise HarnessError(
                "FAILED_PRECONDITION",
                f"arm {ep.arm_tag}: paraphrase arms are drawn by the paraphraser service "
                f"(the reference orchestrator's), and this server is bound to none: "
                f"serve --paraphraser HOST:PORT; the canonical arm draws none")
        where = getattr(client, "address", type(client).__name__)
        try:
            reply = client.paraphrase(text, int(spec.paraphrase.k), int(ep.seed),
                                      spec.paraphrase.backend)
        except Exception as err:                 # noqa: BLE001 - mapped, never swallowed
            code = err.code() if callable(getattr(err, "code", None)) else None
            name = getattr(code, "name", None)
            details = (err.details() if callable(getattr(err, "details", None))
                       else str(err)) or ""
            to = {"NOT_FOUND": "FAILED_PRECONDITION", "FAILED_PRECONDITION":
                  "FAILED_PRECONDITION", "OUT_OF_RANGE": "INVALID_ARGUMENT",
                  "INVALID_ARGUMENT": "INVALID_ARGUMENT", "UNAVAILABLE": "UNAVAILABLE",
                  "DEADLINE_EXCEEDED": "UNAVAILABLE"}.get(name, "FAILED_PRECONDITION")
            raise HarnessError(to, f"arm {ep.arm_tag}: the paraphraser service at {where} "
                                   f"did not draw it ({name or type(err).__name__}: "
                                   f"{details}); the canonical arm draws none") from None
        drawn = reply.get("paraphrases") or []
        if ep.paraphrase_arm >= len(drawn):
            raise HarnessError("INVALID_ARGUMENT",
                               f"arm {ep.arm_tag}: the paraphraser service drew "
                               f"{len(drawn)} paraphrase(s) of k={spec.paraphrase.k}")
        rec = dict(drawn[ep.paraphrase_arm])
        if rec.get("seed") != int(ep.seed):
            raise HarnessError("INTERNAL", f"arm {ep.arm_tag}: the service answered seed "
                                           f"{rec.get('seed')} for seed {ep.seed}")
        via = {"service": where,
               "transport": ("mutual TLS" if getattr(client, "secure", False)
                             else "plaintext (loopback)"),
               "validator_digest": reply.get("validator_digest"),
               "store_digest": reply.get("store_digest"),
               "source_id": reply.get("source_id")}
        return ServiceParaphrase(record=rec, via=via)

    def _ref(self, e: _Episode) -> dict:
        return {"episode_id": e.episode_id, "scenario_id": e.ep.cell.scenario_id,
                "cell_id": e.ep.episode_id, "run_id": e.ep.run_id, "seed": e.ep.seed,
                "arm": e.ep.arm_tag, "profile": e.profile,
                "run_end_sim_t": round(e.t_end, 6), "policy_hash": e.policy_hash,
                "session_id": e.session_id, "live": e.live, "backend": e.backend,
                "scenario_events_via": e.rail.events_via if e.rail is not None else ""}

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

    @staticmethod
    def _consumer_takes_no_events(e: _Episode, rpc: str) -> None:
        """Refuse a mid-flight rule change (HotApply, EmitEvent, a session
        HotApply forwarded while it flies) to an episode whose pilot consumes
        the CSP (2026-11-16, profile `prefix_consumer`), before anything is
        queued, forwarded or acked. Since 2026-11-26 (audit card WP4-19)
        generation N's CSP follows the SCENARIO's own events to the pilot
        (headless at the event; on a rail with --csp-topic, which only such
        a scenario is sent): a change injected here is one the scenario does
        not declare. This refusal is what keeps it out of such an episode:
        once applied, a rail's Shield node would publish that generation to
        the pilot like any other (EpisodeRecord.on_generation fires for every
        generation, a REST hot-apply's too). A rail flight the stress harness
        scores is also refused afterwards, on either arm, when a generation
        above 0 was made by no scenario_event of the run
        (experiments/sweep_scenarios.py prefix_generation_problems). Both
        arms are refused alike, so the pair stays one scenario."""
        if getattr(e.ep, "prefix_consumer", None):
            raise HarnessError(
                "FAILED_PRECONDITION",
                f"{rpc} refused: episode {e.episode_id} is arm {e.ep.arm_tag} of a pilot "
                f"that consumes the CSP (guardrail.prefix_consumer, profile "
                f"{e.profile or '?'}); generation N's CSP reaches that pilot only for the "
                f"scenario's own declared events (audit card WP4-19), so its rules cannot "
                f"be changed in flight from here")

    def hot_apply(self, episode_id: str = "", at_sim_t: float = 0.0,
                  body: dict | None = None, *, session_id: str = "",
                  now: bool = False) -> dict:
        """A REST hot-apply body for an episode (scheduled or live), or, with
        `session_id` instead, forwarded to the session's REST endpoint. An
        episode of a pilot that consumes the CSP takes none
        (_consumer_takes_no_events)."""
        if session_id and episode_id:
            raise HarnessError("INVALID_ARGUMENT", "address a HotApply to an episode_id or "
                                                   "to a session_id, not both")
        if session_id:
            return self._forward_hot_apply(session_id, body)
        e = self._get(episode_id)
        self._consumer_takes_no_events(e, "HotApply")
        if e.rail is not None:
            return self._rail_hot_apply(e, at_sim_t, body, now)
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
        self._consumer_takes_no_events(e, "EmitEvent")
        if e.rail is not None:
            return self._rail_emit_event(e, at_sim_t, type_, payload, now)
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
            if e.rail is not None:
                out["rail_phase"] = e.rail.phase
            return out

    def end_episode(self, episode_id: str) -> dict:
        """Fly, score and bundle the episode. Idempotent: an ENDED episode
        returns its bundle again, and a call that finds another call flying
        it waits for that flight and returns the same bundle (or its
        failure) - a client that retries after its deadline expired while it
        queued gets the episode, not an error. A live episode is let finish
        unpaced, and its bundle returned."""
        e = self._get(episode_id)
        s = self._flight_session(e)          # before e.lock: self._lock is never taken under it
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
                if s is not None:
                    # Counted in the step that makes it RUNNING: a StopSession
                    # either finds it ARMED (and fails it) or waits for it.
                    with s.cond:
                        s.flying += 1
        if flying_elsewhere:
            e.done.wait()            # set by _finish, whatever the flight's outcome
            with e.lock:
                e.waiters -= 1
                if e.state == "ENDED":
                    return e.result
                raise HarnessError("FAILED_PRECONDITION",
                                   f"episode {episode_id} failed: {e.error}")
        try:
            return self._fly_scheduled(e, queued)
        finally:
            self._flight_over(s)

    def _fly_scheduled(self, e: _Episode, queued: list[_Queued]) -> dict:
        """end_episode's own flight: fly, close the episode, and append its
        rows, run events and end (or its failure) to its session."""
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
        """A live episode's thread: _fly_live, counted as a flight of its
        session. Counted before its first RUNNING check, so a StopSession
        that has not failed it yet waits for every frame it appends (one is
        appended outside e.lock) before `session_stopped`."""
        s = self._flight_session(e)
        if s is not None:
            with s.cond:
                s.flying += 1
        try:
            self._fly_live(e)
        finally:
            self._flight_over(s)

    def _fly_live(self, e: _Episode) -> None:
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

    def _flight_session(self, e: _Episode) -> _Session | None:
        """The session an episode's flight appends to (None: it has none)."""
        if not e.session_id:
            return None
        with self._lock:
            return self._sessions.get(e.session_id)

    @staticmethod
    def _flight_over(s: _Session | None) -> None:
        """A flight of `s` has appended all it will: StopSession may close."""
        if s is not None:
            with s.cond:
                s.flying -= 1
                s.cond.notify_all()

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
        # Emitted by stop_session only if THIS call stops it: two sweeps that
        # both found it idle wrote session_expired (and session_stopped) twice.
        self.stop_session(s.session_id, first=("session_expired", {
            "idle_s": round(now - s.idle_since, 3),
            "why": f"no call on the session for {self.session_idle_ttl_s:g} s (the "
                   f"server's limit) and no episode of it armed or flying; stopped "
                   f"as StopSession would stop it"}))
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
        if backend not in BACKENDS:
            raise HarnessError("INVALID_ARGUMENT", f"backend {backend!r}: one of {BACKENDS}")
        if backend in RAIL_BACKENDS and (self.rail is None
                                         or backend not in self.rail.backends):
            raise HarnessError("UNIMPLEMENTED",
                               f"backend {backend}: this server drives no {backend} rail "
                               f"(it was started without `serve --rail {backend}`, the rig "
                               f"its episodes would be flown on); the headless backend "
                               f"needs none")
        for name, v in (("scene", scene), ("airframe", airframe), ("label", label)):
            if len(v) > 256:
                raise HarnessError("INVALID_ARGUMENT", f"{name} is longer than 256 characters")
        endpoint = check_endpoint(rest_endpoint, self.rest_hosts) if rest_endpoint else ""
        if endpoint.lower().startswith("https:") and self._rest_tls is None:
            # Refused here, not later as an UNAVAILABLE on every forward: a
            # guardrail/api.py https endpoint requires a client certificate.
            raise HarnessError("FAILED_PRECONDITION",
                               f"rest_endpoint {endpoint} is https, and this server has no "
                               f"TLS identity for it (serve --rest-pki: the lab CA and a "
                               f"client certificate the endpoint's allow-list admits)")
        # Sessions left OPEN by clients that went away are stopped here,
        # before they are counted against MAX_OPEN_SESSIONS.
        now = time.monotonic()
        with self._lock:
            open_sessions = [x for x in self._sessions.values() if x.state == "OPEN"]
        for x in open_sessions:
            self._expire_session_if_idle(x, now)
        if backend in RAIL_BACKENDS and endpoint:
            self._check_rail_endpoint(backend, endpoint)
        sid = "ss-" + uuid.uuid4().hex[:12]
        s = _Session(session_id=sid, backend=backend, scene=scene, airframe=airframe,
                     seed=None if seed is None else int(seed), rest_endpoint=endpoint,
                     label=label, client=client)
        with self._lock:
            open_n = sum(1 for x in self._sessions.values() if x.state != "STOPPED")
            if open_n >= MAX_OPEN_SESSIONS:
                raise HarnessError("RESOURCE_EXHAUSTED", f"{open_n} sessions are open; stop "
                                                         f"some first")
            self._sessions[sid] = s
        self._emit(s, "session_started", "", {**self._session_ref(s), "scene": scene,
                                              "airframe": airframe, "label": label})
        return self._session_ref(s)

    def _check_rail_endpoint(self, backend: str, endpoint: str) -> None:
        """A rail session's endpoint is where the mission's own listener is
        started (--hotapply-port): it needs the port, and off loopback the
        node serves mutual TLS only, from the rig's PKI folder."""
        u = urllib.parse.urlsplit(endpoint)
        if u.port is None:
            raise HarnessError("INVALID_ARGUMENT",
                               f"rest_endpoint {endpoint}: a {backend} session's "
                               f"endpoint names the port the mission's hot-apply "
                               f"listener is started on")
        if not is_loopback(u.hostname) and u.scheme != "https":
            raise HarnessError("INVALID_ARGUMENT",
                               f"rest_endpoint {endpoint}: off loopback the mission's "
                               f"listener serves mutual TLS only (https)")
        if not is_loopback(u.hostname) and not self.rail.hotapply_pki:
            raise HarnessError("FAILED_PRECONDITION",
                               f"rest_endpoint {endpoint} is off loopback, and this "
                               f"server's rig names no PKI folder for the mission's "
                               f"listener (serve --rail-hotapply-pki: the lab PKI as "
                               f"the node sees it)")

    def stop_session(self, session_id: str, *,
                     first: tuple[str, dict] | None = None) -> dict:
        """Fail the session's ARMED and live episodes, let a scheduled one
        already flying finish, then end both streams with `session_stopped`
        - after that flight's rows, run events and episode_ended, so it
        blocks for the rest of the flight: its wait for the run lock behind
        other episodes' scheduled flights (any session's), its run_episode
        and its bundling - several run_episodes under contention. Until
        2026-10-19 the streams ended at once and that episode's items landed
        after `session_stopped`, where no open stream delivered them. A call
        that finds the session already stopping waits for it to stop. The
        call that stops it emits `first` (kind, detail) before anything else
        (the expiry's session_expired). If that call raises on its way, the
        session is still STOPPED (see _stopped_anyway) before the error
        leaves."""
        s = self._session(session_id)
        closed, mine = 0, False
        try:
            with s.cond:
                # Check and set under one lock: of two StopSessions, or a stop and
                # an expiry sweep, one stops the session, once.
                mine = s.state == "OPEN"
                if mine:
                    s.state = "STOPPING"
                    if first is not None:
                        self._emit(s, first[0], "", first[1])
            if not mine:
                with s.cond:
                    s.cond.wait_for(lambda: s.state == "STOPPED")
            else:
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
                    # Every flight of it has appended all it will (the failed
                    # live ones return at their next tick; the wait releases the
                    # lock they append under).
                    s.cond.wait_for(lambda: s.flying <= 0)
                    # The last event and the state change under one lock, so a
                    # stream cannot see STOPPED before it has seen the stop.
                    self._mark_stopped(s, {"episodes_closed_unfinished": closed})
                with self._lock:
                    stopped = sorted((x for x in self._sessions.values()
                                      if x.state == "STOPPED"), key=lambda x: x.stopped_at)
                    for x in stopped[:max(0, len(stopped) - MAX_KEPT_SESSIONS)]:
                        self._sessions.pop(x.session_id, None)
        except BaseException as exc:
            if mine:
                self._stopped_anyway(s, closed, exc)
            raise
        return {"session_id": s.session_id, "episodes": len(s.episodes),
                "episodes_closed_unfinished": closed, "telemetry_frames": len(s.telemetry),
                "telemetry_dropped": s.telemetry_dropped, "events": len(s.events)}

    @staticmethod
    def _mark_stopped(s: _Session, detail: dict) -> None:
        """Append `session_stopped` and make the session STOPPED, under one
        hold of s.cond (an RLock, so a caller may already hold it): a stream
        never sees STOPPED before the stop event, and every waiter - a second
        StopSession, a stream poll - is woken."""
        with s.cond:
            s.events.append({"session_id": s.session_id, "episode_id": "",
                             "seq": len(s.events), "wall_time": time.time(),
                             "kind": "session_stopped",
                             "detail_json": json.dumps(detail, sort_keys=True,
                                                       default=str)})
            s.state = "STOPPED"
            s.stopped_at = time.monotonic()
            s.cond.notify_all()

    def _stopped_anyway(self, s: _Session, closed: int, exc: BaseException) -> None:
        """The stopping call raised between STOPPING and STOPPED. STOPPED is
        reached only at the end of that call, and every other waiter waits
        for it with no timeout: a second StopSession (and an expiry sweep)
        in wait_for(STOPPED), both streams in wait_stream, which ends only
        on STOPPED. Before this (review of the 2026-10-19 fix), an error
        anywhere in between - an episode's expiry check, the `first` event,
        an interrupted wait - left the session STOPPING for good: later
        StopSessions hung, the streams never ended, and it counted against
        MAX_OPEN_SESSIONS until a restart. So the error still stops it, with
        the reason in `session_stopped`, and is re-raised by the caller. It
        does not wait for a flight still going (the wait may be what
        failed): such a flight's items can land after the stop, the
        pre-2026-10-19 behaviour, on this error path only."""
        with s.cond:
            if s.state == "STOPPED":          # the error came after the stop
                return
            self._mark_stopped(s, {"episodes_closed_unfinished": closed,
                                   "stop_error": f"{type(exc).__name__}: {exc}"})

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
        rail = s.backend in RAIL_BACKENDS
        if rail and bundle_bytes:
            raise HarnessError("INVALID_ARGUMENT",
                               f"a {s.backend} session flies a policy file both hosts "
                               f"hold (bundle_path under policies/ or bundles/): an "
                               f"uploaded archive exists on this server only")
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
        # A rail session's endpoint is the mission's own listener, which is
        # not up before StartEpisode: the mission is launched pinned to this
        # hash, and the endpoint is checked against it when it first answers
        # (_rail_driver), before anything is forwarded.
        if s.rest_endpoint and not rail:
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
        if rail:
            # What the rail is told to fly: the checkout-relative file, as
            # both hosts hold it, pinned to the injected hash.
            s.policy["rail_ref"] = (
                PolicyRef(path=source, expected_hash=pol.policy_hash) if is_yaml
                else PolicyRef(bundle_path=source, expected_hash=pol.policy_hash))
        self._emit(s, "policy_injected", "", {k: v for k, v in s.policy.items()
                                              if k not in ("ref", "rail_ref")})
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
        refusals mapped to gRPC codes. An https endpoint is reached with the
        server's REST TLS identity (rest_pki_dir); a TLS failure that is a
        configuration error (an endpoint certificate the lab CA did not sign
        or that does not name the host, a TLS alert refusing ours) is
        FAILED_PRECONDITION, not UNAVAILABLE: retrying cannot fix it. A common
        name off the endpoint's allow-list is a connection reset (api.py's
        gate aborts it), which the network can also cause: UNAVAILABLE, with
        a hint naming the allow-list and this server's certificate, since a
        client retrying it will never get through."""
        import ssl
        url = s.rest_endpoint + path
        data = None if body is None else json.dumps(body).encode("utf-8")
        req = urllib.request.Request(url, data=data, method=method,
                                     headers={"Content-Type": "application/json",
                                              "Accept": "application/json"})
        handlers: list = [urllib.request.ProxyHandler({})]
        if self._rest_tls is not None:      # replaces build_opener's default HTTPSHandler
            handlers.append(urllib.request.HTTPSHandler(context=self._rest_tls))
        opener = urllib.request.build_opener(*handlers)
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
            why = err.reason if isinstance(err, urllib.error.URLError) else err
            # An SSLEOFError (the peer just went away) may be the network, so
            # it stays UNAVAILABLE; a verify failure or a TLS alert is not.
            if isinstance(why, ssl.SSLError) and not isinstance(why, ssl.SSLEOFError):
                raise HarnessError("FAILED_PRECONDITION",
                                   f"the REST endpoint {url} failed the TLS handshake: {why} "
                                   f"(its certificate must be the lab CA's and name the "
                                   f"host; this server's must be on its allow-list)") from None
            # A reset over https is also what api.py's common-name gate does to
            # a lab-CA certificate that is not on the rail's --hotapply-client
            # allow-list (transport.abort(), no answer): verified end to end
            # in the review of the 2026-10-19 fix, where 'other-client' got
            # UNAVAILABLE 'Remote end closed connection without response' and
            # nothing else. The code stays UNAVAILABLE - a reset can be the
            # network - but the message names the likely configuration
            # error, or a client retries it for ever without a clue.
            hint = ""
            if (url.lower().startswith("https:") and self.rest_client_name
                    and isinstance(why, (ConnectionResetError, ConnectionAbortedError,
                                         BrokenPipeError, ssl.SSLEOFError))):
                hint = (f"; if this repeats, check that the endpoint's allow-list "
                        f"(--hotapply-client on the rail) names this server's client "
                        f"certificate {self.rest_client_name!r}: a common name off it "
                        f"is reset, not refused")
            raise HarnessError("UNAVAILABLE", f"the REST endpoint {url} did not answer: "
                                              f"{err}{hint}") from None
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
        # A rail episode of this session that is flying: the forward is also
        # recorded in ITS bundle (harness_events.jsonl), at its mission time.
        # Its endpoint is that mission's flying Shield, so an episode of a
        # pilot that consumes the CSP refuses it as it refuses its own.
        flying = self._rail_episode_of(s)
        if flying is not None:
            self._consumer_takes_no_events(flying, "HotApply to the session")
        return self._forward(s, body, episode=flying)[0]

    def _rail_episode_of(self, s: _Session) -> "_Episode | None":
        with self._lock:
            eps = [self._episodes.get(i) for i in list(s.episodes)]
        for e in eps:
            if e is not None and e.rail is not None and e.state == "RUNNING":
                return e
        return None

    def _forward(self, s: _Session, body, *, episode: "_Episode | None" = None,
                 at_sim_t: float | None = None, queue_index: int | None = None,
                 source: str = "session", extra: dict | None = None) -> tuple[dict, dict]:
        """One hot-apply body to the session's REST endpoint, by its own route
        (rest_route: POST /hot_apply, or PATCH /dynamic_nfz/<id> for an edit
        of a zone), with the endpoint's answer. Returns (the PolicyAck, the
        record); a refusal is recorded and re-raised. With `episode` (a rail
        episode in flight) the record - its mission time as this server
        reckons it, the request, the generation and the endpoint's receipt,
        or the refusal - is kept for the episode's harness_events.jsonl.
        `source` says who asked: session, episode (a HotApply to it),
        emit_event, or scenario (the scenario's own event, StartEpisode
        drive_scenario_events); `extra` (the ScenarioEvent's type and index)
        goes into the record as it is."""
        method, path, payload = rest_route(body)
        url = s.rest_endpoint + path
        rf = episode.rail if episode is not None else None
        sent = time.monotonic()
        rec: dict[str, Any] = {
            "type": "hot_apply", "via": "rest", "source": source,
            "t": (None if rf is None or rf.t0 is None else round(sent - rf.t0, 3)),
            "at_sim_t": at_sim_t, "queue_index": queue_index,
            "wall_time": round(time.time(), 4), "method": method, "url": url,
            "body": body, "event": str(body.get("type") or ""),
            "op": str(body.get("op") or ""),
            "rule_id": str(body.get("id") or body.get("target_id") or ""), **(extra or {})}
        detail = {"url": url, "body": body}
        if method != "POST":
            detail["method"] = method
        if episode is not None:
            detail.update(mission_t=rec["t"], at_sim_t=at_sim_t, source=source, **(extra or {}))
        eid = episode.episode_id if episode is not None else ""
        try:
            res = self._rest(s, method, path, payload)
        except HarnessError as err:
            rec.update(refused=True, code=err.code, reason=err.message,
                       rtt_ms=round((time.monotonic() - sent) * 1e3, 2))
            if rf is not None:
                with rf.cond:
                    rf.forwarded.append(rec)
            self._emit(s, "hot_apply_forwarded", eid, {**detail, "refused": err.code,
                                                       "reason": err.message})
            raise
        gen = res.get("generation")
        ack = {"session_id": s.session_id, "rule_id": str(res.get("applied") or ""),
               "op": str(res.get("op") or ""), "type": str(res.get("type") or ""),
               "generation": int(gen) if isinstance(gen, int) else 0,
               "policy_hash": str(res.get("policy_hash") or ""), "forwarded_to": url,
               "forwarded_method": method}
        ack["validated_generation"] = ack["generation"]
        ack["validated_policy_hash"] = ack["policy_hash"]
        rec.update(refused=False, rule_id=ack["rule_id"] or rec["rule_id"],
                   op=ack["op"] or rec["op"],
                   generation=gen if isinstance(gen, int) else None,
                   policy_hash=ack["policy_hash"] or None, receipt=res,
                   rtt_ms=round((time.monotonic() - sent) * 1e3, 2))
        if rf is not None:
            with rf.cond:
                rf.forwarded.append(rec)
        self._emit(s, "hot_apply_forwarded", eid, {**detail, "response": res})
        return ack, rec

    # ---- rail episodes (2026-11-10, WP4-05 / WP4-03) --------------------- #
    #
    # A session whose backend is a flight rail (sitl, hil) flies its episodes
    # on the server's rig, through experiments/sweep_scenarios.py's own rail
    # backends: run_hil with its transport (SITL in WSL, the scenario's files
    # to the Orin, the mission container, the run folder fetched and accepted
    # by its own records) or run_sitl, then the same score() / rail_gate_set /
    # episode_verdict / write_scenario_score a `--backend hil` campaign runs.
    # StartEpisode LAUNCHES the mission (it flies in real time, like a live
    # episode); a HotApply addressed to the episode waits for its mission
    # time and is forwarded to the mission's own REST listener, which the
    # mission was launched with (the session's endpoint names its port);
    # EndEpisode waits for the flight, and the bundle is the grant's six
    # items under the grant's names.

    def _hil_cfg(self):
        return self.rail.hil_config or self._S.HilConfig()

    def _rail_hot_args(self, sess: _Session) -> list[str]:
        """The Shield node's hot-apply listener flags for the session's
        endpoint ([] when it names none: the mission then takes no REST
        event). Off loopback the node serves mutual TLS from the rig's PKI
        folder, as the node sees it (rail_hotapply_args)."""
        return rail_hotapply_args(sess.rest_endpoint, self.rail)

    def _rail_prepare(self, sess: _Session, spec, ep, defaults, *,
                      drive: bool = False) -> _RailFlight:
        """What a rail StartEpisode launches, or why it cannot: the sweep's own
        refusal (rail_refusal: what neither rail can realise), a Shield-off
        control arm the rig is not set to fly, a policy the rail cannot be
        pinned to, an argument a node would drop. Nothing is launched, and
        no episode created, when it raises.

        `drive` (StartEpisode drive_scenario_events, 2026-11-14, WP4-03): the
        scenario's own events are the HARNESS's to apply. Each must have a
        REST form (scenario_rest_timeline; one that has none refuses the
        episode, naming it), the session must be bound to the mission's
        listener, and the mission is launched with the same scenario minus
        its events - no --events file - so the flying Shield gets each one
        exactly once, over REST, at its mission time."""
        S, rig, backend = self._S, self.rail, sess.backend
        rail_ref = (sess.policy or {}).get("rail_ref")
        rspec = spec.model_copy(update={"policy": rail_ref}) if rail_ref is not None else spec
        try:
            why = S.rail_refusal(rspec, backend, episode=ep, defaults=defaults)
        except Exception as err:                                 # noqa: BLE001
            raise HarnessError("FAILED_PRECONDITION",
                               f"the {backend} rail cannot be asked about this scenario: "
                               f"{type(err).__name__}: {err}") from None
        if why is None and not rspec.shield and not rig.controls:
            why = ("a Shield-off control arm: flown only on a rig started with "
                   "--rail-controls (it flies the raw pilot into the zones, and the "
                   "GeoFence backstop may take the aircraft)")
        if why:
            raise HarnessError("FAILED_PRECONDITION", why)
        timeline: list[dict] = []
        if drive:
            if not sess.rest_endpoint:
                raise HarnessError("FAILED_PRECONDITION",
                                   "drive_scenario_events forwards the scenario's events to "
                                   "the mission's REST listener, and this session names no "
                                   "rest_endpoint to start it on")
            timeline = scenario_rest_timeline(rspec)
            rspec = rspec.model_copy(update={"events": []})
        hot = self._rail_hot_args(sess)
        try:
            cfg = self._hil_cfg() if backend == "hil" else None
            S.rail_node_args(rspec, ep.seed, defaults, backend=backend,
                             bundles_dir=cfg.bundles_dir if cfg else None, episode=ep)
            S.rail_extra_args(hot)
        except S.UnsupportedScenario as err:
            raise HarnessError("FAILED_PRECONDITION", str(err)) from None
        if backend == "sitl" and rig.sitl_runner is None and not S.sitl_available():
            raise HarnessError("FAILED_PRECONDITION",
                               "the ArduPilot SITL rail is not on this host (wsl, "
                               "sitl/run_ros2_demo.sh, sitl/ros2_shield_node.py)")
        dt = float(rspec.dt or defaults.dt)
        ticks = int(rspec.ticks or defaults.ticks)
        max_s = float(S.rail_max_s(ticks * dt))
        late = [x for x in timeline if x["at_sim_t"] > max_s + 1e-9]
        if late:
            raise HarnessError("FAILED_PRECONDITION",
                               f"scenario event #{late[0]['index']} at {late[0]['at_sim_t']:g} s "
                               f"is past the rail's time cap {max_s:.1f} s: it would never be "
                               f"forwarded")
        return _RailFlight(spec=rspec, hot_args=hot, endpoint=sess.rest_endpoint,
                           max_s=max_s, events_via="harness" if drive else "mission",
                           timeline=timeline)

    def plan_rail_episode(self, *, backend: str = "hil", rest_endpoint: str = "",
                          scenario_id: str = "", scenario_yaml: str = "", cell_id: str = "",
                          seed: int | None = None, profile: str = "", arm: str = "",
                          drive_scenario_events: bool = False, hot_apply=(), emit=(),
                          k: int = 1) -> dict:
        """What one rail session's StartEpisode would launch and forward,
        launching nothing (2026-11-14; tools/harness_hil_episode.py --dry-run).
        No session or episode is created, the rig is not taken, and nothing
        goes over a network: the request is resolved by start_episode's own
        _resolve, refused by its own _rail_prepare (a HarnessError, as
        StartEpisode would answer), and the mission's arguments are built as
        run_hil builds them (rail_node_args with the campaign files' paths,
        plus the listener's flags) for mission number `k` of this server's
        campaign. `hot_apply`: (at_sim_t, body) HotApplys to the episode;
        `emit`: (at_sim_t, type, payload) EmitEvents. Returns {cell, seed,
        arm, tag, sitl_home, node_args, scenario_events_via, timeline (every
        forward in mission-time order: source, at_sim_t, method, url,
        payload), run_dir, bundle_dir, bundle_files}. What it cannot see: the
        server's REST TLS identity, the Orin, the mission's image."""
        S, rig = self._S, self.rail
        if backend not in RAIL_BACKENDS or rig is None or backend not in rig.backends:
            raise HarnessError("UNIMPLEMENTED", f"backend {backend}: this server drives no "
                                                f"{backend} rail (serve --rail {backend})")
        if bool(scenario_id) == bool(scenario_yaml):
            raise HarnessError("INVALID_ARGUMENT",
                               "give exactly one of scenario_id or scenario_yaml")
        endpoint = check_endpoint(rest_endpoint, self.rest_hosts) if rest_endpoint else ""
        if endpoint:
            self._check_rail_endpoint(backend, endpoint)
        sess = _Session(session_id="(plan)", backend=backend, scene="", airframe="",
                        seed=None if seed is None else int(seed), rest_endpoint=endpoint,
                        label="", client="")
        r = self._resolve(scenario_id=scenario_id, scenario_yaml=scenario_yaml,
                          cell_id=cell_id, seed=seed, profile=profile, arm=arm, sess=sess)
        defaults = r.lib.defaults
        flight = self._rail_prepare(sess, r.spec, r.ep, defaults,
                                    drive=bool(drive_scenario_events))
        if (hot_apply or emit) and not endpoint:
            raise HarnessError("FAILED_PRECONDITION",
                               "a HotApply or EmitEvent to a rail episode needs the session "
                               "bound to the mission's listener (rest_endpoint)")
        mission = flight.spec
        cfg = self._hil_cfg() if backend == "hil" else None
        tag = S.hil_run_id(self._rail_campaign, int(k), mission)
        rel = f"{S.CAMPAIGN_DIR}/{tag}"
        try:
            args = S.rail_node_args(mission, r.ep.seed, defaults, backend=backend,
                                    events=(f"{rel}/events.json" if mission.events else None),
                                    meta=f"{rel}/scenario.json",
                                    bundles_dir=cfg.bundles_dir if cfg else None,
                                    episode=r.ep)
            args += S.rail_extra_args(flight.hot_args)
            home = S.sitl_home(mission, mission.policy.load(ROOT), defaults, r.ep.seed)
        except S.UnsupportedScenario as err:
            raise HarnessError("FAILED_PRECONDITION", str(err)) from None

        def item(source: str, at: float, body: dict, **more) -> dict:
            method, path, payload = rest_route(body)
            return {"source": source, "at_sim_t": at, "method": method,
                    "url": endpoint + path, "payload": payload, **more}

        def when(at) -> float:
            a = _finite(at, "at_sim_t")
            if a < 0 or a > flight.max_s + 1e-9:
                raise HarnessError("OUT_OF_RANGE", f"at_sim_t {a} s is outside the mission "
                                                   f"(0 to the rail's cap {flight.max_s:.1f} s)")
            return a

        timeline = [item("scenario", x["at_sim_t"], x["body"], scenario_event=x["type"],
                         scenario_event_index=x["index"]) for x in flight.timeline]
        timeline += [item("episode", when(at), dict(body)) for at, body in hot_apply]
        for at, typ, payload in emit:
            try:
                ev = ScenarioEvent.model_validate({"at_sim_t": when(at), "type": typ,
                                                   "payload": dict(payload or {})})
            except ValidationError as err:
                raise HarnessError("INVALID_ARGUMENT",
                                   f"ScenarioEvent refused: {err}") from None
            timeline.append(item("emit_event", ev.at_sim_t,
                                 rest_body_of_event(ev, origin=_policy_origin(mission)),
                                 scenario_event=ev.type))
        timeline.sort(key=lambda x: x["at_sim_t"])            # stable: ties as queued
        store = (Path(rig.store or (cfg.store if cfg else None) or self.runs_dir / "rail_runs")
                 if backend == "hil" else
                 Path(rig.sitl_out) if rig.sitl_out else ROOT / "demo" / "out")
        if not store.is_absolute():
            store = ROOT / store
        return {"backend": backend, "scenario_id": r.ep.cell.scenario_id,
                "cell_id": r.ep.episode_id, "seed": r.ep.seed, "arm": r.ep.arm_tag,
                "policy": mission.policy.source, "tag": tag, "sitl_home": home,
                "shield": "on" if mission.shield else "off",
                "rail_max_s": flight.max_s, "node_args": args,
                "listener_args": list(flight.hot_args), "rest_endpoint": endpoint or None,
                "scenario_events_via": flight.events_via,
                "mission_events": len(mission.events), "timeline": timeline,
                "run_dir": (store / tag).as_posix(),
                "bundle_dir": (self.runs_dir / "<episode_id>"
                               / r.ep.dir_name("<UTC time>")).as_posix(),
                "bundle_files": ["manifest.json", POLICY_BUNDLE_FILE, EPISODE_BAG_DIR + "/",
                                 "shield_audit.jsonl", "harness_events.jsonl", "kpi.json"]}

    def _rail_phase(self, e: _Episode, phase: str, **detail) -> None:
        e.rail.phase = phase
        self._emit_episode(e, "rail_phase", {"phase": phase, **detail})

    def _rail_emit_event(self, e: _Episode, at_sim_t, type_: str, payload, now: bool) -> dict:
        """An EmitEvent addressed to a rail episode (2026-11-14, WP4-03; until
        then UNIMPLEMENTED): the ScenarioEvent is validated as everywhere,
        turned into the REST body that makes the flying Shield apply it as
        the mission would (rest_body_of_event: a translate / rotate / scale
        of a zone is PATCH /dynamic_nfz/<id>), and queued for its mission
        time like a HotApply to the episode. On a rail the event names its
        zone (`id`): the mission's own spawn counter and last zone are not
        this server's to guess. A form with no REST body (a zone placed
        against the vehicle, a change of motion, an expected refusal) is
        UNIMPLEMENTED, with why."""
        try:
            ev = ScenarioEvent.model_validate({"at_sim_t": 0.0 if now else at_sim_t,
                                               "type": type_, "payload": dict(payload or {})})
        except ValidationError as err:
            raise HarnessError("INVALID_ARGUMENT", f"ScenarioEvent refused: {err}") from None
        body = rest_body_of_event(ev, origin=e.policy.frame_origin)
        ack = self._rail_hot_apply(
            e, at_sim_t, body, now, source="emit_event", kind=ev.type,
            extra={"scenario_event": ev.type},
            received={"rpc": "EmitEvent", "at_sim_t": None if now else at_sim_t,
                      "now": bool(now), "type": type_, "payload": payload, "body": body})
        return {"episode_id": ack["episode_id"], "queue_index": ack["queue_index"],
                "at_sim_t": ack["at_sim_t"], "type": ev.type, "pending": True,
                "forwarded_method": ack["forwarded_method"],
                "forwarded_to": ack["forwarded_to"],
                "rest_body_json": json.dumps(body, sort_keys=True)}

    def _rail_hot_apply(self, e: _Episode, at_sim_t, body, now: bool, *,
                        source: str = "episode", kind: str = "",
                        extra: dict | None = None, received: dict | None = None) -> dict:
        """A HotApply addressed to a rail episode: queued for mission time
        `at_sim_t` (`now`: sent as soon as the mission's listener answers),
        then forwarded by the episode's driver to the endpoint the mission
        serves. The endpoint is the validator - it holds the flying Shield,
        the scenario's own events included, which the headless dry run never
        knew - so the ack is `pending`, and the receipt (or the refusal)
        follows as a hot_apply_forwarded event and in the bundle. `source`
        (episode, emit_event), `kind`, `extra` and `received` are what an
        EmitEvent records instead of a HotApply's own."""
        s = self._flight_session(e)
        rf = e.rail
        if s is None or not s.rest_endpoint:
            raise HarnessError("FAILED_PRECONDITION",
                               f"episode {e.episode_id}'s session is not bound to a REST "
                               f"endpoint: its mission was launched with no hot-apply "
                               f"listener")
        method, path, _ = rest_route(body)
        if not body.get("type"):
            raise HarnessError("INVALID_ARGUMENT", "body must be a non-empty object with a "
                                                   "`type`")
        with e.lock:
            if e.state != "RUNNING" or rf.over.is_set():
                why = f" ({e.error})" if e.state == "FAILED" and e.error else ""
                raise HarnessError("FAILED_PRECONDITION",
                                   f"episode {e.episode_id} is {e.state}{why}: its mission "
                                   f"takes no more events")
            if rf.endpoint_problem:
                raise HarnessError("FAILED_PRECONDITION",
                                   f"nothing is forwarded to episode {e.episode_id}'s "
                                   f"mission: {rf.endpoint_problem}")
            if len(e.queued) >= MAX_EVENTS_PER_EPISODE:
                raise HarnessError("RESOURCE_EXHAUSTED",
                                   f"{len(e.queued)} events queued in {e.episode_id}")
            if now:
                at = None
            else:
                at = _finite(at_sim_t, "at_sim_t")
                if at < 0:
                    raise HarnessError("INVALID_ARGUMENT",
                                       f"at_sim_t {at} is before the mission")
                if at > rf.max_s + 1e-9:
                    raise HarnessError("OUT_OF_RANGE",
                                       f"at_sim_t {at} s never fires: the rail's time cap "
                                       f"for {e.episode_id} is {rf.max_s:.1f} s")
                if rf.t0 is not None and at < time.monotonic() - rf.t0:
                    raise HarnessError("OUT_OF_RANGE",
                                       f"at_sim_t {at} s has already flown: the mission is "
                                       f"at {time.monotonic() - rf.t0:.2f} s (send it with "
                                       f"`now`)")
            idx = len(e.queued)
            rid = str(body.get("id") or body.get("target_id") or "")
            kind = kind or (str(body.get("type")) + (f"/{body['op']}" if body.get("op") else ""))
            q = _Queued(index=idx,
                        event=SimpleNamespace(at_sim_t=0.0 if at is None else at, type=kind),
                        source="emit_event" if source == "emit_event" else "hot_apply",
                        rule_id=rid,
                        received=received or {"rpc": "HotApply", "at_sim_t": at,
                                              "now": bool(now), "body": body})
            e.queued.append(q)
            with rf.cond:
                rf.scheduled.append((at, idx, dict(body), source, dict(extra or {})))
                rf.cond.notify_all()
            e.idle_since = time.monotonic()
            ack = {"episode_id": e.episode_id, "queue_index": idx,
                   "at_sim_t": 0.0 if at is None else at, "rule_id": rid,
                   "op": str(body.get("op") or ""), "type": str(body.get("type") or ""),
                   "session_id": e.session_id, "pending": True,
                   "forwarded_to": s.rest_endpoint + path, "forwarded_method": method}
        self._emit_episode(e, "emit_event" if source == "emit_event" else "hot_apply",
                           {**ack, "now": bool(now), **({"body": body} if source == "emit_event"
                                                       else {}), **(extra or {})})
        return ack

    def _rail_loop(self, e: _Episode) -> None:
        """A rail episode's thread: the flight (the sweep's rail backend,
        blocking for the whole mission), its driver beside it when the
        session names an endpoint, then the score and the bundle. Counted as
        a flight of its session, so StopSession waits for it; the rig is
        released when it is over, whatever happened."""
        s = self._flight_session(e)
        rf = e.rail
        if s is not None:
            with s.cond:
                s.flying += 1
        try:
            if s is not None and s.rest_endpoint:
                rf.driver = threading.Thread(target=self._rail_driver, args=(e, s),
                                             daemon=True, name=f"rail-drv-{e.episode_id}")
                rf.driver.start()
            self._fly_rail(e, s)
        finally:
            rf.over.set()
            with rf.cond:
                rf.cond.notify_all()
            self._rail_busy.release()
            self._flight_over(s)

    def _rail_driver(self, e: _Episode, s: _Session) -> None:
        """Beside a rail flight: wait for the mission's REST listener (it
        starts on the pad, before arming), check it flies the episode's
        policy, find mission time 0, and forward each queued hot-apply at its
        mission time - never early.

        MISSION TIME 0 is when GET <endpoint>/health first reports
        `mission_started` (the Shield's first filter() tick), polled every
        RAIL_HEALTH_POLL_S and only until then: the clock is the time the
        answer that saw it was RECEIVED. The listener answered after the
        mission started, so this server's mission time runs late by at most
        one poll period plus that request's round trip, and an event is
        forwarded that much late, never early. (Until 14 Nov the clock was the
        time the request was SENT; a request in flight across the start - on
        the fake rig, one waiting for the lock the first tick held - put time
        0 before the mission's own, and the lab CI at e07ff4b caught a spawn
        forwarded at 0.187 s of the mission's clock for its 0.2 s.) The rail's own
        record of each applied event carries its true mission time
        (metrics.json hot_apply.events), and the bundle keeps it beside this
        server's (`rail`). An endpoint whose FIRST answer already says
        mission_started gives no time 0 (a listener left by another mission,
        or one this server reached too late): nothing is forwarded to it."""
        rf = e.rail
        self._rail_phase(e, "awaiting_endpoint", endpoint=rf.endpoint)
        up = False
        while not rf.over.is_set():
            if not up or rf.t0 is None:
                try:
                    h = self._rest(s, "GET", "/health")
                except HarnessError as err:
                    if err.code != "UNAVAILABLE":
                        # A configuration error (TLS, not a guardrail endpoint):
                        # retrying cannot fix it.
                        rf.endpoint_problem = f"{err.code}: {err.message}"
                        break
                    h = None
                if h is not None and not up:
                    up = True
                    rf.endpoint_policy_hash = str(h.get("policy_hash") or "")
                    if h.get("mission_started"):
                        rf.endpoint_problem = (
                            f"{rf.endpoint} reported mission_started on its first "
                            f"answer: its mission time 0 was not observed")
                        break
                    if h.get("generation") in (0, None) \
                            and not e.policy.matches_hash(rf.endpoint_policy_hash):
                        rf.endpoint_problem = (
                            f"{rf.endpoint} flies policy "
                            f"{rf.endpoint_policy_hash or '(none reported)'}, not the "
                            f"episode's {e.policy_hash}")
                        break
                    self._rail_phase(e, "awaiting_mission_start", endpoint=rf.endpoint,
                                     endpoint_policy_hash=rf.endpoint_policy_hash)
                elif h is not None and h.get("mission_started"):
                    # The receipt, not the send: the start may fall while this
                    # request is in flight, never after its answer.
                    rf.t0, rf.t0_wall = time.monotonic(), time.time()
                    self._rail_phase(e, "mission", mission_t0_wall=rf.t0_wall)
            now = time.monotonic()
            with rf.cond:
                due = [x for x in rf.scheduled
                       if (x[0] is None and up)
                       or (x[0] is not None and rf.t0 is not None and rf.t0 + x[0] <= now)]
                for x in due:
                    rf.scheduled.remove(x)
            # `now` bodies in arrival order first, then by mission time.
            for at, idx, body, source, extra in sorted(
                    due, key=lambda x: (x[0] is not None, x[0] or 0.0, x[1])):
                try:
                    self._forward(s, body, episode=e, at_sim_t=at, queue_index=idx,
                                  source=source, extra=extra)
                except HarnessError:
                    pass                    # recorded, and streamed, by _forward
            with rf.cond:
                if rf.over.is_set():
                    break
                if not up:
                    wait = RAIL_ENDPOINT_POLL_S
                elif rf.t0 is None:
                    wait = RAIL_HEALTH_POLL_S
                elif any(x[0] is None for x in rf.scheduled):
                    wait = 0.0
                else:
                    nxt = min((rf.t0 + x[0] for x in rf.scheduled), default=None)
                    wait = 0.5 if nxt is None else max(0.0, min(0.5, nxt - time.monotonic()))
                if wait > 0:
                    rf.cond.wait(wait)
        if rf.endpoint_problem:
            self._rail_phase(e, rf.phase, endpoint_problem=rf.endpoint_problem)
            rf.over.wait()
        # What was never sent is said, never dropped.
        with rf.cond:
            left, rf.scheduled = sorted(rf.scheduled, key=lambda x: x[1]), []
        why = rf.endpoint_problem or (
            "the mission ended before its mission time" if rf.t0 is not None else
            "the mission ended before its listener reported mission_started")
        for at, idx, body, source, extra in left:
            rec = {"type": "hot_apply", "via": "rest", "source": source, "t": None,
                   "at_sim_t": at, "queue_index": idx, "wall_time": round(time.time(), 4),
                   "body": body, "event": str(body.get("type") or ""),
                   "op": str(body.get("op") or ""),
                   "rule_id": str(body.get("id") or body.get("target_id") or ""),
                   **extra,
                   "refused": True, "sent": False, "code": "NOT_SENT", "reason": why}
            with rf.cond:
                rf.forwarded.append(rec)
                rf.not_sent.append(idx)
            self._emit(s, "hot_apply_forwarded", e.episode_id,
                       {"body": body, "at_sim_t": at, "not_sent": why})

    def _fly_rail(self, e: _Episode, s: _Session | None) -> None:
        rf = e.rail
        try:
            try:
                run = self._rail_run(e)
            finally:
                # The mission is over (or never flew): the driver stops, and
                # says what it never sent, before anything is bundled.
                rf.over.set()
                with rf.cond:
                    rf.cond.notify_all()
                if rf.driver is not None:
                    rf.driver.join(REST_TIMEOUT_S + 10.0)
            self._rail_phase(e, "fetched", run=Path(run["run_dir"]).as_posix())
            self._rail_scenario_check(e, run)
            result = self._bundle_rail(e, s, run)
        except HarnessError as err:
            msg = err.message
        except Exception as err:                                 # noqa: BLE001
            msg = (f"the {e.backend} rail did not fly the episode: "
                   f"{type(err).__name__}: {err}")
        else:
            e.ticks_flown = len(run["rows"])
            if self._finish(e, "ENDED", result=result):
                rf.phase = "scored"
                self._frames(e, [_row_frame(e, r) for r in run["rows"]])
                self._emit_episode(e, "episode_ended", {
                    k: result.get(k) for k in ("bundle_path", "status", "gates_failed",
                                               "mission_outcome", "kpi_grade",
                                               "rail_run_path", "bundle_files")})
            return
        if self._finish(e, "FAILED", error=msg):
            self._emit_episode(e, "episode_failed", {"error": msg})

    @staticmethod
    def _rail_scenario_check(e: _Episode, run: dict) -> None:
        """A harness-driven scenario (drive_scenario_events) flew as the
        scenario only if every one of its own events reached the flying
        Shield: each forwarded once and applied, with a generation. One the
        endpoint refused, or that was never sent (the mission ended first,
        the listener never reported its start), makes it another flight -
        the campaign's own rule for the mission's --events
        (sweep_scenarios.hil_flight_problems: an error, never scored). So the
        episode is FAILED_PRECONDITION with which and why, and nothing is
        bundled; the run folder keeps everything the mission recorded."""
        rf = e.rail
        if rf.events_via != "harness":
            return
        with rf.cond:
            recs = [r for r in rf.forwarded if r.get("source") == "scenario"]
        ok = {r.get("queue_index") for r in recs
              if not r.get("refused") and r.get("generation") is not None}
        want = [q for q in e.queued if q.source == "scenario"]
        bad = []
        for q in want:
            if q.index in ok:
                continue
            r = next((x for x in recs if x.get("queue_index") == q.index), None)
            why = ("never forwarded" if r is None else
                   f"{r.get('code')}: {r.get('reason')}")
            bad.append(f"#{q.received['scenario_event']['index']} {q.event.type} at "
                       f"{q.event.at_sim_t:g} s ({why})")
        if bad:
            raise HarnessError(
                "FAILED_PRECONDITION",
                f"the scenario's own event(s) the harness drove did not all reach the "
                f"flying Shield: {'; '.join(bad)}. The flight is not the scenario's, so it "
                f"is not scored (the campaign's rule for its --events); nothing is "
                f"bundled under this episode (its run folder: "
                f"{Path(run['run_dir']).as_posix()})")

    def _rail_run(self, e: _Episode) -> dict:
        """Fly the episode's mission on the rig and return the sweep's own
        record of it ({rows, extra, policy, prios, run_dir, tag, rc, meta,
        metrics}): run_hil's, or the same built from run_sitl's."""
        S, rig, rf, ep = self._S, self.rail, e.rail, e.ep
        with self._lock:
            self._rail_k += 1
            k = self._rail_k
        rf.tag = S.hil_run_id(self._rail_campaign, k, rf.spec)
        extra_args = list(rf.hot_args) or None
        if e.backend == "hil":
            cfg = self._hil_cfg()
            transport = (rig.transport or S.make_hil_transport)(cfg)
            # The run store: the rig's, else the hil config's own (the
            # sweep's --hil-config `store`), else beside this server's bundles.
            store = Path(rig.store or cfg.store or self.runs_dir / "rail_runs")
            if not store.is_absolute():
                store = ROOT / store
            return S.run_hil(rf.spec, ep, ep.seed, e.defaults, cfg, transport,
                             self._rail_campaign, k, store, extra_args=extra_args)
        got = (rig.sitl_runner or S.run_sitl)(rf.spec, rf.tag, ep.seed, e.defaults,
                                              episode=ep, extra_args=extra_args)
        if got is None:
            raise HarnessError("FAILED_PRECONDITION",
                               "the ArduPilot SITL rail is not on this host")
        rows, extra = got
        run_dir = (Path(rig.sitl_out) if rig.sitl_out else ROOT / "demo" / "out") / rf.tag
        mpath = run_dir / "metrics.json"
        metrics = (json.loads(mpath.read_text(encoding="utf-8")) if mpath.is_file() else {})
        pol = rf.spec.policy.load(ROOT)
        meta = metrics.get("scenario") if isinstance(metrics.get("scenario"), dict) else {}
        return {"rows": rows, "rows_all": rows, "extra": extra, "policy": pol,
                "prios": S.rail_rule_priorities(pol, S.run_generations(run_dir)),
                "run_dir": run_dir, "tag": rf.tag, "rc": 0, "meta": meta,
                "metrics": metrics}

    def _bundle_rail(self, e: _Episode, s: _Session | None, run: dict) -> dict:
        """Score a flown rail run as a `--backend hil` campaign does (score,
        rail_gate_set on the cell's own gates, episode_verdict, and for hil
        scenario_score.json beside the run), then write the episode bundle
        under the grant's names (Stress Testing, "Episode bundle"):

          manifest.json         the mission's own, byte for byte;
          policy_bundle.tar.gz  the policy file the rail was given and EVERY
                                generation's IR, from the run's own
                                policy_g<N>.json (write_policy_bundle, held);
          episode.bag2/         the run's rosbag2 directory. The rail records
                                it as bag/ (deploy/docker/entrypoint.sh,
                                sitl/run_ros2_demo.sh), and it keeps that
                                name THERE: the mission's replay pack, the
                                exporters and the entrypoint's own handling
                                of an earlier bag read it, and renaming it
                                is a change on the flying side. The bundle
                                gets the same files under the grant's name -
                                an alias (hard links on one volume, copies
                                otherwise; alias_tree), each with its SHA-256
                                in kpi.json, not a rename;
          shield_audit.jsonl    the run's, byte for byte;
          harness_events.jsonl  the episode's start, each hot-apply this
                                server forwarded (the request, its mission
                                time, the generation and the endpoint's
                                receipt, or the refusal; the rail's own
                                record of it beside, `rail`), what was never
                                sent, and the end;
          kpi.json              the KPI table, the verdict, and `harness`
                                (which of the six were written, and why not).

        Beside them since 2026-11-18 (bundle_replay_inputs, kpi.json
        harness.replay_files): metrics.json, trajectory.json and the run's
        signed <tag>.replay.tar.gz, so the bundle alone can be replayed at
        its flown revision (tools/replay_at_revision.py).

        A file the run folder lacks is NOT made up: kpi.json's
        harness.bundle_files says "not written" and why."""
        import shutil
        S = self._S
        from . import kpi as K
        from .manifest import is_kpi_grade
        ep, rf, spec = e.ep, e.rail, e.rail.spec
        run_dir = Path(run["run_dir"])
        rows, extra, policy, prios = run["rows"], run["extra"], run["policy"], run["prios"]
        metrics = run.get("metrics") if isinstance(run.get("metrics"), dict) else {}
        meta = run.get("meta") if isinstance(run.get("meta"), dict) else {}
        held = read_held_generations(run_dir)
        manifest = None
        man_src = run_dir / "manifest.json"
        if man_src.is_file():
            try:
                manifest = json.loads(man_src.read_text(encoding="utf-8"))
            except ValueError:
                manifest = None
            if not isinstance(manifest, dict):
                manifest = None
        flown = flown_start_policy_hash(manifest, held)
        if flown and not e.policy.matches_hash(flown):
            raise HarnessError("FAILED_PRECONDITION",
                               f"the mission flew policy {flown}, not the episode's "
                               f"{e.policy_hash}; nothing is bundled under this episode "
                               f"(its run folder: {run_dir.as_posix()})")

        # Scored as the sweep scores it: against the cell's spec, less its
        # labels on the prefix-on arm of a consuming pilot (scoring_spec).
        sspec = S.scoring_spec(spec, ep)
        res = S.score(sspec, rows, extra, policy, prios)
        gates, _ = S.rail_gate_set("ideal", spec, ep.seed, e.defaults)
        status, bad, labels_failed, known_why = S.episode_verdict(
            sspec, res, ep.cell.params, ep.seed, gates=gates)
        score_path = None
        if e.backend == "hil" and meta:
            # Beside the fetched run, as the campaign writes it (kpi_report.py).
            score_path = S.write_scenario_score(
                run_dir, meta, res, extra, status, bad, labels_failed, known_why, prios,
                sspec, gates_basis="ideal", gates=gates)

        stamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
        ep_dir = self.runs_dir / e.episode_id / ep.dir_name(stamp)
        ep_dir.mkdir(parents=True, exist_ok=True)
        files: dict[str, str] = {}

        def text(name: str, doc) -> None:
            with open(ep_dir / name, "w", encoding="utf-8", newline="\n") as fh:
                fh.write(doc)

        # manifest.json
        if manifest is not None:
            shutil.copyfile(man_src, ep_dir / "manifest.json")
            files["manifest.json"] = "written: the mission's own manifest.json, byte for byte"
            try:
                grade, why_not = is_kpi_grade(manifest, metrics)
            except Exception as err:                             # noqa: BLE001
                grade, why_not = False, [f"is_kpi_grade could not read the manifest: "
                                         f"{type(err).__name__}: {err}"]
        else:
            files["manifest.json"] = (f"not written: the run folder {run_dir.name} has no "
                                      f"readable manifest.json, and none is made up for a "
                                      f"flight this server did not record")
            grade, why_not = False, [f"the run folder {run_dir.name} has no manifest.json"]

        # The forwards, each beside the rail's own record of the same generation.
        with rf.cond:
            forwarded = [dict(r) for r in rf.forwarded]
        hot = metrics.get("hot_apply") if isinstance(metrics.get("hot_apply"), dict) else {}
        rail_by_gen = {m.get("generation"): m for m in (hot.get("events") or [])
                       if isinstance(m, dict)}
        for r in forwarded:
            m = rail_by_gen.get(r.get("generation")) if not r.get("refused") else None
            if m is not None:
                r["rail"] = {k: m.get(k) for k in ("t", "op", "type", "rule_id",
                                                   "receipt_to_swap_ms") if k in m}
        fw_by_gen = {r["generation"]: r for r in forwarded
                     if not r.get("refused") and r.get("generation") is not None}

        # policy_bundle.tar.gz: every generation the run kept, and every one
        # an endpoint receipt named (a receipt with no IR beside it is a gap,
        # and the index says so).
        gens = []
        for n in sorted(set(held) | set(fw_by_gen)):
            fw, rl = fw_by_gen.get(n), rail_by_gen.get(n)
            h = (fw or {}).get("policy_hash") or (
                "sha256:" + hashlib.sha256(held[n]).hexdigest() if n in held else None)
            gens.append({"generation": n, "policy_hash": h,
                         "t": (0.0 if n == 0 and not fw and not rl else
                               (rl or {}).get("t") if rl and rl.get("t") is not None
                               else (fw or {}).get("t")),
                         "event": (fw or {}).get("event") or (rl or {}).get("type"),
                         "rule_id": (fw or {}).get("rule_id") or (rl or {}).get("rule_id")})
        src = Path(spec.policy.source)
        src = src if src.is_absolute() else ROOT / src
        try:
            cfg = self._hil_cfg() if e.backend == "hil" else None
            src = S.rail_policy_source(spec, e.backend, cfg.bundles_dir if cfg else None)
        except Exception:                                        # noqa: BLE001
            pass                 # the scenario's own reference, as resolved above
        pb = write_policy_bundle(ep_dir, src, None, None, gens, held=held)
        files[POLICY_BUNDLE_FILE] = ("written, complete" if pb["complete"]
                                     else f"written, incomplete: {pb['missing']}")

        # episode.bag2/ and shield_audit.jsonl
        got, bag_info = rail_bag_and_audit(run_dir, ep_dir)
        files.update(got)
        if e.keep_log and (run_dir / "flight_log.jsonl").is_file():
            shutil.copyfile(run_dir / "flight_log.jsonl", ep_dir / "flight_log.jsonl")
        replay_files = bundle_replay_inputs(run_dir, ep_dir)

        # harness_events.jsonl
        shield = "on" if spec.shield else "off"
        g0 = next((g["policy_hash"] for g in gens if g["generation"] == 0), None)
        last = max(held) if held else None
        events = [{"t": 0.0, "type": "episode_start", "scenario_id": spec.scenario_id,
                   "seed": ep.seed, "policy_hash": g0 or flown or e.policy_hash,
                   "generation": 0, "shield": shield, "backend": e.backend,
                   "rail_run": run.get("tag"), "rest_endpoint": rf.endpoint or None,
                   "endpoint_policy_hash": rf.endpoint_policy_hash or None,
                   "mission_t0_wall_time": rf.t0_wall,
                   "mission_t0_basis": (MISSION_T0_BASIS if rf.endpoint else None),
                   "scenario_events_via": rf.events_via}]
        events += forwarded
        events.append({"t": (rows[-1].get("t") if rows else None), "type": "episode_end",
                       "generation": last,
                       "policy_hash": ("sha256:" + hashlib.sha256(held[last]).hexdigest()
                                       if last is not None else None),
                       "mission_rc": run.get("rc"),
                       "forwarded_applied": len(fw_by_gen),
                       "forwarded_refused": sum(1 for r in forwarded
                                                if r.get("refused") and r.get("sent", True)),
                       "not_sent": len(rf.not_sent)})
        text("harness_events.jsonl", "".join(json.dumps(_json_safe(x), sort_keys=True) + "\n"
                                             for x in events))
        files["harness_events.jsonl"] = "written"

        # kpi.json
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
        files["kpi.json"] = "written"
        harness = {"episode_id": e.episode_id, "transport": "grpc", "client": e.client,
                   "profile": e.profile or None, "backend": e.backend,
                   "session_id": e.session_id or None, "live": False, "pace": None,
                   "task": task,
                   "rail": {"run": run_dir.as_posix(), "tag": run.get("tag"),
                            "mission_rc": run.get("rc"), "campaign": self._rail_campaign,
                            "hotapply_args": list(rf.hot_args),
                            "rest_endpoint": rf.endpoint or None,
                            "endpoint_policy_hash": rf.endpoint_policy_hash or None,
                            "endpoint_problem": rf.endpoint_problem or None,
                            "mission_t0_basis": MISSION_T0_BASIS if rf.endpoint else None,
                            "scenario_score": (Path(score_path).as_posix()
                                               if score_path else None),
                            "scored_by": "experiments/sweep_scenarios.py (score, "
                                         "rail_gate_set ideal, episode_verdict)",
                            # Who applied the scenario's own events: the
                            # mission (--events) or this server over REST
                            # (drive_scenario_events), each with its route.
                            "scenario_events_via": rf.events_via,
                            "scenario_events": [
                                {k: x[k] for k in ("index", "at_sim_t", "type", "payload",
                                                   "body", "method", "path")}
                                for x in rf.timeline] if rf.events_via == "harness"
                            else len(e.ep.cell.spec.events)},
                   "events_forwarded": [{k: r.get(k) for k in (
                       "queue_index", "source", "scenario_event", "scenario_event_index",
                       "t", "at_sim_t", "method", "url", "event", "op", "rule_id",
                       "generation", "policy_hash", "refused", "code", "reason", "rtt_ms")}
                       for r in forwarded],
                   "bundle_files": files, "episode_bag": bag_info,
                   "replay_files": replay_files}
        kpi_doc = _json_safe({
            **common, "status": status, "gates_failed": bad, "kpi_grade": bool(grade),
            "kpi_grade_reasons": why_not, "arm_record": e.arm_record,
            "rule_priorities": prios, "gates_basis": "ideal", "gates": gates,
            "metrics": {"reached_goal": extra.get("reached_goal"),
                        "breaches": extra.get("breaches"), "shield": shield,
                        "fsm": extra.get("fsm")},
            "kpi": res, "harness": harness})
        text("kpi.json", json.dumps(kpi_doc, indent=2, allow_nan=False) + "\n")

        extras: dict[str, Any] = {"backend": e.backend, "rail_run": run.get("tag"),
                                  "rail_run_path": run_dir.as_posix()}
        extras.update(S.manifest_extras(e.arm_record) or {})
        if e.task:
            extras.update(task_prompt_sha256=e.task["prompt_sha256"],
                          task_prompt_chars=e.task["chars"], task_prompt_source="IssueTask",
                          task_read_by_pilot=False)
        if e.session_id:
            extras["harness_session_id"] = e.session_id
        text("manifest_extras.json", json.dumps(_json_safe(extras), indent=2) + "\n")
        text("harness_request.json", json.dumps(_json_safe({
            "rpc": "StartEpisode", "request": e.request, "resolved": self._ref(e),
            "events": [q.received for q in e.queued], "client": e.client, "task": e.task,
            "manifest": manifest}), indent=2) + "\n")

        table = {**res, "id": ep.run_id, "family": spec.template, "cell": ep.episode_id,
                 "arm": shield, "prefix": ep.prefix, "params": ep.cell.params,
                 "bundle": ep_dir.as_posix(), "kpi_grade": bool(grade)}
        table["worst_tick"] = _json_safe(K.worst_tick(rows, prios))
        table["null_hover_mission"] = K.null_hover_mission(
            rows, {"reached_goal": extra.get("reached_goal")})
        if not spec.shield:
            table = _kpi_report().shield_off_table(table)
        rollup = K.rollup([table])
        applied = [{"t": float(r.get("t") or 0.0),
                    "event": str(r.get("event") or "") + (f"/{r['op']}" if r.get("op") else ""),
                    "rule_id": str(r.get("rule_id") or ""),
                    "refused": bool(r.get("refused", False)),
                    "reason": str(r.get("reason") or ""),
                    "generation": int(r.get("generation") or 0),
                    "policy_hash": str(r.get("policy_hash") or ""), "via": "rest",
                    "receipt_json": (json.dumps(_json_safe(r["receipt"]), sort_keys=True)
                                     if r.get("receipt") is not None else ""),
                    "source": str(r.get("source") or "")}
                   for r in forwarded]
        return {"backend": e.backend, "rail_run_path": run_dir.as_posix(),
                "bundle_files": dict(files),
                "episode_id": e.episode_id, "bundle_path": ep_dir.as_posix(),
                "status": status, "gates_failed": list(bad),
                "mission_outcome": str(res.get("mission_outcome") or ""),
                "kpi": kpi_row(rollup, spec.template, ep.episode_id, shield),
                "kpi_json": json.dumps(_json_safe(res), sort_keys=True, allow_nan=False),
                "rollup_json": json.dumps(_json_safe(rollup), sort_keys=True,
                                          allow_nan=False),
                "manifest_json": json.dumps(_json_safe(manifest or {}), sort_keys=True),
                "kpi_grade": bool(grade), "kpi_grade_reasons": list(why_not),
                "applied_events": applied, "session_id": e.session_id,
                "task_prompt_sha256": e.task["prompt_sha256"] if e.task else "",
                "live": False}

    def wait_stream(self, session_id: str, which: str, index: int,
                    timeout: float) -> tuple[list[dict], int, bool]:
        """The session's events or telemetry from `index` on, waiting up to
        `timeout` for one to arrive. Returns (items, next index, finished):
        finished once the session is STOPPED (not merely stopping: a flight
        still appending) and nothing is left to send."""
        s = self._session(session_id)
        if s.state == "OPEN":
            s.idle_since = time.monotonic()          # a client is still reading it
        with s.cond:
            buf = s.events if which == "events" else s.telemetry
            if index >= len(buf) and s.state != "STOPPED":
                s.cond.wait(timeout)
            items = buf[index:]
            nxt = index + len(items)
            return list(items), nxt, s.state == "STOPPED" and nxt >= len(buf)

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

        sspec = S.scoring_spec(spec, ep)      # the sweep's (a consumer's prefix-on arm)
        res = S.score(sspec, run.rows, run.extra, run.policy, run.priorities)
        # The sweep's own verdict function, the one its main() calls (until
        # 2026-10-11 this module carried a copy of it).
        status, bad, labels_failed, known_why = S.episode_verdict(
            sspec, res, ep.cell.params, ep.seed)

        manifest = S.episode_manifest(self._manifest_base, run.policy_hash_start, ep.seed,
                                      run.sim_s, run.wall_s)
        grade, why_not = is_kpi_grade(manifest, {})
        stamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
        ep_dir = self.runs_dir / e.episode_id / ep.dir_name(stamp)
        # The grant's policy_bundle.tar.gz: the injected file, generation 0's
        # IR and the last generation's (write_policy_bundle).
        src = Path(spec.policy.source)
        try:
            start_pol, _ = spec.policy.load_with_source(ROOT)
        except Exception:                                        # noqa: BLE001
            start_pol = None              # said in the index: g0's IR is missing
        pb = write_policy_bundle(ep_dir, src if src.is_absolute() else ROOT / src,
                                 start_pol, run.policy, S.policy_generations(run.events),
                                 held=getattr(run, "generations", None))
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
                                       for q in queued],
                   # The grant's episode-bundle file names, and which of them
                   # this backend writes (Stress Testing, "Episode bundle").
                   "bundle_files": {
                       POLICY_BUNDLE_FILE: ("written, complete" if pb["complete"]
                                            else f"written, incomplete: {pb['missing']}"),
                       "shield_audit.jsonl": "written",
                       "episode.bag2/": NO_EPISODE_BAG}}
        kpi_doc = _json_safe({
            **common, "status": status, "gates_failed": bad, "kpi_grade": grade,
            "kpi_grade_reasons": why_not, "arm_record": arm_record,
            "rule_priorities": run.priorities,
            "metrics": {"reached_goal": run.extra.get("reached_goal"),
                        "breaches": run.extra.get("breaches"), "shield": shield,
                        "fsm": run.extra.get("fsm")},
            "kpi": res, "harness": harness})
        S.write_bundle(ep_dir, manifest, kpi_doc, run.events,
                       run.rows if e.keep_log else None, self._extras(e, spec, run),
                       audit=run.audit)
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
                            "policy_hash": str(ev.get("policy_hash") or ""),
                            "via": "runner"})
        return {"backend": "headless", "bundle_files": dict(harness["bundle_files"]),
                "episode_id": e.episode_id, "bundle_path": ep_dir.as_posix(),
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

def _struct_to_dict(msg, what: str = "body") -> dict:
    """A google.protobuf.Struct as a dict. A NaN or Infinity number_value (a
    proto3 double carries one, and Struct.update stores it without
    complaint) makes json_format refuse the message with ValueError: that is
    the client's input failing validation, INVALID_ARGUMENT, not a server
    fault. Until 2026-10-19 it surfaced as INTERNAL."""
    M = _grpc_modules()
    if msg is None:
        return {}
    try:
        return M.json_format.MessageToDict(msg)
    except ValueError as e:
        raise HarnessError("INVALID_ARGUMENT", f"{what}: {e}") from None


_BACKEND_NAME = {0: "headless", 1: "headless", 2: "sitl", 3: "hil"}
_BACKEND_ENUM = {"headless": 1, "sitl": 2, "hil": 3}


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
                                         _struct_to_dict(request.body, "body"),
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
                pace=request.pace if request.HasField("pace") else None,
                drive_scenario_events=request.drive_scenario_events)
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
                                          _struct_to_dict(request.payload, "payload"),
                                          now=request.now)
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


def paraphraser_client(address: str, *, pki_dir: str | Path | None = None,
                       client_name: str = DEFAULT_CLIENT_CN):
    """A client of the paraphraser service for `Harness(paraphraser=...)`:
    mutual TLS with the lab PKI's `client_name` certificate (the control
    plane's one CA, as the service's own `--pki`), or, without `pki_dir`,
    plaintext - which only a loopback address may use; the service refuses
    plaintext anywhere else too."""
    host = address.rsplit(":", 1)[0] if ":" in address else address
    if pki_dir is None and not is_loopback(host):
        raise ValueError(f"paraphraser {address}: plaintext is for loopback only; give "
                         f"the lab PKI for mutual TLS")
    try:
        from .paraphraser_service import ParaphraserClient
        return ParaphraserClient(address, pki_dir=pki_dir, client_name=client_name)
    except ImportError as e:
        raise HarnessUnavailable(f"the paraphraser client needs grpcio: {e}") from e
    except Exception as e:                       # noqa: BLE001 - ServiceError and kin
        if isinstance(e, (FileNotFoundError, ValueError)):
            raise
        raise HarnessUnavailable(f"{type(e).__name__}: {e}") from e


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


POLICY_SIGNER_CN = "policy-signer"


def _validity(cert) -> tuple[_dt.datetime, _dt.datetime]:
    """(not before, not after) of a certificate as aware UTC datetimes, on
    cryptography 41 (naive properties only) and 42+ (the *_utc ones)."""
    nb = getattr(cert, "not_valid_before_utc", None)
    na = getattr(cert, "not_valid_after_utc", None)
    if nb is None or na is None:
        nb = cert.not_valid_before.replace(tzinfo=_dt.timezone.utc)
        na = cert.not_valid_after.replace(tzinfo=_dt.timezone.utc)
    return nb, na


def issue_policy_signer(pki_dir: str | Path, ed25519_public, *,
                        name: str | None = None) -> bytes:
    """The certificate (PEM) that puts a policy-bundle signing key under the
    lab root CA in `pki_dir`: what makes a bundle's `signed_by` a "Lab CA
    reference" (grant, Policy DSL page) instead of a bare key label.

    `ed25519_public` is the PUBLIC half of the bundle key - its 32 raw bytes,
    or a cryptography Ed25519PublicKey. Nothing private is asked for, created
    or written: the AGX Orin's key is certified from the public key the trust
    store already holds, and its private key never leaves the Orin. The CA's
    private key is read from `pki_dir` (it exists on the desktop only) and the
    certificate is returned, not written: it is public data, and
    guardrail.bundle records it in the trust store.

    The leaf: subject CN `policy-signer-<first 16 hex of sha256(public key)>`
    (the bundle's key id; `name` overrides it), basicConstraints CA:FALSE,
    keyUsage digitalSignature, extendedKeyUsage codeSigning only (it is not a
    TLS identity: the harness allow-list never sees it, and a TLS leaf is not
    accepted as a bundle signer). notBefore is the CA's own notBefore (a
    verifier with a clock behind the issuer's still accepts it), and notAfter
    is the CA's own notAfter, not 365
    days: a bundle and the KPI runs flown under it are evidence that has to
    verify for as long as its authority does, and a replay bundle states no
    issue time against which an expired leaf could be excused. When the CA is
    replaced, the new one is a new authority entry and a new leaf."""
    C = _crypto()
    from cryptography.hazmat.primitives.asymmetric import ed25519
    if isinstance(ed25519_public, (bytes, bytearray)):
        if len(ed25519_public) != 32:
            raise ValueError("an Ed25519 public key is 32 bytes")
        pub = ed25519.Ed25519PublicKey.from_public_bytes(bytes(ed25519_public))
    elif isinstance(ed25519_public, ed25519.Ed25519PublicKey):
        pub = ed25519_public
    else:
        raise TypeError("ed25519_public: 32 raw bytes or an Ed25519PublicKey (the "
                        "public half; a private key is never passed here)")
    raw = pub.public_bytes(C.serialization.Encoding.Raw, C.serialization.PublicFormat.Raw)
    cn = name or f"{POLICY_SIGNER_CN}-{hashlib.sha256(raw).hexdigest()[:16]}"
    if not _NAME.match(cn):
        raise ValueError(f"certificate name {cn!r}: letters, digits, . _ - only")
    d = Path(pki_dir)
    ca_cert = C.x509.load_pem_x509_certificate((d / "ca.cert.pem").read_bytes())
    ca_key = C.serialization.load_pem_private_key((d / "ca.key.pem").read_bytes(),
                                                  password=None)
    spki = (C.serialization.Encoding.DER, C.serialization.PublicFormat.SubjectPublicKeyInfo)
    if ca_key.public_key().public_bytes(*spki) != ca_cert.public_key().public_bytes(*spki):
        raise ValueError(f"{d / 'ca.key.pem'} is not the key of {d / 'ca.cert.pem'}")
    ca_from, ca_until = _validity(ca_cert)
    now = _dt.datetime.now(_dt.timezone.utc)
    if not ca_from <= now < ca_until:
        raise ValueError(f"the CA in {d} is valid {ca_from:%Y-%m-%d} to "
                         f"{ca_until:%Y-%m-%d}: it signs nothing today")
    cert = (C.x509.CertificateBuilder()
            .subject_name(_name(C, cn)).issuer_name(ca_cert.subject)
            .public_key(pub).serial_number(C.x509.random_serial_number())
            # From the CA's own notBefore, not "now minus 5 minutes": a
            # verifier whose clock runs behind the desktop's (the Orin, a
            # container) would otherwise refuse a freshly re-signed bundle as
            # "not valid now", which a bare pinned key never was. The leaf
            # claims nothing the CA did not already: same window, end to end.
            .not_valid_before(ca_from)
            .not_valid_after(ca_until)
            .add_extension(C.x509.BasicConstraints(ca=False, path_length=None), critical=True)
            .add_extension(_key_usage(C, ca=False), critical=True)
            .add_extension(C.x509.ExtendedKeyUsage([C.EKU.CODE_SIGNING]), critical=False)
            .add_extension(C.x509.SubjectKeyIdentifier.from_public_key(pub), critical=False)
            .add_extension(C.x509.AuthorityKeyIdentifier.from_issuer_public_key(
                ca_key.public_key()), critical=False)
            # an Ed25519 CA (not what generate_pki writes) signs without a digest
            .sign(ca_key, None if isinstance(ca_key, ed25519.Ed25519PrivateKey)
                  else C.hashes.SHA256()))
    return cert.public_bytes(C.serialization.Encoding.PEM)


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
    s.add_argument("--rest-pki", default=None,
                   help="a gen-pki folder: the lab CA and the client certificate this "
                        "server presents to an https REST endpoint (a rail's "
                        "--hotapply-pki listener); without it only http endpoints bind")
    s.add_argument("--rest-client", default=DEFAULT_CLIENT_CN,
                   help=f"the client certificate name in --rest-pki (default "
                        f"{DEFAULT_CLIENT_CN}; a dedicated one, e.g. issue --name "
                        f"harness-forwarder allowed on the rail with --hotapply-client, "
                        f"keeps gRPC clients from holding the rail's identity)")
    s.add_argument("--paraphraser", default=None, metavar="HOST:PORT",
                   help="the paraphraser service (python -m guardrail.paraphraser_service "
                        "serve) every paraphrase arm is drawn from; mutual TLS with "
                        "--pki's client certificate, plaintext with --insecure (loopback "
                        "only). Without it this server draws no paraphrase arm")
    s.add_argument("--rail", action="append", default=None, choices=RAIL_BACKENDS,
                   help="fly rail sessions on this desktop's rig (repeat): sitl "
                        "(sitl/run_ros2_demo.sh in WSL) and/or hil (SITL in WSL, the "
                        "mission on the Orin), through experiments/sweep_scenarios.py's "
                        "own rail backends, one mission at a time. Without it a sitl / "
                        "hil session is UNIMPLEMENTED")
    s.add_argument("--hil-config", default=None, metavar="FILE",
                   help="--rail hil: the rig (the sweep's --hil-config; default: the "
                        "rig of 2026-11-06)")
    s.add_argument("--rail-store", default=None, metavar="DIR",
                   help="--rail hil: where fetched run folders go (default: the hil "
                        "config's store, else <runs>/rail_runs)")
    s.add_argument("--rail-campaign", default="",
                   help="prefix of every mission tag (default h<UTC month day>T<time>)")
    s.add_argument("--rail-controls", action="store_true",
                   help="also fly Shield-off control arms (the sweep's --hil-controls)")
    s.add_argument("--rail-hotapply-pki", default="", metavar="DIR",
                   help="the lab PKI folder AS THE MISSION SEES IT (on the Orin, inside "
                        "its container): its hot-apply listener serves mutual TLS from "
                        "it when a session's endpoint is off loopback")
    s.add_argument("--rail-hotapply-host", default="",
                   help="the address the mission's listener binds (default: the "
                        "session endpoint's host)")
    s.add_argument("--rail-hotapply-server-cert", default="", metavar="NAME",
                   help="the listener's server certificate name in that folder "
                        "(default harness-server; it must name the endpoint's host)")
    s.add_argument("--paraphraser-client", default=DEFAULT_CLIENT_CN,
                   help=f"the client certificate name in --pki this server presents to "
                        f"the paraphraser service (default {DEFAULT_CLIENT_CN}, the name "
                        f"the service allows by default)")
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
        para = None
        if args.paraphraser:
            para = paraphraser_client(args.paraphraser,
                                      pki_dir=None if args.insecure else args.pki,
                                      client_name=args.paraphraser_client)
        rig = None
        if args.rail:
            cfg = _sweep().HilConfig.load(args.hil_config) if "hil" in args.rail else None
            rig = RailRig(backends=tuple(args.rail), hil_config=cfg,
                          store=args.rail_store or (cfg.store if cfg else None),
                          campaign=args.rail_campaign, controls=args.rail_controls,
                          hotapply_pki=args.rail_hotapply_pki,
                          hotapply_host=args.rail_hotapply_host,
                          hotapply_server_cert=args.rail_hotapply_server_cert,
                          hotapply_clients=((args.rest_client,)
                                            if args.rest_client != DEFAULT_CLIENT_CN
                                            else ()))
        srv = serve(Harness(runs_dir=args.runs, library=args.library,
                            armed_idle_ttl_s=args.armed_ttl,
                            session_idle_ttl_s=args.session_ttl,
                            rest_hosts=tuple(args.allow_rest_host or ()),
                            rest_pki_dir=args.rest_pki,
                            rest_client_name=args.rest_client, paraphraser=para,
                            rail=rig),
                    host=args.host,
                    port=args.port, pki_dir=None if args.insecure else args.pki,
                    allowed_clients=tuple(args.allow_client or (DEFAULT_CLIENT_CN,)))
    except (HarnessUnavailable, FileExistsError, FileNotFoundError, ValueError,
            RuntimeError) as e:
        print(f"{type(e).__name__}: {e}", file=sys.stderr)
        return 2
    mode = "mutual TLS" if srv.secure else "PLAINTEXT (loopback only)"
    # Flushed: under a service manager stdout is a file, and an unflushed
    # "listening" line reads as a server that never came up.
    print(f"StressHarness listening on {srv.address}, {mode}; bundles -> {args.runs}; "
          f"paraphrase arms from "
          f"{'the paraphraser service at ' + args.paraphraser if para else 'nowhere (no --paraphraser: refused)'}",
          flush=True)
    try:
        srv.server.wait_for_termination()
    except KeyboardInterrupt:
        srv.stop(1.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
