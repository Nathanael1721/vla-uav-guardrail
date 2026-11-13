"""Client for the stress harness's gRPC control plane (guardrail/harness_service.py).

    python tools/harness_client.py run --scenario nfz-head-on --seed 1001
    python tools/harness_client.py run --scenario-file family.yaml --cell "fam@x=2" \\
        --hot-apply 3.0 zone.json --event 5.0 scale_radius '{"factor": 2}'
    python tools/harness_client.py run --scenario dyn-nfz-translate --profile smoke
    python tools/harness_client.py run --scenario nfz-head-on --seed 1001 \
        --policy-bundle bundles/x.tar.gz --expected-hash sha256:... \
        --prompt "fly east to the tower" --live --pace 10
    python tools/harness_client.py forward --rest-endpoint http://127.0.0.1:8071 zone.json
    python tools/harness_client.py drive --rest-endpoint http://127.0.0.1:8071 \
        --policy-bundle bundles/x.tar.gz --timeline events.yaml \
        --events-file demo/out/<tag>/events.jsonl --log demo/out/<tag>/<tag>.harness.jsonl
    python tools/harness_client.py --timeout 60 run --backend hil \
        --rest-endpoint https://10.42.0.3:8071 --scenario nfz-head-on --seed 1001 \
        --hot-apply 2.0 zone.json --hot-apply 8.0 translate.json
    python tools/harness_client.py status ep-0123456789ab
    python tools/harness_client.py --insecure --target 127.0.0.1:50051 run ...

Mutual TLS by default: the client trusts <pki>/ca.cert.pem for the server and
presents <pki>/<name>.cert.pem (default: harness-client in the folder
`python -m guardrail.harness_service gen-pki` writes, outside the checkout -
%LOCALAPPDATA%/vla-guardrail/harness_pki on Windows,
~/.local/share/vla-guardrail/harness_pki elsewhere). `--insecure` speaks
plaintext, which the server accepts on loopback only.

`run` arms an episode, sends every --hot-apply (a REST hot-apply body in a
JSON or YAML file, applied at the given simulated second) and --event (a
ScenarioEvent type and its JSON payload), flies it with EndEpisode and prints
the episode's row of the Stress Testing KPI report, each score beside its null,
and why any figure is unset. A figure the episode has no value for is printed
as unset with the reason, never as 0.

With --policy-bundle (and --expected-hash), --prompt or --live, `run` follows
the page's session lifecycle: StartSession, InjectPolicy, IssueTask,
StartEpisode in the session (live: flying at once, paced at --pace simulated
seconds per wall second, the --hot-apply / --event times then injected while
it flies), EndEpisode, StopSession; it prints how many telemetry frames and
events the session's streams carried. `forward` opens a session bound to a
REST hot-apply endpoint (guardrail/api.py) and sends one HotApply to it,
which the server POSTs to <endpoint>/hot_apply.

`run --backend sitl|hil` (2026-11-10, WP4-05) is one harness session = one
flown, scored episode: the session is opened on the server's rig (serve
--rail), StartEpisode LAUNCHES the mission through the sweep's own rail
backend, each --hot-apply is queued for its MISSION time and forwarded by the
server to the mission's own REST listener (--rest-endpoint names the address
it is started on), EndEpisode waits for the flight (--end-timeout, default one
hour), scores it with the sweep's rail scorer and returns the bundle under
the grant's names. A --hot-apply body may be an edit of a zone, which the
server sends as PATCH /dynamic_nfz/<id>: {"type": "dynamic_nfz", "op":
"translate", "id": "nfz-1", "dx": 5, "dy": 0} (rotate: angle_deg; scale:
factor). Since 2026-11-14 an --event on a rail is turned by the server into
the REST body that applies it (translate_polygon_fence {"id", "dx_m",
"dy_m"} as PATCH translate, rotate_polygon_fence {"id", "angle_deg"},
scale_radius {"id", "factor"}, a spawn with vertices, a switch, a swap) and
forwarded at its mission time; one with no REST form (a zone placed against
the vehicle, a change of motion) is UNIMPLEMENTED. --drive-scenario-events
has the server forward the scenario's OWN events over REST instead of the
mission applying them (tools/harness_hil_episode.py flies one such episode
and dry-runs it). The same bodies work in a `drive` timeline and with
`forward`.

`drive` (2026-11-03, WP4-03) is the harness driving a FLYING rail: it opens
a session bound to the rail's REST hot-apply endpoint (the Shield node's
--hotapply-port), injects the policy the rail flies (InjectPolicy checks the
endpoint's GET /health reports the same hash, and is retried while the
endpoint is not up yet), waits for the rail's own `mission_start` in its
events.jsonl, then sends each timeline entry as a session HotApply at its
mission time - the server POSTs it to <endpoint>/hot_apply, the grant's
"harness drives dynamic_nfz, time_window_switch, and corridor_swap events
via the Policy DSL's REST hot-apply endpoint" - and stops the session. The
session's event stream (StreamEvents) is read on a thread throughout. Every
request, acknowledgement and refusal, and every streamed event, is a line of
--log. The timeline is a JSON/YAML list of {"at": seconds after
mission_start, "body": a REST hot-apply body with its "type"}. What it does
NOT do: start the rail or its episode (StartEpisode's SITL backend is
reserved, UNIMPLEMENTED), or read the flight's telemetry (the rail's own
records hold it).

Exit codes: 0 the episode's status is pass, tracked or known_failure (the
sweep's meaning: reported, not failed); 1 fail or unexpected_pass; 2 the server
refused a call (the gRPC status code is printed) or the client could not start.
When a HotApply or EmitEvent is refused after StartEpisode, nothing flies and
the tool names the episode it leaves ARMED; the server expires it after its
idle TTL (default one hour), so it does not hold a slot for ever.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail import harness_service as H                       # noqa: E402

DEFAULT_TARGET = f"127.0.0.1:{H.DEFAULT_PORT}"
OK_STATUSES = ("pass", "tracked", "known_failure")


class HarnessClient:
    """The page's ten RPCs and GetStatus with plain Python arguments. Raises
    grpc.RpcError when the server refuses a call (`.code()`, `.details()`)."""

    def __init__(self, target: str = DEFAULT_TARGET, *, insecure: bool = False,
                 pki_dir: str | Path | None = None, name: str = H.DEFAULT_CLIENT_CN,
                 ca_cert: str | Path | None = None, cert: str | Path | None = None,
                 key: str | Path | None = None, timeout: float = 120.0):
        M = H._grpc_modules()
        self._M = M
        self.timeout = timeout
        if insecure:
            self.channel = M.grpc.insecure_channel(target)
        else:
            d_ca, d_cert, d_key = H.pki_paths(pki_dir or H.DEFAULT_PKI_DIR, name)
            creds = H.channel_credentials(ca_cert or d_ca,
                                          cert or (d_cert if name else None),
                                          key or (d_key if name else None))
            self.channel = M.grpc.secure_channel(target, creds)
        self.stub = M.pb_grpc.StressHarnessStub(self.channel)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def close(self) -> None:
        self.channel.close()

    def _struct(self, d: dict | None):
        s = self._M.struct_pb2.Struct()
        if d:
            s.update(d)
        return s

    # ---- session lifecycle and policy ------------------------------------ #

    def start_session(self, *, backend: str = "headless", scene: str = "",
                      airframe: str = "", seed: int | None = None,
                      rest_endpoint: str = "", label: str = ""):
        pb = self._M.pb
        kw = {"backend": {"headless": pb.BACKEND_HEADLESS, "sitl": pb.BACKEND_SITL,
                          "hil": pb.BACKEND_HIL}
              .get(backend, pb.BACKEND_UNSPECIFIED), "scene": scene, "airframe": airframe,
              "rest_endpoint": rest_endpoint, "label": label}
        if seed is not None:
            kw["seed"] = int(seed)
        return self.stub.StartSession(pb.StartSessionRequest(**kw), timeout=self.timeout)

    def stop_session(self, session_id: str):
        return self.stub.StopSession(self._M.pb.SessionRef(session_id=session_id),
                                     timeout=self.timeout)

    def inject_policy(self, session_id: str, *, bundle_path: str = "",
                      bundle_bytes: bytes = b"", expected_hash: str = ""):
        pb = self._M.pb
        kw = {"session_id": session_id, "expected_hash": expected_hash}
        if bundle_path:
            kw["bundle_path"] = bundle_path
        if bundle_bytes:
            kw["bundle_tar_gz"] = bundle_bytes
        return self.stub.InjectPolicy(pb.InjectPolicyRequest(**kw), timeout=self.timeout)

    def issue_task(self, prompt: str, *, session_id: str = "", episode_id: str = ""):
        pb = self._M.pb
        return self.stub.IssueTask(pb.IssueTaskRequest(session_id=session_id,
                                                       episode_id=episode_id,
                                                       natural_language_prompt=prompt),
                                   timeout=self.timeout)

    def stream_telemetry(self, session_id: str, timeout: float | None = None):
        """An iterator of TelemetryFrame until the session stops."""
        return self.stub.StreamTelemetry(self._M.pb.SessionRef(session_id=session_id),
                                         timeout=timeout)

    def stream_events(self, session_id: str, timeout: float | None = None):
        """An iterator of HarnessEvent until the session stops."""
        return self.stub.StreamEvents(self._M.pb.SessionRef(session_id=session_id),
                                      timeout=timeout)

    # ---- episodes ---------------------------------------------------------- #

    def start_episode(self, *, scenario_id: str = "", scenario_yaml: str = "",
                      cell_id: str = "", seed: int | None = None, profile: str = "",
                      arm: str = "", keep_flight_log: bool = False,
                      session_id: str = "", live: bool = False,
                      pace: float | None = None, drive_scenario_events: bool = False):
        """`drive_scenario_events` (a rail session bound to the mission's
        listener, 2026-11-14): the harness forwards the scenario's own events
        over REST instead of the mission applying them from --events."""
        pb = self._M.pb
        kw = {"cell_id": cell_id, "profile": profile, "arm": arm,
              "keep_flight_log": keep_flight_log, "session_id": session_id, "live": live,
              "drive_scenario_events": bool(drive_scenario_events)}
        if scenario_id:
            kw["scenario_id"] = scenario_id
        if scenario_yaml:
            kw["scenario_yaml"] = scenario_yaml
        if seed is not None:
            kw["seed"] = int(seed)
        if pace is not None:
            kw["pace"] = float(pace)
        return self.stub.StartEpisode(pb.StartEpisodeRequest(**kw), timeout=self.timeout)

    def hot_apply(self, episode_id: str, at_sim_t: float, body: dict, *,
                  session_id: str = "", now: bool = False):
        """To an episode (at_sim_t, or `now` on a live one), or with
        `session_id` (and episode_id "") forwarded to the session's REST
        endpoint."""
        pb = self._M.pb
        return self.stub.HotApply(pb.HotApplyRequest(episode_id=episode_id,
                                                     at_sim_t=float(at_sim_t),
                                                     body=self._struct(body),
                                                     session_id=session_id, now=now),
                                  timeout=self.timeout)

    def emit_event(self, episode_id: str, at_sim_t: float, type_: str,
                   payload: dict | None = None, *, now: bool = False):
        pb = self._M.pb
        return self.stub.EmitEvent(pb.EmitEventRequest(episode_id=episode_id,
                                                       at_sim_t=float(at_sim_t),
                                                       type=type_,
                                                       payload=self._struct(payload),
                                                       now=now),
                                   timeout=self.timeout)

    def status(self, episode_id: str):
        return self.stub.GetStatus(self._M.pb.EpisodeRef(episode_id=episode_id),
                                   timeout=self.timeout)

    def end_episode(self, episode_id: str, timeout: float | None = None):
        """`timeout`: a rail episode's EndEpisode blocks for the rest of the
        mission, its fetch and its scoring (minutes)."""
        return self.stub.EndEpisode(self._M.pb.EpisodeRef(episode_id=episode_id),
                                    timeout=self.timeout if timeout is None else timeout)


# --------------------------------------------------------------------------- #
# printing
# --------------------------------------------------------------------------- #

# (label, score field, [(null label, null field), ...])
_ROWS = (
    ("P0 escape rate", "p0_escape_rate",
     [("passthrough", "null_passthrough_p0_escape_rate")]),
    ("fail-safe correctness", "fail_safe_correctness",
     [("never trigger", "null_never_trigger"), ("always trigger", "null_always_trigger")]),
    ("mission success rate", "mission_success_rate",
     [("hover", "null_hover_mission_success_rate")]),
    ("repair success rate", "repair_success_rate",
     [("always brake", "null_always_brake_repair_success_rate")]),
    ("mean repair count", "mean_repair_count", []),
    ("mean repair magnitude, m/s", "mean_repair_magnitude_mps", []),
    ("mean time to safe, s", "mean_time_to_safe_s", []),
)


def _value(kpi, name: str) -> str:
    if kpi.HasField(name):
        v = getattr(kpi, name)
        return f"{v:.6g}"
    return f"unset ({kpi.unset.get(name, 'no value')})"


def render_kpi(kpi) -> str:
    """The episode's KPI-report row: each score, then its null(s). An unset
    figure prints as unset with its reason."""
    lines = [f"KPI row: {kpi.family} / {kpi.cell}, Shield {kpi.shield}, "
             f"{kpi.episodes} episode(s)"]
    for label, name, nulls in _ROWS:
        line = f"  {label:<28} {_value(kpi, name):<34}"
        if nulls:
            line += "null " + ", ".join(f"{nl} {_value(kpi, nf)}" for nl, nf in nulls)
        lines.append(line.rstrip())
    return "\n".join(lines)


def render_bundle(b) -> str:
    out = [f"episode {b.episode_id}: {b.status} (outcome {b.mission_outcome or '-'})",
           f"bundle: {b.bundle_path}"]
    for g in b.gates_failed:
        out.append(f"  gate failed: {g}")
    if b.rail_run_path:
        out.append(f"{b.backend} rail run: {b.rail_run_path}")
    for a in b.applied_events:
        what = f"REFUSED {a.reason}" if a.refused else \
            f"generation {a.generation} {a.policy_hash[:19]}"
        out.append(f"  t={a.t:.2f} s {a.event} {a.rule_id}"
                   + (f" (over {a.via})" if a.via and a.via != "runner" else "")
                   + f": {what}" + (f" [from {a.source}]" if a.source else ""))
    for name in sorted(b.bundle_files):
        out.append(f"  {name:<22} {b.bundle_files[name]}")
    out.append(render_kpi(b.kpi))
    if not b.kpi_grade:
        out.append("NOT KPI-grade: " + "; ".join(b.kpi_grade_reasons))
    return "\n".join(out)


# --------------------------------------------------------------------------- #
# command line
# --------------------------------------------------------------------------- #

def _load_body(path: str) -> dict:
    """A hot-apply body file. Every way it can be wrong raises OSError or
    ValueError (yaml.YAMLError is not a ValueError: it is wrapped), so the
    pre-flight block in main() answers "cannot start", exit 2."""
    import yaml
    try:
        doc = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except yaml.YAMLError as e:
        raise ValueError(f"{path}: not valid JSON/YAML: {e}") from None
    if not isinstance(doc, dict):
        raise ValueError(f"{path}: a hot-apply body is one JSON/YAML object")
    return doc


def _load_timeline(path: str) -> list[dict]:
    """A drive timeline: a list of {"at": seconds after mission_start,
    "body": a hot-apply body with its type}, sorted by time."""
    import yaml
    try:
        doc = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except yaml.YAMLError as e:
        raise ValueError(f"{path}: not valid JSON/YAML: {e}") from None
    if not isinstance(doc, list) or not doc:
        raise ValueError(f"{path}: a timeline is a non-empty list of {{at, body}}")
    out = []
    for i, e in enumerate(doc):
        if not isinstance(e, dict) or not isinstance(e.get("body"), dict) \
                or not e["body"].get("type"):
            raise ValueError(f"{path} entry {i}: needs a body object with a type")
        at = float(e.get("at"))
        if not at >= 0.0:
            raise ValueError(f"{path} entry {i}: at must be >= 0 s")
        out.append({"at": at, "body": e["body"], "label": e.get("label", "")})
    return sorted(out, key=lambda e: e["at"])


def wait_for_event(path: Path, kind: str, timeout_s: float, poll_s: float = 0.05):
    """The first `kind` event appended to a rail's events.jsonl, or None. A
    file that a new episode rotates aside and recreates is followed."""
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        try:
            with open(path, encoding="utf-8") as fh:
                for line in fh:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        rec = json.loads(line)
                    except ValueError:
                        continue
                    if rec.get("kind") == kind:
                        return rec
        except OSError:
            pass
        time.sleep(poll_s)
    return None


def drive(client: "HarnessClient", args, timeline: list[dict]) -> int:
    """The `drive` command: a session bound to a flying rail's REST endpoint,
    its policy injected, the timeline sent as HotApply at mission time."""
    from google.protobuf import json_format
    grpc = H._grpc_modules().grpc
    log_path = Path(args.log)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    fh = open(log_path, "a", encoding="utf-8", newline="\n")
    lock = threading.Lock()

    def note(kind: str, **data) -> dict:
        rec = {"wall": round(time.time(), 4), "kind": kind, **data}
        with lock:
            fh.write(json.dumps(rec, default=str) + "\n")
            fh.flush()
        return rec

    def msg(m) -> dict:
        return json_format.MessageToDict(m, preserving_proto_field_name=True)

    note("drive_start", target=args.target, rest_endpoint=args.rest_endpoint,
         policy_bundle=args.policy_bundle, timeline=timeline, insecure=args.insecure)
    ref = client.start_session(rest_endpoint=args.rest_endpoint, label=args.label)
    session = ref.session_id
    note("StartSession", ack=msg(ref))
    print(f"session {session} bound to {args.rest_endpoint}")
    stream_n = {"n": 0}

    def read_stream():
        try:
            for ev in client.stream_events(session, timeout=args.wait_s + 600):
                stream_n["n"] += 1
                note("StreamEvents", event=msg(ev))
        except grpc.RpcError as e:
            note("StreamEvents_end", code=e.code().name, details=e.details())
    th = threading.Thread(target=read_stream, daemon=True)
    th.start()
    rc = 0
    try:
        # InjectPolicy reads the endpoint's /health: retried until the
        # rail's listener is up (it starts on the pad, before arming).
        end = time.monotonic() + args.wait_s
        while True:
            try:
                ack = client.inject_policy(session, bundle_path=args.policy_bundle,
                                           expected_hash=args.expected_hash)
                note("InjectPolicy", ack=msg(ack))
                print(f"  policy {ack.policy_hash} = the endpoint's "
                      f"{ack.endpoint_policy_hash} (signature {ack.signature})")
                break
            except grpc.RpcError as e:
                if e.code().name == "UNAVAILABLE" and time.monotonic() < end:
                    time.sleep(1.0)
                    continue
                note("InjectPolicy_refused", code=e.code().name, details=e.details())
                raise
        start = wait_for_event(Path(args.events_file), "mission_start",
                               max(0.0, end - time.monotonic()))
        if start is None:
            note("no_mission_start", events_file=args.events_file)
            print(f"no mission_start in {args.events_file} within {args.wait_s:g} s",
                  file=sys.stderr)
            return 2
        t0 = time.monotonic()
        note("mission_start_seen", event_wall=start.get("wall"))
        print("  mission_start seen")
        for e in timeline:
            # Never early: a sleep may wake before the monotonic clock says
            # the time has come (Windows' timer granularity).
            while True:
                delay = t0 + e["at"] - time.monotonic()
                if delay <= 0:
                    break
                time.sleep(min(delay, 0.05))
            sent = time.monotonic()
            try:
                ack = client.hot_apply("", 0.0, e["body"], session_id=session)
                rtt = time.monotonic() - sent
                note("HotApply", at=e["at"], mission_t=round(sent - t0, 3),
                     label=e["label"], body=e["body"], ack=msg(ack), rtt_s=round(rtt, 4))
                print(f"  t={sent - t0:6.2f} s HotApply {ack.type} {ack.rule_id} ({ack.op}) "
                      f"-> {ack.forwarded_to}: generation {ack.generation}, "
                      f"{rtt * 1e3:.1f} ms")
            except grpc.RpcError as err:
                rc = 1
                note("HotApply_refused", at=e["at"], mission_t=round(sent - t0, 3),
                     label=e["label"], body=e["body"], code=err.code().name,
                     details=err.details())
                print(f"  t={sent - t0:6.2f} s HotApply REFUSED {err.code().name}: "
                      f"{err.details()}", file=sys.stderr)
        time.sleep(1.0)
    finally:
        try:
            stop = client.stop_session(session)
            note("StopSession", ack=msg(stop))
            print(f"session {session} stopped: {stop.events} events")
        except grpc.RpcError as e:
            note("StopSession_refused", code=e.code().name, details=e.details())
        th.join(timeout=5.0)
        note("drive_end", streamed_events=stream_n["n"], rc=rc)
        fh.close()
    return rc


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--target", default=DEFAULT_TARGET)
    ap.add_argument("--insecure", action="store_true", help="plaintext (loopback servers)")
    ap.add_argument("--pki", default=str(H.DEFAULT_PKI_DIR))
    ap.add_argument("--name", default=H.DEFAULT_CLIENT_CN, help="client certificate name")
    ap.add_argument("--timeout", type=float, default=120.0)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="start, send events, end; print the KPI row")
    src = r.add_mutually_exclusive_group(required=True)
    src.add_argument("--scenario", help="a scenario_id of the server's library")
    src.add_argument("--scenario-file", help="one ScenarioSpec family (YAML)")
    r.add_argument("--cell", default="")
    r.add_argument("--seed", type=int, default=None)
    r.add_argument("--profile", default="")
    r.add_argument("--arm", default="")
    r.add_argument("--keep-log", action="store_true")
    r.add_argument("--hot-apply", nargs=2, action="append", default=[],
                   metavar=("AT_SIM_T", "BODY_FILE"))
    r.add_argument("--event", nargs=3, action="append", default=[],
                   metavar=("AT_SIM_T", "TYPE", "PAYLOAD_JSON"))
    r.add_argument("--json", action="store_true", help="print the bundle as JSON")
    r.add_argument("--policy-bundle", default="",
                   help="InjectPolicy: a bundle or policy YAML under the server's "
                        "policies/ or bundles/ (opens a session)")
    r.add_argument("--expected-hash", default="", help="the policy hash it must have")
    r.add_argument("--prompt", default="", help="IssueTask: the natural-language task "
                                                "(opens a session)")
    r.add_argument("--live", action="store_true",
                   help="fly the episode as it starts, events injected while it flies")
    r.add_argument("--backend", default="headless", choices=H.BACKENDS,
                   help="sitl or hil: fly it on the server's rig (serve --rail); "
                        "StartEpisode launches the mission and EndEpisode waits for it")
    r.add_argument("--rest-endpoint", default="",
                   help="a rail backend: the address the mission's own hot-apply "
                        "listener is started on (each --hot-apply is forwarded there "
                        "at its mission time)")
    r.add_argument("--end-timeout", type=float, default=None,
                   help="seconds EndEpisode may take (default: --timeout; one hour on "
                        "a rail backend)")
    r.add_argument("--pace", type=float, default=None,
                   help="live: simulated seconds per wall second (default 1.0; 0 unpaced)")
    r.add_argument("--drive-scenario-events", action="store_true",
                   help="a rail backend: the harness forwards the scenario's own events "
                        "over REST at their mission times (the mission flies none of "
                        "them from --events); needs --rest-endpoint")
    f = sub.add_parser("forward", help="send one HotApply through a session bound to a "
                                       "REST hot-apply endpoint")
    f.add_argument("--rest-endpoint", required=True)
    f.add_argument("body_file")
    s = sub.add_parser("status", help="GetStatus of an episode")
    s.add_argument("episode_id")
    d = sub.add_parser("drive", help="drive a flying rail: a session bound to its REST "
                                     "endpoint, a timeline of HotApply at mission time")
    d.add_argument("--rest-endpoint", required=True)
    d.add_argument("--policy-bundle", required=True,
                   help="the bundle the rail flies, under the server's bundles/")
    d.add_argument("--expected-hash", default="")
    d.add_argument("--timeline", required=True,
                   help="JSON/YAML list of {at: s after mission_start, body: {type, ...}}")
    d.add_argument("--events-file", required=True,
                   help="the rail's events.jsonl, watched for mission_start")
    d.add_argument("--log", required=True, help="JSONL of every request, answer and "
                                                "streamed event")
    d.add_argument("--wait-s", type=float, default=900.0,
                   help="how long to wait for the endpoint and the mission start")
    d.add_argument("--label", default="")
    args = ap.parse_args(argv)

    # Every file and argument is read and checked HERE, before any call: a
    # usage error is "cannot start" (exit 2), never a traceback with exit 1
    # (which reads as a failed episode), and never a refusal after
    # StartEpisode that leaves an episode ARMED. Until 2026-10-19 `forward`
    # loaded its body inside the gRPC block (a missing file: traceback, exit
    # 1), malformed YAML escaped as yaml.YAMLError, and an --event payload
    # that was not an object failed in Struct.update after StartEpisode. The
    # client is built last, so a refusal here leaves no channel open.
    try:
        fwd_body = _load_body(args.body_file) if args.cmd == "forward" else None
        timeline = _load_timeline(args.timeline) if args.cmd == "drive" else None
        hot = [(float(at), _load_body(p)) for at, p in args.hot_apply] \
            if args.cmd == "run" else []
        evs = [(float(at), t, json.loads(pl)) for at, t, pl in args.event] \
            if args.cmd == "run" else []
        for _, t, pl in evs:
            if pl is not None and not isinstance(pl, dict):     # null: no payload
                raise ValueError(f"--event {t}: PAYLOAD_JSON must be a JSON object, "
                                 f"got {type(pl).__name__}")
        if args.cmd == "run" and args.expected_hash and not args.policy_bundle:
            raise ValueError("--expected-hash goes with --policy-bundle")
        rail = args.cmd == "run" and args.backend in H.RAIL_BACKENDS
        if args.cmd == "run" and args.rest_endpoint and not rail:
            raise ValueError("--rest-endpoint goes with --backend sitl or hil (drive and "
                             "forward bind a session to a rail flown outside the harness)")
        if rail and (args.live or args.pace is not None):
            raise ValueError("--live / --pace are the headless backend's: a rail flies "
                             "in real time")
        if rail and (hot or evs or args.drive_scenario_events) and not args.rest_endpoint:
            raise ValueError("--hot-apply, --event and --drive-scenario-events on a rail "
                             "need --rest-endpoint: the mission's listener is started on it")
        if args.cmd == "run" and args.drive_scenario_events and not rail:
            raise ValueError("--drive-scenario-events goes with --backend sitl or hil")
        yml = (Path(args.scenario_file).read_text(encoding="utf-8")
               if args.cmd == "run" and args.scenario_file else "")
        client = HarnessClient(args.target, insecure=args.insecure, pki_dir=args.pki,
                               name=args.name, timeout=args.timeout)
    except (H.HarnessUnavailable, OSError, ValueError) as e:
        print(f"cannot start: {type(e).__name__}: {e}", file=sys.stderr)
        return 2
    grpc = H._grpc_modules().grpc
    armed = None                     # an episode started and not yet sent to EndEpisode
    session = None                   # a session this run opened
    try:
        with client:
            if args.cmd == "status":
                st = client.status(args.episode_id)
                print(st)
                return 0
            if args.cmd == "drive":
                return drive(client, args, timeline)
            if args.cmd == "forward":
                session = client.start_session(rest_endpoint=args.rest_endpoint).session_id
                ack = client.hot_apply("", 0.0, fwd_body, session_id=session)
                print(f"forwarded to {ack.forwarded_to}: {ack.type} {ack.rule_id} "
                      f"({ack.op}), generation {ack.generation} {ack.policy_hash}")
                client.stop_session(session)
                return 0
            if args.policy_bundle or args.prompt or args.live or rail:
                session = client.start_session(seed=args.seed, backend=args.backend,
                                               rest_endpoint=args.rest_endpoint
                                               ).session_id
                print(f"session {session}"
                      + (f" on the {args.backend} rail" if rail else "")
                      + (f", endpoint {args.rest_endpoint}" if args.rest_endpoint else ""))
                if args.policy_bundle:
                    ack = client.inject_policy(session, bundle_path=args.policy_bundle,
                                               expected_hash=args.expected_hash)
                    print(f"  policy {ack.policy_hash} (generation {ack.generation}, "
                          f"signature {ack.signature})")
                if args.prompt:
                    ack = client.issue_task(args.prompt, session_id=session)
                    print(f"  task {ack.prompt_sha256} ({ack.note})")
            ref = client.start_episode(scenario_id=args.scenario or "", scenario_yaml=yml,
                                       cell_id=args.cell, seed=args.seed,
                                       profile=args.profile, arm=args.arm,
                                       keep_flight_log=args.keep_log,
                                       session_id=session or "", live=args.live,
                                       pace=args.pace if args.live else None,
                                       drive_scenario_events=args.drive_scenario_events)
            armed = None if args.live or rail else ref.episode_id
            print(f"started {ref.episode_id}: {ref.run_id} seed {ref.seed}"
                  + (f" profile {ref.profile}" if ref.profile else "")
                  + (" (live, flying)" if ref.live else "")
                  + (f" (launched on the {ref.backend} rail)" if rail else "")
                  + (", the scenario's own events forwarded by the harness over REST"
                     if ref.scenario_events_via == "harness" else "")
                  + f", ends at {ref.run_end_sim_t:.1f} s")
            for at, body in hot:
                ack = client.hot_apply(ref.episode_id, at, body)
                print(f"  hot-apply #{ack.queue_index} at {ack.at_sim_t} s: {ack.type} "
                      f"{ack.rule_id} ({ack.op})"
                      + (f" queued for {ack.forwarded_method} {ack.forwarded_to}"
                         if ack.pending else ""))
            for at, t, payload in evs:
                ack = client.emit_event(ref.episode_id, at, t, payload)
                print(f"  event #{ack.queue_index} at {ack.at_sim_t} s: {ack.type}"
                      + (f" queued for {ack.forwarded_method} {ack.forwarded_to} "
                         f"{ack.rest_body_json}" if ack.pending else ""))
            armed = None
            b = client.end_episode(ref.episode_id, timeout=(
                args.end_timeout if args.end_timeout is not None
                else 3600.0 if rail else None))
            if session:
                stop = client.stop_session(session)
                print(f"session {session} stopped: {stop.telemetry_frames} telemetry "
                      f"frames, {stop.events} events")
                session = None
    except grpc.RpcError as e:
        print(f"REFUSED {e.code().name}: {e.details()}", file=sys.stderr)
        if armed:
            print(f"episode {armed} was left ARMED: nothing flew, and the server "
                  f"expires it once it has been idle for its TTL", file=sys.stderr)
        if session:
            print(f"session {session} was left OPEN", file=sys.stderr)
        return 2
    if args.json:
        from google.protobuf import json_format
        print(json_format.MessageToJson(b))
    else:
        print(render_bundle(b))
    return 0 if b.status in OK_STATUSES else 1


if __name__ == "__main__":
    sys.exit(main())
