"""Client for the stress harness's gRPC control plane (guardrail/harness_service.py).

    python tools/harness_client.py run --scenario nfz-head-on --seed 1001
    python tools/harness_client.py run --scenario-file family.yaml --cell "fam@x=2" \\
        --hot-apply 3.0 zone.json --event 5.0 scale_radius '{"factor": 2}'
    python tools/harness_client.py run --scenario dyn-nfz-translate --profile smoke
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
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail import harness_service as H                       # noqa: E402

DEFAULT_TARGET = f"127.0.0.1:{H.DEFAULT_PORT}"
OK_STATUSES = ("pass", "tracked", "known_failure")


class HarnessClient:
    """The five RPCs with plain Python arguments. Raises grpc.RpcError when
    the server refuses a call (`.code()`, `.details()`)."""

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

    def start_episode(self, *, scenario_id: str = "", scenario_yaml: str = "",
                      cell_id: str = "", seed: int | None = None, profile: str = "",
                      arm: str = "", keep_flight_log: bool = False):
        pb = self._M.pb
        kw = {"cell_id": cell_id, "profile": profile, "arm": arm,
              "keep_flight_log": keep_flight_log}
        if scenario_id:
            kw["scenario_id"] = scenario_id
        if scenario_yaml:
            kw["scenario_yaml"] = scenario_yaml
        if seed is not None:
            kw["seed"] = int(seed)
        return self.stub.StartEpisode(pb.StartEpisodeRequest(**kw), timeout=self.timeout)

    def hot_apply(self, episode_id: str, at_sim_t: float, body: dict):
        pb = self._M.pb
        return self.stub.HotApply(pb.HotApplyRequest(episode_id=episode_id,
                                                     at_sim_t=float(at_sim_t),
                                                     body=self._struct(body)),
                                  timeout=self.timeout)

    def emit_event(self, episode_id: str, at_sim_t: float, type_: str,
                   payload: dict | None = None):
        pb = self._M.pb
        return self.stub.EmitEvent(pb.EmitEventRequest(episode_id=episode_id,
                                                       at_sim_t=float(at_sim_t),
                                                       type=type_,
                                                       payload=self._struct(payload)),
                                   timeout=self.timeout)

    def status(self, episode_id: str):
        return self.stub.GetStatus(self._M.pb.EpisodeRef(episode_id=episode_id),
                                   timeout=self.timeout)

    def end_episode(self, episode_id: str):
        return self.stub.EndEpisode(self._M.pb.EpisodeRef(episode_id=episode_id),
                                    timeout=self.timeout)


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
    for a in b.applied_events:
        what = f"REFUSED {a.reason}" if a.refused else \
            f"generation {a.generation} {a.policy_hash[:19]}"
        out.append(f"  t={a.t:.2f} s {a.event} {a.rule_id}: {what}")
    out.append(render_kpi(b.kpi))
    if not b.kpi_grade:
        out.append("NOT KPI-grade: " + "; ".join(b.kpi_grade_reasons))
    return "\n".join(out)


# --------------------------------------------------------------------------- #
# command line
# --------------------------------------------------------------------------- #

def _load_body(path: str) -> dict:
    import yaml
    doc = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(doc, dict):
        raise ValueError(f"{path}: a hot-apply body is one JSON/YAML object")
    return doc


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
    s = sub.add_parser("status", help="GetStatus of an episode")
    s.add_argument("episode_id")
    args = ap.parse_args(argv)

    try:
        client = HarnessClient(args.target, insecure=args.insecure, pki_dir=args.pki,
                               name=args.name, timeout=args.timeout)
        hot = [(float(at), _load_body(p)) for at, p in args.hot_apply] \
            if args.cmd == "run" else []
        evs = [(float(at), t, json.loads(pl)) for at, t, pl in args.event] \
            if args.cmd == "run" else []
        yml = (Path(args.scenario_file).read_text(encoding="utf-8")
               if args.cmd == "run" and args.scenario_file else "")
    except (H.HarnessUnavailable, OSError, ValueError) as e:
        print(f"cannot start: {type(e).__name__}: {e}", file=sys.stderr)
        return 2
    grpc = H._grpc_modules().grpc
    armed = None                     # an episode started and not yet sent to EndEpisode
    try:
        with client:
            if args.cmd == "status":
                st = client.status(args.episode_id)
                print(st)
                return 0
            ref = client.start_episode(scenario_id=args.scenario or "", scenario_yaml=yml,
                                       cell_id=args.cell, seed=args.seed,
                                       profile=args.profile, arm=args.arm,
                                       keep_flight_log=args.keep_log)
            armed = ref.episode_id
            print(f"started {ref.episode_id}: {ref.run_id} seed {ref.seed}"
                  + (f" profile {ref.profile}" if ref.profile else "")
                  + f", ends at {ref.run_end_sim_t:.1f} s")
            for at, body in hot:
                ack = client.hot_apply(ref.episode_id, at, body)
                print(f"  hot-apply #{ack.queue_index} at {ack.at_sim_t} s: {ack.type} "
                      f"{ack.rule_id} ({ack.op})")
            for at, t, payload in evs:
                ack = client.emit_event(ref.episode_id, at, t, payload)
                print(f"  event #{ack.queue_index} at {ack.at_sim_t} s: {ack.type}")
            armed = None
            b = client.end_episode(ref.episode_id)
    except grpc.RpcError as e:
        print(f"REFUSED {e.code().name}: {e.details()}", file=sys.stderr)
        if armed:
            print(f"episode {armed} was left ARMED: nothing flew, and the server "
                  f"expires it once it has been idle for its TTL", file=sys.stderr)
        return 2
    if args.json:
        from google.protobuf import json_format
        print(json_format.MessageToJson(b))
    else:
        print(render_bundle(b))
    return 0 if b.status in OK_STATUSES else 1


if __name__ == "__main__":
    sys.exit(main())
