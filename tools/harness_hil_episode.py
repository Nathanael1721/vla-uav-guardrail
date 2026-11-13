"""One harness-driven hil episode: a gRPC session on the server's rig whose
scenario's own zone events the HARNESS forwards over REST into the flying
Shield, and the grant's six-name bundle checked (2026-11-14; audit cards
WP4-03, WP4-05, WP4-11).

    # window 1, the lab checkout, nothing else flying: the harness and its rig
    python -m guardrail.harness_service serve --rail hil --rail-campaign hh01 \\
        --allow-rest-host 10.42.0.3 --rest-pki "$LOCALAPPDATA/vla-guardrail/harness_pki" \\
        --rail-hotapply-pki /opt/vlaguard/demo/out/_pki \\
        --rail-hotapply-server-cert orin-hotapply
    # window 2: one episode (what it would do first, then fly it)
    python tools/harness_hil_episode.py --dry-run
    python tools/harness_hil_episode.py

A thin client (tools/harness_client.py's HarnessClient and printing): it
opens a BACKEND_HIL session bound to the address the mission's hot-apply
listener is started on (--rest-endpoint, default the Orin's cable address
https://10.42.0.3:8071), starts the episode with drive_scenario_events - the
server launches the mission through experiments/sweep_scenarios.py's run_hil
with the scenario MINUS its events (no --events file), and queues each event
as a REST forward at its mission time: a spawn as POST /hot_apply, a
translate / rotate / scale of the zone as PATCH /dynamic_nfz/<id> -, sends
any extra --hot-apply / --event the same way, and calls EndEpisode, which
blocks for the mission, its fetch and its scoring (the campaign's own score,
rail_gate_set ideal, episode_verdict, guardrail.kpi). It prints the bundle and
checks that it holds the grant's six names (manifest.json,
policy_bundle.tar.gz, episode.bag2/, shield_audit.jsonl,
harness_events.jsonl, kpi.json).

The default scenario is dyn-nfz-rotate-step: a 40 x 4 m bar spawned beside
the route at 3 s (absolute vertices: POST /hot_apply) and turned 90 degrees
across it at 8 s (PATCH /dynamic_nfz/bar rotate), gates P0 escape 0, the bar
inert before the turn and binding after it, no breach, no polygon entry. Its
nightly cells have flown on hil with the mission applying both events, so a
harness-driven flight has a campaign twin. dyn-nfz-rotate-spin cannot be
driven this way: its bar is placed against the vehicle and spun with a yaw
rate, neither of which has a REST form (the server refuses it, naming both).
radius-scaling-shrink (spawn at 0 s, scale at 8 s) is the scale case.

--dry-run: NOTHING goes over a network and nothing is launched or written.
The request is resolved and refused exactly as the server's StartEpisode
would (Harness.plan_rail_episode, the same _resolve and _rail_prepare), and
it prints the mission's arguments as run_hil builds them (with the listener's
flags), every forward at its mission time with its route and body, where the
run folder and the bundle will be written, and the server command this run
needs. The --rail-* / --hil-config / --runs options mirror `serve`'s and are
read only by the dry run (the real run's are the server's).

Exit codes: 0 the episode's status is pass, tracked or known_failure and all
six names were written; 1 a status that is not, or a name not written; 2 the
server (or the dry run) refused it, or the client could not start.
"""
from __future__ import annotations

import argparse
import json
import shlex
import sys
import textwrap
import urllib.parse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT, ROOT / "tools"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from guardrail import harness_service as H                       # noqa: E402

DEFAULT_SCENARIO = "dyn-nfz-rotate-step"
DEFAULT_ENDPOINT = "https://10.42.0.3:8071"
DEFAULT_HOTAPPLY_PKI = "/opt/vlaguard/demo/out/_pki"
DEFAULT_HOTAPPLY_CERT = "orin-hotapply"
GRANT_NAMES = ("manifest.json", H.POLICY_BUNDLE_FILE, H.EPISODE_BAG_DIR + "/",
               "shield_audit.jsonl", "harness_events.jsonl", "kpi.json")


def _parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--scenario", default=DEFAULT_SCENARIO,
                    help=f"a scenario_id of the library (default {DEFAULT_SCENARIO})")
    ap.add_argument("--cell", default="", help="the cell, when the family has several")
    ap.add_argument("--seed", type=int, default=1001)
    ap.add_argument("--rest-endpoint", default=DEFAULT_ENDPOINT,
                    help=f"where the mission's hot-apply listener is started (default "
                         f"{DEFAULT_ENDPOINT}, the Orin over the cable)")
    ap.add_argument("--events-via", choices=("harness", "mission"), default="harness",
                    help="who applies the scenario's own events: the harness over REST "
                         "(default) or the mission from its --events file, as a "
                         "campaign flies it")
    ap.add_argument("--hot-apply", nargs=2, action="append", default=[],
                    metavar=("AT_SIM_T", "BODY_FILE"),
                    help="an extra REST hot-apply body (JSON/YAML file) at a mission time")
    ap.add_argument("--event", nargs=3, action="append", default=[],
                    metavar=("AT_SIM_T", "TYPE", "PAYLOAD_JSON"),
                    help="an extra ScenarioEvent (EmitEvent); on a rail it names its zone")
    ap.add_argument("--policy-bundle", default="",
                    help="InjectPolicy first (a file both hosts hold); default: the "
                         "scenario's own policy")
    ap.add_argument("--expected-hash", default="")
    ap.add_argument("--keep-log", action="store_true",
                    help="also copy flight_log.jsonl into the bundle")
    ap.add_argument("--label", default="harness-hil-episode")
    ap.add_argument("--json", action="store_true", help="print the bundle (or the plan) "
                                                        "as JSON")
    # the client
    ap.add_argument("--target", default=f"127.0.0.1:{H.DEFAULT_PORT}")
    ap.add_argument("--insecure", action="store_true", help="plaintext (loopback server)")
    ap.add_argument("--pki", default=str(H.DEFAULT_PKI_DIR))
    ap.add_argument("--name", default=H.DEFAULT_CLIENT_CN, help="client certificate name")
    ap.add_argument("--timeout", type=float, default=120.0, help="each RPC but EndEpisode")
    ap.add_argument("--end-timeout", type=float, default=3600.0,
                    help="EndEpisode: the mission, its fetch and its scoring")
    # the dry run's stand-in for the server's options
    ap.add_argument("--dry-run", action="store_true",
                    help="print what would be launched and forwarded; send nothing")
    ap.add_argument("--hil-config", default=None, metavar="FILE")
    ap.add_argument("--runs", default=str(H.DEFAULT_RUNS_DIR))
    ap.add_argument("--rail-store", default=None, metavar="DIR")
    ap.add_argument("--rail-campaign", default="hh01")
    ap.add_argument("--rail-hotapply-pki", default=DEFAULT_HOTAPPLY_PKI)
    ap.add_argument("--rail-hotapply-host", default="")
    ap.add_argument("--rail-hotapply-server-cert", default=DEFAULT_HOTAPPLY_CERT)
    ap.add_argument("--rest-pki", default=str(H.DEFAULT_PKI_DIR),
                    help="the server's REST TLS identity folder (printed in its command)")
    ap.add_argument("--rest-client", default=H.DEFAULT_CLIENT_CN)
    return ap


def _inputs(args) -> tuple[list, list]:
    """The extra --hot-apply bodies and --event triples, read before anything
    is sent (a usage error is exit 2, never a refusal after StartEpisode)."""
    import harness_client as HC
    hot = [(float(at), HC._load_body(p)) for at, p in args.hot_apply]
    evs = []
    for at, typ, pl in args.event:
        payload = json.loads(pl)
        if payload is not None and not isinstance(payload, dict):
            raise ValueError(f"--event {typ}: PAYLOAD_JSON must be a JSON object")
        evs.append((float(at), typ, payload or {}))
    if args.expected_hash and not args.policy_bundle:
        raise ValueError("--expected-hash goes with --policy-bundle")
    return hot, evs


def server_command(args) -> str:
    """The `serve` command the run needs (window 1)."""
    host = urllib.parse.urlsplit(args.rest_endpoint).hostname or ""
    parts = ["python -m guardrail.harness_service serve --rail hil",
             f"--rail-campaign {args.rail_campaign}"]
    if host and not H.is_loopback(host):
        parts.append(f"--allow-rest-host {host}")
    if args.rest_endpoint.lower().startswith("https:"):
        parts += [f'--rest-pki "{args.rest_pki}"',
                  f"--rail-hotapply-pki {args.rail_hotapply_pki}"]
        if args.rail_hotapply_server_cert:
            parts.append(f"--rail-hotapply-server-cert {args.rail_hotapply_server_cert}")
    if args.rest_client != H.DEFAULT_CLIENT_CN:
        parts.append(f"--rest-client {args.rest_client}")
    if args.hil_config:
        parts.append(f"--hil-config {args.hil_config}")
    if args.rail_store:
        parts.append(f"--rail-store {args.rail_store}")
    return " \\\n    ".join(parts)


def dry_run(args, hot: list, evs: list) -> int:
    """Harness.plan_rail_episode on a rig built from the serve-like options:
    the same resolution and refusals as StartEpisode, nothing sent."""
    S = H._sweep()
    host = urllib.parse.urlsplit(args.rest_endpoint).hostname or ""
    cfg = S.HilConfig.load(args.hil_config) if args.hil_config else None
    rig = H.RailRig(backends=("hil",), hil_config=cfg,
                    store=args.rail_store or (cfg.store if cfg else None),
                    campaign=args.rail_campaign, hotapply_pki=args.rail_hotapply_pki,
                    hotapply_host=args.rail_hotapply_host,
                    hotapply_server_cert=args.rail_hotapply_server_cert,
                    hotapply_clients=((args.rest_client,)
                                      if args.rest_client != H.DEFAULT_CLIENT_CN else ()))
    h = H.Harness(runs_dir=args.runs, rest_hosts=(host,) if host else (), rail=rig)
    try:
        plan = h.plan_rail_episode(backend="hil", rest_endpoint=args.rest_endpoint,
                                   scenario_id=args.scenario, cell_id=args.cell,
                                   seed=args.seed,
                                   drive_scenario_events=args.events_via == "harness",
                                   hot_apply=hot, emit=evs)
    except H.HarnessError as e:
        print(f"DRY RUN: the server would refuse it, {e.code}: {e.message}", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(H._json_safe(plan), indent=2))
        return 0
    via = ("forwarded by the HARNESS over REST (the mission is launched with none)"
           if plan["scenario_events_via"] == "harness" else
           f"applied by the mission from its --events file ({plan['mission_events']})")
    out = ["DRY RUN: nothing launched, nothing sent, nothing written.",
           f"scenario {plan['scenario_id']}, cell {plan['cell_id']}, seed {plan['seed']}, "
           f"Shield {plan['shield']}, policy {plan['policy']}",
           f"mission {plan['tag']} on the hil rail (the rail's time cap "
           f"{plan['rail_max_s']:.1f} s), SITL_HOME {plan['sitl_home']}",
           f"its scenario's own events: {via}",
           "the mission's arguments (both nodes, as run_hil builds them; the "
           "listener's flags last):"]
    out += textwrap.wrap(" ".join(shlex.quote(a) for a in plan["node_args"]), width=96,
                         initial_indent="    ", subsequent_indent="        ",
                         break_long_words=False, break_on_hyphens=False)
    out.append("the forwards, each at its mission time (never early; late by at most "
               "one GET /health poll plus a round trip):")
    if not plan["timeline"]:
        out.append("    none")
    for x in plan["timeline"]:
        tag = x["source"] + (f" #{x['scenario_event_index']}"
                             if "scenario_event_index" in x else "")
        what = f" {x['scenario_event']}" if x.get("scenario_event") else ""
        out.append(f"    t={x['at_sim_t']:6.2f} s  {x['method']:<5} {x['url']}  "
                   f"{json.dumps(x['payload'], sort_keys=True)}  [{tag}{what}]")
    out += ["after the flight (EndEpisode):",
            f"    run folder  {plan['run_dir']}  (fetched from the Orin; "
            f"scenario_score.json beside it)",
            f"    bundle      {plan['bundle_dir']}/",
            "                " + ", ".join(plan["bundle_files"]),
            "the server it needs (window 1, the lab checkout, nothing else flying):",
            "    " + server_command(args).replace("\n", "\n    "),
            "then this, without --dry-run (window 2)."]
    if args.policy_bundle:
        out.append(f"note: the plan is for the scenario's own policy; --policy-bundle "
                   f"{args.policy_bundle} is injected (and the mission pinned to it) only "
                   f"by the real run")
    if plan["rest_endpoint"]:
        out += ["what a dry run cannot check (the flight needs each):",
                "    - the Orin's mission image serves the listener only with fastapi and "
                "uvicorn installed (guardrail/api.py; deploy/docker/companion.Dockerfile "
                "does not install them as of 2026-11-14);",
                "    - the signed policy bundle named above, built on both hosts "
                "(tools/build_flight_bundles.py);"]
        if plan["rest_endpoint"].startswith("https:"):
            out += [f"    - in {args.rail_hotapply_pki} as the container sees it: "
                    f"ca.cert.pem and {args.rail_hotapply_server_cert}.cert.pem / .key.pem "
                    f"(a server certificate naming "
                    f"{urllib.parse.urlsplit(plan['rest_endpoint']).hostname}: "
                    f"python -m guardrail.harness_service issue --name "
                    f"{args.rail_hotapply_server_cert} --server-host "
                    f"{urllib.parse.urlsplit(plan['rest_endpoint']).hostname}), never the "
                    f"CA key;",
                    f"    - the server's --rest-pki holds ca.cert.pem and "
                    f"{args.rest_client}.cert.pem / .key.pem."]
    print("\n".join(out))
    return 0


def fly(args, hot: list, evs: list) -> int:
    import harness_client as HC
    try:
        client = HC.HarnessClient(args.target, insecure=args.insecure, pki_dir=args.pki,
                                  name=args.name, timeout=args.timeout)
    except (H.HarnessUnavailable, OSError, ValueError) as e:
        print(f"cannot start: {type(e).__name__}: {e}", file=sys.stderr)
        return 2
    grpc = H._grpc_modules().grpc
    session = None
    try:
        with client:
            session = client.start_session(backend="hil", seed=args.seed,
                                           rest_endpoint=args.rest_endpoint,
                                           label=args.label).session_id
            print(f"session {session} on the hil rail, endpoint {args.rest_endpoint}")
            if args.policy_bundle:
                ack = client.inject_policy(session, bundle_path=args.policy_bundle,
                                           expected_hash=args.expected_hash)
                print(f"  policy {ack.policy_hash} (signature {ack.signature})")
            ref = client.start_episode(scenario_id=args.scenario, cell_id=args.cell,
                                       seed=args.seed, keep_flight_log=args.keep_log,
                                       session_id=session,
                                       drive_scenario_events=args.events_via == "harness")
            print(f"launched {ref.episode_id}: {ref.run_id} seed {ref.seed}, its "
                  f"scenario's events applied by the {ref.scenario_events_via}")
            st = client.status(ref.episode_id)
            for q in st.events:
                print(f"  queued #{q.index} at {q.at_sim_t:g} s: {q.type} {q.rule_id} "
                      f"[{q.source}]")
            for at, body in hot:
                ack = client.hot_apply(ref.episode_id, at, body)
                print(f"  hot-apply #{ack.queue_index} at {ack.at_sim_t:g} s: "
                      f"{ack.forwarded_method} {ack.forwarded_to}")
            for at, typ, payload in evs:
                ack = client.emit_event(ref.episode_id, at, typ, payload)
                print(f"  event #{ack.queue_index} at {ack.at_sim_t:g} s: {ack.type} -> "
                      f"{ack.forwarded_method} {ack.forwarded_to}")
            print(f"flying: EndEpisode waits for the mission, its fetch and its scoring "
                  f"(up to {args.end_timeout:g} s)", flush=True)
            b = client.end_episode(ref.episode_id, timeout=args.end_timeout)
            stop = client.stop_session(session)
            session = None
            print(f"session stopped: {stop.events} events, {stop.telemetry_frames} "
                  f"telemetry frames")
    except grpc.RpcError as e:
        print(f"REFUSED {e.code().name}: {e.details()}", file=sys.stderr)
        if session:
            # Ours to close: a refused StartEpisode launched nothing, and a
            # failed flight is over (its forwards stay in the session's stream).
            try:
                with HC.HarnessClient(args.target, insecure=args.insecure, pki_dir=args.pki,
                                      name=args.name, timeout=args.timeout) as c2:
                    c2.stop_session(session)
                print(f"session {session} stopped", file=sys.stderr)
            except (grpc.RpcError, H.HarnessUnavailable, OSError, ValueError):
                print(f"session {session} was left OPEN (the server stops it after its "
                      f"idle TTL)", file=sys.stderr)
        return 2
    files = dict(b.bundle_files)
    written = [n for n in GRANT_NAMES if str(files.get(n, "")).startswith("written")]
    if args.json:
        from google.protobuf import json_format
        print(json_format.MessageToJson(b))
    else:
        print(HC.render_bundle(b))
    print(f"grant bundle: {len(written)}/{len(GRANT_NAMES)} names written"
          + ("" if len(written) == len(GRANT_NAMES) else
             "; not written: " + ", ".join(f"{n} ({files.get(n, 'absent')})"
                                           for n in GRANT_NAMES if n not in written)))
    return 0 if b.status in HC.OK_STATUSES and len(written) == len(GRANT_NAMES) else 1


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        hot, evs = _inputs(args)
    except (OSError, ValueError) as e:
        print(f"cannot start: {type(e).__name__}: {e}", file=sys.stderr)
        return 2
    return dry_run(args, hot, evs) if args.dry_run else fly(args, hot, evs)


if __name__ == "__main__":
    sys.exit(main())
