"""The offline replay gate: recorded camera pairs through the VLA server,
before anyone flies on its answers.

    # a server already up (vla-real env, GPU):
    python demo/vla_server_v2.py serve --precision bf16 --attn sdpa --no-merge
    python tools/vla_replay_gate.py --run demo/out/vla_goto_01 --vla-server 127.0.0.1:8790

    # or let the gate start one (explicit configuration, as demo/vla_fly.py):
    python tools/vla_replay_gate.py --run demo/out/vla_goto_01 --start-server
    python tools/vla_replay_gate.py --run demo/out/vla_goto_01 --start-server --low-vram

    # the committed protocol mosaics (Pillow needed to cut them into tiles):
    python tools/vla_replay_gate.py --frames-dir deploy/evidence/vla_protocol_frames --start-server

Exit 0: every check passed. 1: a check failed (the report says which). 2: no
input, or the server could not be reached.

WHAT IT SENDS

A run of demo/vla_fly.py keeps every request's JPEG pair in vla_frames/
(named by SHA-256) and the request itself in vla_steps.jsonl (pose at
capture, phrase, hint source, object words) - the very bytes the model was
given in flight. For each distinct pair (at most --max-frames, evenly
spread over the run) the gate sends:

  * the RECORDED request again (its own phrase and hint source): parse_ok,
    LAND, and whether the answer and the mosaic hash repeat the flight's;
  * the PHRASE SWEEP, the five bands a real box or goal can produce, left to
    right ("to your left ", "forward-left ", "straight ahead ",
    "forward-right ", "to your right "), with hint_source "goal" as the
    server's own fidelity probe does (vla_server_v2.probe_requests): these
    are probes, not flight decisions, and the server's log says so only by
    their sequence numbers;
  * the EMPTY phrase (hint_source "none"), the trained "you have arrived".

THE CHECKS (the integration plan, step 5: "on CityLife frames, check that
dpsi is monotonic in the phrase, the parse-fail rate is below 2 %, and LAND
appears only with no hint or near the target")

  parse_fail     malformed answers / all answers <= --max-parse-fail (0.02).
                 The strict reader is the server's (aerialvla_codec.
                 parse_action) and the client re-reads raw_text
                 (vla_link.parse_infer_response), so a parse_ok answer here
                 is exactly three 2-digit bins (+ LAND).
  monotonic      the mean heading change per sweep phrase, over the frames,
                 never decreases from left to right by more than half a bin
                 (0.011 rad), and right minus left spans at least
                 --min-spread-rad (0.1): the phrase must steer, the right
                 way. On 4 Aug's ablation the means were -0.466 / -0.167 /
                 -0.035 / +0.121 / +0.161 (vla_bridge.py, corrected units).
                 Also the share of frames whose own five answers are
                 monotone, >= --min-frame-monotone (0.5).
  land_only      a LAND answer under a hint (any phrase but "") counts as a
                 violation unless its pose lies within --near-goal-m of the
                 run's goal (metrics.json mission.goal_xy; an APPROACH run
                 and a frames dir without a target have none, so there every
                 hinted LAND is one). At most --max-land-violations (0).
  answered       every request answered (no server error, no link failure):
                 a gate the server did not answer has measured nothing.

The no-hint LAND rate is reported (17 of 18 no-hint frames stopped in the
ablation, read strictly), and gated only with --min-nohint-land.

The goal never travels to the server: it is read here for the land_only
check alone, from the run's own record of what it commanded.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail.vla_link import (NO_HINT, Capture, Hint, Pose, RemoteVLA,   # noqa: E402
                                build_infer_fields, identity_of)

SWEEP = ("to your left ", "forward-left ", "straight ahead ", "forward-right ", "to your right ")
HALF_BIN_RAD = 0.5 * 2.2 / 98          # half of one dpsi bin
SCHEMA = "vlaguard.vla_replay_gate/1"


# --------------------------------------------------------------------------- #
# Inputs
# --------------------------------------------------------------------------- #
def frames_from_run(run: Path, max_frames: int | None = None) -> tuple[list[dict], dict]:
    """The distinct recorded pairs of a demo/vla_fly.py run, with the
    request each went out with, and the run's goal (or None). Records whose
    JPEGs are not in vla_frames/ are counted and skipped."""
    steps = run / "vla_steps.jsonl"
    if not steps.is_file():
        raise FileNotFoundError(f"{steps} does not exist (is {run} a demo/vla_fly.py run?)")
    fdir = run / "vla_frames"
    items, seen, missing, rows = [], set(), 0, 0
    for line in steps.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        rec = json.loads(line)
        rows += 1
        f, d = rec.get("front_jpg_sha256"), rec.get("down_jpg_sha256")
        if not f or not d or rec.get("pose_capture") is None:
            missing += 1
            continue
        if (f, d) in seen:
            continue
        fp, dp = fdir / f"{f}.jpg", fdir / f"{d}.jpg"
        if not (fp.is_file() and dp.is_file()):
            missing += 1
            continue
        seen.add((f, d))
        items.append({"front": fp.read_bytes(), "down": dp.read_bytes(),
                      "pose": rec["pose_capture"], "t_capture": float(rec.get("t_capture") or 0.0),
                      "phrase": rec.get("phrase"), "hint_source": rec.get("hint_source"),
                      "object": rec.get("object") or "", "src": f"seq {rec.get('seq')}",
                      "recorded": {"kind": rec.get("kind"), "bins": rec.get("bins"),
                                   "land": rec.get("land"), "image_sha256": rec.get("image_sha256")}})
    goal = radius = None
    try:
        m = json.loads((run / "metrics.json").read_text(encoding="utf-8"))
        mission = m.get("mission") or {}
        if mission.get("kind") == "goto" and mission.get("goal_xy"):
            goal = tuple(float(v) for v in mission["goal_xy"])
            radius = ((mission.get("monitor") or {}).get("config") or {}).get("arrive_radius_m")
    except (OSError, ValueError, TypeError):
        pass
    info = {"source": str(run), "records": rows, "distinct_pairs": len(items),
            "skipped_no_frames": missing, "goal_xy": list(goal) if goal else None,
            "arrive_radius_m": radius}
    return _spread_pick(items, max_frames), info


def frames_from_dir(frames_dir: Path, object_text: str = "orange barrier",
                    max_frames: int | None = None) -> tuple[list[dict], dict]:
    """The committed bench format (frames.json + 224x448 mosaics, front over
    down; vla_server_v2.bench_requests): each mosaic cut into its tiles and
    JPEG-encoded the way the bench does. Needs Pillow."""
    from guardrail import aerialvla_codec as C
    from PIL import Image
    meta = json.loads((frames_dir / "frames.json").read_text(encoding="utf-8"))
    target = tuple(meta["target"]) if meta.get("target") else None
    items = []
    for f in meta["frames"]:
        img = Image.open(frames_dir / f["file"]).convert("RGB")
        front, down = img.crop((0, 0, 224, 224)), img.crop((0, 224, 224, 448))
        items.append({"front": base64.b64decode(C.encode_jpeg_b64(front)),
                      "down": base64.b64decode(C.encode_jpeg_b64(down)),
                      "pose": {"x": f["x"], "y": f["y"], "up": f["up"], "yaw_rad": f["psi"]},
                      "t_capture": 0.0, "phrase": None, "hint_source": None,
                      "object": object_text, "src": f["file"], "recorded": None})
    info = {"source": str(frames_dir), "records": len(items), "distinct_pairs": len(items),
            "skipped_no_frames": 0, "goal_xy": list(target) if target else None,
            "arrive_radius_m": None}
    return _spread_pick(items, max_frames), info


def _spread_pick(items: list, n: int | None) -> list:
    if not n or len(items) <= n:
        return items
    step = len(items) / n
    return [items[int(i * step)] for i in range(n)]


# --------------------------------------------------------------------------- #
# Asking
# --------------------------------------------------------------------------- #
def ask(client: RemoteVLA, item: dict, phrase: str, source: str, timeout_s: float) -> dict:
    """One request, blocking (tools only). Returns {kind, parse_ok, land,
    bins, dpsi, image_sha256, error}."""
    p = item["pose"]
    cap = Capture(float(item["t_capture"]), Pose(float(p["x"]), float(p["y"]), float(p["up"]),
                                                 float(p["yaw_rad"])), item["front"], item["down"])
    fields, _ = build_infer_fields(cap, Hint(phrase, source), item["object"])
    seq = None
    t_end = time.monotonic() + timeout_s
    while seq is None:
        seq = client.submit(fields)
        if seq is None:
            if time.monotonic() > t_end:
                return {"kind": "link_failure", "error": "could not submit"}
            time.sleep(0.05)
    client.wait_idle(timeout_s)
    res = client.poll()
    if res is None:
        return {"kind": "link_failure", "error": "no result"}
    if res.reply is None:
        return {"kind": "link_failure", "error": f"{res.failure}: {res.detail}"}
    r = res.reply
    return {"kind": r.kind, "parse_ok": r.parse_ok, "land": r.land,
            "bins": list(r.bins) if r.bins else None,
            "dpsi": r.step.dpsi_rad if r.step is not None else None,
            "image_sha256": r.image_sha256, "error": r.error,
            "latency_s": r.latency_s}


def run_gate(client: RemoteVLA, items: list[dict], info: dict, *, timeout_s: float = 60.0,
             max_parse_fail: float = 0.02, min_spread_rad: float = 0.1,
             min_frame_monotone: float = 0.5, near_goal_m: float | None = None,
             max_land_violations: int = 0, min_nohint_land: float | None = None) -> dict:
    goal = tuple(info["goal_xy"]) if info.get("goal_xy") else None
    if near_goal_m is None:
        r = info.get("arrive_radius_m")
        near_goal_m = 2.0 * float(r) if r else 6.0
    answers: list[dict] = []
    per_frame = []
    for k, it in enumerate(items):
        fr = {"src": it["src"], "pose": it["pose"], "sweep": {}}
        if it.get("phrase") is not None and it.get("hint_source") is not None:
            a = ask(client, it, it["phrase"], it["hint_source"], timeout_s)
            a.update(probe="recorded", phrase=it["phrase"], frame=k, pose=it["pose"])
            rec = it.get("recorded") or {}
            a["same_bins_as_flight"] = (a.get("bins") == rec.get("bins")
                                        if rec.get("bins") is not None else None)
            a["same_mosaic_as_flight"] = (a.get("image_sha256") == rec.get("image_sha256")
                                          if rec.get("image_sha256") else None)
            answers.append(a)
            fr["recorded"] = a
        for ph in SWEEP:
            a = ask(client, it, ph, "goal", timeout_s)
            a.update(probe="sweep", phrase=ph, frame=k, pose=it["pose"])
            answers.append(a)
            fr["sweep"][ph] = a.get("dpsi") if a.get("kind") == "step" else None
        a = ask(client, it, NO_HINT, "none", timeout_s)
        a.update(probe="no_hint", phrase=NO_HINT, frame=k, pose=it["pose"])
        answers.append(a)
        fr["no_hint_land"] = bool(a.get("land"))
        per_frame.append(fr)

    n = len(answers)
    unanswered = [a for a in answers if a["kind"] in ("server_error", "link_failure")]
    parsed_or_not = [a for a in answers if a["kind"] in ("step", "land", "parse_fail")]
    n_pf = sum(1 for a in parsed_or_not if a["kind"] == "parse_fail")
    pf_rate = n_pf / len(parsed_or_not) if parsed_or_not else None
    means = {}
    for ph in SWEEP:
        v = [f["sweep"][ph] for f in per_frame if f["sweep"].get(ph) is not None]
        means[ph] = round(statistics.fmean(v), 4) if v else None
    seq = [means[ph] for ph in SWEEP]
    mono_steps = ([b - a >= -HALF_BIN_RAD for a, b in zip(seq, seq[1:])]
                  if all(m is not None for m in seq) else None)
    spread = (seq[-1] - seq[0]) if all(m is not None for m in seq) else None
    frame_mono = []
    for f in per_frame:
        v = [f["sweep"].get(ph) for ph in SWEEP]
        if all(x is not None for x in v):
            frame_mono.append(all(b - a >= -HALF_BIN_RAD for a, b in zip(v, v[1:])))
    frame_mono_share = (sum(frame_mono) / len(frame_mono)) if frame_mono else None
    violations = []
    for a in answers:
        if not a.get("land") or a["phrase"] == NO_HINT:
            continue
        d = (math.hypot(a["pose"]["x"] - goal[0], a["pose"]["y"] - goal[1])
             if goal is not None else None)
        if d is None or d > near_goal_m:
            violations.append({"probe": a["probe"], "phrase": a["phrase"], "frame": a["frame"],
                               "dist_to_goal_m": None if d is None else round(d, 2)})
    nohint = [f["no_hint_land"] for f in per_frame]
    nohint_rate = sum(nohint) / len(nohint) if nohint else None
    rec = [a for a in answers if a["probe"] == "recorded"]

    checks = {
        "answered": {"pass": not unanswered and n > 0, "unanswered": len(unanswered),
                     "examples": [a.get("error") for a in unanswered[:3]]},
        "parse_fail": {"pass": pf_rate is not None and pf_rate <= max_parse_fail,
                       "rate": None if pf_rate is None else round(pf_rate, 4),
                       "count": n_pf, "of": len(parsed_or_not), "max": max_parse_fail},
        "monotonic": {"pass": bool(mono_steps and all(mono_steps) and spread is not None
                                   and spread >= min_spread_rad
                                   and frame_mono_share is not None
                                   and frame_mono_share >= min_frame_monotone),
                      "mean_dpsi_rad": means, "adjacent_ok": mono_steps,
                      "spread_rad": None if spread is None else round(spread, 4),
                      "min_spread_rad": min_spread_rad,
                      "frame_monotone_share": (None if frame_mono_share is None
                                               else round(frame_mono_share, 4)),
                      "min_frame_monotone": min_frame_monotone, "frames_scored": len(frame_mono),
                      "tolerance_rad": round(HALF_BIN_RAD, 5)},
        "land_only_near_goal_or_without_hint": {
            "pass": len(violations) <= max_land_violations, "violations": len(violations),
            "max": max_land_violations, "near_goal_m": near_goal_m,
            "goal_known": goal is not None, "examples": violations[:10]},
    }
    if min_nohint_land is not None:
        checks["nohint_land"] = {"pass": nohint_rate is not None and nohint_rate >= min_nohint_land,
                                 "rate": nohint_rate, "min": min_nohint_land}
    return {
        "schema": SCHEMA, "input": info, "frames": len(items), "requests": n,
        "identity": identity_of(client.identity),
        "agreement": (client.identity or {}).get("agreement"),
        "checks": checks, "pass": all(c["pass"] for c in checks.values()),
        "no_hint_land_rate": None if nohint_rate is None else round(nohint_rate, 4),
        "recorded_replay": {
            "n": len(rec),
            "same_bins_as_flight": sum(1 for a in rec if a.get("same_bins_as_flight")),
            "same_mosaic_as_flight": sum(1 for a in rec if a.get("same_mosaic_as_flight")),
            "land": sum(1 for a in rec if a.get("land"))},
        "latency_s_median": (round(statistics.median([a["latency_s"] for a in answers
                                                      if a.get("latency_s") is not None]), 4)
                             if any(a.get("latency_s") is not None for a in answers) else None),
        "per_frame": per_frame,
    }


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="AerialVLA offline replay gate")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--run", help="a demo/vla_fly.py run folder (vla_steps.jsonl + vla_frames/)")
    src.add_argument("--frames-dir", help="frames.json + 224x448 mosaics (Pillow)")
    ap.add_argument("--object", default="orange barrier",
                    help="object words for --frames-dir (inert; a run uses its own)")
    ap.add_argument("--vla-server", default=None, metavar="HOST:PORT")
    ap.add_argument("--start-server", action="store_true")
    ap.add_argument("--port", type=int, default=8790)
    ap.add_argument("--python", default=None, help="the server's interpreter")
    ap.add_argument("--precision", choices=("bf16", "nf4"), default="bf16")
    ap.add_argument("--attn", choices=("sdpa", "eager"), default="sdpa")
    ap.add_argument("--merge", action=argparse.BooleanOptionalAction, default=False)
    ap.add_argument("--low-vram", action="store_true", help="nf4/sdpa/merge")
    ap.add_argument("--hello-timeout-s", type=float, default=900.0)
    ap.add_argument("--allow-stand-in", action="store_true",
                    help="accept a server whose identity is not a VLA (tests, dry runs)")
    ap.add_argument("--max-frames", type=int, default=60)
    ap.add_argument("--timeout-s", type=float, default=60.0)
    ap.add_argument("--max-parse-fail", type=float, default=0.02)
    ap.add_argument("--min-spread-rad", type=float, default=0.1)
    ap.add_argument("--min-frame-monotone", type=float, default=0.5)
    ap.add_argument("--near-goal-m", type=float, default=None,
                    help="default: twice the run's arrive radius, else 6 m")
    ap.add_argument("--max-land-violations", type=int, default=0)
    ap.add_argument("--min-nohint-land", type=float, default=None)
    ap.add_argument("--out", default=None, help="report JSON (default <run>/replay_gate.json)")
    return ap


def main(argv: list[str] | None = None) -> int:
    import vla_mission as VM
    args = build_arg_parser().parse_args(argv)
    if bool(args.vla_server) == bool(args.start_server):
        print("give --vla-server HOST:PORT or --start-server", file=sys.stderr)
        return 2
    try:
        if args.run:
            items, info = frames_from_run(Path(args.run), args.max_frames)
        else:
            items, info = frames_from_dir(Path(args.frames_dir), args.object, args.max_frames)
    except (OSError, ValueError, KeyError) as exc:
        print(f"no input: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    if not items:
        print(f"no recorded camera pair to replay in {info['source']} "
              f"({info['skipped_no_frames']} records without frames)", file=sys.stderr)
        return 2
    if args.vla_server:
        host, port = VM.parse_addr(args.vla_server)
        server = VM.VLAServer(host, port)
    else:
        cfg = dict(VM.LOW_VRAM) if args.low_vram else {"precision": args.precision,
                                                       "attn": args.attn, "merge": args.merge}
        py = args.python or (VM.VLA_REAL_PYTHON if Path(VM.VLA_REAL_PYTHON).exists()
                             else sys.executable)
        logdir = Path(args.run) if args.run else ROOT / "demo" / "out" / "vla_replay_gate"
        server = VM.VLAServer("127.0.0.1", args.port,
                              argv=VM.server_argv(py, **cfg, port=args.port,
                                                  log=str(logdir / "replay_gate_server.jsonl")),
                              log_path=logdir / "replay_gate_server_stdout.log")
    try:
        server.start()
        hello = server.wait_hello(args.hello_timeout_s)
        ok, why = VM.identity_is_vla(hello)
        if not ok and not args.allow_stand_in:
            print(f"the server is not AerialVLA ({why}); --allow-stand-in to gate a stand-in",
                  file=sys.stderr)
            return 2
        client = RemoteVLA(server.host, server.port, response_timeout_s=args.timeout_s,
                           reconnect_backoff_s=0.0, strict_identity=ok)
        try:
            client.connect()
            rep = run_gate(client, items, info, timeout_s=args.timeout_s,
                           max_parse_fail=args.max_parse_fail,
                           min_spread_rad=args.min_spread_rad,
                           min_frame_monotone=args.min_frame_monotone,
                           near_goal_m=args.near_goal_m,
                           max_land_violations=args.max_land_violations,
                           min_nohint_land=args.min_nohint_land)
        finally:
            client.close()
    except Exception as exc:                                       # noqa: BLE001
        print(f"the gate could not run: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    finally:
        rec = server.stop()
    rep["server"] = rec
    rep["stand_in"] = not ok
    out = Path(args.out) if args.out else ((Path(args.run) / "replay_gate.json") if args.run
                                           else ROOT / "demo" / "out" / "vla_replay_gate.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps(rep, indent=1))
    for name, c in rep["checks"].items():
        print(f"{'PASS' if c['pass'] else 'FAIL'}  {name}: "
              + ", ".join(f"{k}={v}" for k, v in c.items()
                          if k not in ("pass", "examples", "mean_dpsi_rad", "adjacent_ok")))
    m = rep["checks"]["monotonic"]["mean_dpsi_rad"]
    print("      mean dpsi by phrase (rad): " + "  ".join(f"{p.strip()!r} {v}" for p, v in m.items()))
    print(f"      no-hint LAND rate {rep['no_hint_land_rate']}; recorded replays "
          f"{rep['recorded_replay']}; frames {rep['frames']}, requests {rep['requests']}")
    print(f"{'GATE PASSED' if rep['pass'] else 'GATE FAILED'} -> {out}")
    return 0 if rep["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
