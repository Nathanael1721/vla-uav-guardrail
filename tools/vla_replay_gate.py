"""The offline replay gate: recorded camera pairs through the VLA server,
before anyone flies on its answers.

    # a server already up (vla-real env, GPU):
    python demo/vla_server_v2.py serve --precision bf16 --attn sdpa --no-merge
    python tools/vla_replay_gate.py --run demo/out/vla_goto_01 --vla-server 127.0.0.1:8790

    # or let the gate start one (explicit configuration, as demo/vla_fly.py):
    python tools/vla_replay_gate.py --run demo/out/vla_goto_01 --start-server
    python tools/vla_replay_gate.py --run demo/out/vla_goto_01 --start-server --low-vram

    # frames a stand-in flight recorded (a dry run), gating the model on them:
    python tools/vla_replay_gate.py --run demo/out/vla_dry --vla-server 127.0.0.1:8790 --frames-only

    # the committed protocol mosaics (Pillow needed to cut them into tiles):
    python tools/vla_replay_gate.py --frames-dir deploy/evidence/vla_protocol_frames --start-server

EXIT CODES (review of the CityLife flight, P2-7: a gate that could not run
exited 0, and a stand-in could print PASS):

  0  GATE PASSED: a real AerialVLA identity, the flight's own model (or
     --frames-only), at least --min-frames frames, every check passed;
  1  GATE FAILED: a check failed (the report says which);
  2  the gate COULD NOT RUN: no input, the server unreachable or its hello
     not within --hello-timeout-s, or not AerialVLA without --allow-stand-in;
  3  NOT COMPARABLE: the server is not the model the flight recorded
     (metrics.json pilot.server_identity), or the run recorded none - the
     verdict would be about another model. --frames-only gates the frames
     alone, without the flight's answers;
  4  STAND-IN: --allow-stand-in ran the checks against a server that is not
     a VLA. Never a gate pass, whatever the checks say; PASS is never printed.

WHAT IT SENDS

A run of demo/vla_fly.py keeps every request's JPEG pair in vla_frames/
(named by SHA-256) and the request itself in vla_steps.jsonl (pose at
capture, phrase, hint source, object words) - the very bytes the model was
given in flight. For each distinct pair (at most --max-frames, evenly
spread over the run) the gate sends:

  * the RECORDED request again (its own phrase and hint source): parse_ok,
    LAND, and whether the answer and the mosaic hash repeat the flight's
    (skipped with --frames-only: another model's answers are no reference);
  * the PHRASE SWEEP, all seven bands, left rear to right rear
    ("to your left rear ", "to your left ", "forward-left ",
    "straight ahead ", "forward-right ", "to your right ",
    "to your right rear "), with hint_source "goal" as the server's own
    fidelity probe does (vla_server_v2.probe_requests): these are probes,
    not flight decisions. The rear phrases are what a GOTO sends after an
    overshoot; until 2026-10-23 the sweep left them out;
  * the EMPTY phrase (hint_source "none"), the trained "you have arrived".

THE CHECKS (the integration plan, step 5: "on CityLife frames, check that
dpsi is monotonic in the phrase, the parse-fail rate is below 2 %, and LAND
appears only with no hint or near the target")

  frames         at least --min-frames (10) frames were replayed; the report
                 says how many were used of how many distinct pairs.
  parse_fail     malformed answers / all answers <= --max-parse-fail (0.02).
                 The strict reader is the server's (aerialvla_codec.
                 parse_action) and the client re-reads raw_text
                 (vla_link.parse_infer_response), so a parse_ok answer here
                 is exactly three 2-digit bins (+ LAND).
  monotonic      the mean heading change per sweep phrase, over the frames,
                 never decreases from left rear to right rear by more than
                 half a bin (0.011 rad), and "to your right " minus "to your
                 left " spans at least --min-spread-rad (0.1): the phrase
                 must steer, the right way. On 4 Aug's ablation the five
                 side and front means were -0.466 / -0.167 / -0.035 /
                 +0.121 / +0.161 (vla_bridge.py, corrected units). Also the
                 share of frames whose own seven answers are monotone,
                 >= --min-frame-monotone (0.5).
  land_only      a LAND answer under a hint (any phrase but "") counts as a
                 violation unless its pose lies within --near-goal-m of the
                 run's goal (metrics.json mission.goal_xy; an APPROACH run
                 and a frames dir without a target have none, so there every
                 hinted LAND is one). At most --max-land-violations (0).
  plausible_steps  away from the goal (beyond --near-goal-m, or no goal), a
                 step that descends --max-down-m (1.5 m) or more, or that
                 would end below --min-alt-m (2 m) above the ground from its
                 capture altitude, is implausible for a flight at cruise
                 height; at most --max-implausible-share (0.05) of the steps.
  answered       every request answered (no server error, no link failure):
                 a gate the server did not answer has measured nothing.

The no-hint LAND rate is reported (17 of 18 no-hint frames stopped in the
ablation, read strictly), and gated only with --min-nohint-land.

The goal never travels to the server: it is read here for the land_only and
plausible_steps checks alone, from the run's own record of what it
commanded.
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

SWEEP = ("to your left rear ", "to your left ", "forward-left ", "straight ahead ",
         "forward-right ", "to your right ", "to your right rear ")
SIDES = ("to your left ", "to your right ")      # the pair --min-spread-rad is read on
HALF_BIN_RAD = 0.5 * 2.2 / 98          # half of one dpsi bin
SCHEMA = "vlaguard.vla_replay_gate/2"
EXIT_PASS, EXIT_FAIL, EXIT_NO_RUN, EXIT_NOT_COMPARABLE, EXIT_STAND_IN = 0, 1, 2, 3, 4
STATUS_EXIT = {"pass": EXIT_PASS, "fail": EXIT_FAIL, "not_comparable": EXIT_NOT_COMPARABLE,
               "stand_in": EXIT_STAND_IN}


# --------------------------------------------------------------------------- #
# Inputs
# --------------------------------------------------------------------------- #
def frames_from_run(run: Path, max_frames: int | None = None) -> tuple[list[dict], dict]:
    """The distinct recorded pairs of a demo/vla_fly.py run, with the
    request each went out with, the run's goal (or None) and the server
    identity the flight recorded (metrics.json pilot.server_identity, or
    None). Records whose JPEGs are not in vla_frames/ are counted and
    skipped."""
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
    goal = radius = flight_identity = None
    try:
        m = json.loads((run / "metrics.json").read_text(encoding="utf-8"))
        mission = m.get("mission") or {}
        if mission.get("kind") == "goto" and mission.get("goal_xy"):
            goal = tuple(float(v) for v in mission["goal_xy"])
            radius = ((mission.get("monitor") or {}).get("config") or {}).get("arrive_radius_m")
        pilot = m.get("pilot") if isinstance(m.get("pilot"), dict) else {}
        if isinstance(pilot.get("server_identity"), dict):
            flight_identity = dict(pilot["server_identity"])
    except (OSError, ValueError, TypeError, AttributeError):
        pass
    info = {"source": str(run), "records": rows, "distinct_pairs": len(items),
            "skipped_no_frames": missing, "goal_xy": list(goal) if goal else None,
            "arrive_radius_m": radius, "flight_identity": flight_identity}
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
            "arrive_radius_m": None, "flight_identity": None}
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
    bins, dpsi, fwd_m, down_m, image_sha256, error}."""
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
            "fwd_m": r.step.fwd_m if r.step is not None else None,
            "down_m": r.step.down_m if r.step is not None else None,
            "image_sha256": r.image_sha256, "error": r.error,
            "latency_s": r.latency_s}


def _monotone(v: list) -> bool:
    return all(b - a >= -HALF_BIN_RAD for a, b in zip(v, v[1:]))


def run_gate(client: RemoteVLA, items: list[dict], info: dict, *, timeout_s: float = 60.0,
             max_parse_fail: float = 0.02, min_spread_rad: float = 0.1,
             min_frame_monotone: float = 0.5, near_goal_m: float | None = None,
             max_land_violations: int = 0, min_nohint_land: float | None = None,
             min_frames: int = 10, max_down_m: float = 1.5, min_alt_m: float = 2.0,
             max_implausible_share: float = 0.05, frames_only: bool = False) -> dict:
    goal = tuple(info["goal_xy"]) if info.get("goal_xy") else None
    if near_goal_m is None:
        r = info.get("arrive_radius_m")
        near_goal_m = 2.0 * float(r) if r else 6.0
    answers: list[dict] = []
    per_frame = []
    for k, it in enumerate(items):
        fr = {"src": it["src"], "pose": it["pose"], "sweep": {}}
        if (not frames_only and it.get("phrase") is not None
                and it.get("hint_source") is not None):
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
    spread = (means[SIDES[1]] - means[SIDES[0]]
              if means[SIDES[0]] is not None and means[SIDES[1]] is not None else None)
    frame_mono = []
    for f in per_frame:
        v = [f["sweep"].get(ph) for ph in SWEEP]
        if all(x is not None for x in v):
            frame_mono.append(_monotone(v))
    frame_mono_share = (sum(frame_mono) / len(frame_mono)) if frame_mono else None

    def far(pose) -> tuple[bool, float | None]:
        if goal is None:
            return True, None
        d = math.hypot(pose["x"] - goal[0], pose["y"] - goal[1])
        return d > near_goal_m, round(d, 2)
    violations = []
    for a in answers:
        if not a.get("land") or a["phrase"] == NO_HINT:
            continue
        is_far, d = far(a["pose"])
        if is_far:
            violations.append({"probe": a["probe"], "phrase": a["phrase"], "frame": a["frame"],
                               "dist_to_goal_m": d})
    # Step plausibility: a descent of max_down_m or more, or one that would
    # end below min_alt_m above the ground, away from the goal.
    stepped = [a for a in answers if a["kind"] == "step" and a.get("down_m") is not None]
    implausible = []
    for a in stepped:
        is_far, d = far(a["pose"])
        flags = []
        fwd = a.get("fwd_m")
        if fwd is None or not math.isfinite(fwd) or not 0.0 <= fwd <= 5.0:
            flags.append("fwd_out_of_range")
        if is_far and a["down_m"] >= max_down_m:
            flags.append("large_descent")
        if is_far and float(a["pose"]["up"]) - a["down_m"] < min_alt_m:
            flags.append("toward_ground")
        if flags:
            implausible.append({"probe": a["probe"], "phrase": a["phrase"], "frame": a["frame"],
                                "flags": flags, "fwd_m": fwd, "down_m": a["down_m"],
                                "up_capture": a["pose"]["up"], "dist_to_goal_m": d})
    imp_share = (len(implausible) / len(stepped)) if stepped else None
    nohint = [f["no_hint_land"] for f in per_frame]
    nohint_rate = sum(nohint) / len(nohint) if nohint else None
    rec = [a for a in answers if a["probe"] == "recorded"]

    checks = {
        "frames": {"pass": len(items) >= min_frames, "used": len(items),
                   "distinct_pairs": info.get("distinct_pairs"), "min": min_frames},
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
                      "spread_of": list(SIDES), "min_spread_rad": min_spread_rad,
                      "frame_monotone_share": (None if frame_mono_share is None
                                               else round(frame_mono_share, 4)),
                      "min_frame_monotone": min_frame_monotone, "frames_scored": len(frame_mono),
                      "tolerance_rad": round(HALF_BIN_RAD, 5)},
        "land_only_near_goal_or_without_hint": {
            "pass": len(violations) <= max_land_violations, "violations": len(violations),
            "max": max_land_violations, "near_goal_m": near_goal_m,
            "goal_known": goal is not None, "examples": violations[:10]},
        "plausible_steps": {
            "pass": imp_share is not None and imp_share <= max_implausible_share,
            "implausible": len(implausible), "of": len(stepped),
            "share": None if imp_share is None else round(imp_share, 4),
            "max_share": max_implausible_share, "max_down_m": max_down_m,
            "min_alt_m": min_alt_m, "near_goal_m": near_goal_m,
            "fwd_m_median": (round(statistics.median([a["fwd_m"] for a in stepped]), 3)
                             if stepped else None),
            "down_m_max": round(max(a["down_m"] for a in stepped), 3) if stepped else None,
            "examples": implausible[:10]},
    }
    if min_nohint_land is not None:
        checks["nohint_land"] = {"pass": nohint_rate is not None and nohint_rate >= min_nohint_land,
                                 "rate": nohint_rate, "min": min_nohint_land}
    return {
        "schema": SCHEMA, "input": info, "frames": len(items), "requests": n,
        "frames_only": bool(frames_only),
        "identity": identity_of(client.identity),
        "agreement": (client.identity or {}).get("agreement"),
        "checks": checks, "checks_pass": all(c["pass"] for c in checks.values()),
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


def compare_identity(info: dict, gate_identity: dict | None, *, from_run: bool,
                     frames_only: bool) -> dict:
    """Is the server the model the flight recorded? None when there is no
    flight to compare with (a frames dir) or --frames-only says not to."""
    flight = info.get("flight_identity")
    out = {"flight_identity": identity_of(flight) if flight else None,
           "gate_identity": identity_of(gate_identity), "frames_only": bool(frames_only)}
    if not from_run or frames_only:
        out.update(match=None, why="frames only: the flight's answers are not the reference"
                   if frames_only else "no flight: a frames directory")
    elif not flight:
        out.update(match=False, why="the run records no server identity (metrics.json "
                                    "pilot.server_identity): replay it with --frames-only")
    else:
        same = identity_of(flight) == identity_of(gate_identity)
        out.update(match=same, why=None if same else
                   "the server is not the model the flight recorded: its verdict would be "
                   "about another model (--frames-only gates the frames alone)")
    return out


def verdict(rep: dict, *, stand_in: bool) -> str:
    """pass / fail / not_comparable / stand_in (STATUS_EXIT has the codes).
    A stand-in is never a pass; a failed check is a fail whatever model
    answered; a model that is not the flight's cannot pass for it."""
    if stand_in:
        return "stand_in"
    if not rep.get("checks_pass"):
        return "fail"
    if (rep.get("comparison") or {}).get("match") is False:
        return "not_comparable"
    return "pass"


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def build_arg_parser() -> argparse.ArgumentParser:
    import vla_mission as VM
    ap = argparse.ArgumentParser(description="AerialVLA offline replay gate")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--run", help="a demo/vla_fly.py run folder (vla_steps.jsonl + vla_frames/)")
    src.add_argument("--frames-dir", help="frames.json + 224x448 mosaics (Pillow)")
    ap.add_argument("--object", default="orange barrier",
                    help="object words for --frames-dir (inert; a run uses its own)")
    ap.add_argument("--frames-only", action="store_true",
                    help="use the run's frames only: no replay of its recorded answers and no "
                         "comparison with the model that flew it (e.g. a stand-in dry run)")
    ap.add_argument("--vla-server", default=None, metavar="HOST:PORT")
    ap.add_argument("--start-server", action="store_true")
    ap.add_argument("--port", type=int, default=8790)
    ap.add_argument("--python", default=None, help="the server's interpreter")
    ap.add_argument("--precision", choices=("bf16", "nf4"), default="bf16")
    ap.add_argument("--attn", choices=("sdpa", "eager"), default="sdpa")
    ap.add_argument("--merge", action=argparse.BooleanOptionalAction, default=False)
    ap.add_argument("--low-vram", action="store_true", help="nf4/sdpa/merge")
    ap.add_argument("--hello-timeout-s", type=float, default=VM.HELLO_TIMEOUT_S,
                    help="how long to wait for the server's hello (a bf16 cold load under RAM "
                         "pressure took over 15 min; default %(default)g s)")
    ap.add_argument("--allow-stand-in", action="store_true",
                    help="run the checks against a server whose identity is not a VLA (tests, "
                         "dry runs): exit 4, never a gate pass")
    ap.add_argument("--max-frames", type=int, default=60)
    ap.add_argument("--min-frames", type=int, default=10)
    ap.add_argument("--timeout-s", type=float, default=60.0)
    ap.add_argument("--max-parse-fail", type=float, default=0.02)
    ap.add_argument("--min-spread-rad", type=float, default=0.1)
    ap.add_argument("--min-frame-monotone", type=float, default=0.5)
    ap.add_argument("--near-goal-m", type=float, default=None,
                    help="default: twice the run's arrive radius, else 6 m")
    ap.add_argument("--max-land-violations", type=int, default=0)
    ap.add_argument("--min-nohint-land", type=float, default=None)
    ap.add_argument("--max-down-m", type=float, default=1.5)
    ap.add_argument("--min-alt-m", type=float, default=2.0)
    ap.add_argument("--max-implausible-share", type=float, default=0.05)
    ap.add_argument("--out", default=None, help="report JSON (default <run>/replay_gate.json)")
    return ap


def _no_run(msg: str) -> int:
    print(f"the gate could not run: {msg}", file=sys.stderr, flush=True)
    return EXIT_NO_RUN


def main(argv: list[str] | None = None) -> int:
    try:
        return _main(argv)
    except KeyboardInterrupt:
        return _no_run("interrupted (Ctrl+C)")
    except Exception as exc:                                       # noqa: BLE001
        # Whatever went wrong outside the gate's own handling: never exit 0.
        return _no_run(f"{type(exc).__name__}: {exc}")


def _main(argv: list[str] | None) -> int:
    import vla_mission as VM
    args = build_arg_parser().parse_args(argv)
    if bool(args.vla_server) == bool(args.start_server):
        return _no_run("give --vla-server HOST:PORT or --start-server")
    try:
        if args.run:
            items, info = frames_from_run(Path(args.run), args.max_frames)
        else:
            items, info = frames_from_dir(Path(args.frames_dir), args.object, args.max_frames)
    except (OSError, ValueError, KeyError) as exc:
        return _no_run(f"no input: {type(exc).__name__}: {exc}")
    if not items:
        return _no_run(f"no recorded camera pair to replay in {info['source']} "
                       f"({info['skipped_no_frames']} records without frames)")
    print(f"[gate] {len(items)} frames from {info['source']} ({info['distinct_pairs']} distinct "
          f"pairs; at most --max-frames {args.max_frames}, at least --min-frames "
          f"{args.min_frames})", flush=True)
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
    said = {"t": 0.0}

    def on_wait(w: float) -> None:
        if w - said["t"] >= 30.0:
            said["t"] = w
            print(f"[gate] waiting for the server's hello at {server.host}:{server.port}: "
                  f"{w:.0f} s of {args.hello_timeout_s:g} s"
                  + ("" if server.alive else " - the process has exited"), flush=True)
    ok = False
    try:
        server.start()
        hello = server.wait_hello(args.hello_timeout_s, on_wait=on_wait)
        ok, why = VM.identity_is_vla(hello)
        if not ok and not args.allow_stand_in:
            return _no_run(f"the server is not AerialVLA ({why}); --allow-stand-in runs the "
                           f"checks against a stand-in (exit 4, never a gate pass)")
        client = RemoteVLA(server.host, server.port, response_timeout_s=args.timeout_s,
                           reconnect_backoff_s=0.0, strict_identity=ok,
                           late_reply_wait_s=VM.LATE_REPLY_WAIT_S)
        try:
            client.connect()
            rep = run_gate(client, items, info, timeout_s=args.timeout_s,
                           max_parse_fail=args.max_parse_fail,
                           min_spread_rad=args.min_spread_rad,
                           min_frame_monotone=args.min_frame_monotone,
                           near_goal_m=args.near_goal_m,
                           max_land_violations=args.max_land_violations,
                           min_nohint_land=args.min_nohint_land,
                           min_frames=args.min_frames, max_down_m=args.max_down_m,
                           min_alt_m=args.min_alt_m,
                           max_implausible_share=args.max_implausible_share,
                           frames_only=args.frames_only)
        finally:
            client.close()
    except Exception as exc:                                       # noqa: BLE001
        return _no_run(f"{type(exc).__name__}: {exc}")
    finally:
        rec = server.stop()
    rep["server"] = rec
    rep["stand_in"] = not ok
    rep["comparison"] = compare_identity(info, client.identity, from_run=bool(args.run),
                                         frames_only=args.frames_only)
    status = verdict(rep, stand_in=not ok)
    rep["status"] = status
    rep["pass"] = status == "pass"
    out = Path(args.out) if args.out else ((Path(args.run) / "replay_gate.json") if args.run
                                           else ROOT / "demo" / "out" / "vla_replay_gate.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps(rep, indent=1))
    word = ("ok  " if status == "stand_in" else "PASS"), "FAIL"
    for name, c in rep["checks"].items():
        print(f"{word[0] if c['pass'] else word[1]}  {name}: "
              + ", ".join(f"{k}={v}" for k, v in c.items()
                          if k not in ("pass", "examples", "mean_dpsi_rad", "adjacent_ok")))
    m = rep["checks"]["monotonic"]["mean_dpsi_rad"]
    print("      mean dpsi by phrase (rad): " + "  ".join(f"{p.strip()!r} {v}" for p, v in m.items()))
    print(f"      no-hint LAND rate {rep['no_hint_land_rate']}; recorded replays "
          f"{rep['recorded_replay']}; frames {rep['frames']}, requests {rep['requests']}")
    cmp_ = rep["comparison"]
    print(f"      the flight's model {cmp_['flight_identity']}; the gate's {cmp_['gate_identity']}"
          f" -> {'same' if cmp_['match'] else 'n/a' if cmp_['match'] is None else 'DIFFERENT'}")
    lines = {
        "pass": "GATE PASSED",
        "fail": "GATE FAILED",
        "not_comparable": f"NOT COMPARABLE - {cmp_.get('why')}; no gate verdict for this flight",
        "stand_in": "STAND-IN - the checks ran against a server that is not a VLA; this is "
                    "NOT a gate pass, whatever the checks say",
    }
    print(f"{lines[status]} -> {out}")
    return STATUS_EXIT[status]


if __name__ == "__main__":
    sys.exit(main())
