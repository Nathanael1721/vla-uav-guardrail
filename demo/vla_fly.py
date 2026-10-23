"""
AerialVLA flies the drone: CityLife_Day on Project AirSim (simple_flight),
every 10 Hz tick through the Safety Shield, with honest records.

    # the launcher: starts the simulator on CityLife_Day, flies, stops it
    .\\scripts\\run_vla_fly.ps1 -Mission goto -Seconds 180 -Tag vla_goto_01
    .\\scripts\\run_vla_fly.ps1 -Mission approach -Seconds 180 -Tag vla_appr_01

    # by hand, the simulator already up on CityLife_Day (vla-real env); this
    # script starts the VLA server itself, waits for its hello, stops it after
    python demo/vla_fly.py --mission goto --tag vla_goto_01 --max-s 180
    python demo/vla_fly.py --mission approach --object "an orange car" --tag vla_appr_01
    python demo/vla_fly.py --mission goto --vla-server 127.0.0.1:8790 ...   # attach
    python demo/vla_fly.py --mission goto --vla-low-vram ...   # nf4/sdpa/merge
    # the whole flight path without the 7B model: the scripted server (its
    # identity is fake:scripted; the run is recorded as a stand-in, never a VLA)
    python demo/vla_fly.py --mission goto --vla-fake-output "96 48 49</s>" --tag vla_dry

WHY A NEW SCRIPT AND NOT `follow_vlm.py --pilot vla`

follow_vlm.fly() is 2,700 lines of a hand-written follower: FenceGuard's
gate and slide, the trail, the guard smoother and the slew limiter all sit
between its controller and the Shield (follow_vlm.py around the
shield.filter call), and the integration discovery report (section 3.4)
said a VLA arm there must turn every one of them off or call the run
hybrid. Here nothing stands between the pilot and the Shield: the step
tracker's action IS the raw action the Shield judges (demo/vla_mission.
MissionLoop). What this script takes from follow_vlm is infrastructure
only - the heartbeat, the collision topic, the controlled landing
(_fly_to_landing_site + its descent), OWL-ViT's Grounder for M2, the
recorder - by import, so a fix there reaches here.

THE MISSIONS (demo/vla_mission.py has the geometry and the stop rules)

  M1 goto      take off at the usual spawn (35, -20), climb to 10 m, a
               hand-written transfer through the Shield to the mission start
               (42, -18), face the goal, then AerialVLA flies to the goal
               (default (43, 40), 58 m down the street) on the goal-derived
               phrase. Ends on LAND, on arriving without LAND, or at the
               time limit.
  M2 approach  the same start, facing down the street; the scenario car is
               parked at --park-at (default (48, 12)); OWL-ViT looks for
               --object on its own camera, the bridge turns the box into the
               phrase. Ends on LAND, the box large, the target lost, or time.

Either way the VLA's steps go StepTracker -> Shield.filter ->
move_by_velocity, LAND becomes the controlled landing, and a dead, slow or
erroring server makes the tracker hover and, after --dead-timeout-s, the
mission end and the aircraft land. --shield off flies the step tracker's
action unfiltered (the control arm, recorded as such), its altitude targets
clamped only to the simulator safety band (vla_mission.SIM_SAFETY_BAND_M,
2-40 m), never to the policy's band.

CADENCE: --cadence stop_and_go (the default since 2026-10-23) is upstream's:
the next image is captured only after the step is flown and the tracker has
held 0.5 s, so each answer's turn and climb are applied once per step, as in
training. `pipelined` sends the next image while a step is flown; with fast
answers it applies each answer's turn and climb at the decision rate (the
point-vehicle probe: 30 s of a constant 2-bin turn drifts -54.6 deg pipelined
against -12.9 deg stop-and-go; docs/DESIGN-vla-pilot.md section 13).

BEFORE TAKE-OFF: one warm-up inference on a live camera pair; a round trip
over --warmup-max-latency-s (5 s), or no answer, refuses the flight (exit 4).
A transfer that ends further than --start-tolerance-m from the mission start
refuses the mission and lands (exit 4). Ctrl+C lands where it is, writes the
records marked interrupted, and exits 130.

RECORDS, in demo/out/<tag>/: vla_steps.jsonl (one line per request: the
server's answer, its model identity, what the tracker made of it),
vla_frames/ (every request's JPEGs by SHA-256, for tools/vla_replay_gate.py),
vla_captures.jsonl, flight_log.jsonl (per tick: author vla_step / hold /
hover / none, the decision's phrase, hint source and step, the Shield's
decision), metrics.json ("pilot": pilot_record kind vla with the server's
reported digest and RemoteVLABackend.stats under "vla"), manifest.json,
kpi.json, audit.jsonl, the replay bundle, view/ (the video frames and the
HUD: phrase, step, authorship, decision age) and vla_server_stdout.log.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import random
import stat
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail.compiler import join_negative_xy                     # noqa: E402

import vla_mission as VM                                             # noqa: E402

TICK = 0.1
SIM_CONFIG_DIR = ROOT / "demo" / "pas_config"
SCENES = {"goto": "scene_vla.jsonc", "approach": "scene_vla_det.jsonc",
          "follow": "scene_vla_det.jsonc"}
# M3 follow's subject: the level's red car of the city follow flights
# (run_citylife_follow.ps1 -Object "a red car" -LevelCar Car_10).
DEFAULT_FOLLOW_OBJECT = "a red car"
DEFAULT_LEVEL_CAR = "Car_10"
# The level car's pose poll, at most this often (an RPC on the loop thread;
# follow_vlm's --level-truth-period is 0.1 s, a follow scores at 2 Hz).
LEVEL_TRUTH_PERIOD_S = 0.5
# A follow waits for a car the level drives around a loop, out of view for
# long spells; an approach's parked car is in view from the start.
LOST_TIMEOUT_S = {"goto": 15.0, "approach": 15.0, "follow": 90.0}
DEFAULT_POLICY = ROOT / "policies" / "vla_goto_citylife.yaml"
DEFAULT_CITYMAP = ROOT / "demo" / "out" / "citymap_citylife" / "occ_day.npz"
CAMERAS = {"front": "VLAFront", "down": "VLADown", "detector": "FrontCamera", "chase": "Chase"}
# Records of an earlier run under the same tag, removed before this one
# starts: a partial run must never sit beside the last run's manifest and
# metrics (follow_vlm's lesson, 2026-10-06 review) - nor beside its replay
# bundle, its replay gate's report or its server's logs (review of the
# CityLife flight, P3-9: the server logs are opened for append, so a reused
# tag's log held two runs). <tag>.replay.tar.gz is added by
# clear_previous_run. Left alone: view/ (the recorder clears its own tps/fpv
# frames; it is usually a junction to C:, never followed here) and
# vla_frames/ (content-addressed JPEGs: this run's vla_steps.jsonl names only
# its own, and an earlier run's frames are not deleted).
RUN_FILES = ("flight_log.jsonl", "vla_steps.jsonl", "vla_captures.jsonl", "detections.jsonl",
             "metrics.json", "kpi.json", "manifest.json", "audit.jsonl", "trajectory.json",
             "vla_server.jsonl", "vla_server_stdout.log", "replay_gate.json",
             "replay_gate_server.jsonl", "replay_gate_server_stdout.log")
XY_FLAGS = ("--goal", "--start-xy", "--park-at")
# Exit codes (scripts/run_vla_fly.ps1 reads them).
EXIT_OK, EXIT_ABORTED, EXIT_VLA_FAILURE, EXIT_REFUSED, EXIT_INTERRUPTED = 0, 1, 3, 4, 130


def _is_link(p: Path) -> bool:
    """A symlink or a Windows junction (Path.is_symlink misses junctions
    before Python 3.12)."""
    try:
        st = os.lstat(p)
    except OSError:
        return False
    if stat.S_ISLNK(st.st_mode):
        return True
    return bool(getattr(st, "st_file_attributes", 0)
                & getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400))


def clear_previous_run(out: Path, tag: str) -> dict:
    """Remove the records of an earlier run under this tag (RUN_FILES and
    <tag>.replay.tar.gz) - regular files only, directly in `out`; a link or
    junction in their place is left alone and named. Never touches view/ or
    vla_frames/ (RUN_FILES' comment says why). Returns what it did."""
    removed, left = [], []
    for name in RUN_FILES + (f"{tag}.replay.tar.gz",):
        p = out / name
        if _is_link(p):
            left.append(f"{name} (a link: left alone)")
            continue
        try:
            p.unlink()
            removed.append(name)
        except FileNotFoundError:
            pass
        except OSError as exc:
            left.append(f"{name} ({type(exc).__name__}: {exc})")
    if left:
        print(f"[run] *** not removed from {out}: {'; '.join(left)}")
    return {"removed": removed, "left": left}


def parse_xy(s: str) -> tuple[float, float]:
    try:
        x, y = (float(v) for v in str(s).split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"{s!r} is not X,Y in metres (NED x North, y East)")
    if not (math.isfinite(x) and math.isfinite(y)):
        raise argparse.ArgumentTypeError(f"{s!r} is not finite")
    return x, y


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="AerialVLA flies CityLife behind the Shield")
    ap.add_argument("--mission", choices=VM.MISSIONS, default="goto")
    ap.add_argument("--tag", default=None, help="demo/out/<tag> (default vla_<mission>)")
    ap.add_argument("--out-dir", default=None,
                    help="write the run here instead of demo/out/<tag> (tests; the "
                         "dashboard reads only demo/out)")
    ap.add_argument("--max-s", type=float, default=180.0, help="mission seconds")
    ap.add_argument("--seed", type=int, default=20261022)
    # -- the mission ---------------------------------------------------------
    ap.add_argument("--goal", type=parse_xy, default=None,
                    help=f"M1 goal X,Y (default {VM.DEFAULT_GOAL_XY})")
    ap.add_argument("--start-xy", type=parse_xy, default=VM.DEFAULT_START_XY,
                    help="where the hand-written transfer puts the aircraft before the "
                         "mission clock (default %(default)s)")
    ap.add_argument("--no-transfer", action="store_true",
                    help="start the VLA at the spawn (35, -20), beside two street trees")
    ap.add_argument("--start-heading", default=None,
                    help="'goal' (M1 default: face the goal), 'street' (M2 default: "
                         f"{VM.STREET_HEADING_DEG:g} deg) or degrees")
    ap.add_argument("--cruise-alt", type=float, default=VM.DEFAULT_CRUISE_UP)
    ap.add_argument("--start-tolerance-m", type=float, default=2.0,
                    help="refuse the mission (land) when the transfer ends further than "
                         "this from the mission start (default %(default)s)")
    ap.add_argument("--object", default=None,
                    help="the prompt's object words (INERT for AerialVLA) and, for M2, the "
                         "OWL-ViT query. Default: vla_mission.GOTO_OBJECT_TEXT (a fixed, "
                         "neutral street description in upstream's style) for goto, "
                         "'an orange car' for approach")
    ap.add_argument("--park-at", type=parse_xy, default=None,
                    help=f"M2: park the scenario car here (default {VM.DEFAULT_PARK_XY}); "
                         f"truth for scoring only")
    ap.add_argument("--no-park", action="store_true", help="M2: do not place the scenario car")
    ap.add_argument("--level-car", default=None,
                    help=f"M3 follow: the car the LEVEL drives that is the subject, read from "
                         f"the simulator for SCORING only (subject_dist_m per row; never the "
                         f"phrase, the VLA or a stop rule). Default {DEFAULT_LEVEL_CAR}")
    ap.add_argument("--arrive-radius-m", type=float, default=3.0)
    ap.add_argument("--arrive-grace-s", type=float, default=8.0)
    ap.add_argument("--overshoot-s", type=float, default=3.0,
                    help="M1: after arriving, the goal behind and receding this long ends "
                         "the mission arrived_overshoot")
    ap.add_argument("--close-box-frac", type=float, default=0.15)
    ap.add_argument("--lost-timeout-s", type=float, default=None,
                    help="no fresh box this long ends a detector mission (default "
                         "15 s for approach, 90 s for follow)")
    ap.add_argument("--det-fresh-s", type=float, default=1.0)
    ap.add_argument("--approach-radius-m", type=float, default=10.0)
    ap.add_argument("--dead-timeout-s", type=float, default=10.0)
    ap.add_argument("--first-decision-timeout-s", type=float, default=60.0)
    ap.add_argument("--det-thresh", type=float, default=0.02)
    ap.add_argument("--colour-min", type=float, default=0.10)
    ap.add_argument("--bearing-gain", type=float, default=1.0,
                    help="VLABridge gain (1.0 honest; anything else overstates the bearing "
                         "and is recorded)")
    # -- the Shield and the policy ------------------------------------------
    ap.add_argument("--shield", choices=("on", "off"), default="on",
                    help="off: the control arm, recorded - the step tracker's action flies "
                         "unfiltered, its altitude targets clamped only to the simulator "
                         "safety band (vla_mission.SIM_SAFETY_BAND_M), not the policy's")
    ap.add_argument("--bundle", default=None)
    ap.add_argument("--policy", default=None)
    ap.add_argument("--dev-yaml", action="store_true",
                    help="fly a YAML policy (unsigned), recorded as a development run")
    ap.add_argument("--allow-unverified-bundle", action="store_true")
    ap.add_argument("--citymap", default=str(DEFAULT_CITYMAP))
    # -- the VLA server -----------------------------------------------------
    ap.add_argument("--vla-server", default=None, metavar="HOST:PORT",
                    help="attach to a running server instead of starting one")
    ap.add_argument("--vla-port", type=int, default=8790)
    ap.add_argument("--vla-python", default=None,
                    help=f"the server's interpreter (default {VM.VLA_REAL_PYTHON} when it "
                         f"exists, else this one)")
    ap.add_argument("--vla-precision", choices=("bf16", "nf4"), default=None)
    ap.add_argument("--vla-attn", choices=("sdpa", "eager"), default=None)
    ap.add_argument("--vla-merge", action=argparse.BooleanOptionalAction, default=None)
    ap.add_argument("--vla-low-vram", action="store_true",
                    help="nf4/sdpa/merge, the measured low-VRAM option (agrees with the "
                         "reference on 25-33 %% of probe inputs, against 96 %% for the "
                         "default bf16/sdpa/no-merge)")
    ap.add_argument("--vla-hello-timeout-s", type=float, default=VM.HELLO_TIMEOUT_S,
                    help="how long to wait for the server's hello (a bf16 cold load under "
                         "RAM pressure took over 15 min; default %(default)g s)")
    ap.add_argument("--vla-fake-output", nargs="+", default=None,
                    help="start the scripted stand-in server instead of the model")
    ap.add_argument("--vla-fake-delay-s", type=float, default=0.3)
    ap.add_argument("--allow-stand-in", action="store_true",
                    help="fly against an attached server whose identity is not a VLA")
    ap.add_argument("--no-local-weights", action="store_true",
                    help="record the server's reported digest without hashing the "
                         "weight files here")
    # -- the pilot ----------------------------------------------------------
    ap.add_argument("--cadence", choices=("pipelined", "stop_and_go"), default="stop_and_go",
                    help="stop_and_go (default): upstream's - the next image only after the "
                         "step is flown and held 0.5 s; pipelined: the next image while the "
                         "step is flown (applies each answer's turn/climb at the decision rate)")
    ap.add_argument("--decision-timeout-s", type=float, default=10.0)
    ap.add_argument("--warmup-max-latency-s", type=float, default=VM.WARMUP_MAX_LATENCY_S,
                    help="refuse to take off when the warm-up inference's round trip is "
                         "slower than this (default %(default)g s)")
    ap.add_argument("--late-reply-wait-s", type=float, default=VM.LATE_REPLY_WAIT_S,
                    help="after a request times out, wait this long for its late answer "
                         "before asking again (the server cannot cancel it)")
    ap.add_argument("--cruise-mps", type=float, default=1.0)
    ap.add_argument("--speed-cap-mps", type=float, default=2.0)
    # -- flight infrastructure ----------------------------------------------
    ap.add_argument("--alt-gain", type=float, default=0.6)
    ap.add_argument("--climb-max", type=float, default=1.8)
    ap.add_argument("--land-site", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--save-view", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--record-hz", type=float, default=20.0)
    ap.add_argument("--record-height", type=int, default=720)
    ap.add_argument("--live-view", action="store_true")
    ap.add_argument("--live-height", type=int, default=480)
    ap.add_argument("--view-switch", type=int, default=0)
    ap.add_argument("--no-policy-hud", action="store_true")
    ap.add_argument("--check-args", action="store_true",
                    help="parse and resolve the arguments, print them, exit 0 (nothing "
                         "started; scripts/run_vla_fly.ps1 runs this before the simulator)")
    return ap


def parse_cli(ap: argparse.ArgumentParser, argv: list[str] | None = None):
    """ap.parse_args with a negative first X,Y value joined to its flag
    (guardrail.compiler.join_negative_xy, as follow_vlm and the rails do),
    then the mission-dependent defaults and refusals (resolve_args)."""
    argv = list(sys.argv[1:] if argv is None else argv)
    args = ap.parse_args(join_negative_xy(argv, XY_FLAGS))
    try:
        return resolve_args(args)
    except ValueError as exc:
        ap.error(str(exc))


def resolve_args(args):
    """Fill the mission defaults and refuse combinations that cannot fly
    honestly. ValueError says why."""
    m = args.mission
    args.tag = args.tag or f"vla_{m}"
    if args.lost_timeout_s is None:
        args.lost_timeout_s = LOST_TIMEOUT_S[m]
    if m != "follow" and args.level_car is not None:
        raise ValueError("--level-car is for --mission follow")
    if m == "follow":
        # M3: chase the level's moving car. The phrase comes from OWL-ViT's box
        # (as M2's); the subject's truth is the level car, read for scoring.
        if args.park_at is not None or args.no_park:
            raise ValueError("--park-at / --no-park are for --mission approach: a follow's "
                             "subject is a car the level drives (--level-car)")
        if args.goal is not None:
            raise ValueError("--goal is for --mission goto: a follow's hint comes from the "
                             "detector box, never from a position")
        if args.start_heading == "goal":
            raise ValueError("--start-heading goal is for --mission goto")
        args.object = DEFAULT_FOLLOW_OBJECT if args.object is None else args.object
        if not args.object.strip():
            raise ValueError("a follow needs --object words: they are OWL-ViT's query")
        args.level_car = args.level_car or DEFAULT_LEVEL_CAR
        args.park_at = None
        args.start_heading = args.start_heading or "street"
    elif m == "goto":
        if args.park_at is not None or args.no_park:
            raise ValueError("--park-at / --no-park are for --mission approach")
        args.goal = tuple(args.goal) if args.goal is not None else VM.DEFAULT_GOAL_XY
        args.object = VM.GOTO_OBJECT_TEXT if args.object is None else args.object
        args.start_heading = args.start_heading or "goal"
    else:
        if args.goal is not None:
            raise ValueError("--goal is for --mission goto: an approach's hint comes from the "
                             "detector box, never from a position")
        if args.start_heading == "goal":
            raise ValueError("--start-heading goal is for --mission goto")
        args.object = "an orange car" if args.object is None else args.object
        if not args.object.strip():
            raise ValueError("an approach needs --object words: they are OWL-ViT's query")
        args.park_at = None if args.no_park else (tuple(args.park_at) if args.park_at
                                                  else VM.DEFAULT_PARK_XY)
        args.start_heading = args.start_heading or "street"
    if args.start_heading not in ("goal", "street"):
        try:
            float(args.start_heading)
        except ValueError:
            raise ValueError(f"--start-heading {args.start_heading!r} is not goal, street or "
                             f"degrees")
    # The server refuses object words it would not put in a prompt (a
    # newline, surrounding blanks, a position): refuse them here, before
    # anything starts, not on the first request in the air.
    from guardrail.aerialvla_codec import CodecError, validate_object
    try:
        validate_object(args.object)
    except CodecError as exc:
        raise ValueError(f"--object {args.object!r}: {exc}")
    for k in ("start_tolerance_m", "warmup_max_latency_s", "decision_timeout_s"):
        v = getattr(args, k)
        if not (math.isfinite(v) and v > 0):
            raise ValueError(f"--{k.replace('_', '-')} must be a positive number, got {v!r}")
    if not (math.isfinite(args.late_reply_wait_s) and args.late_reply_wait_s >= 0):
        raise ValueError("--late-reply-wait-s must be 0 or more")
    # The VLA server's configuration: always all three, explicitly.
    if args.vla_server and (args.vla_fake_output or args.vla_low_vram or args.vla_precision
                            or args.vla_attn or args.vla_merge is not None):
        raise ValueError("--vla-server attaches to a running server: its configuration is "
                         "whatever it was started with (recorded from its hello); drop the "
                         "--vla-precision/--vla-attn/--vla-merge/--vla-low-vram/"
                         "--vla-fake-output flags")
    if args.vla_low_vram:
        if args.vla_precision or args.vla_attn or args.vla_merge is not None:
            raise ValueError("--vla-low-vram is nf4/sdpa/merge; give it or the three flags")
        cfg = dict(VM.LOW_VRAM)
    else:
        cfg = dict(VM.FIDELITY)
        if args.vla_precision:
            cfg["precision"] = args.vla_precision
        if args.vla_attn:
            cfg["attn"] = args.vla_attn
        if args.vla_merge is not None:
            cfg["merge"] = bool(args.vla_merge)
    args.vla_config = cfg
    if args.vla_python is None:
        args.vla_python = VM.VLA_REAL_PYTHON if Path(VM.VLA_REAL_PYTHON).exists() else sys.executable
    if args.live_view and not args.save_view:
        raise ValueError("--live-view needs --save-view")
    if args.policy and not args.dev_yaml and not args.bundle:
        raise ValueError(f"{args.policy} is a YAML policy: fly its signed bundle (python "
                         f"tools/build_flight_bundles.py vla_goto_citylife), or pass --dev-yaml "
                         f"to fly it as a development run, recorded as such")
    # The stop rules validate themselves (MonitorConfig).
    args.monitor = VM.MonitorConfig(
        mission=m, max_s=args.max_s, arrive_radius_m=args.arrive_radius_m,
        arrive_grace_s=args.arrive_grace_s, close_box_frac=args.close_box_frac,
        lost_timeout_s=args.lost_timeout_s, det_fresh_s=args.det_fresh_s,
        dead_timeout_s=args.dead_timeout_s,
        first_decision_timeout_s=args.first_decision_timeout_s,
        overshoot_s=args.overshoot_s)
    return args


def check_args_report(args) -> dict:
    """What --check-args prints: the resolved flight, nothing started."""
    return {"ok": True, "mission": args.mission, "tag": args.tag,
            "goal_xy": list(args.goal) if args.goal else None,
            "start_xy": list(args.start_xy), "park_at": list(args.park_at) if args.park_at else None,
            "object": args.object, "shield": args.shield, "cadence": args.cadence,
            "vla_server": args.vla_server, "vla_config": args.vla_config,
            "vla_fake_output": args.vla_fake_output, "max_s": args.max_s,
            "warmup_max_latency_s": args.warmup_max_latency_s,
            "monitor": args.monitor.as_dict()}


def server_plan(args, out: Path) -> "VM.VLAServer":
    """The server this flight talks to: attached (--vla-server) or started
    here with an explicit configuration (or the scripted stand-in)."""
    if args.vla_server:
        host, port = VM.parse_addr(args.vla_server)
        return VM.VLAServer(host, port)
    c = args.vla_config
    argv = VM.server_argv(args.vla_python, precision=c["precision"], attn=c["attn"],
                          merge=c["merge"], port=args.vla_port,
                          log=str(out / "vla_server.jsonl"),
                          fake_output=args.vla_fake_output, fake_delay_s=args.vla_fake_delay_s)
    return VM.VLAServer("127.0.0.1", args.vla_port, argv=argv,
                        log_path=out / "vla_server_stdout.log")


def start_heading_rad(args, start_xy) -> float:
    if args.start_heading == "goal":
        return math.atan2(args.goal[1] - start_xy[1], args.goal[0] - start_xy[0])
    if args.start_heading == "street":
        return math.radians(VM.STREET_HEADING_DEG)
    return math.radians(float(args.start_heading))


# --------------------------------------------------------------------------- #
# The recorder's view and the HUD (recorder thread; Pillow)
# --------------------------------------------------------------------------- #
def decode_native(msg):
    """A camera message -> RGB PIL image at its own size, or None."""
    if not msg or "data" not in msg or not len(msg["data"]):
        return None
    import cv2
    import numpy as np
    from PIL import Image
    data = msg["data"]
    buf = (np.array(data, dtype=np.uint8) if isinstance(data, list)
           else np.frombuffer(data, dtype=np.uint8))
    arr = (cv2.imdecode(buf, cv2.IMREAD_COLOR) if msg.get("encoding") == "PNG"
           else buf.reshape(int(msg["height"]), int(msg["width"]), 3))
    if arr is None:
        return None
    return Image.fromarray(cv2.cvtColor(arr, cv2.COLOR_BGR2RGB))


class RecorderView:
    """What demo/recorder.FrameRecorder films. Third person: the Chase view.
    First person: for M1 the two tiles AerialVLA sees, front | down, side by
    side (each 256x256, the recorder enlarges them before the HUD is drawn);
    for M2 the detector's 768x432 camera, whose box the HUD draws, with the
    two VLA tiles inset at the bottom left."""

    def __init__(self, obs, frames: VM.VLAFrames, mission: str):
        self.obs, self.frames, self.mission = obs, frames, mission

    def get_chase_native(self):
        return self.obs.get_chase_native()

    def get_front_native(self):
        from PIL import Image
        front = decode_native(self.frames.latest_msg("front"))
        down = decode_native(self.frames.latest_msg("down"))
        if self.mission == "goto":
            if front is None:
                return None
            canvas = Image.new("RGB", (front.width * 2, front.height), (0, 0, 0))
            canvas.paste(front, (0, 0))
            if down is not None:
                canvas.paste(down.resize(front.size), (front.width, 0))
            return canvas
        base = self.obs.get_front_native()
        if base is None:
            return front
        s = int(base.height * 0.30)
        for i, tile in enumerate(t for t in (front, down) if t is not None):
            base.paste(tile.resize((s, s)), (6 + i * (s + 4), base.height - s - 6))
        return base


# The VLA panel lives in demo/vla_mission.py since 2026-10-23 (wave V2b: the
# ArduPilot rail's VLA pilot draws the same one); kept here under its names.
AUTHOR_LABEL = VM.AUTHOR_LABEL
hud_lines = VM.hud_lines
annotate_vla = VM.annotate_vla


# --------------------------------------------------------------------------- #
# The flight
# --------------------------------------------------------------------------- #
def _state_from_kin(kin, State, quat_yaw):
    p = kin["pose"]["position"]
    yaw = quat_yaw(kin["pose"]["orientation"])
    tw = (kin.get("twist") or {}).get("linear") or {}
    vel = None
    if all(tw.get(k) is not None for k in ("x", "y", "z")):
        vel = (float(tw["x"]), float(tw["y"]), -float(tw["z"]))
    return State(x=float(p["x"]), y=float(p["y"]), up=-float(p["z"]),
                 yaw_deg=math.degrees(yaw), velocity=vel), yaw


async def _transfer(drone, shield, State, Action4D, quat_yaw, target_xy, cruise_up, args,
                    live, timeout_s: float = 40.0) -> dict:
    """The hand-written pre-mission transfer, through the Shield, at cruise
    altitude: the VLA's mission clock starts only after it. Recorded; never
    VLA-authored (it is before the first act())."""
    rec = {"author": "hand-written (not the VLA), before the mission clock",
           "target": list(target_xy), "reached": False, "ticks": 0, "shield_touched": 0}
    t_end = time.time() + timeout_s
    while time.time() < t_end:
        kin = drone.get_ground_truth_kinematics()
        st, _ = _state_from_kin(kin, State, quat_yaw)
        if rec["ticks"] == 0:
            rec["start"] = [round(st.x, 2), round(st.y, 2)]
        dx, dy = target_xy[0] - st.x, target_xy[1] - st.y
        dist = math.hypot(dx, dy)
        if dist < 0.6:
            rec["reached"] = True
            break
        v = min(1.5, 0.6 * dist)
        vz = max(-args.climb_max, min(args.climb_max, (cruise_up - st.up) * args.alt_gain))
        dec = shield.filter(st, Action4D(vx=v * dx / dist, vy=v * dy / dist, vz_up=vz))
        rec["ticks"] += 1
        rec["shield_touched"] += int(dec.touched)
        e = dec.emitted
        live.beat(detail=f"transfer to the mission start, {dist:.0f} m")
        await drone.move_by_velocity_async(e.vx, e.vy, -e.vz_up, duration=0.3,
                                           yaw_is_rate=True, yaw=e.yaw_rate)
        await asyncio.sleep(TICK)
    await drone.move_by_velocity_async(0.0, 0.0, 0.0, duration=0.3)
    kin = drone.get_ground_truth_kinematics()
    st, _ = _state_from_kin(kin, State, quat_yaw)
    rec["final"] = [round(st.x, 2), round(st.y, 2), round(st.up, 2)]
    return rec


class PreflightRefused(RuntimeError):
    """The flight is refused before its mission (vla_mission.NOT_FLOWN_ENDS:
    `kind`), with the reason as the message."""

    def __init__(self, kind: str, why: str):
        super().__init__(why)
        self.kind = kind


def _feed_pose(drone, obs, State, quat_yaw, FV):
    """One kinematics read into SemanticObs's pose ring (what VLAFrames reads
    each capture's pose from), stamped mid-RPC as follow_vlm stamps it.
    Returns (state, yaw, kin)."""
    tk0 = time.time()
    kin = drone.get_ground_truth_kinematics()
    t_kin = (tk0 + time.time()) / 2.0
    state, yaw = _state_from_kin(kin, State, quat_yaw)
    q = kin["pose"]["orientation"]
    obs.put_pose_full(t_kin, state.x, state.y, state.up, yaw, FV.quat_pitch(q),
                      FV.quat_roll(q), stamp=kin.get("time_stamp"))
    return state, yaw, kin


async def _warmup(args, drone, obs, frames, client, hint_src, State, quat_yaw, FV, live,
                  wait_frames_s: float = 30.0) -> dict:
    """The warm-up gate, before arming (review of the CityLife flight, P2-5):
    VM.WARMUP_REQUESTS inferences, each on a fresh LIVE camera pair (the pose
    ring fed here, as the loop feeds it), timed on the client's clock, their
    frames kept like any request sent (vla_captures.jsonl marks them
    warmup). The LAST is judged (vla_mission.warmup_series_verdict; re-review
    N1: the first is the model's first inference ever and pays a cold start
    no flight request pays, so it gets first_decision_timeout_s). Every
    attempt is recorded under `attempts`. Never flown: the aircraft is on
    the ground."""
    attempts: list[dict] = []
    res: dict = {}
    for k in range(VM.WARMUP_REQUESTS):
        # A ready client first (review of the ArduPilot rail's VLA pilot,
        # 4a0da6d, P1-1 / P2-8): a late answer still awaited or the backoff
        # after a server error made the next request "not sent" here, and a
        # healthy server was refused. Blocking, so off the loop; the pose
        # feed and the capture stay on it.
        ready = await asyncio.to_thread(VM.wait_client_ready, client)
        cap = None
        t_end = time.time() + wait_frames_s
        while time.time() < t_end:
            _feed_pose(drone, obs, State, quat_yaw, FV)
            cap = frames.capture()
            if cap is not None:
                break
            live.beat(detail="waiting for the VLA camera pair (warm-up)")
            await asyncio.sleep(0.1)
        if cap is None:
            return {"pass": False, "answered": False, "max_latency_s": args.warmup_max_latency_s,
                    "why": f"no VLA camera pair (front + down, paired by stamp) within "
                           f"{wait_frames_s:g} s", "frames": frames.stats(),
                    "attempts": attempts}
        hint = VM.warmup_hint(args.mission, hint_src, cap)
        live.beat(detail=f"warm-up inference {k + 1}/{VM.WARMUP_REQUESTS} before take-off")
        # The request carries its own timeout (the first: the model's first
        # inference ever), and the record says the one the client enforced.
        timeout_s = VM.warmup_timeout_s(k, args.decision_timeout_s, args.first_decision_timeout_s)
        res = await asyncio.to_thread(VM.warmup_infer, client, cap, hint, args.object, timeout_s)
        res["client_ready"] = ready
        if res.get("seq") is not None:
            frames.commit(cap, hint, res["seq"], warmup=True)
        attempts.append(dict(res))
    return VM.warmup_result(res, attempts, args.warmup_max_latency_s,
                            getattr(client, "late", None))


async def _land_here(drone) -> str:
    """The abort's landing (an exception, Ctrl+C, a refusal in the air): stop,
    then the simulator's land where the aircraft is. Best effort; a second
    Ctrl+C is not swallowed. Returns what happened, for the records."""
    try:
        await drone.move_by_velocity_async(0.0, 0.0, 0.0, duration=0.5)
        await asyncio.wait_for(await drone.land_async(), timeout=60.0)
        return "landed where it was (land_async)"
    except Exception as exc:                                       # noqa: BLE001
        return f"the abort landing failed: {type(exc).__name__}: {exc}"


def _write_json_lenient(path: Path, obj) -> None:
    """For the records' last resort: anything that is not JSON becomes its
    repr instead of losing the file."""
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps(obj, indent=1, ensure_ascii=False, default=repr))


async def fly(args) -> int:
    """The flight. A VLA server this script started is ALWAYS stopped on the
    way out, whatever raised and wherever (review of the CityLife flight,
    P3-10: an exception between its start and the simulator's try left about
    17 GB on the GPU); an attached one (--vla-server) never is
    (VLAServer.stop() has no process to stop)."""
    holder: dict = {}
    try:
        return await _fly(args, holder)
    finally:
        srv = holder.get("server")
        if srv is not None:
            srv.stop()                 # idempotent: a second stop finds the process gone


async def _fly(args, holder: dict) -> int:
    from projectairsim import Drone, EnvActor, ProjectAirSimClient, World

    import follow_vlm as FV
    from guardrail import AuditLogger, State
    from guardrail.bundle import load_for_flight
    from guardrail.manifest import flight_bundle_path, flight_policy_choice
    from guardrail.models import Action4D, AltitudeEnvelope, ObstacleClearance
    from guardrail.replay import rail_shield, verify_replay, write_replay
    from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S
    from guardrail.vla_backends import make_backend
    from guardrail.vla_link import RemoteVLA, TrackerConfig, identity_of
    from semantic_demo import SemanticObs, quat_yaw
    import city_planner
    import occ_bands

    random.seed(args.seed)
    try:
        import numpy as np
        np.random.seed(args.seed & 0xFFFFFFFF)
    except ImportError:
        pass
    shield_on = args.shield == "on"

    # ---- the policy: the signed bundle by default (the SITL rails' rule) --
    bundle, yaml_path, choice = flight_policy_choice(
        args.bundle, args.policy, dev_yaml=args.dev_yaml, default_yaml=DEFAULT_POLICY,
        default_bundle=flight_bundle_path(DEFAULT_POLICY))
    policy, policy_source = load_for_flight(bundle, yaml_path,
                                            allow_unverified=args.allow_unverified_bundle)
    policy_source = {**policy_source, **choice}

    out = Path(args.out_dir) if args.out_dir else ROOT / "demo" / "out" / args.tag
    out.mkdir(parents=True, exist_ok=True)
    cleared = clear_previous_run(out, args.tag)

    # ---- the obstacle map for the policy's band (follow_vlm's rule) --------
    cmap = city_planner.load_occ(args.citymap)
    alt = policy.by_type(AltitudeEnvelope)
    occ_report = []
    if policy.by_type(ObstacleClearance) and alt:
        sel = occ_bands.select_for_band(Path(args.citymap).parent, alt[0].alt_min_m,
                                        alt[0].alt_max_m)
        occ_report = list(sel["report"])
        for ln in occ_report:
            print(ln)
        cmap = sel["map"]
    smap = (None if cmap is None or not policy.by_type(ObstacleClearance) else
            {"occ": cmap["occ"], "res": cmap["res"], "ox": cmap["ox"], "oy": cmap["oy"]})
    if policy.by_type(ObstacleClearance) and smap is None:
        print("[occ] *** no obstacle map for the policy band: ObstacleClearance is INERT")
    # The grant's 5 s / 0.1 s forecast, no escalation FSM inside the Shield
    # (guardrail/replay.rail_shield, docs/DESIGN-shield-enforcement.md): this
    # rail flies `emitted`. follow_vlm's city default (3 s / 0.5 s) exists for
    # its subject stand-off; there is no subject here.
    shield = rail_shield(policy, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S, obstacle_map=smap)
    audit = AuditLogger(out / "audit.jsonl", policy)
    street = None
    sm = Path(args.citymap).parent / "street.npz"
    if sm.is_file():
        from build_street_mask import load_street
        street = load_street(sm)
    pind = pind_error = None
    if not args.no_policy_hud and args.save_view:
        try:
            from policy_hud import PolicyIndicator, render_base_map
            pind = PolicyIndicator(policy, base_map=render_base_map(smap, street), shield=shield)
        except Exception as exc:                                  # noqa: BLE001
            pind_error = f"{type(exc).__name__}: {exc}"
    # The pilot side's altitude clamp: the policy's band with the Shield on;
    # with it off, only the simulator safety band (P1-2).
    backend_policy, pilot_band, pilot_band_rec = VM.pilot_alt_config(shield_on, policy)
    print(f"[policy] {policy.policy_id} {policy.policy_hash} from {policy_source.get('kind')} "
          f"{policy_source.get('path')} - signature {policy_source.get('signature')}")
    if shield_on:
        print(f"[shield] ON, forecast {shield.lookahead_s:g} s at {shield.dt:g} s")
    else:
        lo, hi = VM.SIM_SAFETY_BAND_M
        print(f"[shield] OFF (control arm): the step tracker's action flies unfiltered, its "
              f"altitude targets clamped only to the simulator safety band {lo:g}-{hi:g} m "
              f"(not the policy's); the Shield judges in shadow, forecast "
              f"{shield.lookahead_s:g} s at {shield.dt:g} s")
    print(f"[pilot] cadence {args.cadence}"
          + (" (upstream's: capture after the step is flown and held 0.5 s)"
             if args.cadence == "stop_and_go" else
             " (the next capture while a step is flown: turns and climbs at the decision rate)"))

    live = FV._live_heartbeat(args.tag, log_dir=out, total_s=args.max_s, phase="starting",
                              detail="starting the VLA server")
    server = server_plan(args, out)
    holder["server"] = server
    identity = None
    said = {"t": 0.0}

    def _on_wait(w: float) -> None:
        live.beat(detail=f"waiting for the VLA server ({w:.0f} s)")
        if w - said["t"] >= 60.0:
            said["t"] = w
            print(f"[vla] still waiting for the server's hello: {w:.0f} s of "
                  f"{args.vla_hello_timeout_s:g} s"
                  + ("" if server.alive else " - the process has exited"), flush=True)
    try:
        server.start()
        print(f"[vla] {'started ' + ' '.join(server.argv[1:]) if server.started_here else 'attaching to ' + server.host + ':' + str(server.port)}")
        identity = server.wait_hello(args.vla_hello_timeout_s, on_wait=_on_wait)
    except BaseException as exc:
        server.stop()
        live.stop("failed", detail=f"no VLA server ({type(exc).__name__})")
        if isinstance(exc, (KeyboardInterrupt, SystemExit)):
            raise
        raise SystemExit(f"[vla] *** no VLA server: {type(exc).__name__}: {exc}")
    is_vla, why_not = VM.identity_is_vla(identity)
    if not is_vla and not (args.vla_fake_output or args.allow_stand_in):
        server.stop()
        live.stop("failed", detail="the server is not a VLA")
        raise SystemExit(f"[vla] *** the server's identity is not AerialVLA ({why_not}); "
                         f"pass --allow-stand-in to fly a stand-in, recorded as one")
    print(f"[vla] hello: {identity_of(identity)}"
          + ("" if is_vla else f"  *** STAND-IN, not a VLA: {why_not}"))
    weight_paths = None if (args.no_local_weights or not is_vla) else VM.local_weight_paths(identity)

    backend = saver = None
    try:
        client = RemoteVLA(server.host, server.port, response_timeout_s=args.decision_timeout_s,
                           strict_identity=is_vla, late_reply_wait_s=args.late_reply_wait_s,
                           clock=time.time)
        obs = SemanticObs()
        saver = VM.FrameSaver(out / "vla_frames")
        frames = VM.VLAFrames(obs, saver=saver)
        det_source = detector_hint = grounder = None
        if args.mission == "goto":
            hint_src = VM.goto_hint_source(args.goal, args.arrive_radius_m)
        else:
            from vla_bridge import VLABridge
            bridge = VLABridge(args.object, hfov_deg=90.0, bearing_gain=args.bearing_gain)
            grounder_box = {"g": None}

            def latest_box():
                """(det, t of its frame, the heading at its frame, t published)
                from follow_vlm.Grounder (its cap carries the frame's pose)."""
                g = grounder_box["g"].latest() if grounder_box["g"] is not None else None
                if not g or g.get("det") is None:
                    return None, None, None, None
                cap = g.get("cap") or {}
                yaw = (cap.get("pose") or {}).get("yaw")
                return (g["det"], (cap.get("t_capture") or cap.get("t_cap") or g.get("t_det")),
                        None if yaw is None else float(yaw), g.get("t_det"))
            det_source = latest_box
            detector_hint = VM.DetectorHint(latest_box, bridge, fresh_s=args.det_fresh_s)
            hint_src = VM.HintSource(detector_hint, "detector")
        tcfg = TrackerConfig(cruise_mps=args.cruise_mps, speed_cap_mps=args.speed_cap_mps,
                             decision_timeout_s=args.decision_timeout_s, alt_band=pilot_band)
        backend = make_backend("aerovla", None, backend_policy, get_capture=frames.capture,
                               get_hint=hint_src, client=client, object_text=args.object,
                               tracker_config=tcfg, cadence=args.cadence,
                               steps_log=out / "vla_steps.jsonl", on_submit=frames.commit,
                               clock=time.time)
        pilot_alt_band = {**pilot_band_rec, "applied": dict(backend.alt_band_info)}
        print(f"[pilot] altitude clamp {backend.alt_band_info.get('band')} "
              f"({backend.alt_band_info.get('source')}): {pilot_band_rec['why']}")
        try:
            client.connect()                   # pins the identity before the loop
        except Exception as exc:                                   # noqa: BLE001
            raise SystemExit(f"[vla] *** the link failed before take-off: {exc}")
        if identity_of(client.identity) != identity_of(identity):
            raise SystemExit("[vla] *** the pinned identity is not the hello's")
        sim = ProjectAirSimClient()
        sim.connect()
    except BaseException as exc:
        # Everything between the hello and the simulator: close what opened,
        # stop the server (fly()'s finally stops it too), say why.
        for closer in (backend.close if backend is not None else None,
                       saver.close if saver is not None else None):
            try:
                if closer is not None:
                    closer()
            except Exception:                                      # noqa: BLE001
                pass
        server.stop()
        live.stop("failed", detail=f"before take-off ({type(exc).__name__})")
        if isinstance(exc, (KeyboardInterrupt, SystemExit)):
            raise
        raise SystemExit(f"[vla] *** before take-off: {type(exc).__name__}: {exc}")

    rows: list[dict] = []
    traj: list[dict] = []
    collisions: list[dict] = []
    collisions_subscribed = False
    recorder = view_dir = None
    landing_info = {"enabled": bool(args.land_site)}
    pre_mission: dict = {}
    car = car_truth = None
    level_car = None
    level_truth_errors: list[str] = []
    aborted = abort_landing = None
    interrupted = False
    refused = None
    warmup = None
    airborne = False
    end_reason = None
    loop = None
    t0 = t_connect = None
    start_heading_err = None
    work_ms: list[float] = []
    cmd_times: list[float] = []
    t_mission_end = None
    monitor = None
    frames_stats = saver_stats = stats_at_end = None
    drone = None
    try:
        live.beat(detail="loading the scene")
        world = World(sim, SCENES[args.mission], delay_after_load_sec=2,
                      sim_config_path=str(SIM_CONFIG_DIR))
        for _ in range(max(0, args.view_switch)):
            try:
                world.switch_streaming_view()
            except Exception:                                      # noqa: BLE001
                break
        drone = Drone(sim, world, "Drone1")
        t_connect = time.time()

        def _on_collision(_, m):
            if isinstance(m, dict) and m.get("has_collided", True):
                collisions.append({"t_conn": round(time.time() - t_connect, 2),
                                   "object": m.get("object_name"),
                                   "penetration_m": m.get("penetration_depth"),
                                   "impact": m.get("impact_point")})
        try:
            sim.subscribe(drone.robot_info["collision_info"], _on_collision)
            collisions_subscribed = True
        except Exception as exc:                                   # noqa: BLE001
            print(f"[collide] *** not subscribed ({type(exc).__name__}): contacts NOT counted")
        sim.subscribe(drone.sensors[CAMERAS["front"]]["scene_camera"],
                      lambda _, m: frames.put_front(m))
        sim.subscribe(drone.sensors[CAMERAS["down"]]["scene_camera"],
                      lambda _, m: frames.put_down(m))
        if args.mission in VM.DETECTOR_MISSIONS:
            sim.subscribe(drone.sensors[CAMERAS["detector"]]["scene_camera"],
                          lambda _, m: obs.put_front(m))
            # OWL-ViT loads in its own thread from here, beside the warm-up
            # and the take-off (2026-10-23): a follow's subject, the level's
            # car, starts driving when the scene loads, and loading the
            # detector only after the transfer cost the first ~30 s of it.
            grounder = FV.Grounder(obs, args.object, thresh=args.det_thresh,
                                   log_path=out / "detections.jsonl",
                                   colour_min=args.colour_min, lock=None)
            grounder_box["g"] = grounder
            grounder.start()
        if args.save_view:
            try:
                sim.subscribe(drone.sensors[CAMERAS["chase"]]["scene_camera"],
                              lambda _, m: obs.put_chase(m))
                from recorder import FrameRecorder
                view_dir = out / "view"
                (view_dir / "tps").mkdir(parents=True, exist_ok=True)
                (view_dir / "fpv").mkdir(parents=True, exist_ok=True)
                recorder = FrameRecorder(RecorderView(obs, frames, args.mission), view_dir,
                                         annotate_vla, hz=args.record_hz,
                                         height=args.record_height, live_view=args.live_view,
                                         live_height=args.live_height)
                recorder.start()
            except Exception as exc:                               # noqa: BLE001
                print(f"[view] *** no recording ({type(exc).__name__}: {exc})")
                recorder = None

        if args.mission == "approach" and args.park_at is not None:
            import car_trajectory
            import moving_car
            px, py = args.park_at
            # A one-metre, one-shot route ENDING at the park point: the car
            # comes to rest there (follow_vlm --park-at, the env actor path).
            car = moving_car.MovingCar(world, speed_mps=0.5, route=[(px, py - 1.0), (px, py)],
                                       one_shot=True, phase_s=0.0)
            try:
                env_car = EnvActor(sim, world, FV.ENV_CAR_NAME)
                car_trajectory.upload(world, car, seconds=args.max_s + 120.0)
                car.actual_name = None
                car_trajectory.start(env_car)
                car_truth = (px, py)
                print(f"[car] the scenario car parked at ({px:.1f}, {py:.1f}) - truth for "
                      f"scoring only")
            except Exception as exc:                               # noqa: BLE001
                print(f"[car] *** could not park the scenario car ({type(exc).__name__}: {exc})")
                car = None

        # M3: the level's car, resolved before arming (refuses the flight if
        # the tag is absent, as follow_vlm --level-car does). Truth for
        # scoring only: polled at most every LEVEL_TRUTH_PERIOD_S.
        if args.mission == "follow":
            import level_actors
            level_car = level_actors.LevelCar(world, args.level_car, desc=args.object,
                                              min_period_s=LEVEL_TRUTH_PERIOD_S)
            try:
                level_car.spawn()
            except Exception as exc:                               # noqa: BLE001
                raise PreflightRefused("level_car_missing",
                                       f"the level car {args.level_car} did not resolve "
                                       f"({type(exc).__name__}: {exc})") from None
            print(f"[car] following the level's {args.level_car} ({args.object!r}) at "
                  f"({level_car.pos[0]:.1f}, {level_car.pos[1]:.1f}) - truth for scoring only")

        # ---- the warm-up gate: one inference on a live pair, before arming --
        warmup = await _warmup(args, drone, obs, frames, client, hint_src, State, quat_yaw,
                               FV, live)
        print(f"[vla] warm-up: {warmup.get('kind')} in {warmup.get('rtt_s')} s round trip "
              f"(server {warmup.get('latency_server_s')} s; gate "
              f"{args.warmup_max_latency_s:g} s) - "
              + ("PASS" if warmup["pass"] else f"REFUSED: {warmup['why']}"))
        if not warmup["pass"]:
            raise PreflightRefused("warmup_refused", warmup["why"])

        live.beat(detail="arming and taking off")
        drone.enable_api_control()
        drone.arm()
        airborne = True
        await (await drone.takeoff_async())
        for _ in range(400):
            kin = drone.get_ground_truth_kinematics()
            if -kin["pose"]["position"]["z"] >= args.cruise_alt - 0.5:
                break
            live.beat(detail=f"climbing to {args.cruise_alt:.0f} m")
            await drone.move_by_velocity_async(0.0, 0.0, -2.0, duration=0.2)
            await asyncio.sleep(0.1)
        live.beat(phase="airborne", detail="at cruise altitude")
        start_xy = VM.CITYLIFE_SPAWN_XY if args.no_transfer else tuple(args.start_xy)
        if not args.no_transfer:
            pre_mission = await _transfer(drone, shield, State, Action4D, quat_yaw, start_xy,
                                          args.cruise_alt, args, live)
            print(f"[transfer] to {start_xy}: reached {pre_mission['reached']}, "
                  f"{pre_mission['ticks']} ticks, Shield touched {pre_mission['shield_touched']}")
        st0, _ = _state_from_kin(drone.get_ground_truth_kinematics(), State, quat_yaw)
        start_err = math.hypot(st0.x - start_xy[0], st0.y - start_xy[1])
        pre_mission.update({"start_xy": list(start_xy), "start_error_m": round(start_err, 2),
                            "start_tolerance_m": args.start_tolerance_m})
        if start_err > args.start_tolerance_m:
            raise PreflightRefused(
                "start_not_reached",
                f"the transfer ended {start_err:.1f} m from the mission start "
                f"({start_xy[0]:g}, {start_xy[1]:g}), more than --start-tolerance-m "
                f"{args.start_tolerance_m:g}: the mission would not start where it was planned")
        psi0 = start_heading_rad(args, start_xy)
        await (await drone.rotate_to_yaw_async(yaw=psi0))
        await asyncio.sleep(0.5)
        st1, got = _state_from_kin(drone.get_ground_truth_kinematics(), State, quat_yaw)
        start_heading_err = math.degrees(abs((psi0 - got + math.pi) % (2 * math.pi) - math.pi))
        pre_mission.update({"start_heading_deg": round(math.degrees(psi0), 2),
                            "start_heading_rule": args.start_heading,
                            "start_heading_err_deg": round(start_heading_err, 2),
                            # where the mission really began: the pose after the turn
                            "start_pose_actual": {"x": round(st1.x, 2), "y": round(st1.y, 2),
                                                  "up": round(st1.up, 2),
                                                  "yaw_deg": round(st1.yaw_deg, 2)}})

        if grounder is not None:
            # Started with the cameras (above); its first inference, if not yet.
            live.beat(detail="loading the detector")
            for _ in range(600):
                if grounder.latest()["seq"] > 0:
                    break
                await asyncio.sleep(0.5)

        # ---- the mission: AerialVLA flies ---------------------------------
        t0 = time.time()
        monitor = VM.MissionMonitor(args.monitor, t0)
        loop = VM.MissionLoop(backend, shield, monitor, mission=args.mission,
                              shield_on=shield_on,
                              goal_xy=args.goal if args.mission == "goto" else None,
                              det_source=det_source, hint_source=hint_src,
                              detector_hint=detector_hint, audit=audit,
                              server_alive=lambda: server.alive, clock=time.time)
        print(f"[vla] mission {args.mission} - AerialVLA flies; hint source "
              f"{hint_src.kind}; object words {args.object!r} (inert)")
        deadline = None
        while True:
            tk0 = time.time()
            state, yaw, kin = _feed_pose(drone, obs, State, quat_yaw, FV)
            off_map = shield.off_map(state.x, state.y) if shield.has_obstacle_map else None
            res = loop.tick(state, extra_row={"off_map": off_map})
            e = res.flown
            cmd_times.append(time.time())
            await drone.move_by_velocity_async(e.vx, e.vy, -e.vz_up, duration=0.3,
                                               yaw_is_rate=True, yaw=e.yaw_rate)
            if level_car is not None:
                # Truth for SCORING only: the level car against the aircraft.
                try:
                    level_car.update(time.time() - t0)
                    cx, cy = level_car.pos
                    if math.isfinite(cx) and math.isfinite(cy):
                        res.row["subject_xy"] = [round(cx, 2), round(cy, 2)]
                        res.row["subject_dist_m"] = round(math.hypot(state.x - cx,
                                                                     state.y - cy), 2)
                except Exception as exc:                       # noqa: BLE001
                    if len(level_truth_errors) < 20:
                        level_truth_errors.append(f"{type(exc).__name__}: {exc}")
            rows.append(res.row)
            traj.append({"x": state.x, "y": state.y, "up": state.up,
                         "touched": bool(res.hud_decision.touched)})
            if recorder is not None:
                pol = None
                if pind is not None and pind_error is None:
                    try:
                        # With the Shield off: no repair, the flown action,
                        # the shadow's violations as escapes (P1-1).
                        pol = pind.update(res.row["t"], res.hud_decision, state,
                                          clearance_m=(shield.clearance_at(state.x, state.y)
                                                       if shield.has_obstacle_map else None),
                                          off_map=bool(off_map), yaw_rad=yaw)
                    except Exception as exc:                       # noqa: BLE001
                        pind_error = f"{type(exc).__name__}: {exc}"
                det = None
                if args.mission in VM.DETECTOR_MISSIONS and res.row.get("det") is not None:
                    det = det_source()[0]
                recorder.set_hud({**res.hud, "policy": pol}, det)
            live.beat(progress=min(1.0, res.row["t"] / max(1e-6, args.max_s)),
                      detail=f"{res.row['author']} #{res.row['vla_seq']}")
            if res.row["tick"] % 50 == 0:
                print(f"  tick {res.row['tick']}: ({state.x:6.1f},{state.y:6.1f},{state.up:4.1f}) "
                      f"{res.row['author']:8} {res.row['author_reason']:<14} "
                      f"phrase={res.row['phrase']!r} shield="
                      + ("-" if not res.decision.touched else
                         "HIT" if shield_on else "shadow-HIT (not flown)"))
            work_ms.append((time.time() - tk0) * 1000.0)
            if res.end_reason is not None:
                end_reason = res.end_reason
                print(f"[vla] mission ends: {end_reason} - {VM.END_REASONS[end_reason]}")
                break
            now = time.time()
            deadline = (tk0 if deadline is None else deadline) + TICK
            if deadline < now - TICK:
                deadline = now
            await asyncio.sleep(max(0.0, deadline - now))
        t_mission_end = time.time() + 0.3
        await drone.move_by_velocity_async(0.0, 0.0, 0.0, duration=0.5)
        # Read before close(): stats() rates decisions over the time since the
        # first act(), and neither the close nor a minute of landing belongs
        # in that span (a 2.5 Hz pilot read 1.5 Hz with the landing in it).
        stats_at_end = backend.stats()
        backend.close()            # the VLA stops here; its records close (vla_steps.jsonl)
        after = backend.stats()    # the log's counts are final only after close()
        stats_at_end.update({k: after.get(k) for k in ("log_written", "log_errors",
                                                       "last_log_error", "log_pending",
                                                       "late_replies")})

        # ---- the controlled landing (follow_vlm's), never a mode request ---
        live.beat(progress=1.0, phase="landing", detail=f"mission over ({end_reason}); landing")
        if grounder is not None:
            grounder.stop()
        hud_live = recorder is not None and pind is not None and pind_error is None
        if args.land_site:
            landing_info = await FV._fly_to_landing_site(
                drone, shield, SimpleNamespace(citymap=args.citymap, cruise_alt=args.cruise_alt,
                                               alt_gain=args.alt_gain, climb_max=args.climb_max),
                pind=pind if hud_live else None, recorder=recorder if hud_live else None,
                t0=t0, query=args.object, live=live)
        for _k in range(600):
            kin = drone.get_ground_truth_kinematics()
            up = -kin["pose"]["position"]["z"]
            live.beat(detail=f"descending ({up:.0f} m)")
            if recorder is not None and _k % 5 == 0:
                pol = None
                if hud_live:
                    try:
                        p_ = kin["pose"]["position"]
                        pol = pind.ended(State(x=float(p_["x"]), y=float(p_["y"]), up=up),
                                         "MISSION ENDED - DESCENDING TO LAND (Shield not in "
                                         "this loop)", quat_yaw(kin["pose"]["orientation"]),
                                         t=time.time() - t0)
                    except Exception:                              # noqa: BLE001
                        pol = None
                recorder.set_hud({"t": time.time() - t0, "alt": up, "frame": "landing",
                                  "label": f"descending after {end_reason} (no Shield)",
                                  "policy": pol}, None)
            if up <= 1.2:
                break
            await drone.move_by_velocity_async(0.0, 0.0, max(0.6, min(2.0, up * 0.15)),
                                               duration=0.3)
            await asyncio.sleep(0.1)
        await (await drone.land_async())
        airborne = False
        drone.disarm()
        drone.disable_api_control()
    except PreflightRefused as exc:
        refused = {"kind": exc.kind, "why": str(exc)}
        end_reason = exc.kind
        print(f"[vla] *** REFUSED ({exc.kind}): {exc}")
        if airborne and drone is not None:
            abort_landing = await _land_here(drone)
            airborne = False
            print(f"[vla] {abort_landing}")
    except (Exception, KeyboardInterrupt, asyncio.CancelledError) as exc:
        # An abort - or Ctrl+C, which is not an Exception (P3-11): either way
        # put it down where it is, then fall through to the records.
        interrupted = isinstance(exc, (KeyboardInterrupt, asyncio.CancelledError))
        aborted = (f"interrupted by the operator ({type(exc).__name__})" if interrupted
                   else f"{type(exc).__name__}: {exc}")
        if interrupted and end_reason is None:
            end_reason = "interrupted"
        print(f"[warn] flight {'interrupted' if interrupted else 'aborted'}: {aborted}")
        if airborne and drone is not None:
            abort_landing = await _land_here(drone)
            airborne = False
            print(f"[warn] {abort_landing}")
    finally:
        try:
            VM.write_jsonl(out / "flight_log.jsonl", rows)
        except Exception as exc:                                   # noqa: BLE001
            print(f"[warn] could not write the flight log: {type(exc).__name__}: {exc}")
        leaving = sys.exc_info()[0]
        if leaving is not None:
            live.stop("stopped" if issubclass(leaving, (KeyboardInterrupt, asyncio.CancelledError))
                      else "failed", detail=f"ended by {leaving.__name__}")

        def _teardown():
            if recorder is not None:
                recorder.stop()
                recorder.join(timeout=5.0)
                recorder.write_sidecar(Path(view_dir) / "recorder.json")
            if grounder is not None:
                grounder.stop()
            try:
                if car is not None:
                    car.destroy()
            except Exception:                                      # noqa: BLE001
                pass
            try:
                sim.disconnect()
            except Exception:                                      # noqa: BLE001
                pass
        td = threading.Thread(target=_teardown, daemon=True)
        td.start()
        td.join(timeout=30.0)
        try:
            backend.close()        # idempotent enough: a second close finds nothing open
        except Exception:                                          # noqa: BLE001
            pass
        saver_stats = saver.close()
        frames_stats = frames.stats()
        server_rec = server.stop()
        try:
            VM.write_jsonl(out / "vla_captures.jsonl", list(frames.log))
        except Exception:                                          # noqa: BLE001
            pass

    # ---- the records: every part guarded (P3-12), so a fault in one still
    # leaves metrics.json - with the error - and the manifest if it builds ---
    from guardrail.replay import zone_dwell
    records_errors: list[str] = []

    def guard(name, fn, default=None):
        try:
            return fn()
        except Exception as exc:                                   # noqa: BLE001
            records_errors.append(f"{name}: {type(exc).__name__}: {exc}")
            print(f"[records] *** {name} failed: {type(exc).__name__}: {exc}")
            return default

    stats = stats_at_end if stats_at_end is not None else guard("backend.stats", backend.stats, {})
    records = list(backend.records)
    dwell = guard("zone_dwell", lambda: zone_dwell(shield, traj, TICK), {}) or {}
    band = policy.by_type(AltitudeEnvelope)
    alt_bad = (sum(1 for p in traj if p["up"] < band[0].alt_min_m or p["up"] > band[0].alt_max_m)
               if band else 0)
    goal = args.goal if args.mission == "goto" else None
    scores = guard("mission_scores",
                   lambda: VM.mission_scores(args.mission, rows, goal_xy=goal,
                                             arrive_radius_m=args.arrive_radius_m,
                                             truth_xy=car_truth,
                                             approach_radius_m=args.approach_radius_m),
                   {"reached_goal": None}) or {"reached_goal": None}
    mission_s = rows[-1]["t"] if rows else None
    n_touched = sum(1 for r in rows if r.get("touched"))
    metrics = {
        "tag": args.tag, "rail": "projectairsim-simple_flight", "level": "CityLife_Day",
        "scene": SCENES[args.mission], "ticks": len(rows), "mission_s": mission_s,
        "loop_hz": VM.loop_rate(rows), "end_reason": end_reason,
        "end_reason_means": VM.END_REASONS.get(end_reason) or VM.NOT_FLOWN_ENDS.get(end_reason),
        "aborted": aborted, "interrupted": interrupted, "refused": refused,
        "abort_landing": abort_landing,
        "tick_work_ms": VM.spread(work_ms),
        "mission": guard("mission", lambda: {
            "kind": args.mission, "goal_xy": list(args.goal) if args.goal else None,
            "start_xy": list(VM.CITYLIFE_SPAWN_XY if args.no_transfer else args.start_xy),
            "start_pose_actual": pre_mission.get("start_pose_actual"),
            "spawn_xy": list(VM.CITYLIFE_SPAWN_XY), "cruise_alt_m": args.cruise_alt,
            "object": args.object, "object_words_are_inert": True,
            "object_is_default": (args.mission == "goto" and args.object == VM.GOTO_OBJECT_TEXT),
            "hint_source": hint_src.kind, "hint": hint_src.stats(),
            "detector_hint": detector_hint.stats() if detector_hint is not None else None,
            "park_at": list(args.park_at) if args.park_at else None,
            "bearing_gain": args.bearing_gain if args.mission in VM.DETECTOR_MISSIONS else None,
            "detector": FV.DETECTOR_ID if args.mission in VM.DETECTOR_MISSIONS else None,
            "level_car": args.level_car,
            "monitor": monitor.summary() if monitor is not None else None}),
        **scores,
        "follow": (guard("follow_scores", lambda: {
            **VM.follow_scores(rows), "level_car": args.level_car,
            "truth_period_s": LEVEL_TRUTH_PERIOD_S,
            "truth_errors": list(level_truth_errors),
            "level_stats": level_car.stats() if level_car is not None else None})
                   if args.mission == "follow" else None),
        "land": guard("land", lambda: VM.land_record(rows, records, goal)),
        "pre_mission": pre_mission,
        "warmup": warmup,
        "shield": args.shield,
        "pilot_alt_band": pilot_alt_band,
        "shield_lookahead": guard("shield_lookahead", lambda: FV.shield_lookahead_record(shield)),
        "shield_velocity_source": "ground-truth kinematics (Project AirSim), not an EKF",
        # Ticks on which the Shield corrected what FLEW; with the Shield off
        # nothing it decided flew, and its would-be corrections are the
        # shadow's (P1-1).
        "interventions": n_touched if shield_on else 0,
        "shadow_interventions": None if shield_on else n_touched,
        "nfz_s": dwell.get("nfz_s"), "nfz_entered": bool(dwell.get("nfz_ticks")),
        "alt_violation_s": round(alt_bad * TICK, 1),
        "off_map_ticks": (sum(1 for r in rows if r.get("off_map")) if shield.has_obstacle_map
                          else None),
        "obstacle_map_loaded": bool(shield.has_obstacle_map), "occ_band_report": occ_report,
        "collisions": guard("collisions", lambda: FV.collisions_summary(
            collisions, collisions_subscribed,
            None if t0 is None or t_connect is None else t0 - t_connect,
            None if t_mission_end is None or t_connect is None else t_mission_end - t_connect)),
        "landing": landing_info,
        "frames": {"capture": frames_stats, "saved": saver_stats},
        "policy_hud": (None if pind is None else {"counts": dict(pind.counts),
                                                  "banner_ticks": dict(pind.banner_ticks),
                                                  "error": pind_error,
                                                  "decision_read": ("the Shield's" if shield_on
                                                                    else "shadow, no repairs: "
                                                                    "vla_mission.hud_decision")}),
        "policy_source": policy_source,
        "previous_run_cleared": cleared,
        "params": {"cadence": args.cadence, "decision_timeout_s": args.decision_timeout_s,
                   "cruise_mps": args.cruise_mps, "speed_cap_mps": args.speed_cap_mps,
                   "warmup_max_latency_s": args.warmup_max_latency_s,
                   "late_reply_wait_s": args.late_reply_wait_s,
                   "start_tolerance_m": args.start_tolerance_m,
                   "det_fresh_s": args.det_fresh_s,
                   "tracker": {k: getattr(tcfg, k) for k in tcfg.__dataclass_fields__},
                   "citymap": str(args.citymap), "seed": args.seed},
    }
    metrics["pilot"] = guard("pilot_entry", lambda: VM.pilot_entry(
        identity, stats, records, rows, server=server_rec,
        reported_by=f"{server.host}:{server.port}", weight_paths=weight_paths,
        cadence=args.cadence, warmup=warmup,
        # The configuration this script asked for (None when attached: then
        # the server's own identity, above, says what it runs).
        vla_config=(args.vla_config if server.started_here and not args.vla_fake_output
                    else None),
        vla_fake_output=args.vla_fake_output, stand_in=not is_vla),
        {"kind": "unrecorded", "server_identity": identity_of(identity), "server": server_rec})
    if not is_vla and isinstance(metrics["pilot"], dict):
        metrics["pilot"]["warning"] = ("a scripted stand-in answered: this run tests the "
                                       "flight path, it is NOT a VLA flight")
    manifest = guard("manifest", lambda: VM.manifest_for(
        policy.policy_hash, identity, args.seed, SIM_CONFIG_DIR / SCENES[args.mission],
        weight_paths))
    kpis = {}
    if manifest is not None:
        kpis = guard("finish_run", lambda: VM.finish_run(out, rows, metrics, manifest, policy),
                     {}) or {}
    if records_errors or not (out / "metrics.json").is_file():
        # The last resort: whatever was built, with what failed.
        metrics["records_errors"] = records_errors
        try:
            _write_json_lenient(out / "metrics.json", metrics)
        except Exception as exc:                                   # noqa: BLE001
            print(f"[records] *** metrics.json could not be written: {type(exc).__name__}: {exc}")
        if manifest is not None and not (out / "manifest.json").is_file():
            try:
                _write_json_lenient(out / "manifest.json", manifest)
            except Exception as exc:                               # noqa: BLE001
                print(f"[records] *** manifest.json could not be written: {exc}")
    guard("trajectory.json", lambda: VM.write_json(out / "trajectory.json", traj))
    try:
        bp = write_replay(out, policy, out / f"{args.tag}.replay.tar.gz",
                          changelog=f"VLA flight {args.tag}", policy_source=policy_source)
        ok, why = verify_replay(bp)
        print(f"[replay] {bp.name} {'re-derives its KPIs' if ok else 'FAILED verification'}")
    except Exception as exc:                                       # noqa: BLE001
        print(f"[replay] not written: {type(exc).__name__}: {exc}")

    try:
        s = (metrics.get("pilot") or {}).get("summary") or {}
        lat = s.get("latency_client_s") or {}
        print(f"\n[report] {args.mission}: {len(rows)} ticks, end {end_reason}, aborted {aborted}"
              + (", INTERRUPTED" if interrupted else "")
              + (f", REFUSED ({refused['kind']})" if refused else ""))
        print(f"[report] cadence {args.cadence}; decisions {s.get('decisions_accepted')} at "
              f"{s.get('decision_hz')} Hz; latency p50 {lat.get('median')} s p95 {lat.get('p95')} s "
              f"(capture -> answer); parse_fail {s.get('parse_fail')}, server_error "
              f"{s.get('server_error')}, link_failure {s.get('link_failure')} "
              f"{s.get('link_failure_by_kind')}, LAND {s.get('land')}")
        print(f"[report] ticks by author {s.get('ticks_by_author')}; VLA-authored share "
              f"{s.get('vla_authored_tick_share')}")
        print(f"[report] reached_goal {metrics.get('reached_goal')}; P0 escape rate "
              f"{kpis.get('p0_violation_escape_rate')}; interventions {metrics['interventions']}"
              + ("" if shield_on else f" (shadow {metrics['shadow_interventions']})")
              + f"; nfz_s {metrics['nfz_s']}")
        print(f"[report] manifest vla_model_hash {(manifest or {}).get('vla_model_hash')}")
        if not is_vla:
            print("[report] *** a STAND-IN answered: not a VLA flight ***")
        if records_errors:
            print(f"[report] *** RECORDS INCOMPLETE: {records_errors}")
    except Exception as exc:                                       # noqa: BLE001
        print(f"[report] *** the report failed: {type(exc).__name__}: {exc}")
    if interrupted:
        code = EXIT_INTERRUPTED
    elif refused is not None:
        code = EXIT_REFUSED
    elif aborted or not rows or records_errors:
        code = EXIT_ABORTED
    elif end_reason in VM.VLA_FAILURE_ENDS:
        code = EXIT_VLA_FAILURE
    else:
        code = EXIT_OK
    live.stop("ok" if code == 0 else ("stopped" if interrupted else "failed"),
              detail=None if code == 0 else (
                  "interrupted (Ctrl+C)" if interrupted else
                  f"refused ({refused['kind']})" if refused else
                  f"aborted ({str(aborted).split(':', 1)[0]})" if aborted else
                  "records incomplete" if records_errors else f"VLA ended {end_reason}"))
    return code


def main(argv: list[str] | None = None) -> int:
    ap = build_arg_parser()
    args = parse_cli(ap, argv)
    if args.check_args:
        print(json.dumps(check_args_report(args)))
        return 0
    import follow_vlm as FV
    fine = FV._fine_timer_begin()
    try:
        return asyncio.run(fly(args))
    except KeyboardInterrupt:
        # asyncio.run re-raises Ctrl+C after fly() has landed and written its
        # records (marked interrupted); the exit code says so.
        print("[vla] interrupted (Ctrl+C): the records say interrupted", file=sys.stderr)
        return EXIT_INTERRUPTED
    finally:
        FV._fine_timer_end(fine)


if __name__ == "__main__":
    code = main()
    # As follow_vlm: everything is on disk by now; the simulator client's
    # receive threads are not daemons and may hold the interpreter open.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code or 0)
