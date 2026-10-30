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
    # a follow fine-tune's checkpoint instead of upstream's adapter; the follow
    # flies its step tracker smoothed unless --no-smooth (TrackerConfig)
    python demo/vla_fly.py --mission follow --cadence pipelined \\
        --vla-adapter D:/models/aerialvla-ft-follow/<run>/epoch<N> ...
    # a checkpoint trained on a prev_front dataset, its steps flown as rates
    python demo/vla_fly.py --mission follow --cadence pipelined \\
        --vla-adapter ... --vla-bottom-tile prev_front --step-as-rate 1.0 ...
    # ... and on a --speed-words dataset: its own speed as words in the prompt
    python demo/vla_fly.py ... --vla-bottom-tile prev_front --step-as-rate 1.0 \\
        --vla-speed-words ...
    # the whole flight path without the 7B model: the scripted server (its
    # identity is fake:scripted; the run is recorded as a stand-in, never a VLA)
    python demo/vla_fly.py --mission goto --vla-fake-output "96 48 49</s>" --tag vla_dry
    # a foreign drone VLA behind demo/vla_server_foreign.py (attached; it is
    # recorded as itself, dronecats:... / racevla:...), the detector's frames
    # as its front image when its server was started --front-source detector
    python demo/vla_fly.py --mission follow --cadence pipelined \\
        --vla-server 127.0.0.1:8791 --vla-front-source detector ...

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
  M3 follow    the level's car (--level-car, default Car_10), chased on
               OWL-ViT's phrase as M2's. Which box is the car is follow_vlm's
               perception: TargetLock, the identity check with its strict lock
               and follow_vlm's Reacquirer as the start gate, re-acquiring at
               once on a lapse from the last sighting and, after a long loss,
               on the start gate's evidence again, a pass allowed gaps of
               --acquire-gap-s (_acquire_step); without identity the
               DetectorHint's gate (follow_perception). The
               Shield pinning the aircraft or a close box with no request
               resets that perception (_perception_reset). The level car's
               pose scores the run (frac_within_30m at the top of metrics, as
               follow_vlm's) and reaches nothing else. Ends on the target
               lost, nothing ever asked (subject_not_acquired), or time.

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

THE FOLLOW FINE-TUNE'S INPUT AND OUTPUT (2026-10-24, both off by default):
--vla-bottom-tile prev_front puts the front frame --vla-prev-dt-s (1.0 s)
earlier in the mosaic's lower tile instead of the down camera
(vla_mission.VLAFrames; aerialvla_codec, THE MOSAIC'S LOWER TILE), and the
flight is refused before take-off (exit 4) when the adapter hello announces
was trained on another tile (vla_mission.bottom_tile_check; an adapter that
records none is "down"). --step-as-rate H flies each step as a velocity,
fwd/H and dpsi/H, until the next answer replaces it (vla_link.TrackerConfig,
RATE EXECUTION). Both go into metrics.params; every request's record says
its lower tile and every row its commanded rates. --vla-speed-words
appends the aircraft's own horizontal speed as words to the object ("a red
car while flying fast"; aerialvla_codec, THE AIRCRAFT'S OWN SPEED: a 0.5 s
backward difference of the pose ring at or before the capture, four
classes - the dataset builder's --speed-words rule), refused before
take-off (exit 4) when the adapter's provenance says otherwise either way
(vla_mission.speed_words_check; an adapter that records none is off);
every request's record says its speed and words (vla_steps.jsonl
`own_speed`), metrics.params the check.

THE FRONT IMAGE (2026-10-27, default unchanged): --vla-front-source vla
sends VLAFront (256x256, AerialVLA's training camera); detector sends the
detector's FrontCamera frame (768x432, 90 deg, pitched -20 deg; approach and
follow only, the scenes that have it) for a foreign model that wants a
larger image (DroneCATS: at most 640 px wide, the paper's 640x360 at 90
deg; its server downscales). The camera's field of view and pitch are the
server's intrinsics, so the flight is refused before take-off (exit 4)
unless the server reads that camera: AerialVLA and its fine-tunes read
VLAFront, a foreign server declares its own (vla_mission.
front_source_check). metrics.params says which, and VLAFrames' stats the
sizes sent.

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
# The follow's step-tracker smoothing (TrackerConfig.smooth_accel_mps2,
# smooth_yaw_accel_rps2, carry_s; build_arg_parser says where each comes from).
DEFAULT_SMOOTH = {"accel": 2.5, "yaw_accel": 0.6, "carry_s": 1.0}


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
    ap.add_argument("--follow-memory-s", type=float, default=6.0,
                    help="M3: past the box's freshness, keep the phrase of its last bearing "
                         "(world frame, re-read against the current heading) this long")
    ap.add_argument("--follow-search-s", type=float, default=20.0,
                    help="M3: then turn toward the side it was last seen on this long")
    ap.add_argument("--close-bottom-frac", type=float, default=0.92,
                    help="M3: a box whose bottom edge is this low in the image is close: no new "
                         "step, horizontal motion held")
    ap.add_argument("--close-width-frac", type=float, default=0.25,
                    help="M3: a box at least this wide (fraction of the image) is close")
    ap.add_argument("--lock-gate", type=float, default=0.12,
                    help="M3: follow_vlm's TargetLock on the detector (one instance, not the "
                         "most salient box this frame): max jump from its prediction, as a "
                         "fraction of the image width (the earlier city follows' --lock-target)")
    ap.add_argument("--start-gate-n", type=int, default=5,
                    help="M3: no VLA request until this many successive detector frames gave a "
                         "near box (follow_vlm's --start-when-seen); 0 disables")
    ap.add_argument("--start-gate-width", type=float, default=0.04,
                    help="M3: a near box is at least this wide (fraction of the image; a car "
                         "about 20 m away)")
    ap.add_argument("--no-lock", action="store_true",
                    help="M3: no TargetLock (each frame's best box, as M2)")
    ap.add_argument("--acquire-max-range-m", type=float, default=30.0,
                    help="M3 with identity: the start gate / re-acquisition accepts a subject "
                         "this close (run_citylife_follow.ps1 -StartMaxRange 30)")
    ap.add_argument("--acquire-gap-s", type=float, default=ACQUIRE_GAP_S,
                    help="M3 with identity: while the start gate's moving test applies (the "
                         "start, a long loss), a pass of sightings survives inferences with "
                         "no acceptable candidate this long (follow_vlm.Reacquirer gap_s); "
                         "0 = follow_vlm's rule, one empty frame ends it")
    ap.add_argument("--gate-truth", action="store_true",
                    help="M3: the detector the start gate and re-acquisition judge is fed the "
                         "level car's TRUE position (follow_vlm --teacher-truth's box), for "
                         "targets the hand-written detector cannot find (a white or a black "
                         "car, 30 Oct). The VLA flies on its own perception; metrics.json "
                         "params.follow.gate_truth says so")
    ap.add_argument("--no-identity", action="store_true",
                    help="M3: no identity check on the detector's candidates (follow_vlm "
                         "--identity: physical size from depth, the street mask)")
    ap.add_argument("--no-stuck-reset", action="store_true",
                    help="M3: never reset the detector lock when the Shield pins the aircraft")
    ap.add_argument("--close-stall-s", type=float, default=8.0,
                    help="M3: a close box with no request sent this long resets the detector "
                         "lock / re-acquires, recorded (a wrong large box held in a hover); "
                         "0 disables")
    ap.add_argument("--no-follow-executor", action="store_true",
                    help="M3: fly upstream's execution (rotate in place on a big turn, the "
                         "model's altitude) instead of turning while moving at a held altitude")
    # SMOOTHING (guardrail/vla_link.TrackerConfig, SMOOTHING): on by default
    # for the follow executor, never for M1 / M2. The defaults are follow_vlm's
    # --accel-max (2.5 m/s^2) and the hand-written follow's measured yaw
    # acceleration p99 (0.50 rad/s^2, 19 Oct; 0.6 the target set for the
    # VLA follow), and about one decision interval of carry.
    ap.add_argument("--smooth", action=argparse.BooleanOptionalAction, default=None,
                    help="M3: the step tracker's smoothing - an acceleration limit on the "
                         "commanded velocity and yaw rate, and no stop between steps while the "
                         "next answer is pending (default on with the follow executor; "
                         "--no-smooth flies the tracker as before)")
    ap.add_argument("--smooth-accel", type=float, default=DEFAULT_SMOOTH["accel"],
                    help="m/s^2 on the commanded horizontal velocity (default %(default)s)")
    ap.add_argument("--smooth-yaw-accel", type=float, default=DEFAULT_SMOOTH["yaw_accel"],
                    help="rad/s^2 on the commanded yaw rate (default %(default)s)")
    ap.add_argument("--carry-s", type=float, default=DEFAULT_SMOOTH["carry_s"],
                    help="s: a step flown to its end while the next answer is pending keeps "
                         "its velocity, decaying to zero over this (default %(default)s)")
    # RATE EXECUTION (guardrail/vla_link.TrackerConfig.rate_horizon_s; off by
    # default): the follow fine-tune's label is the expert's displacement over
    # its horizon (tools/build_follow_dataset.py --horizon-s), a velocity.
    ap.add_argument("--step-as-rate", type=float, default=None, metavar="H",
                    help="M3: fly each step as a velocity - fwd/H m/s along the heading, "
                         "dpsi/H rad/s until the heading is reached - until the next answer "
                         "replaces it (past 1.2 H the carry or the hold ends it), instead of "
                         "an endpoint at the cruise speed. H = the dataset's --horizon-s "
                         "(1.0). Default off")
    ap.add_argument("--rate-yaw", choices=("heading", "rate"), default="heading",
                    help="with --step-as-rate: 'heading' (default) turns to psi_capture + dpsi "
                         "at dpsi/H and holds it; 'rate' flies dpsi/H as the yaw rate until the "
                         "next answer (a model whose label is a yaw-rate command x H, aiotvla; "
                         "guardrail/vla_link.TrackerConfig.rate_yaw)")
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
    ap.add_argument("--first-decision-timeout-s", type=float, default=None,
                    help="no parsed answer this long after the mission start ends it "
                         "(default 60 s; a follow's: its lost timeout + 60 s, since its start "
                         "gate may wait for the subject)")
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
    ap.add_argument("--sim-address", default="127.0.0.1",
                    help="host of the Project AirSim server (topics 8989, services 8990); "
                         "another machine's address when this flight runs on the Jetson "
                         "Orin and the simulator on the desktop (follow_vlm --sim-address)")
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
    ap.add_argument("--vla-adapter", default=None, metavar="DIR",
                    help="the LoRA the started server loads instead of upstream's aero_vla - "
                         "a follow fine-tune's checkpoint (training/finetune_aerialvla_follow.py), "
                         "announced and recorded under its own id")
    # THE LOWER TILE (guardrail/aerialvla_codec, BottomTile; 2026-10-24).
    ap.add_argument("--vla-bottom-tile", choices=("down", "prev_front"), default="down",
                    help="the mosaic's lower tile: 'down' (upstream's: the down camera) or "
                         "'prev_front' (the front frame --vla-prev-dt-s earlier: a car's "
                         "motion in one input). Must be the tile the adapter was trained on "
                         "(its provenance, announced in hello): refused before take-off "
                         "otherwise. Default down")
    ap.add_argument("--vla-prev-dt-s", type=float, default=None,
                    help="with --vla-bottom-tile prev_front: how many seconds earlier (default "
                         "1.0; the frame nearest that, within 0.25 s, else no request)")
    # SPEED WORDS (guardrail/aerialvla_codec, SpeedWords; 2026-10-24).
    ap.add_argument("--vla-speed-words", action="store_true",
                    help="append the aircraft's own horizontal speed as words to the object "
                         "('a red car while flying fast': a 0.5 s backward difference of the "
                         "pose ring at or before each capture, four classes). Must be what the "
                         "adapter was trained with (its provenance, announced in hello): "
                         "refused before take-off otherwise, either way. Default off")
    # THE FRONT IMAGE (aerialvla_codec.FRONT_SOURCES; 2026-10-27).
    ap.add_argument("--vla-front-source", choices=("vla", "detector"), default="vla",
                    help="the request's front frame: 'vla' (VLAFront, 256x256, AerialVLA's "
                         "training camera) or 'detector' (the detector's FrontCamera, "
                         "768x432, pitched -20 deg; approach / follow) for a foreign model "
                         "that wants a larger image. Must be the camera the server reads "
                         "(hello): refused before take-off otherwise. Default vla")
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


def front_camera(args) -> str:
    """The camera whose frames go in the request's front slot
    (--vla-front-source): VLAFront, or the detector's FrontCamera."""
    return CAMERAS["detector"] if args.vla_front_source == "detector" else CAMERAS["front"]


def resolve_args(args):
    """Fill the mission defaults and refuse combinations that cannot fly
    honestly. ValueError says why."""
    m = args.mission
    args.tag = args.tag or f"vla_{m}"
    if args.lost_timeout_s is None:
        args.lost_timeout_s = LOST_TIMEOUT_S[m]
    if args.first_decision_timeout_s is None:
        args.first_decision_timeout_s = (args.lost_timeout_s + 60.0 if m == "follow" else 60.0)
    if args.start_gate_n < 0 or not 0 < args.start_gate_width < 1:
        raise ValueError("--start-gate-n must be >= 0 and --start-gate-width in (0, 1)")
    if m != "follow" and args.level_car is not None:
        raise ValueError("--level-car is for --mission follow")
    # The follow executor (TrackerConfig.turn_while_moving + hold_altitude_m):
    # on for a follow unless --no-follow-executor; never for M1 / M2.
    args.follow_executor = m == "follow" and not args.no_follow_executor
    # Its smoothing (TrackerConfig, SMOOTHING): on with the follow executor
    # unless --no-smooth; refused for M1 / M2, which keep the tracker's motion.
    if args.smooth and not args.follow_executor:
        raise ValueError("--smooth is the follow executor's (--mission follow, without "
                         "--no-follow-executor): goto and approach keep the tracker's motion")
    args.smooth = args.follow_executor if args.smooth is None else bool(args.smooth)
    for k in ("smooth_accel", "smooth_yaw_accel"):
        v = getattr(args, k)
        if not (math.isfinite(v) and v > 0):
            raise ValueError(f"--{k.replace('_', '-')} must be a positive number")
    if not (math.isfinite(args.carry_s) and args.carry_s >= 0):
        raise ValueError("--carry-s must be 0 (no carry) or more")
    # RATE EXECUTION: the follow fine-tune's (its labels are the expert's
    # motion over a horizon); goto and approach fly upstream's step.
    if args.step_as_rate is not None:
        if m != "follow":
            raise ValueError("--step-as-rate is the follow fine-tune's execution (its labels "
                             "are the expert's motion over a horizon): goto and approach fly "
                             "upstream's step")
        if not (math.isfinite(args.step_as_rate) and args.step_as_rate > 0):
            raise ValueError("--step-as-rate must be a positive number of seconds")
    if getattr(args, "rate_yaw", "heading") != "heading" and args.step_as_rate is None:
        raise ValueError("--rate-yaw rate flies a rate step: it needs --step-as-rate")
    # THE LOWER TILE (aerialvla_codec.BottomTile), checked against the
    # adapter's own after hello.
    from guardrail.aerialvla_codec import bottom_tile
    if args.vla_bottom_tile != "prev_front" and args.vla_prev_dt_s is not None:
        raise ValueError("--vla-prev-dt-s is for --vla-bottom-tile prev_front")
    try:
        args.bottom_tile = bottom_tile(args.vla_bottom_tile, args.vla_prev_dt_s)
    except ValueError as exc:
        raise ValueError(f"--vla-bottom-tile / --vla-prev-dt-s: {exc}")
    # SPEED WORDS (aerialvla_codec.SpeedWords, None = off), checked against
    # the adapter's own after hello.
    from guardrail.aerialvla_codec import speed_words
    args.speed_words = speed_words(bool(args.vla_speed_words))
    # THE FRONT IMAGE: the detector's camera exists in the approach and
    # follow scenes only (robot_vla_quad_det.jsonc); checked against the
    # server's after hello.
    if args.vla_front_source == "detector" and m not in VM.DETECTOR_MISSIONS:
        raise ValueError(f"--vla-front-source detector sends the detector's camera "
                         f"({CAMERAS['detector']}), which only the "
                         f"{'/'.join(VM.DETECTOR_MISSIONS)} scenes have")
    if args.vla_adapter is not None and (args.vla_server or args.vla_fake_output):
        raise ValueError("--vla-adapter is the adapter of a server this flight STARTS: an "
                         "attached server flies whatever it was started with (its hello's "
                         "identity names it), the scripted stand-in has none")
    for k in ("follow_memory_s", "follow_search_s"):
        if not (getattr(args, k) >= 0 and math.isfinite(getattr(args, k))):
            raise ValueError(f"--{k.replace('_', '-')} must not be negative")
    for k in ("close_bottom_frac", "close_width_frac"):
        if not 0 < getattr(args, k) <= 1:
            raise ValueError(f"--{k.replace('_', '-')} must be in (0, 1]")
    # Review of d6e7123 (P3): the follow's perception flags, validated, and
    # refused where the flight could not do what they say (the records then
    # disagreed with what flew).
    if not (math.isfinite(args.lock_gate) and 0 < args.lock_gate < 1):
        raise ValueError("--lock-gate must be in (0, 1): a fraction of the image width")
    if not (math.isfinite(args.acquire_max_range_m) and args.acquire_max_range_m > 0):
        raise ValueError("--acquire-max-range-m must be a positive number of metres")
    if not (math.isfinite(args.acquire_gap_s) and args.acquire_gap_s >= 0):
        raise ValueError("--acquire-gap-s must be 0 (follow_vlm's rule) or more seconds")
    if not (math.isfinite(args.close_stall_s) and args.close_stall_s >= 0):
        raise ValueError("--close-stall-s must be 0 (off) or more")
    if m == "follow" and not args.no_identity:
        if args.no_lock:
            raise ValueError("--no-lock needs --no-identity: the identity check works through "
                             "follow_vlm's strict TargetLock (follow_vlm --identity), so there "
                             "is no identity without a lock")
        if args.start_gate_n == 0:
            raise ValueError("--start-gate-n 0 needs --no-identity: with identity the start "
                             "gate is the acquisition that seeds the strict lock (follow_vlm's "
                             "Reacquirer, at least one frame)")
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


def smooth_record(args) -> dict:
    """The follow's smoothing as this flight applies it (TrackerConfig's
    SMOOTHING fields), for --check-args and metrics.params."""
    on = bool(getattr(args, "smooth", False))
    return {"on": on,
            "accel_mps2": args.smooth_accel if on else None,
            "yaw_accel_rps2": args.smooth_yaw_accel if on else None,
            "carry_s": args.carry_s if on else 0.0}


def tracker_config(args, pilot_band, TrackerConfig):
    """The step tracker's configuration for this flight: upstream's
    execution for M1 / M2; the follow executor (turn while moving, the
    altitude held, steps anchored at the answer, LAND as arrival) and, unless
    --no-smooth, its smoothing for M3; with --step-as-rate H (M3) each step
    flown as a velocity over H (TrackerConfig, RATE EXECUTION)."""
    s = smooth_record(args)
    fe = args.follow_executor
    return TrackerConfig(cruise_mps=args.cruise_mps, speed_cap_mps=args.speed_cap_mps,
                         turn_while_moving=fe, anchor_at_answer=fe, land_as_arrival=fe,
                         hold_altitude_m=args.cruise_alt if fe else None,
                         decision_timeout_s=args.decision_timeout_s, alt_band=pilot_band,
                         smooth_accel_mps2=s["accel_mps2"],
                         smooth_yaw_accel_rps2=s["yaw_accel_rps2"], carry_s=s["carry_s"],
                         rate_horizon_s=getattr(args, "step_as_rate", None),
                         rate_yaw=getattr(args, "rate_yaw", "heading"))


def attitude_fields(kin, FV) -> dict:
    """pitch_deg / roll_deg of a kinematics read, as follow_vlm's rows carry
    them (FV.quat_pitch / quat_roll of the NED quaternion: pitch positive
    nose up, roll positive right wing down). Until 2026-10-23 vla_fly's rows
    had none, so tools/compare_motion.py read 0 tilt and 0 deg/s for every
    VLA run - a zero nobody measured. None when the read has no
    orientation."""
    try:
        q = kin["pose"]["orientation"]
        return {"pitch_deg": round(math.degrees(FV.quat_pitch(q)), 2),
                "roll_deg": round(math.degrees(FV.quat_roll(q)), 2)}
    except (KeyError, TypeError, ValueError):
        return {"pitch_deg": None, "roll_deg": None}


def motion_record(rows: list[dict]) -> dict | None:
    """metrics["motion"]: tools/compare_motion.motion_summary of the flight
    log (the hand-written follow's smoothness definitions), or None for a
    log too short to measure."""
    tools = str(ROOT / "tools")
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import compare_motion
    return compare_motion.motion_summary(rows) if len(rows) >= 2 else None


def check_args_report(args) -> dict:
    """What --check-args prints: the resolved flight, nothing started."""
    return {"ok": True, "mission": args.mission, "tag": args.tag,
            "goal_xy": list(args.goal) if args.goal else None,
            "start_xy": list(args.start_xy), "park_at": list(args.park_at) if args.park_at else None,
            "object": args.object, "shield": args.shield, "cadence": args.cadence,
            "smooth": smooth_record(args), "step_as_rate_s": args.step_as_rate,
            "rate_yaw": getattr(args, "rate_yaw", "heading"),
            "bottom_tile": args.bottom_tile.as_dict(),
            "speed_words": None if args.speed_words is None else args.speed_words.as_dict(),
            "vla_server": args.vla_server, "vla_config": args.vla_config,
            "vla_adapter": args.vla_adapter,
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
                          fake_output=args.vla_fake_output, fake_delay_s=args.vla_fake_delay_s,
                          adapter=getattr(args, "vla_adapter", None))
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


# follow_vlm --lapse-s's default, past TargetLock's 2 s hold: a committed
# subject with no accepted box this long has lapsed (_acquire_step).
LAPSE_S = 3.0
# --acquire-gap-s's default: a pass of sightings at the start gate or after a
# long loss survives what a committed subject survives before it lapses
# (follow_vlm.Reacquirer gap_s). vla_follow_ft_07_r3_off (2026-10-24) never
# took the car back: its two passes past the hovering aircraft were cut
# after 7 and 9 OK inferences by gaps of ~2 s, short of the 10 in a row the
# moving test needs at the ~2.2 Hz the detector ran while acquiring.
ACQUIRE_GAP_S = LAPSE_S


def make_acquirer(args, FV):
    """The follow's start gate and re-acquisition with identity, one object
    for the flight: follow_vlm's Reacquirer with --start-gate-n sightings,
    --acquire-max-range-m and --acquire-gap-s (started by the caller)."""
    return FV.Reacquirer(need=args.start_gate_n, max_range_m=args.acquire_max_range_m,
                         gap_s=args.acquire_gap_s)


def new_acquisition(acq, t0: float | None = None) -> dict:
    """The follow's acquisition state for _acquire_step (identity on): the
    Reacquirer `acq` (started by the caller), mission start `t0` (wall
    time: the records' times are mission seconds), and the subject's memory
    since the last commit - follow_vlm's no-estimator path: when it was last
    seen (`seen_t`) and last seen OK (`t_ok`), the map points of its held
    boxes (`hist`, for the measured speed), the last sighting (`last`: its
    frame's capture time and place), the speeds. `attempts`: one record per
    acquisition (the start gate, each lapse, each perception reset) with
    what every judged frame decided (_open_attempt), `attempt` the open one."""
    return {"acq": acq, "committed": False, "last_seq": -1, "commits": [], "reacquires": 0,
            "lapses": [], "errors": [], "t0": t0, "seen_t": None, "t_ok": -math.inf,
            "last_t_det": None, "hist": [], "last": None, "v": None, "v_prior": None,
            "attempts": [], "attempt": None}


def _open_attempt(acq_box: dict, kind: str, t) -> dict:
    """A new acquisition attempt - "start" (the start gate), "lapse", or
    "reset:<why>" (a perception reset) - from mission second `t`: the frames
    it judged, what each decided (the Reacquirer's last_outcome: "committed"
    or why not, so a pass that never got as far as a refusal is counted
    too), the candidates' refusals (Reacquirer.refused), its best streak and
    when it committed (None: never). vla_follow_ft_07_r3_off spent 600 s
    acquiring and its metrics said only "reacquires 1"; this says why."""
    att = {"t": t, "kind": kind, "judged": 0, "outcomes": {}, "refused": {},
           "best_streak": 0, "committed_s": None}
    acq_box.setdefault("attempts", []).append(att)
    acq_box["attempt"] = att
    return att


def _acquire_step(grounder, acq_box: dict, args, clock=time.time) -> None:
    """M3 with identity, once per tick, perception only: follow_vlm's own
    handling of the subject when it has no estimator (fly(): the start
    gate, the measurement, the lapse, the re-acquisition).

    * Acquiring: each NEW detector frame's candidates step follow_vlm's
      Reacquirer; its pick is committed (the strict lock's seed) and the
      subject's memory moves to it (follow_vlm's _take_subject).
    * Committed: each NEW held box (a new t_det; a SOFT one only within 3 s
      of an OK one) is the measurement: its map point, its frame's capture
      time, the speed over the last 2 s (follow_vlm.measured_speed).
    * The LAPSE: the strict lock answering "no held instance" - its 2 s hold
      over and no estimator prior to stand in (this flight sets none: the
      lock can never take the subject back by itself) - or no accepted box
      for LAPSE_S: re-acquire AT ONCE, as follow_vlm does ("LAPSE: no
      accepted measurement for lapse_s ... reacq.start(anchor);
      grounder.begin_reacquire()"): the Reacquirer anchored at the last
      sighting {t, P, v}, so the car is taken back within `need` frames
      wherever it could have driven since, without the motion test.
    * A LONG loss: once the reach has grown past the Reacquirer's
      reach_cap_m (60 m: 10 m + 3-12 m/s x 4-17 s, always inside memory_s +
      search_s, 26 s, at the defaults) the anchor no longer says where the
      car is, and the start gate's evidence is asked again - identity OK and
      SEEN MOVING over 2 x need sightings, the pass allowed gaps of up to
      --acquire-gap-s (make_acquirer). vla_follow_ft_07_r3_off (2026-10-24)
      hovered 600 s acquiring while the car drove past twice: one or three
      empty frames ended each pass short of the 10 sightings in a row.

    Every judged frame is counted in the open attempt (_open_attempt): what
    the Reacquirer decided and the candidates' refusals, for metrics.

    Until the review of d6e7123 (P2-3) the re-acquisition waited for
    memory_s + search_s (26 s) unseen, counted only once the Grounder had
    dropped the stale box (8 s): #8 spent 1060 of its 1306 detector frames
    acquiring and #9 hovered 54 % of the time. The hint's memory and search
    keep the phrases coming meanwhile (DetectorHint). `args` is unused since
    (kept for the callers); `clock` is the wall clock of the Grounder."""
    import follow_vlm as FV
    gw = grounder.latest()
    if gw.get("seq") == acq_box["last_seq"]:
        return
    acq_box["last_seq"] = gw.get("seq")
    now = clock()
    t0 = acq_box.get("t0")

    def mt(t):
        return round(t - t0, 3) if t0 is not None else round(t, 3)
    if not acq_box["committed"]:
        att = acq_box.get("attempt") or _open_attempt(acq_box, "start", mt(now))
        if not (gw.get("t") and now - gw["t"] < 1.0):
            att["outcomes"]["stale frame"] = att["outcomes"].get("stale frame", 0) + 1
            return
        ic = gw.get("inf_cap") or {}
        t_ic = ic.get("t_capture") or ic.get("t_cap") or now
        acq = acq_box["acq"]
        before = dict(getattr(acq, "refused", None) or {})
        got = acq.step(gw.get("cands") or [], gw.get("cand_tiers") or [],
                       gw.get("cand_feats") or [], t_ic)
        out = "committed" if got else (getattr(acq, "last_outcome", None) or "unknown")
        att["judged"] += 1
        att["outcomes"][out] = att["outcomes"].get(out, 0) + 1
        for k, n in (getattr(acq, "refused", None) or {}).items():
            if n != before.get(k, 0):
                att["refused"][k] = att["refused"].get(k, 0) + n - before.get(k, 0)
        att["best_streak"] = max(att["best_streak"], int(getattr(acq, "streak", 0) or 0))
        if got:
            grounder.commit(acq.pick, yaw=(ic.get("pose") or {}).get("yaw"))
            acq_box["committed"] = True
            acq_box["commits"].append(mt(now))
            att["committed_s"] = mt(now)
            acq_box["attempt"] = None
            # The subject's memory moves to the committing sighting.
            P = (getattr(acq, "pick_feat", None) or {}).get("P")
            if acq_box["v"] is not None:
                acq_box["v_prior"] = acq_box["v"]
            acq_box["v"] = None                 # a streak's ~3 m jitter is no speed
            acq_box["hist"] = [] if P is None else [(now, float(P[0]), float(P[1]))]
            acq_box["last"] = (None if P is None else
                               {"t": float(t_ic), "P": (float(P[0]), float(P[1]))})
            acq_box["seen_t"] = acq_box["t_ok"] = now
            acq_box["last_t_det"] = gw.get("t_det")
            print("[follow] subject acquired (identity): lock seeded")
        return
    t_det = gw.get("t_det")
    if gw.get("det") is not None and t_det and t_det != acq_box["last_t_det"]:
        acq_box["last_t_det"] = t_det
        tier = gw.get("tier")
        if tier == "ok" or now - acq_box["t_ok"] < 3.0:
            if tier == "ok":
                acq_box["t_ok"] = now
            acq_box["seen_t"] = now
            P = (gw.get("feat") or {}).get("P")
            if P is not None:
                cap = gw.get("cap") or {}
                hist = acq_box["hist"]
                hist.append((now, float(P[0]), float(P[1])))
                del hist[:-40]
                v = FV.measured_speed(hist)
                if v is not None:
                    acq_box["v"] = v
                acq_box["last"] = {"t": float(cap.get("t_capture") or cap.get("t_cap") or now),
                                   "P": (float(P[0]), float(P[1]))}
    unseen = now - (acq_box["seen_t"] if acq_box["seen_t"] is not None else now)
    why = ("lock: no held instance" if gw.get("lock_why") == "no held instance"
           else f"unseen {unseen:.1f} s" if unseen > LAPSE_S else None)
    if why is None:
        return
    last = acq_box["last"]
    v = acq_box["v"] if acq_box["v"] is not None else acq_box["v_prior"]
    anchor = None if last is None else {"t": last["t"], "P": last["P"], "v": v}
    grounder.begin_reacquire()
    acq_box["acq"].start(anchor)
    acq_box["committed"] = False
    acq_box["reacquires"] += 1
    acq_box["lapses"].append({"t": mt(now), "why": why, "unseen_s": round(unseen, 2),
                              "P": None if anchor is None else [round(c, 2) for c in anchor["P"]],
                              "v": None if v is None else round(v, 2)})
    _open_attempt(acq_box, "lapse", mt(now))
    print(f"[follow] subject lost ({why}): re-acquiring from its last sighting")


def _identity_setup(args, sim, drone, obs, FV) -> tuple[dict | None, str | None]:
    """M3 with identity (follow_vlm --identity): the detector camera's depth
    subscribed, demo/identity.py's thresholds and the street mask loaded ->
    (follow_vlm.Grounder's identity configuration, None). Any failure -
    (None, why), never an exception: the follow then flies without identity,
    with the DetectorHint's start gate (follow_perception), and the records
    say why (review of d6e7123, P2-5)."""
    try:
        topic = drone.sensors[CAMERAS["detector"]]["depth_camera"]
        import identity as identity_mod
        th = identity_mod.load_thresholds(None)
        sp = Path(args.citymap).parent / "street.npz"
        sd = identity_mod.StreetDistance.load(sp) if sp.is_file() else None
        if sd is None:
            print(f"[identity] *** no street mask at {sp}: the off-street rules cannot fire")
        cfg = {"thresholds": th, "street": sd, "hfov": FV.CAMERA_HFOV_DEG,
               "hash": identity_mod.thresholds_hash(th),
               "path": str(identity_mod.DEFAULT_THRESHOLDS),
               "nouns": {args.object: identity_mod.noun_of(args.object)}}
        sim.subscribe(topic, lambda _, m: obs.put_depth(m))
    except Exception as exc:                                       # noqa: BLE001
        return None, f"{type(exc).__name__}: {exc}"
    return cfg, None


def follow_perception(args, identity_on: bool, identity_error: str | None = None) -> dict:
    """What a follow's perception applies, decided AFTER the identity setup
    (review of d6e7123, P2-5: the DetectorHint's gate was chosen from
    --no-identity before that setup ran, so a follow whose identity failed
    to load flew with no gate while its params claimed one):

      lock        follow_vlm's TargetLock (strict with identity), unless
                  --no-lock (refused with identity: resolve_args);
      identity    asked for (not --no-identity), on, and why not when it
                  was asked for and failed;
      start_gate  identity on: follow_vlm's Reacquirer (need, range, the
                  pass's gaps), the acquisition that seeds the strict lock;
                  otherwise the DetectorHint's (n successive near boxes)
                  unless --start-gate-n 0; else none."""
    requested = not args.no_identity
    on = bool(identity_on and requested)
    lock = on or not args.no_lock
    if on:
        gate = {"kind": "identity_reacquirer", "need": int(args.start_gate_n),
                "max_range_m": float(args.acquire_max_range_m),
                "gap_s": float(args.acquire_gap_s)}
    elif args.start_gate_n > 0:
        gate = {"kind": "detector_hint", "n": int(args.start_gate_n),
                "width_frac": float(args.start_gate_width)}
    else:
        gate = None
    return {"lock": lock, "lock_gate": float(args.lock_gate) if lock else None,
            "identity": {"requested": requested, "on": on,
                         "error": identity_error if requested and not on else None},
            "start_gate": gate}


def follow_params(args, plan: dict | None) -> dict:
    """params["follow"]: the follow's CONFIGURATION, as applied (`plan`:
    follow_perception's, None when the flight ended before the perception
    was set up). What it measured - the acquisition, the resets, the
    close-hold ticks - is metrics["follow"]'s (follow_measured; review of
    d6e7123, P3)."""
    return {"memory_s": args.follow_memory_s, "search_s": args.follow_search_s,
            "close_bottom_frac": args.close_bottom_frac,
            "close_width_frac": args.close_width_frac,
            "executor": args.follow_executor,
            "perception_applied": plan is not None,
            "lock_gate": None if plan is None else plan["lock_gate"],
            "identity": None if plan is None else dict(plan["identity"]),
            "start_gate": None if plan is None else plan["start_gate"],
            "lapse_s": LAPSE_S, "stuck_reset": not args.no_stuck_reset,
            "close_stall_s": args.close_stall_s,
            "gate_truth": bool(getattr(args, "gate_truth", False))}


def follow_measured(acq_box: dict, resets: list[dict], loop) -> dict:
    """The follow's measured perception outcomes for metrics["follow"], in
    mission seconds: the identity acquisition (None with identity off) -
    commits, lapses and re-acquisitions, and every attempt with what each
    judged frame decided (_open_attempt), summed over the flight in
    `judged`, `outcomes` and `refused` - every perception reset (StuckWatch
    "stuck", CloseStall "close_stall") and the close-hold ticks."""
    acq = None
    if acq_box.get("acq") is not None:
        atts = [dict(a, outcomes=dict(a["outcomes"]), refused=dict(a["refused"]))
                for a in acq_box.get("attempts") or []]
        outcomes: dict = {}
        refused: dict = {}
        for a in atts:
            for total, part in ((outcomes, a["outcomes"]), (refused, a["refused"])):
                for k, n in part.items():
                    total[k] = total.get(k, 0) + n
        acq = {"committed": acq_box["committed"], "commits_s": list(acq_box["commits"]),
               "reacquires": acq_box["reacquires"], "lapses": list(acq_box["lapses"]),
               "errors": list(acq_box["errors"]),
               "judged": sum(a["judged"] for a in atts), "outcomes": outcomes,
               "refused": refused, "attempts": atts}
    return {"acquisition": acq, "perception_resets": list(resets),
            "stuck_resets": sum(1 for e in resets if e.get("why") == "stuck"),
            "close_stalls": sum(1 for e in resets if e.get("why") == "close_stall"),
            "close_hold_ticks": getattr(loop, "n_close_hold", None)}


def _perception_reset(why: str, now: float, t_mission: float, grounder, detector_hint,
                      acq_box: dict, state) -> dict:
    """Drop the subject the follow steers on - the Shield pinning the
    aircraft (StuckWatch, "stuck") or a close box with no request
    (CloseStall, "close_stall"); perception only, the VLA still steers:

      * the hint ignores every box captured up to `now` (DetectorHint.
        forget(now)): the dropped box stays fresh for det_fresh_s and the
        Grounder publishes it up to its drop_after (8 s), and the next hint
        re-seeded the memory with it (review of d6e7123, P2-4);
      * with identity, the re-acquisition starts at once and UNANCHORED
        (begin_reacquire, Reacquirer.start(None), committed False): the box
        was probably the wrong object, so its place is no anchor. lock.reset()
        alone left the strict lock empty while _acquire_step still believed
        it committed - ~26 s with nothing;
      * without identity, the TargetLock forgets its instance.
    Returns the event record (mission time, where)."""
    ev: dict = {"t": t_mission, "why": why, "x": round(state.x, 2), "y": round(state.y, 2)}
    errors = []
    if detector_hint is not None:
        try:
            detector_hint.forget(now)
        except Exception as exc:                                   # noqa: BLE001
            errors.append(f"forget: {type(exc).__name__}: {exc}")
    if acq_box.get("acq") is not None and grounder is not None:
        try:
            grounder.begin_reacquire()
            acq_box["acq"].start(None)
            acq_box["committed"] = False
            _open_attempt(acq_box, f"reset:{why}", t_mission)
            ev["reacquiring"] = True
        except Exception as exc:                                   # noqa: BLE001
            errors.append(f"reacquire: {type(exc).__name__}: {exc}")
    else:
        lk = getattr(grounder, "lock", None) if grounder is not None else None
        if lk is not None:
            try:
                lk.reset()
                ev["lock_reset"] = True
            except Exception as exc:                               # noqa: BLE001
                errors.append(f"lock: {type(exc).__name__}: {exc}")
    if errors:
        ev["errors"] = errors
    return ev


def _follow_watch(res, state, stuck, close_watch, requests) -> str | None:
    """Which perception reset this tick calls for, if any: "stuck" when the
    Shield has pinned what FLEW (res.hud_decision.touched - with the Shield
    off nothing it decided flew, and res.decision is the shadow's; review of
    d6e7123, P3), "close_stall" when a close box has lasted CloseStall's
    max_s with no request (the row's close_box, the backend's count)."""
    if stuck is not None and stuck.update(res.row["t"], state.x, state.y,
                                          bool(res.hud_decision.touched)):
        return "stuck"
    if close_watch is not None and close_watch.update(res.row["t"],
                                                      bool(res.row.get("close_box")), requests):
        return "close_stall"
    return None


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
    # The lower tile the adapter was trained on must be this flight's
    # (aerialvla_codec, THE MOSAIC'S LOWER TILE): refused before take-off.
    tile_check = VM.bottom_tile_check(identity, args.bottom_tile, is_vla=is_vla)
    if not tile_check["ok"]:
        server.stop()
        live.stop("failed", detail="the adapter's lower tile is not this flight's")
        print(f"[vla] *** refused before take-off: {tile_check['why']} (--vla-bottom-tile "
              f"{args.bottom_tile.mode}; the adapter's from {tile_check['source']})",
              file=sys.stderr)
        raise SystemExit(EXIT_REFUSED)
    # The speed words the adapter was trained with must be this flight's,
    # either way round (aerialvla_codec, THE AIRCRAFT'S OWN SPEED): refused
    # before take-off.
    sw_check = VM.speed_words_check(identity, args.speed_words, is_vla=is_vla)
    if not sw_check["ok"]:
        server.stop()
        live.stop("failed", detail="the adapter's speed words are not this flight's")
        print(f"[vla] *** refused before take-off: {sw_check['why']} (--vla-speed-words "
              f"{'on' if args.speed_words else 'off'}; the adapter's from "
              f"{sw_check['source']})", file=sys.stderr)
        raise SystemExit(EXIT_REFUSED)
    # The front camera the server reads must be the one this flight sends
    # (--vla-front-source; vla_mission.front_source_check): refused before
    # take-off.
    front_check = VM.front_source_check(identity, args.vla_front_source, is_vla=is_vla)
    if not front_check["ok"]:
        server.stop()
        live.stop("failed", detail="the server reads another front camera")
        print(f"[vla] *** refused before take-off: {front_check['why']} (--vla-front-source "
              f"{args.vla_front_source})", file=sys.stderr)
        raise SystemExit(EXIT_REFUSED)
    print(f"[vla] front image {front_camera(args)} (--vla-front-source {args.vla_front_source})"
          + (f", the server's ({front_check['source']})" if front_check["checked"]
             else f" - {front_check['source']}"))
    print(f"[vla] speed words {'on' if args.speed_words else 'off'}"
          + (f" ({args.speed_words.window_s:g} s window)" if args.speed_words else "")
          + (f", the adapter's ({sw_check['source']})" if sw_check["checked"]
             else f" - {sw_check['source']}"))
    print(f"[vla] lower tile {args.bottom_tile.mode}"
          + (f" (the front frame {args.bottom_tile.dt_s:g} s earlier)"
             if args.bottom_tile.prev_front else "")
          + (f", the adapter's ({tile_check['source']})" if tile_check["checked"]
             else f" - {tile_check['source']}")
          + (f"; steps flown as rates over {args.step_as_rate:g} s" if args.step_as_rate
             else ""))
    weight_paths = None if (args.no_local_weights or not is_vla) else VM.local_weight_paths(identity)

    backend = saver = None
    try:
        client = RemoteVLA(server.host, server.port, response_timeout_s=args.decision_timeout_s,
                           strict_identity=is_vla, late_reply_wait_s=args.late_reply_wait_s,
                           clock=time.time)
        obs = SemanticObs()
        saver = VM.FrameSaver(out / "vla_frames")
        frames = VM.VLAFrames(obs, saver=saver, bottom_tile=args.bottom_tile,
                              speed_words=args.speed_words)
        det_source = detector_hint = grounder = close_rule = None
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
            if args.mission == "follow":
                close_rule = VM.BoxClose(args.close_bottom_frac, args.close_width_frac)
                # Built ungated: the start gate is armed once the identity
                # setup has run, which may fail (follow_perception; review of
                # d6e7123, P2-5). Each hint records its kind (P2-2).
                detector_hint = VM.DetectorHint(latest_box, bridge, fresh_s=args.det_fresh_s,
                                                memory_s=args.follow_memory_s,
                                                search_s=args.follow_search_s,
                                                close=close_rule, record_kind=True)
            else:
                detector_hint = VM.DetectorHint(latest_box, bridge, fresh_s=args.det_fresh_s)
            hint_src = VM.HintSource(detector_hint, "detector")
        tcfg = tracker_config(args, pilot_band, TrackerConfig)
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
        # --sim-address: the simulator on another machine (the Jetson Orin
        # flying against the desktop's Project AirSim).
        sim = ProjectAirSimClient(address=args.sim_address)
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
    gate_truth_box: dict = {}
    level_truth_errors: list[str] = []
    resets: list[dict] = []                    # perception resets (stuck, close_stall)
    identity_rec: dict = {"on": False, "hash": None}
    follow_plan = None                         # follow_perception's, once set up
    acq_box: dict = new_acquisition(None)
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
        # --vla-front-source detector: the detector's frame is also the
        # request's front frame, from ONE subscription feeding both (a
        # pointer each, nothing decoded in the callback); VLAFront is then
        # not subscribed. The down frame is paired and sent as always.
        detector_front = args.vla_front_source == "detector"
        if not detector_front:
            sim.subscribe(drone.sensors[CAMERAS["front"]]["scene_camera"],
                          lambda _, m: frames.put_front(m))
        sim.subscribe(drone.sensors[CAMERAS["down"]]["scene_camera"],
                      lambda _, m: frames.put_down(m))
        if args.mission in VM.DETECTOR_MISSIONS:
            def _detector_frame(_, m):
                obs.put_front(m)
                if detector_front:
                    frames.put_front(m)
            sim.subscribe(drone.sensors[CAMERAS["detector"]]["scene_camera"], _detector_frame)
            # OWL-ViT loads in its own thread from here, beside the warm-up
            # and the take-off (2026-10-23): a follow's subject, the level's
            # car, starts driving when the scene loads, and loading the
            # detector only after the transfer cost the first ~30 s of it.
            # M3: the earlier city follows' TargetLock and identity check
            # (perception only: which box is the subject; the VLA still
            # steers). Follow #6 (23 Oct) without them: the box matched the
            # car on 157 of 1036 chase ticks. The identity tiers judge each
            # candidate physically from its frame's depth and pose and the
            # street mask (demo/identity.py), as follow_vlm --identity does.
            # M2 keeps each frame's best box. The lock, the identity and the
            # start gate are decided AFTER the identity setup, which may fail
            # (follow_perception; review of d6e7123, P2-5).
            det_lock = id_cfg = None
            if args.mission == "follow":
                id_err = None
                if not args.no_identity:
                    id_cfg, id_err = _identity_setup(args, sim, drone, obs, FV)
                    print(f"[identity] ON - thresholds {id_cfg['hash']}, strict lock"
                          if id_cfg is not None else
                          f"[identity] *** OFF ({id_err}): the DetectorHint's start gate "
                          f"instead")
                follow_plan = follow_perception(args, id_cfg is not None, id_err)
                follow_plan["identity"]["hash"] = None if id_cfg is None else id_cfg["hash"]
                if follow_plan["lock"]:
                    det_lock = FV.TargetLock(gate_frac=args.lock_gate)
                gate = follow_plan["start_gate"]
                if gate is not None and gate["kind"] == "detector_hint":
                    detector_hint.set_start_gate((gate["n"], gate["width_frac"]))
            identity_rec["on"] = id_cfg is not None
            identity_rec["hash"] = None if id_cfg is None else id_cfg["hash"]
            # --gate-truth: the box is the level car's true position, from the
            # level car resolved below (gate_truth_box holds its truth function)
            teacher = None
            if getattr(args, "gate_truth", False):
                teacher = (lambda t_cap: gate_truth_box["f"](t_cap)
                           if "f" in gate_truth_box else None)
            grounder = FV.Grounder(obs, args.object, thresh=args.det_thresh,
                                   log_path=out / "detections.jsonl",
                                   colour_min=args.colour_min, lock=det_lock,
                                   identity=id_cfg, teacher=teacher,
                                   teacher_dims=(FV.subject_width(args.object)[0],
                                                 FV.subject_height_m(args.object)))
            grounder_box["g"] = grounder
            grounder.start()
            if id_cfg is not None:
                # follow_vlm's start gate with identity: the Grounder publishes
                # candidates (acquiring), follow_vlm's Reacquirer judges them
                # (identity tiers, range, motion) and commit() seeds the strict
                # lock; until then there is no box, so no VLA request.
                # resolve_args refuses --start-gate-n 0 with identity.
                acq = make_acquirer(args, FV)
                acq.start(None)
                acq_box = new_acquisition(acq)
                grounder.begin_acquire()
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
            # "Car_00,Car_23,Car_14": every level car the phrase names, scored
            # against the nearest (level_actors.LevelCars; 31 Oct)
            tags = [t.strip() for t in str(args.level_car).split(",") if t.strip()]
            level_car = (level_actors.LevelCars(world, tags, desc=args.object,
                                                min_period_s=LEVEL_TRUTH_PERIOD_S)
                         if len(tags) > 1 else
                         level_actors.LevelCar(world, args.level_car, desc=args.object,
                                               min_period_s=LEVEL_TRUTH_PERIOD_S))
            try:
                level_car.spawn()
            except Exception as exc:                               # noqa: BLE001
                raise PreflightRefused("level_car_missing",
                                       f"the level car {args.level_car} did not resolve "
                                       f"({type(exc).__name__}: {exc})") from None
            print(f"[car] following the level's {args.level_car} ({args.object!r}) at "
                  f"({level_car.pos[0]:.1f}, {level_car.pos[1]:.1f}) - truth for scoring only")
            if getattr(args, "gate_truth", False):
                # the start gate's box on the level car's true position (with
                # several tags, the nearest the scorer last found; the first
                # before the loop)
                gate_truth_box["f"] = FV.teacher_truth_fn(level_car)
                print(f"[gate] --gate-truth: the start gate and re-acquisition judge "
                      f"{args.level_car}'s TRUE position; the VLA flies on its own "
                      f"perception")

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
        acq_box["t0"] = t0                     # its records in mission seconds
        monitor = VM.MissionMonitor(args.monitor, t0)
        loop = VM.MissionLoop(backend, shield, monitor, mission=args.mission,
                              shield_on=shield_on,
                              goal_xy=args.goal if args.mission == "goto" else None,
                              det_source=det_source, hint_source=hint_src,
                              detector_hint=detector_hint, audit=audit,
                              server_alive=lambda: server.alive, clock=time.time,
                              close_hold=close_rule)
        print(f"[vla] mission {args.mission} - AerialVLA flies; hint source "
              f"{hint_src.kind}; object words {args.object!r} (inert)")
        deadline = None
        stuck = (VM.StuckWatch() if args.mission == "follow" and not args.no_stuck_reset
                 else None)
        close_watch = (VM.CloseStall(args.close_stall_s)
                       if args.mission == "follow" and args.close_stall_s > 0 else None)
        while True:
            tk0 = time.time()
            state, yaw, kin = _feed_pose(drone, obs, State, quat_yaw, FV)
            if acq_box["acq"] is not None and grounder is not None:
                try:
                    _acquire_step(grounder, acq_box, args)
                except Exception as exc:                           # noqa: BLE001
                    if len(acq_box["errors"]) < 20:
                        acq_box["errors"].append(f"{type(exc).__name__}: {exc}")
            off_map = shield.off_map(state.x, state.y) if shield.has_obstacle_map else None
            # the attitude, so the motion compares with follow_vlm's rows
            # (tools/compare_motion.py tilt and pitch/roll rates)
            res = loop.tick(state, extra_row={"off_map": off_map, **attitude_fields(kin, FV)})
            e = res.flown
            cmd_times.append(time.time())
            await drone.move_by_velocity_async(e.vx, e.vy, -e.vz_up, duration=0.3,
                                               yaw_is_rate=True, yaw=e.yaw_rate)
            if level_car is not None:
                # Truth for SCORING only: the level car against the aircraft.
                try:
                    level_car.update(time.time() - t0)
                    if hasattr(level_car, "nearest"):
                        # several candidates: the nearest is the subject's truth,
                        # every candidate's distance is kept for the scorer
                        near = level_car.nearest(state.x, state.y)
                        res.row["subject_tag"] = near[0] if near else None
                        res.row["subject_dists"] = {
                            t: round(math.hypot(state.x - fx, state.y - fy), 2)
                            for t, fx, fy in level_car.positions()}
                    cx, cy = level_car.pos
                    if math.isfinite(cx) and math.isfinite(cy):
                        res.row["subject_xy"] = [round(cx, 2), round(cy, 2)]
                        res.row["subject_dist_m"] = round(math.hypot(state.x - cx,
                                                                     state.y - cy), 2)
                except Exception as exc:                       # noqa: BLE001
                    if len(level_truth_errors) < 20:
                        level_truth_errors.append(f"{type(exc).__name__}: {exc}")
            why_reset = (_follow_watch(res, state, stuck, close_watch,
                                       backend.counts.get("requests", 0))
                         if args.mission == "follow" else None)
            if why_reset is not None:
                # Pinned by the Shield (what flew), or a close box and no
                # request for close_stall_s: the box is probably the wrong
                # object. Perception only; the VLA still steers.
                ev = _perception_reset(why_reset, time.time(), res.row["t"], grounder,
                                       detector_hint, acq_box, state)
                resets.append(ev)
                res.row["perception_reset"] = why_reset
                if why_reset == "stuck":
                    res.row["stuck_reset"] = True
                what = ("pinned by the Shield" if why_reset == "stuck" else
                        f"a close box and no request for {args.close_stall_s:g} s")
                print(f"[follow] {what} at ({state.x:.1f}, {state.y:.1f}): "
                      + ("re-acquiring" if ev.get("reacquiring") else "target lock reset")
                      + (f" *** {ev['errors']}" if ev.get("errors") else ""))
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
                # The box this tick read (one read per tick, MissionLoop).
                det = res.det if args.mission in VM.DETECTOR_MISSIONS else None
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
    # M3: the follow's score from the level car's truth rows; its two KPI
    # keys go to the TOP of metrics, where guardrail/kpi.py reads them
    # (review of d6e7123, P1-1). A score that could not be computed is no
    # score: follow_scored False, so kpi says "not measurable", never success.
    fscores = (guard("follow_scores", lambda: VM.follow_scores(
        rows, acquired_at_s=(acq_box["commits"][0] if acq_box.get("commits") else None)), None)
               if args.mission == "follow" else None)
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
            "detector_hint": detector_hint.stats(t0=t0) if detector_hint is not None else None,
            "park_at": list(args.park_at) if args.park_at else None,
            "bearing_gain": args.bearing_gain if args.mission in VM.DETECTOR_MISSIONS else None,
            "detector": FV.DETECTOR_ID if args.mission in VM.DETECTOR_MISSIONS else None,
            "level_car": args.level_car,
            "monitor": monitor.summary() if monitor is not None else None}),
        **scores,
        **(VM.follow_kpi_fields(fscores) if args.mission == "follow" else {}),
        "follow": (guard("follow_record", lambda: {
            **(fscores or {}), "level_car": args.level_car,
            "truth_period_s": LEVEL_TRUTH_PERIOD_S,
            "truth_errors": list(level_truth_errors),
            "level_stats": level_car.stats() if level_car is not None else None,
            # what the perception did, measured (review of d6e7123, P3: these
            # were under params, with epoch times)
            **follow_measured(acq_box, resets, loop)})
                   if args.mission == "follow" else None),
        # The scene and robot config the simulator was given, and whether
        # this run read the detector's depth stream (review of d6e7123, P3).
        **({"sim_config": guard("sim_config", lambda: VM.sim_config_record(
            SIM_CONFIG_DIR / SCENES[args.mission], depth_read=bool(identity_rec["on"])))}
           if args.mission in VM.DETECTOR_MISSIONS else {}),
        "land": guard("land", lambda: VM.land_record(rows, records, goal)),
        # Smoothness on the hand-written follow's definitions (tools/
        # compare_motion.py), so the two controllers compare number for number.
        "motion": guard("motion", lambda: motion_record(rows)),
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
                   # the follow's configuration as APPLIED (follow_perception,
                   # decided after the identity setup; review of d6e7123)
                   "follow": (follow_params(args, follow_plan)
                              if args.mission == "follow" else None),
                   "tracker": {k: getattr(tcfg, k) for k in tcfg.__dataclass_fields__},
                   "smooth": smooth_record(args),
                   "step_as_rate_s": args.step_as_rate,
                   # the lower tile flown and the adapter's, as checked after hello
                   "bottom_tile": tile_check,
                   # the speed words sent and the adapter's, as checked after hello
                   "speed_words": sw_check,
                   # the front camera sent and the server's, as checked after hello
                   "front_source": front_check,
                   "vla_adapter": args.vla_adapter,
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
        weight_paths, sim_address=args.sim_address))
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
              f"{s.get('link_failure_by_kind')}, LAND {s.get('land')}"
              + (f" (held as arrival {s['land_as_arrival']}: no landing)"
                 if s.get("land_as_arrival") else ""))
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
