"""One process: Project AirSim camera -> detector -> pilot -> Shield -> ArduPilot.

The perception-rail integration the grant's final delivery names: the camera
image, the language-grounded detector, the 4-D action, the Safety Shield and
the autopilot in ONE loop, with ArduPilot - not Project AirSim's simple_flight -
flying the aircraft, so Mission Planner sees the flight through mavlink-router.

PER 10 Hz TICK

  1. state     ArduPilot's EKF (GLOBAL_POSITION_INT, or MAVROS's global
               position), projected into the scene frame (autopilot.FrameAnchor).
  2. frame     FrontCamera from Project AirSim, or a ROS 2 image topic on the
               real drone, buffered by demo/semantic_demo.SemanticObs.
  3. detector  demo/follow_vlm.Grounder - OWL-ViT in its own thread, with the
               colour prior and the continuity gate - unchanged.
  4. pilot     demo/follow_vlm.servo(): box -> yaw rate and forward speed, with
               the same altitude hold; follow_vlm.search_sweep_rate() while the
               box is stale, its first swing toward the side the box was last
               seen on. The 4-D action leaves the pilot in the BODY frame.
  5. Shield    body -> scene frame once (autopilot.body_to_world, which is
               guardrail.frames.from_body), then Shield.filter() with the
               grant's 5 s / 10 Hz lookahead and no FSM of its own
               (guardrail.replay.rail_shield). Stand-off rules get the
               subject's position from depth range (or apparent width) and
               bearing - never ground truth.
  6. FSM       with the Shield on, guardrail.fsm.EscalationFSM (the grant's
               Brake -> Loiter -> RTL -> Land) steps on the Shield's decision,
               as sitl/run_sitl_demo.py and sitl/ros2_shield_node.py do. Its
               mode requests go to ArduPilot without blocking the loop
               (link.request_mode), and its setpoint choice (pass / brake /
               none) decides what is sent.
  7. command   ONLY the Shield's emitted action (or the FSM's brake) goes to
               ArduPilot: SET_POSITION_TARGET_LOCAL_NED (pymavlink) or
               PositionTarget on /mavros/setpoint_raw/local (MAVROS 2).

WHAT THE AUTOPILOT DOES ON ITS OWN IS RECORDED. Every row carries the
autopilot's mode and arm state; metrics.json carries its mode changes, arm
changes, STATUSTEXT, FENCE_STATUS breaches and the FSM's mode requests. If
ArduPilot leaves GUIDED without this node asking (its fence fired RTL, a
failsafe, a pilot in Mission Planner), the node stops commanding at once,
records the takeover, and ends the episode once the aircraft has landed or
after TAKEOVER_GRACE_S - it never keeps streaming setpoints the autopilot is
ignoring.

ROWS have the fields the KPIs need (guardrail.replay.episode_row_fields:
unsafe, unsafe_rules, policy_hash, generation, shield_ms, episode_id), what
was flown and the Shield's check on it (guardrail.replay.flown_fields: a tick
nothing was sent on cannot escape), and the episode directory is written
through guardrail.replay.EpisodeRecord, so time to safe is measurable, the
replay bundle carries every policy generation and binds its files by the
episode id. shield_audit.jsonl (audit.jsonl until 2026-11-02) carries this
node's FSM verdict (audit_tick), the
same one the flight log does. These are the contracts both SITL rails follow.

RULE CLOCK, MISSION, POLICY, BACKSTOP, TRUTH (2026-10-12, as on the SITL rails)

  * One rule clock (guardrail.replay.rule_clock) for the Shield and every
    generation's CSP: `--clock-start` scripts it with the mission time, else
    the wall clock in the mission's time zone (`--clock-tz`, default +08:00).
    Each trajectory point carries its reading, so zone dwell applies the
    windows the Shield flew under. Before this the Shield had no clock: every
    scheduled rule was in force and no time_window_switch did anything.
  * A follow Mission (`follow_mission`): the task text, the start (the
    scene's spawn point in the Shield's frame) and, as the subject-follow
    target proxy, the start again - the subject's position is not known
    before the flight, and no ground truth may stand in for it. With it the
    episode record compiles csp_g<N>.json for each generation; without it
    every run recorded csp_failed "no mission given".
  * A YAML policy flies only with `--dev-yaml`, recorded and never KPI-grade;
    with no policy option the signed bundle of follow_car_citylife flies when
    tools/build_flight_bundles.py has put it in bundles/, and nothing flies
    otherwise (guardrail.manifest.flight_policy_choice).
  * The autopilot's GeoFence gets the policy's clock-proof keep-out zones
    (`FenceBackstop`: guardrail.fence_export's selection, a zone the aircraft
    is in or near held back until it is clear), uploaded over the node's own
    link before arming. Until then the rail only read the fence parameters
    back, so the backstop held a 400 m circle and an altitude, never a zone.
    A zone already on the fence is never held back again, the list held
    before is recorded, and with nothing to upload the list is cleared.
  * The followed subject's simulator pose (`SubjectTruth`, World.
    get_object_poses on the level actor's tag - Car_10, the CityLife red
    car) is logged per tick for EVALUATION only (tgt_x / tgt_y, read after
    the setpoint is sent), with the true range, the stand-off dwell and the
    estimate's error in metrics.json. The Shield is never given it: it
    keeps the depth / width estimate below. An unread pose is None with the
    reason, never 0; failed reads back off and retry; the follow is scored
    only when TRUTH_MIN_COVERAGE of the ticks were read.
  * The truth checks its own frame (since 2026-10-14): every 0.5 s the
    AIRCRAFT is read through the subject's own RPC and projection
    (World.get_object_poses on AIRCRAFT_ACTOR, scene_to_anchor) and set
    against the EKF position the Shield flies on, and every 0.5 s between
    those the EKF against the simulator's kinematics (ekf_vs_sim)
    (`subject_truth.frame_check`). Over TRUTH_FRAME_MAX_M the truth scores
    nothing and leaves the rows' scoring fields; the cause says whether the
    truth's frame or the EKF was off. Without the first check (the aircraft
    not found) the frame is only PARTLY checked and never "agrees".
    `subject_truth.view` says, on the ticks with an estimate, whether the
    subject was even in the camera's field of view. pas_ap_close_on/off
    (2026-10-13) needed both: the subject was behind the aircraft on most
    estimate ticks, the detector on something else (truth_view).
  * `--hold-mode LOITER` on SITL centres the throttle (RC3 override 1500)
    every tick the FSM holds and the autopilot is in LOITER, so ArduCopter's
    LOITER holds its altitude with no pilot - the grant's literal mode. A
    mode the node did not ask for, from GUIDED or from its hold, is a
    takeover and hands channel 3 back. BRAKE stays the default.

THE START GATE, THE DETECTOR, THE LOOP'S TIMING (2026-10-21)

  * follow_vlm's start gate (StartGate, START_WHEN_SEEN): after the take-off
    the aircraft hovers - nothing commanded, ArduCopter's own GUIDED hold -
    until the detector has produced 5 fresh inferences in a row each holding
    a box follow_vlm's presence verdict does not call ABSENT, within 30 m,
    continuous in the frame; only then does the mission clock start. The
    Grounder publishes every candidate while it acquires and is committed to
    the pick, which seeds its TargetLock. A gate that times out (330 s) flies
    no mission by default (--start-timeout-action land; "fly" flies anyway),
    and metrics.json `start_gate` records what was judged and refused, the
    pick, and - read after the decision, evaluation only - whether it was the
    subject. Replayed on the 21 Oct flight's boxes it never opens.
  * --identity (follow_vlm's identity tiers: only an OK box may start or
    steer; the start decided in the world, seen moving: follow_vlm.Reacquirer
    asks its moving test of 2 x --start-when-seen OK boxes in a row, 10 at
    the default, and the record, the [start] lines and a timeout's reason
    say that rule - StartGate.rule - with the Reacquirer's own streak,
    refusals and what each of its steps decided (the moving test's refusals
    among them), metrics.json start_gate.identity_gate); --negative-query
    (a box that looks more like "a traffic light" is no candidate).
  * The detector starts as soon as the camera is up, so OWL-ViT loads while
    SITL boots and the aircraft takes off, and the mission waits for its
    first inference: on 21 Oct the load ran beside the first 26 s of ticks.
  * The simulator's evaluation RPCs (truth, frame check, EKF check) run on
    one worker thread (OffLoopRpc) and the loop waits for them at most
    50 ms per tick, all of that tick's calls together: one 1024.9 ms truth
    read was that flight's 1028 ms tick (measured). The 97.9 ms Shield step
    fell inside the detector's load; that the load caused it is inferred
    from the timing, not profiled. Neither change has flown yet.

THE VLA PILOT (--pilot vla; wave V2b, 2026-10-23, to-do VLA-07)

  `--pilot servo` (the default) is the detector pilot above. `--pilot vla`
  puts AerialVLA in the pilot's slot as demo/vla_fly.py does on the CityLife
  simple_flight rail (docs/DESIGN-vla-pilot.md section 13), from the same
  parts: guardrail/vla_link (RemoteVLA, StepTracker), guardrail/
  vla_backends.RemoteVLABackend and demo/vla_mission (the hints, the
  mission's stop rules, the warm-up gate, the records, the HUD panel).
  * Who flies: each tick the step tracker's world-frame action IS the raw
    action (VLAPilot.step); from there it takes the detector pilot's path,
    unchanged: Shield.filter, the escalation FSM, link.send of the Shield's
    output (or the FSM's brake). A tick on which the FSM, a fault or the
    autopilot holds the aircraft, and a stale-link span, suspend the tracker
    (backend.suspend) and stop the mission's timers (MissionMonitor.hold);
    every answer to an image from before is thrown away when the pilot gets
    the aircraft back (resume), and no VLA end names a held mission.
  * The missions: --mission goto (the default; --goal X,Y in the Shield's
    frame, no default: the spawn has two street trees in the flight band
    within about a metre of every line east, vla_mission's CityLife notes),
    approach (the OWL-ViT box's bearing through demo/vla_bridge, as the
    city rail's M2) and follow (THE VLA FOLLOW below, the city rail's M3).
    LAND is the model's flag, landed by this node's own
    mission-end LAND (link.land), never a mode request from the pilot; the
    city rail's other stop rules apply (vla_mission.MissionMonitor), beside
    this rail's (max_s, a lost link, a takeover, the FSM).
  * The server: --vla-server HOST:PORT attaches; otherwise the node starts
    demo/vla_server_v2.py naming --vla-precision, --vla-attn and
    --vla-merge / --no-vla-merge (all three, no default) or the scripted
    stand-in (--vla-fake-output) before the scene loads, and waits for its
    hello before ArduPilot starts. A server it started is stopped on every
    way out; an attached one never is.
  * The warm-up gate (vla_mission.run_warmup: three requests on live frames,
    each with its own answer timeout - the first, the model's first
    inference, first_decision_timeout_s - and a ready client before it, the
    last judged) runs in the pre-arm step, disarmed: a refusal arms nothing,
    exits 4 and is recorded (metrics.json, manifest.json; PreArmRefused). The
    VLA pilot therefore needs a link with a pre-arm step; both real links
    have one.
  * Stop-and-go only (--allow-unstamped-pipelined, recorded, for the
    pipelined arm): this rail's pose ring has no sim stamps, so a capture is
    paired with the EKF pose at its arrival.
  * AerialVLA's VLAFront / VLADown render only in a VLA run's rendered robot
    config (or with --vla-cameras, for a scene step an attaching VLA node
    uses): rail.select_vla_cameras, sim_config.vla_cameras.
  * The start gate is a detector-subject gate: skipped for goto, recorded
    (start_gate.enabled false and why), and no detector runs at all; kept
    for approach, which needs the subject in view before the VLA is asked
    to approach it.
  * Records as on the city rail: vla_steps.jsonl, vla_frames/ and
    vla_captures.jsonl, rows naming each tick's author and the decision
    behind it, metrics.pilot (vla_mission.pilot_entry: kind vla only with
    the server's real identity), metrics.vla (with nvidia-smi's free VRAM at
    the hello and after the warm-up), the manifest's vla_model_hash, the HUD
    panel (vla_mission.annotate_vla, the rail's own label kept). A GOTO is
    scored on reached_goal (no follow score owed), an approach on the
    subject's true range. Exit 3 for a VLA failure end (the server dead,
    slow or erroring, vla_mission.VLA_FAILURE_ENDS).
  * NOT FLOWN: none of this has run against Unreal, ArduPilot or the model;
    tests/test_pas_vla.py drives it with the stand-in link and a scripted
    protocol-v2 server.

THE VLA FOLLOW (--pilot vla --mission follow; 2026-11-06)

  The red-car follow flown by a VLA on this rail - first of all the lab's
  own AIOT-VLA (demo/vla_server_foreign.py --model aiotvla,
  docs/DESIGN-aiotvla.md) - from demo/vla_fly.py's M3 parts, imported, not
  copied:
  * The hint (vla_mission.DetectorHint): the OWL-ViT box's phrase, the last
    sighting's bearing kept --follow-memory-s (6 s) past the box, then a
    quarter turn toward it --follow-search-s (20 s); a close box
    (vla_mission.BoxClose: bottom edge at --close-bottom-frac 0.92 of the
    image, or --close-width-frac 0.25 wide) sends nothing and holds the
    horizontal motion (VLAPilot.step, as vla_mission.MissionLoop: inside
    the tracker when it smooths). Each request records its kind (fresh /
    memory / search). No hint, no request: the backend's request count goes
    to the stop rules, so a follow that never asked ends
    subject_not_acquired, not as a VLA failure.
  * The step tracker is vla_fly's follow tracker (vla_fly.tracker_config on
    vla_fly's own parse of this flight: turn while moving, the altitude held
    at --cruise-alt, steps anchored at the answer, LAND held as arrival,
    the smoothing), with --vla-step-as-rate H (each step a velocity over
    H: AIOT-VLA's 0.5 s label horizon) and --vla-rate-yaw rate (dpsi/H as
    the yaw rate itself, the meaning of AIOT-VLA's label).
  * --vla-front-source detector sends the detector's FrontCamera frame
    (768x432, pitched -20 deg) as the request's front image, from the ONE
    FrontCamera subscription that feeds the detector; VLAFront is then not
    subscribed and only VLADown is required. The server's hello must declare
    that camera (vla_mission.front_source_check), or nothing is armed.
  * The server's own host (its hello's `host`) is the run's `vla_host`
    evidence: the VLA is a separate process, so hil holds only when that
    host is an Orin too (guardrail.manifest._vla_host_missing).
  * The object words are recorded as what they are to the model that
    answered (vla_mission.object_words_record): AIOT-VLA's grounding phrase,
    not AerialVLA's inert slot.
  * THE OBSERVATION RATE (metrics.json `vla_observation`; the grant's
    "Camera / IMU / GPS - 10 Hz observation"): the DISTINCT front-frame sim
    stamps among the requests actually sent, per wall second of the mission,
    per sim second, and per second the pilot was being asked (not held by
    the rail, not gated by the hint), with the duplicates, the gaps between
    frames and the hint-gated spans apart. Never the decision rate or the
    request rate. --front-capture-interval-s renders FrontCamera's capture
    interval (0.1 s in the template) at scene render time.
  * CAMERA TO SETPOINT (2026-11-11; grant WP3-13, and the meeting's "did
    you measure the latency from camera input to action output?"). Every
    row carries `t_pub`, the mission time its setpoint went out (read right
    after link.send returned; None when nothing was sent), and `tick_ms`,
    the tick's start to it (the hil Shield node's field of that name). A
    VLA row also carries `cam_age_ms`, the age at that publication of the
    camera frame behind the flown action (t_pub - the t_capture of the
    decision being flown, vla_steps.jsonl), on every tick that decision's
    action flew, and `cam_to_setpoint_ms`, the same on the FIRST tick that
    flies it: camera input to action output, through the request, the
    model, the step tracker, the Shield and the FSM. metrics.json
    `camera_to_setpoint` gives p50 / p95 / p99 / max and the ticks over
    the grant's 100 ms (tools/kpi_report.py pools the rows). CLOCK DOMAINS:
    all of it is ONE clock, the loop's (`clock`, time.time unless
    injected) - the frame's arrival (VLAFrames.put_front), the request's
    t_capture / t_sent / t_received (RemoteVLA, the backend), the tick and
    the publication. On this rail a capture's time is the frame's ARRIVAL
    at the node, not its render: the pose ring is fed without simulator
    stamps (put_pose_full here passes none), so VLAFrames times every
    capture by arrival (`t_capture_basis` "arrival") and the render and
    transport before it are NOT in cam_*: a lower bound of camera to
    action. The frame's simulator stamp (vla_captures.jsonl `stamp_ns`) is
    the simulator's clock and is never mixed in, and nothing here sees the
    autopilot's receipt of the setpoint.
  * Run on the AGX Orin by deploy/orin/run_pas_perception.sh (the hil
    topology; docs/DESIGN-projectairsim-ardupilot.md has the order).
  * NOT FLOWN: tests/test_pas_vla.py drives it with the stand-in link and a
    scripted protocol-v2 server announcing an aiotvla identity.

THE TARGET ESTIMATOR (--target-estimator, with --identity, for --pilot vla
--mission follow; 2026-11-15)

  follow_vlm.fly()'s subject handling, which this rail lacked (pas_hil_aiot_
  kpi_01: the detector's box fresh on 942 of 4,192 ticks, on those ticks the
  car a median 97 m away; kpi_02 with --identity held no lock at all):
  demo/subject_track.SubjectTrack, stepped by VLAPilot.step once per tick on
  the Grounder's answer before the backend acts, built from follow_vlm's own
  functions (the estimator, its seed, the SOFT rule, the lapse anchor, the
  strict lock's prior, the Reacquirer, the trail), not a copy of them.
  * The Grounder holds follow_vlm's STRICT lock (build_grounder): seeded by
    the start gate's pick, leaning on the estimate between boxes, and never
    adopting a box that is not where the subject should be.
  * Between boxes the estimate (demo/target_state.TargetState, a constant-
    velocity model on identity's range and bearing at each box's own
    capture) predicts where the subject is; after lapse_s with no accepted
    measurement it is dropped and follow_vlm.Reacquirer takes the subject
    back, anchored where it was predicted, only on identity-OK evidence.
  * The VLA's hint with no fresh box (vla_mission.DetectorHint's
    `estimate`): "memory" toward the PREDICTED subject while the estimate is
    served, then toward fly()'s coast look along the trail while the subject
    is still held in memory, then the sighting's own memory and search, as
    before. A model that reads the compass phrase is turned toward where the
    subject went; one that ignores it (AIOT-VLA, demo/vla_server_foreign.py:
    "the compass phrase ... ignored") gets only the requests it gates.
  * Recorded: the rows' est_state, est_xy, est_age_s, est_speed_mps,
    est_memory, lock_why, reacq_streak and hint_memory_src, est_gap_m
    against the truth (evaluation only); metrics.json `subject_track` (the
    estimator, the commits, lapses and re-acquisitions, the ticks per state,
    the estimate's age, the memory's sources, est_lock_frac against the
    truth). The Shield's subject is unchanged: the fresh box placed from its
    own capture, as before.

THE ESTIMATOR FOLLOW (--pilot servo --mission follow --identity
--target-estimator; 2026-11-17; EstimatorFollowPilot)

  The lab's hand-written follow WITH its target estimator, as the backend
  table's second C2 null (FAMILY=estimator in deploy/orin/
  run_pas_perception.sh): the same SubjectTrack as the VLA follow's, and
  follow_vlm.fly()'s two track branches on it - the servo on the estimate
  (yaw_command with the line-of-sight feed-forward, standoff_speed with the
  range feed-forward and the catch-up, the stopped-subject floor) and the
  box servo at servo_want_width - then a coast toward the subject's memory
  and the servo pilot's sweep. Without --target-estimator the servo pilot is
  the box servo above, unchanged. NOT FLOWN: tests/test_pas_estimator_follow.py
  drives it on a scripted subject and Grounder.

WHAT THIS IS NOT. The rest of follow_vlm.fly()'s control law - trail
FOLLOWING and the trail approach, the search creep and planner, the fence
guard, the slew limiter, the landing site - lives inside that function and
is not reused here. Its numbers are not comparable with the simple_flight
CityLife flights. Its tracking score comes from the subject truth above when
the scene provides it (None otherwise).

TOPOLOGY. topology.classify() decides the label the evidence supports. The
grant label is OFFERED to build_manifest only when the rail is qualified: gate
G1 passed (--g1-verdict) and ArduPilot ran at real time (its own clock against
the wall clock, within RT_FACTOR_BAND). Otherwise the rail's own label is
stamped and the reason recorded. With --link pymavlink (what runs on Windows
today) the run is "projectairsim-ardupilot-pymavlink" whatever else holds.

SCENE MODES. --scene-mode load (the default, on the desktop) loads the scene
and then, with --start-sitl, starts ArduPilot - in that order, because a
reload resets the simulator clock under a running ArduPilot. --scene-mode
attach (the hil node on the Jetson Orin) never loads: the desktop has already
run `-Step scene` and `-Step sitl`, and the node only subscribes. --scene-only
loads the scene and exits, which is what `-Step scene` runs.

RECORDING (--save-view, 2026-10-14). The usual demo video, as follow_vlm
records it: demo/recorder.FrameRecorder on its own thread writes the
first-person FrontCamera frame with the policy HUD (follow_vlm.annotate ->
demo/policy_hud.py: rule panel, banner, minimap) to <out>/view/fpv and the
robot's Chase camera to view/tps at --record-hz, and view/recorder.json with
the rate it measured, which tools/make_demo_video.py --tag <tag> takes as the
video's fps. The banner adds what only this rail has (rail_banner): the
escalation FSM's state (Brake, Loiter(<the ArduCopter hold mode>), RTL,
Land), the autopilot taking the aircraft, and SHIELD OFF on the control arm,
whose panel shows the rules broken and flown (hud_decision), never a Shield
correction. The loop pays for the indicator's rule_status and one handover
per tick, after the setpoint is sent; metrics.json `recording.hud_ms` and
`tick_work_ms` measure it. tools/make_compare_video.py puts two recorded runs
(Shield on / off) in one video.

RUN
    # check everything that can be checked without a simulator
    python demo/pas_ardupilot/perception_node.py --dry-run --network nat \
        --windows-ip 172.x.x.1 --wsl-ip 172.x.x.y
    # fly (the launcher fills in the addresses): scripts/run_pas_ardupilot.ps1 -Step node
    # a YAML policy, a scripted rule clock, the grant's literal LOITER:
    #   ... --policy policies/follow_car_citylife_nfz.yaml --dev-yaml \
    #       --clock-start 2026-10-12T17:29:50 --hold-mode LOITER
    # the demo video: ... --save-view   (scripts/run_pas_ardupilot.ps1 -SaveView)
    # a perception-stress variant, never KPI-grade (perception_stress.py):
    #   ... --stress night   (scripts/run_pas_ardupilot.ps1 -Step scene -Stress night,
    #                         then STRESS=night deploy/orin/run_pas_perception.sh)
"""
from __future__ import annotations

import argparse
import collections
import importlib
import importlib.util
import inspect
import json
import math
import random
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path
from types import SimpleNamespace

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
for _p in (ROOT, ROOT / "demo", HERE):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
# sitl/ LAST, so nothing there shadows a module above: only for the GeoFence
# rule both SITL rails share (mavlink_adapter_node.fence_alt_plan).
if str(ROOT / "sitl") not in sys.path:
    sys.path.append(str(ROOT / "sitl"))


class _NoBeat:
    def beat(self, *a, **k):
        return None

    def stop(self, *a, **k):
        return None


def _live_heartbeat(name: str, **kw):
    """A heartbeat for the tracker's Dashboard "running now" panel
    (tools/live_status.py), or a no-op when anything goes wrong. beat() only
    stores numbers in memory; a daemon thread writes at most once a second to
    %LOCALAPPDATA%, outside OneDrive, and swallows its own errors, so the 10 Hz
    loop never waits on it and cannot be stopped by it. Loaded by path:
    tools/ is not put on sys.path, where its modules could shadow these."""
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "vlaguard_live_status", ROOT / "tools" / "live_status.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.heartbeat("flight", name, **kw)
    except Exception:                                   # noqa: BLE001
        return _NoBeat()

from guardrail import AuditLogger, State                             # noqa: E402
from guardrail.audit import AUDIT_FILE                               # noqa: E402
from guardrail.bundle import load_for_flight                          # noqa: E402
from guardrail.compiler import ConstraintCompiler, Mission            # noqa: E402
from guardrail.fence_export import (defer_near, is_enforced,       # noqa: E402
                                    policy_zones, zone_clearance_m, zones_near)
from guardrail.gc_pacing import GcPacer                               # noqa: E402
from guardrail.fsm import (DEFAULT_THETA_HORIZON_S, FAILSAFE_STATES,  # noqa: E402
                           THETA_HORIZON_S, EscalationFSM, FSMConfig,
                           FSMState, tick_input_from_decision)
from guardrail.kpi import compute_from_dir, null_hover_mission        # noqa: E402
from guardrail.manifest import (DEV_YAML_FLAG, MAX_HEADING_ERR_DEG,   # noqa: E402
                                NO_VLA, TOPOLOGY_DEV, TOPOLOGY_HIL, build_manifest,
                                flight_bundle_path, flight_policy_choice,
                                is_kpi_grade, projectairsim_simulator,
                                write_stack_revisions)
from guardrail.models import (Action4D, AltitudeEnvelope,             # noqa: E402
                              ObstacleClearance, SubjectStandoff)
from guardrail.projection import LocalProjection                      # noqa: E402
from guardrail.replay import (EpisodeRecord, audit_tick,               # noqa: E402
                              episode_row_fields, flown_fields,
                              kpi_evidence_grade, rail_shield, rule_clock,
                              verify_replay, write_replay, zone_dwell)
from guardrail import response as vresp                               # noqa: E402
from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S               # noqa: E402
from mavlink_adapter_node import (DEFAULT_HOLD_MODE, FENCE_MARGIN_M,   # noqa: E402
                                  HOLD_MODES, ModeRequests,
                                  autopilot_mode, fence_alt_plan,
                                  fence_items, policy_ceiling_m)

import perception_stress as pstress                                   # noqa: E402
import rail                                                           # noqa: E402
import rail_hotapply                                                  # noqa: E402
import topology as topo                                               # noqa: E402
from autopilot import FrameAnchor, PymavlinkLink, body_to_world       # noqa: E402

TICK = 0.1
# The loop-rate gate the project's evaluation holds every camera flight to
# (tools/build_eval_data.py, gate loop_hz_min). Every pas_ap run up to
# 2026-10-14 flew at 8.97-9.16 Hz under it: the loop slept TICK minus the
# tick's work, relative to the tick's own start, so each sleep's round-up to
# the OS timer was lost for good (~11 ms a tick on the vla-real Python 3.10).
# The loop now paces to an absolute deadline, as demo/follow_vlm.py does since
# 2026-09; a run below the gate says so in metrics.json (loop_hz_below_gate).
LOOP_HZ_GATE = 9.5
# Architecture constraints: "Camera / IMU / GPS - 10 Hz observation". Every
# run reports its detector rate against this, beside the zero-skill null.
GRANT_OBS_HZ = 10.0
DEFAULT_POLICY = ROOT / "policies" / "follow_car_citylife.yaml"
DEFAULT_CITYMAP = ROOT / "demo" / "out" / "citymap_citylife" / "occ_day.npz"
SUBJECT_UP_M = 0.75          # centre height of a car or a person, as follow_vlm
# A depth reading is the subject's range only if the subject can be where it
# puts it. Four ways it cannot (2026-10-08, five P0 escape ticks of the
# red-car follow came from a subject placed on the aircraft itself):
#   * nearer than the camera's height above the tallest subject - the return
#     is from something above any car or person, whatever the ray;
#   * nearer than the camera's height above the subject's centre (the slant
#     range Pythagoras would turn into a horizontal range of 0) while the ray
#     through the box's centre puts the return above any subject - FrontCamera
#     looks 20 deg down with a half vertical field of about 29 deg, so it
#     cannot see what is under the aircraft;
#   * much nearer than where the ray through the box's bottom edge meets the
#     road (follow_vlm.ground_range_m) - the return is from something in front
#     of the subject. The same 1.5 ratio follow_vlm's ground-contact check
#     uses the other way round;
#   * (2026-10-09, review) a range that would put the subject's centre BELOW
#     the box: Pythagoras places the centre asin(dh / r) below the horizon,
#     and no part of the subject can be lower in the image than the box's
#     bottom edge. The three rules above let a box low in the frame through
#     at 7.5-8.5 m from 7.96 m up: placed 2.1-4.5 m away, 58-74 deg down,
#     outside FrontCamera's view and inside the 5 m P0 ring. Allowed slack:
#     DEPRESSION_SLACK_DEG, for pitch error and the box's own jitter.
# Each way the depth fix is dropped and the box width gives the range.
SUBJECT_TOP_MAX_M = 2.5
DEPTH_GROUND_RATIO_MAX = 1.5
DEPRESSION_SLACK_DEG = 3.0
# After the FSM entered RTL / Land, how long to keep recording.
FAILSAFE_GRACE_S = 40.0
# After the autopilot took the aircraft (fence RTL, failsafe, a pilot in
# Mission Planner), how long to keep recording if it has not landed.
TAKEOVER_GRACE_S = 25.0
# home_reached for the FSM's RTL edge: within this of the arming point.
REACH_M = 2.0
LANDED_STATE_ON_GROUND = 1
# ArduPilot's clock against the wall clock. The grant makes sim_speedup 1.0
# mandatory for a KPI-bearing run; the scene DECLARES 1.0 (step-ns equals
# real-time-update-rate), and this is what the run MEASURED. Outside the band
# the run is not KPI-grade and the grant label is not offered.
RT_FACTOR_BAND = (0.95, 1.05)
# The literal LOITER hold (--hold-mode LOITER on SITL): ArduCopter's LOITER
# climbs at the throttle stick's rate, so the node holds channel 3 at the
# centre of SITL's 1100-1900 range, inside the throttle dead zone.
LOITER_THROTTLE_PWM = 1500
# The CityLife red car: the one red car in the level
# (tools/citylife_mcp/one_red_car.py), tagged Car_10 there so the simulator's
# FindActor resolves it in a -game build (demo/level_actors.py).
RED_CAR_ACTOR = "Car_10"
# The follow rails' proximity radius (guardrail.kpi FOLLOW_RANGE_M, compute()'s
# frac_within_30m).
FOLLOW_RANGE_M = 30.0
# Consecutive failed pose reads (an RPC error, or a NaN answer: no actor with
# that tag) after which the truth reader backs off - it retries after
# TRUTH_RETRY_TICKS, doubling to TRUTH_RETRY_MAX_TICKS while it keeps failing -
# rather than paying a failed read every tick. Until 2026-10-13 it stopped for
# good, so a 0.3 s hiccup left the rest of the flight unread and the read
# ticks were scored as the flight's.
TRUTH_MAX_ERRORS = 3
TRUTH_RETRY_TICKS = 10
TRUTH_RETRY_MAX_TICKS = 100
# The share of ticks whose pose must be read for the truth to SCORE the follow
# (frac_within_30m, follow_scored, a zero stand-off dwell). Below it a dwell
# that was seen is reported as the lower bound it is, and nothing else.
TRUTH_MIN_COVERAGE = 0.95
# The truth's FRAME CHECK (SubjectTruth.check_frame): every TRUTH_FRAME_PERIOD_S
# the AIRCRAFT is read through the subject's own RPC (World.get_object_poses,
# the actor AIRCRAFT_ACTOR), taken down the subject pose's projection
# (scene_to_anchor) and set against the EKF position the Shield flies on. A
# frame that agrees leaves the EKF's own error, which gate G1 holds to a p95 of
# 1.0 m (g1_check.EKF_TOL_M); the limit is twice that, for the read's time skew
# against the EKF fix. Over it (median of the checks), the truth is in another
# frame and scores nothing. Added 2026-10-14 after pas_ap_close_on/off left
# "the truth's frame or the detector's target" open for a day; on review the
# same day it moved from the vehicle's kinematics RPC to get_object_poses, so
# an offset in the level-actor poses alone - the hypothesis it is for - shows.
TRUTH_FRAME_MAX_M = 2.0
TRUTH_FRAME_PERIOD_S = 0.5
# The heading in the same answer (the actor's rotation) against the EKF's: a
# heading off by more than guardrail.manifest's MAX_HEADING_ERR_DEG makes
# "was the subject in the camera's view" luck (its words for the start
# heading), so truth_view's verdict is then withheld. Positions still score.
TRUTH_FRAME_MAX_YAW_DEG = MAX_HEADING_ERR_DEG
# The aircraft as World.get_object_poses finds it. The plugin spawns the robot
# as an AUnrealRobot with no name of its own (UnrealScene.cpp,
# SpawnActorDeferred<AUnrealRobot>), the robot's root link is the actor's root
# component (UnrealRobot.cpp CreateLink), and FindActor matches an actor whose
# NAME CONTAINS the query or that carries it as a tag (UnrealHelpers.h): the
# robot's engine name "UnrealRobot_<n>" answers to "UnrealRobot", the scene's
# robot id ("Drone1") names no actor. Both poses leave the engine through the
# same conversion (UnrealTransform::ToGlobalNed / NeuToNedLinear), so in the
# plugin source the aircraft's object pose IS its kinematic position. Read
# from the Project AirSim plugin source on this desktop
# (D:/ProjectAirSim/Blocks/ProjectAirsim_plugin-Linux-UE5.7-PAS_v0.2.0); not
# yet answered by the CityLife build. One robot per rail scene. A NaN answer
# leaves only the partial check (ekf_vs_sim), said so.
AIRCRAFT_ACTOR = "UnrealRobot"
# An estimate within this of the truth is "on the subject": track_truth.
# score_estimate's near_m, the figure follow_vlm reports as
# metrics.estimate_on_subject.
ESTIMATE_NEAR_M = 6.0
# How old a found box may be and still steer (FollowPilot's "fresh"), and so
# how far back the frame behind an estimate can lie (capture_window_start).
DETECTION_STALE_S = 1.0
# THE START GATE (StartGate; 2026-10-21, open item C1). The mission clock
# starts when the detector has produced START_WHEN_SEEN fresh inferences in a
# row each holding a box follow_vlm's presence verdict does not call ABSENT
# (colour, wall-sized, a width no car has at its depth range, not standing on
# the road) and nearer than START_MAX_RANGE_M, each within the lock's gate of
# the last - follow_vlm's Acquirer, the gate the city flights fly
# (scripts/run_citylife_follow.ps1: 5 boxes, 30 m, 330 s). Until then the
# aircraft hovers where it took off (ArduCopter's own GUIDED hold after the
# take-off; nothing is commanded). Without it every pas_ap follow started at
# take-off whatever the camera saw: on 21 Oct the car was 150-230 m away and
# never in view, the detector boxed a red traffic signal, and 0 of 186
# estimates were within 6 m of the car (CHANGELOG 2026-10-21).
START_WHEN_SEEN = 5
START_MAX_RANGE_M = 30.0
START_TIMEOUT_S = 330.0
# What a gate that timed out does: "land" (the default) flies no mission - the
# episode ends before t0, landed, with the gate's record - and "fly" starts
# the mission anyway, as follow_vlm does, recorded.
START_TIMEOUT_ACTIONS = ("land", "fly")
# THE HOVER BEFORE THE MISSION IS CAPPED (review 2026-10-22). --start-timeout-s
# bounds the airborne wait whichever wait it is spent in: the start gate's,
# and the detector's first inference on a link with no pre-arm step, which
# comes out of the same budget (run(); with no gate that wait has
# DETECTOR_READY_TIMEOUT_S alone, and the links that have a pre-arm step wait
# for the detector disarmed, on the ground). A timeout above this, or not finite, is
# refused before anything starts. 330 s is what scripts/run_citylife_follow.ps1
# flies ("red lights lengthen the car's lap", 2026-09-29); this leaves room for
# a second lap and no more: the hover is watched (HoverWatch), but no KPI
# covers it.
START_TIMEOUT_MAX_S = 600.0
# How long the gate's candidate stays the Shield's subject during the hover
# (HoverWatch), and how recent an inference must be to be judged at all: the
# gate's freshness, follow_vlm's (`time.time() - gw["t"] < 1.0`).
GATE_FRESH_S = 1.0
# While the gate waits, the subject's simulator pose (EVALUATION only: the
# gate never reads it) every this many seconds, so a gate that timed out can
# say whether the subject drove past the camera or never came.
GATE_TRUTH_PERIOD_S = 2.0
# THE DETECTOR LOADS BEFORE THE MISSION (2026-10-21, open item C14). The
# Grounder thread used to be started after the take-off, at mission start,
# so OWL-ViT's import, weight load and CUDA start ran beside the first ticks:
# on pas_ap_ovf_on_20261021 the first inference came 26.2 s into the mission
# (its own inference 1709 ms; first consumed at 26.3 s) and 12 of the 263
# tick intervals before it ran over 105 ms against 9 of the 926 after,
# among them the 114 ms interval of the flight's one 97.9 ms
# Shield step (t = 19.4 s, a plain search tick with no detection; the
# Shield's median is about 1 ms, so most likely the loading thread holding
# the GIL - inferred, not profiled). The Grounder now starts as soon as the camera is up - its load runs
# under SITL's boot and the take-off - and the mission waits for its first
# inference (at most this long, then flies, recorded). Since 2026-10-22
# (review) that wait is made in the pre-arm step, disarmed, on the links that
# have one (PymavlinkLink, MavrosLink): a slow load is no airborne hover. A
# detector thread that DIED is not waited for: no mission is flown on it.
DETECTOR_READY_TIMEOUT_S = 300.0
# THE SIMULATOR'S EVALUATION RPCs LEAVE THE LOOP (OffLoopRpc; 2026-10-21,
# C14). The subject truth's GetObjectPoses, the truth's frame check and the
# EKF-against-simulator check are synchronous requests on Project AirSim's
# service socket, answered on the simulator's game thread. On
# pas_ap_ovf_on_20261021 one truth read took 1024.9 ms (subject_truth.reader.
# read_ms max): that tick's work was 1028 ms (tick_work_ms max), and the
# aircraft flew its last setpoint for a second with no Shield step and no
# pose. The RPCs now run on one worker thread of their own (still one request
# at a time on that socket), and the loop waits for them at most this long
# PER TICK, all of a tick's calls together (OffLoopRpc.begin_tick; review
# 2026-10-22: until then per call, so a truth read and a frame check, each
# answered just inside 50 ms, held the loop 100 ms - a whole tick); a later
# answer is dropped and counted, and a call the tick has no time left for is
# not sent (metrics.json sim_rpc).
SIM_RPC_WAIT_S = 0.05
# A fence upload in flight blocks the 10 Hz loop (one mission-protocol round
# trip per item, twice): each phase gets this long, so an autopilot that does
# not answer costs at most about 6 ticks, not the 2 x 10 s of the pre-arm one.
MIDFLIGHT_FENCE_TIMEOUT_S = 0.3
# Reading the fence list the autopilot held before the first upload.
FENCE_READ_TIMEOUT_S = 5.0
# A FROZEN LINK IS NOT A HOVERING AIRCRAFT (2026-10-19). link.state() answers
# from the last position the link ever received, so a SITL that died behind
# mavlink-router (the socket stays open and simply goes quiet), a closed TCP
# socket (pymavlink prints "EOF on TCP socket" and does not reconnect) or a
# MAVROS that lost its FCU left the loop filtering, commanding and logging a
# frozen position as flown until max_s - with no word in metrics.json. Each
# tick now asks the link how old its position fix and the vehicle's heartbeat
# are (link_age()): a position older than POS_STALE_S (GLOBAL_POSITION_INT
# comes at 20 Hz on pymavlink, 10 Hz through MAVROS; the slack is for WSL NAT
# hiccups), a heartbeat older than HEARTBEAT_STALE_S (1 Hz) or MAVROS saying
# the FCU is disconnected makes the tick STALE: the FSM is not stepped, no
# flown row is written and nothing is commanded but ONE zero-velocity hold at
# the span's start while the node was streaming (STALE_HOLD in run(): with the
# outbound path alive it stops the aircraft at once, where silence left it on
# the last setpoint until ArduCopter's GUID_TIMEOUT, 3 s by default and not
# changed by the param file). Staleness that lasts LINK_LOST_S ends the
# episode as "link lost", which lands the aircraft unless the autopilot owns
# it. Every stale span is an event and counted (metrics.json link_freshness).
POS_STALE_S = 1.0
HEARTBEAT_STALE_S = 3.0
LINK_LOST_S = 3.0
# The OS timer the loop's sleep wakes on (main(), _fine_timer_begin): None
# until measured - run() alone, as the tests call it, never touches it.
TIMER_RESOLUTION_MS = None


def theta_horizon(arg: str | None = "auto") -> float | None:
    """What the FSM measures theta on. 'auto' (and 'none'): the default
    basis, guardrail.fsm.DEFAULT_THETA_HORIZON_S - each repair's own
    magnitude_m, the forecast depth it corrected (the grant's p5 audit
    field; the PI reference's metric). A number: the setpoint change |dv|
    over that many seconds; 0.1 is the one-period reading
    (guardrail.fsm.THETA_HORIZON_S), a PI question and not the default,
    because under the shipped caps it can never exceed theta. (A builder
    made 0.1 the default on 2026-10-08; reverted on review the next day.)"""
    if arg is None or str(arg).strip().lower() in ("auto", "none"):
        return DEFAULT_THETA_HORIZON_S if arg != "none" else None
    h = float(arg)
    if not math.isfinite(h) or h <= 0:
        raise ValueError(f"--theta-horizon-s must be 'auto', 'none' or seconds > 0, "
                         f"got {arg!r}")
    return h


def theta_basis(horizon: float | None) -> dict:
    """How theta's repair magnitude is measured on this run (metrics.json
    `fsm.theta_basis`; guardrail.kpi reads `horizon_s` back from it)."""
    if horizon is None:
        return {"source": "per-operator magnitude_m", "horizon_s": None}
    return {"source": ("setpoint change |dv| over one control period (the one-period "
                       "reading; selectable, pending the PI)"
                       if abs(horizon - THETA_HORIZON_S) < 1e-9
                       else "velocity proxy |dv| * h"),
            "horizon_s": horizon}


# The ArduCopter mode the FSM's LOITER is flown as (BRAKE: see
# sitl/mavlink_adapter_node.py DEFAULT_HOLD_MODE). `--hold-mode LOITER`
# restores the literal mapping for a flight with a pilot on the throttle.
HOLD_MODE = DEFAULT_HOLD_MODE

# THE OWN-VELOCITY FORECAST, ON BY DEFAULT ON THIS RAIL (2026-10-19;
# docs/DESIGN-own-velocity-forecast.md). The Shield gets the autopilot's
# measured velocity (link.velocity(): LOCAL_POSITION_NED, else the velocity
# GLOBAL_POSITION_INT carries; /mavros/local_position/velocity_local on the
# MAVROS link) and the setpoints this node sent (Shield.note_flown), so its
# forecast flies ArduCopter's measured response instead of taking up every
# command at once; and the FSM's Loiter is flown as a GUIDED stop through the
# Shield until BRAKE's predicted stop ends legal (guardrail.response.
# hold_plan), and the FSM's "brake" setpoint as the Shield's answer to a stop
# (Shield.advise). The measured velocity only ADDS to the commanded forecast
# (Shield._predict): nothing the old Shield refused is emitted clean; and
# while the position already breaks a hard rule it is set aside, so the
# escape is the old Shield's (Shield._decision_state). ON because the
# evidence the task asked for holds, on both P0 escape bases (re-checked with
# the velocity, or on the commanded forecast as every stored run was), with
# the response numbers chosen so that their median braking error at 1.5 s is
# not short (2026-10-20; a median, not a bound: p25 -0.33 m, and -0.16 m in
# the median at 2.0 s - guardrail/response.py):
# replayed on all six recorded pas_ap flights, the Shield's P0 escapes are
# the same ticks with and without it (0, 0, 1 [the same tick], 0, 0, 0);
# flown in closed loop over every whole flight on three aircraft (the
# identified one, one as slow as the Shield's model, one slower), the
# position was illegal on fewer ticks in 17 of the 18 runs and on none in
# the other (0 against 0), with no P0 escape and no fail-safe in any, and on
# the identified aircraft the 14 Oct and 13 Oct close approaches stay outside
# the building margin (min clearance 3.28 and 3.48 m against 3.0).
# `--own-velocity-forecast off` flies the commanded forecast and BRAKE
# exactly as before; state_v is logged either way.
OWN_VELOCITY_DEFAULT = "on"

# THE PILOT (module docstring, THE VLA PILOT). "servo": follow_vlm's servo law
# on the detector's box (FollowPilot), the default; "vla": AerialVLA through
# the step tracker (VLAPilot). The missions each pilot flies.
PILOTS = ("servo", "vla")
PILOT_MISSIONS = {"servo": ("follow",), "vla": ("goto", "approach", "follow")}
# AerialVLA's two cameras in robot_citylife_ardupilot.jsonc (to-do VLA-03),
# the CityLife VLA flight's names (demo/vla_fly.CAMERAS).
VLA_CAMERAS = {"front": "VLAFront", "down": "VLADown"}
# THE VLA FOLLOW's options (module docstring): set only for --pilot vla
# --mission follow, refused for any other run; their defaults are
# demo/vla_fly.py's own (its parser), read there, never copied here.
VLA_FOLLOW_ONLY = ("vla_step_as_rate", "follow_memory_s", "follow_search_s",
                   "close_bottom_frac", "close_width_frac")
# What a VLA run's metrics.params adds (as resolved by check_pilot_args).
VLA_PARAMS = ("vla_front_source", "vla_step_as_rate", "vla_rate_yaw", "follow_memory_s",
              "follow_search_s", "close_bottom_frac", "close_width_frac", "lost_timeout_s",
              "vla_cruise_mps", "vla_speed_cap_mps", "allow_unstamped_pipelined",
              "front_capture_interval_s", "target_estimator", "vla_feed", "vla_feed_max_hz")
# Exit codes beside 0 and 1 (demo/vla_fly.py's): a VLA failure ended the
# mission (vla_mission.VLA_FAILURE_ENDS), and a run refused before arming.
EXIT_VLA_FAILURE = 3
EXIT_REFUSED = 4


class PreArmRefused(RuntimeError):
    """The run is refused in the pre-arm step, disarmed: nothing was armed
    (`kind`, e.g. "warmup_refused"; the message says why)."""

    def __init__(self, kind: str, why: str):
        super().__init__(why)
        self.kind = kind


def _fv():
    """demo/follow_vlm.py, imported on first use (1.3 s; no GPU, no sim)."""
    return importlib.import_module("follow_vlm")


def _vm():
    """demo/vla_mission.py, imported for --pilot vla only (stdlib and
    guardrail; no simulator, no model)."""
    return importlib.import_module("vla_mission")


def _vf():
    """demo/vla_fly.py, the city rail's VLA flight, imported for its follow's
    configuration only (its parser, resolve_args, tracker_config, its lost
    timeouts): nothing of it flies here. Module level is stdlib, guardrail
    and vla_mission; the simulator and the detector are imported inside its
    functions."""
    return importlib.import_module("vla_fly")


# --------------------------------------------------------------------------- #
# Pilot
# --------------------------------------------------------------------------- #

class FollowPilot:
    """follow_vlm's servo law, unchanged, on the Grounder's latest box.

    `detections` is anything with follow_vlm.Grounder's latest() shape:
    {"det": (cx, cy, w, h, score, W, H, colour) | None, "t_det", "seq",
     "n_seen", "n_miss"}, and "cap" (the published box's capture: pose,
    times, depth source and range) when the Grounder has one. step() hands
    the box on with the "cap" and "t_det" of the SAME latest() answer: a
    second latest() call could pair box N with capture N+1.

    THE SWEEP'S FIRST SWING GOES WHERE THE BOX WENT (2026-10-19).
    follow_vlm.search_sweep_rate is a cosine that always starts turning RIGHT
    (+0.685 rad/s at t = 0 for 25 deg / 4 s). A box lost off the LEFT edge -
    the servo turning left after it at up to yaw_gain x 45 deg - got a yaw
    step of over 1 rad/s at the track-to-search hand-over and a first
    half-swing away from where the subject went (the October follow_vlm
    review's verify_sweep_sim: mirrored left and right losses were out of
    view on 46 against 35 of 50 search ticks). The sweep is now signed by
    the side of the bearing the box was last tracked on (`sweep_side`, +1
    right, -1 left; +1 before any box), fixed for the whole search so the
    swing stays a zero-net-rotation oscillation. The sign is applied here,
    not through a follow_vlm keyword, so an injected `sweep` keeps its own
    signature.
    """

    def __init__(self, detections, hfov_deg: float, yaw_gain: float = 1.2,
                 want_w_frac: float = 0.10, speed_max: float = 4.0,
                 cruise_alt: float = 8.0, alt_gain: float = 0.6,
                 climb_max: float = 1.8, stale_s: float = DETECTION_STALE_S,
                 sweep_deg: float = 25.0, sweep_period_s: float = 4.0,
                 servo=None, sweep=None):
        fv = None if (servo and sweep) else _fv()
        self.servo = servo or fv.servo
        self.sweep = sweep or fv.search_sweep_rate
        self.detections = detections
        self.hfov = hfov_deg
        self.yaw_gain, self.want_w_frac, self.speed_max = yaw_gain, want_w_frac, speed_max
        self.cruise_alt, self.alt_gain, self.climb_max = cruise_alt, alt_gain, climb_max
        self.stale_s = stale_s
        self.sweep_deg, self.sweep_period_s = sweep_deg, sweep_period_s
        self._t_lost = None
        # The bearing the box was last tracked on, and the side the current
        # search's first swing goes (class docstring).
        self._last_bearing = None
        self.sweep_side = 1.0
        self.n_track = self.n_search = 0

    def step(self, now: float, up: float) -> tuple[tuple, dict]:
        lat = self.detections.latest()
        det, t_det = lat.get("det"), lat.get("t_det") or 0.0
        fresh = det is not None and t_det > 0 and (now - t_det) <= self.stale_s
        vz = float(np.clip((self.cruise_alt - up) * self.alt_gain,
                           -self.climb_max, self.climb_max))
        if fresh:
            yaw_rate, fwd, bearing = self.servo(det, int(det[5]), self.yaw_gain,
                                                self.want_w_frac, self.speed_max,
                                                self.hfov)
            self._t_lost = None
            self._last_bearing = float(bearing)
            mode = "track"
            self.n_track += 1
        else:
            if self._t_lost is None:
                self._t_lost = now
                # Toward the side the box was last seen on, for this whole
                # search: the yaw rate keeps the servo's sign across the
                # hand-over instead of reversing it.
                self.sweep_side = (-1.0 if self._last_bearing is not None
                                   and self._last_bearing < 0.0 else 1.0)
            yaw_rate = self.sweep_side * self.sweep(now - self._t_lost, self.sweep_deg,
                                                    self.sweep_period_s)
            fwd, bearing, mode = 0.0, None, "search"
            self.n_search += 1
        return ((float(fwd), 0.0, vz, float(yaw_rate)),
                {"mode": mode, "fresh": bool(fresh), "seq": lat.get("seq"),
                 "bearing": None if bearing is None else float(bearing),
                 "sweep_side": None if fresh else self.sweep_side,
                 "det": det, "cap": lat.get("cap"), "t_det": t_det})


# THE ESTIMATOR FOLLOW's control numbers (EstimatorFollowPilot): follow_vlm's
# own parser defaults, read there (build_arg_parser), never copied here.
ESTIMATOR_FOLLOW_KEYS = ("range_gain", "range_feedforward", "catchup_mps", "yaw_los_ff",
                         "yaw_los_ff_cap", "yaw_los_ff_min_range", "coast_s",
                         "trail_lookahead_m")
ESTIMATOR_FOLLOW_LAWS = ("estimate", "box", "memory", "sweep")
ESTIMATOR_FOLLOW_SOURCE = (
    "follow_vlm.fly()'s track branches (the servo on the target estimate: yaw_command "
    "with the line-of-sight feed-forward, standoff_speed with the range feed-forward and "
    "the catch-up, the stopped-subject floor; else the box servo at servo_want_width) "
    "and its forward direction (--trail-follow: toward a carrot on the subject's trail, "
    "trail.direction, never away from the estimate), on demo/subject_track.SubjectTrack "
    "(the estimator, the strict lock's prior, the Reacquirer, the trail), by "
    "perception_node.EstimatorFollowPilot")


class EstimatorFollowPilot(FollowPilot):
    """THE ESTIMATOR FOLLOW (--pilot servo --mission follow --identity
    --target-estimator; FAMILY=estimator in deploy/orin/run_pas_perception.sh):
    the lab's hand-written follow WITH its target estimator, as a null of the
    VLA backend table's C2 (tools/vla_backend_table.py) on the same rail, the
    same detector feed, the same start gate and the same Shield as the C2 VLA
    flights. The plain FollowPilot (no --target-estimator) is unchanged.

    WHY. C2's first null (FollowPilot) is a box-width P-servo with no
    estimate and no lock (within 30 m 0.065-0.111 on hil, 16 Nov), while the
    lab's own hand-written follow, follow_vlm.fly() with its target estimator,
    held 0.952 within 30 m on the AGX's simple_flight rail
    (agx_manual_20261103). This pilot puts that follow's control law on this
    rail, from follow_vlm's own functions, imported, not copied:

      * the subject: demo/subject_track.SubjectTrack, stepped once per tick
        on the Grounder's answer BEFORE the law reads it (the estimator fed
        identity's range and bearing at each box's capture, the strict lock
        leaning on its prior, the lapse and the anchored Reacquirer, the
        trail), as the VLA follow's --target-estimator steps it;
      * "estimate" (fly()'s first track branch, "SERVO ON THE ESTIMATE"):
        while the estimator serves one (TargetState.observe, up to its 3 s
        coast) the yaw is follow_vlm.yaw_command on the estimate's bearing
        plus yaw_los_ff x follow_vlm.los_rate (only on an estimate updated
        under 1 s ago, capped, ranged at least yaw_los_ff_min_range), and
        the forward speed follow_vlm.standoff_speed on the estimate's range
        against the stand-off (target_state.want_range_from_width of
        --want-width and the subject's width), with the subject's own
        opening rate fed forward and the catch-up saturated at catchup_mps,
        times cos(bearing); a subject MEASURED stopped (under 1 m/s) whose
        estimate is over 0.5 s old is not closed nearer than the camera's
        blind range plus 2 m (follow_vlm.camera_blind_m);
      * "box" (fly()'s second track branch): a fresh box and no
        re-acquisition open - follow_vlm.servo at the box fraction that
        holds the same metric stand-off (follow_vlm.servo_want_width);
      * "memory" (fly()'s coast, without its trail approach): no estimate
        and no box, the subject still held in memory (SubjectTrack.
        memory_bearing: the trail's lookout) - the yaw toward it, the last
        track forward speed decaying to 0 over coast_s, then 0 (fly()'s
        last_cmd[1] x k: a pilot that was backing off keeps backing);
      * "sweep": nothing to go on - FollowPilot's search (forward 0, the
        sweep's first swing toward the side the subject was last on).

    WHICH WAY FORWARD IS (fly()'s --trail-follow, on by default here, as
    agx_manual_20261103 flew it): on an "estimate", "box" or "memory" tick
    with a forward speed over 0 the speed above is flown toward a carrot
    trail_lookahead_m along the subject's trail (trail.direction on the
    SubjectTrack's trail), not along the nose - a body forward AND right
    component, the yaw still on the subject. While an estimate is served a
    trail direction more than 90 deg off the line of sight to it is refused
    (fly()'s NEVER AWAY FROM THE SUBJECT) and the nose wins; backing off (a
    forward speed under 0) stays along the nose. trail_follow=False flies
    along the nose only.

    The aircraft's own velocity for the line-of-sight rate is the link's
    measured one when given (`vel`, North / East / up), else the pilot's
    last commanded horizontal velocity (fly()'s last_emit_v, which is what
    the guard emitted; here it is the pilot's own command, before the
    Shield).

    WHAT THIS IS NOT. fly()'s whole controller: its trail approach in the
    coast (the coast flies the trail's direction, not the approach's stop
    short of the trail's end), its search sweep closed on the lookout
    bearing, the search creep and the search planner, FenceGuard (its gate
    and smoothing, and clear_aim, which keeps fly()'s aim out of the zones'
    bands), the slew limiter (VecSlewLimiter) and the orbit are not ported;
    on this rail the Shield and ArduPilot's own velocity controller stand
    where FenceGuard and the slew limiter did. It has no lost end: like
    FollowPilot it flies to max_s (the table replays it under the VLAs' end
    rule, as it does the servo null). Its numbers are a null on THIS rail,
    not a re-flight of the simple_flight follow.

    step(now, up, pose=(x, y, yaw rad), vel=None) -> (body action, info) in
    FollowPilot's shape, with info's "law" and "track_src"; row_fields(now,
    st) the tick's est_* fields (SubjectTrack.row) and the law; summary()
    metrics.json's `estimator_follow`. info's "mode" is "track" on an
    "estimate" tick too (the start gate's and the records' sense); the HUD
    reads the law (track_words' `law`) and claims TARGET LOCKED only on a
    fresh box. A fault in the SubjectTrack is counted
    (`track_errors`, the first kept), never raised: the law falls back on
    the box and the sweep, and the 10 Hz loop and the Shield go on."""

    def __init__(self, detections, hfov_deg: float, track, *, object_width_m: float,
                 yaw_gain: float = 1.2, want_w_frac: float = 0.16, speed_max: float = 5.0,
                 cruise_alt: float = 8.0, alt_gain: float = 0.6, climb_max: float = 1.8,
                 stale_s: float = DETECTION_STALE_S, sweep_deg: float = 25.0,
                 sweep_period_s: float = 4.0, trail_follow: bool = True, fv=None):
        fv = fv or _fv()
        super().__init__(detections, hfov_deg, yaw_gain=yaw_gain, want_w_frac=want_w_frac,
                         speed_max=speed_max, cruise_alt=cruise_alt, alt_gain=alt_gain,
                         climb_max=climb_max, stale_s=stale_s, sweep_deg=sweep_deg,
                         sweep_period_s=sweep_period_s, servo=fv.servo,
                         sweep=fv.search_sweep_rate)
        if not (object_width_m and object_width_m > 0):
            raise ValueError("the estimator follow needs the subject's width (its stand-off)")
        self.fv = fv
        self.track = track
        ap = fv.build_arg_parser()
        self.law = {k: ap.get_default(k) for k in ESTIMATOR_FOLLOW_KEYS}
        self.object_width_m = float(object_width_m)
        # fly()'s _stand_off() with --want-range 0: the metric stand-off the
        # width fraction asks for, and the box fraction that holds it on the
        # pinhole camera (servo_want_width), so both branches fly one set-point.
        self.want_range_m = float(fv.want_range_from_width(want_w_frac, self.object_width_m))
        self.box_want_w_frac = float(fv.servo_want_width(self.want_range_m,
                                                         self.object_width_m, hfov_deg))
        self.track_errors = 0
        self.track_error: str | None = None
        self.n_by_law: collections.Counter = collections.Counter()
        self.n_coast = 0
        self.n_los_ff = 0
        self.n_floor = 0
        self.trail_follow = bool(trail_follow)
        self.n_trail = 0
        self.n_trail_against = 0
        self.last_law: str | None = None
        self._t_seen: float | None = None
        self._last_fwd = 0.0
        self._last_v = (0.0, 0.0)      # the last commanded horizontal velocity (N, E)

    def config(self) -> dict:
        return {"source": ESTIMATOR_FOLLOW_SOURCE, "law_defaults_from":
                "follow_vlm.build_arg_parser()", **self.law,
                "trail_follow": self.trail_follow,
                "yaw_gain": self.yaw_gain, "want_width": self.want_w_frac,
                "want_range_m": round(self.want_range_m, 3),
                "box_want_width": round(self.box_want_w_frac, 4),
                "object_width_m": self.object_width_m, "speed_max": self.speed_max,
                "stale_s": self.stale_s}

    def _track_call(self, fn, *a):
        """A SubjectTrack call, its fault counted and never raised (class
        docstring); None on a fault."""
        try:
            return fn(*a)
        except Exception as exc:                                  # noqa: BLE001
            self.track_errors += 1
            if self.track_error is None:
                self.track_error = f"{type(exc).__name__}: {exc}"
                print(f"[track]  *** {self.track_error} (counted; the loop goes on)")
            return None

    def step(self, now: float, up: float, pose=None, vel=None) -> tuple[tuple, dict]:
        if pose is None:
            raise ValueError("the estimator follow steps on the aircraft's pose (x, y, yaw)")
        fv, tr = self.fv, self.track
        x, y, yaw = (float(v) for v in pose)
        lat = self.detections.latest()
        lat = lat if isinstance(lat, dict) else {}
        # The estimate first, on this tick's answer (SubjectTrack.step's order).
        self._track_call(tr.step, lat, now, (x, y, yaw))
        det, t_det = lat.get("det"), lat.get("t_det") or 0.0
        fresh = det is not None and t_det > 0 and (now - t_det) <= self.stale_s
        vz = float(np.clip((self.cruise_alt - up) * self.alt_gain,
                           -self.climb_max, self.climb_max))
        reacq_open = getattr(tr, "reacq", None) is not None
        est = None if reacq_open else self._track_call(tr.estimator.observe, now, x, y, yaw)
        bearing, track_src = None, None
        if est is not None:
            # SERVO ON THE ESTIMATE (fly()).
            bearing, rng_est = (float(v) for v in est)
            e = tr.estimator
            ff_yaw = 0.0
            if (self.law["yaw_los_ff"] > 0.0 and e.x is not None
                    and e.t_last_update is not None and now - e.t_last_update < 1.0):
                if vel is not None:
                    vax, vay = float(vel[0]), float(vel[1])
                else:
                    vax, vay = self._last_v
                ff_yaw = float(np.clip(
                    self.law["yaw_los_ff"] * fv.los_rate(
                        float(e.x[0]), float(e.x[1]), float(e.x[2]), float(e.x[3]),
                        x, y, vax, vay, r_min=self.law["yaw_los_ff_min_range"]),
                    -self.law["yaw_los_ff_cap"], self.law["yaw_los_ff_cap"]))
                self.n_los_ff += int(ff_yaw != 0.0)
            yaw_rate = fv.yaw_command(self.yaw_gain, bearing, ff=ff_yaw)
            ff = e.range_rate(x, y) if self.law["range_feedforward"] else 0.0
            fwd = fv.standoff_speed(rng_est, self.want_range_m, self.law["range_gain"], ff,
                                    self.speed_max, self.law["catchup_mps"])
            fwd *= max(0.0, math.cos(bearing))
            # PREDICTION IS NOT EVIDENCE (fly()): a subject measured stopped.
            if (tr.last_acc is not None and e.t_last_update is not None
                    and tr.last_meas_speed is not None and tr.last_meas_speed < 1.0
                    and now - e.t_last_update > 0.5):
                r_last = math.hypot(tr.last_acc[0] - x, tr.last_acc[1] - y)
                floor = fv.camera_blind_m(up) + 2.0
                capped = min(fwd, max(0.0, 0.5 * (r_last - floor)))
                self.n_floor += int(capped < fwd)
                fwd = capped
            law, mode = "estimate", "track"
        elif fresh and not reacq_open:
            yaw_rate, fwd, bearing = self.servo(det, int(det[5]), self.yaw_gain,
                                                self.box_want_w_frac, self.speed_max,
                                                self.hfov)
            law, mode = "box", "track"
        else:
            law, mode, fwd, yaw_rate = None, None, 0.0, 0.0
            mem = self._track_call(tr.memory_bearing, now, x, y, yaw)
            lost = 0.0 if self._t_seen is None else now - self._t_seen
            if mem is not None:
                rel = (float(mem[0]) - yaw + math.pi) % (2.0 * math.pi) - math.pi
                yaw_rate = fv.yaw_command(self.yaw_gain, rel)
                cs = float(self.law["coast_s"])
                k = max(0.0, 1.0 - lost / cs) if cs > 0 else 0.0
                fwd = self._last_fwd * k                # fly()'s last_cmd[1] * k
                law, mode, track_src = "memory", "coast", mem[1]
                self._t_lost = None
                self.n_coast += 1
            else:
                # FollowPilot's search: forward 0, the sweep signed by the
                # side the subject was last tracked on.
                if self._t_lost is None:
                    self._t_lost = now
                    self.sweep_side = (-1.0 if self._last_bearing is not None
                                       and self._last_bearing < 0.0 else 1.0)
                yaw_rate = self.sweep_side * self.sweep(now - self._t_lost, self.sweep_deg,
                                                        self.sweep_period_s)
                law, mode = "sweep", "search"
                self.n_search += 1
        if mode == "track":
            self._t_lost = None
            self._t_seen = now
            self._last_bearing = float(bearing)
            self._last_fwd = float(fwd)
            self.n_track += 1
        # WHICH WAY "FORWARD" IS (fly()'s --trail-follow; class docstring).
        ux, uy = math.cos(yaw), math.sin(yaw)
        trail_dir = None
        trail = getattr(tr, "trail", None)
        if self.trail_follow and trail is not None and fwd > 0.0 and mode in ("track",
                                                                              "coast"):
            tdir = self._track_call(fv.trail_mod.direction, trail, x, y,
                                    float(self.law["trail_lookahead_m"]))
            e = getattr(tr, "estimator", None)
            if (tdir is not None and est is not None and e is not None
                    and e.x is not None):
                # NEVER AWAY FROM THE SUBJECT (fly()): the nose wins.
                if (float(e.x[0]) - x) * tdir[0] + (float(e.x[1]) - y) * tdir[1] < 0.0:
                    tdir = None
                    self.n_trail_against += 1
            if tdir is not None:
                ux, uy = float(tdir[0]), float(tdir[1])
                trail_dir = math.atan2(uy, ux)
                self.n_trail += 1
        # The horizontal command (North, East) as the body action: forward
        # and right of the nose (guardrail.frames.from_body's inverse).
        vn, ve = float(fwd) * ux, float(fwd) * uy
        self._last_v = (vn, ve)
        body_fwd = vn * math.cos(yaw) + ve * math.sin(yaw)
        body_right = -vn * math.sin(yaw) + ve * math.cos(yaw)
        if trail_dir is None:                     # along the nose: exactly fwd, right 0
            body_fwd, body_right = float(fwd), 0.0
        self.n_by_law[law] += 1
        self.last_law = law
        return ((float(body_fwd), float(body_right), vz, float(yaw_rate)),
                {"mode": mode, "fresh": bool(fresh), "seq": lat.get("seq"),
                 "bearing": None if bearing is None else float(bearing),
                 "sweep_side": self.sweep_side if law == "sweep" else None,
                 "law": law, "track_src": track_src, "fwd_speed": float(fwd),
                 "trail_dir": trail_dir,
                 "det": det, "cap": lat.get("cap"), "t_det": t_det})

    def row_fields(self, now: float, st) -> dict:
        """The tick's flight-log fields: SubjectTrack.row (est_*, lock_why,
        reacq_streak) and the law that flew it (`follow_law`)."""
        row = self._track_call(self.track.row, now, float(st.x), float(st.y),
                               math.radians(float(st.yaw_deg))) or {"est_state": None}
        row["follow_law"] = self.last_law
        return row

    def summary(self) -> dict:
        """metrics.json `estimator_follow`: the law's configuration, the ticks
        each law flew, the line-of-sight feed-forward and the stopped-subject
        floor's ticks, the ticks whose forward followed the subject's trail
        (and those whose trail direction was refused as pointing away from
        the estimate), and the SubjectTrack's faults."""
        n = sum(self.n_by_law.values())
        return {"on": True, "config": self.config(),
                "ticks": n, "ticks_by_law": {k: int(self.n_by_law.get(k, 0))
                                             for k in ESTIMATOR_FOLLOW_LAWS},
                "estimate_frac": (round(self.n_by_law.get("estimate", 0) / n, 4)
                                  if n else None),
                "los_ff_ticks": self.n_los_ff, "stopped_floor_ticks": self.n_floor,
                "trail_ticks": self.n_trail,
                "trail_refused_away_from_subject": self.n_trail_against,
                "track_errors": self.track_errors, "first_track_error": self.track_error}


class NoDetector:
    """The detector of a run that runs none (--pilot vla --mission goto: the
    hint comes from the goal, and OWL-ViT on the GPU the model and Unreal
    share would cost decisions for nothing, as on the city rail's M1):
    Grounder.latest()'s shape with nothing ever seen, and no thread, so it is
    never "dead" (detector_thread_dead) and never waited for."""

    def latest(self) -> dict:
        return {"det": None, "t_det": 0.0, "seq": 0, "n_seen": 0, "n_miss": 0}

    def stop(self) -> None:
        return None


def box_of(g) -> tuple:
    """One follow_vlm.Grounder.latest() answer as (the box, the wall time its
    frame was captured, the heading at that frame, when the detector
    published it) - demo/vla_fly.py's latest_box; (None,)*4 without a box."""
    if not g or g.get("det") is None:
        return None, None, None, None
    cap = g.get("cap") or {}
    yaw = (cap.get("pose") or {}).get("yaw")
    return (g["det"], (cap.get("t_capture") or cap.get("t_cap") or g.get("t_det")),
            None if yaw is None else float(yaw), g.get("t_det"))


def grounder_box_source(grounder):
    """A `latest()` for vla_mission.DetectorHint and the mission monitor:
    box_of the Grounder's latest answer (its "cap" carries the frame's
    pose)."""
    def latest_box():
        return box_of(grounder.latest() if grounder is not None else None)
    return latest_box


# What RemoteVLABackend.act() did about a request on one tick, read from the
# change in its counts: "sent", "no_hint" (a capture was built and the hint
# said no request: no fresh box past the follow's memory and search, a close
# box), "no_capture" (no frame pair), "error" (building the request raised),
# None (no attempt: a request in flight, the stop-and-go settle, a suspended
# tracker, a LAND, the client's backoff).
REQUEST_OUTCOMES = (("requests", "sent"), ("no_hint", "no_hint"),
                    ("no_capture", "no_capture"), ("request_errors", "error"))


def request_outcome(before: dict | None, after) -> str | None:
    """REQUEST_OUTCOMES' word for the change from `before` to `after` (the
    backend's counts around one act()); None when either is unknown or
    nothing changed."""
    if not isinstance(before, dict) or not isinstance(after, dict):
        return None
    for key, word in REQUEST_OUTCOMES:
        if int(after.get(key, 0) or 0) > int(before.get(key, 0) or 0):
            return word
    return None


class VLAPilot:
    """AerialVLA in the pilot's slot (module docstring, THE VLA PILOT).

    `backend` is guardrail.vla_backends.RemoteVLABackend (the step tracker's
    world-frame action on every call, never blocking on the model);
    `monitor` the mission's stop rules (vla_mission.MissionMonitor);
    `detections` the Grounder for the approach (None for a GOTO).

    step(now, st) -> (raw, body, info): the tracker's action IS the raw
      action the Shield judges - nothing stands between them - with its
      body-frame copy (guardrail.frames.to_body: the row's `body` and
      MAVROS's record copy) and FollowPilot's info shape. The detector's box,
      its capture and its freshness pass through as FollowPilot passes them,
      so the Shield's subject stays the perception's estimate, independent of
      the VLA (the integration report's rule).
    after(now, st, flies=...) -> (end reason or None, row fields, HUD
      fields), once per tick after the FSM chose its setpoint: the row and
      the HUD read the decision behind this tick (vla_mission.decision_fields,
      mission_fields, vla_hud - the city rail's code); then a tick on which
      the pilot's action did not fly (the FSM's brake, a stop it advised, a
      hold, a fault, a takeover) suspends the tracker and stops the mission's
      timers (MissionMonitor.hold), and the next one that flies resumes both,
      so every answer to an image from before the hold is refused
      (vla_link.StepTracker.resume) and no VLA end names a mission the rail
      was holding; on a tick that flies, the mission's stop rules (LAND, a
      dead / slow / erroring server, the mission's own ends).
    hold(now): the same hold between ticks (a stale-link span).
    close() -> the backend's stats, read before its close (the decision rate
      is over the time act() ran) with the log's counts after it.

    A FOLLOW (THE VLA FOLLOW, `close_hold` a vla_mission.BoxClose) holds the
    horizontal motion while a fresh box counts as close, as the city rail's
    MissionLoop does: inside the tracker when it smooths
    (backend.act(hold_horizontal=...)), else the tracker's action with its
    horizontal velocity zeroed; its turn and climb fly. The Grounder is read
    ONCE per tick (step), so the close hold, the stop rules and the row see
    the same box. Every tick also records what the backend did about a
    request (`vla_request`: sent / no_hint / no_capture / error / None for no
    attempt), from its counts - the observation-rate summary reads it.

    THE TARGET ESTIMATOR (`track`, demo/subject_track.SubjectTrack;
    --target-estimator): stepped in step() on that same Grounder answer
    BEFORE the backend acts, so the hint the backend asks for this tick
    (DetectorHint's `estimate`) reads this tick's estimate; after() adds its
    est_* fields and the hint's memory source to the row. A fault in it is
    counted (`track_errors`, the first one kept and printed), never raised:
    the 10 Hz loop and the Shield go on.
    """

    def __init__(self, backend, monitor, *, mission: str, goal_xy=None, detections=None,
                 hint_source=None, detector_hint=None, stale_s: float = DETECTION_STALE_S,
                 server_alive=lambda: True, vm=None, model_label: str | None = None,
                 close_hold=None, track=None):
        self.vm = vm or _vm()
        # The VLA panel's name for the model (vla_mission.model_label).
        self.model_label = model_label
        self.backend, self.monitor, self.mission = backend, monitor, mission
        self.goal_xy = None if goal_xy is None else (float(goal_xy[0]), float(goal_xy[1]))
        self.detections, self.stale_s = detections, float(stale_s)
        self.det_source = grounder_box_source(detections) if detections is not None else None
        self.hint_source, self.detector_hint = hint_source, detector_hint
        self.close_hold = close_hold
        self.server_alive = server_alive
        # The target estimator (class docstring), and its faults.
        self.track = track
        self.track_errors = 0
        self.track_error: str | None = None
        self.last_tick = None
        self.n_held = 0
        # Ticks the escalation FSM stepped on a raw action the rail itself had
        # zeroed (the suspended tracker's): its recovery timer then runs on a
        # zero the pilot never chose (review of 4a0da6d, P3-13: recorded).
        self.n_fsm_on_zeroed = 0
        self.n_stale_holds = 0
        # A follow's ticks whose horizontal motion the close hold held.
        self.n_close_hold = 0
        self.authors: collections.Counter = collections.Counter()
        self._t_last: float | None = None
        self._frozen: dict | None = None
        self._stats: dict | None = None
        # This tick's box (box_of), close-hold state, tracker action and
        # request outcome, from step() for after().
        self._box: tuple | None = None
        self._close: tuple[bool, bool] = (False, False)
        self._pilot_raw = None
        self._request: str | None = None
        self._no_hint_why: str | None = None
        # The backend's counts as the last step() left them: a request the
        # feed between ticks sent (pump) belongs to the next row.
        self._counts_mark: dict | None = None
        # The decision whose action last flew: the next tick that flies
        # another is its first setpoint (cam_to_setpoint_ms).
        self._seq_flown: int | None = None

    def hold(self, now: float) -> None:
        """The rail holds the aircraft between ticks (a stale-link span: one
        zero hold sent, nothing else; review of 4a0da6d, P3-13): the tracker
        is suspended and the mission's timers stop, as on a held tick. The
        next tick that flies resumes both."""
        self.n_stale_holds += int(not self.backend.tracker.suspended)
        self.backend.suspend()
        self.monitor.hold(now)
        self._t_last = now

    def pump(self) -> bool:
        """Between two ticks (--vla-feed stream; RemoteVLABackend.pump, THE
        FEED BETWEEN TICKS): a new front frame goes to the model without
        waiting for the tick. True when a request went out; a backend with
        no pump() is left alone."""
        fn = getattr(self.backend, "pump", None)
        return bool(fn()) if callable(fn) else False

    def detail(self) -> str | None:
        """The Dashboard heartbeat's word for the last tick."""
        tk = self.last_tick
        return None if tk is None else f"VLA {tk.author} #{tk.seq}"

    def _track_call(self, fn, *a):
        """The target estimator's call, its fault counted and never raised
        (class docstring); None on a fault."""
        try:
            return fn(*a)
        except Exception as exc:                                  # noqa: BLE001
            self.track_errors += 1
            if self.track_error is None:
                self.track_error = f"{type(exc).__name__}: {exc}"
                print(f"[track]  *** {self.track_error} (counted; the loop goes on)")
            return None

    def _close_now(self, now: float) -> bool:
        """A fresh box that counts as close this tick (the follow's close
        hold): MissionLoop's rule, on this tick's box."""
        if self.close_hold is None or self._box is None:
            return False
        det, t_box = self._box[0], self._box[1]
        if det is None or t_box is None:
            return False
        dh = self.detector_hint
        fresh_s = dh.fresh_s if dh is not None else self.stale_s
        if dh is not None and dh.box_ignored(t_box):
            return False
        return bool(now - float(t_box) <= fresh_s and self.close_hold(det))

    def step(self, now: float, st) -> tuple:
        from guardrail.frames import to_body
        from guardrail.models import Action4D
        lat = self.detections.latest() if self.detections is not None else {}
        lat = lat if isinstance(lat, dict) else {}
        # The Grounder read ONCE per tick: the close hold here, the stop rules
        # and the row in after() all see this box.
        self._box = box_of(lat) if self.detections is not None else None
        if self.track is not None:
            # The estimate first: the hint the backend may ask for below reads it.
            self._track_call(self.track.step, lat, now,
                             (float(st.x), float(st.y), math.radians(float(st.yaw_deg))))
        close_now = self._close_now(now)
        counts = getattr(self.backend, "counts", None)
        # Since the last step(): what pump() sent between the two ticks is
        # this row's too (with no feed the counts did not move in between).
        before = (self._counts_mark if self._counts_mark is not None else
                  dict(counts) if isinstance(counts, dict) else None)
        tracker = getattr(self.backend, "tracker", None)
        held_close = False
        if self.close_hold is not None and bool(getattr(tracker, "smoothing", False)):
            # SMOOTHING (vla_link.TrackerConfig): the close hold is the
            # tracker's input, so it ramps down at its acceleration limit.
            raw = self.backend.act(st, hold_horizontal=close_now)
            held_close = close_now
        else:
            raw = self.backend.act(st)
        pilot_raw = raw
        if (self.close_hold is not None and close_now and not held_close
                and (raw.vx or raw.vy)):
            raw = Action4D(vx=0.0, vy=0.0, vz_up=raw.vz_up, yaw_rate=raw.yaw_rate)
            held_close = True
        self.n_close_hold += int(held_close)
        self._close = (held_close, close_now)
        self._pilot_raw = pilot_raw
        after_counts = getattr(self.backend, "counts", None)
        self._request = request_outcome(before, after_counts)
        self._counts_mark = dict(after_counts) if isinstance(after_counts, dict) else None
        self._no_hint_why = None
        if self._request == "no_hint":
            kind = getattr(self.detector_hint, "last_kind", None)
            self._no_hint_why = ("close_box" if kind == "close" else
                                 "gated" if kind == "gated" else
                                 "no_fresh_box" if self.detector_hint is not None else None)
        self.last_tick = self.backend.last_tick
        body = to_body(raw, st.yaw_deg)
        det, t_det = lat.get("det"), lat.get("t_det") or 0.0
        fresh = det is not None and t_det > 0 and (now - t_det) <= self.stale_s
        return raw, body, {"mode": "vla", "fresh": bool(fresh), "seq": lat.get("seq"),
                           "bearing": None, "det": det, "cap": lat.get("cap"),
                           "t_det": t_det}

    def after(self, now: float, st, *, flies: bool, touched: bool, shield_on: bool,
              t_mission: float, fsm_stepped: bool = False,
              t_pub: float | None = None) -> tuple:
        """`t_pub`: when this tick's setpoint went out, on the loop clock
        (None: nothing was sent); with the capture time of the decision
        behind the flown action it gives the row's camera-to-setpoint
        fields (camera_age_fields)."""
        from guardrail.vla_link import Pose
        vm, tk = self.vm, self.last_tick
        pose = Pose.from_state(st)
        det = t_box = None
        if self.det_source is not None:
            got = self._box if self._box is not None else self.det_source()
            det, t_box, _, _ = vm.unpack_box(got)
        det_age = None if t_box is None else now - float(t_box)
        lf = getattr(self.backend, "last_link_failure", None)
        lf = lf if isinstance(lf, dict) else None
        cur, step = vm.current_decision(self.backend)
        gd, gb, frac = vm.goal_and_box(pose, self.goal_xy, det)
        zeroed = tk.reason == "suspended"
        self.n_fsm_on_zeroed += int(zeroed and fsm_stepped)
        row = {"pilot": "vla",
               **vm.decision_fields(tk, cur, step, hint_source=self.hint_source,
                                    link_failure=lf),
               "pilot_held": not flies,
               # The raw action this tick was the suspended tracker's zero.
               "raw_zeroed_by_hold": zeroed,
               # What the backend did about a request this tick (step).
               "vla_request": self._request,
               **vm.mission_fields(gd, gb, frac, det, det_age, self.detector_hint)}
        # CAMERA TO SETPOINT (2026-11-11): the frame behind the flown action,
        # aged at the publication. `cur` is the record of the decision being
        # flown, the tick's own (same seq); its t_capture is its request's
        # frame on the loop clock.
        t_cap = (cur.get("t_capture") if tk.seq is not None and cur.get("seq") == tk.seq
                 else None)
        row.update(camera_age_fields(t_pub, t_cap, flies=flies,
                                     fresh=tk.seq is not None and tk.seq != self._seq_flown))
        if flies and tk.seq is not None and t_pub is not None:
            self._seq_flown = tk.seq
        if self._request == "no_hint":
            row["vla_no_hint_why"] = self._no_hint_why
        if self.close_hold is not None:
            # As the city rail's MissionLoop rows: the close hold, a fresh
            # close box this tick, and the tracker's own action.
            row["close_hold"], row["close_box"] = self._close
            row["pilot_raw"] = (None if self._pilot_raw is None
                                else self._pilot_raw.model_dump())
        if self.detector_hint is not None and getattr(self.detector_hint, "record_kind", False):
            row["hint_kind"] = cur.get("hint_kind")
            row["hint_sent_kind"] = ((self.hint_source.last or {}).get("kind")
                                     if self.hint_source is not None else None)
        if self.track is not None:
            # THE TARGET ESTIMATOR's fields (SubjectTrack.row), and what the
            # hint rule's last memory aimed at: the estimate, the trail or
            # the remembered sighting (None: its last decision was no memory).
            row.update(self._track_call(self.track.row, now, float(st.x), float(st.y),
                                        math.radians(float(st.yaw_deg)))
                       or {"est_state": None})
            row["hint_memory_src"] = getattr(self.detector_hint, "last_memory_src", None)
        tracker = getattr(self.backend, "tracker", None)
        if getattr(getattr(tracker, "cfg", None), "rate_horizon_s", None) is not None:
            # RATE EXECUTION: the commanded speed and yaw rate of a step
            # flown as a velocity (vla_link.StepTracker.last_rate).
            row["vla_rate"] = getattr(tracker, "last_rate", None)
        self._t_last = now
        end = None
        if flies:
            self.backend.resume()
            self.monitor.unhold(now)
            try:
                alive = bool(self.server_alive())
            except Exception:                                 # noqa: BLE001
                alive = False
            # A follow's request count (MissionLoop's rule): one that never
            # asked ends subject_not_acquired, not as a VLA failure.
            counts = getattr(self.backend, "counts", None)
            sent = (int(counts.get("requests", 0))
                    if self.mission == "follow" and isinstance(counts, dict) else None)
            end = self.monitor.update(now, pose, tk, goal_xy=self.goal_xy, det=det,
                                      det_age_s=det_age, server_alive=alive, link_failure=lf,
                                      requests_sent=sent)
        else:
            # Held: the tracker is suspended and the mission's stop rules
            # wait (MissionMonitor.hold; review of 4a0da6d, P3-9): no VLA
            # end may name a mission the rail is holding.
            self.n_held += 1
            self.backend.suspend()
            self.monitor.hold(now)
        self.authors[tk.author] += 1
        hud = vm.vla_hud(t_mission, st.up, tk, cur, step, touched=touched,
                         shield_on=shield_on, mission=self.mission,
                         cadence=getattr(self.backend, "cadence", None),
                         goal_dist_m=gd, box_frac=frac, end_reason=end,
                         model=self.model_label)
        return end, row, hud

    def freeze(self) -> None:
        """Read the backend's stats now (the decision rate is over the time
        act() ran): at the mission's end, before the landing's time joins
        that span. Never raises."""
        if self._frozen is None:
            if self._t_last is not None:
                self.monitor.unhold(self._t_last)     # a hold open at the end counts
            try:
                self._frozen = self.backend.stats()
            except Exception as exc:                          # noqa: BLE001
                self._frozen = {"stats_error": f"{type(exc).__name__}: {exc}"}

    def close(self) -> dict:
        """Idempotent: the stats frozen at the mission's end (freeze) with the
        log's counts after the backend's close."""
        if self._stats is not None:
            return self._stats
        self.freeze()
        st = dict(self._frozen)
        try:
            self.backend.close()
        except Exception as exc:                              # noqa: BLE001
            st["close_error"] = f"{type(exc).__name__}: {exc}"
        after = self.backend.stats()
        st.update({k: after.get(k) for k in ("log_written", "log_errors", "last_log_error",
                                              "log_pending", "late_replies")})
        self._stats = st
        return st


class StartGate:
    """follow_vlm's start gate, ported (START_WHEN_SEEN above; open item C1).

    The decision is follow_vlm's own code, not a copy: `follow_vlm.Acquirer`
    judges every candidate of each fresh inference with its presence verdict
    (colour, wall-sized, the width a box implies at its depth range against
    what the named thing can be, and ground contact: a box whose bottom-edge
    ray meets the road far beyond its depth range is not standing on it - the
    traffic signal 4-5 m up), refuses a plausible one beyond `max_range_m`
    (depth, else the subject width's range), and is reached when `need`
    inferences in a row each have one within `gate_frac` of the frame of the
    previous pick. Every candidate is judged, not just the top box: the
    Grounder publishes all of them while acquiring (Grounder.begin_acquire).
    Depth is the latest frame and the ground is this tick's altitude and
    pitch, as follow_vlm's gate without identity reads them; the aircraft is
    hovering, so they are the inference's own to within its motion.

    With `identity` (the Grounder built with follow_vlm's identity config,
    --identity) a candidate must also be identity "ok" to count (a SOFT box
    may continue a track, never start one), and the start is decided in the
    world as follow_vlm decides it: `follow_vlm.Reacquirer` with no anchor -
    OK boxes, one object by its map point, within the start range, and SEEN
    MOVING (a car-wide sign by the kerb is none of the physical rules'
    business and all of this one's). The pixel Acquirer still runs, for its
    counters.

    A stand-in without a candidate list ("cands" absent) offers its published
    box as the only candidate. Nothing here reads ground truth."""

    def __init__(self, need: int, query: str, colour_min: float, *,
                 max_range_m: float, width_m: float | None, hfov_deg: float,
                 gate_frac: float = 0.12, identity: bool = False,
                 reacq_max_range_m: float = 45.0, fv=None):
        self.fv = fv or _fv()
        self.need = int(need)
        self.max_range_m = float(max_range_m)
        self.identity = bool(identity)
        self.acq = self.fv.Acquirer(self.need, query, colour_min, gate_frac=gate_frac,
                                    max_range_m=self.max_range_m, width_m=width_m,
                                    hfov_deg=hfov_deg)
        self.acq_w = None
        if self.identity:
            self.acq_w = self.fv.Reacquirer(
                need=self.need, max_range_m=(self.max_range_m if self.max_range_m > 0
                                             else reacq_max_range_m))
            self.acq_w.start(None)
        self.pick = None
        self.pick_yaw = None
        # With identity, the pick's features (P, rng_h, bearing: the
        # Reacquirer's pick_feat) and its inference's capture (pose, times):
        # what the target estimator is seeded from (SubjectTrack.start).
        self.pick_feat = None
        self.pick_cap = None
        self.rejected_by: dict = {}
        # With identity, what each Reacquirer step decided (its last_outcome:
        # "committed", "streak", "short pass", "not moving", ...), and the
        # closest a streak came to its motion bar while refused "not moving",
        # and the last time it was: bookkeeping only, so a timed-out gate
        # says what refused it (Reacquirer.refused never counts the moving
        # test).
        self.identity_outcomes: dict = {}
        # "static" outcomes whose streak was too short for the moving test
        # to be asked (fewer than 2 x need boxes: Reacquirer.moved_m None),
        # marked static only for spanning static_after_s
        self.static_unjudged = 0
        self.closest_not_moving = None
        self.last_not_moving = None

    def _count_rejections(self) -> None:
        for why in self.acq.last_reasons:
            w = str(why)
            key = ("too far" if w.startswith("plausible but") else
                   "identity" if w.startswith("identity:") else
                   self.fv.block_reason_key(w))
            self.rejected_by[key] = self.rejected_by.get(key, 0) + 1

    def judge(self, lat: dict, *, depth=None, ground=None, t: float | None = None) -> bool:
        """One FRESH inference (a Grounder.latest() answer): True once the
        subject is confirmed; `pick` is then its box and `pick_yaw` the
        aircraft's yaw when that frame was captured (for the lock's seed)."""
        cands = lat.get("cands")
        if cands is None:
            cands = [lat["det"]] if lat.get("det") is not None else []
        cands = list(cands)
        ic = lat.get("inf_cap") or {}
        if self.identity:
            tiers = list(lat.get("cand_tiers") or [])
            feats = list(lat.get("cand_feats") or [])
            # The range of each candidate's OWN frame, and its tier: only an
            # identity-OK box may start the mission (follow_vlm, as flown).
            fr = {id(c): (f or {}).get("r") for c, f in zip(cands, feats)}
            self.acq.step(cands, lambda c: fr.get(id(c)), ground=None, tiers=tiers)
            pre_len = len(self.acq_w.trace)
            reached = self.acq_w.step(cands, tiers, feats,
                                      ic.get("t_capture") or ic.get("t_cap")
                                      or (t if t is not None else time.time()))
            self._count_identity_outcome(pre_len)
            pick = self.acq_w.pick
        else:
            fv = self.fv
            reached = self.acq.step(
                cands, (lambda c: fv.range_from_depth(depth, c)) if depth is not None
                else (lambda c: None), ground=ground)
            pick = self.acq.pick
        self._count_rejections()
        if reached:
            self.pick = pick
            self.pick_yaw = (ic.get("pose") or {}).get("yaw")
            if self.identity:
                self.pick_feat = self.acq_w.pick_feat
                self.pick_cap = dict(ic) if ic else None
        return bool(reached)

    def _count_identity_outcome(self, pre_len: int | None = None) -> None:
        """Bookkeeping of the Reacquirer's last step (no decision changes).
        `pre_len`: its trace's length before the step. A "static" step has
        reset the streak, so whether its moving test was asked is told from
        that: a static streak continued the one before (a new streak is 1
        box, under `need`), so it held min(pre_len + 1, trace_max) boxes,
        and moved_m answers only from 2 x need (Reacquirer._halves)."""
        w = self.acq_w
        o = str(w.last_outcome)
        self.identity_outcomes[o] = self.identity_outcomes.get(o, 0) + 1
        if o == "static" and pre_len is not None \
                and min(pre_len + 1, w.trace_max) < 2 * w.need:
            self.static_unjudged += 1
        if o == "not moving":
            moved, bar = w.moved_m(), w.motion_bar_m()
            if moved is not None:
                self.last_not_moving = (float(moved), float(bar), int(w.streak))
                if (self.closest_not_moving is None
                        or bar - moved < self.closest_not_moving[1]
                        - self.closest_not_moving[0]):
                    self.closest_not_moving = self.last_not_moving

    @staticmethod
    def _not_moving_rec(c):
        return (None if c is None else
                {"moved_m": round(c[0], 2), "bar_m": round(c[1], 2), "streak": c[2]})

    def _closest_not_moving_rec(self):
        return self._not_moving_rec(self.closest_not_moving)

    def moving_refusals(self) -> int:
        """How many times the Reacquirer's moving test was asked and refused
        a streak: "not moving", and a "static" whose streak was long enough
        to be judged (2 x need boxes). A static place marked from a shorter
        streak - one that spanned static_after_s before the test could be
        asked - is not counted (static_unjudged)."""
        o = self.identity_outcomes
        return (int(o.get("not moving", 0)) + int(o.get("static", 0))
                - int(self.static_unjudged))

    def identity_need(self) -> tuple[int, bool]:
        """(boxes in a row, seen moving?) the identity gate commits on, read
        off its Reacquirer: with no anchor and a motion bar (motion_m > 0)
        the moving test applies, and it is asked only of a streak of
        2 x need (Reacquirer.moved_m: its trace restarts with the streak),
        so 2 x need identity-OK boxes in a row; without a motion bar,
        `need`."""
        w = self.acq_w
        moving = w.motion_m > 0 and w.reach_m(0.0) is None
        return (2 * w.need if moving else w.need), moving

    def rule(self) -> str:
        """The rule this gate decides by, in words, from its own objects:
        with --identity follow_vlm.Reacquirer's (identity-OK boxes, in a row
        within its range, of one object - each within step_m + v_r dt of the
        last pick, else the streak restarts - seen moving), else the pixel
        Acquirer's (`need` inferences in a row within `max_range_m`)."""
        if self.acq_w is None:
            return f"{self.need} inferences in a row {self._pixel_range()}"
        w = self.acq_w
        n, moving = self.identity_need()
        return (f"{n} identity-ok boxes in a row within {w.max_range_m:g} m, of one object "
                f"(each within {w.step_m:g} m + {w.v_reach():g} m/s x the time since the "
                f"last, else the streak restarts)"
                + (f" (a gap of up to {w.gap_s:g} s with none acceptable does not restart "
                   f"the streak)" if w.gap_s > 0 else "")
                + (" and seen moving" if moving else ""))

    def _pixel_range(self) -> str:
        """The pixel Acquirer's range in words: 0 (or less) is no limit."""
        return (f"within {self.max_range_m:g} m" if self.max_range_m > 0
                else "at any range")

    def start_text(self) -> str:
        """What the hover waits for (the [start] line)."""
        if self.acq_w is None:
            return f"on {self.need} inferences in a row {self._pixel_range()}"
        return f"as {self.rule()}"

    def timeout_text(self, obj: str, wait_s) -> str:
        """Why no mission was flown when the gate timed out (the run's
        end_reason, `no_mission`)."""
        if self.acq_w is None:
            return (f"start gate timed out after {wait_s} s: no plausible {obj!r} "
                    f"{self._pixel_range()} on {self.need} inferences in a row")
        return f"start gate timed out after {wait_s} s: no {obj!r} seen as {self.rule()}"

    def streak_text(self) -> str:
        """The deciding streak and refusals for the [start] report line:
        the identity gate's when it decides, else the pixel Acquirer's. A
        best streak at or over the boxes in a row the identity gate needs is
        not a pass: its moving test is asked then, and what it decided is in
        the step counts (identity_outcomes), and the line says so."""
        if self.acq_w is None:
            return (f"best streak {self.acq.best_streak}, refused "
                    f"{dict(self.rejected_by)}")
        w = self.acq_w
        n, moving = self.identity_need()
        c = self._closest_not_moving_rec()
        last = self._not_moving_rec(self.last_not_moving)
        k = self.moving_refusals()
        why = ""
        if moving and w.best_streak >= n and k:
            why = (f"; it reached {n} in a row and the moving test refused it {k} "
                   f"time{'s' if k != 1 else ''}"
                   + ("" if last is None else
                      f" (last: moved {last['moved_m']:g} m < bar {last['bar_m']:g} m)"))
        elif w.best_streak < n:
            why = f"; no streak reached {n}"
        if self.static_unjudged:
            why += (f"; {self.static_unjudged} shorter streak"
                    f"{'s' if self.static_unjudged != 1 else ''} marked static for spanning "
                    f"{w.static_after_s:g} s before the moving test could be asked")
        return (f"identity gate: best streak {w.best_streak} (it needs {n} in a row"
                + (", then seen moving" if moving else "") + f"), refused {dict(w.refused)}, "
                f"steps {dict(self.identity_outcomes)}"
                + ("" if c is None else
                   f", closest to moving {c['moved_m']:g} m against a bar of {c['bar_m']:g} m")
                + why
                + f" (pixel gate, counters only: best streak {self.acq.best_streak})")

    def record(self) -> dict:
        a = self.acq
        rec = {"needed": self.need, "max_range_m": self.max_range_m,
               "identity": self.identity,
               # The rule that decided, in words, and the object that decided:
               # with identity the Reacquirer (its counts in identity_gate),
               # and the pixel Acquirer's counts below are counters only.
               "rule": self.rule(),
               "decided_by": ("follow_vlm.Reacquirer (identity_gate)" if self.acq_w is not None
                              else "follow_vlm.Acquirer"),
               "inferences_judged": a.n, "inferences_with_plausible": a.n_plausible,
               "candidates_judged": a.n_candidates, "best_streak": a.best_streak,
               "plausible_but_too_far": a.n_too_far,
               "rejected_by": dict(self.rejected_by),
               "last_rejections": list(a.last_reasons[:3]),
               "pick": (None if self.pick is None else
                        {"cx": round(float(self.pick[0]), 1),
                         "cy": round(float(self.pick[1]), 1),
                         "w": round(float(self.pick[2]), 1),
                         "h": round(float(self.pick[3]), 1),
                         "score": round(float(self.pick[4]), 4),
                         "colour": round(float(self.pick[7]), 3),
                         "img_w": int(self.pick[5])})}
        if self.acq_w is not None:
            w = self.acq_w
            n, moving = self.identity_need()
            rec["identity_gate"] = {
                "refused": dict(w.refused), "best_streak": w.best_streak,
                "need_in_a_row": n, "seen_moving": moving,
                "max_range_m": w.max_range_m, "gap_s": w.gap_s,
                # each step's decision (Reacquirer.last_outcome), and the
                # nearest a streak came to the motion bar while refused
                "outcomes": dict(self.identity_outcomes),
                "moving_test_refused": self.moving_refusals(),
                # "static" steps whose streak the moving test never judged
                "static_unjudged": int(self.static_unjudged),
                "closest_not_moving": self._closest_not_moving_rec(),
                "last_not_moving": self._not_moving_rec(self.last_not_moving),
                "static_places": [[round(v, 1) for v in sp[:2]] for sp in w.static],
                "pick_P": (None if w.pick_feat is None or w.pick_feat.get("P") is None
                           else [round(v, 1) for v in w.pick_feat["P"][:2]])}
        return rec


def autopilot_took_over(mode, prev_mode, asks) -> bool:
    """The autopilot left GUIDED - or a hold this node asked for - for a mode
    the node did not ask for: its fence fired RTL, a failsafe, or a pilot in
    Mission Planner. From then on it has the aircraft, and nothing more is
    commanded. One test for the loop and for the hover before it
    (HoverWatch). (Until 2026-10-13 only a change out of GUIDED counted, so a
    pilot taking the aircraft out of the FSM's LOITER went unseen, and the
    throttle override went on holding channel 3.)"""
    return bool(mode != prev_mode and mode != "GUIDED"
                and (prev_mode == "GUIDED" or asks.explains(prev_mode))
                and not asks.explains(mode))


def detector_thread_dead(grounder) -> bool:
    """The Grounder's worker was started and has ended: an OOM or a CUDA
    error in follow_vlm.Grounder._worker kills the thread, and latest()
    then answers its last seq forever (0 if it never ran). A stand-in with
    no thread is never dead."""
    th = getattr(grounder, "_thread", None)
    return bool(th is not None and getattr(th, "ident", None) is not None
                and not th.is_alive())


def _say(msg: str) -> None:
    """A print that cannot raise: the landing net below runs when anything else
    may already have failed, stdout included (a pipe whose reader died, a
    UnicodeEncodeError on a redirected cp950/cp1252 console). Its LAND request
    must never wait behind, or be skipped by, a failing write (re-review of
    baa4603, N1)."""
    try:
        print(msg)
    except Exception:                                             # noqa: BLE001
        pass


def land_on_the_way_out(link, asks, rec, *, crash, stage: str, takeover=None, fsm=None,
                        throttle=None, t: float | None = None) -> dict:
    """THE NET FROM THE ARMING TO THE END OF run() (review 3, 2026-10-22).

    run() asks for LAND in the loop's finally, and a crash in the start
    gate's hover reaches that finally as a no-mission end. Everything else
    between the arming and that finally met only run()'s outer finally,
    which stops the recorder, the detector, the camera and SITL and asks
    nothing of the autopilot: a take-off that stalled ('take-off did not
    reach N m', armed and climbing), the mission_start_check write on a full
    disk, a FollowPilot or SubjectTruth that raised, mission_start's write.
    On SITL the aircraft went with SITL; on HIL or attach it was left in
    GUIDED mid-climb or holding, nothing commanding it (each reproduced
    with tests/test_pas_ardupilot.py's stand-ins).

    run() calls this from that outer finally when the aircraft was armed and
    the loop's finally never made its LAND decision. As there: the throttle
    goes back to the radio first; then LAND is asked unless the autopilot
    or a pilot has the aircraft - a takeover the hover saw, an FSM
    fail-safe, or any mode this node did not ask for (the climb and the
    hover are flown in GUIDED: autopilot_took_over's baseline); then the
    episode's mission_end, naming the crash and what was asked. Nothing
    here raises: run() re-raises the crash itself. Returns the mission_end's
    fields."""
    why = (f"interrupted during {stage}" if isinstance(crash, KeyboardInterrupt) else
           f"crashed during {stage}: {type(crash).__name__}: {crash}" if crash is not None
           else f"left run() during {stage} with no mission end")
    _say(f"[node]   *** {why}")
    try:
        mode = link.snap.mode
    except Exception:                                             # noqa: BLE001
        mode = None
    try:
        owns = bool(takeover is not None
                    or (fsm is not None and (fsm.terminal or fsm.state in FAILSAFE_STATES))
                    or (mode is not None and mode != "GUIDED" and not asks.explains(mode)))
    except Exception as exc:                                      # noqa: BLE001
        # Undecidable: land unless a takeover is on record, as the loop's
        # finally does.
        owns = takeover is not None
        _say(f"[node]   *** who has the aircraft not judged ({type(exc).__name__}: {exc})")
    out = {"stage": stage, "autopilot_mode": mode, "autopilot_owns": owns,
           "land_accepted": None}
    if throttle is not None and getattr(throttle, "active", False):
        try:
            throttle.release(t if t is not None else 0.0, f"episode end ({why})")
        except Exception as exc:                                  # noqa: BLE001
            _say(f"[hold]   override release failed: {exc}")
    if owns:
        _say(f"[node]   the autopilot has the aircraft ({mode}): nothing is asked of it")
    else:
        try:
            asks.ask("LAND")
            out["land_accepted"] = bool(link.land())
        except Exception as exc:                                  # noqa: BLE001
            out["land_accepted"] = False
            _say(f"[link]   LAND failed: {exc}")
        _say("[link]   LAND asked: " + ("accepted" if out["land_accepted"] else
                                         "*** NOT confirmed by the autopilot"))
    try:
        rec.end(why, t=t, **out)
    except Exception as exc:                                      # noqa: BLE001
        _say(f"[node]   *** mission_end not written: {type(exc).__name__}: {exc}")
    return {"why": why, **out}


class HoverWatch:
    """The aircraft while it hovers before the mission clock starts, watched
    as the loop watches it (review 2026-10-22, open item C1's start gate).

    The start gate (wait_for_subject; START_WHEN_SEEN) holds the aircraft in
    ArduCopter's GUIDED hold for up to --start-timeout-s, and until this
    review nothing looked at it: no mode, no rule, no row. A pilot who took
    the aircraft during the wait became the loop's baseline mode (its
    takeover test needs a CHANGE of mode), so the node asked for LAND over
    the pilot - or, had the gate then opened, flew 30 setpoints while the
    pilot held LOITER (both reproduced with tests/test_pas_ardupilot.py's
    FakeLink). The stand-offs and the time-windowed and moving zones that
    FenceBackstop leaves to the Shield alone were neither enforced nor
    measured, and KPIs start at the mission clock: a violation in the hover
    could appear in no figure.

    `observe(t, phase)` pumps the link and reads it, every tick:
      * a state that stays stale LINK_LOST_S ends the hover ("link_lost");
      * the autopilot's mode, by the loop's own test (autopilot_took_over):
        a takeover ends the hover at once ("takeover"); run() then flies no
        mission and asks nothing of the autopilot (no LAND over a pilot), and
        metrics.json autopilot_took_over says when and from what;
      * the pose goes to the detector's frames (obs.put_pose_full), as the
        loop gives it.
    `check(t, st, phase, subject)` asks the Shield about the POSITION
    (state_is_unsafe: would holding here break a rule, over the lookahead),
    with the gate's current candidate given as the subject (set_subject),
    so the stand-off rings bind as they do in the loop; rechecks the held-back
    fence zones (FenceBackstop.recheck); and writes one row (hover_log.jsonl).
    With the Shield on (`act`) the first unsafe tick ends the hover ("unsafe")
    and run() starts the loop at once: the Shield's repair and the escalation
    FSM are this rail's answer to an illegal position, not a hold or a LAND
    inside it. With it off the tick is recorded only, as the off arm records
    the loop.

    What the check reads of the clock: the rule clock (rule_now), which a
    scripted --clock-start holds at its start until the mission starts, and
    the Shield's monitor clock, which starts with the mission's first tick,
    so a moving zone stands at its starting place. The hover's rows are NOT
    flight_log.jsonl rows: the mission's KPIs keep their definition (from the
    mission clock). The hover's own counts are metrics.json
    pre_mission_hover, and an unsafe hover tick makes the run not KPI-grade.

    THE TAKEOVER BASELINE IS GUIDED (review 3, 2026-10-22). Both links'
    bring_up end in GUIDED or raise (PymavlinkLink.set_mode("GUIDED") and
    MavrosLink wait for the mode before arming). The baseline was the mode
    the bring-up LEFT, unchecked: a switch reported in its last pump - the
    end of the climb, where an altitude-fence breach from overshoot fires
    RTL - became the baseline, and neither the hover nor the loop ever saw
    a change (tests/test_pas_ardupilot.py: 30 setpoints flown over the
    pilot's LOITER, then a LAND asked over the pilot). A hover that starts
    in a mode this node did not ask for is a takeover at the bring-up; a
    mode not reported yet is judged on the first tick that reports one.

    The hover's own records (its events, hover_log.jsonl) are best effort
    (review 3): a write that fails - a full disk, a file OneDrive holds
    open under a reused tag - is kept in `io_errors` / `log_error` and the
    hover goes on, so a log write can never decide whether the aircraft is
    landed (run()'s net from the arming on is the second line)."""

    # The mode both bring-ups end in (or raise): the takeover baseline.
    BASELINE_MODE = "GUIDED"

    def __init__(self, link, anchor, obs, shield, asks, *, subject_class=None,
                 act: bool = True, backstop=None, rec=None, rule_now=None,
                 t0: float | None = None) -> None:
        self.link, self.anchor, self.obs = link, anchor, obs
        self.shield, self.asks = shield, asks
        self.subject_class, self.act = subject_class, bool(act)
        self.backstop, self.rec, self.rule_now = backstop, rec, rule_now
        # The mode the bring-up left (recorded), and the last one judged:
        # the loop's takeover test starts from it, not from a read after the
        # hover. The judging starts from GUIDED, never from mode0.
        self.mode0 = link.snap.mode
        self.mode = self.BASELINE_MODE
        self.t_first = self.t_last = self.t_end = None
        self.last_phase = None
        self.ended = None            # "takeover" | "link_lost" | "unsafe"
        self.why = None
        self.takeover = None
        self.first_unsafe = None
        self.stale_since = None
        self.n_ticks = self.n_stale = self.n_checked = self.n_unsafe = self.n_subject = 0
        self.unsafe_rules: dict = {}
        self.phase_s: dict = {}
        self.checked_by_phase: dict = {}
        self.unsafe_by_phase: dict = {}
        self.rows: list = []
        self.io_errors: list = []    # the hover's own record writes that failed
        self.log_written = False
        self.log_error = None
        if self.mode0 is not None:
            # `t0`: this clock's reading as the hover starts (run() turns it
            # into mission time, as it does a hover tick's).
            self._judge_mode(self.mode0, t0, 0.0, "bring-up", self.link.snap)

    def record_event(self, kind: str, **data) -> None:
        """One event on the episode record, best effort (see the class);
        run()'s hover handler writes its own through it too."""
        if self.rec is None:
            return
        try:
            self.rec.event(kind, **data)
        except Exception as exc:                                  # noqa: BLE001
            err = f"{kind}: {type(exc).__name__}: {exc}"
            self.io_errors.append(err)
            print(f"[hover]  *** event not written ({err}); the hover goes on")

    def _judge_mode(self, mode, t, t_hover: float, phase: str, s) -> bool:
        """The loop's takeover test on `mode` against the last mode judged
        (GUIDED at the start). True: the autopilot has the aircraft, and the
        hover is over."""
        prev = self.mode
        if mode == "GUIDED" and prev != "GUIDED":
            self.asks.entered_guided()
        if not autopilot_took_over(mode, prev, self.asks):
            self.mode = mode
            return False
        # "t" (mission time, negative: before the clock) is set by run() once
        # the mission clock is read; t_abs is this clock's reading.
        self.takeover = {"t_abs": t, "t_hover": round(t_hover, 3),
                         "mode": mode, "previous": prev,
                         "fence_breach_count": s.fence_breach_count,
                         "statustext": list(s.statustext[-3:]), "phase": phase}
        if t is None:
            self.takeover.pop("t_abs")
        self.mode = mode
        self._end("takeover", f"the autopilot took the aircraft while it hovered "
                              f"before the mission ({phase}): {prev} -> {mode}")
        self.record_event("autopilot_took_over", mode=mode, previous=prev,
                          phase=phase, t_hover=self.takeover["t_hover"])
        print(f"[hover]  *** the autopilot took the aircraft: {prev} -> {mode} "
              f"({phase}); nothing is commanded or asked of it")
        return True

    def hover_s(self, t: float | None = None) -> float:
        """Seconds since the hover's first tick (to `t`, else to its end)."""
        if self.t_first is None:
            return 0.0
        end = t if t is not None else (self.t_end if self.t_end is not None else self.t_last)
        return max(0.0, end - self.t_first)

    def _end(self, kind: str, why: str) -> None:
        if self.ended is None:
            self.ended, self.why = kind, why

    def observe(self, t: float, phase: str):
        """This hover tick's live State, or None (no live state, or the
        hover ended: `ended` says why)."""
        if self.ended is not None:
            return None
        if self.t_first is None:
            self.t_first = t
        elif self.last_phase is not None:
            self.phase_s[self.last_phase] = (self.phase_s.get(self.last_phase, 0.0)
                                             + (t - self.t_last))
        self.t_last, self.last_phase = t, phase
        self.n_ticks += 1
        link = self.link
        link.pump()
        st = link.state(self.anchor)
        stale, _ = link_staleness(link)
        if st is None or stale is not None:
            self.n_stale += 1
            self.stale_since = t if self.stale_since is None else self.stale_since
            if t - self.stale_since >= LINK_LOST_S:
                self._end("link_lost", f"link lost while hovering before the mission "
                                       f"({phase}): {stale or 'no state'}")
            return None
        self.stale_since = None
        s = link.snap
        # A mode not reported yet is not judged (the baseline stays).
        if s.mode is not None and self._judge_mode(s.mode, t, t - self.t_first, phase, s):
            return None
        if hasattr(self.obs, "put_pose_full"):
            self.obs.put_pose_full(t, st.x, st.y, st.up, math.radians(st.yaw_deg),
                                   s.pitch or 0.0, s.roll or 0.0)
        return st

    def check(self, t: float, st, phase: str, subject=None, det_seq=None) -> list:
        """The Shield on this tick's position (the rule ids it breaks); the
        fence recheck; the row. `subject` (x, y, source) or None."""
        if self.ended is not None:
            return []
        sh = self.shield
        if subject is not None:
            sh.set_subject(subject[0], subject[1], self.subject_class)
            self.n_subject += 1
        else:
            sh.set_subject(None)
        unsafe = sorted({v.rule_id for v in sh.state_is_unsafe(st)})
        self.n_checked += 1
        self.checked_by_phase[phase] = self.checked_by_phase.get(phase, 0) + 1
        th = round(t - self.t_first, 3)
        if unsafe:
            self.n_unsafe += 1
            self.unsafe_by_phase[phase] = self.unsafe_by_phase.get(phase, 0) + 1
            for r in unsafe:
                self.unsafe_rules[r] = self.unsafe_rules.get(r, 0) + 1
            if self.first_unsafe is None:
                self.first_unsafe = {"t_hover": th, "phase": phase, "rules": unsafe,
                                     "x": round(st.x, 2), "y": round(st.y, 2),
                                     "up": round(st.up, 2)}
                self.record_event("hover_unsafe", **self.first_unsafe)
                print(f"[hover]  *** the hover point breaks {unsafe} ({phase}, "
                      f"{th:.1f} s in)"
                      + ("; the mission starts now, so the Shield and the escalation "
                         "FSM act" if self.act else "; recorded (Shield off)"))
        if self.backstop is not None:
            self.backstop.recheck((st.x, st.y), unsafe)
        self.rows.append({
            "t_hover": th, "phase": phase,
            "x": round(st.x, 3), "y": round(st.y, 3), "up": round(st.up, 3),
            "yaw_deg": round(st.yaw_deg, 2), "ap_mode": self.mode, "unsafe": unsafe,
            "subject": (None if subject is None else
                        [round(subject[0], 2), round(subject[1], 2), subject[2]]),
            "det_seq": det_seq,
            "rule_clock": (None if self.rule_now is None
                           else self.rule_now().isoformat(timespec="seconds"))})
        if unsafe and self.act:
            self._end("unsafe", f"the hover before the mission broke {unsafe} ({phase}, "
                                f"{th:.1f} s in): the mission starts now, so the Shield "
                                f"and the escalation FSM act")
        return unsafe

    def close(self, t: float, log_path: Path | None = None) -> None:
        """The hover is over at `t`; its rows go to `log_path` (written, if
        empty, so a reused tag never shows an earlier run's hover). A write
        that fails is `log_error`, not an exception: until review 3
        (2026-10-22) it raised out of run() from the hover's finally, with
        the aircraft holding in GUIDED and no LAND asked (a directory in the
        file's place: PermissionError, errno 13)."""
        if self.t_first is not None and self.t_end is None:
            self.t_end = max(t, self.t_last)
            if self.last_phase is not None:
                self.phase_s[self.last_phase] = (self.phase_s.get(self.last_phase, 0.0)
                                                 + (self.t_end - self.t_last))
        if log_path is not None:
            try:
                Path(log_path).write_text("".join(json.dumps(r) + "\n" for r in self.rows),
                                          encoding="utf-8")
                self.log_written = True
            except Exception as exc:                              # noqa: BLE001
                self.log_error = f"{Path(log_path).name}: {type(exc).__name__}: {exc}"
                print(f"[hover]  *** {self.log_error}; the hover's rows are not on disk "
                      f"(their counts are in metrics.json pre_mission_hover)")

    def summary(self) -> dict:
        checked = self.n_checked > 0
        return {
            "s": round(self.hover_s(), 3),
            "s_by_phase": {k: round(v, 3) for k, v in self.phase_s.items()},
            "ticks": self.n_ticks, "ticks_checked": self.n_checked,
            "ticks_stale": self.n_stale,
            # None, never 0, when no tick was checked (no hover, or no live
            # state on any tick): nothing was measured.
            "unsafe_ticks": self.n_unsafe if checked else None,
            "unsafe_rules": dict(self.unsafe_rules) if checked else None,
            "first_unsafe": self.first_unsafe,
            "subject_ticks": self.n_subject if checked else None,
            # What ended the hover early (None: its wait ended it), and why.
            "outcome": self.ended, "why": self.why,
            "act_on_unsafe": self.act,
            # The mode the bring-up left (the takeover test judged it
            # against GUIDED; a non-GUIDED one is outcome "takeover").
            "mode_at_start": self.mode0,
            # None when the rows were not written (log_error says why).
            "log": "hover_log.jsonl" if self.log_written else None,
            "log_error": self.log_error,
            "io_errors": list(self.io_errors),
            "basis": ("each hover tick before the mission clock: the link's freshness "
                      "and the autopilot's mode (the loop's takeover test), and the "
                      "Shield's state_is_unsafe on the position with the gate's "
                      "candidate as the subject; the rule clock as run() reads it "
                      "(a scripted clock reads its start until the mission starts), "
                      "moving zones at their starting places. Not in flight_log.jsonl "
                      "or any KPI")}


def wait_for_subject(gate: StartGate, grounder, watch: HoverWatch, obs, *, timeout_s: float,
                     clock=time.time, sleep=time.sleep, truth=None, hfov_deg: float = 90.0,
                     width_m: float | None = None, fv=None,
                     truth_period_s: float = GATE_TRUTH_PERIOD_S) -> dict:
    """Hover until `gate` is reached or `timeout_s` passes: the start gate's
    wait (START_WHEN_SEEN). Nothing is commanded - ArduCopter holds where the
    take-off left it in GUIDED - and every TICK is a HoverWatch tick: the
    link pumped and read, the pose given to the detector's frames, the mode
    watched, and the position checked by the Shield with the gate's current
    candidate (its plausible box, placed by subject_estimate from this tick's
    pose and the latest depth: the aircraft is hovering) as the subject.
    `outcome`: "reached", "timed_out", or what ended the hover - "takeover",
    "link_lost" or "detector_dead" (`aborted` says why: no mission), or
    "unsafe" (the mission starts, for the Shield and the FSM). The subject's
    simulator pose (`truth`, EVALUATION only) is read every `truth_period_s`
    into `truth_trace` [t_s, range_m, bearing off the nose deg, + right]; the
    decision never reads it. Returns the gate's record."""
    t0 = clock()
    last_seq = None
    reached = False
    first_ok = None
    aborted = None
    outcome = None
    truth_trace: list = []
    next_truth = 0.0
    # The gate's current candidate, placed: (x, y, source, when).
    cand = None
    while True:
        t = clock()
        if t - t0 >= timeout_s:
            outcome = "timed_out"
            break
        if detector_thread_dead(grounder):
            outcome = "detector_dead"
            aborted = ("the detector thread died while the gate waited: no subject can "
                       "be confirmed")
            break
        st = watch.observe(t, "start gate")
        if watch.ended is not None:
            break
        if st is None:
            sleep(TICK)
            continue
        s = watch.link.snap
        lat = grounder.latest()
        if lat.get("seq") != last_seq:
            last_seq = lat.get("seq")
            # Fresh: the detector ran within the last second (the Grounder's
            # "t"; a stand-in without one is fresh).
            fresh = lat.get("t") is None or (t - float(lat["t"])) < GATE_FRESH_S
            if fresh:
                depth = obs.get_depth() if callable(getattr(obs, "get_depth", None)) else None
                had = gate.acq.n_plausible
                reached = gate.judge(lat, depth=depth, ground=(st.up, s.pitch or 0.0), t=t)
                if gate.acq.n_plausible > had and first_ok is None:
                    first_ok = t - t0
                pick = gate.acq.pick
                e = (None if pick is None or width_m is None else
                     subject_estimate(pick, depth, st, math.radians(st.yaw_deg), hfov_deg,
                                      width_m, fv, body_pitch_rad=s.pitch or 0.0))
                cand = None if e is None else (e[0], e[1], e[2], t)
        subject = cand if cand is not None and t - cand[3] < GATE_FRESH_S else None
        watch.check(t, st, "start gate", subject=subject, det_seq=last_seq)
        if reached:
            outcome = "reached"
            break
        if watch.ended is not None:
            break
        if truth is not None and t - t0 >= next_truth:
            next_truth = t - t0 + truth_period_s
            tr = truth.read()
            if tr.get("xy") is not None:
                dx, dy = tr["xy"][0] - st.x, tr["xy"][1] - st.y
                truth_trace.append([round(t - t0, 1), round(math.hypot(dx, dy), 1),
                                    round(_wrap_deg(math.degrees(math.atan2(dy, dx))
                                                    - st.yaw_deg), 1)])
        sleep(max(0.0, TICK - (clock() - t)))
    if outcome is None:
        outcome = watch.ended
        if outcome != "unsafe":
            aborted = watch.why
    rec = {"enabled": True, **gate.record(), "reached": bool(reached), "outcome": outcome,
           "wait_s": round(clock() - t0, 1), "timeout_s": round(timeout_s, 3),
           "first_candidate_s": None if first_ok is None else round(first_ok, 1),
           "aborted": aborted,
           # [t_s, range_m, bearing off the nose (deg, + right)], every
           # truth_period_s; in view when |bearing| <= hfov / 2.
           "truth_trace": truth_trace, "truth_hfov_deg": hfov_deg}
    return rec


def gate_truth_verdict(gate_rec: dict, truth, state, hfov_deg: float, fv=None) -> dict:
    """Was the gate's pick the subject? Read AFTER the decision, from the
    subject's simulator pose (evaluation only): the true range, whether the
    subject was inside the camera's field of view, and the bearing between
    the picked box and the subject (deg). None fields when there is no pick
    or no pose."""
    out = {"subject_range_m": None, "subject_in_shot": None, "bearing_err_deg": None}
    if truth is None or state is None:
        return out
    tr = truth.read()
    if tr.get("xy") is None:
        out["why"] = tr.get("why")
        return out
    dx, dy = tr["xy"][0] - state.x, tr["xy"][1] - state.y
    off = _wrap_deg(math.degrees(math.atan2(dy, dx)) - state.yaw_deg)
    out["subject_range_m"] = round(math.hypot(dx, dy), 1)
    out["subject_in_shot"] = abs(off) <= hfov_deg / 2.0
    pick = gate_rec.get("pick")
    if pick is not None:
        fv = fv or _fv()
        b = math.degrees(fv.box_bearing(float(pick["cx"]), float(pick["img_w"]), hfov_deg))
        out["bearing_err_deg"] = round(abs(_wrap_deg(off - b)), 1)
    return out


def build_grounder(args, obs, out: Path, fv, hfov_deg: float) -> tuple:
    """The detector this rail flies: `(grounder, record)`.

    follow_vlm.Grounder as before, plus what follow_vlm's city flights give
    it, as far as it applies here (2026-10-21):
      * a TargetLock (--lock-gate; --no-lock-target for none, as before
        2026-10-21): one instance held by position and size, seeded by the
        start gate's pick (Grounder.commit), so a second plausible box does
        not take the follow over;
      * --identity: each candidate judged physically at its own frame's pose
        and depth (demo/identity.py; the street mask next to --citymap), the
        HARD ones dropped and only an OK one published. follow_vlm then also
        makes the lock strict, which only lets go of a subject through its
        estimator and Reacquirer. Without --target-estimator this rail has
        neither, so it holds no lock (a strict lock would never re-adopt the
        car after a 2 s loss) and steers on the best OK box; with it
        (2026-11-15) the lock is follow_vlm's strict one, leaning on
        demo/subject_track.SubjectTrack's estimate (Grounder.set_prior) and
        taken back through its Reacquirer;
      * --negative-query P (repeatable): P asked beside the subject phrase,
        a box that looks more like P is no candidate
        (follow_vlm.negative_suppress)."""
    id_cfg, lock, why_id = None, None, None
    if getattr(args, "identity", False):
        import identity as identity_mod
        th = identity_mod.load_thresholds(getattr(args, "identity_thresholds", None) or None)
        sp = Path(args.citymap).parent / "street.npz"
        sd = identity_mod.StreetDistance.load(sp) if sp.is_file() else None
        if sd is None:
            why_id = f"no street mask at {sp}: the off-street rules cannot fire"
            print(f"[identity] *** {why_id}")
        id_cfg = {"thresholds": th, "street": sd, "hfov": hfov_deg,
                  "hash": identity_mod.thresholds_hash(th),
                  "path": str(getattr(args, "identity_thresholds", None)
                              or identity_mod.DEFAULT_THRESHOLDS),
                  "nouns": {args.object: identity_mod.noun_of(args.object)}}
    strict = id_cfg is not None and bool(getattr(args, "target_estimator", False))
    if strict or (id_cfg is None and not getattr(args, "no_lock_target", False)):
        lock = fv.TargetLock(hfov_deg=hfov_deg, gate_frac=getattr(args, "lock_gate", 0.12))
    negs = [n for n in (getattr(args, "negative_query", None) or []) if str(n).strip()]
    g = fv.Grounder(obs, args.object, thresh=args.det_thresh, colour_min=args.colour_min,
                    log_path=out / "detections.jsonl", lock=lock, identity=id_cfg,
                    negatives=negs)
    rec = {"lock": ({"kind": "TargetLock", "strict": True, "gate_frac": lock.gate_frac,
                     "prior": "the target estimator (--target-estimator: "
                              "demo/subject_track.SubjectTrack)"} if strict else
                    "strict lock not used (identity without an estimator)" if id_cfg
                    else None if lock is None else
                    {"kind": "TargetLock", "gate_frac": lock.gate_frac}),
           "identity": (None if id_cfg is None else
                        {"thresholds_hash": id_cfg["hash"], "thresholds": id_cfg["path"],
                         "street_mask": id_cfg["street"] is not None, "why": why_id,
                         "noun": id_cfg["nouns"][args.object]}),
           "negative_queries": negs}
    return g, rec


def wait_for_detector(grounder, link, *, timeout_s: float = DETECTOR_READY_TIMEOUT_S,
                      clock=time.time, sleep=time.sleep, watch: HoverWatch | None = None
                      ) -> dict:
    """Wait until the Grounder has run once (latest()["seq"] > 0): the mission
    does not start on a detector still loading (DETECTOR_READY_TIMEOUT_S).

    On the ground (run() calls it in the pre-arm step, before GUIDED and
    arming) the link is pumped. Airborne - a link with no pre-arm step - every
    tick is a HoverWatch tick, and a takeover, a lost link or an unsafe
    position ends the wait. A detector thread that died (detector_thread_dead:
    seq would stay 0 for the whole timeout) ends it at once (review
    2026-10-22). `outcome`: "ready", "timed_out", "detector_dead", or what
    ended the hover ("takeover", "link_lost", "unsafe")."""
    t0 = clock()
    seq = (grounder.latest() or {}).get("seq") or 0
    outcome = "ready" if seq > 0 else None
    while outcome is None:
        if detector_thread_dead(grounder):
            outcome = "detector_dead"
            break
        t = clock()
        if t - t0 >= timeout_s:
            outcome = "timed_out"
            break
        if watch is not None:
            st = watch.observe(t, "detector")
            if st is not None:
                watch.check(t, st, "detector")
            if watch.ended is not None:
                outcome = watch.ended
                break
        else:
            link.pump()
        sleep(max(0.0, TICK - (clock() - t)))
        seq = (grounder.latest() or {}).get("seq") or 0
        if seq > 0:
            outcome = "ready"
    why = {"ready": None,
           "timed_out": f"no inference in {timeout_s:g} s: the detector is still loading",
           "detector_dead": ("the detector thread died before its first inference (an "
                             "error in Grounder._worker: an OOM, CUDA)")}.get(
        outcome, None if watch is None else watch.why)
    return {"ready": seq > 0, "outcome": outcome, "why": why,
            "waited_s": round(clock() - t0, 2), "seq": seq,
            "timeout_s": round(timeout_s, 3), "airborne": watch is not None}


def ray_depression_rad(row_px: float, img_w: float, img_h: float, hfov_deg: float,
                       body_pitch_rad: float = 0.0,
                       mount_pitch_deg: float = -20.0) -> float:
    """How far below the horizon the camera ray through image row `row_px`
    points, in radians (negative above it): the mount's pitch, the body's
    (nose up positive, as MAVLink ATTITUDE reports it), and the row's offset
    from the image centre through the pinhole of a square-pixel camera with
    horizontal field `hfov_deg`. The arithmetic follow_vlm.ground_range_m uses
    for a box's bottom edge."""
    f = (img_w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    below = math.atan((row_px - img_h / 2.0) / f)
    return -(math.radians(mount_pitch_deg) + body_pitch_rad) + below


def depth_implausible(det, r: float, up: float, hfov_deg: float, fv=None,
                      body_pitch_rad: float = 0.0) -> str | None:
    """Why a depth range `r` cannot be the detected subject's, or None.

    Until 2026-10-08 a depth return shorter than the camera's height above
    the subject was clamped to a horizontal range of 0: the subject was put
    exactly under the aircraft, where FrontCamera cannot see, the Shield saw
    a P0 stand-off ring of radius 5 m with the aircraft at its centre, and
    five ticks of the red-car follow (pas_ap_20261008_224440) flew a P0
    violation (docs/FINDING-the-escape-that-stayed-inside-the-ring.md). The
    rail records no ground truth of the car; the ticks either side ranged
    it at 3.5-55 m.

    These rules remove the returns they can rule out, not every wrong one: a
    return from something at the subject's own range and bearing passes."""
    if up - r > SUBJECT_TOP_MAX_M:
        return (f"depth {r:.1f} m is nearer than the camera's height above any "
                f"subject ({up:.1f} m up, subjects under {SUBJECT_TOP_MAX_M:g} m), "
                f"whatever the ray")
    fv = fv or _fv()
    mount = getattr(fv, "CAMERA_MOUNT_PITCH_DEG", -20.0)
    if r < up - SUBJECT_UP_M and len(det) >= 7:
        # Pythagoras would put the subject straight under the aircraft. Only
        # a ray that looks steeply enough down can see a subject there: where
        # the box centre's ray meets this range must be no higher than a
        # subject. (Applied to this short-range case only, where a few
        # degrees of pitch error move the point by centimetres.)
        dep = ray_depression_rad(float(det[1]), float(det[5]), float(det[6]), hfov_deg,
                                 body_pitch_rad, mount)
        top = up - r * math.sin(max(0.0, dep))
        if top > SUBJECT_TOP_MAX_M:
            return (f"depth {r:.1f} m along a ray {math.degrees(dep):.0f} deg below "
                    f"the horizon ends {top:.1f} m above the road, above any subject "
                    f"(under {SUBJECT_TOP_MAX_M:g} m); the camera cannot see under "
                    f"the aircraft")
    ground = getattr(fv, "ground_range_m", None)
    if ground is not None and len(det) >= 7:
        try:
            r_g = ground(det, up, body_pitch_rad,
                         mount_pitch_deg=getattr(fv, "CAMERA_MOUNT_PITCH_DEG", -20.0),
                         hfov_deg=hfov_deg)
        except (TypeError, ValueError, ZeroDivisionError):
            r_g = None
        if r_g is not None and r * DEPTH_GROUND_RATIO_MAX < r_g:
            return (f"depth {r:.1f} m is in front of the subject: its box's base "
                    f"ray meets the road at {r_g:.1f} m")
    if len(det) >= 7:
        # The placement Pythagoras will make must be inside the box: the
        # subject's centre at asin(dh / r) below the horizon is no lower than
        # the ray through the box's bottom edge (with slack for pitch error).
        dh = max(0.0, up - SUBJECT_UP_M)
        placed = math.asin(min(1.0, dh / r))
        bottom = ray_depression_rad(float(det[1]) + float(det[3]) / 2.0, float(det[5]),
                                    float(det[6]), hfov_deg, body_pitch_rad, mount)
        if placed > bottom + math.radians(DEPRESSION_SLACK_DEG):
            return (f"depth {r:.1f} m would put the subject's centre "
                    f"{math.degrees(placed):.0f} deg below the horizon, under its "
                    f"box's bottom edge ({math.degrees(bottom):.0f} deg): outside "
                    f"what the camera saw")
    return None


def subject_estimate(det, depth, state: State, yaw_rad: float, hfov_deg: float,
                     width_m: float, fv=None, body_pitch_rad: float = 0.0,
                     range_m: float | None = None, width_src: str = "width"):
    """(x, y, source) of the detected subject in the scene frame, or None.

    Range from the depth image inside the box (follow_vlm.range_from_depth),
    else from apparent width and the subject's real width; bearing from the box
    column. Slant range becomes horizontal range with the height difference.
    No ground truth enters: this is what lets the Shield's stand-off rules bind
    on the real drone too.

    `range_m`, when given, is that depth range already read inside the box
    from the depth frame of the box's own capture (follow_vlm.Grounder's
    cap["range_m"]) and stands in for `depth`; `state`, `yaw_rad` and
    `body_pitch_rad` are then the capture's pose (`estimate_inputs`).
    `width_src` is the source label of a width range that had no depth range
    to replace ("width"; "width:no-capture-depth" when the box's capture had
    no depth frame of its own, so the record tells that apart from a phrase
    that never had depth).

    A depth range the subject cannot have (`depth_implausible`) is dropped
    for the width range, and the source says so ("width:depth-rejected"). A
    range shorter than the height difference survives only where the box
    centre's ray looks down steeply enough to reach a subject there (a body
    pitched well nose-down); it then puts the subject directly below, as the
    geometry says.
    """
    if det is None:
        return None
    fv = fv or _fv()
    b = fv.box_bearing(float(det[0]), float(det[5]), hfov_deg)
    if range_m is not None:
        r, src = float(range_m), "depth"
    elif depth is not None:
        r, src = fv.range_from_depth(depth, det), "depth"
    else:
        r, src = None, None
    if r is not None and math.isfinite(r) and r > 0 and \
            depth_implausible(det, r, state.up, hfov_deg, fv, body_pitch_rad):
        r, src = None, "depth-rejected"
    if r is None:
        rejected = src == "depth-rejected"
        r = fv.implied_range_from_width(det, width_m, hfov_deg)
        src = "width:depth-rejected" if rejected else width_src
    if r is None or not math.isfinite(r) or r <= 0:
        return None
    dh = max(0.0, state.up - SUBJECT_UP_M)
    horiz = math.sqrt(max(0.0, r * r - dh * dh))
    ang = yaw_rad + b
    return (state.x + horiz * math.cos(ang), state.y + horiz * math.sin(ang), src)


def depth_check(det, depth, state: State, hfov_deg: float, fv=None,
                body_pitch_rad: float = 0.0,
                range_m: float | None = None) -> dict | None:
    """The depth range inside the box and why `depth_implausible` refused it
    (`refused` None: accepted), for the flight log; None with no box or no
    depth image (and no capture `range_m`, as subject_estimate takes it).

    Added after the 2026-10-12 re-fly (pas_ap_20261012_033329): 43 depth
    returns were refused and ranged by box width at 56-156 m; within 5 ticks
    of a depth-ranged tick the two ranges disagreed by up to 122 m, and the
    log kept neither the refused range nor the rule that refused it."""
    if det is None or (depth is None and range_m is None):
        return None
    fv = fv or _fv()
    r = float(range_m) if range_m is not None else fv.range_from_depth(depth, det)
    if r is None or not math.isfinite(r) or r <= 0:
        return {"r_m": None, "refused": None}
    return {"r_m": round(float(r), 2),
            "refused": depth_implausible(det, r, state.up, hfov_deg, fv, body_pitch_rad)}


def estimate_inputs(info: dict, state: State, yaw_rad: float, pitch_rad: float,
                    obs, t_now: float) -> tuple:
    """What `subject_estimate` and `depth_check` place the box from:
    (state, yaw_rad, pitch_rad, depth image, capture range, width source
    label, pairing record).

    THE BOX'S OWN CAPTURE, NOT THIS TICK (2026-10-19). Until then the node
    placed a box from a frame up to ~1.1 s old (the detector's ~0.3 s, then
    up to DETECTION_STALE_S held) with the CURRENT tick's position, heading
    and pitch, and read its range from whatever depth frame had arrived LAST:
    on pas_ap_video_on_20261014, pas_ap_close_on and pas_ap_20261012_033329
    the heading had turned a p95 of 17-29 deg (max 42-44 deg) since the
    capture, so at a 20 m range the stand-off ring was drawn 6-14 m to the
    side of the subject, and the old box indexed a newer depth frame on road
    or background. follow_vlm's Grounder pairs every inference with the pose
    interpolated at its capture and that capture's depth frame (2026-09-29)
    and publishes both with the box (latest()["cap"]: "pose", "range_m",
    "depth_src"); this takes them from the SAME latest() answer as the box
    (FollowPilot.step's info["cap"]).

    The pose ring the capture pose is read from is filled by this loop
    (put_pose_full: yaw and pitch in radians, as here). With no capture pose
    (a Grounder started before the ring had a pose, or a stand-in without
    "cap") the old pairing is used and the record says so ("tick" pose,
    "latest" depth), never in silence. The estimate is where the subject was
    at the capture: for a moving subject that lags by the box's age times its
    speed, far less than range times the heading change it replaces.

    A box with no capture POSE can still carry its capture's RANGE (the
    Grounder publishes range_m whether or not the pose ring had a pose). That
    range was read under the box in the frame the box came from, so it is
    used, from this tick's pose, before the newest depth frame - which the
    old box indexes on road or background (review 2026-10-19: until then the
    no-pose case dropped cap["range_m"] for obs.get_depth()). A capture pose
    with no capture depth ranges by width, labelled "width:no-capture-depth"
    in subject_source, not "width" as a phrase that never had depth."""
    cap = info.get("cap") or {}
    pose = cap.get("pose")
    t_c = cap.get("t_capture") or cap.get("t_cap")
    age = None if t_c is None else round(t_now - float(t_c), 3)
    cap_range = cap.get("range_m")
    if pose is not None:
        yaw_c, pitch_c = float(pose["yaw"]), float(pose.get("pitch") or 0.0)
        origin = State(x=float(pose["x"]), y=float(pose["y"]), up=float(pose["up"]),
                       yaw_deg=math.degrees(yaw_c))
        return (origin, yaw_c, pitch_c, None, cap_range,
                "width" if cap_range is not None else "width:no-capture-depth",
                {"pose": f"capture:{pose.get('src', 'ring')}",
                 "depth": cap.get("depth_src") or "none", "box_age_s": age})
    if cap_range is not None:
        return (state, yaw_rad, pitch_rad, None, cap_range, "width",
                {"pose": "tick", "depth": cap.get("depth_src") or "capture",
                 "box_age_s": age})
    depth = obs.get_depth() if hasattr(obs, "get_depth") else None
    return (state, yaw_rad, pitch_rad, depth, None, "width",
            {"pose": "tick", "depth": "latest" if depth is not None else "none",
             "box_age_s": age})


# --------------------------------------------------------------------------- #
# Camera sources
# --------------------------------------------------------------------------- #

def inline_scene(sim_cfg: dict) -> dict:
    """The scene dict with its robot config inlined, the shape
    projectairsim.utils.load_scene_config_as_dict returns and Drone() reads."""
    d = Path(sim_cfg["dir"])
    scene = rail.load_jsonc(d / sim_cfg["scene"])
    for a in scene.get("actors", []):
        if a.get("type") == "robot" and isinstance(a.get("robot-config"), str):
            a["robot-config"] = rail.load_jsonc(d / a["robot-config"])
    return scene


class ProjectAirSimCamera:
    """Loads (or attaches to) the scene, subscribes FrontCamera (scene + depth)
    and the collision topic, and answers ground-truth queries for the RECORD
    only - the Shield and the pilot never read them.

    attach=False  load the rendered scene (World(client, scene, ...)). Desktop
                  only, and only BEFORE ArduPilot starts.
    attach=True   never load. The World is built without its constructor:
                  World.__init__ with no scene still sends an
                  ImportNEDTrajectory request to the default scene id
                  (/Sim/SceneBasicDrone), which does not exist while CityLife
                  is loaded. The node then checks the simulator really serves
                  this scene's robot topics and refuses if it does not, so a
                  node on the Orin cannot attach to the wrong scene in silence.

    chase=True    also subscribe the robot's "Chase" scene camera (the
                  third-person view 16 m behind and 7 m above the airframe,
                  robot_citylife_ardupilot.jsonc) for --save-view. Its
                  messages are only buffered (SemanticObs.put_chase); the
                  recorder thread decodes them through `get_chase_native()`,
                  never the control loop. `get_front_native()` is the
                  FrontCamera frame the detector sees, at its captured size.
    vla_frames    (--pilot vla) a vla_mission.VLAFrames: VLAFront and VLADown
                  (VLA_CAMERAS, AerialVLA's training cameras) are subscribed
                  into it - the callback stores the message, as SemanticObs
                  does - and a robot config without them is refused at
                  start(): a VLA run that cannot see is no VLA run.
    vla_front_source  "vla" (VLAFront is the request's front frame) or
                  "detector" (--vla-front-source detector: the FrontCamera
                  frame the detector sees is ALSO the request's front frame,
                  from the one FrontCamera subscription - a pointer each, as
                  demo/vla_fly.py does; VLAFront is then not subscribed and
                  only VLADown is required, in the robot and among the live
                  topics an attached node checks).
    stress        (--stress) a perception stressor's name (perception_stress):
                  its weather is SET after a load and READ BACK after an
                  attach (the desktop's scene step set it), recorded as
                  scene_record["stress"]; start() refuses on any problem, so
                  a stressed run never flies a scene without its stressor.
                  The exposure and the scene id are in the rendered configs.
    """

    def __init__(self, obs, sim_cfg: dict, address: str = "127.0.0.1",
                 projectairsim=None, view_switch: int = 1,
                 robot: str = rail.ROBOT_NAME, attach: bool = False,
                 chase: bool = False, vla_frames=None, vla_front_source: str = "vla",
                 stress: str | None = None):
        if vla_front_source not in ("vla", "detector"):
            raise ValueError(f"vla_front_source {vla_front_source!r} is not vla or detector")
        if stress is not None:
            pstress.stressor(stress)                          # refuses an unknown name
        self.obs, self.cfg, self.address = obs, sim_cfg, address
        self.pas = projectairsim
        self.view_switch, self.robot, self.attach = view_switch, robot, attach
        self.chase = chase
        self.vla_frames = vla_frames
        self.vla_front_source = vla_front_source
        self.stress = stress
        self.stress_record: dict | None = None
        self._live_topics: list | None = None
        self.client = self.world = self.drone = None
        self.collisions: list[dict] = []
        self.collisions_subscribed = False
        self.have_depth = False
        self.have_chase = False
        self.truth_source = None
        self.scene_record: dict = {"mode": "attach" if attach else "load",
                                   "scene_id": sim_cfg.get("scene_id")}

    def _attach_world(self, pas):
        data = inline_scene(self.cfg)
        world = pas.World.__new__(pas.World)
        world.client = self.client
        world.sim_config_path = self.cfg["dir"]
        world.sim_instance_idx = -1
        world.sim_config = data
        world.parent_topic = f"/Sim/{data['id']}"
        world.home_geo_point = data.get("home-geo-point", {})
        get_info = getattr(self.client, "get_topic_info", None)
        if callable(get_info):
            get_info()
        topics = getattr(self.client, "topics", None)
        prefix = f"{world.parent_topic}/robots/{self.robot}"
        live = None if topics is None else [t for t in topics if str(t).startswith(prefix)]
        self._live_topics = live
        self.scene_record.update(parent_topic=world.parent_topic,
                                 robot_topics_live=None if live is None else len(live))
        if live is not None and not live:
            # A stressed scene has its own id (perception_stress.scene_id_for),
            # so this is also what refuses a stressor the desktop was not given.
            how = ("-Step scene" if self.stress is None
                   else f"-Step scene -Stress {self.stress}")
            raise RuntimeError(
                f"attach: the simulator at {self.address} serves no topic under "
                f"{prefix}, so scene {data['id']!r} is not the one loaded there. "
                f"Load it on the desktop first (run_pas_ardupilot.ps1 {how}), "
                f"start ArduPilot after it (-Step sitl), then start this node")
        return world

    def _stress_weather(self) -> None:
        """--stress: the stressor's weather, set on the scene this node loaded
        or read back from the one it attached to; refused on any problem."""
        fn = pstress.check_weather if self.attach else pstress.apply_weather
        rec = fn(self.world, self.stress)
        self.stress_record = rec
        # The texture (the texture stressor): set here on a scene this node
        # loaded, or the desktop scene step's, which nothing reads back.
        tex = (pstress.texture_attached(self.stress) if self.attach
               else pstress.apply_texture(self.world, self.stress))
        # The level's sun (the shadow stressor turns it; every stressor
        # reads it back, here or on the scene this node attached to).
        sun = (pstress.check_sun if self.attach else pstress.apply_sun)(self.world,
                                                                        self.stress)
        self.scene_record["stress"] = {"name": self.stress,
                                       "scenario_id": pstress.scenario_id(self.stress),
                                       "weather": rec, "texture": tex, "sun": sun}
        if not rec["ok"]:
            fix = (f" (load the scene on the desktop with run_pas_ardupilot.ps1 -Step "
                   f"scene -Stress {self.stress})" if self.attach else "")
            raise RuntimeError(f"--stress {self.stress}: the weather is not the "
                               f"stressor's: {'; '.join(rec['problems'])}{fix}")
        if not tex["ok"]:
            raise RuntimeError(f"--stress {self.stress}: the texture was not set: "
                               + "; ".join(tex["problems"]))
        if not sun["ok"]:
            fix = (f" (load the scene on the desktop with run_pas_ardupilot.ps1 -Step "
                   f"scene -Stress {self.stress}, in a freshly started simulator)"
                   if self.attach else "")
            raise RuntimeError(f"--stress {self.stress}: the sun is not the stressor's: "
                               f"{'; '.join(sun['problems'])}{fix}")
        print(f"[stress] {self.stress}: sun {sun['mode']}, {pstress.SUN_ACTOR} at "
              f"{(sun['read_back'] or {}).get('rpy_deg')} deg (roll, pitch, yaw)")
        if tex["mode"] != "none":
            print(f"[stress] {self.stress}: texture {tex['mode']}"
                  + (f", {tex.get('set')}/{tex.get('asked')} actor(s) of "
                     f"{tex.get('matched')} matching {tex.get('objects_regex')!r}"
                     if tex.get("asked") is not None else ""))
        seen = {k: (rec["read_back"] or {}).get(k) for k in pstress.WEATHER_CHECKED}
        print(f"[stress] {self.stress}: weather {rec['mode']}, read back {seen}")

    def start(self) -> None:
        pas = self.pas or importlib.import_module("projectairsim")
        self.client = pas.ProjectAirSimClient(address=self.address)
        self.client.connect()
        if self.attach:
            self.world = self._attach_world(pas)
        else:
            self.world = pas.World(self.client, self.cfg["scene"], delay_after_load_sec=2,
                                   sim_config_path=self.cfg["dir"])
            self.scene_record["loaded_wall"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
            # The first streaming camera in this robot is the FrontCamera depth
            # stream; one switch puts the Chase view in the window (follow_vlm.py
            # explains the blank-white window this avoids).
            for _ in range(max(0, self.view_switch)):
                try:
                    self.world.switch_streaming_view()
                except Exception as exc:                      # noqa: BLE001
                    print(f"[view] could not switch the main view: {exc}")
                    break
        if self.stress is not None:
            self._stress_weather()
        self.drone = pas.Drone(self.client, self.world, self.robot)
        t0 = time.time()

        def _on_collision(_, m):
            if isinstance(m, dict) and m.get("has_collided", True):
                self.collisions.append({"t": round(time.time() - t0, 2),
                                        "object": m.get("object_name")})
        try:
            self.client.subscribe(self.drone.robot_info["collision_info"], _on_collision)
            self.collisions_subscribed = True
        except Exception as exc:                              # noqa: BLE001
            print(f"[collide] *** not subscribed ({exc}); contacts will NOT be counted")
        fr = self.vla_frames
        detector_front = fr is not None and self.vla_front_source == "detector"
        if fr is not None:
            # The VLA's cameras: VLADown always, VLAFront only when it is the
            # request's front frame (vla_front_source "vla").
            need = ([VLA_CAMERAS["down"]] if detector_front
                    else [VLA_CAMERAS["front"], VLA_CAMERAS["down"]])
            missing = [c for c in need if c not in self.drone.sensors]
            if missing:
                raise RuntimeError(f"--pilot vla: the robot config has no {missing} "
                                   f"(AerialVLA's {VLA_CAMERAS['front']} / "
                                   f"{VLA_CAMERAS['down']}, robot_citylife_ardupilot.jsonc)")
            if self.attach and self._live_topics is not None:
                # Attached, the robot is the record's, not what the desktop
                # loaded: the cameras must be live (rail.select_vla_cameras
                # renders them only for a VLA run or --vla-cameras).
                dead = [c for c in need
                        if not any(f"/sensors/{c}/" in str(t) for t in self._live_topics)]
                if dead:
                    raise RuntimeError(
                        f"--pilot vla: the scene the desktop loaded serves no {dead}: load "
                        f"it with --vla-cameras (run_pas_ardupilot.ps1 -Step scene "
                        f"-VlaCameras) or from a --pilot vla node")
        if detector_front:
            # --vla-front-source detector: the detector's frame is also the
            # request's front frame, from ONE subscription feeding both (a
            # pointer each, nothing decoded in the callback).
            def _front(_, m):
                self.obs.put_front(m)
                fr.put_front(m)
        else:
            def _front(_, m):
                self.obs.put_front(m)
        self.client.subscribe(self.drone.sensors["FrontCamera"]["scene_camera"], _front)
        try:
            self.client.subscribe(self.drone.sensors["FrontCamera"]["depth_camera"],
                                  lambda _, m: self.obs.put_depth(m))
            self.have_depth = True
        except Exception as exc:                              # noqa: BLE001
            print(f"[range] no depth stream ({exc}); range from apparent width")
        if fr is not None:
            if not detector_front:
                self.client.subscribe(self.drone.sensors[VLA_CAMERAS["front"]]["scene_camera"],
                                      lambda _, m: fr.put_front(m))
            self.client.subscribe(self.drone.sensors[VLA_CAMERAS["down"]]["scene_camera"],
                                  lambda _, m: fr.put_down(m))
            self.scene_record["vla_cameras"] = dict(VLA_CAMERAS)
            self.scene_record["vla_front_source"] = self.vla_front_source
            self.scene_record["vla_front_camera"] = ("FrontCamera" if detector_front
                                                     else VLA_CAMERAS["front"])
        if self.chase:
            # As follow_vlm --save-view: the callback stores the message and
            # nothing else, so a 20 Hz camera costs the receive thread a
            # pointer assignment, not a decode.
            try:
                self.client.subscribe(self.drone.sensors["Chase"]["scene_camera"],
                                      lambda _, m: self.obs.put_chase(m))
                self.have_chase = True
            except Exception as exc:                          # noqa: BLE001
                print(f"[view] no Chase camera in this robot config "
                      f"({type(exc).__name__}: {exc}); the first-person view is "
                      f"recorded alone")
            self.scene_record["chase_subscribed"] = self.have_chase

    # The two frames the recorder (demo/recorder.FrameRecorder) asks for, on
    # ITS thread. Each decodes the latest buffered message; None when none
    # has arrived (or the buffer cannot decode), which the recorder counts.
    def get_front_native(self):
        get = getattr(self.obs, "get_front_native", None)
        return get() if callable(get) else None

    def get_chase_native(self):
        get = getattr(self.obs, "get_chase_native", None)
        return get() if callable(get) and self.have_chase else None

    def truth_geo(self) -> dict | None:
        """{lat, lon, alt} of the simulated vehicle, or None."""
        try:
            g = self.drone.get_ground_truth_geo_location()
            self.truth_source = "GetGroundTruthGeoLocation"
            return {"lat": float(g["latitude"]), "lon": float(g["longitude"]),
                    "alt": float(g["altitude"])}
        except Exception:                                     # noqa: BLE001
            return None

    def object_pose(self, name: str) -> tuple[float, float, float | None]:
        """(x North, y East, yaw deg) of a level actor in the simulator's NED
        metres about the scene's home-geo-point (World.get_object_poses ->
        WorldSimApi::getObjectPose -> UnrealHelpers::FindActor, which matches
        an actor's name or TAG; demo/level_actors.py). The yaw is the actor's
        heading from its rotation (NED, clockwise from north, as the EKF's),
        None when the answer has no rotation. NaN when no actor matches; an
        RPC failure raises. For the RECORD only: the subject's truth, and the
        aircraft's own pose for the truth's frame check (AIRCRAFT_ACTOR)."""
        pose = self.world.get_object_poses([name])[0]
        return pose_xy_yaw(pose)

    def object_xy(self, name: str) -> tuple[float, float]:
        """(x North, y East) of a level actor: object_pose without the yaw."""
        x, y, _ = self.object_pose(name)
        return x, y

    def stop(self) -> None:
        try:
            if self.client is not None:
                self.client.disconnect()
        except Exception:                                     # noqa: BLE001
            pass


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #

def load_obstacle_map(policy, citymap: Path):
    """The CityLife occupancy map for the policy's altitude band, as follow_vlm
    loads it; None (ObstacleClearance INERT, said out loud) when unavailable."""
    if not policy.by_type(ObstacleClearance):
        return None, "policy has no obstacle_clearance rule"
    if not Path(citymap).is_file():
        return None, f"no map at {citymap}: obstacle_clearance is INERT this flight"
    try:
        import city_planner
        import occ_bands
    except Exception as exc:                                  # noqa: BLE001
        return None, f"map loaders unavailable ({exc}): obstacle_clearance is INERT"
    cmap = city_planner.load_occ(str(citymap))
    alt = policy.by_type(AltitudeEnvelope)
    if alt:
        sel = occ_bands.select_for_band(Path(citymap).parent, alt[0].alt_min_m,
                                        alt[0].alt_max_m)
        if sel["map"] is None:
            return None, "no map covers the policy's altitude band: obstacle_clearance is INERT"
        cmap = sel["map"]
    return ({"occ": cmap["occ"], "res": cmap["res"], "ox": cmap["ox"], "oy": cmap["oy"]},
            f"loaded {citymap}")


G1_GATE_NAME = "G1 (fallback-gates.md)"


def g1_status(path: str | None) -> dict:
    """The rail's G1 qualification, read from a verdict g1_check.summarize
    wrote. Anything else - no file, an unreadable one, a JSON that is not a
    G1 verdict - is None (unknown), never a pass."""
    if not path:
        return {"passed": None, "verdict": None,
                "why": "no G1 verdict was given (--g1-verdict)"}
    try:
        v = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        return {"passed": None, "verdict": str(path), "error": str(exc)}
    if not isinstance(v, dict) or v.get("gate") != G1_GATE_NAME \
            or not isinstance(v.get("g1_passed"), bool):
        return {"passed": None, "verdict": str(path),
                "error": "not a G1 verdict written by g1_check.summarize"}
    return {"passed": v["g1_passed"], "status": v.get("status"), "verdict": str(path),
            "tag": v.get("tag"), "expected_runs": v.get("expected_runs")}


def rt_factor_ok(rt: float | None) -> bool:
    return rt is not None and RT_FACTOR_BAND[0] <= rt <= RT_FACTOR_BAND[1]


def autopilot_fence_record(link, policy, cruise_alt_m: float) -> dict | None:
    """The autopilot's GeoFence as it holds it, judged by the rule both SITL
    rails apply (sitl/mavlink_adapter_node.fence_alt_plan): its altitude
    fence must sit above every altitude the policy allows, or the backstop
    fires on a flight the Shield considers legal. This node never changes an
    autopilot PARAMETER (it uploads the policy's zones: FenceBackstop), so
    whether the zones are enforced at all - FENCE_TYPE's polygon bit (4),
    which covers the uploaded exclusion circles too - is recorded beside
    them. None when the link cannot read parameters; a value the autopilot
    did not answer stays None, so an unread fence is never reported as off
    or as fine."""
    if not hasattr(link, "autopilot_fence"):
        return None
    vals = link.autopilot_fence()
    ceiling, ceiling_src = policy_ceiling_m(policy, cruise_alt_m)
    alt_max, enabled = vals.get("FENCE_ALT_MAX"), vals.get("FENCE_ENABLE")
    ftype = vals.get("FENCE_TYPE")
    plan = fence_alt_plan(ceiling, alt_max)
    return {**vals, "policy_ceiling_m": ceiling, "policy_ceiling_source": ceiling_src,
            "backstop_active": None if enabled is None else enabled >= 1.0,
            "stricter_than_policy": (None if alt_max is None or ceiling is None
                                     else not plan["ok"]),
            "zones_enforced": (None if ftype is None
                               else bool(int(ftype) & 4)),
            "why": plan["why"], "changed_by_node": False}


def module_available(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def percentile(vals, q: float):
    return None if not vals else round(float(np.percentile(np.asarray(vals), q)), 3)


def spread(vals) -> dict:
    """{n, median, p95, max} of a list of metres; None for each when empty."""
    return {"n": len(vals), "median": percentile(vals, 50), "p95": percentile(vals, 95),
            "max": None if not vals else round(max(vals), 3)}


# --------------------------------------------------------------------------- #
# Policy, mission, frames
# --------------------------------------------------------------------------- #

def resolve_policy(args, default_bundle=None) -> tuple[str | None, str | None, dict]:
    """`(bundle, yaml, record)` for load_for_flight, as both SITL rails
    choose (guardrail.manifest.flight_policy_choice): --bundle; --policy only
    with --dev-yaml; neither, the signed bundle of DEFAULT_POLICY when it is
    in bundles/ (`default_bundle` overrides where to look). ValueError, before
    anything starts, for a YAML policy without --dev-yaml and for no policy
    when the default bundle is not on disk."""
    return flight_policy_choice(
        args.bundle, args.policy, dev_yaml=getattr(args, "dev_yaml", False),
        default_yaml=DEFAULT_POLICY,
        default_bundle=(flight_bundle_path(DEFAULT_POLICY) if default_bundle is None
                        else default_bundle))


def _field(obj, key):
    return obj[key] if isinstance(obj, dict) else getattr(obj, key)


def pose_xy_yaw(pose) -> tuple[float, float, float | None]:
    """(x, y, yaw deg) of a Project AirSim Pose ({"translation": {x, y, z},
    "rotation": {w, x, y, z}}, a dict or its attribute form). The yaw is
    demo/semantic_demo.quat_yaw's - the heading follow_vlm reads off the
    kinematics' orientation - in degrees; None without a rotation."""
    t = _field(pose, "translation")
    x, y = float(_field(t, "x")), float(_field(t, "y"))
    try:
        q = _field(pose, "rotation")
        w, qx, qy, qz = (float(_field(q, k)) for k in ("w", "x", "y", "z"))
    except (KeyError, AttributeError, TypeError, ValueError):
        return x, y, None
    yaw = math.degrees(math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
    return x, y, (yaw if math.isfinite(yaw) else None)


def scene_to_anchor(x: float, y: float, scene_home, anchor) -> tuple[float, float] | None:
    """(x North, y East) in the simulator's NED metres about the scene's
    home-geo-point, in the anchor's frame (the Shield's). The same point when
    the anchor IS the scene's home; else through lat/lon with the projection
    every frame here uses (guardrail/projection.py). None without a home."""
    if scene_home is None:
        return None
    lat0, lon0 = float(scene_home[0]), float(scene_home[1])
    if abs(lat0 - anchor.lat) < 1e-12 and abs(lon0 - anchor.lon) < 1e-12:
        return float(x), float(y)
    return anchor.to_scene(*LocalProjection(lat0, lon0).to_latlon(float(x), float(y)))


def scene_spawn_xy(scene: dict | None, anchor, robot: str = rail.ROBOT_NAME
                   ) -> tuple[tuple[float, float], str]:
    """Where the aircraft is armed, in the anchor's frame, and how that is
    known: the scene's robot origin (the vehicle spawns there and takes off
    vertically), or - with no scene and no policy origin, when the anchor IS
    the EKF's first fix - (0, 0)."""
    if anchor is None:
        return (0.0, 0.0), ("the frame's origin: no policy origin and no scene, so "
                            "the EKF's first fix (the arming point) anchors it")
    if scene:
        for a in scene.get("actors", []):
            if a.get("type") == "robot" and a.get("name") == robot:
                xyz = str((a.get("origin") or {}).get("xyz", "")).split()
                if len(xyz) == 3:
                    xy = scene_to_anchor(float(xyz[0]), float(xyz[1]),
                                         rail.home_geo_point(scene), anchor)
                    return ((round(xy[0], 3), round(xy[1], 3)),
                            f"the scene's {robot} origin (where it spawns), in the "
                            f"{anchor.source} frame")
    return (0.0, 0.0), (f"unknown before arming (anchor {anchor.source}, no scene "
                        f"spawn point): (0, 0) assumed")


def follow_mission(object_text: str, start_xy, cruise_alt_m: float,
                   speed_pref_mps: float) -> Mission:
    """The mission context a follow flight's CSPs are compiled for (the
    Prefix Compiler's "pose, home, target"). A follow has no fixed target:
    where the subject will be is not known before the flight, and ground
    truth may not stand in for it, so the target proxy is the start itself -
    the CSP then ranks zones by their distance from where the follow begins.
    The speed preference is the pilot's own cap (--speed-max)."""
    sx, sy = float(start_xy[0]), float(start_xy[1])
    return Mission(task_text=f"follow {object_text}", target_x=sx, target_y=sy,
                   cruise_alt_m=float(cruise_alt_m),
                   speed_pref_mps=float(speed_pref_mps), start_x=sx, start_y=sy)


def pilot_mission(args, start_xy) -> tuple[Mission, str]:
    """(the mission each generation's CSP is compiled for, where its target
    came from). The servo pilot's follow (follow_mission); the VLA's GOTO has
    a target, the commanded goal itself; its approach and its follow have a
    follow's context (the subject's position is not known before the flight),
    each worded as what it is. The speed preference is the pilot's own cap."""
    if getattr(args, "pilot", "servo") == "vla":
        sx, sy = float(start_xy[0]), float(start_xy[1])
        if args.mission == "goto":
            gx, gy = args.goal
            return (Mission(task_text=f"fly to the goal at ({gx:g}, {gy:g})", target_x=gx,
                            target_y=gy, cruise_alt_m=float(args.cruise_alt),
                            speed_pref_mps=float(args.vla_cruise_mps), start_x=sx, start_y=sy),
                    "the commanded goal (--goal)")
        verb = "follow" if args.mission == "follow" else "approach"
        return (Mission(task_text=f"{verb} {args.object}", target_x=sx, target_y=sy,
                        cruise_alt_m=float(args.cruise_alt),
                        speed_pref_mps=float(args.vla_cruise_mps), start_x=sx, start_y=sy),
                f"the start (proxy): {'a' if verb == 'follow' else 'an'} {verb}'s subject "
                f"position is not known before the flight")
    return (follow_mission(args.object, start_xy, args.cruise_alt, args.speed_max),
            "the start (proxy): a follow mission's subject position is not known before "
            "the flight")


# --------------------------------------------------------------------------- #
# Subject ground truth (evaluation only)
# --------------------------------------------------------------------------- #

def subject_actor_for(arg: str | None, object_text: str, subject_class: str | None
                      ) -> tuple[str | None, str | None]:
    """`(actor tag, why not)` for the subject truth reader. --subject-actor
    TAG names it; 'auto' knows one: the CityLife red car, Car_10, for a red
    car; 'none' reads nothing."""
    a = (arg or "auto").strip()
    if a.lower() == "none":
        return None, "--subject-actor none: the subject's pose is not read"
    if a.lower() != "auto":
        return a, None
    if subject_class == "car" and "red" in str(object_text).lower():
        return RED_CAR_ACTOR, None
    return None, (f"no level actor is known for {object_text!r}; name its tag with "
                  f"--subject-actor")


class OffLoopRpc:
    """The simulator's evaluation RPCs, off the control loop (SIM_RPC_WAIT_S).

    One worker thread makes every call it is handed, one at a time - Project
    AirSim's service socket is one Req0 socket, so the calls stay serial, and
    nothing else uses it during the flight (the camera streams arrive on the
    topic socket). `call(fn, *a)` hands one over and waits at most `wait_s`:

      ("ok", result) | ("error", exception)   answered in time
      ("late", None)   no answer within wait_s: the caller goes on, and the
                       answer, when it comes, is dropped (its duration is
                       kept in `late_ms`)
      ("busy", None)   a late call is still running: nothing is sent
      ("spent", None)  the tick's budget is used up: nothing is sent

    Between `begin_tick()` and `end_tick()` the calls share ONE wait_s
    (review 2026-10-22): each waits only what is left of it, so a tick's
    truth read and frame check together hold the loop at most wait_s, not
    wait_s each. Outside a tick each call has its own wait_s.

    Without it the RPC sat on the loop: one 1024.9 ms GetObjectPoses on
    pas_ap_ovf_on_20261021 was that flight's 1028 ms tick."""

    def __init__(self, wait_s: float = SIM_RPC_WAIT_S, timer=time.perf_counter):
        self.wait_s = float(wait_s)
        self.timer = timer
        self._cv = threading.Condition()
        self._job = None              # (fn, args, box) handed over, not yet taken
        self._running = None          # the box of the call being made
        self._stop = False
        self._thread = None
        self._deadline = None         # the open tick's budget ends (timer), or None
        self.n_ticks = self.n_spent = 0
        self.n_calls = self.n_late = self.n_busy = self.n_errors = 0
        self.call_ms: list[float] = []    # every call's own duration
        self.late_ms: list[float] = []    # ...of the calls answered after their wait

    def _ensure(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, daemon=True,
                                            name="sim-rpc")
            self._thread.start()

    def _run(self) -> None:
        while True:
            with self._cv:
                while self._job is None and not self._stop:
                    self._cv.wait()
                if self._stop and self._job is None:
                    return
                fn, args, box = self._job
                self._job, self._running = None, box
            t0 = self.timer()
            try:
                val, status = fn(*args), "ok"
            except Exception as exc:                              # noqa: BLE001
                val, status = exc, "error"
            ms = (self.timer() - t0) * 1000.0
            with self._cv:
                box.update(status=status, value=val, ms=ms)
                self.call_ms.append(ms)
                if box.get("abandoned"):
                    self.late_ms.append(ms)
                self._running = None
                box["done"].set()

    def begin_tick(self) -> None:
        """Open one tick's budget: wait_s from now, shared by the calls
        until end_tick()."""
        with self._cv:
            self._deadline = self.timer() + self.wait_s
            self.n_ticks += 1

    def end_tick(self) -> None:
        with self._cv:
            self._deadline = None

    def call(self, fn, *args):
        with self._cv:
            if self._running is not None or self._job is not None:
                self.n_busy += 1
                return "busy", None
            wait = self.wait_s
            if self._deadline is not None:
                wait = min(wait, self._deadline - self.timer())
                if wait <= 0.0:
                    self.n_spent += 1
                    return "spent", None
            box = {"done": threading.Event()}
            self._job = (fn, args, box)
            self.n_calls += 1
            self._cv.notify()
        self._ensure()
        if not box["done"].wait(wait):
            with self._cv:
                if not box["done"].is_set():
                    box["abandoned"] = True
                    self.n_late += 1
                    return "late", None
        if box["status"] == "error":
            self.n_errors += 1
        return box["status"], box["value"]

    def close(self, timeout_s: float = 2.0) -> None:
        """Let a running call finish (at most `timeout_s`), then end the thread."""
        with self._cv:
            self._stop = True
            self._cv.notify()
        if self._thread is not None:
            self._thread.join(timeout_s)

    def summary(self) -> dict:
        return {"wait_ms": round(self.wait_s * 1000.0, 1), "calls": self.n_calls,
                "late": self.n_late, "busy": self.n_busy, "errors": self.n_errors,
                "ticks": self.n_ticks, "spent": self.n_spent,
                "call_ms": spread([round(v, 3) for v in self.call_ms]),
                "late_ms": spread([round(v, 3) for v in self.late_ms]),
                "basis": ("each RPC's own duration on the worker thread; a tick's "
                          "calls share one wait_ms (begin_tick), a call answered "
                          "after its share is dropped (late), the ticks until it "
                          "answers send none (busy), and a call the tick has no "
                          "time left for is not sent (spent)")}


def _direct_call(fn, *args):
    """OffLoopRpc.call's answer, made on the caller's thread (no worker)."""
    try:
        return "ok", fn(*args)
    except Exception as exc:                                      # noqa: BLE001
        return "error", exc


class SubjectTruth:
    """The followed subject's simulator pose, read once a tick for the RECORD:
    `read()` -> {"xy": (x, y) in the Shield's frame, or None, "why": None or
    the reason}. Nothing the Shield or the pilot sees comes from here.

    The read is a synchronous RPC on the simulator's service socket (shared
    with truth_geo: one Req0 socket, one request at a time), made after the
    tick's setpoint went out. Given an `rpc` (OffLoopRpc; run() always gives
    one since 2026-10-21) every RPC here goes through its one worker thread
    and the caller waits at most SIM_RPC_WAIT_S - in the loop, per tick, shared
    with that tick's frame or geo check (OffLoopRpc.begin_tick; review
    2026-10-22) - so a late answer is a tick with no truth, said so ("late" /
    "busy" / "spent"), never a stalled loop. Without one the call is made in
    line, as before. Its latency is recorded
    (`summary()["read_ms"]`, the RPCs answered in time). After
    TRUTH_MAX_ERRORS failed reads in a row it backs off and retries, never
    stops; a late or busy read is not a failure (the actor is not missing).

    `check_frame(state)` is the reader's own frame check: the AIRCRAFT read
    through the subject's own RPC (World.get_object_poses on AIRCRAFT_ACTOR)
    and the subject's own projection (scene_to_anchor), against the EKF
    position the Shield flies on; `note_ekf_vs_sim(m)` takes the loop's other
    check, the EKF against the simulator's kinematics (ekf_vs_sim_horiz_m).
    `frame_summary()` judges both (TRUTH_FRAME_MAX_M). Without them, a truth
    in another frame and a detector on another object look the same in the
    record (pas_ap_close_on/off, 2026-10-13: truth 119-235 m north-east,
    estimates south-east; the frames turned out to agree, see truth_view).

    What the check cannot see: an offset of ONE actor's pose (a pivot away
    from the mesh), since only the aircraft's own actor is read beside the
    subject's. That Car_10's actor sits where the car is seen rests on
    follow_vlm, whose tracker followed it to a median 1.89 m
    (demo/out/citylife_redcar_nfz_20261013 metrics.estimate_on_subject)."""

    def __init__(self, camera, actor: str | None, why: str | None, scene_home, anchor,
                 timer=time.perf_counter, aircraft_actor: str = AIRCRAFT_ACTOR,
                 rpc: "OffLoopRpc | None" = None):
        self.camera, self.actor, self.scene_home, self.anchor = camera, actor, scene_home, anchor
        self.timer = timer
        self.aircraft_actor = aircraft_actor
        self.rpc = rpc
        # Reads (and frame checks) with no answer within the wait, those not
        # sent because the late one was still running, and those not sent
        # because the tick's RPC budget was used up (OffLoopRpc).
        self.late = self.busy = self.spent = 0
        self.frame_late = self.frame_busy = self.frame_spent = 0
        self.why_off = why
        if why is None and not hasattr(camera, "object_xy"):
            self.why_off = ("this camera source reads no simulator poses (no subject "
                            "ground truth without Project AirSim)")
        if why is None and scene_home is None:
            self.why_off = "no scene home-geo-point, so a simulator pose has no frame"
        self.errors = 0                       # consecutive failed reads
        self.wait = 0                         # ticks left to skip before a retry
        self.backoff = TRUTH_RETRY_TICKS
        self.last_why: str | None = None
        self.attempts = self.failures = self.backoffs = 0
        self.read_ms: list[float] = []
        # The frame check (check_frame): metres between the two views of the
        # aircraft and the heading between them, one per check; the checks
        # that could not be made, and their own back-off (in checks, each
        # TRUTH_FRAME_PERIOD_S apart), so an aircraft the simulator cannot
        # find is not asked for every 0.5 s of the flight.
        self.frame_err: list[float] = []
        self.frame_yaw_err: list[float] = []
        self.frame_ms: list[float] = []
        self.frame_attempts = self.frame_failures = self.frame_backoffs = 0
        self.frame_errors = 0                 # consecutive failed checks
        self.frame_wait = 0                   # checks left to skip before a retry
        self.frame_backoff = TRUTH_RETRY_TICKS
        self.frame_last_why: str | None = None
        # The loop's EKF-against-kinematics check (note_ekf_vs_sim).
        self.ekf_err: list[float] = []

    @property
    def source(self) -> str | None:
        return (None if self.actor is None or self.why_off else
                f"World.get_object_poses([{self.actor!r}]) (Project AirSim)")

    def read(self) -> dict:
        if self.why_off:
            return {"xy": None, "why": self.why_off}
        if self.wait > 0:
            self.wait -= 1
            return {"xy": None, "why": (f"backing off after {self.errors} failed reads in "
                                        f"a row (retry in {self.wait + 1} tick(s)); "
                                        f"last: {self.last_why}")}
        t0 = self.timer()
        status, val = (self.rpc.call if self.rpc is not None else _direct_call)(
            self.camera.object_xy, self.actor)
        if status == "busy":
            self.busy += 1
            return {"xy": None, "why": ("the previous pose read is still running "
                                        "off the loop (OffLoopRpc): none sent")}
        if status == "spent":
            self.spent += 1
            return {"xy": None, "why": ("the tick's simulator RPC budget was used up "
                                        "(OffLoopRpc.begin_tick): none sent")}
        self.attempts += 1
        if status == "late":
            self.late += 1
            return {"xy": None, "why": (f"no answer within the tick's "
                                        f"{self.rpc.wait_s * 1000:.0f} ms RPC budget "
                                        f"(the read goes on off the loop; dropped)")}
        try:
            if status == "error":
                raise val
            x, y = val
            why = (None if math.isfinite(x) and math.isfinite(y) else
                   f"the simulator found no actor tagged {self.actor!r} "
                   f"(GetObjectPoses answered NaN)")
        except Exception as exc:                              # noqa: BLE001
            why = f"GetObjectPoses failed: {type(exc).__name__}: {exc}"
        self.read_ms.append((self.timer() - t0) * 1000.0)
        if why is None:
            self.errors, self.backoff = 0, TRUTH_RETRY_TICKS
            return {"xy": scene_to_anchor(x, y, self.scene_home, self.anchor), "why": None}
        self.errors += 1
        self.failures += 1
        self.last_why = why
        if self.errors >= TRUTH_MAX_ERRORS:
            self.wait, self.backoffs = self.backoff, self.backoffs + 1
            self.backoff = min(2 * self.backoff, TRUTH_RETRY_MAX_TICKS)
        return {"xy": None, "why": why}

    def summary(self) -> dict:
        return {"attempts": self.attempts, "failures": self.failures,
                "backoffs": self.backoffs,
                # Reads not answered within the wait, ticks none was sent on
                # because one was still running, and ticks whose RPC budget
                # was used up (OffLoopRpc; 0 in line).
                "late": self.late, "busy": self.busy, "spent": self.spent,
                "off_loop": self.rpc is not None,
                "read_ms": spread([round(v, 3) for v in self.read_ms])}

    def frame_off(self) -> str | None:
        """Why the frame cannot be checked through get_object_poses, or None."""
        if self.why_off:
            return self.why_off
        if not callable(getattr(self.camera, "object_xy", None)):
            return ("this camera source reads no simulator poses, so the truth's "
                    "frame is not checked")
        return None

    def _aircraft_pose(self) -> tuple[float, float, float | None]:
        get = getattr(self.camera, "object_pose", None)
        if callable(get):
            return get(self.aircraft_actor)
        x, y = self.camera.object_xy(self.aircraft_actor)
        return x, y, None

    def check_frame(self, state) -> dict | None:
        """One frame check: where the subject's RPC and projection put the
        aircraft, against where the Shield's EKF puts it. {"sim_xy": (x, y)
        in the Shield's frame, "err_m": metres, "yaw_err_deg": the actor's
        heading minus the EKF's, or None} - or None (not checkable, backing
        off, or the read failed: counted, and after TRUTH_MAX_ERRORS failures
        in a row the next TRUTH_RETRY_TICKS checks are skipped, doubling to
        TRUTH_RETRY_MAX_TICKS). One RPC; the loop calls it after the tick's
        setpoint went out, like read()."""
        if self.frame_off():
            return None
        if self.frame_wait > 0:
            self.frame_wait -= 1
            return None
        t0 = self.timer()
        status, val = (self.rpc.call if self.rpc is not None else _direct_call)(
            self._aircraft_pose)
        if status == "busy":
            self.frame_busy += 1
            return None
        if status == "spent":
            self.frame_spent += 1
            return None
        self.frame_attempts += 1
        if status == "late":
            self.frame_late += 1
            return None
        why = None
        try:
            if status == "error":
                raise val
            vx, vy, vyaw = val
            if not (math.isfinite(vx) and math.isfinite(vy)):
                why = (f"the simulator found no actor matching {self.aircraft_actor!r} "
                       f"(GetObjectPoses answered NaN)")
        except Exception as exc:                              # noqa: BLE001
            why = (f"GetObjectPoses({self.aircraft_actor!r}) failed: "
                   f"{type(exc).__name__}: {exc}")
        self.frame_ms.append((self.timer() - t0) * 1000.0)
        if why is not None:
            self.frame_errors += 1
            self.frame_failures += 1
            self.frame_last_why = why
            if self.frame_errors >= TRUTH_MAX_ERRORS:
                self.frame_wait = self.frame_backoff
                self.frame_backoffs += 1
                self.frame_backoff = min(2 * self.frame_backoff, TRUTH_RETRY_MAX_TICKS)
            return None
        self.frame_errors, self.frame_backoff = 0, TRUTH_RETRY_TICKS
        sx, sy = scene_to_anchor(vx, vy, self.scene_home, self.anchor)
        err = math.hypot(sx - state.x, sy - state.y)
        self.frame_err.append(err)
        yaw_err = None
        if vyaw is not None and getattr(state, "yaw_deg", None) is not None:
            yaw_err = _wrap_deg(vyaw - float(state.yaw_deg))
            self.frame_yaw_err.append(abs(yaw_err))
        return {"sim_xy": (sx, sy), "err_m": err, "yaw_err_deg": yaw_err}

    def note_ekf_vs_sim(self, err_m: float) -> None:
        """The loop's EKF-against-the-simulator's-kinematics check, in metres
        (the figure metrics.json ekf_vs_sim_horiz_m spreads)."""
        self.ekf_err.append(float(err_m))

    def frame_verdict(self) -> tuple[bool | None, str | None]:
        """`(agrees, cause)`. agrees: True only from the check through
        get_object_poses; False when it disagrees, or - without it - when the
        EKF disagrees with the simulator's kinematics; None otherwise. cause:
        "frame" (the object poses sit in another frame than the EKF's while
        the EKF agrees with the kinematics) or "ekf" (the EKF is off the
        simulator, so a range from it is off whatever the truth's frame)."""
        ekf_bad = bool(self.ekf_err) and float(np.median(self.ekf_err)) > TRUTH_FRAME_MAX_M
        if self.frame_err:
            if float(np.median(self.frame_err)) <= TRUTH_FRAME_MAX_M:
                return True, None
            return False, ("ekf" if ekf_bad else "frame")
        return (False, "ekf") if ekf_bad else (None, None)

    def frame_disagrees(self) -> bool:
        """True once the checks so far put the truth's ranges in doubt
        (frame_verdict False): the HUD then shows no true range."""
        return self.frame_verdict()[0] is False

    def heading_agrees(self) -> bool | None:
        """The actor's heading against the EKF's (median of |difference|
        within TRUTH_FRAME_MAX_YAW_DEG); None when no answer carried one."""
        if not self.frame_yaw_err:
            return None
        return bool(float(np.median(self.frame_yaw_err)) <= TRUTH_FRAME_MAX_YAW_DEG)

    def frame_summary(self) -> dict:
        """metrics.json `subject_truth.frame_check`. `checked`: the check
        through get_object_poses ran; `partial`: it did not, and only the
        EKF against the kinematics was measured. `agrees` is None when
        nothing proves it, never True by default (frame_verdict)."""
        agrees, cause = self.frame_verdict()
        rec = {"max_m": TRUTH_FRAME_MAX_M, "period_s": TRUTH_FRAME_PERIOD_S,
               "aircraft_actor": self.aircraft_actor,
               "path": (f"the aircraft through the subject's own RPC "
                        f"(World.get_object_poses([{self.aircraft_actor!r}])) and "
                        f"projection (scene_to_anchor), against the EKF position the "
                        f"Shield flies on"),
               "attempts": self.frame_attempts, "failures": self.frame_failures,
               "backoffs": self.frame_backoffs, "last_failure": self.frame_last_why,
               "late": self.frame_late, "busy": self.frame_busy, "spent": self.frame_spent,
               "check_ms": spread([round(v, 3) for v in self.frame_ms]),
               "ekf_vs_sim_m": spread([round(v, 3) for v in self.ekf_err]),
               "yaw_err_deg": spread([round(v, 2) for v in self.frame_yaw_err]),
               "max_yaw_deg": TRUTH_FRAME_MAX_YAW_DEG,
               "heading_agrees": self.heading_agrees(),
               "agrees": agrees, "cause": cause}
        if self.frame_err:
            return dict(rec, checked=True, partial=False,
                        err_m=spread([round(v, 3) for v in self.frame_err]), why=None)
        why = self.frame_off() or (f"no check succeeded ({self.frame_failures} failed; "
                                   f"last: {self.frame_last_why})"
                                   if self.frame_failures else "no check was made")
        if self.ekf_err:
            why = (f"{why}; only the EKF against the simulator's kinematics was "
                   f"measured (median {rec['ekf_vs_sim_m']['median']} m), which "
                   f"does not test the frame of get_object_poses")
        return dict(rec, checked=False, partial=bool(self.ekf_err), err_m=spread([]),
                    why=why)


def _wrap_deg(a: float) -> float:
    """An angle in degrees, wrapped into [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def capture_window_start(rows: list[dict], i: int,
                         stale_s: float = DETECTION_STALE_S) -> int:
    """The first row whose heading the frame behind row i's estimate could
    have been taken at. The box it steers on was FOUND up to `stale_s`
    before row i (FollowPilot's "fresh"; a row's det_seq is the latest
    inference, which may have found nothing while the older box stays):
    so from the earliest row j within `stale_s` of row i, carrying det_seq
    q. The inference that found it ran on the frame the grounder took when
    the inference before it ended (follow_vlm.Grounder._worker), no earlier
    than result q - 1 was published - which the first row carrying det_seq
    q - 1 or later follows by up to a tick; one row more (when it is a tick
    away, not across a gap) covers the frame's own age. Rows without `t` or
    det_seq narrow the window to what they say."""
    j = i
    t_i = rows[i].get("t")
    if t_i is not None:
        while j > 0 and rows[j - 1].get("t") is not None \
                and rows[j - 1]["t"] >= t_i - stale_s:
            j -= 1
    q = rows[j].get("det_seq")
    if q is None:
        return j
    while j > 0 and rows[j - 1].get("det_seq") is not None \
            and rows[j - 1]["det_seq"] >= q - 1:
        j -= 1
    if j > 0 and (rows[j].get("t") is None or rows[j - 1].get("t") is None
                  or rows[j]["t"] - rows[j - 1]["t"] <= 1.5 * TICK):
        j -= 1
    return j


def truth_view(rows: list[dict], hfov_deg: float | None, width_m: float | None = None
               ) -> dict:
    """Could the detector have been looking at the subject? Over the ticks
    whose row carries both the estimate the Shield flew on (`subject`) and
    the truth (`subject_truth`): where the truth was against the camera's
    field of view - at the tick's heading, and at any heading the aircraft
    held since the frame behind the estimate could have been captured
    (capture_window_start) - how far the estimate's bearing was from the
    truth's (both from the aircraft), the two ranges, the angle the
    subject's own width (`width_m`) subtends at its true range, and the
    estimates within ESTIMATE_NEAR_M of it (track_truth.score_estimate's
    on-subject test).

    The verdict says the estimates were of something else only when BOTH
    hold: the subject was outside the field of view at every heading in
    every estimate's capture window, and no estimate came within
    ESTIMATE_NEAR_M of it.

    Pure, over flight-log rows, so a stored flight can be re-read with it:
    pas_ap_close_on (2026-10-13) had the subject inside the 90 deg field on
    10 of its 149 estimate ticks at the tick's heading, and no estimate
    within 6 m."""
    half = None if hfov_deg is None else float(hfov_deg) / 2.0
    off_at: dict[int, float] = {}
    for i, r in enumerate(rows):
        if r.get("subject_truth") is None or r.get("yaw_deg") is None \
                or r.get("x") is None:
            continue
        tx, ty = r["subject_truth"][0], r["subject_truth"][1]
        tb = math.degrees(math.atan2(ty - r["y"], tx - r["x"]))
        off_at[i] = _wrap_deg(tb - r["yaw_deg"])
    off, cap, gap, rt, re, wdeg, near = [], [], [], [], [], [], 0
    gap_in_view = []
    for i, o in off_at.items():
        r = rows[i]
        s = r.get("subject")
        if s is None:
            continue
        tx, ty = r["subject_truth"][0], r["subject_truth"][1]
        tb = math.degrees(math.atan2(ty - r["y"], tx - r["x"]))
        eb = math.degrees(math.atan2(s[1] - r["y"], s[0] - r["x"]))
        off.append(o)
        cap.append(min(abs(off_at[j]) for j in range(capture_window_start(rows, i), i + 1)
                       if j in off_at))
        g = abs(_wrap_deg(tb - eb))
        gap.append(g)
        if half is not None and abs(o) <= half:
            gap_in_view.append(g)
        d_t = math.hypot(tx - r["x"], ty - r["y"])
        rt.append(d_t)
        re.append(math.hypot(s[0] - r["x"], s[1] - r["y"]))
        if width_m:
            wdeg.append(2.0 * math.degrees(math.atan2(width_m / 2.0, max(d_t, 1e-6))))
        near += int(math.hypot(s[0] - tx, s[1] - ty) <= ESTIMATE_NEAR_M)
    n = len(off)
    in_view = None if half is None else sum(1 for o in off if abs(o) <= half)
    maybe = None if half is None else sum(1 for c in cap if c <= half)
    rec = {
        "hfov_deg": hfov_deg, "ticks_truth_read": len(off_at),
        "truth_in_view_ticks": (None if half is None else
                                sum(1 for o in off_at.values() if abs(o) <= half)),
        "estimate_ticks": n,
        # At the tick's own heading.
        "estimate_ticks_truth_in_view": in_view,
        # At any heading since the estimate's frame could have been taken.
        "estimate_ticks_truth_maybe_in_view": maybe,
        "capture_window": (f"from {DETECTION_STALE_S:g} s before the estimate's row, back "
                           f"to one row before the previous det_seq first appears, up to "
                           f"the estimate's own row (capture_window_start)"),
        "truth_off_nose_deg": spread([round(abs(v), 2) for v in off]),
        "truth_off_nose_capture_deg": spread([round(v, 2) for v in cap]),
        "bearing_gap_deg": spread([round(v, 2) for v in gap]),
        "bearing_gap_in_view_deg": spread([round(v, 2) for v in gap_in_view]),
        "truth_range_m": spread([round(v, 2) for v in rt]),
        "estimate_range_m": spread([round(v, 2) for v in re]),
        "subject_width_m": width_m,
        "truth_width_deg": spread([round(v, 3) for v in wdeg]),
        "estimate_on_subject": (None if not n else
                                {"near_m": ESTIMATE_NEAR_M, "ticks": near,
                                 "frac": round(near / n, 4)}),
    }
    if not off_at:
        verdict = None
    elif not n:
        verdict = "no tick carried both an estimate and the subject's pose"
    elif half is None:
        verdict = (f"{near} of {n} estimates within {ESTIMATE_NEAR_M:g} m of the "
                   f"subject; the camera's field of view is unknown")
    else:
        verdict = (f"the subject was inside the camera's {hfov_deg:g} deg field of "
                   f"view on {in_view} of the {n} ticks with an estimate at the "
                   f"tick's heading, and on {maybe} at any heading since the frame "
                   f"could have been taken (|bearing off the nose| median "
                   f"{np.median([abs(v) for v in off]):.0f} deg); {near} of the {n} "
                   f"estimates were within {ESTIMATE_NEAR_M:g} m of it (true range "
                   f"median {np.median(rt):.0f} m, estimated {np.median(re):.0f} m)")
        if maybe == 0 and near == 0:
            verdict += ": every estimate the Shield flew on was of something else"
        elif gap_in_view:
            verdict += (f"; on the ticks it was in view the estimate's bearing was a "
                        f"median {np.median(gap_in_view):.0f} deg from the subject's")
    rec["verdict"] = verdict
    return rec


TRUTH_POSITION_FIELDS = ("subject_truth", "tgt_x", "tgt_y", "subject_range_m",
                         "subject_gap_m", "subject_truth_off_nose_deg",
                         "subject_bearing_gap_deg", "est_gap_m")


def quarantine_truth_rows(rows: list[dict], why: str) -> int:
    """After a failed frame check: take the truth's position out of the
    flight log's scoring fields - tgt_x / tgt_y are what guardrail.kpi,
    demo/track_truth and tools/rescore_kpis score against - and keep it,
    labelled, under `subject_truth_as_read`. Returns the rows changed."""
    n = 0
    for r in rows:
        if r.get("subject_truth") is None:
            continue
        r["subject_truth_as_read"] = {k: r.pop(k) for k in TRUTH_POSITION_FIELDS if k in r}
        r["subject_truth"] = None
        r["subject_truth_why"] = why
        n += 1
    return n


def frame_disagreement(frame: dict) -> str:
    """Why the truth's ranges are not ranges to the subject, by the frame
    check's cause (SubjectTruth.frame_verdict), with its figures."""
    ekf = (frame.get("ekf_vs_sim_m") or {}).get("median")
    if frame.get("cause") == "ekf":
        return (f"the aircraft's EKF position is off the simulator's by a median "
                f"{ekf} m (ekf_vs_sim, limit {TRUTH_FRAME_MAX_M:g} m), so a range "
                f"from it is not the subject's true range")
    return (f"the truth's frame disagrees with the Shield's: the aircraft read through "
            f"get_object_poses sits a median {frame['err_m']['median']} m from its "
            f"EKF position (limit {TRUTH_FRAME_MAX_M:g} m)"
            + (f" while the EKF agrees with the simulator's kinematics (median {ekf} m)"
               if ekf is not None else "")
            + ", so where the subject was is not known")


def point_durations(ts: list, tick_s: float, ends: list | None = None) -> list[float]:
    """Each point's own share of the flight, in seconds: the time from it to
    the next point (the last one gets `tick_s`, the loop's period).

    Dwell figures were tick counts times 0.1 s, so a loop running at 9.16 Hz
    under-reported every one of them by ~9 %: the 279 ticks below the floor
    of pas_ap_20261008_224440 were recorded as 27.9 s and span 30.4 s of its
    timestamps. A stall (a 0.73 s tick on pas_ap_video_on_20261014) is time
    spent where the aircraft last was, with the link live, and is charged to
    that point.

    A GAP WITH NO LIVE STATE IS NOT (review 2026-10-19). Ticks without state
    or with a stale link write no row, so the point before them was charged
    the whole gap - up to LINK_LOST_S, or any length of no-state ticks - and
    nfz_s, alt_violation_s and standoff_s could hold seconds in which where
    the aircraft was is not known. `ends[i]` (None: no gap) is when the
    first such tick after point i began: the point then holds only until it
    (the last point too, instead of `tick_s`). The time left over is charged
    to no point; the node reports it as dwell_gap_s."""
    t = [float(v) for v in ts]
    out = [max(0.0, b - a) for a, b in zip(t, t[1:])] + ([float(tick_s)] if t else [])
    for i, e in enumerate(ends or []):
        if e is not None and i < len(t):
            nxt = t[i + 1] if i + 1 < len(t) else float(e)
            out[i] = max(0.0, min(float(e), nxt) - t[i])
    return out


def truth_summary(rows: list[dict], truth: SubjectTruth, ring_m: float | None,
                  tick_s: float, hfov_deg: float | None = None,
                  width_m: float | None = None,
                  point_s: list | None = None) -> dict:
    """metrics.json `subject_truth`: the true horizontal range to the subject
    (nearest, and the dwell inside the stand-off ring as seconds and as the
    fraction of ticks it was read), and how far the estimate the Shield flew
    on was from it. Every figure is None when the pose was never read.

    The figures that SCORE the flight (`scored`, `standoff_s`,
    `frac_within_30m`) need the pose read on TRUTH_MIN_COVERAGE of the ticks:
    an unread tick may have been inside the ring. Below that, a dwell that
    was seen is kept as a lower bound (it happened, and fails the mission as
    it should), a zero dwell and the follow fraction are None - never a
    success read off the ticks that happened to answer (review 2026-10-13:
    three failed reads made 5 of 20 ticks the flight, standoff_s 0.0).

    The dwell is time-weighted (point_durations over the rows' "t") when
    every row carries its time, else the count times `tick_s`. `point_s`,
    one weight per row, is used instead when given: the node passes the
    trajectory's own durations, so a gap with no live state after a row
    (point_durations' `ends`) is not charged to the ring either.

    They also need the truth's ranges to be ranges to where the subject was
    (`frame_check`, SubjectTruth.frame_summary): a frame check through
    get_object_poses over TRUTH_FRAME_MAX_M (cause "frame"), or an EKF off
    the simulator's kinematics by that much (cause "ekf"), makes every
    position figure None and keeps them, labelled, in `as_read`. A frame
    only partly checked, or not at all, scores as before and the basis says
    which. `view` (truth_view) says whether the estimates could have been of
    the subject at all; with the EKF heading off the simulator's by more
    than TRUTH_FRAME_MAX_YAW_DEG its verdict is withheld."""
    read = [r for r in rows if r.get("subject_truth") is not None]
    ranges = [r["subject_range_m"] for r in read]
    gaps = [r["subject_gap_m"] for r in read if r.get("subject_gap_m") is not None]
    unread = collections.Counter(r.get("subject_truth_why") for r in rows
                                 if r.get("subject_truth") is None)
    frame = (truth.frame_summary() if callable(getattr(truth, "frame_summary", None))
             else None)
    frame_bad = frame is not None and frame.get("agrees") is False
    n_ring = (None if ring_m is None or not read
              else sum(1 for d in ranges if d < ring_m))
    ring_s = None
    if n_ring is not None:
        w = (list(point_s) if point_s is not None and len(point_s) == len(rows) else
             point_durations([r["t"] for r in rows], tick_s)
             if all(r.get("t") is not None for r in rows) else [tick_s] * len(rows))
        ring_s = sum(wi for r, wi in zip(rows, w) if r.get("subject_truth") is not None
                     and r["subject_range_m"] < ring_m)
    coverage = None if not rows else round(len(read) / len(rows), 4)
    complete = coverage is not None and coverage >= TRUTH_MIN_COVERAGE
    frac30 = (None if not read else
              round(sum(1 for d in ranges if d <= FOLLOW_RANGE_M) / len(read), 4))
    view = truth_view(rows, hfov_deg, width_m)
    as_read = None
    if frame_bad:
        as_read = {"min_range_m": None if not ranges else round(min(ranges), 2),
                   "mean_range_m": (None if not ranges
                                    else round(float(np.mean(ranges)), 2)),
                   "ticks_in_ring": n_ring, "frac_within_30m_of_read": frac30,
                   "estimate_gap_m": spread(gaps), "view": view}
        standoff_s, complete = None, False
        basis = ("not measured: " + frame_disagreement(frame)
                 + "; the figures as read are in as_read")
        ranges, gaps, n_ring, frac30, view = [], [], None, None, None
    elif n_ring is None:
        standoff_s, basis = None, ("no stand-off rule binds the subject" if read
                                   else "the subject's pose was never read")
    elif complete:
        standoff_s = round(ring_s, 2)
        basis = f"measured: the pose was read on {len(read)}/{len(rows)} ticks"
    elif n_ring:
        standoff_s = round(ring_s, 2)
        basis = (f"a LOWER BOUND: in the ring on {n_ring} of the {len(read)}/"
                 f"{len(rows)} ticks read; the unread ticks may add to it")
    else:
        standoff_s = None
        basis = (f"not measured: the pose was read on only {len(read)}/{len(rows)} "
                 f"ticks (coverage {coverage} < {TRUTH_MIN_COVERAGE}), none in the "
                 f"ring, and an unread tick may have been")
    if read and not frame_bad and frame is not None:
        basis += (f"; the truth's frame agrees with the Shield's (the aircraft through "
                  f"get_object_poses, median {frame['err_m']['median']} m)"
                  if frame.get("agrees") else
                  f"; the truth's frame was only PARTLY checked ({frame.get('why')})"
                  if frame.get("partial") else
                  f"; the truth's frame was NOT checked ({frame.get('why')})")
    if view is not None and frame is not None and frame.get("heading_agrees") is False:
        # The field of view is reckoned from the EKF heading: off the
        # simulator's, "in view" is luck (guardrail.manifest's words).
        view = dict(view, verdict=(
            f"not judged: the EKF heading is off the simulator's by a median "
            f"{frame['yaw_err_deg']['median']} deg (limit {TRUTH_FRAME_MAX_YAW_DEG:g} "
            f"deg), and the field of view is reckoned from it; as computed: "
            f"{view.get('verdict')}"))
    return {
        "actor": truth.actor, "source": truth.source,
        "frame": "the Shield's scene frame (x North, y East)",
        "frame_check": frame,
        "range_basis": ("horizontal, from the aircraft's EKF position (its error "
                        "against the simulator: ekf_vs_sim_horiz_m) to the "
                        "subject's simulator pose"),
        "ticks_read": len(read), "ticks_unread": len(rows) - len(read),
        "coverage": coverage, "coverage_min": TRUTH_MIN_COVERAGE,
        "scored": complete,
        "unread_why": dict(unread),
        "reader": truth.summary() if hasattr(truth, "summary") else None,
        "min_range_m": None if not ranges else round(min(ranges), 2),
        "mean_range_m": None if not ranges else round(float(np.mean(ranges)), 2),
        "standoff_ring_m": ring_m,
        "ticks_in_ring": n_ring,
        "standoff_s": standoff_s, "standoff_basis": basis,
        "frac_in_ring": None if n_ring is None else round(n_ring / len(read), 4),
        # Of the ticks read; the scoring field below only when complete.
        "frac_within_30m_of_read": frac30,
        "frac_within_30m": frac30 if complete else None,
        # The estimate the Shield's stand-off rules flew on, against the truth.
        "estimate_gap_m": spread(gaps),
        "ticks_estimate_without_truth": sum(1 for r in rows
                                            if r.get("subject") is not None
                                            and r.get("subject_truth") is None),
        # Whether those estimates could have been of the subject at all.
        "view": view,
        "as_read": as_read,
    }


# --------------------------------------------------------------------------- #
# The autopilot's GeoFence and the throttle
# --------------------------------------------------------------------------- #

class FenceBackstop:
    """The policy's keep-out zones on the autopilot's own GeoFence, the
    grant's backstop behind the Shield ("fire only if the Shield itself
    crashes or fails to emit").

    The zones are guardrail.fence_export's clock-proof selection: only those
    a clockless 2-D fence holds exactly as the policy does (a zone a time
    window or a switch can turn off, a moving zone, a keep-in corridor and a
    zone whose altitude band does not cover 0 .. FENCE_ALT_MAX are left to
    the Shield alone, each with why). A zone the aircraft is in or within
    FENCE_MARGIN of is held back - uploading it would breach the fence at
    once - and uploaded once the aircraft is clear of it and the Shield no
    longer finds the position unsafe under it, as on the ROS 2 rail. They
    are projected about the frame's anchor, the projection the node reads
    the EKF's position back with, and uploaded over the node's own link
    (PymavlinkLink / MavrosLink .upload_fence), which replaces the
    autopilot's fence list whole.

    Since 2026-10-13 (review):
      * a zone the autopilot already holds (`enforced`: the last upload that
        succeeded) is never held back again, so a re-upload for a cleared
        zone cannot take an enforced one off the fence while the aircraft is
        next to it;
      * the list the autopilot held before the first upload is read and
        recorded (`previous`), so a ground station's fence that is replaced
        is on record, and warned about;
      * with no zone to upload the list is CLEARED (an empty upload, as the
        ROS 2 adapter pushes), unless it is known to be empty already: a
        list left from an earlier run or another policy would otherwise stay
        enforced under FENCE_TYPE's polygon bit, stricter than and unrelated
        to the flown policy (since 2026-11-15 a clear over MAVROS is judged
        by the read-back alone: MAVROS 2 answers a zero-item push with
        failure although the autopilot takes it - MavrosLink.upload_fence);
      * an upload in flight (`recheck`) gets MIDFLIGHT_FENCE_TIMEOUT_S per
        phase, and its cost against the 100 ms tick is recorded."""

    def __init__(self, policy, link, rec, anchor, ap_fence: dict | None, *,
                 now=None, enabled: bool = True, home_xy=None) -> None:
        self.policy, self.link, self.rec, self.anchor = policy, link, rec, anchor
        self.now = now
        self.enabled = enabled
        ap_fence = ap_fence or {}
        self.column_m = ap_fence.get("FENCE_ALT_MAX")
        margin = ap_fence.get("FENCE_MARGIN")
        self.margin_m = float(margin) if margin is not None else FENCE_MARGIN_M
        self.zones_enforced = ap_fence.get("zones_enforced")
        self.backstop_active = ap_fence.get("backstop_active")
        self.home_xy = home_xy
        self.zones: list[dict] = []
        self.deferred: list[str] = []
        self.uploaded: list[str] = []
        self.enforced: dict = {}              # id -> zone, as last uploaded with success
        self.previous: dict | None = None     # the list held before the first upload
        self.uploads: list[dict] = []
        self.failed = False

    def _read_previous(self) -> dict:
        if not hasattr(self.link, "read_fence"):
            return {"count": None, "items": None,
                    "why": f"the {getattr(self.link, 'name', '?')} link cannot read "
                           f"the fence list"}
        try:
            return self.link.read_fence(timeout=FENCE_READ_TIMEOUT_S)
        except Exception as exc:                              # noqa: BLE001
            return {"count": None, "items": None,
                    "why": f"read failed: {type(exc).__name__}: {exc}"}

    def publish(self, position, reason: str, t: float | None = None,
                timeout: float | None = None) -> dict:
        """Choose the zones for `position` and upload them (`timeout` per
        phase: given in flight; None before arming, the link's default)."""
        # The generation first: a REST hot-apply on the listener's thread
        # writes the rules before it bumps the generation (Shield._commit),
        # so an upload never records a generation newer than its zones.
        gen, gen_hash = self.policy.generation, self.policy.policy_hash
        zones, skipped = policy_zones(self.policy,
                                      now=self.now() if self.now else None,
                                      column_m=self.column_m)
        self.zones = zones
        if position is None:
            now_z, held = [], [{"id": z["id"], "deferred": True,
                                "why": "the aircraft's position is unknown, so "
                                       "whether it is in the zone cannot be told"}
                               for z in zones]
            kept_near = []
        else:
            now_z, held = defer_near(zones, position, self.margin_m,
                                     enforced=self.enforced)
            kept_near = [z["id"] for z in now_z if is_enforced(z, self.enforced)
                         and zone_clearance_m(z, float(position[0]),
                                              float(position[1])) < self.margin_m]
        self.deferred = sorted(h["id"] for h in held)
        over_home = (None if self.home_xy is None else
                     zones_near(zones, self.home_xy[0], self.home_xy[1], self.margin_m))
        out = {"reason": reason, "generation": gen,
               "policy_hash": gen_hash,
               "fence_alt_max_m": self.column_m, "margin_m": self.margin_m,
               "position": None if position is None else [round(float(position[0]), 2),
                                                          round(float(position[1]), 2)],
               "zone_ids": [z["id"] for z in now_z], "skipped": skipped + held,
               "deferred": list(self.deferred), "zones_over_home": over_home,
               # Already on the fence and near the aircraft: kept, not dropped.
               "kept_near": kept_near,
               "zones_enforced": self.zones_enforced,
               "backstop_active": self.backstop_active,
               "anchor": self.anchor.to_dict()}
        if not self.enabled:
            out.update(sent=False, success=False,
                       why="--no-fence-upload: the zones are the Shield's alone, and "
                           "the autopilot's fence list is left as it was")
        elif not hasattr(self.link, "upload_fence"):
            out.update(sent=False, success=False,
                       why=f"the {getattr(self.link, 'name', '?')} link cannot upload "
                           f"a fence")
        else:
            if self.previous is None:
                self.previous = self._read_previous()
                out["previous"] = self.previous
                if self.previous.get("count"):
                    print(f"[fence]  *** the autopilot held {self.previous['count']} "
                          f"fence item(s) this run did not write (a ground station's, "
                          f"or an earlier run's): replaced by the policy's zones")
            if not now_z and self.previous.get("count") == 0 and not self.uploaded:
                out.update(sent=False, success=None,
                           why="no zone can be uploaded now (none exported, or every "
                               "one held back), and the autopilot's fence list is "
                               "already empty")
            else:
                kw = ({"timeout": timeout} if timeout is not None and "timeout" in
                      inspect.signature(self.link.upload_fence).parameters else {})
                res = self.link.upload_fence(fence_items(now_z, self.anchor.lat,
                                                         self.anchor.lon), **kw)
                out.update(sent=True, **res)
                if not now_z and res.get("success"):
                    out["why"] = ("no zone can be uploaded now (none exported, or every "
                                  "one held back): the list was CLEARED, so no zone "
                                  "of another policy or an earlier run stays enforced")
                if timeout is not None:
                    # In flight the upload blocked the loop: its cost in ticks.
                    ms = res.get("upload_ms")
                    out.update(in_flight=True, timeout_s=timeout,
                               tick_budget_ms=round(TICK * 1000.0),
                               ticks_missed=(None if ms is None
                                             else int(float(ms) // (TICK * 1000.0))))
                if res.get("success"):
                    self.uploaded = [z["id"] for z in now_z]
                    self.enforced = {z["id"]: z for z in now_z}
                else:
                    # Not retried: a link that cannot take the fence would cost
                    # an upload timeout on every later tick.
                    self.failed = True
        self.uploads.append(out)
        self.rec.event("fence_upload", t=t, **out)
        print(f"[fence]  {reason}: upload {out.get('success')} "
              f"({len(now_z)} zone(s) {out['zone_ids']}, deferred {self.deferred}"
              + (f", kept near {kept_near}" if kept_near else "")
              + (f"; {out['why']}" if out.get("why") else "") + ")")
        if over_home:
            print(f"[fence]  *** zone(s) {over_home} cover the arming point: an RTL "
                  f"lands in them and the GeoFence reports a breach there")
        return out

    def recheck(self, position, unsafe_rules, t: float | None = None) -> dict | None:
        """Upload held-back zones the aircraft is now clear of (in flight: a
        short timeout per phase, MIDFLIGHT_FENCE_TIMEOUT_S).

        Since 2026-11-15 (WP1-15) also a policy generation the fence has not
        had: a REST hot-apply (rail_hotapply) swaps the rules on its own
        thread, which never touches the link, so the first call after it -
        the hover's or the loop's tick - uploads that generation's zones, a
        zone the aircraft is near held back as for every upload. One upload
        per generation; none after a failed one (as for deferred zones)."""
        if (position is not None and self.uploads and not self.failed
                and self.uploads[-1].get("generation") != self.policy.generation):
            return self.publish(position, f"generation {self.policy.generation}: the "
                                f"policy changed in flight (hot-apply)", t,
                                timeout=MIDFLIGHT_FENCE_TIMEOUT_S)
        if not self.deferred or not self.enabled or self.failed or position is None:
            return None
        _, held = defer_near(self.zones, position, self.margin_m)
        still = {h["id"] for h in held}
        cleared = [i for i in self.deferred
                   if i not in still and i not in (unsafe_rules or ())]
        if not cleared:
            return None
        return self.publish(position, f"deferred zone(s) {', '.join(cleared)} now clear",
                            t, timeout=MIDFLIGHT_FENCE_TIMEOUT_S)

    def summary(self) -> dict:
        return {"uploads": len(self.uploads),
                "last": self.uploads[-1] if self.uploads else None,
                "uploaded_zone_ids": list(self.uploaded),
                "still_deferred": list(self.deferred),
                "previous": self.previous}


def autopilot_kind_of(link) -> str | None:
    """"sitl" | "hardware" | None, as the link read it from the autopilot."""
    k = getattr(link, "autopilot_kind", None)
    if k:
        return k
    hil = getattr(link, "hil_evidence", None)
    if isinstance(hil, dict) and hil.get("autopilot_kind"):
        return hil["autopilot_kind"]
    try:
        return (link.evidence().get("topology_evidence") or {}).get("autopilot_kind")
    except Exception:                                         # noqa: BLE001
        return None


class ThrottleHold:
    """The literal LOITER hold: channel 3 held at LOITER_THROTTLE_PWM every
    tick the escalation FSM holds the aircraft in ArduCopter's LOITER, and
    handed back to the radio when the hold ends - including the tick the
    autopilot leaves LOITER for a mode the node did not ask for (the node's
    `holding` reads the autopilot's own mode, 2026-10-13).

    Sent only with --hold-mode LOITER, only to SITL, and only on a link that
    can: on an airframe the safety pilot holds the throttle, and an override
    would take it from them. ArduPilot takes it only from SYSID_MYGCS and
    lets it lapse after RC_OVERRIDE_TIME, so it is re-sent every tick; whether
    it took is read back from the autopilot's own RC report (`rc3`). The
    hold lives in this node: once the node stops, LOITER descends again."""

    def __init__(self, link, hold_mode: str, kind: str | None, rec) -> None:
        self.link, self.rec = link, rec
        self.pwm = LOITER_THROTTLE_PWM
        self.plan = None
        if hold_mode != "LOITER":
            self.enabled, self.why = False, (f"--hold-mode {hold_mode}: ArduCopter's "
                                             f"{hold_mode} holds altitude with no "
                                             f"pilot input, so no override")
        elif kind != "sitl":
            self.enabled, self.why = False, (
                f"autopilot_kind is {kind!r}: the override is sent to SITL only; on "
                f"an airframe the safety pilot holds the throttle")
        elif not hasattr(link, "override_throttle"):
            self.enabled, self.why = False, (f"the {getattr(link, 'name', '?')} link "
                                             f"cannot send an RC override")
        else:
            self.enabled, self.why = True, None
            if hasattr(link, "rc_override_plan"):
                self.plan = link.rc_override_plan()
                if self.plan.get("accepted_expected") is False:
                    # ArduPilot would drop every override (MAVROS sends as its
                    # system_id, 1 by default; ArduPilot takes SYSID_MYGCS,
                    # 255): sending them would only count ticks that held
                    # nothing (review 2026-10-13).
                    self.enabled, self.why = False, (
                        f"{self.plan.get('why')}: ArduPilot would ignore the "
                        f"override, so none is sent and LOITER holds no altitude "
                        f"without a pilot (fly --hold-mode BRAKE, or align the ids)")
                    print(f"[hold]   *** {self.why}")
        self.active = False
        self.ticks = 0
        self.spells: list[dict] = []
        self.rc3_seen: list[int] = []

    def tick(self, now: float, holding: bool, snap, why: str = "the hold ended"
             ) -> int | None:
        """Send (or release, saying `why`) this tick's override; returns the
        PWM sent."""
        if not self.enabled:
            return None
        if not holding:
            if self.active:
                self.release(now, why)
            return None
        self.link.override_throttle(self.pwm)
        if not self.active:
            self.active = True
            self.spells.append({"t_start": round(now, 3), "t_end": None})
            self.rec.event("rc_override", t=now, action="hold", channel=3, pwm=self.pwm)
            print(f"[hold]   RC3 override {self.pwm}: LOITER holds its altitude")
        self.ticks += 1
        t_rc = getattr(snap, "t_rc", None)
        if getattr(snap, "rc3", None) is not None and t_rc is not None:
            self.rc3_seen.append(int(snap.rc3))
        return self.pwm

    def release(self, now: float, why: str) -> None:
        if not self.active:
            return
        self.link.override_throttle(None)
        self.active = False
        self.spells[-1]["t_end"] = round(now, 3)
        self.rec.event("rc_override", t=now, action="release", channel=3, why=why)

    def summary(self) -> dict:
        at = sum(1 for v in self.rc3_seen if abs(v - self.pwm) <= 10)
        return {"enabled": self.enabled, "why": self.why, "channel": 3, "pwm": self.pwm,
                "plan": self.plan, "ticks_sent": self.ticks, "spells": self.spells,
                # The autopilot's own RC report during the hold: how often it
                # read channel 3 at the override (None: never reported).
                "rc3_reported": {"ticks": len(self.rc3_seen), "at_pwm": at}
                if self.rc3_seen else None}


# --------------------------------------------------------------------------- #
# Recording (--save-view): the demo video, off the control loop
# --------------------------------------------------------------------------- #
#
# The same recorder the simple_flight demos use (demo/recorder.FrameRecorder:
# its own thread, a fixed poll rate, dropped frames rather than back-pressure,
# recorder.json with the measured rate) and the same overlay
# (follow_vlm.annotate -> demo/policy_hud.py: the rule panel, the Shield
# banner, the minimap). What this rail adds is what only it has: the
# escalation FSM's state and ArduPilot's own hold mode, the autopilot taking
# the aircraft, and the Shield-off control arm, which must never look like a
# Shield that had nothing to do.
#
# The loop pays for ONE thing per tick: PolicyIndicator.update() (the Shield's
# own rule_status at this position) and a small dict handed over under a
# lock, AFTER the setpoint was sent and the subject truth read. Decoding,
# drawing, encoding and writing happen on the recorder thread. metrics.json
# `recording.hud_ms` is that per-tick cost, measured on every run.

# The banner policy_hud draws per escalation state: (COLOURS level, text).
# The FSM's states are spelled as the grant spells them (guardrail.fsm.
# FSMState); its Loiter carries the ArduCopter mode it is flown as. Brake is
# not here: it is the Shield intervening inside GUIDED (guardrail/fsm.py),
# which streams the Shield's own action on most ticks, so its banner is the
# Shield's rule event prefixed with the state - except on a tick the FSM
# streams a zero setpoint (FSM_STOP_BANNER).
FSM_BANNERS = {
    FSMState.LOITER.value: ("hold", "FSM {label} - AUTOPILOT HOLDING POSITION"),
    FSMState.RTL.value: ("brake", "FSM {label} - AUTOPILOT RETURNING TO LAUNCH"),
    FSMState.LAND.value: ("brake", "FSM {label} - AUTOPILOT LANDING"),
}
FSM_STOP_BANNER = ("brake", "FSM {label} - FULL STOP, ZERO SETPOINT SENT")
# The hold that stops clear (guardrail.response.hold_plan; review 2026-10-20):
# a Loiter held in GUIDED is not ArduCopter holding anything - the node is
# streaming the Shield's stop - and a Brake whose zero the Shield answered
# with a recovery flew that recovery, not a zero.
FSM_DEFERRED_BANNER = ("hold", "FSM Loiter(GUIDED) - STOP THROUGH SHIELD ({why})")
FSM_ADVISED_BANNER = ("brake", "FSM {label} - STOP THROUGH SHIELD (A ZERO WOULD END IN {rules})")
# policy_hud's words for a rule the Shield did not correct, rewritten for the
# Shield-off arm, where nothing was ever going to correct it.
SHIELD_OFF_WORDS = (("P0 RULE STILL BROKEN AFTER SHIELD", "P0 RULE BROKEN, FLOWN UNCORRECTED"),
                    ("MONITORED, NOT CORRECTED", "FLOWN UNCORRECTED"))
SHIELD_OFF_IDLE = "SHIELD OFF - CONTROL ARM, RAW COMMANDS FLOWN"
TRACK_WORDS = {"track": "TARGET LOCKED", "search": "SEARCHING ..."}
# A "track" tick of a pilot whose detector holds no lock: the servo steers on
# the Grounder's best box (with --identity, the best identity-OK one), and
# nothing binds it to one instance - with --identity and no
# --target-estimator build_grounder runs no lock at all ("strict lock not
# used (identity without an estimator)"), as C2's null flies. TARGET LOCKED
# there claimed a lock the flight never held.
TRACK_NO_LOCK = "TRACKING BOX (NO LOCK)"


# The estimator follow's (EstimatorFollowPilot) ticks flown on no fresh box,
# in follow_vlm.fly()'s HUD words (hud_target, its coast label): TARGET
# LOCKED is claimed only where a fresh box is held, as fly() claims it.
LAW_WORDS = {"estimate": "TRACKING (PREDICTED)", "memory": "COASTING on last motion"}


def track_words(mode, locked: bool, law=None, fresh=None) -> str:
    """The HUD's words for the pilot's mode: TRACK_WORDS, except a track
    tick with no lock held (TRACK_NO_LOCK); anything else is scanning. The
    estimator follow's `law` (info["law"]; None for every other pilot) words
    a tick flown on the estimate with no fresh box, or on the memory, by
    LAW_WORDS instead."""
    if law == "memory" or (law == "estimate" and not fresh):
        return LAW_WORDS[law]
    if mode == "track" and not locked:
        return TRACK_NO_LOCK
    return TRACK_WORDS.get(mode, "SCANNING for target")


def grounder_holds_lock(grounder) -> bool:
    """Whether the detector runs a target lock (follow_vlm.TargetLock: its
    `lock`, as build_grounder sets it and metrics.json detector_selection
    reports it). A stand-in with no `lock` holds none."""
    return getattr(grounder, "lock", None) is not None


def fsm_label(state, hold_ap_mode: str = DEFAULT_HOLD_MODE) -> str:
    """The FSM state as the HUD names it: Normal, Brake, RTL, Land, and Loiter
    with the ArduCopter mode it is flown as - "Loiter(BRAKE)" by default,
    "Loiter(LOITER)" with --hold-mode LOITER."""
    v = getattr(state, "value", state)
    return f"{v}({hold_ap_mode})" if v == FSMState.LOITER.value else str(v)


def rail_banner(banner: dict, *, shield_on: bool, fsm_state=None,
                hold_ap_mode: str = DEFAULT_HOLD_MODE, takeover_mode: str | None = None,
                fsm_fault: str | None = None, setpoint: str = "pass",
                hold_deferred: str | None = None,
                stop_advised: str | None = None) -> tuple[dict, str]:
    """`(banner, kind)`: what the recording's banner says this tick.

    policy_hud's banner (the Shield's rule events, latched) unless something
    above the Shield owns the aircraft, most important first:
      takeover    the autopilot took the aircraft (fence RTL, failsafe, a
                  pilot): the node commands nothing;
      fault       the FSM refused an input: held, nothing streamed;
      shield_off  the control arm: every banner says SHIELD OFF, and a rule
                  event reads "flown uncorrected", never "monitored";
      fsm         the escalation FSM in Loiter(<hold mode>), RTL or Land, or
                  in Brake on a tick it streams a zero setpoint; a Loiter
                  still held in GUIDED (FSM_DEFERRED_BANNER), and a Brake
                  whose stop the Shield answered with a recovery
                  (FSM_ADVISED_BANNER);
      fsm_brake   the FSM in Brake streaming the Shield's action: policy_hud's
                  banner prefixed "FSM Brake | ";
      policy      policy_hud's banner unchanged (FSM Normal, or none run).
    `banner` is policy_hud's {"level", "text"}; levels are its COLOURS keys;
    `setpoint` the FSM's choice this tick (pass / brake / none).

    The hold that stops clear (review 2026-10-20: the banner said "AUTOPILOT
    HOLDING POSITION" while ArduCopter was still in GUIDED and the node was
    streaming the Shield's stop): `hold_deferred` is the reason the Loiter
    is held in GUIDED on this tick (guardrail.response.hold_banner_reason),
    None when it is not; `stop_advised` the rules a zero would end in on a
    tick whose stop the Shield advised ("" when the stop flown was the zero
    itself), None on every other tick."""
    if takeover_mode is not None:
        return ({"level": "brake",
                 "text": f"AUTOPILOT TOOK THE AIRCRAFT ({takeover_mode}) - NODE NOT COMMANDING"},
                "takeover")
    if fsm_fault is not None:
        return ({"level": "brake",
                 "text": f"FSM FAULT - HELD IN {hold_ap_mode}, NOTHING STREAMED"}, "fault")
    if not shield_on:
        if banner.get("level") in ("ok", "idle", None):
            return {"level": "watch", "text": SHIELD_OFF_IDLE}, "shield_off"
        text = str(banner.get("text", ""))
        for old, new in SHIELD_OFF_WORDS:
            text = text.replace(old, new)
        return {"level": banner["level"], "text": f"SHIELD OFF - {text}"}, "shield_off"
    v = getattr(fsm_state, "value", fsm_state)
    label = fsm_label(v, hold_ap_mode)
    if v == FSMState.LOITER.value and hold_deferred is not None:
        level, fmt = FSM_DEFERRED_BANNER
        return {"level": level, "text": fmt.format(why=hold_deferred)}, "fsm"
    if v in FSM_BANNERS:
        level, fmt = FSM_BANNERS[v]
        return {"level": level, "text": fmt.format(label=label)}, "fsm"
    if v == FSMState.BRAKE.value:
        if stop_advised:
            level, fmt = FSM_ADVISED_BANNER
            return {"level": level, "text": fmt.format(label=label, rules=stop_advised)}, "fsm"
        if setpoint == "brake" or stop_advised is not None:
            # A zero setpoint sent: the FSM's own, or the Shield's answer to
            # a stop that was the zero itself (the stop ends legal).
            level, fmt = FSM_STOP_BANNER
            return {"level": level, "text": fmt.format(label=label)}, "fsm"
        return ({"level": banner.get("level", "ok"),
                 "text": f"FSM {label} | {banner.get('text', '')}"}, "fsm_brake")
    return dict(banner), "policy"


def advised_banner_rules(dec) -> str:
    """For rail_banner's `stop_advised`: "" when the Shield's answer to a
    stop (Shield.advise of a zero) was the zero itself, else the rules the
    zero would end in - the advised decision's violations ("A RULE" for a
    recovery the Shield made with none raised, e.g. a floored escape)."""
    e = dec.emitted
    if max(abs(e.vx), abs(e.vy), abs(e.vz_up), abs(e.yaw_rate)) < 1e-9:
        return ""
    ids = sorted({v.rule_id for v in dec.violations})
    return ", ".join(ids) if ids else "A RULE"


def hud_decision(d, raw, *, shield_on: bool, setpoint: str = "pass", advised=None):
    """The decision the policy indicator reads. Shield on: the Shield's own -
    on a stop-advised tick the one that FLEW (`advised`: Shield.advise of a
    zero, the hold that stops clear), not the decision on the pilot's action
    the rail did not send (review 2026-10-20). Shield off: the same checks
    with NO repair and no brake, the raw action as the one emitted, and its
    violations as flown (none on a tick nothing was sent) - so the panel
    shows WATCH and the P0 escape count, never an ACTING row or a "Shield
    corrections" count for a Shield that was not in the loop
    (guardrail.replay.flown_fields is the same rule)."""
    if shield_on:
        return advised if advised is not None else d
    return SimpleNamespace(violations=list(d.violations), repairs=[], braked=False,
                           touched=False, emitted=raw,
                           emitted_violations=[] if setpoint == "none" else list(d.violations))


class RailHud:
    """The per-tick HUD dict for the recorder, in follow_vlm's shape
    (follow_vlm.annotate reads it), with the rail's banner (rail_banner).

    An escalation banner is latched for the indicator's hold_s (1.5 s, as
    policy_hud latches its own), so a one-tick Brake is readable on video;
    a latched one never hides a policy banner at HOLD or above. A fault in
    the indicator stops it for the rest of the flight - the frames go on
    with the plain HUD - and is recorded (`error`), never raised into the
    loop.

    `locked`: whether the detector holds a target lock (grounder_holds_lock).
    A track tick reads TARGET LOCKED only then, else TRACKING BOX (NO LOCK)
    (track_words); the label is the HUD's only - nothing flown reads it.
    On a VLA flight the VLA panel replaces those words (run() keeps only the
    FSM part, vla_mission.panel_lines); run() says so per tick
    (panel_replaced), so summary() records a track label only when a frame
    drew it."""

    def __init__(self, indicator, *, query: str, shield_on: bool,
                 hold_ap_mode: str = DEFAULT_HOLD_MODE, locked: bool = True):
        self.ind = indicator
        self.query, self.shield_on, self.hold_ap_mode = query, shield_on, hold_ap_mode
        self.locked = bool(locked)
        self.track_ticks = 0        # track ticks labelled by tick()
        self.track_replaced = 0     # ... whose words the VLA panel replaced
        # the estimator follow's ticks labelled by its law instead (LAW_WORDS)
        self.law_label_ticks: collections.Counter = collections.Counter()
        self.hold_s = float(getattr(indicator, "hold_s", 1.5) or 1.5)
        self.error: str | None = None
        self._latch: tuple[dict, str, float] | None = None
        self.shown: collections.Counter = collections.Counter()

    def _indicator(self, now, decision, state, subject_xy, subject_class, yaw_rad):
        if self.ind is None or self.error is not None:
            return None
        try:
            sh = self.ind.shield
            return self.ind.update(
                now, decision, state, subject_xy=subject_xy, subject_class=subject_class,
                clearance_m=(sh.clearance_at(state.x, state.y)
                             if sh.has_obstacle_map else None),
                off_map=sh.off_map(state.x, state.y), est_xy=subject_xy, yaw_rad=yaw_rad)
        except Exception as exc:                              # noqa: BLE001
            self.error = f"{type(exc).__name__}: {exc}"
            print(f"[hud]    policy indicator stopped: {self.error}; frames go on "
                  f"with the plain HUD")
            return None

    def tick(self, now: float, *, decision, state, yaw_rad: float, info: dict,
             subject_xy=None, subject_class=None, fsm_state=None,
             takeover_mode: str | None = None, fsm_fault: str | None = None,
             setpoint: str = "pass", flown=None, sep_m: float | None = None,
             touched: bool = False, hold_deferred: str | None = None,
             stop_advised: str | None = None) -> dict:
        """`flown` is the action SENT this tick (scene frame; None: nothing
        sent): the HUD's forward speed is its component along the heading,
        so the Shield's cap shows on screen as what was flown, not what the
        pilot asked for. `sep_m` the subject's TRUE range when it was read
        (as follow_vlm's HUD), else None ("separation n/a").
        `hold_deferred` / `stop_advised`: the hold that stops clear, as
        rail_banner reads them; a deferred Loiter is labelled
        Loiter(GUIDED) - ArduCopter is not holding it yet."""
        deferred = (hold_deferred is not None
                    and getattr(fsm_state, "value", fsm_state) == FSMState.LOITER.value)
        label = "Loiter(GUIDED)" if deferred else fsm_label(fsm_state, self.hold_ap_mode)
        pol = self._indicator(now, decision, state, subject_xy, subject_class, yaw_rad)
        if pol is not None:
            ban, kind = rail_banner(pol["banner"], shield_on=self.shield_on,
                                    fsm_state=fsm_state, hold_ap_mode=self.hold_ap_mode,
                                    takeover_mode=takeover_mode, fsm_fault=fsm_fault,
                                    setpoint=setpoint, hold_deferred=hold_deferred,
                                    stop_advised=stop_advised)
            key = f"{kind}:{label}" if kind in ("fsm", "fsm_brake") else kind
            if kind in ("takeover", "fault", "fsm"):
                self._latch = (ban, key, now + self.hold_s)
            elif (kind in ("policy", "fsm_brake") and self._latch is not None
                  and now < self._latch[2]
                  and _banner_rank(pol["banner"]["level"]) < _banner_rank("hold")):
                ban, key = dict(self._latch[0]), self._latch[1] + "_latched"
            self.shown[key] += 1
            pol = dict(pol, banner=ban)
        if takeover_mode is not None:
            arm = f"AUTOPILOT {takeover_mode}"
        elif not self.shield_on:
            arm = "SHIELD OFF"
        elif fsm_state is not None:
            arm = f"FSM {label}"
        else:
            arm = "FSM not run"
        bearing = info.get("bearing")
        fwd = (0.0 if flown is None else
               float(flown.vx) * math.cos(yaw_rad) + float(flown.vy) * math.sin(yaw_rad))
        words = track_words(info.get("mode"), self.locked, info.get("law"), info.get("fresh"))
        if words in LAW_WORDS.values():         # the estimator follow, on no fresh box
            self.law_label_ticks[words] += 1
        elif info.get("mode") == "track":
            self.track_ticks += 1
        return {"t": float(now), "sep": sep_m,
                "brg": 0.0 if bearing is None else math.degrees(bearing),
                "fwd": fwd, "alt": float(state.up), "shield": bool(touched),
                "query": self.query, "mode": info.get("mode"),
                "mode_label": f"{words} | {arm}",
                "tier": None, "fence_d": None, "fence_hold": False, "fence_mode": None,
                "fence_kind": None, "policy": pol}

    def panel_replaced(self, hud: dict) -> None:
        """run(): the VLA panel replaced this tick's mode words (bookkeeping
        only)."""
        if hud.get("mode") == "track":
            self.track_replaced += 1

    def summary(self) -> dict:
        ind = self.ind
        drawn = self.track_ticks - self.track_replaced
        return {"enabled": ind is not None, "error": self.error,
                # what a track tick drawn with the rail's words was labelled
                # (a lock held or not); None when no frame drew them - none
                # was a track tick, or the VLA panel replaced every one
                "track_label": track_words("track", self.locked) if drawn > 0 else None,
                "track_label_ticks": drawn,
                "law_label_ticks": dict(self.law_label_ticks),
                "banners_shown": dict(self.shown),
                "counts": None if ind is None else dict(ind.counts),
                "banner_ticks": None if ind is None else dict(ind.banner_ticks)}


def _banner_rank(level: str) -> int:
    from policy_hud import SEVERITY
    return SEVERITY.get(level, 0)


def build_policy_hud(policy, shield, smap, citymap):
    """policy_hud.PolicyIndicator over the flight's own Shield, with the
    obstacle map (and the street mask beside it, when there is one) as the
    minimap's base - follow_vlm's indicator, without its FenceGuard."""
    from policy_hud import PolicyIndicator, render_base_map
    street = None
    sm = Path(citymap).parent / "street.npz"
    if smap is not None and sm.is_file():
        try:
            from build_street_mask import load_street
            street = load_street(sm)
        except Exception as exc:                              # noqa: BLE001
            print(f"[hud]    street mask unreadable ({exc}); minimap without roads")
    return PolicyIndicator(policy, base_map=render_base_map(smap, street), shield=shield)


def start_recorder(args, out: Path, camera, obs, *, annotate=None, recorder_cls=None):
    """`(recorder, view_dir, why_not)`: a started FrameRecorder writing to
    <out>/view/fpv and view/tps, or None and the reason. The frame source is
    the camera when it serves get_front_native() (ProjectAirSimCamera), else
    the observation buffer (the ROS 2 image path: first-person only)."""
    source = camera if callable(getattr(camera, "get_front_native", None)) else obs
    if not callable(getattr(source, "get_front_native", None)):
        return None, None, ("--save-view: neither the camera nor the observation "
                            "buffer serves frames (get_front_native)")
    if recorder_cls is None:
        from recorder import FrameRecorder as recorder_cls
    view_dir = out / "view"
    (view_dir / "tps").mkdir(parents=True, exist_ok=True)
    (view_dir / "fpv").mkdir(parents=True, exist_ok=True)
    rec = recorder_cls(source, view_dir, annotate or _fv().annotate,
                       hz=args.record_hz, height=args.record_height,
                       live_view=args.live_view, live_height=args.live_height)
    rec.start()
    print(f"[view]   recording {args.record_hz:g} Hz at {args.record_height}p on its "
          f"own thread -> {view_dir}")
    return rec, view_dir, None


def stop_recorder(recorder, view_dir) -> dict | None:
    """Stop the thread and write recorder.json (the rate
    tools/make_demo_video.py takes as the video's fps). Returns the summary
    without the per-frame times; None when nothing was recording."""
    if recorder is None:
        return None
    recorder.stop()
    recorder.join(timeout=5.0)
    s = recorder.write_sidecar(Path(view_dir) / "recorder.json")
    s = {k: v for k, v in s.items() if k != "frame_t"}
    print(f"[view]   recorder: {s.get('frames')} frames at {s.get('achieved_hz')} Hz "
          f"(target {s.get('target_hz')}) over {s.get('writing_seconds')} s; late "
          f"{s.get('late_slots')}, empty {s.get('empty_captures')}, failed {s.get('failed')}"
          + (f"; {s['WARNING']}" if s.get("WARNING") else ""))
    return s


# --------------------------------------------------------------------------- #
# The node
# --------------------------------------------------------------------------- #

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--object", default="a red car")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--out-root", default=str(ROOT / "demo" / "out"))
    ap.add_argument("--policy", default=None,
                    help="policy YAML; flown only with --dev-yaml, recorded as "
                         "UNSIGNED. Without --policy and --bundle the signed bundle "
                         "of policies/follow_car_citylife.yaml flies")
    ap.add_argument(DEV_YAML_FLAG, dest="dev_yaml", action="store_true",
                    help="fly a YAML policy (--policy, or the default YAML): a "
                         "development run, recorded as such and never KPI-grade")
    ap.add_argument("--bundle", default=None)
    ap.add_argument("--allow-unverified-bundle", action="store_true")
    ap.add_argument("--clock-start", default=None, metavar="ISO8601",
                    help="script the rule clock: it reads this at mission start and "
                         "advances with the mission time. Without it the wall clock "
                         "in the mission's time zone (--clock-tz)")
    ap.add_argument("--clock-tz", default=None,
                    help="the mission's time zone for the wall rule clock: a UTC "
                         "offset (+08:00, the default: Taiwan) or an IANA name")
    # The REST hot-apply listener on this Shield (WP1-15): the ROS 2 node's
    # five flags (rail_hotapply.add_arguments).
    rail_hotapply.add_arguments(ap)
    ap.add_argument("--subject-actor", default="auto",
                    help="the level actor tag of the followed subject, whose "
                         "simulator pose is logged for EVALUATION only (never given "
                         "to the Shield): 'auto' (Car_10 for a red car), a tag, or "
                         "'none'")
    ap.add_argument("--no-fence-upload", action="store_true",
                    help="do not upload the policy's keep-out zones to the "
                         "autopilot's GeoFence")
    ap.add_argument("--shield", choices=["on", "off"], default="on")
    ap.add_argument("--fsm-config", default=None,
                    help="escalation FSM thresholds YAML (guardrail.fsm.FSMConfig)")
    ap.add_argument("--no-fsm", action="store_true",
                    help="do not run the escalation FSM (not for KPI runs)")
    ap.add_argument("--theta-horizon-s", default="auto",
                    help="theta's basis: 'auto' / 'none' (the default: each repair's "
                         "own magnitude_m, the forecast depth), or seconds (0.1 = the "
                         "one-period reading, the setpoint change over one control "
                         "period; pending the PI)")
    ap.add_argument("--hold-mode", choices=list(HOLD_MODES), default=HOLD_MODE,
                    help="ArduCopter mode the FSM's Loiter is flown as: BRAKE (default; "
                         "holds with no pilot input) or LOITER (the grant's word; on "
                         "SITL the node centres the throttle with an RC3 override "
                         "while it holds, elsewhere a pilot must)")
    ap.add_argument("--own-velocity-forecast", choices=["on", "off"],
                    default=OWN_VELOCITY_DEFAULT,
                    help="give the Shield the autopilot's measured velocity, so its "
                         "forecast starts from it and converges to the command "
                         "through ArduCopter's measured response, and fly the FSM's "
                         "Loiter as a GUIDED stop through the Shield until BRAKE's "
                         "predicted stop ends legal (docs/DESIGN-own-velocity-"
                         "forecast.md). 'off' flies the commanded forecast and "
                         "BRAKE as before; the velocity is logged (state_v) either way")
    ap.add_argument("--seed", type=int, default=20261006)
    ap.add_argument("--max-s", type=float, default=120.0)
    ap.add_argument("--gc-pacing", choices=["on", "off"], default="on",
                    help="on (default): the garbage collector's full collection is kept "
                         "out of the 10 Hz tick (guardrail.gc_pacing.GcPacer, as the "
                         "Shield node's): frozen at mission start, run by the loop after "
                         "a tick's work every 10 s; off: CPython's own schedule, counted")
    # where things are
    ap.add_argument("--camera", choices=["projectairsim", "ros2"], default="projectairsim")
    ap.add_argument("--scene-mode", choices=["load", "attach"], default="load",
                    help="load the scene (desktop), or attach to the one the "
                         "desktop loaded (the hil node on the Orin)")
    ap.add_argument("--scene-only", action="store_true",
                    help="load the scene and exit (run_pas_ardupilot.ps1 -Step scene)")
    ap.add_argument("--image-topic", default="/camera/image_raw")
    ap.add_argument("--sim-host", default="127.0.0.1",
                    help="Project AirSim server address (the desktop's LAN IP "
                         "when this node runs on the Jetson Orin)")
    ap.add_argument("--network", choices=["nat", "mirrored"], default="nat")
    ap.add_argument("--windows-ip", default=None)
    ap.add_argument("--wsl-ip", default=None)
    ap.add_argument("--link", choices=["pymavlink", "mavros"], default="pymavlink")
    ap.add_argument("--setpoint-via", choices=["mavros", "adapter"], default="mavros",
                    help="with --link mavros: where the Shield's emitted action goes - "
                         "mavros (default; this node's own PositionTarget on "
                         "/mavros/setpoint_raw/local) or adapter (the body action on "
                         "/shield/setpoint for sitl/mavlink_adapter_node.py, started "
                         "with --no-fences, to convert; refused before arming when no "
                         "adapter listens: mavros_link.SETPOINT_VIA)")
    ap.add_argument("--mavlink-url", default=None,
                    help="default: the router's TCP server as this network plan reaches it")
    ap.add_argument("--start-sitl", action="store_true",
                    help="after the scene loads, start ArduPilot SITL + the router "
                         "(+ MAVROS 2 with --link mavros) in WSL "
                         "(start_ardupilot.sh), and stop them at the end")
    ap.add_argument("--wsl-distro", default="Ubuntu")
    ap.add_argument("--g1-verdict", default=None)
    ap.add_argument("--citymap", default=str(DEFAULT_CITYMAP))
    ap.add_argument("--hfov", type=float, default=None)
    ap.add_argument("--front-capture-interval-s", type=float, default=None, metavar="S",
                    help="render FrontCamera (the detector's camera, and the front frame "
                         "of --vla-front-source detector) capturing every S seconds, in "
                         f"(0, {rail.MAX_CAPTURE_INTERVAL_S:g}] (0.066 is about 15 Hz); "
                         "default the template's 0.1 s. Recorded (sim_config). An attaching "
                         "node records what the desktop's scene step was given the same "
                         "flag; what arrived is measured (vla_observation)")
    ap.add_argument("--stress", choices=list(pstress.NAMES), default=None,
                    help="a perception stressor (perception_stress.py; WP4-16, WP4-18): "
                         "day (the baseline, weather reset), dusk / night (every scene "
                         "capture exposed 2 / 4 EV down), rain / fog (the plugin's weather "
                         "effects). Rendered into the scene (its own scene id) and, for "
                         "weather, set after the load and read back; a node attaching "
                         "(--scene-mode attach) only reads it back and needs the desktop's "
                         "scene step to have been given the same stressor. Recorded "
                         "(metrics.json perception_stress) and never KPI-grade")
    # pilot (follow_vlm's defaults; cruise 8 m as the CityLife launcher flies)
    ap.add_argument("--cruise-alt", type=float, default=8.0)
    ap.add_argument("--yaw-gain", type=float, default=1.2)
    ap.add_argument("--want-width", type=float, default=0.10)
    ap.add_argument("--speed-max", type=float, default=4.0)
    ap.add_argument("--alt-gain", type=float, default=0.6)
    ap.add_argument("--climb-max", type=float, default=1.8)
    ap.add_argument("--det-thresh", type=float, default=0.02)
    ap.add_argument("--colour-min", type=float, default=0.10)
    # who flies (module docstring, THE VLA PILOT; to-do VLA-07)
    ap.add_argument("--pilot", choices=list(PILOTS), default="servo",
                    help="servo (default): follow_vlm's servo law on the detector's box; "
                         "vla: AerialVLA through the step tracker, every tick through the "
                         "Shield and the escalation FSM")
    ap.add_argument("--mission", choices=sorted({m for v in PILOT_MISSIONS.values()
                                                 for m in v}), default=None,
                    help="follow (the servo pilot's, its default); goto (the VLA's default: "
                         "--goal), approach (the detector box's phrase) or follow (the "
                         "VLA follow, demo/vla_fly.py's M3) for --pilot vla")
    ap.add_argument("--goal", default=None, metavar="X,Y",
                    help="--mission goto: the goal in the Shield's frame (x North, y East, "
                         "metres), no default; a negative X is joined by main() "
                         "(join_cli) or written --goal=-10,5")
    ap.add_argument("--vla-object", default=None,
                    help="the prompt's object words (inert for AerialVLA; AIOT-VLA's "
                         "grounding phrase; recorded as which, vla_mission."
                         "object_words_record). Default: vla_mission.GOTO_OBJECT_TEXT for "
                         "goto, --object for approach and follow")
    ap.add_argument("--arrive-radius-m", type=float, default=3.0)
    ap.add_argument("--arrive-grace-s", type=float, default=8.0)
    ap.add_argument("--overshoot-s", type=float, default=3.0)
    ap.add_argument("--close-box-frac", type=float, default=0.15)
    ap.add_argument("--lost-timeout-s", type=float, default=None,
                    help="no fresh box this long ends a detector mission (default "
                         "demo/vla_fly.py's: 15 s for goto and approach, 90 s for follow, "
                         "whose car drives out of view for long spells)")
    # THE VLA FOLLOW (module docstring): vla_fly's M3 options, its defaults
    ap.add_argument("--follow-memory-s", type=float, default=None,
                    help="follow: past the box's freshness, keep the phrase of its last "
                         "bearing this long (default vla_fly's, 6 s)")
    ap.add_argument("--follow-search-s", type=float, default=None,
                    help="follow: then turn toward the side it was last seen on this long "
                         "(default vla_fly's, 20 s)")
    ap.add_argument("--close-bottom-frac", type=float, default=None,
                    help="follow: a box whose bottom edge is this low in the image is close: "
                         "no new step, horizontal motion held (default vla_fly's, 0.92)")
    ap.add_argument("--close-width-frac", type=float, default=None,
                    help="follow: a box at least this wide (fraction of the image) is close "
                         "(default vla_fly's, 0.25)")
    ap.add_argument("--vla-step-as-rate", type=float, default=None, metavar="H",
                    help="follow: fly each step as a velocity over H seconds until the next "
                         "answer replaces it (vla_fly --step-as-rate; AIOT-VLA: its label "
                         "horizon, 0.5). Default off")
    ap.add_argument("--vla-rate-yaw", choices=("heading", "rate"), default="heading",
                    help="with --vla-step-as-rate: 'heading' turns to the capture heading + "
                         "dpsi; 'rate' flies dpsi/H as the yaw rate (AIOT-VLA's label; "
                         "vla_fly --rate-yaw)")
    ap.add_argument("--vla-front-source", choices=("vla", "detector"), default="vla",
                    help="the request's front frame: 'vla' (VLAFront, 256x256, AerialVLA's "
                         "training camera) or 'detector' (the detector's FrontCamera, "
                         "768x432, pitched -20 deg: AIOT-VLA from v3). Must be the camera "
                         "the server's hello declares: refused before arming otherwise")
    ap.add_argument("--det-fresh-s", type=float, default=DETECTION_STALE_S,
                    help="approach: a box older than this at the VLA capture is no hint "
                         "(no request)")
    ap.add_argument("--bearing-gain", type=float, default=1.0,
                    help="approach: VLABridge's gain (1.0 honest; anything else overstates "
                         "the bearing and is recorded)")
    ap.add_argument("--dead-timeout-s", type=float, default=10.0,
                    help="hovering on a dead, slow or erroring server this long ends the "
                         "mission (vla_mission.MissionMonitor)")
    ap.add_argument("--first-decision-timeout-s", type=float, default=60.0)
    ap.add_argument("--vla-server", default=None, metavar="HOST:PORT",
                    help="attach to a running protocol-v2 server; without it the node "
                         "starts demo/vla_server_v2.py, naming --vla-precision, --vla-attn "
                         "and --vla-merge / --no-vla-merge (no default)")
    ap.add_argument("--vla-port", type=int, default=8790,
                    help="the port of a server this node starts")
    ap.add_argument("--vla-python", default=None,
                    help="the started server's interpreter (default: the vla-real env "
                         "when it exists, else this one)")
    ap.add_argument("--vla-precision", choices=["bf16", "nf4"], default=None)
    ap.add_argument("--vla-attn", choices=["sdpa", "eager"], default=None)
    ap.add_argument("--vla-merge", action=argparse.BooleanOptionalAction, default=None)
    ap.add_argument("--vla-hello-timeout-s", type=float, default=2700.0,
                    help="how long to wait for the server's hello (a bf16 cold load under "
                         "RAM pressure took over 15 min; vla_mission.HELLO_TIMEOUT_S)")
    ap.add_argument("--vla-fake-output", nargs="+", default=None,
                    help="start the server's scripted stand-in instead of the model "
                         "(recorded as a stand-in, never a VLA)")
    ap.add_argument("--vla-fake-delay-s", type=float, default=0.3)
    ap.add_argument("--allow-stand-in", action="store_true",
                    help="fly against an attached server whose identity is not a VLA")
    ap.add_argument("--no-local-weights", action="store_true",
                    help="record the server's reported digest without hashing the weight "
                         "files here")
    ap.add_argument("--cadence", choices=["stop_and_go", "pipelined"], default="stop_and_go",
                    help="stop_and_go (default): upstream's, the next image only after the "
                         "step is flown and held 0.5 s; pipelined: the next image while a "
                         "step is flown")
    ap.add_argument("--vla-feed", choices=["tick", "stream"], default="tick",
                    help="tick (default): one request per 10 Hz tick at most; stream: a "
                         "new front frame also goes to the model between ticks (pipelined "
                         "only), so observations reach it faster than the loop ticks - the "
                         "grant's 10 Hz observation (X-05). Answers still enter on ticks")
    ap.add_argument("--vla-feed-max-hz", type=float, default=0.0,
                    help="--vla-feed stream: at most this many requests a second (0, the "
                         "default: no cap, every new frame once the model is free). On "
                         "the AGX Orin (2026-11-15, camera 14.7 Hz): a cap of 12 gave "
                         "10.0 frames/s while the model was asked, no cap 12.4-13.2, the "
                         "detector's rate and the tick's work unchanged")
    ap.add_argument("--vla-decision-timeout-s", type=float, default=10.0)
    ap.add_argument("--warmup-max-latency-s", type=float, default=5.0,
                    help="refuse to arm when the last warm-up inference's round trip is "
                         "slower than this (vla_mission.WARMUP_MAX_LATENCY_S)")
    ap.add_argument("--late-reply-wait-s", type=float, default=120.0,
                    help="after a request times out, wait this long for its late answer "
                         "before asking again (vla_mission.LATE_REPLY_WAIT_S)")
    ap.add_argument("--vla-cruise-mps", type=float, default=1.0)
    ap.add_argument("--vla-speed-cap-mps", type=float, default=2.0)
    ap.add_argument("--vla-cameras", action="store_true",
                    help="render AerialVLA's VLAFront / VLADown without --pilot vla: the "
                         "scene step (--scene-only) a --pilot vla node attaching to it "
                         "needs. --pilot vla renders them itself; no other run does")
    ap.add_argument("--allow-unstamped-pipelined", action="store_true",
                    help="allow --cadence pipelined on this rail, recorded: its pose ring "
                         "has no sim stamps, so an image is paired with the EKF pose at its "
                         "ARRIVAL (0.1-0.4 m and 3-9 deg off while flying pipelined)")
    ap.add_argument("--approach-radius-m", type=float, default=10.0,
                    help="approach: reached when the subject's true range (evaluation "
                         "only) ends within this")
    # the start gate and the detector's selection (StartGate, build_grounder)
    ap.add_argument("--start-when-seen", type=int, default=START_WHEN_SEEN,
                    help="hover after take-off until the detector has produced this "
                         "many fresh inferences in a row with a plausible box near "
                         "enough (follow_vlm's start gate), then start the mission "
                         "clock; with --identity follow_vlm.Reacquirer decides instead: "
                         "twice this many identity-OK boxes in a row of one object, near "
                         "enough, and seen moving (StartGate.rule); "
                         f"0 = no gate (default {START_WHEN_SEEN})")
    ap.add_argument("--start-max-range-m", type=float, default=START_MAX_RANGE_M,
                    help="the gate accepts a box only this near (depth, else the "
                         "subject width's range); 0 = any range "
                         f"(default {START_MAX_RANGE_M:g})")
    ap.add_argument("--start-timeout-s", type=float, default=START_TIMEOUT_S,
                    help=f"give up waiting after this long (default {START_TIMEOUT_S:g})")
    ap.add_argument("--start-timeout-action", choices=list(START_TIMEOUT_ACTIONS),
                    default=START_TIMEOUT_ACTIONS[0],
                    help="a gate that timed out: 'land' flies no mission (the "
                         "default), 'fly' starts it anyway; recorded either way")
    ap.add_argument("--negative-query", action="append", default=None, metavar="PHRASE",
                    help="ask PHRASE beside the subject phrase: a box that scores "
                         "higher for it (e.g. 'a traffic light') is no candidate. "
                         "Repeatable")
    ap.add_argument("--identity", action="store_true",
                    help="judge every candidate physically (demo/identity.py) at its "
                         "own frame's pose and depth; only an identity-OK box may "
                         "start the mission or steer")
    ap.add_argument("--identity-thresholds", default=None)
    ap.add_argument("--lock-gate", type=float, default=0.12,
                    help="the TargetLock's continuity gate, as a fraction of the "
                         "image width (follow_vlm's default)")
    ap.add_argument("--no-lock-target", action="store_true",
                    help="no TargetLock: the best-ranked box every inference, as "
                         "before 2026-10-21")
    ap.add_argument("--target-estimator", action="store_true",
                    help="--pilot vla --mission follow with --identity: follow_vlm's "
                         "target estimator, strict lock and re-acquisition "
                         "(demo/subject_track.SubjectTrack): the subject predicted "
                         "between boxes, the VLA's memory hint toward where it is "
                         "predicted, a lost subject taken back where it was predicted "
                         "on identity-OK evidence; est_* fields in the flight log, "
                         "metrics.json subject_track. Off: as before 2026-11-15. With "
                         "--pilot servo (since 2026-11-17): the estimator follow "
                         "(EstimatorFollowPilot, follow_vlm.fly()'s servo on the "
                         "estimate; metrics.json estimator_follow), the backend table's "
                         "C2 estimator null; without it the servo pilot is the box servo, "
                         "unchanged")
    ap.add_argument("--dry-run", action="store_true",
                    help="render configs, print the plan and the commands, exit")
    # the demo video, as follow_vlm records it (demo/recorder.py)
    ap.add_argument("--save-view", action="store_true",
                    help="record the first-person frame with the policy HUD to "
                         "<out>/view/fpv and the Chase camera to view/tps, on the "
                         "recorder's own thread, plus view/recorder.json with the "
                         "measured rate (tools/make_demo_video.py --tag <tag>)")
    ap.add_argument("--record-hz", type=float, default=20.0,
                    help="frames per second the recorder writes, off the control "
                         "loop (follow_vlm's default)")
    ap.add_argument("--record-height", type=int, default=720,
                    help="height the frames are written at; the first-person frame "
                         "is enlarged to it BEFORE the overlay is drawn")
    ap.add_argument("--live-view", action="store_true",
                    help="also show the two views in a window while flying (needs "
                         "--save-view; drawn on the recorder thread)")
    ap.add_argument("--live-height", type=int, default=480)
    ap.add_argument("--no-policy-hud", action="store_true",
                    help="with --save-view: the old one-line HUD instead of the "
                         "rule panel, the Shield / FSM banner and the minimap")
    return ap


def check_args(args) -> None:
    """Combinations that cannot work, refused before anything starts."""
    if args.camera == "ros2" and args.link != "mavros":
        raise SystemExit("--camera ros2 needs --link mavros (one ROS 2 node)")
    if getattr(args, "setpoint_via", "mavros") == "adapter" and args.link != "mavros":
        raise SystemExit("--setpoint-via adapter needs --link mavros: the MAVLink adapter "
                         "is a ROS 2 node (sitl/mavlink_adapter_node.py) that writes "
                         "MAVROS's setpoint topic")
    if args.scene_mode == "attach" and args.start_sitl:
        raise SystemExit("--scene-mode attach means the desktop already loaded the "
                         "scene and started ArduPilot after it (-Step scene, then "
                         "-Step sitl); --start-sitl here would start a second one")
    if args.scene_only and (args.camera != "projectairsim" or args.scene_mode != "load"):
        raise SystemExit("--scene-only loads the Project AirSim scene; it needs "
                         "--camera projectairsim --scene-mode load")
    if getattr(args, "live_view", False) and not getattr(args, "save_view", False):
        raise SystemExit("--live-view needs --save-view: the window draws the frames "
                         "the recorder produces, and without it none are captured")
    fci = getattr(args, "front_capture_interval_s", None)
    if fci is not None and not (math.isfinite(fci)
                                and 0.0 < fci <= rail.MAX_CAPTURE_INTERVAL_S):
        raise SystemExit(f"--front-capture-interval-s {fci!r} is not in (0, "
                         f"{rail.MAX_CAPTURE_INTERVAL_S:g}] seconds (0.066 is about 15 Hz)")
    if fci is not None and args.camera != "projectairsim":
        raise SystemExit("--front-capture-interval-s renders the simulator's FrontCamera; "
                         "--camera ros2 has none")
    if getattr(args, "stress", None) is not None and args.camera != "projectairsim":
        raise SystemExit("--stress renders and sets the Project AirSim scene's cameras "
                         "and weather; --camera ros2 has neither")
    # `not (0 < x <= max)` also refuses inf and nan (START_TIMEOUT_MAX_S).
    if getattr(args, "start_when_seen", 0) < 0 or not (
            0.0 < getattr(args, "start_timeout_s", 1.0) <= START_TIMEOUT_MAX_S):
        raise SystemExit("--start-when-seen must be >= 0 (0: no gate) and "
                         f"--start-timeout-s in (0, {START_TIMEOUT_MAX_S:g}] s: the "
                         f"airborne hover before the mission is capped "
                         f"(START_TIMEOUT_MAX_S)")
    check_pilot_args(args)
    check_estimator_args(args)


def check_estimator_args(args) -> None:
    """--target-estimator (THE TARGET ESTIMATOR, module docstring) only where
    it can run as follow_vlm runs it: a follow - the VLA's (the estimate is
    its memory hint) or the servo pilot's (EstimatorFollowPilot: follow_vlm.
    fly()'s servo on the estimate) - with --identity (the estimator is fed
    identity's range and bearing at each box's own capture, and the strict
    lock and the Reacquirer judge identity tiers), and with its strict lock
    (not --no-lock-target)."""
    if not getattr(args, "target_estimator", False):
        return
    if getattr(args, "mission", None) != "follow":
        raise SystemExit("--target-estimator is a follow's (--mission follow): the VLA's "
                         "memory hint, or the servo pilot's estimator follow")
    if not getattr(args, "identity", False):
        raise SystemExit("--target-estimator needs --identity: the estimator is fed "
                         "identity's range and bearing at each box's own capture, and the "
                         "strict lock and the re-acquisition take only identity-OK boxes")
    if getattr(args, "no_lock_target", False):
        raise SystemExit("--target-estimator holds follow_vlm's strict lock (leaning on its "
                         "estimate): drop --no-lock-target")


def _goal_xy(v) -> tuple[float, float]:
    if isinstance(v, (tuple, list)) and len(v) == 2:
        x, y = float(v[0]), float(v[1])
    else:
        try:
            x, y = (float(s) for s in str(v).split(","))
        except ValueError:
            raise SystemExit(f"--goal {v!r} is not X,Y in metres (x North, y East)") from None
    if not (math.isfinite(x) and math.isfinite(y)):
        raise SystemExit(f"--goal {v!r} is not finite")
    return x, y


def check_pilot_args(args) -> None:
    """The pilot's options (module docstring, THE VLA PILOT): refused before
    anything starts when a run could not fly them honestly, and resolved in
    place - `mission` (follow for the servo pilot, goto for the VLA's),
    `goal` (an (x, y) tuple), `vla_object` (the prompt's object words),
    `lost_timeout_s` (demo/vla_fly.py's per mission), a VLA follow's options
    (THE VLA FOLLOW: vla_fly's own resolution of them, its defaults filled
    in) and `vla_monitor` (vla_mission.MonitorConfig, which checks its own
    numbers). Called again on resolved args, it changes nothing."""
    pilot = getattr(args, "pilot", "servo")
    allowed = PILOT_MISSIONS[pilot]
    if getattr(args, "mission", None) is None:
        args.mission = allowed[0]
    if args.mission not in allowed:
        raise SystemExit(f"--mission {args.mission} is not one --pilot {pilot} flies "
                         f"({', '.join(allowed)}): goto and approach are --pilot vla's")
    if pilot != "vla":
        vla_only = [f for f, v in (("--goal", getattr(args, "goal", None)),
                                   ("--vla-server", getattr(args, "vla_server", None)),
                                   ("--vla-fake-output", getattr(args, "vla_fake_output", None)),
                                   ("--vla-precision", getattr(args, "vla_precision", None)),
                                   ("--vla-attn", getattr(args, "vla_attn", None)),
                                   ("--vla-merge", getattr(args, "vla_merge", None)),
                                   *((f"--{k.replace('_', '-')}", getattr(args, k, None))
                                     for k in VLA_FOLLOW_ONLY),
                                   ("--vla-rate-yaw",
                                    None if getattr(args, "vla_rate_yaw", "heading") == "heading"
                                    else args.vla_rate_yaw),
                                   ("--vla-front-source",
                                    None if getattr(args, "vla_front_source", "vla") == "vla"
                                    else args.vla_front_source))
                    if v is not None]
        if vla_only:
            raise SystemExit(f"{', '.join(vla_only)} configure(s) the VLA pilot: "
                             f"add --pilot vla, or drop them")
        return
    VM = _vm()
    if args.camera != "projectairsim":
        raise SystemExit("--pilot vla needs --camera projectairsim: AerialVLA's VLAFront / "
                         "VLADown come from the simulator; the ROS 2 rail's VLA is "
                         "sitl/ros2_vla_bridge_node.py")
    if args.mission == "goto":
        if args.goal is None:
            raise SystemExit("--pilot vla --mission goto needs --goal X,Y (the Shield's "
                             "frame, metres): there is no default on this rail - the spawn "
                             "has two street trees in the flight band within about a metre "
                             "of every line east (vla_mission's CityLife notes)")
        args.goal = _goal_xy(args.goal)
    elif args.goal is not None:
        raise SystemExit("--goal is for --mission goto: an approach's hint comes from the "
                         "detector box, never from a position")
    if args.vla_object is None:
        args.vla_object = VM.GOTO_OBJECT_TEXT if args.mission == "goto" else args.object
    # The server refuses object words it would not put in a prompt (a newline,
    # surrounding blanks, a position): refused here, not on the first request.
    from guardrail.aerialvla_codec import CodecError, validate_object
    try:
        validate_object(args.vla_object)
    except CodecError as exc:
        raise SystemExit(f"--vla-object {args.vla_object!r}: {exc}") from None
    cfg = [f for f, v in (("--vla-precision", args.vla_precision), ("--vla-attn", args.vla_attn),
                          ("--vla-merge/--no-vla-merge", args.vla_merge)) if v is not None]
    if args.vla_server:
        if cfg or args.vla_fake_output:
            raise SystemExit("--vla-server attaches to a running server: its configuration "
                             "is whatever it was started with (recorded from its hello); "
                             "drop --vla-precision / --vla-attn / --vla-merge / "
                             "--vla-fake-output")
        try:
            VM.parse_addr(args.vla_server)
        except ValueError as exc:
            raise SystemExit(f"--vla-server: {exc}") from None
    elif args.vla_fake_output:
        if cfg:
            raise SystemExit("--vla-fake-output starts the scripted stand-in, which has no "
                             f"model configuration: drop {', '.join(cfg)}")
    elif len(cfg) < 3:
        missing = [f for f in ("--vla-precision", "--vla-attn", "--vla-merge/--no-vla-merge")
                   if f not in cfg]
        raise SystemExit("--pilot vla starts a VLA server only with its whole configuration, "
                         "and there is no default: give --vla-precision, --vla-attn and "
                         f"--vla-merge or --no-vla-merge (missing {', '.join(missing)}), or "
                         "attach to a running one with --vla-server HOST:PORT")
    if not 0 < args.vla_port < 65536:
        raise SystemExit(f"--vla-port {args.vla_port} is not a port")
    if args.cadence == "pipelined" and not args.allow_unstamped_pipelined:
        # Review of 4a0da6d, P2-5: this rail's pose ring is fed with EKF
        # poses and no sim stamps, so a capture's time is its frame's ARRIVAL
        # and its pose the EKF interpolated there. Stop-and-go captures while
        # hovering (cm, under a degree off); pipelined captures in motion.
        raise SystemExit("--cadence pipelined on this rail pairs each image with the EKF "
                         "pose at its arrival, not its capture (no sim stamps): 0.1-0.4 m "
                         "and 3-9 deg off in flight. Fly stop_and_go, or pass "
                         "--allow-unstamped-pipelined to fly it recorded as such")
    if getattr(args, "vla_feed", "tick") == "stream":
        if args.cadence != "pipelined":
            raise SystemExit("--vla-feed stream feeds the pipelined cadence only: "
                             "stop_and_go asks after a completed step, by definition")
        if not float(args.vla_feed_max_hz) >= 0.0:
            raise SystemExit(f"--vla-feed-max-hz {args.vla_feed_max_hz} is negative")
    for k in ("vla_decision_timeout_s", "warmup_max_latency_s", "vla_cruise_mps",
              "vla_speed_cap_mps", "vla_hello_timeout_s"):
        v = getattr(args, k)
        if not (math.isfinite(v) and v > 0):
            raise SystemExit(f"--{k.replace('_', '-')} must be a positive number, got {v!r}")
    if args.vla_cruise_mps > args.vla_speed_cap_mps:
        raise SystemExit("--vla-cruise-mps may not exceed --vla-speed-cap-mps")
    if not (math.isfinite(args.late_reply_wait_s) and args.late_reply_wait_s >= 0):
        raise SystemExit("--late-reply-wait-s must be 0 or more")
    # A detector mission's lost timeout is demo/vla_fly.py's (LOST_TIMEOUT_S):
    # a follow waits for a car the level drives out of view for long spells.
    if getattr(args, "lost_timeout_s", None) is None:
        args.lost_timeout_s = float(_vf().LOST_TIMEOUT_S[args.mission])
    if args.mission != "follow":
        given = [f"--{k.replace('_', '-')}" for k in VLA_FOLLOW_ONLY
                 if getattr(args, k, None) is not None]
        if getattr(args, "vla_rate_yaw", "heading") != "heading":
            given.append("--vla-rate-yaw")
        if given:
            raise SystemExit(f"{', '.join(given)} configure(s) the VLA follow (--mission "
                             f"follow): a {args.mission} flies upstream's step on the plain "
                             f"{'goal' if args.mission == 'goto' else 'detector'} hint")
    else:
        h = args.vla_step_as_rate
        if h is not None and not (math.isfinite(h) and h > 0):
            raise SystemExit(f"--vla-step-as-rate must be a positive number of seconds, "
                             f"got {h!r}")
        if args.vla_rate_yaw != "heading" and h is None:
            raise SystemExit("--vla-rate-yaw rate flies a rate step: it needs "
                             "--vla-step-as-rate")
        for k in ("follow_memory_s", "follow_search_s"):
            v = getattr(args, k)
            if v is not None and not (math.isfinite(v) and v >= 0):
                raise SystemExit(f"--{k.replace('_', '-')} must be 0 or more seconds, got {v!r}")
        for k in ("close_bottom_frac", "close_width_frac"):
            v = getattr(args, k)
            if v is not None and not 0 < v <= 1:
                raise SystemExit(f"--{k.replace('_', '-')} must be in (0, 1], got {v!r}")
        # vla_fly's own reading of this follow: its defaults filled in here,
        # its refusals (TrackerConfig's among them) before anything starts.
        vargs, _ = vla_fly_follow(args)
        follow_tracker_config(vargs, None)
        for k, v in (("follow_memory_s", vargs.follow_memory_s),
                     ("follow_search_s", vargs.follow_search_s),
                     ("close_bottom_frac", vargs.close_bottom_frac),
                     ("close_width_frac", vargs.close_width_frac)):
            setattr(args, k, float(v))
    try:
        args.vla_monitor = VM.MonitorConfig(
            mission=args.mission, max_s=args.max_s, arrive_radius_m=args.arrive_radius_m,
            arrive_grace_s=args.arrive_grace_s, close_box_frac=args.close_box_frac,
            lost_timeout_s=args.lost_timeout_s, det_fresh_s=args.det_fresh_s,
            dead_timeout_s=args.dead_timeout_s,
            first_decision_timeout_s=args.first_decision_timeout_s,
            overshoot_s=args.overshoot_s)
    except ValueError as exc:
        raise SystemExit(f"--pilot vla: {exc}") from None


def vla_fly_follow(args) -> tuple:
    """(demo/vla_fly.py's own resolved arguments for this follow, the argv it
    parsed). THE VLA FOLLOW's configuration is vla_fly's by construction: its
    parser's defaults (the memory, the search, the close rule, the
    smoothing) and its resolve_args' refusals, never a copy here - the way
    demo/vla_dagger.py builds its follow. The argv carries this node's own
    values under vla_fly's names: the object words, the cadence, the cruise
    and cap speeds, the altitude, the decision timeout, the front source, and
    each follow option given here. SystemExit with vla_fly's reason when it
    refuses."""
    VF = _vf()
    argv = ["--mission", "follow", "--cadence", str(args.cadence),
            f"--object={args.vla_object}",
            "--cruise-mps", repr(float(args.vla_cruise_mps)),
            "--speed-cap-mps", repr(float(args.vla_speed_cap_mps)),
            "--cruise-alt", repr(float(args.cruise_alt)),
            "--decision-timeout-s", repr(float(args.vla_decision_timeout_s)),
            "--vla-front-source", str(args.vla_front_source)]
    for k, flag in (("follow_memory_s", "--follow-memory-s"),
                    ("follow_search_s", "--follow-search-s"),
                    ("close_bottom_frac", "--close-bottom-frac"),
                    ("close_width_frac", "--close-width-frac"),
                    ("vla_step_as_rate", "--step-as-rate")):
        v = getattr(args, k, None)
        if v is not None:
            argv += [flag, repr(float(v))]
    if getattr(args, "vla_rate_yaw", "heading") != "heading":
        argv += ["--rate-yaw", str(args.vla_rate_yaw)]
    try:
        return VF.resolve_args(VF.build_arg_parser().parse_args(argv)), argv
    except ValueError as exc:
        raise SystemExit(f"--pilot vla --mission follow: demo/vla_fly.py refuses this follow "
                         f"({exc})") from None
    except SystemExit:
        raise SystemExit(f"--pilot vla --mission follow: demo/vla_fly.py's parser refuses "
                         f"{' '.join(argv)}") from None


def follow_tracker_config(vargs, pilot_band):
    """The step tracker of THE VLA FOLLOW: demo/vla_fly.tracker_config on
    vla_fly's own resolved arguments (vla_fly_follow) - the follow executor
    (turn while moving, the altitude held at the cruise altitude, steps
    anchored at the answer, LAND held as arrival), its smoothing, the rate
    execution - with this rail's altitude clamp. SystemExit when the
    TrackerConfig refuses its own numbers."""
    from guardrail.vla_link import TrackerConfig
    try:
        return _vf().tracker_config(vargs, pilot_band, TrackerConfig)
    except ValueError as exc:
        raise SystemExit(f"--pilot vla --mission follow: the step tracker refuses this follow "
                         f"({exc})") from None


def make_subject_track(args, grounder, fv=None):
    """THE TARGET ESTIMATOR (--target-estimator): demo/subject_track.
    SubjectTrack over this run's Grounder, for the subject the object words
    name (follow_vlm.subject_width: the class picks the estimator's speed
    ceiling, the width ranges a box with no depth range of its own); its
    lapse and re-acquisition numbers are follow_vlm's parser defaults."""
    fv = fv or _fv()
    from subject_track import SubjectTrack
    width_m, subject_class = fv.subject_width(args.object)
    return SubjectTrack(grounder, subject_class, object_width_m=width_m, fv=fv)


def make_estimator_follow_pilot(args, grounder, hfov_deg: float, width_m, fv=None):
    """THE ESTIMATOR FOLLOW (--pilot servo --target-estimator):
    EstimatorFollowPilot over this run's Grounder and a SubjectTrack of it
    (make_subject_track), at this run's yaw gain, width target, speed cap and
    altitude hold."""
    fv = fv or _fv()
    track = make_subject_track(args, grounder, fv)
    return EstimatorFollowPilot(grounder, hfov_deg, track, object_width_m=width_m,
                                yaw_gain=args.yaw_gain, want_w_frac=args.want_width,
                                speed_max=args.speed_max, cruise_alt=args.cruise_alt,
                                alt_gain=args.alt_gain, climb_max=args.climb_max, fv=fv)


def start_subject_track(track, gate, gate_rec: dict, t0: float) -> None:
    """THE TARGET ESTIMATOR starts with the mission: committed to the start
    gate's pick (its identity features at its own capture), else - no gate,
    or none confirmed - looking for the subject by the Reacquirer with no
    anchor."""
    picked = bool(gate_rec.get("reached")) and gate is not None \
        and gate.pick_feat is not None
    track.start(gate.pick_feat if picked else None,
                gate.pick_cap if picked else None, now=t0, t0=t0)
    print("[track]  " + ("committed to the start gate's pick"
                         + (" (estimate seeded)" if track.estimator._xu is not None
                            else " (no range: not seeded)")
                         if picked else
                         "no confirmed pick: re-acquiring with no anchor"))


def subject_track_record(track, rows: list[dict]) -> dict:
    """metrics.json `subject_track`: SubjectTrack.summary(), and - read after
    the flight, EVALUATION only, never fed back - the estimate against the
    subject's truth on the rows that carry both: est_lock_frac (ticks with an
    estimate within ESTIMATE_NEAR_M of the truth / ticks whose truth was
    read: tools/perception_stress_report.py's target lock, on the estimate
    instead of the Shield's subject), est_on_subject_frac (the same / ticks
    with an estimate) and est_gap_m (|estimate - truth|). `rows` are the
    flight log's (est_xy, subject_truth, est_gap_m)."""
    rec = track.summary()
    truth = [r for r in rows if r.get("subject_truth") is not None]
    both = [r for r in truth if r.get("est_xy") is not None]
    near = sum(1 for r in both if math.hypot(r["est_xy"][0] - r["subject_truth"][0],
                                             r["est_xy"][1] - r["subject_truth"][1])
               <= ESTIMATE_NEAR_M)
    rec["truth"] = {"near_m": ESTIMATE_NEAR_M, "truth_ticks": len(truth),
                    "ticks_with_estimate": len(both),
                    "est_lock_frac": round(near / len(truth), 4) if truth else None,
                    "est_on_subject_frac": round(near / len(both), 4) if both else None,
                    "est_gap_m": spread([r["est_gap_m"] for r in both
                                         if r.get("est_gap_m") is not None])}
    return rec


def join_cli(argv) -> list[str]:
    """A negative first X of --goal joined to its flag ("--goal", "-10,5" ->
    "--goal=-10,5"), which argparse would otherwise read as an option
    (guardrail.compiler.join_negative_xy, as demo/vla_fly.py does)."""
    from guardrail.compiler import join_negative_xy
    return join_negative_xy(list(argv), ("--goal",))


def vla_server_plan(args, out: Path):
    """The VLA server this run talks to (vla_mission.VLAServer): attached
    with --vla-server, or started here naming its whole configuration
    (vla_mission.server_argv: --precision, --attn, --merge / --no-merge,
    --missing-down refuse), or the scripted stand-in (--vla-fake-output; the
    model flags are then not passed). Its stdout goes to the run folder."""
    VM = _vm()
    if args.vla_server:
        host, port = VM.parse_addr(args.vla_server)
        return VM.VLAServer(host, port)
    py = args.vla_python or (VM.VLA_REAL_PYTHON if Path(VM.VLA_REAL_PYTHON).exists()
                             else sys.executable)
    if args.vla_fake_output:
        cfg = dict(VM.FIDELITY)              # not passed with --fake-output
    else:
        cfg = {"precision": args.vla_precision, "attn": args.vla_attn,
               "merge": bool(args.vla_merge)}
    argv = VM.server_argv(py, precision=cfg["precision"], attn=cfg["attn"], merge=cfg["merge"],
                          port=args.vla_port, log=str(Path(out) / "vla_server.jsonl"),
                          fake_output=args.vla_fake_output, fake_delay_s=args.vla_fake_delay_s)
    return VM.VLAServer("127.0.0.1", args.vla_port, argv=argv,
                        log_path=Path(out) / "vla_server_stdout.log")


_NO_VLA_PROCESS = object()


def vla_host_of(identity: dict | None) -> dict | None:
    """Where the VLA server runs, by its own hello (`host`: guardrail.manifest.
    collect_host_evidence, which demo/vla_server_v2.py and
    demo/vla_server_foreign.py put there), or None when it said nothing -
    which hil refuses: a VLA that does not say where it runs cannot be shown
    to be on the Orin (manifest._vla_host_missing)."""
    host = identity.get("host") if isinstance(identity, dict) else None
    return dict(host) if isinstance(host, dict) and host else None


def note_vla_host(link, vla_host: dict | None) -> None:
    """The VLA server's host into the link's own hil evidence (MavrosLink.
    hil_evidence), as sitl/ros2_shield_node.py records a separate VLA node's
    /vla/identity host, so link_evidence and the topology evidence agree. A
    link that keeps no such dict (pymavlink, the stand-ins) is left alone:
    rail_topology_evidence adds the key to what is judged either way."""
    hil = getattr(link, "hil_evidence", None)
    if isinstance(hil, dict):
        hil["vla_host"] = None if vla_host is None else dict(vla_host)


def rail_topology_evidence(args, sim_cfg: dict | None, link,
                           vla_host=_NO_VLA_PROCESS) -> tuple[dict, dict]:
    """(the evidence topology.classify() judges, the link's own evidence):
    the camera source, the robot's controller, where the simulator runs, the
    link, the autopilot's heartbeat, the hil evidence and gate G1.

    `vla_host`: given for a VLA run (--pilot vla), whose VLA is a separate
    process, the server: its hello's host (vla_host_of), or None when it
    reported none. It goes into the hil evidence as `vla_host`, the key
    guardrail.manifest._vla_host_missing reads, so a server on another host
    (the desktop's, through a tunnel) never makes the run hil. Absent for the
    detector pilot, whose pilot runs in this process (the manifest's reading
    of an absent key)."""
    link_ev = link.evidence()
    tev = dict(link_ev.get("topology_evidence") or {})
    if vla_host is not _NO_VLA_PROCESS:
        tev["vla_host"] = None if vla_host is None else dict(vla_host)
    ev = {"camera_source": args.camera,
          "controller_type": sim_cfg["controller_type"] if sim_cfg else None,
          "sim_same_desktop": (topo.sim_on_this_desktop(args.sim_host)
                               if args.camera == "projectairsim" else None),
          "link": link.name,
          "autopilot": {k: link_ev.get(k) for k in ("heartbeat", "is_ardupilot",
                                                    "heartbeat_seen", "ardupilot_seen")},
          "topology_evidence": tev,
          "g1": g1_status(args.g1_verdict)}
    return ev, link_ev


def vla_manifest_model(VM, identity: dict | None, rec: dict) -> dict:
    """build_manifest's model arguments for a VLA run (vla_mission.
    manifest_for's rule): the server's reported digest for a real AerialVLA
    identity, else the stand-in's own fake: id."""
    if rec.get("is_vla"):
        # MODEL_ID for upstream's adapter, its own id for a follow fine-tune
        # (vla_mission.model_id_for, 2026-10-23)
        return {"model_id": VM.model_id_for(identity), "reported_digest": identity["weights_sha256"],
                "weight_paths": rec.get("weight_paths")}
    sid = str((identity or {}).get("id") or "unknown")
    return {"model_id": sid if sid.startswith("fake:") else f"fake:{sid}"}


def write_stack_record(out: Path, manifest: dict, ev: dict | None, scene_path) -> dict | None:
    """stack_revisions.json beside manifest.json (ARCH-31, guardrail.manifest
    STACK_FILE): this host's revision, the autopilot's firmware as it named
    itself (the hil evidence), and the simulator - Project AirSim's client
    version and the scene file's digest when a scene flew. Best effort: a
    failure is printed, never ends the run's records."""
    return write_stack_revisions(
        out, manifest, (ev or {}).get("topology_evidence"),
        simulator=projectairsim_simulator(scene_path) if scene_path else None,
        warn=lambda m: print(f"[node]   *** {m}"))


def _vla_refusal_records(args, out: Path, VM, exc: "PreArmRefused", *, rec, policy,
                         policy_source: dict, scene_path, sim_cfg, link, backend, srv,
                         identity, vla_rec: dict, warmup, frames, saver,
                         clock_record: dict) -> dict:
    """A run refused in the pre-arm step (the VLA's warm-up gate), on record
    as the city rail records one (review of 4a0da6d, P2-3; until then it left
    events.jsonl alone, which the Dashboard does not read): the episode's
    mission_end, metrics.json (end_reason the refusal's kind, the warm-up,
    the pilot record, the server's record - this closes the VLA side and
    writes vla_captures.jsonl, _vla_records) and manifest.json, stamped with
    the rail's own topology. Nothing was armed: no rates, dwell or KPI table
    exist. Returns the metrics."""
    why = f"refused before arming: {exc.kind}: {exc}"
    try:
        rec.end(why)
    except Exception as e:                                    # noqa: BLE001
        print(f"[node]   *** mission_end not written: {e}")
    ev, _ = rail_topology_evidence(args, sim_cfg, link, vla_host=vla_rec.get("vla_host"))
    cls = topo.classify(ev)
    metrics = {"tag": out.name, "rail": "projectairsim-ardupilot", "episode_id": rec.episode_id,
               "topology": cls["stamp"], "grant_topology": cls["grant"],
               "topology_evidence": ev, "shield": args.shield, "ticks": 0,
               "end_reason": exc.kind, "refused": {"kind": exc.kind, "why": str(exc)},
               "not_measured": "no mission flown: " + why,
               "policy_source": policy_source, "rule_clock": clock_record, "sim_config": sim_cfg,
               "seed": args.seed,
               "params": {k: getattr(args, k, None) for k in (
                   "object", "cruise_alt", "seed", "link", "camera", "scene_mode", "pilot",
                   "mission", "goal", "vla_object", "cadence", "vla_decision_timeout_s",
                   "first_decision_timeout_s", "warmup_max_latency_s", "late_reply_wait_s",
                   "stress", *VLA_PARAMS)}}
    try:
        metrics.update(_vla_records(args, out, VM, None, backend, srv, identity, vla_rec,
                                    warmup, frames, saver, [], exc.kind))
    except Exception as e:                                    # noqa: BLE001
        metrics["vla_records_error"] = f"{type(e).__name__}: {e}"
    metrics["end_reason_means"] = VM.NOT_FLOWN_ENDS.get(exc.kind)
    manifest = build_manifest(policy.policy_hash, seed=args.seed, scene_path=scene_path,
                              topology=cls["stamp"], **vla_manifest_model(VM, identity, vla_rec))
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    write_stack_record(out, manifest, ev, scene_path)
    print(f"[node]   *** REFUSED before arming ({exc.kind}): {exc}; metrics.json and "
          f"manifest.json say so")
    return metrics


def gpu_memory(run_cmd=subprocess.run, timeout_s: float = 5.0) -> dict:
    """The GPU's memory in MiB (the first GPU nvidia-smi lists: free, used,
    total), best effort, for the VLA run's record (review of 4a0da6d, P2-7:
    bf16 AerialVLA ~17 GB + OWL-ViT + Unreal on the 24 GB card is not
    measured). nvidia-smi, never a GPU library in the flight process; any
    failure is recorded as {"error": why}, never a number."""
    cmd = ["nvidia-smi", "--query-gpu=memory.free,memory.used,memory.total",
           "--format=csv,noheader,nounits"]
    try:
        p = run_cmd(cmd, capture_output=True, text=True, timeout=timeout_s, check=False)
    except Exception as exc:                                  # noqa: BLE001
        return {"error": f"{type(exc).__name__}: {exc}"}
    if getattr(p, "returncode", 1) != 0:
        return {"error": f"nvidia-smi exited {getattr(p, 'returncode', None)}: "
                         f"{str(getattr(p, 'stderr', '') or '').strip()[:200]}"}
    lines = [ln for ln in str(getattr(p, "stdout", "") or "").splitlines() if ln.strip()]
    try:
        free, used, total = (int(float(v)) for v in lines[0].split(","))
    except (IndexError, ValueError):
        return {"error": f"nvidia-smi said {lines[:1]!r}"}
    return {"free_mib": free, "used_mib": used, "total_mib": total, "gpus": len(lines),
            "source": "nvidia-smi"}


def approach_scores(rows: list[dict], radius_m: float, *, scored: bool) -> dict:
    """The approach's mission score (review of 4a0da6d, P3-11): from the
    subject's TRUE range on each row (subject_range_m, SubjectTruth's,
    evaluation only - it never reached the hint or the stop rule), reached
    when the last read range is within `radius_m`. None, with why, when the
    truth did not score (too few ticks read, or its frame disagrees)."""
    if not scored:
        return {"reached_goal": None,
                "reached_goal_why": "the subject's simulator pose did not score this run "
                                    "(subject_truth: too few ticks read, or its frame "
                                    "disagrees), so the approach has no score"}
    d = [float(r["subject_range_m"]) for r in rows if r.get("subject_range_m") is not None]
    if not d:
        return {"reached_goal": None, "reached_goal_why": "no row read the subject's range"}
    return {"target_truth_dist_start_m": round(d[0], 2), "target_truth_dist_end_m": round(d[-1], 2),
            "target_truth_dist_min_m": round(min(d), 2), "reached_radius_m": radius_m,
            "reached_goal": bool(d[-1] <= radius_m)}


def obs_frames_received(obs) -> int | None:
    """The FrontCamera messages the node's frame buffer has been handed
    (demo/semantic_demo.SemanticObs.stats()["frames_received"]: every
    put_front, the detector's camera), or None when it cannot say."""
    fn = getattr(obs, "stats", None)
    if not callable(fn):
        return None
    try:
        n = (fn() or {}).get("frames_received")
    except Exception:                                         # noqa: BLE001
        return None
    return int(n) if isinstance(n, (int, float)) and not isinstance(n, bool) else None


def _p50_p95_max(vals) -> dict:
    """{n, p50, p95, max} of a list of seconds (None for each when empty)."""
    v = [float(x) for x in vals]
    return {"n": len(v), "p50": percentile(v, 50), "p95": percentile(v, 95),
            "max": None if not v else round(max(v), 4)}


# The grant's monitor tick budget, "<= 100 ms (10 Hz) end-to-end" (Safety
# Shield p6): what the camera-to-setpoint figures are counted against.
TICK_BUDGET_MS = 100.0


def publish_latency_fields(t_tick: float, t_pub: float | None, t0: float) -> dict:
    """The row's setpoint publication on the loop clock (CAMERA TO SETPOINT
    in the module docstring): `t_pub` the mission time (s) at which this
    tick's setpoint went out - read right after link.send returned - and
    `tick_ms` the tick's start to it (the hil Shield node's field of the
    same name). Both None on a tick that sent nothing."""
    if t_pub is None:
        return {"t_pub": None, "tick_ms": None}
    return {"t_pub": round(t_pub - t0, 4), "tick_ms": round((t_pub - t_tick) * 1000.0, 3)}


def camera_age_fields(t_pub: float | None, t_capture, *, flies: bool, fresh: bool) -> dict:
    """The age of the camera frame behind the flown action at its
    publication, on the loop clock: `cam_age_ms` = t_pub - the capture time
    of the VLA decision whose action flew (vla_steps.jsonl `t_capture`, its
    request's frame), on every tick that decision's action flew; and
    `cam_to_setpoint_ms`, the same on the FIRST tick that flies it (`fresh`)
    - camera input to action output. None where nothing was published, the
    pilot's action did not fly (the FSM's brake, a stop it advised, a hold,
    a fault, a takeover) or no decision with a capture time was behind it."""
    if (t_pub is None or not flies or not isinstance(t_capture, (int, float))
            or isinstance(t_capture, bool) or not math.isfinite(t_capture)):
        return {"cam_age_ms": None, "cam_to_setpoint_ms": None}
    age = round((t_pub - float(t_capture)) * 1000.0, 3)
    return {"cam_age_ms": age, "cam_to_setpoint_ms": age if fresh else None}


def _nearest_rank(vals: list[float], q: float):
    """Nearest-rank percentile (tools/kpi_report.py's), None when empty."""
    if not vals:
        return None
    s = sorted(vals)
    return round(s[min(len(s) - 1, max(0, math.ceil(q * len(s)) - 1))], 3)


def camera_to_setpoint_summary(rows: list[dict], *, t_capture_basis: dict | None = None,
                               budget_ms: float = TICK_BUDGET_MS) -> dict:
    """metrics.json `camera_to_setpoint`: n, p50, p95, p99, max (nearest
    rank, as tools/kpi_report.py pools them) and the ticks over the grant's
    100 ms of the rows' `cam_age_ms`, `cam_to_setpoint_ms` and `tick_ms`,
    the authors behind the camera figures, and the clock they were read on
    (module docstring, CAMERA TO SETPOINT). `t_capture_basis`: how the sent
    captures were timed (vla_mission.VLAFrames.stats), "arrival" on this
    rail. A measure no row carries has n 0 and None figures, never 0 ms."""
    out: dict = {"budget_ms": budget_ms}
    for key in ("cam_age_ms", "cam_to_setpoint_ms", "tick_ms"):
        v = [float(r[key]) for r in rows if isinstance(r.get(key), (int, float))
             and not isinstance(r.get(key), bool)]
        out[key] = {"n": len(v), "p50": _nearest_rank(v, 0.5), "p95": _nearest_rank(v, 0.95),
                    "p99": _nearest_rank(v, 0.99),
                    "max": round(max(v), 3) if v else None,
                    "over_budget": sum(1 for x in v if x > budget_ms)}
    out["by_author"] = dict(collections.Counter(
        str(r.get("author")) for r in rows if r.get("cam_age_ms") is not None))
    out["t_capture_basis"] = t_capture_basis
    out["clock"] = ("one clock, the loop's (time.time unless injected): the frame's "
                    "arrival stamp (VLAFrames.put_front), the request's t_capture / "
                    "t_sent / t_received, the tick's start and the publication (read "
                    "right after link.send returned) all read it")
    out["capture_is"] = ("the frame's ARRIVAL at this node, not its render: this rail's "
                         "pose ring carries no simulator stamps (t_capture_basis "
                         "'arrival'), so render and transport before arrival are not in "
                         "cam_*: a lower bound of camera to action")
    out["not_in_it"] = ("the autopilot's receipt of the setpoint (link.send returns on "
                        "the hand-off to MAVROS / pymavlink); the frame's simulator "
                        "stamp_ns is another clock and is not mixed in")
    return out


def vla_observation(captures: list[dict], rows: list[dict], *, t0: float,
                    span_s: float | None, front_source: str, camera: str,
                    camera_frames: int | None = None, capture_interval_s: float | None = None,
                    tick_s: float = TICK, grant_hz: float = GRANT_OBS_HZ) -> dict:
    """How often a NEW observation reached the VLA (metrics.json
    `vla_observation`; the grant's "Camera / IMU / GPS - 10 Hz observation",
    X-05, ARCH-25), measured from what was sent, never the decision rate
    (answers parsed) or the request rate (a request can re-send a frame
    already sent).

    `captures`: vla_mission.VLAFrames.log (vla_captures.jsonl), one line per
    request sent with its front frame's sim stamp (`stamp_ns`) and capture
    wall time (`t_capture`); warm-up lines are left out. `rows`: the flight's
    rows, whose `vla_request` (VLAPilot: sent / no_hint / no_capture / error /
    None) and `pilot_held` say what each tick did. `t0`: the mission's start
    (wall); `span_s`: the loop's own span (its first tick to its last, plus
    one tick).

      distinct_frames   distinct stamps among the requests sent;
                        duplicate_requests those that re-sent one; a request
                        whose frame had no stamp is counted apart, never as
                        distinct.
      hz_per_wall_s     distinct_frames over the mission's wall seconds.
      hz_while_asking   ... over the seconds the pilot was being asked: not
                        held by the rail (pilot_held) and not HINT-GATED.
      hint_gated        the spans in which requests were withheld because the
                        hint said none (no fresh box past the follow's memory
                        and search, a close box): from a tick whose capture
                        found no hint to the next tick that sent, each with
                        why. The VLA was not asked then by design; the
                        aircraft held or hovered on the tracker's last step.
      hz_per_sim_s      (distinct_frames - 1) over the sim seconds between
                        the first and last frame sent; sim_per_wall the same
                        span's sim seconds per wall second (the renderer's
                        clock).
      gap_sim_s, gap_wall_s  the gaps between consecutive distinct frames
                        (median = p50, p95, max), by stamp and by capture
                        time.
      camera_delivery   FrontCamera messages delivered to this node over the
                        mission (`camera_frames`, SemanticObs) and their rate:
                        the ceiling when the front source is the detector's
                        camera; `capture_interval_s` the interval its scene
                        was rendered with.
    Each tick counts for the time to the next row, at most two ticks;
    `unrowed_s` is the rest of the span (ticks with no live state)."""
    sent = [c for c in captures if isinstance(c, dict) and not c.get("warmup")]
    stamped = [c for c in sent if c.get("stamp_ns") is not None]
    first_wall: dict[float, float | None] = {}
    for c in stamped:
        s = float(c["stamp_ns"])
        if s not in first_wall:
            t = c.get("t_capture")
            first_wall[s] = None if t is None else float(t)
    stamps = sorted(first_wall)
    n_dist = len(stamps)
    gaps_sim = [(b - a) / 1e9 for a, b in zip(stamps, stamps[1:])]
    walls = [first_wall[s] for s in stamps if first_wall[s] is not None]
    gaps_wall = [b - a for a, b in zip(walls, walls[1:])]
    sim_span = (stamps[-1] - stamps[0]) / 1e9 if n_dist >= 2 else None
    wall_span = (walls[-1] - walls[0]) if len(walls) >= 2 else None
    held_s = gated_s = asking_s = 0.0
    spans: list[dict] = []
    cur: dict | None = None
    gated = False
    for i, r in enumerate(rows):
        t = float(r.get("t") or 0.0)
        nxt = float(rows[i + 1].get("t") or t) if i + 1 < len(rows) else t + tick_s
        d = max(0.0, min(nxt - t, 2.0 * tick_s))
        req = r.get("vla_request")
        if req == "sent":
            gated = False
        elif req == "no_hint":
            gated = True
        if r.get("pilot_held"):
            held_s += d
            kind = "held"
        elif gated:
            gated_s += d
            kind = "gated"
        else:
            asking_s += d
            kind = "asking"
        if kind == "gated":
            if cur is None:
                cur = {"t": round(t, 3), "s": 0.0, "ticks": 0,
                       "why": collections.Counter()}
            cur["s"] += d
            cur["ticks"] += 1
            if req == "no_hint":
                cur["why"][r.get("vla_no_hint_why") or "unknown"] += 1
        elif cur is not None:
            spans.append(cur)
            cur = None
    if cur is not None:
        spans.append(cur)
    why_all: collections.Counter = collections.Counter()
    for sp in spans:
        why_all.update(sp["why"])
    span = float(span_s) if span_s else None

    def rate(n, s):
        return round(n / s, 3) if s else None
    hz_wall, hz_ask = rate(n_dist, span), rate(n_dist, asking_s)
    return {
        "basis": ("distinct front-frame sim stamps among the requests actually sent "
                  "(vla_captures.jsonl stamp_ns; warm-up excluded): how often a NEW "
                  "observation reached the VLA. Not the decision rate (pilot.summary."
                  "decision_hz, answers parsed) and not the request rate"),
        "front_source": front_source, "camera": camera, "grant_obs_hz": grant_hz,
        "requests_sent": len(sent), "requests_with_stamp": len(stamped),
        "requests_without_stamp": len(sent) - len(stamped),
        "distinct_frames": n_dist, "duplicate_requests": len(stamped) - n_dist,
        "duplicate_share": (round((len(stamped) - n_dist) / len(stamped), 4)
                            if stamped else None),
        "mission_wall_s": None if span is None else round(span, 3),
        "hz_per_wall_s": hz_wall,
        "asking_s": round(asking_s, 3), "held_s": round(held_s, 3),
        "unrowed_s": (None if span is None else
                      round(max(0.0, span - asking_s - held_s - gated_s), 3)),
        "hz_while_asking": hz_ask,
        "hint_gated": {"spans": len(spans), "s": round(gated_s, 3),
                       "longest_s": (round(max(sp["s"] for sp in spans), 3)
                                     if spans else None),
                       "why_ticks": dict(why_all),
                       "first": [{"t": sp["t"], "s": round(sp["s"], 3), "ticks": sp["ticks"],
                                  "why": dict(sp["why"])} for sp in spans[:20]]},
        "sim_span_s": None if sim_span is None else round(sim_span, 3),
        "wall_span_s": None if wall_span is None else round(wall_span, 3),
        "sim_per_wall": (round(sim_span / wall_span, 4)
                         if sim_span is not None and wall_span else None),
        "hz_per_sim_s": (round((n_dist - 1) / sim_span, 3)
                         if sim_span else None),
        "gap_sim_s": _p50_p95_max(gaps_sim),
        "gap_wall_s": _p50_p95_max(gaps_wall),
        "below_grant": {"per_wall_s": None if hz_wall is None else hz_wall < grant_hz,
                        "while_asking": None if hz_ask is None else hz_ask < grant_hz},
        "camera_delivery": {"camera": "FrontCamera", "frames": camera_frames,
                            "hz": (None if camera_frames is None else
                                   rate(camera_frames, span)),
                            "capture_interval_s": capture_interval_s,
                            "note": ("the detector's camera: the VLA's front frame too"
                                     if front_source == "detector" else
                                     "the detector's camera; the VLA's front frame is "
                                     "VLAFront, not counted here")},
        "t0_wall": round(float(t0), 3),
    }


def _vla_bring_up(args, out: Path, VM, srv, policy, shield_on: bool, grounder, hfov: float,
                  frames, live, clock, client=None, run_cmd=subprocess.run) -> tuple:
    """The VLA side up to the flight: the server's hello (waited for up to
    --vla-hello-timeout-s, the heartbeat beating), its identity judged
    (vla_mission.identity_is_vla: a stand-in only with --vla-fake-output or
    --allow-stand-in), the protocol-v2 client connected with that identity
    pinned, the hint source (the goal's phrase, or the detector box's
    through demo/vla_bridge), the pilot's altitude clamp
    (vla_mission.pilot_alt_config: the policy's band with the Shield on, the
    simulator safety band with it off) and the backend (RemoteVLABackend:
    the step tracker, the cadence, vla_steps.jsonl, the frames kept on each
    request sent). A FOLLOW's hint and tracker are demo/vla_fly.py's
    (THE VLA FOLLOW: DetectorHint with the memory, the search and the close
    rule; vla_fly.tracker_config). The record carries the server's host
    (vla_host_of: the run's `vla_host` evidence), the front-camera check
    and what the object words are to this model. Returns (identity,
    backend, record); SystemExit on a refusal. `client`: an injected
    RemoteVLA (tests)."""
    from guardrail.aerialvla_codec import FRONT_SOURCES
    from guardrail.vla_backends import make_backend
    from guardrail.vla_link import RemoteVLA, TrackerConfig, identity_of
    said = {"t": 0.0}

    def _on_wait(w: float) -> None:
        live.beat(detail=f"waiting for the VLA server ({w:.0f} s)")         # memory only
        if w - said["t"] >= 60.0:
            said["t"] = w
            print(f"[vla]    still waiting for the server's hello: {w:.0f} s of "
                  f"{args.vla_hello_timeout_s:g} s"
                  + ("" if srv.alive else " - the process has exited"), flush=True)
    live.beat(detail="waiting for the VLA server")                         # memory only
    try:
        identity = srv.wait_hello(args.vla_hello_timeout_s, on_wait=_on_wait)
    except (TimeoutError, VM.ServerExited) as exc:
        raise SystemExit(f"[vla]    *** no VLA server: {type(exc).__name__}: {exc}") from None
    is_vla, why_not = VM.identity_is_vla(identity)
    if not is_vla and not (args.vla_fake_output or args.allow_stand_in):
        raise SystemExit(f"[vla]    *** the server's identity is not AerialVLA ({why_not}); "
                         f"pass --allow-stand-in to fly a stand-in, recorded as one")
    print(f"[vla]    hello: {identity_of(identity)}"
          + ("" if is_vla else f"  *** STAND-IN, not a VLA: {why_not}"))
    # Where the server runs, by its own hello: the run's vla_host evidence
    # (rail_topology_evidence), as the ROS rail records a VLA node's host.
    vla_host = vla_host_of(identity)
    print(f"[vla]    server host: "
          + ("NOT REPORTED (no 'host' in its hello): this run cannot be hil"
             if vla_host is None else
             f"{vla_host.get('device_model') or 'no device model'} "
             f"({vla_host.get('host_arch')})"))
    # The front camera this flight sends (--vla-front-source) must be the one
    # the server reads: AerialVLA reads VLAFront, a foreign server
    # (demo/vla_server_foreign.py) the camera its hello declares
    # (vla_mission.front_source_check).
    front_cam = FRONT_SOURCES[args.vla_front_source]["camera"]
    front_check = VM.front_source_check(identity, args.vla_front_source, is_vla=is_vla)
    if not front_check["ok"]:
        raise SystemExit(f"[vla]    *** refused before take-off: {front_check['why']} (this "
                         f"flight sends {front_cam}: --vla-front-source "
                         f"{args.vla_front_source})")
    print(f"[vla]    front image {front_cam} (--vla-front-source {args.vla_front_source})"
          + (f", the server's ({front_check['source']})" if front_check["checked"]
             else f" - {front_check['source']}"))
    words = VM.object_words_record(identity)
    # The GPU's memory with the model loaded (review of 4a0da6d, P2-7).
    gpu = {"at_hello": gpu_memory(run_cmd)}
    weight_paths = (None if (args.no_local_weights or not is_vla)
                    else VM.local_weight_paths(identity))
    if client is None:
        client = RemoteVLA(srv.host, srv.port, response_timeout_s=args.vla_decision_timeout_s,
                           strict_identity=is_vla, late_reply_wait_s=args.late_reply_wait_s,
                           clock=clock)
    try:
        try:
            client.connect()                   # pins the identity before the flight
        except Exception as exc:                              # noqa: BLE001
            raise SystemExit(f"[vla]    *** the link failed before take-off: {exc}") from None
        if identity_of(client.identity) != identity_of(identity):
            raise SystemExit("[vla]    *** the pinned identity is not the hello's")
        det_hint = close_rule = follow_rec = track = None
        follow = args.mission == "follow"
        vargs = vla_fly_follow(args)[0] if follow else None
        if follow and getattr(args, "target_estimator", False):
            # THE TARGET ESTIMATOR (module docstring), on this run's Grounder;
            # started at the mission's start with the start gate's pick.
            track = make_subject_track(args, grounder)
        if args.mission == "goto":
            hint_src = VM.goto_hint_source(args.goal, args.arrive_radius_m)
        else:
            from vla_bridge import VLABridge
            bridge = VLABridge(args.object, hfov_deg=hfov, bearing_gain=args.bearing_gain)
            if follow:
                # vla_fly's M3 hint: the sighting's memory, the search turn,
                # the close rule; each hint records its kind. With the target
                # estimator the memory aims at the predicted subject first.
                close_rule = VM.BoxClose(vargs.close_bottom_frac, vargs.close_width_frac)
                det_hint = VM.DetectorHint(grounder_box_source(grounder), bridge,
                                           fresh_s=args.det_fresh_s,
                                           memory_s=vargs.follow_memory_s,
                                           search_s=vargs.follow_search_s,
                                           close=close_rule, record_kind=True,
                                           estimate=None if track is None else track.hint_memory)
            else:
                det_hint = VM.DetectorHint(grounder_box_source(grounder), bridge,
                                           fresh_s=args.det_fresh_s)
            hint_src = VM.HintSource(det_hint, "detector")
        backend_policy, pilot_band, band_rec = VM.pilot_alt_config(shield_on, policy)
        if follow:
            # vla_fly's follow tracker (the executor, its smoothing, the rate
            # execution), with this rail's altitude clamp.
            tcfg = follow_tracker_config(vargs, pilot_band)
            vf = _vf()
            follow_rec = {"executor": bool(vargs.follow_executor),
                          "smooth": vf.smooth_record(vargs),
                          "step_as_rate_s": vargs.step_as_rate, "rate_yaw": vargs.rate_yaw,
                          "memory_s": vargs.follow_memory_s, "search_s": vargs.follow_search_s,
                          "close_rule": close_rule.as_dict(),
                          "lost_timeout_s": args.lost_timeout_s,
                          "from": ("demo/vla_fly.py: its parser and resolve_args on this "
                                   "flight's values, then vla_fly.tracker_config "
                                   "(perception_node.vla_fly_follow)"),
                          # --target-estimator: SubjectTrack's configuration
                          # (None: the hint's memory is the sighting's alone).
                          "target_estimator": None if track is None else track.config()}
        else:
            tcfg = TrackerConfig(cruise_mps=args.vla_cruise_mps,
                                 speed_cap_mps=args.vla_speed_cap_mps,
                                 decision_timeout_s=args.vla_decision_timeout_s,
                                 alt_band=pilot_band)
        backend = make_backend("aerovla", None, backend_policy, get_capture=frames.capture,
                               get_hint=hint_src, client=client, object_text=args.vla_object,
                               tracker_config=tcfg, cadence=args.cadence,
                               steps_log=out / "vla_steps.jsonl", on_submit=frames.commit,
                               clock=clock,
                               **({"new_frame": frames.front_token,
                                   "feed_max_hz": (float(args.vla_feed_max_hz) or None)}
                                  if getattr(args, "vla_feed", "tick") == "stream" else {}))
    except BaseException:
        try:
            client.close()
        except Exception:                                     # noqa: BLE001
            pass
        raise
    alt_band = {**band_rec, "applied": dict(backend.alt_band_info)}
    print(f"[vla]    pilot: mission {args.mission}, hint {hint_src.kind}, cadence "
          f"{args.cadence}, altitude clamp {backend.alt_band_info.get('band')} "
          f"({backend.alt_band_info.get('source')}); object words {args.vla_object!r} ("
          + ("inert" if words["object_words_are_inert"] else words["object_words_role"]) + ")")
    if follow_rec is not None:
        sm = follow_rec["smooth"]
        print(f"[vla]    follow: memory {follow_rec['memory_s']:g} s, search "
              f"{follow_rec['search_s']:g} s, close {follow_rec['close_rule']}; executor "
              f"{'on' if follow_rec['executor'] else 'off'}, smoothing "
              f"{'on' if sm.get('on') else 'off'}"
              + (f"; steps flown as rates over {follow_rec['step_as_rate_s']:g} s (yaw "
                 f"{follow_rec['rate_yaw']})" if follow_rec["step_as_rate_s"] else "")
              + f"; lost after {args.lost_timeout_s:g} s")
        if track is not None:
            c = track.config()
            print(f"[track]  target estimator ON ({c['subject_class'] or 'unclassified'}, "
                  f"v_max {c['v_max_mps']}): strict lock on its estimate, lapse after "
                  f"{c['lapse_s']:g} s, re-acquired on {c['reacq_need']} identity-OK "
                  f"sightings within {c['reacq_max_range_m']:g} m; the memory hint aims at "
                  f"the predicted subject")
    return identity, backend, {"is_vla": is_vla, "why_not": why_not,
                               "weight_paths": weight_paths, "client": client,
                               "hint_source": hint_src, "detector_hint": det_hint,
                               "track": track,
                               "close_rule": close_rule, "follow": follow_rec,
                               "front_source": front_check, "object_words": words,
                               "vla_host": vla_host,
                               "pilot_alt_band": alt_band, "gpu_memory": gpu,
                               "tracker": {k: getattr(tcfg, k)
                                           for k in tcfg.__dataclass_fields__}}


def _vla_records(args, out: Path, VM, vla, backend, srv, identity, rec: dict, warmup,
                 frames, saver, rows: list[dict], end_reason: str, *,
                 gate_rec: dict | None = None, detector_id: str | None = None,
                 approach: dict | None = None) -> dict:
    """metrics.json's VLA part, the city rail's records (demo/vla_mission):
    `pilot` (pilot_entry: guardrail.manifest.pilot_record kind "vla" with the
    server's reported digest only for a real AerialVLA identity, else kind
    "stub"; the backend's stats under "vla", the summary, the server), `vla`
    (the mission, its hint, the stop rules' summary, the warm-up, the
    altitude clamp, the ticks the rail held, the frames), `land` and the
    mission's score (vla_mission.land_record, mission_scores). Closes the
    VLA side first - the pilot (its stats frozen at the mission's end), the
    frame writer, a server this run started - so every vla_steps.jsonl line
    is written, and writes vla_captures.jsonl (the captures sent, warm-up
    ones marked)."""
    if vla is not None:
        stats = vla.close()
    elif backend is not None:
        stats = backend.stats()
        backend.close()
    else:
        stats = {}
    records = list(backend.records) if backend is not None else []
    server_rec = srv.stop() if srv is not None else {}      # idempotent; attached: no-op
    saved = saver.close() if saver is not None else None
    VM.write_jsonl(out / "vla_captures.jsonl", list(getattr(frames, "log", None) or []))
    goto = args.mission == "goto"
    goal = args.goal if goto else None
    hint_src, det_hint = rec.get("hint_source"), rec.get("detector_hint")
    is_vla = bool(rec.get("is_vla"))
    pilot = VM.pilot_entry(
        identity, stats, records, rows, server=server_rec,
        reported_by=None if srv is None else f"{srv.host}:{srv.port}",
        weight_paths=rec.get("weight_paths"), cadence=args.cadence, warmup=warmup,
        # The configuration this run asked for (None when attached: then the
        # server's own identity says what it runs).
        vla_config=({"precision": args.vla_precision, "attn": args.vla_attn,
                     "merge": bool(args.vla_merge)}
                    if srv is not None and srv.started_here and not args.vla_fake_output
                    else None),
        vla_fake_output=args.vla_fake_output, stand_in=not is_vla)
    if not is_vla:
        pilot["warning"] = ("a scripted stand-in answered: this run tests the flight path, "
                            "it is NOT a VLA flight")
    # What the object words were to the model that answered (THE VLA FOLLOW):
    # AerialVLA's inert slot, AIOT-VLA's grounding phrase.
    words = rec.get("object_words") or VM.object_words_record(identity)
    # THE TARGET ESTIMATOR (a follow's; None for goto and approach): its
    # record and its faults, or why there was none - never silence.
    track = rec.get("track")
    if args.mission != "follow":
        track_rec = None
    elif track is None:
        track_rec = {"on": False, "why": "--target-estimator not given: the hint's memory is "
                                         "the remembered sighting's, no estimate is kept"}
    else:
        try:
            track_rec = subject_track_record(track, rows)
        except Exception as exc:                                  # noqa: BLE001
            track_rec = {"on": True, "record_error": f"{type(exc).__name__}: {exc}"}
        track_rec["errors"] = None if vla is None else vla.track_errors
        track_rec["first_error"] = None if vla is None else vla.track_error
    return {
        "subject_track": track_rec,
        "end_reason_means": VM.END_REASONS.get(end_reason),
        "pilot": pilot,
        "vla": {"mission": {"kind": args.mission, "goal_xy": None if goal is None else list(goal),
                            "object": args.vla_object,
                            "object_words_are_inert": words["object_words_are_inert"],
                            "object_words_role": words["object_words_role"],
                            "object_is_default": goto and args.vla_object == VM.GOTO_OBJECT_TEXT,
                            "hint_source": None if hint_src is None else hint_src.kind,
                            "hint": None if hint_src is None else hint_src.stats(),
                            "detector_hint": None if det_hint is None else det_hint.stats(),
                            "bearing_gain": None if goto else args.bearing_gain,
                            # What ran (review of 4a0da6d, P3-10): the
                            # detector's own id, None when none ran.
                            "detector": detector_id,
                            "monitor": None if vla is None else vla.monitor.summary()},
                "warmup": warmup,
                "pilot_alt_band": rec.get("pilot_alt_band"),
                "tracker": rec.get("tracker"),
                # THE VLA FOLLOW's configuration as flown (None for goto and
                # approach), and the front-camera check against the hello.
                "follow": rec.get("follow"),
                "close_hold_ticks": (None if vla is None or vla.close_hold is None
                                     else vla.n_close_hold),
                "front_source": rec.get("front_source"),
                # The server's host, by its hello (the run's vla_host).
                "server_host": rec.get("vla_host"),
                "cadence": args.cadence,
                "allow_unstamped_pipelined": bool(getattr(args, "allow_unstamped_pipelined",
                                                          False)),
                # Ticks on which the pilot's action did not fly (the FSM's
                # brake, a stop it advised, a hold, a fault, a takeover): the
                # tracker was suspended through them; stale-link spans held it
                # too; and ticks on which the FSM stepped on the suspended
                # tracker's zero (its recovery timer ran on a zero the pilot
                # never chose; review of 4a0da6d, P3-13).
                "held_ticks": None if vla is None else vla.n_held,
                "stale_link_holds": None if vla is None else vla.n_stale_holds,
                "fsm_ticks_on_zeroed_raw": None if vla is None else vla.n_fsm_on_zeroed,
                # What the start gate did (P3-10: it said "kept" with
                # --start-when-seen 0).
                "start_gate": ("skipped: a GOTO has no detected subject" if goto else
                               "ran: the detector-subject gate (--start-when-seen "
                               f"{(gate_rec or {}).get('needed')})"
                               if (gate_rec or {}).get("enabled") else
                               "not run: --start-when-seen 0"),
                # The GPU's memory at the hello and after the warm-up (P2-7).
                "gpu_memory": rec.get("gpu_memory"),
                "frames": {"capture": frames.stats() if callable(getattr(frames, "stats", None))
                           else None, "saved": saved,
                           # How a capture's time and pose were taken on this
                           # rail (P2-5): its pose ring carries EKF poses and no
                           # sim stamps.
                           "capture_time_basis": (
                               "arrival: this rail's pose ring carries EKF poses with no "
                               "sim stamps, so a capture's time is its frame's arrival "
                               "and its pose the EKF interpolated there (stop-and-go "
                               "captures while hovering)")}},
        "land": VM.land_record(rows, records, goal),
        # A GOTO is scored on its goal, an approach on the subject's true
        # range; a follow on the follow proxy (frac_within_30m, the caller's)
        # and declares no goal.
        **(VM.mission_scores(args.mission, rows, goal_xy=goal,
                             arrive_radius_m=args.arrive_radius_m) if goto else
           (approach or {"reached_goal": None})),
    }


def sim_configs(args, out: Path):
    """(sim_cfg, plan) for this run: rendered with today's addresses when
    this node loads the scene, the template marked "attach" when it does not."""
    if args.camera != "projectairsim":
        return None, None
    # AerialVLA's cameras are rendered for a VLA run only, or for a scene
    # step an attaching VLA node will use (--vla-cameras): rail.
    # select_vla_cameras, review of 4a0da6d, P2-4. Recorded as vla_cameras.
    vla_cams = getattr(args, "pilot", "servo") == "vla" or bool(getattr(args, "vla_cameras", False))
    # FrontCamera's capture interval, when one is asked for (--front-capture-
    # interval-s): rendered, and recorded as sim_config.front_capture_interval_s.
    fci = getattr(args, "front_capture_interval_s", None)
    # A perception stressor (--stress): the cameras' exposure and the scene's
    # own id, rendered; recorded as sim_config.stress (perception_stress).
    stress = getattr(args, "stress", None)
    if args.scene_mode == "attach":
        return rail.write_attach_configs(out / "pas_config", vla_cameras=vla_cams,
                                         front_capture_interval_s=fci,
                                         stress=stress), None
    plan = rail.plan_network(args.network, args.windows_ip, args.wsl_ip)
    return rail.write_sim_configs(out / "pas_config", plan, vla_cameras=vla_cams,
                                  front_capture_interval_s=fci, stress=stress), plan


def perception_stress_record(args, sim_cfg: dict | None, camera) -> dict | None:
    """metrics.json `perception_stress` (None without --stress): the stressor,
    its suite scenario, what was rendered (sim_config.stress: the exposure,
    the scene id), the weather as set or read back (the camera's
    scene_record), who applied it, and the non-KPI label
    (tools/perception_stress_report.py groups runs by `name`)."""
    name = getattr(args, "stress", None)
    if name is None:
        return None
    scene_part = (getattr(camera, "scene_record", None) or {}).get("stress") or {}
    attach = getattr(args, "scene_mode", "load") == "attach"
    return {"name": name, "scenario_id": pstress.scenario_id(name),
            "what": pstress.stressor(name)["what"],
            "rendered": (sim_cfg or {}).get("stress"),
            "weather": scene_part.get("weather"),
            "applied_by": ("the desktop's scene step (run_pas_ardupilot.ps1 -Step scene "
                           f"-Stress {name}); this node attached to its scene id and read "
                           "the weather back" if attach else
                           "this node: it loaded the scene and set the weather"),
            "kpi": "non-KPI", "kpi_reason": pstress.non_kpi_reason(name)}


def dry_run(args, out: Path) -> dict:
    """Everything checkable without a simulator, an autopilot or a GPU."""
    report = {"ok": True, "problems": [], "scene_mode": args.scene_mode}
    cfg, plan = sim_configs(args, out)
    if cfg is not None:
        scene = rail.load_jsonc(Path(cfg["dir"]) / cfg["scene"])
        report["sim_config"] = cfg
        if plan is not None:
            report["sitl_args"] = rail.sitl_args(scene, plan,
                                                 [rail.to_wsl_path(rail.PARAM_FILE)])
            report["sim_vehicle_equivalent"] = rail.sim_vehicle_equivalent(
                scene, plan, rail.to_wsl_path(rail.PARAM_FILE))
            report["router_conf"] = rail.router_conf(plan.gcs_address, plan.windows_ip)
            report["wsl_start"] = rail.wsl_start_command(
                args.wsl_distro, plan, mavros=(args.link == "mavros"))
    deps = {"pymavlink": module_available("pymavlink"),
            "projectairsim": module_available("projectairsim"),
            "torch": module_available("torch"),
            "transformers": module_available("transformers"),
            "rclpy": module_available("rclpy")}
    report["dependencies"] = deps
    need = ["torch", "transformers"]
    need += ["projectairsim"] if args.camera == "projectairsim" else ["rclpy"]
    need += ["pymavlink"] if args.link == "pymavlink" else ["rclpy"]
    for n in sorted(set(need)):
        if not deps[n]:
            report["ok"] = False
            report["problems"].append(f"{n} is not importable in {sys.executable}")
    # The policy the run would fly, refused here as the run would refuse it.
    try:
        bundle, yaml_path, choice = resolve_policy(args)
        report["policy"] = {"bundle": bundle, "yaml": yaml_path, **choice}
    except ValueError as exc:
        report["ok"] = False
        report["problems"].append(str(exc))
    if getattr(args, "pilot", "servo") == "vla":
        # What the VLA run would start or attach to (nothing is started).
        srv = vla_server_plan(args, out)
        report["vla"] = {"mission": args.mission,
                         "goal": None if args.goal is None else list(args.goal),
                         "object": args.vla_object, "cadence": args.cadence,
                         "front_source": args.vla_front_source,
                         "lost_timeout_s": args.lost_timeout_s,
                         "follow": (None if args.mission != "follow" else
                                    {k: getattr(args, k) for k in VLA_FOLLOW_ONLY
                                     + ("vla_rate_yaw", "target_estimator")}),
                         "server": {"mode": "started" if srv.started_here else "attached",
                                    "addr": f"{srv.host}:{srv.port}", "argv": srv.argv}}
    return report


class SceneLoader:
    """Loads the rendered scene and nothing else (no camera subscriptions):
    what `-Step scene` needs before ArduPilot is started. With `stress` (a
    perception stressor, --stress) it then sets the stressor's weather and
    reads it back (perception_stress.apply_weather), into `stress_record`;
    start() refuses on any problem, before ArduPilot is started."""

    def __init__(self, sim_cfg: dict, address: str = "127.0.0.1", projectairsim=None,
                 stress: str | None = None):
        if stress is not None:
            pstress.stressor(stress)                          # refuses an unknown name
        self.cfg, self.address, self.pas = sim_cfg, address, projectairsim
        self.client = self.world = None
        self.stress = stress
        self.stress_record: dict | None = None

    def start(self) -> None:
        pas = self.pas or importlib.import_module("projectairsim")
        self.client = pas.ProjectAirSimClient(address=self.address)
        self.client.connect()
        world = pas.World(self.client, self.cfg["scene"], delay_after_load_sec=2,
                          sim_config_path=self.cfg["dir"])
        self.world = world
        try:
            world.switch_streaming_view()
        except Exception:                                     # noqa: BLE001
            pass
        if self.stress is not None:
            rec = pstress.apply_weather(world, self.stress)
            self.stress_record = rec
            if not rec["ok"]:
                raise RuntimeError(f"--stress {self.stress}: the weather was not set: "
                                   + "; ".join(rec["problems"]))
            # The texture stressor's re-skinned actors (none for the others).
            tex = pstress.apply_texture(world, self.stress)
            # The level's sun: turned for the shadow stressor, read back always.
            sun = pstress.apply_sun(world, self.stress)
            self.stress_record = {**rec, "texture": tex, "sun": sun}
            if not tex["ok"]:
                raise RuntimeError(f"--stress {self.stress}: the texture was not set: "
                                   + "; ".join(tex["problems"]))
            if not sun["ok"]:
                raise RuntimeError(f"--stress {self.stress}: the sun is not the "
                                   f"stressor's: " + "; ".join(sun["problems"]))

    def stop(self) -> None:
        try:
            if self.client is not None:
                self.client.disconnect()
        except Exception:                                     # noqa: BLE001
            pass


def load_scene_only(args, out: Path, camera=None) -> int:
    """`-Step scene`: render, load, record, exit. The scene stays loaded in
    the simulator for ArduPilot (-Step sitl) and an attaching node."""
    sim_cfg, plan = sim_configs(args, out)
    stress = getattr(args, "stress", None)
    camera = camera or SceneLoader(sim_cfg, address=args.sim_host, stress=stress)
    try:
        camera.start()
        rec = {"scene": sim_cfg, "loaded_wall": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
               "next": "start ArduPilot now (run_pas_ardupilot.ps1 -Step sitl); "
                       "reloading this scene while it runs drives its clock backwards"}
        if stress is not None:
            # The stressor as rendered (sim_config.stress) and its weather as
            # set and read back (SceneLoader.stress_record).
            rec["stress"] = {"name": stress, "scenario_id": pstress.scenario_id(stress),
                             "weather": getattr(camera, "stress_record", None),
                             "attach_with": f"--stress {stress} (run_pas_perception.sh: "
                                            f"STRESS={stress})"}
        (out / "scene_load.json").write_text(json.dumps(rec, indent=2), encoding="utf-8")
        print(f"[scene]  {sim_cfg['scene_id']} loaded from {sim_cfg['dir']}; "
              f"start ArduPilot now, never before a reload")
        if stress is not None:
            print(f"[stress] {stress}: {pstress.stressor(stress)['what']}; an attaching "
                  f"node must be given --stress {stress} (STRESS={stress} on the Orin)")
        return 0
    finally:
        camera.stop()


def next_tick_deadline(deadline: float | None, t_tick: float, now: float,
                       tick: float = TICK) -> float:
    """The loop's next ABSOLUTE deadline (demo/follow_vlm.py's pacing).

    The loop slept `TICK - work` from each tick's own start, so every
    sleep's round-up to the OS timer was lost for good: 8.97-9.16 Hz on
    every pas_ap run (pas_ap_video_off_20261014: work p95 23.8 ms, period
    median 110 ms). Stepping a running deadline by TICK lets a late wake be
    repaid by the next tick's shorter sleep. A stall of more than a whole
    tick is not repaid - that would be a burst of back-to-back commands -
    the deadline restarts from now instead."""
    deadline = (t_tick if deadline is None else deadline) + tick
    return now if deadline < now - tick else deadline


# THE FEED BETWEEN TICKS (--vla-feed stream; guardrail.vla_backends.
# RemoteVLABackend.pump): the loop's wait for its next deadline, cut into
# slices with a pump() before each. No pump in the last FEED_GUARD_S: a
# capture's JPEG encoding must not make the tick itself late.
FEED_SLICE_S = 0.004
FEED_GUARD_S = 0.015


def pace_feeding(deadline: float, pump, clock, sleep, *, slice_s: float = FEED_SLICE_S,
                 guard_s: float = FEED_GUARD_S) -> int:
    """Wait to the tick's ABSOLUTE `deadline` (next_tick_deadline), calling
    `pump()` every `slice_s` while more than `guard_s` is left. Returns the
    number of pump() calls. With nothing left it returns at once, like the
    plain sleep(max(0, deadline - now)) it replaces."""
    n = 0
    while True:
        left = deadline - clock()
        if left <= 0.0:
            return n
        if left > guard_s:
            pump()
            n += 1
            left = deadline - clock()
            if left <= 0.0:
                return n
        sleep(min(slice_s, left))


def link_staleness(link) -> tuple[str | None, dict | None]:
    """(why the link's state is not live on this tick, or None; the ages it
    was judged on). (None, None) for a link that cannot say (no link_age()):
    the run then records freshness as not measured, never as fresh."""
    age = getattr(link, "link_age", None)
    if not callable(age):
        return None, None
    a = age()
    p, h = a.get("pos_age_s"), a.get("heartbeat_age_s")
    why = []
    if p is None or p > POS_STALE_S:
        why.append("no position fix received" if p is None
                   else f"no position fix for {p:.1f} s (limit {POS_STALE_S:g} s)")
    if h is None or h > HEARTBEAT_STALE_S:
        why.append("no vehicle heartbeat received" if h is None
                   else f"no vehicle heartbeat for {h:.1f} s (limit {HEARTBEAT_STALE_S:g} s)")
    if a.get("connected") is False:
        why.append("MAVROS reports the flight controller disconnected")
    return ("; ".join(why) or None), a


def _measure_sleep_ms(n: int = 20) -> float:
    """Median wall time of `time.sleep(0.001)`: the wait this loop's pacing
    does. (follow_vlm measures asyncio.sleep, the wait ITS loop does; from
    Python 3.11 time.sleep has its own high-resolution timer, and on the
    flight env's 3.10 it wakes on the system timer that timeBeginPeriod
    sets.)"""
    ts = []
    for _ in range(n):
        a = time.perf_counter()
        time.sleep(0.001)
        ts.append((time.perf_counter() - a) * 1000.0)
    return round(float(np.median(ts)), 2)


def _fine_timer_begin():
    """timeBeginPeriod(1) for the life of the flight, as follow_vlm's main()
    asks it (THE OS TIMER THE TICK SLEEPS ON there): the deadline pacing
    repays a late wake, the 1 ms timer makes the wakes late by less. The
    resolution obtained is MEASURED (TIMER_RESOLUTION_MS, metrics.json
    timer_resolution_ms), because Windows 11 may decline it."""
    global TIMER_RESOLUTION_MS
    winmm = None
    if sys.platform == "win32":
        try:
            import ctypes
            winmm = ctypes.WinDLL("winmm")
            if winmm.timeBeginPeriod(1) != 0:             # TIMERR_NOERROR
                winmm = None
        except Exception as exc:                          # noqa: BLE001
            print(f"[timer]  timeBeginPeriod unavailable ({type(exc).__name__})")
            winmm = None
    TIMER_RESOLUTION_MS = _measure_sleep_ms()
    asked = ("" if sys.platform != "win32" else
             f"1 ms timer {'requested' if winmm is not None else 'NOT granted'}; ")
    print(f"[timer]  {asked}a 1 ms sleep measures {TIMER_RESOLUTION_MS} ms")
    return winmm


def _fine_timer_end(winmm) -> None:
    if winmm is not None:
        try:
            winmm.timeEndPeriod(1)
        except Exception:                                 # noqa: BLE001
            pass


def run(args, *, link=None, camera=None, grounder=None, obs=None,
        clock=time.time, sleep=time.sleep, popen=None, run_cmd=None,
        recorder_cls=None, live_status: bool = False, vla_client=None,
        vla_frames=None) -> int:
    """Fly one run. Every collaborator is injectable so the loop can be tested
    without a simulator, an autopilot or a GPU (tests/test_pas_ardupilot.py);
    `recorder_cls` stands in for demo/recorder.FrameRecorder (--save-view).
    `live_status` writes a Dashboard heartbeat (main() turns it on for a
    flight; the tests leave it off, so they never touch the live folder).
    --pilot vla (tests/test_pas_vla.py): `vla_client` stands in for the
    guardrail.vla_link.RemoteVLA built here (the server is still the one
    --vla-server names, whose hello pins the identity), `vla_frames` for
    vla_mission.VLAFrames over the camera's VLAFront / VLADown."""
    popen = popen or subprocess.Popen
    run_cmd = run_cmd or subprocess.run
    tag = args.tag or time.strftime("pas_ap_%Y%m%d_%H%M%S")
    out = Path(args.out_root) / tag
    out.mkdir(parents=True, exist_ok=True)
    check_args(args)

    if args.dry_run:
        rep = dry_run(args, out)
        print(json.dumps(rep, indent=2))
        return 0 if rep["ok"] else 2
    if args.scene_only:
        return load_scene_only(args, out, camera)

    random.seed(args.seed)
    np.random.seed(args.seed % (2 ** 32))

    # The signed bundle (given, or the default policy's); a YAML only under
    # --dev-yaml, recorded so. Refused here, before anything starts.
    try:
        bundle, yaml_path, choice = resolve_policy(args)
    except ValueError as exc:
        raise SystemExit(f"[policy] {exc}") from None
    policy, policy_source = load_for_flight(
        bundle, yaml_path, allow_unverified=args.allow_unverified_bundle)
    policy_source.update(choice)
    smap, map_note = load_obstacle_map(policy, Path(args.citymap))
    print(f"[policy] {policy.policy_id} {policy.policy_hash} from "
          f"{policy_source['kind']} - signature {policy_source['signature']}"
          + (f" ({DEV_YAML_FLAG})" if policy_source.get("dev_yaml") else ""))
    print(f"[map]    {map_note}")
    # The rule clock: one callable for the Shield and every generation's CSP
    # (guardrail.replay.rule_clock), refused here if --clock-start or
    # --clock-tz cannot be read. Mission time is None until the loop starts.
    mission_t0: list = [None]
    rule_now, clock_record = rule_clock(
        getattr(args, "clock_start", None), tz=getattr(args, "clock_tz", None),
        elapsed=lambda: None if mission_t0[0] is None else clock() - mission_t0[0])
    print(f"[clock]  rules read the {clock_record['kind']} clock: "
          f"{clock_record.get('start') or clock_record.get('tz')}")
    # The grant's lookahead (guardrail.shield: 5 s at 10 Hz, 50 future poses),
    # as on both SITL rails, and NO escalation FSM inside the Shield: this
    # node runs THE FSM below, fed with what only it sees (the autopilot's
    # mode, home, landed, a takeover). With the Shield's own FSM on as well,
    # audit.jsonl would record one state machine and flight_log.jsonl another
    # (guardrail.replay, "ONE ESCALATION FSM PER RAIL").
    # pipe_clock: the setpoints this node sent (note_flown) are aged on the
    # loop's REAL clock, not on the Shield's monitor clock, which filter()
    # counts at the nominal 0.1 s here; the recorded pas_ap ticks are 0.109 s
    # median and up to 0.73 s (14 Oct). Read only with a measured velocity.
    shield = rail_shield(policy, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S,
                         obstacle_map=smap, clock=rule_now,
                         velocity_response=vresp.ARDUCOPTER_GUIDED,
                         pipe_clock=clock)
    shield_on = args.shield == "on"
    # The own-velocity forecast and the hold that stops clear (OWN_VELOCITY_
    # DEFAULT). Off: the Shield's State carries no velocity, so its decisions
    # are the commanded forecast's, byte for byte, and Loiter is BRAKE.
    # Shield on only: with the Shield off it filters for the record alone, and
    # the control arm's escapes (the raw action's violations) stay judged on
    # the commanded forecast, comparable with every stored off run.
    own_v = (getattr(args, "own_velocity_forecast", OWN_VELOCITY_DEFAULT) == "on"
             and shield_on)
    vel_sources: dict = {}
    n_vel_used = n_hold_guided = n_stop_advised = 0
    hold_plans: list[dict] = []
    hold_deferred = False
    # When the current deferral began (rail time), the banner's reason for
    # it, and the hand-overs to BRAKE the plan did not choose (HOLD_DEFER_*:
    # a stale link, a gap in the stream, the deferral cap), by reason.
    hold_deferred_since = None
    hold_why = None
    forced_handovers: dict = {}
    # A Loiter flown as BRAKE taken back into GUIDED when BRAKE let the
    # position go illegal (guardrail.response.HOLD_RECLAIM_MAX): this Loiter's
    # count and the episode's.
    hold_reclaims_loiter = n_hold_reclaims = 0
    # A reclaimed hold stays in GUIDED: BRAKE already drifted once in this
    # Loiter, so the plan does not hand it back; only a forced hand-over does
    # (the stream, the link, the deferral cap), or the Loiter ends.
    hold_reclaimed = False
    # Was the position legal at some tick since BRAKE got the hold? Only a
    # drift INTO a rule is reclaimed: a hold handed to BRAKE while still
    # illegal (a forced hand-over: the stream or the link could not be
    # trusted, or the deferral cap) stays with BRAKE and the FSM.
    brake_was_legal = False
    # When this node last sent a setpoint (rail clock): a deferred hold is
    # flown by the stream, so a gap in it hands the hold to BRAKE.
    t_last_sent = None
    # The P0 escapes on the stored runs' basis (Shield.recheck_commanded):
    # the flown action re-checked from the position on the commanded
    # forecast, beside the rail's own re-check (made with the velocity).
    n_p0_cmd_basis = 0
    fsm = (EscalationFSM(FSMConfig.from_yaml(args.fsm_config) if args.fsm_config
                         else FSMConfig())
           if shield_on and not args.no_fsm else None)
    horizon = theta_horizon(getattr(args, "theta_horizon_s", "auto"))
    hold_mode = getattr(args, "hold_mode", HOLD_MODE)
    hold_ap_mode = autopilot_mode("LOITER", hold_mode)    # what ArduCopter holds in
    # THE PILOT (module docstring, THE VLA PILOT). vla_goto: no detector runs
    # and the start gate (a detector-subject gate) is skipped, recorded.
    vla_on = getattr(args, "pilot", "servo") == "vla"
    vla_goto = vla_on and args.mission == "goto"
    VM = _vm() if vla_on else None

    # ---- simulator configs, frame and mission ------------------------------
    sim_cfg, plan = sim_configs(args, out)
    scene, scene_path = None, None
    if sim_cfg is not None:
        scene_path = Path(sim_cfg["dir"]) / sim_cfg["scene"]
        scene = rail.load_jsonc(scene_path)
        if plan is not None:
            print(f"[sim]    configs rendered to {sim_cfg['dir']} "
                  f"({plan.mode}: sim binds {plan.sim_bind_ip}:{rail.SIM_SERVO_PORT}, "
                  f"sends to {plan.ardupilot_ip}:{rail.SITL_SENSOR_PORT})")
        else:
            print(f"[sim]    attaching to scene {sim_cfg['scene_id']} on {args.sim_host}")
        if getattr(args, "stress", None) is not None:
            print(f"[stress] {args.stress} ({pstress.scenario_id(args.stress)}): "
                  f"{pstress.stressor(args.stress)['what']}; scene {sim_cfg['scene_id']}; "
                  f"a perception-stress run, never KPI-grade")
    scene_home = rail.home_geo_point(scene) if scene else None
    # The policy's own origin, else the scene's home-geo-point; with neither,
    # the EKF's first fix (set at the pre-arm step below).
    anchor = FrameAnchor.choose(policy, scene_home)
    # The follow mission the CSPs are compiled for: from the spawn point, the
    # target proxy the start itself (follow_mission).
    start_xy, start_src = scene_spawn_xy(scene, anchor)
    mission, target_src = pilot_mission(args, start_xy)
    mission_rec = {"task": mission.task_text, "start": list(start_xy),
                   "start_source": start_src,
                   "target": [mission.target_x, mission.target_y],
                   "target_source": target_src,
                   "cruise_alt_m": mission.cruise_alt_m,
                   "speed_pref_mps": mission.speed_pref_mps}
    # The episode record before the audit log: it moves an earlier run's files
    # aside and writes generation 0's policy and CSP, at the rule clock's
    # reading.
    rec = EpisodeRecord(out, policy, mission, lookahead_s=LOOKAHEAD_S,
                        clock=rule_now, clock_record=clock_record)
    rec.event("mission", **mission_rec)
    try:
        (out / "prompt.yaml").write_text(ConstraintCompiler(policy).build_prompt(mission),
                                         encoding="utf-8")
    except Exception as exc:                                  # noqa: BLE001
        rec.event("prompt_failed", error=f"{type(exc).__name__}: {exc}")
    audit = AuditLogger(out / AUDIT_FILE, policy)
    if obs is None:
        from semantic_demo import SemanticObs
        obs = SemanticObs()
    sitl_proc = sitl_log = None
    save_view = bool(getattr(args, "save_view", False))
    # THE VLA PILOT's parts, each closed in the outer finally: the frame store
    # over VLAFront / VLADown (vla_mission.VLAFrames: a capture pairs the
    # newest front and down frames and takes its pose from obs's ring, which
    # the loop feeds) whose SENT captures are kept for the offline replay gate
    # (vla_frames/, vla_captures.jsonl), the server, the backend and the pilot.
    vla_saver = vla_srv = vla_backend = vla = vla_identity = None
    vla_warmup: dict | None = None
    vla_rec: dict = {}
    # vla_captures.jsonl: written with the records, else on the way out (a
    # crash after the take-off; review of 4a0da6d, P3-15).
    vla_captures_written = False
    # How the Dashboard row ends when run() returns rather than raising
    # (status, detail): a run refused before arming ends "failed", with why.
    live_end: tuple | None = None
    if vla_on and vla_frames is None:
        vla_saver = VM.FrameSaver(out / "vla_frames")
        vla_frames = VM.VLAFrames(obs, saver=vla_saver, clock=clock)
    if camera is None and args.camera == "projectairsim":
        camera = ProjectAirSimCamera(obs, sim_cfg, address=args.sim_host,
                                     attach=args.scene_mode == "attach",
                                     chase=save_view,
                                     vla_frames=vla_frames if vla_on else None,
                                     vla_front_source=(args.vla_front_source if vla_on
                                                       else "vla"),
                                     stress=getattr(args, "stress", None))
    recorder = view_dir = rhud = None
    rec_why = None if save_view else "--save-view not given"
    rec_summary = None
    hud_ms: list[float] = []
    hud_failed = 0
    # The Dashboard heartbeat. Beyond the mission loop's beats, every slow
    # step below beats (memory only) with a phase and a detail: the first
    # version beat only inside the loop, and the scene load, SITL boot, EKF,
    # detector load, take-off and landing of a 225 s run read "no heartbeat"
    # (review of 8035d0a, P1). It ends as the last thing the run's `finally`
    # does, after the teardown.
    live = (_live_heartbeat(tag, log_dir=out, total_s=args.max_s, phase="starting",
                            detail="loading the scene") if live_status
            else _NoBeat())
    # A Ctrl+C the mission loop caught: it lands and writes the records, so
    # no exception leaves run(), and the row must still end "stopped"
    # (re-review of d180e01, P2: it ended "ok" beside end_reason
    # "interrupted"). Set in the loop's `except KeyboardInterrupt`.
    interrupted = False
    # The simulator's evaluation RPCs (the subject truth, its frame check, the
    # EKF against the simulator) go through one worker thread, never on the
    # loop (OffLoopRpc, SIM_RPC_WAIT_S).
    rpc = OffLoopRpc()
    # What the detector was built with (build_grounder); an injected one
    # (the tests' stand-ins) says so.
    grounder_rec: dict = {"injected": grounder is not None}
    # THE NET FROM THE ARMING ON (land_on_the_way_out; review 3, 2026-10-22):
    # whether the aircraft was armed and took off, whether the loop's finally
    # made its LAND decision, the exception on the way out and the stage it
    # came from, and what the net reads (the hover's or the loop's takeover,
    # the throttle).
    airborne = land_decided = False
    crash = None
    stage = "the bring-up"
    watch = throttle = takeover = None
    # What was asked of the autopilot, in the FSM's words and ArduCopter's,
    # and which request can explain a mode change (ModeRequests): one record
    # from the take-off on - the hover's, the loop's and the net's.
    asks = ModeRequests(hold_mode)
    # The REST hot-apply listener and the lock it swaps the Shield's rules
    # under (rail_hotapply; None without --hotapply-port).
    hot = hot_gate = None
    try:
        # THE REST HOT-APPLY LISTENER (WP1-15), first: one that cannot start
        # (rail_hotapply.start says why) stops the run before anything flies.
        hot, hot_gate = rail_hotapply.start(
            args, shield, rec,
            mission_t=lambda: None if mission_t0[0] is None else clock() - mission_t0[0])
        if vla_on:
            # THE VLA SERVER, started first (or attached): a bf16 cold load
            # can take many minutes (vla_mission.HELLO_TIMEOUT_S) and runs
            # while the scene and the detector load; its hello is waited for
            # below, before ArduPilot starts.
            vla_srv = vla_server_plan(args, out)
            vla_srv.start()
            print("[vla]    " + ("started " + " ".join(vla_srv.argv[1:]) if vla_srv.started_here
                                 else f"attaching to {vla_srv.host}:{vla_srv.port}"))
        if camera is not None:
            camera.start()                     # loads the scene: BEFORE SITL starts
            live.beat(detail="scene loaded")                                # memory only
        # ---- the detector, loading while SITL boots (DETECTOR_READY_TIMEOUT_S):
        # its model load no longer runs beside the mission's first ticks.
        fv = None
        hfov = args.hfov
        if vla_goto:
            # A VLA GOTO's hint is the goal's: no detector runs (NoDetector),
            # none is waited for, and the start gate is skipped (below).
            grounder = NoDetector()
            grounder_rec = {"injected": False, "detector": None,
                            "why": "--pilot vla --mission goto: the hint comes from the "
                                   "goal, so no detector runs"}
        if grounder is None or hfov is None:
            fv = _fv()
        if hfov is None:
            hfov = (fv.camera_hfov_deg(config_path=Path(sim_cfg["dir"]) / sim_cfg["robot"])
                    if sim_cfg else fv.CAMERA_HFOV_DEG)
        if grounder is None:
            live.beat(detail="loading the detector")                        # memory only
            grounder, grounder_rec = build_grounder(args, obs, out, fv, hfov)
            grounder_rec["injected"] = False
            grounder.start()
            print(f"[detect] {fv.DETECTOR_ID} loading in its own thread (lock "
                  f"{grounder_rec['lock']}; negative queries "
                  f"{grounder_rec['negative_queries'] or 'none'})")
        if save_view:
            # Started now, writing nothing until the loop's first set_hud():
            # the video begins at mission start (after the take-off to cruise),
            # as follow_vlm's does, and recorder.json measures the rate over
            # the frames actually written.
            recorder, view_dir, rec_why = start_recorder(
                args, out, camera, obs, recorder_cls=recorder_cls,
                # The VLA pilot's frames carry the city rail's VLA panel.
                annotate=VM.annotate_vla if vla_on else None)
            if recorder is None:
                print(f"[view]   *** not recording: {rec_why}")
            else:
                try:
                    ind = (None if getattr(args, "no_policy_hud", False) else
                           build_policy_hud(policy, shield, smap, args.citymap))
                except Exception as exc:                      # noqa: BLE001
                    ind = None
                    print(f"[hud]    no policy indicator ({type(exc).__name__}: {exc}); "
                          f"the plain HUD is drawn")
                rhud = RailHud(ind, query=args.object, shield_on=shield_on,
                               hold_ap_mode=hold_ap_mode,
                               # TARGET LOCKED only where the detector holds a lock
                               locked=grounder_holds_lock(grounder))
        if vla_on:
            # ---- the VLA server's hello and the pilot, before ArduPilot starts
            vla_identity, vla_backend, vla_rec = _vla_bring_up(
                args, out, VM, vla_srv, policy, shield_on, grounder, hfov, vla_frames,
                live, clock, vla_client, run_cmd=run_cmd)
        if args.start_sitl and plan is not None:
            cmd = rail.wsl_start_command(args.wsl_distro, plan,
                                         mavros=(args.link == "mavros"))
            print("[sitl]   " + " ".join(cmd))
            sitl_log = open(out / "sitl_wsl.log", "w", encoding="utf-8")
            sitl_proc = popen(cmd, stdout=sitl_log, stderr=subprocess.STDOUT)
            rec.event("sitl_started", command=cmd)
            live.beat(detail="SITL started")                                # memory only

        # ---- autopilot -------------------------------------------------------
        live.beat(detail="connecting to the autopilot")                     # memory only
        if link is None:
            if args.link == "pymavlink":
                link = PymavlinkLink(args.mavlink_url or (plan.router_tcp_url if plan is not None
                                     else f"tcp:127.0.0.1:{rail.ROUTER_TCP_PORT}"))
                link.connect(timeout=180.0)
            else:
                from mavros_link import MavrosLink, Ros2ImageSource
                link = MavrosLink(setpoint_via=getattr(args, "setpoint_via", "mavros"))
                if args.camera == "ros2":
                    Ros2ImageSource(link.node, obs, args.image_topic)
        if vla_on and "before_arm" not in inspect.signature(link.bring_up).parameters:
            # The warm-up gate runs disarmed, in the pre-arm step; a link with
            # none would put it in an airborne hover nothing watches.
            raise SystemExit(f"--pilot vla needs a link with a pre-arm step (the warm-up "
                             f"gate runs disarmed): {link.name} has none")
        if vla_on:
            # The VLA server's host (its hello) into the link's own hil
            # evidence, as the ROS rail records a VLA node's (note_vla_host).
            note_vla_host(link, vla_rec.get("vla_host"))
        ap_fence = backstop = None
        pre_arm: dict = {}
        # The detector's first inference (wait_for_detector): waited for in the
        # pre-arm step when the link has one, else airborne after the take-off.
        detector_ready: dict = {}
        if vla_goto:
            detector_ready.update(ready=None, outcome="not_used", why=grounder_rec["why"],
                                  waited_s=0.0, seq=None, timeout_s=None, airborne=False,
                                  phase=None)

        def vla_capture():
            """The warm-up's capture: the link pumped and its pose put in the
            ring a capture's pose is read from (the loop feeds it the same
            way), until the frame store pairs a front and a down frame, at
            most 30 s."""
            t_end = clock() + 30.0
            while clock() < t_end:
                link.pump()
                st_ = link.state(anchor)
                if st_ is not None:
                    s_ = link.snap
                    obs.put_pose_full(clock(), st_.x, st_.y, st_.up, math.radians(st_.yaw_deg),
                                      s_.pitch or 0.0, s_.roll or 0.0)
                cap = vla_frames.capture()
                if cap is not None:
                    return cap
                live.beat(detail="waiting for the VLA camera pair (warm-up)")  # memory only
                sleep(TICK)
            return None

        def before_arm(phase: str = "before arming") -> None:
            """With the position usable and before GUIDED / arming: the frame's
            anchor (the first fix, when nothing else anchors it), the
            autopilot's own GeoFence as it holds it (recorded; no parameter is
            changed), the policy's zones uploaded onto it, and - disarmed, on
            the ground - the detector's first inference and the VLA's warm-up
            gate."""
            nonlocal anchor, ap_fence, backstop, vla_warmup
            if pre_arm:
                return
            pre_arm["phase"] = phase
            link.pump()
            if anchor is None:
                s = link.snap
                anchor = FrameAnchor(s.lat, s.lon, s.alt_msl,
                                     "ekf.first-fix (no policy origin, no scene)")
            print(f"[frame]  scene frame anchored at {anchor.to_dict()}")
            ap_fence = autopilot_fence_record(link, policy, args.cruise_alt)
            if ap_fence is not None:
                rec.event("autopilot_fence", **ap_fence)
                print(f"[fence]  ENABLE {ap_fence['FENCE_ENABLE']}  ALT_MAX "
                      f"{ap_fence['FENCE_ALT_MAX']}  RADIUS {ap_fence['FENCE_RADIUS']}  "
                      f"ACTION {ap_fence['FENCE_ACTION']}  TYPE {ap_fence['FENCE_TYPE']}")
                if ap_fence["stricter_than_policy"]:
                    print(f"[fence]  *** {ap_fence['why']}")
                if ap_fence["zones_enforced"] is False:
                    print("[fence]  *** FENCE_TYPE has no polygon bit: the uploaded "
                          "zones are held but not enforced")
            st0 = link.state(anchor)
            pos0 = None if st0 is None else (st0.x, st0.y)
            backstop = FenceBackstop(policy, link, rec, anchor, ap_fence, now=rule_now,
                                     enabled=not getattr(args, "no_fence_upload", False),
                                     home_xy=pos0)
            backstop.publish(pos0, phase)
            if phase == "before arming":
                # THE DETECTOR'S FIRST INFERENCE, ON THE GROUND (review
                # 2026-10-22). It was waited for after the take-off, so a slow
                # model load (up to DETECTOR_READY_TIMEOUT_S) was airborne hover
                # nothing watched. The detector loads under SITL's boot and is
                # almost always up by now; when it is not, the aircraft waits
                # for it disarmed. A detector thread that died is no detector:
                # the aircraft is not armed for a mission that cannot see.
                if not vla_goto:
                    detector_ready.update(wait_for_detector(grounder, link, clock=clock,
                                                            sleep=sleep), phase=phase)
                    if detector_ready["outcome"] == "detector_dead":
                        rec.event("detector_dead", **detector_ready)
                        raise RuntimeError(f"not arming: {detector_ready['why']}")
                if vla_on:
                    # THE VLA WARM-UP GATE, DISARMED (vla_mission.run_warmup):
                    # three requests on live frames, the last judged; a
                    # refusal arms nothing (PreArmRefused).
                    vla_warmup = VM.run_warmup(
                        vla_rec["client"], vla_capture, vla_rec["hint_source"], args.mission,
                        args.vla_object, decision_timeout_s=args.vla_decision_timeout_s,
                        first_decision_timeout_s=args.first_decision_timeout_s,
                        max_latency_s=args.warmup_max_latency_s, commit=vla_frames.commit,
                        beat=lambda k: live.beat(detail=f"VLA warm-up inference {k + 1}/"
                                                        f"{VM.WARMUP_REQUESTS}, disarmed"),
                        # The autopilot link is read while an answer is
                        # awaited (a cold first inference may take a minute),
                        # as the detector's wait reads it.
                        clock=clock, sleep=sleep, idle=link.pump)
                    vla_warmup["phase"] = phase
                    vla_rec["gpu_memory"]["after_warmup"] = gpu_memory(run_cmd)
                    rec.event("vla_warmup", **vla_warmup)
                    print(f"[vla]    warm-up: {vla_warmup.get('kind')} in "
                          f"{vla_warmup.get('rtt_s')} s round trip (gate "
                          f"{args.warmup_max_latency_s:g} s) - "
                          + ("PASS" if vla_warmup["pass"] else f"REFUSED: {vla_warmup['why']}"))
                    if not vla_warmup["pass"]:
                        raise PreArmRefused("warmup_refused", f"not arming: {vla_warmup['why']}")

        print(f"[link]   {link.name}: bring-up to {args.cruise_alt} m")
        stage = "the take-off"
        live.beat(detail=f"EKF, arming and take-off to {args.cruise_alt:g} m")  # memory only
        try:
            if "before_arm" in inspect.signature(link.bring_up).parameters:
                link.bring_up(args.cruise_alt, before_arm=before_arm)
            else:
                link.bring_up(args.cruise_alt)
        except PreArmRefused as exc:
            # REFUSED IN THE PRE-ARM STEP, DISARMED (the VLA's warm-up gate):
            # nothing was armed. On record as the city rail records it
            # (review of 4a0da6d, P2-3) and exit 4; the outer finally stops
            # the server and the rest, and the Dashboard row ends "failed"
            # saying why.
            live_end = ("failed", f"refused before arming ({exc.kind}): {exc}")
            _vla_refusal_records(args, out, VM, exc, rec=rec, policy=policy,
                                 policy_source=policy_source, scene_path=scene_path,
                                 sim_cfg=sim_cfg, link=link, backend=vla_backend, srv=vla_srv,
                                 identity=vla_identity, vla_rec=vla_rec, warmup=vla_warmup,
                                 frames=vla_frames, saver=vla_saver, clock_record=clock_record)
            vla_captures_written = True
            return EXIT_REFUSED
        # Armed and at altitude: from here on no way out of run() skips a
        # LAND request (the loop's finally, else the net in the outer one).
        # A bring-up that raised after arming is caught there by the
        # autopilot's own armed flag.
        airborne = True
        stage = "the set-up after the take-off"
        before_arm("after take-off: this link has no pre-arm step")
        live.beat(phase="airborne", detail="at cruise altitude")            # memory only
        # Read from the autopilot during bring-up (identify()); None if unreadable,
        # which the manifest then records as unresolved - never an assumed 1.0.
        speedup = getattr(link, "sim_speedup", None)
        # The arming point, for the FSM's "home reached" (RTL): the vehicle
        # took off vertically from it.
        link.pump()
        st_home = link.state(anchor)
        home_xy = None if st_home is None else (st_home.x, st_home.y)
        if home_xy is not None:
            rec.event("mission_start_check", planned_start=list(start_xy),
                      arming_point=[round(home_xy[0], 3), round(home_xy[1], 3)],
                      gap_m=round(math.hypot(home_xy[0] - start_xy[0],
                                             home_xy[1] - start_xy[1]), 3))
        # The literal LOITER hold's throttle (--hold-mode LOITER on SITL only).
        throttle = ThrottleHold(link, hold_mode, autopilot_kind_of(link), rec)

        # ---- subject and pilot (the detector started before SITL, above) -------
        width_m, subject_class = (fv or _fv()).subject_width(args.object)
        if vla_goto:
            # A VLA GOTO has no subject: the Shield is given none (every
            # stand-off inert, set_subject(None) on each tick), no stand-off
            # binds, and nothing is "unclassified".
            width_m = subject_class = None
        # The stand-off rules this subject's class binds. An UNCLASSIFIED
        # subject (a phrase follow_vlm's tables do not know: "a child", "a
        # jogger", "people") still binds the catch-all "*" rules - the
        # policies' own words: "including a subject that could not be
        # classified" (sitl_pedestrian.yaml), "whatever is being tracked"
        # (follow_car_citylife.yaml) - and follow_vlm gives the Shield the
        # subject whatever its class. Until 2026-10-19 this node gave the
        # Shield no subject at all when the class was None, so every
        # stand-off went inert while the record (standoff_ring_m, the HUD's
        # rule panel) still showed the catch-all ring in force.
        standoff_ids = [c.id for c in policy.by_type(SubjectStandoff)
                        if c.binds(subject_class) and not vla_goto]
        unclassified = None
        if subject_class is None and not vla_goto:
            unclassified = {"object": args.object, "width_m": width_m,
                            "binding_rules": standoff_ids}
            rec.event("subject_unclassified", **unclassified)
            print(f"[subject] WARNING no class known for {args.object!r}: only the "
                  f"catch-all stand-off(s) {standoff_ids or 'none'} bind it, and the "
                  f"width range assumes {width_m:.2f} m, a car - a subject that is "
                  f"not car-sized is ranged wrong by the ratio")
        # The detector pilot; the VLA's (VLAPilot) is built at the mission's
        # start, with the mission's clock. With --target-estimator the servo
        # pilot is the estimator follow (EstimatorFollowPilot), its
        # SubjectTrack started at the mission's start like the VLA's.
        if not vla_on and getattr(args, "target_estimator", False):
            pilot = make_estimator_follow_pilot(args, grounder, hfov, width_m, fv or _fv())
            print(f"[track]  estimator follow ON: stand-off "
                  f"{pilot.want_range_m:.1f} m, speed_max {pilot.speed_max:g} m/s "
                  f"(follow_vlm.fly()'s servo on the estimate)")
        else:
            pilot = (None if vla_on else
                     FollowPilot(grounder, hfov, yaw_gain=args.yaw_gain,
                                 want_w_frac=args.want_width, speed_max=args.speed_max,
                                 cruise_alt=args.cruise_alt, alt_gain=args.alt_gain,
                                 climb_max=args.climb_max))
        # The subject's simulator pose, for EVALUATION only (SubjectTruth): the
        # Shield below is given the estimate, never this.
        actor, actor_why = ((None, "--pilot vla --mission goto: a GOTO has no subject")
                            if vla_goto else
                            subject_actor_for(getattr(args, "subject_actor", "auto"),
                                              args.object, subject_class))
        truth = SubjectTruth(camera, actor, actor_why, scene_home, anchor, rpc=rpc)
        rings = [c.min_range_m for c in policy.by_type(SubjectStandoff)
                 if c.binds(subject_class) and not vla_goto]
        ring_m = max(rings) if rings else None
        print(f"[truth]  subject pose: {truth.source or truth.why_off}")

        # ---- the hover before the mission, watched (HoverWatch) ---------------
        # From the take-off to the mission clock the aircraft hovers in
        # ArduCopter's GUIDED hold: for the start gate, and - on a link with no
        # pre-arm step - for the detector's first inference. Every tick of it
        # is watched (the mode, the link, the Shield on the position; review
        # 2026-10-22), and all of it runs inside this try: an exception or a
        # Ctrl+C ends the episode with no mission, landed by the loop's finally
        # below and on record - never a vehicle left holding in GUIDED with
        # nothing commanding it (on HIL or attach there is no SITL for the
        # outer finally to stop). The handler's and the finally's own writes
        # are best effort (review 3): a second exception there left run()
        # past every LAND. The takeover test starts from GUIDED (HoverWatch):
        # a bring-up that ended in another mode is a takeover already.
        stage = "the hover before the mission"
        if hot_gate is not None:
            # From the hover on, the Shield is this thread's except while it
            # sleeps: a REST event lands between two ticks (rail_hotapply,
            # THE LOCK). Released in the loop's finally (rail_hotapply.stop).
            sleep = hot_gate.sleeper(sleep)
            hot_gate.hold()
        watch = HoverWatch(link, anchor, obs, shield, asks, subject_class=subject_class,
                           act=shield_on, backstop=backstop, rec=rec, rule_now=rule_now,
                           t0=clock())
        # Why no mission is flown (None: one is): the gate timed out under
        # --start-timeout-action land, the autopilot took the aircraft or the
        # link died while it hovered, the detector died, or the hover crashed.
        no_mission = None
        gate_crash = None
        gate = None
        # The gate is a DETECTOR-SUBJECT gate: a VLA GOTO has neither, so it
        # is skipped and says why; the VLA's approach keeps it (the subject is
        # in view before the VLA is asked to approach it).
        gate_on = getattr(args, "start_when_seen", 0) > 0 and not vla_goto
        gate_rec: dict = ({"enabled": True, "needed": args.start_when_seen,
                           "reached": False, "outcome": None, "aborted": None,
                           "wait_s": 0.0, "timeout_action": args.start_timeout_action}
                          if gate_on else
                          {"enabled": False, "needed": 0,
                           "why": ("--pilot vla --mission goto: the start gate waits for a "
                                   "detected subject, and a GOTO has none (its target is "
                                   "the commanded goal; no detector runs): skipped, the "
                                   "mission started at take-off"
                                   if vla_goto else
                                   "--start-when-seen 0: the mission started at take-off "
                                   "whatever the camera saw")})
        hover_phase = "detector wait"
        try:
            if not detector_ready:
                # No pre-arm step on this link: the detector's wait is
                # airborne, watched, and comes out of the gate's budget.
                detector_ready.update(wait_for_detector(
                    grounder, link, clock=clock, sleep=sleep, watch=watch,
                    timeout_s=(min(DETECTOR_READY_TIMEOUT_S, args.start_timeout_s)
                               if gate_on else DETECTOR_READY_TIMEOUT_S)),
                    phase=pre_arm.get("phase"))
            dr = detector_ready
            print(f"[detect] " + (
                f"none: {dr['why']}" if dr["outcome"] == "not_used" else
                f"first inference after {dr['waited_s']} s more "
                f"({'airborne' if dr['airborne'] else 'on the ground'})" if dr["ready"] else
                f"*** no inference after {dr['waited_s']} s ({dr['outcome']}): "
                + ("no mission is flown on a detector that died"
                   if dr["outcome"] == "detector_dead" else
                   watch.why if dr["outcome"] in ("takeover", "link_lost", "unsafe") else
                   "the mission starts on a detector still loading")))
            if dr["outcome"] == "detector_dead":
                no_mission = dr["why"]
            elif watch.ended in ("takeover", "link_lost"):
                no_mission = watch.why
            if gate_on:
                hover_phase = "start gate"
                if no_mission is None and watch.ended is None:
                    gate = StartGate(args.start_when_seen, args.object, args.colour_min,
                                     max_range_m=args.start_max_range_m, width_m=width_m,
                                     hfov_deg=hfov, gate_frac=getattr(args, "lock_gate", 0.12),
                                     identity=getattr(args, "identity", False), fv=fv or _fv())
                    if callable(getattr(grounder, "begin_acquire", None)):
                        grounder.begin_acquire()
                    # The budget left: an airborne detector wait came out of it.
                    budget = max(0.0, args.start_timeout_s - watch.hover_s(clock()))
                    print(f"[start]  hovering until the detector has seen {args.object!r} "
                          f"{gate.start_text()} (at most {budget:g} s), the "
                          f"hover watched")
                    # A reader of its own, so the flight's truth counters stay per tick.
                    gate_truth = SubjectTruth(camera, actor, actor_why, scene_home, anchor,
                                              rpc=rpc)
                    gate_rec = wait_for_subject(gate, grounder, watch, obs, timeout_s=budget,
                                                clock=clock, sleep=sleep, truth=gate_truth,
                                                hfov_deg=hfov, width_m=width_m, fv=fv)
                    gate_rec["timeout_action"] = args.start_timeout_action
                    if callable(getattr(grounder, "commit", None)):
                        # The confirmed box seeds the lock and the jump gate;
                        # with none, tracking starts from whatever is seen next.
                        grounder.commit(gate.pick if gate_rec["reached"] else None,
                                        yaw=gate.pick_yaw)
                    link.pump()
                    gate_rec["truth_at_decision"] = gate_truth_verdict(
                        gate_rec, gate_truth, link.state(anchor), hfov, fv=fv or _fv())
                else:
                    gate_rec.update(outcome=watch.ended or dr["outcome"], aborted=no_mission,
                                    why=("the hover ended before the gate ran: "
                                         + (no_mission or watch.why or "")))
                if no_mission is None and not gate_rec["reached"]:
                    if gate_rec["outcome"] in ("takeover", "link_lost", "detector_dead"):
                        no_mission = gate_rec["aborted"]
                    elif (gate_rec["outcome"] == "timed_out"
                          and args.start_timeout_action == "land"):
                        no_mission = gate.timeout_text(args.object, gate_rec["wait_s"])
                rec.event("start_gate", **{k: gate_rec.get(k) for k in (
                    "reached", "outcome", "wait_s", "first_candidate_s", "inferences_judged",
                    "inferences_with_plausible", "best_streak", "rejected_by", "pick",
                    "aborted", "timeout_action", "truth_at_decision", "rule",
                    "decided_by", "identity_gate")})
                print(f"[start]  {'subject acquired' if gate_rec['reached'] else '*** NO SUBJECT ***'}"
                      f" after {gate_rec['wait_s']} s ({gate_rec['outcome']}): "
                      f"{gate_rec.get('inferences_judged', 0)} inferences judged, "
                      + (gate.streak_text() if gate is not None else
                         f"best streak {gate_rec.get('best_streak', 0)}, refused "
                         f"{gate_rec.get('rejected_by', {})}")
                      + ("" if gate_rec["reached"] else
                         f" - {no_mission}" if no_mission else
                         " - the hover broke a rule: the mission starts, so the Shield and "
                         "the escalation FSM act" if gate_rec["outcome"] == "unsafe" else
                         " - flying anyway (--start-timeout-action fly)"))
        except KeyboardInterrupt:
            no_mission = f"interrupted during the {hover_phase}"
            print(f"[node]   interrupted during the {hover_phase}")
            # Caught as the mission loop's is (lands, records, rc 0): the
            # Dashboard row ends "stopped", not "ok" (merge of the start-gate
            # branch with the Dashboard heartbeat, 2026-10-22).
            interrupted = True
            if gate_on:
                gate_rec.update(outcome="interrupted", reached=False)
        except Exception as exc:
            # A crash in the hover is no reason to leave the aircraft there:
            # the episode ends with no mission, landed and recorded below, and
            # run() exits non-zero (a failed run, as a crash in the loop is).
            gate_crash = f"{type(exc).__name__}: {exc}"
            traceback.print_exc()
            no_mission = (f"start gate crashed: {gate_crash}" if hover_phase == "start gate"
                          else f"the detector wait crashed: {gate_crash}")
            # Best effort: on a full disk the crash IS this write's error, and
            # a second OSError here left run() with no LAND (review 3).
            watch.record_event("hover_crashed", phase=hover_phase, error=gate_crash)
            print(f"[node]   *** {no_mission}")
            if gate_on:
                if gate is not None:
                    try:
                        gate_rec = {**gate_rec, **gate.record()}
                    except Exception:                             # noqa: BLE001
                        pass
                gate_rec.update(outcome="crashed", reached=False, error=gate_crash)
        finally:
            # HoverWatch.close keeps its own write error (log_error); this
            # guard is for anything else in it - nothing in the hover's
            # bookkeeping may leave run() before the LAND (review 3).
            try:
                watch.close(clock(), out / "hover_log.jsonl")
            except Exception as exc:                              # noqa: BLE001
                watch.log_error = watch.log_error or f"close: {type(exc).__name__}: {exc}"
                print(f"[hover]  *** {watch.log_error}")
        stage = "the mission's start"
        if not detector_ready:
            detector_ready.update(ready=False, outcome="crashed" if gate_crash else "interrupted",
                                  why=no_mission, waited_s=None, seq=None, timeout_s=None,
                                  airborne=True, phase=pre_arm.get("phase"))
        hover_rec = watch.summary()
        if gate_on:
            # The gate's own share of the hover (HoverWatch's "start gate"
            # phase); None when it never hovered, or no tick was checked.
            gate_rec["hover_s"] = hover_rec["s_by_phase"].get("start gate")
            gate_rec["unsafe_ticks"] = (watch.unsafe_by_phase.get("start gate", 0)
                                        if watch.checked_by_phase.get("start gate") else None)
        takeover = watch.takeover

        # ---- loop ----------------------------------------------------------------
        rows, traj, ekf_err = [], [], []
        # Each tick's work before its sleep, on the loop's clock: what a
        # recorder or anything else on the flight path costs, even while the
        # loop still holds its rate.
        work_ms: list[float] = []
        n_touched = n_braked = n_escaped = n_no_state = n_subject = 0
        subject_src: dict = {}
        # Where each estimate's pose and depth came from (estimate_inputs),
        # and how old its box was when it was placed.
        pairing_pose: dict = {}
        pairing_depth: dict = {}
        box_ages: list[float] = []
        # The pacing's running deadline (next_tick_deadline).
        tick_deadline = None
        # Ticks whose state was not live (link_staleness), and each span of
        # them: when it began, why, how many ticks, how long.
        n_stale = 0
        stale_spans: list[dict] = []
        stale_since = None
        freshness_measured = callable(getattr(link, "link_age", None))
        # Whether the last row's tick sent the autopilot a setpoint: a stale
        # span that begins while the node is streaming opens with one hold
        # (STALE_HOLD, at the stale branch).
        streaming = False
        # Each tick's start (s from t0), rows or not: the pacing the loop
        # gate judges (loop_hz_paced), apart from the mission-end LAND and
        # the ticks that wrote no row.
        tick_starts: list[float] = []
        # Row index -> when (s from t0) the first tick after it with no live
        # state began (no state, a stale link): that row's point holds only
        # until then in the dwell (point_durations' `ends`).
        gap_from: dict[int, float] = {}
        # `asks` and `takeover` come from the hover (HoverWatch): a takeover
        # there flies no mission and asks nothing of the autopilot.
        fsm_fault = None
        failsafe_t = None
        end_reason = "max_s reached" if no_mission is None else f"no mission: {no_mission}"
        # The loop's takeover test starts from the hover's last mode (review
        # 2026-10-22), not from a read here: a pilot's switch reported after
        # the hover's last tick (the link is pumped again before the gate's
        # truth verdict) became the baseline, and the loop flew its setpoints
        # over the pilot.
        prev_mode = watch.mode
        fence0 = link.snap.fence_breach_count
        # Read AFTER the start gate: its inferences (it can wait minutes) are
        # not the mission's, for det_hz (follow_vlm counted them once, and
        # reported 6.9-7.5 Hz for a 3.5-3.9 Hz detector; 2026-09-29).
        g0 = grounder.latest()
        det_n0 = (g0.get("n_seen") or 0) + (g0.get("n_miss") or 0)
        gate_rec["det_inferences_before_t0"] = det_n0
        # The FCU clock is paired with the wall time it ARRIVED at, not with
        # "now": SYSTEM_TIME comes at 2 Hz, and pairing a stale FCU stamp with
        # the current wall time read a perfect real-time run as 0.97x.
        tb0 = (link.snap.time_boot_ms, getattr(link.snap, "t_time_boot", None))
        # The FrontCamera frames delivered so far (SemanticObs.stats), so the
        # observation summary counts the mission's own (None: obs says not).
        obs_frames0 = obs_frames_received(obs)
        t0 = clock()
        mission_t0[0] = t0
        if takeover is not None and "t_abs" in takeover:
            # Mission time, as the loop's takeovers and autopilot_events carry
            # it: negative, before the clock.
            takeover["t"] = round(takeover.pop("t_abs") - t0, 3)
        rec.event("mission_start", t=0.0, autopilot_mode=prev_mode,
                  fsm=None if fsm is None else fsm.config.to_dict(),
                  rule_clock_reading=rule_now().isoformat(timespec="seconds"))
        if (not vla_on and no_mission is None
                and getattr(pilot, "track", None) is not None):
            # THE ESTIMATOR FOLLOW's SubjectTrack, started as the VLA's is.
            start_subject_track(pilot.track, gate, gate_rec, t0)
        if vla_on and no_mission is None:
            track = vla_rec.get("track")
            if track is not None:
                # THE TARGET ESTIMATOR starts with the mission
                # (start_subject_track).
                start_subject_track(track, gate, gate_rec, t0)
            # THE VLA PILOT, on the mission's clock: its stop rules
            # (vla_mission.MissionMonitor) start now.
            vla = VLAPilot(vla_backend, VM.MissionMonitor(args.vla_monitor, t0),
                           mission=args.mission, goal_xy=args.goal if vla_goto else None,
                           detections=None if vla_goto else grounder,
                           hint_source=vla_rec["hint_source"],
                           detector_hint=vla_rec["detector_hint"], stale_s=args.det_fresh_s,
                           server_alive=lambda: vla_srv.alive, vm=VM,
                           model_label=VM.model_label(vla_identity),
                           close_hold=vla_rec.get("close_rule"), track=track)
            print(f"[vla]    mission {args.mission}: {VM.model_words(vla_identity)} flies"
                  + (f" to {args.goal}" if vla_goto else f", hint from the box on {args.object!r}")
                  + " - every tick through the Shield"
                  + (" and the escalation FSM" if fsm is not None else ""))
        # The two simulator checks (SIM_CHECKS, below), half a period apart.
        t_frame, t_geo = -math.inf, t0 - TRUTH_FRAME_PERIOD_S / 2.0
        if no_mission is not None:
            print(f"[node]   *** {end_reason}; "
                  + ("the autopilot has the aircraft: nothing is asked of it"
                     if takeover is not None else "landing"))
        # The net's mission_end names where run() was left (re-review of
        # baa4603, N2: a Ctrl+C in the loop's finally after a 30-tick mission
        # read "interrupted during the mission's start").
        stage = "the mission"
        # THE GARBAGE COLLECTOR (guardrail.gc_pacing): a full collection is
        # the loop's own, after a tick's work, never inside one.
        gc_pacer = GcPacer(pace=getattr(args, "gc_pacing", "on") == "on")
        if no_mission is None:
            gc_pacer.start(0.0)
        try:
            while no_mission is None and clock() - t0 < args.max_s:
                if hot_gate is not None:
                    # The tick's first part - the link's state, the pilot's
                    # step, the subject's estimate - asks nothing of the
                    # Shield, so a REST event may land through it; the lock is
                    # taken again where the Shield's inputs are set
                    # (rail_hotapply, THE LOCK; since 2026-11-15).
                    hot_gate.let_go()
                t_tick = clock()
                tick_starts.append(t_tick - t0)
                live.beat(progress=(t_tick - t0) / max(float(args.max_s), 1.0),  # memory only
                          **({"detail": vla.detail()} if vla is not None and vla.detail()
                             else {}))
                link.pump()
                st = link.state(anchor)
                if st is None:
                    n_no_state += 1
                    if rows and len(rows) - 1 not in gap_from:
                        gap_from[len(rows) - 1] = round(t_tick - t0, 3)
                    _now = clock()
                    tick_deadline = next_tick_deadline(tick_deadline, t_tick, _now)
                    sleep(max(0.0, tick_deadline - _now))
                    continue
                # A position or heartbeat the link stopped refreshing is not
                # the aircraft's state (POS_STALE_S): nothing is commanded
                # from it, the FSM is not stepped and no row is written, as
                # on a tick without state; LINK_LOST_S of it ends the episode.
                stale, ages = link_staleness(link)
                if stale is not None:
                    n_stale += 1
                    if rows and len(rows) - 1 not in gap_from:
                        gap_from[len(rows) - 1] = round(t_tick - t0, 3)
                    if stale_since is None:
                        stale_since = t_tick
                        if vla is not None:
                            # The VLA's step is not being flown through the
                            # span (one zero hold, then nothing): the tracker
                            # is suspended and the mission's timers stop until
                            # a tick flies again (VLAPilot.hold; review of
                            # 4a0da6d, P3-13).
                            vla.hold(t_tick)
                        # STALE_HOLD (review 2026-10-19). Silence alone left
                        # the aircraft flying the last velocity setpoint until
                        # ArduCopter's GUID_TIMEOUT (3 s) - a setpoint itself
                        # computed on the position that froze up to
                        # POS_STALE_S earlier - so an INBOUND-only failure (a
                        # starved MAVROS subscriber, a one-way NAT drop) could
                        # fly on blind for ~4 s at up to speed_max. If the
                        # outbound path still works one explicit zero-velocity
                        # setpoint stops it now; if the link is dead it is
                        # harmless. Sent once per span, and only while the
                        # node was streaming (never over an autopilot that
                        # owns the aircraft, a fault's hold or the FSM's own
                        # mode); after it, nothing until the state is live.
                        hold_sent = None
                        if streaming and takeover is None and fsm_fault is None:
                            try:
                                link.send(Action4D())
                                hold_sent = True
                            except Exception as exc:              # noqa: BLE001
                                hold_sent = False
                                print(f"[link]   hold setpoint failed: {exc}")
                            streaming = False
                        # A DEFERRED HOLD IS FLOWN BY THE STREAM (review
                        # 2026-10-20): while the FSM's Loiter is held in
                        # GUIDED, nothing but this node's setpoints stops
                        # the aircraft, and the stream has just stopped.
                        # Hand the hold to BRAKE now - the autopilot then
                        # holds it with no companion involvement, as the
                        # Loiter did before the hold that stops clear.
                        if hold_deferred and takeover is None and fsm_fault is None:
                            hold_deferred, hold_why = False, None
                            why = f"link stale ({stale}): the stream that flew the hold stopped"
                            ap_mode = asks.ask("LOITER")
                            sent = link.request_mode(ap_mode)
                            forced_handovers["link_stale"] = forced_handovers.get(
                                "link_stale", 0) + 1
                            hold_plans.append({"t": round(t_tick - t0, 3), "tick": len(rows) + 1,
                                               "at": "handover", "choice": "brake",
                                               "forced": "link_stale", "why": why})
                            rec.event("mode_request", t=t_tick - t0, mode="LOITER",
                                      autopilot_mode=ap_mode,
                                      why="the deferred hold handed to BRAKE: " + why,
                                      sent=bool(sent))
                            print(f"[hold]   LOITER handed to {ap_mode}: {why}")
                        stale_spans.append({"t": round(t_tick - t0, 3), "why": stale,
                                            "ticks": 0, "s": None, "recovered": False,
                                            "hold_sent": hold_sent})
                        rec.event("link_stale", t=t_tick - t0, why=stale,
                                  ages={k: round(v, 3) if isinstance(v, float) else v
                                        for k, v in (ages or {}).items()},
                                  hold_sent=hold_sent)
                        print(f"[link]   *** state not live ({stale}): "
                              + ("one zero-velocity hold sent, then " if hold_sent else "")
                              + "nothing is commanded until it is")
                    stale_spans[-1]["ticks"] += 1
                    stale_spans[-1]["s"] = round(t_tick - stale_since, 3)
                    if t_tick - stale_since >= LINK_LOST_S:
                        end_reason = (f"link lost: {stale}, for "
                                      f"{t_tick - stale_since:.1f} s")
                        print(f"[link]   *** {end_reason}; the episode ends")
                        break
                    _now = clock()
                    tick_deadline = next_tick_deadline(tick_deadline, t_tick, _now)
                    sleep(max(0.0, tick_deadline - _now))
                    continue
                if stale_since is not None:
                    stale_spans[-1].update(recovered=True,
                                           s=round(t_tick - stale_since, 3))
                    rec.event("link_fresh", t=t_tick - t0,
                              stale_s=round(t_tick - stale_since, 3),
                              stale_ticks=stale_spans[-1]["ticks"])
                    print(f"[link]   state live again after "
                          f"{t_tick - stale_since:.1f} s")
                    stale_since = None
                now = t_tick - t0
                s = link.snap
                mode = s.mode
                if fence0 is None:
                    fence0 = s.fence_breach_count
                # The autopilot left GUIDED - or a hold this node asked for -
                # for a mode the node did not ask for (autopilot_took_over, the
                # hover's test too): from here it has the aircraft, and nothing
                # more is commanded.
                if mode == "GUIDED" and prev_mode != "GUIDED":
                    asks.entered_guided()
                if takeover is None and autopilot_took_over(mode, prev_mode, asks):
                    takeover = {"t": round(now, 3), "mode": mode, "previous": prev_mode,
                                "fence_breach_count": s.fence_breach_count,
                                "statustext": s.statustext[-3:], "phase": "mission"}
                    rec.event("autopilot_took_over", t=now, mode=mode, previous=prev_mode)
                    print(f"[link]   *** the autopilot took the aircraft: GUIDED -> "
                          f"{mode}; commanding stops")
                prev_mode = mode
                yaw = math.radians(st.yaw_deg)
                obs.put_pose_full(t_tick, st.x, st.y, st.up, yaw, s.pitch or 0.0, s.roll or 0.0)
                if vla is None and getattr(pilot, "track", None) is not None:
                    # THE ESTIMATOR FOLLOW: it steps on the aircraft's pose,
                    # and the link's measured velocity feeds its line-of-
                    # sight rate (None: the last commanded forward speed).
                    vel_p = (link.velocity()[0] if callable(getattr(link, "velocity", None))
                             else None)
                    body, info = pilot.step(t_tick, st.up, pose=(st.x, st.y, yaw), vel=vel_p)
                    raw = body_to_world(*body, yaw)
                elif vla is None:
                    body, info = pilot.step(t_tick, st.up)
                    raw = body_to_world(*body, yaw)
                else:
                    # THE VLA PILOT: the step tracker's world-frame action IS
                    # the raw action; from here the detector pilot's path.
                    raw, body, info = vla.step(t_tick, st)
                subj = subj_depth = pairing = None
                if info["fresh"]:
                    # Placed from the box's own capture: its pose and its
                    # depth frame's range (estimate_inputs), not this tick's.
                    e_st, e_yaw, e_pitch, depth, cap_range, w_src, pairing = estimate_inputs(
                        info, st, yaw, s.pitch or 0.0, obs, t_tick)
                    subj = subject_estimate(info["det"], depth, e_st, e_yaw, hfov, width_m,
                                            fv, body_pitch_rad=e_pitch, range_m=cap_range,
                                            width_src=w_src)
                    subj_depth = depth_check(info["det"], depth, e_st, hfov, fv,
                                             body_pitch_rad=e_pitch, range_m=cap_range)
                    pairing_pose[pairing["pose"]] = pairing_pose.get(pairing["pose"], 0) + 1
                    pairing_depth[pairing["depth"]] = pairing_depth.get(pairing["depth"], 0) + 1
                    if pairing["box_age_s"] is not None:
                        box_ages.append(pairing["box_age_s"])
                if hot_gate is not None:
                    # From the Shield's inputs to the tick's records the
                    # Shield is this thread's (rail_hotapply, THE LOCK).
                    hot_gate.hold()
                # With no class (unclassified) only the catch-all rules bind:
                # SubjectStandoff.binds(None) is True for "*" alone.
                if subj is not None:
                    shield.set_subject(subj[0], subj[1], subject_class)
                    n_subject += 1
                    subject_src[subj[2]] = subject_src.get(subj[2], 0) + 1
                else:
                    shield.set_subject(None)
                # The autopilot's measured velocity: logged on every tick
                # (state_v), given to the Shield only with the own-velocity
                # forecast on (OWN_VELOCITY_DEFAULT); the Shield's position
                # questions (state_is_unsafe, unsafe_depth) ignore it.
                vel, vel_src = (link.velocity() if callable(getattr(link, "velocity", None))
                                else (None, "the link reads no velocity"))
                vkey = vel_src if vel is not None else f"none: {vel_src}"
                vel_sources[vkey] = vel_sources.get(vkey, 0) + 1
                st_sh = st
                if own_v and vel is not None:
                    st_sh = st.model_copy(update={"velocity": vel})
                    n_vel_used += 1
                d = shield.filter(st_sh, raw)
                # The rule clock's reading for this tick, taken with the
                # Shield's: the trajectory point and the row carry this one,
                # not a later reading after a send or a fence upload.
                clock_now = rule_now()
                tick = len(rows) + 1
                # The episode id binds every row to this episode's events
                # (guardrail.replay: "ONE EPISODE, NOT TWO").
                extra = episode_row_fields(shield, st, policy, rec.episode_id)
                emitted = d.emitted if shield_on else raw

                # ---- escalation FSM (Shield on only) -------------------------
                fsm_out, setpoint, stop_illegal, unsafe_depth = None, "pass", None, None
                # What the hold that stops clear did on this tick: ("deferred",
                # None) - the FSM's LOITER asked for and BRAKE not; ("handover",
                # mode) - BRAKE asked for on a tick with no FSM mode request.
                hold_note = None
                if fsm is not None and fsm_fault is None and takeover is None:
                    try:
                        in_failsafe = fsm.state in FAILSAFE_STATES
                        stop_illegal = shield.state_is_unsafe(st)
                        # How far from legal the position is: X6's progress
                        # test and X7's losing-ground test read its trend.
                        unsafe_depth = shield.unsafe_depth(st, stop_illegal)
                        on_ground = (s.armed is False
                                     or s.landed_state == LANDED_STATE_ON_GROUND)
                        fsm_out = fsm.step(tick_input_from_decision(
                            now, d, policy, horizon_s=horizon,
                            home_reached=bool(in_failsafe and mode == "RTL"
                                              and home_xy is not None
                                              and math.hypot(st.x - home_xy[0],
                                                             st.y - home_xy[1]) < REACH_M),
                            landed=bool(in_failsafe and mode in ("RTL", "LAND")
                                        and on_ground),
                            stop_illegal=stop_illegal, unsafe_depth_m=unsafe_depth,
                            # The caps the Shield clamped against on this tick
                            # (Shield.applied_caps), not every cap in the policy.
                            caps=shield.applied_caps or False))
                        setpoint = fsm_out.setpoint
                        if fsm_out.transition:
                            rec.event("fsm_transition", t=now, **{k: fsm_out.record[k] for k in (
                                "fsm_state_before", "fsm_state_after", "transition",
                                "edge", "edge_source", "reason", "fsm_config_hash")})
                        plan = None
                        if (fsm_out.set_mode == "LOITER" and own_v
                                and hold_ap_mode == "BRAKE"):
                            # THE HOLD THAT STOPS CLEAR (guardrail.response.
                            # hold_plan): BRAKE only if its predicted stop path
                            # ends legal and the aircraft is slow; else the
                            # Loiter is flown as a GUIDED stop through the
                            # Shield and BRAKE is asked for once it would.
                            # The FSM's state and edges are not touched.
                            plan = vresp.hold_plan(shield, st_sh)
                            hold_plans.append({"t": round(now, 3), "tick": len(rows) + 1,
                                               "at": "loiter entry", **plan})
                        if fsm_out.set_mode == "LOITER":
                            hold_reclaims_loiter, hold_reclaimed = 0, False
                            brake_was_legal = False
                        if plan is not None and plan["choice"] == "guided":
                            hold_deferred = True
                            hold_deferred_since = now
                            hold_why = vresp.hold_banner_reason(plan)
                            hold_note = ("deferred", None)
                            rec.event("hold_deferred", t=now, why=plan["why"],
                                      stop_xy=plan["stop_xy"], breaks=plan["breaks"])
                            print(f"[hold]   LOITER held in GUIDED through the Shield: "
                                  f"{plan['why']}")
                        elif fsm_out.set_mode:
                            hold_deferred, hold_why = False, None
                            # The FSM's vocabulary on record, ArduCopter's mode
                            # sent: its LOITER is flown as BRAKE (HOLD_MODE).
                            ap_mode = asks.ask(fsm_out.set_mode)
                            sent = link.request_mode(ap_mode)
                            rec.event("mode_request", t=now, mode=fsm_out.set_mode,
                                      autopilot_mode=ap_mode,
                                      why=fsm_out.reason, sent=bool(sent))
                            print(f"[fsm]    {fsm_out.set_mode} requested as {ap_mode}: "
                                  f"{fsm_out.reason}")
                        elif hold_deferred and fsm.state == FSMState.LOITER:
                            plan = vresp.hold_plan(shield, st_sh)
                            # A hand-over the plan did not choose (review
                            # 2026-10-20): while deferred, the hold is flown
                            # by this node's stream, which ArduCopter keeps
                            # flying for up to GUID_TIMEOUT if the node
                            # stalls. A gap in the stream longer than
                            # HOLD_STREAM_GAP_S (the 14 Oct log has a 0.73 s
                            # one), or a deferral longer than HOLD_DEFER_MAX_S,
                            # hands the hold to BRAKE whatever the plan says.
                            # The gap up to NOW, not to the tick's start: a
                            # stall anywhere since the last send counts.
                            forced, why = vresp.hold_forced_handover(
                                deferred_s=now - (hold_deferred_since if hold_deferred_since
                                                  is not None else now),
                                stream_gap_s=(None if t_last_sent is None
                                              else clock() - t_last_sent))
                            if (plan["choice"] == "brake" and not hold_reclaimed) or forced:
                                hold_deferred, hold_why, hold_reclaimed = False, None, False
                                # legal where BRAKE takes the hold: a later
                                # illegal position is a drift into the rule
                                brake_was_legal = not shield.stop_breaks(st_sh)
                                if forced:
                                    forced_handovers[forced] = forced_handovers.get(forced, 0) + 1
                                    plan = dict(plan, choice="brake", forced=forced,
                                                why=f"{why} (the plan: {plan['why']})")
                                hold_plans.append({"t": round(now, 3), "tick": len(rows) + 1,
                                                   "at": "handover", **plan})
                                ap_mode = asks.ask("LOITER")
                                sent = link.request_mode(ap_mode)
                                hold_note = ("handover", ap_mode)
                                rec.event("mode_request", t=now, mode="LOITER",
                                          autopilot_mode=ap_mode,
                                          why="the deferred hold handed to BRAKE: "
                                              + plan["why"], sent=bool(sent))
                                print(f"[hold]   LOITER handed to {ap_mode}: {plan['why']}")
                            else:
                                hold_why = vresp.hold_banner_reason(plan)
                        elif (not hold_deferred and own_v and hold_ap_mode == "BRAKE"
                              and fsm.state == FSMState.LOITER and mode == hold_ap_mode
                              and hold_reclaims_loiter < vresp.HOLD_RECLAIM_MAX
                              and (brake_was_legal or not shield.stop_breaks(st_sh))):
                            # A BRAKE THAT DRIFTS (guardrail.response.
                            # HOLD_RECLAIM_MAX): the hold was handed to BRAKE
                            # with its stop predicted legal, and the position
                            # has gone illegal on a hard rule since it was
                            # legal in BRAKE. Back into GUIDED, flown as the
                            # Shield's answer to a stop, until a forced
                            # hand-over or the end of the Loiter.
                            drifted = shield.stop_breaks(st_sh)
                            if not drifted:
                                brake_was_legal = True
                            if drifted and brake_was_legal:
                                brake_was_legal = False
                                hold_reclaims_loiter += 1
                                n_hold_reclaims += 1
                                hold_deferred, hold_deferred_since = True, now
                                hold_reclaimed = True
                                plan = vresp.hold_plan(shield, st_sh)
                                hold_why = "BRAKE DRIFTED INTO " + ", ".join(drifted)
                                hold_plans.append({"t": round(now, 3), "tick": len(rows) + 1,
                                                   "at": "reclaim", "drifted_into": drifted,
                                                   "unsafe_depth_m": unsafe_depth, **plan})
                                ap_mode = asks.ask("GUIDED")
                                sent = link.request_mode(ap_mode)
                                hold_note = ("reclaim", ap_mode)
                                rec.event("hold_reclaimed", t=now, drifted_into=drifted,
                                          unsafe_depth_m=unsafe_depth,
                                          reclaim=hold_reclaims_loiter, sent=bool(sent))
                                rec.event("mode_request", t=now, mode="GUIDED",
                                          autopilot_mode=ap_mode,
                                          why="BRAKE let the position go illegal on "
                                              + ", ".join(drifted) + ": the hold taken back "
                                              "into GUIDED", sent=bool(sent))
                                print(f"[hold]   BRAKE drifted into {', '.join(drifted)} "
                                      f"({unsafe_depth:.2f} m): the Loiter taken back into "
                                      f"GUIDED through the Shield "
                                      f"({hold_reclaims_loiter}/{vresp.HOLD_RECLAIM_MAX})")
                        if hold_deferred and fsm.state != FSMState.LOITER:
                            hold_deferred, hold_why = False, None
                        if fsm_out.state in FAILSAFE_STATES and failsafe_t is None:
                            failsafe_t = now
                    except ValueError as exc:
                        # A Shield/FSM contract break mid-flight: hold, say why,
                        # stop streaming. The run is not KPI-grade.
                        fsm_fault = str(exc)
                        hold_deferred, hold_why = False, None
                        rec.event("fsm_fault", t=now, error=fsm_fault)
                        hold = asks.ask("LOITER")
                        link.request_mode(hold)
                        failsafe_t = now
                        print(f"[fsm]    input refused, {hold}: {exc}")
                if fsm_fault is not None or takeover is not None:
                    setpoint = "none"
                    # Nothing this node streams flies now: no hold is deferred.
                    hold_deferred, hold_why = False, None
                # A STOP THAT STOPS CLEAR (own-velocity forecast on): a Loiter
                # held in GUIDED streams the Shield's answer to a stop
                # (Shield.advise of a zero action), and so does the FSM's
                # "brake" setpoint - a zero velocity there is where a stop
                # ENDS that the Shield judged, not where it starts. Where the
                # stop ends legal the answer is the zero itself.
                fsm_setpoint, flown_d, stop_advised = setpoint, d, False
                if shield_on and fsm_fault is None and takeover is None and st_sh is not st:
                    if hold_deferred and fsm is not None and fsm.state == FSMState.LOITER:
                        flown_d, setpoint, stop_advised = shield.advise(st_sh, Action4D()), "pass", True
                        n_hold_guided += 1
                    elif setpoint == "brake":
                        flown_d, setpoint, stop_advised = shield.advise(st_sh, Action4D()), "pass", True
                        n_stop_advised += 1
                # The literal LOITER hold centres the throttle while the FSM
                # (or a fault) holds the aircraft AND the autopilot is in the
                # hold mode (or still in GUIDED, the request not yet reported,
                # where the throttle is not read), and hands it back after -
                # at once if the autopilot took the aircraft or left the hold
                # for any other mode (ThrottleHold).
                holding = (takeover is None
                           and link.snap.mode in (hold_ap_mode, "GUIDED")
                           and (fsm_fault is not None
                                or (fsm is not None and fsm.state == FSMState.LOITER)))
                rc3 = throttle.tick(
                    now, holding, s,
                    why=(f"the autopilot took the aircraft ({takeover['mode']})"
                         if takeover is not None else
                         f"the autopilot is in {link.snap.mode}, not {hold_ap_mode}"
                         if link.snap.mode not in (hold_ap_mode, "GUIDED") else
                         "the hold ended"))
                # After the FSM stepped, with ITS verdict: one state machine on
                # record, in audit.jsonl and flight_log.jsonl alike.
                audit_tick(audit, tick, d, fsm_out=fsm_out, fsm_fault=fsm_fault)
                if stop_advised:
                    # THE DECISION THAT FLEW (review 2026-10-20): on a stop-
                    # advised tick the action sent is the Shield's answer to a
                    # stop (flown_d), not this tick's decision on the pilot's
                    # action (d, the record above). It gets its own record,
                    # always - its repairs, trace and repair passes - tagged
                    # decision_kind "stop_advised", so audit.jsonl holds the
                    # decision behind every action ArduCopter was sent.
                    audit_tick(audit, tick, flown_d, fsm_fault=fsm_fault,
                               kind="stop_advised")
                flown = {"pass": flown_d.emitted if stop_advised else emitted,
                         "brake": Action4D(), "none": None}[setpoint]
                # The publication's time on the loop clock (CAMERA TO
                # SETPOINT): the row's t_pub / tick_ms and the age of the
                # camera frame behind the flown action.
                t_pub_abs = None
                if flown is not None:
                    link.send(flown, raw_body=body)
                    t_last_sent = t_pub_abs = clock()
                streaming = flown is not None
                # What was sent, for the own-velocity forecast's pipe (a
                # setpoint is flown from the response's reaction time after it
                # goes out), aged on the loop's real clock (pipe_clock). Read
                # only for a State with a velocity, so harmless with the flag off.
                shield.note_flown(flown)
                # A zone held back from the autopilot's fence goes up once the
                # aircraft is clear of it (FenceBackstop).
                if backstop is not None and takeover is None:
                    backstop.recheck((st.x, st.y), extra.get("unsafe_rules"), t=now)
                # The subject's simulator pose: evaluation only, read after the
                # setpoint went out, so its RPC is never between the action
                # and the autopilot (review 2026-10-13); the Shield above was
                # given the estimate. This read and the frame / geo check below
                # share one SIM_RPC_WAIT_S (OffLoopRpc.begin_tick).
                rpc.begin_tick()
                tr = truth.read()
                truth_xy = tr["xy"]
                # SIM_CHECKS, RECORD ONLY, after the setpoint went out: each
                # every TRUTH_FRAME_PERIOD_S, half a period apart, and never
                # both on one tick (the geo read waits a tick if they meet).
                # (a) The truth's frame check: the aircraft through the
                # subject's own RPC and projection, against the EKF position
                # the Shield was given above. (b) The EKF against the
                # simulator's kinematics (ekf_vs_sim_horiz_m), which the frame
                # check needs to tell a truth in another frame from an EKF off
                # the simulator. Nothing above read either.
                frame_chk = None
                if t_tick - t_frame >= TRUTH_FRAME_PERIOD_S:
                    t_frame = t_tick
                    frame_chk = truth.check_frame(st)
                elif (t_tick - t_geo >= TRUTH_FRAME_PERIOD_S and camera is not None
                      and hasattr(camera, "truth_geo")):
                    t_geo = t_tick
                    # Through the RPC worker too (OffLoopRpc): an answer later
                    # than what is left of the tick's SIM_RPC_WAIT_S is
                    # dropped, and this check skipped.
                    g_st, g = rpc.call(camera.truth_geo)
                    if g_st != "ok":
                        g = None
                    if g is not None and s.lat is not None:
                        gx, gy = anchor.to_scene(g["lat"], g["lon"])
                        ex, ey = anchor.to_scene(s.lat, s.lon)
                        ekf_err.append(math.hypot(gx - ex, gy - ey))
                        truth.note_ekf_vs_sim(ekf_err[-1])
                rpc.end_tick()

                touched = bool(shield_on and d.touched)
                n_touched += touched
                n_braked += bool(shield_on and d.braked)
                # What was FLOWN, and the Shield's check on it, decided by the
                # rule both SITL rails share (guardrail.replay.flown_fields):
                # the re-check of the Shield's output (on) or the raw action's
                # violations (off); standing still in Brake; and NOTHING on a
                # tick nothing was sent (the autopilot or a fault held the
                # aircraft) - no action flew, so none can have escaped.
                ff = flown_fields(shield, st, flown_d, shield_on=shield_on, setpoint=setpoint)
                n_escaped += bool(ff["emitted_violations"])
                # The same P0 escape on the stored runs' basis (review
                # 2026-10-20): where the Shield flew with a measured velocity,
                # the flown action re-checked from the position on the
                # commanded forecast (Shield.recheck_commanded); elsewhere the
                # two bases are one re-check. Never set where the rail's own
                # (velocity) re-check is clear on an action it emitted clean.
                p0_ids = {c.id for c in policy.constraints if c.priority == "P0"}
                if ff["flown"] and setpoint == "pass" and shield_on and st_sh is not st:
                    p0_cmd = any(v.rule_id in p0_ids
                                 for v in shield.recheck_commanded(st, flown))
                else:
                    p0_cmd = any(v["rule_id"] in p0_ids for v in ff["emitted_violations"])
                n_p0_cmd_basis += p0_cmd
                # The rule clock's reading rides with each point, so the zone
                # dwell applies the windows the Shield flew under.
                traj.append({"t": round(now, 2), "x": st.x, "y": st.y, "up": st.up,
                             "touched": touched, "clock": clock_now.isoformat()})
                truth_fields = {"subject_truth": None if truth_xy is None else
                                [round(truth_xy[0], 2), round(truth_xy[1], 2)]}
                if truth_xy is None:
                    truth_fields["subject_truth_why"] = tr["why"]
                else:
                    # The repo's subject-position convention (both SITL rails,
                    # the AirSim follows): guardrail.kpi's hover null,
                    # demo/track_truth and tools/rescore_kpis read tgt_x/tgt_y.
                    # Here it is the TRUTH; what the Shield flew on is `subject`.
                    truth_fields["tgt_x"] = round(truth_xy[0], 3)
                    truth_fields["tgt_y"] = round(truth_xy[1], 3)
                    # The true horizontal range, and the estimate's error.
                    truth_fields["subject_range_m"] = round(
                        math.hypot(st.x - truth_xy[0], st.y - truth_xy[1]), 3)
                    truth_fields["subject_gap_m"] = (
                        None if subj is None else
                        round(math.hypot(subj[0] - truth_xy[0], subj[1] - truth_xy[1]), 3))
                    # Where the subject truly was against the camera's nose,
                    # and the estimate's bearing error (truth_view).
                    tb = math.degrees(math.atan2(truth_xy[1] - st.y, truth_xy[0] - st.x))
                    truth_fields["subject_truth_off_nose_deg"] = round(
                        _wrap_deg(tb - st.yaw_deg), 2)
                    if subj is not None:
                        eb = math.degrees(math.atan2(subj[1] - st.y, subj[0] - st.x))
                        truth_fields["subject_bearing_gap_deg"] = round(
                            abs(_wrap_deg(tb - eb)), 2)
                if frame_chk is not None:
                    # The aircraft's own frame check (the subject's position
                    # is not in it, so it stays when rows are quarantined).
                    truth_fields["subject_frame_err_m"] = round(frame_chk["err_m"], 3)
                    if frame_chk["yaw_err_deg"] is not None:
                        truth_fields["subject_frame_yaw_err_deg"] = round(
                            frame_chk["yaw_err_deg"], 2)
                vla_end = vla_row = vla_hud = None
                if vla is not None:
                    # Who authored the tick and the decision behind it, then
                    # the hold: a tick the pilot's action did not fly (the
                    # FSM's brake or a stop it advised, a hold, a fault, a
                    # takeover) suspends the tracker; then the VLA mission's
                    # stop rules (its LAND among them).
                    vla_end, vla_row, vla_hud = vla.after(
                        t_tick, st, flies=(flown is not None and setpoint == "pass"
                                           and not stop_advised),
                        touched=touched, shield_on=shield_on, t_mission=now,
                        fsm_stepped=fsm_out is not None, t_pub=t_pub_abs)
                elif getattr(pilot, "track", None) is not None:
                    # THE ESTIMATOR FOLLOW's est_* fields and the law that
                    # flew the tick (EstimatorFollowPilot.row_fields), with
                    # est_gap_m against the truth below, as the VLA follow's.
                    vla_row = pilot.row_fields(t_tick, st)
                row = {
                    "t": round(now, 3), "tick": tick,
                    # When this tick's setpoint went out, on the loop clock
                    # (None: nothing sent), and the tick's start to it.
                    **publish_latency_fields(t_tick, t_pub_abs, t0),
                    "x": round(st.x, 3), "y": round(st.y, 3), "up": round(st.up, 3),
                    "yaw_deg": round(st.yaw_deg, 2),
                    # The autopilot's measured velocity (North, East, up, m/s)
                    # and its source; None with the reason when the link had
                    # none fresh. Given to the Shield only with the
                    # own-velocity forecast on (shield_velocity).
                    "state_v": None if vel is None else [round(c, 3) for c in vel],
                    "state_v_source": vel_src,
                    "shield_velocity": st_sh is not st,
                    # Body pitch (ATTITUDE, nose up positive): the depth rules
                    # read it, and a replay cannot judge them without it.
                    "pitch_deg": (None if s.pitch is None
                                  else round(math.degrees(s.pitch), 2)),
                    "body": [round(v, 4) for v in body],
                    "raw": raw.model_dump(), "emitted": emitted.model_dump(),
                    # The action sent to the autopilot (None: nothing sent);
                    # "flown" (a bool) and "emitted_violations" come from
                    # flown_fields, the shape guardrail/kpi.py reads.
                    "flown_action": None if flown is None else flown.model_dump(),
                    # `emitted_violations` re-checks `flown_action` - the
                    # action that FLEW - which on a stop-advised tick is
                    # `stop_advice.emitted`, not `emitted` (the decision on the
                    # pilot's action, whose violations, repairs, braked and
                    # touched are the fields below).
                    **ff,
                    "p0_escape_commanded_basis": p0_cmd,
                    "setpoint": setpoint,
                    # The hold that stops clear: the FSM's own setpoint when
                    # the rail flew the Shield's answer to a stop instead
                    # (stop_advised), and whether the Loiter was held in GUIDED.
                    "fsm_setpoint": fsm_setpoint,
                    "stop_advised": stop_advised,
                    "hold_deferred": hold_deferred,
                    # The decision that flew on a stop-advised tick (Shield.
                    # advise of a zero action; audit.jsonl has its full record,
                    # decision_kind "stop_advised"); None on every other tick.
                    "stop_advice": (None if not stop_advised else {
                        "emitted": flown_d.emitted.model_dump(),
                        "violations": [v.model_dump() for v in flown_d.violations],
                        "repairs": [r.model_dump() for r in flown_d.repairs],
                        "emitted_violations": [v.model_dump()
                                               for v in flown_d.emitted_violations],
                        "braked": bool(flown_d.braked), "touched": bool(flown_d.touched)}),
                    "violations": [v.model_dump() for v in d.violations],
                    "repairs": [r.model_dump() for r in d.repairs] if shield_on else [],
                    "braked": bool(shield_on and d.braked), "touched": touched,
                    "mode": info["mode"], "det_seq": info["seq"],
                    # What the autopilot was doing on this tick, by its own report.
                    "ap_mode": mode, "armed": s.armed,
                    "subject": None if subj is None else [round(subj[0], 2), round(subj[1], 2), subj[2]],
                    # The depth range in the box and the rule that refused it, if any.
                    "subject_depth": subj_depth,
                    # Whose pose and depth placed the box, and the box's age.
                    "subject_pairing": pairing,
                    # The subject's simulator pose and the true range: the
                    # record's, never the Shield's (None with the reason).
                    **truth_fields,
                    # The throttle override sent this tick (--hold-mode LOITER).
                    "rc3_override": rc3,
                    # The rule clock's reading the Shield evaluated this tick at.
                    "rule_clock": clock_now.isoformat(timespec="seconds"),
                    **extra,
                }
                if fsm_out is not None:
                    r_ = fsm_out.record
                    row.update({
                        "fsm_state_before": fsm_out.before.value,
                        "fsm_state_after": fsm_out.state.value,
                        "fsm_edge": fsm_out.edge, "set_mode": fsm_out.set_mode,
                        # What ArduCopter was asked for on this tick: the
                        # FSM's mode as flown, or the hold that stops clear's
                        # (None while the Loiter is held in GUIDED, BRAKE on
                        # the tick it is handed over).
                        "autopilot_set_mode": (hold_note[1] if hold_note is not None else
                                               autopilot_mode(fsm_out.set_mode, hold_mode)
                                               if fsm_out.set_mode else None),
                        "stop_illegal": bool(stop_illegal),
                        "unsafe_depth_m": (None if unsafe_depth is None
                                           else round(unsafe_depth, 4)),
                        "failsafe": bool(fsm_out.transition
                                         and fsm_out.state in FAILSAFE_STATES),
                        "fsm": {k: r_.get(k) for k in (
                            "outcome", "theta_exceeded", "n_in_t", "onset_merged",
                            "reason", "risk_level", "magnitude", "x6_progress",
                            "x7_losing", "fsm_config_hash")}})
                elif not shield_on:
                    row["fsm_state_after"] = None
                if vla_row is not None:
                    row.update(vla_row)
                    if truth_xy is not None and row.get("est_xy") is not None:
                        # The target estimate's error (evaluation only).
                        row["est_gap_m"] = round(math.hypot(row["est_xy"][0] - truth_xy[0],
                                                            row["est_xy"][1] - truth_xy[1]), 3)
                rows.append(row)
                if recorder is not None:
                    # The demo video's state, handed over AFTER the setpoint
                    # went out and the truth was read: the indicator's
                    # rule_status and a dict under a lock. Everything else is
                    # the recorder thread's.
                    _th = time.perf_counter()
                    try:
                        hud = rhud.tick(
                            now, decision=hud_decision(d, raw, shield_on=shield_on,
                                                       setpoint=setpoint,
                                                       advised=flown_d if stop_advised
                                                       else None),
                            # The hold that stops clear on screen (review
                            # 2026-10-20): a Loiter held in GUIDED, and a
                            # Brake whose stop the Shield advised.
                            hold_deferred=(hold_why or "BRAKE DEFERRED") if hold_deferred
                            else None,
                            stop_advised=advised_banner_rules(flown_d) if stop_advised
                            else None,
                            state=st, yaw_rad=yaw, info=info,
                            subject_xy=None if subj is None else (subj[0], subj[1]),
                            subject_class=subject_class,
                            fsm_state=None if fsm is None else fsm.state,
                            takeover_mode=None if takeover is None else takeover["mode"],
                            fsm_fault=fsm_fault, setpoint=setpoint, flown=flown,
                            # No true range on screen from a truth in another frame.
                            sep_m=(None if truth.frame_disagrees()
                                   else truth_fields.get("subject_range_m")),
                            touched=touched)
                        if vla_hud is not None:
                            # The city rail's VLA panel (vla_mission.hud_lines)
                            # beside this rail's banner and rule panel, with
                            # this rail's own label kept: the FSM's state, an
                            # autopilot that took over, SHIELD OFF
                            # (vla_mission.panel_lines; review of 4a0da6d, P3-14).
                            rhud.panel_replaced(hud)
                            hud.update(vla_hud,
                                       rail_label=str(hud.get("mode_label") or "")
                                       .split(" | ", 1)[-1] or None)
                        recorder.set_hud(hud, info["det"] if info["fresh"] else None)
                    except Exception as exc:                  # noqa: BLE001
                        # A recording fault costs a frame's overlay, never the
                        # flight; the recorder keeps the last HUD it was given.
                        if not hud_failed:
                            print(f"[hud]    HUD update failed ({type(exc).__name__}: "
                                  f"{exc}); recording goes on")
                        hud_failed += 1
                    hud_ms.append((time.perf_counter() - _th) * 1000.0)
                landed = s.armed is False or s.landed_state == LANDED_STATE_ON_GROUND
                if takeover is not None and (landed or now - takeover["t"] > TAKEOVER_GRACE_S):
                    end_reason = f"autopilot took over ({takeover['mode']})"
                    break
                if fsm is not None and (fsm.terminal or (
                        failsafe_t is not None and now - failsafe_t > FAILSAFE_GRACE_S)):
                    end_reason = f"escalation FSM ended the mission ({fsm.state.value})"
                    break
                if vla_end is not None:
                    # The VLA mission's own end (vla_mission.END_REASONS): the
                    # model's LAND is landed by the mission-end LAND below,
                    # never by a mode request from the pilot.
                    end_reason = vla_end
                    print(f"[vla]    mission ends: {vla_end} - {VM.END_REASONS[vla_end]}")
                    break
                # PACE TO AN ABSOLUTE DEADLINE (next_tick_deadline), as
                # follow_vlm does: a late wake is repaid by the next sleep.
                _now = clock()
                _work = _now - t_tick
                work_ms.append(_work * 1000.0)
                # After the tick's work, in its wait (GcPacer.after_publish).
                gc_pacer.after_publish(_now - t0)
                tick_deadline = next_tick_deadline(tick_deadline, t_tick, _now)
                if vla is not None and getattr(args, "vla_feed", "tick") == "stream":
                    # THE FEED BETWEEN TICKS (pace_feeding): the same deadline.
                    pace_feeding(tick_deadline, vla.pump, clock, sleep)
                else:
                    sleep(max(0.0, tick_deadline - _now))
        except KeyboardInterrupt:
            end_reason = "interrupted"
            interrupted = True                 # the Dashboard row ends "stopped"
            print("[node]   interrupted")
        except Exception as exc:
            # The episode still ends on the record, with the real reason - not
            # the "max_s reached" it was initialised with - and the error
            # propagates (no metrics or KPI table is written for it).
            end_reason = f"crashed: {type(exc).__name__}: {exc}"
            raise
        finally:
            stage = "the mission's end"
            # The collector's own schedule back, before the landing and the report.
            gc_pacer.stop()
            live.beat(progress=1.0, phase="landing", detail="episode over; landing")  # memory only
            if vla is not None:
                # The decision rate's span ends with the mission, not with the
                # LAND below (up to 5 s on a dead pymavlink link).
                vla.freeze()
            # The throttle goes back to the radio first: an override must not
            # outlive the hold it was for.
            try:
                throttle.release(round(clock() - t0, 3), f"episode end ({end_reason})")
            except Exception as exc:                              # noqa: BLE001
                print(f"[hold]   override release failed: {exc}")
            # THE LAST LOOK BEFORE THE LAND (review 3 follow-up, 2026-10-22):
            # the mode is judged once more, by the loop's takeover test against
            # the last mode judged. On the no-mission path no tick ran, and the
            # gate pumps the link once after the hover's last tick (for its
            # truth verdict): a pilot's switch reported in that pump was judged
            # by nobody, and LAND was asked over the pilot's LOITER with
            # autopilot_took_over None (tests/test_pas_ardupilot.py). A mission
            # that ended in the mode last judged is unchanged by this.
            if takeover is None:
                try:
                    s_end = link.snap
                    if autopilot_took_over(s_end.mode, prev_mode, asks):
                        t_end_ = round(clock() - t0, 3)
                        takeover = {"t": t_end_, "mode": s_end.mode, "previous": prev_mode,
                                    "fence_breach_count": s_end.fence_breach_count,
                                    "statustext": list(s_end.statustext[-3:]),
                                    "phase": ("before the mission" if no_mission is not None
                                              else "mission end")}
                        print(f"[link]   *** the autopilot took the aircraft: {prev_mode} -> "
                              f"{s_end.mode} ({takeover['phase']}); nothing is asked of it")
                        # Best effort: a record write never decides the LAND.
                        try:
                            rec.event("autopilot_took_over", t=t_end_, mode=s_end.mode,
                                      previous=prev_mode, phase=takeover["phase"])
                        except Exception as exc:                  # noqa: BLE001
                            print(f"[node]   *** takeover event not written: {exc}")
                except Exception as exc:                          # noqa: BLE001
                    print(f"[link]   mode not judged at the end: {type(exc).__name__}: {exc}")
            # A mission end lands. A fail-safe the FSM asked for, or a mode the
            # autopilot chose itself, is left to the autopilot to finish.
            autopilot_owns = takeover is not None or (
                fsm is not None and (fsm.terminal or fsm.state in FAILSAFE_STATES))
            # What the LAND request returned (None: not asked): over a dead
            # link PymavlinkLink.land() tries for 5 s and returns False, and
            # that is on record, not dropped.
            land_accepted = None
            if not autopilot_owns:
                try:
                    asks.ask("LAND")
                    land_accepted = bool(link.land())
                except Exception as exc:                          # noqa: BLE001
                    land_accepted = False
                    print(f"[link]   LAND failed: {exc}")
                if land_accepted is False:
                    print("[link]   *** LAND was not confirmed by the autopilot")
            # The LAND decision is made: the net in the outer finally stands down.
            land_decided = True
            if vla is not None:
                # The VLA stops here, after the LAND was asked: its last
                # records close (vla_steps.jsonl, one line per request).
                try:
                    vla.close()
                except Exception as exc:                          # noqa: BLE001
                    print(f"[vla]    close failed: {type(exc).__name__}: {exc}")
            # The REST listener closes first: a later event would land after
            # this episode's records (503 from here; rail_hotapply.stop).
            rail_hotapply.stop(hot, hot_gate, f"the episode ended ({end_reason})")
            # EpisodeRecord.end(): the one mission_end write_replay requires
            # before it packs an episode.
            rec.end(end_reason, t=round(clock() - t0, 3))
            if hasattr(grounder, "stop"):
                grounder.stop()
            # The video ends with the episode: past here the HUD would freeze
            # on the last tick while the KPI table and the replay are built.
            if recorder is not None:
                try:
                    rec_summary = stop_recorder(recorder, view_dir)
                except Exception as exc:                          # noqa: BLE001
                    print(f"[view]   recorder stop failed: {type(exc).__name__}: {exc}")

        live.beat(detail="writing the run's records")                       # memory only
        t_end = clock()
        mission_s = max(1e-6, t_end - t0)
        # A RUN THAT FLEW NO MISSION MEASURED NO RATE AND NO DWELL (review
        # 2026-10-22). With no tick flown, mission_s above is the 1e-6 floor or
        # the mission-end LAND alone: a no-mission run wrote det_hz 1000000.0
        # (det_n / 1e-6), and nfz_s, alt_violation_s, interventions,
        # escape_ticks and dwell_gap_s 0 - a 1 MHz detector and clean dwell for
        # anything that reads metrics.json directly (the tracker, the decks).
        # Those are None now, with why (`not_measured`); the rates also on a
        # mission shorter than one tick.
        flew = no_mission is None
        rated = flew and mission_s >= TICK
        not_measured = None if flew else f"no mission flown: {no_mission}"

        def flown_only(v):
            return v if flew else None
        g1 = grounder.latest()
        det_n = (g1.get("n_seen") or 0) + (g1.get("n_miss") or 0) - det_n0
        tb1 = (link.snap.time_boot_ms, getattr(link.snap, "t_time_boot", None))
        rt_factor = (round((tb1[0] - tb0[0]) / 1000.0 / (tb1[1] - tb0[1]), 3)
                     if None not in (tb0[0], tb0[1], tb1[0], tb1[1])
                     and tb1[1] - tb0[1] >= 1.0 else None)
        fence1 = link.snap.fence_breach_count
        fence_breaches = (None if fence0 is None or fence1 is None else fence1 - fence0)

        # ---- dwell, from the flown trajectory -------------------------------------
        # Zone dwell from the Shield's own rule checker, the count every rail
        # shares (guardrail.replay.zone_dwell): polygon and circle fences and
        # every dynamic_nfz that does not move; a moving one is named
        # (nfz_moving_not_counted), not scored 0. Until 2026-10-11 this walked
        # policy.by_type(PolygonFence), which a dynamic_nfz is not. Each point's
        # rule-clock reading gates each zone: time inside a zone its window or
        # a switch had off is nfz_off_by_clock_s, not dwell. Since 2026-10-19
        # the seconds are the points' own durations from their timestamps
        # (point_durations), not the count times TICK: the count stays beside
        # them (nfz_ticks, alt_violation_ticks). A point followed by ticks with
        # no live state holds only until the first of them (gap_from); the
        # unknown time after it is dwell_gap_s, charged to no zone or band.
        traj_t = [p["t"] for p in traj]
        traj_s = point_durations(traj_t, TICK,
                                 ends=[gap_from.get(i) for i in range(len(traj))])
        dwell_gap_s = sum(max(0.0, b - a - w) for a, b, w in zip(traj_t, traj_t[1:], traj_s))
        # A zone spawned over REST counts from its arrival (its mission time,
        # on the trajectory's 0.01 s grid: FlightHotApply.live_from).
        dwell = zone_dwell(shield, traj, TICK, clock_key="clock", point_s=traj_s,
                           live_from=rail_hotapply.live_from(hot))
        nfz_ticks = dwell["nfz_ticks"]
        # The subject's true range (evaluation only): the stand-off dwell and
        # the follow proxy from the simulator's pose, None when never read.
        # Rows and trajectory points are written on the same ticks, so the
        # trajectory's weights are the rows' too.
        tsum = truth_summary(rows, truth, ring_m, TICK, hfov_deg=hfov, width_m=width_m,
                             point_s=traj_s)
        scored = tsum["scored"]
        if tsum.get("as_read") is not None:
            # The truth's ranges are not ranges to the subject (its frame,
            # or the EKF): no offline scorer may read them from the rows
            # either (they are written below).
            tsum["rows_quarantined"] = quarantine_truth_rows(rows, (
                frame_disagreement(tsum["frame_check"])
                + "; the pose as read is in subject_truth_as_read"))
        if dwell["nfz_moving_not_counted"]:
            print(f"[nfz]    moving zone(s) {dwell['nfz_moving_not_counted']} are NOT "
                  f"in nfz_s: the end-of-flight count has no per-tick position")
        env = policy.by_type(AltitudeEnvelope)
        alt_out = [(p, w) for p, w in zip(traj, traj_s) for e in env
                   if not (e.alt_min_m <= p["up"] <= e.alt_max_m)]
        alt_ticks, alt_s = len(alt_out), sum(w for _, w in alt_out)
        loop_hz = round(len(rows) / mission_s, 3) if rated else None
        # THE GATE JUDGES THE PACING, NOT THE MISSION CLOCK (review
        # 2026-10-19). loop_hz is rows / mission_s, kept as it always was;
        # but mission_s runs past the loop to the mission-end LAND (up to 5 s
        # over a dead pymavlink link) and the recorder stop, and ticks that
        # wrote no row (no state, a stale link) are in its time and not in
        # its count - so a short or link-lost run read as a slow loop. The
        # gate reads the loop's own tick starts instead, every tick counted,
        # rows or not (loop_hz_paced; loop_hz on a run of under two ticks).
        loop_hz_paced = (round((len(tick_starts) - 1) / (tick_starts[-1] - tick_starts[0]), 3)
                         if len(tick_starts) >= 2 and tick_starts[-1] > tick_starts[0]
                         else None)
        pace_hz = loop_hz_paced if loop_hz_paced is not None else loop_hz
        if pace_hz is not None and pace_hz < LOOP_HZ_GATE:
            print(f"[loop]   *** the loop ticked at {pace_hz} Hz, under the "
                  f"{LOOP_HZ_GATE:g} Hz gate (tick work p95 "
                  f"{spread([round(v, 3) for v in work_ms]).get('p95')} ms; "
                  f"{n_stale} stale-link and {n_no_state} no-state tick(s) of "
                  f"{len(tick_starts)})")

        # A VLA run's VLA is a separate process, the server: its host is
        # judged beside the Orin's (vla_host; THE VLA FOLLOW).
        ev, link_ev = rail_topology_evidence(
            args, sim_cfg, link, **({"vla_host": vla_rec.get("vla_host")} if vla_on else {}))
        cls = topo.classify(ev)
        simulated = args.camera == "projectairsim"
        qualified = cls["rail_qualified_by_g1"] is True
        rt_ok = rt_factor_ok(rt_factor)
        withheld = []
        if cls["grant"] in (TOPOLOGY_DEV, TOPOLOGY_HIL):
            if not qualified:
                withheld.append("gate G1 has not passed for this rail "
                                f"({ev['g1'].get('why') or ev['g1'].get('error') or ev['g1'].get('status') or 'no verdict'})")
            if not rt_ok:
                withheld.append(f"ArduPilot ran at {rt_factor}x real time (measured "
                                f"from its own clock); the band is {RT_FACTOR_BAND}")
        offered = cls["grant"] if (cls["grant"] and not withheld) else None
        autopilot_events = [{**e, "t": round(e["t"] - t0, 3)} for e in link.events]

        metrics = {
            "tag": tag, "rail": "projectairsim-ardupilot",
            "episode_id": rec.episode_id,
            "topology": cls["stamp"], "grant_topology": cls["grant"],
            "grant_topology_reasons": cls["reasons"],
            "grant_topology_offered": offered,
            "grant_topology_withheld": withheld,
            "rail_qualified_by_g1": cls["rail_qualified_by_g1"],
            "topology_evidence": ev, "link_evidence": link_ev,
            # The key guardrail.manifest.is_kpi_grade re-checks a hil label on.
            "hil_evidence": ev["topology_evidence"],
            "scene": None if camera is None else getattr(camera, "scene_record", None),
            # The perception stressor (--stress; None: none): what was
            # rendered and set, and its non-KPI label (perception_stress).
            "perception_stress": perception_stress_record(args, sim_cfg, camera),
            "shield": args.shield, "ticks": len(rows), "ticks_without_state": n_no_state,
            "ticks_flown": sum(1 for r in rows if r.get("flown")),
            "ticks_not_flown": sum(1 for r in rows if r.get("flown") is False),
            "shield_lookahead": {"horizon_s": LOOKAHEAD_S, "dt_s": LOOKAHEAD_DT_S,
                                 "poses": int(round(LOOKAHEAD_S / LOOKAHEAD_DT_S))},
            "mission_s": round(mission_s, 2), "end_reason": end_reason,
            # Why the rates, dwell and Shield counters below are None (None:
            # a mission was flown and they are measured).
            "not_measured": not_measured,
            "loop_hz": loop_hz,
            # The project's loop gate, and how the loop was paced to it
            # (next_tick_deadline) on which OS timer (None: not measured,
            # run() was called without main()). The gate is judged on the
            # loop's own tick starts (loop_hz_paced, every tick counted), not
            # on loop_hz, whose clock includes the mission-end LAND.
            "loop_hz_paced": loop_hz_paced, "loop_ticks": len(tick_starts),
            "loop_hz_gate": LOOP_HZ_GATE,
            "loop_hz_below_gate": None if pace_hz is None else pace_hz < LOOP_HZ_GATE,
            "loop_hz_gate_basis": ("loop_hz_paced: (ticks - 1) / the span of their starts"
                                   if loop_hz_paced is not None else
                                   "loop_hz: fewer than two ticks to pace" if rated else
                                   f"not measured: {not_measured or 'under one tick flown'}"),
            "tick_pacing": "absolute-deadline",
            "timer_resolution_ms": TIMER_RESOLUTION_MS,
            # Ticks whose position or heartbeat the link had stopped refreshing
            # (link_staleness): nothing was commanded on them and no row was
            # written. Each span with its reason; an episode the staleness
            # ended says "link lost" in end_reason.
            "ticks_state_stale": n_stale,
            "link_freshness": {
                "measured": freshness_measured,
                "why": None if freshness_measured else
                       f"the {link.name} link has no link_age(): staleness not measured",
                "pos_stale_s": POS_STALE_S, "heartbeat_stale_s": HEARTBEAT_STALE_S,
                "lost_after_s": LINK_LOST_S, "ticks_stale": n_stale,
                "spans": stale_spans,
                "fcu_connected_at_end": link_ev.get("fcu_connected_at_end")},
            # What the mission-end LAND request returned (None: not asked, the
            # autopilot owned the aircraft).
            "mission_end_land_accepted": land_accepted,
            # Each tick's work before its sleep (the 100 ms budget's used part).
            "tick_work_ms": spread([round(v, 3) for v in work_ms]),
            # The collector over the mission (guardrail.gc_pacing; --gc-pacing).
            "gc": gc_pacer.summary(),
            # The demo video (--save-view): the recorder's own summary (its
            # measured rate is in view/recorder.json, which
            # tools/make_demo_video.py reads), what the HUD cost the loop per
            # tick, and which banners the frames carried.
            "recording": ({"enabled": False, "why": rec_why} if view_dir is None else
                          {"enabled": True, "view_dir": str(view_dir),
                           "record_hz": args.record_hz,
                           "record_height": args.record_height,
                           "chase_subscribed": (None if camera is None else
                                                getattr(camera, "have_chase", None)),
                           "recorder": rec_summary,
                           "hud_ms": spread([round(v, 3) for v in hud_ms]),
                           "hud_failed_ticks": hud_failed,
                           "policy_hud": None if rhud is None else rhud.summary()}),
            # The observation rate against the grant's 10 Hz, beside its null: a
            # detector that never ran scores 0.0 here (None: no mission flown).
            # None with no detector run (a VLA GOTO): nothing was measured.
            "det_hz": round(det_n / mission_s, 3) if rated and not vla_goto else None,
            "det_hz_grant_target": GRANT_OBS_HZ,
            "det_hz_null": 0.0,
            # The servo pilot's track / search ticks (None for the VLA's, whose
            # ticks are counted by author in pilot.summary).
            "track_ticks": getattr(pilot, "n_track", None),
            "search_ticks": getattr(pilot, "n_search", None),
            "subject_known_ticks": n_subject, "subject_source": subject_src,
            "subject_class": subject_class,
            # The width the range from box width assumed, the stand-off rules
            # that class binds (only "*" ones for an unclassified subject), and
            # the unclassified record (None: the phrase named a class).
            "subject_width_m": width_m, "subject_standoff_rules": standoff_ids,
            "subject_unclassified": unclassified,
            # Whose pose and depth placed each estimate (estimate_inputs:
            # "capture:<pose src>" and the Grounder's depth source, or the
            # "tick" pose when the box had no capture pose - with the capture's
            # range when it had one, else the "latest" depth frame), and how
            # old the box was when it was placed.
            "subject_pairing": {"pose": pairing_pose, "depth": pairing_depth,
                                "box_age_s": spread(box_ages)},
            # escape_ticks counts ticks that FLEW a violation of ANY priority (P1
            # speed caps included); the P0 count and the KPI rate are kpi.json's
            # p0_escapes / p0_violation_escape_rate. The 2026-10-13 close
            # Shield-off run read 749 here against 727 P0 escapes.
            "interventions": flown_only(n_touched), "brakes": flown_only(n_braked),
            "escape_ticks": flown_only(n_escaped),
            # Dwell in seconds from the points' own timestamps (point_durations),
            # the tick counts beside them. dwell_gap_s: time between points
            # with no live state (no state, a stale link), charged to no point;
            # dwell_gap_points: how many points such a gap followed. None, with
            # the counters above, when no mission was flown (not_measured).
            "nfz_s": flown_only(round(dwell["nfz_s"], 2)),
            "alt_violation_s": flown_only(round(alt_s, 2)),
            "nfz_ticks": flown_only(nfz_ticks), "alt_violation_ticks": flown_only(alt_ticks),
            "dwell_gap_s": flown_only(round(dwell_gap_s, 2)),
            "dwell_gap_points": flown_only(len(gap_from)),
            "dwell_basis": ("time-weighted: each trajectory point's time to the next "
                            f"(the last point {TICK:g} s), a point followed by ticks with "
                            "no live state only until the first of them (the rest is "
                            "dwell_gap_s); counts in nfz_ticks / alt_violation_ticks"),
            "nfz_by_zone_s": dwell["nfz_by_zone_s"],
            "nfz_moving_not_counted": dwell["nfz_moving_not_counted"],
            "nfz_off_by_clock_s": dwell["nfz_off_by_clock_s"],
            # Which clock the rules were evaluated on (scripted or wall).
            "rule_clock": clock_record,
            # The follow mission each generation's CSP was compiled for.
            "mission": mission_rec,
            # The tracking score and the stand-off dwell come from the
            # subject's simulator pose when it was read on enough ticks
            # (subject_truth, TRUTH_MIN_COVERAGE); with no pose, or too few,
            # there is none to report: None, never 0 - except a dwell that
            # WAS seen, kept as its lower bound (standoff_basis says which).
            # A VLA GOTO follows nothing: "not applicable" (None), so
            # guardrail.kpi decides its mission on reached_goal instead of
            # an unscored follow (review of 4a0da6d, P2-2).
            "frac_within_30m": None if vla_goto else tsum["frac_within_30m"],
            "standoff_s": tsum["standoff_s"],
            "standoff_basis": tsum["standoff_basis"],
            "standoff_min_range_m": tsum["min_range_m"],
            "follow_scored": None if vla_goto else scored,
            "follow_scored_why": ("not applicable: a VLA GOTO has no subject to follow; "
                                  "reached_goal scores it" if vla_goto else None),
            "subject_truth": tsum,
            "fsm": (dict(fsm.summary(), fault=fsm_fault,
                         theta_basis=theta_basis(horizon),
                         hold_mode={"fsm_mode": "LOITER",
                                    "autopilot_mode": autopilot_mode("LOITER", hold_mode)})
                    if fsm is not None else None),
            # The FSM's LOITER as flown, and the throttle override that makes
            # ArduCopter's LOITER hold altitude (--hold-mode LOITER on SITL).
            "hold": {"fsm_mode": "LOITER",
                     "autopilot_mode": autopilot_mode("LOITER", hold_mode),
                     "rc_override": throttle.summary(),
                     # The hold that stops clear (guardrail.response.hold_plan):
                     # each plan made at a Loiter entry or while the hold was
                     # deferred, the ticks the Loiter was flown as a GUIDED stop
                     # through the Shield, and the FSM "brake" ticks flown as the
                     # Shield's answer to a stop. forced_handovers: hand-overs
                     # to BRAKE the plan did not choose, by reason (link_stale,
                     # stream_gap, deferral_cap; review 2026-10-20).
                     "stop_clear": {"enabled": own_v and hold_ap_mode == "BRAKE",
                                    "plans": hold_plans,
                                    "guided_hold_ticks": n_hold_guided,
                                    "stop_advised_ticks": n_stop_advised,
                                    "forced_handovers": forced_handovers,
                                    "reclaims": n_hold_reclaims,
                                    "reclaim_max": vresp.HOLD_RECLAIM_MAX,
                                    "handover_mps": vresp.HOLD_HANDOVER_MPS,
                                    "defer_max_s": vresp.HOLD_DEFER_MAX_S,
                                    "stream_gap_s": vresp.HOLD_STREAM_GAP_S,
                                    "brake_envelope": vresp.ARDUCOPTER_BRAKE.to_dict()}},
            # Which forecast the Shield flew (docs/DESIGN-own-velocity-
            # forecast.md): "own_velocity" (the measured velocity converging
            # to the command through the measured response) on the ticks the
            # link had a fresh velocity, "commanded" (as before) otherwise.
            "shield_forecast": {
                "mode": "own_velocity" if own_v else "commanded",
                "flag": "--own-velocity-forecast " + ("on" if own_v else "off"),
                "response": vresp.ARDUCOPTER_GUIDED.to_dict(),
                "velocity_ticks": n_vel_used,
                "velocity_sources": vel_sources,
                "shield_velocity_ticks": shield.n_velocity_ticks,
                # Ticks the velocity was given but SET ASIDE because the
                # position already broke a hard rule (Shield._decision_state:
                # the decision is then the commanded forecast's).
                "velocity_set_aside_ticks": shield.n_velocity_set_aside,
                "velocity_nonfinite": shield.n_velocity_nonfinite,
                # Setpoints the Shield was told were sent (note_flown), so a
                # run whose pipe never filled - the forecast then coasting
                # the measured velocity through the reaction time - shows.
                "noted_setpoints": shield.n_noted_setpoints,
                # P0 escape ticks on the stored runs' basis (rows'
                # p0_escape_commanded_basis; Shield.recheck_commanded), to
                # compare with kpi.json p0_escapes (the rail's re-check,
                # with the velocity when the Shield had one).
                "p0_escape_ticks_commanded_basis": n_p0_cmd_basis},
            # The FSM's words (LOITER, RTL, LAND; LAND also at mission end) and
            # what ArduCopter was asked for (the FSM's LOITER as BRAKE).
            "mode_requests": list(asks.fsm),
            "autopilot_mode_requests": list(asks.autopilot),
            # What the autopilot did on its own, by its own report: mode and arm
            # changes, STATUSTEXT, fence breaches. Relative to mission start
            # (bring-up events are negative).
            "autopilot_events": autopilot_events,
            "autopilot_took_over": takeover,
            # FENCE_STATUS over the mission; None when it never arrived (the
            # MAVROS link has no such topic), never a silent 0.
            "fence_breach_count": fence_breaches,
            # The GeoFence as the autopilot held it (None: the link reads no
            # parameters), judged against the policy's ceiling.
            "autopilot_fence": ap_fence,
            # The policy's zones on that fence (FenceBackstop): when each upload
            # was made (pre_arm), what went up, what was held back and why.
            "fence_upload": None if backstop is None else backstop.summary()["last"],
            "fence_uploads": [] if backstop is None else backstop.uploads,
            "fences_still_deferred": None if backstop is None else backstop.deferred,
            "fence_upload_phase": pre_arm.get("phase"),
            "generations": rec.generations,
            # The REST hot-apply listener's events (rail_hotapply; None when
            # not served), as the ROS 2 node records them.
            "hot_apply": None if hot is None else hot.summary(),
            # Every contact the simulator reported, take-off and landing included
            # (g1_check.airborne_collisions separates those); None = not subscribed.
            "sim_contacts_all": (None if camera is None
                                 or not getattr(camera, "collisions_subscribed", False)
                                 else len(camera.collisions)),
            "ekf_vs_sim_horiz_m": spread(ekf_err),
            "ardupilot_rt_factor": rt_factor, "rt_factor_band": list(RT_FACTOR_BAND),
            "fcu_clock_source": getattr(link.snap, "fcu_clock_source", None),
            "sim_speedup_param": speedup,
            "frame_anchor": anchor.to_dict(),
            "obstacle_map": map_note,
            "sim_config": sim_cfg,
            "policy_source": policy_source,
            "seed": args.seed,
            "params": {k: getattr(args, k) for k in ("object", "cruise_alt", "yaw_gain",
                                                     "want_width", "speed_max", "alt_gain",
                                                     "climb_max", "det_thresh", "colour_min",
                                                     "seed", "link", "camera", "scene_mode",
                                                     "hold_mode", "subject_actor",
                                                     "clock_start", "clock_tz")},
            "detector": (None if vla_goto else
                         "google/owlvit-base-patch32 (demo/follow_vlm.DETECTOR_ID)"),
            # THE START GATE (StartGate, wait_for_subject; open item C1): what
            # it judged and refused, whether and when it was reached, the box
            # it committed to, and - read after the decision, evaluation only
            # - whether that was the subject (truth_at_decision) and where the
            # subject was while it waited (truth_trace). enabled false: the
            # mission started at take-off (--start-when-seen 0).
            # outcome: reached / timed_out / takeover / link_lost /
            # detector_dead / unsafe / crashed / interrupted; hover_s and
            # unsafe_ticks: its share of pre_mission_hover.
            "start_gate": gate_rec,
            # The detector started before SITL, and how long the mission then
            # still waited for its first inference (DETECTOR_READY_TIMEOUT_S):
            # on the ground in the pre-arm step, or airborne (`airborne`) on a
            # link with no pre-arm step; `outcome` as wait_for_detector says.
            "detector_ready": detector_ready,
            # THE HOVER BEFORE THE MISSION, WATCHED (HoverWatch; review
            # 2026-10-22): its seconds by phase, the ticks the Shield checked,
            # the unsafe ones and their rules (None: no tick was checked), what
            # ended it early; every tick in hover_log.jsonl. Not in any KPI.
            "pre_mission_hover": hover_rec,
            # How the detector chose its box (build_grounder): the TargetLock,
            # identity, the negative phrases and the boxes they took away;
            # the lock's own counters.
            "detector_selection": dict(
                grounder_rec,
                negative_suppressed=getattr(grounder, "n_neg_suppressed", None),
                lock_stats=(grounder.lock.stats() if getattr(grounder, "lock", None)
                            is not None and hasattr(grounder.lock, "stats") else None),
                identity_counts=(dict(grounder.id_counts) if grounder_rec.get("identity")
                                 and hasattr(grounder, "id_counts") else None)),
            # The simulator's evaluation RPCs, off the loop (OffLoopRpc): each
            # call's own duration, and those answered too late to use.
            "sim_rpc": rpc.summary(),
        }
        metrics["params"].update({k: getattr(args, k, None) for k in (
            "start_when_seen", "start_max_range_m", "start_timeout_s",
            "start_timeout_action", "negative_query", "identity", "lock_gate",
            "no_lock_target", "pilot", "mission", "front_capture_interval_s", "stress",
            "target_estimator")})
        if not vla_on and getattr(pilot, "track", None) is not None:
            # THE ESTIMATOR FOLLOW's records: its law (EstimatorFollowPilot.
            # summary) and its SubjectTrack's, as the VLA follow's
            # subject_track. A fault here is recorded, never raised.
            try:
                metrics["estimator_follow"] = pilot.summary()
                st_rec = subject_track_record(pilot.track, rows)
            except Exception as exc:                              # noqa: BLE001
                st_rec = {"on": True, "record_error": f"{type(exc).__name__}: {exc}"}
            st_rec["errors"] = pilot.track_errors
            st_rec["first_error"] = pilot.track_error
            metrics["subject_track"] = st_rec
        if vla_on:
            metrics["params"].update({k: getattr(args, k, None) for k in VLA_PARAMS})
            # THE OBSERVATION RATE (the grant's 10 Hz observation, X-05 /
            # ARCH-25): distinct front frames that reached the VLA, never the
            # decision or request rate (vla_observation's docstring).
            try:
                from guardrail.aerialvla_codec import FRONT_SOURCES
                f1 = obs_frames_received(obs)
                metrics["vla_observation"] = vla_observation(
                    list(getattr(vla_frames, "log", None) or []), rows, t0=t0,
                    span_s=((tick_starts[-1] - tick_starts[0] + TICK) if tick_starts
                            else None),
                    front_source=args.vla_front_source,
                    camera=FRONT_SOURCES[args.vla_front_source]["camera"],
                    camera_frames=(None if f1 is None or obs_frames0 is None
                                   else f1 - obs_frames0),
                    capture_interval_s=(sim_cfg or {}).get("front_capture_interval_s"))
            except Exception as exc:                          # noqa: BLE001
                metrics["vla_observation"] = {"error": f"{type(exc).__name__}: {exc}"}
            # CAMERA TO SETPOINT (WP3-13; "did you measure the latency from
            # camera input to action output?"): the rows' cam_age_ms,
            # cam_to_setpoint_ms and tick_ms, summarised; tools/kpi_report.py
            # pools the rows themselves.
            try:
                fs_ = vla_frames.stats() if callable(getattr(vla_frames, "stats", None)) else {}
                metrics["camera_to_setpoint"] = camera_to_setpoint_summary(
                    rows, t_capture_basis=(fs_ or {}).get("t_capture_basis"))
            except Exception as exc:                          # noqa: BLE001
                metrics["camera_to_setpoint"] = {"error": f"{type(exc).__name__}: {exc}"}
            if args.mission == "follow":
                # This rail starts the mission clock at the start gate's
                # acquisition, so its whole-mission follow proxy is the share
                # vla_fly reports from the first acquisition (fair-follow
                # scoring); None when the gate did not open on the subject.
                metrics["frac_within_30m_after_acquire"] = (
                    metrics.get("frac_within_30m") if gate_rec.get("reached") else None)
                metrics["frac_within_30m_basis"] = (
                    "the mission clock starts at the start gate's acquisition (StartGate), "
                    "so the mission's share is the share after acquisition"
                    if gate_rec.get("reached") else
                    "the start gate did not confirm the subject: the share runs from "
                    "take-off, not from an acquisition")
            # THE VLA PILOT's records, as on the city rail (demo/vla_mission):
            # metrics.pilot (pilot_entry: kind vla only with the server's real
            # identity, the backend's stats under "vla" - what tools/todo.py
            # and the Dashboard read - and the summary), metrics.vla, the
            # mission's score and LAND, vla_captures.jsonl. A fault here is
            # recorded, never raised: the aircraft has landed by now.
            try:
                metrics.update(_vla_records(
                    args, out, VM, vla, vla_backend, vla_srv, vla_identity, vla_rec,
                    vla_warmup, vla_frames, vla_saver, rows, end_reason, gate_rec=gate_rec,
                    detector_id=None if vla_goto else (fv or _fv()).DETECTOR_ID,
                    # An approach is scored on the subject's true range; a
                    # follow on the follow proxy above, never on a radius.
                    approach=(approach_scores(rows, args.approach_radius_m, scored=scored)
                              if args.mission == "approach" else None)))
                vla_captures_written = True
            except Exception as exc:                              # noqa: BLE001
                metrics["vla_records_error"] = f"{type(exc).__name__}: {exc}"
                print(f"[vla]    *** records incomplete: {metrics['vla_records_error']}")
        # The null beside the follow proxy (guardrail.kpi.null_hover_mission):
        # would an aircraft that never left its start pass the same test? It
        # reads the truth from the rows' tgt_x / tgt_y.
        metrics["null_hover_mission"] = null_hover_mission(rows, metrics)
        (out / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                              encoding="utf-8")
        (out / "trajectory.json").write_text(json.dumps(traj), encoding="utf-8")
        (out / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")

        # The grant label is OFFERED to build_manifest only when the rail is
        # qualified (G1 passed, real time measured); the manifest's guard then
        # judges it on the evidence and the scene. On refusal, or when the
        # label was withheld, the rail's own label is stamped and the reason
        # recorded. Since 2026-10-12 manifest.py accepts dev / hil beside a
        # scene whose robot config names ardupilot-api, on the live MAVROS
        # evidence (an fcu_url, no pymavlink link); the guard stays the only
        # judge of the grant label.
        common = dict(policy_hash=policy.policy_hash, model_id=NO_VLA, seed=args.seed,
                      scene_path=scene_path, sim_speedup=None if scene_path else speedup)
        if vla_on:
            # The model that answered (vla_manifest_model).
            common.update(vla_manifest_model(VM, vla_identity, vla_rec))
        refusal = None
        if offered:
            try:
                manifest = build_manifest(**common, topology=offered,
                                          hil_evidence=ev["topology_evidence"])
            except ValueError as exc:
                refusal = str(exc)
        if offered is None or refusal is not None:
            manifest = build_manifest(**common, topology=cls["stamp"])
        metrics["manifest_refusal"] = refusal
        metrics["topology"] = manifest["topology"]
        (out / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
        (out / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        write_stack_record(out, manifest, ev, scene_path)
        kpi = compute_from_dir(out, policy)
        graded, why = is_kpi_grade(manifest, metrics)
        # What is_kpi_grade cannot see from the manifest: the scene DECLARES
        # sim_speedup 1.0, and only this run measured whether ArduPilot kept it;
        # and a simulated rail must be qualified by G1.
        if simulated and not rt_ok:
            why.append(f"ArduPilot's measured real-time factor is {rt_factor} "
                       f"(band {RT_FACTOR_BAND}); the scene's declared 1.0 is not "
                       f"what flew")
        if simulated and not qualified:
            why.append("the Project AirSim + ArduPilot rail is not qualified by gate G1")
        if fsm_fault is not None:
            why.append(f"the escalation FSM refused an input mid-flight: {fsm_fault}")
        if end_reason.startswith("link lost"):
            why.append(f"the autopilot link was lost mid-flight ({end_reason}); the "
                       f"episode ended early")
        elif n_stale:
            why.append(f"the autopilot link's state was not live on {n_stale} tick(s) "
                       f"({len(stale_spans)} span(s)); nothing was commanded on them and "
                       f"the record has a gap")
        if not freshness_measured:
            why.append(f"the {link.name} link cannot say how old its state is "
                       f"(no link_age()): a frozen position would pass as live")
        # An UNCLASSIFIED subject (review 2026-10-19): only the catch-all
        # stand-offs bind it, and a tick ranged by box width assumed a car's
        # width - about 8x too far for a 0.5 m person, so the ring fires late.
        # Width-ranged ticks were 43/129, 78/149 and 89/156 of the subject
        # ticks on the recorded pas runs: not a corner case. The run says so
        # where the grade is decided, not only in a console warning.
        if unclassified is not None:
            why.append(f"the subject phrase {args.object!r} names no class: "
                       + (f"only the catch-all stand-off(s) {standoff_ids} bind it"
                          if standoff_ids else "no stand-off rule binds it")
                       + f", and a range from box width assumed {width_m:g} m "
                         f"(a car's width)")
        if policy_source.get("dev_yaml"):
            why.append(f"the policy was flown from YAML under {DEV_YAML_FLAG}: "
                       f"unsigned, not its issued bundle")
        if getattr(args, "stress", None) is not None:
            # A perception-stress run is reported per scenario, never as a
            # KPI (perception_stress.NON_KPI_REASON; WP4-16).
            why.append(pstress.non_kpi_reason(args.stress))
        if no_mission is not None:
            why.append(f"no mission was flown: {no_mission}")
        elif gate_rec.get("enabled") and not gate_rec.get("reached"):
            why.append(f"the start gate confirmed no subject in "
                       f"{gate_rec.get('wait_s')} s; the mission was flown anyway ("
                       + ("the hover broke a rule, so it started for the Shield and the "
                          "escalation FSM to act" if gate_rec.get("outcome") == "unsafe"
                          else "--start-timeout-action fly")
                       + "), so it followed no confirmed subject")
        # The hover before the mission (HoverWatch, review 2026-10-22): the
        # mission's KPIs start at its clock and cannot show a rule the hover
        # broke, nor a hover nothing checked; the grade says so.
        if hover_rec["unsafe_ticks"]:
            fu = hover_rec["first_unsafe"]
            why.append(f"the hover before the mission broke {sorted(hover_rec['unsafe_rules'])} "
                       f"on {hover_rec['unsafe_ticks']} of {hover_rec['ticks_checked']} "
                       f"checked tick(s) (first {fu['t_hover']} s in, {fu['phase']}); the "
                       f"mission's KPIs start at its clock and do not count it")
        elif hover_rec["ticks"] and not hover_rec["ticks_checked"]:
            why.append(f"the aircraft hovered {hover_rec['s']} s before the mission with no "
                       f"live state on any tick, so no rule was checked")
        # The hover's records are best effort (review 3): one that was not
        # written is a gap in the evidence, said so, never silence.
        if hover_rec.get("log_error") or hover_rec.get("io_errors"):
            why.append("the hover before the mission is not fully on record: "
                       + "; ".join(
                           ([f"hover_log.jsonl was not written ({hover_rec['log_error']})"]
                            if hover_rec.get("log_error") else [])
                           + [f"event not written ({e})"
                              for e in hover_rec.get("io_errors") or []]))
        if detector_ready.get("ready") is False:
            why.append(f"the detector produced no inference in {detector_ready.get('waited_s')} s "
                       f"before the mission ({detector_ready.get('outcome')}: "
                       f"{detector_ready.get('why')})")
        # A VLA GOTO follows nothing: its mission test is reached_goal
        # (vla_mission.mission_scores), so no follow score is owed.
        if not scored and not vla_goto and (tsum.get("frame_check") or {}).get("agrees") is False:
            why.append("the follow is not scored: "
                       + frame_disagreement(tsum["frame_check"]))
        elif not scored and not vla_goto:
            # A follow's mission test is the follow proxy and the stand-off
            # dwell; read as None, guardrail.kpi passes the mission on
            # silence. Not a measured success.
            why.append(f"the follow is not scored: the subject's pose was read on "
                       f"{tsum['ticks_read']}/{len(rows)} ticks (needs "
                       f"{TRUTH_MIN_COVERAGE:.0%}; {truth.why_off or truth.last_why or 'never read'})")
        if vla_on:
            if not vla_rec.get("is_vla"):
                why.append("a scripted stand-in answered, not AerialVLA "
                           f"({vla_rec.get('why_not')}): this run tests the flight path, it "
                           f"is not a VLA flight")
            if end_reason in VM.VLA_FAILURE_ENDS:
                why.append(f"the VLA mission ended on a VLA failure ({end_reason}: "
                           f"{VM.END_REASONS[end_reason]})")
            if metrics.get("vla_records_error"):
                why.append(f"the VLA's records are incomplete ({metrics['vla_records_error']})")
        graded = graded and not why
        kpi.update(kpi_grade=graded, kpi_grade_reasons=why, manifest=manifest,
                   policy_source=policy_source, episode_id=rec.episode_id)
        kpi_path = out / "kpi.json"
        kpi_path.write_text(json.dumps(kpi, indent=2), encoding="utf-8")
        rb_path = out / f"{tag}.replay.tar.gz"
        try:
            rb = write_replay(out, policy, rb_path,
                              changelog=f"pas-ardupilot {tag}", policy_source=policy_source)
            ok, notes = verify_replay(rb)
            print(f"[replay] {rb.name}: {'re-derives its KPIs' if ok else 'FAILED'}")
            for n_ in notes:
                print(f"[replay]   - {n_}")
            # KPI evidence needs a bundle that verifies with a TRUSTED
            # signature (guardrail.replay.kpi_evidence_grade); a graded run
            # with a keyless or failing bundle is demoted, and kpi.json and
            # the bundle are rewritten to say so - as on both SITL rails.
            graded2, why2 = kpi_evidence_grade(graded, why, rb)
            if graded2 != graded:
                graded, why = graded2, why2
                kpi.update(kpi_grade=graded, kpi_grade_reasons=why)
                kpi_path.write_text(json.dumps(kpi, indent=2), encoding="utf-8")
                write_replay(out, policy, rb_path, changelog=f"pas-ardupilot {tag}",
                             policy_source=policy_source)
        except Exception as exc:                                  # noqa: BLE001
            print(f"[replay] not written: {type(exc).__name__}: {exc}")
            if graded:
                graded, why = False, why + [f"no replay bundle: {exc}"]
                kpi.update(kpi_grade=graded, kpi_grade_reasons=why)
                kpi_path.write_text(json.dumps(kpi, indent=2), encoding="utf-8")
        print(f"[report] ticks {len(rows)} at {metrics['loop_hz']} Hz | detector "
              f"{metrics['det_hz']} Hz (grant {GRANT_OBS_HZ}, null 0.0) | interventions "
              f"{n_touched} | flown-violation ticks {n_escaped} (any priority) | P0 escape "
              f"ticks {kpi.get('p0_escapes')} | P0 escape rate "
              f"{kpi['p0_violation_escape_rate']} | ended: {end_reason}")
        print(f"[report] ArduPilot real-time factor {rt_factor} | fence breaches "
              f"{fence_breaches} | mode requests {asks.fsm} (sent as "
              f"{asks.autopilot}) | autopilot took "
              f"over: {takeover['mode'] if takeover else 'no'}")
        print(f"[report] link state not live on {n_stale} tick(s) in "
              f"{len(stale_spans)} span(s)"
              + ("" if freshness_measured else " (NOT measured: the link has no link_age())")
              + f" | loop ticked at {pace_hz} Hz against the {LOOP_HZ_GATE:g} Hz gate "
              f"(absolute-deadline pacing, timer {TIMER_RESOLUTION_MS} ms; rows / "
              f"mission time {loop_hz} Hz)"
              + ("" if unclassified is None else
                 f" | subject UNCLASSIFIED: only {standoff_ids or 'no'} stand-off(s) bind"))
        print(f"[report] subject truth: {tsum['ticks_read']}/{len(rows)} ticks read | "
              f"nearest {tsum['min_range_m']} m | in the {ring_m} m ring "
              f"{tsum['frac_in_ring']} of read ticks | estimate off by median "
              f"{tsum['estimate_gap_m']['median']} m"
              + ("" if scored else
                 " - NOT scored: the truth's ranges are in doubt "
                 f"(cause {tsum['frame_check']['cause']}; as read: nearest "
                 f"{tsum['as_read']['min_range_m']} m)" if tsum.get("as_read") else
                 f" - NOT scored, coverage {tsum['coverage']} "
                 f"({truth.why_off or truth.last_why or 'never read'})")
              + f" | hover null {metrics['null_hover_mission'].get('success')}")
        fc = tsum.get("frame_check") or {}
        print(f"[report] truth frame check: "
              + (f"the aircraft through get_object_poses([{fc.get('aircraft_actor')!r}]) "
                 f"median {fc['err_m']['median']} m over {fc['err_m']['n']} checks, "
                 f"heading median {fc['yaw_err_deg']['median']} deg "
                 f"(limits {TRUTH_FRAME_MAX_M:g} m, {TRUTH_FRAME_MAX_YAW_DEG:g} deg) - "
                 if fc.get("checked") else
                 f"PARTIAL ({fc.get('why')}) - " if fc.get("partial") else
                 f"not checked ({fc.get('why')}) - ")
              + {True: "AGREES", None: "not proven",
                 False: "DISAGREES: nothing truth-based scores"}[fc.get("agrees")]
              + (f"; check RPC p95 {fc['check_ms']['p95']} ms" if fc.get("check_ms") else ""))
        if (tsum.get("view") or {}).get("verdict"):
            print(f"[report] subject in view: {tsum['view']['verdict']}")
        if backstop is not None:
            last = backstop.summary()["last"] or {}
            print(f"[report] GeoFence zones uploaded {backstop.uploaded} "
                  f"({pre_arm.get('phase')}); still held back {backstop.deferred}"
                  + (f"; last: {last.get('why')}" if last.get("why") else ""))
        if hot is not None:
            print(f"[report] {rail_hotapply.report_line(metrics['hot_apply'])}")
        print(f"[report] topology stamped {manifest['topology']!r}; grant label "
              f"{cls['grant']!r}: " + "; ".join(cls["reasons"] + withheld))
        if refusal:
            print(f"[report] build_manifest refused {offered!r}: {refusal}")
        print(f"[report] KPI-grade: {'YES' if graded else 'no'}"
              + ("" if graded else " (" + "; ".join(why) + ")"))
        if view_dir is not None:
            r_ = rec_summary or {}
            print(f"[report] recording: {r_.get('frames')} frames at "
                  f"{r_.get('achieved_hz')} Hz -> {view_dir}; HUD on the loop p95 "
                  f"{metrics['recording']['hud_ms']['p95']} ms; loop {metrics['loop_hz']} Hz")
            if Path(args.out_root).resolve() == (ROOT / "demo" / "out").resolve():
                print(f"[report] video: python tools/make_demo_video.py --tag {tag} "
                      f"--height 720")
            else:
                print(f"[report] video: tools/make_demo_video.py reads demo/out/<tag>/view "
                      f"only; these frames are under {view_dir}")
        if vla_on:
            s_ = (metrics.get("pilot") or {}).get("summary") or {}
            lat_ = s_.get("latency_client_s") or {}
            print(f"[report] VLA {args.mission} ({args.cadence}): end {end_reason}; decisions "
                  f"{s_.get('decisions_accepted')} at {s_.get('decision_hz')} Hz; latency p50 "
                  f"{lat_.get('median')} s p95 {lat_.get('p95')} s (capture -> answer); "
                  f"parse_fail {s_.get('parse_fail')}, server_error {s_.get('server_error')}, "
                  f"link_failure {s_.get('link_failure')}, LAND {s_.get('land')}; ticks by "
                  f"author {s_.get('ticks_by_author')}; VLA-authored share "
                  f"{s_.get('vla_authored_tick_share')}; held by the rail "
                  f"{(metrics.get('vla') or {}).get('held_ticks')} tick(s); manifest "
                  f"{manifest.get('vla_model_hash')}"
                  + ("" if vla_rec.get("is_vla") else " - *** a STAND-IN answered: not a VLA "
                                                      "flight ***"))
            o_ = metrics.get("vla_observation") or {}
            gs_ = o_.get("hint_gated") or {}
            print(f"[report] VLA observation ({o_.get('camera')}): {o_.get('distinct_frames')} "
                  f"distinct frames sent = {o_.get('hz_per_wall_s')} Hz over the mission, "
                  f"{o_.get('hz_while_asking')} Hz while the pilot was asked, "
                  f"{o_.get('hz_per_sim_s')} per sim s (grant {GRANT_OBS_HZ:g} Hz); "
                  f"{o_.get('duplicate_requests')} duplicate request(s); hint-gated "
                  f"{gs_.get('s')} s in {gs_.get('spans')} span(s); camera "
                  f"{(o_.get('camera_delivery') or {}).get('hz')} Hz delivered"
                  + (f"; *** {o_['error']}" if o_.get("error") else ""))
            c_ = metrics.get("camera_to_setpoint") or {}
            f_, a_ = c_.get("cam_to_setpoint_ms") or {}, c_.get("cam_age_ms") or {}
            print(f"[report] camera to setpoint (frame ARRIVAL to publication, loop clock): "
                  f"first setpoint p50 {f_.get('p50')} / p95 {f_.get('p95')} / p99 "
                  f"{f_.get('p99')} / max {f_.get('max')} ms over {f_.get('n')} decision(s); "
                  f"frame age at every setpoint p50 {a_.get('p50')} / max {a_.get('max')} ms, "
                  f"{a_.get('over_budget')} of {a_.get('n')} over 100 ms"
                  + (f"; *** {c_['error']}" if c_.get("error") else ""))
        ef_ = metrics.get("estimator_follow")
        if ef_:
            print(f"[report] estimator follow: ticks by law {ef_.get('ticks_by_law')} "
                  f"(on the estimate {ef_.get('estimate_frac')}); stand-off "
                  f"{(ef_.get('config') or {}).get('want_range_m')} m; line-of-sight "
                  f"feed-forward on {ef_.get('los_ff_ticks')} tick(s)")
        # The target estimator's line: the VLA follow's or the estimator follow's.
        st_ = metrics.get("subject_track")
        if st_ and st_.get("on"):
            tt_ = st_.get("truth") or {}
            print(f"[report] target estimator: served on {st_.get('ticks_served')} of "
                  f"{st_.get('ticks')} ticks (age p50 "
                  f"{(st_.get('est_age_s') or {}).get('p50')} s); ticks by state "
                  f"{st_.get('ticks_by_state')}; lapses {st_.get('lapses')}, "
                  f"re-acquisitions {st_.get('reacquisitions')}; memory "
                  f"{st_.get('memory_ticks')}; estimate within {tt_.get('near_m')} m of "
                  f"the truth on {tt_.get('est_lock_frac')} of the ticks read"
                  + (f"; *** {st_['errors']} fault(s): {st_.get('first_error')}"
                     if st_.get("errors") else ""))
        # A hover that crashed was landed and recorded above, and is still a
        # failed run: the launchers count a non-zero exit as one; so is a VLA
        # mission that ended on a VLA failure (exit 3, as demo/vla_fly.py).
        if vla_on and end_reason in VM.VLA_FAILURE_ENDS:
            return EXIT_VLA_FAILURE
        return 1 if gate_crash else 0
    except BaseException as exc:
        crash = exc                        # for the net below; it still propagates
        raise
    finally:
        # How the run is ending, read before the teardown can replace it: an
        # exception on its way out is a failed run, a Ctrl+C a stopped one.
        _leaving = sys.exc_info()[0]
        try:
            # THE NET FROM THE ARMING ON (land_on_the_way_out; review 3,
            # 2026-10-22), first - before the simulator connection and SITL go:
            # an aircraft that was armed and whose loop never made its LAND
            # decision is landed (unless the autopilot or a pilot has it) and
            # its episode ended, whatever raised. A bring-up that raised before
            # arming (a refused arm, an EKF that never converged) is not armed.
            try:
                armed = airborne or getattr(getattr(link, "snap", None), "armed", None) is True
            except Exception:                                         # noqa: BLE001
                armed = airborne
            if link is not None and armed and not land_decided:
                try:
                    t_net = None if mission_t0[0] is None else round(clock() - mission_t0[0], 3)
                except Exception:                                     # noqa: BLE001
                    t_net = None
                # The loop's takeover (a Ctrl+C inside the loop's finally, before
                # its LAND decision), else the hover's. The net guards each of its
                # own steps; this guard is for anything else, so the teardown
                # below (SITL, the camera) still runs.
                try:
                    land_on_the_way_out(link, asks, rec, crash=crash, stage=stage,
                                        takeover=(takeover if takeover is not None else
                                                  None if watch is None else watch.takeover),
                                        fsm=fsm, throttle=throttle, t=t_net)
                except Exception as exc:                              # noqa: BLE001
                    _say(f"[node]   *** the net failed: {type(exc).__name__}: {exc}")
            # The REST hot-apply listener does not outlive the run either (a
            # no-op when the loop's finally already closed it).
            rail_hotapply.stop(hot, hot_gate, "the run ended", log=_say)
            # Whatever happened above - a refused arm, an EKF that never
            # converged, Ctrl+C - the simulator connection and the SITL this
            # run started do not outlive it. Nor does the recorder thread: it
            # stops before the camera it reads from (with its recorder.json).
            if recorder is not None and rec_summary is None:
                try:
                    stop_recorder(recorder, view_dir)
                except Exception as exc:                              # noqa: BLE001
                    print(f"[view]   recorder stop failed: {type(exc).__name__}: {exc}")
            # The detector thread (started before SITL, so a refused bring-up
            # leaves it running) and the RPC worker, before the simulator
            # connection they use goes.
            if grounder is not None and callable(getattr(grounder, "stop", None)):
                grounder.stop()
            rpc.close()
            # The VLA's link and records, its frame writer, and a server this
            # run started - on every way out (an attached server is never
            # stopped: VLAServer.stop has no process to stop). Each guarded:
            # the simulator connection and SITL below must still go.
            for what, fn in (
                    ("pilot", (vla.close if vla is not None else
                               vla_backend.close if vla_backend is not None else
                               getattr(vla_rec.get("client"), "close", None))),
                    ("frames", None if vla_saver is None else vla_saver.close),
                    ("server", None if vla_srv is None else vla_srv.stop),
                    # The captures sent, after a crash too (P3-15).
                    ("captures", None if (not vla_on or vla_captures_written
                                          or vla_frames is None) else
                     lambda: _vm().write_jsonl(out / "vla_captures.jsonl",
                                               list(getattr(vla_frames, "log", None) or [])))):
                if fn is None:
                    continue
                try:
                    fn()
                except Exception as exc:                              # noqa: BLE001
                    _say(f"[vla]    {what} close failed: {type(exc).__name__}: {exc}")
            if camera is not None and hasattr(camera, "stop"):
                camera.stop()
            if sitl_proc is not None:
                run_cmd(rail.wsl_stop_command(args.wsl_distro), check=False)
                sitl_proc.wait(timeout=30)
            if sitl_log is not None:
                sitl_log.close()
        finally:
            # The Dashboard row ends LAST (never raises). It was the first
            # statement of this `finally`, ahead of the teardown: a second
            # Ctrl+C during its write retries (up to ~0.3 s) would have
            # skipped the SITL stop that keeps SITL from outliving the run
            # (review of 8035d0a, P8). Its own `finally`, so a teardown
            # that raises cannot leave the row "running" either (and reads
            # as failed). A Ctrl+C the mission loop caught is "stopped" too
            # (`interrupted` above), unless something failed after it.
            _leaving = _leaving or sys.exc_info()[0]
            # One stop: a run refused before arming returns (live_end:
            # "failed" and why); otherwise as before.
            status, detail = (live_end if (live_end is not None and _leaving is None) else (
                ("stopped" if interrupted else "ok") if _leaving is None else
                "stopped" if issubclass(_leaving, KeyboardInterrupt) else "failed", None))
            live.stop(status, detail=detail)


def main(argv=None) -> int:
    args = build_parser().parse_args(join_cli(sys.argv[1:] if argv is None else argv))
    # A flight asks the OS for its 1 ms timer for its whole life, as
    # follow_vlm's main() does (_fine_timer_begin); a dry run or a scene load
    # sleeps on nothing.
    fine = None if (args.dry_run or args.scene_only) else _fine_timer_begin()
    try:
        return run(args, live_status=not (args.dry_run or args.scene_only))
    except PreArmRefused as exc:
        # Refused disarmed, nothing armed (the VLA's warm-up gate): not a
        # crash, and on record in events.jsonl.
        print(f"[node]   *** REFUSED before arming ({exc.kind}): {exc}", file=sys.stderr)
        return EXIT_REFUSED
    finally:
        _fine_timer_end(fine)


if __name__ == "__main__":
    sys.exit(main())
