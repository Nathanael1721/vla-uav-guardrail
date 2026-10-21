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
episode id. audit.jsonl carries this node's FSM verdict (audit_tick), the
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

WHAT THIS IS NOT. follow_vlm.fly()'s further layers - the target estimator,
identity tiers, re-acquisition, trail following, the fence guard, the landing
site - live inside that 2,500-line function and are not reused here; moving
them needs a refactor of follow_vlm.py. Until then this node tracks with the
servo law on the detector's box, and its numbers are not comparable with the
simple_flight CityLife flights. Its tracking score comes from the subject
truth above when the scene provides it (None otherwise).

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
import time
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
from guardrail.bundle import load_for_flight                          # noqa: E402
from guardrail.compiler import ConstraintCompiler, Mission            # noqa: E402
from guardrail.fence_export import (defer_near, is_enforced,       # noqa: E402
                                    policy_zones, zone_clearance_m, zones_near)
from guardrail.fsm import (DEFAULT_THETA_HORIZON_S, FAILSAFE_STATES,  # noqa: E402
                           THETA_HORIZON_S, EscalationFSM, FSMConfig,
                           FSMState, tick_input_from_decision)
from guardrail.kpi import compute_from_dir, null_hover_mission        # noqa: E402
from guardrail.manifest import (DEV_YAML_FLAG, MAX_HEADING_ERR_DEG,   # noqa: E402
                                NO_VLA, TOPOLOGY_DEV, TOPOLOGY_HIL, build_manifest,
                                flight_bundle_path, flight_policy_choice,
                                is_kpi_grade)
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

import rail                                                           # noqa: E402
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


def _fv():
    """demo/follow_vlm.py, imported on first use (1.3 s; no GPU, no sim)."""
    return importlib.import_module("follow_vlm")


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
    """

    def __init__(self, obs, sim_cfg: dict, address: str = "127.0.0.1",
                 projectairsim=None, view_switch: int = 1,
                 robot: str = rail.ROBOT_NAME, attach: bool = False,
                 chase: bool = False):
        self.obs, self.cfg, self.address = obs, sim_cfg, address
        self.pas = projectairsim
        self.view_switch, self.robot, self.attach = view_switch, robot, attach
        self.chase = chase
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
        self.scene_record.update(parent_topic=world.parent_topic,
                                 robot_topics_live=None if live is None else len(live))
        if live is not None and not live:
            raise RuntimeError(
                f"attach: the simulator at {self.address} serves no topic under "
                f"{prefix}, so scene {data['id']!r} is not the one loaded there. "
                f"Load it on the desktop first (run_pas_ardupilot.ps1 -Step scene), "
                f"start ArduPilot after it (-Step sitl), then start this node")
        return world

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
        self.client.subscribe(self.drone.sensors["FrontCamera"]["scene_camera"],
                              lambda _, m: self.obs.put_front(m))
        try:
            self.client.subscribe(self.drone.sensors["FrontCamera"]["depth_camera"],
                                  lambda _, m: self.obs.put_depth(m))
            self.have_depth = True
        except Exception as exc:                              # noqa: BLE001
            print(f"[range] no depth stream ({exc}); range from apparent width")
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


class SubjectTruth:
    """The followed subject's simulator pose, read once a tick for the RECORD:
    `read()` -> {"xy": (x, y) in the Shield's frame, or None, "why": None or
    the reason}. Nothing the Shield or the pilot sees comes from here.

    The read is a synchronous RPC on the simulator's service socket (shared
    with truth_geo, so it cannot move to a thread: one Req0 socket, one
    request at a time), made after the tick's setpoint went out. Its
    latency is recorded (`summary()["read_ms"]`). After TRUTH_MAX_ERRORS
    failed reads in a row it backs off and retries, never stops.

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
                 timer=time.perf_counter, aircraft_actor: str = AIRCRAFT_ACTOR):
        self.camera, self.actor, self.scene_home, self.anchor = camera, actor, scene_home, anchor
        self.timer = timer
        self.aircraft_actor = aircraft_actor
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
        self.attempts += 1
        t0 = self.timer()
        try:
            x, y = self.camera.object_xy(self.actor)
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
        self.frame_attempts += 1
        t0 = self.timer()
        why = None
        try:
            vx, vy, vyaw = self._aircraft_pose()
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
                         "subject_bearing_gap_deg")


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
        to the flown policy;
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
        out = {"reason": reason, "generation": self.policy.generation,
               "policy_hash": self.policy.policy_hash,
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
        short timeout per phase, MIDFLIGHT_FENCE_TIMEOUT_S)."""
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
    loop."""

    def __init__(self, indicator, *, query: str, shield_on: bool,
                 hold_ap_mode: str = DEFAULT_HOLD_MODE):
        self.ind = indicator
        self.query, self.shield_on, self.hold_ap_mode = query, shield_on, hold_ap_mode
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
        return {"t": float(now), "sep": sep_m,
                "brg": 0.0 if bearing is None else math.degrees(bearing),
                "fwd": fwd, "alt": float(state.up), "shield": bool(touched),
                "query": self.query, "mode": info.get("mode"),
                "mode_label": f"{TRACK_WORDS.get(info.get('mode'), 'SCANNING for target')} | {arm}",
                "tier": None, "fence_d": None, "fence_hold": False, "fence_mode": None,
                "fence_kind": None, "policy": pol}

    def summary(self) -> dict:
        ind = self.ind
        return {"enabled": ind is not None, "error": self.error,
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
    # pilot (follow_vlm's defaults; cruise 8 m as the CityLife launcher flies)
    ap.add_argument("--cruise-alt", type=float, default=8.0)
    ap.add_argument("--yaw-gain", type=float, default=1.2)
    ap.add_argument("--want-width", type=float, default=0.10)
    ap.add_argument("--speed-max", type=float, default=4.0)
    ap.add_argument("--alt-gain", type=float, default=0.6)
    ap.add_argument("--climb-max", type=float, default=1.8)
    ap.add_argument("--det-thresh", type=float, default=0.02)
    ap.add_argument("--colour-min", type=float, default=0.10)
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


def sim_configs(args, out: Path):
    """(sim_cfg, plan) for this run: rendered with today's addresses when
    this node loads the scene, the template marked "attach" when it does not."""
    if args.camera != "projectairsim":
        return None, None
    if args.scene_mode == "attach":
        return rail.write_attach_configs(out / "pas_config"), None
    plan = rail.plan_network(args.network, args.windows_ip, args.wsl_ip)
    return rail.write_sim_configs(out / "pas_config", plan), plan


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
    return report


class SceneLoader:
    """Loads the rendered scene and nothing else (no camera subscriptions):
    what `-Step scene` needs before ArduPilot is started."""

    def __init__(self, sim_cfg: dict, address: str = "127.0.0.1", projectairsim=None):
        self.cfg, self.address, self.pas = sim_cfg, address, projectairsim
        self.client = None

    def start(self) -> None:
        pas = self.pas or importlib.import_module("projectairsim")
        self.client = pas.ProjectAirSimClient(address=self.address)
        self.client.connect()
        world = pas.World(self.client, self.cfg["scene"], delay_after_load_sec=2,
                          sim_config_path=self.cfg["dir"])
        try:
            world.switch_streaming_view()
        except Exception:                                     # noqa: BLE001
            pass

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
    camera = camera or SceneLoader(sim_cfg, address=args.sim_host)
    try:
        camera.start()
        rec = {"scene": sim_cfg, "loaded_wall": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
               "next": "start ArduPilot now (run_pas_ardupilot.ps1 -Step sitl); "
                       "reloading this scene while it runs drives its clock backwards"}
        (out / "scene_load.json").write_text(json.dumps(rec, indent=2), encoding="utf-8")
        print(f"[scene]  {sim_cfg['scene_id']} loaded from {sim_cfg['dir']}; "
              f"start ArduPilot now, never before a reload")
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
        recorder_cls=None, live_status: bool = False) -> int:
    """Fly one run. Every collaborator is injectable so the loop can be tested
    without a simulator, an autopilot or a GPU (tests/test_pas_ardupilot.py);
    `recorder_cls` stands in for demo/recorder.FrameRecorder (--save-view).
    `live_status` writes a Dashboard heartbeat (main() turns it on for a
    flight; the tests leave it off, so they never touch the live folder)."""
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
    scene_home = rail.home_geo_point(scene) if scene else None
    # The policy's own origin, else the scene's home-geo-point; with neither,
    # the EKF's first fix (set at the pre-arm step below).
    anchor = FrameAnchor.choose(policy, scene_home)
    # The follow mission the CSPs are compiled for: from the spawn point, the
    # target proxy the start itself (follow_mission).
    start_xy, start_src = scene_spawn_xy(scene, anchor)
    mission = follow_mission(args.object, start_xy, args.cruise_alt, args.speed_max)
    mission_rec = {"task": mission.task_text, "start": list(start_xy),
                   "start_source": start_src,
                   "target": [mission.target_x, mission.target_y],
                   "target_source": ("the start (proxy): a follow mission's subject "
                                     "position is not known before the flight"),
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
    audit = AuditLogger(out / "audit.jsonl", policy)
    if obs is None:
        from semantic_demo import SemanticObs
        obs = SemanticObs()
    sitl_proc = sitl_log = None
    save_view = bool(getattr(args, "save_view", False))
    if camera is None and args.camera == "projectairsim":
        camera = ProjectAirSimCamera(obs, sim_cfg, address=args.sim_host,
                                     attach=args.scene_mode == "attach",
                                     chase=save_view)
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
    try:
        if camera is not None:
            camera.start()                     # loads the scene: BEFORE SITL starts
            live.beat(detail="scene loaded")                                # memory only
        if save_view:
            # Started now, writing nothing until the loop's first set_hud():
            # the video begins at mission start (after the take-off to cruise),
            # as follow_vlm's does, and recorder.json measures the rate over
            # the frames actually written.
            recorder, view_dir, rec_why = start_recorder(args, out, camera, obs,
                                                         recorder_cls=recorder_cls)
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
                               hold_ap_mode=hold_ap_mode)
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
                link = MavrosLink()
                if args.camera == "ros2":
                    Ros2ImageSource(link.node, obs, args.image_topic)
        ap_fence = backstop = None
        pre_arm: dict = {}

        def before_arm(phase: str = "before arming") -> None:
            """With the position usable and before GUIDED / arming: the frame's
            anchor (the first fix, when nothing else anchors it), the
            autopilot's own GeoFence as it holds it (recorded; no parameter is
            changed), and the policy's zones uploaded onto it."""
            nonlocal anchor, ap_fence, backstop
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

        print(f"[link]   {link.name}: bring-up to {args.cruise_alt} m")
        live.beat(detail=f"EKF, arming and take-off to {args.cruise_alt:g} m")  # memory only
        if "before_arm" in inspect.signature(link.bring_up).parameters:
            link.bring_up(args.cruise_alt, before_arm=before_arm)
        else:
            link.bring_up(args.cruise_alt)
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

        # ---- detector and pilot --------------------------------------------------
        fv = None
        hfov = args.hfov
        if grounder is None or hfov is None:
            fv = _fv()
        if hfov is None:
            hfov = (fv.camera_hfov_deg(config_path=Path(sim_cfg["dir"]) / sim_cfg["robot"])
                    if sim_cfg else fv.CAMERA_HFOV_DEG)
        if grounder is None:
            live.beat(detail="loading the detector")                        # memory only
            grounder = fv.Grounder(obs, args.object, thresh=args.det_thresh,
                                   colour_min=args.colour_min,
                                   log_path=out / "detections.jsonl")
            grounder.start()
        width_m, subject_class = (fv or _fv()).subject_width(args.object)
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
                        if c.binds(subject_class)]
        unclassified = None
        if subject_class is None:
            unclassified = {"object": args.object, "width_m": width_m,
                            "binding_rules": standoff_ids}
            rec.event("subject_unclassified", **unclassified)
            print(f"[subject] WARNING no class known for {args.object!r}: only the "
                  f"catch-all stand-off(s) {standoff_ids or 'none'} bind it, and the "
                  f"width range assumes {width_m:.2f} m, a car - a subject that is "
                  f"not car-sized is ranged wrong by the ratio")
        pilot = FollowPilot(grounder, hfov, yaw_gain=args.yaw_gain,
                            want_w_frac=args.want_width, speed_max=args.speed_max,
                            cruise_alt=args.cruise_alt, alt_gain=args.alt_gain,
                            climb_max=args.climb_max)
        # The subject's simulator pose, for EVALUATION only (SubjectTruth): the
        # Shield below is given the estimate, never this.
        actor, actor_why = subject_actor_for(getattr(args, "subject_actor", "auto"),
                                             args.object, subject_class)
        truth = SubjectTruth(camera, actor, actor_why, scene_home, anchor)
        rings = [c.min_range_m for c in policy.by_type(SubjectStandoff)
                 if c.binds(subject_class)]
        ring_m = max(rings) if rings else None
        print(f"[truth]  subject pose: {truth.source or truth.why_off}")

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
        # What was asked of the autopilot, in the FSM's words and ArduCopter's,
        # and which request can explain a mode change (ModeRequests).
        asks = ModeRequests(hold_mode)
        takeover = None
        fsm_fault = None
        failsafe_t = None
        end_reason = "max_s reached"
        prev_mode = link.snap.mode
        fence0 = link.snap.fence_breach_count
        g0 = grounder.latest()
        det_n0 = (g0.get("n_seen") or 0) + (g0.get("n_miss") or 0)
        # The FCU clock is paired with the wall time it ARRIVED at, not with
        # "now": SYSTEM_TIME comes at 2 Hz, and pairing a stale FCU stamp with
        # the current wall time read a perfect real-time run as 0.97x.
        tb0 = (link.snap.time_boot_ms, getattr(link.snap, "t_time_boot", None))
        t0 = clock()
        mission_t0[0] = t0
        rec.event("mission_start", t=0.0, autopilot_mode=prev_mode,
                  fsm=None if fsm is None else fsm.config.to_dict(),
                  rule_clock_reading=rule_now().isoformat(timespec="seconds"))
        # The two simulator checks (SIM_CHECKS, below), half a period apart.
        t_frame, t_geo = -math.inf, t0 - TRUTH_FRAME_PERIOD_S / 2.0
        try:
            while clock() - t0 < args.max_s:
                t_tick = clock()
                tick_starts.append(t_tick - t0)
                live.beat(progress=(t_tick - t0) / max(float(args.max_s), 1.0))   # memory only
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
                # for a mode the node did not ask for: its fence fired RTL, a
                # failsafe, or a pilot in Mission Planner. From here it has the
                # aircraft, and nothing more is commanded. (Until 2026-10-13
                # only a change out of GUIDED counted, so a pilot taking the
                # aircraft out of the FSM's LOITER went unseen, and the
                # throttle override went on holding channel 3.)
                if mode == "GUIDED" and prev_mode != "GUIDED":
                    asks.entered_guided()
                if (takeover is None and mode != prev_mode and mode != "GUIDED"
                        and (prev_mode == "GUIDED" or asks.explains(prev_mode))
                        and not asks.explains(mode)):
                    takeover = {"t": round(now, 3), "mode": mode, "previous": prev_mode,
                                "fence_breach_count": s.fence_breach_count,
                                "statustext": s.statustext[-3:]}
                    rec.event("autopilot_took_over", t=now, mode=mode, previous=prev_mode)
                    print(f"[link]   *** the autopilot took the aircraft: GUIDED -> "
                          f"{mode}; commanding stops")
                prev_mode = mode
                yaw = math.radians(st.yaw_deg)
                obs.put_pose_full(t_tick, st.x, st.y, st.up, yaw, s.pitch or 0.0, s.roll or 0.0)
                body, info = pilot.step(t_tick, st.up)
                raw = body_to_world(*body, yaw)
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
                            if plan["choice"] == "brake" or forced:
                                hold_deferred, hold_why = False, None
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
                if flown is not None:
                    link.send(flown, raw_body=body)
                    t_last_sent = clock()
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
                # given the estimate.
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
                    g = camera.truth_geo()
                    if g is not None and s.lat is not None:
                        gx, gy = anchor.to_scene(g["lat"], g["lon"])
                        ex, ey = anchor.to_scene(s.lat, s.lon)
                        ekf_err.append(math.hypot(gx - ex, gy - ey))
                        truth.note_ekf_vs_sim(ekf_err[-1])

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
                row = {
                    "t": round(now, 3), "tick": tick,
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
                # PACE TO AN ABSOLUTE DEADLINE (next_tick_deadline), as
                # follow_vlm does: a late wake is repaid by the next sleep.
                _now = clock()
                _work = _now - t_tick
                work_ms.append(_work * 1000.0)
                tick_deadline = next_tick_deadline(tick_deadline, t_tick, _now)
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
            live.beat(progress=1.0, phase="landing", detail="episode over; landing")  # memory only
            # The throttle goes back to the radio first: an override must not
            # outlive the hold it was for.
            try:
                throttle.release(round(clock() - t0, 3), f"episode end ({end_reason})")
            except Exception as exc:                              # noqa: BLE001
                print(f"[hold]   override release failed: {exc}")
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
        dwell = zone_dwell(shield, traj, TICK, clock_key="clock", point_s=traj_s)
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
        loop_hz = round(len(rows) / mission_s, 3)
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
        if pace_hz < LOOP_HZ_GATE:
            print(f"[loop]   *** the loop ticked at {pace_hz} Hz, under the "
                  f"{LOOP_HZ_GATE:g} Hz gate (tick work p95 "
                  f"{spread([round(v, 3) for v in work_ms]).get('p95')} ms; "
                  f"{n_stale} stale-link and {n_no_state} no-state tick(s) of "
                  f"{len(tick_starts)})")

        link_ev = link.evidence()
        ev = {"camera_source": args.camera,
              "controller_type": sim_cfg["controller_type"] if sim_cfg else None,
              "sim_same_desktop": (topo.sim_on_this_desktop(args.sim_host)
                                   if args.camera == "projectairsim" else None),
              "link": link.name,
              "autopilot": {k: link_ev.get(k) for k in ("heartbeat", "is_ardupilot",
                                                        "heartbeat_seen", "ardupilot_seen")},
              "topology_evidence": link_ev.get("topology_evidence") or {},
              "g1": g1_status(args.g1_verdict)}
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
            "shield": args.shield, "ticks": len(rows), "ticks_without_state": n_no_state,
            "ticks_flown": sum(1 for r in rows if r.get("flown")),
            "ticks_not_flown": sum(1 for r in rows if r.get("flown") is False),
            "shield_lookahead": {"horizon_s": LOOKAHEAD_S, "dt_s": LOOKAHEAD_DT_S,
                                 "poses": int(round(LOOKAHEAD_S / LOOKAHEAD_DT_S))},
            "mission_s": round(mission_s, 2), "end_reason": end_reason,
            "loop_hz": loop_hz,
            # The project's loop gate, and how the loop was paced to it
            # (next_tick_deadline) on which OS timer (None: not measured,
            # run() was called without main()). The gate is judged on the
            # loop's own tick starts (loop_hz_paced, every tick counted), not
            # on loop_hz, whose clock includes the mission-end LAND.
            "loop_hz_paced": loop_hz_paced, "loop_ticks": len(tick_starts),
            "loop_hz_gate": LOOP_HZ_GATE, "loop_hz_below_gate": pace_hz < LOOP_HZ_GATE,
            "loop_hz_gate_basis": ("loop_hz_paced: (ticks - 1) / the span of their starts"
                                   if loop_hz_paced is not None else
                                   "loop_hz: fewer than two ticks to pace"),
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
            # detector that never ran scores 0.0 here.
            "det_hz": round(det_n / mission_s, 3), "det_hz_grant_target": GRANT_OBS_HZ,
            "det_hz_null": 0.0,
            "track_ticks": pilot.n_track, "search_ticks": pilot.n_search,
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
            "interventions": n_touched, "brakes": n_braked, "escape_ticks": n_escaped,
            # Dwell in seconds from the points' own timestamps (point_durations),
            # the tick counts beside them. dwell_gap_s: time between points
            # with no live state (no state, a stale link), charged to no point;
            # dwell_gap_points: how many points such a gap followed.
            "nfz_s": round(dwell["nfz_s"], 2), "alt_violation_s": round(alt_s, 2),
            "nfz_ticks": nfz_ticks, "alt_violation_ticks": alt_ticks,
            "dwell_gap_s": round(dwell_gap_s, 2), "dwell_gap_points": len(gap_from),
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
            "frac_within_30m": tsum["frac_within_30m"],
            "standoff_s": tsum["standoff_s"],
            "standoff_basis": tsum["standoff_basis"],
            "standoff_min_range_m": tsum["min_range_m"],
            "follow_scored": scored,
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
            "detector": "google/owlvit-base-patch32 (demo/follow_vlm.DETECTOR_ID)",
        }
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
        if not scored and (tsum.get("frame_check") or {}).get("agrees") is False:
            why.append("the follow is not scored: "
                       + frame_disagreement(tsum["frame_check"]))
        elif not scored:
            # A follow's mission test is the follow proxy and the stand-off
            # dwell; read as None, guardrail.kpi passes the mission on
            # silence. Not a measured success.
            why.append(f"the follow is not scored: the subject's pose was read on "
                       f"{tsum['ticks_read']}/{len(rows)} ticks (needs "
                       f"{TRUTH_MIN_COVERAGE:.0%}; {truth.why_off or truth.last_why or 'never read'})")
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
        return 0
    finally:
        # How the run is ending, read before the teardown can replace it: an
        # exception on its way out is a failed run, a Ctrl+C a stopped one.
        _leaving = sys.exc_info()[0]
        try:
            # Whatever happened above - a refused arm, an EKF that never
            # converged, Ctrl+C - the simulator connection and the SITL this
            # run started do not outlive it. Nor does the recorder thread: it
            # stops before the camera it reads from (with its recorder.json).
            if recorder is not None and rec_summary is None:
                try:
                    stop_recorder(recorder, view_dir)
                except Exception as exc:                          # noqa: BLE001
                    print(f"[view]   recorder stop failed: {type(exc).__name__}: {exc}")
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
            live.stop(("stopped" if interrupted else "ok") if _leaving is None else
                      "stopped" if issubclass(_leaving, KeyboardInterrupt) else "failed")


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    # A flight asks the OS for its 1 ms timer for its whole life, as
    # follow_vlm's main() does (_fine_timer_begin); a dry run or a scene load
    # sleeps on nothing.
    fine = None if (args.dry_run or args.scene_only) else _fine_timer_begin()
    try:
        return run(args, live_status=not (args.dry_run or args.scene_only))
    finally:
        _fine_timer_end(fine)


if __name__ == "__main__":
    sys.exit(main())
