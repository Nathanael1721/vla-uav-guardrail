"""One protocol for comparing VLA backends, and the table of what is measured where.

    python tools/vla_backend_table.py protocol                    # the protocol, as JSON
    python tools/vla_backend_table.py table                       # assemble from files on disk
    python tools/vla_backend_table.py table --demo-out <checkout>/demo/out \
        --experiments-out <checkout>/experiments/out --runs <more run folders> \
        --bundles <checkout>/bundles      # from a worktree: the run folders are not in git
    python tools/vla_backend_table.py table --default aiotvla_v10b --default-scope \
        "the follow task" --default-date 2026-11-20   # refused unless the set holds it
    python tools/vla_backend_table.py measure --backend stub  --topology dev
    python tools/vla_backend_table.py measure --backend bc_v3 --topology dev
    python tools/vla_backend_table.py measure --backend openvla_7b_bf16 \
        --allow-gpu --model-root ~/models --topology hil   # the AGX Orin: no bitsandbytes
    python tools/vla_backend_table.py measure --backend openvla_7b_int8wo \
        --allow-gpu --model-root ~/models --topology hil   # an attempt: a refusal is recorded
    python tools/vla_backend_table.py serving --bench <bench JSONs> --vram-budget-gb 10 \
        [--desktop-baseline-gb 3.4]   # for benches without a per-run reading before the load
        # AerialVLA serving configurations (demo/vla_server_v2.py bench), added to
        # docs/data/aerialvla_serving_4090.json and to the committed table; options,
        # no default

WHY THIS EXISTS

The grant (Architecture constraints p.2) names the backends the slot must take
- "CognitiveDrone, OpenVLA generic, BitVLA, in-house stubs" - and says the
project "does not commit to any one of them as a 'default' until the Q2
measurement set lands". Audit cards ARCH-30 and X-04 found no such set: a few
desktop numbers from July and August, each taken under different conditions,
and a mid-term table (docs/MIDTERM-REPORT-Aug2026.md section 6.4) with no rate
column and no common scenario. ARCH-03 adds that no VLA has been timed on the
Jetson Orin the grant measures latency on.

This tool fixes the protocol first, then fills the table from what is already
on disk - citing each file and the conditions it was measured under - and marks
every cell that has to be measured on the Orin.

THE PROTOCOL (vla-backend-protocol/2; `protocol` prints it in full)

  R1  inference rate, open loop. Fixed inputs committed with this tool
      (deploy/evidence/vla_protocol_frames/: two 224x448 front-over-down
      mosaics captured in Project AirSim at recorded poses; OpenVLA gets the
      front half), or for state policies the packed states of the dev-rail
      flight ros2_shield_on. Load once, 5 untimed warm-up inferences, 50 timed,
      greedy decoding, nothing else on the GPU. Per-inference wall time from
      input ready to action decoded: p50, p95, max, Hz at p50, peak VRAM, and
      the number of outputs that did not decode to an action (never dropped).
      Reference line: the grant's 10 Hz (100 ms per action). Floor: the stub on
      the same host.
  C1  closed-loop success. policies/urban_demo_policy.yaml, "fly to (40, 40)
      at 6 m/s altitude 20", 5 seeds, Shield on, 120 s limit, reached = within
      3 m of the target. No assist: goal blend 0, no planner, no path
      follower; a run with any assist is recorded as assisted and is not C1.
      Reported: reached n/N, time to reach, Shield interventions per 100 ticks,
      P0 escapes (must be 0), achieved action rate in the loop. Nulls: hover
      (zero action) and the stub, which is handed the target coordinates - a
      learned backend that does not beat the stub has shown nothing on C1.
      Kept as it was; no backend has flown it on hil, and each row states why
      (BACKENDS[...]["c1"]).
  C2  closed-loop follow, added in /2. The Project AirSim follow of the red
      car on hil: ArduPilot SITL + MAVROS 2, the perception node
      (demo/pas_ardupilot/perception_node.py --pilot vla --mission follow),
      the Shield and the VLA server on the Orin, launched by
      deploy/orin/run_pas_perception.sh FAMILY=<family> with --identity
      --negative-query "a traffic light" --target-estimator, 420 s, three
      KPI-grade flights per backend. Reported per flight and never pooled
      into one figure: the share of the mission within 30 m of the car,
      decisions per second over the mission and while the model was asked,
      Shield interventions per 100 ticks, P0 escapes (must be 0), how it
      ended, and the flight's own KPI verdict from its kpi.json (outcome,
      fail reasons, the ticks spent in a P0-unsafe position). What C2 fixes
      - the negative query, the default signed bundle, the camera interval
      of the stream feed, the 420 s limit - is checked from the run's own
      record, not taken from the launch command.
      R1 DOES NOT COVER A SERVED BACKEND: `measure` loads the model in its own
      process, and the four backends served over protocol v2 are timed only
      by their servers' benches, which this table labels "not R1".
      C2 WAS ADDED AFTER THE CHOICE IT BEARS ON: the lab named AIoT-VLA v10b
      for the follow task on 15 November 2026, the test is dated 16 November.
      It was added because that choice rests on a follow and C1 is a GOTO to
      a coordinate, which a follow-only model cannot fly. It therefore
      describes the chosen backend and compares it only with backends that
      flew the same test; it is not a prediction made before the choice.

WHAT IS FOLDED IN FROM DISK, AND UNDER WHICH LABEL

  Orin rate column: an R1 result made on an Orin (vla_rate_*.json), else the
  AGX Orin benches under deploy/evidence/hil/ - demo/vla_server_v2.py bench
  (vlaguard.aerialvla_serving_bench/1), tools/bench_foreign_vla.py
  (vlaguard.foreign_vla_bench/1) and deploy/orin/vla_bf16_check.py - each
  shown as "not R1" with what makes it so: its own inputs where they are not
  the R1 frames, its count, the server in the path, the host evidence it
  carries. A cell with no measurement says "to measure".
  hil closed loop: C2 flights from the run folders (<demo-out>/pas_hil_*,
  --runs) and, for a flight whose folder is not on this host, from the
  committed docs/data/pas_hil_*/flights.json (which hold no within-30 m
  share: the cell says so). A hil follow that is not C2 - a stressor, a
  policy hot-applied in flight, a paraphrased instruction, another feed, a
  start gate that never passed - is listed with the reason, not scored.
  The follow matrices of 28 October (docs/data/progress_1028.json: PC and
  Jetson Orin Nano client) and 4 November (the agx_*_2026110[34] run folders:
  client and server on the AGX Orin): Project AirSim's simple_flight, not
  ArduPilot, so they are records "not C1, not C2".

THE DEFAULT LINE

  `table` writes "Default backend: none" unless --default names a row, with
  --default-scope and --default-date. It is refused when the date is before
  the newest measurement the set holds or before the protocol's own date
  (a default is named after the set, not before), when the date is in the
  future, or when the set is not a comparison: every backend with a launch
  family (the four that can fly C2) needs an Orin timing and the protocol's
  C2 flights, and the stub - R1's floor - an R1 on an Orin. Dates are the
  lab's local dates (UTC+8): a measurement's UTC time is brought to that
  zone before it is compared. The line then carries the backend's own
  numbers, good or not, and says who named it: the lab, with this tool. It
  never records anyone's confirmation.

WHAT `measure` WILL AND WILL NOT DO

`stub` and `bc_v3` run on the CPU (bc_v3 is a 0.54 M-parameter TorchScript MLP
and is loaded with CUDA hidden; a model that lands anywhere but the CPU is
refused, not timed). The OpenVLA-based backends load a 7 B model onto
the GPU, so they refuse to run without `--allow-gpu`; they reuse the loaders
and the predict call of the flight scripts (demo/real_vla_demo.py,
demo/aerialvla_demo.py) rather than a second copy. `openvla_7b_bf16` loads
through deploy/orin/vla_bf16_check.py's own loader (bfloat16, no
quantisation), so R1 runs in the AGX Orin's environment, which installs no
bitsandbytes; a model that did not land on a CUDA device in bfloat16 is
refused, not timed. `openvla_7b_int8wo` is an ATTEMPT at the precision the
grant's gate G3 names: the bf16 load, then torchao's int8 weight-only
quantisation. Where torchao is absent, or the quantisation or the first
inference fails, the result file records the refusal, its stage and its
cause (status "refused", no timing) and the table shows it as that. A bf16
load that fails (a wrong --model-root, a GPU another process holds) is not
an int8 result: the run fails, exit 1, and writes nothing. The
backends served over protocol v2 (AerialVLA bf16, AIoT-VLA, DroneCATS,
RaceVLA) are timed by their servers' own benches, not here.
The OpenVLA prompt is the
flight script's default (`--csp off`); `--csp on` times the longer prompt
with the Constraint Summary Pack ahead of the question, as `real_vla_demo.py
--csp on` flies it. Either way the prompt and its token count are recorded,
because the rate depends on the prompt length. CognitiveDrone and BitVLA have
no weights and no loader on this machine (see the table).

A power mode is not visible to this tool: on the Orin pass it with --note
(`nvpmodel -q`, and whether jetson_clocks locked the clocks).

Results are written to <evidence>/<topology>/vla_rate_<backend>.json, where
<evidence> is $VLAGUARD_EVIDENCE_OUT or deploy/evidence (the Orin compose file
sets the git-ignored deploy/evidence/incoming; see tools/profile_shield_tick.py).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shlex
import statistics
import sys
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

TABLE_SCHEMA = "vlaguard.vla_backend_table/3"
# /2: the host is judged by guardrail.manifest's Orin rule (host.jetson.is_orin).
# /3: protocol /2. The Orin rate cell also takes a labelled non-R1 bench made
# on an Orin; the hil closed-loop cell is two, C1 and C2; follow flights on
# Project AirSim's simple_flight are records of their own; default_backend is
# null or a dated record (`table --default`).
RATE_SCHEMA = "vlaguard.vla_rate/2"
PROTOCOL_ID = "vla-backend-protocol/2"
# The day /2 was written, as the lab's local date (UTC+8). A default cannot be
# dated before it (THE DEFAULT LINE).
PROTOCOL_DATE = "2026-11-16"
# The lab's clock. The dates a default is compared with - the protocol's, the
# one given with --default-date, today's - are the lab's local dates, so a
# measurement's time (stored in UTC) is brought to this zone before it is
# compared with them: a flight flown at 07:30 on the 17th here is of the 17th,
# although its UTC timestamp still reads the 16th.
LAB_TZ = timezone(timedelta(hours=8))
# R1 and C1 are word for word /1's, so a rate result made under either id is R1.
R1_PROTOCOLS = ("vla-backend-protocol/1", PROTOCOL_ID)

EVIDENCE = ROOT / "deploy" / "evidence"
FRAMES_DIR = EVIDENCE / "vla_protocol_frames"
DEFAULT_TABLE_JSON = EVIDENCE / "vla_backend_table.json"
DEFAULT_TABLE_MD = EVIDENCE / "vla_backend_table.md"
CONTRACT_HZ = 10.0
CONTRACT_MS = 1000.0 / CONTRACT_HZ
DOCS_DATA = ROOT / "docs" / "data"
BUNDLES = ROOT / "bundles"
PAS_LAUNCHER = "deploy/orin/run_pas_perception.sh"

# C2's launch configuration. The family's option set is the launch script's
# (FAMILY=<family>; tests/test_deploy_configs.py holds the two together); on
# top of it every C2 flight is given these node arguments, as the full-length
# AIoT-VLA flights of 15 November were (pas_hil_feed_07 / _08).
C2_NODE_ARGS = ("--identity", "--negative-query", "a traffic light", "--target-estimator")
# The feed each family's C2 flies: the launch script's default for it.
C2_FEED = {"aiotvla": "stream", "aerialvla": "tick", "dronecats": "tick", "racevla": "tick"}
C2_FLIGHTS_PER_BACKEND = 3
C2_RANGE_M = 30.0
# What C2 fixes beside the family's option set, each read back from the run's
# own record (metrics.json), never taken on trust from the launch command:
#   the negative query         params.negative_query
#   the policy                 policy_source: the node's default signed bundle,
#                              follow-car-citylife-v<version> (not the -nfz*
#                              bundles, which put a no-fly zone on the route)
#   the camera for the stream  params.front_capture_interval_s: the feed
#   feed                       between ticks needs a 15 Hz FrontCamera
C2_NEGATIVE_QUERY = [C2_NODE_ARGS[C2_NODE_ARGS.index("--negative-query") + 1]]
C2_BUNDLE = re.compile(r"^follow-car-citylife-v\d")
C2_STREAM_CAPTURE_INTERVAL_S = 0.05
# C2's mission limit (the launch script's default). A VLA mission records the
# limit it was given (metrics.json vla.mission.monitor.config.max_s), and that
# record decides: a flight given another limit is not C2, and a flight given
# this one is C2 however it ended - an autopilot takeover after 200 s is a
# result, not a discard. Only a folder with no such record falls back on how
# the flight ended: one that ran to a shorter limit ("max_s reached" with a
# mission shorter than this by more than the slack) or that ended early
# without losing the car cannot be told from a shakedown and is not C2. A
# shakedown is also told by its tag, which the operator gives it (the mark).
C2_MAX_S = 420
C2_MAX_S_SLACK_S = 5.0
C2_SHAKEDOWN_MARK = "_shake"
C2_LOST_THE_CAR = "target_lost"
# Flights known NOT to have flown alone: something else of the lab's ran on the
# rig beside them. No run folder records that, so it is kept here, by hand,
# with the committed note that says so (tests/test_vla_backend_table.py checks
# the note still says it). Tests run on the desktop beside a flight slow the
# simulator's host, and with it the camera and the detector.
C2_NOT_ALONE = {
    "pas_hil_feed_05": "desktop tests ran beside it (docs/data/pas_hil_feed_20261115/"
                       "flights.md)",
    "pas_hil_feed_06": "desktop tests ran beside it (docs/data/pas_hil_feed_20261115/"
                       "flights.md)",
}

PROTOCOL = {
    "id": PROTOCOL_ID,
    "date": PROTOCOL_DATE,
    "changes_from_1": "R1 and C1 are unchanged. C2 is new. The table's Orin rate column also "
                      "takes benches that are not R1, each labelled with what makes it so. "
                      "R1 is stated not to cover a backend served over protocol v2: its "
                      "Orin timing is its server's bench",
    "R1_rate": {
        "kind": "open loop",
        "inputs": {
            "camera_vla": "deploy/evidence/vla_protocol_frames/*.png (224x448 front-over-down "
                          "mosaics, Project AirSim, poses in frames.json); OpenVLA-7B is given "
                          "the front (upper) 224x224 half",
            "state_policy": "rows of flight ros2_shield_on in deploy/evidence/shield_tick_pack.json, "
                            "mission 'fly to the northeast pad at 6 m/s' under "
                            "policies/sim_demo_policy.yaml (the dev rail's own mission)",
        },
        "warmup": 5, "timed": 50,
        "decoding": "greedy; the flight scripts' own max_new_tokens",
        "isolation": "the model's process alone on the GPU: no simulator, no second model",
        "timer": "wall time from input ready to action decoded, per inference",
        "outputs": ["p50_ms", "p95_ms", "max_ms", "hz_at_p50", "peak_vram_gb", "load_s",
                    "undecodable_outputs"],
        "reference": f"the grant's {CONTRACT_HZ:g} Hz VLA -> Shield rate ({CONTRACT_MS:g} ms)",
        "floor": "the stub on the same host",
        "hosts": ["desktop (dev)", "Jetson Orin (hil), same container image definition"],
        "served_backends": "not covered: a backend served over protocol v2 (AerialVLA bf16, "
                           "AIoT-VLA, DroneCATS, RaceVLA) runs in its server's process, which "
                           "`measure` does not load; its Orin timing is its server's own bench "
                           "(demo/vla_server_v2.py bench, tools/bench_foreign_vla.py), shown "
                           "as 'not R1' with what makes it so",
    },
    "C1_closed_loop": {
        "scenario": "policies/urban_demo_policy.yaml; instruction 'fly to (40, 40) at 6 m/s "
                    "altitude 20'; spawn (0, 0); the policy's no-fly zone lies on the "
                    "straight line",
        "rail": "hil: ArduPilot SITL + MAVROS 2, Shield and VLA on the Orin; camera VLAs need "
                "Project AirSim for physics and camera (ardupilot-api controller)",
        "seeds": 5, "time_limit_s": 120, "reached_radius_m": 3.0,
        "assist": "none: goal blend 0, no planner, no path follower. Assisted runs are "
                  "recorded as assisted and are not C1.",
        "outputs": ["reached_n_of_N", "time_to_reach_s", "interventions_per_100_ticks",
                    "p0_escapes", "action_rate_hz_in_loop"],
        "nulls": ["hover (zero action): reached 0/N by construction",
                  "stub P-controller, given the target coordinates: the no-learning "
                  "reference a learned backend has to beat"],
        "status": "this tool reads no C1 flight: a row states why it has not flown C1 "
                  "(BACKENDS[<backend>]['c1']), and the table counts the rows with a C1 "
                  "record",
    },
    "C2_follow_closed_loop": {
        "added": PROTOCOL_DATE,
        "added_after_the_choice": (
            "C2 was added on 16 November 2026, after the lab named AIoT-VLA v10b for the "
            "follow task on 15 November 2026. It was added because that choice rests on a "
            "follow, and C1 is a GOTO to a coordinate that a follow-only model cannot fly. "
            "A test written after the choice it bears on is not a prediction: C2 describes "
            "the chosen backend, and compares it only with backends that flew the same test"),
        "scenario": "the Project AirSim follow: 'follow a red car' in CityLife under the "
                    "perception node's default signed bundle (follow_car_citylife), cruise "
                    "altitude 8 m; the mission clock starts when the start gate passes (5 "
                    "detector inferences in a row with a plausible box within 30 m)",
        "rail": "hil: Project AirSim and ArduPilot SITL on the desktop; MAVROS 2, the "
                "perception node (demo/pas_ardupilot/perception_node.py --pilot vla --mission "
                "follow), the Shield and the VLA server on the Jetson Orin",
        "launch": {"script": f"{PAS_LAUNCHER} FAMILY=<family>",
                   "node_args": list(C2_NODE_ARGS), "feed": dict(C2_FEED),
                   "max_s": C2_MAX_S,
                   "front_capture_interval_s_for_the_stream_feed":
                       C2_STREAM_CAPTURE_INTERVAL_S},
        "counts_as_c2": [
            "KPI-grade by the run's own kpi.json (hil, sim_speedup 1, signed bundle, clean "
            "commit)",
            "no perception stressor, no policy hot-applied in flight, the scenario's own "
            "instruction (not a paraphrase)",
            "--identity and --target-estimator on, the family's feed, and for the stream "
            f"feed a FrontCamera capture interval of {C2_STREAM_CAPTURE_INTERVAL_S:g} s, "
            "each as the run's params record it",
            f"the negative query {C2_NEGATIVE_QUERY!r} in the run's params, and the node's "
            "default signed bundle (follow-car-citylife-v<version>) as the run's "
            "policy_source",
            "flown alone: nothing else of the lab's running on the desktop or the Orin "
            "(no run folder records this; the flights known not to be are listed by hand "
            "in tools/vla_backend_table.py C2_NOT_ALONE)",
            "the start gate passed (a flight that never started has no mission to score)",
            f"the {C2_MAX_S} s limit, by the limit the run's mission monitor records it was "
            f"given: a flight given that limit is C2 however it ended, one given another is "
            f"not; where a folder records no limit, a flight that ran to a shorter one or "
            f"ended early without losing the car is not C2; and a shakedown is not C2 (the "
            f"operator marks it '{C2_SHAKEDOWN_MARK}' in its tag)"],
        "flights_per_backend": C2_FLIGHTS_PER_BACKEND,
        "outputs": [
            f"frac_within_30m: share of the mission's ticks with the aircraft within "
            f"{C2_RANGE_M:g} m of the car's true position (metrics.json; the simulator's "
            f"pose, evaluation only)",
            "decision_hz_mission: answers the model parsed per second over the mission; "
            "decision_hz_asked: the same while it was being asked (not held by the rail, "
            "not withheld because the detector had no box)",
            "interventions_per_100_ticks", "p0_escapes (must be 0)", "ticks, end_reason",
            "the flight's own KPI verdict, from its kpi.json: outcome (success or fail) "
            "with mission_fail_reasons; unsafe_p0_ticks, the ticks the aircraft was in a "
            "position a P0 rule calls unsafe (position-based: 0 P0 escapes does not imply "
            "0 of these); p0_ticks_not_measurable and p0_ticks_blind, the ticks on which "
            "the escape count could not be measured"],
        "reported": "every flight on its own line and the range over the backend's C2 "
                    "flights; a flight that lost the car is a C2 flight, not a discard; a "
                    "value a run did not record is shown as '-', never as 0",
        "nulls": ["the hand-written follow on the same rail (--pilot servo): the "
                  "no-learning reference"],
        "not_separated": "the detector supplies the hint, and while it has no box the model "
                         "is not asked: a C2 score is the model's and the detector's together",
    },
}

# What a row says under C1 when no flight of it is on disk. Every row of
# BACKENDS states its own reason ("c1"); a row added without one falls back
# to protocol /1's bare cell, which the tests refuse.
C1_TO_FLY = "to measure on hil (C1)"
_C1_OPENVLA =("not flown: base OpenVLA is arm-trained and is given no target, so 'reached "
               "(40, 40)' is not defined for it; and nothing on the hil SITL rail publishes "
               "the camera topics the ROS 2 VLA bridge reads (sitl/ros2_vla_bridge_node.py)")
_C1_AERIAL = ("not flown on hil: AerialVLA has flown a GOTO on ArduPilot at dev only "
              "(pas_vla_goto_01 / _02, CHANGELOG 2026-10-29, another goal and policy); on "
              "hil the perception node could fly it (--pilot vla --mission goto), but no "
              "signed bundle of policies/urban_demo_policy.yaml is placed in its scene")
_C2_OPENVLA = "n/a: it takes no follow hint and no server speaks protocol v2 for it"
_C2_AERIAL_4BIT = ("not flyable on the AGX Orin as built: the 4-bit load needs bitsandbytes, "
                   "which its environment does not install (deploy/orin/agx_vla_env.sh); "
                   "AerialVLA's C2 is the bf16 row's")

# What each backend is, where its weights live, and the code that flies it.
# `weights_root` paths are relative to --model-root (D:/models on the lab
# desktop, /models in the containers); `weights_repo` paths to the repository.
BACKENDS = {
    "stub": {
        "label": "Stub (hand-written P-controller, no model)",
        "kind": "code", "inputs": "state + target coordinates",
        "weights_repo": [], "weights_root": [], "code": "guardrail/vla_stub.py",
        "role": "the null; pilot of the stored dev-rail runs (demo/out/ros2_*, not "
                "KPI-bearing: the grant takes KPI figures from hil)",
        "c1": "not flown: the hil rail's stub node (sitl/ros2_vla_stub_node.py) can fly it "
              "once a signed bundle of policies/urban_demo_policy.yaml exists",
        "c2": "n/a: the stub is given coordinates and has no follow; C2's null is the "
              "hand-written follow (--pilot servo)",
    },
    "bc_v3": {
        "label": "Behaviour-cloned state policy (vla_policy_v2)",
        "kind": "state + fence-geometry policy, MLP, 26 features",
        "inputs": "state + policy geometry (no camera, no language)",
        "weights_repo": ["models/vla_policy_v2.pt", "models/vla_policy_v2.onnx"],
        "weights_root": [], "code": "guardrail/vla_bc.py (BCVLAv3)",
        "c1": "not flown: no node on the hil rail loads this policy (the rail's pilot node "
              "is the stub's, sitl/ros2_vla_stub_node.py)",
        "c2": "n/a: a state policy, no camera and no follow",
    },
    "openvla_7b_4bit": {
        "label": "OpenVLA-7B, 4-bit NF4",
        "kind": "camera + language VLA (arm-trained; unnorm_key bridge_orig)",
        "inputs": "224x224 image + instruction",
        "weights_repo": [], "weights_root": ["openvla-7b"], "hf_id": "openvla/openvla-7b",
        "code": "demo/real_vla_demo.py (OpenVLABackend)",
        "class_7b": True, "needs": "bitsandbytes",
        "c1": _C1_OPENVLA, "c2": _C2_OPENVLA,
    },
    "openvla_7b_bf16": {
        "label": "OpenVLA-7B, bf16 (unquantised)",
        "kind": "camera + language VLA (arm-trained; unnorm_key bridge_orig)",
        "inputs": "224x224 image + instruction",
        "weights_repo": [], "weights_root": ["openvla-7b"], "hf_id": "openvla/openvla-7b",
        "code": "deploy/orin/vla_bf16_check.py (load_bf16) with demo/real_vla_demo.py's "
                "prompt and predict call",
        "class_7b": True,
        "c1": _C1_OPENVLA, "c2": _C2_OPENVLA,
    },
    "openvla_7b_int8wo": {
        "label": "OpenVLA-7B, int8 weight-only (torchao): an attempt",
        "kind": "camera + language VLA (arm-trained; unnorm_key bridge_orig)",
        "inputs": "224x224 image + instruction",
        "weights_repo": [], "weights_root": ["openvla-7b"], "hf_id": "openvla/openvla-7b",
        "code": "tools/vla_backend_table.py (load_openvla_int8wo: the bf16 load, then "
                "torchao's int8 weight-only)",
        "class_7b": True, "needs": "torchao",
        "attempt": "the precision the grant's candidate gate G3 names. One per-layer "
                   "figure exists, on the Orin Nano (CHANGELOG 2026-10-28 (night): one "
                   "2048x6144 layer, int8 weight-only 7.7 ms against bf16 9.9 ms); the "
                   "whole model has not been timed in int8 anywhere",
        "c1": _C1_OPENVLA, "c2": _C2_OPENVLA,
    },
    "aerialvla_lora": {
        "label": "AerialVLA LoRA on OpenVLA-7B, 4-bit",
        "kind": "camera + language VLA, UAV-trained adapter",
        "inputs": "224x448 front+down mosaic + direction phrase + object phrase",
        "weights_repo": [], "weights_root": ["openvla-7b", "aerialvla-lora/aero_vla"],
        "hf_id": "XuPeng23/AerialVLA", "code": "demo/aerialvla_demo.py (AerialVLABackend)",
        "class_7b": True, "needs": "bitsandbytes",
        "c1": _C1_AERIAL, "c2": _C2_AERIAL_4BIT,
    },
    "aerialvla_ft_run2": {
        "label": "AerialVLA + our QLoRA fine-tune (run2/epoch1), 4-bit",
        "kind": "camera + language VLA, fine-tuned on 1,961 self-collected samples",
        "inputs": "as AerialVLA",
        "weights_repo": ["models/aerialvla_deploy_manifest.json"],
        "weights_root": ["openvla-7b", "aerialvla-ft/run2/epoch1"],
        "code": "demo/aerialvla_demo.py (AerialVLABackend, --adapter)",
        "class_7b": True, "needs": "bitsandbytes",
        "c1": _C1_AERIAL, "c2": _C2_AERIAL_4BIT,
    },
    "aerialvla_bf16": {
        "label": "AerialVLA LoRA on OpenVLA-7B, bf16 (served over protocol v2)",
        "kind": "camera + language VLA, UAV-trained adapter",
        "inputs": "224x448 front+down mosaic + direction phrase + object phrase",
        "weights_repo": [], "weights_root": ["openvla-7b", "aerialvla-lora/aero_vla"],
        "hf_id": "XuPeng23/AerialVLA",
        "code": "demo/vla_server_v2.py (serve and bench, --precision bf16)",
        "class_7b": True, "family": "aerialvla",
        "measured_by": "python demo/vla_server_v2.py bench --precision bf16 --attn sdpa "
                       "with --merge or --no-merge (the R1 frames, 5 + 50, through the "
                       "server's protocol core)",
        "c1": _C1_AERIAL,
    },
    "aiotvla_v10b": {
        "label": "AIoT-VLA v10b (the lab's own: frozen OWL-ViT + a trained head)",
        "kind": "camera + object words follow model (not a 7 B model)",
        "inputs": "FrontCamera frame (768x432) + object words + pose",
        "weights_repo": [], "weights_root": ["aiotvla/v10b"],
        "code": "demo/vla_server_foreign.py (family aiotvla; the model: training/aiotvla)",
        "family": "aiotvla",
        "measured_by": "python tools/bench_foreign_vla.py --server 127.0.0.1:<port> against "
                       "demo/vla_server_foreign.py serve --model aiotvla",
        "c1": "cannot fly C1: a follow-only model. Its input is a camera frame and object "
              "words, the request's direction phrase is ignored (recorded under "
              "native.ignored), and it is given no goal coordinate. Its closed-loop test "
              "is C2",
    },
    "dronecats_2b": {
        "label": "DroneCATS-Cosmos3-Edge-2B (NAVER), bf16",
        "kind": "camera + language drone VLA, 2 B (go / rotate / think / finished)",
        "inputs": "one frame (at most 640 px wide) + 'go to <object>' + its last 5 actions",
        "weights_repo": [], "weights_root": ["dronecats/DroneCATS-Cosmos3-Edge-2B"],
        "hf_id": "naver-ai/DroneCATS-Cosmos3-Edge-2B",
        "code": "demo/vla_server_foreign.py (family dronecats)", "family": "dronecats",
        "measured_by": "python tools/bench_foreign_vla.py --server 127.0.0.1:<port> against "
                       "demo/vla_server_foreign.py serve --model dronecats",
        "c1": "not flown: its command is 'go to <the object>' on a camera frame, not a goal "
              "coordinate; it has been flown as a follow only",
    },
    "racevla_7b": {
        "label": "RaceVLA drone6_all (Skoltech; an OpenVLA-7B fine-tune), bf16",
        "kind": "camera + language drone VLA, 7 B (body velocities + yaw)",
        "inputs": "224x224 grey frame + 'fly to <object>'",
        "weights_repo": [], "weights_root": ["racevla/RaceVLA_models/drone6_all"],
        "hf_id": "SerValera/RaceVLA_models",
        "code": "demo/vla_server_foreign.py (family racevla)", "family": "racevla",
        "class_7b": True,
        "measured_by": "python tools/bench_foreign_vla.py --server 127.0.0.1:<port> against "
                       "demo/vla_server_foreign.py serve --model racevla",
        "c1": "not flown: its instruction is 'fly to <the object>' on a camera frame, not a "
              "goal coordinate; it has been flown as a follow only",
    },
    "cognitivedrone": {
        "label": "CognitiveDrone (the grant's action-space reference)",
        "kind": "camera + language VLA (OpenVLA-7B fine-tune, 4-D actions)",
        "inputs": "FPV image + instruction",
        "weights_repo": [], "weights_root": [], "code": None,
        "unavailable": ("no checkpoint on this machine. The saved project page "
                        "(source/pages/cognitivedrone.html) links a dataset "
                        "(huggingface ArtemLykov/cognitiveDrone_dataset) and a data-collector "
                        "repository, no model weights; the paper is "
                        "source/papers/2503.02572-CognitiveDrone.pdf"),
        "c1": "not flown: weights not obtained", "c2": "weights not obtained",
    },
    "bitvla": {
        "label": "BitVLA (named by the grant among the switchable backends)",
        "kind": "quantised / bit-efficient camera + language VLA",
        "inputs": "image + instruction",
        "weights_repo": [], "weights_root": [], "code": None,
        "unavailable": ("no checkpoint and no loader in this repository. The grant lists it "
                        "with CognitiveDrone, OpenVLA and in-house stubs as backends the "
                        "slot must accept (Architecture constraints p.2-3); the reference "
                        "design names it as a smaller backend to swap in if a 7 B model "
                        "misses 10 Hz on the Orin (docs/04-risks-and-fallbacks/"
                        "fallback-gates.md), with its citation still to be added"),
        "c1": "not flown: weights not obtained", "c2": "weights not obtained",
    },
}
# The OpenVLA-7B keys `measure` times, and the precision each loads.
OPENVLA_PRECISION = {"openvla_7b_4bit": "nf4", "openvla_7b_bf16": "bf16",
                     "openvla_7b_int8wo": "int8wo"}
GPU_BACKENDS = ("openvla_7b_4bit", "openvla_7b_bf16", "openvla_7b_int8wo",
                "aerialvla_lora", "aerialvla_ft_run2")
CPU_BACKENDS = ("stub", "bc_v3")
# Served over protocol v2 and timed by their servers' benches, not by `measure`.
SERVED_BACKENDS = ("aerialvla_bf16", "aiotvla_v10b", "dronecats_2b", "racevla_7b")

# The AerialVLA serving configurations (wave V1, 2026-10-21): precision x
# attention x LoRA merged or not, each timed by `demo/vla_server_v2.py bench`
# (one process per configuration, the R1 inputs, through the server's own
# protocol core) and summarised by `python tools/vla_backend_table.py serving`
# into SERVING_JSON. The gate is the build plan's: p50 <= 1.0 s standalone
# (vla-assets discovery report, section 7 step 0). The reference for output
# fidelity is upstream's precision, unmerged: bf16 + eager, no merge.
#
# NO SERVING DEFAULT (decided 2026-10-22, review of V1a). The summary used to
# name a "recommended" row - the lowest p50 within a VRAM budget, which was
# nf4/sdpa/merged - and the server took it as its CLI default. But the
# precision changes what the model says (bf16 reproduces the reference on
# 88-96 % of the probe, nf4 on 25-33 %), so the summary now names OPTIONS: the
# low-VRAM row (same fixed rule, applied to what the PROCESS holds), the row
# closest to the reference, the fastest. demo/vla_server_v2.py refuses to load
# a model without an explicit configuration; the CityLife VRAM measurement
# (next wave) decides which flies.
SERVING_JSON = ROOT / "docs" / "data" / "aerialvla_serving_4090.json"
SERVING_SCHEMA = "vlaguard.aerialvla_serving/1"
SERVING_GATE_P50_MS = 1000.0
SERVING_REFERENCE = {"precision": "bf16", "attn": "eager", "merged": False}
SERVING_CHOICE = (
    "demo/vla_server_v2.py refuses to load the model until --precision, "
    "--attn and --merge/--no-merge are all given, and every response's identity carries the "
    "chosen configuration's agreement with the reference. Which configuration flies is decided "
    "by measuring CityLife's own VRAM beside the model (next wave): the bf16 rows fit only if "
    "the renderer and OWL-ViT leave about 17 GB of the 24 GB card free.")
_MIB_GB = 1048576 / 1e9        # nvidia-smi reports MiB; the table's GB are 1e9 bytes

# The assisted evaluation behind the retracted row, its null (the adapter
# before fine-tuning, same assist), and the unassisted runs of both. Read from
# the files every time, so the correction carries the numbers on disk.
ASSIST_EVIDENCE = ("training/eval_ft2_deploy.json", "training/eval_base_deploy.json",
                   "training/eval_ft3_deploy.json", "training/eval_ft2_e1.json",
                   "training/eval_ft2_yg04.json", "training/eval_baseline.json")
_ASSIST_WHY = (
    "the fine-tune's 5/5 reached and 0.996 efficiency (training/eval_ft2_deploy.json) "
    "were flown with goal_blend 0.55, i.e. with the mission-direction assist ON. Its null "
    "does not separate it from the adapter before fine-tuning: under the same assist the "
    "original AerialVLA adapter also reached 5/5 (efficiency 0.942, "
    "training/eval_base_deploy.json), and the run3 fine-tune also reached 5/5 "
    "(training/eval_ft3_deploy.json). Without the assist (goal_blend 0) the run2 "
    "fine-tune reached 0/5 at yaw gain 1.0 (training/eval_ft2_e1.json) and 1/5 at yaw "
    "gain 0.4 (training/eval_ft2_yg04.json), and the original adapter 1/5 "
    "(training/eval_baseline.json). Correct statement: 5/5 reached (efficiency 0.996) "
    "with mission-direction assist (goal_blend 0.55); the original adapter under the "
    "same assist also reached 5/5 (0.942); without assist 0/5 (yaw gain 1.0) and 1/5 "
    "(yaw gain 0.4)")
_NO_ASSIST_WORDS = ("assist", "goal_blend", "goal blend")

# Statements that a file on disk contradicts. Each is checked every time the
# table is built, so a correction that is never made keeps showing up. A line
# is flagged when it holds every `must_contain` string and none of the
# `must_not_contain` ones, and the `file` it is about shows the assist was on.
CLAIM_CHECKS = [
    {
        "doc": "docs/MIDTERM-REPORT-Aug2026.md",
        "must_contain": ["0.996", "goal assist off"], "must_not_contain": [],
        "file": "training/eval_ft2_deploy.json",
        "field": "results[*].params.goal_blend",
        "note": "states the assisted result as 'goal assist off'",
        "why": _ASSIST_WHY,
    },
    {
        # The generator of the line above: a hand fix to the .md would be
        # overwritten the next time the report is built.
        "doc": "tools/build_report_results.py",
        "must_contain": ["0.996", "goal assist off"], "must_not_contain": [],
        "file": "training/eval_ft2_deploy.json",
        "field": "results[*].params.goal_blend",
        "note": "generates docs/MIDTERM-REPORT-Aug2026.md section 6.4, so a hand fix to "
                "the report would be overwritten",
        "why": _ASSIST_WHY,
    },
    {
        # The mid-term deck states the number with no condition at all.
        "doc": "tools/deck/build_midterm_deck.js",
        "must_contain": ["0.996", "reached"], "must_not_contain": list(_NO_ASSIST_WORDS),
        "file": "training/eval_ft2_deploy.json",
        "field": "results[*].params.goal_blend",
        "note": "states 0.996 with no assist condition",
        "why": _ASSIST_WHY,
    },
]


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
from profile_shield_tick import write_lf  # noqa: E402  (LF on every OS)


# Run folders are not in git, so a worktree reads them from another checkout
# (`table --demo-out ...`) or from the disk the runs were moved to (`--runs`).
# A source under such a folder is cited by the name it has in a checkout
# ("demo/out/<tag>/metrics.json"), or by the --runs folder as it was given,
# never by this machine's path to another checkout. build_table registers
# the folders for the length of one build.
_ALIASES: list[tuple[Path, str]] = []


def _abs(p: Path) -> Path:
    """Absolute without following links: demo/out/<tag> is often a link to the
    disk the run was moved to, and the citation is the name in the checkout."""
    return Path(os.path.abspath(p))


def _rel(p: Path) -> str:
    ap = _abs(p)
    for base, label in [(_abs(ROOT), "")] + _ALIASES:
        try:
            rel = ap.relative_to(base).as_posix()
        except ValueError:
            continue
        if rel == ".":
            return label or "."
        return f"{label}/{rel}" if label else rel
    try:
        return Path(p).resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return Path(p).as_posix()


def _read_json(p: Path):
    """The JSON in `p`, or None when it is absent or not JSON."""
    try:
        return json.loads(Path(p).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def _num(v) -> float | None:
    """A number from a JSON number or a numeric string (the 14 November
    summary keeps its figures as strings); None for anything else."""
    if isinstance(v, bool) or v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def lab_date(when) -> str | None:
    """The lab's local date (LAB_TZ, YYYY-MM-DD) of a moment: a Unix time,
    or a UTC timestamp as the files write it ("2026-11-16T23:30:00Z"). The
    dates a default is compared with are the lab's, so every measurement's
    date is this one, never the UTC date. None for what is not a time."""
    if when is None or when == "":
        return None
    try:
        if isinstance(when, (int, float)) and not isinstance(when, bool):
            t = datetime.fromtimestamp(float(when), timezone.utc)
        else:
            s = str(when).strip().replace("Z", "+00:00")
            t = datetime.fromisoformat(s)
            if t.tzinfo is None:                      # a bare stamp is UTC here
                t = t.replace(tzinfo=timezone.utc)
    except (TypeError, ValueError, OverflowError, OSError):
        return None
    return t.astimezone(LAB_TZ).strftime("%Y-%m-%d")


def _bundle_name(policy) -> str | None:
    """The file name of the signed bundle a run's policy_source names (a
    path written on the Orin or on Windows), or None when it names none."""
    if not isinstance(policy, dict) or not policy.get("path"):
        return None
    return re.split(r"[\\/]", str(policy["path"]))[-1] or None


def _sha256(p: Path) -> str:
    h = hashlib.sha256()
    with Path(p).open("rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def _mtime_date(p: Path) -> str:
    return datetime.fromtimestamp(Path(p).stat().st_mtime).strftime("%Y-%m-%d")


def _src(p: Path) -> dict:
    return {"file": _rel(p), "sha256": _sha256(p), "file_date": _mtime_date(p)}


def missing(backend: str, metric: str, path: Path, what: str) -> dict:
    """A record that says a source is absent here, rather than a zero."""
    return {"backend": backend, "metric": metric, "status": "source missing on this host",
            "source": {"file": _rel(path)}, "what": what}


def latency_stats(seconds: list[float]) -> dict:
    """Latency list in seconds -> ms percentiles (nearest rank) and Hz at p50."""
    if not seconds:
        return {"n": 0}
    ms = sorted(s * 1e3 for s in seconds)

    def rank(q):
        return ms[min(len(ms) - 1, max(0, int(math.ceil(q * len(ms))) - 1))]

    p50 = statistics.median(ms)
    return {"n": len(ms), "p50_ms": p50, "p95_ms": rank(0.95), "max_ms": ms[-1],
            "mean_ms": statistics.fmean(ms), "hz_at_p50": (1000.0 / p50) if p50 > 0 else None,
            "meets_contract_p95": rank(0.95) <= CONTRACT_MS}


def weights_status(backend: str, model_root: Path, repo_root: Path = ROOT) -> list[dict]:
    """Where each weight file or directory should be, whether it is there, and
    its size. Never hashes a 15 GB directory: presence and bytes only.
    `repo_root` is the checkout whose models/ is looked in: the policy's
    weight files are not in git, so a worktree names the lab checkout."""
    spec = BACKENDS[backend]
    out = []
    for rel in spec.get("weights_repo") or []:
        p = Path(repo_root) / rel
        out.append({"path": rel, "where": "repository", "present": p.exists(),
                    "bytes": p.stat().st_size if p.is_file() else None})
    for rel in spec.get("weights_root") or []:
        p = Path(model_root) / rel
        size = None
        if p.is_dir():
            size = sum(f.stat().st_size for f in p.rglob("*") if f.is_file())
        elif p.is_file():
            size = p.stat().st_size
        out.append({"path": f"<model-root>/{rel}", "where": str(Path(model_root).as_posix()),
                    "present": p.exists(), "bytes": size})
    return out


# --------------------------------------------------------------------------- #
# Extractors: what is already on disk
# --------------------------------------------------------------------------- #
_INFER = re.compile(r"infer\s+([0-9.]+)\s*s")
_LOADED = re.compile(r"MODEL_LOADED(?:_4BIT)?\s+(?:([0-9.]+)\s*s\s+)?VRAM\s+([0-9.]+)\s*GB")


def extract_openvla_probes(demo_out: Path) -> list[dict]:
    """demo/out/_vla4bit.txt (4-bit) and _vlaload.txt (bf16): two standalone
    load probes, two inferences each, no simulator."""
    recs = []
    for name, quant in (("_vla4bit.txt", "4-bit NF4"), ("_vlaload.txt", "bf16 (unquantised)")):
        p = Path(demo_out) / name
        if not p.is_file():
            recs.append(missing("openvla_7b_4bit", "latency", p,
                                f"OpenVLA-7B {quant} load probe"))
            continue
        text = p.read_text(encoding="utf-8", errors="replace")
        lat = [float(v) for v in _INFER.findall(text)]
        m = _LOADED.search(text)
        recs.append({
            "backend": "openvla_7b_4bit", "metric": "latency", "protocol": "not R1",
            "quantisation": quant, "values_s": lat,
            "load_s": float(m.group(1)) if m and m.group(1) else None,
            "vram_gb": float(m.group(2)) if m else None,
            "warm_latency_s": lat[-1] if len(lat) > 1 else None,
            "hz_warm": (1.0 / lat[-1]) if len(lat) > 1 and lat[-1] > 0 else None,
            "n": len(lat),
            "conditions": "standalone load probe on the lab desktop, no simulator; the first "
                          "call includes warm-up; n is far below R1's 50",
            "source": _src(p)})
    return recs


def extract_aerialvla_flight_rates(results: Path) -> list[dict]:
    """experiments/out/results.jsonl: the eight simultaneity flights. Their
    inference ran inside the flight process while Project AirSim rendered on
    the same GPU. The flights are fixed-length (the script ends on timeout or
    error only), so they carry no success figure."""
    if not Path(results).is_file():
        return [missing("aerialvla_lora", "inference_hz_in_flight", results,
                        "AerialVLA simultaneity flights")]
    by_adapter: dict[str, list[dict]] = {}
    for line in Path(results).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        by_adapter.setdefault(str(r.get("adapter")), []).append(r)
    recs = []
    for adapter, rows in sorted(by_adapter.items()):
        backend = "aerialvla_ft_run2" if "run2" in adapter else (
            "aerialvla_lora" if "aero_vla" in adapter else "unknown")
        hz = [r["inference_hz"] for r in rows if r.get("inference_hz") is not None]
        recs.append({
            "backend": backend, "metric": "inference_hz_in_flight", "protocol": "not R1",
            "adapter": adapter, "flights": len(rows),
            "hz_min": min(hz) if hz else None, "hz_max": max(hz) if hz else None,
            "inferences": sum(int(r.get("n_inference") or 0) for r in rows),
            "terminated_by": sorted({str(r.get("terminated_by")) for r in rows}),
            "conditions": "in flight, Project AirSim rendering on the same GPU, inference in a "
                          "background thread of the flight process; fixed-length flights "
                          "(no reached-target ending), so no success figure",
            "source": _src(results)})
    return recs


def extract_stated_rates() -> list[dict]:
    """Rates that are stated in a text but have no per-inference file behind
    them. Recorded as stated, so the table never shows them as measured."""
    recs = []
    log = ROOT / "training" / "research_log.md"
    if log.is_file():
        text = log.read_text(encoding="utf-8", errors="replace")
        for i, line in enumerate(text.splitlines(), 1):
            if "0.8 Hz inference" in line:
                recs.append({"backend": "aerialvla_ft_run2", "metric": "inference_hz",
                             "protocol": "stated only", "value_hz": 0.8,
                             "conditions": "fine-tuning campaign evaluation, AirSimNH; no "
                                           "per-inference timing file was kept",
                             "source": {"file": f"{_rel(log)}:{i}"}})
                break
    srv = ROOT / "demo" / "vla_server.py"
    if srv.is_file():
        text = srv.read_text(encoding="utf-8", errors="replace")
        if "separate process" in text and "225 ms/token" in text:
            recs.append({"backend": "aerialvla_lora", "metric": "seconds_per_action",
                         "protocol": "stated only",
                         "value": "about 2.5 s per action in a separate process vs about 12 s "
                                  "inside the 10 Hz flight process",
                         "conditions": "one GPU, a flight airborne and two 7 B models resident; "
                                       "measured while writing demo/vla_server.py, no file kept",
                         "source": {"file": _rel(srv)}})
    return recs


def extract_aerialvla_evals(training: Path) -> list[dict]:
    """training/eval_*.json: closed-loop episodes in AirSimNH (classic AirSim
    1.8), 5 random targets each, with the assist recorded per run."""
    recs = []
    files = sorted(Path(training).glob("eval_*.json"))
    if not files:
        return [missing("aerialvla_ft_run2", "closed_loop", Path(training) / "eval_*.json",
                        "AerialVLA closed-loop evaluations")]
    for p in files:
        d = json.loads(p.read_text(encoding="utf-8"))
        res = d.get("results") or []
        adapter = str(d.get("adapter") or "")
        if "aero_vla" in adapter:
            backend = "aerialvla_lora"
        elif "run2/epoch1" in adapter:
            backend = "aerialvla_ft_run2"
        else:
            backend = "aerialvla_ft_other"
        blends = sorted({r.get("params", {}).get("goal_blend") for r in res})
        yaws = sorted({r.get("params", {}).get("yaw_gain") for r in res})
        reached = sum(1 for r in res if r.get("reached"))
        assisted = any((b or 0) > 0 for b in blends)
        recs.append({
            "backend": backend, "metric": "closed_loop", "protocol": "not C1",
            "adapter": adapter, "episodes": len(res), "reached": reached,
            "goal_blend": blends, "yaw_gain": yaws, "assisted": assisted,
            "mean_efficiency": d.get("mean_efficiency"),
            "mean_interventions": d.get("mean_interventions"),
            "nfz_total_s": d.get("nfz_total_s"),
            "conditions": "AirSimNH (classic AirSim 1.8), 5 random targets, Shield on, "
                          "training/eval_aerialvla.py; "
                          + ("ASSISTED: goal_blend > 0 mixes a mission-direction term into "
                             "the action" if assisted else "pure VLA (goal_blend 0)"),
            "source": _src(p)})
    return recs


def extract_bc_offline(models: Path) -> list[dict]:
    p = Path(models) / "vla_policy_v2.metrics.json"
    if not p.is_file():
        return [missing("bc_v3", "closed_loop", p, "BC policy offline evaluation")]
    d = json.loads(p.read_text(encoding="utf-8"))
    fin = d.get("final_with_altitude_aware_decoder") or {}
    return [{
        "backend": "bc_v3", "metric": "closed_loop", "protocol": "not C1",
        "episodes": fin.get("eval_episodes"), "reach_rate_unshielded": fin.get("reach_rate"),
        "reach_rate_shielded": fin.get("shielded_reach_rate"),
        "nfz_entry_rate_unshielded": fin.get("nfz_entry_rate"),
        "intervention_rate": fin.get("intervention_rate"),
        "conditions": "offline kinematic rollouts of held-out scenarios "
                      "(training/evaluate_policy.py), not a simulator and not ArduPilot",
        "source": _src(p)}]


def extract_stub_dev_rail(demo_out: Path) -> list[dict]:
    """The stored dev-rail runs: ArduPilot SITL + MAVROS 2, flown by the stub."""
    recs = []
    for tag in ("ros2_shield_on", "ros2_shield_on_dynamic", "ros2_ped_on"):
        d = Path(demo_out) / tag
        mp, kp = d / "metrics.json", d / "kpi.json"
        if not mp.is_file():
            recs.append(missing("stub", "closed_loop", mp, f"dev-rail run {tag}"))
            continue
        m = json.loads(mp.read_text(encoding="utf-8"))
        k = json.loads(kp.read_text(encoding="utf-8")) if kp.is_file() else {}
        recs.append({
            "backend": "stub", "metric": "closed_loop", "protocol": "not C1", "run": tag,
            "reached": bool(m.get("reached")), "ticks": m.get("ticks"),
            "interventions": m.get("interventions"),
            "p0_violation_escape_rate": k.get("p0_violation_escape_rate"),
            "conditions": "dev topology: ArduPilot SITL + MAVROS 2 on one desktop, "
                          "policies of the sim demo, one flight",
            "source": _src(mp)})
    return recs


def extract_openvla_closed_loop_statement() -> list[dict]:
    doc = ROOT / "docs" / "MIDTERM-REPORT-Aug2026.md"
    if not doc.is_file():
        return []
    for i, line in enumerate(doc.read_text(encoding="utf-8").splitlines(), 1):
        if line.startswith("| OpenVLA-7B, 4-bit") and "553 ticks" in line:
            return [{"backend": "openvla_7b_4bit", "metric": "closed_loop",
                     "protocol": "stated only",
                     "value": "553 ticks of a 10 Hz control loop, 10 Shield interventions, "
                              "NFZ 0.0 s",
                     "conditions": "the 10 Hz is the control loop, which held the last action "
                                   "between inferences; base OpenVLA was not given a target, "
                                   "so there is no success criterion; no run directory of "
                                   "this flight is on this machine",
                     "source": {"file": f"{_rel(doc)}:{i}"}}]
    return []


def eval_summary(path: Path) -> dict | None:
    """One closed-loop evaluation file as the numbers a correction quotes:
    reached n/N, the assist (goal_blend), the yaw gain and the efficiency."""
    p = Path(path)
    if not p.is_file():
        return None
    d = json.loads(p.read_text(encoding="utf-8"))
    res = d.get("results") or []
    blends = sorted({(r.get("params") or {}).get("goal_blend") for r in res},
                    key=lambda v: (v is None, v))
    return {"file": _rel(p), "adapter": d.get("adapter"),
            "reached": sum(1 for r in res if r.get("reached")), "episodes": len(res),
            "goal_blend": blends,
            "yaw_gain": sorted({(r.get("params") or {}).get("yaw_gain") for r in res},
                               key=lambda v: (v is None, v)),
            "assisted": any((b or 0) > 0 for b in blends),
            "mean_efficiency": d.get("mean_efficiency")}


def check_claims(checks=None, root: Path = ROOT) -> list[dict]:
    """Each statement in CLAIM_CHECKS that its file contradicts, today, with
    the evidence a correction has to carry (ASSIST_EVIDENCE, read now)."""
    out = []
    for c in (CLAIM_CHECKS if checks is None else checks):
        doc = Path(root) / c["doc"]
        src = Path(root) / c["file"]
        if not (doc.is_file() and src.is_file()):
            continue
        hit = None
        for i, line in enumerate(doc.read_text(encoding="utf-8").splitlines(), 1):
            low = line.lower()
            if all(s in line for s in c["must_contain"]) and \
                    not any(w.lower() in low for w in c.get("must_not_contain") or []):
                hit = (i, line.strip())
                break
        if hit is None:
            continue
        data = json.loads(src.read_text(encoding="utf-8"))
        blends = sorted({(r.get("params") or {}).get("goal_blend")
                         for r in data.get("results") or []},
                        key=lambda v: (v is None, v))
        if any((b or 0) > 0 for b in blends):
            evidence = [s for s in (eval_summary(Path(root) / f) for f in ASSIST_EVIDENCE) if s]
            out.append({"doc": f"{c['doc']}:{hit[0]}", "claim": hit[1],
                        "contradicted_by": c["file"], "field": c["field"],
                        "value": blends, "note": c.get("note"), "why": c["why"],
                        "evidence": evidence})
    return out


# --------------------------------------------------------------------------- #
# Which row a served model belongs to
# --------------------------------------------------------------------------- #
HAND_WRITTEN = "hand_written_follow"      # C2's null; not a table row
STAND_IN = "scripted_stand_in"            # a scripted server, never a VLA


def backend_of_identity(identity: dict | None) -> tuple[str, str | None]:
    """(table row, launch family) of a protocol-v2 server, by the identity
    the server itself reported: "<family>/<model>@<digest>" for a foreign
    model (demo/vla_server_foreign.py), "aerialvla-lora/aero_vla@..." and
    its precision for AerialVLA (demo/vla_server_v2.py). A model that is not
    a row keeps a name of its own, so its records are kept apart
    (other_records) and never counted under a row."""
    ident = identity if isinstance(identity, dict) else {}
    sid = str(ident.get("id") or "")
    head, _, rest = sid.partition("/")
    name = rest.split("@")[0]
    if head == "aiotvla":
        return (f"aiotvla_{name}" if name else "aiotvla"), "aiotvla"
    if head == "dronecats":
        return ("dronecats_2b" if name == "DroneCATS-Cosmos3-Edge-2B"
                else f"dronecats:{name}"), "dronecats"
    if head == "racevla":
        return ("racevla_7b" if name == "drone6_all" else f"racevla:{name}"), "racevla"
    if head == "aerialvla-lora" and name == "aero_vla":
        prec = ident.get("precision")
        return ({"bf16": "aerialvla_bf16", "nf4": "aerialvla_lora"}.get(
            prec, f"aerialvla_lora:{prec}"), "aerialvla")
    if head.startswith("aerialvla-ft"):
        if name == "run2/epoch1" and ident.get("precision") == "nf4":
            return "aerialvla_ft_run2", "aerialvla"
        return f"aerialvla_ft:{name}", "aerialvla"
    if head.startswith("fake"):
        return STAND_IN, None
    return (f"unidentified:{sid}" if sid else "unidentified"), None


def backend_of_pilot(pilot: dict | None) -> tuple[str, str | None]:
    """(row, family) of a run's pilot (metrics.json `pilot`): its server's
    identity when a VLA flew, the hand-written controller otherwise."""
    pilot = pilot if isinstance(pilot, dict) else {}
    if pilot.get("kind") == "vla":
        return backend_of_identity(pilot.get("server_identity"))
    if pilot.get("kind") == "stub":
        return STAND_IN, None
    return HAND_WRITTEN, None


# --------------------------------------------------------------------------- #
# Benches made on an Orin that are not `measure`'s (deploy/evidence/hil)
# --------------------------------------------------------------------------- #
AERIAL_BENCH_SCHEMA = "vlaguard.aerialvla_serving_bench/1"
FOREIGN_BENCH_SCHEMA = "vlaguard.foreign_vla_bench/1"
# The bench files carry no power mode. What deploy/evidence/README.md states
# for the session of each file committed before protocol /2, by file name;
# any other file is said to record none, and nothing more (a later session's
# mode is the operator's to note, and is not guessed here).
_README_POWER_MODE = {
    "aerialvla_serving_bench_agx.json": "deploy/evidence/README.md states that session's: "
                                        "MAXN setting, clocks not locked",
    "aerialvla_serving_bench_agx_merged.json": "deploy/evidence/README.md states that "
                                               "session's: MAXN setting, clocks not locked",
    "vla_bf16_check.json": "deploy/evidence/README.md states that session's: MAXN setting, "
                           "clocks not locked",
    "aiotvla_v10b_bench_agx.json": "deploy/evidence/README.md states none for that session "
                                   "('Power mode not recorded')",
}
_RUNBOOK_POWER = "docs/RUNBOOK-orin-hil.md step 4 times in a budgeted mode with the clocks locked"


def _power_mode_reason(p: Path, d: dict) -> str:
    """What a bench file says of the Orin's power mode: a mode it records
    (a `power_mode` field, or the --note R1 results carry), else that it
    records none, with what the evidence README states for the committed
    sessions it names."""
    said = d.get("power_mode") or d.get("note")
    if said:
        return f"power mode as the file records it: {said}"
    known = _README_POWER_MODE.get(Path(p).name)
    return ("the file records no power mode" + (f" ({known})" if known else "")
            + f"; {_RUNBOOK_POWER}")


def _orin_by_host_block(host) -> tuple[bool, str]:
    """(is an Orin, on what basis) for a host block as guardrail.manifest.
    collect_host_evidence writes it into a server's facts or hello."""
    from guardrail.manifest import orin_missing
    if not isinstance(host, dict) or not host:
        return False, "the file carries no host block"
    missing = orin_missing(host)
    if missing:
        return False, "; ".join(missing)
    return True, (f"guardrail.manifest's Orin rule on the file's own host block "
                  f"({host.get('device_model')}, L4T {host.get('l4t_release')})")


def _stats_of(d: dict | None) -> dict:
    d = d if isinstance(d, dict) else {}
    return {k: d.get(k) for k in ("n", "p50_ms", "p95_ms", "max_ms", "mean_ms", "hz_at_p50")}


def _is_r1_frames(path) -> bool:
    return Path(str(path or "")).name == FRAMES_DIR.name


def _aerial_bench_record(p: Path, d: dict) -> dict:
    ident = d.get("identity") or {}
    backend, _ = backend_of_identity({**ident, "precision": (d.get("config") or {}).get(
        "precision", ident.get("precision"))})
    host = (d.get("facts") or {}).get("host")
    is_orin, basis = _orin_by_host_block(host)
    own = not _is_r1_frames(d.get("frames"))
    why = ["timed through the server's protocol core (demo/vla_server_v2.py bench), not "
           "by `measure`", _power_mode_reason(p, d)]
    if own:
        why.insert(0, f"own inputs: the frames of '{Path(str(d.get('frames'))).name}', "
                      f"not the R1 frames")
    if (d.get("warmup"), d.get("timed")) != (PROTOCOL["R1_rate"]["warmup"],
                                             PROTOCOL["R1_rate"]["timed"]):
        why.insert(0, f"{d.get('warmup')} warm-up + {d.get('timed')} timed, not R1's 5 + 50")
    return {
        "backend": backend, "metric": "bench", "protocol": "not R1", "tool": "vla_server_v2 bench",
        "host_is_orin": is_orin, "host_basis": basis,
        "created_utc": d.get("created_utc"), "config": _cfg_name(d["config"]),
        "model_id": ident.get("id"), "weights_sha256": ident.get("weights_sha256"),
        "inputs": "the R1 frames" if not own else f"own: {Path(str(d.get('frames'))).name}",
        "own_inputs": own, "warmup": d.get("warmup"), "timed": d.get("timed"),
        "stats": _stats_of(d.get("stats_latency")),
        "parse_fail_timed": d.get("parse_fail_timed"),
        "peak_vram_gb": (d.get("vram") or {}).get("inference_peak_allocated_gb"),
        "why_not_r1": why, "power_mode": d.get("power_mode") or d.get("note"),
        "conditions": "AerialVLA served in process by demo/vla_server_v2.py bench; "
                      + "; ".join(why),
        "source": _src(p)}


def _foreign_bench_record(p: Path, d: dict) -> dict:
    ident = d.get("identity") or {}
    backend, _ = backend_of_identity(ident)
    is_orin, basis = _orin_by_host_block(ident.get("host"))
    frames = Path(str(d.get("frames") or "")).name
    own = not _is_r1_frames(d.get("frames"))
    why = []
    if own:
        why.append(f"own inputs: {d.get('n')} frames of '{frames}' sent as the model's front "
                   f"camera, object '{d.get('object')}' (not the R1 frames)")
    if (d.get("warmup"), d.get("n")) != (PROTOCOL["R1_rate"]["warmup"],
                                         PROTOCOL["R1_rate"]["timed"]):
        why.append(f"{d.get('warmup')} warm-up + {d.get('n')} timed, not R1's 5 + 50")
    why += ["a round trip over TCP on 127.0.0.1 through demo/vla_server_foreign.py "
            "(tools/bench_foreign_vla.py), not `measure`", _power_mode_reason(p, d)]
    return {
        "backend": backend, "metric": "bench", "protocol": "not R1",
        "tool": "bench_foreign_vla",
        "host_is_orin": is_orin, "host_basis": basis,
        "created_utc": d.get("created_utc"),
        "config": f"{ident.get('precision')}/{ident.get('attn')}",
        "model_id": ident.get("id"), "weights_sha256": ident.get("weights_sha256"),
        "inputs": f"own: {frames}" if own else "the R1 frames (front tile)",
        "own_inputs": own, "warmup": d.get("warmup"), "timed": d.get("n"),
        "stats": _stats_of(d.get("roundtrip")),
        "server_latency": _stats_of(d.get("server_latency")),
        "parse_fail_timed": d.get("parse_fail"), "refused": d.get("refused"),
        "why_not_r1": why, "power_mode": d.get("power_mode") or d.get("note"),
        "conditions": "served by demo/vla_server_foreign.py; the headline is the round trip, "
                      "the server's own model time is beside it; " + "; ".join(why),
        "source": _src(p)}


def _bf16_check_record(p: Path, d: dict) -> dict:
    """deploy/orin/vla_bf16_check.py's report: a gate (does OpenVLA-7B answer
    in bf16 on this GPU), a handful of answers, the first one cold."""
    facts = d.get("facts") or {}
    lat = [float(v) for v in facts.get("latency_s") or []]
    rest = lat[1:]
    by_torch = facts.get("device_name")
    why = [f"a gate check, not a rate: {len(lat)} answers on one R1 frame "
           f"({facts.get('frame')}), the first cold, not 5 + 50 on both",
           f"the host is named by torch's device name only ('{by_torch}'): the file "
           f"carries no host block, so guardrail.manifest's Orin rule was not applied",
           _power_mode_reason(p, d)]
    return {
        "backend": "openvla_7b_bf16", "metric": "bench", "protocol": "not R1",
        "tool": "vla_bf16_check",
        "host_is_orin": "orin" in str(by_torch or "").lower(),
        "host_basis": f"torch.cuda.get_device_name(0) = '{by_torch}' in the file (no host "
                      f"block)",
        "created_utc": None, "config": "bf16/eager", "verdict": d.get("verdict"),
        "inputs": f"one R1 frame ({facts.get('frame')}, front half), the flight prompt",
        "own_inputs": False, "warmup": 0, "timed": len(lat),
        "values_s": lat, "first_s": lat[0] if lat else None,
        "rest_min_s": min(rest) if rest else None, "rest_max_s": max(rest) if rest else None,
        "warm_median_s": facts.get("warm_median_s"),
        "load_s": facts.get("load_s"), "peak_vram_gb": facts.get("peak_memory_gb"),
        "param_dtype": facts.get("param_dtype"), "torch": facts.get("torch"),
        "why_not_r1": why, "power_mode": d.get("power_mode") or d.get("note"),
        "conditions": "deploy/orin/vla_bf16_check.py (the loader R1's openvla_7b_bf16 key "
                      "uses); " + "; ".join(why),
        "source": _src(p)}


def extract_orin_benches(evidence: Path) -> list[dict]:
    """The benches under <evidence>/<topology>/ that `measure` did not make:
    demo/vla_server_v2.py bench, tools/bench_foreign_vla.py and
    deploy/orin/vla_bf16_check.py. Each is a record "not R1" that says why;
    whether it was made on an Orin is read from the file, not from the folder
    it lies in."""
    recs = []
    for p in sorted(Path(evidence).glob("*/*.json")):
        d = _read_json(p)
        if not isinstance(d, dict):
            continue
        if d.get("schema") == AERIAL_BENCH_SCHEMA:
            recs.append(_aerial_bench_record(p, d))
        elif d.get("schema") == FOREIGN_BENCH_SCHEMA:
            recs.append(_foreign_bench_record(p, d))
        elif p.name.startswith("vla_bf16_check") and isinstance(d.get("facts"), dict):
            recs.append(_bf16_check_record(p, d))
    return recs


def bench_cell(rec: dict) -> str:
    """One bench as the Orin rate cell shows it: 'not R1', the reason, the
    numbers, the file."""
    src = rec["source"]["file"]
    if rec["tool"] == "vla_bf16_check":
        if not rec.get("rest_min_s"):
            return f"not R1 - gate check with no timed answer [{src}]"
        wm = rec.get("warm_median_s")
        warm = f", warm median {wm:.3f} s ({1 / wm:.3g} Hz)" if wm else ""
        return (f"not R1 - gate check, {rec['timed']} answers on one R1 frame (the first, "
                f"cold, {rec['first_s']:.2f} s): the others {rec['rest_min_s']:.3f}-"
                f"{rec['rest_max_s']:.3f} s{warm} [{src}]")
    s = rec["stats"]
    if not s.get("n"):
        return f"not R1 - bench with no timed sample [{src}]"
    head = (f"p50 {s['p50_ms']:.4g} ms ({s['hz_at_p50']:.3g} Hz), p95 {s['p95_ms']:.4g} ms, "
            f"n={s['n']}")
    if rec["tool"] == "bench_foreign_vla":
        sv = rec.get("server_latency") or {}
        model = (f"; the model alone p50 {sv['p50_ms']:.4g} ms, p95 {sv['p95_ms']:.4g} ms"
                 if sv.get("n") else "")
        what = (f"own inputs ({rec['inputs'][5:]}), " if rec["own_inputs"] else "") + \
            "round trip through its server"
        return f"not R1 - {what}: {head}{model} [{src}]"
    what = f"{rec['config']} through demo/vla_server_v2.py bench, " + (
        f"{rec['inputs']}, " if rec["own_inputs"] else "the R1 frames, ") + \
        f"{rec['warmup']} + {rec['timed']}, " + (
            f"power mode {rec['power_mode']}" if rec.get("power_mode")
            else "power mode not in the file")
    return f"not R1 - {what}: {head} [{src}]"


# --------------------------------------------------------------------------- #
# C2: the Project AirSim follow on hil (run folders, else the committed summaries)
# --------------------------------------------------------------------------- #
def paraphrase_flights(docs_data: Path) -> dict[str, str]:
    """{run tag: the instruction it flew} for the flights whose instruction
    was a paraphrase (docs/data/pas_hil_paraphrase_*/flights.json): the run's
    own metrics.json keeps the parsed mission, not the words it was given.
    A flight given the source instruction itself - the row "source", and a
    repeat of it under another id, whose text is the source's word for word -
    is not a paraphrase."""
    out = {}
    for p in sorted(Path(docs_data).glob("pas_hil_paraphrase_*/flights.json")):
        d = _read_json(p)
        rows = [r for r in (d.get("runs") if isinstance(d, dict) else None) or []
                if isinstance(r, dict)]
        source = {str(r.get("text")) for r in rows if r.get("paraphrase_id") == "source"}
        for r in rows:
            if r.get("paraphrase_id") in (None, "source") or str(r.get("text")) in source:
                continue
            out[str(r.get("run"))] = str(r.get("text"))
    return out


def c2_exclusions(f: dict, paraphrases: dict[str, str] | None = None) -> list[str]:
    """Why a hil follow flight is NOT a C2 flight (PROTOCOL C2 `counts_as_c2`).
    Empty = it is one. A flight that lost the car is still C2."""
    why = []
    if f.get("summary_only"):
        return ["only its committed summary is on this host (no run folder): the summary "
                "holds no within-30 m share and not the options C2 fixes"]
    if f.get("backend") == HAND_WRITTEN:
        why.append("the hand-written follow: C2's null, not a backend")
    if f.get("topology") != "hil":
        why.append(f"topology {f.get('topology')!r}, not hil")
    if f.get("kpi_grade") is not True:
        why.append("not KPI-grade by its kpi.json")
    if f.get("stress"):
        why.append(f"perception stressor '{f['stress']}'")
    if f.get("hot_apply"):
        why.append("a policy was hot-applied in flight")
    if (paraphrases or {}).get(f.get("tag")):
        why.append(f"paraphrased instruction ({paraphrases[f['tag']]!r})")
    if f.get("tag") in C2_NOT_ALONE:
        why.append(f"not flown alone: {C2_NOT_ALONE[f['tag']]}")
    if not f.get("start_gate_reached") or not f.get("ticks"):
        why.append("the start gate never passed: no mission")
    if C2_SHAKEDOWN_MARK in str(f.get("tag") or ""):
        why.append(f"a shakedown by its tag ('{C2_SHAKEDOWN_MARK}')")
    ran, given = f.get("mission_s"), f.get("max_s_given")
    if given is not None:
        # The limit the mission monitor records it was given decides.
        if abs(float(given) - C2_MAX_S) > 1e-6:
            why.append(f"given a time limit of {float(given):g} s, not C2's {C2_MAX_S} s")
    elif f.get("ticks"):
        # No record of the limit: told only by how the flight ended.
        if f.get("end_reason") == "max_s reached" and ran is not None \
                and ran < C2_MAX_S - C2_MAX_S_SLACK_S:
            why.append(f"ran to a time limit of about {ran:.0f} s, under C2's {C2_MAX_S} s")
        elif f.get("end_reason") not in ("max_s reached", C2_LOST_THE_CAR) and (
                ran is None or ran < C2_MAX_S - C2_MAX_S_SLACK_S):
            why.append(f"ended after {_show(ran, '.0f')} s ({f.get('end_reason')}) without "
                       f"losing the car, and the run records no time limit: it cannot be "
                       f"told from a shakedown")
    if f.get("negative_query") != C2_NEGATIVE_QUERY:
        why.append(f"negative query {f.get('negative_query')!r}, not C2's "
                   f"{C2_NEGATIVE_QUERY!r}")
    bundle = f.get("bundle")
    if not bundle:
        why.append("the run records no policy bundle (policy_source)")
    elif not C2_BUNDLE.match(bundle):
        why.append(f"policy bundle {bundle}, not the node's default "
                   f"follow-car-citylife-v<version>")
    if f.get("backend") != HAND_WRITTEN:
        if not (f.get("identity") and f.get("target_estimator")):
            why.append("flown without --identity and --target-estimator")
        want = C2_FEED.get(f.get("family"), "tick")
        capped = f.get("vla_feed") == "stream" and (f.get("vla_feed_max_hz") or 0) > 0
        if f.get("vla_feed") != want or capped:
            why.append(f"feed {f.get('vla_feed')}"
                       + (f" capped at {f['vla_feed_max_hz']:g} Hz" if capped else "")
                       + f", not the family's C2 feed ({want}"
                       + (", no cap)" if want == "stream" else ")"))
        cap_s = f.get("front_capture_interval_s")
        if want == "stream" and (cap_s is None or not math.isclose(
                float(cap_s), C2_STREAM_CAPTURE_INTERVAL_S, rel_tol=0, abs_tol=1e-9)):
            why.append(f"FrontCamera capture interval {_show(cap_s, 'g')} s, not the "
                       f"{C2_STREAM_CAPTURE_INTERVAL_S:g} s the stream feed needs")
    return why


def hil_follow_flight(run_dir: Path, paraphrases: dict[str, str] | None = None) -> dict | None:
    """One follow flown on the perception rail (Project AirSim + ArduPilot),
    read from its run folder: what C2 reports and what decides whether the
    flight is C2. None when the folder is not such a run."""
    m = _read_json(Path(run_dir) / "metrics.json")
    if not isinstance(m, dict) or not str(m.get("rail") or "").startswith(
            "projectairsim-ardupilot"):
        return None
    params = m.get("params") or {}
    task = str((m.get("mission") or {}).get("task") or "")
    if task.split(" ")[0] != "follow" and params.get("mission") != "follow":
        return None
    kpi = _read_json(Path(run_dir) / "kpi.json") or {}
    pilot = m.get("pilot") or {}
    backend, family = backend_of_pilot(pilot)
    summary = pilot.get("summary") or {}
    obs = m.get("vla_observation") or {}
    ticks = int(m.get("ticks") or 0)
    parsed = sum((summary.get("answers_by_kind") or {}).values()) - int(
        summary.get("parse_fail") or 0) if summary.get("answers_by_kind") else None
    asking_s = _num(obs.get("asking_s"))
    stress = m.get("perception_stress")
    interventions = m.get("interventions")
    t0 = _num(obs.get("t0_wall"))
    monitor = ((m.get("vla") or {}).get("mission") or {}).get("monitor") or {}
    policy = m.get("policy_source") or kpi.get("policy_source") or {}
    f = {
        "backend": backend, "family": family, "metric": "hil_follow",
        "tag": str(m.get("tag") or Path(run_dir).name),
        "model_id": (pilot.get("server_identity") or {}).get("id"),
        "topology": m.get("topology"), "kpi_grade": kpi.get("kpi_grade"),
        "code_revision": (kpi.get("manifest") or {}).get("code_revision"),
        # the lab's local date of the mission start (LAB_TZ)
        "date": lab_date(t0) if t0 else None,
        "ticks": ticks, "end_reason": m.get("end_reason"),
        "mission_s": _num(m.get("mission_s")),
        "frac_within_30m": _num(m.get("frac_within_30m")),
        "decision_hz_mission": _num(summary.get("decision_hz")),
        "decision_hz_asked": (parsed / asking_s if parsed is not None and asking_s else None),
        "asking_s": asking_s,
        "interventions": interventions,
        "interventions_per_100_ticks": (100.0 * interventions / ticks
                                        if ticks and interventions is not None else None),
        "p0_escapes": kpi.get("p0_escapes"),
        "p0_violation_escape_rate": kpi.get("p0_violation_escape_rate"),
        # the flight's own KPI verdict (kpi.json), beside the escape count
        "kpi_outcome": kpi.get("outcome"),
        "mission_fail_reasons": kpi.get("mission_fail_reasons"),
        "unsafe_p0_ticks": kpi.get("unsafe_p0_ticks"),
        "p0_ticks_not_measurable": kpi.get("p0_ticks_not_measurable"),
        "p0_ticks_blind": kpi.get("p0_ticks_blind"),
        "detector_hz": _num(m.get("det_hz")),
        "stress": (stress.get("name") if isinstance(stress, dict) else stress)
        or params.get("stress"),
        "hot_apply": m.get("hot_apply") is not None,
        "identity": bool(params.get("identity")),
        "target_estimator": bool(params.get("target_estimator")),
        "vla_feed": params.get("vla_feed") or "tick",
        "vla_feed_max_hz": _num(params.get("vla_feed_max_hz")),
        "negative_query": params.get("negative_query"),
        "front_capture_interval_s": _num(params.get("front_capture_interval_s")),
        "bundle": _bundle_name(policy),
        "max_s_given": _num((monitor.get("config") or {}).get("max_s")),
        "start_gate_reached": (m.get("start_gate") or {}).get("reached"),
        "summary_only": False,
        "source": _src(Path(run_dir) / "metrics.json"),
    }
    f["not_c2"] = c2_exclusions(f, paraphrases)
    f["protocol"] = "C2" if not f["not_c2"] else "not C2"
    return f


def _summary_rows(d) -> tuple[list[dict], str]:
    """(rows, the file's own description) of a committed flights.json, whose
    three shapes are a list, {"flights": [...]} and {"runs": [...]}."""
    if isinstance(d, list):
        return [r for r in d if isinstance(r, dict)], ""
    if isinstance(d, dict):
        rows = d.get("flights") or d.get("runs") or []
        return [r for r in rows if isinstance(r, dict)], str(d.get("what") or "")
    return [], ""


def extract_hil_follow_summaries(docs_data: Path, have: set[str]) -> list[dict]:
    """The flights of docs/data/pas_hil_*/flights.json and pas_aiot_hil_*/
    flights.json whose run folder is not on this host (`have` = the tags
    already read from folders). A summary holds the decision rate, the
    interventions and the P0 escapes, never the within-30 m share, so such a
    flight is listed and is never counted as C2."""
    recs = []
    files = sorted(set(Path(docs_data).glob("pas_hil_*/flights.json"))
                   | set(Path(docs_data).glob("pas_*_hil_*/flights.json")))
    for p in files:
        rows, what = _summary_rows(_read_json(p))
        for r in rows:
            tag = str(r.get("tag") or r.get("run") or "")
            if not tag or tag in have:
                continue
            have.add(tag)
            ticks = int(_num(r.get("ticks")) or 0)
            inter = _num(r.get("interventions"))
            p0 = r.get("p0_escapes", r.get("p0_escape_ticks"))
            grade = r.get("kpi_grade")
            # The row names no model; the file's own description may.
            backend = "aiotvla_v10b" if "v10b" in what else "unidentified (summary only)"
            f = {
                "backend": backend, "family": "aiotvla" if "v10b" in what else None,
                "metric": "hil_follow", "tag": tag, "model_id": None,
                "topology": r.get("topology"),
                "kpi_grade": (grade if isinstance(grade, bool)
                              else str(grade).lower() == "yes" if grade is not None else None),
                "code_revision": r.get("code_revision"), "date": None,
                "ticks": ticks, "end_reason": r.get("end_reason", r.get("ended")),
                "mission_s": None, "frac_within_30m": None,
                "decision_hz_mission": _num(r.get("decision_hz_mission", r.get("vla_hz"))),
                "decision_hz_asked": _num(r.get("decision_hz_while_asking")),
                "asking_s": _num(r.get("asking_s")),
                "interventions": inter,
                "interventions_per_100_ticks": (100.0 * inter / ticks
                                                if ticks and inter is not None else None),
                "p0_escapes": int(_num(p0)) if _num(p0) is not None else None,
                "p0_violation_escape_rate": _num(r.get("p0_violation_escape_rate")),
                "kpi_outcome": r.get("outcome"),
                "mission_fail_reasons": r.get("mission_fail_reasons"),
                "unsafe_p0_ticks": r.get("unsafe_p0_ticks"),
                "p0_ticks_not_measurable": r.get("p0_ticks_not_measurable"),
                "p0_ticks_blind": r.get("p0_ticks_blind"),
                "detector_hz": _num(r.get("detector_hz")),
                "stress": None if r.get("stress") in (None, "none") else r.get("stress"),
                "hot_apply": bool(r.get("hot_apply_events")),
                "identity": None, "target_estimator": None,
                "vla_feed": r.get("vla_feed"), "vla_feed_max_hz": _num(r.get("vla_feed_max_hz")),
                "negative_query": None, "front_capture_interval_s": None, "bundle": None,
                "max_s_given": None,
                "start_gate_reached": None, "summary_only": True,
                "source": _src(p),
            }
            f["not_c2"] = c2_exclusions(f)
            f["protocol"] = "not C2"
            recs.append(f)
    return recs


def extract_hil_follows(run_roots: list[Path], docs_data: Path) -> list[dict]:
    """Every perception-rail hil follow this host can see: the run folders
    under each root (`pas_*`; the first root that holds a tag wins), then
    the committed summaries for the flights with no folder here."""
    paraphrases = paraphrase_flights(docs_data)
    recs, have = [], set()
    for root in run_roots:
        for d in sorted(Path(root).glob("pas_*")):
            if d.name in have or not d.is_dir():
                continue
            f = hil_follow_flight(d, paraphrases)
            if f is not None and "hil" in (f["topology"], _grant_topology(d)):
                have.add(d.name)
                recs.append(f)
    return recs + extract_hil_follow_summaries(docs_data, have)


def _grant_topology(run_dir: Path) -> str | None:
    """The topology the run was offered (a flight whose start gate never
    passed is stamped with its rail's name, and was still flown on the rig)."""
    m = _read_json(Path(run_dir) / "metrics.json") or {}
    return m.get("grant_topology")


def _span(vals: list[float], fmt: str = ".3f") -> str:
    vals = [v for v in vals if v is not None]
    if not vals:
        return "not recorded"
    if len(vals) == 1 or format(min(vals), fmt) == format(max(vals), fmt):
        return format(vals[0], fmt)
    return (f"{format(min(vals), fmt)}-{format(max(vals), fmt)} "
            f"(median {format(statistics.median(vals), fmt)})")


def p0_escapes_text(flights: list[dict]) -> str:
    """The P0 escapes of these flights, summed only over the flights that
    recorded the count: a missing count is said, never added as 0."""
    known = [int(f["p0_escapes"]) for f in flights if f.get("p0_escapes") is not None]
    gap = len(flights) - len(known)
    if not known:
        return "P0 escapes not recorded"
    return f"P0 escapes {sum(known)}" + (f" (not recorded for {gap} of {len(flights)})"
                                         if gap else "")


def kpi_outcome_text(f: dict) -> str:
    """One flight's own KPI verdict: outcome and fail reasons, '-' when its
    kpi.json does not record one."""
    if f.get("kpi_outcome") is None:
        return "-"
    reasons = f.get("mission_fail_reasons") or []
    return str(f["kpi_outcome"]) + (f" ({', '.join(map(str, reasons))})" if reasons else "")


def kpi_verdict_text(flights: list[dict]) -> str:
    """The flights' own KPI verdicts (kpi.json): how many succeeded and how
    many failed, and why; the ticks spent in a P0-unsafe position (position-
    based, so 0 escapes does not make it 0); the ticks the escape count could
    not see. A value a run did not record is said to be missing."""
    outs: dict[str, int] = {}
    reasons: dict[str, int] = {}
    for f in flights:
        o = str(f.get("kpi_outcome")) if f.get("kpi_outcome") is not None else "not recorded"
        outs[o] = outs.get(o, 0) + 1
        for why in f.get("mission_fail_reasons") or []:
            reasons[str(why)] = reasons.get(str(why), 0) + 1
    text = "KPI outcome " + ", ".join(f"{n} {o}" for o, n in sorted(outs.items()))
    if reasons:
        text += " (" + ", ".join(f"{w} {n}" for w, n in sorted(reasons.items())) + ")"
    unsafe = [f.get("unsafe_p0_ticks") for f in flights]
    text += "; ticks in a P0-unsafe position " + (
        ", ".join(_count(u) for u in unsafe))
    blind = [(f.get("p0_ticks_not_measurable"), f.get("p0_ticks_blind")) for f in flights]
    text += "; P0 ticks not measurable / blind " + ", ".join(
        f"{_count(a)}/{_count(b)}" for a, b in blind)
    return text


def c2_cell(flights: list[dict], spec: dict) -> str:
    """The hil C2 cell of one backend: its C2 flights as a range (the lines
    themselves are in the C2 section), how they ended, how many more are
    asked for, and how many hil follows of it are not C2."""
    c2 = [f for f in flights if not f["not_c2"]]
    rest = [f for f in flights if f["not_c2"]]
    if not flights:
        return spec.get("c2") or ("weights not obtained" if spec.get("unavailable")
                                  else "to fly on hil (C2)")
    if c2:
        ends: dict[str, int] = {}
        for f in c2:
            ends[str(f["end_reason"])] = ends.get(str(f["end_reason"]), 0) + 1
        text = (f"C2, {len(c2)} flight{'s' if len(c2) != 1 else ''} ("
                + ", ".join(f"{n} {e}" for e, n in sorted(ends.items())) + "): within 30 m "
                + _span([f["frac_within_30m"] for f in c2])
                + "; decisions/s over the mission "
                + _span([f["decision_hz_mission"] for f in c2], ".3g")
                + "; interventions per 100 ticks "
                + _span([f["interventions_per_100_ticks"] for f in c2], ".3g")
                + "; " + p0_escapes_text(c2) + "; " + kpi_verdict_text(c2))
        if len(c2) < C2_FLIGHTS_PER_BACKEND:
            text += (f"; the protocol asks for {C2_FLIGHTS_PER_BACKEND}: "
                     f"{C2_FLIGHTS_PER_BACKEND - len(c2)} more to fly")
    else:
        text = "to fly on hil (C2): no flight of it on disk is C2"
    if rest:
        text += f"; {len(rest)} more hil follow{'s' if len(rest) != 1 else ''} outside C2"
    return text + " (see the C2 section)"


# --------------------------------------------------------------------------- #
# The follow matrices on Project AirSim's simple_flight (not ArduPilot)
# --------------------------------------------------------------------------- #
_NOT_ARDUPILOT = ("Project AirSim CityLife with simple_flight: not ArduPilot, not hil, never "
                  "KPI-grade. One flight per configuration unless more are listed, and one "
                  "flight can differ widely from the next; flights at another commit or "
                  "under another policy bundle are not repeats of one configuration "
                  "(where a run folder names them, the table shows both)")
_MATRIX_MODEL = {"aerovla": "aerialvla_bf16", "dronecats": "dronecats_2b",
                 "racevla": "racevla_7b", "manual": HAND_WRITTEN, "fixed12": STAND_IN}
_MATRIX_KIND = {"pure": "pure (the VLA flies every step)",
                "hier": "hierarchical (the VLA sets the intent, the controller flies)",
                "manual": "hand-written follow (no VLA)",
                "fixed12": "hierarchical, a scripted 12 m stand-in (no VLA)"}
# The 3-4 November matrix (agx_*_2026110[34], and nfz3b_agx_*: the same
# configuration under another bundle, which each flight's own policy record
# names), and the first AGX Orin follows of 3 November before it
# (agx_follow_01, agx_e2e_aerovla_01, agx_hier_aerovla_01: an earlier commit,
# which each flight's own record names).
_AGX_MATRIX_TAG = re.compile(r"^(nfz3b_)?agx_.+_(2026110[34]|\d\d)$")


def _matrix_records(by: dict[str, list[dict]], matrix: str, conditions: str,
                    source: dict) -> list[dict]:
    return [{"backend": b, "metric": "follow_matrix", "protocol": "not C1, not C2",
             "matrix": matrix, "flights": fl, "conditions": conditions, "source": source}
            for b, fl in by.items()]


def extract_matrix_1028(docs_data: Path, run_roots: list[Path] | tuple = ()) -> list[dict]:
    """docs/data/progress_1028.json: the follow comparison of 27-28 October
    2026 (PC, and the flight client on the Jetson Orin Nano with every VLA
    served from the PC's GPU).

    The file's label is checked where it contradicts its own row: a flight
    labelled hand-written ("manual") that records a server latency asked a
    server, which the hand-written follow never does. Its run folder, where
    one of `run_roots` holds it, says which pilot flew, and the flight is
    listed as that with a note; without the folder it keeps the file's label
    and the note says the label is contradicted."""
    p = Path(docs_data) / "progress_1028.json"
    d = _read_json(p)
    if not isinstance(d, dict) or not d.get("matrix_rows"):
        return [missing("aerialvla_bf16", "follow_matrix", p,
                        "the 27-28 October follow matrix")]
    by: dict[str, list[dict]] = {}
    for r in d["matrix_rows"]:
        model = str(r.get("model"))
        backend = _MATRIX_MODEL.get(model, f"aerialvla_ft:{model}")
        kind = _MATRIX_KIND.get(r.get("kind"), r.get("kind"))
        note = None
        if backend == HAND_WRITTEN and r.get("latency_p50_s") is not None:
            folder = next((Path(root) / str(r.get("tag")) for root in run_roots
                           if (Path(root) / str(r.get("tag")) / "metrics.json").is_file()), None)
            run = simple_flight_follow(folder) if folder else None
            if run and run["backend"] != HAND_WRITTEN:
                backend, kind = run["backend"], run["kind"]
                note = (f"labelled '{model}' in {p.name}; listed by its run folder's own pilot "
                        f"record ({run.get('model_id') or 'no server identity'})")
            else:
                note = (f"labelled '{model}' in {p.name}, yet the row records a server latency, "
                        f"which a hand-written follow does not have; its run folder is not on "
                        f"this host, so the label stands unchecked")
        by.setdefault(backend, []).append({
            "tag": r.get("tag"),
            "platform": {"PC": "PC", "Jetson": "Jetson Orin Nano (client; VLA on the PC)"}.get(
                r.get("platform"), r.get("platform")),
            "kind": kind, "note": note,
            "within_30m": r.get("score"), "latency_p50_s": r.get("latency_p50_s"),
            "p0_escape_rate": r.get("p0_escape_rate"),
            "bundle": None, "code_revision": None})          # the file names neither
    return _matrix_records(
        by, "27-28 October 2026",
        _NOT_ARDUPILOT + ". The file names the models, not the server configurations; "
        "CHANGELOG 2026-11-04 states them (the AerialVLA family bf16/sdpa/merged, DroneCATS "
        "and RaceVLA bf16). Pure flights are scored from the first acquisition, the others "
        "from the start gate", _src(p))


def simple_flight_follow(run_dir: Path) -> dict | None:
    """One follow of the simple_flight comparison, read from its run folder."""
    m = _read_json(Path(run_dir) / "metrics.json")
    if not isinstance(m, dict):
        return None
    kpi = _read_json(Path(run_dir) / "kpi.json") or {}
    pilot = m.get("pilot") or {}
    backend, _ = backend_of_pilot(pilot)
    hier = m.get("vla_hierarchy") if isinstance(m.get("vla_hierarchy"), dict) else None
    if hier is not None:
        kind = _MATRIX_KIND["fixed12" if backend == STAND_IN else "hier"]
        score, lat = m.get("frac_within_30m"), (hier.get("latency_s") or {}).get("p50")
    elif pilot.get("kind") == "vla":
        kind, score = _MATRIX_KIND["pure"], m.get("frac_within_30m_after_acquire")
        lat = ((pilot.get("summary") or {}).get("latency_server_s") or {}).get("median")
    else:
        kind, score, lat = _MATRIX_KIND["manual"], m.get("frac_within_30m"), None
    sid = pilot.get("server_identity") or {}
    return {"backend": backend, "tag": str(m.get("tag") or Path(run_dir).name),
            "platform": "AGX Orin (client and VLA server)", "kind": kind,
            "within_30m": _num(score), "latency_p50_s": _num(lat),
            "p0_escape_rate": kpi.get("p0_violation_escape_rate"),
            "ticks": m.get("ticks"), "end_reason": m.get("end_reason"),
            "model_id": sid.get("id"),
            "config": (f"{sid.get('precision')}/{sid.get('attn')}/"
                       f"{'merged' if sid.get('merged') else 'unmerged'}" if sid else None),
            "topology": (kpi.get("manifest") or {}).get("topology"),
            "code_revision": (kpi.get("manifest") or {}).get("code_revision"),
            "bundle": _bundle_name(m.get("policy_source") or kpi.get("policy_source"))}


def extract_matrix_agx(run_roots: list[Path]) -> list[dict]:
    """The 28 October matrix re-flown on 3-4 November 2026 with the flight
    client and every VLA server on the AGX Orin (run folders
    agx_*_2026110[34] and nfz3b_agx_*; CHANGELOG 2026-11-04), and the first
    AGX follows of 3 November before it (agx_*_01). Every folder found is
    listed, a configuration flown twice twice, each with the commit and the
    policy bundle its own record names: flights under different bundles are
    not run-to-run spread of one configuration."""
    by: dict[str, list[dict]] = {}
    seen: set[str] = set()
    where = None
    for root in run_roots:
        for d in sorted(Path(root).glob("*agx_*")):
            if d.name in seen or not _AGX_MATRIX_TAG.match(d.name):
                continue
            f = simple_flight_follow(d)
            if f is None:
                continue
            seen.add(d.name)
            where = where or _rel(Path(root))
            by.setdefault(f.pop("backend"), []).append(f)
    if not by:
        return [missing("aerialvla_bf16", "follow_matrix",
                        Path(run_roots[0] if run_roots else ROOT / "demo" / "out")
                        / "agx_*_2026110[34]" / "metrics.json",
                        "the 4 November AGX Orin re-flight of the follow matrix")]
    return _matrix_records(
        by, "3-4 November 2026",
        _NOT_ARDUPILOT + ". The simulator ran on the PC; identities, scores, commits and "
        "policy bundles are each run's own metrics.json and kpi.json. The agx_*_01 flights "
        "are the first AGX follows of 3 November, at an earlier commit than the matrix; the "
        "nfz3b_ flights flew another bundle (a no-fly zone on the route), so they are "
        "another scenario, not a repeat. Pure flights are scored from the first "
        "acquisition, the others from the start gate",
        {"file": f"{where}/*agx_*/metrics.json (tags *_2026110[34] and *_01)",
         "runs": len(seen)})


# --------------------------------------------------------------------------- #
# Measurements made with the protocol
# --------------------------------------------------------------------------- #
def load_rate_results(root: Path = EVIDENCE) -> list[dict]:
    """The `measure` results under <root>/<topology>/. Each is marked with
    whether it was made under a protocol whose R1 this one's is (R1_PROTOCOLS):
    a result under any other id is kept and listed, never shown as R1."""
    out = []
    for p in sorted(Path(root).glob("*/vla_rate_*.json")):
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            continue
        if d.get("schema") == RATE_SCHEMA:
            d["_file"] = _rel(p)
            d["_is_r1"] = d.get("protocol") in R1_PROTOCOLS and d.get("test", "R1") == "R1"
            out.append(d)
    return out


def _other_protocol_record(r: dict) -> dict:
    """A rate result made under a protocol whose R1 is not this one's."""
    return {"backend": r.get("backend"), "metric": "rate_other_protocol",
            "status": (f"a rate result under {r.get('protocol')!r} (test {r.get('test')!r}), "
                       f"not R1 of {', '.join(R1_PROTOCOLS)}: not shown as R1"),
            "source": {"file": r.get("_file")}, "what": "a rate result"}


def _protocol_states(n: int):
    """The R1 state inputs: rows of ros2_shield_on from the committed pack."""
    from guardrail.models import State
    from profile_shield_tick import DEFAULT_PACK
    pack = json.loads(Path(DEFAULT_PACK).read_text(encoding="utf-8"))
    flight = next(f for f in pack["flights"] if f["tag"] == "ros2_shield_on")
    rows = flight["rows"]
    return [State(x=r[1], y=r[2], up=r[3], yaw_deg=r[4]) for r in rows[:n]], \
        {"file": _rel(Path(DEFAULT_PACK)), "flight": "ros2_shield_on", "rows": min(n, len(rows))}


def _load_bc_v3(mission, policy):
    from guardrail.vla_bc import BCVLAv3
    return BCVLAv3(mission, policy)


def make_cpu_backend(name: str, bc_loader=None):
    """The backend under R1 on the CPU. `bc_loader(mission, policy)` stands in
    for guardrail.vla_bc.BCVLAv3 in tests."""
    from guardrail import load_policy
    from guardrail.compiler import ConstraintCompiler
    policy = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    mission = ConstraintCompiler(policy).parse_command("fly to the northeast pad at 6 m/s")
    if name == "stub":
        from guardrail.vla_stub import StubVLA
        return StubVLA(mission), {}
    if name == "bc_v3":
        # Hide every GPU before torch is imported: this backend is measured on
        # the CPU, and nothing here may put a model on the card.
        os.environ["CUDA_VISIBLE_DEVICES"] = ""
        b = (bc_loader or _load_bc_v3)(mission, policy)
        devs = sorted({str(p.device) for p in b.model.parameters()})
        if devs != ["cpu"]:
            raise RuntimeError(f"bc_v3 loaded onto {devs}, refusing to time it as a CPU run")
        return b, {"param_devices": devs,
                   "params": sum(p.numel() for p in b.model.parameters())}
    raise ValueError(name)


def measure_cpu(name: str, warmup: int = 5, timed: int = 50, clock=time.perf_counter,
                bc_loader=None) -> dict:
    t0 = clock()
    backend, extra = make_cpu_backend(name, bc_loader=bc_loader)
    load_s = clock() - t0
    states, inputs = _protocol_states(warmup + timed)
    if len(states) < warmup + timed:
        raise RuntimeError(f"only {len(states)} protocol states, {warmup + timed} needed")
    for st in states[:warmup]:
        backend.act(st)
    lat = []
    for st in states[warmup:warmup + timed]:
        a = clock()
        backend.act(st)
        lat.append(clock() - a)
    return {"load_s": load_s, "latency_s": lat, "inputs": inputs,
            "undecodable_outputs": 0, "device": "cpu", **extra}


def _protocol_frames():
    """(front-over-down mosaics, metadata) from the committed frame set."""
    from PIL import Image
    meta = json.loads((FRAMES_DIR / "frames.json").read_text(encoding="utf-8"))
    imgs = [Image.open(FRAMES_DIR / f["file"]).convert("RGB") for f in meta["frames"]]
    return imgs, meta


def openvla_r1_prompt(RV, instruction: str, csp: str = "off") -> tuple[str, dict]:
    """The prompt R1 times, built by demo/real_vla_demo.py itself.

    "off" is that script's default (`--csp off`, the 2026-07-14 prompt byte for
    byte). "on" is `--csp on`: the Constraint Summary Pack of the script's
    default policy and mission (policies/urban_demo_policy.yaml,
    RV.DEFAULT_MISSION) ahead of the question, compiled by the script's own
    compile_flight_csp. The CSP prompt is longer, and a 7 B model's latency
    grows with prompt length, so the arm is part of the record."""
    if csp == "off":
        return RV.build_openvla_prompt(instruction), {"csp": "off"}
    if csp != "on":
        raise ValueError(f"csp {csp!r} is not 'on' or 'off'")
    from guardrail import load_policy
    from guardrail.compiler import ConstraintCompiler
    policy_path = ROOT / "policies" / "urban_demo_policy.yaml"
    policy = load_policy(policy_path)
    mission = ConstraintCompiler(policy).parse_command(RV.DEFAULT_MISSION)
    pack = RV.compile_flight_csp(policy, mission)
    return (RV.build_openvla_prompt(instruction, pack.natural_language_prompt),
            {"csp": "on", "csp_policy": _rel(policy_path), "csp_mission": RV.DEFAULT_MISSION})


class MeasureRefused(Exception):
    """An attempt that could not be made: where it stopped and why. `measure`
    writes it into the result (status "refused") instead of a timing, so the
    table can show the attempt and its cause."""

    def __init__(self, stage: str, cause: str):
        super().__init__(f"{stage}: {cause}")
        self.stage, self.cause = stage, cause


def _package_versions(names=("torch", "transformers", "torchao", "bitsandbytes")) -> dict:
    """Installed versions by distribution name (None = not installed), read
    without importing the package."""
    from importlib.metadata import PackageNotFoundError, version
    out = {}
    for n in names:
        try:
            out[n] = version(n)
        except PackageNotFoundError:
            out[n] = None
    return out


def _bf16_check_module():
    """deploy/orin/vla_bf16_check.py as a module (it is a script, not a
    package): the one bf16 loader lives there."""
    import importlib.util
    path = ROOT / "deploy" / "orin" / "vla_bf16_check.py"
    spec = importlib.util.spec_from_file_location("vlaguard_vla_bf16_check", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _param_facts(model) -> dict | None:
    """Where the model's weights are and in which type, by its first
    parameter (what deploy/orin/vla_bf16_check.py reads); None for an object
    with no parameters (a test's stand-in)."""
    try:
        par = next(iter(model.parameters()))
    except (AttributeError, StopIteration, TypeError):
        return None
    return {"param_dtype": str(par.dtype), "param_device": str(par.device)}


def load_openvla_bf16(model_dir, load=None):
    """(processor, model, torch) for OpenVLA-7B in bfloat16, unquantised, by
    deploy/orin/vla_bf16_check.py's load_bf16 (`load` stands in for it in
    tests). No bitsandbytes is needed, so it loads in the AGX Orin's
    environment. A model that is not bfloat16 on a CUDA device is refused,
    not timed as the bf16 row."""
    proc, model, torch = (load or _bf16_check_module().load_bf16)(str(model_dir))
    facts = _param_facts(model)
    if facts is not None:
        dev = facts["param_device"]
        if not (dev == "cuda" or dev.startswith("cuda:")) \
                or facts["param_dtype"] != "torch.bfloat16":
            raise RuntimeError(f"OpenVLA loaded as {facts['param_dtype']} on {dev}, not "
                               f"torch.bfloat16 on a CUDA device: refusing to time it as bf16")
    return proc, model, torch


def _torchao_int8_weight_only():
    """torchao's int8 weight-only quantisation as one call on a model (every
    nn.Linear's weight to int8, activations left as they are)."""
    from torchao.quantization import quantize_
    try:
        from torchao.quantization import Int8WeightOnlyConfig
        cfg = Int8WeightOnlyConfig()
    except ImportError:                       # torchao < 0.10 names it as a function
        from torchao.quantization import int8_weight_only
        cfg = int8_weight_only()

    def run(model):
        quantize_(model, cfg)
    return run


def load_openvla_int8wo(model_dir, load=None, quantizer=None, find_spec=None):
    """(processor, model, torch) for the int8 weight-only ATTEMPT: the bf16
    load, then torchao's int8 weight-only quantisation in place. What int8
    itself stops at is a MeasureRefused naming the stage - "import" (no
    torchao, or one that does not import), "quantize" - and torchao is looked
    for BEFORE the 15 GB load, so an environment without it refuses at once.
    A bf16 load that fails is NOT an int8 result (a wrong --model-root, a GPU
    another process holds, a model off the GPU): it is a RuntimeError, so the
    run fails with exit 1 and writes no result file.
    `quantizer(model)` and `find_spec` stand in for torchao in tests."""
    if quantizer is None:
        import importlib.util
        if (find_spec or importlib.util.find_spec)("torchao") is None:
            raise MeasureRefused("import", "torchao is not installed in this environment")
        try:
            quantizer = _torchao_int8_weight_only()
        except Exception as exc:                          # noqa: BLE001
            raise MeasureRefused("import", f"{type(exc).__name__}: {exc}") from exc
    try:
        proc, model, torch = load_openvla_bf16(model_dir, load=load)
    except Exception as exc:                              # noqa: BLE001
        raise RuntimeError(f"the bf16 load the int8 attempt starts from failed "
                           f"({type(exc).__name__}: {exc}); that is not an int8 result, so "
                           f"nothing is recorded: check --model-root and that no other "
                           f"process holds the GPU") from exc
    try:
        quantizer(model)
    except Exception as exc:                              # noqa: BLE001
        raise MeasureRefused("quantize", f"{type(exc).__name__}: {exc}") from exc
    return proc, model, torch


_OPENVLA_LOADER_TEXT = {
    "nf4": "demo/real_vla_demo.py OpenVLABackend._load (4-bit NF4, bitsandbytes)",
    "bf16": "deploy/orin/vla_bf16_check.py load_bf16 (bfloat16, unquantised)",
    "int8wo": "deploy/orin/vla_bf16_check.py load_bf16, then torchao int8 weight-only",
}


def measure_openvla(model_root: Path, warmup: int = 5, timed: int = 50,
                    loader=None, frames=None, clock=time.perf_counter,
                    instruction: str = "fly forward and avoid restricted areas",
                    csp: str = "off", precision: str = "nf4") -> dict:
    """R1 for OpenVLA-7B, with demo/real_vla_demo.py's own predict_action call
    (input_ids + pixel_values ONLY - passing the attention mask crashes
    generate, see that file's worker) and the script's own prompt (`csp`, see
    openvla_r1_prompt). The prompt and its token count are recorded: exact
    with the processor's tokenizer, else a labelled estimate
    (real_vla_demo.count_tokens).

    `precision` picks the loader: "nf4" is the flight script's own 4-bit
    loader, "bf16" deploy/orin/vla_bf16_check.py's (no bitsandbytes), and
    "int8wo" the attempt on top of it (load_openvla_int8wo). An int8wo
    inference that raises is a MeasureRefused at stage "inference"."""
    if precision not in _OPENVLA_LOADER_TEXT:
        raise ValueError(f"precision {precision!r} is not one of {tuple(_OPENVLA_LOADER_TEXT)}")
    sys.path.insert(0, str(ROOT / "demo"))
    import real_vla_demo as RV
    RV.MODEL_ID = str(Path(model_root) / "openvla-7b")
    prompt, prompt_arm = openvla_r1_prompt(RV, instruction, csp)
    imgs, meta = frames if frames is not None else _protocol_frames()
    fronts = [im.crop((0, 0, 224, 224)).resize(RV.IMAGE_SIZE) for im in imgs]
    if loader is None:
        loader = {"nf4": lambda: RV.OpenVLABackend._load(None),
                  "bf16": lambda: load_openvla_bf16(RV.MODEL_ID),
                  "int8wo": lambda: load_openvla_int8wo(RV.MODEL_ID)}[precision]
    t0 = clock()
    proc, model, torch = loader()
    load_s = clock() - t0
    tokens = RV.count_tokens(prompt, tokenizer=getattr(proc, "tokenizer", None))
    lat, bad = [], 0
    for k in range(warmup + timed):
        img = fronts[k % len(fronts)]
        a = clock()
        try:
            inputs = proc(prompt, img).to("cuda", dtype=torch.bfloat16)
            with torch.no_grad():
                raw = model.predict_action(input_ids=inputs["input_ids"],
                                           pixel_values=inputs["pixel_values"],
                                           unnorm_key=RV.UNNORM_KEY, do_sample=False)
        except Exception as exc:                          # noqa: BLE001
            if precision != "int8wo":
                raise
            raise MeasureRefused("inference", f"inference {k + 1}: {type(exc).__name__}: "
                                              f"{exc}") from exc
        dt = clock() - a
        try:
            ok = len(list(raw)) >= 7 and all(math.isfinite(float(v)) for v in raw)
        except (TypeError, ValueError):
            ok = False
        if k >= warmup:
            lat.append(dt)
            bad += not ok
    vram = None
    try:
        vram = torch.cuda.max_memory_allocated() / 1e9
    except Exception:                                     # noqa: BLE001
        vram = None
    return {"load_s": load_s, "latency_s": lat, "undecodable_outputs": bad,
            "peak_vram_gb": vram, "prompt": prompt, "prompt_arm": prompt_arm,
            "prompt_tokens": tokens, "precision": precision,
            "loader": _OPENVLA_LOADER_TEXT[precision], "param": _param_facts(model),
            "inputs": {"frames": [f["file"] for f in meta["frames"]], "crop": "front 224x224"}}


def measure_aerialvla(model_root: Path, adapter_rel: str, warmup: int = 5, timed: int = 50,
                      backend_cls=None, frames=None, timeout_s: float = 1800.0,
                      object_phrase: str = "orange barrier") -> dict:
    """R1 for an AerialVLA adapter, through demo/aerialvla_demo.AerialVLABackend
    unchanged: the protocol frames are served by its obs_factory hook and the
    per-inference timing comes from its own inference log (gen_s, cycle_s)."""
    import tempfile
    sys.path.insert(0, str(ROOT / "demo"))
    if backend_cls is None:
        import aerialvla_demo as AV
        AV.BASE_ID = str(Path(model_root) / "openvla-7b")
        backend_cls = AV.AerialVLABackend
    imgs, meta = frames if frames is not None else _protocol_frames()
    poses = [(f["x"], f["y"], f["psi"]) for f in meta["frames"]]
    tiles = [(im.crop((0, 0, 224, 224)), im.crop((0, 224, 224, 448))) for im in imgs]
    counter = {"k": 0}

    def obs_factory():
        def get():
            k = counter["k"]
            counter["k"] += 1
            front, down = tiles[k % len(tiles)]
            return front, down, poses[k % len(poses)]
        return get

    log = Path(tempfile.mkdtemp()) / "inference.jsonl"
    t0 = time.perf_counter()
    b = backend_cls(object_phrase, tuple(meta["target"]), obs_factory=obs_factory,
                    lora_id=str(Path(model_root) / adapter_rel), inference_log=log)
    load_s = time.perf_counter() - t0
    b.start()
    deadline = time.perf_counter() + timeout_s
    want = warmup + timed
    try:
        while time.perf_counter() < deadline:
            n = len(log.read_text(encoding="utf-8").splitlines()) if log.is_file() else 0
            if n >= want:
                break
            time.sleep(0.2)
    finally:
        b.stop()
    recs = [json.loads(x) for x in log.read_text(encoding="utf-8").splitlines()] \
        if log.is_file() else []
    timed_recs = recs[warmup:want]
    # Until 2026-10-21 an output that did not decode to three action bins
    # wrote no log line, so it showed only as a frame served without a line
    # (an upper bound: one frame may also have been in flight when the worker
    # was stopped). Since then the backend reads with the strict parser and
    # logs every inference with parse_ok, so the count is exact when every
    # timed line carries it; a log without the key keeps the upper bound only.
    undecodable = (sum(1 for r in timed_recs if not r["parse_ok"])
                   if timed_recs and all("parse_ok" in r for r in timed_recs) else None)
    served = counter["k"]
    vram = None
    try:
        import torch
        vram = torch.cuda.max_memory_allocated() / 1e9
    except Exception:                                     # noqa: BLE001
        vram = None
    return {"load_s": load_s, "latency_s": [r.get("cycle_s") for r in timed_recs
                                            if r.get("cycle_s") is not None],
            "generate_s": [r.get("gen_s") for r in timed_recs],
            "new_tokens": [r.get("new_tokens") for r in timed_recs],
            "frames_served": served, "log_lines": len(recs),
            "undecodable_outputs": undecodable,
            "undecodable_outputs_upper_bound": max(0, served - len(recs)),
            "peak_vram_gb": vram,
            "inputs": {"frames": [f["file"] for f in meta["frames"]],
                       "object_phrase": object_phrase, "hint": "from the frames' poses "
                       "to the recorded target (as trained)"},
            "complete": len(timed_recs) == timed}


def measure(name: str, topology: str, model_root: Path | None = None,
            allow_gpu: bool = False, warmup: int = 5, timed: int = 50,
            facts: dict | None = None, **kw) -> dict:
    from profile_shield_tick import check_topology_claim, host_facts
    facts = facts if facts is not None else host_facts()
    refused = check_topology_claim(topology, facts)
    if refused:
        raise PermissionError("; ".join(refused))
    if name in CPU_BACKENDS:
        if kw.pop("csp", "off") != "off":
            raise ValueError(f"{name} reads no prompt; --csp applies to the OpenVLA-7B "
                             f"keys only ({', '.join(OPENVLA_PRECISION)})")
        res = measure_cpu(name, warmup, timed, **kw)
    elif name in GPU_BACKENDS:
        if not allow_gpu:
            raise PermissionError(f"{name} loads a 7 B model onto the GPU; pass --allow-gpu "
                                  f"on a host where that is intended")
        if model_root is None:
            raise ValueError("--model-root is required for a GPU backend")
        if name in OPENVLA_PRECISION:
            try:
                res = measure_openvla(model_root, warmup, timed,
                                      precision=OPENVLA_PRECISION[name], **kw)
            except MeasureRefused as exc:
                # The attempt is the result: what stopped it, at which stage,
                # and what is installed here. No timing, never a zero.
                res = {"status": "refused", "latency_s": [], "undecodable_outputs": None,
                       "precision": OPENVLA_PRECISION[name],
                       "loader": _OPENVLA_LOADER_TEXT[OPENVLA_PRECISION[name]],
                       "refusal": {"stage": exc.stage, "cause": exc.cause,
                                   "packages": _package_versions()}}
        else:
            rel = BACKENDS[name]["weights_root"][1]
            if kw.pop("csp", "off") != "off":
                raise ValueError(f"{name} builds its own direction + object prompt; --csp "
                                 f"applies to the OpenVLA-7B keys only "
                                 f"({', '.join(OPENVLA_PRECISION)})")
            res = measure_aerialvla(model_root, rel, warmup, timed, **kw)
    elif name in SERVED_BACKENDS:
        raise ValueError(f"{name} is served over protocol v2 and timed by its server's own "
                         f"bench, not here: {BACKENDS[name]['measured_by']}")
    else:
        raise ValueError(f"{name}: no measurement path ({BACKENDS.get(name, {}).get('unavailable', 'unknown backend')})")
    res.setdefault("status", "measured")
    lat = latency_stats([s for s in res["latency_s"] if s is not None])
    return {"schema": RATE_SCHEMA, "protocol": PROTOCOL_ID, "test": "R1",
            "backend": name, "topology": topology,
            "created_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "host": facts, "warmup": warmup, "timed": timed, "stats": lat,
            "weights": weights_status(name, model_root or Path("/models")),
            **{k: v for k, v in res.items() if k != "latency_s"},
            "latency_s": res["latency_s"]}


# --------------------------------------------------------------------------- #
# AerialVLA serving configurations (demo/vla_server_v2.py bench)
# --------------------------------------------------------------------------- #
def _cfg_key(c: dict) -> tuple:
    return (c["precision"], c["attn"], bool(c["merged"]))


def _cfg_name(c: dict) -> str:
    return f"{c['precision']}/{c['attn']}/{'merged' if c['merged'] else 'unmerged'}"


def _out_key(o: dict) -> tuple:
    return (o["frame"], o["phrase"])


def _verdict(o: dict) -> tuple:
    return (bool(o["parse_ok"]), tuple(o["bins"]) if o["bins"] else None, bool(o["land"]))


def _agreement(outs: list[dict], ref: dict[tuple, tuple]) -> dict:
    """How often a configuration's output equals the reference's on the same
    input: exactly (parse_ok, bins, LAND), and closely (same parse_ok and
    LAND, every bin within 3 - 0.15 m forward, 0.31 m vertical, 0.067 rad)."""
    pairs = [(_verdict(o), ref[_out_key(o)]) for o in outs if _out_key(o) in ref]
    if not pairs:
        return {"n": 0, "exact": None, "close": None}

    def close(a, b):
        if a[0] != b[0] or a[2] != b[2]:
            return False
        if a[1] is None or b[1] is None:
            return a[1] == b[1]
        return all(abs(x - y) <= 3 for x, y in zip(a[1], b[1]))
    return {"n": len(pairs), "exact": round(sum(a == b for a, b in pairs) / len(pairs), 3),
            "close": round(sum(close(a, b) for a, b in pairs) / len(pairs), 3)}


def _process_vram(v: dict, desktop_baseline_gb: float | None) -> tuple[float | None, str]:
    """(GB the model's process holds on the card, how that was read). The
    budget rule reads this, not torch's allocated peak, which leaves out the
    allocator's cache and the CUDA context (review of V1a, P5: 5.4 GB
    allocated against about 7.9 GB held, nf4 merged). Best basis first:
    nvidia-smi after the timed run minus before the load (one process per
    bench); the card after the run minus a desktop baseline declared for the
    whole summary (benches made before the per-run reading existed); torch's
    reserved peak (the cache, no CUDA context: a lower bound); else None."""
    after = v.get("nvidia_smi_used_mib_after_timed")
    before = v.get("nvidia_smi_used_mib_before_load")
    if after is not None and before is not None:
        return (round((after - before) * _MIB_GB, 2),
                "nvidia-smi after the timed run minus before the load")
    if after is not None and desktop_baseline_gb is not None:
        return (round(after * _MIB_GB - desktop_baseline_gb, 2),
                f"nvidia-smi after the timed run minus the desktop's {desktop_baseline_gb:g} GB "
                f"read once before the runs (not per run)")
    if v.get("inference_peak_reserved_gb") is not None:
        return (v["inference_peak_reserved_gb"],
                "torch reserved peak (no CUDA context: a lower bound)")
    return None, "not measured (torch allocated only)"


def summarise_serving(benches: list[dict], vram_budget_gb: float,
                      roundtrip: dict | None = None,
                      gate_p50_ms: float = SERVING_GATE_P50_MS,
                      desktop_baseline_gb: float | None = None) -> dict:
    """The serving table from per-configuration bench results. There is NO
    DEFAULT (see SERVING_CHOICE); three options are named, by rules fixed here
    rather than chosen after the numbers:

    LOW-VRAM OPTION: the lowest p50 among the configurations that (1) meet
    the gate (p50 <= gate_p50_ms), (2) produced no parse failure on the
    timed set, and (3) hold at most `vram_budget_gb` as a process
    (_process_vram) - the room the model may take on the 24 GB card that
    Unreal (CityLife) and OWL-ViT share in flight; the budget is an
    assumption until CityLife's VRAM is measured.
    FIDELITY OPTION: among the other configurations meeting (1) and (2), the
    one whose probe outputs most often equal the reference's (exact, then
    close, then p50). The probe has 48 inputs and no ground truth, so
    agreement says how far a configuration moves the outputs from upstream's
    precision, not which is right.
    FASTEST: the lowest p50 overall."""
    rows = []
    ref_bench = next((b for b in benches if _cfg_key(b["config"]) == _cfg_key(SERVING_REFERENCE)),
                     None)
    ref_probe = {_out_key(o): _verdict(o) for o in (ref_bench or {}).get("probe_outputs", [])}
    ref_timed = {_out_key(o): _verdict(o) for o in (ref_bench or {}).get("timed_outputs", [])}
    digests = sorted({b["identity"].get("weights_sha256") for b in benches})
    for b in benches:
        s, cfg = b["stats_latency"], b["config"]
        load = (b.get("facts") or {}).get("load") or {}
        proc_gb, proc_basis = _process_vram(b["vram"], desktop_baseline_gb)
        rows.append({
            "precision": cfg["precision"], "attn": cfg["attn"], "merged": bool(cfg["merged"]),
            "p50_ms": s.get("p50_ms"), "p95_ms": s.get("p95_ms"), "max_ms": s.get("max_ms"),
            "mean_ms": s.get("mean_ms"), "hz_at_p50": s.get("hz_at_p50"), "n": s.get("n"),
            "generate_p50_ms": b["stats_gen"].get("p50_ms"),
            "server_p50_ms": b["stats_server"].get("p50_ms"),
            "probe_p50_ms": b["stats_probe_latency"].get("p50_ms"),
            "probe_p95_ms": b["stats_probe_latency"].get("p95_ms"),
            "new_tokens": b.get("new_tokens"),
            "parse_fail_timed": b["parse_fail_timed"], "parse_fail_probe": b["parse_fail_probe"],
            "probe_n": len(b.get("probe_outputs") or []), "land_probe": b.get("land_probe"),
            "agreement_probe": _agreement(b.get("probe_outputs") or [], ref_probe),
            "agreement_timed": _agreement(b.get("timed_outputs") or [], ref_timed),
            # torch's allocated peaks (the tensors only), kept under their old
            # names; the column says "allocated by torch" since the review of V1a
            "vram_inference_peak_gb": b["vram"].get("inference_peak_allocated_gb"),
            "vram_load_peak_gb": b["vram"].get("load_peak_allocated_gb"),
            "vram_inference_peak_reserved_gb": b["vram"].get("inference_peak_reserved_gb"),
            "nvidia_smi_used_mib": b["vram"].get("nvidia_smi_used_mib_after_timed"),
            "vram_process_gb": proc_gb, "vram_process_basis": proc_basis,
            "load_s": (round(load["load_total_s"], 1)
                       if load.get("load_total_s") is not None else None),
            "merge_s": round(load["merge_s"], 1) if load.get("merge_s") is not None else None,
            "meets_gate": s.get("p50_ms") is not None and s["p50_ms"] <= gate_p50_ms,
            "bench_created_utc": b.get("created_utc"),
            "probe_outputs": [{"frame": o["frame"], "phrase": o["phrase"],
                               "raw_text": o["raw_text"]} for o in b.get("probe_outputs") or []],
        })
    rows.sort(key=lambda r: (r["precision"], r["attn"], r["merged"]))
    usable = [r for r in rows if r["meets_gate"] and r["parse_fail_timed"] == 0]
    small = [r for r in usable if r["vram_process_gb"] is not None
             and r["vram_process_gb"] <= vram_budget_gb]
    low = min(small, key=lambda r: r["p50_ms"]) if small else None
    near = [r for r in usable if _cfg_key(r) != _cfg_key(SERVING_REFERENCE)
            and r["agreement_probe"]["n"]]
    fid = (min(near, key=lambda r: (-r["agreement_probe"]["exact"],
                                    -r["agreement_probe"]["close"], r["p50_ms"]))
           if near else None)
    fast = min(rows, key=lambda r: r["p50_ms"]) if rows else None
    first = benches[0] if benches else {}
    measured = sorted(b["created_utc"] for b in benches if b.get("created_utc"))

    def cfg(r):
        return {k: r[k] for k in ("precision", "attn", "merged")} if r else None
    return {
        "schema": SERVING_SCHEMA,
        "created_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        # when the benches ran (first, last): created_utc is when they were summarised
        "measured_utc": [measured[0], measured[-1]] if measured else None,
        "made_by": "python tools/vla_backend_table.py serving (from demo/vla_server_v2.py "
                   "bench, one process per configuration)",
        # The machine's name is left out: the other committed evidence files
        # describe the host by its hardware and software only.
        "host": {"libraries": (first.get("facts") or {}).get("libraries"),
                 "host": {k: v for k, v in ((first.get("facts") or {}).get("host") or {}).items()
                          if k != "hostname"} or None},
        "model": {k: (first.get("identity") or {}).get(k) for k in ("id", "base")},
        "weights_sha256": digests[0] if len(digests) == 1 else digests,
        "inputs": {"timed": "deploy/evidence/vla_protocol_frames (the R1 frames: 2 Project "
                            "AirSim mosaics), each tile sent as JPEG q95 through the server's "
                            "protocol core; phrase derived from each frame's pose to the "
                            "recorded target (hint_source goal, as trained); object 'orange "
                            "barrier'; greedy, max_new_tokens 12",
                   "warmup": first.get("warmup"), "timed_n": first.get("timed"),
                   "probe": "the 6 mosaics of the 4 Aug ablation (experiments/out/frames, "
                            "lab desktop) x the 8 phrases = 48 inputs, the same in every "
                            "configuration",
                   "why_these": "real captures, not synthetic frames: demo/out holds no "
                                "front+down capture pair (checked 2026-10-21), so both sets "
                                "are the Project AirSim captures of 4 Aug"},
        "latency_definition": "latency_s as the server returns it: tokenizer + image "
                              "processor + generate + decode, mosaic and prompt ready to "
                              "text decoded; server_p50_ms adds the JPEG decode and the "
                              "mosaic; generate_p50_ms is generate() alone",
        "conditions": "RTX 4090 (24 GB) on the lab desktop, no simulator and no second "
                      "model running; ordinary desktop applications held about 3.4 GB of "
                      "VRAM before the runs (nvidia_smi_used_mib includes them)",
        "gate_p50_ms": gate_p50_ms,
        "reference": dict(SERVING_REFERENCE),
        "vram_budget_gb": vram_budget_gb,
        "desktop_baseline_gb": desktop_baseline_gb,
        "rule": "low-VRAM option = lowest p50 among configurations with p50 <= gate, no timed "
                "parse failure and process VRAM (vram_process_gb) <= vram_budget_gb; fidelity "
                "option = highest exact agreement with the reference among the other "
                "configurations with p50 <= gate and no timed parse failure; neither is a "
                "default",
        "configs": rows,
        "default": None,
        "choice": SERVING_CHOICE,
        "low_vram": _cfg_name(low) if low else None,
        "low_vram_config": cfg(low),
        "fidelity": _cfg_name(fid) if fid else None,
        "fidelity_config": cfg(fid),
        "fastest": _cfg_name(fast) if fast else None,
        "tcp_roundtrip": roundtrip,
    }


def serving_r1_cell(serving: dict | None) -> str | None:
    """The AerialVLA row's desktop rate cell from the serving measurement:
    the low-VRAM option's R1 numbers (the row is the 4-bit backend), with
    its agreement with the reference, and said not to be a default."""
    if not serving or not serving.get("low_vram"):
        return None
    r = next(c for c in serving["configs"] if _cfg_name(c) == serving["low_vram"])
    hz = r.get("hz_at_p50")
    ap = r.get("agreement_probe") or {}
    agree = (f"; agrees with {_cfg_name(serving['reference'])} on {ap['exact']:.0%} of "
             f"{ap['n']} probe inputs" if ap.get("n") else "")
    return (f"R1 via demo/vla_server_v2.py, {serving['low_vram']} (the measured low-VRAM "
            f"option, not a default{agree}): p50 {r['p50_ms']:.4g} ms ({hz:.3g} Hz), p95 "
            f"{r['p95_ms']:.4g} ms, n={r['n']} [{_rel(SERVING_JSON)}]")


def serving_bf16_cell(serving: dict | None) -> str | None:
    """The AerialVLA bf16 row's desktop rate cell: the serving measurement's
    bf16/sdpa configurations (the ones the Orin benches and the flights
    serve), merged and not."""
    if not serving:
        return None
    rows = [c for c in serving.get("configs") or []
            if c["precision"] == "bf16" and c["attn"] == "sdpa" and c.get("p50_ms") is not None]
    if not rows:
        return None
    parts = [f"{_cfg_name(c)} p50 {c['p50_ms']:.4g} ms ({c['hz_at_p50']:.3g} Hz), p95 "
             f"{c['p95_ms']:.4g} ms, n={c['n']}" for c in rows]
    return f"R1 via demo/vla_server_v2.py: {'; '.join(parts)} [{_rel(SERVING_JSON)}]"


def serving_for_table(s: dict | None) -> dict | None:
    """The serving summary as the backend table carries it: everything but
    each configuration's 48 probe texts, which stay in SERVING_JSON (the
    table points there) instead of being copied into a second file."""
    if not s:
        return None
    out = dict(s)
    out["configs"] = [{k: v for k, v in c.items() if k != "probe_outputs"}
                      for c in s.get("configs", [])]
    out["probe_outputs_in"] = _rel(SERVING_JSON)
    return out


def load_serving(path: Path = SERVING_JSON) -> dict | None:
    try:
        d = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return d if d.get("schema") == SERVING_SCHEMA else None


def render_serving(s: dict) -> list[str]:
    when = (s.get("measured_utc") or [s["created_utc"]])[0][:10]
    out = ["## AerialVLA serving configurations (demo/vla_server_v2.py)", "",
           f"Measured {when} on the lab desktop's RTX 4090, one process per "
           f"configuration, through the VLA server's own protocol core (`demo/vla_server_v2.py "
           f"bench`; summarised by `python tools/vla_backend_table.py serving`). "
           f"{s['conditions']}. Timed inputs: {s['inputs']['timed']}; "
           f"{s['inputs']['warmup']} warm-up, {s['inputs']['timed_n']} timed. Probe: "
           f"{s['inputs']['probe']}"
           + (f" ({s['inputs']['why_these']})" if s["inputs"].get("why_these") else "")
           + f". Latency: {s['latency_definition']}. Weights: "
           f"`{s['model']['id']}` on `{s['model']['base']}`, weights_sha256 "
           f"`{str(s['weights_sha256'])[:16]}...` (base shards + adapter). "
           f"Gate: p50 <= {s['gate_p50_ms']:.0f} ms.", "",
           "| Precision | Attention | LoRA | p50 ms | p95 ms | generate p50 ms | server p50 ms "
           "| probe p50 / p95 ms | VRAM allocated by torch, inference / load (GB) "
           "| VRAM of the process (GB) | load s | parse failures, "
           "timed / probe | agrees with bf16/eager/unmerged, exact / close | gate |",
           "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    def n0(v, fmt=".0f"):
        return "n/a" if v is None else format(v, fmt)

    bases = []
    for r in s["configs"]:
        ap = r["agreement_probe"]
        agree = ("reference" if (r["precision"], r["attn"], r["merged"]) ==
                 _cfg_key(s["reference"]) else
                 (f"{ap['exact']:.0%} / {ap['close']:.0%} (n={ap['n']})" if ap["n"] else "n/a"))
        basis = r.get("vram_process_basis")
        if basis and basis not in bases:
            bases.append(basis)
        mark = "*" * (bases.index(basis) + 1) if basis else ""
        out.append(
            f"| {r['precision']} | {r['attn']} | {'merged' if r['merged'] else 'unmerged'} "
            f"| {n0(r['p50_ms'])} | {n0(r['p95_ms'])} | {n0(r['generate_p50_ms'])} "
            f"| {n0(r['server_p50_ms'])} | {n0(r['probe_p50_ms'])} / {n0(r['probe_p95_ms'])} "
            f"| {n0(r['vram_inference_peak_gb'], '.1f')} / {n0(r['vram_load_peak_gb'], '.1f')} "
            f"| {n0(r.get('vram_process_gb'), '.1f')}{mark} "
            f"| {n0(r['load_s'], '.1f')} | {r['parse_fail_timed']} / {r['parse_fail_probe']} "
            f"| {agree} | {'meets' if r['meets_gate'] else 'misses'} |")
    if bases:
        out += [""] + [f"{'*' * (k + 1)} VRAM of the process: {b}." for k, b in enumerate(bases)]

    def opt(name):
        if not name:
            return "none qualifies"
        r = next(c for c in s["configs"] if _cfg_name(c) == name)
        ap = r["agreement_probe"]
        return (f"{name} (p50 {n0(r['p50_ms'])} ms, process VRAM "
                f"{n0(r.get('vram_process_gb'), '.1f')} GB, agrees with the reference on "
                f"{ap['exact']:.0%} of {ap['n']} probe inputs)" if ap["n"] else name)
    out += ["", f"**No serving default.** {s['choice']}", "",
            f"**Low-VRAM option:** {opt(s.get('low_vram'))}. **Fidelity option:** "
            f"{opt(s.get('fidelity'))}. Fastest overall: {s['fastest']}. Rule: {s['rule']} "
            f"(budget {s['vram_budget_gb']:g} GB, an assumption until CityLife's VRAM is "
            f"measured)."]
    if s.get("choice_note"):
        out += ["", s["choice_note"]]
    rt = s.get("tcp_roundtrip")
    if rt:
        out += ["", f"Over TCP (127.0.0.1, `demo/vla_server_v2.py ping`, {rt.get('config')}): "
                    f"round trip p50 {rt['roundtrip']['p50_ms']:.0f} ms, p95 "
                    f"{rt['roundtrip']['p95_ms']:.0f} ms against server latency_s p50 "
                    f"{rt['server_latency']['p50_ms']:.0f} ms (n={rt['roundtrip']['n']}): "
                    f"the transport and JPEG frames add the difference."]
    out += ["", "These are the AerialVLA server's configurations, not the project's "
                "default backend: the grant defers that choice to the full measurement set "
                "(the default line at the top). Every number is standalone; in flight "
                "the renderer shares the GPU (OWL-ViT's forward pass ran 13.8-21.5x slower "
                "in flight than offline: CHANGELOG 0.5.0, 2026-09-10, Known limitations), "
                "so these are the floor, not the flight rate.", ""]
    return out


# --------------------------------------------------------------------------- #
# The table
# --------------------------------------------------------------------------- #
def _fmt_ms(s: dict) -> str:
    if not s.get("n"):
        return "no samples"
    hz = s.get("hz_at_p50")
    rate = f"{hz:.3g} Hz" if hz is not None else "rate n/a"
    return f"p50 {s['p50_ms']:.3g} ms ({rate}), p95 {s['p95_ms']:.3g} ms, n={s['n']}"


def _r1_cell(results: list[dict]) -> str:
    """R1 results for one backend on one kind of host, one per prompt arm, so a
    CSP-prompt rate is never shown as the plain prompt's."""
    def arm(r):
        return (r.get("prompt_arm") or {}).get("csp", "off")
    parts = []
    off = [r for r in results if arm(r) == "off"]
    on = [r for r in results if arm(r) == "on"]
    if off:
        parts.append(f"R1: {_fmt_ms(off[-1]['stats'])} [{off[-1]['_file']}]")
    if on:
        parts.append(f"R1, CSP prompt: {_fmt_ms(on[-1]['stats'])} [{on[-1]['_file']}]")
    return "; ".join(parts)


def _refused_cell(results: list[dict]) -> str:
    """A recorded attempt that produced no timing: when, where it stopped, why."""
    r = results[-1]
    ref = r.get("refusal") or {}
    return (f"attempted {str(r.get('created_utc') or '')[:10]}, refused at "
            f"{ref.get('stage')}: {ref.get('cause')} [{r['_file']}]")


def _is_refused(r: dict) -> bool:
    return r.get("status") == "refused"


# The gate the grant's reference design lists as a candidate, quoted from
# kuanting-vla-uav-guardrail/docs/04-risks-and-fallbacks/fallback-gates.md
# ("Gate G3 candidate - Orin VLA inference latency"; that repository is not
# part of this one).
G3_TEXT = ("If the chosen 7B-class VLA backend cannot produce 4-D actions at 10 Hz on the "
           "Orin in `int8` quantization, a gate could be added: \"if Orin inference latency "
           "> 100 ms p95 by 2026-08-01, swap to a smaller / quantized backend (e.g. BitVLA, "
           "TinyVLA).\" Whether to add this gate depends on Q2 measurement.")
G3_SOURCE = "docs/04-risks-and-fallbacks/fallback-gates.md of the reference design"


def g3_facts(rows: list[dict]) -> dict:
    """What this set holds against the candidate gate G3: every 7 B-class
    row's Orin timing (R1 where there is one, else the labelled bench), the
    int8 attempt's state, and the rows that are not 7 B. Facts only: whether
    the gate is added, and applied, is not this tool's to record."""
    def orin_lines(r):
        out = []
        for res in r["r1_results"]:
            if not res["host"]["jetson"]["is_orin"]:
                continue
            if _is_refused(res):
                out.append({"what": "attempt refused", "p95_ms": None,
                            "stage": res["refusal"]["stage"], "cause": res["refusal"]["cause"],
                            "file": res["_file"]})
            else:
                out.append({"what": "R1", "p95_ms": res["stats"].get("p95_ms"),
                            "n": res["stats"].get("n"), "file": res["_file"]})
        for b in r["records"]:
            if b.get("metric") == "follow_matrix" and not b.get("status"):
                lat = [f["latency_p50_s"] for f in b["flights"]
                       if str(f.get("platform")).startswith("AGX Orin")
                       and f.get("latency_p50_s") is not None]
                if lat:
                    out.append({"what": f"not R1: in flight, {b['matrix']}, the server's "
                                        f"latency p50 per flight", "p95_ms": None,
                                "in_flight_p50_ms": [min(lat) * 1e3, max(lat) * 1e3],
                                "n": len(lat), "file": b["source"]["file"]})
            if b.get("metric") != "bench" or not b.get("host_is_orin"):
                continue
            if b["tool"] == "vla_bf16_check":
                out.append({"what": "not R1: gate check, slowest answer after the first",
                            "p95_ms": None,
                            "slowest_ms": (b["rest_max_s"] * 1e3 if b.get("rest_max_s") else None),
                            "n": b["timed"], "file": b["source"]["file"]})
            else:
                out.append({"what": f"not R1: {b['config']}"
                                    + (", own inputs" if b["own_inputs"] else ""),
                            "p95_ms": b["stats"].get("p95_ms"), "n": b["stats"].get("n"),
                            "file": b["source"]["file"]})
        return out
    seven, smaller = [], []
    for r in rows:
        if r.get("unavailable") or r["backend"] in CPU_BACKENDS:
            continue
        lines = orin_lines(r)
        (seven if r.get("class_7b") else smaller).append(
            {"backend": r["backend"], "label": r["label"], "orin": lines})
    int8 = next(r for r in rows if r["backend"] == "openvla_7b_int8wo")
    int8_orin = [x for x in int8["r1_results"] if x["host"]["jetson"]["is_orin"]]
    return {"gate": G3_TEXT, "source": G3_SOURCE, "threshold_ms": CONTRACT_MS,
            "gate_date": "2026-08-01",
            "seven_b": seven, "smaller": smaller,
            "int8_on_orin": ("not attempted" if not int8_orin else
                             "refused" if _is_refused(int8_orin[-1]) else "measured"),
            "int8_note": int8.get("attempt"),
            "decision": "not recorded by this tool"}


class DefaultRefused(ValueError):
    """Why `table --default` names no default (name_default)."""


def newest_measurement(t: dict) -> str | None:
    """The lab's local date (LAB_TZ, YYYY-MM-DD) of the newest measurement
    the set holds, by the timestamps inside the files: R1 results, benches,
    hil flights. File modification times are not used (a checkout rewrites
    them). A UTC timestamp is brought to the lab's zone first: a flight at
    07:30 on the 17th in the lab is of the 17th, though its UTC date is the
    16th."""
    dates = []
    for r in t["rows"]:
        dates += [lab_date(x.get("created_utc")) for x in r["r1_results"]]
        for rec in r["records"]:
            if rec.get("metric") == "bench":
                dates.append(lab_date(rec.get("created_utc")))
            if rec.get("metric") == "hil_follow":
                dates.append(rec.get("date"))
    dates = [d for d in dates if d]
    return max(dates) if dates else None


def comparison_gaps(t: dict) -> list[str]:
    """What the set lacks to compare the backends on the same tests - the
    grant's "Q2 measurement set" - each as '<row>: <what>': every row with a
    launch family (the backends that can fly C2) needs an Orin timing (R1, or
    its server's bench where R1 does not cover it) and the protocol's count
    of C2 flights; the stub, R1's floor, needs an R1 on an Orin."""
    gaps = []
    for r in t["rows"]:
        if r.get("unavailable"):
            continue
        cells = r["cells"]
        if r.get("family"):
            lacks = []
            if not r.get("orin_timed"):
                lacks.append("no Orin timing")
            n = sum(1 for f in r["records"] if f.get("metric") == "hil_follow"
                    and not f["not_c2"])
            if n < C2_FLIGHTS_PER_BACKEND:
                lacks.append(f"{n} of {C2_FLIGHTS_PER_BACKEND} C2 flights")
            if lacks:
                gaps.append(f"{r['backend']}: " + ", ".join(lacks))
        if r["backend"] == "stub" and not cells["rate_orin"].startswith("R1"):
            gaps.append("stub: no R1 on an Orin (R1's floor on the same host)")
    return gaps


def name_default(t: dict, backend: str, scope: str | None, date: str | None,
                 today: str | None = None) -> dict:
    """The default-backend record for table `t`, or DefaultRefused saying why
    it is refused. The grant: no default "until the Q2 measurement set lands".
    So the named backend must be a row the set holds an Orin timing and the
    protocol's count of C2 flights for; the set must compare it with the
    others on the same tests (comparison_gaps: every backend that can fly C2
    measured the same way, and the stub's R1 on an Orin); and the date must
    be after the set: later than its newest measurement, not before the
    protocol, not in the future - all as the lab's local dates (LAB_TZ).
    This tool reads no C1 flight, so C1 cannot stand in for C2 here."""
    row = next((r for r in t["rows"] if r["backend"] == backend), None)
    if row is None:
        raise DefaultRefused(f"--default {backend!r} is not a row of the table "
                         f"({', '.join(r['backend'] for r in t['rows'])})")
    if row.get("unavailable"):
        raise DefaultRefused(f"--default {backend}: {row['unavailable']}")
    if not (scope or "").strip():
        raise DefaultRefused("--default needs --default-scope: what it is the default for "
                         "(e.g. \"the follow task\")")
    try:
        day = datetime.strptime(str(date), "%Y-%m-%d").strftime("%Y-%m-%d")
    except ValueError:
        raise DefaultRefused(f"--default needs --default-date YYYY-MM-DD, got {date!r}") from None
    today = today or datetime.now(LAB_TZ).strftime("%Y-%m-%d")
    newest = newest_measurement(t)
    if day > today:
        raise DefaultRefused(f"--default-date {day} is in the future (today is {today})")
    if day < PROTOCOL_DATE:
        raise DefaultRefused(f"--default-date {day} is before {PROTOCOL_ID} ({PROTOCOL_DATE}): "
                         f"a default is named after the measurement set, not before it")
    if newest and day <= newest:
        raise DefaultRefused(f"--default-date {day} is not after the set's newest measurement "
                         f"({newest}): a default is named after the set")
    cells = row["cells"]
    c2 = [f for f in row["records"] if f.get("metric") == "hil_follow" and not f["not_c2"]]
    if not row.get("orin_timed"):
        raise DefaultRefused(f"--default {backend}: the set holds no Orin rate for it "
                         f"({cells['rate_orin']})")
    if len(c2) < C2_FLIGHTS_PER_BACKEND:
        raise DefaultRefused(f"--default {backend}: the set holds {len(c2)} C2 flight(s) of "
                         f"it on hil; the protocol asks for {C2_FLIGHTS_PER_BACKEND}")
    gaps = comparison_gaps(t)
    if gaps:
        raise DefaultRefused(f"--default {backend}: the set does not yet compare it with the "
                         f"other backends on the same tests - " + "; ".join(gaps))
    return {"backend": backend, "label": row["label"], "scope": scope.strip(), "date": day,
            "named_by": "the lab",
            "recorded_with": "python tools/vla_backend_table.py table --default",
            "set_newest_measurement": newest, "protocol_date": PROTOCOL_DATE,
            "rate_orin": cells["rate_orin"],
            "closed_loop_hil_c1": cells["closed_loop_hil_c1"],
            "closed_loop_hil_c2": cells["closed_loop_hil_c2"],
            "confirmation": "this tool records no confirmation of the choice by anyone"}


def build_table(demo_out: Path = ROOT / "demo" / "out",
                experiments_out: Path = ROOT / "experiments" / "out",
                model_root: Path = Path(os.environ.get("VLAGUARD_MODEL_ROOT", "D:/models")),
                evidence: Path = EVIDENCE, serving_json: Path = SERVING_JSON,
                docs_data: Path = DOCS_DATA, runs: list[Path] | tuple = (),
                bundles: Path = BUNDLES, default: dict | None = None,
                today: str | None = None, repo_weights: Path = ROOT) -> dict:
    """Assemble the table. `demo_out` and `runs` are where run folders are
    looked for (`runs`: further folders, e.g. the disk they were moved to);
    `docs_data` holds the committed flight summaries and the 28 October
    matrix; `default` is {"backend", "scope", "date"} (name_default) or None."""
    run_roots = [Path(demo_out)] + [Path(r) for r in runs]
    added = [(_abs(demo_out), "demo/out"), (_abs(experiments_out), "experiments/out")] + \
        [(_abs(r), Path(r).as_posix().rstrip("/")) for r in runs]
    _ALIASES.extend(added)
    try:
        hil_flights = extract_hil_follows(run_roots, Path(docs_data))
        records = (extract_openvla_probes(demo_out)
                   + extract_aerialvla_flight_rates(Path(experiments_out) / "results.jsonl")
                   + extract_stated_rates()
                   + extract_aerialvla_evals(ROOT / "training")
                   + extract_bc_offline(ROOT / "models")
                   + extract_stub_dev_rail(demo_out)
                   + extract_openvla_closed_loop_statement()
                   + extract_orin_benches(evidence)
                   + hil_flights
                   + extract_matrix_1028(Path(docs_data), run_roots)
                   + extract_matrix_agx(run_roots))
        all_rates = load_rate_results(evidence)
    finally:
        del _ALIASES[len(_ALIASES) - len(added):]
    rates = [r for r in all_rates if r["_is_r1"]]
    records += [_other_protocol_record(r) for r in all_rates if not r["_is_r1"]]
    serving = load_serving(serving_json)
    rows = []
    for bid, spec in BACKENDS.items():
        mine = [r for r in records if r.get("backend") == bid]
        r1 = [r for r in rates if r["backend"] == bid]
        # The Orin column takes only a result whose host passed
        # guardrail.manifest's Orin rule, whatever folder it was found in.
        desk = [r for r in r1 if not r["host"]["jetson"]["is_orin"] and not _is_refused(r)]
        orin = [r for r in r1 if r["host"]["jetson"]["is_orin"] and not _is_refused(r)]
        desk_no = [r for r in r1 if not r["host"]["jetson"]["is_orin"] and _is_refused(r)]
        orin_no = [r for r in r1 if r["host"]["jetson"]["is_orin"] and _is_refused(r)]
        benches = [m for m in mine if m.get("metric") == "bench" and m.get("host_is_orin")]
        cells = {}
        if desk:
            cells["rate_desktop"] = _r1_cell(desk)
        elif desk_no:
            cells["rate_desktop"] = _refused_cell(desk_no)
        elif bid == "aerialvla_lora" and serving_r1_cell(serving):
            cells["rate_desktop"] = serving_r1_cell(serving)
        elif bid == "aerialvla_bf16" and serving_bf16_cell(serving):
            cells["rate_desktop"] = serving_bf16_cell(serving)
        else:
            other = [m for m in mine
                     if m["metric"] in ("latency", "inference_hz_in_flight", "inference_hz",
                                        "seconds_per_action")
                     or (m["metric"] == "follow_matrix" and not m.get("status"))]
            cells["rate_desktop"] = ("not R1 - see records" if other else
                                     ("n/a (no weights here)" if spec.get("unavailable")
                                      else "not measured"))
        if orin:
            cells["rate_orin"] = _r1_cell(orin)
        elif spec.get("unavailable"):
            cells["rate_orin"] = "weights not obtained"
        elif bid in SERVED_BACKENDS:
            # R1 does not cover a served model (PROTOCOL R1 served_backends):
            # its Orin timing is its server's bench, and no R1 is owed.
            no_r1 = (f"R1 does not cover a model served over protocol v2 (`measure` refuses "
                     f"it): its Orin timing is its server's bench ({spec['measured_by']})")
            parts = [bench_cell(b) for b in benches]
            cells["rate_orin"] = ("; ".join(parts + [no_r1]) if parts
                                  else "no Orin bench yet; " + no_r1)
        else:
            # No R1 from an Orin: an attempt that was refused, then the benches
            # that are not R1, each with its label, then what is still owed.
            to_measure = "to measure on the Orin (R1)" + (
                f"; needs {spec['needs']}, which the AGX Orin's native environment does not "
                f"install (deploy/orin/agx_vla_env.sh)" if spec.get("needs") else "")
            parts = ([_refused_cell(orin_no)] if orin_no else []) + \
                [bench_cell(b) for b in benches]
            cells["rate_orin"] = ("; ".join(parts + ["R1: " + to_measure.replace(" (R1)", "", 1)])
                                  if parts else to_measure)
        cl = [m for m in mine if m["metric"] in ("closed_loop", "follow_matrix")
              and not m.get("status")]
        cl_missing = [m for m in mine if m["metric"] == "closed_loop"]
        cells["closed_loop_desktop"] = "not C1 - see records" if (cl or cl_missing) else (
            "n/a (no weights here)" if spec.get("unavailable") else "not measured")
        cells["closed_loop_hil_c1"] = ("weights not obtained" if spec.get("unavailable")
                                       else spec.get("c1") or C1_TO_FLY)
        cells["closed_loop_hil_c2"] = c2_cell(
            [m for m in mine if m.get("metric") == "hil_follow"], spec)
        rows.append({"backend": bid, **{k: v for k, v in spec.items()},
                     "weights_status": weights_status(bid, model_root, repo_weights),
                     "cells": cells, "records": mine,
                     # timed on an Orin at all: an R1 result, or a labelled bench
                     "orin_timed": bool(orin or benches),
                     "orin_r1": bool(orin), "orin_bench": bool(benches) and not orin,
                     "orin_attempt_only": bool(orin_no) and not orin,
                     "r1_results": [{k: v for k, v in r.items() if k not in ("latency_s",)}
                                    for r in r1]})
    listed = set(BACKENDS)
    timeable = [r for r in rows if not r.get("unavailable")
                and r["backend"] not in SERVED_BACKENDS]
    n_r1 = sum(1 for r in rows if r["orin_r1"])
    n_bench = sum(1 for r in rows if r["orin_bench"])
    n_attempt = sum(1 for r in rows if r["orin_attempt_only"])
    n_c2 = sum(1 for r in rows if r["cells"]["closed_loop_hil_c2"].startswith("C2,"))
    n_c1 = sum(1 for r in rows if any(m.get("protocol") == "C1" for m in r["records"]))
    if n_r1 or n_bench or n_attempt:
        orin_says = (f"the Orin rate column holds R1 for {n_r1} of the {len(timeable)} "
                     f"backends `measure` can time, and a bench that is not R1, labelled as "
                     f"such, for {n_bench} backends"
                     + (f"; a recorded attempt with no timing for {n_attempt} more"
                        if n_attempt else "")
                     + f" (R1 does not cover the {len(SERVED_BACKENDS)} backends served over "
                       f"protocol v2: their Orin timing is their server's bench)")
    else:
        orin_says = "the Orin columns are empty"
    c1_says = (f"C1 has been flown by {n_c1} backend{'s' if n_c1 != 1 else ''}" if n_c1
               else "C1 has been flown by no backend")
    urban = sorted(q.name for q in Path(bundles).glob("*urban*")) if Path(bundles).is_dir() \
        else []
    t = {
        "schema": TABLE_SCHEMA, "protocol": PROTOCOL,
        "created_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "model_root": str(Path(model_root).as_posix()),
        "rows": rows,
        # Records for backends that are not table rows (the run1 and run3
        # fine-tunes, the follow fine-tunes, the hand-written follow, the
        # scripted stand-in): kept, so nothing read from disk disappears
        # silently.
        "other_records": [r for r in records if r.get("backend") not in listed],
        "contradictions": check_claims(),
        "aerialvla_serving": serving_for_table(serving),
        "c1_bundles": {"policy": "policies/urban_demo_policy.yaml", "looked_in": "bundles/",
                       "found": urban},
        "g3": g3_facts(rows),
        "default_backend": None,
        "decision": (
            "The grant defers the choice to this measurement set, and " + orin_says
            + f". On hil, {c1_says} and C2 by {n_c2} (the protocol "
              f"asks for {C2_FLIGHTS_PER_BACKEND} flights of each)"
            + ". The stored dev-rail runs (demo/out/ros2_*, not KPI-bearing: the grant "
              "takes KPI figures from hil) were flown by the stub."),
    }
    t["comparison_gaps"] = comparison_gaps(t)
    if default:
        t["default_backend"] = name_default(t, default.get("backend"), default.get("scope"),
                                            default.get("date"), today=today)
    return t


def _summary_line(rec: dict) -> str:
    src = rec.get("source", {}).get("file", "?")
    if rec.get("status"):
        return f"{rec['metric']}: {rec['status']} ({src})"
    m = rec["metric"]
    if m == "latency":
        vals = ", ".join(f"{v:.2f} s" for v in rec["values_s"])
        warm = (f"; warm {rec['warm_latency_s']:.2f} s = {rec['hz_warm']:.2f} Hz"
                if rec.get("warm_latency_s") else "")
        return (f"{rec['quantisation']}: per-inference {vals}{warm}; VRAM {rec['vram_gb']} GB "
                f"[{src}, {rec['source']['file_date']}]")
    if m == "inference_hz_in_flight":
        return (f"in flight: {rec['hz_min']:.2f}-{rec['hz_max']:.2f} Hz over {rec['flights']} "
                f"flights ({rec['inferences']} inferences) [{src}]")
    if m == "inference_hz":
        return f"stated {rec['value_hz']} Hz, no timing file [{src}]"
    if m == "seconds_per_action":
        return f"stated: {rec['value']} [{src}]"
    if m == "bench":
        where = "an Orin" if rec.get("host_is_orin") else "not an Orin"
        return f"{bench_cell(rec)} (host: {where}, by {rec['host_basis']})"
    if m == "follow_matrix":
        fl = rec["flights"]
        return (f"follow, {rec['matrix']}, simple_flight ({rec['protocol']}): {len(fl)} "
                f"flight{'s' if len(fl) != 1 else ''}, within 30 m "
                + _span([f.get("within_30m") for f in fl]) + f" [{src}]")
    if m == "hil_follow":
        return (f"{rec['tag']} ({rec['protocol']}): within 30 m "
                f"{_show(rec.get('frac_within_30m'), '.3f')}, {rec['ticks']} ticks, "
                f"{rec['end_reason']} [{src}]")
    if m == "closed_loop":
        if rec.get("protocol") == "stated only":
            return f"stated: {rec['value']} [{src}]"
        if rec["backend"] == "bc_v3":
            return (f"offline: reach {rec['reach_rate_unshielded']} unshielded, "
                    f"{rec['reach_rate_shielded']} shielded over {rec['episodes']} episodes "
                    f"(kinematic rollouts, not a simulator) [{src}]")
        if rec["backend"] == "stub":
            return (f"{rec['run']}: reached={rec['reached']}, {rec['interventions']} "
                    f"interventions in {rec['ticks']} ticks, P0 escape rate "
                    f"{rec['p0_violation_escape_rate']} [{src}]")
        mode = (f"ASSISTED goal_blend {rec['goal_blend']}" if rec["assisted"]
                else f"pure (goal_blend 0), yaw gain {rec['yaw_gain']}")
        return (f"{rec['reached']}/{rec['episodes']} reached, {mode}, mean interventions "
                f"{rec['mean_interventions']} [{src}]")
    return json.dumps(rec)


def _show(v, fmt: str = ".3g", none: str = "-") -> str:
    return none if v is None else format(v, fmt)


def _count(v, none: str = "-") -> str:
    """A count as a file wrote it (an int, or a float that is whole); '-'
    when the file did not record it, never 0."""
    n = _num(v)
    if n is None:
        return none
    return str(int(n)) if n.is_integer() else format(n, "g")


def _all_records(t: dict, metric: str) -> list[dict]:
    """Every record of one kind, rows first, then the backends that are not
    rows; each with the label it is shown under."""
    out = []
    for r in t["rows"]:
        out += [dict(rec, _label=r["label"].split(" (")[0]) for rec in r["records"]
                if rec.get("metric") == metric]
    out += [dict(rec, _label=_other_label(rec.get("backend")))
            for rec in t.get("other_records") or [] if rec.get("metric") == metric]
    return out


def _other_label(backend) -> str:
    b = str(backend)
    if b == HAND_WRITTEN:
        return "Hand-written follow (no VLA)"
    if b == STAND_IN:
        return "Scripted stand-in (no VLA)"
    if b.startswith("aerialvla_ft:"):
        return f"AerialVLA follow fine-tune {b.split(':', 1)[1]} (not a row)"
    return f"{b} (not a row)"


def render_c2(t: dict) -> list[str]:
    c2p = t["protocol"]["C2_follow_closed_loop"]
    flights = _all_records(t, "hil_follow")
    out = ["## C2 on hil: the Project AirSim follow", "",
           f"{c2p['added_after_the_choice']}.", "",
           f"Scenario: {c2p['scenario']}. Rail: {c2p['rail']}. Launch: `{c2p['launch']['script']}` "
           f"with `{' '.join(c2p['launch']['node_args'])}`, {c2p['launch']['max_s']} s. A "
           f"flight is C2 when: " + "; ".join(c2p["counts_as_c2"]) + f". "
           f"{c2p['flights_per_backend']} flights per backend; {c2p['reported']}. "
           f"{c2p['not_separated'][0].upper() + c2p['not_separated'][1:]}.", ""]
    yes = sorted((f for f in flights if not f["not_c2"]), key=lambda f: (f["_label"], f["tag"]))
    no = sorted((f for f in flights if f["not_c2"]), key=lambda f: (f["_label"], f["tag"]))
    if yes:
        out += ["| Flight | Backend | Commit | Ticks | Ended | Within 30 m | Decisions/s, mission "
                "| Decisions/s, asked | Interventions per 100 ticks | P0 escapes "
                "| KPI outcome (kpi.json) | Ticks in a P0-unsafe position "
                "| P0 ticks not measurable / blind | Detector Hz |",
                "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for f in yes:
            out.append(
                f"| {f['tag']} | {f['_label']} | {str(f.get('code_revision') or '-')[:7]} "
                f"| {f['ticks']} | {f['end_reason']} | {_show(f['frac_within_30m'], '.3f')} "
                f"| {_show(f['decision_hz_mission'])} | {_show(f['decision_hz_asked'])} "
                f"| {_show(f['interventions_per_100_ticks'])} | {_count(f['p0_escapes'])} "
                f"| {kpi_outcome_text(f)} | {_count(f.get('unsafe_p0_ticks'))} "
                f"| {_count(f.get('p0_ticks_not_measurable'))} / "
                f"{_count(f.get('p0_ticks_blind'))} "
                f"| {_show(f['detector_hz'])} |")
        out += ["", "A P0 escape is a tick on which a P0 rule was violated and the action "
                    "flown still violated it after the Shield (guardrail/kpi.py); a tick in a "
                    "P0-unsafe position is counted by where the aircraft was. A flight can "
                    "have 0 escapes and still fail its KPI on position: the outcome column is "
                    "the flight's own verdict."]
    else:
        out.append("No C2 flight is on disk.")
    nulls = [f for f in flights if f.get("backend") == HAND_WRITTEN]
    out += ["", f"C2's null ({c2p['nulls'][0]}): "
            + (f"{len(nulls)} hil follow{'s' if len(nulls) != 1 else ''} of it on disk, "
               f"listed below" if nulls else "no flight of it on disk") + "."]
    owed = [f"{r['label']}: {r['cells']['closed_loop_hil_c2']}" for r in t["rows"]
            if r.get("family") and not r["cells"]["closed_loop_hil_c2"].startswith("C2,")]
    if owed:
        out += ["", "Backends with a launch family and no C2 flight yet:", ""] + \
            [f"- {o}" for o in owed]
    if no:
        out += ["", f"hil follow flights that are not C2 ({len(no)}), listed so none is "
                    f"dropped silently; they are not scored here:", "",
                "| Flight | Backend | Why it is not C2 | Ticks | Ended | Within 30 m "
                "| P0 escapes | KPI outcome |", "|---|---|---|---|---|---|---|---|"]
        for f in no:
            # a flight with no tick flew no mission: its counts are not 0, they
            # do not exist
            flew = bool(f.get("ticks"))
            out.append(f"| {f['tag']} | {f['_label']} | {'; '.join(f['not_c2'])} | {f['ticks']} "
                       f"| {f['end_reason']} | {_show(f['frac_within_30m'], '.3f')} "
                       f"| {_count(f['p0_escapes']) if flew else '-'} "
                       f"| {kpi_outcome_text(f) if flew else '- (no mission)'} |")
    out.append("")
    return out


def render_c1(t: dict) -> list[str]:
    b = t.get("c1_bundles") or {}
    flown = [r["label"] for r in t["rows"]
             if any(m.get("protocol") == "C1" for m in r["records"])]
    out = [("## C1 on hil: flown by " + ", ".join(flown) if flown
            else "## C1 on hil: flown by no backend, and why"), "",
           f"C1 stays in the protocol as it was. Signed bundles of `{b.get('policy')}` in "
           f"`{b.get('looked_in')}` on this host: "
           + (", ".join(f"`{n}`" for n in b.get("found") or []) or "none") + ".", ""]
    for r in t["rows"]:
        out.append(f"- **{r['label']}**: {r['cells']['closed_loop_hil_c1']}")
    out.append("")
    return out


def render_matrices(t: dict) -> list[str]:
    recs = [r for r in _all_records(t, "follow_matrix")]
    if not recs:
        return []
    out = ["## Follow flights on Project AirSim simple_flight (not ArduPilot: not C1, not C2)",
           ""]
    for matrix in sorted({r.get("matrix") or "?" for r in recs if not r.get("status")}):
        group = [r for r in recs if r.get("matrix") == matrix and not r.get("status")]
        out += [f"**{matrix}.** {group[0]['conditions']}. Source: "
                f"`{group[0]['source']['file']}`.", "",
                "| Model | Flight | Where | Kind | Commit | Policy bundle | Within 30 m "
                "| VLA latency p50 (s) | P0 escape rate |",
                "|---|---|---|---|---|---|---|---|---|"]
        for r in group:
            for f in r["flights"]:
                kind = f"{f.get('kind')}" + (f" ({f['note']})" if f.get("note") else "")
                out.append(f"| {r['_label']} | {f.get('tag')} | {f.get('platform')} "
                           f"| {kind} | {str(f.get('code_revision') or '-')[:7]} "
                           f"| {f.get('bundle') or '-'} "
                           f"| {_show(f.get('within_30m'), '.3f')} "
                           f"| {_show(f.get('latency_p50_s'))} "
                           f"| {_show(f.get('p0_escape_rate'))} |")
        out.append("")
    for r in recs:
        if r.get("status"):
            out += [f"- {r['what']}: {r['status']} (`{r['source']['file']}`)", ""]
    return out


def render_g3(t: dict) -> list[str]:
    g = t.get("g3")
    if not g:
        return []
    out = ["## The reference design's candidate gate G3 (Orin VLA latency) against this set",
           "", f"The gate as written ({g['source']}): {g['gate']}", "",
           f"What this set holds on an Orin against {g['threshold_ms']:g} ms at p95 (the gate "
           f"names {g['gate_date']}):", ""]

    def lines(group):
        res = []
        for b in group:
            if not b["orin"]:
                res.append(f"- {b['label']}: no Orin timing on disk")
            for o in b["orin"]:
                if o["what"] == "attempt refused":
                    val = f"refused at {o['stage']}: {o['cause']}"
                elif o.get("p95_ms") is not None:
                    val = f"p95 {o['p95_ms']:.4g} ms (n={o.get('n')})"
                elif o.get("slowest_ms") is not None:
                    val = f"{o['slowest_ms']:.4g} ms (n={o.get('n')}, no p95)"
                elif o.get("in_flight_p50_ms"):
                    lo, hi = o["in_flight_p50_ms"]
                    val = (f"{lo:.4g} ms" if lo == hi else f"{lo:.4g}-{hi:.4g} ms") + \
                        f" over {o.get('n')} flight{'s' if o.get('n') != 1 else ''} (no p95)"
                else:
                    val = "no timed sample"
                res.append(f"- {b['label']}: {o['what']}: {val} [{o['file']}]")
        return res
    out += ["7 B-class rows:", ""] + lines(g["seven_b"])
    out += ["", "Rows that are not 7 B-class:", ""] + lines(g["smaller"])
    out += ["", f"int8 on an Orin: {g['int8_on_orin']}. {g['int8_note'][0].upper()}"
                f"{g['int8_note'][1:]}.", "",
            "This table records measurements. Whether the gate is added and applied is not "
            "recorded by this tool.", ""]
    return out


def render_markdown(t: dict) -> str:
    d = t.get("default_backend")
    if d:
        default_line = (
            f"**Default backend:** {d['label']} for {d['scope']}, named by {d['named_by']} on "
            f"{d['date']} (`{d['recorded_with']}`), after this measurement set (its newest "
            f"measurement: "
            f"{d['set_newest_measurement']}; `{t['protocol']['id']}` of {d['protocol_date']}). "
            f"Its own numbers in this set. Orin rate: {d['rate_orin']}. C1: "
            f"{d['closed_loop_hil_c1']}. C2: {d['closed_loop_hil_c2']}. "
            f"{d['confirmation'][0].upper() + d['confirmation'][1:]}. {t['decision']}")
    else:
        default_line = f"**Default backend:** none. {t['decision']}"
        gaps = t.get("comparison_gaps")
        if gaps:
            default_line += (" What the set lacks before `table --default` names one (the same "
                             "tests for every backend that can fly C2, and R1's floor): "
                             + "; ".join(gaps) + ".")
    out = ["# VLA backend comparison - measurement table",
           "",
           f"Generated by `python tools/vla_backend_table.py table` on {t['created_utc']}. "
           f"Protocol `{t['protocol']['id']}` (the protocol itself: "
           f"`python tools/vla_backend_table.py protocol`). Do not edit by hand; "
           f"re-run the tool."
           + (f" The AerialVLA serving section and that row's desktop rate were added on "
              f"{t['serving_added_utc']} by `python tools/vla_backend_table.py serving`, "
              f"which re-renders this file from vla_backend_table.json without re-reading "
              f"the run folders." if t.get("serving_added_utc") else ""),
           "",
           default_line,
           "",
           "| Backend | Weights on this host | Rate, desktop | Rate, Jetson Orin | "
           "Closed loop, desktop | Closed loop on hil, C1 | Closed loop on hil, C2 |",
           "|---|---|---|---|---|---|---|"]
    for r in t["rows"]:
        ws = r["weights_status"]
        if not ws:
            w = "none needed" if not r.get("unavailable") else "not obtained"
        else:
            w = "; ".join(f"{x['path']}: {'yes' if x['present'] else 'MISSING'}" for x in ws)
        c = r["cells"]
        out.append(f"| {r['label']} | {w} | {c['rate_desktop']} | {c['rate_orin']} | "
                   f"{c['closed_loop_desktop']} | {c['closed_loop_hil_c1']} | "
                   f"{c['closed_loop_hil_c2']} |")
    out.append("")
    out += render_c2(t) + render_c1(t) + render_matrices(t) + render_g3(t)
    if t.get("aerialvla_serving"):
        out += render_serving(t["aerialvla_serving"])
    out += ["## Records already on disk (none of them is R1 or C1)", "",
            "The hil follow flights and the simple_flight matrices have their own sections "
            "above and are not repeated here.", ""]
    for r in t["rows"]:
        shown = [rec for rec in r["records"]
                 if rec.get("metric") not in ("hil_follow", "follow_matrix")]
        if not shown and not r.get("unavailable") and not r.get("attempt"):
            continue
        out.append(f"**{r['label']}**")
        out.append("")
        if r.get("unavailable"):
            out.append(f"- {r['unavailable']}")
        if r.get("attempt"):
            out.append(f"- an attempt: {r['attempt']}")
        for rec in shown:
            out.append(f"- {_summary_line(rec)}")
            if rec.get("conditions"):
                out.append(f"  - conditions: {rec['conditions']}")
        out.append("")
    if t["contradictions"]:
        out += ["## Statements a file on disk contradicts", ""]
        whys: list[str] = []
        for c in t["contradictions"]:
            note = f" ({c['note']})" if c.get("note") else ""
            out.append(f"- `{c['doc']}`{note}: \"{c['claim'][:160]}\" - contradicted by "
                       f"`{c['contradicted_by']}` ({c['field']} = {c['value']})")
            if c["why"] not in whys:
                whys.append(c["why"])
        for w in whys:
            out += ["", f"Why, and the correct statement: {w}."]
        ev = t["contradictions"][0].get("evidence") or []
        if ev:
            out += ["", "The evaluations behind the correction, read from disk:", "",
                    "| File | Adapter | Reached | goal_blend (assist) | yaw gain | "
                    "Mean efficiency |", "|---|---|---|---|---|---|"]
            for e in ev:
                out.append(f"| `{e['file']}` | `{e['adapter']}` | {e['reached']}/{e['episodes']} "
                           f"| {e['goal_blend']} ({'on' if e['assisted'] else 'off'}) "
                           f"| {e['yaw_gain']} | {e['mean_efficiency']} |")
        out.append("")
    c2p = t["protocol"]["C2_follow_closed_loop"]
    out += ["## What R1, C1 and C2 are", "",
            f"- **R1 (rate, open loop):** {t['protocol']['R1_rate']['inputs']['camera_vla']}; "
            f"{t['protocol']['R1_rate']['warmup']} warm-up, {t['protocol']['R1_rate']['timed']} "
            f"timed inferences, {t['protocol']['R1_rate']['isolation']}. Reference: "
            f"{t['protocol']['R1_rate']['reference']}. Floor: {t['protocol']['R1_rate']['floor']}.",
            f"- **C1 (closed loop):** {t['protocol']['C1_closed_loop']['scenario']}; "
            f"{t['protocol']['C1_closed_loop']['seeds']} seeds, "
            f"{t['protocol']['C1_closed_loop']['time_limit_s']} s limit, reached within "
            f"{t['protocol']['C1_closed_loop']['reached_radius_m']} m. "
            f"{t['protocol']['C1_closed_loop']['assist']} Nulls: "
            + "; ".join(t["protocol"]["C1_closed_loop"]["nulls"]) + ".",
            f"- **C2 (closed-loop follow, added {c2p['added']}):** {c2p['scenario']}; "
            f"{c2p['rail']}. Reported: " + "; ".join(c2p["outputs"]) + ". Null: "
            + "; ".join(c2p["nulls"]) + f". {c2p['added_after_the_choice']}.",
            f"- **Changes from vla-backend-protocol/1:** {t['protocol']['changes_from_1']}.",
            ""]
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("protocol", help="print the protocol as JSON")
    p_t = sub.add_parser("table", help="assemble the table from files on disk")
    p_t.add_argument("--out-json", type=Path, default=DEFAULT_TABLE_JSON)
    p_t.add_argument("--out-md", type=Path, default=DEFAULT_TABLE_MD)
    p_t.add_argument("--model-root", type=Path,
                     default=Path(os.environ.get("VLAGUARD_MODEL_ROOT", "D:/models")))
    p_t.add_argument("--demo-out", type=Path, default=ROOT / "demo" / "out",
                     help="where the run folders are (default: this checkout's demo/out; a "
                          "worktree has none, so name the lab checkout's)")
    p_t.add_argument("--experiments-out", type=Path, default=ROOT / "experiments" / "out")
    p_t.add_argument("--runs", type=Path, nargs="*", default=[],
                     help="further folders of run folders (the disk the hil runs were moved "
                          "to); demo-out wins for a tag in both")
    p_t.add_argument("--bundles", type=Path, default=BUNDLES,
                     help="where signed bundles are looked for (C1's urban_demo bundle)")
    p_t.add_argument("--repo-weights", type=Path, default=ROOT,
                     help="the checkout whose models/ holds the weight files that are not in "
                          "git (default: this one)")
    p_t.add_argument("--default", default=None, metavar="BACKEND",
                     help="name a default backend; refused unless the set holds an Orin "
                          "timing and the C2 flights of every backend that can fly C2, the "
                          "stub's R1 on an Orin, and --default-date (the lab's local date) is "
                          "after the set (module docstring, THE DEFAULT LINE)")
    p_t.add_argument("--default-scope", default=None,
                     help="what it is the default for, e.g. \"the follow task\"")
    p_t.add_argument("--default-date", default=None, metavar="YYYY-MM-DD",
                     help="the day the lab named it, as the lab's local date (UTC+8)")
    p_v = sub.add_parser("serving", help="summarise demo/vla_server_v2.py bench results into "
                                         "docs/data and add them to the committed table")
    p_v.add_argument("--bench", type=Path, nargs="+", required=True,
                     help="bench JSONs, one per configuration")
    p_v.add_argument("--roundtrip", type=Path, default=None,
                     help="a `vla_server_v2.py ping --out` JSON (TCP round trip)")
    p_v.add_argument("--roundtrip-config", default=None,
                     help="the configuration the ping ran against, e.g. nf4/sdpa/merged")
    p_v.add_argument("--vram-budget-gb", type=float, required=True,
                     help="the most VRAM the model's process may take beside the simulator "
                          "(recorded; the low-VRAM option's rule)")
    p_v.add_argument("--desktop-baseline-gb", type=float, default=None,
                     help="for benches without a per-run nvidia-smi reading before the load: "
                          "what the desktop held on the card before the runs (recorded)")
    p_v.add_argument("--note", default=None, help="the note on the choice stored with the table")
    p_v.add_argument("--out", type=Path, default=SERVING_JSON)
    p_v.add_argument("--table-json", type=Path, default=DEFAULT_TABLE_JSON)
    p_v.add_argument("--table-md", type=Path, default=DEFAULT_TABLE_MD)
    p_m = sub.add_parser("measure", help="run R1 for one backend on this host")
    p_m.add_argument("--backend", required=True, choices=list(BACKENDS))
    p_m.add_argument("--topology", default=os.environ.get("VLAGUARD_TOPOLOGY"))
    p_m.add_argument("--allow-gpu", action="store_true")
    p_m.add_argument("--model-root", type=Path, default=None)
    p_m.add_argument("--warmup", type=int, default=PROTOCOL["R1_rate"]["warmup"])
    p_m.add_argument("--timed", type=int, default=PROTOCOL["R1_rate"]["timed"])
    p_m.add_argument("--csp", choices=("off", "on"), default="off",
                     help="the OpenVLA-7B keys only: the prompt arm, as real_vla_demo.py "
                          "--csp (default off, that script's default)")
    p_m.add_argument("--out", type=Path, default=None,
                     help="default: <evidence>/<topology>/vla_rate_<backend>.json, "
                          "<evidence> = $VLAGUARD_EVIDENCE_OUT or deploy/evidence")
    p_m.add_argument("--cpus", default=None,
                     help="pin to these logical CPUs, e.g. 0-15 (recorded in the result)")
    p_m.add_argument("--note", default=None,
                     help="free text stored with the result, e.g. the Orin's power mode "
                          "(nvpmodel is not visible inside a container)")
    args = ap.parse_args(argv)

    if args.cmd == "protocol":
        print(json.dumps(PROTOCOL, indent=2))
        return 0
    if args.cmd == "table":
        default = None
        if args.default or args.default_scope or args.default_date:
            if not args.default:
                print("REFUSED  --default-scope / --default-date name no backend (--default)")
                return 2
            default = {"backend": args.default, "scope": args.default_scope,
                       "date": args.default_date}
        try:
            t = build_table(demo_out=args.demo_out, experiments_out=args.experiments_out,
                            model_root=args.model_root, runs=args.runs, bundles=args.bundles,
                            default=default, repo_weights=args.repo_weights)
        except DefaultRefused as exc:           # nothing is written
            print(f"REFUSED  {exc}")
            return 2
        args.out_json.parent.mkdir(parents=True, exist_ok=True)
        write_lf(args.out_json, json.dumps(t, indent=1) + "\n")
        write_lf(args.out_md, render_markdown(t) + "\n")
        for r in t["rows"]:
            print(f"{r['backend']:<20} rate desktop: {r['cells']['rate_desktop']}")
            print(f"{'':<20} rate Orin   : {r['cells']['rate_orin']}")
            print(f"{'':<20} hil C2      : {r['cells']['closed_loop_hil_c2']}")
        d = t["default_backend"]
        print("default backend: " + (f"{d['backend']} for {d['scope']}, {d['date']}"
                                     if d else "none"))
        for c in t["contradictions"]:
            print(f"CONTRADICTION  {c['doc']}: {c.get('note') or ''}")
        if t["contradictions"]:
            print(f"               why: {t['contradictions'][0]['why']}")
        print(f"wrote {_rel(args.out_json)} and {_rel(args.out_md)}")
        return 0
    if args.cmd == "serving":
        benches = [json.loads(p.read_text(encoding="utf-8")) for p in args.bench]
        rt = None
        if args.roundtrip:
            raw = json.loads(args.roundtrip.read_text(encoding="utf-8"))
            rt = {"config": args.roundtrip_config, "n": raw.get("n"),
                  "roundtrip": raw["roundtrip"], "server_latency": raw["server_latency"],
                  "model": (raw.get("hello") or {}).get("model", {}).get("id")}
        s = summarise_serving(benches, args.vram_budget_gb, rt,
                              desktop_baseline_gb=args.desktop_baseline_gb)
        if args.note:
            s["choice_note"] = args.note
        write_lf(args.out, json.dumps(s, indent=1) + "\n")
        # Add the section to the COMMITTED table and re-render it from its own
        # JSON: a full `table` run re-reads the gitignored run folders, which a
        # worktree does not have (their rows would turn into "source missing").
        t = json.loads(args.table_json.read_text(encoding="utf-8"))
        t["aerialvla_serving"] = serving_for_table(s)
        t["serving_added_utc"] = s["created_utc"]
        for r in t["rows"]:
            if r["backend"] == "aerialvla_lora" and not r["cells"]["rate_desktop"].startswith("R1:"):
                r["cells"]["rate_desktop"] = serving_r1_cell(s) or r["cells"]["rate_desktop"]
        write_lf(args.table_json, json.dumps(t, indent=1) + "\n")
        write_lf(args.table_md, render_markdown(t) + "\n")
        for c in s["configs"]:
            print(f"{_cfg_name(c):<22} p50 {c['p50_ms']:>7} ms  p95 {c['p95_ms']:>7} ms  "
                  f"VRAM {c['vram_inference_peak_gb']} GB allocated, {c['vram_process_gb']} GB "
                  f"process  agree {c['agreement_probe']}")
        print(f"no default; low-VRAM option: {s['low_vram']}; fidelity option: {s['fidelity']}; "
              f"fastest: {s['fastest']}")
        print(f"wrote {_rel(args.out)}, {_rel(args.table_json)} and {_rel(args.table_md)}")
        return 0
    pinned = None
    if args.cpus:
        from profile_shield_tick import parse_cpus, pin_cpus
        try:
            pinned = pin_cpus(parse_cpus(args.cpus))
        except (RuntimeError, ValueError, OSError) as exc:
            print(f"REFUSED  --cpus {args.cpus}: {exc}")
            return 2
    try:
        res = measure(args.backend, args.topology, model_root=args.model_root,
                      allow_gpu=args.allow_gpu, warmup=args.warmup, timed=args.timed,
                      csp=args.csp)
        res["cpu_affinity"] = pinned
        res["note"] = args.note
    except PermissionError as exc:
        print(f"REFUSED  {exc}")
        return 2
    except (ValueError, RuntimeError) as exc:
        print(f"FAILED   {exc}")
        return 1
    res["command"] = "python tools/vla_backend_table.py " + shlex.join(
        argv if argv is not None else sys.argv[1:])
    from profile_shield_tick import evidence_root
    suffix = "-csp" if args.csp == "on" else ""
    out = args.out or (evidence_root() / args.topology / f"vla_rate_{args.backend}{suffix}.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    write_lf(out, json.dumps(res, indent=1) + "\n")
    if res.get("status") == "refused":
        # The attempt is on record with its cause; that is this key's result.
        ref = res["refusal"]
        print(f"{args.backend}: attempt refused at {ref['stage']}: {ref['cause']}")
        print(f"wrote {_rel(out)} (the refusal, no timing)")
        return 0
    s = res["stats"]
    print(f"{args.backend}: {_fmt_ms(s)}; meets {CONTRACT_HZ:g} Hz at p95: "
          f"{s.get('meets_contract_p95')}; undecodable {res.get('undecodable_outputs')}")
    print(f"wrote {_rel(out)}")
    return 0 if s.get("n") else 1


if __name__ == "__main__":
    sys.exit(main())
