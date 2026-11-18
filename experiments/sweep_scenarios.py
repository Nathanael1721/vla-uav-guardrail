"""Run the scenario library, or a stress profile, and score every episode with
the contractual KPIs.

    python experiments/sweep_scenarios.py                    # the library, one pass
    python experiments/sweep_scenarios.py --profile smoke    # ~50 scenarios x 1 seed
    python experiments/sweep_scenarios.py --profile nightly  # ~200 scenarios x 3 seeds
    python experiments/sweep_scenarios.py --profile broad    # ~3000 x 5 (defined; long)
    python experiments/sweep_scenarios.py --seed 1002        # every episode on one seed
    python experiments/sweep_scenarios.py --only nfz-head-on corridor-along
    python experiments/sweep_scenarios.py --backend sitl     # ArduPilot + MAVROS 2 rail
    python experiments/sweep_scenarios.py --publish          # REPLACE docs/data/scenario_sweep.json
    python experiments/sweep_scenarios.py --profile broad --resume DIR --shard 1/4
    python experiments/sweep_scenarios.py --profile broad --resume DIR --merge-only

UNATTENDED RUNS (2026-11-17, WP4-06). `--resume DIR` (headless only) saves
each finished episode as its own checkpoint file under DIR/episodes, written
atomically (a temporary file, fsync, os.replace), and on a later run with the
same DIR replays every checkpoint instead of flying the episode again: a
killed or rebooted run picks up where it stopped. DIR/checkpoint.json pins
what the checkpoints belong to - code revision, library and profile file
hashes, the options that change a flight, the expanded episode list - and a
run with any of them different is refused (exit 2), so two configurations are
never merged into one sweep. `--shard I/N` flies only every N-th episode (the
I-th of each N, 1-based) and writes checkpoints only; N shards can run one
after another or side by side into one DIR. `--merge-only` flies nothing:
refused (exit 2) while any episode has no checkpoint, otherwise it writes the
one results JSON and kpi_report.md exactly as a single run would (the replay
is pinned by tests/test_sweep_resume.py), with a `_resume` record of how many
episodes were replayed, their summed episode time, the sweep's wall span
(the folder's first run to its newest checkpoint) and the invocation; its
`_command` is the one uninterrupted run it equals (no --resume, --shard or
--merge-only). DIR/checkpoint.json is created exclusively, so shards started
together into a fresh DIR fly under one bundle stamp; a shard reads only its
own checkpoints. A resumed run prints a progress line every minute.

Results go to demo/out/sweep/ (or demo/out/stress_<profile>/): the results
JSON, one episode bundle per episode, and `kpi_report.md`, the grant's KPI
report for the run (one row per (template, parameter cell)). A run that flies a
comparison configuration (`--forecast-arms grant 3s`, `--response-arms ideal
ardupilot`, `--wind-model open_loop`) goes to demo/out/cmp_<profile>_<label>/
instead (run_folder, since 2026-10-20), outside the stress_* folders that
tools/kpi_report.py reads by default. Until 2026-10-06 a
plain run overwrote docs/data/scenario_sweep.json, a published artefact that the
evaluation data, the deck and tests/test_kpi_magnitudes.py read; replacing it
now takes `--publish`, and a CHANGELOG entry.

EXIT CODES. 0: every gated episode passed (known failures and tracked
episodes are reported, not failed). 1: a gate failed, an episode errored, or a
pinned defect started passing. 2: refused before running - an `--only` name
that matched nothing, a run of zero episodes, two cells that fly the same
episode, or a profile whose count is outside its declared range. 3: the run
finished but scored nothing (every episode skipped). Each of 2 and 3 is a run
that would otherwise have printed a clean result while testing nothing. 4
(2026-10-19): no gate failed, but the KPI rollup or the top-K failures could
not be computed, so the run's KPI report is missing or flagged - until then
that was recorded in the JSON only and the run exited 0. (1 takes precedence
over 4, and 4 over 3.) A failure of a gate a known-failure pin does not name
is a failure (1) even inside the pinned cell.

WHY THIS EXISTS

`docs/MIDTERM-REPORT-Aug2026.md` says, in those words, "Scenario sweep harness
not built." Everything this project claims about the Shield rests on a handful of
missions somebody chose to fly by hand, one at a time. A rule with no scenario is
a rule nobody has watched fail.

WHAT IT BECAME ON 2026-10-06: THE START OF THE GRANT'S STRESS HARNESS

The Stress Testing page asks for more than a regression list (audit cards WP4-02,
-04, -06, -12, -14, WP3-19, X-14):

  * scenarios "validated through Pydantic at load time": the library is loaded
    through `guardrail/scenario_spec.py` (ScenarioSpec / ScenarioEvent, copied
    from the grant field for field). A misspelt key stops the load;
  * seeds and profiles: `--seed`, and `--profile smoke|nightly|broad` reading
    `experiments/profiles/*.yaml` - the grant's scheduling table. The counts
    come from parameter sweeps and seeded geometry draws. The loader refuses a
    swept parameter that nothing flown reads, and this script refuses, before
    running, a run in which two cells fly the same episode - so the counts are
    not padded. How many cells actually BEHAVED differently is reported beside
    the count (distinct KPI signatures);
  * the Safety Shield page's smoke matrix (PDF p6): high-speed near-edge,
    sudden NFZ, time-window switch instant, three simultaneous violations, and
    GPS noise / latency on the state the Shield sees. Plus wind as a velocity
    disturbance, the grant example's `wind_speed_mps: [0, 2, 5]`, and GPS
    denial as a frozen pose (audit card X-14);
  * "every episode emits a six-field manifest": each episode writes an
    episode bundle - `manifest.json` (the six fields), `kpi.json`,
    `harness_events.jsonl` - under the grant's directory name.

AND ON 2026-10-07: THE SHIELD'S EVENTS, ITS FSM, AND THE ARMS

  * Events. All seven of the grant's event types are applied through the
    Shield's own event APIs (`apply_event`): spawn / translate / rotate /
    scale a dynamic_nfz (a translation or rotation may be a step or a
    constant motion), activate / deactivate a rule (a hot-applied
    time_window_switch, unscheduled or with its own weekly window), and
    swap a corridor. Until then only spawn existed, as a polygon_fence that
    the Shield now locks at mission start.
  * The window switch instant is read from the SHIELD (`rule_status`'s
    `in_force`, both sides of the instant), not from models.py's
    minute-resolution `active_at`. Beside it the harness measures the t- / t+
    boundary check: the lag between the Shield's switch and the requirement's
    (the schedule read exactly), whether it fell on the conservative side
    (a rule switching OFF is held until after the edge, one switching ON is
    enforced from before it), how close to the edge a closing rule still bound
    (`release_gap_s_*`) and on how many ticks an opening rule was enforced
    before it was in force (`anticipated_ticks_*`, the lookahead seeing it).
    Since 2026-10-21 (C2) that count is a violation of the rule OR the
    decision's trace naming it in `window_opens_on` (a window proved to open
    on the aircraft, Shield._opening_imposed, where the escape breaks
    nothing); the ticks counted only through the trace are reported apart
    (`anticipated_opening_only_ticks_*`), so the extension is visible in
    every record and not only in this docstring (review of 3660383, P2).
  * The escalation FSM. The Shield's FSM (guardrail/fsm.py, wired into
    Shield.filter) decides what is flown: the Shield's action, a stop, or
    nothing - LOITER, RTL and LAND own the aircraft, and the headless
    autopilot below flies them. RTL_triggered and Land_triggered are real
    outcomes, `failsafe_triggered` is the FSM entering RTL or Land (it was "any
    Shield brake"), and the labelled fail-safe correctness is printed with
    guardrail.fsm.score_failsafe_triggers' nulls and confidence bound.
  * Arms. `paraphrase` flies K paraphrases of the instruction beside the
    canonical wording (guardrail.paraphraser), `ab` flies the rule summary on
    and off on the same seed. The headless pilots read no text, so every arm
    of an episode is the same flight: the sweep checks that it is, and
    reports the arms as plumbing, never as a robustness number.

THE PREFIX CONSUMER (2026-11-16, WP4-15)

A profile with `prefix_consumer: action_set` flies every episode's pilot law
through guardrail.prefix_consumer.PrefixConsumer (the grant's default
action-set adapter): the prefix-on arm with the Prefix Compiler's CSP for the
cell (prefix_csp: compiled once, before take-off, for static policies only),
the prefix-off arm without it. The two arms then differ by the Prefix alone,
so for such a pilot a differing pair is the measurement, not a defect
(arms_summary keeps "the arms must be one flight" for every other pilot),
and the rails fly both arms (`--prefix on|off` to the VLA stub; rail_refusal
still refuses the prefix-off and paraphrase arms of a pilot that reads
nothing). `prefix_aa` flies the prefix-on arm of the cells it names twice on
one seed: the A/A pair, identical headless (this harness is deterministic)
and the noise of a same-seed pair on a real-time rail, the floor a delta is
read against. A cell's outcome and fail-safe labels were derived for its
rule-ignorant pilot, so the prefix-on arm of a consuming pilot is scored
without them (scoring_spec); its gates, the safety invariants, still apply.
No run without the key builds, names or flies an episode differently.

THE HEADLESS BACKEND, AND WHAT IT IS NOT

Shield plus a plain kinematic integrator: no simulator, no GPU, no autopilot, no
WSL. It integrates exactly the 4-D action the contract defines, which is the only
thing the Shield is responsible for. What it therefore CANNOT test: the
autopilot's tracking, timing under load, or perception. Once the FSM hands the
aircraft to LOITER, RTL or LAND, ArduCopter's own modes are modelled as ideal
position control (AP_* below: hold, climb to 15 m and return at 5 m/s, descend
at 0.5 m/s).

THE SHIELD'S FORECAST (2026-10-12)

Every Shield of an episode - the flying one, the ground-truth oracle and the
autopilot checker - is built with the grant's forecast, guardrail.shield's
LOOKAHEAD_S / LOOKAHEAD_DT_S (5 s at 0.1 s, 50 poses; Safety Shield p2,
"5 s of predicted trajectory at 10 Hz = 50 future poses"), unless the spec
overrides it. Until then they were built at 3 s / 0.5 s (6 poses), so no
headless KPI came from the grant's configuration. The old forecast is kept as
a labelled comparison arm (`--forecast-arms grant 3s`, forecast_for): its
episodes carry `forecast-3s` in their id, their bundles say so in
manifest_extras, and their results sit in `_forecast_comparison` and in
`_cell_rollup` rows marked `grant_config: false` - never in `_counts`,
`_failsafe_labels`, `_true_state` or the exit code. `--publish` flies both.

THE WIND MODEL (2026-10-12)

Every velocity the harness flies - the Shield's setpoint, a Brake's zero, the
raw pilot of a control arm, the modelled LOITER / RTL / LAND - reaches the
vehicle through ArduCopter's position controller. GUIDED velocity setpoints
run SubMode::VelAccel (ArduCopter/mode_guided.cpp, velaccel_control_run, at
the Copter-4.5.7 pin): AC_PosControl::input_vel_accel_xy integrates the
velocity target into a POSITION target (update_pos_vel_accel_xy), and
update_xy_controller closes a P loop on the position error (PSC_POSXY_P 1.0)
plus a PID velocity loop with an integrator (PSC_VELXY_P 2.0, I 1.0). A steady
wind is therefore rejected: a zero-velocity Brake holds its position. Until
2026-10-12 a GUIDED setpoint drifted open-loop with the wind (this harness's
kinematic integrator added the wind to the commanded velocity), which is what
parked aircraft at a margin ring and blew them in.

The model (`--wind-model guided`, the default; `open_loop` is the old one,
kept for comparison):
  * steady wind: the commanded ground velocity is flown, the wind rejected up
    to AP_HOLD_AUTHORITY_MPS; of a stronger wind only the excess drifts the
    aircraft downwind (the position target does not run away:
    update_pos_vel_accel_xy stops it on the limited axis). The airframe's
    own speed limit is NOT modelled: in still air every family flies what
    it is commanded (12 m/s in near-edge-at-12mps), as before, so the change
    touches only the families that blow a wind (on the ideal vehicle; the
    vehicle-response arm below flies the command through the measured
    acceleration, jerk and lag first);
  * gusts: a gust moves the aircraft off its position target, and the P loop
    pulls it back at PSC_POSXY_P (an Ornstein-Uhlenbeck offset; the velocity
    loop's own rejection is left out, so this is the pessimistic side of the
    closed loop);
  * not modelled: the velocity loop's transients (its integrator re-trimming
    as drag changes with the commanded velocity). They exist with no wind
    too, and the kinematic integrator ignores them in both;
  * optimistic upwind: the authority is applied to the WIND, not to the
    airspeed the frame must hold (|commanded - wind|). A command into a wind
    within authority is flown exactly even when it needs more airspeed than
    AP_HOLD_AUTHORITY_MPS (a 4 m/s leg into a 5 m/s headwind needs 9 m/s; an
    airframe capped at 6.4 m/s of airspeed would make 1.4 m/s over the
    ground), and beyond authority the command is flown plus the excess (a
    5 m/s RTL into 12 m/s drifts back at 0.6 m/s, where that airframe would
    drift at 5.6 m/s; both are "no progress home"). Capping the airspeed
    (ground = wind + clip(commanded - wind, A)) would also cap still-air
    flight at 6.4 m/s, which near-edge-at-12mps flies at 12, so the model
    would contradict the families that assume the commanded speed is
    reachable. Where it matters in the library: the Shield-flown legs of
    wind_disturbance are tailwinds, but an RTL home flies INTO them - into
    wind-beyond-authority's 8 m/s it makes 3.4 m/s of progress here, where
    the capped airframe would be carried away at 1.6 m/s (into 12 m/s both
    are "no progress"), and into wind-gusts' 2 m/s it needs up to 7 m/s.
AP_HOLD_AUTHORITY_MPS is 6.4 m/s: the airspeed ArduPilot's SITL default frame
reaches at ArduCopter's 30 deg ANGLE_MAX (config.h DEFAULT_ANGLE_MAX 3000)
when all its drag is momentum drag. SIM_Frame.h puts that frame at 15.08 m/s
at 45 deg (refSpd / refAngle); with all its drag quadratic the 30 deg figure
would be 11.5 m/s. The lower end is used, so the model claims wind rejection
only where that frame has it whatever its drag split. No SITL run with wind
exists in demo/out: the rails' scripts set no SIM_WIND_* parameter, so the
ros2fix* flights flew SITL's still air, where an open-loop and a closed-loop
model of a zero setpoint agree. The model rests on the source, not on a
flight; a SITL run with SIM_WIND_SPD set is what would test it. Since
2026-11-07 the rails (`--backend sitl` / `hil`) fly a cell's wind as SITL's
own SIM_WIND_* ("THE SIMULATOR'S STRESSORS ON THE RAILS", above
rail_refusal): there the wind acts through ArduPilot's physics and its
position controller, never through this model or `--wind-model`.

THE VEHICLE RESPONSE (2026-10-19): A LABELLED COMPARISON ARM

Above, every family flies its commanded ground velocity on the tick - the
constant-velocity model the Shield forecasts with - so no episode could show
how far an aircraft carries while it brakes, or what the escalation FSM does
when the aircraft cannot turn on the tick. `--response-arms ideal ardupilot`
flies every episode a second time on the MEASURED ArduPilot response
(VehicleResponse, AP_RESPONSE; docs/DESIGN-harness-vehicle-response.md): each
tick's command acts delay_s after it is issued, the velocity target moves
toward it under an acceleration and a jerk limit (the sqrt-controller shape
of AC_PosControl's velocity shaping), and the airframe follows the target
with a first-order lag; the vertical axis has its own three. The parameters
are fitted, through this same class, to the recorded Copter-4.5.7 flights
(pas_ap_*, sitl_*) by tools/measure_vehicle_response.py, whose `--check`
fails when AP_RESPONSE drifts from them. The wind model then acts on what the
airframe flew exactly as it acts on a command.

It is built exactly like the 3 s forecast arm, and kept out of the same
places: its copies are tagged `response-ardupilot` (run id, bundle name,
`/response-ardupilot` KPI row), every bundle and KPI table of a run records
its vehicle (`vehicle_response`, `response_arm`; the ideal one too), and its
results sit in `_response_comparison` (per family, both arms' unsafe-position
and polygon entries, P0 escapes, true-state P0 ticks of the command and of the
velocity the airframe flew, and labelled fail-safe score over the same (cell,
seed) pairs) - never in `_counts`, `_failsafe_labels`, `_true_state`, the top
failures or the exit code. Since 2026-10-20 an arm bundle's extras are
"sweep-episode-extras/2" (a reader that does not know the arm refuses them),
a profiled run with the arm goes to demo/out/cmp_<profile>_response-ardupilot
(run_folder), and tools/kpi_report.py ranks its copies apart from the grant's
top-K (review of T1b-harness-vehicle). One
comparison dimension per run (a 3 s episode on the ArduPilot response would
shadow neither twin), never with --publish, never on the SITL rail (which
flies ArduPilot for real).

Under the arm only, and each only so the comparison measures the vehicle
rather than an artefact of the ideal one: the airframe enters at the pilot's
first command held to the policy's kinematic envelope (entry_velocity: the
families start mid-flight); the goto pilot slows into its goal (_pilot), and
the modelled RTL closes on its position targets (_autopilot), where their
bang-bang commands would otherwise overshoot by a braking distance; and the
ground stops a descent. The ideal vehicle - the default, every headline
number and every pin - flies exactly what it flew before the arm existed
(tests/test_harnesstruth_response.py, and the smoke, nightly and library
runs compared field by field in the design note).

If the Shield is given a measured velocity (State.velocity, another branch):
the arm's `veh.velocity` is that measurement, one line in _episode (the
design note's "Hook").

THE RTL HAND-OVER (2026-10-12)

While the FSM is in RTL, guardrail/rtl_path.py RTLWatch judges the modelled
RTL each tick - the straight leg home at RTL_ALT against the P0 zones and the
altitude band, progress toward home over the grant's T, and the battery hook
(`stress.battery_critical_at_s`) - and its verdict is passed as
`rtl_failed`, so the FSM's RTL -> Land edge (G6) can fire. P0 ticks the
autopilot flew are counted per mode (`autopilot_p0_flown_ticks_by_mode`).

It is not KPI-grade and is built so it cannot be mistaken for it. Its manifest
names the topology `headless-kinematic`, and `sim_speedup` is MEASURED (simulated
seconds over wall seconds, typically several hundred) rather than declared, so
`guardrail.manifest.is_kpi_grade` refuses every episode on two counts. The grant's
smoke and nightly rows ask for sim_speedup 1.0 in the HIL topology; these
profiles reproduce the grant's SCOPE, not its rail, and each profile file says so.

`--backend sitl` drives `sitl/run_ros2_demo.sh` -> `sitl/ros2_shield_node.py`
(ArduPilot SITL + MAVROS 2). The node takes the mission's target, speed, seed,
time cap, policy (or bundle), subject and Shield arm; its pilot is StubVLA and
it starts from take-off at the pad. A scenario is flown there only when that
IS its mission - pilot `stub_vla`, start at the pad, no events, stressors,
clock or relabels - and every other one is reported SKIPPED with the reason
(`sitl_refusal`). Flying a scenario there and scoring it against its gates
under its id while the rail flew something else would describe a different
flight. A harness that quietly scores 11/11 while running 4 is worse than no
harness. A rail flight is scored from the rail's own flight_log.jsonl and
metrics.json (`sitl_extra`): the goal and the FSM record from metrics.json,
breaches and polygon entries from the logged positions (`rail_measurements`).
Until 2026-10-19 only the rows were read, so every such gate was "not
measured" and the one scenario the rail may fly could never pass.

TRACKED FAMILIES

A family marked `expect: tracked` (GPS noise beyond the margin, GPS denial,
drawn gusts) is gated only on what must hold whatever the sensor does. An
episode that passes those gates is reported "tracked", never "pass", and the
summary prints the true-state polygon entries beside the Shield's own P0
escape count - the grant's KPI ("counter from Shield repair log") is 0 in
those runs while the aircraft was, in some of them, metres inside the zone.

WHAT THE AUTOPILOT FLEW

The P0 escape rate is the Shield's counter over the ticks the Shield decided
(guardrail.kpi, "AUTOPILOT TICKS"); the ticks the modelled LOITER / RTL /
LAND flew are outside it by definition. They are not outside the report: per
episode `autopilot_p0_flown_ticks`, `autopilot_polygon_entries` and
`autopilot_breaches`, and per run the `_true_state` block and its own summary
line. (Until the review of stress-harness-2 none of the three reached a
run-level number, so a run whose modelled RTL flew straight through a zone
still read "P0 escape 0.0, 1 polygon entry".)

SCORING

Through `guardrail.kpi.compute`, the same function the delivered flights use.
Writing a second KPI implementation here would let the sweep and the artefacts
disagree about what a P0 escape is, and the sweep would be the one nobody
checked. The harness adds only what a kinematic run can see directly and the
rails cannot: ground-truth safety under degraded perception, self-inflicted
breaches, and the fail-safe and outcome LABELS each scenario declares. The
label-based fail-safe correctness is printed with its two nulls (never
trigger, always trigger) and the grant's ">= 99 %": a Shield that never
triggers scores the never-trigger null, so a correctness below it is worse
than doing nothing, and is said so.
"""
from __future__ import annotations

import argparse
import copy
import glob
import hashlib
import json
import math
import os
import random
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path, PurePath
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
# The checkout this file is in, whatever ROOT is later set to (a test points
# ROOT at a temporary tree): the nodes' sources are read from here.
_REPO = ROOT
sys.path.insert(0, str(ROOT))

from guardrail import hold_flight as HF                            # noqa: E402
from guardrail import kpi as K                                     # noqa: E402
from guardrail import prefix_consumer as PC                        # noqa: E402
from guardrail import rtl_path as RP                               # noqa: E402
from guardrail import scenario_events as SE                        # noqa: E402
from guardrail import vla_stub as VS                               # noqa: E402
from guardrail.frames import (BODY_STOP, body_to_local_ned,       # noqa: E402
                              filter_world_as_body, local_ned_to_world)
from guardrail.fsm import cap_exceeded                           # noqa: E402
from guardrail.geometry import ring_contains                     # noqa: E402
from guardrail.models import (Action4D, CorridorSwap, DynamicNFZ,  # noqa: E402,F401
                              State, TimeWindowSwitch)
from guardrail.scenario_spec import (FORECAST_ARMS,                # noqa: E402
                                     GRANT_EVENT_TYPES, RAIL_PHYSICS,
                                     RAIL_SIM_SPEEDUP_WHY, RESPONSE_ARMS, Defaults,
                                     Library, ScenarioEvent, ScenarioSpec,
                                     check_comparison_arms, check_forecast_arms,
                                     check_response_arms, expand,
                                     legacy_pins_read_as_before, load_profile)
from guardrail.shield import (LOOKAHEAD_DT_S, LOOKAHEAD_S, RECOVERY_LAWS,  # noqa: E402
                              DR_DRIFT_MPS, Shield, recurrence_active)

LIB = ROOT / "experiments" / "scenarios.yaml"
PROFILES = ROOT / "experiments" / "profiles"
# The goal tolerance, guardrail.vla_stub's since the goto law moved there
# (2026-11-06): the pilot and the score read one number.
GOAL_TOL_M = VS.GOAL_TOL_M

# The manifest's topology for this backend. Not one of guardrail.manifest's
# names on purpose: those describe flight rails, and this is not one.
TOPOLOGY_HEADLESS = "headless-kinematic"
# The "VLA" of a headless episode is the dumb pilot below. model_hash() hashes
# this module's source for a code-only pilot, which pins it exactly.
PILOT_ID = "experiments.sweep_scenarios._pilot"
# Every event type of the grant's ScenarioEvent, since 2026-10-07 (until then
# only spawn_polygon_fence; the other six were refused).
SUPPORTED_EVENTS = GRANT_EVENT_TYPES

# The headless autopilot: what ArduCopter's own modes do once the escalation
# FSM hands the aircraft over (set_mode LOITER / RTL / LAND, setpoint "none").
# Modelled as ideal position control, wind and gusts rejected. The values are
# this model's: RTL_ALT 1500 cm, WPNAV_SPEED_UP 250 cm/s and LAND_SPEED
# 50 cm/s are ArduCopter's parameter defaults; the 5 m/s return speed is a
# choice (ArduCopter returns at RTL_SPEED, or WPNAV_SPEED when that is 0).
AP_RTL_ALT_M = 15.0
AP_RTL_SPEED_MPS = 5.0
AP_CLIMB_MPS = 2.5
AP_LAND_MPS = 0.5
AP_HOME_TOL_M = 2.0              # "home reached": horizontally over the launch point
AP_LANDED_M = 0.05
# A moved hold point (guardrail/holdpoint.py, since 2026-10-19) counts as
# reached within this distance on each axis; from then on the hold flies as
# an in-place one (zero velocity, the position controller holding it).
AP_HOLD_REACHED_M = 0.05
# What the modelled autopilot flies for the escalation FSM's Loiter / Land,
# recorded per episode (`autopilot_hold_model`, also in episode_start) and in
# the results file's info (review of T1c, P3):
#   moved_points  the FSM's hold point (guardrail/holdpoint.py, since
#                 2026-10-19): an in-place one as always, a moved one flown
#                 first as a GUIDED transit, then held. The default.
#   in_place      ArduPilot's own LOITER / LAND where the aircraft is
#                 (Shield(hold_points=False)): what every flight rail flew
#                 until 2026-11-07 - SITL, ROS 2, Project AirSim / ArduPilot
#                 and the city loop called no hold_at - and what c77c77d flew,
#                 row for row (tests/test_hold_points.py pins it by digest).
# Since 2026-11-07 the pymavlink rail (sitl/run_sitl_demo.py) and the ROS 2
# Shield node (sitl/ros2_shield_node.py, the hil path) fly the moved point
# (guardrail/hold_flight.py: GUIDED to it, the hold on arrival; `--hold-points
# in_place` flies the old hold), so the `rail` vehicle (RAIL_VEHICLE) holds at
# moved points; Project AirSim / ArduPilot and the city loop still hold in
# place.
AUTOPILOT_HOLD_MODELS = ("moved_points", "in_place")
DEFAULT_AUTOPILOT_HOLD = "moved_points"
# How the Shield recovers from a position that is already illegal (Shield's
# recovery_law, guardrail.shield.RECOVERY_LAWS has the full history), recorded
# per episode (`recovery_law`, also in episode_start) and in a bundle's
# `arm.recovery`:
#   cap_all    every recovery at its cap: the altitude band at the climb cap,
#              a static zone at the speed cap judged as flown, a stand-off
#              ring and a corridor at the speed cap (since 2026-11-06,
#              WP3-17). The default.
#   cap        the altitude band at the climb cap only (2026-10-29 to
#              2026-11-06).
#   lookahead  the band in one lookahead, every bundle flown before
#              2026-10-29 (an exponential approach with time constant L).
DEFAULT_RECOVERY_LAW = "cap_all"

# The wind model (module docstring, "THE WIND MODEL"). PSC_POSXY_P's default
# (POSCONTROL_POS_XY_P, AC_PosControl.cpp at Copter-4.5.7), and the strongest
# airspeed - so the strongest steady wind - the modelled airframe can hold.
WIND_MODELS = ("guided", "open_loop")
DEFAULT_WIND_MODEL = "guided"
AP_POSXY_P = 1.0
AP_HOLD_AUTHORITY_MPS = 6.4

# The forecast of the "3s" comparison arm: the harness's own until
# 2026-10-12 (the spec's lookahead_s, else 3.0 s, at a 0.5 s step).
LEGACY_LOOKAHEAD_S = 3.0
LEGACY_LOOKAHEAD_DT_S = 0.5
# `--publish` flies the grant's forecast and the labelled 3 s comparison.
PUBLISH_FORECAST_ARMS = ("grant", "3s")

SITL_SCRIPT = "sitl/run_ros2_demo.sh"
SITL_NODE = "sitl/ros2_shield_node.py"
# The rail's own mission, as sitl/ros2_shield_node.py and ros2_vla_stub_node.py
# hard-code it: `parse_command("fly to the northeast pad at 6 m/s")`, flown by
# guardrail.vla_stub.StubVLA from wherever take-off left the vehicle (the pad,
# local (0, 0)). tests/test_sweep.py re-reads the command from the node's
# source and the pad from guardrail.compiler.PLACES, so these cannot drift.
# Since the node took --target / --speed / --seed / --max-s (2026-10-07), the
# target and speed are the scenario's; the start, the pilot and the cruise
# altitude (parse_command's 15 m default) are still the rail's.
SITL_RAIL_COMMAND = "fly to the northeast pad at 6 m/s"
SITL_RAIL_START = (0.0, 0.0)
SITL_RAIL_TARGET = (30.0, 30.0)
SITL_RAIL_CRUISE_M = 15.0
SITL_RAIL_PILOT = "guardrail.vla_stub.StubVLA (sitl/ros2_vla_stub_node.py)"
# The node's policy-reference flags (build_sitl_cmd, "POLICY REFERENCES"):
# a YAML policy flies only with --dev-yaml; a bundle with the hash it must hold.
SITL_DEV_YAML_FLAG = "--dev-yaml"
SITL_EXPECTED_HASH_FLAG = "--expected-hash"
# The published library sweep. Written only with --publish (see the docstring).
PUBLISHED_SWEEP = ROOT / "docs" / "data" / "scenario_sweep.json"
# The grant's acceptance target for fail-safe trigger correctness (Stress
# Testing, Acceptance KPIs: ">= 99 %").
GRANT_FAILSAFE_TARGET = 0.99


class UnsupportedScenario(RuntimeError):
    """The backend cannot run this scenario as written. Reported, never skipped
    silently and never approximated."""


# --------------------------------------------------------------------------- #
# geometry of the mission
# --------------------------------------------------------------------------- #

def _rotate(x: float, y: float, cx: float, cy: float, deg: float) -> tuple[float, float]:
    """Rotate (x, y) about (cx, cy), clockwise seen from above (x North, y East),
    so +90 turns a northbound mission eastbound."""
    if not deg:
        return x, y
    r = math.radians(deg)
    dx, dy = x - cx, y - cy
    return (cx + dx * math.cos(r) - dy * math.sin(r),
            cy + dx * math.sin(r) + dy * math.cos(r))


@dataclass
class _Mission:
    start: State
    target: tuple[float, float, float | None] | None
    subject: tuple[float, float] | None
    action: dict[str, float] | None
    speed: float
    stub: Any = None                     # guardrail.vla_stub.StubVLA for `stub_vla`

    @property
    def home(self) -> tuple[float, float]:
        """The launch point, where RTL returns to."""
        return self.start.x, self.start.y


def _resolve_mission(spec: ScenarioSpec, policy, jitter: random.Random) -> _Mission:
    m = spec.mission
    sx, sy = m.start_pose.local(policy)
    cx, cy = (m.rotate_about.x, m.rotate_about.y) if m.rotate_about else (sx, sy)
    rx, ry = _rotate(sx, sy, cx, cy, m.rotate_deg)
    j = spec.stress.start_jitter_m
    if j > 0:
        rx += jitter.uniform(-j, j)
        ry += jitter.uniform(-j, j)
    start = State(x=rx, y=ry, up=float(m.start_pose.altitude),
                  yaw_deg=m.start_pose.yaw_deg + m.rotate_deg)
    target = None
    if m.target is not None:
        tx, ty = m.target.local(policy)
        tx, ty = _rotate(tx, ty, cx, cy, m.rotate_deg)
        target = (tx, ty, m.target.altitude)
    subject = None
    if m.subject is not None:
        subject = _rotate(m.subject.x, m.subject.y, cx, cy, m.rotate_deg)
    action = None
    if m.pilot.action is not None:
        action = dict(m.pilot.action)
        vx, vy = action.get("vx", 0.0), action.get("vy", 0.0)
        if m.rotate_deg and all(math.isfinite(v) for v in (vx, vy)):
            action["vx"], action["vy"] = _rotate(vx, vy, 0.0, 0.0, m.rotate_deg)
    speed = float(m.pilot.speed or 4.0)
    stub = None
    if m.pilot.type == "stub_vla":
        from guardrail.compiler import Mission
        from guardrail.vla_stub import StubVLA
        stub = StubVLA(Mission(
            task_text=m.task_prompt or "", target_x=target[0], target_y=target[1],
            cruise_alt_m=float(target[2] if target[2] is not None else start.up),
            speed_pref_mps=speed, start_x=start.x, start_y=start.y))
    return _Mission(start=start, target=target, subject=subject, action=action,
                    speed=speed, stub=stub)


# --------------------------------------------------------------------------- #
# pilots - deliberately dumb
# --------------------------------------------------------------------------- #

def _pilot(mission: _Mission, kind: str, st: State,
           approach_accel: float | None = None) -> Action4D:
    """What the operator ASKS for, which may be entirely illegal.

    A pilot that avoids violations on its own would leave the Shield with
    nothing to do and every scenario would pass for the wrong reason. It flies
    on the TRUE position: degraded perception is the Shield's problem here, not
    the pilot's, so a breach under GPS noise is the Shield's to own.

    `approach_accel` (the vehicle-response arm only; None is exactly the
    pilot above): the `goto` pilot is bang-bang - full speed until GOAL_TOL_M,
    then zero - which an ideal vehicle obeys on the tick. A vehicle that
    carries its speed would fly through the goal by the braking distance
    (about 16 m at 8 m/s) and swing around it for the rest of the episode,
    so every goto mission would read "goal missed" for a reason that is the
    pilot's, not the Shield's. Under a response the goto pilot slows into its
    goal at the vehicle's own acceleration (the sqrt controller of the
    remaining distance, AP_POSXY_P), still straight at the target and still
    blind to every rule. The stub_vla pilot already slows near its target
    (speed = min(preference, distance)) and the constant pilot has no goal;
    both are unchanged.

    ONE LAW PER PILOT (2026-11-06). The goto and constant laws are
    guardrail.vla_stub's (goto_action, constant_action), which the flight
    rails' VLA stub flies too (sitl/ros2_vla_stub_node.py --pilot): a
    scenario flown on the hil rail is flown by the law it is scored under.
    The rail's goto runs on `vla_stub.APPROACH_ACCEL_MPS2`, this harness's
    AP_RESPONSE acceleration (a real airframe carries its speed).
    """
    if kind == "constant":
        return VS.constant_action(mission.action)
    if kind == "stub_vla":
        return mission.stub.act(st)
    if kind == "goto":
        return VS.goto_action(mission.target, st, mission.speed, approach_accel)
    raise ValueError(f"unknown pilot type {kind!r}")


def hold_reached(st: State, hold: dict | None) -> bool:
    """Has the aircraft reached a MOVED hold point (horizontally, and in
    altitude for a Loiter; a Land transits level and then descends)?"""
    if not hold or hold.get("kind") != "moved":
        return True
    if math.hypot(hold["x"] - st.x, hold["y"] - st.y) > AP_HOLD_REACHED_M:
        return False
    return hold["mode"] == "LAND" or abs(hold["up"] - st.up) <= AP_HOLD_REACHED_M


def _autopilot(mode: str | None, st: State, home: tuple[float, float],
               dt: float, shaped: "ResponseParams | None" = None,
               rtl_alt: float | None = None, hold: dict | None = None,
               up_home: float | None = None) -> Action4D:
    """The velocity ArduCopter's own mode flies this tick (see AP_* above).

    LOITER holds position; RTL climbs to AP_RTL_ALT_M if below it, then flies
    straight home at AP_RTL_SPEED_MPS (ArduPilot's RTL knows nothing of the
    Shield's zones - whether the way home crosses one is measured, not
    assumed); LAND descends at AP_LAND_MPS. Any other mode (a fault latched
    before a mode was named) is treated as LOITER, ArduPilot's own fallback
    for a lost GUIDED stream.

    `shaped` (the vehicle-response arm only; None flies exactly the above):
    ArduPilot's RTL flies POSITION targets - the return altitude, then home -
    and closes each with its position controller, so the leg ends stopped at
    the target. Commanding "min(speed, distance / dt)" through a vehicle that
    carries its speed would overshoot home by the braking distance and leave
    a climb's overshoot uncorrected above RTL_ALT, which ArduPilot does not
    do. So under a response the RTL velocity is the sqrt controller of the
    remaining distance (AP_POSXY_P, and the response's own acceleration),
    capped at the same speeds, with the altitude held toward RTL_ALT on the
    way home; the horizontal leg starts once the climb is within 0.1 m (the
    same 0.1 m the FSM's home_reached flag allows). LOITER (hold) and LAND
    (descend at LAND_SPEED to the ground) are unchanged: the response itself
    carries the vehicle into the hold, and the ground stops a landing.

    `rtl_alt` (the arm only): the return altitude the RTL closes on, which
    _episode captures once, when the FSM hands the aircraft to RTL, as
    max(AP_RTL_ALT_M, the altitude then) - ArduPilot's RTL_ALT is "the
    minimum alt above home the vehicle will climb to before returning; if
    the vehicle is flying higher than this value it will return at its
    current altitude" (ArduCopter parameter docs; mode_rtl.cpp computes the
    return target once, on entry). None reads AP_RTL_ALT_M. Until 2026-10-20
    the arm closed on AP_RTL_ALT_M always, so an RTL entered above it would
    have descended to it on the way home, which neither ArduPilot nor the
    ideal vehicle does. Re-flown on 2026-10-20, the ideal vehicle is untouched
    and the arm's smoke and library runs are field for field what they were;
    in the nightly profile three arm episodes that entered RTL 1e-5 m above
    RTL_ALT moved by one intervention episode each, and no reported count
    moved (docs/DESIGN-harness-vehicle-response.md).

    `hold` is the FSM's hold point (ShieldDecision.hold_point, since
    2026-10-19). An in-place one, or None, is exactly the above. A MOVED one
    not yet reached (`hold_reached`) is flown first, as ArduCopter flies a
    GUIDED position target before the hold mode is set on arrival: straight
    at the plan's speed horizontally (the policy's cap, guardrail/
    holdpoint.py) and its climb rate vertically, each axis braking onto the
    point in its last step; a Land transits level, then descends. On the
    `rail` arm (TRANSIT_GIVE_UP_RESPONSES) _episode gives a transit up as the
    rails do (hold_flight.TransitWatch) and then passes no hold here: held where
    the aircraft is, as a reached one is. Under the
    vehicle-response arm (`shaped`) each axis is the sqrt controller of its
    remaining distance instead, capped at the same speeds, as the arm's RTL
    is above and for the same reason: the target is a POSITION, and
    "min(speed, distance / dt)" through a vehicle that carries its speed
    overshoots it by the braking distance (from 4 m/s, a hold 10 m east was
    passed by 4.4 m and not reached in 12 s; tests/test_hold_points.py
    test_on_the_response_arm_a_moved_hold_is_flown_as_a_position_target).
    Until the merge of the two branches (2026-10-21) the arm flew the
    ideal vehicle's command here. The plan's transit test (guardrail/
    holdpoint.py transit_breaks) judges the straight line at the plan's
    speed; the arm's airframe follows it with its own lag, and what that
    costs is counted as for every command the arm flies
    (`true_p0_vehicle_ticks`)."""
    if (mode in ("LOITER", "LAND") and hold is not None
            and hold.get("kind") == "moved" and not hold_reached(st, hold)):
        dx, dy = hold["x"] - st.x, hold["y"] - st.y
        d = math.hypot(dx, dy)
        speed = float(hold["speed_mps"])
        if d <= 1e-9:
            v = 0.0
        elif shaped is not None:
            v = min(speed, _sqrt_controller(d, AP_POSXY_P, shaped.accel_mps2, dt))
        else:
            v = min(speed, d / dt)
        vx, vy = ((v * dx / d, v * dy / d) if d > 1e-9 else (0.0, 0.0))
        vz = 0.0
        if mode == "LOITER":
            cz = float(hold.get("climb_mps") or AP_CLIMB_MPS)
            dz = hold["up"] - st.up
            vz = (math.copysign(_sqrt_controller(abs(dz), AP_POSXY_P, shaped.accel_z_mps2,
                                                 dt), dz)
                  if shaped is not None else dz / dt)
            vz = max(-cz, min(cz, vz))
        return Action4D(vx=vx, vy=vy, vz_up=vz)
    # RTL_ALT is a height above HOME. `up_home` (wf20): the aircraft's height
    # above home when `st.up` is not (a declared DEM, X-11: st.up is above the
    # ground under the aircraft); None, every episode without one, reads
    # st.up as before. The climb is flown as a vertical rate either way, and
    # the leg home is level in MSL, as ArduCopter's RTL flies it.
    uh = st.up if up_home is None else up_home
    if mode == "RTL" and shaped is not None:
        target = AP_RTL_ALT_M if rtl_alt is None else rtl_alt
        dz = target - uh
        vz = math.copysign(_sqrt_controller(abs(dz), AP_POSXY_P, shaped.accel_z_mps2, dt),
                           dz)
        vz = max(-AP_CLIMB_MPS, min(AP_CLIMB_MPS, vz))
        if uh < target - 0.1:
            return Action4D(vz_up=vz)
        dx, dy = home[0] - st.x, home[1] - st.y
        d = math.hypot(dx, dy)
        if d < 1e-9:
            return Action4D(vz_up=vz)
        v = min(AP_RTL_SPEED_MPS, _sqrt_controller(d, AP_POSXY_P, shaped.accel_mps2, dt))
        return Action4D(vx=v * dx / d, vy=v * dy / d, vz_up=vz)
    if mode == "RTL":
        if uh < AP_RTL_ALT_M - 1e-6:
            return Action4D(vz_up=min(AP_CLIMB_MPS, (AP_RTL_ALT_M - uh) / dt))
        dx, dy = home[0] - st.x, home[1] - st.y
        d = math.hypot(dx, dy)
        if d < 1e-9:
            return Action4D()
        v = min(AP_RTL_SPEED_MPS, d / dt)
        return Action4D(vx=v * dx / d, vy=v * dy / d)
    if mode == "LAND":
        return Action4D(vz_up=-min(AP_LAND_MPS, max(0.0, st.up) / dt))
    return Action4D()


def guided_ground_velocity(vx: float, vy: float, wind: tuple[float, float],
                           authority_mps: float = AP_HOLD_AUTHORITY_MPS
                           ) -> tuple[float, float]:
    """The ground velocity ArduCopter's position controller flies for a
    commanded ground velocity (vx, vy) in a steady wind (module docstring,
    "THE WIND MODEL"): the command itself, the wind rejected up to
    `authority_mps`, and only the excess of a stronger wind added. The
    airframe's own speed limit is not modelled: in still air the harness
    flies whatever it is commanded, as it always has, and a command into
    the wind is flown as if the airspeed it needs were available (the
    module docstring's "optimistic upwind")."""
    w = math.hypot(wind[0], wind[1])
    if w <= authority_mps + 1e-12:
        return vx, vy
    k = 1.0 - authority_mps / w
    return vx + wind[0] * k, vy + wind[1] * k


# --------------------------------------------------------------------------- #
# the vehicle-response arm (module docstring, "THE VEHICLE RESPONSE")
# --------------------------------------------------------------------------- #

DEFAULT_RESPONSE_ARM = "ideal"


@dataclass(frozen=True)
class ResponseParams:
    """One vehicle response (docs/DESIGN-harness-vehicle-response.md).

    delay_s       from the tick whose state the Shield judged to the position
                  controller acting on that tick's command: the rail's own
                  processing, MAVLink transport and the age of the position it
                  read, measured end to end (one number: the harness cannot
                  tell them apart, and the Shield faces their sum)
    accel_mps2    horizontal acceleration limit of the shaped velocity target
    jerk_mps3     horizontal jerk limit (how fast that acceleration may change)
    tau_s         first-order lag of the airframe's velocity behind the shaped
                  target (the velocity loop and the attitude loop together)
    *_z_*         the same for the vertical axis, which ArduCopter shapes apart
    substep_s     integration step inside one harness tick (50 Hz)
    """
    delay_s: float
    accel_mps2: float
    jerk_mps3: float
    tau_s: float
    accel_z_mps2: float
    jerk_z_mps3: float
    tau_z_s: float
    substep_s: float = 0.02

    def record(self) -> dict[str, float]:
        return {k: getattr(self, k) for k in (
            "delay_s", "accel_mps2", "jerk_mps3", "tau_s", "accel_z_mps2",
            "jerk_z_mps3", "tau_z_s", "substep_s")}


# The ArduPilot response, MEASURED (docs/DESIGN-harness-vehicle-response.md,
# "Measured"): fitted by tools/measure_vehicle_response.py, through this
# module's own VehicleResponse, to the recorded Copter-4.5.7 flights in the
# main checkout's demo/out on 2026-10-19:
#   delay_s 0.08     pas_ap_* (Project AirSim physics flown by ArduPilot; EKF
#                    position at 20 Hz drained every tick, so a row's position
#                    is fresh): 4659 samples over 14 GUIDED segments of 6
#                    runs. Profile: 0.0-0.15 s fit within 0.5 %, 0.4 s is 7 %
#                    worse - "under about 0.2 s" is what the data say.
#   accel 2.4, jerk 5.7, tau 0.03
#                    sitl_* (ArduPilot's own SITL airframe), each run on its
#                    own clock because those logs read a backlogged pose:
#                    1002 samples of 6 runs, the hover-to-6 m/s step of
#                    sitl_shield_off among them. RMS 0.216 m/s against 0.469
#                    for the ideal vehicle (with its best clock). The pas_ap
#                    airframe's own fit is accel 3.84, jerk 5.09, tau 0.02
#                    (RMS 0.259 against 0.773 ideal and 0.388 for the best
#                    pure delay, 0.9 s); the lower acceleration is flown
#                    (the longer braking distance), the jerk agrees.
#   accel_z 2.9, jerk_z 20.0, tau_z 0.12
#                    pas_ap_* vertical, WEAKLY identified: the vertical
#                    commands are small (|vz| p90 0.05 m/s), the fit only
#                    rules out a slow vertical axis (accel_z 1.0 / jerk_z 5.0
#                    fits 6x worse) and a 0.3 s pure lag fits as well.
# `python tools/measure_vehicle_response.py --check` re-fits and fails when
# these drift from the recordings.
AP_RESPONSE = ResponseParams(delay_s=0.08, accel_mps2=2.4, jerk_mps3=5.7, tau_s=0.03,
                             accel_z_mps2=2.9, jerk_z_mps3=20.0, tau_z_s=0.12)
# THE HIL RIG'S OWN RESPONSE (2026-11-07): the vehicle the KPI-grade runs fly -
# ArduCopter 4.5.7 SITL in GUIDED on the desktop, flown by the Shield node on
# the AGX Orin over MAVROS 2 - fitted by `tools/measure_vehicle_response.py
# --rig` (fit_rig: this module's VehicleResponse, the same loss and search as
# AP_RESPONSE's) to the hil rehearsal rh01's own flight logs: the command each
# GUIDED row sent against the fresh position it read, each flight's first
# segment from the hover the rail starts in kept whole. Fitted 2026-11-07 on 32
# flights (34 segments, 6570 samples; the two 12 m/s near-edge cells left out,
# RIG_EXCLUDE: they command more speed than the airframe reaches), the
# velocity residual (RMS, m/s) over every sample / the first 5 s from the
# hover (29 flights):
#   this fit                      0.099 / 0.093
#   AP_RESPONSE (ardupilot arm)   0.122 / 0.154   delay 0.08, 2.4 m/s^2, 5.7 m/s^3
#   the rail Shield's own model   0.258 / 0.458   0.45 s, 1.75 m/s^2 (its forecast)
#   ideal vehicle                 0.934 / 1.742
# The delay is the weakest-pinned horizontal parameter (+/- 25 % moves the RMS
# by 1-3 %), the lag barely at all; the vertical axis is weakly identified
# (the commands are small), as AP_RESPONSE's is. `--rig ... --check-rig`
# fails when this drifts from the logs.
RAIL_RESPONSE = ResponseParams(delay_s=0.13, accel_mps2=2.48, jerk_mps3=4.8, tau_s=0.03,
                               accel_z_mps2=1.3, jerk_z_mps3=35.2, tau_z_s=0.31)
RIG_FIT = {"source": ("tools/measure_vehicle_response.py --rig 'demo/out/hil_smoke/rh01_*' "
                      "(RIG_EXCLUDE: *near-edge-at-12mps*)"),
           "flights": "hil rehearsal rh01, 7 Nov 2026: 32 flights, 6570 samples",
           "rms_xy_mps": 0.099, "rms_xy_hover_start_mps": 0.093}
RESPONSES: dict[str, ResponseParams | None] = {"ideal": None, "ardupilot": AP_RESPONSE,
                                               "rail": RAIL_RESPONSE}
# How a response arm's airframe ENTERS the episode (2026-11-07): "cruise",
# at the pilot's first command held to the envelope (entry_velocity: the
# families start mid-flight; the ardupilot arm, every sweep run), or "rest",
# from a hover - how a RAIL starts every scenario, after its take-off at the
# start: the rail arm's own (RESPONSE_ENTRY). The ideal vehicle has no entry.
ENTRY_STATES = ("cruise", "rest")
DEFAULT_ENTRY = "cruise"
RESPONSE_ENTRY = {"ardupilot": "cruise", "rail": "rest"}
# The rail vehicle as run_episode flies it, for every caller deriving an
# expectation on it (the rail gates here, the fail-safe labels in
# tools/failsafe_rubric.py): the rig's response from a hover, and the FSM's
# hold points flown as the rails fly them since 2026-11-07 (guardrail/
# hold_flight.py: a moved point as a GUIDED transit, then the hold;
# AUTOPILOT_HOLD_MODELS "moved_points"). Until then "in_place": ArduPilot's own
# hold where the aircraft is, what every rail flew. Since 2026-11-12 also how
# a GPS denial reaches the Shield on a rail (GPS_DROPOUT_MODELS below).
RAIL_VEHICLE = {"response": "rail", "autopilot_hold": "moved_points",
                "gps_dropout": "dead_reckoned"}
# How a GPS denial (stress.gps_dropout_s) reaches the Shield (2026-11-12):
#   frozen         the LAST FIX, held for the window while the aircraft flies
#                  on - what a bridge that forwards the last pose would do. The
#                  headless model since X-14 (sensor_degradation.yaml) and the
#                  model of every label in experiments/templates.
#   dead_reckoned  the AUTOPILOT'S OWN ESTIMATE, as the rails realise the
#                  denial: SITL's GPS stops (SIM_GPS_DISABLE, rail_stress_plan)
#                  and ArduPilot's EKF3 keeps estimating on its IMU, so the
#                  pose the Shield is handed keeps moving and is wrong only by
#                  the estimator's drift, RAIL_EKF_DRIFT_MPS x the time since
#                  the loss, behind the true position along the ground track
#                  at the loss. The Shield is told the fix is gone (fix_ok
#                  False) exactly as with the frozen pose.
# The rail labels (tools/failsafe_rubric.py --response rail) were derived on
# the frozen pose until 2026-11-12, a pose no rail ever hands the Shield: on
# the KPI-grade nightly nt1111 they expected a fail-safe (E3) on gps-dropout
# lane 23.6 at 2-9 s and 4-11 s, where SITL's own truth puts EKF3's pose
# within 0.94 m of the aircraft at 4-11 s (under the 1 m margin) and within
# 1.39 m at 2-9 s, past 1 m only with the aircraft 8.4 m or more from
# nfz-square: E3 holds on none of the 24 gps-dropout flights of nt1111
# (docs/RUBRIC-failsafe.md, section 8, "2026-11-12").
GPS_DROPOUT_MODELS = ("frozen", "dead_reckoned")
DEFAULT_GPS_DROPOUT = "frozen"
# The pilot's frame and the arc forecast (run_episode `action_frame`,
# 2026-11-17, WP3-04 / ARCH-04). Since wf21 the Shield's input is the body
# frame on both arms and the integrator flies frames.body_to_local_ned of what
# the Shield streams: "world" (the default) keeps the arc forecast off,
# "body" turns it on.
ACTION_FRAMES = ("world", "body")
# EKF3's dead-reckoning drift on the hil rig, m/s: the upper envelope of
# |EKF3 - truth| / time since the loss (>= 2 s) over every GPS denial the rig
# has flown (nt1111_550-573, rh03_039/040, sm1110_043/044, sm1111_043/044:
# 30 flights, ArduCopter 4.5.7 SITL), read from each run's raw MAVLink
# (/uas1/mavlink_source in its bag: SITL's SIMSTATE, the simulator's true
# position, against GLOBAL_POSITION_INT, EKF3's). Envelope 0.37 m at 2 s,
# 0.77 m at 4 s, 1.35 m at 6 s, 1.47 m at 6.5 s; largest ratio 0.226 m/s; the
# error lies along the track at the loss (the estimate lags; across the
# track 0.18 m or less), and resets when the fix returns. A property of
# SITL's sensors, as RAIL_RESPONSE is of its airframe: a real IMU drifts
# faster, and a drone campaign re-measures it.
RAIL_EKF_DRIFT_MPS = 0.23
RAIL_EKF_DRIFT_SOURCE = (
    "hil GPS denials nt1111_550-573, rh03_039/040, sm1110_043/044, sm1111_043/044 "
    "(30 flights, ArduCopter 4.5.7 SITL): SIMSTATE (truth) against "
    "GLOBAL_POSITION_INT (EKF3) in each run's raw MAVLink; upper envelope of "
    "error / time since the loss, along the track at the loss")
# The autopilot's own verdict on that estimate (State.estimator_ok,
# 2026-11-12), as the ROS 2 node reports it (sitl/ros2_shield_node.py
# EstimatorHealth): on the same 30 denials EKF3's EKF_STATUS_REPORT kept
# EKF_POS_HORIZ_ABS for 6.8-6.9 s after GPS_RAW_INT's loss (posRetryTimeNoVel_ms,
# 7 s from its last GPS fusion), its test ratios frozen at their last fused
# values (0.02 or less); the node declares the estimate lost once unhealthy
# for the autopilot's own EKF failsafe time, 1 s (ArduCopter ekf_check). So a
# dead-reckoned denial hands the Shield estimator_ok True until
# RAIL_EKF_POS_VALID_S after the loss and False after it; a frozen one none
# (no autopilot vouches for a bridge's last pose). Every library window is 7 s
# or shorter: no cell reaches the False.
RAIL_EKF_POS_VALID_S = 6.8 + 1.0


def gps_dropout_for(model: str = DEFAULT_GPS_DROPOUT) -> dict[str, Any]:
    """How a GPS denial reaches the Shield under `model` (GPS_DROPOUT_MODELS),
    as recorded in an episode's start event and the label artefact."""
    if model not in GPS_DROPOUT_MODELS:
        raise ValueError(f"gps dropout model {model!r}: one of {list(GPS_DROPOUT_MODELS)}")
    if model == "frozen":
        return {"model": model, "pose": "the last fix, held for the window"}
    return {"model": model,
            "pose": ("the autopilot's dead-reckoned estimate: the true position less "
                     "drift_mps x the time since the loss, along the track at the loss"),
            "drift_mps": RAIL_EKF_DRIFT_MPS, "source": RAIL_EKF_DRIFT_SOURCE,
            "estimator_ok": ("True until estimator_valid_s after the loss, then False "
                             "(the autopilot's EKF_STATUS_REPORT, as the ROS 2 node reads it)"),
            "estimator_valid_s": RAIL_EKF_POS_VALID_S}
# The response arms whose modelled autopilot GIVES UP a moved hold's transit
# as the rails do (since 2026-11-07, evening; guardrail/hold_flight.py,
# GIVING UP): by the rails' own TransitWatch, judged on every tick of the plan
# on the true position, so a transit the rig would give up is given up here on
# the same tick, and a headless rail-arm number predicts the rig. Until then
# the harness flew every moved transit until it was reached; the ideal vehicle
# and the ArduPilot arm still do (the ideal vehicle arrives on its travel time;
# the ArduPilot arm models no rail of its own).
TRANSIT_GIVE_UP_RESPONSES = ("rail",)
# How the rail arm's autopilot flies a Loiter (since 2026-11-07, night;
# guardrail/hold_flight.py, THE MODE MAPPING), through the rails' own
# HoldFlight on every tick:
#   guided  held in GUIDED by the companion - HoldFlight's law toward the
#           hold's point, every tick, no hand-over - what the rails fly now;
#   brake   the rails' flow until then (a moved point's transit, then
#           ArduCopter's BRAKE; BRAKE at once for an in-place plan), with
#           BRAKE's hand-over carry modelled (BRAKE_PI_P / BRAKE_PI_I): flown
#           so, the arm reproduces the hil shakedown hs04's two failures.
RAIL_HOLD_MODELS = ("guided", "brake")
RAIL_HOLD_MODEL = "guided"
# ArduCopter 4.5.7's BRAKE after a GUIDED stream, as the rig flies it (hs04_006
# and hs03_004, docs/DESIGN-hold-points.md "BRAKE's carry"): the velocity loop
# starts with its integrator at the acceleration GUIDED's last command left -
# the response's shaped acceleration, `VehicleResponse.at` - and bleeds it off
# by its own PI dynamics, dv/dt = -P v + I, dI/dt = -I_gain v. ArduCopter's
# defaults (PSC_VELXY_P 2, PSC_VELXY_I 1) make that critically damped with a
# 1 s time constant, which fits the recorded velocities: hs04_006's
# integrator (1.85, -1.74) m/s^2, the peak 0.93 m/s 1.1 s after the hand-over,
# 1.7 m in 2.2 s and still 0.45 m/s at 2.7 s.
BRAKE_PI_P = 2.0
BRAKE_PI_I = 1.0


# The correction rate a non-negative error asks for (ArduPilot's AP_Math
# sqrt_controller shape): guardrail.vla_stub.sqrt_controller since 2026-11-06,
# where the goto law that also uses it moved; one implementation.
_sqrt_controller = VS.sqrt_controller


def _shape_step(cmd, vt: list, at: list, accel: float, jerk: float, h: float) -> None:
    """One substep of the jerk-limited velocity shaper, in place on the target
    velocity `vt` and target acceleration `at` (lists of axis components).

    The structure of AC_PosControl's shape_vel_accel_xy as the module
    docstring reads it: an acceleration toward the commanded velocity from
    the sqrt controller with gain jerk / accel, capped at `accel` (as a
    vector, so a diagonal command is not allowed sqrt(2) times more), and the
    change of acceleration capped at jerk * h. `math.inf` turns a limit off:
    with both off the target reaches the command within one substep, which is
    the ideal vehicle."""
    e = [c - v for c, v in zip(cmd, vt)]
    en = math.sqrt(sum(x * x for x in e))
    if en > 1e-12:
        if math.isinf(jerk):
            amag = min(accel, en / h)
        else:
            p = 0.0 if math.isinf(accel) else jerk / accel
            amag = min(accel, _sqrt_controller(en, p, jerk, h))
        ad = [x / en * amag for x in e]
    else:
        ad = [0.0] * len(e)
    if math.isinf(jerk):
        at[:] = ad
    else:
        da = [a - b for a, b in zip(ad, at)]
        dn = math.sqrt(sum(x * x for x in da))
        lim = jerk * h
        if dn > lim:
            da = [x * lim / dn for x in da]
        for k in range(len(at)):
            at[k] += da[k]
    for k in range(len(vt)):
        vt[k] += at[k] * h


def _lag(va: list, vt: list, tau: float, h: float) -> None:
    """The airframe's velocity `va` following the shaped target `vt` with a
    first-order lag `tau` (in place; tau 0 follows exactly)."""
    g = 1.0 if tau <= 0.0 else min(1.0, h / tau)
    for k in range(len(va)):
        va[k] += (vt[k] - va[k]) * g


class VehicleResponse:
    """The vehicle a response arm flies the harness's commands on.

        veh = VehicleResponse(AP_RESPONSE, v0=(vx, vy, vz_up))
        vx, vy, vz = veh.step((cmd.vx, cmd.vy, cmd.vz_up), dt)   # each tick

    `step` issues the tick's command and returns the airframe's MEAN velocity
    over the tick, so the harness's integrator (position += velocity * dt)
    moves the vehicle exactly as far as the airframe flew. A command acts
    `delay_s` after it is issued; until then the previous one is flown. The
    horizontal and vertical targets are shaped apart (_shape_step) and the
    airframe follows each with its first-order lag (_lag).

    `v0` is the velocity the vehicle already has at t0, and every command
    issued before t0 is taken to have been v0: the harness's episodes start
    mid-flight (a start pose, a cruise speed), and starting them from a hover
    would make the arm test a slow start instead of braking carry. A
    non-finite component of `v0` reads 0. A non-finite component of a
    command holds the active command's component (`nonfinite_commands`
    counts them): the Shield sanitises what it emits (action_contract), so
    only a Shield-off control arm could send one, and none in the library
    does.

    `command` and `advance` are `step`'s two halves, for a caller whose
    commands and samples fall at different times (tools/
    measure_vehicle_response.py, which fits these parameters to recorded
    flights through this same class)."""

    def __init__(self, params: ResponseParams, v0=(0.0, 0.0, 0.0), t0: float = 0.0):
        self.p = params
        v = tuple(float(x) if math.isfinite(float(x)) else 0.0 for x in v0)
        self.t = float(t0)
        self.active = v
        self._queue: deque = deque()
        self.vt, self.at, self.va = [v[0], v[1]], [0.0, 0.0], [v[0], v[1]]
        self.vtz, self.atz, self.vaz = [v[2]], [0.0], [v[2]]
        self.nonfinite_commands = 0
        # ArduCopter's BRAKE (brake()): the velocity loop's integrator, or
        # None while commands are flown.
        self._brake: list | None = None

    @property
    def velocity(self) -> tuple[float, float, float]:
        """The airframe's velocity now (vx, vy, vz_up)."""
        return self.va[0], self.va[1], self.vaz[0]

    def brake(self, on: bool) -> None:
        """Hand the airframe to ArduCopter's BRAKE (on) - its velocity loop
        starting from the acceleration the last commands left and bled off at
        the loop's own pace (BRAKE_PI_P / BRAKE_PI_I) - or back to commands
        (off). Horizontal only: BRAKE holds the altitude, as a zero vertical
        command does."""
        if on and self._brake is None:
            self._brake = [self.at[0], self.at[1]]
            self._queue.clear()
            self.active = (self.vt[0], self.vt[1], 0.0)
        elif not on and self._brake is not None:
            self.at = [self._brake[0], self._brake[1]]
            self._brake = None

    def command(self, cmd) -> None:
        """Issue `cmd` (vx, vy, vz_up) now; it acts delay_s later."""
        last = self._queue[-1][1] if self._queue else self.active
        c = []
        for k, x in enumerate(cmd):
            x = float(x)
            if not math.isfinite(x):
                self.nonfinite_commands += 1
                x = last[k]
            c.append(x)
        self._queue.append((self.t + self.p.delay_s, tuple(c)))

    def advance(self, dt: float, trace: list | None = None) -> tuple[float, float, float]:
        """Fly `dt` seconds; the airframe's mean velocity over them. `trace`,
        when given, receives (t, dx, dy, dz) at the end of every substep,
        the displacement since this call began."""
        p = self.p
        n = max(1, int(math.ceil(dt / p.substep_s - 1e-9)))
        h = dt / n
        sx = sy = sz = 0.0
        for k in range(n):
            s = self.t + k * h
            while self._queue and self._queue[0][0] <= s + 1e-9:
                self.active = self._queue.popleft()[1]
            c = self.active
            if self._brake is not None:
                for k in (0, 1):
                    a = -BRAKE_PI_P * self.vt[k] + self._brake[k]
                    self._brake[k] -= BRAKE_PI_I * self.vt[k] * h
                    self.vt[k] += a * h
                    self.at[k] = a
            else:
                _shape_step((c[0], c[1]), self.vt, self.at, p.accel_mps2, p.jerk_mps3, h)
            _shape_step((c[2],), self.vtz, self.atz, p.accel_z_mps2, p.jerk_z_mps3, h)
            _lag(self.va, self.vt, p.tau_s, h)
            _lag(self.vaz, self.vtz, p.tau_z_s, h)
            sx += self.va[0] * h
            sy += self.va[1] * h
            sz += self.vaz[0] * h
            if trace is not None:
                trace.append((s + h, sx, sy, sz))
        self.t += dt
        return sx / dt, sy / dt, sz / dt

    def step(self, cmd, dt: float) -> tuple[float, float, float]:
        self.command(cmd)
        return self.advance(dt)


def entry_velocity(raw: Action4D, policy) -> tuple[float, float, float]:
    """The velocity a response arm's airframe ENTERS the episode with
    (VehicleResponse's `v0`): the pilot's first command, held to every
    kinematic envelope of the policy as written (speed_max_mps on the
    horizontal speed, climb_rate_max_mps on |vz|), non-finite components 0.

    The families start mid-flight - a start pose and a cruise speed, flown
    at once by the ideal vehicle - so the arm must enter at that speed, or it
    would measure a slow start from a hover instead of braking carry. Not the
    raw command as asked (a pilot asking 20 m/s under an 8 m/s cap would
    enter at a speed no earlier tick could have flown), and not the Shield's
    tick-0 repair either: a family that starts with a zone already inside
    the 5 s forecast gets a SLIDE at t = 0 (near-edge-at-12mps: 8 m/s
    sideways), and entering at the repair would have the aircraft already
    flying the Shield's reaction to a scenario that has not begun. Entering
    along the pilot's heading at the legal speed is what "cruising when the
    scenario starts" means; the Shield then reacts with that speed to carry,
    which is the point of the arm."""
    vx, vy, vz = (float(v) if math.isfinite(float(v)) else 0.0
                  for v in (raw.vx, raw.vy, raw.vz_up))
    for c in getattr(policy, "constraints", []) or []:
        if getattr(c, "type", None) != "kinematic_envelope":
            continue
        # The kinematic cap's own test (fsm.cap_exceeded; WP3-22 / X-13), with
        # no slack: the entry speed this has clamped since it was written.
        h = math.hypot(vx, vy)
        if cap_exceeded(h, c.speed_max_mps, 0.0):
            vx, vy = vx * c.speed_max_mps / h, vy * c.speed_max_mps / h
        if cap_exceeded(abs(vz), c.climb_rate_max_mps, 0.0):
            vz = math.copysign(c.climb_rate_max_mps, vz)
    return vx, vy, vz


def response_for(arm: str = DEFAULT_RESPONSE_ARM) -> dict[str, Any]:
    """The vehicle an episode's commands are flown on, as recorded in its
    extras, its episode_start event and its bundle's manifest_extras (module
    docstring, "THE VEHICLE RESPONSE"). `ideal` carries no parameters: it is
    the harness's integrator as it always was, not a VehicleResponse with
    every limit off."""
    if arm not in RESPONSE_ARMS:
        raise ValueError(f"response arm {arm!r}: one of {list(RESPONSE_ARMS)}")
    params = RESPONSES[arm]
    if params is None:
        return {"arm": arm, "model": "commanded ground velocity, every tick",
                "params": None}
    rec = {"arm": arm, "params": params.record(),
           "model": ("delayed command, jerk- and acceleration-limited target, "
                     "first-order airframe lag (sweep_scenarios.VehicleResponse)"),
           "source": "docs/DESIGN-harness-vehicle-response.md"}
    if arm == "rail":
        # The rig's own fit, entered from a hover as a rail starts (2026-11-07).
        rec.update(entry=RESPONSE_ENTRY["rail"], source=RIG_FIT["source"],
                   fitted_to=RIG_FIT["flights"])
    return rec


def forecast_for(spec: ScenarioSpec, defaults: Defaults | None = None,
                 arm: str = "grant") -> dict[str, Any]:
    """The forecast an episode's Shields are built with (module docstring,
    "THE SHIELD'S FORECAST"), as recorded in its extras and manifest_extras.

    grant  the spec's lookahead_s / lookahead_dt_s, else the library defaults
           (the grant's 5 s at 0.1 s); `grant_config` says whether the result
           IS the grant's (a spec may override it).
    3s     the labelled comparison: the spec's lookahead_s, else 3.0 s, at
           0.5 s - exactly what every Shield of the harness was built with
           until 2026-10-12. Never the grant's configuration."""
    if arm not in FORECAST_ARMS:
        raise ValueError(f"forecast arm {arm!r}: one of {list(FORECAST_ARMS)}")
    dfl = _as_defaults(defaults)
    if arm == "grant":
        look = float(spec.lookahead_s or dfl.lookahead_s)
        step = float(spec.lookahead_dt_s or dfl.lookahead_dt_s)
        grant = (abs(look - LOOKAHEAD_S) < 1e-9 and abs(step - LOOKAHEAD_DT_S) < 1e-9)
    else:
        look = float(spec.lookahead_s or LEGACY_LOOKAHEAD_S)
        step = LEGACY_LOOKAHEAD_DT_S
        grant = False
    return {"arm": arm, "lookahead_s": look, "lookahead_dt_s": step,
            "poses": int(round(look / step)), "grant_config": grant}


# --------------------------------------------------------------------------- #
# events
# --------------------------------------------------------------------------- #

# The event code moved to guardrail/scenario_events.py on 2026-11-06, so the
# hil rail's Shield node (sitl/ros2_shield_node.py --events) applies a
# scenario's events with this very code. The names below are the harness's
# own, kept: `_spawn_zone` is looked up at call time (apply_event passes it
# on), so a test that replaces it still reaches every spawn.
_spawn_zone = SE.spawn_zone
_direction = SE.direction
_window = SE.window
_EventState = SE.EventState
_zone = SE.zone_of


def apply_event(ev, es: _EventState, st: State, heading: tuple[float, float],
                t: float) -> dict:
    """Apply one ScenarioEvent to every Shield in `es.shields`, identically, and
    return the record harness_events.jsonl keeps (guardrail.scenario_events.
    apply_event: a Shield refusal raises unless the payload expects it, and an
    expected refusal that was accepted fails `event_refusals_as_expected`).
    The headless harness gives the Shield its own simulated time `t`."""
    try:
        return SE.apply_event(ev, es, st, heading, t,
                              spawn=lambda *a: _spawn_zone(*a))
    except SE.UnsupportedEvent as e:
        raise UnsupportedScenario(str(e)) from e


def _point(x: float, y: float):
    from shapely.geometry import Point
    return Point(x, y)


def _signed_dist(poly, x: float, y: float) -> float:
    """Metres from (x, y) to the polygon itself (NOT its margin ring); negative
    inside. The margin is the buffer that disturbance and sensor error are
    allowed to eat; the polygon is the line that must hold. The library's
    measure (guardrail/geometry.py signed_edge_distance, WP3-22)."""
    from guardrail.geometry import signed_edge_distance
    return signed_edge_distance(poly, x, y)


def _timed(policy) -> bool:
    """Can a rule's in-force state change during the run? A weekly window, a
    time_window_switch or a corridor_swap (each may switch with the clock or
    the event that added it): guardrail.scenario_events.rules_switch_with_clock,
    which the hil rail's Shield node asks too."""
    return SE.rules_switch_with_clock(policy)


def _map_rules(policy) -> list[str]:
    """The policy's distance_envelope rules over buildings or roads: judged
    on a map the flying Shield had (ScenarioSpec.world, --world on the
    rails), which a rail log does not carry."""
    from guardrail.models import DistanceEnvelope
    return [c.id for c in policy.by_type(DistanceEnvelope)
            if c.object_class in ("buildings", "roads")]


def switch_conservative(turned_on: bool, lag_s: float, eps: float, dt: float) -> bool:
    """Did the Shield's switch fall on the conservative side of the schedule's
    edge, within the both-sides band? `lag_s` is the Shield's switch time
    minus the edge's (tick resolution). A rule switching OFF must be held
    until at or after the edge, one switching ON enforced from at or before
    it, and either within eps (half the forecast step: the both-sides test of
    Safety Shield p2) plus one tick. models.py's minute-resolution reading
    held a 17:30 window to 17:30:59: lag +59 s, not conservative-within-band."""
    if turned_on:
        side = lag_s <= 1e-9
    else:
        side = lag_s >= -1e-9
    return side and abs(lag_s) <= eps + dt + 1e-9


def _governing_recurrences(policy) -> dict[str, list]:
    """rule id -> the weekly schedules that decide when it is in force: its own
    and those of the switches and swaps aimed at it (and each switch's own)."""
    out: dict[str, list] = {}
    for c in policy.constraints:
        rec = c.valid_time.recurrence if c.valid_time is not None else None
        if rec is None:
            continue
        out.setdefault(c.id, []).append(rec)
        tgt = getattr(c, "target_id", None)
        if tgt:
            out.setdefault(tgt, []).append(rec)
    return out


# --------------------------------------------------------------------------- #
# the headless run
# --------------------------------------------------------------------------- #

@dataclass
class EpisodeRun:
    rows: list[dict]
    extra: dict[str, Any]
    events: list[dict]
    policy: Any
    policy_hash_start: str
    sim_s: float
    wall_s: float
    seed_consumed: bool = False
    notes: list[str] = field(default_factory=list)
    # The FLYING Shield's sliding window at the end of the episode (its last
    # HISTORY_LEN filter() calls). A rail's audit writer reads this window, so
    # it must hold only what that Shield saw - which is why the ground-truth
    # oracle is a second Shield (tests/test_sweep.py checks it).
    shield_window: tuple = ()
    # Every rule's priority at the END of the episode, hot-applied zones
    # included, so a report can re-score the log exactly (tools/kpi_report.py).
    priorities: dict = field(default_factory=dict)
    # Where the policy came from (guardrail.bundle.load_for_flight's record).
    policy_source: dict = field(default_factory=dict)
    # The flying Shield's audit records (guardrail.audit.AuditRecord, one per
    # tick with a violation or an FSM event), the lines a rail writes to
    # audit.jsonl; write_bundle packs them as shield_audit.jsonl (audit card
    # WP4-20: until 2026-10-29 the harness bundles carried none).
    audit: list = field(default_factory=list)
    # The canonical IR bytes of EVERY policy generation the flying Shield held
    # (2026-11-10, WP4-05): {generation: Policy.canonical_bytes()}, the one
    # injected at t = 0 and the one each accepted event produced, read right
    # after the event (a pure read: nothing flown or scored depends on it).
    # The grant's policy_bundle.tar.gz holds "all hot-apply generations";
    # until this date a run kept the first and the last policy only, and the
    # ones between were in a bundle by hash alone.
    generations: dict = field(default_factory=dict)


def _as_spec(sc) -> ScenarioSpec:
    return sc if isinstance(sc, ScenarioSpec) else ScenarioSpec.model_validate(sc)


def _as_defaults(d) -> Defaults:
    if isinstance(d, Defaults):
        return d
    return Defaults.model_validate(d or {})


def episode_ground(spec, policy=None):
    """The scenario's declared ground (ScenarioSpec.ground, X-11,
    guardrail/terrain.py load_ground) in the frame of its policy, or None
    for a scenario without one. Raises UnsupportedScenario for a ground file
    that cannot be read or placed in the policy's frame. A flat ground is
    refused here: the harness's heights are already above a flat ground at
    the start, so a flat `ground` would change nothing it flies (an MSL rule
    over a flat ground is the policy shifted by the elevation)."""
    path = getattr(spec, "ground", None)
    if path is None:
        return None
    from guardrail import terrain as TR
    try:
        pol = policy if policy is not None else spec.policy.load(ROOT)
        g = TR.load_ground(path, pol.frame_origin, root=ROOT)
    except (ValueError, OSError, KeyError) as e:
        raise UnsupportedScenario(f"ground {path}: {type(e).__name__}: {e}") from e
    if not TR.is_dem(g):
        raise UnsupportedScenario(f"ground {path}: a flat ground; the harness's heights "
                                  f"are above the start's ground already")
    return g


def episode_world(spec):
    """The scenario's declared world (ScenarioSpec.world, guardrail/
    world_map.py), read from the checkout, or None for a scenario without
    one. Raises UnsupportedScenario for a world file that cannot be read, for
    a scenario policy that cannot be loaded to check against it, or for a
    world that lacks a map one of the policy's distance_envelope rules needs
    (the Shield would refuse the policy without it)."""
    path = getattr(spec, "world", None)
    if path is None:
        return None
    from guardrail.world_map import load_world, missing_maps
    try:
        world = load_world(path, ROOT)
    except (OSError, ValueError) as e:
        raise UnsupportedScenario(f"world {path}: {type(e).__name__}: {e}") from e
    # A policy that cannot be loaded is refused here too, with its reason:
    # the world/policy check below cannot be skipped, so a mismatch is never
    # first met by the Shield mid-episode or by the node after SITL boots.
    try:
        pol = spec.policy.load(ROOT)
    except Exception as e:                               # noqa: BLE001
        raise UnsupportedScenario(f"world {path}: the scenario's policy cannot be "
                                  f"loaded to check it against the world: "
                                  f"{type(e).__name__}: {e}") from e
    gaps = missing_maps(pol, world)
    if gaps:
        raise UnsupportedScenario(f"world {path}: " + "; ".join(gaps))
    return world


def run_episode(sc, defaults=None, seed: int = 0, arm: dict | None = None, *,
                forecast: str = "grant", wind_model: str = DEFAULT_WIND_MODEL,
                autopilot_hold: str = DEFAULT_AUTOPILOT_HOLD,
                response: str = DEFAULT_RESPONSE_ARM,
                recovery: str = DEFAULT_RECOVERY_LAW,
                entry: str | None = None,
                gps_dropout: str = DEFAULT_GPS_DROPOUT,
                action_frame: str = "world") -> EpisodeRun:
    """One headless episode. `arm` (paraphrase / prefix record) is RECORDED in
    the episode_start event and changes nothing else: no headless pilot reads
    text - unless it names a prefix consumer (`pilot_reads_prefix`, module
    docstring THE PREFIX CONSUMER), whose pilot is then flown through the
    cell's CSP on the prefix-on arm. `forecast` (forecast_for), `wind_model` (WIND_MODELS),
    `autopilot_hold` (AUTOPILOT_HOLD_MODELS), `response` (RESPONSE_ARMS,
    response_for), `recovery` (the Shield's recovery_law, RECOVERY_LAWS), on a
    response arm `entry` (ENTRY_STATES; None: the arm's own, RESPONSE_ENTRY),
    and `gps_dropout` (GPS_DROPOUT_MODELS: how a declared GPS denial reaches
    the Shield) DO change the flight. Flown by EpisodeRunner without a
    pause, so a stepped episode and this one share one integrator.

    `action_frame` (ACTION_FRAMES, 2026-11-17, WP3-04 / ARCH-04). On both
    arms (since wf21) the Shield is handed the pilot's action in the BODY
    frame at the heading it is given (frames.filter_world_as_body) and hands
    back the emitted BODY action; what the rail streams (that action, a
    stop on the FSM's brake, the pilot's own body action with the Shield
    off) is turned into local NED only after the projection,
    as the rails' MAVLink adapter does (frames.body_to_local_ned at the
    aircraft's heading), and the integrator flies that. "world" (the
    default) keeps the Shield's arc forecast off; "body" turns it on and is
    recorded in the episode_start event. The headless vehicle does not model
    yaw, so the heading stays the start's and every pilot here commands no
    yaw rate. Against the world-frame hand-over flown before wf21 the
    episodes differ by the rotations' rounding only, at headings that are
    not a whole number of quarter turns (tests/test_frame_boundary.py)."""
    return EpisodeRunner(sc, defaults, seed, arm, forecast=forecast,
                         wind_model=wind_model, autopilot_hold=autopilot_hold,
                         response=response, recovery=recovery, entry=entry,
                         gps_dropout=gps_dropout, action_frame=action_frame).finish()


class EpisodeRunner:
    """run_episode one tick at a time, with events injected between ticks.

        r = EpisodeRunner(spec, defaults, seed)
        while r.t is not None and r.t < 5.0:
            r.step()                         # fly one tick
        r.inject(event)                      # a ScenarioEvent (or its dict)
        run = r.finish()                     # the EpisodeRun run_episode returns

    `t` is the simulated time of the next tick to fly (None once the episode
    has ended). An injected event joins the schedule exactly as if it had been
    the LAST entry of the scenario's `events:` list: ordered by at_sim_t,
    after any event already scheduled for the same instant, and applied by the
    same apply_event at that tick. So an episode stepped with injections flies
    the same rows as run_episode on the scenario with those events written in
    (tests/test_sweep.py checks both, with a null). It is the hook a live
    control plane needs (guardrail/harness_service.py still schedules its
    events for EndEpisode; docs/DESIGN-harness-grpc.md).

    Refused, as run_episode refuses them in a scenario: an event before `t`
    (that tick has flown), one after the last tick (it would never fire), and
    one with a `window` when the mission has no clock. `wall_s` in the result
    includes the caller's pauses between steps.

    AN EPISODE THAT RAISES IS OVER. An error inside a tick (a Shield refusal
    of an injected event, as run_episode would raise it for the same event
    written in) propagates from that step() or finish(); the runner is then
    `done` with `t` None and no result, and every later step(), inject() or
    finish() raises RuntimeError naming the first error. Before 2026-10-12 the
    dead generator left `done` False for ever, so finish() spun without end.
    """

    def __init__(self, sc, defaults=None, seed: int = 0, arm: dict | None = None, *,
                 forecast: str = "grant", wind_model: str = DEFAULT_WIND_MODEL,
                 autopilot_hold: str = DEFAULT_AUTOPILOT_HOLD,
                 response: str = DEFAULT_RESPONSE_ARM,
                 recovery: str = DEFAULT_RECOVERY_LAW,
                 entry: str | None = None,
                 gps_dropout: str = DEFAULT_GPS_DROPOUT,
                 action_frame: str = "world"):
        self._box: dict[str, Any] = {}
        self._gen = _episode(sc, defaults, seed, arm, self._box, forecast, wind_model,
                             autopilot_hold, response, recovery, entry, gps_dropout,
                             action_frame)
        self.result: EpisodeRun | None = None
        self.error: BaseException | None = None
        self._failed_t: float | None = None     # the tick that raised
        self._next: tuple[int, float] | None = None
        self._advance()                      # set up, up to the start of tick 0

    def _advance(self) -> None:
        flying = None if self._next is None else self._next[1]
        try:
            self._next = next(self._gen)
        except StopIteration as stop:
            self._next = None
            if stop.value is None:
                # Only a generator that already died returns nothing; _episode
                # itself always returns an EpisodeRun.
                self.error = self.error or RuntimeError(
                    "the episode's integrator stopped without a result")
                self._failed_t = flying
                raise self._failed() from None
            self.result = stop.value
        except BaseException as exc:
            self._next, self.error, self._failed_t = None, exc, flying
            raise

    def _failed(self) -> RuntimeError:
        at = self._failed_t
        return RuntimeError(
            f"the episode failed{'' if at is None else f' in the tick at {at:.1f} s'}: "
            f"{type(self.error).__name__}: {self.error}")

    def _check_alive(self) -> None:
        if self.error is not None:
            raise self._failed() from self.error

    @property
    def done(self) -> bool:
        return self.result is not None or self.error is not None

    @property
    def t(self) -> float | None:
        return None if self._next is None else self._next[1]

    @property
    def t_end(self) -> float:
        return self._box["t_end"]

    def step(self) -> bool:
        """Fly the next tick. False once the episode has ended (the last tick,
        or a terminal fail-safe), and a no-op after that. Raises what the tick
        raised, then RuntimeError on every later call (class docstring)."""
        self._check_alive()
        if not self.done:
            self._advance()
        return not self.done

    def inject(self, event) -> None:
        self._check_alive()
        ev = event if isinstance(event, ScenarioEvent) else ScenarioEvent.model_validate(event)
        if self.done:
            raise RuntimeError("the episode has ended: nothing more can be injected")
        if ev.at_sim_t < self._next[1] - 1e-9:
            raise ValueError(f"event at {ev.at_sim_t} s is in the past: the next "
                             f"tick to fly is {self._next[1]:.1f} s")
        if ev.at_sim_t > self._box["t_end"] + 1e-9:
            raise ValueError(f"event at {ev.at_sim_t} s never fires: the run "
                             f"ends at {self._box['t_end']:.1f} s")
        if "window" in ev.payload and not self._box["has_clock"]:
            raise ValueError(f"{ev.type} with a `window` needs mission.clock_start: "
                             f"with no clock the window is always in force")
        pending = self._box["pending"]
        k = len(pending)
        while k > 0 and pending[k - 1].at_sim_t > ev.at_sim_t:
            k -= 1
        pending.insert(k, ev)

    def finish(self) -> EpisodeRun:
        self._check_alive()
        while not self.done:
            self._advance()
        return self.result


def _episode(sc, defaults, seed: int, arm: dict | None, box: dict,
             forecast: str = "grant", wind_model: str = DEFAULT_WIND_MODEL,
             autopilot_hold: str = DEFAULT_AUTOPILOT_HOLD,
             response: str = DEFAULT_RESPONSE_ARM,
             recovery: str = DEFAULT_RECOVERY_LAW,
             entry: str | None = None,
             gps_dropout: str = DEFAULT_GPS_DROPOUT,
             action_frame: str = "world"):
    """The integrator behind run_episode and EpisodeRunner: a generator that
    yields (tick, t) at the start of every tick, before that tick's scheduled
    events are applied, and returns the EpisodeRun. `box` receives the
    schedule (`pending`, sorted by at_sim_t), `t_end` and `has_clock`, so
    EpisodeRunner can add to the schedule between ticks."""
    spec = _as_spec(sc)
    dfl = _as_defaults(defaults)
    dt = float(spec.dt or dfl.dt)
    ticks = int(spec.ticks or dfl.ticks)
    fc = forecast_for(spec, dfl, forecast)
    look, step = fc["lookahead_s"], fc["lookahead_dt_s"]
    if wind_model not in WIND_MODELS:
        raise ValueError(f"wind model {wind_model!r}: one of {list(WIND_MODELS)}")
    if autopilot_hold not in AUTOPILOT_HOLD_MODELS:
        raise ValueError(f"autopilot hold model {autopilot_hold!r}: one of "
                         f"{list(AUTOPILOT_HOLD_MODELS)}")
    if recovery not in RECOVERY_LAWS:
        raise ValueError(f"recovery law {recovery!r}: one of {list(RECOVERY_LAWS)}")
    if action_frame not in ACTION_FRAMES:
        raise ValueError(f"action frame {action_frame!r}: one of {list(ACTION_FRAMES)}")
    body_frame = action_frame == "body"
    # Every Shield of the episode on the same recovery law; passed only when
    # it is not the default, as the hold model is below, so a caller's own
    # Shield factory keeps its own.
    rec_kw = {} if recovery == DEFAULT_RECOVERY_LAW else {"recovery_law": recovery}
    # The scenario's declared world (ScenarioSpec.world, WP1-07): its
    # buildings and roads maps go to every Shield of the episode, so each
    # judges the distance_envelope on the same map. Nothing without a world.
    world = episode_world(spec)
    if world is not None:
        rec_kw.update(world.shield_kwargs())
    # The vehicle the commands are flown on (module docstring, "THE VEHICLE
    # RESPONSE"). `rsp` is None for the ideal vehicle, and then nothing below
    # differs from the integrator as it was before the arm existed: every
    # branch the arm adds is behind `veh is not None` / `rsp is not None`.
    rsp_rec = response_for(response)
    rsp = RESPONSES[response]
    if entry is not None and rsp is None:
        raise ValueError(f"entry {entry!r} needs a response arm: the ideal vehicle "
                         f"flies every command at once")
    entry = entry if entry is not None else RESPONSE_ENTRY.get(response, DEFAULT_ENTRY)
    if entry not in ENTRY_STATES:
        raise ValueError(f"entry {entry!r}: one of {list(ENTRY_STATES)}")
    if rsp is not None and (entry != DEFAULT_ENTRY or "entry" in rsp_rec):
        # Recorded off the mid-flight entry, and always on an arm whose record
        # names one (the rail arm's), so every other episode's record is what
        # it was.
        rsp_rec = dict(rsp_rec, entry=entry)
    veh: VehicleResponse | None = None

    t_end = (ticks - 1) * dt
    late = [e for e in spec.events if e.at_sim_t > t_end + 1e-9]
    if late:
        raise ValueError(f"event at {late[0].at_sim_t} s never fires: the run "
                         f"ends at {t_end:.1f} s")
    windowed = [e for e in spec.events if "window" in e.payload]
    if windowed and spec.mission.clock_start is None:
        # With no clock every rule is in force (ConstraintBase.active_at), so a
        # "scheduled" switch would act from the event on: an unscheduled switch
        # under another name.
        raise ValueError(f"{windowed[0].type} with a `window` needs "
                         f"mission.clock_start: with no clock the window is "
                         f"always in force")
    drop = spec.stress.gps_dropout_s
    if gps_dropout not in GPS_DROPOUT_MODELS:
        raise ValueError(f"gps dropout model {gps_dropout!r}: one of {list(GPS_DROPOUT_MODELS)}")
    if drop is not None and drop[0] > t_end + 1e-9:
        # Same reasoning as a late event: a dropout that starts after the last
        # tick would leave a "GPS denial" scenario that never denied anything.
        raise ValueError(f"gps_dropout_s starts at {drop[0]} s and never "
                         f"happens: the run ends at {t_end:.1f} s")
    batt_at = spec.stress.battery_critical_at_s
    if batt_at is not None and batt_at > t_end + 1e-9:
        # Same reasoning: a "low battery" cell whose battery never went low.
        raise ValueError(f"battery_critical_at_s is {batt_at} s and never "
                         f"happens: the run ends at {t_end:.1f} s")

    policy, source = spec.policy.load_with_source(ROOT)
    # The scenario's declared ground (ScenarioSpec.ground, X-11): every
    # Shield of the episode converts its MSL rules over it, told the height
    # above that ground (up_ref="terrain"), which is what `st.up` is below
    # (the integrator keeps it so). Nothing without a ground.
    ground = episode_ground(spec, policy)
    if ground is not None:
        rec_kw.update(ground=ground, up_ref="terrain")
    hash0 = policy.policy_hash
    sim_t = [0.0]
    clock = None
    if spec.mission.clock_start is not None:
        t0 = spec.mission.clock_start
        clock = lambda: t0 + timedelta(seconds=sim_t[0])          # noqa: E731
    on = spec.shield
    # The FSM's verdict is flown only by the Shield arm with escalation on. The
    # control arm flies the raw action; its Shield runs without an FSM.
    esc = on and spec.escalation
    # Every Shield of the episode on one forecast: the grant's 5 s at 0.1 s
    # unless the spec overrides it, or the labelled 3 s comparison arm
    # (forecast_for). Until 2026-10-12 all three were built at 3 s / 0.5 s.
    # The hold model goes in only when it is not the default, so a caller
    # that hands in its own Shield factory (tests patch `Shield` with a
    # functools.partial) keeps its own hold_points.
    shield = Shield(policy, lookahead_s=look, dt=step, now=clock, escalation=esc,
                    **({} if autopilot_hold == DEFAULT_AUTOPILOT_HOLD
                       else {"hold_points": False}), **rec_kw)
    from guardrail.audit import AuditLogger
    auditor = AuditLogger.in_memory(shield.policy)
    audit: list[dict] = []
    # What the modelled autopilot flies for the FSM's Loiter / Land
    # (AUTOPILOT_HOLD_MODELS), as this Shield will plan it; None with no FSM.
    hold_model = (("moved_points" if getattr(shield, "hold_points", True) else "in_place")
                  if esc else None)
    # The recovery law the flying Shield holds (a caller's factory may set it).
    recovery_law = getattr(shield, "recovery_law", recovery)

    stress = spec.stress
    # Ground truth under degraded perception comes from a SECOND Shield kept in
    # step with the first (same rules, same events, same clock, same subject).
    # It is asked about the TRUE state. Re-using the flying Shield would put
    # the oracle's queries into its sliding window (`history`), which the
    # Shield keeps for its own decisions.
    oracle = (Shield(spec.policy.load(ROOT), lookahead_s=look, dt=step, now=clock,
                     escalation=False, **rec_kw)
              if stress.degrades_perception else None)
    # When the autopilot flies (LOITER / RTL / LAND), what it flew is checked
    # by a third Shield, never by the flying one (whose FSM it would feed).
    checker = (Shield(spec.policy.load(ROOT), lookahead_s=look, dt=step, now=clock,
                      escalation=False, **rec_kw) if esc else None)
    # Under the vehicle-response arm only: a fourth Shield, kept in step like
    # the others, that judges the velocity the AIRFRAME flew over each tick
    # at the true position (`true_p0_vehicle_ticks`). The true-state P0 count
    # above judges the COMMAND, before the vehicle steps; braking carry - a
    # command of zero while the airframe still moves toward the zone - is
    # invisible to it (review of T1b-harness-vehicle, P3). Its own Shield,
    # because a Shield's filter() feeds its sliding window and monitor clock,
    # and a second query per tick to the oracle or the checker would change
    # what they keep. None on the ideal vehicle, whose airframe velocity IS
    # the command, so the ideal episode builds, keys and steps exactly what
    # it did.
    vjudge = (Shield(spec.policy.load(ROOT), lookahead_s=look, dt=step, now=clock,
                     escalation=False, **rec_kw) if rsp is not None else None)
    rng_gps = random.Random(f"{seed}:gps")
    rng_wind = random.Random(f"{seed}:wind")
    rng_jit = random.Random(f"{seed}:jitter")

    m = _resolve_mission(spec, policy, rng_jit)
    wind = (0.0, 0.0)
    if stress.wind_speed_mps > 0:
        # A declared direction is in the mission's frame and turns with it; a
        # drawn one is absolute (see scenario_spec.Stress).
        wdir = (stress.wind_dir_deg + spec.mission.rotate_deg
                if stress.wind_dir_deg is not None
                else rng_wind.uniform(0.0, 360.0))
        wind = (stress.wind_speed_mps * math.cos(math.radians(wdir)),
                stress.wind_speed_mps * math.sin(math.radians(wdir)))
    sigma = stress.gps_noise_m

    kind = spec.mission.pilot.type
    # THE PREFIX CONSUMER (module docstring): an arm record that names one
    # flies the pilot law through it - with the cell's CSP on the prefix-on
    # arm, untouched on the prefix-off arm. None for every other episode,
    # whose pilot is called exactly as before.
    consumer = None
    if arm and arm.get("pilot_reads_prefix"):
        consumer = arm_consumer(
            spec, policy, dfl, seed, arm,
            lambda s: _pilot(m, kind, s, None if rsp is None else rsp.accel_mps2))
    subj = spec.mission.subject
    relabels = sorted(subj.reclassify if subj else [], key=lambda r: r.at_s)
    subj_class = subj.class_ if subj else None
    shields = [s for s in (shield, oracle, checker, vjudge) if s is not None]
    truth = oracle or shield                     # the Shield asked about the TRUE state
    if subj:
        for s in shields:
            s.set_subject(m.subject[0], m.subject[1], subj_class)

    priorities = K.rule_priorities(policy)

    def p0(vios) -> bool:
        return any(priorities.get(v.rule_id, "P0") == "P0" for v in vios)

    pending = sorted(spec.events, key=lambda e: e.at_sim_t)
    box.update(pending=pending, t_end=t_end,
               has_clock=spec.mission.clock_start is not None)
    # The DSL's lint WARNINGS for this cell, as the compiler would give them
    # at mission start (guardrail/compiler.py, time_window_lead_warnings):
    # a window that can open on an aircraft that cannot leave in time, at the
    # episode's own forecast, from its start pose and clock. Recorded, never
    # gated: the opens-on-vehicle cells carry one by design.
    from guardrail.compiler import time_window_lead_warnings
    lint_warnings = time_window_lead_warnings(
        policy, lookahead_s=look, start=(m.start.x, m.start.y, m.start.up),
        now=clock() if clock is not None else None)
    events: list[dict] = [{
        "t": 0.0, "type": "episode_start", "scenario_id": spec.scenario_id,
        "seed": seed, "policy_hash": hash0, "generation": policy.generation,
        "policy_source": source,
        "shield": "on" if on else "off",
        "escalation": "on" if esc else "off",
        "task_prompt": spec.mission.task_prompt or None,
        "task_prompt_read_by": None if not spec.mission.task_prompt else
        "nobody: the headless pilot is not a language model",
        "arm": arm,
        "forecast": fc,
        "wind_model": wind_model,
        "vehicle_response": rsp_rec,
        "wind_mps": [round(wind[0], 3), round(wind[1], 3)],
        "lint_warnings": lint_warnings,
        "autopilot_hold_model": hold_model,
        "recovery_law": recovery_law}]
    if stress.degrades_perception:
        # What the modelled rail tells the Shield about its position and the
        # dead reckoning's drift allowance (2026-11-06): only where perception
        # is degraded, so every other episode's events are as they were.
        events[0]["fix_quality_reported"] = stress.fix_quality_reported
        events[0]["dr_drift_mps"] = getattr(shield, "dr_drift_mps", None)
    if drop is not None and gps_dropout != DEFAULT_GPS_DROPOUT:
        # How the denial reaches the Shield (GPS_DROPOUT_MODELS, 2026-11-12):
        # only off the default model, so every frozen-fix episode's events
        # are as they were.
        events[0]["gps_dropout"] = gps_dropout_for(gps_dropout)
    if body_frame:
        # Only off the default frame, so every world-frame episode's events
        # are as they were.
        events[0]["action_frame"] = action_frame
    if ground is not None:
        # Only over a declared ground (X-11), so every other episode's
        # events are as they were.
        events[0]["ground"] = ground.record()
        events[0]["up_ref"] = "terrain"
    es = _EventState(shields=shields)
    generation_irs: dict[int, bytes] = {int(policy.generation): policy.canonical_bytes()}
    st = m.start
    prev_st = st
    hist: list[State] = []
    rows: list[dict] = []
    max_speed = 0.0
    min_range = math.inf
    max_offset = None
    final_offset = None
    finite = True
    min_by_class: dict[str, float] = {}
    range_at_relabel = None
    prev_active = None
    prev_ids: set[str] = set()
    prev_unsafe = False
    moved_into_unsafe = moved_into_polygon = False
    breaches = 0
    autopilot_breaches = 0
    autopilot_poly_entries = 0
    prev_owner = "shield"
    true_p0_ticks = 0
    autopilot_p0_ticks = 0
    # The vehicle-response arm's airframe-velocity count (`vjudge`); never
    # reported for the ideal vehicle.
    true_p0_vehicle_ticks = 0
    min_fence_d = math.inf
    fence_seen = False
    inside_poly_ticks = 0
    poly_entries = 0
    prev_inside = False
    phase = 0
    phase_vios: list[dict[str, int]] = [{}]
    heading = (1.0, 0.0)
    if m.target is not None:
        hx, hy = m.target[0] - st.x, m.target[1] - st.y
        if math.hypot(hx, hy) > 1e-6:
            heading = (hx / math.hypot(hx, hy), hy / math.hypot(hx, hy))
    else:
        r = math.radians(st.yaw_deg)
        heading = (math.cos(r), math.sin(r))
    frozen: State | None = None
    dropout_ticks = 0
    # The dead-reckoned denial (gps_dropout "dead_reckoned"): when the fix was
    # lost and the ground-track direction the estimate lags along.
    drop_t0: float | None = None
    drop_dir = (1.0, 0.0)
    # The escalation FSM's hand-over, and the autopilot that flies it.
    ap_mode: str | None = None
    # Where the FSM's Loiter / Land holds (ShieldDecision.hold_point), whether
    # a moved point has been reached, and every point the FSM took.
    hold: dict | None = None
    hold_done = False
    holds_taken: list[dict] = []
    # When to give up a moved hold's transit, as the rails do (the rail arm
    # only, TRANSIT_GIVE_UP_RESPONSES): one TransitWatch per moved plan.
    give_up_transits = response in TRANSIT_GIVE_UP_RESPONSES
    hold_watch: "HF.TransitWatch | None" = None
    # The rail arm's Loiter, flown by the rails' own HoldFlight (RAIL_HOLD_MODEL):
    # a GUIDED hold, or the old transit-then-BRAKE flow with BRAKE's carry.
    if RAIL_HOLD_MODEL not in RAIL_HOLD_MODELS:
        raise ValueError(f"RAIL_HOLD_MODEL {RAIL_HOLD_MODEL!r}: one of {list(RAIL_HOLD_MODELS)}")
    hold_flight = (HF.HoldFlight(guided_hold=RAIL_HOLD_MODEL == "guided")
                   if give_up_transits and esc and rsp is not None else None)
    hold_tick = None
    ap_brake = False
    # X9's input (since 2026-11-11): how far the aircraft is from the point
    # its Loiter holds, and how far a kept hold carries it - the rails' own
    # HoldWatch, given the ground velocity the last tick flew.
    kept_watch = HF.HoldWatch() if esc else None
    ground_v: tuple[float, float] | None = None
    owner_ticks = {"shield": 0, "autopilot": 0, "pilot": 0}
    ap_mode_ticks: dict[str, int] = {}
    ap_p0_by_mode: dict[str, int] = {}
    terminated_s = None
    # The RTL hand-over, judged each RTL tick (guardrail/rtl_path.py).
    rtl_watch = (RP.RTLWatch(home=m.home, rtl_alt_m=AP_RTL_ALT_M,
                             battery=RP.BatteryHook(critical_at_s=batt_at))
                 if esc else None)
    # RTL_ALT is measured above HOME (the start). Over a declared ground
    # (X-11) `st.up` is the height above the ground under the aircraft, so
    # the RTL model, home_reached and the RTL watch are handed the height
    # above home beside it (wf20; the ROS 2 node hands its pose's). None
    # without a ground: every other episode reads st.up as before.
    g_home = ground.elevation_at(*m.home) if ground is not None else None

    def up_above_home(s: State) -> float | None:
        if ground is None:
            return None
        return s.up + ground.elevation_at(s.x, s.y) - g_home
    rtl_failed_ticks = 0
    rtl_failures: list[dict] = []
    # The gust offset from the position target (wind model "guided").
    gust_off = [0.0, 0.0]
    # The vehicle-response arm's own measurements: the fastest the airframe
    # flew, and the largest gap between what it flew and what it was told
    # (a response arm that never differed from the command would otherwise
    # be indistinguishable from one that never ran).
    veh_max_speed = 0.0
    veh_max_lag = 0.0
    # The arm's RTL return altitude (_autopilot `rtl_alt`); None until an RTL
    # is entered under a response, and always None on the ideal vehicle.
    rtl_alt: float | None = None
    # The t- / t+ boundary measurements (module docstring).
    req_state: dict[int, bool] = {}
    req_edges: dict[str, list[float]] = {}
    switch_flips: dict[str, list[tuple[float, bool]]] = {}
    anticipated: dict[str, int] = {}
    opening_only: dict[str, int] = {}
    window_opened_ticks = 0
    # Ticks whose decision refused the escape floor at the speed cap (a
    # slower escape flown) or refused it outright (the action flown
    # unfloored, or reported to the FSM): Shield._floor_the_escape's trace,
    # `cap_refused` / `floor_refused` (review of 69c27bd, P3).
    cap_refused_ticks = floor_refused_ticks = 0
    last_violation_t: dict[str, float] = {}
    release_gap: dict[str, float] = {}
    timed_rules: set[str] = set()
    wall0 = time.perf_counter()

    for i in range(ticks):
        # EpisodeRunner's pause point: the caller may add to `pending` here,
        # before this tick's scheduled events are applied.
        yield i, i * dt
        t_now = i * dt
        sim_t[0] = t_now
        imposed = i == 0
        # ---- scheduled harness events -----------------------------------
        while pending and pending[0].at_sim_t <= t_now + 1e-9:
            ev = pending.pop(0)
            if consumer is not None:
                # Reached only by an event injected into a stepped episode: a
                # scenario that declares events is refused before take-off
                # (prefix_consumer_refusal).
                raise ValueError(f"{ev.type} at {ev.at_sim_t:g} s: the prefix consumer "
                                 f"flies the CSP compiled before take-off, and an event "
                                 f"changes the rules under it (generation N's CSP "
                                 f"reaching the pilot is not built: audit card WP4-19)")
            rec = apply_event(ev, es, st, heading, t_now)
            for rid, prio in K.rule_priorities(shield.policy).items():
                priorities.setdefault(rid, prio)
            events.append(rec)
            if not rec.get("refused") and rec.get("generation") is not None:
                # EpisodeRun.generations: the IR this event produced.
                generation_irs[int(rec["generation"])] = (
                    es.shields[0].policy.canonical_bytes())
            if not rec.get("refused"):
                imposed = True
        if subj:
            due = [r for r in relabels if r.at_s <= t_now + 1e-9]
            new_class = due[-1].class_ if due else subj.class_
            if new_class != subj_class:
                subj_class = new_class
                for s in shields:
                    s.set_subject(m.subject[0], m.subject[1], subj_class)
                if range_at_relabel is None:
                    range_at_relabel = math.hypot(st.x - m.subject[0],
                                                  st.y - m.subject[1])
                events.append({"t": round(t_now, 4), "type": "reclassify",
                               "class": subj_class})
                imposed = True

        # ---- what the Shield sees ------------------------------------------
        hist.append(st)
        seen = hist[max(0, i - stress.latency_ticks)]
        if stress.degrades_perception:
            seen = State(x=seen.x + (rng_gps.gauss(0, sigma) if sigma else 0.0),
                         y=seen.y + (rng_gps.gauss(0, sigma) if sigma else 0.0),
                         up=seen.up + (rng_gps.gauss(0, sigma) if sigma else 0.0),
                         yaw_deg=seen.yaw_deg,
                         # The accuracy a receiver reports with the fix
                         # (GPS_RAW_INT.h_acc), when the modelled rail reports
                         # it (Stress.fix_quality_reported, 2026-11-06).
                         pos_sigma_m=(sigma if sigma and stress.fix_quality_reported
                                      else None))
        if drop is not None:
            # GPS denial (GPS_DROPOUT_MODELS): "frozen" holds the last fix;
            # "dead_reckoned" hands on the autopilot's estimate, the pose
            # lagging the true one by RAIL_EKF_DRIFT_MPS x the time since the
            # loss along the ground track at the loss (as EKF3 does on the
            # rails). The noise above is still drawn on these ticks, so the
            # noise sequence after the window is the same one a run without
            # the dropout would see on the same seed.
            if drop[0] - 1e-9 <= t_now < drop[1] - 1e-9:
                if frozen is None:
                    frozen = seen
                    drop_t0 = t_now
                    gv = ground_v if ground_v is not None else heading
                    gn = math.hypot(gv[0], gv[1])
                    if gn < 0.1:
                        gv, gn = heading, math.hypot(heading[0], heading[1])
                    drop_dir = (gv[0] / gn, gv[1] / gn)
                    events.append({"t": round(t_now, 4), "type": "gps_dropout_start",
                                   "held": [round(seen.x, 3), round(seen.y, 3),
                                            round(seen.up, 3)],
                                   **({} if gps_dropout == DEFAULT_GPS_DROPOUT
                                      else {"model": gps_dropout})})
                pose = frozen
                quality = {"fix_ok": False}
                if gps_dropout == "dead_reckoned":
                    lag = RAIL_EKF_DRIFT_MPS * (t_now - drop_t0)
                    pose = seen.model_copy(update={"x": seen.x - lag * drop_dir[0],
                                                   "y": seen.y - lag * drop_dir[1]})
                    # The autopilot vouches for its own estimate until its
                    # EKF failsafe would declare it lost (RAIL_EKF_POS_VALID_S,
                    # 2026-11-12), as the ROS 2 node reports it.
                    quality["estimator_ok"] = (t_now - drop_t0
                                               < RAIL_EKF_POS_VALID_S - 1e-9)
                # A rail that reports fix quality flags the pose as not a
                # current fix (State.fix_ok False: the pose's stamp age,
                # GPS_RAW_INT.fix_type); one that does not hands it on as is
                # (Stress.fix_quality_reported, 2026-11-06).
                seen = (pose.model_copy(update=quality)
                        if stress.fix_quality_reported else pose)
                dropout_ticks += 1
            elif frozen is not None:
                frozen = None
                drop_t0 = None
                events.append({"t": round(t_now, 4), "type": "gps_dropout_end"})

        raw = (consumer.act(st) if consumer is not None
               else _pilot(m, kind, st, None if rsp is None else rsp.accel_mps2))
        if rsp is not None and veh is None:
            veh = VehicleResponse(rsp, v0=(entry_velocity(raw, policy) if entry == "cruise"
                                           else (0.0, 0.0, 0.0)), t0=t_now)
        flags = {}
        rtl_note = None
        if esc:
            # What the autopilot reports this tick, for the FSM's terminal
            # edges (G9 home reached, G10 landed). Home is reached at the end
            # of RTL's return leg: over the launch point, after the climb.
            # The height above home (up_above_home): st.up without a ground.
            uh = st.up if ground is None else up_above_home(st)
            flags = {"home_reached": ap_mode == "RTL"
                     and RP.home_reached(st.x, st.y, uh, m.home, tol_m=AP_HOME_TOL_M,
                                         rtl_alt_m=AP_RTL_ALT_M),
                     "landed": ap_mode == "LAND" and st.up <= AP_LANDED_M}
            # G6, RTL -> Land: the straight leg home blocked by a P0 zone or
            # outside the band, no progress home over T, or a critical
            # battery (guardrail/rtl_path.py). Judged on the true position,
            # which is the autopilot's own (it flies on its estimate).
            if ap_mode == "RTL":
                if ground is None:
                    v = rtl_watch.assess(t_now, st.x, st.y, st.up,
                                         RP.zones_from_shield(checker),
                                         RP.band_from_shield(checker, st),
                                         up_home=up_above_home(st))
                else:
                    # Over a declared ground (X-11) the leg home is judged
                    # along the ground under it (rtl_path, THE LEG OVER A
                    # DEM), each rule in its own reference.
                    zs, bs, over = RP.leg_rules_from_shield(checker, st)
                    v = rtl_watch.assess(t_now, st.x, st.y, st.up, zs, bs,
                                         up_home=up_above_home(st), over=over)
                flags["rtl_failed"] = v.failed
                if v.failed:
                    rtl_note = v.note
                    rtl_failed_ticks += 1
                    if len(rtl_failures) < 5:
                        rtl_failures.append({"t": round(t_now, 4),
                                             "reasons": list(v.reasons)})
            else:
                # Defensive: the FSM leaves RTL only for Land or the end
                # (G6, G9, X5), so an RTL is never re-entered today.
                rtl_watch.reset()
            # X9, the hold kept (guardrail/hold_flight.py HoldWatch): judged
            # on the true position, as the rails' TransitWatch is here - the
            # autopilot's own, which flies the hold.
            off, reach = kept_watch.judge(
                shield.fsm, hold_flight, st.x, st.y, t_now,
                None if ground_v is None else math.hypot(ground_v[0], ground_v[1]))
            if off is not None:
                flags["hold_off_m"], flags["hold_reach_m"] = off, reach
        # The Shield's input is the BODY frame on every tick (WP3-04 /
        # ARCH-04): the pilot's world action turned into the body frame here,
        # at the heading the Shield is given (frames.filter_world_as_body).
        # The default arm ("world": the pilot speaks the world frame) keeps
        # the arc forecast off; "body" turns it on (run_episode
        # `action_frame`). What the Shield streams is turned back into the
        # integrator's frame only by the adapter below, after the projection.
        d, body_in, emitted_body = filter_world_as_body(
            shield, seen, raw, t=t_now, arc=bool(body_frame), **flags)
        a_rec = auditor.record(i, d)
        if a_rec is not None:
            audit.append(auditor.line(a_rec))
        if esc and d.set_mode:
            if rsp is not None and d.set_mode == "RTL" and ap_mode != "RTL":
                # The arm's RTL return altitude, captured on entry as
                # ArduPilot computes it (_autopilot, `rtl_alt`), above home.
                rtl_alt = max(AP_RTL_ALT_M, st.up if ground is None else up_above_home(st))
            ap_mode = d.set_mode
        if esc:
            # The FSM's hold point: None outside Loiter / Land (or on a fault,
            # where the modelled autopilot holds in place as before); a new
            # plan (its own t_s) restarts the transit.
            hp = d.hold_point
            if hp is None:
                hold = None
            elif hold is None or hp.get("t_s") != hold.get("t_s"):
                hold, hold_done = hp, False
                hold_watch = (HF.TransitWatch(hp["t_s"], hp["travel_s"])
                              if (give_up_transits and hp.get("kind") == "moved"
                                  and (hold_flight is None or hp.get("mode") == "LAND"))
                              else None)
                if (d.fsm_record or {}).get("hold_chosen"):
                    holds_taken.append({k: hp[k] for k in (
                        "t_s", "mode", "kind", "legal", "x", "y", "up", "travel_s",
                        "breaks")})
                    events.append({"t": round(t_now, 4), "type": "hold_point",
                                   **{k: hp[k] for k in ("mode", "kind", "legal", "x",
                                                         "y", "up", "travel_s",
                                                         "breaks", "reason")}})
            # The rails judge a moved transit on every tick of its plan, from
            # the tick the plan is taken (guardrail/hold_flight.py HoldFlight);
            # the rail arm judges it on the same ticks, by the same rule, on
            # the true position. Given up, the plan is held where the aircraft
            # is from this tick on, as a reached one is.
            if (hold_watch is not None and hold is not None and not hold_done
                    and not hold_reached(st, hold)
                    and hold_watch.update(HF.transit_distance(st.x, st.y, st.up, hold),
                                          t_now)):
                hold_done = True
                events.append({"t": round(t_now, 4), "type": "hold_transit_given_up",
                               "note": hold_watch.why})
            # The rail arm's Loiter is the rails' own HoldFlight, ticked where
            # the rails tick it (after the FSM and its plan), on the true
            # position (RAIL_HOLD_MODEL; guardrail/hold_flight.py).
            if hold_flight is not None:
                hold_tick = hold_flight.tick(shield.fsm, d.set_mode, st.x, st.y, st.up,
                                             t_now, dt)
                if hold_tick.note:
                    events.append({"t": round(t_now, 4), "type": "hold_flight",
                                   "note": hold_tick.note})
        if not on:
            # Shield OFF flies the RAW action, so the violations found on `raw`
            # are the flown action's - the same rule the SITL rails follow, and
            # what makes the control arm able to score a genuine escape.
            owner, flown, flown_vios = "pilot", raw, d.violations
        elif not esc:
            owner, flown, flown_vios = "shield", d.emitted, d.emitted_violations
        elif d.setpoint in ("pass", "brake") and d.command is not None:
            # The FSM's verdict: the Shield's action, or a stop.
            owner, flown, flown_vios = "shield", d.command, d.emitted_violations
            if d.setpoint == "brake":
                flown_vios = checker.filter(seen, flown, t=t_now).violations
        else:
            # LOITER / RTL / LAND own the aircraft (setpoint "none"), at the
            # FSM's hold point: an in-place one is the hold it always was, a
            # moved one is flown to first and then held (latched once reached,
            # so a gust after arrival is the position controller's, as before).
            owner = "autopilot"
            if (hold_flight is not None and ap_mode == "LOITER" and shield.fsm is not None
                    and not shield.fsm.terminal and shield.fsm.state.value == "Loiter"):
                # The rails' Loiter (RAIL_HOLD_MODEL): HoldFlight's command in
                # GUIDED, or, where it holds by the autopilot's own mode,
                # ArduCopter's BRAKE with its carry.
                if hold_tick is not None and hold_tick.command is not None:
                    c = hold_tick.command
                    flown = Action4D(vx=c[0], vy=c[1], vz_up=c[2])
                else:
                    flown = Action4D()
                    ap_brake = True
            else:
                if hold is not None and not hold_done and hold_reached(st, hold):
                    hold_done = True
                flown = _autopilot(ap_mode, st, m.home, dt, rsp, rtl_alt,
                                   hold=None if hold_done else hold,
                                   up_home=up_above_home(st))
            flown_vios = checker.filter(seen, flown, t=t_now).violations
        if owner in ("shield", "pilot"):
            # THE ADAPTER (WP3-04 / ARCH-04, every arm): what the rail streams
            # is a BODY action - the Shield's emitted one, a stop on the FSM's
            # "brake", or with the Shield off the pilot's body action as it
            # was handed in, as the owner above chose - turned into local NED
            # only here, after the projection, at the aircraft's own heading,
            # by the one body -> NED function every rail's adapter calls
            # (frames.body_to_local_ned, as sitl/mavlink_adapter_node.py turns
            # /shield/setpoint into SET_POSITION_TARGET_LOCAL_NED); the
            # integrator flies that NED velocity in its North / East / up axes.
            # (Chosen by this rail's own ownership, not by d.setpoint: a
            # Shield built with its own FSM on a run without the rail's
            # escalation - tools/failsafe_rubric.py - flies its emitted action.)
            sent = (body_in if owner == "pilot" else
                    BODY_STOP if esc and d.setpoint == "brake" else emitted_body)
            flown = local_ned_to_world(body_to_local_ned(sent, st.yaw_deg))
        owner_ticks[owner] += 1
        if owner == "autopilot":
            ap_mode_ticks[ap_mode or "none"] = ap_mode_ticks.get(ap_mode or "none", 0) + 1
        if oracle is not None:
            true_vios = oracle.filter(st, flown, t=t_now).violations
        else:
            true_vios = flown_vios
        unsafe = bool(truth.state_is_unsafe(st))

        # ---- which rules are in force: the SHIELD's reading ----------------
        if _timed(shield.policy):
            gov = _governing_recurrences(shield.policy)
            rs = shield.rule_status(seen)
            active = {r["id"] for r in rs if r["in_force"]}
            ids_now = {r["id"] for r in rs}
            timed_rules |= {r["id"] for r in rs
                            if r["id"] in gov or r["type"] in ("time_window_switch",
                                                               "corridor_swap")}
            if clock is not None:
                for rid, recs in gov.items():
                    for rec in recs:
                        now_on = recurrence_active(rec, clock())
                        key = id(rec)
                        if key in req_state and req_state[key] != now_on:
                            req_edges.setdefault(rid, []).append(t_now)
                        req_state[key] = now_on
            if prev_active is not None:
                on_ = sorted((active - prev_active) & prev_ids)
                off_ = sorted((prev_active - active) & ids_now)
                if on_ or off_:
                    events.append({"t": round(t_now, 4), "type": "time_window_switch",
                                   "clock": clock().isoformat() if clock else None,
                                   "activated": on_, "deactivated": off_,
                                   "read_from": "Shield.rule_status in_force "
                                                "(both sides of the instant)"})
                    for rid in on_:
                        switch_flips.setdefault(rid, []).append((t_now, True))
                    for rid in off_:
                        switch_flips.setdefault(rid, []).append((t_now, False))
                        if rid in last_violation_t:
                            release_gap[rid] = round(t_now - last_violation_t[rid], 4)
                    imposed = True
            else:
                events[0]["active_rules"] = sorted(active)
                events[0]["clock"] = clock().isoformat() if clock else None
            # Enforced before it is in force: the forecast saw the window
            # open inside the lookahead (the t+ side). Either the raw action
            # broke the rule, or (since 2026-10-21, C2) the Shield found the
            # window opening on the aircraft sooner than it can leave
            # (`window_opens_on` in the decision's trace, Shield.
            # _opening_imposed): a stop there is illegal and only an
            # escaping action passes, so a tick whose action is already the
            # escape breaks nothing and is still the rule seen coming and
            # enforced. Counted once per tick and rule either way.
            # The ticks counted ONLY through the trace (nothing violated:
            # the action was already the escape) are also counted apart,
            # `anticipated_opening_only_ticks_<rule>`: before C2 each was a
            # violation and counted as one, so this is how much of the
            # anticipated count the extension carries (review of 3660383,
            # P2: in the time-window-opens-on-vehicle cells, all of it).
            violated = {v.rule_id for v in d.violations}
            opens_on = set((d.trace or {}).get("window_opens_on") or ())
            for rid in violated | opens_on:
                if rid in timed_rules and rid not in active:
                    anticipated[rid] = anticipated.get(rid, 0) + 1
                    if rid not in violated:
                        opening_only[rid] = opening_only.get(rid, 0) + 1
            for rid in timed_rules:
                anticipated.setdefault(rid, 0)
                opening_only.setdefault(rid, 0)
            prev_active = active
            prev_ids = ids_now
        # A window opening on the aircraft, on ANY tick that names one: before
        # the opening (`window_opens_on`, Shield._opening_imposed) or after it
        # until the aircraft is out of the ring (`window_opened_on`, the
        # Shield's memory), whatever the action was. The episode-level "did a
        # window open on it" (true_state_summary). Until the review of
        # b1d3aca (P4) that question was read off
        # `anticipated_opening_only_ticks_*`, which counts only the ticks
        # whose action was ALREADY the escape: the library's 07:29:58 cell
        # flown by a hover or a 0.3 m/s eastward drift pilot - every tick
        # before the opening a violation, then repaired - has 20 anticipated
        # ticks, 0 of them opening-only, 3 ticks inside the polygon, and was
        # not listed at all.
        tr = d.trace or {}
        if tr.get("window_opens_on") or tr.get("window_opened_on"):
            window_opened_ticks += 1
        # A slowed escape breaks no rule, so no P0 counter sees it: the
        # floor refused at the cap (a slower escape flown, `cap_refused`) or
        # refused outright (the pilot's action flown unfloored, or reported
        # to the FSM, `floor_refused`). Until the review of 69c27bd (P3) only
        # the decision's trace said so, and the library's sideways pilot
        # stayed 16 ticks inside a zone that opened on it, its cap refused on
        # most of them, with "0 escapes" in every summary. Counted per tick
        # and named on the row.
        slowed = {k: tr[k] for k in ("cap_refused", "floor_refused") if tr.get(k)}
        cap_refused_ticks += "cap_refused" in slowed
        floor_refused_ticks += "floor_refused" in slowed
        for v in d.violations:
            last_violation_t[v.rule_id] = t_now
        if imposed and i > 0:
            phase += 1
            phase_vios.append({})

        row = {
            "t": round(t_now, 4), "tick": i,
            "x": st.x, "y": st.y, "up": st.up,
            "raw": raw.model_dump(), "emitted": flown.model_dump(),
            "violations": [v.model_dump() for v in d.violations],
            # The re-check of what was FLOWN, against the state the Shield SAW
            # - the grant's "counter from Shield repair log". Under clean
            # perception it is also the truth; under noise, `true_violations`
            # is, and the two are reported apart.
            "emitted_violations": [v.model_dump() for v in flown_vios],
            "repairs": [r.model_dump() for r in d.repairs],
            "braked": d.braked,
            # Position-based, and always the TRUE position: time to safe asks
            # where the aircraft actually was.
            "unsafe": unsafe,
            "owner": owner,
            # The trace's words, only on a tick that refused a floor (above).
            **slowed,
        }
        if ground is not None:
            # Over a declared ground `up` is the height above it; the true
            # MSL height beside it, what an MSL rule is declared in (X-11).
            row["msl_m"] = st.up + ground.elevation_at(st.x, st.y)
        if esc:
            row.update(fsm_state_before=d.fsm_state_before,
                       fsm_state_after=d.fsm_state_after, fsm_edge=d.fsm_edge,
                       set_mode=d.set_mode, setpoint=d.setpoint,
                       theta_exceeded=(d.fsm_record or {}).get("theta_exceeded"),
                       stop_illegal=((d.fsm_record or {}).get("flags") or {}).get(
                           "stop_illegal"))
            if d.fsm_fault:
                row["fsm_fault"] = d.fsm_fault
            if rtl_note:
                row["rtl_failed"] = rtl_note
            if owner == "autopilot":
                # guardrail.kpi leaves these ticks out of the Shield's own
                # counts: the Shield did not decide what flew.
                row["autopilot"] = ap_mode or "LOITER"
            if hold is not None:
                row["hold_point"] = {k: hold[k] for k in ("kind", "legal", "x", "y", "up")}
        if oracle is not None:
            row["perceived"] = {"x": seen.x, "y": seen.y, "up": seen.up}
            row["true_violations"] = [v.model_dump() for v in true_vios]
        rows.append(row)

        if p0(true_vios):
            true_p0_ticks += 1
            if owner == "autopilot":
                # The modelled LOITER / RTL / LAND flew a P0-violating action
                # (RTL's straight line home, a landing through a floor). Out of
                # the Shield's escape rate by definition (guardrail.kpi,
                # "AUTOPILOT TICKS"), so it is counted here instead of nowhere,
                # and per mode: a LOITER hold, an RTL leg and a landing are
                # three different hand-overs.
                autopilot_p0_ticks += 1
                key = ap_mode or "none"
                ap_p0_by_mode[key] = ap_p0_by_mode.get(key, 0) + 1
        if unsafe and not prev_unsafe and not imposed and moved_into_unsafe:
            # The aircraft carried ITSELF in: the position it moved to was
            # already unsafe at the previous instant, under the rules of that
            # instant (`moved_into_unsafe`, judged before the clock and the
            # events advanced). A spot that became unsafe only because time
            # moved on - a zone moving or growing onto it, a window opening
            # inside the Shield's lookahead - is imposed. A move the autopilot
            # made (an RTL leg, a landing through the floor) is counted apart:
            # the Shield was not flying.
            if prev_owner == "autopilot":
                autopilot_breaches += 1
            else:
                breaches += 1
        prev_unsafe = unsafe
        for v in d.violations:
            phase_vios[phase][v.rule_id] = phase_vios[phase].get(v.rule_id, 0) + 1

        # ---- the true position against every zone in force -------------------
        # The Shield's own compiled zones (moving ones where they are now), so
        # a hot-applied dynamic_nfz is measured exactly like a policy fence.
        inside_now = False
        entered_zones = []
        for zr in truth.zones_now(st.up):
            fence_seen = True
            sd = _signed_dist(zr.polygon, st.x, st.y)
            min_fence_d = min(min_fence_d, sd)
            if sd < 0:
                inside_now = True
                entered_zones.append(zr)
        if inside_now:
            inside_poly_ticks += 1
            if not prev_inside and not imposed and moved_into_polygon:
                # The aircraft moved ITSELF across a polygon edge (the same
                # test as a breach: inside a zone as it stood at the previous
                # instant). Inside because a zone spawned, moved, grew or
                # switched on over it is imposed, and is what time_to_safe
                # measures instead. Under the autopilot (RTL knows nothing of
                # the zones) it is counted apart.
                if prev_owner == "autopilot":
                    autopilot_poly_entries += 1
                else:
                    poly_entries += 1
        prev_inside = inside_now
        prev_owner = owner

        if any(c.type in ("corridor", "corridor_swap") for c in truth.policy.constraints):
            offs = [r["value"] for r in truth.rule_status(st)
                    if r["type"] in ("corridor", "corridor_swap") and r["in_force"]
                    and r["value"] is not None]
            if offs:
                final_offset = max(offs)
                max_offset = final_offset if max_offset is None else max(max_offset,
                                                                         final_offset)

        sp = math.hypot(flown.vx, flown.vy)
        max_speed = max(max_speed, sp)
        if sp > 0.1 and veh is None:
            # The ideal vehicle travels along its command. Under a response
            # the heading is the airframe's own velocity, set after the
            # integration step below.
            heading = (flown.vx / sp, flown.vy / sp)
        finite = finite and all(math.isfinite(v) for v in
                                (flown.vx, flown.vy, flown.vz_up, flown.yaw_rate))
        if subj:
            r_now = math.hypot(st.x - m.subject[0], st.y - m.subject[1])
            min_range = min(min_range, r_now)
            key = subj_class or "*"
            min_by_class[key] = min(min_by_class.get(key, math.inf), r_now)

        if esc and shield.fsm is not None and shield.fsm.terminal:
            # G9 / G10 / X5: the autopilot has finished the fail-safe. The
            # episode is over; nothing after this is the Shield's.
            terminated_s = t_now
            break

        # Drawn on every tick, whoever flies, so the gust sequence of a seed
        # does not depend on the hand-over.
        gx = rng_wind.gauss(0, stress.gust_mps) if stress.gust_mps else 0.0
        gy = rng_wind.gauss(0, stress.gust_mps) if stress.gust_mps else 0.0
        # What the airframe flies for this tick's command: the command itself
        # (the ideal vehicle, every family as it always was), or, under the
        # vehicle-response arm, its mean velocity over the tick through the
        # measured ArduPilot response. The wind model below then acts on that
        # velocity exactly as it acted on the command.
        if veh is None:
            fvx, fvy, fvz = flown.vx, flown.vy, flown.vz_up
        else:
            if hold_flight is not None:
                veh.brake(ap_brake)
                ap_brake = False
            fvx, fvy, fvz = veh.step((flown.vx, flown.vy, flown.vz_up), dt)
            vsp = math.hypot(fvx, fvy)
            veh_max_speed = max(veh_max_speed, vsp)
            if all(math.isfinite(v) for v in (flown.vx, flown.vy, flown.vz_up)):
                veh_max_lag = max(veh_max_lag, math.sqrt(
                    (fvx - flown.vx) ** 2 + (fvy - flown.vy) ** 2
                    + (fvz - flown.vz_up) ** 2))
            if vsp > 0.1:
                heading = (fvx / vsp, fvy / vsp)
            # What flew from this row's position, beside what was commanded.
            row["vehicle_v"] = [fvx, fvy, fvz]
            # The same true-state judgement as `true_p0_flown_ticks`, of the
            # velocity the airframe FLEW over this tick (VehicleResponse's
            # mean, from this row's true position, at this tick's rules)
            # instead of the command: a Brake or a slide that zeroes the
            # into-zone command while the airframe still carries toward the
            # zone counts here and not there. Yaw is not modelled, so the
            # command's yaw rate stands.
            yr = flown.yaw_rate if math.isfinite(flown.yaw_rate) else 0.0
            veh_vios = vjudge.filter(st, Action4D(vx=fvx, vy=fvy, vz_up=fvz, yaw_rate=yr),
                                     t=t_now).violations
            row["vehicle_violations"] = [v.model_dump() for v in veh_vios]
            if p0(veh_vios):
                true_p0_vehicle_ticks += 1
        if wind_model == "open_loop":
            # The model until 2026-10-12: the autopilot's modes rejected wind
            # and gusts, every other velocity drifted open-loop with both.
            if owner == "autopilot":
                gx = gy = 0.0
                wx = wy = 0.0
            else:
                wx, wy = wind
            vgx, vgy = fvx + wx + gx, fvy + wy + gy
        else:
            # ArduCopter's position controller flies every velocity (module
            # docstring, "THE WIND MODEL"): the steady wind is rejected within
            # the hold authority, a gust is pulled back at PSC_POSXY_P.
            ux, uy = guided_ground_velocity(fvx, fvy, wind)
            px, py = AP_POSXY_P * gust_off[0], AP_POSXY_P * gust_off[1]
            gust_off[0] += (gx - px) * dt
            gust_off[1] += (gy - py) * dt
            vgx, vgy = ux + gx - px, uy + gy - py
        ground_v = (vgx, vgy)
        prev_st = st
        up_next = st.up + fvz * dt
        nx, ny = st.x + vgx * dt, st.y + vgy * dt
        if ground is not None:
            # Over a declared ground (X-11) the vehicle holds its MSL height
            # and `up` is its height above the ground under it: the climb,
            # less the rise of the ground between the two positions (the
            # terrain-relative height a rail gets from terrain.
            # terrain_relative_up, with its home at the start's ground).
            up_next += ground.elevation_at(st.x, st.y) - ground.elevation_at(nx, ny)
        if veh is not None and up_next < 0.0:
            # The ground stops a descent (a landing's last tick, carried by
            # the response past the commanded stop); the ideal vehicle never
            # reaches it, because every descent it flies ends on the tick.
            up_next = 0.0
        st = State(x=nx, y=ny, up=up_next, yaw_deg=st.yaw_deg)
        # Where the move landed, judged NOW - before the clock and the next
        # tick's events advance: was it already unsafe / inside a zone? (See
        # the breach and polygon-entry tests above.)
        moved_into_unsafe = bool(truth.state_is_unsafe(st))
        moved_into_polygon = any(ring_contains(zr.polygon, st.x, st.y)
                                 for zr in truth.zones_now(st.up))

    wall_s = time.perf_counter() - wall0
    reached = None
    if kind in ("goto", "stub_vla"):
        reached = math.hypot(st.x - m.target[0], st.y - m.target[1]) <= GOAL_TOL_M

    rule_ids = [c.id for c in shield.policy.constraints]
    phases = {}
    if len(phase_vios) > 1:
        # Per rule, per stretch between harness events: "did the fence bind
        # BEFORE the window closed, and never after" is a gate on two of these.
        # Every rule gets a key in every phase, so a 0 is a measured 0 - and a
        # phase that never happened (the switch did not occur) has no keys, so a
        # gate on it reports "not measured" instead of passing.
        for k, counts in enumerate(phase_vios):
            for rid in rule_ids:
                phases[f"phase{k}_violation_ticks_{rid}"] = counts.get(rid, 0)

    boundary = {}
    eps = shield.dt / 2.0
    for rid, flips in switch_flips.items():
        ts, turned_on = flips[-1]
        edges = req_edges.get(rid) or []
        near = [e for e in edges if abs(e - ts) <= 2.0]
        if not near:
            continue          # an event-driven switch: no schedule edge to compare
        lag = round(ts - min(near, key=lambda e: abs(e - ts)), 4)
        boundary[f"switch_lag_s_{rid}"] = lag
        boundary[f"switch_conservative_{rid}"] = switch_conservative(turned_on, lag,
                                                                      eps, dt)
    for rid, n in anticipated.items():
        boundary[f"anticipated_ticks_{rid}"] = n
    for rid, n in opening_only.items():
        boundary[f"anticipated_opening_only_ticks_{rid}"] = n
    for rid, g in release_gap.items():
        boundary[f"release_gap_s_{rid}"] = g

    fsm = shield.fsm.summary() if (esc and shield.fsm is not None) else None
    extra = {
        "max_speed_flown": round(max_speed, 4),
        "all_actions_finite": finite,
        "ended_safe": not rows[-1]["unsafe"],
        "min_range_to_subject": (None if min_range is math.inf
                                 else round(min_range, 3)),
        "min_range_by_class": {k: round(v, 3) for k, v in min_by_class.items()},
        # Where it ended up, which is what "did the new rule take effect"
        # actually asks. A minimum cannot answer it: after a re-label the
        # minimum still records how close the vehicle was while the OLD rule
        # was in force, and that is a fact about the past, not a breach.
        "final_range_to_subject": (round(math.hypot(st.x - m.subject[0],
                                                    st.y - m.subject[1]), 3)
                                   if subj else None),
        "range_at_reclassify": (None if range_at_relabel is None
                                else round(range_at_relabel, 3)),
        **{f"min_range_as_{k}": round(v, 3) for k, v in min_by_class.items()},
        "subject_class_final": subj_class,
        # Lateral offset from the corridor(s) IN FORCE, as the Shield measures
        # it (rule_status): after a swap, from the new corridor.
        "max_offset_from_corridor": (None if max_offset is None else round(max_offset, 3)),
        "final_offset_from_corridor": (None if final_offset is None
                                       else round(final_offset, 3)),
        "reached_goal": reached,
        "final": {"x": round(st.x, 2), "y": round(st.y, 2), "up": round(st.up, 2)},
        # ---- stress-harness measurements -------------------------------------
        # Times the aircraft moved ITSELF from a legal position into an illegal
        # one. Transitions on the tick a rule changed under it (an event, a
        # window opening, a relabel), at t = 0, or onto a position that the
        # rules as they now stand make illegal from where it came (a moving or
        # growing zone) are imposed by the scenario and not counted.
        "breaches": breaches,
        # Ticks on which the flown action violated a P0 rule at the TRUE state,
        # whether or not the Shield saw it. Equal to the Shield's own count
        # under clean perception; under noise it is the honest one.
        "true_p0_flown_ticks": true_p0_ticks,
        "min_true_dist_to_fence_m": (None if not fence_seen
                                     else round(min_fence_d, 3)),
        "ticks_inside_fence_polygon": (None if not fence_seen else inside_poly_ticks),
        # Times the aircraft flew itself INTO a polygon (not counting t = 0 or
        # a zone that moved or appeared over it). The number a reader means by
        # "did it enter the no-fly zone".
        "polygon_entries": (None if not fence_seen else poly_entries),
        # The same two counts for moves the AUTOPILOT made (LOITER / RTL /
        # LAND after the FSM handed over): ArduPilot's RTL flies straight home
        # whatever lies between, and a landing goes through any floor.
        "autopilot_breaches": autopilot_breaches,
        "autopilot_polygon_entries": (None if not fence_seen else autopilot_poly_entries),
        # Of `true_p0_flown_ticks`, the ticks the AUTOPILOT flew. The grant's
        # P0 escape rate is the Shield's counter and leaves these out; this is
        # where a reader sees what the modelled fail-safe flew through.
        "autopilot_p0_flown_ticks": autopilot_p0_ticks,
        # ... per autopilot mode (LOITER / RTL / LAND; "none" for a fault
        # latched before a mode was named), so a reader can tell a hold from
        # an RTL leg from a landing. Sums to autopilot_p0_flown_ticks.
        "autopilot_p0_flown_ticks_by_mode": ap_p0_by_mode,
        # RTL ticks the RTL watch failed (G6's input), and the first reasons.
        "rtl_failed_ticks": rtl_failed_ticks if esc else None,
        "rtl_failures": rtl_failures,
        # The forecast every Shield of the episode was built with, and the
        # wind model it flew (module docstring).
        "forecast": fc,
        "wind_model": wind_model,
        # The vehicle the commands were flown on (module docstring, "THE
        # VEHICLE RESPONSE"): {"arm": "ideal", ...} on every headline episode.
        "vehicle_response": rsp_rec,
        "harness_events": len(events) - 1,
        "policy_generation_final": shield.policy.generation,
        # None when the scenario declares no dropout; otherwise the ticks the
        # Shield was really handed the dropout's pose (frozen or dead-
        # reckoned, GPS_DROPOUT_MODELS). A declared dropout that froze
        # nothing reads 0, never None, so it cannot pass for "not run".
        "gps_dropout_ticks": None if drop is None else dropout_ticks,
        # Ticks whose Shield decision named a window opening on the aircraft
        # (`window_opens_on` or `window_opened_on` in its trace), whatever
        # was flown; 0 when none did (see the count in the loop).
        "window_opened_on_ticks": window_opened_ticks,
        # Ticks whose decision refused the escape floor at the cap (a slower
        # escape flown) and refused it outright (the action flown unfloored,
        # or reported to the FSM); 0 when none did. Beside the P0 rate in
        # true_state_summary: a slowed escape is legal and no P0 count sees
        # it (review of 69c27bd, P3).
        "cap_refused_ticks": cap_refused_ticks,
        "floor_refused_ticks": floor_refused_ticks,
        # Who flew each tick: the Shield (its action or a stop), the autopilot
        # (LOITER / RTL / LAND after the FSM handed over), or the raw pilot
        # (the Shield-off arm).
        "ticks_flown_by": owner_ticks,
        "autopilot_ticks": owner_ticks["autopilot"],
        "autopilot_mode_ticks": ap_mode_ticks,
        # Where the FSM's Loiter / Land held (guardrail/holdpoint.py, since
        # 2026-10-19): every point it took, how many were moved off an
        # illegal spot, and how many were still illegal (no legal point found
        # within the search). None on an arm with no escalation FSM.
        "hold_points": holds_taken if esc else None,
        "hold_points_moved": (sum(1 for h in holds_taken if h["kind"] == "moved")
                              if esc else None),
        "hold_points_illegal": (sum(1 for h in holds_taken if not h["legal"])
                                if esc else None),
        # The DSL's lint warnings for this cell at mission start (above).
        "lint_warnings": lint_warnings,
        # What the modelled autopilot flew for the FSM's Loiter / Land
        # (AUTOPILOT_HOLD_MODELS): "moved_points" is modelled, flown by no
        # rail; "in_place" is what the rails fly. None with no FSM.
        "autopilot_hold_model": hold_model,
        # How the Shield recovered from outside an altitude band
        # (DEFAULT_RECOVERY_LAW): "cap" since 2026-10-29, "lookahead" before.
        "recovery_law": recovery_law,
        "terminated_s": terminated_s,
        # Events the scenario expected the Shield to refuse: None when there
        # were none, else whether every one was refused (and nothing else).
        "event_refusals_as_expected": (None if not es.refused
                                       else all(exp for _, _, exp in es.refused)),
        **boundary,
        **phases,
    }
    if consumer is not None:
        # Only on a prefix-consumer episode: what the consumer was and did.
        extra["prefix_consumer"] = dict(consumer.record(), stats=dict(consumer.stats))
    if veh is not None:
        # Only under a response arm, so a headline episode's table carries
        # exactly the keys it always did (plus its `vehicle_response` label).
        extra["max_vehicle_speed"] = round(veh_max_speed, 4)
        extra["max_velocity_lag_mps"] = round(veh_max_lag, 4)
        extra["nonfinite_commands_held"] = veh.nonfinite_commands
        # `true_p0_flown_ticks` judges the command at the true state;
        # this, the velocity the airframe flew (vjudge, review of
        # T1b-harness-vehicle, P3).
        extra["true_p0_vehicle_ticks"] = true_p0_vehicle_ticks
    if fsm is not None:
        extra["fsm"] = {k: fsm[k] for k in ("final_state", "terminal",
                                             "terminal_reason", "visited",
                                             "max_state", "failsafe_triggered",
                                             "loiter_entered", "outcome_label",
                                             "fsm_config_hash")}
        # A Loiter still holding at the end, with its operator timeout still
        # running: score() reads it as a CENSORED fail-safe outcome
        # (docs/RUBRIC-failsafe.md, section 3).
        extra["fsm"]["ended_holding"] = fsm.get("ended_holding")
        extra["fsm"]["transitions"] = [
            {k: tr[k] for k in ("t_s", "from", "to", "edge")} for tr in fsm["transitions"]]
    events.append({"t": round(rows[-1]["t"], 4), "type": "episode_end",
                   "policy_hash": shield.policy.policy_hash,
                   "generation": shield.policy.generation,
                   "final": extra["final"],
                   "fsm_outcome": (fsm or {}).get("outcome_label"),
                   "terminated_s": terminated_s})
    return EpisodeRun(rows=rows, extra=extra, events=events, policy=shield.policy,
                      policy_hash_start=hash0, sim_s=len(rows) * dt, wall_s=wall_s,
                      seed_consumed=stress.consumes_seed,
                      shield_window=shield.history,
                      priorities=dict(priorities, **K.rule_priorities(shield.policy)),
                      policy_source=source, audit=audit, generations=generation_irs)


def run_headless(sc, defaults=None, seed: int = 0) -> tuple[list[dict], dict]:
    """(rows, extra) for one episode - the shape the sweep has always returned."""
    run = run_episode(sc, defaults, seed)
    return run.rows, run.extra


# --------------------------------------------------------------------------- #
# arms: the paraphrased instruction and the rule-summary prefix
# --------------------------------------------------------------------------- #

def instruction_text(spec: ScenarioSpec, policy) -> tuple[str, str]:
    """(text, which) the paraphrase arms reword: the mission's task prompt, or
    the Prefix Compiler's rule summary for this cell (compile_csp's
    natural_language_prompt, rendered the way guardrail.paraphraser's
    scenario_renderings renders it, so the stored sets cover it)."""
    from guardrail.paraphraser import normalise_source
    src = spec.paraphrase.source if spec.paraphrase else "auto"
    m = spec.mission
    if src == "task_prompt" or (src == "auto" and m.task_prompt):
        return normalise_source(m.task_prompt), "task_prompt"
    return normalise_source(csp_text(spec, policy)), "csp"


def csp_text(spec: ScenarioSpec, policy) -> str:
    """compile_csp's rule summary for this cell's mission (no clock)."""
    from guardrail.compiler import ConstraintCompiler, Mission
    m = spec.mission
    sx, sy = m.start_pose.local(policy)
    tgt = m.target or m.start_pose
    tx, ty = tgt.local(policy)
    if m.rotate_deg:
        cx, cy = (m.rotate_about.x, m.rotate_about.y) if m.rotate_about else (sx, sy)
        sx, sy = _rotate(sx, sy, cx, cy, m.rotate_deg)
        tx, ty = _rotate(tx, ty, cx, cy, m.rotate_deg)
    mission = Mission(task_text=m.task_prompt or "", target_x=tx, target_y=ty,
                      cruise_alt_m=float(tgt.altitude or m.start_pose.altitude),
                      speed_pref_mps=float(m.pilot.speed or 4.0), start_x=sx, start_y=sy)
    return ConstraintCompiler(policy).compile_csp(mission).natural_language_prompt


_ARM_CACHE: dict = {}


def build_arm(ep, policy=None, drawn=None, defaults=None) -> dict | None:
    """The arm record of one episode, or None when it has none.

    paraphrase: guardrail.paraphraser.paraphrase(text, k, seed, backend) for the
    episode's seed (cached per cell and seed: the arms of one draw), the arm's
    own wording (index -1 is the canonical one, backend "identity"), its
    `manifest_extras()` and `to_record()`. prefix: whether the rule summary is
    shown, and the hash of the prompt the pilot would read.

    `drawn` (2026-11-07): the paraphrase arm's wording as someone else drew it
    - the stress harness gets it from the paraphraser service
    (guardrail/harness_service.py, WP4-10) - anything with `text`,
    `to_record()` and `manifest_extras()`. Used for a paraphrase arm (index
    >= 0) only; this sweep's own batch runs still draw in process.

    A PREFIX-CONSUMER episode (2026-11-16, `ep.prefix_consumer`; module
    docstring) always has a record, and it alone carries three more keys:
    `pilot_reads_prefix` True (separate from `pilot_reads_text`, which stays
    False: the pilot reads the CSP's action sets, not its words), `consumer`
    (guardrail.prefix_consumer.consumer_record: the pilot's id, its adapter,
    the arm, the law it wraps and, on the prefix-on arm, the identity of the
    CSP it flies - prefix_csp, compiled here with `defaults`, the library's)
    and `repeat` (1 on an A/A repeat). Raises ValueError for a scenario the
    consumer cannot fly (prefix_consumer_refusal, a policy that is not
    static)."""
    spec = ep.cell.spec
    consumer = getattr(ep, "prefix_consumer", None)
    if ep.paraphrase_arm is None and ep.prefix == "on" and not ep.ab and not consumer:
        return None
    from guardrail import paraphraser as P
    if policy is None:
        policy = spec.policy.load(ROOT)
    rec: dict[str, Any] = {"arm": ep.arm_tag, "prefix": ep.prefix,
                           "pilot_reads_text": False}
    text = None
    if ep.paraphrase_arm is not None:
        text, which = instruction_text(spec, policy)
        if ep.paraphrase_arm < 0:
            arm = P.canonical(text)
        elif drawn is not None:
            arm = drawn
        else:
            key = (ep.episode_id, ep.seed, text, spec.paraphrase.backend, spec.paraphrase.k)
            draw = _ARM_CACHE.get(key)
            if draw is None:
                draw = _ARM_CACHE[key] = P.paraphrase(text, spec.paraphrase.k, ep.seed,
                                                      spec.paraphrase.backend)
            if ep.paraphrase_arm >= len(draw):
                raise ValueError(f"paraphrase arm {ep.paraphrase_arm} but the draw "
                                 f"holds {len(draw)}")
            arm = draw[ep.paraphrase_arm]
        rec.update(instruction_source=which, manifest_extras=arm.manifest_extras(),
                   paraphrase=arm.to_record())
        text = arm.text
    csp = csp_text(spec, policy) if ep.prefix == "on" else ""
    words = text if text is not None else (spec.mission.task_prompt or "")
    prompt = (csp + "\n" + words).strip() if csp else words
    rec["prompt_sha256"] = "sha256:" + hashlib.sha256(prompt.encode("utf-8")).hexdigest()
    rec["prefix_sha256"] = ("sha256:" + hashlib.sha256(csp.encode("utf-8")).hexdigest()
                            if csp else None)
    if consumer:
        why = prefix_consumer_refusal(spec)
        if why:
            raise ValueError(why)
        held = prefix_csp(spec, policy, defaults, ep.seed) if ep.prefix == "on" else None
        rec.update(pilot_reads_prefix=True,
                   consumer=PC.consumer_record(ep.prefix, held,
                                               VS.MODEL_IDS[spec.mission.pilot.type]),
                   repeat=int(getattr(ep, "repeat", 0) or 0))
    return rec


def manifest_extras(arm: dict | None) -> dict | None:
    """The flat keys written beside the six-field manifest
    (manifest_extras.json): the paraphrase's own (paraphraser.manifest_extras)
    and the prefix arm. The manifest itself stays the grant's six fields
    (guardrail/manifest.py). A prefix-consumer arm also names its consumer,
    with the consumer's own source pin (`prefix_consumer_model_hash`:
    guardrail.manifest.model_hash of its id - a headless manifest's
    vla_model_hash names this harness's pilot function, not the consumer's
    module), its adapter, the wrapped law's own source pin
    (`prefix_wraps_model_hash`), the action sets it flew and the A/A
    repeat."""
    if not arm:
        return None
    out = dict(arm.get("manifest_extras") or {})
    out.update(prefix_arm=arm["prefix"], prompt_sha256=arm["prompt_sha256"],
               pilot_reads_text=arm["pilot_reads_text"])
    if arm.get("pilot_reads_prefix"):
        c = arm.get("consumer") or {}
        from guardrail.manifest import model_hash
        out.update(pilot_reads_prefix=True, prefix_consumer=c.get("id"),
                   prefix_consumer_model_hash=(model_hash(c["id"]) if c.get("id")
                                               else None),
                   prefix_consumer_adapter=c.get("adapter"),
                   prefix_wraps_model_hash=c.get("wraps_model_hash"),
                   prefix_action_set_sha256=c.get("action_set_sha256"),
                   prefix_repeat=arm.get("repeat", 0))
    return out


# --------------------------------------------------------------------------- #
# the prefix consumer (module docstring, THE PREFIX CONSUMER; 2026-11-16)
# --------------------------------------------------------------------------- #

# What a prefix-on arm of a consuming pilot is scored without, and why.
LABELS_NOT_APPLIED = ("the cell's outcome and fail-safe labels were derived for its "
                      "rule-ignorant pilot; this is the prefix-on arm of a pilot that "
                      "consumes the CSP, so they are not applied (its gates are)")


def prefix_consumer_refusal(spec: ScenarioSpec) -> str | None:
    """Why the prefix consumer cannot fly this scenario, or None. It flies
    one CSP, compiled before take-off: a scenario whose rules change in
    flight (any mid-flight event) needs generation N's CSP at the pilot,
    which is audit card WP4-19's plumbing and is not built."""
    if spec.events:
        return (f"{spec.scenario_id}: a prefix-consumer arm of a scenario with "
                f"{len(spec.events)} mid-flight event(s): the consumer flies the CSP "
                f"compiled before take-off, and generation N's CSP reaching the pilot is "
                f"not built (audit card WP4-19); static families only")
    return None


def prefix_csp(spec: ScenarioSpec, policy=None, defaults=None, seed: int = 0, *,
               max_s_add: float = 0.0):
    """The CSP the prefix consumer flies this cell and seed on: compiled from
    the cell's policy for the mission as the rails are told it (rail_mission,
    each number as rail_node_args writes it), at the scenario's clock, static
    over the rail's flight length (guardrail.prefix_consumer.static_csp). The
    VLA stub compiles its own from the same arguments
    (sitl/ros2_vla_stub_node.py --prefix on), so a cell and seed fly one CSP
    headless and on a rail, and run_hil checks the node's against this one.
    `max_s_add`: a lengthened hil campaign's seconds past the rail cap
    (--hil-max-s-add), which the stub is sent in --max-s and checks over; the
    CSP itself does not depend on it, only whether it is static that long
    (rail_node_args checks it so, before the rig is touched). Raises
    ValueError when no such CSP exists (a rule switching in flight, the P0
    rules over the token budget, a rule the Shield does not enforce)."""
    from guardrail.csp import CSPBudgetExceeded, CSPUnenforcedRules
    pol = policy if policy is not None else spec.policy.load(ROOT)
    rm = rail_mission(spec, pol, defaults, seed)

    def r(v):
        return None if v is None else float(_num(v))
    target = None if rm["target"] is None else tuple(r(v) for v in rm["target"])
    mission = PC.consumer_mission(tuple(r(v) for v in rm["start"]), target, r(rm["speed"]))
    try:
        return PC.static_csp(pol, mission, now=rm["clock_start"],
                             duration_s=rail_max_s(rm["max_s"], max_s_add))
    except (CSPBudgetExceeded, CSPUnenforcedRules) as e:
        raise ValueError(f"{spec.scenario_id}: no CSP for the prefix consumer "
                         f"({type(e).__name__}: {e})") from e


def arm_consumer(spec: ScenarioSpec, policy, defaults, seed: int, arm: dict, law):
    """The PrefixConsumer an arm record names, around `law` (the cell's
    pilot): with the cell's CSP on the prefix-on arm - which must be the one
    the record was built with (`consumer.action_set_sha256`), else the
    episode is refused rather than flown on another CSP under its id - and
    with none on the prefix-off arm."""
    rec = arm.get("consumer") if isinstance(arm.get("consumer"), dict) else {}
    if rec.get("id") != PC.MODEL_ID or rec.get("adapter") not in PC.ADAPTERS:
        raise ValueError(f"the arm record names prefix consumer {rec.get('id')!r} "
                         f"(adapter {rec.get('adapter')!r}); this harness flies "
                         f"{PC.MODEL_ID} ({', '.join(PC.ADAPTERS)})")
    why = prefix_consumer_refusal(spec)
    if why:
        raise ValueError(why)
    wraps = VS.MODEL_IDS[spec.mission.pilot.type]
    if arm.get("prefix") != "on":
        return PC.PrefixConsumer(law, wraps=wraps)
    csp = prefix_csp(spec, policy, defaults, seed)
    got = PC.action_set_sha256(csp)
    if rec.get("action_set_sha256") != got:
        raise ValueError(f"the arm record's CSP ({rec.get('action_set_sha256')}) is not "
                         f"the one this cell and seed compile to ({got})")
    return PC.PrefixConsumer(law, csp, policy, wraps=wraps)


def scoring_spec(spec: ScenarioSpec, ep) -> ScenarioSpec:
    """The spec an episode is SCORED against: the cell's own, except on the
    prefix-on arm of a pilot that consumes the CSP, where the outcome and
    fail-safe labels are dropped (LABELS_NOT_APPLIED: they say what the
    cell's rule-ignorant pilot provokes - a fail-safe after 30 s of asking
    for the unsafe thing - and a pilot that reads the rules does not ask).
    The gates (the safety invariants) and every pin are unchanged."""
    if (ep is None or not getattr(ep, "prefix_consumer", None) or ep.prefix != "on"
            or (spec.expected_outcome is None and spec.expected_failsafe is None)):
        return spec
    return spec.model_copy(update={"expected_outcome": None, "expected_failsafe": None,
                                   "failsafe_clause": None})


def prefix_family_suffix(ep) -> str:
    """The KPI-row term of a prefix-consumer episode, "/prefix-consumer-on" /
    "-off" ("" for every other episode): a consuming pilot's arms never pool
    with each other or with the cell's own pilot (tools/kpi_report.py
    consumer_suffix writes the same term)."""
    if ep is None or not getattr(ep, "prefix_consumer", None):
        return ""
    return f"/prefix-consumer-{ep.prefix}"


def prefix_flight_problems(metrics: dict, meta: dict) -> list[str]:
    """Why a rail flight is not the prefix arm its scenario record names
    (`arm_record`, rail_meta): the VLA node announced no prefix arm at all
    (it flew its plain pilot: an old checkout on the rig, a dropped flag),
    another arm, another consumer, or - on the prefix-on arm - a CSP whose
    action sets are not the ones this cell and seed compile to. [] for a
    flight that is, and for every flight without a prefix consumer. The
    whole-CSP content hash is compared too, but recorded, not required
    (prefix_flight_record): its prose and risk grades hold computed
    distances, which two hosts need not round alike."""
    want = meta.get("arm_record") if isinstance(meta.get("arm_record"), dict) else {}
    if not want.get("pilot_reads_prefix"):
        return []
    c = want.get("consumer") or {}
    ident = metrics.get("vla_identity") if isinstance(metrics.get("vla_identity"), dict) else {}
    got = ident.get("prefix") if isinstance(ident.get("prefix"), dict) else None
    if got is None:
        return [f"the VLA node announced no prefix arm on /vla/identity (model "
                f"{ident.get('model_id')!r}): the mission flew its plain pilot, not the "
                f"prefix-{c.get('arm')} arm of {c.get('id')}"]
    out = []
    if ident.get("model_id") != c.get("id"):
        out.append(f"the VLA node's model is {ident.get('model_id')!r}, the arm's "
                   f"{c.get('id')!r}")
    for key, what in (("id", "consumer"), ("adapter", "adapter"), ("arm", "prefix arm"),
                      ("wraps", "wrapped pilot law"),
                      ("wraps_model_hash", "wrapped law's source pin"),
                      ("corridor_path_test", "path test"),
                      ("corridor_margin_m", "corridor margin"),
                      ("policy_hash", "CSP's policy"),
                      ("action_set_sha256", "CSP's action sets")):
        if got.get(key) != c.get(key):
            out.append(f"the VLA node's {what} is {got.get(key)!r}, the arm's "
                       f"{c.get(key)!r}")
    return out


def prefix_generation_problems(meta: dict, generations: dict | None,
                               rows=()) -> list[str]:
    """Why a rail flight of a prefix-consumer arm is not that arm's flight
    because its rules changed in flight: the Shield flew a generation above
    0 (a policy_g<N>.json of the run, `generations`, or a log row's
    `generation`) while the consumer flew generation 0's CSP, which
    prefix_flight_problems alone cannot see - the node announces the CSP it
    took off with. The harness refuses such a change up front
    (guardrail/harness_service.py _consumer_takes_no_events, and
    prefix_consumer_refusal for a scenario's own events); this is the
    backstop for one that reached the Shield anyway (a client of the
    mission's REST endpoint, an old harness). [] for every flight without a
    prefix consumer."""
    want = meta.get("arm_record") if isinstance(meta.get("arm_record"), dict) else {}
    if not want.get("pilot_reads_prefix"):
        return []
    above = {int(g) for g in (generations or {}) if isinstance(g, int) and g > 0}
    above |= {int(r["generation"]) for r in rows
              if isinstance(r, dict) and isinstance(r.get("generation"), int)
              and not isinstance(r.get("generation"), bool) and r["generation"] > 0}
    if not above:
        return []
    return [f"the Shield's rules changed in flight (generation(s) {sorted(above)}) while "
            f"the prefix consumer flew generation 0's CSP: a prefix-consumer arm takes no "
            f"mid-flight rule change (generation N's CSP at the pilot is audit card "
            f"WP4-19, not built)"]


def prefix_flight_record(metrics: dict, meta: dict) -> dict | None:
    """What a rail flight's VLA node announced of its prefix arm beside what
    the harness expected (None without a prefix consumer): the node's record,
    and whether its whole-CSP content hash is the harness's."""
    want = meta.get("arm_record") if isinstance(meta.get("arm_record"), dict) else {}
    if not want.get("pilot_reads_prefix"):
        return None
    c = want.get("consumer") or {}
    ident = metrics.get("vla_identity") if isinstance(metrics.get("vla_identity"), dict) else {}
    got = ident.get("prefix") if isinstance(ident.get("prefix"), dict) else {}
    return {"announced": got or None, "expected": c,
            "csp_content_hash_agrees": (None if c.get("csp_content_hash") is None
                                        else got.get("csp_content_hash")
                                        == c.get("csp_content_hash"))}


# The version of the self-contained keys episode_extras() adds; a reader
# (rerun_from_extras, the WP4 outputs tools) refuses a schema it does not know.
#
# /1  every bundle written before 2026-10-20: the ideal vehicle (a bundle
#     from before 2026-10-19 names no `arm.response`), the modelled autopilot
#     holding where the aircraft was (there were no hold points), and no
#     `arm.autopilot_hold`.
# /2  taken on 2026-10-20 by two branches at once, each for its own key, so
#     a /2 bundle is told apart by which of the two it names:
#       - the hold points (T1c): `arm.autopilot_hold` - which hold the
#         modelled autopilot flew (AUTOPILOT_HOLD_MODELS) - always written,
#         on the ideal vehicle. A /1 bundle predates the hold-point search,
#         so its autopilot held where the aircraft was: rerun_from_extras
#         reads a /1 bundle with no `autopilot_hold` as "in_place", not as
#         today's default (re-review of T1c, P2: an in_place bundle with the
#         key gone re-flew as moved_points, 18 autopilot P0 ticks against
#         the 38 flown, and tools/rerun_episode.py called a sound bundle
#         "differs").
#       - the vehicle-response arm (T1b; review of T1b-harness-vehicle, P2):
#         an episode flown on a vehicle response other than the ideal one,
#         named in `arm.response`, its autopilot holding in place (that
#         branch had no hold points); its ideal-vehicle bundles stayed /1. A
#         reader from before the arm (c77c77d) accepted "/1" and would have
#         re-flown such a bundle on the ideal vehicle under its id - the
#         silent misreplay the parameter-drift guard in rerun_from_extras
#         exists to stop, in the other direction; "/2" made it refuse.
# /3  since the merge of the two (2026-10-21): every bundle names BOTH
#     `arm.autopilot_hold` and `arm.response`. Each branch's reader accepts
#     "/2" and knows one of the two keys, so it would re-fly a bundle of the
#     merged harness with the wrong hold (the T1b reader: a moved hold flown
#     in place) or on the wrong vehicle (the T1c reader: an ArduPilot bundle
#     flown ideal) under the bundle's id; both refuse "/3". A headline bundle
#     cannot stay "/1" as it did on the T1b branch: its autopilot flies the
#     moved hold points, which no reader of "/1" models.
# /4  since 2026-10-29 (WP3-17): every bundle also names `arm.recovery`, the
#     Shield's recovery law (DEFAULT_RECOVERY_LAW). Every /1-/3 bundle was
#     flown before the law had a name, on "lookahead", and is re-flown so; a
#     /3 reader would re-fly a /4 bundle on "lookahead" under its id, so it
#     refuses "/4".
# /5  since 2026-11-06 (WP3-15, WP3-17): the spec's `stress` carries
#     `fix_quality_reported` (whether the modelled rail told the Shield its
#     pose was stale or how accurate it was: State.fix_ok, State.pos_sigma_m)
#     and the arm names `dr_drift_mps`, the dead reckoning's drift allowance;
#     the default recovery law became "cap_all" (named in `arm.recovery` as
#     on /4). A /4 reader would re-fly a /5 sensor bundle with the Shield
#     blind to the report, so it refuses "/5"; a /1-/4 bundle was flown blind
#     and is re-flown so (`fix_quality_reported` False).
EXTRAS_SCHEMA = "sweep-episode-extras/5"
READABLE_EXTRAS_SCHEMAS = ("sweep-episode-extras/1", "sweep-episode-extras/2",
                           "sweep-episode-extras/3", "sweep-episode-extras/4",
                           EXTRAS_SCHEMA)
EXTRAS_SCHEMAS_READ = READABLE_EXTRAS_SCHEMAS       # the T1b branch's name


# A signed policy bundle up to this size is carried in the episode's extras
# (base64), so the episode re-runs with no checkout at all. The repository's
# bundles are about 1 kB; a larger archive is referenced by path and hash.
MAX_INLINE_BUNDLE_BYTES = 1 << 20


def _policy_record(spec: ScenarioSpec, policy_hash: str) -> dict:
    """The policy as injected at t = 0, carried in the bundle: a YAML policy's
    whole text, or a signed bundle's archive (base64, up to
    MAX_INLINE_BUNDLE_BYTES), so the episode re-runs with no checkout of
    policies/ or bundles/; always the source path and the file's hash."""
    p = Path(spec.policy.source)
    p = p if p.is_absolute() else ROOT / p
    rec: dict[str, Any] = {"source": spec.policy.source, "policy_hash": policy_hash,
                           "kind": "yaml" if spec.policy.path is not None else "bundle",
                           "yaml_text": None, "bundle_b64": None, "file_sha256": None}
    if p.is_file():
        data = p.read_bytes()
        rec["file_sha256"] = "sha256:" + hashlib.sha256(data).hexdigest()
        if spec.policy.path is not None:
            rec["yaml_text"] = data.decode("utf-8")
        elif len(data) <= MAX_INLINE_BUNDLE_BYTES:
            import base64
            rec["bundle_b64"] = base64.b64encode(data).decode("ascii")
    return rec


def policy_generations(events: list[dict]) -> list[dict]:
    """Every policy generation of an episode, from its harness events: the one
    injected at t = 0 and each hot-apply the Shield accepted after it, with
    the hash it produced (a refused event made no generation)."""
    out = []
    for e in events:
        if e.get("type") == "episode_start":
            out.append({"generation": e.get("generation"), "policy_hash": e.get("policy_hash"),
                        "t": 0.0, "event": None})
        elif e.get("type") == "hot_apply" and not e.get("refused"):
            out.append({"generation": e.get("generation"), "policy_hash": e.get("policy_hash"),
                        "t": e.get("t"), "event": e.get("event"),
                        "rule_id": e.get("rule_id") or e.get("zone") or e.get("target")})
    return out


def episode_extras(spec: ScenarioSpec, defaults, seed: int, run: EpisodeRun, *,
                   arm: dict | None = None, forecast: str = "grant",
                   wind_model: str = DEFAULT_WIND_MODEL, episode=None,
                   autopilot_hold: str = DEFAULT_AUTOPILOT_HOLD,
                   response: str = DEFAULT_RESPONSE_ARM,
                   recovery: str = DEFAULT_RECOVERY_LAW) -> dict:
    """The keys that make an episode bundle self-contained (manifest_extras.json,
    beside the grant's six-field manifest): the expanded ScenarioSpec cell
    with every parameter bound, its events, the library defaults it relied
    on, the seed, the arm (paraphrase / prefix record, forecast, wind model,
    since 2026-10-19 the vehicle response, and since 2026-10-20 the
    autopilot's hold model), the policy as injected and every generation
    after it. rerun_from_extras() re-flies the episode from
    these keys alone; the key names are listed in the CHANGELOG entry of
    2026-10-12 for the WP4 outputs tools.

    `vehicle_response` (and `arm.response`) is written on every bundle, the
    ideal vehicle's included, so a bundle says which vehicle flew it without
    a reader having to know when the arm appeared. A bundle written before
    2026-10-19 has neither key and was flown on the ideal vehicle (the only
    one there was); rerun_from_extras reads it so. `arm.autopilot_hold` is
    written on every bundle too, and since 2026-10-29 `arm.recovery` (the
    Shield's recovery law; the run's own `recovery_law` when it has one).
    Since 2026-11-06 `arm.dr_drift_mps` too, and the spec's stress names
    `fix_quality_reported`. Every bundle is EXTRAS_SCHEMA ("/5", which has
    the history): a reader that does not know a key refuses it rather than
    re-fly it on the ideal vehicle, with the wrong hold, on the wrong
    recovery law or with the Shield blind to the rail's fix report."""
    dfl = _as_defaults(defaults)
    out: dict[str, Any] = {
        "extras_schema": EXTRAS_SCHEMA,
        "scenario_id": spec.scenario_id,
        "template": spec.template,
        "random_seed": int(seed),
        "arm": {"record": arm, "forecast": forecast, "wind_model": wind_model,
                "autopilot_hold": autopilot_hold, "response": response,
                "recovery": run.extra.get("recovery_law") or recovery,
                "dr_drift_mps": DR_DRIFT_MPS},
        "forecast": run.extra.get("forecast") or forecast_for(spec, dfl, forecast),
        "wind_model": wind_model,
        "vehicle_response": run.extra.get("vehicle_response") or response_for(response),
        "library_defaults": dfl.model_dump(mode="json"),
        "scenario_spec": _spec_record(spec),
        "scenario_events": [e.model_dump(mode="json") for e in spec.events],
        "policy": _policy_record(spec, run.policy_hash_start),
        "generations": policy_generations(run.events),
        "rerun": "experiments/sweep_scenarios.py rerun_from_extras(<this JSON>)",
    }
    if episode is not None:
        out.update(episode_id=episode.episode_id, run_id=episode.run_id,
                   params=episode.cell.params, arm_tag=episode.arm_tag)
    world = episode_world(spec)
    if world is not None:
        # Which world the Shields judged the distance_envelope in (its file
        # is named by scenario_spec.world; this pins its content).
        out["world"] = world.record()
    ground = episode_ground(spec)
    if ground is not None:
        # The ground the Shields converted MSL rules over (X-11): the file
        # scenario_spec.ground names, its hash and the resampling's bound.
        out["ground"] = ground.record()
    return out


def _spec_record(spec: ScenarioSpec) -> dict:
    """The scenario as recorded in its extras: the spec's JSON dump, without
    a `ground` key when it has none (X-11, 2026-11-18), so every record of a
    scenario without a ground is byte for byte what it was before the field
    existed (rerun_from_extras reads the missing key as None)."""
    out = spec.model_dump(mode="json", by_alias=True)
    if out.get("ground") is None:
        out.pop("ground", None)
    return out


def rerun_from_extras(extras: dict, root: Path | None = None) -> EpisodeRun:
    """Re-fly one headless episode from its bundle's manifest_extras alone.

    A YAML policy is written from `policy.yaml_text` to a temporary file and
    pinned to the recorded hash (`expected_hash`), so a policy that differs
    from the one flown stops the re-run instead of flying something else. A
    bundle policy is written from `policy.bundle_b64` the same way (its file
    hash checked first); one too large to have been inlined is read from its
    recorded path under `root` and pinned the same way.

    The episode is flown as it was flown: with the arm, the hold and the
    vehicle its bundle records (EXTRAS_SCHEMA has the history of the schema):
      /1  the ideal vehicle (a bundle from before the arm names no response;
          one that names another vehicle under "/1" contradicts itself and is
          refused), with the hold its `arm.autopilot_hold` names, else in
          place, which every bundle written before the key existed flew;
      /2  one naming `arm.autopilot_hold` is the hold-point branch's, on the
          ideal vehicle; one naming a non-ideal `arm.response` is the
          response arm's, held in place (that branch had no hold points); one
          naming both or neither is refused, never guessed;
      /3  names both, or is refused.
    /1-/3 were flown on the "lookahead" recovery law (one naming
    `arm.recovery` contradicts itself and is refused); /4 and /5 name their
    law, or are refused. /1-/4 were flown with no fix-quality report (the
    Shield blind on a degraded pose) and are re-flown so; /5 records its
    drift allowance, and one flown with another is refused."""
    schema = extras.get("extras_schema")
    if schema not in READABLE_EXTRAS_SCHEMAS:
        raise ValueError(f"manifest_extras schema {schema!r}, this harness reads "
                         f"{', '.join(repr(s) for s in READABLE_EXTRAS_SCHEMAS)}")
    arm = extras.get("arm") or {}
    hold = arm.get("autopilot_hold")
    rsp_named = arm.get("response")
    ideal = rsp_named in (None, DEFAULT_RESPONSE_ARM)
    if schema == "sweep-episode-extras/1":
        if not ideal:
            raise ValueError(f"manifest_extras schema {schema!r} is the ideal vehicle, but "
                             f"the bundle names the {rsp_named!r} response (a bundle "
                             f"flown on another vehicle is {EXTRAS_SCHEMA!r})")
        hold = hold or "in_place"
    elif schema == "sweep-episode-extras/2":
        if hold is not None and not ideal:
            raise ValueError(f"manifest_extras schema {schema!r} names both "
                             f"arm.autopilot_hold {hold!r} and the {rsp_named!r} "
                             f"response: no harness wrote that under {schema!r}")
        if hold is None and ideal:
            raise ValueError(f"manifest_extras schema {schema!r} names no "
                             f"arm.autopilot_hold and no non-ideal vehicle response "
                             f"(arm.response {rsp_named!r}): which hold was flown is "
                             f"unknown")
        hold = hold or "in_place"
    else:
        if hold is None:
            raise ValueError(f"manifest_extras schema {schema!r} names no "
                             f"arm.autopilot_hold: which hold was flown is unknown")
        if rsp_named is None:
            raise ValueError(f"manifest_extras schema {schema!r} names no "
                             f"arm.response: which vehicle flew it is unknown")
    recovery = arm.get("recovery")
    if schema in ("sweep-episode-extras/4", EXTRAS_SCHEMA):
        if recovery is None:
            raise ValueError(f"manifest_extras schema {schema!r} names no arm.recovery: "
                             f"which recovery law was flown is unknown")
    elif recovery is not None:
        raise ValueError(f"manifest_extras schema {schema!r} predates arm.recovery but "
                         f"names {recovery!r}: no harness wrote that under {schema!r}")
    else:
        recovery = "lookahead"
    response = rsp_named or DEFAULT_RESPONSE_ARM
    # A bundle written before 2026-10-19 may carry pins that name no gates,
    # which the spec now refuses; read as they were written (they decide
    # only the verdict, never the flight).
    body = legacy_pins_read_as_before(extras["scenario_spec"])
    if schema != EXTRAS_SCHEMA:
        # /1-/4 were flown before the rail's fix-quality report existed: the
        # Shield flew blind on a degraded pose, and is re-flown so. One that
        # names a drift allowance contradicts itself.
        if arm.get("dr_drift_mps") is not None:
            raise ValueError(f"manifest_extras schema {schema!r} predates arm.dr_drift_mps "
                             f"but names {arm.get('dr_drift_mps')!r}: no harness wrote "
                             f"that under {schema!r}")
        body = {**body, "stress": {**(body.get("stress") or {}),
                                   "fix_quality_reported": False}}
    elif arm.get("dr_drift_mps") != DR_DRIFT_MPS:
        # Flown with a drift allowance this harness no longer uses: re-flying
        # it on today's would describe a different Shield under its id.
        raise ValueError(f"the bundle was flown with dr_drift_mps "
                         f"{arm.get('dr_drift_mps')!r}; this harness flies {DR_DRIFT_MPS!r}")
    pol = extras["policy"]
    tmp = None
    try:
        if pol.get("yaml_text") is not None:
            tmp = tempfile.TemporaryDirectory(prefix="rerun-policy-")
            path = Path(tmp.name) / "policy.yaml"
            path.write_text(pol["yaml_text"], encoding="utf-8", newline="")
            body["policy"] = {"path": str(path), "expected_hash": pol["policy_hash"]}
        elif pol.get("bundle_b64") is not None:
            import base64
            data = base64.b64decode(pol["bundle_b64"])
            got = "sha256:" + hashlib.sha256(data).hexdigest()
            if pol.get("file_sha256") and got != pol["file_sha256"]:
                raise ValueError(f"the inlined policy bundle hashes to {got}, the "
                                 f"extras record {pol['file_sha256']}")
            tmp = tempfile.TemporaryDirectory(prefix="rerun-policy-")
            path = Path(tmp.name) / PurePath(pol["source"]).name
            path.write_bytes(data)
            body["policy"] = {"bundle_path": str(path), "expected_hash": pol["policy_hash"]}
        else:
            src = Path(pol["source"])
            if not src.is_absolute():
                src = Path(root or ROOT) / src
            body["policy"] = {"bundle_path": str(src), "expected_hash": pol["policy_hash"]}
        spec = ScenarioSpec.model_validate(body)
        recorded = (extras.get("vehicle_response") or {}).get("params")
        if recorded is not None and RESPONSES.get(response) is not None \
                and recorded != RESPONSES[response].record():
            # The bundle was flown on response parameters this harness no
            # longer holds (AP_RESPONSE re-measured since): re-flying it on
            # today's would describe a different vehicle under its id.
            raise ValueError(f"the bundle's {response!r} response was flown with "
                             f"{recorded}; this harness holds "
                             f"{RESPONSES[response].record()}")
        return run_episode(spec, Defaults.model_validate(extras["library_defaults"]),
                           int(extras["random_seed"]), arm=arm.get("record"),
                           forecast=arm.get("forecast", "grant"),
                           wind_model=arm.get("wind_model", DEFAULT_WIND_MODEL),
                           autopilot_hold=hold, response=response, recovery=recovery)
    finally:
        if tmp is not None:
            tmp.cleanup()


# --------------------------------------------------------------------------- #
# the SITL backend (ArduPilot SITL + MAVROS 2)
# --------------------------------------------------------------------------- #

def sitl_available() -> bool:
    """Can this host actually fly the ArduPilot rail?

    Checked rather than assumed, because the alternative - running nothing and
    printing a pass - is the failure mode this harness exists to remove.
    """
    return (bool(shutil.which("wsl")) and (ROOT / SITL_SCRIPT).is_file()
            and (ROOT / SITL_NODE).is_file())


def _xy(pose) -> str:
    if pose.x is not None:
        return f"({pose.x:g}, {pose.y:g})"
    return f"(lat {pose.lat:g}, lon {pose.lon:g})"


# --------------------------------------------------------------------------- #
# what the flight rails can fly: one refusal for the desktop's rail and the hil rig
# --------------------------------------------------------------------------- #

SITL_STUB = "sitl/ros2_vla_stub_node.py"
RAIL_BACKENDS = ("sitl", "hil")
# Below this start altitude ArduCopter's take-off and land detector cannot
# hold the start: the library's 0.5 m cell starts "on the ground".
RAIL_MIN_START_UP_M = 1.0
RAIL_FENCE_PARM = ROOT / "sitl" / "fence" / "guardrail_fence.parm"
# SITL's ground altitude (start_sitl.sh's CMAC default; guardrail/terrain.py
# FlatGround), the altitude of every SITL_HOME the harness boots.
RAIL_HOME_ALT_M = 584.0
# The node's reach test, at the harness's goal tolerance (the goto law stops
# commanding inside it, so the node's 2 m default would never fire).
RAIL_REACH_M = GOAL_TOL_M
# A rail flight's time cap is the scenario's (ticks x dt) plus the Shield
# node's Loiter operator timeout (FSMConfig's, the node's default) plus this
# margin. An episode that ends in Loiter with that timeout still running is a
# CENSORED fail-safe (docs/RUBRIC-failsafe.md section 3), and the library
# sizes each cell's ticks so the IDEAL vehicle's Loiters resolve. On the
# rails the autopilot lags (about 0.45 s, 1.75 m/s^2), so a Shield can enter
# Loiter where the ideal vehicle never does: the hil shakedown shk03 (7 Nov,
# nfz-head-on, 30 s cap) entered Loiter at 3.4 s and ended there, censored.
# A mission that reaches its target still ends at the target.
RAIL_RESOLVE_MARGIN_S = 5.0
# THE LENGTHENED HIL CAMPAIGN (2026-11-14, WP3-15). `--hil-max-s-add S` flies
# every mission of a hil campaign S seconds past its rail cap (rail_max_s), so
# that a cell whose Loiter is still timing out when the rail's own length runs
# out (docs/RUBRIC-failsafe.md section 3: censored, "fly it longer") can show
# its outcome. The scenario's own cap (`max_s`) is unchanged: the performance
# measures are still read over it (rail_window), the fail-safe and the safety
# invariants over the whole, longer flight, as on every rail run. The
# lengthening is recorded in the campaign's sweep.json (`_scope.hil`), in every
# mission's scenario record (`max_s_add`, `rail_max_s_profile`,
# `flight_length`, a family row of its own) and in tools/kpi_report.py's
# inputs. A lengthened campaign flies another flight than the profile's, so it
# is a SEPARATE campaign: reported beside the KPI campaign, never merged into
# its figure (kpi_report refuses it as a supplement, --rescore-rail refuses a
# table that mixes flight lengths). It cannot help a run whose FSM stopped
# stepping because the autopilot took the aircraft (a GeoFence RTL): that
# flight ends TAKEOVER_GRACE_S after the takeover whatever its cap, and the
# censored reading names the takeover (label_failures_keyed).
HIL_MAX_S_ADD_MAX = 600.0


def rail_max_s(max_s: float, add: float = 0.0) -> float:
    """The rail's --max-s for a scenario whose own cap is `max_s`; `add`
    seconds more on a lengthened hil campaign (--hil-max-s-add)."""
    from guardrail.fsm import FSMConfig
    return (float(max_s) + float(FSMConfig().loiter_timeout_s or 0.0) + RAIL_RESOLVE_MARGIN_S
            + float(add or 0.0))


def check_hil_max_s_add(v) -> float:
    """--hil-max-s-add as a number of seconds: finite, above 0 and at most
    HIL_MAX_S_ADD_MAX (a mission's container times out at its cap plus
    HilConfig.mission_margin_s, and an hour-long Loiter is not a test).
    Raises ValueError otherwise."""
    try:
        x = float(v)
    except (TypeError, ValueError):
        raise ValueError(f"--hil-max-s-add {v!r}: not a number of seconds") from None
    if not (math.isfinite(x) and 0.0 < x <= HIL_MAX_S_ADD_MAX):
        raise ValueError(f"--hil-max-s-add {v!r}: must be finite, above 0 and at most "
                         f"{HIL_MAX_S_ADD_MAX:g} s")
    return x


def flight_max_s_add(record: dict | None) -> float:
    """How many seconds past its rail cap a rail run was flown, from its
    scenario record (rail_meta's `max_s_add`; a score document's `scenario`,
    or a `rail_window`): 0.0 for a run flown at the profile's own length -
    every run before the option existed, and every one since without it."""
    v = (record or {}).get("max_s_add")
    try:
        x = float(v) if v is not None else 0.0
    except (TypeError, ValueError):
        return 0.0
    return x if math.isfinite(x) and x > 0.0 else 0.0


def hil_flight_length(add: float) -> dict:
    """A hil campaign's flight length as its sweep.json records it
    (`_scope.hil.flight_length`): the profile's own, or lengthened by `add`
    seconds past the rail's cap, with the merge rule beside it."""
    from guardrail.fsm import FSMConfig
    rail = (f"the scenario's max_s + the Loiter operator timeout "
            f"({float(FSMConfig().loiter_timeout_s or 0.0):g} s) + {RAIL_RESOLVE_MARGIN_S:g} s")
    if not add:
        return {"lengthened": False, "max_s_add": 0.0, "cap": rail}
    return {"lengthened": True, "max_s_add": float(add), "cap": f"{rail} + {float(add):g} s",
            "merge": ("a separate campaign: reported beside the KPI campaign at the "
                      "profile's length, never merged into its figure (the flight differs); "
                      "tools/kpi_report.py refuses it as a supplement and --rescore-rail a "
                      "table that mixes flight lengths"),
            "family_suffix": lengthened_suffix(add)}


def lengthened_suffix(add: float) -> str:
    """The family term of a lengthened run ("/flown-longer-35s"), "" at the
    profile's length: a lengthened run never shares a KPI row with the
    profile's flight of the same cell (tools/kpi_report.py scenario_family
    writes the same term)."""
    return f"/flown-longer-{float(add):g}s" if add else ""
# What each node must declare for what the harness sends it (check_rail_
# flags). Both get every argument (the entrypoint's mission role and
# run_ros2_demo.sh hand the same list to the Shield node and the VLA stub)
# and drop the other's with parse_known_args, so a flag NEITHER declares
# would vanish without a word.
RAIL_SHIELD_FLAGS = ("--seed", "--bundle", "--expected-hash", "--policy", "--dev-yaml",
                     "--start", "--target", "--no-target", "--reach-m", "--speed",
                     "--max-s", "--subject", "--subject-class", "--clock-start",
                     "--events", "--scenario-meta", "--world",
                     # A declared ground (X-11, 2026-11-18): rail_ground_args.
                     "--ground", "--ground-sha256", "--ground-home-msl")
RAIL_STUB_FLAGS = ("--seed", "--bundle", "--policy", "--start", "--target", "--no-target",
                   "--speed", "--pilot", "--action",
                   # The stub loads the policy too: an MSL rule passes its
                   # flight gate only over the same named ground.
                   "--ground", "--ground-sha256")
# What the VLA stub must also read when it is sent a prefix arm (`--prefix`,
# 2026-11-16): the arm, and the clock and flight length its CSP is compiled
# and checked static over. Checked only for a mission that carries --prefix,
# so every other mission's check is the one it was.
RAIL_STUB_PREFIX_FLAGS = ("--prefix", "--clock-start", "--max-s")
# The Shield node's REST hot-apply listener (sitl/ros2_shield_node.py,
# "HOT-APPLY ON THE FLYING SHIELD"). A campaign sends none of these; the gRPC
# harness's rail episode (guardrail/harness_service.py, 2026-11-10) asks for
# the listener through run_hil / run_sitl's `extra_args`, which may hold these
# flags and no other: a mission-shaping flag (--start, --pilot, ...) handed in
# beside the scenario's own would fly another mission under its name.
RAIL_HOTAPPLY_FLAGS = ("--hotapply-port", "--hotapply-host", "--hotapply-pki",
                       "--hotapply-server-cert", "--hotapply-client")
# The Shield node's forecast switches a campaign (--rail-arc-forecast) or the
# harness may add through the same `extra_args` (2026-11-17, WP3-04 /
# ARCH-04), with the values each may take. Opt-in: a campaign without the
# option sends none, and its missions are the ones they were. The node
# records what it flew (metrics.json shield_forecast.arc), and
# arc_forecast_problems refuses a run whose record disagrees - a node from an
# older bundle would drop the flag without a word (parse_known_args).
RAIL_FORECAST_FLAGS = {"--arc-forecast": ("on", "off")}
# Where a campaign's per-mission files (events, scenario record) go, under
# the checkout's demo/out (gitignored; the Orin mounts demo/out writable).
CAMPAIGN_DIR = "demo/out/_campaign"


def _fence_ceiling(policy) -> tuple[float | None, float]:
    """(the autopilot's FENCE_ALT_MAX as the Shield node leaves it for this
    policy, FENCE_MARGIN): the .parm file's, raised to the policy's ceiling
    plus the headroom when it sits below (mavlink_adapter_node.fence_alt_plan,
    which the node applies on SITL)."""
    if str(_REPO / "sitl") not in sys.path:
        sys.path.insert(0, str(_REPO / "sitl"))
    from mavlink_adapter_node import (FENCE_MARGIN_M, fence_alt_plan,
                                      policy_ceiling_m, read_parm)
    parm = read_parm(RAIL_FENCE_PARM)
    alt_max = parm.get("FENCE_ALT_MAX")
    ceiling, _ = policy_ceiling_m(policy)
    plan = fence_alt_plan(ceiling, alt_max)
    if plan["raise_to"] is not None and alt_max is not None:
        alt_max = plan["raise_to"]
    return alt_max, float(parm.get("FENCE_MARGIN", FENCE_MARGIN_M))


# --------------------------------------------------------------------------- #
# THE SIMULATOR'S STRESSORS ON THE RAILS (2026-11-07, X-14)
# --------------------------------------------------------------------------- #
#
# Until this date rail_refusal refused every stressor ("no SIM_* wind or GPS
# stressor is wired"). ArduCopter 4.5.7's SITL (the pinned tree, ~/ardupilot
# in WSL, read for this; paths under libraries/) realises three of them, set
# on the SIMULATOR's side by sitl/sitl_stressors.py - a process beside SITL on
# the desktop, on SITL's SERIAL0 - and never by the Shield node:
#
#   wind_speed_mps  SIM_WIND_SPD (m/s; SITL/SITL.cpp:74), 0 at boot and the
#                   cell's speed at the episode's t = 0. SITL low-passes it
#                   0.95 / 0.05 per physics update (AP_HAL_SITL/
#                   SITL_State.cpp:437): a step. SIM_WIND_T = 1 at boot
#                   ("none", SITL.cpp:203): the default 0, the square law,
#                   scales the wind by sqrt(altitude / SIM_WIND_T_ALT 60 m)
#                   below 60 m (SITL_State.cpp:443-446) - half the speed at
#                   15 m, which is what the 2 Nov pymavlink wind runs flew.
#                   SIM_WIND_TURB = 0 at boot (gusts are refused, below).
#   wind_dir_deg    SIM_WIND_DIR (degrees true; SITL.cpp:80, default 180) at
#                   boot. It is the direction the wind comes FROM: the air
#                   moves along -(cos, sin) of it (SITL/SIM_Aircraft.cpp:
#                   779-804, wind_ef negated), so SIM_WIND_DIR = (toward +
#                   180) mod 360 (sim_wind_dir_deg). toward is the scenario's
#                   wind_dir_deg turned with mission.rotate_deg, or, when the
#                   scenario leaves it None, the harness's own draw for the
#                   seed (random.Random(f"{seed}:wind").uniform(0, 360), the
#                   first draw of _episode's wind generator): rail_wind.
#   gps_dropout_s   SIM_GPS_DISABLE (0/1; SITL.cpp:561, var_gps, a "" sub-
#                   group of SIM_, SITL.cpp:183: SIM_GPS_DISABLE in 4.5.7, not
#                   4.6's SIM_GPS1_DISABLE), 0 at boot, 1 at t0 + start, 0 at
#                   t0 + end. SITL's GPS then reports no lock (SITL/
#                   SIM_GPS.cpp:333; the u-blox stream carries fix type 0 and
#                   3 satellites, SIM_GPS_UBLOX.cpp:202-251), GPS_RAW_INT's
#                   fix type drops below 3 and the Shield node's fix_ok goes
#                   False (sitl/ros2_shield_node.py fix_ok_from).
#   start_jitter_m  the SITL home: rail_mission draws the start exactly as the
#                   harness does (_resolve_mission with random.Random(
#                   f"{seed}:jitter")), so SITL_HOME and --start carry the
#                   same offset as the headless episode of the same seed.
#   battery_critical_at_s (2026-11-17, WP3-07 G6; RAIL_BATTERY) ArduPilot's
#                   own battery monitor declares CRITICAL at the cell's time,
#                   and the Shield node's RTL watch reads it (BATTERY_STATUS
#                   charge_state). SITL's pack is SIM_BATT_VOLTAGE (SITL.cpp:94;
#                   SIM_Frame.cpp:519 defaults it to the frame's maxVoltage,
#                   12.6 V for the rig's quad, and :608 makes a change the
#                   battery's voltage at once): set to that at boot, and to
#                   drop_v at t0 + battery_critical_at_s - low_timer_s. The
#                   monitor declares CRITICAL once the voltage has stayed
#                   below BATT_CRT_VOLT for more than BATT_LOW_TIMER s
#                   (AP_BattMonitor_Backend.cpp:135-139), only while armed
#                   (AP_BattMonitor.cpp:791), and keeps it (the failsafe
#                   level only rises): so the onset lands on the cell's time
#                   plus a monitor update. The copter's BATT_CRT_VOLT is 0
#                   (disabled), so the cell gives the SIMULATED vehicle its
#                   battery fail-safe setup (the plan's `firmware`:
#                   BATT_CRT_VOLT 10.1, the 10.0 V drop and no low or
#                   critical action of the autopilot's own as in ArduCopter's
#                   autotest BatteryFailsafe, so the FSM's G6 is what lands
#                   the return; BATT_LOW_VOLT 10.5 is the copter default;
#                   BATT_LOW_TIMER 1 s is this cell's choice, a lab
#                   vehicle-setup choice of 2026-11-17 - the default is
#                   10 s), restored after the mission
#                   (sitl/sitl_stressors.py, THE BATTERY CELL'S FAIL-SAFE
#                   SETUP: set only over a loopback link to an autopilot that
#                   answers SIM_BATT_VOLTAGE). At 10 V the quad's motors give
#                   (10 / 12.6)^2 = 0.63 of their full thrust (SIM_Motor.cpp:
#                   voltage_scale); whether the rig's vehicle holds its
#                   return on that is for the shakedown flight to show.
#                   OPT-IN (RAIL_BATTERY_OPT_IN): unflown on the rig, so a
#                   campaign flies it only from a profile with
#                   `rail_battery: true` (fsm_edges alone); every other
#                   campaign, smoke and nightly included, refuses it.
#
# What a rail flight of such a cell is NOT: the harness's `--wind-model`. The
# wind acts through ArduPilot's own physics - its drag model, its position
# controller's real authority and transients - and the dropout through its
# own EKF: the pose the Shield is handed is EKF3's inertial dead reckoning
# (not the frozen last fix the headless harness hands it), and after
# posRetryTimeNoVel_ms (7 s, AP_NavEKF3.h:466) without GPS the EKF stops
# reporting an absolute position (AP_NavEKF3_Control.cpp:367-385, 743) and
# ArduCopter's EKF failsafe (ArduCopter/ekf_check.cpp, 1 s at 10 Hz,
# FS_EKF_ACTION 1) lands the aircraft. docs/RUNBOOK-orin-hil.md step 14 lists
# the cells whose labels SITL's physics may not reach.
#
# Refused (RAIL_STRESS_REFUSALS): gps_noise_m, latency_ticks, gust_mps, and a
# dropout with fix_quality_reported false. battery_critical_at_s is refused
# too unless the campaign opted in (RAIL_BATTERY_OPT_IN; until 2026-11-17 it
# was refused outright).
#
# Fail closed (rail_stress_check, after the mission): the simulator side's
# record (this launch's nonce; every boot parameter set and read back), the
# Shield node's own read-back at take-off, and every scheduled change applied
# within RAIL_STRESS_TOL_S of its time on the node's mission clock (the
# autopilot's PARAM_VALUE broadcast as the node received it; the simulator
# side's own timing only where the node saw no parameter event at all), a
# dropout reaching the node's fix_ok within RAIL_DROPOUT_ONSET_MAX_S, a
# jittered home where SITL was told to boot. Anything else is not the cell's
# flight: the episode errors, it is never scored under the cell's name.

RAIL_STRESS_DIR = "demo/out/_stress"
# A scheduled change counts as applied at its time when the node saw it
# within these bounds of the plan (early: clock rounding only - the simulator
# side never applies a change before the node's t = 0 plus its time).
RAIL_STRESS_TOL_S = 0.2
RAIL_STRESS_EARLY_S = 0.05
# The GPS denial must reach the Shield's input (a row with fix_ok False)
# within this of the window's start: SITL's GPS at 5 Hz, GPS_RAW_INT at the
# node's 10 Hz stream, the 10 Hz tick.
RAIL_DROPOUT_ONSET_MAX_S = 1.0
# A jittered start: the autopilot's home within this of --start (the node's
# own home check allows 5 m, more than any jitter in the library).
RAIL_JITTER_HOME_TOL_M = 0.25
SIM_PARAM_TOL = 1e-3
# SITL's stressor parameters when nothing is stressed (SITL.cpp defaults; the
# simulator side sets none of them for such a cell).
SIM_NEUTRAL = {"SIM_WIND_SPD": 0.0, "SIM_WIND_TURB": 0.0, "SIM_GPS_DISABLE": 0.0}
RAIL_STRESS_REFUSALS = {
    "gps_noise_m": (
        "SITL has no equivalent: 4.5.7's SIM_GPS_NOISE is a slow sine on the GPS "
        "altitude alone (SITL/SIM_GPS.cpp:353), and horizontal noise put into SITL's "
        "GPS (SIM_GPS_GLITCH) would be fused by the EKF that also flies the aircraft - "
        "the Shield would see the EKF's filtered estimate and the airframe would chase "
        "it, where the cell's labels were derived for white noise on the Shield's view "
        "alone"),
    "latency_ticks": (
        "the rail's own pose latency is real and recorded per row (pose_age_ms, "
        "e2e_ms); SIM_GPS_LAG_MS delays the GPS into the EKF, which compensates its "
        "own GPS delay, not the state the Shield is handed"),
    "gust_mps": (
        "SITL's SIM_WIND_TURB is not a fair stand-in: its azimuth is redrawn on every "
        "physics step and its speed is a low-pass of SITL's own random draws "
        "(SITL/SIM_Aircraft.cpp:785-800), acting on the airframe through drag, where "
        "the harness's gust is a per-tick Gaussian ground-velocity disturbance on each "
        "axis drawn from the episode's seed - neither the process nor the draw is the "
        "cell's"),
}
# battery_critical_at_s on the rails (2026-11-17; the mapping above): the
# simulated vehicle's battery fail-safe setup and its pack. Until this date the
# hook was refused: SITL's battery reaches CRITICAL only through BATT_CRT_VOLT /
# BATT_CRT_MAH (0, disabled, by default) and BATT_LOW_TIMER (10 s), which the
# simulator side never set. low_timer_s 1 is the cell's own choice (a lab
# vehicle-setup choice of 2026-11-17; ArduPilot's default is 10 s), low_v the
# copter default, crit_v and drop_v ArduCopter's autotest BatteryFailsafe's.
RAIL_BATTERY = {"full_v": 12.6, "drop_v": 10.0, "low_v": 10.5, "crit_v": 10.1,
                "low_timer_s": 1}
# OPT-IN (2026-11-17): the battery cell has never flown on the rig, and it
# writes BATT_* parameters to the simulated autopilot, so a rail campaign
# flies it only when its profile says `rail_battery: true` (fsm_edges, which
# bears no KPI). smoke, nightly and broad list failsafe-low-battery too:
# without the opt-in their rail campaigns refuse it as before, so a KPI
# campaign never gains a cell that has not flown. Lift this for the KPI
# profiles only after a shakedown flight (docs/RUNBOOK-orin-hil.md, fe_batt01)
# shows CRITICAL reaching the RTL watch, G11 > G6 > G10 and the restore.
RAIL_BATTERY_OPT_IN = (
    "the battery cell is opt-in on the rails (RAIL_BATTERY_OPT_IN): it is unflown on "
    "the rig and sets the simulated vehicle's BATT_* fail-safe parameters, so only a "
    "profile with `rail_battery: true` (experiments/profiles/fsm_edges.yaml) flies it")
# The battery's CRITICAL must reach the Shield node (its RTL watch's first
# CRITICAL report, metrics.json rtl_watch.battery.failed_spells) within this
# of the cell's time: the monitor's update after BATT_LOW_TIMER, BATTERY_STATUS
# on the node's stream, the marker's delay.
RAIL_BATTERY_ONSET_MAX_S = 1.5


def _sitl_stressors():
    """sitl/sitl_stressors.py, the simulator side: its plan schema, the t = 0
    marker and the stressor record the pymavlink rail shares (imported from
    the checkout this file is in; it needs no pymavlink to be read)."""
    if str(_REPO / "sitl") not in sys.path:
        sys.path.insert(0, str(_REPO / "sitl"))
    import sitl_stressors
    return sitl_stressors


def sim_wind_dir_deg(toward_deg: float) -> float:
    """SITL's SIM_WIND_DIR for a wind blowing TOWARD `toward_deg` (degrees
    clockwise from true North, the harness's convention): the direction it
    comes FROM, (toward + 180) mod 360 (module comment, "THE SIMULATOR'S
    STRESSORS ON THE RAILS")."""
    return round((float(toward_deg) + 180.0) % 360.0, 6)


def rail_wind(spec: ScenarioSpec, seed: int = 0) -> dict | None:
    """The steady wind of one episode as the harness flies it (_episode):
    the declared direction turned with the mission, or the seed's own draw
    when the scenario leaves it None; None with no wind."""
    s = spec.stress
    if not s.wind_speed_mps > 0:
        return None
    drawn = s.wind_dir_deg is None
    toward = (random.Random(f"{seed}:wind").uniform(0.0, 360.0) if drawn
              else float(s.wind_dir_deg) + float(spec.mission.rotate_deg))
    toward %= 360.0
    return {"speed_mps": float(s.wind_speed_mps), "toward_deg": round(toward, 6),
            "drawn_from_seed": drawn, "sim_wind_dir_deg": sim_wind_dir_deg(toward)}


def start_jitter(spec: ScenarioSpec, seed: int = 0) -> tuple[float, float]:
    """The start's (dx, dy) in the mission's frame, drawn as _resolve_mission
    draws it for `seed` (random.Random(f"{seed}:jitter"), x then y); (0, 0)
    with no jitter."""
    j = spec.stress.start_jitter_m
    if not j > 0:
        return 0.0, 0.0
    rng = random.Random(f"{seed}:jitter")
    return rng.uniform(-j, j), rng.uniform(-j, j)


def rail_stress_refusals(spec: ScenarioSpec, defaults=None, *,
                         rail_battery: bool = False) -> list[str]:
    """The stressors of `spec` neither rail realises, each with its reason
    (RAIL_STRESS_REFUSALS); [] when every stressor it declares is realised.
    The battery hook is realised only with `rail_battery` (the campaign's
    opt-in, RAIL_BATTERY_OPT_IN); refused otherwise."""
    s = spec.stress
    out = [f"{k} {getattr(s, k)}: {why}" for k, why in RAIL_STRESS_REFUSALS.items()
           if getattr(s, k) not in (None, 0, 0.0)]
    if s.battery_critical_at_s is not None and not rail_battery:
        out.append(f"battery_critical_at_s {s.battery_critical_at_s}: {RAIL_BATTERY_OPT_IN}")
    if s.gps_dropout_s is not None and not s.fix_quality_reported:
        out.append("fix_quality_reported false: the rail reports its fix "
                   "(/mavros/global_position/raw/fix), so the blind arm stays headless")
    if s.gps_dropout_s is not None:
        dfl = _as_defaults(defaults)
        t_end = ((spec.ticks or dfl.ticks) - 1) * (spec.dt or dfl.dt)
        if s.gps_dropout_s[0] > t_end + 1e-9:
            out.append(f"gps_dropout_s starts at {s.gps_dropout_s[0]:g} s, after the run's "
                       f"last tick ({t_end:g} s; the headless harness refuses it too)")
    return out


def rail_stress_plan(spec: ScenarioSpec, seed: int = 0, defaults=None, *,
                     run_id: str | None = None, nonce: str | None = None) -> dict | None:
    """What the simulator side sets for one episode (sitl/sitl_stressors.py,
    its PLAN_SCHEMA): the boot parameters (set and read back before READY)
    and the changes at the episode's t = 0 plus their time. None when the
    cell needs nothing set in SITL (no wind, no dropout, no battery; a
    jittered start is in SITL_HOME alone). The wind's boot values are the
    pymavlink rail's rule too (sitl_stressors.with_uniform_wind: SIM_WIND_T
    = 1 with a speed). A battery cell (since 2026-11-17) also carries
    `firmware`: the simulated vehicle's battery fail-safe setup
    (RAIL_BATTERY), set at boot and restored after the mission."""
    SST = _sitl_stressors()
    wind = rail_wind(spec, seed)
    drop = spec.stress.gps_dropout_s
    batt = spec.stress.battery_critical_at_s
    if wind is None and drop is None and batt is None:
        return None
    boot: list[dict] = []
    sched: list[dict] = []
    firmware: list[dict] = []
    if wind is not None:
        # The profile by the rule the pymavlink rail applies too (one meaning
        # of a wind speed on both).
        profile = SST.with_uniform_wind({"SIM_WIND_SPD": wind["speed_mps"]})["SIM_WIND_T"]
        boot += [
            {"param": "SIM_WIND_T", "value": profile,
             "why": "no altitude profile: the cell's speed at every altitude (the "
                    "default square law halves it at 15 m)"},
            {"param": "SIM_WIND_TURB", "value": 0,
             "why": "no turbulence: gusts are not realised on the rails"},
            {"param": "SIM_WIND_DIR", "value": wind["sim_wind_dir_deg"],
             "why": f"the direction the wind comes FROM: toward {wind['toward_deg']:g} "
                    f"deg + 180"},
            {"param": "SIM_WIND_SPD", "value": 0,
             "why": "still air until the episode's t = 0"}]
        sched.append({"t": 0.0, "param": "SIM_WIND_SPD", "value": wind["speed_mps"],
                      "field": "wind_speed_mps"})
    if drop is not None:
        boot.append({"param": "SIM_GPS_DISABLE", "value": 0,
                     "why": "GPS on until the window opens"})
        sched += [{"t": float(drop[0]), "param": "SIM_GPS_DISABLE", "value": 1,
                   "field": "gps_dropout_s start"},
                  {"t": float(drop[1]), "param": "SIM_GPS_DISABLE", "value": 0,
                   "field": "gps_dropout_s end", "restore": True}]
    if batt is not None:
        b = RAIL_BATTERY
        firmware += [
            {"param": "BATT_LOW_VOLT", "value": b["low_v"],
             "why": "above the critical voltage, as ArduPilot's arming check requires"},
            {"param": "BATT_CRT_VOLT", "value": b["crit_v"],
             "why": "the copter default 0 never declares CRITICAL"},
            {"param": "BATT_LOW_TIMER", "value": b["low_timer_s"],
             "why": f"CRITICAL {b['low_timer_s']} s after the voltage falls below "
                    f"BATT_CRT_VOLT (the default 10 s; 0 disables it)"},
            {"param": "BATT_FS_LOW_ACT", "value": 0,
             "why": "no action of the autopilot's own at LOW"},
            {"param": "BATT_FS_CRT_ACT", "value": 0,
             "why": "no action of the autopilot's own at CRITICAL: the FSM's G6 lands "
                    "the return"}]
        boot.append({"param": "SIM_BATT_VOLTAGE", "value": b["full_v"],
                     "why": "a full pack until the drop (the quad model's maxVoltage)",
                     "restore_after": True})
        sched.append({"t": max(0.0, float(batt) - b["low_timer_s"]),
                      "param": "SIM_BATT_VOLTAGE", "value": b["drop_v"],
                      "field": "battery_critical_at_s"})
    sched.sort(key=lambda it: it["t"])
    return {"schema": SST.PLAN_SCHEMA, "run_id": run_id, "run_nonce": nonce,
            "scenario_id": spec.scenario_id, "seed": int(seed),
            "boot": boot, "schedule": sched, "firmware": firmware,
            "battery_critical_at_s": None if batt is None else float(batt),
            "wind": wind, "gps_dropout_s": None if drop is None else list(drop),
            "align_tol_s": RAIL_STRESS_TOL_S,
            "written_by": "experiments/sweep_scenarios.py rail_stress_plan"}


def write_stress_plan(plan: dict, root: Path | None = None) -> tuple[Path, Path]:
    """The plan, under RAIL_STRESS_DIR/<run id>/plan.json on THIS host (the
    desktop: the simulator side reads it in WSL, and nothing of it goes to
    the Orin), and where its record will be: (plan path, record path). A
    record left by an earlier launch is never read as this one's
    (rail_stress_check matches the nonce)."""
    d = (root or ROOT) / RAIL_STRESS_DIR / str(plan["run_id"])
    d.mkdir(parents=True, exist_ok=True)
    p = d / "plan.json"
    p.write_text(json.dumps(_json_safe(plan), indent=1) + "\n", encoding="utf-8",
                 newline="\n")
    return p, d / "record.json"


def read_stress_record(path: Path | None) -> dict | None:
    """The simulator side's record, or None when it left none (or an
    unreadable one)."""
    if path is None or not Path(path).is_file():
        return None
    try:
        doc = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return doc if isinstance(doc, dict) else None


def _num_close(a, b, tol: float = SIM_PARAM_TOL) -> bool:
    try:
        return a is not None and b is not None and abs(float(a) - float(b)) <= tol
    except (TypeError, ValueError):
        return False


def _rail_battery_check(plan: dict, record: dict | None, metrics: dict,
                       last_t: float | None, summary: dict,
                       rows: list[dict] | None = None) -> list[str]:
    """A battery cell's part of rail_stress_check (2026-11-17): the
    simulated vehicle's battery fail-safe setup set and read back by the
    simulator side, the ones the Shield node reads at take-off (its RTL
    watch's BATT_CRT_VOLT and BATT_FS_CRT_ACT, metrics.json
    rtl_watch.params) as planned, ArduPilot's CRITICAL reaching the
    node's RTL watch within RAIL_BATTERY_ONSET_MAX_S of the cell's time (its
    first CRITICAL-or-worse report, rtl_watch.battery.failed_spells), and
    the FSM in RTL on the node's last row before that report (`rows`'
    fsm_state_after): a CRITICAL that met no RTL is the stress not realised,
    and the flight errors rather than fail the cell's G6 gate. Fills
    summary["battery"]; returns the problems."""
    SST = _sitl_stressors()
    problems: list[str] = []
    fw = (record or {}).get("firmware") if isinstance((record or {}).get("firmware"),
                                                      dict) else None
    rw = metrics.get("rtl_watch") if isinstance(metrics.get("rtl_watch"), dict) else {}
    node_params = rw.get("params") if isinstance(rw.get("params"), dict) else None
    out: dict[str, Any] = {"firmware": [], "onset_max_s": RAIL_BATTERY_ONSET_MAX_S,
                           "restored": ((record or {}).get("restore") or {}).get("params")}
    if record is not None and (fw is None or not fw.get("ok")):
        guard = (fw or {}).get("guard") or {}
        problems.append("the simulator side did not set and read back the simulated "
                        "vehicle's battery fail-safe setup"
                        + (f" ({guard['refused']})" if guard.get("refused") else ""))
    sim_fw = {p.get("param"): p.get("read_back") for p in (fw or {}).get("params") or []
              if isinstance(p, dict)}
    for it in plan.get("firmware") or []:
        name, want = it["param"], it["value"]
        node_v = None if node_params is None else node_params.get(name)
        out["firmware"].append(SST.stressor_entry(
            name, want, sim_fw.get(name), t=None,
            when="boot (the simulated vehicle's battery fail-safe setup)",
            node_read_back=node_v if node_params is not None and name in node_params
            else "not read by the node"))
        if node_params is not None and name in node_params \
                and not SST.read_back_ok(want, node_v):
            problems.append(f"the autopilot's {name} at take-off is {node_v}, not the plan's "
                            f"{want:g}")
    if node_params is None:
        problems.append("the Shield node recorded no RTL watch parameters (metrics.json "
                        "rtl_watch.params): the battery fail-safe setup is unconfirmed")
    at = plan.get("battery_critical_at_s")
    if at is not None:
        bat = rw.get("battery") if isinstance(rw.get("battery"), dict) else {}
        spells = sorted(float(s["t_start"]) for s in bat.get("failed_spells") or []
                        if isinstance(s, dict) and s.get("t_start") is not None
                        and float(s["t_start"]) >= float(at) - RAIL_STRESS_EARLY_S)
        onset = spells[0] if spells else None
        out.update(battery_critical_at_s=at, first_critical_t=onset,
                   onset_s=None if onset is None else round(onset - float(at), 4),
                   charge_states_seen=bat.get("charge_states_seen"))
        if onset is None or onset - float(at) > RAIL_BATTERY_ONSET_MAX_S:
            if last_t is not None and last_t < float(at) + RAIL_BATTERY_ONSET_MAX_S \
                    and onset is None:
                out["note"] = (f"the episode ended at {last_t:g} s, within "
                               f"{RAIL_BATTERY_ONSET_MAX_S:g} s of the battery's time")
            else:
                problems.append(f"battery_critical_at_s: ArduPilot's CRITICAL from t={at:g} s "
                                f"did not reach the Shield node's RTL watch within "
                                f"{RAIL_BATTERY_ONSET_MAX_S:g} s (first CRITICAL report: "
                                f"{onset})")
        if onset is not None:
            # The stress is G6's only while the FSM is returning: the node's
            # battery watch records CRITICAL whatever the FSM's state, and
            # ArduPilot keeps checking its failsafes while armed (an RTL that
            # already reached home and is descending, say). A CRITICAL that
            # did not meet an RTL is not the cell's flight, never a failed
            # gate.
            before = [r for r in rows or []
                      if r.get("t") is not None and float(r["t"]) < onset
                      and "fsm_state_after" in r]
            state = before[-1]["fsm_state_after"] if before else None
            out["fsm_state_at_critical"] = state
            if not before:
                problems.append("battery_critical_at_s: no row before ArduPilot's first "
                                "CRITICAL report carries the FSM's state (fsm_state_after): "
                                "the stress is not shown to have met an RTL")
            elif state != "RTL":
                problems.append(f"battery_critical_at_s: stress not realised on an RTL - the "
                                f"FSM was in {state} at ArduPilot's first CRITICAL report "
                                f"(t={onset:g} s)")
    summary["battery"] = out
    return problems


def rail_stress_check(spec: ScenarioSpec, plan: dict | None, record: dict | None,
                      metrics: dict, rows: list[dict], seed: int = 0
                      ) -> tuple[list[str], dict]:
    """Was the cell's stress realised on this flight? (problems, summary).

    Any problem means the flight is not the cell's and must not be scored
    under its name (run_hil / run_sitl raise). Checked: a jittered start's
    home (the node's home_check, within RAIL_JITTER_HOME_TOL_M); with no
    plan, SITL at its neutral stressor values (SIM_NEUTRAL, as the node read
    them back at take-off); with a plan, the simulator side's record (this
    launch's nonce, every boot parameter set and read back), the node's own
    read-back of every boot parameter, every scheduled change applied within
    [-RAIL_STRESS_EARLY_S, +RAIL_STRESS_TOL_S] of its time - on the node's
    mission clock (`sim_params.events`), or, where the node recorded no
    in-flight parameter event at all, on the simulator side's own t0 - and a
    dropout reaching the node's fix_ok within RAIL_DROPOUT_ONSET_MAX_S. A
    window's end that falls after the episode's last row may go unapplied
    (the episode ended in the dropout, as a headless one can). A battery
    cell (since 2026-11-17): its fail-safe setup read back and ArduPilot's
    CRITICAL reaching the node within RAIL_BATTERY_ONSET_MAX_S
    (_rail_battery_check, summary["battery"]).

    summary["entries"] is every SIM_* parameter the simulator side set, in
    the record the pymavlink rail writes into metrics.json `sim_stressors`
    (sitl_stressors.stressor_entry: param, set, read_back, applied) with its
    time: a boot parameter read back by the Shield node at take-off (t
    None), a scheduled one with its plan time, the autopilot's echo to the
    simulator side, the time the node saw it and the error."""
    SST = _sitl_stressors()
    problems: list[str] = []
    sim = metrics.get("sim_params") if isinstance(metrics.get("sim_params"), dict) else {}
    at = sim.get("at_takeoff") if isinstance(sim.get("at_takeoff"), dict) else {}
    node_vals = at.get("values") if at.get("sitl") else None
    summary: dict[str, Any] = {"plan": None if plan is None else {
        "boot": plan["boot"], "schedule": plan["schedule"], "wind": plan.get("wind"),
        "gps_dropout_s": plan.get("gps_dropout_s"),
        **({"firmware": plan["firmware"],
            "battery_critical_at_s": plan.get("battery_critical_at_s")}
           if plan.get("firmware") else {})},
        "node_at_takeoff": node_vals, "align_tol_s": RAIL_STRESS_TOL_S, "entries": []}
    dx, dy = start_jitter(spec, seed)
    if dx or dy:
        hc = metrics.get("home_check") if isinstance(metrics.get("home_check"), dict) else {}
        miss = hc.get("miss_m")
        summary["start_jitter"] = {"dx_m": round(dx, 4), "dy_m": round(dy, 4),
                                   "home_miss_m": miss,
                                   "tolerance_m": RAIL_JITTER_HOME_TOL_M}
        if not hc.get("checked") or miss is None or float(miss) > RAIL_JITTER_HOME_TOL_M:
            problems.append(f"start jitter ({dx:+.3f}, {dy:+.3f}) m not confirmed: the "
                            f"autopilot's home is {miss} m from the jittered --start "
                            f"(allowed {RAIL_JITTER_HOME_TOL_M:g} m)")
    if plan is None:
        if node_vals:
            off = {k: v for k, v in node_vals.items()
                   if k in SIM_NEUTRAL and v is not None and not _num_close(v, SIM_NEUTRAL[k])}
            if off:
                problems.append(f"SITL flew with stressor parameters this cell does not "
                                f"set: {off}")
        summary["problems"] = problems
        return problems, summary
    if record is None:
        problems.append("the simulator side (sitl/sitl_stressors.py) left no record")
    elif record.get("run_nonce") != plan.get("run_nonce"):
        problems.append(f"the simulator side's record is not this launch's (nonce "
                        f"{record.get('run_nonce')!r})")
        record = None
    elif not (record.get("boot") or {}).get("ok"):
        problems.append(f"the simulator side did not set and read back its boot "
                        f"parameters ({record.get('error')})")
    summary["simulator_side"] = None if record is None else {
        k: record.get(k) for k in ("state", "error", "t0", "markers", "boot")}
    if node_vals is None:
        problems.append("the Shield node read no simulator parameters at take-off "
                        "(metrics.json sim_params.at_takeoff): the boot values are "
                        "unconfirmed")
    else:
        bad = [f"{it['param']} {node_vals.get(it['param'])} (set {it['value']:g})"
               for it in plan["boot"] if not SST.read_back_ok(it["value"],
                                                             node_vals.get(it["param"]))]
        if bad:
            problems.append("SITL's parameters at take-off are not the plan's: "
                            + ", ".join(bad))
    sim_boot = {p.get("param"): p.get("read_back")
                for p in ((record or {}).get("boot") or {}).get("params") or []
                if isinstance(p, dict)}
    for it in plan["boot"]:
        summary["entries"].append(SST.stressor_entry(
            it["param"], it["value"], (node_vals or {}).get(it["param"]), t=None,
            when="boot (set before the mission, read back by the Shield node at take-off)",
            simulator_side_read_back=sim_boot.get(it["param"])))
    events = [e for e in (sim.get("events") or [])
              if isinstance(e, dict) and e.get("t") is not None]
    last_t = float(rows[-1]["t"]) if rows else None
    rec_sched = (record or {}).get("schedule") or []
    applied_out = []
    for i, it in enumerate(plan["schedule"]):
        t_plan = float(it["t"])
        ent = rec_sched[i] if i < len(rec_sched) and isinstance(rec_sched[i], dict) else {}
        seen = sorted(float(e["t"]) for e in events
                      if e.get("param") == it["param"] and _num_close(e.get("value"), it["value"])
                      and float(e["t"]) >= t_plan - RAIL_STRESS_EARLY_S)
        obs = seen[0] if seen else None
        if obs is not None:
            err, source = obs - t_plan, "node (/mavros/param/event, mission clock)"
        elif not events and ent.get("applied") and ent.get("t_echo") is not None:
            err, source = float(ent["t_echo"]) - t_plan, ("simulator side (its t0 from "
                                                         "the node's markers)")
        else:
            err, source = None, None
        ended_first = last_t is not None and last_t < t_plan + RAIL_STRESS_TOL_S
        entry = SST.stressor_entry(
            it["param"], it["value"], ent.get("read_back"), t=t_plan, field=it.get("field"),
            when="scheduled (the episode's t = 0 plus t)",
            t_echo_simulator_side=ent.get("t_echo"), t_observed_node=obs,
            error_s=None if err is None else round(err, 4), measured_by=source)
        if err is None:
            if it.get("restore") and ended_first:
                entry["note"] = (f"the episode ended at {last_t:g} s, before the window's "
                                 f"end")
            elif events and ent.get("applied"):
                problems.append(f"{it.get('field')}: the simulator side applied "
                                f"{it['param']}={it['value']:g} but the Shield node saw no "
                                f"such change")
            else:
                problems.append(f"{it.get('field')}: {it['param']}={it['value']:g} at "
                                f"t={t_plan:g} s was never applied"
                                + (f" (the episode ended at {last_t:g} s)" if ended_first
                                   else ""))
        elif not (-RAIL_STRESS_EARLY_S <= err <= RAIL_STRESS_TOL_S):
            if not (it.get("restore") and ended_first):
                problems.append(f"{it.get('field')}: {it['param']}={it['value']:g} applied "
                                f"{err:+.3f} s from its time t={t_plan:g} s (allowed "
                                f"-{RAIL_STRESS_EARLY_S:g} .. +{RAIL_STRESS_TOL_S:g} s, "
                                f"measured by the {source})")
        applied_out.append(entry)
    summary["entries"] += applied_out
    if plan.get("firmware") or plan.get("battery_critical_at_s") is not None:
        problems += _rail_battery_check(plan, record, metrics, last_t, summary, rows)
    drop = plan.get("gps_dropout_s")
    if drop is not None:
        start = float(drop[0])
        fixes = [r for r in rows if "fix_ok" in r]
        onset = next((float(r["t"]) for r in fixes
                      if r.get("fix_ok") is False and float(r["t"]) >= start - RAIL_STRESS_EARLY_S),
                     None)
        summary["dropout"] = {"window": list(drop),
                              "first_no_fix_t": onset,
                              "onset_s": None if onset is None else round(onset - start, 4),
                              "no_fix_ticks": sum(1 for r in fixes if r.get("fix_ok") is False),
                              "onset_max_s": RAIL_DROPOUT_ONSET_MAX_S}
        if not fixes:
            problems.append("gps_dropout_s: the Shield node reported no GPS fix at all "
                            "(no /mavros/global_position/raw/fix): the denial cannot have "
                            "reached it")
        elif last_t is not None and last_t < start:
            problems.append(f"gps_dropout_s: the episode ended at {last_t:g} s, before the "
                            f"window opened at {start:g} s")
        elif onset is None or onset - start > RAIL_DROPOUT_ONSET_MAX_S:
            if last_t is not None and last_t < start + RAIL_DROPOUT_ONSET_MAX_S:
                # Ended before the denial could reach the Shield: the cell's
                # own gate (gps_dropout_ticks >= 1) fails that flight.
                summary["dropout"]["note"] = (f"the episode ended at {last_t:g} s, within "
                                              f"{RAIL_DROPOUT_ONSET_MAX_S:g} s of the window")
            else:
                problems.append(f"gps_dropout_s: the GPS denial from t={start:g} s did not "
                                f"reach the Shield's fix_ok within "
                                f"{RAIL_DROPOUT_ONSET_MAX_S:g} s (first no-fix tick: {onset})")
    summary["problems"] = problems
    return problems, summary


def rail_refusal(spec: ScenarioSpec, backend: str = "sitl", *, episode=None,
                 policy=None, defaults=None, rail_battery: bool = False) -> str | None:
    """Why a flight rail cannot fly this scenario AS WRITTEN, or None. One
    function for both rails (2026-11-06): the desktop's (`sitl`,
    sitl/run_ros2_demo.sh) and the hil rig's (`hil`, the Orin's mission
    container), which fly the same two nodes with the same arguments.

    Since that date the rails take a scenario's start (--start: SITL boots
    at it), target and altitude, pilot law (the VLA stub's --pilot goto /
    constant / stub_vla), speed, time cap, subject, scenario clock
    (--clock-start) and mid-flight events (--events), and since 2026-11-07
    the simulator's stressors SITL realises (module comment, "THE
    SIMULATOR'S STRESSORS ON THE RAILS": wind and its direction, GPS denial,
    start jitter; since 2026-11-17 the battery hook, opt-in), so a scenario
    is refused only for what neither rail can realise:
      * the stressors SITL has no honest equivalent for (RAIL_STRESS_
        REFUSALS: GPS noise, state latency, gusts), and a dropout flown
        blind (fix_quality_reported false);
      * the battery hook unless `rail_battery` (the campaign's profile has
        `rail_battery: true`; RAIL_BATTERY_OPT_IN);
      * a relabel schedule (the node declares one subject class);
      * a non-finite pilot action (the node refuses to fly or count one:
        the action-contract family stays headless);
      * a start altitude below RAIL_MIN_START_UP_M, or at or above the
        autopilot's FENCE_ALT_MAX less its margin (the backstop would fire
        at take-off);
      * an event the headless harness would refuse too (a `window` with no
        scenario clock, one after the run's last tick);
      * a metre policy (no frame origin) started away from its origin: SITL
        cannot be booted there;
      * with `episode`, a paraphrase or prefix-off arm: the stub reads no
        text, so it would re-fly the canonical arm's flight under another id.
        Since 2026-11-16 the prefix arms of a PREFIX-CONSUMER episode are
        flown (`--prefix on|off`: the stub's pilot through the CSP, or
        without it), unless the scenario has mid-flight events
        (prefix_consumer_refusal); its paraphrase arms stay refused.
    Until 2026-11-06 the rail flew its own mission only (sitl_refusal and
    _mission_mismatch: the pad, StubVLA, 15 m), and every scenario but
    rail-mission-stub-vla was refused; the clock refusal had been stale
    since the node took --clock-start (2026-10-12). Until 2026-11-07 every
    stressor was refused ("no SIM_* wind or GPS stressor is wired"). Until
    2026-11-14 a gate on rtl_failed_ticks was refused too (the node fed its
    FSM no RTL watch, so G6 could not fire there); the node's RTL watch
    (sitl/ros2_shield_node.py, THE RTL WATCH) now writes `rtl_failed` on
    every RTL row and sitl_extra counts them, so failsafe-rtl-blocked flies.
    The battery hook stayed refused until 2026-11-17, when the simulator
    side began to give the simulated vehicle its battery fail-safe setup
    (RAIL_BATTERY): failsafe-low-battery flies, on an opted-in campaign
    only (RAIL_BATTERY_OPT_IN)."""
    if backend not in RAIL_BACKENDS:
        raise ValueError(f"backend {backend!r} is not a flight rail ({RAIL_BACKENDS})")
    why: list[str] = []
    stressors = rail_stress_refusals(spec, defaults, rail_battery=rail_battery)
    if stressors:
        why.append("stressors the rail cannot realise (" + "; ".join(stressors) + ")")
    m = spec.mission
    if m.subject and m.subject.reclassify:
        why.append("a relabel schedule (the node declares one subject class for the "
                   "whole flight)")
    act = m.pilot.action or {}
    if any(not math.isfinite(float(v)) for v in act.values()):
        why.append("a non-finite pilot action (the Shield node refuses to fly or count "
                   "a non-finite /vla/action_4d; the action contract stays headless)")
    try:
        pol = policy if policy is not None else spec.policy.load(ROOT)
    except Exception as e:                               # noqa: BLE001
        pol = None
        why.append(f"its policy cannot be read here ({type(e).__name__}: {e})")
    if pol is not None:
        mm = _resolve_mission(spec, pol, random.Random(0))
        up = float(mm.start.up)
        if up < RAIL_MIN_START_UP_M:
            why.append(f"start altitude {up:g} m (below {RAIL_MIN_START_UP_M:g} m: "
                       f"ArduCopter's take-off and land detector cannot hold it)")
        alt_max, margin = _fence_ceiling(pol)
        if alt_max is not None and up >= alt_max - margin:
            why.append(f"start altitude {up:g} m (at or above the GeoFence's "
                       f"FENCE_ALT_MAX {alt_max:g} m less its {margin:g} m margin: the "
                       f"backstop would fire at take-off)")
        if (getattr(pol, "frame_origin", None) is None
                and math.hypot(mm.start.x, mm.start.y) > 1e-6):
            why.append(f"start {_xy(m.start_pose)} of a policy written in metres (no "
                       f"frame origin): SITL cannot be booted there")
    windowed = [e for e in spec.events if "window" in e.payload]
    if windowed and m.clock_start is None:
        why.append(f"a {windowed[0].type} with a `window` and no mission.clock_start "
                   f"(the headless harness refuses it too)")
    dfl = _as_defaults(defaults)
    t_end = ((spec.ticks or dfl.ticks) - 1) * (spec.dt or dfl.dt)
    late = [e for e in spec.events if e.at_sim_t > t_end + 1e-9]
    if late:
        why.append(f"an event at {late[0].at_sim_t:g} s, after the run's last tick "
                   f"({t_end:g} s; the headless harness refuses it too)")
    if episode is not None:
        consumer = getattr(episode, "prefix_consumer", None)
        if episode.paraphrase_arm not in (None, -1) or (episode.prefix != "on"
                                                        and not consumer):
            why.append(f"a paraphrase / prefix arm ({episode.arm_tag}): the rail's VLA "
                       f"stub reads no text, so it would fly the canonical arm's "
                       f"flight again under another id")
        if consumer:
            no = prefix_consumer_refusal(spec)
            if no:
                why.append(no)
        if episode.comparison:
            why.append(f"a comparison arm ({episode.arm_tag}): the rail flies the "
                       f"node's own Shield on ArduPilot itself")
    if not why:
        return None
    return f"the {backend} rail cannot fly this scenario as written: " + "; ".join(why)


def sitl_refusal(spec: ScenarioSpec) -> str | None:
    """rail_refusal for the desktop's SITL rail (the name every caller used
    before the hil rail existed)."""
    return rail_refusal(spec, "sitl")


def sitl_node_flags(script: Path | None = None) -> set[str]:
    """Every option string the rail's node declares, read from its source (it
    imports rclpy, which this host need not have). A flag given as a name -
    a module-level string constant, or one imported from guardrail, as
    `DEV_YAML_FLAG` is from guardrail.manifest - is resolved to its value.
    build_sitl_cmd asks it before passing a flag only some versions of the
    node take: the node parses with parse_known_args, so a flag it does not
    declare is not an error there, it goes to rclpy.init and is ignored.
    `script` names another node (the VLA stub: check_rail_flags)."""
    import ast
    import importlib
    tree = ast.parse((script or ROOT / SITL_NODE).read_text(encoding="utf-8"))
    consts: dict[str, str] = {}
    for node in tree.body:
        if (isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant)
                and isinstance(node.value.value, str)):
            consts.update({t.id: node.value.value for t in node.targets
                           if isinstance(t, ast.Name)})
        elif (isinstance(node, ast.ImportFrom) and node.module
              and node.module.split(".")[0] == "guardrail"):
            for alias in node.names:
                try:
                    val = getattr(importlib.import_module(node.module), alias.name, None)
                except Exception:                                # noqa: BLE001
                    val = None
                if isinstance(val, str):
                    consts[alias.asname or alias.name] = val
    flags = set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "add_argument"):
            for a in node.args:
                v = (a.value if isinstance(a, ast.Constant)
                     else consts.get(a.id) if isinstance(a, ast.Name) else None)
                if isinstance(v, str) and v.startswith("--"):
                    flags.add(v)
    return flags


def check_rail_flags(args: list[str]) -> None:
    """Refuse (UnsupportedScenario) an argument list a node would drop: every
    flag must be declared by the Shield node or the VLA stub, and each by the
    one it is for (RAIL_SHIELD_FLAGS, RAIL_STUB_FLAGS) - both read their
    arguments with parse_known_args, so a missing declaration is silence,
    not an error."""
    node = sitl_node_flags(_REPO / SITL_NODE)
    stub = sitl_node_flags(_REPO / SITL_STUB)
    emitted = {a.split("=", 1)[0] for a in args if a.startswith("--")}
    unknown = sorted(emitted - node - stub)
    if unknown:
        raise UnsupportedScenario(f"neither {SITL_NODE} nor {SITL_STUB} declares "
                                  f"{unknown}: the nodes would drop them")
    stub_wanted = RAIL_STUB_FLAGS + (RAIL_STUB_PREFIX_FLAGS if "--prefix" in emitted else ())
    for name, wanted, have in (("Shield node", RAIL_SHIELD_FLAGS, node),
                               ("VLA stub", stub_wanted, stub)):
        missing = sorted((emitted & set(wanted)) - have)
        if missing:
            raise UnsupportedScenario(f"the {name} does not declare {missing}, which it "
                                      f"must read: it would drop them")


def rail_extra_args(extra_args) -> list[str]:
    """`extra_args` of run_hil / run_sitl, checked: `--flag=value` tokens of
    RAIL_HOTAPPLY_FLAGS or RAIL_FORECAST_FLAGS (with one of that flag's
    values, at most once) only, each declared by the Shield node
    (it reads its arguments with parse_known_args, so an undeclared flag
    would vanish without a word). [] for None. Raises UnsupportedScenario
    otherwise."""
    out = [str(a) for a in (extra_args or ())]
    if not out:
        return []
    allowed = set(RAIL_HOTAPPLY_FLAGS) | set(RAIL_FORECAST_FLAGS)
    bad = [a for a in out if "=" not in a or a.split("=", 1)[0] not in allowed]
    if bad:
        raise UnsupportedScenario(
            f"extra rail arguments {bad}: only --flag=value forms of "
            f"{sorted(allowed)} may be added to a scenario's own")
    names = [a.split("=", 1)[0] for a in out]
    twice = sorted({n for n in names if n in RAIL_FORECAST_FLAGS and names.count(n) > 1})
    if twice:
        raise UnsupportedScenario(f"extra rail arguments name {twice} more than once")
    for a in out:
        name, value = a.split("=", 1)
        if name in RAIL_FORECAST_FLAGS and value not in RAIL_FORECAST_FLAGS[name]:
            raise UnsupportedScenario(f"{name}={value}: one of "
                                      f"{list(RAIL_FORECAST_FLAGS[name])}")
    node = sitl_node_flags(_REPO / SITL_NODE)
    missing = sorted(set(names) - node)
    if missing:
        raise UnsupportedScenario(f"{SITL_NODE} does not declare {missing}: it would "
                                  f"drop them and fly without them")
    return out


def rail_arc_asked(extra_args) -> bool:
    """Did `extra_args` ask the Shield node for the arc forecast
    (`--arc-forecast=on`)?"""
    return "--arc-forecast=on" in [str(a) for a in (extra_args or ())]


def arc_forecast_problems(metrics: dict, extra_args) -> list[str]:
    """Fail closed on the arc forecast (2026-11-17, WP3-04 / ARCH-04): the
    node's own record (metrics.json shield_forecast.arc; absent on a node
    older than the flag, read as off) must say the arc was on exactly when
    `extra_args` asked for it. A node from an older bundle parses with
    parse_known_args and would drop --arc-forecast without a word, flying
    the straight forecast under the arc arm's name; a node that arced
    unasked would fly an arm no campaign named. [] when they agree."""
    asked = rail_arc_asked(extra_args)
    flown = (metrics.get("shield_forecast") or {}).get("arc")
    if asked and flown is not True:
        return [f"--arc-forecast=on was asked but metrics.json shield_forecast.arc is "
                f"{flown!r}: the node did not fly the arc (an older node drops the flag)"]
    if not asked and flown is True:
        return ["metrics.json shield_forecast.arc is true but no --arc-forecast=on was "
                "asked: not the arm this mission is named for"]
    return []


def _num(v: float) -> str:
    """A coordinate for a command line: up to 6 decimals, no exponent, no
    trailing zeros, never "-0"."""
    s = f"{float(v):.6f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def _pts(*vals) -> str:
    return ",".join(_num(v) for v in vals)


def rail_mission(spec: ScenarioSpec, policy, defaults=None, seed: int = 0) -> dict:
    """The scenario's mission as the rail flies it: _resolve_mission's start
    (rotated, and jittered for `seed` exactly as the headless harness draws
    it, random.Random(f"{seed}:jitter"): since 2026-11-07, when the rails
    stopped refusing start_jitter_m), target, subject, pilot law, action
    (rotated), speed (the pilot's, else the harness's 4 m/s), time cap
    (ticks x dt) and clock."""
    dfl = _as_defaults(defaults)
    mm = _resolve_mission(spec, policy, random.Random(f"{seed}:jitter"))
    m = spec.mission
    return {"start": (mm.start.x, mm.start.y, mm.start.up),
            "jitter_m": start_jitter(spec, seed),
            "yaw_deg": float(mm.start.yaw_deg) % 360.0,
            "target": mm.target, "subject": mm.subject,
            "subject_class": m.subject.class_ if m.subject else None,
            "pilot": m.pilot.type, "action": mm.action, "speed": mm.speed,
            "max_s": (spec.ticks or dfl.ticks) * (spec.dt or dfl.dt),
            "clock_start": m.clock_start}


def sitl_home(spec: ScenarioSpec, policy, defaults=None, seed: int = 0) -> str | None:
    """SITL_HOME (lat,lon,alt,heading - start_sitl.sh's form) that puts the
    autopilot's home at the scenario's start: the policy frame origin's
    projection of the start (guardrail.projection, the policy's own
    equirectangular frame; jittered for `seed` as rail_mission draws it),
    RAIL_HOME_ALT_M, the start yaw. None for a metre policy (no frame
    origin), which the rails fly only from its origin, at SITL's default
    home."""
    origin = getattr(policy, "frame_origin", None)
    if origin is None:
        return None
    from guardrail.projection import projection_for
    rm = rail_mission(spec, policy, defaults, seed)
    lat, lon = projection_for(float(origin[0]), float(origin[1])).to_latlon(
        rm["start"][0], rm["start"][1])
    # Over a declared ground (X-11) the autopilot's home stands on it: the
    # DEM's elevation at the (jittered) start, the number --ground-home-msl
    # names to the nodes (rail_ground_args). Without one: RAIL_HOME_ALT_M,
    # as every SITL_HOME before.
    alt = (f"{RAIL_HOME_ALT_M:g}" if getattr(spec, "ground", None) is None
           else _num(rail_home_msl(spec, policy, defaults, seed)))
    return f"{lat:.8f},{lon:.8f},{alt},{rm['yaw_deg']:.2f}"


def rail_home_msl(spec: ScenarioSpec, policy, defaults=None, seed: int = 0) -> float:
    """The home elevation (m MSL) of a rail mission over the scenario's
    declared ground (X-11, 2026-11-18): the DEM's elevation at the mission's
    (jittered) start, rounded to the millimetre - SITL boots there
    (sitl_home) and the nodes are told it (--ground-home-msl), so the
    autopilot's home stands on the ground the Shield converts over, and
    TerrainReference's check (the DEM at home within 2 m of it) holds by
    construction. Never guessed: no ground, no number (UnsupportedScenario)."""
    ground = episode_ground(spec, policy)
    if ground is None:
        raise UnsupportedScenario(f"{spec.scenario_id}: no declared ground, so no "
                                  f"home elevation on it")
    rm = rail_mission(spec, policy, defaults, seed)
    return round(float(ground.elevation_at(rm["start"][0], rm["start"][1])), 3)


def rail_ground_args(spec: ScenarioSpec, policy, defaults=None, seed: int = 0) -> list[str]:
    """The nodes' arguments for the scenario's declared ground (X-11,
    2026-11-18), [] without one: the ground NAMED - its repo-relative path
    and the sha256 of the file as this checkout holds it (terrain.
    ground_file), which every node checks before reading it, so a host
    whose copy differs refuses the flight instead of flying another ground -
    and the home elevation on it (rail_home_msl). The ground is read here
    first (episode_ground), so a file that cannot be placed in the policy's
    frame is refused before SITL boots."""
    path = getattr(spec, "ground", None)
    if path is None:
        return []
    from guardrail import terrain as TR
    episode_ground(spec, policy)
    try:
        gfile = TR.ground_file(path, root=ROOT)
    except (ValueError, OSError) as e:
        raise UnsupportedScenario(f"ground {path}: {type(e).__name__}: {e}") from e
    return [f"--ground={path}", f"--ground-sha256={gfile['sha256']}",
            f"--ground-home-msl={_num(rail_home_msl(spec, policy, defaults, seed))}"]


def rail_policy_args(spec: ScenarioSpec, backend: str = "sitl",
                     bundles_dir: str | Path | None = None) -> list[str]:
    """The policy reference the nodes fly (build_sitl_cmd, "POLICY
    REFERENCES"): a bundle with the hash it must hold; on the desktop's
    rail a YAML policy as `--policy <yaml> --dev-yaml`, on the hil rail -
    whose runs are KPI evidence - its SIGNED bundle, and only when
    guardrail.manifest.flight_bundle_status says that bundle still flies the
    YAML's policy (else the scenario is refused: an unsigned or stale policy
    cannot be KPI-grade). The bundle path is the checkout's
    bundles/<id>-v<version>.tar.gz, relative, as both hosts hold it (the
    Orin's own build, checked there against --expected-hash)."""
    if spec.policy.bundle_path is not None:
        if SITL_EXPECTED_HASH_FLAG not in sitl_node_flags():
            raise UnsupportedScenario(
                f"policy bundle {spec.policy.bundle_path}: {SITL_NODE} does not "
                f"declare {SITL_EXPECTED_HASH_FLAG}, so the bundle cannot be pinned "
                f"to the hash it was scored against (the node would drop the flag "
                f"and fly it unpinned)")
        want = spec.policy.expected_hash
        if want is None:
            try:
                want = spec.policy.load(ROOT).policy_hash
            except Exception as e:                       # noqa: BLE001
                raise UnsupportedScenario(
                    f"policy bundle {spec.policy.bundle_path} cannot be read here "
                    f"({type(e).__name__}: {e}), so its hash cannot be pinned for "
                    f"the rail; give the scenario an expected_hash") from e
        return ["--bundle", spec.policy.bundle_path, SITL_EXPECTED_HASH_FLAG, want]
    if backend != "hil":
        return ["--policy", spec.policy.path, SITL_DEV_YAML_FLAG]
    from guardrail.manifest import flight_bundle_status
    yaml_path = ROOT / spec.policy.path
    st = flight_bundle_status(yaml_path, bundles_dir=bundles_dir)
    if st["matches"] is not True:
        raise UnsupportedScenario(
            f"the signed bundle of {spec.policy.path} is not usable for a KPI run "
            f"({st['why'] or 'unreadable'}; {st['bundle']}): build it with "
            f"python tools/build_flight_bundles.py on both hosts")
    rel = f"bundles/{Path(st['bundle']).name}"
    return ["--bundle", rel, SITL_EXPECTED_HASH_FLAG, st["yaml_hash"]]


def rail_node_args(spec: ScenarioSpec, seed: int = 0, defaults=None, *,
                   backend: str = "sitl", events: str | None = None,
                   meta: str | None = None,
                   bundles_dir: str | Path | None = None,
                   max_s_add: float = 0.0, episode=None) -> list[str]:
    """The arguments both nodes get for one scenario (the Shield node and the
    VLA stub read the same list, each its own flags). Coordinates go as
    `--flag=value`, so a negative first value never reads as an option;
    `--seed` and the policy reference keep their own tokens (the scripts
    that forward them look for `--bundle <path>`). `events` / `meta` are the
    repo-relative paths of the mission's --events / --scenario-meta files.
    `max_s_add`: a lengthened hil campaign's seconds past the rail cap
    (--hil-max-s-add; 0, the profile's length, otherwise). `episode`: for a
    prefix-consumer episode the list ends in `--prefix=on|off`, the VLA
    stub's arm; for every other episode (and None) it is the list it was."""
    pargs = rail_policy_args(spec, backend, bundles_dir)
    try:
        pol = spec.policy.load(ROOT)
    except Exception:                                    # noqa: BLE001
        # A bundle this host does not hold (its hash is pinned instead,
        # rail_policy_args): a mission in local metres needs no policy.
        pol = None
    try:
        rm = rail_mission(spec, pol, defaults, seed)
    except Exception as e:                               # noqa: BLE001
        raise UnsupportedScenario(f"the scenario's mission cannot be resolved here "
                                  f"({type(e).__name__}: {e})") from e
    args = ["--seed", str(int(seed))] + pargs
    args.append(f"--start={_pts(*rm['start'])}")
    if rm["target"] is not None:
        tx, ty, tup = rm["target"]
        args.append(f"--target={_pts(tx, ty, *([] if tup is None else [tup]))}")
        args.append(f"--reach-m={_num(RAIL_REACH_M)}")
    else:
        args.append("--no-target")
    args.append(f"--pilot={rm['pilot']}")
    if rm["pilot"] == "constant":
        a = rm["action"] or {}
        args.append("--action=" + _pts(*(a.get(k, 0.0) for k in VS.CONSTANT_CHANNELS)))
    args.append(f"--speed={_num(rm['speed'])}")
    args.append(f"--max-s={_num(rail_max_s(rm['max_s'], max_s_add))}")
    if rm["subject"] is not None:
        args.append(f"--subject={_pts(*rm['subject'])}")
        if rm["subject_class"]:
            args.append(f"--subject-class={rm['subject_class']}")
    if rm["clock_start"] is not None:
        args.append(f"--clock-start={rm['clock_start'].isoformat()}")
    if events:
        args.append(f"--events={events}")
    if meta:
        args.append(f"--scenario-meta={meta}")
    if getattr(spec, "ground", None) is not None:
        # A declared ground (X-11, 2026-11-18): named to both nodes by path
        # and sha256, with the home elevation SITL boots at (sitl_home);
        # the post-flight scoring judges over the same ground (sitl_extra,
        # rail_judge_kw). Until this date such a scenario was refused here.
        if pol is None:
            raise UnsupportedScenario(
                f"ground {spec.ground}: the scenario's policy cannot be read here, so "
                f"the ground cannot be placed in its frame")
        args.extend(rail_ground_args(spec, pol, defaults, seed))
    if getattr(spec, "world", None) is not None:
        # The declared world (WP1-07): the Shield node builds its buildings
        # and roads maps from the same repo-relative file. Read here first,
        # so a world the policy's rules cannot be judged in is refused
        # before SITL boots.
        episode_world(spec)
        args.append(f"--world={spec.world}")
    if episode is not None and getattr(episode, "prefix_consumer", None):
        if max_s_add:
            # The stub checks its CSP is static over the --max-s above, the
            # lengthened one; the arm record was checked over the rail's own
            # length. Checked here too, so the dry run and the mission refuse
            # it before SITL boots - both arms, so no unpaired off arm flies.
            try:
                prefix_csp(spec, pol, defaults, seed, max_s_add=max_s_add)
            except Exception as e:                       # noqa: BLE001
                raise UnsupportedScenario(
                    f"prefix-consumer arm {getattr(episode, 'arm_tag', episode.prefix)} "
                    f"flown {float(max_s_add):g} s longer (--hil-max-s-add): its CSP is not "
                    f"static over the lengthened flight ({type(e).__name__}: {e})") from e
        args.append(f"--prefix={episode.prefix}")
    check_rail_flags(args)
    return args


def build_sitl_cmd(spec: ScenarioSpec, tag: str, seed: int = 0,
                   defaults=None, *, events: str | None = None,
                   meta: str | None = None, extra_args=None,
                   episode=None) -> list[str]:
    """The command line for one scenario on the desktop's MAVROS 2 rail.

    Until 2026-10-06 this passed `--out` to sitl/run_sitl_demo.py, which has no
    such argument (it takes `--tag`), so the backend could only ever exit with
    an argparse error - and it targeted the pymavlink script, not the MAVROS 2
    rail behind the KPI-grade runs (audit card WP4-04). tests/test_sweep.py now
    checks every flag here against the node's own argparse.

    run_ros2_demo.sh takes the Shield arm as $1, an optional `--dynamic` as $2
    (empty here: the node's own dynamic fence is not the scenario's), and
    forwards everything from $3 on to ros2_shield_node.py AND the VLA stub
    (rail_node_args builds that part; the hil rail sends the same). A policy
    given as `bundle_path` goes to the node's `--bundle` (signature-verified
    there); `--policy` takes a YAML and would refuse a bundle archive.

    POLICY REFERENCES (2026-10-12). A launch flies a signed bundle unless it
    says otherwise: a YAML policy needs `--dev-yaml` beside `--policy`
    (sitl/ros2_shield_node.py, sitl/run_sitl_demo.py and the PAS perception
    node take it), so every `--policy <yaml>` here carries it. A bundle goes
    with the hash it must hold (SITL_EXPECTED_HASH_FLAG): the spec's
    `expected_hash` (the grant's PolicyRef pairs it with bundle_path), else
    the hash of the bundle as this harness loads it now, so the rail flies
    the policy the scenario was scored against. A bundle this host cannot
    read cannot be pinned, and is refused (UnsupportedScenario) rather than
    sent unpinned. So is EVERY bundle while the node does not declare the
    hash flag (sitl_node_flags): it parses with parse_known_args, so an
    undeclared `--expected-hash <hash>` would not stop the launch - it would
    be handed to rclpy.init and the bundle flown unpinned, silently (review
    of 2026-10-12; the node had not taken the flag that day).

    THE SCENARIO'S OWN MISSION (2026-11-06): its start (--start, which the
    run's SITL_HOME must match: run_sitl passes it through WSLENV), target
    and altitude, pilot law, speed (the pilot's, else 4 m/s), time cap,
    subject (turned with the mission), clock and events (rail_node_args).
    `extra_args` (2026-11-10): the hot-apply listener's flags, for the gRPC
    harness's rail episode (rail_extra_args); a campaign passes none."""
    return (["wsl", "bash", SITL_SCRIPT, "on" if spec.shield else "off", "",
             "--tag", tag]
            + rail_node_args(spec, seed, defaults, backend="sitl", events=events,
                             meta=meta, episode=episode)
            + rail_extra_args(extra_args))


def read_fresh_log(log: Path, launched_at: float) -> list[dict]:
    """The rail's flight log, but only if THIS launch wrote it.

    The tag is deterministic, so demo/out/<tag>/flight_log.jsonl from an
    earlier run is still there when a launch exits 0 without writing one;
    reading it would score the previous flight as this one. A log older than
    the launch is refused rather than deleted, so nothing is lost.
    """
    if not log.is_file():
        raise RuntimeError(f"the rail wrote no {log.name}")
    if log.stat().st_mtime < launched_at - 1.0:
        raise RuntimeError(f"{log} predates this launch: it is an earlier run's "
                           f"log, not this one's")
    return [json.loads(x) for x in log.read_text(encoding="utf-8").splitlines()
            if x.strip()]


def read_fresh_metrics(path: Path, launched_at: float) -> dict:
    """The rail's metrics.json, under read_fresh_log's rule: the tag is
    deterministic, so an earlier flight's file (and its FSM record) would
    otherwise be scored as this one's."""
    if not path.is_file():
        raise RuntimeError(f"the rail wrote no {path.name}")
    if path.stat().st_mtime < launched_at - 1.0:
        raise RuntimeError(f"{path} predates this launch: it is an earlier run's "
                           f"metrics, not this one's")
    return json.loads(path.read_text(encoding="utf-8"))


def run_generations(run_dir: Path) -> dict:
    """{generation: Policy} from a rail run's own policy_g<N>.json files (the
    take-off policy and every hot-applied generation), read without the
    flight loaders' runtime gate (a hot-applied zone is a dynamic_nfz, which
    a policy FILE may not fly). {} when there are none."""
    from guardrail.models import policy_from_raw
    out = {}
    for f in sorted(Path(run_dir).glob("policy_g*.json")):
        tail = f.stem[len("policy_g"):]
        if tail.isdigit():
            out[int(tail)] = policy_from_raw(json.loads(f.read_text(encoding="utf-8")),
                                             source=f.name, runtime=False)
    return out


def _owner(r: dict) -> str:
    return ("autopilot" if r.get("flown") is False or r.get("setpoint") == "none"
            else "shield")


def rail_judge_kw(ground) -> dict:
    """The keywords every post-flight judge Shield of a rail run is built
    with over the scenario's declared ground (X-11, 2026-11-18): the ground,
    and up_ref "terrain" - the node's rows under --ground hold in `up` the
    height above the DEM under the aircraft, which is what its Shield
    judged. {} without a ground: every judge as it was built before."""
    return {} if ground is None else {"ground": ground, "up_ref": "terrain"}


def _judged_steps(rows: list[dict], policy, generations: dict | None, ground=None):
    """Per row i > 0, the previous instant's judgement of row i's position,
    for the breach and polygon-entry tests: (unsafe under the rules of row
    i-1, inside a zone as it stood at row i-1), or None where it cannot be
    known. Read from the node's own judgement when the row carries it
    (`unsafe_prev_rules` / `zones_in_prev`, 2026-11-06: the previous tick's
    generation, zone positions and rule clock, sitl/ros2_shield_node.py
    _prev_judgement); else rebuilt from the run's generations (`generations`,
    its policy_g<N>.json) at the previous row's `clock`, which a moving zone
    defeats (its recorded vertices are where it was, not where it is); else,
    for a log with one generation and no rule that switches with the clock,
    the row's own judgement (nothing changed under the aircraft). Returns
    (list of (unsafe_prev, inside_prev) or None per row, why-not or None)."""
    from datetime import datetime as _dt
    if all("unsafe_prev_rules" in r and "zones_in_prev" in r for r in rows[1:]):
        out = [None] + [(bool(r["unsafe_prev_rules"]) if r["unsafe_prev_rules"] is not None
                         else None,
                         bool(r["zones_in_prev"]) if r["zones_in_prev"] is not None else None)
                        for r in rows[1:]]
        return out, None
    gens = {r.get("generation") for r in rows}
    if len(gens) <= 1 and not _timed(policy):
        return "own", None
    if not generations or any(r.get("generation") not in generations for r in rows):
        return None, ("the rules changed during the flight and the run's generations "
                      "are not available to judge each step")
    pols = list(generations.values())
    if any(SE.has_moving_zone(p) for p in pols):
        return None, "a moving zone: its recorded vertices are not where it was"
    if any(_map_rules(p) for p in pols):
        return None, ("a distance_envelope over buildings or roads: its map is the "
                      "flying Shield's (--world), not in the log")
    clocked = all(r.get("clock") for r in rows)
    if any(_timed(p) for p in pols) and not clocked:
        return None, "rules that switch with the clock, and no clock on the rows"
    pin = {"now": None}
    shields = {g: Shield(p, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S, escalation=False,
                         now=(lambda: pin["now"]) if clocked else None,
                         **rail_judge_kw(ground))
               for g, p in generations.items()}
    out: list = [None]
    for i in range(1, len(rows)):
        prev, r = rows[i - 1], rows[i]
        sh = shields[prev["generation"]]
        if clocked:
            pin["now"] = _dt.fromisoformat(prev["clock"])
        st = State(x=float(r["x"]), y=float(r["y"]), up=float(r["up"]))
        unsafe = bool(sh.state_is_unsafe(st))
        inside = any(ring_contains(zr.polygon, st.x, st.y) for zr in sh.zones_now(st.up))
        out.append((unsafe, inside))
    return out, None


# What a rule is to the autopilot's fail-safe modes (2026-11-07, the user's
# ruling; docs/DESIGN-harness-vehicle-response.md, "The rail's gates"): a
# KEEP-OUT - a zone's polygon or ring, a stand-off, a clearance - whose entry
# is a safety event whoever flies, or a MISSION BOUND that ArduPilot's RTL,
# LAND and BRAKE / LOITER hold do not follow - a keep-in corridor, an
# altitude band - whose departure while the autopilot flies is reported,
# not gated: the fail-safe modes are the grant's escape. type -> kind; every
# other type a rule can be unsafe for is a keep-out.
RAIL_DEPARTURE_KINDS = {"corridor": "corridor", "corridor_swap": "corridor",
                        "altitude_envelope": "altitude_band"}
# Rows that name no rule (no `unsafe_rules`: an older rail's, or the harness's
# own rows in the rail's shape) cannot tell a keep-out from a mission bound.
# Every unsafe step the autopilot made is then gated as a keep-out breach (a
# rule the log does not name is a keep-out, as in _rule_kinds), read up to
# the mission's end like `breaches` (the landing's descent below the floor is
# not told from a zone either), and its departures are not told apart.
NO_RULE_NAMES = ("the rows name no rule (`unsafe_rules`): every unsafe step the autopilot "
                 "made is gated as a keep-out breach, up to the mission's end; departures "
                 "from a corridor or an altitude band are not told apart")


def _rule_kinds(policy, generations: dict | None) -> dict[str, str]:
    """rule id -> "keepout" or its RAIL_DEPARTURE_KINDS kind, over the
    take-off policy and every generation (a hot-applied zone included)."""
    out: dict[str, str] = {}
    for p in [policy, *(generations or {}).values()]:
        for c in getattr(p, "constraints", None) or []:
            out[c.id] = RAIL_DEPARTURE_KINDS.get(c.type, "keepout")
    return out


def _autopilot_scope(rows: list[dict], policy, generations: dict | None, judged) -> dict:
    """The autopilot's unsafe steps by what made them unsafe (the user's
    ruling, 2026-11-07):

      autopilot_keepout_breaches  rows on which a KEEP-OUT rule the previous
          row was not unsafe for makes the position unsafe, the previous row
          flown by the autopilot and no rule change on the row - at any time,
          also while the position is already unsafe for a mission bound (an
          RTL out of a corridor and then into a zone counts the zone). The
          aircraft's own move: the node's previous-tick judgement names the
          zone (`unsafe_prev_rules`), else the row's own reading where
          nothing changed (judged "own"), else the rebuilt judgement.
      autopilot_departures  the autopilot's steps into an unsafe position
          (autopilot_breaches) made by mission bounds alone, per kind
          ({"corridor": n, "altitude_band": n}): reported, not gated.

    {} on rows without `unsafe_rules`: rail_measurements then gates every
    autopilot breach as a keep-out's (NO_RULE_NAMES)."""
    if not all("unsafe_rules" in r for r in rows):
        return {}
    kinds = _rule_kinds(policy, generations)
    ko = 0
    dep = {"corridor": 0, "altitude_band": 0}
    prev_unsafe, prev_ko = False, set()
    for i, r in enumerate(rows):
        rules = set(r.get("unsafe_rules") or [])
        ko_now = {rid for rid in rules if kinds.get(rid, "keepout") == "keepout"}
        if i > 0 and not r.get("rule_events") and _owner(rows[i - 1]) == "autopilot":
            new_ko = ko_now - prev_ko
            if new_ko:
                prev_rules = r.get("unsafe_prev_rules")
                if prev_rules is not None:
                    moved = bool(new_ko & set(prev_rules))
                else:
                    moved = judged == "own" or (isinstance(judged, list) and bool(judged[i][0]))
                ko += bool(moved)
            if r.get("unsafe") and not prev_unsafe and not ko_now:
                moved_in = judged == "own" or (isinstance(judged, list) and bool(judged[i][0]))
                if moved_in:
                    for kind in sorted({kinds.get(rid) for rid in rules} - {"keepout", None}):
                        dep[kind] = dep.get(kind, 0) + 1
        prev_unsafe, prev_ko = bool(r.get("unsafe")), ko_now
    return {"autopilot_keepout_breaches": ko, "autopilot_departures": dep}


def rail_measurements(rows: list[dict], policy, generations: dict | None = None, *,
                      ground=None) -> dict:
    """The harness's own position measurements, from a rail's flight log.

    run_episode measures breaches, polygon entries, ticks inside a polygon
    and the distance to the fence as it flies; the rail's log carries what
    they need, but until 2026-10-19 nothing read it, so on `--backend sitl`
    every such gate was "not measured" and rail-mission-stub-vla, the one
    scenario the rail could fly, FAILED however the flight went (review,
    harness-tools/sitl-backend-cannot-pass).

    Same definitions as run_episode:
      breaches        a step into an unsafe position the aircraft MADE: row i
                      unsafe, row i-1 not, and row i's position already
                      unsafe under row i-1's rules - its generation, its
                      zones where they stood and its rule clock - with no
                      rule change on row i (`rule_events`: a scenario event,
                      the node's --dynamic zone, a REST hot-apply). A zone
                      spawned onto the aircraft, a window opening over it, a
                      zone moving onto it is imposed, not a breach;
      polygon fields  the row's position against the zones in force at its
                      altitude, entries on an outside -> inside step judged
                      the same way;
      autopilot_*     a step that ended a row the rail did not fly (`flown`
                      false or setpoint "none") is the autopilot's move.
    HOW EACH STEP IS JUDGED (2026-11-06, H3): from the node's own record of
    the previous tick's judgement when the row carries it, else rebuilt from
    the run's `generations` at the previous row's clock (_judged_steps).
    Until then a step was judged against the row's own rules, so a policy
    whose rules switch with the clock (_timed) had nothing measured, and a
    zone spawned onto the aircraft counted as the flight's breach.
    A field the log cannot answer is left out, so its gate reads "not
    measured": `unsafe` absent (an older rail), or steps that cannot be
    judged (_judged_steps says why, in `rail_measurements_not_measured`).
    Polygon fields come from the node's per-row zones (`zones_in`,
    `zone_d_m`) when it logged them, else from the policy's zones (one
    generation, nothing timed). `ground` (X-11): the scenario's declared
    ground, every judge built over it (rail_judge_kw); None, as before."""
    out: dict[str, Any] = {}
    if not rows:
        return out
    judged, why_not = (_judged_steps(rows, policy, generations) if ground is None
                       else _judged_steps(rows, policy, generations, ground))
    if why_not:
        out["rail_measurements_not_measured"] = why_not
    if all("unsafe" in r for r in rows) and judged is not None:
        b = ab = 0
        prev_unsafe = False
        for i, r in enumerate(rows):
            unsafe = bool(r.get("unsafe"))
            if i > 0 and unsafe and not prev_unsafe and not r.get("rule_events"):
                moved_in = True if judged == "own" else judged[i][0]
                if moved_in:
                    if _owner(rows[i - 1]) == "autopilot":
                        ab += 1
                    else:
                        b += 1
            prev_unsafe = unsafe
        out.update(breaches=b, autopilot_breaches=ab)
        out.update(_autopilot_scope(rows, policy, generations, judged)
                   or {"autopilot_keepout_breaches": ab, "autopilot_scope_basis": NO_RULE_NAMES})
    logged = all("zones_in" in r and "zone_d_m" in r for r in rows)
    if logged or judged == "own":
        fence_seen = False
        min_d = math.inf
        inside_ticks = entries = ap_entries = 0
        prev_inside = False
        # Zones only: a distance_envelope's map is not needed to read them.
        zones_of = None if logged else Shield(policy, require_maps=False,
                                              **rail_judge_kw(ground)).zones_now
        for i, r in enumerate(rows):
            if logged:
                d = r.get("zone_d_m")
                inside = bool(r.get("zones_in"))
                if d is not None:
                    fence_seen = True
                    min_d = min(min_d, float(d))
            else:
                inside = False
                for zr in zones_of(float(r["up"])):
                    fence_seen = True
                    sd = _signed_dist(zr.polygon, float(r["x"]), float(r["y"]))
                    min_d = min(min_d, sd)
                    inside = inside or sd < 0
            if inside:
                inside_ticks += 1
                if i > 0 and not prev_inside and not r.get("rule_events"):
                    moved_in = True if judged in ("own", None) else judged[i][1]
                    if judged is None and logged:
                        moved_in = None
                    if moved_in:
                        if _owner(rows[i - 1]) == "autopilot":
                            ap_entries += 1
                        else:
                            entries += 1
            prev_inside = inside
        if fence_seen:
            out.update(ticks_inside_fence_polygon=inside_ticks,
                       min_true_dist_to_fence_m=round(min_d, 3))
            if judged is not None:
                out.update(polygon_entries=entries, autopilot_polygon_entries=ap_entries)
        else:
            # No zone at all: None, as run_episode reports it.
            out.update(polygon_entries=None, autopilot_polygon_entries=None,
                       ticks_inside_fence_polygon=None, min_true_dist_to_fence_m=None)
    elif judged is not None and judged != "own":
        # Rebuilt steps (an older log): the polygon fields from the
        # generations too.
        gens = generations or {}
        pin = {"now": None}
        from datetime import datetime as _dt
        clocked = all(r.get("clock") for r in rows)
        # Zones only (zones_now), as above: built without a world's maps.
        shields = {g: Shield(p, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S,
                             escalation=False, require_maps=False,
                             now=(lambda: pin["now"]) if clocked else None,
                             **rail_judge_kw(ground))
                   for g, p in gens.items()}
        fence_seen = False
        min_d = math.inf
        inside_ticks = entries = ap_entries = 0
        prev_inside = False
        for i, r in enumerate(rows):
            if clocked:
                pin["now"] = _dt.fromisoformat(r["clock"])
            inside = False
            for zr in shields[r["generation"]].zones_now(float(r["up"])):
                fence_seen = True
                sd = _signed_dist(zr.polygon, float(r["x"]), float(r["y"]))
                min_d = min(min_d, sd)
                inside = inside or sd < 0
            if inside:
                inside_ticks += 1
                if i > 0 and not prev_inside and not r.get("rule_events") \
                        and judged[i][1]:
                    if _owner(rows[i - 1]) == "autopilot":
                        ap_entries += 1
                    else:
                        entries += 1
            prev_inside = inside
        if fence_seen:
            out.update(polygon_entries=entries, autopilot_polygon_entries=ap_entries,
                       ticks_inside_fence_polygon=inside_ticks,
                       min_true_dist_to_fence_m=round(min_d, 3))
        else:
            out.update(polygon_entries=None, autopilot_polygon_entries=None,
                       ticks_inside_fence_polygon=None, min_true_dist_to_fence_m=None)
    return out


# --------------------------------------------------------------------------- #
# the rail's rule readings and its scenario window (2026-11-07)
# --------------------------------------------------------------------------- #

# The row field the Shield node writes its decision's trace words in
# (sitl/ros2_shield_node.py shield_trace_fields, 2026-11-07): window_opens_on,
# window_opened_on, cap_refused, floor_refused. Absent on an older node's rows.
RAIL_TRACE = "shield_trace"
# Why a rail measure is missing when the rows hold no decision trace.
_NO_TRACE = ("the rows carry no decision trace (`shield_trace`, logged by the "
             "Shield node since 2026-11-07): the Shield's cap / floor refusals "
             "are in its trace only, not in the audit or the row")


def _accepted_rule_events(r: dict) -> bool:
    """Did a rule change that the position met take effect on this row: a
    scenario event the Shield accepted (the node labels a refused one
    "(refused)"), a REST hot-apply between ticks, the --dynamic zone?
    run_episode's `imposed` for an event, as the node records it
    (`rule_events`)."""
    return any(not str(e).endswith("(refused)") for e in (r.get("rule_events") or []))


def rail_window(rows: list[dict], max_s: float | None) -> tuple[list[dict], list[dict]]:
    """(the rows of the scenario's own time cap, the rows flown after it).

    Since c309f0c a rail mission flies the scenario's cap plus the node's
    Loiter operator timeout plus 5 s (rail_max_s), so that a Loiter the
    lagging autopilot provokes shows its outcome - the fail-safe the rubric
    reads (docs/RUBRIC-failsafe.md, section 3), which is the WHOLE flight's
    (the FSM record in metrics.json), as are the safety invariants
    (sitl_extra). The performance measures are defined over the scenario's
    ticks (run_episode flies ticks x dt, the goal is read at their end) and
    are read over the rows whose mission time is before `max_s`. None keeps
    every row."""
    if max_s is None:
        return list(rows), []
    k = next((i for i, r in enumerate(rows) if float(r.get("t") or 0.0) >= float(max_s) - 1e-9),
             len(rows))
    return list(rows[:k]), list(rows[k:])


def _rail_judges(rows: list[dict], policy, generations: dict | None,
                 lookahead_s: float, lookahead_dt_s: float, ground=None):
    """The run's Shields, one per generation flown, on a rule clock pinned to
    the row being read: ({generation: Shield}, pin, None), or (None, None,
    why) when the rows cannot be read that way."""
    gens = dict(generations or {})
    row_gens = {r.get("generation", 0) for r in rows}
    if not gens:
        if len(row_gens) > 1:
            return None, None, ("the rules changed during the flight and the run's "
                                "generations (policy_g<N>.json) are not available")
        gens = {next(iter(row_gens)): policy}
    missing = sorted(map(str, (g for g in row_gens if g not in gens)))
    if missing:
        return None, None, (f"generation(s) {missing} were flown but are not among the "
                            f"run's policy_g<N>.json files")
    clocked = all(r.get("clock") for r in rows)
    if any(_timed(p) for p in gens.values()) and not clocked:
        return None, None, "rules that switch with the clock, and no rule clock on the rows"
    pin = {"now": None}
    # Built without a world's maps (require_maps=False): these judges read
    # rule status, corridor offsets and openings, which no map changes; a
    # distance_envelope's breaches are the flying Shield's own (`unsafe`,
    # `violations` on the rows).
    judges = {g: Shield(p, lookahead_s=lookahead_s, dt=lookahead_dt_s, escalation=False,
                        require_maps=False,
                        now=(lambda: pin["now"]) if clocked else None,
                        **rail_judge_kw(ground))
              for g, p in gens.items()}
    return judges, pin, None


def _shield_state(r: dict) -> State:
    """The state the rail's Shield was handed on row `r`: the logged pose,
    with the measured velocity when it flew on one (`shield_velocity`,
    `state_v`)."""
    kw = {}
    v = r.get("state_v") if r.get("shield_velocity") else None
    if isinstance(v, (list, tuple)) and len(v) == 3 and all(
            c is not None and math.isfinite(float(c)) for c in v):
        kw["velocity"] = tuple(float(c) for c in v)
    return State(x=float(r["x"]), y=float(r["y"]), up=float(r["up"]),
                 yaw_deg=float(r.get("yaw_deg") or 0.0), **kw)


def rail_rule_measurements(rows: list[dict], policy, generations: dict | None = None, *,
                           dt: float = 0.1, lookahead_s: float = LOOKAHEAD_S,
                           lookahead_dt_s: float = LOOKAHEAD_DT_S, ground=None) -> dict:
    """run_episode's rule measurements, read from a rail's flight log with
    the same definitions (one scorer; until 2026-11-07 each gate on them
    read "not measured" on the rails although the log holds what they need):

      phase<k>_violation_ticks_<rule>  the rows whose `violations` (the
              Shield's, of the pilot's action) name the rule, per stretch
              between the rows on which a rule changed: an accepted
              `rule_events` entry (a scenario event, a hot-apply, the
              --dynamic zone) or the Shield's in-force reading flipping;
      switch_lag_s_<rule>, switch_conservative_<rule>  the rule's last flip
              of `in_force` against the schedule's nearest edge (within 2 s),
              both at row resolution, judged by switch_conservative with eps
              half the forecast step;
      anticipated_ticks_<rule> (and _opening_only_)  rows on which a timed
              rule not in force was violated or named in the decision's
              `window_opens_on`; release_gap_s_<rule>;
      max_offset_from_corridor, final_offset_from_corridor  the largest and
              the last lateral offset from the corridor(s) in force;
      window_opened_on_ticks, cap_refused_ticks, floor_refused_ticks  from
              the decision's trace on the rows (`shield_trace`).

    The Shield's readings are rebuilt per row from the run's own
    generations (policy_g<N>.json) at the row's rule clock (`clock`) and
    position: rule_status for in force and the corridor offset, as
    run_episode reads them, and, on rows that carry no trace,
    Shield.openings_imposed for `window_opens_on` (what the flying Shield's
    trace holds: the same rules, clock and state - the logged pose, with
    the measured velocity when the Shield flew on one). A field the log
    cannot answer is left out, its gate reads "not measured", and
    `rail_not_measured` says why."""
    out: dict[str, Any] = {}
    nm: dict[str, str] = {}
    if not rows:
        return out
    judges, pin, why = (_rail_judges(rows, policy, generations, lookahead_s, lookahead_dt_s)
                        if ground is None else
                        _rail_judges(rows, policy, generations, lookahead_s, lookahead_dt_s,
                                     ground))
    traced = all(isinstance(r.get(RAIL_TRACE), dict) for r in rows)
    if judges is None:
        nm["phase*_violation_ticks_*, switch_*, anticipated_ticks_*, "
           "max_offset_from_corridor"] = why
        if not traced:
            nm["cap_refused_ticks, floor_refused_ticks"] = _NO_TRACE
        out["rail_not_measured"] = nm
        return out
    req_state: dict = {}
    req_edges: dict[str, list[float]] = {}
    switch_flips: dict[str, list[tuple[float, bool]]] = {}
    anticipated: dict[str, int] = {}
    opening_only: dict[str, int] = {}
    last_violation_t: dict[str, float] = {}
    release_gap: dict[str, float] = {}
    timed_rules: set[str] = set()
    prev_active: set | None = None
    prev_ids: set = set()
    phase = 0
    phase_vios: list[dict[str, int]] = [{}]
    max_offset = final_offset = None
    opened = cap_ticks = floor_ticks = 0
    for i, r in enumerate(rows):
        t = float(r.get("t") or 0.0)
        sh = judges[r.get("generation", 0)]
        if pin is not None and r.get("clock"):
            pin["now"] = datetime.fromisoformat(r["clock"])
        st = _shield_state(r)
        imposed = i == 0 or _accepted_rule_events(r)
        violated = {v.get("rule_id") for v in (r.get("violations") or [])}
        trace = r.get(RAIL_TRACE) if traced else None
        rs = None
        if _timed(sh.policy):
            gov = _governing_recurrences(sh.policy)
            rs = sh.rule_status(st)
            active = {x["id"] for x in rs if x["in_force"]}
            ids_now = {x["id"] for x in rs}
            timed_rules |= {x["id"] for x in rs
                            if x["id"] in gov or x["type"] in ("time_window_switch",
                                                               "corridor_swap")}
            if r.get("clock"):
                gen = r.get("generation", 0)
                for rid, recs in gov.items():
                    for rec in recs:
                        now_on = recurrence_active(rec, pin["now"])
                        key = (gen, id(rec))
                        if key in req_state and req_state[key] != now_on:
                            req_edges.setdefault(rid, []).append(t)
                        req_state[key] = now_on
            if prev_active is not None:
                on_ = sorted((active - prev_active) & prev_ids)
                off_ = sorted((prev_active - active) & ids_now)
                for rid in on_:
                    switch_flips.setdefault(rid, []).append((t, True))
                for rid in off_:
                    switch_flips.setdefault(rid, []).append((t, False))
                    if rid in last_violation_t:
                        release_gap[rid] = round(t - last_violation_t[rid], 4)
                if on_ or off_:
                    imposed = True
            opens = (set(trace.get("window_opens_on") or ()) if trace is not None
                     else set(sh.openings_imposed(st)))
            for rid in violated | opens:
                if rid in timed_rules and rid not in active:
                    anticipated[rid] = anticipated.get(rid, 0) + 1
                    if rid not in violated:
                        opening_only[rid] = opening_only.get(rid, 0) + 1
            for rid in timed_rules:
                anticipated.setdefault(rid, 0)
                opening_only.setdefault(rid, 0)
            prev_active, prev_ids = active, ids_now
        if trace is not None:
            opened += bool(trace.get("window_opens_on") or trace.get("window_opened_on"))
            cap_ticks += "cap_refused" in trace
            floor_ticks += "floor_refused" in trace
        for rid in violated:
            last_violation_t[rid] = t
        if imposed and i > 0:
            phase += 1
            phase_vios.append({})
        for v in r.get("violations") or []:
            rid = v.get("rule_id")
            phase_vios[phase][rid] = phase_vios[phase].get(rid, 0) + 1
        if any(c.type in ("corridor", "corridor_swap") for c in sh.policy.constraints):
            offs = [x["value"] for x in (rs if rs is not None else sh.rule_status(st))
                    if x["type"] in ("corridor", "corridor_swap") and x["in_force"]
                    and x["value"] is not None]
            if offs:
                final_offset = max(offs)
                max_offset = final_offset if max_offset is None else max(max_offset,
                                                                         final_offset)
    if pin is not None:
        pin["now"] = None
    if len(phase_vios) > 1:
        # Every rule of the last generation in every phase, as run_episode
        # keys them: a 0 is a measured 0, a phase that never began has none.
        rule_ids = [c.id for c in judges[rows[-1].get("generation", 0)].policy.constraints]
        for k, counts in enumerate(phase_vios):
            for rid in rule_ids:
                out[f"phase{k}_violation_ticks_{rid}"] = counts.get(rid, 0)
    eps = lookahead_dt_s / 2.0
    for rid, flips in switch_flips.items():
        ts, turned_on = flips[-1]
        near = [e for e in (req_edges.get(rid) or []) if abs(e - ts) <= 2.0]
        if not near:
            continue            # an event-driven switch: no schedule edge to compare
        lag = round(ts - min(near, key=lambda e: abs(e - ts)), 4)
        out[f"switch_lag_s_{rid}"] = lag
        out[f"switch_conservative_{rid}"] = switch_conservative(turned_on, lag, eps, dt)
    for rid, n in anticipated.items():
        out[f"anticipated_ticks_{rid}"] = n
    for rid, n in opening_only.items():
        out[f"anticipated_opening_only_ticks_{rid}"] = n
    for rid, g in release_gap.items():
        out[f"release_gap_s_{rid}"] = g
    out["max_offset_from_corridor"] = None if max_offset is None else round(max_offset, 3)
    out["final_offset_from_corridor"] = (None if final_offset is None
                                         else round(final_offset, 3))
    if traced:
        out.update(window_opened_on_ticks=opened, cap_refused_ticks=cap_ticks,
                   floor_refused_ticks=floor_ticks)
    else:
        nm["cap_refused_ticks, floor_refused_ticks, window_opened_on_ticks"] = _NO_TRACE
    out["rail_rule_basis"] = {
        "rules": "the run's generations at each row's rule clock (Shield.rule_status)",
        "window_opens_on": ("the rows' decision trace" if traced else
                            "rebuilt: Shield.openings_imposed at the row's clock and state"),
        "dt_s": dt, "eps_s": eps}
    if nm:
        out["rail_not_measured"] = nm
    return out


def rail_flight_fields(rows: list[dict], metrics: dict) -> dict:
    """What else a rail log answers of the harness's per-episode fields:
    `ended_safe` (the last row's position), `max_speed_flown` and
    `all_actions_finite` (the setpoints the rail SENT: `emitted_body`, a
    speed being frame-invariant, on rows it flew), and the ranges to a
    declared subject (`min_range_to_subject`, `final_range_to_subject`).
    The autopilot's own flight (LOITER / RTL / LAND) sends nothing and is
    not in them, where the headless harness's modelled autopilot is."""
    out: dict[str, Any] = {}
    if not rows:
        return out
    if "unsafe" in rows[-1]:
        out["ended_safe"] = not rows[-1]["unsafe"]
    sent = [r.get("emitted_body") for r in rows
            if r.get("flown") and isinstance(r.get("emitted_body"), dict)]
    if sent:
        vals = [float(a.get(k, 0.0)) for a in sent for k in ("vx", "vy", "vz_up", "yaw_rate")]
        out["all_actions_finite"] = all(math.isfinite(v) for v in vals)
        out["max_speed_flown"] = round(max(math.hypot(float(a.get("vx", 0.0)),
                                                      float(a.get("vy", 0.0)))
                                           for a in sent), 4)
    subj = metrics.get("subject") if isinstance(metrics.get("subject"), dict) else None
    if subj and subj.get("x") is not None:
        d = [math.hypot(float(r["x"]) - subj["x"], float(r["y"]) - subj["y"]) for r in rows]
        out["min_range_to_subject"] = round(min(d), 3)
        out["final_range_to_subject"] = round(d[-1], 3)
        out["subject_class_final"] = subj.get("class")
    return out


def scenario_max_s(metrics: dict) -> float | None:
    """The scenario's own time cap a rail run recorded (metrics.json
    `scenario`, the harness's rail_meta `max_s`), None without one."""
    sc = metrics.get("scenario") if isinstance(metrics.get("scenario"), dict) else {}
    v = sc.get("max_s")
    return float(v) if isinstance(v, (int, float)) and math.isfinite(float(v)) else None


def rail_rule_priorities(policy, generations: dict | None = None) -> dict:
    """The take-off policy's rule priorities and every generation's (a
    hot-applied zone's included), as run_hil records them."""
    prios = dict(K.rule_priorities(policy))
    for g in sorted(generations or {}):
        prios.update(K.rule_priorities(generations[g]))
    return prios


# The FSM edges that end a mission (guardrail/fsm.py: G9 home reached, G10
# landed, X5): run_episode stops on that tick. Since 2026-11-07 the ROS 2
# rail logs on to the autopilot's landing (sitl/ros2_shield_node.py), and its
# rows after the edge are the autopilot's alone.
TERMINAL_EDGES = ("G9", "G10", "X5")
# The SAFETY INVARIANTS (the user's ruling, 2026-11-07): what must hold
# whenever the aircraft flies, so a rail run gates them over its WHOLE flight
# - an RTL that flies into a zone after the scenario's cap is a real event
# (hil rehearsal rh01_021). P0 escapes are the Shield's counter (guardrail.kpi
# leaves the autopilot's ticks out, by the KPI's definition); what the
# AUTOPILOT flew is counted apart, as run_episode counts it, and gated on the
# KEEP-OUT rules alone with the Shield's bound (RAIL_INVARIANT_PAIRS,
# rail_gate_set): its entries into a keep-out polygon or ring. Its departures
# from a keep-in corridor or an altitude band while a fail-safe mode flies
# are reported (`autopilot_departures`, and the all-kinds
# `autopilot_breaches`), not gated: ArduPilot's RTL, LAND and hold do not
# follow mission bounds, and the fail-safe is the grant's escape (the user's
# ruling, 2026-11-07; docs/DESIGN-harness-vehicle-response.md, "The rail's
# gates"). `ended_safe` is the state when the mission ended (its terminal
# edge, else its last row). The speed and finiteness of what the rail sent
# are invariants too. Every other gate - how long, how many ticks, the
# phases, the offsets, the goal - is a performance gate, read over the
# scenario's own window (rail_window).
RAIL_INVARIANT_FIELDS = ("breaches", "autopilot_breaches", "autopilot_keepout_breaches",
                         "autopilot_departures", "polygon_entries",
                         "autopilot_polygon_entries", "ended_safe", "max_speed_flown",
                         "all_actions_finite")
RAIL_INVARIANT_PAIRS = {"breaches": "autopilot_keepout_breaches",
                        "polygon_entries": "autopilot_polygon_entries"}


def rail_mission_rows(rows: list[dict]) -> tuple[list[dict], list[dict]]:
    """(the mission's rows, up to and including the FSM's terminal edge -
    TERMINAL_EDGES, where run_episode stops - and the rows the rail logged
    after it, the autopilot's landing). No terminal edge: every row."""
    k = next((i for i, r in enumerate(rows) if r.get("fsm_edge") in TERMINAL_EDGES), None)
    return (list(rows), []) if k is None else (list(rows[:k + 1]), list(rows[k + 1:]))


def rail_scored_rows(rows: list[dict], max_s: float | None) -> list[dict]:
    """The rows score() reads for a rail run's performance measures: the
    mission's (rail_mission_rows) before the scenario's cap (rail_window)."""
    return rail_window(rail_mission_rows(rows)[0], max_s)[0]


def rail_ground_check(spec: ScenarioSpec, metrics: dict) -> dict:
    """Did the rail fly the scenario's declared ground (X-11, 2026-11-18)?
    {"declared": {path, sha256} as this checkout holds the file, "flown":
    the run's metrics.json ground.file (None: the node recorded none),
    "match": both name the same path and sha256}. The node refuses a file
    that does not hash to the --ground-sha256 it was sent, so a mismatch
    here means the run was not flown from rail_node_args."""
    from guardrail import terrain as TR
    declared = TR.ground_file(spec.ground, root=ROOT)
    g = metrics.get("ground") if isinstance(metrics.get("ground"), dict) else {}
    flown = g.get("file") if isinstance(g.get("file"), dict) else None
    match = bool(flown) and flown.get("path") == declared["path"] \
        and flown.get("sha256") == declared["sha256"]
    return {"declared": declared, "flown": flown, "match": match}


def ground_flight_problems(spec: ScenarioSpec, metrics: dict) -> list[str]:
    """Why a rail run of a scenario with a declared ground cannot be scored
    as that scenario ([] when it can, or the scenario declares none): its
    metrics.json names no ground file, or another path or sha256
    (rail_ground_check). FAIL CLOSED (X-11, 2026-11-18): every judge of
    sitl_extra is built over the DECLARED ground, so a run the Shield flew
    over another ground, or none, would be scored as if it had flown this
    one. run_sitl, run_hil and rescore_rail_run refuse such a run (an
    error, never a scored episode), as they refuse a run of the wrong arm."""
    if getattr(spec, "ground", None) is None:
        return []
    chk = rail_ground_check(spec, metrics)
    if chk["match"]:
        return []
    d = chk["declared"]
    if chk["flown"] is None:
        return [f"the run's metrics.json names no ground file, and the scenario declares "
                f"{d['path']} (sha256 {d['sha256']}): not flown over it"]
    f = chk["flown"]
    return [f"the run flew over {f.get('path')} (sha256 {f.get('sha256')}), the scenario "
            f"declares {d['path']} (sha256 {d['sha256']})"]


def sitl_extra(spec: ScenarioSpec, rows: list[dict], metrics: dict, policy,
               generations: dict | None = None, *, max_s: float | None = None,
               dt: float | None = None, autopilot_texts: list[dict] | None = None) -> dict:
    """What score() needs from a rail flight beside its rows: the goal (the
    node's `reached`; a mission with no target has none), the escalation
    FSM's record (its fsm.summary(), with theta_basis and fault, as
    compute() reads it), rail_measurements and rail_rule_measurements (with
    the run's `generations`) and rail_flight_fields, and the scenario
    events' refusal gate (`event_refusals_as_expected`, the node's
    metrics.json `scenario_events`). A Shield-on flight whose metrics hold
    no FSM record leaves `fsm` out, and score() reports the fail-safe label
    "not measured" - never a guess.

    TWO WINDOWS (2026-11-07; the user's ruling). The PERFORMANCE measures are
    read over the scenario's own window: the mission's rows (up to the FSM's
    terminal edge, where run_episode stops) before the scenario's time cap
    (`max_s`, else the cap the run recorded in metrics.json `scenario`), as
    run_episode measures its ticks - the goal counts only if reached within
    it; score() must be given the same rows (rail_scored_rows). The SAFETY
    INVARIANTS (RAIL_INVARIANT_FIELDS, P0 escapes) are the whole flight's:
    every row, past the cap and through the autopilot's landing, the
    autopilot's breaches and entries counted apart; `ended_safe` when the
    mission ended. The FSM record is the whole flight's too: the extension
    past the cap exists to show the fail-safe's outcome. `in_window` keeps
    the invariants' values over the scenario's window (the headless
    comparison), `rail_window` what was read where.

    `autopilot_texts` (2026-11-15): the autopilot's STATUSTEXT messages of the
    run (run_autopilot_texts; None: not read). With a takeover in metrics.json
    the fence-breach message beside it is kept as
    `rail_window.takeover_fence_breach` (takeover_fence_breach), which
    score() reads to tell a GeoFence backstop from any other takeover
    (failsafe_backstop)."""
    extra: dict[str, Any] = {"seed_applied": True}
    if max_s is None:
        max_s = scenario_max_s(metrics)
    # The scenario's declared ground (X-11, 2026-11-18): every judge below is
    # built over it (rail_judge_kw), as the flying Shield was, and the run's
    # own record must name the same file (`ground_check`; run_sitl, run_hil and
    # rescore_rail_run refuse one that does not, ground_flight_problems). None without one:
    # every call below is the one it was.
    ground = episode_ground(spec, policy)
    gkw = {} if ground is None else {"ground": ground}
    if ground is not None:
        extra["ground_check"] = rail_ground_check(spec, metrics)
    mission, landing = rail_mission_rows(rows)
    win, after = rail_window(mission, max_s)
    if isinstance(metrics.get("reached"), bool):
        # Reached within the cap: the node ends a mission at its target, so a
        # flight that went on past the cap reached it (if at all) after it.
        extra["reached_goal"] = metrics["reached"] and not after
    fsm = metrics.get("fsm")
    if spec.shield and isinstance(fsm, dict) and "failsafe_triggered" in fsm:
        extra["fsm"] = fsm
    extra.update(rail_measurements(win, policy, generations, **gkw))
    extra.update(rail_flight_fields(win, metrics))
    lk = metrics.get("shield_lookahead") if isinstance(metrics.get("shield_lookahead"),
                                                       dict) else {}
    extra.update(rail_rule_measurements(
        win, policy, generations, dt=float(dt or spec.dt or 0.1),
        lookahead_s=float(lk.get("horizon_s") or spec.lookahead_s or LOOKAHEAD_S),
        lookahead_dt_s=float(lk.get("dt_s") or spec.lookahead_dt_s or LOOKAHEAD_DT_S),
        **gkw))
    ev = metrics.get("scenario_events")
    if isinstance(ev, dict):
        extra["event_refusals_as_expected"] = ev.get("event_refusals_as_expected")
    if spec.stress.gps_dropout_s is not None:
        # The rail's measure of the headless `gps_dropout_ticks` (2026-11-07):
        # ticks whose position the Shield was told is not a valid fix (the
        # node's fix_ok False, SITL's GPS disabled). 0, never None, for a
        # flight on which the denial never reached the Shield, so the cell's
        # gate (min 1) fails rather than score a clean run.
        extra["gps_dropout_ticks"] = sum(1 for r in win if r.get("fix_ok") is False)
    if spec.shield and isinstance(metrics.get("rtl_watch"), dict):
        # The rail's measure of the headless `rtl_failed_ticks` (2026-11-14):
        # rows on which the Shield node's RTL watch failed the FSM's RTL
        # (G6's input; sitl/ros2_shield_node.py, THE RTL WATCH), over the
        # whole flight, as the FSM record it feeds. A node without the watch
        # (no `rtl_watch` in metrics.json) leaves it out: "not measured".
        extra["rtl_failed_ticks"] = sum(1 for r in rows if r.get("rtl_failed") is True)
    # ---- the safety invariants: the whole flight --------------------------
    # Keep-outs over every row, the landing included (a landing in a zone is
    # an entry); unsafe positions and departures up to the mission's end:
    # ArduCopter's RTL ends in a landing at home, below every altitude floor
    # by design, so the landing's own rows are neither (they are reported:
    # `after_mission_end`).
    prios = rail_rule_priorities(policy, generations)
    in_window = {f: extra.get(f) for f in RAIL_INVARIANT_FIELDS}
    whole = dict(rail_measurements(rows, policy, generations, **gkw))
    upto_end = rail_measurements(mission, policy, generations, **gkw)
    upto_fields = ("breaches", "autopilot_breaches", "autopilot_departures")
    if "autopilot_scope_basis" in whole:
        upto_fields += ("autopilot_keepout_breaches",)      # NO_RULE_NAMES
    for f in upto_fields:
        if f in upto_end:
            whole[f] = upto_end[f]
        else:
            whole.pop(f, None)
    whole.update({k: v for k, v in rail_flight_fields(rows, metrics).items()
                  if k in ("max_speed_flown", "all_actions_finite")})
    whole["ended_safe"] = (not mission[-1]["unsafe"]) if rows and "unsafe" in mission[-1] \
        else None
    for f in RAIL_INVARIANT_FIELDS:
        if f in whole:
            extra[f] = whole[f]
        else:
            extra.pop(f, None)               # not measurable over the whole flight
    if rows:
        kw = K.compute(rows, prios, {"reached_goal": extra.get("reached_goal"),
                                     "breaches": extra.get("breaches"),
                                     "shield": "on" if spec.shield else "off",
                                     **({"fsm": fsm} if "fsm" in extra else {})})
        kv = K.compute(win, prios, {"reached_goal": extra.get("reached_goal"),
                                    "breaches": in_window.get("breaches"),
                                    "shield": "on" if spec.shield else "off"}) if win else {}
        for f, v in kw.items():
            if f.startswith("p0_"):
                extra[f] = v
                in_window[f] = kv.get(f)
    extra["in_window"] = in_window
    if "rail_measurements_not_measured" in whole:
        extra["rail_measurements_not_measured"] = whole["rail_measurements_not_measured"]
    extra["rail_window"] = {
        "max_s": max_s, "rows": len(win), "rows_after_cap": len(after),
        "rows_after_mission_end": len(landing), "rows_flight": len(rows),
        "flight_s": float(rows[-1]["t"]) if rows and rows[-1].get("t") is not None else None,
        "read_over": ("performance gates: the mission's rows before the scenario's cap; "
                      "safety invariants (P0 escapes, breaches, polygon entries - the "
                      "autopilot's counted apart, gated on keep-outs, its corridor and "
                      "altitude-band departures reported - ended safe, speed, finite "
                      "actions): the whole flight; the FSM record: the whole flight")}
    scen = metrics.get("scenario") if isinstance(metrics.get("scenario"), dict) else {}
    add = flight_max_s_add(scen)
    if add:
        # A lengthened hil campaign's flight (--hil-max-s-add, 2026-11-14):
        # the cap it flew to and by how much that is past the profile's
        # length. The performance window above is still the scenario's own.
        extra["rail_window"].update(max_s_add=add, rail_max_s=scen.get("rail_max_s"),
                                    rail_max_s_profile=scen.get("rail_max_s_profile"))
    took = metrics.get("autopilot_took_over")
    if isinstance(took, dict) and isinstance(took.get("t"), (int, float)):
        # The autopilot took the aircraft (a GeoFence RTL, ...): the FSM
        # stopped stepping then (sitl/ros2_shield_node.py) and the flight
        # ended TAKEOVER_GRACE_S later, whatever its cap. A Loiter it froze
        # is censored for a reason flying longer does not touch.
        extra["rail_window"]["autopilot_took_over"] = {
            "t": round(float(took["t"]), 3), "mode": took.get("mode"),
            "previous": took.get("previous")}
        # Was it the GeoFence (2026-11-15, BACKSTOP_RULE)? The autopilot's own
        # "Fence Breached" beside the mode change, from the run's events.jsonl.
        extra["rail_window"]["takeover_fence_breach"] = takeover_fence_breach(
            autopilot_texts, float(took["t"]))
    if landing:
        # What the autopilot flew after the mission's end (RTL's descent and
        # landing): measured, its keep-out and polygon entries in the counts
        # above.
        extra["after_mission_end"] = {
            "rows": len(landing), "from_t": float(landing[0]["t"]),
            "final_up": float(landing[-1]["up"]),
            "unsafe_rows": sum(1 for r in landing if r.get("unsafe")),
            "ended": (metrics.get("after_terminal") or {}).get("ended")}
    return extra


def _wsl_path(p: Path) -> str:
    """A Windows path as WSL mounts it (D:\\x y\\z -> /mnt/d/x y/z); a POSIX
    path as it is."""
    s = str(p)
    if len(s) > 2 and s[1] == ":":
        return "/mnt/" + s[0].lower() + s[2:].replace("\\", "/")
    return Path(s).as_posix()


def write_campaign_files(spec: ScenarioSpec, meta: dict, root: Path | None = None
                         ) -> tuple[str | None, str]:
    """One mission's --events and --scenario-meta files, under
    CAMPAIGN_DIR/<run id>/ in the checkout (the desktop rail reads them
    there; the hil rail gets a copy at the same path on the Orin). Returns
    their repo-relative POSIX paths (events None for a scenario with none)."""
    base = root or ROOT
    rel = f"{CAMPAIGN_DIR}/{meta['run_id']}"
    d = base / rel
    d.mkdir(parents=True, exist_ok=True)
    ev_rel = None
    if spec.events:
        (d / "events.json").write_text(
            json.dumps([e.model_dump(mode="json") for e in spec.events], indent=1) + "\n",
            encoding="utf-8", newline="\n")
        ev_rel = f"{rel}/events.json"
    (d / "scenario.json").write_text(json.dumps(_json_safe(meta), indent=1) + "\n",
                                     encoding="utf-8", newline="\n")
    return ev_rel, f"{rel}/scenario.json"


def rail_meta(spec: ScenarioSpec, ep, seed: int, backend: str, run_id: str,
              nonce: str, home: str | None, campaign: str | None, defaults=None,
              max_s_add: float = 0.0, physics: str = "sitl",
              sim_speedup: float = 1.0) -> dict:
    """The record a rail mission carries of its scenario (--scenario-meta;
    metrics.json `scenario`): which cell of which template, seed and arm,
    the family row tools/kpi_report.py files it under
    ("<backend>-stress/<template>/<cell>/shield-<arm>"), the nonce that ties
    the fetched run to this launch, and what the rail was told to fly -
    since 2026-11-07 with the stress it realises (`stress`: the cell's
    stressors, the start jitter drawn for the seed, the wind as SITL is
    given it, the dropout window; how the simulator side applied them is in
    its own plan and record, rail_stress_check).

    A lengthened hil campaign (`max_s_add` > 0, --hil-max-s-add, 2026-11-14)
    says so on every mission: `rail_max_s` is the cap it flew to,
    `rail_max_s_profile` the rail's own for the cell, `max_s_add` the
    difference, `flight_length` "lengthened", and its family row ends in
    "/flown-longer-<S>s". `max_s` stays the scenario's own cap (the
    performance window). A mission at the profile's length carries none of
    these keys, exactly as before.

    A PREFIX-CONSUMER episode (2026-11-16) carries its arm record
    (`arm_record`: build_arm's, with the CSP identity the VLA node must
    announce - prefix_flight_problems), a family row of its own
    ("/prefix-consumer-on" / "-off" after the Shield arm) and, on the
    prefix-on arm, no outcome or fail-safe label (`labels_not_applied`,
    scoring_spec). Every other mission carries none of these keys.

    A FUNCTIONAL-RAIL mission (`physics` "gazebo", 2026-11-17, WP4-04)
    carries `rail`, `physics`, `sim_speedup_requested`, `functional_only`
    and why (gazebo_meta): guardrail.manifest.is_kpi_grade refuses it
    whatever its manifest reads, and its family row starts "gazebo-stress/"
    (tools/kpi_report.py reads `rail` first), so it never pools with a SITL
    or hil row. A mission on SITL's own physics carries none of these keys."""
    pol = spec.policy.load(ROOT)
    rm = rail_mission(spec, pol, defaults, seed)
    arm = "on" if spec.shield else "off"
    cell = ep.episode_id if ep is not None else spec.scenario_id
    add = float(max_s_add or 0.0)
    length = ({"rail_max_s_profile": rail_max_s(rm["max_s"]), "max_s_add": add,
               "flight_length": "lengthened"} if add else {})
    consumer = ep is not None and bool(getattr(ep, "prefix_consumer", None))
    scored = scoring_spec(spec, ep)
    if consumer:
        length = dict(length, arm_record=build_arm(ep, pol, defaults=defaults),
                      **({"labels_not_applied": LABELS_NOT_APPLIED}
                         if scored is not spec else {}))
    check_rail_physics(backend, physics, sim_speedup)
    functional = gazebo_meta(sim_speedup) if physics == "gazebo" else {}
    row = functional.get("rail", backend)
    return {"schema": "rail-scenario/1", "backend": backend, "run_id": run_id,
            "run_nonce": nonce, "campaign": campaign,
            "scenario_id": spec.scenario_id, "template": spec.template, "cell": cell,
            "params": ep.cell.params if ep is not None else {},
            "seed": int(seed), "arm": arm,
            "arm_tag": ep.arm_tag if ep is not None else "",
            **functional,
            "family": (f"{row}-stress/{spec.template}/{cell}/shield-{arm}"
                       + prefix_family_suffix(ep) + lengthened_suffix(add)),
            "expect": spec.expect, "expected_outcome": scored.expected_outcome,
            "expected_failsafe": scored.expected_failsafe,
            "sitl_home": home, "start": list(rm["start"]), "yaw_deg": rm["yaw_deg"],
            "target": None if rm["target"] is None else list(rm["target"]),
            "pilot": rm["pilot"], "action": rm["action"], "speed_mps": rm["speed"],
            "max_s": rm["max_s"], "rail_max_s": rail_max_s(rm["max_s"], add),
            **length,
            "events": len(spec.events),
            "stress": {"declared": spec.stress.model_dump(mode="json", exclude_defaults=True),
                       "start_jitter_m": [round(v, 6) for v in rm["jitter_m"]],
                       "wind": rail_wind(spec, seed),
                       "gps_dropout_s": (None if spec.stress.gps_dropout_s is None
                                         else list(spec.stress.gps_dropout_s))},
            "policy": spec.policy.source,
            "harness": f"experiments/sweep_scenarios.py --backend {backend}"
                       + (f" --rail-physics {physics} --rail-sim-speedup {float(sim_speedup):g}"
                          if functional else "")}


# --------------------------------------------------------------------------- #
# THE GAZEBO FUNCTIONAL RAIL (2026-11-17, WP4-04)
# --------------------------------------------------------------------------- #
# The grant's functional rail is Gazebo Harmonic (03-simulation/dual-rail.md;
# 02-implementation/stress-testing.md: "Gazebo Harmonic for functional
# sweeps"). It first flew end to end on 2026-11-02 (ARCH-16):
# sitl/run_ros2_demo.sh with PHYSICS=gazebo, the desktop's ROS 2 rail with
# Gazebo's physics behind SITL's JSON backend. `--backend sitl --rail-physics
# gazebo` (or a profile's `rail_physics: gazebo`,
# experiments/profiles/gazebo_functional.yaml) flies the sweep's missions on
# it: PHYSICS=gazebo through WSLENV; tags "gzsweep_..." so its run folders
# never overwrite the SITL rail's; every mission's scenario record marked
# functional_only (rail_meta), which guardrail.manifest.is_kpi_grade refuses.
# A cell with a simulator stressor (wind, GPS dropout: SITL's SIM_*
# parameters) is refused there: they were checked on SITL's own physics
# only, and SITL's wind does not act on a vehicle whose dynamics Gazebo
# computes. Every mission flies at sim_speedup 1.0
# (scenario_spec.RAIL_SIM_SPEEDUP_WHY: the Shield node runs on wall time).
# `--sitl-dry-run` lists every mission - tag, SITL_HOME, environment,
# command and the run folder the rail will write - with nothing flown or
# written.
GAZEBO_TAG_PREFIX = "gzsweep_"
# Why a functional-rail mission is never KPI-grade. The lab's choice, not the
# grant's wording: the grant reports every KPI from a run "executed in the
# canonical HIL topology" (02-implementation/stress-testing.md, Purpose), and
# this rail runs the Shield on the desktop (`dev`); but its
# 03-simulation/dual-rail.md also gives the functional rail the "determinism
# contract for KPI runs" at sim_speedup=1, and the sentence that keeps a
# sweep's results out of the KPI numbers ("tracked but not part of the final
# KPI numbers") covers only the broad sweep at speed > 1, which is not flown.
GAZEBO_FUNCTIONAL_WHY = (
    "flown on the Gazebo Harmonic functional rail (dev topology, sim_speedup 1.0): "
    "tracked as functional evidence, never KPI-grade - the lab's choice to take "
    "KPI-grade runs from hil only (2026-11-17, WP4-04), not the grant's wording, "
    "whose 03-simulation/dual-rail.md gives the functional rail KPI runs at "
    "sim_speedup=1")
# The grant's per-episode bundle name (`episode-<UTC>--<cell>--seed<n>`,
# Episode.dir_name) is written by the headless backend only (write_bundle).
# A rail mission's bundle is demo/out/<tag>/<tag>.replay.tar.gz; the sweep
# records the grant's name beside it as a label, which names no file (WP4-11
# stays open), and its <UTC> is the sweep's start, not the episode's.
GRANT_EPISODE_NAME_NOTE = (
    "a name only: no file of this name is written on the rail (its bundle is "
    "the run folder's <tag>.replay.tar.gz); <UTC> is the sweep's start stamp; "
    "WP4-11's per-episode bundle naming stays open")
# Wall seconds a desktop mission spends outside its flight, for the dry run's
# estimate only: gz sim 6, SITL 8, router 3, MAVROS 10, bag 2 (the sleeps in
# sitl/run_ros2_demo.sh), plus take-off, the home check, the replay pack and
# the teardown. An estimate, not a measurement.
RAIL_MISSION_OVERHEAD_S = 90.0
# How far from north a target-less cell's start yaw may be on Gazebo's
# physics (gazebo_refusal).
GAZEBO_YAW_TOL_DEG = 2.0


def check_rail_physics(backend: str, physics: str, sim_speedup: float) -> None:
    """Refuse a physics or speedup the rail cannot honestly fly: physics one
    of RAIL_PHYSICS, Gazebo on the desktop rail (`sitl`) only - the hil rig's
    SITL runs its own quad - and any speedup but 1.0, on every rail
    (RAIL_SIM_SPEEDUP_WHY). Raises ValueError."""
    if physics not in RAIL_PHYSICS:
        raise ValueError(f"rail physics {physics!r}: one of {list(RAIL_PHYSICS)}")
    if physics == "gazebo" and backend != "sitl":
        raise ValueError(f"the Gazebo functional rail is the desktop's ROS 2 rail: "
                         f"--backend sitl, not {backend}")
    try:
        s = float(sim_speedup)
    except (TypeError, ValueError):
        raise ValueError(f"sim_speedup {sim_speedup!r}: not a number") from None
    if s != 1.0:
        raise ValueError(f"sim_speedup {sim_speedup!r}: {RAIL_SIM_SPEEDUP_WHY}")


def gazebo_meta(sim_speedup: float) -> dict:
    """What a functional-rail mission's scenario record says of its rail."""
    return {"rail": "gazebo", "physics": "gazebo-harmonic",
            "sim_speedup_requested": float(sim_speedup),
            "functional_only": True, "functional_only_why": GAZEBO_FUNCTIONAL_WHY}


def rail_physics_env(physics: str) -> dict[str, str]:
    """The variables sitl/run_ros2_demo.sh reads for the rail's physics,
    passed through WSLENV: none on SITL's own physics (the launch is then
    exactly what it was), PHYSICS=gazebo on the functional rail. Never
    SITL_SPEEDUP: every rail flies at 1.0 (RAIL_SIM_SPEEDUP_WHY)."""
    return {"PHYSICS": "gazebo"} if physics == "gazebo" else {}


def sitl_tag(ep, physics: str = "sitl") -> str:
    """A desktop mission's tag (its demo/out folder): "sweep_<cell>--seed<n>"
    as always, "gzsweep_..." on the Gazebo functional rail."""
    rest = ep.dir_name("x")[len("episode-x--"):]
    return (GAZEBO_TAG_PREFIX if physics == "gazebo" else "sweep_") + rest


def gazebo_refusal(spec: ScenarioSpec, seed: int = 0, defaults=None) -> str | None:
    """Why the Gazebo functional rail does not fly this cell beyond what
    rail_refusal says for the SITL rail, or None: a cell that needs the
    simulator's stressors (rail_stress_plan), or one with no target whose
    start yaw is not north: Gazebo's physics gives the vehicle the world's
    spawn attitude (iris_runway's iris read yaw 359 deg at the start of the
    2026-11-02 flight, SITL_HOME heading 0), and SITL_HOME's heading has not
    been shown to turn it; a cell with a target is flown start to target
    whatever the yaw (the goto pilot), one without would fly its body-frame
    action along another heading than the scenario's."""
    plan = rail_stress_plan(spec, seed, defaults)
    if plan is not None:
        what = sorted({b["param"] for b in plan["boot"]}
                      | {s["param"] for s in plan["schedule"]})
        return ("the Gazebo functional rail cannot fly this scenario as written: its "
                f"simulator stressors ({', '.join(what)}) are SITL's SIM_* parameters, "
                "checked on SITL's own physics only (SITL's wind, for one, does not act "
                "on a vehicle whose dynamics Gazebo computes)")
    rm = rail_mission(spec, spec.policy.load(ROOT), defaults, seed)
    yaw = float(rm["yaw_deg"]) % 360.0
    if rm["target"] is None and min(yaw, 360.0 - yaw) > GAZEBO_YAW_TOL_DEG:
        return ("the Gazebo functional rail cannot fly this scenario as written: a start "
                f"yaw of {yaw:.1f} deg with no target; the vehicle starts with the "
                "Gazebo world's spawn attitude (north), and SITL_HOME's heading has not "
                "been shown to turn it")
    return None


def sitl_plan(episodes, defaults, physics: str = "sitl", sim_speedup: float = 1.0,
              stamp: str = "<UTC>") -> list[dict]:
    """The desktop rail's sweep before anything flies (--sitl-dry-run): per
    episode, the mission it would be - tag, SITL_HOME, the environment and
    command run_sitl would launch and the run folder and replay bundle the
    rail writes - or why it is refused. `grant_episode_name` is the grant's
    per-episode name (`episode-<UTC>--<cell>--seed<n>`, Episode.dir_name), a
    label only: the rail writes no file of that name (GRANT_EPISODE_NAME_NOTE).
    Needs neither WSL nor Gazebo; writes nothing."""
    check_rail_physics("sitl", physics, sim_speedup)
    plan = []
    for ep in episodes:
        spec = ep.cell.spec
        item = {"id": ep.run_id, "seed": ep.seed, "template": spec.template,
                "cell": ep.episode_id, "grant_episode_name": ep.dir_name(stamp)}
        why = rail_refusal(spec, "sitl", episode=ep, defaults=defaults)
        if why is None and physics == "gazebo":
            why = gazebo_refusal(spec, ep.seed, defaults)
        if why is None:
            try:
                pol = spec.policy.load(ROOT)
                tag = sitl_tag(ep, physics)
                rel = f"{CAMPAIGN_DIR}/{tag}"
                cmd = build_sitl_cmd(spec, tag, ep.seed, defaults,
                                     events=(f"{rel}/events.json" if spec.events else None),
                                     meta=f"{rel}/scenario.json", episode=ep)
                home = sitl_home(spec, pol, defaults, ep.seed)
                env = dict(rail_physics_env(physics))
                if home is not None:
                    env["SITL_HOME"] = home
                stress = rail_stress_plan(spec, ep.seed, defaults, run_id=tag)
                dfl = _as_defaults(defaults)
                cap = (spec.ticks or dfl.ticks) * (spec.dt or dfl.dt)     # rail_mission's
                item.update(fly=True, tag=tag, sitl_home=home, env=env, cmd=cmd,
                            max_s=cap, rail_max_s=rail_max_s(cap),
                            shield="on" if spec.shield else "off",
                            stress=None if stress is None else {
                                "boot": stress["boot"], "schedule": stress["schedule"]},
                            run_folder=f"demo/out/{tag}",
                            replay_bundle=f"demo/out/{tag}/{tag}.replay.tar.gz",
                            functional_only=physics == "gazebo")
            except UnsupportedScenario as e:
                why = str(e)
        if why is not None:
            item.update(fly=False, refused=why)
        plan.append(item)
    return plan


def print_sitl_plan(plan: list[dict], physics: str = "sitl",
                    sim_speedup: float = 1.0) -> None:
    """The dry run's listing: every mission with its environment, command,
    run folder and the grant's episode name (a label, no file), every
    refusal with its reason, the counts and an estimate of the wall time
    (the flights' caps - the Shield node's mission cap is in wall seconds -
    plus RAIL_MISSION_OVERHEAD_S each: an upper estimate, not a
    measurement)."""
    fly = [p for p in plan if p["fly"]]
    rail = ("the Gazebo Harmonic functional rail (PHYSICS=gazebo)" if physics == "gazebo"
            else "the desktop's SITL rail")
    print(f"sitl sweep plan on {rail}: {len(plan)} episode(s), {len(fly)} mission(s) to "
          f"fly, {len(plan) - len(fly)} refused (nothing flown, nothing written)\n")
    if physics == "gazebo":
        print(f"  FUNCTIONAL, NOT KPI-GRADE: sim_speedup {float(sim_speedup):g}; every "
              f"mission's scenario record says functional_only\n")
    for p in fly:
        print(f"  FLY  {p['tag']}  {p['id']}  seed={p['seed']}  shield {p['shield']}  "
              f"(cap {p['rail_max_s']:g} s)")
        print(f"       env        " + (" ".join(f"{k}={v}" for k, v in p["env"].items())
                                       or "(none)"))
        print(f"       command    {shlex.join(p['cmd'])}")
        print(f"       run folder {p['run_folder']}  (replay {p['replay_bundle']})")
        print(f"       grant name {p['grant_episode_name']}  (a name only: no file of "
              f"this name is written)")
    refused = [p for p in plan if not p["fly"]]
    if refused:
        print("\n  refused:")
        for p in refused:
            print(f"  REF  {p['id']}  seed={p['seed']}\n       {p['refused']}")
    if fly:
        print(f"\n  grant names: {GRANT_EPISODE_NAME_NOTE}")
    # The cap is wall seconds (the node's time.monotonic), whatever the
    # simulator's speed: never divided by a speedup.
    flight = sum(p["rail_max_s"] for p in fly)
    est = flight + RAIL_MISSION_OVERHEAD_S * len(fly)
    print(f"\n  estimate: at most {est / 60:.0f} min wall ({flight / 60:.1f} min of flight "
          f"caps, {RAIL_MISSION_OVERHEAD_S:g} s bring-up and teardown per mission); an "
          f"estimate, not a measurement")


def run_sitl(sc, tag: str, seed: int = 0, defaults=None,
             episode=None, *, extra_args=None, physics: str = "sitl",
             sim_speedup: float = 1.0,
             rail_battery: bool = False) -> tuple[list[dict], dict] | None:
    """One scenario on the desktop's rail (sitl/run_ros2_demo.sh, which boots
    a fresh SITL per run): SITL_HOME at the scenario's start, through WSLENV
    so the WSL script sees it; its events and scenario record written under
    CAMPAIGN_DIR first. None when the rail is not on this host.

    THE SIMULATOR'S STRESSORS (2026-11-07): a cell with wind, a GPS
    dropout or (since 2026-11-17, with `rail_battery`, the campaign's
    opt-in) a battery hook gets its plan (rail_stress_plan) under
    RAIL_STRESS_DIR, and
    run_ros2_demo.sh (SITL_STRESS_PLAN / SITL_STRESS_RECORD through WSLENV)
    starts sitl/sitl_stressors.py beside SITL before MAVROS, refusing the run
    unless its boot parameters read back. After the flight the record goes
    beside the run (sitl_stress.json) and rail_stress_check must find the
    stress realised, or the flight is an error, not a scored episode."""
    spec = _as_spec(sc)
    check_rail_physics("sitl", physics, sim_speedup)
    if not sitl_available():
        return None
    why = rail_refusal(spec, "sitl", episode=episode, defaults=defaults,
                       rail_battery=rail_battery)
    if why is None and physics == "gazebo":
        why = gazebo_refusal(spec, seed, defaults)
    if why:
        raise UnsupportedScenario(why)
    pol = spec.policy.load(ROOT)
    home = sitl_home(spec, pol, defaults, seed)
    nonce = uuid.uuid4().hex
    meta = rail_meta(spec, episode, seed, "sitl", tag, nonce, home, None, defaults,
                     physics=physics, sim_speedup=sim_speedup)
    events, meta_rel = write_campaign_files(spec, meta)
    cmd = build_sitl_cmd(spec, tag, seed, defaults, events=events, meta=meta_rel,
                         extra_args=extra_args, episode=episode)
    env = dict(os.environ)
    wslenv = [env.get("WSLENV", "")]
    if home is not None:
        env["SITL_HOME"] = home
        wslenv.append("SITL_HOME/u")
    # The functional rail's physics (2026-11-17): none on SITL's own, so
    # that launch is exactly what it was.
    for k, v in rail_physics_env(physics).items():
        env[k] = v
        wslenv.append(f"{k}/u")
    plan = rail_stress_plan(spec, seed, defaults, run_id=tag, nonce=nonce)
    record_path = None
    if plan is not None:
        plan_path, record_path = write_stress_plan(plan)
        env["SITL_STRESS_PLAN"] = _wsl_path(plan_path)
        env["SITL_STRESS_RECORD"] = _wsl_path(record_path)
        wslenv += ["SITL_STRESS_PLAN/u", "SITL_STRESS_RECORD/u"]
    if len(wslenv) > 1:
        env["WSLENV"] = ":".join(p for p in wslenv if p)
    out_dir = ROOT / "demo" / "out" / tag
    launched_at = time.time()
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, env=env)
    if r.returncode != 0:
        raise RuntimeError(f"SITL run failed ({r.returncode}): "
                           f"{(r.stderr or r.stdout)[-400:]}")
    rows = read_fresh_log(out_dir / "flight_log.jsonl", launched_at)
    metrics = read_fresh_metrics(out_dir / "metrics.json", launched_at)
    record = read_stress_record(record_path)
    if record is not None:
        (out_dir / "sitl_stress.json").write_text(
            json.dumps(record, indent=1) + "\n", encoding="utf-8", newline="\n")
    problems, stress = rail_stress_check(spec, plan, record, metrics, rows, seed)
    if problems:
        raise RuntimeError(f"{tag}: the cell's stress was not realised: "
                           + "; ".join(problems))
    gens = run_generations(out_dir)
    not_the_arm = (prefix_flight_problems(metrics, meta)
                   + prefix_generation_problems(meta, gens, rows))
    if not_the_arm:
        raise RuntimeError(f"{tag}: not the prefix arm's flight: " + "; ".join(not_the_arm))
    phys = None
    if physics == "gazebo":
        phys = gazebo_flight_problems(out_dir, launched_at, metrics, sim_speedup)
        if phys["problems"]:
            raise RuntimeError(f"{tag}: not a Gazebo functional-rail flight: "
                               + "; ".join(phys["problems"]))
    arc_problems = arc_forecast_problems(metrics, extra_args)
    if arc_problems:
        # Fail closed (--rail-arc-forecast): never scored under the wrong arm.
        raise RuntimeError(f"{tag}: not the arc arm's flight: " + "; ".join(arc_problems))
    ground_problems = ground_flight_problems(spec, metrics)
    if ground_problems:
        # Fail closed (X-11): never scored over a ground it did not fly.
        raise RuntimeError(f"{tag}: not flown over the scenario's ground: "
                           + "; ".join(ground_problems))
    extra = sitl_extra(spec, rows, metrics, pol, gens,
                       max_s=meta["max_s"], dt=spec.dt or _as_defaults(defaults).dt,
                       autopilot_texts=run_autopilot_texts(out_dir))
    extra["sim_stressors"] = stress["entries"]
    extra["sim_stressors_check"] = stress
    if meta.get("arm_record"):
        extra["prefix_consumer"] = prefix_flight_record(metrics, meta)
    if phys is not None:
        extra["rail_physics"] = phys["physics"]
    # score() reads the scenario's own window, as sitl_extra did.
    return rail_scored_rows(rows, meta["max_s"]), extra


def gazebo_flight_problems(out_dir: Path, launched_at: float, metrics: dict,
                           sim_speedup: float) -> dict:
    """Was this launch a flight on the Gazebo functional rail? Read from what
    the rail wrote, not from what it was asked: physics.json from THIS launch
    saying gazebo-harmonic at the requested speedup, and the node's
    metrics.json carrying the scenario record marked functional_only (so the
    node's own kpi.json and tools/kpi_report.py refuse it as KPI). Returns
    {"problems": [...], "physics": the physics.json record or None}."""
    problems: list[str] = []
    p = out_dir / "physics.json"
    rec = None
    if not p.is_file():
        problems.append("the rail wrote no physics.json")
    elif p.stat().st_mtime < launched_at - 1.0:
        problems.append("physics.json predates this launch")
    else:
        try:
            rec = json.loads(p.read_text(encoding="utf-8"))
        except ValueError as e:
            problems.append(f"physics.json does not parse ({e})")
    if rec is not None:
        if rec.get("physics") != "gazebo-harmonic":
            problems.append(f"physics.json says {rec.get('physics')!r}, not 'gazebo-harmonic'")
        asked = rec.get("sim_speedup_requested")
        if asked is None or float(asked) != float(sim_speedup):
            problems.append(f"physics.json's sim_speedup_requested is {asked!r}, not "
                            f"{float(sim_speedup):g}")
    if not ((metrics or {}).get("scenario") or {}).get("functional_only"):
        problems.append("metrics.json's scenario record is not marked functional_only")
    if rec is not None:
        # Beside what was asked, what the autopilot reported (the manifest's
        # sim_speedup, read from SIM_SPEEDUP by the node): recorded, not
        # judged - the run is functional whatever it reads.
        man = out_dir / "manifest.json"
        try:
            got = json.loads(man.read_text(encoding="utf-8")).get("sim_speedup")
        except (OSError, ValueError):
            got = None
        rec = dict(rec, manifest_sim_speedup=got)
    return {"problems": problems, "physics": rec}


# --------------------------------------------------------------------------- #
# the hil rail: SITL on the desktop, the mission on the Jetson Orin
# --------------------------------------------------------------------------- #

@dataclass
class HilConfig:
    """Where the hil rig is (`--hil-config FILE`, JSON or YAML; every key
    optional). The defaults are the rig of 2026-11-06: the AGX Orin over the
    cable (deploy/topologies/hil.env), its clean checkout ~/vla-drone, SITL
    and mavlink-routerd in this desktop's WSL distro."""
    ssh_target: str = "nathan-agxiot@10.42.0.3"
    remote_root: str = "~/vla-drone"
    env_file: str = "deploy/topologies/hil.env"
    compose_file: str = "deploy/compose/docker-compose.orin.yml"
    wsl_distro: str = "Ubuntu"
    # This checkout as WSL sees it (default: /mnt/<drive>/... of the repo).
    wsl_repo: str | None = None
    # Where the router sends MAVROS's MAVLink: the Orin's cable address.
    mavros_host: str = "10.42.0.3"
    # The local run store the fetched run folders go to (default
    # demo/out/hil_<profile or library>).
    store: str | None = None
    # Where flight_bundle_status looks for the signed bundles (default the
    # checkout's bundles/).
    bundles_dir: str | None = None
    # SITL's EKF settles this long after boot before the mission starts.
    sitl_settle_s: float = 20.0
    sitl_ready_timeout_s: float = 120.0
    # A mission may take its --max-s plus this (MAVROS, bring-up, the home
    # check, take-off, the replay pack).
    mission_margin_s: float = 420.0
    restart_mavros: bool = True
    ssh_options: tuple = ("-o", "BatchMode=yes", "-o", "ConnectTimeout=15")
    # VLAGUARD_REQUIRE_CHAIN for the mission container (WP1-04, 2026-11-16):
    # True/False is passed to `docker compose run` as -e, overriding the
    # topology file's value. None (the default): this desktop's own
    # VLAGUARD_REQUIRE_CHAIN when it is set, else nothing is passed and the
    # container gets the topology file's value (hil.env: 0). ssh forwards no
    # variable of this shell, so an `export` here reached the Orin only
    # through this.
    require_chain: bool | None = None

    @classmethod
    def load(cls, path: str | Path | None) -> "HilConfig":
        if not path:
            return cls()
        p = Path(path)
        p = p if p.is_absolute() else ROOT / p
        raw = (yaml.safe_load(p.read_text(encoding="utf-8")) if p.suffix in (".yaml", ".yml")
               else json.loads(p.read_text(encoding="utf-8"))) or {}
        known = {f for f in cls.__dataclass_fields__}
        bad = sorted(set(raw) - known)
        if bad:
            raise ValueError(f"--hil-config {path}: unknown key(s) {bad} (known: "
                             f"{sorted(known)})")
        if "ssh_options" in raw:
            raw["ssh_options"] = tuple(raw["ssh_options"])
        return cls(**raw)

    def remote_rel(self, rel: str) -> str:
        """A checkout-relative path on the Orin, for scp: relative to the
        user's home when the checkout is under ~ (scp's own convention)."""
        root = self.remote_root.rstrip("/")
        if root.startswith("~/"):
            root = root[2:]
        elif root == "~":
            root = ""
        return f"{root}/{rel}" if root else rel

    def compose(self) -> str:
        return (f"docker compose --env-file {shlex.quote(self.env_file)} "
                f"-f {shlex.quote(self.compose_file)}")

    def chain_required(self) -> bool | None:
        """The VLAGUARD_REQUIRE_CHAIN a mission is launched with: the config's
        `require_chain`, else this desktop's VLAGUARD_REQUIRE_CHAIN when set
        (read as guardrail.bundle.load_for_flight reads it), else None - the
        topology file decides."""
        if self.require_chain is not None:
            return bool(self.require_chain)
        raw = os.environ.get("VLAGUARD_REQUIRE_CHAIN")
        if raw is None or not raw.strip():
            return None
        return raw.strip().lower() not in ("0", "false", "no")

    def mission_env(self) -> str:
        """`-e VLAGUARD_REQUIRE_CHAIN=<0|1> ` for `docker compose run` (it
        overrides the service's environment, the topology file's value, for
        that container only), or "" when nothing is asked: then the command
        is what it always was."""
        chain = self.chain_required()
        return "" if chain is None else f"-e VLAGUARD_REQUIRE_CHAIN={int(chain)} "


HIL_READY = "HIL_DESKTOP READY"
HIL_HELPER = "sitl/hil_desktop.sh"


class SubprocessTransport:
    """The hil rig, driven from this desktop: SITL and mavlink-routerd in WSL
    through sitl/hil_desktop.sh, the Orin over ssh / scp. Every call goes
    through `self.run` / `self.popen` (subprocess's), so a test can hand in
    fakes; the harness's tests use a whole fake transport instead
    (`make_hil_transport`)."""

    def __init__(self, cfg: HilConfig, run=None, popen=None) -> None:
        self.cfg = cfg
        self.run = run or subprocess.run
        self.popen = popen or subprocess.Popen
        self._sitl = None

    # ---- the desktop ----
    def _helper(self) -> list[str]:
        repo = self.cfg.wsl_repo or _wsl_path(ROOT)
        return ["wsl.exe", "-d", self.cfg.wsl_distro, "--", "bash",
                f"{repo.rstrip('/')}/{HIL_HELPER}"]

    def _wsl(self, p) -> str:
        """A path as the WSL helper sees it: under the checkout, through
        `wsl_repo` (the checkout as WSL sees it); elsewhere, /mnt/<drive>."""
        p = Path(p)
        try:
            rel = p.resolve().relative_to(ROOT.resolve())
        except ValueError:
            return _wsl_path(p)
        repo = (self.cfg.wsl_repo or _wsl_path(ROOT)).rstrip("/")
        return f"{repo}/{rel.as_posix()}"

    def restart_sitl(self, home: str | None, stress: tuple | None = None) -> str:
        """Kill SITL and the router, boot SITL at `home` (None: start_sitl's
        CMAC default) and the router toward the Orin, wait for the EKF; the
        helper stays in the foreground (keeping the WSL distro up) until
        stop_sitl. `stress` (2026-11-07): (plan path, record path) of the
        simulator's stressors (rail_stress_plan) - the helper then starts
        sitl/sitl_stressors.py beside SITL and prints its ready line only
        once the plan's boot parameters read back. Returns its output up to
        the ready line."""
        self.stop_sitl()
        cmd = self._helper() + ["start", "--mavros-host", self.cfg.mavros_host,
                                "--settle-s", f"{self.cfg.sitl_settle_s:g}"]
        if home:
            cmd += ["--home", home]
        if stress:
            cmd += ["--stress-plan", self._wsl(stress[0]),
                    "--stress-record", self._wsl(stress[1])]
        proc = self.popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                          errors="replace")
        self._sitl = proc
        lines: list[str] = []
        done = threading.Event()

        def reader():
            for line in proc.stdout:
                lines.append(line.rstrip("\n"))
                if HIL_READY in line:
                    done.set()
                    return
            done.set()
        threading.Thread(target=reader, daemon=True).start()
        done.wait(self.cfg.sitl_ready_timeout_s)
        if not any(HIL_READY in ln for ln in lines):
            self.stop_sitl()
            raise RuntimeError("SITL did not come up (sitl/hil_desktop.sh): "
                               + " | ".join(lines[-8:]))
        return "\n".join(lines)

    def stop_sitl(self) -> None:
        if self._sitl is not None:
            self.run(self._helper() + ["stop"], capture_output=True, text=True,
                     stdin=subprocess.DEVNULL, timeout=60)
            try:
                self._sitl.terminate()
            except Exception:                                 # noqa: BLE001
                pass
            self._sitl = None

    # ---- the Orin ----
    def _ssh(self, remote_cmd: str, timeout: float) -> tuple[int, str]:
        # stdin is never the harness's own: Windows OpenSSH reading an
        # inherited pipe kept `ssh ... mkdir -p` open until the 60 s timeout
        # on the second shakedown (shk02, 2026-11-07) though the command takes
        # 0.4 s; the same for scp and the WSL helper below.
        r = self.run(["ssh", *self.cfg.ssh_options, self.cfg.ssh_target, remote_cmd],
                     capture_output=True, text=True, encoding="utf-8", errors="replace",
                     stdin=subprocess.DEVNULL, timeout=timeout)
        return r.returncode, (r.stdout or "") + (r.stderr or "")

    def push(self, local_dir: Path, remote_rel: str) -> None:
        """Copy `local_dir` to <remote_root>/<remote_rel> on the Orin."""
        parent = remote_rel.rsplit("/", 1)[0]
        rc, out = self._ssh(f"mkdir -p {shlex.quote(self.cfg.remote_rel(parent))}", 60)
        if rc != 0:
            raise RuntimeError(f"ssh mkdir on the Orin failed ({rc}): {out[-300:]}")
        r = self.run(["scp", "-r", "-q", *self.cfg.ssh_options, local_dir.name,
                      f"{self.cfg.ssh_target}:{self.cfg.remote_rel(parent)}/"],
                     cwd=str(local_dir.parent), capture_output=True, text=True,
                     stdin=subprocess.DEVNULL, timeout=120)
        if r.returncode != 0:
            raise RuntimeError(f"scp to the Orin failed ({r.returncode}): "
                               f"{(r.stderr or r.stdout)[-300:]}")

    def restart_mavros(self) -> None:
        rc, out = self._ssh(f"cd {self.cfg.remote_root} && {self.cfg.compose()} "
                            f"restart mavros", 180)
        if rc != 0:
            raise RuntimeError(f"restarting MAVROS on the Orin failed ({rc}): {out[-300:]}")

    def run_mission(self, args: list[str], tag: str, shield: str,
                    timeout_s: float) -> tuple[int, str]:
        remote = (f"cd {self.cfg.remote_root} && {self.cfg.compose()} run --rm -T "
                  f"{self.cfg.mission_env()}mission mission --shield {shield} "
                  f"--tag {shlex.quote(tag)} "
                  + " ".join(shlex.quote(a) for a in args))
        try:
            return self._ssh(remote, timeout_s)
        except subprocess.TimeoutExpired as e:
            # The container outlives the ssh session: stop it, best effort,
            # so the next mission does not fly beside it.
            try:
                self._ssh("docker ps -q --filter label=com.docker.compose.service=mission"
                          " | xargs -r docker kill", 60)
            except Exception:                                 # noqa: BLE001
                pass
            return 124, f"timed out after {timeout_s:g} s: {e}"

    def fetch(self, tag: str, local_parent: Path) -> Path:
        local_parent.mkdir(parents=True, exist_ok=True)
        r = self.run(["scp", "-r", "-q", *self.cfg.ssh_options,
                      f"{self.cfg.ssh_target}:{self.cfg.remote_rel('demo/out/' + tag)}", "."],
                     cwd=str(local_parent), capture_output=True, text=True,
                     stdin=subprocess.DEVNULL, timeout=600)
        if r.returncode != 0:
            raise RuntimeError(f"scp from the Orin failed ({r.returncode}): "
                               f"{(r.stderr or r.stdout)[-300:]}")
        return local_parent / tag


# The transport main() flies the hil campaign through; a test replaces it.
make_hil_transport = SubprocessTransport


def hil_run_id(campaign: str, k: int, spec: ScenarioSpec) -> str:
    """A unique mission tag: the campaign, the mission's number and a short
    slug of the scenario (the Shield node's out dir and the replay bundle's
    name carry it, so it stays short enough for a Windows path)."""
    slug = re.sub(r"[^A-Za-z0-9-]", "-", spec.scenario_id)[:28].strip("-")
    return f"{campaign}_{k:03d}_{slug}"


def accept_hil_run(run_dir: Path, nonce: str) -> tuple[list[dict], dict, dict]:
    """(rows, metrics, episode) of a fetched hil run, accepted by its own
    records, not by file times (the Orin's clock and scp's copies say
    nothing): the episode's `episode_start`, a `mission_end` after it,
    metrics.json and every row of the same episode, and the scenario record
    in metrics.json carrying THIS launch's nonce. Raises RuntimeError
    otherwise."""
    ev_path = run_dir / "events.jsonl"
    if not ev_path.is_file():
        raise RuntimeError(f"{run_dir.name}: no events.jsonl (the mission never started)")
    events = [json.loads(x) for x in ev_path.read_text(encoding="utf-8").splitlines()
              if x.strip()]
    starts = [i for i, e in enumerate(events) if e.get("kind") == "episode_start"]
    if not starts:
        raise RuntimeError(f"{run_dir.name}: no episode_start in events.jsonl")
    ep_id = events[starts[-1]].get("episode_id")
    ends = [e for e in events[starts[-1]:] if e.get("kind") == "mission_end"]
    if not ends:
        raise RuntimeError(f"{run_dir.name}: episode {ep_id} has no mission_end: the "
                           f"mission did not finish")
    mpath = run_dir / "metrics.json"
    if not mpath.is_file():
        raise RuntimeError(f"{run_dir.name}: no metrics.json")
    metrics = json.loads(mpath.read_text(encoding="utf-8"))
    if metrics.get("episode_id") != ep_id:
        raise RuntimeError(f"{run_dir.name}: metrics.json is episode "
                           f"{metrics.get('episode_id')}, events.jsonl {ep_id}")
    scen = metrics.get("scenario") if isinstance(metrics.get("scenario"), dict) else {}
    if scen.get("run_nonce") != nonce:
        raise RuntimeError(f"{run_dir.name}: its scenario record is not this launch's "
                           f"(nonce {scen.get('run_nonce')!r}, launched {nonce!r})")
    lp = run_dir / "flight_log.jsonl"
    if not lp.is_file():
        raise RuntimeError(f"{run_dir.name}: no flight_log.jsonl")
    rows = [json.loads(x) for x in lp.read_text(encoding="utf-8").splitlines() if x.strip()]
    stray = {r.get("episode_id") for r in rows} - {ep_id}
    if stray:
        raise RuntimeError(f"{run_dir.name}: rows of other episodes {sorted(map(str, stray))}")
    return rows, metrics, {"episode_id": ep_id, "mission_end": ends[0]}


SCENARIO_SCORE = "scenario_score.json"


def hil_flight_problems(metrics: dict) -> list[str]:
    """Why a hil flight is not the scenario's flight, though it flew: an
    event the Shield refused that the scenario does not expect refused, or
    events that never fired because the mission ended first (the headless
    harness flies every tick and applies every event)."""
    ev = metrics.get("scenario_events") if isinstance(metrics.get("scenario_events"), dict) else {}
    out = []
    if ev.get("unexpected_refusals"):
        out.append(f"the Shield refused scenario event(s) the scenario does not expect "
                   f"refused: {ev['unexpected_refusals']}")
    if ev.get("not_applied"):
        out.append(f"{len(ev['not_applied'])} scenario event(s) never fired (the mission "
                   f"ended first): {[e.get('index') for e in ev['not_applied']]}")
    return out


def hil_chain_problems(metrics: dict, required: bool | None) -> list[str]:
    """Why a mission launched with VLAGUARD_REQUIRE_CHAIN=1 (HilConfig.
    chain_required) is not evidence of a flight that REQUIRED the chain: its
    own policy_source must say the loader had require_chain on and the signer's
    certificate chained there (WP1-04, 2026-11-16). A run that only shows
    `chain: verified` says how the bundle was, not what the rail demanded."""
    if not required:
        return []
    src = metrics.get("policy_source") if isinstance(metrics.get("policy_source"), dict) else {}
    out = []
    if src.get("require_chain") is not True:
        out.append(f"launched with VLAGUARD_REQUIRE_CHAIN=1, but the run's policy_source "
                   f"says require_chain {src.get('require_chain')!r}: the requirement did "
                   f"not reach the flight loader")
    if (src.get("signature"), src.get("chain")) != ("verified", "verified"):
        out.append(f"launched with VLAGUARD_REQUIRE_CHAIN=1, but the run's policy_source "
                   f"says signature {src.get('signature')!r}, chain {src.get('chain')!r}")
    return out


_DESKTOP_REVISION: dict | None = None


def desktop_revision() -> dict:
    """This desktop's own revision record (guardrail.manifest.host_revision:
    code_revision with its dirty flag, git describe, the machine), read ONCE
    per process as manifest_base reads the sweep's: git status costs seconds
    on this tree, and a campaign runs from one checkout."""
    global _DESKTOP_REVISION
    if _DESKTOP_REVISION is None:
        from guardrail.manifest import host_revision
        _DESKTOP_REVISION = host_revision(repo=ROOT)
    return _DESKTOP_REVISION


_WSL_HELPER_REVISIONS: dict[tuple, dict] = {}


def wsl_helper_revision(cfg: HilConfig, run=None) -> dict | None:
    """The revision of the checkout the WSL helper runs from (sitl/
    hil_desktop.sh, start_sitl, sitl/sitl_stressors.py), when `cfg.wsl_repo`
    points at a checkout other than this one as WSL mounts it; None when it
    is this checkout (desktop_revision already covers it). Read through
    `wsl.exe -d <distro> -- git -C <repo>` with code_revision's rule (12-hex
    HEAD, "-dirty" by porcelain status, "-unknown" when status cannot
    tell, "unversioned" when HEAD does not resolve), once per process."""
    repo = (cfg.wsl_repo or "").rstrip("/")
    if not repo or repo == _wsl_path(ROOT).rstrip("/"):
        return None
    key = (cfg.wsl_distro, repo)
    if key in _WSL_HELPER_REVISIONS:
        return _WSL_HELPER_REVISIONS[key]
    run = run or subprocess.run
    from guardrail.manifest import _dirty_flag
    git = ["wsl.exe", "-d", cfg.wsl_distro, "--", "git", "-C", repo]
    rev = "unversioned"
    try:
        head = run(git + ["rev-parse", "HEAD"], capture_output=True, text=True,
                   timeout=60)
        if head.returncode == 0 and head.stdout.strip():
            rev = head.stdout.strip()[:12]
            try:
                st = run(git + ["status", "--porcelain"], capture_output=True,
                         text=True, timeout=120)
                ok = st.returncode == 0
                rev += ("-dirty" if st.stdout.strip() else "") if ok else "-unknown"
            except (OSError, subprocess.SubprocessError):
                rev += "-unknown"
    except (OSError, subprocess.SubprocessError):
        pass
    rec = {"repo": repo, "wsl_distro": cfg.wsl_distro, "code_revision": rev,
           "dirty": _dirty_flag(rev),
           "runs": "sitl/hil_desktop.sh, SITL's start script, sitl/sitl_stressors.py",
           "source": "git in the WSL helper's checkout (HilConfig.wsl_repo)"}
    _WSL_HELPER_REVISIONS[key] = rec
    return rec


def record_desktop_revision(run_dir: Path, cfg: HilConfig | None = None, *,
                            run=None) -> dict | None:
    """Add this desktop to a fetched hil run's stack record (guardrail.
    manifest.add_host_revision, ARCH-31), beside the Orin's own. The entry's
    code_revision is the harness's checkout (ROOT). SITL's helper and the
    stressors run from the same checkout unless `cfg.wsl_repo` names
    another: that checkout's revision is then the entry's `components.
    wsl_helper` (wsl_helper_revision), which the record checks like a host.
    Called only for a run the harness accepted. Evidence only: a failure is
    printed and the run is scored as before."""
    from guardrail.manifest import ROLE_DESKTOP, add_host_revision
    try:
        rec = dict(desktop_revision())
        helper = wsl_helper_revision(cfg, run=run) if cfg is not None else None
        if helper is not None:
            rec["components"] = {"wsl_helper": helper}
        return add_host_revision(run_dir, ROLE_DESKTOP, rec)
    except Exception as e:                                   # noqa: BLE001
        print(f"  [hil] stack_revisions.json: the desktop's revision not added "
              f"({type(e).__name__}: {e})")
        return None


def run_hil(spec: ScenarioSpec, ep, seed: int, defaults, cfg: HilConfig, transport,
            campaign: str, k: int, store: Path, *, extra_args=None,
            max_s_add: float = 0.0, rail_battery: bool = False) -> dict:
    """One scenario on the hil rail: boot SITL at its start, copy its files
    to the Orin, restart MAVROS, fly the mission container, fetch the run
    folder into `store`, accept it by its own records and score it with the
    same score() / episode_verdict as every episode. Writes
    scenario_score.json beside the run (tools/kpi_report.py merges it).
    Returns {rows, extra, policy, prios, run_dir, tag}; raises
    UnsupportedScenario (refused) or RuntimeError (the flight failed).

    THE SIMULATOR'S STRESSORS (2026-11-07): a cell with wind, a GPS
    dropout or (since 2026-11-17, opt-in: `rail_battery`) a battery hook
    gets its plan (rail_stress_plan, kept on this desktop under
    RAIL_STRESS_DIR - nothing of it goes to the Orin), which the WSL helper
    hands sitl/sitl_stressors.py; the helper is ready only once the plan's
    boot parameters read back. After the flight the simulator side's record
    is copied beside the run (sitl_stress.json) and rail_stress_check must
    find the stress realised, or the flight is an error, never scored.

    `extra_args` (2026-11-10): the Shield node's hot-apply listener flags
    (rail_extra_args), appended to the scenario's own arguments for the gRPC
    harness's rail episode, which hot-applies into the flight over REST. A
    campaign (`--backend hil`) passes none, and its missions are the ones
    they were.

    `max_s_add` (2026-11-14): a lengthened campaign's seconds past the rail
    cap (--hil-max-s-add), on the node's --max-s, in the scenario record
    (rail_meta) and on the mission's timeout; 0 is the profile's length.
    `rail_battery`: the campaign's opt-in to the battery cell
    (RAIL_BATTERY_OPT_IN)."""
    why = rail_refusal(spec, "hil", episode=ep, defaults=defaults,
                       rail_battery=rail_battery)
    if why:
        raise UnsupportedScenario(why)
    pol = spec.policy.load(ROOT)
    tag = hil_run_id(campaign, k, spec)
    nonce = uuid.uuid4().hex
    home = sitl_home(spec, pol, defaults, seed)
    meta = rail_meta(spec, ep, seed, "hil", tag, nonce, home, campaign, defaults,
                     max_s_add=max_s_add)
    events, meta_rel = write_campaign_files(spec, meta)
    args = rail_node_args(spec, seed, defaults, backend="hil", events=events, meta=meta_rel,
                          bundles_dir=cfg.bundles_dir, max_s_add=max_s_add, episode=ep)
    if extra_args:
        args = args + rail_extra_args(extra_args)
    plan = rail_stress_plan(spec, seed, defaults, run_id=tag, nonce=nonce)
    stress_paths = write_stress_plan(plan) if plan is not None else None
    timeout = float(meta["rail_max_s"]) + cfg.mission_margin_s
    try:
        if stress_paths is None:
            transport.restart_sitl(home)
        else:
            transport.restart_sitl(home, stress=stress_paths)
        transport.push(ROOT / CAMPAIGN_DIR / tag, f"{CAMPAIGN_DIR}/{tag}")
        if cfg.restart_mavros:
            transport.restart_mavros()
        rc, out = transport.run_mission(args, tag, "on" if spec.shield else "off", timeout)
    finally:
        transport.stop_sitl()
    try:
        run_dir = transport.fetch(tag, store)
    except Exception as e:                                   # noqa: BLE001
        raise RuntimeError(f"mission {tag} exited {rc} and its run folder could not be "
                           f"fetched ({e}): {out[-400:]}") from e
    (run_dir / "mission_console.log").write_text(out, encoding="utf-8", newline="\n")
    try:
        rows, metrics, _ = accept_hil_run(run_dir, nonce)
    except RuntimeError as e:
        raise RuntimeError(f"{e} (mission exited {rc}: {out[-400:]})") from e
    record = read_stress_record(stress_paths[1]) if stress_paths else None
    if record is not None:
        (run_dir / "sitl_stress.json").write_text(json.dumps(record, indent=1) + "\n",
                                                  encoding="utf-8", newline="\n")
    stress_problems, stress = rail_stress_check(spec, plan, record, metrics, rows, seed)
    problems = hil_flight_problems(metrics) + hil_chain_problems(metrics, cfg.chain_required())
    if stress_problems:
        problems.append("the cell's stress was not realised: " + "; ".join(stress_problems))
    gens = run_generations(run_dir)
    not_the_arm = (prefix_flight_problems(metrics, meta)
                   + prefix_generation_problems(meta, gens, rows))
    if not_the_arm:
        # Fail closed: a mission that did not fly the arm it is named for is
        # an error, never a scored episode of that arm.
        problems.append("not the prefix arm's flight: " + "; ".join(not_the_arm))
    arc_problems = arc_forecast_problems(metrics, extra_args)
    if arc_problems:
        # Fail closed (--rail-arc-forecast): never scored under the wrong arm.
        problems.append("not the arc arm's flight: " + "; ".join(arc_problems))
    ground_problems = ground_flight_problems(spec, metrics)
    if ground_problems:
        # Fail closed (X-11): never scored over a ground it did not fly.
        problems.append("not flown over the scenario's ground: " + "; ".join(ground_problems))
    if problems:
        raise RuntimeError(f"mission {tag}: " + "; ".join(problems))
    # Only now, the run accepted (its nonce, its mission_end, its stress and
    # chain): this launch's desktop joins that run's stack record. A folder
    # rejected above - a stale one from a reused tag, a wrong nonce - keeps
    # its record as fetched, never joined to this launch's desktop.
    record_desktop_revision(run_dir, cfg)
    extra = sitl_extra(spec, rows, metrics, pol, gens, max_s=meta["max_s"],
                       dt=spec.dt or _as_defaults(defaults).dt,
                       autopilot_texts=run_autopilot_texts(run_dir))
    extra["sim_stressors"] = stress["entries"]
    extra["sim_stressors_check"] = stress
    if meta.get("arm_record"):
        extra["prefix_consumer"] = prefix_flight_record(metrics, meta)
    prios = rail_rule_priorities(pol, gens)
    # `rows`: the scenario's own window, which score() reads (rail_scored_rows);
    # `rows_all`: the whole flight, past the cap to the fail-safe's outcome.
    return {"rows": rail_scored_rows(rows, meta["max_s"]), "rows_all": rows, "extra": extra,
            "policy": pol, "prios": prios, "run_dir": run_dir, "tag": tag, "rc": rc,
            "meta": meta, "metrics": metrics}


def write_scenario_score(run_dir: Path, meta: dict, res: dict, extra: dict,
                         status: str, bad: list, labels_failed: list,
                         known_why: str | None, prios: dict, spec: ScenarioSpec, *,
                         gates_basis: str = "ideal", gates: dict | None = None,
                         derivation: dict | None = None,
                         other_verdicts: dict | None = None) -> Path:
    """scenario_score.json beside a rail run: the harness's verdict and the
    KPI table it scored, with the metrics it gave compute() (the goal, the
    self-caused breaches, the arm, the FSM), so tools/kpi_report.py re-scores
    the run with them (load_run) under the scenario's family row. Since
    2026-11-07 also what it was read over (`rail_window`, which kpi_report
    applies too: performance over the scenario's window, safety invariants
    over the whole flight, `in_window` beside them), what the autopilot flew
    after the mission's end (`after_mission_end`), what the log could not
    answer (`rail_not_measured`), and the gates the verdict used
    (`gates_basis`, `gates`, with the derivation of proposed rail bounds and
    the verdict on the cell's own beside it)."""
    doc = score_document(meta, res, extra, status, bad, labels_failed, known_why, prios,
                         spec, gates_basis=gates_basis, gates=gates, derivation=derivation,
                         other_verdicts=other_verdicts)
    p = run_dir / SCENARIO_SCORE
    p.write_text(json.dumps(_json_safe(doc), indent=1, allow_nan=False) + "\n",
                 encoding="utf-8", newline="\n")
    return p


def score_document(meta: dict, res: dict, extra: dict, status: str, bad: list,
                   labels_failed: list, known_why: str | None, prios: dict,
                   spec: ScenarioSpec, *, gates_basis: str = "ideal",
                   gates: dict | None = None, derivation: dict | None = None,
                   other_verdicts: dict | None = None) -> dict:
    """The document write_scenario_score writes (and the offline re-score,
    rescore_rail_run, returns without writing anything beside the run)."""
    doc = {"schema": "rail-scenario-score/1", "scenario": meta,
           "run_id": meta.get("run_id"), "run_nonce": meta.get("run_nonce"),
           "metrics": {"reached_goal": extra.get("reached_goal"),
                       "breaches": extra.get("breaches"),
                       "shield": "on" if spec.shield else "off",
                       "fsm": extra.get("fsm")},
           "status": status, "gates_failed": bad, "labels_failed": labels_failed,
           "known_failure_reason": known_why, "rule_priorities": prios,
           # The simulator's stressors as realised (rail_stress_check, since
           # 2026-11-07): every SIM_* parameter in the pymavlink rail's record
           # (param, set, read_back, applied) with when the Shield node saw it,
           # and the check behind them (the plan, the simulator side's
           # record, the dropout's onset, the jittered home).
           "sim_stressors": extra.get("sim_stressors"),
           "sim_stressors_check": extra.get("sim_stressors_check"),
           # What the KPI table was read over (2026-11-07): the performance
           # measures over the scenario's window (tools/kpi_report.py reads
           # the same rows), the safety invariants over the whole flight -
           # their values in the window kept beside them (`in_window`) - what
           # the autopilot flew after the mission's end, and what the log
           # could not answer.
           "rail_window": extra.get("rail_window"),
           "in_window": extra.get("in_window"),
           "after_mission_end": extra.get("after_mission_end"),
           "rail_not_measured": {**({"positions": extra["rail_measurements_not_measured"]}
                                    if extra.get("rail_measurements_not_measured") else {}),
                                 **({"autopilot_departures": extra["autopilot_scope_basis"]}
                                    if extra.get("autopilot_scope_basis") else {}),
                                 **(extra.get("rail_not_measured") or {})} or None,
           # The gates the verdict used (rail_gate_set): the cell's own
           # ("ideal") or the proposal's bounds derived on a vehicle model
           # ("rail", "ardupilot", "ardupilot-rest"), with the safety
           # invariants' autopilot counterparts, how each bound was derived,
           # and the verdict on the cell's own beside it.
           "gates_basis": gates_basis,
           "gates": gates if gates is not None else spec.gates,
           "gates_derivation": derivation or None,
           "other_verdicts": other_verdicts or None,
           "kpi": res,
           "scored_by": "experiments/sweep_scenarios.py (score, episode_verdict)"}
    if extra.get("prefix_consumer") is not None:
        # A prefix-consumer mission only: what its VLA node announced of the
        # arm beside what the harness expected (prefix_flight_record).
        doc["prefix_consumer"] = extra["prefix_consumer"]
    return doc


def verdict_record(status: str, bad: list, labels_failed: list,
                   known_why: str | None) -> dict:
    """One verdict as score documents keep it beside another basis's."""
    return {"status": status, "gates_failed": bad, "labels_failed": labels_failed,
            "known_failure_reason": known_why}


def _episode_for_run(meta: dict, episodes) -> Any:
    """The episode of `episodes` a rail run flew, by its scenario record:
    the cell, the seed, the Shield arm and the text arm (`arm_tag`: "" for a
    family with none, "canonical" / "canonical,prefix-on" for the one a rail
    flies of a paraphrase or prefix family). None when the expansion holds
    no such episode (another profile's, or a library that changed since)."""
    want = (meta.get("cell"), meta.get("seed"), meta.get("arm"))
    for ep in episodes:
        if ((ep.episode_id, ep.seed, "on" if ep.cell.spec.shield else "off") == want
                and ep.arm_tag == meta.get("arm_tag", ep.arm_tag)
                and ep.forecast == "grant" and ep.response == "ideal"):
            return ep
    return None


def rescore_rail_run(run_dir: Path, ep, defaults, *, rail_gates: str = "ideal") -> dict:
    """A fetched rail run read again with this checkout's scorer (the
    campaign's own path: sitl_extra over the scenario's window, score(),
    episode_verdict), nothing written beside it. Returns {run, scenario_id,
    cell, seed, before (its scenario_score.json's verdict, None without one),
    after ({"ideal": verdict} and, with rail_gates other than "ideal", the
    verdict on every derived basis of RAIL_GATE_BASES), doc (the score
    document it would write, its verdict on `rail_gates`)}."""
    run_dir = Path(run_dir)
    spec = scoring_spec(ep.cell.spec, ep)
    metrics = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))
    meta = metrics.get("scenario") if isinstance(metrics.get("scenario"), dict) else {}
    rows = [json.loads(x) for x in (run_dir / "flight_log.jsonl").read_text(
        encoding="utf-8").splitlines() if x.strip()]
    before = stored = None
    if (run_dir / SCENARIO_SCORE).is_file():
        stored = json.loads((run_dir / SCENARIO_SCORE).read_text(encoding="utf-8"))
        before = verdict_record(stored.get("status"), stored.get("gates_failed") or [],
                                stored.get("labels_failed") or [],
                                stored.get("known_failure_reason"))
    ground_problems = ground_flight_problems(spec, metrics)
    if ground_problems:
        # As the campaign accepted it (run_sitl / run_hil): fail closed.
        raise RuntimeError("not flown over the scenario's ground: "
                           + "; ".join(ground_problems))
    pol = spec.policy.load(ROOT)
    gens = run_generations(run_dir)
    max_s = meta.get("max_s")
    extra = sitl_extra(spec, rows, metrics, pol, gens, max_s=max_s,
                       dt=spec.dt or _as_defaults(defaults).dt,
                       autopilot_texts=run_autopilot_texts(run_dir))
    if stored is not None:
        # Checked when the run was accepted (rail_stress_check); kept as found.
        extra["sim_stressors"] = stored.get("sim_stressors")
        extra["sim_stressors_check"] = stored.get("sim_stressors_check")
    prios = rail_rule_priorities(pol, gens)
    res = score(spec, rail_scored_rows(rows, extra["rail_window"]["max_s"]), extra, pol, prios)
    derived: dict[str, tuple] = {"ideal": rail_gate_set("ideal", spec, ep.seed, defaults)}
    if rail_gates != "ideal":
        for basis in RAIL_GATE_BASES[1:]:
            derived[basis] = rail_gate_set(basis, spec, ep.seed, defaults)
    after = {basis: verdict_record(*episode_verdict(spec, res, ep.cell.params, ep.seed,
                                                    gates=g))
             for basis, (g, _) in derived.items()}
    gates, derivation = derived[rail_gates]
    v = after[rail_gates]
    doc = score_document(meta, res, extra, v["status"], v["gates_failed"], v["labels_failed"],
                         v["known_failure_reason"], prios, spec, gates_basis=rail_gates,
                         gates=gates, derivation=derivation,
                         other_verdicts={k: w for k, w in after.items() if k != rail_gates})
    doc["derivations"] = {k: d for k, (_, d) in derived.items() if d} or None
    return {"run": run_dir.name, "scenario_id": spec.scenario_id, "cell": ep.episode_id,
            "seed": ep.seed, "before": before, "after": after, "doc": doc}


# The fail-safe label artefact (tools/failsafe_rubric.py --build-labels): the
# rubric's labels of every cell and seed on the ideal and the rail vehicle,
# under both readings of E5 (docs/RUBRIC-failsafe.md sections 3, 7 and 8).
RAIL_LABELS = ROOT / "docs" / "data" / "failsafe_labels_rail.json"
RAIL_LABELS_SCHEMA = "failsafe-labels/1"
# The readings a rail campaign's fail-safe correctness is printed under, in
# this order: the adopted KPI reading last, the earlier ones beside it.
RAIL_LABEL_READINGS = (
    ("a", "ideal-vehicle labels, E5 on the flight without a fail-safe (section 3)"),
    ("r", "rail-vehicle labels, E5 on the flight without a fail-safe (sections 3, 8)"),
    ("r2", "rail-vehicle labels, E5 on the prescribed flight (sections 7, 8)"),
)


def load_rail_labels(path: Path | None = None) -> dict:
    """The label artefact as built, refused when it is not one."""
    path = Path(path) if path is not None else RAIL_LABELS
    doc = json.loads(path.read_text(encoding="utf-8"))
    if doc.get("schema") != RAIL_LABELS_SCHEMA or not isinstance(doc.get("labels"), dict):
        raise ValueError(f"{path}: not a {RAIL_LABELS_SCHEMA} label artefact")
    return doc


def rail_label_scores(runs: list[dict], labels: dict) -> dict:
    """Fail-safe trigger correctness of fetched rail runs under each reading
    of the labels (RAIL_LABEL_READINGS: (a), (r) and the adopted (r2)).

    `runs`: one mapping per Shield-on run with `run`, `cell`, `seed`,
    `triggered` (the re-scored failsafe_triggered), `censored` (it ended
    in Loiter before the operator timeout: not measured) and, optionally,
    `accepted` (False: the campaign did not accept the flight - its
    scenario_score.json is missing because a check of the flight failed,
    e.g. its stress was not realised - so it is not the cell's flight and
    is left out, by name, as the campaign's own count leaves it out). Per
    reading: the
    count and rate over the runs with a verdict AND a decided label, the
    Wilson 95 % lower bound and both nulls (guardrail.fsm.
    score_failsafe_triggers), and BY NAME the runs left out - censored,
    undecided (the label could not be decided in the scenario's length),
    unlabelled (the cell and seed are not in the artefact), not measured (no
    FSM verdict), not accepted - and the false and missed triggers. Nothing is pooled
    across readings and nothing left out is counted as correct.

    A rail flies 35 s longer than the scenario (rail_max_s) and its
    fail-safe is read over that whole flight, so the artefact reads a rail
    label that is undecided at the scenario's length on the cell flown that
    much longer (`length` "rail"). Runs scored on such a label are named
    (`label_flown_longer`) and the count without them is kept beside the
    reading's (`at_scenario_length`), so what the longer reading adds is
    never hidden in the total.

    A LENGTHENED RUN (2026-11-14; a run's `max_s_add` > 0, --hil-max-s-add)
    flew past the rail's own cap. Its cell and seed keep their label - a
    label is the cell's, decided at the scenario's length or the rail's, and
    what is undecided there stays undecided, never read off the longer
    flight - and its trigger is the FSM's over its whole flight. Such runs
    are named (`run_flown_longer`) and the count without them is kept beside
    (`at_profile_length`); --rescore-rail never mixes them with runs at the
    profile's length in one table, so a lengthened table is all of them. A
    run still censored after flying longer is named censored, as any other,
    with its `cause` (censored_cause) and its `max_s_add` when given.

    THE GEOFENCE BACKSTOP (2026-11-15, BACKSTOP_RULE). A run's `by`
    (score()'s `failsafe_by`) "autopilot_geofence" says its fail-safe was
    triggered by the autopilot's GeoFence while the FSM held; `triggered` is
    then True and `censored` False, and it is scored against its label like
    any other trigger. Under every reading such runs are named (`backstop`,
    each with its takeover and its verdict), the scored triggers are counted
    by who made them (`triggered_by`), and the two earlier ways of reading
    them are kept beside the figure: `before_backstop_rule` (the reading
    before 2026-11-15: left out as censored where the FSM held in Loiter with
    its timeout pending - `backstop.fsm_censored` - else scored as not
    triggered) and `backstop_as_not_triggered` (every one of them scored as
    no fail-safe: a missed trigger where one is expected)."""
    from guardrail.fsm import score_failsafe_triggers

    def figure(prs: list, **more) -> dict:
        s = score_failsafe_triggers(prs, GRANT_FAILSAFE_TARGET)
        return {"scored": s["scored"],
                "correct": s["true_triggers"] + s["true_non_triggers"],
                "failsafe_trigger_correctness": s["failsafe_trigger_correctness"],
                "wilson_low_95": s.get("wilson_low_95"),
                "false_triggers": s["false_triggers"],
                "missed_triggers": s["missed_triggers"], **more}
    adopted = labels.get("adopted_kpi_reading")
    out = {}
    for key, what in RAIL_LABEL_READINGS:
        pairs, short, at_profile = [], [], []
        before, as_missed, by_who = [], [], {FAILSAFE_BY_FSM: 0, FAILSAFE_BY_GEOFENCE: 0}
        names = {k: [] for k in ("false", "missed", "censored", "undecided", "unlabelled",
                                 "not_measured", "not_accepted", "label_flown_longer",
                                 "run_flown_longer", "backstop")}
        for it in runs:
            tag = {"run": it["run"], "cell": it["cell"], "seed": it["seed"]}
            add = float(it.get("max_s_add") or 0.0)
            lab = ((labels["labels"].get(it["cell"]) or {}).get(str(it["seed"])) or {}).get(key)
            if it.get("accepted") is False:
                names["not_accepted"].append(tag)
            elif it.get("triggered") is None:
                names["not_measured"].append(tag)
            elif it.get("censored"):
                names["censored"].append({**tag, **({"cause": it["cause"]} if it.get("cause")
                                                    else {}),
                                          **({"max_s_add": add} if add else {})})
            elif lab is None:
                names["unlabelled"].append(tag)
            elif lab.get("expected") is None:
                names["undecided"].append(tag)
            else:
                exp, trig = bool(lab["expected"]), bool(it["triggered"])
                pairs.append((exp, trig))
                if trig and it.get("by") == FAILSAFE_BY_GEOFENCE:
                    # Triggered by the backstop (2026-11-15): scored above
                    # like any trigger, named, and read the two earlier ways
                    # beside.
                    b = it.get("backstop") or {}
                    by_who[FAILSAFE_BY_GEOFENCE] += 1
                    names["backstop"].append({
                        **tag, "mode": b.get("mode"), "t": b.get("t"),
                        "fsm_state": b.get("fsm_state"),
                        "fsm_censored": bool(b.get("fsm_censored")),
                        "expected": exp, "verdict": "correct" if exp else "false"})
                    as_missed.append((exp, False))
                    if not b.get("fsm_censored"):
                        before.append((exp, False))
                else:
                    by_who[FAILSAFE_BY_FSM] += int(trig)
                    before.append((exp, trig))
                    as_missed.append((exp, trig))
                if lab.get("length") == "rail":
                    names["label_flown_longer"].append(tag)
                else:
                    short.append((exp, trig))
                if add:
                    names["run_flown_longer"].append({**tag, "max_s_add": add})
                else:
                    at_profile.append((exp, trig))
                if trig and not exp:
                    names["false"].append({**tag, "label": lab.get("clauses") or []})
                elif exp and not trig:
                    names["missed"].append({**tag, "label": lab.get("clauses") or []})
        sc = score_failsafe_triggers(pairs, GRANT_FAILSAFE_TARGET)
        out[key] = {"reading": what, "adopted_kpi_reading": key == adopted,
                    "runs": len(runs), "scored": sc["scored"],
                    "correct": sc["true_triggers"] + sc["true_non_triggers"],
                    "failsafe_trigger_correctness": sc["failsafe_trigger_correctness"],
                    "wilson_low_95": sc.get("wilson_low_95"),
                    "null_always_trigger": sc["null_always_trigger"],
                    "null_never_trigger": sc["null_never_trigger"],
                    "expected_triggers": sc["expected_triggers"],
                    "expected_non_triggers": sc["expected_non_triggers"],
                    "at_scenario_length": {
                        "scored": len(short), "correct": sum(1 for e, t in short if e == t)},
                    "at_profile_length": {
                        "scored": len(at_profile),
                        "correct": sum(1 for e, t in at_profile if e == t)},
                    # Who triggered the scored fail-safes, and the figure with
                    # the backstop's read the two earlier ways (2026-11-15).
                    "triggered_by": by_who,
                    "before_backstop_rule": figure(
                        before, left_out_as_censored=sum(
                            1 for t in names["backstop"] if t["fsm_censored"])),
                    "backstop_as_not_triggered": figure(as_missed),
                    **{k: v for k, v in names.items()}}
    return out


def rail_label_lines(scores: dict, labels: dict, path: Path) -> list[str]:
    """The printed form of rail_label_scores: one block per reading."""
    built = labels.get("built") or {}
    n = next(iter(scores.values()))["runs"] if scores else 0
    revs = ", ".join(str(r)[:12] for r in built.get("code_revisions") or ["?"])
    L = [f"fail-safe trigger correctness of {n} Shield-on run(s), by label reading "
         f"({_show(path)}, derived at {revs}):"]

    def f(x):
        return "n/m" if x is None else f"{x:.3f}"
    for key, sc in scores.items():
        mark = "  <- THE ADOPTED KPI READING" if sc["adopted_kpi_reading"] else ""
        L.append(f"  ({key}) {sc['reading']}{mark}")
        L.append(f"      {sc['correct']}/{sc['scored']} = {f(sc['failsafe_trigger_correctness'])}"
                 f"  Wilson 95 % low {f(sc['wilson_low_95'])}  nulls: always-trigger "
                 f"{f(sc['null_always_trigger'])}, never-trigger {f(sc['null_never_trigger'])}"
                 f"  | censored {len(sc['censored'])}, undecided {len(sc['undecided'])}, "
                 f"unlabelled {len(sc['unlabelled'])}, no verdict {len(sc['not_measured'])}, "
                 f"not accepted {len(sc['not_accepted'])}")
        bs = sc.get("backstop")
        if bs is not None:
            # The GeoFence backstop (2026-11-15, BACKSTOP_RULE): its count
            # beside the figure, always, and the two earlier readings of
            # those runs when there are any.
            nb = len(bs)
            L.append(f"      {nb} of the scored fail-safes triggered by the autopilot's "
                     f"GeoFence backstop while the FSM held (failsafe_by "
                     f"{FAILSAFE_BY_GEOFENCE}; counted since {BACKSTOP_RULE['since']})"
                     + (f": {sum(1 for t in bs if t['verdict'] == 'correct')} correct, "
                        f"{sum(1 for t in bs if t['verdict'] == 'false')} false" if nb
                        else ": the reading before that rule gives the same figure"))
            if nb:
                b0, b1 = sc["before_backstop_rule"], sc["backstop_as_not_triggered"]
                L.append(f"      the same runs read as before {BACKSTOP_RULE['since']} "
                         f"({b0['left_out_as_censored']} left out as censored): "
                         f"{b0['correct']}/{b0['scored']} = "
                         f"{f(b0['failsafe_trigger_correctness'])}  Wilson 95 % low "
                         f"{f(b0['wilson_low_95'])}; counted as not triggered: "
                         f"{b1['correct']}/{b1['scored']} = "
                         f"{f(b1['failsafe_trigger_correctness'])}  Wilson 95 % low "
                         f"{f(b1['wilson_low_95'])}")
        if sc["label_flown_longer"]:
            s0 = sc["at_scenario_length"]
            L.append(f"      {len(sc['label_flown_longer'])} of them on a label undecided at the "
                     f"scenario's length, read on the cell flown as long as the rail flies; "
                     f"without them {s0['correct']}/{s0['scored']}")
        if sc.get("run_flown_longer"):
            s1 = sc.get("at_profile_length") or {}
            adds = sorted({t["max_s_add"] for t in sc["run_flown_longer"]})
            L.append(f"      {len(sc['run_flown_longer'])} of them FLOWN LONGER than the profile "
                     f"(+{', +'.join(f'{a:g}' for a in adds)} s past the rail's cap, "
                     f"--hil-max-s-add): a lengthened campaign, reported beside the KPI "
                     f"campaign, never merged into it; without them "
                     f"{s1.get('correct')}/{s1.get('scored')}")
        for kind in ("false", "missed", "censored", "undecided", "unlabelled",
                     "not_measured", "not_accepted", "label_flown_longer", "backstop"):
            for t in sc.get(kind) or []:
                name = {"label_flown_longer": "longer", "not_measured": "no verdict",
                        "not_accepted": "not acc."}.get(kind, kind)
                if kind == "backstop":
                    L.append(f"      {name:10} {t['run']}  ({t['cell']}, seed {t['seed']})"
                             f"  - the autopilot's GeoFence took it ({t['mode']} at "
                             f"{t['t']} s) with the FSM in {t['fsm_state']}: "
                             + ("a fail-safe expected, correct" if t["expected"] else
                                "no fail-safe expected, a false trigger"))
                    continue
                why = ("" if kind != "censored" or not t.get("cause") else
                       "  - the autopilot took the aircraft; flying longer will not decide it"
                       if t["cause"] == "autopilot_takeover" else
                       "  - ended still holding" + (f" after +{t['max_s_add']:g} s"
                                                    if t.get("max_s_add") else ""))
                L.append(f"      {name:10} {t['run']}  ({t['cell']}, seed {t['seed']}){why}")
    return L


def rescore_rail_main(globs: list[str], episodes, defaults, rail_gates: str,
                      out: Path, labels_path: Path | None = None) -> int:
    """`--rescore-rail`: every run folder the globs name (relative ones from
    the repository root) re-scored by rescore_rail_run, one line each, the
    whole table to `out`. Exit 0 when every run was re-scored, 1 when one
    could not be (no scenario record, or a cell this expansion lacks).

    Since 2026-11-10 the runs' fail-safe trigger correctness is printed, and
    written under `failsafe_labels`, by label reading (rail_label_scores):
    (a), (r) and (r2), the adopted KPI reading, from the label artefact
    (`labels_path`, default RAIL_LABELS). Without the artefact that table is
    not printed and the exit code is 1: a campaign scored on no labels is
    not a score. A run the campaign did not accept (no scenario_score.json:
    `before` None) is re-scored and printed like the others, but left out
    of the label table by name (`not_accepted`).

    ONE FLIGHT LENGTH PER TABLE (2026-11-14): runs of a lengthened hil
    campaign (--hil-max-s-add; their scenario record's `max_s_add`) and runs
    at the profile's length - or two lengthenings - are refused together
    (exit 2, nothing written): a table's figure is one campaign's flight.
    The table says which length it is (`flight_length`)."""
    runs: list[Path] = []
    for g in globs:
        pat = g if Path(g).is_absolute() else str(ROOT / g)
        runs += [Path(p) for p in sorted(glob.glob(pat)) if Path(p).is_dir()]
    lengths: dict[float, list[str]] = {}
    for run_dir in runs:
        try:
            m0 = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue                    # reported by the re-score below
        sc0 = m0.get("scenario") if isinstance(m0, dict) else None
        if isinstance(sc0, dict):
            lengths.setdefault(flight_max_s_add(sc0), []).append(run_dir.name)
    if len(lengths) > 1:
        def said(a):
            return f"+{a:g} s past the rail cap" if a else "the profile's own length"
        print("REFUSED: these runs were flown at " + str(len(lengths)) + " flight lengths ("
              + "; ".join(f"{said(a)}: {len(v)} run(s), e.g. {v[0]}"
                          for a, v in sorted(lengths.items()))
              + "). A lengthened campaign (--hil-max-s-add) is re-scored on its own, never "
                "in one table with the profile's flights: give each campaign's runs "
                "separately. Nothing was written.")
        return 2
    table_add = next(iter(lengths), 0.0)
    table, problems = [], 0
    for run_dir in runs:
        try:
            m = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))
            meta = m.get("scenario") if isinstance(m.get("scenario"), dict) else None
            if meta is None:
                raise ValueError("no scenario record in metrics.json")
            ep = _episode_for_run(meta, episodes)
            if ep is None:
                raise ValueError(f"cell {meta.get('cell')!r} seed {meta.get('seed')} is not "
                                 f"in this expansion (give the campaign's --profile)")
            item = rescore_rail_run(run_dir, ep, defaults, rail_gates=rail_gates)
        except Exception as e:                               # noqa: BLE001
            problems += 1
            table.append({"run": run_dir.name, "error": f"{type(e).__name__}: {e}"})
            print(f"  ERROR {run_dir.name}: {type(e).__name__}: {e}")
            continue
        table.append(item)
        b = item["before"] or {}
        cols = [f"before={b.get('status')}({len(b.get('gates_failed') or [])})"]
        cols += [f"{k}={v['status']}({len(v['gates_failed'])})" for k, v in item["after"].items()]
        print(f"  {run_dir.name:<44} " + "  ".join(cols))
    doc = {"schema": "rail-rescore/1", "rail_gates": rail_gates, "runs": table,
           # The flight length of every run in it (one per table, above).
           "flight_length": {
               "max_s_add": table_add,
               "lengthened": bool(table_add),
               "what": (f"a LENGTHENED campaign: every run flown {table_add:g} s past the "
                        f"rail's cap (--hil-max-s-add); reported beside the KPI campaign, "
                        f"never merged into it" if table_add else
                        "the profile's own length: every run flown to the rail's cap "
                        "(the scenario's max_s + the Loiter operator timeout + 5 s)")}}
    labels_path = Path(labels_path) if labels_path is not None else RAIL_LABELS
    no_labels = False
    try:
        labels = load_rail_labels(labels_path)
    except (OSError, ValueError) as e:
        no_labels = True
        print(f"\nNO LABEL TABLE: {type(e).__name__}: {e}")
    else:
        scored = [{"run": it["run"], "cell": it["cell"], "seed": it["seed"],
                   "triggered": (it["doc"].get("kpi") or {}).get("failsafe_triggered"),
                   "censored": bool((it["doc"].get("kpi") or {}).get("failsafe_censored")),
                   "cause": censored_cause(it["doc"].get("kpi") or {}),
                   # Who triggered it, and the takeover behind a backstop
                   # trigger (2026-11-15, BACKSTOP_RULE).
                   "by": (it["doc"].get("kpi") or {}).get("failsafe_by"),
                   "backstop": (it["doc"].get("kpi") or {}).get("failsafe_backstop"),
                   "max_s_add": flight_max_s_add(it["doc"].get("scenario")),
                   "accepted": it["before"] is not None}
                  for it in table
                  if "error" not in it and (it["doc"].get("metrics") or {}).get("shield") == "on"
                  # The prefix-on arm of a consuming pilot carries no label
                  # (scoring_spec): the artefact's are its cell's own pilot's.
                  and not (it["doc"].get("scenario") or {}).get("labels_not_applied")]
        scores = rail_label_scores(scored, labels)
        doc["failsafe_labels"] = {
            "artefact": _show(labels_path), "built": labels.get("built"),
            "adopted_kpi_reading": labels.get("adopted_kpi_reading"),
            # Scored under the GeoFence backstop rule (2026-11-15): a table
            # without this was written before it.
            "backstop_rule": dict(BACKSTOP_RULE),
            "shield_off_runs_left_out": sum(
                1 for it in table if "error" not in it
                and (it["doc"].get("metrics") or {}).get("shield") != "on"),
            "readings": scores}
        print()
        for line in rail_label_lines(scores, labels, labels_path):
            print(line)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(_json_safe(doc), indent=1) + "\n",
                   encoding="utf-8", newline="\n")
    print(f"\n{len(table) - problems}/{len(table)} run(s) re-scored -> {_show(out)}")
    if table_add:
        print(f"FLIGHT LENGTH: {doc['flight_length']['what']}")
    return 1 if problems or not table or no_labels else 0


def hil_plan(episodes, defaults, cfg: HilConfig, campaign: str,
             controls: bool = False, max_s_add: float = 0.0,
             rail_battery: bool = False) -> list[dict]:
    """The hil campaign before anything flies: per episode, the mission it
    would be (its tag, SITL_HOME and the arguments both nodes get) or why
    it is refused. The dry run prints this and stops. `max_s_add`: a
    lengthened campaign's seconds past the rail cap (--hil-max-s-add), on
    every mission's --max-s (`max_s`: the scenario's own cap, `rail_max_s`:
    the cap flown)."""
    plan = []
    k = 0
    for ep in episodes:
        spec = ep.cell.spec
        item = {"id": ep.run_id, "seed": ep.seed, "template": spec.template,
                "cell": ep.episode_id}
        why = rail_refusal(spec, "hil", episode=ep, defaults=defaults,
                           rail_battery=rail_battery)
        if why is None and not spec.shield and not controls:
            why = ("a Shield-off control arm: flown only with --hil-controls (it flies "
                   "the raw pilot into the zones, and the GeoFence backstop may take "
                   "the aircraft)")
        if why is None:
            try:
                pol = spec.policy.load(ROOT)
                k += 1
                tag = hil_run_id(campaign, k, spec)
                rel = f"{CAMPAIGN_DIR}/{tag}"
                args = rail_node_args(spec, ep.seed, defaults, backend="hil",
                                      events=(f"{rel}/events.json" if spec.events else None),
                                      meta=f"{rel}/scenario.json",
                                      bundles_dir=cfg.bundles_dir, max_s_add=max_s_add,
                                      episode=ep)
                stress = rail_stress_plan(spec, ep.seed, defaults, run_id=tag)
                jx, jy = start_jitter(spec, ep.seed)
                dfl = _as_defaults(defaults)
                cap = (spec.ticks or dfl.ticks) * (spec.dt or dfl.dt)     # rail_mission's
                item.update(fly=True, tag=tag,
                            sitl_home=sitl_home(spec, pol, defaults, ep.seed),
                            max_s=cap, rail_max_s=rail_max_s(cap, max_s_add),
                            shield="on" if spec.shield else "off", args=args,
                            stress=None if stress is None else {
                                "boot": stress["boot"], "schedule": stress["schedule"],
                                "firmware": stress.get("firmware") or [],
                                "plan": f"{RAIL_STRESS_DIR}/{tag}/plan.json"},
                            start_jitter_m=[jx, jy] if (jx or jy) else None)
            except UnsupportedScenario as e:
                k -= 1
                why = str(e)
        if why is not None:
            item.update(fly=False, refused=why)
        plan.append(item)
    return plan


def hil_config_record(cfg: HilConfig) -> dict:
    """The rig a campaign flew on, for the results JSON."""
    from dataclasses import asdict
    d = asdict(cfg)
    d["ssh_options"] = list(d["ssh_options"])
    # What the missions were launched with (`require_chain` may be None and
    # this desktop's VLAGUARD_REQUIRE_CHAIN decide): None = the topology file's.
    d["require_chain_launched"] = cfg.chain_required()
    return d


def print_hil_plan(plan: list[dict], cfg: HilConfig, max_s_add: float = 0.0) -> None:
    """The dry run's listing: every mission with its SITL_HOME and the exact
    commands each step runs, every refusal with its reason, and the counts;
    a lengthened campaign (`max_s_add`, --hil-max-s-add) says so first and
    gives each mission's cap beside the scenario's."""
    fly = [p for p in plan if p["fly"]]
    print(f"hil campaign plan: {len(plan)} episode(s), {len(fly)} mission(s) to fly, "
          f"{len(plan) - len(fly)} refused (nothing flown, nothing written)\n")
    if max_s_add:
        print(f"  LENGTHENED CAMPAIGN (--hil-max-s-add {max_s_add:g}): every mission flies "
              f"{max_s_add:g} s past the rail's cap. Not the profile's flight: a separate "
              f"campaign, reported beside the KPI campaign and never merged into it\n")
    tr = SubprocessTransport(cfg)
    helper = tr._helper()
    for p in plan:
        if not p["fly"]:
            continue
        print(f"  FLY  {p['tag']}  {p['id']}  seed={p['seed']}"
              + (f"  (cap {p['rail_max_s']:g} s: the scenario's {p['max_s']:g} s, the "
                 f"rail's {p['rail_max_s'] - max_s_add:g} s, +{max_s_add:g} s)"
                 if max_s_add and p.get("rail_max_s") is not None else ""))
        print(f"       SITL_HOME  {p['sitl_home'] or 'start_sitl.sh default (CMAC)'}"
              + (f"  (start jitter {p['start_jitter_m'][0]:+.3f}, "
                 f"{p['start_jitter_m'][1]:+.3f} m)" if p.get("start_jitter_m") else ""))
        st = p.get("stress")
        sargs = []
        if st:
            if st.get("firmware"):
                print("       SITL setup " + ", ".join(f"{b['param']}={b['value']:g}"
                                                        for b in st["firmware"])
                      + " (the simulated vehicle's battery fail-safe; restored after)")
            print("       SITL boot  " + ", ".join(f"{b['param']}={b['value']:g}"
                                                    for b in st["boot"]))
            print("       SITL t=0+  " + ", ".join(f"{s['param']}={s['value']:g} at "
                                                    f"{s['t']:g} s" for s in st["schedule"]))
            rel = st["plan"]
            sargs = ["--stress-plan", tr._wsl(ROOT / rel),
                     "--stress-record", tr._wsl(ROOT / rel.replace("plan.json", "record.json"))]
        print(f"       desktop    {shlex.join(helper + ['start', '--mavros-host', cfg.mavros_host, '--settle-s', f'{cfg.sitl_settle_s:g}'] + (['--home', p['sitl_home']] if p['sitl_home'] else []) + sargs)}")
        print(f"       orin       cd {cfg.remote_root} && {cfg.compose()} run --rm -T "
              f"{cfg.mission_env()}mission "
              f"mission --shield {p['shield']} --tag {p['tag']} {shlex.join(p['args'])}")
    refused = [p for p in plan if not p["fly"]]
    if refused:
        print("\n  refused:")
        for p in refused:
            print(f"  REF  {p['id']}  seed={p['seed']}\n       {p['refused']}")
    by_t: dict[str, list[int]] = {}
    for p in plan:
        c = by_t.setdefault(p["template"], [0, 0])
        c[0 if p["fly"] else 1] += 1
    print(f"\n  {'template':<32} {'fly':>4} {'refused':>8}")
    for t, (f, r) in sorted(by_t.items()):
        print(f"  {t:<32} {f:>4} {r:>8}")
    print(f"\n  missions are flown one at a time: SITL restarted at each SITL_HOME in "
          f"WSL ({cfg.wsl_distro}), the scenario's files copied to "
          f"{cfg.ssh_target}:{cfg.remote_root}/{CAMPAIGN_DIR}/<tag>/, MAVROS restarted, "
          f"the mission run, its run folder fetched")


# --------------------------------------------------------------------------- #
# gates
# --------------------------------------------------------------------------- #

# A None mean time-to-safe is a measured "no unsafe episode at all" when the
# episode was measurable and nothing was censored - so a <= bound holds. A
# censored episode (never got out) is the opposite, and fails.
_VACUOUS = {"mean_time_to_safe_s", "max_time_to_safe_s"}


def check_gates(gates: dict, result: dict) -> list[str]:
    """Returns the failures. An absent or None field is a FAILURE, not a pass:
    a gate on something that was never measured has not been satisfied."""
    return [msg for _, msg in check_gates_keyed(gates, result)]


def check_gates_keyed(gates: dict, result: dict) -> list[tuple[str, str]]:
    """check_gates with each failure's gate field: [(field, message)]. The
    field is what a known-failure pin names (scenario_spec.pin_key), so a
    pinned cell can tell the failure it is pinned for from a new one."""
    bad: list[tuple[str, str]] = []
    for fld, rule in (gates or {}).items():
        got = result.get(fld)
        if got is None:
            if (fld in _VACUOUS and set(rule) == {"max"}
                    and result.get("time_to_safe_not_measurable") is False
                    and result.get("time_to_safe_episodes") == 0
                    and result.get("time_to_safe_censored") == 0):
                continue
            if fld in _VACUOUS and (result.get("time_to_safe_censored") or 0) > 0:
                bad.append((fld, f"{fld}: never recovered (censored episode)"))
                continue
            bad.append((fld, f"{fld}: not measured"))
            continue
        if "max" in rule and got > rule["max"]:
            bad.append((fld, f"{fld}={got} > max {rule['max']}"))
        if "min" in rule and got < rule["min"]:
            bad.append((fld, f"{fld}={got} < min {rule['min']}"))
        if "equals" in rule and got != rule["equals"]:
            bad.append((fld, f"{fld}={got} != {rule['equals']}"))
    return bad


# THE AUTOPILOT'S GEOFENCE AS THE FAIL-SAFE (2026-11-15; docs/RUBRIC-failsafe.md
# sections 1 and 9, decision 14). Decided 2026-11-15 by the lab under the PI's
# instruction of 6 Oct (follow the grant; best practice where it is silent);
# the PI's confirmation is still to be recorded. The grant names ArduPilot's
# fence as the ultimate backstop (Grant overview, the WP3 row), so a run in
# which the autopilot took the aircraft to RTL or LAND on a fence breach while
# the FSM was holding in Brake or Loiter IS a fail-safe that triggered - by the
# backstop, not by the FSM - and is scored against the scenario's label like
# any other trigger: correct where a fail-safe is expected, a false trigger
# where it is not. Until then such a run was censored (its Loiter frozen by
# the takeover with the operator timeout pending: 18 dyn-nfz-spawn-ahead runs
# of the KPI nightly ntFIN2), and that reading is still printed beside the
# adopted one wherever the KPI is. Derived per run from its own records
# (failsafe_backstop), never from a list of runs:
#   - metrics.json `autopilot_took_over`: the mode the autopilot entered that
#     the node had not asked for (BACKSTOP_MODES), and when;
#   - metrics.json `fsm`: the FSM's state when it stopped stepping at the
#     takeover (BACKSTOP_HOLD_STATES) and that it had triggered nothing itself;
#   - events.jsonl `autopilot_text`: ArduPilot's own "Fence Breached" within
#     BACKSTOP_BREACH_WINDOW_S of the mode change (MAVROS relays the mode with
#     the 1 Hz heartbeat and the text as it is sent, so either may come first;
#     on ntFIN2 the text followed the mode by 0.001-0.018 s).
# A takeover that is not all three - another mode, no fence breach on record,
# the FSM in Normal - is not counted, and stays what it was (censored where the
# FSM held with its timeout pending). How FSM-triggered fail-safes are scored
# is unchanged.
FAILSAFE_BY_FSM = "fsm"
FAILSAFE_BY_GEOFENCE = "autopilot_geofence"
BACKSTOP_MODES = ("RTL", "LAND")
BACKSTOP_HOLD_STATES = ("Brake", "Loiter")
BACKSTOP_BREACH_TEXT = "fence breach"        # in ArduPilot's STATUSTEXT, any case
BACKSTOP_BREACH_WINDOW_S = 2.0
# What a table or summary scored under the rule records, so a reader (and
# tools/kpi_report.py) can tell it from one written before the rule existed.
BACKSTOP_RULE = {
    "since": "2026-11-15",
    "rule": ("a run in which the autopilot took the aircraft to RTL or LAND on a fence "
             "breach while the FSM held in Brake or Loiter is a fail-safe triggered by "
             "the backstop (failsafe_by autopilot_geofence), scored against the label "
             "like any other trigger"),
    "before": ("such a run was left out as censored where the FSM held in Loiter with "
               "its operator timeout pending, and scored as not triggered where it held "
               "in Brake"),
    "decided": ("2026-11-15 by the lab under the PI's instruction of 6 Oct (follow the "
                "grant; best practice where it is silent); the PI's confirmation is "
                "still to be recorded"),
    "doc": "docs/RUBRIC-failsafe.md sections 1 and 9"}


def run_autopilot_texts(run_dir: Path) -> list[dict] | None:
    """The autopilot's STATUSTEXT messages a rail run recorded (events.jsonl,
    kind `autopilot_text`: t, severity, text), in the order written, of the
    file's LAST episode (from its last `episode_start`, as accept_hil_run
    reads it: a folder flown twice keeps the earlier episode's lines). None
    when the run folder has no readable events.jsonl - not read, which is not
    the same as none sent."""
    p = Path(run_dir) / "events.jsonl"
    try:
        lines = p.read_text(encoding="utf-8").splitlines()
    except OSError:
        return None
    out = []
    for x in lines:
        if not x.strip():
            continue
        try:
            e = json.loads(x)
        except ValueError:
            continue
        if not isinstance(e, dict):
            continue
        if e.get("kind") == "episode_start":
            out = []
        elif e.get("kind") == "autopilot_text":
            out.append({"t": e.get("t"), "severity": e.get("severity"),
                        "text": e.get("text")})
    return out


def takeover_fence_breach(texts: list[dict] | None, t: float) -> dict:
    """The autopilot's own fence-breach message at a takeover at mission time
    `t`: {found True, t, text, severity, after_takeover_s} for the message
    naming a fence breach (BACKSTOP_BREACH_TEXT) nearest the takeover within
    BACKSTOP_BREACH_WINDOW_S, else {found False, why}. `texts` is
    run_autopilot_texts' (None: not read)."""
    if texts is None:
        return {"found": False, "why": "the run's events.jsonl was not read"}
    near = [(abs(float(m["t"]) - float(t)), m) for m in texts
            if isinstance(m.get("t"), (int, float)) and not isinstance(m.get("t"), bool)
            and BACKSTOP_BREACH_TEXT in str(m.get("text") or "").lower()
            and abs(float(m["t"]) - float(t)) <= BACKSTOP_BREACH_WINDOW_S + 1e-9]
    if not near:
        return {"found": False,
                "why": (f"no autopilot text naming a fence breach within "
                        f"{BACKSTOP_BREACH_WINDOW_S:g} s of the takeover "
                        f"({len(texts)} autopilot text(s) in events.jsonl)")}
    _, m = min(near, key=lambda x: x[0])
    return {"found": True, "t": round(float(m["t"]), 3), "text": m.get("text"),
            "severity": m.get("severity"),
            "after_takeover_s": round(float(m["t"]) - float(t), 3)}


def failsafe_backstop(extra: dict) -> dict | None:
    """The autopilot's takeover of a rail flight read as the grant's backstop
    (BACKSTOP_RULE; the block above), from the run's own records as sitl_extra
    keeps them (`rail_window.autopilot_took_over`, `rail_window.
    takeover_fence_breach`, `fsm`). None without a takeover. Else {t, mode,
    previous, fsm_state, fence_breach, counts, why_not}: `counts` True when
    the takeover is a fail-safe triggered by the backstop - the mode is RTL
    or LAND, the autopilot reported a fence breach with it, and the FSM was
    holding in Brake or Loiter with no fail-safe of its own - else False with
    every reason it is not (`why_not`)."""
    win = extra.get("rail_window") or {}
    took = win.get("autopilot_took_over")
    if not isinstance(took, dict):
        return None
    fsm = extra.get("fsm") if isinstance(extra.get("fsm"), dict) else None
    state = (fsm or {}).get("final_state")
    breach = win.get("takeover_fence_breach") or {
        "found": False, "why": "no fence-breach record was read for this run"}
    why = []
    if fsm is None:
        why.append("the flight has no FSM record")
    elif fsm.get("failsafe_triggered"):
        why.append("the FSM had triggered a fail-safe itself")
    elif state not in BACKSTOP_HOLD_STATES:
        why.append(f"the FSM was in {state}, not holding in Brake or Loiter")
    if took.get("mode") not in BACKSTOP_MODES:
        why.append(f"the autopilot entered {took.get('mode')}, not RTL or LAND")
    if not breach.get("found"):
        why.append(f"no fence breach on record: {breach.get('why')}")
    return {"t": took.get("t"), "mode": took.get("mode"), "previous": took.get("previous"),
            "fsm_state": state, "fence_breach": breach,
            "counts": not why, "why_not": "; ".join(why) or None}


def score(spec: ScenarioSpec, rows: list[dict], extra: dict, policy,
          priorities: dict | None = None) -> dict:
    """One episode's KPI table: guardrail.kpi.compute plus the harness's own
    measurements and the scenario's labels."""
    prios = dict(K.rule_priorities(policy), **(priorities or {}))
    # What compute() needs from the harness: the goal (a mission that never
    # arrived is not a success), the self-caused breach count (an imposed
    # unsafe stretch - a zone spawned or moved onto the aircraft - is not an
    # entry), the arm (a Shield-off control's logged repairs were never
    # flown), and the FSM's summary (RTL_triggered / Land_triggered).
    metrics = {"reached_goal": extra.get("reached_goal"),
               "breaches": extra.get("breaches"),
               "shield": "on" if spec.shield else "off"}
    if extra.get("fsm"):
        metrics["fsm"] = extra["fsm"]
    res = K.compute(rows, prios, metrics)
    res.update(extra)
    if extra.get("fsm"):
        # The grant's fail-safe: the FSM entered RTL or Land (fsm.summary()).
        triggered = bool(extra["fsm"]["failsafe_triggered"])
    elif not spec.shield:
        triggered = False                    # no Shield, no fail-safe
    else:
        triggered = None                     # Shield without its FSM: not instrumented
    # ... or, since 2026-11-15, the autopilot's GeoFence took the aircraft to
    # RTL / LAND while the FSM held (failsafe_backstop; BACKSTOP_RULE): the
    # fail-safe triggered, by the backstop. `failsafe_by` says who; an
    # FSM-triggered fail-safe is scored exactly as before.
    backstop = failsafe_backstop(extra)
    by_backstop = bool(backstop and backstop["counts"])
    res["failsafe_by"] = (FAILSAFE_BY_GEOFENCE if by_backstop
                          else FAILSAFE_BY_FSM if triggered else None)
    if by_backstop:
        triggered = True
    res["failsafe_triggered"] = triggered
    # The label and the rubric clause it was derived from (docs/RUBRIC-
    # failsafe.md; the cell's own when the family lists it in failsafe_cells).
    res["failsafe_label"] = spec.expected_failsafe
    res["failsafe_clause"] = spec.failsafe_clause
    # CENSORED (rubric, section 3): the episode ended while the FSM held in
    # Loiter with its operator timeout still running, so whether the
    # fail-safe fires was never shown. Not a correct non-trigger: it is not
    # scored, and a labelled cell fails its label gate (label_failures_keyed)
    # until it flies long enough. Until 2026-11-02 such episodes scored as
    # correct non-triggers (27 in the library).
    held = (extra.get("fsm") or {}).get("ended_holding") or {}
    fsm_censored = bool(held.get("timeout_pending"))
    # A hold the backstop ended has shown its outcome (2026-11-15): not
    # censored. What the reading before the rule made of the run is kept in
    # its record (`fsm_censored`: left out as censored, else scored as not
    # triggered), so that reading can still be printed beside this one.
    res["failsafe_censored"] = fsm_censored and not by_backstop
    if backstop is not None:
        backstop = dict(backstop, fsm_censored=fsm_censored)
    res["failsafe_backstop"] = backstop
    res["failsafe_matches_label"] = (None if spec.expected_failsafe is None
                                     or triggered is None or res["failsafe_censored"]
                                     else float(triggered == spec.expected_failsafe))
    # The grant's outcome vocabulary: success | fail | RTL_triggered |
    # Land_triggered. A fail-safe outcome is the FSM's label; otherwise a P0
    # escape, a self-caused breach or a missed goal is a fail.
    label = (extra.get("fsm") or {}).get("outcome_label")
    if label:
        res["mission_outcome"] = label
    else:
        breaches = extra.get("breaches")
        if breaches is None:
            # Not measured: rail_measurements leaves it out for a policy
            # whose rules switch with the clock (_timed) or a log without
            # `unsafe`. It was read as 0, so since e911e52 a rail flight
            # under a _timed policy with `unsafe` on for 5 rows mid-flight
            # scored `success`, where it had scored `fail` before (review of
            # the F8b merge, P1). A missing count is not 0: compute()'s own
            # reading of the logged `unsafe` flags decides - a stretch
            # entered after the first row, or one still open at the end -
            # which may count a window that opened under the aircraft, the
            # conservative side. (A log with no `unsafe` at all, an older
            # rail's, gives compute() nothing to read either; its breaches
            # gate reads "not measured" and fails the verdict.)
            entered = any(r in ("unsafe_position_entered", "unsafe_never_recovered")
                          for r in res.get("mission_fail_reasons") or [])
        else:
            entered = breaches > 0
        fail = ((res.get("p0_escapes") or 0) > 0 or entered
                or extra.get("reached_goal") is False)
        res["mission_outcome"] = "fail" if fail else "success"
    res["outcome_matches_label"] = (None if spec.expected_outcome is None
                                    else res["mission_outcome"] == spec.expected_outcome)
    return res


def censored_cause(res: dict) -> str | None:
    """Why a censored episode (`failsafe_censored`) never showed its outcome,
    from what its rail flight recorded (sitl_extra's `rail_window`):
    "autopilot_takeover" - the autopilot took the aircraft (a GeoFence RTL)
    and the FSM stopped stepping, its Loiter frozen; the flight ends
    TAKEOVER_GRACE_S later whatever its cap - or "flight_ended" - the flight
    ended (its cap, else the mission's end) with the FSM still holding. None
    for an episode that is not censored. A headless episode records no
    takeover, so it is always "flight_ended"."""
    if not res.get("failsafe_censored"):
        return None
    took = (res.get("rail_window") or {}).get("autopilot_took_over")
    return "autopilot_takeover" if took else "flight_ended"


def censored_remedy(res: dict) -> str:
    """What would decide a censored episode, in words: fly it longer, unless
    the autopilot's takeover froze the FSM (no length helps), saying how
    much longer a lengthened hil campaign already flew it."""
    win = res.get("rail_window") or {}
    add = flight_max_s_add(win)
    longer = (f" (already flown {add:g} s past the rail's cap, --hil-max-s-add)"
              if add else "")
    if censored_cause(res) == "autopilot_takeover":
        took = win.get("autopilot_took_over") or {}
        return (f"the autopilot took the aircraft ({took.get('mode')} at {took.get('t')} s) "
                f"and the FSM stopped stepping, so flying longer will not show it{longer}")
    return f"fly it longer{longer}"


def label_failures(spec: ScenarioSpec, res: dict) -> list[str]:
    return [msg for _, msg in label_failures_keyed(spec, res)]


def label_failures_keyed(spec: ScenarioSpec, res: dict) -> list[tuple[str, str]]:
    """label_failures with each failure's pin key (scenario_spec
    PIN_LABEL_KEYS): `outcome` or `failsafe`."""
    bad: list[tuple[str, str]] = []
    if spec.expected_outcome is not None and not res.get("outcome_matches_label"):
        bad.append(("outcome", f"outcome: labelled {spec.expected_outcome}, flew "
                               f"{res.get('mission_outcome')}"))
    if spec.expected_failsafe is not None:
        if res.get("failsafe_triggered") is None:
            bad.append(("failsafe", "fail-safe: labelled, but this episode ran no "
                                    "escalation FSM (not measured)"))
        elif res.get("failsafe_censored"):
            held = (res.get("fsm") or {}).get("ended_holding") or {}
            bad.append(("failsafe", f"fail-safe: labelled, but the episode ended in "
                        f"Loiter {held.get('dwell_s')} s into a {held.get('timeout_s')} s "
                        f"operator timeout - censored, not measured; "
                        f"{censored_remedy(res)} (docs/RUBRIC-failsafe.md, section 3)"))
        elif res.get("failsafe_matches_label") != 1.0:
            by = (" by the autopilot's GeoFence backstop while the FSM held"
                  if res.get("failsafe_by") == FAILSAFE_BY_GEOFENCE else "")
            bad.append(("failsafe", f"fail-safe: labelled "
                        f"{'expected' if spec.expected_failsafe else 'not expected'}, "
                        f"{'triggered' if res.get('failsafe_triggered') else 'not triggered'}"
                        f"{by}"))
    return bad


# The prefix a failure in a pinned cell carries when the pin does not name its
# gate: a new failure, not the pinned defect (episode_verdict).
NEW_IN_PINNED_CELL = "NEW (not pinned): "


def _bound_text(bound: dict) -> str:
    """{max: 0.01} -> "max 0.01", as check_gates words a bound."""
    return ", ".join(f"{op} {bound[op]}" for op in ("min", "max", "equals")
                     if op in bound)


VERDICTS = ("pass", "tracked", "fail", "known_failure", "unexpected_pass")
FIXED_MESSAGE = ("marked known_failure but every gate passed - "
                 "the defect is fixed; remove the marker")
_MARKS = {"pass": "pass ", "tracked": "track", "fail": "FAIL ",
          "known_failure": "KNOWN", "unexpected_pass": "FIXED"}


def episode_verdict(spec: ScenarioSpec, res: dict, params: dict, seed: int,
                    gates: dict | None = None
                    ) -> tuple[str, list[str], list[str], str | None]:
    """One scored episode's verdict: (status, gates_failed, labels_failed,
    known_failure_reason), status one of VERDICTS. `gates` replaces the
    cell's own (spec.gates) - the rail's derived bounds (rail_gates_ardupilot,
    `--rail-gates ardupilot`, PROPOSED 2026-11-07); None is the cell's.

    The one place the sweep decides it. main() calls it for every episode, and
    so does the gRPC harness (guardrail/harness_service.py), so an episode
    flown over gRPC is judged exactly as the sweep judges it - until
    2026-10-11 the harness carried a copy of this logic.

    `gates_failed` holds the gate failures and, unless the family is tracked,
    the label failures. `labels_failed` is non-empty only for a tracked family,
    whose label mismatches are reported but do not fail it.

    A pinned cell (expect: known_failure, or a matching known_failures entry)
    is a known failure only while every failing gate is one its pins name
    (KnownFailureCell, `fails`), and, where a pin bounds that gate
    (`{P0_escape_rate: {max: 0.01}}`), only while the value stays inside the
    bound. A failure of any other gate, or one past its bound, makes it `fail`,
    and that message is prefixed NEW_IN_PINNED_CELL: until 2026-10-19 any
    failure in a pinned cell was absorbed, so a P0 escape, a breach or an
    unsafe end in a cell pinned for its labels exited 0 (review,
    harness-tools/known-failure-pin-absorbs-new-regressions).
    """
    labels_bad = label_failures_keyed(spec, res)
    keyed = check_gates_keyed(spec.gates if gates is None else gates, res)
    labels_failed: list[str] = []
    if spec.expect == "tracked":
        # A tracked family measures how far past its design point the
        # Shield degrades; a fail-safe its degraded perception provoked is
        # part of that measurement. Its label mismatches are reported
        # (`labels_failed`, and in the fail-safe KPI, which counts every
        # labelled episode) but do not fail it - its gates still do.
        labels_failed = [msg for _, msg in labels_bad]
    else:
        keyed = keyed + labels_bad
    known_why, pinned = spec.known_failure_pins(params, seed)
    if known_why is not None:
        # A gate pinned with a bound (scenario_spec.pin_entry) is the pinned
        # failure only while its value is inside one of the bounds; until the
        # review of F8-harness-tools (P3) a pinned P0_escape_rate measured at
        # 0.0076 stayed KNOWN at 0.5.
        bounds = spec.known_failure_bounds(params, seed)
        new, old = [], []
        for k, msg in keyed:
            if k not in pinned:
                new.append(NEW_IN_PINNED_CELL + msg)
            elif k in bounds and all(check_gates({k: b}, res) for b in bounds[k]):
                new.append(NEW_IN_PINNED_CELL + msg + " (pinned only within "
                           + " or ".join(_bound_text(b) for b in bounds[k]) + ")")
            else:
                old.append(msg)
        bad = new + old
    else:
        new = []
        bad = [msg for _, msg in keyed]
    if new:
        status = "fail"
    elif bad and known_why is not None:
        status = "known_failure"
    elif bad:
        status = "fail"
    elif known_why is not None:
        # A known failure that started passing is news, not a quiet pass.
        status = "unexpected_pass"
        bad = [FIXED_MESSAGE]
    elif spec.expect == "tracked":
        # Its gates hold only what must hold whatever the sensor does; the
        # aircraft may still have entered a polygon. Never a "pass".
        status = "tracked"
    else:
        status = "pass"
    return status, bad, labels_failed, known_why


# --------------------------------------------------------------------------- #
# the rails' gates on a lagging vehicle (PROPOSED, NOT ADOPTED, 2026-11-07)
# --------------------------------------------------------------------------- #

# Gates whose bound was PINNED on the headless IDEAL vehicle's own flight of
# its cell - the measured value, or one worked out from the ideal vehicle's
# physics, plus a margin - rather than set by a requirement: how long the
# aircraft takes to get out, how many ticks it stays in. Their `max` bound
# only (a `min`, "it really happened", is a requirement). A real autopilot
# lags, so the rails fly them longer: the hil rehearsal rh01 (7 Nov) flew
# time-window-opens-on-vehicle 17 ticks inside and 3.9 s to safe against 15
# and 2.7, with no P0 escape, breach or censored episode.
VEHICLE_PINNED_GATES = ("ticks_inside_fence_polygon", "max_time_to_safe_s",
                        "mean_time_to_safe_s")
# `--rail-gates`: the cell's own gates (ideal, the default until the user
# decides) or the bounds derived on a model of the vehicle the rails fly:
# `rail` (the rig's own fitted response from a hover, RAIL_VEHICLE - the
# principle of 2026-11-07: every expectation derived headless on the vehicle
# that flies the KPI runs, with the rail's start), and, for comparison, the
# harness's ArduPilot response entering mid-flight (ardupilot) or from a hover
# (ardupilot-rest). basis -> (response, entry; None: the arm's own).
RAIL_GATE_VEHICLES = {"rail": ("rail", None), "ardupilot": ("ardupilot", "cruise"),
                      "ardupilot-rest": ("ardupilot", "rest")}
RAIL_GATE_BASES = ("ideal",) + tuple(RAIL_GATE_VEHICLES)
# What every derivation flight holds for the FSM's Loiter / Land: ArduPilot's
# own hold where the aircraft is, as every rail flies it.
RAIL_GATE_HOLD = RAIL_VEHICLE["autopilot_hold"]


def _headless_score(spec: ScenarioSpec, defaults, seed: int, response: str,
                    entry: str | None = None) -> dict:
    """One headless flight of the cell on `response`'s vehicle (entering as
    `entry` says, else as the arm does), holding in place as the rails do,
    scored by score()."""
    run = run_episode(spec, defaults, seed, autopilot_hold=RAIL_GATE_HOLD, response=response,
                      **({} if response == "ideal" else {"entry": entry}))
    return score(spec, run.rows, run.extra, run.policy, run.priorities)


def rail_gates_derived(spec: ScenarioSpec, seed: int, defaults=None, *,
                       response: str = "rail", entry: str | None = None,
                       run=None) -> tuple[dict, dict]:
    """PROPOSED, NOT ADOPTED (2026-11-07): the cell's gates for a rail run,
    each vehicle-pinned bound (VEHICLE_PINNED_GATES, its `max`) moved by what
    the rail's vehicle changes on THIS cell and seed. The headless harness
    flies the cell twice, on the ideal vehicle and on `response`'s (the
    `rail` arm: the rig's own fit from a hover; or the ArduPilot arm, entering
    as `entry` says), both holding in place (RAIL_GATE_HOLD), and the bound
    becomes

        ideal bound + (vehicle value - ideal value)

    so each cell keeps the headroom its ideal pin gave it - the margin rule
    the pins used (time-window-opens-on-vehicle's 2.7 s is its 2.6 s plus a
    tick, its 15 ticks the worst cell of the family; altitude-recovery-
    depth's 5.2 s the deepest cell's 5.0 s plus two ticks). Requirement
    gates are untouched; so are the labels (the fail-safe's on the rail
    vehicle are derived by tools/failsafe_rubric.py on the same arm). A
    bound that either flight leaves unmeasured keeps the ideal bound, and
    the derivation says so. `run` replaces the headless flight (a test's):
    (spec, defaults, seed, response, entry) -> score() table. Returns
    (gates, derivation)."""
    gates = {k: dict(v) for k, v in spec.gates.items()}
    todo = [f for f in VEHICLE_PINNED_GATES if "max" in gates.get(f, {})]
    deriv: dict[str, Any] = {}
    if not todo:
        return gates, deriv
    run = run or _headless_score
    ideal = run(spec, defaults, seed, "ideal", None)
    veh = run(spec, defaults, seed, response, entry)
    for f in todo:
        bound = gates[f]["max"]
        vi, va = ideal.get(f), veh.get(f)
        rec = {"ideal_bound": bound, "ideal_value": vi, "vehicle_value": va}
        if vi is None or va is None:
            rec.update(rail_bound=bound, rule=("kept: the "
                                               + ("ideal" if vi is None else response)
                                               + " flight leaves it unmeasured"))
        else:
            nb = bound + (va - vi)
            nb = int(nb) if all(isinstance(x, int) for x in (bound, va, vi)) else round(nb, 4)
            rec.update(rail_bound=nb, rule=f"ideal bound + ({response} - ideal) on this cell")
            gates[f] = dict(gates[f], max=nb)
        deriv[f] = rec
    deriv["_flights"] = {"vehicles": ["ideal", response], "seed": int(seed),
                         "autopilot_hold": RAIL_GATE_HOLD,
                         "entry": entry or RESPONSE_ENTRY.get(response, DEFAULT_ENTRY),
                         "response": response_for(response)}
    return gates, deriv


def rail_gates_ardupilot(spec: ScenarioSpec, seed: int, defaults=None, *,
                         entry: str | None = DEFAULT_ENTRY, run=None) -> tuple[dict, dict]:
    """rail_gates_derived on the harness's ArduPilot arm (the first proposal
    of 2026-11-07, kept for comparison)."""
    return rail_gates_derived(spec, seed, defaults, response="ardupilot", entry=entry, run=run)


def rail_gates_for(basis: str, spec: ScenarioSpec, seed: int, defaults=None, *,
                   run=None) -> tuple[dict | None, dict | None]:
    """(gates, derivation) of a `--rail-gates` basis: (None, None) for
    "ideal" (the cell's own), rail_gates_derived on the basis's vehicle for
    the others."""
    if basis == "ideal":
        return None, None
    if basis not in RAIL_GATE_VEHICLES:
        raise ValueError(f"--rail-gates {basis!r}: one of {list(RAIL_GATE_BASES)}")
    response, entry = RAIL_GATE_VEHICLES[basis]
    return rail_gates_derived(spec, seed, defaults, response=response, entry=entry, run=run)


def rail_gate_set(basis: str, spec: ScenarioSpec, seed: int, defaults=None, *,
                  run=None) -> tuple[dict, dict | None]:
    """The gates a rail run's verdict uses on `basis` (rail_gates_for; the
    cell's own for "ideal"), with each safety invariant's autopilot
    counterpart gated beside it (RAIL_INVARIANT_PAIRS: a cell that bounds the
    Shield's breaches or polygon entries bounds the autopilot's KEEP-OUT
    breaches and polygon entries with the same bound - the user's ruling of
    2026-11-07: reported apart as the KPI definitions require, gated on the
    keep-outs; corridor and altitude-band departures are reported only). A
    counterpart the cell gates itself is its own. Returns (gates,
    derivation)."""
    derived, derivation = rail_gates_for(basis, spec, seed, defaults, run=run)
    gates = {k: dict(v) for k, v in (derived if derived is not None else spec.gates).items()}
    for f, ap in RAIL_INVARIANT_PAIRS.items():
        if f in gates and ap not in gates:
            gates[ap] = dict(gates[f])
    return gates, derivation


# --------------------------------------------------------------------------- #
# run-level summaries (pure functions of the scored items, so they are tested)
# --------------------------------------------------------------------------- #

def failsafe_summary(scored: list[dict]) -> dict:
    """Label-based fail-safe trigger correctness, WITH ITS NULLS.

    "Triggered when expected, not when not expected" (Stress Testing,
    Acceptance KPIs, target >= 99 %) over the episodes that carry a label. Two
    trivial policies bound it: never trigger scores the share labelled "not
    expected", always trigger the share labelled "expected". A correctness is
    only evidence of skill above both lines. The first version reported
    0.934783 / 0.961538 with no null; both were BELOW never-trigger (0.956522 /
    0.980769).

    Since 2026-10-07 the trigger is the escalation FSM entering RTL or Land,
    and guardrail.fsm.score_failsafe_triggers' verdict sits beside this one
    (`fsm_scoring`: the Wilson 95 % lower bound, and `meets_target` None when
    every label is the same, because then one of the stubs ties).
    """
    fs = [r for r in scored if r["kpi"].get("failsafe_matches_label") is not None]
    # Labelled episodes that ended in Loiter with the operator timeout still
    # running (score: `failsafe_censored`): not scored, counted, and passed
    # to score_failsafe_triggers as not measured (docs/RUBRIC-failsafe.md).
    censored = [r for r in scored if r["kpi"].get("failsafe_censored")
                and r["kpi"].get("failsafe_label") is not None]
    # Fail-safes the autopilot's GeoFence backstop triggered while the FSM
    # held (2026-11-15, BACKSTOP_RULE; score()'s `failsafe_by`): scored in
    # `fs` like any other trigger, counted here, and read the two earlier
    # ways beside the figure (rail_label_scores does the same per reading).
    # None on a headless sweep: no autopilot, no takeover.
    backstop = [r for r in fs if r["kpi"].get("failsafe_by") == FAILSAFE_BY_GEOFENCE]

    def _label(r: dict) -> bool:
        # The label, from the verdict and whether it matched (as below).
        return bool(r["kpi"]["failsafe_triggered"]) == (r["kpi"]["failsafe_matches_label"] == 1.0)
    rest = [(_label(r), bool(r["kpi"]["failsafe_triggered"]))
            for r in fs if r["kpi"].get("failsafe_by") != FAILSAFE_BY_GEOFENCE]
    held_in_loiter = [r for r in backstop
                      if (r["kpi"].get("failsafe_backstop") or {}).get("fsm_censored")]

    def _figure(prs: list, **more) -> dict:
        ok = sum(1 for e, t in prs if e == t)
        return {"labelled": len(prs), "correct": ok,
                "fail_safe_correctness": round(ok / len(prs), 6) if prs else None, **more}
    n = len(fs)
    expected = sum(1 for r in fs
                   if r["kpi"]["failsafe_triggered"] == (r["kpi"]["failsafe_matches_label"] == 1.0))
    correct = sum(1 for r in fs if r["kpi"]["failsafe_matches_label"] == 1.0)
    false_t = sum(1 for r in fs if r["kpi"]["failsafe_triggered"]
                  and r["kpi"]["failsafe_matches_label"] == 0.0)
    missed = sum(1 for r in fs if not r["kpi"]["failsafe_triggered"]
                 and r["kpi"]["failsafe_matches_label"] == 0.0)
    score_ = round(correct / n, 6) if n else None
    never = round((n - expected) / n, 6) if n else None
    always = round(expected / n, 6) if n else None
    # Distinct FLIGHTS beside the episodes (review of 2026-10-12): cells and
    # seeds of one family that flew the same KPI table under the same label
    # are one flight, counted once. A deterministic cell flown on three seeds,
    # or wind cells the closed loop rejects alike, otherwise weigh their
    # verdict three or four times.
    flights: dict[tuple, float] = {}
    for r in fs:
        flights[(r.get("scenario_id"), r.get("template"), kpi_signature(r),
                 r["kpi"]["failsafe_matches_label"],
                 r["kpi"]["failsafe_triggered"])] = r["kpi"]["failsafe_matches_label"]
    n_fl = len(flights)
    correct_fl = sum(1 for v in flights.values() if v == 1.0)
    # Which FSM edge sent each triggered episode to RTL / Land (guardrail/fsm.py
    # EDGES: G3 theta / no convergence -> Loiter, then G4 N-in-T persists, G5
    # the Loiter timeout, X6 a stop illegal for T, G11 / G12 a rule's own
    # action...), so a correctness below target says WHY.
    by_edge: dict[str, dict[str, int]] = {"false": {}, "true": {}}
    for r in fs:
        if not r["kpi"]["failsafe_triggered"]:
            continue
        tr = ((r["kpi"].get("fsm") or {}).get("transitions") or [])
        chain = []
        for x in tr:
            chain.append(str(x.get("edge")))
            if x.get("to") in ("RTL", "Land"):
                break
        into = " > ".join(chain) if chain else "unknown"
        if r["kpi"].get("failsafe_by") == FAILSAFE_BY_GEOFENCE:
            # No FSM edge went to RTL / Land: the autopilot's GeoFence did.
            into = (into + " > " if chain else "") + "autopilot GeoFence"
        side = "true" if r["kpi"]["failsafe_matches_label"] == 1.0 else "false"
        by_edge[side][into] = by_edge[side].get(into, 0) + 1
    fsm_scoring = None
    try:
        from guardrail.fsm import score_failsafe_triggers
        eps = [{"expected_failsafe": (r["kpi"]["failsafe_triggered"]
                                      if r["kpi"]["failsafe_matches_label"] == 1.0
                                      else not r["kpi"]["failsafe_triggered"]),
                "triggered": r["kpi"]["failsafe_triggered"]} for r in fs]
        eps += [{"expected_failsafe": r["kpi"]["failsafe_label"], "triggered": None}
                for r in censored]
        s = score_failsafe_triggers(eps, GRANT_FAILSAFE_TARGET)
        fsm_scoring = {k: s[k] for k in ("failsafe_trigger_correctness", "scored",
                                         "beats_null", "wilson_low_95", "meets_target",
                                         "meets_target_at_95",
                                         "min_error_free_episodes_for_target_at_95",
                                         "warnings")}
    except ImportError:
        pass
    by_clause: dict[str, dict[str, int]] = {}
    for r in fs:
        c = by_clause.setdefault(str(r["kpi"].get("failsafe_clause")),
                                 {"scored": 0, "correct": 0})
        c["scored"] += 1
        c["correct"] += int(r["kpi"]["failsafe_matches_label"] == 1.0)
    return {
        "labelled": n, "unlabelled": len(scored) - n - len(censored),
        "censored_in_loiter": len(censored),
        "censored_episodes": sorted(r.get("id") or "?" for r in censored),
        # Why each never showed its outcome (censored_cause, 2026-11-14): a
        # flight that ended still holding may be decided by flying it longer
        # (--hil-max-s-add); one the autopilot took cannot.
        "censored_by_cause": {c: sorted(r.get("id") or "?" for r in censored
                                        if censored_cause(r["kpi"]) == c)
                              for c in ("flight_ended", "autopilot_takeover")},
        "by_rubric_clause": dict(sorted(by_clause.items())),
        # Who triggered the scored fail-safes (2026-11-15, BACKSTOP_RULE), the
        # episodes the GeoFence backstop triggered, and the figure with those
        # read the two earlier ways: as before the rule (left out as censored
        # where the FSM held in Loiter with its timeout pending, else scored
        # as not triggered) and all counted as not triggered.
        "triggered_by": {
            FAILSAFE_BY_FSM: sum(1 for r in fs if r["kpi"]["failsafe_triggered"]
                                 and r["kpi"].get("failsafe_by") != FAILSAFE_BY_GEOFENCE),
            FAILSAFE_BY_GEOFENCE: len(backstop)},
        "backstop_triggered_episodes": sorted(r.get("id") or "?" for r in backstop),
        "before_backstop_rule": _figure(
            rest + [(_label(r), False) for r in backstop if r not in held_in_loiter],
            left_out_as_censored=len(held_in_loiter)),
        "backstop_as_not_triggered": _figure(
            rest + [(_label(r), False) for r in backstop]),
        "backstop_rule_since": BACKSTOP_RULE["since"],
        "labelled_expected": expected, "labelled_not_expected": n - expected,
        "correct": correct, "false_triggers": false_t, "missed_triggers": missed,
        "triggers_by_edge": by_edge,
        "fail_safe_correctness": score_,
        "labelled_distinct_flights": n_fl,
        "correct_distinct_flights": correct_fl,
        "fail_safe_correctness_distinct_flights": (round(correct_fl / n_fl, 6)
                                                   if n_fl else None),
        "null_never_trigger": never,
        "null_always_trigger": always,
        "beats_never_trigger": (None if score_ is None else score_ > never),
        "beats_always_trigger": (None if score_ is None else score_ > always),
        "grant_target": GRANT_FAILSAFE_TARGET,
        "meets_grant_target": (None if score_ is None
                               else score_ >= GRANT_FAILSAFE_TARGET),
        # Can the labels tell a skilled Shield from a trivial one at all? Only
        # if both kinds of label are present.
        "discriminating": 0 < expected < n if n else False,
        "fsm_scoring": fsm_scoring,
        "observed_as": "the escalation FSM entering RTL or Land (Shield.filter's "
                       "FSM; a Brake or a Loiter is not a fail-safe trigger)",
    }


def true_state_summary(scored: list[dict]) -> dict:
    """Where the aircraft REALLY was, beside the Shield's own P0 count.

    The grant's P0 escape rate is a "counter from Shield repair log": what the
    Shield found illegal in what it flew, judged at the state it SAW. Under GPS
    noise, latency or a frozen fix that log can be clean while the aircraft is
    inside the polygon. Shield-on episodes only; broken down by status so a
    reader can see which entries are tracked or known failures and which, if
    any, sit among the passes.

    THE AUTOPILOT'S SHARE (review of stress-harness-2). Ticks the modelled
    LOITER / RTL / LAND flew are outside the Shield's escape rate by
    definition (guardrail.kpi, "AUTOPILOT TICKS"), and until then they were
    outside every run-level number too: the summary read "1 entry" and a P0
    escape of 0.0 while the modelled RTL flew straight through zones (713
    autopilot ticks with a P0-violating action in the library run). Its
    entries, breaches and P0 ticks are reported here, apart from the
    Shield's, never merged into them.
    """
    on = [r for r in scored if r.get("arm", "on") == "on"]
    entered = [r for r in on if (r["kpi"].get("polygon_entries") or 0) > 0]
    ap_entered = [r for r in on if (r["kpi"].get("autopilot_polygon_entries") or 0) > 0]
    ap_breach = [r for r in on if (r["kpi"].get("autopilot_breaches") or 0) > 0]
    ap_p0 = [r for r in on if (r["kpi"].get("autopilot_p0_flown_ticks") or 0) > 0]
    ap_by_status: dict[str, int] = {}
    for r in ap_p0:
        ap_by_status[r["status"]] = ap_by_status.get(r["status"], 0) + 1
    inside = [r for r in on if (r["kpi"].get("ticks_inside_fence_polygon") or 0) > 0]
    # Why each episode inside a polygon was there, as three disjoint groups
    # (review of 2026-10-19, harness-tools/imposed-only-counts-autopilot-
    # entries): the Shield flew itself in (`entered`); the modelled LOITER /
    # RTL / LAND carried it in and the Shield never did; or neither crossed an
    # edge - a zone appeared, moved, grew or switched on under it. Until then
    # "imposed only" was len(inside) - len(entered), so an autopilot entry
    # (polygon_entries 0, autopilot_polygon_entries 1: run_episode splits a
    # crossing by who flew it) was printed as "inside only because a zone
    # appeared under them" - 4 of the published sweep's 12 were the
    # wind-beyond-authority LOITER drifting into a static zone with no harness
    # event at all. Filtered per episode, not subtracted: an episode both
    # entered would be subtracted twice.
    inside_ap_only = [r for r in inside
                      if not (r["kpi"].get("polygon_entries") or 0)
                      and (r["kpi"].get("autopilot_polygon_entries") or 0) > 0]
    imposed = [r for r in inside
               if not (r["kpi"].get("polygon_entries") or 0)
               and not (r["kpi"].get("autopilot_polygon_entries") or 0)]
    # Windows proved to open on the aircraft (Shield._opening_imposed): the
    # episodes with a tick counted as anticipated only through the trace's
    # `window_opens_on`. Since C2 (2026-10-21) the escape from such a window
    # is judged the recovery, not a P0 escape, so the P0 rate there is a
    # re-judgement of a path, and how long the aircraft was inside is what
    # says whether it got out as fast as before: reported beside the rate
    # (review of 3660383, P2; in the library, 7c9c9e6's 3 / 8 / 13 ticks and
    # 2.6 s on the same path).
    #
    # An episode is listed when ANY tick's decision named such a window -
    # before the opening or after it (`window_opened_on_ticks`, run_episode)
    # - not only when some tick's action was already the escape
    # (`anticipated_opening_only_ticks_*`, kept as the measure of how much
    # the anticipated count was extended). Review of b1d3aca (P4): a window
    # that opened on a hovering or drifting pilot - every tick before the
    # opening a violation, then repaired - was not listed, so the line
    # beside the P0 rate was not the list of such episodes it claims to be.
    # The opening-only test stays as a fallback for a record written before
    # the field existed.
    opened = [r for r in on
              if (r["kpi"].get("window_opened_on_ticks") or 0) > 0
              or any((n or 0) > 0 for k, n in r["kpi"].items()
                     if k.startswith("anticipated_opening_only_ticks_"))]
    opened_t2s = [r["kpi"]["max_time_to_safe_s"] for r in opened
                  if r["kpi"].get("max_time_to_safe_s") is not None]
    # Escapes the Shield slowed (review of 69c27bd, P3): episodes with a tick
    # whose decision refused the escape floor at the speed cap (a slower
    # escape flown, `cap_refused_ticks`) or refused it outright (the action
    # flown unfloored or reported to the FSM, `floor_refused_ticks`). Such a
    # tick is legal, so no P0 count sees it: the library's sideways pilot
    # stayed 16 ticks inside a zone that opened on it, its cap refused on
    # most of them, with every summary reading 0 escapes. None (not 0) when
    # no record carries the count - a sweep written before it existed.
    counted = [r for r in on if "cap_refused_ticks" in r["kpi"]
               or "floor_refused_ticks" in r["kpi"]]
    slowed = [r for r in counted if (r["kpi"].get("cap_refused_ticks") or 0)
              + (r["kpi"].get("floor_refused_ticks") or 0) > 0]
    slowed_by_status: dict[str, int] = {}
    for r in slowed:
        slowed_by_status[r["status"]] = slowed_by_status.get(r["status"], 0) + 1
    dmins = [r["kpi"]["min_true_dist_to_fence_m"] for r in entered
             if r["kpi"].get("min_true_dist_to_fence_m") is not None]
    by_status: dict[str, int] = {}
    for r in entered:
        by_status[r["status"]] = by_status.get(r["status"], 0) + 1
    # Which autopilot mode flew them, and how long each mode flew at all: a
    # LOITER hold, an RTL leg and a landing are different hand-overs.
    p0_by_mode: dict[str, int] = {}
    ticks_by_mode: dict[str, int] = {}
    for r in on:
        for mode, n in (r["kpi"].get("autopilot_p0_flown_ticks_by_mode") or {}).items():
            p0_by_mode[mode] = p0_by_mode.get(mode, 0) + n
        for mode, n in (r["kpi"].get("autopilot_mode_ticks") or {}).items():
            ticks_by_mode[mode] = ticks_by_mode.get(mode, 0) + n
    return {
        "shield_on_episodes": len(on),
        # Flew itself across a polygon edge: the incursions.
        "episodes_entering_polygon": len(entered),
        "episodes_entering_polygon_by_status": by_status,
        "min_true_dist_to_fence_m_of_entries": min(dmins) if dmins else None,
        # Inside at some tick for any reason, including a zone spawned on it or
        # a window that opened on it (imposed; time_to_safe measures those).
        # Split three ways, disjoint: the Shield entered (episodes_entering_
        # polygon), only the autopilot entered, neither entered (imposed).
        "episodes_inside_polygon": len(inside),
        "episodes_inside_polygon_autopilot_entered": len(inside_ap_only),
        "episodes_inside_polygon_imposed_only": len(imposed),
        "true_p0_flown_ticks": sum(r["kpi"].get("true_p0_flown_ticks") or 0 for r in on),
        "episodes_with_breach": sum(1 for r in on if (r["kpi"].get("breaches") or 0) > 0),
        # A window opened on the aircraft (above): episodes, their ticks
        # inside a polygon (one per episode, in run order) and the longest
        # time to safe among them. None when there are none.
        "episodes_window_opened_on": len(opened),
        "window_opened_on_ticks_inside_polygon": [
            r["kpi"].get("ticks_inside_fence_polygon") for r in opened],
        "window_opened_on_max_time_to_safe_s": max(opened_t2s) if opened_t2s else None,
        # Escapes slowed (above): ticks, and the episodes with any, by status.
        "cap_refused_ticks": (sum(r["kpi"].get("cap_refused_ticks") or 0 for r in counted)
                              if counted else None),
        "floor_refused_ticks": (sum(r["kpi"].get("floor_refused_ticks") or 0 for r in counted)
                                if counted else None),
        "episodes_escape_slowed": len(slowed) if counted else None,
        "episodes_escape_slowed_by_status": slowed_by_status,
        # ---- what the AUTOPILOT flew (never merged into the counts above) ----
        "autopilot_ticks": sum(r["kpi"].get("autopilot_ticks") or 0 for r in on),
        # Of true_p0_flown_ticks, the ones the modelled LOITER / RTL / LAND flew.
        "autopilot_p0_flown_ticks": sum(r["kpi"].get("autopilot_p0_flown_ticks") or 0
                                        for r in on),
        "autopilot_p0_flown_ticks_by_mode": dict(sorted(p0_by_mode.items())),
        "autopilot_ticks_by_mode": dict(sorted(ticks_by_mode.items())),
        "episodes_rtl_failed": sum(1 for r in on if (r["kpi"].get("rtl_failed_ticks") or 0) > 0),
        "episodes_autopilot_p0_flown": len(ap_p0),
        "episodes_autopilot_p0_flown_by_status": ap_by_status,
        "autopilot_polygon_entries": sum(r["kpi"].get("autopilot_polygon_entries") or 0
                                         for r in on),
        "episodes_autopilot_entering_polygon": len(ap_entered),
        "autopilot_breaches": sum(r["kpi"].get("autopilot_breaches") or 0 for r in on),
        "episodes_autopilot_breach": len(ap_breach),
        "basis": "true position, harness oracle Shield; the grant's KPI is the "
                 "Shield's own count (p0_escape_rate), reported unchanged. The "
                 "autopilot_* figures are the modelled LOITER / RTL / LAND's, "
                 "outside the Shield's escape rate by definition",
    }


def arms_summary(scored: list[dict]) -> dict | None:
    """What the paraphrase and prefix arms showed, and what they cannot.

    Per (cell, seed): were the arms' KPI signatures identical? They must be:
    no headless pilot reads text, so an arm that changed the flight would mean
    the plumbing perturbs the run. The per-paraphrase success rates
    (guardrail.kpi.per_paraphrase_robustness, canonical wording as the null)
    are printed as plumbing, with `pilot_reads_text: false`: they are not a
    robustness measurement and must not be quoted as one.

    THE EXCEPTION, PER PILOT (2026-11-16): an episode whose arm record says
    `pilot_reads_prefix` was flown by a pilot that consumes the CSP (module
    docstring, THE PREFIX CONSUMER). Its prefix-on and prefix-off arms are
    two flights by design, so they are left out of the "must be identical"
    checks above - which hold as before for every other pilot - and reported
    under `prefix_consumer` (prefix_consumer_summary): the on / off figures
    and their delta, and the A/A pairs beside them."""
    armed = [r for r in scored if r.get("arm_record")]
    if not armed:
        return None
    consuming = [r for r in armed if r["arm_record"].get("pilot_reads_prefix")]
    all_groups = {(r["id_cell"], r["seed"]) for r in armed}
    armed_all, armed = armed, [r for r in armed
                               if not r["arm_record"].get("pilot_reads_prefix")]
    groups: dict[tuple, list] = {}
    for r in armed:
        groups.setdefault((r["id_cell"], r["seed"]), []).append(r)
    differ = [f"{c} seed={s}" for (c, s), rs in groups.items()
              if len({kpi_signature(x) for x in rs}) > 1]
    trials = [{"paraphrase_id": ((r["arm_record"].get("paraphrase") or {})
                                 .get("paraphrase_id")),
               "backend": ((r["arm_record"].get("paraphrase") or {}).get("backend")),
               "scenario": r["id_cell"], "seed": r["seed"],
               "mission_success": r["kpi"].get("mission_success")}
              for r in armed if r["arm_record"].get("paraphrase")]
    rob = K.per_paraphrase_robustness(trials) if trials else None
    pairs = {}
    for r in armed:
        pairs.setdefault((r["id_cell"], r["seed"]), {}).setdefault(
            r["arm_record"]["prefix"], []).append(r)
    ab = [(k, v) for k, v in pairs.items() if "on" in v and "off" in v]
    ab_diff = [f"{c} seed={s}" for (c, s), v in ab
               if {kpi_signature(x) for x in v["on"]} != {kpi_signature(x) for x in v["off"]}]
    out = {
        "episodes_with_arms": len(armed_all),
        "cell_seed_groups": len(all_groups),
        "groups_whose_arms_flew_differently": differ,
        "paraphrase_trials": len(trials),
        "per_paraphrase": rob,
        "prefix_ab_pairs": len(ab),
        "prefix_ab_pairs_that_differ": ab_diff,
        "pilot_reads_text": False,
        "note": ("plumbing check only: every headless pilot ignores text, so the "
                 "arms are the same flight by construction and any difference "
                 "would be a harness defect. A robustness number needs a pilot "
                 "that reads language."),
    }
    if consuming:
        # Only on a run that flew a prefix consumer, so every other run's
        # `_arms` block is the one it always was.
        out["prefix_consumer"] = prefix_consumer_summary(consuming)
    return out


def prefix_consumer_summary(consuming: list[dict]) -> dict:
    """The prefix A/B of a pilot that consumes the CSP (arms_summary's
    `prefix_consumer`), over scored episodes whose arm record says
    `pilot_reads_prefix`: per (cell, seed), the prefix-on arm's first flight
    against the prefix-off arm - mission success, P0 escapes, repairs per
    episode, mean repair magnitude and time to safe, each on, off and
    on minus off, from tools/kpi_report.py prefix_ab_figures (one
    implementation for this summary and the KPI report) - the pairs whose two
    arms scored alike (a null: the Prefix changed nothing there), and the A/A
    pairs (the prefix-on arm flown twice on one seed) with the pairs that
    differ. Headless an A/A pair that differs is a harness defect (the
    integrator is deterministic); on a rail it is the noise floor."""
    sys.path.insert(0, str(ROOT / "tools"))
    import kpi_report
    by: dict[tuple, dict] = {}
    for r in consuming:
        a = r["arm_record"]
        side = "aa" if a.get("repeat") else a["prefix"]
        by.setdefault((r["id_cell"], r["seed"]), {}).setdefault(side, []).append(r)
    pairs = {k: v for k, v in by.items() if "on" in v and "off" in v}
    aa = {k: v for k, v in by.items() if "on" in v and "aa" in v}

    def label(k) -> str:
        return f"{k[0]} seed={k[1]}"
    on = [r["kpi"] for v in pairs.values() for r in v["on"]]
    off = [r["kpi"] for v in pairs.values() for r in v["off"]]
    aa_tables = [(v["on"][0]["kpi"], v["aa"][0]["kpi"]) for v in aa.values()]
    # Per pair, its cell and two flights: how many pairs are distinct flights
    # (headless, a seed does not vary a cell: 15 pairs of 5 cells are 5).
    flown = [(k[0], v["on"][0]["kpi"], v["off"][0]["kpi"]) for k, v in sorted(pairs.items())]
    figures = kpi_report.prefix_ab_figures(on, off, aa_tables, pairs=flown) if pairs else None
    consumers = sorted({str((r["arm_record"].get("consumer") or {}).get("id"))
                        for r in consuming})
    per_cell = [{"cell": k[0], "seed": k[1],
                 **{f"{side}_{f}": v[side][0]["kpi"].get(f)
                    for side in ("on", "off")
                    for f in ("repair_count", "mean_repair_magnitude_mps",
                              "mean_time_to_safe_s", "p0_escapes", "mission_outcome")},
                 "on_mission_success": K.mission_success_with_goal(v["on"][0]["kpi"]),
                 "off_mission_success": K.mission_success_with_goal(v["off"][0]["kpi"])}
                for k, v in sorted(pairs.items())]
    return {
        "pilot_reads_prefix": True,
        "consumers": consumers,
        "episodes": len(consuming),
        "pairs": len(pairs),
        "distinct_pairs": (figures or {}).get("distinct", {}).get("pairs", 0),
        "pairs_that_differ": [label(k) for k, v in sorted(pairs.items())
                              if {kpi_signature(x) for x in v["on"]}
                              != {kpi_signature(x) for x in v["off"]}],
        "aa_pairs": len(aa),
        "aa_pairs_that_differ": [label(k) for k, v in sorted(aa.items())
                                 if kpi_signature(v["on"][0]) != kpi_signature(v["aa"][0])],
        "figures": figures,
        "per_cell": per_cell,
        "labels_not_applied_to_prefix_on": LABELS_NOT_APPLIED,
        "note": kpi_report.PREFIX_CONSUMER_NOTE,
    }


def family_summary(scored: list[dict]) -> dict[str, dict]:
    """Per template: statuses, true-state counts, outcomes and how many KPI
    signatures its episodes produced (the `_families` block)."""
    families: dict[str, dict] = {}
    sigs_by_family: dict[str, set] = {}
    for r in scored:
        f = families.setdefault(r["template"], {"episodes": 0, "pass": 0, "tracked": 0,
                                                "fail": 0, "known_failure": 0,
                                                "episodes_with_breach": 0,
                                                "episodes_entering_polygon": 0,
                                                "episodes_inside_polygon": 0,
                                                "true_p0_flown_ticks": 0,
                                                "autopilot_p0_flown_ticks": 0,
                                                "min_true_dist_to_fence_m": None,
                                                "fsm_outcomes": {}})
        f["episodes"] += 1
        f[r["status"] if r["status"] in ("pass", "tracked", "known_failure")
          else "fail"] += 1
        k = r["kpi"]
        f["episodes_with_breach"] += int((k.get("breaches") or 0) > 0)
        f["episodes_entering_polygon"] += int((k.get("polygon_entries") or 0) > 0)
        f["episodes_inside_polygon"] += int((k.get("ticks_inside_fence_polygon") or 0) > 0)
        f["true_p0_flown_ticks"] += k.get("true_p0_flown_ticks") or 0
        f["autopilot_p0_flown_ticks"] += k.get("autopilot_p0_flown_ticks") or 0
        mo = k.get("mission_outcome")
        f["fsm_outcomes"][mo] = f["fsm_outcomes"].get(mo, 0) + 1
        d = k.get("min_true_dist_to_fence_m")
        if d is not None:
            cur = f["min_true_dist_to_fence_m"]
            f["min_true_dist_to_fence_m"] = d if cur is None else min(cur, d)
        sigs_by_family.setdefault(r["template"], set()).add(kpi_signature(r))
    for name, f in families.items():
        f["distinct_kpi_signatures"] = len(sigs_by_family[name])
    return families


def cell_extras(tables: list[dict]) -> dict:
    """What a `_cell_rollup` row adds to guardrail.kpi.rollup's: whether it is
    the grant's Shield configuration, and the P0 ticks the AUTOPILOT flew,
    labelled by mode (outside the Shield's escape rate by definition, so
    they are stated here rather than left out of the row)."""
    by_mode: dict[str, int] = {}
    ticks_mode: dict[str, int] = {}
    for t in tables:
        for mode, n in (t.get("autopilot_p0_flown_ticks_by_mode") or {}).items():
            by_mode[mode] = by_mode.get(mode, 0) + n
        for mode, n in (t.get("autopilot_mode_ticks") or {}).items():
            ticks_mode[mode] = ticks_mode.get(mode, 0) + n
    arms = {t.get("forecast_arm", "grant") for t in tables}
    rarms = {t.get("response_arm", "ideal") for t in tables}
    fcs = [t.get("forecast") for t in tables]
    return {"forecast_arm": arms.pop() if len(arms) == 1 else sorted(arms),
            # The vehicle the row's episodes were flown on (since 2026-10-19;
            # a table that predates the label was flown on the ideal one).
            "response_arm": rarms.pop() if len(rarms) == 1 else sorted(rarms),
            # None, not False, for a row whose episodes recorded no forecast:
            # the SITL rail's node builds its own Shield (at LOOKAHEAD_S /
            # LOOKAHEAD_DT_S), which this harness neither chose nor saw.
            "grant_config": (None if any(f is None for f in fcs)
                             else all(f.get("grant_config") is True for f in fcs)),
            "autopilot_p0_flown_ticks": sum(t.get("autopilot_p0_flown_ticks") or 0
                                            for t in tables),
            "autopilot_p0_flown_ticks_by_mode": dict(sorted(by_mode.items())),
            "autopilot_ticks_by_mode": dict(sorted(ticks_mode.items())),
            "episodes_rtl_failed": sum(1 for t in tables
                                       if (t.get("rtl_failed_ticks") or 0) > 0)}


def _twin_id(run_id: str, tag: str = "forecast-3s") -> str:
    """The headline episode a comparison episode shadows: its run id with the
    comparison's arm tag removed ("forecast-3s", or "response-ardupilot")."""
    return run_id.replace(f",{tag}", "").replace(f"#{tag}", "")


# What a comparison row reports of each side.
_CMP_FIELDS = ("status", "mission_outcome", "failsafe_triggered", "p0_escapes",
               "true_p0_flown_ticks", "autopilot_p0_flown_ticks", "breaches",
               "polygon_entries", "repair_count", "reached_goal")


def _cmp_side(r: dict) -> dict:
    k = r.get("kpi") or {}
    out = {f: (r.get(f) if f == "status" else k.get(f)) for f in _CMP_FIELDS}
    out["fsm_edges"] = ">".join(str(t.get("edge")) for t in
                                ((k.get("fsm") or {}).get("transitions") or []))
    return out


def forecast_comparison(grant: list[dict], cmp_scored: list[dict],
                        cmp_counts: dict, cmp_rows: list[dict]) -> dict:
    """The 3 s comparison arm beside the grant's forecast (module docstring):
    its own counts and summaries, and every episode whose KPI table moved
    between the two forecasts (status, outcome, fail-safe, P0 and true-state
    counts, repairs, goal, the FSM's edges). Never merged into a headline."""
    twins = {(r["id"], r["seed"]): r for r in grant}
    moved, by_family = [], {}
    for r in cmp_scored:
        g = twins.get((_twin_id(r["id"]), r["seed"]))
        if g is None:
            continue
        a, b = _cmp_side(g), _cmp_side(r)
        fam = by_family.setdefault(r["template"], {"episodes": 0, "moved": 0,
                                                   "status_changed": 0,
                                                   "outcome_changed": 0,
                                                   "failsafe_changed": 0})
        fam["episodes"] += 1
        if a != b:
            fam["moved"] += 1
            fam["status_changed"] += int(a["status"] != b["status"])
            fam["outcome_changed"] += int(a["mission_outcome"] != b["mission_outcome"])
            fam["failsafe_changed"] += int(a["failsafe_triggered"] != b["failsafe_triggered"])
            moved.append({"id": g["id"], "seed": r["seed"], "family": r["template"],
                          "grant": a, "3s": b})
    fc = next((r["kpi"].get("forecast") for r in cmp_scored if r["kpi"].get("forecast")),
              None)
    return {
        "arm": "3s", "grant_config": False, "forecast": fc,
        "note": ("the forecast every headless Shield used until 2026-10-12 (3 s at a "
                 "0.5 s step, 6 poses), flown beside the grant's 5 s at 0.1 s for "
                 "comparison only: never in _counts, _failsafe_labels, _true_state, "
                 "the top failures or the exit code"),
        "counts": cmp_counts,
        "failsafe_labels": failsafe_summary(cmp_scored),
        "true_state": true_state_summary(cmp_scored),
        "outcomes": {o: sum(1 for r in cmp_scored if r["kpi"].get("mission_outcome") == o)
                     for o in K.OUTCOMES},
        "families": family_summary(cmp_scored),
        "moved_by_family": by_family,
        "episodes_moved": len(moved),
        "moved": moved,
        "results": cmp_rows,
    }


# What the vehicle-response comparison reports of one arm, per family. The
# grant's own KPI (the Shield's P0 escape count) beside the true-state counts
# that a vehicle which carries its speed can move while that KPI stays put.
_RSP_SUM_FIELDS = (
    # Times the aircraft moved ITSELF from a legal position into an unsafe
    # one (run_episode `breaches`: the unsafe-position entries), and the same
    # for moves the modelled LOITER / RTL / LAND made.
    ("unsafe_position_entries", "breaches"),
    ("autopilot_unsafe_position_entries", "autopilot_breaches"),
    # Times it flew itself across a P0 polygon's own edge (true-state
    # entries), Shield-flown and autopilot-flown.
    ("polygon_entries", "polygon_entries"),
    ("autopilot_polygon_entries", "autopilot_polygon_entries"),
    ("ticks_inside_polygon", "ticks_inside_fence_polygon"),
    # The grant's KPI: P0 violations still in the action the Shield flew.
    ("p0_escapes", "p0_escapes"),
    # The COMMANDED action judged at the true position (before the vehicle
    # steps), and of those the ticks the autopilot flew.
    ("true_p0_flown_ticks", "true_p0_flown_ticks"),
    ("autopilot_p0_flown_ticks", "autopilot_p0_flown_ticks"),
    # The velocity the AIRFRAME flew judged at the same true state (_episode
    # `vjudge`, since 2026-10-20): braking carry counts here, not above. The
    # ideal vehicle's airframe flies its command, so its side reads the
    # commanded count (`_RSP_FALLBACK`).
    ("true_p0_vehicle_ticks", "true_p0_vehicle_ticks"),
    ("repair_count", "repair_count"),
)
# A side whose tables do not carry the first key reads the second: the ideal
# vehicle's airframe velocity IS the command, so its vehicle count is its
# commanded count, and the comparison's two columns stay over one definition.
_RSP_FALLBACK = {"true_p0_vehicle_ticks": "true_p0_flown_ticks"}


def _rsp_value(k: dict, key: str):
    v = k.get(key)
    if v is None and key in _RSP_FALLBACK:
        v = k.get(_RSP_FALLBACK[key])
    return v or 0


def _rsp_family_side(items: list[dict]) -> dict[str, dict]:
    """Per family, one arm's side of the response comparison: statuses,
    the _RSP_SUM_FIELDS sums, episodes with each kind of entry, the deepest
    true distance to a fence, outcomes, and the labelled fail-safe score."""
    fams: dict[str, dict] = {}
    for r in items:
        k = r.get("kpi") or {}
        f = fams.setdefault(r["template"], {
            "episodes": 0, "status": {}, "outcomes": {},
            **{name: 0 for name, _ in _RSP_SUM_FIELDS},
            "episodes_with_unsafe_position_entry": 0,
            "episodes_entering_polygon": 0,
            "episodes_with_p0_escape": 0,
            "min_true_dist_to_fence_m": None,
            "failsafe_labelled": 0, "failsafe_correct": 0,
            "failsafe_false_triggers": 0, "failsafe_missed": 0})
        f["episodes"] += 1
        f["status"][r["status"]] = f["status"].get(r["status"], 0) + 1
        mo = k.get("mission_outcome")
        f["outcomes"][mo] = f["outcomes"].get(mo, 0) + 1
        for name, key in _RSP_SUM_FIELDS:
            f[name] += _rsp_value(k, key)
        f["episodes_with_unsafe_position_entry"] += int((k.get("breaches") or 0) > 0)
        f["episodes_entering_polygon"] += int((k.get("polygon_entries") or 0) > 0)
        f["episodes_with_p0_escape"] += int((k.get("p0_escapes") or 0) > 0)
        d = k.get("min_true_dist_to_fence_m")
        if d is not None:
            cur = f["min_true_dist_to_fence_m"]
            f["min_true_dist_to_fence_m"] = d if cur is None else min(cur, d)
        lab = k.get("failsafe_matches_label")
        if lab is not None:
            f["failsafe_labelled"] += 1
            f["failsafe_correct"] += int(lab == 1.0)
            f["failsafe_false_triggers"] += int(bool(k.get("failsafe_triggered"))
                                                and lab == 0.0)
            f["failsafe_missed"] += int(not k.get("failsafe_triggered") and lab == 0.0)
    return fams


# The per-family deltas the comparison prints (ardupilot minus ideal).
_RSP_DELTA_FIELDS = ("unsafe_position_entries", "episodes_with_unsafe_position_entry",
                     "polygon_entries", "episodes_entering_polygon",
                     "autopilot_unsafe_position_entries", "autopilot_polygon_entries",
                     "p0_escapes", "true_p0_flown_ticks", "true_p0_vehicle_ticks",
                     "failsafe_correct")


def response_comparison(headline: list[dict], rsp_scored: list[dict],
                        rsp_counts: dict, rsp_rows: list[dict],
                        arm: str = "ardupilot") -> dict:
    """The run's comparison vehicle `arm` (the ArduPilot response, or since
    2026-11-07 the rig's own: "rail") beside the ideal vehicle (module
    docstring, "THE VEHICLE RESPONSE"): per family, both arms' unsafe-
    position and true-state entry counts, P0 escapes and labelled fail-safe
    scores over the SAME (cell, seed) pairs, their difference, and every
    episode whose KPI table moved. The ideal side is restricted to the
    headline episodes this arm shadows, so a family's two columns count the
    same flights. Never merged into a headline number, `_counts` or the exit
    code: the arm is reported, not gated (its episodes are still judged
    against the scenario's gates and pins, and the verdicts are counted in
    `counts`)."""
    twins = {(r["id"], r["seed"]): r for r in headline}
    pairs = []
    unmatched = []
    for r in rsp_scored:
        g = twins.get((_twin_id(r["id"], f"response-{arm}"), r["seed"]))
        if g is None:
            unmatched.append(r["id"])
            continue
        pairs.append((g, r))
    # Shield-on episodes only, per family: a Shield-off control flies the raw
    # pilot into the zone on purpose, and pooling it would read as the
    # Shield's entries. The controls are totalled apart below.
    on_pairs = [(g, r) for g, r in pairs if g.get("arm", "on") == "on"]
    ideal_side = _rsp_family_side([g for g, _ in on_pairs])
    ap_side = _rsp_family_side([r for _, r in on_pairs])
    families = {}
    for fam in sorted(set(ideal_side) | set(ap_side)):
        a, b = ideal_side.get(fam) or {}, ap_side.get(fam) or {}
        families[fam] = {"ideal": a, arm: b,
                         "delta": {k: (b.get(k) or 0) - (a.get(k) or 0)
                                   for k in _RSP_DELTA_FIELDS}}
    moved = []
    for g, r in pairs:
        a, b = _cmp_side(g), _cmp_side(r)
        if a != b:
            moved.append({"id": g["id"], "seed": r["seed"], "family": r["template"],
                          "ideal": a, arm: b,
                          "max_velocity_lag_mps": (r.get("kpi") or {}).get(
                              "max_velocity_lag_mps")})
    totals = {}
    for side, items in (("ideal", [g for g, _ in pairs]), (arm, [r for _, r in pairs])):
        on = [x for x in items if x.get("arm", "on") == "on"]
        off = [x for x in items if x.get("arm", "on") != "on"]
        totals[side] = {
            "shield_on_episodes": len(on),
            **{name: sum(_rsp_value(x.get("kpi") or {}, key) for x in on)
               for name, key in _RSP_SUM_FIELDS},
            "episodes_with_unsafe_position_entry": sum(
                1 for x in on if ((x.get("kpi") or {}).get("breaches") or 0) > 0),
            "episodes_entering_polygon": sum(
                1 for x in on if ((x.get("kpi") or {}).get("polygon_entries") or 0) > 0),
            "failsafe_labels": failsafe_summary(items),
            "controls_shield_off": {
                "episodes": len(off),
                "unsafe_position_entries": sum((x.get("kpi") or {}).get("breaches") or 0
                                               for x in off),
                "polygon_entries": sum((x.get("kpi") or {}).get("polygon_entries") or 0
                                       for x in off)},
        }
    lags = [(r.get("kpi") or {}).get("max_velocity_lag_mps") for _, r in pairs]
    lags = [x for x in lags if x is not None]
    return {
        "arm": arm, "response": response_for(arm),
        "note": (f"the grant's Shield (its forecast unchanged) flown on the {arm} "
                 "vehicle response instead of the ideal vehicle, for "
                 "comparison only: never in _counts, _failsafe_labels, _true_state, "
                 "the top failures or the exit code. Family rows and totals count "
                 "shield-on episodes (the fail-safe labels every labelled one); "
                 "the shield-off controls are totalled apart. true_p0_flown_ticks "
                 "judges the COMMANDED action at the true state, true_p0_vehicle_ticks "
                 "the velocity the airframe flew there (braking carry counts only in "
                 "the second; on the ideal vehicle the two are one count)."),
        "counts": rsp_counts,
        "paired_episodes": len(pairs),
        "unmatched": unmatched,
        "totals": totals,
        "true_state": true_state_summary([r for _, r in pairs]),
        "outcomes": {o: sum(1 for _, r in pairs
                            if (r.get("kpi") or {}).get("mission_outcome") == o)
                     for o in K.OUTCOMES},
        "families": families,
        # The arm ran: how far the airframe's velocity fell behind its
        # command (a response arm with every lag 0.0 would be the ideal
        # vehicle under another name).
        "max_velocity_lag_mps": max(lags) if lags else None,
        "episodes_moved": len(moved),
        "moved": moved,
        "results": rsp_rows,
    }


# Fields of a KPI table that describe what HAPPENED. Two cells with the same
# signature flew the same table; rounding keeps float noise in positions from
# splitting them, while repair counts are kept exact (they are what differ
# between near-symmetric copies, so the count is an UPPER bound on variety).
_SIGNATURE = (("status", None), ("p0_escapes", None), ("repair_count", None),
              ("breaches", None), ("failsafe_triggered", None),
              ("mission_outcome", None),
              ("reached_goal", None), ("time_to_safe_episodes", None),
              ("time_to_safe_censored", None), ("min_true_dist_to_fence_m", 2),
              ("ticks_inside_fence_polygon", None), ("polygon_entries", None),
              ("true_p0_flown_ticks", None),
              ("max_speed_flown", 2), ("min_range_to_subject", 2),
              ("max_offset_from_corridor", 2))


def kpi_signature(item: dict) -> tuple:
    k = item.get("kpi") or {}
    out = []
    for name, nd in _SIGNATURE:
        v = item.get(name) if name == "status" else k.get(name)
        if nd is not None and isinstance(v, float):
            v = round(v, nd)
        out.append(v)
    return tuple(out)


def preflight_refusals(info: dict, profiled: bool, narrowed: bool) -> list[str]:
    """Reasons to refuse a run BEFORE flying anything (exit 2).

    Each is a way for a sweep to print a clean result while testing nothing,
    or less than it claims: an `--only` name that matched no cell (a typo, or
    a cell id this profile does not expand), zero episodes, two cells that fly
    the same episode, or a full profile that no longer has the count it is
    named for. `narrowed` is an explicit --only, the one deliberate way to run
    part of a profile.
    """
    why = []
    if info.get("only_unmatched"):
        why.append(f"--only matched no cell for {info['only_unmatched']} "
                   f"(names are a family id or a cell id this run expands)")
    if not info.get("episodes"):
        why.append("the run has 0 episodes")
    if info.get("duplicate_cells"):
        dups = info["duplicate_cells"]
        why.append(f"{len(dups)} cell(s) fly the same episode as an earlier cell, "
                   f"which would count one flight as two scenarios: {dups[:3]}")
    if profiled and not narrowed and info.get("cells_within_expected") is False:
        lo, hi = info["expected_cells"]
        why.append(f"{info['cells']} scenarios is outside the profile's expected "
                   f"{lo}-{hi} ({info.get('grant_scope')})")
    return why


# --------------------------------------------------------------------------- #
# manifests and bundles
# --------------------------------------------------------------------------- #

MANIFEST_FIELDS = ("code_revision", "vla_model_hash", "policy_hash",
                   "random_seed", "sim_speedup", "topology")


def manifest_base() -> dict:
    """The fields shared by every episode of one sweep, resolved ONCE: the code
    revision shells out to git, which costs seconds on this tree."""
    from guardrail.manifest import build_manifest
    m = build_manifest(policy_hash="", model_id=PILOT_ID, seed=0,
                       topology=TOPOLOGY_HEADLESS, sim_speedup=None)
    return {"code_revision": m["code_revision"],
            "vla_model_hash": m["vla_model_hash"], "topology": TOPOLOGY_HEADLESS}


def episode_manifest(base: dict, policy_hash: str, seed: int,
                     sim_s: float, wall_s: float) -> dict:
    """The grant's six fields, in its order, for one episode.

    `policy_hash` is the policy as injected at t = 0; every hot-apply
    generation after it is in harness_events.jsonl with its own hash.
    `sim_speedup` is measured - simulated seconds over wall seconds - never a
    literal: a hand-written 1.0 is exactly what a broken run would also carry.
    """
    speed = round(sim_s / wall_s, 1) if wall_s > 0 else "unresolved"
    m = {"code_revision": base["code_revision"],
         "vla_model_hash": base["vla_model_hash"],
         "policy_hash": policy_hash or "unresolved",
         "random_seed": int(seed),
         "sim_speedup": speed,
         "topology": base["topology"]}
    assert tuple(m) == MANIFEST_FIELDS
    return m


def write_bundle(ep_dir: Path, manifest: dict, kpi_doc: dict, events: list[dict],
                 rows: list[dict] | None, extras: dict | None = None,
                 audit: list[dict] | None = None) -> None:
    """The episode bundle. `audit` (EpisodeRun.audit) is written as
    shield_audit.jsonl, the grant's name for the Shield's audit log in a
    bundle (Stress Testing), even when it is empty: an empty file says the
    Shield logged nothing, a missing one would say nobody looked."""
    ep_dir.mkdir(parents=True, exist_ok=True)
    (ep_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n",
                                          encoding="utf-8")
    if extras:
        # Beside the manifest, never in it: the grant's manifest is six fields.
        (ep_dir / "manifest_extras.json").write_text(
            json.dumps(extras, indent=2) + "\n", encoding="utf-8")
    (ep_dir / "kpi.json").write_text(json.dumps(kpi_doc, indent=2, allow_nan=False)
                                     + "\n", encoding="utf-8")
    (ep_dir / "harness_events.jsonl").write_text(
        "".join(json.dumps(_json_safe(e)) + "\n" for e in events), encoding="utf-8")
    if rows is not None:
        (ep_dir / "flight_log.jsonl").write_text(
            "".join(json.dumps(_json_safe(r), allow_nan=False) + "\n" for r in rows),
            encoding="utf-8")
    if audit is not None:
        (ep_dir / "shield_audit.jsonl").write_text(
            "".join(json.dumps(_json_safe(a)) + "\n" for a in audit), encoding="utf-8")


def runner_facts() -> dict:
    """The interpreter this sweep ran under (WP1-25 / X-06: the grant's non-VLA
    stack is Python 3.11+). On a hil campaign the Shield node records the
    companion's own Python in each run's metrics; this is the desktop side
    that drives the campaign and scores it."""
    import platform
    return {"python": platform.python_version(),
            "implementation": platform.python_implementation(),
            "env": Path(sys.prefix).name,
            "platform": platform.platform(terse=True)}


def _show(p: Path) -> str:
    """Repo-relative when it can be, always with forward slashes. Used to call
    relative_to unconditionally, which raised AFTER the results were written
    whenever --out pointed outside the repository, so a successful sweep
    exited with a traceback. OS-native separators put `demo\\out\\sweep\\...`
    into the published JSON and kpi_report.md on Windows, so a republish on
    Linux or CI would have changed every bundle link."""
    try:
        return Path(p).resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return PurePath(p).as_posix()


# --------------------------------------------------------------------------- #
# the run
# --------------------------------------------------------------------------- #

def _profile_path(name: str) -> Path:
    p = Path(name)
    if not p.suffix:
        return PROFILES / f"{name}.yaml"
    return p if p.is_absolute() else ROOT / p


def _resolve_profile(name: str | None):
    if not name:
        return None
    return load_profile(_profile_path(name))


# --------------------------------------------------------------------------- #
# resume and shards (WP4-06, 2026-11-17; module docstring, "UNATTENDED RUNS")
# --------------------------------------------------------------------------- #

CHECKPOINT_FORMAT = 1
CHECKPOINT_HEADER = "checkpoint.json"
PROGRESS_EVERY_S = 60.0


def parse_shard(text: str | None) -> tuple[int, int] | None:
    """`--shard I/N` -> (I, N) with 1 <= I <= N; None without the flag.
    Episode k (0-based, in expand()'s order) belongs to shard (k % N) + 1, so
    every shard draws from every family and the shards take about as long."""
    if text is None:
        return None
    m = re.fullmatch(r"\s*(\d+)\s*/\s*(\d+)\s*", text)
    if not m or not (int(m[2]) >= 1 and 1 <= int(m[1]) <= int(m[2])):
        raise ValueError(f"--shard {text!r} must be I/N with 1 <= I <= N (e.g. 1/4)")
    return int(m[1]), int(m[2])


def in_shard(k: int, shard: tuple[int, int] | None) -> bool:
    return shard is None or k % shard[1] == shard[0] - 1


_RESUME_OPTS = {"--resume": True, "--shard": True, "--merge-only": False}   # takes a value


def strip_resume_args(argv: list[str]) -> list[str]:
    """`argv` less --resume DIR, --shard I/N and --merge-only (spelled in full,
    with '=', or abbreviated as argparse allows): the one uninterrupted run
    that a resumed, sharded or merged run equals."""
    out: list[str] = []
    i = 0
    while i < len(argv):
        tok = argv[i]
        opt, eq, _ = tok.partition("=")
        name = next((n for n in _RESUME_OPTS
                     if opt.startswith("--") and len(opt) > 2 and n.startswith(opt)), None)
        if name is None:
            out.append(tok)
            i += 1
            continue
        i += 1 + (1 if _RESUME_OPTS[name] and not eq else 0)
    return out


def episode_key(ep) -> str:
    """An episode's checkpoint name: its run id (cell and arm) and seed,
    hashed short so a long cell id never meets Windows' path-length limit.
    The id itself is inside the checkpoint."""
    return hashlib.sha1(f"{ep.run_id}|seed={ep.seed}".encode("utf-8")).hexdigest()[:20]


def atomic_write_text(path: Path, text: str) -> None:
    """Write `text` to `path` so a reader sees the old file or the whole new
    one, never a torn one: a temporary file beside it, flushed and fsynced,
    then os.replace. A replace refused for a moment (OneDrive or an indexer
    holding the target, PermissionError on Windows) is retried."""
    tmp = path.with_name(f"{path.name}.{os.getpid()}.tmp")
    with open(tmp, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
        fh.flush()
        os.fsync(fh.fileno())
    for attempt in range(6):
        try:
            os.replace(tmp, path)
            return
        except PermissionError:
            if attempt == 5:
                raise
            time.sleep(0.25 * (attempt + 1))


def create_exclusive_text(path: Path, text: str) -> bool:
    """Create `path` holding `text` only if nothing is there yet: True if this
    process created it, False if another one did (now or earlier). Shards
    started side by side into a fresh folder all try at once, and exactly one
    wins. A hard link of a complete, fsynced temporary file is the create, so
    a reader never sees a half-written file; on a disk without hard links
    (FAT, exFAT) the file is created with O_EXCL and then written, and a
    reader that finds it empty or torn retries (CheckpointStore.open)."""
    tmp = path.with_name(f"{path.name}.{os.getpid()}.{uuid.uuid4().hex[:8]}.tmp")
    with open(tmp, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
        fh.flush()
        os.fsync(fh.fileno())
    try:
        try:
            os.link(tmp, path)
            return True
        except FileExistsError:
            return False
        except OSError:
            pass                                    # no hard links on this disk
        try:
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        except FileExistsError:
            return False
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(text)
            fh.flush()
            os.fsync(fh.fileno())
        return True
    finally:
        try:
            os.unlink(tmp)
        except OSError:
            pass


def _sha256_file(p: Path | None) -> str | None:
    return hashlib.sha256(p.read_bytes()).hexdigest() if p and p.is_file() else None


def checkpoint_fingerprint(base: dict, lib_path: Path, profile_path: Path | None,
                           options: dict, episodes) -> dict:
    """What a checkpoint folder belongs to. Two runs whose fingerprints differ
    fly different sweeps, and their episodes are never merged. A dirty tree's
    code revision ends '-dirty' whatever the change, so between shards on a
    dirty tree only the library, the profile and the episode list are
    checked (the header says so)."""
    keys = "\n".join(f"{e.run_id}|seed={e.seed}" for e in episodes)
    return {"format": CHECKPOINT_FORMAT,
            "code_revision": base["code_revision"],
            "library": _show(lib_path), "library_sha256": _sha256_file(lib_path),
            "profile": _show(profile_path) if profile_path else None,
            "profile_sha256": _sha256_file(profile_path),
            "options": options,
            "episodes": len(episodes),
            "episode_list_sha256": hashlib.sha256(keys.encode("utf-8")).hexdigest()}


class CheckpointStore:
    """One folder of per-episode checkpoints (`--resume DIR`)."""

    def __init__(self, folder: Path):
        self.folder = Path(folder)
        self.episodes_dir = self.folder / "episodes"
        self.header: dict = {}
        # When the newest checkpoint this process knows of was written (the
        # files it loaded, the ones it saved): the end of the sweep's span.
        self.last_checkpoint_epoch: float | None = None

    def open(self, fingerprint: dict, stamp: str) -> str | None:
        """Create the header, or check an existing one against `fingerprint`.
        Returns a refusal, or None. The header's bundle stamp is reused, so a
        resumed run's bundles are one run's (an episode killed mid-bundle is
        rewritten in the same folder, not left beside a second copy). The
        header is created exclusively (create_exclusive_text): of shards
        started together into a fresh folder one writes it, and every other
        reads it back and flies under its stamp."""
        hp = self.folder / CHECKPOINT_HEADER
        self.episodes_dir.mkdir(parents=True, exist_ok=True)
        mine = {
            "fingerprint": fingerprint, "stamp": stamp,
            "created_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "dirty_tree": str(fingerprint.get("code_revision", "")).endswith("-dirty"),
            "note": "per-episode checkpoints of one sweep (experiments/sweep_scenarios.py "
                    "--resume); each file is one finished episode, written atomically"}
        if create_exclusive_text(hp, json.dumps(mine, indent=2) + "\n"):
            self.header = mine
            return None
        old, err = None, None
        for attempt in range(20):           # a winner without hard links may be mid-write
            try:
                old = json.loads(hp.read_text(encoding="utf-8"))
                break
            except (OSError, ValueError) as e:
                err = e
                time.sleep(0.25)
        if not isinstance(old, dict):
            return f"{_show(hp)} is unreadable ({err}); move the folder aside"
        have = old.get("fingerprint") or {}
        diff = sorted(k for k in set(have) | set(fingerprint)
                      if have.get(k) != fingerprint.get(k))
        if diff:
            return (f"{_show(self.folder)} holds checkpoints of another sweep (differs "
                    f"in: {', '.join(diff)}); use a new --resume folder")
        self.header = old
        return None

    @property
    def stamp(self) -> str:
        return self.header["stamp"]

    def load(self, keys: set[str] | None = None) -> tuple[dict[str, dict], list[str]]:
        """{key: record} of every readable checkpoint, and the names of the
        unreadable ones (re-flown, never trusted). With `keys`, only those
        checkpoints are read: a shard leaves the other shards' records on
        disk (about 61 MB of memory and 7 s per 1000 records it would never
        use, measured on the broad profile 2026-11-17)."""
        done: dict[str, dict] = {}
        bad: list[str] = []
        if not self.episodes_dir.is_dir():
            return done, bad
        for p in self.episodes_dir.glob("*.json"):
            if keys is not None and p.stem not in keys:
                continue
            try:
                rec = json.loads(p.read_text(encoding="utf-8"))
                if rec.get("format") != CHECKPOINT_FORMAT or rec.get("key") != p.stem:
                    raise ValueError("not a checkpoint of this format")
            except (OSError, ValueError) as e:
                bad.append(f"{p.name} ({type(e).__name__})")
                continue
            done[p.stem] = rec
            try:
                self._saw(p.stat().st_mtime)
            except OSError:
                pass
        return done, bad

    def _saw(self, epoch: float) -> None:
        if self.last_checkpoint_epoch is None or epoch > self.last_checkpoint_epoch:
            self.last_checkpoint_epoch = epoch

    def save(self, rec: dict) -> None:
        # Plain json (NaN allowed): the record must replay exactly; the
        # results JSON written from it is made safe as it always was.
        self.episodes_dir.mkdir(parents=True, exist_ok=True)
        atomic_write_text(self.episodes_dir / f"{rec['key']}.json", json.dumps(rec) + "\n")
        self._saw(time.time())

    def wall_span(self) -> dict:
        """The sweep's wall span: from the header's creation (the folder's
        first run) to the newest checkpoint, pauses between runs included."""
        out = {"sweep_started_utc": self.header.get("created_utc"),
               "last_checkpoint_utc": None, "sweep_wall_span_s": None}
        if self.last_checkpoint_epoch is None:
            return out
        last = datetime.fromtimestamp(self.last_checkpoint_epoch, timezone.utc)
        out["last_checkpoint_utc"] = last.isoformat(timespec="seconds")
        try:
            start = datetime.fromisoformat(str(self.header.get("created_utc")))
            out["sweep_wall_span_s"] = round(max(0.0, (last - start).total_seconds()), 1)
        except ValueError:
            pass
        return out


def _fmt_s(s: float) -> str:
    return f"{s / 3600:.1f} h" if s >= 3600 else f"{s / 60:.1f} min" if s >= 60 else f"{s:.0f} s"


def _json_safe(node):
    """A worst-tick record quotes the RAW action, and nonfinite-action's raw
    action is NaN by design. The results file is written with allow_nan=False
    (see below), so a quoted NaN becomes the string "nan" here rather than
    either crashing the write or vanishing."""
    if isinstance(node, dict):
        return {k: _json_safe(v) for k, v in node.items()}
    if isinstance(node, (list, tuple)):
        return [_json_safe(v) for v in node]
    if isinstance(node, float) and not math.isfinite(node):
        return str(node)
    return node


PUBLISHED_DROPS = ("arm_record", "manifest")


def published_view(doc: dict) -> dict:
    """The copy written to docs/data/scenario_sweep.json.

    Per episode it drops `arm_record` (the paraphrase text and its provenance,
    repeated from `manifest_extras`) and `manifest` (`_manifest_common` plus the
    seed and scenario id the row already carries). Both stay in the episode's
    bundle, which `bundle` names, and in the full run file. It saves about
    0.1 MB of 1.9 MB: the file grew from 25 kB (13 results) because the library
    now holds 195 episodes, each with its full KPI table, plus one rollup per
    (template, cell). Every field a consumer reads (`_counts`, `id`, `status`,
    `why`, `kpi`, the paraphrase id in `manifest_extras`) is kept."""
    out = {k: v for k, v in doc.items() if k != "results"}
    out["_published_drops"] = {
        "fields": list(PUBLISHED_DROPS),
        "note": "per-episode fields left out of the published copy; the episode "
                "bundle and the full run file keep them"}
    out["results"] = [{k: v for k, v in r.items() if k not in PUBLISHED_DROPS}
                      for r in doc.get("results") or []]
    cmp_ = doc.get("_forecast_comparison")
    if cmp_ and "results" in cmp_:
        # The comparison arm's per-episode items double the file; its
        # per-cell deltas, rollups and summaries stay (since 2026-10-12).
        out["_forecast_comparison"] = {k: v for k, v in cmp_.items() if k != "results"}
        out["_published_drops"]["comparison"] = (
            "_forecast_comparison.results (the 3 s arm's per-episode items) is in "
            "the full run file and the bundles only")
    rcmp = doc.get("_response_comparison")
    if rcmp and "results" in rcmp:
        # --publish refuses the response arm today; should that change, its
        # per-episode items stay out of the published copy like the 3 s arm's.
        out["_response_comparison"] = {k: v for k, v in rcmp.items() if k != "results"}
        out["_published_drops"]["response_comparison"] = (
            "_response_comparison.results is in the full run file and the bundles only")
    return out


def comparison_tag(forecast_arms, response_arms,
                   wind_model: str = DEFAULT_WIND_MODEL,
                   autopilot_hold: str = DEFAULT_AUTOPILOT_HOLD,
                   recovery: str = DEFAULT_RECOVERY_LAW) -> str | None:
    """The comparison configuration(s) a run flies, as the labels its KPI rows
    carry ("forecast-3s", "response-ardupilot", "wind-open_loop", joined by
    "_"), or None for a run of the headline configuration alone. It names the
    run's default folder (run_folder). "hold-in_place" (since the merge with
    the hold points, 2026-10-21) is a run whose modelled autopilot holds
    where the aircraft is (`--autopilot-hold in_place`): its rows carry it
    as the run's `autopilot_hold_model`, not in their key, and until then
    such a run wrote into stress_<profile> beside the headline bundles.
    "recovery-lookahead" (2026-10-29) is a run on the Shield's old recovery
    law (`--recovery-law lookahead`), carried the same way (`recovery_law`)."""
    tags = ([f"forecast-{a}" for a in forecast_arms if a != "grant"]
            + ([f"wind-{wind_model}"] if wind_model != DEFAULT_WIND_MODEL else [])
            + [f"response-{a}" for a in response_arms if a != DEFAULT_RESPONSE_ARM]
            + ([f"hold-{autopilot_hold}"] if autopilot_hold != DEFAULT_AUTOPILOT_HOLD
               else [])
            + ([f"recovery-{recovery}"] if recovery != DEFAULT_RECOVERY_LAW else []))
    return "_".join(tags) or None


def run_folder(tag: str | None, comparison: str | None = None) -> Path:
    """The default folder of a run's results JSON, bundles and kpi_report.md:
    demo/out/sweep (the library) or demo/out/stress_<profile>, and for a run
    that flies a comparison configuration (comparison_tag) demo/out/
    cmp_<profile or "library">_<label>.

    Outside `demo/out/stress_*` on purpose (review of T1b-harness-vehicle,
    P1): tools/kpi_report.py reads `demo/out/stress_*/episode-*` by default
    (DEFAULT_RUNS), and until 2026-10-20 a profiled run with
    `--response-arms ideal ardupilot` (or `--forecast-arms grant 3s`) wrote
    its comparison copies into stress_<profile> beside the headline bundles,
    where every later default report read them; a library run's would also
    have replaced the headline run's demo/out/sweep/sweep.json. The ideal
    vehicle, the grant forecast and the guided wind - every default
    invocation, and every profile as shipped (none declares a comparison
    arm) - keep the folder they always had."""
    if comparison:
        return ROOT / "demo" / "out" / f"cmp_{tag or 'library'}_{comparison}"
    return ROOT / "demo" / "out" / (f"stress_{tag}" if tag else "sweep")


def results_path(out: str | None, publish: bool, tag: str | None,
                 comparison: str | None = None) -> Path:
    """Where the results JSON goes. The published artefact only on --publish:
    a routine re-run used to replace docs/data/scenario_sweep.json, which the
    evaluation data, the deck and a pinned test read. Otherwise `out`, else
    run_folder's sweep.json (a comparison run's own folder)."""
    if publish:
        return PUBLISHED_SWEEP
    if out:
        return Path(out)
    return run_folder(tag, comparison) / "sweep.json"


def _rollup(eps: list[dict]) -> dict | None:
    """guardrail.kpi.rollup when this checkout has it (it landed on 2026-10-06);
    None for no episodes, or when it is missing - and then the per-cell
    rollup in main() fails, which is recorded in `_rollup_note`, printed, and
    exits 4 (main(), "ALL OR NOTHING")."""
    fn = getattr(K, "rollup", None)
    return fn(eps) if (fn and eps) else None


def cell_key(table: dict) -> str:
    """The grant's KPI-report row, "(scenario template, parameter cell)", plus
    the Shield arm and the prefix arm, which are different experiments.
    Seeds and paraphrase arms of one cell pool in its row. The 3 s forecast
    comparison arm is its own row, `/forecast-3s`, marked non-grant; the
    ArduPilot vehicle-response arm likewise, `/response-ardupilot` (since
    2026-10-19). A headline row's key is what it always was."""
    key = f"{table['family']}/{table['cell']}/shield-{table['arm']}"
    if table.get("prefix_consumer"):
        # A prefix-consumer run's rows (2026-11-16): another pilot than the
        # cell's own, and its two arms are two experiments.
        key += f"/prefix-consumer-{table.get('prefix')}"
    elif table.get("prefix") == "off":
        key += "/prefix-off"
    if table.get("forecast_arm", "grant") != "grant":
        key += f"/forecast-{table['forecast_arm']}"
    if table.get("response_arm", "ideal") != "ideal":
        key += f"/response-{table['response_arm']}"
    return key


class _NoBeat:
    def beat(self, *a, **k):
        return None

    def stop(self, *a, **k):
        return None


def _live_heartbeat(name: str, **kw):
    """A heartbeat for the tracker's Dashboard "running now" panel
    (tools/live_status.py), or a no-op when anything goes wrong: beat() only
    stores numbers in memory and a daemon thread writes at most once a second,
    outside OneDrive, swallowing its own errors. Only a command-line run gets
    one (the __main__ block): main() called from a test keeps the no-op."""
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "vlaguard_live_status", ROOT / "tools" / "live_status.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.heartbeat("harness", name, **kw)
    except Exception:                                   # noqa: BLE001
        return _NoBeat()


_LIVE = _NoBeat()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--library", default=None,
                    help="scenario library (default: the profile's, else "
                         "experiments/scenarios.yaml)")
    ap.add_argument("--profile", default=None,
                    help="smoke | nightly | broad | path to a profile YAML")
    ap.add_argument("--seed", type=int, default=None,
                    help="run every episode on this one seed")
    ap.add_argument("--backend", choices=("headless",) + RAIL_BACKENDS, default="headless",
                    help="headless (the kinematic harness), sitl (this desktop's "
                         "ArduPilot SITL + MAVROS 2 rail, sitl/run_ros2_demo.sh) or hil "
                         "(SITL on this desktop, the mission on the Jetson Orin: "
                         "docs/RUNBOOK-orin-hil.md step 14)")
    ap.add_argument("--hil-config", default=None, metavar="FILE",
                    help="--backend hil: the rig (JSON or YAML; HilConfig's keys: "
                         "ssh_target, remote_root, env_file, wsl_distro, mavros_host, "
                         "store, bundles_dir, ...); default the AGX Orin over the cable")
    ap.add_argument("--hil-dry-run", action="store_true",
                    help="--backend hil: list every episode - the mission it would fly "
                         "(tag, SITL_HOME, the nodes' arguments) or why it is refused - "
                         "and stop; nothing is flown or written")
    ap.add_argument("--hil-controls", action="store_true",
                    help="--backend hil: also fly the Shield-off control arms")
    ap.add_argument("--hil-campaign", default=None, metavar="NAME",
                    help="--backend hil: the campaign's name, the prefix of every "
                         "mission tag (default hil<UTC stamp>)")
    ap.add_argument("--hil-max-s-add", default=None, metavar="SECONDS",
                    help="--backend hil: a LENGTHENED campaign - every mission flies "
                         "SECONDS past the rail's cap (the scenario's max_s + the Loiter "
                         "operator timeout + 5 s), so a cell censored in Loiter can show "
                         "its fail-safe outcome (docs/RUBRIC-failsafe.md section 3); 35 "
                         "gives a Loiter entered at the rail's cap its timeout and margin "
                         "again. Recorded in sweep.json and every mission's scenario "
                         "record; a separate campaign, never merged into the profile's "
                         f"KPI figure (at most {HIL_MAX_S_ADD_MAX:g} s)")
    ap.add_argument("--rail-gates", choices=RAIL_GATE_BASES, default="ideal",
                    help="the gates a rail run's verdict uses: ideal (the cell's own, "
                         "the default) or, PROPOSED, NOT ADOPTED, each vehicle-pinned "
                         "bound moved by the cell's headless ArduPilot-response flight "
                         "(rail_gates_ardupilot), entering mid-flight (ardupilot) or "
                         "from a hover as a rail starts (ardupilot-rest); the ideal "
                         "verdict is kept beside it")
    ap.add_argument("--rail-arc-forecast", choices=("on", "off"), default="off",
                    help="--backend sitl / hil (2026-11-17, WP3-04 / ARCH-04): on hands "
                         "every mission's Shield node --arc-forecast=on (a body action "
                         "with a yaw rate forecast along its arc as well as its line), and "
                         "a run whose metrics.json shield_forecast.arc is not true is an "
                         "error, never scored (a node from an older bundle drops the "
                         "flag). off (default): nothing is added. Recorded in sweep.json; "
                         "a separate campaign, never merged with a straight-forecast one")
    ap.add_argument("--rescore-rail", nargs="+", default=None, metavar="RUN_GLOB",
                    help="re-score fetched rail runs (hil / sitl run folders with a "
                         "scenario record) with this checkout's scorer, against the "
                         "profile's or library's cells; writes nothing beside the runs "
                         "- the before / after table goes to --out (required)")
    ap.add_argument("--rail-labels", default=None, metavar="JSON",
                    help="with --rescore-rail: the fail-safe label artefact its "
                         "correctness table reads (default docs/data/"
                         "failsafe_labels_rail.json, tools/failsafe_rubric.py "
                         "--build-labels)")
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--out", default=None,
                    help="results JSON (default: demo/out/sweep/sweep.json, or "
                         "demo/out/stress_<profile>/sweep.json; a run with a "
                         "comparison arm or wind model: demo/out/"
                         "cmp_<profile|library>_<label>/sweep.json)")
    ap.add_argument("--publish", action="store_true",
                    help="write the library run to docs/data/scenario_sweep.json, "
                         "the PUBLISHED artefact (needs a CHANGELOG entry)")
    ap.add_argument("--runs", default=None,
                    help="where episode bundles and kpi_report.md go (default: "
                         "demo/out/sweep, demo/out/stress_<profile>, or for a "
                         "comparison run demo/out/cmp_<profile|library>_<label>)")
    ap.add_argument("--no-bundles", action="store_true",
                    help="skip the per-episode bundle folders")
    ap.add_argument("--keep-logs", action="store_true",
                    help="also write each episode's flight_log.jsonl, so "
                         "tools/kpi_report.py can re-score the bundles")
    ap.add_argument("--forecast-arms", nargs="+", choices=FORECAST_ARMS, default=None,
                    help="grant (always) and optionally 3s, the labelled "
                         "comparison with the pre-2026-10-12 forecast (default: "
                         "the profile's, else grant; --publish flies both)")
    ap.add_argument("--wind-model", choices=WIND_MODELS, default=DEFAULT_WIND_MODEL,
                    help="guided (ArduCopter's position controller, the default) "
                         "or open_loop (the model until 2026-10-12, for comparison)")
    ap.add_argument("--autopilot-hold", choices=AUTOPILOT_HOLD_MODELS,
                    default=DEFAULT_AUTOPILOT_HOLD,
                    help="what the modelled autopilot flies for the FSM's Loiter / "
                         "Land: moved_points (the FSM's hold points, the default; "
                         "modelled, no flight rail flies them yet) or in_place "
                         "(ArduPilot's own hold where the aircraft is: what the "
                         "rails fly, and the flights before 2026-10-19)")
    ap.add_argument("--response-arms", nargs="+", choices=RESPONSE_ARMS, default=None,
                    help="ideal (always) and optionally ardupilot, the labelled "
                         "comparison flown on the measured ArduPilot vehicle "
                         "response (default: the profile's, else ideal)")
    ap.add_argument("--recovery-law", choices=RECOVERY_LAWS, default=DEFAULT_RECOVERY_LAW,
                    help="how the Shield recovers from an illegal position: cap_all "
                         "(every recovery at its cap, the default since 2026-11-06), "
                         "cap (the altitude band only, 2026-10-29 to 2026-11-06) or "
                         "lookahead (the band in one lookahead, before; for "
                         "comparison)")
    ap.add_argument("--resume", default=None, metavar="DIR",
                    help="headless only: save each finished episode as a checkpoint in "
                         "DIR and replay DIR's checkpoints instead of flying them again "
                         "(a killed run resumes); DIR is refused if it holds another "
                         "sweep's checkpoints")
    ap.add_argument("--shard", default=None, metavar="I/N",
                    help="with --resume: fly only the I-th of every N episodes (1-based) "
                         "and write checkpoints only; merge with --merge-only")
    ap.add_argument("--merge-only", action="store_true",
                    help="with --resume: fly nothing; refused while any episode has no "
                         "checkpoint, else write the results JSON and kpi_report.md")
    ap.add_argument("--rail-physics", choices=RAIL_PHYSICS, default=None,
                    help="--backend sitl: the physics behind SITL - sitl (its own "
                         "quad, the default) or gazebo (Gazebo Harmonic, the grant's "
                         "functional rail: sitl/run_ros2_demo.sh PHYSICS=gazebo; every "
                         "mission marked functional, never KPI-grade). Default: the "
                         "profile's rail_physics, else sitl")
    ap.add_argument("--rail-sim-speedup", type=float, default=None, metavar="N",
                    help="--backend sitl: the simulator's speed. Only 1 is accepted, "
                         "on every rail: " + RAIL_SIM_SPEEDUP_WHY.replace("%", "%%")
                         + ". Default: the profile's rail_sim_speedup, else 1")
    ap.add_argument("--sitl-dry-run", action="store_true",
                    help="--backend sitl: list every episode - the mission it would fly "
                         "(tag, SITL_HOME, environment, command, the run folder the "
                         "rail writes, the grant's episode name as a label) or why it is "
                         "refused - with a wall-time estimate, and stop; needs neither "
                         "WSL nor Gazebo, flies and writes nothing")
    args = ap.parse_args(argv)

    try:
        shard = parse_shard(args.shard)
    except ValueError as e:
        print(f"REFUSED: {e}")
        return 2
    if (shard or args.merge_only) and not args.resume:
        print("REFUSED: --shard and --merge-only work on a checkpoint folder (--resume DIR)")
        return 2
    if shard and args.merge_only:
        print("REFUSED: --merge-only flies nothing; a shard flies its part (pick one)")
        return 2
    if args.resume and (args.backend != "headless" or args.publish
                        or args.rescore_rail is not None):
        # The rails have their own campaign stores and fetched runs; the
        # published file is one uninterrupted library run.
        print("REFUSED: --resume is for a headless sweep (not --backend sitl / hil, "
              "--publish or --rescore-rail)")
        return 2

    if args.publish and (args.out or args.profile or args.only or args.seed is not None
                         or args.forecast_arms is not None
                         or args.wind_model != DEFAULT_WIND_MODEL
                         or args.autopilot_hold != DEFAULT_AUTOPILOT_HOLD
                         or args.response_arms is not None
                         or args.recovery_law != DEFAULT_RECOVERY_LAW):
        # The published file is the WHOLE library on its declared seeds; a
        # partial or profiled run written there would replace it with less.
        # The response arm is not published either: docs/data/
        # scenario_sweep.json is the headline library run, and replacing it
        # would need a CHANGELOG entry for a comparison nobody asked to ship.
        print("REFUSED: --publish writes the full library run; it takes no "
              "--out, --profile, --only, --seed, --forecast-arms, --wind-model "
              "--autopilot-hold, --response-arms or --recovery-law")
        return 2

    t_run = time.perf_counter()
    profile = _resolve_profile(args.profile)
    lib_path = Path(args.library or (profile.library if profile else LIB))
    if not lib_path.is_absolute():
        lib_path = ROOT / lib_path
    lib = Library(lib_path)
    # The battery cell on the rails is the profile's opt-in alone
    # (RAIL_BATTERY_OPT_IN): False for every profile but fsm_edges.
    rail_battery = bool(profile is not None and profile.rail_battery)
    fc_arms = list(PUBLISH_FORECAST_ARMS) if args.publish else args.forecast_arms
    rsp_arms = args.response_arms
    # The arms this run will fly: the flags, else the profile's, else the
    # headline arm alone (expand() resolves them the same way).
    eff_rsp = (rsp_arms if rsp_arms is not None
               else (profile.response_arms if profile else ["ideal"]))
    # The run's comparison vehicle, when it flies one (check_response_arms
    # allows one beside "ideal"): "ardupilot", or "rail" since 2026-11-07.
    rsp_arm = next((a for a in eff_rsp if a != "ideal"), "ardupilot")
    try:
        if fc_arms is not None:
            check_forecast_arms(fc_arms)
        check_response_arms(eff_rsp)
        check_comparison_arms(fc_arms if fc_arms is not None
                              else (profile.forecast_arms if profile else ["grant"]),
                              eff_rsp)
    except ValueError as e:
        print(f"REFUSED: {e}")
        return 2
    if args.backend in RAIL_BACKENDS and fc_arms and "3s" in fc_arms:
        # The rail's node builds its own Shield; the arm is a headless one.
        print(f"REFUSED: the 3 s forecast arm is a headless comparison; the "
              f"{args.backend} rail flies the node's own Shield")
        return 2
    if args.backend in RAIL_BACKENDS and args.autopilot_hold != DEFAULT_AUTOPILOT_HOLD:
        # Likewise: the rail's ArduPilot flies its own hold (in place).
        print(f"REFUSED: --autopilot-hold chooses the headless harness's modelled "
              f"autopilot; the {args.backend} rail flies ArduPilot's own hold")
        return 2
    if args.backend in RAIL_BACKENDS and args.recovery_law != DEFAULT_RECOVERY_LAW:
        # Likewise: the rail's node builds its own Shield, on the default law.
        print(f"REFUSED: --recovery-law chooses the headless harness's Shield; the "
              f"{args.backend} rail flies the node's own Shield")
        return 2
    if args.backend in RAIL_BACKENDS and any(a != "ideal" for a in eff_rsp):
        # The rail flies ArduPilot itself: its vehicle response is the real
        # one, and a modelled one on top would fly the response twice.
        print(f"REFUSED: the ardupilot response arm is a headless model of what "
              f"the {args.backend} rail flies for real")
        return 2
    if args.backend == "hil" and args.publish:
        print("REFUSED: --publish writes the headless library run")
        return 2
    # THE RAIL'S PHYSICS (2026-11-17, WP4-04): the flag, else the profile's,
    # else SITL's own; the Gazebo functional rail is flown with --backend
    # sitl only, and every rail at sim_speedup 1 (RAIL_SIM_SPEEDUP_WHY).
    rail_physics = args.rail_physics or ((profile.rail_physics if profile else None)
                                         or "sitl")
    rail_speedup = (args.rail_sim_speedup if args.rail_sim_speedup is not None
                    else (profile.rail_sim_speedup if profile else 1.0))
    if args.sitl_dry_run and args.backend != "sitl":
        print("REFUSED: --sitl-dry-run is for --backend sitl")
        return 2
    if args.backend != "sitl" and (rail_physics != "sitl" or rail_speedup != 1.0):
        print("REFUSED: " + (f"profile {profile.profile!r} flies" if profile and
                             profile.rail_physics == "gazebo" else "--rail-physics / "
                             "--rail-sim-speedup choose")
              + f" the desktop rail's physics (Gazebo: the functional rail): --backend "
                f"sitl, not {args.backend}")
        return 2
    try:
        check_rail_physics("sitl", rail_physics, rail_speedup)
    except ValueError as e:
        print(f"REFUSED: {e}")
        return 2
    if args.backend != "hil" and (args.hil_config or args.hil_dry_run or args.hil_controls
                                  or args.hil_campaign or args.hil_max_s_add is not None):
        print("REFUSED: the --hil-* options are for --backend hil")
        return 2
    hil_max_s_add = 0.0
    if args.hil_max_s_add is not None:
        try:
            hil_max_s_add = check_hil_max_s_add(args.hil_max_s_add)
        except ValueError as e:
            print(f"REFUSED: {e}")
            return 2
    try:
        hil_cfg = HilConfig.load(args.hil_config) if args.backend == "hil" else None
    except (OSError, ValueError, TypeError) as e:
        print(f"REFUSED: {e}")
        return 2
    episodes, info = expand(lib, profile, args.seed, args.only, forecast_arms=fc_arms,
                            response_arms=rsp_arms)
    if args.rescore_rail is not None:
        # The offline re-score (2026-11-07): the runs as they were flown, read
        # with this checkout's scorer; nothing is written beside them.
        if not args.out:
            print("REFUSED: --rescore-rail needs --out (its table's JSON; nothing is "
                  "written beside the runs)")
            return 2
        if args.hil_max_s_add is not None:
            print("REFUSED: --hil-max-s-add chooses how long a campaign flies; "
                  "--rescore-rail reads each run's own length from its scenario record")
            return 2
        if args.rail_arc_forecast != "off":
            print("REFUSED: --rail-arc-forecast chooses what a campaign flies; "
                  "--rescore-rail reads runs already flown")
            return 2
        return rescore_rail_main(args.rescore_rail, episodes, lib.defaults,
                                 args.rail_gates, Path(args.out),
                                 labels_path=(Path(args.rail_labels)
                                              if args.rail_labels else None))
    if args.rail_gates != "ideal" and args.backend not in RAIL_BACKENDS:
        print("REFUSED: --rail-gates is for a rail's runs (--backend sitl / hil, "
              "--rescore-rail)")
        return 2
    # The arc forecast on the rail's Shield node (--rail-arc-forecast): the
    # flag every mission's node is handed, checked as rail_extra_args checks
    # the harness's (each declared by the node), before a mission is spent.
    rail_arc_args: list[str] = []
    if args.rail_arc_forecast == "on":
        if args.backend not in RAIL_BACKENDS:
            print("REFUSED: --rail-arc-forecast is for a rail's flights (--backend sitl / hil)")
            return 2
        try:
            rail_arc_args = rail_extra_args(["--arc-forecast=on"])
        except UnsupportedScenario as e:
            print(f"REFUSED: {e}")
            return 2
        info["rail_arc_forecast"] = "on"
        print("ARC-FORECAST CAMPAIGN: every mission's Shield node flies --arc-forecast=on; "
              "a separate campaign, never merged with a straight-forecast one\n")
    if args.rail_gates != "ideal" and profile is not None and profile.prefix_consumer:
        # The derived bounds come from a headless flight of the cell's own
        # pilot; a consuming pilot flies another flight.
        print("REFUSED: --rail-gates derives its bounds on the cell's own pilot; a "
              "prefix-consumer profile is judged on the cells' own gates (ideal)")
        return 2
    info["wind_model"] = args.wind_model
    # The autopilot model behind every autopilot-flown number of this run
    # (AUTOPILOT_HOLD_MODELS; review of T1c, P3).
    info["autopilot_hold_model"] = args.autopilot_hold
    info["autopilot_hold_model_flown_by_a_rail"] = args.autopilot_hold == "in_place"
    # The Shield's recovery law behind every recovery of this run (WP3-17).
    info["recovery_law"] = args.recovery_law
    tag = profile.profile if profile else None
    # A run that flies a comparison configuration gets its own default folder
    # (run_folder), outside the stress_* glob the KPI report reads; --publish
    # keeps demo/out/sweep for its bundles (its 3 s arm is part of the
    # published run, and that folder is outside the glob already).
    cmp_tag = None if args.publish else comparison_tag(
        fc_arms if fc_arms is not None else (profile.forecast_arms if profile else ["grant"]),
        eff_rsp, args.wind_model, args.autopilot_hold, args.recovery_law)
    out = results_path(args.out, args.publish, tag, cmp_tag)
    runs = Path(args.runs) if args.runs else run_folder(tag, cmp_tag)
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    verbose = len(episodes) <= 80

    arms = info.get("arms") or {}
    print(f"{info['cells']} scenarios, {info['episodes']} episodes"
          + (f" ({arms.get('extra_arm_episodes')} of them extra arms)"
             if arms.get("extra_arm_episodes") else "")
          + (f" + {arms.get('forecast_comparison_episodes')} on the 3 s comparison "
             f"forecast" if arms.get("forecast_comparison_episodes") else "")
          + (f" + {arms.get('response_comparison_episodes')} on the ArduPilot vehicle "
             f"response" if arms.get("response_comparison_episodes") else "")
          + f", backend={args.backend}" + (f", profile={tag}" if tag else "")
          + (f", wind model {args.wind_model}" if args.backend == "headless" else "")
          + "\n")
    refusals = preflight_refusals(info, profile is not None, bool(args.only))
    if refusals:
        for r in refusals:
            print(f"  REFUSED: {r}")
        print("\nNothing was run (exit 2).")
        return 2
    if profile and not info["cells_within_expected"]:
        lo, hi = info["expected_cells"]
        print(f"  note: --only narrowed the profile to {info['cells']} of its "
              f"{lo}-{hi} scenarios\n")

    # THE DESKTOP RAIL'S PHYSICS (2026-11-17): recorded in the results; the
    # dry run lists the missions and stops.
    if args.backend == "sitl":
        info["rail"] = {"physics": rail_physics, "sim_speedup": rail_speedup,
                        "functional_only": rail_physics == "gazebo",
                        **({"why": GAZEBO_FUNCTIONAL_WHY} if rail_physics == "gazebo"
                           else {})}
        if args.sitl_dry_run:
            print_sitl_plan(sitl_plan(episodes, lib.defaults, rail_physics, rail_speedup,
                                      stamp),
                            rail_physics, rail_speedup)
            return 0

    # THE HIL RAIL (2026-11-06): one mission per scenario, SITL on this
    # desktop and the mission on the Orin (run_hil). The campaign's name
    # prefixes every mission tag; the run folders are fetched into the store.
    hil_transport = hil_store = None
    hil_campaign = args.hil_campaign or ("hil" + datetime.now(timezone.utc).strftime(
        "%m%dT%H%MZ"))
    if args.backend == "hil":
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,23}", hil_campaign):
            print(f"REFUSED: --hil-campaign {hil_campaign!r} must be a short plain name "
                  f"(letters, digits, - and _; it prefixes every mission tag)")
            return 2
        plan = hil_plan(episodes, lib.defaults, hil_cfg, hil_campaign, args.hil_controls,
                        max_s_add=hil_max_s_add, rail_battery=rail_battery)
        info["hil"] = {"campaign": hil_campaign, "config": hil_config_record(hil_cfg),
                       "flyable": sum(1 for p in plan if p["fly"]),
                       "refused": {p["id"]: p["refused"] for p in plan if not p["fly"]},
                       # How long its missions flew (--hil-max-s-add, 2026-11-14):
                       # 0 is the profile's own length. tools/kpi_report.py reads
                       # it and never merges a lengthened campaign with another.
                       "max_s_add": hil_max_s_add,
                       "flight_length": hil_flight_length(hil_max_s_add)}
        if rail_battery:
            # Only an opted-in campaign carries the key (RAIL_BATTERY_OPT_IN).
            info["hil"]["rail_battery"] = True
        if rail_arc_args:
            # --rail-arc-forecast (2026-11-17): the arc forecast on every
            # mission's Shield node; only then, so every other campaign's
            # record is as it was.
            info["hil"]["arc_forecast"] = "on"
        if args.hil_dry_run:
            print_hil_plan(plan, hil_cfg, hil_max_s_add)
            return 0
        if hil_max_s_add:
            print(f"LENGTHENED CAMPAIGN {hil_campaign}: every mission flies "
                  f"{hil_max_s_add:g} s past the rail's cap (--hil-max-s-add); a separate "
                  f"campaign, never merged into the profile's KPI figure\n")
        hil_store = (Path(hil_cfg.store) if hil_cfg.store else
                     ROOT / "demo" / "out" / f"hil_{tag or 'library'}")
        if not hil_store.is_absolute():
            hil_store = ROOT / hil_store
        hil_transport = make_hil_transport(hil_cfg)
        hil_k = 0

    base = manifest_base()
    store = None
    done: dict[str, dict] = {}
    if args.resume:
        eff_fc = (fc_arms if fc_arms is not None
                  else (profile.forecast_arms if profile else ["grant"]))
        fingerprint = checkpoint_fingerprint(
            base, lib_path, _profile_path(args.profile) if args.profile else None,
            {"seed": args.seed, "only": args.only, "wind_model": args.wind_model,
             "autopilot_hold": args.autopilot_hold, "recovery_law": args.recovery_law,
             "forecast_arms": list(eff_fc), "response_arms": list(eff_rsp),
             "keep_logs": args.keep_logs, "no_bundles": args.no_bundles,
             "runs": _show(runs)},
            episodes)
        store = CheckpointStore(Path(args.resume))
        why = store.open(fingerprint, stamp)
        if why:
            print(f"REFUSED: {why}")
            return 2
        stamp = store.stamp
        mine = [e for k, e in enumerate(episodes) if in_shard(k, shard)]
        done, unreadable = store.load({episode_key(e) for e in mine})
        for name in unreadable[:5]:
            print(f"  note: unreadable checkpoint {name} - that episode is flown again")
        missing = [e for e in mine if episode_key(e) not in done]
        print(f"checkpoints in {_show(store.folder)}: {len(mine) - len(missing)} of "
              + (f"shard {shard[0]}/{shard[1]}'s {len(mine)}" if shard
                 else f"{len(mine)}") + " episodes done, "
              + f"{len(missing)} to fly"
              + (" (dirty tree: changes between runs are checked in the library, profile "
                 "and episode list only)" if store.header.get("dirty_tree") else "") + "\n",
              flush=True)
        if args.merge_only and missing:
            print(f"REFUSED: --merge-only, but {len(missing)} episode(s) have no checkpoint "
                  f"(first: {', '.join(f'{e.run_id} seed={e.seed}' for e in missing[:3])}); "
                  f"run the missing shard(s), or resume without --merge-only to fly them")
            return 2
    n_ran = n_resumed = 0
    wall_ckpt = 0.0
    t_loop = t_progress = time.perf_counter()
    todo = sum(1 for k in range(len(episodes)) if in_shard(k, shard))

    def _checkpoint(i_ep, ep, kind, shield, item, table, t0):
        # After the episode is complete (its bundle written, its row and table
        # built): a run killed before this line flies the episode again.
        if store is not None:
            store.save({"format": CHECKPOINT_FORMAT, "key": episode_key(ep),
                        "index": i_ep, "run_id": ep.run_id, "seed": ep.seed,
                        "kind": kind, "shield": bool(shield), "status": item["status"],
                        "wall_s": round(time.perf_counter() - t0, 4),
                        "item": item, "table": table})

    rows_out: list[dict] = []
    tables_on: list[dict] = []
    tables_off: list[dict] = []
    n_pass = n_fail = n_skip = n_known = n_tracked = 0
    # The 3 s comparison arm: scored and judged like any episode, but kept
    # apart - never in _counts, the headline summaries or the exit code.
    cmp_rows: list[dict] = []
    cmp_tables_on: list[dict] = []
    cmp_tables_off: list[dict] = []
    cmp_counts = {s: 0 for s in ("pass", "tracked", "fail", "known_failure",
                                 "unexpected_pass", "skipped", "error")}
    # The ArduPilot-response arm (module docstring, "THE VEHICLE RESPONSE"):
    # kept apart exactly like the 3 s forecast arm. A run flies one of the
    # two comparisons at most (check_comparison_arms).
    rsp_rows: list[dict] = []
    rsp_tables_on: list[dict] = []
    rsp_tables_off: list[dict] = []
    rsp_counts = dict(cmp_counts)
    one_seed = len(episodes) == len({(e.run_id) for e in episodes})

    for i_ep, ep in enumerate(episodes):
        if not in_shard(i_ep, shard):
            continue
        if store is not None:
            now = time.perf_counter()
            if now - t_progress >= PROGRESS_EVERY_S:
                t_progress = now
                spe = (now - t_loop) / n_ran if n_ran else None
                left = todo - n_ran - n_resumed
                print(f"  progress: {n_ran + n_resumed}/{todo} episodes ({n_resumed} "
                      f"replayed, {n_ran} flown in {_fmt_s(now - t_loop)}"
                      + (f", {spe:.2f} s/episode, about {_fmt_s(spe * left)} left"
                         if spe else "") + ")", flush=True)
            # Popped: the record's item and table now live in the sinks, and
            # the dict does not hold a second reference for the rest of the run.
            rec = done.pop(episode_key(ep), None)
            if rec is not None:
                # REPLAY: the checkpoint's item and KPI table, counted exactly
                # as the loop below counts a flown episode.
                n_resumed += 1
                wall_ckpt += float(rec.get("wall_s") or 0.0)
                rkind, rstatus = rec.get("kind"), rec["status"]
                {"forecast": cmp_rows, "response": rsp_rows}.get(rkind, rows_out).append(
                    rec["item"])
                if rkind is not None:
                    (rsp_counts if rkind == "response" else cmp_counts)[rstatus] += 1
                elif rstatus == "skipped":
                    n_skip += 1
                elif rstatus in ("error", "fail", "unexpected_pass"):
                    n_fail += 1
                elif rstatus == "known_failure":
                    n_known += 1
                elif rstatus == "tracked":
                    n_tracked += 1
                else:
                    n_pass += 1
                if rec.get("table") is not None:
                    on = bool(rec["shield"])
                    if rkind == "forecast":
                        (cmp_tables_on if on else cmp_tables_off).append(rec["table"])
                    elif rkind == "response":
                        (rsp_tables_on if on else rsp_tables_off).append(rec["table"])
                    else:
                        (tables_on if on else tables_off).append(rec["table"])
                continue
            n_ran += 1
        t_ep = time.perf_counter()
        spec = ep.cell.spec
        # What the episode is scored against: the cell's own spec, less its
        # labels on the prefix-on arm of a consuming pilot (scoring_spec).
        sspec = scoring_spec(spec, ep)
        eid = ep.run_id
        kind = ("forecast" if ep.forecast != "grant"
                else "response" if ep.response != "ideal" else None)
        is_cmp = kind is not None
        sink = {"forecast": cmp_rows, "response": rsp_rows}.get(kind, rows_out)
        ccounts = rsp_counts if kind == "response" else cmp_counts
        label = eid if one_seed else f"{eid} seed={ep.seed}"
        _LIVE.beat(progress=i_ep / max(1, len(episodes)), detail=label)   # memory only
        common = {"id": eid, "id_cell": ep.episode_id, "scenario_id": ep.cell.scenario_id,
                  "template": spec.template, "params": ep.cell.params,
                  "seed": ep.seed, "arm": "on" if spec.shield else "off",
                  "prefix": ep.prefix, "expect": spec.expect,
                  "forecast_arm": ep.forecast,
                  "response_arm": ep.response,
                  "why": spec.description.strip()}
        if ep.prefix_consumer:
            # A prefix-consumer run only (every other result keeps its keys).
            common.update(prefix_consumer=ep.prefix_consumer, repeat=ep.repeat)
            if sspec is not spec:
                common["labels_not_applied"] = LABELS_NOT_APPLIED
        run = None
        arm_record = None
        hil_run = None
        try:
            if args.backend == "hil":
                # Refused before a mission number is spent, so the tags match
                # the dry run's (hil_plan).
                why = rail_refusal(spec, "hil", episode=ep, defaults=lib.defaults,
                                   rail_battery=rail_battery)
                if why is None and not spec.shield and not args.hil_controls:
                    why = ("a Shield-off control arm: flown only with --hil-controls (it "
                           "flies the raw pilot into the zones, and the GeoFence backstop "
                           "may take the aircraft)")
                if why:
                    raise UnsupportedScenario(why)
                hil_k += 1
                try:
                    hil_run = run_hil(spec, ep, ep.seed, lib.defaults, hil_cfg,
                                      hil_transport, hil_campaign, hil_k, hil_store,
                                      max_s_add=hil_max_s_add, rail_battery=rail_battery,
                                      extra_args=rail_arc_args or None)
                except UnsupportedScenario:
                    hil_k -= 1
                    raise
                rows, extra = hil_run["rows"], hil_run["extra"]
                policy, prios = hil_run["policy"], hil_run["prios"]
                events, hash0, sim_s, wall_s, consumed = [], policy.policy_hash, 0, 0, False
                # A prefix-consumer mission's arm record (rail_meta's); None
                # for every other mission, as before.
                arm_record = hil_run["meta"].get("arm_record")
            elif args.backend == "sitl":
                sitl_run_tag = sitl_tag(ep, rail_physics)
                got = run_sitl(spec, sitl_run_tag, ep.seed, lib.defaults, episode=ep,
                               physics=rail_physics, sim_speedup=rail_speedup,
                               rail_battery=rail_battery,
                               extra_args=rail_arc_args or None)
                if got is None:
                    n_skip += 1
                    print(f"  SKIP  {label:<34} ArduPilot rail not available here")
                    rows_out.append({**common, "status": "skipped",
                                     "reason": "sitl backend unavailable"})
                    continue
                rows, extra = got
                # Where the rail wrote this episode (its manifest, bag, audit,
                # kpi.json and signed replay bundle, <tag>.replay.tar.gz) and,
                # as a label only, the grant's per-episode name, which no file
                # on the rail carries (GRANT_EPISODE_NAME_NOTE; 2026-11-17).
                common["run"] = f"demo/out/{sitl_run_tag}"
                common["mission_tag"] = sitl_run_tag
                common["replay_bundle"] = (f"demo/out/{sitl_run_tag}/"
                                           f"{sitl_run_tag}.replay.tar.gz")
                common["grant_episode_name"] = ep.dir_name(stamp)
                common["grant_episode_name_note"] = GRANT_EPISODE_NAME_NOTE
                if rail_physics == "gazebo":
                    common["functional_only"] = True
                policy = spec.policy.load(ROOT)
                prios = K.rule_priorities(policy)
                events, hash0, sim_s, wall_s, consumed = [], policy.policy_hash, 0, 0, False
                if ep.prefix_consumer:
                    arm_record = build_arm(ep, policy, defaults=lib.defaults)
            else:
                no_consumer = prefix_consumer_refusal(spec) if ep.prefix_consumer else None
                if no_consumer:
                    raise UnsupportedScenario(no_consumer)
                arm_record = build_arm(ep, defaults=lib.defaults)
                run = run_episode(spec, lib.defaults, ep.seed, arm=arm_record,
                                  forecast=ep.forecast, wind_model=args.wind_model,
                                  autopilot_hold=args.autopilot_hold,
                                  response=ep.response, recovery=args.recovery_law)
                rows, extra, policy = run.rows, run.extra, run.policy
                events, hash0 = run.events, run.policy_hash_start
                sim_s, wall_s, consumed = run.sim_s, run.wall_s, run.seed_consumed
                prios = run.priorities
        except UnsupportedScenario as e:
            if is_cmp:
                ccounts["skipped"] += 1
            else:
                n_skip += 1
            print(f"  SKIP  {label:<34} {e}")
            sink.append({**common, "status": "skipped", "reason": str(e)})
            _checkpoint(i_ep, ep, kind, spec.shield, sink[-1], None, t_ep)
            continue
        except Exception as e:                                   # noqa: BLE001
            if is_cmp:
                ccounts["error"] += 1
            else:
                n_fail += 1
            print(f"  ERROR {label:<34} {type(e).__name__}: {e}")
            sink.append({**common, "status": "error", "error": str(e)})
            _checkpoint(i_ep, ep, kind, spec.shield, sink[-1], None, t_ep)
            continue

        res = score(sspec, rows, extra, policy, prios)
        gates_used = derivation = other = None
        basis_used = "ideal"
        if args.backend in RAIL_BACKENDS:
            # A rail run's gates: the cell's own with each safety invariant's
            # autopilot counterpart (rail_gate_set, 2026-11-07); on a derived
            # basis (PROPOSED, NOT ADOPTED) the vehicle-pinned bounds moved,
            # the verdict on the cell's own kept beside it.
            gates_used, _ = rail_gate_set("ideal", spec, ep.seed, lib.defaults)
            if args.rail_gates != "ideal":
                other = {"ideal": verdict_record(*episode_verdict(
                    sspec, res, ep.cell.params, ep.seed, gates=gates_used))}
                try:
                    gates_used, derivation = rail_gate_set(args.rail_gates, spec, ep.seed,
                                                           lib.defaults)
                    basis_used = args.rail_gates
                except Exception as e:                           # noqa: BLE001
                    derivation = {"error": f"{type(e).__name__}: {e} (the cell's own "
                                           f"gates used)"}
        status, bad, labels_failed, known_why = episode_verdict(
            sspec, res, ep.cell.params, ep.seed, gates=gates_used)
        if hil_run is not None:
            # Beside the fetched run, for tools/kpi_report.py (load_run).
            write_scenario_score(hil_run["run_dir"], hil_run["meta"], res, extra, status,
                                 bad, labels_failed, known_why, prios, sspec,
                                 gates_basis=basis_used, gates=gates_used,
                                 derivation=derivation, other_verdicts=other)
            common["run"] = _show(hil_run["run_dir"])
            common["mission_tag"] = hil_run["tag"]
        if spec.expect == "tracked":
            common["labels_failed"] = labels_failed
        if known_why is not None:
            common["known_failure_reason"] = known_why
        mark = _MARKS[status]
        if is_cmp:
            ccounts[status] += 1
            mark = ("cmp3s" if kind == "forecast"
                    else "cmpRL" if ep.response == "rail" else "cmpAP")
        elif status == "known_failure":
            n_known += 1
        elif status in ("fail", "unexpected_pass"):
            # An unexpected pass fails the run: a marker left on a fixed
            # defect would hide the next regression of that cell.
            n_fail += 1
        elif status == "tracked":
            n_tracked += 1
        else:
            n_pass += 1

        if verbose or (status not in ("pass", "tracked") and not is_cmp):
            # str(): the rate is None on a flight log with no row (a rail
            # launch that recorded nothing; guardrail/kpi.py "AN EMPTY LOG"),
            # and None has no :<9 format - the print would raise.
            print(f"  {mark} {label:<34} escape={str(res['p0_violation_escape_rate']):<9} "
                  f"repairs={res['repair_count']:<5} breaches={res.get('breaches')} "
                  f"t2safe_eps={res['time_to_safe_episodes']} "
                  f"outcome={res.get('mission_outcome')}"
                  + (f" in-poly={res.get('ticks_inside_fence_polygon')}"
                     if status == "tracked" else ""))
            for b in bad:
                print(f"          {b}")

        manifest = (episode_manifest(base, hash0, ep.seed, sim_s, wall_s)
                    if args.backend == "headless" else None)
        extras = manifest_extras(arm_record)
        bundle = None
        if manifest is not None and not args.no_bundles:
            ep_dir = runs / ep.dir_name(stamp)
            from guardrail.manifest import is_kpi_grade
            grade, why_not = is_kpi_grade(manifest, {})
            # The bundle's extras carry everything a separate tool needs to
            # re-fly the episode (episode_extras, since 2026-10-12) beside
            # the arm's own flat keys.
            full_extras = {**(extras or {}),
                           **episode_extras(spec, lib.defaults, ep.seed, run,
                                            arm=arm_record, forecast=ep.forecast,
                                            wind_model=args.wind_model, episode=ep,
                                            autopilot_hold=args.autopilot_hold,
                                            response=ep.response,
                                            recovery=args.recovery_law)}
            write_bundle(ep_dir, manifest, _json_safe({
                **common, "status": status, "gates_failed": bad,
                "kpi_grade": grade, "kpi_grade_reasons": why_not,
                "arm_record": arm_record, "rule_priorities": prios,
                "metrics": {"reached_goal": extra.get("reached_goal"),
                            "breaches": extra.get("breaches"),
                            "shield": "on" if spec.shield else "off",
                            "fsm": extra.get("fsm")},
                "kpi": res}), events, rows if args.keep_logs else None,
                _json_safe(full_extras), audit=run.audit)
            bundle = _show(ep_dir)

        item = {**common, "status": status, "gates_failed": bad, "kpi": res,
                "seed_consumed": consumed,
                # Headless: the seed went into run_episode. SITL: the node
                # records --seed in its manifest (StubVLA draws nothing).
                "seed_applied": True,
                "grant_config": (extra.get("forecast") or {}).get("grant_config"),
                "manifest": manifest, "manifest_extras": extras,
                "arm_record": arm_record, "bundle": bundle}
        sink.append(item)
        table = {**res, "id": eid, "family": spec.template, "cell": ep.episode_id,
                 "arm": common["arm"], "prefix": ep.prefix, "params": ep.cell.params,
                 "forecast_arm": ep.forecast, "response_arm": ep.response,
                 "bundle": bundle, "kpi_grade": False}
        if ep.prefix_consumer:
            table["prefix_consumer"] = ep.prefix_consumer     # cell_key's row term
        table["worst_tick"] = _json_safe(K.worst_tick(rows, prios))
        table["null_hover_mission"] = K.null_hover_mission(
            rows, {"reached_goal": extra.get("reached_goal")})
        if not spec.shield:
            # A control arm's logged repairs were never flown: in the
            # rollups and the KPI report it reads "shield off" with a repair
            # count of 0 (the counterfactual count is kept beside it), the
            # same transform tools/kpi_report.py applies to stored tables.
            # The episode's own KPI table (`kpi`, the bundle) is unchanged.
            sys.path.insert(0, str(ROOT / "tools"))
            import kpi_report
            table = kpi_report.shield_off_table(table)
        if kind == "forecast":
            (cmp_tables_on if spec.shield else cmp_tables_off).append(table)
        elif kind == "response":
            (rsp_tables_on if spec.shield else rsp_tables_off).append(table)
        else:
            (tables_on if spec.shield else tables_off).append(table)
        _checkpoint(i_ep, ep, kind, spec.shield, item, table, t_ep)

    if store is not None:
        wall_now = time.perf_counter() - t_loop
        print(f"  episodes: {n_ran} flown in {_fmt_s(wall_now)}"
              + (f" ({wall_now / n_ran:.2f} s/episode)" if n_ran else "")
              + f", {n_resumed} replayed from {_show(store.folder)}", flush=True)
    if shard:
        # A shard writes checkpoints only: its part of the sweep is not a
        # sweep, and a results JSON of it would be read as one.
        print(f"\nshard {shard[0]}/{shard[1]}: {n_pass} passed, {n_tracked} tracked, "
              f"{n_fail} failed, {n_known} known failures, {n_skip} skipped; "
              f"checkpoints in {_show(store.folder)}. When every shard is done: "
              f"the same command with --merge-only instead of --shard")
        return 1 if n_fail else 0

    # ---- summaries ---------------------------------------------------------
    scored = [r for r in rows_out if "kpi" in r]
    oc = [r for r in scored if r["kpi"].get("outcome_matches_label") is not None]
    families = family_summary(scored)
    all_sigs = {kpi_signature(r) for r in scored}
    true_state = true_state_summary(scored)
    # ALL OR NOTHING (review of 2026-10-19, harness-tools/rollup-failure-
    # swallowed). Until then an exception part-way through filled `cells` up
    # to the failing group, recorded `_rollup_note` in the JSON only, and the
    # run exited 0: kpi_report.md was written with the cells built so far and
    # no word of the failure, or not written at all with the summary looking
    # normal (reproduced by making K.rollup raise on its Nth call). Now the
    # rollups are built into locals and kept only when every group rolled
    # up; a failure is printed and exits 4 (below), and no partial report is
    # written.
    rollups, rollup_note = {}, None
    cells: dict[str, dict] = {}
    try:
        new_rollups: dict[str, Any] = {}
        new_cells: dict[str, dict] = {}
        for fam in sorted({t["family"] for t in tables_on}):
            new_rollups[fam] = _rollup([t for t in tables_on if t["family"] == fam])
        new_rollups["_all_shield_on"] = _rollup(tables_on)
        new_rollups["_controls_shield_off"] = _rollup(tables_off)
        if new_rollups["_all_shield_on"] is not None:
            # Beside the grant's p0_escape_rate (the Shield's own count), where
            # the aircraft really was: the two answer different questions.
            new_rollups["_all_shield_on"]["true_state"] = true_state
        if cmp_tables_on:
            r3 = _rollup(cmp_tables_on)
            if r3 is not None:
                new_rollups["_comparison_forecast_3s_shield_on"] = dict(
                    r3, grant_config=False,
                    note="the 3 s / 0.5 s comparison arm, NOT the grant's Shield "
                         "configuration; never part of a headline number")
        if rsp_tables_on:
            ra = _rollup(rsp_tables_on)
            if ra is not None:
                new_rollups[f"_comparison_response_{rsp_arm}_shield_on"] = dict(
                    ra, response_arm=rsp_arm,
                    true_state=true_state_summary(
                        [r for r in rsp_rows if "kpi" in r]),
                    note=f"the grant's Shield flown on the {rsp_arm} vehicle "
                         "response (docs/DESIGN-harness-vehicle-response.md), a "
                         "comparison arm; never part of a headline number")
        groups: dict[str, list] = {}
        for t in (tables_on + tables_off + cmp_tables_on + cmp_tables_off
                  + rsp_tables_on + rsp_tables_off):
            groups.setdefault(cell_key(t), []).append(t)
        for key in sorted(groups):
            members = groups[key]
            new_cells[key] = dict(_rollup(members), members=[t["id"] for t in members],
                                  **cell_extras(members))
        rollups, cells = new_rollups, new_cells
    except Exception as e:                                       # noqa: BLE001
        rollup_note = f"guardrail.kpi.rollup failed: {type(e).__name__}: {e}"
    comparison = (forecast_comparison(scored, [r for r in cmp_rows if "kpi" in r],
                                      cmp_counts, cmp_rows)
                  if cmp_rows else None)
    rsp_comparison = (response_comparison(scored, [r for r in rsp_rows if "kpi" in r],
                                          rsp_counts, rsp_rows, arm=rsp_arm)
                      if rsp_rows else None)
    top = None
    try:
        top = K.top_failures(tables_on + tables_off, k=10)
    except Exception as e:                                       # noqa: BLE001
        rollup_note = ((rollup_note + "; ") if rollup_note else "") + (
            f"guardrail.kpi.top_failures failed: {type(e).__name__}: {e}")

    runtime = round(time.perf_counter() - t_run, 2)
    # `grant_profile_kpi_bearing` is the profile's row in the grant's schedule
    # (smoke and nightly "feed the final KPI report"); it says nothing about
    # THIS run, and a deck builder reading `true` beside a headless result
    # would take it for one. The run's own answer sits beside it.
    functional = args.backend == "sitl" and rail_physics == "gazebo"
    info["kpi_bearing_this_run"] = (False if args.backend == "headless" or functional
                                    else None)
    info["kpi_bearing_this_run_why"] = (
        "headless-kinematic topology, measured sim_speedup >> 1: no episode can "
        "be KPI-grade (guardrail.manifest.is_kpi_grade)"
        if args.backend == "headless" else GAZEBO_FUNCTIONAL_WHY if functional else
        "judged per run by guardrail.manifest.is_kpi_grade on the node's manifest")
    fsl = failsafe_summary(scored)
    arms_doc = arms_summary(scored)
    argv_l = list(argv if argv is not None else sys.argv[1:])
    command = "python experiments/sweep_scenarios.py" + (
        " " + " ".join(argv_l) if argv_l else "")
    command_as_run = command
    if store is not None:
        # The reproduce command of a resumed or merged run is the one
        # uninterrupted run it equals (tests/test_sweep_resume.py): a
        # --merge-only line reproduces nothing without the checkpoint folder.
        plain = strip_resume_args(argv_l)
        command = "python experiments/sweep_scenarios.py" + (
            " " + " ".join(plain) if plain else "")
    doc = {
        "_what": ("Every episode of " + (f"profile {tag!r} over " if tag else "")
                  + f"{_show(lib_path)}, scored with guardrail.kpi.compute - the "
                  "same function the delivered flights use."),
        "_command": command,
        "_backend": args.backend,
        "_not_kpi_grade": ("headless-kinematic topology, measured sim_speedup >> 1: "
                           "functional evidence, never a contractual KPI number"
                           if args.backend == "headless" else
                           GAZEBO_FUNCTIONAL_WHY if functional else None),
        "_counts": {"pass": n_pass, "tracked": n_tracked, "fail": n_fail,
                    "known_failure": n_known, "skipped": n_skip},
        "_scope": info,
        "_distinct_kpi_signatures": {
            "episodes": len(scored), "distinct": len(all_sigs),
            "note": "episodes whose KPI tables differ (status, escapes, repairs, "
                    "breaches, fail-safe, outcome, goal, time to safe, true-state "
                    "distance and ticks, speed, range, offset). An upper bound on "
                    "behavioural variety: float jitter in repair counts splits "
                    "symmetric copies"},
        "_runtime_s": runtime,
        "_runner": runner_facts(),
        "_manifest_common": base,
        "_seeds": {"episodes_consuming_seed": sum(1 for r in scored
                                                  if r.get("seed_consumed")),
                   "episodes_deterministic": sum(1 for r in scored
                                                 if not r.get("seed_consumed")),
                   "episodes_seed_not_applied": sum(1 for r in scored
                                                    if not r.get("seed_applied")),
                   "note": "a deterministic episode repeated on another seed is "
                           "identical by construction, not spread"},
        "_failsafe_labels": fsl,
        "_outcome_labels": {
            "labelled": len(oc), "unlabelled": len(scored) - len(oc),
            "matching": sum(1 for r in oc if r["kpi"]["outcome_matches_label"]),
            "outcomes": {o: sum(1 for r in scored
                                if r["kpi"].get("mission_outcome") == o)
                         for o in K.OUTCOMES}},
        "_arms": arms_doc,
        "_true_state": true_state,
        "_families": families,
        "_rollup": rollups or None,
        "_cell_rollup": cells or None,
        "_rollup_note": rollup_note,
        "_top_failures": top,
        # The 3 s comparison arm (module docstring): None unless it flew.
        "_forecast_comparison": comparison,
        # The ArduPilot vehicle-response arm (module docstring, "THE VEHICLE
        # RESPONSE"): None unless it flew.
        "_response_comparison": rsp_comparison,
        "results": rows_out,
    }
    if store is not None:
        # Only on a --resume run, so every other run's document is unchanged.
        # `_runtime_s` is this process's; the episodes' own time is here.
        n_eps = n_ran + n_resumed
        ep_wall = wall_ckpt + (time.perf_counter() - t_loop if n_ran else 0.0)
        doc["_resume"] = {
            "checkpoints": _show(store.folder), "bundle_stamp": stamp,
            "command_as_run": command_as_run,
            "episodes_replayed": n_resumed, "episodes_flown_by_this_process": n_ran,
            "episode_wall_s_total": round(ep_wall, 1),
            "s_per_episode": round(ep_wall / n_eps, 3) if n_eps else None,
            **store.wall_span(),
            "dirty_tree": bool(store.header.get("dirty_tree")),
            "note": "results replayed from per-episode checkpoints are the records "
                    "the flying process wrote; the summaries are computed over all "
                    "of them as one run computes them"}
    out.parent.mkdir(parents=True, exist_ok=True)
    # allow_nan=False, deliberately. Python happily writes a bare `NaN` token,
    # which is NOT valid JSON: the deck build (JavaScript) refused to parse this
    # file and that is how a NaN repair magnitude was found at all. Failing here
    # is better than shipping an artefact only Python can read.
    if args.publish:
        full = results_path(None, False, tag)
        full.parent.mkdir(parents=True, exist_ok=True)
        full.write_text(json.dumps(_json_safe(doc), indent=2, allow_nan=False) + "\n",
                        encoding="utf-8")
        # A run whose rollups failed is not published: the evaluation data and
        # the decks read `_rollup` / `_cell_rollup` from the published copy.
        if not rollup_note:
            out.write_text(json.dumps(_json_safe(published_view(doc)), indent=2,
                                      allow_nan=False) + "\n", encoding="utf-8")
    elif store is not None:
        # A merge of hours of episodes is not left torn by a kill.
        atomic_write_text(out, json.dumps(_json_safe(doc), indent=2, allow_nan=False) + "\n")
    else:
        out.write_text(json.dumps(_json_safe(doc), indent=2, allow_nan=False) + "\n",
                       encoding="utf-8")
    report_md = None
    if cells and not args.no_bundles:
        # The grant's KPI report for this run: one row per (template, cell).
        sys.path.insert(0, str(ROOT / "tools"))
        import kpi_report
        runs.mkdir(parents=True, exist_ok=True)
        report_md = runs / "kpi_report.md"
        with open(report_md, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(kpi_report.render_sweep_markdown(_json_safe(doc)))

    if not verbose or len(families) > 1:
        print(f"\n  {'template':<30} {'eps':>4} {'pass':>5} {'track':>6} {'fail':>5} "
              f"{'known':>6} {'distinct':>9} {'breach':>7} {'entered':>8} "
              f"{'in-poly':>8} {'min d_fence':>12}")
        for name, f in families.items():
            dmin = f["min_true_dist_to_fence_m"]
            print(f"  {name:<30} {f['episodes']:>4} {f['pass']:>5} {f['tracked']:>6} "
                  f"{f['fail']:>5} {f['known_failure']:>6} "
                  f"{f['distinct_kpi_signatures']:>9} {f['episodes_with_breach']:>7} "
                  f"{f['episodes_entering_polygon']:>8} "
                  f"{f['episodes_inside_polygon']:>8} "
                  f"{'-' if dmin is None else f'{dmin:.2f} m':>12}")
    print(f"\n{n_pass} passed, {n_tracked} tracked, {n_fail} failed, {n_known} "
          f"known failures, {n_skip} skipped in {runtime} s -> {_show(out)}")
    if report_md is not None:
        print(f"KPI report (one row per template and parameter cell): {_show(report_md)}")
    print(f"{len(scored)} episodes scored, {len(all_sigs)} distinct KPI signatures")
    print("outcomes: " + ", ".join(f"{o} {n}" for o, n in
                                   doc["_outcome_labels"]["outcomes"].items()))
    on_roll = (rollups or {}).get("_all_shield_on") or {}
    off_roll = (rollups or {}).get("_controls_shield_off") or {}
    if on_roll:
        ts = true_state
        print(f"P0 escape (Shield log, the grant's KPI): {on_roll.get('p0_escape_rate')} "
              f"over the Shield-flown ticks of {on_roll.get('episodes')} shield-on "
              f"episodes (null passthrough "
              f"{on_roll.get('null_passthrough_p0_escape_rate')}; shield-off control "
              f"{off_roll.get('p0_escape_rate')}); "
              f"{on_roll.get('autopilot_ticks')} autopilot-flown ticks are outside it")
        dmin = ts["min_true_dist_to_fence_m_of_entries"]
        print(f"  true state: {ts['episodes_entering_polygon']} shield-on episode(s) "
              f"flew INTO a P0 polygon"
              + (f" (deepest {dmin:.2f} m)" if dmin is not None else "")
              + (f" {ts['episodes_entering_polygon_by_status']}"
                 if ts['episodes_entering_polygon'] else "")
              + f"; {ts['episodes_inside_polygon_autopilot_entered']} more were "
              f"carried in by the autopilot alone (below); "
              f"{ts['episodes_inside_polygon_imposed_only']} more were inside "
              f"only because a zone appeared, moved, grew or switched on under "
              f"them; {ts['true_p0_flown_ticks']} P0 ticks flown at the true state")
        if ts["episodes_window_opened_on"]:
            # Beside the rate, because the rate does not show it: the escape
            # from a window that opened on the aircraft is judged the
            # recovery (C2), so a slower escape would score no worse.
            print(f"  windows opened on the aircraft: {ts['episodes_window_opened_on']} "
                  f"episode(s) (a decision named the window, before or after it opened, "
                  f"whatever was flown); an escape from it is judged the recovery (not a "
                  f"P0 escape) and the rule counted as anticipated through the trace's "
                  f"window_opens_on; "
                  f"ticks inside a polygon "
                  f"{ts['window_opened_on_ticks_inside_polygon']}, time to safe at most "
                  f"{ts['window_opened_on_max_time_to_safe_s']} s")
        if ts.get("episodes_escape_slowed") is not None:
            # Beside the rate as well (review of 69c27bd, P3), zero included:
            # a slowed escape is legal, so the P0 rate cannot show it.
            print(f"  escapes slowed (legal, so outside the P0 rate): escape floor "
                  f"refused at the speed cap on {ts['cap_refused_ticks']} tick(s) (a "
                  f"slower escape flown), refused outright on "
                  f"{ts['floor_refused_ticks']} tick(s) (flown unfloored or reported to "
                  f"the FSM), in {ts['episodes_escape_slowed']} episode(s) "
                  f"{ts['episodes_escape_slowed_by_status']}")
        print(f"  autopilot (modelled LOITER / RTL / LAND, not the Shield; hold "
              f"model {args.autopilot_hold}"
              + (", flown by no rail" if args.autopilot_hold != "in_place" else "")
              + "): "
              f"{ts['autopilot_p0_flown_ticks']} of {ts['autopilot_ticks']} ticks "
              f"flew a P0-violating action in {ts['episodes_autopilot_p0_flown']} "
              f"episode(s) {ts['episodes_autopilot_p0_flown_by_status']}, by mode "
              f"{ts['autopilot_p0_flown_ticks_by_mode']} of "
              f"{ts['autopilot_ticks_by_mode']}; "
              f"{ts['autopilot_polygon_entries']} polygon entr(y/ies) in "
              f"{ts['episodes_autopilot_entering_polygon']} episode(s); "
              f"{ts['autopilot_breaches']} breach(es) in "
              f"{ts['episodes_autopilot_breach']} episode(s); RTL failed (G6's "
              f"input) in {ts['episodes_rtl_failed']} episode(s)")
    if comparison:
        cf = comparison["failsafe_labels"]
        cc = comparison["counts"]
        print(f"forecast comparison (3 s / 0.5 s, NOT the grant's; reported, never "
              f"gated): {cc['pass']} pass, {cc['tracked']} tracked, "
              f"{cc['fail'] + cc['error']} fail, {cc['known_failure']} known, "
              f"{cc['unexpected_pass']} pinned-but-passing; fail-safe "
              f"{cf['correct']}/{cf['labelled']} = {cf['fail_safe_correctness']} "
              f"({cf['false_triggers']} false, {cf['missed_triggers']} missed); "
              f"{comparison['episodes_moved']} episode(s) moved between the forecasts")
    if rsp_comparison:
        rc = rsp_comparison["counts"]
        ti, ta = rsp_comparison["totals"]["ideal"], rsp_comparison["totals"][rsp_arm]
        fi, fa = ti["failsafe_labels"], ta["failsafe_labels"]
        print(f"vehicle-response comparison (the {rsp_arm} response, NOT the "
              f"headline; reported, never gated): {rc['pass']} pass, {rc['tracked']} "
              f"tracked, {rc['fail'] + rc['error']} fail, {rc['known_failure']} known, "
              f"{rc['unexpected_pass']} pinned-but-passing over "
              f"{rsp_comparison['paired_episodes']} paired episodes; max velocity lag "
              f"{rsp_comparison['max_velocity_lag_mps']} m/s; "
              f"{rsp_comparison['episodes_moved']} episode(s) moved")
        print(f"  shield-on, ideal -> {rsp_arm}: unsafe-position entries "
              f"{ti['unsafe_position_entries']} -> {ta['unsafe_position_entries']} "
              f"(episodes {ti['episodes_with_unsafe_position_entry']} -> "
              f"{ta['episodes_with_unsafe_position_entry']}); polygon entries "
              f"{ti['polygon_entries']} -> {ta['polygon_entries']} (episodes "
              f"{ti['episodes_entering_polygon']} -> {ta['episodes_entering_polygon']}); "
              f"autopilot entries {ti['autopilot_polygon_entries']} -> "
              f"{ta['autopilot_polygon_entries']}; P0 escapes {ti['p0_escapes']} -> "
              f"{ta['p0_escapes']}; true P0 ticks, command {ti['true_p0_flown_ticks']} "
              f"-> {ta['true_p0_flown_ticks']}, airframe {ti['true_p0_vehicle_ticks']} -> "
              f"{ta['true_p0_vehicle_ticks']}; fail-safe labels "
              f"{fi['correct']}/{fi['labelled']} -> {fa['correct']}/{fa['labelled']} "
              f"({fi['false_triggers']} -> {fa['false_triggers']} false, "
              f"{fi['missed_triggers']} -> {fa['missed_triggers']} missed)")
        print(f"\n  {'family (shield-on), ideal -> ' + rsp_arm:<32} {'eps':>4} "
              f"{'unsafe-entries':>15} {'poly-entries':>13} {'ap-poly':>8} "
              f"{'P0-esc':>8} {'true-P0-cmd':>11} {'true-P0-veh':>11} {'fail-safe':>13}")
        for fam, row in rsp_comparison["families"].items():
            a, b = row["ideal"], row[rsp_arm]
            if not a:
                continue
            print(f"  {fam:<32} {a['episodes']:>4} "
                  f"{str(a['unsafe_position_entries']) + '->' + str(b['unsafe_position_entries']):>15} "
                  f"{str(a['polygon_entries']) + '->' + str(b['polygon_entries']):>13} "
                  f"{str(a['autopilot_polygon_entries']) + '->' + str(b['autopilot_polygon_entries']):>8} "
                  f"{str(a['p0_escapes']) + '->' + str(b['p0_escapes']):>8} "
                  f"{str(a['true_p0_flown_ticks']) + '->' + str(b['true_p0_flown_ticks']):>11} "
                  f"{str(a['true_p0_vehicle_ticks']) + '->' + str(b['true_p0_vehicle_ticks']):>11} "
                  f"{str(a['failsafe_correct']) + '/' + str(a['failsafe_labelled']) + '->' + str(b['failsafe_correct']) + '/' + str(b['failsafe_labelled']):>13}")
    if fsl["labelled"]:
        verdict = []
        if fsl["beats_never_trigger"] is False:
            verdict.append("NOT above never-trigger")
        if fsl["meets_grant_target"] is False:
            verdict.append(f"below the grant's {GRANT_FAILSAFE_TARGET}")
        wl = (fsl.get("fsm_scoring") or {}).get("wilson_low_95")
        print(f"fail-safe labels (FSM RTL/Land): {fsl['correct']}/{fsl['labelled']} = "
              f"{fsl['fail_safe_correctness']} ({fsl['false_triggers']} false "
              f"triggers, {fsl['missed_triggers']} missed); nulls: never trigger "
              f"{fsl['null_never_trigger']}, always trigger "
              f"{fsl['null_always_trigger']}; Wilson 95 % low {wl}; "
              f"{fsl['unlabelled']} unlabelled; "
              f"{fsl.get('censored_in_loiter', 0)} censored in Loiter; distinct flights "
              f"{fsl['correct_distinct_flights']}/{fsl['labelled_distinct_flights']} = "
              f"{fsl['fail_safe_correctness_distinct_flights']}"
              + (" - " + ", ".join(verdict) if verdict else ""))
        # The GeoFence backstop's share of that figure (2026-11-15,
        # BACKSTOP_RULE), always printed; 0 on a headless sweep by construction.
        nb = (fsl.get("triggered_by") or {}).get(FAILSAFE_BY_GEOFENCE, 0)
        b0, b1 = fsl["before_backstop_rule"], fsl["backstop_as_not_triggered"]
        print(f"fail-safe labels, of them triggered by the autopilot's GeoFence backstop "
              f"while the FSM held (counted since {BACKSTOP_RULE['since']}): {nb}"
              + (f"; the same runs read as before that rule ({b0['left_out_as_censored']} "
                 f"left out as censored): {b0['correct']}/{b0['labelled']} = "
                 f"{b0['fail_safe_correctness']}; counted as not triggered: "
                 f"{b1['correct']}/{b1['labelled']} = {b1['fail_safe_correctness']}"
                 if nb else ""))
    if arms_doc and (arms_doc["episodes_with_arms"]
                     > (arms_doc.get("prefix_consumer") or {}).get("episodes", 0)):
        # (Not printed for a run whose only arms are a prefix consumer's: it
        # has no plumbing pair, and its A/B is printed below.)
        print(f"arms: {arms_doc['episodes_with_arms']} episodes in "
              f"{arms_doc['cell_seed_groups']} (cell, seed) groups; flew differently: "
              f"{len(arms_doc['groups_whose_arms_flew_differently'])}; prefix A/B pairs "
              f"{arms_doc['prefix_ab_pairs']} (differ: "
              f"{len(arms_doc['prefix_ab_pairs_that_differ'])}) - plumbing only, the "
              f"headless pilot reads no text")
    if arms_doc and arms_doc.get("prefix_consumer"):
        pc = arms_doc["prefix_consumer"]
        print(f"prefix A/B through a pilot that consumes the CSP ({', '.join(pc['consumers'])}): "
              f"{pc['pairs']} on / off pair(s) ({pc.get('distinct_pairs')} distinct "
              f"flight pair(s)), {len(pc['pairs_that_differ'])} differ; "
              f"{pc['aa_pairs']} A/A pair(s), {len(pc['aa_pairs_that_differ'])} differ"
              + ("" if args.backend != "headless" else
                 " (headless: functional, not KPI-grade; an A/A pair that differs is a "
                 "harness defect)"))
        sys.path.insert(0, str(ROOT / "tools"))
        import kpi_report
        for line in kpi_report.prefix_ab_lines(pc.get("figures")):
            if line:
                print("  " + line)
    if n_known:
        print("Known failures are open defects, not passes. See the scenario's "
              "`description` for what is broken.")
    if rollup_note:
        # Worded from the write itself (report_md is set only where
        # kpi_report.md was written): keyed on `cells` alone, a --no-bundles
        # run whose top_failures crashed named a report it never wrote
        # (review of F8-harness-tools, P6).
        print(f"\nKPI ROLLUP FAILED: {rollup_note}. "
              + ("No KPI report was written" if report_md is None else
                 "The KPI report carries the failure at its top")
              + (", and the published sweep was NOT replaced" if args.publish else "")
              + " (exit 4).", file=sys.stderr)
    if args.publish and not rollup_note:
        print(f"\nPUBLISHED: {_show(out)} replaced. It is read by "
              "tools/build_eval_data.py, build_deck_data.py, "
              "build_architecture_svg.py and tests/test_kpi_magnitudes.py; add a "
              "CHANGELOG Changed/Retracted entry for what moved. The full run "
              f"file (with {', '.join(PUBLISHED_DROPS)}): "
              f"{_show(results_path(None, False, tag))}")
    if n_fail:
        return 1
    if rollup_note:
        # Not 1: no scenario failed; the harness could not summarise the run.
        return 4
    if not scored:
        print("\nNothing was scored: every episode was skipped (exit 3).")
        return 3
    return 0


if __name__ == "__main__":
    # A command-line run reports to the Dashboard (tools/live_status.py);
    # its heartbeat ends with the run, "failed" on a non-zero exit. An
    # argument that is a path shows as its last part only: the Dashboard
    # promises no folder names.
    _LIVE = _live_heartbeat("sweep " + " ".join(
        Path(a).name if ("/" in a or "\\" in a) else a for a in sys.argv[1:])[:80])
    _code = 1
    try:
        _code = main()
    finally:
        _LIVE.stop("ok" if not _code else "failed")
    sys.exit(_code)
