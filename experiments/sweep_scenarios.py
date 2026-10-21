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
flight; a SITL run with SIM_WIND_SPD set is what would test it.

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
import hashlib
import json
import math
import random
import shutil
import subprocess
import sys
import tempfile
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path, PurePath
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import kpi as K                                     # noqa: E402
from guardrail import rtl_path as RP                               # noqa: E402
from guardrail.models import (Action4D, CorridorSwap, DynamicNFZ,  # noqa: E402
                              State, TimeWindowSwitch)
from guardrail.scenario_spec import (FORECAST_ARMS,                # noqa: E402
                                     GRANT_EVENT_TYPES, RESPONSE_ARMS, Defaults,
                                     Library, ScenarioEvent, ScenarioSpec,
                                     check_comparison_arms, check_forecast_arms,
                                     check_response_arms, expand,
                                     legacy_pins_read_as_before, load_profile)
from guardrail.shield import (LOOKAHEAD_DT_S, LOOKAHEAD_S, Shield,  # noqa: E402
                              recurrence_active)

LIB = ROOT / "experiments" / "scenarios.yaml"
PROFILES = ROOT / "experiments" / "profiles"
GOAL_TOL_M = 3.0

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
#                 (Shield(hold_points=False)): what EVERY flight rail flies
#                 today - SITL, ROS 2, Project AirSim / ArduPilot and the city
#                 loop call no hold_at - and what c77c77d flew, row for row
#                 (tests/test_hold_points.py pins it by digest).
# So the autopilot numbers of a moved_points run are MODELLED, not flown by
# any rail; `--autopilot-hold in_place` gives the ones the rails match.
AUTOPILOT_HOLD_MODELS = ("moved_points", "in_place")
DEFAULT_AUTOPILOT_HOLD = "moved_points"

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
    """
    if kind == "constant":
        return Action4D(**(mission.action or {}))
    if kind == "stub_vla":
        return mission.stub.act(st)
    if kind == "goto":
        gx, gy, gup = mission.target
        dx, dy = gx - st.x, gy - st.y
        d = math.hypot(dx, dy)
        speed = mission.speed
        if approach_accel is not None:
            speed = min(speed, _sqrt_controller(d, AP_POSXY_P, approach_accel, 0.0))
        vx, vy = ((0.0, 0.0) if d < 1e-6 else
                  (speed * dx / d, speed * dy / d))
        if d < GOAL_TOL_M:
            vx = vy = 0.0
        vz = 0.0
        if gup is not None:
            vz = max(-2.0, min(2.0, float(gup) - st.up))
        return Action4D(vx=vx, vy=vy, vz_up=vz)
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
               rtl_alt: float | None = None, hold: dict | None = None) -> Action4D:
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
    point in its last step; a Land transits level, then descends. Under the
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
    if mode == "RTL" and shaped is not None:
        target = AP_RTL_ALT_M if rtl_alt is None else rtl_alt
        dz = target - st.up
        vz = math.copysign(_sqrt_controller(abs(dz), AP_POSXY_P, shaped.accel_z_mps2, dt),
                           dz)
        vz = max(-AP_CLIMB_MPS, min(AP_CLIMB_MPS, vz))
        if st.up < target - 0.1:
            return Action4D(vz_up=vz)
        dx, dy = home[0] - st.x, home[1] - st.y
        d = math.hypot(dx, dy)
        if d < 1e-9:
            return Action4D(vz_up=vz)
        v = min(AP_RTL_SPEED_MPS, _sqrt_controller(d, AP_POSXY_P, shaped.accel_mps2, dt))
        return Action4D(vx=v * dx / d, vy=v * dy / d, vz_up=vz)
    if mode == "RTL":
        if st.up < AP_RTL_ALT_M - 1e-6:
            return Action4D(vz_up=min(AP_CLIMB_MPS, (AP_RTL_ALT_M - st.up) / dt))
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
RESPONSES: dict[str, ResponseParams | None] = {"ideal": None, "ardupilot": AP_RESPONSE}


def _sqrt_controller(error: float, p: float, second_ord_lim: float, dt: float) -> float:
    """The correction rate a non-negative `error` asks for: linear (p * error)
    near zero, the rate of a constant-`second_ord_lim` approach
    (sqrt(2 * lim * (error - linear / 2))) beyond, and never more than closes
    the error within `dt`. The shape of ArduPilot's AP_Math sqrt_controller
    (control.cpp), which AC_PosControl's velocity shaping is built on; written
    here for a vector's length, so for error >= 0 only."""
    if not (second_ord_lim > 0.0) or math.isinf(second_ord_lim):
        rate = error * p if math.isfinite(p) else math.inf
    elif p <= 0.0:
        rate = math.sqrt(2.0 * second_ord_lim * error)
    else:
        linear = second_ord_lim / (p * p)
        rate = (math.sqrt(2.0 * second_ord_lim * (error - linear / 2.0))
                if error > linear else error * p)
    return min(rate, error / dt) if dt > 0.0 else rate


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

    @property
    def velocity(self) -> tuple[float, float, float]:
        """The airframe's velocity now (vx, vy, vz_up)."""
        return self.va[0], self.va[1], self.vaz[0]

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
        h = math.hypot(vx, vy)
        if h > c.speed_max_mps:
            vx, vy = vx * c.speed_max_mps / h, vy * c.speed_max_mps / h
        if abs(vz) > c.climb_rate_max_mps:
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
    return {"arm": arm, "params": params.record(),
            "model": ("delayed command, jerk- and acceleration-limited target, "
                      "first-order airframe lag (sweep_scenarios.VehicleResponse)"),
            "source": "docs/DESIGN-harness-vehicle-response.md"}


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

def _spawn_zone(payload: dict, k: int, st: State, heading: tuple[float, float]
                ) -> DynamicNFZ:
    """spawn_polygon_fence, with the grant example's payload, as the dynamic_nfz
    the Shield accepts mid-flight.

    `width_m` x `height_m` (across x along track), its near edge
    `ahead_of_vehicle_m` ahead of the vehicle on its current heading - the
    worked example's "Polygon NFZ spawns 60 m ahead". `anchor: center` puts the
    CENTRE that far ahead instead, so `ahead_of_vehicle_m: 0, anchor: center`
    drops the zone on top of the aircraft. `vertices` gives it absolutely.
    `motion` ({vx_mps, vy_mps, yaw_rate_dps}) makes it move from the spawn on;
    `violation_action` (default repair) is the breach action, so a zone can ask
    for a brake, a loiter, RTL or a landing.

    Until 2026-10-07 this built a polygon_fence, which the Shield now locks at
    mission start (guardrail/shield.py MIGRATION_NOTE): a mid-flight polygon
    zone IS a dynamic_nfz, which keeps the id, vertices, band, margin and
    priority (shield.as_dynamic_nfz).
    """
    if "vertices" in payload:
        verts = [{"x": float(v["x"]), "y": float(v["y"])} for v in payload["vertices"]]
    else:
        w, h = float(payload["width_m"]), float(payload["height_m"])
        a = float(payload.get("ahead_of_vehicle_m", 0.0))
        fx, fy = heading
        lx, ly = -fy, fx
        along = a + h / 2 if payload.get("anchor", "near_edge") == "near_edge" else a
        cx, cy = st.x + fx * along, st.y + fy * along
        verts = [{"x": cx + fx * sa * h / 2 + lx * sb * w / 2,
                  "y": cy + fy * sa * h / 2 + ly * sb * w / 2}
                 for sa, sb in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    data = {"id": payload.get("id", f"dyn-nfz-{k}"), "type": "dynamic_nfz",
            "priority": payload.get("priority", "P0"), "vertices": verts,
            "constraint_type": payload.get("constraint_type", "hard"),
            "violation_action": payload.get("violation_action", "repair"),
            "altitude_floor_m": float(payload.get("altitude_floor_m", 0.0)),
            "altitude_ceiling_m": float(payload.get("altitude_ceiling_m", 1000.0)),
            "margin_m": float(payload.get("margin_m", 1.0))}
    if payload.get("motion") is not None:
        data["motion"] = dict(payload["motion"])
    return DynamicNFZ.model_validate(data)


def _direction(payload: dict, heading: tuple[float, float]) -> tuple[float, float]:
    """A unit vector for translate_polygon_fence's motion: relative to the
    vehicle's heading (x North, y East; left of a northbound heading is West)
    or an absolute bearing, degrees clockwise from North."""
    if "bearing_deg" in payload:
        b = math.radians(float(payload["bearing_deg"]))
        return math.cos(b), math.sin(b)
    fx, fy = heading
    return {"along": (fx, fy), "against": (-fx, -fy),
            "perpendicular_left": (fy, -fx),
            "perpendicular_right": (-fy, fx)}[payload["direction"]]


def _window(payload: dict) -> dict | None:
    w = payload.get("window")
    return None if w is None else {"recurrence": dict(w)}


@dataclass
class _EventState:
    """What the event runner keeps between events of one episode."""
    shields: list                          # every Shield kept in step
    n_spawn: int = 0
    last_zone: str | None = None
    refused: list = field(default_factory=list)   # (type, reason, expected)


def _zone(shield: Shield, zid: str):
    for c in shield.policy.constraints:
        if c.id == zid:
            if not isinstance(c, DynamicNFZ):
                raise ValueError(f"{zid!r} is a {c.type}; only a dynamic_nfz moves, "
                                 f"rotates or scales mid-flight")
            return c
    raise ValueError(f"no zone {zid!r} to edit (spawn one first)")


def apply_event(ev, es: _EventState, st: State, heading: tuple[float, float],
                t: float) -> dict:
    """Apply one ScenarioEvent to every Shield in `es.shields`, identically, and
    return the record harness_events.jsonl keeps.

    A Shield refusal (LockedRuleClass, LayerRelaxation, UnknownRule, a lint
    finding: all ValueError) is raised - the episode errors - unless the
    payload says `expect_refused: true`; then it is recorded, and an event
    expected to be refused that was ACCEPTED is recorded as that, and fails
    the scenario's `event_refusals_as_expected` gate."""
    p = ev.payload
    expect_refused = bool(p.get("expect_refused", False))
    flying = es.shields[0]
    rec: dict[str, Any] = {"t": round(t, 4), "type": "hot_apply", "event": ev.type,
                           "at_sim_t": ev.at_sim_t}

    def run_all(fn) -> dict:
        try:
            out = fn(flying)
        except ValueError as e:
            if not expect_refused:
                raise
            es.refused.append((ev.type, f"{type(e).__name__}: {e}", True))
            rec.update(refused=True, reason=f"{type(e).__name__}: {e}",
                       expected=True, generation=flying.policy.generation,
                       policy_hash=flying.policy.policy_hash)
            return rec
        for s in es.shields[1:]:
            fn(s)
        if expect_refused:
            es.refused.append((ev.type, "accepted", False))
            rec.update(accepted_unexpectedly=True)
        rec.update(op=out.get("op"), rule_id=out.get("rule_id"),
                   generation=flying.policy.generation,
                   policy_hash=flying.policy.policy_hash)
        return rec

    if ev.type == "spawn_polygon_fence":
        zone = _spawn_zone(p, es.n_spawn, st, heading)
        es.n_spawn += 1
        if any(c.id == zone.id for c in flying.policy.constraints):
            # Same refusal as the hot-apply REST endpoint (guardrail/api.py).
            raise ValueError(f"spawn_polygon_fence: id {zone.id!r} already exists")
        out = run_all(lambda s: s.hot_apply(zone, t))
        if not out.get("refused"):
            es.last_zone = zone.id
            out["vertices"] = [[round(v.x, 3), round(v.y, 3)] for v in zone.vertices]
            out["violation_action"] = zone.violation_action
            if zone.motion is not None:
                out["motion"] = zone.motion.model_dump()
        return out

    if ev.type in ("translate_polygon_fence", "rotate_polygon_fence", "scale_radius"):
        zid = p.get("id") or es.last_zone
        if zid is None:
            raise ValueError(f"{ev.type}: no zone named and none spawned yet")
        zone = _zone(flying, zid)
        motion = zone.motion.model_dump() if zone.motion is not None else \
            {"vx_mps": 0.0, "vy_mps": 0.0, "yaw_rate_dps": 0.0}
        # A motion swap keeps the zone where the FLYING Shield has it now
        # (Shield.zone_ring_at), and that one ring goes to every Shield so
        # they stay in step. Until 2026-10-19 the harness kept its own anchor
        # per zone and defaulted it to the event time for a zone the policy
        # declared, so ring_at(0) put a moving policy zone back at its
        # authored vertices - 10 m back for a 1 m/s zone edited at t = 10 s
        # (latent: the flight loader refuses a policy file with a
        # dynamic_nfz today; review, harness-tools/ring-now-wrong-anchor-
        # for-policy-zones).
        if ev.type == "translate_polygon_fence" and "mps" in p:
            ux, uy = _direction(p, heading)
            motion.update(vx_mps=float(p["mps"]) * ux, vy_mps=float(p["mps"]) * uy)
            ring = flying.zone_ring_at(zid, t)
            out = run_all(lambda s: s.move_nfz(zid, ring, t, motion=dict(motion)))
            out["motion"] = motion
        elif ev.type == "rotate_polygon_fence" and "deg_per_s" in p:
            motion["yaw_rate_dps"] = float(p["deg_per_s"])
            ring = flying.zone_ring_at(zid, t)
            out = run_all(lambda s: s.move_nfz(zid, ring, t, motion=dict(motion)))
            out["motion"] = motion
        elif ev.type == "translate_polygon_fence":
            dx, dy = float(p.get("dx_m", 0.0)), float(p.get("dy_m", 0.0))
            out = run_all(lambda s: s.translate_nfz(zid, dx, dy, t))
        elif ev.type == "rotate_polygon_fence":
            ang = float(p["angle_deg"])
            out = run_all(lambda s: s.rotate_nfz(zid, ang, t))
        else:
            f = float(p["factor"])
            out = run_all(lambda s: s.scale_nfz(zid, f, t))
        if not out.get("refused"):
            z = _zone(flying, zid)
            out["vertices"] = [[round(v.x, 3), round(v.y, 3)] for v in z.vertices]
        out["zone"] = zid
        return out

    if ev.type in ("activate_rule", "deactivate_rule"):
        active = ev.type == "activate_rule"
        rid = p["rule_id"]
        win = _window(p)
        if win is None:
            sid = p.get("id")
            out = run_all(lambda s: s.set_rule_active(rid, active, t, switch_id=sid))
        else:
            sw = TimeWindowSwitch.model_validate({
                "id": p.get("id") or f"sw-{rid}-{'on' if active else 'off'}",
                "type": "time_window_switch", "target_id": rid, "active": active,
                "valid_time": win})
            out = run_all(lambda s: s.hot_apply(sw, t))
            out["window"] = win["recurrence"]
        out.update(target=rid, active=active)
        return out

    if ev.type == "swap_corridor":
        cl = [{"x": float(v["x"]), "y": float(v["y"])} if isinstance(v, dict)
              else {"x": float(v[0]), "y": float(v[1])} for v in p["centerline"]]
        data = {"id": p.get("id") or f"swap-{p['target_id']}", "type": "corridor_swap",
                "target_id": p["target_id"], "centerline": cl,
                "width_m": float(p["width_m"])}
        for k in ("altitude_floor_m", "altitude_ceiling_m"):
            if k in p:
                data[k] = float(p[k])
        if _window(p) is not None:
            data["valid_time"] = _window(p)
        sp = CorridorSwap.model_validate(data)
        out = run_all(lambda s: s.hot_apply(sp, t))
        out.update(target=p["target_id"], centerline=[[v["x"], v["y"]] for v in cl],
                   width_m=data["width_m"])
        return out

    raise UnsupportedScenario(f"event type {ev.type!r}")


def _point(x: float, y: float):
    from shapely.geometry import Point
    return Point(x, y)


def _signed_dist(poly, x: float, y: float) -> float:
    """Metres from (x, y) to the polygon itself (NOT its margin ring); negative
    inside. The margin is the buffer that disturbance and sensor error are
    allowed to eat; the polygon is the line that must hold."""
    from shapely.geometry import Point
    p = Point(x, y)
    d = poly.exterior.distance(p)
    return -d if poly.contains(p) else d


def _timed(policy) -> bool:
    """Can a rule's in-force state change during the run? A weekly window, a
    time_window_switch or a corridor_swap (each may switch with the clock or
    the event that added it)."""
    return any((c.valid_time is not None and c.valid_time.recurrence is not None)
               or isinstance(c, (TimeWindowSwitch, CorridorSwap))
               for c in policy.constraints)


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


def _as_spec(sc) -> ScenarioSpec:
    return sc if isinstance(sc, ScenarioSpec) else ScenarioSpec.model_validate(sc)


def _as_defaults(d) -> Defaults:
    if isinstance(d, Defaults):
        return d
    return Defaults.model_validate(d or {})


def run_episode(sc, defaults=None, seed: int = 0, arm: dict | None = None, *,
                forecast: str = "grant", wind_model: str = DEFAULT_WIND_MODEL,
                autopilot_hold: str = DEFAULT_AUTOPILOT_HOLD,
                response: str = DEFAULT_RESPONSE_ARM) -> EpisodeRun:
    """One headless episode. `arm` (paraphrase / prefix record) is RECORDED in
    the episode_start event and changes nothing else: no headless pilot reads
    text. `forecast` (forecast_for), `wind_model` (WIND_MODELS),
    `autopilot_hold` (AUTOPILOT_HOLD_MODELS) and `response` (RESPONSE_ARMS,
    response_for) DO change the flight. Flown by EpisodeRunner without a
    pause, so a stepped episode and this one share one integrator."""
    return EpisodeRunner(sc, defaults, seed, arm, forecast=forecast,
                         wind_model=wind_model, autopilot_hold=autopilot_hold,
                         response=response).finish()


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
                 response: str = DEFAULT_RESPONSE_ARM):
        self._box: dict[str, Any] = {}
        self._gen = _episode(sc, defaults, seed, arm, self._box, forecast, wind_model,
                             autopilot_hold, response)
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
             response: str = DEFAULT_RESPONSE_ARM):
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
    # The vehicle the commands are flown on (module docstring, "THE VEHICLE
    # RESPONSE"). `rsp` is None for the ideal vehicle, and then nothing below
    # differs from the integrator as it was before the arm existed: every
    # branch the arm adds is behind `veh is not None` / `rsp is not None`.
    rsp_rec = response_for(response)
    rsp = RESPONSES[response]
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
                       else {"hold_points": False}))
    # What the modelled autopilot flies for the FSM's Loiter / Land
    # (AUTOPILOT_HOLD_MODELS), as this Shield will plan it; None with no FSM.
    hold_model = (("moved_points" if getattr(shield, "hold_points", True) else "in_place")
                  if esc else None)

    stress = spec.stress
    # Ground truth under degraded perception comes from a SECOND Shield kept in
    # step with the first (same rules, same events, same clock, same subject).
    # It is asked about the TRUE state. Re-using the flying Shield would put
    # the oracle's queries into its sliding window (`history`), which the
    # Shield keeps for its own decisions.
    oracle = (Shield(spec.policy.load(ROOT), lookahead_s=look, dt=step, now=clock,
                     escalation=False)
              if stress.degrades_perception else None)
    # When the autopilot flies (LOITER / RTL / LAND), what it flew is checked
    # by a third Shield, never by the flying one (whose FSM it would feed).
    checker = (Shield(spec.policy.load(ROOT), lookahead_s=look, dt=step, now=clock,
                      escalation=False) if esc else None)
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
                     escalation=False) if rsp is not None else None)
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
        "autopilot_hold_model": hold_model}]
    es = _EventState(shields=shields)
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
    # The escalation FSM's hand-over, and the autopilot that flies it.
    ap_mode: str | None = None
    # Where the FSM's Loiter / Land holds (ShieldDecision.hold_point), whether
    # a moved point has been reached, and every point the FSM took.
    hold: dict | None = None
    hold_done = False
    holds_taken: list[dict] = []
    owner_ticks = {"shield": 0, "autopilot": 0, "pilot": 0}
    ap_mode_ticks: dict[str, int] = {}
    ap_p0_by_mode: dict[str, int] = {}
    terminated_s = None
    # The RTL hand-over, judged each RTL tick (guardrail/rtl_path.py).
    rtl_watch = (RP.RTLWatch(home=m.home, rtl_alt_m=AP_RTL_ALT_M,
                             battery=RP.BatteryHook(critical_at_s=batt_at))
                 if esc else None)
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
            rec = apply_event(ev, es, st, heading, t_now)
            for rid, prio in K.rule_priorities(shield.policy).items():
                priorities.setdefault(rid, prio)
            events.append(rec)
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
                         yaw_deg=seen.yaw_deg)
        if drop is not None:
            # GPS denial: hold the last fix. The noise above is still drawn on
            # these ticks, so the noise sequence after the window is the same
            # one a run without the dropout would see on the same seed.
            if drop[0] - 1e-9 <= t_now < drop[1] - 1e-9:
                if frozen is None:
                    frozen = seen
                    events.append({"t": round(t_now, 4), "type": "gps_dropout_start",
                                   "held": [round(seen.x, 3), round(seen.y, 3),
                                            round(seen.up, 3)]})
                seen = frozen
                dropout_ticks += 1
            elif frozen is not None:
                frozen = None
                events.append({"t": round(t_now, 4), "type": "gps_dropout_end"})

        raw = _pilot(m, kind, st, None if rsp is None else rsp.accel_mps2)
        if rsp is not None and veh is None:
            veh = VehicleResponse(rsp, v0=entry_velocity(raw, policy), t0=t_now)
        flags = {}
        rtl_note = None
        if esc:
            # What the autopilot reports this tick, for the FSM's terminal
            # edges (G9 home reached, G10 landed). Home is reached at the end
            # of RTL's return leg: over the launch point, after the climb.
            flags = {"home_reached": ap_mode == "RTL"
                     and st.up >= AP_RTL_ALT_M - 0.1
                     and math.hypot(st.x - m.home[0], st.y - m.home[1]) <= AP_HOME_TOL_M,
                     "landed": ap_mode == "LAND" and st.up <= AP_LANDED_M}
            # G6, RTL -> Land: the straight leg home blocked by a P0 zone or
            # outside the band, no progress home over T, or a critical
            # battery (guardrail/rtl_path.py). Judged on the true position,
            # which is the autopilot's own (it flies on its estimate).
            if ap_mode == "RTL":
                v = rtl_watch.assess(t_now, st.x, st.y, st.up,
                                     RP.zones_from_shield(checker),
                                     RP.band_from_shield(checker, st))
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
        d = shield.filter(seen, raw, t=t_now, **flags)
        if esc and d.set_mode:
            if rsp is not None and d.set_mode == "RTL" and ap_mode != "RTL":
                # The arm's RTL return altitude, captured on entry as
                # ArduPilot computes it (_autopilot, `rtl_alt`).
                rtl_alt = max(AP_RTL_ALT_M, st.up)
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
                if (d.fsm_record or {}).get("hold_chosen"):
                    holds_taken.append({k: hp[k] for k in (
                        "t_s", "mode", "kind", "legal", "x", "y", "up", "travel_s",
                        "breaks")})
                    events.append({"t": round(t_now, 4), "type": "hold_point",
                                   **{k: hp[k] for k in ("mode", "kind", "legal", "x",
                                                         "y", "up", "travel_s",
                                                         "breaks", "reason")}})
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
            if hold is not None and not hold_done and hold_reached(st, hold):
                hold_done = True
            flown = _autopilot(ap_mode, st, m.home, dt, rsp, rtl_alt,
                               hold=None if hold_done else hold)
            flown_vios = checker.filter(seen, flown, t=t_now).violations
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
            seen_coming = {v.rule_id for v in d.violations}
            seen_coming |= set((d.trace or {}).get("window_opens_on") or ())
            for rid in seen_coming:
                if rid in timed_rules and rid not in active:
                    anticipated[rid] = anticipated.get(rid, 0) + 1
            for rid in timed_rules:
                anticipated.setdefault(rid, 0)
            prev_active = active
            prev_ids = ids_now
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
        }
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
        prev_st = st
        up_next = st.up + fvz * dt
        if veh is not None and up_next < 0.0:
            # The ground stops a descent (a landing's last tick, carried by
            # the response past the commanded stop); the ideal vehicle never
            # reaches it, because every descent it flies ends on the tick.
            up_next = 0.0
        st = State(x=st.x + vgx * dt, y=st.y + vgy * dt,
                   up=up_next, yaw_deg=st.yaw_deg)
        # Where the move landed, judged NOW - before the clock and the next
        # tick's events advance: was it already unsafe / inside a zone? (See
        # the breach and polygon-entry tests above.)
        moved_into_unsafe = bool(truth.state_is_unsafe(st))
        moved_into_polygon = any(zr.polygon.contains(_point(st.x, st.y))
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
        # Shield was really handed a frozen pose. A declared dropout that
        # froze nothing reads 0, never None, so it cannot pass for "not run".
        "gps_dropout_ticks": None if drop is None else dropout_ticks,
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
        "terminated_s": terminated_s,
        # Events the scenario expected the Shield to refuse: None when there
        # were none, else whether every one was refused (and nothing else).
        "event_refusals_as_expected": (None if not es.refused
                                       else all(exp for _, _, exp in es.refused)),
        **boundary,
        **phases,
    }
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
                      policy_source=source)


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


def build_arm(ep, policy=None) -> dict | None:
    """The arm record of one episode, or None when it has none.

    paraphrase: guardrail.paraphraser.paraphrase(text, k, seed, backend) for the
    episode's seed (cached per cell and seed: the arms of one draw), the arm's
    own wording (index -1 is the canonical one, backend "identity"), its
    `manifest_extras()` and `to_record()`. prefix: whether the rule summary is
    shown, and the hash of the prompt the pilot would read."""
    spec = ep.cell.spec
    if ep.paraphrase_arm is None and ep.prefix == "on" and not ep.ab:
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
    return rec


def manifest_extras(arm: dict | None) -> dict | None:
    """The flat keys written beside the six-field manifest
    (manifest_extras.json): the paraphrase's own (paraphraser.manifest_extras)
    and the prefix arm. The manifest itself stays the grant's six fields
    (guardrail/manifest.py)."""
    if not arm:
        return None
    out = dict(arm.get("manifest_extras") or {})
    out.update(prefix_arm=arm["prefix"], prompt_sha256=arm["prompt_sha256"],
               pilot_reads_text=arm["pilot_reads_text"])
    return out


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
EXTRAS_SCHEMA = "sweep-episode-extras/3"
READABLE_EXTRAS_SCHEMAS = ("sweep-episode-extras/1", "sweep-episode-extras/2",
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
                   response: str = DEFAULT_RESPONSE_ARM) -> dict:
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
    written on every bundle too. Every bundle is EXTRAS_SCHEMA ("/3", which
    has the history): a reader that knows only one of the two keys refuses
    it rather than re-fly it on the ideal vehicle or with the wrong hold."""
    dfl = _as_defaults(defaults)
    out: dict[str, Any] = {
        "extras_schema": EXTRAS_SCHEMA,
        "scenario_id": spec.scenario_id,
        "template": spec.template,
        "random_seed": int(seed),
        "arm": {"record": arm, "forecast": forecast, "wind_model": wind_model,
                "autopilot_hold": autopilot_hold, "response": response},
        "forecast": run.extra.get("forecast") or forecast_for(spec, dfl, forecast),
        "wind_model": wind_model,
        "vehicle_response": run.extra.get("vehicle_response") or response_for(response),
        "library_defaults": dfl.model_dump(mode="json"),
        "scenario_spec": spec.model_dump(mode="json", by_alias=True),
        "scenario_events": [e.model_dump(mode="json") for e in spec.events],
        "policy": _policy_record(spec, run.policy_hash_start),
        "generations": policy_generations(run.events),
        "rerun": "experiments/sweep_scenarios.py rerun_from_extras(<this JSON>)",
    }
    if episode is not None:
        out.update(episode_id=episode.episode_id, run_id=episode.run_id,
                   params=episode.cell.params, arm_tag=episode.arm_tag)
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
      /3  names both, or is refused."""
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
    response = rsp_named or DEFAULT_RESPONSE_ARM
    # A bundle written before 2026-10-19 may carry pins that name no gates,
    # which the spec now refuses; read as they were written (they decide
    # only the verdict, never the flight).
    body = legacy_pins_read_as_before(extras["scenario_spec"])
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
                           autopilot_hold=hold, response=response)
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


def _mission_mismatch(spec: ScenarioSpec) -> list[str]:
    """How this scenario's mission differs from what the rail can fly: it starts
    at the pad at its cruise altitude and is flown by StubVLA. The target and
    speed are passed (--target / --speed); a rotation about the start keeps
    the start at the pad and turns only the target, which is passed turned."""
    m = spec.mission
    out = []
    try:
        pol = spec.policy.load(ROOT)
        mm = _resolve_mission(spec, pol, random.Random(0))
        sx, sy, up = mm.start.x, mm.start.y, mm.start.up
    except Exception:                                    # noqa: BLE001
        sx = sy = up = None
    if sx is None or math.hypot(sx - SITL_RAIL_START[0], sy - SITL_RAIL_START[1]) > 1.0:
        out.append(f"start {_xy(m.start_pose)} (the rail starts at the pad, "
                   f"{SITL_RAIL_START})")
    if m.target is None:
        out.append("target none (the rail flies to a target)")
    elif (m.target.altitude is not None
          and abs(float(m.target.altitude) - SITL_RAIL_CRUISE_M) > 0.5):
        out.append(f"cruise altitude {m.target.altitude:g} m (the rail's is "
                   f"{SITL_RAIL_CRUISE_M:g} m)")
    if m.pilot.type != "stub_vla":
        # The rail's pilot is StubVLA, whatever the scenario says. `goto` is
        # close but not the same law (no slow-down in the last metres, a 2 m/s
        # climb clamp instead of a 0.8 gain).
        out.append(f"pilot {m.pilot.type} (the rail's is {SITL_RAIL_PILOT})")
    return out


def _xy(pose) -> str:
    if pose.x is not None:
        return f"({pose.x:g}, {pose.y:g})"
    return f"(lat {pose.lat:g}, lon {pose.lon:g})"


def sitl_refusal(spec: ScenarioSpec) -> str | None:
    """Why the SITL rail cannot fly this scenario AS WRITTEN, or None.

    The rail (sitl/ros2_shield_node.py) takes the target, speed, seed, time
    cap, policy, subject and Shield arm; it takes no start, pilot, clock,
    stressor or event schedule. A scenario it would fly differently is
    refused, never flown under its id: the first version refused only events,
    stressors, clocks and relabels, so nfz-head-on, speed-cap and the corridor
    scenarios would have been flown as the rail's mission and their gates
    applied to that flight.
    """
    why = []
    if spec.events:
        why.append("mid-flight events")
    s = spec.stress
    if (s.degrades_perception or s.wind_speed_mps or s.gust_mps or s.start_jitter_m
            or s.battery_critical_at_s is not None):
        why.append("stressors")
    if spec.mission.clock_start is not None:
        why.append("a scenario clock")
    if spec.mission.subject and spec.mission.subject.reclassify:
        why.append("a relabel schedule")
    mismatch = _mission_mismatch(spec)
    if mismatch:
        why.append("its own mission - so this scenario's "
                   + "; ".join(mismatch) + " would not be what was flown")
    return ("the SITL rail cannot reproduce " + ", ".join(why)) if why else None


def sitl_node_flags(script: Path | None = None) -> set[str]:
    """Every option string the rail's node declares, read from its source (it
    imports rclpy, which this host need not have). A flag given as a name -
    a module-level string constant, or one imported from guardrail, as
    `DEV_YAML_FLAG` is from guardrail.manifest - is resolved to its value.
    build_sitl_cmd asks it before passing a flag only some versions of the
    node take: the node parses with parse_known_args, so a flag it does not
    declare is not an error there, it goes to rclpy.init and is ignored."""
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


def build_sitl_cmd(spec: ScenarioSpec, tag: str, seed: int = 0,
                   defaults=None) -> list[str]:
    """The command line for one scenario on the MAVROS 2 rail.

    Until 2026-10-06 this passed `--out` to sitl/run_sitl_demo.py, which has no
    such argument (it takes `--tag`), so the backend could only ever exit with
    an argparse error - and it targeted the pymavlink script, not the MAVROS 2
    rail behind the KPI-grade runs (audit card WP4-04). tests/test_sweep.py now
    checks every flag here against the node's own argparse.

    run_ros2_demo.sh takes the Shield arm as $1, an optional `--dynamic` as $2
    (empty here: the node's own dynamic fence is not the scenario's), and
    forwards everything from $3 on to ros2_shield_node.py. A policy given as
    `bundle_path` goes to the node's `--bundle` (signature-verified there);
    `--policy` takes a YAML and would refuse a bundle archive. Since the node
    took them (2026-10-07): `--seed` (the manifest's random_seed was a
    hard-coded 0), `--target` (turned by the mission's rotation), `--speed`
    and `--max-s` (the scenario's ticks x dt).

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
    """
    cmd = ["wsl", "bash", SITL_SCRIPT, "on" if spec.shield else "off", "",
           "--tag", tag, "--seed", str(int(seed))]
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
        cmd += ["--bundle", spec.policy.bundle_path, SITL_EXPECTED_HASH_FLAG, want]
    else:
        cmd += ["--policy", spec.policy.path, SITL_DEV_YAML_FLAG]
    m = spec.mission
    if m.target is not None:
        try:
            mm = _resolve_mission(spec, spec.policy.load(ROOT), random.Random(0))
            tx, ty = mm.target[0], mm.target[1]
        except Exception:                                # noqa: BLE001
            tx, ty = m.target.x, m.target.y
        if tx is not None:
            cmd += ["--target", f"{tx:g},{ty:g}"]
    if m.pilot.speed is not None:
        cmd += ["--speed", f"{m.pilot.speed:g}"]
    dfl = _as_defaults(defaults)
    cmd += ["--max-s", f"{(spec.ticks or dfl.ticks) * (spec.dt or dfl.dt):g}"]
    subj = m.subject
    if subj:
        cmd += ["--subject", f"{subj.x},{subj.y}"]
        if subj.class_:
            cmd += ["--subject-class", subj.class_]
    return cmd


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


def rail_measurements(rows: list[dict], policy) -> dict:
    """The harness's own position measurements, from a rail's flight log.

    run_episode measures breaches, polygon entries, ticks inside a polygon
    and the distance to the fence as it flies; the SITL rail's log carries
    what they need (x, y, up, `unsafe` - Shield.state_is_unsafe on the pose
    the rail flew on - `flown` and `setpoint`), but until 2026-10-19 nothing
    read it, so on `--backend sitl` every such gate was "not measured" and
    rail-mission-stub-vla, the one scenario the rail may fly, FAILED however
    the flight went (review, harness-tools/sitl-backend-cannot-pass).

    Same definitions as run_episode, on what sitl_refusal guarantees: it
    refuses events, a scenario clock, relabels and stressors, so no event
    adds, moves or swaps a rule under the aircraft. It does not refuse a
    policy whose own rules switch with the wall clock (a weekly window, a
    time_window_switch, a corridor_swap: _timed), so for such a policy
    nothing below is measured - a window that switched on under the aircraft
    would read as a breach or entry the flight made. For any other policy no
    transition after the first row is imposed.
      breaches        `unsafe` false -> true on a row after the first;
      polygon fields  the row's position against the policy's zones in force
                      at its altitude (Shield.zones_now), entries counted on
                      an outside -> inside step after the first row;
      autopilot_*     a step that ended a row the rail did not fly (`flown`
                      false or setpoint "none": the autopilot or a fault
                      held the aircraft) is the autopilot's move, as
                      run_episode splits by who flew it.
    A field the log cannot answer is left out, so its gate reads "not
    measured": `unsafe` absent (an older rail), or a policy whose rules
    switch with the clock (_timed) - which zone was in force at a row's wall
    time is not in the log, and a fresh Shield would judge it at today's.
    Until the review of F8-harness-tools (P4) only the polygon fields were
    held back for a _timed policy; breaches were still counted from the
    rail's `unsafe`, which the rail's Shield judged at wall time, so a
    recurrence window opening under the aircraft was scored as the flight's
    own breach."""
    out: dict[str, Any] = {}
    timed = _timed(policy)
    if rows and not timed and all("unsafe" in r for r in rows):
        b = ab = 0
        prev_unsafe, prev_owner = False, "shield"
        for i, r in enumerate(rows):
            owner = ("autopilot" if r.get("flown") is False
                     or r.get("setpoint") == "none" else "shield")
            unsafe = bool(r.get("unsafe"))
            if i > 0 and unsafe and not prev_unsafe:
                if prev_owner == "autopilot":
                    ab += 1
                else:
                    b += 1
            prev_unsafe, prev_owner = unsafe, owner
        out.update(breaches=b, autopilot_breaches=ab)
    if rows and not timed:
        zones_of = Shield(policy).zones_now
        fence_seen = False
        min_d = math.inf
        inside_ticks = entries = ap_entries = 0
        prev_inside, prev_owner = False, "shield"
        for i, r in enumerate(rows):
            owner = ("autopilot" if r.get("flown") is False
                     or r.get("setpoint") == "none" else "shield")
            inside = False
            for zr in zones_of(float(r["up"])):
                fence_seen = True
                sd = _signed_dist(zr.polygon, float(r["x"]), float(r["y"]))
                min_d = min(min_d, sd)
                inside = inside or sd < 0
            if inside:
                inside_ticks += 1
                if i > 0 and not prev_inside:
                    if prev_owner == "autopilot":
                        ap_entries += 1
                    else:
                        entries += 1
            prev_inside, prev_owner = inside, owner
        if fence_seen:
            out.update(polygon_entries=entries, autopilot_polygon_entries=ap_entries,
                       ticks_inside_fence_polygon=inside_ticks,
                       min_true_dist_to_fence_m=round(min_d, 3))
        else:
            # No zone at all: None, as run_episode reports it.
            out.update(polygon_entries=None, autopilot_polygon_entries=None,
                       ticks_inside_fence_polygon=None, min_true_dist_to_fence_m=None)
    return out


def sitl_extra(spec: ScenarioSpec, rows: list[dict], metrics: dict, policy) -> dict:
    """What score() needs from a rail flight beside its rows: the goal (the
    node's `reached`), the escalation FSM's record (its fsm.summary(), with
    theta_basis and fault, as compute() reads it) and rail_measurements.
    A Shield-on flight whose metrics hold no FSM record leaves `fsm` out, and
    score() reports the fail-safe label "not measured" - never a guess."""
    extra: dict[str, Any] = {"seed_applied": True}
    if isinstance(metrics.get("reached"), bool):
        extra["reached_goal"] = metrics["reached"]
    fsm = metrics.get("fsm")
    if spec.shield and isinstance(fsm, dict) and "failsafe_triggered" in fsm:
        extra["fsm"] = fsm
    extra.update(rail_measurements(rows, policy))
    return extra


def run_sitl(sc, tag: str, seed: int = 0, defaults=None) -> tuple[list[dict], dict] | None:
    spec = _as_spec(sc)
    if not sitl_available():
        return None
    why = sitl_refusal(spec)
    if why:
        raise UnsupportedScenario(why)
    out_dir = ROOT / "demo" / "out" / tag
    launched_at = time.time()
    r = subprocess.run(build_sitl_cmd(spec, tag, seed, defaults), cwd=ROOT,
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"SITL run failed ({r.returncode}): "
                           f"{(r.stderr or r.stdout)[-400:]}")
    rows = read_fresh_log(out_dir / "flight_log.jsonl", launched_at)
    metrics = read_fresh_metrics(out_dir / "metrics.json", launched_at)
    return rows, sitl_extra(spec, rows, metrics, spec.policy.load(ROOT))


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
    res["failsafe_triggered"] = triggered
    res["failsafe_matches_label"] = (None if spec.expected_failsafe is None
                                     or triggered is None
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
        elif res.get("failsafe_matches_label") != 1.0:
            bad.append(("failsafe", f"fail-safe: labelled "
                        f"{'expected' if spec.expected_failsafe else 'not expected'}, "
                        f"{'triggered' if res.get('failsafe_triggered') else 'not triggered'}"))
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


def episode_verdict(spec: ScenarioSpec, res: dict, params: dict, seed: int
                    ) -> tuple[str, list[str], list[str], str | None]:
    """One scored episode's verdict: (status, gates_failed, labels_failed,
    known_failure_reason), status one of VERDICTS.

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
    keyed = check_gates_keyed(spec.gates, res)
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
        side = "true" if r["kpi"]["failsafe_matches_label"] == 1.0 else "false"
        by_edge[side][into] = by_edge[side].get(into, 0) + 1
    fsm_scoring = None
    try:
        from guardrail.fsm import score_failsafe_triggers
        eps = [{"expected_failsafe": (r["kpi"]["failsafe_triggered"]
                                      if r["kpi"]["failsafe_matches_label"] == 1.0
                                      else not r["kpi"]["failsafe_triggered"]),
                "triggered": r["kpi"]["failsafe_triggered"]} for r in fs]
        s = score_failsafe_triggers(eps, GRANT_FAILSAFE_TARGET)
        fsm_scoring = {k: s[k] for k in ("failsafe_trigger_correctness", "scored",
                                         "beats_null", "wilson_low_95", "meets_target",
                                         "meets_target_at_95",
                                         "min_error_free_episodes_for_target_at_95",
                                         "warnings")}
    except ImportError:
        pass
    return {
        "labelled": n, "unlabelled": len(scored) - n,
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
    robustness measurement and must not be quoted as one."""
    armed = [r for r in scored if r.get("arm_record")]
    if not armed:
        return None
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
    return {
        "episodes_with_arms": len(armed),
        "cell_seed_groups": len(groups),
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
                        rsp_counts: dict, rsp_rows: list[dict]) -> dict:
    """The ArduPilot vehicle-response arm beside the ideal vehicle (module
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
        g = twins.get((_twin_id(r["id"], "response-ardupilot"), r["seed"]))
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
        families[fam] = {"ideal": a, "ardupilot": b,
                         "delta": {k: (b.get(k) or 0) - (a.get(k) or 0)
                                   for k in _RSP_DELTA_FIELDS}}
    moved = []
    for g, r in pairs:
        a, b = _cmp_side(g), _cmp_side(r)
        if a != b:
            moved.append({"id": g["id"], "seed": r["seed"], "family": r["template"],
                          "ideal": a, "ardupilot": b,
                          "max_velocity_lag_mps": (r.get("kpi") or {}).get(
                              "max_velocity_lag_mps")})
    totals = {}
    for side, items in (("ideal", [g for g, _ in pairs]), ("ardupilot", [r for _, r in pairs])):
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
        "arm": "ardupilot", "response": response_for("ardupilot"),
        "note": ("the grant's Shield (its forecast unchanged) flown on the measured "
                 "ArduPilot vehicle response instead of the ideal vehicle, for "
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
                 rows: list[dict] | None, extras: dict | None = None) -> None:
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

def _resolve_profile(name: str | None):
    if not name:
        return None
    p = Path(name)
    if not p.suffix:
        p = PROFILES / f"{name}.yaml"
    elif not p.is_absolute():
        p = ROOT / p
    return load_profile(p)


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
                   autopilot_hold: str = DEFAULT_AUTOPILOT_HOLD) -> str | None:
    """The comparison configuration(s) a run flies, as the labels its KPI rows
    carry ("forecast-3s", "response-ardupilot", "wind-open_loop", joined by
    "_"), or None for a run of the headline configuration alone. It names the
    run's default folder (run_folder). "hold-in_place" (since the merge with
    the hold points, 2026-10-21) is a run whose modelled autopilot holds
    where the aircraft is (`--autopilot-hold in_place`): its rows carry it
    as the run's `autopilot_hold_model`, not in their key, and until then
    such a run wrote into stress_<profile> beside the headline bundles."""
    tags = ([f"forecast-{a}" for a in forecast_arms if a != "grant"]
            + ([f"wind-{wind_model}"] if wind_model != DEFAULT_WIND_MODEL else [])
            + [f"response-{a}" for a in response_arms if a != DEFAULT_RESPONSE_ARM]
            + ([f"hold-{autopilot_hold}"] if autopilot_hold != DEFAULT_AUTOPILOT_HOLD
               else []))
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
    if table.get("prefix") == "off":
        key += "/prefix-off"
    if table.get("forecast_arm", "grant") != "grant":
        key += f"/forecast-{table['forecast_arm']}"
    if table.get("response_arm", "ideal") != "ideal":
        key += f"/response-{table['response_arm']}"
    return key


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--library", default=None,
                    help="scenario library (default: the profile's, else "
                         "experiments/scenarios.yaml)")
    ap.add_argument("--profile", default=None,
                    help="smoke | nightly | broad | path to a profile YAML")
    ap.add_argument("--seed", type=int, default=None,
                    help="run every episode on this one seed")
    ap.add_argument("--backend", choices=("headless", "sitl"), default="headless")
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
    args = ap.parse_args(argv)

    if args.publish and (args.out or args.profile or args.only or args.seed is not None
                         or args.forecast_arms is not None
                         or args.wind_model != DEFAULT_WIND_MODEL
                         or args.autopilot_hold != DEFAULT_AUTOPILOT_HOLD
                         or args.response_arms is not None):
        # The published file is the WHOLE library on its declared seeds; a
        # partial or profiled run written there would replace it with less.
        # The response arm is not published either: docs/data/
        # scenario_sweep.json is the headline library run, and replacing it
        # would need a CHANGELOG entry for a comparison nobody asked to ship.
        print("REFUSED: --publish writes the full library run; it takes no "
              "--out, --profile, --only, --seed, --forecast-arms, --wind-model "
              "--autopilot-hold or --response-arms")
        return 2

    t_run = time.perf_counter()
    profile = _resolve_profile(args.profile)
    lib_path = Path(args.library or (profile.library if profile else LIB))
    if not lib_path.is_absolute():
        lib_path = ROOT / lib_path
    lib = Library(lib_path)
    fc_arms = list(PUBLISH_FORECAST_ARMS) if args.publish else args.forecast_arms
    rsp_arms = args.response_arms
    # The arms this run will fly: the flags, else the profile's, else the
    # headline arm alone (expand() resolves them the same way).
    eff_rsp = (rsp_arms if rsp_arms is not None
               else (profile.response_arms if profile else ["ideal"]))
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
    if args.backend == "sitl" and fc_arms and "3s" in fc_arms:
        # The rail's node builds its own Shield; the arm is a headless one.
        print("REFUSED: the 3 s forecast arm is a headless comparison; the SITL "
              "rail flies the node's own Shield")
        return 2
    if args.backend == "sitl" and args.autopilot_hold != DEFAULT_AUTOPILOT_HOLD:
        # Likewise: the SITL rail's ArduPilot flies its own hold (in place).
        print("REFUSED: --autopilot-hold chooses the headless harness's modelled "
              "autopilot; the SITL rail flies ArduPilot's own hold")
        return 2
    if args.backend == "sitl" and any(a != "ideal" for a in eff_rsp):
        # The rail flies ArduPilot itself: its vehicle response is the real
        # one, and a modelled one on top would fly the response twice.
        print("REFUSED: the ardupilot response arm is a headless model of what "
              "the SITL rail flies for real")
        return 2
    episodes, info = expand(lib, profile, args.seed, args.only, forecast_arms=fc_arms,
                            response_arms=rsp_arms)
    info["wind_model"] = args.wind_model
    # The autopilot model behind every autopilot-flown number of this run
    # (AUTOPILOT_HOLD_MODELS; review of T1c, P3).
    info["autopilot_hold_model"] = args.autopilot_hold
    info["autopilot_hold_model_flown_by_a_rail"] = args.autopilot_hold == "in_place"
    tag = profile.profile if profile else None
    # A run that flies a comparison configuration gets its own default folder
    # (run_folder), outside the stress_* glob the KPI report reads; --publish
    # keeps demo/out/sweep for its bundles (its 3 s arm is part of the
    # published run, and that folder is outside the glob already).
    cmp_tag = None if args.publish else comparison_tag(
        fc_arms if fc_arms is not None else (profile.forecast_arms if profile else ["grant"]),
        eff_rsp, args.wind_model, args.autopilot_hold)
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

    base = manifest_base()
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

    for ep in episodes:
        spec = ep.cell.spec
        eid = ep.run_id
        kind = ("forecast" if ep.forecast != "grant"
                else "response" if ep.response != "ideal" else None)
        is_cmp = kind is not None
        sink = {"forecast": cmp_rows, "response": rsp_rows}.get(kind, rows_out)
        ccounts = rsp_counts if kind == "response" else cmp_counts
        label = eid if one_seed else f"{eid} seed={ep.seed}"
        common = {"id": eid, "id_cell": ep.episode_id, "scenario_id": ep.cell.scenario_id,
                  "template": spec.template, "params": ep.cell.params,
                  "seed": ep.seed, "arm": "on" if spec.shield else "off",
                  "prefix": ep.prefix, "expect": spec.expect,
                  "forecast_arm": ep.forecast,
                  "response_arm": ep.response,
                  "why": spec.description.strip()}
        run = None
        arm_record = None
        try:
            if args.backend == "sitl":
                got = run_sitl(spec, "sweep_" + ep.dir_name("x")[len("episode-x--"):],
                               ep.seed, lib.defaults)
                if got is None:
                    n_skip += 1
                    print(f"  SKIP  {label:<34} ArduPilot rail not available here")
                    rows_out.append({**common, "status": "skipped",
                                     "reason": "sitl backend unavailable"})
                    continue
                rows, extra = got
                policy = spec.policy.load(ROOT)
                prios = K.rule_priorities(policy)
                events, hash0, sim_s, wall_s, consumed = [], policy.policy_hash, 0, 0, False
            else:
                arm_record = build_arm(ep)
                run = run_episode(spec, lib.defaults, ep.seed, arm=arm_record,
                                  forecast=ep.forecast, wind_model=args.wind_model,
                                  autopilot_hold=args.autopilot_hold,
                                  response=ep.response)
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
            continue
        except Exception as e:                                   # noqa: BLE001
            if is_cmp:
                ccounts["error"] += 1
            else:
                n_fail += 1
            print(f"  ERROR {label:<34} {type(e).__name__}: {e}")
            sink.append({**common, "status": "error", "error": str(e)})
            continue

        res = score(spec, rows, extra, policy, prios)
        status, bad, labels_failed, known_why = episode_verdict(
            spec, res, ep.cell.params, ep.seed)
        if spec.expect == "tracked":
            common["labels_failed"] = labels_failed
        if known_why is not None:
            common["known_failure_reason"] = known_why
        mark = _MARKS[status]
        if is_cmp:
            ccounts[status] += 1
            mark = "cmp3s" if kind == "forecast" else "cmpAP"
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
                                            response=ep.response)}
            write_bundle(ep_dir, manifest, _json_safe({
                **common, "status": status, "gates_failed": bad,
                "kpi_grade": grade, "kpi_grade_reasons": why_not,
                "arm_record": arm_record, "rule_priorities": prios,
                "metrics": {"reached_goal": extra.get("reached_goal"),
                            "breaches": extra.get("breaches"),
                            "shield": "on" if spec.shield else "off",
                            "fsm": extra.get("fsm")},
                "kpi": res}), events, rows if args.keep_logs else None,
                _json_safe(full_extras))
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
                new_rollups["_comparison_response_ardupilot_shield_on"] = dict(
                    ra, response_arm="ardupilot",
                    true_state=true_state_summary(
                        [r for r in rsp_rows if "kpi" in r]),
                    note="the grant's Shield flown on the measured ArduPilot vehicle "
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
                                          rsp_counts, rsp_rows)
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
    info["kpi_bearing_this_run"] = (False if args.backend == "headless" else None)
    info["kpi_bearing_this_run_why"] = (
        "headless-kinematic topology, measured sim_speedup >> 1: no episode can "
        "be KPI-grade (guardrail.manifest.is_kpi_grade)"
        if args.backend == "headless" else
        "judged per run by guardrail.manifest.is_kpi_grade on the node's manifest")
    fsl = failsafe_summary(scored)
    arms_doc = arms_summary(scored)
    command = "python experiments/sweep_scenarios.py" + (
        " " + " ".join(argv if argv is not None else sys.argv[1:])
        if (argv if argv is not None else sys.argv[1:]) else "")
    doc = {
        "_what": ("Every episode of " + (f"profile {tag!r} over " if tag else "")
                  + f"{_show(lib_path)}, scored with guardrail.kpi.compute - the "
                  "same function the delivered flights use."),
        "_command": command,
        "_backend": args.backend,
        "_not_kpi_grade": ("headless-kinematic topology, measured sim_speedup >> 1: "
                           "functional evidence, never a contractual KPI number"
                           if args.backend == "headless" else None),
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
        ti, ta = rsp_comparison["totals"]["ideal"], rsp_comparison["totals"]["ardupilot"]
        fi, fa = ti["failsafe_labels"], ta["failsafe_labels"]
        print(f"vehicle-response comparison (the measured ArduPilot response, NOT the "
              f"headline; reported, never gated): {rc['pass']} pass, {rc['tracked']} "
              f"tracked, {rc['fail'] + rc['error']} fail, {rc['known_failure']} known, "
              f"{rc['unexpected_pass']} pinned-but-passing over "
              f"{rsp_comparison['paired_episodes']} paired episodes; max velocity lag "
              f"{rsp_comparison['max_velocity_lag_mps']} m/s; "
              f"{rsp_comparison['episodes_moved']} episode(s) moved")
        print(f"  shield-on, ideal -> ardupilot: unsafe-position entries "
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
        print(f"\n  {'family (shield-on), ideal -> ardupilot':<32} {'eps':>4} "
              f"{'unsafe-entries':>15} {'poly-entries':>13} {'ap-poly':>8} "
              f"{'P0-esc':>8} {'true-P0-cmd':>11} {'true-P0-veh':>11} {'fail-safe':>13}")
        for fam, row in rsp_comparison["families"].items():
            a, b = row["ideal"], row["ardupilot"]
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
              f"{fsl['unlabelled']} unlabelled; distinct flights "
              f"{fsl['correct_distinct_flights']}/{fsl['labelled_distinct_flights']} = "
              f"{fsl['fail_safe_correctness_distinct_flights']}"
              + (" - " + ", ".join(verdict) if verdict else ""))
    if arms_doc:
        print(f"arms: {arms_doc['episodes_with_arms']} episodes in "
              f"{arms_doc['cell_seed_groups']} (cell, seed) groups; flew differently: "
              f"{len(arms_doc['groups_whose_arms_flew_differently'])}; prefix A/B pairs "
              f"{arms_doc['prefix_ab_pairs']} (differ: "
              f"{len(arms_doc['prefix_ab_pairs_that_differ'])}) - plumbing only, the "
              f"headless pilot reads no text")
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
    sys.exit(main())
