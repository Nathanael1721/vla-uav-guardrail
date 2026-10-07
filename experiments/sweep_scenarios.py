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
report for the run (one row per (template, parameter cell)). Until 2026-10-06 a
plain run overwrote docs/data/scenario_sweep.json, a published artefact that the
evaluation data, the deck and tests/test_kpi_magnitudes.py read; replacing it
now takes `--publish`, and a CHANGELOG entry.

EXIT CODES. 0: every gated episode passed (known failures and tracked
episodes are reported, not failed). 1: a gate failed, an episode errored, or a
pinned defect started passing. 2: refused before running - an `--only` name
that matched nothing, a run of zero episodes, two cells that fly the same
episode, or a profile whose count is outside its declared range. 3: the run
finished but scored nothing (every episode skipped). Each of 2 and 3 is a run
that would otherwise have printed a clean result while testing nothing.

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
at 0.5 m/s), with wind and gusts rejected; a GUIDED setpoint keeps the
pessimistic open-loop disturbance of `stress`.

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
harness.

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
import hashlib
import json
import math
import random
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path, PurePath
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import kpi as K                                     # noqa: E402
from guardrail.models import (Action4D, CorridorSwap, DynamicNFZ,  # noqa: E402
                              State, TimeWindowSwitch)
from guardrail.scenario_spec import (GRANT_EVENT_TYPES, Defaults,  # noqa: E402
                                     Library, ScenarioSpec, expand,
                                     load_profile)
from guardrail.shield import Shield, recurrence_active             # noqa: E402

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

def _pilot(mission: _Mission, kind: str, st: State) -> Action4D:
    """What the operator ASKS for, which may be entirely illegal.

    A pilot that avoids violations on its own would leave the Shield with
    nothing to do and every scenario would pass for the wrong reason. It flies
    on the TRUE position: degraded perception is the Shield's problem here, not
    the pilot's, so a breach under GPS noise is the Shield's to own.
    """
    if kind == "constant":
        return Action4D(**(mission.action or {}))
    if kind == "stub_vla":
        return mission.stub.act(st)
    if kind == "goto":
        gx, gy, gup = mission.target
        dx, dy = gx - st.x, gy - st.y
        d = math.hypot(dx, dy)
        vx, vy = ((0.0, 0.0) if d < 1e-6 else
                  (mission.speed * dx / d, mission.speed * dy / d))
        if d < GOAL_TOL_M:
            vx = vy = 0.0
        vz = 0.0
        if gup is not None:
            vz = max(-2.0, min(2.0, float(gup) - st.up))
        return Action4D(vx=vx, vy=vy, vz_up=vz)
    raise ValueError(f"unknown pilot type {kind!r}")


def _autopilot(mode: str | None, st: State, home: tuple[float, float],
               dt: float) -> Action4D:
    """The velocity ArduCopter's own mode flies this tick (see AP_* above).

    LOITER holds position; RTL climbs to AP_RTL_ALT_M if below it, then flies
    straight home at AP_RTL_SPEED_MPS (ArduPilot's RTL knows nothing of the
    Shield's zones - whether the way home crosses one is measured, not
    assumed); LAND descends at AP_LAND_MPS. Any other mode (a fault latched
    before a mode was named) is treated as LOITER, ArduPilot's own fallback
    for a lost GUIDED stream."""
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
    anchors: dict = field(default_factory=dict)   # zone id -> event time it was based at
    refused: list = field(default_factory=list)   # (type, reason, expected)


def _zone(shield: Shield, zid: str):
    for c in shield.policy.constraints:
        if c.id == zid:
            if not isinstance(c, DynamicNFZ):
                raise ValueError(f"{zid!r} is a {c.type}; only a dynamic_nfz moves, "
                                 f"rotates or scales mid-flight")
            return c
    raise ValueError(f"no zone {zid!r} to edit (spawn one first)")


def _ring_now(zone: DynamicNFZ, based_at: float, t: float) -> list[tuple[float, float]]:
    """Where a zone is at time t: its declared motion since the event it was
    last based at (the Shield anchors each edit at the event time)."""
    return zone.ring_at(max(0.0, t - based_at)) if zone.motion is not None else \
        [(v.x, v.y) for v in zone.vertices]


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
            es.anchors[zone.id] = t
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
        based = es.anchors.get(zid, t)
        motion = zone.motion.model_dump() if zone.motion is not None else \
            {"vx_mps": 0.0, "vy_mps": 0.0, "yaw_rate_dps": 0.0}
        if ev.type == "translate_polygon_fence" and "mps" in p:
            ux, uy = _direction(p, heading)
            motion.update(vx_mps=float(p["mps"]) * ux, vy_mps=float(p["mps"]) * uy)
            ring = _ring_now(zone, based, t)
            out = run_all(lambda s: s.move_nfz(zid, ring, t, motion=dict(motion)))
            out["motion"] = motion
        elif ev.type == "rotate_polygon_fence" and "deg_per_s" in p:
            motion["yaw_rate_dps"] = float(p["deg_per_s"])
            ring = _ring_now(zone, based, t)
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
            es.anchors[zid] = t
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


def run_episode(sc, defaults=None, seed: int = 0, arm: dict | None = None) -> EpisodeRun:
    """One headless episode. `arm` (paraphrase / prefix record) is RECORDED in
    the episode_start event and changes nothing else: no headless pilot reads
    text."""
    spec = _as_spec(sc)
    dfl = _as_defaults(defaults)
    dt = float(spec.dt or dfl.dt)
    ticks = int(spec.ticks or dfl.ticks)
    look = float(spec.lookahead_s or dfl.lookahead_s)

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
    shield = Shield(policy, lookahead_s=look, dt=0.5, now=clock, escalation=esc)

    stress = spec.stress
    # Ground truth under degraded perception comes from a SECOND Shield kept in
    # step with the first (same rules, same events, same clock, same subject).
    # It is asked about the TRUE state. Re-using the flying Shield would put
    # the oracle's queries into its sliding window (`history`), which the
    # Shield keeps for its own decisions.
    oracle = (Shield(spec.policy.load(ROOT), lookahead_s=look, dt=0.5, now=clock,
                     escalation=False)
              if stress.degrades_perception else None)
    # When the autopilot flies (LOITER / RTL / LAND), what it flew is checked
    # by a third Shield, never by the flying one (whose FSM it would feed).
    checker = (Shield(spec.policy.load(ROOT), lookahead_s=look, dt=0.5, now=clock,
                      escalation=False) if esc else None)
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
    shields = [s for s in (shield, oracle, checker) if s is not None]
    truth = oracle or shield                     # the Shield asked about the TRUE state
    if subj:
        for s in shields:
            s.set_subject(m.subject[0], m.subject[1], subj_class)

    priorities = K.rule_priorities(policy)

    def p0(vios) -> bool:
        return any(priorities.get(v.rule_id, "P0") == "P0" for v in vios)

    pending = sorted(spec.events, key=lambda e: e.at_sim_t)
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
        "wind_mps": [round(wind[0], 3), round(wind[1], 3)]}]
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
    owner_ticks = {"shield": 0, "autopilot": 0, "pilot": 0}
    ap_mode_ticks: dict[str, int] = {}
    terminated_s = None
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

        raw = _pilot(m, kind, st)
        flags = {}
        if esc:
            # What the autopilot reports this tick, for the FSM's terminal
            # edges (G9 home reached, G10 landed). Home is reached at the end
            # of RTL's return leg: over the launch point, after the climb.
            flags = {"home_reached": ap_mode == "RTL"
                     and st.up >= AP_RTL_ALT_M - 0.1
                     and math.hypot(st.x - m.home[0], st.y - m.home[1]) <= AP_HOME_TOL_M,
                     "landed": ap_mode == "LAND" and st.up <= AP_LANDED_M}
        d = shield.filter(seen, raw, t=t_now, **flags)
        if esc and d.set_mode:
            ap_mode = d.set_mode
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
            # LOITER / RTL / LAND own the aircraft (setpoint "none").
            owner = "autopilot"
            flown = _autopilot(ap_mode, st, m.home, dt)
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
            for v in d.violations:
                if v.rule_id in timed_rules and v.rule_id not in active:
                    # Enforced before it is in force: the forecast saw the
                    # window open inside the lookahead (the t+ side).
                    anticipated[v.rule_id] = anticipated.get(v.rule_id, 0) + 1
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
            if owner == "autopilot":
                # guardrail.kpi leaves these ticks out of the Shield's own
                # counts: the Shield did not decide what flew.
                row["autopilot"] = ap_mode or "LOITER"
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
                # "AUTOPILOT TICKS"), so it is counted here instead of nowhere.
                autopilot_p0_ticks += 1
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
        if sp > 0.1:
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

        gx = rng_wind.gauss(0, stress.gust_mps) if stress.gust_mps else 0.0
        gy = rng_wind.gauss(0, stress.gust_mps) if stress.gust_mps else 0.0
        if owner == "autopilot":
            gx = gy = 0.0                    # position control rejects them
            wx = wy = 0.0
        else:
            wx, wy = wind
        prev_st = st
        st = State(x=st.x + (flown.vx + wx + gx) * dt,
                   y=st.y + (flown.vy + wy + gy) * dt,
                   up=st.up + flown.vz_up * dt, yaw_deg=st.yaw_deg)
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
        "terminated_s": terminated_s,
        # Events the scenario expected the Shield to refuse: None when there
        # were none, else whether every one was refused (and nothing else).
        "event_refusals_as_expected": (None if not es.refused
                                       else all(exp for _, _, exp in es.refused)),
        **boundary,
        **phases,
    }
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
    if s.degrades_perception or s.wind_speed_mps or s.gust_mps or s.start_jitter_m:
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
    """
    cmd = ["wsl", "bash", SITL_SCRIPT, "on" if spec.shield else "off", "",
           "--tag", tag, "--seed", str(int(seed))]
    if spec.policy.bundle_path is not None:
        cmd += ["--bundle", spec.policy.bundle_path]
    else:
        cmd += ["--policy", spec.policy.path]
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
    return rows, {"seed_applied": True}


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
    bad = []
    for fld, rule in (gates or {}).items():
        got = result.get(fld)
        if got is None:
            if (fld in _VACUOUS and set(rule) == {"max"}
                    and result.get("time_to_safe_not_measurable") is False
                    and result.get("time_to_safe_episodes") == 0
                    and result.get("time_to_safe_censored") == 0):
                continue
            if fld in _VACUOUS and (result.get("time_to_safe_censored") or 0) > 0:
                bad.append(f"{fld}: never recovered (censored episode)")
                continue
            bad.append(f"{fld}: not measured")
            continue
        if "max" in rule and got > rule["max"]:
            bad.append(f"{fld}={got} > max {rule['max']}")
        if "min" in rule and got < rule["min"]:
            bad.append(f"{fld}={got} < min {rule['min']}")
        if "equals" in rule and got != rule["equals"]:
            bad.append(f"{fld}={got} != {rule['equals']}")
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
        fail = ((res.get("p0_escapes") or 0) > 0 or extra.get("breaches", 0) > 0
                or extra.get("reached_goal") is False)
        res["mission_outcome"] = "fail" if fail else "success"
    res["outcome_matches_label"] = (None if spec.expected_outcome is None
                                    else res["mission_outcome"] == spec.expected_outcome)
    return res


def label_failures(spec: ScenarioSpec, res: dict) -> list[str]:
    bad = []
    if spec.expected_outcome is not None and not res.get("outcome_matches_label"):
        bad.append(f"outcome: labelled {spec.expected_outcome}, flew "
                   f"{res.get('mission_outcome')}")
    if spec.expected_failsafe is not None:
        if res.get("failsafe_triggered") is None:
            bad.append("fail-safe: labelled, but this episode ran no escalation FSM "
                       "(not measured)")
        elif res.get("failsafe_matches_label") != 1.0:
            bad.append(f"fail-safe: labelled "
                       f"{'expected' if spec.expected_failsafe else 'not expected'}, "
                       f"{'triggered' if res.get('failsafe_triggered') else 'not triggered'}")
    return bad


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
    dmins = [r["kpi"]["min_true_dist_to_fence_m"] for r in entered
             if r["kpi"].get("min_true_dist_to_fence_m") is not None]
    by_status: dict[str, int] = {}
    for r in entered:
        by_status[r["status"]] = by_status.get(r["status"], 0) + 1
    return {
        "shield_on_episodes": len(on),
        # Flew itself across a polygon edge: the incursions.
        "episodes_entering_polygon": len(entered),
        "episodes_entering_polygon_by_status": by_status,
        "min_true_dist_to_fence_m_of_entries": min(dmins) if dmins else None,
        # Inside at some tick for any reason, including a zone spawned on it or
        # a window that opened on it (imposed; time_to_safe measures those).
        "episodes_inside_polygon": len(inside),
        "episodes_inside_polygon_imposed_only": len(inside) - len(entered),
        "true_p0_flown_ticks": sum(r["kpi"].get("true_p0_flown_ticks") or 0 for r in on),
        "episodes_with_breach": sum(1 for r in on if (r["kpi"].get("breaches") or 0) > 0),
        # ---- what the AUTOPILOT flew (never merged into the counts above) ----
        "autopilot_ticks": sum(r["kpi"].get("autopilot_ticks") or 0 for r in on),
        # Of true_p0_flown_ticks, the ones the modelled LOITER / RTL / LAND flew.
        "autopilot_p0_flown_ticks": sum(r["kpi"].get("autopilot_p0_flown_ticks") or 0
                                        for r in on),
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
    return out


def results_path(out: str | None, publish: bool, tag: str | None) -> Path:
    """Where the results JSON goes. The published artefact only on --publish:
    a routine re-run used to replace docs/data/scenario_sweep.json, which the
    evaluation data, the deck and a pinned test read."""
    if publish:
        return PUBLISHED_SWEEP
    if out:
        return Path(out)
    return ROOT / "demo" / "out" / (f"stress_{tag}" if tag else "sweep") / "sweep.json"


def _rollup(eps: list[dict]) -> dict | None:
    """guardrail.kpi.rollup when this checkout has it (it landed on 2026-10-06);
    None otherwise, and the caller records that it is missing."""
    fn = getattr(K, "rollup", None)
    return fn(eps) if (fn and eps) else None


def cell_key(table: dict) -> str:
    """The grant's KPI-report row, "(scenario template, parameter cell)", plus
    the Shield arm and the prefix arm, which are different experiments.
    Seeds and paraphrase arms of one cell pool in its row."""
    key = f"{table['family']}/{table['cell']}/shield-{table['arm']}"
    if table.get("prefix") == "off":
        key += "/prefix-off"
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
                         "demo/out/stress_<profile>/sweep.json)")
    ap.add_argument("--publish", action="store_true",
                    help="write the library run to docs/data/scenario_sweep.json, "
                         "the PUBLISHED artefact (needs a CHANGELOG entry)")
    ap.add_argument("--runs", default=None,
                    help="where episode bundles and kpi_report.md go")
    ap.add_argument("--no-bundles", action="store_true",
                    help="skip the per-episode bundle folders")
    ap.add_argument("--keep-logs", action="store_true",
                    help="also write each episode's flight_log.jsonl, so "
                         "tools/kpi_report.py can re-score the bundles")
    args = ap.parse_args(argv)

    if args.publish and (args.out or args.profile or args.only or args.seed is not None):
        # The published file is the WHOLE library on its declared seeds; a
        # partial or profiled run written there would replace it with less.
        print("REFUSED: --publish writes the full library run; it takes no "
              "--out, --profile, --only or --seed")
        return 2

    t_run = time.perf_counter()
    profile = _resolve_profile(args.profile)
    lib_path = Path(args.library or (profile.library if profile else LIB))
    if not lib_path.is_absolute():
        lib_path = ROOT / lib_path
    lib = Library(lib_path)
    episodes, info = expand(lib, profile, args.seed, args.only)
    tag = profile.profile if profile else None
    out = results_path(args.out, args.publish, tag)
    runs = Path(args.runs) if args.runs else (
        ROOT / "demo" / "out" / (f"stress_{tag}" if tag else "sweep"))
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    verbose = len(episodes) <= 80

    arms = info.get("arms") or {}
    print(f"{info['cells']} scenarios, {len(episodes)} episodes"
          + (f" ({arms.get('extra_arm_episodes')} of them extra arms)"
             if arms.get("extra_arm_episodes") else "")
          + f", backend={args.backend}" + (f", profile={tag}" if tag else "") + "\n")
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
    one_seed = len(episodes) == len({(e.run_id) for e in episodes})

    for ep in episodes:
        spec = ep.cell.spec
        eid = ep.run_id
        label = eid if one_seed else f"{eid} seed={ep.seed}"
        common = {"id": eid, "id_cell": ep.episode_id, "scenario_id": ep.cell.scenario_id,
                  "template": spec.template, "params": ep.cell.params,
                  "seed": ep.seed, "arm": "on" if spec.shield else "off",
                  "prefix": ep.prefix, "expect": spec.expect,
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
                run = run_episode(spec, lib.defaults, ep.seed, arm=arm_record)
                rows, extra, policy = run.rows, run.extra, run.policy
                events, hash0 = run.events, run.policy_hash_start
                sim_s, wall_s, consumed = run.sim_s, run.wall_s, run.seed_consumed
                prios = run.priorities
        except UnsupportedScenario as e:
            n_skip += 1
            print(f"  SKIP  {label:<34} {e}")
            rows_out.append({**common, "status": "skipped", "reason": str(e)})
            continue
        except Exception as e:                                   # noqa: BLE001
            n_fail += 1
            print(f"  ERROR {label:<34} {type(e).__name__}: {e}")
            rows_out.append({**common, "status": "error", "error": str(e)})
            continue

        res = score(spec, rows, extra, policy, prios)
        labels_bad = label_failures(spec, res)
        bad = check_gates(spec.gates, res)
        if spec.expect == "tracked":
            # A tracked family measures how far past its design point the
            # Shield degrades; a fail-safe its degraded perception provoked is
            # part of that measurement. Its label mismatches are reported
            # (`labels_failed`, and in the fail-safe KPI, which counts every
            # labelled episode) but do not fail it - its gates still do.
            common["labels_failed"] = labels_bad
        else:
            bad = bad + labels_bad
        known_why = spec.known_failure_for(ep.cell.params, ep.seed)
        known = known_why is not None
        if known:
            common["known_failure_reason"] = known_why
        if bad and known:
            n_known += 1
            status, mark = "known_failure", "KNOWN"
        elif bad:
            n_fail += 1
            status, mark = "fail", "FAIL "
        elif known:
            # A known failure that started passing is news, not a quiet pass.
            n_fail += 1
            status, mark = "unexpected_pass", "FIXED"
            bad = ["marked known_failure but every gate passed - "
                   "the defect is fixed; remove the marker"]
        elif spec.expect == "tracked":
            # Its gates hold only what must hold whatever the sensor does; the
            # aircraft may still have entered a polygon. Never a "pass".
            n_tracked += 1
            status, mark = "tracked", "track"
        else:
            n_pass += 1
            status, mark = "pass", "pass "

        if verbose or status not in ("pass", "tracked"):
            print(f"  {mark} {label:<34} escape={res['p0_violation_escape_rate']:<9} "
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
            write_bundle(ep_dir, manifest, _json_safe({
                **common, "status": status, "gates_failed": bad,
                "kpi_grade": grade, "kpi_grade_reasons": why_not,
                "arm_record": arm_record, "rule_priorities": prios,
                "metrics": {"reached_goal": extra.get("reached_goal"),
                            "breaches": extra.get("breaches"),
                            "shield": "on" if spec.shield else "off",
                            "fsm": extra.get("fsm")},
                "kpi": res}), events, rows if args.keep_logs else None, extras)
            bundle = _show(ep_dir)

        item = {**common, "status": status, "gates_failed": bad, "kpi": res,
                "seed_consumed": consumed,
                # Headless: the seed went into run_episode. SITL: the node
                # records --seed in its manifest (StubVLA draws nothing).
                "seed_applied": True,
                "manifest": manifest, "manifest_extras": extras,
                "arm_record": arm_record, "bundle": bundle}
        rows_out.append(item)
        table = {**res, "id": eid, "family": spec.template, "cell": ep.episode_id,
                 "arm": common["arm"], "prefix": ep.prefix, "params": ep.cell.params,
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
        (tables_on if spec.shield else tables_off).append(table)

    # ---- summaries ---------------------------------------------------------
    scored = [r for r in rows_out if "kpi" in r]
    oc = [r for r in scored if r["kpi"].get("outcome_matches_label") is not None]
    families: dict[str, dict] = {}
    sigs_by_family: dict[str, set] = {}
    for r in scored:
        f = families.setdefault(r["template"], {"episodes": 0, "pass": 0, "tracked": 0,
                                                "fail": 0, "known_failure": 0,
                                                "episodes_with_breach": 0,
                                                "episodes_entering_polygon": 0,
                                                "episodes_inside_polygon": 0,
                                                "true_p0_flown_ticks": 0,
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
        mo = k.get("mission_outcome")
        f["fsm_outcomes"][mo] = f["fsm_outcomes"].get(mo, 0) + 1
        d = k.get("min_true_dist_to_fence_m")
        if d is not None:
            cur = f["min_true_dist_to_fence_m"]
            f["min_true_dist_to_fence_m"] = d if cur is None else min(cur, d)
        sigs_by_family.setdefault(r["template"], set()).add(kpi_signature(r))
    for name, f in families.items():
        f["distinct_kpi_signatures"] = len(sigs_by_family[name])
    all_sigs = {kpi_signature(r) for r in scored}
    true_state = true_state_summary(scored)
    rollups, rollup_note = {}, None
    cells: dict[str, dict] = {}
    try:
        for fam in sorted({t["family"] for t in tables_on}):
            rollups[fam] = _rollup([t for t in tables_on if t["family"] == fam])
        rollups["_all_shield_on"] = _rollup(tables_on)
        rollups["_controls_shield_off"] = _rollup(tables_off)
        if rollups["_all_shield_on"] is not None:
            # Beside the grant's p0_escape_rate (the Shield's own count), where
            # the aircraft really was: the two answer different questions.
            rollups["_all_shield_on"]["true_state"] = true_state
        groups: dict[str, list] = {}
        for t in tables_on + tables_off:
            groups.setdefault(cell_key(t), []).append(t)
        for key in sorted(groups):
            cells[key] = dict(_rollup(groups[key]), members=[t["id"] for t in groups[key]])
    except Exception as e:                                       # noqa: BLE001
        rollup_note = f"guardrail.kpi.rollup failed: {type(e).__name__}: {e}"
    top = None
    try:
        top = K.top_failures(tables_on + tables_off, k=10)
    except Exception as e:                                       # noqa: BLE001
        rollup_note = (rollup_note or "") + f"; top_failures failed: {e}"

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
              + f"; {ts['episodes_inside_polygon_imposed_only']} more were inside "
              f"only because a zone appeared under them; "
              f"{ts['true_p0_flown_ticks']} P0 ticks flown at the true state")
        print(f"  autopilot (modelled LOITER / RTL / LAND, not the Shield): "
              f"{ts['autopilot_p0_flown_ticks']} of {ts['autopilot_ticks']} ticks "
              f"flew a P0-violating action in {ts['episodes_autopilot_p0_flown']} "
              f"episode(s) {ts['episodes_autopilot_p0_flown_by_status']}; "
              f"{ts['autopilot_polygon_entries']} polygon entr(y/ies) in "
              f"{ts['episodes_autopilot_entering_polygon']} episode(s); "
              f"{ts['autopilot_breaches']} breach(es) in "
              f"{ts['episodes_autopilot_breach']} episode(s)")
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
              f"{fsl['unlabelled']} unlabelled"
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
    if args.publish:
        print(f"\nPUBLISHED: {_show(out)} replaced. It is read by "
              "tools/build_eval_data.py, build_deck_data.py, "
              "build_architecture_svg.py and tests/test_kpi_magnitudes.py; add a "
              "CHANGELOG Changed/Retracted entry for what moved. The full run "
              f"file (with {', '.join(PUBLISHED_DROPS)}): "
              f"{_show(results_path(None, False, tag))}")
    if n_fail:
        return 1
    if not scored:
        print("\nNothing was scored: every episode was skipped (exit 3).")
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
