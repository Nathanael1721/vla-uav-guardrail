"""Run the scenario library, or a stress profile, and score every episode with
the contractual KPIs.

    python experiments/sweep_scenarios.py                    # the library, one pass
    python experiments/sweep_scenarios.py --profile smoke    # ~50 scenarios x 1 seed
    python experiments/sweep_scenarios.py --profile nightly  # ~200 scenarios x 3 seeds
    python experiments/sweep_scenarios.py --seed 1002        # every episode on one seed
    python experiments/sweep_scenarios.py --only nfz-head-on corridor-along
    python experiments/sweep_scenarios.py --backend sitl     # ArduPilot + MAVROS 2 rail
    python experiments/sweep_scenarios.py --publish          # REPLACE docs/data/scenario_sweep.json

Results go to demo/out/sweep/ (or demo/out/stress_<profile>/). Until
2026-10-06 a plain run overwrote docs/data/scenario_sweep.json, a published
artefact that the evaluation data, the deck and tests/test_kpi_magnitudes.py
read; replacing it now takes `--publish`, and a CHANGELOG entry.

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

  * scenarios "validated through Pydantic at load time": the library is now
    loaded through `guardrail/scenario_spec.py` (ScenarioSpec / ScenarioEvent,
    copied from the grant field for field). A misspelt key stops the load;
  * seeds and profiles: `--seed`, and `--profile smoke|nightly` reading
    `experiments/profiles/*.yaml` - the grant's scheduling table, "~50
    scenarios x 1 seed" per build and "~200 scenarios x 3 seeds" nightly. The
    counts come from parameter sweeps and seeded geometry draws. The loader
    refuses a swept parameter that nothing flown reads, and this script
    refuses, before running, a run in which two cells fly the same episode -
    so the counts are not padded. How many cells actually BEHAVED differently
    is reported beside the count (distinct KPI signatures);
  * the Safety Shield page's smoke matrix (PDF p6): high-speed near-edge,
    sudden NFZ (hot-applied mid-flight through `Shield.hot_apply`), time-window
    switch instant (an advancing scenario clock crossing a `valid_time` edge),
    three simultaneous violations, and GPS noise / latency on the state the
    Shield sees. Plus wind as a velocity disturbance, the grant example's
    `wind_speed_mps: [0, 2, 5]`, and GPS denial as a frozen pose (audit card
    X-14);
  * "every episode emits a six-field manifest": each episode writes an
    episode bundle - `manifest.json` (the six fields), `kpi.json`,
    `harness_events.jsonl` - under the grant's directory name.

THE HEADLESS BACKEND, AND WHAT IT IS NOT

Shield plus a plain kinematic integrator: no simulator, no GPU, no autopilot, no
WSL. It integrates exactly the 4-D action the contract defines, which is the only
thing the Shield is responsible for. What it therefore CANNOT test: the
autopilot's tracking, timing under load, or perception.

It is not KPI-grade and is built so it cannot be mistaken for it. Its manifest
names the topology `headless-kinematic`, and `sim_speedup` is MEASURED (simulated
seconds over wall seconds, typically several hundred) rather than declared, so
`guardrail.manifest.is_kpi_grade` refuses every episode on two counts. The grant's
smoke and nightly rows ask for sim_speedup 1.0 in the HIL topology; these
profiles reproduce the grant's SCOPE, not its rail, and each profile file says so.

`--backend sitl` drives `sitl/run_ros2_demo.sh` -> `sitl/ros2_shield_node.py`
(ArduPilot SITL + MAVROS 2) with the scenario's policy (or bundle), subject and
Shield arm. That rail flies its OWN fixed mission - StubVLA from the pad to the
"northeast pad" (30, 30) at 6 m/s - and takes no start, target, pilot, clock,
stressor, event or seed. Flying a scenario there and scoring it against that
scenario's gates under that scenario's id would describe a different flight,
so every scenario whose mission is not the rail's is reported SKIPPED with
the reason - today that is every scenario in the library, because no headless
pilot is the rail's StubVLA. The command line, the stale-log guard and the
bundle mapping are built and tested for the day the node takes a mission;
until then `--backend sitl` exits 3 (nothing scored), never 0. A harness that
quietly scores 11/11 while running 4 is worse than no harness.

TRACKED FAMILIES

A family marked `expect: tracked` (GPS noise beyond the margin, GPS denial,
drawn gusts) is gated only on what must hold whatever the sensor does. An
episode that passes those gates is reported "tracked", never "pass", and the
summary prints the true-state polygon entries beside the Shield's own P0
escape count - the grant's KPI ("counter from Shield repair log") is 0 in
those runs while the aircraft was, in some of them, metres inside the zone.

SCORING

Through `guardrail.kpi.compute`, the same function the delivered flights use.
Writing a second KPI implementation here would let the sweep and the artefacts
disagree about what a P0 escape is, and the sweep would be the one nobody
checked. The harness adds only what a kinematic run can see directly and the
rails cannot: ground-truth safety under degraded perception, self-inflicted
breaches, and the fail-safe and outcome LABELS each scenario declares. The
label-based fail-safe correctness is printed with its two nulls (never
trigger, always trigger) and the grant's ">= 99 %": a Shield that never brakes
scores the never-trigger null, so a correctness below it is worse than doing
nothing, and is said so.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import kpi as K                                     # noqa: E402
from guardrail.geometry import fence_polygon, nearest_on_polyline  # noqa: E402
from guardrail.models import (Action4D, Corridor, PolygonFence,    # noqa: E402
                              State)
from guardrail.scenario_spec import (Defaults, Library,            # noqa: E402
                                     ScenarioSpec, expand, load_profile)
from guardrail.shield import Shield                                # noqa: E402

LIB = ROOT / "experiments" / "scenarios.yaml"
PROFILES = ROOT / "experiments" / "profiles"
GOAL_TOL_M = 3.0

# The manifest's topology for this backend. Not one of guardrail.manifest's
# names on purpose: those describe flight rails, and this is not one.
TOPOLOGY_HEADLESS = "headless-kinematic"
# The "VLA" of a headless episode is the dumb pilot below. model_hash() hashes
# this module's source for a code-only pilot, which pins it exactly.
PILOT_ID = "experiments.sweep_scenarios._pilot"
# The only event the Shield has an API for. The rest of the grant's vocabulary
# (translate / rotate / activate / deactivate / swap / scale) is refused.
SUPPORTED_EVENTS = ("spawn_polygon_fence",)

SITL_SCRIPT = "sitl/run_ros2_demo.sh"
SITL_NODE = "sitl/ros2_shield_node.py"
# The rail's own mission, as sitl/ros2_shield_node.py and ros2_vla_stub_node.py
# hard-code it: `parse_command("fly to the northeast pad at 6 m/s")`, flown by
# guardrail.vla_stub.StubVLA from wherever take-off left the vehicle (the pad,
# local (0, 0)). tests/test_sweep.py re-reads the command from the node's
# source and the pad from guardrail.compiler.PLACES, so these cannot drift.
SITL_RAIL_COMMAND = "fly to the northeast pad at 6 m/s"
SITL_RAIL_START = (0.0, 0.0)
SITL_RAIL_TARGET = (30.0, 30.0)
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
    return _Mission(start=start, target=target, subject=subject, action=action,
                    speed=float(m.pilot.speed or 4.0))


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


# --------------------------------------------------------------------------- #
# events
# --------------------------------------------------------------------------- #

def _spawn_fence(payload: dict, k: int, st: State, heading: tuple[float, float]
                 ) -> PolygonFence:
    """spawn_polygon_fence, with the grant example's payload.

    `width_m` x `height_m` (across x along track), its near edge
    `ahead_of_vehicle_m` ahead of the vehicle on its current heading - the
    worked example's "Polygon NFZ spawns 60 m ahead". `anchor: center` puts the
    CENTRE that far ahead instead, so `ahead_of_vehicle_m: 0, anchor: center`
    drops the zone on top of the aircraft. `vertices` gives it absolutely.
    """
    known = {"id", "width_m", "height_m", "ahead_of_vehicle_m", "anchor",
             "vertices", "altitude_floor_m", "altitude_ceiling_m", "margin_m",
             "priority"}
    bad = set(payload) - known
    if bad:
        raise ValueError(f"spawn_polygon_fence: unknown payload key(s) {sorted(bad)}")
    if "vertices" in payload:
        verts = [{"x": float(v["x"]), "y": float(v["y"])} for v in payload["vertices"]]
    else:
        w, h = float(payload["width_m"]), float(payload["height_m"])
        a = float(payload.get("ahead_of_vehicle_m", 0.0))
        fx, fy = heading
        lx, ly = -fy, fx
        along = a + h / 2 if payload.get("anchor", "near_edge") == "near_edge" else a
        if payload.get("anchor", "near_edge") not in ("near_edge", "center"):
            raise ValueError("spawn_polygon_fence: anchor is near_edge or center")
        cx, cy = st.x + fx * along, st.y + fy * along
        verts = [{"x": cx + fx * sa * h / 2 + lx * sb * w / 2,
                  "y": cy + fy * sa * h / 2 + ly * sb * w / 2}
                 for sa, sb in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    return PolygonFence.model_validate({
        "id": payload.get("id", f"dyn-nfz-{k}"), "type": "polygon_fence",
        "priority": payload.get("priority", "P0"), "vertices": verts,
        "altitude_floor_m": float(payload.get("altitude_floor_m", 0.0)),
        "altitude_ceiling_m": float(payload.get("altitude_ceiling_m", 1000.0)),
        "margin_m": float(payload.get("margin_m", 1.0))})


def _signed_dist(poly, x: float, y: float) -> float:
    """Metres from (x, y) to the polygon itself (NOT its margin ring); negative
    inside. The margin is the buffer that disturbance and sensor error are
    allowed to eat; the polygon is the line that must hold."""
    from shapely.geometry import Point
    p = Point(x, y)
    d = poly.exterior.distance(p)
    return -d if poly.contains(p) else d


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


def _as_spec(sc) -> ScenarioSpec:
    return sc if isinstance(sc, ScenarioSpec) else ScenarioSpec.model_validate(sc)


def _as_defaults(d) -> Defaults:
    if isinstance(d, Defaults):
        return d
    return Defaults.model_validate(d or {})


def run_episode(sc, defaults=None, seed: int = 0) -> EpisodeRun:
    spec = _as_spec(sc)
    dfl = _as_defaults(defaults)
    dt = float(spec.dt or dfl.dt)
    ticks = int(spec.ticks or dfl.ticks)
    look = float(spec.lookahead_s or dfl.lookahead_s)

    unsupported = sorted({e.type for e in spec.events} - set(SUPPORTED_EVENTS))
    if unsupported:
        raise UnsupportedScenario(
            f"event type(s) {unsupported} have no Shield API to apply them "
            f"(only {list(SUPPORTED_EVENTS)} through Shield.hot_apply)")
    t_end = (ticks - 1) * dt
    late = [e for e in spec.events if e.at_sim_t > t_end + 1e-9]
    if late:
        raise ValueError(f"event at {late[0].at_sim_t} s never fires: the run "
                         f"ends at {t_end:.1f} s")
    drop = spec.stress.gps_dropout_s
    if drop is not None and drop[0] > t_end + 1e-9:
        # Same reasoning as a late event: a dropout that starts after the last
        # tick would leave a "GPS denial" scenario that never denied anything.
        raise ValueError(f"gps_dropout_s starts at {drop[0]} s and never "
                         f"happens: the run ends at {t_end:.1f} s")

    policy = spec.policy.load(ROOT)
    hash0 = policy.policy_hash
    sim_t = [0.0]
    clock = None
    if spec.mission.clock_start is not None:
        t0 = spec.mission.clock_start
        clock = lambda: t0 + timedelta(seconds=sim_t[0])          # noqa: E731
    shield = Shield(policy, lookahead_s=look, dt=0.5, now=clock)

    stress = spec.stress
    # Ground truth under degraded perception comes from a SECOND Shield kept in
    # step with the first (same rules, same hot-applies, same clock, same
    # subject). It is asked about the TRUE state. Re-using the flying Shield
    # would put the oracle's queries into its sliding window (`history`), which
    # the Shield keeps for its own decisions.
    oracle = (Shield(spec.policy.load(ROOT), lookahead_s=look, dt=0.5, now=clock)
              if stress.degrades_perception else None)
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

    on = spec.shield
    kind = spec.mission.pilot.type
    subj = spec.mission.subject
    relabels = sorted(subj.reclassify if subj else [], key=lambda r: r.at_s)
    subj_class = subj.class_ if subj else None
    shields = [s for s in (shield, oracle) if s is not None]
    if subj:
        for s in shields:
            s.set_subject(m.subject[0], m.subject[1], subj_class)

    priorities = K.rule_priorities(policy)

    def p0(vios) -> bool:
        return any(priorities.get(v.rule_id, "P0") == "P0" for v in vios)

    corridors = policy.by_type(Corridor)
    pending = sorted(spec.events, key=lambda e: e.at_sim_t)
    events: list[dict] = [{
        "t": 0.0, "type": "episode_start", "scenario_id": spec.scenario_id,
        "seed": seed, "policy_hash": hash0, "generation": policy.generation,
        "shield": "on" if on else "off",
        "task_prompt": spec.mission.task_prompt or None,
        "task_prompt_read_by": None if not spec.mission.task_prompt else
        "nobody: the headless pilot is not a language model",
        "wind_mps": [round(wind[0], 3), round(wind[1], 3)]}]
    fence_polys: dict[str, Any] = {}
    st = m.start
    hist: list[State] = []
    rows: list[dict] = []
    max_speed = 0.0
    min_range = math.inf
    max_offset = 0.0
    finite = True
    min_by_class: dict[str, float] = {}
    range_at_relabel = None
    prev_active = None
    prev_ids: set[str] = set()
    prev_unsafe = False
    breaches = 0
    true_p0_ticks = 0
    min_fence_d = math.inf
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
    n_spawn = 0
    frozen: State | None = None
    dropout_ticks = 0
    wall0 = time.perf_counter()

    for i in range(ticks):
        t_now = i * dt
        sim_t[0] = t_now
        imposed = i == 0
        # ---- scheduled harness events -----------------------------------
        while pending and pending[0].at_sim_t <= t_now + 1e-9:
            ev = pending.pop(0)
            fence = _spawn_fence(ev.payload, n_spawn, st, heading)
            n_spawn += 1
            if any(c.id == fence.id for c in shield.policy.constraints):
                # Same refusal as the hot-apply REST endpoint (guardrail/api.py).
                raise ValueError(f"spawn_polygon_fence: id {fence.id!r} already exists")
            for s in shields:
                s.hot_apply(fence)
            priorities[fence.id] = fence.priority
            events.append({"t": round(t_now, 4), "type": "hot_apply",
                           "event": ev.type, "at_sim_t": ev.at_sim_t,
                           "rule_id": fence.id,
                           "vertices": [[round(v.x, 3), round(v.y, 3)]
                                        for v in fence.vertices],
                           "generation": shield.policy.generation,
                           "policy_hash": shield.policy.policy_hash})
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
        ids_now = {c.id for c in shield.policy.constraints}
        if clock is not None:
            active = {c.id for c in shield.policy.constraints if c.active_at(clock())}
            if prev_active is not None:
                on_ = sorted((active - prev_active) & prev_ids)
                off_ = sorted((prev_active - active) & ids_now)
                if on_ or off_:
                    events.append({"t": round(t_now, 4), "type": "time_window_switch",
                                   "clock": clock().isoformat(),
                                   "activated": on_, "deactivated": off_})
                    imposed = True
            else:
                events[0]["active_rules"] = sorted(active)
                events[0]["clock"] = clock().isoformat()
            prev_active = active
        prev_ids = ids_now
        if imposed and i > 0:
            phase += 1
            phase_vios.append({})

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
        d = shield.filter(seen, raw)
        # Shield OFF flies the RAW action, so the violations found on `raw` are
        # the flown action's - the same rule the SITL rails follow, and what
        # makes the control arm able to score a genuine escape.
        flown = d.emitted if on else raw
        flown_vios = d.emitted_violations if on else d.violations
        if oracle is not None:
            true_vios = oracle.filter(st, flown).violations
            unsafe = bool(oracle.state_is_unsafe(st))
        else:
            true_vios = flown_vios
            unsafe = bool(shield.state_is_unsafe(st))

        row = {
            "t": round(t_now, 4), "tick": i,
            "x": st.x, "y": st.y, "up": st.up,
            "raw": raw.model_dump(), "emitted": flown.model_dump(),
            "violations": [v.model_dump() for v in d.violations],
            # The Shield's OWN re-check of what it flew, against the state it
            # SAW - the grant's "counter from Shield repair log". Under clean
            # perception it is also the truth; under noise, `true_violations`
            # is, and the two are reported apart.
            "emitted_violations": [v.model_dump() for v in flown_vios],
            "repairs": [r.model_dump() for r in d.repairs],
            "braked": d.braked,
            # Position-based, and always the TRUE position: time to safe asks
            # where the aircraft actually was.
            "unsafe": unsafe,
        }
        if oracle is not None:
            row["perceived"] = {"x": seen.x, "y": seen.y, "up": seen.up}
            row["true_violations"] = [v.model_dump() for v in true_vios]
        rows.append(row)

        if p0(true_vios):
            true_p0_ticks += 1
        if unsafe and not prev_unsafe and not imposed:
            breaches += 1
        prev_unsafe = unsafe
        for v in d.violations:
            phase_vios[phase][v.rule_id] = phase_vios[phase].get(v.rule_id, 0) + 1

        inside_now = False
        for c in shield.policy.constraints:
            if not isinstance(c, PolygonFence):
                continue
            if clock is not None and not c.active_at(clock()):
                continue
            if not (c.altitude_floor_m <= st.up <= c.altitude_ceiling_m):
                continue
            poly = fence_polys.get(c.id)
            if poly is None:
                poly = fence_polys[c.id] = fence_polygon(c)
            sd = _signed_dist(poly, st.x, st.y)
            min_fence_d = min(min_fence_d, sd)
            if sd < 0:
                inside_now = True
        if inside_now:
            inside_poly_ticks += 1
            if not prev_inside and not imposed:
                # The aircraft moved ITSELF across a polygon edge. Being inside
                # because a zone spawned on it or a window opened on it is
                # imposed, and is what time_to_safe measures instead.
                poly_entries += 1
        prev_inside = inside_now

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
        for c in corridors:
            off, _, _ = nearest_on_polyline(st.x, st.y, c.points())
            max_offset = max(max_offset, off)

        gx = rng_wind.gauss(0, stress.gust_mps) if stress.gust_mps else 0.0
        gy = rng_wind.gauss(0, stress.gust_mps) if stress.gust_mps else 0.0
        st = State(x=st.x + (flown.vx + wind[0] + gx) * dt,
                   y=st.y + (flown.vy + wind[1] + gy) * dt,
                   up=st.up + flown.vz_up * dt, yaw_deg=st.yaw_deg)

    wall_s = time.perf_counter() - wall0
    reached = None
    if kind == "goto":
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
        "max_offset_from_corridor": (round(max_offset, 3) if corridors else None),
        "reached_goal": reached,
        "final": {"x": round(st.x, 2), "y": round(st.y, 2), "up": round(st.up, 2)},
        # ---- stress-harness measurements -------------------------------------
        # Times the aircraft moved ITSELF from a legal position into an illegal
        # one. Transitions on the tick a rule changed under it (a spawn, a
        # window opening, a relabel) or at t = 0 are imposed by the scenario
        # and not counted.
        "breaches": breaches,
        # Ticks on which the flown action violated a P0 rule at the TRUE state,
        # whether or not the Shield saw it. Equal to the Shield's own count
        # under clean perception; under noise it is the honest one.
        "true_p0_flown_ticks": true_p0_ticks,
        "min_true_dist_to_fence_m": (None if min_fence_d is math.inf
                                     else round(min_fence_d, 3)),
        "ticks_inside_fence_polygon": (None if min_fence_d is math.inf
                                       else inside_poly_ticks),
        # Times the aircraft flew itself INTO a polygon (not counting t = 0 or
        # a tick on which a rule appeared under it). The number a reader means
        # by "did it enter the no-fly zone".
        "polygon_entries": (None if min_fence_d is math.inf else poly_entries),
        "harness_events": len(events) - 1,
        "policy_generation_final": shield.policy.generation,
        # None when the scenario declares no dropout; otherwise the ticks the
        # Shield was really handed a frozen pose. A declared dropout that
        # froze nothing reads 0, never None, so it cannot pass for "not run".
        "gps_dropout_ticks": None if drop is None else dropout_ticks,
        **phases,
    }
    events.append({"t": round((ticks - 1) * dt, 4), "type": "episode_end",
                   "policy_hash": shield.policy.policy_hash,
                   "generation": shield.policy.generation,
                   "final": extra["final"]})
    return EpisodeRun(rows=rows, extra=extra, events=events, policy=shield.policy,
                      policy_hash_start=hash0, sim_s=ticks * dt, wall_s=wall_s,
                      seed_consumed=stress.consumes_seed,
                      shield_window=shield.history)


def run_headless(sc, defaults=None, seed: int = 0) -> tuple[list[dict], dict]:
    """(rows, extra) for one episode - the shape the sweep has always returned."""
    run = run_episode(sc, defaults, seed)
    return run.rows, run.extra


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
    """How this scenario's mission differs from the one the rail flies."""
    m = spec.mission
    out = []
    if m.start_pose.x is None or math.hypot(m.start_pose.x - SITL_RAIL_START[0],
                                            m.start_pose.y - SITL_RAIL_START[1]) > 1.0:
        out.append(f"start {_xy(m.start_pose)} (the rail starts at the pad, "
                   f"{SITL_RAIL_START})")
    if (m.target is None or m.target.x is None
            or math.hypot(m.target.x - SITL_RAIL_TARGET[0],
                          m.target.y - SITL_RAIL_TARGET[1]) > 0.5):
        out.append(f"target {_xy(m.target) if m.target else 'none'} (the rail "
                   f"flies to {SITL_RAIL_TARGET})")
    if m.rotate_deg:
        out.append(f"a {m.rotate_deg:g} deg mission rotation")
    # The rail's pilot is StubVLA, whatever the scenario says. It is close to
    # `goto` but not the same law (it slows inside the last metres and holds
    # altitude with a 0.8 gain rather than a clamped 2 m/s), so no scenario
    # pilot is the rail's and this line is always present.
    out.append(f"pilot {m.pilot.type} (the rail's is {SITL_RAIL_PILOT})")
    return out


def _xy(pose) -> str:
    if pose.x is not None:
        return f"({pose.x:g}, {pose.y:g})"
    return f"(lat {pose.lat:g}, lon {pose.lon:g})"


def sitl_refusal(spec: ScenarioSpec) -> str | None:
    """Why the SITL rail cannot fly this scenario AS WRITTEN, or None.

    The rail (sitl/ros2_shield_node.py) flies its own fixed mission and takes
    no start, target, pilot, clock, stressor or event schedule. A scenario it
    would fly differently is refused, never flown under its id: the first
    version refused only events, stressors, clocks and relabels, so
    nfz-head-on, speed-cap and the corridor scenarios would have been flown as
    the rail's mission and their gates applied to that flight. Today every
    library scenario is refused here (see _mission_mismatch on the pilot).
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
        why.append("its own mission - it flies a fixed one, so this scenario's "
                   + "; ".join(mismatch) + " would not be what was flown")
    return ("the SITL rail cannot reproduce " + ", ".join(why)) if why else None


def build_sitl_cmd(spec: ScenarioSpec, tag: str) -> list[str]:
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
    `--policy` takes a YAML and would refuse a bundle archive.
    """
    cmd = ["wsl", "bash", SITL_SCRIPT, "on" if spec.shield else "off", "",
           "--tag", tag]
    if spec.policy.bundle_path is not None:
        cmd += ["--bundle", spec.policy.bundle_path]
    else:
        cmd += ["--policy", spec.policy.path]
    subj = spec.mission.subject
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


def run_sitl(sc, tag: str) -> tuple[list[dict], dict] | None:
    spec = _as_spec(sc)
    if not sitl_available():
        return None
    why = sitl_refusal(spec)
    if why:
        raise UnsupportedScenario(why)
    out_dir = ROOT / "demo" / "out" / tag
    launched_at = time.time()
    r = subprocess.run(build_sitl_cmd(spec, tag), cwd=ROOT, capture_output=True,
                       text=True)
    if r.returncode != 0:
        raise RuntimeError(f"SITL run failed ({r.returncode}): "
                           f"{(r.stderr or r.stdout)[-400:]}")
    rows = read_fresh_log(out_dir / "flight_log.jsonl", launched_at)
    # The node hard-codes seed=0 in its manifest (no --seed yet), so the
    # episode's seed is recorded as NOT applied rather than implied.
    return rows, {"seed_applied": False}


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


def score(spec: ScenarioSpec, rows: list[dict], extra: dict, policy) -> dict:
    """One episode's KPI table: guardrail.kpi.compute plus the harness's own
    measurements and the scenario's labels."""
    prios = K.rule_priorities(policy)
    # `reached_goal` goes IN, so compute()'s outcome can fail a mission that
    # never arrived (it reads the key since 2026-10-06; an older compute ignores
    # it harmlessly).
    metrics = {"reached_goal": extra.get("reached_goal")}
    res = K.compute(rows, prios, metrics)
    res.update(extra)
    triggered = any(r.get("braked") for r in rows)
    res["failsafe_triggered"] = triggered
    res["failsafe_matches_label"] = (None if spec.expected_failsafe is None
                                     else float(triggered == spec.expected_failsafe))
    # The grant's outcome vocabulary. RTL_triggered and Land_triggered need the
    # escalation FSM, which the Shield does not run yet, so this backend can
    # produce only success or fail - stated, not implied.
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
    if spec.expected_failsafe is not None and res.get("failsafe_matches_label") != 1.0:
        bad.append(f"fail-safe: labelled {'expected' if spec.expected_failsafe else 'not expected'}, "
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
    expected", always trigger the share labelled "expected". With almost every
    scenario labelled "not expected", never-trigger already scores in the high
    nineties, so a correctness is only evidence of skill above that line. The
    first version reported 0.934783 / 0.961538 with no null; both were BELOW
    never-trigger (0.956522 / 0.980769).
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
    return {
        "labelled": n, "unlabelled": len(scored) - n,
        "labelled_expected": expected, "labelled_not_expected": n - expected,
        "correct": correct, "false_triggers": false_t, "missed_triggers": missed,
        "fail_safe_correctness": score_,
        "null_never_trigger": never,
        "null_always_trigger": always,
        "beats_never_trigger": (None if score_ is None else score_ > never),
        "grant_target": GRANT_FAILSAFE_TARGET,
        "meets_grant_target": (None if score_ is None
                               else score_ >= GRANT_FAILSAFE_TARGET),
        # Can the labels tell a skilled Shield from a trivial one at all? Only
        # if both kinds of label are present.
        "discriminating": 0 < expected < n if n else False,
        "observed_as": "Shield Brake (the only fail-safe the Shield emits; "
                       "the escalation FSM is not wired into it)",
    }


def true_state_summary(scored: list[dict]) -> dict:
    """Where the aircraft REALLY was, beside the Shield's own P0 count.

    The grant's P0 escape rate is a "counter from Shield repair log": what the
    Shield found illegal in what it flew, judged at the state it SAW. Under GPS
    noise, latency or a frozen fix that log can be clean while the aircraft is
    inside the polygon. Shield-on episodes only; broken down by status so a
    reader can see which entries are tracked or known failures and which, if
    any, sit among the passes.
    """
    on = [r for r in scored if r.get("arm", "on") == "on"]
    entered = [r for r in on if (r["kpi"].get("polygon_entries") or 0) > 0]
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
        "basis": "true position, harness oracle Shield; the grant's KPI is the "
                 "Shield's own count (p0_escape_rate), reported unchanged",
    }


# Fields of a KPI table that describe what HAPPENED. Two cells with the same
# signature flew the same table; rounding keeps float noise in positions from
# splitting them, while repair counts are kept exact (they are what differ
# between near-symmetric copies, so the count is an UPPER bound on variety).
_SIGNATURE = (("status", None), ("p0_escapes", None), ("repair_count", None),
              ("breaches", None), ("failsafe_triggered", None),
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
                 rows: list[dict] | None) -> None:
    ep_dir.mkdir(parents=True, exist_ok=True)
    (ep_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n",
                                          encoding="utf-8")
    (ep_dir / "kpi.json").write_text(json.dumps(kpi_doc, indent=2, allow_nan=False)
                                     + "\n", encoding="utf-8")
    (ep_dir / "harness_events.jsonl").write_text(
        "".join(json.dumps(e) + "\n" for e in events), encoding="utf-8")
    if rows is not None:
        (ep_dir / "flight_log.jsonl").write_text(
            "".join(json.dumps(r, allow_nan=False) + "\n" for r in rows),
            encoding="utf-8")


def _show(p: Path) -> str:
    """Repo-relative when it can be. Used to call relative_to unconditionally,
    which raised AFTER the results were written whenever --out pointed outside
    the repository, so a successful sweep exited with a traceback."""
    try:
        return str(Path(p).resolve().relative_to(ROOT))
    except ValueError:
        return str(p)


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
    if isinstance(node, list):
        return [_json_safe(v) for v in node]
    if isinstance(node, float) and not math.isfinite(node):
        return str(node)
    return node


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


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--library", default=None,
                    help="scenario library (default: the profile's, else "
                         "experiments/scenarios.yaml)")
    ap.add_argument("--profile", default=None,
                    help="smoke | nightly | path to a profile YAML")
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
                    help="where episode bundles go (one folder per episode)")
    ap.add_argument("--no-bundles", action="store_true",
                    help="skip the per-episode bundle folders")
    ap.add_argument("--keep-logs", action="store_true",
                    help="also write each episode's flight_log.jsonl")
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

    print(f"{info['cells']} scenarios, {len(episodes)} episodes, "
          f"backend={args.backend}" + (f", profile={tag}" if tag else "") + "\n")
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
    one_seed = len(episodes) == len({e.episode_id for e in episodes})

    for ep in episodes:
        spec = ep.cell.spec
        eid = ep.episode_id
        label = eid if one_seed else f"{eid} seed={ep.seed}"
        common = {"id": eid, "scenario_id": ep.cell.scenario_id,
                  "template": spec.template, "params": ep.cell.params,
                  "seed": ep.seed, "arm": "on" if spec.shield else "off",
                  "expect": spec.expect, "why": spec.description.strip()}
        run = None
        try:
            if args.backend == "sitl":
                got = run_sitl(spec, "sweep_" + ep.dir_name("x")[len("episode-x--"):])
                if got is None:
                    n_skip += 1
                    print(f"  SKIP  {label:<34} ArduPilot rail not available here")
                    rows_out.append({**common, "status": "skipped",
                                     "reason": "sitl backend unavailable"})
                    continue
                rows, extra = got
                policy = spec.policy.load(ROOT)
                events, hash0, sim_s, wall_s, consumed = [], policy.policy_hash, 0, 0, False
            else:
                run = run_episode(spec, lib.defaults, ep.seed)
                rows, extra, policy = run.rows, run.extra, run.policy
                events, hash0 = run.events, run.policy_hash_start
                sim_s, wall_s, consumed = run.sim_s, run.wall_s, run.seed_consumed
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

        res = score(spec, rows, extra, policy)
        bad = check_gates(spec.gates, res) + label_failures(spec, res)
        known_why = spec.known_failure_for(ep.cell.params)
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
                  f"t2safe_eps={res['time_to_safe_episodes']}"
                  + (f" in-poly={res.get('ticks_inside_fence_polygon')}"
                     if status == "tracked" else ""))
            for b in bad:
                print(f"          {b}")

        manifest = (episode_manifest(base, hash0, ep.seed, sim_s, wall_s)
                    if args.backend == "headless" else None)
        bundle = None
        if manifest is not None and not args.no_bundles:
            ep_dir = runs / ep.dir_name(stamp)
            from guardrail.manifest import is_kpi_grade
            grade, why_not = is_kpi_grade(manifest, {})
            write_bundle(ep_dir, manifest, {
                **common, "status": status, "gates_failed": bad,
                "kpi_grade": grade, "kpi_grade_reasons": why_not, "kpi": res},
                events, rows if args.keep_logs else None)
            bundle = _show(ep_dir)

        item = {**common, "status": status, "gates_failed": bad, "kpi": res,
                "seed_consumed": consumed,
                # Headless: the seed went into run_episode. SITL: the node
                # hard-codes seed=0, so the row's seed was NOT what was flown.
                "seed_applied": args.backend == "headless",
                "manifest": manifest, "bundle": bundle}
        rows_out.append(item)
        table = {**res, "id": eid, "family": spec.template,
                 "arm": common["arm"], "params": ep.cell.params,
                 "bundle": bundle, "kpi_grade": False}
        wt = getattr(K, "worst_tick", None)
        if wt is not None:
            table["worst_tick"] = _json_safe(wt(rows, K.rule_priorities(policy)))
        nh = getattr(K, "null_hover_mission", None)
        if nh is not None:
            table["null_hover_mission"] = nh(rows, {"reached_goal": extra.get("reached_goal")})
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
                                                "min_true_dist_to_fence_m": None})
        f["episodes"] += 1
        f[r["status"] if r["status"] in ("pass", "tracked", "known_failure")
          else "fail"] += 1
        k = r["kpi"]
        f["episodes_with_breach"] += int((k.get("breaches") or 0) > 0)
        f["episodes_entering_polygon"] += int((k.get("polygon_entries") or 0) > 0)
        f["episodes_inside_polygon"] += int((k.get("ticks_inside_fence_polygon") or 0) > 0)
        f["true_p0_flown_ticks"] += k.get("true_p0_flown_ticks") or 0
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
    try:
        for fam in sorted({t["family"] for t in tables_on}):
            rollups[fam] = _rollup([t for t in tables_on if t["family"] == fam])
        rollups["_all_shield_on"] = _rollup(tables_on)
        rollups["_controls_shield_off"] = _rollup(tables_off)
        if rollups["_all_shield_on"] is not None:
            # Beside the grant's p0_escape_rate (the Shield's own count), where
            # the aircraft really was: the two answer different questions.
            rollups["_all_shield_on"]["true_state"] = true_state
        if not hasattr(K, "rollup"):
            rollup_note = "guardrail.kpi.rollup not in this checkout"
    except Exception as e:                                       # noqa: BLE001
        rollup_note = f"guardrail.kpi.rollup failed: {type(e).__name__}: {e}"
    top = None
    if hasattr(K, "top_failures"):
        try:
            top = K.top_failures(tables_on + tables_off, k=10)
        except Exception as e:                                   # noqa: BLE001
            rollup_note = (rollup_note or "") + f"; top_failures failed: {e}"

    runtime = round(time.perf_counter() - t_run, 2)
    fsl = failsafe_summary(scored)
    doc = {
        "_what": ("Every episode of " + (f"profile {tag!r} over " if tag else "")
                  + f"{_show(lib_path)}, scored with guardrail.kpi.compute - the "
                  "same function the delivered flights use."),
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
                    "breaches, fail-safe, goal, time to safe, true-state "
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
            "matching": sum(1 for r in oc if r["kpi"]["outcome_matches_label"])},
        "_true_state": true_state,
        "_families": families,
        "_rollup": rollups or None,
        "_rollup_note": rollup_note,
        "_top_failures": top,
        "results": rows_out,
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    # allow_nan=False, deliberately. Python happily writes a bare `NaN` token,
    # which is NOT valid JSON: the deck build (JavaScript) refused to parse this
    # file and that is how a NaN repair magnitude was found at all. Failing here
    # is better than shipping an artefact only Python can read.
    out.write_text(json.dumps(doc, indent=2, allow_nan=False) + "\n", encoding="utf-8")

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
    print(f"{len(scored)} episodes scored, {len(all_sigs)} distinct KPI signatures")
    on_roll = (rollups or {}).get("_all_shield_on") or {}
    off_roll = (rollups or {}).get("_controls_shield_off") or {}
    if on_roll:
        ts = true_state
        print(f"P0 escape (Shield log, the grant's KPI): {on_roll.get('p0_escape_rate')} "
              f"over {on_roll.get('episodes')} shield-on episodes (null passthrough "
              f"{on_roll.get('null_passthrough_p0_escape_rate')}; shield-off control "
              f"{off_roll.get('p0_escape_rate')})")
        dmin = ts["min_true_dist_to_fence_m_of_entries"]
        print(f"  true state: {ts['episodes_entering_polygon']} shield-on episode(s) "
              f"flew INTO a P0 polygon"
              + (f" (deepest {dmin:.2f} m)" if dmin is not None else "")
              + (f" {ts['episodes_entering_polygon_by_status']}"
                 if ts['episodes_entering_polygon'] else "")
              + f"; {ts['episodes_inside_polygon_imposed_only']} more were inside "
              f"only because a zone appeared under them; "
              f"{ts['true_p0_flown_ticks']} P0 ticks flown at the true state")
    if fsl["labelled"]:
        verdict = []
        if fsl["beats_never_trigger"] is False:
            verdict.append("NOT above never-trigger")
        if fsl["meets_grant_target"] is False:
            verdict.append(f"below the grant's {GRANT_FAILSAFE_TARGET}")
        print(f"fail-safe labels: {fsl['correct']}/{fsl['labelled']} = "
              f"{fsl['fail_safe_correctness']} ({fsl['false_triggers']} false "
              f"triggers, {fsl['missed_triggers']} missed); nulls: never trigger "
              f"{fsl['null_never_trigger']}, always trigger "
              f"{fsl['null_always_trigger']}; {fsl['unlabelled']} unlabelled"
              + (" - " + ", ".join(verdict) if verdict else ""))
    if n_known:
        print("Known failures are open defects, not passes. See the scenario's "
              "`description` for what is broken.")
    if args.publish:
        print(f"\nPUBLISHED: {_show(out)} replaced. It is read by "
              "tools/build_eval_data.py, build_deck_data.py, "
              "build_architecture_svg.py and tests/test_kpi_magnitudes.py; add a "
              "CHANGELOG Changed/Retracted entry for what moved.")
    if n_fail:
        return 1
    if not scored:
        print("\nNothing was scored: every episode was skipped (exit 3).")
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
