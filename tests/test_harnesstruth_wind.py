"""A GUIDED velocity setpoint under wind: ArduCopter's position controller, not
an open-loop integrator.

    python tests/test_harnesstruth_wind.py      (no pytest needed)

WHY THIS FILE EXISTS

Until 2026-10-12 the headless harness flew every Shield setpoint, a Brake's
zero included, as velocity plus wind: x += (v + w) dt. ArduCopter does not.
A GUIDED velocity setpoint runs SubMode::VelAccel (ArduCopter/mode_guided.cpp
velaccel_control_run at the Copter-4.5.7 pin), whose
AC_PosControl::input_vel_accel_xy integrates the velocity target into a
position target, and update_xy_controller closes a P loop on the position
error (PSC_POSXY_P 1.0) and a PID velocity loop with an integrator: a steady
wind within the airframe's authority is rejected, and a zero-velocity Brake
holds its position. The open-loop model parked aircraft at a margin ring and
blew them in, and the escalation FSM answered with fail-safes the labels do
not expect (the tailwind and flicker pins of wind-within-authority).

Shown failing first: every test below fails or errors when this file is run
from a pristine copy of HEAD 4418ae0 (`git archive HEAD | tar -x`, this file
copied into its tests/): there the steady wind moves a braking aircraft, the
tailwind cell still ends in a false fail-safe, and guided_ground_velocity /
the wind_model argument do not exist.
"""
from __future__ import annotations

import math
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

import sweep_scenarios as S                                       # noqa: E402
from guardrail.scenario_spec import Library, ScenarioSpec         # noqa: E402

LIBRARY = Library(ROOT / "experiments" / "scenarios.yaml")
DEFAULTS = LIBRARY.defaults
FAMILIES = LIBRARY.by_id()
SKIP = "SKIP"


def _with(spec: ScenarioSpec, **changes) -> ScenarioSpec:
    d = spec.model_dump(mode="json", by_alias=True)
    for path, value in changes.items():
        node = d
        keys = path.split(".")
        for k in keys[:-1]:
            node = node[k]
        node[keys[-1]] = value
    return ScenarioSpec.model_validate(d)


# A hover far from every zone: the pilot asks for nothing, the Shield has
# nothing to repair, and only the wind can move the aircraft.
HOVER = {"scenario_id": "hover-test", "template": "wind_disturbance",
         "description": "test", "ticks": 100, "escalation": False,
         "mission": {"start_pose": {"x": -100.0, "y": -100.0, "up": 15.0},
                     "pilot": {"type": "constant", "action": {"vx": 0.0}}},
         "policy": {"path": "policies/sim_demo_policy.yaml"},
         "expected_kpis": {"P0_escape_rate": 0}}


def _hover(**stress) -> ScenarioSpec:
    return ScenarioSpec.model_validate(dict(HOVER, stress=stress))


def test_the_model_rejects_wind_up_to_its_authority_and_drifts_by_the_excess():
    g = S.guided_ground_velocity
    assert S.AP_HOLD_AUTHORITY_MPS == 6.4 and S.AP_POSXY_P == 1.0
    assert g(0.0, 0.0, (5.0, 0.0)) == (0.0, 0.0)            # holds against 5 m/s
    assert g(-2.0, 3.0, (5.0, 0.0)) == (-2.0, 3.0)          # flies its command
    vx, vy = g(0.0, 0.0, (12.0, 0.0))                       # 12 m/s: 5.6 m/s drift
    assert abs(vx - 5.6) < 1e-9 and abs(vy) < 1e-12, (vx, vy)
    vx, vy = g(-5.0, 0.0, (12.0, 0.0))                      # 5 m/s upwind: carried
    assert abs(vx - 0.6) < 1e-9, vx                         # 0.6 m/s downwind
    # Still air: the airframe's speed limit is not modelled, as before.
    assert g(20.0, 0.0, (0.0, 0.0)) == (20.0, 0.0)
    # The authority itself is held (a wind of exactly 6.4 m/s is rejected).
    assert g(0.0, 0.0, (6.4, 0.0)) == (0.0, 0.0)
    assert g(0.0, 0.0, (0.0, -6.4)) == (0.0, 0.0)


def test_a_steady_wind_within_authority_does_not_move_a_hovering_aircraft():
    """REGRESSION. The default model is ArduCopter's position controller: a
    zero setpoint holds its position in a 3 m/s wind. (HEAD: 90 m downwind in
    30 s, the wind integrated open-loop.)"""
    calm = S.run_episode(_hover(), DEFAULTS, 0)
    windy = S.run_episode(_hover(wind_speed_mps=3.0, wind_dir_deg=90.0), DEFAULTS, 0)
    assert windy.extra["final"] == calm.extra["final"] == {"x": -100.0, "y": -100.0,
                                                          "up": 15.0}, windy.extra["final"]
    assert windy.extra["wind_model"] == "guided"
    assert windy.events[0]["wind_mps"] == [0.0, 3.0], windy.events[0]["wind_mps"]


def test_the_open_loop_model_is_kept_for_comparison_and_still_drifts():
    run = S.run_episode(_hover(wind_speed_mps=3.0, wind_dir_deg=90.0), DEFAULTS, 0,
                        wind_model="open_loop")
    # 100 ticks, each followed by a 0.1 s step: 3 m/s x 10 s downwind.
    assert abs(run.extra["final"]["y"] - (-100.0 + 3.0 * 10.0)) < 0.05, run.extra["final"]
    try:
        S.run_episode(_hover(), DEFAULTS, 0, wind_model="laminar")
    except ValueError as e:
        assert "wind model" in str(e), e
    else:
        raise AssertionError("an unknown wind model was flown")


def test_a_wind_beyond_authority_drifts_by_the_excess():
    """12 m/s against a 6.4 m/s hold authority: the aircraft is carried
    downwind at 5.6 m/s, whatever it is commanded."""
    run = S.run_episode(_hover(wind_speed_mps=12.0, wind_dir_deg=0.0), DEFAULTS, 0)
    xs = [r["x"] for r in run.rows]
    rates = {round((b - a) / DEFAULTS.dt, 6) for a, b in zip(xs, xs[1:])}
    assert rates == {5.6}, rates


def test_a_gust_is_pulled_back_to_the_position_target():
    """Gusts move the aircraft off its position target and the P loop
    (PSC_POSXY_P) pulls it back: the offset stays bounded (an
    Ornstein-Uhlenbeck process, stationary sigma about 0.12 m for 0.5 m/s
    gusts at 10 Hz) where the open-loop model random-walks."""
    spec = _hover(gust_mps=0.5)
    spec = spec.model_copy(update={"ticks": 3000})
    run = S.run_episode(spec, DEFAULTS, 1001)
    off = [math.hypot(r["x"] + 100.0, r["y"] + 100.0) for r in run.rows]
    sx = statistics.pstdev([r["x"] for r in run.rows[500:]])
    assert max(off) < 0.8 and 0.06 < sx < 0.2, (max(off), sx)
    ol = S.run_episode(spec, DEFAULTS, 1001, wind_model="open_loop")
    drift = max(math.hypot(r["x"] + 100.0, r["y"] + 100.0) for r in ol.rows)
    assert drift > 3 * max(off), (drift, max(off))


def test_the_tailwind_false_failsafe_was_the_open_loop_model():
    """wind-within-authority's tailwind into the zone (lane 15, 0 deg) was a
    known failure from 2026-10-06 to 2026-10-12: the aircraft the Shield parked
    at the ring with a zero setpoint was carried in, the repair exceeded
    theta, LOITER held it and the 30 s timeout sent it home - a fail-safe the
    labels do not expect. ArduCopter's GUIDED holds that zero setpoint: no
    fail-safe, never inside the polygon. The null is the same cell on the old
    open-loop model, where the false trigger is still there."""
    base = FAMILIES["wind-within-authority"].first.spec
    spec = _with(base, **{"mission.start_pose.y": 15.0, "mission.target.y": 15.0,
                          "stress.wind_speed_mps": 3.0, "stress.wind_dir_deg": 0.0})
    run = S.run_episode(spec, DEFAULTS, 0)
    res = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
    assert res["failsafe_triggered"] is False, run.extra["fsm"]["transitions"]
    assert res["failsafe_matches_label"] == 1.0
    assert run.extra["min_true_dist_to_fence_m"] > 0.0, run.extra["min_true_dist_to_fence_m"]
    old = S.run_episode(spec, DEFAULTS, 0, wind_model="open_loop")
    res_old = S.score(spec, old.rows, old.extra, old.policy, old.priorities)
    assert res_old["failsafe_triggered"] is True, old.extra["fsm"]["transitions"]


def test_the_autopilots_own_hold_drifts_beyond_authority_and_holds_within():
    """REGRESSION (review of 2026-10-12, mutant A). ArduCopter's LOITER is
    the same position controller as GUIDED, so the closed loop applies to
    the autopilot's modes too: a hold forced by a zone whose own action is
    `loiter` (X4) drifts with the excess of a 12 m/s crosswind, 5.6 m/s,
    and holds in 5 m/s. On the open-loop comparison model the autopilot
    rejected every wind, as it did until 2026-10-12. A harness that left the
    autopilot out of the closed loop (HEAD's behaviour) passes every other
    test in this file and flips every wind-beyond-authority cell."""
    def hold(wind, model):
        d = {"scenario_id": "loiter-test", "template": "wind_disturbance",
             "description": "test", "ticks": 200,
             "mission": {"start_pose": {"x": -200.0, "y": 0.0, "up": 15.0},
                         "target": {"x": -80.0, "y": 0.0, "up": 15.0},
                         "pilot": {"type": "goto", "speed": 4.0}},
             "policy": {"path": "policies/sim_demo_policy.yaml"},
             "events": [{"at_sim_t": 5.0, "type": "spawn_polygon_fence",
                         "payload": {"id": "hold-zone", "width_m": 30, "height_m": 20,
                                     "ahead_of_vehicle_m": 12,
                                     "violation_action": "loiter"}}],
             "stress": {"wind_speed_mps": wind, "wind_dir_deg": 90.0},
             "expected_kpis": {"P0_escape_rate": 0}}
        run = S.run_episode(ScenarioSpec.model_validate(d), DEFAULTS, 0, wind_model=model)
        edges = [t["edge"] for t in run.extra["fsm"]["transitions"]]
        lo = [r for r in run.rows if r.get("autopilot") == "LOITER"]
        rates = {round((b["y"] - a["y"]) / DEFAULTS.dt, 6)
                 for a, b in zip(lo, lo[1:]) if b["tick"] == a["tick"] + 1}
        return edges, len(lo), rates
    edges, n, rates = hold(12.0, "guided")
    assert edges[0] == "X4" and n > 20 and rates == {5.6}, (edges, n, rates)
    edges, n, rates = hold(5.0, "guided")
    assert edges[0] == "X4" and n > 20 and rates == {0.0}, (edges, n, rates)
    edges, n, rates = hold(12.0, "open_loop")
    assert edges[0] == "X4" and n > 20 and rates == {0.0}, (edges, n, rates)


def test_the_within_authority_cells_are_distinct_flights():
    """REGRESSION (review of 2026-10-12). Within the hold authority the
    closed loop rejects the wind whatever its speed or direction, so the
    first re-cut's 8 cells (2 lanes x 2 speeds x 2 directions) were two
    flights flown four times each, counted 8 times in the headline fail-safe
    score. The family is one wind per lane now: every cell a different
    flight (KPI signature), and each still its lane's calm flight."""
    fam = FAMILIES["wind-within-authority"]
    sigs, finals = [], []
    for c in fam.cells():
        run = S.run_episode(c.spec, DEFAULTS, 0)
        res = S.score(c.spec, run.rows, run.extra, run.policy, run.priorities)
        sigs.append(S.kpi_signature({"status": None, "kpi": res}))
        calm = _with(c.spec, **{"stress.wind_speed_mps": 0.0})
        assert S.run_episode(calm, DEFAULTS, 0).extra["final"] == run.extra["final"], c.params
    assert len(sigs) == len(set(sigs)) >= 2, sigs


def test_the_beyond_authority_cell_is_beyond_the_modelled_authority():
    """wind-beyond-authority keeps its purpose under the closed-loop model:
    every cell's wind is stronger than the modelled hold authority, at least
    one is stronger than the 11.5 m/s of the all-quadratic-drag reading too
    (beyond the airframe whatever its drag split), and a fail-safe is
    expected."""
    fam = FAMILIES["wind-beyond-authority"]
    winds = [c.spec.stress.wind_speed_mps for c in fam.cells()]
    assert min(winds) > S.AP_HOLD_AUTHORITY_MPS and max(winds) > 11.5, winds
    assert all(c.spec.expected_failsafe is True for c in fam.cells())
    assert fam.first.spec.expect == "tracked"
    within = FAMILIES["wind-within-authority"]
    assert max(c.spec.stress.wind_speed_mps for c in within.cells()) < S.AP_HOLD_AUTHORITY_MPS


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
