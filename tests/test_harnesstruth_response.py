"""The stress harness's vehicle-response arm: the measured ArduPilot response,
flown as a labelled comparison beside the ideal vehicle.

    python tests/test_harnesstruth_response.py      (no pytest needed)

WHY THIS FILE EXISTS

Until 2026-10-19 the headless harness moved its vehicle at exactly the
commanded ground velocity on every tick (experiments/sweep_scenarios.py, "in
still air every family flies what it is commanded") - the same constant-
velocity model the Shield forecasts with. So no episode of the library could
show the distance a real aircraft carries while it brakes, or what the
escalation FSM does when the aircraft cannot turn on the tick. The arm adds
acceleration and jerk limits and the position controller's lag, MEASURED from
the recorded ArduPilot flights (tools/measure_vehicle_response.py, docs/
DESIGN-harness-vehicle-response.md), as a comparison ARM exactly like the 3 s
forecast arm: labelled in every bundle and KPI row, and kept out of every
headline number, pin and exit code. The ideal vehicle stays the default, and
flies exactly what it flew before.

Shown failing first: every test below fails or errors when this file is run
from a pristine copy of c77c77d (`git archive c77c77d | tar -x`, this file
copied into its tests/): there RESPONSE_ARMS, VehicleResponse, the `response`
argument, `--response-arms` and the `/response-ardupilot` label do not exist.
"""
from __future__ import annotations

import json
import math
import random
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))
sys.path.insert(0, str(ROOT / "tools"))

import sweep_scenarios as S                                       # noqa: E402
from guardrail.models import Action4D, State                      # noqa: E402
from guardrail.scenario_spec import (Library, Profile,            # noqa: E402
                                     check_comparison_arms, check_response_arms,
                                     expand)

LIBRARY = Library(ROOT / "experiments" / "scenarios.yaml")
FAMILIES = LIBRARY.by_id()
SKIP = "SKIP"
TAG = "response-ardupilot"
WIND_CELL = "wind-within-authority@lane_y=15,wind_speed_mps=5"


def _main(argv):
    """sweep_scenarios.main, with an argparse exit (an unknown flag on an older
    harness) reported as this test's failure rather than ending the run."""
    try:
        return S.main(argv)
    except SystemExit as e:
        raise AssertionError(f"main() exited {e.code} on {argv}") from None


def _refused(fn, *errors) -> str:
    try:
        fn()
    except errors or (ValueError,) as e:              # noqa: B030
        return str(e)
    raise AssertionError(f"{fn} was not refused")


def _spec(scenario_id: str):
    return FAMILIES[scenario_id].first.spec


def _cell(cell_id: str):
    fam = FAMILIES[cell_id.split("@", 1)[0]]
    return next(c for c in fam.cells() if c.episode_id == cell_id)


# --------------------------------------------------------------------------- #
# the arm's vocabulary
# --------------------------------------------------------------------------- #

def test_the_response_arms_are_refused_like_the_forecast_arms():
    """"ideal" always, "ardupilot" beside it, never alone, never repeated,
    never an unknown name - and never in the same run as a comparison
    forecast, whose episodes would shadow neither twin."""
    assert S.RESPONSE_ARMS == ("ideal", "ardupilot")
    assert check_response_arms(["ideal", "ardupilot"]) == ["ideal", "ardupilot"]
    for bad in ([], ["ardupilot"], ["ideal", "ideal"], ["ideal", "wobbly"]):
        _refused(lambda bad=bad: check_response_arms(bad))
    check_comparison_arms(["grant", "3s"], ["ideal"])
    check_comparison_arms(["grant"], ["ideal", "ardupilot"])
    assert "one comparison dimension" in _refused(
        lambda: check_comparison_arms(["grant", "3s"], ["ideal", "ardupilot"]))
    base = dict(profile="p", cadence="c", grant_scope="g", grant_sim_speedup="s",
                seeds=[1], expected_cells=[1, 5])
    assert Profile(**base).response_arms == ["ideal"]
    assert Profile(**base, response_arms=["ideal", "ardupilot"]).response_arms[1] == "ardupilot"
    for bad in (dict(response_arms=["ardupilot"]),
                dict(response_arms=["ideal", "ardupilot"], forecast_arms=["grant", "3s"])):
        _refused(lambda bad=bad: Profile(**base, **bad), ValueError)


def test_expand_tags_the_response_copy_and_counts_it_apart():
    """The arm multiplies episodes, never cells; its copy carries the tag in
    its run id and bundle name, and is counted in its own key, not in
    `episodes`. With the ideal arm alone the expansion is what it was."""
    eps, info = expand(LIBRARY, only=["nfz-head-on-control"],
                       response_arms=["ideal", "ardupilot"])
    assert [e.response for e in eps] == ["ideal", "ardupilot"], [e.run_id for e in eps]
    assert eps[1].arm_tag == TAG and eps[1].run_id == f"nfz-head-on-control#{TAG}"
    assert TAG in eps[1].dir_name("T") and TAG not in eps[0].dir_name("T")
    assert [e.comparison for e in eps] == [False, True]
    assert (info["cells"], info["episodes"], info["episodes_with_comparison"]) == (1, 1, 2)
    assert info["arms"]["response_arms"] == ["ideal", "ardupilot"]
    assert info["arms"]["response_comparison_episodes"] == 1
    assert info["arms"]["forecast_comparison_episodes"] == 0
    plain, pinfo = expand(LIBRARY, only=["nfz-head-on"])
    same, sinfo = expand(LIBRARY, only=["nfz-head-on"], response_arms=["ideal"])
    assert [e.run_id for e in plain] == [e.run_id for e in same]
    assert pinfo == sinfo and pinfo["arms"]["response_comparison_episodes"] == 0


# --------------------------------------------------------------------------- #
# the ideal vehicle is untouched
# --------------------------------------------------------------------------- #

def test_the_ideal_vehicle_flies_exactly_what_it_flew_before():
    """The default and an explicit "ideal" fly the same rows, with none of the
    arm's per-row keys, and the extras carry only the label. Pinned against
    the values these episodes flew at c77c77d (before the arm): nfz-head-on
    held 20.6 m from the fence; failsafe-rtl-zone's modelled RTL ended 6.0 m
    from its zone; wind-gusts drew the same seeded gusts."""
    pins = {"nfz-head-on": 20.6, "failsafe-rtl-zone": 6.0}
    for sid in ("nfz-head-on", "failsafe-rtl-zone", "wind-gusts", "altitude-recovery-depth"):
        a = S.run_episode(_spec(sid), LIBRARY.defaults, 1001)
        b = S.run_episode(_spec(sid), LIBRARY.defaults, 1001, response="ideal")
        assert json.dumps(a.rows, default=str) == json.dumps(b.rows, default=str), sid
        assert not any("vehicle_v" in r for r in a.rows), sid
        assert a.extra["vehicle_response"] == {"arm": "ideal", "params": None,
                                               "model": "commanded ground velocity, every tick"}
        assert "max_vehicle_speed" not in a.extra and "max_velocity_lag_mps" not in a.extra
        assert a.events[0]["vehicle_response"]["arm"] == "ideal"
        if sid in pins:
            assert a.extra["min_true_dist_to_fence_m"] == pins[sid], (sid, a.extra)


# --------------------------------------------------------------------------- #
# the model
# --------------------------------------------------------------------------- #

P = S.ResponseParams(delay_s=0.1, accel_mps2=2.4, jerk_mps3=5.7, tau_s=0.03,
                     accel_z_mps2=2.9, jerk_z_mps3=20.0, tau_z_s=0.12)


def test_the_response_obeys_its_delay_acceleration_and_jerk():
    """A hover-to-4 m/s step: nothing moves before the delay, the target's
    acceleration never passes the limit nor changes faster than the jerk
    limit allows, and the airframe settles on the command."""
    veh = S.VehicleResponse(P, v0=(0.0, 0.0, 0.0))
    veh.command((4.0, 0.0, 0.0))
    h = P.substep_s
    prev_a, max_a, max_j = 0.0, 0.0, 0.0
    moved_before_delay = 0.0
    for k in range(int(6.0 / h)):
        veh.advance(h)
        a = math.hypot(*veh.at)
        max_a = max(max_a, a)
        max_j = max(max_j, abs(a - prev_a) / h)
        prev_a = a
        if veh.t <= P.delay_s - 1e-9:
            moved_before_delay = max(moved_before_delay, abs(veh.velocity[0]))
    assert moved_before_delay == 0.0, moved_before_delay
    assert max_a <= P.accel_mps2 + 1e-9, max_a
    assert max_j <= P.jerk_mps3 + 1e-6, max_j
    assert max_a > 0.95 * P.accel_mps2, f"a 4 m/s step should reach the limit: {max_a}"
    assert abs(veh.velocity[0] - 4.0) < 1e-3 and abs(veh.velocity[1]) < 1e-9, veh.velocity


def test_the_braking_distance_lies_between_its_closed_form_bounds():
    """From cruise to a zero command, the airframe covers at least the delay's
    distance plus v^2 / 2a, and at most that plus the jerk ramp's and the
    lag's share (v * (accel / jerk + tau)). The ideal vehicle covers 0."""
    for v0 in (4.0, 8.0, 12.0):
        veh = S.VehicleResponse(P, v0=(v0, 0.0, 0.0))
        dist, n = 0.0, 0
        veh.command((0.0, 0.0, 0.0))
        while veh.velocity[0] > 1e-3 and n < 2000:
            dist += veh.advance(0.1)[0] * 0.1
            n += 1
        lo = v0 ** 2 / (2 * P.accel_mps2) + v0 * P.delay_s
        hi = lo + v0 * (P.accel_mps2 / P.jerk_mps3 + P.tau_s) + 0.1
        assert lo - 1e-6 <= dist <= hi, (v0, lo, dist, hi)
    # ... which at the measured parameters is metres, not centimetres.
    veh = S.VehicleResponse(S.AP_RESPONSE, v0=(4.0, 0.0, 0.0))
    veh.command((0.0, 0.0, 0.0))
    carry = sum(veh.advance(0.1)[0] * 0.1 for _ in range(60))
    assert 3.0 < carry < 6.0, carry


def test_every_limit_off_is_the_ideal_vehicle_and_a_bad_command_holds():
    """With no delay, no limits and no lag the mean velocity of every tick is
    its command (to float rounding), whatever the command; a non-finite
    component holds the active command's and is counted."""
    off = S.ResponseParams(0.0, math.inf, math.inf, 0.0, math.inf, math.inf, 0.0)
    veh = S.VehicleResponse(off, v0=(1.0, 0.0, 0.0))
    rng = random.Random(7)
    for _ in range(50):
        c = (rng.uniform(-12, 12), rng.uniform(-12, 12), rng.uniform(-3, 3))
        got = veh.step(c, 0.1)
        assert all(abs(g - x) < 1e-9 for g, x in zip(got, c)), (got, c)
    veh = S.VehicleResponse(off, v0=(0.0, 0.0, 0.0))
    veh.step((1.0, 1.0, 0.0), 0.1)
    got = veh.step((float("nan"), 2.0, float("inf")), 0.1)
    assert all(abs(g - x) < 1e-9 for g, x in zip(got, (1.0, 2.0, 0.0))), got
    assert veh.nonfinite_commands == 2
    assert S.VehicleResponse(off, v0=(float("nan"), 1.0, 0.0)).velocity == (0.0, 1.0, 0.0)


def test_the_entry_velocity_is_the_pilots_held_to_the_kinematic_envelope():
    """The airframe enters at the pilot's heading and the policy's legal speed
    (sim_demo_policy: 4 m/s, climb 2 m/s), never at an asked-for 20 m/s, and
    a non-finite component enters as 0."""
    pol = _spec("nfz-head-on").policy.load(ROOT)
    vx, vy, vz = S.entry_velocity(Action4D(vx=12.0, vy=16.0, vz_up=5.0), pol)
    assert abs(math.hypot(vx, vy) - 4.0) < 1e-9 and abs(vx / vy - 0.75) < 1e-9, (vx, vy)
    assert vz == 2.0
    assert S.entry_velocity(Action4D(vx=float("nan"), vy=1.0), pol) == (0.0, 1.0, 0.0)


def test_the_goto_pilot_and_the_rtl_close_on_their_targets_only_under_the_arm():
    """Under the arm the goto pilot slows into its goal and the modelled RTL
    flies position targets, so neither overshoots by a braking distance; with
    no response both fly exactly the old commands."""
    m = S._Mission(start=State(x=0.0, y=0.0, up=15.0), target=(10.0, 0.0, 15.0),
                   subject=None, action=None, speed=4.0)
    for d, ideal_v in ((8.0, 4.0), (4.0, 4.0), (2.9, 0.0)):
        st = State(x=10.0 - d, y=0.0, up=15.0)
        assert S._pilot(m, "goto", st).vx == ideal_v
        shaped = S._pilot(m, "goto", st, 2.4).vx
        want = 0.0 if d < S.GOAL_TOL_M else min(4.0, math.sqrt(2 * 2.4 * (d - 2.4 / 2)))
        assert abs(shaped - want) < 1e-9, (d, shaped, want)
    home = (0.0, 0.0)
    low = State(x=30.0, y=0.0, up=10.0)
    assert S._autopilot("RTL", low, home, 0.1) == Action4D(vz_up=S.AP_CLIMB_MPS)
    up = S._autopilot("RTL", low, home, 0.1, S.AP_RESPONSE)
    assert up.vx == 0.0 and 0.0 < up.vz_up <= S.AP_CLIMB_MPS, up
    high = State(x=30.0, y=0.0, up=15.5)
    old = S._autopilot("RTL", high, home, 0.1)
    assert (old.vx, old.vz_up) == (-S.AP_RTL_SPEED_MPS, 0.0), old
    new = S._autopilot("RTL", high, home, 0.1, S.AP_RESPONSE)
    assert new.vx == -S.AP_RTL_SPEED_MPS and new.vz_up < 0.0, new
    near = S._autopilot("RTL", State(x=2.5, y=0.0, up=15.0), home, 0.1, S.AP_RESPONSE)
    assert -S.AP_RTL_SPEED_MPS < near.vx < 0.0, near
    # An RTL entered above RTL_ALT returns at the altitude it was entered at
    # (ArduPilot's RTL_ALT is a minimum), so it closes on that, not on 15 m.
    above = S._autopilot("RTL", State(x=30.0, y=0.0, up=20.0), home, 0.1, S.AP_RESPONSE,
                         rtl_alt=20.0)
    assert above.vx == -S.AP_RTL_SPEED_MPS and above.vz_up == 0.0, above


def test_an_rtl_entered_above_rtl_alt_returns_at_that_altitude_on_both_vehicles():
    """failsafe-rtl-zone flown at 20 m (above RTL_ALT, 15 m): the ideal
    vehicle's RTL flies home at 20 m, as ArduPilot's does ("if the vehicle is
    flying higher than RTL_ALT it will return at its current altitude"), and
    so must the arm's. Shown failing first: until 2026-10-20 the arm's RTL
    closed on 15 m always and descended 5 m on the way home (min 15.0 m)."""
    from guardrail.scenario_spec import ScenarioSpec
    body = _spec("failsafe-rtl-zone").model_dump(mode="json", by_alias=True)
    body["mission"]["start_pose"]["up"] = body["mission"]["target"]["up"] = 20.0
    spec = ScenarioSpec.model_validate(body)
    for arm in ("ideal", "ardupilot"):
        run = S.run_episode(spec, LIBRARY.defaults, 1001, response=arm)
        rtl = [r["up"] for r in run.rows if r.get("autopilot") == "RTL"]
        assert len(rtl) > 10, (arm, len(rtl))
        assert min(rtl) > 19.9 and max(rtl) < 20.1, (arm, min(rtl), max(rtl))


# --------------------------------------------------------------------------- #
# what the arm shows
# --------------------------------------------------------------------------- #

def test_braking_carry_shows_under_the_arm_and_nowhere_else():
    """nfz-head-on: the ideal vehicle stops dead 20.6 m from the fence; the
    measured response carries it metres closer, and says how far its velocity
    fell behind the command (an arm with no lag would be the ideal vehicle
    under another name)."""
    spec = _spec("nfz-head-on")
    ideal = S.run_episode(spec, LIBRARY.defaults, 1001)
    ap = S.run_episode(spec, LIBRARY.defaults, 1001, response="ardupilot")
    d_i, d_a = (ideal.extra["min_true_dist_to_fence_m"],
                ap.extra["min_true_dist_to_fence_m"])
    assert d_i == 20.6 and d_a < d_i - 2.0, (d_i, d_a)
    assert ap.extra["vehicle_response"]["arm"] == "ardupilot"
    assert ap.extra["vehicle_response"]["params"] == S.AP_RESPONSE.record()
    assert ap.extra["max_velocity_lag_mps"] > 1.0, ap.extra
    assert ap.extra["max_vehicle_speed"] <= 4.0 + 1e-6, ap.extra
    assert all("vehicle_v" in r for r in ap.rows[:-1])


def test_main_keeps_the_arm_out_of_every_headline_and_labels_every_bundle():
    """A run with both arms: the comparison copies are judged (one fails its
    fail-safe label: under the measured response the FSM's convergence test
    sends wind-within-authority to Loiter and RTL) but never reach `_counts`,
    the headline summaries or the exit code. Every bundle says which vehicle
    flew it, an ArduPilot bundle re-flies from its own extras, and the KPI
    rows and report file the arm under `/response-ardupilot`."""
    with tempfile.TemporaryDirectory() as td:
        out, runs = Path(td) / "s.json", Path(td) / "runs"
        rc = _main(["--only", "nfz-head-on-control", WIND_CELL, "--seed", "1001",
                    "--response-arms", "ideal", "ardupilot", "--out", str(out),
                    "--runs", str(runs), "--keep-logs"])
        doc = json.loads(out.read_text(encoding="utf-8"))
        assert rc == 0, rc
        assert sum(doc["_counts"].values()) == 2, doc["_counts"]
        assert [r["response_arm"] for r in doc["results"]] == ["ideal", "ideal"]
        assert all(TAG not in r["id"] for r in doc["results"])
        rcmp = doc["_response_comparison"]
        assert rcmp["paired_episodes"] == 2 and rcmp["unmatched"] == []
        assert all(TAG in r["id"] for r in rcmp["results"])
        assert rcmp["counts"]["fail"] >= 1, rcmp["counts"]
        fs = rcmp["totals"]
        assert fs["ardupilot"]["failsafe_labels"]["false_triggers"] > \
            fs["ideal"]["failsafe_labels"]["false_triggers"], fs
        assert doc["_failsafe_labels"]["false_triggers"] == fs["ideal"]["failsafe_labels"][
            "false_triggers"], "the headline fail-safe score must be the ideal arm's alone"
        assert rcmp["max_velocity_lag_mps"] > 1.0
        cells = doc["_cell_rollup"]
        tagged = {k: v for k, v in cells.items() if k.endswith("/" + TAG)}
        assert len(tagged) == 2 and all(v["response_arm"] == "ardupilot" for v in tagged.values())
        assert all(v["response_arm"] == "ideal" for k, v in cells.items() if k not in tagged)
        assert "_comparison_response_ardupilot_shield_on" in doc["_rollup"]
        assert doc["_scope"]["arms"]["response_comparison_episodes"] == 2
        dirs = sorted(p for p in runs.iterdir() if p.is_dir())
        assert len(dirs) == 4, [d.name for d in dirs]
        for d in dirs:
            ex = json.loads((d / "manifest_extras.json").read_text(encoding="utf-8"))
            kpi = json.loads((d / "kpi.json").read_text(encoding="utf-8"))
            arm = "ardupilot" if TAG in d.name else "ideal"
            assert ex["arm"]["response"] == arm and ex["vehicle_response"]["arm"] == arm, d.name
            assert kpi["response_arm"] == arm and kpi["kpi"]["vehicle_response"]["arm"] == arm
            start = json.loads((d / "harness_events.jsonl").read_text(
                encoding="utf-8").splitlines()[0])
            assert start["vehicle_response"]["arm"] == arm
            rows = [json.loads(x) for x in (d / "flight_log.jsonl").read_text(
                encoding="utf-8").splitlines() if x]
            assert any("vehicle_v" in r for r in rows) == (arm == "ardupilot"), d.name
            if arm == "ardupilot":
                again = S.rerun_from_extras(ex)
                assert [(r["x"], r["y"]) for r in again.rows] == \
                    [(r["x"], r["y"]) for r in rows], d.name
        report = (runs / "kpi_report.md").read_text(encoding="utf-8")
        assert "## Vehicle-response comparison" in report
        assert f"/shield-on/{TAG}`" in report


def test_main_refuses_what_the_arm_cannot_fly():
    """The published sweep is the headline library run; a 3 s forecast and the
    ArduPilot response in one run would shadow neither twin; the arm without
    its ideal twin is no comparison; and the SITL rail flies ArduPilot for
    real, so a modelled response on top would fly it twice. Each exits 2
    before anything flies."""
    with tempfile.TemporaryDirectory() as td:
        out = str(Path(td) / "s.json")
        assert _main(["--publish", "--response-arms", "ideal", "ardupilot"]) == 2
        assert _main(["--only", "nfz-head-on-control", "--forecast-arms", "grant", "3s",
                      "--response-arms", "ideal", "ardupilot", "--out", out]) == 2
        assert _main(["--only", "nfz-head-on-control", "--response-arms", "ardupilot",
                      "--out", out]) == 2
        assert _main(["--only", "nfz-head-on-control", "--backend", "sitl",
                      "--response-arms", "ideal", "ardupilot", "--out", out]) == 2
        assert not Path(out).exists()


def test_a_rerun_refuses_a_bundle_flown_on_other_response_parameters():
    """A bundle flown before AP_RESPONSE was re-measured would otherwise be
    re-flown on today's vehicle under its old id."""
    spec = _spec("nfz-head-on-control")
    run = S.run_episode(spec, LIBRARY.defaults, 0, response="ardupilot")
    ex = json.loads(json.dumps(S._json_safe(S.episode_extras(
        spec, LIBRARY.defaults, 0, run, response="ardupilot"))))
    assert ex["arm"]["response"] == "ardupilot"
    again = S.rerun_from_extras(ex)
    assert [r["x"] for r in again.rows] == [r["x"] for r in run.rows]
    stale = json.loads(json.dumps(ex))
    stale["vehicle_response"]["params"]["accel_mps2"] = 9.0
    assert "flown with" in _refused(lambda: S.rerun_from_extras(stale))
    # A bundle from before the arm (no key at all) re-flies on the ideal vehicle.
    old = json.loads(json.dumps(ex))
    del old["vehicle_response"]
    del old["arm"]["response"]
    assert not any("vehicle_v" in r for r in S.rerun_from_extras(old).rows)


def test_the_kpi_report_files_the_arm_apart_and_says_nothing_without_it():
    """tools/kpi_report.py: the `/response-ardupilot` family suffix from every
    place the label lives (and none for the ideal vehicle, labelled or not),
    and the sweep page's comparison section only when the arm flew."""
    import kpi_report as KR
    assert KR.response_suffix({"response_arm": "ardupilot"}) == "/" + TAG
    assert KR.response_suffix({"kpi": {"vehicle_response": {"arm": "ardupilot"}}}) == "/" + TAG
    assert KR.response_suffix({"vehicle_response": {"arm": "ardupilot"}}) == "/" + TAG
    assert KR.response_suffix({"id": f"x@a=1#pp0,{TAG}"}) == "/" + TAG
    assert KR.response_suffix({"id": "fsm-response-time"}) == ""
    assert KR.response_suffix({"response_arm": "ideal"}, {}) == ""
    assert KR.response_suffix({}, None) == ""
    assert KR._run_labels({"forecast_arm": "grant", "response_arm": "ardupilot"}) == "/" + TAG
    page = KR.render_sweep_markdown({"_cell_rollup": {}, "_response_comparison": None})
    assert "Vehicle-response" not in page


def test_the_fit_recovers_known_parameters_from_a_synthetic_flight():
    """tools/measure_vehicle_response.py, end to end on a log whose vehicle is
    known: positions flown through VehicleResponse with chosen parameters,
    logged the way the pas_ap rail logs them (a row per ~0.1 s tick, the
    command it sent, the position it read), read back by segments() and
    fitted by the tool's own search from a wrong start. A fitter that cannot
    find a vehicle it was handed cannot be trusted with a recorded one."""
    import measure_vehicle_response as M
    truth = S.ResponseParams(delay_s=0.15, accel_mps2=2.0, jerk_mps3=4.0, tau_s=0.05,
                             accel_z_mps2=math.inf, jerk_z_mps3=math.inf, tau_z_s=0.0)
    rng = random.Random(3)
    veh = S.VehicleResponse(truth, v0=(0.0, 0.0, 0.0))
    t, x, y = 0.0, 0.0, 0.0
    cmd = (0.0, 0.0, 0.0)
    lines = []
    for k in range(600):
        if k % 40 == 0:
            cmd = (rng.uniform(-5, 5), rng.uniform(-5, 5), 0.0)
        lines.append(json.dumps({"t": round(t, 4), "tick": k + 1, "x": x, "y": y, "up": 15.0,
                                 "emitted": {"vx": cmd[0], "vy": cmd[1], "vz_up": 0.0,
                                             "yaw_rate": 0.0},
                                 "flown": True, "setpoint": "pass", "ap_mode": "GUIDED"}))
        dt = 0.1 + rng.uniform(-0.01, 0.01)
        mx, my, _ = veh.step(cmd, dt)
        x, y, t = x + mx * dt, y + my * dt, t + dt
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "flight_log.jsonl"
        p.write_text("\n".join(lines) + "\n", encoding="utf-8")
        segs = M.segments(p)
    assert len(segs) == 1 and len(segs[0]["t"]) == 600

    def f(xx):
        return M.pas_loss(M._params(xx), segs)[0]
    got, rms, _ = M.pattern_search(
        f, {"delay_s": 0.4, "accel_mps2": 3.5, "jerk_mps3": 8.0, "tau_s": 0.2},
        steps={"delay_s": 0.1, "accel_mps2": 0.5, "jerk_mps3": 2.0, "tau_s": 0.1},
        lo={"delay_s": 0.0, "accel_mps2": 0.3, "jerk_mps3": 0.5, "tau_s": 0.0},
        hi={"delay_s": 1.5, "accel_mps2": 10.0, "jerk_mps3": 60.0, "tau_s": 2.0},
        max_evals=300)
    ideal = f({"delay_s": 0.0, "accel_mps2": math.inf, "jerk_mps3": math.inf, "tau_s": 0.0})
    assert rms < 0.05 < ideal, (rms, ideal)
    assert abs(got["delay_s"] - 0.15) <= 0.06, got
    assert abs(got["accel_mps2"] - 2.0) <= 0.3, got
    assert abs(got["jerk_mps3"] - 4.0) <= 1.0, got


def test_the_harness_constants_explain_the_recorded_flights():
    """AP_RESPONSE against the recorded ArduPilot flights it was fitted to,
    through the tool's own losses: far better than the ideal vehicle and than
    the best pure delay on the fresh-position pas_ap runs (fitted 2026-10-19:
    0.282 m/s at these constants, 0.259 for pas_ap's own optimum, 0.388 for
    a 0.9 s delay, 0.773 ideal), within a few per cent of the backlogged
    sitl_* runs' own optimum (0.216; ideal 0.469), and better than ideal
    vertically. The full re-fit, which re-derives every constant, is the
    tool's `--check` (about 4 minutes, too long for every CI run). The
    recordings are gitignored (demo/out of the main checkout): on a host
    without them this is skipped, by name."""
    import measure_vehicle_response as M
    dirs = M.default_out_dirs()
    if not all(M.find_run(n, dirs) for n in M.PAS_RUNS + M.SITL_RUNS):
        print("      SKIP: the recorded pas_ap_* / sitl_* flights are not on this host")
        return SKIP
    pas = [s for n in M.PAS_RUNS for s in M.segments(M.find_run(n, dirs))]
    sitl = {n: M.segments(M.find_run(n, dirs)) for n in M.SITL_RUNS}
    ideal = S.ResponseParams(0.0, math.inf, math.inf, 0.0, math.inf, math.inf, 0.0)
    delay = S.ResponseParams(0.9, math.inf, math.inf, 0.0, math.inf, math.inf, 0.0)
    xy, n = M.pas_loss(S.AP_RESPONSE, pas, "xy")
    assert n > 4000, n
    assert xy < 0.30 and xy < 0.8 * M.pas_loss(delay, pas, "xy")[0], xy
    assert xy < 0.4 * M.pas_loss(ideal, pas, "xy")[0], xy
    s_ap, s_n, _ = M.sitl_loss(S.AP_RESPONSE, sitl)
    assert s_n > 900 and s_ap < 0.23 and s_ap < 0.5 * M.sitl_loss(ideal, sitl)[0], s_ap
    z = M.pas_loss(S.AP_RESPONSE, pas, "z")[0]
    z_ideal = M.pas_loss(S.ResponseParams(S.AP_RESPONSE.delay_s, math.inf, math.inf, 0.0,
                                          math.inf, math.inf, 0.0), pas, "z")[0]
    assert z < 0.8 * z_ideal, (z, z_ideal)


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
