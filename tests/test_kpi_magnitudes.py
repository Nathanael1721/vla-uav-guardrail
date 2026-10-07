"""The two acceptance KPIs that were never computed.

Run either way:
    pytest tests/test_kpi_magnitudes.py -v
    python tests/test_kpi_magnitudes.py

WHY THIS FILE EXISTS

`Grant overview` names five acceptance KPIs. Until 2026-09-01 this repository
computed three. `mean repair magnitude` and `mean time to safe` had no number
anywhere - not in kpi.json, not in the midterm report, not in any deck - even
though every field they are derived from has been written to `flight_log.jsonl`
since the first flight.

So these tests are not guarding a refactor. They pin down two definitions that
had never been written, and the traps are all in the definitions rather than in
the arithmetic:

  * a magnitude averaged over ALL ticks shrinks as the flight gets longer;
  * a four-channel norm mixes m/s with deg/s and has no unit;
  * an unrecovered episode dropped from the mean makes the worst flight score
    the best;
  * a tick period assumed to be 0.1 s is wrong on the SITL rails.

Each of those has a test below that fails if the easy version is written.

2026-10-06: three more locked KPIs had no number anywhere (audit cards WP3-20 /
X-02, WP4-08, X-03), and they get the same treatment:

  * repair success rate - a Shield that brakes on everything must score 0.0, a
    Shield that never tried must score "no repairs attempted", and a log that
    cannot say must score "not measurable". Three answers, never one zero;
  * mission success - a run that never reaches its goal is not a success,
    however clean its rule record (the sweep scored four of those as passes);
  * average repair count / episode - a mean OVER episodes, which no code
    computed; `rollup()` does, and pools rates instead of averaging them.

The same day's review found that first version wrong in six ways, each now
pinned below by a test that fails on it:

  * a Shield-off arm's counterfactual repairs scored as a measured 0.0;
  * "converged" ignored the grant's theta cap ("magnitude < theta");
  * escalation was read from an invented `failsafe` field instead of the
    grant's `fsm_state_after`;
  * a brake after an unrepairable P1 violation counted as a false trigger;
  * an unsafe position the aircraft entered did not fail the mission;
  * `rollup()` pooled a missing field as a zero.
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import kpi as K                                     # noqa: E402

A0 = {"vx": 0.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0}

SKIP = "SKIP"


def _row(t, raw=None, em=None, vio=False, reps=0):
    """One flight-log row, shaped the way every rail writes them."""
    return {
        "t": t,
        "raw": dict(A0, **(raw or {})),
        "emitted": dict(A0, **(em or {})),
        "violations": ([{"rule_id": "r", "category": "fence"}] if vio else []),
        "emitted_violations": [],
        "repairs": [{"operator": "X", "detail": ""}] * reps,
        "braked": False,
    }


# --------------------------------------------------------------------------- #
# mean repair magnitude
# --------------------------------------------------------------------------- #

def test_repair_magnitude_is_the_euclidean_change_in_velocity():
    """3-4-5: a 3 m/s North cut and a 4 m/s East cut is a 5 m/s repair."""
    rows = [_row(0.0, raw={"vx": 3.0, "vy": 4.0}, em={"vx": 0.0, "vy": 0.0},
                 vio=True, reps=1)]
    res = K.compute(rows, {}, {})
    assert abs(res["mean_repair_magnitude_mps"] - 5.0) < 1e-6, res
    assert res["repaired_ticks"] == 1


def test_yaw_is_reported_apart_and_never_folded_into_the_norm():
    """deg/s and m/s are different quantities.

    A pure yaw repair must show ZERO translational magnitude. If yaw were
    included in the norm this reports the turn rate, and the KPI would silently
    depend on whether it happened to be expressed in degrees or radians.

    And the NAME must be true: Action4D.yaw_rate is rad/s, the field is called
    mean_yaw_repair_dps, so the conversion has to happen. It did not until
    2026-09-08, and eight runs published a figure 57.296x too small - the input
    below is half a radian per second and the answer is 28.6 deg/s, not 0.5.
    """
    rows = [_row(0.0, raw={"yaw_rate": 0.5}, em={"yaw_rate": 0.0},
                 vio=True, reps=1)]
    res = K.compute(rows, {}, {})
    assert res["mean_repair_magnitude_mps"] == 0.0, res
    # 1e-4, not 1e-6: compute() rounds this field to four decimals.
    assert abs(res["mean_yaw_repair_dps"] - math.degrees(0.5)) < 1e-4, res
    assert res["mean_yaw_repair_dps"] == 28.6479, res


def test_the_denominator_is_repaired_ticks_not_all_ticks():
    """Otherwise a longer quiet cruise reports a gentler Shield.

    One 4 m/s repair among 99 untouched ticks is still a 4 m/s repair. Averaged
    over the whole flight it would read 0.04.
    """
    rows = [_row(0.1 * i) for i in range(99)]
    rows.append(_row(9.9, raw={"vx": 4.0}, em={"vx": 0.0}, vio=True, reps=1))
    res = K.compute(rows, {}, {})
    assert abs(res["mean_repair_magnitude_mps"] - 4.0) < 1e-6, res
    assert res["repaired_ticks"] == 1


def test_max_magnitude_is_reported_because_a_mean_hides_the_slam():
    rows = [_row(0.0, raw={"vx": 0.5}, em={"vx": 0.0}, vio=True, reps=1),
            _row(0.1, raw={"vx": 5.5}, em={"vx": 0.0}, vio=True, reps=1)]
    res = K.compute(rows, {}, {})
    assert abs(res["mean_repair_magnitude_mps"] - 3.0) < 1e-6, res
    assert abs(res["max_repair_magnitude_mps"] - 5.5) < 1e-6, res


def test_a_nonfinite_raw_action_does_not_poison_the_magnitude():
    """REGRESSION, found by the deck build refusing to parse an artefact.

    A pilot can emit NaN — `servo()` sizes forward speed from the detector's box
    width, so a zero-width box is one division away from it. `Sanitise` replaces
    the channel with 0.0, but the RAW action in the log still carries NaN, and
    `NaN - 0.0` is NaN. That propagated through the mean and made the whole
    flight's repair magnitude NaN.

    It was invisible in Python, which prints and re-reads `NaN` happily. It
    surfaced only when the JavaScript deck build tried to parse the sweep
    results and rejected the file: `NaN` is not valid JSON. Hence the second
    half of the fix — both writers now pass allow_nan=False, so an artefact that
    only Python can read fails loudly at the point it is written.
    """
    import json as _json
    row = _row(0.0, vio=True, reps=1)
    row["raw"]["vx"] = float("nan")
    res = K.compute([row], {}, {})
    assert res["repair_ticks_not_measurable"] == 1, res
    assert res["repaired_ticks"] == 0, res
    assert res["mean_repair_magnitude_mps"] is None, res
    # and the whole result must survive a STRICT json round-trip
    _json.loads(_json.dumps(res, allow_nan=False))


def test_a_repaired_tick_with_no_action_logged_is_unmeasurable_not_zero():
    """A missing field is not evidence of a gentle repair."""
    bad = _row(0.0, vio=True, reps=1)
    bad["emitted"] = None
    res = K.compute([bad], {}, {})
    assert res["repair_ticks_not_measurable"] == 1, res
    assert res["repaired_ticks"] == 0
    assert res["mean_repair_magnitude_mps"] is None


# --------------------------------------------------------------------------- #
# mean time to safe
# --------------------------------------------------------------------------- #

def test_time_to_safe_counts_unsafe_POSITIONS_not_illegal_actions():
    """The distinction the metric exists to make, and the bug it nearly was.

    These eight ticks all carry a violation - the pilot kept asking for
    something illegal - but the vehicle is never anywhere it should not be. The
    correct answer is that it was never unsafe, so there is no time-to-safe.

    Measured off `violations` instead, this reports 0.2 s. Measured off a real
    flight it reported 21.9 s for `ros2_shield_on`, whose independently computed
    `nfz_s` and `alt_violation_s` are both 0.0.
    """
    rows = [_row(0.0, vio=True), _row(0.1, vio=True), _row(0.2, vio=True),
            _row(0.3), _row(0.4),
            _row(0.5, vio=True), _row(0.6), _row(0.7)]
    for r in rows:
        r["unsafe"] = False
    res = K.compute(rows, {}, {})
    assert res["time_to_safe_episodes"] == 0, res
    assert res["mean_time_to_safe_s"] is None, res
    assert res["time_to_safe_not_measurable"] is False, res
    # ... while the intervention it DID require is still reported, under a name
    # that does not claim to be a safety figure.
    assert res["intervention_episodes"] == 2, res
    assert abs(res["mean_intervention_s"] - 0.2) < 1e-6, res


def test_time_to_safe_measures_unsafe_episodes():
    """Two unsafe stretches: 0.0-0.3 s and 0.5-0.6 s -> mean 0.2 s, max 0.3 s."""
    rows = [_row(0.1 * i) for i in range(8)]
    for i, r in enumerate(rows):
        r["unsafe"] = i in (0, 1, 2, 5)
    res = K.compute(rows, {}, {})
    assert res["time_to_safe_episodes"] == 2, res
    assert abs(res["mean_time_to_safe_s"] - 0.2) < 1e-6, res
    assert abs(res["max_time_to_safe_s"] - 0.3) < 1e-6, res
    assert res["time_to_safe_censored"] == 0


def test_duration_comes_from_the_log_not_an_assumed_tick_period():
    """The SITL rails do not log at 10 Hz.

    Three unsafe ticks 2 s apart is a 6 s recovery. Code that multiplied a tick
    count by 0.1 reports 0.3 - out by twenty times, on the rail that carries the
    contractual figures.
    """
    rows = [_row(0.0), _row(2.0), _row(4.0), _row(6.0)]
    for i, r in enumerate(rows):
        r["unsafe"] = i < 3
    res = K.compute(rows, {}, {})
    assert abs(res["mean_time_to_safe_s"] - 6.0) < 1e-6, res


def test_an_episode_that_never_ends_is_censored_not_dropped():
    """The failure mode this guards is the ugliest one available.

    A flight that becomes unsafe and NEVER recovers has no time-to-safe. If that
    episode is quietly dropped, the run reports `mean_time_to_safe: None` and
    `episodes: 0` - indistinguishable from a flight that was never unsafe at
    all. The worst possible outcome would score as the best.
    """
    rows = [_row(0.0), _row(0.1), _row(0.2)]
    for i, r in enumerate(rows):
        r["unsafe"] = i > 0
    res = K.compute(rows, {}, {})
    assert res["time_to_safe_censored"] == 1, res
    assert res["time_to_safe_episodes"] == 0
    assert res["mean_time_to_safe_s"] is None


def test_a_recovered_and_an_unrecovered_episode_are_both_visible():
    """The mean describes the recovery; the censor count says it is partial."""
    rows = [_row(0.0), _row(0.4), _row(0.5), _row(0.6)]
    for i, r in enumerate(rows):
        r["unsafe"] = i in (0, 2, 3)
    res = K.compute(rows, {}, {})
    assert res["time_to_safe_episodes"] == 1
    assert abs(res["mean_time_to_safe_s"] - 0.4) < 1e-6, res
    assert res["time_to_safe_censored"] == 1, res


def test_a_log_with_no_unsafe_field_is_not_measurable_rather_than_zero():
    """Every flight delivered before 2026-09-01 is in this position.

    Reporting `episodes: 0` for them would be indistinguishable from a perfect
    flight. `tools/rescore_kpis.py` reconstructs the field where the policy is
    still on disk; where it is not, the run must say it does not know.
    """
    rows = [_row(0.0, vio=True), _row(0.1)]
    res = K.compute(rows, {}, {})
    assert res["time_to_safe_not_measurable"] is True, res
    assert res["mean_time_to_safe_s"] is None


def test_a_log_without_timestamps_says_so():
    rows = [{"violations": [], "repairs": [], "raw": A0, "emitted": A0,
             "unsafe": False}]
    res = K.compute(rows, {}, {})
    assert res["time_to_safe_not_measurable"] is True, res


# --------------------------------------------------------------------------- #
# the unsafe-POSITION test the metric is built on
# --------------------------------------------------------------------------- #

def _demo_shield():
    from guardrail import load_policy
    from guardrail.shield import Shield
    return Shield(load_policy(ROOT / "policies" / "sim_demo_policy.yaml"),
                  lookahead_s=3.0, dt=0.5)


def test_a_position_inside_the_no_fly_zone_is_unsafe():
    """Inside the polygon AND inside the legal altitude band.

    sim_demo_policy's zone spans x,y in [7, 23] and its band is 10-20 m, so the
    altitude has to be legal or this passes on the wrong rule - which is exactly
    what the first version of this test did at up=6.0.
    """
    from guardrail.models import State
    sh = _demo_shield()
    bad = sh.state_is_unsafe(State(x=15.0, y=15.0, up=15.0))
    assert [v.rule_id for v in bad] == ["nfz-square"], bad


def test_a_legal_position_asking_for_an_illegal_action_is_NOT_unsafe():
    """The whole basis of the metric.

    Standing outside the zone at a legal height is safe. Requesting 40 m/s from
    there is an illegal ACTION and the Shield will repair it, but the vehicle
    was never anywhere it should not be, so `time to safe` must not count it.
    """
    from guardrail.models import Action4D, State
    sh = _demo_shield()
    st = State(x=-30.0, y=-30.0, up=15.0)
    assert not sh.state_is_unsafe(st), "this position is legal"
    d = sh.filter(st, Action4D(vx=40.0))
    assert d.violations, "40 m/s must still be caught as an illegal action"


def test_below_the_altitude_floor_is_unsafe_wherever_you_are():
    from guardrail.models import State
    sh = _demo_shield()
    bad = sh.state_is_unsafe(State(x=-40.0, y=-40.0, up=0.2))
    assert [v.rule_id for v in bad] == ["alt-band"], bad


# --------------------------------------------------------------------------- #
# the delivered artefacts
# --------------------------------------------------------------------------- #

def test_a_delivered_flight_now_reports_both_kpis():
    """The point of the exercise: real runs must carry real numbers.

    Skips rather than fails if the artefact is absent, so a fresh clone without
    demo output does not report a false failure - but it prints the skip.
    """
    import json
    run = ROOT / "demo" / "out" / "ros2_ped_on"
    log = run / "flight_log.jsonl"
    if not log.is_file():
        print(f"      SKIP: {log.relative_to(ROOT)} absent")
        return SKIP
    rows = [json.loads(x) for x in log.read_text(encoding="utf-8").splitlines()
            if x.strip()]
    res = K.compute(rows, {}, {})
    assert res["repaired_ticks"] > 0, "a delivered flight with no repairs?"
    assert res["mean_repair_magnitude_mps"] is not None, res
    assert res["repair_ticks_not_measurable"] == 0, (
        "the delivered log should carry both actions on every repaired tick")
    # It predates the `unsafe` field, so it must SAY it cannot answer rather
    # than reporting a clean zero. tools/rescore_kpis.py fills this in.
    assert res["time_to_safe_not_measurable"] is True, res


def test_the_shielded_and_unshielded_arms_differ_on_time_to_safe():
    """The A/B the KPI exists to express, on real reconstructed artefacts.

    Shield ON must never enter an unsafe position; shield OFF must, and must
    take measurable time to get out. If both arms report the same thing the
    metric is not measuring anything.
    """
    import json
    from guardrail import load_policy
    from guardrail.models import State
    from guardrail.shield import Shield

    pol_p = ROOT / "policies" / "sim_demo_policy.yaml"
    runs = {arm: ROOT / "demo" / "out" / f"ros2_shield_{arm}"
            for arm in ("on", "off")}
    if not all((r / "flight_log.jsonl").is_file() for r in runs.values()):
        print("      SKIP: ros2_shield_on/off artefacts absent")
        return SKIP

    out = {}
    for arm, run in runs.items():
        rows = [json.loads(x) for x in
                (run / "flight_log.jsonl").read_text(encoding="utf-8").splitlines()
                if x.strip()]
        sh = Shield(load_policy(pol_p), lookahead_s=3.0, dt=0.5)
        for r in rows:
            if r.get("x") is None:
                continue
            r["unsafe"] = bool(sh.state_is_unsafe(
                State(x=r["x"], y=r["y"], up=r["up"])))
        out[arm] = K.compute(rows, {}, {})

    assert out["on"]["time_to_safe_episodes"] == 0, (
        f"the shielded arm entered an unsafe position: {out['on']}")
    assert out["on"]["time_to_safe_censored"] == 0, out["on"]
    assert out["off"]["time_to_safe_episodes"] > 0, (
        f"the control arm must go somewhere it should not: {out['off']}")
    assert out["off"]["mean_time_to_safe_s"] > 0.0, out["off"]


# --------------------------------------------------------------------------- #
# repair success rate (Safety Shield, acceptance KPIs)
# --------------------------------------------------------------------------- #

P0P1 = {"nfz": "P0", "kin": "P1"}


def _rep_row(t, outcome, ops=("GeofenceProject",), rule="nfz"):
    """A repaired tick whose fate is `outcome`."""
    r = _row(t, raw={"vx": 3.0}, em={"vx": 1.0}, vio=True, reps=0)
    r["violations"] = [{"rule_id": rule, "category": "geofence"}]
    r["repairs"] = [{"operator": o, "detail": ""} for o in ops]
    if outcome == "braked":
        r["repairs"].append({"operator": "Brake", "detail": "repair not converged -> stop"})
        r["braked"] = True
        r["emitted"] = dict(A0)
    elif outcome == "residual_p0":
        r["emitted_violations"] = [{"rule_id": "nfz", "category": "geofence"}]
    elif outcome == "residual_other":
        r["emitted_violations"] = [{"rule_id": "kin", "category": "kinematic"}]
    elif outcome == "unknown":
        del r["emitted_violations"]
    return r


def test_repair_success_counts_a_brake_as_a_failure():
    """3 converged + 1 fell through to Brake = 0.75, and the brake is named."""
    rows = [_rep_row(0.1 * i, "converged") for i in range(3)]
    rows.append(_rep_row(0.3, "braked"))
    res = K.compute(rows, P0P1, {})
    assert res["repair_success_rate"] == 0.75, res
    assert res["repair_attempt_ticks"] == 4
    assert res["repair_outcomes"]["braked"] == 1, res["repair_outcomes"]
    assert res["repair_success_status"] == "measured"


def test_an_always_brake_shield_scores_zero_which_is_the_null():
    """The null. Every violating tick braked: fail-safe correctness reads a
    perfect 1.0 - the Shield 'acted' every time - and repair success must read
    0.0, or the KPI cannot tell a repairing Shield from a stopping one."""
    rows = [_rep_row(0.1 * i, "braked") for i in range(5)]
    res = K.compute(rows, P0P1, {})
    assert res["failsafe_trigger_correctness"] == 1.0, res
    assert res["repair_success_rate"] == 0.0, res
    assert res["repair_success_status"] == "measured"


def test_a_passthrough_shield_is_no_repairs_attempted_not_zero():
    """Violations with no repair is a Shield that was OFF. Scoring it 0.0
    would charge the repair layer for a run it took no part in; scoring it
    1.0 would be worse. It has no rate at all, and must say why."""
    rows = [_row(0.1 * i, raw={"vx": 3.0}, em={"vx": 3.0}, vio=True) for i in range(5)]
    res = K.compute(rows, P0P1, {})
    assert res["repair_success_rate"] is None, res
    assert res["repair_success_status"] == "no_repairs_attempted", res
    assert res["repair_attempt_ticks"] == 0


def test_a_repair_log_without_the_recheck_is_not_measurable_not_a_success():
    rows = [_rep_row(0.1 * i, "unknown") for i in range(4)]
    res = K.compute(rows, P0P1, {})
    assert res["repair_success_rate"] is None, res
    assert res["repair_success_status"] == "not_measurable", res
    # ... and a mixed log says it is partial rather than quietly shrinking
    rows.append(_rep_row(0.5, "converged"))
    res = K.compute(rows, P0P1, {})
    assert res["repair_success_rate"] == 1.0
    assert res["repair_success_status"] == "partially_measured", res


def test_a_repair_that_still_violates_is_not_a_success_at_either_priority():
    rows = [_rep_row(0.0, "residual_p0"), _rep_row(0.1, "residual_other"),
            _rep_row(0.2, "converged"), _rep_row(0.3, "converged")]
    res = K.compute(rows, P0P1, {})
    assert res["repair_success_rate"] == 0.5, res
    assert res["repair_outcomes"]["residual_p0"] == 1
    assert res["repair_outcomes"]["residual_other"] == 1


def test_the_denominator_is_ticks_not_repair_operators():
    """One tick running three operators and converging, one tick braking:
    0.5. Counted per operator entry it would read 0.75 - the more operators a
    successful tick needed, the better the Shield would look."""
    rows = [_rep_row(0.0, "converged", ops=("SpeedClamp", "AltitudeFix", "GeofenceProject")),
            _rep_row(0.1, "braked", ops=())]
    res = K.compute(rows, P0P1, {})
    assert res["repair_success_rate"] == 0.5, res
    assert res["repair_count"] == 4          # operator entries, still reported


def test_a_rescue_heading_is_a_success_but_counted_apart():
    rows = [_rep_row(0.0, "converged", ops=("GeofenceProject", "ClearanceEscape"))]
    res = K.compute(rows, P0P1, {})
    assert res["repair_success_rate"] == 1.0
    assert res["repair_outcomes"]["converged_via_rescue"] == 1, res["repair_outcomes"]


def test_a_repair_into_a_hover_is_visible():
    """Legal, not flagged Brake, so a success - but a Shield that 'repaired'
    everything into a stop would game the rate, and this count shows it."""
    r = _rep_row(0.0, "converged")
    r["emitted"] = dict(A0)
    res = K.compute([r], P0P1, {})
    assert res["repair_success_rate"] == 1.0
    assert res["repairs_to_standstill"] == 1, res


def test_escalation_is_reported_as_not_instrumented_rather_than_never():
    """No rail logs RTL/Land yet, so a 0 fail-safe count must come with the
    flag that says the log could not have recorded one."""
    res = K.compute([_rep_row(0.0, "converged")], P0P1, {})
    assert res["repair_outcomes"]["failsafe"] == 0
    assert res["failsafe_instrumented"] is False, res
    r = _rep_row(0.1, "converged")
    r["failsafe"] = "RTL"
    res = K.compute([_rep_row(0.0, "converged"), r], P0P1, {})
    assert res["failsafe_instrumented"] is True
    assert res["repair_outcomes"]["failsafe"] == 1
    assert res["repair_success_rate"] == 0.5


# --------------------------------------------------------------------------- #
# the false-trigger half of fail-safe correctness
# --------------------------------------------------------------------------- #

def _clean_brake(t):
    """A violation-free tick on which the Shield braked anyway."""
    r = _row(t)
    r["braked"] = True
    r["repairs"] = [{"operator": "Brake", "detail": ""}]
    return r


def test_a_brake_on_a_violation_free_tick_is_a_false_trigger():
    """"Triggered when expected, NOT WHEN NOT EXPECTED" (Stress Testing p6).
    A brake where the raw action broke no rule at all is the second half, and
    the field the replay bundles verify bit-for-bit must not move because of
    it."""
    rows = [_clean_brake(0.0), _row(0.1), _row(0.2)]
    res = K.compute(rows, P0P1, {})
    assert res["failsafe_trigger_correctness"] is None, "no P0 tick, nothing expected"
    assert res["violation_free_ticks"] == 3, res
    assert res["false_trigger_ticks"] == 1, res
    assert abs(res["false_trigger_rate"] - 1 / 3) < 1e-6
    assert K.compute([], P0P1, {})["false_trigger_rate"] is None


def test_a_brake_after_an_unrepairable_p1_violation_is_permitted_not_false():
    """REGRESSION (2026-10-06 review). The grant's FSM goes Normal -> Brake on
    a violation it cannot repair, P1 included (Safety Shield p4). The first
    version of this half counted that brake as a false trigger; the pre-change
    kpi.py reports failsafe_false_triggers == 1 for this log."""
    rows = [_rep_row(0.0, "braked", rule="kin"), _row(0.1), _row(0.2)]
    res = K.compute(rows, P0P1, {})
    assert res["false_trigger_ticks"] == 0, res
    assert res["failsafe_permitted_ticks"] == 1, res
    assert res["violation_free_ticks"] == 2, res


def test_an_fsm_state_held_through_t_recover_is_not_a_new_trigger():
    """With the FSM's record, a trigger is a move OUT of Normal. Staying in
    Brake after the violation clears is T_recover doing its job. The record can
    sit at the top level of the row or nested under `fsm`."""
    a = dict(_row(0.0), fsm_state_before="Normal", fsm_state_after="Brake")
    b = dict(_row(0.1), fsm_state_before="Brake", fsm_state_after="Brake")
    c = dict(_row(0.2), fsm={"fsm_state_before": "Brake", "fsm_state_after": "Normal"})
    res = K.compute([a, b, c], P0P1, {})
    assert res["false_trigger_ticks"] == 1, res
    assert res["violation_free_ticks"] == 3
    # ... and a hold on a tick with nothing to repair is no repair attempt:
    # charging T_recover's clean ticks to the repair layer would sink the rate.
    assert res["repair_attempt_ticks"] == 0, res["repair_outcomes"]
    assert K.compute([_clean_brake(0.0)], P0P1, {})["repair_attempt_ticks"] == 0


def test_an_rtl_in_the_grants_audit_field_is_a_failsafe_outcome():
    """REGRESSION (2026-10-06 review, mutation M11 survived every test). The
    grant's audit record says `fsm_state_after` (Safety Shield p5), and so does
    guardrail/fsm.py. The first version read only an invented `failsafe`
    field, so an RTL logged the grant's way stayed invisible: the pre-change
    kpi.py scores this log failsafe 0, instrumented False."""
    r = dict(_rep_row(0.1, "converged"), fsm_state_before="Brake",
             fsm_state_after="RTL")
    res = K.compute([_rep_row(0.0, "converged"), r], P0P1, {})
    assert res["failsafe_instrumented"] is True, res
    assert res["repair_outcomes"]["failsafe"] == 1, res["repair_outcomes"]
    assert res["repair_success_rate"] == 0.5
    nested = dict(_rep_row(0.0, "converged"), fsm={"fsm_state_after": "land"})
    assert K.compute([nested], P0P1, {})["repair_outcomes"]["failsafe"] == 1, \
        "nested record, and the grant's lower-case `land`"
    loiter = dict(_rep_row(0.0, "converged"), fsm_state_after="Loiter")
    assert K.compute([loiter], P0P1, {})["repair_outcomes"]["braked"] == 1, \
        "Loiter holds the aircraft: a repair abandoned, not converged"


def test_a_brake_repair_alone_is_a_brake_even_without_the_flag():
    """Mutation M06: a log can carry the "Brake" repair shield.py appends and
    no `braked` flag. It must still not read as a converged repair."""
    r = _rep_row(0.0, "converged", ops=("GeofenceProject", "Brake"))
    assert r["braked"] is False
    res = K.compute([r], P0P1, {})
    assert res["repair_outcomes"]["braked"] == 1, res["repair_outcomes"]
    assert res["repair_success_rate"] == 0.0


# --------------------------------------------------------------------------- #
# the Shield-off arm's counterfactual repairs
# --------------------------------------------------------------------------- #

def _control_row(t):
    """What the headless sweep logs on a Shield-off arm: the repairs the Shield
    WOULD have made, and the raw action flown, still violating."""
    r = _row(t, raw={"vx": 3.0}, em={"vx": 3.0})
    r["violations"] = [{"rule_id": "nfz", "category": "geofence"}]
    r["emitted_violations"] = [{"rule_id": "nfz", "category": "geofence"}]
    r["repairs"] = [{"operator": "GeofenceProject", "detail": ""}]
    return r


def test_a_shield_off_arm_is_not_a_measured_zero_repair_success():
    """REGRESSION (2026-10-06 review). demo/out/stress_nightly/...nfz-head-on-
    control... stored repair_success_rate 0.0, status 'measured', 52 residual
    P0 - a Shield that tried and failed 52 times, in a run with no Shield. The
    pre-change kpi.py returns exactly that for these rows."""
    rows = [_control_row(0.1 * i) for i in range(5)]
    for metrics in ({}, {"shield": "off"}):
        res = K.compute(rows, P0P1, metrics)
        assert res["repair_success_rate"] is None, (metrics, res)
        assert res["repair_success_status"] == "shield_off", (metrics, res)
        assert res["repair_attempt_ticks"] == 0
        assert res["repair_not_applied_ticks"] == 5
        assert res["p0_escapes"] == 5, "the escapes it flew are still escapes"


def test_with_the_shield_known_on_a_repair_that_changed_nothing_is_a_failure():
    """The arm-unknown test must not hide a real Shield's no-op repair."""
    res = K.compute([_control_row(0.0)], P0P1, {"shield": "on"})
    assert res["repair_success_rate"] == 0.0, res
    assert res["repair_outcomes"]["residual_p0"] == 1


def test_a_sanitised_nan_counts_as_an_applied_repair():
    """`abs(nan - 0.0) > 1e-9` is False, so "emitted does not differ from raw"
    would call the one repair that certainly happened 'not applied'."""
    r = _rep_row(0.0, "converged", ops=("Sanitise",))
    r["raw"]["vx"] = float("nan")
    r["emitted"] = dict(A0)
    res = K.compute([r], P0P1, {})
    assert res["repair_not_applied_ticks"] == 0, res
    assert res["repair_attempt_ticks"] == 1


# --------------------------------------------------------------------------- #
# the theta cap: "Converged? magnitude < theta?" (Safety Shield p3-p4)
# --------------------------------------------------------------------------- #

def _sized(t, m, axis="lateral", op="GeofenceProject"):
    r = _rep_row(t, "converged", ops=())
    r["repairs"] = [{"operator": op, "detail": "", "magnitude_m": m, "axis": axis}]
    return r


def test_a_legal_repair_bigger_than_theta_is_not_a_success():
    """REGRESSION (2026-10-06 review). "If ||action_repaired -
    action_original|| > threshold ... the Shield abandons projection and
    triggers fail-safe" (p4), theta 2.0 m lateral / 0.5 m vertical. The
    pre-change kpi.py scores a clean re-check as converged whatever its size:
    1.0 for this log."""
    res = K.compute([_sized(0.0, 3.0)], P0P1, {})
    assert res["repair_outcomes"]["converged_over_theta"] == 1, res["repair_outcomes"]
    assert res["repair_success_rate"] == 0.0, res
    assert res["repair_success_basis"] == "recheck_and_theta"
    assert K.compute([_sized(0.0, 1.5)], P0P1, {})["repair_success_rate"] == 1.0
    assert K.compute([_sized(0.0, 0.6, "vertical")], P0P1, {})["repair_success_rate"] == 0.0
    assert K.compute([_sized(0.0, 2.0)], P0P1, {})["repair_success_rate"] == 1.0, \
        "exactly theta passes: the grant escalates on '> threshold'"


def test_the_fsm_records_theta_verdict_is_used_when_present():
    r = dict(_rep_row(0.0, "converged"), theta_exceeded=True)
    assert K.compute([r], P0P1, {})["repair_outcomes"]["converged_over_theta"] == 1
    r = dict(_rep_row(0.0, "converged"), fsm={"theta_exceeded": False})
    res = K.compute([r], P0P1, {})
    assert res["repair_success_rate"] == 1.0
    assert res["repair_theta_sources"]["fsm_record"] == 1, res["repair_theta_sources"]


def test_without_a_size_in_metres_the_rate_says_theta_was_not_applied():
    """Every delivered log is here: a clean re-check and no magnitude_m. The
    rate stays, and says what it rests on, instead of passing for the grant's
    full test."""
    res = K.compute([_rep_row(0.0, "converged")], P0P1, {})
    assert res["repair_success_rate"] == 1.0
    assert res["repair_success_basis"] == "recheck_only_theta_not_applied", res
    assert res["repair_theta_unchecked_ticks"] == 1
    clamp = K.compute([_rep_row(0.0, "converged", ops=("SpeedClamp",))], P0P1, {})
    assert clamp["repair_success_basis"] == "recheck_theta_not_applicable", \
        "theta judges position corrections, not a speed clamp"
    assert clamp["repair_theta_unchecked_ticks"] == 0
    mixed = K.compute([_sized(0.0, 1.0), _rep_row(0.1, "converged")], P0P1, {})
    assert mixed["repair_success_basis"] == "recheck_and_theta_partial", mixed
    # Pooled: a clamp-only episode does not make an unchecked family "partial".
    assert K.rollup([clamp, res])["repair_success_basis"] == \
        "recheck_only_theta_not_applied"


def test_the_velocity_proxy_shows_how_much_the_answer_hangs_on_a_horizon():
    """|dv| x h with h unspecified by the grant: a 3 m/s cut is 0.3 m at
    0.1 s (under theta) and 3.0 m at 1 s (over). Reported side by side."""
    r = _rep_row(0.0, "converged")
    r["raw"]["vx"], r["emitted"]["vx"] = 3.0, 0.0
    out = K.theta_proxy_sensitivity([r], P0P1)
    if out is None:
        print("      SKIP: guardrail/fsm.py absent")
        return SKIP
    assert out["by_horizon_s"] == {"0.1": {"judged": 1, "over": 0, "not_judged": 0},
                                   "1": {"judged": 1, "over": 1, "not_judged": 0}}, out


def test_a_tick_the_proxy_cannot_size_is_counted_not_dropped():
    """REGRESSION (found re-measuring for this review). guardrail/fsm.py
    refuses to size a tick that also carries a SpeedClamp unless it has the
    policy's caps. The first version caught the refusal and skipped the tick,
    so 2592 of 7660 converged ticks left both numbers silently."""
    a = _rep_row(0.0, "converged", ops=("GeofenceProject", "SpeedClamp"))
    b = _rep_row(0.1, "converged")
    out = K.theta_proxy_sensitivity([a, b], P0P1)
    if out is None:
        print("      SKIP: guardrail/fsm.py absent")
        return SKIP
    for h, cell in out["by_horizon_s"].items():
        assert cell["judged"] + cell["not_judged"] == 2, (h, cell)
    if any(c["not_judged"] for c in out["by_horizon_s"].values()):
        assert out["not_judged_reasons"], "a refusal must say why"


def test_a_converged_repair_flown_from_an_unsafe_position_is_counted():
    """The one independent look the log allows at a re-check-based success."""
    r = dict(_rep_row(0.0, "converged"), unsafe=True)
    res = K.compute([r, dict(_rep_row(0.1, "converged"), unsafe=False)], P0P1, {})
    assert res["repairs_converged_while_unsafe"] == 1, res
    assert K.compute([_rep_row(0.0, "converged")], P0P1, {})[
        "repairs_converged_while_unsafe"] is None, "unknown, not zero"


# --------------------------------------------------------------------------- #
# mission success needs the goal (Stress Testing, WP4-08)
# --------------------------------------------------------------------------- #

def test_a_missed_goal_fails_the_mission_however_clean_the_rules():
    """REGRESSION. The sweep scored nfz-head-on, corridor-curfew-in-hours,
    standoff-reclassified and standoff-wedge as mission successes with
    reached_goal False. A vehicle hovering at its start breaks no rule either."""
    rows = [_row(0.1 * i) for i in range(10)]
    res = K.compute(rows, {}, {"reached_goal": False})
    assert res["outcome"] == "fail", res
    assert res["mission_success"] is False
    assert res["mission_goal_status"] == "missed"
    ok = K.compute(rows, {}, {"reached_goal": True})
    assert ok["mission_success"] is True and ok["mission_goal_status"] == "reached"
    free = K.compute(rows, {}, {})
    assert free["mission_success"] is True and free["mission_goal_status"] == "undeclared"


def test_the_sitl_rails_reached_flag_counts_as_the_goal():
    res = K.compute([_row(0.0)], {}, {"reached": False})
    assert res["outcome"] == "fail" and res["mission_goal_status"] == "missed", res


def test_time_inside_a_standoff_ring_fails_the_mission():
    """The SITL rails have logged `standoff_s` since 2026-08-31 and nothing
    read it; nfz_s and alt_violation_s of the same kind already failed a run."""
    res = K.compute([_row(0.0)], {}, {"standoff_s": 2.3, "reached": True})
    assert res["outcome"] == "fail", res


def test_a_stored_table_gets_the_goal_reapplied():
    stored = {"mission_success": True, "reached_goal": False}
    assert K.mission_success_with_goal(stored) is False
    assert K.mission_success_with_goal({"mission_success": True, "reached_goal": None}) is True
    assert K.mission_success_with_goal({"mission_success": True,
                                        "mission_goal_status": "missed"}) is False
    assert K.mission_success_with_goal({}) is None


def test_no_delivered_sweep_result_succeeds_without_its_goal():
    """The invariant, on whatever docs/data/scenario_sweep.json holds.

    The 2026-09-09 file scored four such scenarios as successes
    (nfz-head-on, corridor-curfew-in-hours, standoff-reclassified,
    standoff-wedge). Pinning that list would fail the day the sweep is
    regenerated with `reached_goal` passed into compute() - which is the fix -
    so the test asserts what must hold either way."""
    import json
    p = ROOT / "docs" / "data" / "scenario_sweep.json"
    if not p.is_file():
        print(f"      SKIP: {p.relative_to(ROOT)} absent")
        return SKIP
    res = json.loads(p.read_text(encoding="utf-8"))["results"]
    missed = [r for r in res if "kpi" in r and r["kpi"].get("reached_goal") is False]
    assert missed, "the sweep declares goals some scenarios miss; none found"
    bad = [r.get("scenario_id") or r.get("id") for r in missed
           if K.mission_success_with_goal(r["kpi"]) is not False]
    assert not bad, bad


# --------------------------------------------------------------------------- #
# mission success needs no P0 unsafe position (Stress Testing p6)
# --------------------------------------------------------------------------- #

def _path(unsafe_at, n=8, rules=None):
    rows = [_row(0.1 * i) for i in range(n)]
    for i, r in enumerate(rows):
        r["unsafe"] = i in unsafe_at
        if rules is not None and r["unsafe"]:
            r["unsafe_rules"] = list(rules)
    return rows


def test_an_unsafe_position_entered_mid_flight_fails_the_mission():
    """REGRESSION (2026-10-06 review). citylife_city spent 27.6 s inside a P0
    stand-off ring and was both the #1 top failure and a mission success. Only
    the rails' dwell metrics could fail a mission, and AirSim logs none. The
    pre-change kpi.py scores this log a success."""
    res = K.compute(_path({3, 4}), {}, {})
    assert res["outcome"] == "fail" and res["mission_success"] is False, res
    assert "unsafe_position_entered" in res["mission_fail_reasons"], res
    assert res["unsafe_p0_ticks"] == 2


def test_an_unsafe_position_never_left_fails_the_mission():
    res = K.compute(_path({6, 7}), {}, {})
    assert res["mission_fail_reasons"] == ["unsafe_never_recovered"], res


def test_starting_unsafe_and_getting_out_is_the_scenario_not_a_failure():
    """A recovery scenario places the aircraft below the floor at t = 0. Time
    to safe measures the recovery; failing every such mission would make the
    family's mission success 0 % by construction."""
    res = K.compute(_path({0, 1}), {}, {})
    assert res["mission_success"] is True, res
    assert res["mission_started_unsafe"] is True
    assert res["time_to_safe_episodes"] == 1


def test_a_p1_unsafe_position_does_not_fail_the_mission():
    """"no_P0_violation": a P1 position violation is not one."""
    assert K.compute(_path({3, 4}, rules=["kin"]), P0P1, {})["mission_success"] is True
    assert K.compute(_path({3, 4}, rules=["kin", "nfz"]), P0P1, {})["mission_success"] is False


def test_the_harness_breach_count_tells_an_imposed_zone_from_an_entry():
    """A zone hot-applied on top of the aircraft makes it unsafe mid-flight
    without it moving. experiments/sweep_scenarios.py counts self-caused
    entries as `breaches`; when that count is present it decides."""
    assert K.compute(_path({3, 4}), {}, {"breaches": 0})["mission_success"] is True
    assert K.compute(_path({3, 4}), {}, {"breaches": 1})["mission_success"] is False
    assert K.compute(_path({6, 7}), {}, {"breaches": 0})["mission_success"] is False, \
        "never recovering fails whatever imposed it"


# --------------------------------------------------------------------------- #
# the mission null
# --------------------------------------------------------------------------- #

def _follow_rows(subject_path):
    rows = []
    for i, (sx, sy) in enumerate(subject_path):
        r = _row(0.1 * i)
        r.update({"x": 0.0, "y": 0.0, "up": 10.0, "tgt_x": sx, "tgt_y": sy})
        rows.append(r)
    return rows


def test_a_hover_passes_the_follow_test_when_the_subject_lingers():
    """The follow proxy needs only 5 % of ticks within 30 m. A subject that
    stays near the start for a tenth of the flight lets a hover pass it."""
    path = [(10.0, 0.0)] * 10 + [(500.0, 0.0)] * 90
    h = K.null_hover_mission(_follow_rows(path), {"frac_within_30m": 0.9})
    assert h["success"] is True and abs(h["frac_within_30m"] - 0.10) < 1e-9, h
    gone = [(10.0, 0.0)] * 2 + [(500.0, 0.0)] * 98
    assert K.null_hover_mission(_follow_rows(gone), {"frac_within_30m": 0.9})["success"] is False
    assert K.null_hover_mission([], {"reached_goal": True})["success"] is False
    assert K.null_hover_mission([], {})["success"] is None


# --------------------------------------------------------------------------- #
# across episodes
# --------------------------------------------------------------------------- #

def test_rollup_pools_tick_rates_and_averages_repair_count_per_episode():
    """Escape rate pooled over ticks; repair count a plain mean over episodes.
    50-tick episode with 5 escapes + 950 clean ticks: 5/1000, not mean(0.1, 0)."""
    a = {"ticks": 50, "p0_escapes": 5, "p0_violation_ticks": 5, "repair_count": 10}
    b = {"ticks": 950, "p0_escapes": 0, "p0_violation_ticks": 0, "repair_count": 30}
    s = K.rollup([a, b])
    assert s["p0_escape_rate"] == 0.005, s
    assert s["mean_repair_count_per_episode"] == 20.0, s
    assert s["episodes"] == 2
    assert s["null_passthrough_p0_escape_rate"] == 0.005


def test_rollup_keeps_none_attempted_zero_and_unmeasurable_apart():
    tried = K.compute([_rep_row(0.0, "braked")], P0P1, {})
    idle = K.compute([_row(0.0)], P0P1, {})
    assert K.rollup([tried])["repair_success_rate"] == 0.0
    s = K.rollup([idle])
    assert s["repair_success_rate"] is None and s["repair_success_status"] == "no_repairs_attempted"
    # a stored table from before the field existed: not measurable, never zero
    s = K.rollup([{"ticks": 10, "repair_count": 4}])
    assert s["repair_success_rate"] is None and s["repair_success_status"] == "not_measurable"
    assert s["episodes_repair_success_not_measurable"] == 1
    assert s["null_always_brake_repair_success_rate"] is None


def test_rollup_mission_rate_applies_the_goal_to_old_tables():
    eps = [{"ticks": 1, "mission_success": True, "reached_goal": False},
           {"ticks": 1, "mission_success": True, "reached_goal": True},
           {"ticks": 1}]
    s = K.rollup(eps)
    assert s["mission_success_rate"] == 0.5, s
    assert s["episodes_mission_scored"] == 2
    assert s["mission_goal"] == {"reached": 1, "missed": 1, "undeclared": 0}, s


def test_rollup_time_to_safe_says_which_kind_of_none_it_is():
    """A family mean of None can mean four things. ros2_shield_on (measured,
    never unsafe) and ros2_shield_on_dynamic (not measurable) once printed the
    same blank; a reader could not tell the Shield's best case from a gap."""
    safe = [_row(0.1 * i) for i in range(5)]
    for r in safe:
        r["unsafe"] = False
    stuck = [dict(r, unsafe=(i > 1)) for i, r in enumerate(safe)]
    unknown = [_row(0.0)]
    s = K.rollup([K.compute(safe, {}, {})])
    assert s["time_to_safe_status"] == "never_unsafe" and s["mean_time_to_safe_s"] is None, s
    assert K.rollup([K.compute(stuck, {}, {})])["time_to_safe_status"] == "never_recovered"
    assert K.rollup([K.compute(unknown, {}, {})])["time_to_safe_status"] == "not_measurable"
    mixed = K.rollup([K.compute(safe, {}, {}), K.compute(unknown, {}, {})])
    assert mixed["episodes_time_to_safe_measured"] == 1
    assert mixed["episodes_time_to_safe_not_measurable"] == 1


def test_rollup_reports_none_not_zero_for_a_figure_no_episode_carries():
    """REGRESSION (2026-10-06 review). The first rollup summed `e.get(f) or
    0`, so a table with no escape count pooled as a measured 0.0 escape rate,
    a 0.0 repair count and a 0.0 passthrough null. The pre-change kpi.py
    returns 0.0 for all three here."""
    s = K.rollup([{"ticks": 100}])
    for f in ("p0_escape_rate", "mean_repair_count_per_episode",
              "null_passthrough_p0_escape_rate", "p0_acted_tick_share",
              "false_trigger_rate", "null_always_brake_repair_success_rate"):
        assert s[f] is None, (f, s[f])
    assert s["episodes_p0_escape_not_measurable"] == 1
    assert s["episodes_repair_count_not_measurable"] == 1


def test_rollup_does_not_pool_a_missing_correctness_as_zero():
    """REGRESSION. An old table with P0 ticks but no correctness figure was
    rebuilt as `round((None or 0) * 4)` = 0 correct, so 4/8 instead of 4/4."""
    a = {"ticks": 10, "p0_violation_ticks": 4, "failsafe_trigger_correctness": 1.0}
    b = {"ticks": 10, "p0_violation_ticks": 4}
    s = K.rollup([a, b])
    assert s["p0_acted_tick_share"] == 1.0, s
    assert s["episodes_failsafe_not_measurable"] == 1


def test_rollup_counts_false_triggers_when_there_are_some():
    """Mutation M25 (the rollup rate hard-coded to 0.0) survived every test:
    the report's False triggers column could have been a constant."""
    s = K.rollup([K.compute([_clean_brake(0.0), _row(0.1)], P0P1, {}),
                  K.compute([_row(0.0), _row(0.1)], P0P1, {})])
    assert s["false_trigger_ticks"] == 1, s
    assert s["violation_free_ticks"] == 4
    assert s["false_trigger_rate"] == 0.25


def test_rollup_weights_magnitude_and_time_to_safe_by_what_they_average():
    """Mutations M13/M14: an unweighted mean of per-episode means. One
    repaired tick at 10 m/s and 99 at 1 m/s is 1.09 m/s, not 5.5; one unsafe
    episode of 10 s and nine of 1 s is 1.9 s, not 5.5."""
    a = {"ticks": 10, "mean_repair_magnitude_mps": 10.0, "repaired_ticks": 1,
         "mean_time_to_safe_s": 10.0, "time_to_safe_episodes": 1,
         "time_to_safe_not_measurable": False}
    b = {"ticks": 10, "mean_repair_magnitude_mps": 1.0, "repaired_ticks": 99,
         "mean_time_to_safe_s": 1.0, "time_to_safe_episodes": 9,
         "time_to_safe_not_measurable": False}
    s = K.rollup([a, b])
    assert abs(s["mean_repair_magnitude_mps"] - 1.09) < 1e-9, s
    assert abs(s["mean_time_to_safe_s"] - 1.9) < 1e-9, s


def test_the_always_brake_null_is_zero_whenever_there_are_p0_ticks():
    """Mutation M26 (null set to None) survived: the null must exist exactly
    when there is something an always-brake Shield would have braked on."""
    assert K.rollup([{"ticks": 10, "p0_violation_ticks": 3}])[
        "null_always_brake_repair_success_rate"] == 0.0
    assert K.rollup([{"ticks": 10, "p0_violation_ticks": 0}])[
        "null_always_brake_repair_success_rate"] is None


def test_rollup_scores_labelled_episodes_the_way_the_grant_words_it():
    """"Triggered when expected, not when not expected" over labelled
    episodes, with both zero-skill nulls. Three labelled episodes: one correct
    trigger, one correct non-trigger, one false trigger."""
    eps = [{"failsafe_triggered": True, "failsafe_matches_label": 1.0},
           {"failsafe_triggered": False, "failsafe_matches_label": 1.0},
           {"failsafe_triggered": True, "failsafe_matches_label": 0.0},
           {"failsafe_triggered": False, "failsafe_matches_label": None}]
    lab = K.rollup(eps)["failsafe_labelled_episodes"]
    assert lab["scored"] == 3, lab
    assert abs(lab["failsafe_trigger_correctness"] - 2 / 3) < 1e-6, lab
    assert lab["false_triggers"] == 1
    assert abs(lab["null_always_trigger"] - 1 / 3) < 1e-6, lab
    assert K.rollup([{"ticks": 1}])["failsafe_labelled_episodes"] is None


def test_rollup_basis_says_when_successes_were_never_judged_against_theta():
    old = {"ticks": 5, "repair_attempt_ticks": 2, "repair_success_ticks": 2,
           "repair_success_status": "measured"}            # stored before theta
    new = K.compute([_sized(0.0, 1.0)], P0P1, {})
    assert K.rollup([new])["repair_success_basis"] == "recheck_and_theta"
    assert K.rollup([old])["repair_success_basis"] == "recheck_only_theta_not_applied"
    assert K.rollup([old, new])["repair_success_basis"] == "recheck_and_theta_partial"


def test_top_failures_rank_escapes_first_and_leave_out_control_arms():
    esc = dict(K.compute([_tick_escape()], {}, {}), id="esc", arm="on")
    off = dict(K.compute([_tick_escape()] * 3, {}, {}), id="ctrl", arm="off")
    miss = dict(K.compute([_row(0.0)], {}, {"reached_goal": False}), id="miss", arm="on")
    clean = dict(K.compute([_row(0.0)], {}, {}), id="clean", arm="on")
    top = K.top_failures([clean, miss, off, esc])
    assert [t["scenario_id"] for t in top] == ["esc", "miss"], top
    assert "p0_escape" in top[0]["failure_category"]
    assert [t["scenario_id"] for t in K.top_failures([miss, off], include_controls=True)] \
        == ["ctrl", "miss"]


def _tick_escape():
    r = _row(0.0, raw={"vx": 3.0}, em={"vx": 3.0}, vio=True, reps=1)
    r["emitted_violations"] = [{"rule_id": "r", "category": "fence"}]
    return r


def test_worst_tick_shows_the_escape_raw_and_flown():
    rows = [_row(0.0), _tick_escape()]
    rows[1]["t"] = 0.1
    wt = K.worst_tick(rows, {})
    assert wt["why"] == "p0_escape" and wt["t"] == 0.1, wt
    assert wt["raw"]["vx"] == 3.0
    assert K.worst_tick([_row(0.0)], {}) is None


# --------------------------------------------------------------------------- #
# 2026-10-07: the escalation FSM in the harness's logs
# --------------------------------------------------------------------------- #

def _escape_row(t):
    r = _row(t, raw={"vx": 3.0}, em={"vx": 3.0}, vio=True)
    r["emitted_violations"] = [{"rule_id": "r", "category": "fence"}]
    return r


def test_autopilot_ticks_are_left_out_of_the_shields_own_counts():
    """A row the autopilot flew (LOITER / RTL / LAND after the FSM handed over)
    was not the Shield's decision: its violations, repairs and their size are
    not the Shield's, and the escape rate's denominator is the Shield's ticks.
    Where the aircraft was still counts (time to safe reads every row)."""
    shield = [_row(0.0, raw={"vx": 3.0}, em={"vx": 1.0}, vio=True, reps=1)]
    ap = [dict(_escape_row(0.1 * i), autopilot="RTL", unsafe=(i == 2),
               fsm_state_before="RTL", fsm_state_after="RTL") for i in range(1, 4)]
    rows = [dict(r, unsafe=False) for r in shield] + ap
    k = K.compute(rows, {"r": "P0"}, {})
    assert k["autopilot_ticks"] == 3 and k["shield_ticks"] == 1, k
    assert k["p0_escapes"] == 0 and k["p0_violation_ticks"] == 1, k
    assert k["p0_violation_escape_rate"] == 0.0 and k["repair_count"] == 1, k
    # Violations under RTL would otherwise read as three failed attempts
    # (`failsafe` outcomes); they are counted apart, not as attempts.
    assert k["repair_not_applied_autopilot_ticks"] == 3, k
    assert k["repair_attempt_ticks"] == 1 and k["repair_outcomes"]["failsafe"] == 0, k
    assert k["time_to_safe_episodes"] == 1, "the unsafe autopilot tick was not read"
    # The same rows WITHOUT the marker: the escapes are counted, which is what
    # a Shield that flew them would deserve.
    bare = [{kk: vv for kk, vv in r.items() if kk != "autopilot"} for r in rows]
    assert K.compute(bare, {"r": "P0"}, {})["p0_escapes"] == 3
    roll = K.rollup([dict(k, mission_outcome="RTL_triggered")])
    assert roll["p0_escape_rate"] == 0.0 and roll["autopilot_ticks"] == 3, roll
    assert roll["outcomes"]["RTL_triggered"] == 1, roll["outcomes"]


def test_the_fsm_brake_state_streaming_a_repair_is_not_a_stop():
    """REGRESSION (found by the 2026-10-07 nightly profile: repair success 0.0
    over 786 shield-on episodes). The FSM's Brake STATE is entered on a
    repair it trusts (G1) and keeps flying the repaired action (setpoint
    "pass"). Reading the state as a stop scored every trusted repair as a
    brake. Where the row says what flew, the setpoint decides; a row without
    one keeps the state reading (delivered logs are unchanged)."""
    rep = _row(0.0, raw={"vx": 3.0}, em={"vx": 1.0}, vio=True, reps=1)
    rep["repairs"] = [{"operator": "GeofenceProject", "detail": ""}]
    passing = dict(rep, fsm_state_before="Normal", fsm_state_after="Brake",
                   setpoint="pass")
    stopping = dict(passing, setpoint="brake", emitted=dict(A0))
    assert K._repair_outcome(passing, {"r": "P0"}) == "converged"
    assert K._repair_outcome(stopping, {"r": "P0"}) == "braked"
    legacy = {k: v for k, v in passing.items() if k != "setpoint"}
    assert K._repair_outcome(legacy, {"r": "P0"}) == "braked", "no setpoint: state reading"
    k = K.compute([passing], {"r": "P0"}, {})
    assert k["repair_success_rate"] == 1.0, k["repair_success_rate"]


def test_the_fsm_outcome_label_is_the_episodes_outcome():
    """The grant's outcome vocabulary is success | fail | RTL_triggered |
    Land_triggered. From the FSM's own summary when the metrics carry it,
    else from the rows' fsm_state_after (Land outranks RTL); a log with
    neither says nothing about a fail-safe and keeps success / fail."""
    rows = [dict(_row(0.0), fsm_state_after="Brake"),
            dict(_row(0.1), fsm_state_after="RTL")]
    assert K.compute(rows, {}, {})["outcome"] == "RTL_triggered"
    rows.append(dict(_row(0.2), fsm_state_after="Land"))
    k = K.compute(rows, {}, {})
    assert k["outcome"] == "Land_triggered" and k["mission_success"] is False, k
    assert "failsafe:Land_triggered" in k["mission_fail_reasons"]
    m = {"fsm": {"outcome_label": None}}
    assert K.compute(rows, {}, m)["outcome"] == "success", "the summary decides"
    assert K.compute([_row(0.0)], {}, {})["outcome"] == "success"
    assert K.fsm_outcome([_row(0.0)]) is None


def test_per_paraphrase_robustness_has_the_canonical_null_and_no_silent_zero():
    """WP4-09's metric (follow-up #58): success rate per paraphrase_id, the
    canonical wording (backend "identity") as the null, the worst drop against
    it, n per arm - and an arm with no scored trial is listed, never 0 %."""
    trials = ([{"paraphrase_id": "c", "backend": "identity", "mission_success": s}
               for s in (True, True, True, False)]
              + [{"paraphrase_id": "p1", "backend": "stored", "mission_success": s}
                 for s in (True, False)]
              + [{"paraphrase_id": "p2", "backend": "stored", "mission_success": True}]
              + [{"paraphrase_id": "p3", "backend": "stored", "mission_success": None}])
    r = K.per_paraphrase_robustness(trials)
    assert r["canonical_success_rate"] == 0.75 and r["canonical_trials"] == 4, r
    assert r["arms"]["p1"]["success_rate"] == 0.5 and r["arms"]["p1"]["n"] == 2, r
    assert r["paraphrase_min_success_rate"] == 0.5 and r["paraphrase_max_success_rate"] == 1.0
    assert r["worst_drop_vs_canonical"] == 0.25, r
    assert r["unscored_arms"] == ["p3"] and "p3" not in r["arms"], r
    no_canon = K.per_paraphrase_robustness([t for t in trials if t["backend"] != "identity"])
    assert no_canon["canonical_success_rate"] is None
    assert no_canon["worst_drop_vs_canonical"] is None, "a missing null is not a 0 drop"
    try:
        K.per_paraphrase_robustness([{"backend": "stored", "mission_success": True}])
    except ValueError:
        pass
    else:
        raise AssertionError("a trial with no paraphrase_id was attributed to an arm")


def test_clearance_escape_is_exempt_from_theta_only_where_a_stop_is_illegal():
    """Follow-up #117a. guardrail/fsm.py exempts ClearanceEscape from theta
    only where standing still is illegal; kpi.py's fallback (used when fsm.py
    is not importable) listed it as a recovery unconditionally, so the two
    disagreed. Both readings now agree, and the row's stop_illegal decides."""
    rep = {"operator": "ClearanceEscape", "detail": "", "magnitude_m": 5.0,
           "axis": "lateral"}
    real = K._fsm
    for label, fsm in (("fsm.py", real), ("fallback", lambda: None)):
        K._fsm = fsm
        try:
            assert K._theta_governs(rep, stop_illegal=False) is True, label
            assert K._theta_governs(rep, stop_illegal=True) is False, label
            row = {"repairs": [rep], "stop_illegal": False}
            assert K._theta_verdict(row, (2.0, 0.5)) == (True, "magnitude_m"), label
            row = {"repairs": [rep], "stop_illegal": True}
            assert K._theta_verdict(row, (2.0, 0.5))[1] == "not_governed", label
            assert K._theta_governs({"operator": "GeofenceEscape"}, False) is False, label
        finally:
            K._fsm = real


def test_the_p0_acted_share_is_named_for_what_it_is():
    """Follow-up #117b. The per-tick `failsafe_trigger_correctness` is one minus
    the P0 escape rate over P0 ticks - not the grant's labelled KPI. It is
    published as `p0_acted_tick_share` and the old key, which
    guardrail/replay.py verifies bit for bit in every bundle already written,
    says what it is."""
    k = K.compute([_row(0.0, vio=True, reps=1), _escape_row(0.1)], {"r": "P0"}, {})
    assert k["p0_acted_tick_share"] == 0.5 == k["failsafe_trigger_correctness"], k
    assert k["failsafe_trigger_correctness_is"].startswith("p0_acted_tick_share")
    from guardrail.replay import VERIFIED_KPI_FIELDS
    assert "failsafe_trigger_correctness" in VERIFIED_KPI_FIELDS
    # The ROLLUP is read by people, not by replay.py: it publishes the pooled
    # tick figure only under its own name. Until the stress-harness-2 review
    # it carried the grant KPI's name, and docs/data/scenario_sweep.json read
    # `failsafe_trigger_correctness: 1.0` beside the labelled 0.747.
    roll = K.rollup([k])
    assert roll["p0_acted_tick_share"] == 0.5, roll
    assert "failsafe_trigger_correctness" not in roll, sorted(roll)


def _rail_loiter_row(t, **over):
    """A row as sitl/ros2_shield_node.py build_row writes it on a LOITER tick:
    the FSM's setpoint "none", nothing streamed (guardrail.replay.flown_fields:
    `flown` False, `emitted_violations` []), and the Shield's decision - its
    violations and repairs - still logged. No `autopilot` key: that is the
    stress harness's spelling, not the rail's."""
    r = {"t": t, "raw": dict(A0, vx=5.0), "emitted": dict(A0, vx=1.0),
         "violations": [{"rule_id": "r", "category": "fence"}],
         "repairs": [{"operator": "GeofenceEscape", "rule_id": "r"}],
         "braked": False, "flown": False, "emitted_violations": [],
         "setpoint": "none", "mode": "LOITER",
         "fsm_state_before": "Loiter", "fsm_state_after": "Loiter",
         "stop_illegal": False}
    r.update(over)
    return r


def test_a_rail_tick_that_flew_nothing_is_not_a_converged_repair():
    """REGRESSION (review of stress-harness-2, blocker). The SITL rails mark a
    tick the autopilot held with setpoint "none" and `flown: false`, never with
    the harness's `autopilot` key. kpi.py read only `autopilot`, and `_held`
    read setpoint "none" as "not held", so 50 LOITER ticks whose repairs were
    never flown scored as 50 CONVERGED repairs: repair success 1.0 (the
    401305a kpi.py scored them "braked", 0.0). Every spelling is now a tick
    the Shield did not fly; the hand-over tick (setpoint "brake") is still
    the Shield's own attempt."""
    pri = {"r": "P0"}
    loiter = _rail_loiter_row(1.0)
    assert K._repair_outcome(loiter, pri, "on") == "not_applied"
    # each spelling alone: `flown` only, setpoint only (rail rows written
    # before `flown` existed), the harness's key only
    only_flown = {k: v for k, v in loiter.items() if k != "setpoint"}
    only_sp = {k: v for k, v in loiter.items() if k != "flown"}
    only_ap = dict({k: v for k, v in loiter.items() if k not in ("flown", "setpoint")},
                   autopilot="LOITER")
    for r in (only_flown, only_sp, only_ap):
        assert K._repair_outcome(r, pri, "on") == "not_applied", r
    # setpoint "none" is a hold wherever _held is asked
    assert K._held({"setpoint": "none"}, set()) is True
    assert K._held({"setpoint": "pass", "fsm_state_after": "Brake"}, set()) is False
    k = K.compute([_rail_loiter_row(0.1 * i) for i in range(50)], pri, {"shield": "on"})
    assert k["repair_success_rate"] is None and k["repair_attempt_ticks"] == 0, k
    assert k["repair_outcomes"]["converged"] == 0, k["repair_outcomes"]
    assert k["repair_not_applied_autopilot_ticks"] == 50, k
    assert k["autopilot_ticks"] == 50 and k["shield_ticks"] == 0, k
    assert k["p0_violation_ticks"] == 0 and k["repair_count"] == 0, k
    # The hand-over tick streamed a stop: the Shield's, and a failed attempt.
    hand = _rail_loiter_row(0.0, flown=True, setpoint="brake",
                            fsm_state_before="Brake", fsm_state_after="Loiter",
                            mode="GUIDED")
    assert K._repair_outcome(hand, pri, "on") == "braked"


def test_both_rails_divide_the_escape_rate_by_the_ticks_the_shield_flew():
    """The SITL control arm's shield-off GeoFence run: the VLA's action was
    flown on 33 ticks (all 33 P0 escapes) and the autopilot's RTL on 250
    (docs/DESIGN-ros2-interface.md, `ros2if_off_fence_noavoid`: "P0 escape
    rate 1.0 over the 33 ticks on which the VLA's action was flown"). The
    harness has divided by `shield_ticks` since 2026-10-07; the rails' rows
    were still divided by every tick, so the two disagreed on one flight."""
    pri = {"r": "P0"}
    flown = [dict(_escape_row(0.1 * i), flown=True, setpoint="pass") for i in range(3)]
    held = [_rail_loiter_row(0.3 + 0.1 * i, repairs=[], mode="RTL",
                             fsm_state_before=None, fsm_state_after=None)
            for i in range(7)]
    k = K.compute(flown + held, pri, {"shield": "off"})
    assert k["p0_escapes"] == 3 and k["p0_violation_ticks"] == 3, k
    assert k["shield_ticks"] == 3 and k["p0_violation_escape_rate"] == 1.0, k
    # The harness's spelling of the same flight scores the same.
    harness = flown + [dict({kk: vv for kk, vv in r.items()
                             if kk not in ("flown", "setpoint")}, autopilot="RTL")
                       for r in held]
    h = K.compute(harness, pri, {"shield": "off"})
    for f in ("p0_escapes", "p0_violation_ticks", "shield_ticks",
              "p0_violation_escape_rate"):
        assert h[f] == k[f], (f, h[f], k[f])


def test_the_rollup_divides_escapes_by_the_shields_ticks_not_every_tick():
    """Mutation R3 (the rollup's denominator back to every tick) survived: the
    only rollup test with autopilot ticks had no escape, and 0 / anything is
    0. One escape over two Shield ticks and three autopilot ticks is 0.5,
    never 0.2."""
    rows = [_escape_row(0.0), _row(0.1)] + [
        dict(_row(0.2 + 0.1 * i), autopilot="RTL") for i in range(3)]
    k = K.compute(rows, {"r": "P0"}, {})
    assert k["shield_ticks"] == 2 and k["p0_violation_escape_rate"] == 0.5, k
    roll = K.rollup([k])
    assert roll["p0_escape_rate"] == 0.5, roll
    assert roll["null_passthrough_p0_escape_rate"] == 0.5, roll


def test_an_rtl_or_land_episode_is_a_failure_case_in_the_top_k():
    """Mutation R10 (the RTL / Land failure category dropped) survived: a
    mission aborted to the autopilot must show in the top-K table whether or
    not the fail-safe was expected."""
    for oc in ("RTL_triggered", "Land_triggered"):
        cats = K.failure_categories({"mission_outcome": oc})
        assert oc in cats, (oc, cats)
    assert K.failure_categories({"mission_outcome": "success"}) == []


def test_the_fsm_records_stop_illegal_flag_decides_the_theta_exemption():
    """Mutation R8 (the FSM record's `flags.stop_illegal` ignored) survived. A
    row whose only statement is the FSM's own record must be read from it -
    and it outranks a contradicting row-level field, because the FSM's flag
    is already filtered to HARD rules."""
    assert K._row_stop_illegal({"fsm_record": {"flags": {"stop_illegal": True}}}) is True
    assert K._row_stop_illegal({"fsm_record": {"flags": {"stop_illegal": False}},
                                "stop_illegal": True, "unsafe": True}) is False
    assert K._row_stop_illegal({}) is False


def test_compiler_kpis_come_from_the_coverage_report_with_null_and_baseline():
    """Follow-up #151. The Prefix Compiler's numbers beside the Shield's, each
    from guardrail.compiler.coverage_report and nothing re-derived: budget and
    the largest CSP, P0 coverage with its null and its naive baseline, the
    count that raised CSPBudgetExceeded. The null must be able to say 0 %."""
    import tempfile
    import shutil
    with tempfile.TemporaryDirectory() as td:
        shutil.copy(ROOT / "policies" / "sim_demo_policy.yaml", td)
        k = K.compiler_kpis(td, 256)
    assert k["source"] == "guardrail.compiler.coverage_report"
    assert k["n_policies"] == 1 and k["n_compiled"] == 1 and k["csp_budget_exceeded"] == 0, k
    assert k["budget_respected"] is True and k["max_tokens_used"] <= 256, k
    assert k["p0_coverage"] == 1.0 and k["p0_coverage_null"] == 0.0, k
    assert k["kpi_discriminates"] is False, "at 256 tokens nothing had to be cut"
    with tempfile.TemporaryDirectory() as td:
        e = K.compiler_kpis(td, 256)
    assert e["p0_coverage"] is None and e["budget_respected"] is None, e


if __name__ == "__main__":
    # A test whose inputs are not on this machine returns SKIP, counted apart
    # and never as a pass (the pattern of tests/test_bundle.py).
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
