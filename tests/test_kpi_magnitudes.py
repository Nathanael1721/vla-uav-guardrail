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
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import kpi as K                                     # noqa: E402

A0 = {"vx": 0.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0}


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
    included in the norm this reports 30.0, and the KPI would silently depend on
    whether turn rate happened to be expressed in degrees or radians.
    """
    rows = [_row(0.0, raw={"yaw_rate": 30.0}, em={"yaw_rate": 0.0},
                 vio=True, reps=1)]
    res = K.compute(rows, {}, {})
    assert res["mean_repair_magnitude_mps"] == 0.0, res
    assert abs(res["mean_yaw_repair_dps"] - 30.0) < 1e-6, res


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
        return
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
        return

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


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
