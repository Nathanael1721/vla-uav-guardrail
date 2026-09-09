"""The target estimator: does it actually smooth the command, and does it fail safe?

Run either way:
    pytest tests/test_target_state.py -v
    python tests/test_target_state.py

The estimator exists to fix one measured problem: the forward channel was
proportional on the detector's box width, which jitters p95 37.7% between
detections in traffic, so the raw command stepped up to 1.897 m/s in a single
0.1 s tick and the slew limiter clipped 20% of ticks. These tests hold it to
that, and to failing safe when the detector is wrong or absent - because an
estimator that propagates a bad lock is worse than one that forgets.
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from target_state import TargetState, want_range_from_width      # noqa: E402

DT = 0.1                      # control tick
DET_EVERY = 0.25              # ~4 Hz detector, as measured in flight


def _run(noise_m=1.5, dropout=(), seed=7, secs=20.0, tgt_speed=2.0):
    """Aircraft trailing a car that drives +y at constant speed.

    Returns (estimator, per-tick served range, truth range) so a test can score
    both smoothness and accuracy.
    """
    rng = np.random.default_rng(seed)
    est = TargetState()
    dx, dy, yaw = 35.0, 0.0, math.radians(90.0)
    served, truth, t_next_det = [], [], 0.0
    t = 0.0
    while t < secs:
        tx, ty = 38.0, 20.0 + tgt_speed * t          # the car
        dy += 1.9 * DT                                # the aircraft, trailing
        if t >= t_next_det and not any(a <= t <= b for a, b in dropout):
            t_next_det = t + DET_EVERY
            true_b = math.atan2(ty - dy, tx - dx) - yaw
            true_r = math.hypot(tx - dx, ty - dy)
            # depth is quantised to whole metres; the box wobbles on top
            meas_r = round(true_r) + rng.normal(0, noise_m * 0.4)
            meas_b = true_b + math.radians(rng.normal(0, 1.5))
            est.update(t, dx, dy, yaw, meas_b, meas_r)
        obs = est.observe(t, dx, dy, yaw)
        served.append(None if obs is None else obs[1])
        truth.append(math.hypot(tx - dx, ty - dy))
        t += DT
    return est, served, truth


def _steps(v):
    v = [x for x in v if x is not None]
    return [abs(v[i + 1] - v[i]) for i in range(len(v) - 1)]


def test_it_serves_a_value_on_every_tick():
    """The detector runs at 3.8-5.6 Hz against a 10 Hz loop, so 62% of ticks
    used to reuse a frozen box. A prediction is available on all of them."""
    _, served, _ = _run()
    assert all(s is not None for s in served[5:]), "gaps in the served estimate"


def test_the_forward_command_is_smoother_than_the_raw_measurement():
    """THE POINT OF THE CHANGE. Compare the per-tick step of a range-driven
    forward command against one driven by the raw noisy measurement."""
    est, served, _ = _run()
    want = want_range_from_width(0.16)
    smooth_cmd = [np.clip((r - want) * 0.25, -1.2, 3.0) for r in served if r is not None]
    # the same controller fed the raw measurement instead
    rng = np.random.default_rng(7)
    raw = [round(r) + rng.normal(0, 0.6) for r in _run()[2]]
    raw_cmd = [np.clip((r - want) * 0.25, -1.2, 3.0) for r in raw]
    s_smooth, s_raw = _steps(smooth_cmd), _steps(raw_cmd)
    p95 = lambda a: sorted(a)[int(0.95 * len(a))]
    assert p95(s_smooth) < 0.5 * p95(s_raw), (
        f"estimator did not smooth the command: p95 {p95(s_smooth):.4f} "
        f"against raw {p95(s_raw):.4f}")


def test_it_tracks_a_moving_target_accurately_enough_to_servo_on():
    est, served, truth = _run()
    err = [abs(s - t) for s, t in zip(served, truth) if s is not None][20:]
    assert np.median(err) < 2.5, f"median range error {np.median(err):.2f} m"


def test_it_coasts_through_a_dropout_and_then_gives_up():
    """A dropout must degrade gracefully: keep predicting for a while, then
    refuse to answer rather than confidently point at a stale guess."""
    est, served, _ = _run(dropout=[(8.0, 12.0)])
    i_mid = int(9.0 / DT)
    assert served[i_mid] is not None, "gave up immediately on a 1 s gap"
    i_late = int(11.8 / DT)
    assert served[i_late] is None, (
        "still answering after 3.8 s unseen; max_coast_s should have expired")


def test_a_wild_measurement_is_gated_out():
    """The risk this design adds: a confident lock on the WRONG vehicle would be
    propagated instead of forgotten. The innovation gate is what stops it."""
    est = TargetState()
    yaw = math.radians(90.0)
    for k in range(6):                      # establish a good track
        est.update(k * 0.25, 35.0, 0.0, yaw, 0.0, 20.0)
    before = est.x.copy()
    ok = est.update(1.6, 35.0, 0.0, yaw, math.radians(40.0), 80.0)
    assert ok is False and est.n_rejected == 1, "wild measurement was accepted"
    assert np.allclose(est.x[:2], before[:2], atol=2.0), "state moved anyway"


def test_the_velocity_feedforward_removes_the_stand_off_lag():
    """A proportional stand-off loop lags a moving target. Measured in flight:
    a 2 m/s car and a 0.25 gain settled at 23.4 m against a 15.8 m stand-off,
    the (r - want)*gain = 2.0 fixed point. An I term would fix it and would wind
    up whenever the Shield overrides the actuator; the estimator already knows
    the target's velocity, so feed it forward instead."""
    est, _, _ = _run(tgt_speed=2.0)
    dx, dy = 35.0, 0.0
    ff = est.range_rate(dx, dy)
    assert ff > 1.0, f"feedforward should see the car opening: {ff:.2f} m/s"
    want = want_range_from_width(0.16)
    # equilibrium range where the loop is balanced, with and without the term
    r_p_only = want + 2.0 / 0.25                      # 23.8 m: the measured lag
    r_with_ff = want + max(0.0, (2.0 - ff)) / 0.25
    assert r_with_ff < r_p_only - 5.0, (
        f"feedforward barely helped: {r_with_ff:.1f} m against {r_p_only:.1f} m")


def test_the_feedforward_is_zero_for_a_stationary_target():
    """It must not push the aircraft in when the car stops - that is the failure
    an integrator would have, draining slowly after the target halts."""
    est = TargetState()
    yaw = math.radians(90.0)
    for k in range(8):
        est.update(k * 0.25, 35.0, 0.0, yaw, 0.0, 20.0)   # never moves
    assert abs(est.range_rate(35.0, 0.0)) < 0.4, (
        f"feedforward {est.range_rate(35.0, 0.0):.2f} m/s on a parked car")


def test_it_reports_nothing_before_it_has_seen_anything():
    est = TargetState()
    assert est.observe(0.0, 0.0, 0.0, 0.0) is None
    assert est.speed() == 0.0
    assert est.age_s(1.0) is None


def test_the_stand_off_matches_what_want_width_used_to_hold():
    """--want-width must keep its meaning after the servo stops using apparent
    width, or every recorded flight becomes incomparable. The width servo held
    13-15 m at 0.16 on a 4 m car."""
    r = want_range_from_width(0.16, 4.0)
    assert 14.0 < r < 18.0, f"stand-off moved to {r:.1f} m"
    assert want_range_from_width(0.32, 4.0) < r, "bigger box must mean closer"


# ------------------------------------------------- subject width by name

def _subject_width():
    """Load the width table without importing follow_vlm, which needs the sim."""
    src = (ROOT / "demo" / "follow_vlm.py").read_text(encoding="utf-8")
    start = src.index("SUBJECT_WIDTH_M = {")
    end = src.index("\n\n\n", src.index("def subject_width"))
    ns = {}
    exec(src[start:end], ns)                                     # noqa: S102
    return ns["subject_width"]


def test_a_pedestrian_is_not_assumed_to_be_a_car():
    """implied_range_from_width assumed 4.0 m for everything. A pedestrian is
    about 0.5 m, so the same code reported a person at ~8x their true distance -
    and the 2026-08-19 review asked for a pedestrian in the scene next."""
    sw = _subject_width()
    w_car, from_car = sw("a yellow car")
    w_ped, from_ped = sw("a pedestrian")
    assert (w_car, from_car) == (4.0, "car")
    assert (w_ped, from_ped) == (0.5, "pedestrian")
    assert w_car / w_ped == 8.0, "the error this fixes is a factor of eight"


def test_the_derived_stand_off_shrinks_with_the_subject():
    """--want-width is angular, so the stand-off it asks for scales with the
    subject's real width. Same flag, different mission."""
    sw = _subject_width()
    car = want_range_from_width(0.16, sw("a yellow car")[0])
    ped = want_range_from_width(0.16, sw("a pedestrian")[0])
    assert car > 4 * ped, f"car {car:.1f} m vs pedestrian {ped:.1f} m"
    assert 15.0 < car < 17.0, f"the measured car stand-off moved: {car:.1f} m"


def test_an_unknown_subject_falls_back_audibly():
    """A silent 4.0 must not look like a decision: the caller needs to know no
    word matched so it can say so."""
    sw = _subject_width()
    w, word = sw("a purple giraffe")
    assert w == 4.0 and word is None


def test_the_longest_matching_word_wins():
    """So a more specific entry is never shadowed by a shorter one inside it."""
    sw = _subject_width()
    assert sw("a motorcycle")[1] == "motorcycle"
    assert sw("a white bus")[1] == "bus"


def test_every_car_query_still_resolves_to_four_metres():
    """Every recorded flight used 4.0. If any car-ish phrase now derives
    something else, the existing measured numbers stop being comparable."""
    sw = _subject_width()
    for q in ("a yellow car", "a red car", "the taxi", "a blue sedan"):
        assert sw(q)[0] == 4.0, f"{q!r} changed the width of past flights"


# --- counters across a subject change --------------------------------------


def test_reset_banks_the_counters_instead_of_dropping_them():
    """`update()` re-initialises with `n_updates = 1`, so without banking, a
    flight that retargeted reported post-retarget updates beside whole-flight
    rejections: two numbers from different tracks in one dict. That pair read
    as an estimator rejecting more than it accepted, which it was not."""
    e = TargetState()
    e.n_updates, e.n_rejected = 40, 7
    e.reset()
    assert e.n_updates == 0 and e.n_rejected == 0
    e.n_updates, e.n_rejected = 3, 2
    out = e.summary()
    assert out["updates"] == 3 and out["gated_out"] == 2, out
    assert out["resets"] == 1, out
    assert out["updates_total"] == 43 and out["gated_out_total"] == 9, out


def test_a_flight_that_never_retargeted_reports_the_same_keys_as_before():
    """No reset, no extra keys - the 40-odd metrics.json on disk keep their
    shape and nothing downstream has to learn a new field it will not see."""
    e = TargetState()
    assert set(e.summary()) == {"updates", "gated_out", "speed_mps"}


def test_the_gate_arms_on_the_new_track_not_the_old_one():
    """The gate is `n_updates >= 3`. After a reset the filter knows nothing, so
    it must accept the first observations of the new subject unconditionally -
    otherwise a retarget would gate out the very target it was pointed at."""
    e = TargetState()
    e.n_updates = 500
    e.reset()
    assert e.n_updates == 0


# --------------------------------------------------------------------------
# The per-class speed prior, replayed from the flight that needed it.
# --------------------------------------------------------------------------

FLIGHT = ROOT / "demo" / "out" / "retarget_fixed" / "flight_log.jsonl"


def _replay_pedestrian_phase(v_max):
    """Feed the recorded post-retarget measurements through a fresh filter.

    No simulator: the rows carry the aircraft pose, the box bearing and the
    depth range, which is everything `update()` takes. The point of replaying a
    REAL flight rather than a synthetic one is that the failure being guarded
    against was not synthesisable - it needed a detector emitting boxes
    scattered across a city block, which is what a 0.5 m subject at 8-13 px
    produces and what no hand-written noise model was going to reproduce.
    """
    import json
    rows = [json.loads(l) for l in FLIGHT.open(encoding="utf-8")]
    seg = [r for r in rows if (r.get("truth") or {}).get("class") == "pedestrian"]
    est = TargetState(v_max=v_max)
    served, speeds, last_seq = 0, [], None
    for r in seg:
        det, rng, seq = r.get("det"), r.get("rng_m"), r.get("det_seq")
        if det and rng and seq != last_seq:
            last_seq = seq
            est.update(r["t"], r["x"], r["y"], r["psi"],
                       math.radians(r["bearing_deg"]), float(rng))
        est.predict(r["t"])
        if est.observe(r["t"], r["x"], r["y"], r["psi"]) is not None:
            served += 1
            speeds.append(est.speed())
    return est, served, len(seg), (max(speeds) if speeds else 0.0)


def test_a_pedestrian_is_never_credited_with_a_cars_speed():
    """Without the prior this flight attributed 13.06 m/s to a person.

    That is not a cosmetic number. Once the estimate had run away, the filter's
    own 4-sigma gate rejected 149 of the next 154 measurements, so it coasted
    the runaway velocity to the end of the flight and served the Shield a
    position 6-12 m from any real person. All six firings of the 10 m stand-off
    ring were against that phantom.
    """
    if not FLIGHT.exists():
        print("SKIP  no recorded flight at demo/out/retarget_fixed")
        return
    _, _, _, fast = _replay_pedestrian_phase(None)
    assert fast > 10.0, (
        f"the flight this guards no longer diverges ({fast:.2f} m/s) - if that "
        "is a real improvement, re-point this test at a flight that does, but "
        "do not delete it: a guard whose failure case has vanished from the "
        "fixtures stops guarding silently")
    _, _, _, capped = _replay_pedestrian_phase(2.0)
    assert capped <= 2.0 + 1e-6, f"clamp let {capped:.2f} m/s through"


def test_the_clamp_keeps_the_estimate_usable_instead_of_coasting():
    """Served on more than half the ticks, against under a fifth without it.

    The clamp is not a cosmetic bound on a number nobody reads: capping the
    velocity stops the divergence that makes the Mahalanobis gate reject
    everything, so the filter keeps ACCEPTING measurements. Fewer than half the
    ticks served means the controller is steering on coast, which is where the
    yaw chaos came from.
    """
    if not FLIGHT.exists():
        print("SKIP  no recorded flight at demo/out/retarget_fixed")
        return
    _, loose, n, _ = _replay_pedestrian_phase(None)
    _, tight, _, _ = _replay_pedestrian_phase(2.0)
    assert loose / n < 0.25, f"baseline served {loose}/{n}, expected the failure"
    assert tight / n > 0.5, f"with the clamp served only {tight}/{n}"
    assert tight > loose * 2, f"{loose} -> {tight} is not the measured improvement"


def test_the_clamp_scales_the_pair_and_does_not_rotate_it():
    """Clipping vx and vy separately would turn the target's heading, and the
    heading is what the yaw servos on - a speed error is recoverable, a bearing
    error steers the aircraft the wrong way."""
    e = TargetState(v_max=2.0)
    e.x = np.array([0.0, 0.0, 6.0, 8.0], float)     # 10 m/s at atan2(8,6)
    e.P = np.eye(4)
    e.t_last_update = 0.0
    e._t_state = 0.0
    e.n_updates = 5
    before = math.atan2(e.x[3], e.x[2])
    # one in-gate measurement, so the clamp is the only thing that moves it
    e.update(0.1, 0.0, 0.0, 0.0, 0.0, 1.0)
    assert e.speed() <= 2.0 + 1e-6
    assert abs(math.atan2(e.x[3], e.x[2]) - before) < math.radians(20.0), (
        "the clamp rotated the velocity - scale the pair, do not clip the axes")


def test_no_ceiling_is_the_old_behaviour_exactly():
    """Every caller that predates the prior must be bit-for-bit unaffected."""
    a, b = TargetState(), TargetState(v_max=None)
    for e in (a, b):
        e.update(0.0, 0.0, 0.0, 0.0, 0.0, 20.0)
        e.update(0.3, 0.0, 0.0, 0.0, 0.05, 22.0)
        e.update(0.6, 0.0, 0.0, 0.0, 0.10, 25.0)
    assert np.allclose(a.x, b.x)
    assert "v_max_mps" not in a.summary(), (
        "a filter with no ceiling must not advertise one")
    c = TargetState(v_max=2.0)
    c.update(0.0, 0.0, 0.0, 0.0, 0.0, 20.0)
    assert c.summary()["clamped"] == 0, (
        "report the counter even at zero - an unfired clamp and an uninstalled "
        "one must not look the same in a summary")


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
