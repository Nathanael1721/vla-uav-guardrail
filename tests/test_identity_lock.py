"""The strict lock, the re-acquisition gate, and the estimate scorer.

demo/follow_vlm.py TargetLock.select_strict, Reacquirer, Acquirer(tiers=),
identity_rule_key; demo/track_truth.py score_estimate. Added 2026-09-29
after citylife_redcar_trail ended TARGET LOCKED on a red pedestrian signal
110 m from the car: the old lock adopted the best-scoring box whenever
nothing sat where it expected, and nothing stopped a far box re-seeding the
estimate.

Run either way:
    pytest tests/test_identity_lock.py -v
    python tests/test_identity_lock.py
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from follow_vlm import (Acquirer, Reacquirer, TargetLock,     # noqa: E402
                        identity_rule_key)
import track_truth                                            # noqa: E402

W, H = 768, 432


def box(cx, w=40.0, h=24.0, score=0.1, colour=0.5):
    return (cx, 250.0, w, h, score, W, H, colour)


def feat(x, y, rng_h=20.0):
    return {"P": (x, y, 0.8), "rng_h": rng_h, "r": rng_h, "bearing": 0.0}


# ----------------------------------------------------------------- strict lock
def test_with_nothing_held_and_no_prior_the_strict_lock_adopts_nothing():
    lk = TargetLock()
    k, why = lk.select_strict([box(400)], ["ok"], [feat(20, 0)], W, 0.0, 1.0)
    assert k is None and why == "no held instance"


def test_the_legacy_lock_would_have_adopted_it():
    """The behaviour being replaced, pinned so the difference is on record."""
    lk = TargetLock()
    chosen, _ = lk.select([box(400)], W, 0.0, 1.0)
    assert chosen is not None


def test_a_seeded_lock_follows_its_instance_and_refuses_a_far_one():
    lk = TargetLock()
    lk.seed(box(384), 0.0, 0.0)
    k, _ = lk.select_strict([box(700), box(390)], ["ok", "ok"], None, W, 0.0, 0.3)
    assert k == 1
    k, why = lk.select_strict([box(700)], ["ok"], None, W, 0.0, 0.6)
    assert k is None and "nothing where" in why


def test_hard_is_never_chosen_and_soft_only_close_to_the_prediction():
    lk = TargetLock()
    lk.seed(box(384), 0.0, 0.0)
    k, _ = lk.select_strict([box(386)], ["hard"], None, W, 0.0, 0.3)
    assert k is None
    # 0.06 W = 46 px: a SOFT sliver 30 px off continues, 70 px off does not
    k, _ = lk.select_strict([box(414, w=20)], ["soft"], None, W, 0.0, 0.5)
    assert k == 0
    lk.seed(box(384), 0.0, 0.6)
    k, _ = lk.select_strict([box(454, w=30)], ["soft"], None, W, 0.0, 0.7)
    assert k is None


def test_a_sliver_continues_the_track_without_becoming_its_size():
    """A car half behind a truck is a third of its width; when it comes out,
    the whole car must still pass the size check against the held width."""
    lk = TargetLock()
    lk.seed(box(384, w=40), 0.0, 0.0)
    k, _ = lk.select_strict([box(390, w=13)], ["soft"], None, W, 0.0, 0.3)
    assert k == 0 and lk.w == 40.0
    k, _ = lk.select_strict([box(392, w=41)], ["ok"], None, W, 0.0, 0.6)
    assert k == 0

def test_ok_beats_a_nearer_soft():
    lk = TargetLock()
    lk.seed(box(384), 0.0, 0.0)
    k, _ = lk.select_strict([box(386, w=30), box(420)], ["soft", "ok"], None, W, 0.0, 0.2)
    assert k == 1


def test_once_stale_the_estimator_prior_carries_the_prediction():
    lk = TargetLock()
    lk.seed(box(100), 0.0, 0.0)
    prior = {"cx": 500.0, "P": (30.0, 5.0), "tol_m": 0.0}
    # at 5 s the lock's own prediction (cx 100) is stale; the prior says 500
    k, _ = lk.select_strict([box(110), box(505)], ["ok", "ok"],
                            [feat(30, -20), feat(30.5, 5.2)], W, 0.0, 5.0, prior=prior)
    assert k == 1


def test_a_box_in_the_right_pixel_column_but_the_wrong_place_is_refused():
    """The pedestrian signal behind the occluded car: same bearing, 30 m on."""
    lk = TargetLock()
    prior = {"cx": 384.0, "P": (25.0, 0.0), "tol_m": 1.0}
    k, _ = lk.select_strict([box(386)], ["ok"], [feat(55.0, 0.0)], W, 0.0, 1.0,
                            prior=prior)
    assert k is None
    k, _ = lk.select_strict([box(386)], ["ok"], [feat(27.0, 0.5)], W, 0.0, 1.0,
                            prior=prior)
    assert k == 0


def test_a_soft_box_with_no_map_point_cannot_continue_against_a_prior():
    lk = TargetLock()
    prior = {"cx": 384.0, "P": (25.0, 0.0), "tol_m": 0.0}
    k, _ = lk.select_strict([box(386, w=12)], ["soft"], [{"P": None}], W, 0.0, 1.0,
                            prior=prior)
    assert k is None


# ----------------------------------------------------------------- re-acquisition
def _run(rq, frames, t0=10.0, dt=0.27):
    done = False
    for i, fr in enumerate(frames):
        cands = [box(400) for _ in fr]
        tiers = [f[0] for f in fr]
        feats = [f[1] for f in fr]
        done = rq.step(cands, tiers, feats, t0 + i * dt)
        if done:
            return i + 1
    return None


def test_four_ok_sightings_in_a_row_near_where_it_was_last_seen():
    rq = Reacquirer()
    rq.start({"t": 10.0, "P": (0.0, 0.0), "v": 3.0})
    n = _run(rq, [[("ok", feat(5.0 + 0.8 * i, 0.0))] for i in range(6)])
    assert n == 4 and rq.pick_feat["P"][0] > 5.0


def test_soft_or_rangeless_boxes_never_count():
    rq = Reacquirer()
    rq.start({"t": 10.0, "P": (0.0, 0.0), "v": 3.0})
    assert _run(rq, [[("soft", feat(5.0, 0.0))]] * 6) is None
    assert _run(rq, [[("ok", {"P": None, "rng_h": None})]] * 6) is None
    assert rq.refused.get("not ok") and rq.refused.get("no range")


def test_a_box_beyond_reach_is_refused_and_reach_grows_with_time():
    """110 m from the last sighting 3 s later is not the car (v_r = 4.5 m/s,
    reach 10 + 4.5 * 3 = 23.5 m); 20 m is."""
    rq = Reacquirer()
    rq.start({"t": 7.0, "P": (0.0, 0.0), "v": 3.0})
    assert _run(rq, [[("ok", feat(110.0, 0.0, rng_h=30.0))]] * 6) is None
    assert rq.refused.get("out of reach")
    rq.start({"t": 7.0, "P": (0.0, 0.0), "v": 3.0})
    assert _run(rq, [[("ok", feat(20.0 + 0.8 * i, 0.0))] for i in range(6)]) == 4


def test_too_far_from_the_aircraft_is_refused():
    rq = Reacquirer(max_range_m=45.0)
    rq.start(None)
    assert _run(rq, [[("ok", feat(5.0, 0.0, rng_h=60.0))]] * 6) is None
    assert rq.refused.get("too far")


def test_two_flickering_things_do_not_add_up_to_a_sighting():
    rq = Reacquirer()
    rq.start(None)
    a, b = feat(10.0, 0.0), feat(10.0, 25.0)
    assert _run(rq, [[("ok", a)], [("ok", b)], [("ok", a)], [("ok", b)],
                     [("ok", a)], [("ok", b)]]) is None
    assert rq.refused.get("discontinuous")


def test_with_no_anchor_a_standing_thing_never_starts_a_track():
    """The red fire-hydrant sign at the launch point: car-wide by depth, on
    the street, perfectly steady - and seeded the estimate twice in replay."""
    rq = Reacquirer()
    rq.start(None)
    # 35 sightings 0.27 s apart: 9.2 s without moving, past static_after_s (8)
    assert _run(rq, [[("ok", feat(29.3, -13.7))]] * 35) is None
    assert rq.static and rq.refused.get("static")
    # ...and once marked static it is refused outright
    assert _run(rq, [[("ok", feat(29.5, -13.5))]] * 6, t0=20.0) is None
    assert rq.refused["static"] > 1
    # ...but not for ever: people wait at kerbs, cars at red lights
    n = _run(rq, [[("ok", feat(29.5 + 0.9 * i, -13.5))] for i in range(12)], t0=60.0)
    assert n is not None


def test_a_slow_walker_is_not_taken_for_a_sign():
    """1.3 m/s at the start gate's 5.5 Hz: the old count-capped test (3 x need
    sightings) needed 1.47 m/s and blacklisted the walker as STATIC."""
    rq = Reacquirer(need=5)
    rq.start(None)
    walk = [[("ok", feat(15.0 + 1.3 * 0.182 * i, 2.0))] for i in range(40)]
    n = _run(rq, walk, dt=0.182)
    assert n is not None and not rq.static, (n, rq.static, rq.refused)
    assert rq.heading() is not None and abs(rq.heading()) < 0.2   # northward


def test_with_no_anchor_a_moving_car_is_taken():
    rq = Reacquirer()
    rq.start(None)
    n = _run(rq, [[("ok", feat(20.0 + 0.9 * i, 0.0))] for i in range(8)])
    assert n is not None and n >= 4


def test_after_a_long_loss_the_reach_stops_constraining_and_motion_is_asked():
    rq = Reacquirer()
    rq.start({"t": 0.0, "P": (0.0, 0.0), "v": 3.0})
    assert rq.reach_m(100.0) > rq.reach_cap_m
    # 10 s after the loss (reach 55 m) a standing car-like box is accepted...
    assert _run(rq, [[("ok", feat(30.0, 0.0))]] * 6, t0=10.0) == 4
    # ...100 s after, the same standing box is not
    rq.start({"t": 0.0, "P": (0.0, 0.0), "v": 3.0})
    assert _run(rq, [[("ok", feat(30.0, 0.0))]] * 6, t0=100.0) is None


def _jitter_run(v, sigma, seed, dt=0.27, T=60.0):
    import random
    rng = random.Random(seed)
    rq = Reacquirer()
    rq.start(None)
    t = 0.0
    while t < T:
        f = feat(20.0 + v * t + rng.gauss(0, sigma), rng.gauss(0, sigma))
        if rq.step([box(400)], ["ok"], [f], 10.0 + t):
            return t
        t += dt
    return None


def test_a_jittering_sign_never_passes_and_a_jittering_car_does():
    """A fixed 2 m bar, re-tested on every sighting of a growing streak, let a
    standing sign with 1.5 m of map-point jitter through on 186 of 200
    simulated minutes. The bar now grows with the streak's own jitter."""
    for sigma in (1.0, 1.5, 2.0):
        passed = [s for s in range(40) if _jitter_run(0.0, sigma, s) is not None]
        assert not passed, (sigma, passed)
    times = [_jitter_run(3.2, 1.0, s) for s in range(40)]
    assert all(t is not None for t in times) and sorted(times)[20] < 4.0, times
    walk = [_jitter_run(1.3, 0.5, s, dt=0.182) for s in range(40)]
    assert all(t is not None for t in walk) and sorted(walk)[20] < 6.0, walk

def test_a_car_standing_beyond_the_gate_range_is_a_lead_not_a_pick():
    """citylife_redcar_id3: the car stood at a light 52 m away, every box OK
    and within reach, every box refused "too far". It is kept as a far lead
    to fly toward - never picked - once three agree."""
    rq = Reacquirer()
    rq.start({"t": 10.0, "P": (0.0, 0.0), "v": 3.0})
    t = 30.0                                                 # reach 10 + 4.5 x 20 = 100 m
    for k in range(3):
        assert rq.step([box(400)], ["ok"], [feat(50.0, 20.0, rng_h=52.0)], t + 0.3 * k) is False
    assert rq.streak == 0 and rq.pick is None
    assert rq.far_lead(t + 0.7) == (50.0, 20.0), rq.far
    assert rq.far_lead(t + 0.7 + rq.far_window_s + 1.0) is None       # gone stale
    # two sightings are not enough
    rq.start({"t": 10.0, "P": (0.0, 0.0), "v": 3.0})
    for k in range(2):
        rq.step([box(400)], ["ok"], [feat(50.0, 20.0, rng_h=52.0)], t + 0.3 * k)
    assert rq.far_lead(t + 0.4) is None


def test_a_far_lead_must_be_ok_within_reach_consistent_and_not_static():
    rq = Reacquirer()
    rq.start({"t": 10.0, "P": (0.0, 0.0), "v": 3.0})
    for k in range(4):                                       # SOFT: never a lead
        rq.step([box(400)], ["soft"], [feat(50.0, 20.0, rng_h=52.0)], 30.0 + 0.3 * k)
    assert rq.far_lead(31.0) is None
    rq.start({"t": 29.0, "P": (0.0, 0.0), "v": 3.0})         # reach 13 m: 54 m is out of it
    for k in range(4):
        rq.step([box(400)], ["ok"], [feat(50.0, 20.0, rng_h=52.0)], 30.0 + 0.3 * k)
    assert rq.far_lead(31.0) is None and rq.refused.get("too far")
    rq.start({"t": 10.0, "P": (0.0, 0.0), "v": 3.0})         # two places: not one lead
    for k, y in enumerate((20.0, -40.0, 20.0)):
        rq.step([box(400)], ["ok"], [feat(50.0, y, rng_h=60.0)], 30.0 + 0.3 * k)
    assert rq.far_lead(31.0) is None
    rq.start({"t": 10.0, "P": (0.0, 0.0), "v": 3.0})         # a static place is no lead
    rq.static.append((50.0, 20.0, 29.0))
    for k in range(4):
        rq.step([box(400)], ["ok"], [feat(50.0, 20.0, rng_h=52.0)], 30.0 + 0.3 * k)
    assert rq.far_lead(31.0) is None
    rq.start(None)                                           # no anchor: no lead
    for k in range(4):
        rq.step([box(400)], ["ok"], [feat(50.0, 20.0, rng_h=52.0)], 30.0 + 0.3 * k)
    assert rq.far_lead(31.0) is None


def test_speed_is_clipped_into_the_reach():
    rq = Reacquirer()
    rq.start({"t": 0.0, "P": (0.0, 0.0), "v": None})
    assert rq.v_reach() == 3.0
    rq.start({"t": 0.0, "P": (0.0, 0.0), "v": 20.0})
    assert rq.v_reach() == 12.0



# ----------------------------------------------------------------- grounder
def _grounder():
    from follow_vlm import Grounder
    from semantic_demo import SemanticObs
    import identity
    return Grounder(SemanticObs(), "a red car", lock=TargetLock(),
                    identity={"thresholds": identity.load_thresholds(), "street": None,
                              "hfov": 90.0})


def test_commit_carries_the_capture_yaw_to_the_lock():
    g = _grounder()
    g.commit(box(400), yaw=0.7)
    assert g._seed == (box(400), 0.7)
    g.commit(None)
    assert g._seed is None


def test_the_prior_lands_where_the_camera_would_see_the_point():
    """_prior_at projects through the 20 deg mount and the body attitude; the
    old level-camera shortcut was 20-45 px out for a near, off-axis subject."""
    import camera_model as cam
    g = _grounder()
    pose = {"x": 0.0, "y": 0.0, "up": 8.0, "yaw": 0.3, "pitch": -0.05, "roll": 0.02}
    n, e = 8.0, 6.0                                  # 10 m out, ~37 deg right
    g.set_prior({"t": 5.0, "x": n, "y": e, "vx": 0.0, "vy": 0.0, "t_upd": 5.0, "up": 0.75})
    pr = g._prior_at(pose, W, H, 5.0)
    u, _v = cam.world_to_pixel(n, e, 0.75, 0.0, 0.0, 8.0, W, H, 90.0, 0.3, -0.05, 0.02)
    assert abs(pr["cx"] - u) < 1e-6
    b = math.atan2(e, n) - 0.3
    level = cam.bearing_to_cx(b, W, 90.0)
    assert abs(level - u) > 15.0                     # what the shortcut got wrong


def test_the_rule_counters_file_each_reason_under_its_own_tier():
    """A signal head is HARD by its bottom; its narrow width and aspect only
    ever demote and must not be counted as HARD rejections."""
    g = _grounder()
    pose = {"x": 0.0, "y": 0.0, "up": 8.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}
    dep = __import__("numpy").full((H, W), 15.0, dtype="float32")
    # a small box high in the frame at 15 m: bottom well above the road
    tier, why, f = g._judge((400.0, 150.0, 12.0, 22.0, 0.1, W, H, 0.5), dep, pose, W, H)
    assert tier == "hard", (tier, why, f)
    assert g.id_rules == {"hard:bottom": 1, "soft:width-min": 1, "soft:aspect": 1}, g.id_rules


def test_a_prior_under_or_behind_the_aircraft_gives_no_column():
    """Behind or under the nose a point still projects to SOME column; a
    prior there must not steer the lock - the subject is in the blind spot."""
    g = _grounder()
    pose = {"x": 0.0, "y": 0.0, "up": 8.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}
    for n, e in ((-2.0, 0.5), (0.5, 0.0), (2.0, 0.0)):
        g.set_prior({"t": 1.0, "x": n, "y": e, "vx": 0.0, "vy": 0.0, "t_upd": 1.0, "up": 0.75})
        assert g._prior_at(pose, W, H, 1.0)["cx"] is None, (n, e)
    g.set_prior({"t": 1.0, "x": 12.0, "y": 1.0, "vx": 0.0, "vy": 0.0, "t_upd": 1.0, "up": 0.75})
    assert g._prior_at(pose, W, H, 1.0)["cx"] is not None

# ----------------------------------------------------------------- start gate
def test_the_start_gate_may_not_start_on_a_soft_box():
    acq = Acquirer(2, "a red car", 0.1)
    c = box(400)
    assert not acq.step([c], lambda _c: 20.0, tiers=["soft"])
    assert not acq.step([c], lambda _c: 20.0, tiers=["soft"])
    assert any("identity" in r for r in acq.last_reasons)
    assert not acq.step([c], lambda _c: 20.0, tiers=["ok"])
    assert acq.step([c], lambda _c: 20.0, tiers=["ok"])


# ----------------------------------------------------------------- counting
def test_rule_keys():
    assert identity_rule_key("bottom 2.4 m above the road > 2") == "bottom"
    assert identity_rule_key("far: 3.1 m off the street > 1.5") == "far-off-street"
    assert identity_rule_key("9.2 m wide > 8") == "width-max"
    assert identity_rule_key("0.62 m wide < 1") == "width-min"
    assert identity_rule_key("no range") == "no-range"
    assert identity_rule_key("far: aspect 0.90 < 1.3") == "far-aspect"


def test_score_estimate():
    rows = [{"est_xy": [0.0, 0.0], "tgt_x": 1.0, "tgt_y": 0.0},
            {"est_xy": [0.0, 0.0], "tgt_x": 30.0, "tgt_y": 0.0},
            {"est_xy": None, "tgt_x": 1.0, "tgt_y": 0.0},
            {"est_xy": [0.0, 0.0]}]
    s = track_truth.score_estimate(rows)
    assert s["ticks"] == 2 and s["on_subject_frac"] == 0.5
    assert track_truth.score_estimate([{"est_xy": None}]) is None


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
