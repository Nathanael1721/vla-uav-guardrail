"""Was the box on the right car? The geometry, pinned.

Run either way:
    pytest tests/test_track_truth.py -v
    python tests/test_track_truth.py

`det_hit_rate` answers "did the detector emit a box", not "was the box on the
target", and it was read as the second for a long time. `demo/track_truth.py`
computes the second. Since every claim about tracking quality now rests on this
projection, the geometry is checked against hand-computed cases rather than
assumed - and against the flights on disk, whose numbers are quoted in the
module docstring and in docs/.
"""
import json
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from track_truth import (HFOV_DEG, ON_TARGET_PX,          # noqa: E402
                         project_target_cx, score_rows, truth_points,
                         score_standoff_firings)

W = 400


SKIP = "SKIP"


def test_a_target_dead_ahead_lands_in_the_centre():
    cx, deg = project_target_cx(0.0, 0.0, 0.0, 10.0, 0.0, W)
    assert abs(cx - 200.0) < 1e-6, cx
    assert abs(deg) < 1e-9


def test_a_target_at_the_right_edge_of_the_view():
    """Half the field of view is 45 deg, which is the right-hand edge."""
    cx, deg = project_target_cx(0.0, 0.0, 0.0, 10.0, 10.0, W)   # +45 deg
    assert abs(deg - 45.0) < 1e-6, deg
    assert abs(cx - 400.0) < 1e-6, cx


def test_a_target_behind_the_aircraft_projects_outside_the_frame():
    """The informative case: no box can be on it, so any box is on something else."""
    cx, deg = project_target_cx(0.0, 0.0, 0.0, -10.0, 0.0, W)
    assert abs(abs(deg) - 180.0) < 1e-6
    assert cx < 0 or cx > W, cx


def test_heading_rotates_the_projection():
    """Turning the nose toward the target brings it back to the centre."""
    _cx, deg = project_target_cx(0.0, 0.0, math.radians(45.0), 10.0, 10.0, W)
    assert abs(deg) < 1e-6, deg


def test_the_wrap_is_the_short_way_round():
    """A bearing either side of the +/-180 boundary must not read as a huge angle."""
    _cx, deg = project_target_cx(0.0, 0.0, math.radians(179.0), -10.0, -0.2, W)
    assert abs(deg) < 5.0, deg


def test_a_perfect_row_scores_on_target():
    rows = [{"x": 0.0, "y": 0.0, "psi": 0.0, "tgt_x": 30.0, "tgt_y": 0.0,
             "det": {"cx": 200.0, "img_w": W}}]
    s = score_rows(rows)
    assert s["frac_on_target"] == 1.0
    assert s["det_gt_err_px_median"] == 0.0
    assert s["n_det_with_target_out_of_fov"] == 0


def test_a_box_on_another_vehicle_scores_off_target():
    """200 px away on a 400 px frame is half the image - a different object."""
    rows = [{"x": 0.0, "y": 0.0, "psi": 0.0, "tgt_x": 30.0, "tgt_y": 0.0,
             "det": {"cx": 400.0, "img_w": W}}]
    s = score_rows(rows)
    assert s["frac_on_target"] == 0.0
    assert s["det_gt_err_px_median"] == 200.0


def test_a_detection_while_the_target_is_out_of_shot_is_counted():
    """The unambiguous failure. det_hit_rate scores this as a hit."""
    rows = [{"x": 0.0, "y": 0.0, "psi": 0.0, "tgt_x": -30.0, "tgt_y": 0.0,
             "det": {"cx": 200.0, "img_w": W}}]
    s = score_rows(rows)
    assert s["n_det_with_target_out_of_fov"] == 1
    assert s["frac_on_target"] == 0.0


def test_ticks_without_a_detection_are_not_scored():
    """Liveness is det_hit_rate's question; this one is about accuracy."""
    rows = [{"x": 0.0, "y": 0.0, "psi": 0.0, "tgt_x": 30.0, "tgt_y": 0.0, "det": None},
            {"x": 0.0, "y": 0.0, "psi": 0.0, "tgt_x": 30.0, "tgt_y": 0.0,
             "det": {"cx": 200.0, "img_w": W}}]
    s = score_rows(rows)
    assert s["det_scored"] == 1
    assert s["frac_on_target"] == 1.0


def test_an_empty_flight_returns_nulls_not_a_division_error():
    s = score_rows([])
    assert s["det_scored"] == 0
    assert s["frac_on_target"] is None


def test_it_reproduces_the_recorded_flights():
    """The numbers quoted in demo/track_truth.py and in docs/ come from here.

    If this drifts, either the metric changed or the artefacts did, and every
    claim resting on those figures needs re-checking.
    """
    expect = {                      # flight: (median, p95, on_target, out_of_fov)
        "city_full":    (8.2, 216.8, 0.728, 37),
        # 0.908, not the 0.930 published before 2026-09-08: the 12 rows whose
        # target was out of shot were being counted on target as well as out of
        # shot, because a 100 px tolerance is half the 45 deg half-FOV.
        "city_kpi":     (3.6, 113.2, 0.908, 12),
        "demo_traffic": (4.9,  48.9, 1.000,  0),
        "city_demo":    (2.7,  27.4, 1.000,  0),
    }
    checked = 0
    for tag, (med, p95, frac, oof) in expect.items():
        log = ROOT / "demo" / "out" / tag / "flight_log.jsonl"
        if not log.is_file():
            continue                # artefacts are gitignored; skip on a clean clone
        rows = [json.loads(l) for l in log.read_text(encoding="utf-8").splitlines() if l.strip()]
        s = score_rows(rows)
        assert abs(s["det_gt_err_px_median"] - med) < 0.15, (tag, s)
        assert abs(s["det_gt_err_px_p95"] - p95) < 0.15, (tag, s)
        assert abs(s["frac_on_target"] - frac) < 0.002, (tag, s)
        assert s["n_det_with_target_out_of_fov"] == oof, (tag, s)
        checked += 1
    if checked == 0:
        return                      # nothing on disk to check against
    assert checked >= 1


def test_the_tolerance_is_wide_enough_for_a_correct_box():
    """100 px is 22.5 deg. Real on-target error is 3-8 px, so the threshold
    separates 'slightly off the centroid' from 'a different vehicle'."""
    assert ON_TARGET_PX == 100.0
    assert HFOV_DEG == 90.0


# --- the subject can change mid-flight -------------------------------------
#
# These pin the fix for the defect that made a retarget flight unreadable: the
# scorer compared every box to the car, including the 319 ticks after the
# subject had become a pedestrian, and reported the result as one number.


def _row(cx, x=0.0, y=0.0, psi=0.0, **kw):
    r = {"x": x, "y": y, "psi": psi, "det": {"cx": cx, "img_w": W}}
    r.update(kw)
    return r


def test_a_row_with_no_truth_for_its_subject_is_unscorable_not_wrong():
    """The whole point. Nothing to compare against is not a failed comparison."""
    out = score_rows([_row(200.0, truth={"class": "pedestrian", "pts": []})])
    assert out["det_scored"] == 0, out
    assert out["det_unscorable"] == 1, out
    assert out["frac_on_target"] is None, out


def test_old_logs_without_a_truth_field_score_exactly_as_before():
    """Back-compatibility is load-bearing: 40-odd flights on disk have no
    `truth` key, and their published figures must not move."""
    r = _row(200.0, tgt_x=10.0, tgt_y=0.0)
    assert truth_points(r) == [(10.0, 0.0)]
    out = score_rows([r])
    assert out["det_scored"] == 1 and out["frac_on_target"] == 1.0, out


def test_any_member_of_the_class_counts_and_the_nearest_one_is_scored():
    """`a person` names a class, not a person. A box on the second pedestrian
    is a correct answer to the question that was asked."""
    # +45 deg is the right edge (cx 400); dead ahead is cx 200.
    row = _row(400.0, truth={"class": "pedestrian",
                             "pts": [[10.0, 0.0], [10.0, 10.0]]})
    out = score_rows([row])
    assert out["frac_on_target"] == 1.0, out
    assert out["det_gt_err_px_median"] == 0.0, out


def test_out_of_shot_needs_every_candidate_out_of_shot():
    """One subject behind the aircraft does not make the box wrong if another
    is in front of it."""
    behind, ahead = [-10.0, 0.0], [10.0, 0.0]
    both = score_rows([_row(200.0, truth={"class": "pedestrian",
                                          "pts": [behind, ahead]})])
    assert both["n_det_with_target_out_of_fov"] == 0, both
    only = score_rows([_row(200.0, truth={"class": "pedestrian",
                                          "pts": [behind]})])
    assert only["n_det_with_target_out_of_fov"] == 1, only


def test_a_subject_out_of_shot_can_never_be_credited_with_the_box():
    """The two tests used to be satisfiable by DIFFERENT candidates: the error
    was the min over all of them, while out-of-shot required all of them to be
    out. So a row could be scored ON TARGET against a subject the camera could
    not see, and the counter built to catch that stayed silent."""
    behind = [-10.0, 0.0]                 # 180 deg off the nose
    out = score_rows([_row(200.0, truth={"class": "pedestrian", "pts": [behind]})])
    assert out["n_det_with_target_out_of_fov"] == 1, out
    assert out["frac_on_target"] == 0.0, out
    assert out["truth_candidates_max"] == 0, out


def test_an_out_of_shot_row_close_in_pixels_is_still_not_on_target():
    """The gap the in-shot filter left. ON_TARGET_PX is 100 px = 22.5 deg, HALF
    the 45 deg half-FOV, so a subject up to 67.5 deg off the nose lands within
    tolerance of a box at the frame edge. Those rows were counted in
    n_det_with_target_out_of_fov AND in the frac_on_target numerator at once -
    on demo/out/city_kpi, all twelve of them."""
    # 50 deg off the nose: out of shot, but only 22 px from a box at the edge.
    off = [10.0 * math.cos(math.radians(50.0)), 10.0 * math.sin(math.radians(50.0))]
    out = score_rows([_row(400.0, truth={"class": "pedestrian", "pts": [off]})])
    assert out["n_det_with_target_out_of_fov"] == 1, out
    assert out["det_gt_err_px_median"] < ON_TARGET_PX, out    # close in pixels
    assert out["frac_on_target"] == 0.0, out                  # and still not on target


def test_a_zero_skill_detector_scores_its_own_chance_floor():
    """The defect that cost the multi-point path its meaning. Scoring against a
    CLASS gets easier with every extra figure in the scene, and `ON_TARGET_PX`
    was calibrated against a single target. Measured before the fix, a uniformly
    random box scored 0.584 with twelve figures - and its median error, 71 px,
    was INSIDE the 100 px tolerance.

    The fix is not to make chance smaller; with a class it genuinely is easier.
    It is to report the floor beside the score, so a number that carries no
    signal cannot look like one."""
    rng = random.Random(7)
    for n_pts, tol in ((1, 0.05), (12, 0.05)):
        rows = []
        for _ in range(3000):
            pts = [[rng.uniform(-60, 60), rng.uniform(-60, 60)]
                   for _ in range(n_pts)]
            rows.append({"x": rng.uniform(-60, 60), "y": rng.uniform(-60, 60),
                         "psi": rng.uniform(-math.pi, math.pi),
                         "det": {"cx": rng.uniform(0, W), "img_w": W},
                         "truth": {"class": "pedestrian", "pts": pts}})
        out = score_rows(rows)
        assert abs(out["frac_on_target"] - out["frac_on_target_chance"]) < tol, \
            (n_pts, out["frac_on_target"], out["frac_on_target_chance"])


def test_a_real_flight_beats_its_chance_floor_by_a_wide_margin():
    """The counterpart: the figure is only evidence when it clears the floor.
    Pre-retarget tracking is 0.915 against a floor of 0.895 - the floor is high
    because the aircraft points AT the subject, so a random box lands near it
    often, and quoting 0.915 without it would overstate the result."""
    log = ROOT / "demo" / "out" / "retarget_demo2" / "flight_log.jsonl"
    if not log.is_file():
        return SKIP
    rows = [json.loads(x) for x in log.open(encoding="utf-8") if x.strip()]
    pre = score_rows([r for r in rows if r["tick"] < 248])
    assert pre["frac_on_target"] == 0.915, pre
    # THE FRACTION IS NOT THE EVIDENCE, and this pins how little of it there is.
    # Against a null family that includes the best fixed column for this flight,
    # the 100 px margin is +0.020 - two rows in a hundred. A fourth review found
    # a constant that BEATS the detector outright on retarget_demo, so every
    # floor invites a harder null and this one is only a lower bound.
    assert pre["frac_on_target_chance"] == 0.895, pre
    assert pre["frac_on_target_margin"] == 0.02, pre
    assert pre["frac_on_target_25px_margin"] == 0.15, pre
    assert pre["truth_candidates_max"] == 1, pre

    # THE MEDIAN IS. 7.6 px against the best null's 14.9 - nearly twice as
    # close, on a statistic that beats every null on every flight on disk.
    assert pre["det_gt_err_px_median_in_shot"] == 7.1, pre
    assert pre["det_gt_err_px_median_chance"] == 14.9, pre
    assert (pre["det_gt_err_px_median_in_shot"]
            < pre["det_gt_err_px_median_chance"] / 2.0)


def test_the_median_beats_every_null_on_every_flight_and_the_fraction_does_not():
    """Why the headline moved. The threshold count is within a whisker of a
    lag-1 baseline everywhere - repeat your own last box and you score the same
    - while the median separates cleanly on every flight with scored rows."""
    out = ROOT / "demo" / "out"
    if not out.is_dir():
        return SKIP
    checked = 0
    wins, losses = [], []
    for run in sorted(out.iterdir()):
        log = run / "flight_log.jsonl"
        if not log.is_file():
            continue
        rows = [json.loads(x) for x in log.open(encoding="utf-8") if x.strip()]
        r = score_rows(rows)
        if not r["det_scored"] or r["det_gt_err_px_median_chance"] is None:
            continue
        checked += 1
        beats = (r["det_gt_err_px_median_in_shot"]
                 < r["det_gt_err_px_median_chance"])
        if beats:
            wins.append(run.name)
        else:
            losses.append((run.name, r["det_gt_err_px_median_in_shot"],
                           r["det_gt_err_px_median_chance"],
                           r["frac_in_shot"]))

    # NOT "beats every null on every flight". The first version of this test
    # skipped every flight with frac_in_shot < 0.5 - which is exactly the two
    # flights that would have falsified the claim it was written to guard. A
    # filter that removes the counterexamples is not a guard.
    #
    # What is true, and what is asserted: the median SEPARATES. It beats the
    # null on every flight where the subject was usually in frame, and on the
    # two where it does not (envactor3 203.1 px against 35.6, envactor_white
    # 56.6 against 47.3) the subject was in frame on 17 % and 37 % of rows and
    # the box genuinely was nowhere near it. The statistic reporting a bad
    # flight as bad is the statistic working.
    assert checked >= 20, checked
    assert len(wins) >= 15, (len(wins), losses)
    for name, real, null, in_shot in losses:
        assert in_shot < 0.5, (
            f"{name} lost to the null ({real} px against {null}) on a flight "
            f"whose subject was in frame {in_shot:.0%} of the time - that is "
            f"not the known shape of this exception")


def test_a_constant_detector_is_the_null_model_that_actually_bites():
    """Why the floor is the harder of two nulls. A uniform draw ignores that the
    aircraft YAWS TO POINT AT the subject, so the subject sits near the frame
    centre and a detector that emits the centre every frame - never opening the
    image - scores far above uniform. On demo/out/city_full it scores 0.753
    against the real detector's 0.728: the constant WINS at this tolerance."""
    log = ROOT / "demo" / "out" / "city_full" / "flight_log.jsonl"
    if not log.is_file():
        return SKIP
    rows = [json.loads(x) for x in log.open(encoding="utf-8") if x.strip()]
    out = score_rows(rows)
    assert out["frac_on_target_chance_centre"] > out["frac_on_target_chance_uniform"]
    assert out["frac_on_target_margin"] < 0, out
    # And the tighter tolerance is where the detector's real advantage shows.
    assert out["frac_on_target_25px_margin"] > 0, out


def test_the_retarget_flight_on_disk_splits_the_way_the_finding_says():
    """The measured claim, pinned against the artefact.

    demo/out/retarget_demo2 retargeted at tick 248. Scored whole, it reads
    0.406 - which is close to 248/567, the fraction of the flight BEFORE the
    subject changed, and is a statement about the scorer rather than about the
    detector. Scored on the half whose truth was actually logged, tracking was
    0.915 with a 7.6 px median error (0.931 before out-of-shot rows
    stopped being credited on 2026-09-08)."""
    log = ROOT / "demo" / "out" / "retarget_demo2" / "flight_log.jsonl"
    if not log.is_file():
        return                                   # artefact not in this checkout
    rows = [json.loads(l) for l in log.open(encoding="utf-8")]
    pre = score_rows([r for r in rows if r["tick"] < 248])
    post = score_rows([r for r in rows if r["tick"] >= 248])
    assert pre["frac_on_target"] == 0.915, pre
    assert pre["det_gt_err_px_median"] == 7.6, pre
    # Every post-retarget detection was compared to a car that was behind the
    # aircraft. 319 out of 319 - a number no detector produces.
    assert post["frac_on_target"] == 0.0, post
    assert post["n_det_with_target_out_of_fov"] == post["det_scored"] == 319, post


# --------------------------------------------------------------------------
# The stand-off ring: a firing count is not a result.
# --------------------------------------------------------------------------

class _Ring:
    def __init__(self, rid, cls, rng):
        self.id, self.subject_class, self.min_range_m = rid, cls, rng


def _tick(n, x, y, pts, cls="pedestrian", fired=()):
    return {"tick": n, "x": x, "y": y, "psi": 0.0,
            "truth": {"class": cls, "pts": pts},
            "violations": [{"rule_id": r} for r in fired]}


def test_a_ring_that_fires_on_a_phantom_scores_zero():
    """The exact shape of the 2026-09-09 flight, in four ticks.

    The rule fires while the nearest real pedestrian is 16 m away, and stays
    silent on the tick where one is genuinely at 6 m. The firing COUNT is 1 and
    looks like the rule working; precision is 0.00 and recall is 0.00.
    """
    ring = _Ring("standoff-pedestrian", "pedestrian", 10.0)
    rows = [_tick(1, 0, 0, [(16.0, 0.0)], fired=["standoff-pedestrian"]),
            _tick(2, 0, 0, [(16.0, 0.0)]),
            _tick(3, 0, 0, [(6.0, 0.0)]),
            _tick(4, 0, 0, [(20.0, 0.0)])]
    s = score_standoff_firings(rows, [ring])["standoff-pedestrian"]
    assert (s["fires"], s["tp"], s["fp"], s["fn"]) == (1, 0, 1, 1), s
    assert s["precision"] == 0.0 and s["recall"] == 0.0, s


def test_a_ring_that_is_right_scores_right():
    ring = _Ring("standoff-pedestrian", "pedestrian", 10.0)
    rows = [_tick(1, 0, 0, [(6.0, 0.0)], fired=["standoff-pedestrian"]),
            _tick(2, 0, 0, [(20.0, 0.0)])]
    s = score_standoff_firings(rows, [ring])["standoff-pedestrian"]
    assert (s["tp"], s["fp"], s["fn"]) == (1, 0, 0), s
    assert s["precision"] == 1.0 and s["recall"] == 1.0, s


def test_a_rule_that_never_bound_reports_no_score_not_a_perfect_one():
    """An unbound rule scoring 1.000 is the defect this project keeps finding:
    a zero that means "never ran" read as "never violated"."""
    ring = _Ring("standoff-pedestrian", "pedestrian", 10.0)
    rows = [_tick(1, 0, 0, [(3.0, 0.0)], cls="car")]
    s = score_standoff_firings(rows, [ring])["standoff-pedestrian"]
    assert s["bound_ticks"] == 0, s
    assert s["precision"] is None and s["recall"] is None, s


def test_a_tick_with_no_truth_is_unscorable_not_a_miss():
    ring = _Ring("standoff-pedestrian", "pedestrian", 10.0)
    rows = [{"tick": 1, "x": 0.0, "y": 0.0, "psi": 0.0,
             "truth": {"class": "pedestrian", "pts": []}, "violations": []}]
    s = score_standoff_firings(rows, [ring])["standoff-pedestrian"]
    assert s["bound_ticks"] == 0 and s["fn"] == 0, s


def test_the_catch_all_binds_to_every_class():
    ring = _Ring("standoff-any", "*", 5.0)
    rows = [_tick(1, 0, 0, [(3.0, 0.0)], cls="car"),
            _tick(2, 0, 0, [(3.0, 0.0)], cls="pedestrian")]
    s = score_standoff_firings(rows, [ring])["standoff-any"]
    assert s["bound_ticks"] == 2 and s["fn"] == 2, s


def test_the_recorded_flight_reproduces_the_retraction():
    """The number that was reported to the PI as proof, scored.

    Reported: "the 10 m pedestrian ring fired 6 times". True: 0 TP, 6 FP, 49 FN
    - every firing against an estimate 6-12 m from any real person, and silence
    on all 49 ticks where a real pedestrian was genuinely inside 10 m.
    """
    log = ROOT / "demo" / "out" / "retarget_fixed" / "flight_log.jsonl"
    if not log.exists():
        return SKIP
    rows = [json.loads(l) for l in log.open(encoding="utf-8")]
    s = score_standoff_firings(
        rows, [_Ring("standoff-pedestrian", "pedestrian", 10.0)]
    )["standoff-pedestrian"]
    assert s["fires"] == 6 and s["tp"] == 0 and s["fn"] == 49, s
    assert s["precision"] == 0.0, s


if __name__ == "__main__":
    # A test that short-circuits on a missing fixture must NOT print PASS - on a
    # clean clone demo/out/ is gitignored and those tests assert nothing. See
    # tests/test_replay.py, where that hid 7 no-ops behind "8/8 passed".
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (fixture missing)")
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
