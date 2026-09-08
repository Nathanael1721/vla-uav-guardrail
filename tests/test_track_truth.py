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
                         project_target_cx, score_rows, truth_points)

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
        "city_kpi":     (3.6, 113.2, 0.930, 12),
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
    assert out["truth_candidates_median"] == 0, out


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
    Pre-retarget tracking is 0.931 against a floor of 0.437 - the floor is high
    because the aircraft points AT the subject, so a random box lands near it
    often, and quoting 0.931 without it would overstate the result."""
    log = ROOT / "demo" / "out" / "retarget_demo2" / "flight_log.jsonl"
    if not log.is_file():
        return SKIP
    rows = [json.loads(x) for x in log.open(encoding="utf-8") if x.strip()]
    pre = score_rows([r for r in rows if r["tick"] < 248])
    assert pre["frac_on_target"] == 0.931, pre
    assert pre["frac_on_target_chance"] == 0.437, pre
    assert pre["truth_candidates_median"] == 1, pre


def test_the_retarget_flight_on_disk_splits_the_way_the_finding_says():
    """The measured claim, pinned against the artefact.

    demo/out/retarget_demo2 retargeted at tick 248. Scored whole, it reads
    0.406 - which is close to 248/567, the fraction of the flight BEFORE the
    subject changed, and is a statement about the scorer rather than about the
    detector. Scored on the half whose truth was actually logged, tracking was
    0.931 with a 7.6 px median error."""
    log = ROOT / "demo" / "out" / "retarget_demo2" / "flight_log.jsonl"
    if not log.is_file():
        return                                   # artefact not in this checkout
    rows = [json.loads(l) for l in log.open(encoding="utf-8")]
    pre = score_rows([r for r in rows if r["tick"] < 248])
    post = score_rows([r for r in rows if r["tick"] >= 248])
    assert pre["frac_on_target"] == 0.931, pre
    assert pre["det_gt_err_px_median"] == 7.6, pre
    # Every post-retarget detection was compared to a car that was behind the
    # aircraft. 319 out of 319 - a number no detector produces.
    assert post["frac_on_target"] == 0.0, post
    assert post["n_det_with_target_out_of_fov"] == post["det_scored"] == 319, post


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
