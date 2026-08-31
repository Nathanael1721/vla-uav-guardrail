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
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from track_truth import (HFOV_DEG, ON_TARGET_PX,          # noqa: E402
                         project_target_cx, score_rows)

W = 400


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
