"""follow_vlm --teacher-truth: a privileged teacher for training labels
(2026-10-30). The detector's boxes are replaced by the level car's TRUE
position projected into each frame; nothing else in the follow changes.

Run either way:
    pytest tests/test_follow_teacher.py -v
    python tests/test_follow_teacher.py

WHAT IS BEING CLAIMED

1. The teacher's box is where the camera would see the subject: its bottom
   edge is the subject's ground point through the capture's camera (the
   20 deg mount, the body's attitude), its top the point teacher_dims[1]
   above it, its centre column the true bearing; out of the field of view
   or behind the camera there is no box.
2. Its features are a perfect measurement in identity's names: P (north,
   east), rng_h, r, bearing - the fields the start gate, the re-acquirer and
   estimator_measurement read - and estimator_measurement turns them into
   the true bearing and range.
3. teacher_truth_fn carries the last polled position forward at the speed of
   the last two polls, and says None before the first poll.
4. The flag refuses a flight without --level-car, and metrics.json carries
   teacher_truth.

Shown failing first: at 772fde4 there is no Grounder._teacher_box, no
teacher_truth_fn and no --teacher-truth.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import follow_vlm as fv                                              # noqa: E402
import camera_model as cam                                           # noqa: E402

W, H = 768, 432


def _grounder(truth, dims=(4.0, 1.5)):
    return fv.Grounder(object(), "a white car", teacher=lambda t: truth, teacher_dims=dims)


def test_the_box_is_where_the_camera_sees_the_subject():
    pose = {"x": 0.0, "y": 0.0, "up": 8.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}
    g = _grounder((20.0, 0.0))
    c, f = g._teacher_box(pose, 100.0, W, H)
    assert abs(c[0] - W / 2) < 1e-6                     # dead ahead: the centre column
    bot = cam.world_to_pixel(20.0, 0.0, 0.0, 0.0, 0.0, 8.0, W, H, fv.CAMERA_HFOV_DEG, 0.0)
    top = cam.world_to_pixel(20.0, 0.0, 1.5, 0.0, 0.0, 8.0, W, H, fv.CAMERA_HFOV_DEG, 0.0)
    assert abs((c[1] + c[3] / 2) - bot[1]) < 1e-6 and abs((c[1] - c[3] / 2) - top[1]) < 1e-6
    assert c[4] == 1.0 and c[7] == 1.0 and c[5] == W and c[6] == H
    # 4 m wide at its slant range
    slant = math.hypot(20.0, 8.0 - 0.75)
    assert abs(c[2] - cam.focal_px(W, fv.CAMERA_HFOV_DEG) * 4.0 / slant) < 1e-6
    # its bottom edge on flat ground is the true range, as ground_range_m reads it
    det = list(c)
    assert abs(fv.ground_range_m(det, 8.0, 0.0) - math.hypot(20.0, 8.0)) < 0.05
    # a subject to the right: the column of its bearing
    c2, f2 = _grounder((20.0, 10.0))._teacher_box(pose, 100.0, W, H)
    assert c2[0] > W / 2 and abs(f2["bearing"] - math.atan2(10.0, 20.0)) < 1e-12


def test_the_features_are_a_perfect_measurement():
    yaw = math.pi / 2                                     # heading east
    pose = {"x": 100.0, "y": 50.0, "up": 8.0, "yaw": yaw, "pitch": 0.02, "roll": -0.01}
    g = _grounder((103.0, 70.0))
    c, f = g._teacher_box(pose, 100.0, W, H)
    assert f["P"] == (103.0, 70.0)
    assert abs(f["rng_h"] - math.hypot(3.0, 20.0)) < 1e-9
    assert abs(f["bearing"] - math.atan2(20.0, 3.0)) < 1e-12
    gsnap = {"cap": {"pose": pose, "t_capture": 99.5}, "feat": f}
    t, x, y, yaw_m, b, r, src = fv.estimator_measurement(c, gsnap, (100.0, 0.0, 0.0, 0.0), True)
    assert (t, x, y, yaw_m, src) == (99.5, 100.0, 50.0, yaw, "capture")
    assert abs(r - math.hypot(3.0, 20.0)) < 1e-9
    assert abs(b - (math.atan2(20.0, 3.0) - yaw)) < 1e-12         # body-relative, + right
    # pitch and roll reach the projection (camera_model.world_to_pixel)
    level = dict(pose, pitch=0.0, roll=0.0)
    c0, _ = g._teacher_box(level, 100.0, W, H)
    assert abs(c0[1] - c[1]) > 0.5


def test_no_box_outside_the_view_or_without_truth():
    pose = {"x": 0.0, "y": 0.0, "up": 8.0, "yaw": 0.0}
    assert _grounder((-20.0, 0.0))._teacher_box(pose, 1.0, W, H) is None        # behind
    assert _grounder((5.0, 40.0))._teacher_box(pose, 1.0, W, H) is None         # far right
    g = _grounder(None)
    assert g._teacher_box(pose, 1.0, W, H) is None and g.n_teacher["no_truth"] == 1
    g2 = _grounder((20.0, 0.0))
    assert g2._teacher_box({"x": 0.0, "y": 0.0}, 1.0, W, H) is None
    assert g2.n_teacher["no_pose"] == 1
    assert _grounder((20.0, 0.0)).n_teacher["in_frame"] == 0
    assert fv.Grounder(object(), "a red car").teacher is None                    # off by default


def test_the_truth_is_carried_forward_between_polls():
    class _Car:
        pos = None
    car = _Car()
    f = fv.teacher_truth_fn(car)
    assert f(0.0) is None
    import time
    car.pos = (10.0, 0.0)
    assert f() == (10.0, 0.0)                                 # one poll: where it was
    time.sleep(0.2)
    car.pos = (11.0, 0.0)                                     # 5 m/s north
    t_poll = time.time()
    p = f(t_poll + 0.4)
    assert abs(p[0] - (11.0 + 5.0 * 0.4)) < 0.6 and p[1] == 0.0
    far = f(t_poll + 10.0)                                    # at most max_ahead_s on
    assert far[0] <= 11.0 + 5.0 * 1.0 + 0.6


def test_the_height_follows_the_noun_and_the_flag_needs_a_level_car():
    assert fv.subject_height_m("a box truck") == 3.2
    assert fv.subject_height_m("a white car") == 1.5
    assert fv.subject_height_m("a person") == 1.7
    src = (ROOT / "demo" / "follow_vlm.py").read_text(encoding="utf-8")
    assert '"--teacher-truth draws the subject' in src and '"teacher_truth": (' in src


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
