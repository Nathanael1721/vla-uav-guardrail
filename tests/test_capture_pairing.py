"""A detection is paired with the pose and the depth OF ITS OWN FRAME.

demo/semantic_demo.py SemanticObs: put_pose_full / pose_at / get_front_capture.
Until 2026-09-29 a box was combined with the pose read on the tick that
consumed it and with whichever depth frame had arrived last - 0.1-0.3 s and up
to a whole depth frame away from the image the box was found in.

Run either way:
    pytest tests/test_capture_pairing.py -v
    python tests/test_capture_pairing.py
"""
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "demo"))

from semantic_demo import SemanticObs  # noqa: E402


def _front(stamp, w=16, h=9):
    img = np.zeros((h, w, 3), np.uint8)
    return {"data": img.tobytes(), "width": w, "height": h, "encoding": "BGR",
            "time_stamp": stamp}


def _depth(stamp, metres, w=16, h=9):
    d = np.full((h, w), metres, np.uint16)
    return {"data": d.tobytes(), "width": w, "height": h, "encoding": "16UC1",
            "time_stamp": stamp}


def test_pose_is_interpolated_between_ticks_and_yaw_takes_the_short_way():
    o = SemanticObs()
    o.put_pose_full(10.0, 0.0, 0.0, 8.0, math.radians(170))
    o.put_pose_full(10.1, 1.0, 2.0, 8.0, math.radians(-170))
    p = o.pose_at(10.05)
    assert abs(p["x"] - 0.5) < 1e-9 and abs(p["y"] - 1.0) < 1e-9
    assert abs(abs(math.degrees(p["yaw"])) - 180.0) < 1e-6      # not 0 deg
    assert p["src"] == "ring"


def test_outside_the_ring_is_the_nearest_end_and_says_so():
    o = SemanticObs()
    assert o.pose_at(1.0) is None
    o.put_pose_full(10.0, 0.0, 0.0, 8.0, 0.0)
    o.put_pose_full(10.1, 1.0, 0.0, 8.0, 0.0)
    assert o.pose_at(9.0)["x"] == 0.0 and o.pose_at(9.0)["src"] == "ring-edge"
    assert o.pose_at(11.0)["x"] == 1.0


def test_sim_stamps_win_over_arrival_time_when_both_sides_have_them():
    o = SemanticObs()
    # wall times say 0.5 of the way; sim stamps say a quarter
    o.put_pose_full(10.0, 0.0, 0.0, 8.0, 0.0, stamp=1_000_000_000)
    o.put_pose_full(10.1, 4.0, 0.0, 8.0, 0.0, stamp=1_100_000_000)
    p = o.pose_at(10.05, stamp=1_025_000_000)
    assert abs(p["x"] - 1.0) < 1e-9 and p["src"] == "ring-stamp"


def test_the_depth_frame_nearest_the_image_is_used_not_the_latest():
    o = SemanticObs()
    o.put_pose_full(0.0, 0.0, 0.0, 8.0, 0.0, stamp=0)
    o.put_depth(_depth(1_000_000_000, 20))
    o.put_depth(_depth(1_050_000_000, 30))      # nearest the image below
    o.put_depth(_depth(1_300_000_000, 40))      # the latest
    o.put_front(_front(1_060_000_000))
    img, meta = o.get_front_capture()
    assert img is not None and img.size == (16, 9)
    assert float(np.median(meta["depth"])) == 30.0
    assert meta["depth_dt_ms"] == 10.0
    assert float(np.median(o.get_depth())) == 40.0              # unchanged API


def test_no_depth_when_the_nearest_is_too_far_in_time():
    o = SemanticObs()
    o.put_depth(_depth(1_000_000_000, 20))
    o.put_front(_front(1_200_000_000))
    _img, meta = o.get_front_capture()
    assert meta["depth"] is None and meta["depth_dt_ms"] == 200.0


def test_capture_meta_carries_the_pose_at_the_frame():
    o = SemanticObs()
    o.put_pose_full(5.0, 0.0, 0.0, 8.0, 0.0, stamp=1_000_000_000)
    o.put_pose_full(5.1, 2.0, 0.0, 8.0, 0.0, stamp=1_100_000_000)
    o.put_front(_front(1_050_000_000))
    _img, meta = o.get_front_capture()
    assert abs(meta["pose"]["x"] - 1.0) < 1e-9
    assert meta["t_cap"] is not None


def test_capture_time_comes_from_the_sim_stamp_not_the_arrival():
    """A frame captured at sim 1.05 s arrives later: its measurement time is
    the wall time the kinematics had sim 1.05 s, not the arrival time."""
    t0 = time.time() - 0.3
    o = SemanticObs()
    o.put_pose_full(t0, 0.0, 0.0, 8.0, 0.0, stamp=1_000_000_000)
    o.put_pose_full(t0 + 0.1, 1.0, 0.0, 8.0, 0.0, stamp=1_100_000_000)
    o.put_front(_front(1_050_000_000))              # arrives now, ~0.25 s later
    _img, meta = o.get_front_capture()
    assert abs(meta["t_capture"] - (t0 + 0.05)) < 1e-6
    assert meta["t_cap"] - meta["t_capture"] > 0.2
    # past the ring's end: extrapolated 1:1 from the newest entry
    assert abs(o.wall_at_stamp(1_150_000_000) - (t0 + 0.15)) < 1e-6
    # no stamps: the arrival time
    o2 = SemanticObs()
    o2.put_pose_full(5.0, 0.0, 0.0, 8.0, 0.0)
    o2.put_front(_front(None))
    _i, m2 = o2.get_front_capture()
    assert m2["t_capture"] == m2["t_cap"]


def test_depth_can_be_paired_again_after_it_arrives():
    o = SemanticObs()
    o.put_depth(_depth(1_000_000_000, 20))
    o.put_front(_front(1_200_000_000))
    _img, meta = o.get_front_capture()
    assert meta["depth"] is None                    # nothing within 70 ms yet
    o.put_depth(_depth(1_190_000_000, 33))          # its own frame, late
    d, dt_ms = o.depth_near(meta["stamp"], meta["t_cap"])
    assert d is not None and float(np.median(d)) == 33.0 and dt_ms == 10.0
    assert SemanticObs().depth_near(1, 1.0) == (None, None)


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
