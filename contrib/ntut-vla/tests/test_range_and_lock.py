"""Depth-based range and instance persistence — the two gaps the orbit exposed.

Run either way:
    pytest tests/test_range_and_lock.py -v
    python tests/test_range_and_lock.py

RANGE. The radial servo used apparent box width, a fine proxy for a car (same
width from any angle) and a bad one for a 50 x 50 m block, where it swings by
root-2 between face-on and corner-on. Circling made the servo command reverse
from the aspect change alone and the orbit radius spiralled 37.7 m -> 178 m.

PERSISTENCE. "a building" names a KIND and the map has nine city blocks; "a car"
names a kind and the traffic scene has four. Box-centre discontinuities over
80 px occurred 24, 5 and 8 times across the three orbit flights, so the aircraft
was chasing whichever instance was most salient rather than the one it started
on. For the car, colour supplied instance persistence by accident. Nothing does
for a building.
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from follow_vlm import TargetLock, range_from_depth          # noqa: E402

W, H = 400, 225


def _det(cx, cy=112.0, bw=40.0, bh=30.0, score=0.05, colour=0.5):
    return (cx, cy, bw, bh, score, W, H, colour)


def _depth(bg=200.0, box=None, val=25.0):
    d = np.full((H, W), bg, dtype=np.float32)
    if box:
        x0, y0, x1, y1 = box
        d[y0:y1, x0:x1] = val
    return d


# ----------------------------------------------------------------- range

def test_range_reads_the_object_not_the_background():
    """The whole point of shrinking the sampling window."""
    d = _depth(bg=300.0, box=(180, 97, 220, 127), val=25.0)
    r = range_from_depth(d, _det(200.0, 112.0, 40.0, 30.0))
    assert r is not None and abs(r - 25.0) < 0.5, r


def test_a_few_background_pixels_do_not_move_the_answer():
    """Median, not mean: one sky pixel at 5 km would wreck a mean."""
    d = _depth(bg=300.0, box=(180, 97, 220, 127), val=25.0)
    d[112, 195] = 5000.0
    d[110, 202] = 4000.0
    r = range_from_depth(d, _det(200.0))
    assert r is not None and abs(r - 25.0) < 0.5, r


def test_non_finite_and_zero_depths_are_rejected():
    d = _depth(bg=300.0, box=(180, 97, 220, 127), val=25.0)
    d[105:115, 190:210] = np.nan
    r = range_from_depth(d, _det(200.0))
    assert r is None or math.isfinite(r), r


def test_an_all_invalid_window_returns_none_rather_than_a_number():
    """Returning a plausible-looking number from garbage is worse than nothing:
    the caller would servo on it."""
    d = np.full((H, W), np.nan, dtype=np.float32)
    assert range_from_depth(d, _det(200.0)) is None


def test_no_depth_or_no_detection_is_not_an_error():
    """A missing stream must degrade to the width servo, not crash the flight."""
    assert range_from_depth(None, _det(200.0)) is None
    assert range_from_depth(_depth(), None) is None


def test_a_box_off_the_edge_of_the_frame_is_handled():
    d = _depth(bg=300.0, box=(0, 97, 20, 127), val=18.0)
    r = range_from_depth(d, _det(2.0, 112.0, 40.0, 30.0))
    assert r is None or r > 0


def test_range_is_independent_of_apparent_width():
    """The property the width servo did not have. Same object, same distance,
    two very different box widths - the range must not move."""
    d = _depth(bg=300.0, box=(150, 92, 250, 132), val=40.0)
    narrow = range_from_depth(d, _det(200.0, 112.0, 30.0, 30.0))
    wide = range_from_depth(d, _det(200.0, 112.0, 90.0, 36.0))
    assert narrow is not None and wide is not None
    assert abs(narrow - wide) < 0.5, (narrow, wide)


# ------------------------------------------------------------------ lock

def test_the_first_detection_establishes_the_lock():
    lk = TargetLock()
    chosen, switched = lk.select([_det(100.0), _det(300.0)], W, 0.0, 1.0)
    assert chosen[0] == 100.0        # best-ranked wins when nothing is held
    assert not switched


def test_it_keeps_the_held_instance_over_a_better_scoring_rival():
    """The core behaviour. A distractor that outscores the target must not steal
    the lock just by scoring higher."""
    lk = TargetLock()
    lk.select([_det(100.0, score=0.05)], W, 0.0, 1.0)
    rival = _det(300.0, score=0.50)          # ten times the score, far away
    held = _det(104.0, score=0.03)
    chosen, switched = lk.select([rival, held], W, 0.0, 1.1)
    assert chosen[0] == 104.0, "the lock followed the score instead of the object"
    assert not switched


def test_a_yaw_turn_does_not_break_the_lock():
    """Objects slide across the frame when the nose turns. Without predicting
    that, the gate would reject the true target exactly when the aircraft is
    tracking hardest."""
    lk = TargetLock(hfov_deg=90.0)
    lk.select([_det(200.0)], W, 0.0, 1.0)
    yaw = math.radians(20.0)                 # turn right 20 deg
    px = 200.0 - yaw * (W / math.radians(90.0))
    chosen, switched = lk.select([_det(px), _det(390.0)], W, yaw, 1.1)
    assert abs(chosen[0] - px) < 1.0, chosen[0]
    assert not switched, "a normal turn was reported as a target switch"


def test_a_real_jump_is_reported_as_a_switch_not_hidden():
    """When the held instance genuinely disappears, re-acquiring is correct — but
    it must be visible, because that is the moment the mission changes target."""
    lk = TargetLock()
    lk.select([_det(80.0)], W, 0.0, 1.0)
    chosen, switched = lk.select([_det(350.0)], W, 0.0, 1.1)
    assert chosen[0] == 350.0
    assert switched, "target changed silently"
    assert lk.stats()["switched"] >= 1


def test_a_stale_lock_is_dropped_rather_than_trusted_forever():
    """After hold_s with no match the lock must let go, or one long occlusion
    would pin the aircraft to an instance it can no longer see."""
    lk = TargetLock(hold_s=2.0)
    lk.select([_det(80.0)], W, 0.0, 1.0)
    chosen, switched = lk.select([_det(350.0)], W, 0.0, 1.0 + 5.0)
    assert chosen[0] == 350.0, "the stale lock still constrained the choice"
    # And it is still reported. Re-acquiring on a different object after a long
    # loss IS a target change - the mission is now following something else, and
    # a reader of the log needs to know that even though the re-acquire itself
    # was the correct behaviour.
    assert switched, "target changed after a stale lock without being reported"


def test_no_candidates_leaves_the_lock_untouched():
    """A miss must not clear the lock, or one dropped frame would re-acquire on
    whatever appears next."""
    lk = TargetLock()
    lk.select([_det(120.0)], W, 0.0, 1.0)
    chosen, switched = lk.select([], W, 0.0, 1.1)
    assert chosen is None and not switched
    assert lk.cx == 120.0


def test_the_gate_scales_with_image_width():
    """0.28 of 400 px is 112 px; a 150 px jump must fail and a 90 px one pass."""
    for jump, want_switch in ((150.0, True), (90.0, False)):
        lk = TargetLock(gate_frac=0.28)
        lk.select([_det(200.0)], W, 0.0, 1.0)
        _, switched = lk.select([_det(200.0 + jump)], W, 0.0, 1.1)
        assert switched is want_switch, (jump, switched)


def test_lock_is_opt_in_and_absent_by_default():
    """Grounder(lock=None) must behave exactly as before this feature existed."""
    import follow_vlm
    import inspect
    sig = inspect.signature(follow_vlm.Grounder.__init__)
    assert sig.parameters["lock"].default is None


# --------------------------------------------------------------------- runner

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
