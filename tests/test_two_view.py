"""The two-view layout, which the live window and the recorded video share.

Run either way:
    pytest tests/test_two_view.py -v
    python tests/test_two_view.py

`demo/two_view.py` exists because there were about to be two copies of this
layout: one in `tools/make_demo_video.py`, which stitches frames after the
flight, and one in `demo/recorder.py --live-view`, which shows the same pair
while the aircraft is still flying. Two copies drift, and the drift would only
surface when someone held a live screenshot next to the delivered video.

The cases below are the ones that actually occur in flight. A camera returning
nothing is normal - `get_chase_native()` yields None until the first message
arrives on its topic - and the recorder counts those as empty rather than
failing, so the layout has to survive one panel being absent.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

import numpy as np                                          # noqa: E402

from two_view import pil_to_bgr, side_by_side               # noqa: E402


def _img(h, w, v=128):
    return np.full((h, w, 3), v, dtype=np.uint8)


def test_two_panels_are_placed_side_by_side():
    out = side_by_side(_img(540, 960), _img(540, 960))
    assert out.shape == (540, 1920, 3), out.shape


def test_a_missing_panel_yields_the_other_rather_than_nothing():
    """A camera with no message yet must not cost the whole frame."""
    assert side_by_side(_img(540, 960), None).shape == (540, 960, 3)
    assert side_by_side(None, _img(540, 960)).shape == (540, 960, 3)


def test_both_missing_is_the_only_case_that_yields_none():
    assert side_by_side(None, None) is None


def test_mismatched_heights_are_reconciled_without_resampling_the_left():
    """The left panel carries the detector overlay.

    Rescaling it would resample the very box the demo is about, so the right
    panel is the one that moves.
    """
    left = _img(540, 960)
    out = side_by_side(left, _img(225, 400))
    assert out.shape[0] == 540, "left panel height must be preserved"
    assert out[:, :960].shape == left.shape
    assert np.array_equal(out[:, :960], left), "left panel was altered"


def test_height_argument_scales_both_panels():
    out = side_by_side(_img(1080, 1920), _img(720, 1280), height=540)
    assert out.shape[0] == 540
    assert out.shape[1] == 960 + 960


def test_pil_conversion_swaps_the_channel_order():
    """PIL is RGB, OpenCV is BGR. Getting this wrong turns the sky orange and
    is the kind of thing that looks like a simulator problem."""
    from PIL import Image
    rgb = Image.fromarray(np.dstack([
        np.full((4, 4), 255, np.uint8),      # R
        np.zeros((4, 4), np.uint8),          # G
        np.zeros((4, 4), np.uint8),          # B
    ]))
    bgr = pil_to_bgr(rgb)
    assert bgr[0, 0].tolist() == [0, 0, 255], bgr[0, 0].tolist()
    assert pil_to_bgr(None) is None


def test_the_video_tool_and_the_live_window_use_the_same_function():
    """Not a style check: the whole reason this module exists."""
    src = (ROOT / "tools" / "make_demo_video.py").read_text(encoding="utf-8")
    assert "from two_view import side_by_side" in src
    assert "np.hstack" not in src, (
        "make_demo_video.py open-codes the layout again; it must call "
        "two_view.side_by_side so the window and the video cannot diverge")
    rec = (ROOT / "demo" / "recorder.py").read_text(encoding="utf-8")
    assert "side_by_side" in rec


def test_the_live_window_cannot_take_the_flight_down():
    """A display fault must cost a frame, not the mission."""
    rec = (ROOT / "demo" / "recorder.py").read_text(encoding="utf-8")
    assert "_live_failed" in rec, "no latch to stop retrying a dead display"
    assert "recording continues" in rec


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
