"""tools/make_demo_video.py: a flight's two camera views stitched into one video.

Run either way:
    pytest tests/test_make_demo_video.py -v
    python tests/test_make_demo_video.py

WHY THIS FILE EXISTS (review of 2026-10-19, harness-tools/demo-video-silent-
writer-failure)

The VideoWriter was never checked: `--out` into a missing directory, or the
previous video held open (a player, OneDrive), wrote nothing while the script
printed the frame count and exited 0 - and in the second case the OLD video
stayed at the path. And the two cameras were paired by position in each
sorted list, so a camera that skipped one recorder slot shifted its panel
against the other for the rest of the video.

Pinned: pairing by recorder index (runs anywhere); with OpenCV, a missing
output directory and a video that cannot be replaced exit 1 and leave what
was there, and a good run writes every frame. The OpenCV tests return SKIP,
counted apart and printed, where OpenCV is not installed (the vla-real and
pas environments have it) - never a silent pass. Every file is written in a
temporary directory, never under demo/out.
"""
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import make_demo_video as V                                       # noqa: E402

SKIP = "SKIP"


def test_frames_are_paired_by_recorder_index_not_list_position():
    """fpv skipped slot 2 (the recorder advanced on the tps frame alone): the
    panel for index k must show fpv <= k beside tps k. By position, fpv 3
    sat beside tps 2 and every later pair was one slot apart."""
    fpv = [f"v/fpv/{i:05d}.jpg" for i in (0, 1, 3, 4)]
    tps = [f"v/tps/{i:05d}.jpg" for i in range(5)]
    pairs = V.pair_frames(fpv, tps)
    assert len(pairs) == 5, pairs
    for k, (a, b) in enumerate(pairs):
        assert b.endswith(f"{k:05d}.jpg"), (k, b)
    assert [Path(a).stem for a, _ in pairs] == ["00000", "00001", "00001", "00003",
                                               "00004"], pairs
    # a gap-free run pairs exactly as the positional rule did
    same = V.pair_frames(tps, [p.replace("tps", "fpv") for p in tps])
    assert [(Path(a).stem, Path(b).stem) for a, b in same] == [
        (f"{i:05d}", f"{i:05d}") for i in range(5)]
    # one camera only; and a stray non-frame file is ignored
    assert V.pair_frames([], tps + ["v/tps/notes.jpg"])[-1] == (None, tps[-1])
    assert V.pair_frames([], []) == []


def _cv2():
    try:
        import cv2                                            # noqa: F401
        import numpy                                          # noqa: F401
        return True
    except ImportError:
        print("      (OpenCV is not installed here: run it in the vla-real or "
              "pas environment)")
        return False


def _rig(td: Path, n=6, skip_fpv=()):
    import cv2
    import numpy as np
    view = td / "demo" / "out" / "t1" / "view"
    for cam in ("fpv", "tps"):
        (view / cam).mkdir(parents=True)
        for i in range(n):
            if cam == "fpv" and i in skip_fpv:
                continue
            cv2.imwrite(str(view / cam / f"{i:05d}.jpg"),
                        np.full((48, 64, 3), 10 * i, np.uint8))
    (view / "recorder.json").write_text('{"achieved_hz": 10, "frames": %d}' % n,
                                       encoding="utf-8")


def _main(td: Path, *argv):
    saved = V.ROOT
    V.ROOT = td
    try:
        return V.main(["--tag", "t1", "--height", "48", *argv])
    finally:
        V.ROOT = saved


def test_a_missing_output_directory_exits_1_instead_of_claiming_a_video():
    if not _cv2():
        return SKIP
    with tempfile.TemporaryDirectory() as d:
        td = Path(d)
        _rig(td)
        out = td / "nosuchdir" / "x.mp4"
        assert _main(td, "--out", str(out)) == 1
        assert not out.exists() and not out.parent.exists()


def test_a_good_run_writes_every_frame_and_leaves_no_partial_file():
    if not _cv2():
        return SKIP
    import cv2
    with tempfile.TemporaryDirectory() as d:
        td = Path(d)
        _rig(td, n=6, skip_fpv=(2,))
        out = td / "demo" / "out" / "t1" / "t1_demo.mp4"
        assert _main(td) == 0
        cap = cv2.VideoCapture(str(out))
        assert int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) == 6
        cap.release()
        assert not list(out.parent.glob("*.partial*"))


def test_a_video_that_cannot_be_replaced_is_reported_and_left_alone():
    """Windows refuses to replace a file another program holds open; the
    previous video stays and the script says so with exit 1 (until
    2026-10-19 it printed a new frame count beside the old file)."""
    if not _cv2():
        return SKIP
    with tempfile.TemporaryDirectory() as d:
        td = Path(d)
        _rig(td)
        out = td / "demo" / "out" / "t1" / "t1_demo.mp4"
        out.write_bytes(b"the previous video")

        def busy(src, dst):
            raise PermissionError(32, "in use")
        saved = V.os.replace
        V.os.replace = busy
        try:
            assert _main(td) == 1
        finally:
            V.os.replace = saved
        assert out.read_bytes() == b"the previous video"
        assert not list(out.parent.glob("*.partial*"))


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed, {skipped} skipped")
    sys.exit(1 if failed else 0)
