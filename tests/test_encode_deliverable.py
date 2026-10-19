"""tools/encode_deliverable.py: the H.264 deliverable copy of a demo master.

Run either way:
    pytest tests/test_encode_deliverable.py -v
    python tests/test_encode_deliverable.py

WHY THIS FILE EXISTS (review of 2026-10-19, harness-tools/encode-deliverable-
destroys-previous-copy)

docs/video/ is gitignored, so a shipped deliverable has no backup. ffmpeg
wrote straight over it: a failed encode (a `--preset` typo, measured with
ffmpeg 9.0) left it at 0 bytes, Ctrl-C left a partial file, and a REJECTED
encode deleted the path after the old copy was already overwritten. A probe
that read fps 0 on both sides made the duration check pass vacuously.

Pinned, with ffmpeg and OpenCV replaced (so it runs on CI, which has
neither): a pre-existing deliverable is byte-identical after a failed, a
rejected and an interrupted encode, and after a replace refused because the
file is in use; no `.partial.mp4` is left behind; a verified encode does
replace it; a probe with no rate or no frames refuses.
"""
import math
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import encode_deliverable as E                                    # noqa: E402

OLD = b"the shipped deliverable, no backup"
TAG = "demo_test"


def _info(seconds, fps=20.0):
    frames = int(round(seconds * fps)) if fps and math.isfinite(fps) else 0
    return {"fps": fps, "frames": frames, "w": 8, "h": 8,
            "seconds": (frames / fps) if fps else 0.0, "codec": "avc1", "mb": 0.1}


class _Rig:
    """E with its paths in a temp dir, ffmpeg and the probe replaced."""

    def __init__(self, run, encoded_s=10.0, master_s=10.0, fps=20.0):
        self.td = tempfile.TemporaryDirectory()
        self.root = Path(self.td.name)
        master = self.root / "demo" / "out" / TAG / f"{TAG}_demo.mp4"
        master.parent.mkdir(parents=True)
        master.write_bytes(b"master")
        self.dest = self.root / "docs" / "video" / f"{TAG}.mp4"
        self.dest.parent.mkdir(parents=True)
        self.dest.write_bytes(OLD)
        self.run, self.encoded_s, self.master_s, self.fps = run, encoded_s, master_s, fps

    def __enter__(self):
        self.saved = (E.ROOT, E.OUT_DIR, E.find_ffmpeg, E.probe, E.subprocess.run)
        E.ROOT, E.OUT_DIR = self.root, self.root / "docs" / "video"
        E.find_ffmpeg = lambda: "ffmpeg"

        def probe(path):
            s = self.master_s if path.name.endswith("_demo.mp4") else self.encoded_s
            return E.check_probe(_info(s, self.fps), path)
        E.probe = probe
        E.subprocess.run = self.run
        return self

    def __exit__(self, *exc):
        E.ROOT, E.OUT_DIR, E.find_ffmpeg, E.probe, E.subprocess.run = self.saved
        self.td.cleanup()
        return False

    def partials(self):
        return sorted(p.name for p in self.dest.parent.glob("*.partial*"))


def _writes(data):
    def run(cmd, **kw):
        Path(cmd[-1]).write_bytes(data)
        return subprocess.CompletedProcess(cmd, 0)
    return run


def _expect(exc_type, fn, needle=""):
    try:
        fn()
    except exc_type as e:
        assert needle in str(e), (needle, str(e))
        return
    raise AssertionError(f"expected {exc_type.__name__}")


def test_a_failed_encode_leaves_the_previous_deliverable_untouched():
    def run(cmd, **kw):
        Path(cmd[-1]).write_bytes(b"")            # ffmpeg truncates, then fails
        raise subprocess.CalledProcessError(1, cmd)
    with _Rig(run) as r:
        _expect(subprocess.CalledProcessError, lambda: E.encode(TAG, 23, "slwo", 0.15))
        assert r.dest.read_bytes() == OLD, "the deliverable was overwritten"
        assert r.partials() == [], r.partials()


def test_an_interrupted_encode_leaves_the_previous_deliverable_untouched():
    def run(cmd, **kw):
        Path(cmd[-1]).write_bytes(b"half a vid")
        raise KeyboardInterrupt
    with _Rig(run) as r:
        _expect(KeyboardInterrupt, lambda: E.encode(TAG, 23, "slow", 0.15))
        assert r.dest.read_bytes() == OLD and r.partials() == []


def test_a_rejected_encode_keeps_the_previous_deliverable_and_says_so():
    with _Rig(_writes(b"short"), encoded_s=5.0) as r:
        _expect(SystemExit, lambda: E.encode(TAG, 23, "slow", 0.15), "REJECTED")
        assert r.dest.read_bytes() == OLD, "a rejection deleted the shipped copy"
        assert r.partials() == []
    with _Rig(_writes(b"short"), encoded_s=5.0) as r:
        _expect(SystemExit, lambda: E.encode(TAG, 23, "slow", 0.15), "left in place")


def test_a_verified_encode_replaces_the_deliverable():
    with _Rig(_writes(b"the new h264 copy")) as r:
        got = E.encode(TAG, 23, "slow", 0.15)
        assert r.dest.read_bytes() == b"the new h264 copy"
        assert got["path"] == r.dest and r.partials() == []


def test_a_deliverable_in_use_is_left_intact_and_the_refusal_says_so():
    def busy(src, dst):
        raise PermissionError(32, "in use")
    with _Rig(_writes(b"new")) as r:
        saved = E.os.replace
        E.os.replace = busy
        try:
            _expect(SystemExit, lambda: E.encode(TAG, 23, "slow", 0.15), "intact")
        finally:
            E.os.replace = saved
        assert r.dest.read_bytes() == OLD and r.partials() == []


def test_a_probe_with_no_rate_or_no_frames_refuses_instead_of_passing_vacuously():
    for fps in (0.0, float("nan")):
        _expect(SystemExit, lambda: E.check_probe(_info(10.0, fps), Path("x.mp4")),
                "vacuous")
    _expect(SystemExit, lambda: E.check_probe(dict(_info(10.0), frames=0), Path("x.mp4")))
    assert E.check_probe(_info(10.0), Path("x.mp4"))["frames"] == 200
    # both sides unreadable: refused, and the old copy stays
    with _Rig(_writes(b"half"), encoded_s=5.0, fps=0.0) as r:
        _expect(SystemExit, lambda: E.encode(TAG, 23, "slow", 0.15), "vacuous")
        assert r.dest.read_bytes() == OLD and r.partials() == []


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
        except BaseException as e:                   # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
