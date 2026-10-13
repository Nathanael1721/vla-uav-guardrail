"""tools/make_compare_video.py: two recorded runs (Shield on / Shield off) in
one video (rail-video unit, part 1).

    python tests/test_railvideo_compare.py      (no pytest needed)

The runs here are synthetic view/ folders in the recorder's layout (fpv/,
tps/, recorder.json), small frames so the encodes take a second. The encode
tests need ffmpeg and OpenCV and print SKIP without them; the refusals and
the alignment run anywhere (placeholder frames without OpenCV).

Shown failing on a pristine HEAD 5cf7a08 copy (git archive) with this file
copied in: tools/make_compare_video.py does not exist there, so every test
errors on the import.
"""
from __future__ import annotations

import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT / "tools", ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

SKIP = "SKIP"


def _mvc():
    import make_compare_video
    return make_compare_video


def _tmp() -> Path:
    return Path(tempfile.mkdtemp(prefix="pas_cmp_"))


def _have_cv2() -> bool:
    try:
        import cv2                                                   # noqa: F401
        import PIL                                                   # noqa: F401
        return True
    except ImportError:
        return False


def _run(root: Path, name: str, n: int, colour, *, hz=20.0, frame_t=None,
         sidecar=True, gap=None) -> Path:
    """A recorded run: n frames per view (one missing tps frame at `gap`).
    Without OpenCV the frames are empty placeholders: enough for everything
    that does not decode them (the refusals, the alignment)."""
    view = root / name / "view"
    (view / "fpv").mkdir(parents=True)
    (view / "tps").mkdir(parents=True)
    for i in range(n):
        if not _have_cv2():
            (view / "fpv" / f"{i:05d}.jpg").write_bytes(b"")
            if i != gap:
                (view / "tps" / f"{i:05d}.jpg").write_bytes(b"")
            continue
        import cv2
        import numpy as np
        im = np.zeros((72, 128, 3), dtype=np.uint8)
        im[:] = colour
        cv2.putText(im, str(i), (4, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        cv2.imwrite(str(view / "fpv" / f"{i:05d}.jpg"), im)
        if i != gap:
            cv2.imwrite(str(view / "tps" / f"{i:05d}.jpg"), im[:, ::-1])
    if sidecar:
        ft = frame_t if frame_t is not None else [round(i / hz, 3) for i in range(n)]
        (view / "recorder.json").write_text(json.dumps(
            {"frames": n, "target_hz": 20.0, "achieved_hz": hz, "frame_t": ft}),
            encoding="utf-8")
    return view


def _probe(path: Path) -> dict:
    r = subprocess.run(["ffprobe", "-v", "error", "-count_frames", "-select_streams", "v:0",
                        "-show_entries", "stream=width,height,nb_read_frames,r_frame_rate",
                        "-of", "json", str(path)], capture_output=True, text=True, timeout=60)
    return json.loads(r.stdout)["streams"][0]


def test_an_existing_output_is_refused_without_force():
    """Never overwrite: refused before anything is decoded or encoded (and
    before ffmpeg is looked for), the file left byte for byte."""
    m = _mvc()
    root = _tmp()
    a = _run(root, "on", 10, (0, 120, 0))
    b = _run(root, "off", 10, (0, 0, 160))
    out = root / "cmp.mp4"
    out.write_bytes(b"an earlier video")
    rc = m.main(["--a", str(a), "--b", str(b), "--out", str(out),
                 "--ffmpeg", str(root / "no-such-ffmpeg.exe")])
    assert rc == 3, rc
    assert out.read_bytes() == b"an earlier video", "an existing output was touched"
    assert not list(root.glob("*.partial*")), "a temporary file was left behind"


def test_force_replaces_an_existing_output():
    if shutil.which("ffmpeg") is None or not _have_cv2():
        return SKIP
    m = _mvc()
    root = _tmp()
    a = _run(root, "on", 10, (0, 120, 0))
    b = _run(root, "off", 10, (0, 0, 160))
    out = root / "cmp.mp4"
    out.write_bytes(b"an earlier video")
    rc = m.main(["--a", str(a), "--b", str(b), "--out", str(out), "--force"])
    assert rc == 0 and out.read_bytes()[:16] != b"an earlier video"[:16]
    assert _probe(out)["nb_read_frames"] == "10"


def test_the_default_output_is_beside_run_a_and_also_never_overwritten():
    m = _mvc()
    root = _tmp()
    import uuid
    tag = f"rvtest_{uuid.uuid4().hex[:8]}"
    a = _run(root, f"{tag}_on", 4, (0, 120, 0))
    b = _run(root, f"{tag}_off", 4, (0, 0, 160))
    ra = m.load_run(str(a))
    assert ra.name == f"{tag}_on", ra.name
    # A run directory and its view/ both resolve to the view.
    assert m.resolve_view(str(a.parent)) == a and m.resolve_view(str(a)) == a
    default = ROOT / "demo" / "out" / f"{tag}_on" / f"{tag}_on_vs_{tag}_off_compare.mp4"
    if default.exists():
        return SKIP                      # never touch a real video to test this
    default.parent.mkdir(parents=True, exist_ok=True)
    made_dir = not any(default.parent.iterdir())
    default.write_bytes(b"keep")
    try:
        assert m.main(["--a", str(a), "--b", str(b)]) == 3
        assert default.read_bytes() == b"keep"
    finally:
        default.unlink()
        if made_dir:
            default.parent.rmdir()


def test_each_half_starts_at_mission_time_zero_and_runs_at_its_own_rate():
    """Aligned from take-off: the start frame is the first one stamped at or
    after mission time 0 (a recorder that wrote frames before the mission
    clock started has negative stamps), and each half maps output time to
    frames by ITS run's measured rate; a run that ends first holds its last
    frame."""
    m = _mvc()
    root = _tmp()
    # A: 10 Hz, its first 5 frames before mission start (t -0.5 .. -0.1).
    a = _run(root, "a", 25, (0, 120, 0), hz=10.0,
             frame_t=[round(-0.5 + i / 10, 3) for i in range(25)])
    # B: 20 Hz from t 0, no per-frame stamps beyond the rate.
    b = _run(root, "b", 30, (0, 0, 160), hz=20.0, frame_t=[None] * 30)
    ra, rb = m.load_run(str(a)), m.load_run(str(b))
    assert ra.start == 5 and rb.start == 0, (ra.start, rb.start)
    assert m.start_index([None, -0.2, 0.0, 0.1], 0.0) == 2
    assert m.start_index([], 0.0) == 0
    assert m.frame_index(ra, 1.0) == (15, False), "10 Hz: 1 s after the start is 10 frames on"
    assert m.frame_index(rb, 1.0) == (20, False), "20 Hz: 20 frames on"
    assert m.frame_index(rb, 5.0) == (29, True), "past its end a run holds its last frame"
    p = m.plan(ra, rb)
    assert abs(p["seconds_a"] - 2.0) < 1e-9 and abs(p["seconds_b"] - 1.5) < 1e-9
    assert p["fps"] == 20.0 and abs(p["seconds"] - 2.0) < 1e-9 and p["frames"] == 40
    assert m.plan(ra, rb, end="shortest")["frames"] == 30
    # --start-a moves A's alignment to another mission time.
    assert m.load_run(str(a), start_t=1.0).start == 15
    # The title bar's time is the frame's own stamp.
    assert m.mission_time(ra, 15, 1.0) == 1.0 and m.mission_time(rb, 20, 1.0) == 1.0


def test_a_run_without_a_capture_rate_is_refused_not_guessed():
    m = _mvc()
    root = _tmp()
    a = _run(root, "a", 5, (0, 120, 0))
    b = _run(root, "b", 5, (0, 0, 160), sidecar=False)
    out = root / "x.mp4"
    assert m.main(["--a", str(a), "--b", str(b), "--out", str(out)]) == 2
    assert not out.exists()
    try:
        m.load_run(str(b))
    except ValueError as exc:
        assert "--fps-b" in str(exc) and "b" in str(exc)
    else:
        raise AssertionError("a run with no rate was given one")
    assert m.load_run(str(b), fps=12.5).fps == 12.5
    try:
        m.load_run(str(root / "nothing-here"))
    except ValueError as exc:
        assert "--save-view" in str(exc)
    else:
        raise AssertionError("a folder without frames was accepted")


def test_the_video_has_both_titled_halves_aligned_and_a_failed_encode_writes_nothing():
    if not _have_cv2() or shutil.which("ffmpeg") is None:
        return SKIP
    import cv2
    m = _mvc()
    root = _tmp()
    a = _run(root, "on", 20, (0, 120, 0), hz=10.0, gap=3)
    b = _run(root, "off", 10, (0, 0, 160), hz=10.0)
    out = root / "cmp.mp4"
    assert m.main(["--a", str(a), "--b", str(b), "--out", str(out), "--height", "144"]) == 0
    pr = _probe(out)
    # 20 frames of A at 10 Hz = 2.0 s; B holds its last frame for the second.
    assert pr["nb_read_frames"] == "20" and pr["r_frame_rate"] == "10/1", pr
    W, H = int(pr["width"]), int(pr["height"])
    assert W % 2 == 0 and H % 2 == 0
    title_h = max(28, int(144 * 0.09))
    assert H == 2 * (title_h + 144), (H, title_h)
    ok, fr = cv2.VideoCapture(str(out)).read()
    assert ok
    # The title bars in their colours (BGR in the decoded frame): A's green
    # above, B's red below.
    ga = fr[title_h // 2, W - 4].astype(int)
    rb = fr[title_h + 144 + title_h // 2, W - 4].astype(int)
    assert ga[1] > 110 and ga[2] < 90, ga
    assert rb[2] > 150 and rb[1] < 80, rb
    # A failed encode (an "ffmpeg" that exits 1 and writes no video): nothing
    # at the output and no temporary file.
    if sys.platform.startswith("win"):
        fake = root / "fake_ffmpeg.bat"
        fake.write_text("@exit /b 1\r\n", encoding="ascii")
    else:
        fake = root / "fake_ffmpeg.sh"
        fake.write_text("#!/bin/sh\nexit 1\n", encoding="ascii")
        fake.chmod(0o755)
    bad = root / "bad.mp4"
    rc = m.main(["--a", str(a), "--b", str(b), "--out", str(bad), "--ffmpeg", str(fake)])
    assert rc == 1 and not bad.exists() and not list(root.glob("*.partial*")), rc


def test_the_default_titles_are_the_ones_the_brief_names():
    """The halves read "SHIELD ON" and "SHIELD OFF - control" unless told
    otherwise."""
    a = _mvc().build_parser().parse_args(["--a", "x", "--b", "y"])
    assert (a.label_a, a.label_b) == ("SHIELD ON", "SHIELD OFF - control"), (a.label_a, a.label_b)
    assert a.force is False and a.layout == "stacked"


def test_the_title_shows_the_frames_own_mission_time():
    """A recorder's frames are stamped with the HUD time they carried, which
    drifts from start + elapsed whenever a slot is late: the title shows the
    stamp, and start + elapsed only for a frame with none."""
    m = _mvc()
    root = _tmp()
    a = _run(root, "a", 6, (0, 120, 0), hz=10.0,
             frame_t=[0.0, 0.13, 0.19, 0.33, 0.41, 0.52])
    b = _run(root, "b", 6, (0, 0, 160), hz=10.0, frame_t=[None] * 6)
    ra, rb = m.load_run(str(a)), m.load_run(str(b))
    assert m.mission_time(ra, 3, 0.3) == 0.33 and m.mission_time(rb, 3, 0.3) == 0.3
    assert m.title_right(ra, 3, 0.3, False) == "a   t    0.3 s"
    assert m.title_right(ra, 1, 0.1, False).endswith("t    0.1 s")
    assert m.title_right(ra, 5, 9.0, True) == "a   t    0.5 s   ENDED"
    rs = m.load_run(str(a), start_t=0.15)
    assert rs.start == 2 and m.mission_time(rs, 2, 0.0) == 0.19


def _stub_ffmpeg(root: Path, mode: str) -> Path:
    """An "ffmpeg" that reads the frames it is piped and then: writes part
    of a video and fails (partial_fail); succeeds with an empty file
    (empty_ok); or succeeds while someone else's file appears at the output
    path (appear). The output it is given is the tool's temporary file."""
    stub = root / f"stub_{mode}.py"
    stub.write_text("\n".join([
        "import sys, pathlib",
        "sys.stdin.buffer.read()",
        "tmp = pathlib.Path(sys.argv[-1])",
        f"mode = {mode!r}",
        "if mode in ('partial_fail', 'appear'):",
        "    tmp.write_bytes(b'x' * 100)",
        "if mode == 'empty_ok':",
        "    tmp.write_bytes(b'')",
        "if mode == 'appear':",
        "    tmp.with_name(tmp.name.replace('.partial', '')).write_bytes(b'theirs')",
        "sys.exit(1 if mode == 'partial_fail' else 0)", ""]), encoding="ascii")
    if sys.platform.startswith("win"):
        fake = root / f"ffmpeg_{mode}.bat"
        fake.write_text(f'@"{sys.executable}" "{stub}" %*\r\n@exit /b %ERRORLEVEL%\r\n',
                        encoding="ascii")
    else:
        fake = root / f"ffmpeg_{mode}.sh"
        fake.write_text(f'#!/bin/sh\nexec "{sys.executable}" "{stub}" "$@"\n', encoding="ascii")
        fake.chmod(0o755)
    return fake


def test_a_failed_or_empty_encode_and_a_late_output_leave_nothing_of_ours():
    """ffmpeg's exit code alone is not trusted, and the output is checked
    again after the encode: a failed encode that wrote part of a file, an
    'ok' with an empty file, and a file that appeared at the output path
    while encoding each end with nothing of this run's at the output and no
    temporary file left; the file that appeared is not replaced."""
    if not _have_cv2():
        return SKIP
    m = _mvc()
    root = _tmp()
    a = _run(root, "on", 6, (0, 120, 0), hz=10.0)
    b = _run(root, "off", 6, (0, 0, 160), hz=10.0)
    for mode, rc_want in (("partial_fail", 1), ("empty_ok", 1), ("appear", 3)):
        out = root / f"cmp_{mode}.mp4"
        rc = m.main(["--a", str(a), "--b", str(b), "--out", str(out), "--height", "72",
                     "--ffmpeg", str(_stub_ffmpeg(root, mode))])
        assert rc == rc_want, (mode, rc)
        assert not list(root.glob("*.partial*")), (mode, list(root.glob("*.partial*")))
        if mode == "appear":
            assert out.read_bytes() == b"theirs", "a file that appeared mid-encode was replaced"
        else:
            assert not out.exists(), (mode, out)
    # --force replaces even the late one.
    out = root / "cmp_appear.mp4"
    stub = _stub_ffmpeg(root, "appear")
    assert m.main(["--a", str(a), "--b", str(b), "--out", str(root / "late.mp4"),
                   "--height", "72", "--ffmpeg", str(stub), "--force"]) == 0
    assert (root / "late.mp4").read_bytes() == b"x" * 100 and out.read_bytes() == b"theirs"


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
        except (Exception, SystemExit) as e:          # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
