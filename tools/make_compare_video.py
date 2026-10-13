"""
Put two recorded flights in one video: the Shield on beside the Shield off.

Each run is a flight recorded with --save-view (demo/pas_ardupilot/
perception_node.py, scripts/run_pas_ardupilot.ps1 -SaveView, or
demo/follow_vlm.py): view/fpv holds the drone's view with the policy HUD,
view/tps the chase camera, view/recorder.json the rate the frames were
captured at and the mission time each one was stamped with. Each half of the
output is one run's two panels side by side, under a title bar - "SHIELD ON"
and "SHIELD OFF - control" by default - that also carries the run's name and
its mission time. The halves are stacked (default) or put side by side.

ALIGNED FROM TAKE-OFF. Both halves start at mission time 0 (--start-a /
--start-b move either): on the perception rail that is the first control tick
after the link's take-off (pymavlink: ArduPilot within 1 m of the cruise
altitude, autopilot.PymavlinkLink.takeoff), the moment the recorder writes
its first frame. The start frame is read from recorder.json's `frame_t` (the
HUD time each frame carried), never assumed to be frame 0.

REAL TIME, PER RUN. Each half advances at its own run's measured capture rate
(recorder.json `achieved_hz`, else `target_hz`; --fps-a / --fps-b override),
so two runs whose recorders kept different rates both play in real time side
by side. With no rate to read, the tool refuses rather than guess: guessed rates
are how earlier demo videos came to play 1.24x and 1.35x off real time
(tools/make_demo_video.py has that history). When one run ends first its half
holds its last frame, marked ENDED, until the other ends (--end shortest
stops at the first end instead).

NEVER OVERWRITES. An existing output file is refused unless --force; the
video is encoded to a temporary file beside it and moved into place only when
ffmpeg succeeded, so a failed run leaves nothing at the output path.

Encoding is ffmpeg's (libx264, yuv420p): the frames are composed here and
piped to it raw.

Run:
    python tools/make_compare_video.py --a pas_ap_on --b pas_ap_off
    python tools/make_compare_video.py --a pas_ap_on --b pas_ap_off --layout side --panel tps
    python tools/make_compare_video.py --a demo/out/x/view --b demo/out/y/view --out cmp.mp4
"""
from __future__ import annotations

import argparse
import bisect
import json
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "demo") not in sys.path:
    sys.path.insert(0, str(ROOT / "demo"))

DEFAULT_LABEL_A = "SHIELD ON"
DEFAULT_LABEL_B = "SHIELD OFF - control"
# Title bar colours (RGB): the policy HUD's OK green and BRAKE red, so the
# two halves read the way the banners inside them do.
COLOUR_A = (40, 150, 75)
COLOUR_B = (190, 35, 35)


@dataclass
class Run:
    """One recorded flight's frames, by recorder index."""
    name: str
    view: Path
    fps: float
    fps_source: str
    fpv: dict = field(default_factory=dict)      # index -> path
    tps: dict = field(default_factory=dict)
    frame_t: list = field(default_factory=list)  # mission time per index (or None)
    start: int = 0                               # first index at/after the start time
    start_t: float = 0.0

    @property
    def n(self) -> int:
        idx = set(self.fpv) | set(self.tps)
        return (max(idx) + 1) if idx else 0

    @property
    def seconds(self) -> float:
        return max(0, self.n - self.start) / self.fps


def resolve_view(spec: str) -> Path:
    """A run as given: a view directory, a run directory holding view/, or a
    tag under demo/out/."""
    p = Path(spec)
    for cand in (p, p / "view", ROOT / "demo" / "out" / spec / "view"):
        if cand.is_dir() and ((cand / "fpv").is_dir() or (cand / "tps").is_dir()):
            return cand
    raise ValueError(f"no recorded frames for {spec!r}: looked for fpv/ or tps/ in "
                     f"{p}, {p / 'view'} and demo/out/{spec}/view (was it flown with "
                     f"--save-view?)")


def _frames(d: Path) -> dict:
    out = {}
    if d.is_dir():
        for f in d.glob("*.jpg"):
            try:
                out[int(f.stem)] = f
            except ValueError:
                continue
    return out


def start_index(frame_t: list, start_t: float) -> int:
    """The first frame whose stamped mission time is at or after `start_t`;
    0 when no frame carries a time (an old recording)."""
    for i, t in enumerate(frame_t or []):
        if t is not None and float(t) >= start_t - 1e-9:
            return i
    return 0


def load_run(spec: str, *, fps: float | None = None, start_t: float = 0.0) -> Run:
    view = resolve_view(spec)
    name = view.parent.name if view.name == "view" else view.name
    side = view / "recorder.json"
    rec = {}
    if side.is_file():
        try:
            rec = json.loads(side.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            rec = {}
    if fps is not None:
        rate, src = float(fps), "given (--fps-a / --fps-b)"
    elif rec.get("achieved_hz"):
        rate, src = float(rec["achieved_hz"]), "recorder.json achieved_hz"
    elif rec.get("target_hz"):
        rate, src = float(rec["target_hz"]), "recorder.json target_hz (no achieved rate)"
    else:
        raise ValueError(f"{name}: no capture rate - {side} is missing or has none. "
                         f"Pass --fps-a / --fps-b with the rate the frames were "
                         f"recorded at; a guessed rate plays the half off real time")
    if not rate > 0:
        raise ValueError(f"{name}: capture rate {rate} is not positive")
    run = Run(name=name, view=view, fps=rate, fps_source=src,
              fpv=_frames(view / "fpv"), tps=_frames(view / "tps"),
              frame_t=list(rec.get("frame_t") or []), start_t=float(start_t))
    if run.n == 0:
        raise ValueError(f"{name}: no frames under {view}")
    run.start = start_index(run.frame_t, start_t)
    return run


def frame_index(run: Run, tau: float) -> tuple[int, bool]:
    """(recorder index, ended) for output time `tau` seconds after the start:
    the run's own rate maps time to frames; past its last frame it holds."""
    i = run.start + int(tau * run.fps + 1e-6)
    last = run.n - 1
    return (last, True) if i > last else (i, False)


def _latest(frames: dict, keys: list, i: int):
    """The frame at index i, or the latest one before it (a camera that
    returned nothing for a slot leaves a gap in its numbering)."""
    k = bisect.bisect_right(keys, i) - 1
    return frames[keys[k]] if k >= 0 else None


def mission_time(run: Run, i: int, tau: float) -> float:
    """The mission time shown in the title bar: the frame's stamp when it has
    one, else the start time plus the elapsed output time."""
    if 0 <= i < len(run.frame_t) and run.frame_t[i] is not None:
        return float(run.frame_t[i])
    return run.start_t + tau


def title_right(run: Run, i: int, tau: float, ended: bool) -> str:
    """The right-hand text of a half's title bar: the run's name and the
    mission time of the frame on screen (mission_time), ENDED once held."""
    return (f"{run.name}   t {mission_time(run, i, tau):6.1f} s"
            + ("   ENDED" if ended else ""))


def _title(width: int, height: int, label: str, right: str, colour) -> "object":
    import numpy as np
    from PIL import Image, ImageDraw
    try:
        from policy_hud import font
        big, small = font(int(height * 0.55), bold=True), font(int(height * 0.40))
    except Exception:                                         # noqa: BLE001
        from PIL import ImageFont
        big = small = ImageFont.load_default()
    im = Image.new("RGB", (width, height), colour)
    d = ImageDraw.Draw(im)
    d.text((int(height * 0.35), int(height * 0.18)), label, font=big, fill=(255, 255, 255))
    tw = d.textlength(right, font=small)
    d.text((width - tw - int(height * 0.35), int(height * 0.28)), right, font=small,
           fill=(240, 240, 240))
    return np.asarray(im)[:, :, ::-1].copy()           # RGB -> BGR


def _pad(im, w: int, h: int):
    import numpy as np
    if im.shape[1] == w and im.shape[0] == h:
        return im
    out = np.zeros((h, w, 3), dtype=im.dtype)
    out[:im.shape[0], :im.shape[1]] = im
    return out


class Half:
    """One run's half of the output: its panels at `height`, cached by
    index so a held or repeated frame is not decoded again."""

    def __init__(self, run: Run, panel: str, height: int):
        self.run, self.panel, self.height = run, panel, height
        self.fpv_keys, self.tps_keys = sorted(run.fpv), sorted(run.tps)
        self._key = None
        self._img = None

    def _load(self, path):
        import cv2
        if path is None:
            return None
        im = cv2.imread(str(path))
        if im is None:
            return None
        s = self.height / im.shape[0]
        return cv2.resize(im, (max(1, int(round(im.shape[1] * s))), self.height),
                          interpolation=cv2.INTER_AREA)

    def image(self, i: int):
        from two_view import side_by_side
        a = _latest(self.run.fpv, self.fpv_keys, i) if self.panel in ("both", "fpv") else None
        b = _latest(self.run.tps, self.tps_keys, i) if self.panel in ("both", "tps") else None
        key = (a, b)
        if key != self._key:
            self._img = side_by_side(self._load(a), self._load(b))
            self._key = key
        return self._img


def plan(run_a: Run, run_b: Run, *, end: str = "longest", max_s: float | None = None,
         fps: float | None = None) -> dict:
    """The output's rate, duration and frame count."""
    da, db = run_a.seconds, run_b.seconds
    dur = max(da, db) if end == "longest" else min(da, db)
    if max_s is not None:
        dur = min(dur, float(max_s))
    rate = float(fps) if fps else max(run_a.fps, run_b.fps)
    return {"fps": rate, "seconds": dur, "frames": max(1, int(round(dur * rate))),
            "seconds_a": da, "seconds_b": db}


def compose(half_a: Half, half_b: Half, run_a: Run, run_b: Run, tau: float, *,
            layout: str, labels: tuple[str, str], title_h: int):
    import numpy as np
    parts = []
    for half, run, label, colour in ((half_a, run_a, labels[0], COLOUR_A),
                                     (half_b, run_b, labels[1], COLOUR_B)):
        i, ended = frame_index(run, tau)
        img = half.image(i)
        if img is None:
            img = np.zeros((half.height, int(half.height * 16 / 9), 3), dtype=np.uint8)
        parts.append((img, label, title_right(run, i, tau, ended), colour))
    if layout == "stacked":
        w = max(p[0].shape[1] for p in parts)
        rows = []
        for img, label, right, colour in parts:
            rows.append(_title(w, title_h, label, right, colour))
            rows.append(_pad(img, w, img.shape[0]))
        frame = np.vstack(rows)
    else:
        h = max(p[0].shape[0] for p in parts)
        cols = []
        for img, label, right, colour in parts:
            col = np.vstack([_title(img.shape[1], title_h, label, right, colour),
                             _pad(img, img.shape[1], h)])
            cols.append(col)
        frame = np.hstack(cols)
    # yuv420p needs even dimensions.
    H, W = frame.shape[0] + frame.shape[0] % 2, frame.shape[1] + frame.shape[1] % 2
    return _pad(frame, W, H)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.strip().split("\n\n")[0])
    ap.add_argument("--a", required=True, help="run A (default title SHIELD ON): a tag "
                    "under demo/out/, a run directory, or its view/ directory")
    ap.add_argument("--b", required=True, help="run B (default title SHIELD OFF - control)")
    ap.add_argument("--label-a", default=DEFAULT_LABEL_A)
    ap.add_argument("--label-b", default=DEFAULT_LABEL_B)
    ap.add_argument("--layout", choices=["stacked", "side"], default="stacked",
                    help="stacked (A above B, default) or side (A left of B)")
    ap.add_argument("--panel", choices=["both", "fpv", "tps"], default="both",
                    help="what each half shows: both views (default), the drone's "
                         "view with the HUD, or the chase view")
    ap.add_argument("--height", type=int, default=540,
                    help="height of each panel in pixels (default 540)")
    ap.add_argument("--start-a", type=float, default=0.0,
                    help="mission time run A's half starts at (default 0: take-off "
                         "complete, the first control tick)")
    ap.add_argument("--start-b", type=float, default=0.0)
    ap.add_argument("--fps-a", type=float, default=None,
                    help="run A's capture rate, when its recorder.json has none")
    ap.add_argument("--fps-b", type=float, default=None)
    ap.add_argument("--fps", type=float, default=None,
                    help="output frame rate (default: the faster run's capture rate)")
    ap.add_argument("--end", choices=["longest", "shortest"], default="longest")
    ap.add_argument("--max-s", type=float, default=None, help="cut the video at this length")
    ap.add_argument("--crf", type=int, default=20)
    ap.add_argument("--ffmpeg", default=None, help="ffmpeg executable (default: on PATH)")
    ap.add_argument("--out", default=None,
                    help="output .mp4 (default demo/out/<A>/<A>_vs_<B>_compare.mp4)")
    ap.add_argument("--force", action="store_true",
                    help="replace an existing output file (refused without it)")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        run_a = load_run(args.a, fps=args.fps_a, start_t=args.start_a)
        run_b = load_run(args.b, fps=args.fps_b, start_t=args.start_b)
    except ValueError as exc:
        print(f"[compare] refused: {exc}")
        return 2
    out = Path(args.out) if args.out else (
        ROOT / "demo" / "out" / run_a.name / f"{run_a.name}_vs_{run_b.name}_compare.mp4")
    # Refused before anything is decoded or encoded, and before ffmpeg is
    # even looked for.
    if out.exists() and not args.force:
        print(f"[compare] refused: {out} exists. Pass --force to replace it, or "
              f"--out a new path.")
        return 3
    ffmpeg = args.ffmpeg or shutil.which("ffmpeg")
    if not ffmpeg:
        print("[compare] refused: ffmpeg not found on PATH (or pass --ffmpeg)")
        return 2
    p = plan(run_a, run_b, end=args.end, max_s=args.max_s, fps=args.fps)
    for r in (run_a, run_b):
        print(f"[compare] {r.name}: {r.n} frames at {r.fps} fps ({r.fps_source}); starts "
              f"at frame {r.start} (mission t >= {r.start_t:g} s); {r.seconds:.1f} s")
    print(f"[compare] {p['frames']} frames at {p['fps']:g} fps = {p['seconds']:.1f} s "
          f"({args.end}; A {p['seconds_a']:.1f} s, B {p['seconds_b']:.1f} s), "
          f"{args.layout}, panels {args.panel}")
    title_h = max(28, int(args.height * 0.09))
    half_a, half_b = Half(run_a, args.panel, args.height), Half(run_b, args.panel, args.height)
    first = compose(half_a, half_b, run_a, run_b, 0.0, layout=args.layout,
                    labels=(args.label_a, args.label_b), title_h=title_h)
    H, W = first.shape[:2]
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_name(out.stem + ".partial" + out.suffix)
    cmd = [ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
           "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{W}x{H}", "-r", f"{p['fps']:g}",
           "-i", "-", "-c:v", "libx264", "-preset", "veryfast", "-crf", str(args.crf),
           "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(tmp)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    ok = False
    try:
        for k in range(p["frames"]):
            fr = first if k == 0 else compose(
                half_a, half_b, run_a, run_b, k / p["fps"], layout=args.layout,
                labels=(args.label_a, args.label_b), title_h=title_h)
            if fr.shape[:2] != (H, W):
                fr = _pad(fr[:H, :W], W, H)
            proc.stdin.write(fr.tobytes())
        proc.stdin.close()
        ok = proc.wait() == 0
    except (BrokenPipeError, OSError) as exc:
        print(f"[compare] ffmpeg stopped reading: {exc}")
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
    # An exit code of 0 is not a video: the file must be there and non-empty.
    ok = ok and tmp.is_file() and tmp.stat().st_size > 0
    if not ok:
        tmp.unlink(missing_ok=True)
        print(f"[compare] ffmpeg failed (exit {proc.returncode}); nothing written to {out}")
        return 1
    # Checked again: the output may have appeared while this was encoding.
    if out.exists() and not args.force:
        tmp.unlink(missing_ok=True)
        print(f"[compare] refused: {out} appeared while encoding; nothing replaced")
        return 3
    os.replace(tmp, out)
    print(f"[compare] {p['frames']} frames, {W}x{H}, {p['fps']:g} fps -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
