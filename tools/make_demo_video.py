"""
Stitch a flight's recorded frames into one demo video.

Left: what the drone sees, with the detected box and the live telemetry drawn on
it. Right: the third-person chase view. Side by side, because the claim only
reads if both are visible at once — a word chose the box, the box drove the
aircraft, and the guardrail was underneath the whole time.

Run:
    python tools/make_demo_video.py --tag vlm_col
    python tools/make_demo_video.py --tag vlm_col --fps 12 --out demo.mp4

Exit 1, with the reason, when nothing usable was written (review of
2026-10-19, harness-tools/demo-video-silent-writer-failure). Until then the
VideoWriter was never checked: `--out` into a missing directory, or the
previous video held open by a player or OneDrive, wrote nothing while the
script printed "[video] N frames ... -> <path>" and exited 0 - in the second
case the OLD video stayed at the path and flowed on into
tools/encode_deliverable.py. Now the video is written beside the target
(`<name>.partial.mp4`), checked (the writer opened, the file is non-empty and
reads back with as many frames as were written) and only then moved into
place. Frames are paired by their recorder index, not by position in each
camera's list (pair_frames).
"""
from __future__ import annotations

import argparse
import bisect
import glob
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "demo"))

# The SAME layout the live window uses (demo/two_view.py). Two
# open-coded copies would drift, and the drift would only show up
# when someone compared a live screenshot against the video.
from two_view import picture_in_picture, side_by_side    # noqa: E402


def _by_index(paths: list[str]) -> dict[int, str]:
    out = {}
    for p in paths:
        try:
            out[int(Path(p).stem)] = p
        except ValueError:
            continue                        # not a recorder frame (00042.jpg)
    return out


def pair_frames(fpv: list[str], tps: list[str]) -> list[tuple[str | None, str | None]]:
    """(fpv, tps) per recorder index 0..max, each the camera's frame at that
    index or, where it skipped the slot, its latest one before it.

    demo/recorder.py advances its index when EITHER camera wrote, so one
    camera can miss a slot. Pairing by position in each sorted list - what
    this script did until 2026-10-19 - then showed every later fpv frame
    beside the tps frame one slot on, for the rest of the video, and
    repeated the last fpv frame at the end. Keyed by index, as
    make_compare_video's _latest does. (All 47 recorded runs on disk on
    2026-10-19 are gap-free, so every existing master pairs the same way.)"""
    a, b = _by_index(fpv), _by_index(tps)
    ka, kb = sorted(a), sorted(b)
    if not ka and not kb:
        return []
    n = max(ka[-1] if ka else -1, kb[-1] if kb else -1) + 1

    def latest(frames, keys, i):
        k = bisect.bisect_right(keys, i) - 1
        return frames[keys[k]] if k >= 0 else None
    return [(latest(a, ka, i), latest(b, kb, i)) for i in range(n)]


def main(argv=None) -> int:
    import cv2
    import numpy as np                                        # noqa: F401

    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True, help="flight tag under demo/out/")
    ap.add_argument("--fps", default="auto",
                    help="frames per second. 'auto' (default) DERIVES it from "
                         "the flight log so the video runs in real time. A "
                         "fixed value is a guess about the control loop's rate, "
                         "and that guess was wrong: the loop holds about 7.4 Hz, "
                         "not the nominal 10, so writing at 10 played every "
                         "recording 1.35x fast while claiming to be real time.")
    ap.add_argument("--out", default=None)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--layout", choices=("side", "pip"), default="side",
                    help="side: drone view | chase view (default); pip: the drone view "
                         "at full size, the chase view small bottom-left")
    args = ap.parse_args(argv)

    base = ROOT / "demo" / "out" / args.tag / "view"
    fpv = sorted(glob.glob(str(base / "fpv" / "*.jpg")))
    tps = sorted(glob.glob(str(base / "tps" / "*.jpg")))
    if not fpv and not tps:
        print(f"no frames under {base} — was the flight run with --save-view?")
        return 1
    pairs = pair_frames(fpv, tps)

    # Real time means the rate the frames were ACTUALLY CAPTURED at, and the
    # only component that knows that is the recorder, which now writes it to
    # view/recorder.json.
    #
    # Deriving it from the flight log instead is wrong, and was wrong here for
    # two different reasons in turn. First this script assumed the nominal 10 Hz
    # tick rate, while the loop held about 7.4 Hz, so every clip played 1.35x
    # fast. The fix was frames / flight_seconds - which is also wrong, because
    # the recorder starts before the mission clock (scene load, prop spawn,
    # take-off) and stops after it. Measured: 1734 frames spanning ~87 s of
    # recorder life, divided by a 70 s mission, gives 24.8 fps for a recorder
    # pacing at 20. That plays 1.24x fast while claiming real time.
    #
    # The sidecar removes the guess. The flight-log path is kept only as a
    # fallback for older runs, and it says so on the console when it is used.
    fps = None
    if str(args.fps).lower() == "auto":
        side = base / "recorder.json"
        if side.is_file():
            try:
                rec = json.loads(side.read_text(encoding="utf-8"))
                cand = rec.get("achieved_hz") or rec.get("target_hz")
                if cand and float(cand) > 0:
                    fps = round(float(cand), 2)
                    # The WRITING window, not the recorder's whole life. The
                    # recorder is up before the mission clock is, and quoting
                    # the lifetime here made the arithmetic look wrong even
                    # once achieved_hz was right.
                    win = rec.get("writing_seconds") or rec.get("seconds")
                    print(f"[video] recorder captured {rec.get('frames')} frames "
                          f"over {win} s of writing -> {fps} fps "
                          f"(target was {rec.get('target_hz')} Hz)")
            except (ValueError, KeyError, TypeError):
                pass

        if fps is None:
            log = ROOT / "demo" / "out" / args.tag / "flight_log.jsonl"
            if log.is_file():
                try:
                    lines = [l for l in log.read_text(encoding="utf-8").splitlines() if l.strip()]
                    secs = json.loads(lines[-1]).get("t")
                    n_frames = len(pairs)           # the frames the video holds
                    if secs and secs > 1 and n_frames > 1:
                        fps = max(1.0, round(n_frames / float(secs), 2))
                        print(f"[video] NO recorder.json; falling back to "
                              f"{n_frames} frames / {secs:.1f} s of flight = {fps} fps. "
                              f"This OVERSTATES the rate if the recorder outlived "
                              f"the mission clock; playback will be faster than real time.")
                except (ValueError, KeyError, IndexError):
                    pass

        if fps is None:
            fps = 10.0
            print("[video] no recorder.json and no usable flight log; falling back to 10 fps")
    else:
        fps = float(args.fps)

    out_path = Path(args.out) if args.out else (
        ROOT / "demo" / "out" / args.tag / f"{args.tag}_demo.mp4")

    def load(p, h):
        im = cv2.imread(p)
        if im is None:
            return None
        s = h / im.shape[0]
        return cv2.resize(im, (int(im.shape[1] * s), h))

    def panel_at(pa, pb):
        a = load(pa, args.height) if pa else None
        b = load(pb, args.height) if pb else None
        if args.layout == "pip":
            return picture_in_picture(a, b)
        return side_by_side(a, b)

    first = None
    for pa, pb in pairs:
        panel = panel_at(pa, pb)
        if panel is None:
            continue
        first = panel.shape
        break
    if first is None:
        print("frames present but none decodable")
        return 1

    # Written beside the target and moved into place only once checked
    # (module docstring). A missing directory is refused, not created.
    tmp = out_path.with_name(out_path.stem + ".partial" + out_path.suffix)
    writer = cv2.VideoWriter(str(tmp), cv2.VideoWriter_fourcc(*"mp4v"),
                             fps, (first[1], first[0]))
    if not writer.isOpened():
        print(f"[video] refused: cannot open {tmp} for writing (missing directory, "
              f"no write access, or no mp4v codec); nothing was written")
        return 1
    written = 0
    try:
        for pa, pb in pairs:
            panel = panel_at(pa, pb)
            if panel is None:
                continue
            if panel.shape[:2] != first[:2]:
                panel = cv2.resize(panel, (first[1], first[0]))
            writer.write(panel)
            written += 1
    except BaseException:
        writer.release()
        tmp.unlink(missing_ok=True)
        raise
    writer.release()
    got = 0
    if tmp.is_file() and tmp.stat().st_size > 0:
        cap = cv2.VideoCapture(str(tmp))
        got = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if cap.isOpened() else 0
        cap.release()
    if got != written:
        tmp.unlink(missing_ok=True)
        print(f"[video] refused: {written} frames written but {got} read back from "
              f"{tmp} (a full disk truncates the file); {out_path} was not touched")
        return 1
    try:
        os.replace(tmp, out_path)
    except PermissionError as e:
        tmp.unlink(missing_ok=True)
        print(f"[video] refused: cannot replace {out_path} ({e}) - it is open in "
              f"another program; the previous video is still there, close it and "
              f"run again")
        return 1
    print(f"[video] {written} frames at {fps} fps "
          f"({written/fps:.0f}s) -> {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
