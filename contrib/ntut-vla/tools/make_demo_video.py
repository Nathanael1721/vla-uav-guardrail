"""
Stitch a flight's recorded frames into one demo video.

Left: what the drone sees, with the detected box and the live telemetry drawn on
it. Right: the third-person chase view. Side by side, because the claim only
reads if both are visible at once — a word chose the box, the box drove the
aircraft, and the guardrail was underneath the whole time.

Run:
    python tools/make_demo_video.py --tag vlm_col
    python tools/make_demo_video.py --tag vlm_col --fps 12 --out demo.mp4
"""
from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    import cv2
    import numpy as np

    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True, help="flight tag under demo/out/")
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--out", default=None)
    ap.add_argument("--height", type=int, default=480)
    args = ap.parse_args()

    base = ROOT / "demo" / "out" / args.tag / "view"
    fpv = sorted(glob.glob(str(base / "fpv" / "*.jpg")))
    tps = sorted(glob.glob(str(base / "tps" / "*.jpg")))
    if not fpv and not tps:
        print(f"no frames under {base} — was the flight run with --save-view?")
        return 1

    out_path = Path(args.out) if args.out else (
        ROOT / "demo" / "out" / args.tag / f"{args.tag}_demo.mp4")

    def load(p, h):
        im = cv2.imread(p)
        if im is None:
            return None
        s = h / im.shape[0]
        return cv2.resize(im, (int(im.shape[1] * s), h))

    n = max(len(fpv), len(tps))
    first = None
    for i in range(n):
        a = load(fpv[min(i, len(fpv) - 1)], args.height) if fpv else None
        b = load(tps[min(i, len(tps) - 1)], args.height) if tps else None
        if a is None and b is None:
            continue
        panel = (np.hstack([a, b]) if a is not None and b is not None
                 else (a if a is not None else b))
        first = panel.shape
        break
    if first is None:
        print("frames present but none decodable")
        return 1

    writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"),
                             args.fps, (first[1], first[0]))
    written = 0
    for i in range(n):
        a = load(fpv[min(i, len(fpv) - 1)], args.height) if fpv else None
        b = load(tps[min(i, len(tps) - 1)], args.height) if tps else None
        if a is None and b is None:
            continue
        if a is not None and b is not None:
            if a.shape[0] != b.shape[0]:
                b = cv2.resize(b, (b.shape[1], a.shape[0]))
            panel = np.hstack([a, b])
        else:
            panel = a if a is not None else b
        if panel.shape[:2] != first[:2]:
            panel = cv2.resize(panel, (first[1], first[0]))
        writer.write(panel)
        written += 1
    writer.release()
    print(f"[video] {written} frames at {args.fps} fps "
          f"({written/args.fps:.0f}s) -> {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
