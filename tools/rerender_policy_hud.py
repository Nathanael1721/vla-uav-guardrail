"""Re-draw the policy indicator onto a recorded flight, from its own log.

    python tools/rerender_policy_hud.py --tag citylife_redcar_id4 \
        --policy policies/follow_car_citylife.yaml --from-s 95 --to-s 110 \
        --out docs/img/policy_hud_preview.jpg

    python tools/rerender_policy_hud.py --tag citylife_redcar_id4 \
        --policy policies/follow_car_citylife.yaml --video out.mp4

Two uses. It previews demo/policy_hud.py on real decisions without flying,
and it lets a flight flown before the indicator existed be shown with it.
Everything drawn comes from that flight's flight_log.jsonl: the ShieldDecision
fields (violations, repairs, braked, emitted_violations), the estimator's
subject position (`est_xy`, the same value the Shield was given), the
controller's fence_mode, and the pose. Clearance is re-read from the map the
flight's metrics.json names (params.citymap) when that file still exists, else
from --citymap; either way the map file may be NEWER than the flight (CityLife's
was rebuilt on 2026-09-24), and the tool says so when it is, because clearance
and BREACH drawn from a later map are not what that flight's Shield measured.
Frames that were recorded with the old HUD keep its burned-in box and label on
the target; the new panels cover the old text.

Logs from before 2026-10-03 do not record which hazard set fence_mode. With no
polygon fence in the policy it can only have been an obstacle, which is what
this assumes; with a fence the cause is passed as unknown, and the banner says
"cause not logged" rather than blaming either.

A frame is mapped to flight time by the recorder's per-frame stamp
(`frame_t` in recorder.json, from 2026-10-03); older recordings fall back to
index / achieved_hz, which drifts by a few seconds over a flight. The recorder
runs at about 20 Hz against a 10 Hz log, so each frame takes the latest log
row at or before its time.
"""
from __future__ import annotations

import argparse
import bisect
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import Shield, State, load_policy            # noqa: E402
from guardrail.shield import ShieldDecision                 # noqa: E402
from policy_hud import (PolicyIndicator, draw_policy_overlay,  # noqa: E402
                        draw_status_box, render_base_map)


def load_rows(tag: str) -> list[dict]:
    p = ROOT / "demo" / "out" / tag / "flight_log.jsonl"
    rows = [json.loads(ln) for ln in p.read_text(encoding="utf-8").splitlines() if ln.strip()]
    return [r for r in rows if "tick" in r and "raw" in r]


def decision_of(r: dict) -> ShieldDecision:
    return ShieldDecision.model_validate({
        "raw": r["raw"], "emitted": r["emitted"],
        "violations": r.get("violations") or [], "repairs": r.get("repairs") or [],
        "braked": bool(r.get("braked")),
        "emitted_violations": r.get("emitted_violations") or []})


def status_lines(r: dict) -> list[str]:
    """The top-left flight lines, rebuilt from a log row. Covers the old HUD's
    burned-in text on frames recorded before the indicator existed."""
    import math
    sep = None
    pts = (r.get("truth") or {}).get("pts") or []
    if pts:
        sep = min(math.hypot(r["x"] - p[0], r["y"] - p[1]) for p in pts)
    mode = {"track": "TARGET LOCKED", "coast": "COASTING on last motion",
            "search": "SEARCHING ...", "scan": "SCANNING for target"}.get(
        r.get("mode"), str(r.get("mode") or "").upper())
    if r.get("mode") == "track" and r.get("tier") == "soft":
        mode = "TRACKING (PARTLY HIDDEN)"
    return [f"t {r['t']:5.1f}s   alt {r['up']:4.1f} m",
            f"bearing {r.get('bearing_deg', 0.0):+6.1f}deg   "
            f"speed {math.hypot(r['emitted']['vx'], r['emitted']['vy']):4.1f} m/s",
            f"separation {sep:5.1f} m" if sep is not None else "separation n/a",
            mode]


def snapshots(rows, policy, citymap: Path | None):
    """One indicator snapshot per log row, in order."""
    import numpy as np
    smap, street = None, None
    if citymap is not None and citymap.is_file():
        z = np.load(citymap)
        smap = {"occ": z["occ"], "res": float(z["res"]),
                "ox": float(z["origin_x"]), "oy": float(z["origin_y"])}
        sm = citymap.parent / "street.npz"
        if sm.is_file():
            zs = np.load(sm)
            street = {"street": zs["street"]}
    shield = Shield(policy, lookahead_s=3.0, dt=0.5, obstacle_map=smap)
    has_fence = any(getattr(c, "type", "") == "polygon_fence" for c in policy.constraints)
    if has_fence:
        print("[rerender] policy has a fence and the log may not say which hazard "
              "held; logs before 2026-10-03 are shown with the cause unknown")
    ind = PolicyIndicator(policy, base_map=render_base_map(smap, street))
    out = []
    for r in rows:
        st = State(x=r["x"], y=r["y"], up=r["up"])
        est = r.get("est_xy")
        cause = r.get("fence_cause")
        if cause is None and r.get("fence_mode") in ("skirt", "hold", "near") and not has_fence:
            cause = "obstacle"
        cls = (r.get("truth") or {}).get("class") if est is not None else None
        out.append(ind.update(
            r["t"], decision_of(r), st, subject_xy=est, subject_class=cls,
            clearance_m=(shield.clearance_at(st.x, st.y) if smap else None),
            off_map=shield.off_map(st.x, st.y),
            fence_cause=cause, fence_mode=r.get("fence_mode"), est_xy=est,
            yaw_rad=r.get("psi")))
    return out, ind


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--tag", required=True)
    ap.add_argument("--policy", required=True)
    ap.add_argument("--citymap", default=str(ROOT / "demo/out/citymap_citylife/occ_day.npz"))
    ap.add_argument("--from-s", type=float, default=0.0)
    ap.add_argument("--to-s", type=float, default=1e9)
    ap.add_argument("--out", help="write ONE frame (the first in range) here")
    ap.add_argument("--video", help="write the range as an mp4 here (needs ffmpeg)")
    ap.add_argument("--fps", type=float, default=20.0)
    args = ap.parse_args()

    rows = load_rows(args.tag)
    policy = load_policy(args.policy)
    cmap = Path(args.citymap) if args.citymap else None
    mp = ROOT / "demo" / "out" / args.tag / "metrics.json"
    if mp.is_file():
        flown = (json.loads(mp.read_text(encoding="utf-8")).get("params") or {}).get("citymap")
        if flown and Path(flown).is_file():
            cmap = Path(flown)
    log = ROOT / "demo" / "out" / args.tag / "flight_log.jsonl"
    if cmap is not None and cmap.is_file() and cmap.stat().st_mtime > log.stat().st_mtime:
        print(f"[rerender] WARNING {cmap} is newer than this flight; clearance and "
              f"BREACH are drawn from a map the flight's Shield did not use")
    snaps, ind = snapshots(rows, policy, cmap)
    ts = [r["t"] for r in rows]
    print(f"[rerender] {len(rows)} ticks; counts {ind.counts}; banner ticks {ind.banner_ticks}")

    vdir = ROOT / "demo" / "out" / args.tag / "view"
    rec = json.loads((vdir / "recorder.json").read_text(encoding="utf-8"))
    hz = float(rec.get("achieved_hz") or rec.get("target_hz") or 20.0)
    # Recorders from 2026-10-03 stamp each frame with the flight time it
    # showed; older ones only allow index / rate, which drifts by seconds.
    ft = rec.get("frame_t")

    def t_of(i: int) -> float:
        if ft and i < len(ft) and ft[i] is not None:
            return float(ft[i])
        return i / hz

    frames = sorted((vdir / "fpv").glob("*.jpg"))
    pick = [f for f in frames if args.from_s <= t_of(int(f.stem)) <= args.to_s]
    if not pick:
        print("[rerender] no frames in range")
        return 1

    from PIL import Image

    def render(f: Path):
        t = t_of(int(f.stem))
        k = max(0, bisect.bisect_right(ts, t) - 1)
        im = Image.open(f).convert("RGB")
        # Blank the old HUD's burned-in lines before the new box goes on.
        from PIL import ImageDraw
        sc = im.size[0] / 1280.0
        ImageDraw.Draw(im).rectangle([0, 0, 336 * sc, 112 * sc], fill=(12, 12, 16))
        draw_status_box(im, status_lines(rows[k]))
        draw_policy_overlay(im, snaps[k])
        return im

    if args.out:
        render(pick[0]).save(args.out, quality=92)
        print(f"[rerender] wrote {args.out}")
    if args.video:
        tmp = ROOT / "demo" / "out" / args.tag / "rerender_hud"
        tmp.mkdir(exist_ok=True)
        # A shorter range must not pick up the last render's frames at its tail.
        for old in tmp.glob("*.jpg"):
            old.unlink()
        for i, f in enumerate(pick):
            render(f).save(tmp / f"{i:05d}.jpg", quality=90)
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(args.fps),
                        "-i", str(tmp / "%05d.jpg"), "-c:v", "libx264", "-pix_fmt", "yuv420p",
                        "-crf", "24", args.video], check=True)
        print(f"[rerender] wrote {args.video} ({len(pick)} frames)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
