"""NTUT-VLA labels from the expert's COMMAND, not from its motion.

    python -m training.ntutvla.build_labels --cache D:/vla_drone_offload/ntutvla/cache \\
        --out D:/vla_drone_offload/ntutvla/labels_cmd_v1 --holdout ft_expert_03 ft_dagger_pf_5

WHY (the first closed-loop flight, ntut_v0b_01, 29 Oct). v0's labels were
tools/build_follow_dataset's: the expert's DISPLACEMENT over the next 0.5 s.
vla_fly --step-as-rate flies a step as a velocity COMMAND (fwd/H m/s). The
airframe lags its command, so the displacement the expert achieved is
smaller than the speed it asked for, and a model that asks for what the
expert achieved asks for less each time. The aircraft fell from 9 m to 47 m
behind the car in 50 s and lost it. Here the label is what the expert ASKED
for at the capture's tick, written in the step's units over H so that the
flown rate equals the expert's command:

    fwd_m    = max(v_forward, 0) * H    v_forward: (vx, vy) on the capture's heading
    down_m   = -vz_up * H
    dpsi_rad = yaw_rate * H             (+ clockwise, as the codec)

`asked` is the expert's own command before any perturbation (a DART offset
or a DAgger window replaced only the EXECUTED `cmd`). So every perturbed or
VLA-flown capture keeps its label: the expert's answer in the state the
perturbation led to, which is the point of DART and DAgger. v0 dropped
22,390 such samples.

A capture is a sample when its frame exists, it has an expert tick, and the
expert has acquired the car (the first capture with a box no older than
FRESH_S; before it the expert waits at its start gate, a behaviour vla_fly
never asks a VLA for). hint_kind is "fresh" for a box no older than FRESH_S
at the capture, which with its bearing feeds the head's auxiliary outputs
only. The record of each run is the one build_cache read (its info.json).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from training.ntutvla import core                                   # noqa: E402

FRESH_S = 1.0
HFOV_DEG = 90.0          # the detector's FrontCamera, whose frame the box is in


def box_bearing_deg(det) -> float:
    """The bearing of a detector box's centre, + right, on a pinhole camera of
    HFOV_DEG (det = [cx, cy, w, h, score, W, H, ...])."""
    cx, w_img = float(det[0]), float(det[5])
    return math.degrees(math.atan((cx - w_img / 2.0) / (w_img / 2.0)
                                  * math.tan(math.radians(HFOV_DEG / 2.0))))


def run_samples(tag: str, rec: Path, frame_ok: np.ndarray, rows_of: dict, split: str,
                horizon_s: float) -> tuple[list[dict], dict]:
    caps = sorted((json.loads(l) for l in (rec / "captures.jsonl").read_text(
        encoding="utf-8").splitlines() if l.strip()), key=lambda c: c["t_capture"])
    out, why = [], {"no_frame": 0, "no_tick": 0, "before_acquire": 0, "kept": 0}
    acquired = False
    for c in caps:
        box = c.get("box")
        fresh = bool(box) and (c["t_capture"] - box["t_box"]) <= FRESH_S
        acquired = acquired or fresh
        k = rows_of.get(int(c["i"]))
        if k is None or not frame_ok[k]:
            why["no_frame"] += 1
            continue
        tick = c.get("tick")
        if not tick or not tick.get("asked"):
            why["no_tick"] += 1
            continue
        if not acquired:
            why["before_acquire"] += 1
            continue
        a = tick["asked"]
        yaw = float(c["pose"]["yaw_rad"])
        v_fwd = a["vx"] * math.cos(yaw) + a["vy"] * math.sin(yaw)
        lab = {"fwd_m": max(v_fwd, 0.0) * horizon_s, "down_m": -a["vz_up"] * horizon_s,
               "dpsi_rad": a["yaw_rate"] * horizon_s}
        s = {"id": f"{tag}/{c['i']}", "run": tag, "split": split, "i": int(c["i"]),
             "t_capture": c["t_capture"], "label_raw": lab, "label_kind": "expert_asked",
             "v_right": -a["vx"] * math.sin(yaw) + a["vy"] * math.cos(yaw),
             "mode": tick.get("mode"), "hint_kind": "fresh" if fresh else None,
             "box": {"bearing_deg": box_bearing_deg(box["det"])} if fresh else None,
             "executed_differs": tick.get("cmd") != a}
        out.append(s)
        why["kept"] += 1
    return out, why


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cache", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--runs", nargs="*", default=None,
                    help="cached run tags (default every run in --cache)")
    ap.add_argument("--holdout", nargs="*", default=[])
    ap.add_argument("--horizon-s", type=float, default=core.LABEL_HORIZON_S)
    args = ap.parse_args(argv)
    cache = Path(args.cache)
    runs = args.runs or sorted(p.name for p in cache.iterdir() if (p / "info.json").exists())
    bad = [h for h in args.holdout if h not in runs]
    if bad:
        raise SystemExit(f"--holdout {bad}: not among the cached runs")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    allx, stats = [], {}
    for tag in runs:
        info = json.loads((cache / tag / "info.json").read_text(encoding="utf-8"))
        meta = np.load(cache / tag / "meta.npz")
        rows_of = {int(i): k for k, i in enumerate(meta["i"])}
        rec = Path(info["record"])
        if not rec.is_absolute():
            rec = ROOT / rec
        split = "holdout" if tag in args.holdout else "train"
        s, why = run_samples(tag, rec, meta["frame_ok"], rows_of, split, args.horizon_s)
        allx += s
        d = np.array([x["label_raw"]["dpsi_rad"] for x in s]) if s else np.zeros(0)
        stats[tag] = {**why, "split": split,
                      "turning_share": round(float((np.abs(d) > 0.05).mean()), 3) if s else None,
                      "executed_differs": sum(1 for x in s if x["executed_differs"]),
                      "modes": {m: sum(1 for x in s if x["mode"] == m)
                                for m in sorted({x["mode"] or "none" for x in s})}}
        print(f"[run] {tag:16s} {split:7s} {json.dumps(stats[tag])}")
    p = out / "samples.jsonl"
    with open(p, "w", encoding="utf-8", newline="\n") as fh:
        for x in allx:
            fh.write(json.dumps(x) + "\n")
    n_tr = sum(1 for x in allx if x["split"] == "train")
    summary = {"n_train": n_tr, "n_holdout": len(allx) - n_tr, "horizon_s": args.horizon_s,
               "label": "expert asked command x horizon (module docstring)", "runs": stats,
               "samples_sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
    (out / "stats.json").write_text(json.dumps(summary, indent=1) + "\n", encoding="utf-8")
    print(f"[labels] {n_tr} train / {len(allx) - n_tr} hold-out -> {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
