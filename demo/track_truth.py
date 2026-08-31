"""Was the box on the RIGHT car? Scored against ground truth.

WHY THIS EXISTS

`det_hit_rate` does not answer that question and never did. It is
`n_seen / (n_seen + n_miss)` where `n_seen` increments whenever the detector
emitted any box at all (`demo/follow_vlm.py`), so a box on a parked lookalike,
on a building, or on road paint counts exactly like a box on the taxi. It is a
detector-LIVENESS rate.

That distinction was not academic. Measured across the flights on disk, the
reported rate is anti-correlated with actual correctness:

    flight        det_hit_rate   median err   p95      >100 px   taxi off-frame
    city_full        0.995          8.2 px   216.8 px   27 %        37 rows
    city_kpi         1.000          3.6 px   113.2 px    7 %        12 rows
    demo_traffic     0.977          4.9 px    48.9 px    0 %         0
    city_demo        1.000          2.7 px    27.4 px    0 %         0

The flight with the BEST reported hit rate spent a quarter of its detections
more than 100 px from the taxi, including 37 frames where the taxi was outside
the camera's field of view entirely and the controller followed a box anyway.

The ground truth was in the flight log the whole time. Nothing here needs new
instrumentation - only the arithmetic that was never done.
"""
from __future__ import annotations

import math
from typing import Iterable, Optional

# The Chase/front camera's horizontal field of view, degrees. Matches the
# value TargetLock uses to convert a yaw change into pixels.
HFOV_DEG = 90.0

# How far the chosen box may sit from the projected target and still count as
# "on target". At 400 px across a 90 deg view, 100 px is 22.5 deg - far wider
# than any plausible box-centre error on the right vehicle (median is 3-8 px),
# and narrow enough that a different vehicle in another lane fails it.
ON_TARGET_PX = 100.0


def project_target_cx(x: float, y: float, psi: float,
                      tgt_x: float, tgt_y: float,
                      img_w: int, hfov_deg: float = HFOV_DEG):
    """Where the true target should appear horizontally, in pixels.

    Returns (cx, bearing_deg). `cx` may fall outside [0, img_w] - that is the
    informative case, because it means the target is not in shot at all and any
    box the detector produced is on something else.

    A pinhole approximation: a bearing of `b` degrees off the nose lands at
    `img_w * (0.5 + b / hfov)`. Validated against the `bearing_deg` the flight
    log already records, which it reproduces to a few degrees - well inside the
    100 px (22.5 deg) tolerance being judged.
    """
    rel = math.atan2(tgt_y - y, tgt_x - x) - psi
    rel = math.atan2(math.sin(rel), math.cos(rel))       # wrap to (-pi, pi]
    deg = math.degrees(rel)
    return img_w * (0.5 + deg / hfov_deg), deg


def score_rows(rows: Iterable[dict], hfov_deg: float = HFOV_DEG,
               on_target_px: float = ON_TARGET_PX) -> dict:
    """Tracking accuracy over a flight's per-tick rows.

    Rows are the flight-log records. A row counts only when it carries BOTH a
    detection and a target position; ticks with no detection are the
    liveness question, which `det_hit_rate` already answers.
    """
    errs: list[float] = []
    n_out_of_fov = 0
    n_scored = 0

    for r in rows:
        det = r.get("det")
        if not det or r.get("tgt_x") is None or r.get("psi") is None:
            continue
        img_w = int(det.get("img_w") or 400)
        cx_gt, deg = project_target_cx(r["x"], r["y"], r["psi"],
                                       r["tgt_x"], r["tgt_y"], img_w, hfov_deg)
        errs.append(abs(float(det["cx"]) - cx_gt))
        if abs(deg) > hfov_deg / 2.0:
            n_out_of_fov += 1
        n_scored += 1

    if not errs:
        return {"det_scored": 0, "det_gt_err_px_median": None,
                "det_gt_err_px_p95": None, "frac_on_target": None,
                "n_det_with_target_out_of_fov": 0}

    s = sorted(errs)
    return {
        "det_scored": n_scored,
        "det_gt_err_px_median": round(s[len(s) // 2], 1),
        "det_gt_err_px_p95": round(s[min(len(s) - 1, int(0.95 * len(s)))], 1),
        # THE number. What fraction of the boxes the controller acted on were
        # actually on the vehicle it was told to follow.
        "frac_on_target": round(sum(1 for e in errs if e <= on_target_px) / len(errs), 3),
        # The unambiguous failures: the target was not in shot, so whatever the
        # detector found, it was not the target.
        "n_det_with_target_out_of_fov": n_out_of_fov,
    }
