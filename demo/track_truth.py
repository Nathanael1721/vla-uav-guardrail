"""Was the box on the RIGHT thing? Scored against ground truth.

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

WHAT THE 2026-09-07 RETARGET FLIGHT ADDED

The first version scored every detection against `tgt_x/tgt_y`, which is the
CAR - the only ground truth the flight log carried. That is correct for a flight
whose subject never changes, and silently wrong for one that retargets: after
the subject became a pedestrian, all 319 remaining detections were compared to a
car that was by then behind the aircraft. Every one of them was flagged
"target out of FOV", `frac_on_target` came out 0.000 for that half of the
flight, and the flight-wide figure (0.406) was read as evidence that the
detector could not hold a pedestrian. It was not evidence of anything: 0.406 is
approximately 248/567, the fraction of the flight that happened BEFORE the
subject changed.

So a row now carries the truth for whatever the subject is at that tick, and a
row whose subject has no logged truth is counted as UNSCORABLE rather than
scored against the wrong object. Unscorable is reported. The one thing this
module must never do again is return a number when it has nothing to compare
against.
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


def truth_points(row: dict) -> Optional[list]:
    """Where the CURRENT subject truly is, as a list of acceptable positions.

    `None` means the subject's truth was not logged for this tick, and the row
    is unscorable. That is a different answer from "the box was wrong", and
    conflating the two is exactly the mistake this function exists to prevent.

    A list because "a person" does not name one person. The detector was asked
    for a class, so a box on any pedestrian is a correct answer to the question
    that was actually put to it; instance stability is what `TargetLock`
    measures, and it is a separate question with a separate number.
    """
    t = row.get("truth")
    if isinstance(t, dict):
        pts = [(float(a), float(b)) for a, b in (t.get("pts") or [])]
        return pts or None
    if row.get("tgt_x") is not None and row.get("tgt_y") is not None:
        return [(float(row["tgt_x"]), float(row["tgt_y"]))]
    return None


def score_rows(rows: Iterable[dict], hfov_deg: float = HFOV_DEG,
               on_target_px: float = ON_TARGET_PX) -> dict:
    """Tracking accuracy over a flight's per-tick rows.

    Rows are the flight-log records. A row counts only when it carries BOTH a
    detection and ground truth for the subject in force at that tick; ticks with
    no detection are the liveness question, which `det_hit_rate` already
    answers, and ticks with no truth are reported as `det_unscorable`.
    """
    errs: list[float] = []
    n_out_of_fov = 0
    n_scored = 0
    n_unscorable = 0

    for r in rows:
        det = r.get("det")
        if not det or r.get("psi") is None:
            continue
        pts = truth_points(r)
        if not pts:
            # A detection with nothing to compare it against. Counted, never
            # scored: the flight-wide figure that read as a detector failure was
            # 319 rows of exactly this.
            n_unscorable += 1
            continue
        img_w = int(det.get("img_w") or 400)
        best = None
        for tx, ty in pts:
            cx_gt, deg = project_target_cx(r["x"], r["y"], r["psi"],
                                           tx, ty, img_w, hfov_deg)
            err = abs(float(det["cx"]) - cx_gt)
            if best is None or err < best[0]:
                best = (err, deg)
        errs.append(best[0])
        # Out of shot only if EVERY acceptable subject was out of shot - with
        # one truth point this is the original test unchanged.
        if all(abs(project_target_cx(r["x"], r["y"], r["psi"], tx, ty,
                                     img_w, hfov_deg)[1]) > hfov_deg / 2.0
               for tx, ty in pts):
            n_out_of_fov += 1
        n_scored += 1

    if not errs:
        return {"det_scored": 0, "det_unscorable": n_unscorable,
                "det_gt_err_px_median": None,
                "det_gt_err_px_p95": None, "frac_on_target": None,
                "n_det_with_target_out_of_fov": 0}

    s = sorted(errs)
    return {
        "det_scored": n_scored,
        # Detections the log could not judge. A non-zero value here means
        # `frac_on_target` describes only part of the flight, and any claim made
        # from it has to say which part.
        "det_unscorable": n_unscorable,
        "det_gt_err_px_median": round(s[len(s) // 2], 1),
        "det_gt_err_px_p95": round(s[min(len(s) - 1, int(0.95 * len(s)))], 1),
        # THE number. What fraction of the boxes the controller acted on were
        # actually on the subject it was told to follow.
        "frac_on_target": round(sum(1 for e in errs if e <= on_target_px) / len(errs), 3),
        # The unambiguous failures: no acceptable subject was in shot, so
        # whatever the detector found, it was not the subject.
        "n_det_with_target_out_of_fov": n_out_of_fov,
    }
