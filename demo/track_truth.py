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

(The `frac_on_target` figures those rows imply moved slightly on 2026-09-08 -
city_kpi 0.930 -> 0.908, the retarget flight's first half 0.931 -> 0.915 - when
rows whose subject was out of shot stopped being credited. See ON_TARGET_PX.)

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

AND THE THING THAT COST THAT FIX ITS MEANING

Scoring against a CLASS is not scoring against an object, and the first version
of the multi-point path did not admit the difference. It took the minimum error
over every logged figure, so the more pedestrians in the scene the easier the
test became. Measured with a detector of literally zero skill - a uniformly
random box centre:

    truth points     1       2       4       8      12
    frac_on_target  0.132   0.212   0.360   0.494   0.584
    median err      393 px  275 px  176 px  102 px   71 px

At twelve figures a random box beats the 100 px tolerance more often than not,
and the median lands INSIDE it. `ON_TARGET_PX` was calibrated against a single
target (median error on the right vehicle is 3-8 px) and that calibration does
not survive a candidate set.

Two changes make the number mean something again. Only candidates actually IN
SHOT can be credited, because a box cannot be on a subject the camera cannot
see. And every result carries `frac_on_target_chance` - the same computation
over the same rows with the box centre replaced by a seeded uniform draw - so a
reader can see the floor this flight's geometry sets. A score near its own
chance level is not tracking, however high it looks.
"""
from __future__ import annotations

import math
import random
from typing import Iterable, Optional

# The Chase/front camera's horizontal field of view, degrees. Matches the
# value TargetLock uses to convert a yaw change into pixels.
HFOV_DEG = 90.0

# How far the chosen box may sit from the projected target and still count as
# "on target". At 400 px across a 90 deg view, 100 px is 22.5 deg - far wider
# than any plausible box-centre error on the right vehicle (median is 3-8 px),
# and narrow enough that a different vehicle in another lane fails it.
#
# THAT WIDTH IS ALSO WHY THE FIGURE ALONE PROVES LITTLE. The aircraft yaws to
# point AT what it is following, so the subject sits near the frame centre, and
# a detector that emits the centre on every frame - never opening the image -
# scores almost the same. Measured on demo/out/city_full, in-shot rows only:
#
#     tolerance   real detector   centre constant   margin
#        100 px       0.779            0.806        -0.027   <- constant WINS
#         50 px       0.747            0.735        +0.011
#         25 px       0.728            0.665        +0.063
#         10 px       0.581            0.469        +0.112
#
# The real detector IS better - median error 7.1 px against the constant's 11.6 -
# but 100 px cannot see it. So the 100 px figure is kept for continuity with the
# forty-odd published flights, and it is never reported alone: the floor, the
# margin over it, and a 25 px companion go with it.
ON_TARGET_PX = 100.0

# The discriminating tolerance. 25 px is 5.6 deg, still three times the median
# error of a correct box, and it separates the detector from a constant by a
# margin that survives.
TIGHT_ON_TARGET_PX = 25.0


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


def _score(rows, hfov_deg, on_target_px, cx_of):
    """The scoring loop. `cx_of(row)` supplies the box centre being judged.

    Factored out so the chance floor is computed by THE SAME code with a random
    box rather than by a second implementation that could disagree with it.
    """
    errs: list[float] = []
    creditable: list[bool] = []      # was any acceptable subject in frame at all
    cand_counts: list[int] = []
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
        cx = cx_of(r, img_w)

        # Project every candidate once, then split them by whether the camera
        # could see them at all.
        projected = [project_target_cx(r["x"], r["y"], r["psi"], tx, ty,
                                       img_w, hfov_deg) for tx, ty in pts]
        in_shot = [(abs(cx - c), d) for c, d in projected
                   if abs(d) <= hfov_deg / 2.0]

        if in_shot:
            # Credit only a subject that was actually in frame. Taking the min
            # over ALL candidates let a row be scored ON TARGET against a
            # subject behind the aircraft while the out-of-shot counter - which
            # exists to catch exactly that - stayed silent, because the two
            # tests could be satisfied by different candidates.
            errs.append(min(e for e, _ in in_shot))
            creditable.append(True)
            cand_counts.append(len(in_shot))
        else:
            # Nothing acceptable was in shot, so whatever the detector found, it
            # was not the subject - and it must not be counted on target however
            # small the pixel error comes out.
            #
            # "However small" is not hypothetical. ON_TARGET_PX is 100 px = 22.5
            # deg, which is HALF the 45 deg half-FOV, so a subject up to 67.5 deg
            # off the nose still lands within tolerance of a box at the frame
            # edge. On demo/out/city_kpi all 12 out-of-shot rows scored inside
            # the tolerance, and were counted in n_det_with_target_out_of_fov and
            # in the frac_on_target numerator at once - the previous comment here
            # asserted "it will be large", which the artefact refutes.
            errs.append(min(abs(cx - c) for c, _ in projected))
            creditable.append(False)
            cand_counts.append(0)
            n_out_of_fov += 1
        n_scored += 1

    return errs, creditable, cand_counts, n_out_of_fov, n_scored, n_unscorable


def score_rows(rows: Iterable[dict], hfov_deg: float = HFOV_DEG,
               on_target_px: float = ON_TARGET_PX,
               chance_seed: int = 20260908) -> dict:
    """Tracking accuracy over a flight's per-tick rows.

    Rows are the flight-log records. A row counts only when it carries BOTH a
    detection and ground truth for the subject in force at that tick; ticks with
    no detection are the liveness question, which `det_hit_rate` already
    answers, and ticks with no truth are reported as `det_unscorable`.

    `frac_on_target_chance` is the same statistic computed with the box centre
    replaced by a seeded uniform draw. It is the floor this flight's geometry
    sets, and `frac_on_target` means nothing without it once the subject is a
    class rather than an object.
    """
    rows = list(rows)
    errs, credit, cands, n_out_of_fov, n_scored, n_unscorable = _score(
        rows, hfov_deg, on_target_px, lambda r, w: float(r["det"]["cx"]))

    if not errs:
        return {"det_scored": 0, "det_unscorable": n_unscorable,
                "det_gt_err_px_median": None,
                "det_gt_err_px_p95": None, "frac_on_target": None,
                "frac_on_target_chance": None,
                "truth_candidates_median": None,
                "n_det_with_target_out_of_fov": 0}

    # TWO null models, because one of them was measured to be far too weak.
    #
    # `uniform` throws a box anywhere in the frame. `centre` puts it at the
    # image centre on every frame - a detector that never opens the image and
    # cannot tell a car from a wall. Since the aircraft YAWS TO POINT AT the
    # subject, the subject's true cx is concentrated near the centre (measured
    # mean cx/img_w = 0.51), so the constant detector is the stronger null by a
    # wide margin, and on `city_full` it beats the real detector while sailing
    # past the uniform floor. Quoting only the uniform floor would have let a
    # score that is worse than a constant look like tracking.
    #
    # `frac_on_target_chance` reports the HARDER of the two, because a floor is
    # only useful if it is the highest one a zero-skill detector can reach.
    rnd = random.Random(chance_seed)
    e_uniform, c_uniform, _, _, _, _ = _score(
        rows, hfov_deg, on_target_px, lambda r, w: rnd.uniform(0.0, float(w)))
    e_centre, c_centre, _, _, _, _ = _score(
        rows, hfov_deg, on_target_px, lambda r, w: float(w) / 2.0)

    def _frac_at(es, cs, tol):
        return sum(1 for e, ok in zip(es, cs) if ok and e <= tol) / len(es)

    def _frac(es, cs):
        return _frac_at(es, cs, on_target_px)

    chance_uniform = _frac(e_uniform, c_uniform)
    chance_centre = _frac(e_centre, c_centre)
    tight_chance = max(_frac_at(e_uniform, c_uniform, TIGHT_ON_TARGET_PX),
                       _frac_at(e_centre, c_centre, TIGHT_ON_TARGET_PX))

    s = sorted(errs)
    c = sorted(cands)
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
        "frac_on_target": round(_frac(errs, credit), 3),
        # The floor: the best a detector with NO skill scores on these same rows,
        # taken as the harder of two null models (see above). Measured on the
        # flights in this repository it runs 0.47-0.55 even with a single
        # subject, because the aircraft points at what it is following - so a
        # score is evidence of tracking only by the margin it clears this.
        "frac_on_target_chance": round(max(chance_uniform, chance_centre), 3),
        "frac_on_target_chance_uniform": round(chance_uniform, 3),
        "frac_on_target_chance_centre": round(chance_centre, 3),
        # THE EVIDENCE, as opposed to the number. How far the detector clears
        # the best a zero-skill detector manages on these same rows. Negative
        # means a constant would have done better, which has happened.
        "frac_on_target_margin": round(
            _frac(errs, credit) - max(chance_uniform, chance_centre), 3),
        # The same three at a tolerance tight enough to tell them apart.
        "frac_on_target_25px": round(_frac_at(errs, credit, TIGHT_ON_TARGET_PX), 3),
        "frac_on_target_25px_chance": round(tight_chance, 3),
        "frac_on_target_25px_margin": round(
            _frac_at(errs, credit, TIGHT_ON_TARGET_PX) - tight_chance, 3),
        # How many acceptable subjects were in shot. Reported as a RANGE, not a
        # median: a retarget flight is bimodal by construction - one subject
        # before the switch, twelve after - and a median reports whichever mode
        # holds one more row, so a single row could flip the reader's verdict on
        # whether the figure above is comparable with a single-target flight.
        "truth_candidates_min": c[0],
        "truth_candidates_max": c[-1],
        # The unambiguous failures: no acceptable subject was in shot, so
        # whatever the detector found, it was not the subject.
        "n_det_with_target_out_of_fov": n_out_of_fov,
    }
