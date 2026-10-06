"""
Follow a named object with vision and language, through the guardrail.

You type "a car". A vision-language model finds that thing in the camera image.
A servo loop keeps it centred and at a chosen distance. The Shield still checks
every action. Nothing in the steering path knows the target's coordinates.

Why this and not AerialVLA
--------------------------
AerialVLA was measured, over 108 controlled forward passes and several flights,
to ignore its object-description slot entirely: correct and wrong colour words
produce indistinguishable actions, and with no coordinate-derived bearing phrase
it emits stop-and-land on 11 of 18 real frames. Flown against a real car mesh it
never moved at all — 12 of 12 inferences returned the same token triple. See
docs/FINDING-what-drives-aerialvla.md.

So the language grounding is done by a model that actually does it. OWL-ViT is an
open-vocabulary detector: text in, boxes out. Measured here at 34-56 ms per frame
against AerialVLA's 11-12 s, which also removes the latency problem that made
closed-loop tracking impossible.

This is still vision-language-action — the words choose the target, the camera
finds it, the controller acts — with the model swapped for one whose language
input reaches the output.

What the controller does
------------------------
    horizontal box offset  -> yaw rate      (turn to centre the target)
    box width vs desired   -> forward speed (hold a standoff distance)
    altitude error         -> climb rate    (hold the cruise height)

The altitude term is deliberate: the earlier scripts had none and leaned on the
Shield, which is a constraint filter making minimal corrections, not a
controller. Flights sagged tens of seconds below the floor as a result.

Run:
    python demo/follow_vlm.py --object "a car" --tag vlmfollow
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import random
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import AuditLogger, Shield, State, load_policy       # noqa: E402
from guardrail import kpi as kpi_mod                                # noqa: E402
from guardrail.bundle import load_for_flight                        # noqa: E402
from guardrail.manifest import build_manifest, is_kpi_grade         # noqa: E402
from guardrail.replay import verify_replay, write_replay           # noqa: E402
from guardrail.geometry import fence_polygon                        # noqa: E402
from guardrail.models import (                                      # noqa: E402
    Action4D, AltitudeEnvelope, ObstacleClearance, PolygonFence,
    SubjectStandoff,
)
from shapely.geometry import Point                                  # noqa: E402

import city_planner                                                 # noqa: E402
import occ_bands                                                    # noqa: E402
import city_traffic
import track_truth
from recorder import FrameRecorder
import car_trajectory
import pedestrians as people_mod
from target_state import TargetState, want_range_from_width
import trail as trail_mod
import moving_car                                                   # noqa: E402
from aerialvla_demo import RateLimiter                              # noqa: E402
from semantic_demo import SemanticObs, quat_yaw                     # noqa: E402

TICK = 0.1
SIM_CONFIG_DIR = str(ROOT / "demo" / "pas_config")
SCENE = "scene_semantic.jsonc"
ROBOT_CONFIG = ROOT / "demo" / "pas_config" / "robot_semantic_quad.jsonc"
# The YAML a flight falls back to when neither --policy nor --bundle is given.
DEFAULT_POLICY = ROOT / "policies" / "follow_car.yaml"


def camera_hfov_deg(config_path=ROBOT_CONFIG, sensor: str = "FrontCamera",
                    image_type: int = 0, default: float = 90.0) -> float:
    """The detector camera's horizontal FOV, READ FROM THE SIM CONFIG.

    It was a literal in six places, and in the one that mattered most it was
    written as `math.radians(45.0)` - the HALF-angle, inlined, in the estimator
    feed - while `servo()` three hundred lines away took `hfov_deg` as a
    parameter. Change the camera and those two disagree silently: every bearing
    the estimator is fed would be scaled wrong, the Shield would be served a
    subject in the wrong direction, and nothing would raise. That is the exact
    shape of the yaw-units defect this project has already published a finding
    about, waiting to happen a second time.

    Reading it here means the camera can be re-aimed by editing the config
    alone, which is the whole point: narrowing the FOV is the only lever that
    makes a 0.5 m subject bigger to the model, and it must not require six
    correct edits to attempt.

    Falls back to `default` with a warning rather than raising, because a
    missing config must not stop a flight that would otherwise be correct - but
    it says so, because a silent fallback is how the last one hid.
    """
    try:
        raw = Path(config_path).read_text(encoding="utf-8")
        # JSONC: strip // comments, keeping any inside strings alone. The
        # comments in this file are all full-line or trailing, never in a
        # string, so a line-wise strip is exact here.
        stripped = chr(10).join(
            (ln.split("//", 1)[0] if '"' not in ln.split("//", 1)[0] or
             ln.strip().startswith("//") else ln)
            for ln in raw.splitlines())
        cfg = json.loads(stripped)
        for sen in cfg.get("sensors", []):
            if sen.get("id") != sensor:
                continue
            for cap in sen.get("capture-settings", []):
                if int(cap.get("image-type", -1)) == image_type:
                    return float(cap["fov-degrees"])
    except Exception as exc:                                   # noqa: BLE001
        print(f"[camera] WARNING could not read fov from {config_path}: "
              f"{type(exc).__name__}: {exc}; using {default} deg", flush=True)
        return float(default)
    print(f"[camera] WARNING no {sensor} image-type {image_type} in "
          f"{config_path}; using {default} deg", flush=True)
    return float(default)


# Resolved once, at import. Every bearing computation in this module uses it.
CAMERA_HFOV_DEG = camera_hfov_deg()

# PIXEL -> ANGLE, IN ONE PLACE (demo/camera_model.py).
#
# Every bearing and every box width used to be `hfov/2 * offset/(W/2)`, a LINEAR
# map. The camera is a pinhole: at 90 deg HFOV the two agree only at the centre
# and the edges, differ by ~4 deg in between, and the linear map made every
# physical width ~21 % small (found 2026-09-29). `--linear-bearing` restores it,
# for reproducing flights recorded before the change.
import camera_model as _cam  # noqa: E402
LINEAR_BEARING = False


def box_bearing(cx: float, img_w: float, hfov_deg: float = None) -> float:
    """Angle (rad) of image column `cx` off the nose; positive right."""
    hfov = CAMERA_HFOV_DEG if hfov_deg is None else hfov_deg
    if LINEAR_BEARING:
        return math.radians(hfov / 2.0) * ((cx - img_w / 2.0) / (img_w / 2.0))
    return _cam.pixel_bearing(cx, img_w, hfov)


def box_half_angle(bw: float, img_w: float, hfov_deg: float = None,
                   cx: float = None) -> float:
    """Half the angle (rad) a box of width `bw` px subtends, centred on `cx`
    (the frame centre when not given)."""
    hfov = CAMERA_HFOV_DEG if hfov_deg is None else hfov_deg
    if LINEAR_BEARING:
        return math.radians(hfov / 2.0) * (bw / img_w)
    c = img_w / 2.0 if cx is None else cx
    return _cam.box_angular_width(c, bw, img_w, hfov) / 2.0
# Declared in that scene file. The simulator drives this actor along an uploaded
# trajectory; see demo/env_actor_car.jsonc and demo/car_trajectory.py.
ENV_CAR_NAME = "SemCarActor"
DETECTOR_ID = "google/owlvit-base-patch32"


# Hue ranges in OpenCV's 0-179 scale, plus how saturated a pixel must be to count
# as that colour at all. Grey road and pale concrete have low saturation, so the
# saturation floor does most of the rejecting.
COLOUR_HUE = {
    "red": [(0, 10), (170, 179)], "orange": [(8, 24)], "yellow": [(22, 35)],
    "green": [(36, 85)], "blue": [(90, 130)], "purple": [(130, 160)],
}
COLOUR_ACHROMATIC = {"white": "white", "black": "black", "grey": "grey", "gray": "grey"}

# How much absolute colour a pixel must carry to count as its hue, on the 0-255
# scale of max(R,G,B) - min(R,G,B). Replaces a saturation floor, which is chroma
# divided by brightness and therefore collapses in sunlight; see colour_match.
# 40 admits the sunlit taxi (0.174 of its pixels) and still rejects road and
# zebra crossing outright (0.0004).
COLOUR_MIN_CHROMA = 40.0


def colour_word(query: str):
    q = query.lower()
    for w in list(COLOUR_HUE) + list(COLOUR_ACHROMATIC):
        if w in q:
            return w
    return None


# Real-world width of the subject, used to turn an apparent box width into a
# range whenever depth is unusable - which is most of the time, because the
# depth stream quantises to whole metres.
#
# This was a single hardcoded 4.0, a car, applied to whatever was named. The
# 2026-08-19 review asked for a pedestrian in the scene next, and a pedestrian
# is about 0.5 m wide: the same code would have reported a person at roughly
# EIGHT TIMES their true distance, and the aircraft would have flown that far
# in to close the gap. The pedestrian demo is meaningless until the width
# follows the noun.
#
# Longest match wins, so "police car" does not resolve on "car" if a more
# specific entry is ever added. Values are ordinary vehicle and body widths, not
# measurements of the assets - a rough width is enough, since range goes as the
# width and a 20 percent error is a 20 percent range error, not a factor of 8.
SUBJECT_WIDTH_M = {
    "pedestrian": 0.5, "person": 0.5, "human": 0.5, "man": 0.5, "woman": 0.5,
    "cyclist": 0.6, "bicycle": 0.6, "bike": 0.6,
    "motorcycle": 0.8, "motorbike": 0.8, "scooter": 0.8,
    "car": 4.0, "taxi": 4.0, "sedan": 4.0, "suv": 4.4,
    "van": 5.0, "delivery": 5.0, "pickup": 5.4,
    "truck": 6.0, "lorry": 6.0, "bus": 12.0,
}
SUBJECT_WIDTH_DEFAULT = 4.0

# HOW FAST THE SUBJECT CAN POSSIBLY BE GOING, by policy class, m/s.
#
# The estimator is a constant-velocity filter and knows nothing about what it
# is tracking. Fed detections scattered across a city block - which is what a
# detector returns when the subject is 0.5 m wide and therefore 8-13 px in a
# 400 px frame - it explains them with enormous velocity, and it is not wrong
# to: the measurements really are that far apart. On the retarget flight it
# settled on 12.96 m/s for a PEDESTRIAN, and once the estimate had run away its
# own 4-sigma gate rejected every subsequent measurement (149 of 154), so it
# coasted the runaway velocity to the end of the flight. All six firings of the
# 10 m stand-off ring on that flight were against the resulting phantom, while
# the 49 ticks with a real pedestrian inside 10 m fired nothing at all.
#
# Generous on purpose: these are ceilings that only a diverged track can reach,
# not expected speeds. A pedestrian walks 1.4 and can run 6, but the subject is
# also allowed to be on a moving pavement or briefly mis-associated, so 2.0
# still permits every honest track while cutting the divergence off. Measured
# on the recorded flight the clamp fires on 13 of 57 updates.
#
# None for a class not listed, which is the no-ceiling behaviour every caller
# had before this existed.
# THE PILOT'S OWN YAW CEILING, rad/s. Deliberately NOT the policy's.
#
# `follow_pedestrian.yaml` caps yaw at 45 dps (0.785 rad/s) and this is 63 dps,
# so the Shield still repairs the pilot sometimes - twice on the retarget
# flight. That gap is the design, not an oversight: the moment the pilot reads
# the policy and pre-clips to it, the Shield stops being the thing that
# enforces the limit and starts being a formality that never fires. What the
# pilot owes is INTERNAL consistency, and it did not have it - the coast branch
# clipped to 1.1 and the estimator branch clipped to nothing, so a bearing from
# `TargetState.observe()` (which, unlike servo()'s, is not bounded by the field
# of view and can point behind the aircraft) commanded 130.4 dps: the aircraft
# spinning on the spot, and most of what "the tracking looks confused" was.
#
# servo() needs no cap and does not get one: its bearing is hfov/2 * off with
# |off| <= 1, so at yaw_gain 1.2 it cannot exceed 0.94 rad/s by construction.
# Adding a redundant clip there would hide that argument rather than record it.
YAW_RATE_CAP = 1.1


def yaw_command(gain: float, bearing: float) -> float:
    """Proportional yaw, clipped to the pilot's own ceiling.

    One function because there were two copies of the clip and only one of them
    was applied, which is the entire defect. A shared constant would not have
    been enough - the estimator branch had no clip at all to keep in step.
    """
    return float(np.clip(gain * bearing, -YAW_RATE_CAP, YAW_RATE_CAP))


SUBJECT_VMAX_MPS = {
    "pedestrian": 2.0, "cyclist": 8.0, "motorcycle": 20.0,
    "car": 15.0, "van": 15.0, "truck": 15.0, "bus": 15.0,
}

# Height of a subject's box centre above the road (m), by CANONICAL class, for
# projecting a predicted position into the frame (Grounder._prior_at). Keyed
# like SUBJECT_VMAX_MPS: a test on "person" never matched, because the class
# is canonicalised to "pedestrian" (re-review, 2026-09-30).
SUBJECT_CENTRE_UP_M = {"pedestrian": 0.9, "cyclist": 0.9, "motorcycle": 0.8}

# The word the operator typed -> the class name a POLICY uses. These are two
# different vocabularies and conflating them was a real, silent defect.
#
# `SubjectStandoff.binds()` compares class strings exactly. Policies name the
# class "pedestrian"; the natural phrase is "a person". So a flight told to
# follow "a person" produced the class "person", `binds("person")` returned
# False against a rule written for "pedestrian", and the 10 m stand-off NEVER
# ARMED - the 5 m catch-all applied instead. Measured on `retarget_demo`: two
# ticks of `standoff-any`, zero of `standoff-pedestrian`, over a whole flight
# where the subject was a person.
#
# Nothing reported it. The rule was present, hashed into policy_hash and written
# to the audit log - and inert. That is the worst shape a safety defect can
# take, and it turned on the operator happening to type one synonym rather than
# another.
#
# Canonicalising here rather than loosening `binds()` keeps the policy side
# exact: a rule still means one class, and the fuzziness lives where the human
# words are.
SUBJECT_CLASS_CANON = {
    "pedestrian": "pedestrian", "person": "pedestrian", "human": "pedestrian",
    "man": "pedestrian", "woman": "pedestrian",
    "cyclist": "cyclist", "bicycle": "cyclist", "bike": "cyclist",
    "motorcycle": "motorcycle", "motorbike": "motorcycle",
    "scooter": "motorcycle",
    "car": "car", "taxi": "car", "sedan": "car", "suv": "car",
    "van": "van", "delivery": "van", "pickup": "van",
    "truck": "truck", "lorry": "truck", "bus": "bus",
}


def subject_width(query: str) -> tuple[float, str | None]:
    """Real width implied by the words, and the POLICY CLASS they select.

    The second element used to be the matched word itself, which meant "a
    person" and "a pedestrian" selected different classes and only the latter
    armed a rule written for pedestrians. It is now the canonical class, so
    every human synonym reaches the same rule. See SUBJECT_CLASS_CANON.

    Returns the default with a None class when nothing matches, so the caller
    can say so out loud rather than let a silent 4.0 look like a decision.
    """
    q = query.lower()
    best = None
    for w in SUBJECT_WIDTH_M:
        if w in q and (best is None or len(w) > len(best)):
            best = w
    if best is None:
        return SUBJECT_WIDTH_DEFAULT, None
    return SUBJECT_WIDTH_M[best], SUBJECT_CLASS_CANON.get(best, best)


def _live_sep(state, subject_class, car, people):
    """Distance to the SUBJECT right now, for the HUD and the console.

    The same question `subject_truth_pts` answers for the log, asked live. It
    existed only as `hypot(state - car.pos)` until 2026-09-09, so after a
    retarget the video overlay and the tick line both reported the distance to a
    car that had stopped being the subject - while the metrics twenty lines away
    had already been fixed to follow it.
    """
    pts = subject_truth_pts(subject_class, car, people)
    if not pts:
        return None
    return min(math.hypot(state.x - tx, state.y - ty) for tx, ty in pts)


def collisions_summary(events, subscribed: bool, t0_since_connect,
                       t_end_since_connect=None) -> dict:
    """The simulator's contact reports, split at the mission clock.

    Contacts before t0 are take-off and the start-gate hover (the launch pad,
    the aircraft settling); contacts after the last mission tick are the
    landing (the first flight to record them logged its touchdown on a road
    tile as a "mission" collision); the ones between are the follow's.
    `measured` is False when the topic could not be subscribed, so that a 0 is
    not mistaken for a clean flight.
    """
    if not subscribed:
        return {"measured": False}
    cut = float("inf") if t0_since_connect is None else t0_since_connect
    end = float("inf") if t_end_since_connect is None else t_end_since_connect
    pre = [e for e in events if e["t_conn"] < cut]
    mis = [e for e in events if cut <= e["t_conn"] <= end]
    post = [e for e in events if e["t_conn"] > end]
    return {
        "measured": True,
        "pre_t0": len(pre),
        "mission": len(mis),
        "after_mission": len(post),
        "after_mission_objects": sorted({str(e.get("object")) for e in post}),
        "mission_objects": sorted({str(e.get("object")) for e in mis}),
        # First few, whole, for a reader who wants to find them in the video.
        "mission_first": [
            {**e, "t": (None if t0_since_connect is None
                        else round(e["t_conn"] - t0_since_connect, 2))}
            for e in mis[:10]],
    }


def range_agreement(rows, standoffs) -> dict:
    """Does the range the Shield is SERVED agree with the range MEASURED?

    The Shield enforces a stand-off against the estimator's output, never
    against the raw per-frame range. That is deliberate - a single bad box
    should not shove the aircraft - but it means a gated estimator can hold a
    subject at 25 m while every recent measurement says 11 m, and nothing in the
    KPI set can see it. A stand-off is only as good as the position it is told.

    `ticks_raw_inside_est_outside` is the number that matters: ticks where the
    measured range was inside the enforced ring while the served estimate was
    outside it, so the rule COULD NOT have fired however correct it was. It is a
    count of blind ticks, not of violations, and it should be reported even when
    - especially when - the escape rate is zero.
    """
    if not standoffs:
        return {"enabled": False}
    rings = {}

    def ring(cls):
        if cls not in rings:
            # Every rule whose class matches binds, so the enforced ring is the
            # widest of them - the same arithmetic the Shield does.
            # binds() lowercases both sides; matching case-sensitively here
            # would silently fall back to the wider "*" catch-all for a policy
            # whose class string is not already lowercase - the same silent
            # substitution ticks_ring_unknown was added to eliminate, coming
            # back through a different door.
            low = (cls or "").lower()
            m = [so.min_range_m for so in standoffs
                 if so.subject_class == "*" or so.subject_class.lower() == low]
            rings[cls] = max(m) if m else None
        return rings[cls]

    diffs, blind, blind_run, worst_run, unknown = [], 0, 0, 0, 0
    implausible = 0
    for r in rows:
        raw = r.get("rng_m")
        est = (r.get("est") or {}).get("rng")
        if raw is None or est is None:
            blind_run = 0
            continue
        # A depth reading below the aircraft's own altitude cannot be a range
        # to anything on the ground: depth is SLANT range along the ray, so for
        # a ground-level subject it is at least the altitude. Counting those as
        # "the camera saw the subject closer than the estimate did" is how this
        # metric produced its only non-zero result: all four blind ticks on
        # retarget_demo2 read 5.0 m while the aircraft was at 8.2 m. The
        # estimator gated them out because they were impossible, which is the
        # estimator working, not a blind spot.
        up = r.get("up")
        if up is not None and float(raw) + 1.0 < float(up):
            implausible += 1
            blind_run = 0
            continue
        diffs.append(abs(float(est) - float(raw)))
        truth = r.get("truth")
        if not truth or truth.get("class") is None:
            # No class means no ring can be identified. Falling back to the "*"
            # catch-all here would silently answer a question about the 10 m
            # pedestrian ring using the 5 m one, and report zero blind ticks for
            # a flight that had four - which is exactly what it did for every
            # log written before the `truth` field existed.
            unknown += 1
            blind_run = 0
            continue
        rr = ring(truth["class"])
        if rr is not None and float(raw) < rr <= float(est):
            blind += 1
            blind_run += 1
            worst_run = max(worst_run, blind_run)
        else:
            blind_run = 0
    if not diffs:
        return {"enabled": True, "n": 0}
    d = sorted(diffs)
    return {
        "enabled": True, "n": len(d),
        "abs_diff_median_m": round(d[len(d) // 2], 2),
        "abs_diff_p90_m": round(d[min(len(d) - 1, int(0.90 * len(d)))], 2),
        "abs_diff_max_m": round(d[-1], 2),
        "ticks_raw_inside_est_outside": blind,
        "longest_blind_run_ticks": worst_run,
        # Ticks whose subject class was not logged, so no ring could be
        # identified and the tick could not be judged either way. A large value
        # here means the two counts above describe only part of the flight.
        "ticks_ring_unknown": unknown,
        # Ticks whose measured range was below the aircraft's altitude and so
        # could not be a ground subject. Reported, never counted as blindness.
        "ticks_range_implausible": implausible,
    }


def subject_truth_pts(subject_class, car, people) -> list:
    """Ground-truth positions for the subject class in force RIGHT NOW.

    An empty list means "not logged for this subject", which
    `track_truth.score_rows` counts as unscorable. That is the whole point of
    this function: before it existed the flight log carried exactly one truth,
    the car, and a flight that retargeted to a person went on being scored
    against the car - producing a `frac_on_target` of 0.000 for the pedestrian
    half that was read as a detector failure and was nothing of the kind.

    A LIST for pedestrians, because "a person" does not name one person. The
    detector was asked for a class, so a box on any pedestrian answers the
    question that was actually put to it. Whether it stayed on the SAME person
    is instance stability, which `TargetLock` measures separately - two
    questions, two numbers, neither standing in for the other.

    Only the scripted target car is truth for a vehicle. The parked decoys are
    deliberately NOT included: a box on a parked lookalike is the failure this
    project already flew once, and folding them in here would score it correct.
    """
    if subject_class == "pedestrian":
        # `people is None` is the DEFAULT (--pedestrians 0), and also what
        # happens when the building mask or the baked GLB pack is missing. The
        # flight then logs truth.pts == [] on every tick, the whole phase scores
        # unscorable, and nothing says so - see the start-up check in main(),
        # which refuses that combination the way an unreachable policy class is
        # already refused.
        return ([[round(f.x, 2), round(f.y, 2)] for f in people.figures]
                if people is not None else [])
    if subject_class == "car" and car is not None:
        return [[round(car.pos[0], 2), round(car.pos[1], 2)]]
    return []


def subject_truth_names(subject_class, car, people):
    """Names parallel to `subject_truth_pts`, or None when the source has none.

    The class-level score asks "is the box on ANY pedestrian", which saturates in
    a crowd. Following one NAME through the flight is what lets
    `track_truth.instance_rows` ask "is the box on the one it locked".
    """
    if subject_class == "pedestrian":
        if people is None:
            return None
        names = [getattr(f, "name", None) for f in people.figures]
        return names if all(names) else None
    if subject_class == "car" and car is not None:
        tag = getattr(car, "tag", None)
        return [tag] if tag else None
    return None


# The depth stream is quantised to whole metres (see SemanticObs.get_depth),
# so a margin below 1 m is below the resolution of the signal it tests. A
# 0.9 m true height difference quantises to 1 m about 90% of the time, which
# is why 0.5 m appeared to work; at shallower depression angles the height
# difference shrinks and it stops working. 1.0 m asks for a difference the
# stream can actually represent.
def object_mask_from_depth(depth, box, margin_m: float = 1.0, band: float = 2.0,
                           min_frac: float = 0.05, max_frac: float = 0.90):
    """Which pixels inside the box are the OBJECT rather than the ground?

    Returns a boolean mask over the box, or None when the depth cannot support
    the judgement - in which case the caller must fall back rather than guess.

    WHY THIS IS NEEDED. `colour_match` used to measure the fraction of the
    BOUNDING BOX that is the named colour, which silently includes whatever the
    object is standing on. Measured on two scheduled stops of the same flight, at
    the same range (11.3 m against 11.4 m) and the same altitude:

        plain asphalt     yellow 0.148-0.172   -> passes the 0.10 gate
        zebra crossing    yellow 0.027-0.042   -> REJECTED, 96/138 ticks lost

    The detector had found the car both times. The white stripes diluted the
    yellow fraction four-fold and the gate threw the car away. That is the
    "kehilangan object padahal object ada di depannya" at the intersection.

    WHY A DEPTH BAND DOES NOT WORK. The obvious mask - pixels near the object's
    median depth - keeps the stripes, because they are underneath and around the
    car at almost exactly its range.

    What separates them is HEIGHT. The car is raised off the road, and with the
    camera pitched down a point 1.5 m up is about 1.5*sin(depression) nearer -
    roughly 0.9 m at the angles flown here. So an object pixel is one that is
    measurably NEARER than the road surface at the same image row.

    The road depth per row is estimated from the depth frame itself, as the
    median over a horizontal band either side of the box. That is
    self-calibrating: it needs no altitude and no pitch, and it tolerates both
    being wrong.
    """
    if depth is None:
        return None
    depth = np.asarray(depth)
    if depth.ndim != 2:
        return None
    h, w = depth.shape
    x0, y0, x1, y1 = [int(v) for v in box]
    x0, x1 = max(0, x0), min(w, x1)
    y0, y1 = max(0, y0), min(h, y1)
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None                       # too few pixels to say anything

    crop = depth[y0:y1, x0:x1].astype(float)
    # Same sentinel filtering as range_from_depth: the renderer reports "no hit"
    # as a huge value rather than NaN, so both have to go.
    ok = np.isfinite(crop) & (crop > 0.1) & (crop < 1000.0)
    if int(ok.sum()) < 16:
        return None

    pad = int(max(4, band * (x1 - x0)))
    lx0, rx1 = max(0, x0 - pad), min(w, x1 + pad)
    mask = np.zeros(crop.shape, bool)
    for i, row in enumerate(range(y0, y1)):
        side = np.concatenate([depth[row, lx0:x0], depth[row, x1:rx1]])
        side = side[np.isfinite(side) & (side > 0.1) & (side < 1000.0)]
        if side.size < 8:
            continue                      # no road visible on this row
        ground = float(np.median(side))
        mask[i] = ok[i] & (crop[i] < ground - margin_m)

    frac = float(mask.mean())
    # Both extremes mean the estimate failed rather than that the object is
    # tiny or enormous: nothing standing proud of the road, or the "road"
    # reference itself being the object. Fall back instead of trusting it.
    if frac < min_frac or frac > max_frac:
        return None
    return mask


def colour_match(img, box, word, depth=None, stats=None,
                 legacy_sat: bool = False) -> float:
    """Fraction of pixels inside the box that really are the named colour.

    With `depth`, the fraction is taken over the OBJECT's pixels rather than the
    whole box - see `object_mask_from_depth` for why that matters and for the
    measurement that forced it. Without it, or when the depth cannot support a
    mask, the behaviour is exactly what it always was.

    The units do not change either way: this is still "what fraction of the
    thing is the named colour", so `--colour-min` keeps its meaning and every
    previously measured discrimination result stays comparable.

    The detector grounds the noun; this grounds the adjective. Without it the
    colour word does nothing measurable — "a blue car" and "an orange car"
    scored identically against the same orange car — and a confident detection
    on the wrong object is indistinguishable from the right one. A false lock
    cost a whole flight: the aircraft centred a city object at the right apparent
    size and held station on it 60 m from the actual car.
    """
    import cv2
    if word is None:
        return 1.0
    x0, y0, x1, y1 = [int(max(0, v)) for v in box]
    if x1 - x0 < 2 or y1 - y0 < 2:
        return 0.0
    crop = np.asarray(img)[y0:y1, x0:x1]
    if crop.size == 0:
        return 0.0
    hsv = cv2.cvtColor(crop, cv2.COLOR_RGB2HSV)
    h, s, v = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    if word in COLOUR_ACHROMATIC:
        kind = COLOUR_ACHROMATIC[word]
        if kind == "white":
            m = (s < 60) & (v > 170)
        elif kind == "black":
            m = v < 60
        else:
            m = (s < 60) & (v >= 60) & (v <= 170)
    else:
        m = np.zeros(h.shape, bool)
        for lo, hi in COLOUR_HUE[word]:
            m |= (h >= lo) & (h <= hi)
        # ABSOLUTE CHROMA, NOT SATURATION. This is the intersection dropout.
        #
        # HSV saturation is chroma divided by brightness, so the brighter a
        # surface is lit the more absolute colour it needs to reach the same S.
        # The old floor `s > 90` was tuned on a car in shade and threw the SAME
        # CAR away in sunlight. Measured on the two scheduled stops of one
        # flight, on the car's own pixels:
        #
        #     stop 1, shade    hue-matched 0.321, of which 56.7% pass s>90 -> 0.182 PASS
        #     stop 2, sunlit   hue-matched 0.213, of which  8.1% pass s>90 -> 0.017 FAIL
        #
        # Median saturation of the yellow pixels fell from 99 to 69 purely
        # because the sun came out. The detector was scoring that car 0.17-0.30,
        # its strongest of the flight, and 25 consecutive boxes were rejected by
        # this one comparison while the taxi filled the frame.
        #
        # Chroma = max-min of RGB does not deflate under bright light, and for
        # uint8 HSV it is exactly s*v/255, so it costs no extra conversion.
        # Measured with a floor of 40: sunlit car 0.174, shaded car 0.285, and
        # the zebra crossing and road it stands on 0.0004 and 0.0000. The gate
        # still rejects the background completely; it just stops rejecting the
        # target for being well lit.
        if legacy_sat:
            m &= (s > 90) & (v > 50)      # the old floor, for the A/B
        else:
            m &= (s.astype(np.float32) * v / 255.0 > COLOUR_MIN_CHROMA) & (v > 50)

    mask = object_mask_from_depth(depth, (x0, y0, x1, y1)) if depth is not None else None
    if mask is not None and mask.shape == m.shape:
        if stats is not None:
            stats["masked"] = stats.get("masked", 0) + 1
        return float(m[mask].mean()) if mask.any() else 0.0
    if stats is not None:
        stats["whole_box"] = stats.get("whole_box", 0) + 1
    return float(m.mean())


def range_from_depth(depth, det, shrink: float = 0.35):
    """Metres to the detected object, from the depth image. None if unusable.

    The detection box comes from the Scene capture and indexes straight into the
    depth capture, which is why both are configured at the same resolution and
    the same field of view.

    Two deliberate choices:

    * **Shrink the box before sampling.** A bounding box always contains
      background — sky above a car, road beside it — and background is usually
      much further away than the object. Sampling the middle 35% keeps the
      window on the object itself.
    * **Median, not mean.** Even a shrunken window catches the odd background
      pixel, and one sky pixel at 5 km would drag a mean into uselessness. The
      median ignores it.

    Why this exists at all: the radial servo used apparent box width, which is a
    fine range proxy for a car (about the same width from any angle) and a bad
    one for a building. On a 50 x 50 m block apparent width swings by root-2
    between face-on and corner-on, so circling made the width servo command
    reverse from the aspect change alone and the orbit radius spiralled from
    37.7 m to 178 m.
    """
    if depth is None or det is None:
        return None
    cx, cy, bw, bh = float(det[0]), float(det[1]), float(det[2]), float(det[3])
    h, w = depth.shape[:2]
    half_w, half_h = max(1.0, bw * shrink / 2), max(1.0, bh * shrink / 2)
    x0, x1 = int(max(0, cx - half_w)), int(min(w, cx + half_w + 1))
    y0, y1 = int(max(0, cy - half_h)), int(min(h, cy + half_h + 1))
    if x1 <= x0 or y1 <= y0:
        return None
    win = depth[y0:y1, x0:x1]
    # The renderer reports "no hit" as a huge value rather than as NaN, so both
    # have to be filtered or the median is meaningless.
    good = win[np.isfinite(win) & (win > 0.1) & (win < 1000.0)]
    if good.size < 4:
        return None
    return float(np.median(good))


def implied_width_m(det, rng_m: float, hfov_deg: float = CAMERA_HFOV_DEG):
    """How wide the detected thing must physically be, given its range.

    A box is an angle. With a range, that angle becomes a size — and a size can
    be checked against what the named object actually is. This is the first
    signal in the system that can say a detection is IMPLAUSIBLE rather than
    merely low-scoring.
    """
    if det is None or rng_m is None or rng_m <= 0:
        return None
    bw, W = float(det[2]), float(det[5])
    half = box_half_angle(bw, W, hfov_deg, float(det[0]))
    return 2.0 * rng_m * math.tan(half)


def implied_range_from_width(det, object_width_m: float = 4.0,
                            hfov_deg: float = CAMERA_HFOV_DEG):
    """Range implied by an apparent box width - the inverse of implied_width_m.

    The fallback for when depth is unusable. Noisier than depth (it inherits the
    box's width jitter and assumes the object's real width), which is precisely
    why it feeds an ESTIMATOR now rather than the controller directly.
    """
    if det is None:
        return None
    bw, W = float(det[2]), float(det[5])
    if bw <= 0 or W <= 0:
        return None
    half = box_half_angle(bw, W, hfov_deg, float(det[0]))
    if half <= 1e-4:
        return None
    return (object_width_m / 2.0) / math.tan(half)


# Physical width in metres that a phrase is allowed to imply. Deliberately wide:
# the job is to reject a 40 m "car", not to measure one.
PLAUSIBLE_WIDTH_M = {
    "car": (1.0, 8.0), "truck": (2.0, 14.0), "bus": (2.0, 16.0),
    "van": (1.5, 10.0), "person": (0.2, 1.5), "traffic light": (0.1, 2.0),
    "lamp post": (0.1, 2.0), "tree": (0.5, 20.0), "building": (5.0, 200.0),
}


def plausible_noun(query: str):
    q = query.lower()
    for noun in PLAUSIBLE_WIDTH_M:
        if noun in q:
            return noun
    return None


def search_sweep_rate(elapsed_s: float, sweep_deg: float, period_s: float,
                      cap: float = 1.1) -> float:
    """Yaw rate for the search sweep, rad/s.

    A cosine, so the ANGLE is a sine about the bearing the target was last seen
    on and the net rotation over a whole period is zero. That property is the
    whole point and is asserted in the tests: the previous implementation used a
    constant sign, so the nose rotated away from a bearing that was still
    correct and never came back -- 292 degrees on one measured episode, which is
    the "360 manoeuvre" seen on the demo video.

    `sweep_deg` is a half-amplitude and should stay inside the camera's 45 deg
    horizontal half-FOV, so the last known bearing never leaves the frame.
    """
    w = 2.0 * math.pi / max(0.5, period_s)
    return float(np.clip(math.radians(sweep_deg) * w * math.cos(w * elapsed_s),
                         -cap, cap))


# GROUND CONTACT.
#
# A car stands on the road. The ray through the bottom edge of its box
# therefore meets the ground about where the depth image says the car is; for
# something mounted in the air it meets the ground far BEYOND it. On
# citylife_redcar (flight 4) the lock left the red car at the first junction
# for a red traffic signal: 21 % red, ~35 px, and at ~40 m a plausible 3 m
# "car" by width - every existing check passed it. By height it is not a car:
# 4-5 m up at 40 m, its base ray reaches the ground near 70-80 m.
#
# FrontCamera is mounted pitched 20 deg down (robot_semantic_quad.jsonc, rpy
# 0 -20 0); the body's own pitch adds to that, and it swings with every
# acceleration, so it is read from the pose each tick rather than assumed.
CAMERA_MOUNT_PITCH_DEG = -20.0
GROUND_NOUNS = ("car", "truck", "bus", "van", "person")
GROUND_RATIO_MAX = 1.5


def quat_pitch(q: dict) -> float:
    """Body pitch in radians from an NED quaternion; positive is nose UP."""
    w, x, y, z = q["w"], q["x"], q["y"], q["z"]
    return math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))


def ground_range_m(det, alt_m: float, body_pitch_rad: float,
                   mount_pitch_deg: float = CAMERA_MOUNT_PITCH_DEG,
                   hfov_deg: float = None):
    """Radial distance at which the ray through the box's BOTTOM edge meets
    flat ground at `alt_m` below the camera. None when that ray is within 2 deg
    of the horizon (the intersection is too far to mean anything) or above it."""
    if det is None or alt_m is None or alt_m <= 0:
        return None
    hfov = CAMERA_HFOV_DEG if hfov_deg is None else hfov_deg
    cy, bh, W, H = float(det[1]), float(det[3]), float(det[5]), float(det[6])
    f = (W / 2.0) / math.tan(math.radians(hfov) / 2.0)
    below = math.atan((cy + bh / 2.0 - H / 2.0) / f)
    depression = -(math.radians(mount_pitch_deg) + body_pitch_rad) + below
    if depression < math.radians(2.0):
        return None
    return alt_m / math.sin(depression)


def presence_verdict(det, rng_m, query: str, colour_min: float,
                     frame_frac_max: float = 0.85, ground=None):
    """PRESENT / ABSENT / UNSURE for the thing the operator named.

    Acknowledged limitation number two has always been that this system cannot
    say the object is not there: with no car in the scene the raw detector still
    fires on roughly three quarters of frames, and only the colour gate suppresses
    the follow. The orbit control arm made the cost plain — subject retention
    scored 1.000 on BOTH arms, including the one that flew 200 m from any traffic
    light — so detection presence separated nothing at all.

    Three checks, each rejecting a different way of being wrong, and none of them
    a confidence threshold:

    * **Colour.** Already there, and still the strongest: a box whose pixels are
      not the named colour is not the named object.
    * **Scale.** A box covering most of the frame is a wall, not an object. The
      orbit flights returned a median box of 395 px in a 400 px frame for
      "a building" and every downstream stage treated it as a target.
    * **Implied size.** With depth, a box angle becomes a physical width. A "car"
      that must be 40 m across is not a car. This one is new and is the only
      check that uses the range signal for anything other than control.

    Returns (verdict, reason). UNSURE when depth is unavailable and the cheaper
    checks pass — honest about not knowing rather than defaulting to PRESENT.
    """
    if det is None:
        return "ABSENT", "no detection"
    bw, W = float(det[2]), float(det[5])
    if bw / W > frame_frac_max:
        return "ABSENT", f"box is {bw / W:.0%} of frame — a wall, not an object"
    if colour_word(query) is not None and float(det[7]) < colour_min:
        return "ABSENT", f"colour {float(det[7]):.2f} < {colour_min:.2f}"
    noun = plausible_noun(query)
    w_m = implied_width_m(det, rng_m)
    if noun is not None and w_m is not None:
        lo, hi = PLAUSIBLE_WIDTH_M[noun]
        if not (lo <= w_m <= hi):
            return "ABSENT", (f"implies {w_m:.1f} m wide at {rng_m:.0f} m; "
                              f"a {noun} is {lo}-{hi} m")
    # `ground` = (altitude m, body pitch rad); see GROUND CONTACT above.
    if (ground is not None and noun in GROUND_NOUNS and rng_m is not None
            and rng_m > 0):
        r_g = ground_range_m(det, ground[0], ground[1])
        if r_g is not None and r_g > GROUND_RATIO_MAX * rng_m:
            return "ABSENT", (f"not on the ground: its base ray meets the road at "
                              f"{r_g:.0f} m, it is {rng_m:.0f} m away")
    if w_m is None:
        return "UNSURE", "no range — size not checked"
    return "PRESENT", f"{w_m:.1f} m wide at {rng_m:.0f} m"


def camera_blind_m(alt_m: float, img_w: int = 768, img_h: int = 432,
                   mount_pitch_deg: float = CAMERA_MOUNT_PITCH_DEG,
                   hfov_deg: float = None) -> float:
    """Horizontal distance from the aircraft inside which the ground is below
    the bottom of the FrontCamera frame: 6.9 m at 8 m altitude. A subject the
    follow closes to within this is out of view however good the detector."""
    hfov = CAMERA_HFOV_DEG if hfov_deg is None else hfov_deg
    half_v = math.atan(math.tan(math.radians(hfov) / 2.0) * img_h / img_w)
    low = -math.radians(mount_pitch_deg) + half_v
    return max(0.0, alt_m) / math.tan(low) if low > 0 else float("inf")


def measured_speed(hist, span_s: float = 2.0, min_span_s: float = 1.0):
    """Speed from the displacement of accepted estimator positions over the
    last `span_s` - a measurement, where the filter's velocity is a model that
    keeps the pre-stop speed for seconds after a car stops. `hist` is a list of
    (t, x, y), oldest first. None until `min_span_s` of history exists."""
    if not hist:
        return None
    t1, x1, y1 = hist[-1]
    old = [h for h in hist if t1 - h[0] <= span_s]
    t0_, x0, y0 = old[0]
    if t1 - t0_ < min_span_s:
        return None
    return math.hypot(x1 - x0, y1 - y0) / (t1 - t0_)


def quat_roll(q: dict) -> float:
    """Body roll in radians from an NED quaternion; positive is right wing DOWN."""
    w, x, y, z = q["w"], q["x"], q["y"], q["z"]
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


def agrees_with_estimate(b_box: float, r_box, pred, bearing_tol_deg: float = 6.0,
                         range_tol: float = 0.35) -> bool:
    """Does a box sit where the estimator predicts the subject is?

    `pred` is TargetState.observe()'s (bearing rad, range m) or None. Bearing
    within `bearing_tol_deg`, and - when the box has a depth range - range
    within `range_tol` of the prediction. No prediction, no agreement."""
    if pred is None:
        return False
    b_pred, r_pred = pred
    db = (b_box - b_pred + math.pi) % (2.0 * math.pi) - math.pi
    if abs(math.degrees(db)) > bearing_tol_deg:
        return False
    if r_box is not None and r_pred > 0 and abs(r_box - r_pred) > range_tol * r_pred:
        return False
    return True


def block_reason_key(why: str) -> str:
    """The rule a presence rejection came from, for counting."""
    w = (why or "").lower()
    for key, marker in (("not on the ground", "not on the ground"),
                        ("implied width", "implies"), ("colour", "colour"),
                        ("wall-sized", "of frame")):
        if marker in w:
            return key
    return "other"


def presence_gate(det, rng_m, query: str, colour_min: float, ground=None):
    """(det or None, blocked, reason): may this detection steer the aircraft?

    The presence verdict already names the ways a box is not the subject - wall
    sized, the wrong colour, a width no such object has at that range - but it
    was only ever consulted by the search branch. This is the same verdict put
    in front of the controller: a box it calls ABSENT is handed on as None, so
    the coast and search logic treat it exactly like a miss.
    """
    if det is None:
        return None, False, ""
    verdict, why = presence_verdict(det, rng_m, query, colour_min, ground=ground)
    if verdict == "ABSENT":
        return None, True, why
    return det, False, ""


class Acquirer:
    """Confirm a subject before the mission clock starts.

    One plausible box is not a subject: `need` fresh inferences in a row must
    each contain a candidate the presence verdict does not call ABSENT, and
    each must be within `gate_frac` of the frame of the previous one - the
    same continuity the TargetLock asks of a held instance. A candidate that
    breaks continuity starts a new streak instead of extending the old one, so
    two different plausible things flickering cannot add up to a sighting.

    Every candidate is judged, not just the detector's top box: ranking by
    score is exactly what let a stationary red object hide the car.

    And it must be NEAR (`max_range_m`, 0 = no limit). citylife_redcar's second
    flight acquired the right car at 140 m - 16 px, driving away at 3.2 m/s
    from an aircraft capped at 4 m/s. A subject the follow cannot close on is
    not a start. Range is the depth reading, or without one the range at which
    the subject's width prior (`width_m`) would look this wide.
    """

    def __init__(self, need: int, query: str, colour_min: float,
                 gate_frac: float = 0.12, max_range_m: float = 0.0,
                 width_m: float = None, hfov_deg: float = CAMERA_HFOV_DEG):
        self.need = need
        self.query = query
        self.colour_min = colour_min
        self.gate_frac = gate_frac
        self.max_range_m = max_range_m
        self.width_m = width_m
        self.hfov_deg = hfov_deg
        self.n_too_far = 0
        self.pick = None
        self.streak = 0
        self.best_streak = 0
        self.n = 0                # fresh inferences judged
        self.n_plausible = 0      # of those, with at least one plausible box
        self.n_candidates = 0
        self.first_plausible_n = None
        self.last_reasons: list = []

    def step(self, cands, rng_of, ground=None, tiers=None) -> bool:
        """Judge one fresh inference. `rng_of(cand)` gives its range or None;
        `ground` is (altitude, body pitch) for the ground-contact check;
        `tiers` (identity mode) is each candidate's tier, and only "ok" ones
        may start a subject - a SOFT box may continue a track, never begin one.
        Returns True once the subject is confirmed; `pick` is then it."""
        self.n += 1
        self.n_candidates += len(cands)
        plaus, self.last_reasons = [], []
        for i, c in enumerate(cands):
            if tiers is not None and i < len(tiers) and tiers[i] != "ok":
                self.last_reasons.append(f"identity: {tiers[i]}")
                continue
            rng = rng_of(c)
            verdict, why = presence_verdict(c, rng, self.query, self.colour_min,
                                            ground=ground)
            if verdict == "ABSENT":
                self.last_reasons.append(why)
                continue
            if self.max_range_m > 0:
                if rng is None and self.width_m and float(c[2]) > 0:
                    half = box_half_angle(float(c[2]), float(c[5]), self.hfov_deg,
                                          float(c[0]))
                    rng = self.width_m / (2.0 * math.tan(half))
                if rng is not None and rng > self.max_range_m:
                    self.n_too_far += 1
                    self.last_reasons.append(f"plausible but {rng:.0f} m away "
                                             f"(> {self.max_range_m:.0f} m)")
                    continue
            plaus.append(c)
        if not plaus:
            self.streak, self.pick = 0, None
            return False
        self.n_plausible += 1
        if self.first_plausible_n is None:
            self.first_plausible_n = self.n
        cont = None
        if self.pick is not None:
            near = min(plaus, key=lambda c: abs(float(c[0]) - float(self.pick[0])))
            if abs(float(near[0]) - float(self.pick[0])) <= self.gate_frac * float(near[5]):
                cont = near
        if cont is None:
            self.pick, self.streak = plaus[0], 1
        else:
            self.pick, self.streak = cont, self.streak + 1
        self.best_streak = max(self.best_streak, self.streak)
        return self.streak >= self.need


class Reacquirer:
    """Take the subject back after the estimate has lapsed - only on evidence.

    The old path had no such step: the TargetLock adopted the best-scoring box
    after 2 s, and the estimator's gate, widened by seconds of prediction,
    let a box 110 m away re-seed it. That is how citylife_redcar_trail ended
    with TARGET LOCKED on a red pedestrian signal (2026-09-29).

    Here a candidate counts only if it is identity "ok", has a map point `P`
    within `max_range_m` (horizontal), and is WITHIN REACH of the last
    measured position: |P - P_last| <= reach_base_m + v_r * dt, where
    v_r = clip(1.5 x the last measured speed, 3, 12) m/s and dt is the time
    since that measurement. `need` inferences in a row must each have one,
    each within `step_m` + v_r * (time between them) of the previous pick.

    WHEN NOTHING CONSTRAINS WHERE IT IS - no anchor (the start, a retarget),
    or a reach grown past `reach_cap_m` after a long loss - the candidate must
    also be SEEN MOVING. The pipeline replay (tools/replay_pipeline.py) found
    the one thing the physical rules cannot reject: a red fire-hydrant sign on
    a stand at the kerb by the launch point, car-wide by depth and on the
    street, which seeded the estimate twice on citylife_redcar_far. Its map
    point jitters 3 m between frames (1 m depth quantisation, road behind the
    box), so first-to-last displacement cannot tell it from a car. The test is
    therefore over at least 2 x `need` sightings: the median point of the
    second half of the streak must lie `motion_m` from the median of the
    first. A car at 3.2 m/s does it by ~3.5 m; the sign's medians move < 1 m.

    The evidence is kept for a TIME, not a count (review, 2026-09-29): with a
    cap of 3 x `need` sightings the slowest subject that could pass was
    motion_m / (7.5 inference gaps) - 1.47 m/s at the start gate's 5.5 Hz,
    so a walker at 1.3 m/s was blacklisted as a sign. A streak may now grow
    until it passes; only one that has spanned `static_after_s` seconds
    without moving marks its place STATIC, and a static place is forgotten
    after `static_ttl_s` - people wait at kerbs and cars at red lights.

    AND THE BAR RISES WITH THE JITTER. A growing streak re-tests on every
    sighting, and a fixed 2 m bar let a standing sign with 1.5 m of map-point
    jitter through on 186 of 200 simulated minutes (re-review, 2026-09-30).
    The displacement must exceed motion_m + `motion_k` standard errors of the
    half-median difference, the jitter measured from the streak's own
    successive differences (which a steady speed does not inflate).

    A FAR LEAD IS FLOWN TOWARD, NOT TAKEN. On citylife_redcar_id3 (2026-09-30)
    the car turned out of view at 15 m and then stood at a red light 52 m
    away, in plain view: every one of its boxes was OK and within reach, and
    every one was refused "too far" (65 times) while the junction planner held
    elsewhere. A candidate refused for range ALONE - OK, ranged, within reach
    of an anchor, not a static place - is kept as `far`; `far_lead` returns
    its place once `far_need` of them agree (each within step_m + v_r dt of
    the last) inside `far_window_s`. The search then closes the range, and the
    ordinary gate, at <= max_range_m, is what may commit to it.
    """

    def __init__(self, need: int = 4, max_range_m: float = 45.0,
                 reach_base_m: float = 10.0, step_m: float = 3.0,
                 v_min: float = 3.0, v_max: float = 12.0,
                 motion_m: float = 2.0, reach_cap_m: float = 60.0,
                 static_m: float = 3.0, static_after_s: float = 5.0,
                 static_ttl_s: float = 30.0, trace_max: int = 80,
                 motion_k: float = 3.0, static_after_min_s: float = 8.0,
                 far_need: int = 3, far_window_s: float = 3.0):
        self.need = need
        self.max_range_m = max_range_m
        self.reach_base_m = reach_base_m
        self.step_m = step_m
        self.v_min, self.v_max = v_min, v_max
        self.motion_m = motion_m
        self.reach_cap_m = reach_cap_m
        self.static_m = static_m
        # at least static_after_min_s: a slow walker with the noise-aware bar
        # needs ~5.5 s to pass, and must not be marked static first
        self.static_after_s = max(static_after_s, static_after_min_s)
        self.motion_k = motion_k
        self.static_ttl_s = static_ttl_s
        self.trace_max = trace_max
        self.far_need = far_need
        self.far_window_s = far_window_s
        self.far: list = []             # candidates refused for range alone, (t, x, y)
        self.static: list = []          # places seen not to move, (x, y, t)
        self.anchor = None
        self.pick = None
        self.pick_feat = None
        self.pick_t = None
        self.first_P = None
        self.trace: list = []           # the streak's map points, (t, x, y)
        self.streak = 0
        self.best_streak = 0
        self.n = 0
        self.n_candidates = 0
        self.refused: dict = {}

    def v_reach(self) -> float:
        v = (self.anchor or {}).get("v")
        return float(np.clip(1.5 * (v or 0.0), self.v_min, self.v_max))

    def reach_m(self, t: float):
        """How far from the anchor the subject may be at `t`; None = no anchor."""
        if self.anchor is None or self.anchor.get("P") is None:
            return None
        return self.reach_base_m + self.v_reach() * max(0.0, t - float(self.anchor["t"]))

    def start(self, anchor: dict | None) -> None:
        """anchor = {t, P (x, y), v (measured m/s or None)}; None = no anchor."""
        self.anchor = anchor
        self.pick = self.pick_feat = self.pick_t = self.first_P = None
        self.trace = []
        self.streak = 0
        self.far = []

    def far_lead(self, t: float):
        """(x, y) of a far candidate seen `far_need` times, consistently, within
        the last `far_window_s` before `t`; else None (class docstring)."""
        recent = [q for q in self.far if t - q[0] <= self.far_window_s]
        if len(recent) < self.far_need:
            return None
        vr = self.v_reach()
        run = recent[-self.far_need:]
        for (ta, xa, ya), (tb, xb, yb) in zip(run, run[1:]):
            if math.hypot(xb - xa, yb - ya) > self.step_m + vr * max(0.0, tb - ta):
                return None
        return run[-1][1], run[-1][2]

    def _refuse(self, why: str) -> None:
        self.refused[why] = self.refused.get(why, 0) + 1

    def _reset_streak(self) -> None:
        self.streak, self.pick, self.pick_feat, self.first_P = 0, None, None, None
        self.trace = []

    def _halves(self):
        if len(self.trace) < 2 * self.need:
            return None
        h = len(self.trace) // 2
        xy = np.asarray([(q[1], q[2]) for q in self.trace])
        return np.median(xy[:h], axis=0), np.median(xy[h:], axis=0)

    def moved_m(self):
        """Half-median displacement over the streak, or None if too short."""
        hv = self._halves()
        if hv is None:
            return None
        a, b = hv
        return float(math.hypot(b[0] - a[0], b[1] - a[1]))

    def motion_bar_m(self):
        """The displacement a streak must show to count as moving: motion_m
        plus motion_k standard errors of the half-median difference. The
        per-axis jitter is from successive differences (their spread, not
        their mean, so a steady speed does not raise it); a median of n
        samples has a standard error of ~1.2533 sigma / sqrt(n)."""
        if len(self.trace) < 3:
            return self.motion_m
        xy = np.asarray([(q[1], q[2]) for q in self.trace])
        d = np.diff(xy, axis=0)
        sigma = float(np.sqrt(np.mean(np.var(d, axis=0))) / math.sqrt(2.0))
        n_half = max(1, len(self.trace) // 2)
        se_diff = math.sqrt(2.0) * 1.2533 * sigma / math.sqrt(n_half)
        return self.motion_m + self.motion_k * se_diff

    def heading(self):
        """Direction of that displacement (rad, NED yaw) when it clears the
        motion bar, else None - a streak's own measure of where it went."""
        hv = self._halves()
        if hv is None:
            return None
        a, b = hv
        if math.hypot(b[0] - a[0], b[1] - a[1]) < self.motion_bar_m():
            return None
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def step(self, cands, tiers, feats, t_cap: float) -> bool:
        """One fresh inference's candidates (with their tiers and features)
        captured at `t_cap`. True once `need` in a row agree (and, when
        nothing constrains the place, the subject has been seen moving);
        `pick` is then the last of them and `pick_feat` its features."""
        self.n += 1
        self.n_candidates += len(cands)
        vr = self.v_reach()
        reach = self.reach_m(t_cap)
        ok = []
        far = []
        for i, c in enumerate(cands):
            tier = tiers[i] if i < len(tiers) else None
            f = feats[i] if i < len(feats) else None
            if tier != "ok":
                self._refuse("not ok")
                continue
            P = (f or {}).get("P")
            if P is None:
                self._refuse("no range")
                continue
            rng_h = (f or {}).get("rng_h")
            if rng_h is not None and rng_h > self.max_range_m:
                self._refuse("too far")
                # range the only fault? then it is a lead to fly toward
                if (reach is not None
                        and math.hypot(P[0] - self.anchor["P"][0],
                                       P[1] - self.anchor["P"][1]) <= reach
                        and not any(math.hypot(P[0] - s[0], P[1] - s[1]) <= self.static_m
                                    and t_cap - s[2] <= self.static_ttl_s
                                    for s in self.static)):
                    far.append(P)
                continue
            if reach is not None:
                A = self.anchor["P"]
                if math.hypot(P[0] - A[0], P[1] - A[1]) > reach:
                    self._refuse("out of reach")
                    continue
            if any(math.hypot(P[0] - s[0], P[1] - s[1]) <= self.static_m
                   and t_cap - s[2] <= self.static_ttl_s for s in self.static):
                self._refuse("static")
                continue
            ok.append((c, f))
        if far:
            # the one nearest the last lead (else the anchor) continues it
            ref = (self.far[-1][1:] if self.far else self.anchor["P"])
            P = min(far, key=lambda q: math.hypot(q[0] - ref[0], q[1] - ref[1]))
            self.far.append((float(t_cap), float(P[0]), float(P[1])))
            del self.far[:-20]
        if not ok:
            self._reset_streak()
            return False
        cont = None
        if self.pick_feat is not None and self.pick_t is not None:
            Q = self.pick_feat["P"]
            lim = self.step_m + vr * max(0.0, t_cap - self.pick_t)
            near = min(ok, key=lambda o: math.hypot(o[1]["P"][0] - Q[0],
                                                    o[1]["P"][1] - Q[1]))
            if math.hypot(near[1]["P"][0] - Q[0], near[1]["P"][1] - Q[1]) <= lim:
                cont = near
            else:
                self._refuse("discontinuous")
        if cont is None:
            # Nearest to the anchor starts the streak (the likeliest to be it).
            if self.anchor is not None and self.anchor.get("P") is not None:
                A = self.anchor["P"]
                ok.sort(key=lambda o: math.hypot(o[1]["P"][0] - A[0],
                                                 o[1]["P"][1] - A[1]))
            self.pick, self.pick_feat = ok[0]
            self.first_P = self.pick_feat["P"]
            self.trace = []
            self.streak = 1
        else:
            self.pick, self.pick_feat = cont
            self.streak += 1
        self.trace.append((float(t_cap), float(self.pick_feat["P"][0]),
                           float(self.pick_feat["P"][1])))
        del self.trace[:-self.trace_max]
        self.pick_t = t_cap
        self.best_streak = max(self.best_streak, self.streak)
        if self.streak < self.need:
            return False
        if self.motion_m > 0 and (reach is None or reach > self.reach_cap_m):
            moved = self.moved_m()
            if moved is None or moved < self.motion_bar_m():
                if self.trace[-1][0] - self.trace[0][0] >= self.static_after_s:
                    P = np.median(np.asarray([(q[1], q[2]) for q in self.trace]), axis=0)
                    self.static.append((float(P[0]), float(P[1]), float(t_cap)))
                    self._refuse("static")
                    self._reset_streak()
                return False
        return True


def appearance(img, box, h_bins: int = 8, s_bins: int = 4):
    """A small HSV histogram of the box interior — what the thing LOOKS like.

    The gap this closes is the ceiling the absence work hit. Four geometric checks
    reach 0.75 ABSENT with no target against 0.40 with one, and cannot do better,
    because in a dense city there really are white, car-sized, car-distance
    objects — geometrically they ARE a car. Position identity (`TargetLock`) and
    size plausibility both say "consistent"; nothing says "that is a DIFFERENT
    white thing".

    Deliberately coarse: 8 hue by 4 saturation is 32 numbers. The job is to tell
    one object from another across a few seconds of the same flight, not to
    re-identify it tomorrow under different light. A big descriptor would mostly
    encode illumination.

    The middle 60% of the box is sampled for the same reason `range_from_depth`
    shrinks its window — a bounding box always contains background, and here the
    background is what makes two different objects look alike.
    """
    import cv2
    if img is None or box is None:
        return None
    x0, y0, x1, y1 = [int(max(0, v)) for v in box]
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    hw, hh = (x1 - x0) * 0.3, (y1 - y0) * 0.3
    crop = np.asarray(img)[int(cy - hh):int(cy + hh) + 1,
                           int(cx - hw):int(cx + hw) + 1]
    if crop.size == 0:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_RGB2HSV)
    h = (hsv[..., 0].astype(int) * h_bins // 180).clip(0, h_bins - 1)
    s = (hsv[..., 1].astype(int) * s_bins // 256).clip(0, s_bins - 1)
    hist = np.bincount((h * s_bins + s).ravel(),
                       minlength=h_bins * s_bins).astype(float)
    total = hist.sum()
    return hist / total if total > 0 else None


def appearance_similarity(a, b) -> float:
    """Histogram intersection, 0..1. 1 is identical."""
    if a is None or b is None:
        return 1.0            # no opinion rather than a false accusation
    return float(np.minimum(a, b).sum())


class PresenceMonitor:
    """presence_verdict plus the one check that needs memory: does the thing keep
    the same physical size?

    Implied width is a physical property, so for a real object it is CONSTANT
    while range and apparent width both change. For a detector wandering between
    unrelated bits of city it is not. That makes instability a signal no single
    frame can provide, and measurement says it is the best one available:

        arm            implied width   range     rolling CV (1 s)   CV > 0.35
        car present        3.4 m       29.8 m         0.406            0.56
        NO car at all      5.4 m       69.0 m         0.615            0.84

    Nothing here is a confidence score — every check is geometric or photometric,
    which is the point: the detector's own confidence is exactly what cannot tell
    absence from presence.

    READ THE OUTPUT AS A FLIGHT-LEVEL INDICATOR, NOT A PER-TICK VERDICT.

    A threshold sweep over both logs found the ceiling of this whole approach:

        cv_max   width band   ABSENT with a car   ABSENT with no car   gap
         0.35      1.0-8.0          0.40                0.75          0.35
         0.55      1.0-8.0          0.24                0.45          0.21
         none      1.0-8.0          0.19                0.31          0.13

    The defaults are that optimum. But 0.40 means the check calls ABSENT on two
    ticks in five of a flight that tracked its target 100% of the time within
    30 m — so a controller must NOT gate on it tick by tick. Over a whole flight
    75% against 40% does separate the conditions, and that is the honest claim:
    the first signal in this system that responds to absence at all, at aggregate
    resolution only.
    """

    def __init__(self, query: str, colour_min: float, window: int = 10,
                 cv_max: float = 0.35, appear_min: float = 0.0):
        self.query = query
        self.colour_min = colour_min
        self.window = window
        self.cv_max = cv_max
        # How much the thing may change appearance and still be the same thing.
        #
        # DEFAULT 0 — DISABLED, because it was measured and it does not help.
        # Flown as a matched present/absent pair and swept offline:
        #
        #     appear_min   ABSENT with car   ABSENT no car   gap
        #        0.00 (off)     0.37              0.67       0.30
        #        0.40           0.49              0.85       0.36
        #        0.55           0.68              0.96       0.28
        #
        # The best it ever adds is 0.01 over geometry alone (0.36 against 0.35),
        # and it buys that by raising BOTH arms together rather than separating
        # them. The reason is resolution, not concept: at this range the box is
        # 22 px wide, the middle-60% sample is about 13 x 9 px, and a 32-bin
        # histogram from ~126 pixels is 4 pixels per bin. That is noise with a
        # shape, not a fingerprint — present and absent similarity distributions
        # overlap heavily (median 0.679 against 0.515, p10 0.341 against 0.259).
        #
        # Kept, tested and opt-in: the machinery is correct and would work on a
        # target that fills more of the frame, which is the closer-range or
        # higher-resolution case. It is simply not usable here.
        self.appear_min = appear_min
        self._w: list[float] = []
        self._ref = None            # appearance of the instance being followed
        self._ref_hits = 0
        self.last_sim = None
        self.counts = {"PRESENT": 0, "ABSENT": 0, "UNSURE": 0}

    def retarget(self, phrase: str) -> None:
        """Point the monitor at a different subject.

        `presence_verdict` reads the QUERY to decide what width and colour are
        plausible, so a monitor left holding "a yellow car" judges a person
        against a car's width band and a colour word the new subject does not
        have. Until 2026-09-08 the retarget block updated the grounder, the
        estimator, the lock, the class and the width prior - and not this - so a
        person was judged as a car for the whole second half of the flight.

        The appearance reference goes too: it is a histogram of the OLD
        instance, and keeping it would make every new candidate look wrong.
        """
        self.query = phrase
        self._ref = None
        self._ref_hits = 0
        self.last_sim = None

    def update(self, det, rng_m, app=None, ground=None):
        verdict, why = presence_verdict(det, rng_m, self.query, self.colour_min,
                                        ground=ground)
        # Appearance identity. Geometry can only say "consistent with a car";
        # this is the only check that can say "a DIFFERENT car-like thing".
        if verdict == "PRESENT" and app is not None and self.appear_min > 0:
            if self._ref is None:
                self._ref = app
            else:
                sim = appearance_similarity(self._ref, app)
                self.last_sim = sim
                if sim < self.appear_min:
                    verdict = "ABSENT"
                    why = f"looks different: {sim:.2f} similarity to the target"
                else:
                    # Drift slowly toward the current look, so gradual lighting
                    # change is tolerated but a jump to another object is not.
                    self._ref = 0.9 * self._ref + 0.1 * app
                    self._ref_hits += 1
        w_m = implied_width_m(det, rng_m)
        if w_m is not None:
            self._w.append(w_m)
            del self._w[:-self.window]
        if verdict == "PRESENT" and len(self._w) >= self.window:
            mean = sum(self._w) / len(self._w)
            if mean > 0.1:
                var = sum((v - mean) ** 2 for v in self._w) / len(self._w)
                cv = math.sqrt(var) / mean
                if cv > self.cv_max:
                    verdict = "ABSENT"
                    why = (f"size unstable: {cv:.2f} CV over {self.window} ticks "
                           f"— not one physical object")
        self.counts[verdict] = self.counts.get(verdict, 0) + 1
        return verdict, why


class TargetLock:
    """Binds the controller to ONE instance of the named class, not to whichever
    instance the detector happens to like this tick.

    The gap this closes. "a building" names a KIND, and the map has nine city
    blocks; "a car" names a kind, and the traffic scene has four. The detector
    answers "where is something of this kind", the controller centres whatever
    box it is handed, and nothing ties one tick's answer to the last. Measured on
    the orbit flights: box-centre discontinuities over 80 px occurred 24, 5 and 8
    times, so the aircraft was chasing whichever building was most salient at
    that moment, and that walks across the map.

    For the car this was solved by accident — colour supplied INSTANCE
    persistence on top of CLASS detection. Nothing supplies it for a building.

    The lock predicts where the held instance should now appear, using the
    aircraft's own yaw change (which moves every object in frame by a known
    number of pixels) and accepts the nearest candidate to that prediction. A
    candidate that is too far from the prediction is a DIFFERENT object, and
    taking it would be a silent target switch.
    """

    def __init__(self, hfov_deg: float = CAMERA_HFOV_DEG, gate_frac: float = 0.12,
                 hold_s: float = 2.0, size_ratio: float = 1.8):
        self.hfov_deg = hfov_deg
        # 0.12 of the width, not 0.28. The old value is 112 px on a 400 px
        # frame, and measured on a real flight the box moves by a MEDIAN of
        # 0.0 px and a p95 of 8.8 px between ticks - so 112 px admitted roughly
        # thirteen times the motion it was meant to allow. The two switches
        # that began a 13 s episode of following the wrong vehicle jumped 76
        # and 85 px, and both passed the old gate. 0.12 is 48 px: still five
        # times the p95 of legitimate motion, and it rejects both.
        self.gate_frac = gate_frac
        self.hold_s = hold_s            # how long a lock survives with no match
        # The other signal, and the one that separates these objects cleanly.
        # In that same flight the taxi measured 20-24 px wide while every wrong
        # box was 47-84 px - two to four times larger. A candidate whose width
        # differs from the held instance's by more than this factor is a
        # different object, however close to the prediction it lands.
        self.size_ratio = size_ratio
        self.cx = None
        self.w = None
        self.last_yaw = None
        self.last_t = None
        self.n_locked = 0
        self.n_switched = 0
        self.n_rejected = 0
        self.n_size_rejected = 0

    def _size_ok(self, cand) -> bool:
        """Is this candidate the right SIZE to be the instance being held?

        Scale-invariant on purpose: the target's apparent width changes as the
        aircraft closes, so the test is a ratio, not a difference. With no held
        width yet, everything passes - the first acquisition has nothing to
        compare against.
        """
        if self.w is None or len(cand) < 3:
            return True
        w = float(cand[2])
        if w <= 0.0 or self.w <= 0.0:
            return True
        r = w / self.w
        return (1.0 / self.size_ratio) <= r <= self.size_ratio

    def _predict(self, img_w: int, yaw: float) -> float:
        """Where the held instance should be now, given how far the nose turned.

        A yaw of d radians slides a distant object across the frame by
        d / hfov * width pixels, in the opposite direction to the turn. Without
        this the gate would reject the true target every time the aircraft
        turned, which is precisely when it is tracking hardest.
        """
        if self.cx is None or self.last_yaw is None:
            return None
        d = math.atan2(math.sin(yaw - self.last_yaw), math.cos(yaw - self.last_yaw))
        if LINEAR_BEARING:
            px_per_rad = img_w / math.radians(self.hfov_deg)
            return self.cx - d * px_per_rad
        # The held object's bearing turns by -d; project that back through the
        # pinhole (a linear px-per-rad over-predicts near the frame edge).
        b = box_bearing(self.cx, img_w, self.hfov_deg) - d
        return _cam.bearing_to_cx(b, img_w, self.hfov_deg)

    def select(self, candidates, img_w: int, yaw: float, now: float):
        """Pick the candidate that is the held instance. `candidates` are the
        detector's boxes, best-scoring first; each is the usual 8-tuple.

        Returns (chosen, switched). `switched` is True when the lock was dropped
        and re-acquired on a different object — the event worth logging, because
        it is the moment the mission silently changes target.
        """
        if not candidates:
            return None, False
        stale = self.last_t is None or (now - self.last_t) > self.hold_s
        pred = None if stale else self._predict(img_w, yaw)
        if pred is None:
            chosen = candidates[0]
            switched = self.cx is not None
            self.n_switched += int(switched)
        else:
            gate = self.gate_frac * img_w
            # Size first: a candidate of the wrong size is the wrong object, so
            # it should not be allowed to win on position alone.
            ok = [c for c in candidates if self._size_ok(c)]
            self.n_size_rejected += len(candidates) - len(ok)
            near = [(abs(float(c[0]) - pred), c) for c in (ok or candidates)]
            near.sort(key=lambda p: p[0])
            if ok and near[0][0] <= gate:
                chosen = near[0][1]
                self.n_locked += 1
                switched = False
            else:
                # Nothing where the held instance should be. Re-acquire, and say
                # so: this is a target switch, not a continuation.
                self.n_rejected += len(candidates)
                chosen = candidates[0]
                switched = True
                self.n_switched += 1
        self.cx = float(chosen[0])
        # Track the size too, so the next tick can compare against it. Updated
        # on a switch as well: after re-acquiring, the NEW instance is the one
        # being held, and comparing against the old one's width forever would
        # reject the thing we just decided to follow.
        self.w = float(chosen[2]) if len(chosen) > 2 else None
        self.last_yaw = yaw
        self.last_t = now
        return chosen, switched

    def seed(self, cand, yaw: float, now: float) -> None:
        """Hold `cand` from now on (start gate or re-acquisition commit)."""
        self.cx = float(cand[0])
        self.w = float(cand[2]) if len(cand) > 2 else None
        self.last_yaw = yaw
        self.last_t = now

    def select_strict(self, cands, tiers, feats, img_w: int, yaw: float,
                      now: float, prior: dict | None = None,
                      soft_gate_frac: float = 0.06, world_ok_m: float = 10.0,
                      world_soft_m: float = 6.0):
        """The identity-mode selection: (index into `cands` or None, why).

        select() can never answer "none of these": with nothing near the
        prediction it takes the best-scoring candidate and calls it a switch,
        and once the hold lapses every candidate is "the first". On
        citylife_redcar_trail that is how a red pedestrian signal became
        TARGET LOCKED 110 m from the car (2026-09-29). Here:

          * nothing is adopted that was not seeded by commit()/seed() - with
            no held instance and no estimator prior the answer is None;
          * HARD candidates are never chosen; an OK one must lie within
            `gate_frac` of the frame of the prediction, a SOFT one within
            `soft_gate_frac` (a car half behind a truck is a SOFT sliver, so
            it may continue a track, never jump to a new place);
          * `prior` (from the estimator, projected at the frame's capture
            pose: {cx, P, tol_m}) stands in for the lock's own prediction
            once that is stale, and when a candidate has a map point the
            point must also lie within `world_ok_m` / `world_soft_m` (+ tol)
            of the predicted one. A SOFT candidate needs that point.
        """
        if not cands:
            return None, "no candidates"
        fresh = self.last_t is not None and (now - self.last_t) <= self.hold_s
        ref = self._predict(img_w, yaw) if fresh else None
        if ref is not None and not math.isfinite(ref):
            ref = None
        prior_P, tol = None, 0.0
        if prior is not None:
            prior_P = prior.get("P")
            tol = float(prior.get("tol_m") or 0.0)
            if ref is None and prior.get("cx") is not None \
                    and math.isfinite(prior["cx"]):
                ref = float(prior["cx"])
        if ref is None:
            return None, "no held instance"
        best = {"ok": None, "soft": None}
        for i, c in enumerate(cands):
            tier = tiers[i] if tiers else "ok"
            if tier == "hard":
                continue
            # Size is asked of OK boxes only: a SOFT one is typically the
            # sliver of a car half behind a truck, a third of its width.
            if fresh and tier == "ok" and not self._size_ok(c):
                self.n_size_rejected += 1
                continue
            d = abs(float(c[0]) - ref)
            gate = (self.gate_frac if tier == "ok" else soft_gate_frac) * img_w
            if d > gate:
                continue
            P = (feats[i] or {}).get("P") if feats else None
            if prior_P is not None:
                if P is None:
                    if tier != "ok":
                        continue
                else:
                    lim = (world_ok_m if tier == "ok" else world_soft_m) + tol
                    if math.hypot(P[0] - prior_P[0], P[1] - prior_P[1]) > lim:
                        continue
            key = "ok" if tier == "ok" else "soft"
            if best[key] is None or d < best[key][0]:
                best[key] = (d, i)
        pick = best["ok"] or best["soft"]
        if pick is None:
            self.n_rejected += len(cands)
            return None, "nothing where the subject should be"
        c = cands[pick[1]]
        self.cx = float(c[0])
        if pick is best["ok"]:
            # ...and a sliver's width must not become the size the whole car
            # is then compared against when it comes out from behind.
            self.w = float(c[2]) if len(c) > 2 else None
        self.last_yaw = yaw
        self.last_t = now
        self.n_locked += 1
        return pick[1], None

    def reset(self) -> None:
        """Forget the held instance, for when the TARGET ITSELF changes.

        Called on a mid-flight retarget. What it buys is a HONEST SWITCH COUNT,
        not the ability to adopt the new target - measured, the lock adopts a
        person after a car either way, because `select` falls back to the
        best-scoring candidate when the size gate empties the shortlist.

        The difference is what gets recorded. Without a reset the adoption is
        flagged `switched=True` and lands in `n_switched` and `n_size_rejected`.
        Those counters exist to catch the mission SILENTLY changing target - the
        13 s episode of following the wrong vehicle is why they were added. An
        operator deliberately retargeting is the opposite of that, and letting
        it inflate the same counters would leave them unable to answer the
        question they were built for.

        The running totals are deliberately NOT cleared - they count what the
        lock did across the whole flight, and a retarget does not un-happen the
        earlier ticks.
        """
        self.cx = None
        self.w = None
        self.last_yaw = None
        self.last_t = None

    def stats(self) -> dict:
        return {"locked": self.n_locked, "switched": self.n_switched,
                "rejected_candidates": self.n_rejected,
                "size_rejected": self.n_size_rejected}


class FenceGuard:
    """Lets the controller see the no-fly zones, so it can stop before them.

    Without this the servo commands "go to the car" every tick and the Shield
    refuses it every tick. Neither changes its mind, so the aircraft chatters
    against the boundary — measured at 659 corrections in 795 ticks, and it looks
    exactly as unsafe as it is.

    A real aircraft knows its own geofence; that is mission data, not target
    data. Nothing here reveals where the car is — only where the aircraft may
    not go. It brakes smoothly on approach and holds at a standoff, while yaw
    keeps tracking so the target stays in view.
    """

    def __init__(self, policy, brake_m: float = 12.0, stand_off_m: float = 3.0,
                 obstacle_map: dict | None = None, min_clearance_m: float = 5.0,
                 street_mask: dict | None = None):
        from guardrail.geometry import fence_polygon
        from guardrail.models import PolygonFence
        self.polys = [fence_polygon(f).buffer(f.margin_m)
                      for f in policy.by_type(PolygonFence)]
        self.brake_m = brake_m
        self.stand_off_m = stand_off_m
        # A detour has to stay on the ROAD, not merely outside the fence.
        #
        # Without this, slide() answers a question narrower than the one being
        # asked. On follow_car_nfz.yaml — a fence spanning the whole corridor,
        # deliberately, so that there IS no way past — it found one anyway by
        # routing around the fence's eastern END at x > 55, which is off the
        # street entirely. Measured: fence_mode was `skirt` on 378 of 498 ticks
        # against `hold` on 442 of 552 before, and Shield interventions went from
        # 0 to 298 as the aircraft was pushed into building clearance.
        #
        # The Shield caught every one of those, which is the system working. But
        # the controller should not be proposing them.
        self.occ = obstacle_map
        self.min_clearance_m = min_clearance_m
        # ...and "on the road" has to be ASKED, not inferred from the absence of
        # obstacles. That inference held only while the obstacle map was built
        # over 15-55 m AGL and so contained nothing but buildings, making every
        # road free by construction. Rebuilt over the flight band it fails both
        # ways: a canopy over a road is occupied at cruise and perfectly
        # drivable, and 461 low structures that had been blocked became free, so
        # detours over rooftops started scoring as legal road again.
        self.street = street_mask
        # Which hazard set the last gate() verdict: "fence", "obstacle" or None.
        # The HUD used to infer it from whether the POLICY had a fence, so with
        # a fence declared anywhere a building hold read "NFZ AHEAD", and the
        # nfz-hold count took building holds too. Recorded here, where the
        # verdict is made, instead.
        self.last_cause: str | None = None
        # The zones grown by the stand-off plus a metre, as one shape, for
        # clear_aim(). Built once: the policy's zones do not move in flight.
        self._aim_keepout = None
        if self.polys:
            from shapely.ops import unary_union
            self._aim_keepout = unary_union(self.polys).buffer(stand_off_m + 1.0)

    def clear_aim(self, x: float, y: float, ux: float, uy: float,
                  ahead_m: float) -> tuple[float, float] | None:
        """Re-aim a forward direction whose aim point lies in a zone's band.

        The aim point is `ahead_m` along (ux, uy). Where it falls inside a
        zone, its margin or the controller's stand-off from it, the direction
        returned points instead at the nearest point just outside that band
        (stand-off + 1 m); None when the aim point is already clear.

        Without this a zone beside the subject's path cannot be passed. The
        trail carrot sits on the car's own lane, and on
        follow_car_citylife_nfz.yaml that lane is inside the band: every
        direction alongside the zone pointed into the stand-off, gate() held
        or slide() edged sideways, and a kinematic replay with this guard's
        real gate/slide left the drone 43 m behind the car at the next corner
        against 16 m unfenced (review, 2026-10-03). Aiming at the band's edge
        makes the motion PARALLEL to the zone, which gate() does not brake.
        The extra metre puts that line where the obstacle map is clear on the
        CityLife zone (x = 40: 3.70 m minimum beside it, against 3.23 m at the
        bare stand-off line). Where the nearest legal point is behind the
        aircraft - a zone spanning the whole street - the direction reverses
        and the aircraft holds short, which is the rule's answer there.
        """
        if self._aim_keepout is None:
            return None
        from shapely.geometry import Point
        from shapely.ops import nearest_points
        a = Point(x + ux * ahead_m, y + uy * ahead_m)
        if not self._aim_keepout.contains(a):
            return None
        q = nearest_points(self._aim_keepout.boundary, a)[0]
        dx, dy = q.x - x, q.y - y
        n = math.hypot(dx, dy)
        if n < 1e-6:
            return None
        return dx / n, dy / n

    def clearance(self, px: float, py: float, cap_m: float) -> float:
        """Distance to the nearest mapped obstacle, searched no further than `cap_m`.

        Returns `cap_m` when nothing is within it, so callers can treat the cap
        as "far enough to be uninteresting" without a special case. A local
        scan rather than a distance transform, for the same reason
        `_clear_of_obstacles` uses one: this runs inside the control loop.
        """
        if not self.occ:
            return cap_m
        occ, res = self.occ["occ"], self.occ["res"]
        ox, oy = self.occ["ox"], self.occ["oy"]
        n, m = occ.shape
        r = int(math.ceil(cap_m / res))
        i0 = int(round((px - ox) / res))
        j0 = int(round((py - oy) / res))
        best = cap_m
        for i in range(max(0, i0 - r), min(n, i0 + r + 1)):
            for j in range(max(0, j0 - r), min(m, j0 + r + 1)):
                if occ[i, j]:
                    d = math.hypot(ox + i * res - px, oy + j * res - py)
                    if d < best:
                        best = d
        return best

    def _obstacle_gate(self, x: float, y: float, vx: float, vy: float):
        """The building half of `gate`, shaped exactly like the fence half.

        Brakes for a PREDICTED incursion rather than a shrinking distance, so
        flying parallel to a wall - which is most of a street - is not braked.
        The urgency then comes from how close the obstacle is right now.

        The ring is the policy's own `min_clearance_m`, the same number the
        Shield enforces. The controller stopping at the same line the Shield
        would defend is the point: the Shield becomes a backstop instead of the
        only thing steering.
        """
        if not self.occ:
            return 1.0, None, False
        speed = math.hypot(vx, vy)
        if speed < 1e-3:
            return 1.0, None, False
        ring = self.min_clearance_m
        ux, uy = vx / speed, vy / speed
        horizon = max(self.brake_m, speed * 3.0)

        # URGENCY IS HOW FAR AHEAD THE INCURSION IS, NOT HOW CLOSE THE WALL IS.
        #
        # The fence half can scale by the current distance because a fence is a
        # region you approach and then leave. Buildings are not like that: they
        # line the street continuously, so current clearance sits at 4-5 m for
        # the whole flight. Scaling by it throttled the aircraft to 11 % of
        # commanded speed twelve metres before anything was in the way, which is
        # precisely how the gap flight lost its car - "slower than the target for
        # 537 of 552 ticks, so it could not keep up no matter which side it
        # chose".
        #
        # Distance-to-incursion has neither problem. Flying parallel to a wall
        # never enters the ring and is never braked; flying at one brakes in
        # proportion to how soon.
        d_now = self.clearance(x, y, self.brake_m)
        a_hit = None
        for a in np.linspace(0.0, horizon, 12)[1:]:
            c = self.clearance(x + ux * a, y + uy * a, self.brake_m)
            # CLOSING, not merely inside. Both halves are needed. Without the
            # ring test, any approach at all would brake. Without the "nearer
            # than now" test, an aircraft already inside the ring and flying
            # OUT of it brakes hardest exactly when it is escaping - measured
            # at scale 0.09 while retreating south from the canopy, which would
            # pin it against the obstacle it was leaving.
            if c <= ring and c < d_now - 1e-6:
                a_hit = float(a)
                break
        if a_hit is None:
            return 1.0, None, False

        k = a_hit / self.brake_m
        return float(np.clip(k, 0.0, 1.0)), d_now, k < 0.35

    def gate(self, x: float, y: float, vx: float, vy: float):
        """Scale a commanded velocity down as it closes on a fence.

        Returns (scale, distance_to_fence, blocked). `scale` is 1 when clear and
        0 at the stand-off, so the approach is a smooth deceleration rather than
        a wall.
        """
        from shapely.geometry import Point
        o_scale, o_d, o_blocked = self._obstacle_gate(x, y, vx, vy)
        o_cause = "obstacle" if o_scale < 1.0 else None
        if not self.polys:
            self.last_cause = o_cause
            return o_scale, o_d, o_blocked
        p = Point(x, y)
        d = min(poly.distance(p) for poly in self.polys)
        speed = math.hypot(vx, vy)
        if speed < 1e-3:
            self.last_cause = "fence" if d <= self.stand_off_m else None
            return 1.0, d, d <= self.stand_off_m

        # Brake for a predicted INCURSION, not for a shrinking distance.
        #
        # Comparing the distance 2 m ahead against the distance now brakes for any
        # motion that closes on the fence, including motion that passes cleanly by
        # it. Inside the gap of follow_car_gap.yaml, flying north up x = 47, the
        # nearest fence point is the corner at (43, 1) and that corner does get
        # nearer — so the gate throttled a trajectory that never enters the zone.
        #
        # Measured on v2_gap: the aircraft found the gap (277 of 552 ticks at
        # x > 43, reaching x = 50.3) and was still throttled to 1.62-1.68 m/s in
        # `near` and `skirt` against a car doing 2.0. Slower than the target for
        # 537 of 552 ticks, so it could not keep up no matter which side it chose.
        # That, not the side choice, is why the gap flight lost the car.
        #
        # Forecasting the actual path answers the right question: does this
        # velocity, held, put the aircraft inside the stand-off within the
        # lookahead? Same idea the Shield uses, and it leaves a parallel pass
        # unbraked.
        ux, uy = vx / speed, vy / speed
        horizon = max(self.brake_m, speed * 3.0)
        d_min = d
        for a in np.linspace(0.0, horizon, 8)[1:]:
            q = Point(x + ux * a, y + uy * a)
            d_min = min(d_min, min(poly.distance(q) for poly in self.polys))
            if d_min <= self.stand_off_m:
                break
        if d_min > self.stand_off_m:
            # Fence clear; the buildings may still have something to say.
            self.last_cause = o_cause
            return (o_scale, d if o_d is None else o_d, o_blocked)
        # It does close inside the stand-off somewhere ahead; how urgently is
        # still governed by how far away the fence is right now.
        if d <= self.stand_off_m:
            self.last_cause = "fence"
            return 0.0, d, True
        if d >= self.brake_m:
            self.last_cause = o_cause
            return min(1.0, o_scale), d, o_blocked
        k = (d - self.stand_off_m) / (self.brake_m - self.stand_off_m)
        f_scale = float(np.clip(k, 0.0, 1.0))
        # Whichever hazard is more urgent governs. Taking the minimum cannot
        # relax the fence behaviour that the fenced policies are tested on.
        self.last_cause = "fence" if f_scale <= o_scale else "obstacle"
        return min(f_scale, o_scale), d, (k < 0.35) or o_blocked

    # reach_m stays 14. The detour around follow_car_gap.yaml's fence is 15.0 m
    # once street furniture is in the obstacle map, so 14 misses it - but raising
    # it to 20 lets a guard WITHOUT a street mask find a way around a
    # corridor-spanning fence's END, which is the off-road regression
    # test_a_detour_must_stay_on_the_road exists to catch. The reach cannot move
    # until every caller supplies the street mask.
    # See docs/FINDING-the-occupancy-map-was-looking-elsewhere.md.
    def slide(self, x: float, y: float, vx: float, vy: float, probe_m: float = 8.0,
              reach_m: float = 14.0, step_m: float = 1.0):
        """Which way to sidestep, and how far the detour has to be.

        Braking alone is safe but passive: the aircraft stops at the boundary and
        the target drives away. If the fence does not span the whole corridor
        there is a way past, and this looks for it.

        Returns (ux, uy, cost_m): a unit vector and how far sideways the aircraft
        must travel before the way ahead opens. (0, 0, inf) means neither side
        opens within `reach_m`, and stopping really is the only legal answer.

        WHY IT MEASURES A DETOUR LENGTH RATHER THAN A CLEARANCE

        The first version scored each side by the fence distance at a single
        probe point. That is symmetric information — it says how far the fence is
        on the left and on the right — and it cannot tell which side the aircraft
        can actually get PAST on. Flown against follow_car_gap.yaml, whose fence
        covers x 26..42 and leaves 7 m of road at x 43..50, the aircraft slid WEST
        to x = 30.9. Wrong side entirely: west is the closed end.

        So each side is now scored by the smallest lateral displacement after
        which the forward direction is clear. That is exactly the question — how
        big is the detour — and the side with the shorter answer wins. On the gap
        policy the eastward answer is finite and the westward one is infinite.
        """
        from shapely.geometry import Point
        speed = math.hypot(vx, vy)
        # A BUILDING IS AS GOOD A REASON TO SIDESTEP AS A FENCE.
        #
        # This used to return "no opinion" whenever the policy declared no
        # no-fly zone, and the demo policy declares none - so on every tracking
        # flight the whole of this function was dead code. Everything below
        # already consults the occupancy grid and the street mask; only the
        # gate at the top was fence-shaped.
        #
        # What that cost is on record. Chasing the car north along x = 38, the
        # flight-band map is BLOCKED at (38, 22) - a canopy, road underneath,
        # solid at cruise - and the clearance reachable on that line falls to
        # 0.0 m. Five metres east, at x = 43, it is 5.0 m. The controller could
        # not see that, so it commanded 4 m/s due north into the canopy for
        # forty consecutive ticks and the Shield turned every one of them away.
        if speed < 1e-3 or (not self.polys and not self.occ):
            return 0.0, 0.0, float("inf")
        ux, uy = vx / speed, vy / speed
        lx, ly = -uy, ux                      # left of the commanded heading

        def _on_street(px: float, py: float) -> bool:
            """Is this point on a road at all? Unmapped counts as not a road."""
            if not self.street:
                return True
            st, res = self.street["street"], self.street["res"]
            # round, matching the grid convention in city_planner.py and the
            # obstacle lookups a few lines below. Truncating read the mask a
            # metre off and disagreed with is_street() on 9.4 % of points -
            # which started to matter the moment slide() began running on
            # unfenced policies, i.e. on every tracking flight.
            i = int(round((px - self.street["ox"]) / res))
            j = int(round((py - self.street["oy"]) / res))
            if not (0 <= i < st.shape[0] and 0 <= j < st.shape[1]):
                return False
            return bool(st[i, j])

        def _clear_of_obstacles(px: float, py: float) -> bool:
            """Far enough from every MAPPED obstacle. Cheap Chebyshev scan of the
            occupancy grid rather than a distance transform, because slide() runs
            inside the 10 Hz control loop."""
            if not self.occ:
                return True
            occ, res = self.occ["occ"], self.occ["res"]
            ox, oy = self.occ["ox"], self.occ["oy"]
            r = int(math.ceil(self.min_clearance_m / res))
            i0 = int(round((px - ox) / res))
            j0 = int(round((py - oy) / res))
            n, m = occ.shape
            for i in range(max(0, i0 - r), min(n, i0 + r + 1)):
                for j in range(max(0, j0 - r), min(m, j0 + r + 1)):
                    if occ[i, j] and math.hypot(ox + i * res - px,
                                                oy + j * res - py) < self.min_clearance_m:
                        return False
            return True

        def _ahead_clear(px: float, py: float) -> bool:
            pts = [(px + ux * a, py + uy * a)
                   for a in (probe_m * 0.5, probe_m, probe_m * 1.5)]
            if self.polys and not all(
                    min(poly.distance(Point(*q)) for poly in self.polys)
                    >= self.stand_off_m for q in pts):
                return False
            return all(_clear_of_obstacles(*q) and _on_street(*q) for q in pts)

        # No blockage, no detour. Without this the probe reaches past nothing
        # when the fence is still far away, BOTH sides score the minimum cost,
        # and the tie is broken by iteration order -- which silently picked WEST,
        # the closed end of the gap policy, for the whole distant approach.
        # "Which way round" is only a question once there is something in the way.
        if _ahead_clear(x, y):
            return 0.0, 0.0, float("inf")

        best_cost, bx, by = float("inf"), 0.0, 0.0
        n = max(1, int(reach_m / step_m))
        for sgn in (1.0, -1.0):
            for k in range(1, n + 1):
                off = k * step_m
                px = x + lx * sgn * off
                py = y + ly * sgn * off
                # Standing here must itself be legal, with the stand-off kept.
                if self.polys and min(poly.distance(Point(px, py))
                                      for poly in self.polys) < self.stand_off_m:
                    continue
                if not (_clear_of_obstacles(px, py) and _on_street(px, py)):
                    continue          # outside the fence but off the street
                # ...and the way ahead from here must be open for a real distance,
                # not merely one step: a one-step gap is a corner, not a route.
                if _ahead_clear(px, py):
                    if off < best_cost:
                        best_cost, bx, by = off, lx * sgn, ly * sgn
                    break                     # shortest detour on this side
        return bx, by, best_cost


_OVERLAY = None          # policy_hud.OverlayCache, made on first use


def annotate(img, det, hud: dict):
    """Draw what the drone is actually seeing and deciding, for the demo.

    A raw camera frame proves nothing to a viewer — the whole claim is that a
    word picked the box and the box drove the aircraft, so both have to be on
    screen at once.
    """
    from PIL import ImageDraw
    im = img.copy()
    d = ImageDraw.Draw(im)
    W, H = im.size
    # The policy indicator (demo/policy_hud.py) when the loop supplied one;
    # without it the frame is drawn exactly as before, so old flights re-render
    # the same.
    pol = hud.get("policy")
    if pol is not None:
        from policy_hud import font
        f_tag = font(int(13 * W / 1280))
    if det is not None:
        cx, cy, bw, bh = det[0], det[1], det[2], det[3]
        x0, y0, x1, y1 = cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2
        # Box colour says how much the identity check believes it: green for a
        # box that passed every test, amber for one only allowed to continue a
        # track it agrees with (a car half hidden behind a truck).
        col = (255, 200, 0) if hud.get("tier") == "soft" else (0, 255, 0)
        d.rectangle([x0, y0, x1, y1], outline=col, width=2)
        d.line([(cx, 0), (cx, H)], fill=col + (90,))
        tag = f"{hud['query']}  p={det[4]:.3f}"
        if len(det) > 7:
            tag += f"  colour={det[7]:.2f}"
        if pol is None:
            d.text((max(2, x0), max(2, y0 - 11)), tag, fill=col)
        else:
            # The identity tier by its display name (REJECT / DOUBTFUL / OK),
            # never the stored "hard" / "soft" - see identity.TIER_LABEL.
            if hud.get("tier"):
                from identity import TIER_LABEL
                tag += f"  {TIER_LABEL.get(hud['tier'], hud['tier'])}"
            d.text((max(2, x0), max(2, y0 - 16 * W / 1280)), tag, fill=col, font=f_tag)
    d.line([(W / 2, H / 2 - 8), (W / 2, H / 2 + 8)], fill=(255, 255, 255))
    d.line([(W / 2 - 8, H / 2), (W / 2 + 8, H / 2)], fill=(255, 255, 255))
    _obstacle = hud.get("fence_kind") == "obstacle"
    _what = "OBSTACLE" if _obstacle else "NFZ"
    _what_lc = "obstacle" if _obstacle else "no-fly zone"
    lines = [
        f"t {hud['t']:5.1f}s   alt {hud['alt']:4.1f} m",
        f"bearing {hud['brg']:+6.1f}deg   fwd {hud['fwd']:+4.1f} m/s",
        (f"separation {hud['sep']:5.1f} m" if hud.get("sep") is not None
         else "separation n/a"),
        # "NFZ" only when a no-fly POLYGON caused it. With none declared the
        # guard holds for buildings, and the HUD used to call that an NFZ too -
        # in the reference video "NFZ AHEAD - HOLDING" at a building corner.
        (f"{_what} - SKIRTING AROUND" if hud.get("fence_mode") == "skirt"
         else f"{_what} AHEAD - HOLDING" if hud.get("fence_hold")
         else (f"{_what_lc} {hud['fence_d']:.0f} m ahead"
               if hud.get("fence_d") is not None and hud["fence_d"] < 20
               else "GUARDRAIL: correcting" if hud["shield"] else "GUARDRAIL: clear")),
        hud.get("mode_label") or {
            "track": "TARGET LOCKED", "coast": "COASTING on last motion",
            "search": "SEARCHING ...", "scan": "SCANNING for target"}.get(
            hud.get("mode"), "SCANNING for target"),
    ]
    if pol is not None:
        # The guardrail line moves to the banner and the rule panel, which
        # say which rule and why; the flight lines get a legible size.
        # Drawn at 5 Hz and pasted in between (policy_hud.OverlayCache): drawn
        # on every 20 Hz frame it took the detector from 2.89 to 2.43 Hz.
        global _OVERLAY
        if _OVERLAY is None:
            from policy_hud import OverlayCache
            _OVERLAY = OverlayCache()
        _OVERLAY.draw(im, pol, [lines[0], lines[1], lines[2], lines[4]])
        return im
    for i, ln in enumerate(lines):
        d.text((6, 6 + 11 * i), ln,
               fill=((255, 90, 90) if ("HOLDING" in ln or "correcting" in ln)
                     else (120, 220, 255) if "SKIRTING" in ln
                     else (255, 200, 90) if ("no-fly zone" in ln or "obstacle" in ln)
                     else (255, 255, 255)))
    return im


class Grounder:
    """Open-vocabulary detector in a background thread: text in, box out.

    Publishes the latest detection; the control loop reads it without blocking.
    Detection scores for a small distant object are low in absolute terms
    (0.03-0.07 measured at 22 m), so the box is accepted on a low threshold and
    filtered by continuity instead: a detection far from the last accepted one is
    rejected unless nothing has been seen for a while. That rejects the
    occasional confident tree without needing a confident car.
    """

    def __init__(self, obs: SemanticObs, query: str, thresh: float = 0.02,
                 jump_frac: float = 0.35, log_path: Path | None = None,
                 colour_min: float = 0.10, lock: "TargetLock | None" = None,
                 colour_mask: bool = True, legacy_sat: bool = False,
                 identity: dict | None = None):
        # Measure the colour on the object's pixels rather than the whole box.
        # `mask_stats` records which path actually ran, because a fix that
        # silently never engages is the failure mode to watch for here - the
        # start-heading fault failed silently for an unknown number of runs.
        self.colour_mask = bool(colour_mask)
        self.legacy_sat = bool(legacy_sat)
        self.mask_stats: dict = {"masked": 0, "whole_box": 0}
        self.obs = obs
        # Instance persistence. None keeps the historical behaviour exactly:
        # take the best-ranked candidate every tick and never ask whether it is
        # the same object as last tick.
        self.lock = lock
        self.query = query
        self.thresh = thresh
        self.jump_frac = jump_frac
        self.colour = colour_word(query)
        self.colour_min = colour_min
        if self.colour:
            print(f"[grounder] colour prior: {self.colour!r} "
                  f"(a box must be >={colour_min:.0%} that colour to qualify)")
        self.log_path = log_path
        self._lock = threading.Lock()
        self._stop = False
        # Bumped by retarget(). The worker builds its query list once, outside
        # its loop, so re-assigning self.query alone would change nothing - the
        # detector would keep asking for the old phrase while every other part
        # of the system believed the target had changed. The counter is what
        # makes the swap actually reach the model.
        self._query_gen = 0
        self._seq = 0
        self._det = None          # (cx, cy, w, h, score, W, H, colour)
        self._app = None          # HSV histogram of the chosen box
        self._t = 0.0             # last time the detector RAN
        self._t_det = 0.0         # last time it actually FOUND the target
        self.drop_after = 8.0     # forget the box entirely after this long
        self._infer_ms = 0.0
        self._pre_ms = 0.0
        self._fwd_ms = 0.0
        self._n_seen = 0
        self._n_miss = 0
        # ACQUISITION. The jump gate and the instance lock both protect a
        # subject the system already has. Before it has one they protect
        # whatever happened to be seen first: on citylife_redcar a stationary
        # red object 22 deg right of the nose was the first box, the jump gate
        # then discarded every candidate more than 35 % of the frame away from
        # it, the lock held it for 1,101 inferences, and the red car - never
        # plausible-looking box number one - was not even a candidate for 300 s.
        # While `acquiring`, neither applies and every colour-passing candidate
        # is published; `commit()` ends it on the candidate the start gate
        # confirmed, seeding the lock and the jump memory with it.
        self.acquiring = False
        self._cands: list = []
        self._seed = None
        # IDENTITY (demo/identity.py; 2026-09-29). None keeps every behaviour
        # above exactly. A dict {thresholds, street, hfov} makes each candidate
        # carry its physical description and a tier - "hard" (not the named
        # thing: dropped), "soft" (unlikely: may continue a track, never start
        # one) or "ok" - and makes the lock strict (TargetLock.select_strict):
        # it answers None rather than adopting the best-scoring box.
        self.identity = identity
        self._prior = None          # estimator snapshot, see set_prior()
        self._cand_tiers: list = []
        self._cand_feats: list = []
        self._det_tier = None
        self._det_feat = None
        self._det_cap = None        # capture meta of the published box
        self._lock_why = None
        self.id_counts = {"ok": 0, "soft": 0, "hard": 0}
        self.id_rules: dict = {}    # "hard:<rule>" / "soft:<rule>" -> candidates
        self.pose_src: dict = {}    # where each inference's pose came from
        self.depth_src: dict = {}   # ...and its depth
        self._thread = threading.Thread(target=self._worker, daemon=True)

    def set_prior(self, snap: dict | None) -> None:
        """The estimator's latest state for the strict lock: {t, x, y, vx, vy,
        t_upd} (t = when x/y hold, t_upd = its last measurement), or None when
        there is no estimate the lock may lean on (lapse, re-acquisition)."""
        self._prior = snap

    def _prior_at(self, pose, W: int, H: int, t_cap):
        """The prior projected into THIS frame: {cx or None, P, tol_m}.

        Through the real camera (camera_model.world_to_pixel: the 20 deg mount
        and the body's pitch and roll at capture), at the subject's centre
        height. The level-camera shortcut bearing_to_cx(azimuth) put a near,
        off-axis subject 20-45 px too far out - most of the SOFT gate
        (review, 2026-09-29)."""
        pr = self._prior
        if pr is None or pose is None or pose.get("up") is None:
            return None
        tq = t_cap if t_cap is not None else time.time()
        dt = max(0.0, tq - pr["t"])
        n = pr["x"] + pr["vx"] * dt
        e = pr["y"] + pr["vy"] * dt
        age = max(0.0, tq - pr.get("t_upd", pr["t"]))
        hfov = (self.identity or {}).get("hfov", CAMERA_HFOV_DEG)
        uv = _cam.world_to_pixel(n, e, float(pr.get("up", 0.75)), pose["x"], pose["y"],
                                 pose["up"], W, H, hfov, pose["yaw"],
                                 pose.get("pitch") or 0.0, pose.get("roll") or 0.0)
        # In the frame, both ways: behind or under the aircraft a point still
        # projects to a column (re-review, 2026-09-30), and a prior there must
        # not steer the lock - the subject is in the blind spot.
        cx = (float(uv[0]) if uv is not None and -0.1 * W <= uv[0] <= 1.1 * W
              and -0.1 * H <= uv[1] <= 1.05 * H else None)
        return {"cx": cx, "P": (n, e), "tol_m": min(10.0, 2.0 * age)}

    def retarget(self, phrase: str) -> None:
        """Point the detector at a different thing, without restarting anything.

        This is the whole open-vocabulary argument in one method. A conventional
        tracker must be handed a BOX and cannot be handed a NOUN; it returns an
        id, never a class. Here the target changes because the WORDS changed -
        no re-initialisation, no click, no retraining - and because the words
        carry a class, the Shield can pick a different stand-off rule for it.

        Everything tied to the OLD target has to go, and each for its own reason:

          * `_det` / `_app` - a cached box on the previous object. Left in place,
            the controller would keep steering at it for up to `drop_after`
            seconds after the operator had asked for something else.
          * `_t_det` - the freshness clock for that box. Not clearing it would
            make a stale detection look current.
          * the TargetLock - see TargetLock.reset; without it the new target is
            still adopted, but recorded as a silent target SWITCH, which is the
            one thing those counters exist to detect.
          * `last`, in the worker - the jump-suppression memory. The new target
            is somewhere else in frame by definition, so the old position would
            veto every candidate that is actually correct.

        `colour` is recomputed here because the colour gate reads it live on
        every candidate; `queries` is rebuilt by the worker off `_query_gen`.
        """
        with self._lock:
            self.query = phrase
            self.colour = colour_word(phrase)
            self._det = None
            self._app = None
            self._t_det = 0.0
            # ...and the candidate list: a re-acquisition started for the NEW
            # phrase would otherwise step on the OLD phrase's boxes (review,
            # 2026-09-29).
            self._cands, self._cand_tiers, self._cand_feats = [], [], []
            self._lock_why = None
            self._inf_cap = None
            self._query_gen += 1
        if self.lock is not None:
            self.lock.reset()

    def begin_acquire(self) -> None:
        """Look for the subject without assuming where it is (see __init__)."""
        with self._lock:
            self.acquiring = True
            self._det = None
            self._t_det = 0.0
        if self.lock is not None:
            self.lock.reset()

    def begin_reacquire(self) -> None:
        """The subject was lost for good (estimate lapsed): publish every
        candidate again, hold nothing, and let the caller's gate decide
        (see Reacquirer). Same as begin_acquire, plus no estimator prior."""
        self._prior = None
        self.begin_acquire()

    def commit(self, cand, yaw: float = None) -> None:
        """End acquisition. `cand` (an 8-tuple) becomes the held instance and
        the jump gate's reference; None ends it with no seed, so tracking
        starts from whatever is seen next - the historical behaviour.

        `yaw` is the aircraft's yaw when `cand`'s frame was CAPTURED. The lock
        predicts where the held box moves from the yaw change since then;
        seeding it with the yaw of the moment the worker picked the seed up,
        ~0.5 s later while the aircraft was turning toward the candidate,
        put the prediction ~60 px off (review, 2026-09-29)."""
        with self._lock:
            self.acquiring = False
            self._seed = None if cand is None else (cand, yaw)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop = True

    def _worker(self):
        import torch
        from transformers import OwlViTProcessor, OwlViTForObjectDetection
        t0 = time.time()
        proc = OwlViTProcessor.from_pretrained(DETECTOR_ID)
        model = OwlViTForObjectDetection.from_pretrained(DETECTOR_ID).to("cuda").eval()
        print(f"[grounder] {DETECTOR_ID} loaded in {time.time()-t0:.0f}s "
              f"(VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB)", flush=True)
        queries = [[self.query]]
        gen = self._query_gen
        last = None
        while not self._stop:
            # Pick up a retarget. Cheap: an int compare per frame, and the
            # rebuild only runs on the tick the phrase actually changed.
            if self._query_gen != gen:
                with self._lock:
                    gen = self._query_gen
                    queries = [[self.query]]
                # The jump gate compares against where the OLD target was, so it
                # would reject the new one on exactly the frames that matter.
                last = None
                print(f"[grounder] retargeted -> {queries[0][0]!r}", flush=True)
            if self._seed is not None:
                with self._lock:
                    (seed, seed_yaw), self._seed = self._seed, None
                last = (float(seed[0]), time.time())
                if self.lock is not None:
                    self.lock.reset()
                    yaw_s = (seed_yaw if seed_yaw is not None else
                             self.obs.pose[2] if getattr(self.obs, "pose", None)
                             else 0.0)
                    if self.identity is not None:
                        self.lock.seed(seed, yaw_s, time.time())
                    else:
                        self.lock.select([seed], int(seed[5]), yaw_s, time.time())
            acquiring = self.acquiring
            # THE FRAME, AND THE POSE AND DEPTH OF THAT FRAME. get_front_capture
            # pairs the image with the pose interpolated at its capture and the
            # depth frame nearest it in time (None beyond 70 ms); before
            # 2026-09-29 a box met whatever depth had arrived last and, in the
            # control loop, the pose of the tick that consumed it.
            meta = None
            if hasattr(self.obs, "get_front_capture"):
                img, meta = self.obs.get_front_capture()
            else:
                img = self.obs.get_front_native()
            if img is None:
                time.sleep(0.05)
                continue
            cap_pose = meta.get("pose") if meta else None
            pose_src = cap_pose.get("src", "ring") if cap_pose else "none"
            self.pose_src[pose_src] = self.pose_src.get(pose_src, 0) + 1
            t_cap = meta.get("t_cap") if meta else None
            # When the frame was CAPTURED (sim stamp through the pose ring),
            # not when it arrived: see SemanticObs.get_front_capture.
            t_capture = (meta.get("t_capture") or t_cap) if meta else None
            W, H = img.size
            # Split, because the total alone cannot say WHY it is slow. Offline
            # on an idle GPU this whole block is 66 ms (26.7 CPU preprocessing,
            # 36.2 forward) at the real 400x225 input - see
            # experiments/profile_owlvit.py. In flight it measures 287 ms, which
            # is 4.3x slower and is NOT explained by capture resolution: it
            # barely moved across a 2.4x change in Chase pixels and not at all
            # across a 1.8x change in window pixels.
            #
            # So the slowdown comes from sharing the machine with the simulator,
            # and the two halves point at different culprits. Preprocessing is
            # CPU and would be starved by the recorder's JPEG encoding, the
            # control loop and msgpack deserialisation. The forward pass is GPU
            # and would be starved by Unreal's rendering. Logging them apart is
            # the difference between knowing and guessing.
            t = time.time()
            inputs = proc(text=queries, images=img, return_tensors="pt").to("cuda")
            t_pre = time.time()
            with torch.no_grad():
                out = model(**inputs)
            torch.cuda.synchronize()
            t_fwd = time.time()
            pre_ms = (t_pre - t) * 1000
            fwd_ms = (t_fwd - t_pre) * 1000
            ms = (t_fwd - t) * 1000
            res = proc.post_process_object_detection(
                out, threshold=0.0,
                target_sizes=torch.tensor([[H, W]]).to("cuda"))[0]
            sc, bx = res["scores"], res["boxes"]
            # One depth frame per inference, not per candidate box. It indexes
            # straight into the scene image because both captures are configured
            # at the same resolution and the same field of view - the same
            # property range_from_depth already relies on.
            #
            # THE CAPTURE'S OWN DEPTH, OR NONE. Paired after the forward pass,
            # when the depth frame of this capture (10 Hz, same moment) has
            # usually arrived; the newest frame instead - the old fallback - is
            # by construction further off, and at 0.5 rad/s of yaw 200 ms is
            # ~38 px: the range window lands off a 30 px car (review,
            # 2026-09-29). With no depth within 70 ms the box gets no range
            # (identity: SOFT "no range") and colour is measured on the whole box.
            dep_dt = (meta or {}).get("depth_dt_ms")
            if meta is not None and meta.get("depth") is not None:
                dep, dep_src = meta["depth"], "capture"
            elif meta is not None and hasattr(self.obs, "depth_near"):
                dep, dep_dt = self.obs.depth_near(meta.get("stamp"), t_cap)
                dep_src = "capture-late" if dep is not None else "none-unpaired"
            elif self.colour_mask or self.identity is not None:
                dep, dep_src, dep_dt = self.obs.get_depth(), "latest", None
            else:
                dep, dep_src, dep_dt = None, "none", None
            self.depth_src[dep_src] = self.depth_src.get(dep_src, 0) + 1
            det, switched, cands = None, False, []
            if len(sc):
                # Score every plausible box, do not just take the detector's top
                # one. Ranking by detector score alone is what let a city object
                # win and hold the aircraft 60 m from the car.
                for i in sc.argsort(descending=True)[:12]:
                    s = float(sc[i])
                    if s < self.thresh:
                        break
                    x0, y0, x1, y1 = [float(v) for v in bx[i].tolist()]
                    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
                    if (last is not None and not acquiring
                            and (time.time() - last[1]) < 1.5):
                        if abs(cx - last[0]) > self.jump_frac * W:
                            continue          # too far from where it just was
                    cm = colour_match(img, (x0, y0, x1, y1), self.colour,
                                      depth=dep, stats=self.mask_stats,
                                      legacy_sat=self.legacy_sat)
                    if self.colour is not None and cm < self.colour_min:
                        continue              # right shape, wrong colour
                    cands.append((cx, cy, x1 - x0, y1 - y0, s, W, H, cm))
            id_all, tiers, feats, det_tier, det_feat, lock_why = [], [], [], None, None, None
            if cands and self.identity is not None:
                # Every candidate described and judged, and the HARD ones out.
                ranked = []
                for c in cands:
                    tier, why, f = self._judge(c, dep, cap_pose, W, H)
                    id_all.append((c, tier, why, f))
                    if tier != "hard":
                        ranked.append((c, tier, f))
                # OK before SOFT, then score-and-colour as before.
                ranked.sort(key=lambda r: (r[1] != "ok",
                                           -(r[0][4] * (0.25 + 0.75 * r[0][7]))))
                cands = [r[0] for r in ranked]
                tiers = [r[1] for r in ranked]
                feats = [r[2] for r in ranked]
                if cands and self.lock is not None and not acquiring:
                    yaw_c = (cap_pose["yaw"] if cap_pose else
                             (self.obs.pose[2] if getattr(self.obs, "pose", None)
                              else 0.0))
                    k, lock_why = self.lock.select_strict(
                        cands, tiers, feats, W, yaw_c, time.time(),
                        prior=self._prior_at(cap_pose, W, H, t_capture))
                    if k is not None:
                        det, det_tier, det_feat = cands[k], tiers[k], feats[k]
                elif cands and self.lock is None and not acquiring:
                    det, det_tier, det_feat = cands[0], tiers[0], feats[0]
                    if det_tier != "ok":
                        det, det_tier, det_feat = None, None, None
                # While acquiring nothing is published as THE box: the caller's
                # gate judges every candidate and commits one.
            elif cands:
                # Rank by score-and-colour, then let the instance lock decide
                # WHICH of the survivors is the one we were already following.
                # Ranking alone answers "is this the right kind of thing"; it has
                # nothing to say about "is this the same one", and with several
                # identical vehicles or nine city blocks those are different
                # questions.
                cands.sort(key=lambda c: -(c[4] * (0.25 + 0.75 * c[7])))
                if self.lock is None or acquiring:
                    det = cands[0]
                else:
                    yaw_now = (self.obs.pose[2] if getattr(self.obs, "pose", None)
                               else 0.0)
                    det, switched = self.lock.select(cands, W, yaw_now, time.time())
            app = None
            if det is not None:
                x0 = det[0] - det[2] / 2, det[1] - det[3] / 2
                app = appearance(img, (x0[0], x0[1],
                                       det[0] + det[2] / 2, det[1] + det[3] / 2))
            with self._lock:
                # A retarget that landed inside this inference: nothing of it may
                # be published - not the box, and not the candidate list either,
                # which a re-acquisition for the NEW phrase would step on (the
                # candidates were even judged under the new noun's rules).
                if self._query_gen != gen:
                    # select() may have seeded the lock with the OLD phrase's
                    # box after retarget() reset it (re-review, 2026-09-30)
                    if self.lock is not None:
                        self.lock.reset()
                    self._n_miss += 1
                    continue
                self._seq += 1
                self._infer_ms = ms
                self._pre_ms = pre_ms
                self._fwd_ms = fwd_ms
                # `_t` is when the detector last RAN. `_t_det` is when it last
                # actually FOUND something. Conflating the two was a real bug:
                # the control loop aged the detection against `_t`, which
                # refreshes on every frame including misses, so a stale box was
                # treated as fresh forever. The aircraft chased a box that was no
                # longer there and the HUD kept reporting TARGET LOCKED.
                self._t = time.time()
                self._cands = list(cands)
                self._cand_tiers = list(tiers)
                self._cand_feats = list(feats)
                self._lock_why = lock_why
                self._inf_cap = {"t_cap": t_cap, "t_capture": t_capture, "pose": cap_pose}
                # (The generation is re-checked at the top of this block, not
                # only at the top of the loop: an inference takes ~287 ms, and a
                # retarget inside it would otherwise publish the OLD subject box
                # under the NEW phrase - the first sighting of the new target,
                # which sets the size gate for everything after it.)
                if det is not None:
                    self._det = det
                    self._app = app
                    self._det_switched = bool(switched)
                    self._t_det = time.time()
                    self._det_tier = det_tier
                    self._det_feat = det_feat
                    self._det_cap = {"t_cap": t_cap, "t_capture": t_capture,
                                     "pose": cap_pose,
                                     "depth_src": dep_src,
                                     "depth_dt_ms": dep_dt}
                    self._n_seen += 1
                    last = (det[0], time.time())
                else:
                    self._n_miss += 1
                    # drop the box once it is unusably old, so nothing downstream
                    # can accidentally act on it
                    if self._t_det and time.time() - self._t_det > self.drop_after:
                        self._det = None
            if self.log_path is not None:
                rec = {"seq": self._seq, "t": self._t, "infer_ms": round(ms, 1),
                       "query": self.query,
                       "switched": bool(switched),
                       "det": (None if det is None else
                               {"cx": round(det[0], 1), "cy": round(det[1], 1),
                                "w": round(det[2], 1), "h": round(det[3], 1),
                                "score": round(det[4], 4), "colour": round(det[7], 3),
                                "img_w": det[5], "img_h": det[6]}),
                       # The RUNNER-UP, and how many candidates there were.
                       #
                       # Only the winner used to be logged, so the margin
                       # between first and second choice was unrecoverable from
                       # any artefact - and a selection failure cannot be
                       # diagnosed after the fact without it. Establishing that
                       # one flight had followed the wrong vehicle needed a
                       # ground-truth reconstruction precisely because this was
                       # missing. Two extra numbers per tick is a cheap price.
                       "n_cands": len(cands),
                       "acquiring": bool(acquiring),
                       # While acquiring, every candidate: the start gate judges
                       # all of them, so a failed gate must be explainable from
                       # this file alone.
                       "cands": ([[round(c[0], 1), round(c[1], 1), round(c[2], 1),
                                   round(c[3], 1), round(c[4], 4), round(c[7], 3)]
                                  for c in cands[:6]] if acquiring else None),
                       "runner_up": (None if len(cands) < 2 else
                                     {"cx": round(cands[1][0], 1),
                                      "w": round(cands[1][2], 1),
                                      "score": round(cands[1][4], 4),
                                      "colour": round(cands[1][7], 3)})}
                if self.identity is not None:
                    # EVERY candidate, HARD ones included, with what it was
                    # judged on - so a tier can be re-derived offline and a
                    # lock decision explained from this file alone.
                    rec["t_cap"] = None if t_cap is None else round(t_cap, 3)
                    rec["t_capture"] = None if t_capture is None else round(t_capture, 3)
                    rec["stamp"] = (meta or {}).get("stamp")
                    rec["pose_src"] = pose_src
                    rec["depth_src"] = dep_src
                    rec["depth_dt_ms"] = dep_dt
                    rec["pose_cap"] = (None if cap_pose is None else
                                       {k: round(float(cap_pose[k]), 4)
                                        for k in ("x", "y", "up", "yaw", "pitch",
                                                  "roll") if cap_pose.get(k) is not None})
                    rec["tier"] = det_tier
                    rec["lock_why"] = lock_why
                    rec["id"] = [_id_record(c, tier, why, f)
                                 for c, tier, why, f in id_all]
                with self.log_path.open("a", encoding="utf-8") as fh:
                    fh.write(json.dumps(rec) + "\n")

    def _judge(self, c, dep, pose, W, H):
        """(tier, reasons, features) of one candidate. No capture pose, no
        verdict beyond SOFT: a box cannot be placed without knowing where the
        camera was."""
        import identity as _id
        if pose is None or pose.get("up") is None:
            tier, why, f = "soft", ["no capture pose"], {}
        else:
            r = range_from_depth(dep, c) if dep is not None else None
            # (A box cut off by the bottom of the frame has no bottom edge:
            # features_for sets bottom_clipped and leaves bottom_h None.)
            f = _id.features_for(c, r, pose, W, H,
                                 self.identity.get("hfov", CAMERA_HFOV_DEG),
                                 self.identity.get("street"))
            tier, hard, soft = _id.classify_split(f, self.query,
                                                  self.identity.get("thresholds"))
            why = hard + soft if tier == "hard" else soft if tier == "soft" else []
        self.id_counts[tier] = self.id_counts.get(tier, 0) + 1
        # Each reason under the tier of the RULE that produced it: a HARD box
        # also carries SOFT reasons, which only ever demote (review, 09-29).
        if pose is None or pose.get("up") is None:
            by = (("soft", why),)
        else:
            by = (("hard", hard), ("soft", soft if tier != "ok" else []))
        seen = set()
        for rule_tier, reasons in by:
            for w in reasons:
                k = f"{rule_tier}:{identity_rule_key(w)}"
                if k not in seen:
                    seen.add(k)
                    self.id_rules[k] = self.id_rules.get(k, 0) + 1
        return tier, why, f

    def latest(self) -> dict:
        with self._lock:
            return {"seq": self._seq, "t": self._t, "t_det": self._t_det,
                    "det": self._det, "app": self._app, "infer_ms": self._infer_ms,
                    "switched": getattr(self, "_det_switched", False),
                    "pre_ms": self._pre_ms, "fwd_ms": self._fwd_ms,
                    "n_seen": self._n_seen, "n_miss": self._n_miss,
                    "cands": list(self._cands),
                    "cand_tiers": list(self._cand_tiers),
                    "cand_feats": list(self._cand_feats),
                    "tier": self._det_tier, "feat": self._det_feat,
                    "cap": self._det_cap, "lock_why": self._lock_why,
                    "inf_cap": getattr(self, "_inf_cap", None)}


def identity_rule_key(reason: str) -> str:
    """The rule an identity reason string came from, for counting."""
    w = (reason or "").lower()
    far = w.startswith("far:")
    for key, marker in (("bottom", "above the"), ("off-street", "off the street"),
                        ("frame", "of the frame"), ("no-range", "no range"),
                        ("no-pose", "no capture pose"), ("aspect", "aspect"),
                        ("score", "score"), ("h-px", "px tall")):
        if marker in w:
            return ("far-" if far else "") + key
    if "wide >" in w:
        return "width-max"
    if "wide <" in w:
        return "width-min"
    return "other"


def _id_record(c, tier, why, f) -> dict:
    """One candidate for detections.jsonl: the box, the verdict, and the
    features it was reached on (rounded; P as [north, east, up])."""
    def r(v, n=2):
        return None if v is None else round(float(v), n)
    P = (f or {}).get("P")
    return {"cx": r(c[0], 1), "cy": r(c[1], 1), "w": r(c[2], 1), "h": r(c[3], 1),
            "score": r(c[4], 4), "colour": r(c[7], 3), "tier": tier, "why": why,
            "r": r((f or {}).get("r")), "rng_h": r((f or {}).get("rng_h")),
            "width_m": r((f or {}).get("width_m")),
            "aspect": r((f or {}).get("aspect")),
            "bottom_h": r((f or {}).get("bottom_h")),
            "off_street_m": r((f or {}).get("off_street_m")),
            "P": None if P is None else [r(v) for v in P]}


def servo(det, img_w: int, yaw_gain: float, want_w_frac: float,
          speed_max: float, hfov_deg: float = CAMERA_HFOV_DEG):
    """Box in the image -> (yaw rate rad/s, forward speed m/s, bearing error rad).

    Bearing comes from the horizontal offset scaled by the real horizontal FOV,
    so the yaw command is in true angular units rather than arbitrary pixels.
    Forward speed closes on a target apparent width: too small means too far, so
    accelerate; too large means too close, so back off.
    """
    cx, _cy, bw, _bh, _s, W, _H = det[:7]
    bearing = box_bearing(cx, W, hfov_deg)
    yaw_rate = yaw_gain * bearing
    w_frac = bw / W
    err = (want_w_frac - w_frac) / max(1e-3, want_w_frac)
    fwd = float(np.clip(err * speed_max, -0.4 * speed_max, speed_max))
    # do not charge forward while the target is far off to one side
    fwd *= max(0.0, math.cos(bearing))
    return yaw_rate, fwd, bearing


async def _fly_to_landing_site(drone, shield, args, timeout_s: float = 45.0,
                               pind=None, recorder=None, t0=None, query="") -> dict:
    """Fly at cruise altitude, through the Shield, to a cell demo/landing.py
    calls landable, before the vertical descent.

    The descent used to start wherever the mission ended: in a hedge on
    citylife_redcar_trail2, on a parked car on citylife_ped_final. Returns what
    was chosen and whether it was reached, for metrics.json."""
    info = {"enabled": True}
    try:
        import landing as landing_mod
        map_dir = Path(args.citymap).parent
        # The car lanes and the street grid landing.py knows are CityLife's
        # (tools/citylife_routes.py). On any other level they would mark the
        # wrong cells as carriageway, so there the site is judged on the maps
        # alone (no lane rule, no pavement preference) and the record says so.
        citylife = map_dir.name == "citymap_citylife"
        maps = (landing_mod.load_landing_maps(map_dir) if citylife else
                landing_mod.load_landing_maps(map_dir, lane_paths={}, road_grid={}))
        info["lanes_known"] = citylife
        kin = drone.get_ground_truth_kinematics()
        p = kin["pose"]["position"]
        x0, y0 = float(p["x"]), float(p["y"])
        r = landing_mod.choose_site(maps, x0, y0, prefer="pavement" if citylife else None)
    except Exception as exc:
        info["error"] = f"{type(exc).__name__}: {exc}"
        print(f"[land] *** no landing site chosen ({info['error']}); descending here")
        return info
    info.update(start=[round(x0, 2), round(y0, 2)], here_landable=r["here_landable"],
                reasons_here=r["reasons_rejected_here"],
                site=(None if r["site"] is None else [round(v, 2) for v in r["site"]]),
                path_m=(None if r.get("path_m") is None else round(r["path_m"], 1)),
                crosses_building=r.get("crosses_building"),
                lanes_checked=r.get("lanes_checked"))
    if r["site"] is None:
        print("[land] *** no landable cell within reach; descending here")
        return info
    sx, sy = r["site"]
    print(f"[land] landing site ({sx:.1f}, {sy:.1f}), {info['path_m']} m away"
          f"{'' if not r['reasons_rejected_here'] else ' - here: ' + '; '.join(r['reasons_rejected_here'][:2])}")
    shield.set_subject(None)
    t_end, reached, n_touched = time.time() + timeout_s, False, 0
    while time.time() < t_end:
        kin = drone.get_ground_truth_kinematics()
        p = kin["pose"]["position"]
        up = -p["z"]
        dx, dy = sx - p["x"], sy - p["y"]
        d = math.hypot(dx, dy)
        if d < 0.8:
            reached = True
            break
        v = min(3.0, 0.6 * d)
        vz = float(np.clip((args.cruise_alt - up) * args.alt_gain,
                           -args.climb_max, args.climb_max))
        dec = shield.filter(State(x=p["x"], y=p["y"], up=up),
                            Action4D(vx=v * dx / d, vy=v * dy / d, vz_up=vz,
                                     yaw_rate=0.0))
        n_touched += int(dec.touched)
        if pind is not None and recorder is not None:
            # The flight to the landing site goes through the Shield, so the
            # indicator stays live for it rather than freezing on the last
            # mission tick.
            try:
                st = State(x=float(p["x"]), y=float(p["y"]), up=up)
                pol = pind.update(time.time() - t0, dec, st,
                                  clearance_m=(shield.clearance_at(st.x, st.y)
                                               if shield.has_obstacle_map else None),
                                  off_map=shield.off_map(st.x, st.y),
                                  yaw_rad=quat_yaw(kin["pose"]["orientation"]))
                recorder.set_hud({
                    "t": time.time() - t0, "sep": None, "brg": 0.0, "fwd": v,
                    "alt": up, "shield": dec.touched, "query": query, "mode": "land",
                    "mode_label": f"LANDING - to site, {d:.0f} m", "tier": None,
                    "policy": pol}, None)
            except Exception as exc:                            # noqa: BLE001
                print(f"[hud] landing indicator stopped: {type(exc).__name__}: {exc}")
                pind = None
        e = dec.emitted
        await drone.move_by_velocity_async(e.vx, e.vy, -e.vz_up, duration=0.3,
                                           yaw_is_rate=True, yaw=e.yaw_rate)
        await asyncio.sleep(0.1)
    await drone.move_by_velocity_async(0.0, 0.0, 0.0, duration=0.3)
    kin = drone.get_ground_truth_kinematics()
    p = kin["pose"]["position"]
    ok, why = landing_mod.is_landable(maps, float(p["x"]), float(p["y"]))
    info.update(reached=reached, shield_touched=n_touched,
                final=[round(float(p["x"]), 2), round(float(p["y"]), 2)],
                final_landable=bool(ok), final_reasons=why)
    return info


async def fly(args) -> int:
    from projectairsim import Drone, EnvActor, ProjectAirSimClient, World

    # Resolve the subject's real width from what was actually named, unless the
    # command line said otherwise. Printed either way: a width the flight chose
    # for itself has to be visible, because it scales every range estimate.
    # The word the width came from is also what a SubjectStandoff rule matches
    # on, so "a pedestrian" selects both the 0.5 m width and the 10 m stand-off
    # from one phrase rather than two flags that can disagree.
    subject_class = subject_width(args.object)[1]


    # Mid-flight retarget schedule: [(seconds, phrase), ...], soonest first.
    # Parsed here, before anything flies, so a typo is a start-up error rather
    # than a surprise twenty seconds into a recording.
    retargets: list[tuple[float, str]] = []
    for spec in (args.retarget or []):
        if ":" not in spec:
            raise SystemExit(f"--retarget {spec!r} is not SECONDS:PHRASE")
        when, _, phrase = spec.partition(":")
        try:
            t_at = float(when)
        except ValueError:
            raise SystemExit(f"--retarget {spec!r}: {when!r} is not a number")
        if not phrase.strip():
            raise SystemExit(f"--retarget {spec!r} has an empty phrase")
        retargets.append((t_at, phrase.strip()))
    retargets.sort(key=lambda r: r[0])
    # Declared at function scope, not inside the flight block: the block
    # is conditional and metrics is written either way, so a declaration
    # in there would be a NameError on any path that does not fly.
    retarget_events: list[dict] = []
    if retargets:
        print("[retarget] schedule: " + ", ".join(
            f"t+{t:.0f}s -> {p!r} (class "
            f"{subject_width(p)[1] or 'unclassified'})" for t, p in retargets))

    # Whether the width came from the operator or from the phrase. A retarget
    # re-derives it from the new phrase, but must never overwrite a number the
    # operator pinned by hand - they know something the word list does not.
    args.object_width_pinned = args.object_width_m is not None

    if args.object_width_m is None:
        args.object_width_m, matched = subject_width(args.object)
        if matched:
            print(f"[subject]  width prior: {args.object_width_m:.2f} m "
                  f"(class {matched!r} from {args.object!r})")
        else:
            print(f"[subject]  WARNING no width known for {args.object!r}; "
                  f"falling back to {args.object_width_m:.2f} m, a car. If the "
                  f"subject is not car-sized, every range estimate is wrong by "
                  f"the ratio - pass --object-width-m.")
    else:
        print(f"[subject]  width {args.object_width_m:.2f} m (from the command line)")

    # Seed every RNG the flight can reach. WP4 requires random_seed in the
    # determinism manifest, and a manifest that records a seed nothing honours
    # would be worse than no manifest at all.
    random.seed(args.seed)
    np.random.seed(args.seed & 0xFFFFFFFF)

    # The signed bundle when --bundle is given (refused unless its signature
    # verifies, or --allow-unverified-bundle records that it did not); else the
    # YAML, recorded as unsigned. Audit card ARCH-29: every flight used to read
    # YAML, so no run could show which issued policy was in force.
    # Loaded BEFORE the output folder is cleared: a refused bundle (or --bundle
    # with --policy) used to delete the previous run's flight log first and
    # leave its manifest, kpi.json and metrics.json describing a log that was
    # gone (2026-10-06 review).
    policy, policy_source = load_for_flight(
        args.bundle,
        args.policy or (None if args.bundle else DEFAULT_POLICY),
        allow_unverified=args.allow_unverified_bundle)

    out = ROOT / "demo" / "out" / args.tag
    out.mkdir(parents=True, exist_ok=True)
    for f in ("flight_log.jsonl", "detections.jsonl"):
        if (out / f).exists():
            (out / f).unlink()

    # A stand-off rule naming a class no phrase can ever produce is inert: it is
    # hashed, audited, and never fires. That is exactly how the 10 m pedestrian
    # rule sat dead through a whole flight while "a person" produced the class
    # "person" and the rule said "pedestrian". Nothing reported it, because
    # there was nothing to report - no violation is indistinguishable from no
    # rule. Check it before take-off, where it is cheap and loud.
    # A pedestrian flight with no pedestrians in it. `--pedestrians` defaults to
    # 0, so a phrase that selects the pedestrian class would log truth.pts == []
    # on every tick of that phase: the whole phase scores UNSCORABLE and the
    # flight says nothing about it. Same shape as the inert stand-off rule
    # below - the artefact looks complete and measures nothing - so it gets the
    # same treatment, a start-up refusal where it is cheap and loud.
    _phrases = [args.object] + [ph for _, ph in retargets]
    _ped_phrases = [ph for ph in _phrases if subject_width(ph)[1] == "pedestrian"]
    if _ped_phrases and args.pedestrians <= 0 and args.level_peds <= 0:
        raise SystemExit(
            f"{_ped_phrases[0]!r} selects the pedestrian class, but "
            f"--pedestrians is {args.pedestrians} and --level-peds is "
            f"{args.level_peds}, so no pedestrian exists to be ground truth. "
            f"Every tick of that phase would log an empty truth and score as "
            f"unscorable. Pass --pedestrians N (the demo uses 12) for "
            f"client-spawned figures, --level-peds N in a level that walks its "
            f"own, or follow something that is in the scene.")
    if args.pedestrians > 0 and args.level_peds > 0:
        raise SystemExit(
            "--pedestrians and --level-peds both place pedestrians in the "
            "truth, from two sources that do not know about each other. Pick "
            "one: the client's baked figures, or the level's own.")
    # A car the LEVEL drives replaces the client's scripted one; with both, the
    # truth row would describe one car and the camera would be looking at two.
    if args.level_car:
        if not args.no_car or args.traffic > 0:
            raise SystemExit(
                "--level-car takes the subject from the level, so the client "
                "must not drive one of its own: pass --no-car and leave "
                "--traffic at 0.")
        if subject_class not in ("car", "van", "truck", "bus"):
            raise SystemExit(
                f"--level-car {args.level_car} is a vehicle, but {args.object!r} "
                f"selects the class {subject_class!r}. The truth row would score "
                f"a car against a box drawn on something else.")

    # NOTE on ordering: both refusals below run AFTER fly() has unlinked the
    # tag's previous flight_log.jsonl and detections.jsonl. That unlink predates
    # them, so what used to be "wipe, then fly" is now "wipe, then refuse to
    # fly" - a refused start-up destroys the last run's delivered artefacts
    # under the same tag. Left as-is deliberately: moving the unlink after the
    # checks would be a bigger change to the flight path than the fix is worth
    # eleven days from a demo, and the replay bundle is the durable copy. Do not
    # re-run a refused tag expecting its previous log to still be there.
    _reachable = set(SUBJECT_CLASS_CANON.values())
    for _so in policy.by_type(SubjectStandoff):
        if _so.subject_class != "*" and _so.subject_class not in _reachable:
            raise SystemExit(
                f"policy rule {_so.id!r} binds subject_class "
                f"{_so.subject_class!r}, which no --object phrase can produce. "
                f"The rule would never fire. Reachable classes: "
                f"{', '.join(sorted(_reachable))}.")
    cmap = city_planner.load_occ(args.citymap)

    # WHICH ALTITUDE IS THIS MAP TRUE FOR?
    #
    # The occupancy map is a 2-D projection of ONE altitude band, and every
    # flight loaded occ_day.npz (6-14 m) whatever the policy permitted. That is
    # correct for the car demos and not for this one: follow_pedestrian.yaml
    # permits descent to 4 m, the band maps are not contiguous, and NOTHING maps
    # 4-6 m. See docs/FINDING-the-map-was-true-for-the-wrong-altitude.md.
    #
    # This DOES change what some flights load, and an earlier version of this
    # comment claimed otherwise. Measured across the 8 policies carrying both an
    # AltitudeEnvelope and an ObstacleClearance, 5 get a different grid:
    #
    #   follow_car / follow_car_nfz / follow_pedestrian   identical (2015 cells)
    #   follow_car_gap  10-17 m   orbit_building 12-28 m  -> 2476 cells
    #   rth_poly / semantic_conflict / urban_clearance    -> 2212 cells (15-55 m)
    #
    # That is the point rather than a regression: a policy cruising at 12-28 m
    # was being checked against a map of the 6-14 m band. But it is a behaviour
    # change on flights the tutorial runs, so it is stated here rather than
    # asserted away.
    _alt = policy.by_type(AltitudeEnvelope)
    if policy.by_type(ObstacleClearance) and _alt:
        _sel = occ_bands.select_for_band(Path(args.citymap).parent,
                                         _alt[0].alt_min_m, _alt[0].alt_max_m)
        for _line in _sel["report"]:
            print(_line)
        if _sel["map"] is not None:
            cmap = _sel["map"]
        elif cmap is not None:
            # The selector answered "no band map covers this envelope". Keeping
            # the default would fly the aircraft on precisely the wrong-altitude
            # grid this module exists to prevent, while the printed line read
            # like a refusal. Drop it: no map at all is honest, and
            # ObstacleClearance is INERT without one (guardrail/models.py) rather
            # than wrong.
            print("[occ] *** no usable map for this altitude band, so "
                  "ObstacleClearance will be INERT for this flight. It is "
                  "better to fly with no obstacle map than with one built for "
                  "an altitude the aircraft is not at.")
            cmap = None

    smap = None
    if policy.by_type(ObstacleClearance) and cmap is not None:
        smap = {"occ": cmap["occ"], "res": cmap["res"],
                "ox": cmap["ox"], "oy": cmap["oy"]}
    shield = Shield(policy, lookahead_s=3.0, dt=0.5, obstacle_map=smap)
    clr = policy.by_type(ObstacleClearance)

    # Where the roads are, which is a different question from where the
    # obstacles are - see demo/build_street_mask.py. The detour search needs
    # both: outside the fence, clear of obstacles, AND on a road.
    street = None
    _sm = Path(args.citymap).parent / "street.npz"
    if _sm.is_file():
        from build_street_mask import load_street
        street = load_street(_sm)
        print(f"[fence] street mask loaded: {int(street['street'].sum())} road cells")
    else:
        print(f"[fence] WARNING no street mask at {_sm}; detours will be judged "
              f"on obstacles alone, which lets slide() route off-road. "
              f"Build it with: python demo/build_street_mask.py")

    fence = FenceGuard(policy, brake_m=args.fence_brake,
                       stand_off_m=args.fence_standoff, obstacle_map=smap,
                       min_clearance_m=(clr[0].min_clearance_m if clr else 5.0),
                       street_mask=street)

    audit = AuditLogger(out / "audit.jsonl", policy)   # the POLICY, so a hot-applied rule restamps the hash

    # On-screen policy indicator (demo/policy_hud.py): the rules in force, the
    # aircraft's distance to each limit and a banner whenever the Shield acts -
    # the audit trail above, made visible in the demo video.
    pind = None
    pind_error = None
    if not args.no_policy_hud:
        from policy_hud import PolicyIndicator, render_base_map
        _bm = smap if smap is not None else (
            {"occ": cmap["occ"], "res": cmap["res"], "ox": cmap["ox"], "oy": cmap["oy"]}
            if cmap is not None else None)
        pind = PolicyIndicator(policy, base_map=render_base_map(_bm, street),
                               fence_near_m=args.fence_brake)
    if fence.polys:
        print(f"[fence] {len(fence.polys)} no-fly zone(s) known to the controller: "
              f"brake from {args.fence_brake:.0f} m, hold at {args.fence_standoff:.0f} m")

    print(f"[policy]  {policy.policy_id} {policy.policy_hash}")
    print(f"[policy]  from {policy_source['kind']} {policy_source['path']} - "
          f"signature {policy_source['signature']}")
    print(f"[follow]  query = {args.object!r}")
    print("[follow]  the ONLY steering input is where the detector puts the box; "
          "no target coordinates reach the controller")

    obs = SemanticObs()
    rows, traj, n_touched = [], [], 0
    car = None
    traffic = None
    env_car = None
    people = None
    parked = None
    n_absent = 0
    # Ticks whose detection the presence verdict called ABSENT and which were
    # therefore NOT allowed to steer (see --presence-gates-control).
    n_presence_blocked = 0
    # Ticks whose horizontal command followed the subject's trail rather than
    # the nose (see --trail-follow), and ticks flown off the obstacle map, where
    # the clearance rule is blind (Shield.off_map).
    n_trail_ticks = 0
    n_trail_lookout = 0         # coast/search ticks steered by the lookout
    n_far_approach = 0          # search ticks steered toward a far lead (Reacquirer)
    n_trail_against = 0         # trail directions refused: away from the subject
    n_off_map = 0
    # Why the presence gate refused each box it refused (by rule), and how often
    # the ground-contact check was waived for a box on the estimated subject.
    # The gate's reason was discarded until 2026-09-24, so a gate rejecting the
    # real car read exactly like a detector that had missed it.
    block_reasons = {}
    n_ground_skipped = 0        # boxes that agreed with the estimate (waived)
    n_ground_rescued = 0        # ...of those, inferences the check would have refused
    have_depth = False
    t_ground_ok = 0.0           # last estimator update from a ground-checked box
    # IDENTITY AND RE-ACQUISITION (--identity; 2026-09-29). See Reacquirer and
    # TargetLock.select_strict for why the old path could end TARGET LOCKED on
    # a red pedestrian signal 110 m from the car.
    id_cfg = None
    reacq = None                # a Reacquirer while the subject is being re-found
    last_reacq_seq = None
    lock_events = []            # lapses and re-acquisitions, with where and why
    reacq_refused = {}          # every Reacquirer's refusals, over the flight
    t_meas_last = None          # identity: last accepted measurement / commit
    t_meas_last_cap = None      # ...its capture time, for an anchor without an estimator
    subject_committed = False   # identity: a subject is held (gate or reacquire)
    v_prior = None              # the speed measured before the last re-acquisition
    pending_seed = None         # the start gate's pick, until the estimator exists
    t_ok_last = 0.0             # last estimator update from an identity-OK box
    n_soft_fed = 0              # SOFT boxes that continued the track
    n_soft_refused = 0          # ...and those refused (no fresh track to continue)
    det_latency = []            # capture -> consumed, ms, per fresh detection
    planner = None              # demo/search.py, after coast + search
    plan_row = None
    last_heading = None         # the subject's measured direction of travel
    landing_info = {"enabled": False}
    # Every contact the SIMULATOR reported, from the robot's collision_info
    # topic. Nothing listened to it before 2026-09-24, so a flight could scrape
    # a pole and the artefact would say nothing - a zero nobody measured. The
    # plugin logged every contact to the simulator's own log; the client did not.
    collisions = []
    collisions_subscribed = False
    t_connect = time.time()
    t0 = None                   # the mission clock; set once the gate opens
    det_n_at_t0 = 0             # detector inferences before t0 (see det_hz)
    t_mission_end = None        # ...and when its loop ended
    start_gate = {"enabled": False}
    stage_ms = {"kin": [], "truth": [], "shield": [], "cmd": [], "work": []}
    # Initialised at function scope, not inside the `async with`. When the sim
    # fails to connect the block raises before its own initialisers run, and the
    # metrics section then dies with UnboundLocalError - which buries the real
    # error under a confusing one. A failed flight should report zeros.
    tick = 0
    # The same for everything the metrics and the teardown read that is only
    # built once connected (review, 2026-09-24: a failed connect raised
    # NameError on `estimator` in the metrics and on `recorder` in `finally`).
    estimator = None
    lock = None
    trail = None
    trail_stop_short = 0.0
    recorder = None
    view_dir = None
    start_heading_err_deg = None
    nfz_hold_ticks = 0
    n_fence_aim = 0          # ticks the forward aim was moved out of a zone's band
    guard_hold_ticks = 0        # any hold: fence OR building

    presence = PresenceMonitor(args.object, args.colour_min)
    rng_f = None            # low-passed range, for the orbit radial term
    orbit_fwd_prev = 0.0
    client = ProjectAirSimClient()
    client.connect()
    grounder = None
    try:
        world = World(client, SCENE, delay_after_load_sec=2,
                      sim_config_path=SIM_CONFIG_DIR)

        # MAKE THE SIMULATOR WINDOW SHOW THE SIMULATION.
        #
        # The reported symptom was "window berukuran kecil dan tidak menampilkan
        # simulasi" - a small window, blank white, no scene. It is not a
        # rendering fault and it is not the depth captures being expensive,
        # which is what I wrongly blamed first.
        #
        # `SwitchStreamingView` binds the game window to "the next available
        # camera with streaming-enabled=true". This robot declares exactly two,
        # and the FIRST is the FrontCamera DEPTH stream at 400x225. So the main
        # view is a depth image: the window resizes itself to 400x225 and shows
        # near-white, because everything in the scene is far away and depth maps
        # to white. Exactly the reported symptom, and it only appears once a
        # client loads the scene - an idle simulator still shows the level,
        # which is why this looked fixed when it was not.
        #
        # The depth stream cannot simply be turned off: `streaming-enabled: true`
        # on it is load-bearing for the range signal (with it false the depth
        # arrives as all zeros, silently). So advance the view instead. One
        # switch lands on the Chase camera, which is the third-person view of
        # the aircraft - the thing worth watching anyway.
        for _ in range(max(0, args.view_switch)):
            try:
                world.switch_streaming_view()
            except Exception as e:
                print(f"[view] could not switch the main view "
                      f"({type(e).__name__}: {e})")
                break
        if args.view_switch:
            print(f"[view] simulator window switched {args.view_switch}x -> "
                  f"showing the Chase camera, not the depth stream")

        drone = Drone(client, world, "Drone1")
        t_connect = time.time()

        def _on_collision(_, m):
            # Only real contacts. `t` is seconds since connect; the metrics
            # split it at the mission clock's t0 later, so take-off scrapes on
            # the launch pad are not counted against the follow.
            if isinstance(m, dict) and m.get("has_collided", True):
                collisions.append({
                    "t_conn": round(time.time() - t_connect, 2),
                    "object": m.get("object_name"),
                    "penetration_m": m.get("penetration_depth"),
                    "impact": m.get("impact_point"),
                })
        try:
            client.subscribe(drone.robot_info["collision_info"], _on_collision)
            collisions_subscribed = True
            print("[collide] subscribed to the robot's collision_info topic")
        except Exception as exc:
            print(f"[collide] *** could not subscribe to collision_info "
                  f"({type(exc).__name__}: {exc}) - contacts will NOT be counted")
        client.subscribe(drone.sensors["FrontCamera"]["scene_camera"],
                         lambda _, m: obs.put_front(m))
        client.subscribe(drone.sensors["DownCamera"]["scene_camera"],
                         lambda _, m: obs.put_down(m))
        # Range signal. Optional by design: an older robot config has no depth
        # capture on FrontCamera, and every mission except the orbit works
        # perfectly well on apparent width, so a missing stream degrades to the
        # width servo rather than failing the flight.
        have_depth = False
        try:
            client.subscribe(drone.sensors["FrontCamera"]["depth_camera"],
                             lambda _, m: obs.put_depth(m))
            have_depth = True
            print("[range] FrontCamera depth stream subscribed")
        except Exception as exc:
            print(f"[range] no depth stream ({type(exc).__name__}) — "
                  "falling back to apparent box width")
        view_dir = None
        recorder = None
        if args.save_view:
            try:
                client.subscribe(drone.sensors["Chase"]["scene_camera"],
                                 lambda _, m: obs.put_chase(m))
                view_dir = out / "view"
                (view_dir / "tps").mkdir(parents=True, exist_ok=True)
                (view_dir / "fpv").mkdir(parents=True, exist_ok=True)
                recorder = FrameRecorder(obs, view_dir, annotate,
                                         hz=args.record_hz,
                                         height=args.record_height,
                                         live_view=args.live_view,
                                         live_height=args.live_height)
                recorder.start()
                print(f"[view] recording {args.record_hz:.0f} Hz at "
                      f"{args.record_height}p on its own thread -> {view_dir}")
                if args.live_view:
                    print(f"[view] live window open at {args.live_height}p "
                          f"(drone view | chase view)")
            except Exception as exc:
                print(f"[view] no Chase camera in this config "
                      f"({type(exc).__name__}: {exc})")

        if not args.no_car:
            # Any FIXED route gets the two stops, not just --straight. The
            # stop-and-go is the clearest evidence the aircraft is tracking the
            # car rather than flying down the same street: a follower has to
            # stop too, and pull away when the car does. Gating it on --straight
            # alone silently dropped it the moment --route was introduced.
            stops = ([(0.30, args.car_stop_s), (0.62, args.car_stop_s)]
                     if (args.route or args.straight) and args.car_stop_s > 0
                     else None)
            if args.traffic > 0:
                # Several vehicles, same mesh, only the target painted. `car`
                # stays a MovingCar (the fleet's target), so every downstream
                # use — ground-truth separation, the HUD, the metrics — is
                # unchanged and the traffic is purely additive.
                traffic = city_traffic.Traffic(world, fleet=city_traffic.default_fleet(
                    n_background=args.traffic, speed=args.car_speed,
                    target_stops=stops, bg_every=args.traffic_every,
                    mode=args.traffic_mode,
                    models=city_traffic.glb_dir(args.glb_dir),
                    route_name=(args.route or "straight")))
                traffic.spawn()
                car = traffic.target
            else:
                # The lone subject gets the same upgrade as the fleet when the
                # models are installed. It is the same detector and the same
                # colour gate, so the measured gain applies here too: the glTF
                # taxi scores 0.108 as "a yellow car" with colour_match 0.317,
                # against 0.047 / 0.119 for the orange buggy at the same pose.
                spec = None
                d = city_traffic.glb_dir(args.glb_dir)
                if d is not None and (d / city_traffic.GLB_TARGET[0]).is_file():
                    spec = moving_car.CarSpec()
                    spec.glb_path = str(d / city_traffic.GLB_TARGET[0])
                    spec.materials = []
                    spec.desc_match = f"a {city_traffic.GLB_TARGET[1]} car"
                    spec.desc_mismatch = "a red car"
                if args.park_at:
                    px, py = [float(v) for v in args.park_at.split(",")]
                    # A one-metre route driven once: the car reaches the far end
                    # almost immediately and stays there. pose_at clamps, and
                    # update() now skips the no-op teleport, so a parked subject
                    # costs nothing per tick.
                    car = moving_car.MovingCar(
                        world, speed_mps=0.5, route=[(px, py), (px, py + 1.0)],
                        one_shot=True, phase_s=0.0, spec=spec)
                else:
                    # --route wins; --straight is kept so every existing
                    # command line and every recorded flight keeps its meaning.
                    route_name = args.route or ("straight" if args.straight else None)
                    fixed = moving_car.ROUTES.get(route_name) if route_name else None
                    car = moving_car.MovingCar(
                        world, speed_mps=args.car_speed,
                        route=fixed,
                        one_shot=fixed is not None,
                        phase_s=(0.0 if fixed is not None else 10.0),
                        stops=stops, spec=spec)
                    if route_name:
                        print(f"[car] route '{route_name}': "
                              f"{car.path.total:.0f} m, {car.lap_time:.0f} s to drive")

                # Prefer the environment actor: the simulator interpolates the
                # whole route at render rate, instead of the client teleporting
                # once per control tick. Teleporting sampled a continuous motion
                # model at 8.69 Hz in 54 cm steps against a 15.57 Hz capture, so
                # 44 % of frames during motion repeated a position - the whole of
                # the "choppy" complaint.
                #
                # Falls back to spawning if the actor is not in the scene, so a
                # scene config without it still flies.
                if args.car_mode != "spawn":
                    try:
                        env_car = EnvActor(client, world, ENV_CAR_NAME)
                        info = car_trajectory.upload(world, car,
                                                     seconds=args.max_s + 5.0)
                        car.actual_name = None      # nothing to teleport or destroy
                        print(f"[car] driven by the simulator: {info['samples']} "
                              f"samples over {info['seconds']:.0f}s at "
                              f"{info['sample_hz']:.0f} Hz, one upload")
                    except Exception as exc:
                        env_car = None
                        print(f"[car] env actor {ENV_CAR_NAME!r} unavailable "
                              f"({type(exc).__name__}: {exc}); falling back to "
                              f"per-tick teleport, which will look choppier")
                        car.spawn()
                else:
                    car.spawn()

        # TRUTH THAT THE LEVEL OWNS.
        #
        # CityLife_Day walks its own 16 pedestrians. Nothing here placed them,
        # so nothing here knows where they are, and the whole pedestrian phase
        # would log `truth.pts == []` - scored as unscorable, reported as
        # nothing. `level_actors` asks the simulator instead, which is better
        # evidence than the commanded position the client keeps for its own
        # figures. It resolves by TAG, because a placed Blueprint is named
        # after its class (`BP_CityPed_M1_C_1`) and `Ped_07` is only an editor
        # label, which a -game build does not have.
        if args.level_peds > 0:
            import level_actors
            people = level_actors.LevelActors(
                world, level_actors.names(args.level_ped_prefix, args.level_peds),
                kind="pedestrian", min_period_s=args.level_truth_period)
            people.resolve()          # refuses the flight if none resolve

        # One car the level drives, as THE subject. Its truth is a single
        # point, so the car mission is scored against one instance - a street
        # full of other cars cannot saturate the null the way forty
        # pedestrians did (docs/FINDING-crowd-pedestrians-and-traffic.md).
        if args.level_car:
            import level_actors
            car = level_actors.LevelCar(world, args.level_car, desc=args.object,
                                        min_period_s=args.level_truth_period)
            car.spawn()               # refuses the flight if the tag is absent

        # Scenery. Spawned once and, for all but a couple of them, never touched
        # again - a standing figure costs no per-tick RPC, so it cannot take
        # anything from the detector. Default 0 so every recorded flight and every
        # existing command line keeps its meaning.
        # Both kinds of scenery share the kerb search and the building
        # mask, so they are placed together - `--parked` alone must work
        # without `--pedestrians`, which gating on the latter would break.
        if (args.pedestrians > 0 or args.parked > 0) and street is not None:
            import numpy as _np
            _b = Path(args.citymap).parent / "occ_day_highband_15to55.npz"
            _bld = _np.load(_b)["occ"] if _b.is_file() else None
            if _bld is None:
                print("[people] no building mask; cannot tell a pavement from a road, "
                      "so nobody is placed")
            else:
                _route = moving_car.ROUTES.get(args.route or "turn") or []
                _samp = [(x, y) for x, y in _route]
                if args.pedestrians > 0:
                    people = people_mod.Pedestrians(
                        world, street, _bld, count=args.pedestrians,
                        walking=args.pedestrians_walking, seed=args.seed,
                        people_dir=args.people_dir, avoid=_samp)
                    people.spawn()
                    # The start-up refusal checks the FLAG; this checks the
                    # outcome. --pedestrians 12 with no baked GLB pack, no
                    # building mask, or no clear pavement places nobody, and the
                    # flight would then log truth.pts == [] on every tick of the
                    # pedestrian phase - exactly what the refusal exists to
                    # prevent, reached by a route it cannot see.
                    if _ped_phrases and not people.figures:
                        raise SystemExit(
                            f"{_ped_phrases[0]!r} selects the pedestrian class "
                            f"and --pedestrians is {args.pedestrians}, but "
                            f"spawning placed NOBODY, so there is no ground "
                            f"truth for that phase. See the [people] line above "
                            f"for why.")

                # Parked vehicles, allocated from the SAME kerb after the
                # pedestrians have taken theirs. Passing their positions is
                # what stops a car being spawned on top of a person - two
                # objects in one cell reads as a bug, not as a street.
                if args.parked > 0:
                    import parked_cars as parked_mod
                    parked = parked_mod.ParkedCars(
                        world, street, _bld, count=args.parked, seed=args.seed,
                        models_dir=args.glb_dir, route=_samp,
                        circuit=city_traffic.circuit(),
                        taken=([(f.x, f.y) for f in people.figures]
                               if people is not None else []))
                    parked.spawn()


            # DOES THE QUESTION MATCH THE SUBJECT?
            #
            # This exact mismatch is what made detection worse rather than
            # better once already: the target mesh was changed to SKM_SportsCar
            # for its stronger noun score, M_Orange renders WHITE on that mesh,
            # and the query still said "orange". Nothing failed, nothing warned,
            # and the run produced a plausible number for a scene where the
            # colour gate could never fire. Never silent again.
            want = colour_word(args.object)
            have = colour_word(car.spec.desc_match)
            if want and have and want != have:
                print(f"[follow] *** the query asks for {want.upper()} but the "
                      f"subject is {have.upper()} ({car.spec.desc_match}). The "
                      f"colour gate cannot pass. Use --object "
                      f"{car.spec.desc_match!r} ***")
            elif want and not have:
                print(f"[follow] note: query asks for {want.upper()}; the "
                      f"subject makes no colour claim ({car.spec.desc_match!r})")

            # let the actor settle before the first teleport; it is briefly
            # not movable straight after spawning
            for _ in range(10):
                await asyncio.sleep(0.4)
                before = car.pos
                car.update(0.5)
                if car.pos != before or getattr(car, "_fail_streak", 0) == 0:
                    break
            car.update(0.0)

        drone.enable_api_control()
        drone.arm()
        await (await drone.takeoff_async())
        for _ in range(400):
            kin = drone.get_ground_truth_kinematics()
            if -kin["pose"]["position"]["z"] >= args.cruise_alt - 0.5:
                break
            await drone.move_by_velocity_async(0.0, 0.0, -2.0, duration=0.2)
            await asyncio.sleep(0.1)

        # POINT DOWN THE STREET BEFORE HANDING OVER, AND PROVE IT WORKED.
        #
        # This was silently broken, and it was corrupting every flight
        # comparison. The old version commanded
        # `move_by_velocity_async(0,0,0, yaw_is_rate=False, yaw=psi0)`, which
        # does not turn the aircraft, and then simply carried on. Measured
        # across two runs of the same demo:
        #
        #     run A   psi0 =  60.3 deg, car at bearing  83.2 -> +22.9 deg, in frame
        #     run B   psi0 = 135.2 deg, car at bearing  83.2 -> -52.1 deg, OUTSIDE
        #                                                       the 45 deg half-FOV
        #
        # Run B never saw the car at all for 11.5 s, by which time it had driven
        # 23 m away, and the flight never recovered: hit rate 0.331 against
        # 0.740, mean separation 54.8 m against 17.5 m. That is a bigger effect
        # than any change actually being tested, so without this fix an A/B on
        # this demo measures the takeoff lottery, not the change.
        #
        # The heading is now a SCENE CONSTANT - the direction of the road - and
        # not derived from the car. The old code read `car.pos`, which is target
        # ground truth; the demo's whole claim is that the only steering input
        # is where the detector puts the box, so pointing the nose using the
        # answer was worth removing on its own.
        psi0 = math.radians(args.start_heading_deg)
        await (await drone.rotate_to_yaw_async(yaw=psi0))
        await asyncio.sleep(0.5)
        got = quat_yaw(drone.get_ground_truth_kinematics()["pose"]["orientation"])
        err = abs((psi0 - got + math.pi) % (2 * math.pi) - math.pi)
        start_heading_err_deg = math.degrees(err)
        print(f"[flight] start heading {math.degrees(got):.1f} deg "
              f"(asked {args.start_heading_deg:.1f})")
        if err > math.radians(10):
            print(f"[flight] *** the aircraft did not take up the start heading "
                  f"({math.degrees(err):.0f} deg off). Whether the subject is in "
                  f"the first frames is now luck, and this run is not comparable "
                  f"with others ***")

        lock = TargetLock(gate_frac=args.lock_gate) if args.lock_target else None
        if args.identity:
            # Each candidate judged physically (demo/identity.py) with the pose
            # and depth of its own frame; the lock made strict. It needs a lock.
            import identity as identity_mod
            _th = identity_mod.load_thresholds(args.identity_thresholds or None)
            _sp = Path(args.citymap).parent / "street.npz"
            _sd = identity_mod.StreetDistance.load(_sp) if _sp.is_file() else None
            if _sd is None:
                print(f"[identity] *** no street mask at {_sp}: the off-street "
                      f"rules cannot fire")
            id_cfg = {"thresholds": _th, "street": _sd, "hfov": CAMERA_HFOV_DEG,
                      "hash": identity_mod.thresholds_hash(_th),
                      "path": str(args.identity_thresholds or
                                  identity_mod.DEFAULT_THRESHOLDS)}
            if lock is None:
                lock = TargetLock(gate_frac=args.lock_gate)
            print(f"[identity] ON - thresholds {id_cfg['hash']}, strict lock, "
                  f"re-acquire after {args.lapse_s:.1f} s unseen")
        if args.search_planner:
            try:
                import search as search_mod
                planner = search_mod.SearchPlanner(
                    search_mod.junctions_from_routes(),
                    Path(args.citymap).parent / "street.npz")
                print(f"[search] junction planner ON "
                      f"({len(planner.junctions)} junctions)")
            except Exception as exc:
                planner = None
                print(f"[search] *** junction planner unavailable "
                      f"({type(exc).__name__}: {exc}); scanning in place")
        grounder = Grounder(obs, args.object, thresh=args.det_thresh,
                            log_path=out / "detections.jsonl",
                            colour_min=args.colour_min, lock=lock,
                            colour_mask=args.colour_mask,
                            legacy_sat=args.colour_legacy_sat,
                            identity=id_cfg)
        grounder.start()
        for _ in range(600):                      # wait for the detector to load
            if grounder.latest()["seq"] > 0:
                break
            await asyncio.sleep(0.5)
        print(f"[flight] cruise {args.cruise_alt:.0f} m — following {args.object!r}")

        limiter = RateLimiter(args.dv_h, args.dv_z)

        # Start the car with the MISSION clock, not at scene setup. The
        # trajectory plays from the instant it is bound, so binding it earlier
        # had the car driving through arming and the climb to cruise.
        if env_car is not None:
            car_trajectory.start(env_car)
            print("[car] trajectory playback started with the mission clock")

        # WAIT FOR THE SUBJECT, ON THE DETECTOR'S WORD.
        #
        # A level that drives its own traffic does not wait for the aircraft:
        # the red car is somewhere on a 684 m loop when the flight begins. The
        # mission clock therefore starts when the DETECTOR has produced N
        # consecutive boxes that pass the colour gate and the presence checks -
        # the same evidence the controller steers by. Ground truth is read
        # afterwards only to REPORT whether that was the subject, never to
        # decide when to go.
        if args.start_when_seen > 0:
            t_wait0 = time.time()
            acq = Acquirer(args.start_when_seen, args.object, args.colour_min,
                           gate_frac=args.lock_gate,
                           max_range_m=args.start_max_range_m,
                           width_m=args.object_width_m)
            # With identity the START is decided in the world, like a
            # re-acquisition with no anchor: OK boxes, one object by its map
            # point, within the start range, and SEEN MOVING - the red
            # fire-hydrant sign by the launch point is none of the physical
            # rules' business and all of this one's. The pixel Acquirer still
            # runs, for its counters.
            acq_w = None
            _ic = {}
            if id_cfg is not None:
                acq_w = Reacquirer(need=args.start_when_seen,
                                   max_range_m=(args.start_max_range_m
                                                if args.start_max_range_m > 0
                                                else args.reacq_max_range_m))
                acq_w.start(None)
            grounder.begin_acquire()
            last_gseq, first_ok, reached = -1, None, False
            # A frame every 5 s, and at the first plausible sighting, so a gate
            # that fails can be looked at rather than argued about.
            gate_dir = out / "gate"
            gate_dir.mkdir(parents=True, exist_ok=True)
            n_snaps, next_snap = 0, 0.0
            # Where the subject really was while we waited, every 2 s: REPORTED
            # only (the decision never reads it), so a timed-out gate can say
            # whether the car drove past the camera or never came.
            truth_trace, next_truth = [], 0.0
            while time.time() - t_wait0 < args.start_timeout_s:
                _tw = time.time()
                kin_w = drone.get_ground_truth_kinematics()
                pw = kin_w["pose"]["position"]
                _qw = kin_w["pose"]["orientation"]
                obs.put_pose_full((_tw + time.time()) / 2.0, pw["x"], pw["y"], -pw["z"],
                                  quat_yaw(_qw), quat_pitch(_qw), quat_roll(_qw),
                                  stamp=kin_w.get("time_stamp"))
                gw = grounder.latest()
                if gw["seq"] != last_gseq:
                    last_gseq = gw["seq"]
                    fresh = gw["t"] and (time.time() - gw["t"]) < 1.0
                    dep_w = obs.get_depth() if have_depth else None
                    had = acq.n_plausible
                    if fresh and id_cfg is not None:
                        # The range of each candidate's OWN frame, and its tier:
                        # only an identity-OK box may start the mission.
                        _fr = {id(c): (f or {}).get("r")
                               for c, f in zip(gw["cands"], gw["cand_feats"])}
                        acq.step(gw["cands"], lambda c: _fr.get(id(c)),
                                 ground=None, tiers=gw["cand_tiers"])
                        _ic = gw.get("inf_cap") or {}
                        reached = acq_w.step(gw["cands"], gw["cand_tiers"],
                                             gw["cand_feats"],
                                             _ic.get("t_capture") or _ic.get("t_cap")
                                             or time.time())
                    elif fresh:
                        reached = acq.step(
                            gw["cands"],
                            lambda c: (range_from_depth(dep_w, c)
                                       if dep_w is not None else None),
                            ground=(-pw["z"], quat_pitch(kin_w["pose"]["orientation"])))
                    if acq.n_plausible > had and first_ok is None:
                        first_ok = time.time() - t_wait0
                    now_w = time.time() - t_wait0
                    if now_w >= next_snap or (acq.n_plausible == 1 and had == 0):
                        img_w = obs.get_front_native()
                        if img_w is not None:
                            try:
                                img_w.save(gate_dir / f"gate_{now_w:06.1f}s.jpg", quality=85)
                                n_snaps += 1
                            except Exception:
                                pass
                        next_snap = now_w + 5.0
                    if reached:
                        break
                if (car is not None and hasattr(car, "refresh")
                        and time.time() - t_wait0 >= next_truth):
                    next_truth = time.time() - t_wait0 + 2.0
                    try:
                        car.refresh()
                        yaw_t = quat_yaw(kin_w["pose"]["orientation"])
                        dx, dy = car.pos[0] - pw["x"], car.pos[1] - pw["y"]
                        brg = math.degrees(math.atan2(math.sin(math.atan2(dy, dx) - yaw_t),
                                                      math.cos(math.atan2(dy, dx) - yaw_t)))
                        truth_trace.append([round(time.time() - t_wait0, 1),
                                            round(math.hypot(dx, dy), 1), round(brg, 1)])
                    except Exception:
                        pass
                vz_w = float(np.clip((args.cruise_alt + pw["z"]) * args.alt_gain,
                                     -args.climb_max, args.climb_max))
                await drone.move_by_velocity_async(0.0, 0.0, -vz_w, duration=0.3,
                                                   yaw_is_rate=True, yaw=0.0)
                await asyncio.sleep(0.1)
            waited = time.time() - t_wait0
            _start_pick = (acq_w.pick if acq_w is not None else acq.pick)
            grounder.commit(_start_pick if reached else None,
                            yaw=((_ic.get("pose") or {}).get("yaw") if acq_w is not None
                                 else None))
            if acq_w is not None and reached:
                # Seeded into the estimator once it exists (below): committing
                # the lock alone left a flight whose car hid for 2 s before the
                # first OK box with no lock, no estimate and no re-acquisition
                # for the rest of the mission (review, 2026-09-29).
                pending_seed = (acq_w.pick_feat, dict(_ic))
            start_gate = {"enabled": True, "needed": args.start_when_seen,
                          "reached": reached,
                          "wait_s": round(waited, 1),
                          "first_candidate_s": (None if first_ok is None
                                                else round(first_ok, 1)),
                          "inferences_judged": acq.n,
                          "inferences_with_plausible": acq.n_plausible,
                          "candidates_judged": acq.n_candidates,
                          "best_streak": acq.best_streak,
                          "max_range_m": args.start_max_range_m,
                          "plausible_but_too_far": acq.n_too_far,
                          "last_rejections": acq.last_reasons[:3],
                          "snapshots": n_snaps,
                          "identity_gate": (None if acq_w is None else
                                            {"refused": dict(acq_w.refused),
                                             "best_streak": acq_w.best_streak,
                                             "static_places": [[round(v, 1) for v in sp[:2]]
                                                               for sp in acq_w.static],
                                             "pick_P": (None if acq_w.pick_feat is None
                                                        else [round(v, 1) for v in
                                                              acq_w.pick_feat["P"][:2]])}),
                          # [t_s, range_m, bearing_deg (+ right)]; in view when
                          # |bearing| <= 45
                          "truth_trace": truth_trace}
            # Was it the subject? Truth answers that AFTER the decision.
            if car is not None and hasattr(car, "refresh"):
                car.refresh()
                kin_w = drone.get_ground_truth_kinematics()
                pw = kin_w["pose"]["position"]
                dw = _start_pick if reached else grounder.latest()["det"]
                if dw is not None:
                    tcx, tdeg = track_truth.project_target_cx(
                        pw["x"], pw["y"], quat_yaw(kin_w["pose"]["orientation"]),
                        car.pos[0], car.pos[1], int(dw[5]), CAMERA_HFOV_DEG)
                    start_gate["subject_px_err"] = round(abs(float(dw[0]) - tcx), 1)
                    start_gate["subject_in_shot"] = abs(tdeg) <= CAMERA_HFOV_DEG / 2
                    start_gate["subject_range_m"] = round(math.hypot(
                        car.pos[0] - pw["x"], car.pos[1] - pw["y"]), 1)
            print(f"[start] {'subject acquired' if start_gate['reached'] else '*** TIMED OUT, no subject ***'}"
                  f" after {waited:.0f} s: {start_gate}")

        t0, last_seen = time.time(), 0.0
        # Detector inferences made BEFORE the mission clock started (the start
        # gate can wait minutes). det_hz must not count them: it did, dividing
        # them by mission time only, and reported 6.9-7.5 Hz for flights whose
        # detector ran at 3.5-3.9 Hz during the mission (found 2026-09-29).
        _g0 = grounder.latest()
        det_n_at_t0 = _g0["n_seen"] + _g0["n_miss"]
        tick_deadline = None        # the pacing's running deadline, see below
        # Where the subject was last MEASURED to be, and how fast it was then
        # measured to move (displacement of accepted positions over 2 s).
        acc_hist = []
        last_acc = None
        last_meas_speed = None
        last_bearing, brg_rate, last_cmd, mode = 0.0, 0.0, (0.0, 0.0), "hold"
        # The presence verdict is computed further down the tick, so the search
        # logic uses the PREVIOUS tick's. That is the honest signal anyway: it is
        # what the system last knew about whether the object is really out there.
        # (start_heading_err_deg was reset to None here, AFTER the rotate had
        # measured it, so every metrics.json carried null - found in review,
        # 2026-09-24. The function-scope initialiser covers an early abort.)
        last_verdict = "UNSURE"
        # Servo on an ESTIMATE of the target rather than the latest box. See
        # demo/target_state.py for the measurements that forced this.
        estimator = (TargetState(v_max=SUBJECT_VMAX_MPS.get(subject_class))
                     if args.target_estimator else None)
        # WHERE THE SUBJECT DROVE (--trail-follow; demo/trail.py).
        #
        # Built from the estimator's own position after each accepted update,
        # so it carries no ground truth. Needs the estimator: the box servo has
        # no position to lay a breadcrumb at.
        trail = (trail_mod.Trail(spacing_m=1.5, max_len_m=120.0)
                 if args.trail_follow and estimator is not None else None)
        if args.trail_follow and trail is None:
            print("[trail] *** --trail-follow needs the target estimator; "
                  "flying along the nose")

        def _take_subject(pf, ic):
            """A subject committed by the start gate or a re-acquisition: the
            estimator seeded from the committing sighting at its capture pose,
            and the flight's own memory of the subject moved to it - the last
            measured position, its history, speed, heading and trail. Left on
            the pre-loss track, a second loss soon after anchored the reach on
            the old place and refused the car where it really was (review,
            2026-09-29). Returns (seeded, P)."""
            nonlocal last_acc, last_meas_speed, last_heading, t_ok_last
            nonlocal t_meas_last, subject_committed, v_prior, t_meas_last_cap
            cp = (ic or {}).get("pose")
            t_c = (ic or {}).get("t_capture") or (ic or {}).get("t_cap") or time.time()
            seeded = False
            if estimator is not None:
                estimator.reset()
                if (cp is not None and pf.get("rng_h") is not None
                        and pf.get("bearing") is not None):
                    b0 = pf["bearing"] - cp["yaw"]
                    b0 = math.atan2(math.sin(b0), math.cos(b0))
                    seeded = bool(estimator.update(t_c, cp["x"], cp["y"], cp["yaw"],
                                                   b0, float(pf["rng_h"])))
            if seeded:
                P0 = (float(estimator._xu[0]), float(estimator._xu[1]))
            elif pf.get("P") is not None:
                P0 = (float(pf["P"][0]), float(pf["P"][1]))
            else:
                P0 = None
            if last_meas_speed is not None:
                v_prior = last_meas_speed
            last_meas_speed = None          # a streak's ~3 m jitter is no speed
            last_heading = None
            acc_hist.clear()
            if P0 is not None:
                last_acc = P0
                acc_hist.append((time.time(), P0[0], P0[1]))
                if trail is not None:       # the breadcrumbs to here are unknown
                    trail.clear()
                    trail.add(P0[0], P0[1])
            t_ok_last = time.time()
            t_meas_last = time.time()
            t_meas_last_cap = t_c
            subject_committed = True
            return seeded, P0

        def _retire(r, why):
            """Fold a finished Reacquirer's refusals into the flight's total and
            return them for its event record."""
            for k_, v_ in r.refused.items():
                reacq_refused[k_] = reacq_refused.get(k_, 0) + v_
            return {"refused": dict(r.refused), "n": r.n, "best_streak": r.best_streak,
                    "static_places": [[round(v, 1) for v in q[:2]] for q in r.static],
                    "ended": why}

        if pending_seed is not None:
            _seeded0, _P0 = _take_subject(*pending_seed)
            lock_events.append({"t": 0.0, "event": "acquired", "seeded": bool(_seeded0),
                                "P": None if _P0 is None else [round(v, 1) for v in _P0]})

        def _trail_stop_short():
            # How far short of the last sighting a coast approach stops: the
            # policy's own stand-off for this class of subject (0 if none binds).
            return max((so.min_range_m for so in policy.by_type(SubjectStandoff)
                        if so.binds(subject_class)), default=0.0)
        trail_stop_short = _trail_stop_short()
        def _stand_off():
            """The range the servo aims for, from the CURRENT subject width.

            A function rather than a value because the subject can change
            mid-flight. It was a value until 2026-09-09, computed once before
            the loop - so a retarget updated `args.object_width_m` and left the
            set-point derived from it, and the aircraft spent the whole
            pedestrian half of the demo holding the CAR's 15.83 m.

            That is the actual reason the 10 m ring never fired, and it was
            reported for two days as a detector weakness. Measured on
            demo/out/retarget_demo2: 54 of 319 post-retarget ticks sat within a
            metre of 15.83 m, the closest served range was 14.68 m, and while
            in that band the pilot was commanding -1.18 m/s. The aircraft was
            holding station exactly as instructed.
            """
            return (args.want_range if args.want_range > 0
                    else want_range_from_width(args.want_width,
                                               args.object_width_m))

        want_range = _stand_off()

        # --want-width is an ANGULAR target, so the stand-off it asks for scales
        # with the subject's real width. 0.16 was tuned against a 4 m car and
        # gives 15.8 m; the same flag against a 0.5 m pedestrian asks for 2.0 m,
        # which is not a small adjustment but a different mission.
        #
        # FrontCamera is pitched 20 deg down with a 29.4 deg vertical half-FOV,
        # so it sees the ground from about 0.86 x altitude outward. Inside that
        # the subject is under the aircraft and out of frame, and the tracker
        # would be closing on something it can no longer see.
        blind_m = 0.86 * args.cruise_alt

        def _warn_blind():
            if want_range >= blind_m:
                return
            print(f"[flight] WARNING stand-off {want_range:.1f} m is inside the "
                  f"camera's near blind spot ({blind_m:.1f} m at {args.cruise_alt:.0f} m "
                  f"altitude): the subject leaves frame before the aircraft gets "
                  f"there. --want-width {args.want_width} was calibrated for a 4 m "
                  f"car; for a {args.object_width_m:.2f} m subject set --want-range "
                  f"directly, or fly lower.")
            print(f"[flight]   (a SubjectStandoff wider than {blind_m:.1f} m keeps "
                  f"the subject in frame anyway - the Shield stops the approach "
                  f"before the blind spot does.)")

        _warn_blind()

        last_est_seq = None
        if id_cfg is not None and not start_gate.get("reached"):
            # Strict lock and nothing committed: nothing would ever be held.
            # Find the subject by the re-acquisition gate instead (no anchor).
            reacq = Reacquirer(need=args.reacq_need, max_range_m=args.reacq_max_range_m)
            reacq.start(None)
            grounder.begin_reacquire()
            lock_events.append({"t": 0.0, "event": "acquire", "anchor": None})
        if estimator is not None:
            print(f"[flight] target estimator ON, stand-off {want_range:.1f} m "
                  f"(from --want-width {args.want_width} and a "
                  f"{args.object_width_m:.2f} m subject)")
        while time.time() - t0 < args.max_s:
            tick += 1
            _tk0 = time.time()
            mode_label = None       # HUD state text when not the default
            det_tier = None         # identity tier of the steering box
            plan_dir = None         # the junction planner's flight direction
            plan_row = None
            # Retarget BEFORE the tick reads anything, so the whole tick - the
            # detector query, the stand-off class, the log row - describes one
            # target rather than half of each.
            now_s = time.time() - t0
            while retargets and now_s >= retargets[0][0]:
                t_at, phrase = retargets.pop(0)
                old_class, old_obj = subject_class, args.object
                args.object = phrase
                new_w, subject_class = subject_width(phrase)
                # The range estimate is scaled by the subject's real width, so a
                # 4 m car prior left on a 0.5 m person reports them ~8x too far
                # away. Only override a width the user did not pin by hand.
                if not args.object_width_pinned:
                    args.object_width_m = new_w
                # THE SET-POINT, recomputed from the new width. Updating the
                # width and not this is what made the pedestrian half of the
                # demo hold the car's 15.83 m and never approach the 10 m ring.
                # --want-width is an ANGULAR target, so the range it asks for
                # scales with the subject's real width: 0.16 is 15.83 m for a
                # 4 m car and 1.98 m for a 0.5 m person.
                old_range = want_range
                want_range = _stand_off()
                if abs(want_range - old_range) > 0.05:
                    print(f"[retarget]   stand-off {old_range:.2f} m -> "
                          f"{want_range:.2f} m (--want-width {args.want_width} "
                          f"against a {args.object_width_m:.2f} m subject)")
                    _warn_blind()
                if grounder is not None:
                    grounder.retarget(phrase)
                # The presence monitor holds the OLD subject's plausible-width
                # band and colour word until told otherwise, so without this a
                # person is judged against a car's width for the whole second
                # half of the flight - the same "left pinned to the car across
                # the retarget" defect already fixed for truth and for sep_*,
                # in a third place.
                if presence is not None:
                    presence.retarget(phrase)
                if estimator is not None:
                    # The old track is a different object's position and
                    # velocity. Carrying it would have the Shield hold a
                    # stand-off from where the CAR was.
                    estimator.reset()
                    # And the speed ceiling belongs to the CLASS, so it moves
                    # with the subject exactly as the width prior does. Left
                    # pinned to the car's 15 m/s, a pedestrian track is free to
                    # diverge to 13 m/s again - which is the fourth place in
                    # this loop where a per-subject quantity had to be told
                    # that the subject changed.
                    estimator.v_max = SUBJECT_VMAX_MPS.get(subject_class)
                if trail is not None:
                    # Where the CAR drove is not where the new subject went;
                    # coast, search and the forward direction would otherwise
                    # follow the old subject's breadcrumbs (review, 2026-09-24).
                    trail.clear()
                    trail_stop_short = _trail_stop_short()
                # ...and so is where it was last measured, and how fast.
                acc_hist.clear()
                last_acc = None
                last_meas_speed = None
                last_heading = None   # the old subject's direction is not this one's
                v_prior = None
                t_ground_ok = 0.0     # no box of the NEW subject is ground-checked yet
                if id_cfg is not None:
                    # The strict lock holds nothing of the new subject; only the
                    # re-acquisition gate may start one (no anchor: new subject).
                    if reacq is not None:
                        lock_events.append({"t": round(now_s, 2), "event": "reacq_abandoned",
                                            **_retire(reacq, "retarget")})
                    subject_committed = False
                    t_meas_last = None
                    t_meas_last_cap = None
                    last_reacq_seq = grounder.latest()["seq"]
                    reacq = Reacquirer(need=args.reacq_need,
                                       max_range_m=args.reacq_max_range_m)
                    reacq.start(None)
                    grounder.begin_reacquire()
                    lock_events.append({"t": round(now_s, 2), "event": "retarget",
                                        "anchor": None})
                ev = {"t": round(now_s, 3), "tick": tick,
                      "scheduled_at_s": t_at,
                      "from": {"object": old_obj, "class": old_class},
                      "to": {"object": phrase, "class": subject_class,
                             "object_width_m": round(args.object_width_m, 3)}}
                retarget_events.append(ev)
                print(f"[retarget] t+{now_s:.1f}s  {old_obj!r} ({old_class}) "
                      f"-> {phrase!r} ({subject_class})", flush=True)
            _ta = time.time()
            kin = drone.get_ground_truth_kinematics()
            _ms_kin = (time.time() - _ta) * 1000
            p = kin["pose"]["position"]
            yaw = quat_yaw(kin["pose"]["orientation"])
            state = State(x=p["x"], y=p["y"], up=-p["z"])
            obs.put_pose(p["x"], p["y"], yaw)
            # The full pose, into the ring get_front_capture reads: stamped
            # mid-RPC, and with the sim's own time_stamp when it has one.
            _qk = kin["pose"]["orientation"]
            obs.put_pose_full(_ta + _ms_kin / 2000.0, p["x"], p["y"], -p["z"], yaw,
                              quat_pitch(_qk), quat_roll(_qk),
                              stamp=kin.get("time_stamp"))
            _ta = time.time()
            if people is not None:
                # Only the pacing few cost anything here; the standing majority
                # is skipped without an RPC. Scenery is never allowed to fail a
                # flight, so update() swallows its own errors.
                people.update(time.time() - t0)
            if traffic is not None:
                traffic.update(time.time() - t0, tick)
            elif car is not None and type(car).__name__ == "LevelCar":
                # The level drives it; update() polls the simulator and throttles
                # itself. It has no pose_at: nothing about its path is known in
                # advance. Falling through to the branch below is what crashed
                # citylife_redcar on its first tick.
                car.update(time.time() - t0)
            elif car is not None and env_car is None and tick % 2 == 0:
                # Only when the client owns the motion. With an env actor the
                # simulator is already interpolating the uploaded trajectory,
                # and teleporting on top of it would fight the playback.
                car.update(time.time() - t0)
            elif car is not None:
                # Keep ground truth current for scoring without an RPC: pose_at
                # is a pure function of time and describes exactly the path that
                # was uploaded, so the metrics and the simulator agree.
                car.pos = car.pose_at(time.time() - t0)[:2]
            _ms_truth = (time.time() - _ta) * 1000

            g = grounder.latest()
            # age against the last DETECTION, not the last inference
            det = g["det"]
            det_raw = det
            age = (time.time() - g["t_det"]) if g["t_det"] else 1e9
            # THE SYSTEM ALREADY KNEW, AND STEERED ANYWAY.
            #
            # On citylife_city the presence check called the box ABSENT on 83 %
            # of ticks - "implies 2.9 m wide at 75 m; a person is 0.2-1.5 m" -
            # and the controller servoed on it regardless, because the verdict
            # only decided whether the search branch may creep forward. The
            # video shows the result: a TARGET LOCKED box on a building facade
            # while the people were in plain view. With the gate on, a box the
            # verdict rejects is not a sighting: no estimator update, no servo,
            # and the coast/search branches take over exactly as for a miss.
            presence_blocked = False
            gate_why = None
            ground_skipped = False
            ground_rescued = False
            ground_ok_now = False
            pitch_now = quat_pitch(kin["pose"]["orientation"])
            # Is this tick's box a NEW detection? Decided before the gate and
            # consumed here whatever the gate says: a detection refused on its
            # own tick and passed on a later one - pitch and the depth behind
            # the held box both change - was fed seconds late with the
            # aircraft's current pose (review round 4). The estimator, the
            # trail and the waiver's counters act only on fresh ones.
            fresh_det = bool(det is not None and g["t_det"]
                             and g["t_det"] != last_est_seq)
            if fresh_det:
                last_est_seq = g["t_det"]
            if id_cfg is not None:
                det_tier = g.get("tier") if det is not None else None
                _c0 = g.get("cap") or {}
                if fresh_det and (_c0.get("t_capture") or _c0.get("t_cap")):
                    det_latency.append((time.time() - (_c0.get("t_capture") or _c0["t_cap"]))
                                       * 1000.0)
                if estimator is None and fresh_det and reacq is None:
                    # NO ESTIMATOR: the fresh held box is the measurement, so
                    # the flight's memory of the subject follows IT - else a
                    # lapse anchored on the place of the commit, 60+ m behind
                    # a car followed for 20 s, and refused the car for good as
                    # "out of reach" (re-review, 2026-09-30). SOFT boxes only
                    # continue a track with an OK box in the last 3 s, as the
                    # estimator path has it.
                    _now = time.time()
                    _P = (g.get("feat") or {}).get("P")
                    if det_tier == "ok" or _now - t_ok_last < 3.0:
                        if det_tier == "ok":
                            t_ok_last = _now
                        t_meas_last = _now
                        if _P is not None:
                            t_meas_last_cap = (_c0.get("t_capture") or _c0.get("t_cap") or _now)
                            acc_hist.append((_now, float(_P[0]), float(_P[1])))
                            del acc_hist[:-40]
                            last_acc = (float(_P[0]), float(_P[1]))
                            _v = measured_speed(acc_hist)
                            if _v is not None:
                                last_meas_speed = _v
                            _h0 = [h for h in acc_hist if acc_hist[-1][0] - h[0] <= 2.0][0]
                            if acc_hist[-1][0] - _h0[0] >= 1.0:
                                _dx, _dy = acc_hist[-1][1] - _h0[1], acc_hist[-1][2] - _h0[2]
                                if math.hypot(_dx, _dy) > 1.0:
                                    last_heading = math.atan2(_dy, _dx)
                # RE-ACQUISITION: judge each NEW inference's candidates; the box
                # the grounder publishes meanwhile is nobody's (it holds none).
                if reacq is not None:
                    det, fresh_det = None, False
                    if g["seq"] != last_reacq_seq and g.get("inf_cap"):
                        last_reacq_seq = g["seq"]
                        ic = g["inf_cap"]
                        t_ic = ic.get("t_capture") or ic.get("t_cap") or time.time()
                        if reacq.step(g["cands"], g["cand_tiers"], g["cand_feats"], t_ic):
                            pick, pf, cp = reacq.pick, reacq.pick_feat, ic.get("pose")
                            grounder.commit(pick, yaw=(cp or {}).get("yaw"))
                            _hd = reacq.heading()
                            seeded, _P0 = _take_subject(pf, ic)
                            if _hd is not None:
                                last_heading = _hd
                            ev = {"t": round(time.time() - t0, 2), "event": "reacquired",
                                  "after_s": round(time.time() - t0 - lock_events[-1]["t"], 2)
                                  if lock_events else None,
                                  "P": [round(v, 1) for v in pf["P"][:2]],
                                  "rng_h": round(pf["rng_h"], 1) if pf.get("rng_h") else None,
                                  "seeded": bool(seeded), "streak": reacq.streak,
                                  **_retire(reacq, "reacquired")}
                            lock_events.append(ev)
                            print(f"[identity] t+{ev['t']:.1f}s RE-ACQUIRED at "
                                  f"{ev['P']} ({ev['rng_h']} m)", flush=True)
                            reacq = None
                # LAPSE: no accepted measurement for lapse_s. The estimate is
                # a guess by now; nothing may re-seed it except the gate above.
                # Keyed on the flight's own clock of accepted measurements and
                # commits, not on the estimator: with the estimator never
                # seeded (or --no-target-estimator) the old test could never
                # fire, and the flight sat with nothing held for good.
                elif (subject_committed and t_meas_last is not None
                      and time.time() - t_meas_last > args.lapse_s):
                    have_est = estimator is not None and estimator._xu is not None
                    P_a = ((float(estimator._xu[0]), float(estimator._xu[1]))
                           if have_est else last_acc)
                    anchor = (None if P_a is None else
                              {"t": (estimator.t_last_update if have_est
                                     and estimator.t_last_update is not None
                                     else (t_meas_last_cap if t_meas_last_cap is not None
                                           else t_meas_last)),
                               "P": P_a,
                               "v": (last_meas_speed if last_meas_speed is not None
                                     else v_prior)})
                    if estimator is not None:
                        estimator.reset()
                    subject_committed = False
                    reacq = Reacquirer(need=args.reacq_need,
                                       max_range_m=args.reacq_max_range_m)
                    reacq.start(anchor)
                    grounder.begin_reacquire()
                    det, fresh_det = None, False
                    lock_events.append({"t": round(time.time() - t0, 2), "event": "lapse",
                                        "P": (None if anchor is None else
                                              [round(v, 1) for v in anchor["P"]]),
                                        "v": (None if anchor is None or anchor["v"] is None
                                              else round(anchor["v"], 2))})
                    print(f"[identity] t+{time.time() - t0:.1f}s subject LOST "
                          f"(> {args.lapse_s:.1f} s unseen) - re-acquiring", flush=True)
                if reacq is None and estimator is not None and estimator._xu is not None:
                    xu = estimator._xu
                    grounder.set_prior({"t": estimator.t_last_update,
                                        "x": float(xu[0]), "y": float(xu[1]),
                                        "vx": float(xu[2]), "vy": float(xu[3]),
                                        "t_upd": estimator.t_last_update,
                                        # the subject's centre height, for the
                                        # projection into the frame
                                        "up": SUBJECT_CENTRE_UP_M.get(subject_class, 0.75)})
                else:
                    grounder.set_prior(None)
            if args.presence_gates_control and det is not None and id_cfg is None:
                r_pre = range_from_depth(obs.get_depth(), det) if have_depth else None
                ground = (state.up, pitch_now)
                # THE CAR BEING TRACKED IS NOT A DISTRACTOR.
                #
                # The ground-contact check exists to keep a red signal from
                # CAPTURING the lock. On citylife_redcar_trail it rejected the
                # tracked car itself - 34 inferences with the box on the car,
                # every other check passing - through the second corner, where
                # the follow then fell 40 m behind and lost it. Its inputs are
                # fragile exactly there: a small-angle ray, a box bottom a few
                # pixels off, and a body pitch read NOW for a frame captured
                # 150-250 ms ago during braking and banking. A box that lands
                # where the estimator already predicts the subject (bearing,
                # and range when depth has one) is continuity, not capture, so
                # it is not asked to prove it stands on the road. With no
                # estimate - acquisition, or after 3 s unseen - it still is.
                #
                # And only while the estimate is itself anchored on the road:
                # a box that PASSED the ground check must have fed it within
                # the last 3 s. Otherwise a red signal beside a stopped car
                # could be waived in, pull the estimate onto itself, and keep
                # the waiver for as long as it stood there - the capture this
                # check exists to stop (found in review, 2026-09-24).
                det_in = det
                if (estimator is not None and r_pre is not None
                        and time.time() - t_ground_ok < 3.0):
                    pred = estimator.observe(time.time(), state.x, state.y, yaw)
                    b_box = box_bearing(det[0], det[5])
                    if agrees_with_estimate(b_box, r_pre, pred):
                        ground, ground_skipped = None, True
                        if fresh_det:                         # per detection
                            n_ground_skipped += 1
                det, presence_blocked, gate_why = presence_gate(
                    det, r_pre, args.object, args.colour_min, ground=ground)
                n_presence_blocked += int(presence_blocked)
                if presence_blocked:
                    k = block_reason_key(gate_why)
                    block_reasons[k] = block_reasons.get(k, 0) + 1
                elif ground_skipped:
                    # A RESCUE is a box the ground check would have refused
                    # and the waiver let through; agreement alone is not one.
                    _, _, why_g = presence_gate(
                        det_in, r_pre, args.object, args.colour_min,
                        ground=(state.up, pitch_now))
                    if block_reason_key(why_g) == "not on the ground":
                        ground_rescued = True
                        if fresh_det:                         # per detection
                            n_ground_rescued += 1
                    elif (ground_range_m(det_in, state.up, pitch_now) is not None
                          and block_reason_key(why_g) != "not on the ground"):
                        # An agreeing box that would ALSO have passed the
                        # ground check keeps the anchor fresh: in steady
                        # tracking every box agrees, and the anchor used to
                        # lapse every 3 s for want of a non-agreeing one
                        # (review round 4).
                        ground_ok_now = True
                elif (r_pre is not None and ground is not None
                      and plausible_noun(args.object) in GROUND_NOUNS
                      and ground_range_m(det_in, state.up, pitch_now) is not None):
                    # Passed WITH the ground check actually evaluated: near the
                    # horizon ground_range_m gives up and the verdict never
                    # tests contact, and such a box must not arm the waiver
                    # (review, 2026-09-24).
                    ground_ok_now = True
            # FEED THE ESTIMATOR, ONCE PER NEW DETECTION.
            #
            # Only measurements go in: the box centre, the depth range, and the
            # aircraft's own pose. No target ground truth, ever - tgt_x/tgt_y in
            # the log are for scoring and must not reach this.
            # A FRESH detection, not a fresh inference: the Grounder keeps its
            # last box for up to 8 s through inferences that find nothing, and
            # keying this on `seq` re-fed that box with the aircraft's CURRENT
            # pose on every one of them - breadcrumbs laid 15-190 m from the
            # car, an estimate at 4.9 m/s for a car standing still
            # (citylife_redcar_trail2; found in review, 2026-09-24).
            if estimator is not None and det is not None and fresh_det:
                # CAMERA_HFOV_DEG / 2, not a literal 45.0. This line is
                # servo()'s bearing formula written out a second time, and it
                # is the one the ESTIMATOR is fed - so a camera change that
                # missed it would send the Shield a subject in the wrong
                # direction with nothing raising.
                b_meas = box_bearing(det[0], det[5])
                t_meas, x_m, y_m, yaw_m = time.time(), state.x, state.y, yaw
                feed = True
                r_meas = None
                if id_cfg is not None:
                    # AT CAPTURE: the frame's own pose and time, and the range
                    # and bearing identity measured from them (pinhole, pitch
                    # and roll included, horizontal range).
                    cap = g.get("cap") or {}
                    feat = g.get("feat") or {}
                    cp = cap.get("pose")
                    if cp is not None:
                        t_meas = cap.get("t_capture") or cap.get("t_cap") or t_meas
                        x_m, y_m, yaw_m = cp["x"], cp["y"], cp["yaw"]
                    if feat.get("rng_h") is not None and cp is not None:
                        r_meas = float(feat["rng_h"])
                        b_meas = feat["bearing"] - yaw_m
                        b_meas = math.atan2(math.sin(b_meas), math.cos(b_meas))
                    if g.get("tier") != "ok":
                        # SOFT: may CONTINUE a live track only.
                        est_age = (t_meas - estimator.t_last_update
                                   if estimator.t_last_update is not None else 1e9)
                        feed = est_age < 1.5 and time.time() - t_ok_last < 3.0
                        n_soft_fed += int(feed)
                        n_soft_refused += int(not feed)
                elif have_depth:
                    r_meas = range_from_depth(obs.get_depth(), det)
                if r_meas is None:
                    r_meas = implied_range_from_width(det, args.object_width_m)
                if r_meas and feed:
                    accepted = estimator.update(t_meas, x_m, y_m,
                                                yaw_m, b_meas, float(r_meas))
                    if accepted and id_cfg is not None and g.get("tier") == "ok":
                        t_ok_last = time.time()
                    if accepted and id_cfg is not None:
                        t_meas_last = time.time()
                    if accepted and ground_ok_now:
                        t_ground_ok = time.time()
                    if accepted:
                        acc_hist.append((time.time(), float(estimator.x[0]),
                                         float(estimator.x[1])))
                        del acc_hist[:-40]
                        last_acc = (float(estimator.x[0]), float(estimator.x[1]))
                        _v = measured_speed(acc_hist)
                        if _v is not None:
                            last_meas_speed = _v
                        if len(acc_hist) >= 2 and acc_hist[-1][0] - acc_hist[0][0] >= 1.0:
                            _h0 = [h for h in acc_hist if acc_hist[-1][0] - h[0] <= 2.0][0]
                            _dx, _dy = acc_hist[-1][1] - _h0[1], acc_hist[-1][2] - _h0[2]
                            if math.hypot(_dx, _dy) > 1.0:
                                last_heading = math.atan2(_dy, _dx)
                    if accepted and trail is not None:
                        trail.add(float(estimator.x[0]), float(estimator.x[1]))

            est_obs = (estimator.observe(time.time(), state.x, state.y, yaw)
                       if estimator is not None else None)

            if est_obs is not None:
                # SERVO ON THE ESTIMATE, NOT ON THE LAST BOX.
                #
                # The forward channel used apparent box width, which jitters
                # p95 37.7% between detections in traffic and pushed the raw
                # command 1.897 m/s in a single 0.1 s tick, clipped by the slew
                # limiter on 20% of ticks. Replayed on the recorded flights,
                # servoing on the estimated range instead drops that p95 from
                # 0.6375 to 0.0991 - and the estimate is available on 100% of
                # ticks rather than the 38-55% that carried a fresh box.
                bearing, rng_est = est_obs
                # THE SAME CAP THE COAST BRANCH HAS HAD ALL ALONG (see below).
                #
                # Uncapped, this branch commanded 130.4 deg/s on the retarget
                # flight - the aircraft spinning on the spot, which is most of
                # what "the tracking looks confused" was describing. The coast
                # branch clips to 1.1 rad/s and this one did not, so the fast
                # yaw only ever appeared when the estimator was driving, which
                # is why it survived every flight the estimator sat out.
                yaw_rate = yaw_command(args.yaw_gain, bearing)
                # Proportional term plus the target's own opening rate fed
                # FORWARD. Without the feedforward the loop settles at a lag
                # error - a 2 m/s car held 23.4 m against a 15.8 m stand-off,
                # exactly the (r - want)*gain = 2.0 fixed point. An integrator
                # would also fix it and would wind up every time the Shield
                # overrides the actuator; the estimator already knows the
                # target's velocity, so no memory is needed.
                ff = (estimator.range_rate(state.x, state.y)
                      if args.range_feedforward else 0.0)
                fwd = float(np.clip((rng_est - want_range) * args.range_gain + ff,
                                    -0.4 * args.speed_max, args.speed_max))
                fwd *= max(0.0, math.cos(bearing))
                # PREDICTION IS NOT EVIDENCE. For up to 3 s after the last
                # accepted box the estimator serves a constant-velocity guess;
                # when the car has in fact stopped, closing on that guess flew
                # citylife_redcar_trail2 from 14.8 m to 1.7 m of a stationary
                # car, into the camera's blind spot, in TRACK mode. Once the
                # estimate is half a second stale, do not advance nearer the
                # last MEASURED position than the blind spot plus 2 m - for a
                # subject MEASURED to have stopped. Applied to a moving car it
                # throttled the chase to 1.4 m/s whenever updates paused, since
                # following a car puts the aircraft ~10-13 m from where the car
                # last WAS (citylife_redcar_final1: 13 m -> 25 m behind in 8 s,
                # then lost; review round 4 had flagged it).
                if (last_acc is not None and estimator.t_last_update is not None
                        and last_meas_speed is not None and last_meas_speed < 1.0
                        and time.time() - estimator.t_last_update > 0.5):
                    r_last = math.hypot(last_acc[0] - state.x, last_acc[1] - state.y)
                    floor = camera_blind_m(state.up) + 2.0
                    fwd = min(fwd, max(0.0, 0.5 * (r_last - floor)))
                if last_seen:
                    dt = max(1e-3, time.time() - last_seen)
                    brg_rate = 0.6 * brg_rate + 0.4 * ((bearing - last_bearing) / dt)
                last_seen, last_bearing = time.time(), bearing
                last_cmd = (yaw_rate, fwd)
                mode, seen = "track", True
                if id_cfg is not None:
                    # TARGET LOCKED only on a fresh identity-OK box; otherwise
                    # say that the aircraft is flying on the estimate.
                    if det is None or age > 0.6:
                        mode_label = "TRACKING (PREDICTED)"
                    elif det_tier == "soft":
                        mode_label = "TRACKING (PARTLY HIDDEN)"
            elif det is not None and age < args.det_max_age and reacq is None:
                yaw_rate, fwd, bearing = servo(
                    det, det[5], args.yaw_gain, args.want_width, args.speed_max)
                # remember what it was doing, so a gap can be coasted through
                if last_seen:
                    dt = max(1e-3, time.time() - last_seen)
                    brg_rate = 0.6 * brg_rate + 0.4 * ((bearing - last_bearing) / dt)
                last_seen, last_bearing = time.time(), bearing
                last_cmd = (yaw_rate, fwd)
                mode, seen = "track", True
            else:
                # Detection drops on roughly a quarter of ticks — the car leaves
                # frame on a corner, or the detector simply misses. Freezing on
                # every gap meant the drone stopped dead and fell behind, so it
                # coasts on what the target was doing, then searches, and only
                # then gives up.
                seen = False
                # With a trail: where to look (a little past the last sighting,
                # along the subject's last direction of travel) and how much
                # street is left to fly to reach that sighting. None without.
                look_b = (trail_mod.lookout(trail, state.x, state.y, yaw)
                          if trail is not None else None)
                trail_rem = (trail.remaining(state.x, state.y)
                             if look_b is not None else 0.0)
                # How much is left to the last sighting, and how far short of
                # it to stop. The Shield has no subject while the estimator is
                # silent, so its ring is not held: the controller keeps out of
                # it. Straight-line distance as well as along the trail - at a
                # corner the carrot cuts across, and the along-trail figure
                # alone let the approach end inside the ring (review,
                # 2026-09-24). And a subject last seen STOPPED is probably
                # still there: the follow's own stand-off, not the policy's
                # minimum. Approaching a stopped car to 5 m put it in the
                # camera's 6.9 m blind spot under the nose, and
                # citylife_redcar_trail2 never saw it again.
                # "Stopped" is MEASURED - displacement of accepted positions -
                # not the filter's velocity, which kept 1.5-5 m/s for seconds
                # after the car stood still (review, 2026-09-24). Never stop
                # nearer than the camera's blind spot plus 2 m either way.
                subject_stopped = (last_meas_speed is not None
                                   and last_meas_speed < 1.0)
                if look_b is not None:
                    _end = trail.end()
                    d_end = math.hypot(_end[0] - state.x, _end[1] - state.y)
                    left = min(trail_rem, d_end)
                    floor = camera_blind_m(state.up) + 2.0
                    stop_at = (max(trail_stop_short, want_range, floor)
                               if subject_stopped
                               else max(trail_stop_short, floor))
                else:
                    d_end = left = stop_at = 0.0
                if not last_seen:
                    # Never acquired yet. Sweep — do not sit still waiting for a
                    # target to wander into frame. This was the failure mode the
                    # first time: the aircraft held its launch heading for the
                    # whole flight while the car drove a lap behind it.
                    yaw_rate, fwd, bearing = args.search_rate, 0.0, 0.0
                    mode = "search"
                    lost = 0.0
                elif (lost := time.time() - last_seen) < args.coast_s:
                    # keep turning the way the target was moving, decaying
                    k = 1.0 - lost / args.coast_s
                    bearing = last_bearing + brg_rate * lost
                    fwd = last_cmd[1] * k
                    if look_b is not None:
                        # Not "keep doing what it was doing": that carried the
                        # aircraft straight past the junction the car had
                        # turned at. Fly the trail to the last sighting, and
                        # look where the car was heading from there.
                        bearing = look_b
                        # ...but stop the policy's stand-off short of it: the
                        # estimator has nothing, so the Shield has no subject
                        # and its ring is not being held. A car stopped behind
                        # a tree is still there. And an aircraft that was
                        # BACKING OFF keeps backing - the approach may not turn
                        # a retreat into a closing run (review, 2026-09-24).
                        if last_cmd[1] >= 0.0:
                            fwd = trail_mod.approach_speed(
                                max(0.0, left - stop_at), max(last_cmd[1], 1.0))
                    yaw_rate = yaw_command(args.yaw_gain, bearing)
                    mode = "coast"
                elif lost < args.coast_s + args.search_s:
                    # A BOUNDED SWEEP AROUND THE LAST BEARING, AND KEEP MOVING.
                    #
                    # What this replaced was `yaw_rate = side * search_rate` --
                    # a constant sign, so despite the comment calling it a sweep
                    # the nose just rotated and never came back. Measured on
                    # demo_traffic: one lost-lock episode swept 292 degrees,
                    # which is the "manuver 360" seen on the video.
                    #
                    # It is the worst possible response to why the lock is
                    # actually lost here. All three episodes across two flights
                    # happened at the SAME place, with the target dead ahead
                    # (aspect 0.4-2.9 deg) and the presence monitor still saying
                    # PRESENT: the car is behind a street tree at the
                    # intersection, roughly where it stops. Turning away from a
                    # bearing that is still correct cannot help.
                    #
                    # Two things do. A sweep that oscillates about the last
                    # bearing keeps re-crossing where the target really is, and
                    # integrates to zero net rotation so the nose does not walk
                    # off. And creeping FORWARD changes the parallax, which is
                    # what actually moves a tree out of the line of sight --
                    # while also holding the range. Standing still cost 9 m of
                    # separation in one 16 s episode (26.3 -> 35.5 m), shrinking
                    # the target and making the reacquisition harder the longer
                    # it took: positive feedback.
                    if args.search_legacy_spin:
                        side = 1.0 if last_bearing >= 0 else -1.0
                        yaw_rate, bearing = side * args.search_rate, 0.0
                    elif look_b is not None:
                        # The same sweep, but CLOSED on the lookout bearing
                        # instead of integrated open-loop from wherever the
                        # nose happened to be when the coast ended - so it is
                        # centred down the street the subject went into.
                        w = 2.0 * math.pi / max(0.5, args.search_period_s)
                        bearing = look_b + math.radians(args.search_sweep_deg) \
                            * math.sin(w * (lost - args.coast_s))
                        yaw_rate = yaw_command(args.yaw_gain, bearing)
                    else:
                        yaw_rate = search_sweep_rate(lost - args.coast_s,
                                                     args.search_sweep_deg,
                                                     args.search_period_s)
                        bearing = last_bearing
                    # Creep only while the system still believes the object is
                    # out there. On the --no-car control the verdict is ABSENT
                    # and the aircraft must not go wandering up the street.
                    fwd = (abs(last_cmd[1]) * args.search_creep
                           if last_verdict != "ABSENT" else 0.0)
                    if look_b is not None:
                        if last_cmd[1] < 0.0 or (subject_stopped and d_end <= stop_at):
                            # Never creep up on a subject last seen STOPPED -
                            # it is probably still there - and never turn a
                            # retreat into an advance. A subject last seen
                            # MOVING has left the last sighting: once the
                            # approach is done, creep on along its direction
                            # (review, 2026-09-24: zeroing it here parked the
                            # aircraft short of the junction mouth).
                            fwd = 0.0
                        elif not subject_stopped:
                            # Last seen MOVING: go where it went, at the speed
                            # it was measured going (at least 1.5 m/s), down its
                            # trail and on along its last direction - whatever
                            # the presence verdict, which only says the held
                            # box has dropped. Scaling this from the last
                            # command crept at 0.3 m/s, because the prediction
                            # window's cap had already slowed that command,
                            # and parked the aircraft ~9 m short of the
                            # junction mouth in every emulated corner (review
                            # round 4).
                            fwd = max(fwd, min(args.speed_max,
                                               max(last_meas_speed or 0.0, 1.5)))
                    mode = "search"
                elif planner is not None and last_acc is not None:
                    # ON ITS STREETS, NOT IN PLACE (demo/search.py). The scan
                    # below rotated at zero speed wherever the aircraft stood -
                    # against a building corner in citylife_redcar_trail while
                    # the car drove on. Pursue to the next junction the way the
                    # car was going, look down each exit, then hold over it.
                    t_plan = lost - args.coast_s - args.search_s
                    cmd = planner.step(t_plan, (state.x, state.y),
                                       {"p": last_acc, "heading": last_heading,
                                        "speed": last_meas_speed,
                                        "stopped": subject_stopped})
                    tx, ty = cmd["target_xy"]
                    dxp, dyp = tx - state.x, ty - state.y
                    dist = math.hypot(dxp, dyp)
                    look = cmd["look_heading_rad"]
                    if 0.0 < cmd.get("sweep_deg", 0.0) < 180.0 and not cmd.get("rotate_rad_s"):
                        w = 2.0 * math.pi / max(0.5, args.search_period_s)
                        look += math.radians(cmd["sweep_deg"]) * math.sin(w * t_plan)
                    bearing = math.atan2(math.sin(look - yaw), math.cos(look - yaw))
                    yaw_rate = yaw_command(args.yaw_gain, bearing)
                    fwd = (min(cmd["speed_cap_mps"], args.speed_max, 0.5 * dist)
                           if dist > 1.0 else 0.0)
                    plan_dir = (dxp / dist, dyp / dist) if dist > 1e-3 else None
                    mode = "plan"
                    mode_label = {"pursue": "PURSUING down its street",
                                  "dwell": f"WATCHING {str(cmd.get('exit') or '').upper()} EXIT",
                                  "hold": "HOLDING OVER JUNCTION",
                                  "standoff": "WAITING AT STAND-OFF"}.get(
                                      cmd["mode"], "SEARCHING ...")
                    plan_row = {"mode": cmd["mode"], "t": round(t_plan, 1),
                                "target": [round(tx, 1), round(ty, 1)],
                                "look_deg": round(math.degrees(look), 1),
                                "junction": cmd.get("junction"),
                                "exit": cmd.get("exit")}
                else:
                    # Only NOW is a full rotation the right answer. The bounded
                    # sweep has already had `search_s` to re-find something near
                    # the last bearing and failed, so the target is genuinely
                    # somewhere else -- on the circuit demos it is driving a lap
                    # and will come back round. A drone that gives up stays
                    # pointed at nothing for the rest of the flight, which is
                    # what it used to do before there was a scan at all.
                    side = 1.0 if last_bearing >= 0 else -1.0
                    yaw_rate = side * args.search_rate * 0.6
                    fwd, bearing = 0.0, 0.0
                    mode = "scan"

            if reacq is not None:
                if reacq.pick_feat is not None and reacq.streak > 0:
                    # Keep the candidate being confirmed in frame, and wait.
                    Pk = reacq.pick_feat["P"]
                    b_k = math.atan2(Pk[1] - state.y, Pk[0] - state.x) - yaw
                    bearing = math.atan2(math.sin(b_k), math.cos(b_k))
                    yaw_rate = yaw_command(args.yaw_gain, bearing)
                    fwd = min(fwd, 1.0)
                    mode_label = f"RE-ACQUIRING {reacq.streak}/{reacq.need}"
                elif (far_xy := reacq.far_lead(time.time())) is not None:
                    # CLOSE THE RANGE ON A FAR LEAD (Reacquirer docstring): the
                    # only thing wrong with it is its distance. Nose on it,
                    # forward only once the nose is on it, and stop short of
                    # the gate's range - the gate decides from there.
                    dxf, dyf = far_xy[0] - state.x, far_xy[1] - state.y
                    dist_f = math.hypot(dxf, dyf)
                    b_f = math.atan2(dyf, dxf) - yaw
                    bearing = math.atan2(math.sin(b_f), math.cos(b_f))
                    yaw_rate = yaw_command(args.yaw_gain, bearing)
                    gap_f = dist_f - 0.8 * reacq.max_range_m
                    fwd = (min(args.speed_max, max(0.0, 0.5 * gap_f))
                           * max(0.0, math.cos(bearing)))
                    plan_dir = None                    # along the nose
                    n_far_approach += 1
                    mode_label = f"APPROACHING {dist_f:.0f} m"
                elif mode_label is None:
                    mode_label = "SEARCHING ..."

            # altitude hold, in the controller where it belongs — the Shield is a
            # constraint filter, not a regulator
            vz_up = float(np.clip((args.cruise_alt - state.up) * args.alt_gain,
                                  -args.climb_max, args.climb_max))
            # Radial term: forward speed along the nose, which the width servo
            # sizes to hold a stand-off. On its own this is STATION KEEPING —
            # point at the subject, hold distance, hover facing it. Measured on
            # the first orbit attempt: radius held at 37.7 +- 2.6 m (pass) while
            # angular coverage reached only -23.9 deg (fail). The radius was
            # right and the angle never advanced, because nothing in the
            # commanded velocity carried the aircraft AROUND anything.
            #
            # Tangential term: the yaw servo already keeps the nose on the
            # subject, so perpendicular to the nose IS the tangent of a circle
            # about it. One lateral component is the whole orbit.
            #
            # Only while the target is actually in view. Orbiting on a coasted or
            # searched heading would circle a place the subject is not, and the
            # aircraft would spiral away from a target it had already lost.
            orbit_fwd = fwd
            rng_m = range_from_depth(obs.get_depth(), det) if have_depth else None
            verdict, why = presence.update(
                det_raw, rng_m, g.get("app"),
                ground=(state.up, quat_pitch(kin["pose"]["orientation"])))
            # Carried to the next tick, where the search branch decides whether
            # creeping forward is justified. See --search-creep.
            last_verdict = verdict
            if verdict == "ABSENT":
                n_absent += 1
            if args.orbit_speed and mode == "track" and args.orbit_radius > 0:
                if rng_m is not None:
                    # Low-pass the range before it drives anything.
                    #
                    # The sim publishes depth as 16UC1 — uint16 METRES — so the
                    # signal is quantised to 1 m, and the median inside a
                    # shrunken box jumps whenever the box wobbles. A pure
                    # proportional radial term turns each of those jumps into a
                    # command: raising the gain from 0.15 to 0.45 took the flight
                    # from a 24.3 m mean radius to 120.8 m, the aircraft leaving
                    # entirely. The answer is a quieter signal, not a louder
                    # response.
                    rng_f = (rng_m if rng_f is None
                             else (1.0 - args.orbit_rng_lp) * rng_f
                                  + args.orbit_rng_lp * rng_m)
                    # A true range closes the loop the width servo could not.
                    # Positive error means too far, so close in. Same sign as the
                    # width servo, but the signal no longer depends on which face
                    # of the subject happens to be showing.
                    want = float(np.clip(
                        (rng_f - args.orbit_radius) * args.orbit_radial_gain,
                        -args.speed_max, args.speed_max))
                    # ...and rate-limit the command itself, so even a filtered
                    # step cannot become an instant full-speed dash.
                    step = args.orbit_radial_slew * TICK
                    orbit_fwd = float(np.clip(want, orbit_fwd_prev - step,
                                              orbit_fwd_prev + step))
                    orbit_fwd_prev = orbit_fwd
            if args.orbit_speed and mode == "track" and rng_m is None:
                # Clamp the radial term hard while orbiting.
                #
                # Apparent width is a usable range proxy for a car, which looks
                # about the same width from any angle at these distances. It is a
                # BAD one for a 50 x 50 m block: apparent width swings by root-2
                # between face-on and corner-on, so circling the subject makes the
                # width servo read "too close" and command reverse purely from the
                # changing aspect. Measured on the first orbit-mode flight, with
                # the tangential term added and the radial term unclamped: angular
                # coverage improved from -23.9 to +97.7 deg (the tangential term
                # works) while the radius blew out from 37.7 +- 2.6 m to
                # 94.5 +- 51.2 m, reaching 178 m. It orbited, and spiralled away
                # while doing it.
                #
                # There is no range sensor in this path, so width is the only
                # radial feedback there is. Limiting how fast it may act keeps the
                # aspect swing from becoming a spiral while still letting a real
                # range error be corrected, just slowly.
                orbit_fwd = float(np.clip(fwd, -args.orbit_radial_max,
                                          args.orbit_radial_max))
            # WHICH WAY "FORWARD" IS.
            #
            # Along the nose, and the nose is on the subject - so at a corner
            # the aircraft cut straight across toward a car that had already
            # turned, into the corner block. With a trail, forward is toward a
            # carrot on the subject's own track instead: the same speed (the
            # stand-off law above decides it), the yaw still on the subject, but
            # the path the car drove. On a straight street the two coincide.
            # Backing off (a negative forward) stays along the nose, and the
            # orbit and the scan rotation keep their own geometry.
            ux, uy = math.cos(yaw), math.sin(yaw)
            trail_row = None
            if (trail is not None and orbit_fwd > 0.0 and not args.orbit_speed
                    and mode in ("track", "coast", "search")):
                tdir = trail_mod.direction(trail, state.x, state.y,
                                           args.trail_lookahead_m)
                # NEVER AWAY FROM THE SUBJECT. While the estimator serves a
                # position, a trail direction pointing away from it (more than
                # 90 deg off the line of sight) is an old leg - a re-acquisition
                # behind the trail's end, a car come back round the loop - and
                # following it would fly the aircraft away at stand-off speed
                # (review, 2026-09-24). The nose, which is on the subject, wins.
                if (tdir is not None and est_obs is not None
                        and estimator is not None and estimator.x is not None):
                    lx = float(estimator.x[0]) - state.x
                    ly = float(estimator.x[1]) - state.y
                    if lx * tdir[0] + ly * tdir[1] < 0.0:
                        tdir = None
                        n_trail_against += 1
                if tdir is not None:
                    ux, uy = tdir
                    n_trail_ticks += 1
                    end = trail.end()
                    trail_row = {"n": len(trail),
                                 "dir_deg": round(math.degrees(math.atan2(uy, ux)), 1),
                                 "end": [round(end[0], 1), round(end[1], 1)]}
            if mode in ("coast", "search") and look_b is not None:
                # The lookout bearing and the approach are the trail acting too,
                # on ticks where the direction above may not (a zero forward
                # command at the end of the approach, an ABSENT verdict).
                n_trail_lookout += 1
                trail_row = dict(trail_row or {}, look_deg=round(math.degrees(look_b), 1),
                                 rem=round(trail_rem, 1), d_end=round(d_end, 1),
                                 stop_at=round(stop_at, 1), stopped=subject_stopped,
                                 v_meas=(None if last_meas_speed is None
                                         else round(last_meas_speed, 2)))
            if mode == "plan" and plan_dir is not None:
                ux, uy = plan_dir
            # Keep the aim out of the zones' bands (FenceGuard.clear_aim), so a
            # zone beside the car's lane is passed alongside, not stopped at.
            fence_aim = False
            if orbit_fwd > 0.0 and not args.orbit_speed:
                _aim = fence.clear_aim(state.x, state.y, ux, uy,
                                       args.trail_lookahead_m)
                if _aim is not None:
                    ux, uy = _aim
                    fence_aim = True
                    n_fence_aim += 1
            cvx, cvy = orbit_fwd * ux, orbit_fwd * uy
            if args.orbit_speed and mode == "track":
                cvx += -args.orbit_speed * math.sin(yaw)
                cvy += args.orbit_speed * math.cos(yaw)
            fscale, fdist, fblocked = fence.gate(state.x, state.y, cvx, cvy)
            gvx, gvy = cvx * fscale, cvy * fscale
            # Commit to a side at the BRAKE distance, not at the stand-off.
            #
            # gate() only raises `fblocked` inside 6.15 m (k < 0.35), by which
            # point the aircraft is already down to 35% speed, and 5 m of lateral
            # travel at that speed costs more ground than a 2 m/s target gives
            # away. Measured on follow_car_gap.yaml: 26.1% of the flight within
            # 30 m against 99.6% unfenced -- the rule cost the path AND the
            # target. Starting the detour at 12 m is what buys the aircraft
            # enough room to still be following something at the far end.
            approaching = (fdist is not None and fdist < fence.brake_m
                           and fscale < 1.0)
            if fblocked or approaching:
                if fblocked:
                    guard_hold_ticks += 1
                    # ...and NFZ holds only when there is actually an NFZ.
                    #
                    # `fblocked` used to mean "a fence blocked us" and now also
                    # means "a building did", because gate() was taught about
                    # obstacles. Counting both under `nfz_hold_ticks` reported
                    # 7 no-fly-zone holds on follow_car.yaml, a policy that
                    # declares no fence at all. The mission-outcome test reads
                    # `nfz_s`, which is measured separately and was never
                    # affected - this is a reported number being wrong, not a
                    # verdict being wrong.
                    #
                    # ...and `fence.polys` was the wrong test too: with a fence
                    # declared anywhere, a building hold a block away from it
                    # still counted. gate() now says which hazard held.
                    if fence.last_cause == "fence":
                        nfz_hold_ticks += 1
                sx, sy, scost = fence.slide(state.x, state.y, cvx, cvy)
                if sx or sy:
                    # Lateral urgency rises as the fence closes, but never waits
                    # for the stand-off: at the brake distance it is already a
                    # third of slide_speed, which is what makes it anticipatory.
                    urgency = float(np.clip(1.0 - fscale, 0.33, 1.0))
                    lat = args.slide_speed * urgency
                    gvx += sx * lat
                    gvy += sy * lat
                    fmode = "skirt"
                else:
                    fmode = "hold" if fblocked else "near"
            else:
                fmode = "clear" if fdist is None or fdist > 20 else "near"
            raw = Action4D(vx=gvx, vy=gvy, vz_up=vz_up, yaw_rate=yaw_rate)
            smooth = limiter(raw)

            # Tell the Shield where the subject is, so SubjectStandoff rules can
            # bind. The position comes from the ESTIMATOR's state vector, which
            # is built from bearing, range and the aircraft's own pose - so this
            # carries no target ground truth and the no-leak property holds.
            #
            # Cleared to None the moment the estimator has nothing, and that is
            # required rather than tidy: a stale position would have the Shield
            # enforcing a stand-off from where the subject used to be, which is
            # both wrong and invisible in the logs.
            if estimator is not None and estimator.x is not None and est_obs is not None:
                shield.set_subject(float(estimator.x[0]), float(estimator.x[1]),
                                   subject_class)
            else:
                shield.set_subject(None)

            _ta = time.time()
            d = shield.filter(state, smooth)
            audit.log(tick, d)
            _ms_shield = (time.time() - _ta) * 1000
            pol = None
            if pind is not None and pind_error is None:
                try:
                    pol = pind.update(
                        time.time() - t0, d, state, subject_xy=shield.subject,
                        subject_class=shield.subject_class,
                        clearance_m=(shield.clearance_at(state.x, state.y)
                                     if shield.has_obstacle_map else None),
                        off_map=shield.off_map(state.x, state.y),
                        # A re-aimed tick IS the controller routing round
                        # the zone, even when gate() saw nothing to brake.
                        fence_cause=("fence" if fence_aim and fmode in ("clear", "near")
                                     else fence.last_cause),
                        fence_mode=("skirt" if fence_aim and fmode in ("clear", "near")
                                    else fmode),
                        est_xy=shield.subject, yaw_rad=yaw)
                except Exception as exc:                      # noqa: BLE001
                    # A display fault must never cost the flight: the loop
                    # goes on with the plain HUD, and metrics.json says why.
                    pind_error = f"{type(exc).__name__}: {exc}"
                    print(f"[hud] policy indicator stopped: {pind_error}")
                    pol = None
            if d.touched:
                n_touched += 1
            e = d.emitted
            # Off the obstacle map the clearance rule sees nothing - not
            # "clear", unknown. Counted so a flight that left the map says so.
            off_map = (shield.off_map(state.x, state.y)
                       if shield.has_obstacle_map else None)
            if off_map:
                n_off_map += 1
                if n_off_map == 1:
                    print(f"[map] *** t={time.time() - t0:.1f}s: the aircraft is "
                          f"OFF the obstacle map at ({state.x:.1f}, {state.y:.1f}); "
                          "building clearance is not being checked here")

            traj.append({"x": state.x, "y": state.y, "up": state.up,
                         "touched": d.touched})
            rows.append({
                "t": round(time.time() - t0, 3), "tick": tick,
                "x": state.x, "y": state.y, "up": state.up, "psi": yaw,
                "det_seq": g["seq"], "det_age_s": round(min(age, 99), 3),
                "seen": seen, "mode": mode,
                "fence_d": (round(fdist, 2) if fdist is not None else None),
                "fence_scale": round(fscale, 3), "fence_hold": fblocked,
                "fence_mode": fmode,
                "fence_cause": fence.last_cause,
                "fence_aim": fence_aim,
                "trail": trail_row, "off_map": off_map,
                "est": (None if estimator is None else
                        {"served": est_obs is not None,
                         "rng": None if est_obs is None else round(est_obs[1], 2),
                         "spd": round(estimator.speed(), 2),
                         "gated": estimator.n_rejected}),
                "presence": verdict, "presence_why": why,
                "app_sim": (round(presence.last_sim, 3)
                            if presence.last_sim is not None else None),
                "rng_m": (round(rng_m, 2) if rng_m is not None else None),
                "infer_ms": round(g["infer_ms"], 1),
                "pre_ms": round(g.get("pre_ms", 0.0), 1),
                "fwd_ms": round(g.get("fwd_ms", 0.0), 1),
                "det": (None if det is None else
                        {"cx": round(det[0], 1), "cy": round(det[1], 1),
                         "w": round(det[2], 1), "score": round(det[4], 4),
                         "colour": round(det[7], 3), "img_w": int(det[5]),
                         # Was this box a re-acquisition by the instance lock?
                         # The instance scorer re-associates the subject there.
                         "switched": bool(g.get("switched"))}),
                "presence_blocked": presence_blocked,
                "tier": det_tier,
                "lock_why": g.get("lock_why") if id_cfg is not None else None,
                "reacq": (None if reacq is None else
                          {"streak": reacq.streak, "n": reacq.n,
                           "far": len(reacq.far)}),
                "plan": plan_row,
                "est_xy": (None if estimator is None or est_obs is None
                           or estimator.x is None else
                           [round(float(estimator.x[0]), 2),
                            round(float(estimator.x[1]), 2)]),
                "gate_why": gate_why if presence_blocked else None,
                "ground_skip": ground_skipped,
                "ground_rescue": ground_rescued,
                "pitch_deg": round(math.degrees(pitch_now), 2),
                "roll_deg": round(math.degrees(quat_roll(kin["pose"]["orientation"])), 2),
                "bearing_deg": round(math.degrees(bearing), 2),
                "raw": raw.model_dump(), "smooth": smooth.model_dump(),
                "emitted": e.model_dump(),
                "touched": d.touched, "braked": d.braked,
                "violations": [v.model_dump() for v in d.violations],
                "repairs": [r.model_dump() for r in d.repairs],
                # The check on what was FLOWN, not on what was asked for.
                # `violations` above is the check on `raw`, so without this the
                # grant's hard KPI - a P0 seen and then flown anyway - could
                # only be inferred, and guardrail/kpi.py inferred it wrongly.
                # Empty is the good case and the normal one.
                "emitted_violations": [v.model_dump() for v in d.emitted_violations],
                "tgt_x": (car.pos[0] if car else None),
                "tgt_y": (car.pos[1] if car else None),
                # Truth for the CURRENT subject, which `tgt_x/tgt_y` above is
                # not once the flight has retargeted. Kept as its own field so
                # older logs keep scoring exactly as they did.
                "truth": {"class": subject_class,
                          "pts": subject_truth_pts(subject_class, car, people),
                          # Names parallel to pts, when the source has them,
                          # so an instance-level scorer can follow ONE figure.
                          "names": subject_truth_names(subject_class, car, people)},
                "ms": {"kin": round(_ms_kin, 1), "truth": round(_ms_truth, 1),
                       "shield": round(_ms_shield, 1),
                       "work": round((time.time() - _tk0) * 1000, 1)},
            })
            stage_ms["kin"].append(_ms_kin)
            stage_ms["truth"].append(_ms_truth)
            stage_ms["shield"].append(_ms_shield)

            _ta = time.time()
            await drone.move_by_velocity_async(
                e.vx, e.vy, -e.vz_up, duration=0.3,
                yaw_is_rate=True, yaw=e.yaw_rate)
            stage_ms["cmd"].append((time.time() - _ta) * 1000)
            if recorder is not None:
                # Hand over state only. The decode, draw and encode happen on the
                # recorder thread: doing them here cost 32-40 ms a tick and drove
                # the control loop from 10 Hz down to 7.4 Hz, which is what the
                # video showed as lag.
                # To the SUBJECT, not the car - the fourth site of this same
                # defect, after the scorer, the metrics and the presence
                # monitor. The HUD is what a viewer of the demo video reads, so
                # it was the one that mattered most and the one nobody checked.
                sep = _live_sep(state, subject_class, car, people)
                recorder.set_hud({
                    "t": time.time() - t0, "sep": sep,
                    "brg": math.degrees(bearing), "fwd": fwd,
                    "alt": state.up, "shield": d.touched,
                    "query": args.object, "mode": mode,
                    "fence_d": fdist, "fence_hold": fblocked,
                    "presence": verdict, "rng_m": rng_m,
                    "fence_mode": fmode,
                    "fence_kind": (fence.last_cause
                                   or ("fence" if fence.polys else "obstacle")),
                    "mode_label": mode_label, "tier": det_tier,
                    "policy": pol,
                }, det if seen and (id_cfg is None or age < 0.6) else None)
            if tick % 50 == 0:
                sep = _live_sep(state, subject_class, car, people)
                sep = float("nan") if sep is None else sep
                print(f"  tick {tick}: pos=({state.x:6.1f},{state.y:6.1f},"
                      f"{state.up:4.1f}) {mode.upper():6} "
                      f"brg={math.degrees(bearing):+5.1f} fwd={fwd:4.1f} "
                      f"sep={sep:5.1f}m shield={'HIT' if d.touched else '-'}")
            # PACE TO A DEADLINE, NOT A NAP.
            #
            # This slept a full TICK after the tick's work, so the period was
            # work + 100 ms and the loop could never reach its own 10 Hz:
            # 8.33 Hz on retarget_smooth is ~20 ms of work plus the nap, and
            # 4-7 Hz on the CityLife flights is the same nap on a heavier tick.
            # The 9.5 Hz gate was being measured against a loop that was
            # structurally unable to pass it. --legacy-tick-sleep restores the
            # old pacing so recorded flights can be reproduced.
            #
            # ...and to an ABSOLUTE deadline. `TICK - work` was relative to each
            # tick's own start, so every sleep's round-up to the OS timer (15.6
            # ms by default on Windows) was lost for good: 9.31-9.37 Hz on every
            # red-car flight. Stepping a running deadline by TICK lets a late
            # tick be repaid by the next one's shorter sleep. A stall of more
            # than a whole tick is not repaid - that would be a burst of
            # back-to-back commands - the deadline restarts from now instead.
            _work = time.time() - _tk0
            stage_ms["work"].append(_work * 1000)
            if args.legacy_tick_sleep:
                await asyncio.sleep(TICK)
            else:
                _now = time.time()
                tick_deadline = (_tk0 if tick_deadline is None
                                 else tick_deadline) + TICK
                if tick_deadline < _now - TICK:
                    tick_deadline = _now
                await asyncio.sleep(max(0.0, tick_deadline - _now))

        # The mission window closes here, plus the last command's 0.3 s: what
        # the simulator reports after that is the descent and the landing.
        t_mission_end = time.time() + 0.3
        grounder.stop()
        _hud_live = recorder is not None and pind is not None and pind_error is None
        if args.land_site:
            landing_info = await _fly_to_landing_site(
                drone, shield, args, pind=pind if _hud_live else None,
                recorder=recorder if _hud_live else None, t0=t0, query=args.object)
        for _k in range(600):
            kin = drone.get_ground_truth_kinematics()
            up = -kin["pose"]["position"]["z"]
            if _hud_live and _k % 5 == 0:
                # The descent is flown without the Shield and below the policy
                # band by design; say so instead of leaving the last mission
                # tick's verdicts on screen.
                _p = kin["pose"]["position"]
                _st = State(x=float(_p["x"]), y=float(_p["y"]), up=up)
                recorder.set_hud({
                    "t": time.time() - t0, "sep": None, "brg": 0.0, "fwd": 0.0,
                    "alt": up, "shield": False, "query": args.object, "mode": "land",
                    "mode_label": "LANDING - descending", "tier": None,
                    "policy": pind.ended(_st, "MISSION ENDED - DESCENDING TO LAND (Shield not in this loop)",
                                         quat_yaw(kin["pose"]["orientation"]),
                                         t=time.time() - t0),
                }, None)
            if up <= 1.2:
                break
            await drone.move_by_velocity_async(
                0.0, 0.0, max(0.6, min(2.0, up * 0.15)), duration=0.3)
            await asyncio.sleep(0.1)
        await (await drone.land_async())
        drone.disarm()
        drone.disable_api_control()
    except Exception as exc:
        print(f"[warn] flight aborted: {type(exc).__name__}: {exc}")
    finally:
        # WRITE THE FLIGHT LOG FIRST. It used to be written after teardown, and a
        # teardown that hung threw the whole flight away: 70 s flown, 1742 frames
        # recorded, and no flight_log.jsonl, so not one number was recoverable.
        # Nothing below this line may cost us the data again.
        try:
            with (out / "flight_log.jsonl").open("w", encoding="utf-8") as fh:
                for r in rows:
                    fh.write(json.dumps(r) + "\n")
        except Exception as exc:
            print(f"[warn] could not write the flight log: {type(exc).__name__}: {exc}")

        # BOUND THE WHOLE TEARDOWN, not just the disconnect.
        #
        # Every step below is an RPC to the simulator, and a wedged simulator
        # does not raise - it simply never returns, so `try/except` is no
        # protection. Measured twice: a completed flight sat for 48 minutes with
        # the frames written and nothing else, and had to be killed by hand.
        # The flight log is already on disk by this point, so the worst a
        # timeout costs is an undestroyed prop in a simulator we are leaving.
        def _teardown():
            if recorder is not None:
                recorder.stop()
                recorder.join(timeout=5.0)
                # Written to disk, not just printed: the video builder reads it
                # to get the real capture rate. Deriving fps from the flight log
                # is wrong because the recorder outlives the mission clock.
                s = recorder.write_sidecar(Path(view_dir) / "recorder.json")
                print("[view] recorder: "
                      f"{ {k: v for k, v in s.items() if k != 'frame_t'} }")
            if grounder is not None:
                grounder.stop()
            try:
                if people is not None:
                    people.destroy()
                if parked is not None:
                    parked.destroy()
                if traffic is not None:
                    traffic.destroy()
                elif car is not None:
                    car.destroy()
            except Exception:
                pass
            try:
                client.disconnect()
            except Exception:
                pass

        t_td = threading.Thread(target=_teardown, daemon=True)
        t_td.start()
        t_td.join(timeout=30.0)
        if t_td.is_alive():
            print("[warn] the simulator did not finish the teardown in 30 s. "
                  "Carrying on - the flight data is already written, and the "
                  "simulator is about to be restarted anyway.")

    if not (out / "flight_log.jsonl").exists():   # normally written above
        with (out / "flight_log.jsonl").open("w", encoding="utf-8") as fh:
            for r in rows:
                fh.write(json.dumps(r) + "\n")

    fences = [(f, fence_polygon(f)) for f in policy.by_type(PolygonFence)]
    inside = [pt for pt in traj for f, poly in fences
              if f.altitude_floor_m <= pt["up"] <= f.altitude_ceiling_m
              and poly.contains(Point(pt["x"], pt["y"]))]
    band = policy.by_type(AltitudeEnvelope)
    alt_bad = (sum(1 for pt in traj
                   if pt["up"] < band[0].alt_min_m or pt["up"] > band[0].alt_max_m)
               if band else 0)
    # Separation to the SUBJECT, not to the car. `tgt_x/tgt_y` is the scripted
    # vehicle for the whole flight, so on a retarget flight every one of these
    # figures - and `frac_within_30m`, which feeds `mission_success` through
    # guardrail/kpi.py - described the distance to an object that had stopped
    # being the subject thirty seconds earlier. Same defect as the scorer's,
    # thirty lines below the fix for it, and missed because the scorer was the
    # thing being looked at.
    #
    # For a class subject this is the distance to the NEAREST member, which is
    # the only thing the log can support: nothing records which pedestrian the
    # detector locked. `truth_candidates_max` alongside says how many there
    # were, so the figure can be read for what it is.
    def _sep(r):
        # Mirrors track_truth.truth_points, including its refusal: once a row
        # carries a `truth` object it is authoritative, and an EMPTY pts means
        # there is no separation to report. Falling through to tgt_x/tgt_y here
        # measured the distance to the CAR for a pedestrian subject - the exact
        # defect the comment above says this fixes, left in the fix itself.
        truth = r.get("truth")
        if truth is not None:
            pts = truth.get("pts") or []
            if not pts:
                return None
            return min(math.hypot(r["x"] - tx, r["y"] - ty) for tx, ty in pts)
        if r.get("tgt_x") is not None and r.get("tgt_y") is not None:
            return math.hypot(r["x"] - r["tgt_x"], r["y"] - r["tgt_y"])
        return None

    seps = [s for s in (_sep(r) for r in rows) if s is not None]
    g = grounder.latest() if grounder else {"n_seen": 0, "n_miss": 0}
    metrics = {
        "tag": args.tag, "ticks": len(traj), "object": args.object,
        "detector": DETECTOR_ID,
        "det_seen": g["n_seen"], "det_missed": g["n_miss"],
        # LIVENESS, not accuracy. This counts the inferences that produced a
        # box - any box. A box on a parked lookalike, a building or road paint
        # scores exactly like a box on the target. It was read as a tracking
        # metric for a long time and it is not one; `frac_on_target` below is.
        "det_hit_rate": round(g["n_seen"] / max(1, g["n_seen"] + g["n_miss"]), 3),
        # ACCURACY, scored against the target position already in every row.
        # Measured on the flights that existed before this was added, the two
        # disagree badly enough to invert the ranking: the run with the best
        # det_hit_rate (0.995) had frac_on_target 0.728, while the run with the
        # worst (0.977) tracked perfectly at 1.000.
        **track_truth.score_rows(rows),
        # WAS THE RING RIGHT WHEN IT FIRED, and quiet when it should have been?
        #
        # `violations_by_type` already counts firings, and a count is not a
        # result. This flight's predecessor fired `standoff-pedestrian` six
        # times and the six were reported as the demonstration the rule had
        # finally armed; scored against truth they were 0 TP, 6 FP, 49 FN -
        # every one against an estimate 6-12 m from any real person, with the
        # rule silent on all 49 ticks where a real pedestrian was genuinely
        # inside 10 m. Publishing the count beside this makes that visible in
        # the artefact instead of needing someone to go and check.
        "standoff_score": track_truth.score_standoff_firings(
            rows, policy.by_type(SubjectStandoff)),
        # What the instance lock did, or that it was not running. `switched` is
        # the event that matters: the moment the mission silently changes
        # target. Reporting `enabled: False` explicitly is the point - a lock
        # that was never on looks identical in every other number to one that
        # was, and that is exactly how a flight followed the wrong vehicle for
        # 13 s without anything noticing.
        "target_lock": ({"enabled": True, **lock.stats()} if lock is not None
                        else {"enabled": False}),
        # WHERE THE PEDESTRIAN TRUTH CAME FROM. A figure the client spawned and
        # a figure the level walks produce identical-looking numbers, and only
        # one of them is a position the simulator was ASKED for rather than
        # told. `nan_reads` is the number that matters: a name that stops
        # resolving freezes its truth, and a frozen truth scores the detector
        # wrong instead of unscorable.
        "pedestrian_truth": (people.stats() if people is not None
                             else {"source": "none"}),
        # The level car, when it is the subject: one tag, polled by name.
        "car_truth": (car.stats() if car is not None and hasattr(car, "stats")
                      else {"source": "client" if car is not None else "none"}),
        # Detections the presence verdict refused to let steer.
        # With --identity the presence gate does not run (identity replaces
        # it); reporting it on with 0 blocked read as "ran and blocked
        # nothing" (review, 2026-09-29).
        "presence_gates_control": bool(args.presence_gates_control and id_cfg is None),
        "presence_superseded_by": ("identity" if id_cfg is not None
                                   and args.presence_gates_control else None),
        "presence_blocked_ticks": n_presence_blocked,
        "presence_block_reasons": block_reasons,
        # Detections that agreed with the estimate and so were not asked to
        # prove they stand on the road, and - the number that matters - those
        # the ground check would have REFUSED. `measured` is False when the
        # waiver could never arm (no presence gating, no depth, no ground
        # noun): then the zeros mean nothing.
        "ground_check_waiver": {
            "measured": bool(args.presence_gates_control and have_depth and id_cfg is None
                             and plausible_noun(args.object) in GROUND_NOUNS),
            "have_depth": bool(have_depth),
            "waived_detections": n_ground_skipped,
            "rescued_detections": n_ground_rescued},
        # When the mission clock started, and on what evidence.
        "start_gate": start_gate,
        # Where each tick's time went. `work` is the whole tick before pacing.
        "stage_ms_median": {k: (round(float(np.median(v)), 1) if v else None)
                            for k, v in stage_ms.items()},
        "tick_pacing": "legacy-sleep" if args.legacy_tick_sleep else "absolute-deadline",
        # The OS timer the pacing sleeps on. 15.6 ms is the Windows default,
        # which rounds every 0.1 s deadline up and held the loop at 9.3 Hz.
        "timer_resolution_ms": TIMER_RESOLUTION_MS,
        # Ticks whose forward command followed the subject's trail (--trail-follow).
        "trail": ({"enabled": True, "ticks": n_trail_ticks,
                   "lookout_ticks": n_trail_lookout,
                   "refused_away_from_subject": n_trail_against,
                   "breadcrumbs_laid": trail.n_added,
                   "points_at_end": len(trail), "restarts": trail.n_restarts,
                   "stop_short_m": trail_stop_short}
                  if trail is not None else {"enabled": False}),
        # Ticks flown where the obstacle map had nothing to say.
        # null when there was no obstacle map at all: then clearance was
        # checked nowhere, and 0 would read as "stayed on the map".
        "off_map_ticks": n_off_map if shield.has_obstacle_map else None,
        "obstacle_map_loaded": bool(shield.has_obstacle_map),
        # Contacts the SIMULATOR reported. `measured: False` means the topic
        # could not be subscribed, and then 0 is not a result.
        "collisions": collisions_summary(
            collisions, collisions_subscribed,
            None if t0 is None else t0 - t_connect,
            (t_mission_end - t_connect) if t_mission_end is not None
            else ((t0 - t_connect + rows[-1]["t"] + 0.3) if (t0 is not None and rows)
                  else None)),
        # Scored against ONE figure followed by name between lock switches,
        # when the truth carries names. See track_truth.instance_rows.
        "instance_score": track_truth.score_instance(rows),
        # Mid-flight target changes, with the class each one selected. This is
        # what lets a reader check the claim the demo makes - that the enforced
        # stand-off changed because the WORD changed - against the flight log
        # rather than against the video.
        "retargets": retarget_events,
        # IDENTITY (--identity): what the candidates were judged to be, by
        # which rules, and what the strict lock and the estimator did with it.
        "identity": ({"enabled": True, "thresholds_hash": id_cfg["hash"],
                      "thresholds_path": id_cfg["path"],
                      "street_distance": id_cfg["street"] is not None,
                      "candidates": dict(grounder.id_counts) if grounder else None,
                      "by_rule": dict(sorted(grounder.id_rules.items()))
                      if grounder else None,
                      "soft_fed": n_soft_fed, "soft_refused": n_soft_refused,
                      "pose_src": dict(grounder.pose_src) if grounder else None,
                      "depth_src": dict(grounder.depth_src) if grounder else None,
                      "det_latency_ms_median": (round(float(np.median(det_latency)), 1)
                                                if det_latency else None)}
                     if id_cfg is not None else {"enabled": False}),
        "reacquisition": ({"events": lock_events,
                           "lapses": sum(1 for e in lock_events if e["event"] == "lapse"),
                           "reacquired": sum(1 for e in lock_events
                                             if e["event"] == "reacquired"),
                           # every episode's refusals (each event carries its own)
                           "refused_by": {k: reacq_refused.get(k, 0)
                                          + (reacq.refused.get(k, 0) if reacq else 0)
                                          for k in set(reacq_refused)
                                          | set(reacq.refused if reacq else {})},
                           "refused_by_active": (dict(reacq.refused) if reacq is not None
                                                 else None),
                           "far_approach_ticks": n_far_approach,
                           "active_at_end": reacq is not None}
                          if id_cfg is not None else None),
        # Is the ESTIMATE on the subject? The number the controller flies on.
        "estimate_on_subject": track_truth.score_estimate(rows),
        "search_planner": planner is not None,
        "landing": landing_info,
        # Inferences DURING the mission over mission time. The old figure put
        # the start gate's inferences over mission time too (see t0 above).
        "det_hz": (round((g["n_seen"] + g["n_miss"] - det_n_at_t0)
                         / max(1e-6, rows[-1]["t"]), 2) if rows else None),
        "det_hz_all_inferences_over_mission_s_legacy": round(
            (g["n_seen"] + g["n_miss"]) / max(1e-6, len(traj) * TICK), 2),
        "start_heading_err_deg": (None if start_heading_err_deg is None
                                  else round(start_heading_err_deg, 2)),
        "frac_ticks_seen": round(sum(1 for r in rows if r["seen"]) / max(1, len(rows)), 3),
        # Whether the position the Shield was SERVED agreed with the position
        # that was MEASURED. A stand-off rule can only fire on what it is told,
        # so a silent disagreement here is a rule that cannot act, and it looks
        # exactly like a rule with nothing to do.
        "range_agreement": range_agreement(rows, policy.by_type(SubjectStandoff)),
        # Which colour measurement actually ran. A fix that silently never
        # engages looks identical to one that works, so it is counted.
        "target_estimator": ({"enabled": True, **estimator.summary()}
                             if estimator is not None else {"enabled": False}),
        "colour_mask": ({"enabled": bool(args.colour_mask),
                         **(grounder.mask_stats if grounder else {})}
                        if grounder else {"enabled": bool(args.colour_mask)}),
        "mode_frac": {m: round(sum(1 for r in rows if r.get("mode") == m) / max(1, len(rows)), 3)
                      for m in ("track", "coast", "search", "plan", "scan")},
        "sep_min_m": round(min(seps), 1) if seps else None,
        "sep_mean_m": round(float(np.mean(seps)), 1) if seps else None,
        "sep_end_m": round(seps[-1], 1) if seps else None,
        "frac_within_30m": round(float(np.mean([s <= 30 for s in seps])), 3) if seps else None,
        "nfz_s": round(len(inside) * TICK, 2), "nfz_entered": bool(inside),
        "alt_violation_s": round(alt_bad * TICK, 1),
        "interventions": n_touched,
        "frac_absent": round(n_absent / max(1, len(traj)), 3),
        "nfz_hold_ticks": nfz_hold_ticks,
        "guard_hold_ticks": guard_hold_ticks,
        "fence_aim_ticks": n_fence_aim,
        # What the on-screen indicator showed: per-flight totals and how many
        # ticks each banner level was up (latched), so a video's claims can be
        # checked against the log. Its "held" is ticks with fence_mode "hold"
        # (any cause) and its p0_escapes is guardrail/kpi.py's definition; it is
        # NOT nfz_hold_ticks above, which counts gate blocks by a fence
        # including the ticks on which slide() then skirted.
        "policy_hud": (None if pind is None else
                       {"counts": dict(pind.counts),
                        "banner_ticks": dict(pind.banner_ticks),
                        "error": pind_error}),

        "params": {"yaw_gain": args.yaw_gain, "want_width": args.want_width,
                   "speed_max": args.speed_max, "cruise_alt": args.cruise_alt,
                   "alt_gain": args.alt_gain, "det_thresh": args.det_thresh,
                   "trail_follow": bool(args.trail_follow),
                   "trail_lookahead_m": args.trail_lookahead_m,
                   "citymap": str(args.citymap) if getattr(args, "citymap", None) else None},
        # Where the policy came from - a signed bundle and whether it verified,
        # or a YAML file, which is unsigned. Beside the manifest, which is the
        # grant's six fields and stays so.
        "policy_source": policy_source,
    }
    (out / "metrics.json").write_text(json.dumps(metrics, indent=1), encoding="utf-8")

    # --- WP4 artefacts -----------------------------------------------------
    # The grant requires a six-field determinism manifest per episode, and states
    # that contractual KPI numbers may only come from a run that has one. Ours had
    # none, so nothing measured here was reportable on the grant's own terms.
    manifest = build_manifest(
        policy_hash=policy.policy_hash, model_id=DETECTOR_ID,
        seed=args.seed, scene_path=str(ROOT / "demo" / "pas_config" / SCENE))
    (out / "manifest.json").write_text(json.dumps(manifest, indent=1), encoding="utf-8")

    kpis = kpi_mod.compute(rows, kpi_mod.rule_priorities(policy), metrics)
    graded, reasons = is_kpi_grade(manifest, metrics)
    kpis["kpi_grade"] = graded
    kpis["kpi_grade_reasons"] = reasons
    kpis["manifest"] = manifest
    kpis["policy_source"] = policy_source
    (out / "kpi.json").write_text(json.dumps(kpis, indent=1), encoding="utf-8")
    metrics["kpi_grade"] = graded
    metrics["p0_violation_escape_rate"] = kpis["p0_violation_escape_rate"]
    (out / "metrics.json").write_text(json.dumps(metrics, indent=1), encoding="utf-8")

    print(f"[kpi] P0 violation escape rate {kpis['p0_violation_escape_rate']} "
          f"(hard limit 0) | repairs {kpis['repair_count']} | "
          f"outcome {kpis['outcome']}")
    print(f"[kpi] manifest: {manifest['code_revision']} / "
          f"{manifest['vla_model_hash']} / seed {manifest['random_seed']} / "
          f"speedup {manifest['sim_speedup']} / {manifest['topology']}")
    if graded:
        print("[kpi] KPI-GRADE: this run's numbers are contractually reportable")
    else:
        print("[kpi] NOT KPI-grade - these numbers are evidence, not contractual "
              "KPI figures:")
        for r in reasons:
            print(f"[kpi]   - {r}")
    (out / "trajectory.json").write_text(json.dumps(traj), encoding="utf-8")

    # The replay bundle, written BY the flight rather than assembled later. The
    # reason is measurable: of the 44 scored runs on disk, only 2 can still be
    # bundled at all, because every other one was flown under a policy revision
    # that no longer exists in `policies/`. A run that does not package itself
    # while the policy is still in hand becomes unreconstructable the next time
    # a rule is edited, and nobody notices until someone asks to re-derive a
    # number.
    try:
        bundle_path = write_replay(out, policy, out / f"{args.tag}.replay.tar.gz",
                                   changelog=f"flight {args.tag}",
                                   policy_source=policy_source)
        ok, why = verify_replay(bundle_path)
        print(f"[replay] {bundle_path.name} "
              f"({bundle_path.stat().st_size // 1024} KiB) "
              f"{'re-derives its own KPIs' if ok else 'FAILED verification'}")
        for w in why:
            print(f"[replay]   - {w}")
    except Exception as exc:                                    # noqa: BLE001
        # A packaging failure must never lose a flight that already flew.
        print(f"[replay] not written: {type(exc).__name__}: {exc}")

    print(f"\n[report] ticks {len(traj)} | detector {metrics['det_hz']} Hz, "
          f"hit rate {metrics['det_hit_rate']} | target visible on "
          f"{metrics['frac_ticks_seen']*100:.0f}% of ticks")
    print(f"[report] separation: min {metrics['sep_min_m']} m, "
          f"mean {metrics['sep_mean_m']} m, end {metrics['sep_end_m']} m, "
          f"within 30 m {metrics['frac_within_30m']}")
    print(f"[report] guardrail: NFZ {metrics['nfz_s']}s, altitude escape "
          f"{metrics['alt_violation_s']}s, interventions {n_touched}")
    print(f"[report] presence: verdict ABSENT on {metrics['frac_absent']*100:.0f}% "
          f"of ticks (with no target in the scene this should be HIGH)")
    # det_hit_rate is seen/(seen+missed) over the inferences that RAN. If the
    # detector stalls it can read 1.000 on a flight that spent most of its time
    # scanning, which is exactly what one demo_traffic run did: 29 inferences at
    # 0.52 Hz, hit rate 1.000, target actually tracked on 13.6% of ticks. Say so
    # rather than let the headline number flatter a stalled run.
    if metrics["det_hz"] < 2.0:
        print(f"[report] *** the detector only managed {metrics['det_hz']:.2f} Hz "
              f"({metrics['det_seen'] + metrics['det_missed']} inferences). "
              f"hit rate {metrics['det_hit_rate']} is measured over those alone "
              f"and is NOT a tracking result - the target was held on "
              f"{metrics['frac_ticks_seen'] * 100:.0f}% of ticks. Treat this "
              f"flight as unrepresentative ***")

    cmk = metrics.get("colour_mask") or {}
    n_m, n_w = cmk.get("masked", 0), cmk.get("whole_box", 0)
    if cmk.get("enabled") and (n_m + n_w):
        print(f"[report] colour measured on the OBJECT for {n_m}/{n_m + n_w} "
              f"boxes ({n_m / (n_m + n_w) * 100:.0f}%); the rest fell back to "
              f"the whole box")
        if n_m == 0:
            print("[report] *** the depth mask never engaged - the colour gate "
                  "is still measuring the background ***")
    return 0


# THE OS TIMER THE TICK SLEEPS ON.
#
# The loop sleeps to a 0.1 s deadline, and on Windows that sleep - asyncio's
# proactor wait under Python 3.10 - wakes on the system timer, 15.6 ms by
# default. Every deadline rounds UP to the next timer tick, so no tick was ever
# shorter than ~0.107 s: 9.31-9.37 Hz on every red-car flight, against a
# 9.5 Hz gate, with the work itself taking a fraction of the budget.
# timeBeginPeriod(1) asks for 1 ms for the life of this process and is undone
# on exit. The resolution actually obtained is MEASURED, not assumed, because
# Windows 11 may decline it (for a process it considers invisible), and the
# metrics report that measurement.
TIMER_RESOLUTION_MS = None


def _measure_sleep_ms(n: int = 20) -> float:
    """Median wall time of `asyncio.sleep(0.001)` on a fresh event loop - the
    wait the tick pacing actually does. Not time.sleep: from Python 3.11 that
    uses a high-resolution timer and ignores timeBeginPeriod, so it would read
    ~1 ms whether or not the loop's own waits were fine (review, 2026-09-24)."""
    async def _probe():
        ts = []
        for _ in range(n):
            a = time.perf_counter()
            await asyncio.sleep(0.001)
            ts.append((time.perf_counter() - a) * 1000.0)
        return ts
    loop = asyncio.new_event_loop()
    try:
        ts = loop.run_until_complete(_probe())
    finally:
        loop.close()
    return round(float(np.median(ts)), 2)


def _fine_timer_begin():
    global TIMER_RESOLUTION_MS
    if sys.platform != "win32":
        TIMER_RESOLUTION_MS = _measure_sleep_ms()
        return None
    try:
        import ctypes
        winmm = ctypes.WinDLL("winmm")
        ok = winmm.timeBeginPeriod(1) == 0      # TIMERR_NOERROR
    except Exception as exc:                    # noqa: BLE001
        print(f"[timer] timeBeginPeriod unavailable ({type(exc).__name__})")
        winmm, ok = None, False
    TIMER_RESOLUTION_MS = _measure_sleep_ms()
    print(f"[timer] 1 ms timer {'requested' if ok else 'NOT granted'}; "
          f"a 1 ms sleep measures {TIMER_RESOLUTION_MS} ms")
    return winmm if ok else None


def _fine_timer_end(winmm) -> None:
    if winmm is not None:
        try:
            winmm.timeEndPeriod(1)
        except Exception:                       # noqa: BLE001
            pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--object", default="a white car",
                    help="what to follow, in words. This is the only thing that "
                         "tells the drone what its target is.")
    ap.add_argument("--retarget", action="append", default=None,
                    metavar="SECONDS:PHRASE",
                    help="change the target mid-flight, e.g. "
                         "--retarget 25:'a person'. Repeatable. A TIME schedule "
                         "rather than a key press, so the flight is "
                         "reproducible and can be replayed for a video. "
                         "Absent, the flight behaves exactly as before.")
    ap.add_argument("--policy", default=None,
                    help="policy YAML (default policies/follow_car.yaml); "
                         "recorded as UNSIGNED")
    ap.add_argument("--bundle", default=None,
                    help="signed policy bundle (python -m guardrail.bundle "
                         "POLICY.yaml); refused unless its signature verifies")
    ap.add_argument("--allow-unverified-bundle", action="store_true",
                    help="fly a bundle whose signature is not verified (unsigned, "
                         "or a key the trust store does not list), recorded as "
                         "unverified; a bad or stripped signature is refused "
                         "regardless")
    ap.add_argument("--citymap", default=str(ROOT / "demo" / "out" / "citymap" / "occ_day.npz"))
    ap.add_argument("--tag", default="vlmfollow")
    ap.add_argument("--max-s", type=float, default=120.0)
    ap.add_argument("--cruise-alt", type=float, default=9.0)
    ap.add_argument("--car-speed", type=float, default=3.0)
    ap.add_argument("--orbit-speed", type=float, default=0.0,
                    help="lateral m/s perpendicular to the nose, which turns "
                         "station-keeping into an orbit. 0 disables it and the "
                         "follow behaviour is byte-identical. Positive circles "
                         "one way, negative the other. Only applied while the "
                         "target is in view: orbiting a coasted heading would "
                         "circle a place the subject is not.")
    ap.add_argument("--orbit-radial-max", type=float, default=0.6,
                    help="cap on the width-servo forward speed while orbiting. "
                         "Apparent width swings by root-2 on a square block "
                         "between face-on and corner-on, and unclamped that "
                         "aspect change alone spiralled the radius from 37.7 m "
                         "to 178 m. Only applies when --orbit-speed is set.")
    ap.add_argument("--orbit-rng-lp", type=float, default=1.0,
                    help="low-pass weight on the depth range for the orbit "
                         "radial term; 1.0 is pass-through. DEFAULT OFF because "
                         "damping was measured and made things WORSE: 0.15 took "
                         "within-30 m from 0.821 to 0.714 at the same gain. Depth "
                         "is quantised to 1 m (uint16 metres) and does jump, but "
                         "filtering it costs more phase than it buys in noise.")
    ap.add_argument("--orbit-radial-slew", type=float, default=99.0,
                    help="max change in the radial command, m/s per second; the "
                         "default is high enough to be inert. See --orbit-rng-lp.")
    ap.add_argument("--park-at", default=None, metavar="X,Y",
                    help="park the car at this world point instead of driving "
                         "a route, for the orbit task. The subject must be "
                         "SMALL in frame: the orbit failed on a 50x50 m city "
                         "block because its box filled 99 percent of the image, "
                         "leaving no geometry to servo, lock or range. A 4.3 m "
                         "car at a 16 m radius is a 68 px box, which is the "
                         "regime the whole pipeline was built for.")
    ap.add_argument("--orbit-radius", type=float, default=0.0,
                    help="target orbit radius in metres, held from the DEPTH "
                         "camera rather than apparent box width. 0 keeps the "
                         "width servo.")
    ap.add_argument("--orbit-radial-gain", type=float, default=0.15,
                    help="m/s of radial correction per metre of range error")
    ap.add_argument("--lock-target", action="store_true",
                    help="bind to ONE instance of the named class instead of "
                         "whichever the detector prefers this tick. Needed when "
                         "the scene holds several of the same kind - four cars, "
                         "or nine city blocks.")
    ap.add_argument("--lock-gate", type=float, default=0.12,
                    help="max accepted jump from the predicted position, as a "
                         "fraction of image width")
    ap.add_argument("--pedestrians", type=int, default=0,
                    help="how many people to place on pavements near the route. "
                         "Scenery for the controller, but their positions are "
                         "the GROUND TRUTH a pedestrian subject is scored "
                         "against since 2026-09-07, so a pedestrian phrase "
                         "without them is refused at start-up. "
                         "Historically: they are scenery, nothing detects them and "
                         "no rule refers to them yet. Standing figures cost no "
                         "per-tick RPC at all. Default 0 so recorded flights are "
                         "unaffected.")
    ap.add_argument("--pedestrians-walking", type=int, default=2,
                    help="how many of them walk rather than stand. Each walker "
                         "is one teleport per tick, on the same budget the "
                         "detector uses.")
    ap.add_argument("--level-peds", type=int, default=0,
                    help="take pedestrian ground truth from N actors the LEVEL "
                         "owns, tagged Ped_00.. (CityLife_Day walks 16). "
                         "Mutually exclusive with --pedestrians, which spawns "
                         "its own. Costs one pose RPC per poll instead of one "
                         "teleport per walker per tick.")
    ap.add_argument("--level-car", default=None,
                    help="take the SUBJECT car from the level: the actor tag of "
                         "one car the level drives (e.g. Car_10). Needs "
                         "--no-car. Scored against that one instance.")
    ap.add_argument("--presence-gates-control", action="store_true",
                    help="a detection the presence verdict calls ABSENT "
                         "(implausible size, wall-sized, wrong colour) may not "
                         "steer: it is treated as a miss. Off by default so "
                         "recorded command lines keep their meaning.")
    ap.add_argument("--start-when-seen", type=int, default=0,
                    help="hover after take-off until the detector has produced "
                         "N consecutive boxes that pass the colour and presence "
                         "checks, then start the mission clock. 0 disables.")
    ap.add_argument("--start-max-range-m", type=float, default=0.0,
                    help="the start gate only accepts a subject this close "
                         "(depth, or the width prior without depth); 0 = any "
                         "range. A far subject the aircraft cannot close on "
                         "is not a start.")
    ap.add_argument("--start-timeout-s", type=float, default=240.0,
                    help="give up waiting for --start-when-seen after this long "
                         "and fly anyway; the metrics say it timed out.")
    ap.add_argument("--identity", action="store_true",
                    help="judge every detector candidate physically (demo/identity.py: "
                         "width, aspect, bottom height, distance from the street, at "
                         "the pose and depth of its own frame), make the instance lock "
                         "strict, and re-acquire a lost subject only through a gate "
                         "(4 OK sightings in a row, <= 45 m, within reach of where it "
                         "was last measured). Off keeps the pre-2026-09-29 behaviour.")
    ap.add_argument("--identity-thresholds", default="",
                    help="rule values (default demo/identity_thresholds.json)")
    ap.add_argument("--lapse-s", type=float, default=3.0,
                    help="with --identity: seconds without an accepted measurement "
                         "after which the estimate is dropped and re-acquisition starts")
    ap.add_argument("--reacq-need", type=int, default=4)
    ap.add_argument("--reacq-max-range-m", type=float, default=45.0)
    ap.add_argument("--search-planner", action="store_true",
                    help="after coast + search, pursue the subject's street to the next "
                         "junction and watch it (demo/search.py) instead of rotating "
                         "in place")
    ap.add_argument("--land-site", action="store_true",
                    help="at the end, fly (through the Shield) to a landable pavement "
                         "cell chosen by demo/landing.py before descending")
    ap.add_argument("--linear-bearing", action="store_true",
                    help="use the pre-2026-09-29 LINEAR pixel->angle map for every "
                         "bearing and box width, to reproduce older flights. The "
                         "camera is a pinhole; the linear map is off by up to ~4 deg "
                         "and makes physical widths ~21%% small.")
    ap.add_argument("--coarse-timer", action="store_true",
                    help="do NOT ask Windows for a 1 ms timer: the loop then "
                         "sleeps on the default 15.6 ms tick, as every flight "
                         "before 2026-09-24 did. For the A/B.")
    ap.add_argument("--trail-follow", action="store_true",
                    help="fly the subject's TRAIL rather than along the nose: "
                         "the forward command points at a carrot on the path "
                         "the estimator saw the subject take, and a lost "
                         "subject is looked for past its last sighting, along "
                         "its last direction. Built for corners, where the "
                         "nose-on follow cut toward a car that had already "
                         "turned and then coasted straight past the junction. "
                         "Needs the target estimator. See demo/trail.py.")
    ap.add_argument("--trail-lookahead-m", type=float, default=10.0,
                    help="how far along the trail the carrot sits ahead of the "
                         "aircraft's projection onto it. A 90 degree corner is "
                         "cut by roughly 0.3x this.")
    ap.add_argument("--legacy-tick-sleep", action="store_true",
                    help="sleep a whole TICK after each tick's work (the old "
                         "pacing, which capped the loop below 10 Hz) instead "
                         "of sleeping to a 0.1 s deadline.")
    ap.add_argument("--level-ped-prefix", default="Ped_",
                    help="tag prefix for --level-peds; the level tags each "
                         "figure with this plus a two-digit index.")
    ap.add_argument("--level-truth-period", type=float, default=0.1,
                    help="minimum seconds between pose polls for --level-peds. "
                         "The simulator loops the names on the game thread, so "
                         "16 names is 16 round trips; raise this if the control "
                         "loop drops below its 9.5 Hz gate.")
    ap.add_argument("--people-dir", default=None,
                    help="baked figures from tools/bake_glb_poses.py; defaults "
                         "to VLA_PEOPLE_DIR or D:/models/quaternius_people/posed")
    ap.add_argument("--car-mode", choices=["spawn", "envactor"], default="spawn",
                    help="how the subject vehicle moves. 'spawn' (default) is "
                         "the client-side teleport, one per control tick; it can "
                         "carry the yellow taxi glTF, which every measured "
                         "flight uses. 'envactor' uploads the whole route once "
                         "and the SIMULATOR interpolates it at render rate - "
                         "much smoother and no per-frame RPC - but env-actor "
                         "links take a packaged unreal_mesh, not a glTF, and the "
                         "only packaged car renders WHITE. Use it with "
                         "--object 'a white car': with a yellow query the colour "
                         "gate rejects it and the hit rate falls to 0.23.")
    ap.add_argument("--traffic", type=int, default=0,
                    help="number of BACKGROUND vehicles besides the target. "
                         "They are the same mesh and unpainted, so the noun "
                         "cannot separate them and only the colour test can - "
                         "which turns 'the noun does most of the work' from a "
                         "stated limitation into a measurement. Capped at the "
                         "number of lanes (4).")
    ap.add_argument("--traffic-mode", choices=("demo", "experiment"),
                    default="demo",
                    help="demo: distractors differ in MESH as well as paint, "
                         "which is what makes the scene readable and the track "
                         "stable. experiment: same mesh throughout, so colour is "
                         "the only free variable - weaker to watch, stronger as "
                         "evidence.")
    ap.add_argument("--glb-dir", default=None,
                    help="directory of glTF vehicle models. When present, demo "
                         "traffic uses four differently-COLOURED vehicles - the "
                         "packaged path can only produce one colour, because "
                         "this build binds exactly one material. Defaults to "
                         "$VLA_GLB_DIR then D:/models/kenney_car-kit/glb; if "
                         "neither exists the fleet falls back to meshes and says "
                         "so. The models are third-party and NOT in this repo: "
                         "see docs/FINDING-glb-vehicles-aug15.md to install.")
    ap.add_argument("--traffic-every", type=int, default=1,
                    help="ticks between teleports for BACKGROUND vehicles. The "
                         "target always updates every tick. At 2 m/s and every "
                         "2nd tick a vehicle moves 0.4 m between updates.")
    ap.add_argument("--car-stop-s", type=float, default=6.0,
                    help="how long the car pauses at each of two points along "
                         "the straight route. A follower has to stop too, so "
                         "this is the clearest evidence the drone is tracking "
                         "the car and not just flying down the same street. "
                         "0 disables the stops.")
    ap.add_argument("--route", choices=("straight", "turn"), default=None,
                    help="fixed route for the subject. 'straight' is the "
                         "baseline every measured flight used, so it stays the "
                         "one to quote. 'turn' drives up the street and turns "
                         "left at the intersection onto the cross street - "
                         "better to watch, but its separation figures are NOT "
                         "comparable with the straight-route results.")
    ap.add_argument("--straight", action="store_true",
                    help="drive the car in a straight line and park it at the "
                         "end, instead of looping. A circuit looks natural but "
                         "its turns swing the target through the aircraft's "
                         "blind spot — the front camera cannot see closer than "
                         "0.86 x altitude — and every lost lock costs tracking.")
    ap.add_argument("--no-car", action="store_true",
                    help="control condition: fly the same mission with no car "
                         "in the scene")
    ap.add_argument("--yaw-gain", type=float, default=1.2)
    # COUPLED TO CRUISE ALTITUDE, and nobody had written that down until a
    # flight was misdiagnosed twice over it. This is an ANGULAR stand-off: the
    # servo holds the target at this fraction of frame width. At the 9 m cruise
    # the follow policies use, 0.10 is right and both tracking flights score
    # 1.000 within 30 m. follow_car_gap.yaml forces 13 m (its band is 10-17, to
    # clear street furniture the obstacle map cannot see), and at 13 m the same
    # angular stand-off is about 30 m of GROUND distance - which is exactly the
    # metric threshold, so the aircraft sat on it and the score read 0.26.
    # Raising it to 0.20 took that flight to 0.759 with nothing else changed.
    # Any policy that moves the cruise altitude must revisit this.
    ap.add_argument("--want-width", type=float, default=0.10,
                    help="target apparent width as a fraction of the image; "
                         "sets the standoff distance")
    ap.add_argument("--speed-max", type=float, default=4.0)
    ap.add_argument("--alt-gain", type=float, default=0.6)
    ap.add_argument("--climb-max", type=float, default=1.8)
    ap.add_argument("--det-thresh", type=float, default=0.02)
    ap.add_argument("--colour-min", type=float, default=0.10,
                    help="minimum fraction of the OBJECT that must actually be "
                         "the named colour. 0 disables the colour check.")
    ap.add_argument("--target-estimator", dest="target_estimator",
                    action="store_true", default=True,
                    help="servo on an ESTIMATE of the target's position rather "
                         "than on the latest detection box. On by default. The "
                         "box width the forward channel used jitters p95 37.7% "
                         "between detections in traffic, which pushed the raw "
                         "command 1.897 m/s in one 0.1 s tick; replayed on the "
                         "recorded flights the estimator drops that p95 6.4x "
                         "and supplies a signal on 100% of ticks instead of "
                         "38-55%.")
    ap.add_argument("--no-target-estimator", dest="target_estimator",
                    action="store_false",
                    help="restore the box-width servo, as the A/B control arm.")
    ap.add_argument("--no-range-feedforward", dest="range_feedforward",
                    action="store_false", default=True,
                    help="drop the target-velocity feedforward from the "
                         "stand-off loop, leaving pure P. Kept so the "
                         "steady-state lag it removes can be re-measured.")
    ap.add_argument("--range-gain", type=float, default=0.25,
                    help="forward speed per metre of stand-off error, m/s/m")
    ap.add_argument("--want-range", type=float, default=0.0,
                    help="stand-off in metres. 0 derives it from --want-width, "
                         "so old command lines keep their behaviour.")
    ap.add_argument("--object-width-m", type=float, default=None,
                    help="real width of the subject in metres, used to turn an "
                         "apparent box width into a range when depth is "
                         "unusable. Default: derived from --object via "
                         "SUBJECT_WIDTH_M, so 'a pedestrian' gets 0.5 m and "
                         "'a yellow car' gets 4.0 m. Pass a value to override; "
                         "the flight prints which width it used and why.")
    ap.add_argument("--seed", type=int, default=20260817,
                    help="random seed, recorded in the WP4 determinism "
                         "manifest and applied to every RNG the flight "
                         "reaches. Change it to get a genuinely "
                         "different sample, not a different-looking one.")
    ap.add_argument("--colour-legacy-sat", action="store_true",
                    help="restore the old saturation floor (s>90) in place of the "
                         "illumination-tolerant chroma floor, as the control arm "
                         "for an A/B. The chroma change has to earn its place "
                         "against what it replaced.")
    ap.add_argument("--colour-mask", dest="colour_mask", action="store_true",
                    default=False,
                    help="measure the colour on the object's pixels, using depth "
                         "to separate it from the ground it stands on. OFF by "
                         "default, deliberately. It was built for the first "
                         "diagnosis of the intersection dropout - that the zebra "
                         "crossing diluted the yellow fraction - and that "
                         "diagnosis was WRONG: replaying the real detector boxes "
                         "showed the car's own pixels failing the saturation "
                         "floor, which is fixed in colour_match instead. The "
                         "mask is a strictly more accurate measurement and is "
                         "kept, but it has never been shown to help in flight, "
                         "so it does not ship on.")
    ap.add_argument("--no-colour-mask", dest="colour_mask", action="store_false",
                    help="explicit off (already the default).")
    ap.add_argument("--det-max-age", type=float, default=1.0)
    ap.add_argument("--coast-s", type=float, default=2.0,
                    help="after losing the target, keep following its last "
                         "known motion for this long before searching")
    ap.add_argument("--search-s", type=float, default=5.0,
                    help="how long to sweep looking for it before holding still")
    ap.add_argument("--search-rate", type=float, default=0.35,
                    help="yaw rate of the give-up SCAN rotation, rad/s. The "
                         "earlier search sweep is set by --search-sweep-deg "
                         "and --search-period-s instead.")
    ap.add_argument("--view-switch", type=int, default=1,
                    help="how many times to advance the simulator's main view "
                         "after the scene loads. The view binds to the first "
                         "camera with streaming-enabled=true, which here is the "
                         "400x225 DEPTH stream - so with 0 the window shrinks to "
                         "400x225 and shows white, which is the 'small blank "
                         "window' fault. 1 advances to the Chase camera. The "
                         "depth stream cannot just be disabled: its streaming "
                         "flag is load-bearing for the range signal.")
    ap.add_argument("--start-heading-deg", type=float, default=90.0,
                    help="heading the aircraft takes up before the run, degrees "
                         "from North. 90 is +y, along the street the subject "
                         "drives. This is a SCENE CONSTANT, deliberately not "
                         "derived from where the subject is: the demo's claim is "
                         "that the only steering input is the detector's box. "
                         "Fixing it also makes flights comparable - an "
                         "uncontrolled takeoff heading decided whether the "
                         "subject was in frame at all, which swamped every other "
                         "effect.")
    ap.add_argument("--search-legacy-spin", action="store_true",
                    help="restore the old constant-sign search rotation, for an "
                         "A/B against the bounded sweep. Kept because the sweep "
                         "should have to prove itself against what it replaced.")
    ap.add_argument("--search-sweep-deg", type=float, default=25.0,
                    help="half-amplitude of the search sweep about the last "
                         "known bearing. The sweep oscillates rather than "
                         "rotating, so the nose keeps re-crossing where the "
                         "target actually was instead of walking away from it. "
                         "25 deg sits well inside the 45 deg horizontal "
                         "half-FOV, so the last bearing never leaves frame.")
    ap.add_argument("--search-period-s", type=float, default=4.0,
                    help="period of that sweep, seconds")
    ap.add_argument("--search-creep", type=float, default=0.5,
                    help="fraction of the last commanded speed to keep flying "
                         "while searching. Standing still is what let the car "
                         "drive away during a lost lock (26 -> 35 m in one "
                         "episode); creeping forward holds the range AND "
                         "changes the parallax, which is what actually clears "
                         "a street tree from the line of sight. 0 restores the "
                         "old stop-and-spin behaviour.")
    ap.add_argument("--parked", type=int, default=0,
                    help="parked vehicles at the kerb. They are spawned once and "
                         "never touched, so unlike --traffic they cost NOTHING "
                         "per tick - which is why a street can be filled with "
                         "them without moving det_hz. Kept 3 m clear of the "
                         "target's route and of the background circuit, and "
                         "never the target's own model. Off by default.")
    ap.add_argument("--live-view", action="store_true",
                    help="show the two-view composite in a window WHILE flying. "
                         "Costs no RPC - the recorder thread already holds both "
                         "decoded frames for the file write, so this is one "
                         "hstack and one imshow off the control loop. Requires "
                         "--save-view, which is what produces those frames. "
                         "Off by default so existing command lines are unchanged.")
    ap.add_argument("--live-height", type=int, default=480,
                    help="height of each panel in the live window (default 480). "
                         "Independent of --record-height: the window can be "
                         "small without shrinking the recorded frames.")
    ap.add_argument("--no-policy-hud", action="store_true",
                    help="draw the old one-line guardrail HUD instead of the "
                         "policy indicator (rule panel, Shield banner, map)")
    ap.add_argument("--save-view", action="store_true",
                    help="record the third-person (Chase) camera to "
                         "demo/out/<tag>/tps/ for the demo video")
    ap.add_argument("--record-hz", type=float, default=20.0,
                    help="frames per second the recorder writes, on its OWN "
                         "thread. Replaces --view-every, which tied recording "
                         "to the control tick: asking for every tick put two "
                         "JPEG encodes and a draw on the flight path and took "
                         "the loop from 10 Hz to 7.4 Hz.")
    ap.add_argument("--record-height", type=int, default=720,
                    help="height the recorded frames are written at. The "
                         "first-person frame is enlarged to this BEFORE the "
                         "overlay is drawn, so the box and text stay sharp "
                         "instead of being magnified 2.1x afterwards.")
    ap.add_argument("--view-every", type=int, default=0,
                    help="deprecated and ignored; see --record-hz.")
    ap.add_argument("--slide-speed", type=float, default=2.5,
                    help="lateral speed used to skirt round a no-fly zone when "
                         "the direct line is blocked but a gap exists")
    ap.add_argument("--fence-brake", type=float, default=12.0,
                    help="start slowing this far from a no-fly zone")
    ap.add_argument("--fence-standoff", type=float, default=3.0,
                    help="hold this far outside a no-fly zone")
    ap.add_argument("--dv-h", type=float, default=0.4)
    ap.add_argument("--dv-z", type=float, default=0.2)
    args = ap.parse_args()
    if args.live_view and not args.save_view:
        # The window shows the frames the recorder writes. Without --save-view
        # the recorder is never started, so there is nothing to show. Refusing
        # is clearer than silently enabling capture the caller did not ask for,
        # which would also change where the flight writes.
        ap.error("--live-view needs --save-view: the window draws the frames "
                 "the recorder produces, and without it none are captured")
    global LINEAR_BEARING
    LINEAR_BEARING = bool(args.linear_bearing)
    if args.coarse_timer:
        # The A/B arm still measures what it got: a null here would leave the
        # comparison without its control (review, 2026-09-24).
        global TIMER_RESOLUTION_MS
        TIMER_RESOLUTION_MS = _measure_sleep_ms()
        print(f"[timer] coarse (default) timer; a 1 ms asyncio sleep measures "
              f"{TIMER_RESOLUTION_MS} ms")
        fine = None
    else:
        fine = _fine_timer_begin()
    try:
        return asyncio.run(fly(args))
    finally:
        _fine_timer_end(fine)


if __name__ == "__main__":
    code = main()
    # FORCE THE EXIT.
    #
    # Everything this script produces - flight log, metrics, kpi, manifest,
    # frames - is on disk by the time main() returns. What is not guaranteed is
    # that the simulator client's own receive threads have shut down: they are
    # not daemons, so a disconnect that never completes keeps the interpreter
    # alive indefinitely. Measured: a flight that had written every one of its
    # outputs at 17:37 was still running 88 minutes later, holding up the two
    # demos queued behind it.
    #
    # The teardown above is already bounded, so reaching here means the work is
    # done and only bookkeeping can still be pending. Leaving is correct.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code or 0)
