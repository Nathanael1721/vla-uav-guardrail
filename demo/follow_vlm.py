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
open-vocabulary detector: text in, boxes out. On an idle GPU its forward pass
takes 34-56 ms and the whole inference 66 ms (experiments/profile_owlvit.py). In
flight, sharing the GPU with the simulator, an inference measures about
210-290 ms median, so the detector runs at about 3-3.7 Hz (`det_hz` and the
per-inference `infer_ms` in each run's artefacts), and a box is about 0.45 s old
by the time the loop uses it (identity.det_latency_ms_median, 449 ms on
citylife_redcar_nfz_20261013). That is still about 40x faster than AerialVLA's
11-12 s, which is what makes closed-loop tracking feasible - not a latency that
can be ignored.

No VLA flies this demo. The words choose the target and OWL-ViT finds it in
the camera image, but every action comes from the hand-written controller
below, so the manifest's model field reads none:hand-written-controller
(`flight_manifest`; until 2026-10-19 this paragraph called the demo "still
vision-language-action", which the front page repeated, review of
2026-10-19; tracker card ARCH-21).

Except with --vla-hierarchy HOST:PORT (demo/vla_hierarchy.py, 2026-10-25):
then a fine-tuned AerialVLA decides the INTENT at its own rate and this
controller executes it at 10 Hz - with the subject locked the controller
flies with its stand-off at the VLA's desired range, with it lost the VLA's
step tracker flies the search, a LAND holds, an answer 1.5 s old hands
everything back to the controller - and the manifest and metrics["pilot"]
name that model. Off by default; without it the flight is unchanged.

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
from guardrail.audit import AUDIT_FILE                             # noqa: E402
from guardrail import kpi as kpi_mod                                # noqa: E402
from guardrail.bundle import load_for_flight                        # noqa: E402
from guardrail.compiler import join_negative_xy                     # noqa: E402
from guardrail import people as people_feed                         # noqa: E402
from guardrail.frames import filter_as_body                         # noqa: E402
from guardrail.manifest import (                                    # noqa: E402
    DEV_YAML_FLAG, NO_VLA, build_manifest, flight_bundle_path, flight_policy_choice,
    is_kpi_grade, model_hash, pilot_record,
)
from guardrail.replay import EpisodeRecord, verify_replay, write_replay  # noqa: E402
from guardrail.shield import (LOOKAHEAD_DT_S, LOOKAHEAD_S,           # noqa: E402
                              nearest_cell_clearance)
from guardrail.models import (                                      # noqa: E402
    Action4D, AltitudeEnvelope, DistanceEnvelope, ObstacleClearance, SubjectStandoff,
)

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
# --record-vla only (wave FT-D, 2026-10-23; demo/vla_record.py): the same
# scene and airframe with AerialVLA's VLAFront / VLADown cameras added, so the
# flight can be recorded as the model will see it. Without the flag the
# follow loads SCENE, as before (flight_scene). --vla-hierarchy loads it too:
# the VLA asks with those cameras, and nothing is recorded.
SCENE_VLA_RECORD = "scene_semantic_vla.jsonc"


def flight_scene(args) -> str:
    """The scene a flight loads: SCENE, or SCENE_VLA_RECORD with
    --record-vla or --vla-hierarchy (the variant robot config's FrontCamera,
    Chase and DownCamera are robot_semantic_quad.jsonc's;
    tests/test_follow_dataset.py)."""
    return (SCENE_VLA_RECORD if getattr(args, "record_vla", None)
            or getattr(args, "vla_hierarchy", None) else SCENE)
# The policy a flight flies when neither --policy nor --bundle is given: its
# signed bundle (city_policy_choice), or this YAML under --dev-yaml.
DEFAULT_POLICY = ROOT / "policies" / "follow_car.yaml"


class _NoBeat:
    def beat(self, *a, **k):
        return None

    def stop(self, *a, **k):
        return None


def _live_heartbeat(name: str, **kw):
    """A heartbeat for the tracker's Dashboard "running now" panel
    (tools/live_status.py), or a no-op when anything goes wrong. beat() only
    stores numbers in memory; a daemon thread writes at most once a second to
    %LOCALAPPDATA%, outside OneDrive, and swallows its own errors, so the
    flight loop never waits on it and cannot be stopped by it. Loaded by path:
    tools/ is not put on sys.path, where its modules could shadow demo/'s."""
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "vlaguard_live_status", ROOT / "tools" / "live_status.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.heartbeat("flight", name, **kw)
    except Exception:                                   # noqa: BLE001
        return _NoBeat()


def city_policy_choice(args, bundles_dir=None) -> tuple:
    """`(bundle, yaml, record)` for load_for_flight: the city rail flies the
    SIGNED bundle of its policy (audit card ARCH-29, 2026-10-21).

    Until now this rail loaded `--policy P` (default policies/follow_car.yaml)
    as unsigned YAML on every flight - citylife_redcar_nfz_motion2_20261019's
    kpi.json says policy_source "yaml", signature "unsigned" - while the
    ArduPilot rails had flown the signed bundle by default since 12-13 Oct.
    Now, as there (guardrail.manifest.flight_policy_choice):

      --bundle B             B, signature-verified; with --dev-yaml, refused.
      --bundle B --policy P  handed on: load_for_flight refuses "not both".
      --policy P             P's signed bundle, bundles/<policy_id>-v<version>
                             .tar.gz (tools/build_flight_bundles.py builds
                             it), recorded `default_bundle: true` and
                             `bundle_of: P`; refused when it is not on disk,
                             or no longer flies P's policy (an edit without a
                             version bump), naming both ways out.
      --policy P --dev-yaml  P itself, unsigned, recorded `dev_yaml: true` and
                             never KPI-grade: the explicit fallback.
      neither                as --policy DEFAULT_POLICY.

    `--policy` keeps naming WHICH rule set flies, as every launcher uses it;
    only what is loaded for it changed. ValueError before anything starts."""
    dev = bool(getattr(args, "dev_yaml", False))
    named = getattr(args, "policy", None)
    yaml_for = named or str(DEFAULT_POLICY)
    if getattr(args, "bundle", None) or dev:
        return flight_policy_choice(args.bundle, named, dev_yaml=dev,
                                    default_yaml=yaml_for)
    try:
        bundle = flight_bundle_path(yaml_for, bundles_dir)
    except Exception as exc:                                  # noqa: BLE001
        raise ValueError(f"cannot read the policy {yaml_for}: "
                         f"{type(exc).__name__}: {exc}") from None
    if not bundle.is_file():
        raise ValueError(
            f"the signed bundle of {yaml_for} is not on disk ({bundle}). Build it "
            f"(python tools/build_flight_bundles.py {Path(yaml_for).stem}), give "
            f"--bundle, or pass {DEV_YAML_FLAG} to fly the YAML unsigned, recorded "
            f"as a development run")
    b, y, rec = flight_policy_choice(None, None, dev_yaml=False,
                                     default_yaml=yaml_for, default_bundle=bundle)
    shown = Path(yaml_for).resolve()
    rec["bundle_of"] = (shown.relative_to(ROOT).as_posix()
                        if shown.is_relative_to(ROOT) else shown.as_posix())
    return b, y, rec


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

# The subject's HEIGHT by policy class, metres: only --teacher-truth reads it
# (the box it draws from the subject's true ground point up to this height).
SUBJECT_HEIGHT_M = {"pedestrian": 1.7, "cyclist": 1.7, "motorcycle": 1.4, "car": 1.5,
                    "van": 2.0, "truck": 3.2, "bus": 3.2}
SUBJECT_HEIGHT_DEFAULT = 1.5


def subject_height_m(query: str) -> float:
    """The --teacher-truth box's height for the words (subject_width's class)."""
    return SUBJECT_HEIGHT_M.get(subject_width(query)[1], SUBJECT_HEIGHT_DEFAULT)


def teacher_truth_fn(car, max_ahead_s: float = 1.0):
    """f(t_capture) -> (north, east) of `car` (a level car, .pos) at that
    moment, for --teacher-truth: its last polled position carried forward at
    the velocity of its last two DIFFERENT polls (the level is polled at most
    every --level-truth-period, 0.5 s: 2 m stale at 4 m/s), at most
    `max_ahead_s` ahead; None before the first poll. Called from the
    detector's thread only (its history is that thread's)."""
    hist = []

    def f(t_cap=None):
        pos = getattr(car, "pos", None)
        if pos is None:
            return None
        now = time.time()
        p = (float(pos[0]), float(pos[1]))
        if not hist or hist[-1][1:] != p:
            hist.append((now, p[0], p[1]))
            del hist[:-2]
        tq = now if t_cap is None else float(t_cap)
        if len(hist) == 2 and 0.05 < hist[1][0] - hist[0][0] < 3.0:
            dt0 = hist[1][0] - hist[0][0]
            vx, vy = (hist[1][1] - hist[0][1]) / dt0, (hist[1][2] - hist[0][2]) / dt0
            dt = min(max(tq - hist[1][0], -0.5), max_ahead_s)
            return hist[1][1] + vx * dt, hist[1][2] + vy * dt
        return hist[-1][1], hist[-1][2]
    return f

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


def yaw_command(gain: float, bearing: float, ff: float = 0.0) -> float:
    """Proportional yaw, plus an optional feed-forward rate, clipped to the
    pilot's own ceiling.

    One function because there were two copies of the clip and only one of them
    was applied, which is the entire defect. A shared constant would not have
    been enough - the estimator branch had no clip at all to keep in step.

    `ff` (rad/s) is the line-of-sight rate (see `los_rate`); the clip applies
    to the sum, so the ceiling holds whatever the feed-forward says. With
    ff = 0 this is exactly the proportional command it always was.
    """
    return float(np.clip(gain * bearing + ff, -YAW_RATE_CAP, YAW_RATE_CAP))


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
        # ONE figure the level walks, named by its tag (--level-car Ped_04,
        # 1 Nov): the instance, as for a level car - the operator named a
        # person, not the crowd
        if car is not None and car_is_truth_for(subject_class, car):
            return [[round(car.pos[0], 2), round(car.pos[1], 2)]]
        # `people is None` is the DEFAULT (--pedestrians 0), and also what
        # happens when the building mask or the baked GLB pack is missing. The
        # flight then logs truth.pts == [] on every tick, the whole phase scores
        # unscorable, and nothing says so - see the start-up check in main(),
        # which refuses that combination the way an unreachable policy class is
        # already refused.
        return ([[round(f.x, 2), round(f.y, 2)] for f in people.figures]
                if people is not None else [])
    if car is not None and car_is_truth_for(subject_class, car):
        return [[round(car.pos[0], 2), round(car.pos[1], 2)]]
    return []


def car_is_truth_for(subject_class, car) -> bool:
    """Is `car` the subject's truth? The scripted car only for "car" (a van is
    not the scripted car, pinned in tests/test_range_and_lock.py). A LEVEL
    car - one actor of the level named by its tag - for any vehicle class
    the operator names it as (LEVEL_CAR_CLASSES): Car_00 is a box truck, and
    "a box truck" behind --level-car Car_00 is scored against it (31 Oct,
    the multi-target recordings). A level PEDESTRIAN (kind "pedestrian",
    --level-car Ped_04; 1 Nov) is truth for the pedestrian class only, and
    never for a vehicle."""
    tagged = getattr(car, "tag", None) is not None
    if getattr(car, "kind", "car") == "pedestrian":
        return subject_class == "pedestrian" and tagged
    return subject_class == "car" or (subject_class in LEVEL_CAR_CLASSES and tagged)


def subject_truth_names(subject_class, car, people):
    """Names parallel to `subject_truth_pts`, or None when the source has none.

    The class-level score asks "is the box on ANY pedestrian", which saturates in
    a crowd. Following one NAME through the flight is what lets
    `track_truth.instance_rows` ask "is the box on the one it locked".
    """
    if subject_class == "pedestrian":
        if car is not None and car_is_truth_for(subject_class, car):
            return [car.tag]
        if people is None:
            return None
        names = [getattr(f, "name", None) for f in people.figures]
        return names if all(names) else None
    if car is not None and car_is_truth_for(subject_class, car):
        tag = getattr(car, "tag", None)
        return [tag] if tag else None
    return None


def update_level_truth(people, subject_class, t: float,
                       feed_people: bool = False) -> bool:
    """Advance the pedestrian source for this tick; True when it was asked.

    `people` is one of two very different things. Client-spawned figures
    (demo/pedestrians.py) are SCENERY first: update() is what makes the
    walkers walk, so it runs whatever is being followed. A level-owned crowd
    (demo/level_actors.LevelActors) is truth ONLY - nothing in the scene moves
    when it is polled - and one poll is a game-thread RPC that loops over every
    name: about 100 ms for 40 pedestrians. On citylife_redcar_nfz_20261013,
    a CAR mission, that poll ran 429 times for nothing: ms.truth p90 95.8 ms,
    max 1005.7 ms, every 5th-6th tick at the 0.5 s period. It sits between the
    pose read and the Shield, so on those ticks the controller and the Shield
    worked on a pose 100-1000 ms old, and the deadline pacing then fired the
    next tick at once: 411 tick gaps under 20 ms against 422 of 150-300 ms.
    Commands arrived in pairs, and the per-tick slew limit was spent twice in
    a few milliseconds - part of the pitch jerks the user saw.

    Nothing reads pedestrian truth unless the subject IS a pedestrian
    (subject_truth_pts, subject_truth_names, _live_sep), so a level crowd is
    polled only then - or when `feed_people`: the Shield is told every
    pedestrian for a distance_envelope over people (SimPeopleFeed; since
    2026-11-18), at the poll period --level-truth-period sets. A retarget to
    a person happens at the top of the tick, before this, and
    LevelActors.update polls at once when its period has passed, so the
    first pedestrian tick still carries fresh truth.
    """
    if people is None:
        return False
    if (type(people).__name__ == "LevelActors" and subject_class != "pedestrian"
            and not feed_people):
        return False
    people.update(t)
    return True


def sim_people_source(policy, args) -> str | None:
    """The people source this rail gives the Shield for a distance_envelope
    over people (Shield(people_source=...), guardrail/people.py): the
    simulator's pedestrian list - the level's own crowd polled by tag
    (--level-peds) or the client's spawned figures (--pedestrians). None for
    a policy without such a rule (the Shield it builds is the one it built).
    A people policy flown with neither is refused: CityLife walks pedestrians
    of its own, which the Shield would not know about."""
    ids = people_feed.needs_people(policy)
    if not ids:
        return None
    if getattr(args, "level_peds", 0) > 0:
        return (f"simulator: the level's {args.level_peds} pedestrians polled by tag "
                f"({args.level_ped_prefix}*, level_actors.LevelActors, every "
                f">= {args.level_truth_period:g} s)")
    if getattr(args, "pedestrians", 0) > 0:
        return (f"simulator: the client's {args.pedestrians} spawned figures "
                f"(pedestrians.Pedestrians, commanded positions)")
    raise SystemExit(f"[policy] {', '.join(ids)}: a distance_envelope over people "
                     f"needs the simulator's pedestrian list on this rail: "
                     f"--level-peds N (the level's crowd) or --pedestrians N")


class SimPeopleFeed:
    """The simulator's pedestrian list into the people memory, once a tick
    (a distance_envelope over people on the city sim rails, 2026-11-18).

    step(t) is called after the list was brought up to date for mission
    time `t` (LevelActors.update / Pedestrians.update) and returns the
    people for Shield.set_people. A level crowd (level_actors.LevelActors)
    is a POLL: observed once, at the tick it was made (its poll count is
    the key; between polls the memory ages the last one), and the poll
    `resolve` made before the mission - seconds to minutes before the first
    tick - is never observed: the first step polls afresh. Client figures
    (pedestrians.Pedestrians) are where they were commanded this tick. Only
    usable fixes (people.figure_points). `people` None: nobody is listed,
    and the memory only ages and drops."""

    def __init__(self, people, memory: "people_feed.PeopleMemory"):
        self.people = people
        self.memory = memory
        self._first = True

    def step(self, t: float) -> list:
        ppl = self.people
        if ppl is not None:
            if type(ppl).__name__ == "LevelActors":
                if self._first:
                    ppl.refresh()
                key = ("level", ppl.n_calls)
            else:
                key = ("client", float(t))
            self.memory.observe(float(t), people_feed.figure_points(ppl.figures), key=key)
        self._first = False
        return self.memory.current(float(t))


# Share of the mission's ticks that must carry the subject's ground truth for
# the follow to count as SCORED - perception_node's TRUTH_MIN_COVERAGE, so the
# two rails answer "was this follow measured?" the same way.
FOLLOW_TRUTH_MIN_COVERAGE = 0.95


def follow_scored(n_rows: int, n_truth: int) -> bool:
    """Was the follow measured at all? (metrics["follow_scored"])

    guardrail/kpi.py reads a None `frac_within_30m` as "no failure", and only
    `follow_scored is False` turns mission_success into None
    ("follow_not_scored"). This rail never set the key, so a flight with no
    subject truth - "a red van" behind a --level-car, the --no-car control,
    or a flight that aborted before its first tick - wrote kpi.json with
    outcome success and mission_success true from a follow nobody measured.
    Coverage, not "any truth at all", so a flight whose second phase had no
    truth does not count as scored either.
    """
    return n_rows > 0 and n_truth >= FOLLOW_TRUTH_MIN_COVERAGE * n_rows


# The classes a --level-car is ground truth for (subject_truth_pts,
# car_is_truth_for): the vehicles the level drives (cars, the pickups and
# SUVs the policy classes as van and car, the box trucks).
LEVEL_CAR_CLASSES = ("car", "van", "truck")


def level_car_refusal(phrases, kind: str = "car"):
    """(phrase, class) of the first phrase a --level-car cannot score, or None.

    The start-up check accepted car, van, truck and bus, but truth existed only
    for "car", so "a red van" behind a level car flew a follow with an empty
    truth on every tick, frac_within_30m None, which kpi.json read as success.
    Since 31 Oct a level car is truth for every class in LEVEL_CAR_CLASSES
    (car_is_truth_for); a bus or a person still is not. `phrases` is the
    --object phrase first, then the retargets; a retarget may also go to a
    pedestrian, whose own truth source fly() has already required.

    `kind` "pedestrian" (--level-car Ped_04, level_actors.kind_of_tags): the
    level figure is truth for the pedestrian class only, so every phrase,
    retargets included, must name a person.
    """
    for i, ph in enumerate(phrases):
        cls = subject_width(ph)[1]
        if kind == "pedestrian":
            if cls == "pedestrian":
                continue
            return ph, cls
        if cls in LEVEL_CAR_CLASSES or (i > 0 and cls == "pedestrian"):
            continue
        return ph, cls
    return None


def det_rates(g_end: dict, g_t0: dict, mission_s) -> dict:
    """The detector's mission figures from the grounder's counters at the end
    and at the mission clock's t0.

    `det_hit_rate` used to divide by every inference since the detector
    loaded, the start gate's included - the defect already fixed for det_hz.
    In identity mode no box is published while acquiring, so every gate
    inference counted as a miss: citylife_redcar_nfz_20261013 reported 0.495
    for a mission whose inferences found the car on 745 of 750 (0.993). The
    figure tracked how long the gate waited, not the detector. Without
    identity the gate publishes every colour-passing box, so it was inflated
    instead. The old figure is kept as `det_hit_rate_all_inferences_legacy`
    so old and new metrics stay comparable.

    Identity re-acquisition (begin_reacquire) still publishes nothing, so its
    inferences remain misses here: during a re-acquisition the subject is, by
    definition, not held.
    """
    seen, miss = int(g_end.get("n_seen", 0)), int(g_end.get("n_miss", 0))
    seen0, miss0 = int(g_t0.get("n_seen", 0)), int(g_t0.get("n_miss", 0))
    m_seen, m_n = seen - seen0, (seen + miss) - (seen0 + miss0)
    return {
        "det_hit_rate": round(m_seen / max(1, m_n), 3),
        "det_hit_rate_all_inferences_legacy": round(seen / max(1, seen + miss), 3),
        "det_mission_inferences": m_n,
        "det_hz": (round(m_n / max(1e-6, mission_s), 2)
                   if mission_s is not None else None),
    }


def hud_target(identity_on: bool, est_served: bool, det_present: bool,
               age: float, tier=None):
    """(show the box?, mode label or None) for the video HUD on a TRACK tick.

    A box older than 0.6 s is not what the aircraft is flying on when the
    estimator is serving: it is the last place the detector saw something.
    Without --identity the HUD kept that box on screen labelled TARGET LOCKED
    for up to the estimator's 3 s coast - citylife_redcar_final1 frame 340
    shows a green "a red car" box on the bollards, 2.2 s old, with no car in
    it - while identity runs already said TRACKING (PREDICTED). Now both modes
    do. The box servo (no estimate) keeps its box and TARGET LOCKED up to its
    own det_max_age, because there the box IS what it steers on.
    """
    if est_served:
        if not det_present or age > 0.6:
            return False, "TRACKING (PREDICTED)"
        if identity_on and tier == "soft":
            return True, "TRACKING (PARTLY HIDDEN)"
        return True, None
    return det_present, None


def estimator_measurement(det, g: dict, tick_pose, identity_on: bool,
                          newest_range=None, legacy_tick_pose: bool = False,
                          hfov_deg: float = None):
    """What the estimator is fed for one fresh box: (t, x, y, yaw, bearing,
    range or None, source).

    `tick_pose` is (t_now, x, y, yaw) of the tick consuming the box and
    `g` the grounder snapshot, whose `cap` describes the frame the box came
    from (its pose and capture time, filled in every mode) and whose `feat`
    holds identity's measurements of it.

    THE BOX'S OWN MOMENT, IN EVERY MODE. Without --identity (the CLI default,
    and every CityLife person mission) the bearing measured in a frame
    captured ~0.5 s earlier was combined with THIS tick's yaw, position and
    clock, and with the NEWEST depth frame. Replayed over 1,390 on-car boxes
    of the 12 non-identity red-car flights, that pairing put the bearing off by
    a median 1.46 deg (p90 7.50) against 0.61 deg (p90 2.29) at the capture
    pose; on turning ticks 7.6 against 1.1 deg, 2.9 m against 0.4 m sideways.
    The estimate drives the yaw servo and is the subject the Shield is served.
    Now the capture pose and time, and the range measured in the capture's own
    depth frame (Grounder publishes it as cap["rng"]; None when no depth frame
    lay within 70 ms - the caller then falls back to the width prior, as the
    Grounder's colour check does, never to the newest depth).

    `legacy_tick_pose` (--legacy-tick-pose) restores the old pairing, one
    part of reproducing a recorded non-identity flight (--legacy-motion's
    help lists what no flag restores); an observation source with no capture
    pose gets it too.
    """
    t_now, x, y, yaw = tick_pose
    b = box_bearing(det[0], det[5], hfov_deg)
    cap = (g or {}).get("cap") or {}
    cp = cap.get("pose")
    if identity_on:
        feat = (g or {}).get("feat") or {}
        t_m, x_m, y_m, yaw_m, r = t_now, x, y, yaw, None
        if cp is not None:
            t_m = cap.get("t_capture") or cap.get("t_cap") or t_now
            x_m, y_m, yaw_m = cp["x"], cp["y"], cp["yaw"]
        if feat.get("rng_h") is not None and cp is not None:
            r = float(feat["rng_h"])
            b = feat["bearing"] - yaw_m
            b = math.atan2(math.sin(b), math.cos(b))
        return t_m, x_m, y_m, yaw_m, b, r, ("capture" if cp is not None else "tick")
    if cp is not None and not legacy_tick_pose:
        t_m = cap.get("t_capture") or cap.get("t_cap") or t_now
        return t_m, cp["x"], cp["y"], cp["yaw"], b, cap.get("rng"), "capture"
    return t_now, x, y, yaw, b, newest_range, "tick"


# --------------------------------------------------------------------------- #
# THE SUBJECT IN THE WORLD, AS RULES (2026-11-15). fly()'s identity path keeps
# its subject with the target estimator, the strict lock's prior and the
# Reacquirer, and decides five things about them: the estimator and the trail
# it is built with, the seed of a commit, which SOFT box may feed it, where a
# lapse anchors the re-acquisition, and the snapshot the strict lock leans on.
# Those decisions were inline in fly(); they are here, unchanged, so a rail
# whose pilot is not this file's controller - the Project AirSim + ArduPilot
# perception node's VLA follow, through demo/subject_track.SubjectTrack - runs
# the same code instead of a copy. fly() calls every one of them.
# --------------------------------------------------------------------------- #
# A SOFT box (identity: may continue a track, never start one) feeds the
# estimator only while the estimate's last update is under SOFT_EST_AGE_S
# older than the box's capture and an OK box was accepted under
# SOFT_OK_WITHIN_S ago.
SOFT_EST_AGE_S = 1.5
SOFT_OK_WITHIN_S = 3.0
# --trail-follow's breadcrumbs (demo/trail.py): one every 1.5 m, the last 120 m.
TRAIL_SPACING_M = 1.5
TRAIL_MAX_LEN_M = 120.0


def subject_estimator(subject_class) -> TargetState:
    """The follow's target estimator: demo/target_state.TargetState with the
    class's physical speed ceiling (SUBJECT_VMAX_MPS; no ceiling for a class
    it does not name)."""
    return TargetState(v_max=SUBJECT_VMAX_MPS.get(subject_class))


def subject_trail():
    """The breadcrumbs the estimator lays (--trail-follow; demo/trail.py)."""
    return trail_mod.Trail(spacing_m=TRAIL_SPACING_M, max_len_m=TRAIL_MAX_LEN_M)


def seed_estimate(estimator, pf: dict, cap_pose, t_capture: float) -> tuple:
    """A subject committed by the start gate or a re-acquisition: the
    estimator (None: none) reset and seeded from the committing sighting's
    features `pf` (identity's: rng_h, bearing, P) at its capture's pose and
    time. Returns (seeded, P0): whether the estimator took the seed, and the
    subject's place now - the estimator's posterior when seeded, else the
    sighting's own map point, else None."""
    seeded = False
    if estimator is not None:
        estimator.reset()
        if (cap_pose is not None and pf.get("rng_h") is not None
                and pf.get("bearing") is not None):
            b0 = pf["bearing"] - cap_pose["yaw"]
            b0 = math.atan2(math.sin(b0), math.cos(b0))
            seeded = bool(estimator.update(t_capture, cap_pose["x"], cap_pose["y"],
                                           cap_pose["yaw"], b0, float(pf["rng_h"])))
    if seeded:
        P0 = (float(estimator._xu[0]), float(estimator._xu[1]))
    elif pf.get("P") is not None:
        P0 = (float(pf["P"][0]), float(pf["P"][1]))
    else:
        P0 = None
    return seeded, P0


def soft_may_continue(t_meas: float, t_last_update, now: float, t_ok_last: float) -> bool:
    """May a SOFT box captured at `t_meas` feed the estimator at `now`? Only
    to CONTINUE a live track: the estimate's last update (`t_last_update`,
    None: no estimate) under SOFT_EST_AGE_S before it, and an OK box
    accepted (`t_ok_last`) under SOFT_OK_WITHIN_S before now."""
    est_age = t_meas - t_last_update if t_last_update is not None else 1e9
    return est_age < SOFT_EST_AGE_S and now - t_ok_last < SOFT_OK_WITHIN_S


def lapse_anchor(estimator, last_acc, t_meas_last_cap, t_meas_last, last_meas_speed,
                 v_prior) -> dict | None:
    """Where a LAPSE (no accepted measurement for lapse_s) anchors the
    re-acquisition (Reacquirer.start): {t, P, v} - the estimator's posterior
    and its time when there is one, else the last accepted place
    (`last_acc`) at the capture time of its measurement (else the time it
    was taken); the last measured speed, else the one before the last commit.
    None when nothing places the subject."""
    have_est = estimator is not None and estimator._xu is not None
    P_a = ((float(estimator._xu[0]), float(estimator._xu[1])) if have_est else last_acc)
    if P_a is None:
        return None
    return {"t": (estimator.t_last_update if have_est and estimator.t_last_update is not None
                  else (t_meas_last_cap if t_meas_last_cap is not None else t_meas_last)),
            "P": P_a,
            "v": (last_meas_speed if last_meas_speed is not None else v_prior)}


def estimator_prior(estimator, centre_up_m: float) -> dict | None:
    """The snapshot the strict lock leans on (Grounder.set_prior): the
    estimator's posterior {t, x, y, vx, vy, t_upd} with the subject's centre
    height `up` for the projection into a frame; None with no posterior."""
    xu = None if estimator is None else estimator._xu
    if xu is None:
        return None
    return {"t": estimator.t_last_update, "x": float(xu[0]), "y": float(xu[1]),
            "vx": float(xu[2]), "vy": float(xu[3]), "t_upd": estimator.t_last_update,
            "up": centre_up_m}


def servo_want_width(want_range_m: float, object_width_m: float,
                     hfov_deg: float = None) -> float:
    """The box fraction the box servo must hold to stand off `want_range_m`.

    `want_range_from_width` turns --want-width into a range on the LINEAR
    pixel-to-angle map, deliberately left alone so the estimator arm (the
    default, behind every recent flight) keeps its 15.83 m for 0.16 on a 4 m
    car. The box servo held the raw fraction instead, which on the real
    pinhole camera is 12.5 m: the two A/B arms flew set-points 27 % apart,
    and most of the published 3.5 m gap between them (FINDING-target-
    estimator-and-control-aug17) was that, not the estimator. Converting the
    metric stand-off back through the pinhole model puts both arms on one
    set-point: 0.126 for a 4 m car at 15.83 m.
    """
    hfov = CAMERA_HFOV_DEG if hfov_deg is None else hfov_deg
    return (object_width_m / 2.0) / (max(1e-3, want_range_m)
                                     * math.tan(math.radians(hfov) / 2.0))


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
                      cap: float = 1.1, side: float = 1.0) -> float:
    """Yaw rate for the search sweep, rad/s.

    A cosine, so the ANGLE is a sine about the heading the nose had when the
    search began, and the net rotation over a whole period is zero. That
    property is the whole point and is asserted in the tests: the previous
    implementation used a constant sign, so the nose rotated away from a
    bearing that was still correct and never came back -- 292 degrees on one
    measured episode, which is the "360 manoeuvre" seen on the demo video.

    It is NOT centred on the last-seen bearing, whatever this docstring used to
    say: nothing here sees a bearing, and after the open-loop coast the target
    can sit 24-35 deg off the nose. `side` (+1 right, -1 left) picks which way
    the first half-swing goes. It always went right (+0.685 rad/s at t = 0 for
    25 deg / 4 s), so a car lost turning LEFT got a yaw step of up to
    1.2 rad/s at the coast-to-search hand-over and a first swing away from where
    it went; the caller now passes the side the target was lost on
    (verify_sweep_sim, 2026-10: mirrored left and right losses behaved
    differently, out of view on 46 against 35 of 50 search ticks).

    `sweep_deg` is a half-amplitude and should stay inside the camera's 45 deg
    horizontal half-FOV.
    """
    w = 2.0 * math.pi / max(0.5, period_s)
    sgn = -1.0 if side < 0 else 1.0
    return sgn * float(np.clip(math.radians(sweep_deg) * w * math.cos(w * elapsed_s),
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

    A PASS MAY HAVE GAPS - opt-in, `gap_s` > 0; 0 (follow_vlm's) keeps the
    rule above: an inference with nothing acceptable ends the streak. The
    AerialVLA follow vla_follow_ft_07_r3_off (2026-10-24) lost the car at
    39 s and never took it back although it drove past the hovering aircraft
    twice, OK and 7-26 m away: at that flight's acquiring rate (~2.2 Hz) the
    passes gave 7 + 5 and 9 + 3 OK inferences in a row, each run cut by one
    to three frames with no car, and the moving test needs 2 x need = 10 in
    a row - so it was never even asked, though the car had moved 5-11 m.
    With gap_s, while the moving test applies (no anchor, or the reach past
    reach_cap_m), a streak whose last pick is at most gap_s old survives an
    inference with nothing acceptable: nothing is added to it, the next pick
    must still lie within step_m + v_r dt of the last, and a gap never
    counts as a sighting. The bar is unchanged (a gap's long step only
    raises the jitter estimate), and a streak too short to be judged is not
    marked static however long it spans. An anchored short loss is judged
    as before.

    `last_outcome` says what the last step() decided - "committed", or why
    not: "no candidate", "none acceptable" (each candidate's reason is in
    `refused`), "gap", "streak" (fewer than `need`), "short pass" (the
    moving test cannot be asked yet), "not moving" (asked, below the bar),
    "static" (the place just marked) - so a caller can count every refusal,
    including the ones `refused` never sees.
    """

    def __init__(self, need: int = 4, max_range_m: float = 45.0,
                 reach_base_m: float = 10.0, step_m: float = 3.0,
                 v_min: float = 3.0, v_max: float = 12.0,
                 motion_m: float = 2.0, reach_cap_m: float = 60.0,
                 static_m: float = 3.0, static_after_s: float = 5.0,
                 static_ttl_s: float = 30.0, trace_max: int = 80,
                 motion_k: float = 3.0, static_after_min_s: float = 8.0,
                 far_need: int = 3, far_window_s: float = 3.0, gap_s: float = 0.0):
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
        self.gap_s = float(gap_s)
        self.last_outcome = None
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
        # nothing constrains the place: the moving test applies (below)
        free = self.motion_m > 0 and (reach is None or reach > self.reach_cap_m)
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
            if (self.gap_s > 0 and free and self.streak > 0 and self.pick_t is not None
                    and t_cap - self.pick_t <= self.gap_s):
                self.last_outcome = "gap"           # the pass goes on (class docstring)
                return False
            self.last_outcome = "none acceptable" if cands else "no candidate"
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
            self.last_outcome = "streak"
            return False
        if free:
            moved = self.moved_m()
            if moved is None or moved < self.motion_bar_m():
                # (with gaps a streak too short to judge may span
                # static_after_s: no static place until it has been judged)
                if (self.trace[-1][0] - self.trace[0][0] >= self.static_after_s
                        and (moved is not None or self.gap_s <= 0)):
                    P = np.median(np.asarray([(q[1], q[2]) for q in self.trace]), axis=0)
                    self.static.append((float(P[0]), float(P[1]), float(t_cap)))
                    self._refuse("static")
                    self._reset_streak()
                    self.last_outcome = "static"
                else:
                    self.last_outcome = "short pass" if moved is None else "not moving"
                return False
        self.last_outcome = "committed"
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

    THE ZONES ARE THE SHIELD'S (2026-10-07). This class used to build its own
    copy of every zone (`fence_polygon(f).buffer(f.margin_m)`, all of them, at
    construction) and measure against that copy. The grant allows one
    rule-evaluation code path (Safety Shield page), and a copy built once
    could not see a zone hot-applied mid-flight, a zone that moves, or one
    outside its time window. `polys` now asks the Shield
    (`Shield.zones_now()`): the same margin rings, compiled once in its IR, in
    force now, a moving zone where it is this tick. Pass the flight's Shield;
    without one a Shield is built over `policy` (no escalation FSM). Distances
    are 2-D, every zone's footprint whatever its band, as before.

    AND SO ARE THE DISTANCES (2026-11-03, WP3-22). gate() and slide() used to
    measure `poly.distance(Point(x, y))` against those rings themselves. They
    now ask the Shield's ruler (`Shield.zone_ruler()`, the distance
    Shield.fence_distance reports), once per call, for every probe point: the
    same rings, the same distance, one implementation. What stays here is
    the controller's own question - where to brake, which way round - built
    on those answers (`clear_aim`'s keep-out band is the rings grown by the
    stand-off, a planning shape, not a rule).
    """

    def __init__(self, policy, brake_m: float = 12.0, stand_off_m: float = 3.0,
                 obstacle_map: dict | None = None, min_clearance_m: float = 5.0,
                 street_mask: dict | None = None, shield=None,
                 guard_field: str = "grid"):
        # What the building half measures clearance on (`clearance`): "grid"
        # (the default, what every stored follow flight flew) or "shield"
        # (the Shield's own distance field, WP3-22 / X-13; opt-in with
        # --guard-field shield until it has been flown on the AGX).
        if guard_field not in ("grid", "shield"):
            raise ValueError(f"guard_field must be 'grid' or 'shield', not {guard_field!r}")
        self.guard_field = guard_field
        if shield is None:
            from guardrail.shield import Shield
            # Read for its rule geometry, never flown: a distance_envelope's
            # maps are the flight Shield's business (require_maps).
            shield = Shield(policy, escalation=False, require_maps=False)
        self.shield = shield
        self._keepout_key = None
        self._keepout = None
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
        self._clear_fn = None
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
        # ...and the two distances that verdict was made from, each kept
        # apart: the building clearance where the building half braked (None
        # when it did not), and the distance to the nearest zone (None
        # without zones). gate() returns ONE distance, and which one depends
        # on the branch: with a fence 13 m ahead it returns the fence's 13.00
        # while the building half brakes at 4.70 m, inside the 5 m ring
        # (review, 2026-10-19). guard_urgent() read that shared number against
        # the building ring and let the hold be ramped. It reads these.
        self.last_obstacle_d: float | None = None
        self.last_fence_d: float | None = None

    @property
    def polys(self) -> list:
        """The keep-out zones' margin rings, from the Shield, in force now."""
        return [r.buffered for r in self.shield.zones_now()]

    def _ruler(self):
        """What gate() and slide() measure distances with: the Shield's
        ruler over the zones in force now (`Shield.zone_ruler()`). A subclass
        that hands the guard its own zones through `polys`
        (tests/test_follow_motion.py puts a fence where a test needs one) is
        measured by the same ruler over those rings (ZoneRuler.of_rings)."""
        if type(self).polys is FenceGuard.polys:
            return self.shield.zone_ruler()
        from guardrail.shield import ZoneRuler
        return ZoneRuler.of_rings(self.polys)

    @property
    def _aim_keepout(self):
        """The zones grown by the stand-off plus a metre, as one shape, for
        clear_aim(). Rebuilt only when the Shield's zones change (an event, a
        window edge, a moving zone's next position)."""
        recs = self.shield.zones_now()
        key = tuple((r.rule.id, id(r.buffered)) for r in recs)
        if key != self._keepout_key:
            self._keepout_key = key
            self._keepout = None
            if recs:
                from shapely.ops import unary_union
                self._keepout = unary_union([r.buffered for r in recs]).buffer(
                    self.stand_off_m + 1.0)
        return self._keepout

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
        keepout = self._aim_keepout
        if keepout is None:
            return None
        from guardrail.geometry import nearest_boundary_point, ring_contains
        ax, ay = x + ux * ahead_m, y + uy * ahead_m
        if not ring_contains(keepout, ax, ay):
            return None
        qx, qy = nearest_boundary_point(keepout, ax, ay)
        dx, dy = qx - x, qy - y
        n = math.hypot(dx, dy)
        if n < 1e-6:
            return None
        return dx / n, dy / n

    def _clearance_fn(self):
        """What the building half measures with: the Shield's clearance
        field (Shield.clearance_at), when the guard's Shield holds this
        obstacle map; else the same field built over the guard's map and
        read by the same function (guardrail/shield.py ClearanceField,
        field_distance)."""
        fn = self._clear_fn
        if fn is None:
            sh, occ = self.shield, self.occ
            same = (sh.has_obstacle_map and sh._dist.shape == np.asarray(occ["occ"]).shape
                    and sh._res == float(occ["res"]) and sh._ox == float(occ["ox"])
                    and sh._oy == float(occ["oy"]))
            if same:
                fn = sh.clearance_at
            else:
                from guardrail.shield import ClearanceField
                fn = ClearanceField(occ).distance
            self._clear_fn = fn
        return fn

    def clearance(self, px: float, py: float, cap_m: float) -> float:
        """Distance to the nearest mapped obstacle, no more than `cap_m`.

        Returns `cap_m` when nothing is within it, so callers can treat the cap
        as "far enough to be uninteresting" without a special case.

        TWO MEASURES, ONE OPT-IN (2026-11-17, WP3-22 / X-13).
        guard_field="grid" (the default) is a scan of the occupancy grid
        (guardrail/shield.py `nearest_cell_clearance`: the nearest blocked
        cell centre within the cap, rounded to the nearest cell; this file's
        own `grid_scan_clearance` until 2026-11-18, moved into the library
        unchanged and shown bit-identical by tests/test_shieldcore_onepath.py),
        a second measure of the clearance rule beside the Shield's distance
        field, which reads the field bilinearly and chamfer-scaled and so up to
        1.4 m nearer beside a building face. guard_field="shield" reads the
        Shield's own field (`_clearance_fn`): the controller stops at the
        line the Shield defends, measured the same way. That changes the
        follow controller, so it stays opt-in until flown. Replayed open-loop
        on 10 stored AGX follow flights (tools/onepath_guard_replay.py,
        41959 ticks, gate() and slide() both replayed) the shield field
        changes the guard's decision - its mode or its sidestep side - on
        469 ticks (1.12 %; 15 % of logged holds, 4.6 % of near ticks), the
        guard's scale on 3.96 %, the building brake's on/off on 0.26 %, and
        skirts on 8948 ticks against 8730. On follow_car_gap.yaml the detour
        it finds grows from 15 m (reach 20) to 26 m (reach 30). Every stored
        follow result flew the grid scan.
        """
        if not self.occ:
            return cap_m
        if self.guard_field == "grid":
            return nearest_cell_clearance(self.occ, px, py, cap_m)
        return min(cap_m, self._clearance_fn()(px, py))

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
        0 at the stand-off, so along a steady approach it falls with distance.

        It keeps no memory between calls, so it is NOT smooth in time: beside a
        wall at 5.2 m a heading change of 2-4 deg flips it 1.00 -> 0.55 -> 0.27,
        and on citylife_redcar_nfz_20261013 it jumped by more than 0.5 in one
        tick 10 times. The single-call values are what the tests pin and are
        right; the smoothing between calls is the flight loop's GuardSmoother.
        """
        o_scale, o_d, o_blocked = self._obstacle_gate(x, y, vx, vy)
        o_cause = "obstacle" if o_scale < 1.0 else None
        # _obstacle_gate returns the clearance whenever its forecast hits a
        # building, including a hit beyond brake_m (above 4 m/s at the default
        # 12 m), where k > 1 and nothing brakes. guard_urgent() reads this as
        # "the building the guard is braking for", so record it only then
        # (review of f7611f9, 2026-10-19).
        self.last_obstacle_d = o_d if o_scale < 1.0 else None
        ruler = self._ruler()
        if not ruler:
            self.last_fence_d = None
            self.last_cause = o_cause
            return o_scale, o_d, o_blocked
        d = ruler.distance(x, y)
        self.last_fence_d = float(d)
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
            d_min = min(d_min, ruler.distance(x + ux * a, y + uy * a))
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
        speed = math.hypot(vx, vy)
        # The Shield's distances to the zones in force now (one ruler for
        # every probe of this call).
        ruler = self._ruler()
        polys = bool(ruler)
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
        if speed < 1e-3 or (not polys and not self.occ):
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
            """Far enough from every MAPPED obstacle: `clearance` (the grid
            scan by default, the Shield's field with guard_field="shield") at
            least min_clearance_m here. The same answer as the Chebyshev scan
            this held inline until 2026-11-17: the scan with the cap at
            min_clearance_m falls below it exactly when a blocked cell does."""
            if not self.occ:
                return True
            return self.clearance(px, py, self.min_clearance_m) >= self.min_clearance_m

        def _ahead_clear(px: float, py: float) -> bool:
            pts = [(px + ux * a, py + uy * a)
                   for a in (probe_m * 0.5, probe_m, probe_m * 1.5)]
            if polys and not all(ruler.distance(*q) >= self.stand_off_m for q in pts):
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
                if polys and ruler.distance(px, py) < self.stand_off_m:
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


# ---------------------------------------------------------------------------
# MOTION SHAPING (2026-10-19): why the aircraft pitched and rolled in jerks.
#
# The user's complaint: when the red car turns or pulls away, the drone
# "nose-dives / jerks". Measured on citylife_redcar_nfz_20261013 (flight_log,
# poses timed at the kinematics read): body tilt follows horizontal
# acceleration almost exactly (correlation 0.977 with atan(a/g)), pitch reached
# -14.9 deg and roll -12.2 deg, and every peak came from one of four things:
#
#   1. The forward command surging to the 5 m/s cap after the range grew (the
#      car pulling away from a stop, or the guard having held the aircraft):
#      the stand-off law was linear up to speed_max, 241 ticks at the cap.
#   2. FenceGuard's gate and slide switching on and off with no memory:
#      47 mode switches, 10 scale jumps > 0.5 in one tick, raw steps of
#      2.0-3.9 m/s; 8 of the 9 tilt peaks above 15 deg within 1 s of a switch.
#   3. The slew limiter (aerialvla_demo.RateLimiter): 0.4 m/s PER AXIS PER
#      CALL, world frame, no time base. A diagonal step got 0.566; a late tick
#      followed by an immediate one spent two steps within milliseconds (275
#      tick gaps under 10 ms against ~420 of 150-300 ms, see
#      update_level_truth), so bursts doubled the acceleration.
#   4. A ~14-18 deg bearing lag in every turn: a proportional yaw servo on a
#      rotating line of sight settles at lag = rate / gain (measured: truth
#      bearing = 0.98 s x LOS rate).
#
# The pieces below answer them in order: a saturating catch-up law
# (standoff_speed), a stateful smoother between the guard and the command
# (GuardSmoother; gate() and slide() stay pure), a time-based vector slew
# limiter with gentle speed-up, legacy-or-firmer braking and a yaw-rate slew
# (VecSlewLimiter), and a line-of-sight-rate feed-forward on yaw (los_rate).
# --legacy-motion restores all four exactly. That is the motion half of
# reproducing a recorded flight, not all of it: the same day's review fixes
# that change behaviour with no legacy switch are listed in its --help.
#
# Offline only so far (tests/test_follow_motion.py, a closed-loop kinematic
# follow of a car that stops, pulls away and turns, with a chattering guard
# and the 200 ms / 5 ms tick pattern): speed-up peak 3.11 -> 1.62 m/s^2, jerk
# peak 8.35 -> 3.44 m/s^3 (3.83 while the braking floor still fired inside
# turns), braking peak unchanged (2.33 -> 2.40, by design), turn lag 5.0 ->
# 1.4 deg median, range max 24.1 -> 23.8 m (mean 18.4 -> 19.8). Replayed
# open-loop on the 13 Oct commands, the limiter alone takes the
# aircraft-model accel peak 3.35 -> 2.13 m/s^2 (jerk 8.42 -> 6.46; with no
# hold flag the floor decides 207 of 2379 calls, 287 before it was confined
# to braking). Not yet flown.
# ---------------------------------------------------------------------------


class VecSlewLimiter:
    """Time-based slew limit on the commanded velocity, as a VECTOR.

    Replaces, for this script only, aerialvla_demo.RateLimiter (which other
    demos and the fine-tuning eval protocol still use unchanged). Per call:

      * dt is the time since the previous call, CAPPED at one tick. A late
        tick gets no extra allowance and a back-to-back one only its share, so
        a stall can no longer be repaid as a burst. (A floored dt would make
        the burst worse; review of finding rate-limiter-per-call-per-axis.)
      * The horizontal step is split along and across the previous velocity.
        Along-track speed-up is limited to `accel`, along-track slowing to
        `brake`, across-track (turning) to `lat`, all in m/s^2, and the whole
        step is scaled by ONE factor, so its direction is kept - per-axis
        clipping bent it toward 45 deg and allowed sqrt(2) x the limit.
      * BRAKING IS NEVER LATER THAN IT WAS. When the command asks for less
        speed AND it is braking - the step points back along the track
        (|across| <= -along: a stop, a scaled-down command, a blocked hold
        with its sidestep), or the caller passes `hold=True` (the flight loop
        passes guard_urgent()) - the resulting speed is held to no more than
        what the legacy per-axis, per-call step of `brake_floor_dv` would
        have left, in every tick pattern: the guard's holds stop at least as
        soon as before. A turn that merely asks a little less speed is not
        braking and gets no floor (it made every jittered turn dip, see the
        comment in __call__). The direction of the gentle step is kept; only
        its speed is cut. (`n_brake_floor` counts the calls this decided.)
      * Climb rate: `vz` m/s^2, never slower than the legacy `vz_floor_dv`
        step when slowing toward zero. Yaw rate: `yaw` rad/s^2 (0 = off);
        nothing limited it before, and a coast-to-search hand-over stepped it
        by up to 1.2 rad/s in one tick.
    """

    def __init__(self, accel: float = 2.0, brake: float = 4.0, lat: float = 2.5,
                 vz: float = 2.0, yaw: float = 2.0, tick: float = TICK,
                 brake_floor_dv: float | None = 0.4,
                 vz_floor_dv: float | None = 0.2):
        self.accel, self.brake, self.lat = float(accel), float(brake), float(lat)
        self.a_z, self.yaw_acc, self.tick = float(vz), float(yaw), float(tick)
        self.brake_floor_dv = brake_floor_dv
        self.vz_floor_dv = vz_floor_dv
        self.prev = Action4D()
        self.t = None
        self.n_calls = 0
        self.n_limited = 0          # calls whose horizontal step was cut
        self.n_brake_floor = 0      # ...and whose speed the legacy floor set

    def __call__(self, a: Action4D, now: float, hold: bool = False) -> Action4D:
        dt = (self.tick if self.t is None
              else min(max(now - self.t, 0.0), self.tick))
        self.t = now
        self.n_calls += 1
        p = self.prev
        dx, dy = a.vx - p.vx, a.vy - p.vy
        sp_prev = math.hypot(p.vx, p.vy)
        sp_ask = math.hypot(a.vx, a.vy)
        n = math.hypot(dx, dy)
        if sp_prev > 1e-6:
            ux, uy = p.vx / sp_prev, p.vy / sp_prev
        elif n > 1e-12:
            ux, uy = dx / n, dy / n          # from rest every step is along
        else:
            ux, uy = 1.0, 0.0
        d_par = dx * ux + dy * uy
        d_perp = -dx * uy + dy * ux
        a_par = self.brake if d_par < 0.0 else self.accel
        need = math.hypot(d_par / max(1e-6, a_par), d_perp / max(1e-6, self.lat))
        s = 1.0 if need <= dt else dt / need
        vx, vy = p.vx + s * dx, p.vy + s * dy
        if s < 1.0:
            self.n_limited += 1
        # THE FLOOR IS FOR BRAKING, NOT FOR TURNING. It first applied to ANY
        # call whose asked speed was lower than the last, and a turning
        # command at a steady 4 m/s asks for a lower speed every other tick
        # (the estimate's +-0.05 m/s jitter). Worse, the "legacy" step it
        # compares with is taken from this limiter's own prev, which already
        # lags the command in direction (the turn limit), so it named a speed
        # lower than the real legacy limiter would reach. Review, 2026-10-19
        # (40 jittered command turns at 4 m/s, 10 Hz): worst along-track
        # deceleration -2.56 m/s^2 at 6 deg/tick against -0.96 for the legacy
        # limiter and for this one without the floor; -3.20 / -2.55 / -1.06
        # at 10 deg/tick; -3.49 / -3.28 / -1.52 at 20 - a speed dip and a
        # re-acceleration at `accel` inside every turn, the nose-down/nose-up
        # jerk this limiter exists to remove. A stop is a step pointing BACK
        # along the track, so the floor now needs the along-track part to
        # dominate: d_par < 0 and |d_perp| <= -d_par (within 45 deg of
        # straight back). A stop is in that cone, as are a command scaled
        # down along its own direction (within 5 deg for any scale <= 0.9), a
        # reversal, and a blocked gate() (scale <= 0.35) plus a slide()
        # sidestep no faster than the speed it removed. A steady-speed turn
        # enters it only once the velocity lags the command by 90 deg.
        # ...OR the loop says the guard's brake must bite (`hold`, its
        # guard_urgent()): a blocked hold whose sidestep outruns the speed it
        # removes, or whose direction swings, falls outside the cone and is
        # still a hold. Replayed on the 13 Oct commands, the cone alone
        # dropped the floor on 102 calls the speed rule had floored, 21 of
        # them on ticks gate() had blocked (4 'hold', 17 'skirt').
        if (self.brake_floor_dv is not None and sp_ask < sp_prev - 1e-9
                and (hold or (d_par < 0.0 and abs(d_perp) <= -d_par))):
            f = self.brake_floor_dv
            lx = p.vx + float(np.clip(dx, -f, f))
            ly = p.vy + float(np.clip(dy, -f, f))
            sp_leg, sp_new = math.hypot(lx, ly), math.hypot(vx, vy)
            if sp_leg < sp_new - 1e-9:
                k = sp_leg / sp_new
                vx, vy = vx * k, vy * k
                self.n_brake_floor += 1
        dz = a.vz_up - p.vz_up
        lim_z = self.a_z * dt
        if self.vz_floor_dv is not None and abs(a.vz_up) < abs(p.vz_up):
            lim_z = max(lim_z, float(self.vz_floor_dv))
        vz = p.vz_up + float(np.clip(dz, -lim_z, lim_z))
        if self.yaw_acc > 0.0:
            ly_ = self.yaw_acc * dt
            yr = p.yaw_rate + float(np.clip(a.yaw_rate - p.yaw_rate, -ly_, ly_))
        else:
            yr = a.yaw_rate
        out = Action4D(vx=vx, vy=vy, vz_up=vz, yaw_rate=yr)
        self.prev = out
        return out

    def stats(self) -> dict:
        return {"calls": self.n_calls, "limited": self.n_limited,
                "brake_floor": self.n_brake_floor}


def guard_urgent(fence: "FenceGuard", fblocked: bool) -> bool:
    """May the guard's brake bite AT ONCE this tick? Only when it must: gate()
    says blocked (an incursion within 0.35 of the brake distance, or at the
    fence stand-off), or the aircraft is already inside the clearance ring of
    the building it is braking for, or inside a zone's stand-off. Anything
    less is ramped. Call it after this tick's gate().

    Each distance is measured against its own line, from the distances gate()
    recorded (FenceGuard.last_obstacle_d / last_fence_d). It used to take
    gate()'s returned distance and guess what it was from last_cause, but
    with a zone in range and the building half braking, gate() returns the
    ZONE's distance under cause "obstacle": beside a wall with a building
    jutting out ahead (aircraft 4.70 m from it, 5 m ring) and a fence 13 m
    ahead, the ring test read 13.00 > 5 and the hold was ramped at 3 m/s^2
    (review, 2026-10-19). The Shield's ObstacleClearance stayed
    downstream; the guard in front of it was degraded.

    A building ring counts only while the building half brakes: city streets
    keep 4-5 m clearance for whole flights, so a ring test on every tick would
    make every fence brake bite at once. gate() already reports blocked inside
    a zone's stand-off; the stand-off test is kept so this function says what
    it promises whatever gate()'s blocked flag later becomes."""
    if fblocked:
        return True
    od = fence.last_obstacle_d
    if od is not None and od <= fence.min_clearance_m:
        return True
    fd = fence.last_fence_d
    return fd is not None and fd <= fence.stand_off_m


class GuardSmoother:
    """Ramps FenceGuard's output between ticks; gate() and slide() stay pure.

    gate() answers each tick afresh, from a forecast over 11 samples, and a
    heading change of 2-4 deg beside a wall at 5.2 m flips its scale between
    1.00, 0.55 and 0.27 (synthetic check, verifier of finding
    fenceguard-chatter-lunge); slide() adds a 0.8-2.5 m/s sidestep on or off.
    The command then stepped 2-4 m/s in one tick. Their tests pin single-call
    values, and those values are right; what was missing is memory between
    calls, so it lives here, in the loop.

      * The speed scale falls AT ONCE when guard_urgent() says so - a hold is
        never softened - and otherwise by at most `engage` m/s^2 of speed;
        it rises by at most `release` m/s^2. Expressed in speed, not in scale
        fractions: at 5 m/s a 0.1-per-tick scale slew IS the old 4 m/s^2
        limiter, and a replay of the t=214 s release with it gave the same
        peak and the same 8 saturated ticks as before.
      * The sidestep vector fades in and out at `lat` m/s^2.
      * With the guard clear (scale 1, no sidestep) it is exactly a pass-
        through, so it adds no lag to an ordinary follow.
    """

    V_FLOOR = 1.0       # m/s: the speed a scale slew is measured against near a stop

    def __init__(self, release: float = 2.0, engage: float = 3.0,
                 lat: float = 2.0, tick: float = TICK):
        self.release, self.engage, self.lat_acc = float(release), float(engage), float(lat)
        self.tick = float(tick)
        self.scale = 1.0
        self.lat = (0.0, 0.0)
        self.t = None
        self.n_ramped = 0           # ticks whose output differed from the guard's
        self.n_urgent = 0           # ticks the brake was let bite at once

    def __call__(self, now: float, cvx: float, cvy: float, fscale: float,
                 lat_x: float = 0.0, lat_y: float = 0.0,
                 urgent: bool = False) -> tuple[float, float]:
        dt = (self.tick if self.t is None
              else min(max(now - self.t, 0.0), self.tick))
        self.t = now
        sp = max(self.V_FLOOR, math.hypot(cvx, cvy))
        if fscale < self.scale:
            if urgent:
                self.scale = float(fscale)
                self.n_urgent += 1
            else:
                self.scale = max(float(fscale), self.scale - self.engage * dt / sp)
        else:
            self.scale = min(float(fscale), self.scale + self.release * dt / sp)
        ddx, ddy = lat_x - self.lat[0], lat_y - self.lat[1]
        n, lim = math.hypot(ddx, ddy), self.lat_acc * dt
        if n > lim:
            ddx, ddy = ddx * lim / n, ddy * lim / n
        self.lat = (self.lat[0] + ddx, self.lat[1] + ddy)
        gx = self.scale * cvx + self.lat[0]
        gy = self.scale * cvy + self.lat[1]
        if abs(gx - (fscale * cvx + lat_x)) > 1e-6 or abs(gy - (fscale * cvy + lat_y)) > 1e-6:
            self.n_ramped += 1
        return gx, gy

    def stats(self) -> dict:
        return {"ramped_ticks": self.n_ramped, "urgent_ticks": self.n_urgent}


def standoff_speed(rng: float, want: float, gain: float, ff: float,
                   speed_max: float, catchup_mps: float = 0.0) -> float:
    """Forward speed for the stand-off loop: P on the range error plus the
    subject's own opening rate fed forward, clipped to [-0.4, 1] x speed_max.

    `catchup_mps` > 0 SATURATES the catch-up half of the P term:
    catchup * tanh(e / catchup). Small errors are unchanged (slope `gain`);
    a large lag asks for at most `catchup_mps` above the subject's own speed
    instead of jumping to speed_max. Linear, a 13 m lag after the car pulled
    away from a stop asked 3.3 m/s + ff and hit the 5 m/s cap at once (241
    ticks at the cap on citylife_redcar_nfz_20261013), which is the pitch-down
    surge. With the car at cruise ff carries it, so the closing speed - and
    the follow - is what it was (13 Oct: within 30 m of the car 99.1 % of the
    time). Backing off (a negative error) stays linear: too close is the side
    a stand-off is for. 0 restores the linear law exactly.
    """
    e = (rng - want) * gain
    if catchup_mps > 0.0 and e > 0.0:
        e = catchup_mps * math.tanh(e / catchup_mps)
    return float(np.clip(e + ff, -0.4 * speed_max, speed_max))


def los_rate(sx: float, sy: float, svx: float, svy: float,
             x: float, y: float, vx: float, vy: float,
             r_min: float = 0.0) -> float:
    """Rate (rad/s, NED yaw sense, + clockwise) at which the line of sight from
    the aircraft (x, y, moving vx, vy) to the subject (sx, sy, moving svx, svy)
    turns. Fed forward into the yaw servo it removes the turn lag: a P servo on
    a line of sight turning at w settles w / gain behind it (13 Oct: truth
    bearing = 0.98 s x LOS rate, 13.9 deg median / 18.0 p90 in turns; 2.6 /
    5.9 deg once w / 1.2 is taken out). 0 within a metre.

    `r_min` floors the range it divides by: the rate is v_across / max(r,
    r_min). The rate is a velocity over a range, so the estimate's velocity
    noise is amplified as 1/r: 0.5 m/s of noise is 0.025 rad/s at the 13 Oct
    car's 20 m and 0.25 rad/s at a pedestrian's 2 m (0.175 after the 0.7
    gain), a jittering nose on every person mission, which nobody has flown
    or replayed (review, 2026-10-19). Floored at 10 m that noise stays at
    0.05 rad/s, and the rate beyond r_min - a car follow, held at 15.8 m - is
    unchanged.
    0 is the bare rate."""
    dx, dy = sx - x, sy - y
    r2 = dx * dx + dy * dy
    if r2 < 1.0:
        return 0.0
    cross = dx * (svy - vy) - dy * (svx - vx)
    if r_min > 0.0:
        r = math.sqrt(r2)
        return cross / (r * max(r, float(r_min)))
    return cross / r2


def motion_params(args) -> dict:
    """The motion settings a flight ran with, for metrics.json params."""
    if getattr(args, "legacy_motion", False):
        return {"legacy": True, "limiter": "per-axis-per-call",
                "dv_h": args.dv_h, "dv_z": args.dv_z,
                "yaw_accel_rps2": None, "guard_smoother": None,
                "catchup_mps": 0.0, "yaw_los_ff": 0.0}
    return {"legacy": False, "limiter": "vector-dt",
            "accel_mps2": args.accel_max, "brake_mps2": args.brake_max,
            "turn_accel_mps2": args.turn_accel_max,
            "vz_accel_mps2": round(args.dv_z / TICK, 3),
            "brake_floor_dv_h": args.dv_h, "vz_floor_dv": args.dv_z,
            "yaw_accel_rps2": args.yaw_accel_max,
            "guard_smoother": {"release_mps2": args.guard_release_accel,
                               "engage_mps2": args.guard_engage_accel,
                               "lat_mps2": args.guard_lat_accel},
            "catchup_mps": args.catchup_mps,
            "yaw_los_ff": args.yaw_los_ff, "yaw_los_ff_cap": args.yaw_los_ff_cap,
            "yaw_los_ff_min_range_m": args.yaw_los_ff_min_range}


def add_motion_args(ap) -> None:
    """The motion-shaping flags (2026-10-19), on `ap`. Their defaults are the
    flown defaults; --legacy-motion ignores all of them."""
    ap.add_argument("--legacy-motion", action="store_true",
                    help="fly the pre-2026-10-19 motion exactly: the per-axis, "
                         "per-call RateLimiter (--dv-h/--dv-z), no guard "
                         "smoothing, the linear stand-off law up to "
                         "--speed-max and no yaw feed-forward. That is the "
                         "motion half of reproducing a recorded flight; no "
                         "--legacy-* flag restores the search sweep's "
                         "first-swing side, the level-crowd truth poll's "
                         "gating (it changes the tick cadence on car "
                         "missions), the identity rules for taxi/sedan/SUV/"
                         "pickup/lorry phrases, the non-identity HUD label and "
                         "box age, or det_hit_rate's mission-only count.")
    ap.add_argument("--accel-max", type=float, default=2.5,
                    help="m/s^2: how fast the commanded speed may RISE along "
                         "the track. Body tilt follows horizontal "
                         "acceleration (atan(a/g)): 2.5 is ~14.3 deg. The old "
                         "limiter allowed 4 per axis, 5.7 on a diagonal. 2.0 "
                         "(~11.5 deg) was flown on 2026-10-19 "
                         "(citylife_redcar_nfz_motion_20261019): tilt never "
                         "over 15 deg, but the car got beyond 30 m on 6 %% of "
                         "the ticks (0.94 within 30 m, against 0.991 on 13 "
                         "Oct) - every time while catching up after the start "
                         "or a FenceGuard sidestep.")
    ap.add_argument("--brake-max", type=float, default=4.0,
                    help="m/s^2: how fast it may FALL along the track. When "
                         "braking (the step pointing back along the track, as "
                         "every hold does), never slower than the old per-axis "
                         "step (--dv-h); a turn gets no such floor.")
    ap.add_argument("--turn-accel-max", type=float, default=2.5,
                    help="m/s^2 across the track: how fast the velocity may "
                         "turn (a 4 m/s follow round a 0.5 rad/s corner needs 2)")
    ap.add_argument("--yaw-accel-max", type=float, default=2.0,
                    help="rad/s^2 slew on the commanded yaw rate (0 = none). "
                         "Nothing limited it before; a coast-to-search "
                         "hand-over stepped it 1.2 rad/s in one tick.")
    ap.add_argument("--guard-release-accel", type=float, default=2.5,
                    help="m/s^2: how fast speed comes back after a FenceGuard "
                         "brake (GuardSmoother)")
    ap.add_argument("--guard-engage-accel", type=float, default=3.0,
                    help="m/s^2: how fast a NON-urgent guard brake bites; an "
                         "urgent one (blocked, inside the clearance ring of the "
                         "building it brakes for, or a zone's stand-off) bites "
                         "at once")
    ap.add_argument("--guard-lat-accel", type=float, default=2.0,
                    help="m/s^2: how fast slide()'s sidestep fades in and out")
    ap.add_argument("--catchup-mps", type=float, default=3.0,
                    help="the stand-off loop asks at most this much above the "
                         "subject's own opening rate when it has fallen behind "
                         "(saturating, standoff_speed). 0 = the old linear law, "
                         "which chased a 13 m lag at --speed-max.")
    ap.add_argument("--yaw-los-ff", type=float, default=0.7,
                    help="fraction of the line-of-sight rate (from the "
                         "estimate's velocity) fed forward into the yaw servo, "
                         "0 = off. Replayed on citylife_redcar_nfz_20261013 "
                         "the estimate's LOS rate tracks the truth's (r 0.98) "
                         "and 0.7 takes the turn lag from 13.9/18.0 deg "
                         "(median/p90) to 5.4/8.3.")
    ap.add_argument("--yaw-los-ff-cap", type=float, default=0.4,
                    help="rad/s cap on that feed-forward")
    ap.add_argument("--yaw-los-ff-min-range", type=float, default=10.0,
                    help="m: the feed-forward's line-of-sight rate divides by "
                         "at least this range (los_rate r_min). The rate is a "
                         "velocity over a range, so at a pedestrian's 2-10 m "
                         "the estimate's velocity noise would jitter the nose "
                         "(0.5 m/s noise: 0.25 rad/s at 2 m); validated only "
                         "on a car at 15-25 m, which this leaves unchanged. "
                         "0 = the bare rate.")


def make_motion(args):
    """(limiter, guard smoother or None) for the flight's motion settings."""
    if getattr(args, "legacy_motion", False):
        return RateLimiter(args.dv_h, args.dv_z), None
    lim = VecSlewLimiter(accel=args.accel_max, brake=args.brake_max,
                         lat=args.turn_accel_max, vz=args.dv_z / TICK,
                         yaw=args.yaw_accel_max, brake_floor_dv=args.dv_h,
                         vz_floor_dv=args.dv_z)
    gs = GuardSmoother(release=args.guard_release_accel,
                       engage=args.guard_engage_accel, lat=args.guard_lat_accel)
    return lim, gs


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


def negative_suppress(pos, neg, thresh: float):
    """Each box's score for the subject phrase with the NEGATIVE queries asked
    beside it: `(scores, n_suppressed, top)`.

    `pos[i]` is box i's sigmoid score for the subject phrase, `neg[i]` its
    best score over the negative phrases ("a traffic light"), both from the
    same forward pass. OWL-ViT scores each (box, phrase) pair on its own (a
    sigmoid per pair, not a softmax over phrases), so asking a negative phrase
    leaves every box's subject score as it was: a box keeps it unless the
    negative phrase scores HIGHER on it - it looks more like a traffic light
    than like a red car - and then scores 0 and can never pass the detector
    threshold. `n_suppressed` counts the boxes that would have passed
    `thresh` and were taken away; `top` is the index of the best-scoring of
    them (None when none). With no negative phrase, pass `neg=None`: the
    scores come back unchanged.

    Added 2026-10-21 for the ArduPilot rail (demo/pas_ardupilot/
    perception_node.py --negative-query): on the 21 Oct flight the detector's
    box sat on a red traffic signal on every estimate (0/186 within 6 m of
    the car)."""
    if neg is None:
        return list(pos), 0, None
    out, n, top = [], 0, None
    for i, (p, q) in enumerate(zip(pos, neg)):
        p, q = float(p), float(q)
        if q > p:
            if p >= thresh:
                n += 1
                if top is None or p > float(pos[top]):
                    top = i
            out.append(0.0)
        else:
            out.append(p)
    return out, n, top


def subject_scores_with_negatives(logits, scores, labels, thresh: float):
    """The detector's score per box for the SUBJECT phrase when negative
    phrases were asked beside it (--negative-query): `(scores, record,
    n_suppressed)`, one score per box post-processing kept.

    `logits`: OWL-ViT's class logits for the image (out.logits[0]), one row
    per box and one column per phrase in the order Grounder._worker asks
    them - column 0 the subject, 1.. the negatives. `scores` / `labels`:
    post_process_object_detection's answer at threshold 0.0 for the same
    image, per box it kept the best sigmoid over the phrases and the index
    of the phrase that won. That max may be a negative phrase's, so the
    subject's own sigmoid is read from column 0 and negative_suppress zeroes
    a box the negatives win. At threshold 0.0 post-processing keeps every
    box in the logits' order, so row i is box i - unless it dropped one (a
    score of exactly 0.0, a sigmoid underflow): the rows then cannot be
    paired with the boxes, so the labels decide (a box a negative phrase won
    scores 0, the rest keep post-processing's score) and the count is
    unknown (`suppressed` None).

    `record`: suppressed (int, or None when unknown), top_index (the best-
    scoring box taken away, or None), top_pos / top_neg (its subject and
    negative scores), count_unknown (why the count is unknown, or None).
    Pure, on CPU arrays (a CPU torch tensor reads as one): moved out of the
    worker so tests/test_pas_ardupilot.py can check the column, the
    suppression and the fallback without a GPU (review 2026-10-22: in the
    worker they had no test, and a wrong column would have failed silently,
    on the GPU only)."""
    lg = np.asarray(logits, dtype=np.float64)
    if lg.ndim != 2 or lg.shape[1] < 2:
        raise ValueError(f"logits of shape {lg.shape}: one row per box, the subject's "
                         f"column and at least one negative phrase's")
    sc = [float(s) for s in scores]
    lb = [int(v) for v in labels]
    if lg.shape[0] != len(sc):
        kept = [s if lab == 0 else 0.0 for s, lab in zip(sc, lb)]
        return kept, {"suppressed": None, "top_index": None, "top_pos": None,
                      "top_neg": None,
                      "count_unknown": f"{len(sc)} boxes for {lg.shape[0]} logits"}, 0
    # The sigmoid as exp(-log(1 + e^-x)): no overflow for a very negative logit.
    sig = np.exp(-np.logaddexp(0.0, -lg))
    pos, neg = sig[:, 0], sig[:, 1:].max(axis=1)
    kept, n_sup, top = negative_suppress(pos.tolist(), neg.tolist(), thresh)
    return kept, {"suppressed": n_sup, "top_index": top,
                  "top_pos": None if top is None else float(pos[top]),
                  "top_neg": None if top is None else float(neg[top]),
                  "count_unknown": None}, n_sup


# THE PEOPLE PHRASE (2026-11-18, WP1-07). For a distance_envelope over people
# the perception rail asks the detector for every person in the frame, in the
# subject's own forward pass (Grounder(people_query=...)): the phrase, the
# score a box needs for it, the overlap above which two boxes are one person
# (non-maximum suppression over OWL-ViT's per-patch boxes), and how many a
# frame may give. The threshold is not tuned on flight data - no flight has
# asked for people yet - so a flight records every box it kept (detections.
# jsonl `people`) and the rail's record how many fixes became tracks.
PEOPLE_QUERY = "a person"
PEOPLE_DET_THRESH = 0.10
PEOPLE_NMS_IOU = 0.3
PEOPLE_MAX_BOXES = 32


def subject_column_scores(logits, scores, labels) -> list:
    """The subject phrase's own score per box when the people phrase (and no
    negative phrase) was asked beside it: post-processing keeps the best
    sigmoid over the phrases, which may be the people phrase's, so the
    subject's is read from its column (0) of the logits, as
    subject_scores_with_negatives reads it. When post-processing dropped a
    box (the rows cannot be paired) the labels decide: a box another phrase
    won scores 0 for the subject, the rest keep their score."""
    lg = np.asarray(logits, dtype=np.float64)
    sc = [float(v) for v in scores]
    if lg.ndim != 2 or lg.shape[0] != len(sc):
        return [v if int(lab) == 0 else 0.0 for v, lab in zip(sc, labels)]
    return np.exp(-np.logaddexp(0.0, -lg[:, 0])).tolist()


def people_boxes(logits, boxes, col: int, thresh: float, W: int, H: int,
                 iou: float = PEOPLE_NMS_IOU, max_boxes: int = PEOPLE_MAX_BOXES):
    """Every person in one inference: `(boxes, note)`.

    `logits`: OWL-ViT's class logits for the image (one row per box, one
    column per phrase); `col` is the people phrase's column. `boxes`: the
    post-processed (x0, y0, x1, y1) per box, row i = box i (post-processing at
    threshold 0.0 keeps every box in the logits' order unless it dropped one;
    then the rows cannot be paired, the answer is [] and `note` says so).
    Each box whose people-phrase sigmoid is >= `thresh` is a candidate;
    greedy non-maximum suppression (best first, IoU > `iou` with a kept box
    is the same person) leaves at most `max_boxes`, best first, each as the
    7-tuple the follow places a box from: (cx, cy, w, h, score, W, H).
    Pure, on CPU arrays, so tests/test_people_envelope.py checks it without
    a GPU."""
    lg = np.asarray(logits, dtype=np.float64)
    bx = np.asarray(boxes, dtype=np.float64).reshape(-1, 4)
    if lg.ndim != 2 or lg.shape[1] <= col:
        raise ValueError(f"logits of shape {lg.shape} have no column {col}")
    if lg.shape[0] != bx.shape[0]:
        return [], f"{bx.shape[0]} boxes for {lg.shape[0]} logits: not paired"
    sig = np.exp(-np.logaddexp(0.0, -lg[:, col]))
    order = [int(i) for i in np.argsort(-sig, kind="stable") if sig[i] >= thresh]
    kept: list[int] = []
    area = np.maximum(bx[:, 2] - bx[:, 0], 0.0) * np.maximum(bx[:, 3] - bx[:, 1], 0.0)
    for i in order:
        if len(kept) >= max_boxes:
            break
        ok = True
        for j in kept:
            ix = max(0.0, min(bx[i, 2], bx[j, 2]) - max(bx[i, 0], bx[j, 0]))
            iy = max(0.0, min(bx[i, 3], bx[j, 3]) - max(bx[i, 1], bx[j, 1]))
            inter = ix * iy
            union = area[i] + area[j] - inter
            if union > 0 and inter / union > iou:
                ok = False
                break
        if ok:
            kept.append(i)
    out = []
    for i in kept:
        x0, y0, x1, y1 = (float(v) for v in bx[i])
        out.append(((x0 + x1) / 2.0, (y0 + y1) / 2.0, x1 - x0, y1 - y0,
                    float(sig[i]), int(W), int(H)))
    return out, None


# How many of the detector's best boxes the screen looks at, per inference.
SCREEN_TOP_K = 12
# How long a published box keeps the jump gate shut on a box too far from it.
JUMP_GATE_S = 1.5


def _box_place(b) -> list:
    """[cx, cy, w, h] of an (x0, y0, x1, y1) box, to 0.1 px: where a dropped
    box was, for detections.jsonl `drops`."""
    x0, y0, x1, y1 = [float(v) for v in b]
    return [round((x0 + x1) / 2, 1), round((y0 + y1) / 2, 1), round(x1 - x0, 1),
            round(y1 - y0, 1)]


def screen_inputs(sc, bx, thresh: float):
    """What the screen needs of one inference's boxes (torch tensors, any
    device): `(top_scores, top_boxes, above_thresh, below)`. The top are the
    SCREEN_TOP_K best, best first, as plain lists - argsort on the tensors'
    own device, as the worker's loop always did, so ties keep that order;
    `above_thresh` counts every box at or over `thresh`, and `below` is
    (score, (x0, y0, x1, y1)) of the best box under it, or None - over all
    the boxes, on the tensors, so only SCREEN_TOP_K of them become Python
    lists (OWL-ViT base/32 gives 576 per frame)."""
    if not len(sc):
        return [], [], 0, None
    top = sc.argsort(descending=True)[:SCREEN_TOP_K]
    sd = sc.double()                        # as the screen compares: in double
    under = sd < float(thresh)
    n_under = int(under.sum())
    below = None
    if n_under:
        j = int(sd.masked_fill(~under, float("-inf")).argmax())
        below = (float(sc[j]), [float(v) for v in bx[j].tolist()])
    return sc[top].tolist(), bx[top].tolist(), int(len(sc)) - n_under, below


def screen_candidates(scores, boxes, thresh: float, W: int, H: int, *,
                      above_thresh: int, below=None,
                      last=None, acquiring: bool = False, jump_frac: float = 0.35,
                      colour_of=None, colour_gated: bool = False,
                      colour_min: float = 0.0, clock=time.time):
    """The detector's screen before any gate (Grounder._worker): `(cands,
    drops)`.

    `scores` / `boxes`: the SCREEN_TOP_K best boxes' subject scores and
    (x0, y0, x1, y1), best first (screen_inputs). Walking them, the screen
    stops at the first box under `thresh`; a box more than `jump_frac` of
    the frame from the last published box (`last` = (cx, t)) within
    JUMP_GATE_S of it, while not acquiring, is skipped (the jump gate);
    `colour_of(box)` is measured on the rest, and with `colour_gated` a box
    under `colour_min` is skipped (right shape, wrong colour). Each
    survivor is (cx, cy, w, h, score, W, H, colour), in order.

    `drops` (detections.jsonl `drops`) says which stage took boxes away,
    without changing what passes: `thresh`; `above_thresh`, how many of ALL
    the inference's boxes were at or over it (screen_inputs; only the first
    SCREEN_TOP_K are screened); `below_thresh_best` and `below_thresh_box`
    ([cx, cy, w, h]), the best score of ANY box under the threshold and
    where it was (None: no box was under it; a box a negative phrase won
    scores 0, see subject_scores_with_negatives); `jump`, how many the jump
    gate skipped; and `colour` - {n, best_score, best_colour, best_box} of
    the boxes the colour gate skipped (best = the highest detector score
    among them), None when it skipped none. None of these says the box was
    the subject: with ~575 patch boxes under det_thresh the best of them is
    usually background, and telling whether a dropped box was the car needs
    the car's own image box (e.g. the previous inference's `id` box at the
    same place), which this row does not hold."""
    cands = []
    n_jump = 0
    col = []
    for s, b in zip(scores, boxes):
        s = float(s)
        if s < thresh:
            break
        x0, y0, x1, y1 = [float(v) for v in b]
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        if (last is not None and not acquiring
                and (clock() - last[1]) < JUMP_GATE_S):
            if abs(cx - last[0]) > jump_frac * W:
                n_jump += 1
                continue          # too far from where it just was
        cm = colour_of((x0, y0, x1, y1)) if colour_of is not None else 0.0
        if colour_gated and cm < colour_min:
            col.append((s, float(cm), (x0, y0, x1, y1)))
            continue              # right shape, wrong colour
        cands.append((cx, cy, x1 - x0, y1 - y0, s, W, H, cm))
    best_col = max(col, key=lambda c: c[0]) if col else None
    drops = {"thresh": thresh,
             "above_thresh": int(above_thresh),
             "below_thresh_best": None if below is None else round(float(below[0]), 4),
             "below_thresh_box": None if below is None else _box_place(below[1]),
             "jump": n_jump,
             "colour": (None if not col else
                        {"n": len(col), "best_score": round(best_col[0], 4),
                         "best_colour": round(best_col[1], 3),
                         "best_box": _box_place(best_col[2])})}
    return cands, drops


# The frame's mean grey level in detections.jsonl (`luma`, 0-255), from a
# copy LUMA_STEP times smaller each way (768x432 -> 96x54): what tells a
# perception-stress run that was really dark (demo/pas_ardupilot/
# perception_stress.py, dusk / night) from one that only said so, and what
# tools/perception_stress_report.py compares across scenarios.
LUMA_STEP = 8


def frame_luma(img, step: int = LUMA_STEP):
    """Mean grey level (0-255, one decimal) of a PIL image, from a copy
    reduced `step` times each way; None when it cannot be measured. Never
    raises: it rides on the detector's log line, not its decision."""
    try:
        small = img.reduce(int(step)) if step and int(step) > 1 else img
        grey = np.asarray(small.convert("L"), dtype=np.float32)
        if grey.size == 0:
            return None
        return round(float(grey.mean()), 1)
    except Exception:                                           # noqa: BLE001
        return None


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
                 identity: dict | None = None, negatives: list | None = None,
                 teacher=None, teacher_dims: tuple = (4.0, 1.5),
                 people_query: str | None = None,
                 people_thresh: float = PEOPLE_DET_THRESH):
        # --teacher-truth (teacher_truth_fn; 2026-10-30): a TEACHER for
        # training labels, never a follow result. Each frame's candidates are
        # replaced by ONE box drawn where the subject TRULY is (_teacher_box)
        # - through walls, every frame it is in the field of view - with its
        # true range and bearing as its identity features. The model still
        # runs (the detector's cadence and load as in a real flight); its
        # boxes are not used. None: every behaviour below exactly.
        self.teacher = teacher
        self.teacher_dims = (float(teacher_dims[0]), float(teacher_dims[1]))
        self.n_teacher = {"in_frame": 0, "out_of_frame": 0, "no_truth": 0, "no_pose": 0}
        # NEGATIVE PHRASES (negative_suppress; 2026-10-21). Asked beside the
        # subject phrase in the same forward pass; a box that scores higher
        # for one of them than for the subject is no candidate. None or []
        # keeps the single-phrase query exactly.
        self.negatives = [str(n) for n in (negatives or []) if str(n).strip()]
        self.n_neg_suppressed = 0
        # THE PEOPLE PHRASE (people_boxes; 2026-11-18): asked LAST, after the
        # subject and the negatives, in the same forward pass, for every
        # person in the frame (a distance_envelope over people). None keeps
        # the query list, the subject's scores and latest() exactly.
        self.people_query = (str(people_query).strip() or None) if people_query else None
        self.people_thresh = float(people_thresh)
        self._people_pub = None
        self.n_people_frames = 0
        self.n_people_boxes = 0
        self.n_people_unpaired = 0
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

    def _phrases(self) -> list:
        """The phrase list one forward pass asks: the subject first (index
        0, the one candidates come from), the negative phrases, then the
        people phrase when there is one."""
        return [[self.query] + self.negatives
                + ([self.people_query] if self.people_query else [])]

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
        # The subject phrase first: index 0 is the one candidates come from;
        # the negative phrases, if any, follow it (negative_suppress), and the
        # people phrase last (people_boxes).
        queries = self._phrases()
        gen = self._query_gen
        last = None
        while not self._stop:
            # Pick up a retarget. Cheap: an int compare per frame, and the
            # rebuild only runs on the tick the phrase actually changed.
            if self._query_gen != gen:
                with self._lock:
                    gen = self._query_gen
                    queries = self._phrases()
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
            neg_rec = None
            lg_np = (out.logits[0].float().cpu().numpy()
                     if self.people_query else None)
            if self.negatives:
                # post_process takes the max over the phrases; with a negative
                # phrase that max may be the negative's. The subject's own
                # score per box from the logits instead, and negative_suppress
                # (subject_scores_with_negatives: the column, the suppression
                # and the fallback when post_process dropped a box, tested on
                # CPU arrays).
                kept, nr, n_sup = subject_scores_with_negatives(
                    (out.logits[0].float().cpu().numpy() if lg_np is None
                     else lg_np[:, :1 + len(self.negatives)]), sc.tolist(),
                    res["labels"].tolist(), self.thresh)
                sc = torch.tensor(kept, device=sc.device, dtype=sc.dtype)
                self.n_neg_suppressed += n_sup
                top = nr["top_index"]
                neg_rec = ({"queries": list(self.negatives), "suppressed": None,
                            "count_unknown": nr["count_unknown"]}
                           if nr["suppressed"] is None else
                           {"queries": list(self.negatives), "suppressed": n_sup,
                            "top": (None if top is None else
                                    [round(float(v), 1) for v in bx[top].tolist()]
                                    + [round(nr["top_pos"], 4), round(nr["top_neg"], 4)])})
            elif self.people_query:
                # The people phrase alone beside the subject: the subject's
                # own column, not post-processing's best over both phrases.
                sc = torch.tensor(subject_column_scores(lg_np, sc.tolist(),
                                                        res["labels"].tolist()),
                                  device=sc.device, dtype=sc.dtype)
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
            people_pub = None
            if self.people_query:
                # Every person in the frame (people_boxes), each with its range
                # in THIS capture's depth frame, as the subject box's is read.
                pb, p_note = people_boxes(lg_np, bx.cpu().numpy(), lg_np.shape[1] - 1,
                                          self.people_thresh, W, H)
                if p_note is not None:
                    self.n_people_unpaired += 1
                people_pub = {"t_cap": t_cap, "t_capture": t_capture, "pose": cap_pose,
                              "depth_src": dep_src, "note": p_note,
                              "boxes": [b + ((None if dep is None
                                              else range_from_depth(dep, b)),)
                                        for b in pb]}
            det, switched = None, False
            # Score every plausible box, do not just take the detector's top
            # one. Ranking by detector score alone is what let a city object
            # win and hold the aircraft 60 m from the car. The screen
            # (threshold, jump gate, colour gate) is screen_candidates, which
            # also says what it dropped and why (detections.jsonl `drops`).
            top_s, top_b, n_above, below = screen_inputs(sc, bx, self.thresh)
            cands, drops = screen_candidates(
                top_s, top_b, self.thresh, W, H, above_thresh=n_above, below=below,
                last=last, acquiring=acquiring, jump_frac=self.jump_frac,
                colour_of=lambda b: colour_match(img, b, self.colour, depth=dep,
                                                 stats=self.mask_stats,
                                                 legacy_sat=self.legacy_sat),
                colour_gated=self.colour is not None, colour_min=self.colour_min,
                clock=time.time)
            teacher_feat = None
            if self.teacher is not None:
                tb = self._teacher_box(cap_pose, t_capture, W, H)
                cands = [tb[0]] if tb is not None else []
                teacher_feat = tb[1] if tb is not None else None
            id_all, tiers, feats, det_tier, det_feat, lock_why = [], [], [], None, None, None
            if cands and self.teacher is not None:
                # THE box, with its true range and bearing (identity's feature
                # names, which the start gate, the re-acquirer and the
                # estimator read); no lock: there is no other box. While
                # acquiring the caller's gate judges it as any candidate.
                tiers, feats = ["ok"], [teacher_feat]
                id_all = [(cands[0], "ok", ["teacher: truth"], teacher_feat)]
                if not acquiring:
                    det, det_tier, det_feat = cands[0], "ok", teacher_feat
            elif cands and self.identity is not None:
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
            # The box's range in THIS capture's depth frame (published as
            # cap["range_m"] below), read before the lock: the control loop's
            # latest() waits on that lock every tick, and a median over the
            # box's window has no reason to hold it.
            det_range = (None if det is None or dep is None
                         else range_from_depth(dep, det))
            if self.teacher is not None and det is not None:
                det_range = teacher_feat["rng_h"]        # not the depth of a wall
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
                if people_pub is not None:
                    self._people_pub = dict(people_pub, seq=self._seq, t=self._t)
                    self.n_people_frames += 1
                    self.n_people_boxes += len(people_pub["boxes"])
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
                    # The box's range in THIS capture's own depth frame
                    # (det_range, read before the lock), published under two
                    # keys for its two readers: cap["range_m"] for
                    # demo/pas_ardupilot/perception_node.py, which places the
                    # box up to a second later, and cap["rng"] for this
                    # file's control loop (estimator_measurement), which never
                    # sees `dep`. dep is only ever a paired frame (dep_src
                    # 'capture' or 'capture-late'; None when unpaired), so
                    # neither reader indexes the box into a newer frame.
                    self._det_cap = {"t_cap": t_cap, "t_capture": t_capture,
                                     "pose": cap_pose,
                                     "depth_src": dep_src,
                                     "depth_dt_ms": dep_dt,
                                     "range_m": det_range,
                                     "rng": det_range}
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
                       # The frame's mean grey level (frame_luma): how dark
                       # what the detector saw was.
                       "luma": frame_luma(img),
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
                # Which stage took boxes away before any gate
                # (screen_candidates): how many boxes were over det_thresh,
                # the best one under it and where, and what the jump and
                # colour gates dropped (count, best score, place) - so an
                # inference with an empty `id` list says which stage emptied
                # it. Whether a dropped box was the car is not in the row:
                # that needs the car's own image box.
                rec["drops"] = drops
                if neg_rec is not None:
                    # What the negative phrases took away on this frame: how
                    # many boxes over the threshold, and the best of them
                    # (x0, y0, x1, y1, subject score, negative score).
                    rec["neg"] = neg_rec
                if people_pub is not None:
                    # Every person box kept (people_boxes): cx, cy, w, h, the
                    # people phrase's score, and the range in this frame's depth.
                    rec["people"] = [[round(b[0], 1), round(b[1], 1), round(b[2], 1),
                                      round(b[3], 1), round(b[4], 4),
                                      None if b[7] is None else round(b[7], 2)]
                                     for b in people_pub["boxes"]]
                    if people_pub["note"] is not None:
                        rec["people_note"] = people_pub["note"]
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

    def _teacher_box(self, pose, t_cap, W: int, H: int):
        """(8-tuple candidate, features) of the subject's TRUE position in this
        frame (--teacher-truth), or None: no capture pose, no truth yet, or
        outside the field of view. The box spans the subject's ground point
        and the point teacher_dims[1] above it through the capture's camera
        (camera_model.world_to_pixel: the mount, pitch and roll), teacher_dims[0]
        wide at its slant range; score and colour 1. The features are what
        identity's would be for a perfect measurement: P (north, east), rng_h,
        r (slant), bearing (world azimuth to the subject)."""
        if pose is None or pose.get("up") is None:
            self.n_teacher["no_pose"] += 1
            return None
        p = self.teacher(t_cap)
        if p is None:
            self.n_teacher["no_truth"] += 1
            return None
        hfov = (self.identity or {}).get("hfov", CAMERA_HFOV_DEG)
        width_m, height_m = self.teacher_dims
        cam = (pose["x"], pose["y"], pose["up"])
        att = (pose["yaw"], pose.get("pitch") or 0.0, pose.get("roll") or 0.0)
        bot = _cam.world_to_pixel(p[0], p[1], 0.0, *cam, W, H, hfov, *att)
        top = _cam.world_to_pixel(p[0], p[1], height_m, *cam, W, H, hfov, *att)
        if bot is None or top is None:
            self.n_teacher["out_of_frame"] += 1
            return None
        cx = (bot[0] + top[0]) / 2.0
        y0, y1 = top[1], bot[1]
        if not (0.0 <= cx < W) or y1 < 0.0 or y0 >= H:
            self.n_teacher["out_of_frame"] += 1
            return None
        rng_h = math.hypot(p[0] - cam[0], p[1] - cam[1])
        slant = math.hypot(rng_h, cam[2] - height_m / 2.0)
        w = _cam.focal_px(W, hfov) * width_m / max(slant, 1.0)
        self.n_teacher["in_frame"] += 1
        feat = {"P": (float(p[0]), float(p[1])), "rng_h": rng_h, "r": slant,
                "bearing": math.atan2(p[1] - cam[1], p[0] - cam[0])}
        return (cx, (y0 + y1) / 2.0, w, y1 - y0, 1.0, W, H, 1.0), feat

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
            out = {"seq": self._seq, "t": self._t, "t_det": self._t_det,
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
            if self.people_query:
                # The last inference's people (people_boxes), with its capture
                # pose and time: for a distance_envelope over people.
                out["people"] = self._people_pub
            return out


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


# THE CITY DEMO'S FORECAST (finding city-demo-flies-3s-forecast-and-yaml).
#
# This rail builds its Shield at 3 s / 0.5 s (6 forecast steps, the `poses`
# count shield_lookahead_record writes; the grant's 5 s / 0.1 s is 50), not
# the grant's forecast (guardrail.shield LOOKAHEAD_S / LOOKAHEAD_DT_S), and
# nothing in a run's files said so; metrics.json and kpi.json now carry
# `shield_lookahead`. The grant forecast was measured for this rail before
# deciding whether to make it the default (2026-10-19, scratch replay of
# every SECOND tick of citylife_redcar_nfz_20261013 - 1190 of its 2379 -
# through both Shields, the logged commands, the logged estimated subject):
# its cost is small (median 0.24 ms, p99 1.27 ms a tick, against 0.18 /
# 0.32 ms at 3 s / 0.5 s), but it would end the follow. A SubjectStandoff is
# forecast against a subject held STILL - this loop calls set_subject()
# without a velocity (the optional velocity= the Shield gained on 2026-10-19
# is not wired here yet: its theta reading of a hold against an approaching
# subject is open, CHANGELOG 2026-10-19 Not fixed) -
# so 5 s of the aircraft's own 4-5 m/s, ~25 m, reaches the car's 5 m ring from
# ordinary following distance: the Shield's StandoffHold fired on 529 of the
# 1190 replayed ticks (44 %; none at 3 s / 0.5 s) and cut the command by a
# median 3.7 m/s (p90 4.9). The follow would settle behind that ring plus
# 25 m, at the 30 m mark the mission is scored on. `--grant-forecast` flies it
# anyway (with the Shield's own escalation FSM off, as
# docs/DESIGN-shield-enforcement.md asks of any demo that flies `emitted`);
# the default stays what every city flight flew until the Shield forecasts a
# moving subject (the loop's set_subject call says where its velocity goes).
#
# 2026-11-03: the loop now passes the subject's velocity (the estimator's own
# vx, vy) to set_subject, as vla_fly has since 2026-11-01, where 65 replayed
# flights put the grant forecast's stand-off repairs at 1.1 % of ticks with
# the velocity given against 9.1 % held still (CHANGELOG 2026-11-01 late,
# "Decided"). So the grant's 5 s / 0.1 s, with the escalation FSM, is this
# rail's default (WP3-05). `legacy_forecast` builds the 3 s / 0.5 s every city
# flight before 2026-11-03 flew; `grant_forecast` the grant forecast with the
# FSM off, as the --grant-forecast runs before then flew.
def flight_shield(policy, obstacle_map, grant_forecast: bool = False,
                  legacy_forecast: bool = False, road_map: dict | None = None,
                  people_source: str | None = None):
    """The flight's Shield: the grant's 5 s / 0.1 s with the escalation FSM
    (the default since 2026-11-03), 3 s / 0.5 s with the FSM (legacy), or the
    grant's forecast with the FSM off (--grant-forecast). `road_map`: the
    street mask, for a distance_envelope over roads (WP1-07);
    `people_source`: sim_people_source's, for one over people (None: no
    such rule, and the keyword is not passed)."""
    if legacy_forecast and grant_forecast:
        raise ValueError("--legacy-forecast and --grant-forecast are two different "
                         "Shields; give one")
    # PathRepair off on this follow rail (guardrail.replay
    # FOLLOW_RAIL_PATH_REPAIR): the chain it flew before 2026-11-17.
    from guardrail.replay import FOLLOW_RAIL_PATH_REPAIR as pr
    ppl = {} if people_source is None else {"people_source": people_source}
    if legacy_forecast:
        return Shield(policy, lookahead_s=3.0, dt=0.5, obstacle_map=obstacle_map,
                      road_map=road_map, path_repair=pr, **ppl)
    if grant_forecast:
        return Shield(policy, obstacle_map=obstacle_map, escalation=False,
                      road_map=road_map, path_repair=pr, **ppl)
    return Shield(policy, obstacle_map=obstacle_map, road_map=road_map, path_repair=pr,
                  **ppl)


def shield_lookahead_record(shield) -> dict:
    """The forecast this flight's Shield really used, read from the object."""
    return {"horizon_s": shield.lookahead_s, "dt_s": shield.dt,
            "poses": int(round(shield.lookahead_s / shield.dt)),
            "grant": (shield.lookahead_s == LOOKAHEAD_S
                      and shield.dt == LOOKAHEAD_DT_S),
            "escalation_fsm": getattr(shield, "fsm", None) is not None}


def flight_manifest(policy_hash: str, seed: int, sim_address: str | None = None) -> dict:
    """The WP4 six-field manifest for this rail.

    No VLA flies here - the hand-written servo below does - so the model field
    is NO_VLA. It used to be the DETECTOR's id, so every city-rail manifest
    filed OWL-ViT's HuggingFace revision under `vla_model_hash`
    ("google/owlvit-base-patch32@cbc355fb..." on citylife_redcar_nfz_20261013),
    which the grant defines as the hash of the VLA's weights (audit card
    X-16; guardrail/manifest.py NO_VLA). The detector is recorded under its
    own key in `pilot_entry()`. `sim_address` (--sim-address) picks the
    topology label: single-host on loopback, remote-sim when the simulator
    runs on another machine (guardrail.manifest.projectairsim_topology)."""
    from guardrail.manifest import projectairsim_topology
    return build_manifest(policy_hash=policy_hash, model_id=NO_VLA, seed=seed,
                          scene_path=str(ROOT / "demo" / "pas_config" / SCENE),
                          topology=projectairsim_topology(sim_address))


def city_episode(out: Path, policy, lookahead_s: float, start_xy, object_text: str,
                 cruise_alt: float, speed_max: float) -> EpisodeRecord:
    """The city rail's episode record (guardrail.replay.EpisodeRecord), as the
    SITL and perception rails write theirs: events.jsonl from episode_start to
    mission_end, policy_g0.json and the typed CSP csp_g0.json. Until
    2026-10-28 a city flight wrote no CSP at all (audit card WP2-11).

    The mission context is the perception rail's follow mission
    (pas_ardupilot.perception_node.follow_mission): "follow <object>", start
    and target proxy both the position at the mission clock's start, since
    where the subject will be is not known before the flight and ground truth
    may not stand in for it; cruise height and the controller's speed cap.
    The CSP is compiled at the Shield's own forecast horizon (`lookahead_s`).
    No rule clock: this rail's Shield has none, so the record says
    {"kind": "none"} (every scheduled rule in force), which is what flew.
    `rotate=False`: the flight has already written into `out` (fly() clears
    the previous flight's episode files itself, CITY_EPISODE_FILES)."""
    from guardrail.compiler import Mission
    sx, sy = float(start_xy[0]), float(start_xy[1])
    mission = Mission(task_text=f"follow {object_text}", target_x=sx, target_y=sy,
                      cruise_alt_m=float(cruise_alt), speed_pref_mps=float(speed_max),
                      start_x=sx, start_y=sy)
    rec = EpisodeRecord(out, policy, mission, lookahead_s=float(lookahead_s), rotate=False)
    rec.event("mission", task_text=mission.task_text, start=[round(sx, 2), round(sy, 2)],
              target_proxy="start", cruise_alt_m=mission.cruise_alt_m,
              speed_pref_mps=mission.speed_pref_mps)
    return rec


def episode_fields(episode: EpisodeRecord | None) -> dict:
    """The episode id for every flight-log row, metrics.json and kpi.json, as
    the SITL rails stamp theirs; {} for a flight whose record failed.
    guardrail.replay binds the three by it and refuses metrics or a KPI table
    without the id beside a log that carries one: from 2026-10-28 (f6ee433)
    to 2026-11-03 only the rows had it, so no city flight bundled itself."""
    return {} if episode is None else {"episode_id": episode.episode_id}


# The episode files a re-used tag must not inherit (fly() removes them before
# take-off, with the flight log and the detections).
CITY_EPISODE_FILES = ("events.jsonl", "policy_g*.json", "csp_g*.json",
                      "costmap_csp_g*.npz", "costmap_csp_g*.json")


def pilot_entry() -> dict:
    """metrics["pilot"]: what flew (no VLA: a hand-written controller) and the
    detector that steered it, with its HF revision. A missing HF cache must
    not cost a flight that already flew its artefacts, so the hash lookup
    falls back to the bare id."""
    try:
        det_hash = model_hash(DETECTOR_ID)
    except Exception as exc:                                    # noqa: BLE001
        det_hash = f"{DETECTOR_ID}@unresolved ({type(exc).__name__})"
    return pilot_record(NO_VLA, "none",
                        detector={"model_id": DETECTOR_ID, "hash": det_hash})


def loop_timing(stage_ms: dict, cmd_times: list) -> dict:
    """The loop's tail, for metrics.json: per-stage p95 and max, and how the
    commands were actually spaced.

    stage_ms_median stays as it was (tools/build_eval_data.py copies it), but
    a median says nothing about a stall: on citylife_redcar_nfz_20261013 the
    truth stage read 0.0 ms median while every 5th-6th tick spent ~100 ms (max
    1005.7) in it, and the 9.5 Hz loop gate - an average, len(rows)/span -
    passed at 9.9 Hz because the absolute-deadline pacing repaid each late tick
    with an immediate one: 411 command gaps under 20 ms, 422 of 150-300 ms,
    6 over 300 ms. These are the numbers that show it.
    """
    def _pct(v, q):
        return round(float(np.percentile(v, q)), 1) if v else None
    gaps = [b - a for a, b in zip(cmd_times, cmd_times[1:])]
    return {
        "stage_ms_p95": {k: _pct(v, 95) for k, v in stage_ms.items()},
        "stage_ms_max": {k: (round(float(max(v)), 1) if v else None)
                         for k, v in stage_ms.items()},
        "cmd_interval": {"n": len(gaps),
                         "n_under_0_02s": sum(1 for x in gaps if x < 0.02),
                         "n_over_0_15s": sum(1 for x in gaps if x > 0.15),
                         "n_over_0_3s": sum(1 for x in gaps if x > 0.3),
                         "p95_s": (round(float(np.percentile(gaps, 95)), 3)
                                   if gaps else None),
                         "max_s": round(max(gaps), 3) if gaps else None},
    }


def final_report(metrics: dict, n_ticks: int, aborted) -> int:
    """The closing [report] lines, and the exit code.

    A flight that aborted before its first tick has no det_hz (None), and the
    detector check here compared it with 2.0 and raised TypeError - after
    kpi.json was written, and before os._exit could run, so a client thread
    could hold the process open. Without the crash that same flight would have
    returned 0 with a kpi.json saying success: the runner's FLIGHT FAILED
    branch reads the exit code. So: no mission tick, or an abort, is non-zero.
    """
    print(f"\n[report] ticks {n_ticks} | detector {metrics['det_hz']} Hz, "
          f"hit rate {metrics['det_hit_rate']} | target visible on "
          f"{metrics['frac_ticks_seen']*100:.0f}% of ticks")
    print(f"[report] separation: min {metrics['sep_min_m']} m, "
          f"mean {metrics['sep_mean_m']} m, end {metrics['sep_end_m']} m, "
          f"within 30 m {metrics['frac_within_30m']}"
          f"{'' if metrics.get('follow_scored') else '  (follow NOT scored: no subject truth on enough ticks)'}")
    print(f"[report] guardrail: NFZ {metrics['nfz_s']}s, altitude escape "
          f"{metrics['alt_violation_s']}s, interventions {metrics['interventions']}")
    print(f"[report] presence: verdict ABSENT on {metrics['frac_absent']*100:.0f}% "
          f"of ticks (with no target in the scene this should be HIGH)")
    # det_hit_rate is seen/(seen+missed) over the inferences that RAN. If the
    # detector stalls it can read 1.000 on a flight that spent most of its time
    # scanning, which is exactly what one demo_traffic run did: 29 inferences at
    # 0.52 Hz, hit rate 1.000, target actually tracked on 13.6% of ticks. Say so
    # rather than let the headline number flatter a stalled run.
    det_hz = metrics.get("det_hz")
    if det_hz is None:
        print("[report] *** no mission tick was flown (the flight aborted before "
              f"its loop{': ' + aborted if aborted else ''}); every figure above, "
              "and kpi.json's outcome, describe nothing ***")
    elif det_hz < 2.0:
        print(f"[report] *** the detector only managed {det_hz:.2f} Hz "
              f"({metrics.get('det_mission_inferences')} mission inferences). "
              f"hit rate {metrics['det_hit_rate']} is measured over those alone "
              f"and is NOT a tracking result - the target was held on "
              f"{metrics['frac_ticks_seen'] * 100:.0f}% of ticks. Treat this "
              f"flight as unrepresentative ***")
    if aborted:
        print(f"[report] *** the flight ABORTED: {aborted} ***")
    return 0 if (n_ticks > 0 and not aborted) else 1


async def _fly_to_landing_site(drone, shield, args, timeout_s: float = 45.0,
                               pind=None, recorder=None, t0=None, query="",
                               live=None) -> dict:
    """Fly at cruise altitude, through the Shield, to a cell demo/landing.py
    calls landable, before the vertical descent.

    The descent used to start wherever the mission ended: in a hedge on
    citylife_redcar_trail2, on a parked car on citylife_ped_final. Returns what
    was chosen and whether it was reached, for metrics.json. `live` is the
    flight's Dashboard heartbeat (beat() only, memory only): up to 45 s of
    this flight used to pass without one (review of 8035d0a, P1)."""
    live = live if live is not None else _NoBeat()
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
        live.beat(detail=f"flying to the landing site, {d:.0f} m")         # memory only
        v = min(3.0, 0.6 * d)
        vz = float(np.clip((args.cruise_alt - up) * args.alt_gain,
                           -args.climb_max, args.climb_max))
        # In the BODY frame at the Shield's heading (0 here), as the mission
        # loop hands it (frames.filter_as_body, 2026-11-18).
        dec = filter_as_body(shield, State(x=p["x"], y=p["y"], up=up),
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

    # The signed bundle (refused unless its signature verifies, or
    # --allow-unverified-bundle records that it did not): --bundle, or the
    # bundle of --policy / DEFAULT_POLICY that tools/build_flight_bundles.py
    # built; a YAML only under --dev-yaml, recorded so (city_policy_choice,
    # ARCH-29: until 2026-10-21 this rail flew --policy as unsigned YAML).
    # Loaded BEFORE the output folder is cleared: a refused bundle (or --bundle
    # with --policy) used to delete the previous run's flight log first and
    # leave its manifest, kpi.json and metrics.json describing a log that was
    # gone (2026-10-06 review).
    try:
        _bundle, _yaml, _choice = city_policy_choice(args)
    except ValueError as exc:
        raise SystemExit(f"[policy] {exc}") from None
    policy, policy_source = load_for_flight(
        _bundle, _yaml, allow_unverified=args.allow_unverified_bundle)
    policy_source.update(_choice)

    out = ROOT / "demo" / "out" / args.tag
    out.mkdir(parents=True, exist_ok=True)
    for f in ("flight_log.jsonl", "detections.jsonl"):
        if (out / f).exists():
            (out / f).unlink()
    for pattern in CITY_EPISODE_FILES:
        for f in out.glob(pattern):
            f.unlink()
    episode = None              # the episode record, from the mission clock's start

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
    # --level-car Ped_04: ONE figure the level walks is the subject and its
    # truth (1 Nov, AIOT-VLA's person target); a tag mixing kinds is refused
    level_kind = None
    if args.level_car:
        import level_actors
        try:
            level_kind = level_actors.kind_of_tags([args.level_car], args.level_ped_prefix)
        except ValueError as exc:
            raise SystemExit(f"--level-car {args.level_car}: {exc}") from None
    _phrases = [args.object] + [ph for _, ph in retargets]
    _ped_phrases = [ph for ph in _phrases if subject_width(ph)[1] == "pedestrian"]
    if (_ped_phrases and args.pedestrians <= 0 and args.level_peds <= 0
            and level_kind != "pedestrian"):
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
    # --teacher-truth draws the subject where it truly is: only a level car
    # (one tagged actor) says where that is.
    if args.teacher_truth and not args.level_car:
        raise SystemExit("--teacher-truth draws the subject's TRUE position, which only "
                         "a --level-car gives")
    # A car the LEVEL drives replaces the client's scripted one; with both, the
    # truth row would describe one car and the camera would be looking at two.
    if args.level_car:
        if not args.no_car or args.traffic > 0:
            raise SystemExit(
                "--level-car takes the subject from the level, so the client "
                "must not drive one of its own: pass --no-car and leave "
                "--traffic at 0.")
        _bad = level_car_refusal([args.object] + [ph for _, ph in retargets],
                                 kind=level_kind)
        if _bad is not None:
            _for, _as = ((("pedestrian",), "a person (\"a person\")")
                         if level_kind == "pedestrian" else
                         (LEVEL_CAR_CLASSES, "a vehicle (\"a red car\", \"a box truck\")"))
            raise SystemExit(
                f"--level-car {args.level_car} is the subject's ground truth, "
                f"but {_bad[0]!r} selects the class {_bad[1]!r}, and that truth "
                f"exists only for {', '.join(_for)}. Every tick "
                f"would log an empty truth and the follow would go unscored. "
                f"Name it as {_as}.")

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
    # A distance_envelope (WP1-07, 2026-11-03) is measured on the obstacle map
    # (buildings) or the street mask (roads); the Shield refuses one whose map
    # is missing, so a policy that needs a map this run lacks stops here.
    envelope = {c.object_class for c in policy.by_type(DistanceEnvelope)}
    if (policy.by_type(ObstacleClearance) or "buildings" in envelope) and cmap is not None:
        smap = {"occ": cmap["occ"], "res": cmap["res"],
                "ox": cmap["ox"], "oy": cmap["oy"]}
    rmap = None
    if "roads" in envelope:
        _rm = Path(args.citymap).parent / "street.npz"
        if _rm.is_file():
            from build_street_mask import load_street
            _st = load_street(_rm)
            rmap = {"occ": _st["street"], "res": _st["res"], "ox": _st["ox"], "oy": _st["oy"]}
    # The grant's 5 s / 0.1 s with the escalation FSM unless --legacy-forecast
    # or --grant-forecast; see flight_shield. Recorded either way
    # (metrics/kpi "shield_lookahead").
    # The people of a distance_envelope over people (since 2026-11-18): the
    # simulator's pedestrian list, fed every tick (SimPeopleFeed); refused
    # without one. None for every other policy: nothing below changes.
    people_source = sim_people_source(policy, args)
    people_mem = (people_feed.PeopleMemory(stale_s=args.people_stale_s)
                  if people_source is not None else None)
    sim_people = None          # SimPeopleFeed, at the first tick (people exists then)
    if people_source is not None:
        print(f"[people] {people_source}; a person unseen for "
              f"{people_mem.stale_s:g} s is dropped")
    shield = flight_shield(policy, smap, grant_forecast=args.grant_forecast,
                           legacy_forecast=args.legacy_forecast, road_map=rmap,
                           people_source=people_source)
    print(f"[shield] forecast {shield.lookahead_s:g} s at {shield.dt:g} s "
          f"({'the grant forecast, escalation FSM off' if args.grant_forecast else '3 s / 0.5 s, the city rail before 2026-11-03' if args.legacy_forecast else 'the grant forecast, escalation FSM on'})")
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
                       street_mask=street, shield=shield,
                       guard_field=getattr(args, "guard_field", "grid"))
    print(f"[fence] building clearance measured on the "
          f"{'Shield field (opt-in, not yet flown)' if fence.guard_field == 'shield' else 'guard grid scan'}")

    audit = AuditLogger(out / AUDIT_FILE, policy)   # the POLICY, so a hot-applied rule restamps the hash

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
                               fence_near_m=args.fence_brake, shield=shield)
    if fence.polys:
        print(f"[fence] {len(fence.polys)} no-fly zone(s) known to the controller: "
              f"brake from {args.fence_brake:.0f} m, hold at {args.fence_standoff:.0f} m")

    print(f"[policy]  {policy.policy_id} {policy.policy_hash}")
    print(f"[policy]  from {policy_source['kind']} {policy_source['path']} - "
          f"signature {policy_source['signature']}"
          + (f" ({DEV_YAML_FLAG}: a development run)" if policy_source.get("dev_yaml")
             else f" (the signed bundle of {policy_source['bundle_of']})"
             if policy_source.get("bundle_of") else ""))
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
    det_g_at_t0 = {"n_seen": 0, "n_miss": 0}    # ...split, for det_hit_rate
    aborted = None              # why the flight aborted, if it did
    cmd_times = []              # when each mission command was sent
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
    limiter = guard_smooth = None
    n_los_ff = 0                # ticks the yaw servo carried a LOS-rate feed-forward
    n_meas_src = {}             # estimator measurements by pose source (capture/tick)
    trail = None
    trail_stop_short = 0.0
    recorder = None
    view_dir = None
    vla_rec = None              # --record-vla's writer (demo/vla_record.py)
    vla_record_stats = None     # ...its counts, for metrics.json
    perturber = None            # --record-perturb's schedule (vla_record.Perturber), or
                                # --record-perturb-vla's (vla_dagger.VLAPerturber)
    decider = None              # --decide-every-s's hold of the executed command
                                # (decide_rate.DecideRate); never with a perturber
    hierarchy = None            # --vla-hierarchy's VLA intent layer
                                # (vla_hierarchy.VLAHierarchy); never with either
    hierarchy_stats = None      # ...its summary at stop, for metrics.json
    start_heading_err_deg = None
    nfz_hold_ticks = 0
    n_fence_aim = 0          # ticks the forward aim was moved out of a zone's band
    guard_hold_ticks = 0        # any hold: fence OR building

    presence = PresenceMonitor(args.object, args.colour_min)
    rng_f = None            # low-passed range, for the orbit radial term
    orbit_fwd_prev = 0.0
    if args.record_perturb_vla:
        # DAgger (demo/vla_dagger.py): AerialVLA flies windows of this
        # recording. Its server must answer hello HERE, before the simulator
        # is touched: a recording flight with no VLA to fly its windows is
        # refused, not flown as a plain recording. Nothing runs yet - the
        # recorder starts it with the mission clock and stops it first.
        import vla_dagger
        try:
            perturber = vla_dagger.make_vla_perturber(
                args.record_perturb_vla, seed=args.seed, obs=obs,
                box_source=lambda: grounder.latest() if grounder is not None else None,
                policy=policy, object_text=lambda: args.object, cruise_alt=args.cruise_alt,
                record_dir=args.record_vla, every_s=args.perturb_vla_every,
                dur_s=args.perturb_vla_dur, allow_stand_in=args.perturb_vla_allow_stand_in,
                bottom_tile=args.perturb_vla_bottom_tile,
                prev_dt_s=args.perturb_vla_prev_dt_s,
                step_as_rate=args.perturb_vla_step_as_rate,
                speed_words=args.perturb_vla_speed_words,
                rate_yaw=args.perturb_vla_rate_yaw,
                front_source=args.record_vla_front_source, log=print)
        except vla_dagger.DaggerRefused as exc:
            raise SystemExit(f"[vla-dagger] *** refused: {exc}") from None
        print(f"[vla-dagger] {perturber.describe()}")
    if args.decide_every_s is not None:
        # The CEILING flight (demo/decide_rate.py): the expert may act only at
        # a VLA's rate - every P s its own command from D s before, held in
        # between. It runs on the mission clock, so nothing here needs the sim.
        from decide_rate import DecideRate
        decider = DecideRate(args.decide_every_s, args.decide_delay_s or 0.0)
        print(f"[decide] {decider.describe()}")
    if args.vla_hierarchy:
        # HIERARCHICAL CONTROL (demo/vla_hierarchy.py): AerialVLA decides the
        # intent at its own rate, this controller executes it. Its server must
        # answer hello HERE, as AerialVLA with the adapter's tile and speed
        # words, before the simulator is touched. Nothing runs yet - its query
        # thread starts with the mission clock.
        import vla_hierarchy
        try:
            hierarchy = vla_hierarchy.make_vla_hierarchy(
                args.vla_hierarchy, obs=obs,
                box_source=lambda: grounder.latest() if grounder is not None else None,
                policy=policy, object_text=lambda: args.object, cruise_alt=args.cruise_alt,
                record_dir=out / vla_hierarchy.STEPS_DIR, s_range=args.vla_hierarchy_range,
                allow_stand_in=args.vla_hierarchy_allow_stand_in,
                bottom_tile=args.vla_hierarchy_bottom_tile,
                prev_dt_s=args.vla_hierarchy_prev_dt_s,
                step_as_rate=args.vla_hierarchy_step_as_rate,
                speed_words=args.vla_hierarchy_speed_words,
                front_source=args.vla_hierarchy_front_source,
                expiry_s=args.vla_hierarchy_expiry_s, log=print)
        except vla_hierarchy.HierarchyRefused as exc:
            raise SystemExit(f"[vla-hierarchy] *** refused: {exc}") from None
        print(f"[vla-hierarchy] {hierarchy.describe()}")
    # --sim-address: the simulator on another machine (the Jetson Orin
    # running this controller against the desktop's Project AirSim).
    client = ProjectAirSimClient(address=args.sim_address)
    # The Dashboard heartbeat starts here, at the first slow step (the
    # connect), and ends at the very end of this function with the exit code
    # of final_report. A run refused earlier (the SystemExits above) leaves
    # no row; a failed connect ends its row "failed" at once rather than
    # leaving one that reads "stopped unexpectedly". Between the mission
    # loop's beats, the phase boundaries below beat too (memory only): the
    # first version beat only inside the loop, and the ~100 s of connect,
    # scene, detector load, take-off and landing of a 225 s run read "no
    # heartbeat" (review of 8035d0a, P1).
    live = _live_heartbeat(args.tag, log_dir=out, total_s=args.max_s, phase="starting",
                           detail="connecting to the simulator")
    try:
        client.connect()
    except BaseException:
        live.stop("failed", detail="the simulator connection failed")
        raise
    grounder = None
    try:
        world = World(client, flight_scene(args), delay_after_load_sec=2,
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
        live.beat(detail="scene loaded; placing the actors")               # memory only

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
        # --vla-hierarchy-front-source detector: this camera's frame is also
        # the hierarchy's front frame, from this one subscription (a pointer
        # each, nothing decoded in the callback).
        det_front = (args.vla_hierarchy is not None
                     and args.vla_hierarchy_front_source == "detector")
        # --record-vla-front-source detector: the recorder's front frame is
        # this camera's too (filled below, once the recorder exists).
        rec_front: list = []

        def _front_frame(_, m):
            obs.put_front(m)
            if det_front and hierarchy is not None:
                hierarchy.put_front(m)
            if rec_front:
                rec_front[0].put_front(m)
        client.subscribe(drone.sensors["FrontCamera"]["scene_camera"], _front_frame)
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

        if args.record_vla:
            # What AerialVLA would see (demo/vla_record.py): its two cameras,
            # in this scene variant only. The callbacks store the message;
            # pairing, encoding and writing are the recorder's thread. The
            # box source reads the Grounder (built further down) when each
            # capture is built, as a VLA flight's DetectorHint reads it.
            from vla_record import (PERTURB_SPEED_SCALE, ExpertVLARecorder, Perturber,
                                    box_from_grounder)
            if args.record_perturb:
                # Its own RNG from --seed: the flight's global stream is not
                # touched, so nothing else in it moves.
                perturber = Perturber(args.seed, yaw_rate=args.perturb_yaw_rate,
                                      speed_scale=(PERTURB_SPEED_SCALE
                                                   if args.perturb_speed else None),
                                      replace_yaw=args.perturb_replace_yaw)
                lo, hi = args.perturb_yaw_rate
                print(f"[vla-record] DART perturbation ON (seed {args.seed}): every "
                      f"5-8 s, 1.0-1.5 s of +-{lo:g}-{hi:g} rad/s yaw rate"
                      + (" INSTEAD of the expert's" if args.perturb_replace_yaw else "")
                      + (" and 0.5-1.0 x speed" if args.perturb_speed else "")
                      + ", after the controller, before the Shield")
            vla_rec = ExpertVLARecorder(args.record_vla, obs,
                                        lambda: box_from_grounder(
                                            grounder.latest() if grounder is not None else None),
                                        tag=args.tag, object_text=args.object,
                                        perturber=perturber,
                                        front_source=args.record_vla_front_source)
            if args.record_vla_front_source == "detector":
                rec_front.append(vla_rec)
            else:
                client.subscribe(drone.sensors["VLAFront"]["scene_camera"],
                                 lambda _, m: vla_rec.put_front(m))
            client.subscribe(drone.sensors["VLADown"]["scene_camera"],
                             lambda _, m: vla_rec.put_down(m))
            front = ("FrontCamera" if args.record_vla_front_source == "detector"
                     else "VLAFront")
            print(f"[vla-record] front {front}, down VLADown ({flight_scene(args)}) -> "
                  f"{args.record_vla}")
        if hierarchy is not None:
            # --vla-hierarchy: the VLA asks with its own two cameras (this scene
            # variant, flight_scene). The callbacks store the message; pairing
            # and encoding happen on its query thread when it builds a request.
            # With --vla-hierarchy-front-source detector the front frame is
            # FrontCamera's (fed above), VLAFront is not subscribed.
            if args.vla_hierarchy_front_source != "detector":
                client.subscribe(drone.sensors["VLAFront"]["scene_camera"],
                                 lambda _, m: hierarchy.put_front(m))
            client.subscribe(drone.sensors["VLADown"]["scene_camera"],
                             lambda _, m: hierarchy.put_down(m))
            front = ("FrontCamera" if args.vla_hierarchy_front_source == "detector"
                     else "VLAFront")
            print(f"[vla-hierarchy] front {front}, down VLADown ({flight_scene(args)}), "
                  f"nothing recorded")

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
        # A figure (--level-car Ped_04, level_kind "pedestrian") the same way,
        # as one person rather than the crowd.
        if args.level_car:
            import level_actors
            car = level_actors.LevelCar(world, args.level_car, desc=args.object,
                                        min_period_s=args.level_truth_period,
                                        kind=level_kind)
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

        live.beat(detail="arming and taking off")                           # memory only
        drone.enable_api_control()
        drone.arm()
        await (await drone.takeoff_async())
        for _ in range(400):
            kin = drone.get_ground_truth_kinematics()
            if -kin["pose"]["position"]["z"] >= args.cruise_alt - 0.5:
                break
            live.beat(detail=f"climbing to {args.cruise_alt:.0f} m")         # memory only
            await drone.move_by_velocity_async(0.0, 0.0, -2.0, duration=0.2)
            await asyncio.sleep(0.1)
        live.beat(phase="airborne", detail="at cruise altitude; taking up the start heading")

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
            # WHICH RULES WILL RUN. classify_split judges a phrase by its
            # noun; a phrase whose noun it does not know gets NO physical rule
            # and every candidate is OK - with an empty by_rule that reads as
            # a clean pass. Said here, loudly, and recorded per phrase.
            _nouns = {ph: identity_mod.noun_of(ph)
                      for ph in [args.object] + [p_ for _, p_ in retargets]}
            for _ph, _n in _nouns.items():
                if _n is None:
                    print(f"*** [identity] no physical rules for {_ph!r}: every "
                          f"candidate will be OK. Name the subject with a noun "
                          f"identity.noun_of knows (car, van, truck, bus, "
                          f"person and their synonyms) ***")
            id_cfg = {"thresholds": _th, "street": _sd, "hfov": CAMERA_HFOV_DEG,
                      "hash": identity_mod.thresholds_hash(_th),
                      "path": str(args.identity_thresholds or
                                  identity_mod.DEFAULT_THRESHOLDS),
                      "nouns": _nouns}
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
        teacher = None
        if args.teacher_truth:
            teacher = teacher_truth_fn(car)
            print(f"[teacher] --teacher-truth: the detector's boxes are replaced by "
                  f"{args.level_car}'s TRUE position ({args.object_width_m:.1f} m wide, "
                  f"{subject_height_m(args.object):.1f} m tall). A TEACHER for training "
                  f"labels: this flight is not a follow result.")
        grounder = Grounder(obs, args.object, thresh=args.det_thresh,
                            log_path=out / "detections.jsonl",
                            colour_min=args.colour_min, lock=lock,
                            colour_mask=args.colour_mask,
                            legacy_sat=args.colour_legacy_sat,
                            identity=id_cfg, teacher=teacher,
                            teacher_dims=(args.object_width_m,
                                          subject_height_m(args.object)))
        grounder.start()
        live.beat(detail="loading the detector")                            # memory only
        for _ in range(600):                      # wait for the detector to load
            if grounder.latest()["seq"] > 0:
                break
            live.beat()                                                     # memory only
            await asyncio.sleep(0.5)
        print(f"[flight] cruise {args.cruise_alt:.0f} m — following {args.object!r}")

        # The slew limiter, and the guard smoother (None with --legacy-motion,
        # which flies aerialvla_demo.RateLimiter exactly as before). See the
        # MOTION SHAPING note above VecSlewLimiter.
        limiter, guard_smooth = make_motion(args)
        print(f"[motion] {motion_params(args)}")

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
                live.beat(detail="waiting for the detector to see the subject")  # memory only
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
        # The episode record and its CSP, from where the mission starts. A
        # record that cannot be written must not cost the flight: it is said,
        # and the flight log then carries no episode id.
        try:
            _p0 = drone.get_ground_truth_kinematics()["pose"]["position"]
            episode = city_episode(out, policy, shield.lookahead_s, (_p0["x"], _p0["y"]),
                                   args.object, args.cruise_alt, args.speed_max)
            _g0 = episode.generations[0]
            print(f"[episode] {episode.episode_id}: policy_g0.json, "
                  f"{'csp_g0.json' if _g0['csp'] else '*** no CSP (see events.jsonl)'}")
        except Exception as exc:                                    # noqa: BLE001
            episode = None
            print(f"[episode] *** not recorded: {type(exc).__name__}: {exc}")
        if vla_rec is not None:
            vla_rec.start(t0)           # only the mission's frames and ticks
        if hierarchy is not None:
            hierarchy.start(t0)         # asked continuously from the mission's first tick
        # Detector inferences made BEFORE the mission clock started (the start
        # gate can wait minutes). det_hz must not count them: it did, dividing
        # them by mission time only, and reported 6.9-7.5 Hz for flights whose
        # detector ran at 3.5-3.9 Hz during the mission (found 2026-09-29).
        _g0 = grounder.latest()
        det_n_at_t0 = _g0["n_seen"] + _g0["n_miss"]
        # ...and det_hit_rate must not count them either (det_rates).
        det_g_at_t0 = {"n_seen": _g0["n_seen"], "n_miss": _g0["n_miss"]}
        tick_deadline = None        # the pacing's running deadline, see below
        # Where the subject was last MEASURED to be, and how fast it was then
        # measured to move (displacement of accepted positions over 2 s).
        acc_hist = []
        last_acc = None
        last_meas_speed = None
        last_bearing, brg_rate, last_cmd, mode = 0.0, 0.0, (0.0, 0.0), "hold"
        last_emit_v = (0.0, 0.0)    # the last horizontal velocity flown
        v_air = None
        # The presence verdict is computed further down the tick, so the search
        # logic uses the PREVIOUS tick's. That is the honest signal anyway: it is
        # what the system last knew about whether the object is really out there.
        # (start_heading_err_deg was reset to None here, AFTER the rotate had
        # measured it, so every metrics.json carried null - found in review,
        # 2026-09-24. The function-scope initialiser covers an early abort.)
        last_verdict = "UNSURE"
        # Servo on an ESTIMATE of the target rather than the latest box. See
        # demo/target_state.py for the measurements that forced this.
        estimator = (subject_estimator(subject_class) if args.target_estimator else None)
        # WHERE THE SUBJECT DROVE (--trail-follow; demo/trail.py).
        #
        # Built from the estimator's own position after each accepted update,
        # so it carries no ground truth. Needs the estimator: the box servo has
        # no position to lay a breadcrumb at.
        trail = (subject_trail() if args.trail_follow and estimator is not None else None)
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
            seeded, P0 = seed_estimate(estimator, pf, cp, t_c)
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
        live.beat(detail=f"mission: following {args.object!r}")             # memory only
        while time.time() - t0 < args.max_s:
            tick += 1
            _tk0 = time.time()
            live.beat(progress=(_tk0 - t0) / max(float(args.max_s), 1.0))   # memory only
            mode_label = None       # HUD state text when not the default
            hud_box = False         # draw the box on the video (hud_target)?
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
            # The aircraft's own horizontal velocity, for the yaw servo's
            # line-of-sight feed-forward; the last flown command stands in
            # when the kinematics carry no twist.
            _tw = (kin.get("twist") or {}).get("linear") or {}
            v_air = ((float(_tw["x"]), float(_tw["y"]))
                     if _tw.get("x") is not None and _tw.get("y") is not None else None)
            obs.put_pose(p["x"], p["y"], yaw)
            # The full pose, into the ring get_front_capture reads: stamped
            # mid-RPC, and with the sim's own time_stamp when it has one.
            _qk = kin["pose"]["orientation"]
            _t_kin = _ta + _ms_kin / 2000.0
            obs.put_pose_full(_t_kin, p["x"], p["y"], -p["z"], yaw,
                              quat_pitch(_qk), quat_roll(_qk),
                              stamp=kin.get("time_stamp"))
            _ta = time.time()
            # Client-spawned figures: only the pacing few cost anything, the
            # standing majority is skipped without an RPC, and they walk only
            # when updated, so they always are. A LEVEL crowd (--level-peds) is
            # truth only and one poll is ~100 ms for 40 names, so it is polled
            # only while the subject is a pedestrian (update_level_truth).
            # Scenery is never allowed to fail a flight, so update() swallows
            # its own errors.
            update_level_truth(people, subject_class, time.time() - t0,
                               feed_people=people_mem is not None)
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
                    anchor = lapse_anchor(estimator, last_acc, t_meas_last_cap, t_meas_last,
                                          last_meas_speed, v_prior)
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
                    # the subject's centre height, for the projection into the frame
                    grounder.set_prior(estimator_prior(
                        estimator, SUBJECT_CENTRE_UP_M.get(subject_class, 0.75)))
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
                # AT CAPTURE, IN EVERY MODE: the frame's own pose and time,
                # and the range in its own depth frame (identity: the range
                # and bearing it measured from them - pinhole, pitch and roll
                # included, horizontal range). Without --identity this used
                # the tick's pose and clock and the NEWEST depth, ~0.5 s off;
                # see estimator_measurement. --legacy-tick-pose restores it.
                t_meas, x_m, y_m, yaw_m, b_meas, r_meas, _src = estimator_measurement(
                    det, g, (time.time(), state.x, state.y, yaw), id_cfg is not None,
                    newest_range=(range_from_depth(obs.get_depth(), det)
                                  if have_depth and (args.legacy_tick_pose
                                                     or not (g.get("cap") or {}).get("pose"))
                                  else None),
                    legacy_tick_pose=args.legacy_tick_pose)
                n_meas_src[_src] = n_meas_src.get(_src, 0) + 1
                feed = True
                if id_cfg is not None:
                    if g.get("tier") != "ok":
                        # SOFT: may CONTINUE a live track only.
                        feed = soft_may_continue(t_meas, estimator.t_last_update,
                                                 time.time(), t_ok_last)
                        n_soft_fed += int(feed)
                        n_soft_refused += int(not feed)
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

            # --vla-hierarchy (vla_hierarchy.VLAHierarchy.plan): this tick's
            # intent, from the answers its query thread collected. Locked is
            # the controller's own lock - the condition of its `track` branch
            # just below. The stand-off it returns is this tick's want_range,
            # which every arm reads: the VLA's desired range in FOLLOW, the one
            # in force on HOLD and SEARCH, _stand_off()'s on a fallback (each
            # reached at 2 m/s, never a jump). Without the flag want_range is
            # _stand_off()'s, as always.
            if hierarchy is not None:
                want_range = hierarchy.plan(
                    time.time() - t0,
                    locked=(est_obs is not None
                            or (det is not None and age < args.det_max_age and reacq is None)),
                    pose=(state.x, state.y, state.up, yaw), default_m=_stand_off())

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
                # TURN ANTICIPATION: the line of sight's own rate, from the
                # estimate's velocity and the aircraft's, fed forward - so the
                # servo stops settling rate/gain behind a turning car (the
                # ~14-18 deg lag of 13 Oct, see los_rate). Only on a fresh
                # estimate (updated < 1 s ago): a coasted constant-velocity
                # guess must not steer the nose. Scaled, capped, and then
                # slew-limited with the rest of the command. Its range is
                # floored at --yaw-los-ff-min-range (10 m): only a car at
                # 15-25 m has validated it, and the estimate's velocity noise
                # grows as 1/r: at a pedestrian's 2 m, 5x what the floor lets
                # through (see los_rate).
                ff_yaw = 0.0
                if (args.yaw_los_ff > 0.0 and not args.legacy_motion
                        and estimator.x is not None
                        and estimator.t_last_update is not None
                        and time.time() - estimator.t_last_update < 1.0):
                    _va = v_air if v_air is not None else last_emit_v
                    ff_yaw = float(np.clip(
                        args.yaw_los_ff * los_rate(
                            float(estimator.x[0]), float(estimator.x[1]),
                            float(estimator.x[2]), float(estimator.x[3]),
                            state.x, state.y, _va[0], _va[1],
                            r_min=args.yaw_los_ff_min_range),
                        -args.yaw_los_ff_cap, args.yaw_los_ff_cap))
                    n_los_ff += int(ff_yaw != 0.0)
                yaw_rate = yaw_command(args.yaw_gain, bearing, ff=ff_yaw)
                # Proportional term plus the target's own opening rate fed
                # FORWARD. Without the feedforward the loop settles at a lag
                # error - a 2 m/s car held 23.4 m against a 15.8 m stand-off,
                # exactly the (r - want)*gain = 2.0 fixed point. An integrator
                # would also fix it and would wind up every time the Shield
                # overrides the actuator; the estimator already knows the
                # target's velocity, so no memory is needed.
                # The catch-up half saturates at --catchup-mps above the
                # subject's own rate (standoff_speed), so a car pulling away
                # is not chased at speed_max; --legacy-motion: linear.
                ff = (estimator.range_rate(state.x, state.y)
                      if args.range_feedforward else 0.0)
                fwd = standoff_speed(rng_est, want_range, args.range_gain, ff,
                                     args.speed_max,
                                     0.0 if args.legacy_motion else args.catchup_mps)
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
                # TARGET LOCKED only on a fresh box (and, with identity, an
                # OK one); otherwise say that the aircraft is flying on the
                # estimate - in BOTH modes now (hud_target).
                hud_box, _lbl = hud_target(id_cfg is not None, True,
                                           det is not None, age, det_tier)
                if _lbl is not None:
                    mode_label = _lbl
            elif det is not None and age < args.det_max_age and reacq is None:
                # The box servo holds a box FRACTION. Converted from the same
                # metric stand-off the estimator arm holds (servo_want_width),
                # so the two A/B arms fly one set-point; --legacy-servo-width
                # holds the raw --want-width, as before.
                _ww = (args.want_width if args.legacy_servo_width
                       else servo_want_width(want_range, args.object_width_m))
                yaw_rate, fwd, bearing = servo(
                    det, det[5], args.yaw_gain, _ww, args.speed_max)
                # The box IS what this branch steers on (up to det_max_age),
                # so it stays on screen; identity keeps its 0.6 s rule.
                hud_box = id_cfg is None or age < 0.6
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
                        # First swing toward the side the target was lost
                        # on: the sign of the bearing the coast ended on, so
                        # the yaw rate keeps its sign across the hand-over.
                        _side = (1.0 if last_bearing + brg_rate * args.coast_s >= 0.0
                                 else -1.0)
                        yaw_rate = search_sweep_rate(lost - args.coast_s,
                                                     args.search_sweep_deg,
                                                     args.search_period_s,
                                                     side=_side)
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
            lat_x = lat_y = 0.0     # the sidestep slide() adds, for the smoother
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
                    lat_x, lat_y = sx * lat, sy * lat
                    gvx += lat_x
                    gvy += lat_y
                    fmode = "skirt"
                else:
                    fmode = "hold" if fblocked else "near"
            else:
                fmode = "clear" if fdist is None or fdist > 20 else "near"
            # RAMP THE GUARD (GuardSmoother): gate() and slide() answer each
            # tick afresh and stepped the command 2-4 m/s in one tick; the
            # smoother lets a brake bite at once only when it must
            # (guard_urgent) and ramps everything else. A pass-through while
            # the guard is clear. `guard_in` logs what the guard asked for.
            # The same urgency tells the slew limiter this is a hold, so its
            # legacy braking floor applies even where the hold's sidestep
            # points the step sideways (VecSlewLimiter, `hold`).
            guard_in = None
            g_urgent = guard_urgent(fence, fblocked)
            if guard_smooth is not None:
                guard_in = (gvx, gvy)
                gvx, gvy = guard_smooth(time.time(), cvx, cvy, fscale, lat_x, lat_y,
                                        urgent=g_urgent)
            raw = Action4D(vx=gvx, vy=gvy, vz_up=vz_up, yaw_rate=yaw_rate)
            smooth = (limiter(raw, time.time(), hold=g_urgent)
                      if isinstance(limiter, VecSlewLimiter) else limiter(raw))

            # Tell the Shield where the subject is, so SubjectStandoff rules can
            # bind. The position comes from the ESTIMATOR's state vector, which
            # is built from bearing, range and the aircraft's own pose - so this
            # carries no target ground truth and the no-leak property holds.
            #
            # Cleared to None the moment the estimator has nothing, and that is
            # required rather than tidy: a stale position would have the Shield
            # enforcing a stand-off from where the subject used to be, which is
            # both wrong and invisible in the logs.
            #
            # Position only, because that is all Shield.set_subject takes here,
            # so the Shield forecasts its stand-off against a subject standing
            # still - which is why this rail cannot yet fly the grant's 5 s
            # forecast (flight_shield: StandoffHold on 44 % of replayed ticks).
            # The estimator's state is [x, y, vx, vy], so the subject's own
            # velocity is float(estimator.x[2]), float(estimator.x[3]), from
            # the same bearing-and-range estimate as the position (no ground
            # truth). Since 2026-11-03 it is passed (set_subject's velocity=),
            # so the grant's 5 s forecast carries the subject along instead of
            # holding it still; a non-finite estimate gives no velocity.
            if estimator is not None and estimator.x is not None and est_obs is not None:
                svx, svy = float(estimator.x[2]), float(estimator.x[3])
                shield.set_subject(float(estimator.x[0]), float(estimator.x[1]),
                                   subject_class,
                                   velocity=((svx, svy) if math.isfinite(svx)
                                             and math.isfinite(svy) else None))
            else:
                shield.set_subject(None)
            # Every pedestrian the simulator lists, for a distance_envelope
            # over people (people_mem: only with such a rule).
            people_now = None
            if people_mem is not None:
                if sim_people is None:
                    sim_people = SimPeopleFeed(people, people_mem)
                people_now = sim_people.step(time.time() - t0)
                shield.set_people(people_now)

            # --record-perturb (vla_record.Perturber): the EXECUTED command is
            # knocked off for 1.0-1.5 s every 5-8 s, after the controller and
            # its slew limiter (whose own state stays the expert's) and before
            # the Shield, which judges and repairs what is actually sent.
            # --record-perturb-vla (vla_dagger.VLAPerturber), the same place:
            # in its windows AerialVLA's step tracker flies, on this pose.
            # Without either the Shield judges `smooth`, as it always has.
            flown_in = smooth
            perturb_rec = None
            if perturber is not None:
                flown_in, perturb_rec = perturber.apply(smooth, time.time() - t0,
                                                        pose=(state.x, state.y, state.up, yaw))
            # --decide-every-s (decide_rate.DecideRate), the same place and never
            # with a perturber: what flies is a zero-order hold of the expert's
            # own `smooth`, a decision every P s carrying its command from D s
            # before. The controller and its limiter run on, on their own state.
            decide_rec = None
            if decider is not None:
                flown_in, decide_rec = decider.apply(smooth, time.time() - t0)
            # --vla-hierarchy (vla_hierarchy.VLAHierarchy.apply), the same place
            # and never with a perturber or the hold: on a SEARCH tick (and a
            # HOLD without the lock) the VLA's step tracker flies, on this pose;
            # on every other tick the controller's own `smooth`, its stand-off
            # already the VLA's in FOLLOW. The Shield judges whatever flies.
            hier_rec = None
            if hierarchy is not None:
                flown_in, hier_rec = hierarchy.apply(smooth, time.time() - t0,
                                                     pose=(state.x, state.y, state.up, yaw))
                if hier_rec.get("vla_flown"):
                    # the video says who flew this tick
                    mode_label = ("VLA SEARCHING" if hier_rec.get("authority") == "vla_search"
                                  else "VLA HOLDING")
            _ta = time.time()
            # Handed to the Shield in the BODY frame (2026-11-18, WP3-04 /
            # ARCH-04: frames.filter_as_body, the arc off), at the heading the
            # Shield turns it back with: `state`'s, 0 - this rail gives the
            # Shield no heading - so both turns are the identity: the body
            # action is the world action relabelled, not the aircraft's body
            # frame. A tick whose round trip would change a bit (a zero's
            # sign) goes to filter() as before; the decision is filter()'s
            # to the bit (tests/test_rail_body_frame.py).
            d = filter_as_body(shield, state, flown_in)
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
            last_emit_v = (e.vx, e.vy)
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
                **episode_fields(episode),
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
                # What the guard asked for before GuardSmoother ramped it
                # (None with --legacy-motion, where `raw` is that).
                "guard_in": (None if guard_in is None else
                             [round(guard_in[0], 3), round(guard_in[1], 3)]),
                "emitted": e.model_dump(),
                "touched": d.touched, "braked": d.braked,
                "violations": [v.model_dump() for v in d.violations],
                "repairs": [r.model_dump() for r in d.repairs],
                # The check on what was FLOWN, not on what was asked for.
                # `violations` above is the check on `raw`, so without this the
                # P0 escape KPI (limit 0) - a P0 seen and then flown anyway - could
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
                # The people the Shield was given (a people envelope only).
                **({} if people_now is None else
                   {"people": people_feed.rows_people(people_now)}),
            })
            stage_ms["kin"].append(_ms_kin)
            stage_ms["truth"].append(_ms_truth)
            stage_ms["shield"].append(_ms_shield)
            if perturb_rec is not None:
                # A perturbed flight's row says what was added: `smooth` is
                # still the expert's command, `emitted` what flew.
                rows[-1]["perturb"] = perturb_rec
            if decide_rec is not None:
                # A held flight's row says which decision flew: its index,
                # whether this tick held it, its age and the age of the
                # expert's command it carries (`smooth` is still the expert's
                # command of this tick, `emitted` what flew).
                rows[-1]["decide"] = decide_rec
            if hier_rec is not None:
                # A hierarchical flight's row says who flew and why: the
                # authority, the VLA's intent and answer age, the stand-off
                # (`smooth` is still the controller's command, `emitted` what flew).
                rows[-1]["hierarchy"] = hier_rec
            if vla_rec is not None:
                # One deque append: the pose of this tick's kinematics read,
                # the mode, what the controller asked and what flew. The
                # car's truth goes in for analysis only (vla_record.py).
                vla_rec.note_tick(_t_kin, state.x, state.y, state.up, yaw, mode=mode,
                                  cmd=e, asked=smooth, touched=d.touched,
                                  truth=(car.pos if car is not None else None),
                                  obj=args.object, tick=tick, perturb=perturb_rec)

            _ta = time.time()
            cmd_times.append(_ta)
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
                }, det if seen and hud_box else None)
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
        if episode is not None:
            episode.end("the mission window closed", t=round(t_mission_end - t0, 3))
        live.beat(progress=1.0, phase="landing", detail="mission over; landing")   # memory only
        grounder.stop()
        _hud_live = recorder is not None and pind is not None and pind_error is None
        if args.land_site:
            landing_info = await _fly_to_landing_site(
                drone, shield, args, pind=pind if _hud_live else None,
                recorder=recorder if _hud_live else None, t0=t0, query=args.object,
                live=live)
        for _k in range(600):
            kin = drone.get_ground_truth_kinematics()
            up = -kin["pose"]["position"]["z"]
            live.beat(detail=f"descending ({up:.0f} m)")                      # memory only
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
        # Recorded, not only printed: a flight that aborted must not leave a
        # metrics.json and kpi.json that read like a clean run (final_report).
        aborted = f"{type(exc).__name__}: {exc}"
        print(f"[warn] flight aborted: {aborted}")
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
        if vla_rec is not None:
            # Bounded (the writer and FrameSaver are joined with a timeout):
            # the recording never holds up the teardown.
            try:
                vla_record_stats = vla_rec.stop()
                print(f"[vla-record] {vla_record_stats['captures']} captures "
                      f"({vla_record_stats['skipped_front_frames']} front frames skipped, "
                      f"{vla_record_stats['saver'].get('dropped')} JPEGs dropped), "
                      f"{vla_record_stats['ticks']} ticks -> {args.record_vla}")
                if args.record_perturb_vla:
                    from vla_dagger import summary_line
                    print(summary_line(vla_record_stats.get("perturb")))
            except Exception as exc:                                # noqa: BLE001
                print(f"[warn] the VLA recording did not close: {type(exc).__name__}: {exc}")
        if perturber is not None and callable(getattr(perturber, "stop", None)):
            # --record-perturb-vla on a flight that failed before its recorder
            # started it: close the link (a no-op after vla_rec.stop()).
            try:
                perturber.stop()
            except Exception as exc:                                # noqa: BLE001
                print(f"[warn] the VLA perturber did not close: {type(exc).__name__}: {exc}")
        if hierarchy is not None:
            # --vla-hierarchy: bounded (its thread and link are joined with a
            # timeout), and on every way out, a flight that failed before its
            # mission clock included.
            try:
                hierarchy_stats = hierarchy.stop()
                print(hierarchy.summary_line(hierarchy_stats))
            except Exception as exc:                                # noqa: BLE001
                print(f"[warn] the VLA hierarchy did not close: {type(exc).__name__}: {exc}")
        # The Dashboard row normally ends after final_report, at the end of
        # this function, with its exit code. It ended here with
        # "failed" if aborted else "ok", and `aborted` is set only by the
        # `except Exception` above, so a Ctrl+C flight (KeyboardInterrupt,
        # or the CancelledError asyncio.run throws into this coroutine)
        # ended "ok", as did a run of 0 mission ticks (review of 8035d0a,
        # P8). Only an exception that is leaving this `finally` - nothing
        # after it will run - ends the row here (never raises).
        _leaving = sys.exc_info()[0]
        if _leaving is not None:
            live.stop("stopped" if issubclass(_leaving, (KeyboardInterrupt, asyncio.CancelledError))
                      else "failed", detail=f"ended by {_leaving.__name__}")
        else:
            live.beat(detail="tearing down; writing the run's records")     # memory only

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

    # Inside a zone's POLYGON (not its margin ring) and within its band, per
    # zone, read from the Shield's rule checker (Shield.rule_status) rather
    # than from a second walk over the policy's polygons. One entry per (point,
    # zone), as before, for every static zone (time windows are not applied,
    # as before). A MOVING dynamic_nfz is left out: rule_status reads it where
    # it is at the end of the flight, not where it was at each point, so the
    # count would be wrong. It is named instead of counted as zero
    # (`nfz_moving_not_counted`); the flight loaders refuse a policy file that
    # declares a MOVING dynamic_nfz (guardrail/models.py FLIGHT_HELD_TYPES; one
    # without motion flies since 2026-11-15 and is counted here like any
    # static zone), so no flight has a moving one yet. Every other rail runs
    # this same count through
    # guardrail.replay.zone_dwell since 2026-10-11.
    moving_zones = sorted(r.rule.id for r in shield.ir.fences if r.moving)
    inside = [pt for pt in traj
              for r in shield.rule_status(State(x=pt["x"], y=pt["y"], up=pt["up"]))
              if r["type"] in ("polygon_fence", "circle_fence", "dynamic_nfz")
              and r["id"] not in moving_zones and r["inside"] and r["in_band"]]
    if moving_zones:
        print(f"[nfz] moving zone(s) {moving_zones} are NOT in nfz_s: the end-of-"
              f"flight count has no per-tick position for them")
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
    # Over the MISSION's inferences only, like det_hz (det_rates).
    _dr = det_rates(g, det_g_at_t0, rows[-1]["t"] if rows else None)
    metrics = {
        "tag": args.tag, **episode_fields(episode),
        "ticks": len(traj), "object": args.object,
        "detector": DETECTOR_ID,
        # --teacher-truth: the boxes were the subject's TRUE position, so the
        # follow numbers below are a teacher's, never a follow result
        "teacher_truth": (dict(grounder.n_teacher, level_car=args.level_car)
                          if args.teacher_truth else None),
        # What flew: no VLA - a hand-written controller - and the detector
        # that steered it (the manifest's vla_model_hash says NO_VLA).
        "pilot": pilot_entry(),
        "det_seen": g["n_seen"], "det_missed": g["n_miss"],
        # LIVENESS, not accuracy. This counts the inferences that produced a
        # box - any box. A box on a parked lookalike, a building or road paint
        # scores exactly like a box on the target. It was read as a tracking
        # metric for a long time and it is not one; `frac_on_target` below is.
        # Mission inferences only; the old all-inference figure is kept beside
        # it as det_hit_rate_all_inferences_legacy.
        "det_hit_rate": _dr["det_hit_rate"],
        "det_hit_rate_all_inferences_legacy": _dr["det_hit_rate_all_inferences_legacy"],
        "det_mission_inferences": _dr["det_mission_inferences"],
        # Was the follow MEASURED? kpi.py reads a None frac_within_30m as no
        # failure; only follow_scored False makes mission_success None.
        "follow_scored": follow_scored(len(rows), len(seps)),
        "follow_truth_ticks": len(seps),
        # Why the flight ended early, if it did (None: it did not).
        "aborted": aborted,
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
        "start_gate": dict(start_gate, det_inferences_before_t0=det_g_at_t0),
        # Where each tick's time went. `work` is the whole tick before pacing.
        "stage_ms_median": {k: (round(float(np.median(v)), 1) if v else None)
                            for k, v in stage_ms.items()},
        # ...and its tail, with the spacing of the commands actually sent: a
        # median of 0.0 ms hid a ~100 ms truth poll every 5th-6th tick, and
        # the average-rate loop gate passed on commands paired 200 ms / 5 ms
        # apart (loop_timing).
        **loop_timing(stage_ms, cmd_times),
        # The forecast the Shield really used (flight_shield).
        "shield_lookahead": shield_lookahead_record(shield),
        # Where the Shield was told about people (a people envelope only).
        **({} if people_mem is None else
           {"people_feed": {"source": people_source, **people_mem.record(),
                            "people_nonfinite": shield.n_people_nonfinite}}),
        # Estimator measurements by the pose they were placed at: "capture"
        # (the box's own frame) or "tick" (--legacy-tick-pose, or no capture
        # pose to be had). See estimator_measurement.
        "estimator_meas_src": dict(n_meas_src),
        # What the motion shaping did (VecSlewLimiter, GuardSmoother, the
        # yaw servo's line-of-sight feed-forward); settings in params.motion.
        "motion_stats": {"limiter": (limiter.stats() if isinstance(limiter, VecSlewLimiter)
                                     else None),
                         "guard_smoother": (guard_smooth.stats() if guard_smooth is not None
                                            else None),
                         "los_ff_ticks": n_los_ff},
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
                      # The rule family each phrase was judged by; None means
                      # NO physical rule ran for it (an empty by_rule is then
                      # not a clean pass).
                      "nouns": id_cfg.get("nouns"),
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
        "det_hz": _dr["det_hz"],
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
        **({"nfz_moving_not_counted": moving_zones} if moving_zones else {}),
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
                   "citymap": str(args.citymap) if getattr(args, "citymap", None) else None,
                   # The motion shaping this flight flew (motion_params);
                   # {"legacy": true, ...} is the pre-2026-10-19 behaviour.
                   "motion": motion_params(args),
                   "legacy_tick_pose": bool(args.legacy_tick_pose),
                   "legacy_servo_width": bool(args.legacy_servo_width),
                   "grant_forecast": bool(args.grant_forecast)},
        # Where the policy came from - a signed bundle and whether it verified,
        # or a YAML file, which is unsigned. Beside the manifest, which is the
        # grant's six fields and stays so.
        "policy_source": policy_source,
    }
    if vla_record_stats is not None:
        # --record-vla only: which scene and robot config flew (the variant
        # with AerialVLA's cameras) and what was recorded (record.json in the
        # recording folder has the full counts).
        from vla_record import record_summary
        metrics["vla_record"] = {"scene": flight_scene(args),
                                 "robot_config": "robot_semantic_quad_vla.jsonc",
                                 **record_summary(vla_record_stats)}
    if decider is not None:
        # --decide-every-s only: the rate the expert was held to. Every score
        # above (frac_within_30m, track_truth's) is computed as on any flight,
        # so it reads against the expert's own figures directly.
        metrics["decide_rate"] = decider.summary()
        print(decider.summary_line())
    if hierarchy is not None:
        # --vla-hierarchy only: who flew each tick and why, the VLA's intents,
        # the stand-off it set, its answers and the server (identity and
        # adapter), with the scene variant that gave it its cameras. The
        # follow score above is computed as on any flight. The VLA decided,
        # so metrics["pilot"] names it (kind vla, or stub for a stand-in).
        if hierarchy_stats is not None:
            metrics["vla_hierarchy"] = {"scene": flight_scene(args),
                                        "robot_config": "robot_semantic_quad_vla.jsonc",
                                        **hierarchy_stats}
        try:
            metrics["pilot"] = hierarchy.pilot_entry(detector=metrics["pilot"].get("detector"))
        except Exception as exc:                                    # noqa: BLE001
            # metrics.json must still be written; the record says it is wrong.
            metrics["pilot"]["hierarchy_error"] = f"{type(exc).__name__}: {exc}"
            print(f"[warn] the hierarchy's pilot record failed: {type(exc).__name__}: {exc}")
    (out / "metrics.json").write_text(json.dumps(metrics, indent=1), encoding="utf-8")

    # --- WP4 artefacts -----------------------------------------------------
    # The grant requires a six-field determinism manifest per episode, and states
    # that contractual KPI numbers may only come from a run that has one. Ours had
    # none, so nothing measured here was reportable on the grant's own terms.
    # NO_VLA, not the detector: see flight_manifest. A hierarchical flight's
    # VLA decided, so its manifest carries the server's reported digest
    # (vla_hierarchy.VLAHierarchy.manifest) and the scene that flew.
    manifest = flight_manifest(policy.policy_hash, args.seed, args.sim_address)
    if hierarchy is not None:
        manifest = hierarchy.manifest(policy.policy_hash, args.seed,
                                      ROOT / "demo" / "pas_config" / flight_scene(args),
                                      sim_address=args.sim_address)
    (out / "manifest.json").write_text(json.dumps(manifest, indent=1), encoding="utf-8")

    kpis = kpi_mod.compute(rows, kpi_mod.rule_priorities(policy), metrics)
    graded, reasons = is_kpi_grade(manifest, metrics)
    if policy_source.get("dev_yaml"):
        # As on the ArduPilot rails: the YAML is not the issued artefact.
        reasons = list(reasons) + [f"the policy was flown from YAML under "
                                   f"{DEV_YAML_FLAG}: unsigned, not its issued bundle"]
        graded = False
    kpis["kpi_grade"] = graded
    kpis["kpi_grade_reasons"] = reasons
    kpis["manifest"] = manifest
    kpis["policy_source"] = policy_source
    # The P0 escape rate above is the Shield re-checking its own output over
    # THIS forecast, so the figure carries it; and an aborted flight says so.
    kpis["shield_lookahead"] = metrics["shield_lookahead"]
    kpis["aborted"] = aborted
    kpis.update(episode_fields(episode))
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

    # The closing lines and the exit code: non-zero for a flight that flew no
    # mission tick or aborted (final_report).
    code = final_report(metrics, len(traj), aborted)
    # The Dashboard row ends with the run's own verdict (never raises). The
    # exception class only: its message may name a folder, and the
    # Dashboard shows no paths.
    live.stop("ok" if code == 0 else "failed",
              detail=None if code == 0 else (f"aborted ({str(aborted).split(':', 1)[0]})" if aborted
                                             else "no mission tick flown"))

    cmk = metrics.get("colour_mask") or {}
    n_m, n_w = cmk.get("masked", 0), cmk.get("whole_box", 0)
    if cmk.get("enabled") and (n_m + n_w):
        print(f"[report] colour measured on the OBJECT for {n_m}/{n_m + n_w} "
              f"boxes ({n_m / (n_m + n_w) * 100:.0f}%); the rest fell back to "
              f"the whole box")
        if n_m == 0:
            print("[report] *** the depth mask never engaged - the colour gate "
                  "is still measuring the background ***")
    return code


# THE OS TIMER THE TICK SLEEPS ON.
#
# The loop sleeps to a 0.1 s deadline, and on Windows that sleep - asyncio's
# proactor wait under Python 3.10 - wakes on the system timer, 15.6 ms by
# default. Every deadline rounds UP to the next timer tick, so no tick was ever
# shorter than ~0.107 s: 9.31-9.37 Hz on every red-car flight, against a
# 9.5 Hz gate, with the work itself taking a fraction of the budget on most
# ticks. (Not on all: on CityLife car missions a level-crowd truth poll took
# the whole budget every 5th-6th tick until 2026-10-19 - update_level_truth,
# and metrics "cmd_interval" / "stage_ms_max", which now show such stalls.)
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


# The X,Y options whose first value may be negative: argparse reads a
# separate "-20,15" as an option, so `--park-at -20,15` exited with status 2
# (review of F8-harness-tools); parse_cli joins it to its flag
# (guardrail.compiler.join_negative_xy, as the costmap CLI and the rails do).
XY_FLAGS = ("--park-at",)


def _rate_range(text: str) -> tuple[float, float]:
    """'LO,HI' -> (lo, hi), 0 < lo <= hi: rad/s for --perturb-yaw-rate,
    seconds for --perturb-vla-every and --perturb-vla-dur."""
    try:
        lo, hi = (float(v) for v in str(text).split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text!r} is not LO,HI") from None
    if not (0 < lo <= hi and math.isfinite(hi)):
        raise argparse.ArgumentTypeError(f"{text!r}: need 0 < LO <= HI")
    return lo, hi


def parse_cli(ap: argparse.ArgumentParser, argv: list[str] | None = None):
    """ap.parse_args, with a negative first value of an XY_FLAGS option taken
    as that option's value."""
    argv = list(sys.argv[1:] if argv is None else argv)
    return ap.parse_args(join_negative_xy(argv, XY_FLAGS))


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim-address", default="127.0.0.1",
                    help="host of the Project AirSim server (topics 8989, services 8990); "
                         "another machine's address when this controller runs on the "
                         "Jetson Orin and the simulator on the desktop")
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
                    help="the rule set, by its YAML (default policies/follow_car.yaml): "
                         "its SIGNED bundle from bundles/ flies (tools/"
                         "build_flight_bundles.py builds it), refused when absent "
                         "or stale unless --dev-yaml")
    ap.add_argument(DEV_YAML_FLAG, dest="dev_yaml", action="store_true",
                    help="fly the YAML itself (--policy, or the default), unsigned: "
                         "a development run, recorded as such and never KPI-grade")
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
    ap.add_argument("--teacher-truth", action="store_true",
                    help="TRAINING LABELS ONLY: replace the detector's boxes by the "
                         "--level-car's TRUE position projected into each frame "
                         "(Grounder._teacher_box); the controller, the fence and the "
                         "Shield fly as always. Never a follow result: metrics.json "
                         "says teacher_truth.")
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
                         "was last measured). Off keeps the pre-2026-09-29 "
                         "candidate handling; the estimator is fed at the "
                         "capture's own pose either way (--legacy-tick-pose "
                         "for the old pairing). The subject's noun picks the "
                         "rules: car/taxi/sedan/suv, van/pickup, truck/lorry, "
                         "bus, person/pedestrian/human; any other noun runs "
                         "none, and the flight says so.")
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
    ap.add_argument("--people-stale-s", type=float, default=people_feed.PEOPLE_STALE_S,
                    help="a distance_envelope over people only (the Shield is told "
                         "every pedestrian of --level-peds / --pedestrians): a "
                         "pedestrian unseen for longer is dropped "
                         "(guardrail/people.py; default %(default)s s)")
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
                         "box width the forward channel used jitters p95 37.7%% "
                         "between detections in traffic, which pushed the raw "
                         "command 1.897 m/s in one 0.1 s tick; replayed on the "
                         "recorded flights the estimator drops that p95 6.4x "
                         "and supplies a signal on 100%% of ticks instead of "
                         "38-55%%. (Percent signs doubled: argparse formats "
                         "help with %%, and a bare one made --help crash.)")
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
                    help="half-amplitude of the search sweep about the heading "
                         "the nose had when the search began, first toward the "
                         "side the target was lost on (with a trail lookout it "
                         "is centred on the lookout bearing instead). The "
                         "sweep oscillates rather than rotating, so the nose "
                         "does not walk away. 25 deg sits inside the 45 deg "
                         "horizontal half-FOV.")
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
    ap.add_argument("--record-vla", default=None, metavar="DIR",
                    help="record what AerialVLA would see while this follow flies, "
                         "as training data (demo/vla_record.py): the robot config "
                         "gains AerialVLA's VLAFront/VLADown cameras (scene_semantic_"
                         "vla.jsonc) and DIR gets every front+down pair with its pose, "
                         "the box, the command and the track, for tools/"
                         "build_follow_dataset.py. Off by default: the flight, its "
                         "scene and its records are then unchanged.")
    ap.add_argument("--record-vla-front-source", choices=("vla", "detector"), default="vla",
                    help="with --record-vla: the front frame recorded (and asked with in "
                         "--record-perturb-vla windows): 'vla' (default) AerialVLA's VLAFront "
                         "256x256, level; 'detector' this follow's FrontCamera 768x432, pitched "
                         "-20 deg - one subscription feeding the detector and the recorder, as "
                         "vla_fly --vla-front-source detector sends it (AIOT-VLA)")
    ap.add_argument("--record-perturb", action="store_true",
                    help="with --record-vla only: DART-style noise for the training "
                         "data (vla_record.Perturber). Every 5-8 s, for 1.0-1.5 s, a "
                         "yaw-rate offset of random sign and 0.3-0.5 rad/s is added to "
                         "the EXECUTED command, after the controller and before the "
                         "Shield (which still filters it), seeded by --seed and recorded "
                         "per tick, so the recording shows the car off-centre and the "
                         "expert bringing it back. The dataset builder drops every label "
                         "that overlaps a perturbation. Off by default.")
    ap.add_argument("--perturb-speed", action="store_true",
                    help="with --record-perturb: each perturbation also scales the "
                         "horizontal speed by 0.5-1.0")
    ap.add_argument("--perturb-yaw-rate", type=_rate_range, default=(0.3, 0.5),
                    metavar="LO,HI",
                    help="with --record-perturb: the offset's magnitude range, rad/s "
                         "(default 0.3,0.5; the expert's 1.2 gain holds an added 0.3-0.5 "
                         "near the 15 deg phrase edge, vla_record.Perturber)")
    ap.add_argument("--perturb-replace-yaw", action="store_true",
                    help="with --record-perturb: while a perturbation lasts, only the "
                         "offset turns the aircraft (the expert's own yaw-rate command is "
                         "dropped), so the car drifts off-centre uncorrected")
    ap.add_argument("--record-perturb-vla", default=None, metavar="HOST:PORT",
                    help="with --record-vla only: DAgger (demo/vla_dagger.py). AerialVLA, "
                         "served by a running demo/vla_server_v2.py at HOST:PORT, flies "
                         "windows of this recording: every 6-10 s, for 2.5-4.0 s, the "
                         "EXECUTED command is the one vla_fly's follow mission would fly (its "
                         "step tracker on AerialVLA's latest step, asked from 1.5 s before "
                         "with the VLAFront/VLADown pair and DetectorHint's phrase for this "
                         "follow's box), after the controller and before the Shield (which "
                         "still filters it); the expert flies the rest, and every recovery. "
                         "Seeded by --seed, recorded per tick; the dataset builder drops every "
                         "label that overlaps a VLA-flown tick. The server must answer hello "
                         "as AerialVLA before the simulator is touched. Not with "
                         "--record-perturb. Off by default.")
    ap.add_argument("--perturb-vla-every", type=_rate_range, default=(6.0, 10.0),
                    metavar="LO,HI",
                    help="with --record-perturb-vla: seconds from one window's opening to "
                         "the next (default 6,10)")
    ap.add_argument("--perturb-vla-dur", type=_rate_range, default=(2.5, 4.0),
                    metavar="LO,HI",
                    help="with --record-perturb-vla: seconds each window lasts (default "
                         "2.5,4.0); the longest window and its 1.5 s lead-in must fit the "
                         "shortest period")
    ap.add_argument("--perturb-vla-allow-stand-in", action="store_true",
                    help="with --record-perturb-vla: fly the windows from a server that is "
                         "not AerialVLA (the scripted stand-in: a plumbing dry run, recorded "
                         "as one, never training data)")
    ap.add_argument("--perturb-vla-bottom-tile", choices=("down", "prev_front"),
                    default="down",
                    help="with --record-perturb-vla: the mosaic's lower tile the windows ask "
                         "with (vla_fly --vla-bottom-tile): 'prev_front' is the front frame "
                         "--perturb-vla-prev-dt-s earlier. Must be the tile the served adapter "
                         "was trained on, or the recording is refused (default down)")
    ap.add_argument("--perturb-vla-prev-dt-s", type=float, default=None,
                    help="with --perturb-vla-bottom-tile prev_front: how many seconds earlier "
                         "(vla_fly --vla-prev-dt-s; default 1.0)")
    ap.add_argument("--perturb-vla-step-as-rate", type=float, default=None, metavar="H",
                    help="with --record-perturb-vla: the windows fly each step as a velocity "
                         "over H seconds (vla_fly --step-as-rate; default off: an endpoint)")
    ap.add_argument("--perturb-vla-rate-yaw", choices=("heading", "rate"), default="heading",
                    help="with --perturb-vla-step-as-rate: how a window flies the step's dpsi "
                         "(vla_fly --rate-yaw; 'rate' for a model whose label is a yaw-rate "
                         "command x H, aiotvla)")
    ap.add_argument("--perturb-vla-speed-words", action="store_true",
                    help="with --record-perturb-vla: the windows ask with the aircraft's own "
                         "speed as words after the object (vla_fly --vla-speed-words). Must be "
                         "what the served adapter was trained with, or the recording is "
                         "refused (default off)")
    ap.add_argument("--decide-every-s", type=float, default=None, metavar="P",
                    help="the CEILING flight (demo/decide_rate.py): hold this follow to a "
                         "VLA's decision rate. The EXECUTED command, after the controller and "
                         "its slew limiter and before the Shield (which still filters it), "
                         "becomes a zero-order hold of the expert's own command: a decision "
                         "every P mission seconds (on the grid k*P), held in between, all four "
                         "axes; the controller runs every tick as usual. Recorded per tick "
                         "(`decide`) and in metrics.json (`decide_rate`); the follow score is "
                         "computed as on any flight. Not with --record-vla, --record-perturb or "
                         "--record-perturb-vla. Off by default.")
    ap.add_argument("--decide-delay-s", type=float, default=None, metavar="D",
                    help="with --decide-every-s: each decision carries the expert's command "
                         "from D seconds before it (a VLA's capture-to-answer latency; a hover "
                         "until a command is that old). Default 0")
    ap.add_argument("--vla-hierarchy", default=None, metavar="HOST:PORT",
                    help="HIERARCHICAL control (demo/vla_hierarchy.py): AerialVLA, served by a "
                         "running demo/vla_server_v2.py at HOST:PORT, is asked continuously "
                         "with the VLAFront/VLADown pair (this scene's variant with them, no "
                         "recording) and DetectorHint's phrase for this follow's box, and each "
                         "answer sets the intent this controller executes at 10 Hz: with the "
                         "subject locked the controller flies as always with its stand-off at "
                         "the VLA's desired range (--vla-hierarchy-range, moved at most 2 m/s); "
                         "with it lost the VLA's step tracker (vla_fly's follow config) flies "
                         "the search; a LAND holds at the stand-off in force; an answer older "
                         "than 1.5 s hands everything back to the controller's own behaviour. "
                         "After the controller and its slew limiter, before the Shield (which "
                         "still filters every command); recorded per tick (`hierarchy`) and in "
                         "metrics.json (`vla_hierarchy`); the follow score is computed as on "
                         "any flight. The server must answer hello as AerialVLA before the "
                         "simulator is touched. Not with --record-vla, --record-perturb, "
                         "--record-perturb-vla or --decide-every-s. Off by default.")
    ap.add_argument("--vla-hierarchy-range", type=_rate_range, default=(12.0, 22.0),
                    metavar="NEAR,FAR",
                    help="with --vla-hierarchy: the stand-off the VLA may ask for, metres: NEAR "
                         "at its longest step (fwd 5 m), FAR at a zero step, linear between and "
                         "clamped (default 12,22)")
    ap.add_argument("--vla-hierarchy-allow-stand-in", action="store_true",
                    help="with --vla-hierarchy: a server that is not AerialVLA (the scripted "
                         "stand-in: a plumbing dry run, recorded as one)")
    ap.add_argument("--vla-hierarchy-bottom-tile", choices=("down", "prev_front"),
                    default="down",
                    help="with --vla-hierarchy: the mosaic's lower tile it asks with (vla_fly "
                         "--vla-bottom-tile). Must be the tile the served adapter was trained "
                         "on, or the flight is refused (default down)")
    ap.add_argument("--vla-hierarchy-prev-dt-s", type=float, default=None,
                    help="with --vla-hierarchy-bottom-tile prev_front: how many seconds earlier "
                         "(vla_fly --vla-prev-dt-s; default 1.0)")
    ap.add_argument("--vla-hierarchy-step-as-rate", type=float, default=None, metavar="H",
                    help="with --vla-hierarchy: the search flies each step as a velocity over "
                         "H seconds (vla_fly --step-as-rate; default off: an endpoint)")
    ap.add_argument("--vla-hierarchy-speed-words", action="store_true",
                    help="with --vla-hierarchy: ask with the aircraft's own speed as words "
                         "after the object (vla_fly --vla-speed-words). Must be what the "
                         "served adapter was trained with, or the flight is refused")
    ap.add_argument("--vla-hierarchy-front-source", choices=("vla", "detector"), default="vla",
                    help="with --vla-hierarchy: the request's front frame, VLAFront (default) "
                         "or this flight's FrontCamera (768x432, -20 deg) for a foreign "
                         "server that reads it (vla_fly --vla-front-source). Must be the "
                         "camera the server reads, or the flight is refused")
    ap.add_argument("--vla-hierarchy-expiry-s", type=float, default=1.5, metavar="S",
                    help="with --vla-hierarchy: how long an answer's intent holds before the "
                         "controller takes over again (default 1.5). Raise it only for a model "
                         "slower than that per decision (DroneCATS int4 on the Orin Nano, about "
                         "9.6 s); the flight records the value")
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
    ap.add_argument("--guard-field", choices=("grid", "shield"), default="grid",
                    help="what FenceGuard's building half measures clearance on: "
                         "'grid' (default; its own scan of the occupancy grid, what "
                         "every stored follow flight flew) or 'shield' (the Shield's "
                         "distance field, one measure with the rule it defends; "
                         "WP3-22 / X-13, opt-in until flown on the AGX)")
    ap.add_argument("--dv-h", type=float, default=0.4,
                    help="the legacy limiter's step, m/s per axis per tick. "
                         "With the default motion shaping it is the FLOOR on "
                         "braking: a braking command (slower, and pointing back "
                         "along the track) never ends up faster than this step "
                         "would have left it.")
    ap.add_argument("--dv-z", type=float, default=0.2,
                    help="climb-rate step per tick (dv_z/0.1 s is the climb "
                         "acceleration), and the same floor on slowing a climb")
    add_motion_args(ap)
    ap.add_argument("--legacy-tick-pose", action="store_true",
                    help="without --identity, feed the estimator the box's "
                         "bearing at the CONSUMING tick's pose and clock with "
                         "the newest depth frame (before 2026-10-19), instead "
                         "of the capture's own pose, time and depth. Part of "
                         "reproducing a recorded non-identity flight; "
                         "--legacy-motion's help lists what no flag restores.")
    ap.add_argument("--legacy-servo-width", action="store_true",
                    help="the box servo (--no-target-estimator) holds the raw "
                         "--want-width fraction, as before 2026-10-19, instead "
                         "of the box fraction of the estimator arm's metric "
                         "stand-off (0.126 for 0.16 on a 4 m car: 12.5 m "
                         "against 15.83 m before).")
    ap.add_argument("--grant-forecast", action="store_true",
                    help="the grant's 5 s / 0.1 s forecast with the escalation "
                         "FSM OFF, as the --grant-forecast runs before "
                         "2026-11-03 flew (the default is the same forecast "
                         "with the FSM on)")
    ap.add_argument("--legacy-forecast", action="store_true",
                    help="the 3 s / 0.5 s forecast every city flight before "
                         "2026-11-03 flew, with the escalation FSM; the default "
                         "is the grant's 5 s / 0.1 s now that the subject's "
                         "velocity reaches the Shield")
    return ap


def main(argv: list[str] | None = None) -> int:
    ap = build_arg_parser()
    args = parse_cli(ap, argv)
    if args.live_view and not args.save_view:
        # The window shows the frames the recorder writes. Without --save-view
        # the recorder is never started, so there is nothing to show. Refusing
        # is clearer than silently enabling capture the caller did not ask for,
        # which would also change where the flight writes.
        ap.error("--live-view needs --save-view: the window draws the frames "
                 "the recorder produces, and without it none are captured")
    if args.decide_every_s is not None:
        # The ceiling flight (demo/decide_rate.py) measures the expert held to
        # a VLA's rate, and nothing else may replace the command it executes.
        if args.record_perturb or args.record_perturb_vla is not None:
            ap.error("--decide-every-s and --record-perturb / --record-perturb-vla: the "
                     "ceiling flight holds the expert's own command, one replacement at a time")
        if args.record_vla:
            ap.error("--decide-every-s and --record-vla: a ceiling flight is a measurement, and "
                     "its held commands must not become training labels")
        from decide_rate import check_args as decide_check
        why = decide_check(args.decide_every_s,
                           0.0 if args.decide_delay_s is None else args.decide_delay_s)
        if why is not None:
            ap.error(why)
    elif args.decide_delay_s is not None:
        ap.error("--decide-delay-s needs --decide-every-s")
    if args.vla_hierarchy is not None:
        # HIERARCHICAL CONTROL (demo/vla_hierarchy.py): the VLA's intent,
        # executed by this controller. It owns the executed command with the
        # controller, so nothing else may replace it, and its motion is not
        # the expert's, so it is never recorded as training data.
        if (args.record_perturb or args.record_perturb_vla is not None
                or args.decide_every_s is not None):
            ap.error("--vla-hierarchy and --record-perturb / --record-perturb-vla / "
                     "--decide-every-s: one replacement of the executed command at a time")
        if args.record_vla:
            ap.error("--vla-hierarchy and --record-vla: a hierarchical flight flies the VLA's "
                     "intent, not the expert's, and must not become training labels (it flies "
                     "AerialVLA's cameras without a recording)")
        from vla_hierarchy import check_args as hierarchy_check
        why = hierarchy_check(args.vla_hierarchy, s_range=args.vla_hierarchy_range,
                              bottom_tile=args.vla_hierarchy_bottom_tile,
                              prev_dt_s=args.vla_hierarchy_prev_dt_s,
                              step_as_rate=args.vla_hierarchy_step_as_rate,
                              speed_words=args.vla_hierarchy_speed_words,
                              front_source=args.vla_hierarchy_front_source,
                              expiry_s=args.vla_hierarchy_expiry_s)
        if why is not None:
            ap.error(why)
    elif (args.vla_hierarchy_range != (12.0, 22.0) or args.vla_hierarchy_allow_stand_in
          or args.vla_hierarchy_bottom_tile != "down" or args.vla_hierarchy_prev_dt_s is not None
          or args.vla_hierarchy_step_as_rate is not None or args.vla_hierarchy_speed_words
          or args.vla_hierarchy_front_source != "vla"
          or args.vla_hierarchy_expiry_s != 1.5):
        ap.error("--vla-hierarchy-range, --vla-hierarchy-allow-stand-in, "
                 "--vla-hierarchy-bottom-tile, --vla-hierarchy-prev-dt-s, "
                 "--vla-hierarchy-step-as-rate, --vla-hierarchy-speed-words, "
                 "--vla-hierarchy-front-source and --vla-hierarchy-expiry-s need "
                 "--vla-hierarchy")
    if args.record_perturb and not args.record_vla:
        # A perturbed follow is worth flying only to record it: the noise is
        # there so the training data shows a recovery (vla_record.Perturber).
        ap.error("--record-perturb needs --record-vla: the perturbation exists only "
                 "to be recorded as training data")
    if args.record_vla_front_source != "vla" and not args.record_vla:
        ap.error("--record-vla-front-source needs --record-vla")
    if args.perturb_speed and not args.record_perturb:
        ap.error("--perturb-speed needs --record-perturb")
    if not args.record_perturb and (args.perturb_replace_yaw
                                    or args.perturb_yaw_rate != (0.3, 0.5)):
        ap.error("--perturb-replace-yaw and --perturb-yaw-rate need --record-perturb")
    if args.record_perturb_vla is not None:
        # DAgger (demo/vla_dagger.py): the VLA's windows are training data,
        # worth flying only into a recording, and one perturbation at a time
        # (each is the recording's `perturb`).
        if not args.record_vla:
            ap.error("--record-perturb-vla needs --record-vla: AerialVLA flies windows of a "
                     "recording, so the expert's recovery can be labelled")
        if args.record_perturb:
            ap.error("--record-perturb-vla and --record-perturb: one perturbation at a time")
        from vla_dagger import check_args
        why = check_args(args.record_perturb_vla, args.perturb_vla_every, args.perturb_vla_dur,
                         bottom_tile=args.perturb_vla_bottom_tile,
                         prev_dt_s=args.perturb_vla_prev_dt_s,
                         step_as_rate=args.perturb_vla_step_as_rate,
                         speed_words=args.perturb_vla_speed_words,
                         rate_yaw=args.perturb_vla_rate_yaw,
                         front_source=args.record_vla_front_source)
        if why is not None:
            ap.error(why)
    elif (args.perturb_vla_every != (6.0, 10.0) or args.perturb_vla_dur != (2.5, 4.0)
          or args.perturb_vla_allow_stand_in):
        ap.error("--perturb-vla-every, --perturb-vla-dur and --perturb-vla-allow-stand-in "
                 "need --record-perturb-vla")
    elif (args.perturb_vla_bottom_tile != "down" or args.perturb_vla_prev_dt_s is not None
          or args.perturb_vla_step_as_rate is not None or args.perturb_vla_speed_words
          or args.perturb_vla_rate_yaw != "heading"):
        ap.error("--perturb-vla-bottom-tile, --perturb-vla-prev-dt-s, "
                 "--perturb-vla-step-as-rate, --perturb-vla-rate-yaw and "
                 "--perturb-vla-speed-words need --record-perturb-vla")
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
