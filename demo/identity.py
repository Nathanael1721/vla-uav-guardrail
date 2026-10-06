"""Is this red box the car? Physical features of a detection, and a verdict.

WHY

"Follow the red car" was following the wrong red thing on 64 % of estimator
ticks across the trail/final flights: red pedestrian signals 2-3 m up at the
kerb, red signs, traffic cones, boxes more than 60 m away, vehicle signals
4-5 m up. Every one of them passes the colour gate, and the detector's score
barely separates them. What does separate them is physical. A box is an
angle and depth gives the range. From those two you get a width in metres,
a height above the road, and a point on the map that is either on the street
or not. A car is about 1.8-4.5 m wide, stands on the road, and is on the
street.

A labelled pass over the logged boxes (see tools/replay_identity.py; the
table it writes is demo/out/identity_replay/report.md) measured which of
these separate on-car from off-car boxes. Within 60 m the useful ones are
physical width, box aspect w/h, detector score, the height of the box's
bottom edge above the road, and distance from the street mask. Beyond 60 m
depth is quantised to 1 m and a car is a few pixels tall, so the useful
ones are aspect, pixel height and distance from the street.

Measured within 60 m on 1,293 on-car / 2,152 off-car boxes (pinhole
widths, capture-time pose). Medians on-car vs off-car:
  width 2.45 vs 1.11 m        aspect 1.44 vs 0.92
  score 0.09 vs 0.02          bottom_h (pitch logged) 0.23 vs 0.43 m, but
  off-car p75 is 2.58 m against on-car p95 1.14 m.
With the tuned thresholds (demo/identity_thresholds.json, 12 flights,
2,432 off-car / 1,431 on-car boxes), pooled:
  in-sample         off-car not-OK 86.4 %, HARD 38.7 %;
                    on-car not-OK 14.5 %, HARD 0.8 %
  leave-one-out     off-car not-OK 86.5 %, HARD 39.2 %;
                    on-car not-OK 15.4 %, HARD 0.9 %
Of the 331 off-car boxes that still pass, 282 are car-sized and on the
road. They are other vehicles, which no physical rule can tell from the
red car; that is the instance tracker's job, not this module's.

TWO TIERS, because the costs are not symmetric. A HARD reject is physical
impossibility for the named thing: a car whose bottom edge is 2.5 m above
the road, a car 5 m off every street, a "car" 9 m wide, a box filling the
frame. The follow must never steer on those. A SOFT demotion is "unlikely,
but a real car can look like this". The case that forces this tier is a car
partly hidden behind a truck: it shows as an 11-15 px sliver, 0.6-1.0 m
"wide", aspect 0.55-0.65. That is exactly the signature of a pedestrian
signal. A hard rule there throws the car away at the moment it is hardest
to keep, and citylife_redcar_trail already lost the car once to a hard check
that refused it: the ground check, 34 inferences through the second corner
(follow_vlm.py). So width, aspect and score only DEMOTE. In the replay
the near on-car slivers came out SOFT 35, OK 7, HARD 0. The caller
decides what a demotion costs (e.g. SOFT boxes may continue an existing
track but may not start one).

KNOWN LIMIT OF bottom_h. It places the bottom-edge point at the CENTRE's
range. For a deep object seen steeply that is too far along the ray. A car
at 6.5-7.7 m below an 8 m aircraft (bottom ray 41-46 deg down) has its
roof in the depth window and reads 1.8-2.0 m "up". Six of the 11 on-car
HARDs in the replay are that one episode (final2 seq 55-67). Two more are
at 75-78 m, where 1 deg of pitch error is 1.3 m of height. dep_bottom_deg
is returned so a caller can see when bottom_h is in that regime.

A MISSING FEATURE NEVER HARD-REJECTS. With no depth there is no width, no
bottom height and no map point. That is "unknown", which for a vehicle is
SOFT and never HARD. The same holds when pitch was not measured:
bottom_h is None and its rule does not fire.

Frames: NED metres. x = North, y = East, up = height above the flat ground
(positive). Yaw from North, clockwise. Pitch nose-UP positive, roll
right-wing-DOWN positive, both in radians. The box is in FrontCamera pixels
(768x432, HFOV 90 deg, mounted 20 deg nose-down). All geometry goes through
demo/camera_model.py (pinhole). The old LINEAR pixel->angle map
under-reported widths by ~21 % at the centre.

The pose passed in should be the pose AT IMAGE CAPTURE. The tick pose lags
the frame by the inference (median 234 ms), the detection's age at the
tick, and ~0.2 s of image age when the detector fetched it: 0.50 s median,
0.59 s p90 over the 5,815 joined boxes. That is 3.4 deg of yaw median (p90
14.3, max 29.7) and 1.8 deg of pitch (p90 6.9, max 19.9). At 20 m, 7 deg of
pitch moves bottom_h by 2.5 m. The replay's sensitivity row shows these
thresholds on the tick pose: on-car HARD rises from 0.8 to 1.3 %.

Pure: numpy only for the street distance field. Tests: tests/test_identity.py.
Tuned and validated offline by tools/replay_identity.py (leave-one-flight-out).
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Callable, Optional, Sequence, Tuple

import numpy as np

import camera_model as cam

HERE = Path(__file__).resolve().parent
DEFAULT_THRESHOLDS = HERE / "identity_thresholds.json"
DEFAULT_STREET = HERE / "out" / "citymap_citylife" / "street.npz"

VEHICLE_NOUNS = ("car", "truck", "bus", "van")
PERSON_NOUNS = ("person", "pedestrian", "man", "woman", "child", "people",
                "figure", "human")

# What a READER sees for each tier. The values stay "hard" / "soft" / "ok":
# they are what every flight log, threshold file and replay since 2026-09-29
# records, and renaming them would orphan all of it. The display names change
# because the grant's Policy DSL already uses hard/soft for POLICY rules
# (constraint_type, guardrail/models.py), and at the 2026-09-30 meeting
# "HARD / SOFT" on the identity slide was heard as kinds of manoeuvre. Slides,
# HUD and documents say REJECT / DOUBTFUL / OK from 2026-10-03.
TIER_LABEL = {"hard": "REJECT", "soft": "DOUBTFUL", "ok": "OK"}

# Metres returned for a point outside the street grid. "Unmapped" is not
# evidence of a road (same stance as build_street_mask.is_street), so it is
# far enough to trip any off-street rule. Finite so it survives JSON.
OFF_GRID_M = 1000.0

# The rule values. demo/identity_thresholds.json overrides them. These are
# the fallback when that file is missing, and they fill any key it omits.
DEFAULTS = {
    # --- vehicles: HARD (physically not the named thing) ---
    "bottom_max": 2.0,          # m; box bottom edge this far above the road
    "off_street_hard": 3.0,     # m from the nearest street cell
    "width_max": 8.0,           # m; nothing drivable is wider
    "frame_frac_max": 0.85,     # box width / image width; a wall
    # --- vehicles, rng_h <= near_far_m: SOFT ---
    "near_far_m": 60.0,
    "width_min": 1.0,           # m
    "aspect_min": 0.8,          # w/h px
    "score_min": 0.0,
    # --- vehicles, rng_h > near_far_m: SOFT ---
    "far_aspect_min": 1.3,
    "far_h_px_max": 14.0,
    "far_off_street": 1.5,      # m
    # --- people: HARD only ---
    "person_bottom_max": 1.5,
    "person_off_street_hard": 3.0,
    # --- geometry ---
    "min_depression_deg": 3.0,  # below this the bottom ray is too shallow
}
RULE_KEYS = tuple(sorted(DEFAULTS))

# A box whose bottom edge lies within this many pixels of the frame's bottom
# row is cut off by the frame (see features_for, bottom_clipped).
CLIP_PX = 3.0


# ---------------------------------------------------------------------------
# features
# ---------------------------------------------------------------------------
def _opt(v) -> Optional[float]:
    """float(v), or None for a missing or non-finite value."""
    if v is None:
        return None
    v = float(v)
    return v if math.isfinite(v) else None


def _box_fields(box) -> Tuple[float, float, float, float, Optional[float],
                              Optional[float]]:
    """(cx, cy, w, h, score, colour) from any of the box shapes in this repo:
    a dict {cx, cy, w, h, score?, colour?} (detections.jsonl `det`); the
    follow_vlm detector tuple (cx, cy, w, h, score, W, H, colour); or a
    detections.jsonl `cands` row [cx, cy, w, h, score, colour]."""
    if isinstance(box, dict):
        return (float(box["cx"]), float(box["cy"]), float(box["w"]),
                float(box["h"]), _opt(box.get("score")),
                _opt(box.get("colour")))
    b = list(box)
    if len(b) >= 8:
        return (float(b[0]), float(b[1]), float(b[2]), float(b[3]),
                _opt(b[4]), _opt(b[7]))
    if len(b) == 6:
        return (float(b[0]), float(b[1]), float(b[2]), float(b[3]),
                _opt(b[4]), _opt(b[5]))
    if len(b) == 4:
        return float(b[0]), float(b[1]), float(b[2]), float(b[3]), None, None
    raise ValueError(f"unrecognised box shape: {box!r}")


def features_for(box, depth_range_m: Optional[float], pose: dict,
                 img_w: float, img_h: float, hfov: float,
                 street_dist: Optional[Callable] = None, *,
                 r_src: str = "depth",
                 mount_pitch_deg: float = cam.MOUNT_PITCH_DEG,
                 min_depression_deg: float = DEFAULTS["min_depression_deg"]
                 ) -> dict:
    """The physical description of one detection box.

    box          see _box_fields (pixels, centre + size).
    depth_range_m  SLANT range to the object (DEPTH_PERSPECTIVE median over
                 the box's middle), or None when depth had nothing.
    pose         dict(x, y, up, yaw, pitch, roll): NED m, rad. pitch=None
                 means "not measured": the rays use 0, and bottom_h is None
                 rather than a number computed from a guessed attitude.
                 roll=None uses 0.
    street_dist  callable((north, east, up)) -> metres to the street, e.g. a
                 StreetDistance. None leaves off_street_m None.
    r_src        a label for where the range came from, carried through.

    Returned keys: r, r_src, P (north, east, up of the box centre at range
    r), rng_h, bearing (world azimuth of the centre ray, rad, clockwise from
    North), width_m, aspect, w_px, h_px, frac_w, bottom_h, off_street_m,
    score, colour, pitch_measured, dep_bottom_deg (depression of the
    bottom-centre ray; the steeper it is, the more a deep object's
    centre-range lifts bottom_h, see classify). Every geometric key is None
    when its inputs are missing. A non-finite pitch or roll counts as not
    measured; a non-finite x, y, up or yaw means the box cannot be placed, so
    P, rng_h, bearing, bottom_h and off_street_m are None (for a vehicle that
    is SOFT "no range", never HARD).
    """
    cx, cy, bw, bh, score, colour = _box_fields(box)
    pitch_raw = _opt(pose.get("pitch"))
    pitch = 0.0 if pitch_raw is None else pitch_raw
    roll = _opt(pose.get("roll")) or 0.0
    yaw, x, y, up = (_opt(pose.get(k)) for k in ("yaw", "x", "y", "up"))
    placed = None not in (yaw, x, y, up)

    ray_c = (cam.ray_world(cx, cy, img_w, img_h, hfov, yaw, pitch, roll,
                           mount_pitch_deg) if placed else None)
    r = _opt(depth_range_m)
    r = r if r is not None and r > 0 else None

    P = rng_h = width_m = bottom_h = off_street = dep_b = None
    # A box cut off by the bottom of the frame has no bottom edge of its own:
    # the object runs on under the nose (a car inside ~7 m at 8 m altitude)
    # and its "bottom" is the frame's. On final2 that read 1.8-2.0 m above the
    # road and HARD-rejected the car the drone was following.
    clipped = cy + bh / 2.0 >= img_h - CLIP_PX
    if r is not None:
        width_m = r * cam.box_angular_width(cx, bw, img_w, hfov)
    if r is not None and placed:
        P = cam.point_at(ray_c, r, x, y, up)
        rng_h = cam.horizontal_range(ray_c, r)
        if pitch_raw is not None:
            ray_b = cam.ray_world(cx, cy + bh / 2.0, img_w, img_h, hfov, yaw,
                                  pitch, roll, mount_pitch_deg)
            dep = cam.depression(ray_b)
            dep_b = math.degrees(dep)
            if dep >= math.radians(min_depression_deg) and not clipped:
                bottom_h = up - r * math.sin(dep)
        if street_dist is not None:
            d = float(street_dist(P))
            off_street = d if math.isfinite(d) else None

    return {
        "r": r, "r_src": (r_src if r is not None else None),
        "P": P, "rng_h": rng_h,
        "bearing": (math.atan2(ray_c[1], ray_c[0]) if placed else None),
        "width_m": width_m,
        "aspect": (bw / bh) if bh > 0 else None,
        "w_px": bw, "h_px": bh,
        "frac_w": bw / float(img_w),
        "bottom_h": bottom_h,
        "off_street_m": off_street,
        "score": score,
        "colour": colour,
        "pitch_measured": pitch_raw is not None,
        "dep_bottom_deg": dep_b,
        "bottom_clipped": clipped,
    }


# ---------------------------------------------------------------------------
# street distance
# ---------------------------------------------------------------------------
class StreetDistance:
    """Metres from a world point to the nearest street cell.

    The grid convention is demo/city_planner.py's: cell (i, j) is CENTRED at
    (origin_x + i*res, origin_y + j*res), i along North. The field is
    city_planner.distance_field over the street cells (the "blocked" seeds
    are the street), so it is 0 on the street and the chamfer distance to
    the nearest street-cell centre elsewhere. A query is interpolated
    bilinearly between the four surrounding cell centres, which puts a point
    on a kerb about 1 m out instead of snapping it to 0 or 2 m at a 2 m
    grid. Off the grid returns OFF_GRID_M.
    """

    def __init__(self, street, res: float, origin_x: float, origin_y: float):
        import city_planner                     # numpy-only; lazy for tests
        s = (np.asarray(street) > 0).astype(np.uint8)
        self.street = s
        self.res = float(res)
        self.ox, self.oy = float(origin_x), float(origin_y)
        self.field = city_planner.distance_field(s, self.res)
        if not s.any():
            self.field = np.full(s.shape, OFF_GRID_M)

    @classmethod
    def load(cls, path=None) -> "StreetDistance":
        d = np.load(Path(path) if path else DEFAULT_STREET)
        return cls(d["street"], float(d["res"]), float(d["origin_x"]),
                   float(d["origin_y"]))

    def __call__(self, p: Sequence[float]) -> float:
        """Metres to the street; NaN for a non-finite point (an unknown place
        is not "off the street", so it must not trip a HARD rule)."""
        n, e = float(p[0]), float(p[1])
        if not (math.isfinite(n) and math.isfinite(e)):
            return math.nan
        N, M = self.field.shape
        fi = (n - self.ox) / self.res
        fj = (e - self.oy) / self.res
        if not (-0.5 <= fi <= N - 0.5 and -0.5 <= fj <= M - 0.5):
            return OFF_GRID_M
        fi = min(max(fi, 0.0), N - 1.0)
        fj = min(max(fj, 0.0), M - 1.0)
        i0, j0 = int(math.floor(fi)), int(math.floor(fj))
        i1, j1 = min(i0 + 1, N - 1), min(j0 + 1, M - 1)
        a, b = fi - i0, fj - j0
        f = self.field
        return float((1 - a) * (1 - b) * f[i0, j0] + (1 - a) * b * f[i0, j1]
                     + a * (1 - b) * f[i1, j0] + a * b * f[i1, j1])


# ---------------------------------------------------------------------------
# verdict
# ---------------------------------------------------------------------------
_PLURALS = {"cars": "car", "trucks": "truck", "buses": "bus", "busses": "bus",
            "vans": "van", "persons": "person", "pedestrians": "person",
            "men": "person", "women": "person", "children": "person"}


def noun_of(query: Optional[str]) -> Optional[str]:
    """The rule family a query names: a vehicle noun, "person", or None.
    "a red car" -> "car"; "the pedestrian in red" -> "person".

    The FIRST known noun wins: in an English noun phrase the head comes before
    its modifiers, so "the red car past the pedestrian crossing" is a car and
    "the person next to the red car" is a person. Scanning from the end got
    both of those wrong (it would put a car query under the person rules at
    every zebra crossing in CityLife)."""
    if not query:
        return None
    words = [w.strip(".,!?;:'\"()").lower() for w in str(query).split()]
    for w in words:
        w = _PLURALS.get(w, w)
        if w in VEHICLE_NOUNS:
            return w
        if w in PERSON_NOUNS or w == "person":
            return "person"
    return None


def load_thresholds(path=None) -> dict:
    """The rule values: DEFAULTS overlaid with the JSON file (default
    demo/identity_thresholds.json; a missing file gives DEFAULTS). Non-rule
    keys in the file ("note", "tuned_on", results) are kept as they are."""
    th = dict(DEFAULTS)
    p = Path(path) if path else DEFAULT_THRESHOLDS
    if p.is_file():
        with p.open(encoding="utf-8") as fh:
            data = json.load(fh)
        for k, v in data.items():
            if k in DEFAULTS:
                if v is not None:               # null keeps the default
                    th[k] = float(v)
            else:
                th[k] = v
    return th


def thresholds_hash(thresholds: Optional[dict] = None) -> str:
    """12-hex fingerprint of the RULE values only, for logs and manifests.
    Key order, notes and tuning metadata do not change it."""
    th = load_thresholds() if thresholds is None else thresholds
    rules = {k: round(float(th.get(k, DEFAULTS[k])), 6) for k in RULE_KEYS}
    blob = json.dumps(rules, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:12]


def classify(features: dict, noun: Optional[str],
             thresholds: Optional[dict] = None) -> Tuple[str, list]:
    """("ok" | "soft" | "hard", reasons): classify_split with the HARD and
    SOFT reasons joined, HARD first."""
    tier, hard, soft = classify_split(features, noun, thresholds)
    return tier, (hard + soft if tier == "hard" else soft if tier == "soft" else [])


def classify_split(features: dict, noun: Optional[str],
                   thresholds: Optional[dict] = None) -> Tuple[str, list, list]:
    """(tier, hard_reasons, soft_reasons) for a features_for() dict and the
    named noun (a noun or a whole query; see noun_of). Split so a counter can
    file each reason under the rule that produced it: a HARD box also carries
    its SOFT reasons, which only ever demote.

    Vehicles
      HARD  bottom_h > bottom_max; off_street_m > off_street_hard;
            width_m > width_max; box wider than frame_frac_max of the frame.
      SOFT  within near_far_m (horizontal): width_m < width_min,
            aspect < aspect_min, score < score_min, or no range at all;
            beyond it: aspect < far_aspect_min, h_px > far_h_px_max,
            off_street_m > far_off_street.
    People
      HARD  bottom_h > person_bottom_max; off_street_m > person_off_street_hard.
    Anything else: "ok", no rules.

    A missing feature (None) never fires a HARD rule. The reasons list has
    every rule that fired, HARD ones first.
    """
    th = dict(DEFAULTS)
    th.update({k: v for k, v in (thresholds or {}).items() if v is not None})
    n = noun_of(noun)
    f = features
    hard, soft = [], []

    def big(key, lim):
        v = f.get(key)
        return v is not None and v > lim

    def small(key, lim):
        v = f.get(key)
        return v is not None and v < lim

    if n in VEHICLE_NOUNS:
        if big("bottom_h", th["bottom_max"]):
            hard.append(f"bottom {f['bottom_h']:.1f} m above the road "
                        f"> {th['bottom_max']:g}")
        if big("off_street_m", th["off_street_hard"]):
            hard.append(f"{f['off_street_m']:.1f} m off the street "
                        f"> {th['off_street_hard']:g}")
        if big("width_m", th["width_max"]):
            hard.append(f"{f['width_m']:.1f} m wide > {th['width_max']:g}")
        if big("frac_w", th["frame_frac_max"]):
            hard.append(f"box is {f['frac_w']:.0%} of the frame")

        rng_h = f.get("rng_h")
        if f.get("r") is None or rng_h is None:
            soft.append("no range")
        if rng_h is None or rng_h <= th["near_far_m"]:
            if small("width_m", th["width_min"]):
                soft.append(f"{f['width_m']:.2f} m wide < {th['width_min']:g}")
            if small("aspect", th["aspect_min"]):
                soft.append(f"aspect {f['aspect']:.2f} < {th['aspect_min']:g}")
            if small("score", th["score_min"]):
                soft.append(f"score {f['score']:.3f} < {th['score_min']:g}")
        else:
            if small("aspect", th["far_aspect_min"]):
                soft.append(f"far: aspect {f['aspect']:.2f} "
                            f"< {th['far_aspect_min']:g}")
            if big("h_px", th["far_h_px_max"]):
                soft.append(f"far: {f['h_px']:.0f} px tall "
                            f"> {th['far_h_px_max']:g}")
            if big("off_street_m", th["far_off_street"]):
                soft.append(f"far: {f['off_street_m']:.1f} m off the street "
                            f"> {th['far_off_street']:g}")
    elif n == "person":
        if big("bottom_h", th["person_bottom_max"]):
            hard.append(f"bottom {f['bottom_h']:.1f} m above the ground "
                        f"> {th['person_bottom_max']:g}")
        if big("off_street_m", th["person_off_street_hard"]):
            hard.append(f"{f['off_street_m']:.1f} m off the street "
                        f"> {th['person_off_street_hard']:g}")
    else:
        return "ok", [], []

    if hard:
        return "hard", hard, soft
    if soft:
        return "soft", [], soft
    return "ok", [], []
