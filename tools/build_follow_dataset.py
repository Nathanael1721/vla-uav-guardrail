"""
AerialVLA training samples from the follow expert's recordings: the step
label is what the hand-written follow did next.

    # vla-real (Pillow, for the mosaics); the recordings of
    # run_citylife_follow.ps1 -RecordVla / follow_vlm --record-vla
    python tools/build_follow_dataset.py demo/out/ft_expert_01 demo/out/ft_expert_02 \\
        demo/out/ft_expert_03 --out C:/Users/natha/vla_drone_offload/datasets/follow_v1 \\
        --horizon-s 1.0 --stride-s 0.5 --holdout ft_expert_03

    # labels and stats only, no Pillow (CI, a quick look)
    python tools/build_follow_dataset.py demo/out/ft_expert_01 --out <dir> --no-images

    # the lower tile as the FRONT frame 1 s earlier (THE LOWER TILE, below)
    python tools/build_follow_dataset.py ... --bottom-tile prev_front --prev-dt-s 1.0 \\
        --prev-tol-s 0.25 --out C:/Users/natha/vla_drone_offload/datasets/follow_v3_pf

    # also the follow's memory and search hints, with the flight's close rule
    # (MEMORY AND SEARCH, THE CLOSE RULE, below)
    python tools/build_follow_dataset.py ... --hint-modes fresh,memory,search \\
        --close-bottom-frac 1.0 --close-width-frac 0.25

Each RUN_DIR is a flight folder (demo/out/<tag>, holding vla_record/) or the
recording folder itself (the one holding captures.jsonl). Exit 0 when the
dataset was written, 2 when it was refused (an unknown --holdout tag, two
recordings with one tag, no training run left, an existing dataset without
--force, Pillow missing without --no-images).

WHAT A SAMPLE IS (wave FT-D, 2026-10-23; the brief's section FT-D)

For every recorded capture with a FRESH box - the expert's box no older than
--fresh-s (1 s) at the capture, DetectorHint's rule, the only kind of hint a
VLA follow sends on a live sighting - one sample (with --hint-modes, also
the captures a follow sends its memory or search hint on: MEMORY AND
SEARCH, below):

  image   the codec's 224x448 front-over-down mosaic (aerialvla_codec.
          build_mosaic) of the capture's two JPEGs, decoded exactly as the
          server decodes them (decode_image_b64), stored lossless as
          images/<mosaic_sha256>.png - the pixels the processor will get;
  prompt  aerialvla_codec.build_prompt(phrase, object), byte for byte the
          server's: the phrase is what demo/vla_mission.DetectorHint sends
          for that box (vla_bridge's pinhole bearing, rotated by the turn
          between the box's frame and the capture, through VLABridge at
          --bearing-gain 1: computed by vla_mission.follow_hint_step, the
          rule DetectorHint itself calls, not re-written),
          the object words the run's --object ("a red car"; inert for
          AerialVLA, docs/FINDING-what-drives-aerialvla.md, carried so the
          prompt keeps its trained shape);
  label   the expert's motion over the next --horizon-s, in upstream's step
          units (aerialvla_codec docstring, "The numbers are a STEP"):
            dpsi = wrap(yaw(t+h) - yaw(t))                clipped to +-1.1 rad
            fwd  = (p(t+h) - p(t)) . e(yaw(t+h))          clipped to 0..5 m
                   (the displacement on the NEW heading: upstream turns,
                   then moves along the new heading)
            down = up(t) - up(t+h)                        clipped to -5..5 m
          poses interpolated from the loop's track (track.jsonl), yaw the
          short way round; quantized by upstream's rule (aerialvla_codec.
          step_bins: clip, then truncate into 99 bins) and written as
          upstream's text (aerialvla_codec.action_text: "FF DD YY",
          zero-padded, no EOS - the trainer appends the tokenizer's </s>),
          which the server's strict reader reads back as the same bins.

THE HORIZON AND THE CADENCE. The July campaign's run2 learned to turn and
overshot because the expert decided at 10 Hz and the model at 0.8 Hz
(training/aerialvla_tune_log.md); here the label is not the expert's
command but its MOTION over a horizon, so one label covers what one model
decision has to cover. --horizon-s should be the decision period of the VLA
flight that will use the model (about 0.8 s capture-to-answer for the merged
bf16 server, #9; a stop-and-go step longer); 1.0 s is the brief's default.
--stride-s spaces the samples (0.5 s: the 10 Hz captures are near-copies of
one another).

UPSTREAM'S ROTATE-ONLY RULE. Upstream's executor only rotates on a step
whose |dpsi| reaches 0.25 rad (aerialvla_codec.YAW_ONLY_THRESHOLD_RAD). A
follow turning 0.3 rad/s while driving 4 m/s makes exactly such labels with
fwd > 0: a step upstream would not fly as labelled. The follow's executor
(TrackerConfig turn_while_moving) turns while moving, so they are KEPT, and
stats.json counts them (rotate_only_band).

DART PERTURBATIONS (follow_vlm --record-perturb, 2026-10-23). The first
recording, ft_expert_01, gave 507 samples and all 507 said "straight
ahead ": the expert keeps the car centred, so nothing showed an off-centre
car being brought back. A perturbed recording knocks the executed command
off for 1.0-1.5 s every 5-8 s (demo/vla_record.Perturber) and marks each
tick in track.jsonl. Here a sample whose label horizon [t, t+h] overlaps a
perturbation - from its first perturbed tick to the first clean one, plus
--perturb-margin-s (0.3 s: the last perturbed command is flown for up to
0.3 s, move_by_velocity's duration, and the airframe answers with a lag) -
is dropped ("perturbed"): the label must be the expert's clean motion.
Samples that START after one are kept - the car off-centre, the expert
correcting is the point - each with `perturb.since_end_s`, and stats.json
counts those within 3 s (post_perturb_3s) and gives every run's phrase
counts and |dpsi| histogram, so the effect is visible run by run.

DAGGER (follow_vlm --record-perturb-vla, demo/vla_dagger.py, 2026-10-24).
AerialVLA itself flies windows of the recording; each track line's
`perturb` has source "vla" and `active` true exactly on the ticks whose
executed command was the VLA's (a window tick the VLA had no step for is
flown by the expert and is not active). The same rule applies, unchanged:
a label whose horizon overlaps a VLA-flown tick (plus the margin) is
dropped, and the expert's recovery after it is kept - the label is always
the expert's own motion, now from the states the model led it into.

MIRROR (--mirror). Each TRAIN sample also gives a mirrored one: the mosaic
flipped left to right (both tiles span the width, so each is flipped in
place - the world mirrored about the aircraft's long axis), dpsi negated
and re-quantized by the same rule, the phrase mirrored (MIRROR_PHRASE:
forward-left <-> forward-right, to your left <-> to your right, the rear
pair, straight ahead and "" unchanged), the box's centre and bearing
mirrored; marked `mirrored`, `mirror_of`. The hold-out split stays real.
The object words are not mirrored ("a red car" has no side).

THE LOWER TILE (--bottom-tile, 2026-10-24). "down" (the default) is the
mosaic above: the capture's front over its down frame. "prev_front" puts,
in the lower tile, the FRONT JPEG of the run's capture nearest t_capture -
--prev-dt-s (1.0 s) within --prev-tol-s (0.25 s) - aerialvla_codec.
pick_prev_front, the flight's own rule (vla_mission.VLAFrames) - so the
pair shows the car's motion: the first fine-tuned follow
(vla_follow_ft_01_20261024) could not tell a car stopped at a light from
one pulling away, and held 0.0 m for ~6 s while it went from 9.8 to 27.6
m. Every capture of the run whose front JPEG is on disk is a candidate,
the ones that are no sample themselves (no box, a stale box) included: the
flight's ring holds every frame. None within the tolerance: no sample
("no_prev_front", after frame_missing - the flight builds no capture then,
before any hint). The down JPEG is not needed. Each sample's `bottom_tile`
says the mode, dt and tol, and, for prev_front, the earlier capture (its
i, t_capture and front sha256) and the measured dt (dt_actual_s);
stats.json gives the mode and the measured dt's spread, per run and in
all. The trainer copies the mode into the checkpoint's provenance, and
the flight refuses an adapter whose tile is not its own. The mirror flips
the whole mosaic, so both tiles, each in place.

MEMORY AND SEARCH (--hint-modes, 2026-10-24). Fresh boxes only (the
default) leave out every state a follow is in once it has lost the car,
and a VLA follow keeps asking there: DetectorHint sends, with no fresh box,
the remembered sighting's phrase (its world bearing re-read against the
current heading) for memory_s, then a quarter turn toward the side it was
last seen on for search_s, then nothing. Flight vla_follow_ft_03_pf_20261024
(adapter follow_v3_pf_r1/epoch1) overshot a car stopped ahead, the car left
the bottom of the frame, the hint went to memory then search, and the model
- which had never seen a frame without the car - answered "forward 4.85 m"
for 30 s, away from it. --hint-modes fresh,memory,search runs the flight's
rule, vla_mission.follow_hint_step (DetectorHint calls the same function),
over each run's well-formed captures in time order, carrying the
remembered sighting from one to the next, with --memory-s / --search-s
(default vla_fly's --follow-memory-s 6 / --follow-search-s 20) and
--fresh-s: a capture whose hint is of a kind asked for is a sample, labelled
as every other (the expert's motion over the horizon - its own recovery,
and after a DAgger window the recovery from where the model led it; the
perturbed-window drop applies unchanged); one the flight would send nothing
for (no sighting yet, or past memory_s + search_s) is dropped "no_hint", one
of a kind left out "hint_kind". Each sample says its `hint_kind` and the
emulated state (`hint`: the box at the capture - fresh, stale or none - the
bearing the phrase was looked up with, and for memory / search the time
since the sighting and its bearing relative to the heading); stats.json
counts the kinds in all, per split and per run, and gives each kind's
phrases, box states, label spread and post-perturbation count (`hints`).
Not emulated: the flight asks once per decision (~1 Hz) and remembers the
sighting of its last request, the rule here runs on every capture (10 Hz),
so the sighting it remembers can be up to one decision period more recent;
the start gate (a follow with identity gates in the Reacquirer, before any
box is published); a perception reset's forget() (the expert has none).

THE CLOSE RULE (--close-bottom-frac / --close-width-frac). A close box
(vla_mission.BoxClose) is no request in flight - DetectorHint returns
nothing and the rail holds the horizontal motion - so it is no sample here
("close_box"); its sighting is still remembered, as in flight. The default
is BoxClose's own (0.92, 0.25); the follow flies --close-bottom-frac 1.0,
under which a box near (not at) the bottom edge is a request with its
phrase, so a dataset for it is built with the same values. stats.json
(`args`, `hints.close_rule`) and every sample (`close_rule`) record the rule;
--keep-close turns it off (recorded as None).

WHAT IS LEFT OUT, AND SAID. Captures with no box, a stale box (unless a
memory or search hint is asked for), or a CLOSE box (THE CLOSE RULE;
--keep-close keeps them); captures whose
horizon runs past the end of the track or across a gap in it (a loop stall
longer than --max-gap-s); captures whose JPEG FrameSaver dropped. Every drop
is counted by reason, per run and in total. LAND is never labelled: the
expert never arrives. The level car's position is copied into each sample's
`analysis` for diagnosis only; nothing here makes a label or a phrase from it.

No torch and no model. Pillow only when the mosaics are written.
"""
from __future__ import annotations

import argparse
import base64
import bisect
import datetime as _dt
import json
import math
import os
import statistics
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import aerialvla_codec as C                          # noqa: E402
from guardrail.vla_link import wrap_pi                                # noqa: E402

FORMAT = "aerialvla-follow-steps/1"
DEFAULT_HORIZON_S = 1.0
DEFAULT_STRIDE_S = 0.5
# DetectorHint's freshness (vla_fly --det-fresh-s, DETECTION_STALE_S of the
# ArduPilot rail): a box older than this at the capture is no request.
DEFAULT_FRESH_S = 1.0
# vla_fly's follow hint: --follow-memory-s, --follow-search-s (MEMORY AND
# SEARCH), and BoxClose's own close rule (--close-bottom-frac,
# --close-width-frac; the follow flies 1.0 / 0.25).
DEFAULT_MEMORY_S = 6.0
DEFAULT_SEARCH_S = 20.0
DEFAULT_CLOSE_BOTTOM_FRAC = 0.92
DEFAULT_CLOSE_WIDTH_FRAC = 0.25
HINT_MODES = ("fresh", "memory", "search")       # vla_mission.HINT_KINDS
DEFAULT_HINT_MODES = ("fresh",)
# A loop tick every 0.1 s; a stall longer than this between two ticks leaves
# no pose to interpolate (metrics cmd_interval shows such stalls).
DEFAULT_MAX_GAP_S = 0.5
# The follow rail's detector camera: FrontCamera, 90 deg (vla_fly builds its
# VLABridge with hfov_deg=90.0).
DETECTOR_HFOV_DEG = 90.0
HINT_SOURCE = "detector"
SAMPLES = "samples.jsonl"
STATS = "stats.json"
IMAGES = "images"
DROP_ORDER = ("bad_record", "frame_missing", "no_prev_front", "no_box", "stale_box",
              "close_box", "no_hint", "hint_kind", "mode", "no_pose", "no_future_pose",
              "track_gap", "perturbed", "stride")
SMALL_FWD_M = 1.0
# After the last perturbed tick's command, how long the motion is still not
# the expert's own (module docstring, DART PERTURBATIONS).
DEFAULT_PERTURB_MARGIN_S = 0.3
POST_PERTURB_S = 3.0
# |dpsi| bands for the per-run turn histogram (rad per label horizon).
DPSI_BANDS = {"0.00-0.05": (0.0, 0.05), "0.05-0.10": (0.05, 0.10), "0.10-0.20": (0.10, 0.20),
              "0.20-0.30": (0.20, 0.30), "0.30-0.50": (0.30, 0.50), "0.50-0.80": (0.50, 0.80),
              "0.80-1.10": (0.80, math.inf)}
# The compass vocabulary mirrored left to right (aerialvla_codec.PHRASES).
MIRROR_PHRASE = {C.STRAIGHT_AHEAD: C.STRAIGHT_AHEAD, C.NO_HINT: C.NO_HINT,
                 C.FORWARD_LEFT: C.FORWARD_RIGHT, C.FORWARD_RIGHT: C.FORWARD_LEFT,
                 C.LEFT: C.RIGHT, C.RIGHT: C.LEFT,
                 C.LEFT_REAR: C.RIGHT_REAR, C.RIGHT_REAR: C.LEFT_REAR}
assert set(MIRROR_PHRASE) == set(C.PHRASES) == set(MIRROR_PHRASE.values())
MIRROR_NOTE = "horizontal flip of the front-over-down mosaic"


def mirror_phrase(phrase: str) -> str:
    """The phrase for the mirrored world; refuses anything off-vocabulary."""
    if phrase not in MIRROR_PHRASE:
        raise ValueError(f"{phrase!r} is not one of the 8 vocabulary strings")
    return MIRROR_PHRASE[phrase]


def dpsi_band(v: float) -> str:
    a = abs(float(v))
    for name, (lo, hi) in DPSI_BANDS.items():
        if lo <= a < hi:
            return name
    return list(DPSI_BANDS)[-1]


def _dpsi_hist(samples) -> dict:
    c = Counter(dpsi_band(s["label"]["dpsi_rad"]) for s in samples)
    return {name: c[name] for name in DPSI_BANDS if c[name]}


class DatasetError(ValueError):
    """A dataset this tool refuses to build (main() exits 2)."""


# --------------------------------------------------------------------------- #
# The expert's motion
# --------------------------------------------------------------------------- #
class Track:
    """The loop's pose track (track.jsonl rows: t, x, y, up, yaw), linearly
    interpolated in time, yaw the short way round. `max_gap_s`: two ticks
    further apart than this have no pose between them (a stall)."""

    def __init__(self, rows, max_gap_s: float = DEFAULT_MAX_GAP_S):
        by_t: dict[float, dict] = {}
        active: dict[float, bool | None] = {}
        for r in rows:
            try:
                p = {k: float(r[k]) for k in ("t", "x", "y", "up", "yaw")}
            except (KeyError, TypeError, ValueError):
                continue
            if all(math.isfinite(v) for v in p.values()):
                by_t[p["t"]] = p                      # a repeated time keeps the last
                pr = r.get("perturb")
                active[p["t"]] = None if pr is None else bool(pr.get("active"))
        self.rows = [by_t[t] for t in sorted(by_t)]
        self.t = [r["t"] for r in self.rows]
        self.max_gap_s = float(max_gap_s)
        # --record-perturb: [t_on, t_off) of each perturbation, from the
        # first perturbed tick to the first clean one after it (the track's
        # end if the flight ended inside one).
        self.perturb_intervals: list[tuple[float, float]] = []
        on = None
        for t in self.t:
            if active[t] and on is None:
                on = t
            elif not active[t] and on is not None:
                self.perturb_intervals.append((on, t))
                on = None
        if on is not None:
            self.perturb_intervals.append((on, self.t[-1]))

    def perturbed(self, t: float, horizon_s: float, margin_s: float) -> bool:
        """Does the label horizon [t, t + horizon_s] overlap a perturbation,
        counting `margin_s` after its last perturbed command as perturbed?
        A horizon that ends exactly at an onset, or starts exactly at
        t_off + margin, does not (1e-6 s of slack for wall-clock floats)."""
        eps = 1e-6
        return any(t < off + margin_s - eps and t + horizon_s > on + eps
                   for on, off in self.perturb_intervals)

    def since_perturb_end(self, t: float) -> float | None:
        """Seconds from the end of the last perturbation that ended at or
        before t; None when none has."""
        ends = [off for _on, off in self.perturb_intervals if off <= t + 1e-6]
        return None if not ends else t - max(ends)

    def locate(self, t: float) -> tuple[dict | None, str | None]:
        """(pose, None) or (None, why): "no_pose" before the track,
        "no_future_pose" after it, "track_gap" inside a stall."""
        eps = 1e-9
        if not self.t or t < self.t[0] - eps:
            return None, "no_pose"
        if t > self.t[-1] + eps:
            return None, "no_future_pose"
        i = bisect.bisect_left(self.t, t)
        if i < len(self.t) and abs(self.t[i] - t) <= eps:
            return dict(self.rows[i]), None
        if i == 0:
            return dict(self.rows[0]), None
        if i >= len(self.t):
            return dict(self.rows[-1]), None
        a, b = self.rows[i - 1], self.rows[i]
        if b["t"] - a["t"] > self.max_gap_s:
            return None, "track_gap"
        f = (t - a["t"]) / (b["t"] - a["t"])
        out = {k: a[k] + f * (b[k] - a[k]) for k in ("x", "y", "up")}
        out["yaw"] = wrap_pi(a["yaw"] + f * wrap_pi(b["yaw"] - a["yaw"]))
        out["t"] = t
        return out, None

    def at(self, t: float) -> dict | None:
        return self.locate(t)[0]


def motion_label(p0: dict, p1: dict) -> dict:
    """The motion from pose p0 to pose p1 as an upstream step, unclipped:
    the heading change, the displacement on the NEW heading, the drop."""
    yaw1 = p1["yaw"]
    dx, dy = p1["x"] - p0["x"], p1["y"] - p0["y"]
    return {"fwd_m": dx * math.cos(yaw1) + dy * math.sin(yaw1),
            "down_m": p0["up"] - p1["up"],
            "dpsi_rad": wrap_pi(yaw1 - p0["yaw"])}


def clip_label(raw: dict) -> tuple[dict, dict]:
    """(the label clipped to upstream's ranges, which axis was clipped:
    "low" / "high" / None)."""
    lab, flags = {}, {}
    for axis in C.AXES:
        lo, hi = C.RANGES[axis]
        v = float(raw[axis])
        flags[axis] = "low" if v < lo else "high" if v > hi else None
        lab[axis] = min(max(v, lo), hi)
    return lab, flags


def label_at(track: Track, t: float, horizon_s: float) -> dict:
    """The expert's step from wall time t over horizon_s: {"reason": None,
    "raw", "label", "clipped", "bins", "p0", "p1"}, or {"reason": why}."""
    p0, why = track.locate(t)
    if p0 is None:
        return {"reason": why}
    p1, why = track.locate(t + horizon_s)
    if p1 is None:
        return {"reason": why}
    raw = motion_label(p0, p1)
    lab, flags = clip_label(raw)
    bins = C.step_bins(lab["fwd_m"], lab["down_m"], lab["dpsi_rad"])
    return {"reason": None, "raw": raw, "label": lab, "clipped": flags, "bins": bins,
            "p0": p0, "p1": p1}


# --------------------------------------------------------------------------- #
# The phrase: DetectorHint's, for the box the expert had
# --------------------------------------------------------------------------- #
def make_bridge(object_text: str = "a red car", bearing_gain: float = 1.0):
    """vla_fly's VLABridge for the follow: FrontCamera's 90 deg, gain 1."""
    from vla_bridge import VLABridge
    return VLABridge(object_text, hfov_deg=DETECTOR_HFOV_DEG, bearing_gain=bearing_gain)


def close_rule(bottom_frac: float = DEFAULT_CLOSE_BOTTOM_FRAC,
               width_frac: float = DEFAULT_CLOSE_WIDTH_FRAC):
    """vla_mission.BoxClose (THE CLOSE RULE): a VLA follow sends no request
    on such a box."""
    import vla_mission as VM
    return VM.BoxClose(float(bottom_frac), float(width_frac))


def default_close_rule():
    """vla_fly's follow close rule at BoxClose's defaults (--close-bottom-frac
    0.92, --close-width-frac 0.25): a VLA follow sends no request on such a
    box."""
    return close_rule(DEFAULT_CLOSE_BOTTOM_FRAC, DEFAULT_CLOSE_WIDTH_FRAC)


def parse_hint_modes(modes) -> tuple[str, ...]:
    """--hint-modes ("fresh,memory,search", or a sequence of the names) ->
    the kinds asked for, in HINT_MODES order; refuses an unknown kind or
    none at all."""
    if isinstance(modes, str):
        modes = [m for m in modes.replace(" ", ",").split(",") if m]
    got = [str(m) for m in (modes or ())]
    bad = sorted(set(got) - set(HINT_MODES))
    if bad or not got:
        raise DatasetError(f"--hint-modes {','.join(got) or '(none)'}: give one or more of "
                           f"{', '.join(HINT_MODES)}, comma-separated"
                           + (f" ({', '.join(bad)} is no hint kind)" if bad else ""))
    return tuple(m for m in HINT_MODES if m in got)


def hint_step(cap: dict, seen, bridge, *, fresh_s: float = DEFAULT_FRESH_S,
              memory_s: float = 0.0, search_s: float = 0.0, close=None):
    """vla_mission.follow_hint_step for a recorded capture: its box (the
    recorder's box_from_grounder, vla_fly's latest_box reading), its time
    and heading, the sighting remembered so far."""
    import vla_mission as VM
    box = cap.get("box") or {}
    return VM.follow_hint_step((box.get("det"), box.get("t_box"), box.get("yaw_box")),
                               float(cap["t_capture"]), float(cap["pose"]["yaw_rad"]), seen,
                               bridge, fresh_s=fresh_s, memory_s=memory_s, search_s=search_s,
                               close=close)


def hint_drop(step, modes, emulate: bool) -> str | None:
    """Why a capture whose hint is `step` is no sample, or None. A close box:
    "close_box". No request: with memory or search asked for, "no_hint";
    fresh boxes only (no emulation), the box's state - "no_box" /
    "stale_box", the reasons before --hint-modes. A kind not asked for:
    "hint_kind"."""
    if step.kind == "close":
        return "close_box"
    if step.kind is None:
        return "no_hint" if emulate else ("no_box" if step.box == "none" else "stale_box")
    return None if step.kind in modes else "hint_kind"


def detector_phrase(cap: dict, bridge, fresh_s: float = DEFAULT_FRESH_S, close=None):
    """(phrase, None, info) for a capture whose box a VLA follow would send,
    else (None, why, info), why one of "no_box", "stale_box", "close_box" -
    DetectorHint's order. The phrase is vla_mission.follow_hint_step's for
    this box and a capture at this pose and time with no sighting
    remembered, no memory, no search (a fresh box only) - the flight's
    arithmetic, not a copy of it. info: the box's age at the capture, its
    measured bearing and the yaw correction (degrees)."""
    st = hint_step(cap, None, bridge, fresh_s=fresh_s, close=close)
    info = {"age_s": st.age_s,
            "bearing_deg": (None if st.measured_bearing_rad is None
                            else math.degrees(st.measured_bearing_rad)),
            "yaw_correction_deg": (None if st.yaw_correction_rad is None
                                   else math.degrees(st.yaw_correction_rad))}
    why = hint_drop(st, ("fresh",), emulate=False)
    return (st.phrase if why is None else None), why, info


# --------------------------------------------------------------------------- #
# Recordings
# --------------------------------------------------------------------------- #
def find_recording(path: str | Path) -> Path:
    """The folder holding captures.jsonl: `path` itself or its vla_record/."""
    p = Path(path)
    for cand in (p, p / "vla_record"):
        if (cand / "captures.jsonl").is_file():
            return cand
    raise DatasetError(f"{p}: no captures.jsonl here or in {p / 'vla_record'} (a recording of "
                       f"follow_vlm --record-vla / run_citylife_follow.ps1 -RecordVla)")


def _jsonl(path: Path) -> tuple[list[dict], int]:
    rows, bad = [], 0
    if not path.is_file():
        return rows, 0
    with path.open("r", encoding="utf-8") as fh:
        for ln in fh:
            if not ln.strip():
                continue
            try:
                rows.append(json.loads(ln))
            except json.JSONDecodeError:
                bad += 1                    # a line cut short by a flight that died
    return rows, bad


def load_run(path: str | Path) -> dict:
    d = find_recording(path)
    record = {}
    if (d / "record.json").is_file():
        try:
            record = json.loads((d / "record.json").read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            record = {}
    caps, bad_c = _jsonl(d / "captures.jsonl")
    track, bad_t = _jsonl(d / "track.jsonl")
    tag = record.get("tag") or (d.parent.name if d.name == "vla_record" else d.name)
    return {"tag": str(tag), "dir": d, "record": record, "captures": caps, "track": track,
            "bad_lines": {"captures": bad_c, "track": bad_t}}


def _spread(vals) -> dict | None:
    vals = [float(v) for v in vals if v is not None]
    if not vals:
        return None
    s = sorted(vals)
    return {"n": len(s), "min": round(s[0], 4), "p50": round(statistics.median(s), 4),
            "p95": round(s[min(len(s) - 1, int(round(0.95 * (len(s) - 1))))], 4),
            "max": round(s[-1], 4)}


def _hist(bins) -> dict:
    c = Counter(bins)
    return {str(k): c[k] for k in sorted(c)}


def _round(v, nd: int):
    return None if v is None else round(float(v), nd)


def _deg(rad):
    """Radians -> degrees to 3 decimals, None kept."""
    return None if rad is None else round(math.degrees(float(rad)), 3)


def _kinds(samples) -> dict:
    """{hint kind: n} in HINT_MODES order, the kinds present only."""
    c = Counter(s.get("hint_kind") for s in samples)
    return {k: c[k] for k in HINT_MODES if c[k]}


def _hint_stats(samples, real, *, hint_modes, fresh_s, memory_s, search_s, close) -> dict:
    """stats.json's `hints` (MEMORY AND SEARCH): the rule's settings and,
    per kind, the samples (all, real, per split), and over the real ones
    their phrases, box states, label spread and |dpsi| histogram, and how
    many start within 3 s after a perturbation (DART / DAgger)."""
    by_kind = {k: [s for s in real if s.get("hint_kind") == k] for k in HINT_MODES}
    by_kind = {k: v for k, v in by_kind.items() if v}
    return {
        "modes": list(hint_modes), "fresh_s": fresh_s, "memory_s": memory_s,
        "search_s": search_s, "close_rule": None if close is None else close.as_dict(),
        "by_kind": _kinds(samples), "by_kind_real": _kinds(real),
        "by_split": {sp: _kinds(s for s in samples if s["split"] == sp)
                     for sp in ("train", "holdout")},
        "box_state": {k: dict(Counter(s["hint"]["box"] for s in v)) for k, v in by_kind.items()},
        "phrases_by_kind": {k: dict(Counter(s["phrase"] for s in v)) for k, v in by_kind.items()},
        "label_by_kind": {k: {"n": len(v),
                              **{ax: _spread(s["label"][ax] for s in v) for ax in C.AXES},
                              "abs_dpsi_hist": _dpsi_hist(v)} for k, v in by_kind.items()},
        "post_perturb_3s_by_kind": _kinds(
            s for s in real if s["perturb"]["since_end_s"] is not None
            and s["perturb"]["since_end_s"] <= POST_PERTURB_S + 1e-9),
        "rule": ("vla_mission.follow_hint_step - DetectorHint's decision - over each run's "
                 "well-formed captures in time order, the sighting carried from one to the "
                 "next; a capture is a sample when its hint's kind is in `modes`"),
        "not_emulated": ("the flight decides once per request (~1 Hz) and remembers the "
                         "sighting of its last request, this rule runs on every capture "
                         "(10 Hz): the sighting remembered here can be up to one decision "
                         "period more recent; no start gate, no perception reset"),
    }


def _write_png(img, path: Path) -> bool:
    """Write the mosaic losslessly unless it is there; True when written."""
    if path.exists():
        return False
    tmp = path.with_suffix(".tmp")
    img.save(tmp, format="PNG")
    os.replace(tmp, path)
    return True


def _mosaic(frames: Path, front_sha: str, lower_sha: str, lower_what: str = "down"):
    """The server's path: the JPEG bytes as base64, decoded by the codec,
    then the codec's mosaic (a missing lower frame refused, as the flight's
    server runs --missing-down refuse). `lower_sha`: the capture's down
    JPEG, or with --bottom-tile prev_front the earlier capture's front JPEG
    - what the flight sends in the down slot."""
    fb = (frames / f"{front_sha}.jpg").read_bytes()
    lb = (frames / f"{lower_sha}.jpg").read_bytes()
    front = C.decode_image_b64(base64.b64encode(fb).decode("ascii"), "front")
    lower = C.decode_image_b64(base64.b64encode(lb).decode("ascii"), lower_what)
    m, _info = C.build_mosaic(front, lower, missing_down="refuse")
    return m


def prev_front_candidates(caps, frames: Path) -> tuple[list[float], list[dict]]:
    """--bottom-tile prev_front: (capture times ascending, the captures) of
    every capture of a run with a finite t_capture and its front JPEG on
    disk - the frames the flight's ring would hold, samples or not."""
    got = []
    for c in caps:
        try:
            t = float(c["t_capture"])
        except (KeyError, TypeError, ValueError):
            continue
        h = c.get("front_jpg_sha256")
        if math.isfinite(t) and h and (frames / f"{h}.jpg").is_file():
            got.append((t, c))
    got.sort(key=lambda tc: tc[0])
    return [t for t, _c in got], [c for _t, c in got]


def mirror_sample(s: dict, lab: dict, image: str | None, sha: str | None) -> dict:
    """The left-right mirror of sample `s` (module docstring, MIRROR): the
    turn negated and re-quantized by upstream's rule from the unrounded
    label, forward and down unchanged, the phrase mirrored and the prompt
    rebuilt by the codec (a memory or search phrase too: the search's
    quarter turn is directional), the box centre and bearing and the hint's
    body-frame bearings mirrored, its kind kept. `image` / `sha`: the
    flipped mosaic's (None without images). The pose, the expert's command
    and the analysis are the source capture's."""
    flip = {"low": "high", "high": "low", None: None}
    mlab = dict(lab["label"], dpsi_rad=-lab["label"]["dpsi_rad"])
    mraw = dict(lab["raw"], dpsi_rad=-lab["raw"]["dpsi_rad"])
    bins = C.step_bins(mlab["fwd_m"], mlab["down_m"], mlab["dpsi_rad"])
    step = C.bins_to_step(bins)
    phrase = mirror_phrase(s["phrase"])
    prompt = C.build_prompt(phrase, s["object"])
    box = dict(s["box"])
    det = box.get("det")
    if det is not None:
        det = list(det)
        det[0] = float(det[5]) - float(det[0])
        box["det"] = det
    for k in ("bearing_deg", "yaw_correction_deg"):
        if box.get(k) is not None:
            box[k] = -box[k]
    hint = dict(s["hint"])
    for k in ("bearing_deg", "seen_rel_deg"):
        if hint.get(k) is not None:
            hint[k] = -hint[k]
    return {**s, "id": s["id"] + "/mirror", "image": image, "mosaic_sha256": sha,
            "phrase": phrase, "prompt": prompt, "prompt_sha256": C.prompt_sha256(prompt),
            "bins": list(bins), "action_text": C.action_text(bins),
            "label": {k: round(v, 6) for k, v in mlab.items()},
            "label_raw": {k: round(v, 6) for k, v in mraw.items()},
            "clipped": dict(s["clipped"], dpsi_rad=flip[s["clipped"]["dpsi_rad"]]),
            "step_decoded": {k: round(v, 6) for k, v in step.to_dict().items()},
            "rotate_only": step.rotate_only, "box": box, "hint": hint,
            "mirrored": True, "mirror_of": s["id"], "mirror": MIRROR_NOTE}


# --------------------------------------------------------------------------- #
# The dataset
# --------------------------------------------------------------------------- #
def build_dataset(run_dirs, out_dir, *, horizon_s: float = DEFAULT_HORIZON_S,
                  stride_s: float = DEFAULT_STRIDE_S, holdout=(), fresh_s: float = DEFAULT_FRESH_S,
                  max_gap_s: float = DEFAULT_MAX_GAP_S, keep_close: bool = False, modes=None,
                  bearing_gain: float = 1.0, images: bool = True, force: bool = False,
                  perturb_margin_s: float = DEFAULT_PERTURB_MARGIN_S, mirror: bool = False,
                  bottom_tile: str = C.BOTTOM_TILE_DOWN, prev_dt_s: float | None = None,
                  prev_tol_s: float | None = None, hint_modes=DEFAULT_HINT_MODES,
                  memory_s: float | None = None, search_s: float | None = None,
                  close_bottom_frac: float | None = None,
                  close_width_frac: float | None = None, log=None) -> dict:
    """Write samples.jsonl, stats.json (and images/) to out_dir; return the
    stats. Raises DatasetError for what main() refuses. `hint_modes`: the
    hint kinds kept (MEMORY AND SEARCH; "fresh,memory,search" or names);
    memory_s / search_s (None: vla_fly's 6 / 20) only with memory or
    search in it; the close fractions (None: BoxClose's 0.92 / 0.25) not
    with keep_close."""
    say = log or (lambda *_a, **_k: None)
    if not (horizon_s > 0 and stride_s >= 0 and fresh_s > 0 and max_gap_s > 0
            and perturb_margin_s >= 0):
        raise DatasetError("--horizon-s, --fresh-s and --max-gap-s must be positive and "
                           "--stride-s and --perturb-margin-s not negative")
    if bottom_tile != C.BOTTOM_TILE_PREV_FRONT and (prev_dt_s is not None
                                                    or prev_tol_s is not None):
        raise DatasetError("--prev-dt-s / --prev-tol-s are for --bottom-tile prev_front: with "
                           "'down' they would be recorded as if used")
    try:
        tile = C.bottom_tile(bottom_tile, prev_dt_s, prev_tol_s)
    except ValueError as exc:
        raise DatasetError(f"--bottom-tile / --prev-dt-s / --prev-tol-s: {exc}") from None
    hint_modes = parse_hint_modes(hint_modes)
    # The memory / search emulation runs only when one of them is asked for;
    # fresh boxes only is the rule before --hint-modes, drop reasons and all.
    emulate = bool(set(hint_modes) - {"fresh"})
    if not emulate and (memory_s is not None or search_s is not None):
        raise DatasetError("--memory-s / --search-s are for --hint-modes with memory or search: "
                           "with fresh boxes only they would be recorded as if used")
    if emulate:
        memory_s = DEFAULT_MEMORY_S if memory_s is None else float(memory_s)
        search_s = DEFAULT_SEARCH_S if search_s is None else float(search_s)
        if not (memory_s >= 0 and search_s >= 0
                and math.isfinite(memory_s) and math.isfinite(search_s)):
            raise DatasetError("--memory-s and --search-s must not be negative")
    if keep_close and (close_bottom_frac is not None or close_width_frac is not None):
        raise DatasetError("--close-bottom-frac / --close-width-frac set the close rule "
                           "--keep-close turns off: they would be recorded as if used")
    close = None
    if not keep_close:
        cb = DEFAULT_CLOSE_BOTTOM_FRAC if close_bottom_frac is None else float(close_bottom_frac)
        cw = DEFAULT_CLOSE_WIDTH_FRAC if close_width_frac is None else float(close_width_frac)
        if not (0 < cb <= 1 and 0 < cw <= 1):
            raise DatasetError("--close-bottom-frac and --close-width-frac must be in (0, 1] "
                               "(vla_fly's own check)")
        close = close_rule(cb, cw)
    out = Path(out_dir)
    if (out / SAMPLES).exists() and not force:
        raise DatasetError(f"{out / SAMPLES} exists; pass --force to rebuild it (images/ "
                           f"is content-addressed and kept)")
    runs = [load_run(p) for p in run_dirs]
    if not runs:
        raise DatasetError("no recording given")
    tags = Counter(r["tag"] for r in runs)
    dup = sorted(t for t, n in tags.items() if n > 1)
    if dup:
        raise DatasetError(f"two recordings carry the tag {', '.join(dup)}: the split is by "
                           f"run tag, so each run must be given once")
    holdout = [str(h) for h in (holdout or [])]
    unknown = sorted(set(holdout) - set(tags))
    if unknown:
        raise DatasetError(f"--holdout {', '.join(unknown)} names no recording given "
                           f"(runs: {', '.join(sorted(tags))}); a typo would put the run meant "
                           f"to be held out into train")
    if not set(tags) - set(holdout):
        raise DatasetError("every run is held out: no train split would be left")
    if images:
        try:
            import PIL.Image                                         # noqa: F401
        except ImportError:
            raise DatasetError("Pillow is needed to write the mosaics: run under the vla-real "
                               "environment, or pass --no-images for labels and stats only") \
                from None
    out.mkdir(parents=True, exist_ok=True)
    if images:
        (out / IMAGES).mkdir(exist_ok=True)
    modes = set(modes) if modes else None
    # the rule's memory and search spans (0: M2's rule, fresh boxes only)
    mem_s, srch_s = (memory_s, search_s) if emulate else (0.0, 0.0)

    samples: list[dict] = []
    dropped_all: Counter = Counter()
    run_stats: dict[str, dict] = {}
    n_written = n_reused = 0
    pose_dist, pose_dyaw = [], []
    tile_dts: list[float] = []             # prev_front: the measured dt of every real sample
    for run in runs:
        tag, frames = run["tag"], run["dir"] / "frames"
        split = "holdout" if tag in holdout else "train"
        track = Track(run["track"], max_gap_s=max_gap_s)
        caps = sorted((c for c in run["captures"] if isinstance(c, dict)),
                      key=lambda c: (c.get("t_capture") is None, c.get("t_capture") or 0.0))
        prev_t, prev_caps = (prev_front_candidates(caps, frames) if tile.prev_front
                             else ([], []))
        run_dts: list[float] = []
        objects = {c.get("object") or run["record"].get("object") for c in caps}
        objects.discard(None)
        for obj in objects:
            try:
                C.validate_object(obj)
            except C.CodecError as exc:
                raise DatasetError(f"run {tag}: its object words {obj!r} cannot go in a "
                                   f"prompt ({exc})") from None
        bridge = make_bridge(next(iter(objects), "a red car"), bearing_gain)
        dropped: Counter = Counter()
        n_run = 0
        last_t = None
        seen = None                        # the hint rule's sighting: none at a run's start
        for cap in caps:
            try:
                t = float(cap["t_capture"])
                pose = cap["pose"]
                for k in ("x", "y", "up", "yaw_rad"):
                    float(pose[k])
                obj = cap.get("object") or run["record"].get("object")
                if not obj:
                    raise KeyError("object")
            except (KeyError, TypeError, ValueError):
                dropped["bad_record"] += 1
                continue
            # The flight's hint rule on this capture, the sighting carried
            # from the run's previous captures (MEMORY AND SEARCH) - decided
            # for every well-formed capture, sample or not, as the flight's
            # hint follows the box stream; the drop it implies is counted
            # below, after the frames, in the order the reasons always had.
            hstep = hint_step(cap, seen, bridge, fresh_s=fresh_s, memory_s=mem_s,
                              search_s=srch_s, close=close)
            seen = hstep.seen
            hf, hd = cap.get("front_jpg_sha256"), cap.get("down_jpg_sha256")
            if tile.prev_front:
                # the lower tile: the run's front frame from dt earlier (THE
                # LOWER TILE); the down JPEG is not used
                if not (hf and (frames / f"{hf}.jpg").is_file()):
                    dropped["frame_missing"] += 1
                    continue
                j = C.pick_prev_front(t, prev_t, tile.dt_s, tile.tol_s)
                if j is None:
                    dropped["no_prev_front"] += 1
                    continue
                prev = prev_caps[j]
                lower_sha, lower_what = prev["front_jpg_sha256"], "prev_front"
                tile_rec = {**tile.as_dict(), "dt_actual_s": round(t - prev_t[j], 6),
                            "prev_i": prev.get("i"), "prev_t_capture": prev_t[j],
                            "prev_front_jpg_sha256": lower_sha}
            else:
                if not (hf and hd and (frames / f"{hf}.jpg").is_file()
                        and (frames / f"{hd}.jpg").is_file()):
                    dropped["frame_missing"] += 1
                    continue
                lower_sha, lower_what, tile_rec = hd, "down", tile.as_dict()
            why = hint_drop(hstep, hint_modes, emulate)
            if why is not None:
                dropped[why] += 1
                continue
            phrase = hstep.phrase
            tick = cap.get("tick") or {}
            if modes is not None and tick.get("mode") not in modes:
                dropped["mode"] += 1
                continue
            lab = label_at(track, t, horizon_s)
            if lab["reason"] is not None:
                dropped[lab["reason"]] += 1
                continue
            if track.perturbed(t, horizon_s, perturb_margin_s):
                # The label must be the expert's clean motion (DART).
                dropped["perturbed"] += 1
                continue
            if last_t is not None and t - last_t < stride_s - 1e-9:
                dropped["stride"] += 1
                continue
            last_t = t
            bins = lab["bins"]
            prompt = C.build_prompt(phrase, obj)
            step = C.bins_to_step(bins)
            p0 = lab["p0"]
            pose_dist.append(math.hypot(float(pose["x"]) - p0["x"], float(pose["y"]) - p0["y"]))
            pose_dyaw.append(abs(math.degrees(wrap_pi(float(pose["yaw_rad"]) - p0["yaw"]))))
            image = sha = None
            if tile.prev_front:
                run_dts.append(tile_rec["dt_actual_s"])
            if images:
                m = _mosaic(frames, hf, lower_sha, lower_what)
                sha = C.mosaic_sha256(m)
                image = f"{IMAGES}/{sha}.png"
                if _write_png(m, out / image):
                    n_written += 1
                else:
                    n_reused += 1
            truth = cap.get("truth")
            box = cap.get("box") or {}
            samples.append({
                "id": f"{tag}/{cap.get('i')}", "run": tag, "split": split, "i": cap.get("i"),
                "t_capture": t, "t_mission": cap.get("t_mission"),
                "pose": {k: float(pose[k]) for k in ("x", "y", "up", "yaw_rad")},
                "image": image, "mosaic_sha256": sha,
                "front_jpg_sha256": hf, "down_jpg_sha256": hd,
                # what the mosaic's lower tile is (THE LOWER TILE)
                "bottom_tile": tile_rec,
                "phrase": phrase, "hint_source": HINT_SOURCE, "object": obj,
                "prompt": prompt, "prompt_sha256": C.prompt_sha256(prompt),
                "bins": list(bins), "action_text": C.action_text(bins),
                "label": {k: round(v, 6) for k, v in lab["label"].items()},
                "label_raw": {k: round(v, 6) for k, v in lab["raw"].items()},
                "clipped": lab["clipped"],
                "step_decoded": {k: round(v, 6) for k, v in step.to_dict().items()},
                "rotate_only": step.rotate_only, "horizon_s": horizon_s,
                # the recorded box (a memory or search sample's: stale, or
                # none; its bearing only for a fresh one)
                "box": {"det": box.get("det"), "age_s": _round(hstep.age_s, 4),
                        "bearing_deg": _deg(hstep.measured_bearing_rad),
                        "yaw_correction_deg": _deg(hstep.yaw_correction_rad),
                        "seq": box.get("seq"), "tier": box.get("tier")},
                # the hint the flight would send here (MEMORY AND SEARCH)
                "hint_kind": hstep.kind,
                "hint": {"kind": hstep.kind, "box": hstep.box,
                         "bearing_deg": _deg(hstep.bearing_rad),
                         "since_seen_s": _round(hstep.since_seen_s, 4),
                         "seen_rel_deg": _deg(hstep.seen_rel_rad)},
                "close_rule": None if close is None else close.as_dict(),
                "expert": {"mode": tick.get("mode"), "author": tick.get("author"),
                           "cmd": tick.get("cmd"), "asked": tick.get("asked"),
                           "touched": tick.get("touched")},
                # Diagnosis only (the level car): never a label or a phrase.
                "analysis": {"truth_xy": truth,
                             "subject_dist_m": (None if not truth else round(math.hypot(
                                 float(truth[0]) - float(pose["x"]),
                                 float(truth[1]) - float(pose["y"])), 3))},
                "perturb": {"since_end_s": (None if (se := track.since_perturb_end(t)) is None
                                            else round(se, 4))},
                "mirrored": False,
            })
            n_run += 1
            if mirror and split == "train":
                mimage = msha = None
                if images:
                    from PIL import ImageOps
                    mm = ImageOps.mirror(m)
                    msha = C.mosaic_sha256(mm)
                    mimage = f"{IMAGES}/{msha}.png"
                    if _write_png(mm, out / mimage):
                        n_written += 1
                    else:
                        n_reused += 1
                samples.append(mirror_sample(samples[-1], lab, mimage, msha))
        run_real = [s for s in samples if s["run"] == tag and not s["mirrored"]]
        post = sum(1 for s in run_real if s["perturb"]["since_end_s"] is not None
                   and s["perturb"]["since_end_s"] <= POST_PERTURB_S + 1e-9)
        dropped_all.update(dropped)
        tile_dts.extend(run_dts)
        rc = (run["record"].get("counts") or {})
        run_stats[tag] = {"dir": str(run["dir"]), "split": split,
                          "object": sorted(objects), "captures": len(caps),
                          "track_ticks": len(track.rows), "samples": n_run,
                          "dropped": {k: dropped[k] for k in DROP_ORDER if dropped[k]},
                          # What the run teaches, at a glance (real samples).
                          "phrase_counts": dict(Counter(s["phrase"] for s in run_real)),
                          "hint_kinds": _kinds(run_real),
                          "abs_dpsi_hist": _dpsi_hist(run_real),
                          "perturb": {"intervals": len(track.perturb_intervals),
                                      "active_s": round(sum(off - on for on, off
                                                            in track.perturb_intervals), 3),
                                      "dropped": dropped["perturbed"],
                                      "post_perturb_3s": post},
                          "bad_lines": run["bad_lines"],
                          # prev_front: how much earlier each real sample's
                          # lower tile really was (None with down)
                          "bottom_tile_dt_actual_s": (_spread(run_dts) if tile.prev_front
                                                      else None),
                          "recorder": {k: rc.get(k) for k in (
                              "captures", "skipped_front_frames", "capture_failed",
                              "capture_hz", "ticks_dropped", "saver") if k in rc},
                          "has_record_json": bool(run["record"])}
        say(f"[run] {tag:28s} {split:7s} captures {len(caps):5d} -> samples {n_run:5d}  "
            f"dropped {dict(run_stats[tag]['dropped'])}")
        say(f"[run] {'':28s} phrases {run_stats[tag]['phrase_counts']}  |dpsi| "
            f"{run_stats[tag]['abs_dpsi_hist']}  perturbations "
            f"{len(track.perturb_intervals)}, samples after one (<= 3 s) {post}")
        if emulate:
            say(f"[run] {'':28s} hint kinds {run_stats[tag]['hint_kinds']}")

    with (out / SAMPLES).open("w", encoding="utf-8", newline="\n") as fh:
        for s in samples:
            fh.write(json.dumps(s, ensure_ascii=False) + "\n")
    stats = _stats(samples, run_stats, dropped_all, holdout=holdout, horizon_s=horizon_s,
                   stride_s=stride_s, fresh_s=fresh_s, max_gap_s=max_gap_s,
                   keep_close=keep_close, modes=modes, bearing_gain=bearing_gain,
                   images=images, n_written=n_written, n_reused=n_reused,
                   pose_dist=pose_dist, pose_dyaw=pose_dyaw, run_dirs=run_dirs,
                   perturb_margin_s=perturb_margin_s, mirror=mirror, tile=tile,
                   tile_dts=tile_dts, hint_modes=hint_modes,
                   memory_s=memory_s if emulate else None,
                   search_s=search_s if emulate else None, close=close)
    (out / STATS).write_text(json.dumps(stats, indent=1, ensure_ascii=False), encoding="utf-8",
                             newline="\n")
    return stats


def _stats(samples, run_stats, dropped_all, *, holdout, horizon_s, stride_s, fresh_s,
           max_gap_s, keep_close, modes, bearing_gain, images, n_written, n_reused,
           pose_dist, pose_dyaw, run_dirs, perturb_margin_s, mirror,
           tile: "C.BottomTile | None" = None, tile_dts=(), hint_modes=DEFAULT_HINT_MODES,
           memory_s=None, search_s=None, close=None) -> dict:
    tile = tile if tile is not None else C.BottomTile()
    emulate = bool(set(hint_modes) - {"fresh"})
    # What the trainer sees (mirrored samples included): the splits, the
    # label histograms, the phrases. The quality figures below (rotate-only
    # band, small steps, clipping, distances) are over the REAL samples: a
    # mirror would only count each one twice.
    real = [s for s in samples if not s["mirrored"]]
    splits = Counter(s["split"] for s in samples)
    hist, dhist = {}, {}
    for name, sel in (("train", "train"), ("holdout", "holdout"), ("all", None)):
        ss = [s for s in samples if sel is None or s["split"] == sel]
        hist[name] = {axis: _hist(s["bins"][i] for s in ss) for i, axis in enumerate(C.AXES)}
        dhist[name] = _dpsi_hist(ss)
    band = [s for s in real if s["rotate_only"]]
    with_fwd = [s for s in band if s["bins"][0] > 0]
    clipped = {axis: {"low": sum(1 for s in real if s["clipped"][axis] == "low"),
                      "high": sum(1 for s in real if s["clipped"][axis] == "high")}
               for axis in C.AXES}
    mirrored = [s for s in samples if s["mirrored"]]
    return {
        "format": FORMAT, "tool": "tools/build_follow_dataset.py",
        "built_utc": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds"),
        "args": {"runs": [str(p) for p in run_dirs], "horizon_s": horizon_s,
                 "stride_s": stride_s, "holdout": list(holdout), "fresh_s": fresh_s,
                 "max_gap_s": max_gap_s, "keep_close": keep_close,
                 "modes": sorted(modes) if modes else None, "bearing_gain": bearing_gain,
                 "images": images, "perturb_margin_s": perturb_margin_s, "mirror": mirror,
                 "bottom_tile": tile.mode, "prev_dt_s": tile.dt_s, "prev_tol_s": tile.tol_s,
                 "hint_modes": list(hint_modes), "memory_s": memory_s, "search_s": search_s,
                 "close_bottom_frac": None if close is None else close.bottom_frac,
                 "close_width_frac": None if close is None else close.width_frac},
        "codec": {"num_bins": C.NUM_BINS, "ranges": {k: list(v) for k, v in C.RANGES.items()},
                  "quantize": "clip, then truncate into 99 bins (upstream _quantize_action; "
                              "aerialvla_codec.step_bins)",
                  "text": "aerialvla_codec.action_text: 'FF DD YY', zero-padded, no EOS - the "
                          "trainer appends the tokenizer's </s>",
                  "prompt_template": C.PROMPT_TEMPLATE,
                  "mosaic": ("aerialvla_codec.build_mosaic of the decoded JPEGs (the capture's "
                             f"front over the run's front frame {tile.dt_s:g} s earlier, "
                             "224x448), lossless PNG" if tile.prev_front else
                             "aerialvla_codec.build_mosaic of the decoded JPEGs (front over "
                             "down, 224x448), lossless PNG")},
        # The mosaic's lower tile (THE LOWER TILE): what the trainer copies
        # into the checkpoint's provenance, and the flight must match.
        "bottom_tile": {**tile.as_dict(),
                        "dt_actual_s": _spread(tile_dts) if tile.prev_front else None,
                        "no_prev_front": dropped_all["no_prev_front"],
                        "rule": ("the front frame of the run's capture nearest t_capture - "
                                 "dt_s within tol_s (aerialvla_codec.pick_prev_front); none: no "
                                 "sample" if tile.prev_front else
                                 "the capture's own down frame (upstream's)")},
        "splits": {"train": splits.get("train", 0), "holdout": splits.get("holdout", 0),
                   "train_real": sum(1 for s in real if s["split"] == "train")},
        "train_runs": sorted(t for t, r in run_stats.items() if r["split"] == "train"),
        "holdout_runs": sorted(t for t, r in run_stats.items() if r["split"] == "holdout"),
        "runs": run_stats,
        "dropped": {k: dropped_all[k] for k in DROP_ORDER if dropped_all[k]},
        "label_hist": hist,
        "abs_dpsi_hist": dhist,
        "label_spread": {axis: _spread(s["label"][axis] for s in real) for axis in C.AXES},
        "phrase_counts": dict(Counter(s["phrase"] for s in samples)),
        "phrase_counts_real": dict(Counter(s["phrase"] for s in real)),
        # the hint kinds (MEMORY AND SEARCH) and the close rule
        "hints": _hint_stats(samples, real, hint_modes=hint_modes, fresh_s=fresh_s,
                             memory_s=memory_s, search_s=search_s, close=close),
        "mirror": {"enabled": bool(mirror), "samples": len(mirrored),
                   "splits": ["train"] if mirror else []},
        "perturb": {
            "margin_s": perturb_margin_s,
            "intervals": sum(r["perturb"]["intervals"] for r in run_stats.values()),
            "dropped": dropped_all["perturbed"],
            "post_perturb_3s": sum(r["perturb"]["post_perturb_3s"] for r in run_stats.values()),
            "note": ("a sample whose label horizon overlaps a perturbation (its first perturbed "
                     "tick to the first clean one, plus margin_s) is dropped: the label is the "
                     "expert's clean motion; samples starting after one are kept "
                     "(post_perturb_3s: within 3 s of its end)")},
        "by_mode": dict(Counter(s["expert"]["mode"] for s in real)),
        "shield_touched": sum(1 for s in real if s["expert"].get("touched")),
        "rotate_only_band": {
            "n": len(band), "with_fwd": len(with_fwd),
            "frac": round(len(band) / len(real), 4) if real else None,
            "note": ("labels whose decoded |dpsi| reaches 0.25 rad: upstream's executor only "
                     "rotates on such a step, so with fwd > 0 they teach a step upstream would "
                     "not fly as labelled. Kept: the follow's executor (TrackerConfig "
                     "turn_while_moving) turns while moving.")},
        "small_fwd": {"lt_1m": sum(1 for s in real if s["label"]["fwd_m"] < SMALL_FWD_M),
                      "zero_bin": sum(1 for s in real if s["bins"][0] == 0),
                      "note": "the expert standing off close: kept"},
        "clipped": clipped,
        "subject_dist_m": _spread(s["analysis"]["subject_dist_m"] for s in real),
        # the fresh boxes' (a memory sample's stale box is no hint's age)
        "box_age_s": _spread(s["box"]["age_s"] for s in real if s["hint_kind"] == "fresh"),
        "pose_check": {"capture_vs_track_m": _spread(pose_dist),
                       "capture_vs_track_yaw_deg": _spread(pose_dyaw),
                       "note": "the capture's pose (the loop's pose ring at the frame's sim "
                               "stamp) against the track interpolated at its wall time; the "
                               "label uses the track at both ends"},
        "images": ({"dir": IMAGES, "written": n_written, "reused": n_reused}
                   if images else None),
        "notes": [
            (f"Hint kinds {', '.join(hint_modes)}: each capture's hint is the flight's rule "
             f"(vla_mission.follow_hint_step) carried over the run's captures in order, "
             f"memory_s {memory_s:g}, search_s {search_s:g}; see hints.not_emulated."
             if emulate else
             "Fresh-box captures only: the follow's memory and search hints are not represented."),
            "LAND is never labelled: the expert never arrives.",
            "The phrase is computed by demo/vla_mission.follow_hint_step (DetectorHint's rule) "
            "from the recorded box.",
            "analysis.truth_xy / subject_dist_m (the level car) are for diagnosis only.",
        ],
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip(),
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", metavar="RUN_DIR",
                    help="a flight folder (demo/out/<tag>) or its vla_record/ folder")
    ap.add_argument("--out", required=True, metavar="DATASET")
    ap.add_argument("--horizon-s", type=float, default=DEFAULT_HORIZON_S,
                    help="the label is the expert's motion over this many seconds after the "
                         "capture (default %(default)s; match the VLA flight's decision period)")
    ap.add_argument("--stride-s", type=float, default=DEFAULT_STRIDE_S,
                    help="at least this long between two samples of a run (default %(default)s)")
    ap.add_argument("--holdout", action="append", nargs="+", default=None, metavar="RUN_TAG",
                    help="runs (by tag) for the hold-out split, never in train")
    ap.add_argument("--fresh-s", type=float, default=DEFAULT_FRESH_S,
                    help="a box older than this at the capture is no sample (DetectorHint's "
                         "rule; default %(default)s)")
    ap.add_argument("--max-gap-s", type=float, default=DEFAULT_MAX_GAP_S,
                    help="a track gap longer than this has no pose (default %(default)s)")
    ap.add_argument("--keep-close", action="store_true",
                    help="keep captures whose box counts as close (a VLA follow sends no "
                         "request on them)")
    ap.add_argument("--close-bottom-frac", type=float, default=None,
                    help=f"the close rule: a box whose bottom edge is this low (fraction of "
                         f"the image height) is no request (default "
                         f"{DEFAULT_CLOSE_BOTTOM_FRAC:g}; the follow flies 1.0)")
    ap.add_argument("--close-width-frac", type=float, default=None,
                    help=f"the close rule: a box at least this wide is no request (default "
                         f"{DEFAULT_CLOSE_WIDTH_FRAC:g})")
    ap.add_argument("--hint-modes", default=",".join(DEFAULT_HINT_MODES),
                    help="the hint kinds kept, comma-separated: fresh, memory, search "
                         "(module docstring, MEMORY AND SEARCH; default %(default)s)")
    ap.add_argument("--memory-s", type=float, default=None,
                    help=f"with memory or search: the hint's memory span (default "
                         f"{DEFAULT_MEMORY_S:g}, vla_fly's --follow-memory-s)")
    ap.add_argument("--search-s", type=float, default=None,
                    help=f"with memory or search: the hint's search span (default "
                         f"{DEFAULT_SEARCH_S:g}, vla_fly's --follow-search-s)")
    ap.add_argument("--modes", nargs="+", default=None,
                    help="keep only captures the expert flew in these modes (e.g. track)")
    ap.add_argument("--bearing-gain", type=float, default=1.0,
                    help="VLABridge gain (1.0 honest; recorded)")
    ap.add_argument("--perturb-margin-s", type=float, default=DEFAULT_PERTURB_MARGIN_S,
                    help="a perturbed recording: after a perturbation's last perturbed tick, "
                         "this long still counts as perturbed for the label rule (default "
                         "%(default)s; 0 = the bare overlap rule)")
    ap.add_argument("--mirror", action="store_true",
                    help="add a left-right mirrored copy of every TRAIN sample (the mosaic "
                         "flipped, dpsi negated, left/right phrases swapped), marked mirrored")
    ap.add_argument("--no-images", action="store_true",
                    help="labels and stats only: no mosaics, no Pillow")
    ap.add_argument("--bottom-tile", choices=C.BOTTOM_TILE_MODES, default=C.BOTTOM_TILE_DOWN,
                    help="the mosaic's lower tile: 'down' (the capture's down frame, "
                         "upstream's) or 'prev_front' (the run's front frame --prev-dt-s "
                         "earlier; module docstring, THE LOWER TILE). Default %(default)s")
    ap.add_argument("--prev-dt-s", type=float, default=None,
                    help=f"prev_front: how many seconds earlier (default "
                         f"{C.DEFAULT_PREV_DT_S:g})")
    ap.add_argument("--prev-tol-s", type=float, default=None,
                    help=f"prev_front: the earlier frame within this of t - dt, else no sample "
                         f"(default {C.DEFAULT_PREV_TOL_S:g})")
    ap.add_argument("--force", action="store_true", help="rebuild an existing dataset")
    a = ap.parse_args(argv)
    holdout = [t for grp in (a.holdout or []) for t in grp]
    try:
        st = build_dataset(a.runs, a.out, horizon_s=a.horizon_s, stride_s=a.stride_s,
                           holdout=holdout, fresh_s=a.fresh_s, max_gap_s=a.max_gap_s,
                           keep_close=a.keep_close, modes=a.modes, bearing_gain=a.bearing_gain,
                           images=not a.no_images, force=a.force,
                           perturb_margin_s=a.perturb_margin_s, mirror=a.mirror,
                           bottom_tile=a.bottom_tile, prev_dt_s=a.prev_dt_s,
                           prev_tol_s=a.prev_tol_s, hint_modes=a.hint_modes,
                           memory_s=a.memory_s, search_s=a.search_s,
                           close_bottom_frac=a.close_bottom_frac,
                           close_width_frac=a.close_width_frac, log=print)
    except DatasetError as exc:
        print(f"[dataset] *** refused: {exc}", file=sys.stderr)
        return 2
    band = st["rotate_only_band"]
    print(f"[dataset] {st['splits']['train']} train ({st['splits']['train_real']} real, "
          f"{st['mirror']['samples']} mirrored) / {st['splits']['holdout']} hold-out "
          f"samples -> {a.out}")
    bt = st["bottom_tile"]
    if bt["mode"] == C.BOTTOM_TILE_PREV_FRONT:
        sp = bt["dt_actual_s"] or {}
        print(f"[dataset] lower tile prev_front: the front frame {bt['dt_s']:g} s earlier "
              f"(within {bt['tol_s']:g} s); measured dt p50 {sp.get('p50')} s, min "
              f"{sp.get('min')}, max {sp.get('max')}; no earlier frame {bt['no_prev_front']}")
    else:
        print("[dataset] lower tile down (the capture's down frame)")
    hs = st["hints"]
    print(f"[dataset] hint kinds {','.join(hs['modes'])} (memory_s {hs['memory_s']}, search_s "
          f"{hs['search_s']}), close rule {hs['close_rule']}: real samples by kind "
          f"{hs['by_kind_real']}")
    print(f"[dataset] dropped: {st['dropped']}")
    print(f"[dataset] phrases: {st['phrase_counts']}")
    print(f"[dataset] |dpsi| (train): {st['abs_dpsi_hist']['train']}")
    print(f"[dataset] perturbations: {st['perturb']['intervals']}, labels dropped for one "
          f"{st['perturb']['dropped']}, samples starting within 3 s after one "
          f"{st['perturb']['post_perturb_3s']}")
    print(f"[dataset] rotate-only band (|dpsi| >= 0.25 rad): {band['n']}, of which with "
          f"fwd > 0: {band['with_fwd']} (kept; see stats.json)")
    print(f"[dataset] small steps (fwd < 1 m): {st['small_fwd']['lt_1m']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
