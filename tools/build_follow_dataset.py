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

Each RUN_DIR is a flight folder (demo/out/<tag>, holding vla_record/) or the
recording folder itself (the one holding captures.jsonl). Exit 0 when the
dataset was written, 2 when it was refused (an unknown --holdout tag, two
recordings with one tag, no training run left, an existing dataset without
--force, Pillow missing without --no-images).

WHAT A SAMPLE IS (wave FT-D, 2026-10-23; the brief's section FT-D)

For every recorded capture with a FRESH box - the expert's box no older than
--fresh-s (1 s) at the capture, DetectorHint's rule, the only kind of hint a
VLA follow sends on a live sighting - one sample:

  image   the codec's 224x448 front-over-down mosaic (aerialvla_codec.
          build_mosaic) of the capture's two JPEGs, decoded exactly as the
          server decodes them (decode_image_b64), stored lossless as
          images/<mosaic_sha256>.png - the pixels the processor will get;
  prompt  aerialvla_codec.build_prompt(phrase, object), byte for byte the
          server's: the phrase is what demo/vla_mission.DetectorHint sends
          for that box (vla_bridge's pinhole bearing, rotated by the turn
          between the box's frame and the capture, through VLABridge at
          --bearing-gain 1: computed BY DetectorHint here, not re-written),
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

WHAT IS LEFT OUT, AND SAID. Captures with no box, a stale box, or a CLOSE
box (vla_mission.BoxClose at vla_fly's follow defaults: a VLA follow asks
nothing then; --keep-close keeps them) - so the follow's memory and search
hints (DetectorHint memory_s / search_s) are not represented; captures whose
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
from guardrail.vla_link import Capture, Pose, wrap_pi                 # noqa: E402

FORMAT = "aerialvla-follow-steps/1"
DEFAULT_HORIZON_S = 1.0
DEFAULT_STRIDE_S = 0.5
# DetectorHint's freshness (vla_fly --det-fresh-s, DETECTION_STALE_S of the
# ArduPilot rail): a box older than this at the capture is no request.
DEFAULT_FRESH_S = 1.0
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
DROP_ORDER = ("bad_record", "frame_missing", "no_box", "stale_box", "close_box", "mode",
              "no_pose", "no_future_pose", "track_gap", "stride")
SMALL_FWD_M = 1.0


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
        for r in rows:
            try:
                p = {k: float(r[k]) for k in ("t", "x", "y", "up", "yaw")}
            except (KeyError, TypeError, ValueError):
                continue
            if all(math.isfinite(v) for v in p.values()):
                by_t[p["t"]] = p                      # a repeated time keeps the last
        self.rows = [by_t[t] for t in sorted(by_t)]
        self.t = [r["t"] for r in self.rows]
        self.max_gap_s = float(max_gap_s)

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


def default_close_rule():
    """vla_fly's follow close rule (--close-bottom-frac 0.92,
    --close-width-frac 0.25): a VLA follow sends no request on such a box."""
    import vla_mission as VM
    return VM.BoxClose(0.92, 0.25)


def detector_phrase(cap: dict, bridge, fresh_s: float = DEFAULT_FRESH_S, close=None):
    """(phrase, None, info) for a capture whose box a VLA follow would send,
    else (None, why, info), why one of "no_box", "stale_box", "close_box" -
    DetectorHint's order. The phrase itself comes from a DetectorHint fed
    this box and a Capture at this pose and time (no memory, no search, no
    start gate: a fresh box only), so it is the flight's arithmetic, not a
    copy of it. info: the box's age at the capture, its measured bearing and
    the yaw correction (degrees)."""
    import vla_mission as VM
    from vla_bridge import bearing_from_box
    box = cap.get("box") or {}
    det, t_box, yaw_box = box.get("det"), box.get("t_box"), box.get("yaw_box")
    info = {"age_s": None, "bearing_deg": None, "yaw_correction_deg": None}
    if det is None or t_box is None:
        return None, "no_box", info
    t_cap = float(cap["t_capture"])
    age = t_cap - float(t_box)
    info["age_s"] = age
    if age > fresh_s:
        return None, "stale_box", info
    pose = cap["pose"]
    info["bearing_deg"] = math.degrees(bearing_from_box(det, bridge.hfov_deg))
    if yaw_box is not None:
        info["yaw_correction_deg"] = math.degrees(wrap_pi(float(pose["yaw_rad"]) - float(yaw_box)))
    if close is not None and close(det):
        return None, "close_box", info
    hint = VM.DetectorHint(lambda: (det, t_box, yaw_box, box.get("t_published")), bridge,
                           fresh_s=fresh_s, close=close)
    h = hint(Capture(t_cap, Pose(float(pose["x"]), float(pose["y"]), float(pose["up"]),
                                 float(pose["yaw_rad"])), b"\xff\xd8", None))
    if h is None:                          # cannot happen: the checks above are its own
        raise AssertionError(f"DetectorHint sent nothing for a fresh box: {box!r}")
    return h.phrase, None, info


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


def _write_png(img, path: Path) -> bool:
    """Write the mosaic losslessly unless it is there; True when written."""
    if path.exists():
        return False
    tmp = path.with_suffix(".tmp")
    img.save(tmp, format="PNG")
    os.replace(tmp, path)
    return True


def _mosaic(frames: Path, cap: dict):
    """The server's path: the JPEG bytes as base64, decoded by the codec,
    then the codec's mosaic (a missing down frame refused, as the flight's
    server runs --missing-down refuse)."""
    fb = (frames / f"{cap['front_jpg_sha256']}.jpg").read_bytes()
    db = (frames / f"{cap['down_jpg_sha256']}.jpg").read_bytes()
    front = C.decode_image_b64(base64.b64encode(fb).decode("ascii"), "front")
    down = C.decode_image_b64(base64.b64encode(db).decode("ascii"), "down")
    m, _info = C.build_mosaic(front, down, missing_down="refuse")
    return m


# --------------------------------------------------------------------------- #
# The dataset
# --------------------------------------------------------------------------- #
def build_dataset(run_dirs, out_dir, *, horizon_s: float = DEFAULT_HORIZON_S,
                  stride_s: float = DEFAULT_STRIDE_S, holdout=(), fresh_s: float = DEFAULT_FRESH_S,
                  max_gap_s: float = DEFAULT_MAX_GAP_S, keep_close: bool = False, modes=None,
                  bearing_gain: float = 1.0, images: bool = True, force: bool = False,
                  log=None) -> dict:
    """Write samples.jsonl, stats.json (and images/) to out_dir; return the
    stats. Raises DatasetError for what main() refuses."""
    say = log or (lambda *_a, **_k: None)
    if not (horizon_s > 0 and stride_s >= 0 and fresh_s > 0 and max_gap_s > 0):
        raise DatasetError("--horizon-s, --fresh-s and --max-gap-s must be positive and "
                           "--stride-s not negative")
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
    close = None if keep_close else default_close_rule()
    modes = set(modes) if modes else None

    samples: list[dict] = []
    dropped_all: Counter = Counter()
    run_stats: dict[str, dict] = {}
    n_written = n_reused = 0
    pose_dist, pose_dyaw = [], []
    for run in runs:
        tag, frames = run["tag"], run["dir"] / "frames"
        split = "holdout" if tag in holdout else "train"
        track = Track(run["track"], max_gap_s=max_gap_s)
        caps = sorted((c for c in run["captures"] if isinstance(c, dict)),
                      key=lambda c: (c.get("t_capture") is None, c.get("t_capture") or 0.0))
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
            hf, hd = cap.get("front_jpg_sha256"), cap.get("down_jpg_sha256")
            if not (hf and hd and (frames / f"{hf}.jpg").is_file()
                    and (frames / f"{hd}.jpg").is_file()):
                dropped["frame_missing"] += 1
                continue
            phrase, why, info = detector_phrase(cap, bridge, fresh_s=fresh_s, close=close)
            if why is not None:
                dropped[why] += 1
                continue
            tick = cap.get("tick") or {}
            if modes is not None and tick.get("mode") not in modes:
                dropped["mode"] += 1
                continue
            lab = label_at(track, t, horizon_s)
            if lab["reason"] is not None:
                dropped[lab["reason"]] += 1
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
            if images:
                m = _mosaic(frames, cap)
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
                "phrase": phrase, "hint_source": HINT_SOURCE, "object": obj,
                "prompt": prompt, "prompt_sha256": C.prompt_sha256(prompt),
                "bins": list(bins), "action_text": C.action_text(bins),
                "label": {k: round(v, 6) for k, v in lab["label"].items()},
                "label_raw": {k: round(v, 6) for k, v in lab["raw"].items()},
                "clipped": lab["clipped"],
                "step_decoded": {k: round(v, 6) for k, v in step.to_dict().items()},
                "rotate_only": step.rotate_only, "horizon_s": horizon_s,
                "box": {"det": box.get("det"), "age_s": round(info["age_s"], 4),
                        "bearing_deg": round(info["bearing_deg"], 3),
                        "yaw_correction_deg": (None if info["yaw_correction_deg"] is None
                                               else round(info["yaw_correction_deg"], 3)),
                        "seq": box.get("seq"), "tier": box.get("tier")},
                "expert": {"mode": tick.get("mode"), "author": tick.get("author"),
                           "cmd": tick.get("cmd"), "asked": tick.get("asked"),
                           "touched": tick.get("touched")},
                # Diagnosis only (the level car): never a label or a phrase.
                "analysis": {"truth_xy": truth,
                             "subject_dist_m": (None if not truth else round(math.hypot(
                                 float(truth[0]) - float(pose["x"]),
                                 float(truth[1]) - float(pose["y"])), 3))},
            })
            n_run += 1
        dropped_all.update(dropped)
        rc = (run["record"].get("counts") or {})
        run_stats[tag] = {"dir": str(run["dir"]), "split": split,
                          "object": sorted(objects), "captures": len(caps),
                          "track_ticks": len(track.rows), "samples": n_run,
                          "dropped": {k: dropped[k] for k in DROP_ORDER if dropped[k]},
                          "bad_lines": run["bad_lines"],
                          "recorder": {k: rc.get(k) for k in (
                              "captures", "skipped_front_frames", "capture_failed",
                              "capture_hz", "ticks_dropped", "saver") if k in rc},
                          "has_record_json": bool(run["record"])}
        say(f"[run] {tag:28s} {split:7s} captures {len(caps):5d} -> samples {n_run:5d}  "
            f"dropped {dict(run_stats[tag]['dropped'])}")

    with (out / SAMPLES).open("w", encoding="utf-8", newline="\n") as fh:
        for s in samples:
            fh.write(json.dumps(s, ensure_ascii=False) + "\n")
    stats = _stats(samples, run_stats, dropped_all, holdout=holdout, horizon_s=horizon_s,
                   stride_s=stride_s, fresh_s=fresh_s, max_gap_s=max_gap_s,
                   keep_close=keep_close, modes=modes, bearing_gain=bearing_gain,
                   images=images, n_written=n_written, n_reused=n_reused,
                   pose_dist=pose_dist, pose_dyaw=pose_dyaw, run_dirs=run_dirs)
    (out / STATS).write_text(json.dumps(stats, indent=1, ensure_ascii=False), encoding="utf-8",
                             newline="\n")
    return stats


def _stats(samples, run_stats, dropped_all, *, holdout, horizon_s, stride_s, fresh_s,
           max_gap_s, keep_close, modes, bearing_gain, images, n_written, n_reused,
           pose_dist, pose_dyaw, run_dirs) -> dict:
    splits = Counter(s["split"] for s in samples)
    hist = {}
    for name, sel in (("train", "train"), ("holdout", "holdout"), ("all", None)):
        ss = [s for s in samples if sel is None or s["split"] == sel]
        hist[name] = {axis: _hist(s["bins"][i] for s in ss) for i, axis in enumerate(C.AXES)}
    band = [s for s in samples if s["rotate_only"]]
    with_fwd = [s for s in band if s["bins"][0] > 0]
    clipped = {axis: {"low": sum(1 for s in samples if s["clipped"][axis] == "low"),
                      "high": sum(1 for s in samples if s["clipped"][axis] == "high")}
               for axis in C.AXES}
    return {
        "format": FORMAT, "tool": "tools/build_follow_dataset.py",
        "built_utc": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds"),
        "args": {"runs": [str(p) for p in run_dirs], "horizon_s": horizon_s,
                 "stride_s": stride_s, "holdout": list(holdout), "fresh_s": fresh_s,
                 "max_gap_s": max_gap_s, "keep_close": keep_close,
                 "modes": sorted(modes) if modes else None, "bearing_gain": bearing_gain,
                 "images": images},
        "codec": {"num_bins": C.NUM_BINS, "ranges": {k: list(v) for k, v in C.RANGES.items()},
                  "quantize": "clip, then truncate into 99 bins (upstream _quantize_action; "
                              "aerialvla_codec.step_bins)",
                  "text": "aerialvla_codec.action_text: 'FF DD YY', zero-padded, no EOS - the "
                          "trainer appends the tokenizer's </s>",
                  "prompt_template": C.PROMPT_TEMPLATE,
                  "mosaic": "aerialvla_codec.build_mosaic of the decoded JPEGs (front over "
                            "down, 224x448), lossless PNG"},
        "splits": {"train": splits.get("train", 0), "holdout": splits.get("holdout", 0)},
        "train_runs": sorted(t for t, r in run_stats.items() if r["split"] == "train"),
        "holdout_runs": sorted(t for t, r in run_stats.items() if r["split"] == "holdout"),
        "runs": run_stats,
        "dropped": {k: dropped_all[k] for k in DROP_ORDER if dropped_all[k]},
        "label_hist": hist,
        "label_spread": {axis: _spread(s["label"][axis] for s in samples) for axis in C.AXES},
        "phrase_counts": dict(Counter(s["phrase"] for s in samples)),
        "by_mode": dict(Counter(s["expert"]["mode"] for s in samples)),
        "shield_touched": sum(1 for s in samples if s["expert"].get("touched")),
        "rotate_only_band": {
            "n": len(band), "with_fwd": len(with_fwd),
            "frac": round(len(band) / len(samples), 4) if samples else None,
            "note": ("labels whose decoded |dpsi| reaches 0.25 rad: upstream's executor only "
                     "rotates on such a step, so with fwd > 0 they teach a step upstream would "
                     "not fly as labelled. Kept: the follow's executor (TrackerConfig "
                     "turn_while_moving) turns while moving.")},
        "small_fwd": {"lt_1m": sum(1 for s in samples if s["label"]["fwd_m"] < SMALL_FWD_M),
                      "zero_bin": sum(1 for s in samples if s["bins"][0] == 0),
                      "note": "the expert standing off close: kept"},
        "clipped": clipped,
        "subject_dist_m": _spread(s["analysis"]["subject_dist_m"] for s in samples),
        "box_age_s": _spread(s["box"]["age_s"] for s in samples),
        "pose_check": {"capture_vs_track_m": _spread(pose_dist),
                       "capture_vs_track_yaw_deg": _spread(pose_dyaw),
                       "note": "the capture's pose (the loop's pose ring at the frame's sim "
                               "stamp) against the track interpolated at its wall time; the "
                               "label uses the track at both ends"},
        "images": ({"dir": IMAGES, "written": n_written, "reused": n_reused}
                   if images else None),
        "notes": [
            "Fresh-box captures only: the follow's memory and search hints are not represented.",
            "LAND is never labelled: the expert never arrives.",
            "The phrase is computed by demo/vla_mission.DetectorHint from the recorded box.",
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
    ap.add_argument("--modes", nargs="+", default=None,
                    help="keep only captures the expert flew in these modes (e.g. track)")
    ap.add_argument("--bearing-gain", type=float, default=1.0,
                    help="VLABridge gain (1.0 honest; recorded)")
    ap.add_argument("--no-images", action="store_true",
                    help="labels and stats only: no mosaics, no Pillow")
    ap.add_argument("--force", action="store_true", help="rebuild an existing dataset")
    a = ap.parse_args(argv)
    holdout = [t for grp in (a.holdout or []) for t in grp]
    try:
        st = build_dataset(a.runs, a.out, horizon_s=a.horizon_s, stride_s=a.stride_s,
                           holdout=holdout, fresh_s=a.fresh_s, max_gap_s=a.max_gap_s,
                           keep_close=a.keep_close, modes=a.modes, bearing_gain=a.bearing_gain,
                           images=not a.no_images, force=a.force, log=print)
    except DatasetError as exc:
        print(f"[dataset] *** refused: {exc}", file=sys.stderr)
        return 2
    band = st["rotate_only_band"]
    print(f"[dataset] {st['splits']['train']} train / {st['splits']['holdout']} hold-out "
          f"samples -> {a.out}")
    print(f"[dataset] dropped: {st['dropped']}")
    print(f"[dataset] phrases: {st['phrase_counts']}")
    print(f"[dataset] rotate-only band (|dpsi| >= 0.25 rad): {band['n']}, of which with "
          f"fwd > 0: {band['with_fwd']} (kept; see stats.json)")
    print(f"[dataset] small steps (fwd < 1 m): {st['small_fwd']['lt_1m']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
