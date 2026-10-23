"""
What AerialVLA would have seen, recorded while the hand-written follow flies:
the training-data half of fine-tuning AerialVLA on following (wave FT-D,
2026-10-23).

    python demo/follow_vlm.py ... --record-vla demo/out/<tag>/vla_record
    .\\scripts\\run_citylife_follow.ps1 -Object "a red car" -LevelCar Car_10 -RecordVla ...
    python tools/build_follow_dataset.py demo/out/<tag> ... --out <dataset>

WHY

The hand-written follow (demo/follow_vlm.py) kept the red car within 30 m on
0.963 of its ticks (citylife_redcar_nfz_motion2_20261019); AerialVLA's
follows, on 0.308 (#9) and 0.17 (#10) of theirs (CHANGELOG 2026-10-23: #10
held 0.926 while it chased, then lost the car when it outran the drone).
AerialVLA was never trained to pursue. The approved step is behaviour cloning on the
expert's own flights - and a demonstration is only worth training on if it
shows the model exactly what the model will see in flight: the front+down
pair of its own training cameras (VLAFront / VLADown, 256x256, 90 deg,
pitch 0 / -90), not the follow rail's 768x432 FrontCamera pitched -20 deg.
So the follow, when asked, flies a robot config with those two cameras added
(robot_semantic_quad_vla.jsonc, through scene_semantic_vla.jsonc) and this
module writes, at the cameras' 10 Hz:

  * the front+down pair, paired by sim stamp, with the capture's wall time
    and the pose at it - vla_mission.VLAFrames, the very code that builds a
    VLA flight's request (newest front and down frames, paired within
    0.25 s, the pose from the loop's ring at the frame's stamp), and
    vla_mission.FrameSaver, which writes frames/<sha256>.jpg on its own
    thread;
  * the expert's box at that moment, read from the Grounder when the capture
    is built - as vla_mission.DetectorHint reads it when a VLA flight builds
    a request (box_from_grounder is vla_fly's latest_box rule): the box, its
    frame's capture time and the heading at that frame, so the builder can
    give it the phrase DetectorHint would have sent;
  * the expert's command and who made it for the tick in force at the
    capture (the follow's mode, what it asked, what the Shield let fly), and
    the level car's truth - ANALYSIS ONLY, never a label or a phrase;
  * track.jsonl, every loop tick's pose: the expert's motion, from which
    tools/build_follow_dataset.py makes the step labels;
  * with --record-perturb (DART, Perturber below), each tick's perturbation
    of the executed command - on the track line and the capture as
    `perturb` {active, yaw_rate, speed_scale, episode}, the schedule in
    record.json - so the builder can keep only the expert's clean motion as
    labels. An unperturbed recording has no `perturb` key anywhere;
  * with --record-perturb-vla (DAgger, demo/vla_dagger.py), AerialVLA's
    windows the same way: `perturb` {source "vla", active - the executed
    command was the VLA's -, the step, its seq, latency and phrase, ...},
    the summary in record.json, and in this folder vla_steps.jsonl (one line
    per request) and vla_frames/ (each request's JPEGs).

NEVER ON THE LOOP'S CRITICAL PATH

The loop's side is a pointer assignment per camera message (put_front /
put_down: VLAFrames stores the raw message, as SemanticObs does - decoding
in the callback starved the inference thread, semantic_demo.py) and one
deque append per tick (note_tick). The pairing, the JPEG encoding, the box
read, the JSON lines and the file writes happen on this module's writer
thread and FrameSaver's. A writer that falls behind does not queue frames:
it takes the newest pair when it is free and COUNTS the ones it skipped
(`skipped_front_frames`); FrameSaver counts what its full queue dropped.
tests/test_follow_dataset.py measures a tick's cost with an 80 ms encoder.
What the two extra 256x256 renders cost the simulator is not measured here
(the robot config's header says how to read it off metrics.json).

FILES, in the --record-vla folder (frames/ is content-addressed and kept
across runs under one tag; the three files are rewritten):
  captures.jsonl  one line per front+down pair written
  track.jsonl     one line per mission tick
  record.json     what was recorded with what, and the counts
  frames/         <sha256>.jpg, every JPEG a capture names

No projectairsim, cv2, Pillow or torch at module level (VLAFrames' default
JPEG encoder imports cv2 when a frame is encoded, in the flight's env).
"""
from __future__ import annotations

import json
import os
import threading
import time
from collections import deque
from pathlib import Path
from typing import Callable

import vla_mission as VM

FORMAT = "vla_record/1"
CAPTURES = "captures.jsonl"
TRACK = "track.jsonl"
RECORD = "record.json"
FRAMES = "frames"
# The robot / scene variant follow_vlm flies with --record-vla (demo/
# pas_config; tests/test_follow_dataset.py checks the robot config is
# robot_semantic_quad.jsonc plus exactly robot_vla_quad.jsonc's two cameras).
SCENE = "scene_semantic_vla.jsonc"
ROBOT_CONFIG = "robot_semantic_quad_vla.jsonc"
CONFIG_DIR = Path(__file__).resolve().parent / "pas_config"
CAMERAS = {"front": "VLAFront", "down": "VLADown", "size": [256, 256], "fov_deg": 90,
           "front_pitch_deg": 0, "down_pitch_deg": -90, "capture_interval_s": 0.1}
# Who authors every recorded tick: the hand-written controller (the Shield
# may have changed what flew; `touched` says when).
AUTHOR = "follow_vlm"


# --------------------------------------------------------------------------- #
# DART: knock the expert off now and then (--record-perturb)
# --------------------------------------------------------------------------- #
# ft_expert_01 (2026-10-23: 0.953 within 30 m, 2826 captures, 0 drops) gave
# 507 samples and every one said "straight ahead ": the expert keeps the car
# centred, so the data never shows an off-centre car being brought back - no
# turn labels, no left or right phrase. DART (Laskey et al., 2017, "DART:
# Noise Injection for Robust Imitation Learning") injects noise into the
# EXECUTED command while recording the expert, so the states it then has to
# recover from are in the data, and labels only the expert's own clean
# behaviour. tools/build_follow_dataset.py drops every sample whose label
# horizon overlaps a perturbation and keeps those that start just after one.
PERTURB_EVERY_S = (5.0, 8.0)       # onset to onset
PERTURB_DUR_S = (1.0, 1.5)
PERTURB_YAW_RATE = (0.3, 0.5)      # rad/s, random sign
PERTURB_SPEED_SCALE = (0.5, 1.0)   # --perturb-speed only


class Perturber:
    """A seeded schedule of perturbations of the executed command, in mission
    seconds: the first onset 5-8 s after the mission starts, then one every
    5-8 s (onset to onset), each lasting 1.0-1.5 s with a yaw-rate offset of
    random sign and 0.3-0.5 rad/s and, with `speed_scale` (the range, e.g.
    (0.5, 1.0)), the horizontal velocity scaled by a draw from it. Episodes
    are drawn in order from their own random.Random(seed) - the flight's
    global RNG is not touched - and every episode draws the same numbers with
    or without the speed option, so the two arms share one yaw schedule.
    `apply(action, s)` is called once per tick AFTER the expert's controller
    (and its slew limiter) and BEFORE the Shield, which still filters the
    result; an inactive tick returns the expert's action object itself.

    HOW FAR IT PUSHES (measured offline, 2026-10-23: the follow's 1.2 x
    bearing yaw servo on a point vehicle, 600 s, seed 101, through the real
    builder). The default ADDS the offset to the expert's yaw rate, and the
    expert fights it while it lasts: its 1.2 gain holds the bearing near
    offset / 1.2 = 14-24 deg, and by the end of the label margin it has
    mostly decayed under the 15 deg edge of "straight ahead ". So: turn
    labels (|dpsi| >= 0.05 rad on 276 of 782 samples, none clean) but few
    side phrases (13 of 782; 62 with a 0.2 s yaw lag). `replace_yaw` drops
    the expert's own yaw-rate command while a perturbation lasts, so only
    the offset turns the aircraft and the bearing drifts uncorrected (122
    side phrases of 782); a wider `yaw_rate` (0.6-0.9: 116) does about the
    same. Both are options, recorded in the summary; the default is the
    brief's additive 0.3-0.5 rad/s."""

    def __init__(self, seed: int, *, every_s=PERTURB_EVERY_S, dur_s=PERTURB_DUR_S,
                 yaw_rate=PERTURB_YAW_RATE, speed_scale=None, replace_yaw: bool = False):
        import random
        for name, (lo, hi) in (("every_s", every_s), ("dur_s", dur_s), ("yaw_rate", yaw_rate)):
            if not (0 < lo <= hi):
                raise ValueError(f"{name} must be 0 < lo <= hi, got {(lo, hi)!r}")
        if not (dur_s[1] < every_s[0]):
            raise ValueError("a perturbation must end before the next can start")
        if speed_scale is not None and not (0 < speed_scale[0] <= speed_scale[1] <= 1.0):
            raise ValueError(f"speed_scale must be within (0, 1], got {speed_scale!r}")
        self.seed = int(seed)
        self.every_s, self.dur_s = tuple(map(float, every_s)), tuple(map(float, dur_s))
        self.yaw_rate = tuple(map(float, yaw_rate))
        self.speed_scale = None if speed_scale is None else tuple(map(float, speed_scale))
        self.replace_yaw = bool(replace_yaw)
        self._rng = random.Random(self.seed)
        self.episodes: list[dict] = []
        self._max_s = 0.0
        self.n_ticks = self.n_active = 0

    def _draw_until(self, s: float) -> None:
        while not self.episodes or self.episodes[-1]["on_s"] <= s:
            prev = self.episodes[-1]["on_s"] if self.episodes else 0.0
            r = self._rng
            on = prev + r.uniform(*self.every_s)
            dur = r.uniform(*self.dur_s)
            sign = 1.0 if r.random() < 0.5 else -1.0
            mag = r.uniform(*self.yaw_rate)
            u = r.random()                                     # drawn either way
            lo, hi = self.speed_scale or (1.0, 1.0)
            self.episodes.append({"k": len(self.episodes), "on_s": on, "off_s": on + dur,
                                  "yaw_rate": sign * mag, "speed_scale": lo + u * (hi - lo)})

    def state(self, s: float) -> dict:
        """{active, yaw_rate, speed_scale, episode} at mission second s."""
        s = float(s)
        self._max_s = max(self._max_s, s)
        self._draw_until(s)
        for e in reversed(self.episodes):
            if e["on_s"] <= s:
                if s < e["off_s"]:
                    return {"active": True, "yaw_rate": e["yaw_rate"],
                            "speed_scale": e["speed_scale"], "episode": e["k"]}
                break
        return {"active": False, "yaw_rate": 0.0, "speed_scale": 1.0, "episode": None}

    def apply(self, action, s: float, pose=None):
        """(the action to send to the Shield, the tick's record). `pose` is
        not read here: follow_vlm's one call site passes it for the VLA
        perturber (demo/vla_dagger.VLAPerturber, the same interface)."""
        st = self.state(s)
        self.n_ticks += 1
        if not st["active"]:
            return action, st
        self.n_active += 1
        k = st["speed_scale"]
        yaw = st["yaw_rate"] if self.replace_yaw else action.yaw_rate + st["yaw_rate"]
        return type(action)(vx=action.vx * k, vy=action.vy * k, vz_up=action.vz_up,
                            yaw_rate=yaw), st

    def summary(self) -> dict:
        return {"seed": self.seed, "every_s": list(self.every_s), "dur_s": list(self.dur_s),
                "yaw_rate_rad_s": list(self.yaw_rate),
                "speed_scale": None if self.speed_scale is None else list(self.speed_scale),
                "replace_yaw": self.replace_yaw,
                "ticks": self.n_ticks, "active_ticks": self.n_active,
                "episodes": [dict(e) for e in self.episodes if e["on_s"] <= self._max_s],
                "rule": ("applied after the expert's controller and slew limiter, before the "
                         "Shield (which filters the result): "
                         + ("yaw_rate = offset (the expert's yaw dropped)" if self.replace_yaw
                            else "yaw_rate + offset")
                         + ", vx and vy times speed_scale; times are mission seconds")}


def box_from_grounder(g: dict | None) -> dict | None:
    """follow_vlm.Grounder.latest() -> the expert's box as a VLA flight's
    DetectorHint would read it, or None when there is no box: demo/
    vla_fly.py's latest_box rule (the box's frame time is the Grounder's
    cap["t_capture"], else cap["t_cap"], else t_det; its heading is the
    capture pose's yaw), as a dict so the record says which is which:
    {det: [cx, cy, w, h, score, W, H, colour], t_box, yaw_box,
    t_published (the Grounder's t_det), seq, tier}."""
    if not g or g.get("det") is None:
        return None
    cap = g.get("cap") or {}
    yaw = (cap.get("pose") or {}).get("yaw")
    det = list(g["det"][:8])
    out_det = [int(v) if i in (5, 6) else float(v) for i, v in enumerate(det)]
    return {"det": out_det,
            "t_box": cap.get("t_capture") or cap.get("t_cap") or g.get("t_det"),
            "yaw_box": None if yaw is None else float(yaw),
            "t_published": g.get("t_det"), "seq": g.get("seq"), "tier": g.get("tier")}


def _act(a) -> dict | None:
    """An Action4D (or a dict, or None) -> {vx, vy, vz_up, yaw_rate}."""
    if a is None:
        return None
    if hasattr(a, "model_dump"):
        a = a.model_dump()
    return {k: float(a[k]) for k in ("vx", "vy", "vz_up", "yaw_rate")}


class ExpertVLARecorder:
    """The --record-vla writer. `obs` is the flight's SemanticObs (its pose
    ring gives each capture's pose and wall time); `box_source()` returns
    box_from_grounder(...) or None. Call put_front / put_down from the
    camera subscriptions, start(t0) when the mission clock starts,
    note_tick(...) once per mission tick, and stop() on every way out (it
    returns the stats, also written to record.json). Never raises into the
    loop: a failed write is counted and the first errors are kept.

    A `perturber` with a lifecycle of its own (--record-perturb-vla,
    demo/vla_dagger.VLAPerturber: put_front / put_down / start / stop) is
    fed the same camera messages (a pointer assignment each), started with
    the recorder and stopped first in stop(), so its summary in record.json
    is final."""

    def __init__(self, out_dir: str | Path, obs, box_source: Callable[[], dict | None], *,
                 tag: str, object_text: str, expert: str = "demo/follow_vlm.py",
                 sim_config: dict | None = None, encode: Callable[[dict], bytes | None] | None = None,
                 clock: Callable[[], float] = time.time, max_pair_dt_s: float = 0.25,
                 poll_s: float = 0.01, max_pending_ticks: int = 20000, saver_queue: int = 4000,
                 flush_s: float = 1.0, perturber: Perturber | None = None):
        self.perturber = perturber          # --record-perturb's schedule, for the records
        # --record-perturb-vla's perturber asks AerialVLA with these cameras.
        self._tee = perturber if callable(getattr(perturber, "put_front", None)) else None
        self.dir = Path(out_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.tag, self.object_text, self.expert = str(tag), str(object_text), expert
        self.sim_config = (sim_config if sim_config is not None else
                           VM.sim_config_record(CONFIG_DIR / SCENE, depth_read=True))
        self.clock = clock
        self.box_source = box_source
        self.poll_s, self.flush_s = float(poll_s), float(flush_s)
        self.saver = VM.FrameSaver(self.dir / FRAMES, max_queue=saver_queue)
        self.frames = VM.VLAFrames(obs, encode=encode, saver=self.saver, clock=clock,
                                   max_pair_dt_s=max_pair_dt_s)
        self._lock = threading.Lock()
        self._ticks: deque = deque()
        self._max_ticks = int(max_pending_ticks)
        self._ring: deque = deque(maxlen=600)           # the writer's own, ~60 s of ticks
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.running = False
        self.t0: float | None = None
        self.t_start = self.t_stop = None
        # Counted while running only (frames before start() are no capture).
        self._front_seq = 0
        self._seq_at_start = 0
        self._last_seen_seq = 0
        self.n_front = self.n_down = 0
        self.n_attempts = self.n_skipped = self.n_captures = self.n_duplicates = 0
        self.n_ticks = self.n_ticks_dropped = 0
        self.n_with_box = 0
        self.n_no_tick = 0
        self.errors: list[str] = []
        self.n_errors = 0
        self._last_stamp = None
        self._fh_caps = self._fh_track = None
        self.stats_at_stop: dict | None = None

    # ---- the loop's side: pointer assignments and one append --------------
    def put_front(self, msg) -> None:
        self.frames.put_front(msg)
        if self._tee is not None:
            self._tee.put_front(msg)
        if self.running:
            with self._lock:
                self._front_seq += 1
                self.n_front += 1

    def put_down(self, msg) -> None:
        self.frames.put_down(msg)
        if self._tee is not None:
            self._tee.put_down(msg)
        if self.running:
            self.n_down += 1

    def note_tick(self, t: float, x: float, y: float, up: float, yaw: float, *, mode: str | None,
                  cmd=None, asked=None, touched: bool | None = None, truth=None,
                  obj: str | None = None, tick: int | None = None,
                  perturb: dict | None = None) -> None:
        """One mission tick: its pose (wall time `t` of the kinematics read,
        the loop's put_pose_full time), the follow's mode, the command that
        flew (`cmd`, the Shield's emitted action) and the one the controller
        asked for (`asked`), whether the Shield touched it, the subject's
        truth (analysis only), the object words in force and, on a perturbed
        recording only, the tick's perturbation (Perturber.apply's record:
        active, yaw_rate, speed_scale, episode; VLAPerturber.apply's, source
        "vla", kept whole: the step, its seq, latency and phrase) - absent
        otherwise, so an unperturbed recording's lines are as before."""
        if not self.running:
            return
        row = {"tick": tick, "t": float(t), "x": float(x), "y": float(y), "up": float(up),
               "yaw": float(yaw), "mode": mode, "author": AUTHOR, "cmd": _act(cmd),
               "asked": _act(asked), "touched": None if touched is None else bool(touched),
               "truth": None if truth is None else [float(truth[0]), float(truth[1])],
               "object": self.object_text if obj is None else obj}
        if perturb is not None and perturb.get("source") == "vla":
            # `active`: the executed command was AerialVLA's (the builder's rule).
            row["perturb"] = {**perturb, "active": bool(perturb.get("active"))}
        elif perturb is not None:
            row["perturb"] = {"active": bool(perturb.get("active")),
                              "yaw_rate": float(perturb.get("yaw_rate") or 0.0),
                              "speed_scale": float(perturb.get("speed_scale") or 1.0),
                              "episode": perturb.get("episode")}
        with self._lock:
            if len(self._ticks) >= self._max_ticks:
                self.n_ticks_dropped += 1
                return
            self._ticks.append(row)
            self.n_ticks += 1

    # ---- lifecycle ---------------------------------------------------------
    def start(self, t0: float) -> None:
        """The mission clock started at wall time `t0`: open the files and
        start the writer. Captures and ticks before this are not recorded."""
        if self._thread is not None:
            return
        self.t0 = float(t0)
        self.t_start = self.clock()
        self._fh_caps = (self.dir / CAPTURES).open("w", encoding="utf-8", newline="\n")
        self._fh_track = (self.dir / TRACK).open("w", encoding="utf-8", newline="\n")
        with self._lock:
            self._seq_at_start = self._last_seen_seq = self._front_seq
            self.running = True
        self._thread = threading.Thread(target=self._run, name="vla-record", daemon=True)
        self._thread.start()
        if callable(getattr(self.perturber, "start", None)):
            try:
                self.perturber.start(self.t0)
            except Exception as exc:                                 # noqa: BLE001
                self._error(exc)

    def stop(self, timeout_s: float = 10.0) -> dict:
        """Stop the writer, flush and close, write record.json; the stats.
        Safe to call twice and before start(). A perturber with a lifecycle
        is stopped first (bounded: its thread and link are joined with a
        timeout)."""
        if self.stats_at_stop is not None:
            return self.stats_at_stop
        if callable(getattr(self.perturber, "stop", None)):
            try:
                self.perturber.stop(timeout_s=timeout_s)
            except Exception as exc:                                 # noqa: BLE001
                self._error(exc)
        self.running = False
        self._stop.set()
        alive = False
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
            alive = self._thread.is_alive()
        saver = self.saver.close(timeout_s=timeout_s)
        with self._lock:
            # Front frames that arrived after the writer's last look.
            self.n_skipped += max(0, self._front_seq - self._last_seen_seq)
            self._last_seen_seq = self._front_seq
        for fh in (self._fh_caps, self._fh_track):
            try:
                if fh is not None:
                    fh.close()
            except OSError as exc:
                self._error(exc)
        self.t_stop = self.clock()
        st = self.stats()
        st["saver"] = saver
        st["writer_alive_at_stop"] = alive
        rec = self.record(st)
        try:
            tmp = self.dir / (RECORD + ".tmp")
            tmp.write_text(json.dumps(rec, indent=1), encoding="utf-8", newline="\n")
            os.replace(tmp, self.dir / RECORD)
        except OSError as exc:
            self._error(exc)
            st["errors"] = list(self.errors)
        self.stats_at_stop = st
        return st

    def stats(self) -> dict:
        span = (None if self.t_start is None else
                ((self.t_stop or self.clock()) - self.t_start))
        return {"dir": str(self.dir), "front_frames": self.n_front, "down_frames": self.n_down,
                "capture_attempts": self.n_attempts, "skipped_front_frames": self.n_skipped,
                "captures": self.n_captures, "duplicates": self.n_duplicates,
                "captures_with_box": self.n_with_box, "captures_without_tick": self.n_no_tick,
                "capture_failed": {k: v for k, v in self.frames.counts.items()
                                   if k not in ("captures", "committed")},
                "capture_hz": (None if not span else round(self.n_captures / span, 3)),
                "ticks": self.n_ticks, "ticks_dropped": self.n_ticks_dropped,
                "frames": self.frames.stats(), "saver": self.saver.stats(),
                "errors": list(self.errors), "n_errors": self.n_errors,
                **({} if self.perturber is None else {"perturb": self.perturber.summary()})}

    def record(self, st: dict | None = None) -> dict:
        return {"format": FORMAT, "tag": self.tag, "object": self.object_text,
                **({} if self.perturber is None else {"perturb": self.perturber.summary()}),
                "expert": self.expert, "author": AUTHOR, "t0": self.t0,
                "t_start": self.t_start, "t_stop": self.t_stop,
                "dir": str(self.dir.resolve()), "sim_config": self.sim_config,
                "cameras": dict(CAMERAS),
                "detector": {"camera": "FrontCamera", "hfov_deg": 90.0,
                             "box_rule": "vla_fly latest_box (demo/vla_record.box_from_grounder)"},
                "files": {"captures": CAPTURES, "track": TRACK, "frames": FRAMES + "/"},
                "counts": st if st is not None else self.stats(),
                "truth": "the subject's position is for analysis only: never a label, "
                         "a phrase or anything the model is given"}

    # ---- the writer --------------------------------------------------------
    def _error(self, exc: BaseException) -> None:
        self.n_errors += 1
        if len(self.errors) < 10:
            self.errors.append(f"{type(exc).__name__}: {exc}")

    def _run(self) -> None:
        last_flush = time.monotonic()
        while not self._stop.is_set():
            busy = False
            try:
                busy = self._drain_ticks() or busy
                with self._lock:
                    seq, seen = self._front_seq, self._last_seen_seq
                    if seq != seen:
                        self.n_skipped += seq - seen - 1
                        self._last_seen_seq = seq
                if seq != seen:
                    self.n_attempts += 1
                    self._capture_once()
                    busy = True
                if time.monotonic() - last_flush >= self.flush_s:
                    last_flush = time.monotonic()
                    for fh in (self._fh_caps, self._fh_track):
                        fh.flush()
            except Exception as exc:                                 # noqa: BLE001
                self._error(exc)
            if not busy:
                self._stop.wait(self.poll_s)
        try:
            self._drain_ticks()
        except Exception as exc:                                     # noqa: BLE001
            self._error(exc)

    def _drain_ticks(self) -> bool:
        with self._lock:
            rows = list(self._ticks)
            self._ticks.clear()
        for r in rows:
            self._ring.append(r)
            self._fh_track.write(json.dumps(r) + "\n")
        return bool(rows)

    def _tick_at(self, t: float) -> dict | None:
        """The tick in force at wall time `t`: the last one at or before it."""
        for r in reversed(self._ring):
            if r["t"] <= t + 1e-9:
                return r
        return None

    def _capture_once(self) -> None:
        cap = self.frames.capture()
        if cap is None:
            return                     # VLAFrames counted why (no_down, down_unpaired, ...)
        # The expert's box NOW, as DetectorHint reads the Grounder when a
        # VLA flight builds its request.
        box = self.box_source() if self.box_source is not None else None
        # commit() always appends this capture's line to frames.log (only
        # this thread commits) and hands both JPEGs to FrameSaver.
        self.frames.commit(cap, None, self.n_captures)
        base = dict(self.frames.log[-1])
        stamp = base.get("stamp_ns")
        key = stamp if stamp is not None else base.get("front_jpg_sha256")
        if key is not None and key == self._last_stamp:
            # The same front frame again: put_front stored it a moment
            # before the arrival count moved, or the simulator resent a
            # stamp. Its JPEGs are already saved (FrameSaver dedups by hash).
            self.n_duplicates += 1
            self.frames.log.pop()
            return
        self._last_stamp = key
        for k in ("seq", "phrase", "hint_source"):
            base.pop(k, None)
        tick = self._tick_at(cap.t_capture)
        if tick is None:
            self.n_no_tick += 1
        if box is not None:
            self.n_with_box += 1
        line = {"i": self.n_captures, **base, "t_capture": cap.t_capture,
                "t_mission": None if self.t0 is None else cap.t_capture - self.t0,
                "pose": cap.pose.as_wire(), "box": box,
                "tick": (None if tick is None else
                         {"tick": tick.get("tick"), "t": tick["t"], "mode": tick.get("mode"),
                          "author": tick.get("author"), "cmd": tick.get("cmd"),
                          "asked": tick.get("asked"), "touched": tick.get("touched")}),
                "truth": None if tick is None else tick.get("truth"),
                "object": self.object_text if tick is None else tick.get("object")}
        if tick is not None and "perturb" in tick:
            # The perturbation of the tick in force at the capture (the
            # builder reads track.jsonl for the label rule; this is for
            # looking at a capture on its own).
            line["tick"]["perturb"] = line["perturb"] = tick["perturb"]
        self._fh_caps.write(json.dumps(line) + "\n")
        self.n_captures += 1


def record_summary(st: dict | None) -> dict | None:
    """The part of the recorder's stats metrics.json carries (the full
    counts are in record.json beside the frames)."""
    if st is None:
        return None
    keys = ("dir", "front_frames", "capture_attempts", "skipped_front_frames", "captures",
            "captures_with_box", "capture_failed", "capture_hz", "ticks", "ticks_dropped",
            "saver", "errors", "writer_alive_at_stop")
    out = {k: st.get(k) for k in keys}
    if st.get("perturb") is not None:
        # --record-perturb: the schedule's settings and counts; the episode
        # list itself stays in record.json.
        p = st["perturb"]
        out["perturb"] = {**{k: v for k, v in p.items() if k != "episodes"},
                          "episodes": len(p.get("episodes") or []),
                          "active_s": round(sum(e["off_s"] - e["on_s"]
                                                for e in p.get("episodes") or []), 3)}
    return out


__all__ = ["ExpertVLARecorder", "Perturber", "box_from_grounder", "record_summary", "FORMAT",
           "CAPTURES", "TRACK", "RECORD", "FRAMES", "SCENE", "ROBOT_CONFIG", "CAMERAS", "AUTHOR",
           "PERTURB_EVERY_S", "PERTURB_DUR_S", "PERTURB_YAW_RATE", "PERTURB_SPEED_SCALE"]
