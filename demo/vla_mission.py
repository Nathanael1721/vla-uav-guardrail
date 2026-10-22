"""
The VLA-flown CityLife missions, without the simulator: hints, the mission's
own state machine, the 10 Hz loop body, the VLA server's lifecycle and the
run's records. demo/vla_fly.py wires these to Project AirSim; the tests drive
them with a point vehicle and a scripted protocol-v2 server.

WHO FLIES (docs/DESIGN-vla-pilot.md)

AerialVLA (OpenVLA-7B + the original `aero_vla` LoRA, in its own process,
demo/vla_server_v2.py) chooses every step from the front+down camera mosaic
and ONE compass phrase. guardrail/vla_link.StepTracker flies each step at
10 Hz (a deterministic interpolator, not a planner); the Shield filters every
tick. The phrase comes from one of two places, never from language:

  M1 GOTO      hint_source "goal": the bearing of the commanded goal from the
               capture pose, relative to the heading (vla_link.goal_phrase),
               "" inside the arrive radius. This is the task AerialVLA was
               trained on: upstream computes the phrase from the target
               position (src/vlnce_src/closeloop_util.py:120) and the empty
               phrase appears once per training trajectory, on its last frame,
               with LAND (vla-assets discovery report, 7,611 of 410,450).
  M2 APPROACH  hint_source "detector": the OWL-ViT box centre's bearing on its
               own camera (demo/vla_bridge.VLABridge). No fresh box: no
               request (the bridge never emits "", the codec refuses it).

The object words in the prompt are inert (docs/FINDING-what-drives-
aerialvla.md); they are recorded, never claimed as grounding.

WHAT THIS MODULE DOES NOT DO

It never talks to the autopilot. LAND is the tracker's flag
(Tick.land_requested); MissionMonitor turns it into the end reason "vla_land"
and the flight script lands through its own controlled landing
(demo/follow_vlm._fly_to_landing_site + the descent). A dead or erroring
server makes the tracker hover (vla_link: link_lost / server_error); after
`dead_timeout_s` of that the monitor ends the mission and the script lands.

No projectairsim, cv2, Pillow or torch at module level: the CI jobs import
this file with requirements.txt only.
"""
from __future__ import annotations

import json
import math
import os
import socket
import statistics
import subprocess
import sys
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail.models import Action4D, State                         # noqa: E402
from guardrail.vla_link import (                                      # noqa: E402
    NO_HINT, Capture, Hint, Pose, RemoteVLA, LinkError, ProtocolError, check_identity,
    goal_hint, phrase_for_bearing, wrap_pi,
)

MISSIONS = ("goto", "approach")

# --------------------------------------------------------------------------- #
# CityLife geometry, chosen on the level's own maps
# --------------------------------------------------------------------------- #
# Read 2026-10-22 from demo/out/citymap_citylife (build_voxel_map.py over NED
# x -140..220, y -60..140 at 2 m; occ[i, j] with i = (x+140)/2, j = (y+60)/2)
# through the Shield's own distance field (Shield.clearance_at) under the
# 6-14 m band map, the band the policy flies (occ_bands.select_for_band picks
# occ_day_flightband_6to14.npz for alt-band 6-14):
#
# * THE USUAL SPAWN, (35, -20) (demo/pas_config/scene_semantic.jsonc, every
#   CityLife follow flight), has 4.08 m of clearance, but two street trees
#   stand in the flight band at (32-34, -16) and (32-34, -12). Every straight
#   line from the spawn toward the street's east leg passes them within
#   0.8-1.3 m (to (44, 40): 1.12 m at (35.6, -16.1)), inside the 3 m
#   clearance rule. So the VLA does not start at the spawn: after the climb, a
#   hand-written transfer through the Shield (recorded as `pre_mission`,
#   before the mission clock, never VLA-authored) takes the aircraft 7 m to
# * THE MISSION START, (42, -18): clearance 4.46 m, on the street mask, in the
#   open lane between the south trees (x 36-38) and the north trees (x 46-50)
#   (row-by-row clearance at x 36..52: y=-10 2.6/4.5/6.3/8.2/7.1/5.2/4.5/3.7/
#   3.7; y=22 -2.0/-2.0/1.8/3.7/4.5/2.6/1.8/1.8/2.6).
# * THE DEFAULT GOAL, (43, 40): 58.0 m due east of the start (bearing 89.0
#   deg), on the street mask, 13.5 m of clearance at the goal. The straight
#   line start -> goal never drops below 3.97 m (at the y = 22 pinch, x 42.7),
#   so a VLA that flies the phrase it is given never needs the Shield. The
#   city no-fly zone of follow_car_citylife_nfz.yaml (x 45-62, y 62-92,
#   policies/vla_goto_citylife.yaml carries it) begins 22 m past the goal and
#   2 m north of its lane: met only by an overshoot or a drift north. A goal
#   past the zone along the same lane (e.g. --goal 43,100) flies beside it.
CITYLIFE_SPAWN_XY = (35.0, -20.0)
DEFAULT_START_XY = (42.0, -18.0)
DEFAULT_GOAL_XY = (43.0, 40.0)
# Down the street (east). The GOTO aligns to the goal's bearing instead
# (start_heading "goal"); the APPROACH keeps this scene constant, never the
# parked car's bearing (that would aim the camera with the answer, the defect
# follow_vlm's start heading removed).
STREET_HEADING_DEG = 90.0
# M2's scenario car (the orange SM_Offroad_Body env actor, demo/moving_car.py
# CarSpec) parked on the north kerb, 30.6 m from the start at a body bearing
# of -11 deg ("straight ahead "), clearance 4.5 m. Truth for SCORING only.
DEFAULT_PARK_XY = (48.0, 12.0)
# The middle of the 6-14 m policy band, the band the obstacle map is true for.
DEFAULT_CRUISE_UP = 10.0
# The manifest's model id. The server's identity (id/base with revisions,
# precision, attn, merged, agreement) goes into the pilot record beside it;
# tools/todo.py vla_family reads "aerialvla" in both.
MODEL_ID = "aerialvla:openvla-7b+aero_vla"


def bearing_to(pose: Pose, xy: tuple[float, float]) -> float:
    """Body-frame bearing of a world point from `pose`, radians, + = right of
    the nose (the guardrail frame: x North, y East, yaw clockwise from North)."""
    return wrap_pi(math.atan2(float(xy[1]) - pose.y, float(xy[0]) - pose.x) - pose.yaw_rad)


def distance_to(pose: Pose, xy: tuple[float, float]) -> float:
    return math.hypot(float(xy[0]) - pose.x, float(xy[1]) - pose.y)


# --------------------------------------------------------------------------- #
# Hints
# --------------------------------------------------------------------------- #
class HintSource:
    """A `get_hint(capture)` for RemoteVLABackend that remembers what it said:
    the phrase and source of every request's hint, for the per-tick rows and
    the HUD (the phrase OF THE DECISION FLOWN comes from its vla_steps record;
    this one is the latest SENT). `fn(capture) -> Hint | None`."""

    def __init__(self, fn: Callable[[Capture], Hint | None], kind: str):
        if kind not in ("goal", "detector"):
            raise ValueError(f"hint kind {kind!r} is not goal or detector")
        self.fn = fn
        self.kind = kind
        self.last: dict | None = None
        self.counts: Counter = Counter()
        self.n_none = 0

    def __call__(self, capture: Capture) -> Hint | None:
        h = self.fn(capture)
        if h is None:
            self.n_none += 1
            self.last = {"t_capture": capture.t_capture, "phrase": None, "source": None}
            return None
        if h.source != self.kind and not (self.kind == "goal" and h.source in ("goal", "none")):
            raise ValueError(f"a {self.kind} hint source produced a {h.source!r} hint")
        self.counts[h.phrase] += 1
        self.last = {"t_capture": capture.t_capture, "phrase": h.phrase, "source": h.source}
        return h

    def stats(self) -> dict:
        return {"kind": self.kind, "phrase_counts": dict(self.counts), "no_hint": self.n_none,
                "distinct_phrases": len([p for p, n in self.counts.items() if n])}


def goto_hint_source(goal_xy: tuple[float, float], arrive_radius_m: float) -> HintSource:
    """M1: the phrase toward the goal from each capture pose (hint_source
    "goal"), "" within `arrive_radius_m` - AerialVLA's trained "you have
    arrived" cue, on which it learned LAND. vla_link.goal_hint does the
    arithmetic (semantic_direction's bands); only the phrase leaves this
    process, the goal never does."""
    if not arrive_radius_m > 0:
        raise ValueError("arrive_radius_m must be positive")
    return HintSource(goal_hint(goal_xy, arrive_radius_m), "goal")


class DetectorHint:
    """M2: the compass phrase of the newest FRESH OWL-ViT box, through
    demo/vla_bridge.VLABridge (box centre -> pinhole bearing -> the training
    band). `latest() -> (det, t_box)`: the Grounder tuple (cx, cy, w, h,
    score, W, H, colour) and the wall time its frame was captured, or (None,
    None). A box older than `fresh_s` at the VLA capture, or none at all, is
    NO REQUEST (None): the bridge never emits "", and a detector hint of ""
    is refused by the codec, because "" asks the model to LAND wherever the
    box was lost. 1.0 s is DETECTION_STALE_S of the ArduPilot rail
    (perception_node.py). `box_age_s` of the last hint is kept for the rows."""

    def __init__(self, latest: Callable[[], tuple[Any, float | None]], bridge,
                 fresh_s: float = 1.0):
        if not fresh_s > 0:
            raise ValueError("fresh_s must be positive")
        self.latest = latest
        self.bridge = bridge
        self.fresh_s = float(fresh_s)
        self.n_stale = 0
        self.n_none = 0
        self.last_box_age_s: float | None = None

    def __call__(self, capture: Capture) -> Hint | None:
        det, t_box = self.latest()
        if det is None or t_box is None:
            self.n_none += 1
            self.last_box_age_s = None
            return None
        age = float(capture.t_capture) - float(t_box)
        self.last_box_age_s = age
        if age > self.fresh_s:
            self.n_stale += 1
            return None
        phrase = self.bridge.phrase_for(det)
        if phrase is None:
            self.n_none += 1
            return None
        return Hint(phrase, "detector")


def box_fraction(det) -> float | None:
    """Box width over image width (det[2] / det[5]), or None."""
    if det is None:
        return None
    try:
        w, W = float(det[2]), float(det[5])
    except (TypeError, ValueError, IndexError):
        return None
    return w / W if W > 0 else None


# --------------------------------------------------------------------------- #
# The mission's own state machine
# --------------------------------------------------------------------------- #
END_REASONS = {
    # the model's own decision
    "vla_land": "the model answered LAND (Tick.land_requested): controlled landing",
    # the mission's perception-derived or goal-derived stops
    "arrived_no_land": "inside the arrive radius for arrive_grace_s with no LAND",
    "close_box": "the detector's box filled close_box_frac of its frame",
    "target_lost": "no fresh detection for lost_timeout_s",
    "timeout": "the mission time ran out",
    # the VLA failed
    "server_dead": "no answer (link lost) for dead_timeout_s while hovering",
    "server_error": "only server errors (ok=false) for dead_timeout_s while hovering",
    "parse_fail_streak": "only malformed answers (held zero) for dead_timeout_s",
    "no_first_decision": "no answer the model parsed within first_decision_timeout_s",
}
VLA_FAILURE_ENDS = ("server_dead", "server_error", "parse_fail_streak", "no_first_decision")


@dataclass(frozen=True)
class MonitorConfig:
    """The mission's stop rules. Starting values, ours (not upstream's or the
    grant's), recorded in metrics.json with every run."""
    mission: str = "goto"
    max_s: float = 180.0
    arrive_radius_m: float = 3.0         # M1: inside it the hint is "" and the goal counts as reached
    arrive_grace_s: float = 8.0          # M1: time inside without LAND before the mission ends
    close_box_frac: float = 0.15         # M2: box width / frame width that counts as close
    lost_timeout_s: float = 15.0         # M2: no fresh box this long ends the mission
    det_fresh_s: float = 1.0             # M2: a box older than this is not "seen"
    dead_timeout_s: float = 10.0         # hover on link_lost / server_error this long ends it
    first_decision_timeout_s: float = 60.0

    def __post_init__(self):
        if self.mission not in MISSIONS:
            raise ValueError(f"mission {self.mission!r} is not one of {MISSIONS}")
        for k in ("max_s", "arrive_radius_m", "arrive_grace_s", "close_box_frac",
                  "lost_timeout_s", "det_fresh_s", "dead_timeout_s", "first_decision_timeout_s"):
            v = getattr(self, k)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) \
                    or v <= 0:
                raise ValueError(f"{k} must be a positive number, got {v!r}")
        if not self.close_box_frac < 1.0:
            raise ValueError("close_box_frac must be below 1")

    def as_dict(self) -> dict:
        return {k: getattr(self, k) for k in self.__dataclass_fields__}


class MissionMonitor:
    """When the VLA mission ends, and why (END_REASONS). One `update()` per
    control tick, after the backend's act(); it returns the end reason on the
    tick the mission ends, else None. It never flies anything.

    Order, first match wins: LAND; a VLA failure (the tracker has hovered on
    link_lost / server_error, or held on parse_fail, for dead_timeout_s
    without a break; or no parsed answer at all within
    first_decision_timeout_s); the mission's own stop (M1: inside the arrive
    radius for arrive_grace_s; M2: the box large enough, or no fresh box for
    lost_timeout_s); the mission time. The server process having exited is
    not an end by itself: its link failures make the tracker hover, and the
    hover rule ends the mission dead_timeout_s later (the brief: hover, then
    land after a timeout), with the exit recorded."""

    def __init__(self, cfg: MonitorConfig, t0: float):
        self.cfg = cfg
        self.t0 = float(t0)
        self.end_reason: str | None = None
        self.t_end: float | None = None
        self._streak_kind: str | None = None
        self._streak_t0: float | None = None
        self._t_first_decision: float | None = None
        self._t_inside: float | None = None
        self._t_last_box: float | None = None
        self.t_server_exit: float | None = None
        self.events: list[dict] = []
        self.ticks_inside = 0

    def _end(self, why: str, now: float) -> str:
        self.end_reason, self.t_end = why, now
        self.events.append({"t": round(now - self.t0, 3), "event": "end", "reason": why})
        return why

    def update(self, now: float, pose: Pose, tick, *, goal_xy=None, det=None,
               det_age_s: float | None = None, server_alive: bool = True) -> str | None:
        if self.end_reason is not None:
            return self.end_reason
        cfg = self.cfg
        if not server_alive and self.t_server_exit is None:
            self.t_server_exit = now
            self.events.append({"t": round(now - self.t0, 3), "event": "server_exited"})
        if tick.land_requested:
            return self._end("vla_land", now)
        if tick.author in ("vla_step", "hold") and tick.seq is not None \
                and tick.reason != "parse_fail" and self._t_first_decision is None:
            self._t_first_decision = now
        # A VLA failure: the tracker has been in one no-answer state since t.
        kind = None
        if tick.author in ("hover", "none") and tick.reason in ("link_lost", "server_error"):
            kind = "server_error" if tick.reason == "server_error" else "server_dead"
        elif tick.author == "hold" and tick.reason == "parse_fail":
            kind = "parse_fail_streak"
        if kind != self._streak_kind:
            self._streak_kind, self._streak_t0 = kind, (now if kind else None)
        if kind is not None and now - self._streak_t0 >= cfg.dead_timeout_s:
            return self._end(kind, now)
        if self._t_first_decision is None and now - self.t0 >= cfg.first_decision_timeout_s:
            return self._end("no_first_decision", now)
        if cfg.mission == "goto":
            if goal_xy is None:
                raise ValueError("a goto mission needs goal_xy")
            if distance_to(pose, goal_xy) <= cfg.arrive_radius_m:
                self.ticks_inside += 1
                if self._t_inside is None:
                    self._t_inside = now
                    self.events.append({"t": round(now - self.t0, 3), "event": "inside_arrive_radius"})
                if now - self._t_inside >= cfg.arrive_grace_s:
                    return self._end("arrived_no_land", now)
            elif self._t_inside is not None:
                self.events.append({"t": round(now - self.t0, 3), "event": "left_arrive_radius"})
                self._t_inside = None
        else:
            fresh = det is not None and det_age_s is not None and det_age_s <= cfg.det_fresh_s
            if fresh:
                self._t_last_box = now
                frac = box_fraction(det)
                if frac is not None and frac >= cfg.close_box_frac:
                    return self._end("close_box", now)
            since = self._t_last_box if self._t_last_box is not None else self.t0
            if now - since >= cfg.lost_timeout_s:
                return self._end("target_lost", now)
        if now - self.t0 >= cfg.max_s:
            return self._end("timeout", now)
        return None

    def summary(self) -> dict:
        return {"end_reason": self.end_reason,
                "end_reason_means": END_REASONS.get(self.end_reason),
                "vla_failure": self.end_reason in VLA_FAILURE_ENDS,
                "t_end_s": None if self.t_end is None else round(self.t_end - self.t0, 3),
                "t_first_decision_s": (None if self._t_first_decision is None
                                       else round(self._t_first_decision - self.t0, 3)),
                "t_server_exit_s": (None if self.t_server_exit is None
                                    else round(self.t_server_exit - self.t0, 3)),
                "ticks_inside_arrive_radius": self.ticks_inside,
                "events": list(self.events), "config": self.cfg.as_dict()}


# --------------------------------------------------------------------------- #
# One 10 Hz tick: backend -> Shield -> monitor -> row
# --------------------------------------------------------------------------- #
@dataclass
class LoopTick:
    flown: Action4D          # what to send to the autopilot
    decision: Any            # the ShieldDecision
    row: dict                # the flight_log.jsonl row
    hud: dict                # what the recorder draws
    end_reason: str | None


def _r(v, n=3):
    return None if v is None else round(float(v), n)


class MissionLoop:
    """The body of the flight loop, shared by demo/vla_fly.py and the tests.

    tick(state):
      1. backend.act(state): the step tracker's world-frame Action4D, its
         author in backend.last_tick (vla_step / hold / hover / none);
      2. shield.filter(state, action): the Shield ALWAYS judges the action;
         with shield_on the repaired `emitted` flies, with it off the VLA's
         raw action flies and the decision is the shadow record
         (metrics/rows "shield": "off", which guardrail/kpi.py reads as the
         control arm). guardrail.replay.flown_fields says what was FLOWN and
         its re-check, so the P0 escape count is measured, not inferred;
      3. monitor.update(...): the end reason, if this tick ends the mission;
      4. the row: who authored the tick, the decision being flown (seq, age,
         its phrase and hint source, step and raw text from its vla_steps
         record), the Shield's decision, and the mission's numbers.

    `det_source() -> (det, box capture wall time)` for M2 (None for M1).
    `server_alive() -> bool` for a server this process started."""

    def __init__(self, backend, shield, monitor: MissionMonitor, *, mission: str,
                 shield_on: bool = True, goal_xy=None, det_source=None, hint_source=None,
                 detector_hint: DetectorHint | None = None, audit=None,
                 server_alive: Callable[[], bool] = lambda: True,
                 clock: Callable[[], float] = time.time):
        from guardrail.replay import flown_fields
        if mission not in MISSIONS:
            raise ValueError(f"mission {mission!r}")
        if mission == "goto" and goal_xy is None:
            raise ValueError("a goto mission needs goal_xy")
        if mission == "approach" and det_source is None:
            raise ValueError("an approach mission needs det_source")
        self.backend, self.shield, self.monitor = backend, shield, monitor
        self.mission, self.shield_on = mission, bool(shield_on)
        self.goal_xy = None if goal_xy is None else (float(goal_xy[0]), float(goal_xy[1]))
        self.det_source, self.hint_source = det_source, hint_source
        self.detector_hint = detector_hint
        self.audit, self.server_alive, self.clock = audit, server_alive, clock
        self._flown_fields = flown_fields
        self.n = 0
        self.t0: float | None = None
        self.n_touched = 0
        self.authors: Counter = Counter()

    def tick(self, state: State, extra_row: dict | None = None) -> LoopTick:
        now = self.clock()
        if self.t0 is None:
            self.t0 = now
        action = self.backend.act(state)
        tk = self.backend.last_tick
        d = self.shield.filter(state, action)
        flown = d.emitted if self.shield_on else action
        ff = self._flown_fields(self.shield, state, d, shield_on=self.shield_on)
        if self.audit is not None:
            try:
                self.audit.log(self.n, d)
            except Exception:                                    # noqa: BLE001
                pass                       # the audit log never costs a tick
        pose = Pose.from_state(state)
        det = t_box = None
        if self.det_source is not None:
            det, t_box = self.det_source()
        det_age = None if t_box is None else now - float(t_box)
        try:
            alive = bool(self.server_alive())
        except Exception:                                        # noqa: BLE001
            alive = False
        end = self.monitor.update(now, pose, tk, goal_xy=self.goal_xy, det=det,
                                  det_age_s=det_age, server_alive=alive)
        cur = getattr(self.backend.tracker, "current_record", None)
        cur = cur if isinstance(cur, dict) else {}
        step = cur.get("step") if isinstance(cur.get("step"), dict) else None
        self.authors[tk.author] += 1
        self.n_touched += int(d.touched)
        gd = gb = None
        if self.goal_xy is not None:
            gd, gb = distance_to(pose, self.goal_xy), math.degrees(bearing_to(pose, self.goal_xy))
        frac = box_fraction(det)
        last = (self.hint_source.last or {}) if self.hint_source is not None else {}
        row = {
            "t": round(now - self.t0, 3), "tick": self.n,
            "x": state.x, "y": state.y, "up": state.up, "psi": round(state.yaw_deg, 3),
            # who flew this tick (vla_link.Tick) and the decision behind it
            "author": tk.author, "author_reason": tk.reason,
            "vla_seq": tk.seq, "vla_age_s": _r(tk.age_s),
            "land_requested": bool(tk.land_requested),
            "phrase": cur.get("phrase"), "hint_source": cur.get("hint_source"),
            "step": step, "bins": cur.get("bins"), "raw_text": cur.get("raw_text"),
            "hint_sent": last.get("phrase"), "hint_sent_source": last.get("source"),
            # the Shield
            "shield": "on" if self.shield_on else "off",
            "raw": action.model_dump(), "emitted": d.emitted.model_dump(),
            "flown_action": flown.model_dump(),
            "touched": d.touched, "braked": d.braked,
            "violations": [v.model_dump() for v in d.violations],
            "repairs": [r.model_dump() for r in d.repairs],
            **ff,
            # the mission
            "goal_dist_m": _r(gd), "goal_bearing_deg": _r(gb, 2),
            "det": (None if det is None else
                    {"cx": _r(det[0], 1), "cy": _r(det[1], 1), "w": _r(det[2], 1),
                     "score": _r(det[4], 4), "img_w": int(det[5])}),
            "det_age_s": _r(det_age), "box_frac": _r(frac, 4),
            "box_age_at_hint_s": (_r(self.detector_hint.last_box_age_s)
                                  if self.detector_hint is not None else None),
        }
        if extra_row:
            row.update(extra_row)
        hud = {"t": now - self.t0, "alt": state.up, "author": tk.author,
               "reason": tk.reason, "seq": tk.seq, "age_s": tk.age_s,
               "phrase": cur.get("phrase"), "hint_source": cur.get("hint_source"),
               "step": step, "land": bool(tk.land_requested), "shield": d.touched,
               "shield_on": self.shield_on, "mission": self.mission,
               "goal_dist_m": gd, "box_frac": frac, "end_reason": end}
        self.n += 1
        return LoopTick(flown, d, row, hud, end)


# --------------------------------------------------------------------------- #
# vla_frames/: the JPEG bytes of every request, for the offline replay gate
# --------------------------------------------------------------------------- #
class FrameSaver:
    """Writes each request's JPEG bytes to <dir>/<sha256>.jpg on its own
    thread (vla_steps.jsonl names them by front_jpg_sha256 / down_jpg_sha256),
    so tools/vla_replay_gate.py can send the very same bytes to the server
    again. Never raises into the loop; a failed write is counted."""

    def __init__(self, out_dir: str | Path, max_queue: int = 2000):
        import hashlib
        self._sha = hashlib.sha256
        self.dir = Path(out_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self._q: deque = deque()
        self._max = int(max_queue)
        self._cv = threading.Condition()
        self._closed = False
        self._seen: set[str] = set()
        self.n_written = self.n_errors = self.n_dropped = 0
        self.last_error: str | None = None
        self._t = threading.Thread(target=self._run, name="vla-frames", daemon=True)
        self._t.start()

    def save(self, data: bytes | None) -> str | None:
        if not data:
            return None
        h = self._sha(bytes(data)).hexdigest()
        with self._cv:
            if h in self._seen or self._closed:
                return h
            if len(self._q) >= self._max:
                self.n_dropped += 1
                return h
            self._seen.add(h)
            self._q.append((h, bytes(data)))
            self._cv.notify_all()
        return h

    def _run(self) -> None:
        while True:
            with self._cv:
                while not self._q and not self._closed:
                    self._cv.wait()
                if not self._q:
                    return
                h, data = self._q.popleft()
            try:
                p = self.dir / f"{h}.jpg"
                if not p.exists():
                    tmp = p.with_suffix(".tmp")
                    tmp.write_bytes(data)
                    os.replace(tmp, p)
                with self._cv:
                    self.n_written += 1
            except Exception as exc:                             # noqa: BLE001
                with self._cv:
                    self.n_errors += 1
                    self.last_error = f"{type(exc).__name__}: {exc}"

    def close(self, timeout_s: float = 10.0) -> dict:
        with self._cv:
            self._closed = True
            self._cv.notify_all()
        self._t.join(timeout=timeout_s)
        return self.stats()

    def stats(self) -> dict:
        with self._cv:
            return {"dir": self.dir.name, "written": self.n_written, "errors": self.n_errors,
                    "dropped": self.n_dropped, "pending": len(self._q),
                    "last_error": self.last_error}


# --------------------------------------------------------------------------- #
# The camera pair -> a Capture (JPEG bytes + the pose and time of capture)
# --------------------------------------------------------------------------- #
def jpeg_from_msg(msg: dict | None, quality: int = 95) -> bytes | None:
    """A Project AirSim camera message (raw BGR, or PNG when compressed) ->
    JPEG bytes, the wire form protocol v2 carries. OpenCV encodes the BGR
    array as it comes (JPEG stores YCbCr; cv2 expects BGR), and the server
    decodes to RGB with Pillow (aerialvla_codec.decode_image_b64). Quality 95
    is the server bench's (vla_server_v2.bench_requests). None for an empty
    or undecodable message."""
    if not msg or "data" not in msg or not len(msg["data"]):
        return None
    import cv2
    import numpy as np
    data = msg["data"]
    buf = (np.array(data, dtype=np.uint8) if isinstance(data, list)
           else np.frombuffer(data, dtype=np.uint8))
    try:
        arr = (cv2.imdecode(buf, cv2.IMREAD_COLOR) if msg.get("encoding") == "PNG"
               else buf.reshape(int(msg["height"]), int(msg["width"]), 3))
    except (ValueError, KeyError):
        return None
    if arr is None:
        return None
    ok, enc = cv2.imencode(".jpg", arr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return enc.tobytes() if ok else None


def _stamp(msg) -> float | None:
    s = (msg or {}).get("time_stamp")
    try:
        return None if s is None else float(s)
    except (TypeError, ValueError):
        return None


class VLAFrames:
    """The VLA camera pair (VLAFront, VLADown: AerialVLA's training cameras,
    robot_vla_quad.jsonc). Subscription callbacks store the raw message and
    its arrival time - a pointer assignment, as SemanticObs does, because
    decoding in the callback starved the inference thread (semantic_demo.py)
    - and `capture()`, called by RemoteVLABackend when it is about to send a
    request (about once per decision, not per tick), builds the Capture:

      * the newest front frame and the newest down frame, paired by their
        sim time stamps (else by arrival); a down frame further than
        `max_pair_dt_s` from the front one, or none at all, is NO REQUEST
        this tick (counted): the server runs with --missing-down refuse,
        because a black lower tile is an input the model never saw;
      * the capture pose from `obs.pose_at` (SemanticObs's ring of the
        loop's kinematics, interpolated at the frame's sim stamp) and the
        capture time from `obs.wall_at_stamp`, never later than the frame's
        arrival or now (the tracker refuses a capture "from the future");
      * both tiles JPEG-encoded (`encode`, default jpeg_from_msg), handed to
        `saver` (FrameSaver: vla_frames/<sha256>.jpg for the replay gate),
        and a line of `log` with the pairing and the camera sizes - the
        evidence that the frames really were 256x256."""

    def __init__(self, obs, encode: Callable[[dict], bytes | None] | None = None,
                 saver: FrameSaver | None = None, clock: Callable[[], float] = time.time,
                 max_pair_dt_s: float = 0.25, max_log: int = 20000):
        self.obs = obs
        self.encode = encode or jpeg_from_msg
        self.saver = saver
        self.clock = clock
        self.max_pair_dt_s = float(max_pair_dt_s)
        self._lock = threading.Lock()
        self._front: tuple[dict, float] | None = None
        self._down: tuple[dict, float] | None = None
        self.log: deque = deque(maxlen=max_log)
        self.counts: Counter = Counter()

    def put_front(self, msg) -> None:
        with self._lock:
            self._front = (msg, self.clock())

    def put_down(self, msg) -> None:
        with self._lock:
            self._down = (msg, self.clock())

    def latest_msg(self, which: str):
        with self._lock:
            got = self._front if which == "front" else self._down
        return None if got is None else got[0]

    def capture(self) -> Capture | None:
        import hashlib
        with self._lock:
            f, d = self._front, self._down
        if f is None:
            self.counts["no_front"] += 1
            return None
        fmsg, t_arr = f
        stamp = _stamp(fmsg)
        pose = self.obs.pose_at(t_arr, stamp)
        if pose is None:
            self.counts["no_pose"] += 1
            return None
        if d is None:
            self.counts["no_down"] += 1
            return None
        dmsg, d_arr = d
        ds = _stamp(dmsg)
        pair_dt = (abs(ds - stamp) / 1e9 if ds is not None and stamp is not None
                   else abs(d_arr - t_arr))
        if pair_dt > self.max_pair_dt_s:
            self.counts["down_unpaired"] += 1
            return None
        t_cap = self.obs.wall_at_stamp(stamp) if stamp is not None else None
        if t_cap is None or abs(t_cap - t_arr) > 2.0:
            t_cap = t_arr                 # no stamps, or the clocks disagree
        t_cap = min(t_cap, t_arr, self.clock())
        front, down = self.encode(fmsg), self.encode(dmsg)
        if not front or not down:
            self.counts["decode_failed"] += 1
            return None
        cap = Capture(float(t_cap), Pose(float(pose["x"]), float(pose["y"]), float(pose["up"]),
                                         float(pose["yaw"])), front, down)
        if self.saver is not None:
            self.saver.save(front)
            self.saver.save(down)
        self.counts["captures"] += 1
        self.log.append({
            "t_capture": round(cap.t_capture, 4), "t_arrival": round(t_arr, 4),
            "stamp_ns": stamp, "down_stamp_ns": ds, "pair_dt_s": round(pair_dt, 4),
            "pose_src": pose.get("src"), "pose": cap.pose.as_wire(),
            "front_wh": [fmsg.get("width"), fmsg.get("height")],
            "down_wh": [dmsg.get("width"), dmsg.get("height")],
            "front_jpg_sha256": hashlib.sha256(front).hexdigest(),
            "down_jpg_sha256": hashlib.sha256(down).hexdigest()})
        return cap

    def stats(self) -> dict:
        sizes = Counter(tuple(r["front_wh"]) for r in self.log)
        return {"counts": dict(self.counts),
                "front_sizes": {f"{w}x{h}": n for (w, h), n in sizes.items()},
                "pair_dt_s": spread([r["pair_dt_s"] for r in self.log])}


# --------------------------------------------------------------------------- #
# The VLA server: start it, wait for hello, stop it - or attach to one
# --------------------------------------------------------------------------- #
SERVER_SCRIPT = ROOT / "demo" / "vla_server_v2.py"
VLA_REAL_PYTHON = r"C:\Users\natha\.conda\envs\vla-real\python.exe"
# The fidelity option of the RTX 4090 measurement (docs/data/
# aerialvla_serving_4090.json; demo/vla_server_v2.py "THE MODEL OPTIONS"):
# bf16/sdpa/unmerged agrees with the upstream-precision reference on 96 % of
# the probe inputs, p50 486 ms, about 17 GB. The low-VRAM option, nf4/sdpa/
# merged, agrees on 25-33 % (p50 517 ms, about 7.9 GB): 4-bit moves the
# outputs. The server has NO default configuration; this pilot always names
# all three flags.
FIDELITY = {"precision": "bf16", "attn": "sdpa", "merge": False}
LOW_VRAM = {"precision": "nf4", "attn": "sdpa", "merge": True}


def server_argv(python: str, *, precision: str, attn: str, merge: bool,
                host: str = "127.0.0.1", port: int = 8790, log: str | None = None,
                fake_output: list[str] | None = None, fake_delay_s: float = 0.0,
                missing_down: str = "refuse") -> list[str]:
    """The command line of demo/vla_server_v2.py serve. A real model gets
    --precision, --attn and --merge/--no-merge, always explicit (the server
    refuses a run that leaves one out). `fake_output` serves the scripted
    stand-in instead (identity fake:scripted, no weights): a plumbing dry run
    that can never be recorded as a VLA flight. --missing-down refuse: this
    pilot always sends the down frame, so a missing one is a fault, not a
    black tile the model never saw in training."""
    if precision not in ("bf16", "nf4"):
        raise ValueError(f"precision {precision!r} is not bf16 or nf4")
    if attn not in ("sdpa", "eager"):
        raise ValueError(f"attn {attn!r} is not sdpa or eager")
    if not isinstance(merge, bool):
        raise ValueError("merge must be True or False")
    argv = [str(python), str(SERVER_SCRIPT), "serve"]
    if fake_output:
        argv += ["--fake-output", *[str(x) for x in fake_output]]
        if fake_delay_s:
            argv += ["--fake-delay-s", f"{float(fake_delay_s):g}"]
    else:
        argv += ["--precision", precision, "--attn", attn,
                 "--merge" if merge else "--no-merge"]
    argv += ["--missing-down", missing_down, "--host", str(host), "--port", str(int(port))]
    if log:
        argv += ["--log", str(log)]
    return argv


def parse_addr(s: str) -> tuple[str, int]:
    """'host:port' -> (host, port)."""
    host, sep, port = str(s).rpartition(":")
    if not sep or not host or not port.isdigit():
        raise ValueError(f"{s!r} is not host:port")
    return host, int(port)


def port_open(host: str, port: int, timeout_s: float = 0.5) -> bool:
    try:
        with socket.create_connection((host, int(port)), timeout=timeout_s):
            return True
    except OSError:
        return False


class ServerExited(RuntimeError):
    """The server process ended before it said hello."""


class VLAServer:
    """One VLA server, started here (`argv`) or attached (`argv` None).

    start(): refuses when something already listens on the port (a hello
    from ANOTHER server would otherwise be taken for this one's), then
    starts the process with its output in `log_path`.
    wait_hello(timeout_s): polls a connect + hello until the model answers;
    raises ServerExited (with the log's tail) if the process ends first, and
    TimeoutError at the timeout. Returns hello's model object (identity +
    serving facts + health).
    alive: the process runs (always True when attached: the link says).
    stop(): op shutdown, then terminate, then kill; never raises."""

    def __init__(self, host: str, port: int, argv: list[str] | None = None,
                 log_path: str | Path | None = None, cwd: str | Path = ROOT):
        self.host, self.port = host, int(port)
        self.argv = list(argv) if argv is not None else None
        self.log_path = Path(log_path) if log_path else None
        self.cwd = Path(cwd)
        self.proc: subprocess.Popen | None = None
        self._log_fh = None
        self.t_start: float | None = None
        self.t_hello: float | None = None
        self.hello: dict | None = None
        self.exit_code: int | None = None
        self.stop_how: str | None = None

    @property
    def started_here(self) -> bool:
        return self.argv is not None

    def start(self) -> None:
        if not self.started_here:
            return
        if port_open(self.host, self.port):
            raise RuntimeError(f"something already listens on {self.host}:{self.port}: attach "
                               f"to it with --vla-server {self.host}:{self.port}, or stop it")
        if self.log_path is not None:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log_fh = open(self.log_path, "ab")
        flags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0) if os.name == "nt" else 0
        self.t_start = time.time()
        self.proc = subprocess.Popen(self.argv, cwd=str(self.cwd), stdin=subprocess.DEVNULL,
                                     stdout=self._log_fh or subprocess.DEVNULL,
                                     stderr=subprocess.STDOUT, creationflags=flags)

    @property
    def alive(self) -> bool:
        if self.proc is None:
            return True
        code = self.proc.poll()
        if code is not None:
            self.exit_code = code
            return False
        return True

    def log_tail(self, n: int = 20) -> str:
        if self.log_path is None or not self.log_path.is_file():
            return ""
        try:
            lines = self.log_path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            return ""
        return "\n".join(lines[-n:])

    def wait_hello(self, timeout_s: float = 900.0, poll_s: float = 2.0,
                   strict_identity: bool = False, on_wait: Callable[[float], None] | None = None,
                   sleep: Callable[[float], None] = time.sleep) -> dict:
        t0 = time.time()
        last_err = None
        while True:
            if self.proc is not None and not self.alive:
                raise ServerExited(f"the VLA server exited with code {self.exit_code} before "
                                   f"it said hello; its log's tail:\n{self.log_tail()}")
            probe = RemoteVLA(self.host, self.port, connect_timeout_s=1.0,
                              response_timeout_s=10.0, strict_identity=strict_identity)
            try:
                model = probe.connect()
                self.hello, self.t_hello = model, time.time()
                return dict(model)
            except (LinkError, ProtocolError, OSError) as exc:
                last_err = f"{type(exc).__name__}: {exc}"
            finally:
                probe.close()
            waited = time.time() - t0
            if waited >= timeout_s:
                raise TimeoutError(f"no hello from {self.host}:{self.port} within "
                                   f"{timeout_s:g} s (last: {last_err})")
            if on_wait is not None:
                on_wait(waited)
            sleep(poll_s)

    def stop(self, timeout_s: float = 15.0) -> dict:
        if self.proc is None:
            return self.record()
        try:
            if self.proc.poll() is None:
                RemoteVLA(self.host, self.port, connect_timeout_s=1.0, response_timeout_s=3.0,
                          strict_identity=False).shutdown_server()
                try:
                    self.proc.wait(timeout=timeout_s)
                    self.stop_how = "shutdown"
                except subprocess.TimeoutExpired:
                    self.proc.terminate()
                    try:
                        self.proc.wait(timeout=5.0)
                        self.stop_how = "terminate"
                    except subprocess.TimeoutExpired:
                        self.proc.kill()
                        self.proc.wait(timeout=5.0)
                        self.stop_how = "kill"
            else:
                self.stop_how = self.stop_how or "had exited"
            self.exit_code = self.proc.poll()
        except Exception as exc:                                 # noqa: BLE001
            self.stop_how = f"stop failed: {type(exc).__name__}: {exc}"
        finally:
            if self._log_fh is not None:
                try:
                    self._log_fh.close()
                except OSError:
                    pass
                self._log_fh = None
        return self.record()

    def record(self) -> dict:
        h = self.hello or {}
        return {"mode": "started" if self.started_here else "attached",
                "addr": f"{self.host}:{self.port}",
                "argv": self.argv, "log": None if self.log_path is None else self.log_path.name,
                "hello_wait_s": (None if self.t_hello is None or self.t_start is None
                                 else round(self.t_hello - self.t_start, 1)),
                "exit_code": self.exit_code, "stop": self.stop_how,
                "identity": {k: h.get(k) for k in ("id", "base", "weights_sha256", "precision",
                                                    "attn", "merged", "agreement")},
                "max_new_tokens": h.get("max_new_tokens"),
                "libraries": h.get("libraries"), "weight_files": h.get("weight_files")}


def identity_is_vla(identity: dict | None) -> tuple[bool, str | None]:
    """(a real AerialVLA identity?, why not). The brief's six fields
    (vla_link.check_identity) and a weights digest that is not one repeated
    digit (tools/todo.py weights_digest_ok: the test stand-ins report
    "0" * 64). The scripted stand-in (id fake:scripted, digest None) is not."""
    try:
        check_identity(identity)
    except ValueError as exc:
        return False, str(exc)
    d = identity["weights_sha256"]
    if len(set(d)) < 2:
        return False, "weights_sha256 is one repeated digit: a stand-in, not a model"
    return True, None


def local_weight_paths(identity: dict | None) -> list[str] | None:
    """The weight files the server hashed (hello's `weight_files`), when
    every one is readable here: then the manifest's digest is checked
    against the server's report (guardrail/manifest.resolve_vla_hash "local;
    reported agrees" or a recorded mismatch). None otherwise (the report
    stands alone, "reported")."""
    files = (identity or {}).get("weight_files")
    if not isinstance(files, list) or not files:
        return None
    paths = [str(f) for f in files]
    return paths if all(Path(p).exists() for p in paths) else None


# --------------------------------------------------------------------------- #
# Records: metrics.pilot, metrics.json, manifest.json, kpi.json
# --------------------------------------------------------------------------- #
def _pct(vals: list[float], q: float) -> float | None:
    if not vals:
        return None
    s = sorted(vals)
    k = (len(s) - 1) * q / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return round(s[lo] + (s[hi] - s[lo]) * (k - lo), 4)


def spread(vals) -> dict:
    v = [float(x) for x in vals if x is not None]
    return {"n": len(v), "median": _pct(v, 50), "p95": _pct(v, 95),
            "max": None if not v else round(max(v), 4)}


def pilot_summary(stats: dict, records: list[dict], rows: list[dict]) -> dict:
    """The numbers a reader asks of a VLA-flown run, from the backend's own
    counts (RemoteVLABackend.stats), its vla_steps records and the rows:
    decision rate (answers the model PARSED that the tracker took), latency
    p50/p95 (capture -> answer on the client clock, the server's model time,
    the round trip), parse failures, server errors, link failures, LAND, and
    who authored the ticks. `vla_authored_tick_share` counts vla_step ticks
    and the holds of a parsed decision (step_complete, land) - never a held
    parse failure, a hover or a none."""
    answered = [r for r in records if r.get("kind") in ("step", "land", "parse_fail")]
    lat_c = [r.get("latency_client_s") for r in answered]
    lat_s = [r.get("latency_server_s") for r in answered]
    rtt = [r.get("rtt_s") for r in answered]
    n = len(rows)
    by_author = Counter(r.get("author") for r in rows)
    vla_step = by_author.get("vla_step", 0)
    held_parsed = sum(1 for r in rows if r.get("author") == "hold"
                      and r.get("author_reason") in ("step_complete", "land"))
    kinds = Counter(r.get("kind") for r in records)
    phrases = Counter(r.get("phrase") for r in records if "phrase" in r)
    sources = Counter(r.get("hint_source") for r in records if "hint_source" in r)
    return {
        "decisions_accepted": stats.get("decisions_accepted"),
        "decision_hz": stats.get("decision_hz"),
        "decision_hz_note": ("answers the model parsed (a step or LAND) that the step "
                             "tracker took, over the time act() ran; the action stream "
                             "is 10 Hz, the decisions behind it are not"),
        "requests": len(records),
        "answers_by_kind": dict(kinds),
        "latency_client_s": spread(lat_c), "latency_server_s": spread(lat_s),
        "rtt_s": spread(rtt),
        "parse_fail": stats.get("parse_fail"), "server_error": stats.get("server_error"),
        "link_failure": stats.get("link_failure"),
        "land": sum(1 for r in records if r.get("kind") == "land"),
        "land_accepted": sum(1 for r in records if r.get("kind") == "land" and r.get("accepted")),
        "ticks": n, "ticks_by_author": dict(by_author),
        "vla_step_tick_share": round(vla_step / n, 4) if n else None,
        "vla_authored_tick_share": round((vla_step + held_parsed) / n, 4) if n else None,
        "phrase_counts": dict(phrases), "hint_source_counts": dict(sources),
        "distinct_phrases": len([p for p in phrases if p is not None]),
    }


def pilot_entry(identity: dict | None, stats: dict, records: list[dict], rows: list[dict], *,
                server: dict, reported_by: str, weight_paths: list[str] | None = None,
                **extra) -> dict:
    """metrics["pilot"]: guardrail.manifest.pilot_record for the model that
    answered, with the backend's stats under "vla" (the shape tools/todo.py
    _pilot_stats reads: "pilot/vla" with decisions_accepted and
    ticks_by_author), the summary, the server record and `extra`.

    A real AerialVLA identity (identity_is_vla) is kind "vla" with the
    server's weights_sha256 as `reported_digest` (and the local weight files
    when they are readable here, so the two digests are compared). Anything
    else - the scripted stand-in - is kind "stub" under its own id, which
    the dashboard never counts as a VLA."""
    from guardrail.manifest import pilot_record
    ok, why = identity_is_vla(identity)
    if ok:
        rec = pilot_record(MODEL_ID, "vla", weight_paths=weight_paths,
                           reported_digest=identity["weights_sha256"], reported_by=reported_by)
    else:
        sid = str((identity or {}).get("id") or "unknown")
        rec = pilot_record(sid if sid.startswith("fake:") else f"fake:{sid}", "stub")
        rec["not_a_vla_because"] = why
    rec["server_identity"] = {k: (identity or {}).get(k) for k in
                              ("id", "base", "weights_sha256", "precision", "attn", "merged",
                               "agreement")}
    rec["vla"] = stats
    rec["summary"] = pilot_summary(stats, records, rows)
    rec["server"] = server
    rec.update(extra)
    return rec


def manifest_for(policy_hash: str, identity: dict | None, seed: int, scene_path: str | Path,
                 weight_paths: list[str] | None = None) -> dict:
    """The six-field manifest: vla_model_hash from the server's reported
    digest (guardrail/manifest.resolve_vla_hash), or the stand-in's id."""
    from guardrail.manifest import build_manifest
    ok, _ = identity_is_vla(identity)
    if ok:
        return build_manifest(policy_hash, MODEL_ID, seed, scene_path=scene_path,
                              weight_paths=weight_paths,
                              reported_digest=identity["weights_sha256"])
    sid = str((identity or {}).get("id") or "unknown")
    return build_manifest(policy_hash, sid if sid.startswith("fake:") else f"fake:{sid}", seed,
                          scene_path=scene_path)


def write_jsonl(path: Path, rows) -> None:
    """LF line endings on every platform (a CRLF file would be a whole-file
    diff for anything that compares records)."""
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        for r in rows:
            fh.write(json.dumps(r, ensure_ascii=False) + "\n")


def write_json(path: Path, obj) -> None:
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps(obj, indent=1, ensure_ascii=False))


def finish_run(out: str | Path, rows: list[dict], metrics: dict, manifest: dict,
               policy) -> dict:
    """Write metrics.json, manifest.json and kpi.json (guardrail/kpi.py over
    the rows, graded by guardrail/manifest.is_kpi_grade), as every rail does,
    and return the KPIs. flight_log.jsonl is written by the flight first (a
    teardown that hangs must not cost it); here only if it is missing."""
    from guardrail import kpi as kpi_mod
    from guardrail.manifest import is_kpi_grade
    out = Path(out)
    if not (out / "flight_log.jsonl").exists():
        write_jsonl(out / "flight_log.jsonl", rows)
    write_json(out / "metrics.json", metrics)
    write_json(out / "manifest.json", manifest)
    kpis = kpi_mod.compute(rows, kpi_mod.rule_priorities(policy), metrics)
    graded, reasons = is_kpi_grade(manifest, metrics)
    kpis.update({"kpi_grade": graded, "kpi_grade_reasons": reasons, "manifest": manifest,
                 "policy_source": metrics.get("policy_source"),
                 "shield_lookahead": metrics.get("shield_lookahead"),
                 "aborted": metrics.get("aborted"), "end_reason": metrics.get("end_reason")})
    write_json(out / "kpi.json", kpis)
    metrics["kpi_grade"] = graded
    metrics["p0_violation_escape_rate"] = kpis.get("p0_violation_escape_rate")
    write_json(out / "metrics.json", metrics)
    return kpis


def mission_scores(mission: str, rows: list[dict], *, goal_xy=None, arrive_radius_m: float,
                   truth_xy=None, approach_radius_m: float = 10.0) -> dict:
    """reached_goal and the distances behind it. M1: the goal (the commanded
    input itself, not ground truth of anything). M2: the parked car's truth
    position, for SCORING only (it never reached the hint or the stop rule);
    None without one."""
    out: dict[str, Any] = {"reached_goal": None}
    if not rows:
        return out
    ref = goal_xy if mission == "goto" else truth_xy
    if ref is None:
        return out
    d = [math.hypot(r["x"] - ref[0], r["y"] - ref[1]) for r in rows]
    radius = arrive_radius_m if mission == "goto" else approach_radius_m
    key = "goal" if mission == "goto" else "target_truth"
    out.update({f"{key}_dist_start_m": round(d[0], 2), f"{key}_dist_end_m": round(d[-1], 2),
                f"{key}_dist_min_m": round(min(d), 2), "reached_radius_m": radius,
                "reached_goal": bool(d[-1] <= radius)})
    return out


def land_record(rows: list[dict], records: list[dict], goal_xy=None) -> dict:
    """What happened at a LAND: where, under which phrase (the decision's own
    phrase from its vla_steps record), and how far from the goal - the
    evidence for "LAND only near the goal or without a hint"."""
    lands = [r for r in records if r.get("kind") == "land" and r.get("accepted")]
    if not lands:
        return {"requested_by_vla": False}
    rec = lands[0]
    pose = rec.get("pose_capture") or {}
    out = {"requested_by_vla": True, "seq": rec.get("seq"), "phrase": rec.get("phrase"),
           "hint_source": rec.get("hint_source"),
           "with_hint": rec.get("phrase") not in (None, NO_HINT),
           "pose_capture": pose}
    if goal_xy is not None and pose:
        out["dist_to_goal_m"] = round(math.hypot(pose["x"] - goal_xy[0], pose["y"] - goal_xy[1]), 2)
    t_land = next((r["t"] for r in rows if r.get("land_requested")), None)
    out["t_s"] = t_land
    return out


def loop_rate(rows: list[dict]) -> float | None:
    if len(rows) < 2:
        return None
    span = rows[-1]["t"] - rows[0]["t"]
    return round((len(rows) - 1) / span, 2) if span > 0 else None


def median_or_none(vals) -> float | None:
    v = [x for x in vals if x is not None]
    return round(statistics.median(v), 3) if v else None
