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

from guardrail import aerialvla_codec as C                           # noqa: E402
from guardrail.models import Action4D, State                         # noqa: E402
from guardrail.vla_link import (                                      # noqa: E402
    NO_HINT, Capture, Hint, Pose, RemoteVLA, LinkError, ProtocolError, build_infer_fields,
    check_identity, goal_hint, phrase_for_bearing, wrap_pi,
)

MISSIONS = ("goto", "approach", "follow")
# The missions whose phrase comes from OWL-ViT's box (vla_bridge): an approach
# ends when the box is large; a follow keeps chasing a moving subject (the
# level's car) until the time limit or the subject is lost.
DETECTOR_MISSIONS = ("approach", "follow")

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
# M1's prompt object words. Every one of upstream's 410,450 training prompts
# carries a description of the target in this style ("The man is walking on a
# road near some bushes and tall evergreen trees, ...", D:/models/
# aerialvla-lora/aerovla_train_dataset.json), never an empty slot, so a GOTO
# sends one FIXED, neutral description of the street instead of "" (review of
# the CityLife flight, P3-14). It names no object that is there to be found
# and no position (aerialvla_codec.validate_object): the words are inert
# (docs/FINDING-what-drives-aerialvla.md), recorded in metrics.json and in
# every vla_steps.jsonl line (`object`), and --object overrides them.
GOTO_OBJECT_TEXT = ("The target is on a paved city street lined with tall buildings, "
                    "street trees and parked cars.")
# The altitude band the step tracker clamps into when the Shield is OFF (the
# control arm, review of the CityLife flight, P1-2): with the Shield on it is
# the policy's band (6-14 m here), which would quietly keep the "raw" arm
# inside the very envelope the Shield enforces. Off, the tracker is built
# without the policy and clamps only into this simulator safety band - above
# the ground and the CityLife street furniture, below anything the level's
# 2 m map was built for - recorded in metrics.json and kpi.json
# (`pilot_alt_band`; the manifest keeps the grant's six fields only). Ours,
# not upstream's: upstream flies unclamped.
SIM_SAFETY_BAND_M = (2.0, 40.0)
# How long the flight and the replay gate wait for the server's hello: a bf16
# cold load under RAM pressure was measured at more than 15 min (2026-10-23).
HELLO_TIMEOUT_S = 2700.0
# The warm-up gate before arming (review of the CityLife flight, P2-5): one
# inference on a live frame pair; slower than this round trip, the flight is
# refused before take-off. 5 s is ours: the step tracker's decision timeout is
# 10 s, and an answer that takes half of it leaves a 5 m step no time.
WARMUP_MAX_LATENCY_S = 5.0
# Warm-up requests before arming; the last is judged (warmup_series_verdict).
WARMUP_REQUESTS = 3
# After a request times out, how long the client waits for its late answer
# before it asks again (RemoteVLA late_reply_wait_s): the server cannot cancel
# a generate() in progress, so a new request would only queue behind it.
LATE_REPLY_WAIT_S = 120.0


def bearing_to(pose: Pose, xy: tuple[float, float]) -> float:
    """Body-frame bearing of a world point from `pose`, radians, + = right of
    the nose (the guardrail frame: x North, y East, yaw clockwise from North)."""
    return wrap_pi(math.atan2(float(xy[1]) - pose.y, float(xy[0]) - pose.x) - pose.yaw_rad)


def distance_to(pose: Pose, xy: tuple[float, float]) -> float:
    return math.hypot(float(xy[0]) - pose.x, float(xy[1]) - pose.y)


def pilot_alt_config(shield_on: bool, policy) -> tuple[Any, tuple[float, float] | None, dict]:
    """(the policy RemoteVLABackend clamps altitude against, the
    TrackerConfig.alt_band, what to record). Shield on: the policy's hard
    envelopes, as every rail. Shield OFF (the control arm): no policy, and
    the explicit SIM_SAFETY_BAND_M - so the arm that is meant to show what
    the VLA flies without the Shield is not held inside the Shield's own
    altitude band by the pilot side (review of the CityLife flight, P1-2).
    The backend's alt_band_info then says which band clamped (`source`
    "configured" for the safety band, "policy" for the policy's)."""
    if shield_on:
        return policy, None, {"clamp": "policy",
                              "why": "the Shield is on: the step tracker clamps altitude "
                                     "targets into the policy's hard above-ground band"}
    return None, SIM_SAFETY_BAND_M, {
        "clamp": "sim_safety_band", "band_m": list(SIM_SAFETY_BAND_M),
        "why": ("the Shield is off (control arm): the step tracker is built without the "
                "policy and clamps altitude targets only into the simulator safety band "
                f"{SIM_SAFETY_BAND_M[0]:g}-{SIM_SAFETY_BAND_M[1]:g} m (vla_mission."
                "SIM_SAFETY_BAND_M); the policy band is the Shield's to judge, in shadow")}


def hud_decision(d, raw: Action4D, *, shield_on: bool):
    """The decision the policy indicator (demo/policy_hud.PolicyIndicator)
    and its counts read - demo/pas_ardupilot/perception_node.hud_decision's
    rule. Shield on: the Shield's own decision. Shield OFF: the same checks
    with NO repair and no brake, the raw action that flew as the one
    emitted, and the shadow decision's violations as flown, so the panel
    shows WATCH and the P0 escapes, never an ACTING row or "Shield
    corrections" for a Shield that was not in the loop (review of the
    CityLife flight, P1-1)."""
    if shield_on:
        return d
    from types import SimpleNamespace
    return SimpleNamespace(violations=list(d.violations), repairs=[], braked=False,
                           touched=False, emitted=raw,
                           emitted_violations=list(d.violations))


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
        if h.kind is not None:
            self.last["kind"] = h.kind
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


def unpack_box(got) -> tuple[Any, float | None, float | None, float | None]:
    """A detector source's answer -> (det, t_box, yaw_box, t_published).
    `t_box`: the wall time the box's frame was captured; `yaw_box`: the
    aircraft's heading (rad) at that frame; `t_published`: when the
    detector published the box (its latency is t_published - t_box). Two-
    tuples (det, t_box) mean no heading and no publish time."""
    got = tuple(got) if got is not None else (None, None)
    det, t_box = got[0], got[1]
    yaw = got[2] if len(got) > 2 else None
    t_pub = got[3] if len(got) > 3 else None
    return det, t_box, yaw, t_pub


# The kinds of hint a follow sends (Hint.kind): a fresh box's bearing, the
# remembered sighting's, the search turn. "close" and None send nothing.
HINT_KINDS = ("fresh", "memory", "search")


@dataclass(frozen=True)
class HintStep:
    """One decision of the follow's hint rule (follow_hint_step).

    kind     "fresh" / "memory" / "search": a request with `phrase`;
             "close": a fresh close box, no request; None: no request.
    box      the detector's box at the capture: "fresh", "stale" or "none".
    phrase   the compass phrase (VLABridge.phrase_for_bearing of
             bearing_rad) when kind is one of HINT_KINDS, else None.
    bearing_rad  the body-frame bearing the phrase was (or, close, would
             have been) looked up with: the box's, rotated by the turn since
             its frame; the sighting's re-read against the heading; the
             search's quarter turn. None when nothing was looked up.
    seen     the remembered sighting after this step: (world bearing, wall
             time of its frame), or None.
    age_s    the box's age at the capture (None without a box).
    measured_bearing_rad, yaw_correction_rad  a fresh box's bearing on its
             own frame and the turn since (None otherwise, and the turn
             None too when the frame's heading is unknown).
    since_seen_s, seen_rel_rad  memory / search: the time since the
             sighting's frame and its bearing relative to the current
             heading (None otherwise)."""
    kind: str | None
    box: str
    phrase: str | None = None
    bearing_rad: float | None = None
    seen: tuple[float, float] | None = None
    age_s: float | None = None
    measured_bearing_rad: float | None = None
    yaw_correction_rad: float | None = None
    since_seen_s: float | None = None
    seen_rel_rad: float | None = None


def follow_hint_step(box, t_capture: float, yaw_capture: float,
                     seen: tuple[float, float] | None, bridge, *, fresh_s: float,
                     memory_s: float = 0.0, search_s: float = 0.0,
                     close: "BoxClose | None" = None) -> HintStep:
    """THE hint rule of a detector mission (DetectorHint's decision, once the
    start gate is open), pure but for the bridge's phrase counter:
    `box` = (det, t_box, yaw_box) as unpack_box gives them (det None: no
    box), the VLA capture's wall time and heading, the sighting remembered
    so far (`seen`, a previous step's), the bridge that turns a bearing into
    a phrase. DetectorHint calls it on every request; tools/
    build_follow_dataset.py calls it over a recording's captures, carrying
    `seen` from one to the next, so the dataset's phrases and kinds are the
    flight's by construction.

      FRESH (age <= fresh_s): the box's bearing (vla_bridge.bearing_from_box)
        rotated by (heading at the capture - heading at its frame) before
        the band lookup (P2-6, DetectorHint's docstring); the sighting is
        remembered in the world frame (only when the frame's heading is
        known). A close box (`close`, BoxClose) is remembered too, and sends
        nothing ("close").
      NO FRESH BOX: with a sighting and memory_s or search_s set, its world
        bearing re-read against the current heading while the time since
        its frame is at most memory_s ("memory"); then, up to memory_s +
        search_s, a quarter turn toward the side that re-read gives now
        ("search"); else nothing. Both 0: M2's rule (no fresh box, no
        request)."""
    from vla_bridge import bearing_from_box
    det, t_box, yaw_box = box
    t_capture, yaw_capture = float(t_capture), float(yaw_capture)
    age = None if det is None or t_box is None else t_capture - float(t_box)
    if age is None or age > fresh_s:
        state = "none" if age is None else "stale"
        if seen is None or (memory_s <= 0 and search_s <= 0):
            return HintStep(None, state, seen=seen, age_s=age)
        world, t_seen = seen
        since = t_capture - t_seen
        rel = wrap_pi(world - yaw_capture)
        if since <= memory_s:
            kind, bearing = "memory", rel
        elif since <= memory_s + search_s:
            # Toward the side it was last seen on: a quarter turn's phrase
            # ("to your right " / "to your left "), the band AerialVLA turns on.
            kind, bearing = "search", (math.pi / 2 if rel >= 0 else -math.pi / 2)
        else:
            return HintStep(None, state, seen=seen, age_s=age)
        return HintStep(kind, state, bridge.phrase_for_bearing(bearing), bearing, seen, age,
                        since_seen_s=since, seen_rel_rad=rel)
    measured = bearing = bearing_from_box(det, bridge.hfov_deg)
    corr = None
    if yaw_box is not None:
        corr = wrap_pi(yaw_capture - float(yaw_box))
        bearing -= corr
        seen = (wrap_pi(float(yaw_box) + bearing + corr), float(t_box))
    if close is not None and close(det):
        return HintStep("close", "fresh", None, bearing, seen, age, measured, corr)
    return HintStep("fresh", "fresh", bridge.phrase_for_bearing(bearing), bearing, seen, age,
                    measured, corr)


class DetectorHint:
    """M2: the compass phrase of the newest FRESH OWL-ViT box, through
    demo/vla_bridge.VLABridge (box centre -> pinhole bearing -> the training
    band). `latest()` -> (det, t_box[, yaw_box[, t_published]])
    (unpack_box): the Grounder tuple (cx, cy, w, h, score, W, H, colour),
    the wall time its frame was captured, the heading at that frame and when
    the detector published it; or (None, None). A box older than `fresh_s`
    at the VLA capture, or none at all, is NO REQUEST (None): the bridge
    never emits "", and a detector hint of "" is refused by the codec,
    because "" asks the model to LAND wherever the box was lost. 1.0 s is
    DETECTION_STALE_S of the ArduPilot rail (perception_node.py).

    THE TURN SINCE THE BOX (review of the CityLife flight, P2-6). The box's
    bearing is measured on the detector's frame; the VLA's capture can be up
    to `fresh_s` later, and at the tracker's 0.5 rad/s yaw cap the aircraft
    can have turned 28.6 deg in that second. So the bearing is rotated by
    (heading at the VLA capture - heading at the detector frame) before the
    band lookup (VLABridge.phrase_for_bearing: gain and clamp unchanged).
    Without the frame's heading the bearing is used as measured, counted
    `n_no_yaw`. Kept for the rows and stats: the box's age at the hint, the
    correction, and the detector's own latency (publish - frame capture)."""

    def __init__(self, latest: Callable[[], tuple], bridge, fresh_s: float = 1.0,
                 memory_s: float = 0.0, search_s: float = 0.0,
                 close: "BoxClose | None" = None,
                 start_gate: tuple[int, float] | None = None,
                 record_kind: bool = False):
        if not fresh_s > 0:
            raise ValueError("fresh_s must be positive")
        if not (memory_s >= 0 and search_s >= 0):
            raise ValueError("memory_s and search_s must not be negative")
        self.latest = latest
        self.bridge = bridge
        self.fresh_s = float(fresh_s)
        # FOLLOW (review of d6e7123, P2-2): each hint says how it was made -
        # "fresh", "memory" or "search" - as Hint.kind, which the request's
        # vla_steps record and the rows carry (never the wire). Off for an
        # approach and the ArduPilot rail, whose records stay as they were.
        self.record_kind = bool(record_kind)
        # A perception reset (forget(t_reset); review of d6e7123, P2-4):
        # boxes whose frame was captured at or before this are no boxes.
        self._ignore_until: float | None = None
        self.n_ignored = 0
        # FOLLOW (2026-10-23): past fresh_s the last sighting's bearing, held
        # in the world frame and re-read against the current heading, until
        # memory_s ("memory"); then a turn toward the side it was last seen
        # on, until memory_s + search_s ("search"). Both 0: M2's rule (no
        # fresh box, no request). The protocol's hint_source stays
        # "detector": the phrase still comes only from the detector's boxes.
        self.memory_s = float(memory_s)
        self.search_s = float(search_s)
        self.close = close
        # FOLLOW's start gate (as follow_vlm's --start-when-seen): no request
        # until `n` successive detector frames gave a fresh box at least
        # `width_frac` of the image wide (the subject near), so the chase
        # starts close instead of 40 m behind a car already pulling away.
        self.n_gated = 0
        self.set_start_gate(start_gate)
        self._seen: tuple[float, float] | None = None     # (world bearing, t of its frame)
        self.last_kind: str | None = None
        # The last call's follow_hint_step (None when gated or not yet called).
        self.last_step: HintStep | None = None
        self.kinds: Counter = Counter()
        self.n_close = 0
        self.n_stale = 0
        self.n_none = 0
        self.n_no_yaw = 0
        self.n_hints = 0
        self.last_box_age_s: float | None = None
        self.last_yaw_correction_rad: float | None = None
        self.last_detector_latency_s: float | None = None
        self._ages: deque = deque(maxlen=20000)
        self._corr: deque = deque(maxlen=20000)
        self._lat: deque = deque(maxlen=20000)

    def set_start_gate(self, start_gate: tuple[int, float] | None) -> None:
        """(n, width_frac), or None for no gate. Settable after construction
        so a follow arms the gate only once it knows which one applies -
        after the identity setup, which may fail (review of d6e7123, P2-5:
        the gate used to be chosen from --no-identity before that setup ran,
        and a follow whose identity failed to load flew with none). Meant
        for before the first hint: it (re)starts the gate closed."""
        if start_gate is not None:
            n, wf = start_gate
            if not (int(n) >= 1 and 0 < float(wf) < 1):
                raise ValueError("start_gate is (n >= 1, 0 < width_frac < 1)")
            start_gate = (int(n), float(wf))
        self.start_gate = start_gate
        self.gate_open = start_gate is None
        self.gate_open_t: float | None = None
        self._gate_streak = 0
        self._gate_last_t: float | None = None

    def box_ignored(self, t_box) -> bool:
        """Is a box whose frame was captured at `t_box` one a perception reset
        dropped (forget(t_reset): captured at or before t_reset)?"""
        return (self._ignore_until is not None and t_box is not None
                and float(t_box) <= self._ignore_until)

    def __call__(self, capture: Capture) -> Hint | None:
        det, t_box, yaw_box, t_pub = unpack_box(self.latest())
        self.last_yaw_correction_rad = None
        self.last_detector_latency_s = None
        self.last_kind = None
        self.last_step = None
        if det is not None and self.box_ignored(t_box):
            # The box a reset dropped, still published (fresh for fresh_s,
            # kept by the Grounder up to its drop_after): no box at all.
            self.n_ignored += 1
            det = t_box = None
        if det is None or t_box is None:
            self.n_none += 1
            self.last_box_age_s = None
        else:
            self.last_box_age_s = float(capture.t_capture) - float(t_box)
            if t_pub is not None:
                self.last_detector_latency_s = float(t_pub) - float(t_box)
            if self.last_box_age_s > self.fresh_s:
                self.n_stale += 1
        if not self.gate_open and not self._gate_opens(det, t_box, capture):
            return self._gated()
        # The decision itself - fresh, close, memory, search or nothing, the
        # phrase and the sighting kept - is follow_hint_step's, the one rule
        # tools/build_follow_dataset.py emulates over a recording.
        step = follow_hint_step((det, t_box, yaw_box), capture.t_capture, capture.pose.yaw_rad,
                                self._seen, self.bridge, fresh_s=self.fresh_s,
                                memory_s=self.memory_s, search_s=self.search_s, close=self.close)
        self.last_step = step
        self._seen = step.seen
        if step.box == "fresh":
            if yaw_box is None:
                self.n_no_yaw += 1
            else:
                self.last_yaw_correction_rad = step.yaw_correction_rad
        if step.kind == "close":
            # The subject is close (leaving the view below, or large): no new
            # step toward it - the rail holds the horizontal motion meanwhile
            # (MissionLoop, follow).
            self.n_close += 1
            self.last_kind = "close"
            return None
        if step.kind is None:
            return None
        self.n_hints += 1
        self.last_kind = step.kind
        self.kinds[step.kind] += 1
        if step.kind == "fresh":
            self._ages.append(step.age_s)
            if self.last_yaw_correction_rad is not None:
                self._corr.append(abs(math.degrees(self.last_yaw_correction_rad)))
            if self.last_detector_latency_s is not None:
                self._lat.append(self.last_detector_latency_s)
        return self._hint(step.phrase, step.kind)

    def _gate_opens(self, det, t_box, capture: Capture) -> bool:
        """The start gate, while closed: does this call open it? No box or a
        stale one restarts the streak; a fresh box at least width_frac wide
        from a NEW detector frame extends it; n in a row open the gate."""
        if det is None or t_box is None or self.last_box_age_s > self.fresh_s:
            self._gate_streak = 0
            return False
        n, wf = self.start_gate
        frac = box_fraction(det)
        if frac is None or frac < wf:
            self._gate_streak = 0
        elif self._gate_last_t != float(t_box):
            self._gate_streak += 1
            self._gate_last_t = float(t_box)
        if self._gate_streak < n:
            return False
        self.gate_open, self.gate_open_t = True, float(capture.t_capture)
        return True

    def _hint(self, phrase: str, kind: str) -> Hint:
        return Hint(phrase, "detector", kind if self.record_kind else None)

    def forget(self, t_reset: float | None = None) -> None:
        """Drop the remembered sighting (a perception reset: StuckWatch,
        CloseStall): no memory or search toward a box that was probably the
        wrong object. With `t_reset` (wall time, the clock of the boxes'
        frame times) every box captured at or before it is ignored from now
        on: until the review of d6e7123 (P2-4) the dropped box stayed fresh
        for fresh_s and the Grounder kept publishing it up to its drop_after
        (8 s), so the next hint re-seeded the memory with it."""
        self._seen = None
        self.kinds["forgotten"] += 1
        if t_reset is not None:
            t = float(t_reset)
            self._ignore_until = t if self._ignore_until is None else max(self._ignore_until, t)

    def _gated(self) -> None:
        self.n_gated += 1
        self.last_kind = "gated"
        return None

    def stats(self, t0: float | None = None) -> dict:
        """The counts and spreads. With `t0` (the mission start, wall time)
        the start gate's opening is given in mission seconds (`t_open_s`;
        review of d6e7123, P3: it was an epoch time), else as the wall time
        (`t_open`), as before."""
        gate = None
        if self.start_gate is not None:
            gate = {"n": self.start_gate[0], "width_frac": self.start_gate[1],
                    "open": self.gate_open, "gated_calls": self.n_gated}
            if t0 is None:
                gate["t_open"] = self.gate_open_t
            else:
                gate["t_open_s"] = (None if self.gate_open_t is None
                                    else round(self.gate_open_t - float(t0), 3))
        out = {"fresh_s": self.fresh_s, "memory_s": self.memory_s, "search_s": self.search_s,
               "hints_by_kind": dict(self.kinds), "close_no_request": self.n_close,
               "start_gate": gate}
        if self.record_kind or self._ignore_until is not None:
            out["ignored_after_reset"] = self.n_ignored
        return {**out,
                "close_rule": None if self.close is None else self.close.as_dict(),
                "hints": self.n_hints, "stale": self.n_stale,
                "no_box": self.n_none, "no_frame_heading": self.n_no_yaw,
                "box_age_at_hint_s": spread(self._ages),
                "yaw_correction_abs_deg": spread(self._corr),
                "detector_latency_s": spread(self._lat),
                "rule": "bearing rotated by (heading at the VLA capture - heading at the "
                        "detector frame) before the band lookup"}


@dataclass(frozen=True)
class BoxClose:
    """FOLLOW: when the subject counts as close - its box's bottom edge at or
    below `bottom_frac` of the image height (it is leaving the view below,
    the aircraft about to fly over it) or its width at least `width_frac`
    of the image. Ours, recorded; not upstream's."""
    bottom_frac: float = 0.92
    width_frac: float = 0.25

    def __call__(self, det) -> bool:
        if det is None:
            return False
        try:
            cy, h, H = float(det[1]), float(det[3]), float(det[6])
        except (TypeError, ValueError, IndexError):
            h = H = 0.0
            cy = 0.0
        frac = box_fraction(det)
        low = H > 0 and (cy + h / 2.0) / H >= self.bottom_frac
        return bool(low or (frac is not None and frac >= self.width_frac))

    def as_dict(self) -> dict:
        return {"bottom_frac": self.bottom_frac, "width_frac": self.width_frac}


class StuckWatch:
    """FOLLOW: the aircraft pinned by the Shield. When over the last
    `window_s` the Shield touched at least `touched_frac` of the ticks and
    the aircraft moved less than `min_progress_m`, the box it is steering on
    is taken to be the wrong one (follow #4, 23 Oct: the lock held something
    beside a building and the Shield blocked 2447 ticks in 225 s): `update`
    returns True once, then waits `cooldown_s`. The caller resets the
    detector's lock and the remembered sighting - perception only; the VLA
    still steers."""

    def __init__(self, window_s: float = 4.0, touched_frac: float = 0.8,
                 min_progress_m: float = 1.0, cooldown_s: float = 6.0):
        self.window_s, self.touched_frac = float(window_s), float(touched_frac)
        self.min_progress_m, self.cooldown_s = float(min_progress_m), float(cooldown_s)
        self._hist: deque = deque()
        self._t_last: float | None = None
        self.resets = 0

    def update(self, t: float, x: float, y: float, touched: bool) -> bool:
        self._hist.append((float(t), float(x), float(y), bool(touched)))
        while self._hist and t - self._hist[0][0] > self.window_s:
            self._hist.popleft()
        if self._t_last is not None and t - self._t_last < self.cooldown_s:
            return False
        if not self._hist or t - self._hist[0][0] < self.window_s * 0.95:
            return False
        frac = sum(1 for h in self._hist if h[3]) / len(self._hist)
        x0, y0 = self._hist[0][1], self._hist[0][2]
        if frac >= self.touched_frac and math.hypot(x - x0, y - y0) < self.min_progress_m:
            self._t_last = t
            self.resets += 1
            self._hist.clear()
            return True
        return False

    def as_dict(self) -> dict:
        return {"window_s": self.window_s, "touched_frac": self.touched_frac,
                "min_progress_m": self.min_progress_m, "cooldown_s": self.cooldown_s,
                "resets": self.resets}


class CloseStall:
    """FOLLOW: a close box and no request for `max_s` (review of d6e7123,
    P2-8). A locked WRONG box that counts as close (BoxClose: >= 0.25 of the
    width, say a bus or a wall) asks for no step (DetectorHint "close"), the
    close hold zeroes any motion, the tracker hovers, the Shield stays quiet
    (StuckWatch never fires) and the box stays fresh (target_lost never
    fires): a hover until max_s. `update(t, close, requests)` - `close`: a
    fresh close box this tick (the row's close_box); `requests`: the
    backend's request count - returns True once when a spell of close ticks
    has lasted max_s with no request sent; a request sent, or a tick that is
    not close, starts the spell again. The caller resets the perception
    (perception only; the VLA still steers). 0 disables."""

    def __init__(self, max_s: float = 8.0):
        if not (math.isfinite(float(max_s)) and float(max_s) >= 0):
            raise ValueError("max_s must be 0 (off) or more")
        self.max_s = float(max_s)
        self._since: float | None = None
        self._req0 = None
        self.stalls = 0

    def update(self, t: float, close: bool, requests) -> bool:
        if self.max_s <= 0 or not close:
            self._since = None
            return False
        if self._since is None or requests != self._req0:
            self._since, self._req0 = float(t), requests
            return False
        if float(t) - self._since >= self.max_s - 1e-9:
            self._since = None
            self.stalls += 1
            return True
        return False

    def as_dict(self) -> dict:
        return {"max_s": self.max_s, "stalls": self.stalls}


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
    "arrived_overshoot": ("was inside the arrive radius, then the goal stayed behind the "
                          "aircraft and receding for overshoot_s (flew past it; no LAND)"),
    "close_box": "the detector's box filled close_box_frac of its frame",
    "target_lost": "no fresh detection for lost_timeout_s",
    "subject_not_acquired": ("a follow that never sent the model a request within "
                             "first_decision_timeout_s: its start gate never opened, or its "
                             "subject was close from the start - nothing was asked, so not "
                             "a VLA failure"),
    "timeout": "the mission time ran out",
    # the VLA failed
    "server_dead": ("no answer for dead_timeout_s while hovering: the link failed "
                    "(connection refused or closed, a broken reply, the server gone)"),
    "server_slow": ("no answer for dead_timeout_s while hovering: every request timed out "
                    "(the model answers slower than the response timeout)"),
    "server_error": "only server errors (ok=false) for dead_timeout_s while hovering",
    "parse_fail_streak": "only malformed answers (held zero) for dead_timeout_s",
    "no_first_decision": "no answer the model parsed within first_decision_timeout_s",
}
VLA_FAILURE_ENDS = ("server_dead", "server_slow", "server_error", "parse_fail_streak",
                    "no_first_decision")
# Why a flight ended without its mission, set by demo/vla_fly.py (never by the
# monitor): refused before take-off or before the mission clock, or stopped by
# the operator.
NOT_FLOWN_ENDS = {
    "warmup_refused": ("the warm-up inference before arming was too slow or not answered: "
                       "refused before take-off"),
    "start_not_reached": ("the hand-written transfer did not bring the aircraft within "
                          "start_tolerance_m of the mission start: refused, landed"),
    "interrupted": "stopped by the operator (Ctrl+C): landed where it was",
}


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
    overshoot_s: float = 3.0             # M1: goal behind and receding this long after arriving

    def __post_init__(self):
        if self.mission not in MISSIONS:
            raise ValueError(f"mission {self.mission!r} is not one of {MISSIONS}")
        for k in ("max_s", "arrive_radius_m", "arrive_grace_s", "close_box_frac",
                  "lost_timeout_s", "det_fresh_s", "dead_timeout_s", "first_decision_timeout_s",
                  "overshoot_s"):
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
    radius for arrive_grace_s, or flown past the goal - see below; M2: the
    box large enough, or no fresh box for lost_timeout_s); the mission time.
    The server process having exited is not an end by itself: its link
    failures make the tracker hover, and the hover rule ends the mission
    dead_timeout_s later (the brief: hover, then land after a timeout), with
    the exit recorded.

    SLOW IS NOT DEAD (review of the CityLife flight, P2-5). A hover on
    link_lost is `server_slow` when the latest link failure was a timeout
    (`link_failure_kind` "timeout": the model is still computing) and
    `server_dead` otherwise (refused, closed, a broken reply). Before the
    first parsed answer a slow model is given first_decision_timeout_s - a
    cold model's first answers are its slowest - and only then ends the
    mission (`no_first_decision`); until 2026-10-23 the dead rule ended a
    slow model's mission after ~20 s, and the first answer's 60 s never
    applied. Every link failure kind seen is counted in the summary.

    OVERSHOOT (M1). A model that does not LAND on the "" phrase and drifts
    out of the arrive radius is then given the rear phrases until the time
    runs out. Once the aircraft has been inside the radius, the goal behind
    it (|bearing| > 90 deg) and its distance growing on every tick for
    overshoot_s ends the mission `arrived_overshoot`, with the distance at
    the end recorded (`overshoot_m`).

    HELD BY THE RAIL (review of 4a0da6d, P3-9). On a rail with an
    escalation FSM (demo/pas_ardupilot/perception_node.VLAPilot) the pilot's
    action does not fly while the FSM, a fault or the autopilot holds the
    aircraft, and the tracker is suspended. `hold(now)` stops every timer
    here (the first decision's, a failure streak's, the arrive grace, the
    overshoot, the box's absence, the mission time) and `unhold(now)`
    shifts them by the time held, so no stop rule ends - and names - a
    mission the rail was holding; the time is in the summary (`held_s`).
    The caller makes no update() while held.

    NOTHING ASKED (review of d6e7123, P2-7). A follow whose start gate never
    let a request out, or whose box was close from the start, has no parsed
    answer either, and no_first_decision called that a VLA failure (exit
    3). A caller that knows how many requests were sent passes
    `requests_sent` (MissionLoop reads RemoteVLABackend.counts); a FOLLOW
    with none sent by first_decision_timeout_s ends `subject_not_acquired`,
    which is not one of VLA_FAILURE_ENDS. Without the count, and for goto
    and approach, the rule is as before."""

    def __init__(self, cfg: MonitorConfig, t0: float):
        self.cfg = cfg
        self.t0 = float(t0)
        # The timers' origin: t0 shifted by the time held (hold / unhold).
        self._t_ref = self.t0
        self._held_since: float | None = None
        self.held_s = 0.0
        self.holds = 0
        self.end_reason: str | None = None
        self.t_end: float | None = None
        self._streak_kind: str | None = None
        self._streak_t0: float | None = None
        self._t_first_decision: float | None = None
        self._t_inside: float | None = None
        self._ever_inside = False
        self._t_recede: float | None = None
        self._d_prev: float | None = None
        self.overshoot_m: float | None = None
        self.min_goal_dist_m: float | None = None
        self._t_last_box: float | None = None
        self.t_server_exit: float | None = None
        self.events: list[dict] = []
        self.ticks_inside = 0
        # follow: ticks with a fresh box at least close_box_frac wide
        self.ticks_close_box = 0
        self.link_failure_kinds: Counter = Counter()
        self._last_failure_key = None
        self.requests_sent: int | None = None    # the latest count a caller passed

    def _end(self, why: str, now: float) -> str:
        self.end_reason, self.t_end = why, now
        self.events.append({"t": round(now - self.t0, 3), "event": "end", "reason": why})
        return why

    def hold(self, now: float) -> None:
        """The rail holds the aircraft from `now`: every timer stops."""
        if self._held_since is None and self.end_reason is None:
            self._held_since = float(now)
            self.holds += 1

    def unhold(self, now: float) -> None:
        """The pilot's action flies again: the timers resume, shifted by the
        time held."""
        if self._held_since is None:
            return
        d = max(0.0, float(now) - self._held_since)
        self._held_since = None
        self.held_s += d
        self._t_ref += d
        for k in ("_streak_t0", "_t_inside", "_t_recede", "_t_last_box"):
            v = getattr(self, k)
            if v is not None:
                setattr(self, k, v + d)

    def update(self, now: float, pose: Pose, tick, *, goal_xy=None, det=None,
               det_age_s: float | None = None, server_alive: bool = True,
               link_failure: dict | None = None,
               requests_sent: int | None = None) -> str | None:
        """`link_failure`: the backend's last_link_failure ({"kind", "seq",
        ...}, RemoteVLABackend), or None. `requests_sent`: how many requests
        the pilot has sent so far, or None when the caller cannot say."""
        if self.end_reason is not None:
            return self.end_reason
        cfg = self.cfg
        if requests_sent is not None:
            self.requests_sent = int(requests_sent)
        if not server_alive and self.t_server_exit is None:
            self.t_server_exit = now
            self.events.append({"t": round(now - self.t0, 3), "event": "server_exited"})
        if link_failure is not None:
            key = (link_failure.get("seq"), link_failure.get("t"))
            if key != self._last_failure_key:
                self._last_failure_key = key
                self.link_failure_kinds[str(link_failure.get("kind"))] += 1
        if tick.land_requested:
            return self._end("vla_land", now)
        if tick.author in ("vla_step", "hold") and tick.seq is not None \
                and tick.reason != "parse_fail" and self._t_first_decision is None:
            self._t_first_decision = now
        # A VLA failure: the tracker has been in one no-answer state since t.
        kind = None
        if tick.author in ("hover", "none") and tick.reason in ("link_lost", "server_error"):
            if tick.reason == "server_error":
                kind = "server_error"
            elif (link_failure or {}).get("kind") == "timeout":
                kind = "server_slow"
            else:
                kind = "server_dead"
        elif tick.author == "hold" and tick.reason == "parse_fail":
            kind = "parse_fail_streak"
        if kind != self._streak_kind:
            self._streak_kind, self._streak_t0 = kind, (now if kind else None)
        # A slow model before its first parsed answer: first_decision_timeout_s.
        graced = kind == "server_slow" and self._t_first_decision is None
        if kind is not None and not graced and now - self._streak_t0 >= cfg.dead_timeout_s:
            return self._end(kind, now)
        if self._t_first_decision is None and now - self._t_ref >= cfg.first_decision_timeout_s:
            if cfg.mission == "follow" and requests_sent == 0:
                return self._end("subject_not_acquired", now)
            return self._end("no_first_decision", now)
        if cfg.mission == "goto":
            if goal_xy is None:
                raise ValueError("a goto mission needs goal_xy")
            d = distance_to(pose, goal_xy)
            self.min_goal_dist_m = d if self.min_goal_dist_m is None else min(self.min_goal_dist_m, d)
            if d <= cfg.arrive_radius_m:
                self.ticks_inside += 1
                self._ever_inside = True
                self._t_recede = None
                if self._t_inside is None:
                    self._t_inside = now
                    self.events.append({"t": round(now - self.t0, 3), "event": "inside_arrive_radius"})
                if now - self._t_inside >= cfg.arrive_grace_s:
                    return self._end("arrived_no_land", now)
            else:
                if self._t_inside is not None:
                    self.events.append({"t": round(now - self.t0, 3), "event": "left_arrive_radius"})
                    self._t_inside = None
                behind = abs(bearing_to(pose, goal_xy)) > math.pi / 2
                receding = self._d_prev is not None and d > self._d_prev + 1e-3
                if self._ever_inside and behind and receding:
                    if self._t_recede is None:
                        self._t_recede = now
                    if now - self._t_recede >= cfg.overshoot_s:
                        self.overshoot_m = round(d, 2)
                        self.events.append({"t": round(now - self.t0, 3), "event": "overshoot",
                                            "dist_m": round(d, 2)})
                        self._d_prev = d
                        return self._end("arrived_overshoot", now)
                else:
                    self._t_recede = None
            self._d_prev = d
        else:
            fresh = det is not None and det_age_s is not None and det_age_s <= cfg.det_fresh_s
            if fresh:
                self._t_last_box = now
                frac = box_fraction(det)
                # A follow chases a moving subject: a large box is the point,
                # not the end (counted, never a stop rule).
                if frac is not None and frac >= cfg.close_box_frac:
                    if cfg.mission == "approach":
                        return self._end("close_box", now)
                    self.ticks_close_box += 1
            since = self._t_last_box if self._t_last_box is not None else self._t_ref
            if now - since >= cfg.lost_timeout_s:
                return self._end("target_lost", now)
        if now - self._t_ref >= cfg.max_s:
            return self._end("timeout", now)
        return None

    def summary(self) -> dict:
        extra = {} if self.requests_sent is None else {"requests_sent": self.requests_sent}
        return {**extra, "end_reason": self.end_reason,
                "end_reason_means": END_REASONS.get(self.end_reason),
                "vla_failure": self.end_reason in VLA_FAILURE_ENDS,
                "t_end_s": None if self.t_end is None else round(self.t_end - self.t0, 3),
                "t_first_decision_s": (None if self._t_first_decision is None
                                       else round(self._t_first_decision - self.t0, 3)),
                "t_server_exit_s": (None if self.t_server_exit is None
                                    else round(self.t_server_exit - self.t0, 3)),
                "ticks_inside_arrive_radius": self.ticks_inside,
                "ticks_close_box": self.ticks_close_box,
                "ever_inside_arrive_radius": self._ever_inside,
                "min_goal_dist_m": (None if self.min_goal_dist_m is None
                                    else round(self.min_goal_dist_m, 2)),
                "overshoot_m": self.overshoot_m,
                # Time the rail held the aircraft, every timer stopped (hold).
                "held_s": round(self.held_s, 3), "holds": self.holds,
                "link_failure_kinds": dict(self.link_failure_kinds),
                "events": list(self.events), "config": self.cfg.as_dict()}


# --------------------------------------------------------------------------- #
# One 10 Hz tick: backend -> Shield -> monitor -> row
# --------------------------------------------------------------------------- #
@dataclass
class LoopTick:
    flown: Action4D          # what to send to the autopilot
    decision: Any            # the ShieldDecision (with the Shield off: its shadow)
    row: dict                # the flight_log.jsonl row
    hud: dict                # what the recorder draws
    end_reason: str | None
    hud_decision: Any = None  # what the policy indicator reads (hud_decision)
    det: Any = None           # the detector box this tick read (the recorder draws it)


def _r(v, n=3):
    return None if v is None else round(float(v), n)


# The per-tick record of the decision behind a tick and of the mission, shared
# by MissionLoop (the CityLife flight) and the ArduPilot rail's VLA pilot
# (demo/pas_ardupilot/perception_node.VLAPilot), so the two rails write the
# same fields from the same code (wave V2b, 2026-10-23).
def current_decision(backend) -> tuple[dict, dict | None]:
    """(the vla_steps record of the decision being flown - a copy, {} when
    none - and its step, or None)."""
    cur = getattr(backend.tracker, "current_record", None)
    cur = cur if isinstance(cur, dict) else {}
    step = cur.get("step") if isinstance(cur.get("step"), dict) else None
    return cur, step


def decision_fields(tk, cur: dict, step: dict | None, *, hint_source=None,
                    link_failure: dict | None = None) -> dict:
    """The row's VLA fields for one tick: who authored it (vla_link.Tick),
    the decision being flown (its seq, image age, phrase, hint source, step,
    bins and raw text, from its vla_steps record `cur`), what a link_lost
    hover failed with (timeout = slow), and the latest hint SENT."""
    last = (hint_source.last or {}) if hint_source is not None else {}
    return {
        # who flew this tick (vla_link.Tick) and the decision behind it
        "author": tk.author, "author_reason": tk.reason,
        "vla_seq": tk.seq, "vla_age_s": _r(tk.age_s),
        "land_requested": bool(tk.land_requested),
        # a hover on link_lost: what the link failed with (timeout = slow)
        "link_failure": ((link_failure or {}).get("kind") if tk.reason == "link_lost" else None),
        "phrase": cur.get("phrase"), "hint_source": cur.get("hint_source"),
        "step": step, "bins": cur.get("bins"), "raw_text": cur.get("raw_text"),
        "hint_sent": last.get("phrase"), "hint_sent_source": last.get("source"),
    }


def goal_and_box(pose: Pose, goal_xy, det) -> tuple[float | None, float | None, float | None]:
    """(goal distance m, goal bearing deg, box width / frame width), unrounded;
    None where there is no goal or no box."""
    gd = gb = None
    if goal_xy is not None:
        gd, gb = distance_to(pose, goal_xy), math.degrees(bearing_to(pose, goal_xy))
    return gd, gb, box_fraction(det)


def mission_fields(gd, gb, frac, det, det_age, detector_hint: DetectorHint | None) -> dict:
    """The row's mission fields: the goal's distance and bearing, the box and
    its age, and what the detector hint corrected (DetectorHint)."""
    return {
        "goal_dist_m": _r(gd), "goal_bearing_deg": _r(gb, 2),
        "det": (None if det is None else
                {"cx": _r(det[0], 1), "cy": _r(det[1], 1), "w": _r(det[2], 1),
                 "score": _r(det[4], 4), "img_w": int(det[5])}),
        "det_age_s": _r(det_age), "box_frac": _r(frac, 4),
        "box_age_at_hint_s": (_r(detector_hint.last_box_age_s)
                              if detector_hint is not None else None),
        "yaw_correction_deg_at_hint": (
            None if detector_hint is None
            or detector_hint.last_yaw_correction_rad is None
            else round(math.degrees(detector_hint.last_yaw_correction_rad), 2)),
        "detector_latency_s": (_r(detector_hint.last_detector_latency_s)
                               if detector_hint is not None else None),
    }


def vla_hud(t: float, alt: float, tk, cur: dict, step: dict | None, *, touched: bool,
            shield_on: bool, mission: str, cadence, goal_dist_m, box_frac,
            end_reason) -> dict:
    """What the VLA panel (hud_lines) draws for one tick."""
    return {"t": t, "alt": alt, "author": tk.author,
            "reason": tk.reason, "seq": tk.seq, "age_s": tk.age_s,
            "phrase": cur.get("phrase"), "hint_source": cur.get("hint_source"),
            "step": step, "land": bool(tk.land_requested), "shield": bool(touched),
            "shield_on": shield_on, "mission": mission,
            "cadence": cadence,
            "goal_dist_m": goal_dist_m, "box_frac": box_frac, "end_reason": end_reason}


class MissionLoop:
    """The body of the flight loop, shared by demo/vla_fly.py and the tests.

    tick(state):
      1. backend.act(state): the step tracker's world-frame Action4D, its
         author in backend.last_tick (vla_step / hold / hover / none);
      2. shield.filter(state, action): the Shield ALWAYS judges the action;
         with shield_on the repaired `emitted` flies, with it off the
         backend's action flies unfiltered (the step tracker's, clamped to
         whatever altitude band the backend was built with:
         pilot_alt_config) and the decision is the shadow record (rows
         "shield": "off", which guardrail/kpi.py reads as the control arm;
         the row's violations / repairs / touched are that shadow's).
         guardrail.replay.flown_fields says what was FLOWN and its re-check,
         so the P0 escape count is measured, not inferred. `hud_decision`
         is what the policy indicator may read: with the Shield off, no
         repair, the flown action, the shadow's violations as escapes;
      3. monitor.update(...): the end reason, if this tick ends the mission
         (with the backend's last link failure, so slow and dead differ);
      4. the row: who authored the tick, the decision being flown (seq, age,
         its phrase and hint source, step and raw text from its vla_steps
         record), the Shield's decision, and the mission's numbers.

    `det_source() -> (det, box capture wall time[, ...])` for M2 (None for
    M1; unpack_box), read ONCE per tick - the close hold, the monitor, the
    row and LoopTick.det (the recorder's box) see the same box (review of
    d6e7123, P3: the close check and the row were two reads).
    `server_alive() -> bool` for a server this process started. `n_touched`
    counts ticks the Shield corrected what FLEW (0 with the Shield off),
    `n_shadow_touched` the shadow's would-be corrections.

    A FOLLOW's rows (close_hold set) also carry `close_box` (a fresh close
    box this tick, whatever the tracker did; CloseStall reads it) and
    `pilot_raw` (the tracker's own action: `raw` is what the Shield judged,
    the held one on a close-hold tick); with a DetectorHint that records
    kinds, `hint_kind` (the decision flown) and `hint_sent_kind` (the latest
    hint sent). The monitor gets the backend's request count
    (RemoteVLABackend.counts["requests"]) when it has one."""

    def __init__(self, backend, shield, monitor: MissionMonitor, *, mission: str,
                 shield_on: bool = True, goal_xy=None, det_source=None, hint_source=None,
                 detector_hint: DetectorHint | None = None, audit=None,
                 server_alive: Callable[[], bool] = lambda: True,
                 clock: Callable[[], float] = time.time,
                 close_hold: BoxClose | None = None):
        from guardrail.replay import flown_fields
        if mission not in MISSIONS:
            raise ValueError(f"mission {mission!r}")
        if mission == "goto" and goal_xy is None:
            raise ValueError("a goto mission needs goal_xy")
        if mission in DETECTOR_MISSIONS and det_source is None:
            raise ValueError(f"a {mission} mission needs det_source")
        self.backend, self.shield, self.monitor = backend, shield, monitor
        self.mission, self.shield_on = mission, bool(shield_on)
        self.goal_xy = None if goal_xy is None else (float(goal_xy[0]), float(goal_xy[1]))
        self.det_source, self.hint_source = det_source, hint_source
        self.detector_hint = detector_hint
        self.audit, self.server_alive, self.clock = audit, server_alive, clock
        # FOLLOW: while a fresh box counts as close, the VLA's horizontal
        # motion is held (zeroed before the Shield; its yaw and climb fly),
        # so the aircraft keeps the subject in view instead of flying over it.
        # Counted, and the row says so (close_hold).
        self.close_hold = close_hold
        self.n_close_hold = 0
        self._flown_fields = flown_fields
        self.n = 0
        self.t0: float | None = None
        self.n_touched = 0
        self.n_shadow_touched = 0
        self.authors: Counter = Counter()

    def tick(self, state: State, extra_row: dict | None = None) -> LoopTick:
        now = self.clock()
        if self.t0 is None:
            self.t0 = now
        held_close = close_now = False
        got = None

        def _close() -> bool:
            cdet, ct_box = got[0], got[1]
            fresh_s = self.detector_hint.fresh_s if self.detector_hint is not None else 1.0
            dropped = self.detector_hint is not None and self.detector_hint.box_ignored(ct_box)
            return bool(cdet is not None and ct_box is not None and not dropped
                        and now - float(ct_box) <= fresh_s and self.close_hold(cdet))
        tracker = getattr(self.backend, "tracker", None)
        if (self.close_hold is not None and self.det_source is not None
                and bool(getattr(tracker, "smoothing", False))):
            # SMOOTHING (TrackerConfig): the close hold is the TRACKER's
            # input, so it ramps down at the acceleration limit like every
            # other change, instead of zeroing a 4.5 m/s command in one tick
            # here (still once per tick: the box is read before act, which
            # neither reads nor changes it).
            got = unpack_box(self.det_source())
            close_now = _close()
            action = self.backend.act(state, hold_horizontal=close_now)
            held_close = close_now
            self.n_close_hold += int(close_now)
        else:
            action = self.backend.act(state)
        tk = self.backend.last_tick
        pilot_action = action
        if got is None and self.close_hold is not None and self.det_source is not None:
            got = unpack_box(self.det_source())
            close_now = _close()
            if close_now and (action.vx or action.vy):
                action = Action4D(vx=0.0, vy=0.0, vz_up=action.vz_up, yaw_rate=action.yaw_rate)
                held_close = True
                self.n_close_hold += 1
        d = self.shield.filter(state, action)
        flown = d.emitted if self.shield_on else action
        ff = self._flown_fields(self.shield, state, d, shield_on=self.shield_on)
        hd = hud_decision(d, action, shield_on=self.shield_on)
        if self.audit is not None:
            try:
                self.audit.log(self.n, d)
            except Exception:                                    # noqa: BLE001
                pass                       # the audit log never costs a tick
        pose = Pose.from_state(state)
        det = t_box = None
        if self.det_source is not None:
            if got is None:
                got = unpack_box(self.det_source())
            det, t_box = got[0], got[1]
        det_age = None if t_box is None else now - float(t_box)
        try:
            alive = bool(self.server_alive())
        except Exception:                                        # noqa: BLE001
            alive = False
        lf = getattr(self.backend, "last_link_failure", None)
        lf = lf if isinstance(lf, dict) else None
        # The request count, for a follow's subject_not_acquired (goto and
        # approach records stay as they were).
        counts = getattr(self.backend, "counts", None)
        sent = (int(counts.get("requests", 0))
                if self.mission == "follow" and isinstance(counts, dict) else None)
        end = self.monitor.update(now, pose, tk, goal_xy=self.goal_xy, det=det,
                                  det_age_s=det_age, server_alive=alive, link_failure=lf,
                                  requests_sent=sent)
        cur, step = current_decision(self.backend)
        self.authors[tk.author] += 1
        self.n_touched += int(bool(hd.touched))
        self.n_shadow_touched += int(d.touched)
        gd, gb, frac = goal_and_box(pose, self.goal_xy, det)
        row = {
            "t": round(now - self.t0, 3), "tick": self.n,
            "x": state.x, "y": state.y, "up": state.up, "psi": round(state.yaw_deg, 3),
            **decision_fields(tk, cur, step, hint_source=self.hint_source, link_failure=lf),
            # the Shield
            "shield": "on" if self.shield_on else "off",
            "raw": action.model_dump(), "emitted": d.emitted.model_dump(),
            "flown_action": flown.model_dump(),
            "touched": d.touched, "braked": d.braked,
            "violations": [v.model_dump() for v in d.violations],
            "repairs": [r.model_dump() for r in d.repairs],
            **ff,
            # the mission
            **mission_fields(gd, gb, frac, det, det_age, self.detector_hint),
        }
        if self.close_hold is not None:
            row["close_hold"] = held_close
            row["close_box"] = close_now
            row["pilot_raw"] = pilot_action.model_dump()
        if self.detector_hint is not None and self.detector_hint.record_kind:
            row["hint_kind"] = cur.get("hint_kind")
            row["hint_sent_kind"] = ((self.hint_source.last or {}).get("kind")
                                     if self.hint_source is not None else None)
        if getattr(getattr(tracker, "cfg", None), "rate_horizon_s", None) is not None:
            # RATE EXECUTION (vla_link.TrackerConfig): the commanded speed and
            # yaw rate of a step flown as a velocity; None on any other tick
            row["vla_rate"] = getattr(tracker, "last_rate", None)
        if extra_row:
            row.update(extra_row)
        hud = vla_hud(now - self.t0, state.up, tk, cur, step, touched=bool(hd.touched),
                      shield_on=self.shield_on, mission=self.mission,
                      cadence=getattr(self.backend, "cadence", None),
                      goal_dist_m=gd, box_frac=frac, end_reason=end)
        self.n += 1
        return LoopTick(flown, d, row, hud, end, hd, det)


# --------------------------------------------------------------------------- #
# The VLA panel on the recorded frames (the recorder thread; Pillow, lazily)
# --------------------------------------------------------------------------- #
# Moved here from demo/vla_fly.py on 2026-10-23 (wave V2b) so the ArduPilot
# rail's VLA pilot draws the same panel from the same code; vla_fly imports
# them under their old names.
AUTHOR_LABEL = {
    "vla_step": "VLA STEP", "hold": "HOLD", "hover": "HOVER", "none": "NO VLA DECISION"}
_OVERLAY = None


def hud_lines(hud: dict) -> list[str]:
    """The VLA panel: time and altitude (and the goal distance or the box),
    the decision behind this tick and how old its image is, its phrase and
    where the phrase came from, its step, and who authored the tick. The
    last line is the authorship, the one a viewer must not miss."""
    l0 = f"t {hud['t']:5.1f}s  alt {hud['alt']:4.1f} m"
    if hud.get("goal_dist_m") is not None:
        l0 += f"  goal {hud['goal_dist_m']:.0f} m"
    elif hud.get("box_frac") is not None:
        l0 += f"  box {hud['box_frac']:.2f}"
    seq, age = hud.get("seq"), hud.get("age_s")
    cad = {"stop_and_go": "  stop-and-go", "pipelined": "  pipelined"}.get(hud.get("cadence"), "")
    l1 = (f"AerialVLA #{seq}  image {age:.1f} s old{cad}" if seq is not None and age is not None
          else f"AerialVLA  no decision flying{cad}")
    ph = hud.get("phrase")
    l2 = ("phrase -" if ph is None else
          f"phrase '{ph.strip() or 'none: arrived?'}' ({hud.get('hint_source')})")
    st = hud.get("step")
    l3 = ("step -" if not st else
          f"step fwd {st['fwd_m']:.1f} m  turn {math.degrees(st['dpsi_rad']):+.0f}deg  "
          f"up {-st['down_m']:+.1f} m")
    if hud.get("land"):
        l4 = "LAND REQUESTED BY THE VLA"
    else:
        l4 = f"{AUTHOR_LABEL.get(hud.get('author'), hud.get('author'))} - {hud.get('reason')}"
    if not hud.get("shield_on", True):
        l4 += "  [SHIELD OFF]"
    return [l0, l1, l2, l3, l4]


def panel_lines(hud: dict) -> list[str]:
    """hud_lines and, when the rail gives one, its own label (`rail_label`:
    the ArduPilot rail's FSM state, an autopilot that took the aircraft,
    SHIELD OFF; review of 4a0da6d, P3-14 - the VLA panel had dropped it)."""
    lines = hud_lines(hud)
    return lines + [str(hud["rail_label"])] if hud.get("rail_label") else lines


def annotate_vla(img, det, hud: dict):
    """Draw the VLA panel (and, for M2, the detector box) on the first-person
    frame; with the policy indicator, its banner, rule panel and map too
    (demo/policy_hud.OverlayCache, 5 Hz). The caller has demo/ on sys.path
    (policy_hud), as for vla_bridge."""
    from PIL import ImageDraw
    im = img.copy()
    # After the mission the VLA no longer flies: the landing-site transfer
    # (follow_vlm._fly_to_landing_site, through the Shield, whose own HUD
    # dict says mode "land") and the descent (no Shield) say so instead of
    # leaving the last VLA tick on screen.
    landing = hud.get("frame") == "landing" or hud.get("mode") == "land"
    if landing:
        lines = [f"t {hud['t']:5.1f}s  alt {hud['alt']:4.1f} m", "MISSION OVER - VLA STOPPED",
                 hud.get("label") or hud.get("mode_label") or "LANDING (hand-written)"]
    else:
        lines = panel_lines(hud)
    d = ImageDraw.Draw(im)
    if det is not None:
        cx, cy, bw, bh = det[0], det[1], det[2], det[3]
        d.rectangle([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], outline=(0, 255, 0),
                    width=2)
    pol = hud.get("policy")
    if pol is not None:
        global _OVERLAY
        if _OVERLAY is None:
            from policy_hud import OverlayCache
            _OVERLAY = OverlayCache()
        _OVERLAY.draw(im, pol, lines)
        return im
    if not landing:
        lines = lines + [("SHIELD: correcting" if hud.get("shield") else "SHIELD: clear")
                         if hud.get("shield_on", True) else "SHIELD OFF (control arm)"]
    try:
        from policy_hud import draw_status_box
        draw_status_box(im, lines)
    except Exception:                                              # noqa: BLE001
        for i, ln in enumerate(lines):
            d.text((6, 6 + 12 * i), ln, fill=(255, 255, 255))
    return im


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
      * both tiles JPEG-encoded (`encode`, default jpeg_from_msg; the
        request carries the bytes).

    Only a capture that was SENT is kept: `commit(capture, hint, seq)` -
    RemoteVLABackend's on_submit - hands both tiles to `saver` (FrameSaver:
    vla_frames/<sha256>.jpg for the replay gate) and adds a line to `log`
    with the request's seq and phrase, the pairing and the camera sizes (the
    evidence that the frames really were 256x256). Until 2026-10-23 every
    capture was saved and logged before the hint check, so M2's captures
    with no fresh box - never sent - filled vla_frames/ and the log (review
    of the CityLife flight, P3-8). `counts` has "captures" (built) and
    "committed" (sent).

    THE LOWER TILE (`bottom_tile`, aerialvla_codec.BottomTile; 2026-10-24).
    "down" (the default) is everything above. "prev_front" keeps a short
    ring of front messages (put_front appends a pointer; about dt + tol +
    RING_MARGIN_S of them) and puts, in the request's down slot, the front
    frame whose capture time - this class's own rule, the one the capture
    gets - is nearest t_capture - dt_s within tol_s (aerialvla_codec.
    pick_prev_front, the dataset builder's rule). None: NO REQUEST this tick
    (counted no_prev_front), never the down frame; the down camera is not
    needed at all. The Capture says so (`lower`: mode, dt, the measured
    dt_actual_s, the earlier frame's stamp), and so do the request's record
    (vla_steps.jsonl `bottom_tile`), the log line and stats().

    SPEED WORDS (`speed_words`, aerialvla_codec.SpeedWords; 2026-10-24; off
    by default). Each capture also carries the aircraft's own horizontal
    speed at its capture time - aerialvla_codec.own_speed over the pose
    ring (`obs.pose_history()`, the loop's put_pose_full reads) at or
    before that time, a backward difference over the setting's window: the
    dataset builder's rule over track.jsonl - as `own_speed` (speed, class
    words, window); build_infer_fields appends the words to the object. No
    history reaching the window back (the first half second, a stalled
    loop): NO REQUEST this tick (counted no_speed). The log line and
    stats() say the speed and the words."""

    # prev_front: frames kept beyond dt + tol (s of arrival time)
    RING_MARGIN_S = 0.5

    def __init__(self, obs, encode: Callable[[dict], bytes | None] | None = None,
                 saver: FrameSaver | None = None, clock: Callable[[], float] = time.time,
                 max_pair_dt_s: float = 0.25, max_log: int = 20000,
                 bottom_tile: "C.BottomTile | None" = None,
                 speed_words: "C.SpeedWords | None" = None):
        self.obs = obs
        self.encode = encode or jpeg_from_msg
        self.saver = saver
        self.clock = clock
        self.max_pair_dt_s = float(max_pair_dt_s)
        self.tile = bottom_tile if bottom_tile is not None else C.BottomTile()
        if speed_words is not None and not callable(getattr(obs, "pose_history", None)):
            raise ValueError("speed words need the pose ring's history (obs.pose_history(), "
                             "semantic_demo.SemanticObs)")
        self.speed_words = speed_words
        self._lock = threading.Lock()
        self._front: tuple[dict, float] | None = None
        self._down: tuple[dict, float] | None = None
        # prev_front: (message, arrival) of the recent front frames (the
        # length cap only guards a clock that stands still)
        self._fronts: deque = deque(maxlen=256)
        self._ring_s = ((self.tile.dt_s + self.tile.tol_s + self.RING_MARGIN_S)
                        if self.tile.prev_front else 0.0)
        self.log: deque = deque(maxlen=max_log)
        self.counts: Counter = Counter()
        self._built: dict | None = None      # the last capture built, until committed

    def put_front(self, msg) -> None:
        with self._lock:
            t = self.clock()
            self._front = (msg, t)
            if self.tile.prev_front:
                self._fronts.append((msg, t))
                while self._fronts and self._fronts[0][1] < t - self._ring_s:
                    self._fronts.popleft()

    def put_down(self, msg) -> None:
        with self._lock:
            self._down = (msg, self.clock())

    def latest_msg(self, which: str):
        with self._lock:
            got = self._front if which == "front" else self._down
        return None if got is None else got[0]

    def _capture_time(self, msg, t_arr: float) -> tuple[float, str]:
        """(a front frame's capture time, its basis): its sim stamp mapped to
        wall time through the pose ring, never later than its arrival or
        now; the arrival when there is no stamp or the two clocks disagree
        by more than 2 s. Every capture's time, and in prev_front mode every
        ring frame's (the earlier frame is picked on the same clock)."""
        stamp = _stamp(msg)
        t_cap = self.obs.wall_at_stamp(stamp) if stamp is not None else None
        basis = "stamp"
        if t_cap is None or abs(t_cap - t_arr) > 2.0:
            t_cap = t_arr                 # no stamps, or the clocks disagree
            basis = "arrival"
        return min(t_cap, t_arr, self.clock()), basis

    def _prev_front(self, t_cap: float):
        """prev_front: (the ring's front message from dt_s earlier, its
        capture time) by aerialvla_codec.pick_prev_front, or None."""
        with self._lock:
            ring = list(self._fronts)
        cands = sorted(((self._capture_time(m, ta)[0], i, m) for i, (m, ta) in enumerate(ring)),
                       key=lambda c: (c[0], c[1]))
        j = C.pick_prev_front(t_cap, [c[0] for c in cands], self.tile.dt_s, self.tile.tol_s)
        return None if j is None else (cands[j][2], cands[j][0])

    def capture(self) -> Capture | None:
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
        lower = None
        if self.tile.prev_front:
            # the down slot is the front frame from dt_s earlier; the down
            # camera is not used (class docstring, THE LOWER TILE)
            t_cap, basis = self._capture_time(fmsg, t_arr)
            got = self._prev_front(t_cap)
            if got is None:
                self.counts["no_prev_front"] += 1
                return None
            dmsg, t_prev = got
            ds, pair_dt = _stamp(dmsg), None
            lower = {**self.tile.as_dict(), "dt_actual_s": round(t_cap - t_prev, 4),
                     "prev_t_capture": round(t_prev, 4), "prev_stamp_ns": ds}
        else:
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
            t_cap, basis = self._capture_time(fmsg, t_arr)
        own = None
        if self.speed_words is not None:
            # the aircraft's own speed from the ring at or before the capture
            # (class docstring, SPEED WORDS); none, no request
            got = C.own_speed(C.speed_history(self.obs.pose_history()), t_cap,
                              self.speed_words.window_s)
            if got is None:
                self.counts["no_speed"] += 1
                return None
            own = {"speed_mps": got["speed_mps"], "words": got["words"],
                   "window_s": got["window_s"], "t_start": round(got["t_start"], 4),
                   "t_end": round(got["t_end"], 4), "rule": self.speed_words.rule}
        front, down = self.encode(fmsg), self.encode(dmsg)
        if not front or not down:
            self.counts["decode_failed"] += 1
            return None
        cap = Capture(float(t_cap), Pose(float(pose["x"]), float(pose["y"]), float(pose["up"]),
                                         float(pose["yaw"])), front, down, lower=lower,
                      own_speed=own)
        self.counts["captures"] += 1
        rec = {"t_capture": round(cap.t_capture, 4), "t_arrival": round(t_arr, 4),
               "stamp_ns": stamp, "down_stamp_ns": ds,
               "pair_dt_s": None if pair_dt is None else round(pair_dt, 4),
               "pose_src": pose.get("src"), "t_capture_basis": basis,
               "pose": cap.pose.as_wire(),
               "front_wh": [fmsg.get("width"), fmsg.get("height")],
               "down_wh": [dmsg.get("width"), dmsg.get("height")]}
        if lower is not None:
            rec["bottom_tile"] = dict(lower)
        if own is not None:
            rec["own_speed"] = dict(own)
        self._built = {"cap": cap, "rec": rec}
        return cap

    def commit(self, capture: Capture, hint: Hint | None = None, seq: int | None = None,
               **extra) -> None:
        """The capture was sent (request `seq`): keep its frames and log it.
        Called on the loop thread (RemoteVLABackend.on_submit); the frames are
        written by the saver's own thread."""
        import hashlib
        built = self._built if (self._built and self._built["cap"] is capture) else None
        rec = dict(built["rec"]) if built else {
            "t_capture": round(capture.t_capture, 4), "pose": capture.pose.as_wire(),
            "front_wh": [None, None], "down_wh": [None, None], "pair_dt_s": None}
        if built:
            self._built = None
        else:
            if capture.lower is not None:
                rec["bottom_tile"] = dict(capture.lower)
            if capture.own_speed is not None:
                rec["own_speed"] = dict(capture.own_speed)
        front, down = capture.front_jpg, capture.down_jpg
        if self.saver is not None:
            self.saver.save(front)
            if down:
                self.saver.save(down)
        if hint is not None and hint.kind is not None:
            rec["hint_kind"] = hint.kind
        rec.update({"seq": seq, "phrase": None if hint is None else hint.phrase,
                    "hint_source": None if hint is None else hint.source,
                    "front_jpg_sha256": hashlib.sha256(front).hexdigest(),
                    "down_jpg_sha256": hashlib.sha256(down).hexdigest() if down else None,
                    **extra})
        self.counts["committed"] += 1
        self.log.append(rec)

    def stats(self) -> dict:
        """The counts, the sent captures' sizes and pairing, and how each
        sent capture was timed and posed (review of 4a0da6d, P2-5):
        `pose_src` (SemanticObs.pose_at: "ring-stamp" by sim stamp, "ring"
        interpolated at the arrival time, "ring-edge" past the ring's end)
        and `t_capture_basis` ("stamp": the frame's sim stamp mapped to wall
        time through the ring; "arrival": its arrival, render and transport
        latency included - where the ring has no sim stamps, as on the
        ArduPilot rail, whose poses are the EKF's)."""
        sizes = Counter(tuple(r["front_wh"]) for r in self.log)
        out = {"counts": dict(self.counts),
               "front_sizes": {f"{w}x{h}": n for (w, h), n in sizes.items()},
               "pair_dt_s": spread([r["pair_dt_s"] for r in self.log]),
               "pose_src": dict(Counter(r.get("pose_src") for r in self.log)),
               "t_capture_basis": dict(Counter(r.get("t_capture_basis") for r in self.log))}
        if self.tile.prev_front:
            # the lower tile and how much earlier the sent frames really were
            out["bottom_tile"] = {**self.tile.as_dict(), "dt_actual_s": spread(
                [(r.get("bottom_tile") or {}).get("dt_actual_s") for r in self.log])}
        if self.speed_words is not None:
            # the setting and what the sent captures' words said
            words = Counter((r.get("own_speed") or {}).get("words") for r in self.log)
            out["speed_words"] = {**self.speed_words.as_dict(),
                                  "words": {w: words[w] for w in C.SPEED_WORDS if words[w]},
                                  "speed_mps": spread([(r.get("own_speed") or {}).get("speed_mps")
                                                       for r in self.log])}
        return out


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
                missing_down: str = "refuse", adapter: str | None = None) -> list[str]:
    """The command line of demo/vla_server_v2.py serve. A real model gets
    --precision, --attn and --merge/--no-merge, always explicit (the server
    refuses a run that leaves one out). `fake_output` serves the scripted
    stand-in instead (identity fake:scripted, no weights): a plumbing dry run
    that can never be recorded as a VLA flight. --missing-down refuse: this
    pilot always sends the down frame, so a missing one is a fault, not a
    black tile the model never saw in training. `adapter` (a real model
    only): the LoRA directory instead of the server's default, upstream's
    aero_vla - a follow fine-tune's checkpoint, announced under its own id."""
    if adapter is not None and fake_output:
        raise ValueError("an adapter is for a real model, not the scripted stand-in")
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
        if adapter is not None:
            argv += ["--adapter", str(adapter)]
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
                "identity": {k: h.get(k) for k in IDENTITY_RECORD_KEYS},
                "max_new_tokens": h.get("max_new_tokens"),
                "libraries": h.get("libraries"), "weight_files": h.get("weight_files")}


def identity_is_vla(identity: dict | None) -> tuple[bool, str | None]:
    """(a real VLA identity?, why not): AerialVLA, one of our fine-tunes, or
    an allow-listed foreign model (aerialvla_codec.FOREIGN_FAMILIES, served
    by demo/vla_server_foreign.py). The brief's six fields
    (vla_link.check_identity) and a weights digest that is not one repeated
    digit (tools/todo.py weights_digest_ok: the test stand-ins report
    "0" * 64). The scripted stand-in (id fake:scripted, digest None) is not,
    nor is ANY `fake:` id whatever else it carries (re-review N4: a
    `fake:aerialvla...` with a real-looking digest was recorded as kind vla,
    its prefix lost under MODEL_ID)."""
    mid = (identity or {}).get("id") if isinstance(identity, dict) else None
    if isinstance(mid, str) and mid.startswith("fake:"):
        return False, f"id {mid!r} is a fake: stand-in, not a model"
    try:
        check_identity(identity)
    except ValueError as exc:
        return False, str(exc)
    d = identity["weights_sha256"]
    if len(set(d)) < 2:
        return False, "weights_sha256 is one repeated digit: a stand-in, not a model"
    return True, None


def model_id_for(identity: dict | None) -> str:
    """The pilot side's model id for a real VLA identity: MODEL_ID
    ("aerialvla:openvla-7b+aero_vla") for upstream's adapter, and for a
    follow fine-tune (the server's id "aerialvla-ft-follow/<run>/<epoch>@...",
    aerialvla_codec.adapter_variant) "aerialvla:openvla-7b+aerialvla-ft-
    follow/<run>/<epoch>". Until the follow fine-tune (2026-10-23) every
    real identity was recorded under MODEL_ID, so a fine-tune would have
    flown under upstream's adapter name with the fine-tune's digest beside
    it: the digest was honest, the name was not. A foreign model
    (aerialvla_codec.foreign_pilot_id, 2026-10-27) is "<family>:<name>"
    ("dronecats:DroneCATS-Cosmos3-Edge-2B"), never MODEL_ID."""
    from guardrail.aerialvla_codec import adapter_variant, foreign_pilot_id
    sid = str((identity or {}).get("id") or "")
    foreign = foreign_pilot_id(sid)
    if foreign is not None:
        return foreign
    if adapter_variant(sid) == "fine-tune":
        return f"aerialvla:openvla-7b+{sid.split('@', 1)[0]}"
    return MODEL_ID


def model_words(identity: dict | None) -> str:
    """How a console line names the server's model: "AerialVLA", or a
    foreign model by its family ("the foreign VLA dronecats")."""
    fam = C.foreign_family(identity.get("id") if isinstance(identity, dict) else None)
    return "AerialVLA" if fam is None else f"the foreign VLA {fam}"


# The identity keys a record keeps of the server's hello: the brief's six, the
# measured agreement, the adapter block, and a foreign model's variant and
# declared inputs (aerialvla_codec.foreign_block).
IDENTITY_RECORD_KEYS = ("id", "base", "weights_sha256", "precision", "attn", "merged",
                        "agreement", "adapter", "variant", "foreign")


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
# The lower tile the adapter was trained on, against the flight's (2026-10-24)
# --------------------------------------------------------------------------- #
def adapter_tile(identity: dict | None, read_local: bool = True) -> tuple["C.BottomTile", str]:
    """(the lower tile the server's adapter was trained on, where that was
    read): aerialvla_codec.adapter_bottom_tile from hello ("announced", or
    "absent" = down). A server from before the field announces a provenance
    block WITHOUT the key (demo/vla_server_v2.adapter_identity now always
    carries it, None for an older checkpoint): then, with `read_local`, the
    adapter's own provenance file is read here when its path is readable
    ("local provenance file") - a prev_front checkpoint served by an old
    server must not pass as "down". ValueError for a malformed field."""
    tile, src = C.adapter_bottom_tile(identity)
    if src == "announced" or not read_local:
        return tile, src
    a = identity.get("adapter") if isinstance(identity, dict) else None
    prov = a.get("provenance") if isinstance(a, dict) else None
    if isinstance(prov, dict) and "bottom_tile" not in prov and a.get("path"):
        try:
            p = json.loads((Path(str(a["path"])) / C.ADAPTER_PROVENANCE_FILE).read_text(
                encoding="utf-8"))
        except (OSError, ValueError):
            p = None
        if isinstance(p, dict) and p.get("bottom_tile") is not None:
            return C.bottom_tile_from(p["bottom_tile"]), "local provenance file"
    return tile, src


def bottom_tile_check(identity: dict | None, flight: "C.BottomTile", *, is_vla: bool = True,
                      read_local: bool = True) -> dict:
    """Is this flight's lower tile the one the server's adapter was trained
    on? {flight, adapter, source, checked, ok, why}: `ok` False (with `why`)
    refuses the flight before take-off (demo/vla_fly.py, demo/vla_dagger.py)
    - a model is never flown with a tile it never saw. A stand-in (not
    is_vla: the scripted server reads no image) is not checked, and says
    so; an adapter that records no tile is "down"."""
    rec = {"flight": flight.as_dict(), "adapter": None, "source": None, "checked": False,
           "ok": True, "why": None}
    if not is_vla:
        rec["source"] = "not checked: a stand-in, not a VLA (it reads no image)"
        return rec
    rec["checked"] = True
    # A foreign model (aerialvla_codec, FOREIGN DRONE VLAS) declares
    # bottom_tile "none": it flies the default down tile, which its server
    # ignores, and is refused prev_front (foreign_tile_mismatch).
    try:
        blk = C.foreign_block(identity)
    except ValueError as exc:
        rec.update({"ok": False, "why": f"the foreign model's declared inputs are malformed "
                                        f"({exc})"})
        return rec
    if blk is not None:
        rec["source"] = (f"foreign model ({blk['family']}): bottom_tile "
                         f"{C.FOREIGN_TILE_NONE!r}, the server ignores the lower tile")
        why = C.foreign_tile_mismatch(flight)
        if why is not None:
            rec.update({"ok": False, "why": why})
        return rec
    try:
        tile, src = adapter_tile(identity, read_local=read_local)
    except ValueError as exc:
        rec.update({"ok": False, "why": f"the adapter's recorded lower tile is malformed ({exc})"})
        return rec
    rec.update({"adapter": tile.as_dict(), "source": src})
    why = C.bottom_tile_mismatch(tile, flight)
    if why is not None:
        rec.update({"ok": False, "why": why})
    return rec


# --------------------------------------------------------------------------- #
# The speed words the adapter was trained with, against the flight's
# (2026-10-24)
# --------------------------------------------------------------------------- #
def adapter_speed_words(identity: dict | None, read_local: bool = True
                        ) -> tuple["C.SpeedWords | None", str]:
    """(the speed words the server's adapter was trained with - None: off -,
    where that was read): aerialvla_codec.adapter_speed_words from hello
    ("announced", or "absent" = off). A server from before the field
    announces a provenance block WITHOUT the key: then, with `read_local`,
    the adapter's own provenance file is read here when its path is
    readable ("local provenance file"), as adapter_tile does - a
    speed-words checkpoint served by an old server must not pass as off.
    ValueError for a malformed field."""
    sw, src = C.adapter_speed_words(identity)
    if src == "announced" or not read_local:
        return sw, src
    a = identity.get("adapter") if isinstance(identity, dict) else None
    prov = a.get("provenance") if isinstance(a, dict) else None
    if isinstance(prov, dict) and a.get("path"):
        try:
            p = json.loads((Path(str(a["path"])) / C.ADAPTER_PROVENANCE_FILE).read_text(
                encoding="utf-8"))
        except (OSError, ValueError):
            p = None
        if isinstance(p, dict) and "speed_words" in p:
            return C.speed_words_from(p["speed_words"]), "local provenance file"
    return sw, src


def speed_words_check(identity: dict | None, flight: "C.SpeedWords | None", *,
                      is_vla: bool = True, read_local: bool = True) -> dict:
    """Does this flight send the speed words the server's adapter was
    trained with (both none, or the same rule, window and classes)?
    {flight, adapter, source, checked, ok, why}: `ok` False (with `why`)
    refuses the flight before take-off (demo/vla_fly.py, demo/
    vla_dagger.py) - a model is never asked in a prompt it never saw, either
    way round. A stand-in (not is_vla) is not checked, and says so; an
    adapter that records none is off."""
    rec = {"flight": None if flight is None else flight.as_dict(), "adapter": None,
           "source": None, "checked": False, "ok": True, "why": None}
    if not is_vla:
        rec["source"] = "not checked: a stand-in, not a VLA (it reads no prompt)"
        return rec
    rec["checked"] = True
    try:
        blk = C.foreign_block(identity)
    except ValueError as exc:
        rec.update({"ok": False, "why": f"the foreign model's declared inputs are malformed "
                                        f"({exc})"})
        return rec
    if blk is not None:
        # a foreign model declares speed words off (aerialvla_codec, FOREIGN
        # DRONE VLAS): its command is built from the object words
        rec["source"] = f"foreign model ({blk['family']}): speed words off, declared"
        why = C.foreign_speed_words_mismatch(flight)
        if why is not None:
            rec.update({"ok": False, "why": why})
        return rec
    try:
        sw, src = adapter_speed_words(identity, read_local=read_local)
    except ValueError as exc:
        rec.update({"ok": False, "why": f"the adapter's recorded speed words are malformed "
                                        f"({exc})"})
        return rec
    rec.update({"adapter": None if sw is None else sw.as_dict(), "source": src})
    why = C.speed_words_mismatch(sw, flight)
    if why is not None:
        rec.update({"ok": False, "why": why})
    return rec


# --------------------------------------------------------------------------- #
# The front camera the server reads, against the flight's (2026-10-27)
# --------------------------------------------------------------------------- #
def front_source_check(identity: dict | None, source: str, *, is_vla: bool = True) -> dict:
    """Does this flight send the front camera the server reads
    (aerialvla_codec.front_source_mismatch)? {flight, server, checked, ok,
    why, source}: `ok` False refuses the flight before take-off
    (demo/vla_fly.py --vla-front-source). AerialVLA and its fine-tunes read
    VLAFront; a foreign server declares the camera its intrinsics assume. A
    stand-in (not is_vla) is not checked, and says so."""
    rec = {"flight": source, "server": None, "checked": False, "ok": True, "why": None,
           "source": None}
    if not is_vla:
        rec["source"] = "not checked: a stand-in, not a VLA (it reads no image)"
        return rec
    rec["checked"] = True
    try:
        blk = C.foreign_block(identity)
        why = C.front_source_mismatch(identity, source)
    except ValueError as exc:
        rec.update({"ok": False, "why": str(exc)})
        return rec
    rec["server"] = blk["front_source"] if blk is not None else C.DEFAULT_FRONT_SOURCE
    rec["source"] = ("the foreign model's declared camera" if blk is not None
                     else "AerialVLA's training camera (VLAFront)")
    if why is not None:
        rec.update({"ok": False, "why": why})
    return rec


# --------------------------------------------------------------------------- #
# The warm-up gate: one inference on a live frame pair before arming
# --------------------------------------------------------------------------- #
def warmup_hint(mission: str, hint_source: Callable[[Capture], Hint | None] | None,
                capture: Capture) -> Hint:
    """The warm-up request's phrase: M1 the goal phrase from the capture pose
    (the request a GOTO would send there); M2 "straight ahead " labelled
    "goal", as the server's own fidelity probe sends it (the detector is not
    running yet, and a detector hint must come from a box). Never flown."""
    if mission == "goto" and hint_source is not None:
        h = hint_source.fn(capture) if isinstance(hint_source, HintSource) else hint_source(capture)
        if h is not None:
            return h
    return Hint("straight ahead ", "goal")


def warmup_infer(client, capture: Capture, hint: Hint, object_text: str,
                 timeout_s: float, idle: Callable[[], None] | None = None) -> dict:
    """One BLOCKING request before the flight loop (tools, and vla_fly off
    the event loop): the answer's kind, the round trip on the client's clock
    (`rtt_s`), the server's own model time and the capture-to-answer
    latency, or why there was none. `idle()`, if given, is called about
    every 0.1 s while the answer is awaited: the ArduPilot rail pumps its
    autopilot link there, as its detector wait does, so a first inference of
    a minute leaves no minute of MAVLink unread behind it.

    THE REQUEST CARRIES `timeout_s` (review of 4a0da6d, P1-1): RemoteVLA
    gives it this answer timeout instead of its own (the flight's decision
    timeout), and `timeout_s` in the record is the one the client reports in
    force (LinkResult.timeout_s). Until then the record said 60 s for the
    first request while the socket gave up at 10 s."""
    fields, meta = build_infer_fields(capture, hint, object_text)
    # `object`: what was sent (with speed words, the words included)
    out: dict[str, Any] = {"phrase": hint.phrase, "hint_source": hint.source,
                           "object": fields["object"], "seq": None, "answered": False,
                           "kind": None, "rtt_s": None, "latency_server_s": None,
                           "latency_client_s": None, "failure": None, "detail": None,
                           "raw_text": None, "error": None, "timeout_s": float(timeout_s),
                           "client_timeout_s": getattr(client, "response_timeout_s", None)}
    if "own_speed" in meta:
        out["own_speed"] = meta["own_speed"]
    seq = client.submit(fields, response_timeout_s=float(timeout_s))
    if seq is None:
        out.update(failure="not_sent", detail="the client refused to send (busy or backing off)")
        return out
    out["seq"] = seq
    if idle is None:
        client.wait_idle(float(timeout_s) + 2.0)
    else:
        end = time.monotonic() + float(timeout_s) + 2.0
        while not client.wait_idle(0.1) and time.monotonic() < end:
            idle()
    res = client.poll()
    if res is None:
        out.update(failure="no_result", detail=f"no result within {timeout_s:g} s")
        return out
    out["rtt_s"] = round(res.t_done - res.t_sent, 4)
    out["latency_client_s"] = round(res.t_done - float(capture.t_capture), 4)
    if getattr(res, "timeout_s", None) is not None:
        out["timeout_s"] = float(res.timeout_s)          # what the client enforced
    if res.reply is None:
        out.update(kind="link_failure", failure=res.failure, detail=res.detail)
        return out
    r = res.reply
    out.update(answered=True, kind=r.kind, latency_server_s=r.latency_s,
               raw_text=r.raw_text, error=r.error)
    return out


def warmup_verdict(result: dict, max_latency_s: float) -> tuple[bool, str | None]:
    """(fly?, why not). No answer, a server error, or a round trip over
    `max_latency_s` refuse the flight; a malformed answer is answered (its
    latency is what this gate is about; the replay gate judges the parse
    rate)."""
    if not result.get("answered"):
        return False, (f"the warm-up inference got no answer ({result.get('failure')}: "
                       f"{result.get('detail')})")
    if result.get("kind") == "server_error":
        return False, f"the server refused the warm-up inference ({result.get('error')})"
    rtt = result.get("rtt_s")
    if rtt is None or rtt > max_latency_s:
        return False, (f"the warm-up inference took {rtt} s (round trip), over the "
                       f"{max_latency_s:g} s gate: the pilot could not keep up")
    return True, None


def warmup_series_verdict(attempts: list[dict], max_latency_s: float) -> tuple[bool, str | None]:
    """(fly?, why not) over WARMUP_REQUESTS warm-up requests, judging the
    LAST (re-review N1): the server runs no inference at start, so the first
    request is the model's first ever and pays the cold-start cost (CUDA
    kernels, allocator) that no flight request pays; judging it alone could
    refuse every launcher run that starts a fresh server. Every attempt is
    recorded by the caller; a silent or failed last attempt still refuses."""
    if not attempts:
        return False, "no warm-up request was made"
    return warmup_verdict(attempts[-1], max_latency_s)


def warmup_timeout_s(k: int, decision_timeout_s: float, first_decision_timeout_s: float) -> float:
    """The k-th warm-up request's answer timeout (k from 0): the first is the
    model's first inference ever and gets first_decision_timeout_s (re-review
    N1), the others the flight's decision timeout."""
    return (max(decision_timeout_s, first_decision_timeout_s) if k == 0
            else float(decision_timeout_s))


def warmup_result(last: dict, attempts: list[dict], max_latency_s: float,
                  late_replies=None) -> dict:
    """The warm-up record (metrics.json `warmup`): the last attempt's
    measurement, the verdict over the series (warmup_series_verdict), every
    attempt, and the late answers the client kept."""
    ok, why = warmup_series_verdict(attempts, max_latency_s)
    res = dict(last)
    res.update({"pass": ok, "why": why, "max_latency_s": max_latency_s,
                "judged": "last of attempts", "attempts": attempts,
                "late_replies": list(late_replies or [])})
    return res


def wait_client_ready(client, *, max_wait_s: float | None = None,
                      clock: Callable[[], float] = time.time,
                      sleep: Callable[[float], None] = time.sleep,
                      idle: Callable[[], None] | None = None) -> dict:
    """Wait, bounded, until RemoteVLA would send a request: no request in
    flight, no timed-out request's late answer still awaited
    (late_reply_wait_s), no unpolled result, no backoff after a failure or a
    server error. Each of those made a warm-up request made at once "not
    sent" (review of 4a0da6d, P1-1 and P2-8, on both rails). The in-flight
    and late-answer waits block on the client's own condition (real time);
    the backoff is on the client's clock (`clock`, `sleep`); an unpolled
    result is polled and dropped (counted). `idle()` (the ArduPilot rail's
    link pump, P3-15) runs on every look. The bound: `max_wait_s`, by
    default the client's late-answer wait + its backoff + 2 s, on either
    clock. Returns {"ready", "waited_s", "waited_on" {why: looks},
    "dropped_results", "why"}."""
    bound = (float(max_wait_s) if max_wait_s is not None else
             float(getattr(client, "late_reply_wait_s", 0.0) or 0.0)
             + float(getattr(client, "reconnect_backoff_s", 0.0) or 0.0) + 2.0)
    t0c, t0r = clock(), time.monotonic()
    waited: Counter = Counter()
    dropped = 0
    why = None
    while True:
        draining = bool(getattr(client, "draining", False))
        busy = bool(getattr(client, "busy", False))
        back = bool(getattr(client, "backing_off", False))
        if not (busy or back):
            break
        what = ("late_reply" if draining else "in_flight_or_unpolled" if busy else "backoff")
        if clock() - t0c > bound or time.monotonic() - t0r > bound:
            why = f"the client was still not ready after {bound:g} s ({what})"
            break
        if draining:
            waited["late_reply"] += 1
            client.wait_drained(0.1)
        elif busy:
            if client.wait_idle(0.1):
                # Nothing in flight, yet busy: a result nobody polled.
                waited["unpolled_result"] += 1
                dropped += int(client.poll() is not None)
            else:
                waited["in_flight"] += 1
        else:
            waited["backoff"] += 1
            sleep(0.1)
        if idle is not None:
            idle()
    return {"ready": why is None, "waited_s": round(clock() - t0c, 3),
            "waited_real_s": round(time.monotonic() - t0r, 3), "waited_on": dict(waited),
            "dropped_results": dropped, "why": why}


def run_warmup(client, next_capture: Callable[[], Capture | None], hint_source,
               mission: str, object_text: str, *, decision_timeout_s: float,
               first_decision_timeout_s: float, max_latency_s: float,
               commit: Callable | None = None, beat: Callable[[int], None] | None = None,
               n: int = WARMUP_REQUESTS, wait_frames_s: float = 30.0,
               clock: Callable[[], float] = time.time,
               sleep: Callable[[float], None] = time.sleep,
               idle: Callable[[], None] | None = None) -> dict:
    """The warm-up gate as one BLOCKING call, for a rail that runs it in a
    blocking pre-arm step (demo/pas_ardupilot/perception_node.py, disarmed,
    on the ground; demo/vla_fly.py runs the same series on its event loop,
    with wait_client_ready, warmup_infer, warmup_timeout_s and warmup_result
    in a thread). `n` requests, each on a fresh live capture from
    `next_capture()` (which waits for one itself, up to `wait_frames_s`, and
    returns None without one), with warmup_hint's phrase; the LAST is judged
    (warmup_series_verdict). `commit(capture, hint, seq, warmup=True)` keeps
    each sent request's frames; `beat(k)` is called before the k-th request.
    Never flown.

    EACH REQUEST GETS ITS OWN TIMEOUT AND A READY CLIENT (review of
    4a0da6d, P1-1). The first is the model's first inference ever and is
    sent with first_decision_timeout_s (warmup_infer), the rest with the
    decision timeout; before each the client is waited for
    (wait_client_ready: a late answer, the backoff after a server error).
    Until 2026-10-23 a cold first answer slower than the client's own
    timeout, or a refused first request, made the next two "not sent" and a
    healthy server was refused. Every attempt records the wait
    (`client_ready`). `idle()` runs during both waits."""
    attempts: list[dict] = []
    res: dict = {}
    for k in range(n):
        ready = wait_client_ready(client, clock=clock, sleep=sleep, idle=idle)
        cap = next_capture()
        if cap is None:
            return {"pass": False, "answered": False, "max_latency_s": max_latency_s,
                    "why": f"no VLA camera pair (front + down, paired by stamp) within "
                           f"{wait_frames_s:g} s", "attempts": attempts}
        hint = warmup_hint(mission, hint_source, cap)
        if beat is not None:
            beat(k)
        res = warmup_infer(client, cap, hint, object_text,
                           warmup_timeout_s(k, decision_timeout_s, first_decision_timeout_s),
                           idle=idle)
        res["client_ready"] = ready
        if res.get("seq") is not None and commit is not None:
            commit(cap, hint, res["seq"], warmup=True)
        attempts.append(dict(res))
    return warmup_result(res, attempts, max_latency_s, getattr(client, "late", None))


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
    parse failure, a hover or a none - and, on a rail that holds the
    aircraft (rows with `pilot_held`: the ArduPilot rail's FSM, a fault, a
    takeover), never a tick whose action the rail held, whoever authored it
    (review of 4a0da6d, P3-12: the first tick of each hold is the tracker's
    vla_step and flew nothing); those are `ticks_held_by_rail`.

    A FOLLOW (review of d6e7123): a vla_step tick whose horizontal motion
    the close hold zeroed (row `close_hold`) is counted apart -
    `ticks_close_held`, `close_held_tick_share` - and not in the vla_step or
    VLA-authored shares (P3); a LAND held as arrival is the model's decision
    (its hold ticks, reason land_as_arrival, are VLA-authored), counted as
    `land_as_arrival` and never in `land_accepted`, which counts LANDs that
    landed (P2-6); `hint_kind_counts` says how many requests came from a
    fresh box, the memory, the search (P2-2). Each key only where it applies,
    so the other rails' summaries are unchanged."""
    answered = [r for r in records if r.get("kind") in ("step", "land", "parse_fail")]
    lat_c = [r.get("latency_client_s") for r in answered]
    lat_s = [r.get("latency_server_s") for r in answered]
    rtt = [r.get("rtt_s") for r in answered]
    n = len(rows)
    by_author = Counter(r.get("author") for r in rows)
    flew = [r for r in rows if not r.get("pilot_held")]
    close_held = sum(1 for r in flew if r.get("close_hold"))
    vla_step = sum(1 for r in flew if r.get("author") == "vla_step" and not r.get("close_hold"))
    held_parsed = sum(1 for r in flew if r.get("author") == "hold"
                      and r.get("author_reason") in ("step_complete", "land", "land_as_arrival"))
    lands = [r for r in records if r.get("kind") == "land"]
    lands_held = sum(1 for r in lands if r.get("accepted") and r.get("land_as_arrival"))
    # The request in flight when the mission ended comes back as a link
    # failure refused "closed" (RemoteVLABackend.close): the run ended, the
    # server did not fail. Counted apart, so the answers say what the server
    # did while it was asked.
    closed = [r for r in records if r.get("refused") == "closed"]
    kinds = Counter(r.get("kind") for r in records if r.get("refused") != "closed")
    # A link failure's own kind: "timeout" is a slow model, "link" /
    # "protocol" / "internal" a broken one (review of the CityLife flight,
    # P2-5: they used to read alike).
    link_kinds = Counter(str((r.get("failure") or {}).get("kind")) for r in records
                         if r.get("kind") == "link_failure" and r.get("refused") != "closed")
    phrases = Counter(r.get("phrase") for r in records if "phrase" in r)
    sources = Counter(r.get("hint_source") for r in records if "hint_source" in r)
    extra: dict[str, Any] = {}
    if any("close_hold" in r for r in rows):
        extra.update({
            "ticks_close_held": close_held,
            "close_held_tick_share": round(close_held / n, 4) if n else None,
            "close_held_note": ("vla_step ticks whose horizontal motion the rail held "
                                "(MissionLoop close_hold): the model's turn and climb flew, "
                                "its translation did not; apart from the VLA shares")})
    if lands_held:
        extra["land_as_arrival"] = lands_held
    if any("hint_kind" in r for r in records):
        extra["hint_kind_counts"] = dict(Counter(r.get("hint_kind") for r in records
                                                 if "hint_kind" in r))
    return {
        "decisions_accepted": stats.get("decisions_accepted"),
        "decision_hz": stats.get("decision_hz"),
        "decision_hz_note": ("answers the model parsed (a step or LAND) that the step "
                             "tracker took, over the time act() ran; the action stream "
                             "is 10 Hz, the decisions behind it are not"),
        "requests": len(records),
        "answers_by_kind": dict(kinds), "in_flight_at_end": len(closed),
        "latency_client_s": spread(lat_c), "latency_server_s": spread(lat_s),
        "rtt_s": spread(rtt),
        "parse_fail": stats.get("parse_fail"), "server_error": stats.get("server_error"),
        "link_failure": stats.get("link_failure"), "link_failure_by_kind": dict(link_kinds),
        "late_replies": len([x for x in (stats.get("late_replies") or [])
                             if isinstance(x, dict) and "dropped" not in x]),
        "land": len(lands),
        "land_accepted": sum(1 for r in lands if r.get("accepted")
                             and not r.get("land_as_arrival")),
        "ticks": n, "ticks_by_author": dict(by_author),
        "ticks_held_by_rail": n - len(flew),
        "vla_step_tick_share": round(vla_step / n, 4) if n else None,
        "vla_authored_tick_share": round((vla_step + held_parsed) / n, 4) if n else None,
        "phrase_counts": dict(phrases), "hint_source_counts": dict(sources),
        "distinct_phrases": len([p for p in phrases if p is not None]),
        **extra,
    }


def pilot_entry(identity: dict | None, stats: dict, records: list[dict], rows: list[dict], *,
                server: dict, reported_by: str, weight_paths: list[str] | None = None,
                **extra) -> dict:
    """metrics["pilot"]: guardrail.manifest.pilot_record for the model that
    answered, with the backend's stats under "vla" (the shape tools/todo.py
    _pilot_stats reads: "pilot/vla" with decisions_accepted and
    ticks_by_author), the summary, the server record and `extra`.

    A real VLA identity (identity_is_vla: AerialVLA, a fine-tune, or an
    allow-listed foreign model under its own id, model_id_for) is kind "vla"
    with the server's weights_sha256 as `reported_digest` (and the local
    weight files when they are readable here, so the two digests are
    compared). Anything
    else - the scripted stand-in - is kind "stub" under its own id, which
    the dashboard never counts as a VLA."""
    from guardrail.manifest import pilot_record
    ok, why = identity_is_vla(identity)
    if ok:
        rec = pilot_record(model_id_for(identity), "vla", weight_paths=weight_paths,
                           reported_digest=identity["weights_sha256"], reported_by=reported_by)
    else:
        sid = str((identity or {}).get("id") or "unknown")
        rec = pilot_record(sid if sid.startswith("fake:") else f"fake:{sid}", "stub")
        rec["not_a_vla_because"] = why
    rec["server_identity"] = {k: (identity or {}).get(k) for k in IDENTITY_RECORD_KEYS}
    rec["vla"] = stats
    rec["summary"] = pilot_summary(stats, records, rows)
    rec["server"] = server
    rec.update(extra)
    return rec


def manifest_for(policy_hash: str, identity: dict | None, seed: int, scene_path: str | Path,
                 weight_paths: list[str] | None = None, sim_address: str | None = None) -> dict:
    """The six-field manifest: vla_model_hash from the server's reported
    digest (guardrail/manifest.resolve_vla_hash), or the stand-in's id; the
    topology from where the simulator runs (`sim_address`,
    guardrail.manifest.projectairsim_topology)."""
    from guardrail.manifest import build_manifest, projectairsim_topology
    topology = projectairsim_topology(sim_address)
    ok, _ = identity_is_vla(identity)
    if ok:
        return build_manifest(policy_hash, model_id_for(identity), seed, scene_path=scene_path,
                              weight_paths=weight_paths, topology=topology,
                              reported_digest=identity["weights_sha256"])
    sid = str((identity or {}).get("id") or "unknown")
    return build_manifest(policy_hash, sid if sid.startswith("fake:") else f"fake:{sid}", seed,
                          scene_path=scene_path, topology=topology)


# Project AirSim capture image types, as the robot configs here use them.
_IMAGE_TYPES = {0: "scene", 1: "depth_planar", 2: "depth"}
DETECTOR_CAMERA = "FrontCamera"


def sim_config_record(scene_path: str | Path, *, depth_read: bool | None) -> dict:
    """metrics["sim_config"] for a detector mission (review of d6e7123, P3):
    the scene and the robot config it names (sha256 of both), the streams
    the detector camera captures, and whether this run READ the depth. The
    follow's identity check added a depth stream to FrontCamera in
    robot_vla_quad_det.jsonc on 2026-10-23 (d6e7123); the approach flies the
    same robot config, so an approach since renders that stream unread - a
    render load the approaches before it did not carry. Recorded, so runs
    either side of it are compared knowingly. Problems are listed, never
    guessed over."""
    import hashlib
    from guardrail.manifest import _strip_jsonc
    scene_path = Path(scene_path)
    rec: dict[str, Any] = {
        "scene": scene_path.name, "scene_sha256": None, "robot_config": None,
        "robot_config_sha256": None, "detector_camera": None, "detector_streams": None,
        "detector_depth_rendered": None, "detector_depth_read": depth_read,
        "comparability": ("the detector camera captures a depth stream since d6e7123 "
                          "(2026-10-23, the follow's identity check); a run that does not "
                          "read it (an approach; a follow with identity off) still renders "
                          "it, which detector-mission runs before d6e7123 did not"),
        "problems": []}
    try:
        raw = scene_path.read_bytes()
        rec["scene_sha256"] = hashlib.sha256(raw).hexdigest()
        scene = json.loads(_strip_jsonc(raw.decode("utf-8")))
        robot = next(a for a in scene.get("actors") or []
                     if isinstance(a, dict) and a.get("type") == "robot")
        rec["robot_config"] = str(robot.get("robot-config"))
        rraw = (scene_path.parent / rec["robot_config"]).read_bytes()
        rec["robot_config_sha256"] = hashlib.sha256(rraw).hexdigest()
        cfg = json.loads(_strip_jsonc(rraw.decode("utf-8")))
        cam = next((s for s in cfg.get("sensors") or []
                    if isinstance(s, dict) and s.get("id") == DETECTOR_CAMERA), None)
        if cam is None:
            rec["problems"].append(f"{rec['robot_config']} has no {DETECTOR_CAMERA}")
            return rec
        rec["detector_camera"] = DETECTOR_CAMERA
        caps = [c for c in cam.get("capture-settings") or []
                if isinstance(c, dict) and c.get("capture-enabled", True)]
        types = [c.get("image-type") for c in caps]
        rec["detector_streams"] = [_IMAGE_TYPES.get(t, f"image-type {t}") for t in types]
        rec["detector_depth_rendered"] = any(t in (1, 2) for t in types)
    except (OSError, ValueError, StopIteration, AttributeError) as exc:
        rec["problems"].append(f"{type(exc).__name__}: {exc}")
    return rec


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
    out = Path(out)
    if not (out / "flight_log.jsonl").exists():
        write_jsonl(out / "flight_log.jsonl", rows)
    write_json(out / "metrics.json", metrics)
    write_json(out / "manifest.json", manifest)
    kpis = run_kpis(rows, metrics, manifest, policy)
    write_json(out / "kpi.json", kpis)
    metrics["kpi_grade"] = kpis["kpi_grade"]
    metrics["p0_violation_escape_rate"] = kpis.get("p0_violation_escape_rate")
    write_json(out / "metrics.json", metrics)
    return kpis


def run_kpis(rows: list[dict], metrics: dict, manifest: dict, policy) -> dict:
    """kpi.json's content: guardrail/kpi.py over the rows and metrics, graded
    by guardrail/manifest.is_kpi_grade, with the run's own records beside it
    - what finish_run writes, and what tools/rescore_vla_follow.py recomputes
    for a run flown before a records fix."""
    from guardrail import kpi as kpi_mod
    from guardrail.manifest import is_kpi_grade
    kpis = kpi_mod.compute(rows, kpi_mod.rule_priorities(policy), metrics)
    graded, reasons = is_kpi_grade(manifest, metrics)
    kpis.update({"kpi_grade": graded, "kpi_grade_reasons": reasons, "manifest": manifest,
                 "policy_source": metrics.get("policy_source"),
                 "shield_lookahead": metrics.get("shield_lookahead"),
                 "aborted": metrics.get("aborted"), "end_reason": metrics.get("end_reason"),
                 "interrupted": metrics.get("interrupted"),
                 "pilot_alt_band": metrics.get("pilot_alt_band"),
                 "cadence": (metrics.get("params") or {}).get("cadence")})
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


# The share of the ticks the subject's truth must cover for a follow to count
# as SCORED: follow_vlm.FOLLOW_TRUTH_MIN_COVERAGE (perception_node's
# TRUTH_MIN_COVERAGE), so the three rails answer "was this follow measured?"
# the same way (tests/test_vla_fly.py checks the two constants agree).
FOLLOW_TRUTH_MIN_COVERAGE = 0.95


def follow_scores(rows: list[dict], *, within_m: float = 30.0,
                  acquired_at_s: float | None = None) -> dict:
    """How closely a follow kept to the moving subject, from each row's
    `subject_dist_m` (the level car's simulator position against the
    aircraft's: ground truth for SCORING only - it never reaches the phrase,
    the VLA or a stop rule). The same "within 30 m" share the city follow
    reports (follow_vlm frac_within_30m), so the two are comparable.

    WITH follow_vlm's MEANING (review of d6e7123, P1-1): `frac_within_30m`
    is the share of the ticks that HAVE the truth, rounded to 3 places, None
    without any; `follow_scored` is follow_vlm.follow_scored's rule - the
    truth on at least FOLLOW_TRUTH_MIN_COVERAGE of the ticks - so a follow
    whose truth polling failed for a stretch is not called measured.
    `frac_within` (the name of d6e7123) is kept beside it. follow_kpi_fields
    lifts the two keys guardrail/kpi.py reads to the top of metrics.

    FROM THE FIRST ACQUISITION (2026-10-24): follow_vlm starts its mission
    clock, and its score, at its start gate - the hand-written reference
    (citylife_redcar_nfz_motion2_20261019, 0.963) waited 196 s there and
    scored the 240 s after - while vla_fly scores from take-off, so a VLA
    follow's share carried the ~200 s the level car spends on the far side
    of its loop. `after_acquire` is the same share over the rows from
    `acquired_at_s` (mission seconds of the first identity commit; None when
    the follow never acquired): the number that compares with follow_vlm's.
    The whole-flight share stays the KPI key."""
    d = [r["subject_dist_m"] for r in rows if r.get("subject_dist_m") is not None]
    n = len(rows)
    coverage = round(len(d) / n, 4) if n else 0.0
    scored = n > 0 and len(d) >= FOLLOW_TRUTH_MIN_COVERAGE * n
    if not d:
        return {"follow_scored": False, "ticks_scored": 0, "ticks": n,
                "truth_coverage": coverage, "frac_within_30m": None,
                "after_acquire": None}
    frac = round(sum(1 for x in d if x <= within_m) / len(d), 3)
    after = None
    if acquired_at_s is not None:
        da = [r["subject_dist_m"] for r in rows if r.get("subject_dist_m") is not None
              and r.get("t") is not None and r["t"] >= acquired_at_s]
        fa = round(sum(1 for x in da if x <= within_m) / len(da), 3) if da else None
        after = {"first_acquire_s": round(float(acquired_at_s), 3), "ticks_scored": len(da),
                 "frac_within": fa, "frac_within_30m": fa if within_m == 30.0 else None,
                 "basis": "follow_vlm's: scored from the start gate's acquisition"}
    return {"follow_scored": bool(scored), "ticks_scored": len(d), "ticks": n,
            "truth_coverage": coverage, "within_m": within_m,
            "frac_within": frac,
            "frac_within_30m": frac if within_m == 30.0 else None,
            "dist_min_m": round(min(d), 2), "dist_median_m": round(statistics.median(d), 2),
            "dist_max_m": round(max(d), 2), "dist_start_m": round(d[0], 2),
            "dist_end_m": round(d[-1], 2), "after_acquire": after}


def follow_kpi_fields(scores: dict | None) -> dict:
    """The follow's score where guardrail/kpi.py, tools/kpi_report.py and
    tools/build_dashboard.py read it - the TOP of metrics.json, as
    follow_vlm and perception_node write it: `frac_within_30m` (kpi fails a
    follow under FOLLOW_MIN_FRAC, "follow_proxy") and `follow_scored` (False
    makes mission_success None, "follow_not_scored"). Until the review of
    d6e7123 (P1-1) they sat only under metrics["follow"], kpi applied no
    follow test, and every VLA follow's kpi.json said mission_success true -
    two of them with 0.0 of their ticks within 30 m."""
    s = scores if isinstance(scores, dict) else {}
    after = s.get("after_acquire") if isinstance(s.get("after_acquire"), dict) else {}
    return {"frac_within_30m": s.get("frac_within_30m"),
            "follow_scored": bool(s.get("follow_scored")),
            # follow_vlm's basis (follow_scores); not a KPI key
            "frac_within_30m_after_acquire": after.get("frac_within_30m")}


def land_record(rows: list[dict], records: list[dict], goal_xy=None) -> dict:
    """What happened at a LAND: where, under which phrase (the decision's own
    phrase from its vla_steps record), and how far from the goal - the
    evidence for "LAND only near the goal or without a hint".

    A LAND a follow HELD as arrival (TrackerConfig.land_as_arrival) is no
    landing the VLA requested: `requested_by_vla` counts only a LAND that
    landed, and the held ones are under `land_as_arrival` (how many, that
    they were held, the first one's seq and when its hold began). Until the
    review of d6e7123 (P2-6) follows #6 and #9 read "requested_by_vla true"
    with no time, though both ended on the clock."""
    accepted = [r for r in records if r.get("kind") == "land" and r.get("accepted")]
    held = [r for r in accepted if r.get("land_as_arrival")]
    lands = [r for r in accepted if not r.get("land_as_arrival")]
    if not lands:
        out = {"requested_by_vla": False}
        if held:
            first = held[0].get("seq")
            out["land_as_arrival"] = {
                "count": len(held), "held": True, "first_seq": first,
                "first_t_s": next((r["t"] for r in rows if r.get("vla_seq") == first
                                   and r.get("author_reason") == "land_as_arrival"), None),
                "note": ("the model answered LAND; the follow held each as arrival "
                         "(TrackerConfig.land_as_arrival) - no landing was flown")}
        return out
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
