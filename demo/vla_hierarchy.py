"""
Hierarchical control of the red-car follow: AerialVLA decides the INTENT at
its own rate (~2 Hz), the hand-written controller of demo/follow_vlm.py
executes it at 10 Hz (the user's decision, 2026-10-25).

    # the fine-tuned model behind its socket (vla-real env, the GPU)
    python demo/vla_server_v2.py serve --precision bf16 --attn sdpa --merge \\
        --missing-down refuse --adapter D:/models/aerialvla-ft-follow/<run>/epoch<N>
    # the follow, the VLA deciding
    .\\scripts\\run_citylife_follow.ps1 -Object "a red car" -LevelCar Car_10 ... `
        -VlaHierarchy 127.0.0.1:8790 -VlaHierarchyBottomTile prev_front
    python demo/follow_vlm.py ... --vla-hierarchy 127.0.0.1:8790

WHY

AerialVLA flying the follow end to end reached at best 0.238 within 30 m
from first acquisition (vla_follow_ft_02_rate) against 0.963 for the
hand-written follow. The ceiling flights (demo/decide_rate.py) held the
expert itself to a VLA's rate: a decision every 1.0 s from 0.8 s before
scored 0.122, every 0.5 s from 0.4 s before 0.908. The decision RATE is the
bottleneck, and no amount of cloning moves AerialVLA's ~0.5-0.8 s per answer.
So the VLA decides what to do and the controller, which acts every 0.1 s on
OWL-ViT's boxes, does it.

THE INTENT (VLAHierarchy.plan, once per tick before the controller runs)

AerialVLA is asked continuously, pipelined - the next request as soon as an
answer is back - on demo/vla_dagger.py's machinery (VLAPerturber, which this
class extends): its query thread (`vla-hierarchy`), its own VLAFrames from
the VLAFront/VLADown cameras over the loop's pose ring, DetectorHint's
phrase over the follow's locked box (yaw-corrected, vla_fly's memory and
search phrases past fresh_s), vla_fly's follow TrackerConfig built by
vla_fly's own parser (follow_args: the lower tile, the step as a rate and
the speed words are vla_fly's flags), and the refusal of a server whose
adapter was trained on another tile or other speed words. Each answer the
model parsed (a step, or LAND) sets the intent, held until the next one and
EXPIRING `expiry_s` (1.5 s) after it arrived; a parse failure, a server
error or a link failure is no answer and sets nothing. Each tick:

  * no answer yet, or the last one expired: `controller_fallback` - the
    controller flies its own default behaviour (its stand-off, its coast,
    search and junction planner), recorded with why;
  * a fresh LAND: `hold` - the mission's arrival, never a landing. With the
    controller's lock it keeps the subject centred at the stand-off IN
    FORCE (s frozen); without one, vla_fly's follow tracker holds the LAND
    as arrival (a hover, ramped down at its acceleration limit);
  * a fresh step, the subject locked: `controller_follow` - the controller
    flies exactly as it always does (its yaw servo, the stand-off speed with
    the subject's speed fed forward, the trail, the slew limiter, the fence
    guard) EXCEPT its stand-off: desired_range(fwd_m), 22 m at fwd 0 to 12 m
    at fwd 5 (--vla-hierarchy-range), clamped there. The VLA's dpsi is
    recorded and does not steer: the controller keeps the car centred;
  * a fresh step, no lock: `vla_search` - the VLA flies directly, its step
    through vla_link.StepTracker with vla_fly's follow config exactly as a
    DAgger window flies it; the controller's search planner is not flown.
    When the lock comes back, the next tick is FOLLOW again.

"Locked" is the controller's own: the tick follow_vlm's controller tracks on
(an estimate served, or a fresh box with no re-acquisition running) - the
condition of its own `track` branch, passed in by the loop.

THE STAND-OFF NEVER JUMPS (StandOff). The stand-off in force, s, moves toward
its target at most S_RATE_MPS (2 m/s) of the loop's time (dt capped at 0.25
s, so a stalled tick buys no jump): the VLA's desired range in FOLLOW, held
in HOLD and SEARCH, and the controller's own stand-off on a fallback - so an
expiry walks s back to --want-width's range instead of stepping to it, and
from the first tick s starts at the controller's own. follow_vlm assigns s to
its `want_range`, the one stand-off parameter every arm of its controller
reads (standoff_speed, servo_want_width, the coast's stop).

THE HAND-OVER (as DAgger's). While the controller flies, the tracker is
suspended and every answer is PARKED (the newest wins, a link failure drops
it): taken on arrival its step would be flown by the controller. The first
VLA-flown tick resumes the tracker, tells its smoothing the command in force
(StepTracker.note_flown) and takes the parked answer anchored where the
aircraft is then; answers during VLA-flown ticks are taken at once. When the
controller takes the aircraft back the tracker is suspended (its decision
ends "suspended"); its slew limiter never saw the VLA's commands. A VLA tick
whose tracker flies no model decision (no step yet, a hover after a link
failure, a held parse failure, a hold that timed out) is flown by the
controller and recorded `controller_fallback` with the tracker's reason: no
silent VLA tick.

PLACEMENT AND SAFETY. apply() sits where demo/vla_dagger.VLAPerturber and
demo/decide_rate.DecideRate sit in follow_vlm: after the controller and its
slew limiter, before the Shield, which filters every command as always - the
VLA's included. Nothing here raises into the loop: a fault in plan() or
apply() is counted and the controller's command returned; the query thread
counts its own; a dead server is link failures, then an expired intent.

THE RECORDS. Every tick's `hierarchy` row (follow_vlm's flight_log.jsonl):
the authority (and the one planned when the tracker could not fly it), the
intent, whether the subject was locked, the answer's age (receipt and
capture), its seq, kind, fwd and dpsi, the stand-off s, its target and the
controller's own; on a VLA tick the tracker's author, reason and command.
metrics.json `vla_hierarchy`: ticks and share per authority, the VLA-flown
share, the intents histogram, the s distribution (all ticks and FOLLOW
ticks), why each fallback, answers, latency, the server's identity and
adapter, the configuration. One vla_steps.jsonl line per request in
demo/out/<tag>/vla_hierarchy/ (vla_link's record with what became of it);
no frames are kept (a measurement flight, never training data). The follow
score is follow_vlm's, computed as on any flight, so it reads against the
hand-written follow's 0.963 directly.

No projectairsim, cv2, Pillow or torch at module level (VLAFrames' default
JPEG encoder imports cv2 when a frame is encoded, in the flight's env).
"""
from __future__ import annotations

import math
import sys
import threading
import time
from collections import Counter
from dataclasses import asdict
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail.vla_link import (FWD_RANGE_M, VLA_STEPS_SCHEMA, Pose, StepLog,   # noqa: E402
                                decision_from_result, identity_of, make_record)

import vla_dagger as VD                                              # noqa: E402
import vla_mission as VM                                             # noqa: E402

SOURCE = "vla_hierarchy"
# The stand-off the VLA may ask for: (near, far), fwd_m 5 -> near, 0 -> far.
# 12 m sits outside the camera's blind spot at 8 m (6.9 m) and the car
# policy's 10 m ring; 22 m inside the 30 m the follow is scored on.
S_RANGE_M = (12.0, 22.0)
S_RATE_MPS = 2.0                 # the stand-off in force moves at most this fast
MAX_DT_S = 0.25                  # ...over at most this much of a stalled tick
EXPIRY_S = 1.5                   # an intent this long after its answer arrived is gone
AUTHORITIES = ("controller_follow", "vla_search", "hold", "controller_fallback")
INTENTS = ("follow", "search", "hold")
STEPS_DIR = "vla_hierarchy"      # under demo/out/<tag>/: vla_steps.jsonl
ROBOT_CONFIG = "robot_semantic_quad_vla.jsonc"
_FLYING_AUTHORS = ("vla_step", "hold")
_INTENT_OF = {"controller_follow": "follow", "vla_search": "search", "hold": "hold"}
RULE = ("AerialVLA is asked continuously (pipelined); each answer it parsed sets the intent, "
        "held until the next and expiring expiry_s after it arrived. Fresh step + the "
        "controller's lock: controller_follow (the controller flies with its stand-off at the "
        "VLA's desired range, far m at fwd 0 to near m at fwd 5, moved at most s_rate_mps). "
        "Fresh step, no lock: vla_search (vla_fly's follow tracker flies the step). Fresh LAND: "
        "hold (the controller at the stand-off in force with the lock, the tracker's arrival "
        "hold without). Otherwise controller_fallback (the controller's own behaviour, its "
        "stand-off reached at the same rate). After the controller's slew limiter, before the "
        "Shield, which filters every command")


class HierarchyRefused(ValueError):
    """A hierarchical flight that cannot fly honestly (no server, not
    AerialVLA, a tile or speed words the adapter never saw, a bad range):
    refused before the simulator is touched."""


def _r(v, n: int = 4):
    return None if v is None else round(float(v), n)


def _act(a) -> dict | None:
    if a is None:
        return None
    return {k: float(getattr(a, k)) for k in ("vx", "vy", "vz_up", "yaw_rate")}


def _pct(v: list[float], q: float) -> float | None:
    """vla_mission.spread's percentile: linear between the closest ranks."""
    if not v:
        return None
    s = sorted(v)
    k = (len(s) - 1) * q / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return round(s[lo] + (s[hi] - s[lo]) * (k - lo), 4)


def _spread(vals) -> dict:
    """{n, p50, p95, max}, vla_dagger's shape (the DAgger summary's)."""
    v = [float(x) for x in vals if x is not None]
    return {"n": len(v), "p50": _pct(v, 50), "p95": _pct(v, 95),
            "max": None if not v else round(max(v), 4)}


def _dist(vals) -> dict:
    """{n, min, p5, p50, p95, max}."""
    v = [float(x) for x in vals if x is not None]
    return {"n": len(v), "min": None if not v else round(min(v), 4), "p5": _pct(v, 5),
            "p50": _pct(v, 50), "p95": _pct(v, 95), "max": None if not v else round(max(v), 4)}


# --------------------------------------------------------------------------- #
# The intent's arithmetic
# --------------------------------------------------------------------------- #
def check_range(s_range) -> str | None:
    """Why (near, far) cannot be the VLA's stand-off range, or None."""
    try:
        near, far = (float(s_range[0]), float(s_range[1]))
    except (TypeError, ValueError, IndexError):
        return f"the stand-off range must be NEAR,FAR in metres, got {s_range!r}"
    if not (math.isfinite(near) and math.isfinite(far) and 0.0 < near < far):
        return f"the stand-off range must be 0 < NEAR < FAR metres, got {near:g},{far:g}"
    return None


def desired_range(fwd_m: float, s_range=S_RANGE_M, fwd_range=FWD_RANGE_M) -> float:
    """The stand-off the VLA asks for with a step of `fwd_m`: linear from the
    far end at fwd 0 to the near end at fwd 5 (AerialVLA's fwd_m range),
    clamped to the range. The fine-tune's fwd_m is the expert's displacement
    over the next second, so a long step is the expert closing in on a car
    that got away, a zero step the expert standing back."""
    near, far = float(s_range[0]), float(s_range[1])
    f0, f1 = float(fwd_range[0]), float(fwd_range[1])
    k = min(1.0, max(0.0, (float(fwd_m) - f0) / (f1 - f0)))
    return min(far, max(near, far + (near - far) * k))


class StandOff:
    """The stand-off in force (module docstring, THE STAND-OFF NEVER JUMPS):
    step(target, now, default) moves it toward `target` (None: held) at most
    rate_mps x dt, dt the time since the last step capped at max_dt_s; the
    first step starts it at `default`, the controller's own stand-off."""

    def __init__(self, rate_mps: float = S_RATE_MPS, max_dt_s: float = MAX_DT_S):
        if not (math.isfinite(rate_mps) and rate_mps > 0 and math.isfinite(max_dt_s)
                and max_dt_s > 0):
            raise ValueError("rate_mps and max_dt_s must be positive")
        self.rate_mps, self.max_dt_s = float(rate_mps), float(max_dt_s)
        self.s: float | None = None
        self._t: float | None = None
        self.n_limited = 0

    def step(self, target: float | None, now: float, default: float) -> float:
        if self.s is None:
            self.s, self._t = float(default), float(now)
        dt = min(max(float(now) - self._t, 0.0), self.max_dt_s)
        self._t = float(now)
        if target is not None:
            d, lim = float(target) - self.s, self.rate_mps * dt
            if abs(d) > lim:
                d = math.copysign(lim, d)
                self.n_limited += 1
            self.s += d
        return self.s


def check_args(addr: str, *, s_range=S_RANGE_M, bottom_tile: str = "down",
               prev_dt_s: float | None = None, step_as_rate: float | None = None,
               speed_words: bool = False, front_source: str = "vla",
               expiry_s: float = EXPIRY_S) -> str | None:
    """Why follow_vlm should refuse --vla-hierarchy with these, or None. The
    lower tile, the rate and the speed words are vla_fly's own flags:
    vla_fly's parser judges them (vla_dagger.follow_args)."""
    try:
        VM.parse_addr(addr)
    except ValueError as exc:
        return f"--vla-hierarchy: {exc}"
    why = check_range(s_range)
    if why is not None:
        return f"--vla-hierarchy-range: {why}"
    if front_source not in ("vla", "detector"):
        return f"--vla-hierarchy-front-source {front_source!r} is not vla or detector"
    if not (isinstance(expiry_s, (int, float)) and math.isfinite(expiry_s) and expiry_s > 0):
        return f"--vla-hierarchy-expiry-s must be a positive number of seconds, got {expiry_s!r}"
    try:
        VD.follow_args("a red car", cruise_alt=10.0, bottom_tile=bottom_tile,
                       prev_dt_s=prev_dt_s, step_as_rate=step_as_rate, speed_words=speed_words)
    except VD.DaggerRefused as exc:
        return (f"--vla-hierarchy-bottom-tile / --vla-hierarchy-prev-dt-s / "
                f"--vla-hierarchy-step-as-rate: {exc}")
    return None


# --------------------------------------------------------------------------- #
# The intent layer
# --------------------------------------------------------------------------- #
class VLAHierarchy(VD.VLAPerturber):
    """The hierarchical follow (module docstring). The loop calls, once per
    tick, plan(s, locked=, pose=, default_m=) before its controller - the
    stand-off it returns is the controller's `want_range` - and apply(smooth,
    s, pose=) at the perturbers' call site, which returns (the command for
    the Shield, the tick's record). start(t0) / stop() bracket the mission;
    put_front / put_down take the VLA cameras' messages.

    VLAPerturber's query thread, request building, hint, tracker, records
    and teardown are used as they are; its window schedule is not: the
    thread is asked to query from start() to stop() (its `_want_k` is 0
    throughout), and which ticks the VLA flies is decided here, by the
    intent and the controller's lock. Thread ownership as there: the
    tracker, the intent, the stand-off and the records are the loop's; the
    client's submit and poll, the capture and the hint the thread's."""

    def __init__(self, *, client, get_capture: Callable, latest_box: Callable, object_text,
                 tracker_config, policy=None, s_range=S_RANGE_M, expiry_s: float = EXPIRY_S,
                 s_rate_mps: float = S_RATE_MPS, fresh_s: float = 1.0, memory_s: float = 6.0,
                 search_s: float = 20.0, close=None, bearing_gain: float = 1.0,
                 identity: dict | None = None, address: str | None = None,
                 is_vla: bool | None = None, why_not: str | None = None,
                 config: dict | None = None, frames=None, record_dir=None,
                 clock: Callable[[], float] = time.time, poll_s: float = 0.01,
                 retry_s: float = 0.1, log: Callable[[str], Any] | None = None):
        why = check_range(s_range)
        if why is not None:
            raise ValueError(why)
        if not (math.isfinite(expiry_s) and expiry_s > 0):
            raise ValueError(f"expiry_s must be positive, got {expiry_s!r}")
        super().__init__(0, client=client, get_capture=get_capture, latest_box=latest_box,
                         object_text=object_text, tracker_config=tracker_config, policy=policy,
                         fresh_s=fresh_s, memory_s=memory_s, search_s=search_s, close=close,
                         bearing_gain=bearing_gain, identity=identity, address=address,
                         is_vla=is_vla, why_not=why_not, config=config, frames=frames,
                         record_dir=record_dir, clock=clock, poll_s=poll_s, retry_s=retry_s,
                         log=log)
        self.s_range = (float(s_range[0]), float(s_range[1]))
        self.expiry_s = float(expiry_s)
        self.standoff = StandOff(s_rate_mps)
        self.config.update({"s_range_m": list(self.s_range), "s_rate_mps": float(s_rate_mps),
                            "s_max_dt_s": self.standoff.max_dt_s, "expiry_s": self.expiry_s,
                            "fwd_range_m": list(FWD_RANGE_M)})
        # The loop's
        self.intent: dict | None = None         # the newest parsed answer, as an intent
        self._plan: dict | None = None          # this tick's, from plan() to apply()
        self._flying = False                    # is the tracker in the air (resumed)?
        self._closing = False
        self.handovers = 0                      # controller -> VLA hand-overs
        self.n_vla_flown = 0
        self.by_authority: Counter = Counter()
        self.by_intent: Counter = Counter()
        self.fallback_why: Counter = Counter()
        self.intents_set: Counter = Counter()
        self.flown_by_author: Counter = Counter()
        self._s_all: list[float] = []
        self._s_follow: list[float] = []
        self._s_target: list[float] = []
        self._answer_age: list[float] = []

    # ---- lifecycle ---------------------------------------------------------
    def start(self, t0: float, thread: bool = True) -> None:
        """The mission clock started at `t0`: open vla_steps.jsonl (rewritten),
        and ask from now on (`thread=False`: the caller runs _service_once
        itself - the tests). No frames are saved: VLAFrames keeps its log of
        what was sent, its saver stays None."""
        if self.t0 is not None:
            return
        self.t0 = float(t0)
        if self.record_dir is not None:
            self.record_dir.mkdir(parents=True, exist_ok=True)
            p = self.record_dir / VD.STEPS
            if p.is_file() and not p.is_symlink():
                p.unlink()
            self.steps_log = StepLog(p)
        with self._lock:
            self._want_k = 0                    # asked continuously, from the first tick
        if thread:
            self._thread = threading.Thread(target=self._run, name="vla-hierarchy",
                                            daemon=True)
            self._thread.start()

    def stop(self, timeout_s: float = 10.0) -> dict:
        """Stop asking; a parked answer is refused "closed"; then VLAPerturber's
        teardown (every request its line, the link and the log closed). The
        summary. Safe to call twice and before start()."""
        if self._summary_at_stop is None:
            self._closing = True
        return super().stop(timeout_s=timeout_s)

    # ---- the loop's side ---------------------------------------------------
    def plan(self, s: float, *, locked: bool, pose=None, default_m: float) -> float:
        """This tick's intent (module docstring): hand over what the thread
        collected, decide the authority, move the stand-off, tick the tracker
        on a VLA tick. Returns the stand-off for the controller (metres).
        `pose`: (x, y, up, yaw rad) of this tick's kinematics read. Never
        raises: a fault is counted, the tick falls back to the controller with
        its own stand-off."""
        now = self.clock()
        try:
            return self._plan_tick(bool(locked), pose, float(default_m), now)
        except Exception as exc:                                  # noqa: BLE001
            self._error(exc)
            self._plan = {"tk": None, "rec": {
                "source": SOURCE, "authority": "controller_fallback", "intent": None,
                "locked": bool(locked), "why": "error", "vla_flown": False,
                "error": f"{type(exc).__name__}: {exc}"}}
            return float(default_m)

    def apply(self, action, s: float, *, pose=None):
        """(the action to send to the Shield, the tick's record): the VLA
        tracker's command on a VLA tick it flies, else `action` itself (the
        controller's). Never raises: a fault is counted and `action` returned."""
        now = self.clock()
        try:
            out, rec = self._apply_tick(action, now)
        except Exception as exc:                                  # noqa: BLE001
            self._error(exc)
            out = action
            rec = {"source": SOURCE, "authority": "controller_fallback", "intent": None,
                   "why": "error", "vla_flown": False, "error": f"{type(exc).__name__}: {exc}"}
        self.n_ticks += 1
        a = rec.get("authority")
        self.by_authority[a] += 1
        self.by_intent[rec.get("intent") or "none"] += 1
        if a == "controller_fallback":
            self.fallback_why[rec.get("why") or "unknown"] += 1
        if rec.get("vla_flown"):
            self.n_vla_flown += 1
            self.n_active += 1
            self.flown_by_author[rec.get("author")] += 1
        if rec.get("s") is not None:
            self._s_all.append(rec["s"])
            self._s_target.append(rec["s_target"])
            if a == "controller_follow":
                self._s_follow.append(rec["s"])
        if rec.get("answer_age_s") is not None:
            self._answer_age.append(rec["answer_age_s"])
        self._last_exec = (out, now)
        return out, rec

    def _plan_tick(self, locked: bool, pose, default_m: float, now: float) -> float:
        self._plan = None
        pose_now = None if pose is None else Pose(*(float(v) for v in pose))
        got = self._collect(now)
        it = self.intent
        age = None if it is None else now - it["t_received"]
        why = None
        if it is None:
            authority, why = "controller_fallback", "no_answer"
        elif age > self.expiry_s:
            authority, why = "controller_fallback", "expired"
        elif it["kind"] == "land":
            authority = "hold"
        elif locked:
            authority = "controller_follow"
        else:
            authority = "vla_search"
        vla_tick = authority == "vla_search" or (authority == "hold" and not locked)
        if vla_tick and pose_now is None:
            # The tracker flies on the pose: without one the controller flies
            # (counted as a fault; the answers are still handed over below).
            self._error(ValueError("a VLA-flown tick needs the aircraft's pose"))
            authority, why, vla_tick = "controller_fallback", "no_pose", False
        if vla_tick and not self._flying:
            self._take_off(now, pose_now)
        elif not vla_tick and self._flying:
            self._land(now)
        self._hand_over(got, now, pose_now)
        target = (it["s_target"] if authority == "controller_follow" else
                  default_m if authority == "controller_fallback" else None)
        s_now = self.standoff.step(target, now, default_m)
        rec = {"source": SOURCE, "authority": authority, "intent": _INTENT_OF.get(authority),
               "locked": locked, "why": why, "answer_age_s": _r(age),
               "capture_age_s": None if it is None else _r(now - it["t_capture"]),
               "seq": None if it is None else it["seq"],
               "kind": None if it is None else it["kind"],
               "fwd_m": None if it is None else _r(it["fwd_m"]),
               "dpsi_rad": None if it is None else _r(it["dpsi_rad"]),
               "latency_s": None if it is None else _r(it["latency_s"]),
               "s": _r(s_now), "s_target": _r(s_now if target is None else target),
               "s_default": _r(default_m), "vla_flown": False}
        tk = None
        if vla_tick:
            hold = self._close_now(now)
            tk = self.tracker.tick(now, pose_now, pending=bool(self.client.busy),
                                   hold_horizontal=hold)
            rec["close_hold"] = hold
            if self.cfg.rate_horizon_s is not None:
                # vla_fly --step-as-rate: the commanded rates (None off a rate tick)
                rec["vla_rate"] = self.tracker.last_rate
        self._plan = {"tk": tk, "rec": rec}
        return s_now

    def _apply_tick(self, action, now: float):
        p, self._plan = self._plan, None
        if p is None:
            raise RuntimeError("apply() without this tick's plan()")
        rec, tk = p["rec"], p["tk"]
        if tk is None:
            return action, rec                  # the controller flies this tick
        flying = (tk.seq is not None and tk.author in _FLYING_AUTHORS
                  and tk.reason != "parse_fail" and not tk.land_requested)
        rec.update({"author": tk.author, "reason": tk.reason, "tk_seq": tk.seq,
                    "tk_age_s": _r(tk.age_s), "vla_cmd": _act(tk.action)})
        if flying:
            rec["vla_flown"] = True
            return tk.action, rec
        # No model decision to fly: the controller flies, the tracker is told.
        rec.update({"planned": rec["authority"], "authority": "controller_fallback",
                    "why": f"tracker_{tk.author}:{tk.reason}"})
        self.tracker.note_flown(action, now)
        return action, rec

    def _take_off(self, now: float, pose_now: Pose) -> None:
        """Controller -> VLA: resume the tracker (freshness is the intent's
        expiry, not a hold, so nothing captured earlier is invalidated), tell
        its smoothing the command in force, take the parked answer here."""
        self._flying = True
        self.handovers += 1
        if self.tracker.land_requested:
            self.tracker.clear_land()
        self.tracker.resume(-math.inf)
        if self._last_exec is not None:
            prev, t_prev = self._last_exec
            dt = self.cfg.smooth_tick_s
            if not 0.0 < now - t_prev <= 2.0 * dt:
                t_prev = now - dt
            self.tracker.note_flown(prev, t_prev)
        if self._parked is not None:
            d, self._parked = self._parked, None
            self.tracker.accept(d, now, pose_now=pose_now)
        self._flush()

    def _land(self, now: float) -> None:
        """VLA -> controller: the tracker suspended (its decision ends
        "suspended", its smoothing memory reset)."""
        self._flying = False
        self.tracker.suspend(now)
        self._flush()

    def _collect(self, now: float) -> list[tuple[str, Any]]:
        """The answers the thread collected, in order: [("dec", Decision) |
        ("lost", detail)]; each parsed step or LAND becomes the intent."""
        got = []
        while True:
            with self._lock:
                if not self._results:
                    break
                res, meta, _k = self._results.popleft()
            kind = res.reply.kind if res.reply is not None else "link_failure"
            self.answers[kind] += 1
            if res.reply is not None and res.reply.ok:
                self._latency.append(res.t_done - float(meta["t_capture"]))
            self._rtt.append(res.t_done - res.t_sent)
            try:
                dec = decision_from_result(meta, res)
            except Exception as exc:                              # noqa: BLE001
                self._error(exc)
                self._emit({"schema": VLA_STEPS_SCHEMA, "seq": res.seq, **meta, "kind": kind,
                            "accepted": False, "refused": "drain_error",
                            "drain_error": f"{type(exc).__name__}: {exc}", "target": None,
                            "outcome": None})
                got.append(("lost", f"drain: {type(exc).__name__}: {exc}"))
                continue
            if dec is None:
                rec = make_record(meta, res)
                rec.update({"accepted": False, "refused": "link_failure", "target": None,
                            "outcome": None})
                self.last_link_failure = {"kind": res.failure, "seq": res.seq, "t": now,
                                          "detail": res.detail}
                self._emit(rec)
                got.append(("lost", res.detail))
                continue
            if dec.kind in ("step", "land"):
                st = dec.step
                fwd = None if st is None else float(st.fwd_m)
                self.intent = {
                    "kind": dec.kind, "seq": dec.seq, "t_capture": dec.t_capture,
                    "t_received": dec.t_received, "latency_s": dec.t_received - dec.t_capture,
                    "fwd_m": fwd, "dpsi_rad": None if st is None else float(st.dpsi_rad),
                    "s_target": (desired_range(fwd, self.s_range)
                                 if dec.kind == "step" and fwd is not None else None)}
                self.intents_set[dec.kind] += 1
            got.append(("dec", dec))
        return got

    def _hand_over(self, got, now: float, pose_now: Pose | None) -> None:
        """Into the tracker at once while it flies; else parked, the newest
        winning (a link failure or a server error drops the parked one, as
        the tracker drops the step it flies on one)."""
        for what, x in got:
            if what == "lost":
                self._drop_parked("lost_while_parked")
                self.tracker.link_lost(now, x)
            elif self._flying or x.kind == "server_error":
                if x.kind == "server_error":
                    self._drop_parked("lost_while_parked")
                self.tracker.accept(x, now, pose_now=pose_now)
            else:
                self._drop_parked("superseded_while_parked")
                self._parked = x
        self._flush()

    def _drain(self, now: float, pose_now, in_window: bool) -> None:
        """VLAPerturber.stop()'s last collection: the answers still queued,
        handed over as on any tick; at the close a parked one is refused."""
        self._hand_over(self._collect(now), now, pose_now)
        if self._closing:
            self._drop_parked("closed")

    # ---- what it did -------------------------------------------------------
    def summary(self) -> dict:
        n = self.n_ticks

        def share(k: int):
            return round(k / n, 4) if n else None
        with self._lock:
            counts = dict(self.counts)
            errors = {"n": self.n_errors, "first": list(self.errors)}
        log = self.steps_log.snapshot() if self.steps_log is not None else None
        frames = self.frames.stats() if hasattr(self.frames, "stats") else None
        return {
            "source": SOURCE, "ticks": n,
            "authority": {a: self.by_authority.get(a, 0) for a in AUTHORITIES},
            "authority_share": {a: share(self.by_authority.get(a, 0)) for a in AUTHORITIES},
            "vla_flown_ticks": self.n_vla_flown, "vla_flown_share": share(self.n_vla_flown),
            "vla_flown_by_author": dict(self.flown_by_author),
            "intents": {k: self.by_intent.get(k, 0) for k in INTENTS + ("none",)},
            "intents_set": dict(self.intents_set),
            "fallback_why": dict(self.fallback_why),
            "s_m": _dist(self._s_all), "s_follow_m": _dist(self._s_follow),
            "s_target_m": _dist(self._s_target), "s_rate_limited_ticks": self.standoff.n_limited,
            "answer_age_s": _dist(self._answer_age), "handovers": self.handovers,
            "requests": counts.get("requests", 0), "no_capture": counts.get("no_capture", 0),
            "no_hint": counts.get("no_hint", 0),
            "submit_refused": counts.get("submit_refused", 0),
            "answers": dict(self.answers), "accepted": self.tracker.counts.get("accepted", 0),
            "refused": dict(self.refused),
            "latency_s": _spread(self._latency), "rtt_s": _spread(self._rtt),
            "server": self.server_record(), "config": self.config,
            "alt_band": self.alt_band, "tracker": dict(self.tracker.counts),
            "ticks_by_author": dict(self.tracker.ticks),
            "hint": self.detector_hint.stats(self.t0), "phrases": self.hint.stats(),
            "frames": frames, "link": dict(getattr(self.client, "counts", {}) or {}),
            "late_replies": list(getattr(self.client, "late", None) or []),
            "last_link_failure": self.last_link_failure, "steps_log": log, "errors": errors,
            "rule": RULE}

    def summary_line(self, st: dict | None = None) -> str:
        """One console line for the flight's end."""
        st = st if st is not None else self.summary()
        sh = st.get("authority_share") or {}
        lat = st.get("latency_s") or {}
        sf = st.get("s_follow_m") or {}
        return (f"[vla-hierarchy] {st['ticks']} ticks: follow {sh.get('controller_follow')}, "
                f"VLA search {sh.get('vla_search')}, hold {sh.get('hold')}, fallback "
                f"{sh.get('controller_fallback')} (VLA flew {st.get('vla_flown_share')}); "
                f"{st['requests']} requests, answers {st.get('answers')}, latency p50 "
                f"{lat.get('p50')} s; FOLLOW stand-off p50 {sf.get('p50')} m "
                f"[{sf.get('min')}, {sf.get('max')}]")

    def describe(self) -> str:
        sv = self.server_record()
        c = self.cfg
        near, far = self.s_range
        tile = getattr(self.frames, "tile", None)
        sw = getattr(self.frames, "speed_words", None)
        return (f"{VM.model_words(self.identity)} {sv['identity'].get('id')} at {self.address}"
                + ("" if self.is_vla else f" (*** STAND-IN, not a VLA: {self.why_not})")
                + f" decides the intent continuously (pipelined, held {self.expiry_s:g} s): "
                f"FOLLOW sets the controller's stand-off ({far:g} m at fwd 0 to {near:g} m at "
                f"fwd 5, at most {self.standoff.rate_mps:g} m/s), SEARCH flies vla_fly's follow "
                f"tracker at {c.cruise_mps:g}/{c.speed_cap_mps:g} m/s held at "
                f"{c.hold_altitude_m:g} m"
                + (f" with steps as rates over {c.rate_horizon_s:g} s" if c.rate_horizon_s
                   else "")
                + (f", lower tile the front frame {tile.dt_s:g} s earlier"
                   if tile is not None and tile.prev_front else "")
                + (f", its own speed as words ({sw.window_s:g} s window)" if sw is not None
                   else "")
                + "; LAND holds; the controller executes at 10 Hz; the Shield filters every "
                  "command")

    # ---- the flight's manifest and pilot record ----------------------------
    def manifest(self, policy_hash: str, seed: int, scene_path,
                 sim_address: str | None = None) -> dict:
        """The six-field manifest of a hierarchical flight: the VLA decides,
        so vla_model_hash is the server's reported digest
        (vla_mission.manifest_for), or the stand-in's id - never
        none:hand-written-controller. `sim_address` picks the topology label
        (guardrail.manifest.projectairsim_topology)."""
        return VM.manifest_for(policy_hash, self.identity, seed, scene_path,
                               weight_paths=VM.local_weight_paths(self.identity),
                               sim_address=sim_address)

    def pilot_entry(self, detector: dict | None = None) -> dict:
        """metrics["pilot"]: guardrail.manifest.pilot_record for the model that
        answered (kind vla with the server's reported digest; a stand-in is
        kind stub), the authority it had, the controller that executed and
        the detector that steered it; under "vla" the counts tools/todo.py
        reads, its ticks only the ones the VLA flew."""
        from guardrail.manifest import pilot_record
        ident = self.identity
        if self.is_vla:
            rec = pilot_record(VM.model_id_for(ident), "vla",
                               weight_paths=VM.local_weight_paths(ident),
                               reported_digest=ident["weights_sha256"],
                               reported_by=self.address)
        else:
            sid = str(ident.get("id") or "unknown")
            rec = pilot_record(sid if sid.startswith("fake:") else f"fake:{sid}", "stub")
            rec["not_a_vla_because"] = self.why_not
        st = self._summary_at_stop or self.summary()
        rec.update({
            "authority": "hierarchical",
            "controller": "demo/follow_vlm.py hand-written controller, 10 Hz",
            "detector": detector, "server_identity": identity_of(ident),
            "vla": {"decisions_accepted": self.tracker.counts.get("accepted", 0),
                    "identity": identity_of(ident),
                    "ticks_by_author": dict(self.flown_by_author),
                    "tracker": dict(self.tracker.counts),
                    "parse_fail": self.answers.get("parse_fail", 0),
                    "server_error": self.answers.get("server_error", 0),
                    "link_failure": self.answers.get("link_failure", 0)},
            "authority_share": st.get("authority_share"),
            "vla_flown_share": st.get("vla_flown_share")})
        return rec


def make_vla_hierarchy(addr: str, *, obs, box_source: Callable[[], dict | None], policy,
                       object_text, cruise_alt: float, record_dir=None, s_range=S_RANGE_M,
                       allow_stand_in: bool = False, encode: Callable | None = None,
                       cruise_mps: float = VD.FOLLOW_CRUISE_MPS,
                       speed_cap_mps: float = VD.FOLLOW_SPEED_CAP_MPS,
                       bottom_tile: str = "down", prev_dt_s: float | None = None,
                       step_as_rate: float | None = None, speed_words: bool = False,
                       front_source: str = "vla", expiry_s: float = EXPIRY_S,
                       log: Callable[[str], Any] | None = None) -> VLAHierarchy:
    """The --vla-hierarchy layer of a follow_vlm flight: vla_fly's follow
    configuration (vla_dagger.follow_args, with vla_fly's --vla-bottom-tile /
    --vla-prev-dt-s / --step-as-rate / --vla-speed-words when given), hello
    and the identity check (vla_dagger.connect_server), the lower tile and
    the speed words the adapter was trained with against this flight's
    (vla_mission.bottom_tile_check / speed_words_check), the VLA cameras' own
    VLAFrames over the flight's pose ring (`obs`), DetectorHint over
    `box_source` (follow_vlm.Grounder.latest). `front_source` (follow_vlm
    --vla-hierarchy-front-source; aerialvla_codec.FRONT_SOURCES): the camera
    follow_vlm feeds put_front with - VLAFront, or the detector's
    FrontCamera for a foreign server that reads it - checked against the
    server's (vla_mission.front_source_check). `expiry_s` (follow_vlm
    --vla-hierarchy-expiry-s): how long an answer's intent holds; raise it only
    for a model slower than EXPIRY_S per decision (DroneCATS int4 on the Orin
    Nano, ~9.6 s), and the record says so. HierarchyRefused for each refusal.
    Nothing is started: start(t0) does that, with the mission clock."""
    why = check_range(s_range)
    if why is not None:
        raise HierarchyRefused(why)
    obj0 = object_text() if callable(object_text) else object_text
    try:
        vargs, argv = VD.follow_args(obj0, cruise_alt=cruise_alt, cruise_mps=cruise_mps,
                                     speed_cap_mps=speed_cap_mps, bottom_tile=bottom_tile,
                                     prev_dt_s=prev_dt_s, step_as_rate=step_as_rate,
                                     speed_words=speed_words)
        tcfg = VD.follow_tracker_config(vargs)
        client, identity, is_vla, why_not = VD.connect_server(
            addr, allow_stand_in=allow_stand_in, response_timeout_s=vargs.decision_timeout_s,
            late_reply_wait_s=vargs.late_reply_wait_s,
            stand_in_flag="--vla-hierarchy-allow-stand-in")
    except VD.DaggerRefused as exc:
        raise HierarchyRefused(str(exc)) from None
    tile_check = VM.bottom_tile_check(identity, vargs.bottom_tile, is_vla=is_vla)
    if not tile_check["ok"]:
        client.close()
        raise HierarchyRefused(f"the server at {addr}: {tile_check['why']} (the adapter's lower "
                               f"tile from {tile_check['source']}; --vla-hierarchy-bottom-tile "
                               f"{vargs.bottom_tile.mode})")
    sw_check = VM.speed_words_check(identity, vargs.speed_words, is_vla=is_vla)
    if not sw_check["ok"]:
        client.close()
        raise HierarchyRefused(f"the server at {addr}: {sw_check['why']} (the adapter's speed "
                               f"words from {sw_check['source']}; --vla-hierarchy-speed-words "
                               f"{'on' if vargs.speed_words else 'off'})")
    # The camera follow_vlm feeds this layer with must be the one the server
    # reads (vla_mission.front_source_check): AerialVLA reads VLAFront, a
    # foreign server declares its own.
    front_check = VM.front_source_check(identity, front_source, is_vla=is_vla)
    if not front_check["ok"]:
        client.close()
        raise HierarchyRefused(f"the server at {addr}: {front_check['why']} "
                               f"(--vla-hierarchy-front-source {front_source})")
    try:
        frames = VM.VLAFrames(obs, encode=encode, bottom_tile=vargs.bottom_tile,
                              speed_words=vargs.speed_words)
        hier = VLAHierarchy(
            client=client, get_capture=frames.capture,
            latest_box=VD.latest_box_from(box_source), object_text=object_text,
            tracker_config=tcfg, policy=VM.pilot_alt_config(True, policy)[0], s_range=s_range,
            expiry_s=expiry_s, **VD.follow_hint_config(vargs), identity=identity, address=addr, is_vla=is_vla,
            why_not=why_not,
            config={"vla_fly_argv": argv, "cadence": vargs.cadence,
                    "decision_timeout_s": vargs.decision_timeout_s,
                    "late_reply_wait_s": vargs.late_reply_wait_s,
                    "hint": {"fresh_s": vargs.det_fresh_s, "memory_s": vargs.follow_memory_s,
                             "search_s": vargs.follow_search_s,
                             "close": [vargs.close_bottom_frac, vargs.close_width_frac],
                             "start_gate": None},
                    "tracker": asdict(tcfg), "step_as_rate_s": vargs.step_as_rate,
                    "bottom_tile": tile_check, "speed_words": sw_check,
                    "front_source": front_check},
            frames=frames, record_dir=record_dir, log=log)
        hier.front_source = front_source
        return hier
    except BaseException:
        client.close()
        raise


__all__ = ["VLAHierarchy", "StandOff", "HierarchyRefused", "make_vla_hierarchy", "check_args",
           "check_range", "desired_range", "SOURCE", "S_RANGE_M", "S_RATE_MPS", "MAX_DT_S",
           "EXPIRY_S", "AUTHORITIES", "INTENTS", "STEPS_DIR", "ROBOT_CONFIG", "RULE"]
