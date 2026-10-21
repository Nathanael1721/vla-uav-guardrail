"""
VLA backend adapters — the swappable slot.

The grant's core rule: the VLA is a *pluggable* backend. Everything downstream
(Compiler, Shield, MAVLink) only cares that whatever sits in the slot honours
ONE contract:

    observation  ->  Action4D(vx, vy, vz_up, yaw_rate)   at 10 Hz

That is the entire interface. "Plugging a model in" = writing a small adapter
class with a single `.act(state) -> Action4D` method that (1) gathers the
inputs the model wants, (2) runs the model, (3) maps its output onto the 4-D
contract. Nothing else in the stack changes.

THE FRAME. This project's Action4D is WORLD frame: vx +North, vy +East, vz_up
+up, yaw_rate rad/s (guardrail/models.py), because every rule the Shield checks
is world geometry. The grant locks body-frame velocities (vx forward, vy right),
so the frame is a recorded deviation, crossed exactly once by
guardrail.frames.from_body / to_body. A model that emits body-frame deltas (every
camera VLA does: it sees what is ahead of it) must be rotated at the heading the
frame was CAPTURED with before its action enters this slot. Skipping that is the
silent defect in docs/FINDING-forward-flew-north.md: facing East, "forward"
flew North. (Until 2026-10-06 this docstring called the contract body-frame.)

This file gives the Protocol + three adapters at increasing realism:
    StubBackend       hand-written, no model            (vla_stub.StubVLA)
    PolicyBackend     our trained state+geometry policy (vla_bc.BCVLAv3)
    RemoteVLABackend  AerialVLA, a REAL camera VLA, out of process over
                      protocol v2 (guardrail/vla_link.py, demo/vla_server.py)

Until 2026-10-21 the third slot was `AeroVLABackend`, a skeleton that raised
on its first model tick, with a synchronous "run the model every N ticks"
cadence that would have stalled the 10 Hz loop for the whole 2.5-12 s of an
AerialVLA inference. RemoteVLABackend replaces it: the model runs in its own
process, act() never waits for it, and every tick returns the step tracker's
action (docs/DESIGN-vla-pilot.md).
"""
from __future__ import annotations

import time
from dataclasses import replace
from typing import Callable, Protocol

from .models import Action4D, State


# --------------------------------------------------------------------------- #
# The contract every backend must satisfy — this IS the slot.
# --------------------------------------------------------------------------- #
class VLABackend(Protocol):
    def act(self, state: State) -> Action4D:
        """One 10 Hz tick: observation -> 4-D WORLD-frame Action4D (see the
        module docstring; body-frame model output goes through
        guardrail.frames.from_body first)."""
        ...

    # optional: called after a mid-flight policy change (hot-apply)
    def refresh_fences(self, policy) -> None: ...


# --------------------------------------------------------------------------- #
# The REAL VLA in the slot: AerialVLA (OpenVLA-7B + the aero_vla LoRA) served
# out of process, reached over protocol v2.
#
# Why out of process: a 7B model decides at 0.35 s per answer at best (the
# paper, RTX 4090) and 2.5-12 s as measured here on the old GPU, in process or
# not; in process it also shares the GIL with the 10 Hz loop (demo/vla_server.py
# header). So the model never runs inside act(): a worker thread holds the one
# outstanding request, and act() returns the step tracker's action every tick.
#
# What act() returns is WORLD frame already: the tracker turns each step into
# world-frame targets (heading psi_capture + dpsi, position along that
# heading, altitude) at the pose the image was captured at, and steers toward
# them, so no body->world rotation happens here (module docstring, THE FRAME).
# --------------------------------------------------------------------------- #
class RemoteVLABackend:
    """AerialVLA as the pilot, non-blocking.

    Every `act(state)`:
      1. takes the finished reply, if one came back, and hands it to the
         StepTracker (a decision) or tells it the link failed;
      2. returns the tracker's Action4D for this tick (who authored it is in
         `last_tick.author`: vla_step / hold / hover / none);
      3. sends the next request if none is outstanding: the freshest capture
         (`get_capture()`, JPEG bytes + capture pose and time) and its hint
         (`get_hint(capture)`, a vla_link.Hint, or None for "no hint, no
         request", e.g. no detection).

    Cadence. "pipelined" (default) sends the next capture as soon as the
    previous answer is back, while that answer's step is being flown, and the
    newer step supersedes it on arrival. "stop_and_go" is upstream's own
    cadence, as a labelled arm: the next capture is taken only after the
    current step is complete and the tracker has held for `settle_s`
    (upstream's 0.5 s hover).

    LAND sets `land_requested` (also on every tick's `last_tick`) and stops
    sending; the rail decides what to do with it. This class never requests a
    flight mode. `suspend()` / `resume()` bracket a rail hold: decisions
    captured before the hold ended are thrown away.

    Records: every request becomes one vla_steps.jsonl line (`steps_log`, a
    vla_link.StepLog or a path) when it stops being flown; `records` keeps
    them in memory too when no log is given.
    """

    CADENCES = ("pipelined", "stop_and_go")

    def __init__(self, client, get_capture: Callable, get_hint: Callable,
                 object_text: str = "", *, tracker=None, csp_mode: str = "off",
                 csp_text: str | None = None, cadence: str = "pipelined",
                 settle_s: float | None = None, steps_log=None,
                 clock: Callable[[], float] = time.monotonic):
        from .vla_link import (CSP_MODES, UPSTREAM_SETTLE_S, StepLog, StepTracker)
        if cadence not in self.CADENCES:
            raise ValueError(f"cadence {cadence!r} is not one of {self.CADENCES}")
        if csp_mode not in CSP_MODES:
            raise ValueError(f"csp_mode {csp_mode!r} is not one of {CSP_MODES}")
        self.client = client
        self.get_capture = get_capture
        self.get_hint = get_hint
        self.object_text = object_text
        self.tracker = tracker if tracker is not None else StepTracker()
        self.csp_mode, self.csp_text = csp_mode, csp_text
        self.cadence = cadence
        self.settle_s = UPSTREAM_SETTLE_S if settle_s is None else float(settle_s)
        if steps_log is not None and not isinstance(steps_log, StepLog):
            steps_log = StepLog(steps_log)
        self.steps_log = steps_log
        self.records: list[dict] = []
        self.clock = clock
        self._meta: dict[int, dict] = {}
        self.last_tick = None
        self.counts: dict[str, int] = {"no_capture": 0, "no_hint": 0, "requests": 0}
        self._t0: float | None = None

    # -- the slot ------------------------------------------------------------ #
    def act(self, state: State) -> Action4D:
        from .vla_link import Pose
        now = self.clock()
        if self._t0 is None:
            self._t0 = now
        self._drain(now)
        tick = self.tracker.tick(now, Pose.from_state(state))
        self._maybe_request(now)
        self._flush()
        self.last_tick = tick
        return tick.action

    def refresh_fences(self, policy) -> None:
        """A hot-applied policy: the tracker clamps altitude targets into the
        new band from the step being flown on. (The camera model reads no
        geometry; the Shield enforces the policy downstream.)"""
        from .vla_link import alt_band_from_policy
        self.tracker.set_alt_band(alt_band_from_policy(policy))

    # -- the rail's controls ------------------------------------------------- #
    @property
    def land_requested(self) -> bool:
        return self.tracker.land_requested

    def suspend(self) -> None:
        self.tracker.suspend(self.clock())
        self._flush()

    def resume(self) -> None:
        self.tracker.resume(self.clock())

    def clear_land(self) -> None:
        self.tracker.clear_land()

    def close(self) -> None:
        """Finish the open record, stop the client, close the log. A request
        still in flight gets its line too (refused "closed"), so the log has
        one line per request sent."""
        from .vla_link import VLA_STEPS_SCHEMA, make_record
        now = self.clock()
        self._drain(now)
        self.tracker.close(now)
        self._flush()
        self.client.close()
        late = self.client.poll()
        if late is not None and late.seq in self._meta:
            rec = make_record(self._meta.pop(late.seq), late)
            rec.update({"accepted": False, "refused": "closed", "target": None,
                        "outcome": None})
            self._emit(rec)
        for seq, meta in sorted(self._meta.items()):
            rec = {"schema": VLA_STEPS_SCHEMA, "seq": seq, **meta,
                   "kind": "unanswered", "accepted": False, "refused": "closed",
                   "target": None, "outcome": None}
            self._emit(rec)
        self._meta.clear()
        if self.steps_log is not None:
            self.steps_log.close()

    def stats(self) -> dict:
        """For metrics.json: decisions, refusals, ends, ticks by author, the
        decision rate over the time act() has run, and the link's counts."""
        t = self.tracker
        n = t.counts.get("accepted", 0)
        span = (self.clock() - self._t0) if self._t0 is not None else 0.0
        return {"cadence": self.cadence, "csp_mode": self.csp_mode,
                "decisions_accepted": n,
                "decision_hz": (n / span) if span > 0 else None,
                "tracker": dict(t.counts), "ticks_by_author": dict(t.ticks),
                "requests": dict(self.counts), "link": dict(self.client.counts),
                "identity": getattr(self.client, "identity", None)}

    # -- internals ----------------------------------------------------------- #
    def _drain(self, now: float) -> None:
        from .vla_link import decision_from_result, make_record
        res = self.client.poll()
        if res is None:
            return
        meta = self._meta.pop(res.seq)
        dec = decision_from_result(meta, res)
        if dec is None:
            rec = make_record(meta, res)
            rec.update({"accepted": False, "refused": "link_failure",
                        "target": None, "outcome": None})
            self.tracker.link_lost(now, res.detail)
            self._emit(rec)
            return
        self.tracker.accept(dec, now)

    def _maybe_request(self, now: float) -> None:
        from .vla_link import build_infer_fields
        t = self.tracker
        if t.land_requested or t.suspended or self.client.busy:
            return
        if self.cadence == "stop_and_go":
            idle = t.idle_since
            if idle is None or now - idle < self.settle_s:
                return
        if getattr(self.client, "backing_off", False):
            return
        cap = self.get_capture()
        if cap is None:
            self.counts["no_capture"] += 1
            return
        hint = self.get_hint(cap)
        if hint is None:
            self.counts["no_hint"] += 1
            return
        fields, meta = build_infer_fields(cap, hint, self.object_text,
                                          self.csp_mode, self.csp_text)
        seq = self.client.submit(fields)
        if seq is not None:
            self._meta[seq] = meta
            self.counts["requests"] += 1

    def _flush(self) -> None:
        for rec in self.tracker.pop_finished():
            self._emit(rec)

    def _emit(self, rec: dict) -> None:
        if self.steps_log is not None:
            self.steps_log.write(rec)
        else:
            self.records.append(rec)


# --------------------------------------------------------------------------- #
# Factory: choose the slot occupant by name. Wire this into run_demo (--vla).
# 'aerovla' is RemoteVLABackend: it needs a running VLA server (protocol v2,
# demo/vla_server.py) and a `get_capture` callable from the sim/flight loop.
# --------------------------------------------------------------------------- #
def make_backend(name: str, mission, policy, **kw) -> VLABackend:
    if name == "stub":
        from .vla_stub import StubVLA
        return StubVLA(mission)
    if name == "v3":
        from .vla_bc import BCVLAv3
        return BCVLAv3(mission, policy)
    if name == "aerovla":
        return _make_remote(mission, policy, **kw)
    raise ValueError(f"unknown VLA backend: {name!r}")


def _make_remote(mission, policy, *, get_capture, get_hint=None, client=None,
                 host: str | None = None, port: int | None = None,
                 object_text: str = "", tracker_config=None,
                 arrive_radius_m: float = 0.5, **kw) -> "RemoteVLABackend":
    """RemoteVLABackend for a mission.

    The hint defaults to the GOTO phrase toward (mission.target_x,
    mission.target_y) from each capture pose (hint_source "goal"); pass
    `get_hint` for a detector-derived phrase (demo/vla_bridge.py). The
    tracker's altitude band is the policy's hard altitude envelope. Keyword
    arguments of the old skeleton (model_dir, get_frame, vla_hz) are refused:
    the model is loaded by the server, frames come with their capture pose,
    and the decision rate is whatever the server delivers.
    """
    from .vla_link import (DEFAULT_HOST, DEFAULT_PORT, RemoteVLA, StepTracker,
                           TrackerConfig, alt_band_from_policy, goal_hint)
    old = sorted(k for k in ("model_dir", "get_frame", "vla_hz") if k in kw)
    if old:
        raise TypeError(f"make_backend('aerovla') no longer takes {old}: the model "
                        f"runs in the VLA server (demo/vla_server.py, protocol v2), "
                        f"frames come from get_capture with their capture pose, and "
                        f"the decision rate is the server's")
    allowed = {"csp_mode", "csp_text", "cadence", "settle_s", "steps_log", "clock"}
    unknown = sorted(set(kw) - allowed)
    if unknown:
        raise TypeError(f"make_backend('aerovla') got unknown arguments {unknown}")
    cfg = tracker_config or TrackerConfig()
    band = alt_band_from_policy(policy) if policy is not None else None
    if band is not None:
        cfg = replace(cfg, alt_band=band)
    if client is None:
        client = RemoteVLA(host or DEFAULT_HOST, DEFAULT_PORT if port is None else port,
                           response_timeout_s=cfg.decision_timeout_s,
                           **({"clock": kw["clock"]} if "clock" in kw else {}))
    if get_hint is None:
        get_hint = goal_hint((mission.target_x, mission.target_y), arrive_radius_m)
    return RemoteVLABackend(client, get_capture, get_hint, object_text,
                            tracker=StepTracker(cfg), **kw)
