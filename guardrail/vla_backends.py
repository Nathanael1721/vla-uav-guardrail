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
from collections import Counter
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

    act() never raises out of the link, the log or the caller's callbacks
    (review of 7c7cc50, P4 and P6): an exception from get_capture, get_hint
    or building the request is counted (`counts['request_errors']`, the last
    one's text in stats()) and the tick still returns the tracker's action;
    a reply that cannot be handed to the tracker is counted
    (`counts['drain_errors']`), keeps its request's line (refused
    `drain_error`) and is a lost link; vla_steps.jsonl is written by
    StepLog's own thread.

    The altitude band (`policy`, `refresh_fences`): the tracker clamps into
    the tighter of the band its TrackerConfig sets and the policy's hard
    above-ground envelopes (vla_link.combine_alt_bands); `alt_band_info`
    says which applied. A configured band that does not overlap the policy
    band is refused here; on a hot-apply the policy band wins, recorded.

    Records: every request becomes one vla_steps.jsonl line (`steps_log`, a
    vla_link.StepLog or a path) when it stops being flown; `records` keeps
    every one in memory as well, so a record whose write failed is not lost.
    """

    CADENCES = ("pipelined", "stop_and_go")

    def __init__(self, client, get_capture: Callable, get_hint: Callable,
                 object_text: str = "", *, tracker=None, policy=None,
                 csp_mode: str = "off",
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
        self.counts: dict[str, int] = {"no_capture": 0, "no_hint": 0, "requests": 0,
                                       "request_errors": 0, "drain_errors": 0,
                                       "alt_band_disjoint": 0}
        self.alt_band_info: dict = {}
        self._apply_policy_band(policy)
        if self.alt_band_info["source"] == "policy_disjoint":
            raise ValueError(f"the configured altitude band {self.alt_band_info['configured']} "
                             f"does not overlap the policy's {self.alt_band_info['policy']}: "
                             f"the tracker would clamp into heights the Shield refuses")
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
        # Every reply by kind (step / land / parse_fail / server_error /
        # link_failure), accepted by the tracker or not.
        self.answers: Counter = Counter()
        self.last_errors: dict[str, str] = {}
        self._t0: float | None = None

    # -- the slot ------------------------------------------------------------ #
    def act(self, state: State) -> Action4D:
        from .vla_link import Pose
        now = self.clock()
        if self._t0 is None:
            self._t0 = now
        self._safe_drain(now)
        tick = self.tracker.tick(now, Pose.from_state(state))
        try:
            self._maybe_request(now)
        except Exception as exc:                                  # noqa: BLE001
            # get_capture / get_hint are the caller's; Hint() and
            # build_infer_fields refuse bad values by raising. None of it may
            # stop the 10 Hz loop: this tick still returns the tracker's
            # action, and the next tick tries again.
            self._error("request_errors", exc)
        self._flush()
        self.last_tick = tick
        return tick.action

    def refresh_fences(self, policy) -> None:
        """A hot-applied policy: the tracker clamps altitude targets into the
        new band (combined with the configured one) from the step being flown
        on. (The camera model reads no geometry; the Shield enforces the
        policy downstream.)"""
        self._apply_policy_band(policy)

    def _apply_policy_band(self, policy) -> None:
        from .vla_link import combine_alt_bands, policy_alt_band
        info = policy_alt_band(policy)
        configured = self.tracker.cfg.alt_band
        band, source = combine_alt_bands(configured, info["band"])
        self.tracker.set_alt_band(band)
        if source == "policy_disjoint":
            self.counts["alt_band_disjoint"] += 1
        self.alt_band_info = {"band": band, "source": source, "configured": configured,
                              "policy": info["band"], "envelopes": info["envelopes"],
                              "skipped_msl": info["skipped_msl"]}

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
        self._safe_drain(now)
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
        rates over the time act() has run, and the link's counts.

        `decisions_accepted` / `decision_hz` count only answers the model
        parsed (a step or LAND) that the tracker took. Until the review of
        7c7cc50 (P1) they counted every accepted reply: a server refusing
        every request reported a 9.97 Hz decision rate. Malformed answers,
        server errors and link failures are counted and rated on their own
        (`answers` has every reply to a request sent, by kind, accepted or
        not).

        The log counts (`log_written`, `log_errors`, `last_log_error`,
        `log_pending`) are None when there is no steps_log: nothing was
        measured, which a 0 would hide (review of 39c447a, P2). With a log
        they are read without waiting for its writer thread (stats() may run
        on the loop), so `log_errors` is final only when `log_pending` is 0:
        after close(), or StepLog.flush() off the loop."""
        t = self.tracker
        n = t.counts.get("kind_step", 0) + t.counts.get("kind_land", 0)
        span = (self.clock() - self._t0) if self._t0 is not None else 0.0

        def rate(k: int) -> float | None:
            return (k / span) if span > 0 else None
        n_pf = self.answers.get("parse_fail", 0)
        n_se = self.answers.get("server_error", 0)
        n_lf = self.answers.get("link_failure", 0)
        log = (self.steps_log.snapshot() if self.steps_log is not None else
               dict.fromkeys(("written", "errors", "last_error", "pending")))
        return {"cadence": self.cadence, "csp_mode": self.csp_mode,
                "decisions_accepted": n, "decision_hz": rate(n),
                "answers": dict(self.answers),
                "parse_fail": n_pf, "parse_fail_hz": rate(n_pf),
                "server_error": n_se, "server_error_hz": rate(n_se),
                "link_failure": n_lf, "link_failure_hz": rate(n_lf),
                "request_errors": self.counts["request_errors"],
                "last_request_error": self.last_errors.get("request_errors"),
                "drain_errors": self.counts["drain_errors"],
                "last_drain_error": self.last_errors.get("drain_errors"),
                "log_written": log["written"], "log_errors": log["errors"],
                "last_log_error": log["last_error"], "log_pending": log["pending"],
                "alt_band": dict(self.alt_band_info),
                "tracker": dict(t.counts), "ticks_by_author": dict(t.ticks),
                "requests": dict(self.counts), "link": dict(self.client.counts),
                "identity": getattr(self.client, "identity", None)}

    # -- internals ----------------------------------------------------------- #
    def _error(self, key: str, exc: BaseException) -> None:
        self.counts[key] += 1
        self.last_errors[key] = f"{type(exc).__name__}: {exc}"

    def _safe_drain(self, now: float) -> None:
        try:
            self._drain(now)
        except Exception as exc:                                  # noqa: BLE001
            # A reply that could not be handed to the tracker is no answer:
            # hover, counted, and the loop goes on.
            self._error("drain_errors", exc)
            self.tracker.link_lost(now, f"drain: {type(exc).__name__}: {exc}")

    def _drain(self, now: float) -> None:
        from .vla_link import decision_from_result, make_record
        res = self.client.poll()
        if res is None:
            return
        # A reply to a seq this backend never sent raises KeyError here: no
        # request is owed a line, and it is no answer (`answers` counts the
        # replies to requests sent).
        meta = self._meta.pop(res.seq)
        kind = res.reply.kind if res.reply is not None else "link_failure"
        self.answers[kind] += 1
        try:
            dec = decision_from_result(meta, res)
            rec = make_record(meta, res) if dec is None else None
        except Exception as exc:                                  # noqa: BLE001
            # The request was sent and its meta is popped: it keeps its line,
            # one per request (close() keeps one for a request in flight the
            # same way), then _safe_drain treats the reply as a lost link.
            # Until the review of 39c447a (P1) the line was lost. Once the
            # tracker has the decision, the record is the tracker's.
            self._emit(self._unhandled_record(meta, res, kind, exc))
            raise
        if rec is not None:
            rec.update({"accepted": False, "refused": "link_failure",
                        "target": None, "outcome": None})
            self.tracker.link_lost(now, res.detail)
            self._emit(rec)
            return
        self.tracker.accept(dec, now)

    @staticmethod
    def _unhandled_record(meta: dict, res, kind: str, exc: BaseException) -> dict:
        """The line for a request whose reply could not be handed to the
        tracker: the reply's fields when make_record can still build them,
        else the request's own, refused `drain_error` with the error's text."""
        from .vla_link import VLA_STEPS_SCHEMA, make_record
        try:
            rec = make_record(meta, res)
        except Exception:                                         # noqa: BLE001
            rec = {"schema": VLA_STEPS_SCHEMA, "seq": res.seq, **meta, "kind": kind}
        rec.update({"accepted": False, "refused": "drain_error",
                    "drain_error": f"{type(exc).__name__}: {exc}",
                    "target": None, "outcome": None})
        return rec

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
        self.records.append(rec)
        if self.steps_log is not None:
            self.steps_log.write(rec)       # queued; written off this thread, never raises


# --------------------------------------------------------------------------- #
# Factory: choose the slot occupant by name. Wire this into run_demo (--vla).
# 'aerovla' is RemoteVLABackend: it needs a running VLA server (protocol v2,
# demo/vla_server.py), a `get_capture` callable from the sim/flight loop, and
# the hint's source (`get_hint`, or hint="goal" for a GOTO mission).
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


def _make_remote(mission, policy, *, get_capture, get_hint=None, hint: str | None = None,
                 client=None, host: str | None = None, port: int | None = None,
                 object_text: str = "", tracker_config=None,
                 arrive_radius_m: float = 0.5, **kw) -> "RemoteVLABackend":
    """RemoteVLABackend for a mission.

    The hint's source must be named: `get_hint` (a detector-derived phrase,
    the OWL-ViT box bearing: demo/vla_bridge.py) or `hint="goal"` for a GOTO
    mission (the phrase toward (mission.target_x, mission.target_y) from each
    capture pose, hint_source "goal"). Until the review of 7c7cc50 (P7) the
    goal phrase was the silent default for ANY mission, so a follow or
    approach mission wired without its detector hint flew on an oracle
    direction labelled as a GOTO goal; the brief allows the goal phrase for
    GOTO only. The tracker's altitude band is the tighter of
    tracker_config.alt_band and the policy's hard above-ground envelopes
    (RemoteVLABackend). Keyword arguments of the old skeleton (model_dir,
    get_frame, vla_hz) are refused: the model is loaded by the server, frames
    come with their capture pose, and the decision rate is whatever the
    server delivers.
    """
    from .vla_link import (DEFAULT_HOST, DEFAULT_PORT, RemoteVLA, StepTracker,
                           TrackerConfig, goal_hint)
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
    if get_hint is not None and hint is not None:
        raise TypeError("make_backend('aerovla'): give get_hint or hint='goal', not both")
    if get_hint is None:
        if hint != "goal":
            raise TypeError(
                f"make_backend('aerovla') needs the hint's source (got hint={hint!r}): "
                f"get_hint=... for a detector phrase from the OWL-ViT box bearing "
                f"(demo/vla_bridge.py), or hint='goal' for a GOTO mission, whose phrase "
                f"points at mission.target_x/target_y. The goal phrase is never a "
                f"default: on a follow or approach mission it is an oracle direction.")
        get_hint = goal_hint((mission.target_x, mission.target_y), arrive_radius_m)
    cfg = tracker_config or TrackerConfig()
    if client is None:
        client = RemoteVLA(host or DEFAULT_HOST, DEFAULT_PORT if port is None else port,
                           response_timeout_s=cfg.decision_timeout_s,
                           **({"clock": kw["clock"]} if "clock" in kw else {}))
    return RemoteVLABackend(client, get_capture, get_hint, object_text,
                            tracker=StepTracker(cfg), policy=policy, **kw)
