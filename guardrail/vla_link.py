"""
The pilot side of the AerialVLA link: the protocol v2 client, the step tracker
that turns each decision into the 10 Hz action stream, and the per-decision
records written to vla_steps.jsonl.

WHO FLIES

AerialVLA (OpenVLA-7B + the original `aero_vla` LoRA) runs in its own process,
the VLA server (vla-real env, GPU), and answers one request at a time over
newline-delimited JSON on TCP (protocol v2, below). Each answer is ONE STEP,
in the units upstream trained and executes it with:

    fwd_m     forward displacement, metres, 0..5
    down_m    vertical displacement, metres, -5..5, positive DOWN
    dpsi_rad  heading change, radians, -1.1..1.1, positive clockwise (right)
    land      the model's own "I have arrived" token

Upstream executes a step as: new heading = heading + dpsi; if |dpsi| >= 0.25
rad rotate only (plus the altitude change); otherwise move fwd_m along the new
heading and down_m vertically at 1.0 m/s; then hover 0.5 s
(airsim_plugin/AirVLNSimulatorClientTool_AeroVLA.py:300-385 upstream; arXiv
2603.14363, action space). Our earlier code read the same three numbers as m/s,
m/s and rad/s and held them for seconds; that mismatch, not only the decision
rate, is why those flights spun and overshot (discovery report vla-assets.md,
2026-10-21). Nothing here reads them as velocities - except on request:
our follow fine-tune's labels ARE displacements over a known horizon H
(tools/build_follow_dataset.py), and TrackerConfig.rate_horizon_s = H flies
such a step as fwd_m / H and dpsi_rad / H until the next answer (RATE
EXECUTION, off by default; demo/vla_fly.py --step-as-rate).

The model decides at well under 10 Hz (0.35 s per decision on an RTX 4090 in
the paper; 2.5-12 s measured on the old 4080 SUPER, in or out of process). The
grant's contract is a 4-D action at 10 Hz into the Shield. `StepTracker` meets
that contract by INTERPOLATING each step deterministically: a heading
P-controller toward psi_capture + dpsi with a yaw-rate cap, a translation toward
the step's endpoint at a cruise speed with a speed cap, an altitude P-controller
toward up_capture - down_m clamped into the policy's altitude band. It is not a
planner: it never chooses a direction the model did not choose, never avoids
anything (the Shield does that, every tick), and never flies backwards along the
step's heading. Say so wherever a run is described as "VLA-flown".

Each step is anchored in the WORLD at the pose the image was captured at, not at
the pose it arrives at. With pipelining (the next image is captured and sent
while the current step executes) a step can arrive after the aircraft has turned
or moved; anchoring at capture keeps the target heading psi_capture + dpsi
correct, where re-anchoring at arrival would add every turn made since the
capture a second time.

EVERY TICK SAYS WHO AUTHORED IT (`Tick.author`):

    vla_step  motion toward the endpoint of an accepted VLA step (with
              RATE EXECUTION: at its rates, reason "rate")
    hold      a deliberate stand-still that a VLA answer asked for: the step
              is complete (altitude still held at the step's target), the
              answer was malformed (parse_ok false: zero, counted), or the
              model said LAND (zero, latched)
    hover     the fallback when answers stop: the step being flown is older
              than `decision_timeout_s` (capture to now), a hold has lasted
              longer than that with no new answer, the link was lost, the
              server refused the request (ok=false), or a rail hold
              invalidated the decision; all-zero
    none      no VLA decision has been accepted yet (the reason says why:
              no_decision, link_lost, server_error), or the rail has the
              tracker suspended (an escalation hold is flying)

A SERVER ERROR IS NOT AN ANSWER. ok=false (CUDA out of memory, a refused
field) means the model never saw the request. Until the review of 7c7cc50 it
was accepted as a `hold` and reset the hold timer, so a server refusing every
request looked like a pilot holding on purpose at 10 Hz. It is now refused
like a link failure (the step being flown ends, the tracker hovers), the
client backs off before asking again, and the decision rate counts only
answers the model parsed.

NOTHING HERE MAY STOP THE 10 Hz LOOP. The worker thread posts a result for
every request whatever goes wrong in it; vla_steps.jsonl is written by its own
thread (StepLog), and a failed write is counted, never raised.

LAND IS A FLAG, NEVER A MODE REQUEST. A LAND answer stops the tracker and sets
`land_requested`; the rail hands that to the escalation FSM's Land (or its own
landing). Nothing in this module talks to an autopilot, names a flight mode or
imports a MAVLink library (tests/test_vla_link.py checks the source).

PROTOCOL v2 (the client half; the server is demo/vla_server.py)

TCP, default 127.0.0.1:8790, one JSON object per line, UTF-8, one request ->
one response, many requests per connection.

    {"v":2,"op":"hello"} -> {"v":2,"ok":true,"model":{...identity...}}
    {"v":2,"op":"infer","seq","t_capture","pose":{"x","y","up","yaw_rad"},
     "front_jpg_b64","down_jpg_b64"|null,"phrase","hint_source","object",
     "csp_text"|null,"csp_mode"}
    {"v":2,"op":"shutdown"}

The infer response carries the step (or null), land, parse_ok, raw_text, bins,
the server's model latency, the echoed t_capture and pose, the prompt and
mosaic hashes and the model identity. The client does not take parse_ok on
trust: an answer marked parse_ok must BE three 2-digit bins (plus " LAND"
exactly when land is true) equal to `bins`, with a step within half a bin of
their value; the model identity in every answer must be the one hello
announced, and every reconnect's hello must announce the same one
(docs/DESIGN-vla-pilot.md section 10). No request field can carry a target's
coordinates: the server refuses unknown fields, and refuses `target`, `goal`,
`object_position`, `xy`, `lat` and `lon` by name; `build_infer_fields` refuses
them before anything is sent. The hint reaches the model only as one of the
eight training phrases (seven compass phrases, or "" for "you have arrived").

Pose convention: the guardrail's world frame (guardrail/models.py): x metres
North, y metres East, up metres above ground, yaw_rad the heading in radians,
0 = North, clockwise positive (State.yaw_deg in radians). AerialVLA's dpsi is
clockwise-positive too (AirSim NED yaw), so heading + dpsi needs no sign flip.

TIME

t_capture, `now` and every timeout here are on ONE clock, the caller's (default
time.monotonic; a lock-step run passes its simulation clock). Socket timeouts
alone run on the real monotonic clock, because a socket waits in real time.

docs/DESIGN-vla-pilot.md records the hold policy, the timeouts, what the Shield
and FSM do with a slow pilot, and the open PI question on the decision rate.
"""
from __future__ import annotations

import base64
import hashlib
import json
import math
import re
import socket
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Callable

from .models import Action4D, State

# --------------------------------------------------------------------------- #
# Protocol v2 constants
# --------------------------------------------------------------------------- #
PROTOCOL_VERSION = 2
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8790

# The training vocabulary, verbatim (trailing space included): upstream
# src/model_wrapper/aerovla_wrapper_ui.py:46-76, demo/aerialvla_demo.py
# semantic_direction(). tests/test_vla_link.py pins it against both
# demo/vla_bridge.DIRECTION_VOCABULARY and semantic_direction()'s output.
DIRECTION_VOCABULARY: tuple[str, ...] = (
    "straight ahead ", "forward-right ", "forward-left ",
    "to your right ", "to your left ", "to your right rear ", "to your left rear ")
# No hint. In the upstream training set it appears once per trajectory, on the
# final step, with a zero label and LAND: the model learned it as "you have
# arrived" (vla-assets.md, 7,611 of 410,450 samples).
NO_HINT = ""
PHRASES = frozenset(DIRECTION_VOCABULARY) | {NO_HINT}
HINT_SOURCES = ("goal", "detector", "none")
CSP_MODES = ("off", "csp", "neutral")

INFER_FIELDS = ("v", "op", "seq", "t_capture", "pose", "front_jpg_b64",
                "down_jpg_b64", "phrase", "hint_source", "object", "csp_text",
                "csp_mode")
POSE_FIELDS = ("x", "y", "up", "yaw_rad")
# Any of these in a request could carry where the target is. The server
# refuses them by name; so does build_infer_fields, before a byte is sent.
FORBIDDEN_FIELDS = ("target", "goal", "object_position", "xy", "lat", "lon")
RESPONSE_FIELDS = ("v", "seq", "ok", "error", "raw_text", "bins", "step", "land",
                   "parse_ok", "latency_s", "t_capture", "pose", "prompt_sha256",
                   "image_sha256", "model")
STEP_FIELDS = ("fwd_m", "down_m", "dpsi_rad")

# Upstream's action ranges (99 bins each; demo/aerialvla_demo.py NORM).
FWD_RANGE_M = (0.0, 5.0)
DOWN_RANGE_M = (-5.0, 5.0)
DPSI_RANGE_RAD = (-1.1, 1.1)
STEP_RANGES = (FWD_RANGE_M, DOWN_RANGE_M, DPSI_RANGE_RAD)
NUM_BINS = 99
_RANGE_EPS = 1e-6

# The answer text a parse_ok reply must carry, as the server's parser accepts
# it (the brief: zero-padded 2-digit bins "FF DD YY", optional " LAND", then
# EOS; the server decodes with special tokens, so the text may end in the EOS
# marker "</s>"). [0-9], not \d: \d also matches other scripts' digits. The
# client re-reads the text because the brief's headline bug was a server-side
# parser (ints[-3:]) that turned malformed LAND outputs into motion
# ('00 49 494 49' -> a 5 m/s descent); a server still running one is refused
# here, not flown (review of 7c7cc50, P3).
ANSWER_TEXT_RE = re.compile(r"([0-9]{2}) ([0-9]{2}) ([0-9]{2})( LAND)?(?:</s>)?")

# The model identity hello announces and every answer repeats (the brief). The
# dashboard classifies VLA runs by weights_sha256, so a run whose identity has
# none, or whose model changes mid-run, must not pass unnoticed (review of
# 7c7cc50, P9). hello may carry serving facts beside these; only these six
# are compared.
IDENTITY_FIELDS = ("id", "base", "weights_sha256", "precision", "attn", "merged")
PRECISIONS = ("nf4", "bf16")
ATTN_IMPLS = ("eager", "sdpa")
# |dpsi| at or above this: rotate only (plus altitude), no horizontal move.
ROTATE_ONLY_RAD = 0.25
# Upstream executes every step at 1.0 m/s and hovers 0.5 s after it.
UPSTREAM_CRUISE_MPS = 1.0
UPSTREAM_SETTLE_S = 0.5

AUTHORS = ("vla_step", "hold", "hover", "none")
VLA_STEPS_SCHEMA = "vla_steps/1"
_RECV_SLICE_S = 0.1


class ProtocolError(Exception):
    """The server's reply breaks protocol v2: the stream cannot be trusted
    any more, so the client drops the connection."""


class ModelChanged(ProtocolError):
    """An answer, or a reconnect's hello, names another model than the one
    the run started with (counted `model_changed`)."""


class LinkError(Exception):
    """The connection failed: refused, reset, closed by the server."""


class LinkTimeout(LinkError):
    """No complete reply within the response timeout."""


def wrap_pi(a: float) -> float:
    """An angle in [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _finite(v: Any, what: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ValueError(f"{what} must be a finite number, got {v!r}")
    return float(v)


def _sha256(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


# --------------------------------------------------------------------------- #
# Value types
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Pose:
    """A pose in the guardrail's world frame (module docstring)."""
    x: float
    y: float
    up: float
    yaw_rad: float

    def __post_init__(self):
        for k in POSE_FIELDS:
            _finite(getattr(self, k), f"pose.{k}")

    @classmethod
    def from_state(cls, state: State) -> "Pose":
        return cls(float(state.x), float(state.y), float(state.up),
                   math.radians(float(state.yaw_deg)))

    def as_wire(self) -> dict:
        return {"x": self.x, "y": self.y, "up": self.up, "yaw_rad": self.yaw_rad}

    @classmethod
    def from_wire(cls, d: Any) -> "Pose":
        if not isinstance(d, dict) or set(d) != set(POSE_FIELDS):
            raise ValueError(f"a pose has exactly {POSE_FIELDS}, got {d!r}")
        return cls(*(_finite(d[k], f"pose.{k}") for k in POSE_FIELDS))


@dataclass(frozen=True)
class Step:
    """One AerialVLA step, upstream units (module docstring)."""
    fwd_m: float
    down_m: float
    dpsi_rad: float

    def __post_init__(self):
        for k, (lo, hi) in zip(STEP_FIELDS, (FWD_RANGE_M, DOWN_RANGE_M, DPSI_RANGE_RAD)):
            v = _finite(getattr(self, k), f"step.{k}")
            if not lo - _RANGE_EPS <= v <= hi + _RANGE_EPS:
                raise ValueError(f"step.{k} = {v} is outside AerialVLA's range [{lo}, {hi}]")

    @property
    def rotate_only(self) -> bool:
        return abs(self.dpsi_rad) >= ROTATE_ONLY_RAD

    def as_wire(self) -> dict:
        return {"fwd_m": self.fwd_m, "down_m": self.down_m, "dpsi_rad": self.dpsi_rad}

    @classmethod
    def from_wire(cls, d: Any) -> "Step":
        if not isinstance(d, dict) or set(d) != set(STEP_FIELDS):
            raise ValueError(f"a step has exactly {STEP_FIELDS}, got {d!r}")
        return cls(*(_finite(d[k], f"step.{k}") for k in STEP_FIELDS))


@dataclass(frozen=True)
class Capture:
    """One observation for the model: the front (and optional down) camera as
    JPEG bytes, and the pose and time they were captured at. The rail pairs
    them; this module never grabs a frame itself.

    `lower` (optional, never sent): what the down slot carries when it is
    NOT the down camera - aerialvla_codec's prev_front lower tile, the front
    frame from dt_s earlier (demo/vla_mission.VLAFrames): {"mode", "dt_s",
    "tol_s", "dt_actual_s", ...}. It goes into the request's record as
    `bottom_tile`; None is the down camera, as always before."""
    t_capture: float
    pose: Pose
    front_jpg: bytes
    down_jpg: bytes | None = None
    lower: dict | None = field(default=None, compare=False)

    def __post_init__(self):
        _finite(self.t_capture, "t_capture")
        if not isinstance(self.front_jpg, (bytes, bytearray)) or not self.front_jpg:
            raise ValueError("front_jpg must be non-empty JPEG bytes")
        if self.down_jpg is not None and (
                not isinstance(self.down_jpg, (bytes, bytearray)) or not self.down_jpg):
            raise ValueError("down_jpg must be non-empty JPEG bytes or None")


@dataclass(frozen=True)
class Hint:
    """The prompt's direction phrase and where it came from: `goal` (a GOTO
    mission's commanded goal), `detector` (the OWL-ViT box bearing,
    demo/vla_bridge.py) or `none` (no hint: the empty phrase).

    `kind` (optional, never sent): how the source arrived at the phrase,
    for the records only - a follow's detector phrase is a "fresh" box's,
    the remembered sighting's ("memory") or the search turn's ("search")
    (demo/vla_mission.DetectorHint; review of d6e7123, P2-2). It goes into
    the request's record as `hint_kind`; equality is phrase and source."""
    phrase: str
    source: str
    kind: str | None = field(default=None, compare=False)

    def __post_init__(self):
        if self.kind is not None and (not isinstance(self.kind, str) or not self.kind
                                      or self.kind != self.kind.strip()):
            raise ValueError(f"hint kind {self.kind!r} is not a plain word")
        if self.phrase not in PHRASES:
            raise ValueError(f"phrase {self.phrase!r} is not one of the training "
                             f"phrases {sorted(PHRASES)}")
        if self.source not in HINT_SOURCES:
            raise ValueError(f"hint source {self.source!r} is not one of {HINT_SOURCES}")
        if self.source == "none" and self.phrase != NO_HINT:
            raise ValueError("hint source 'none' sends the empty phrase only")
        if self.source == "detector" and self.phrase == NO_HINT:
            # demo/vla_bridge.py: no detection means no inference, never "".
            raise ValueError("a detector hint is a compass phrase; with no detection "
                             "send no request (demo/vla_bridge.py)")


def phrase_for_bearing(bearing_rad: float) -> str:
    """The training band for a body-frame bearing (positive = right of the
    nose), with semantic_direction()'s exact boundaries: the angle is wrapped
    to [-180, 180) degrees and a boundary belongs to the more central band."""
    ang = math.degrees(bearing_rad)
    ang = (ang + 180.0) % 360.0 - 180.0
    if -15.0 <= ang <= 15.0:
        return "straight ahead "
    if 15.0 < ang <= 60.0:
        return "forward-right "
    if 60.0 < ang <= 120.0:
        return "to your right "
    if 120.0 < ang <= 180.0:
        return "to your right rear "
    if -60.0 <= ang < -15.0:
        return "forward-left "
    if -120.0 <= ang < -60.0:
        return "to your left "
    return "to your left rear "


def goal_phrase(pose: Pose, goal_xy: tuple[float, float],
                arrive_radius_m: float = 0.5) -> str:
    """A GOTO mission's hint: the compass phrase toward the commanded goal from
    the capture pose, or "" within `arrive_radius_m` of it (the phrase the
    model learned as "you have arrived"). This is the model's trained task
    (upstream computes the phrase from the goal position,
    src/vlnce_src/closeloop_util.py:120); only the phrase leaves this process.
    The 0.5 m default is semantic_direction()'s."""
    dx, dy = float(goal_xy[0]) - pose.x, float(goal_xy[1]) - pose.y
    if math.hypot(dx, dy) < arrive_radius_m:
        return NO_HINT
    return phrase_for_bearing(math.atan2(dy, dx) - pose.yaw_rad)


def goal_hint(goal_xy: tuple[float, float], arrive_radius_m: float = 0.5
              ) -> Callable[["Capture"], Hint]:
    """A `get_hint` callable for a GOTO mission (hint_source "goal")."""
    gx, gy = _finite(goal_xy[0], "goal x"), _finite(goal_xy[1], "goal y")
    r = _finite(arrive_radius_m, "arrive_radius_m")

    def get_hint(capture: Capture) -> Hint:
        return Hint(goal_phrase(capture.pose, (gx, gy), r), "goal")
    return get_hint


def policy_alt_band(policy) -> dict:
    """The intersection of the policy's HARD above-ground altitude envelopes:
    {"band": (lo, hi) | None, "envelopes": [ids used], "skipped_msl": [ids]}.

    The tracker clamps each step's altitude target into it, so the pilot does
    not ask for a height the Shield would have to refuse; the Shield still
    enforces the band itself. Time-windowed envelopes are included whatever
    the clock says (a tighter band is the conservative side for a pilot-side
    clamp), and corridor bands are not (they hold only inside their tube).
    An envelope with altitude_ref MSL is skipped and named: its heights are
    not above ground, the pose's `up` is, and the Shield itself refuses MSL
    envelopes (no ground-elevation source, Shield.unenforceable); until the
    review of 7c7cc50 (P8) a 120-300 m MSL band was clamped as metres above
    ground. An empty intersection gives band None: the policy lint refuses
    such a policy (guardrail/models.py), and a clamp into nothing has no
    answer."""
    from .models import AltitudeEnvelope
    lo, hi = -math.inf, math.inf
    used: list[str] = []
    msl: list[str] = []
    for c in getattr(policy, "constraints", None) or ():
        if not isinstance(c, AltitudeEnvelope) or getattr(c, "constraint_type", "hard") != "hard":
            continue
        if getattr(c, "altitude_ref", None) == "MSL":
            msl.append(c.id)
            continue
        lo, hi = max(lo, float(c.alt_min_m)), min(hi, float(c.alt_max_m))
        used.append(c.id)
    band = (lo, hi) if used and lo < hi else None
    return {"band": band, "envelopes": used, "skipped_msl": msl}


def alt_band_from_policy(policy) -> tuple[float, float] | None:
    """`policy_alt_band(policy)["band"]`."""
    return policy_alt_band(policy)["band"]


def combine_alt_bands(configured, from_policy) -> tuple[tuple[float, float] | None, str]:
    """(the band the tracker clamps into, which one it is).

    `configured` is the caller's TrackerConfig.alt_band, `from_policy` the
    policy's (policy_alt_band). Both: their intersection ("intersection"),
    so a policy never widens a band the caller set, and a policy without an
    envelope never removes it (until the review of 7c7cc50, P8, the policy
    band replaced the configured one, and a hot-applied policy without an
    envelope dropped the clamp). One: that one ("configured" / "policy").
    Neither: None ("none"). Two bands that do not overlap: the POLICY band
    ("policy_disjoint"), because the Shield enforces the policy band, not the
    configured one; the backend refuses that at construction and records it
    on a hot-apply."""
    if configured is not None:
        _check_band(configured)
    if from_policy is not None:
        _check_band(from_policy)
    if configured is None and from_policy is None:
        return None, "none"
    if from_policy is None:
        return (float(configured[0]), float(configured[1])), "configured"
    pol = (float(from_policy[0]), float(from_policy[1]))
    if configured is None:
        return pol, "policy"
    lo, hi = max(float(configured[0]), pol[0]), min(float(configured[1]), pol[1])
    if lo < hi:
        return (lo, hi), "intersection"
    return pol, "policy_disjoint"


# --------------------------------------------------------------------------- #
# Requests and replies
# --------------------------------------------------------------------------- #
def check_request_fields(msg: dict) -> None:
    """Refuse a request that carries anything the protocol does not name, and
    above all anything that could carry a target's position."""
    bad = sorted(k for k in msg if k in FORBIDDEN_FIELDS)
    if bad:
        raise ValueError(f"refused: {bad} could carry a target's coordinates")
    if msg.get("op") == "infer":
        if set(msg) != set(INFER_FIELDS):
            extra = sorted(set(msg) - set(INFER_FIELDS))
            missing = sorted(set(INFER_FIELDS) - set(msg))
            raise ValueError(f"an infer request has exactly the protocol's fields "
                             f"(unknown {extra}, missing {missing})")
        if not isinstance(msg["pose"], dict) or set(msg["pose"]) != set(POSE_FIELDS):
            raise ValueError(f"the pose has exactly {POSE_FIELDS}")


def build_infer_fields(capture: Capture, hint: Hint, object_text: str,
                       csp_mode: str = "off", csp_text: str | None = None
                       ) -> tuple[dict, dict]:
    """(request fields without `seq`, record metadata without image bytes).

    csp_mode "off" sends no CSP text (the prompt is then byte-identical to
    AerialVLA's template); "csp" and "neutral" need the text (the neutral
    arm is the length-matched filler, the null of tools/prefix_eval.py)."""
    if not isinstance(capture, Capture):
        raise TypeError("capture must be a Capture")
    if not isinstance(hint, Hint):
        raise TypeError("hint must be a Hint")
    if not isinstance(object_text, str):
        raise TypeError("object_text must be a string")
    if csp_mode not in CSP_MODES:
        raise ValueError(f"csp_mode {csp_mode!r} is not one of {CSP_MODES}")
    if csp_mode == "off" and csp_text is not None:
        raise ValueError("csp_mode 'off' sends no CSP text")
    if csp_mode != "off" and (not isinstance(csp_text, str) or not csp_text):
        raise ValueError(f"csp_mode {csp_mode!r} needs the CSP text")
    front = bytes(capture.front_jpg)
    down = bytes(capture.down_jpg) if capture.down_jpg is not None else None
    fields = {
        "v": PROTOCOL_VERSION, "op": "infer",
        "t_capture": float(capture.t_capture),
        "pose": capture.pose.as_wire(),
        "front_jpg_b64": base64.b64encode(front).decode("ascii"),
        "down_jpg_b64": base64.b64encode(down).decode("ascii") if down is not None else None,
        "phrase": hint.phrase, "hint_source": hint.source,
        "object": object_text, "csp_text": csp_text, "csp_mode": csp_mode,
    }
    meta = {
        "t_capture": float(capture.t_capture),
        "pose_capture": capture.pose.as_wire(),
        "phrase": hint.phrase, "hint_source": hint.source, "object": object_text,
        "csp_mode": csp_mode, "csp_text": csp_text,
        "csp_sha256": _sha256(csp_text.encode("utf-8")) if csp_text else None,
        "front_jpg_sha256": _sha256(front), "front_jpg_bytes": len(front),
        "down_jpg_sha256": _sha256(down) if down is not None else None,
        "down_jpg_bytes": len(down) if down is not None else None,
    }
    if hint.kind is not None:
        meta["hint_kind"] = hint.kind       # the record's only: never on the wire
    if capture.lower is not None:
        # the down slot is the front frame from dt_s earlier (Capture.lower):
        # the record says so, and the measured dt; the wire cannot
        meta["bottom_tile"] = dict(capture.lower)
    return fields, meta


@dataclass(frozen=True)
class InferReply:
    """A validated infer response."""
    ok: bool
    error: str | None
    raw_text: str | None
    bins: tuple[int, int, int] | None
    step: Step | None
    land: bool
    parse_ok: bool
    latency_s: float | None
    prompt_sha256: str | None
    image_sha256: str | None
    model: dict | None
    extra_keys: tuple[str, ...] = ()

    @property
    def kind(self) -> str:
        if not self.ok:
            return "server_error"
        if not self.parse_ok:
            return "parse_fail"
        return "land" if self.land else "step"


def _is_int(v) -> bool:
    return type(v) is int


def _is_hex64(v) -> bool:
    return isinstance(v, str) and len(v) == 64 and all(c in "0123456789abcdef" for c in v)


def check_identity(model: Any) -> None:
    """Raise ValueError unless `model` is the brief's model identity: id and
    base non-empty strings, weights_sha256 64 lowercase hex digits, precision
    nf4 or bf16, attn eager or sdpa, merged true or false. Other keys (the
    serving facts a hello may add) are allowed."""
    if not isinstance(model, dict):
        raise ValueError(f"the model identity is not an object: {model!r}")
    missing = [k for k in IDENTITY_FIELDS if k not in model]
    if missing:
        raise ValueError(f"the model identity lacks {missing}")
    for k in ("id", "base"):
        if not isinstance(model[k], str) or not model[k]:
            raise ValueError(f"model.{k} must be a non-empty string, got {model[k]!r}")
    if not _is_hex64(model["weights_sha256"]):
        raise ValueError(f"model.weights_sha256 must be 64 lowercase hex digits, "
                         f"got {model['weights_sha256']!r}")
    if model["precision"] not in PRECISIONS:
        raise ValueError(f"model.precision {model['precision']!r} is not one of {PRECISIONS}")
    if model["attn"] not in ATTN_IMPLS:
        raise ValueError(f"model.attn {model['attn']!r} is not one of {ATTN_IMPLS}")
    if not isinstance(model["merged"], bool):
        raise ValueError(f"model.merged must be true or false, got {model['merged']!r}")


def identity_of(model: Any) -> dict:
    """The six identity fields of a model object (None where absent): what
    is compared between hello and every answer."""
    m = model if isinstance(model, dict) else {}
    return {k: m.get(k) for k in IDENTITY_FIELDS}


def dequantize_bin(b: int, lo: float, hi: float) -> float:
    """Bin -> value at the lower edge of its cell, b/98 * (hi - lo) + lo:
    upstream's decoder, and the server's (the inverse of int truncation)."""
    return b / (NUM_BINS - 1) * (hi - lo) + lo


def _check_answer_text(text: str, bins: tuple[int, int, int], step: Step, land: bool) -> None:
    """A parse_ok reply's raw_text must be the answer it claims (ANSWER_TEXT_RE
    comment): three 2-digit bins equal to `bins`, " LAND" exactly when `land`,
    and each step value within half a bin of its bin's value."""
    m = ANSWER_TEXT_RE.fullmatch(text)
    if m is None:
        raise ProtocolError(f"parse_ok true, but raw_text {text!r} is not an answer (three "
                            f"2-digit bins, optional ' LAND', optional EOS): a parser that "
                            f"reads it as one is not trusted")
    got = (int(m.group(1)), int(m.group(2)), int(m.group(3)))
    if got != bins:
        raise ProtocolError(f"bins {list(bins)} are not raw_text {text!r}")
    if (m.group(4) is not None) != land:
        raise ProtocolError(f"land {land} but raw_text {text!r}")
    for k, b, (lo, hi) in zip(STEP_FIELDS, bins, STEP_RANGES):
        want, v = dequantize_bin(b, lo, hi), getattr(step, k)
        if abs(v - want) > 0.5 * (hi - lo) / (NUM_BINS - 1) + 1e-9:
            raise ProtocolError(f"step.{k} = {v} is not bin {b}'s value {want:.6g} "
                                f"(within half a bin)")


def parse_infer_response(raw: Any, sent: dict, identity: dict | None = None) -> InferReply:
    """Validate an infer response against the request it answers. Raises
    ProtocolError on anything protocol v2 does not allow: a wrong version or
    sequence number, a missing field, a step outside AerialVLA's range, a
    malformed answer that still carries a step, an answer marked parse_ok
    whose raw_text is not that answer, or an echo of t_capture or the pose
    that is not what was sent (a reply to some other request). With
    `identity` (the hello's), an answer from another model raises
    ModelChanged."""
    if not isinstance(raw, dict):
        raise ProtocolError(f"the reply is not a JSON object: {type(raw).__name__}")
    if raw.get("v") != PROTOCOL_VERSION or isinstance(raw.get("v"), bool):
        raise ProtocolError(f"protocol version {raw.get('v')!r}, expected {PROTOCOL_VERSION}")
    ok = raw.get("ok")
    if not isinstance(ok, bool):
        raise ProtocolError(f"'ok' must be true or false, got {ok!r}")
    # A refusal may carry seq null (the server could not read one); anything
    # else must be this request's seq.
    seq = raw.get("seq")
    if (seq is not None or ok) and not (_is_int(seq) and seq == sent["seq"]):
        raise ProtocolError(f"reply seq {seq!r} for request seq {sent['seq']}")
    extra = tuple(sorted(set(raw) - set(RESPONSE_FIELDS)))
    if not ok:
        err = raw.get("error")
        if not isinstance(err, str) or not err:
            raise ProtocolError("a refused request must say why ('error')")
        lat = raw.get("latency_s")
        model = raw.get("model")
        return InferReply(False, err, raw.get("raw_text") if isinstance(raw.get("raw_text"), str) else None,
                          None, None, False, False,
                          float(lat) if isinstance(lat, (int, float)) and not isinstance(lat, bool)
                          and math.isfinite(lat) else None,
                          None, None, model if isinstance(model, dict) else None, extra)
    missing = [k for k in RESPONSE_FIELDS if k not in raw]
    if missing:
        raise ProtocolError(f"the reply lacks {missing}")
    if raw["error"] is not None:
        raise ProtocolError("an ok reply carries error null")
    for k in ("land", "parse_ok"):
        if not isinstance(raw[k], bool):
            raise ProtocolError(f"'{k}' must be true or false")
    if not isinstance(raw["raw_text"], str):
        raise ProtocolError("'raw_text' must be a string")
    bins = raw["bins"]
    if bins is not None:
        if not (isinstance(bins, list) and len(bins) == 3
                and all(_is_int(b) and 0 <= b <= NUM_BINS - 1 for b in bins)):
            raise ProtocolError(f"'bins' must be three integers 0..{NUM_BINS - 1}, got {bins!r}")
        bins = (bins[0], bins[1], bins[2])
    step = None
    if raw["step"] is not None:
        try:
            step = Step.from_wire(raw["step"])
        except ValueError as exc:
            raise ProtocolError(f"bad step: {exc}") from None
    if raw["parse_ok"]:
        if step is None or bins is None:
            raise ProtocolError("parse_ok true needs the decoded step and its bins")
        _check_answer_text(raw["raw_text"], bins, step, raw["land"])
    else:
        if step is not None or raw["land"]:
            raise ProtocolError("parse_ok false must carry step null and land false: "
                                "a malformed answer is never motion")
    try:
        lat = _finite(raw["latency_s"], "latency_s")
    except ValueError as exc:
        raise ProtocolError(str(exc)) from None
    if lat < 0:
        raise ProtocolError("latency_s is negative")
    try:
        t_echo = _finite(raw["t_capture"], "t_capture")
        pose_echo = Pose.from_wire(raw["pose"])
    except ValueError as exc:
        raise ProtocolError(f"bad echo: {exc}") from None
    if abs(t_echo - float(sent["t_capture"])) > 1e-9:
        raise ProtocolError(f"t_capture echo {t_echo} is not the {sent['t_capture']} sent")
    sp = sent["pose"]
    if any(abs(getattr(pose_echo, k) - float(sp[k])) > 1e-9 for k in POSE_FIELDS):
        raise ProtocolError(f"pose echo {raw['pose']!r} is not the pose sent")
    for k in ("prompt_sha256", "image_sha256"):
        if not _is_hex64(raw[k]):
            raise ProtocolError(f"'{k}' must be 64 lowercase hex digits")
    if not isinstance(raw["model"], dict):
        raise ProtocolError("'model' must be an object")
    if identity is not None and identity_of(raw["model"]) != identity_of(identity):
        raise ModelChanged(f"the model changed mid-run: this answer is from "
                           f"{identity_of(raw['model'])}, hello announced {identity_of(identity)}")
    return InferReply(True, None, raw["raw_text"], bins, step, raw["land"],
                      raw["parse_ok"], lat, raw["prompt_sha256"], raw["image_sha256"],
                      raw["model"], extra)


@dataclass(frozen=True)
class LinkResult:
    """What came back for one request: a validated reply, or the failure."""
    seq: int
    request: dict
    reply: InferReply | None
    t_sent: float
    t_done: float
    # "timeout" | "link" | "protocol" | "internal" (anything else that went
    # wrong in the worker: the result is still posted) | None
    failure: str | None = None
    detail: str = ""
    # The response timeout in force for THIS request (submit's
    # response_timeout_s, else the client's): what a record may claim.
    timeout_s: float | None = None


# --------------------------------------------------------------------------- #
# The client
# --------------------------------------------------------------------------- #
class RemoteVLA:
    """Protocol v2 client. One request outstanding at a time, by construction.

    The 10 Hz loop never blocks on it: `submit()` hands a request to a worker
    thread and returns at once (None while one is still outstanding or its
    result is still unread, or while backing off), and `poll()` returns the
    finished `LinkResult` once. Failures close the connection; the next
    request reconnects (after `reconnect_backoff_s`) and says hello again. A
    reply that breaks protocol v2 is a failure too ("protocol"): after it the
    stream cannot be trusted, so it is never read as a decision. A server
    error (ok=false) keeps the connection but backs off just the same: asking
    a server that refuses every request again on every tick only fills the
    log (review of 7c7cc50, P1).

    The first hello's model identity is pinned (`identity`): it must be the
    brief's identity (check_identity) unless `strict_identity=False` - for a
    scripted stand-in with no weights, said out loud by the caller - and
    every answer and every later hello must name the same model, or the
    reply is a protocol failure counted `model_changed`.

    A SLOW MODEL (`late_reply_wait_s`). A request that times out is still
    being computed: demo/vla_server_v2.py runs generate() under one model
    lock and cannot cancel it, so a request sent on a new connection waits
    behind the abandoned one, and with answers slower than the timeout every
    later request times out too (review of the CityLife flight, P2-5). With
    `late_reply_wait_s` > 0 the timeout is still posted at once (the loop
    hovers on it), but the connection is kept and the worker reads on until
    the abandoned request's late answer arrives - the moment the model lock
    is free - or `late_reply_wait_s` passes (then it drops the connection);
    meanwhile `busy` is true and `submit()` refuses (`submit_draining`).
    Late answers are counted (`late_replies`) and kept in `late` (seq, the
    server's latency, how late), never handed to the loop: a step that
    arrives after the timeout is from a scene that no longer exists. 0 (the
    default) drops the connection on a timeout, as before.

    `connect()`, `hello()` and `shutdown_server()` block; call them before the
    flight loop or from tools, never from a control tick.
    """

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, *,
                 connect_timeout_s: float = 2.0, response_timeout_s: float = 10.0,
                 reconnect_backoff_s: float = 1.0, max_line_bytes: int = 8 << 20,
                 strict_identity: bool = True, late_reply_wait_s: float = 0.0,
                 clock: Callable[[], float] = time.monotonic):
        self.host, self.port = host, int(port)
        self.connect_timeout_s = _finite(connect_timeout_s, "connect_timeout_s")
        self.response_timeout_s = _finite(response_timeout_s, "response_timeout_s")
        self.reconnect_backoff_s = _finite(reconnect_backoff_s, "reconnect_backoff_s")
        self.late_reply_wait_s = _finite(late_reply_wait_s, "late_reply_wait_s")
        if self.connect_timeout_s <= 0 or self.response_timeout_s <= 0 \
                or self.reconnect_backoff_s < 0 or self.late_reply_wait_s < 0:
            raise ValueError("timeouts must be positive and the backoff non-negative")
        self.max_line_bytes = int(max_line_bytes)
        self.strict_identity = bool(strict_identity)
        self.clock = clock
        self.identity: dict | None = None
        self._sock: socket.socket | None = None
        self._buf = b""
        self._io = threading.Lock()
        self._cv = threading.Condition()
        self._pending: tuple[int, dict, float] | None = None   # seq, request, timeout
        self._result: LinkResult | None = None
        self._busy = False
        self._draining = False
        self._closed = False
        self._worker: threading.Thread | None = None
        self._next_seq = 1
        self._retry_at = -math.inf
        self.counts: Counter = Counter()
        self.late: deque = deque(maxlen=100)

    # -- blocking primitives (worker thread, or before the loop) ------------- #
    def connect(self) -> dict:
        """Open the connection if it is closed and say hello. Returns the
        server's model identity."""
        with self._io:
            self._ensure_connected()
            return dict(self.identity or {})

    def hello(self) -> dict:
        return self.connect()

    def shutdown_server(self) -> dict | None:
        """Ask the server to exit (op shutdown). Best effort."""
        with self._io:
            try:
                self._ensure_connected()
                self._send({"v": PROTOCOL_VERSION, "op": "shutdown"})
                return self._read(time.monotonic() + self.response_timeout_s)
            except (OSError, LinkError, ProtocolError):
                return None
            finally:
                self._drop()

    def _ensure_connected(self) -> None:
        if self._sock is not None:
            return
        if self._closed:
            raise LinkError("the client is closed")
        try:
            s = socket.create_connection((self.host, self.port),
                                         timeout=self.connect_timeout_s)
        except OSError as exc:
            raise LinkError(f"connect {self.host}:{self.port}: {exc}") from None
        try:
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        self._sock, self._buf = s, b""
        self.counts["connects"] += 1
        try:
            self._send({"v": PROTOCOL_VERSION, "op": "hello"})
            rep = self._read(time.monotonic() + self.response_timeout_s)
            if rep.get("v") != PROTOCOL_VERSION or rep.get("ok") is not True \
                    or not isinstance(rep.get("model"), dict):
                raise ProtocolError(f"bad hello reply: {rep!r}")
            model = rep["model"]
            if self.strict_identity:
                try:
                    check_identity(model)
                except ValueError as exc:
                    raise ProtocolError(f"bad model identity in hello: {exc}") from None
            if self.identity is None:
                self.identity = model
            elif identity_of(model) != identity_of(self.identity):
                # A server restarted with other weights: one run, one model.
                raise ModelChanged(f"the model changed mid-run: hello announces "
                                   f"{identity_of(model)}, the run started with "
                                   f"{identity_of(self.identity)}")
        except BaseException:
            self._drop()
            raise

    def _send(self, msg: dict) -> None:
        data = json.dumps(msg, separators=(",", ":"), ensure_ascii=False,
                          allow_nan=False).encode("utf-8") + b"\n"
        assert self._sock is not None
        self._sock.settimeout(self.response_timeout_s)
        try:
            self._sock.sendall(data)
        except socket.timeout:
            raise LinkTimeout("send timed out") from None
        except OSError as exc:
            raise LinkError(f"send: {exc}") from None

    def _read(self, deadline: float, timeout_s: float | None = None) -> dict:
        assert self._sock is not None
        while b"\n" not in self._buf:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise LinkTimeout(f"no reply within "
                                  f"{self.response_timeout_s if timeout_s is None else timeout_s:g} s")
            if self._closed:
                raise LinkError("the client was closed while waiting for a reply")
            # Short slices, so close() is seen within ~0.1 s: on Windows a
            # shutdown() from another thread does not wake a blocked recv.
            self._sock.settimeout(min(remaining, _RECV_SLICE_S))
            try:
                chunk = self._sock.recv(65536)
            except socket.timeout:
                continue
            except OSError as exc:
                raise LinkError(f"recv: {exc}") from None
            if not chunk:
                raise LinkError("the server closed the connection")
            self._buf += chunk
            if len(self._buf) > self.max_line_bytes and b"\n" not in self._buf:
                raise ProtocolError(f"a reply line longer than {self.max_line_bytes} bytes")
        line, _, self._buf = self._buf.partition(b"\n")

        def _no_constants(name):
            raise ValueError(f"{name} is not JSON")
        try:
            obj = json.loads(line.decode("utf-8"), parse_constant=_no_constants)
        except (UnicodeDecodeError, ValueError) as exc:
            raise ProtocolError(f"the reply is not one JSON line: {exc}") from None
        except RecursionError:
            # '[' * 200000: json recurses per level (review of 7c7cc50, P5).
            raise ProtocolError("the reply nests deeper than the JSON reader follows") from None
        if not isinstance(obj, dict):
            raise ProtocolError("the reply is not a JSON object")
        return obj

    def _drop(self) -> None:
        s, self._sock, self._buf = self._sock, None, b""
        if s is not None:
            try:
                s.close()
            except OSError:
                pass

    # -- the non-blocking surface for the control loop ----------------------- #
    @property
    def busy(self) -> bool:
        """A request is in flight, or its result has not been polled yet, or
        the server is still computing a request that timed out
        (`late_reply_wait_s`)."""
        with self._cv:
            return self._busy or self._result is not None or self._draining

    @property
    def draining(self) -> bool:
        """Waiting for a timed-out request's late answer (`late_reply_wait_s`)."""
        with self._cv:
            return self._draining

    @property
    def backing_off(self) -> bool:
        return self.clock() < self._retry_at

    def submit(self, fields: dict, response_timeout_s: float | None = None) -> int | None:
        """Send one infer request (fields from `build_infer_fields`, without
        `seq`) unless one is already outstanding, the last one's result has
        not been polled, or the client is backing off after a failure or a
        server error. Returns the request's seq, or None.

        `response_timeout_s`: this request's own answer timeout instead of the
        client's (the warm-up's first request, the model's first inference
        ever, gets first_decision_timeout_s: review of 4a0da6d, P1-1 - until
        then the record said 60 s while the socket gave up at the client's
        10 s). The result carries the one in force (LinkResult.timeout_s).

        The unpolled check: until the review of 7c7cc50 (P2) a second request
        could go out while the first one's result sat unread (act() polls,
        then ticks, then submits), and the worker then overwrote that result:
        a step or a LAND that never reached the tracker."""
        with self._cv:
            if self._closed:
                raise RuntimeError("the client is closed")
            if self._busy:
                self.counts["submit_busy"] += 1
                return None
            if self._result is not None:
                self.counts["submit_unpolled"] += 1
                return None
            if self._draining:
                self.counts["submit_draining"] += 1
                return None
            if self.clock() < self._retry_at:
                self.counts["submit_backoff"] += 1
                return None
            timeout = (self.response_timeout_s if response_timeout_s is None
                       else _finite(response_timeout_s, "response_timeout_s"))
            if timeout <= 0:
                raise ValueError("response_timeout_s must be positive")
            msg = dict(fields)
            msg["seq"] = self._next_seq
            check_request_fields(msg)
            self._next_seq += 1
            self._pending = (msg["seq"], msg, timeout)
            self._busy = True
            self.counts["sent"] += 1
            if self._worker is None:
                self._worker = threading.Thread(target=self._run, name="vla-link",
                                                daemon=True)
                self._worker.start()
            self._cv.notify_all()
            return msg["seq"]

    def poll(self) -> LinkResult | None:
        """The finished request's result, once; None while it is in flight."""
        with self._cv:
            res, self._result = self._result, None
            return res

    def wait_idle(self, timeout_s: float) -> bool:
        """Block until no request is outstanding (tools and tests only)."""
        end = time.monotonic() + timeout_s
        with self._cv:
            while self._busy:
                left = end - time.monotonic()
                if left <= 0:
                    return False
                self._cv.wait(left)
            return True

    def wait_drained(self, timeout_s: float) -> bool:
        """Block until no timed-out request is still being waited for
        (`late_reply_wait_s`; tools and tests only)."""
        end = time.monotonic() + timeout_s
        with self._cv:
            while self._draining:
                left = end - time.monotonic()
                if left <= 0:
                    return False
                self._cv.wait(left)
            return True

    def close(self) -> None:
        with self._cv:
            self._closed = True
            self._cv.notify_all()
        # A worker waiting for a reply sees the flag within one recv slice;
        # one still connecting gives up at the connect timeout.
        w = self._worker
        if w is not None and w is not threading.current_thread():
            w.join(timeout=self.connect_timeout_s + 2 * _RECV_SLICE_S + 1.0)
        with self._io:
            self._drop()

    def _run(self) -> None:
        while True:
            with self._cv:
                while self._pending is None and not self._closed:
                    self._cv.wait()
                if self._closed:
                    return
                seq, msg, timeout = self._pending
            t_sent = self.clock()
            reply, failure, detail, changed, drain = None, None, "", False, False
            with self._io:
                try:
                    self._ensure_connected()
                    self._send(msg)
                    reply = parse_infer_response(
                        self._read(time.monotonic() + timeout, timeout), msg, self.identity)
                except LinkTimeout as exc:
                    failure, detail = "timeout", str(exc)
                    # Sent and not answered in time: the server is still on
                    # it. Keep the connection and wait for that answer below.
                    drain = self.late_reply_wait_s > 0 and self._sock is not None
                except ProtocolError as exc:
                    failure, detail = "protocol", str(exc)
                    changed = isinstance(exc, ModelChanged)
                except (LinkError, OSError, ValueError) as exc:
                    failure, detail = "link", str(exc)
                except Exception as exc:                          # noqa: BLE001
                    # Anything else must not kill this thread: it would die
                    # with _busy set, and every later submit() would return
                    # None with no failure recorded (review of 7c7cc50, P5).
                    failure, detail = "internal", f"{type(exc).__name__}: {exc}"
                if failure is not None and not drain:
                    self._drop()
            t_done = self.clock()
            with self._cv:
                if failure is not None:
                    self.counts[f"fail_{failure}"] += 1
                    if changed:
                        self.counts["model_changed"] += 1
                    self._retry_at = t_done + self.reconnect_backoff_s
                else:
                    self.counts["replies"] += 1
                    if not reply.ok:
                        # The stream is still in step, so the connection
                        # stays; the backoff paces a server that refuses
                        # every request (review of 7c7cc50, P1).
                        self.counts["server_error"] += 1
                        self._retry_at = t_done + self.reconnect_backoff_s
                if drain:
                    self._draining = True
                self._pending = None
                self._result = LinkResult(seq, msg, reply, t_sent, t_done, failure, detail,
                                          timeout)
                self._busy = False
                self._cv.notify_all()
            if drain:
                self._await_late_reply(seq, msg, t_sent, timeout)

    def _await_late_reply(self, seq: int, msg: dict, t_sent: float,
                          timeout: float | None = None) -> None:
        """After a timeout (`late_reply_wait_s`): read the kept connection
        until the abandoned request's own answer arrives, so the next request
        is not queued behind it on the server's model lock. Anything else -
        another line, a broken stream, the wait running out, close() - drops
        the connection (the next request reconnects, as without the wait)."""
        why = None
        with self._io:
            try:
                deadline = time.monotonic() + self.late_reply_wait_s
                while True:
                    try:
                        obj = self._read(deadline)
                    except LinkTimeout:
                        why = "late_reply_wait_expired"
                        break
                    rep = parse_infer_response(obj, msg, self.identity)
                    t_late = self.clock()
                    with self._cv:
                        self.counts["late_replies"] += 1
                        self.late.append({"seq": seq, "kind": rep.kind,
                                          "latency_server_s": rep.latency_s,
                                          "rtt_s": t_late - t_sent,
                                          "late_by_s": t_late - t_sent - (
                                              self.response_timeout_s if timeout is None
                                              else timeout)})
                    break
            except (LinkError, ProtocolError, OSError, ValueError) as exc:
                why = f"late_reply_failed: {type(exc).__name__}"
            except Exception as exc:                              # noqa: BLE001
                why = f"late_reply_failed: {type(exc).__name__}"
            if why is not None:
                self._drop()
        with self._cv:
            if why is not None:
                self.counts["late_reply_dropped"] += 1
                self.late.append({"seq": seq, "dropped": why})
            self._draining = False
            self._cv.notify_all()


# --------------------------------------------------------------------------- #
# Decisions and their records
# --------------------------------------------------------------------------- #
@dataclass
class Decision:
    """One accepted-or-refused model answer, ready for the tracker."""
    seq: int
    t_capture: float
    pose: Pose
    kind: str                       # step | land | parse_fail | server_error
    step: Step | None
    t_received: float
    record: dict = field(default_factory=dict)

    @classmethod
    def of(cls, seq: int, t_capture: float, pose: Pose, step: Step | None = None, *,
           land: bool = False, kind: str | None = None,
           t_received: float | None = None) -> "Decision":
        """A decision built without the link (offline replay, tests)."""
        if kind is None:
            kind = "land" if land else ("step" if step is not None else "parse_fail")
        if kind not in ("step", "land", "parse_fail", "server_error"):
            raise ValueError(f"unknown decision kind {kind!r}")
        if kind == "step" and step is None:
            raise ValueError("a step decision needs its step")
        t_rx = float(t_capture if t_received is None else t_received)
        rec = {"schema": VLA_STEPS_SCHEMA, "seq": seq, "kind": kind,
               "t_capture": float(t_capture), "t_received": t_rx,
               "pose_capture": pose.as_wire(),
               "step": step.as_wire() if step is not None else None,
               "land": kind == "land", "source": "scripted"}
        return cls(seq, float(t_capture), pose, kind, step, t_rx, rec)


def make_record(meta: dict, result: LinkResult) -> dict:
    """The vla_steps.jsonl row for one request, before the tracker adds what
    became of it (`accepted`, `refused`, `target`, `outcome`)."""
    rep = result.reply
    rec: dict[str, Any] = {"schema": VLA_STEPS_SCHEMA, "seq": result.seq}
    rec.update(meta)
    rec.update({
        "t_sent": result.t_sent, "t_received": result.t_done,
        "rtt_s": result.t_done - result.t_sent,
        "latency_client_s": result.t_done - float(meta["t_capture"]),
    })
    if rep is None:
        rec.update({"kind": "link_failure",
                    "failure": {"kind": result.failure, "detail": result.detail}})
        return rec
    rec.update({
        "kind": rep.kind, "failure": None,
        "ok": rep.ok, "error": rep.error, "raw_text": rep.raw_text,
        "bins": list(rep.bins) if rep.bins is not None else None,
        "step": rep.step.as_wire() if rep.step is not None else None,
        "land": rep.land, "parse_ok": rep.parse_ok,
        "latency_server_s": rep.latency_s,
        "prompt_sha256": rep.prompt_sha256, "image_sha256": rep.image_sha256,
        "model": rep.model,
    })
    if rep.extra_keys:
        rec["reply_unknown_keys"] = list(rep.extra_keys)
    return rec


def decision_from_result(meta: dict, result: LinkResult) -> Decision | None:
    """The tracker's view of a reply; None for a link failure."""
    if result.reply is None:
        return None
    rep = result.reply
    return Decision(result.seq, float(meta["t_capture"]),
                    Pose.from_wire(meta["pose_capture"]), rep.kind,
                    rep.step if rep.kind in ("step", "land") else None,
                    result.t_done, make_record(meta, result))


class StepLog:
    """vla_steps.jsonl: one line per request, written when the decision stops
    being flown (so the line carries what became of it). LF line endings on
    every platform, flushed per line.

    `write()` only queues the record and never raises: a writer thread
    serializes and writes it. Until the review of 7c7cc50 (P4) the write and
    flush ran on the 10 Hz loop thread inside act(), and an OSError went
    straight out of act() - a full disk (D: runs near full) would have ended
    the flight loop. The repo's rails keep file IO off the loop thread the
    same way (follow_vlm writes detections on its worker thread). A failed
    write (OSError, or a record that is not JSON) is counted in `errors`
    with `last_error`; the caller keeps its own copy (RemoteVLABackend.records).
    `n` counts lines written. `close()` drains the queue, then closes the
    file. Opening the file can raise: that happens before the flight loop.
    `snapshot()` reads the counts with the number still queued, so a count
    read before the queue drained is not taken for the final one."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, "a", encoding="utf-8", newline="\n")
        self.n = 0
        self.errors = 0
        self.last_error: str | None = None
        self._q: deque = deque()
        self._pending = 0
        self._closed = False
        self._cv = threading.Condition()
        self._writer = threading.Thread(target=self._run, name="vla-steps-log", daemon=True)
        self._writer.start()

    def write(self, rec: dict) -> None:
        """Queue one record; returns at once, never raises."""
        with self._cv:
            if self._closed:
                self._fail("the log is closed")
                return
            self._q.append(rec)
            self._pending += 1
            self._cv.notify_all()

    def snapshot(self) -> dict:
        """`written` (n), `errors`, `last_error` and `pending` (queued, not
        yet written or failed), read together under the lock and without
        waiting for the writer, so a control tick may call it. `errors` is
        final only when `pending` is 0: after flush() or close(). (A writer
        that hung past close()'s timeout leaves `pending` above 0.)"""
        with self._cv:
            return {"written": self.n, "errors": self.errors,
                    "last_error": self.last_error, "pending": self._pending}

    def flush(self, timeout_s: float = 5.0) -> bool:
        """Block until every queued record is written or failed (tools,
        tests and close(); never from a control tick)."""
        end = time.monotonic() + timeout_s
        with self._cv:
            while self._pending:
                left = end - time.monotonic()
                if left <= 0:
                    return False
                self._cv.wait(left)
            return True

    def close(self, timeout_s: float = 5.0) -> None:
        with self._cv:
            self._closed = True
            self._cv.notify_all()
        self._writer.join(timeout=timeout_s)
        if self._writer.is_alive():
            # A write hung (a dead network drive): leave the file to it.
            with self._cv:
                self._fail(f"the writer did not finish within {timeout_s:g} s")
            return
        try:
            if not self._fh.closed:
                self._fh.close()
        except Exception as exc:                                   # noqa: BLE001
            with self._cv:
                self._fail(f"{type(exc).__name__}: {exc}")

    def _fail(self, why: str) -> None:
        # Under self._cv.
        self.errors += 1
        self.last_error = why

    def _run(self) -> None:
        while True:
            with self._cv:
                while not self._q and not self._closed:
                    self._cv.wait()
                if not self._q:
                    return                      # closed, and everything queued is done
                rec = self._q.popleft()
            why = None
            try:
                self._fh.write(json.dumps(rec, ensure_ascii=False, allow_nan=False) + "\n")
                self._fh.flush()
            except Exception as exc:                               # noqa: BLE001
                why = f"{type(exc).__name__}: {exc}"
            with self._cv:
                if why is None:
                    self.n += 1
                else:
                    self._fail(why)
                self._pending -= 1
                self._cv.notify_all()


# --------------------------------------------------------------------------- #
# The step tracker: step + capture pose -> 10 Hz world-frame Action4D
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class TrackerConfig:
    """The interpolator's constants. Defaults follow upstream where it has
    one (cruise 1.0 m/s); the rest are this tracker's own and are recorded
    with every run, never presented as the model's."""
    cruise_mps: float = UPSTREAM_CRUISE_MPS   # horizontal speed along a step
    speed_cap_mps: float = 2.0                # hard ceiling on |v_h|
    climb_cap_mps: float = 1.0                # |vz| ceiling (upstream 1.0 m/s)
    yaw_rate_cap_rad_s: float = 0.5           # 28.6 deg/s
    k_yaw: float = 1.5                        # 1/s, heading P gain
    k_pos: float = 1.0                        # 1/s, slows into the endpoint
    k_alt: float = 1.0                        # 1/s, altitude P gain
    pos_tol_m: float = 0.3
    alt_tol_m: float = 0.3
    yaw_tol_rad: float = 0.05                 # 2.9 deg
    # Per-step time cap: factor x (the step's own expected duration) + base,
    # at most time_cap_max_s. A 5 m step at 1 m/s gets 9.5 s.
    time_cap_factor: float = 1.5
    time_cap_base_s: float = 2.0
    time_cap_max_s: float = 15.0
    # A step is flown only while now - t_capture <= this: a 5 m step at the
    # upstream 1 m/s plus ~3 s of latency (the old GPU) fits, and an older one
    # is from a scene that no longer exists. A hold (step complete, or a zero
    # hold) lasts at most this long before it becomes hover: the pilot has
    # gone silent.
    decision_timeout_s: float = 10.0
    alt_band: tuple[float, float] | None = None
    alt_band_inset_m: float = 0.5
    hover_on_link_loss: bool = True
    # A FOLLOW executor's choices (2026-10-23, the user's "make the VLA
    # follow the target"); both default off, so every other run keeps
    # upstream's execution. turn_while_moving: a step whose heading change
    # is at least ROTATE_ONLY_RAD still flies its fwd_m along the new
    # heading (upstream rotates in place), the speed scaled by cos(heading
    # error) so it slows while it turns. hold_altitude_m: the altitude
    # target is this (inside the band), not up_capture - down_m - the
    # model's vertical output is recorded, not flown.
    turn_while_moving: bool = False
    hold_altitude_m: float | None = None
    # anchor_at_answer (follow): a step's horizontal displacement starts from
    # where the aircraft is when the answer is taken, not from the capture
    # pose. Upstream flies stop-and-go, where the two are the same place;
    # pipelined, the aircraft has already flown most of a capture-anchored
    # step by the time its answer arrives (~1.3 s in flight on 23 Oct) and
    # slows into an endpoint it has nearly passed (follow #3: 1.54 m/s median
    # at a 4.5 m/s cruise). The heading target stays capture yaw + dpsi.
    anchor_at_answer: bool = False
    # land_as_arrival (follow): AerialVLA was trained to answer LAND once it
    # has found the target; following a car, that is "caught up", not "land
    # on it" (follow #5, 23 Oct: LAND 0.08 m from the car ended the chase
    # after 17 s). The answer is held as a zero step (author hold, reason
    # land) until the next decision; land_requested is never set. Counted.
    land_as_arrival: bool = False
    # SMOOTHING (follow, 2026-10-23; all off by default, so every other run
    # keeps the tracker's motion as it was). The user found the VLA follow
    # "very not smooth": follow #9 (vla_follow_redcar_09_20261023) commanded
    # a horizontal acceleration of p95 / p99 / max 3.75 / 32.68 / 54.32 m/s^2
    # and a yaw acceleration of p99 / max 6.87 / 18.41 rad/s^2, against the
    # hand-written follow's 2.60 / 3.90 / 5.56 and 0.50 / 2.09 (19 Oct,
    # citylife_redcar_nfz_motion2_20261019; tools/compare_motion.py, the
    # same definitions). Its largest steps were a new answer superseding a
    # step (the velocity re-pointed and re-sped in one tick, the speed having
    # decayed into the old endpoint at k_pos), holds between steps and the
    # heading controller jumping to its cap. Three opt-in answers:
    #   smooth_accel_mps2: the commanded horizontal velocity changes by at
    #     most this x dt per tick, as a vector (its direction kept), dt the
    #     time since the last tick capped at smooth_tick_s - follow_vlm's
    #     VecSlewLimiter rule (its --accel-max 2.5). With it, a step is
    #     approached at min(cruise, sqrt(2 a d)) - the fastest speed from
    #     which that limit still stops at the endpoint - instead of k_pos x
    #     d, which slowed every step from 4.5 m out.
    #   smooth_yaw_accel_rps2: the commanded yaw rate changes by at most this
    #     x dt per tick, and the heading is approached at
    #     min(cap, k_yaw |e|, sqrt(2 alpha |e|)) so the slew cannot overshoot.
    #   carry_s: a step flown to its end while the next answer is PENDING (a
    #     request in flight; StepTracker.tick(pending=True)) keeps its last
    #     horizontal velocity, decaying linearly to zero over carry_s, instead
    #     of stopping (author vla_step, reason "carry", counted): the
    #     aircraft no longer halts between successive steps. It flies on past
    #     the step's endpoint by up to about half v x carry_s - this
    #     executor's choice, never the model's. With no request in flight
    #     (stop-and-go, no hint, a failed request) the step ends in a hold as
    #     before.
    # With smoothing on, every output goes through the limits: a hold, a
    # hover or a close hold (StepTracker.tick(hold_horizontal=True), the
    # follow's close box) ramps down at the limit instead of stopping in one
    # tick, and is counted (smooth_decay_ticks). The Shield still judges, and
    # repairs, every tick after this.
    smooth_accel_mps2: float | None = None
    smooth_yaw_accel_rps2: float | None = None
    carry_s: float = 0.0
    smooth_tick_s: float = 0.1
    # RATE EXECUTION (the follow fine-tune, 2026-10-24; off by default, so
    # every other run keeps the step as an endpoint). The fine-tune's label
    # is the expert's DISPLACEMENT over the next horizon (tools/
    # build_follow_dataset.py --horizon-s, 1.0 s): a 2 m label is the expert
    # flying 2 m/s, not a 2 m endpoint. Flown as an endpoint at cruise (4.5
    # m/s, then min(cruise, sqrt(2 a d)) into it), the first fine-tuned
    # follow (vla_follow_ft_01_20261024) sprinted and braked on every step.
    # With rate_horizon_s = H a step is a VELOCITY until the next answer
    # replaces it:
    #   speed fwd_m / H along the BODY heading, at most speed_cap_mps
    #     (cruise_mps does not apply), scaled by cos(heading error) under
    #     turn_while_moving, zero on a rotate-only step without it;
    #   yaw rate dpsi_rad / H, at most yaw_rate_cap_rad_s, until the heading
    #     reaches psi_capture + dpsi (the anchor rules as for an endpoint;
    #     at or past it, zero - no turning back; never past it within one
    #     smooth_tick_s; with smooth_yaw_accel_rps2 the slew's stopping
    #     profile, as for an endpoint);
    #   the altitude as before (hold_altitude_m, the band).
    # A step no answer has replaced within H x (1 + rate_overrun_frac) of its
    # acceptance is complete (completion "rate_horizon") and the existing
    # rules end it: the carry while an answer is pending, else the hold -
    # no new stop rule, no silent hover. The smoothing limits apply to its
    # output as to any. Every such tick says so (Tick.reason "rate", and
    # StepTracker.last_rate: the commanded speed and yaw rate, before the
    # limits, for the rail's row).
    rate_horizon_s: float | None = None
    rate_overrun_frac: float = 0.2

    @property
    def smoothing(self) -> bool:
        return (self.smooth_accel_mps2 is not None or self.smooth_yaw_accel_rps2 is not None
                or self.carry_s > 0.0)

    def __post_init__(self):
        for k in ("cruise_mps", "speed_cap_mps", "climb_cap_mps", "yaw_rate_cap_rad_s",
                  "k_yaw", "k_pos", "k_alt", "pos_tol_m", "alt_tol_m", "yaw_tol_rad",
                  "time_cap_factor", "time_cap_max_s", "decision_timeout_s", "smooth_tick_s"):
            if _finite(getattr(self, k), k) <= 0:
                raise ValueError(f"{k} must be positive")
        for k in ("time_cap_base_s", "alt_band_inset_m", "carry_s", "rate_overrun_frac"):
            if _finite(getattr(self, k), k) < 0:
                raise ValueError(f"{k} must not be negative")
        for k in ("smooth_accel_mps2", "smooth_yaw_accel_rps2"):
            v = getattr(self, k)
            if v is not None and _finite(v, k) <= 0:
                raise ValueError(f"{k} must be positive, or None for no limit")
        if self.rate_horizon_s is not None and _finite(self.rate_horizon_s,
                                                       "rate_horizon_s") <= 0:
            raise ValueError("rate_horizon_s must be positive, or None for the endpoint")
        if self.hold_altitude_m is not None:
            _finite(self.hold_altitude_m, "hold_altitude_m")
        if self.cruise_mps > self.speed_cap_mps:
            raise ValueError(f"cruise {self.cruise_mps} m/s exceeds the speed cap "
                             f"{self.speed_cap_mps} m/s")
        _check_band(self.alt_band)


def _check_band(band) -> None:
    if band is None:
        return
    lo, hi = (_finite(band[0], "alt_band low"), _finite(band[1], "alt_band high"))
    if not lo < hi:
        raise ValueError(f"alt_band {band!r} is empty")


@dataclass(frozen=True)
class Tick:
    """One control tick's output, and who authored it."""
    action: Action4D
    author: str
    reason: str
    seq: int | None = None
    age_s: float | None = None
    land_requested: bool = False

    def as_row(self) -> dict:
        return {"pilot_author": self.author, "pilot_reason": self.reason,
                "vla_seq": self.seq, "vla_age_s": self.age_s,
                "land_requested": self.land_requested}


@dataclass
class _Current:
    decision: Decision
    t_start: float
    psi_t: float | None = None
    x_t: float | None = None
    y_t: float | None = None
    up_t: float | None = None
    up_raw: float | None = None          # the MODEL's altitude target (up_capture - down_m)
    up_held: float | None = None         # TrackerConfig.hold_altitude_m, when it replaces it
    alt_clamped: bool = False
    alt_held: bool = False
    rotate_only: bool = False
    turn_moving: bool = False
    anchor: str = "capture"
    time_cap_s: float | None = None
    complete: bool = False
    completion: str | None = None
    t_complete: float | None = None
    ticks: Counter = field(default_factory=Counter)
    t_first_tick: float | None = None
    t_last_tick: float | None = None
    # TrackerConfig.carry_s: the horizontal velocity commanded when the step
    # completed, carried (decaying) while the next answer is pending.
    carry_v: tuple[float, float] | None = None
    # TrackerConfig.rate_horizon_s: the step's speed and yaw rate, fixed when
    # it is accepted (None: the step is an endpoint)
    rate: dict | None = None


_ZERO = Action4D()


class StepTracker:
    """Turns accepted decisions into a world-frame Action4D on every call of
    `tick(now, pose)`, at whatever rate the rail runs (10 Hz on every rail).

    The newest accepted decision supersedes the one being flown (pipelining).
    A step's targets, fixed when it is accepted:

        heading   psi_t = psi_capture + dpsi
        position  p_t   = p_capture + fwd_m * (cos psi_t, sin psi_t)
                          (no horizontal target when |dpsi| >= 0.25: rotate only)
        altitude  up_t  = up_capture - down_m, clamped into the policy band
                          (inset by min(alt_band_inset_m, band/4))

    Per tick: yaw_rate = clip(k_yaw * wrap(psi_t - psi), +-yaw cap);
    horizontal velocity toward p_t at min(cruise, k_pos * distance), zero once
    p_t is reached or behind the aircraft along psi_t (never backwards);
    vz_up = clip(k_alt * (up_t - up), +-climb cap). The step is complete when
    heading, position and altitude are within tolerance, or at its time cap;
    the tracker then holds (zero horizontal and yaw, altitude kept at up_t)
    until the next decision.

    With TrackerConfig smoothing (the follow; TrackerConfig.smoothing says
    what each limit does) the approach profiles change to what the limits
    can stop from, a step completed while an answer is pending carries its
    velocity for carry_s, and every output passes the acceleration limits
    last (`_smooth`), whoever authored it.

    With TrackerConfig.rate_horizon_s (RATE EXECUTION, the follow
    fine-tune) a step is no endpoint: it flies fwd_m / H along the body
    heading and dpsi / H toward psi_t until the next answer replaces it or
    H x (1 + rate_overrun_frac) has passed (`_rate_tick`); then the carry
    or the hold above.
    """

    def __init__(self, config: TrackerConfig | None = None):
        self.cfg = config or TrackerConfig()
        self._band = self.cfg.alt_band
        # The smoothing's memory: the last action this tracker output and
        # when (TrackerConfig.smoothing). Reset to zero by suspend(): the
        # rail flies its own actions then, and a resume starts from rest.
        self._prev = _ZERO
        self._t_prev: float | None = None
        # when the smoothed horizontal command last reached zero (None while
        # it is not zero): idle_since's start under smoothing
        self._t_still: float | None = -math.inf
        self._cur: _Current | None = None
        self._fallback: str | None = None    # hover reason when no decision is current
        self._none_reason = "no_decision"    # why nothing has flown yet
        self._ever = False
        self._suspended = False
        self._t_resume = -math.inf
        self._idle_t = -math.inf
        self.land_requested = False
        self._finished: list[dict] = []
        self.counts: Counter = Counter()
        self.ticks: Counter = Counter()
        # RATE EXECUTION: the last tick's commanded rates (last_rate)
        self._rate: dict | None = None

    # -- inputs ------------------------------------------------------------ #
    @property
    def alt_band(self) -> tuple[float, float] | None:
        return self._band

    @property
    def suspended(self) -> bool:
        return self._suspended

    @property
    def smoothing(self) -> bool:
        """TrackerConfig.smoothing: a rail that holds the horizontal motion
        itself (the follow's close box) hands it to tick(hold_horizontal=)
        instead, so the hold ramps like everything else."""
        return self.cfg.smoothing

    def set_alt_band(self, band: tuple[float, float] | None) -> None:
        """A new band (a hot-applied policy). The step being flown is
        re-clamped from its unclamped target."""
        _check_band(band)
        self._band = (float(band[0]), float(band[1])) if band is not None else None
        c = self._cur
        if c is not None and c.up_raw is not None:
            c.up_t, c.alt_clamped = self._clamp_alt(
                c.up_held if c.up_held is not None else c.up_raw)

    def accept(self, decision: Decision, now: float, pose_now: Pose | None = None) -> bool:
        """Take a decision. Refused (and recorded as refused) when it is a
        server error, while suspended, when captured before the last rail
        hold ended, after LAND, when already older than the decision timeout,
        or when its capture time is in the future of this clock."""
        if decision.kind == "server_error":
            # Not an answer from the model: the server refused the request
            # (ok=false) and the model never saw it. Until the review of
            # 7c7cc50 (P1) it was accepted as a `hold`, and each one reset the
            # hold timer: a server refusing every request gave 299 hold ticks
            # and 0 hover in 30 s, a dead model holding "on purpose". It is
            # handled as no answer at all, like a link failure.
            self.counts["server_error"] += 1
            self._no_answer(now, "server_error")
            decision.record.update({"accepted": False, "refused": "server_error",
                                    "target": None, "outcome": None})
            self._finished.append(decision.record)
            return False
        reason = None
        age = now - decision.t_capture
        if self._suspended:
            reason = "suspended"
        elif decision.t_capture < self._t_resume:
            reason = "invalidated"
        elif self.land_requested:
            reason = "after_land"
        elif age > self.cfg.decision_timeout_s:
            reason = "stale_on_arrival"
        elif age < -1e-6:
            reason = "future_capture"
        if reason is not None:
            decision.record.update({"accepted": False, "refused": reason,
                                    "target": None, "outcome": None})
            self.counts[f"refused_{reason}"] += 1
            self._finished.append(decision.record)
            return False
        if self._cur is not None:
            self._finish(self._cur, now, "superseded")
        self._ever, self._fallback = True, None
        self.counts["accepted"] += 1
        self.counts[f"kind_{decision.kind}"] += 1
        decision.record.update({"accepted": True, "refused": None})
        if decision.kind == "land" and self.cfg.land_as_arrival:
            # A follow's arrival: hold here (the next answer supersedes it).
            self.counts["land_as_arrival"] += 1
            decision.record.update({"target": None, "land_as_arrival": True})
            self._cur = _Current(decision, now)
            return True
        if decision.kind == "land":
            # Stop and say so. The rail decides what landing means.
            self.land_requested = True
            decision.record.update({"target": None, "outcome": {
                "t_start": now, "completed": False, "completion": None,
                "t_complete": None, "end": "land", "t_end": now, "ticks": {}}})
            self._finished.append(decision.record)
            self._idle_t = now
            return True
        cur = _Current(decision, now)
        if decision.kind == "step":
            self._targets(cur, pose_now)
            decision.record["target"] = {
                "psi_rad": cur.psi_t, "x": cur.x_t, "y": cur.y_t, "up": cur.up_t,
                "up_raw": cur.up_raw, "alt_clamped": cur.alt_clamped,
                "rotate_only": cur.rotate_only, "time_cap_s": cur.time_cap_s,
                "turn_while_moving": cur.turn_moving, "alt_held": cur.alt_held,
                "anchor": cur.anchor}
            if cur.alt_held:
                # The model's target stays up_raw; the one flown (before the
                # band's clamp) is this (review of d6e7123, P3).
                decision.record["target"]["up_held"] = cur.up_held
            if cur.rate is not None:
                # flown as a velocity (RATE EXECUTION): x/y above are the
                # step's geometry, not a target
                decision.record["target"]["rate"] = dict(cur.rate)
                self.counts["rate_steps"] += 1
            if cur.alt_clamped:
                self.counts["alt_clamped"] += 1
            if cur.turn_moving:
                self.counts["turn_while_moving"] += 1
            if cur.alt_held:
                self.counts["alt_held"] += 1
        else:
            # parse_fail: the model answered, malformed. Hold zero, counted.
            decision.record["target"] = None
        self._cur = cur
        return True

    def link_lost(self, now: float, detail: str = "") -> None:
        """The link failed (timeout, refused, protocol). With
        hover_on_link_loss the decision being flown is dropped and the
        tracker hovers until a fresh one arrives."""
        self.counts["link_lost"] += 1
        self._no_answer(now, "link_lost")

    def _no_answer(self, now: float, why: str) -> None:
        """A request came back with no answer from the model (a link failure,
        a server error)."""
        if not self._ever:
            self._none_reason = why
        if not self.cfg.hover_on_link_loss:
            return
        if self._cur is not None:
            self._finish(self._cur, now, why)
        if self._ever and not self.land_requested:
            self._fallback = why

    def suspend(self, now: float) -> None:
        """The rail took the aircraft (an escalation hold, Loiter, a Shield
        hold). The decision being flown is dropped; nothing is accepted
        until `resume`."""
        if self._suspended:
            return
        self._suspended = True
        self.counts["suspends"] += 1
        self._prev, self._t_prev, self._t_still = _ZERO, None, -math.inf
        if self._cur is not None:
            self._finish(self._cur, now, "suspended")

    def resume(self, now: float) -> None:
        """The rail handed the aircraft back. Every decision captured before
        `now` is invalidated, including the one in flight: it describes a
        scene from before the hold."""
        if not self._suspended:
            return
        self._suspended = False
        self._t_resume = now
        if self._ever and not self.land_requested:
            self._fallback = "awaiting_fresh_decision"

    def clear_land(self) -> None:
        """Re-engage after LAND (a new leg the operator commanded)."""
        self.land_requested = False

    def note_flown(self, action: Action4D, now: float) -> None:
        """The rail flew `action` at `now` instead of this tracker's output
        (follow_vlm --record-perturb-vla, demo/vla_dagger.py: the expert flew
        that tick). With smoothing, the next tick's acceleration limits
        start from what the aircraft was really told, so a hand-over to the
        tracker ramps from the command in force instead of jumping from the
        tracker's own last output (or from rest, after suspend()). Nothing
        else changes: the decision being flown, its targets and timers stay.
        No effect without smoothing; counted (rail_flown)."""
        if not self.cfg.smoothing:
            return
        a = Action4D(vx=float(action.vx), vy=float(action.vy), vz_up=float(action.vz_up),
                     yaw_rate=float(action.yaw_rate))
        self._prev, self._t_prev = a, float(now)
        if math.hypot(a.vx, a.vy) <= 1e-3:
            if self._t_still is None:
                self._t_still = float(now)
        else:
            self._t_still = None
        self.counts["rail_flown"] += 1

    # -- outputs ----------------------------------------------------------- #
    @property
    def idle_since(self) -> float | None:
        """None while a step is being flown, else when the tracker became idle
        (used by the stop-and-go cadence to wait the upstream 0.5 s)."""
        c = self._cur
        if c is None:
            base = self._idle_t
        elif c.decision.kind != "step":
            base = c.t_start
        else:
            base = c.t_complete if c.complete else None
        if base is None or not self.cfg.smoothing:
            return base
        # Smoothing: idle only once the commanded horizontal velocity has
        # ramped down to zero (a carry or a ramp is still motion), and from
        # then: stop-and-go's settle then starts when the aircraft is told to
        # stand still, as upstream's step-then-hover does.
        return None if self._t_still is None else max(base, self._t_still)

    @property
    def last_rate(self) -> dict | None:
        """TrackerConfig RATE EXECUTION: the last tick's commanded
        {horizon_s, speed_mps, yaw_rate_rps, heading_reached} when it flew a
        step as a velocity (reason "rate"), before the smoothing; None after
        any other tick. For the rail's per-tick row (demo/vla_mission.
        MissionLoop, demo/vla_dagger.py): a Tick carries its action, author
        and LAND flag, nothing else."""
        return None if self._rate is None else dict(self._rate)

    @property
    def current_seq(self) -> int | None:
        return self._cur.decision.seq if self._cur is not None else None

    @property
    def current_record(self) -> dict | None:
        """A COPY of the vla_steps record of the decision being flown (its
        phrase, hint source, step, bins and raw text), or None. For the
        rail's per-tick rows and HUD (demo/vla_mission.MissionLoop): a tick
        knows its author and seq, not the phrase that steered it. A copy, so
        nothing a rail does to it reaches the log."""
        c = self._cur
        return dict(c.decision.record) if c is not None else None

    def pop_finished(self) -> list[dict]:
        out, self._finished = self._finished, []
        return out

    def close(self, now: float) -> None:
        if self._cur is not None:
            self._finish(self._cur, now, "closed")

    def tick(self, now: float, pose: Pose, *, pending: bool | None = None,
             hold_horizontal: bool = False) -> Tick:
        """This tick's action. `pending`: is an answer on its way (a request
        in flight)? Only TrackerConfig.carry_s reads it, and only True
        carries. `hold_horizontal`: the rail holds the horizontal motion this
        tick (the follow's close box): the horizontal target is zero, yaw
        and climb fly, counted (held_horizontal). With smoothing the hold
        ramps down at the acceleration limit."""
        self._rate = None
        out = self._tick(now, pose, pending=pending)
        if hold_horizontal and (out.action.vx or out.action.vy):
            a = out.action
            out = replace(out, action=Action4D(vx=0.0, vy=0.0, vz_up=a.vz_up,
                                               yaw_rate=a.yaw_rate))
            self.counts["held_horizontal"] += 1
        if self.cfg.smoothing and not self._suspended:
            out = self._smooth(now, out)
        self.ticks[out.author] += 1
        c = self._cur
        if c is not None:
            c.ticks[out.author] += 1
            if c.t_first_tick is None:
                c.t_first_tick = now
            c.t_last_tick = now
        return out

    def _tick(self, now: float, pose: Pose, pending: bool | None = None) -> Tick:
        if self._suspended:
            return Tick(_ZERO, "none", "suspended", land_requested=self.land_requested)
        if self.land_requested:
            return Tick(_ZERO, "hold", "land", land_requested=True)
        c = self._cur
        if c is not None:
            # decision_timeout_s bounds two different waits. A step still being
            # flown: the age of its image (capture -> now), since past that it
            # describes a scene that no longer exists. A hold (a completed
            # step, or a zero hold): how long the hold has lasted, since a
            # hold is the wait for the next answer, and in the stop-and-go
            # cadence that answer is captured only after the step completes
            # (with 3 s of latency the capture-age rule turned 8 of 9 holds
            # into hovers while the next answer was on its way).
            holding = c.decision.kind != "step" or c.complete
            since = (c.t_complete if c.complete else c.t_start) if holding \
                else c.decision.t_capture
            if now - since > self.cfg.decision_timeout_s:
                self._finish(c, now, "hold_timeout" if holding else "stale")
                self._fallback = "no_new_decision" if holding else "stale"
                c = None
        if c is None:
            if self._fallback is not None:
                return Tick(_ZERO, "hover", self._fallback)
            return Tick(_ZERO, "none", self._none_reason)
        seq, age = c.decision.seq, now - c.decision.t_capture
        if c.decision.kind != "step":
            # A LAND held as a follow's arrival says so: "land" read as a LAND
            # the rail would fly (review of d6e7123, P2-6). Still the model's
            # decision: it chose to stop.
            why = ("land_as_arrival" if c.decision.record.get("land_as_arrival")
                   else c.decision.kind)
            return Tick(_ZERO, "hold", why, seq, age)
        cfg = self.cfg
        e_psi = wrap_pi(c.psi_t - pose.yaw_rad)
        e_up = c.up_t - pose.up
        vz = max(-cfg.climb_cap_mps, min(cfg.climb_cap_mps, cfg.k_alt * e_up))
        if not c.complete and c.rate is not None:
            if now - c.t_start < c.time_cap_s:
                return self._rate_tick(c, pose, e_psi, vz, seq, age)
            # No answer replaced it within H x (1 + overrun): complete, and
            # the carry or the hold below ends it (RATE EXECUTION).
            c.complete, c.t_complete = True, now
            c.completion = "rate_horizon"
            self.counts["completed_rate_horizon"] += 1
            if cfg.carry_s > 0.0:
                c.carry_v = (self._prev.vx, self._prev.vy)
        if not c.complete:
            vx = vy = 0.0
            trans_done = True
            if not c.rotate_only:
                dx, dy = c.x_t - pose.x, c.y_t - pose.y
                dist = math.hypot(dx, dy)
                along = dx * math.cos(c.psi_t) + dy * math.sin(c.psi_t)
                if dist > cfg.pos_tol_m and along > 0.0:
                    trans_done = False
                    # The approach: k_pos x d, or with the smoothing's
                    # acceleration limit the fastest speed it can still stop
                    # from at the endpoint (TrackerConfig, SMOOTHING).
                    near = (math.sqrt(2.0 * cfg.smooth_accel_mps2 * dist)
                            if cfg.smooth_accel_mps2 is not None else cfg.k_pos * dist)
                    speed = min(cfg.cruise_mps, near, cfg.speed_cap_mps)
                    if cfg.turn_while_moving:
                        speed *= max(0.0, math.cos(e_psi))
                    vx, vy = speed * dx / dist, speed * dy / dist
            done = (trans_done and abs(e_psi) <= cfg.yaw_tol_rad
                    and abs(e_up) <= cfg.alt_tol_m)
            if done or now - c.t_start >= c.time_cap_s:
                c.complete, c.t_complete = True, now
                c.completion = "tolerance" if done else "time_cap"
                self.counts[f"completed_{c.completion}"] += 1
                if cfg.carry_s > 0.0:
                    # what the aircraft was last told: the velocity a carry
                    # continues from (TrackerConfig.carry_s)
                    c.carry_v = (self._prev.vx, self._prev.vy)
            else:
                yaw_rate = max(-cfg.yaw_rate_cap_rad_s,
                               min(cfg.yaw_rate_cap_rad_s, cfg.k_yaw * e_psi))
                if cfg.smooth_yaw_accel_rps2 is not None:
                    # the heading approached at what the yaw slew can stop from
                    lim = math.sqrt(2.0 * cfg.smooth_yaw_accel_rps2 * abs(e_psi))
                    yaw_rate = max(-lim, min(lim, yaw_rate))
                # turn_moving: a step whose turn upstream would only have
                # rotated, flown while moving (TrackerConfig.turn_while_moving;
                # review of d6e7123, P3: it read "translate").
                return Tick(Action4D(vx=vx, vy=vy, vz_up=vz, yaw_rate=yaw_rate),
                            "vla_step", ("rotate" if c.rotate_only else
                                         "turn_moving" if c.turn_moving else "translate"),
                            seq, age)
        if (cfg.carry_s > 0.0 and pending is True and c.carry_v is not None
                and c.t_complete is not None and now - c.t_complete < cfg.carry_s):
            # No stop between steps while the next answer is on its way: the
            # last velocity, decaying linearly to zero over carry_s (author
            # vla_step - the motion of the VLA's last step, carried by this
            # executor - reason "carry", counted).
            f = 1.0 - (now - c.t_complete) / cfg.carry_s
            self.counts["carry_ticks"] += 1
            return Tick(Action4D(vx=c.carry_v[0] * f, vy=c.carry_v[1] * f, vz_up=vz),
                        "vla_step", "carry", seq, age)
        return Tick(Action4D(vz_up=vz), "hold", "step_complete", seq, age)

    def _rate_tick(self, c: _Current, pose: Pose, e_psi: float, vz: float, seq: int,
                   age: float) -> Tick:
        """One tick of a step flown as a velocity (TrackerConfig, RATE
        EXECUTION): its speed along the body heading (cos-scaled under
        turn_while_moving), its yaw rate until the heading reaches psi_t."""
        cfg, r = self.cfg, c.rate
        speed = r["speed_mps"]
        if cfg.turn_while_moving:
            speed *= max(0.0, math.cos(e_psi))
        want = r["yaw_rate_rps"]
        # at the target heading, or past it the way the step turns: no turn
        # (a pipelined answer whose turn was flown since its capture)
        reached = abs(e_psi) <= cfg.yaw_tol_rad or want * e_psi <= 0.0
        yaw_rate = 0.0
        if not reached:
            lim = abs(e_psi) / cfg.smooth_tick_s           # never past psi_t in one tick
            if cfg.smooth_yaw_accel_rps2 is not None:
                lim = min(lim, math.sqrt(2.0 * cfg.smooth_yaw_accel_rps2 * abs(e_psi)))
            yaw_rate = math.copysign(min(abs(want), lim), want)
        self.counts["rate_ticks"] += 1
        self._rate = {"horizon_s": r["horizon_s"], "speed_mps": speed, "yaw_rate_rps": yaw_rate,
                      "heading_reached": reached}
        return Tick(Action4D(vx=speed * math.cos(pose.yaw_rad), vy=speed * math.sin(pose.yaw_rad),
                             vz_up=vz, yaw_rate=yaw_rate), "vla_step", "rate", seq, age)

    def _smooth(self, now: float, out: Tick) -> Tick:
        """The acceleration limits (TrackerConfig, SMOOTHING) on this tick's
        action, against the last action output. dt is the time since the
        last call capped at smooth_tick_s, so a late tick buys no burst. The
        horizontal step is scaled as a vector (direction kept); the climb
        rate is not limited (the follow holds its altitude). A tick whose own
        action was zero horizontally but leaves non-zero (a hold or hover
        ramping down) is counted smooth_decay_ticks."""
        cfg = self.cfg
        dt = (cfg.smooth_tick_s if self._t_prev is None
              else min(max(now - self._t_prev, 0.0), cfg.smooth_tick_s))
        self._t_prev = now
        p, a = self._prev, out.action
        vx, vy, yr = a.vx, a.vy, a.yaw_rate
        if cfg.smooth_accel_mps2 is not None:
            dvx, dvy = vx - p.vx, vy - p.vy
            n = math.hypot(dvx, dvy)
            lim = cfg.smooth_accel_mps2 * dt
            if n > lim:
                k = lim / n
                vx, vy = p.vx + dvx * k, p.vy + dvy * k
                self.counts["smooth_limited_h"] += 1
                if a.vx == 0.0 and a.vy == 0.0:
                    self.counts["smooth_decay_ticks"] += 1
        if cfg.smooth_yaw_accel_rps2 is not None:
            lim = cfg.smooth_yaw_accel_rps2 * dt
            d = yr - p.yaw_rate
            if abs(d) > lim:
                yr = p.yaw_rate + math.copysign(lim, d)
                self.counts["smooth_limited_yaw"] += 1
        new = Action4D(vx=vx, vy=vy, vz_up=a.vz_up, yaw_rate=yr)
        self._prev = new
        if math.hypot(vx, vy) <= 1e-3:
            if self._t_still is None:
                self._t_still = now
        else:
            self._t_still = None
        return out if new == a else replace(out, action=new)

    # -- internals ----------------------------------------------------------- #
    def _clamp_alt(self, up_raw: float) -> tuple[float, bool]:
        if self._band is None:
            return up_raw, False
        lo, hi = self._band
        inset = min(self.cfg.alt_band_inset_m, (hi - lo) / 4.0)
        up = min(max(up_raw, lo + inset), hi - inset)
        return up, abs(up - up_raw) > 1e-12

    def _targets(self, c: _Current, pose_now: Pose | None = None) -> None:
        d, s, cfg = c.decision, c.decision.step, self.cfg
        ax, ay = d.pose.x, d.pose.y
        if cfg.anchor_at_answer and pose_now is not None:
            ax, ay, c.anchor = pose_now.x, pose_now.y, "answer"
        c.psi_t = wrap_pi(d.pose.yaw_rad + s.dpsi_rad)
        c.rotate_only = s.rotate_only and not cfg.turn_while_moving
        c.turn_moving = s.rotate_only and cfg.turn_while_moving
        if c.rotate_only:
            c.x_t, c.y_t, dist_h = ax, ay, 0.0
        else:
            c.x_t = ax + s.fwd_m * math.cos(c.psi_t)
            c.y_t = ay + s.fwd_m * math.sin(c.psi_t)
            dist_h = s.fwd_m
        c.up_raw = d.pose.up - s.down_m
        if cfg.hold_altitude_m is not None:
            c.up_held, c.alt_held = float(cfg.hold_altitude_m), True
        c.up_t, c.alt_clamped = self._clamp_alt(c.up_held if c.alt_held else c.up_raw)
        expected = max(dist_h / cfg.cruise_mps,
                       abs(c.up_t - d.pose.up) / cfg.climb_cap_mps,
                       abs(s.dpsi_rad) / cfg.yaw_rate_cap_rad_s)
        c.time_cap_s = min(cfg.time_cap_max_s,
                           cfg.time_cap_factor * expected + cfg.time_cap_base_s)
        if cfg.rate_horizon_s is not None:
            # RATE EXECUTION: the step's velocity, flown until replaced or
            # until H x (1 + overrun) after it is taken
            h = float(cfg.rate_horizon_s)
            c.rate = {"horizon_s": h,
                      "speed_mps": 0.0 if c.rotate_only else min(s.fwd_m / h, cfg.speed_cap_mps),
                      "yaw_rate_rps": max(-cfg.yaw_rate_cap_rad_s,
                                          min(cfg.yaw_rate_cap_rad_s, s.dpsi_rad / h))}
            c.time_cap_s = min(cfg.time_cap_max_s, h * (1.0 + cfg.rate_overrun_frac))

    def _finish(self, c: _Current, now: float, end: str) -> None:
        if self._cur is c:
            self._cur = None
        self._idle_t = now
        self.counts[f"end_{end}"] += 1
        c.decision.record["outcome"] = {
            "t_start": c.t_start, "completed": c.complete, "completion": c.completion,
            "t_complete": c.t_complete, "end": end, "t_end": now,
            "t_first_tick": c.t_first_tick, "t_last_tick": c.t_last_tick,
            "ticks": dict(c.ticks)}
        self._finished.append(c.decision.record)
