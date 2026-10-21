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
2026-10-21). Nothing here reads them as velocities.

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

    vla_step  motion toward the endpoint of an accepted VLA step
    hold      a deliberate stand-still that a VLA answer asked for: the step
              is complete (altitude still held at the step's target), the
              answer was malformed (parse_ok false: zero, counted) or a server
              error (zero, counted), or the model said LAND (zero, latched)
    hover     the fallback when answers stop: the step being flown is older
              than `decision_timeout_s` (capture to now), a hold has lasted
              longer than that with no new answer, the link was lost, or a
              rail hold invalidated the decision; all-zero
    none      no VLA decision has been accepted yet, or the rail has the
              tracker suspended (an escalation hold is flying)

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
mosaic hashes and the model identity. No request field can carry a target's
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
import socket
import threading
import time
from collections import Counter
from dataclasses import dataclass, field
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
NUM_BINS = 99
_RANGE_EPS = 1e-6
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
    them; this module never grabs a frame itself."""
    t_capture: float
    pose: Pose
    front_jpg: bytes
    down_jpg: bytes | None = None

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
    demo/vla_bridge.py) or `none` (no hint: the empty phrase)."""
    phrase: str
    source: str

    def __post_init__(self):
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


def alt_band_from_policy(policy) -> tuple[float, float] | None:
    """The intersection of the policy's HARD altitude envelopes, or None.

    The tracker clamps each step's altitude target into it, so the pilot does
    not ask for a height the Shield would have to refuse; the Shield still
    enforces the band itself. Time-windowed envelopes are included whatever
    the clock says (a tighter band is the conservative side for a pilot-side
    clamp), and corridor bands are not (they hold only inside their tube).
    An empty intersection returns None: the policy lint refuses such a
    policy (guardrail/models.py), and a clamp into nothing has no answer."""
    from .models import AltitudeEnvelope
    lo, hi, found = -math.inf, math.inf, False
    for c in getattr(policy, "constraints", None) or ():
        if isinstance(c, AltitudeEnvelope) and getattr(c, "constraint_type", "hard") == "hard":
            lo, hi, found = max(lo, float(c.alt_min_m)), min(hi, float(c.alt_max_m)), True
    if not found or not lo < hi:
        return None
    return (lo, hi)


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


def parse_infer_response(raw: Any, sent: dict) -> InferReply:
    """Validate an infer response against the request it answers. Raises
    ProtocolError on anything protocol v2 does not allow: a wrong version or
    sequence number, a missing field, a step outside AerialVLA's range, a
    malformed answer that still carries a step, or an echo of t_capture or
    the pose that is not what was sent (a reply to some other request)."""
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
    failure: str | None = None        # "timeout" | "link" | "protocol" | None
    detail: str = ""


# --------------------------------------------------------------------------- #
# The client
# --------------------------------------------------------------------------- #
class RemoteVLA:
    """Protocol v2 client. One request outstanding at a time, by construction.

    The 10 Hz loop never blocks on it: `submit()` hands a request to a worker
    thread and returns at once (None while one is still outstanding, or while
    backing off after a failure), and `poll()` returns the finished
    `LinkResult` once. Failures close the connection; the next request
    reconnects (after `reconnect_backoff_s`) and says hello again. A reply
    that breaks protocol v2 is a failure too ("protocol"): after it the stream
    cannot be trusted, so it is never read as a decision.

    `connect()`, `hello()` and `shutdown_server()` block; call them before the
    flight loop or from tools, never from a control tick.
    """

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, *,
                 connect_timeout_s: float = 2.0, response_timeout_s: float = 10.0,
                 reconnect_backoff_s: float = 1.0, max_line_bytes: int = 8 << 20,
                 clock: Callable[[], float] = time.monotonic):
        self.host, self.port = host, int(port)
        self.connect_timeout_s = _finite(connect_timeout_s, "connect_timeout_s")
        self.response_timeout_s = _finite(response_timeout_s, "response_timeout_s")
        self.reconnect_backoff_s = _finite(reconnect_backoff_s, "reconnect_backoff_s")
        if self.connect_timeout_s <= 0 or self.response_timeout_s <= 0 \
                or self.reconnect_backoff_s < 0:
            raise ValueError("timeouts must be positive and the backoff non-negative")
        self.max_line_bytes = int(max_line_bytes)
        self.clock = clock
        self.identity: dict | None = None
        self._sock: socket.socket | None = None
        self._buf = b""
        self._io = threading.Lock()
        self._cv = threading.Condition()
        self._pending: tuple[int, dict] | None = None
        self._result: LinkResult | None = None
        self._busy = False
        self._closed = False
        self._worker: threading.Thread | None = None
        self._next_seq = 1
        self._retry_at = -math.inf
        self.counts: Counter = Counter()

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
            self.identity = rep["model"]
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

    def _read(self, deadline: float) -> dict:
        assert self._sock is not None
        while b"\n" not in self._buf:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise LinkTimeout(f"no reply within {self.response_timeout_s:g} s")
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
        with self._cv:
            return self._busy

    @property
    def backing_off(self) -> bool:
        return self.clock() < self._retry_at

    def submit(self, fields: dict) -> int | None:
        """Send one infer request (fields from `build_infer_fields`, without
        `seq`) unless one is already outstanding or the client is backing off
        after a failure. Returns the request's seq, or None."""
        with self._cv:
            if self._closed:
                raise RuntimeError("the client is closed")
            if self._busy:
                self.counts["submit_busy"] += 1
                return None
            if self.clock() < self._retry_at:
                self.counts["submit_backoff"] += 1
                return None
            msg = dict(fields)
            msg["seq"] = self._next_seq
            check_request_fields(msg)
            self._next_seq += 1
            self._pending = (msg["seq"], msg)
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
                seq, msg = self._pending
            t_sent = self.clock()
            reply, failure, detail = None, None, ""
            with self._io:
                try:
                    self._ensure_connected()
                    self._send(msg)
                    reply = parse_infer_response(
                        self._read(time.monotonic() + self.response_timeout_s), msg)
                except LinkTimeout as exc:
                    failure, detail = "timeout", str(exc)
                except ProtocolError as exc:
                    failure, detail = "protocol", str(exc)
                except (LinkError, OSError, ValueError) as exc:
                    failure, detail = "link", str(exc)
                if failure is not None:
                    self._drop()
            t_done = self.clock()
            with self._cv:
                if failure is not None:
                    self.counts[f"fail_{failure}"] += 1
                    self._retry_at = t_done + self.reconnect_backoff_s
                else:
                    self.counts["replies"] += 1
                self._pending = None
                self._result = LinkResult(seq, msg, reply, t_sent, t_done, failure, detail)
                self._busy = False
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
    every platform, flushed per line."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, "a", encoding="utf-8", newline="\n")
        self.n = 0

    def write(self, rec: dict) -> None:
        self._fh.write(json.dumps(rec, ensure_ascii=False, allow_nan=False) + "\n")
        self._fh.flush()
        self.n += 1

    def close(self) -> None:
        if not self._fh.closed:
            self._fh.close()


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

    def __post_init__(self):
        for k in ("cruise_mps", "speed_cap_mps", "climb_cap_mps", "yaw_rate_cap_rad_s",
                  "k_yaw", "k_pos", "k_alt", "pos_tol_m", "alt_tol_m", "yaw_tol_rad",
                  "time_cap_factor", "time_cap_max_s", "decision_timeout_s"):
            if _finite(getattr(self, k), k) <= 0:
                raise ValueError(f"{k} must be positive")
        for k in ("time_cap_base_s", "alt_band_inset_m"):
            if _finite(getattr(self, k), k) < 0:
                raise ValueError(f"{k} must not be negative")
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
    up_raw: float | None = None
    alt_clamped: bool = False
    rotate_only: bool = False
    time_cap_s: float | None = None
    complete: bool = False
    completion: str | None = None
    t_complete: float | None = None
    ticks: Counter = field(default_factory=Counter)
    t_first_tick: float | None = None
    t_last_tick: float | None = None


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
    """

    def __init__(self, config: TrackerConfig | None = None):
        self.cfg = config or TrackerConfig()
        self._band = self.cfg.alt_band
        self._cur: _Current | None = None
        self._fallback: str | None = None    # hover reason when no decision is current
        self._ever = False
        self._suspended = False
        self._t_resume = -math.inf
        self._idle_t = -math.inf
        self.land_requested = False
        self._finished: list[dict] = []
        self.counts: Counter = Counter()
        self.ticks: Counter = Counter()

    # -- inputs ------------------------------------------------------------ #
    @property
    def alt_band(self) -> tuple[float, float] | None:
        return self._band

    @property
    def suspended(self) -> bool:
        return self._suspended

    def set_alt_band(self, band: tuple[float, float] | None) -> None:
        """A new band (a hot-applied policy). The step being flown is
        re-clamped from its unclamped target."""
        _check_band(band)
        self._band = (float(band[0]), float(band[1])) if band is not None else None
        c = self._cur
        if c is not None and c.up_raw is not None:
            c.up_t, c.alt_clamped = self._clamp_alt(c.up_raw)

    def accept(self, decision: Decision, now: float) -> bool:
        """Take a decision. Refused (and recorded as refused) while suspended,
        when captured before the last rail hold ended, after LAND, when
        already older than the decision timeout, or when its capture time is
        in the future of this clock."""
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
            self._targets(cur)
            decision.record["target"] = {
                "psi_rad": cur.psi_t, "x": cur.x_t, "y": cur.y_t, "up": cur.up_t,
                "up_raw": cur.up_raw, "alt_clamped": cur.alt_clamped,
                "rotate_only": cur.rotate_only, "time_cap_s": cur.time_cap_s}
            if cur.alt_clamped:
                self.counts["alt_clamped"] += 1
        else:
            # parse_fail / server_error: hold zero, counted.
            decision.record["target"] = None
        self._cur = cur
        return True

    def link_lost(self, now: float, detail: str = "") -> None:
        """The link failed (timeout, refused, protocol). With
        hover_on_link_loss the decision being flown is dropped and the
        tracker hovers until a fresh one arrives."""
        self.counts["link_lost"] += 1
        if not self.cfg.hover_on_link_loss:
            return
        if self._cur is not None:
            self._finish(self._cur, now, "link_lost")
        if self._ever and not self.land_requested:
            self._fallback = "link_lost"

    def suspend(self, now: float) -> None:
        """The rail took the aircraft (an escalation hold, Loiter, a Shield
        hold). The decision being flown is dropped; nothing is accepted
        until `resume`."""
        if self._suspended:
            return
        self._suspended = True
        self.counts["suspends"] += 1
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

    # -- outputs ----------------------------------------------------------- #
    @property
    def idle_since(self) -> float | None:
        """None while a step is being flown, else when the tracker became idle
        (used by the stop-and-go cadence to wait the upstream 0.5 s)."""
        c = self._cur
        if c is None:
            return self._idle_t
        if c.decision.kind != "step":
            return c.t_start
        return c.t_complete if c.complete else None

    @property
    def current_seq(self) -> int | None:
        return self._cur.decision.seq if self._cur is not None else None

    def pop_finished(self) -> list[dict]:
        out, self._finished = self._finished, []
        return out

    def close(self, now: float) -> None:
        if self._cur is not None:
            self._finish(self._cur, now, "closed")

    def tick(self, now: float, pose: Pose) -> Tick:
        out = self._tick(now, pose)
        self.ticks[out.author] += 1
        c = self._cur
        if c is not None:
            c.ticks[out.author] += 1
            if c.t_first_tick is None:
                c.t_first_tick = now
            c.t_last_tick = now
        return out

    def _tick(self, now: float, pose: Pose) -> Tick:
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
            return Tick(_ZERO, "none", "no_decision")
        seq, age = c.decision.seq, now - c.decision.t_capture
        if c.decision.kind != "step":
            return Tick(_ZERO, "hold", c.decision.kind, seq, age)
        cfg = self.cfg
        e_psi = wrap_pi(c.psi_t - pose.yaw_rad)
        e_up = c.up_t - pose.up
        vz = max(-cfg.climb_cap_mps, min(cfg.climb_cap_mps, cfg.k_alt * e_up))
        if not c.complete:
            vx = vy = 0.0
            trans_done = True
            if not c.rotate_only:
                dx, dy = c.x_t - pose.x, c.y_t - pose.y
                dist = math.hypot(dx, dy)
                along = dx * math.cos(c.psi_t) + dy * math.sin(c.psi_t)
                if dist > cfg.pos_tol_m and along > 0.0:
                    trans_done = False
                    speed = min(cfg.cruise_mps, cfg.k_pos * dist, cfg.speed_cap_mps)
                    vx, vy = speed * dx / dist, speed * dy / dist
            done = (trans_done and abs(e_psi) <= cfg.yaw_tol_rad
                    and abs(e_up) <= cfg.alt_tol_m)
            if done or now - c.t_start >= c.time_cap_s:
                c.complete, c.t_complete = True, now
                c.completion = "tolerance" if done else "time_cap"
                self.counts[f"completed_{c.completion}"] += 1
            else:
                yaw_rate = max(-cfg.yaw_rate_cap_rad_s,
                               min(cfg.yaw_rate_cap_rad_s, cfg.k_yaw * e_psi))
                return Tick(Action4D(vx=vx, vy=vy, vz_up=vz, yaw_rate=yaw_rate),
                            "vla_step", "rotate" if c.rotate_only else "translate",
                            seq, age)
        return Tick(Action4D(vz_up=vz), "hold", "step_complete", seq, age)

    # -- internals ----------------------------------------------------------- #
    def _clamp_alt(self, up_raw: float) -> tuple[float, bool]:
        if self._band is None:
            return up_raw, False
        lo, hi = self._band
        inset = min(self.cfg.alt_band_inset_m, (hi - lo) / 4.0)
        up = min(max(up_raw, lo + inset), hi - inset)
        return up, abs(up - up_raw) > 1e-12

    def _targets(self, c: _Current) -> None:
        d, s, cfg = c.decision, c.decision.step, self.cfg
        c.psi_t = wrap_pi(d.pose.yaw_rad + s.dpsi_rad)
        c.rotate_only = s.rotate_only
        if c.rotate_only:
            c.x_t, c.y_t, dist_h = d.pose.x, d.pose.y, 0.0
        else:
            c.x_t = d.pose.x + s.fwd_m * math.cos(c.psi_t)
            c.y_t = d.pose.y + s.fwd_m * math.sin(c.psi_t)
            dist_h = s.fwd_m
        c.up_raw = d.pose.up - s.down_m
        c.up_t, c.alt_clamped = self._clamp_alt(c.up_raw)
        expected = max(dist_h / cfg.cruise_mps,
                       abs(c.up_t - d.pose.up) / cfg.climb_cap_mps,
                       abs(s.dpsi_rad) / cfg.yaw_rate_cap_rad_s)
        c.time_cap_s = min(cfg.time_cap_max_s,
                           cfg.time_cap_factor * expected + cfg.time_cap_base_s)

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
