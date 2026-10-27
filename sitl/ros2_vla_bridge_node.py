"""
ROS 2 node: the AerialVLA bridge - a real camera VLA in the VLA slot of the
grant's ROS 2 / MAVROS 2 topology (to-do VLA-08).

    front + down camera (sensor_msgs/Image) -->  [THIS NODE] --/vla/action_4d--> safety_shield
    /mavros/local_position/pose, /mavros/state ->   |   ^    Float32MultiArray, 10 Hz:
    /shield/setpoint (the Shield is flying) ---->   |   |    [vx_fwd, vy_right, vz_up, yaw_rate,
                                                    v   |     decision_age_s, decision_seq,
                                                            author, flags]
               AerialVLA server (demo/vla_server_v2.py: GPU, its own process,
               often its own host), protocol v2 over TCP

It replaces sitl/ros2_vla_stub_node.py on the same topic, with the same body
frame and the same /vla/identity announcement (`VLA=bridge bash
sitl/run_ros2_demo.sh ...`, or `vla:=bridge` on the launch file); the stub
stays the default.

WHO FLIES (docs/DESIGN-vla-pilot.md). AerialVLA (OpenVLA-7B + the original
aero_vla LoRA) chooses every step from the front + down camera pair and ONE
compass phrase. This node builds nothing of its own around the model: it is
the city rail's pilot (demo/vla_mission.py, guardrail/vla_link.py,
guardrail/vla_backends.RemoteVLABackend) fed from ROS topics. The phrase is
the GOTO's (hint_source "goal"): the bearing of the mission target from the
capture pose, "" inside the arrive radius (vla_link.goal_phrase), the task
AerialVLA was trained on. The target is the SAME mission the Shield node flies
(--command / --target / --bundle, parsed with the same compiler), so the two
nodes agree on it, and only the phrase ever reaches the model. The step
tracker (vla_link.StepTracker, a deterministic interpolator, not a planner)
turns each step into the 10 Hz world-frame action, rotated into the grant's
body frame with the heading of the latest pose, as the stub does. There is
no APPROACH mission here: the ROS rail has no detector topic.

THE MESSAGE. The grant's four values come first, so a Shield that reads
d[:4] (every node before this one) flies it unchanged. Then the decision
behind them, which a republished held action otherwise hides: with only the
four values, a node that kept re-sending a dead model's last step at 10 Hz
would keep the Shield's 0.5 s receipt watchdog green (design section 5,
"Telling a deliberate hold from a dead model"). ACTION_FIELDS:

    decision_age_s  now - t_capture of the decision the action came from, s,
                    on this node's monotonic clock; -1 when no decision is
                    behind it (hover, none)
    decision_seq    that decision's request number (exact in float32 to
                    2**24); -1 when none
    author          AUTHOR_CODES: 0 none, 1 vla_step, 2 hold, 3 hover
                    (vla_link.Tick.author)
    flags           FLAG_LAND (1): the model answered LAND; FLAG_END (2):
                    this node's mission ended (vla_mission.MissionMonitor)
                    and the rail should land

The field names ride in layout.dim[0].label. sitl/ros2_shield_node.py keeps
its own copy (VLA_ACTION_FIELDS) and holds the aircraft when a vla_step tick
carries a decision older than its --decision-timeout-s plus a 1 s grace
(DecisionWatchdog); tests/test_ros2_vla_bridge.py pins the two copies equal.

ZERO WHEN THERE IS NO DECISION. The tracker hovers (all-zero, author hover)
when answers stop, a step outlives the decision timeout, the link fails or the
server refuses; before the first decision and while the rail holds the
aircraft the tick is author none, all-zero. A tick that raises publishes
nothing, so a broken bridge looks like a dead one to the Shield's receipt
watchdog, never like a pilot holding on purpose; so does a tick whose latest
pose is older than --pose-max-age-s (1 s; review of 339f23a, P2-3: a stalled
pose subscription flew a frozen pose).

WHEN THE VLA FLIES (`MissionGate`). The tracker anchors each step at its
capture pose, so a decision taken on the pad or during the climb must never
be flown. The node asks for steps only once the Shield node flies the stream:
armed, GUIDED, and the Shield's first /shield/setpoint seen (it publishes
none before its mission timer starts, sitl/ros2_shield_node.py
start_mission). A later change out of GUIDED - the Shield's own hold
(LOITER flown as BRAKE), an FSM fail-safe, a pilot's takeover - suspends the
tracker (vla_link.StepTracker.suspend), and GUIDED again resumes it with
every decision captured before the resume thrown away. The FSM's Brake state
streams zero in GUIDED and is not seen here: the step being flown resumes
after it, if it is still younger than the decision timeout.

LAND AND THE END. LAND is a flag, never a mode request: this node imports no
MAVLink library and asks for no flight mode. The model's LAND sets FLAG_LAND;
any end of the mission (vla_mission.END_REASONS: vla_land, arrived_no_land,
arrived_overshoot, timeout, server_dead, server_slow, server_error,
parse_fail_streak, no_first_decision) sets FLAG_END, and the Shield node ends
its mission on either through its own mission-end LAND
(ros2_shield_node._finish: never after a takeover, never over an FSM
fail-safe). The Shield's own stops still apply: within 2 m of the target
(REACH_M) it ends "target reached" before the model may have answered LAND.

IDENTITY AND WARM-UP (the city rail's rules, demo/vla_fly.py). The node
attaches to a running server (--vla-server HOST:PORT, required: the server
runs where the GPU is, started with its whole configuration,
demo/vla_server_v2.py serve --precision ... --attn ... --merge/--no-merge).
Its hello's identity must be AerialVLA's (vla_mission.identity_is_vla) unless
--allow-stand-in, and is pinned for the run (RemoteVLA strict_identity).
Before the Shield arms, VM.WARMUP_REQUESTS warm-up inferences on live camera
pairs; the LAST is judged (vla_mission.warmup_series_verdict; the first pays
the model's cold start). Only then is /vla/identity published, with
`ready` true, or false and why: the Shield node started with
--vla-ready-timeout-s (the launch file adds it for vla:=bridge) waits for it
and refuses to arm unless it is ready and announces this node's
action_layout and decision_timeout_s as its own. A node that is not ready publishes
zero with FLAG_END, so a Shield that did not wait lands at its first tick.
/vla/identity carries kind "vla" with the server's weights digest
(`reported_digest`) only for a real AerialVLA identity, else kind "stub" under
the stand-in's fake: id (vla_mission.pilot_entry's rule), and `host` is the
SERVER's host evidence from its hello - where the model runs, which the hil
manifest checks (guardrail/manifest._vla_host_missing) - with this node's own
host under `bridge_host`.

RECORDS, in the run's directory (--out; the launch file and run_ros2_demo.sh
pass the Shield's demo/out/<tag>, and without --out it is the Shield's own
rule: demo/out/<--tag, else ros2_shield_<on|off>[_dynamic]>), beside the
Shield node's, written when the node stops - Ctrl+C, launch's SIGINT, or
SIGTERM (run_ros2_demo.sh's USE_LAUNCH=0 path sends it and waits; until the
review of 339f23a it SIGKILLed the bridge and the records were lost):
vla_steps.jsonl
(guardrail/vla_link.StepLog, one line per request: the answer, the model
identity, what the tracker made of it - what tools/todo.py reads first),
vla_frames/<sha256>.jpg (every request's image bytes, for the replay gate:
the name keeps .jpg whatever the codec, see FrameEncoder), vla_captures.jsonl
(only the captures sent, with pairing, sizes, pose), vla_bridge_ticks.jsonl
(per tick from the mission start: author, decision, phrase, step, the body
action, flags) and vla_bridge.json (the identity, the warm-up attempts, the
mission monitor's summary, RemoteVLABackend.stats(), the pilot record and
summary of vla_mission.pilot_entry). An earlier run's copies of these files
are moved to _previous/<UTC stamp>/ first (guardrail.replay's rule for the
Shield's own files, which does not touch these).

THE CAMERAS. Two sensor_msgs/Image topics (--front-topic, --down-topic,
default /camera/front/image_raw and /camera/down/image_raw), rgb8 or bgr8,
AerialVLA's training cameras: 256x256, 90 deg, pitch 0 and -90 (the CityLife
VLA configs, demo/pas_config/robot_vla_quad*.jsonc). The front and down frames
are paired by their header stamps (else by arrival) within --max-pair-dt-s.
The capture time is the front frame's header stamp when that stamp is on
this host's wall clock (mapped onto the node's clock, within 2 s of the
arrival), else its ARRIVAL; the capture pose is the pose ring's at that
time, by arrival on this node's clock (camera and MAVROS stamps need not
share a clock: MAVROS stamps through FCU timesync) (_RingObs; review of
339f23a, P3). NOTHING ON THE SITL RAIL PUBLISHES THEM YET: ArduPilot
SITL has no camera, so without a camera publisher (Project AirSim's, a
driver on the drone) the warm-up finds no pair and the node is not ready.

NOT TESTED WITH ROS. tests/test_ros2_vla_bridge.py drives the pure core
against tests/vla_fake_server.py on loopback with a point vehicle, and the
ROS class on a stand-in rclpy; no rclpy, MAVROS, camera or SITL ran.

Run (after `source /opt/ros/jazzy/setup.bash`; normally through
run_ros2_demo.sh with VLA=bridge):

    ~/venv-ros/bin/python sitl/ros2_vla_bridge_node.py --vla-server 172.29.0.1:8790 \\
        --bundle B --out demo/out/ros2_shield_on [--check-args]
"""
from __future__ import annotations

import json
import math
import os
import shutil
import struct
import sys
import threading
import time
import zlib
from collections import Counter
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "sitl", ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import Action4D, State                                   # noqa: E402
from guardrail.frames import to_body                                    # noqa: E402
from guardrail.manifest import collect_host_evidence                    # noqa: E402
from guardrail.vla_link import Pose, RemoteVLA, TrackerConfig, identity_of  # noqa: E402
from mavlink_adapter_node import (TOPIC_SETPOINT, XYArgumentParser,     # noqa: E402
                                  heading_deg_from_enu_quaternion)
import vla_mission as VM                                                # noqa: E402

NODE_NAME = "vla_bridge"
TOPIC_ACTION = "/vla/action_4d"
TOPIC_IDENTITY = "/vla/identity"
TOPIC_POSE = "/mavros/local_position/pose"
TOPIC_STATE = "/mavros/state"
FRONT_TOPIC = "/camera/front/image_raw"
DOWN_TOPIC = "/camera/down/image_raw"
TICK = 0.1                                   # 10 Hz, the grant's action rate

# /vla/action_4d, in order (module docstring, THE MESSAGE).
ACTION_FIELDS = ("vx_fwd", "vy_right", "vz_up", "yaw_rate",
                 "decision_age_s", "decision_seq", "author", "flags")
ACTION_LABEL = ",".join(ACTION_FIELDS)
AUTHOR_CODES = {"none": 0, "vla_step": 1, "hold": 2, "hover": 3}
FLAG_LAND = 1
FLAG_END = 2
NO_DECISION = -1.0

# The Shield node's own defaults, so a bridge started with the same extra
# arguments ends on the same clock (sitl/ros2_shield_node.MAX_S; the test
# pins them equal).
MAX_S = 120.0
DEFAULT_POLICY_YAML = ROOT / "policies" / "sim_demo_policy.yaml"
# This node's records (RECORDS above); an earlier run's go to
# _previous/<stamp>/ as guardrail.replay.rotate_stale_episode moves the
# Shield's (guardrail.replay.PREVIOUS_DIR, pinned by the test).
RUN_FILES = ("vla_steps.jsonl", "vla_captures.jsonl", "vla_bridge.json",
             "vla_bridge_ticks.jsonl")
PREVIOUS_DIR = "_previous"
IMAGE_ENCODINGS = ("rgb8", "bgr8")
# How long the warm-up waits for a camera pair (demo/vla_fly._warmup's 30 s).
WAIT_FRAMES_S = 30.0


# --------------------------------------------------------------------------- #
# The message
# --------------------------------------------------------------------------- #
def action_message(body, tick, *, ended: bool) -> list[float]:
    """The eight floats of /vla/action_4d for one tick: the body-frame action
    (vx_fwd, vy_right, vz_up, yaw_rate), then the decision behind it
    (vla_link.Tick: its age and seq, or -1 for none), its author's code and
    the flags."""
    has = tick.seq is not None and tick.age_s is not None
    flags = (FLAG_LAND if tick.land_requested else 0) | (FLAG_END if ended else 0)
    out = [float(v) for v in body[:4]]
    out += [float(tick.age_s) if has else NO_DECISION,
            float(tick.seq) if has else NO_DECISION,
            float(AUTHOR_CODES[tick.author]), float(flags)]
    return out


# --------------------------------------------------------------------------- #
# Camera frames
# --------------------------------------------------------------------------- #
def _stamp_ns(msg) -> int | None:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    try:
        ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None
    return ns or None                        # an unset stamp is 0: no stamp


def image_frame(msg) -> dict:
    """A sensor_msgs/Image -> the frame dict vla_mission.VLAFrames stores
    (width, height, time_stamp in ns or None), keeping the message's own
    bytes: the callback only checks, as the city rail's store only assigns a
    pointer (decoding in a camera callback starved the inference thread,
    semantic_demo.py). rgb8 and bgr8 only; anything else is refused loudly
    (ValueError) rather than decoded into wrong colours."""
    enc = str(getattr(msg, "encoding", "")).lower()
    if enc not in IMAGE_ENCODINGS:
        raise ValueError(f"image encoding {getattr(msg, 'encoding', None)!r} is not one of "
                         f"{IMAGE_ENCODINGS}")
    w, h = int(msg.width), int(msg.height)
    step = int(getattr(msg, "step", 0) or 0) or 3 * w
    if w <= 0 or h <= 0 or step < 3 * w:
        raise ValueError(f"image {w}x{h} with row step {step} is not an RGB image")
    if len(msg.data) < step * h:
        raise ValueError(f"image {w}x{h} (step {step}) carries {len(msg.data)} bytes, "
                         f"needs {step * h}")
    return {"data": msg.data, "encoding": enc, "width": w, "height": h, "step": step,
            "time_stamp": _stamp_ns(msg)}


def rgb_bytes(frame: dict) -> bytes:
    """The frame's pixels as packed RGB rows (row padding dropped, bgr8
    swapped)."""
    w, h, step = frame["width"], frame["height"], frame["step"]
    data = bytes(frame["data"])
    row = 3 * w
    if step != row:
        data = b"".join(data[r * step:r * step + row] for r in range(h))
    else:
        data = data[:row * h]
    if frame["encoding"] == "bgr8":
        out = bytearray(data)
        out[0::3], out[2::3] = data[2::3], data[0::3]
        data = bytes(out)
    return data


def png_bytes(rgb: bytes, w: int, h: int) -> bytes:
    """A lossless 8-bit RGB PNG written with the standard library (zlib)."""
    row = 3 * w
    raw = b"".join(b"\x00" + rgb[r * row:(r + 1) * row] for r in range(h))

    def chunk(kind: bytes, body: bytes) -> bytes:
        return (struct.pack(">I", len(body)) + kind + body
                + struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


class FrameEncoder:
    """A frame dict -> the image bytes a request carries (protocol v2's
    `front_jpg_b64`: "JPEG on the wire; any format Pillow reads",
    aerialvla_codec.decode_image_b64).

    "jpeg": quality 95 through Pillow, the city rail's quality
    (vla_mission.jpeg_from_msg, the server bench's). "png": lossless, the
    standard library only, for an interpreter without Pillow (the companion
    image installs none; ~/venv-ros has it through matplotlib). "auto"
    (default): jpeg when Pillow imports, else png. The codec is recorded in
    /vla/identity and vla_bridge.json; the frame files keep the .jpg name
    vla_mission.FrameSaver gives them (the replay gate finds them by SHA-256
    and resends the bytes as they are)."""

    CODECS = ("auto", "jpeg", "png")

    def __init__(self, prefer: str = "auto", quality: int = 95):
        if prefer not in self.CODECS:
            raise ValueError(f"image codec {prefer!r} is not one of {self.CODECS}")
        self.quality = int(quality)
        have_pil = True
        try:
            import PIL.Image  # noqa: F401
        except ImportError:
            have_pil = False
        if prefer == "jpeg" and not have_pil:
            raise ValueError("--image-codec jpeg needs Pillow, which does not import here")
        self.codec = "png" if prefer == "png" or not have_pil else "jpeg"
        self.n = 0
        self.failures = 0
        self.last_error: str | None = None

    def __call__(self, frame: dict | None) -> bytes | None:
        if not frame:
            return None
        try:
            rgb = rgb_bytes(frame)
            w, h = frame["width"], frame["height"]
            if self.codec == "png":
                out = png_bytes(rgb, w, h)
            else:
                import io
                from PIL import Image
                buf = io.BytesIO()
                Image.frombytes("RGB", (w, h), rgb).save(buf, format="JPEG",
                                                         quality=self.quality)
                out = buf.getvalue()
        except Exception as exc:                                  # noqa: BLE001
            self.failures += 1
            self.last_error = f"{type(exc).__name__}: {exc}"
            return None
        self.n += 1
        return out

    def describe(self) -> dict:
        return {"codec": self.codec, "quality": self.quality if self.codec == "jpeg" else None,
                "encoded": self.n, "failures": self.failures, "last_error": self.last_error}


# --------------------------------------------------------------------------- #
# Poses
# --------------------------------------------------------------------------- #
class PoseRing:
    """The recent poses by their arrival on this node's clock, so a camera
    frame gets the pose the aircraft had when it arrived (interpolated; the
    heading along the shorter arc). A frame newer than the last pose takes it
    only while it is at most `max_gap_s` old; older than the ring, or with no
    pose near it, there is no pose (no request)."""

    def __init__(self, maxlen: int = 400, max_gap_s: float = 0.5):
        self._buf: list[tuple[float, float, float, float, float]] = []
        self.maxlen = int(maxlen)
        self.max_gap_s = float(max_gap_s)
        self._lock = threading.Lock()
        self.n = 0

    def put(self, t: float, x: float, y: float, up: float, yaw_rad: float) -> None:
        with self._lock:
            self._buf.append((float(t), float(x), float(y), float(up), float(yaw_rad)))
            if len(self._buf) > self.maxlen:
                del self._buf[:len(self._buf) - self.maxlen]
            self.n += 1

    def latest(self) -> tuple | None:
        with self._lock:
            return self._buf[-1] if self._buf else None

    def at(self, t: float) -> dict | None:
        with self._lock:
            buf = list(self._buf)
        if not buf:
            return None
        t = float(t)
        last = buf[-1]
        if t >= last[0]:
            if t - last[0] > self.max_gap_s:
                return None
            return {"x": last[1], "y": last[2], "up": last[3], "yaw": last[4],
                    "src": "latest", "age_s": round(t - last[0], 4)}
        if t < buf[0][0]:
            return None
        for a, b in zip(buf, buf[1:]):
            if a[0] <= t <= b[0]:
                if b[0] - a[0] > 2 * self.max_gap_s:
                    return None
                k = 0.0 if b[0] == a[0] else (t - a[0]) / (b[0] - a[0])
                dyaw = (b[4] - a[4] + math.pi) % (2 * math.pi) - math.pi
                yaw = (a[4] + k * dyaw + math.pi) % (2 * math.pi) - math.pi
                return {"x": a[1] + k * (b[1] - a[1]), "y": a[2] + k * (b[2] - a[2]),
                        "up": a[3] + k * (b[3] - a[3]), "yaw": yaw, "src": "interp",
                        "age_s": 0.0}
        return None


# A camera stamp mapped onto this node's clock further than this from the
# frame's arrival is on another clock (sim time, another host) and is not
# used: vla_mission.VLAFrames' own sanity rule.
STAMP_MAX_SKEW_S = 2.0


class _RingObs:
    """What vla_mission.VLAFrames asks of its `obs`: the capture time of a
    frame on this node's clock, and the pose then.

    THE CAPTURE TIME (review of 339f23a, P3). It was the front frame's
    arrival, so the camera's own latency was flown as if the image were
    newer. A header stamp on this host's wall clock (a camera driver's ROS
    time without sim time) maps onto the node's clock by the current offset
    between the two clocks - `stamp + (clock() - wall())` - and is the
    capture time when it lands within STAMP_MAX_SKEW_S of the arrival and not
    after it; a stamp on any other clock falls back to the arrival, as
    VLAFrames does. The pose is the ring's at that same time."""

    def __init__(self, ring: PoseRing, clock: Callable[[], float],
                 wall: Callable[[], float]):
        self.ring, self.clock, self.wall = ring, clock, wall

    def wall_at_stamp(self, stamp):
        """The stamp (ns, wall clock) on this node's clock, or None."""
        if stamp is None:
            return None
        try:
            return float(stamp) / 1e9 + (self.clock() - self.wall())
        except (TypeError, ValueError):
            return None

    def capture_time(self, t_arrival, stamp) -> float:
        t = self.wall_at_stamp(stamp)
        if t is None or abs(t - float(t_arrival)) > STAMP_MAX_SKEW_S:
            return float(t_arrival)
        return min(t, float(t_arrival))

    def pose_at(self, t_arrival, stamp):
        return self.ring.at(self.capture_time(t_arrival, stamp))


# --------------------------------------------------------------------------- #
# When the VLA may fly
# --------------------------------------------------------------------------- #
class MissionGate:
    """The mission starts when the Shield node flies the stream: armed,
    GUIDED, and its first /shield/setpoint seen. After that, out of GUIDED
    (or disarmed) is a rail hold, GUIDED again its end. `update()` returns
    "start", "suspend", "resume" or None (module docstring, WHEN THE VLA
    FLIES)."""

    def __init__(self):
        self.armed: bool | None = None
        self.mode: str | None = None
        self.setpoint_seen = False
        self.started = False
        self.live = False
        self.t_start: float | None = None
        self.events: list[dict] = []

    def on_state(self, armed: bool, mode: str) -> None:
        self.armed, self.mode = bool(armed), str(mode)

    def on_setpoint(self) -> None:
        self.setpoint_seen = True

    def update(self, now: float) -> str | None:
        flying = bool(self.armed) and self.mode == "GUIDED"
        ev = None
        if not self.started:
            if flying and self.setpoint_seen:
                self.started = self.live = True
                self.t_start = float(now)
                ev = "start"
        elif self.live and not flying:
            self.live = False
            ev = "suspend"
        elif not self.live and flying:
            self.live = True
            ev = "resume"
        if ev is not None:
            self.events.append({"t": float(now), "event": ev, "mode": self.mode,
                                "armed": self.armed})
        return ev


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class BridgeConfig:
    host: str
    port: int
    shield_on: bool = True
    cadence: str = "stop_and_go"
    decision_timeout_s: float = 10.0
    late_reply_wait_s: float = VM.LATE_REPLY_WAIT_S
    warmup_max_latency_s: float = VM.WARMUP_MAX_LATENCY_S
    hello_timeout_s: float = VM.HELLO_TIMEOUT_S
    wait_frames_s: float = WAIT_FRAMES_S
    allow_stand_in: bool = False
    object_text: str = VM.GOTO_OBJECT_TEXT
    front_topic: str = FRONT_TOPIC
    down_topic: str = DOWN_TOPIC
    image_codec: str = "auto"
    max_pair_dt_s: float = 0.25
    pose_max_age_s: float = 1.0
    cruise_mps: float = 1.0
    speed_cap_mps: float = 2.0
    max_s: float = MAX_S
    arrive_radius_m: float = 3.0
    arrive_grace_s: float = 8.0
    overshoot_s: float = 3.0
    dead_timeout_s: float = 10.0
    first_decision_timeout_s: float = 60.0

    @property
    def addr(self) -> str:
        return f"{self.host}:{self.port}"

    def monitor(self) -> "VM.MonitorConfig":
        return VM.MonitorConfig(mission="goto", max_s=self.max_s,
                                arrive_radius_m=self.arrive_radius_m,
                                arrive_grace_s=self.arrive_grace_s,
                                overshoot_s=self.overshoot_s,
                                dead_timeout_s=self.dead_timeout_s,
                                first_decision_timeout_s=self.first_decision_timeout_s)


def rotate_previous(out: Path, stamp: str | None = None) -> list[str]:
    """Move an earlier run's RUN_FILES to out/_previous/<UTC stamp>/, never
    append to them (StepLog opens for append). vla_frames/ is content-
    addressed and stays (this run's vla_steps.jsonl names only its own)."""
    stale = [out / n for n in RUN_FILES if (out / n).is_file()]
    if not stale:
        return []
    stamp = stamp or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    dest = out / PREVIOUS_DIR / f"{stamp}_vla_bridge"
    dest.mkdir(parents=True, exist_ok=True)
    for p in stale:
        shutil.move(str(p), str(dest / p.name))
    return [p.name for p in stale]


# --------------------------------------------------------------------------- #
# The pure core (no rclpy): what the node does, driven by its callbacks
# --------------------------------------------------------------------------- #
class VLABridge:
    """The bridge without ROS. The node's callbacks call on_pose /
    on_mav_state / on_shield_setpoint / put_image; its warm-up thread calls
    connect_and_warm_up() once (blocking: hello, then the warm-up series) and
    publishes the identity it returns; its 10 Hz timer calls tick(now) and
    publishes what comes back (None: nothing to publish - no pose yet, or
    the warm-up still running); close() writes the records.

    `clock` is the one clock of t_capture, the tracker and the monitor
    (time.monotonic on the node; a test's simulated clock); `wall` is the
    clock camera stamps are read on (time.time; _RingObs).

    A STALE POSE PUBLISHES NOTHING (review of 339f23a, P2-3). The tracker
    steers from the latest pose; a pose subscription that stalled flew a
    frozen pose - a 5 m step to its time cap, a spin at the yaw-rate cap.
    Once the latest pose is older than `pose_max_age_s` (1.0 s, the Shield
    node's own POSE_TIMEOUT_S: Mission Planner can set the shared link to
    2-4 Hz until the Shield re-asserts 10 Hz) the tick publishes nothing, so
    the Shield's receipt watchdog holds the aircraft; counted
    (`ticks_pose_stale`) with pose_stale / pose_resumed events."""

    def __init__(self, cfg: BridgeConfig, *, mission, policy, policy_source: dict,
                 out: str | Path, clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep,
                 encoder: FrameEncoder | None = None,
                 wall: Callable[[], float] = time.time):
        self.cfg = cfg
        self.mission, self.policy, self.policy_source = mission, policy, dict(policy_source)
        self.goal = (float(mission.target_x), float(mission.target_y))
        self.out = Path(out)
        self.out.mkdir(parents=True, exist_ok=True)
        self.moved_previous = rotate_previous(self.out)
        self.clock, self.sleep = clock, sleep
        self.encoder = encoder or FrameEncoder(cfg.image_codec)
        self.ring = PoseRing()
        self.saver = VM.FrameSaver(self.out / "vla_frames")
        self.frames = VM.VLAFrames(_RingObs(self.ring, clock, wall), encode=self.encoder,
                                   saver=self.saver, clock=clock,
                                   max_pair_dt_s=cfg.max_pair_dt_s)
        self._pose_stale = False
        self.gate = MissionGate()
        self.hint = VM.goto_hint_source(self.goal, cfg.arrive_radius_m)
        self.server = VM.VLAServer(cfg.host, cfg.port)        # attached, never started here
        self.backend = None
        self.client: RemoteVLA | None = None
        self.hello: dict | None = None
        self.is_vla = False
        self.why_not_vla: str | None = None
        self.warmup: dict | None = None
        self.ready: bool | None = None            # None until the warm-up has a verdict
        self.not_ready_because: str | None = None
        self.identity: dict | None = None
        self.pilot_band_rec: dict | None = None
        self.monitor: VM.MissionMonitor | None = None
        self.end_reason: str | None = None
        self.t_end: float | None = None
        self.rows: list[dict] = []
        self.counts: Counter = Counter()
        self.last_errors: dict[str, str] = {}
        self.events: list[dict] = []
        self._closed = False
        self._lock = threading.Lock()

    # -- inputs (ROS callbacks) ---------------------------------------------- #
    def on_pose(self, x: float, y: float, up: float, yaw_deg: float,
                t: float | None = None) -> None:
        """A pose in the guardrail frame (x North, y East, up, heading
        clockwise from North), received at `t` (default: now)."""
        self.ring.put(self.clock() if t is None else t, x, y, up, math.radians(yaw_deg))

    def on_mav_state(self, armed: bool, mode: str) -> None:
        self.gate.on_state(armed, mode)

    def on_shield_setpoint(self) -> None:
        self.gate.on_setpoint()

    def put_image(self, which: str, msg) -> bool:
        """One camera message; False (counted, the first error kept) if it
        cannot be read."""
        try:
            frame = image_frame(msg)
        except (ValueError, TypeError, AttributeError) as exc:
            self.counts[f"{which}_refused"] += 1
            self.last_errors.setdefault(f"{which}_image", f"{type(exc).__name__}: {exc}")
            return False
        (self.frames.put_front if which == "front" else self.frames.put_down)(frame)
        self.counts[f"{which}_frames"] += 1
        return True

    def state_now(self) -> State | None:
        p = self.ring.latest()
        if p is None:
            return None
        return State(x=p[1], y=p[2], up=p[3], yaw_deg=math.degrees(p[4]) % 360.0)

    # -- hello and the warm-up (blocking; the node's worker thread) ----------- #
    def connect_and_warm_up(self) -> dict:
        """Hello, the identity rule, the warm-up series, the backend. Never
        raises: whatever fails makes the node not ready, and says why. Returns
        the /vla/identity record."""
        try:
            self._connect_and_warm_up()
        except Exception as exc:                                  # noqa: BLE001
            self._not_ready(f"{type(exc).__name__}: {exc}")
        self.identity = self.identity_record()
        return self.identity

    def _not_ready(self, why: str) -> None:
        self.not_ready_because = why
        self.ready = False

    def _connect_and_warm_up(self) -> None:
        from guardrail.vla_backends import make_backend
        cfg = self.cfg
        self.hello = self.server.wait_hello(cfg.hello_timeout_s, sleep=self.sleep)
        self.is_vla, self.why_not_vla = VM.identity_is_vla(self.hello)
        if not self.is_vla and not cfg.allow_stand_in:
            self._not_ready(f"the server's identity is not AerialVLA ({self.why_not_vla}); "
                            f"--allow-stand-in flies a stand-in, recorded as one")
            return
        # This node sends VLAFront: a foreign server whose intrinsics assume
        # another camera is not flown (vla_mission.front_source_check).
        front = VM.front_source_check(self.hello, "vla", is_vla=self.is_vla)
        if not front["ok"]:
            self._not_ready(f"{front['why']} (this node sends VLAFront)")
            return
        self.client = RemoteVLA(cfg.host, cfg.port, response_timeout_s=cfg.decision_timeout_s,
                                strict_identity=self.is_vla,
                                late_reply_wait_s=cfg.late_reply_wait_s, clock=self.clock)
        self.client.connect()                 # pins the identity before any request
        if identity_of(self.client.identity) != identity_of(self.hello):
            self._not_ready("the identity the link pinned is not the hello's")
            return
        self.warmup = self.run_warmup()
        if not self.warmup["pass"]:
            self._not_ready(f"warm-up refused: {self.warmup['why']}")
            return
        bpol, band, self.pilot_band_rec = VM.pilot_alt_config(cfg.shield_on, self.policy)
        tcfg = TrackerConfig(cruise_mps=cfg.cruise_mps, speed_cap_mps=cfg.speed_cap_mps,
                             decision_timeout_s=cfg.decision_timeout_s, alt_band=band)
        backend = make_backend("aerovla", None, bpol, get_capture=self.frames.capture,
                               get_hint=self.hint, client=self.client,
                               object_text=cfg.object_text, tracker_config=tcfg,
                               cadence=cfg.cadence, steps_log=self.out / "vla_steps.jsonl",
                               on_submit=self.frames.commit, clock=self.clock)
        backend.suspend()                      # until the Shield flies the stream
        self.pilot_band_rec = {**self.pilot_band_rec, "applied": dict(backend.alt_band_info)}
        with self._lock:
            if self._closed:
                # Shut down while warming up: nothing will tick or close it.
                backend.close()
                self._not_ready("shut down during the warm-up")
                return
            self.backend = backend
            self.ready = True

    def run_warmup(self) -> dict:
        """VM.WARMUP_REQUESTS blocking inferences, each on a fresh live camera
        pair (vla_mission.warmup_hint: the GOTO phrase from the capture pose),
        the first given the first answer's timeout, the LAST judged
        (vla_mission.warmup_series_verdict). Their frames are kept like any
        request sent (vla_captures.jsonl marks them warmup). The aircraft is
        on the pad: nothing here is flown."""
        cfg = self.cfg
        attempts: list[dict] = []
        res: dict = {}
        for k in range(VM.WARMUP_REQUESTS):
            cap = None
            t_end = time.monotonic() + cfg.wait_frames_s
            while True:
                cap = self.frames.capture()
                if cap is not None or time.monotonic() >= t_end:
                    break
                self.sleep(0.1)
            if cap is None:
                return {"pass": False, "answered": False,
                        "max_latency_s": cfg.warmup_max_latency_s,
                        "why": (f"no VLA camera pair (front + down, paired within "
                                f"{cfg.max_pair_dt_s:g} s, with a pose) within "
                                f"{cfg.wait_frames_s:g} s on {cfg.front_topic} / "
                                f"{cfg.down_topic}"),
                        "frames": self.frames.stats(), "attempts": attempts}
            hint = VM.warmup_hint("goto", self.hint, cap)
            timeout_s = (max(cfg.decision_timeout_s, cfg.first_decision_timeout_s) if k == 0
                         else cfg.decision_timeout_s)
            res = VM.warmup_infer(self.client, cap, hint, cfg.object_text, timeout_s)
            if res.get("seq") is not None:
                self.frames.commit(cap, hint, res["seq"], warmup=True)
            attempts.append(dict(res))
        ok, why = VM.warmup_series_verdict(attempts, cfg.warmup_max_latency_s)
        res = dict(res)
        res.update({"pass": ok, "why": why, "max_latency_s": cfg.warmup_max_latency_s,
                    "judged": "last of attempts", "attempts": attempts,
                    "late_replies": list(getattr(self.client, "late", None) or [])})
        return res

    # -- the identity --------------------------------------------------------- #
    def model_id(self) -> str:
        if self.is_vla:
            # MODEL_ID for upstream's adapter, its own id for a follow
            # fine-tune (vla_mission.model_id_for, 2026-10-23)
            return VM.model_id_for(self.hello)
        sid = str((self.hello or {}).get("id") or "unknown")
        return sid if sid.startswith("fake:") else f"fake:{sid}"

    def identity_record(self) -> dict:
        """What /vla/identity announces (module docstring, IDENTITY AND
        WARM-UP). The Shield node records it whole, files metrics.pilot from
        model_id / kind / reported_digest, and waits for `ready`."""
        h = self.hello if isinstance(self.hello, dict) else {}
        w = self.warmup or {}
        last = (w.get("attempts") or [{}])[-1] if w.get("attempts") else {}
        cfg = self.cfg
        return {
            "model_id": self.model_id() if self.hello else "unresolved:no-hello",
            "kind": "vla" if self.is_vla else "stub",
            "frame": "body",
            "node": "sitl/ros2_vla_bridge_node.py",
            "ready": bool(self.ready),
            "not_ready_because": self.not_ready_because,
            "reported_digest": h.get("weights_sha256") if self.is_vla else None,
            "reported_by": cfg.addr,
            "server_identity": {k: h.get(k) for k in ("id", "base", "weights_sha256",
                                                      "precision", "attn", "merged",
                                                      "agreement")},
            "not_a_vla_because": None if self.is_vla else self.why_not_vla,
            # Where the MODEL runs (the server's own evidence, from its hello).
            "host": h.get("host") if isinstance(h.get("host"), dict) else None,
            "bridge_host": collect_host_evidence(),
            "policy_id": self.policy.policy_id,
            "policy_hash": self.policy.policy_hash,
            "policy_source": self.policy_source,
            "mission": "goto", "hint_source": "goal", "goal": list(self.goal),
            "goal_source": "the mission target (--command / --target), as the Shield node's",
            "arrive_radius_m": cfg.arrive_radius_m,
            "object": cfg.object_text,
            "cadence": cfg.cadence, "decision_timeout_s": cfg.decision_timeout_s,
            "shield": "on" if cfg.shield_on else "off",
            "pilot_alt_band": self.pilot_band_rec,
            "warmup": ({"pass": w.get("pass"), "why": w.get("why"),
                        "attempts": len(w.get("attempts") or []),
                        "last_rtt_s": last.get("rtt_s"),
                        "last_latency_server_s": last.get("latency_server_s"),
                        "max_latency_s": w.get("max_latency_s")} if w else None),
            "action_layout": list(ACTION_FIELDS),
            "authors": dict(AUTHOR_CODES), "flags": {"land": FLAG_LAND, "end": FLAG_END},
            "camera_topics": {"front": cfg.front_topic, "down": cfg.down_topic},
            "image_codec": self.encoder.codec,
            "yaw_rate": None,
        }

    # -- the 10 Hz tick ------------------------------------------------------- #
    def safe_tick(self, now: float | None = None) -> list[float] | None:
        """tick(), and a tick that raises publishes nothing (counted, the
        last error kept): a broken bridge must look dead to the Shield's
        receipt watchdog, not like a pilot holding."""
        try:
            return self.tick(self.clock() if now is None else now)
        except Exception as exc:                                  # noqa: BLE001
            self.counts["tick_errors"] += 1
            self.last_errors["tick"] = f"{type(exc).__name__}: {exc}"
            return None

    def tick(self, now: float) -> list[float] | None:
        st = self.state_now()
        latest = self.ring.latest()
        stale = latest is not None and now - latest[0] > self.cfg.pose_max_age_s
        if stale != self._pose_stale:
            self._pose_stale = stale
            self.events.append({"t": now, "event": "pose_stale" if stale else "pose_resumed",
                                "pose_age_s": round(now - latest[0], 3)})
        if stale:
            self.counts["ticks_pose_stale"] += 1
            return None
        if st is None or self.ready is None:
            self.counts["ticks_silent"] += 1
            return None
        with self._lock:
            backend = self.backend
        if not self.ready or backend is None:
            # Not ready: zero, and the end flag, so a Shield that did not wait
            # for the identity lands at its first tick.
            self.counts["ticks_not_ready"] += 1
            return [0.0, 0.0, 0.0, 0.0, NO_DECISION, NO_DECISION,
                    float(AUTHOR_CODES["none"]), float(FLAG_END)]
        ev = self.gate.update(now) if self.end_reason is None else None
        if ev == "start":
            backend.resume()
            self.monitor = VM.MissionMonitor(self.cfg.monitor(), now)
            self.events.append({"t": now, "event": "mission_start"})
        elif ev == "suspend":
            backend.suspend()
            self.events.append({"t": now, "event": "rail_hold", "mode": self.gate.mode})
        elif ev == "resume":
            backend.resume()
            self.events.append({"t": now, "event": "rail_released", "mode": self.gate.mode})
        action = backend.act(st)
        tk = backend.last_tick
        if self.monitor is not None and self.end_reason is None:
            end = self.monitor.update(now, Pose.from_state(st), tk, goal_xy=self.goal,
                                      server_alive=True,
                                      link_failure=backend.last_link_failure)
            if end is not None:
                self.end_reason, self.t_end = end, now
                backend.suspend()             # no more requests; LAND stays latched
                self.events.append({"t": now, "event": "mission_end", "reason": end})
        ended = self.end_reason is not None
        body = (0.0, 0.0, 0.0, 0.0) if ended else to_body(action, st.yaw_deg)
        msg = action_message(body, tk, ended=ended)
        self.counts["ticks_published"] += 1
        if self.gate.started:
            self._row(now, st, tk, body, msg)
        return msg

    def _row(self, now: float, st: State, tk, body, msg: list[float]) -> None:
        cur = getattr(self.backend.tracker, "current_record", None)
        cur = cur if isinstance(cur, dict) else {}
        pose = Pose.from_state(st)
        lf = self.backend.last_link_failure if tk.reason == "link_lost" else None
        self.rows.append({
            "t": round(now - self.gate.t_start, 3), "tick": len(self.rows),
            "x": st.x, "y": st.y, "up": st.up, "psi": round(st.yaw_deg, 3),
            "author": tk.author, "author_reason": tk.reason,
            "vla_seq": tk.seq, "vla_age_s": None if tk.age_s is None else round(tk.age_s, 3),
            "land_requested": bool(tk.land_requested),
            "link_failure": (lf or {}).get("kind") if lf else None,
            "phrase": cur.get("phrase"), "hint_source": cur.get("hint_source"),
            "step": cur.get("step") if isinstance(cur.get("step"), dict) else None,
            "goal_dist_m": round(VM.distance_to(pose, self.goal), 3),
            "goal_bearing_deg": round(math.degrees(VM.bearing_to(pose, self.goal)), 2),
            "rail_live": self.gate.live,
            "body": [round(float(v), 4) for v in body],
            "flags": int(msg[7]), "end_reason": self.end_reason})

    # -- records -------------------------------------------------------------- #
    def close(self) -> dict:
        """Finish the open decision, stop the client, close the logs, write
        vla_captures.jsonl, vla_bridge_ticks.jsonl and vla_bridge.json.
        Idempotent; returns the summary written."""
        with self._lock:
            if self._closed:
                return getattr(self, "summary", {})
            self._closed = True
        stats = records = None
        errors: dict[str, str] = {}

        def guard(name, fn):
            try:
                return fn()
            except Exception as exc:                              # noqa: BLE001
                errors[name] = f"{type(exc).__name__}: {exc}"
                return None
        if self.backend is not None:
            guard("backend_close", self.backend.close)
            stats = guard("stats", self.backend.stats)
            records = list(self.backend.records)
        elif self.client is not None:
            guard("client_close", self.client.close)
        saver = guard("saver_close", self.saver.close)
        guard("captures", lambda: VM.write_jsonl(self.out / "vla_captures.jsonl",
                                                 list(self.frames.log)))
        guard("ticks", lambda: VM.write_jsonl(self.out / "vla_bridge_ticks.jsonl", self.rows))
        pilot = None
        if self.hello is not None and stats is not None:
            pilot = guard("pilot", lambda: VM.pilot_entry(
                self.hello, stats, records or [], self.rows, server=self.server.record(),
                reported_by=self.cfg.addr,
                weight_paths=VM.local_weight_paths(self.hello) if self.is_vla else None,
                cadence=self.cfg.cadence))
        cfg = asdict(self.cfg)
        self.summary = {
            "node": "sitl/ros2_vla_bridge_node.py",
            "identity": self.identity or self.identity_record(),
            "ready": self.ready, "not_ready_because": self.not_ready_because,
            "warmup": self.warmup,
            "mission": {"kind": "goto", "goal_xy": list(self.goal),
                        "monitor": self.monitor.summary() if self.monitor else None,
                        "end_reason": self.end_reason,
                        "end_reason_means": VM.END_REASONS.get(self.end_reason),
                        "gate_events": list(self.gate.events),
                        "started": self.gate.started},
            "pilot": pilot, "stats": stats,
            "frames": self.frames.stats(), "frame_files": saver,
            "encoder": self.encoder.describe(),
            "counts": dict(self.counts), "last_errors": dict(self.last_errors),
            "events": list(self.events), "moved_previous": self.moved_previous,
            "config": cfg, "records_errors": errors or None,
        }
        try:
            VM.write_json(self.out / "vla_bridge.json", self.summary)
        except (OSError, TypeError, ValueError) as exc:
            errors["summary"] = f"{type(exc).__name__}: {exc}"
            with open(self.out / "vla_bridge.json", "w", encoding="utf-8", newline="\n") as fh:
                fh.write(json.dumps(self.summary, indent=1, ensure_ascii=False, default=repr))
        return self.summary


# --------------------------------------------------------------------------- #
# Command line
# --------------------------------------------------------------------------- #
def make_parser():
    # allow_abbrev off: the rail hands every extra argument to the Shield node
    # and this node alike (run_ros2_demo.sh), and an abbreviation of one
    # node's flag must not be read as another's.
    ap = XYArgumentParser(allow_abbrev=False,
                          description="AerialVLA in the ROS 2 rail's VLA slot (to-do VLA-08)")
    ap.add_argument("--vla-server", default=None, metavar="HOST:PORT",
                    help="the running AerialVLA server (demo/vla_server_v2.py serve); "
                         "required: this node never starts one")
    ap.add_argument("--vla-hello-timeout-s", type=float, default=VM.HELLO_TIMEOUT_S)
    ap.add_argument("--allow-stand-in", action="store_true",
                    help="fly a server whose identity is not AerialVLA (a fake: stand-in), "
                         "recorded as kind stub, never as a VLA")
    ap.add_argument("--policy", default=None)
    ap.add_argument("--bundle", default=None)
    ap.add_argument("--allow-unverified-bundle", action="store_true")
    ap.add_argument("--command", default="fly to the northeast pad at 6 m/s",
                    help="the mission text, as the Shield node's (its target is the goal)")
    ap.add_argument("--target", default=None, metavar="X,Y")
    ap.add_argument("--speed", type=float, default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--tag", default=None)
    ap.add_argument("--out", default=None,
                    help="the run's directory (default: the Shield node's, demo/out/<--tag, "
                         "else ros2_shield_<on|off>[_dynamic]>)")
    ap.add_argument("--shield", choices=("on", "off"), default="on",
                    help="the Shield node's arm: off clamps the tracker only into the "
                         "simulator safety band (vla_mission.pilot_alt_config)")
    ap.add_argument("--dynamic", action="store_true",
                    help="the Shield node's flag, read here only to name its directory")
    ap.add_argument("--max-s", type=float, default=MAX_S, help="mission time cap, seconds")
    ap.add_argument("--object", default=None,
                    help=f"the prompt's object words (default: {VM.GOTO_OBJECT_TEXT!r}; inert)")
    ap.add_argument("--cadence", choices=("pipelined", "stop_and_go"), default="stop_and_go")
    ap.add_argument("--decision-timeout-s", type=float, default=10.0,
                    help="the step tracker's decision timeout and the link's response "
                         "timeout; the Shield node's watchdog reads the same flag")
    ap.add_argument("--late-reply-wait-s", type=float, default=VM.LATE_REPLY_WAIT_S)
    ap.add_argument("--warmup-max-latency-s", type=float, default=VM.WARMUP_MAX_LATENCY_S)
    ap.add_argument("--wait-frames-s", type=float, default=WAIT_FRAMES_S,
                    help="how long each warm-up request waits for a camera pair")
    ap.add_argument("--first-decision-timeout-s", type=float, default=60.0)
    ap.add_argument("--dead-timeout-s", type=float, default=10.0)
    ap.add_argument("--arrive-radius-m", type=float, default=3.0)
    ap.add_argument("--arrive-grace-s", type=float, default=8.0)
    ap.add_argument("--overshoot-s", type=float, default=3.0)
    ap.add_argument("--cruise-mps", type=float, default=1.0)
    ap.add_argument("--speed-cap-mps", type=float, default=2.0)
    ap.add_argument("--front-topic", default=FRONT_TOPIC)
    ap.add_argument("--down-topic", default=DOWN_TOPIC)
    ap.add_argument("--image-codec", choices=FrameEncoder.CODECS, default="auto")
    ap.add_argument("--max-pair-dt-s", type=float, default=0.25)
    ap.add_argument("--pose-max-age-s", type=float, default=1.0,
                    help="publish nothing while the latest pose is older than this "
                         "(the Shield's receipt watchdog then holds)")
    ap.add_argument("--check-args", action="store_true",
                    help="print the resolved configuration and exit (nothing starts)")
    return ap


def resolve(args) -> tuple[BridgeConfig, dict]:
    """(the configuration, the plan: mission, policy, its source, the goal,
    the out directory). ValueError says what cannot fly; nothing has
    started."""
    from guardrail.aerialvla_codec import CodecError, validate_object
    from guardrail.bundle import load_for_flight
    from guardrail.compiler import ConstraintCompiler
    if not args.vla_server:
        raise ValueError("--vla-server HOST:PORT is required: the bridge attaches to a running "
                         "AerialVLA server (demo/vla_server_v2.py serve, with its whole "
                         "configuration) and never starts one")
    host, port = VM.parse_addr(args.vla_server)
    obj = VM.GOTO_OBJECT_TEXT if args.object is None else args.object
    try:
        validate_object(obj)
    except CodecError as exc:
        raise ValueError(f"--object {obj!r}: {exc}") from None
    for k in ("decision_timeout_s", "warmup_max_latency_s", "wait_frames_s",
              "vla_hello_timeout_s", "cruise_mps", "speed_cap_mps", "max_pair_dt_s",
              "pose_max_age_s"):
        v = getattr(args, k)
        if not (math.isfinite(v) and v > 0):
            raise ValueError(f"--{k.replace('_', '-')} must be a positive number, got {v!r}")
    if not (math.isfinite(args.late_reply_wait_s) and args.late_reply_wait_s >= 0):
        raise ValueError("--late-reply-wait-s must be 0 or more")
    FrameEncoder(args.image_codec)            # refuses jpeg without Pillow
    cfg = BridgeConfig(
        host=host, port=port, shield_on=args.shield == "on", cadence=args.cadence,
        decision_timeout_s=args.decision_timeout_s, late_reply_wait_s=args.late_reply_wait_s,
        warmup_max_latency_s=args.warmup_max_latency_s,
        hello_timeout_s=args.vla_hello_timeout_s, wait_frames_s=args.wait_frames_s,
        allow_stand_in=args.allow_stand_in, object_text=obj,
        front_topic=args.front_topic, down_topic=args.down_topic,
        image_codec=args.image_codec, max_pair_dt_s=args.max_pair_dt_s,
        pose_max_age_s=args.pose_max_age_s,
        cruise_mps=args.cruise_mps, speed_cap_mps=args.speed_cap_mps, max_s=args.max_s,
        arrive_radius_m=args.arrive_radius_m, arrive_grace_s=args.arrive_grace_s,
        overshoot_s=args.overshoot_s, dead_timeout_s=args.dead_timeout_s,
        first_decision_timeout_s=args.first_decision_timeout_s)
    cfg.monitor()                             # the stop rules validate themselves
    TrackerConfig(cruise_mps=cfg.cruise_mps, speed_cap_mps=cfg.speed_cap_mps,
                  decision_timeout_s=cfg.decision_timeout_s)
    policy, source = load_for_flight(
        args.bundle, args.policy or (None if args.bundle else DEFAULT_POLICY_YAML),
        allow_unverified=args.allow_unverified_bundle)
    mission = ConstraintCompiler(policy).parse_command(args.command)
    if args.target:
        x, y = (float(v) for v in args.target.split(","))
        mission = mission.model_copy(update={"target_x": x, "target_y": y})
    if args.speed is not None:
        mission = mission.model_copy(update={"speed_pref_mps": args.speed})
    # Beside the Shield node's records (review of 339f23a, P3): its own
    # directory rule (ros2_shield_node.default_tag, pinned by the test).
    tag = args.tag or (f"ros2_shield_{args.shield}" + ("_dynamic" if args.dynamic else ""))
    out = Path(args.out) if args.out else ROOT / "demo" / "out" / tag
    return cfg, {"mission": mission, "policy": policy, "policy_source": source,
                 "goal": (mission.target_x, mission.target_y), "out": out}


def check_args_report(cfg: BridgeConfig, plan: dict) -> dict:
    """What --check-args prints: the resolved run, nothing started."""
    pol = plan["policy"]
    return {"ok": True, "vla_server": cfg.addr, "goal_xy": list(plan["goal"]),
            "cruise_alt_m": plan["mission"].cruise_alt_m, "policy_id": pol.policy_id,
            "policy_hash": pol.policy_hash, "policy_source": plan["policy_source"].get("kind"),
            "out": str(plan["out"]), "shield": "on" if cfg.shield_on else "off",
            "cadence": cfg.cadence, "object": cfg.object_text,
            "camera_topics": [cfg.front_topic, cfg.down_topic],
            "image_codec": FrameEncoder(cfg.image_codec).codec,
            "monitor": cfg.monitor().as_dict(), "action_layout": list(ACTION_FIELDS)}


# --------------------------------------------------------------------------- #
# The node (needs rclpy, sensor_msgs and mavros_msgs)
# --------------------------------------------------------------------------- #
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout, String
    from mavros_msgs.msg import State as MavState
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

if HAVE_ROS:
    class VlaBridgeNode(Node):
        def __init__(self, core: VLABridge) -> None:
            super().__init__(NODE_NAME)
            self.core = core
            cfg = core.cfg
            # mavros publishes sensor topics best-effort; a best-effort
            # subscription also takes a reliable camera publisher.
            self.create_subscription(PoseStamped, TOPIC_POSE, self._on_pose,
                                     qos_profile_sensor_data)
            self.create_subscription(MavState, TOPIC_STATE, self._on_state, 10)
            self.create_subscription(Float32MultiArray, TOPIC_SETPOINT, self._on_setpoint, 10)
            self.create_subscription(Image, cfg.front_topic, self._on_front,
                                     qos_profile_sensor_data)
            self.create_subscription(Image, cfg.down_topic, self._on_down,
                                     qos_profile_sensor_data)
            self.pub = self.create_publisher(Float32MultiArray, TOPIC_ACTION, 10)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub_id = self.create_publisher(String, TOPIC_IDENTITY, latched)
            n = len(ACTION_FIELDS)
            self.layout = MultiArrayLayout(
                dim=[MultiArrayDimension(label=ACTION_LABEL, size=n, stride=n)], data_offset=0)
            self._said_images: set[str] = set()
            self.create_timer(TICK, self._tick)                # 10 Hz - the contract
            self.get_logger().info(
                f"VLA bridge up: server {cfg.addr}, goal {core.goal}, cameras "
                f"{cfg.front_topic} + {cfg.down_topic} ({core.encoder.codec}), cadence "
                f"{cfg.cadence}; saying hello and warming up before /vla/identity")
            self._warm = threading.Thread(target=self._warm_up, name="vla-bridge-warmup",
                                          daemon=True)
            self._warm.start()

        def _warm_up(self) -> None:
            ident = self.core.connect_and_warm_up()
            self.pub_id.publish(String(data=json.dumps(ident, default=str)))
            if ident["ready"]:
                self.get_logger().info(
                    f"/vla/identity: {ident['model_id']} kind {ident['kind']}, ready "
                    f"(warm-up last round trip {(ident.get('warmup') or {}).get('last_rtt_s')} s)")
            else:
                self.get_logger().error(f"/vla/identity: NOT READY - {ident['not_ready_because']}")

        def _on_pose(self, msg: PoseStamped) -> None:
            q = msg.pose.orientation
            # ENU (mavros) -> our up-positive local frame: x = N, y = E.
            self.core.on_pose(msg.pose.position.y, msg.pose.position.x, msg.pose.position.z,
                              heading_deg_from_enu_quaternion(q.x, q.y, q.z, q.w))

        def _on_state(self, msg: MavState) -> None:
            self.core.on_mav_state(bool(msg.armed), str(msg.mode))

        def _on_setpoint(self, msg) -> None:
            self.core.on_shield_setpoint()

        def _image(self, which: str, msg) -> None:
            if not self.core.put_image(which, msg) and which not in self._said_images:
                self._said_images.add(which)
                self.get_logger().error(
                    f"{which} camera frame refused: "
                    f"{self.core.last_errors.get(which + '_image')}")

        def _on_front(self, msg) -> None:
            self._image("front", msg)

        def _on_down(self, msg) -> None:
            self._image("down", msg)

        def _tick(self) -> None:
            data = self.core.safe_tick()
            if data is not None:
                self.pub.publish(Float32MultiArray(layout=self.layout, data=data))


def on_sigterm(signum, frame) -> None:
    """SIGTERM ends the spin like Ctrl+C, so main()'s `finally` writes the
    records (review of 339f23a, P2-4: run_ros2_demo.sh's USE_LAUNCH=0 path
    SIGKILLed the bridge in its cleanup trap, and close() never ran; now it
    TERMs the bridge and waits). A background job of a non-interactive bash
    starts with SIGINT ignored (run_ros2_demo.sh, the bag recorder), so
    SIGINT cannot be the signal; ros2 launch sends SIGINT, then SIGTERM."""
    raise KeyboardInterrupt


def main(argv: list[str] | None = None) -> int:
    args, ros_args = make_parser().parse_known_args(argv)
    try:
        cfg, plan = resolve(args)
    except ValueError as exc:
        raise SystemExit(f"[vla_bridge] REFUSED: {exc}")
    if args.check_args:
        print(json.dumps(check_args_report(cfg, plan), indent=1))
        return 0
    if not HAVE_ROS:
        raise SystemExit("ros2_vla_bridge_node needs rclpy, sensor_msgs and mavros_msgs: "
                         "source /opt/ros/jazzy/setup.bash and use ~/venv-ros/bin/python")
    core = VLABridge(cfg, mission=plan["mission"], policy=plan["policy"],
                     policy_source=plan["policy_source"], out=plan["out"])
    import signal
    signal.signal(signal.SIGTERM, on_sigterm)
    node = None
    try:
        rclpy.init(args=ros_args)
        node = VlaBridgeNode(core)
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception) as e:                  # noqa: BLE001
        if not isinstance(e, KeyboardInterrupt):
            print(f"[vla_bridge] {type(e).__name__}: {e}", file=sys.stderr, flush=True)
    finally:
        # A second TERM - or a second Ctrl+C, which close()'s guard does not
        # catch (re-review of 23c88d3, N5) - must not cut the records short.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        core.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
