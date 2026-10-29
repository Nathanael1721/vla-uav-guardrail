"""
Foreign drone VLAs behind VLA server protocol v2: DroneCATS and RaceVLA, flown
on our red-car follow through the same clients, scenario, scoring and Shield as
our fine-tuned AeroVLA (demo/vla_server_v2.py).

    # DroneCATS-Cosmos3-Edge-2B (NAVER). Needs transformers>=5.18: the dronecats env
    C:/Users/natha/.conda/envs/dronecats/python.exe demo/vla_server_foreign.py serve \\
        --model dronecats --weights D:/models/dronecats/DroneCATS-Cosmos3-Edge-2B \\
        --port 8791 --log demo/out/vla_server_foreign/dronecats.jsonl
    #   ... the detector's 768x432 frames instead (vla_fly --vla-front-source detector)
    #   ... --front-source detector
    # RaceVLA drone6_all (Skoltech; a full OpenVLA-7B fine-tune): the vla-real env
    C:/Users/natha/.conda/envs/vla-real/python.exe demo/vla_server_foreign.py serve \\
        --model racevla --weights "D:/models/racevla/RaceVLA_models/drone6_all/<run>" \\
        --port 8792 --log demo/out/vla_server_foreign/racevla.jsonl
    # a scripted stand-in: native outputs given, no model, no GPU (identity fake:...)
    python demo/vla_server_foreign.py serve --model dronecats --port 8793 \\
        --weights D:/models/dronecats/DroneCATS-Cosmos3-Edge-2B \\
        --fake-native '{"action":"go","point":[500,500],"depth_m":20}'

A DROP-IN FOR THE AEROVLA SERVER
--------------------------------
Same transport, ops and validation as demo/vla_server_v2.py, whose protocol
core this subclasses (V2Server): hello / infer / shutdown, every request
checked by guardrail/aerialvla_codec.validate_request, every ok=false answer
with every key, the JSONL log, the health block, one model lock. An infer
answer has protocol v2's keys plus three:
  native   the model's OWN output (its text, or its seven action tokens),
           what it was read as, the command and history it was asked with,
           the camera_aux and the image size it saw, what of the request it
           ignored;
  mapping  how that became the step: the rule, the requested continuous
           step, which axes were clipped, the bins, the step flown, every
           intermediate (bearing, elevation, velocities, the dropped lateral);
  timing   the model's own phases and the whole server time.
`raw_text` is the step TEXT the mapping produced ("59 49 70</s>", "00 49 49
LAND</s>"), read back through the codec's strict reader (parse_action) so
bins, step and land are exactly what the AeroVLA server would answer for that
text, and every client parses it unchanged. A native output the reader of
its family refuses is parse_ok false, step null, raw_text "": the client
holds zero, as for a malformed AeroVLA answer. All three blocks are in the
JSONL log, and guardrail/vla_link keeps native and mapping in the client's
own vla_steps.jsonl record: no silent conversion.

What the request's other fields mean here: the phrase and hint_source are
validated and ignored (neither model takes a compass hint; recorded under
native.ignored), the down frame is ignored (bottom_tile "none"), a CSP arm
is refused (neither prompt has the slot: an A/B that changes nothing would
report "no effect"), and the object words become the model's command.

DRONECATS (naver-ai/DroneCATS-Cosmos3-Edge-2B, read at HF revision e9004010,
2026-10-27: README model card, prompts/*.txt, chat_template.jinja; the
paper, arXiv 2609.01404 v2, Sec. 2 and Appendix A.2 / C)
-----------------------------------------------------------------------
INPUT. One frame, at most 640 px wide (the card: "if image.width > 640:
resize to (640, int(h*640/w)), Image.BOX"; the paper's simulator renders
1280x720 and gives the model 640x360 at 90 deg - our FrontCamera's 768x432
at 90 deg becomes exactly that, VLAFront stays 256x256). The prompt files
verbatim: system_prompt.txt .format(frame_w, frame_h, quarter_x=w//4,
quarter_y=h//4), user_prompt.txt .format(instruction, history), and the
image with `camera_aux` = json.dumps({"width", "height", "fx", "fy"}),
fx = fy = round(k * w, 2), k = 1 / (2 tan(hfov/2)) - on EVERY request (the
card: without it the template leaves its <|camera_aux|> placeholder in the
prompt and the model gets an input it was never trained on; the real
backend refuses to run such a prompt). apply_chat_template(add_generation_
prompt=True, enable_thinking=False, skip_reasoning=True), greedy, 128 new
tokens (1024 right after a THINK, when the reply carries the planning
fields of plan_response_schema.json).
  The command: "go to <the object>" ("a red car" -> "go to the red car"):
the paper's tracking episodes use the same "go to" referring expression
as approach (Sec. 2.2; "the target moves along a one-way route").
  The history: the last <= 5 actions, oldest first, in the card's lines
      "  step 1 of 2: ROTATE angle=-90deg  [done, 1.5s]"
      "  step 2 of 2: GO x=402 y=656 depth=3.2m  [in progress]  <- latest"
(done = the time to the next request's capture), "(none yet - this is the
first step)" before the first (with the card's em dash). The card shows two
lines; numbering within the window of five is this server's reading. Kept
per connection (one thread per connection), reset on hello (every
RemoteVLA connection starts with one) and on a gap of more than
--history-gap-s between captures, or a capture time that runs backwards.
A malformed answer is not an action and is not added.

OUTPUT -> STEP (rule dronecats_go_rotate/1; the paper's A.2 executor):
  go {point [x, y] on 0..1000, depth_m d}: d is CANONICAL depth along the
    optical axis (a reference camera with fx/W = 1, 53 deg; Appendix C:
    D = d * fx/W), so D = d * k metres. The pixel px = x*W/1000, py =
    y*H/1000 back-projects (A.2: square pixels, principal point at the
    centre) to right X = (px - W/2)/fx * D, down Y = (py - H/2)/fy * D,
    forward Z = D, rotated from the camera's pitch p (+ up; FrontCamera's
    -20 deg) to the level frame: F = Z cos p + Y sin p, D_n = -Z sin p +
    Y cos p, R = X. Then bearing = atan2(R, F) (+ right = clockwise, as
    AeroVLA's dpsi), horizontal h = hypot(F, R), down v = D_n, and the
    paper's executor: altitude held (v = 0) while |v| < 0.15 h; one step at
    most 3 m (1.2 m/s held at most 2.5 s): the (h, v) vector scaled to 3 m;
    then x0.6 while D < 5 m, x0 while D < 2 m (the paper does not say
    whether the 0.6 applies before or after the cap: after, the cautious
    reading). fwd_m = h * scale, down_m = v * scale, dpsi_rad = bearing.
    --go-executor direct keeps the whole ray (fwd = h, down = v, no hold,
    cap or bands; the codec's 5 m clips it). Differences from the paper's
    executor, kept as they are: it yaws in place first beyond an 8 deg
    bearing, AeroVLA's step turns first and rotates only from 14.3 deg
    (0.25 rad, the client's StepTracker); it flies a body-frame velocity,
    the step flies along the new heading.
  rotate {angle_deg}: clamped to +-90 (the paper's parser), dpsi = angle
    (+ right), no translation; the codec's yaw range then clips +-63 deg
    (1.1 rad), recorded.
  think: the zero step "00 49 49" (hold one step; the next prompt carries
    THINK in its history, the model plans).
  finished: "00 49 49 LAND" (arrived; the follow client holds it as
    arrival, never a landing).

RACEVLA (SerValera/RaceVLA_models drone6_all, HF revision d14f98f9; code
github.com/SerValera/RaceVLA at 46f7e27, read 2026-10-27: server/server.py,
drone/vla_ws/src/vlm_drone/scripts/drone.py; the paper, arXiv 2503.02572)
--------------------------------------------------------------------------
INPUT. OpenVLA's prompt, server.py:32:
    "In: What action should the robot take to {instruction}?\\nOut:"
with the instruction "fly to <the object>" by default (drone.py:30 flies
"Fly through the gates and avoid obstacles"; the paper's are "Fly through
the arch gate", ...: --command takes a fixed instruction too). One frame,
the processor's own 224x224 resize; converted to grey (replicated to RGB)
by default, because drone.py:28 reads /t265/fisheye1/image_raw, the T265's
monochrome fisheye, through cv_bridge as bgr8 (drone.py:215) - our pinhole
colour camera cannot be made fisheye, only grey (--keep-color: as sent).
  Inference is predict_action's own arithmetic, done here so the tokens
are on record: the empty token 29871 appended after "Out:", greedy
generate of 7 tokens with input_ids and pixel_values only (an attention
mask passed with it is not extended for the appended token:
memory/real-vla-integration), then modeling_prismatic.py's predict_action:
bin = vocab_size - token - 1 (vocab 32064 - 64 padding = 32000), clipped
to 0..254, normalized = bin_centers[bin] of linspace(-1, 1, 256), and
un-normalised with dataset_statistics.json under the unnorm key (the
file's only one, "drone_set6_all" for drone6_all; server.py:35 names
"drone_set3" for its older drone3 model): a = 0.5 (n + 1)(q99 - q01) + q01
where mask is true, n where it is false (the gripper slot).

OUTPUT -> STEP (rule racevla_velocity/1). server.py:43-50: vx = a[0], vy =
a[1], vz = a[2], delta_yaw = a[5] (a[3], a[4], a[6] unused; recorded).
drone.py:37-39 and 66-71: the flown velocity is vel_l_xy 1.875 * (vx, vy),
rotated by the drone's ENU yaw into the map frame - the standard rotation
of a body vector, so x is forward and y is LEFT (ROS FLU) - climb vel_k_z
0.5 * vz (MAVROS setpoint_velocity/cmd_vel, ENU: + up), yaw rate yaw_k
1.5 * delta_yaw as twist.angular.z (drone.py:242: rad/s, ENU + =
counter-clockwise). Held until the next prediction: drone.py:208 predicts
on every 20th image of the 30 Hz T265 stream (0.67 s); the paper reports
4 Hz. Over the horizon T (--horizon-s, 1.0 s by default):
    fwd_m = 1.875 vx T     down_m = -0.5 vz T     dpsi_rad = -1.5 delta_yaw T
(dpsi + = clockwise, hence the sign). The lateral 1.875 vy T has no slot in
AeroVLA's step: dropped, recorded as lateral_left_m_dropped. A negative fwd
(backwards) clips to 0, recorded. Fly it with vla_fly --step-as-rate T
(the step flown as a velocity over T, i.e. RaceVLA's own velocity).

THE STEP TEXT (both). The continuous step is clipped to the codec's ranges
(fwd 0..5 m, down -5..5 m, dpsi -1.1..1.1 rad; clipped axes recorded) and
written as the NEAREST bin, round((v - lo) / (hi - lo) * 98): the client
decodes a bin at its cell's lower edge (aerialvla_codec.dequantize), so the
nearest edge is the closest step it can fly, within half a bin. (The
training labels use truncation, quantize: right for labels a model learns,
a full bin of bias for a value that is flown.)

IDENTITY. hello names the model itself, "<family>/<name>@<first 8 hex of
weights_sha256>" with `variant` "foreign" and the `foreign` block
aerialvla_codec.foreign_block validates (bottom_tile "none", speed_words
null, the front_source and its camera; the upstream sources, the mapping
rule and this server's configuration), so a flight records it under its
own name (vla_mission.model_id_for -> "<family>:<name>"), tools/todo.py
counts it as a VLA of that family, and a client refuses it when its
lower tile, speed words or front camera differ (vla_mission.
bottom_tile_check, speed_words_check, front_source_check). weights_sha256
is guardrail.manifest.weights_digest over the shards
model.safetensors.index.json names, plus RaceVLA's dataset_statistics.json
(it changes every action), cached in vla_server_v2's sidecar; hello's
`weight_files` lists them, so a flight on this host compares the digests.
precision bf16, nf4 (bitsandbytes 4-bit) or int4 (torchao int4 weight-only,
DroneCATS only: the Orin's choice, as the PyPI bitsandbytes wheel has no
sm_87 kernels); quantised DroneCATS keeps its vision encoder, projector and
lm_head in bf16. attn sdpa or eager (RaceVLA's server.py:19 uses
flash_attention_2, not installed here). The scripted stand-in is
"fake:<family>-scripted" with no digest: never a VLA.

IMPORTS. No torch, transformers or Pillow at module level. The dronecats
env has no pydantic or shapely, which guardrail/__init__ (the Shield)
needs: the codec and guardrail.manifest are stdlib-only, so when the
package cannot be imported it is registered by its path alone
(_import_codec) and the server imports the codec without it.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import re
import string
import sys
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))


def _import_codec():
    """guardrail.aerialvla_codec, with or without guardrail/__init__ (module
    docstring, IMPORTS): the real package where it imports, else a package
    entry made from its path, which runs no __init__."""
    try:
        import guardrail                                           # noqa: F401
    except ImportError:
        import types
        pkg = types.ModuleType("guardrail")
        pkg.__path__ = [str(ROOT / "guardrail")]
        pkg.__doc__ = ("guardrail, registered by path by demo/vla_server_foreign.py: its "
                       "__init__ (the Shield) could not import in this environment")
        sys.modules["guardrail"] = pkg
    from guardrail import aerialvla_codec
    return aerialvla_codec


C = _import_codec()
import vla_server_v2 as S                                            # noqa: E402

FAMILIES = C.FOREIGN_FAMILIES
MAPPING_RULES = {"dronecats": "dronecats_go_rotate/1", "racevla": "racevla_velocity/1",
                 "ntutvla": "ntutvla_step/1"}
QUANTIZE_RULE = ("nearest bin, round((v - lo) / (hi - lo) * 98), after clipping to the codec's "
                 "range; the client decodes the bin's lower edge, so the flown step is within "
                 "half a bin of the requested one")
NATIVE_OUTPUT = {
    "dronecats": ('one JSON object: {"action":"go","point":[x,y] (0..1000 grid),"depth_m":d '
                  '(canonical)}, {"action":"rotate","angle_deg":a (+ right)}, '
                  '{"action":"think"}, {"action":"finished"}'),
    "racevla": ("7 OpenVLA action tokens -> 7 un-normalised floats; vx, vy, vz = a[0..2], "
                "delta_yaw = a[5] (server.py:43-50)"),
    "ntutvla": ("three distributions over its own grids (training/ntutvla/core.GRIDS), "
                "decoded to fwd_m, down_m, dpsi_rad over the label horizon (core.decode), "
                "plus visible_p and bearing_rad (training signals, not flown)"),
}
# The upstream sources this server's input and mapping were written from
# (module docstring), as read on 2026-10-27. The weights that run are pinned
# by weights_sha256 and the local download's revision (identity `base`).
UPSTREAM = {
    "dronecats": {"model": "naver-ai/DroneCATS-Cosmos3-Edge-2B",
                  "revision": "e9004010f3a56060b72259f8c1eb966bc2cc8a22",
                  "base_model": "nvidia/Cosmos3-Edge", "license": "OpenMDW-1.1",
                  "paper": "arXiv 2609.01404 (Sec. 2, Appendix A.2, C)",
                  "code": "github.com/naver-ai/DroneCATS (benchmark code not released)"},
    "racevla": {"model": "SerValera/RaceVLA_models drone6_all",
                "revision": "d14f98f99c045cb51fa598b3086470d30222437a",
                "base_model": "openvla/openvla-7b (full fine-tune, merged)",
                "license": None, "paper": "arXiv 2503.02572",
                "code": ("github.com/SerValera/RaceVLA@46f7e27a206d3bf7adb247ba8cda586377a8172a "
                         "server/server.py, drone/vla_ws/src/vlm_drone/scripts/drone.py")},
    "ntutvla": {"model": "NTUT-VLA (this repository: training/ntutvla)", "revision": None,
                "base_model": "google/owlvit-base-patch32 (frozen) + a trained head",
                "license": None, "paper": None, "code": "training/ntutvla"},
}
HISTORY_GAP_S = 5.0
MAX_HISTORY = 5                  # the user prompt: "Your last 5 actions"
MAX_SESSIONS = 64                # connections whose history is kept


# --------------------------------------------------------------------------- #
# The step text: a continuous step -> the codec's "FF DD YY" (+ " LAND")
# --------------------------------------------------------------------------- #
def quantize_nearest(value: float, axis: str) -> int:
    """The bin whose lower edge (the client's decode) is nearest `value`,
    after clipping to the axis range (module docstring, THE STEP TEXT)."""
    lo, hi = C.RANGES[axis]
    v = min(max(float(value), lo), hi)
    return int(math.floor((v - lo) / (hi - lo) * C.MAX_BIN + 0.5))


def encode_step(fwd_m: float, down_m: float, dpsi_rad: float, land: bool = False) -> dict:
    """A continuous step in metres and radians -> {"text" (the step text with
    the EOS marker, what parse_action reads), "bins", "requested", "clipped"
    (the axes outside the codec's range), "flown" (the step the bins decode
    to), "land"}. ValueError for a value that is not a finite number."""
    req = {"fwd_m": fwd_m, "down_m": down_m, "dpsi_rad": dpsi_rad}
    for k, v in req.items():
        if not C._finite_number(v):
            raise ValueError(f"{k} {v!r} is not a finite number")
        req[k] = float(v)
    clipped = [k for k in C.AXES if not C.RANGES[k][0] <= req[k] <= C.RANGES[k][1]]
    bins = tuple(quantize_nearest(req[k], k) for k in C.AXES)
    return {"text": C.action_text(bins, land=bool(land)) + C.EOS_TEXT, "bins": list(bins),
            "requested": req, "clipped": clipped, "flown": C.bins_to_step(bins).to_dict(),
            "land": bool(land)}


# --------------------------------------------------------------------------- #
# The camera
# --------------------------------------------------------------------------- #
def intrinsics(width: int, height: int, hfov_deg: float) -> dict:
    """A pinhole camera of `hfov_deg` over a width x height image: square
    pixels, principal point at the centre (the paper's A.2), k = fx / W =
    1 / (2 tan(hfov/2)) (the card's k; 0.5 at 90 deg; rounded to 12 places so
    float noise in tan(45 deg) does not reach fx)."""
    w, h = int(width), int(height)
    if w <= 0 or h <= 0:
        raise ValueError(f"image size {w}x{h}")
    if not (C._finite_number(hfov_deg) and 0.0 < float(hfov_deg) < 180.0):
        raise ValueError(f"hfov_deg {hfov_deg!r} is not in (0, 180)")
    k = round(0.5 / math.tan(math.radians(float(hfov_deg)) / 2.0), 12)
    fx = k * w
    return {"width": w, "height": h, "hfov_deg": float(hfov_deg), "k": k, "fx": fx, "fy": fx,
            "cx": w / 2.0, "cy": h / 2.0}


def camera_aux(intr: dict) -> str:
    """The card's camera_aux string: json.dumps({"width", "height", "fx",
    "fy"}) with fx = fy = round(k * w, 2)."""
    fx = round(intr["k"] * intr["width"], 2)
    return json.dumps({"width": intr["width"], "height": intr["height"], "fx": fx, "fy": fx})


def model_size(w: int, h: int, max_width: int) -> tuple[int, int]:
    """The card's downscale: wider than max_width -> (max_width, int(h *
    max_width / w)); else unchanged."""
    w, h = int(w), int(h)
    if w <= max_width:
        return w, h
    return int(max_width), int(h * max_width / w)


def image_sha256(img) -> str:
    """SHA-256 of the RGB8 bytes the model's processor is given."""
    return hashlib.sha256(img.convert("RGB").tobytes()).hexdigest()


def _decode_front(req: dict):
    return C.decode_image_b64(req["front_jpg_b64"], "front_jpg_b64")


# --------------------------------------------------------------------------- #
# The command, from the object words
# --------------------------------------------------------------------------- #
DRONECATS_COMMAND = "go to {target}"
RACEVLA_INSTRUCTION = "fly to {target}"
_ARTICLE = re.compile(r"^(?:a|an|the)\s+", re.I)


def target_words(object_text: str) -> str:
    """"a red car" -> "the red car": the object words as a referring
    expression (the paper's "go to the black SUV"). CodecError
    (ERR_BAD_OBJECT) for empty words: there is nothing to go to."""
    t = (object_text or "").strip() if isinstance(object_text, str) else ""
    t = _ARTICLE.sub("", t)
    if not t:
        raise C.CodecError(C.ERR_BAD_OBJECT, "the object words are empty: a foreign model's "
                                             "command names its target")
    return "the " + t


def command_text(object_text: str, template: str, allow_fixed: bool = False) -> str:
    """The template with {target} = target_words(object_text). ValueError for
    a template whose only field is not {target} - or, unless `allow_fixed`
    (a fixed instruction, RaceVLA's "Fly through the gates..."), one with no
    field at all."""
    fields = [f for _, f, _, _ in string.Formatter().parse(template) if f is not None]
    if fields not in (["target"], [] if allow_fixed else ["target"]):
        raise ValueError(f"command template {template!r}: its one field is {{target}}")
    if not fields:
        return template
    return template.format(target=target_words(object_text))


# --------------------------------------------------------------------------- #
# DroneCATS: its output, read strictly, and its mapping
# --------------------------------------------------------------------------- #
DRONECATS_ACTIONS = ("go", "rotate", "think", "finished")
PLAN_FIELDS = ("target_in_view", "target_match", "closing_in", "current_frame",
               "action_planning")
ROTATE_CLAMP_DEG = 90.0          # the paper's parser clamps a rotate to +-90 deg
DEPTH_MAX = 200.0                # plan_response_schema.json depth_m maximum
FIRST_STEP_HISTORY = "(none yet \u2014 this is the first step)"


def _no_constant(name):
    raise ValueError(f"{name} is not a number")


def _number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def parse_dronecats(text: Any) -> dict:
    """The model's decoded text -> {"ok", "action", "reason", "plan",
    "clamped", "unknown_keys"}. Accepted: exactly one JSON object (surrounding
    whitespace only) whose "action" is go / rotate / think / finished; go
    with "point" two numbers in 0..1000 and "depth_m" a number in 0..200
    (plan_response_schema.json), rotate with a number "angle_deg", clamped to
    +-90 (recorded in `clamped`). NaN and Infinity are refused. The planning
    fields a reply after a THINK carries are recorded in `plan`."""
    out: dict[str, Any] = {"ok": False, "action": None, "reason": None, "plan": None,
                           "clamped": [], "unknown_keys": []}
    if not isinstance(text, str):
        out["reason"] = f"not text ({type(text).__name__})"
        return out
    body = text.strip()
    if not body:
        out["reason"] = "empty"
        return out
    try:
        obj = json.loads(body, parse_constant=_no_constant)
    except (ValueError, RecursionError) as exc:
        out["reason"] = f"not one JSON object ({type(exc).__name__}: {exc})"
        return out
    if not isinstance(obj, dict):
        out["reason"] = f"a JSON {type(obj).__name__}, not an object"
        return out
    act = obj.get("action")
    if act not in DRONECATS_ACTIONS:
        out["reason"] = f"action {act!r} is not one of {list(DRONECATS_ACTIONS)}"
        return out
    action: dict[str, Any] = {"action": act}
    if act == "go":
        pt, d = obj.get("point"), obj.get("depth_m")
        if not (isinstance(pt, list) and len(pt) == 2 and all(_number(v) for v in pt)):
            out["reason"] = f"go point {pt!r} is not two numbers"
            return out
        if not all(0 <= v <= 1000 for v in pt):
            out["reason"] = f"go point {pt!r} is outside the 0..1000 grid"
            return out
        if not (_number(d) and 0 <= d <= DEPTH_MAX):
            out["reason"] = f"go depth_m {d!r} is not a number in 0..{DEPTH_MAX:g}"
            return out
        action.update({"point": [pt[0], pt[1]], "depth_m": d})
    elif act == "rotate":
        a = obj.get("angle_deg")
        if not _number(a):
            out["reason"] = f"rotate angle_deg {a!r} is not a number"
            return out
        if abs(a) > ROTATE_CLAMP_DEG:
            a = math.copysign(ROTATE_CLAMP_DEG, a)
            out["clamped"].append("angle_deg")
        action["angle_deg"] = a
    plan = {k: obj[k] for k in PLAN_FIELDS if k in obj}
    known = {"action", "point", "depth_m", "angle_deg", *PLAN_FIELDS}
    out.update({"ok": True, "action": action, "plan": plan or None,
                "unknown_keys": sorted(str(k) for k in obj if k not in known)})
    return out


@dataclass(frozen=True)
class GoExecutor:
    """How a go becomes a step (module docstring, DRONECATS OUTPUT -> STEP):
    the step cap (m), the near band (below near_m metres of depth, x
    near_scale), the stop band (below stop_m, x0), the altitude hold (|down|
    below hold_frac of the horizontal distance). None: not applied."""
    name: str
    step_cap_m: float | None
    near_m: float | None
    near_scale: float
    stop_m: float | None
    hold_frac: float | None

    def as_dict(self) -> dict:
        return asdict(self)


# The paper's simulated controller (arXiv 2609.01404, A.2): "held for at most
# 2.5 s at up to 1.2 m/s, so one step moves at most 3 m"; "Under 5 m the step
# is scaled by 0.6, and under 2 m it is scaled to zero"; altitude held under
# 0.15 of the horizontal distance.
PAPER_EXECUTOR = GoExecutor("paper", 3.0, 5.0, 0.6, 2.0, 0.15)
DIRECT_EXECUTOR = GoExecutor("direct", None, None, 1.0, None, None)
EXECUTORS = {"paper": PAPER_EXECUTOR, "direct": DIRECT_EXECUTOR}


def map_dronecats(action: dict, intr: dict, pitch_deg: float = 0.0,
                  executor: GoExecutor = PAPER_EXECUTOR) -> dict:
    """A parsed DroneCATS action -> {"kind", "fwd_m", "down_m", "dpsi_rad",
    "land", "geometry"} (continuous, before the codec's clip; module
    docstring, OUTPUT -> STEP)."""
    kind = action["action"]
    zero = {"kind": kind, "fwd_m": 0.0, "down_m": 0.0, "dpsi_rad": 0.0, "land": False,
            "geometry": None}
    if kind == "finished":
        return {**zero, "land": True}
    if kind == "think":
        return zero
    if kind == "rotate":
        return {**zero, "dpsi_rad": math.radians(float(action["angle_deg"])),
                "geometry": {"angle_deg": action["angle_deg"]}}
    if kind != "go":
        raise ValueError(f"not a DroneCATS action: {action!r}")
    x, y = (float(v) for v in action["point"])
    d_canon = float(action["depth_m"])
    depth = d_canon * intr["k"]                     # metres along the optical axis
    px, py = x * intr["width"] / 1000.0, y * intr["height"] / 1000.0
    right = (px - intr["cx"]) / intr["fx"] * depth
    down_cam = (py - intr["cy"]) / intr["fy"] * depth
    p = math.radians(float(pitch_deg))
    fwd_l = depth * math.cos(p) + down_cam * math.sin(p)
    down_l = -depth * math.sin(p) + down_cam * math.cos(p)
    bearing = math.atan2(right, fwd_l)
    horiz = math.hypot(fwd_l, right)
    held = executor.hold_frac is not None and abs(down_l) < executor.hold_frac * horiz
    v_down = 0.0 if held else down_l
    norm = math.hypot(horiz, v_down)
    cap = (1.0 if executor.step_cap_m is None or norm <= executor.step_cap_m
           else executor.step_cap_m / norm)
    band = 1.0
    if executor.stop_m is not None and depth < executor.stop_m:
        band = 0.0
    elif executor.near_m is not None and depth < executor.near_m:
        band = executor.near_scale
    geo = {"depth_canonical": d_canon, "k": intr["k"], "depth_axis_m": depth,
           "pixel": [px, py], "camera_m": {"right": right, "down": down_cam, "forward": depth},
           "pitch_deg": float(pitch_deg), "forward_m": fwd_l, "right_m": right, "down_m": down_l,
           "horizontal_m": horiz, "bearing_rad": bearing,
           "elevation_rad": math.atan2(down_l, horiz), "range_m": math.hypot(horiz, down_l),
           "altitude_held": held, "cap_scale": cap, "band_scale": band,
           "executor": executor.name}
    return {"kind": "go", "fwd_m": horiz * cap * band, "down_m": v_down * cap * band,
            "dpsi_rad": bearing, "land": False, "geometry": geo}


def _num_text(v) -> str:
    v = float(v)
    return str(int(v)) if v.is_integer() else f"{v:g}"


def history_line(action: dict) -> str:
    """One action as the card's history writes it: "GO x=402 y=656
    depth=3.2m", "ROTATE angle=-90deg", "THINK", "FINISHED"."""
    a = action["action"]
    if a == "go":
        x, y = action["point"]
        return f"GO x={_num_text(x)} y={_num_text(y)} depth={float(action['depth_m']):.1f}m"
    if a == "rotate":
        return f"ROTATE angle={_num_text(action['angle_deg'])}deg"
    return a.upper()


def history_text(entries: list[dict]) -> str:
    """The {history} of the user prompt from [{"action", "t"}, ...], oldest
    first (at most MAX_HISTORY): each "done" with the time to the next
    capture, the last "in progress", or FIRST_STEP_HISTORY when empty."""
    if not entries:
        return FIRST_STEP_HISTORY
    n = len(entries)
    lines = []
    for i, e in enumerate(entries):
        tag = ("[in progress]  <- latest" if i == n - 1
               else f"[done, {float(entries[i + 1]['t']) - float(e['t']):.1f}s]")
        lines.append(f"  step {i + 1} of {n}: {history_line(e['action'])}  {tag}")
    return "\n".join(lines)


def dronecats_messages(system_tpl: str, user_tpl: str, size: tuple[int, int], instruction: str,
                       history: str, aux: str) -> list[dict]:
    """The card's chat messages, the image entry without its pixels (the
    backend inserts the frame): system_prompt.txt formatted with the frame
    size and its quarter, user_prompt.txt with the command and the history,
    then {"type": "image", "camera_aux": aux}."""
    w, h = size
    return [{"role": "system", "content": [{"type": "text", "text": system_tpl.format(
                frame_w=w, frame_h=h, quarter_x=w // 4, quarter_y=h // 4)}]},
            {"role": "user", "content": [
                {"type": "text", "text": user_tpl.format(instruction=instruction,
                                                         history=history)},
                {"type": "image", "camera_aux": aux}]}]


def _sha_text(t: str) -> str:
    return hashlib.sha256(t.encode("utf-8")).hexdigest()


class DroneCATSFamily:
    """DroneCATS's input, reader and mapping (module docstring). `executor`
    is "paper" / "direct" or a GoExecutor."""
    name = "dronecats"
    history = True

    def __init__(self, system_prompt: str, user_prompt: str, *, command: str = DRONECATS_COMMAND,
                 executor: Any = "paper", max_width: int = 640, max_new_tokens: int = 128,
                 plan_max_new_tokens: int = 1024, prompt_files: dict | None = None):
        for tpl, need in ((system_prompt, {"frame_w", "frame_h", "quarter_x", "quarter_y"}),
                          (user_prompt, {"instruction", "history"})):
            got = {f for _, f, _, _ in string.Formatter().parse(tpl) if f}
            if not need <= got:
                raise ValueError(f"a DroneCATS prompt template lacks {sorted(need - got)}")
        command_text("a car", command)                 # refuses a bad template now
        self.system_prompt, self.user_prompt, self.command = system_prompt, user_prompt, command
        self.executor = EXECUTORS[executor] if isinstance(executor, str) else executor
        if not isinstance(self.executor, GoExecutor):
            raise ValueError(f"go executor {executor!r} is not one of {list(EXECUTORS)}")
        if int(max_width) <= 0 or int(max_new_tokens) <= 0 or int(plan_max_new_tokens) <= 0:
            raise ValueError("max_width and the token limits must be positive")
        self.max_width = int(max_width)
        self.max_new_tokens = int(max_new_tokens)
        self.plan_max_new_tokens = int(plan_max_new_tokens)
        self.prompt_files = dict(prompt_files or {})

    @classmethod
    def from_dir(cls, weights_dir: str | Path, **kw) -> "DroneCATSFamily":
        """The prompt files of a download, verbatim (the card: "Use the prompt
        files as they are")."""
        d = Path(weights_dir) / "prompts"
        texts, files = {}, {}
        for k in ("system_prompt", "user_prompt"):
            p = d / f"{k}.txt"
            texts[k] = p.read_text(encoding="utf-8")
            files[p.name] = _sha_text(texts[k])
        return cls(texts["system_prompt"], texts["user_prompt"], prompt_files=files, **kw)

    def prepare(self, img) -> tuple[Any, dict]:
        """The frame at most max_width wide, the card's BOX downscale."""
        w, h = img.size
        size = model_size(w, h, self.max_width)
        if size == (w, h):
            return img, {"resized": False}
        from PIL import Image
        return img.resize(size, getattr(Image, "Resampling", Image).BOX), \
            {"resized": True, "resample": "box"}

    def inputs(self, req: dict, intr: dict, history: list[dict]) -> dict:
        cmd = command_text(req["object"], self.command)
        hist = history_text(history[-MAX_HISTORY:])
        aux = camera_aux(intr)
        after_think = bool(history) and history[-1]["action"]["action"] == "think"
        return {"messages": dronecats_messages(self.system_prompt, self.user_prompt,
                                               (intr["width"], intr["height"]), cmd, hist, aux),
                "command": cmd, "history": hist, "camera_aux": aux,
                "max_new_tokens": self.plan_max_new_tokens if after_think else self.max_new_tokens}

    def parse(self, g: dict) -> dict:
        nat = parse_dronecats(g.get("raw"))
        nat["text"] = g.get("raw") if isinstance(g.get("raw"), str) else repr(g.get("raw"))
        return nat

    def map(self, action: dict, intr: dict, pitch_deg: float = 0.0) -> dict:
        return map_dronecats(action, intr, pitch_deg=pitch_deg, executor=self.executor)

    def mapping_params(self) -> dict:
        return {"executor": self.executor.as_dict()}

    def describe(self) -> dict:
        return {"command": self.command, "executor": self.executor.as_dict(),
                "max_width": self.max_width, "max_new_tokens": self.max_new_tokens,
                "plan_max_new_tokens": self.plan_max_new_tokens, "history_max": MAX_HISTORY,
                "prompt_files_sha256": self.prompt_files}


# --------------------------------------------------------------------------- #
# RaceVLA: OpenVLA's action tokens, un-normalised, and its mapping
# --------------------------------------------------------------------------- #
RACEVLA_PROMPT = "In: What action should the robot take to {instruction}?\nOut:"
# drone.py:37-39 (vel_l_xy, vel_k_z, yaw_k) as prepare_vel applies them (66-71)
RACEVLA_GAINS = {"xy": 1.875, "z": 0.5, "yaw": 1.5}
OPENVLA_VOCAB_SIZE = 32000        # config text vocab 32064 - pad_to_multiple_of 64
OPENVLA_N_BINS = 256              # config n_action_bins
OPENVLA_EMPTY_TOKEN = 29871       # predict_action appends it after "Out:"
ACTION_DIM = 7


def bin_centers(n_bins: int = OPENVLA_N_BINS) -> list[float]:
    """modeling_prismatic.py: bins = linspace(-1, 1, n_bins), centres the
    midpoints (n_bins - 1 of them)."""
    step = 2.0 / (n_bins - 1)
    bins = [-1.0 + i * step for i in range(n_bins - 1)] + [1.0]
    return [(bins[i] + bins[i + 1]) / 2.0 for i in range(n_bins - 1)]


def tokens_to_normalized(tokens, vocab_size: int = OPENVLA_VOCAB_SIZE,
                         n_bins: int = OPENVLA_N_BINS) -> list[float]:
    """predict_action: bin = vocab_size - token - 1, clipped to the centres,
    -> the centre (normalised, -1..1)."""
    c = bin_centers(n_bins)
    return [c[min(max(vocab_size - int(t) - 1, 0), len(c) - 1)] for t in tokens]


def unnormalize(normalized, stats: dict) -> list[float]:
    """predict_action's un-normalisation: 0.5 (n + 1)(q99 - q01) + q01 where
    the mask is true (all true without one), n where it is false."""
    q01, q99 = stats["q01"], stats["q99"]
    mask = stats.get("mask") or [True] * len(q01)
    return [0.5 * (float(n) + 1.0) * (hi - lo) + lo if m else float(n)
            for n, lo, hi, m in zip(normalized, q01, q99, mask)]


def load_action_stats(path: str | Path, key: str | None = None) -> tuple[str, dict]:
    """(the unnorm key, its action statistics {"q01", "q99", "mask"}) from a
    dataset_statistics.json: the file's only key, or `key`, which must be in
    it (predict_action's _check_unnorm_key). ValueError otherwise."""
    p = Path(path)
    doc = json.loads(p.read_text(encoding="utf-8"))
    keys = sorted(doc) if isinstance(doc, dict) else []
    if key is None:
        if len(keys) != 1:
            raise ValueError(f"{p} has unnorm keys {keys!r}: name one (--unnorm-key)")
        key = keys[0]
    elif key not in keys:
        raise ValueError(f"unnorm key {key!r} is not one of {p}'s {keys!r}")
    act = (doc[key] or {}).get("action") if isinstance(doc[key], dict) else None
    try:
        q01 = [float(v) for v in act["q01"]]
        q99 = [float(v) for v in act["q99"]]
        mask = [bool(v) for v in act.get("mask") or [True] * len(q01)]
    except (TypeError, KeyError, ValueError):
        raise ValueError(f"{p}[{key!r}].action has no q01 / q99 lists") from None
    if not len(q01) == len(q99) == len(mask) == ACTION_DIM:
        raise ValueError(f"{p}[{key!r}].action is not {ACTION_DIM}-dimensional")
    return key, {"q01": q01, "q99": q99, "mask": mask}


def map_racevla(a, horizon_s: float = 1.0, gains: dict = RACEVLA_GAINS) -> dict:
    """RaceVLA's un-normalised action -> {"kind": "velocity", "fwd_m",
    "down_m", "dpsi_rad", "land", "geometry"} (module docstring, RACEVLA
    OUTPUT -> STEP): the drone code's velocities over the horizon."""
    a = [float(v) for v in a]
    vx, vy, vz, dyaw = a[0], a[1], a[2], a[5]
    fwd_v, left_v = gains["xy"] * vx, gains["xy"] * vy
    climb, rate = gains["z"] * vz, gains["yaw"] * dyaw
    t = float(horizon_s)
    geo = {"vx": vx, "vy": vy, "vz": vz, "delta_yaw": dyaw, "unused": {"3": a[3], "4": a[4],
                                                                       "6": a[6]},
           "gains": dict(gains), "forward_mps": fwd_v, "left_mps": left_v, "climb_mps": climb,
           "yaw_rate_ccw_rad_s": rate, "horizon_s": t, "lateral_left_m_dropped": left_v * t}
    return {"kind": "velocity", "fwd_m": fwd_v * t, "down_m": -climb * t, "dpsi_rad": -rate * t,
            "land": False, "geometry": geo}


class RaceVLAFamily:
    """RaceVLA's input, reader and mapping (module docstring)."""
    name = "racevla"
    history = False

    def __init__(self, stats: dict, unnorm_key: str, *, instruction: str = RACEVLA_INSTRUCTION,
                 horizon_s: float = 1.0, gains: dict = RACEVLA_GAINS, gray: bool = True,
                 vocab_size: int = OPENVLA_VOCAB_SIZE, n_bins: int = OPENVLA_N_BINS):
        command_text("a car", instruction, allow_fixed=True)      # refuses a bad template now
        if not (C._finite_number(horizon_s) and float(horizon_s) > 0):
            raise ValueError(f"horizon_s {horizon_s!r} must be a positive number of seconds")
        self.stats, self.unnorm_key = stats, unnorm_key
        self.instruction, self.horizon_s = instruction, float(horizon_s)
        self.gains, self.gray = dict(gains), bool(gray)
        self.vocab_size, self.n_bins = int(vocab_size), int(n_bins)

    def prepare(self, img) -> tuple[Any, dict]:
        """Grey, replicated to RGB (the T265 fisheye1 stream is monochrome),
        unless the family keeps the colour."""
        if not self.gray:
            return img, {"gray": False}
        return img.convert("L").convert("RGB"), {"gray": True}

    def inputs(self, req: dict, intr: dict, history: list[dict]) -> dict:
        ins = command_text(req["object"], self.instruction, allow_fixed=True)
        return {"prompt": RACEVLA_PROMPT.format(instruction=ins), "instruction": ins,
                "max_new_tokens": ACTION_DIM}

    def parse(self, g: dict) -> dict:
        raw = g.get("raw")
        out: dict[str, Any] = {"ok": False, "action": None, "reason": None,
                               "tokens": raw if isinstance(raw, list) else None,
                               "unnorm_key": self.unnorm_key,
                               "text": " ".join(str(t) for t in raw) if isinstance(raw, list)
                               else repr(raw)}
        if not (isinstance(raw, list) and len(raw) == ACTION_DIM
                and all(type(t) is int for t in raw)):
            out["reason"] = f"not {ACTION_DIM} integer action tokens: {raw!r}"
            return out
        lo = self.vocab_size - self.n_bins
        n = tokens_to_normalized(raw, self.vocab_size, self.n_bins)
        out.update({"ok": True, "action": {"action": unnormalize(n, self.stats), "normalized": n},
                    "tokens_outside_action_bins": sum(1 for t in raw
                                                      if not lo <= t < self.vocab_size)})
        return out

    def map(self, action: dict, intr: dict, pitch_deg: float = 0.0) -> dict:
        return map_racevla(action["action"], self.horizon_s, self.gains)

    def mapping_params(self) -> dict:
        return {"horizon_s": self.horizon_s, "gains": dict(self.gains)}

    def describe(self) -> dict:
        return {"instruction": self.instruction, "prompt": RACEVLA_PROMPT,
                "unnorm_key": self.unnorm_key, "stats": self.stats, "horizon_s": self.horizon_s,
                "gains": dict(self.gains), "gray": self.gray, "vocab_size": self.vocab_size,
                "n_bins": self.n_bins}


class NTUTVLAFamily:
    """The lab's own model (training/ntutvla): its input is the VLAFront frame
    as sent (256 x 256, no resize), the object words as the phrase its
    backbone grounds, and the capture's time and pose (its history ring and
    proprio, kept by NTUTVLAModel). Its answer is already a step over the
    label horizon, so the mapping is the identity (rule ntutvla_step/1); the
    compass phrase, the hint source and the down frame are ignored, as for
    the other foreign families."""
    name = "ntutvla"
    history = False           # the model keeps its own ring of frames, not of actions

    def __init__(self, horizon_s: float):
        if not (C._finite_number(horizon_s) and float(horizon_s) > 0):
            raise ValueError(f"horizon_s {horizon_s!r} must be a positive number of seconds")
        self.horizon_s = float(horizon_s)

    def prepare(self, img) -> tuple[Any, dict]:
        return img.convert("RGB"), {}

    def inputs(self, req: dict, intr: dict, history: list[dict]) -> dict:
        return {"instruction": req["object"], "t_capture": float(req["t_capture"]),
                "pose": dict(req["pose"])}

    def parse(self, g: dict) -> dict:
        raw = g.get("raw")
        out: dict[str, Any] = {"ok": False, "action": None, "reason": None,
                               "text": json.dumps(raw, sort_keys=True, default=repr)}
        if not (isinstance(raw, dict) and all(C._finite_number(raw.get(k)) for k in C.AXES)):
            out["reason"] = f"not a finite step {list(C.AXES)}: {raw!r}"
            return out
        out.update({"ok": True, "action": dict(raw)})
        return out

    def map(self, action: dict, intr: dict, pitch_deg: float = 0.0) -> dict:
        geo = {k: action.get(k) for k in ("visible_p", "bearing_rad", "slots_valid",
                                           "history_reset")}
        return {"kind": "step", "fwd_m": float(action["fwd_m"]),
                "down_m": float(action["down_m"]), "dpsi_rad": float(action["dpsi_rad"]),
                "land": False, "geometry": {**geo, "horizon_s": self.horizon_s}}

    def mapping_params(self) -> dict:
        return {"horizon_s": self.horizon_s}

    def describe(self) -> dict:
        return {"horizon_s": self.horizon_s, "input": "VLAFront frame + object words + pose"}


# --------------------------------------------------------------------------- #
# The models (heavy imports inside)
# --------------------------------------------------------------------------- #
class ScriptedForeignModel:
    """A stand-in that answers scripted NATIVE outputs in turn (a DroneCATS
    text, a RaceVLA token list), for client development and tests. Its
    calls are kept: the image, the inputs, the history and the prompt
    text (the inputs as canonical JSON)."""
    precision, attn, merged = "fake", "none", False

    def __init__(self, outputs: list, delay_s: float = 0.0):
        if not outputs:
            raise ValueError("ScriptedForeignModel needs at least one scripted output")
        self.outputs, self.delay_s = list(outputs), float(delay_s)
        self.calls: list[dict] = []

    def generate(self, image, inputs: dict) -> dict:
        t0 = time.perf_counter()
        if self.delay_s:
            time.sleep(self.delay_s)
        raw = self.outputs[len(self.calls) % len(self.outputs)]
        text = json.dumps(inputs, sort_keys=True, ensure_ascii=False, default=repr)
        self.calls.append({"image": image, "inputs": inputs, "history": inputs.get("history"),
                           "prompt_text": text})
        dt = time.perf_counter() - t0
        return {"raw": copy.deepcopy(raw), "prompt_text": text, "pre_s": 0.0, "gen_s": dt,
                "model_s": dt, "new_tokens": None, "prompt_tokens": None}


# DroneCATS nf4: the modules kept in bf16 (the vision encoder, its projector, the
# output head); the language layers are quantised.
DRONECATS_NF4_SKIP = ("visual", "projector", "lm_head")
DRONECATS_INT4_GROUP = 128
# torchao >= 0.15 (transformers 5.20's minimum): its default int4 packing needs
# fbgemm-gpu-genai, which has no Jetson build; tile_packed_to_4d is torch's own
# tinygemm kernel (checked on the Orin Nano, torch 2.8 / torchao 0.15)
DRONECATS_INT4_PACKING = "tile_packed_to_4d"
# the package each quantised precision needs, checked before a model loads
PRECISION_PACKAGE = {"nf4": "bitsandbytes", "int4": "torchao"}


def package_available(name: str) -> bool:
    """`name` is importable here (checked before a model is loaded)."""
    import importlib.util
    return importlib.util.find_spec(name) is not None


def bitsandbytes_available() -> bool:
    return package_available("bitsandbytes")


def _sync(torch, device: str) -> None:
    if str(device).startswith("cuda"):
        torch.cuda.synchronize()


class DroneCATSModel:
    """Cosmos3EdgeForConditionalGeneration + its processor (the card's
    loading code; transformers>=5.18, the dronecats env). bf16 is the card's
    precision. nf4 (bitsandbytes 4-bit, compute bf16) and int4 (torchao
    int4 weight-only, group 128: the kernel is in torch itself, so it runs on
    the Orin, where the PyPI bitsandbytes wheel has no sm_87 kernels) quantise
    the language layers only - the vision encoder, the projector and lm_head
    stay bf16 (DRONECATS_NF4_SKIP) - so the 2B model fits beside the detector
    on an 8 GB Jetson."""

    def __init__(self, weights_dir: str | Path, *, attn: str, device: str = "cuda:0",
                 precision: str = "bf16"):
        import torch
        from transformers import AutoProcessor, Cosmos3EdgeForConditionalGeneration
        self.torch, self.device = torch, device
        t0 = time.perf_counter()
        self.processor = AutoProcessor.from_pretrained(str(weights_dir))
        kw: dict[str, Any] = dict(dtype=torch.bfloat16, device_map={"": device},
                                  attn_implementation=attn)
        if precision == "nf4":
            from transformers import BitsAndBytesConfig
            kw["quantization_config"] = BitsAndBytesConfig(
                load_in_4bit=True, bnb_4bit_compute_dtype=torch.bfloat16,
                bnb_4bit_quant_type="nf4", llm_int8_skip_modules=list(DRONECATS_NF4_SKIP))
        elif precision == "int4":
            from torchao.quantization import Int4WeightOnlyConfig
            from transformers import TorchAoConfig
            kw["quantization_config"] = TorchAoConfig(
                Int4WeightOnlyConfig(group_size=DRONECATS_INT4_GROUP,
                                     int4_packing_format=DRONECATS_INT4_PACKING),
                modules_to_not_convert=list(DRONECATS_NF4_SKIP))
        self.model = Cosmos3EdgeForConditionalGeneration.from_pretrained(
            str(weights_dir), **kw).eval()
        self.timing = {"load_total_s": time.perf_counter() - t0}

    def generate(self, image, inputs: dict) -> dict:
        torch = self.torch
        t0 = time.perf_counter()
        msgs = copy.deepcopy(inputs["messages"])
        msgs[1]["content"][1]["image"] = image
        text = self.processor.apply_chat_template(msgs, add_generation_prompt=True,
                                                  tokenize=False, enable_thinking=False,
                                                  skip_reasoning=True)
        if "<|camera_aux|>" in text:
            raise ValueError("the chat template left its <|camera_aux|> placeholder: the model "
                             "would get an input it was never trained on (the card)")
        enc = self.processor(text=[text], images=[image], return_tensors="pt").to(
            self.model.device)
        n_prompt = int(enc["input_ids"].shape[1])
        t1 = time.perf_counter()
        with torch.inference_mode():
            out = self.model.generate(**enc, max_new_tokens=int(inputs["max_new_tokens"]),
                                      do_sample=False)
        _sync(torch, self.device)
        t2 = time.perf_counter()
        gen = out[0, n_prompt:]
        raw = self.processor.decode(gen, skip_special_tokens=True)
        t3 = time.perf_counter()
        return {"raw": raw, "prompt_text": text, "new_tokens": int(gen.shape[0]),
                "prompt_tokens": n_prompt, "pre_s": t1 - t0, "gen_s": t2 - t1, "model_s": t3 - t0}


class RaceVLAModel:
    """RaceVLA's merged OpenVLA-7B (trust_remote_code, the vla-real env's
    transformers 4.40.1). generate() is predict_action up to the tokens
    (module docstring, RACEVLA INPUT)."""

    def __init__(self, weights_dir: str | Path, *, precision: str, attn: str,
                 device: str = "cuda:0"):
        import torch
        from transformers import AutoModelForVision2Seq, AutoProcessor
        self.torch, self.device = torch, device
        t0 = time.perf_counter()
        self.processor = AutoProcessor.from_pretrained(str(weights_dir), trust_remote_code=True)
        kw: dict[str, Any] = dict(attn_implementation=attn, torch_dtype=torch.bfloat16,
                                  low_cpu_mem_usage=True, trust_remote_code=True,
                                  device_map={"": device})
        if precision == "nf4":
            from transformers import BitsAndBytesConfig
            kw["quantization_config"] = BitsAndBytesConfig(
                load_in_4bit=True, bnb_4bit_compute_dtype=torch.bfloat16,
                bnb_4bit_quant_type="nf4")
        self.model = AutoModelForVision2Seq.from_pretrained(str(weights_dir), **kw).eval()
        self.vocab_size = int(getattr(self.model, "vocab_size", OPENVLA_VOCAB_SIZE))
        self.n_bins = int(len(getattr(self.model, "bin_centers", [0] * (OPENVLA_N_BINS - 1)))) + 1
        self.norm_stats = getattr(self.model, "norm_stats", None)
        self.timing = {"load_total_s": time.perf_counter() - t0}

    def generate(self, image, inputs: dict) -> dict:
        torch = self.torch
        t0 = time.perf_counter()
        enc = self.processor(inputs["prompt"], image, return_tensors="pt")     # server.py:33
        ids = enc["input_ids"]
        if int(ids[0, -1]) != OPENVLA_EMPTY_TOKEN:
            ids = torch.cat((ids, torch.tensor([[OPENVLA_EMPTY_TOKEN]], dtype=ids.dtype)), dim=1)
        pv = enc["pixel_values"].to(self.device, dtype=torch.bfloat16)
        t1 = time.perf_counter()
        with torch.inference_mode():
            out = self.model.generate(ids.to(self.device), pixel_values=pv,
                                      max_new_tokens=ACTION_DIM, do_sample=False)
        _sync(torch, self.device)
        t2 = time.perf_counter()
        toks = [int(t) for t in out[0, -ACTION_DIM:].tolist()]
        return {"raw": toks, "prompt_text": inputs["prompt"], "new_tokens": ACTION_DIM,
                "prompt_tokens": int(ids.shape[1]), "pre_s": t1 - t0, "gen_s": t2 - t1,
                "model_s": time.perf_counter() - t0}


class NTUTVLAModel:
    """training/ntutvla: the frozen OWL-ViT backbone and the trained head of
    <weights>/head.pt. generate() grounds the object words in the frame,
    pushes the maps into its history ring (core.Ring: one flight at a time,
    reset on a gap or a time that runs backwards) and answers the head's
    decoded step. The checkpoint's core constants must be this checkout's
    (history offsets, grids, PCA size): refused otherwise."""
    precision, attn, merged = "fp16", "sdpa", False

    def __init__(self, weights_dir: str | Path, *, device: str = "cuda:0"):
        import numpy as np
        import torch
        from training.ntutvla import core as NC
        from training.ntutvla.backbone import OwlBackbone
        from training.ntutvla.model import HeadConfig, NTUTVLAHead
        self.np, self.torch, self.NC, self.device = np, torch, NC, device
        t0 = time.perf_counter()
        ck = torch.load(Path(weights_dir) / "head.pt", map_location=device, weights_only=False)
        mine = {"HIST_OFFSETS_S": NC.HIST_OFFSETS_S, "PCA_DIM": NC.PCA_DIM,
                "LABEL_HORIZON_S": NC.LABEL_HORIZON_S,
                "GRIDS": {a: [float(NC.GRIDS[a][0]), float(NC.GRIDS[a][-1]), len(NC.GRIDS[a])]
                          for a in NC.AXES}}
        theirs = {k: ck["core"][k] for k in mine}
        if json.dumps(mine, sort_keys=True) != json.dumps(
                {k: (list(v) if isinstance(v, tuple) else v) for k, v in theirs.items()},
                sort_keys=True, default=list):
            raise SystemExit(f"{weights_dir}/head.pt was trained with other core constants "
                             f"({theirs}) than this checkout's ({mine})")
        self.horizon_s = float(ck["core"]["LABEL_HORIZON_S"])
        self.bb = OwlBackbone(device=device)
        self.bb.set_pca(ck["pca_mean"], ck["pca_comps"])
        self.head = NTUTVLAHead(HeadConfig(**ck["config"])).to(device).eval()
        self.head.load_state_dict(ck["state_dict"])
        self.ring = NC.Ring()
        self.checkpoint = {"epoch": ck.get("epoch"), "holdout": ck.get("holdout"),
                           "data": {k: ck["data"][k] for k in ("samples_sha256", "n_train",
                                                              "n_holdout")},
                           "pca_sha256": ck.get("pca_sha256"), "backbone": ck.get("backbone")}
        self.timing = {"load_total_s": time.perf_counter() - t0}

    def generate(self, image, inputs: dict) -> dict:
        np, torch, NC = self.np, self.torch, self.NC
        t0 = time.perf_counter()
        g, a = self.bb.maps([np.asarray(image)], [inputs["instruction"]])
        _sync(torch, self.device)
        t1 = time.perf_counter()
        why = self.ring.push(inputs["t_capture"], inputs["pose"], g[0, 0], a[0])
        s = self.ring.slots()
        dev = self.device
        x = [torch.from_numpy(s[k]).unsqueeze(0).to(dev) for k in ("g", "a", "stats", "proprio")]
        valid = torch.from_numpy(s["valid"]).unsqueeze(0).to(dev)
        with torch.inference_mode():
            logits, aux = self.head(*x, valid)
        probs = {ax: torch.softmax(logits[ax][0].float(), 0).cpu().numpy() for ax in NC.AXES}
        aux = aux[0].float().cpu().numpy()
        _sync(torch, self.device)
        t2 = time.perf_counter()
        raw = {ax: NC.decode(probs[ax], ax) for ax in NC.AXES}
        raw.update({"visible_p": float(1.0 / (1.0 + math.exp(-float(aux[0])))),
                    "bearing_rad": float(aux[1]) * 0.8, "slots_valid": s["n_valid"],
                    "history_reset": why,
                    "peak_p": {ax: round(float(probs[ax].max()), 4) for ax in NC.AXES}})
        return {"raw": raw, "pre_s": t1 - t0, "gen_s": t2 - t1,
                "model_s": time.perf_counter() - t0, "new_tokens": None, "prompt_tokens": None}


# --------------------------------------------------------------------------- #
# Identity
# --------------------------------------------------------------------------- #
def weight_files(family: str, weights_dir: str | Path) -> list[Path]:
    """What weights_sha256 covers: the shards model.safetensors.index.json
    names (vla_server_v2.base_weight_files), and for RaceVLA its
    dataset_statistics.json, which changes every action."""
    if family == "ntutvla":
        return [Path(weights_dir) / "head.pt"]
    files = S.base_weight_files(weights_dir)
    if family == "racevla":
        files.append(Path(weights_dir) / "dataset_statistics.json")
    return files


def foreign_identity(family: str, *, name: str, folder: str, revision: str | None,
                     digest: str | None, precision: str, attn: str, front_source: str,
                     facts: dict | None = None) -> dict:
    """The `model` object every answer carries (module docstring, IDENTITY):
    the brief's six fields under the model's own id, `variant` "foreign" and
    the `foreign` block aerialvla_codec.foreign_block validates."""
    if family not in FAMILIES:
        raise ValueError(f"{family!r} is not one of the allow-listed {list(FAMILIES)}")
    if front_source not in C.FRONT_SOURCES:
        raise ValueError(f"front source {front_source!r} is not one of {list(C.FRONT_SOURCES)}")
    sha8 = digest[:8] if isinstance(digest, str) and digest else "unhashed"
    ident = {"id": f"{family}/{name}@{sha8}", "base": f"{folder}@{(revision or 'unknown')[:8]}",
             "weights_sha256": digest, "precision": precision, "attn": attn, "merged": False,
             "agreement": None, "variant": C.FOREIGN_VARIANT,
             "foreign": {"family": family, "model": name, "upstream": dict(UPSTREAM[family]),
                         "bottom_tile": C.FOREIGN_TILE_NONE, "speed_words": None,
                         "front_source": front_source,
                         "camera": dict(C.FRONT_SOURCES[front_source]),
                         "native_output": NATIVE_OUTPUT[family],
                         "mapping": MAPPING_RULES[family], "config": dict(facts or {})}}
    C.foreign_block(ident)                          # this server's own claim, checked
    return ident


def stand_in_identity(family: str) -> dict:
    """The scripted stand-in's: a fake: id, no digest - never a VLA."""
    return {"id": f"fake:{family}-scripted", "base": "none", "weights_sha256": None,
            "precision": "fake", "attn": "none", "merged": False, "agreement": None,
            "variant": "stand-in", "foreign": None}


# --------------------------------------------------------------------------- #
# The protocol core
# --------------------------------------------------------------------------- #
class ForeignServer(S.V2Server):
    """vla_server_v2.V2Server with a foreign model (module docstring): its
    transport, validation, log, counts and health, and an infer that builds
    the model's own input, runs it, reads its native output and maps it to
    the step text. `family` is a DroneCATSFamily / RaceVLAFamily; `model`
    has generate(image, inputs) -> {"raw", "model_s", ...}; decode_front,
    prepare and hash_fn are parameters so the protocol is tested without
    Pillow. The camera is the identity's front source (or `front_source`
    for a stand-in, whose identity declares none)."""

    def __init__(self, family, model, identity: dict, log_path=None, *,
                 hello_extra: dict | None = None, decode_front: Callable | None = None,
                 hash_fn: Callable | None = None, prepare: Callable | None = None,
                 front_source: str | None = None, history_gap_s: float = HISTORY_GAP_S,
                 session_key: Callable[[], Any] = threading.get_ident,
                 clock: Callable[[], float] = time.perf_counter):
        super().__init__(model, identity, log_path, hello_extra=hello_extra,
                         hash_fn=hash_fn or image_sha256, clock=clock)
        self.family = family
        self.decode_front = decode_front or _decode_front
        self.prepare = prepare or family.prepare
        blk = C.foreign_block(identity)
        src = front_source or (blk["front_source"] if blk is not None else C.DEFAULT_FRONT_SOURCE)
        if src not in C.FRONT_SOURCES:
            raise ValueError(f"front source {src!r} is not one of {list(C.FRONT_SOURCES)}")
        self.front_source, self.camera = src, dict(C.FRONT_SOURCES[src])
        if not (C._finite_number(history_gap_s) and float(history_gap_s) > 0):
            raise ValueError("history_gap_s must be a positive number of seconds")
        self.history_gap_s = float(history_gap_s)
        self.session_key = session_key
        self._sessions: dict[Any, dict] = {}
        self._sess_lock = threading.Lock()

    # -- the per-connection history ------------------------------------------ #
    def _session(self, key) -> dict:
        s = self._sessions.get(key)
        if s is None:
            if len(self._sessions) >= MAX_SESSIONS:
                self._sessions.pop(next(iter(self._sessions)))
            s = self._sessions[key] = {"entries": [], "t_last": None}
        return s

    def _history(self, key, t: float) -> tuple[list[dict], str | None]:
        """The session's actions before this capture, and why they were just
        reset (a gap, time running backwards), or None."""
        with self._sess_lock:
            s = self._session(key)
            why = None
            if s["t_last"] is not None:
                if t < s["t_last"]:
                    why = "t_capture went backwards"
                elif t - s["t_last"] > self.history_gap_s:
                    why = (f"a gap of {t - s['t_last']:.1f} s between captures (more than "
                           f"{self.history_gap_s:g} s)")
            if why is not None:
                s["entries"] = []
            s["t_last"] = t
            return [dict(e) for e in s["entries"]], why

    def _remember(self, key, action: dict, t: float) -> None:
        with self._sess_lock:
            s = self._session(key)
            s["entries"] = (s["entries"] + [{"action": action, "t": float(t)}])[-MAX_HISTORY:]

    # -- responses ------------------------------------------------------------ #
    def _infer_error(self, req: Any, error: str) -> dict:
        return {**super()._infer_error(req, error), "native": None, "mapping": None,
                "timing": None}

    def handle(self, req: Any) -> tuple[dict, bool, dict]:
        resp, stop, extra = super().handle(req)
        if isinstance(req, dict) and req.get("op") == "hello" and resp.get("ok"):
            # every RemoteVLA connection says hello first: a new episode
            with self._sess_lock:
                self._sessions.pop(self.session_key(), None)
        return resp, stop, extra

    @staticmethod
    def _check_result(g: Any) -> float:
        if not isinstance(g, dict):
            raise TypeError(f"generate() returned {type(g).__name__}, not a dict")
        if not C._finite_number(g.get("model_s")):
            raise ValueError(f"generate() returned no finite model_s ({g.get('model_s')!r})")
        return float(g["model_s"])

    def infer(self, req: dict) -> tuple[dict, bool, dict]:
        self._count("infer")
        t0 = self.clock()
        key = self.session_key()
        try:
            if req["csp_mode"] != "off":
                raise C.CodecError(C.ERR_BAD_CSP, f"csp_mode {req['csp_mode']!r}: the "
                                                  f"{self.family.name} prompt has no CSP slot")
            img = self.decode_front(req)
            src_size = list(img.size)
            mimg, pinfo = self.prepare(img)
            intr = intrinsics(*mimg.size, self.camera["hfov_deg"])
            hist, reset = self._history(key, float(req["t_capture"]))
            inputs = self.family.inputs(req, intr, hist)
            image_sha = self.hash_fn(mimg)
        except C.CodecError as exc:
            self._count("refused")
            return self._infer_error(req, f"{exc.code}: {exc.detail}"), False, {}
        try:
            with self._model_lock:
                self._in_flight_since = time.time()
                try:
                    g = self.model.generate(mimg, inputs)
                finally:
                    self._in_flight_since = None
            model_s = self._check_result(g)
        except Exception as exc:                                  # noqa: BLE001
            self._t_last_inference_error = time.time()
            self._count("inference_error")
            return (self._infer_error(req, f"{C.ERR_INFERENCE}: {type(exc).__name__}: {exc}"),
                    False, {"inputs": json.loads(json.dumps(inputs, default=repr))})
        self._t_last_ok_infer = time.time()
        nat = self.family.parse(g)
        mapping, raw_text, bins, step, land = None, "", None, None, False
        if nat["ok"]:
            m = self.family.map(nat["action"], intr, pitch_deg=self.camera["pitch_deg"])
            st = encode_step(m["fwd_m"], m["down_m"], m["dpsi_rad"], land=m["land"])
            # the AeroVLA server's own reading of the text this server answers
            p = C.parse_action(st["text"], require_eos=True)
            if not p.parse_ok:                                     # pragma: no cover
                raise AssertionError(f"the step text {st['text']!r} fails the reader: {p.reason}")
            raw_text, bins, step, land = st["text"], list(p.bins), p.step.to_dict(), p.land
            mapping = {"rule": MAPPING_RULES[self.family.name], "kind": m["kind"],
                       "requested": st["requested"], "clipped": st["clipped"], "bins": bins,
                       "flown": step, "land": land, "quantize": QUANTIZE_RULE,
                       "geometry": m["geometry"], "camera": dict(self.camera),
                       **self.family.mapping_params()}
            if self.family.history:
                self._remember(key, nat["action"], float(req["t_capture"]))
        else:
            self._count("parse_fail")
        self._count("land", 1 if land else 0)
        prompt_text = g.get("prompt_text")
        if not isinstance(prompt_text, str):
            prompt_text = json.dumps(inputs, sort_keys=True, ensure_ascii=False, default=repr)
        native = {"family": self.family.name, **{k: v for k, v in nat.items()},
                  "command": inputs.get("command", inputs.get("instruction")),
                  "history": inputs.get("history"), "history_reset": reset,
                  "camera_aux": inputs.get("camera_aux"),
                  "image": {"src_size": src_size, "model_size": list(mimg.size), **pinfo},
                  "ignored": {"phrase": req["phrase"], "hint_source": req["hint_source"],
                              "down_jpg": (None if req["down_jpg_b64"] is None else
                                           f"sent, ignored (bottom_tile "
                                           f"{C.FOREIGN_TILE_NONE!r})")}}
        timing = {**{k: g.get(k) for k in ("pre_s", "gen_s", "model_s", "new_tokens",
                                           "prompt_tokens")},
                  "server_s": round(self.clock() - t0, 6)}
        resp = {"v": C.PROTOCOL_VERSION, "seq": req["seq"], "ok": True, "error": None,
                "raw_text": raw_text, "bins": bins, "step": step, "land": land,
                "parse_ok": bool(nat["ok"]), "latency_s": round(model_s, 6),
                "t_capture": req["t_capture"], "pose": req["pose"],
                "prompt_sha256": _sha_text(prompt_text), "image_sha256": image_sha,
                "model": dict(self.identity), "native": native, "mapping": mapping,
                "timing": timing}
        extra = {"native": native, "mapping": mapping, "timing": timing, "prompt": prompt_text,
                 "parse_reason": nat.get("reason")}
        return resp, False, extra


make_tcp_server = S.make_tcp_server


# --------------------------------------------------------------------------- #
# Building a real server
# --------------------------------------------------------------------------- #
def _name_of(family: str, weights_dir: Path) -> str:
    """The model's name in its id: DroneCATS its folder; RaceVLA the model's
    folder in the repository (drone6_all), not its long run directory."""
    if family == "racevla" and weights_dir.name.startswith("openvla"):
        return weights_dir.parent.name
    return weights_dir.name


def build_family(args, weights: Path):
    if args.model == "ntutvla":
        if not (weights / "head.pt").is_file():
            raise OSError(f"no head.pt in {weights} (python -m training.ntutvla.train --out)")
        return NTUTVLAFamily(horizon_s=args.horizon_s)
    if args.model == "dronecats":
        return DroneCATSFamily.from_dir(
            weights, command=args.command or DRONECATS_COMMAND, executor=args.go_executor,
            max_width=args.max_width, max_new_tokens=args.max_new_tokens,
            plan_max_new_tokens=args.plan_max_new_tokens)
    key, stats = load_action_stats(weights / "dataset_statistics.json", args.unnorm_key)
    return RaceVLAFamily(stats, key, instruction=args.command or RACEVLA_INSTRUCTION,
                         horizon_s=args.horizon_s, gray=not args.keep_color)


def build_real(args, family, weights: Path):
    """Digest first (CPU work: a refusal is cheaper before the model is on
    the card), then the model, then the identity."""
    paths = weight_files(args.model, weights)
    digest, dinfo = S.weights_sha256(paths, Path(args.digest_sidecar), rehash=args.rehash)
    if digest is None:
        raise SystemExit(f"weights_sha256 did not resolve over {[str(p) for p in paths]}: a "
                         f"named weight is missing or unreadable (a download still running?); "
                         f"refusing to serve an unidentified model")
    if args.model == "dronecats":
        model = DroneCATSModel(weights, attn=args.attn, device=args.device,
                               precision=args.precision)
    elif args.model == "ntutvla":
        model = NTUTVLAModel(weights, device=args.device)
        if abs(model.horizon_s - family.horizon_s) > 1e-9:
            raise SystemExit(f"--horizon-s {family.horizon_s:g}: the checkpoint's label horizon "
                             f"is {model.horizon_s:g} s (fly with --step-as-rate "
                             f"{model.horizon_s:g})")
    else:
        model = RaceVLAModel(weights, precision=args.precision, attn=args.attn,
                             device=args.device)
        if (model.vocab_size, model.n_bins) != (family.vocab_size, family.n_bins):
            raise SystemExit(f"the model's action vocabulary ({model.vocab_size}, "
                             f"{model.n_bins} bins) is not OpenVLA's ({family.vocab_size}, "
                             f"{family.n_bins}): the token reading would be wrong")
    cfg = {**family.describe(), "history_gap_s": args.history_gap_s}
    if getattr(model, "checkpoint", None) is not None:
        cfg["checkpoint"] = model.checkpoint
    if getattr(model, "norm_stats", None) is not None:
        ms = (model.norm_stats.get(family.unnorm_key) or {}).get("action") or {}
        cfg["model_norm_stats_agree"] = (
            [float(v) for v in ms.get("q01", [])] == family.stats["q01"]
            and [float(v) for v in ms.get("q99", [])] == family.stats["q99"])
    ident = foreign_identity(args.model, name=_name_of(args.model, weights), folder=weights.name,
                             revision=S.local_hf_revision(weights), digest=digest,
                             precision=args.precision, attn=args.attn,
                             front_source=args.front_source, facts=cfg)
    facts = {"load": model.timing, "digest": dinfo, "weight_files": [p.as_posix() for p in paths],
             "libraries": S.library_versions(), "protocol": C.PROTOCOL_VERSION,
             "server": "demo/vla_server_foreign.py"}
    try:
        from guardrail.manifest import collect_host_evidence
        facts["host"] = collect_host_evidence()
    except Exception:                                              # noqa: BLE001
        facts["host"] = None
    return model, ident, facts


def _fake_outputs(family: str, texts: list[str]) -> list:
    if family == "dronecats":
        return list(texts)
    out = []
    for t in texts:
        try:
            out.append(json.loads(t))
        except ValueError:
            raise SystemExit(f"--fake-native for racevla: {t!r} is not a JSON list of 7 action "
                             f"token ids") from None
    return out


def build_arg_parser() -> tuple[argparse.ArgumentParser, argparse.ArgumentParser]:
    """(the parser, its serve subparser: the one main's refusals speak for)."""
    ap = argparse.ArgumentParser(description="foreign drone VLAs behind VLA server protocol v2")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("serve", help="serve one foreign model over TCP")
    p.add_argument("--model", choices=FAMILIES, required=True)
    p.add_argument("--weights", required=True, metavar="DIR",
                   help="the download: DroneCATS's repo folder, RaceVLA's run folder")
    p.add_argument("--port", type=int, required=True,
                   help="required: AerialVLA's server owns 8790 by default")
    p.add_argument("--host", default=C.DEFAULT_HOST)
    p.add_argument("--log", default=None, help="JSONL of every request and answer (default "
                   "demo/out/vla_server_foreign/<model>_<utc>.jsonl)")
    p.add_argument("--precision", choices=("bf16", "nf4", "int4", "fp16"), default=None,
                   help="bf16 (both), nf4 (bitsandbytes 4-bit) or int4 (torchao int4 "
                        "weight-only, DroneCATS only); quantised DroneCATS keeps its vision "
                        "encoder, projector and lm_head in bf16; ntutvla is fp16 only "
                        "(default bf16, ntutvla fp16)")
    p.add_argument("--attn", choices=S.ATTNS, default="sdpa")
    p.add_argument("--device", default="cuda:0")
    p.add_argument("--front-source", choices=tuple(C.FRONT_SOURCES), default=C.DEFAULT_FRONT_SOURCE,
                   help="the camera the flight sends (vla_fly --vla-front-source): its field of "
                        "view and pitch are the intrinsics' (default vla: VLAFront)")
    p.add_argument("--command", default=None,
                   help="the command template, {target} = 'the <object words>' (default "
                        f"DroneCATS {DRONECATS_COMMAND!r}, RaceVLA {RACEVLA_INSTRUCTION!r}; "
                        "RaceVLA also takes a fixed instruction)")
    p.add_argument("--go-executor", choices=tuple(EXECUTORS), default="paper",
                   help="DroneCATS: how a go becomes a step (default the paper's)")
    p.add_argument("--max-width", type=int, default=640)
    p.add_argument("--max-new-tokens", type=int, default=128)
    p.add_argument("--plan-max-new-tokens", type=int, default=1024)
    p.add_argument("--history-gap-s", type=float, default=HISTORY_GAP_S)
    p.add_argument("--unnorm-key", default=None,
                   help="RaceVLA: the dataset_statistics.json key (default its only one)")
    p.add_argument("--horizon-s", type=float, default=1.0,
                   help="RaceVLA: the velocities' step horizon (fly with --step-as-rate H); "
                        "ntutvla: must be its checkpoint's label horizon (0.5)")
    p.add_argument("--keep-color", action="store_true",
                   help="RaceVLA: send the frame in colour (default grey, as its T265)")
    p.add_argument("--digest-sidecar", default=str(S.SIDECAR_DEFAULT))
    p.add_argument("--rehash", action="store_true")
    p.add_argument("--fake-native", nargs="+", default=None,
                   help="serve a scripted stand-in of these native outputs (DroneCATS JSON "
                        "texts; RaceVLA JSON lists of 7 token ids): identity fake:, no GPU")
    p.add_argument("--fake-delay-s", type=float, default=0.0)
    return ap, p


def main(argv=None) -> int:
    ap, p = build_arg_parser()
    args = ap.parse_args(argv)
    if args.precision is None:
        args.precision = "fp16" if args.model == "ntutvla" else "bf16"
    if (args.precision == "fp16") != (args.model == "ntutvla"):
        p.error(f"--precision {args.precision}: ntutvla runs fp16 only, and fp16 is ntutvla's only")
    if args.fake_native and args.model == "ntutvla":
        p.error("--fake-native: no scripted stand-in for ntutvla (use vla_server_v2 --fake-output)")
    if args.precision == "int4" and args.model != "dronecats":
        p.error("--precision int4 (torchao) is DroneCATS's only: RaceVLA's OpenVLA loader "
                "takes bf16 or nf4")
    need = PRECISION_PACKAGE.get(args.precision)
    available = bitsandbytes_available() if need == "bitsandbytes" else (
        need is None or package_available(need))
    if need and not args.fake_native and not available:
        p.error(f"--precision {args.precision} needs {need} in this Python environment (the "
                f"PC's dronecats env has neither; on the Jetson use --precision int4: its "
                f"PyPI bitsandbytes wheel has no sm_87 kernels)")
    for k in ("horizon_s", "history_gap_s"):
        v = getattr(args, k)
        if not (math.isfinite(v) and v > 0):
            p.error(f"--{k.replace('_', '-')} must be a positive number of seconds")
    if args.command is not None:
        try:
            command_text("a car", args.command, allow_fixed=args.model == "racevla")
        except ValueError as exc:
            p.error(str(exc))
    weights = Path(args.weights)
    if not weights.is_dir():
        p.error(f"--weights {weights}: no such folder")
    try:
        family = build_family(args, weights)
    except (OSError, ValueError) as exc:
        p.error(f"the {args.model} download at {weights}: {exc}")
    if args.fake_native:
        model = ScriptedForeignModel(_fake_outputs(args.model, args.fake_native),
                                     args.fake_delay_s)
        ident, facts = stand_in_identity(args.model), {"fake": True,
                                                       "config": family.describe()}
    else:
        model, ident, facts = build_real(args, family, weights)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    log = args.log or str(ROOT / "demo" / "out" / "vla_server_foreign" /
                          f"{args.model}_{stamp}.jsonl")
    core = ForeignServer(family, model, ident, log, hello_extra=facts,
                         front_source=args.front_source, history_gap_s=args.history_gap_s)
    core.probe_log()
    srv = make_tcp_server(core, args.host, args.port)
    if args.host not in ("127.0.0.1", "localhost", "::1"):
        print(f"[vla-foreign] WARNING: listening on {args.host}, not loopback", flush=True)
    print(f"[vla-foreign] protocol v{C.PROTOCOL_VERSION} on {args.host}:{args.port} model "
          f"{ident['id']} ({ident['precision']}/{ident['attn']}), front {args.front_source} "
          f"({core.camera['camera']}), mapping {MAPPING_RULES[args.model]}, log {log}",
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        print(f"[vla-foreign] stopped; {core.counts}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
