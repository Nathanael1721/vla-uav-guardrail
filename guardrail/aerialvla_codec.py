"""
AerialVLA's input and output, byte for byte: the prompt, the compass
vocabulary, the image mosaic, a strict reader for what the model says, and
the wire contract of the VLA server (protocol v2).

WHY THIS MODULE EXISTS (2026-10-21)
-----------------------------------
Every AerialVLA caller in this repository read the model's text with

    ints = re.findall(r"\\d+", tail); bins = ints[-3:]

(demo/aerialvla_demo.py, experiments/ablate_image_vs_hint.py, and upstream's
own src/model_wrapper/aerialvla_wrapper_ui.py:_parse_action_from_text, which
ours copied). That reads a MALFORMED landing output as motion: the text
'00 49 494 49' gives the last three integers (49, 494, 49), the 494 is clamped
to bin 98, and the action becomes forward 2.5 m, down +5.0 m - a dive - out of
an output whose first three bins are the stop triple 00 49 49. In the 4 Aug
ablation (docs/FINDING-what-drives-aerialvla.md) six of the 18 no-hint outputs
were of that kind and were scored as motion; read strictly, the original
adapter stops on 17 of 18 no-hint frames, not 11 (the re-score is
docs/data/aerialvla_ablation_rescore.json, made by
experiments/rescore_ablation.py with `parse_action` below).

So the reader here accepts exactly the shape the model was trained to emit
and nothing else, and every other piece of AerialVLA's interface that a flight
depends on lives beside it, so the server (demo/vla_server_v2.py), the client
(guardrail/vla_link.py) and the offline tools agree on one definition.

No torch, no transformers, and no Pillow at module level: the Shield's import
path and the CI jobs (which install neither) import this file. Pillow is
imported inside the image functions only, which run in the VLA environment
(vla-real); tests/test_env_pins.py lists it as source-tree-only for this
module.

THE UPSTREAM FACTS THIS ENCODES (each cited)
--------------------------------------------
Sources: the upstream repository XuPeng23/AerialVLA (copy of the training-era
code in source/AeroVLA/; the dataset builder src/aerovla_dataset.py read from
GitHub on 2026-10-21) and the training labels on disk,
D:/models/aerialvla-lora/aerovla_train_dataset.json.

* Prompt (aerialvla_wrapper_ui.py:146-150), byte for byte:
      "<image>\\nFly {direction}and find the target. {object}\\nAction: "
  `{direction}` carries its own trailing space ("straight ahead "), so with
  the empty phrase the sentence reads "Fly and find the target.". The
  training text is f"{instr}\\n{action}{action_str}" with action = "Action: "
  (aerovla_dataset.py __getitem__), and OpenVLA's Llama tokenizer ends the
  prompt in the separate token '\u2581' exactly as the training text has it
  before the first digit (tokenizer probe 2026-10-21: prompt tail
  ['Action', ':', '\u2581'], continuation '0','0','\u2581','4','9',... ).
  `<image>` is plain text; Prismatic inserts the patches itself.
  The object description is `.strip()`ed upstream
  (aerialvla_wrapper_ui.py:93); `validate_object` refuses surrounding
  whitespace rather than silently stripping it, so the prompt that is logged
  is the prompt that ran.
* Direction vocabulary (aerialvla_wrapper_ui.py:46-76): seven bearing bands
  plus "" for "no hint". Upstream computes it from the GROUND-TRUTH target
  position (get_semantic_direction, closeloop_util.py:120). In the training
  set the empty phrase occurs exactly once per trajectory, always on the final
  frame with a zero label and LAND (7,611 of 410,450 samples): the model was
  taught that "Fly and find the target." means "you have arrived".
* Action space (aerovla_dataset.py): NUM_BINS = 99; forward 0..5 m, down
  -5..5 m (+ = down, NED), yaw -1.1..1.1 rad; `_quantize_action` clips then
  returns int(norm * (NUM_BINS - 1)) - truncation, not rounding; the label
  text is f"{fwd:02d} {down:02d} {yaw:02d}", plus " LAND" when the sample is
  the last or penultimate step of its trajectory, plus the EOS token "</s>".
* The numbers are a STEP, not a velocity
  (airsim_plugin/AirVLNSimulatorClientTool_AerialVLA.py:300-385, move_path):
  new heading = heading + yaw; if |yaw| < 0.25 rad, move fwd metres along the
  NEW heading and `down` metres vertically at CRUISE_SPEED 1.0 m/s (a move
  shorter than 1 m takes 1 s); otherwise rotate only and move to the new
  altitude; then hover 0.5 s. Our flight scripts read the same three numbers
  as m/s, m/s and rad/s and held them for seconds (vla-assets discovery
  report, 2026-10-21); the constants below are upstream's, for the step
  executor (guardrail/vla_link.py StepTracker).

UNITS, ONCE: fwd_m and down_m are metres per step, dpsi_rad radians per step.
Nothing in this module is a rate.

THE INVERSE OF TRUNCATION (bins -> step)
----------------------------------------
Truncation maps the half-open cell [lo + b*w, lo + (b+1)*w), w = (hi-lo)/98,
to bin b; bin 98 holds only hi itself (values are clipped first). The decoder
returns the cell's LOWER EDGE, b/98 * (hi - lo) + lo, which is upstream's own
dequantize (aerialvla_wrapper_ui.py:217-219) - kept for fidelity, so a step
decoded here is the step upstream flew. Two consequences worth knowing:
  * bin 49 decodes to exactly 0.0 on the down and yaw axes (49/98 = 0.5), and
    "00 49 49" to the zero step;
  * bin 48 on the down axis is the cell [-0.102, 0) m. The training labels of
    level flight carry tiny negative noise (e.g. down = -0.0030 m at
    Carla_Town01/00c9b2a9.../000015.png), which truncation puts in bin 48, and
    the model's most frequent hinted output in the 4 Aug ablation is
    '96 48 49'. Decoded at the lower edge that is a 0.102 m climb per step
    the label never contained. `bin_interval` gives the whole cell for anyone
    who wants the centre instead; the step executor's altitude band absorbs
    either.
"""
from __future__ import annotations

import base64
import binascii
import bisect
import hashlib
import io
import math
import re
from dataclasses import asdict, dataclass
from typing import Any

# --------------------------------------------------------------------------- #
# The action space (upstream src/aerovla_dataset.py)
# --------------------------------------------------------------------------- #
NUM_BINS = 99
MAX_BIN = NUM_BINS - 1                          # 98: two digits, 00..98
AXES = ("fwd_m", "down_m", "dpsi_rad")
RANGES: dict[str, tuple[float, float]] = {
    "fwd_m": (0.0, 5.0),        # forward displacement along the NEW heading
    "down_m": (-5.0, 5.0),      # vertical displacement, + = down (NED)
    "dpsi_rad": (-1.1, 1.1),    # heading change, + = clockwise seen from above
}

# The upstream step executor's constants (AirVLNSimulatorClientTool_AerialVLA.py
# move_path, lines 340-347): a step whose |dpsi| reaches YAW_ONLY_THRESHOLD_RAD
# only rotates (and changes altitude); a smaller one also translates, at
# CRUISE_SPEED_MPS, and a move shorter than MICRO_MOVE_THRESHOLD_M takes 1 s.
# Below STEP_EPSILON an axis counts as "no move". Every step ends with a
# HOVER_AFTER_STEP_S hover before the next image is taken.
YAW_ONLY_THRESHOLD_RAD = 0.25
CRUISE_SPEED_MPS = 1.0
MICRO_MOVE_THRESHOLD_M = 1.0
STEP_EPSILON = 0.01
HOVER_AFTER_STEP_S = 0.5

# --------------------------------------------------------------------------- #
# The prompt
# --------------------------------------------------------------------------- #
PROMPT_TEMPLATE = "<image>\nFly {direction}and find the target. {object}\nAction: "

# Verbatim upstream strings, trailing space included. demo/vla_bridge.py and
# demo/aerialvla_demo.semantic_direction carry the same seven; the codec's test
# sweeps semantic_direction over the full circle and compares sets, so the
# three cannot drift apart silently.
STRAIGHT_AHEAD = "straight ahead "
FORWARD_RIGHT = "forward-right "
FORWARD_LEFT = "forward-left "
RIGHT = "to your right "
LEFT = "to your left "
RIGHT_REAR = "to your right rear "
LEFT_REAR = "to your left rear "
NO_HINT = ""
DIRECTION_VOCABULARY: tuple[str, ...] = (
    STRAIGHT_AHEAD, FORWARD_RIGHT, FORWARD_LEFT, RIGHT, LEFT, RIGHT_REAR, LEFT_REAR)
# The eight strings protocol v2 accepts in `phrase`: the seven hints and "".
PHRASES: tuple[str, ...] = DIRECTION_VOCABULARY + (NO_HINT,)

# Where a phrase came from. "goal": computed from a commanded goal position
# by the pilot side (the trained task: a GOTO). "detector": computed from an
# OWL-ViT box bearing (demo/vla_bridge.VLABridge). "none": no hint at all.
# None of them is language: AerialVLA's object slot is inert (FINDING doc).
HINT_SOURCES = ("goal", "detector", "none")

# CSP arms (WP2 prefix A/B, tools/prefix_eval.py ARMS). "off" is the upstream
# prompt byte for byte; "csp" and "neutral" put `csp_text` in the SAME place -
# the arm name only says which text it is (the Constraint Summary Pack, or a
# rule-free filler of about its length, the null).
CSP_MODES = ("off", "csp", "neutral")

MAX_OBJECT_CHARS = 2000     # upstream descriptions run to ~400 characters
MAX_CSP_CHARS = 4000        # the CSP budget is 256 tokens (compiler.py)

_CONTROL = re.compile(r"[\x00-\x1f\x7f]")

# COORDINATES IN FREE TEXT (review of V1a, P10). validate_request refuses a
# coordinate by KEY, but `object` (and `csp_text`) are free text, so
# object="target at x=38.0 y=26.0" went through to the prompt - the one way
# left to tell the VLA where the target is over protocol v2. These patterns
# refuse what a position looks like in text. Measured against every one of
# the 7,611 distinct object descriptions in upstream's training set
# (D:/models/aerialvla-lora/aerovla_train_dataset.json, 2026-10-22): none
# matches; 39 of them carry a number ("24-hour", "speed limit of 30", a
# lot marked "38", "Live Your Life in 2021"), none a decimal number, and 52
# use the word "target" in prose ("the target, a man,") - so a lone number,
# or the word alone, is NOT refused. It is a heuristic, not a proof: a
# position spelt in words ("thirty-eight metres east") passes. The server
# logs every prompt it ran (the JSONL log's "prompt"), so a leak that gets
# through can be audited.
_TARGET_NUMBER = re.compile(            # "target at 38", "goal: (38, 26)", "destination = -4"
    r"\b(?:target|goal|destination)s?\b\W{0,4}(?:(?:is|at|of|=|:)\W{0,4})?[-+]?\d", re.I)
_COORDINATE_TEXT = (
    ("two decimal numbers", re.compile(r"[-+]?\d+\.\d+(?:\D+[-+]?\d+\.\d+)")),
    ("a bracketed number pair",
     re.compile(r"[(\[]\s*[-+]?\d+(?:\.\d+)?\s*[,;]\s*[-+]?\d+(?:\.\d+)?\s*[)\]]")),
    ("an axis given a number",            # "x=38", "y: 26", "north=12"
     re.compile(r"\b(?:x|y|z|xy|north(?:ing)?|east(?:ing)?)\s*[=:]\s*[-+]?\d", re.I)),
    ("a latitude or longitude",
     re.compile(r"\b(?:lat|lon|lng|latitude|longitude)\b\W{0,3}[-+]?\d", re.I)),
    ("coordinates given a number", re.compile(r"\bcoord(?:inate)?s?\b\W{0,4}[-+(\[]?\d", re.I)),
    ("the target given a number", _TARGET_NUMBER),
)


def coordinate_text(text: str, rules=_COORDINATE_TEXT) -> str | None:
    """What in `text` looks like a position ("two decimal numbers", ...), or
    None. `object` is checked against every rule; `csp_text` only against
    the target/goal rule, because a Constraint Summary Pack legitimately
    carries altitudes, radii and speeds - but its relevance reasons can say
    "the straight path from (sx, sy) to the target (tx, ty)"
    (guardrail/templates/reason/*.j2), which must not reach the VLA."""
    for why, rx in rules:
        m = rx.search(text)
        if m:
            return f"{why} ({m.group(0)!r})"
    return None


class CodecError(ValueError):
    """An input the AerialVLA interface does not accept. `code` is the short
    machine-readable reason (the protocol's error prefix)."""

    def __init__(self, code: str, detail: str):
        super().__init__(f"{code}: {detail}")
        self.code = code
        self.detail = detail


def validate_phrase(phrase: Any) -> str:
    """`phrase` must be one of the eight strings, exactly - no stripping, no
    case folding. A phrase the LoRA never saw is worth nothing, and
    "straight ahead" without its space is a different string."""
    if not isinstance(phrase, str):
        raise CodecError(ERR_BAD_PHRASE, f"phrase must be a string, got {type(phrase).__name__}")
    if phrase not in PHRASES:
        raise CodecError(ERR_BAD_PHRASE, f"{phrase!r} is not one of the 8 vocabulary strings "
                                         f"{list(PHRASES)!r}")
    return phrase


def validate_hint(phrase: str, hint_source: Any) -> str:
    """The phrase and its source must agree.

    * hint_source "none" carries the empty phrase and nothing else: a phrase
      with no source is a hint nobody can account for.
    * "detector" never carries the empty phrase: the bridge emits no phrase
      when the detector has nothing (demo/vla_bridge.py, "No detection: no
      prompt") and the caller must not run an inference; "" from a detector
      would ask the model to LAND wherever the box was lost.
    * "goal" may carry "": upstream returns "" when the goal is reached
      (get_semantic_direction, dist_xy < 0.01), which is the trained "you have
      arrived".
    """
    if hint_source not in HINT_SOURCES:
        raise CodecError(ERR_BAD_HINT, f"hint_source {hint_source!r} is not one of {list(HINT_SOURCES)}")
    if hint_source == "none" and phrase != NO_HINT:
        raise CodecError(ERR_BAD_HINT, f"hint_source 'none' with phrase {phrase!r}: a hint with no source")
    if hint_source == "detector" and phrase == NO_HINT:
        raise CodecError(ERR_BAD_HINT, "hint_source 'detector' with the empty phrase: no detection "
                                       "means no inference (demo/vla_bridge.py), not a LAND prompt")
    return hint_source


def validate_object(obj: Any) -> str:
    """The `{object}` slot: free text, inserted verbatim. Refused: a non-string,
    control characters (a newline would move "Action: " to another line of
    the prompt), surrounding whitespace (upstream strips it; we refuse rather
    than strip, so the logged prompt is the one that ran), runaway length,
    and text that looks like a position (coordinate_text; its own error,
    ERR_COORDINATE_TEXT)."""
    if not isinstance(obj, str):
        raise CodecError(ERR_BAD_OBJECT, f"object must be a string, got {type(obj).__name__}")
    if _CONTROL.search(obj):
        raise CodecError(ERR_BAD_OBJECT, "object contains a control character (newline, tab, ...)")
    if obj != obj.strip():
        raise CodecError(ERR_BAD_OBJECT, "object starts or ends with whitespace (upstream strips it)")
    if len(obj) > MAX_OBJECT_CHARS:
        raise CodecError(ERR_BAD_OBJECT, f"object is {len(obj)} characters, more than {MAX_OBJECT_CHARS}")
    why = coordinate_text(obj)
    if why:
        raise CodecError(ERR_COORDINATE_TEXT, f"object carries {why}: the VLA is never told "
                                              f"where the target is; describe it in words")
    return obj


def validate_csp(csp_mode: Any, csp_text: Any) -> str | None:
    """`csp_mode` "off" requires `csp_text` null; "csp" and "neutral" require
    a non-empty single-line text. Returns the text as it will be placed
    (stripped of surrounding whitespace) or None.

    An empty text in an "on" arm is refused, as demo/real_vla_demo.py
    build_openvla_prompt refuses it: an A/B whose two arms read the same
    prompt would report "no effect" for a flag that never did anything."""
    if csp_mode not in CSP_MODES:
        raise CodecError(ERR_BAD_CSP, f"csp_mode {csp_mode!r} is not one of {list(CSP_MODES)}")
    if csp_mode == "off":
        if csp_text is not None:
            raise CodecError(ERR_BAD_CSP, "csp_mode 'off' with a csp_text: the off arm is the "
                                          "upstream prompt byte for byte")
        return None
    if not isinstance(csp_text, str):
        raise CodecError(ERR_BAD_CSP, f"csp_mode {csp_mode!r} needs csp_text as a string")
    text = csp_text.strip()
    if not text:
        raise CodecError(ERR_BAD_CSP, f"csp_mode {csp_mode!r} with an empty csp_text: both arms "
                                      f"would read the same prompt")
    if _CONTROL.search(text):
        raise CodecError(ERR_BAD_CSP, "csp_text contains a control character (newline, tab, ...): "
                                      "the CSP stays on the instruction line")
    if len(text) > MAX_CSP_CHARS:
        raise CodecError(ERR_BAD_CSP, f"csp_text is {len(text)} characters, more than {MAX_CSP_CHARS}")
    why = coordinate_text(text, rules=(("the target given a number", _TARGET_NUMBER),))
    if why:
        raise CodecError(ERR_COORDINATE_TEXT, f"csp_text carries {why}: send the rule "
                                              f"sentences, not the reasons that place the target")
    return text


def build_prompt(phrase: str, obj: str, csp_text: str | None = None,
                 csp_mode: str = "off") -> str:
    """The ONE place an AerialVLA prompt is formed.

    csp_mode "off" (the default): PROMPT_TEMPLATE with the phrase and object
    substituted, byte for byte upstream's prompt.

    csp_mode "csp" / "neutral": the CSP text goes on the instruction line,
    after "<image>\\n" and before "Fly", joined by ONE space:

        "<image>\\n" + csp + " " + "Fly {direction}and find the target. {object}\\nAction: "

    That is where demo/real_vla_demo.py puts it for OpenVLA ("In: {csp} What
    action ..."): inside the instruction, ahead of the task sentence, so the
    prompt still ends in "Action: " - the place the model was trained to start
    its digits. The model never saw rule text in training, so the arm is a
    measurement, not an expected effect; the prompt's SHA-256 is returned to
    the client with every action, so the exact placement is on record.

    Differs from the legacy demo/aerialvla_demo.build_prompt in one case only:
    that function normalises whitespace, so with an EMPTY object it ends the
    sentence "target.\\nAction: " where the template (and this function) give
    "target. \\nAction: ". The 4 Aug ablation's "(none)" object arm ran the
    legacy form; every phrase and every non-empty object gives identical bytes
    (tests/test_aerialvla_codec.py)."""
    validate_phrase(phrase)
    validate_object(obj)
    text = validate_csp(csp_mode, csp_text)
    body = PROMPT_TEMPLATE.format(direction=phrase, object=obj)
    if text is None:
        return body
    head = "<image>\n"
    assert body.startswith(head)
    return head + text + " " + body[len(head):]


def prompt_sha256(prompt: str) -> str:
    """SHA-256 of the prompt's UTF-8 bytes (64 hex)."""
    return hashlib.sha256(prompt.encode("utf-8")).hexdigest()


# --------------------------------------------------------------------------- #
# The image mosaic (aerialvla_wrapper_ui.py:135-143)
# --------------------------------------------------------------------------- #
MOSAIC_TILE = (224, 224)            # (width, height) of each view
MOSAIC_SIZE = (224, 448)            # front on top, down below
MISSING_DOWN_POLICIES = ("black", "refuse")


def _bicubic():
    from PIL import Image
    return getattr(Image, "Resampling", Image).BICUBIC


def build_mosaic(front, down, missing_down: str = "black"):
    """(front, down) PIL images -> (224x448 RGB mosaic, info dict).

    Each view is converted to RGB and resized bicubic to 224x224 (aspect not
    kept: upstream's cameras are square 256x256, ours are 16:9, and upstream
    squashes whatever it is given), front pasted at the top, down at the
    bottom, on a black canvas - upstream's own construction. The Prismatic
    processor then squashes the 224x448 mosaic to 224x224 ("resize-naive");
    that happens in the model's processor, not here.

    A MISSING DOWN FRAME (`down is None`) is handled explicitly, never
    silently: with missing_down="black" the lower tile is left as the black
    canvas upstream pastes onto, and info records down_missing=True and
    down_fill="black" - an input the model never saw in training (every
    upstream sample has a down view), so it is a labelled distribution shift,
    not a neutral default. missing_down="refuse" raises CodecError instead.
    The ArduPilot rail's robot config removed DownCamera
    (demo/pas_ardupilot/robot_citylife_ardupilot.jsonc), which is why the
    case exists at all."""
    from PIL import Image
    if missing_down not in MISSING_DOWN_POLICIES:
        raise ValueError(f"missing_down {missing_down!r} is not one of {MISSING_DOWN_POLICIES}")
    if front is None:
        raise CodecError(ERR_BAD_IMAGE, "no front frame")
    info: dict[str, Any] = {
        "size": list(MOSAIC_SIZE), "tile": list(MOSAIC_TILE), "resample": "bicubic",
        "layout": "front over down", "front_src_size": list(front.size),
        "down_src_size": None if down is None else list(down.size),
        "down_missing": down is None, "down_fill": None}
    if down is None and missing_down == "refuse":
        raise CodecError(ERR_BAD_IMAGE, "no down frame, and missing_down='refuse'")
    rs = _bicubic()
    mosaic = Image.new("RGB", MOSAIC_SIZE, (0, 0, 0))
    mosaic.paste(front.convert("RGB").resize(MOSAIC_TILE, resample=rs), (0, 0))
    if down is not None:
        mosaic.paste(down.convert("RGB").resize(MOSAIC_TILE, resample=rs), (0, MOSAIC_TILE[1]))
    else:
        info["down_fill"] = "black"
    return mosaic, info


def mosaic_sha256(mosaic) -> str:
    """SHA-256 of the mosaic's raw RGB8 bytes, row-major, 224*448*3 bytes - the
    pixels the processor is given. Not of a PNG or JPEG encoding, which
    changes with the encoder's version and settings while the pixels do not."""
    if mosaic.mode != "RGB" or tuple(mosaic.size) != MOSAIC_SIZE:
        raise ValueError(f"not a {MOSAIC_SIZE} RGB mosaic: {mosaic.mode} {mosaic.size}")
    return hashlib.sha256(mosaic.tobytes()).hexdigest()


# --------------------------------------------------------------------------- #
# The mosaic's lower tile: the down camera, or the front frame from dt earlier
# (2026-10-24)
# --------------------------------------------------------------------------- #
# The first fine-tuned follow (demo/out/vla_follow_ft_01_20261024, adapter
# follow_v2_r1/epoch4) held at a red light ~9 m behind the car - right - and
# went on answering fwd 0.0 for ~6 s while the car pulled away from 9.8 to
# 27.6 m: one image cannot tell a STOPPED car from one PULLING AWAY. The
# expert knows the car's speed from its box history; the model sees one
# frame, and the mosaic's lower tile is the DOWN camera (the road under the
# aircraft), close to useless for a car ahead. BOTTOM_TILE_PREV_FRONT puts
# the FRONT frame from dt_s earlier in that tile instead, so one input
# carries the scene's motion. Upstream trained on "down" only: a prev_front
# model is one of our fine-tunes, trained on a prev_front dataset
# (tools/build_follow_dataset.py --bottom-tile prev_front), flown with the
# same tile (demo/vla_fly.py --vla-bottom-tile prev_front), and refused
# otherwise (adapter_bottom_tile, bottom_tile_mismatch; demo/vla_mission.
# bottom_tile_check).
#
# THE RULE, one for both sides (pick_prev_front): the lower tile is the front
# frame whose capture time is nearest t_capture - dt_s, within tol_s; with
# none there is no sample (the builder's drop "no_prev_front") and no request
# (VLAFrames counts no_prev_front) - never the down frame, never black.
# tol_s < dt_s, so the frame is always strictly earlier than the capture.
# The times are capture times on the capture's own clock - captures.jsonl's
# t_capture in the builder, VLAFrames' capture time of each frame in its
# ring in flight - and the recorder builds captures.jsonl with VLAFrames
# itself, so the two are one rule. The request carries the earlier front
# JPEG in its down slot: the wire protocol is unchanged, and the server
# cannot tell (the mosaic is front over "lower"); the records say which.
BOTTOM_TILE_DOWN = "down"
BOTTOM_TILE_PREV_FRONT = "prev_front"
BOTTOM_TILE_MODES = (BOTTOM_TILE_DOWN, BOTTOM_TILE_PREV_FRONT)
DEFAULT_PREV_DT_S = 1.0
DEFAULT_PREV_TOL_S = 0.25


@dataclass(frozen=True)
class BottomTile:
    """What the mosaic's lower tile is: "down" (upstream's; no dt), or
    "prev_front" with `dt_s` (how much earlier) and `tol_s` (how far the
    frame may sit from t_capture - dt_s). Validated: 0 < tol_s < dt_s."""
    mode: str = BOTTOM_TILE_DOWN
    dt_s: float | None = None
    tol_s: float | None = None

    def __post_init__(self):
        if self.mode not in BOTTOM_TILE_MODES:
            raise ValueError(f"lower tile {self.mode!r} is not one of {list(BOTTOM_TILE_MODES)}")
        if self.mode == BOTTOM_TILE_DOWN:
            if self.dt_s is not None or self.tol_s is not None:
                raise ValueError("a 'down' lower tile has no dt_s / tol_s")
            return
        for k in ("dt_s", "tol_s"):
            v = getattr(self, k)
            if not _finite_number(v) or float(v) <= 0:
                raise ValueError(f"prev_front {k} must be a positive number of seconds, got {v!r}")
        if not float(self.tol_s) < float(self.dt_s):
            raise ValueError(f"prev_front tol_s {self.tol_s:g} must be below dt_s {self.dt_s:g}: "
                             f"else the 'earlier' frame could be the capture's own")

    @property
    def prev_front(self) -> bool:
        return self.mode == BOTTOM_TILE_PREV_FRONT

    def as_dict(self) -> dict:
        return {"mode": self.mode, "dt_s": self.dt_s, "tol_s": self.tol_s}


def bottom_tile(mode: str = BOTTOM_TILE_DOWN, dt_s: float | None = None,
                tol_s: float | None = None) -> BottomTile:
    """The tile of `mode`; for prev_front, dt_s and tol_s default to
    DEFAULT_PREV_DT_S / DEFAULT_PREV_TOL_S, for down they are ignored.
    ValueError for anything BottomTile refuses."""
    if mode == BOTTOM_TILE_PREV_FRONT:
        return BottomTile(mode, float(DEFAULT_PREV_DT_S if dt_s is None else dt_s),
                          float(DEFAULT_PREV_TOL_S if tol_s is None else tol_s))
    return BottomTile(mode)


def bottom_tile_from(rec: Any) -> BottomTile:
    """A recorded tile (a sample's or stats.json's `bottom_tile`, a
    provenance file's) -> BottomTile. None, {} or a record without a mode is
    "down": every dataset and adapter from before 2026-10-24, and upstream's.
    A prev_front record must carry its dt_s (tol_s defaults); extra keys
    (dt_actual_s, prev_i, ...) are ignored. ValueError otherwise."""
    if rec is None:
        return BottomTile()
    if not isinstance(rec, dict):
        raise ValueError(f"a lower tile record is an object, got {type(rec).__name__}")
    mode = rec.get("mode")
    if mode is None or mode == BOTTOM_TILE_DOWN:
        return BottomTile()
    if mode != BOTTOM_TILE_PREV_FRONT:
        raise ValueError(f"lower tile {mode!r} is not one of {list(BOTTOM_TILE_MODES)}")
    if rec.get("dt_s") is None:
        raise ValueError("a prev_front lower tile record carries no dt_s")
    return bottom_tile(mode, rec["dt_s"], rec.get("tol_s"))


def pick_prev_front(t_capture: float, times, dt_s: float, tol_s: float) -> int | None:
    """THE RULE (above): the index into `times` (capture times, ascending) of
    the frame nearest t_capture - dt_s, if it is within tol_s of it and
    strictly earlier than t_capture; else None. 1e-9 s of slack on the
    tolerance for clock floats; a tie takes either neighbour."""
    target = float(t_capture) - float(dt_s)
    i = bisect.bisect_left(times, target)
    best, err = None, math.inf
    for j in (i - 1, i):
        if 0 <= j < len(times):
            e = abs(float(times[j]) - target)
            if e < err:
                best, err = j, e
    if best is None or err > float(tol_s) + 1e-9 or not float(times[best]) < float(t_capture):
        return None
    return best


def adapter_bottom_tile(identity: Any) -> tuple[BottomTile, str]:
    """(the lower tile an identity's adapter was trained on, "announced") from
    the provenance block hello carries (demo/vla_server_v2.adapter_identity:
    `adapter.provenance.bottom_tile`, which training/finetune_aerialvla_
    follow.py copied from the dataset's stats.json); (BottomTile("down"),
    "absent") when there is none - upstream's adapter, a stand-in, a
    fine-tune from before 2026-10-24: "down" is the only tile those saw.
    ValueError for a field that is there and malformed."""
    a = identity.get("adapter") if isinstance(identity, dict) else None
    prov = a.get("provenance") if isinstance(a, dict) else None
    if isinstance(prov, dict) and prov.get("bottom_tile") is not None:
        return bottom_tile_from(prov["bottom_tile"]), "announced"
    return BottomTile(), "absent"


def _tile_words(t: BottomTile) -> str:
    return (f"'prev_front' (the front frame {t.dt_s:g} s earlier)" if t.prev_front
            else "'down' (the down camera)")


def bottom_tile_mismatch(trained: BottomTile, given: BottomTile) -> str | None:
    """Why a model trained with lower tile `trained` must not be given
    `given`, or None when they agree. The mode and, for prev_front, dt_s
    must match (to the microsecond); tol_s is sampling slack, not part of
    the input, and may differ."""
    if trained.mode == given.mode and (
            not trained.prev_front or abs(float(trained.dt_s) - float(given.dt_s)) <= 1e-6):
        return None
    return (f"the adapter was trained with the lower tile {_tile_words(trained)} and would be "
            f"given {_tile_words(given)}: an input it never saw")


# The most pixels a wire frame may claim. The frames this protocol carries are
# 400x225 (FrontCamera), 768x432 or 1280x720 at most, and upstream's are
# 256x256; 4096x4096 leaves room for any camera here and stays far below
# Pillow's own decompression-bomb limit (Image.MAX_IMAGE_PIXELS, ~89 M, with
# DecompressionBombError at twice that). Checked on the header, before a
# single pixel is decoded: review of V1a (P1) sent a ~390 KB PNG claiming
# 20000x20000 pixels, which Image.open refused with DecompressionBombError -
# not an OSError, so it escaped this function and dropped the connection.
MAX_FRAME_PIXELS = 4096 * 4096


def decode_image_b64(data: Any, what: str = "image"):
    """Base64 text of an encoded image (JPEG on the wire; any format Pillow
    reads) -> RGB PIL image. Strict base64 (no stray characters); a decode
    failure is a CodecError, never a black frame.

    The bytes come from the network, so ANY exception Pillow raises while
    opening or decoding them is a bad image: its plugins raise
    UnidentifiedImageError, OSError and ValueError, but also
    DecompressionBombError (a plain Exception subclass), SyntaxError, EOFError
    and struct.error on malformed files. Only MemoryError passes through: the
    host ran out, the input is not proven bad, and the server answers it as
    an internal error. A frame claiming more than MAX_FRAME_PIXELS is refused
    from its header."""
    from PIL import Image
    if not isinstance(data, str) or not data:
        raise CodecError(ERR_BAD_IMAGE, f"{what}: expected non-empty base64 text")
    try:
        raw = base64.b64decode(data, validate=True)
    except (binascii.Error, ValueError) as exc:
        raise CodecError(ERR_BAD_IMAGE, f"{what}: not base64 ({exc})") from None
    try:
        img = Image.open(io.BytesIO(raw))           # reads the header only
        w, h = img.size
        if w * h > MAX_FRAME_PIXELS:
            raise CodecError(ERR_BAD_IMAGE, f"{what}: {w}x{h} pixels, more than "
                                            f"{MAX_FRAME_PIXELS} (4096x4096)")
        img.load()
        return img.convert("RGB")
    except (CodecError, MemoryError):
        raise
    except Exception as exc:                                      # noqa: BLE001
        raise CodecError(ERR_BAD_IMAGE, f"{what}: not a readable image "
                                        f"({type(exc).__name__}: {exc})") from None


def encode_jpeg_b64(img, quality: int = 95) -> str:
    """RGB PIL image -> base64 JPEG text, the wire form of a frame."""
    buf = io.BytesIO()
    img.convert("RGB").save(buf, format="JPEG", quality=quality)
    return base64.b64encode(buf.getvalue()).decode("ascii")


# --------------------------------------------------------------------------- #
# The model's text -> a step (strict)
# --------------------------------------------------------------------------- #
EOS_TEXT = "</s>"
LAND_TEXT = " LAND"
# Exactly three two-digit groups of ASCII digits separated by single spaces,
# then optionally " LAND". [0-9], not \d: \d also matches other scripts'
# digits, and the 4 Aug ablation holds an output with a CJK character inside a
# bin ('9\u8fd0 48 60').
_ACTION_RE = re.compile(r"([0-9]{2}) ([0-9]{2}) ([0-9]{2})( LAND)?")


@dataclass(frozen=True)
class Step:
    """One AerialVLA decision, in upstream units: metres and radians PER STEP."""
    fwd_m: float
    down_m: float
    dpsi_rad: float

    @property
    def rotate_only(self) -> bool:
        """Upstream rotates (and changes altitude) without translating when
        |dpsi| >= 0.25 rad (move_path: `if abs(pred_yaw) < YAW_THRESHOLD`)."""
        return abs(self.dpsi_rad) >= YAW_ONLY_THRESHOLD_RAD

    @property
    def is_zero(self) -> bool:
        """Every axis below upstream's 0.01 "no move" epsilon. Upstream's
        evaluator also ENDS the episode on such a step (has_land_num,
        aerialvla_wrapper_ui.py:195); here that is reported, not decided -
        `land` is the LAND token alone."""
        return (abs(self.fwd_m) < STEP_EPSILON and abs(self.down_m) < STEP_EPSILON
                and abs(self.dpsi_rad) < STEP_EPSILON)

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class ParsedAction:
    """What `parse_action` read. parse_ok False => bins None, step None,
    land False: the client HOLDS ZERO and counts the failure."""
    parse_ok: bool
    text: str
    bins: tuple[int, int, int] | None
    step: Step | None
    land: bool
    eos: bool
    reason: str | None

    def to_dict(self) -> dict[str, Any]:
        return {"parse_ok": self.parse_ok, "text": self.text,
                "bins": list(self.bins) if self.bins is not None else None,
                "step": self.step.to_dict() if self.step is not None else None,
                "land": self.land, "eos": self.eos, "reason": self.reason}


def _fail(text: str, reason: str, eos: bool = False) -> ParsedAction:
    return ParsedAction(False, text, None, None, False, eos, reason)


def parse_action(text: Any, require_eos: bool = False) -> ParsedAction:
    """Read the model's GENERATED text - the continuation after the prompt,
    decoded from the new tokens only - and accept exactly

        "FF DD YY"  or  "FF DD YY LAND"

    each of FF, DD, YY two ASCII digits 00..98, single spaces, then the end of
    the text, or the EOS marker "</s>" and then the end. Nothing before, after
    or between: no leading space, no fourth group, no three-digit bin, no "L"
    without "AND", no bin 99 (there are 99 bins, 00..98). Anything else is
    parse_ok=False with the reason; the caller holds zero.

    require_eos=True also refuses a text without the EOS marker: decoded with
    special tokens, a text that stops without "</s>" ran out of
    max_new_tokens rather than ending. The server decodes that way and sets it.

    Decoding new tokens only matters: `decode(full).split("Action:")[-1]`
    keeps the prompt's own trailing space (' 00 49 49</s>'), which this reader
    refuses; `generated_from_decoded` strips exactly that space for texts that
    were cut that way (the 4 Aug ablation's failure rows)."""
    if not isinstance(text, str):
        return _fail(str(text), f"not text ({type(text).__name__})")
    body, eos = text, False
    if body.endswith(EOS_TEXT):
        body, eos = body[: -len(EOS_TEXT)], True
    if require_eos and not eos:
        return _fail(text, "no EOS marker: generation stopped at max_new_tokens", eos)
    m = _ACTION_RE.fullmatch(body)
    if m is None:
        return _fail(text, _why_malformed(body), eos)
    bins = (int(m.group(1)), int(m.group(2)), int(m.group(3)))
    over = [b for b in bins if b > MAX_BIN]
    if over:
        return _fail(text, f"bin {over[0]} is outside 00..{MAX_BIN:02d}", eos)
    return ParsedAction(True, text, bins, bins_to_step(bins), m.group(4) is not None, eos, None)


def _why_malformed(body: str) -> str:
    if not body:
        return "empty"
    if not body.strip():
        return "only whitespace"
    if body != body.strip():
        return "leading or trailing whitespace"
    groups = body.split(" ")
    digits = [g for g in groups if g.isascii() and g.isdigit()]
    if any(not (g.isascii() and g.isdigit()) and g != "LAND" for g in groups):
        return f"non-digit token in {body!r}"
    if any(len(g) != 2 for g in digits):
        return f"a bin is not two digits in {body!r}"
    if len(digits) != 3:
        return f"{len(digits)} bins, not 3, in {body!r}"
    return f"malformed: {body!r}"


def generated_from_decoded(decoded: str) -> str:
    """The generated continuation from a decode of prompt + output, cut the
    legacy way (`.split("Action:")[-1]`): the prompt's own trailing space
    after "Action:" is removed - exactly one space, never more, so a model
    that itself emitted extra spaces is still caught by `parse_action`."""
    tail = decoded.split("Action:")[-1]
    return tail[1:] if tail.startswith(" ") else tail


def dequantize(b: int, axis: str) -> float:
    """Bin -> value at the LOWER EDGE of the bin's cell: b/98*(hi-lo)+lo,
    upstream's own decoder (module docstring, "The inverse of truncation")."""
    if not isinstance(b, int) or isinstance(b, bool) or not 0 <= b <= MAX_BIN:
        raise ValueError(f"bin {b!r} is not an integer in 0..{MAX_BIN}")
    lo, hi = RANGES[axis]
    return b / MAX_BIN * (hi - lo) + lo


def quantize(value: float, axis: str) -> int:
    """Upstream's `_quantize_action`: clip to the axis range, then
    int(norm * 98) - truncation toward the lower edge."""
    lo, hi = RANGES[axis]
    v = min(max(float(value), lo), hi)
    return int((v - lo) / (hi - lo) * MAX_BIN)


def step_bins(fwd_m: float, down_m: float, dpsi_rad: float) -> tuple[int, int, int]:
    """A step in metres and radians -> its (forward, down, yaw) bins by
    upstream's labelling rule (`quantize`: clip to the axis, then truncate).
    The inverse direction of `bins_to_step`, for training labels
    (tools/build_follow_dataset.py): a label made here and read back with
    `parse_action(action_text(bins) + EOS_TEXT)` gives the same bins."""
    return (quantize(fwd_m, "fwd_m"), quantize(down_m, "down_m"),
            quantize(dpsi_rad, "dpsi_rad"))


def action_text(bins, land: bool = False) -> str:
    """(forward, down, yaw) bins -> the text the model is trained to emit:
    upstream's label f"{fwd:02d} {down:02d} {yaw:02d}" (aerovla_dataset.py
    __getitem__), zero-padded, plus " LAND" when `land`. The EOS token is
    NOT part of it - the trainer appends the tokenizer's own end of sequence,
    as upstream's collator does - so `parse_action(action_text(b) +
    EOS_TEXT, require_eos=True)` is the server's reading of a training label.

    THE ONE PLACE the training text is formed (wave FT, 2026-10-23): the old
    training/finetune_aerialvla.py wrote unpadded bins from round() ("4 49
    49" where upstream writes "04 49 49"), a text the strict reader refuses.
    Refused here: anything but three integers in 0..98 (bools and floats
    included - 1.0 would format as "1.0")."""
    if not isinstance(bins, (tuple, list)) or len(bins) != 3:
        raise ValueError(f"bins must be three integers, got {bins!r}")
    for b in bins:
        if not isinstance(b, int) or isinstance(b, bool) or not 0 <= b <= MAX_BIN:
            raise ValueError(f"bin {b!r} is not an integer in 0..{MAX_BIN}")
    f, d, y = bins
    return f"{f:02d} {d:02d} {y:02d}" + (LAND_TEXT if land else "")


def bin_interval(b: int, axis: str) -> tuple[float, float]:
    """The values truncation maps to bin b: [lo_edge, hi_edge) - except bin 98,
    which only the axis maximum itself reaches (returned as (hi, hi))."""
    lo, hi = RANGES[axis]
    w = (hi - lo) / MAX_BIN
    if b == MAX_BIN:
        return (hi, hi)
    edge = dequantize(b, axis)
    return (edge, edge + w)


def bins_to_step(bins) -> Step:
    """(forward, down, yaw) bins -> Step in metres and radians per step."""
    f, d, y = bins
    return Step(fwd_m=dequantize(f, "fwd_m"), down_m=dequantize(d, "down_m"),
                dpsi_rad=dequantize(y, "dpsi_rad"))


def legacy_last_three_ints(text: str) -> dict[str, Any]:
    """THE DEFECTIVE READING, kept only so its effect can be measured: the
    last three integers anywhere in the text, each clamped to 0..98, LAND if
    the text contains "LAND" or the decoded step is all-near-zero - upstream's
    _parse_action_from_text and run(), and our copies of it until 2026-10-21.
    Never use it to fly."""
    ints = re.findall(r"\d+", text)
    if len(ints) < 3:
        return {"ok": False, "bins": None, "step": None, "land": False}
    raw = tuple(int(i) for i in ints[-3:])
    clamped = tuple(max(0, min(MAX_BIN, b)) for b in raw)
    step = bins_to_step(clamped)
    land = ("LAND" in text) or step.is_zero
    return {"ok": True, "bins": list(raw), "step": step.to_dict(), "land": land}


# --------------------------------------------------------------------------- #
# The wire contract: VLA server protocol v2
# --------------------------------------------------------------------------- #
# TCP on 127.0.0.1, newline-delimited UTF-8 JSON, one request -> one response,
# many requests per connection (brief BRIEF-V1, "VLA server protocol v2"). The
# server (demo/vla_server_v2.py) validates every request with
# `validate_request`; a client may call it before sending to fail locally.
PROTOCOL_VERSION = 2
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8790
OPS = ("hello", "infer", "shutdown")
HELLO_FIELDS = frozenset({"v", "op"})
SHUTDOWN_FIELDS = frozenset({"v", "op"})
INFER_FIELDS = frozenset({"v", "op", "seq", "t_capture", "pose", "front_jpg_b64",
                          "down_jpg_b64", "phrase", "hint_source", "object", "csp_text",
                          "csp_mode"})
POSE_FIELDS = frozenset({"x", "y", "up", "yaw_rad"})
# Keys that could carry where the TARGET is. Refused by name, with their own
# error, before the unknown-field check: the coordinate leak is the defect
# every earlier "semantic" run had (FINDING doc, Problem 1), and the legacy
# file protocol carried exactly such a key ("target": [x, y], vla_server.py).
# Any key containing "target" or "goal" is refused the same way.
COORDINATE_FIELD_NAMES = frozenset({
    "target", "goal", "object_position", "xy", "lat", "lon", "latitude", "longitude",
    "tgt", "target_xy", "goal_xy", "object_xy", "object_pos", "target_pos", "goal_pos"})
MAX_LINE_BYTES = 16 * 1024 * 1024   # one request line: two JPEGs in base64, with room

# Error codes: a response's `error` is "<code>: <detail>"; match on the code
# (error.split(":", 1)[0]).
ERR_BAD_JSON = "bad json"
ERR_NOT_OBJECT = "bad request"
ERR_VERSION = "unsupported version"
ERR_OP = "unknown op"
ERR_UNKNOWN_FIELD = "unknown field"
ERR_COORDINATE_FIELD = "coordinate field refused"
ERR_COORDINATE_TEXT = "coordinate text refused"     # a position inside object / csp_text
ERR_MISSING_FIELD = "missing field"
ERR_BAD_VALUE = "bad value"
ERR_BAD_PHRASE = "phrase not in vocabulary"
ERR_BAD_HINT = "bad hint_source"
ERR_BAD_OBJECT = "bad object"
ERR_BAD_CSP = "bad csp"
ERR_BAD_IMAGE = "bad image"
ERR_LINE_TOO_LONG = "line too long"
ERR_INFERENCE = "inference failed"
# Anything else that went wrong on the server while answering a request it
# read (a frame decoder that raised, a host out of memory): the client still
# gets an ok=false answer with every infer key, and the connection stays open
# (demo/vla_server_v2.V2Server.handle_request; review of V1a, P1).
ERR_INTERNAL = "internal error"


class ProtocolError(CodecError):
    """A request protocol v2 refuses."""


# --------------------------------------------------------------------------- #
# Which adapter answered: the original, or one of our fine-tunes (2026-10-23)
# --------------------------------------------------------------------------- #
# The server's model id names its adapter: "aerialvla-lora/aero_vla@<rev>"
# for upstream's own LoRA (D:/models/aerialvla-lora/aero_vla), and since the
# follow fine-tune (training/finetune_aerialvla_follow.py)
# "aerialvla-ft-follow/<run>/<epochN>@<first 8 hex of the adapter's sha256>"
# for a checkpoint that trainer wrote: it leaves ADAPTER_PROVENANCE_FILE in
# every checkpoint directory, and the server builds the id from that file,
# never from the folder's name alone (demo/vla_server_v2.adapter_identity).
# Every reader that decides "was this the VLA" (tools/todo.py) or "which
# weights flew" (demo/vla_mission.model_id_for) reads the variant from here,
# so the three cannot disagree about a prefix. A variant is a LABEL on the
# id; the weights themselves are pinned by weights_sha256 (base shards +
# adapter directory), which changes with any fine-tune.
ADAPTER_PROVENANCE_FILE = "aerialvla_ft_provenance.json"
ORIGINAL_ADAPTER_ID = "aerialvla-lora/aero_vla"
FINE_TUNE_ID_ROOTS = ("aerialvla-ft-follow",)
ADAPTER_VARIANTS = ("original", "fine-tune")


def adapter_variant(model_id: Any) -> str | None:
    """"original" for upstream's adapter id, "fine-tune" for an id under one
    of FINE_TUNE_ID_ROOTS (by prefix, "<root>/"), None for anything else - a
    fake: stand-in, a bare base, another adapter. Reads the server's id
    ("aerialvla-ft-follow/r1/epoch2@1a2b3c4d") and the pilot side's model id
    ("aerialvla:openvla-7b+aerialvla-ft-follow/r1/epoch2@sha256:..."), whose
    adapter part follows the "+"."""
    if not isinstance(model_id, str) or not model_id.strip():
        return None
    s = model_id.strip()
    if s.lower().startswith("fake:"):
        return None
    ident = s.split("@", 1)[0]
    if ident.startswith("aerialvla:") and "+" in ident:
        ident = ident.split("+", 1)[1]
    if ident == ORIGINAL_ADAPTER_ID or ident == "aero_vla":
        return "original"
    if any(ident.startswith(root + "/") and len(ident) > len(root) + 1
           for root in FINE_TUNE_ID_ROOTS):
        return "fine-tune"
    return None


def _is_coordinate_name(key: str) -> bool:
    k = key.lower()
    return k in COORDINATE_FIELD_NAMES or "target" in k or "goal" in k


def _finite_number(v: Any) -> bool:
    """A JSON number that is a finite float. JSON integers have no size
    limit, and float() of one with more than ~309 digits raises
    OverflowError: until the review of V1a that exception came out of the
    validator itself instead of a refusal."""
    if not isinstance(v, (int, float)) or isinstance(v, bool):
        return False
    try:
        return math.isfinite(float(v))
    except OverflowError:
        return False


def validate_request(req: Any) -> dict[str, Any]:
    """Check one decoded request against protocol v2; return it unchanged or
    raise ProtocolError. Order: object -> coordinate-named keys (top level
    and inside pose) -> version -> op -> unknown keys -> missing keys ->
    types -> phrase, hint source, object, CSP. Images are only checked to be
    non-empty text (or null for the down frame) here; the server decodes them.
    """
    if not isinstance(req, dict):
        raise ProtocolError(ERR_NOT_OBJECT, f"a request is a JSON object, got {type(req).__name__}")
    coords = sorted(k for k in req if isinstance(k, str) and _is_coordinate_name(k))
    pose = req.get("pose")
    if isinstance(pose, dict):
        coords += sorted(f"pose.{k}" for k in pose if isinstance(k, str) and _is_coordinate_name(k))
    if coords:
        raise ProtocolError(ERR_COORDINATE_FIELD,
                            f"{', '.join(coords)}: the VLA is never told where the target is; "
                            f"send a compass phrase")
    if type(req.get("v")) is not int or req["v"] != PROTOCOL_VERSION:
        raise ProtocolError(ERR_VERSION, f"v={req.get('v')!r}; this server speaks v={PROTOCOL_VERSION}")
    op = req.get("op")
    if op not in OPS:
        raise ProtocolError(ERR_OP, f"{op!r} is not one of {list(OPS)}")
    allowed = {"hello": HELLO_FIELDS, "shutdown": SHUTDOWN_FIELDS, "infer": INFER_FIELDS}[op]
    unknown = sorted(str(k) for k in req if k not in allowed)
    if unknown:
        raise ProtocolError(ERR_UNKNOWN_FIELD, ", ".join(unknown))
    missing = sorted(allowed - set(req))
    if missing:
        raise ProtocolError(ERR_MISSING_FIELD, ", ".join(missing))
    if op != "infer":
        return req
    seq = req["seq"]
    if not isinstance(seq, int) or isinstance(seq, bool):
        raise ProtocolError(ERR_BAD_VALUE, f"seq must be an integer, got {seq!r}")
    if not _finite_number(req["t_capture"]):
        raise ProtocolError(ERR_BAD_VALUE, f"t_capture must be a finite number of seconds, "
                                           f"got {req['t_capture']!r}")
    if not isinstance(pose, dict):
        raise ProtocolError(ERR_BAD_VALUE, f"pose must be an object {sorted(POSE_FIELDS)}")
    extra = sorted(str(k) for k in pose if k not in POSE_FIELDS)
    if extra:
        raise ProtocolError(ERR_UNKNOWN_FIELD, ", ".join(f"pose.{k}" for k in extra))
    lacking = sorted(POSE_FIELDS - set(pose))
    if lacking:
        raise ProtocolError(ERR_MISSING_FIELD, ", ".join(f"pose.{k}" for k in lacking))
    bad = sorted(k for k in POSE_FIELDS if not _finite_number(pose[k]))
    if bad:
        raise ProtocolError(ERR_BAD_VALUE, f"pose.{bad[0]} must be a finite number")
    if not isinstance(req["front_jpg_b64"], str) or not req["front_jpg_b64"]:
        raise ProtocolError(ERR_BAD_IMAGE, "front_jpg_b64 must be non-empty base64 text")
    down = req["down_jpg_b64"]
    if down is not None and (not isinstance(down, str) or not down):
        raise ProtocolError(ERR_BAD_IMAGE, "down_jpg_b64 must be non-empty base64 text or null")
    try:
        validate_phrase(req["phrase"])
        validate_hint(req["phrase"], req["hint_source"])
        validate_object(req["object"])
        validate_csp(req["csp_mode"], req["csp_text"])
    except ProtocolError:
        raise
    except CodecError as exc:
        raise ProtocolError(exc.code, exc.detail) from None
    return req
