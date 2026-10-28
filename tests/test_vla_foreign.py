"""
demo/vla_server_foreign.py - foreign drone VLAs (DroneCATS, RaceVLA) behind
protocol v2, and the allow-list that records them as themselves.

Run either way:
    pytest tests/test_vla_foreign.py -v
    python tests/test_vla_foreign.py

No test here loads a model, touches a GPU or needs Pillow: the models are
scripted stand-ins that return NATIVE outputs (a DroneCATS JSON action, the
seven OpenVLA action tokens of RaceVLA), the frame decoder is a stand-in that
reads the image size, and the TCP tests run on an ephemeral loopback port
(never 8790). What is tested:

  1. The two mappings, native output -> AeroVLA's step text "FF DD YY" (or
     LAND), exactly: DroneCATS go (bearing, elevation, the paper's executor:
     3 m cap, x0.6 under 5 m, zero under 2 m, altitude held under 0.15 of
     the horizontal distance), rotate (+ = right = clockwise), think (the
     zero step), finished (LAND), malformed (parse_ok false, no step), a
     pitched camera; RaceVLA's tokens -> OpenVLA's bins -> un-normalised with
     a small fake dataset_statistics.json -> its drone code's gains -> a step
     over the horizon, the lateral velocity dropped and recorded; clipping to
     the codec's ranges, recorded, never silent.
  2. DroneCATS's own input: the prompt files' wording, camera_aux on every
     request, the image at most 640 px wide, the command from the object
     words, the history of the last five actions (README's line format),
     reset on hello and on a long gap, kept per connection.
  3. The protocol round trip: the real client (guardrail/vla_link.RemoteVLA,
     strict identity) against the foreign server over TCP, every reply in
     the v2 shape plus the native output, the mapping and the timing, in
     the reply, the server's JSONL log and the client's own record.
  4. Identity and the allow-list: hello names the model itself
     (dronecats/<name>@<sha8>, racevla/<name>@<sha8>), variant "foreign";
     vla_mission records it under its own pilot id, never aerialvla's;
     tools/todo.py classifies it a VLA of its own family; the lower tile
     "none" flies with the client's default down tile and refuses
     prev_front; speed words off; the front camera the server's intrinsics
     assume must be the flight's (--vla-front-source).
  5. The bench tool's latency and native-output histogram.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import vla_server_foreign as F                                       # noqa: E402
import vla_mission as VM                                             # noqa: E402
from guardrail import aerialvla_codec as C                           # noqa: E402
from guardrail.vla_link import (Capture, Hint, Pose, RemoteVLA,      # noqa: E402
                                build_infer_fields, check_identity, make_record)

SKIP = "SKIP"
V2_KEYS = {"v", "seq", "ok", "error", "raw_text", "bins", "step", "land", "parse_ok",
           "latency_s", "t_capture", "pose", "prompt_sha256", "image_sha256", "model"}
FOREIGN_KEYS = V2_KEYS | {"native", "mapping", "timing"}
DIGEST = "1a2b3c4d" + "5e6f7a8b" * 7
SYSTEM_TPL = ("You see ONE image. The image you receive is {frame_w}x{frame_h} px; a quarter "
              "is ({quarter_x}, {quarter_y}). {{\"action\":\"go\"}}")
USER_TPL = ("Command: {instruction}\n\n## Your last 5 actions (use them to avoid circling / "
            "repeating a mistake)\n{history}\n\nOutput ONLY the single JSON object. No text "
            "outside it.")
VLA_INTR = F.intrinsics(256, 256, 90.0)


class Img:
    """Stand-in for a decoded PIL image: only its size is read."""
    def __init__(self, w, h):
        self.size = (int(w), int(h))


def _decode(req):
    """The 'JPEG' is the text WxH (or anything: 256x256)."""
    raw = req["front_jpg_b64"]
    if raw == "BAD":
        raise C.CodecError(C.ERR_BAD_IMAGE, "front_jpg_b64: not a readable image")
    try:
        import base64
        txt = base64.b64decode(raw).decode("ascii")
        w, h = (int(v) for v in txt.split("x"))
        return Img(w, h)
    except Exception:                                              # noqa: BLE001
        return Img(256, 256)


def _hash(img):
    return hashlib.sha256(repr(img.size).encode()).hexdigest()


def _b64(text):
    import base64
    return base64.b64encode(text.encode("ascii")).decode("ascii")


def _req(**over):
    r = {"v": 2, "op": "infer", "seq": 7, "t_capture": 100.0,
         "pose": {"x": 1.0, "y": -2.0, "up": 12.0, "yaw_rad": 0.5},
         "front_jpg_b64": _b64("256x256"), "down_jpg_b64": _b64("256x256"),
         "phrase": "straight ahead ", "hint_source": "detector", "object": "a red car",
         "csp_text": None, "csp_mode": "off"}
    r.update(over)
    return r


def _dronecats(**kw):
    kw.setdefault("system_prompt", SYSTEM_TPL)
    kw.setdefault("user_prompt", USER_TPL)
    return F.DroneCATSFamily(**kw)


def _stats_file(d: Path) -> Path:
    """A small dataset_statistics.json in RaceVLA's shape: one key, q01/q99,
    the gripper masked off (as drone6_all's)."""
    p = d / "dataset_statistics.json"
    p.write_text(json.dumps({"fake_set": {"action": {
        "q01": [-1.0, -1.0, -1.0, 0.0, 0.0, -2.0, 0.0],
        "q99": [1.0, 1.0, 1.0, 0.0, 0.0, 2.0, 1.0],
        "mask": [True, True, True, True, True, True, False]}}}), encoding="utf-8")
    return p


def _racevla(tmp: Path, **kw):
    key, stats = F.load_action_stats(_stats_file(tmp))
    kw.setdefault("gray", False)            # the stand-in image has no pixels to grey
    return F.RaceVLAFamily(stats=stats, unnorm_key=key, **kw)


def _identity(family="dronecats", source="vla", digest=DIGEST):
    name = {"dronecats": "DroneCATS-Cosmos3-Edge-2B", "racevla": "drone6_all"}[family]
    return F.foreign_identity(family, name=name, folder=name, revision=None, digest=digest,
                              precision="bf16", attn="sdpa", front_source=source,
                              facts={"test": True})


def _core(family, outputs, ident=None, log=None, **kw):
    model = F.ScriptedForeignModel(list(outputs))
    core = F.ForeignServer(family, model, ident or _identity(family.name), log,
                           decode_front=_decode, hash_fn=_hash, **kw)
    return core, model


def _tok(idx: int) -> int:
    """The OpenVLA action token whose bin index (vocab - token - 1) is idx."""
    return F.OPENVLA_VOCAB_SIZE - (idx + 1)


# ====================================================== 1. the DroneCATS mapping

def _dc_text(action: dict, intr=VLA_INTR, pitch_deg=0.0, executor="paper") -> str:
    fam = _dronecats(executor=executor)
    m = F.map_dronecats(action, intr, pitch_deg=pitch_deg, executor=fam.executor)
    return F.encode_step(m["fwd_m"], m["down_m"], m["dpsi_rad"], land=m["land"])["text"]


def test_a_centred_go_flies_the_papers_capped_step_straight_ahead():
    # 256x256 at 90 deg: fx = 128, k = fx/W = 0.5; canonical 20 -> 10 m along
    # the axis; the paper's executor caps one step at 3 m: bin round(3/5*98)=59
    assert VLA_INTR["fx"] == 128.0 and VLA_INTR["k"] == 0.5
    assert _dc_text({"action": "go", "point": [500, 500], "depth_m": 20.0}) == "59 49 49</s>"
    # the direct executor flies the whole horizontal distance, clipped to 5 m
    assert _dc_text({"action": "go", "point": [500, 500], "depth_m": 20.0},
                    executor="direct") == "98 49 49</s>"


def test_a_go_to_the_right_turns_clockwise_by_the_points_bearing():
    # x=750 -> px 192, 0.5 D to the right of 10 m ahead: bearing atan2(5, 10)
    fam = _dronecats()
    m = F.map_dronecats({"action": "go", "point": [750, 500], "depth_m": 20.0}, VLA_INTR,
                        executor=fam.executor)
    assert abs(m["dpsi_rad"] - math.atan2(5.0, 10.0)) < 1e-9 and m["dpsi_rad"] > 0
    assert abs(m["fwd_m"] - 3.0) < 1e-9, m
    assert F.encode_step(m["fwd_m"], m["down_m"], m["dpsi_rad"])["text"] == "59 49 70</s>"
    left = F.map_dronecats({"action": "go", "point": [250, 500], "depth_m": 20.0}, VLA_INTR,
                           executor=fam.executor)
    assert abs(left["dpsi_rad"] + math.atan2(5.0, 10.0)) < 1e-9


def test_a_go_below_the_centre_descends_along_the_ray_and_small_ones_hold_altitude():
    # y=900 -> py 230.4, 0.8 D below the axis: 8 m down at 10 m out, a 12.8 m
    # ray scaled to the 3 m cap: fwd 2.343, down 1.874
    assert _dc_text({"action": "go", "point": [500, 900], "depth_m": 20.0}) == "46 67 49</s>"
    # y=550: 1 m down at 10 m, under 0.15 of the horizontal: altitude held
    m = F.map_dronecats({"action": "go", "point": [500, 550], "depth_m": 20.0}, VLA_INTR,
                        executor=_dronecats().executor)
    assert m["down_m"] == 0.0 and m["geometry"]["altitude_held"] is True, m
    assert _dc_text({"action": "go", "point": [500, 550], "depth_m": 20.0}) == "59 49 49</s>"


def test_a_pitched_camera_is_rotated_to_the_level_frame():
    # FrontCamera, 640x360 (768x432 downscaled), pitched -20 deg: the image
    # centre is 20 deg below the horizon - 9.40 m out, 3.42 m down at 10 m
    intr = F.intrinsics(640, 360, 90.0)
    assert intr["fx"] == 320.0 and intr["cy"] == 180.0
    m = F.map_dronecats({"action": "go", "point": [500, 500], "depth_m": 20.0}, intr,
                        pitch_deg=-20.0, executor=_dronecats().executor)
    g = m["geometry"]
    assert abs(g["forward_m"] - 10 * math.cos(math.radians(20))) < 1e-9
    assert abs(g["down_m"] - 10 * math.sin(math.radians(20))) < 1e-9
    assert _dc_text({"action": "go", "point": [500, 500], "depth_m": 20.0}, intr,
                    pitch_deg=-20.0) == "55 59 49</s>"


def test_the_papers_depth_bands_slow_under_5_m_and_stop_under_2_m():
    assert _dc_text({"action": "go", "point": [500, 500], "depth_m": 6.0}) == "35 49 49</s>"
    m = F.map_dronecats({"action": "go", "point": [500, 500], "depth_m": 3.0}, VLA_INTR,
                        executor=_dronecats().executor)
    assert m["geometry"]["band_scale"] == 0.0 and m["fwd_m"] == 0.0
    assert _dc_text({"action": "go", "point": [500, 500], "depth_m": 3.0}) == "00 49 49</s>"


def test_rotate_is_a_heading_change_only_clamped_to_90_then_to_the_codec():
    assert _dc_text({"action": "rotate", "angle_deg": 30}) == "00 49 72</s>"
    st = F.encode_step(0.0, 0.0, math.radians(-90))
    assert st["text"] == "00 49 00</s>" and st["clipped"] == ["dpsi_rad"], st
    nat = F.parse_dronecats('{"action":"rotate","angle_deg":120}')
    assert nat["ok"] and nat["action"]["angle_deg"] == 90.0 and nat["clamped"] == ["angle_deg"]
    assert _dc_text(nat["action"]) == "00 49 98</s>"


def test_finished_is_land_and_think_is_the_zero_step():
    assert _dc_text({"action": "finished"}) == "00 49 49 LAND</s>"
    assert _dc_text({"action": "think"}) == "00 49 49</s>"
    p = C.parse_action(_dc_text({"action": "finished"}), require_eos=True)
    assert p.parse_ok and p.land and p.step.is_zero


def test_malformed_dronecats_outputs_are_refused_never_read_as_motion():
    for text in ("go forward", "", '{"action":"fly"}', '[1, 2]',
                 '{"action":"go","point":[500],"depth_m":3}',
                 '{"action":"go","point":[500,1200],"depth_m":3}',
                 '{"action":"go","point":[500,500]}',
                 '{"action":"go","point":[500,500],"depth_m":-1}',
                 '{"action":"go","point":[500,500],"depth_m":NaN}',
                 '{"action":"go","point":[true,500],"depth_m":3}',
                 '{"action":"rotate"}', '{"action":"rotate","angle_deg":"left"}',
                 '{"action":"go","point":[500,500],"depth_m":3} trailing'):
        nat = F.parse_dronecats(text)
        assert nat["ok"] is False and nat["action"] is None and nat["reason"], (text, nat)
    ok = F.parse_dronecats('\n {"action":"go","point":[402,656],"depth_m":3.2} \n')
    assert ok["ok"] and ok["action"] == {"action": "go", "point": [402, 656], "depth_m": 3.2}
    plan = F.parse_dronecats(json.dumps({"target_in_view": "yes", "target_match": "match",
                                         "closing_in": "yes", "current_frame": "a red car",
                                         "action_planning": "go on", "action": "think"}))
    assert plan["ok"] and plan["action"] == {"action": "think"}
    assert plan["plan"]["target_in_view"] == "yes", "the plan fields are recorded"


# ====================================================== 1b. the RaceVLA mapping

def test_openvla_tokens_decode_to_the_bin_centres_predict_action_uses():
    c = F.bin_centers(256)
    assert len(c) == 255 and abs(c[127]) < 1e-12 and abs(c[254] - 254.5 * 2 / 255 + 1) < 1e-12
    n = F.tokens_to_normalized([_tok(127), _tok(254), _tok(0), F.OPENVLA_VOCAB_SIZE + 5,
                                0, _tok(140), _tok(191)])
    assert abs(n[0]) < 1e-12 and abs(n[1] - c[254]) < 1e-12 and abs(n[2] - c[0]) < 1e-12
    # out of range ids are clipped as predict_action clips them
    assert n[3] == c[0] and n[4] == c[254]
    assert abs(n[5] - c[140]) < 1e-12 and abs(n[6] - c[191]) < 1e-12


def test_unnormalisation_uses_q01_q99_and_leaves_masked_dimensions_normalised():
    with tempfile.TemporaryDirectory() as d:
        key, stats = F.load_action_stats(_stats_file(Path(d)))
    assert key == "fake_set"
    a = F.unnormalize([0.0, 1.0, -1.0, 0.5, 0.5, 0.5, 0.25], stats)
    assert a[0] == 0.0 and a[1] == 1.0 and a[2] == -1.0
    assert a[3] == 0.0 and a[5] == 1.0, "q01 -2 / q99 2 at +0.5 is 1.0"
    assert a[6] == 0.25, "the gripper is masked off: left normalised"


def test_the_unnorm_key_is_the_only_one_or_named_and_present():
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "dataset_statistics.json"
        one = {"action": {"q01": [0.0] * 7, "q99": [1.0] * 7}}
        p.write_text(json.dumps({"a": one, "b": one}), encoding="utf-8")
        for key in (None, "c"):
            try:
                F.load_action_stats(p, key)
                raise AssertionError(f"key {key!r} accepted among two")
            except ValueError as exc:
                assert "'a'" in str(exc) and "'b'" in str(exc), exc
        assert F.load_action_stats(p, "b")[0] == "b"


def test_racevla_velocities_become_a_step_over_the_horizon_with_its_gains():
    # drone.py: forward and left x 1.875, climb x 0.5, yaw rate x 1.5 (rad/s,
    # ENU: + = counter-clockwise); AeroVLA's dpsi is + clockwise
    with tempfile.TemporaryDirectory() as d:
        fam = _racevla(Path(d))
        half_fam = _racevla(Path(d), horizon_s=0.5)
    toks = [_tok(254), _tok(0), _tok(127), _tok(127), _tok(127), _tok(140), _tok(127)]
    nat = fam.parse({"raw": toks, "model_s": 0.1})
    assert nat["ok"] and len(nat["action"]["action"]) == 7
    m = fam.map(nat["action"], VLA_INTR)
    c = F.bin_centers(256)
    dyaw = 0.5 * (c[140] + 1) * 4 - 2
    assert abs(m["fwd_m"] - 1.875 * c[254]) < 1e-9
    assert abs(m["dpsi_rad"] + 1.5 * dyaw) < 1e-9 and m["dpsi_rad"] < 0
    assert abs(m["down_m"]) < 1e-12
    assert abs(m["geometry"]["lateral_left_m_dropped"] - 1.875 * c[0]) < 1e-9, m
    st = F.encode_step(m["fwd_m"], m["down_m"], m["dpsi_rad"])
    assert st["text"] == "37 49 35</s>", st
    # climbing is negative down; backwards cannot be flown and is recorded
    up = fam.map({"action": [c[0], 0, c[254], 0, 0, 0, 0]}, VLA_INTR)
    st = F.encode_step(up["fwd_m"], up["down_m"], up["dpsi_rad"])
    assert st["text"] == "00 44 49</s>" and "fwd_m" in st["clipped"], st
    assert st["requested"]["fwd_m"] < 0
    half = half_fam.map(nat["action"], VLA_INTR)
    assert abs(half["fwd_m"] - m["fwd_m"] / 2) < 1e-12


def test_a_racevla_answer_that_is_not_seven_tokens_is_malformed():
    with tempfile.TemporaryDirectory() as d:
        fam = _racevla(Path(d))
    for raw in ([1, 2, 3], "31872 31872", None, [1.5] * 7, [True] * 7):
        nat = fam.parse({"raw": raw, "model_s": 0.1})
        assert nat["ok"] is False and nat["reason"], (raw, nat)


def test_racevla_builds_openvlas_prompt_from_the_object_words():
    with tempfile.TemporaryDirectory() as d:
        fam = _racevla(Path(d))
        inp = fam.inputs(_req(object="a red car"), VLA_INTR, [])
        assert inp["prompt"] == "In: What action should the robot take to fly to the red car?\nOut:"
        fixed = _racevla(Path(d), instruction="Fly through the gates and avoid obstacles")
        assert fixed.inputs(_req(), VLA_INTR, [])["prompt"].endswith(
            "to Fly through the gates and avoid obstacles?\nOut:")


def test_the_step_encoding_is_the_nearest_bin_and_reads_back_through_the_strict_reader():
    for fwd, down, dpsi in ((0.0, 0.0, 0.0), (1.234, -0.7, 0.1), (5.0, 5.0, 1.1),
                            (0.02, -0.04, -0.011)):
        st = F.encode_step(fwd, down, dpsi)
        p = C.parse_action(st["text"], require_eos=True)
        assert p.parse_ok and tuple(st["bins"]) == p.bins, st
        for k, v in (("fwd_m", fwd), ("down_m", down), ("dpsi_rad", dpsi)):
            lo, hi = C.RANGES[k]
            assert abs(getattr(p.step, k) - v) <= 0.5 * (hi - lo) / 98 + 1e-9, (k, v, p.step)
    assert F.encode_step(0.0, 0.0, 0.0)["flown"] == {"fwd_m": 0.0, "down_m": 0.0, "dpsi_rad": 0.0}
    assert F.encode_step(9.0, -9.0, 0.0)["clipped"] == ["fwd_m", "down_m"]


# ============================================== 2. DroneCATS's own input

def test_the_camera_aux_and_the_image_size_are_the_cards():
    assert F.model_size(768, 432, 640) == (640, 360)
    assert F.model_size(256, 256, 640) == (256, 256)
    aux = F.camera_aux(F.intrinsics(640, 360, 90.0))
    assert aux == '{"width": 640, "height": 360, "fx": 320.0, "fy": 320.0}', aux
    # the card's Tello example: 56.4 deg, k = 0.932
    assert abs(F.intrinsics(640, 480, 56.4)["k"] - 0.9325) < 1e-3


def test_the_command_names_the_object_as_the_papers_tracking_wording_does():
    for obj, want in (("a red car", "go to the red car"), ("the red car", "go to the red car"),
                      ("red car", "go to the red car"), ("an orange barrier",
                                                         "go to the orange barrier"),
                      ("The target is on a street", "go to the target is on a street")):
        assert F.command_text(obj, F.DRONECATS_COMMAND) == want, obj
    assert F.command_text("a red car", "follow {target}") == "follow the red car"
    for bad in ("", "go to it"):
        try:
            F.command_text("a red car" if bad else "", bad or F.DRONECATS_COMMAND)
            raise AssertionError(f"accepted {bad!r}")
        except (ValueError, C.CodecError):
            pass


def test_the_history_is_the_cards_lines_oldest_first_the_latest_in_progress():
    assert F.history_text([]) == F.FIRST_STEP_HISTORY
    assert F.FIRST_STEP_HISTORY == "(none yet \u2014 this is the first step)"
    h = F.history_text([{"action": {"action": "rotate", "angle_deg": -90}, "t": 10.0},
                        {"action": {"action": "go", "point": [402, 656], "depth_m": 3.2},
                         "t": 11.5}])
    assert h == ("  step 1 of 2: ROTATE angle=-90deg  [done, 1.5s]\n"
                 "  step 2 of 2: GO x=402 y=656 depth=3.2m  [in progress]  <- latest"), h
    assert F.history_line({"action": "think"}) == "THINK"
    assert F.history_line({"action": "finished"}) == "FINISHED"


def test_dronecats_messages_carry_the_prompt_files_wording_and_camera_aux():
    fam = _dronecats()
    intr = F.intrinsics(640, 360, 90.0)
    inp = fam.inputs(_req(object="a red car"), intr, [])
    sys_msg, user = inp["messages"]
    assert sys_msg["content"][0]["text"] == SYSTEM_TPL.format(
        frame_w=640, frame_h=360, quarter_x=160, quarter_y=90)
    text, image = user["content"]
    assert text["text"] == USER_TPL.format(instruction="go to the red car",
                                           history=F.FIRST_STEP_HISTORY)
    assert image["type"] == "image" and image["camera_aux"] == F.camera_aux(intr)
    assert inp["command"] == "go to the red car" and inp["camera_aux"] == F.camera_aux(intr)


def test_the_real_prompt_files_are_read_verbatim_when_present():
    d = Path("D:/models/dronecats/DroneCATS-Cosmos3-Edge-2B")
    if not (d / "prompts" / "user_prompt.txt").is_file():
        print("SKIP: the DroneCATS download is not on this host")
        return SKIP
    fam = F.DroneCATSFamily.from_dir(d)
    assert fam.user_prompt == (d / "prompts" / "user_prompt.txt").read_text(encoding="utf-8")
    assert "{instruction}" in fam.user_prompt and "{history}" in fam.user_prompt
    assert fam.system_prompt.format(frame_w=640, frame_h=360, quarter_x=160, quarter_y=90)


def test_the_server_keeps_five_actions_resets_on_hello_and_on_a_gap():
    outs = [json.dumps({"action": "rotate", "angle_deg": 10 * k}) for k in range(1, 9)]
    core, model = _core(_dronecats(), outs, history_gap_s=5.0)
    for k in range(7):
        resp, _, _ = core.handle(_req(seq=k + 1, t_capture=100.0 + k))
        assert resp["ok"] and resp["parse_ok"], resp
    hist = model.calls[-1]["history"]
    assert hist.count("\n") == 4 and hist.startswith("  step 1 of 5: ROTATE angle=20deg"), hist
    assert hist.endswith("ROTATE angle=60deg  [in progress]  <- latest"), hist
    core.handle({"v": 2, "op": "hello"})
    core.handle(_req(seq=20, t_capture=110.0))
    assert model.calls[-1]["history"] == F.FIRST_STEP_HISTORY, "hello did not reset"
    core.handle(_req(seq=21, t_capture=120.0))         # 10 s later: a new episode
    assert model.calls[-1]["history"] == F.FIRST_STEP_HISTORY, "a long gap did not reset"
    resp, _, _ = core.handle(_req(seq=22, t_capture=119.0))   # time ran backwards
    assert model.calls[-1]["history"] == F.FIRST_STEP_HISTORY
    assert resp["native"]["history_reset"] == "t_capture went backwards", resp["native"]


def test_a_malformed_answer_holds_and_is_not_added_to_the_history():
    core, model = _core(_dronecats(), ["oops", '{"action":"think"}', '{"action":"think"}'])
    a, _, _ = core.handle(_req(seq=1, t_capture=1.0))
    assert a["ok"] and a["parse_ok"] is False and a["step"] is None and a["land"] is False
    assert a["raw_text"] == "" and a["native"]["text"] == "oops" and a["native"]["ok"] is False
    core.handle(_req(seq=2, t_capture=2.0))
    core.handle(_req(seq=3, t_capture=3.0))
    assert model.calls[2]["history"] == "  step 1 of 1: THINK  [in progress]  <- latest"
    assert core.counts["parse_fail"] == 1


def test_each_connection_keeps_its_own_history():
    core, model = _core(_dronecats(), ['{"action":"think"}'])
    core.handle(_req(seq=1, t_capture=1.0))
    seen = {}

    def other():
        core.handle(_req(seq=2, t_capture=1.5))
        seen["h"] = model.calls[-1]["history"]
    th = threading.Thread(target=other)
    th.start()
    th.join()
    assert seen["h"] == F.FIRST_STEP_HISTORY, "another connection read this one's history"
    core.handle(_req(seq=3, t_capture=2.0))
    assert model.calls[-1]["history"].startswith("  step 1 of 1: THINK")


def test_the_image_is_downscaled_to_640_wide_before_the_intrinsics():
    # the FrontCamera source (pitched -20 deg) is the identity's, so the
    # server's geometry is that camera's
    core, model = _core(_dronecats(), ['{"action":"go","point":[500,500],"depth_m":20}'],
                        _identity("dronecats", "detector"),
                        prepare=lambda img: (Img(640, 360), {"resized": True}))
    resp, _, _ = core.handle(_req(front_jpg_b64=_b64("768x432")))
    assert resp["native"]["camera_aux"] == '{"width": 640, "height": 360, "fx": 320.0, "fy": 320.0}'
    assert resp["raw_text"] == "55 59 49</s>", resp["raw_text"]
    assert resp["native"]["image"]["src_size"] == [768, 432]
    assert resp["native"]["image"]["model_size"] == [640, 360]


# ====================================================== 3. the protocol

def test_an_infer_answer_is_v2_plus_the_native_output_the_mapping_and_the_timing():
    core, model = _core(_dronecats(), ['{"action":"go","point":[750,500],"depth_m":20}'])
    resp, stop, extra = core.handle(_req())
    assert not stop and set(resp) == FOREIGN_KEYS, set(resp) ^ FOREIGN_KEYS
    assert resp["ok"] and resp["parse_ok"] and resp["raw_text"] == "59 49 70</s>"
    p = C.parse_action(resp["raw_text"], require_eos=True)
    assert resp["bins"] == list(p.bins) and resp["step"] == p.step.to_dict()
    nat, mp = resp["native"], resp["mapping"]
    assert nat["family"] == "dronecats" and nat["action"]["point"] == [750, 500]
    assert nat["text"] == '{"action":"go","point":[750,500],"depth_m":20}'
    assert nat["command"] == "go to the red car" and nat["ignored"]["phrase"] == "straight ahead "
    assert mp["rule"] == F.MAPPING_RULES["dronecats"] and mp["executor"]["step_cap_m"] == 3.0
    assert abs(mp["requested"]["dpsi_rad"] - math.atan2(5, 10)) < 1e-9
    assert mp["flown"] == resp["step"] and mp["quantize"].startswith("nearest bin")
    assert set(resp["timing"]) >= {"model_s", "server_s"} and resp["latency_s"] >= 0
    assert resp["model"]["id"].startswith("dronecats/DroneCATS-Cosmos3-Edge-2B@1a2b3c4d")
    assert resp["prompt_sha256"] == hashlib.sha256(
        model.calls[0]["prompt_text"].encode("utf-8")).hexdigest()
    assert resp["image_sha256"] == _hash(Img(256, 256))
    assert extra["native"] == nat and extra["mapping"] == mp


def test_refusals_keep_every_key_and_the_model_never_runs():
    core, model = _core(_dronecats(), ['{"action":"think"}'])
    for req, code in ((_req(goal=[1, 2]), C.ERR_COORDINATE_FIELD),
                      (_req(phrase="ahead"), C.ERR_BAD_PHRASE),
                      (_req(front_jpg_b64="BAD"), C.ERR_BAD_IMAGE),
                      (_req(object=""), C.ERR_BAD_OBJECT),
                      # neither prompt has a CSP slot: an arm would change nothing
                      (_req(csp_mode="csp", csp_text="stay below 40 m"), C.ERR_BAD_CSP)):
        resp, _, _ = core.handle(req)
        assert resp["ok"] is False and resp["error"].startswith(code), (code, resp)
        assert set(resp) == FOREIGN_KEYS and resp["native"] is None
    assert not model.calls
    resp, _ = core.handle_line(b"not json")
    assert resp["ok"] is False and set(resp) == FOREIGN_KEYS


def test_a_model_that_raises_answers_the_client():
    class Boom(F.ScriptedForeignModel):
        def generate(self, image, inputs):
            raise RuntimeError("CUDA out of memory")
    core = F.ForeignServer(_dronecats(), Boom(["x"]), _identity(), None,
                           decode_front=_decode, hash_fn=_hash)
    resp, _, _ = core.handle(_req())
    assert resp["ok"] is False and resp["error"].startswith(C.ERR_INFERENCE), resp
    assert core.counts["inference_error"] == 1


def test_every_answer_and_its_native_output_is_logged_without_the_frames():
    with tempfile.TemporaryDirectory() as d:
        log = Path(d) / "foreign.jsonl"
        core, _ = _core(_dronecats(), ['{"action":"finished"}'], log=log)
        core.handle_request(_req())
        rec = json.loads(log.read_text(encoding="utf-8").splitlines()[0])
    assert rec["response"]["land"] is True and rec["response"]["raw_text"] == "00 49 49 LAND</s>"
    assert rec["native"]["action"] == {"action": "finished"} and rec["mapping"]["land"] is True
    assert "timing" in rec and isinstance(rec["request"]["front_jpg_b64"], dict)


def _serve(core):
    srv = F.make_tcp_server(core, "127.0.0.1", 0)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    return srv, th, srv.server_address[1]


def _stop(srv, th):
    srv.shutdown()
    srv.server_close()
    th.join(timeout=10)


def _wait(cli, timeout=10.0):
    end = time.time() + timeout
    while time.time() < end:
        res = cli.poll()
        if res is not None:
            return res
        time.sleep(0.01)
    raise AssertionError("no reply")


def test_the_real_client_flies_the_foreign_server_over_tcp():
    outs = ['{"action":"go","point":[500,500],"depth_m":20}', '{"action":"finished"}', "junk"]
    core, model = _core(_dronecats(), outs)
    srv, th, port = _serve(core)
    assert port != 8790
    try:
        cli = RemoteVLA("127.0.0.1", port, response_timeout_s=5.0, strict_identity=True)
        ident = cli.connect()
        check_identity(ident)
        assert ident["variant"] == "foreign" and ident["foreign"]["bottom_tile"] == "none"
        cap = Capture(5.0, Pose(1.0, 2.0, 10.0, 0.3), b"256x256", b"256x256")
        kinds = []
        for k in range(3):
            fields, meta = build_infer_fields(cap, Hint("straight ahead ", "detector"),
                                              "a red car")
            fields["t_capture"] = meta["t_capture"] = 5.0 + k
            seq = cli.submit(fields)
            assert seq is not None
            res = _wait(cli)
            assert res.failure is None, res.detail
            kinds.append(res.reply.kind)
            rec = make_record(meta, res)
            assert rec["native"]["family"] == "dronecats", "the client's record lost the native"
        assert kinds == ["step", "land", "parse_fail"], kinds
        assert len(model.calls) == 3
        assert model.calls[0]["inputs"]["command"] == "go to the red car"
        cli.close()
    finally:
        _stop(srv, th)


def test_the_racevla_server_answers_its_un_normalised_step_over_tcp():
    with tempfile.TemporaryDirectory() as d:
        fam = _racevla(Path(d))
    toks = [_tok(254), _tok(0), _tok(127), _tok(127), _tok(127), _tok(140), _tok(127)]
    core, _ = _core(fam, [toks], _identity("racevla"))
    srv, th, port = _serve(core)
    try:
        cli = RemoteVLA("127.0.0.1", port, response_timeout_s=5.0, strict_identity=True)
        assert cli.connect()["id"].startswith("racevla/drone6_all@1a2b3c4d")
        cap = Capture(1.0, Pose(0.0, 0.0, 10.0, 0.0), b"256x256", None)
        fields, meta = build_infer_fields(cap, Hint("straight ahead ", "detector"), "a red car")
        cli.submit(fields)
        res = _wait(cli)
        assert res.failure is None and res.reply.kind == "step", res
        assert res.reply.raw_text == "37 49 35</s>", res.reply.raw_text
        rec = make_record(meta, res)
        assert rec["native"]["tokens"] == toks and len(rec["native"]["action"]["action"]) == 7
        assert rec["native"]["unnorm_key"] == "fake_set"
        cli.close()
    finally:
        _stop(srv, th)


# ====================================================== 4. identity and the allow-list

def test_hello_names_the_model_itself_with_the_foreign_inputs_declared():
    ident = _identity("dronecats")
    check_identity(ident)
    assert ident["id"] == "dronecats/DroneCATS-Cosmos3-Edge-2B@1a2b3c4d"
    assert ident["variant"] == C.FOREIGN_VARIANT == "foreign" and ident["weights_sha256"] == DIGEST
    blk = C.foreign_block(ident)
    assert blk["family"] == "dronecats" and blk["bottom_tile"] == C.FOREIGN_TILE_NONE == "none"
    assert blk["speed_words"] is None and blk["front_source"] == "vla"
    assert blk["camera"] == C.FRONT_SOURCES["vla"]
    assert C.adapter_variant(ident["id"]) is None, "a foreign model is no AerialVLA adapter"
    r = _identity("racevla", digest="ab" * 32)
    assert r["id"] == "racevla/drone6_all@abababab" and C.foreign_block(r)["family"] == "racevla"
    hello, _, _ = _core(_dronecats(), ["x"])[0].handle({"v": 2, "op": "hello"})
    assert hello["model"]["foreign"]["bottom_tile"] == "none"


def test_the_family_is_read_from_every_id_form_and_nothing_else_passes():
    for mid, fam in (("dronecats/DroneCATS-Cosmos3-Edge-2B@1a2b3c4d", "dronecats"),
                     ("dronecats:DroneCATS-Cosmos3-Edge-2B", "dronecats"),
                     ("dronecats:DroneCATS-Cosmos3-Edge-2B@sha256:" + DIGEST, "dronecats"),
                     ("racevla/drone6_all@abababab", "racevla"),
                     ("racevla:drone6_all@sha256:" + DIGEST, "racevla"),
                     ("fake:dronecats/x@1", None), ("aerialvla-lora/aero_vla@196f2f32", None),
                     ("aerialvla:openvla-7b+aero_vla", None), ("dronecatsx/foo@1", None),
                     ("dronecats/@1", None), ("dronecats", None), ("", None), (None, None)):
        assert C.foreign_family(mid) == fam, (mid, C.foreign_family(mid))
    assert C.foreign_pilot_id("dronecats/DroneCATS-Cosmos3-Edge-2B@1a2b3c4d") == \
        "dronecats:DroneCATS-Cosmos3-Edge-2B"
    assert C.foreign_pilot_id("aerialvla-lora/aero_vla@196f2f32") is None


def test_an_identity_that_claims_foreign_must_declare_its_inputs():
    good = _identity()
    for bad in ({**good, "foreign": None},
                {**good, "foreign": {**good["foreign"], "bottom_tile": "down"}},
                {**good, "foreign": {**good["foreign"], "family": "racevla"}},
                {**good, "foreign": {k: v for k, v in good["foreign"].items()
                                     if k != "speed_words"}},
                {**good, "foreign": {**good["foreign"], "front_source": "chase"}},
                {**good, "id": "aerialvla-lora/aero_vla@196f2f32"}):
        try:
            C.foreign_block(bad)
            raise AssertionError(f"accepted {bad.get('foreign')!r} / {bad['id']}")
        except ValueError:
            pass
    assert C.foreign_block({"id": "aerialvla-lora/aero_vla@196f2f32"}) is None


def test_the_flight_records_the_foreign_model_as_itself_never_as_aerialvla():
    ident = _identity()
    ok, why = VM.identity_is_vla(ident)
    assert ok, why
    assert VM.model_id_for(ident) == "dronecats:DroneCATS-Cosmos3-Edge-2B" != VM.MODEL_ID
    rec = VM.pilot_entry(ident, {"decisions_accepted": 0}, [], [], server={},
                         reported_by="127.0.0.1:8791")
    assert rec["kind"] == "vla" and rec["model_id"] == "dronecats:DroneCATS-Cosmos3-Edge-2B"
    assert rec["hash"] == f"dronecats:DroneCATS-Cosmos3-Edge-2B@sha256:{DIGEST}", rec["hash"]
    assert rec["server_identity"]["variant"] == "foreign"
    assert rec["server_identity"]["foreign"]["family"] == "dronecats"
    man = VM.manifest_for("sha256:" + "0f" * 32, ident, 0, ROOT / "README.md")
    assert man["vla_model_hash"].startswith("dronecats:DroneCATS-Cosmos3-Edge-2B@sha256:")
    assert VM.model_id_for(_identity("racevla")) == "racevla:drone6_all"


def test_the_dashboard_classifies_a_foreign_flight_as_a_vla_of_its_own_family():
    import todo as T
    assert T.FOREIGN_FAMILIES == C.FOREIGN_FAMILIES
    for mid in ("dronecats/DroneCATS-Cosmos3-Edge-2B@1a2b3c4d", "racevla:drone6_all",
                "fake:dronecats/x", "aerialvla:openvla-7b+aero_vla", "dronecatsx/y@1"):
        assert T.foreign_family(mid) == C.foreign_family(mid), mid
    h = f"dronecats:DroneCATS-Cosmos3-Edge-2B@sha256:{DIGEST}"
    assert T.classify_pilot(h) == "vla" and T.vla_family(h) == "dronecats"
    assert T.vla_family("racevla/drone6_all@abababab") == "racevla"
    assert T.vla_family("aerialvla:openvla-7b+aero_vla") == "aerialvla"
    ev = {"model_id": "dronecats/DroneCATS-Cosmos3-Edge-2B@1a2b3c4d", "weights_sha256": DIGEST}
    assert T.server_model_problem(h, ev) is None
    why = T.server_model_problem(f"aerialvla:openvla-7b+aero_vla@sha256:{DIGEST}", ev)
    assert why == "server_model_differs:configured aerialvla, server dronecats", why
    why = T.server_model_problem("racevla:drone6_all@sha256:" + DIGEST, ev)
    assert why == "server_model_differs:configured racevla, server dronecats", why
    assert T.classify_pilot("fake:dronecats/x@1") == "stub"


def test_a_foreign_flights_records_classify_as_a_vla_flight():
    import todo as T
    with tempfile.TemporaryDirectory() as d:
        run = Path(d)
        ident = _identity()
        rows = []
        for k in range(40):
            rows.append({"schema": T.VLA_STEPS_SCHEMA, "seq": k + 1, "kind": "step",
                         "accepted": True, "parse_ok": True, "model": ident,
                         "t_capture": float(k), "t_received": k + 0.4,
                         "outcome": {"ticks": {"vla_step": 9, "hold": 1}}})
        (run / "vla_steps.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n",
                                             encoding="utf-8")
        ev = T.vla_evidence(run, {"ticks": 400}, None)
    assert ev["family"] == "dronecats" and ev["adapter"] is None, ev
    cls, why = T.classify_run(f"dronecats:DroneCATS-Cosmos3-Edge-2B@sha256:{DIGEST}", ev)
    assert (cls, why) == ("vla", None), (cls, why)


def test_the_lower_tile_none_flies_the_default_down_tile_and_refuses_prev_front():
    ident = _identity()
    down = VM.bottom_tile_check(ident, C.bottom_tile("down"))
    assert down["ok"] and down["checked"] and down["adapter"] is None, down
    assert "none" in down["source"] and "ignores" in down["source"], down["source"]
    pf = VM.bottom_tile_check(ident, C.bottom_tile("prev_front"))
    assert pf["ok"] is False and "never reads" in pf["why"], pf
    # AeroVLA's own rule is unchanged: no adapter provenance is "down"
    aero = {"id": "aerialvla-lora/aero_vla@196f2f32", "base": "openvla-7b@47a0ec7f",
            "weights_sha256": DIGEST, "precision": "bf16", "attn": "sdpa", "merged": False}
    assert VM.bottom_tile_check(aero, C.bottom_tile("down"))["ok"]
    assert not VM.bottom_tile_check(aero, C.bottom_tile("prev_front"))["ok"]
    broken = {**ident, "foreign": {**ident["foreign"], "bottom_tile": "down"}}
    assert VM.bottom_tile_check(broken, C.bottom_tile("down"))["ok"] is False


def test_speed_words_are_off_for_a_foreign_model_either_way_round():
    ident = _identity()
    off = VM.speed_words_check(ident, None)
    assert off["ok"] and off["checked"] and "foreign" in off["source"], off
    on = VM.speed_words_check(ident, C.speed_words(True))
    assert on["ok"] is False and "speed words" in on["why"], on


def test_the_front_camera_the_server_assumes_must_be_the_flights():
    aero = {"id": "aerialvla-lora/aero_vla@196f2f32"}
    assert VM.front_source_check(aero, "vla")["ok"]
    no = VM.front_source_check(aero, "detector")
    assert no["ok"] is False and "VLAFront" in no["why"], no
    vla_cam, det_cam = _identity(source="vla"), _identity(source="detector")
    assert VM.front_source_check(vla_cam, "vla")["ok"]
    assert not VM.front_source_check(vla_cam, "detector")["ok"]
    assert VM.front_source_check(det_cam, "detector")["ok"]
    assert not VM.front_source_check(det_cam, "vla")["ok"]
    stand_in = VM.front_source_check({"id": "fake:scripted"}, "detector", is_vla=False)
    assert stand_in["ok"] and not stand_in["checked"]
    assert C.FRONT_SOURCES["detector"] == {"camera": "FrontCamera", "hfov_deg": 90.0,
                                           "pitch_deg": -20.0}


def test_the_hierarchy_flies_a_foreign_server_only_with_the_camera_it_reads():
    import vla_hierarchy as H
    from semantic_demo import SemanticObs
    kw = dict(obs=SemanticObs(), box_source=lambda: None, policy=None,
              object_text="a red car", cruise_alt=8.0, record_dir=None)
    for server_src, flight_src, accepted in (("vla", "vla", True), ("detector", "vla", False),
                                             ("detector", "detector", True),
                                             ("vla", "detector", False)):
        core, _ = _core(_dronecats(), ['{"action":"think"}'], _identity(source=server_src))
        srv, th, port = _serve(core)
        assert port != 8790
        try:
            try:
                h = H.make_vla_hierarchy(f"127.0.0.1:{port}", front_source=flight_src, **kw)
            except H.HierarchyRefused as exc:
                assert not accepted and "--vla-hierarchy-front-source" in str(exc), exc
                continue
            assert accepted, f"a server reading {server_src} was flown with {flight_src}"
            try:
                sv = h.summary()["server"]
                assert sv["is_vla"] and sv["model_id"] == "dronecats:DroneCATS-Cosmos3-Edge-2B", sv
                assert h.config["bottom_tile"]["ok"] and h.config["front_source"]["ok"]
                assert "none" in h.config["bottom_tile"]["source"] and h.front_source == flight_src
                assert h.describe().startswith("the foreign VLA dronecats dronecats/"), \
                    h.describe()[:80]
            finally:
                h.stop()
        finally:
            _stop(srv, th)
    assert H.check_args("127.0.0.1:59999", front_source="side")
    import follow_vlm as FV
    a = FV.build_arg_parser().parse_args(["--vla-hierarchy", "127.0.0.1:59999",
                                          "--vla-hierarchy-front-source", "detector"])
    assert a.vla_hierarchy_front_source == "detector"
    assert FV.build_arg_parser().parse_args([]).vla_hierarchy_front_source == "vla"


def test_follow_vlm_refuses_the_front_source_without_the_hierarchy():
    import contextlib
    import io
    import follow_vlm as FV

    async def _no_flight(args):
        raise AssertionError("accepted, and would have flown")
    real, FV.fly = FV.fly, _no_flight
    err = io.StringIO()
    try:
        with contextlib.redirect_stderr(err):
            FV.main(["--vla-hierarchy-front-source", "detector"])
        raise AssertionError("accepted")
    except SystemExit as exc:
        assert exc.code == 2 and "need --vla-hierarchy" in err.getvalue(), err.getvalue()[-300:]
    finally:
        FV.fly = real
    ps1 = (ROOT / "scripts" / "run_citylife_follow.ps1").read_text(encoding="utf-8")
    assert "[string]$VlaHierarchyFrontSource" in ps1
    assert '"--vla-hierarchy-front-source", $VlaHierarchyFrontSource' in ps1


def test_vla_fly_takes_the_front_source_flag_default_vla():
    import vla_fly as VF
    a = VF.resolve_args(VF.build_arg_parser().parse_args(["--mission", "follow"]))
    assert a.vla_front_source == "vla"
    a = VF.resolve_args(VF.build_arg_parser().parse_args(
        ["--mission", "follow", "--vla-front-source", "detector"]))
    assert a.vla_front_source == "detector"
    try:
        VF.resolve_args(VF.build_arg_parser().parse_args(
            ["--mission", "goto", "--vla-front-source", "detector"]))
        raise AssertionError("goto has no detector camera")
    except ValueError as exc:
        assert "detector" in str(exc)
    assert VF.front_camera(a) == "FrontCamera"


def test_the_scripted_stand_in_is_never_a_vla():
    ident = F.stand_in_identity("dronecats")
    ok, why = VM.identity_is_vla(ident)
    assert not ok and ident["id"].startswith("fake:"), (ident, why)


def test_the_server_module_imports_without_torch_pillow_transformers_or_pydantic():
    # The dronecats env has torch and transformers but no pydantic or shapely
    # (guardrail/__init__ imports the Shield): the server must import the codec
    # without the package's __init__. pydantic is blocked here to show it.
    code = ("import sys; sys.modules['pydantic'] = None; sys.modules['shapely'] = None; "
            f"sys.path[:0] = [{str(ROOT)!r}, {str(ROOT / 'demo')!r}]; "
            "import vla_server_foreign as F; "
            "print(sorted(m for m in ('torch', 'PIL', 'transformers', 'numpy') "
            "if m in sys.modules)); print(F.C.parse_action('00 49 49</s>').parse_ok)")
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                       timeout=120, cwd=str(ROOT))
    assert r.returncode == 0, r.stderr[-2000:]
    assert r.stdout.split() == ["[]", "True"], r.stdout


def test_the_serve_cli_refuses_a_missing_port_and_an_unknown_model():
    import contextlib
    import io
    for argv in (["serve", "--model", "dronecats", "--weights", "x"],
                 ["serve", "--model", "openvla", "--weights", "x", "--port", "1"],
                 ["serve", "--model", "dronecats", "--weights", "x", "--port", "1",
                  "--precision", "nf4"],
                 ["serve", "--model", "dronecats", "--weights", "x", "--port", "1",
                  "--command", "go to it"]):
        err = io.StringIO()
        try:
            with contextlib.redirect_stderr(err):
                F.main(argv)
            raise AssertionError(f"accepted {argv}")
        except SystemExit as exc:
            assert exc.code not in (0, None), argv
        assert "error" in err.getvalue(), (argv, err.getvalue())


def test_dronecats_nf4_needs_bitsandbytes_and_quantises_only_the_language_layers():
    import contextlib
    import io
    import types
    argv = ["serve", "--model", "dronecats", "--weights", "x", "--port", "1",
            "--precision", "nf4"]
    real_check = F.bitsandbytes_available
    try:
        for present, says in ((False, "bitsandbytes"), (True, "no such folder")):
            F.bitsandbytes_available = lambda present=present: present
            err = io.StringIO()
            try:
                with contextlib.redirect_stderr(err):
                    F.main(argv)
                raise AssertionError(f"accepted {argv} with bitsandbytes={present}")
            except SystemExit as exc:
                assert exc.code not in (0, None)
            assert says in err.getvalue(), (present, err.getvalue())
    finally:
        F.bitsandbytes_available = real_check

    seen: dict = {}

    class FakeBnb:
        def __init__(self, **kw):
            self.kw = kw

    class FakeModel:
        @classmethod
        def from_pretrained(cls, path, **kw):
            seen["kw"] = kw
            return cls()

        def eval(self):
            return self

    class FakeProcessor:
        @classmethod
        def from_pretrained(cls, path):
            return cls()

    fake_tf = types.ModuleType("transformers")
    fake_tf.AutoProcessor = FakeProcessor
    fake_tf.Cosmos3EdgeForConditionalGeneration = FakeModel
    fake_tf.BitsAndBytesConfig = FakeBnb
    fake_torch = types.ModuleType("torch")
    fake_torch.bfloat16 = "bf16"
    saved = {k: sys.modules.get(k) for k in ("torch", "transformers")}
    sys.modules["torch"], sys.modules["transformers"] = fake_torch, fake_tf
    try:
        F.DroneCATSModel("w", attn="sdpa", device="cpu", precision="nf4")
        q = seen["kw"]["quantization_config"].kw
        assert q["load_in_4bit"] is True and q["bnb_4bit_quant_type"] == "nf4", q
        assert q["bnb_4bit_compute_dtype"] == "bf16", q
        assert q["llm_int8_skip_modules"] == ["visual", "projector", "lm_head"], q
        assert seen["kw"]["dtype"] == "bf16"
        F.DroneCATSModel("w", attn="sdpa", device="cpu", precision="bf16")
        assert "quantization_config" not in seen["kw"], seen["kw"]
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v


def test_the_weight_files_are_the_index_shards_plus_racevlas_statistics():
    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        (d / "transformer").mkdir()
        (d / "model.safetensors.index.json").write_text(json.dumps(
            {"weight_map": {"a": "transformer/model.safetensors"}}), encoding="utf-8")
        assert F.weight_files("dronecats", d) == [d / "transformer" / "model.safetensors"]
        (d / "model.safetensors.index.json").write_text(json.dumps(
            {"weight_map": {"a": "model-00001-of-00002.safetensors",
                            "b": "model-00002-of-00002.safetensors"}}), encoding="utf-8")
        assert F.weight_files("racevla", d) == [d / "model-00001-of-00002.safetensors",
                                                d / "model-00002-of-00002.safetensors",
                                                d / "dataset_statistics.json"]


# ====================================================== 5. the bench

def test_the_bench_reports_latency_and_the_native_histogram_against_a_server():
    import bench_foreign_vla as B
    outs = ['{"action":"go","point":[500,500],"depth_m":20}', '{"action":"rotate","angle_deg":30}',
            '{"action":"go","point":[600,500],"depth_m":8}', "junk"]
    core, _ = _core(_dronecats(), outs)
    srv, th, port = _serve(core)
    try:
        with tempfile.TemporaryDirectory() as d:
            for k in range(2):
                (Path(d) / f"f{k}.jpg").write_bytes(b"256x256")
            out = Path(d) / "bench.json"
            assert B.main(["--server", f"127.0.0.1:{port}", "--n", "8", "--warmup", "0",
                           "--jpg-dir", d, "--out", str(out)]) == 0
            res = json.loads(out.read_text(encoding="utf-8"))
    finally:
        _stop(srv, th)
    assert res["n"] == 8 and res["server_latency"]["n"] == 8 and "p95_ms" in res["roundtrip"]
    assert res["native_kinds"] == {"go": 4, "rotate": 2, "malformed": 2}, res["native_kinds"]
    assert res["step_texts"]["59 49 49</s>"] == 2 and res["parse_fail"] == 2
    assert res["identity"]["id"].startswith("dronecats/")


def test_the_bench_histogram_counts_each_native_kind():
    import bench_foreign_vla as B
    replies = [{"ok": True, "native": {"ok": True, "action": {"action": "go"}}, "raw_text": "a"},
               {"ok": True, "native": {"ok": False, "action": None}, "raw_text": ""},
               {"ok": True, "native": {"ok": True, "action": {"action": [0.1] * 7}},
                "raw_text": "b"},
               {"ok": False, "native": None, "raw_text": ""}]
    assert B.native_kind(replies[0]) == "go" and B.native_kind(replies[1]) == "malformed"
    assert B.native_kind(replies[2]) == "velocity" and B.native_kind(replies[3]) == "refused"


# --------------------------------------------------------------------- runner

if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
