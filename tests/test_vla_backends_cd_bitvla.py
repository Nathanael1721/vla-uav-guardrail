"""
CognitiveDrone and BitVLA in the switchable backend slot (ARCH-06,
2026-11-17): demo/vla_server_foreign.py --model cognitivedrone | bitvla.

Run either way:
    pytest tests/test_vla_backends_cd_bitvla.py -v
    python tests/test_vla_backends_cd_bitvla.py

Neither model's weights are on this machine (CognitiveDrone has published
none; BitVLA's are LIBERO arm fine-tunes whose download needs the user's
approval), so no test here loads a model or touches a GPU: the models are
ScriptedForeignModel stand-ins that return each family's NATIVE output (the
OpenVLA action tokens of CognitiveDrone, BitVLA's un-normalised action
chunk). What is tested:

  1. The two mappings, native output -> the grant's 4-D (vx, vy, vz,
     yaw_rate) read body FLU -> the lateral step text, exactly, and the
     slots each statistics dimension reads.
  2. 4-D conformance, MEASURED: a seeded sweep of random native outputs per
     arm (CognitiveDrone 4-D and 7-D statistics, BitVLA) through the
     server's protocol core, the client's strict reader and the StepTracker
     in the launcher's rate execution (H 1 s); every one comes out as a
     finite Action4D (vx, vy, vz_up, yaw_rate) - true by construction,
     since encode_step clips into the codec's ranges - so what is checked
     and printed is what the slot LOSES, per axis: the AerialVLA step codec
     cannot carry vx < 0 (it is flown as 0) and caps the turn at 1.1 rad
     per horizon; above the bin width every other axis keeps its sign.
  3. The same over TCP: the real client (strict identity) and
     RemoteVLABackend.act() -> Action4D, the identity naming the family.
  4. The switch: --model picks the family (build_family), the dashboard and
     the video label name it, deploy/orin/run_pas_perception.sh FAMILY=
     gives it an option set the perception node accepts.
  5. The refusal: without the download, serve stops before anything loads,
     saying "weights not obtained" and which files are missing.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import random
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "tools", ROOT / "tests"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import vla_server_foreign as F                                       # noqa: E402
from guardrail import aerialvla_codec as C                           # noqa: E402
from guardrail.models import Action4D                                # noqa: E402
from guardrail.vla_link import (Capture, Decision, Hint, Pose,       # noqa: E402
                                RemoteVLA, Step, StepTracker, TrackerConfig,
                                check_identity)

SKIP = "SKIP"
DIGEST = "c0ffee00" * 8
SWEEP_N = 500


class Img:
    """Stand-in for a decoded PIL image: only its size (and convert) are read."""
    def __init__(self, w=256, h=256):
        self.size = (w, h)

    def convert(self, mode):
        return self


def _decode(req):
    return Img()


def _hash(img):
    return "0" * 64


def _req(seq=1, t=100.0):
    return {"v": 2, "op": "infer", "seq": seq, "t_capture": t,
            "pose": {"x": 0.0, "y": 0.0, "up": 10.0, "yaw_rad": 0.0},
            "front_jpg_b64": "AAAA", "down_jpg_b64": None, "phrase": "straight ahead ",
            "hint_source": "detector", "object": "a red car", "csp_text": None,
            "csp_mode": "off"}


def _stats(d: Path, dim: int) -> Path:
    q01 = [-1.0, -1.0, -0.5, -0.8] if dim == 4 else [-1.0, -1.0, -0.5, 0.0, 0.0, -0.8, 0.0]
    q99 = [1.0, 1.0, 0.5, 0.8] if dim == 4 else [1.0, 1.0, 0.5, 0.0, 0.0, 0.8, 1.0]
    p = d / "dataset_statistics.json"
    p.write_text(json.dumps({"cd_set": {"action": {"q01": q01, "q99": q99}}}), encoding="utf-8")
    return p


def _cd_family(dim: int = 4) -> F.CognitiveDroneFamily:
    with tempfile.TemporaryDirectory() as d:
        key, stats = F.load_action_stats(_stats(Path(d), dim), dims=F.COGNITIVEDRONE_DIMS)
    return F.CognitiveDroneFamily(stats, key)


def _bit_family() -> F.BitVLAFamily:
    return F.BitVLAFamily("libero_goal_no_noops")


def _identity(family: str) -> dict:
    name = {"cognitivedrone": "CognitiveDrone", "bitvla": "ft-bitvla-libero_goal"}[family]
    return F.foreign_identity(family, name=name, folder=name, revision=None, digest=DIGEST,
                              precision="bf16", attn="sdpa", front_source="vla",
                              facts={"test": True})


def _core(family, outputs):
    model = F.ScriptedForeignModel(list(outputs))
    core = F.ForeignServer(family, model, _identity(family.name), None, decode_front=_decode,
                           hash_fn=_hash)
    return core, model


def _tok(idx: int) -> int:
    """The OpenVLA action token whose bin index (vocab - token - 1) is idx."""
    return F.OPENVLA_VOCAB_SIZE - (idx + 1)


# The tracker the hil launcher flies both families with (deploy/orin/
# run_pas_perception.sh FAMILY=cognitivedrone|bitvla: --vla-step-as-rate 1.0,
# --vla-cruise-mps 4.5, --vla-speed-cap-mps 5; the follow executor's turn
# while moving). Rate execution: speed fwd_m / H along the body heading,
# right_m / H to its right, yaw rate dpsi_rad / H (TrackerConfig, RATE
# EXECUTION). Left out: the follow mission's held altitude (its vz is then
# recorded, not flown - kept out so vz's sign is checked here) and its
# smoothing (it limits a change's size over ticks, never its sign).
LAUNCHER_TRACKER = TrackerConfig(cruise_mps=4.5, speed_cap_mps=5.0, rate_horizon_s=1.0,
                                 turn_while_moving=True)


def _action4d(step_wire: dict, t0: float = 100.0, cfg: TrackerConfig | None = None) -> Action4D:
    """A decoded step through the StepTracker (the launcher's rate execution
    unless `cfg`), one tick after it was taken: what the rail hands the
    Shield. Pose yaw 0, so vx is North = forward and vy East = right."""
    tr = StepTracker(LAUNCHER_TRACKER if cfg is None else cfg)
    pose = Pose(0.0, 0.0, 10.0, 0.0)
    tr.accept(Decision.of(1, t0, pose, Step.from_wire(step_wire)), t0 + 0.05)
    return tr.tick(t0 + 0.15, pose).action


def _finite4(a: Action4D) -> bool:
    return isinstance(a, Action4D) and all(
        math.isfinite(v) for v in (a.vx, a.vy, a.vz_up, a.yaw_rate))


# ====================================================== 1. the mappings

def test_velocity4_is_body_flu_held_over_the_horizon_with_the_lateral_kept():
    m = F.map_velocity4(1.5, 0.5, -0.4, 0.2, horizon_s=2.0)
    assert m["kind"] == "velocity4" and m["land"] is False
    assert (m["fwd_m"], m["right_m"], m["down_m"]) == (3.0, -1.0, 0.8)
    assert abs(m["dpsi_rad"] - -0.4) < 1e-12, "omega + counter-clockwise -> dpsi + clockwise"
    st = F.encode_step(m["fwd_m"], m["down_m"], m["dpsi_rad"], right_m=m["right_m"])
    p = C.parse_action(st["text"], require_eos=True, lateral=True)
    assert p.parse_ok and len(p.bins) == 4 and p.step.right_m is not None
    assert abs(p.step.right_m - -1.0) <= 5.0 / 98, p.step


def test_cognitivedrone_reads_4_d_statistics_as_vx_vy_vz_omega():
    fam = _cd_family(4)
    assert fam.action_dim == 4 and fam.gray is False
    toks = [_tok(254), _tok(127), _tok(0), _tok(127)]          # +1, 0, -0.5, ~0
    nat = fam.parse({"raw": toks})
    assert nat["ok"], nat
    a = nat["action"]["action"]
    assert abs(a[0] - 1.0) < 0.01 and abs(a[2] - -0.5) < 0.01, a
    m = fam.map(nat["action"], None)
    assert m["geometry"]["slots"] == [0, 1, 2, 3]
    assert abs(m["fwd_m"] - a[0]) < 1e-9 and abs(m["down_m"] - 0.5) < 0.01
    assert fam.inputs(_req(), None, [])["max_new_tokens"] == 4
    assert fam.inputs(_req(), None, [])["prompt"] == \
        "In: What action should the robot take to navigate to the red car?\nOut:"


def test_cognitivedrone_reads_7_d_statistics_racevlas_way():
    fam = _cd_family(7)
    toks = [_tok(127)] * 5 + [_tok(254), _tok(127)]
    m = fam.map(fam.parse({"raw": toks})["action"], None)
    assert m["geometry"]["slots"] == [0, 1, 2, 5] and m["dpsi_rad"] < -0.7, m


def test_bitvla_flies_the_first_row_of_its_chunk():
    fam = _bit_family()
    chunk = [[0.8, -0.3, 0.2, 0.1, 0.1, -0.5, 1.0]] + [[9.0] * 7] * 7
    nat = fam.parse({"raw": chunk})
    assert nat["ok"] and nat["chunk_len"] == 8
    m = fam.map(nat["action"], None)
    assert (m["fwd_m"], m["right_m"], m["down_m"], m["dpsi_rad"]) == (0.8, 0.3, -0.2, 0.5)
    assert m["geometry"]["rows_not_flown"] == 7 and m["geometry"]["unused"]["6"] == 1.0


def _evaluator_user_text(task_label: str, n_images: int) -> str:
    """BitVLA's evaluator, transcribed (github.com/ustcwhy/BitVLA@a8d62df
    openvla-oft/experiments/robot/bitnet_utils.get_bitnet_vla_action: the
    human turn; bitvla/dataset/bitvla_transform.replace_image_tokens, applied
    by llava_to_openai; bitvla/constants.py's two image tokens)."""
    import re
    value = "<image>\n" * n_images + "<proprio_pad>" + \
        f"What action should the robot take to {task_label.lower()}?"
    return re.sub(r"\n?" + re.escape("<image>") + r"\n?", "<|image_pad|>", value)


def test_bitvla_asks_in_its_evaluators_words_not_openvlas():
    """First real load, 2026-11-22: the remote code's predict_action counts the
    prompt's image tokens against the image features and refuses a mismatch,
    so OpenVLA's "In: ... Out:" prompt (no image token) cannot run. The user
    turn is the evaluator's, with one image token per camera slot."""
    fam = _bit_family()
    ins = fam.inputs(_req(), None, [])
    assert ins["n_images"] == F.BITVLA_N_IMAGES == 2
    assert ins["prompt"] == _evaluator_user_text("fly to the red car", 2), ins["prompt"]
    assert ins["prompt"] == ("<|image_pad|><|image_pad|><proprio_pad>What action should the "
                             "robot take to fly to the red car?")
    assert fam.describe()["prompt"] == F.BITVLA_USER_TEXT
    # each image token becomes one per patch, after the chat template
    t = F.bitvla_expand_images("User: " + ins["prompt"] + "<|eot_id|>Assistant: ", 256, 2)
    assert t.count("<|image_pad|>") == 512 and t.endswith("Assistant: ")
    for bad in (1, 3):
        try:
            F.bitvla_expand_images(ins["prompt"], 256, bad)
            raise AssertionError(f"{bad} images accepted for a prompt holding 2")
        except ValueError:
            pass
    # the evaluator's token ids (bitvla/constants.py BITNET_*), for set_constant
    assert F.BITVLA_TOKEN_IDS == {"image_token_idx": 128010, "proprio_pad_idx": 128011,
                                  "ignore_idx": -100, "action_token_begin_idx": 128011,
                                  "stop_index": 128001}


def test_bitvla_frame_is_resized_and_centre_cropped_as_its_evaluator_does():
    try:
        from PIL import Image
    except ImportError:
        return SKIP
    img = Image.new("RGB", (640, 480), (0, 0, 0))
    img.paste((255, 255, 255), (0, 0, 640, 24))           # a white band at the top
    out = F.bitvla_prepare_image(img)
    assert out.size == (224, 224) and out.mode == "RGB"
    # the 0.9-area centre crop cuts ~2.6 % off each side: the band (5 % of the
    # height) shrinks to ~2.5 % of it, its first row still white
    top = [out.getpixel((112, y))[0] for y in range(12)]
    assert top[0] > 200 and top[-1] < 50, top
    same = F.bitvla_prepare_image(Image.new("L", (224, 224), 77))
    assert same.size == (224, 224) and same.getpixel((5, 5)) == (77, 77, 77)


def test_malformed_native_outputs_are_refused_never_read_as_motion():
    cd, bit = _cd_family(4), _bit_family()
    for raw in ([1, 2, 3], [1.0, 2, 3, 4], "junk", None, [_tok(1)] * 7):
        assert cd.parse({"raw": raw})["ok"] is False, raw
    for raw in ([], [[1.0] * 6], [[float("nan")] * 7], "junk", [[1] * 7, "x"]):
        assert bit.parse({"raw": raw})["ok"] is False, raw
    core, _ = _core(cd, [[1, 2]])
    resp, _, _ = core.handle(_req())
    assert resp["ok"] and resp["parse_ok"] is False and resp["step"] is None


# ====================================================== 2. 4-D conformance, measured

def _random_native(arm: str, rng: random.Random):
    if arm.startswith("cognitivedrone"):
        return [_tok(rng.randrange(0, 255)) for _ in range(4 if arm.endswith("4d") else 7)]
    return [[rng.uniform(-1.5, 1.5) for _ in range(7)] for _ in range(8)]


ARMS = ("cognitivedrone_4d", "cognitivedrone_7d", "bitvla")
# Above these (H = 1 s) the axis is past the codec's bin width and, for yaw,
# the tracker's 0.05 rad heading tolerance; for vz the 0.3 m altitude
# tolerance: the Action4D component must then carry the model's sign.
SIGN_MIN = {"vx": 0.06, "vy": 0.11, "vz": 0.45, "omega": 0.08}


def _sign(v: float) -> int:
    return (v > 0) - (v < 0)


def conformance(arm: str, n: int = SWEEP_N, seed: int = 20261117) -> dict:
    """n seeded random native outputs of `arm` through the server's protocol
    core, the client's strict reader and the StepTracker in the launcher's
    rate execution (LAUNCHER_TRACKER). Counts: finite 4-D Action4D; the
    codec's clips per axis (encode_step's record); vx < 0 flown as vx = 0
    (the step codec has no backward step); and per axis, the samples above
    SIGN_MIN whose Action4D component carries the model's sign (vx forward
    = North, right = -vy = East, vz up, yaw clockwise = -omega)."""
    rng = random.Random(seed)
    fam = {"cognitivedrone_4d": lambda: _cd_family(4), "cognitivedrone_7d": lambda: _cd_family(7),
           "bitvla": _bit_family}[arm]()
    outs = [_random_native(arm, rng) for _ in range(n)]
    core, _ = _core(fam, outs)
    ok, clips = 0, {}
    back = back_zero = 0
    signs = {k: [0, 0] for k in SIGN_MIN}            # [carried, checked]
    yaw_over_cap = 0
    for i in range(n):
        resp, _, _ = core.handle(_req(seq=i + 1, t=100.0 + i))
        if not (resp["ok"] and resp["parse_ok"] and len(resp["bins"]) == 4):
            continue
        p = C.parse_action(resp["raw_text"], require_eos=True, lateral=True)
        if not (p.parse_ok and p.step.to_dict() == resp["step"]):
            continue
        m = resp["mapping"]
        g = m["geometry"]
        for ax in m["clipped"]:
            key = ax
            if ax == "fwd_m":
                key = "fwd_m(vx<0)" if g["vx"] < 0 else "fwd_m(vx>5)"
            clips[key] = clips.get(key, 0) + 1
        a = _action4d(resp["step"])
        if not _finite4(a):
            continue
        ok += 1
        if g["vx"] < 0:
            back += 1
            back_zero += abs(a.vx) < 1e-12
        yaw_over_cap += abs(g["omega"]) > LAUNCHER_TRACKER.yaw_rate_cap_rad_s
        for k, got, want in (("vx", a.vx, g["vx"]), ("vy", a.vy, -g["vy"]),
                             ("vz", a.vz_up, g["vz"]), ("omega", a.yaw_rate, -g["omega"])):
            if abs(want) <= SIGN_MIN[k] or (k == "vx" and want < 0):
                continue
            signs[k][1] += 1
            signs[k][0] += _sign(got) == _sign(want)
    return {"arm": arm, "n": n, "action4d_finite": ok, "clips_per_axis": clips,
            "vx_negative": back, "vx_negative_flown_as_zero": back_zero,
            "sign_carried": {k: f"{v[0]}/{v[1]}" for k, v in signs.items()},
            "signs": signs, "omega_over_tracker_yaw_cap": yaw_over_cap}


def test_every_native_output_becomes_a_finite_4_d_action_and_the_losses_are_counted():
    """The shape conforms (finite Action4D every time - guaranteed by
    encode_step's clipping, so not the point by itself); what the slot LOSES
    is counted and pinned: vx < 0 becomes 0, |omega H| > 1.1 rad is clipped.
    Above SIGN_MIN every other axis keeps the model's sign."""
    for arm in ARMS:
        r = conformance(arm)
        print(f"      conformance {arm}: {r['action4d_finite']}/{r['n']} -> finite Action4D "
              f"(launcher rate execution H 1 s); clips per axis {r['clips_per_axis']}; "
              f"vx<0 flown as 0: {r['vx_negative_flown_as_zero']}/{r['vx_negative']}; "
              f"sign carried {r['sign_carried']}; |omega| over the tracker's 0.5 rad/s "
              f"cap {r['omega_over_tracker_yaw_cap']}")
        assert r["action4d_finite"] == r["n"] == SWEEP_N, r
        # every backward request lost its sign: the codec cannot carry it
        assert r["vx_negative"] > 0 and r["vx_negative_flown_as_zero"] == r["vx_negative"], r
        assert r["clips_per_axis"].get("fwd_m(vx<0)", 0) == r["vx_negative"], r
        for k, (carried, checked) in r["signs"].items():
            assert checked > 0 and carried == checked, (arm, k, r)
        assert set(r["clips_per_axis"]) <= {"fwd_m(vx<0)", "dpsi_rad"}, r
    assert conformance("bitvla")["clips_per_axis"].get("dpsi_rad", 0) > 0


def test_the_step_codec_cannot_fly_backward_and_caps_the_turn_per_horizon():
    """Pinned so the limitation stays visible: a backward velocity flies 0,
    and a turn over 1.1 rad per horizon flies 1.1 rad."""
    m = F.map_velocity4(-1.0, 0.0, 0.0, 0.0, horizon_s=1.0)
    st = F.encode_step(m["fwd_m"], m["down_m"], m["dpsi_rad"], right_m=m["right_m"])
    assert st["clipped"] == ["fwd_m"] and st["flown"]["fwd_m"] == 0.0, st
    a = _action4d(C.parse_action(st["text"], require_eos=True, lateral=True).step.to_dict())
    assert a.vx == 0.0 and abs(a.vy) < 0.06, a        # vy: right_m 0 within one bin
    m = F.map_velocity4(0.0, 0.0, 0.0, 2.0, horizon_s=1.0)       # 2 rad/s ccw
    st = F.encode_step(m["fwd_m"], m["down_m"], m["dpsi_rad"], right_m=m["right_m"])
    assert st["clipped"] == ["dpsi_rad"] and abs(st["flown"]["dpsi_rad"] - -1.1) < 0.03, st


# ====================================================== 3. over TCP, through the backend

def _serve(core):
    srv = F.make_tcp_server(core, "127.0.0.1", 0)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    return srv, th, srv.server_address[1]


def test_the_real_client_and_remote_backend_fly_both_families_as_action4d():
    from guardrail.models import State
    from guardrail.vla_backends import RemoteVLABackend
    natives = {"cognitivedrone": [_tok(254), _tok(60), _tok(127), _tok(160)],
               "bitvla": [[1.0, 0.4, 0.0, 0.0, 0.0, 0.1, 0.0]] * 8}
    for name, native in natives.items():
        fam = _cd_family(4) if name == "cognitivedrone" else _bit_family()
        core, model = _core(fam, [native])
        srv, th, port = _serve(core)
        try:
            cli = RemoteVLA("127.0.0.1", port, response_timeout_s=5.0, strict_identity=True)
            ident = cli.connect()
            check_identity(ident)
            assert ident["id"].startswith(f"{name}/") and C.foreign_family(ident["id"]) == name
            state = State(x=0.0, y=0.0, up=10.0, yaw_deg=0.0)
            be = RemoteVLABackend(cli, lambda: Capture(time.monotonic(), Pose(0, 0, 10, 0),
                                                       b"jpg", None),
                                  lambda cap: Hint("straight ahead ", "detector"), "a red car",
                                  tracker=StepTracker(LAUNCHER_TRACKER))
            end, act = time.time() + 10.0, None
            while time.time() < end:
                act = be.act(state)
                if be.tracker.counts.get("kind_step", 0) and (act.vx or act.vy):
                    break
                time.sleep(0.02)
            assert _finite4(act) and (act.vx or act.vy), (name, act, be.stats()["answers"])
            assert be.records == [] or all(r.get("kind") for r in be.records)
            be.close()
        finally:
            srv.shutdown()
            srv.server_close()
            th.join(timeout=10)


# ====================================================== 4. the switch

def _args(model: str, weights: Path, *extra: str) -> argparse.Namespace:
    ap, _ = F.build_arg_parser()
    a = ap.parse_args(["serve", "--model", model, "--weights", str(weights), "--port", "1",
                       *extra])
    return a


def _download(d: Path, family: str) -> Path:
    for name in F.REQUIRED_FILES[family]:
        if name == "dataset_statistics.json":
            _stats(d, 4 if family == "cognitivedrone" else 7)
        elif name == "model.safetensors.index.json":
            (d / name).write_text(json.dumps({"weight_map": {"a": "model.safetensors"}}),
                                  encoding="utf-8")
        else:
            (d / name.replace("*", "10000_checkpoint")).write_text("{}", encoding="utf-8")
    return d


def test_the_model_flag_switches_the_family_in_the_slot():
    assert {"cognitivedrone", "bitvla"} <= set(F.FAMILIES)
    with tempfile.TemporaryDirectory() as d:
        w = _download(Path(d), "cognitivedrone")
        fam = F.build_family(_args("cognitivedrone", w), w)
        assert isinstance(fam, F.CognitiveDroneFamily) and fam.action_dim == 4
    with tempfile.TemporaryDirectory() as d:
        w = _download(Path(d), "bitvla")
        fam = F.build_family(_args("bitvla", w), w)
        assert isinstance(fam, F.BitVLAFamily) and fam.unnorm_key == "cd_set"
        # the identity digest pins the config and the action head too; not the
        # checkpoint's stale model file, which does not run (BitVLAModel: the
        # repository's class runs, its sha256 is in the identity's facts)
        assert [p.name for p in F.weight_files("bitvla", w)][-3:] == [
            "dataset_statistics.json", "action_head--10000_checkpoint.pt", "config.json"]
        # BitVLA's chunk rows are 7 wide: 4-D statistics are refused
        _stats(Path(d), 4)
        try:
            F.build_family(_args("bitvla", w), w)
            raise AssertionError("bitvla accepted 4-D statistics")
        except ValueError as exc:
            assert "7-dimensional" in str(exc), exc
    # --fake-native (a stand-in, no GPU) needs only the statistics, as RaceVLA's
    with tempfile.TemporaryDirectory() as d:
        for name, dim in (("cognitivedrone", 4), ("bitvla", 7)):
            _stats(Path(d), dim)
            fam = F.build_family(_args(name, Path(d), "--fake-native", "[1]"), Path(d))
            assert fam.name == name
            assert F.missing_files(name, Path(d), fake=True) == []
            assert F.missing_files(name, Path(d)) != []
    # a run of either keeps a name of its own in the backend table, never the
    # row (no weights obtained; LIBERO weights are not follow evidence)
    import vla_backend_table as VBT
    for name in ("cognitivedrone", "bitvla"):
        row, fam_name = VBT.backend_of_identity(_identity(name))
        assert fam_name == name and row.startswith(f"{name}:") and row not in VBT.BACKENDS, row
    import todo as T
    import vla_mission as VM
    assert T.FOREIGN_FAMILIES == C.FOREIGN_FAMILIES
    for name, label in (("cognitivedrone", "CognitiveDrone"), ("bitvla", "BitVLA")):
        ident = _identity(name)
        assert C.foreign_block(ident)["mapping"] == F.MAPPING_RULES[name]
        assert VM.model_label(ident) == label
        assert T.vla_family(VM.model_id_for(ident)) == name


def test_the_hil_launcher_gives_both_families_an_option_set_the_node_accepts():
    import test_deploy_configs as D
    if not D.find_bash():
        return SKIP
    for fam in ("cognitivedrone", "bitvla"):
        r = D._pas_perception([], FAMILY=fam, TAG=None)
        assert r.returncode == 0, (fam, r.stderr)
        av = D.printed(r)
        o = D.options(av)
        assert o["--tag"].startswith(f"pas_hil_{fam}_2"), o
        assert (o["--vla-front-source"], o["--vla-step-as-rate"], o["--vla-feed"]) == \
            ("vla", "1.0", "tick"), o
        reads = D._node_reads(av[2:])
        assert reads[:4] == ["vla", "follow", "vla", 1.0], reads


# ====================================================== 5. the refusal

def test_without_the_download_serve_refuses_before_anything_loads():
    for fam in ("cognitivedrone", "bitvla"):
        with tempfile.TemporaryDirectory() as d:
            err = io.StringIO()
            try:
                with contextlib.redirect_stderr(err):
                    F.main(["serve", "--model", fam, "--weights", d, "--port", "1"])
                raise AssertionError(f"{fam}: served from an empty folder")
            except SystemExit as exc:
                assert exc.code not in (0, None)
            msg = err.getvalue()
            assert "weights not obtained" in msg, msg
            for name in F.REQUIRED_FILES[fam]:
                assert name in msg, (name, msg)
        assert F.missing_files(fam, Path(d) / "nowhere") == list(F.REQUIRED_FILES[fam])
    err = io.StringIO()
    with tempfile.TemporaryDirectory() as d:
        try:
            with contextlib.redirect_stderr(err):
                F.main(["serve", "--model", "bitvla", "--weights", d, "--port", "1",
                        "--precision", "nf4"])
            raise AssertionError("bitvla accepted nf4")
        except SystemExit as exc:
            assert exc.code not in (0, None)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
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
