"""
training/aiotvla - the lab's own VLA: what training and serving must compute
identically, and its place behind protocol v2 (demo/vla_server_foreign.py).

Run either way:
    pytest tests/test_aiotvla.py -v
    python tests/test_aiotvla.py

No test loads OWL-ViT or touches a GPU. torch is used where it is installed
(the head's shapes, the batched grounding stats) and those tests SKIP
without it. What is tested:

  1. core: history slots (nearest capture per offset, never newer, empty out
     of tolerance), backward-only velocity (no future pose), proprio in the
     newest capture's body frame, the mirror (exactly the right-handed
     fields negated; twice is the identity), soft targets and decode (a
     two-peaked answer decodes to a peak, not the mean), grounding stats.
  2. core.Ring, the server's history: reset on a gap or a time that runs
     backwards; its slot inputs equal the trainer's for the same captures.
  3. The trainer's batched grounding stats and decode equal core's.
  4. The head: output shapes, an all-empty history but the newest slot.
  5. The server family: its step passes the codec's reader unchanged, a
     malformed answer is parse_ok false, the identity is an allow-listed
     foreign one (aiotvla/<name>@<sha8>) and tools/todo.py's copy agrees.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from training.aiotvla import core                                   # noqa: E402

SKIP = "SKIP"


def _torch():
    try:
        import torch
        return torch
    except ImportError:
        return None


def _track(n=40, dt=0.1, v=(2.0, 1.0, 0.0), yaw0=0.3, yaw_rate=0.2):
    t = np.arange(n) * dt + 100.0
    poses = np.zeros((n, 4))
    for i in range(n):
        poses[i] = (v[0] * i * dt, v[1] * i * dt, 8.0 + v[2] * i * dt, yaw0 + yaw_rate * i * dt)
    return t, poses


# ------------------------------------------------------------------- 1. core

def test_history_slots_pick_the_nearest_past_capture():
    t, _ = _track()
    idx = core.history_indices(t, 30)
    assert idx[0] == 30
    for off, j in zip(core.HIST_OFFSETS_S, idx):
        assert j <= 30
        assert abs(t[j] - (t[30] - off)) <= core.HIST_TOL_S + 1e-9
    # the first capture has no past: every older slot is empty
    assert core.history_indices(t, 0) == [0] + [-1] * (len(core.HIST_OFFSETS_S) - 1)


def test_history_slot_is_empty_when_no_capture_is_in_tolerance():
    t = np.array([0.0, 0.1, 2.0, 2.1])           # a 1.9 s hole
    idx = core.history_indices(t, 3)
    want = dict(zip(core.HIST_OFFSETS_S, idx))
    assert want[1.0] == -1 and want[1.4] == -1   # t = 1.1 and 0.7: nothing within 0.15 s
    assert want[1.9] == 1                        # t = 0.2: the capture at 0.1
    assert want[2.5] == -1                       # t = -0.4: before the first capture


def test_velocity_is_a_backward_difference_only():
    t, poses = _track(v=(2.0, 1.0, 0.5))
    v, yr, ok = core.backward_velocity(t, poses, 20)
    assert ok and np.allclose(v, [2.0, 1.0, 0.5]) and math.isclose(yr, 0.2, rel_tol=1e-6)
    # changing every FUTURE pose changes nothing
    p2 = poses.copy()
    p2[21:] += 50.0
    v2, yr2, _ = core.backward_velocity(t, p2, 20)
    assert np.allclose(v, v2) and yr == yr2
    v0, yr0, ok0 = core.backward_velocity(t, poses, 1)
    assert not ok0 and not v0.any() and yr0 == 0.0


def test_proprio_is_in_the_newest_capture_body_frame():
    # newest capture heading east (yaw pi/2); an older one 3 m to its west
    t = np.array([0.0, 0.2, 0.4, 0.7])
    poses = np.array([[0.0, -3.0, 8.0, math.pi / 2]] * 3 + [[0.0, 0.0, 9.0, math.pi / 2]])
    vel, yr, _ = core.velocities(t, poses)
    pr = core.proprio(poses, vel, yr, core.history_indices(t, 3))
    f = {k: i for i, k in enumerate(core.PROPRIO_FIELDS)}
    slot = pr[2]                                  # 0.4 s back: the capture at t = 0.4
    assert math.isclose(slot[f["rel_fwd"]], -3.0, abs_tol=1e-9)   # behind
    assert math.isclose(slot[f["rel_right"]], 0.0, abs_tol=1e-9)
    assert math.isclose(slot[f["rel_up"]], -1.0, abs_tol=1e-9)
    # the newest capture flew 3 m east in 0.3 s: 10 m/s straight ahead
    assert math.isclose(pr[0][f["v_fwd"]], 10.0, rel_tol=1e-6)
    assert math.isclose(pr[0][f["v_right"]], 0.0, abs_tol=1e-9)
    assert pr[0][f["valid"]] == 1.0 and pr[-1][f["valid"]] == 0.0


def test_mirror_negates_exactly_the_right_handed_fields():
    t, poses = _track()
    vel, yr, _ = core.velocities(t, poses)
    pr = core.proprio(poses, vel, yr, core.history_indices(t, 30))
    m = core.mirror_proprio(pr)
    for k, name in enumerate(core.PROPRIO_FIELDS):
        if name in core.MIRROR_NEGATE:
            assert np.allclose(m[:, k], -pr[:, k]), name
        else:
            assert np.allclose(m[:, k], pr[:, k]), name
    assert np.allclose(core.mirror_proprio(m), pr)
    lab = {"fwd_m": 1.2, "down_m": -0.1, "dpsi_rad": 0.3}
    assert core.mirror_label(lab) == {"fwd_m": 1.2, "down_m": -0.1, "dpsi_rad": -0.3}


def test_soft_target_and_decode_round_trip():
    for ax in core.AXES:
        g = core.GRIDS[ax]
        cell = g[1] - g[0]
        for v in (g[0], g[len(g) // 3] + 0.3 * cell, 0.5 * (g[0] + g[-1]), g[-1]):
            p = core.soft_target(v, ax)
            assert math.isclose(float(p.sum()), 1.0, rel_tol=1e-5)
            assert abs(core.decode(p, ax) - v) <= 0.5 * cell, (ax, v)
        # outside the grid: clipped to its end, never extrapolated
        assert core.decode(core.soft_target(g[-1] + 10, ax), ax) <= g[-1] + 1e-9


def test_decode_of_two_peaks_is_a_peak_not_their_mean():
    p = 0.55 * core.soft_target(-0.4, "dpsi_rad") + 0.45 * core.soft_target(0.4, "dpsi_rad")
    v = core.decode(p, "dpsi_rad")
    assert abs(v - (-0.4)) < 0.03, v              # the mean would be about -0.04


def test_grounding_stats_and_mirror_map():
    s = core.GRID_SIDE
    g = np.full(core.N_PATCHES, -20.0)
    g[5 * s + 18] = 0.0                            # row 5, column 18
    st = core.grounding_stats(g)
    lin = np.linspace(-1, 1, s)
    assert abs(st[0] - lin[18]) < 0.01 and abs(st[1] - lin[5]) < 0.01
    assert math.isclose(st[2], 0.0) and math.isclose(st[3], 20.0)
    gm = core.mirror_map(g)
    assert gm[5 * s + (s - 1 - 18)] == 0.0
    a = np.zeros((core.N_PATCHES, 3))
    a[5 * s + 18, 1] = 1.0
    assert core.mirror_map(a)[5 * s + (s - 1 - 18), 1] == 1.0


# ------------------------------------------------------------------- 2. Ring

def test_ring_resets_on_a_gap_and_on_time_running_backwards():
    r = core.Ring()
    g, a = np.zeros(core.N_PATCHES), np.zeros((core.N_PATCHES, core.PCA_DIM))
    pose = {"x": 0.0, "y": 0.0, "up": 8.0, "yaw_rad": 0.0}
    assert r.push(10.0, pose, g, a) is None and r.push(10.1, pose, g, a) is None
    assert "gap" in r.push(13.0, pose, g, a) and len(r) == 1
    assert "backwards" in r.push(12.0, pose, g, a) and len(r) == 1
    for k in range(200):
        r.push(20.0 + 0.05 * k, pose, g, a)
    assert len(r) == core.RING_MAXLEN


def test_ring_slots_equal_the_trainers_inputs():
    t, poses = _track(n=35)
    rng = np.random.default_rng(0)
    G = rng.normal(-6, 2, (len(t), core.N_PATCHES)).astype(np.float32)
    A = rng.normal(0, 1, (len(t), core.N_PATCHES, core.PCA_DIM)).astype(np.float32)
    r = core.Ring()
    for i in range(len(t)):
        r.push(t[i], {"x": poses[i, 0], "y": poses[i, 1], "up": poses[i, 2],
                      "yaw_rad": poses[i, 3]}, G[i], A[i])
    s = r.slots()
    # the trainer's view of capture 34 (train.Data): core on the whole recording
    vel, yr, _ = core.velocities(t, poses)
    idx = core.history_indices(t, len(t) - 1)
    assert s["idx"] == idx
    assert np.allclose(s["proprio"], core.proprio(poses, vel, yr, idx), atol=1e-5)
    for k, j in enumerate(idx):
        assert np.array_equal(s["g"][k], G[j]) and np.array_equal(s["a"][k], A[j])
        assert np.allclose(s["stats"][k], core.grounding_stats(G[j]), atol=1e-5)


# ---------------------------------------------------------- 2b. command labels

def _write_record(d: Path, caps: list[dict]) -> None:
    import json
    with open(d / "captures.jsonl", "w", encoding="utf-8", newline="\n") as fh:
        for c in caps:
            fh.write(json.dumps(c) + "\n")


def test_command_labels_are_the_experts_asked_command_over_the_horizon():
    import tempfile
    from training.aiotvla.build_labels import run_samples
    d = Path(tempfile.mkdtemp())
    yaw = math.pi / 2                                    # heading east
    asked = {"vx": 0.0, "vy": 3.0, "vz_up": 0.4, "yaw_rate": -0.3}
    executed = {"vx": 0.0, "vy": 3.0, "vz_up": 0.4, "yaw_rate": 0.5}   # a DART offset
    box = {"det": [384.0, 216.0, 40.0, 20.0, 0.2, 768, 432], "t_box": 10.0}
    caps = [
        {"i": 0, "t_capture": 9.0, "pose": {"x": 0, "y": 0, "up": 8, "yaw_rad": yaw},
         "box": None, "tick": {"mode": "track", "cmd": asked, "asked": asked}},
        {"i": 1, "t_capture": 10.1, "pose": {"x": 0, "y": 0, "up": 8, "yaw_rad": yaw},
         "box": box, "tick": {"mode": "track", "cmd": executed, "asked": asked}},
        {"i": 2, "t_capture": 12.0, "pose": {"x": 0, "y": 0, "up": 8, "yaw_rad": yaw},
         "box": box, "tick": None},
    ]
    _write_record(d, caps)
    s, why = run_samples("r", d, np.ones(3, bool), {0: 0, 1: 1, 2: 2}, "train", 0.5)
    # capture 0: before the first fresh box; capture 2: no expert tick
    assert why == {"no_frame": 0, "no_tick": 1, "before_acquire": 1, "excluded_mode": 0,
                   "kept": 1}, why
    # an expert mode left out (--exclude-modes plan) is no sample
    s3, why3 = run_samples("r", d, np.ones(3, bool), {0: 0, 1: 1, 2: 2}, "train", 0.5,
                           exclude_modes=("track",))
    assert not s3 and why3["excluded_mode"] == 1, why3
    lab = s[0]["label_raw"]
    assert math.isclose(lab["fwd_m"], 1.5, abs_tol=1e-9)          # 3 m/s east, heading east
    assert math.isclose(lab["down_m"], -0.2, abs_tol=1e-9)
    assert math.isclose(lab["dpsi_rad"], -0.15, abs_tol=1e-9)     # ASKED, not the executed +0.5
    assert s[0]["executed_differs"] and s[0]["hint_kind"] == "fresh"
    assert abs(s[0]["box"]["bearing_deg"]) < 1e-9                  # box centred
    # a frame the recording lost is no sample
    s2, why2 = run_samples("r", d, np.array([True, False, True]), {0: 0, 1: 1, 2: 2}, "train", 0.5)
    assert not s2 and why2["no_frame"] == 1


def test_command_labels_never_ask_to_fly_backwards():
    import tempfile
    from training.aiotvla.build_labels import run_samples
    d = Path(tempfile.mkdtemp())
    back = {"vx": -2.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0}  # heading north, moving south
    box = {"det": [384.0, 216.0, 40.0, 20.0, 0.2, 768, 432], "t_box": 5.0}
    _write_record(d, [{"i": 0, "t_capture": 5.0, "pose": {"x": 0, "y": 0, "up": 8, "yaw_rad": 0.0},
                       "box": box, "tick": {"mode": "track", "cmd": back, "asked": back}}])
    s, _ = run_samples("r", d, np.ones(1, bool), {0: 0}, "train", 0.5)
    assert s[0]["label_raw"]["fwd_m"] == 0.0     # the step format has no reverse


def test_ring_box_features_follow_their_capture():
    """with_box (HeadConfig.n_stats 8): each slot's stats end with the box of
    the capture in that slot, as the trainer stacks B[h]; an empty slot is
    all zeros."""
    t, poses = _track(n=12)
    r = core.Ring(offsets=core.LONG_HIST_OFFSETS_S, with_box=True)
    g, a = np.zeros(core.N_PATCHES), np.zeros((core.N_PATCHES, core.PCA_DIM))
    boxes = [np.array([0.1 * i, 0.5, 0.02 * i, 0.03], np.float32) for i in range(len(t))]
    for i in range(len(t)):
        r.push(t[i], {"x": poses[i, 0], "y": poses[i, 1], "up": poses[i, 2],
                      "yaw_rad": poses[i, 3]}, g, a, boxes[i])
    s = r.slots()
    assert s["stats"].shape == (len(core.LONG_HIST_OFFSETS_S), 8)
    for k, j in enumerate(s["idx"]):
        if j >= 0:
            assert np.allclose(s["stats"][k, 4:], boxes[j])
        else:
            assert not s["stats"][k].any()
    assert core.Ring().slots.__func__ is core.Ring.slots      # default: 4 stats


# ------------------------------------------------------------- 3. the trainer

def test_batched_grounding_stats_and_decode_equal_core():
    torch = _torch()
    if torch is None:
        return SKIP
    from training.aiotvla.train import decode_batch, torch_grounding_stats
    rng = np.random.default_rng(1)
    g = rng.normal(-6, 2, (5, core.N_PATCHES)).astype(np.float32)
    got = torch_grounding_stats(torch, torch.from_numpy(g)).numpy()
    for k in range(5):
        assert np.allclose(got[k], core.grounding_stats(g[k]), atol=1e-4)
    logits = {ax: torch.from_numpy(rng.normal(0, 2, (4, len(core.GRIDS[ax]))).astype(np.float32))
              for ax in core.AXES}
    grids = {ax: torch.tensor(core.GRIDS[ax], dtype=torch.float32) for ax in core.AXES}
    out = decode_batch(torch, logits, grids).numpy()
    for b in range(4):
        for c, ax in enumerate(core.AXES):
            p = torch.softmax(logits[ax][b], 0).numpy()
            assert abs(out[b, c] - core.decode(p, ax)) < 1e-5


# ---------------------------------------------------------------- 4. the head

def test_head_shapes_and_an_empty_history():
    torch = _torch()
    if torch is None:
        return SKIP
    from training.aiotvla.model import AIOTVLAHead
    head = AIOTVLAHead().eval()
    k = len(core.HIST_OFFSETS_S)
    b = 3
    g = torch.randn(b, k, core.N_PATCHES)
    a = torch.randn(b, k, core.N_PATCHES, core.PCA_DIM)
    st = torch.randn(b, k, 4)
    pr = torch.randn(b, k, core.PROPRIO_DIM)
    valid = torch.zeros(b, k, dtype=torch.bool)
    valid[:, 0] = True
    with torch.no_grad():
        logits, aux = head(g, a, st, pr, valid)
    for ax in core.AXES:
        assert tuple(logits[ax].shape) == (b, len(core.GRIDS[ax]))
        assert torch.isfinite(logits[ax]).all()
    assert tuple(aux.shape) == (b, 2)
    # an empty slot's contents never reach the answer
    g2 = g.clone()
    g2[:, 3:] = 99.0
    with torch.no_grad():
        logits2, _ = head(g2, a, st, pr, valid)
    assert all(torch.allclose(logits[ax], logits2[ax], atol=1e-5) for ax in core.AXES)


# -------------------------------------------------------------- 5. the server

def _server_parts():
    import vla_server_foreign as F
    from guardrail import aerialvla_codec as C
    return F, C


def test_family_step_passes_the_codec_reader():
    F, C = _server_parts()
    fam = F.AIOTVLAFamily(horizon_s=0.5)
    nat = fam.parse({"raw": {"fwd_m": 1.23, "down_m": -0.05, "dpsi_rad": -0.31,
                             "visible_p": 0.9, "bearing_rad": -0.2}})
    assert nat["ok"]
    m = fam.map(nat["action"], F.intrinsics(256, 256, 90.0))
    st = F.encode_step(m["fwd_m"], m["down_m"], m["dpsi_rad"], land=m["land"])
    p = C.parse_action(st["text"], require_eos=True)
    assert p.parse_ok and not p.land
    flown = p.step.to_dict()
    for ax, v in (("fwd_m", 1.23), ("down_m", -0.05), ("dpsi_rad", -0.31)):
        lo, hi = C.RANGES[ax]
        assert abs(flown[ax] - v) <= 0.5 * (hi - lo) / C.MAX_BIN + 1e-9, ax
    assert m["geometry"]["horizon_s"] == 0.5 and m["geometry"]["visible_p"] == 0.9


def test_family_refuses_a_malformed_answer():
    F, _ = _server_parts()
    fam = F.AIOTVLAFamily(horizon_s=0.5)
    for raw in (None, {"fwd_m": 1.0, "down_m": 0.0}, {"fwd_m": float("nan"), "down_m": 0.0,
                                                       "dpsi_rad": 0.0}, [1, 2, 3]):
        nat = fam.parse({"raw": raw})
        assert not nat["ok"] and nat["reason"], raw
    try:
        F.AIOTVLAFamily(horizon_s=0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("a zero horizon was accepted")


def test_family_inputs_carry_the_object_words_time_and_pose():
    F, _ = _server_parts()
    fam = F.AIOTVLAFamily(horizon_s=0.5)
    req = {"object": "a red car", "t_capture": 12.5,
           "pose": {"x": 1.0, "y": 2.0, "up": 8.0, "yaw_rad": 0.1}, "phrase": "straight ahead "}
    ins = fam.inputs(req, F.intrinsics(256, 256, 90.0), [])
    assert ins == {"instruction": "a red car", "t_capture": 12.5,
                   "pose": {"x": 1.0, "y": 2.0, "up": 8.0, "yaw_rad": 0.1}}


def test_identity_is_an_allow_listed_foreign_family():
    F, C = _server_parts()
    import todo as T
    assert "aiotvla" in C.FOREIGN_FAMILIES and T.FOREIGN_FAMILIES == C.FOREIGN_FAMILIES
    ident = F.foreign_identity("aiotvla", name="v0", folder="v0", revision=None,
                               digest="ab" * 32, precision="fp16", attn="sdpa",
                               front_source="vla", facts={"horizon_s": 0.5})
    blk = C.foreign_block(ident)
    assert blk["family"] == "aiotvla" and blk["front_source"] == "vla"
    assert ident["id"] == "aiotvla/v0@abababab"
    assert C.foreign_pilot_id(ident["id"]) == "aiotvla:v0"
    assert C.foreign_family("aiotvla:v0") == "aiotvla"
    assert F.weight_files("aiotvla", "D:/m/v0") == [Path("D:/m/v0") / "head.pt"]


def test_the_former_name_still_classifies_but_is_not_served():
    """Renamed NTUT-VLA -> AIOT-VLA on 30 Oct: the 29-30 Oct flights recorded
    "ntutvla:<name>", which must still count as this model's VLA flights; no
    server may announce the old name."""
    F, C = _server_parts()
    import todo as T
    assert C.foreign_family("ntutvla:v2") == "ntutvla" and "ntutvla" in T.FOREIGN_FAMILIES
    assert "ntutvla" in C.LEGACY_FOREIGN_FAMILIES and "ntutvla" not in F.FAMILIES
    assert "aiotvla" in F.FAMILIES
    try:
        F.foreign_identity("ntutvla", name="v2", folder="v2", revision=None, digest="ab" * 32,
                           precision="fp16", attn="sdpa", front_source="vla")
    except ValueError:
        return
    raise AssertionError("a server identity under the former name was built")


def test_the_server_answers_a_scripted_aiotvla_step():
    F, C = _server_parts()
    import base64

    class Scripted:
        precision, attn, merged = "fp16", "sdpa", False

        def generate(self, image, inputs):
            return {"raw": {"fwd_m": 0.8, "down_m": 0.0, "dpsi_rad": 0.12, "visible_p": 0.7,
                            "bearing_rad": 0.1, "slots_valid": 1, "history_reset": None},
                    "model_s": 0.02, "pre_s": 0.01, "gen_s": 0.01}

    ident = F.foreign_identity("aiotvla", name="v0", folder="v0", revision=None,
                               digest="cd" * 32, precision="fp16", attn="sdpa",
                               front_source="vla")
    srv = F.ForeignServer(F.AIOTVLAFamily(horizon_s=0.5), Scripted(), ident, None,
                          decode_front=lambda req: type("I", (), {"size": (256, 256)})(),
                          hash_fn=lambda img: "0" * 64, prepare=lambda img: (img, {}))
    req = {"v": 2, "op": "infer", "seq": 3, "t_capture": 50.0,
           "pose": {"x": 0.0, "y": 0.0, "up": 8.0, "yaw_rad": 0.0},
           "front_jpg_b64": base64.b64encode(b"x").decode(), "down_jpg_b64": None,
           "phrase": "straight ahead ", "hint_source": "detector", "object": "a red car",
           "csp_text": None, "csp_mode": "off"}
    resp, stop, extra = srv.handle(req)
    assert resp["ok"] and resp["parse_ok"] and not stop, resp.get("error")
    want = F.encode_step(0.8, 0.0, 0.12)["text"]
    assert resp["raw_text"] == want
    assert resp["mapping"]["rule"] == "aiotvla_step/1"
    assert resp["native"]["family"] == "aiotvla"
    assert resp["native"]["ignored"]["phrase"] == "straight ahead "


# ------------------------------------------------- 6. v7: lateral and memory

ASPECT = 432 / 768


def _box_of(fwd, right, up=8.0, h=0.04, hfov=90.0, pitch=-20.0):
    """The box (cx, cy, w, h) whose bottom-centre pixel images the ground
    point (fwd, right) from a camera `up` m above it: box_measurement's
    inverse, written from the projection side."""
    k = math.tan(math.radians(hfov) / 2.0)
    th = -math.radians(pitch)
    z = fwd * math.cos(th) + up * math.sin(th)
    y = -fwd * math.sin(th) + up * math.cos(th)
    cx = 0.5 + (right / z) / (2.0 * k)
    bottom = 0.5 + (y / z) / (2.0 * k * ASPECT)
    return (cx, bottom - h / 2.0, 0.05, h)


def test_box_measurement_is_the_pinhole_ground_point():
    hfov, pitch = core.CAMERAS["detector"]
    # the bottom edge on the optical axis: 8 m / tan(20 deg) ahead
    m = core.box_measurement((0.5, 0.48, 0.05, 0.04), 8.0, ASPECT, hfov, pitch)
    assert math.isclose(m[0], 8.0 / math.tan(math.radians(20.0)), rel_tol=1e-9)
    assert abs(m[1]) < 1e-9
    for fwd, right in ((15.0, 0.0), (22.0, -6.5), (40.0, 9.0), (6.0, 1.0)):
        got = core.box_measurement(_box_of(fwd, right), 8.0, ASPECT, hfov, pitch)
        assert math.isclose(got[0], fwd, abs_tol=1e-6) and math.isclose(got[1], right,
                                                                       abs_tol=1e-6), got
    # a ray at or above the horizon, a range outside MEM_RANGE_M, no altitude: no measurement
    assert core.box_measurement((0.5, 0.05, 0.05, 0.04), 8.0, ASPECT, hfov, pitch) is None
    assert core.box_measurement(_box_of(90.0, 0.0), 8.0, ASPECT, hfov, pitch) is None
    assert core.box_measurement(_box_of(15.0, 0.0), 0.0, ASPECT, hfov, pitch) is None
    assert core.box_measurement(None, 8.0, ASPECT, hfov, pitch) is None


def _mem():
    hfov, pitch = core.CAMERAS["detector"]
    return core.TargetMemory(ASPECT, hfov, pitch)


def _field(f, name):
    return float(f[core.MEM_FIELDS.index(name)])


def test_memory_confirms_a_track_only_with_two_agreeing_boxes():
    mem = _mem()
    pose = (0.0, 0.0, 8.0, 0.0)
    f = mem.step(0.0, pose, _box_of(20.0, 2.0), -1.0, 0.5)
    assert _field(f, "valid") == 0.0 and _field(f, "meas") == 1.0      # a candidate only
    # a second box far from the first does not confirm; it is the new candidate
    f = mem.step(0.13, pose, _box_of(45.0, -10.0), -1.0, 0.5)
    assert _field(f, "valid") == 0.0
    f = mem.step(0.26, pose, _box_of(45.5, -10.0), -1.0, 0.5)
    assert _field(f, "valid") == 1.0 and _field(f, "accepted") == 1.0
    assert math.isclose(_field(f, "fwd") * 30.0, 45.5, abs_tol=0.5)
    # a box under the seed logit never starts a track
    mem2 = _mem()
    for i in range(4):
        f = mem2.step(0.13 * i, pose, _box_of(20.0, 0.0), core.MEM_SEED_LOGIT - 0.2, 0.5)
    assert _field(f, "valid") == 0.0 and _field(f, "meas") == 1.0


def test_memory_tracks_gates_an_outlier_and_reseeds_on_agreeing_ones():
    mem = _mem()
    pose = (0.0, 0.0, 8.0, 0.0)
    t = 0.0
    for i in range(40):                                  # a car at 3 m/s ahead
        t = 0.13 * i
        f = mem.step(t, pose, _box_of(20.0 + 3.0 * t, 2.0), -1.0, 0.5)
    assert _field(f, "valid") == 1.0
    assert math.isclose(_field(f, "fwd") * 30.0, 20.0 + 3.0 * t, abs_tol=1.5)
    assert math.isclose(_field(f, "v_fwd") * 5.0, 3.0, abs_tol=0.5)
    assert math.isclose(_field(f, "range_rate") * 5.0, 3.0, abs_tol=0.6)
    # one box on something else: gated out, the track kept
    t += 0.13
    f = mem.step(t, pose, _box_of(12.0, -15.0), -1.0, 0.5)
    assert _field(f, "accepted") == 0.0 and _field(f, "valid") == 1.0
    assert _field(f, "fwd") * 30.0 > 30.0
    # MEM_RESEED_N agreeing boxes elsewhere: the track moves there
    for _ in range(core.MEM_RESEED_N - 1):
        t += 0.13
        f = mem.step(t, pose, _box_of(12.0, -15.0), -1.0, 0.5)
    assert _field(f, "accepted") == 1.0
    assert math.isclose(_field(f, "fwd") * 30.0, 12.0, abs_tol=1.0)
    assert math.isclose(_field(f, "right") * 30.0, -15.0, abs_tol=1.0)
    # unseen: predicted on, then dropped after MEM_MAX_AGE_S
    f = mem.step(t + 3.0, pose, None, None)
    assert _field(f, "valid") == 1.0 and math.isclose(_field(f, "age"), 3.0 / core.MEM_MAX_AGE_S,
                                                      abs_tol=1e-6)
    f = mem.step(t + core.MEM_MAX_AGE_S + 0.5, pose, None, None)
    assert _field(f, "valid") == 0.0


def test_memory_needs_the_contrast_and_reads_in_the_body_frame():
    mem = _mem()
    yaw = math.pi / 2                                    # heading east
    pose = (100.0, 50.0, 8.0, yaw)
    for i in range(3):
        f = mem.step(0.13 * i, pose, _box_of(20.0, 3.0), -1.0, core.MEM_CONTRAST_MIN - 0.1)
    assert _field(f, "meas") == 0.0 and _field(f, "valid") == 0.0   # beaten by a contrast phrase
    for i in range(3, 6):
        f = mem.step(0.13 * i, pose, _box_of(20.0, 3.0), -1.0, 0.4)
    assert _field(f, "valid") == 1.0
    assert math.isclose(_field(f, "fwd") * 30.0, 20.0, abs_tol=0.3)
    assert math.isclose(_field(f, "right") * 30.0, 3.0, abs_tol=0.3)
    assert math.isclose(_field(f, "bearing"), math.atan2(3.0, 20.0), abs_tol=0.02)
    assert math.isclose(_field(f, "contrast"), 0.4, abs_tol=1e-6)
    # in the world: 20 m east and 3 m south of (100, 50)
    assert np.allclose(mem.x[:2], (97.0, 70.0), atol=0.3)


def test_mirror_memory_negates_exactly_the_right_handed_fields():
    m = np.arange(1, core.MEM_DIM + 1, dtype=np.float32)
    mm = core.mirror_memory(m)
    for k, f in enumerate(core.MEM_FIELDS):
        assert mm[k] == (-m[k] if f in core.MEM_NEGATE else m[k]), f
    assert np.array_equal(core.mirror_memory(mm), m)
    assert set(core.MEM_NEGATE) <= set(core.MEM_FIELDS)


def test_run_memory_starts_at_the_first_sample_and_resets_on_a_gap():
    hfov, pitch = core.CAMERAS["detector"]
    n = 16
    t = np.arange(n) * 0.13 + 100.0
    t[10:] += 3.0                                        # a 3 s gap before row 10
    poses = np.tile([0.0, 0.0, 8.0, 0.0], (n, 1))
    boxes = np.array([_box_of(20.0, 0.0)] * n)
    logits = np.full(n, -1.0)
    ok = np.ones(n, bool)
    ok[6] = False                                        # a lost frame: no measurement
    M = core.run_memory(t, poses, boxes, logits, ok, 3, ASPECT, hfov, pitch, np.full(n, 0.5))
    v = M[:, core.MEM_FIELDS.index("valid")]
    assert not M[:3].any()                               # before the first sample
    assert v[3] == 0.0 and v[4] == 1.0                   # a candidate, then confirmed
    assert v[6] == 1.0 and M[6, core.MEM_FIELDS.index("meas")] == 0.0
    assert v[10] == 0.0 and v[11] == 1.0                 # reset by the gap, as core.Ring


def test_grounding_contrast_is_the_phrase_against_its_contrast_phrases():
    g = np.full((3, core.N_PATCHES), -6.0)
    g[0, 100], g[1, 100], g[2, 100] = -1.0, -1.5, -0.6   # "white" beats "red" at red's peak
    assert math.isclose(core.grounding_contrast(g, 0, (1, 2)), -0.4, abs_tol=1e-9)
    assert math.isclose(core.grounding_contrast(g, 0, (1,)), 0.5, abs_tol=1e-9)
    assert core.grounding_contrast(g, 0, ()) is None
    assert all(p in core.VOCAB for k, v in core.CONTRAST.items() for p in (k,) + tuple(v))


def test_the_memory_camera_and_constants_are_the_codecs_and_json():
    import json
    from guardrail import aerialvla_codec as C
    for src, (hfov, pitch) in core.CAMERAS.items():
        assert C.FRONT_SOURCES[src]["hfov_deg"] == hfov
        assert C.FRONT_SOURCES[src]["pitch_deg"] == pitch
    mc = core.memory_constants()
    assert json.loads(json.dumps(mc)) == mc
    assert mc["fields"] == list(core.MEM_FIELDS) and "MEM_GATE_SIGMA" in mc


def test_a_lateral_head_with_a_memory_token():
    torch = _torch()
    if torch is None:
        return SKIP
    from training.aiotvla.model import AIOTVLAHead, HeadConfig
    cfg = HeadConfig(n_slots=len(core.LONG_HIST_OFFSETS_S), n_stats=8, axes=core.LATERAL_AXES,
                     n_mem=core.MEM_DIM)
    assert HeadConfig(**cfg.to_dict()) == cfg
    head = AIOTVLAHead(cfg).eval()
    b, k = 2, cfg.n_slots
    x = (torch.randn(b, k, core.N_PATCHES), torch.randn(b, k, core.N_PATCHES, core.PCA_DIM),
         torch.randn(b, k, 8), torch.randn(b, k, core.PROPRIO_DIM),
         torch.ones(b, k, dtype=torch.bool))
    mem = torch.randn(b, core.MEM_DIM)
    with torch.no_grad():
        logits, aux = head(*x, mem)
        logits2, _ = head(*x, mem + 1.0)
    assert set(logits) == set(core.LATERAL_AXES)
    assert tuple(logits["right_m"].shape) == (b, len(core.GRIDS["right_m"]))
    assert not torch.allclose(logits["fwd_m"], logits2["fwd_m"])       # the memory is read
    try:
        head(*x)
    except ValueError:
        pass
    else:
        raise AssertionError("a memory head answered without its memory")
    # a v0-v6 config (no axes, no n_mem) is three axes and no memory token
    old = HeadConfig(**{k2: v for k2, v in cfg.to_dict().items() if k2 not in ("axes", "n_mem")})
    assert old.axes == core.AXES and old.n_mem == 0


def test_vocab_only_grows_and_red_cars_contrast_is_v7s():
    """A cache's row of a phrase is its index in that cache's vocab, and a v7
    checkpoint grounds the contrast phrases it recorded: VOCAB is appended
    to, and "a red car" keeps the two contrast phrases v7 was trained with."""
    assert core.VOCAB[:4] == ("a red car", "a white car", "a blue car", "a person")
    assert len(set(core.VOCAB)) == len(core.VOCAB)
    assert core.CONTRAST["a red car"] == ("a white car", "a blue car")
    for k, v in core.CONTRAST.items():
        assert k in core.VOCAB and k not in v and set(v) <= set(core.VOCAB), k


def test_a_head_with_a_text_token():
    torch = _torch()
    if torch is None:
        return SKIP
    from training.aiotvla.model import AIOTVLAHead, HeadConfig
    cfg = HeadConfig(n_slots=2, n_stats=8, axes=core.LATERAL_AXES, n_mem=core.MEM_DIM,
                     n_text=512)
    head = AIOTVLAHead(cfg).eval()
    b = 2
    x = (torch.randn(b, 2, core.N_PATCHES), torch.randn(b, 2, core.N_PATCHES, core.PCA_DIM),
         torch.randn(b, 2, 8), torch.randn(b, 2, core.PROPRIO_DIM),
         torch.ones(b, 2, dtype=torch.bool), torch.randn(b, core.MEM_DIM))
    t1 = torch.nn.functional.normalize(torch.randn(b, 512), dim=1)
    t2 = torch.nn.functional.normalize(torch.randn(b, 512), dim=1)
    with torch.no_grad():
        l1, _ = head(*x, t1)
        l2, _ = head(*x, t2)
    assert not torch.allclose(l1["fwd_m"], l2["fwd_m"])              # the words are read
    try:
        head(*x)
    except ValueError:
        pass
    else:
        raise AssertionError("a text head answered without its text")
    # without a memory token: the text goes in the txt slot, mem None
    cfg2 = HeadConfig(n_slots=2, n_stats=8, n_text=512)
    with torch.no_grad():
        out, _ = AIOTVLAHead(cfg2).eval()(*x[:5], None, t1)
    assert set(out) == set(core.AXES)
    assert HeadConfig().n_text == 0


def test_command_labels_carry_the_lateral_component():
    import tempfile
    from training.aiotvla.build_labels import run_samples
    d = Path(tempfile.mkdtemp())
    yaw = 0.0                                            # heading north: right is east (+y)
    asked = {"vx": 2.0, "vy": -1.2, "vz_up": 0.0, "yaw_rate": 0.0}   # sliding left
    box = {"det": [384.0, 216.0, 40.0, 20.0, 0.2, 768, 432], "t_box": 5.0}
    _write_record(d, [{"i": 0, "t_capture": 5.0, "pose": {"x": 0, "y": 0, "up": 8, "yaw_rad": yaw},
                       "box": box, "tick": {"mode": "track", "cmd": asked, "asked": asked}}])
    s, _ = run_samples("r", d, np.ones(1, bool), {0: 0}, "train", 0.5)
    lab = s[0]["label_raw"]
    assert math.isclose(lab["fwd_m"], 1.0, abs_tol=1e-9)
    assert math.isclose(lab["right_m"], -0.6, abs_tol=1e-9)
    assert core.mirror_label(lab)["right_m"] == 0.6 and core.mirror_label(lab)["fwd_m"] == 1.0


def test_a_sample_carries_the_experts_own_estimate():
    import json
    import tempfile
    from training.aiotvla.build_labels import run_samples
    d = Path(tempfile.mkdtemp())
    rec = d / "vla_record"
    rec.mkdir()
    asked = {"vx": 2.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0}
    box = {"det": [384.0, 216.0, 40.0, 20.0, 0.2, 768, 432], "t_box": 5.0}
    _write_record(rec, [
        {"i": 0, "t_capture": 5.0, "pose": {"x": 0, "y": 0, "up": 8, "yaw_rad": 0.0}, "box": box,
         "tick": {"tick": 41, "mode": "track", "cmd": asked, "asked": asked}},
        {"i": 1, "t_capture": 5.1, "pose": {"x": 0, "y": 0, "up": 8, "yaw_rad": 0.0}, "box": box,
         "tick": {"tick": 42, "mode": "track", "cmd": asked, "asked": asked}}])
    with open(d / "flight_log.jsonl", "w", encoding="utf-8") as fh:
        fh.write(json.dumps({"tick": 41, "est_xy": [20.0, 1.0], "est": {"served": True}}) + "\n")
        fh.write(json.dumps({"tick": 42, "est_xy": [21.0, 1.0], "est": {"served": False}}) + "\n")
    s, _ = run_samples("r", rec, np.ones(2, bool), {0: 0, 1: 1}, "train", 0.5)
    assert s[0]["expert_est"] == [20.0, 1.0]
    assert s[1]["expert_est"] is None                  # an estimate not served is none


def test_a_memory_off_the_experts_subject_is_found():
    from training.aiotvla.train import memory_disagrees
    m = np.zeros(core.MEM_DIM, np.float32)
    m[core.MEM_FIELDS.index("valid")] = 1.0
    m[core.MEM_FIELDS.index("fwd")] = 20.0 / 30.0
    m[core.MEM_FIELDS.index("right")] = 3.0 / 30.0
    pose = (100.0, 50.0, 8.0, math.pi / 2)             # heading east: (20 fwd, 3 right) = (97, 70)
    assert not memory_disagrees(m, pose, (97.5, 70.0), 15.0)
    assert memory_disagrees(m, pose, (80.0, 70.0), 15.0)
    m[core.MEM_FIELDS.index("valid")] = 0.0            # no track: nothing to disagree with
    assert not memory_disagrees(m, pose, (0.0, 0.0), 15.0)


def test_the_family_maps_right_m_into_a_lateral_step():
    F, C = _server_parts()
    fam = F.AIOTVLAFamily(horizon_s=0.5)
    nat = fam.parse({"raw": {"fwd_m": 0.9, "down_m": 0.0, "dpsi_rad": -0.03, "right_m": 0.7,
                             "memory": {"valid": True}}})
    m = fam.map(nat["action"], F.intrinsics(768, 432, 90.0))
    assert m["right_m"] == 0.7 and m["geometry"]["memory"] == {"valid": True}
    st = F.encode_step(m["fwd_m"], m["down_m"], m["dpsi_rad"], right_m=m["right_m"])
    p = C.parse_action(st["text"], require_eos=True, lateral=True)
    assert p.parse_ok and p.step.lateral
    # a three-axis model's answer is a plain step, as before
    nat3 = fam.parse({"raw": {"fwd_m": 0.9, "down_m": 0.0, "dpsi_rad": -0.03}})
    assert "right_m" not in fam.map(nat3["action"], F.intrinsics(768, 432, 90.0))


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
