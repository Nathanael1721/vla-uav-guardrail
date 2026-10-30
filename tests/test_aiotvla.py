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
