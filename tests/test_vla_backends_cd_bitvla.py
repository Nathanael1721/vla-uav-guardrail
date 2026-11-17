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
     family through the server's protocol core, the client's strict reader
     and the StepTracker; every one must come out as a finite Action4D
     (vx, vy, vz_up, yaw_rate). The figure is printed.
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
                                RemoteVLA, Step, StepTracker, check_identity)

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


def _action4d(step_wire: dict, t0: float = 100.0) -> Action4D:
    """A decoded step through the StepTracker, one tick after it was taken:
    what the rail hands the Shield."""
    tr = StepTracker()
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

def _random_native(family: str, rng: random.Random):
    if family == "cognitivedrone":
        return [_tok(rng.randrange(0, 255)) for _ in range(4)]
    return [[rng.uniform(-1.5, 1.5) for _ in range(7)] for _ in range(8)]


def conformance(family: str, n: int = SWEEP_N, seed: int = 20261117) -> dict:
    """n seeded random native outputs of `family` through the server's
    protocol core, the client's strict reader and the StepTracker: how many
    came out as a finite 4-D Action4D."""
    rng = random.Random(seed)
    fam = _cd_family(4) if family == "cognitivedrone" else _bit_family()
    outs = [_random_native(family, rng) for _ in range(n)]
    core, _ = _core(fam, outs)
    ok, clipped = 0, 0
    for i in range(n):
        resp, _, _ = core.handle(_req(seq=i + 1, t=100.0 + i))
        if not (resp["ok"] and resp["parse_ok"] and len(resp["bins"]) == 4):
            continue
        p = C.parse_action(resp["raw_text"], require_eos=True, lateral=True)
        if not (p.parse_ok and p.step.to_dict() == resp["step"]):
            continue
        clipped += bool(resp["mapping"]["clipped"])
        if _finite4(_action4d(resp["step"])):
            ok += 1
    return {"family": family, "n": n, "action4d_finite": ok, "clipped_recorded": clipped}


def test_every_native_output_of_both_families_becomes_a_finite_4_d_action():
    for fam in ("cognitivedrone", "bitvla"):
        r = conformance(fam)
        print(f"      conformance {fam}: {r['action4d_finite']}/{r['n']} native outputs -> "
              f"finite Action4D(vx, vy, vz_up, yaw_rate); {r['clipped_recorded']} clipped "
              f"to the codec's ranges, recorded")
        assert r["action4d_finite"] == r["n"] == SWEEP_N, r


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
                                  lambda cap: Hint("straight ahead ", "detector"), "a red car")
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
            _stats(d, 4)
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
        assert [p.name for p in F.weight_files("bitvla", w)][-2:] == [
            "dataset_statistics.json", "action_head--10000_checkpoint.pt"]
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
