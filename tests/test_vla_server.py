"""
demo/vla_server_v2.py - VLA server protocol v2, with a scripted model.

Run either way:
    pytest tests/test_vla_server.py -v
    python tests/test_vla_server.py

No test here loads a model or touches a GPU. The protocol core (V2Server) is
driven with a scripted stand-in and, where Pillow is absent (every CI job),
with stand-in frame decoding, so what is tested on CI is the protocol itself:

  1. The response shape is the brief's, key for key, on success, on a
     malformed model output and on every refusal.
  2. A request is refused - before the model runs - for an unknown field, a
     field that could carry where the target is (by name: target, goal,
     object_position, xy, lat, lon, anything with "target" or "goal"), a
     phrase outside the 8 vocabulary strings, a hint source that contradicts
     its phrase, a CSP arm without its text or the off arm with one.
  3. parse_ok=false => step null, land false (the client holds zero); LAND =>
     land true with the decoded step.
  4. It works over TCP: many requests on one connection, newline-delimited
     JSON, shutdown stops the server.
  5. The weights digest is computed once and served from its sidecar until a
     file changes; the identity names the adapter and base revisions.
  6. The legacy file mode is still there, and `--v2` reaches the new server.
  7. The CLI's default model configuration is the measured serving
     recommendation (docs/data/aerialvla_serving_4090.json).
"""
from __future__ import annotations

import hashlib
import inspect
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

import vla_server_v2 as S                                            # noqa: E402
from guardrail import aerialvla_codec as C                           # noqa: E402

SKIP = "SKIP"
RESPONSE_KEYS = {"v", "seq", "ok", "error", "raw_text", "bins", "step", "land", "parse_ok",
                 "latency_s", "t_capture", "pose", "prompt_sha256", "image_sha256", "model"}
IDENTITY = {"id": "aerialvla-lora/aero_vla@196f2f32", "base": "openvla-7b@47a0ec7f",
            "weights_sha256": "ab" * 32, "precision": "nf4", "attn": "sdpa", "merged": True}


def _fake_decode(req):
    """Stand-in for Pillow decoding: the 'image' is the base64 text itself."""
    for k in ("front_jpg_b64", "down_jpg_b64"):
        if req[k] == "BAD":
            raise C.CodecError(C.ERR_BAD_IMAGE, f"{k}: not a readable image")
    return req["front_jpg_b64"], req["down_jpg_b64"]


def _fake_mosaic(front, down, missing_down="black"):
    if down is None and missing_down == "refuse":
        raise C.CodecError(C.ERR_BAD_IMAGE, "no down frame, and missing_down='refuse'")
    return (front, down), {"down_missing": down is None,
                           "down_fill": "black" if down is None else None}


def _fake_hash(mosaic):
    return hashlib.sha256(json.dumps(mosaic).encode()).hexdigest()


def _core(outputs=("96 48 49</s>",), log=None, **kw):
    model = S.ScriptedModel(list(outputs))
    core = S.V2Server(model, IDENTITY, log, decode_frames=_fake_decode,
                      mosaic_fn=_fake_mosaic, hash_fn=_fake_hash, **kw)
    return core, model


def _req(**over):
    r = {"v": 2, "op": "infer", "seq": 7, "t_capture": 123.25,
         "pose": {"x": 1.0, "y": -2.0, "up": 12.0, "yaw_rad": 0.5},
         "front_jpg_b64": "FRONT", "down_jpg_b64": "DOWN",
         "phrase": "forward-left ", "hint_source": "detector", "object": "a parked car",
         "csp_text": None, "csp_mode": "off"}
    r.update(over)
    return r


# ------------------------------------------------------------------ the shape

def test_an_infer_answer_has_exactly_the_briefs_keys_and_a_step_in_metres():
    core, model = _core(["96 48 49</s>"])
    resp, stop, extra = core.handle(_req())
    assert not stop and set(resp) == RESPONSE_KEYS, set(resp) ^ RESPONSE_KEYS
    assert resp["v"] == 2 and resp["ok"] is True and resp["error"] is None and resp["seq"] == 7
    assert resp["parse_ok"] is True and resp["land"] is False and resp["bins"] == [96, 48, 49]
    assert resp["step"] == C.bins_to_step((96, 48, 49)).to_dict()
    assert resp["t_capture"] == 123.25 and resp["pose"] == _req()["pose"]
    prompt = C.build_prompt("forward-left ", "a parked car")
    assert resp["prompt_sha256"] == C.prompt_sha256(prompt) and extra["prompt"] == prompt
    assert model.calls[0][1] == prompt, "the model ran a different prompt than the one hashed"
    assert resp["image_sha256"] == _fake_hash(("FRONT", "DOWN"))
    assert resp["model"] == IDENTITY and isinstance(resp["latency_s"], float)
    assert resp["raw_text"] == "96 48 49</s>"


def test_a_malformed_output_is_parse_fail_with_no_step_and_no_land():
    for text in ("00 49 494 49</s>", "00 49 4949</s>", "00 49 49 L 49</s>", "96 48 49"):
        core, _ = _core([text])
        resp, _, extra = core.handle(_req())
        assert resp["ok"] is True and resp["parse_ok"] is False, (text, resp)
        assert resp["step"] is None and resp["bins"] is None and resp["land"] is False, resp
        assert resp["raw_text"] == text and extra["parse_reason"]
        assert core.counts["parse_fail"] == 1


def test_land_comes_back_with_its_decoded_step():
    core, _ = _core(["00 49 49 LAND</s>"])
    resp, _, _ = core.handle(_req(phrase="", hint_source="goal"))
    assert resp["parse_ok"] and resp["land"] is True
    assert resp["step"] == {"fwd_m": 0.0, "down_m": 0.0, "dpsi_rad": 0.0}


def test_hello_names_the_model_and_shutdown_stops():
    core, _ = _core(hello_extra={"max_new_tokens": 12})
    hello, stop, _ = core.handle({"v": 2, "op": "hello"})
    assert hello["ok"] and not stop and hello["v"] == 2
    assert {k: hello["model"][k] for k in IDENTITY} == IDENTITY
    assert hello["model"]["max_new_tokens"] == 12
    bye, stop, _ = core.handle({"v": 2, "op": "shutdown"})
    assert bye == {"v": 2, "ok": True} and stop


# --------------------------------------------------------------- the refusals

def _refused(core, req, code):
    resp, stop, _ = core.handle(req)
    assert not stop and resp["ok"] is False, (req, resp)
    assert str(resp["error"]).split(":", 1)[0] == code, (code, resp["error"])
    return resp


def test_unknown_fields_are_refused_and_the_model_never_runs():
    core, model = _core()
    for extra in ({"temperature": 0.0}, {"prompt": "<image>\nFly ..."}, {"seq2": 1}):
        resp = _refused(core, _req(**extra), C.ERR_UNKNOWN_FIELD)
        assert set(resp) == RESPONSE_KEYS and resp["seq"] == 7 and resp["model"] == IDENTITY
        assert resp["step"] is None and resp["parse_ok"] is False and resp["land"] is False
    _refused(core, _req(pose={"x": 1.0, "y": 2.0, "up": 3.0, "yaw_rad": 0.0, "z": 1.0}),
             C.ERR_UNKNOWN_FIELD)
    _refused(core, {"v": 2, "op": "hello", "seq": 1}, C.ERR_UNKNOWN_FIELD)
    assert model.calls == [], "the model ran for a refused request"


def test_any_field_that_could_carry_the_target_is_refused_by_name():
    core, model = _core()
    for key in ("target", "goal", "object_position", "xy", "lat", "lon", "target_xy",
                "goal_pos", "Target", "my_goal", "latitude", "tgt"):
        _refused(core, _req(**{key: [38.0, 26.0]}), C.ERR_COORDINATE_FIELD)
    pose = dict(_req()["pose"], target=[1, 2])
    _refused(core, _req(pose=pose), C.ERR_COORDINATE_FIELD)
    _refused(core, {"v": 2, "op": "hello", "goal": [1, 2]}, C.ERR_COORDINATE_FIELD)
    assert model.calls == []


def test_the_phrase_must_be_one_of_the_eight_strings_exactly():
    core, model = _core()
    for bad in ("straight ahead", "Straight ahead ", "go left", "forward right ", None, 0):
        _refused(core, _req(phrase=bad), C.ERR_BAD_PHRASE)
    for phrase in C.PHRASES:
        resp, _, _ = core.handle(_req(phrase=phrase, hint_source="goal"))
        assert resp["ok"], (phrase, resp["error"])
    assert len(model.calls) == len(C.PHRASES)


def test_the_hint_source_must_agree_with_the_phrase():
    core, _ = _core()
    _refused(core, _req(phrase="forward-left ", hint_source="none"), C.ERR_BAD_HINT)
    _refused(core, _req(phrase="", hint_source="detector"), C.ERR_BAD_HINT)
    _refused(core, _req(hint_source="language"), C.ERR_BAD_HINT)
    assert core.handle(_req(phrase="", hint_source="none"))[0]["ok"]
    assert core.handle(_req(phrase="", hint_source="goal"))[0]["ok"]


def test_the_csp_arms_need_their_text_and_off_needs_none():
    core, model = _core()
    _refused(core, _req(csp_mode="off", csp_text="Stay low."), C.ERR_BAD_CSP)
    _refused(core, _req(csp_mode="csp", csp_text=None), C.ERR_BAD_CSP)
    _refused(core, _req(csp_mode="csp", csp_text="  "), C.ERR_BAD_CSP)
    _refused(core, _req(csp_mode="neutral", csp_text="a\nb"), C.ERR_BAD_CSP)
    _refused(core, _req(csp_mode="on", csp_text="x"), C.ERR_BAD_CSP)
    resp, _, extra = core.handle(_req(csp_mode="csp", csp_text="Stay below 14 m."))
    assert resp["ok"] and extra["prompt"].startswith("<image>\nStay below 14 m. Fly ")
    off, _, _ = core.handle(_req())
    assert off["prompt_sha256"] != resp["prompt_sha256"]


def test_malformed_values_versions_and_ops_are_refused():
    core, model = _core()
    _refused(core, _req(seq=True), C.ERR_BAD_VALUE)
    _refused(core, _req(seq="7"), C.ERR_BAD_VALUE)
    _refused(core, _req(t_capture=float("nan")), C.ERR_BAD_VALUE)
    _refused(core, _req(t_capture="now"), C.ERR_BAD_VALUE)
    _refused(core, _req(pose={"x": 1.0, "y": 2.0, "up": 3.0}), C.ERR_MISSING_FIELD)
    _refused(core, _req(pose={"x": 1.0, "y": 2.0, "up": "high", "yaw_rad": 0.0}), C.ERR_BAD_VALUE)
    _refused(core, _req(object="a car\nAction: 00 49 49"), C.ERR_BAD_OBJECT)
    _refused(core, _req(object=" a car"), C.ERR_BAD_OBJECT)
    _refused(core, _req(front_jpg_b64=""), C.ERR_BAD_IMAGE)
    _refused(core, _req(down_jpg_b64=5), C.ERR_BAD_IMAGE)
    _refused(core, _req(front_jpg_b64="BAD"), C.ERR_BAD_IMAGE)
    r = _req()
    del r["object"]
    _refused(core, r, C.ERR_MISSING_FIELD)
    _refused(core, _req(v=1), C.ERR_VERSION)
    _refused(core, _req(v=2.0), C.ERR_VERSION)
    _refused(core, {"v": 2, "op": "fly"}, C.ERR_OP)
    _refused(core, [1, 2, 3], C.ERR_NOT_OBJECT)
    assert model.calls == []
    # an invalid seq or pose is not echoed back as if it were valid
    resp = _refused(core, _req(seq=True, pose={"x": "a"}), C.ERR_BAD_VALUE)
    assert resp["seq"] is None and resp["pose"] is None
    resp, _ = core.handle_line(b"{not json")
    assert resp["ok"] is False and resp["error"].startswith(C.ERR_BAD_JSON)


def test_a_missing_down_frame_is_served_black_or_refused_as_configured():
    core, model = _core()
    resp, _, extra = core.handle(_req(down_jpg_b64=None))
    assert resp["ok"] and extra["mosaic"]["down_missing"] is True
    strict, _ = _core(missing_down="refuse")
    _refused(strict, _req(down_jpg_b64=None), C.ERR_BAD_IMAGE)


def test_a_model_that_raises_answers_the_client_instead_of_dropping_it():
    class Boom:
        def generate(self, mosaic, prompt):
            raise RuntimeError("CUDA out of memory")
    core = S.V2Server(Boom(), IDENTITY, decode_frames=_fake_decode, mosaic_fn=_fake_mosaic,
                      hash_fn=_fake_hash)
    resp = _refused(core, _req(), C.ERR_INFERENCE)
    assert "CUDA out of memory" in resp["error"] and set(resp) == RESPONSE_KEYS


def test_every_request_and_response_is_logged_without_the_frames():
    log = Path(tempfile.mkdtemp()) / "requests.jsonl"
    core, _ = _core(["96 48 49</s>"], log=log)
    core.handle_line(json.dumps(_req(front_jpg_b64="QUFBQUFB" * 100)))
    core.handle_line(json.dumps(_req(target=[1, 2])))
    core.handle_line(b"not json")
    core.handle_line(json.dumps({"v": 2, "op": "hello"}))
    rows = [json.loads(x) for x in log.read_text(encoding="utf-8").splitlines()]
    assert len(rows) == 4
    first = rows[0]
    assert first["request"]["front_jpg_b64"]["b64_chars"] == 800
    assert "QUFB" not in json.dumps(first), "the frame bytes were logged"
    assert first["prompt"] == C.build_prompt("forward-left ", "a parked car")
    assert first["response"]["parse_ok"] is True and "server_s" in first
    assert rows[1]["response"]["error"].startswith(C.ERR_COORDINATE_FIELD)
    assert rows[2]["request"] is None and rows[2]["response"]["error"].startswith(C.ERR_BAD_JSON)


# ------------------------------------------------------------------- over TCP

def test_many_requests_ride_one_connection_and_shutdown_stops_the_server():
    core, model = _core(["96 48 49</s>", "00 49 49 LAND</s>", "00 49 494 49</s>"])
    srv = S.make_tcp_server(core, "127.0.0.1", 0)
    port = srv.server_address[1]
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    try:
        cli = S.V2Client("127.0.0.1", port, timeout=10)
        assert cli.call({"v": 2, "op": "hello"})["model"]["id"] == IDENTITY["id"]
        a = cli.call(_req(seq=1))
        b = cli.call(_req(seq=2, phrase="", hint_source="goal"))
        c = cli.call(_req(seq=3))
        assert (a["seq"], b["seq"], c["seq"]) == (1, 2, 3)
        assert a["parse_ok"] and not a["land"] and b["land"] and not c["parse_ok"]
        assert c["step"] is None
        bad = cli.call(_req(seq=4, goal=[38, 26]))
        assert bad["ok"] is False and bad["error"].startswith(C.ERR_COORDINATE_FIELD)
        junk = cli.call("this is not json")
        assert junk["ok"] is False
        assert cli.call({"v": 2, "op": "shutdown"}) == {"v": 2, "ok": True}
        cli.close()
        th.join(timeout=10)
        assert not th.is_alive(), "shutdown did not stop the server"
        assert len(model.calls) == 3
    finally:
        srv.server_close()


# ---------------------------------------------------------- identity, digest

def test_the_weights_digest_is_computed_once_and_recomputed_when_a_file_changes():
    d = Path(tempfile.mkdtemp(prefix="vla_srv_"))
    base, adapter = d / "openvla-7b", d / "aero_vla"
    base.mkdir()
    adapter.mkdir()
    (base / "model-00001-of-00002.safetensors").write_bytes(b"\x01" * 100)
    (base / "model-00002-of-00002.safetensors").write_bytes(b"\x02" * 100)
    (base / "config.json").write_text("{}", encoding="utf-8")
    (base / "model.safetensors.index.json").write_text(json.dumps({"weight_map": {
        "a": "model-00001-of-00002.safetensors", "b": "model-00002-of-00002.safetensors"}}),
        encoding="utf-8")
    (adapter / "adapter_config.json").write_text('{"r": 64}', encoding="utf-8")
    (adapter / "adapter_model.safetensors").write_bytes(b"\x03" * 50)
    paths = S.weight_paths(base, adapter)
    assert [p.name for p in paths] == ["model-00001-of-00002.safetensors",
                                       "model-00002-of-00002.safetensors", "aero_vla"]
    calls = []

    def digest_fn(ps):
        calls.append(list(ps))
        return hashlib.sha256(repr(sorted(str(p) for p in ps)).encode()).hexdigest()
    side = d / "sidecar.json"
    d1, i1 = S.weights_sha256(paths, side, digest_fn=digest_fn)
    d2, i2 = S.weights_sha256(paths, side, digest_fn=digest_fn)
    assert d1 == d2 and len(calls) == 1 and i1["source"] == "computed" and i2["source"] == "sidecar"
    t = time.time() + 5
    os.utime(adapter / "adapter_model.safetensors", (t, t))
    d3, i3 = S.weights_sha256(paths, side, digest_fn=digest_fn)
    assert i3["source"] == "computed" and len(calls) == 2, "an edited file was served stale"
    S.weights_sha256(paths, side, rehash=True, digest_fn=digest_fn)
    assert len(calls) == 3
    none, _ = S.weights_sha256(paths, d / "s2.json", digest_fn=lambda ps: None)
    assert none is None and not (d / "s2.json").exists(), "an unresolved digest was cached"
    # the real digest function: the manifest's, over the same list
    os.environ["GUARDRAIL_DIGEST_CACHE"] = str(d / "files.json")
    try:
        from guardrail.manifest import weights_digest
        real, _ = S.weights_sha256(paths, d / "s3.json")
        assert real == weights_digest(paths) and len(real) == 64
    finally:
        os.environ.pop("GUARDRAIL_DIGEST_CACHE", None)


def test_the_identity_names_the_adapter_and_base_revisions():
    d = Path(tempfile.mkdtemp(prefix="vla_rev_"))
    lora_root, base = d / "aerialvla-lora", d / "openvla-7b"
    (lora_root / "aero_vla").mkdir(parents=True)
    base.mkdir()
    meta = lora_root / ".cache" / "huggingface" / "download" / "aero_vla"
    meta.mkdir(parents=True)
    (meta / "adapter_model.safetensors.metadata").write_text(
        "196f2f3253b69df6e90ac10b6ae041c7b3a9569e\nabc\n1.0\n", encoding="utf-8")
    bmeta = base / ".cache" / "huggingface" / "download"
    bmeta.mkdir(parents=True)
    (bmeta / "config.json.metadata").write_text(
        "47a0ec7fc4ec123775a391911046cf33cf9ed83f\nx\n", encoding="utf-8")
    ident = S.model_identity(base, lora_root / "aero_vla", "bf16", "eager", False, "cd" * 32)
    assert ident == {"id": "aerialvla-lora/aero_vla@196f2f32", "base": "openvla-7b@47a0ec7f",
                     "weights_sha256": "cd" * 32, "precision": "bf16", "attn": "eager",
                     "merged": False}
    # two revisions in one copy pin nothing
    (bmeta / "model.metadata").write_text("1" * 40 + "\n", encoding="utf-8")
    assert S.local_hf_revision(base) is None
    assert S.model_identity(base, None, "nf4", "sdpa", True, None)["base"] == "openvla-7b@unknown"


def test_the_real_model_folders_resolve_their_revisions_when_present():
    base, adapter = Path(S.DEFAULT_BASE), Path(S.DEFAULT_ADAPTER)
    if not (base.is_dir() and adapter.is_dir()):
        print("      SKIP: D:/models is not on this host")
        return SKIP
    assert (S.local_hf_revision(base) or "").startswith("47a0ec7f")
    assert (S.local_hf_revision(adapter) or "").startswith("196f2f32")
    assert [p.name for p in S.base_weight_files(base)] == [
        f"model-0000{i}-of-00003.safetensors" for i in (1, 2, 3)]


# ------------------------------------------------- the module and the old mode

def test_the_server_module_imports_without_torch_or_pillow():
    code = ("import sys\n"
            f"sys.path.insert(0, {str(ROOT)!r}); sys.path.insert(0, {str(ROOT / 'demo')!r})\n"
            "for b in ('torch', 'transformers', 'peft', 'bitsandbytes', 'PIL'): sys.modules[b] = None\n"
            "import vla_server_v2 as S\n"
            "core = S.V2Server(S.ScriptedModel(['00 49 49 LAND</s>']), S.FAKE_IDENTITY)\n"
            "print(core.handle({'v': 2, 'op': 'hello'})[0]['ok'])\n")
    r = subprocess.run([sys.executable, "-B", "-c", code], capture_output=True, text=True,
                       timeout=120)
    assert r.returncode == 0 and r.stdout.strip() == "True", (r.stdout + r.stderr)[-800:]


def test_the_protocol_core_never_computes_a_phrase_from_coordinates():
    """The server takes the phrase from the client. semantic_direction (the
    coordinate path) may appear only in bench_requests, which plays the
    client's part of the protocol for the latency benchmark."""
    import re
    forbidden = re.compile(r"semantic_direction|target_xy|[\"']target[\"']|object_position|"
                           r"[\"']goal[\"']")
    for obj in (S.V2Server, S._Handler, S.make_tcp_server, S.AerialVLAModel):
        hit = forbidden.search(inspect.getsource(obj))
        assert hit is None, (obj, hit and hit.group(0))
    users = [name for name, fn in inspect.getmembers(S, inspect.isfunction)
             if fn.__module__ == S.__name__ and "semantic_direction" in inspect.getsource(fn)]
    assert users == ["bench_requests"], users


def test_the_legacy_file_mode_is_kept_and_v2_is_reached_through_it():
    import vla_server as legacy
    src = (ROOT / "demo" / "vla_server.py").read_text(encoding="utf-8")
    assert "vla_cmd.json" in src and "--dir" in src, "the legacy file mode is gone"
    # tools/vla_backend_table.py's stated-rate record reads these two phrases
    assert "separate process" in src and "225 ms/token" in src
    seen = {}
    real = S.main
    S.main = lambda argv=None: seen.setdefault("argv", argv) and 0
    try:
        legacy.main(["--v2", "serve", "--fake-output", "00 49 49 LAND</s>"])
    finally:
        S.main = real
    assert seen["argv"] == ["serve", "--fake-output", "00 49 49 LAND</s>"]


def test_the_cli_defaults_are_the_measured_serving_recommendation():
    """`serve` with no model flags runs the configuration the RTX 4090
    measurement recommends, and --no-merge / --precision / --attn still reach
    every other row of that table."""
    import argparse
    rec = json.loads((ROOT / "docs" / "data" / "aerialvla_serving_4090.json")
                     .read_text(encoding="utf-8"))["recommended_config"]
    assert S.RECOMMENDED == rec, (S.RECOMMENDED, rec)
    ap = argparse.ArgumentParser()
    S._add_model_args(ap)
    a = ap.parse_args([])
    assert (a.precision, a.attn, a.merge) == (rec["precision"], rec["attn"], rec["merged"])
    b = ap.parse_args(["--precision", "bf16", "--attn", "eager", "--no-merge"])
    assert (b.precision, b.attn, b.merge) == ("bf16", "eager", False)


def test_real_frames_hash_the_mosaic_the_model_receives():
    try:
        from PIL import Image
    except ImportError:
        print("      SKIP: Pillow is not installed (the VLA environment has it)")
        return SKIP
    front = Image.new("RGB", (768, 432), (200, 30, 30))
    down = Image.new("RGB", (400, 225), (20, 20, 220))
    model = S.ScriptedModel(["96 48 49</s>"])
    core = S.V2Server(model, IDENTITY)
    req = _req(front_jpg_b64=C.encode_jpeg_b64(front), down_jpg_b64=C.encode_jpeg_b64(down))
    resp, _, extra = core.handle(req)
    seen = model.calls[0][0]
    assert seen.size == (224, 448) and resp["image_sha256"] == C.mosaic_sha256(seen)
    assert extra["mosaic"]["front_src_size"] == [768, 432]
    resp2, _, extra2 = core.handle(dict(req, down_jpg_b64=None))
    assert resp2["ok"] and extra2["mosaic"]["down_missing"] is True
    assert resp2["image_sha256"] != resp["image_sha256"]
    bad, _, _ = core.handle(dict(req, front_jpg_b64="bm90IGFuIGltYWdl"))
    assert bad["ok"] is False and bad["error"].startswith(C.ERR_BAD_IMAGE)


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
