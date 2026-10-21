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
     JSON, shutdown stops the server. Every request the server read is
     answered and the connection stays usable, whatever raised: an
     unwritable log, a frame decoder, nested JSON, a model result without
     timing (review of V1a, P1); every ok=false answer has every infer key.
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
    # ...and in the free text (review of V1a, P10): the object slot and the
    # CSP text cannot carry a position either
    _refused(core, _req(object="target at x=38.0 y=26.0"), C.ERR_COORDINATE_TEXT)
    _refused(core, _req(csp_mode="csp", csp_text="the path to the target (38, 26) is clear"),
             C.ERR_COORDINATE_TEXT)
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


# ------------------------------------- every request read gets an answer (V1a P1)
#
# Review of V1a (P1), probe-verified over TCP at 6519b20: only an exception
# inside model.generate() was answered. A JSONL log that could not be written
# (the default path is on D:, which fills up, under OneDrive, which locks
# files) raised out of handle_line AFTER the model had answered, so the client
# met EOF on every request and reconnected forever; nested JSON raised
# RecursionError (not a ValueError), a frame Pillow refuses with
# DecompressionBombError (not an OSError) and a model dict without model_s
# raised too. Each of these dropped the connection with no answer and no log
# line. Every test here failed there.

def _serve(core):
    srv = S.make_tcp_server(core, "127.0.0.1", 0)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    return srv, th, srv.server_address[1]


def _stop(srv, th):
    srv.shutdown()
    srv.server_close()
    th.join(timeout=10)


def test_a_log_that_cannot_be_written_costs_neither_the_answer_nor_the_connection():
    import contextlib
    import io
    logdir = Path(tempfile.mkdtemp(prefix="vla_log_"))      # a directory: every append fails
    core, model = _core(["96 48 49</s>"], log=logdir)
    srv, th, port = _serve(core)
    err = io.StringIO()
    try:
        with contextlib.redirect_stderr(err):
            cli = S.V2Client("127.0.0.1", port, timeout=10)
            for k in (1, 2, 3):
                r = cli.call(_req(seq=k))
                assert r["ok"] is True and r["seq"] == k and r["parse_ok"], r
            junk = cli.call("{not json")
            assert junk["ok"] is False and junk["error"].startswith(C.ERR_BAD_JSON), junk
            assert cli.call({"v": 2, "op": "hello"})["ok"] is True
            cli.close()
    finally:
        _stop(srv, th)
    assert len(model.calls) == 3
    assert core.counts["log_failures"] == 5, core.counts      # 3 infers, the junk, the hello
    warned = [ln for ln in err.getvalue().splitlines() if "log" in ln.lower()]
    assert len(warned) == 1 and str(logdir) in warned[0], err.getvalue()


def test_an_unwritable_log_is_reported_at_startup_not_first_during_a_flight():
    """The default log is under demo/out on D: (which fills up) in OneDrive
    (which locks files). serve probes it once at startup, so the operator
    reads the warning before the pilot connects; the probe never raises and
    the warning is not repeated by the first failed write."""
    import contextlib
    import io
    bad = Path(tempfile.mkdtemp(prefix="vla_log_"))                 # a directory
    core, _ = _core(log=bad)
    err = io.StringIO()
    with contextlib.redirect_stderr(err):
        assert core.probe_log() is False
        core.handle_line(json.dumps(_req()))
    assert err.getvalue().count("WARNING") == 1 and str(bad) in err.getvalue(), err.getvalue()
    assert core.counts["log_failures"] == 1, "the startup probe is not a request"
    good = Path(tempfile.mkdtemp(prefix="vla_log_")) / "sub" / "requests.jsonl"
    ok, _ = _core(log=good)
    assert ok.probe_log() is True and good.parent.is_dir()
    assert _core()[0].probe_log() is True                             # no log at all
    assert "probe_log()" in inspect.getsource(S.main)


def test_any_exception_outside_generate_is_answered_and_the_connection_stays_usable():
    def flaky_decode(req):
        if req["seq"] == 2:
            # stands for PIL.Image.DecompressionBombError, which is not an OSError
            raise RuntimeError("decoder blew up")
        if req["seq"] == 3:
            raise MemoryError()
        return _fake_decode(req)

    class NoTiming:
        """A model whose result dict lacks model_s (was a KeyError at line 532)."""
        def generate(self, mosaic, prompt):
            return {"raw_text": "96 48 49</s>"}

    core = S.V2Server(S.ScriptedModel(["96 48 49</s>"]), IDENTITY, decode_frames=flaky_decode,
                      mosaic_fn=_fake_mosaic, hash_fn=_fake_hash)
    srv, th, port = _serve(core)
    try:
        cli = S.V2Client("127.0.0.1", port, timeout=10)
        assert cli.call(_req(seq=1))["ok"] is True
        for k in (2, 3):
            r = cli.call(_req(seq=k))
            assert r["ok"] is False and r["seq"] == k and set(r) == RESPONSE_KEYS, r
            assert r["error"].split(":", 1)[0] == C.ERR_INTERNAL, r["error"]
            assert r["step"] is None and r["land"] is False and r["parse_ok"] is False
        deep = cli.call("[" * 100000 + "]" * 100000)          # RecursionError in json.loads
        assert deep["ok"] is False and deep["error"].startswith(C.ERR_BAD_JSON), deep
        assert cli.call(_req(seq=4))["ok"] is True, "the connection did not survive"
        cli.close()
    finally:
        _stop(srv, th)
    assert core.counts["internal_error"] == 2 and core.counts["infer"] == 4, core.counts
    bare = S.V2Server(NoTiming(), IDENTITY, decode_frames=_fake_decode, mosaic_fn=_fake_mosaic,
                      hash_fn=_fake_hash)
    r, _ = bare.handle_line(json.dumps(_req()))
    assert r["ok"] is False and r["error"].startswith(C.ERR_INFERENCE) and "model_s" in r["error"]
    assert bare.counts["inference_error"] == 1 and bare.counts["refused"] == 0, bare.counts


def test_every_refusal_has_every_infer_key_whatever_was_sent():
    """P2: an unparseable line, a too-long line, a request without `op` and
    a non-object were answered {v, ok, error} only, though the docstring said
    an infer response always has every key; a client reading resp['seq']
    met a KeyError. Every ok=false answer now has the full shape."""
    core, _ = _core()
    for line in (b"{not json", b"\xff\xfe", b"[1, 2, 3]", b'{"v": 2}', b'"infer"',
                 json.dumps(_req(op=None)).encode()):
        resp, stop = core.handle_line(line)
        assert not stop and resp["ok"] is False and set(resp) == RESPONSE_KEYS, (line, resp)
        assert resp["seq"] is None or resp["seq"] == 7
        assert resp["model"] == IDENTITY and resp["raw_text"] == "" and resp["step"] is None
    real = C.MAX_LINE_BYTES
    C.MAX_LINE_BYTES = 2000
    srv, th, port = _serve(core)
    try:
        cli = S.V2Client("127.0.0.1", port, timeout=10)
        cli.sock.sendall(b"x" * 5000)                         # no newline in reach
        got = json.loads(cli.rfile.readline())
        assert got["ok"] is False and got["error"].startswith(C.ERR_LINE_TOO_LONG), got
        assert set(got) == RESPONSE_KEYS, got
        cli.close()
    finally:
        C.MAX_LINE_BYTES = real
        _stop(srv, th)


def test_a_number_too_big_for_a_float_is_a_bad_value_not_a_crash():
    """`_finite_number` called float() on a JSON integer of 400 digits,
    which raises OverflowError: the validator itself crashed."""
    core, model = _core()
    _refused(core, _req(t_capture=10 ** 400), C.ERR_BAD_VALUE)
    _refused(core, _req(pose={"x": 10 ** 400, "y": 0.0, "up": 1.0, "yaw_rad": 0.0}),
             C.ERR_BAD_VALUE)
    assert model.calls == []


def test_raw_text_is_always_a_string():
    """P2: a model returning None or bytes put null or a non-string in
    raw_text, which the brief types <str>. It is coerced; the strict reader
    still refuses it (parse_ok false, the client holds zero)."""
    for text, want in ((None, ""), (b"96 48 49</s>", "b'96 48 49</s>'"), (964849, "964849")):
        class Odd:
            def generate(self, mosaic, prompt, _t=text):
                return {"raw_text": _t, "model_s": 0.01}
        core = S.V2Server(Odd(), IDENTITY, decode_frames=_fake_decode, mosaic_fn=_fake_mosaic,
                          hash_fn=_fake_hash)
        resp, _, extra = core.handle(_req())
        assert resp["ok"] is True and resp["raw_text"] == want, resp
        assert resp["parse_ok"] is False and resp["step"] is None, resp
        assert "not text" in extra["parse_reason"]


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
                     "merged": False, "agreement": None}
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


SERVING = ROOT / "docs" / "data" / "aerialvla_serving_4090.json"


def _exits_2(fn, *a):
    """Run fn(*a), which must exit through argparse's error (code 2); return
    what it wrote to stderr."""
    import contextlib
    import io
    err = io.StringIO()
    with contextlib.redirect_stderr(err):
        try:
            fn(*a)
        except SystemExit as e:
            assert e.code == 2, e.code
            return err.getvalue()
    raise AssertionError(f"{fn.__name__}{a!r} did not refuse")


def test_the_model_configuration_is_chosen_explicitly_on_every_run():
    """Decided 2026-10-22 (review of V1a): the precision changes what the
    model says - bf16 reproduces the upstream-precision reference (96 % of
    the 48 probe inputs, sdpa unmerged) while every nf4 row agrees on only
    25-33 % - so no default may pick it silently. At 6519b20 `serve` with no
    model flag loaded nf4/sdpa/merged. Now serve and bench refuse until
    --precision, --attn and --merge/--no-merge are all given, and the refusal
    prints the measured table so the choice is an informed one. Also P11:
    --save-merged on anything but bf16 + --merge is refused BEFORE the
    two-minute model load, and the model class has no default either."""
    import argparse
    ap = argparse.ArgumentParser()
    S._add_model_args(ap)
    a = ap.parse_args([])
    assert (a.precision, a.attn, a.merge) == (None, None, None)
    for argv in ([], ["--precision", "nf4"], ["--precision", "nf4", "--attn", "sdpa"],
                 ["--attn", "sdpa", "--no-merge"]):
        msg = _exits_2(S.check_model_args, ap, ap.parse_args(argv))
        assert "--precision" in msg and "--merge" in msg and "CityLife" in msg, msg
        for row in ("nf4/sdpa/merged", "bf16/sdpa/unmerged", "bf16/eager/unmerged"):
            assert row in msg, (row, msg)
        assert "96%" in msg and "31%" in msg, "the agreement of each option is not shown"
    for argv in (["--precision", "nf4", "--attn", "sdpa", "--merge"],
                 ["--precision", "bf16", "--attn", "eager", "--no-merge"],
                 ["--precision", "bf16", "--attn", "sdpa", "--merge", "--save-merged"]):
        S.check_model_args(ap, ap.parse_args(argv))          # chosen: no refusal
    for argv in (["--precision", "nf4", "--attn", "sdpa", "--merge", "--save-merged"],
                 ["--precision", "bf16", "--attn", "sdpa", "--no-merge", "--save-merged"]):
        assert "--save-merged" in _exits_2(S.check_model_args, ap, ap.parse_args(argv))
    # main() checks before it builds anything (build_real would import torch)
    assert "--precision" in _exits_2(S.main, ["serve"])
    assert "--precision" in _exits_2(S.main, ["bench", "--out", "never.json"])
    sig = inspect.signature(S.AerialVLAModel.__init__)
    for k in ("precision", "attn", "merge"):
        p = sig.parameters[k]
        assert p.default is inspect.Parameter.empty and p.kind is p.KEYWORD_ONLY, (k, p)
    assert not hasattr(S, "RECOMMENDED"), "a serving default is back"


def test_the_measured_low_vram_option_is_an_option_not_a_default():
    """The RTX 4090 measurement names nf4/sdpa/merged the low-VRAM option and
    bf16/sdpa/unmerged the closest to the reference, and records that there
    is no default: the CityLife VRAM measurement (next wave) decides."""
    s = json.loads(SERVING.read_text(encoding="utf-8"))
    assert s.get("default") is None and "recommended" not in s and "recommended_config" not in s
    assert s["low_vram_config"] == {"precision": "nf4", "attn": "sdpa", "merged": True}, s
    assert s["fidelity_config"] == {"precision": "bf16", "attn": "sdpa", "merged": False}, s
    assert "CityLife" in s["choice"] and "--precision" in s["choice"], s["choice"]


def test_the_identity_carries_the_configurations_measured_agreement():
    """Every response's identity says how far its configuration moves
    AerialVLA's outputs from the reference (bf16/eager/unmerged), from the
    RTX 4090 probe - and says "not measured" for other weights, a
    configuration the file has no row for, or no file at all."""
    s = json.loads(SERVING.read_text(encoding="utf-8"))
    dig = s["weights_sha256"]
    a = S.measured_agreement("nf4", "sdpa", True, dig)
    assert a["measured"] is True and a["reference"] == "bf16/eager/unmerged", a
    assert (a["probe_n"], a["probe_exact"], a["probe_close"]) == (48, 0.312, 0.5), a
    assert a["source"] == "docs/data/aerialvla_serving_4090.json" and a["why"] is None
    b = S.measured_agreement("bf16", "sdpa", False, dig)
    assert (b["probe_exact"], b["probe_close"]) == (0.958, 0.958), b
    ref = S.measured_agreement("bf16", "eager", False, dig)
    assert ref["probe_exact"] == 1.0 and "reference" in ref["note"], ref
    for args, why in (((("nf4", "sdpa", True, "cd" * 32)), "weights"),
                      ((("nf4", "sdpa", True, dig, Path(tempfile.mkdtemp()) / "no.json")),
                       "no serving measurement")):
        x = S.measured_agreement(*args)
        assert x["measured"] is False and x["probe_exact"] is None and why in x["why"], x
    d = Path(tempfile.mkdtemp(prefix="vla_id_"))
    (d / "openvla-7b").mkdir()
    (d / "aero_vla").mkdir()
    ident = S.serving_identity(d / "openvla-7b", d / "aero_vla", "nf4", "sdpa", True, dig)
    assert ident["agreement"] == a and ident["precision"] == "nf4" and ident["merged"] is True
    core = S.V2Server(S.ScriptedModel(["96 48 49</s>"]), ident, decode_frames=_fake_decode,
                      mosaic_fn=_fake_mosaic, hash_fn=_fake_hash)
    assert core.handle(_req())[0]["model"]["agreement"] == a
    assert S.FAKE_IDENTITY["agreement"] is None


def test_hello_reports_health_because_hello_alone_proves_nothing():
    """P4: hello never touches the model, so a server whose generate() hangs
    (holding the model lock) or fails on every call answered hello ok with
    the full identity. hello now carries health: the counts, the time of the
    last ok infer and, while generate() runs, since when. Liveness is a
    recent ok infer, not a hello."""
    entered, release = threading.Event(), threading.Event()

    class Hangs:
        def generate(self, mosaic, prompt):
            entered.set()
            release.wait(10)
            return {"raw_text": "96 48 49</s>", "model_s": 0.5}
    core = S.V2Server(Hangs(), IDENTITY, decode_frames=_fake_decode, mosaic_fn=_fake_mosaic,
                      hash_fn=_fake_hash)

    def hello():
        box = {}
        t = threading.Thread(target=lambda: box.update(r=core.handle({"v": 2, "op": "hello"})[0]))
        t.start()
        t.join(5)
        assert not t.is_alive(), "hello blocked on the model"
        return box["r"]["model"]["health"]
    h0 = hello()
    assert h0["t_last_ok_infer"] is None and h0["in_flight_since"] is None, h0
    assert h0["counts"]["infer"] == 0 and "liveness" in h0
    worker = threading.Thread(target=lambda: core.handle(_req()))
    worker.start()
    assert entered.wait(5)
    h1 = hello()
    assert h1["in_flight_since"] is not None and h1["in_flight_s"] >= 0.0, h1
    assert h1["t_last_ok_infer"] is None
    release.set()
    worker.join(10)
    h2 = hello()
    assert h2["in_flight_since"] is None and h2["t_last_ok_infer"] is not None, h2
    assert h2["counts"]["infer"] == 1 and h2["s_since_last_ok_infer"] >= 0.0

    class Dies:
        def generate(self, mosaic, prompt):
            raise RuntimeError("CUDA error: an illegal memory access")
    dead = S.V2Server(Dies(), IDENTITY, decode_frames=_fake_decode, mosaic_fn=_fake_mosaic,
                      hash_fn=_fake_hash)
    for _ in range(3):
        dead.handle(_req())
    h = dead.handle({"v": 2, "op": "hello"})[0]["model"]["health"]
    assert h["t_last_ok_infer"] is None and h["t_last_inference_error"] is not None, h
    assert h["counts"]["inference_error"] == 3 and h["counts"]["refused"] == 0, h


def test_the_bench_writes_the_log_it_names_and_measures_reserved_vram():
    """P3: run_bench called core.handle(), which does not log, while the
    bench output stored 'log': <path> for a file nothing wrote. P5: it
    recorded only torch's allocated peak, which leaves out the allocator
    cache and the CUDA context; the reserved peak is recorded too."""
    class FakeCuda:
        def reset_peak_memory_stats(self):
            pass

        def max_memory_allocated(self):
            return 5.0e9

        def max_memory_reserved(self):
            return 6.5e9
    log = Path(tempfile.mkdtemp(prefix="vla_bench_")) / "bench.jsonl"
    core, model = _core(["96 48 49</s>"], log=log)
    reqs = [dict(_req(seq=k), _frame=f"f{k}.png") for k in range(1, 6)]
    probe = [dict(_req(seq=100 + k), _frame="p.png") for k in range(3)]
    res = S.run_bench(core, reqs, 2, probe, cuda=FakeCuda(), smi=lambda: 9000)
    rows = [json.loads(x) for x in log.read_text(encoding="utf-8").splitlines()]
    assert len(rows) == 5 + 3, len(rows)
    assert [r["request"]["seq"] for r in rows[:5]] == [1, 2, 3, 4, 5]
    assert res["vram"] == {"inference_peak_allocated_gb": 5.0, "inference_peak_reserved_gb": 6.5,
                           "nvidia_smi_used_mib_after_timed": 9000}, res["vram"]
    assert res["stats_latency"]["n"] == 3 and len(res["probe_outputs"]) == 3


def test_real_frames_hash_the_mosaic_the_model_receives():
    try:
        from PIL import Image
    except ImportError:
        # CI's Pillow step sets this (review of V1a, P9): there a missing
        # Pillow is a broken install, not an absent package.
        assert os.environ.get("GUARDRAIL_REQUIRE_PILLOW") != "1", \
            "GUARDRAIL_REQUIRE_PILLOW=1 and Pillow does not import"
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
    # Review of V1a (P1): a decompression bomb dropped the connection (Pillow's
    # DecompressionBombError is not an OSError). Over TCP it is now a bad
    # image, and the same connection carries the next request.
    import base64
    from test_aerialvla_codec import png_claiming
    bomb = base64.b64encode(png_claiming(20000, 20000)).decode("ascii")
    srv, th, port = _serve(core)
    try:
        cli = S.V2Client("127.0.0.1", port, timeout=10)
        r = cli.call(dict(req, seq=11, down_jpg_b64=bomb))
        assert r["ok"] is False and r["error"].startswith(C.ERR_BAD_IMAGE), r
        assert set(r) == RESPONSE_KEYS and r["seq"] == 11
        assert cli.call(dict(req, seq=12))["ok"] is True
        cli.close()
    finally:
        _stop(srv, th)


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
