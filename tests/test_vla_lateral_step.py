"""A LATERAL step (guardrail/vla_link.py, guardrail/aerialvla_codec.py,
demo/vla_server_foreign.py; 2026-10-30): a fourth bin, right_m, for a model
whose expert slides sideways with the camera on the subject (AIOT-VLA v7).

Run either way:
    pytest tests/test_vla_lateral_step.py -v
    python tests/test_vla_lateral_step.py

WHAT IS BEING CLAIMED

1. Upstream's step is untouched: three fields on the wire, three bins in the
   text, its reader refuses a fourth group, and its rate and endpoint
   execution fly as before.
2. A lateral step carries right_m on the wire and a fourth bin in the text
   ("FF DD YY RR"), inside RIGHT_RANGE; the codec reads it only when asked
   (lateral=True), the server writes it, and the client refuses an answer
   whose bins, text and step disagree on it.
3. Flown as a rate (TrackerConfig.rate_horizon_s), a lateral step is the
   body-frame velocity (fwd_m / H, right_m / H) on the heading at each tick,
   the pair capped at speed_cap_mps as a vector; flown as an endpoint, its
   endpoint is right_m to the right of the new heading and a pure sideways
   step still moves.
4. End to end: the AIOT-VLA family maps a model's right_m into a lateral
   step, the server answers it, and the client's protocol check reads it.

Shown failing first: at b4a8e4e Step had no right_m (TypeError), the codec
had no lateral reader and the server's encode_step no right_m, so every test
in 2-4 fails there.
"""
from __future__ import annotations

import base64
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import aerialvla_codec as C                          # noqa: E402
from guardrail import vla_link as L                                  # noqa: E402
from guardrail.vla_link import Decision, Pose, Step, StepTracker, TrackerConfig  # noqa: E402


def _raises(exc, fn, *a, **k) -> str:
    try:
        fn(*a, **k)
    except exc as e:
        return str(e)
    raise AssertionError(f"{fn.__name__} did not raise {exc.__name__}")


# ------------------------------------------------------- 1. upstream's step

def test_a_plain_step_keeps_upstreams_three_fields():
    s = Step(1.0, 0.0, 0.1)
    assert not s.lateral and s.right_m is None
    assert set(s.as_wire()) == set(L.STEP_FIELDS)
    assert Step.from_wire(s.as_wire()) == s
    p = C.parse_action("43 49 49</s>", require_eos=True)
    assert p.parse_ok and len(p.bins) == 3 and "right_m" not in p.step.to_dict()
    # upstream's reader refuses a fourth group
    q = C.parse_action("43 49 49 50</s>", require_eos=True)
    assert not q.parse_ok and "4 bins, not 3" in q.reason


def test_a_plain_rate_step_flies_as_before():
    cfg = TrackerConfig(rate_horizon_s=0.5, rate_yaw="rate", speed_cap_mps=5.0,
                        cruise_mps=4.5)
    tr = StepTracker(cfg)
    yaw = 0.7
    assert tr.accept(Decision.of(1, 0.0, Pose(0.0, 0.0, 8.0, yaw), Step(1.0, 0.0, 0.1)), 0.0)
    a = tr.tick(0.05, Pose(0.0, 0.0, 8.0, yaw)).action
    assert abs(a.vx - 2.0 * math.cos(yaw)) < 1e-9 and abs(a.vy - 2.0 * math.sin(yaw)) < 1e-9
    assert abs(a.yaw_rate - 0.2) < 1e-9
    assert "right_mps" not in tr.last_rate


# ------------------------------------------------------ 2. the lateral wire

def test_a_lateral_step_round_trips_with_right_m():
    s = Step(1.0, 0.0, 0.1, right_m=-0.6)
    assert s.lateral
    w = s.as_wire()
    assert set(w) == set(L.LATERAL_STEP_FIELDS) and w["right_m"] == -0.6
    assert Step.from_wire(w) == s
    _raises(ValueError, Step, 1.0, 0.0, 0.1, right_m=2.6)          # outside RIGHT_RANGE
    _raises(ValueError, Step.from_wire, {**w, "left_m": 0.0})
    _raises(ValueError, Step.from_wire, {"fwd_m": 1.0, "down_m": 0.0, "right_m": 0.1})
    _raises(ValueError, Step, 1.0, 0.0, 0.1, right_m=float("nan"))


def test_the_codec_reads_a_lateral_text_only_when_asked():
    p = C.parse_action("43 49 49 30</s>", require_eos=True, lateral=True)
    assert p.parse_ok and p.bins == (43, 49, 49, 30) and not p.land
    lo, hi = C.RIGHT_RANGE
    assert abs(p.step.right_m - (30 / C.MAX_BIN * (hi - lo) + lo)) < 1e-12
    assert set(p.step.to_dict()) == set(C.LATERAL_AXES)
    land = C.parse_action("43 49 49 30 LAND</s>", require_eos=True, lateral=True)
    assert land.parse_ok and land.land
    # three groups are not a lateral answer; a bin past 98 is no bin
    assert not C.parse_action("43 49 49</s>", require_eos=True, lateral=True).parse_ok
    assert not C.parse_action("43 49 49 99</s>", require_eos=True, lateral=True).parse_ok
    assert C.action_text((43, 49, 49, 30)) == "43 49 49 30"
    assert C.bins_to_step((43, 49, 49)).right_m is None
    # RANGES stays upstream's action space
    assert "right_m" not in C.RANGES and C.AXES == ("fwd_m", "down_m", "dpsi_rad")


def test_the_server_encodes_a_lateral_step_its_reader_accepts():
    import vla_server_foreign as F
    st = F.encode_step(1.2, 0.0, -0.1, right_m=0.7)
    assert len(st["bins"]) == 4 and st["text"].count(" ") == 3 and st["text"].endswith("</s>")
    p = C.parse_action(st["text"], require_eos=True, lateral=True)
    assert p.parse_ok and list(p.bins) == st["bins"]
    lo, hi = C.RIGHT_RANGE
    assert abs(p.step.right_m - 0.7) <= 0.5 * (hi - lo) / C.MAX_BIN + 1e-9
    assert st["flown"]["right_m"] == p.step.right_m
    clipped = F.encode_step(1.2, 0.0, -0.1, right_m=3.0)
    assert clipped["clipped"] == ["right_m"] and clipped["flown"]["right_m"] == hi
    # without right_m: upstream's three bins, as before
    assert len(F.encode_step(1.2, 0.0, -0.1)["bins"]) == 3


def _reply(sent: dict, text: str, bins, step: dict) -> dict:
    return {"v": L.PROTOCOL_VERSION, "seq": sent["seq"], "ok": True, "error": None,
            "raw_text": text, "bins": bins, "step": step, "land": False, "parse_ok": True,
            "latency_s": 0.01, "t_capture": sent["t_capture"], "pose": dict(sent["pose"]),
            "prompt_sha256": "a" * 64, "image_sha256": "b" * 64,
            "model": {"id": "aiotvla/v7@abababab", "base": "v7@unknown",
                      "weights_sha256": "ab" * 32, "precision": "fp16", "attn": "sdpa",
                      "merged": False}}


def test_the_client_checks_a_lateral_answers_text_and_bins():
    import vla_server_foreign as F
    sent = {"seq": 4, "t_capture": 10.0, "pose": {"x": 1.0, "y": 2.0, "up": 8.0, "yaw_rad": 0.3}}
    st = F.encode_step(1.2, 0.0, -0.1, right_m=0.7)
    rep = L.parse_infer_response(_reply(sent, st["text"], st["bins"], st["flown"]), sent)
    assert rep.ok and rep.step.lateral and abs(rep.step.right_m - st["flown"]["right_m"]) < 1e-12
    # the text and bins must carry the fourth bin the step claims, and the reverse
    plain = F.encode_step(1.2, 0.0, -0.1)
    for text, bins, step in ((plain["text"], plain["bins"], st["flown"]),
                             (st["text"], st["bins"], plain["flown"]),
                             (plain["text"], st["bins"], st["flown"])):
        _raises(L.ProtocolError, L.parse_infer_response, _reply(sent, text, bins, step), sent)
    # a right_m more than half a bin from its bin's value
    bad = dict(st["flown"], right_m=st["flown"]["right_m"] + 0.2)
    _raises(L.ProtocolError, L.parse_infer_response, _reply(sent, st["text"], st["bins"], bad),
            sent)


# ------------------------------------------------- 3. flying a lateral step

def test_a_lateral_rate_step_flies_sideways_on_the_body_heading():
    cfg = TrackerConfig(rate_horizon_s=0.5, rate_yaw="rate", speed_cap_mps=5.0,
                        cruise_mps=4.5)
    for yaw in (0.0, math.pi / 2, -2.4):
        tr = StepTracker(cfg)
        tr.accept(Decision.of(1, 0.0, Pose(0.0, 0.0, 8.0, yaw),
                              Step(1.0, 0.0, 0.1, right_m=0.5)), 0.0)
        a = tr.tick(0.05, Pose(0.0, 0.0, 8.0, yaw)).action
        want = (2.0 * math.cos(yaw) - 1.0 * math.sin(yaw), 2.0 * math.sin(yaw) + 1.0 * math.cos(yaw))
        assert abs(a.vx - want[0]) < 1e-9 and abs(a.vy - want[1]) < 1e-9, (yaw, a)
        assert abs(a.yaw_rate - 0.2) < 1e-9
        assert abs(tr.last_rate["right_mps"] - 1.0) < 1e-9
    # on the heading NOW: the aircraft turned since the capture
    tr = StepTracker(cfg)
    tr.accept(Decision.of(1, 0.0, Pose(0.0, 0.0, 8.0, 0.0), Step(0.0, 0.0, 0.0, right_m=0.5)), 0.0)
    a = tr.tick(0.05, Pose(0.0, 0.0, 8.0, 0.4)).action
    assert abs(a.vx + math.sin(0.4)) < 1e-9 and abs(a.vy - math.cos(0.4)) < 1e-9


def test_the_speed_cap_holds_a_lateral_step_as_a_vector():
    cfg = TrackerConfig(rate_horizon_s=0.5, rate_yaw="rate", speed_cap_mps=5.0,
                        cruise_mps=4.5)
    tr = StepTracker(cfg)
    tr.accept(Decision.of(1, 0.0, Pose(0.0, 0.0, 8.0, 0.0), Step(2.5, 0.0, 0.0, right_m=2.5)), 0.0)
    a = tr.tick(0.05, Pose(0.0, 0.0, 8.0, 0.0)).action
    assert abs(math.hypot(a.vx, a.vy) - 5.0) < 1e-9
    assert abs(math.atan2(a.vy, a.vx) - math.pi / 4) < 1e-9          # direction kept


def test_a_lateral_endpoint_step_moves_toward_its_offset_endpoint():
    tr = StepTracker()
    d = Decision.of(1, 0.0, Pose(0.0, 0.0, 10.0, 0.0), Step(0.0, 0.0, 0.0, right_m=2.0))
    assert tr.accept(d, 0.0)
    tgt = d.record["target"]
    assert abs(tgt["x"]) < 1e-12 and abs(tgt["y"] - 2.0) < 1e-12
    tk = tr.tick(0.1, Pose(0.0, 0.0, 10.0, 0.0))
    # a pure sideways step is not "done" at once: it moves right (+y at yaw 0)
    assert tk.author == "vla_step" and tk.action.vy > 0.5 and abs(tk.action.vx) < 1e-9
    tr2 = StepTracker()
    d2 = Decision.of(1, 0.0, Pose(0.0, 0.0, 10.0, math.pi / 2), Step(3.0, 0.0, 0.0, right_m=-1.0))
    tr2.accept(d2, 0.0)
    t2 = d2.record["target"]
    assert abs(t2["x"] - 1.0) < 1e-9 and abs(t2["y"] - 3.0) < 1e-9


# ------------------------------------------------------------ 4. end to end

def test_the_aiotvla_family_answers_a_lateral_step_the_client_reads():
    import vla_server_foreign as F

    class Scripted:
        precision, attn, merged = "fp16", "sdpa", False

        def generate(self, image, inputs):
            return {"raw": {"fwd_m": 0.9, "down_m": 0.0, "dpsi_rad": -0.03, "right_m": 0.7,
                            "visible_p": 0.9, "bearing_rad": 0.0, "slots_valid": 10,
                            "history_reset": None,
                            "memory": {"valid": True, "range_m": 25.0}},
                    "model_s": 0.01, "pre_s": 0.005, "gen_s": 0.005}

    ident = F.foreign_identity("aiotvla", name="v7", folder="v7", revision=None,
                               digest="cd" * 32, precision="fp16", attn="sdpa",
                               front_source="detector")
    srv = F.ForeignServer(F.AIOTVLAFamily(horizon_s=0.5), Scripted(), ident, None,
                          decode_front=lambda req: type("I", (), {"size": (768, 432)})(),
                          hash_fn=lambda img: "0" * 64, prepare=lambda img: (img, {}))
    req = {"v": 2, "op": "infer", "seq": 3, "t_capture": 50.0,
           "pose": {"x": 0.0, "y": 0.0, "up": 8.0, "yaw_rad": 0.0},
           "front_jpg_b64": base64.b64encode(b"x").decode(), "down_jpg_b64": None,
           "phrase": "straight ahead ", "hint_source": "detector", "object": "a red car",
           "csp_text": None, "csp_mode": "off"}
    resp, stop, _ = srv.handle(req)
    assert resp["ok"] and resp["parse_ok"] and not stop, resp.get("error")
    assert len(resp["bins"]) == 4 and resp["raw_text"] == F.encode_step(
        0.9, 0.0, -0.03, right_m=0.7)["text"]
    assert resp["mapping"]["geometry"]["memory"]["range_m"] == 25.0
    rep = L.parse_infer_response(resp, req)
    assert rep.ok and rep.step.lateral and abs(rep.step.right_m - 0.7) < 0.03


# --------------------------------------------------------------------- runner

if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
