"""sitl/ros2_vla_bridge_node.py (to-do VLA-08) and what the Shield node does
with it, checked without ROS, MAVROS, a camera, SITL or the model.

Run either way:
    pytest tests/test_ros2_vla_bridge.py -v
    python tests/test_ros2_vla_bridge.py

WHAT IS BEING CLAIMED

1. The message. /vla/action_4d carries the grant's four body-frame values
   first, then the decision behind them (its age and seq, the tick's author,
   the LAND / END flags); the bridge's layout and the Shield node's own copy
   are the same, a float32 round trip keeps it, the stub's four values read
   as before, and a malformed tail is refused, not flown as a decision.
2. The pilot. Fed camera frames and poses, the bridge's pure core says hello,
   pins a real AerialVLA identity, warms up on live pairs (three requests,
   the last judged), asks for nothing until the Shield flies the stream,
   then flies the GOTO toward the Shield's own mission target with the city
   rail's StepTracker, publishing at every tick: a closed loop on a point
   vehicle with a scripted server answering like a perfect AerialVLA ends on
   the model's LAND with both flags set, sends only the training phrases and
   no coordinate field, and writes vla_steps.jsonl that tools/todo.py reads
   as a VLA flight once the manifest the Shield node builds from the
   bridge's identity names it.
3. The failures. A stand-in identity is refused unless allowed and is then a
   stub; no camera pair makes the bridge not ready, publishing zero with the
   end flag; a dead server makes it hover (author hover, no decision) and,
   after the dead timeout, end the mission without LAND; a rail hold (out of
   GUIDED) suspends the tracker and stops the requests; a tick that raises
   publishes nothing; a reused directory loses its old records to
   _previous/.
4. The Shield node (the real ShieldNode on tests/test_railinputs_node.py's
   stand-in ROS stack): it flies the bridge's stream end to end and ends the
   mission on the bridge's LAND through its own mission-end LAND, filing
   metrics.pilot as kind vla with the server's digest and vla_model_hash as
   its sha256; it holds a vla_step whose decision is older than
   --decision-timeout-s and releases on a fresh one, but never judges a
   hold; the stub's four values change nothing; --vla-ready-timeout-s
   refuses to arm without a ready identity.
5. The ROS wiring of the bridge node on a stand-in rclpy: its topics,
   message types and QoS, the latched identity, the layout label, and a
   refused camera frame said once.
6. The review of 339f23a (vla_disc/review-V2c-339f23a.md; its probes in
   rev339/): a malformed, partial, four-value-after-eight, mislabelled or
   seq-regressing message is refused (not flown, not a receipt) and held
   (P1-A); a decision hold that lasts ends the mission past
   --decision-hold-max-s (P3); a lasting second publisher is recorded and
   costs the KPI grade; bring_up arms after a 900 s wait for the VLA and says how
   often arming was refused instead of blaming the EKF (P1-B); only a
   vla_step moves, a vla_step has a decision, a hold has its own age bound
   (P2-1); the bridge publishes nothing on a stale pose (P2-3) and writes
   its records on SIGTERM or an interrupted spin, the run script TERMs and
   waits (P2-4); the ready gate refuses a stub and a contract mismatch and
   outwaits the bridge's hello; a fake: id is a stub and a never-ready
   bridge is unresolved; held rows keep what the VLA sent; a VLA LAND says
   where it lands; the capture time is the camera stamp when it is on this
   host's clock; the bridge's default directory is the Shield's (P3).
   tests/test_ros2_package.py checks the launch file's gate and directory
   (P2-2), tests/test_deploy_configs.py the compose file's pending gaps.
7. The re-review of 23c88d3 (vla_disc/rereview-V2c-23c88d3.md; rerev23c/):
   a sound bridge's 2 s stall near the end of a 9.5 s old step, end to end
   with the real bridge, gives LOITER then GUIDED and flies on, never LAND,
   because silence ages a decision only up to the receipt timeout; a hold
   ends on the next valid message, so a stray refused message between steps
   holds once and the bridge flies on (N1); a malformed identity layout
   never raises (N2); repeated holds end the mission instead of flapping
   (N3); the escalation never overrides the FSM's own Loiter and records
   where (N4); a second Ctrl+C during close() cannot cut the records short
   (N5); an unlabelled eight is refused once the layout is announced (N6); a
   passing second publisher is recorded without costing the grade (N7); a
   non-finite action value is a recorded refusal (N8).

Shown failing first, for 7: the 12 tests new or changed for it fail on
23c88d3's code with this file (the N1-N8 tests, the two tests of 6
rewritten for N1 because it reverses their claim - a hold ended only on a
fresh vla_step - and the second-publisher test, now three checks long),
among them the end-to-end stall: LOITER, LAND there. For 6: the 18 tests of
section 6 failed (11 FAIL, 7 ERROR) in a `git archive` copy of 339f23a with
this file copied in, while the 20 before them passed there; the new launch
and compose tests failed there too. For 1-5: in a `git archive` copy of 5b00e6b (before this change)
with this file copied in: it stops at its import (no
sitl/ros2_vla_bridge_node.py). With the new bridge module added to that copy
and the Shield node left as it was, 11 of the 20 tests fail - every test of
the Shield's side and every cross-check against it (no VLA_ACTION_FIELDS,
pilot_fields, DecisionWatchdog, pilot_from_identity, vla_ready,
--decision-timeout-s or --vla-ready-timeout-s there; the end flag ignored).
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import struct
import sys
import tempfile
import threading
import types
import zlib
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "sitl", ROOT / "tests", ROOT / "demo", ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail.frames import from_body                               # noqa: E402
from guardrail.models import Action4D, State, load_policy            # noqa: E402
from guardrail.vla_link import PHRASES, Tick, TrackerConfig          # noqa: E402
import vla_fake_server as F                                          # noqa: E402
import vla_mission as VM                                             # noqa: E402
import ros2_vla_bridge_node as B                                     # noqa: E402
import ros2_shield_node as SN                                        # noqa: E402

SKIP = "SKIP"
POLICY = ROOT / "policies" / "sim_demo_policy.yaml"
# A model identity like the real server's (vla_server_v2): the dashboard's
# digest rule refuses the fake server's "0" * 64.
REAL_MODEL = {"id": "aerialvla-lora/aero_vla@196f2f32", "base": "openvla-7b@47a0ec7f",
              "weights_sha256": hashlib.sha256(b"base shards + adapter").hexdigest(),
              "precision": "bf16", "attn": "sdpa", "merged": False,
              "agreement": {"measured": True, "probe_exact": 0.96},
              "host": {"host_arch": "x86_64", "device_model": None, "hostname": "gpu-desk"}}


def _with_model(respond, model):
    def r(req):
        out = respond(req)
        if isinstance(out, dict) and "model" in out:
            out = dict(out, model={k: v for k, v in model.items() if k != "host"})
        return out
    return r


def _img(w=8, h=6, enc="rgb8", stamp_ns=None, step=None, seed=1):
    """A sensor_msgs/Image stand-in: a deterministic pattern, optional row
    padding, the header stamp in ns (None: unset)."""
    step = step or 3 * w
    row = bytes(((x * 7 + seed * 13 + c * 29) % 256) for x in range(w) for c in range(3))
    data = b"".join(row + b"\xee" * (step - 3 * w) for _ in range(h))
    sec, ns = (0, 0) if stamp_ns is None else divmod(int(stamp_ns), 1_000_000_000)
    return SimpleNamespace(encoding=enc, width=w, height=h, step=step, data=data,
                           header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=ns)))


class _Veh:
    """A point multirotor on a simulated clock (velocity lag 0.3 s), as in
    tests/test_vla_fly.py: it moves only by the world-frame action given."""

    def __init__(self, x=0.0, y=0.0, up=15.0, yaw_deg=0.0):
        self.t = 1000.0
        self.x, self.y, self.up, self.yaw = x, y, up, math.radians(yaw_deg)
        self.v = [0.0, 0.0, 0.0]

    def clock(self) -> float:
        return self.t

    def yaw_deg(self) -> float:
        return math.degrees(self.yaw) % 360.0

    def state(self) -> State:
        return State(x=self.x, y=self.y, up=self.up, yaw_deg=self.yaw_deg())

    def step(self, a: Action4D, dt=0.1) -> None:
        k = min(1.0, dt / 0.3)
        for i, c in enumerate((a.vx, a.vy, a.vz_up)):
            self.v[i] += k * (c - self.v[i])
        self.x += self.v[0] * dt
        self.y += self.v[1] * dt
        self.up += self.v[2] * dt
        self.yaw = (self.yaw + a.yaw_rate * dt + math.pi) % (2 * math.pi) - math.pi
        self.t = round(self.t + dt, 10)


def _tmp() -> Path:
    return Path(tempfile.mkdtemp(prefix="vlabridge_"))


def _mission(target):
    from guardrail.compiler import ConstraintCompiler
    policy = load_policy(POLICY)
    m = ConstraintCompiler(policy).parse_command("fly to the northeast pad at 6 m/s")
    m = m.model_copy(update={"target_x": float(target[0]), "target_y": float(target[1])})
    return policy, m


def _bridge(srv, veh, out, target, *, clock=None, **cfg):
    policy, mission = _mission(target)
    kw = {"wait_frames_s": 1.0, "max_s": 300.0, "dead_timeout_s": 3.0}
    kw.update(cfg)
    c = B.BridgeConfig(host="127.0.0.1", port=srv.port, **kw)
    return B.VLABridge(c, mission=mission, policy=policy,
                       policy_source={"kind": "yaml", "path": str(POLICY)}, out=out,
                       clock=clock or veh.clock, sleep=lambda s: None)


def _feed(br, veh, clock=None):
    """One pose and one front + down pair, all arriving now."""
    t = (clock or veh.clock)()
    br.on_pose(veh.x, veh.y, veh.up, veh.yaw_deg(), t=t)
    ns = int(round(t * 1e9))
    br.put_image("front", _img(stamp_ns=ns, seed=1))
    br.put_image("down", _img(stamp_ns=ns, seed=2))


def _fly(br, veh, n_max, *, after_end=3):
    """The bridge flying the vehicle by its own body-frame messages until
    `after_end` ticks after the end flag; returns every message."""
    msgs, left = [], None
    for _ in range(n_max):
        _feed(br, veh)
        msg = br.tick(veh.clock())
        msgs.append(msg)
        if br.client is not None:
            br.client.wait_idle(3.0)
        veh.step(from_body(*msg[:4], veh.yaw_deg()))
        if left is None and int(msg[7]) & B.FLAG_END:
            left = after_end
        if left is not None:
            left -= 1
            if left <= 0:
                break
    return msgs


# =========================================================================== #
# 1. The message
# =========================================================================== #
def test_the_layout_is_the_shields_own_and_survives_float32():
    assert B.ACTION_FIELDS == SN.VLA_ACTION_FIELDS
    assert {v: k for k, v in B.AUTHOR_CODES.items()} == SN.VLA_AUTHORS
    assert (B.FLAG_LAND, B.FLAG_END) == (SN.VLA_FLAG_LAND, SN.VLA_FLAG_END)
    assert B.ACTION_LABEL.split(",") == list(B.ACTION_FIELDS)
    f32 = lambda xs: list(struct.unpack(f"{len(xs)}f", struct.pack(f"{len(xs)}f", *xs)))  # noqa: E731
    step = Tick(Action4D(), "vla_step", "translate", 2 ** 24 - 3, 4.25)
    msg = B.action_message((1.0, -0.5, 0.25, 0.1), step, ended=False)
    assert msg[:4] == [1.0, -0.5, 0.25, 0.1] and len(msg) == 8
    got, err = SN.pilot_fields(f32(msg))
    assert err is None and got == {"decision_age_s": 4.25, "decision_seq": 2 ** 24 - 3,
                                   "author": "vla_step", "land": False, "end": False}, got
    hover = B.action_message((0, 0, 0, 0), Tick(Action4D(), "hover", "link_lost"), ended=True)
    assert hover[4:] == [-1.0, -1.0, 3.0, 2.0], hover
    got, _ = SN.pilot_fields(f32(hover))
    assert got["decision_age_s"] is None and got["decision_seq"] is None
    assert got["author"] == "hover" and got["end"] and not got["land"]
    land = B.action_message((0, 0, 0, 0), Tick(Action4D(), "hold", "land",
                                               land_requested=True), ended=True)
    got, _ = SN.pilot_fields(f32(land))
    assert got["land"] and got["end"] and got["author"] == "hold"
    # The stub's four values: no decision, no error - read exactly as before.
    assert SN.pilot_fields([1.0, 0.0, 0.0, 0.0]) == (None, None)
    for bad in ([1, 0, 0, 0, 1.0], [1, 0, 0, 0, 1.0, 2.0, 9.0, 0.0],
                [1, 0, 0, 0, 1.0, 2.5, 1.0, 0.0], [1, 0, 0, 0, -3.0, 4.0, 1.0, 0.0],
                [1, 0, 0, 0, 1.0, 2.0, 1.0, 7.0], [1, 0, 0, 0, float("nan"), 2.0, 1.0, 0.0]):
        fields, err = SN.pilot_fields(bad)
        assert fields is None and err, (bad, err)


def test_the_bridge_agrees_with_the_shield_the_tracker_and_the_replay_rules():
    """The constants this node keeps beside other files' own: the Shield's
    mission time cap and decision timeout, the step tracker's decision
    timeout, guardrail.replay's _previous folder."""
    from guardrail.replay import PREVIOUS_DIR
    assert B.MAX_S == SN.MAX_S
    assert SN.DECISION_TIMEOUT_S == TrackerConfig().decision_timeout_s
    assert B.make_parser().get_default("decision_timeout_s") == SN.DECISION_TIMEOUT_S
    assert SN.make_parser().get_default("decision_timeout_s") == SN.DECISION_TIMEOUT_S
    assert SN.make_parser().get_default("vla_ready_timeout_s") == 0.0
    assert B.PREVIOUS_DIR == PREVIOUS_DIR
    assert B.TOPIC_ACTION == "/vla/action_4d" and B.TOPIC_IDENTITY == "/vla/identity"
    src = (ROOT / "sitl" / "ros2_vla_bridge_node.py").read_text(encoding="utf-8")
    assert "action_4d" in src                       # the to-do's probe
    # LAND is a flag: no MAVLink library, no mode request from the VLA node.
    for word in ("pymavlink", "SetMode", "set_mode", "CommandTOL", "mode_request"):
        assert word not in src, word


# =========================================================================== #
# 2. Frames and poses
# =========================================================================== #
def test_camera_frames_are_checked_in_the_callback_and_encoded_at_capture():
    f = B.image_frame(_img(4, 3, "bgr8", stamp_ns=5_000_000_123, step=16))
    assert (f["width"], f["height"], f["step"], f["time_stamp"]) == (4, 3, 16, 5_000_000_123)
    rgb = B.rgb_bytes(f)
    assert len(rgb) == 4 * 3 * 3
    plain = B.rgb_bytes(B.image_frame(_img(4, 3, "rgb8")))
    assert rgb[0::3] == plain[2::3] and rgb[2::3] == plain[0::3] and rgb[1::3] == plain[1::3]
    assert B.image_frame(_img(4, 3))["time_stamp"] is None        # an unset stamp
    for bad in (_img(enc="mono8"), SimpleNamespace(**{**vars(_img()), "data": b"\0" * 10})):
        try:
            B.image_frame(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted {bad.encoding} {len(bad.data)} bytes")
    png = B.FrameEncoder("png")(B.image_frame(_img(4, 3, "rgb8")))
    assert png.startswith(b"\x89PNG\r\n\x1a\n")
    idat = png[png.index(b"IDAT") + 4:png.index(b"IEND") - 8]
    raw = zlib.decompress(idat)
    assert b"".join(raw[r * 13 + 1:(r + 1) * 13] for r in range(3)) == plain
    try:
        import PIL.Image  # noqa: F401
        have_pil = True
    except ImportError:
        have_pil = False
    if have_pil:
        jpg = B.FrameEncoder("jpeg")(B.image_frame(_img(16, 16)))
        assert jpg[:2] == b"\xff\xd8" and B.FrameEncoder("auto").codec == "jpeg"
    else:
        assert B.FrameEncoder("auto").codec == "png"
        try:
            B.FrameEncoder("jpeg")
        except ValueError as e:
            assert "Pillow" in str(e)
        else:
            raise AssertionError("jpeg without Pillow was accepted")


def test_the_pose_ring_gives_the_pose_at_a_frames_arrival():
    r = B.PoseRing(max_gap_s=0.5)
    assert r.at(1.0) is None
    r.put(10.0, 0.0, 0.0, 15.0, math.radians(170))
    r.put(10.2, 2.0, 1.0, 16.0, math.radians(-170))
    mid = r.at(10.1)
    assert mid["src"] == "interp" and abs(mid["x"] - 1.0) < 1e-9 and abs(mid["up"] - 15.5) < 1e-9
    assert abs(abs(math.degrees(mid["yaw"])) - 180.0) < 1e-6, "the heading took the long arc"
    assert r.at(10.5)["src"] == "latest" and r.at(10.8) is None   # newer than the gap
    assert r.at(9.0) is None                                      # older than the ring


def test_the_gate_opens_when_the_shield_flies_and_follows_its_holds():
    g = B.MissionGate()
    assert g.update(1.0) is None
    g.on_state(True, "GUIDED")
    assert g.update(1.1) is None, "armed and GUIDED is the take-off, not the mission"
    g.on_setpoint()
    assert g.update(1.2) == "start" and g.live and g.t_start == 1.2
    g.on_state(True, "BRAKE")
    assert g.update(1.3) == "suspend" and not g.live
    assert g.update(1.4) is None
    g.on_state(True, "GUIDED")
    assert g.update(1.5) == "resume"
    g.on_state(False, "GUIDED")
    assert g.update(1.6) == "suspend"
    assert [e["event"] for e in g.events] == ["start", "suspend", "resume", "suspend"]


# =========================================================================== #
# 3. The pilot, closed loop
# =========================================================================== #
def test_a_goto_through_the_bridge_lands_on_the_models_land_and_reads_as_a_vla_flight():
    import todo as TODO
    from guardrail.manifest import TOPOLOGY_ARDUPILOT_SITL, build_manifest
    out = _tmp() / "ros2_vla_goto"
    goal = (52.6, 0.0)
    veh = _Veh()
    with F.FakeVLAServer(_with_model(F.goto_responder(goal, 15.0), REAL_MODEL),
                         model=REAL_MODEL) as srv:
        br = _bridge(srv, veh, out, goal)
        assert br.tick(veh.clock()) is None, "no pose yet: nothing to publish"
        _feed(br, veh)
        assert br.tick(veh.clock()) is None, "still warming up: nothing to publish"
        ident = br.connect_and_warm_up()
        assert ident["ready"] is True, ident["not_ready_because"]
        assert ident["kind"] == "vla" and ident["model_id"] == VM.MODEL_ID
        assert ident["reported_digest"] == REAL_MODEL["weights_sha256"]
        assert ident["host"] == REAL_MODEL["host"], "host is where the MODEL runs"
        assert ident["bridge_host"] and ident["goal"] == list(goal)
        assert ident["policy_hash"] == load_policy(POLICY).policy_hash
        assert ident["action_layout"] == list(B.ACTION_FIELDS)
        assert br.warmup["judged"] == "last of attempts"
        assert len(br.warmup["attempts"]) == VM.WARMUP_REQUESTS == len(srv.requests)
        # Ready, but the Shield is not flying yet: zero, author none, no request.
        for _ in range(5):
            _feed(br, veh)
            m = br.tick(veh.clock())
            veh.step(Action4D())
            assert m[:4] == [0.0, 0.0, 0.0, 0.0] and m[4:] == [-1.0, -1.0, 0.0, 0.0], m
        assert len(srv.requests) == VM.WARMUP_REQUESTS and br.rows == []
        br.on_mav_state(True, "GUIDED")
        br.on_shield_setpoint()
        msgs = _fly(br, veh, 1500)
        summary = br.close()
    assert br.end_reason == "vla_land", (br.end_reason, veh.x, veh.y)
    assert math.hypot(veh.x - goal[0], veh.y - goal[1]) < 3.0
    assert int(msgs[-1][7]) == B.FLAG_LAND | B.FLAG_END
    steps = [m for m in msgs if int(m[6]) == B.AUTHOR_CODES["vla_step"]]
    assert len(steps) > len(msgs) / 2
    assert all(m[4] >= 0.0 and m[5] >= 1.0 and m[5] == int(m[5]) for m in steps)
    assert all(m[4] == -1.0 for m in msgs if int(m[6]) in (0, 3))
    assert srv.refused == [] and srv.overlaps == 0
    for q in srv.requests:
        assert q["hint_source"] == "goal" and q["phrase"] in PHRASES
        assert not ({"target", "goal", "xy", "lat", "lon", "object_position"} & set(q))
    assert srv.requests[-1]["phrase"] == ""
    # The records, beside the Shield's.
    lines = [json.loads(x) for x in (out / "vla_steps.jsonl").read_text(
        encoding="utf-8").splitlines()]
    assert len(lines) == len(srv.requests) - VM.WARMUP_REQUESTS
    assert sum(1 for x in lines if x["kind"] == "step" and x["accepted"]) >= 10
    assert lines[-1]["kind"] == "land" and lines[-1]["accepted"]
    caps = [json.loads(x) for x in (out / "vla_captures.jsonl").read_text(
        encoding="utf-8").splitlines()]
    assert sum(1 for c in caps if c.get("warmup")) == VM.WARMUP_REQUESTS
    assert len(caps) == len(srv.requests) and all(c["front_wh"] == [8, 6] for c in caps)
    frames = {p.stem for p in (out / "vla_frames").glob("*.jpg")}
    assert {c["front_jpg_sha256"] for c in caps} <= frames
    p = summary["pilot"]
    assert p["kind"] == "vla" and p["reported_digest"] == REAL_MODEL["weights_sha256"]
    assert p["summary"]["land"] == 1 and p["summary"]["vla_authored_tick_share"] > 0.5
    assert summary["mission"]["end_reason"] == "vla_land"
    ticks = (out / "vla_bridge_ticks.jsonl").read_bytes()
    assert b"\r\n" not in ticks and len(ticks.splitlines()) == len(br.rows)
    # The Shield node files the identity: vla_model_hash names the digest,
    # and the dashboard rule reads the folder as a VLA flight.
    model_id, pilot, digest = SN.pilot_from_identity(ident)
    assert pilot["kind"] == "vla" and pilot["hash_source"] == "reported"
    manifest = build_manifest(load_policy(POLICY).policy_hash, model_id, 0, scene_path=None,
                              topology=TOPOLOGY_ARDUPILOT_SITL, reported_digest=digest)
    assert manifest["vla_model_hash"] == f"{VM.MODEL_ID}@sha256:{REAL_MODEL['weights_sha256']}"
    ev = TODO.vla_evidence(out)
    cls, why = TODO.classify_run(manifest["vla_model_hash"], ev)
    assert (cls, why) == ("vla", None), (cls, why, ev)


def test_a_stand_in_is_refused_unless_allowed_and_then_filed_as_a_stub():
    veh = _Veh()
    with F.FakeVLAServer(F.goto_responder((20.0, 0.0), 15.0)) as srv:
        br = _bridge(srv, veh, _tmp() / "stand_in", (20.0, 0.0))
        _feed(br, veh)
        ident = br.connect_and_warm_up()
        assert ident["ready"] is False and "not AerialVLA" in ident["not_ready_because"]
        assert srv.requests == [], "a refused stand-in was asked to infer"
        _feed(br, veh)
        m = br.tick(veh.clock())
        assert m[:4] == [0.0] * 4 and int(m[7]) == B.FLAG_END
        br.close()
        ok = _bridge(srv, veh, _tmp() / "stand_in_ok", (20.0, 0.0), allow_stand_in=True)
        _feed(ok, veh)
        ident = ok.connect_and_warm_up()
        ok.close()
    assert ident["ready"] is True and ident["kind"] == "stub"
    assert ident["model_id"].startswith("fake:") and ident["reported_digest"] is None
    assert "stand-in" in ident["not_a_vla_because"]
    model_id, pilot, digest = SN.pilot_from_identity(ident)
    assert pilot["kind"] == "stub" and digest is None and model_id.startswith("fake:")


def test_no_camera_pair_means_not_ready_and_zero_with_the_end_flag():
    veh = _Veh()
    with F.FakeVLAServer(_with_model(F.goto_responder((20.0, 0.0), 15.0), REAL_MODEL),
                         model=REAL_MODEL) as srv:
        br = _bridge(srv, veh, _tmp() / "no_cam", (20.0, 0.0), wait_frames_s=0.3)
        br.on_pose(veh.x, veh.y, veh.up, veh.yaw_deg())
        br.put_image("front", _img(stamp_ns=1))          # a lone front: never sent
        ident = br.connect_and_warm_up()
        m = br.tick(veh.clock())
        summary = br.close()
    assert ident["ready"] is False and "camera pair" in ident["not_ready_because"]
    assert srv.requests == []
    assert m[:4] == [0.0] * 4 and m[4:6] == [-1.0, -1.0] and int(m[7]) == B.FLAG_END
    assert SN.vla_ready(ident)[0] is False
    assert summary["ready"] is False and summary["warmup"]["pass"] is False


def test_a_dead_server_hovers_then_ends_the_mission_without_land():
    veh = _Veh()
    n = {"k": 0}

    def respond(req):
        n["k"] += 1
        if n["k"] <= VM.WARMUP_REQUESTS + 1:
            return _with_model(F.goto_responder((40.0, 0.0), 15.0), REAL_MODEL)(req)
        return F.CLOSE
    with F.FakeVLAServer(respond, model=REAL_MODEL) as srv:
        br = _bridge(srv, veh, _tmp() / "dead", (40.0, 0.0), dead_timeout_s=2.0,
                     late_reply_wait_s=0.0)
        _feed(br, veh)
        assert br.connect_and_warm_up()["ready"]
        br.on_mav_state(True, "GUIDED")
        br.on_shield_setpoint()
        msgs = _fly(br, veh, 1500)
        br.close()
    assert br.end_reason == "server_dead", br.end_reason
    authors = [int(m[6]) for m in msgs]
    assert B.AUTHOR_CODES["vla_step"] in authors and B.AUTHOR_CODES["hover"] in authors
    hovers = [m for m in msgs if int(m[6]) == B.AUTHOR_CODES["hover"]]
    assert all(m[:6] == [0.0, 0.0, 0.0, 0.0, -1.0, -1.0] for m in hovers)
    assert int(msgs[-1][7]) == B.FLAG_END, "a dead server's end is not the model's LAND"


def test_a_rail_hold_suspends_the_tracker_and_stops_the_requests():
    veh = _Veh()
    with F.FakeVLAServer(_with_model(F.goto_responder((40.0, 0.0), 15.0), REAL_MODEL),
                         model=REAL_MODEL) as srv:
        br = _bridge(srv, veh, _tmp() / "hold", (40.0, 0.0))
        _feed(br, veh)
        assert br.connect_and_warm_up()["ready"]
        br.on_mav_state(True, "GUIDED")
        br.on_shield_setpoint()
        _fly(br, veh, 40, after_end=1)
        assert br.backend.tracker.counts["accepted"] >= 1
        br.on_mav_state(True, "BRAKE")                   # the Shield's hold
        sent = len(srv.requests)
        held = _fly(br, veh, 80)
        assert len(srv.requests) == sent, "the VLA was asked during a rail hold"
        assert all(m[:4] == [0.0] * 4 and int(m[6]) == B.AUTHOR_CODES["none"] for m in held)
        br.on_mav_state(True, "GUIDED")
        _fly(br, veh, 40, after_end=1)
        br.close()
    events = [e["event"] for e in br.events]
    assert events[:3] == ["mission_start", "rail_hold", "rail_released"], events
    assert len(srv.requests) > sent, "no request after the hold ended"
    assert br.backend.tracker.counts["suspends"] >= 2


def test_a_tick_that_raises_publishes_nothing_and_is_counted():
    veh = _Veh()
    with F.FakeVLAServer(_with_model(F.goto_responder((40.0, 0.0), 15.0), REAL_MODEL),
                         model=REAL_MODEL) as srv:
        br = _bridge(srv, veh, _tmp() / "raises", (40.0, 0.0))
        _feed(br, veh)
        assert br.connect_and_warm_up()["ready"]

        def boom(state):
            raise RuntimeError("tracker exploded")
        br.backend.act = boom
        _feed(br, veh)
        assert br.safe_tick(veh.clock()) is None
        br.close()
    assert br.counts["tick_errors"] == 1 and "exploded" in br.last_errors["tick"]


def test_a_reused_directory_loses_its_old_records_to_previous():
    out = _tmp() / "reused"
    out.mkdir()
    (out / "vla_steps.jsonl").write_text('{"old": true}\n', encoding="utf-8")
    (out / "vla_bridge.json").write_text("{}", encoding="utf-8")
    (out / "metrics.json").write_text("{}", encoding="utf-8")    # the Shield's: left alone
    veh = _Veh()
    with F.FakeVLAServer(F.goto_responder((40.0, 0.0), 15.0)) as srv:
        br = _bridge(srv, veh, out, (40.0, 0.0))
        br.close()
    assert sorted(br.moved_previous) == ["vla_bridge.json", "vla_steps.jsonl"]
    moved = list((out / B.PREVIOUS_DIR).glob("*/vla_steps.jsonl"))
    assert len(moved) == 1 and "old" in moved[0].read_text(encoding="utf-8")
    assert (out / "metrics.json").is_file()


def test_the_command_line_refuses_what_cannot_fly_and_resolves_the_shields_mission():
    try:
        B.main(["--check-args"])
    except SystemExit as e:
        assert "--vla-server" in str(e)
    else:
        raise AssertionError("ran without --vla-server")
    for bad in (["--object", " padded"], ["--decision-timeout-s", "0"],
                ["--object", "the car at 12.5, 3.0"]):
        try:
            B.main(["--check-args", "--vla-server", "127.0.0.1:1", "--policy", str(POLICY)]
                   + bad)
        except SystemExit as e:
            assert "REFUSED" in str(e), (bad, e)
        else:
            raise AssertionError(f"accepted {bad}")
    argv = ["--vla-server", "10.1.2.3:8790", "--policy", str(POLICY), "--target", "-20,15",
            "--dev-yaml", "--clock-start", "2026-10-23T09:00:00", "--subject", "-3,4"]
    args, rest = B.make_parser().parse_known_args(argv)
    cfg, plan = B.resolve(args)
    assert (cfg.host, cfg.port) == ("10.1.2.3", 8790) and plan["goal"] == (-20.0, 15.0)
    assert "--dev-yaml" in rest and "--subject=-3,4" in rest, "the Shield's flags pass by"
    sargs, _ = SN.make_parser().parse_known_args(argv)
    assert SN.parse_xy(sargs.target) == plan["goal"], "the two nodes fly different targets"
    if not B.HAVE_ROS:                                # every CI job: no rclpy
        try:
            B.main(["--vla-server", "127.0.0.1:1", "--policy", str(POLICY)])
        except SystemExit as e:
            assert "needs rclpy" in str(e), e
        else:
            raise AssertionError("started without rclpy")


# =========================================================================== #
# 4. The Shield node, on the stand-in ROS stack of tests/test_railinputs_node.py
# =========================================================================== #
def _shield(argv):
    import test_railinputs_node as RN
    return RN, RN._build(argv)


def test_the_shield_flies_the_bridges_stream_and_lands_on_its_land():
    """End to end without ROS: the bridge's messages into the real ShieldNode,
    its /shield/setpoint flown by the point vehicle, the bridge's gate opened
    by that setpoint; the model's LAND ends the Shield's mission through its
    own mission-end LAND, and its records file the VLA."""
    RN, (N, node, clk, out) = _shield(["--target", "12.6,0"])
    goal = (12.6, 0.0)
    veh = _Veh()
    clk.t = veh.t
    with F.FakeVLAServer(_with_model(F.goto_responder(goal, 15.0), REAL_MODEL),
                         model=REAL_MODEL) as srv:
        br = _bridge(srv, veh, out, goal, clock=clk.monotonic)
        _feed(br, veh, clk.monotonic)
        ident = br.connect_and_warm_up()
        assert ident["ready"]
        node._on_identity(N.String(data=json.dumps(ident)))
        br.on_mav_state(True, "GUIDED")
        node.start_mission()
        pub = node.pubs[N.TOPIC_SETPOINT]
        for _ in range(600):
            veh.t = clk.t
            node.state = veh.state()
            _feed(br, veh, clk.monotonic)
            msg = br.tick(clk.monotonic())
            if br.client is not None:
                br.client.wait_idle(3.0)
            _send(N, node, msg, B.ACTION_LABEL)        # as the node publishes it
            n_before = len(pub.sent)
            clk.t = round(clk.t + 0.1, 10)
            node._tick()
            if node.done:
                break
            if len(pub.sent) > n_before:
                br.on_shield_setpoint()
                b = pub.sent[-1].data
                veh.step(from_body(b[0], b[1], b[2], b[3], veh.yaw_deg()))
            else:
                veh.step(Action4D())
        br.close()
    assert node.done and node.vla_land is not None and node.vla_land["land"], node.vla_land
    assert not node.reached, "the Shield's own reach rule ended it, not the model's LAND"
    modes = [m.data for m in node.pubs[N.TOPIC_MODE_REQUEST].sent]
    assert modes == ["LAND"], modes
    kinds = [e["kind"] for e in RN._events(out)]
    assert "vla_land_request" in kinds and kinds.index("vla_land_request") < kinds.index(
        "mission_end"), kinds
    vla_rows = [r for r in node.rows if r.get("pilot_author") == "vla_step"]
    assert vla_rows and all(r["decision_seq"] is not None and r["decision_age_s"] >= 0
                            and r["decision_stale"] is False for r in vla_rows)
    assert sum(1 for r in node.rows if r.get("flown")) > len(node.rows) / 2
    metrics = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    manifest = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
    digest = REAL_MODEL["weights_sha256"]
    assert metrics["pilot"]["kind"] == "vla" and metrics["pilot"]["reported_digest"] == digest
    assert manifest["vla_model_hash"] == f"{VM.MODEL_ID}@sha256:{digest}"
    vp = metrics["vla_pilot"]
    assert vp["extended_messages"] > 0 and vp["malformed_tails"] == 0
    assert vp["decision_watchdog"]["applies"] and vp["decision_watchdog"]["n_spells"] == 0
    assert vp["land_request"]["land"] is True and vp["layout"] == list(SN.VLA_ACTION_FIELDS)
    assert (out / "vla_steps.jsonl").is_file() and (out / "vla_bridge.json").is_file()


def _ext(N, node, age, seq, author="vla_step", flags=0, vx=1.0):
    node._on_action(N.Float32MultiArray(data=[vx, 0.0, 0.0, 0.0, age, seq,
                                              float(B.AUTHOR_CODES[author]), float(flags)]))


def test_the_shield_holds_a_stale_vla_step_and_releases_on_a_fresh_one():
    """--decision-timeout-s 1 plus the 1 s grace: a step whose decision is
    1.85 s old at the tick (the bridge's last tick of a step that reached
    its own timeout, aged in transit) still flies; 2.5 s does not."""
    RN, (N, node, clk, out) = _shield(["--decision-timeout-s", "1.0"])
    assert node.decision_watchdog.grace_s == SN.DECISION_GRACE_S == 1.0
    node.start_mission()
    seq = [(0.3, 1, "vla_step")] * 2 + [(1.75, 1, "vla_step")]
    seq += [(2.4, 1, "vla_step")] * 3 + [(0.2, 2, "vla_step")] * 2
    # A hold is judged only against its own, longer bound (2 x 1 s + 1 s).
    seq += [(2.5, 2, "hold")] * 2
    for age, s, who in seq:
        _ext(N, node, age, s, who, vx=(0.0 if who == "hold" else 1.0))
        clk.t += 0.1
        node._tick()
    rows = node.rows
    stale = [r["decision_stale"] for r in rows]
    assert stale == [False] * 3 + [True] * 3 + [False] * 4, stale
    sp = [r["setpoint"] for r in rows]
    assert sp[3] == "pass" and sp[4:6] == ["none", "none"] and sp[6] == "pass", sp
    assert rows[3]["raw_body"]["vx"] == 0.0, "the stale step itself was flown"
    modes = [m.data for m in node.pubs[N.TOPIC_MODE_REQUEST].sent]
    assert modes == ["LOITER", "GUIDED"], modes
    kinds = [e["kind"] for e in RN._events(out)]
    assert "decision_stale" in kinds and "decision_resumed" in kinds
    assert [r["pilot_author"] for r in rows[-2:]] == ["hold", "hold"]
    s = node.decision_watchdog.summary()
    assert s["n_spells"] == 1 and s["spells"][0]["decision_age_s_at_onset"] > 2.0
    assert s["spells"][0]["t_end"] is not None and s["grace_s"] == 1.0


def test_the_shield_ends_the_mission_on_the_end_flag_too():
    RN, (N, node, clk, out) = _shield([])
    node.start_mission()
    _ext(N, node, -1.0, -1.0, "hover", flags=B.FLAG_END, vx=0.0)
    clk.t += 0.1
    node._tick()
    assert node.done
    assert {k: node.vla_land[k] for k in ("land", "end", "decision_seq", "why")} == {
        "land": False, "end": True, "decision_seq": None,
        "why": "the VLA bridge ended its mission"}, node.vla_land
    assert [m.data for m in node.pubs[N.TOPIC_MODE_REQUEST].sent] == ["LAND"]


def test_the_stubs_four_values_change_nothing_in_the_shield():
    RN, (N, node, clk, out) = _shield([])
    node.start_mission()
    for _ in range(3):
        RN._act(N, node)
        clk.t += 0.1
        node._tick()
    assert all("pilot_author" not in r and "decision_stale" not in r for r in node.rows)
    assert node.n_pilot_msgs == 0 and node.n_pilot_malformed == 0 and node.pilot is None
    assert not node.decision_watchdog.applies
    assert node.pubs[N.TOPIC_MODE_REQUEST].sent == []
    node._on_identity(N.String(data=json.dumps(
        {"model_id": "guardrail.vla_stub.StubVLA", "kind": "stub", "frame": "body",
         "yaw_rate": 0.0})))
    node._finish("test over")
    metrics = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    assert metrics["pilot"]["kind"] == "stub" and "reported_digest" not in metrics["pilot"]
    assert metrics["vla_pilot"]["extended_messages"] == 0
    assert metrics["vla_pilot"]["decision_watchdog"]["applies"] is False
    assert metrics["vla_ready"] is None


def test_a_vla_identity_with_a_bad_digest_still_leaves_the_records():
    model_id, pilot, digest = SN.pilot_from_identity(
        {"model_id": VM.MODEL_ID, "kind": "vla", "reported_digest": "not-hex", "frame": "body"})
    assert digest is None and pilot["kind"] == "vla" and "reported_digest_refused" in pilot
    assert SN.pilot_from_identity(None) == ("unresolved:no-vla-identity", None, None)
    assert SN.pilot_from_identity({"model_id": "x", "kind": "none"})[1] is None
    RN, (N, node, clk, out) = _shield([])
    node.start_mission()
    node._on_identity(N.String(data=json.dumps(
        {"model_id": VM.MODEL_ID, "kind": "vla", "reported_digest": "0" * 63})))
    node._finish("test over")
    assert (out / "manifest.json").is_file() and (out / "metrics.json").is_file()


def test_the_shield_refuses_to_arm_without_a_ready_vla():
    RN, (N, node, clk, out) = _shield(["--vla-ready-timeout-s", "2"])
    N.rclpy.spin_once = lambda *a, **kw: setattr(clk, "t", clk.t + 0.5)
    for ident, ready in ((None, None), ({"ready": False, "not_ready_because": "warm-up refused: "
                                         "slow"}, False), ({"ready": True}, True)):
        node.vla_identity = ident
        try:
            node.wait_for_vla()
        except RuntimeError as e:
            assert ready is not True, e
            assert ("slow" in str(e)) if ready is False else ("within 2 s" in str(e)), e
        else:
            assert ready is True, ident
        assert node.vla_ready["ready"] is ready, node.vla_ready
    assert [e["kind"] for e in RN._events(out)].count("vla_ready") == 3
    RN2, (N2, plain, clk2, out2) = _shield([])
    plain.vla_identity = None
    plain.wait_for_vla()                              # 0: no wait, the stub's start-up
    assert plain.vla_ready is None


# =========================================================================== #
# 5. The ROS wiring, on a stand-in rclpy
# =========================================================================== #
class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _fake_ros():
    made: dict = {"subs": [], "pubs": {}, "timers": [], "log": []}

    class Pub:
        def __init__(self, topic, qos):
            self.topic, self.qos, self.sent = topic, qos, []

        def publish(self, msg):
            self.sent.append(msg)

    class Logger:
        def info(self, m):
            made["log"].append(("info", m))

        def error(self, m):
            made["log"].append(("error", m))

        warn = warning = info

    class Node:
        def __init__(self, name):
            made["name"] = name

        def get_logger(self):
            return Logger()

        def create_subscription(self, mtype, topic, cb, qos):
            made["subs"].append((mtype.__name__, topic, qos))

        def create_publisher(self, mtype, topic, qos):
            made["pubs"][topic] = Pub(topic, qos)
            return made["pubs"][topic]

        def create_timer(self, period, cb):
            made["timers"].append((period, cb))

        def destroy_node(self):
            made["destroyed"] = True

    m = {n: types.ModuleType(n) for n in (
        "rclpy", "rclpy.node", "rclpy.qos", "geometry_msgs", "geometry_msgs.msg",
        "sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg", "mavros_msgs",
        "mavros_msgs.msg")}
    m["rclpy.node"].Node = Node
    q = m["rclpy.qos"]
    q.QoSProfile = _Msg
    q.DurabilityPolicy = SimpleNamespace(TRANSIENT_LOCAL="transient_local")
    q.ReliabilityPolicy = SimpleNamespace(RELIABLE="reliable")
    q.qos_profile_sensor_data = "sensor_data"
    for mod, names in (("geometry_msgs.msg", ("PoseStamped",)), ("sensor_msgs.msg", ("Image",)),
                       ("std_msgs.msg", ("Float32MultiArray", "MultiArrayDimension",
                                         "MultiArrayLayout", "String")),
                       ("mavros_msgs.msg", ("State",))):
        for n in names:
            setattr(m[mod], n, type(n, (_Msg,), {}))
    return m, made


def test_the_node_wires_its_topics_and_publishes_the_layout():
    fakes, made = _fake_ros()
    name = "ros2_vla_bridge_node_standin"
    saved = {k: sys.modules.get(k) for k in [*fakes, name]}
    sys.modules.update(fakes)
    try:
        spec = importlib.util.spec_from_file_location(
            name, ROOT / "sitl" / "ros2_vla_bridge_node.py")
        M = importlib.util.module_from_spec(spec)
        sys.modules[name] = M                 # @dataclass looks its module up
        spec.loader.exec_module(M)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    assert M.HAVE_ROS, "the stand-in ROS stack did not import"
    policy, mission = _mission((20.0, 0.0))
    cfg = M.BridgeConfig(host="127.0.0.1", port=9, front_topic="/cam/f", down_topic="/cam/d")
    core = M.VLABridge(cfg, mission=mission, policy=policy, policy_source={"kind": "yaml"},
                       out=_tmp() / "node")
    said = threading.Event()

    def warm():
        core.ready, core.not_ready_because = False, "stand-in: no server"
        said.set()
        return core.identity_record()
    core.connect_and_warm_up = warm
    node = M.VlaBridgeNode(core)
    node._warm.join(5.0)
    assert said.is_set() and made["name"] == "vla_bridge"
    subs = {(t, topic) for t, topic, _ in made["subs"]}
    assert subs == {("PoseStamped", "/mavros/local_position/pose"), ("State", "/mavros/state"),
                    ("Float32MultiArray", "/shield/setpoint"), ("Image", "/cam/f"),
                    ("Image", "/cam/d")}, subs
    assert all(qos == "sensor_data" for t, _, qos in made["subs"] if t in ("Image", "PoseStamped"))
    assert set(made["pubs"]) == {"/vla/action_4d", "/vla/identity"}
    idq = made["pubs"]["/vla/identity"].qos
    assert idq.durability == "transient_local" and idq.depth == 1
    ident = json.loads(made["pubs"]["/vla/identity"].sent[0].data)
    assert ident["ready"] is False and ident["camera_topics"] == {"front": "/cam/f",
                                                                  "down": "/cam/d"}
    assert made["timers"][0][0] == 0.1
    q = math.sin(math.radians(90.0 - 30.0) / 2), math.cos(math.radians(90.0 - 30.0) / 2)
    node._on_pose(_Msg(pose=_Msg(position=_Msg(x=4.0, y=3.0, z=12.0),
                                 orientation=_Msg(x=0.0, y=0.0, z=q[0], w=q[1]))))
    st = core.state_now()
    assert (st.x, st.y, st.up) == (3.0, 4.0, 12.0) and abs(st.yaw_deg - 30.0) < 1e-6
    node._on_state(_Msg(armed=True, mode="GUIDED"))
    node._on_setpoint(_Msg(data=[0.0] * 4))
    assert core.gate.armed and core.gate.mode == "GUIDED" and core.gate.setpoint_seen
    node._on_front(_img(enc="mono8"))
    node._on_front(_img(enc="mono8"))
    assert [lvl for lvl, msg in made["log"] if "front camera frame refused" in msg] == ["error"]
    assert core.counts["front_refused"] == 2
    made["timers"][0][1]()                            # one 10 Hz tick
    out = made["pubs"]["/vla/action_4d"].sent[-1]
    assert out.layout.dim[0].label == B.ACTION_LABEL and out.layout.dim[0].size == 8
    assert list(out.data) == [0.0] * 4 + [-1.0, -1.0, 0.0, float(B.FLAG_END)]
    core.close()


# =========================================================================== #
# 6. The review of 339f23a (vla_disc/review-V2c-339f23a.md)
# =========================================================================== #
STEP = B.AUTHOR_CODES["vla_step"]
HOLD = B.AUTHOR_CODES["hold"]
NONE = B.AUTHOR_CODES["none"]


def _send(N, node, data, label=None):
    kw = {"data": list(data)}
    if label is not None:
        kw["layout"] = _Msg(dim=[_Msg(label=label, size=len(data), stride=len(data))])
    node._on_action(N.Float32MultiArray(**kw))


def _run(N, node, clk, msgs, label=None):
    for d in msgs:
        _send(N, node, d, label)
        clk.t += 0.1
        node._tick()
    return [m.data for m in node.pubs[N.TOPIC_MODE_REQUEST].sent]


def test_p1a_a_malformed_tail_after_extended_messages_is_held_never_flown():
    """Probe1: two stale steps, then a NaN-age message moving at 2 m/s, was
    flown, counted as a receipt and released the hold (LOITER, GUIDED). Now
    any message after the bridge's extended ones that is not a whole valid
    tail - a NaN, 5 values, the grant's 4 alone (a second publisher), a seq
    going backwards - is refused: not flown, not a receipt, held, and kept in
    the row as what the VLA sent."""
    RN, (N, node, clk, out) = _shield(["--decision-timeout-s", "1.0"])
    node.start_mission()
    fresh = [1.0, 0.0, 0.0, 0.0, 0.2, 4.0, STEP, 0.0]
    modes = _run(N, node, clk, [fresh, fresh])
    n_ok = node.n_actions
    bad = [[2.0, 0, 0, 0, float("nan"), 4.0, STEP, 0.0], [2.0, 0, 0, 0, 0.3],
           [2.0, 0.0, 0.0, 0.0], [2.0, 0.0, 0.0, 0.0, 0.2, 3.0, STEP, 0.0]]
    modes = _run(N, node, clk, bad)
    assert node.n_actions == n_ok, "a refused message counted as a receipt"
    assert node.n_pilot_malformed == 4 and "backwards" in node.pilot_error, node.pilot_error
    assert modes == ["LOITER"], modes
    rows = node.rows[2:]
    assert all(r["raw_body"]["vx"] == 0.0 for r in rows), "a refused message was flown"
    assert [r["setpoint"] for r in rows] == ["pass", "none", "none", "none"]
    assert all(r["decision_stale"] for r in rows) and rows[0]["pilot_sent"][0] == 2.0
    assert rows[-1]["pilot_refused"] and "backwards" in rows[-1]["pilot_refused"]
    kinds = [e["kind"] for e in RN._events(out)]
    assert kinds.count("vla_action_refused") == 4, kinds
    modes = _run(N, node, clk, [[1.0, 0, 0, 0, 0.2, 5.0, STEP, 0.0]])
    assert modes == ["LOITER", "GUIDED"], modes
    assert node.rows[-1]["setpoint"] == "pass" and node.rows[-1]["raw_body"]["vx"] == 1.0


def test_n1_a_decision_spell_ends_on_the_next_valid_message():
    """Re-review of 23c88d3, N1. 23c88d3 ended a spell only on a fresh
    vla_step, which the hold itself prevents: the Shield's LOITER puts a sound
    bridge in BRAKE, it suspends and sends none ticks, and the mission landed
    at the escalation. A spell now ends on the next valid message that is not
    itself stale - a none tick included - and a bridge that keeps sending
    stale or refused messages is caught by the repeated-spell count (N3)."""
    RN, (N, node, clk, out) = _shield(["--decision-timeout-s", "1.0"])
    node.start_mission()
    stale = [1.0, 0, 0, 0, 3.0, 2.0, STEP, 0.0]
    none = [0.0, 0.0, 0.0, 0.0, -1.0, -1.0, NONE, 0.0]
    modes = _run(N, node, clk, [stale, stale, none])
    assert modes == ["LOITER", "GUIDED"], modes
    assert [r["decision_stale"] for r in node.rows] == [True, True, False]
    assert not node.done


def test_n1_silence_never_ages_a_decision_past_the_receipt_watchdog():
    """N1's cause: the decision watchdog aged the last message through a
    silence, so a 2 s stall near the end of a sound step (decision 9 s old)
    started a decision spell. Silence is the receipt watchdog's: the aging
    term is capped at --action-timeout-s, and a message that itself carries
    an old age is still caught."""
    w = SN.DecisionWatchdog(10.0, max_silence_s=0.5)
    step9 = {"decision_age_s": 9.0, "decision_seq": 3, "author": "vla_step"}
    hold19 = {"decision_age_s": 19.0, "decision_seq": 3, "author": "hold"}
    assert w.age(103.0, step9, 100.0) == 9.5
    assert w.verdict(103.0, step9, 100.0) is None and w.verdict(102.5, hold19, 100.0) is None
    assert w.verdict(100.0, {**step9, "decision_age_s": 11.2}, 100.0) == "stale step"


def test_n1_a_stall_near_the_end_of_a_sound_step_holds_then_resumes_never_lands():
    """The re-review's end-to-end probe: the real bridge flies a 5 m step at
    0.5 m/s, so its decision is 9.5 s old near the step's end; there its
    messages stop reaching the Shield for 2 s (a DDS or CPU stall), then flow
    again. The receipt watchdog holds (LOITER), the bridge sees BRAKE and
    suspends, its ticks release the hold (GUIDED), and it flies on: no
    decision spell, no LAND (23c88d3: LOITER, LAND at x 4.8 of 40)."""
    RN, (N, node, clk, out) = _shield(["--target", "40,0"])
    goal = (40.0, 0.0)
    veh = _Veh()
    clk.t = veh.t
    dropped_at, drop_left, after = None, 0, []
    with F.FakeVLAServer(_with_model(F.goto_responder(goal, 15.0), REAL_MODEL),
                         model=REAL_MODEL) as srv:
        br = _bridge(srv, veh, out, goal, clock=clk.monotonic, cruise_mps=0.5)
        _feed(br, veh, clk.monotonic)
        node._on_identity(N.String(data=json.dumps(br.connect_and_warm_up())))
        br.on_mav_state(True, "GUIDED")
        node.start_mission()
        pub, modes = node.pubs[N.TOPIC_SETPOINT], node.pubs[N.TOPIC_MODE_REQUEST]
        for k in range(400):
            veh.t = clk.t
            node.state = veh.state()
            _feed(br, veh, clk.monotonic)
            msg = br.tick(clk.monotonic())
            if br.client is not None:
                br.client.wait_idle(3.0)
            if dropped_at is None and msg[6] == STEP and msg[4] >= 9.5:
                dropped_at, drop_left = k, 20
            if drop_left > 0:
                drop_left -= 1                       # lost on the way to the Shield
            else:
                _send(N, node, msg, B.ACTION_LABEL)
            if dropped_at is not None and k > dropped_at + 60:
                after.append(int(msg[6]))
            n_sp, n_mode = len(pub.sent), len(modes.sent)
            clk.t = round(clk.t + 0.1, 10)
            node._tick()
            for m in modes.sent[n_mode:]:
                br.on_mav_state(True, {"LOITER": "BRAKE"}.get(m.data, m.data))
            if node.done:
                break
            if len(pub.sent) > n_sp:
                br.on_shield_setpoint()
                b = pub.sent[-1].data
                veh.step(from_body(b[0], b[1], b[2], b[3], veh.yaw_deg()))
            else:
                veh.step(Action4D())
        br.close()
    assert dropped_at is not None, "the step never reached a 9.5 s old decision"
    sent = [m.data for m in modes.sent]
    assert sent[:2] == ["LOITER", "GUIDED"] and "LAND" not in sent, sent
    assert not node.done, "a sound bridge's 2 s stall ended the mission"
    kinds = [e["kind"] for e in RN._events(out)]
    assert "action_stale" in kinds and "decision_stale" not in kinds, kinds
    assert STEP in after, "the bridge did not fly on after the stall"


def test_p1a_a_layout_label_that_is_not_the_contract_is_refused():
    RN, (N, node, clk, out) = _shield([])
    node.start_mission()
    ok = [1.0, 0, 0, 0, 0.2, 1.0, STEP, 0.0]
    _run(N, node, clk, [ok], label=B.ACTION_LABEL)
    assert node.n_pilot_msgs == 1 and node.n_pilot_malformed == 0
    swapped = ",".join(["vy_right", "vx_fwd"] + list(B.ACTION_FIELDS[2:]))
    _run(N, node, clk, [ok], label=swapped)
    assert node.n_pilot_malformed == 1 and "label" in node.pilot_error, node.pilot_error


def test_p1a_the_bridges_identity_makes_the_four_values_alone_a_refusal():
    """Before any extended message has arrived, a /vla/identity announcing the
    bridge's action_layout already makes a plain four-value message (a stub
    left running) a refusal."""
    RN, (N, node, clk, out) = _shield([])
    node.start_mission()
    node._on_identity(N.String(data=json.dumps(
        {"model_id": VM.MODEL_ID, "kind": "vla", "action_layout": list(B.ACTION_FIELDS)})))
    _run(N, node, clk, [[2.0, 0.0, 0.0, 0.0]])
    assert node.n_pilot_malformed == 1 and node.rows[-1]["raw_body"]["vx"] == 0.0


def _publishers(node, counts, names=("vla_bridge", "vla_stub")):
    """count_publishers / get_publishers_info_by_topic stand-ins that answer
    counts[i] on the i-th check (the last one after that)."""
    k = {"i": 0}

    def count(topic):
        n = counts[min(k["i"], len(counts) - 1)] if topic == "/vla/action_4d" else 0
        k["i"] += 1
        return n

    def info(topic):
        n = counts[min(max(k["i"] - 1, 0), len(counts) - 1)]
        return [SimpleNamespace(node_name=names[i % len(names)], topic_name=topic)
                for i in range(n)]
    node.count_publishers, node.get_publishers_info_by_topic = count, info


def test_p1a_a_second_publisher_is_recorded_and_the_run_is_not_kpi_grade():
    RN, (N, node, clk, out) = _shield([])
    _publishers(node, [2])
    node.start_mission()
    _run(N, node, clk, [[1.0, 0, 0, 0, 0.2, 1.0, STEP, 0.0]] * 35)
    node._finish("test over")
    metrics = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    kpi = json.loads((out / "kpi.json").read_text(encoding="utf-8"))
    assert metrics["vla_pilot"]["action_publishers_max"] == 2, metrics["vla_pilot"]
    assert metrics["vla_pilot"]["action_publishers_flagged"] is True
    assert set(metrics["vla_pilot"]["action_publisher_names"]) == {"vla_bridge", "vla_stub"}
    assert any("publishers" in w for w in kpi["kpi_grade_reasons"]), kpi["kpi_grade_reasons"]
    assert kpi["kpi_grade"] is False


def test_n7_a_passing_second_publisher_is_recorded_but_does_not_cost_the_grade():
    """N7: one check that saw two publishers (a ghost after a kill -9, until
    its lease expires) took the KPI grade off the whole run. It is recorded;
    only ACTION_PUBLISHERS_CHECKS consecutive checks flag it."""
    RN, (N, node, clk, out) = _shield([])
    _publishers(node, [2, 1, 2, 1, 1])
    node.start_mission()
    _run(N, node, clk, [[1.0, 0, 0, 0, 0.2, 1.0, STEP, 0.0]] * 45)
    node._finish("test over")
    vp = json.loads((out / "metrics.json").read_text(encoding="utf-8"))["vla_pilot"]
    kpi = json.loads((out / "kpi.json").read_text(encoding="utf-8"))
    assert vp["action_publishers_max"] == 2 and vp["action_publishers_flagged"] is False, vp
    assert not any("publishers" in w for w in kpi["kpi_grade_reasons"])
    assert SN.ACTION_PUBLISHERS_CHECKS >= 3


def test_p1a_a_decision_hold_that_lasts_ends_the_mission():
    """P3-1: a decision hold had no bound but max_s. Past
    --decision-hold-max-s the mission ends through the mission-end LAND."""
    RN, (N, node, clk, out) = _shield(["--decision-timeout-s", "1.0",
                                       "--decision-hold-max-s", "1.0"])
    node.start_mission()
    stale = [1.0, 0, 0, 0, 3.0, 2.0, STEP, 0.0]
    for _ in range(30):
        _send(N, node, stale)
        clk.t += 0.1
        node._tick()
        if node.done:
            break
    assert node.done, "a 3 s decision hold did not end"
    assert [m.data for m in node.pubs[N.TOPIC_MODE_REQUEST].sent] == ["LOITER", "LAND"]
    kinds = [e["kind"] for e in RN._events(out)]
    assert "decision_hold_escalated" in kinds, kinds


def test_p2_1_no_author_but_vla_step_may_move_and_a_hold_has_its_own_age_bound():
    """A hold, hover or none tick with horizontal or yaw motion (probe: a hold
    at 2 m/s on a 500 s old decision flew forever), a hover or none that
    climbs, a hold with no decision that climbs, and a vla_step with no
    decision are malformed; a hold older than 2 x timeout + grace is stale
    (the tracker's longest sound hold: a step completes within the timeout of
    its capture, then holds at most the timeout)."""
    for d in ([2.0, 0, 0, 0, 1.0, 1, HOLD, 0], [0, 0, 0, 0.4, 1.0, 1, HOLD, 0],
              [0, 0, 0.5, 0, -1, -1, HOLD, 0], [0, 0, 0.5, 0, -1, -1, B.AUTHOR_CODES["hover"], 0],
              [0.3, 0, 0, 0, -1, -1, NONE, 0], [2.0, 0, 0, 0, -1, -1, STEP, 0]):
        fields, err = SN.pilot_fields(d)
        assert fields is None and err, (d, fields)
    assert SN.pilot_fields([0, 0, 0.5, 0, 4.0, 2, HOLD, 0])[1] is None
    assert SN.pilot_fields([0, 0, 0, 0, -1, -1, HOLD, B.FLAG_LAND | B.FLAG_END])[1] is None
    RN, (N, node, clk, out) = _shield(["--decision-timeout-s", "1.0"])
    node.start_mission()
    _run(N, node, clk, [[0, 0, 0.3, 0, 2.8, 2, HOLD, 0], [0, 0, 0.3, 0, 3.2, 2, HOLD, 0]])
    assert [r["decision_stale"] for r in node.rows] == [False, True]
    assert node.rows[1]["raw_body"]["vz_up"] == 0.0


def test_p1b_bring_up_arms_after_a_long_wait_for_the_vla():
    """Probe2b: the bridge's identity after 900 s (a cold bf16 load) left the
    shared 180 s bring-up deadline spent, and bring_up raised "arming timed
    out (EKF)" without trying. The deadline for GUIDED, arming and the climb
    now starts after the wait; an arming that is refused says how often."""
    RN, (N, node, clk, out) = _shield(["--vla-ready-timeout-s", "3300"])
    calls = []

    def _prep(arm_ok=True):
        node.mav_connected = True
        node.fence_status = [{"success": True}]
        node.fence_column_m = 100.0
        for name in ("read_autopilot_version", "read_link", "read_autopilot_fence",
                     "read_sim_speedup"):
            setattr(node, name, lambda: None)
        node._publish_fences = lambda *a, **k: None

        def fake_call(client, req, timeout=5.0):
            calls.append(client)
            ok = arm_ok or client is not node.cli_arm
            return SimpleNamespace(mode_sent=ok, success=ok)
        node._call = fake_call
    _prep()
    ident = {"ready": True, "model_id": VM.MODEL_ID, "kind": "vla",
             "decision_timeout_s": SN.DECISION_TIMEOUT_S, "action_layout": list(B.ACTION_FIELDS)}

    def spin_once(*a, **kw):
        clk.t += 0.5
        if clk.t - 1000.0 > 900.0 and node.vla_identity is None:
            node.vla_identity = dict(ident)
    N.rclpy.spin_once = spin_once
    node.bring_up()
    assert node.vla_ready["ready"] is True and node.vla_ready["waited_s"] > 900.0
    assert node.cli_arm in calls and node.cli_tol in calls, "never armed after the wait"
    RN2, (N2, node2, clk2, out2) = _shield([])
    node, clk, N, calls = node2, clk2, N2, []
    _prep(arm_ok=False)
    N.rclpy.spin_once = lambda *a, **kw: setattr(clk, "t", clk.t + 0.5)
    try:
        node.bring_up()
    except RuntimeError as e:
        assert "refused" in str(e) and "EKF" not in str(e), e
        n = sum(1 for c in calls if c is node.cli_arm)
        assert n > 10 and f"{n} time" in str(e), (n, e)
    else:
        raise AssertionError("arming that never succeeds was not refused")


def test_p3_the_ready_gate_refuses_a_stub_and_a_contract_mismatch():
    """vla_ready took an identity with no `ready` key (a stray stub) as
    ready; a bridge whose decision timeout or layout is not this Shield's
    armed anyway. The gate refuses all three now."""
    assert SN.vla_ready({"model_id": "guardrail.vla_stub.StubVLA", "kind": "stub"})[0] is False
    RN, (N, node, clk, out) = _shield(["--vla-ready-timeout-s", "5",
                                       "--decision-timeout-s", "10"])
    N.rclpy.spin_once = lambda *a, **kw: setattr(clk, "t", clk.t + 0.5)
    good = {"ready": True, "decision_timeout_s": 10.0, "action_layout": list(B.ACTION_FIELDS)}
    for ident, why in (({**good, "decision_timeout_s": 4.0}, "decision"),
                       ({**good, "action_layout": ["vx_fwd"]}, "layout"), ({"kind": "stub"}, "ready")):
        node.vla_identity = ident
        try:
            node.wait_for_vla()
        except RuntimeError as e:
            assert why in str(e), (why, e)
        else:
            raise AssertionError(f"armed with {ident}")
    node.vla_identity = good
    node.wait_for_vla()
    assert node.vla_ready["ready"] is True


def test_p3_the_shield_waits_longer_than_the_bridge_says_hello_and_warms_up():
    """The Shield's default wait with VLA=bridge equalled the bridge's hello
    timeout (2700 s): a late hello plus the warm-up lost the race."""
    import re
    warm = (VM.WARMUP_REQUESTS * B.WAIT_FRAMES_S + 60.0
            + (VM.WARMUP_REQUESTS - 1) * B.BridgeConfig("h", 1).decision_timeout_s
            + VM.WARMUP_REQUESTS * 2.0)
    launch = (ROOT / "sitl" / "ros2_ws" / "src" / "safety_shield" / "launch"
              / "guardrail_rail.launch.py").read_text(encoding="utf-8")
    sh = (ROOT / "sitl" / "run_ros2_demo.sh").read_text(encoding="utf-8")
    lw = float(re.search(r'VLA_READY_TIMEOUT_S = "(\d+)"', launch).group(1))
    sw = float(re.search(r"VLA_READY_TIMEOUT_S:-(\d+)", sh).group(1))
    assert lw == sw > VM.HELLO_TIMEOUT_S + warm, (lw, sw, VM.HELLO_TIMEOUT_S + warm)


def test_p3_identities_that_are_not_a_ready_vla_are_never_filed_as_one():
    """Probe3: a fake: id announced as kind vla kept kind vla; a bridge that
    never became ready was filed kind vla with a resolved vla_model_hash."""
    from guardrail.manifest import TOPOLOGY_ARDUPILOT_SITL, build_manifest
    mid, pilot, digest = SN.pilot_from_identity(
        {"model_id": "fake:scripted", "kind": "vla", "reported_digest": "ab" * 32})
    assert pilot["kind"] == "stub" and digest is None and mid == "fake:scripted", pilot
    mid, pilot, digest = SN.pilot_from_identity(
        {"model_id": VM.MODEL_ID, "kind": "vla", "ready": False,
         "not_ready_because": "warm-up refused: slow", "reported_digest": "ab" * 32})
    assert digest is None and pilot["not_ready_because"] == "warm-up refused: slow"
    m = build_manifest("0" * 64, mid, 0, scene_path=None, topology=TOPOLOGY_ARDUPILOT_SITL,
                       reported_digest=digest)
    assert m["vla_model_hash"].endswith("@unresolved"), m["vla_model_hash"]


def test_p3_decision_stale_rows_keep_what_the_vla_sent():
    RN, (N, node, clk, out) = _shield(["--decision-timeout-s", "1.0"])
    node.start_mission()
    _run(N, node, clk, [[1.5, -0.5, 0.0, 0.2, 3.0, 2.0, STEP, 0.0]])
    r = node.rows[-1]
    assert r["decision_stale"] and r["raw_body"]["vx"] == 0.0
    assert r["pilot_sent"] == [1.5, -0.5, 0.0, 0.2], r["pilot_sent"]


def test_p3_a_vla_land_records_where_and_in_which_state_it_lands():
    RN, (N, node, clk, out) = _shield([])
    node.start_mission()
    node.state = State(x=15.0, y=15.0, up=15.0, yaw_deg=0.0)     # inside nfz-square
    _run(N, node, clk, [[0, 0, 0, 0, -1, -1, HOLD, B.FLAG_LAND | B.FLAG_END]])
    land = node.vla_land
    assert land["position"] == {"x": 15.0, "y": 15.0, "up": 15.0}, land
    assert land["position_unsafe_rules"] == ["nfz-square"], land
    assert land["fsm_state"] is not None and "in place" in land["landing"]


def test_p2_3_the_bridge_publishes_nothing_on_a_stale_pose():
    """A stalled pose subscription flew a frozen pose: a 5 m step to its time
    cap and a spin at the yaw cap. Past --pose-max-age-s the bridge publishes
    nothing (the Shield's receipt watchdog holds) and counts it."""
    veh = _Veh()
    with F.FakeVLAServer(_with_model(F.goto_responder((40.0, 0.0), 15.0), REAL_MODEL),
                         model=REAL_MODEL) as srv:
        br = _bridge(srv, veh, _tmp() / "stale_pose", (40.0, 0.0))
        _feed(br, veh)
        assert br.connect_and_warm_up()["ready"]
        br.on_mav_state(True, "GUIDED")
        br.on_shield_setpoint()
        assert br.tick(veh.clock()) is not None
        veh.t += 1.5                                  # no pose for 1.5 s
        assert br.tick(veh.clock()) is None and br.tick(veh.clock() + 0.1) is None
        _feed(br, veh)
        assert br.tick(veh.clock()) is not None
        br.close()
    assert br.counts["ticks_pose_stale"] == 2
    assert [e["event"] for e in br.events if "pose" in e["event"]] == ["pose_stale",
                                                                       "pose_resumed"]


def test_p3_the_capture_time_is_the_image_stamp_when_its_clock_is_this_hosts():
    """The capture pose and time were the front frame's arrival; camera
    latency was ignored. A header stamp on this host's wall clock now gives
    the capture time (and the pose then), a stamp on another clock (sim time,
    another host: more than 2 s away) falls back to the arrival."""
    veh = _Veh()
    wall = {"t": 5000.0}
    with F.FakeVLAServer(F.goto_responder((40.0, 0.0), 15.0)) as srv:
        policy, mission = _mission((40.0, 0.0))
        c = B.BridgeConfig(host="127.0.0.1", port=srv.port)
        br = B.VLABridge(c, mission=mission, policy=policy, policy_source={}, out=_tmp() / "st",
                         clock=veh.clock, sleep=lambda s: None, wall=lambda: wall["t"])
        br.on_pose(0.0, 0.0, 15.0, 0.0, t=veh.t - 0.3)
        br.on_pose(3.0, 0.0, 15.0, 0.0, t=veh.t)
        stamp = int((wall["t"] - 0.15) * 1e9)            # taken 0.15 s before arrival
        br.put_image("front", _img(stamp_ns=stamp, seed=1))
        br.put_image("down", _img(stamp_ns=stamp, seed=2))
        cap = br.frames.capture()
        assert abs(cap.t_capture - (veh.t - 0.15)) < 1e-6, cap.t_capture
        assert abs(cap.pose.x - 1.5) < 1e-6, cap.pose        # the pose 0.15 s earlier
        far = int((wall["t"] - 900.0) * 1e9)              # another clock
        br.put_image("front", _img(stamp_ns=far, seed=1))
        br.put_image("down", _img(stamp_ns=far, seed=2))
        cap = br.frames.capture()
        assert cap.t_capture == veh.t and cap.pose.x == 3.0
        br.close()


def test_p2_4_the_bridge_writes_its_records_on_shutdown_and_sigterm():
    """USE_LAUNCH=0 SIGKILLed the bridge in the cleanup trap after the pack:
    close() never ran. The run script now TERMs the VLA node and waits before
    the pack (and in the trap, before any kill -9); the bridge turns SIGTERM
    into the same shutdown as Ctrl+C, and main() writes the records whatever
    ends the spin (here a stand-in rclpy whose spin is interrupted)."""
    import signal
    sh = (ROOT / "sitl" / "run_ros2_demo.sh").read_text(encoding="utf-8")
    body = sh.split("cleanup() {", 1)[1].split("\n}", 1)[0]
    assert body.index("stop_vla") < body.index("pkill -9"), "the trap kills the bridge first"
    assert 'kill -TERM "$VLA_PID"' in sh and "VLA_PID=$!" in sh
    after_shield = sh.split('"$DIR/ros2_shield_node.py" "${SHIELD_ARGS[@]}"', 1)[1]
    assert "stop_vla" in after_shield.split("guardrail.replay pack", 1)[0], \
        "the replay pack runs before the bridge has written its records"
    try:
        B.on_sigterm(signal.SIGTERM, None)
    except KeyboardInterrupt:
        pass
    else:
        raise AssertionError("SIGTERM does not stop the spin")
    fakes, made = _fake_ros()
    fakes["rclpy"].init = lambda args=None: None
    fakes["rclpy"].ok = lambda: False

    def spin(node):
        raise KeyboardInterrupt
    fakes["rclpy"].spin = spin
    name = "ros2_vla_bridge_node_shutdown"
    saved = {k: sys.modules.get(k) for k in [*fakes, name]}
    sys.modules.update(fakes)
    old = {s: signal.getsignal(s) for s in (signal.SIGTERM, signal.SIGINT)}
    out = _tmp() / "shutdown"
    try:
        spec = importlib.util.spec_from_file_location(name, ROOT / "sitl" / "ros2_vla_bridge_node.py")
        M = importlib.util.module_from_spec(spec)
        sys.modules[name] = M
        spec.loader.exec_module(M)
        import socket
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()                                     # nothing listens there
        rc = M.main(["--vla-server", f"127.0.0.1:{port}", "--policy", str(POLICY),
                     "--out", str(out), "--vla-hello-timeout-s", "0.2"])
    finally:
        for sig, h in old.items():
            signal.signal(sig, h)
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    assert rc == 0 and (out / "vla_bridge.json").is_file(), list(out.iterdir())
    assert (out / "vla_captures.jsonl").is_file() and (out / "vla_bridge_ticks.jsonl").is_file()


def test_n1_a_stray_refused_message_between_steps_holds_once_and_does_not_land():
    """The review's end-to-end gap, as the re-review of 23c88d3 (N1) settles
    it: a stray four-value message (a stub's) lands among the bridge's
    stream; the Shield refuses it and holds (LOITER, flown as BRAKE), the
    bridge sees BRAKE and suspends, its next valid tick releases the hold
    (GUIDED), the bridge resumes and flies on. 23c88d3 waited for a fresh
    vla_step the suspended bridge never sent, and landed."""
    RN, (N, node, clk, out) = _shield(["--target", "40,0", "--decision-hold-max-s", "2.0"])
    goal = (40.0, 0.0)
    veh = _Veh()
    clk.t = veh.t
    with F.FakeVLAServer(_with_model(F.goto_responder(goal, 15.0), REAL_MODEL),
                         model=REAL_MODEL) as srv:
        br = _bridge(srv, veh, out, goal, clock=clk.monotonic)
        _feed(br, veh, clk.monotonic)
        node._on_identity(N.String(data=json.dumps(br.connect_and_warm_up())))
        br.on_mav_state(True, "GUIDED")
        node.start_mission()
        pub, modes = node.pubs[N.TOPIC_SETPOINT], node.pubs[N.TOPIC_MODE_REQUEST]
        authors_after = []
        for k in range(200):
            veh.t = clk.t
            node.state = veh.state()
            _feed(br, veh, clk.monotonic)
            msg = br.tick(clk.monotonic())
            if br.client is not None:
                br.client.wait_idle(3.0)
            _send(N, node, msg, B.ACTION_LABEL)
            if k == 30:
                node._on_action(N.Float32MultiArray(data=[2.0, 0.0, 0.0, 0.0]))
            if k > 80:
                authors_after.append(int(msg[6]))
            n_sp, n_mode = len(pub.sent), len(modes.sent)
            clk.t = round(clk.t + 0.1, 10)
            node._tick()
            for m in modes.sent[n_mode:]:
                br.on_mav_state(True, {"LOITER": "BRAKE"}.get(m.data, m.data))
            if node.done:
                break
            if len(pub.sent) > n_sp:
                br.on_shield_setpoint()
                b = pub.sent[-1].data
                veh.step(from_body(b[0], b[1], b[2], b[3], veh.yaw_deg()))
            else:
                veh.step(Action4D())
        br.close()
    sent = [m.data for m in modes.sent]
    assert sent == ["LOITER", "GUIDED"] and not node.done, (sent, node.done)
    assert STEP in authors_after, "the bridge did not fly on after the hold"
    kinds = [e["kind"] for e in RN._events(out)]
    assert "vla_action_refused" in kinds and "decision_resumed" in kinds, kinds
    assert "decision_hold_escalated" not in kinds
    assert [e["event"] for e in br.events][:3] == ["mission_start", "rail_hold",
                                                    "rail_released"]


def test_p3_the_bridges_default_directory_is_the_shields():
    """A start without --out wrote to demo/out/ros2_vla_bridge, not beside the
    Shield's records: the default is now the Shield's own directory rule."""
    for argv, tag in (([], "ros2_shield_on"), (["--shield", "off", "--dynamic"],
                                                "ros2_shield_off_dynamic"),
                      (["--tag", "t1"], "t1")):
        args, _ = B.make_parser().parse_known_args(
            ["--vla-server", "h:1", "--policy", str(POLICY)] + argv)
        _, plan = B.resolve(args)
        assert plan["out"] == ROOT / "demo" / "out" / tag, (argv, plan["out"])
    assert SN.default_tag("off", True) == "ros2_shield_off_dynamic"


# =========================================================================== #
# 7. The re-review of 23c88d3 (vla_disc/rereview-V2c-23c88d3.md; rerev23c/)
# =========================================================================== #
def test_n2_a_malformed_identity_layout_never_raises():
    """identity_contract_problems did list(layout): TypeError for 5 or true,
    inside _on_identity (a subscription callback) and wait_for_vla."""
    for bad in (5, True, "vx_fwd", {"a": 1}):
        probs = SN.identity_contract_problems({"action_layout": bad}, 10.0)
        assert probs and "layout" in probs[0], (bad, probs)
    assert SN.identity_contract_problems({"decision_timeout_s": "ten"}, 10.0)
    RN, (N, node, clk, out) = _shield(["--vla-ready-timeout-s", "5"])
    node._on_identity(N.String(data=json.dumps({"ready": True, "action_layout": 5})))
    assert node.contract_problems and node.vla_identity["action_layout"] == 5
    try:
        node.wait_for_vla()
    except RuntimeError as e:
        assert "layout" in str(e), e
    else:
        raise AssertionError("armed with a malformed layout")


def test_n3_repeated_holds_end_the_mission_instead_of_flapping():
    """A stub left running beside a live bridge: its messages and the
    bridge's alternate, each stub message is refused and held, each bridge
    message releases (N1), and the aircraft flapped LOITER / GUIDED (the
    re-review's probe: 100 mode requests, 50 holds in 30 s, no escalation).
    DECISION_SPELLS_MAX holds within DECISION_SPELLS_WINDOW_S end the mission
    through the mission-end LAND."""
    RN, (N, node, clk, out) = _shield([])
    node.start_mission()
    good = [1.0, 0, 0, 0, 0.5, 4.0, STEP, 0.0]
    for k in range(60):
        _send(N, node, good, B.ACTION_LABEL)
        if k % 4 == 1:
            node._on_action(N.Float32MultiArray(data=[2.0, 0.0, 0.0, 0.0]))
        clk.t += 0.1
        node._tick()
        if node.done:
            break
    modes = [m.data for m in node.pubs[N.TOPIC_MODE_REQUEST].sent]
    n = SN.DECISION_SPELLS_MAX
    assert node.done and modes == ["LOITER", "GUIDED"] * (n - 1) + ["LOITER", "LAND"], modes
    esc = [e for e in RN._events(out) if e["kind"] == "decision_hold_escalated"]
    assert len(esc) == 1 and f"{n} holds" in esc[0]["why"], esc
    assert SN.DECISION_SPELLS_WINDOW_S >= 10.0


def test_n4_the_escalation_defers_to_the_fsm_and_records_where():
    """The escalation's LAND ignored the FSM (it landed over an FSM Loiter,
    ended the episode in an FSM RTL) and recorded no position. It now acts
    only on a hold this node placed (the FSM's own Loiter or fail-safe owns
    the aircraft otherwise), and records where, the FSM state, the mode and
    the keep-out rules the position breaks."""
    from guardrail.fsm import FSMState
    RN, (N, node, clk, out) = _shield(["--decision-timeout-s", "1.0",
                                       "--decision-hold-max-s", "1.0"])
    node.start_mission()
    node.fsm._state = FSMState.LOITER                   # the FSM has the aircraft
    stale = [1.0, 0, 0, 0, 3.0, 2.0, STEP, 0.0]
    _run(N, node, clk, [stale] * 15)
    assert not node.done and "decision" not in node._holds
    assert not any(e["kind"] == "decision_hold_escalated" for e in RN._events(out))
    RN2, (N2, node2, clk2, out2) = _shield(["--decision-timeout-s", "1.0",
                                            "--decision-hold-max-s", "1.0"])
    node2.start_mission()
    node2.state = State(x=15.0, y=15.0, up=15.0, yaw_deg=0.0)  # inside nfz-square
    _run(N2, node2, clk2, [stale] * 20)
    assert node2.done
    esc = node2.decision_escalation
    assert esc["position"] == {"x": 15.0, "y": 15.0, "up": 15.0}, esc
    assert esc["position_unsafe_rules"] == ["nfz-square"] and esc["fsm_state"], esc
    metrics = json.loads((out2 / "metrics.json").read_text(encoding="utf-8"))
    assert metrics["vla_pilot"]["decision_escalation"]["position"]["x"] == 15.0


def test_n5_a_second_interrupt_during_close_cannot_cut_the_records_short():
    """main() ignored only SIGTERM while close() wrote the records; a second
    Ctrl+C there raised KeyboardInterrupt, which close()'s guard does not
    catch. Both signals are ignored now while the records are written."""
    import signal
    fakes, made = _fake_ros()
    fakes["rclpy"].init = lambda args=None: None
    fakes["rclpy"].ok = lambda: False

    def spin(node):
        raise KeyboardInterrupt
    fakes["rclpy"].spin = spin
    name = "ros2_vla_bridge_node_close"
    saved = {k: sys.modules.get(k) for k in [*fakes, name]}
    sys.modules.update(fakes)
    old = {s: signal.getsignal(s) for s in (signal.SIGTERM, signal.SIGINT)}
    seen = {}
    try:
        spec = importlib.util.spec_from_file_location(name, ROOT / "sitl" / "ros2_vla_bridge_node.py")
        M = importlib.util.module_from_spec(spec)
        sys.modules[name] = M
        spec.loader.exec_module(M)
        real_close = M.VLABridge.close

        def close(self):
            seen["int"] = signal.getsignal(signal.SIGINT)
            seen["term"] = signal.getsignal(signal.SIGTERM)
            return real_close(self)
        M.VLABridge.close = close
        import socket
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()
        M.main(["--vla-server", f"127.0.0.1:{port}", "--policy", str(POLICY),
                "--out", str(_tmp() / "close"), "--vla-hello-timeout-s", "0.2"])
    finally:
        for sig, h in old.items():
            signal.signal(sig, h)
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    assert seen == {"int": signal.SIG_IGN, "term": signal.SIG_IGN}, seen


def test_n6_an_unlabelled_eight_value_message_is_refused_once_the_layout_is_announced():
    RN, (N, node, clk, out) = _shield([])
    node.start_mission()
    ok = [1.0, 0, 0, 0, 0.2, 1.0, STEP, 0.0]
    _run(N, node, clk, [ok])                           # no identity yet: accepted
    assert node.n_pilot_msgs == 1 and node.n_pilot_malformed == 0
    node._on_identity(N.String(data=json.dumps(
        {"model_id": VM.MODEL_ID, "kind": "vla", "action_layout": list(B.ACTION_FIELDS)})))
    _run(N, node, clk, [ok])
    assert node.n_pilot_malformed == 1 and "label" in node.pilot_error, node.pilot_error
    _run(N, node, clk, [ok], label=B.ACTION_LABEL)
    assert node.n_pilot_msgs == 2


def test_n8_a_non_finite_action_value_is_refused_and_recorded():
    """A message with a NaN or an infinity among its four action values was
    dropped without a word; it is a refusal like any other now: counted, an
    event, held."""
    RN, (N, node, clk, out) = _shield([])
    node.start_mission()
    modes = _run(N, node, clk, [[float("nan"), 0.0, 0.0, 0.0],
                                [1.0, 0, float("inf"), 0, 0.2, 1.0, STEP, 0.0]])
    assert node.n_pilot_malformed == 2 and node.n_actions == 0
    assert modes == ["LOITER"], modes
    ev = [e for e in RN._events(out) if e["kind"] == "vla_action_refused"]
    assert len(ev) == 2 and "non-finite" in ev[0]["why"], ev
    assert ev[0]["values"][0] is None, ev[0]


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
        except (Exception, SystemExit) as e:          # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
