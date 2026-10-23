"""demo/vla_fly.py and demo/vla_mission.py: AerialVLA flying the CityLife
missions, checked without the simulator, the GPU or the model.

Run either way:
    pytest tests/test_vla_fly.py -v
    python tests/test_vla_fly.py

WHAT IS BEING CLAIMED

1. The GOTO hint is the goal's bearing from the CAPTURE pose relative to the
   heading, in the guardrail frame (x North, y East, yaw clockwise from
   North, what Project AirSim's NED kinematics give), one of the eight
   training phrases, "" inside the arrive radius, hint_source "goal"; the
   APPROACH hint is the bridge's phrase of a FRESH box and no request
   otherwise (never ""). The per-tick State carries the heading, without
   which RemoteVLABackend reads every pose as facing North.
2. The mission's state machine ends on LAND at once, on a dead or erroring
   server only after it has hovered dead_timeout_s, on malformed answers held
   that long, on no parsed answer at all, on arriving without LAND, on the
   box filling the frame, on a lost target, and at the time limit - and a
   break in any streak resets it.
3. The records: a closed loop through TCP (point vehicle, a scripted server
   answering like a perfect AerialVLA, the real Shield over the CityLife
   policy) writes rows whose author is vla_step / hold / hover / none and
   carries the decision's phrase, hint source and step and the Shield's
   decision; metrics.pilot is pilot_record kind "vla" with the server's
   reported digest; and tools/todo.py classify_run on the written run folder
   says "vla" - and does NOT when the server only refused, or when the
   answers came from a stand-in with no weights.
4. The configs parse: the scenes name the robot configs, the VLA cameras are
   AerialVLA's training cameras (256x256, 90 deg, pitch 0 / -90), only Chase
   streams, the airframe is the follow rail's, the policy is flyable and
   locked.
5. The server lifecycle: every start names --precision, --attn and
   --merge/--no-merge (bf16/sdpa/--no-merge unless asked), a started server
   says hello and stops on shutdown, an occupied port and a server that dies
   are refused before take-off; the launcher, run with its real parameter
   binding, assembles the same arguments.
6. The review of the CityLife flight (2026-10-23): with the Shield off the
   pilot is clamped only by the simulator safety band and the policy HUD
   reads no repair (P1-1, P1-2); stop-and-go flies every step whole and asks
   only after the 0.5 s settle (P2-3); a slow model is not a dead one, its
   first answer gets its own grace, a timed-out request is waited for and
   never stacked behind, and the warm-up gate refuses a slow or silent model
   (P2-5); the detector phrase is corrected for the turn since the box
   (P2-6); only sent captures are kept (P3-8); a reused tag loses its old
   records but never frames or a link (P3-9); a fake: id is a stand-in
   (P3-13); the GOTO's object words are a fixed description (P3-14);
   flying past the goal ends the mission (P3-15).
7. The review of the ArduPilot rail's VLA pilot (4a0da6d, 2026-10-23), on
   this rail's warm-up: a slow cold first answer gets the timeout recorded
   for it (P1-1), and a refused first request is followed by the next once
   the client's backoff is over (P2-8) - vla_mission.wait_client_ready and
   a per-request response timeout, shared with that rail. Both failed on
   4a0da6d: refused, "not sent".

Shown failing first: before this change demo/vla_mission.py, demo/vla_fly.py,
the VLA robot and scene configs, policies/vla_goto_citylife.yaml and
scripts/run_vla_fly.ps1 did not exist, and StepTracker had no
current_record, so every test below failed at import or on its first
assertion. The claims of 6 were run against 46ca291 (the tree before the
review's fixes) with this file: 17 of its 33 tests failed there, every one
of 6's among them.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "tests", ROOT / "demo", ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail.models import Action4D, State, load_policy            # noqa: E402
from guardrail.vla_link import (AUTHORS, PHRASES, Capture, Hint, Pose,  # noqa: E402
                                RemoteVLA, Tick, TrackerConfig)
import vla_fake_server as F                                          # noqa: E402
import vla_mission as VM                                             # noqa: E402
import vla_fly                                                       # noqa: E402

SKIP = "SKIP"
JPG = b"\xff\xd8\xff\xe0fake-jpeg\xff\xd9"
CFG = ROOT / "demo" / "pas_config"
POLICY = ROOT / "policies" / "vla_goto_citylife.yaml"
# A model identity that looks like the real server's (vla_server_v2.
# model_identity): the dashboard's digest rule refuses the fake server's
# "0" * 64, so a run classified "vla" needs a digest that is not a stand-in.
REAL_MODEL = {"id": "aerialvla-lora/aero_vla@196f2f32", "base": "openvla-7b@47a0ec7f",
              "weights_sha256": hashlib.sha256(b"base shards + adapter").hexdigest(),
              "precision": "bf16", "attn": "sdpa", "merged": False,
              "agreement": {"measured": True, "probe_exact": 0.96}}


def _tick(author="vla_step", reason="translate", seq=1, age=0.2, land=False):
    return Tick(Action4D(), author, reason, seq, age, land)


def _with_model(respond, model):
    def r(req):
        out = respond(req)
        if isinstance(out, dict) and "model" in out:
            out = dict(out, model=dict(model))
        return out
    return r


class _Sim:
    """A point multirotor on a simulated clock (velocity lag 0.3 s), as in
    tests/test_vla_backends.py; it reports its heading in State.yaw_deg."""

    def __init__(self, x=42.0, y=-18.0, up=10.0, yaw_deg=90.0):
        self.t = 0.0
        self.x, self.y, self.up, self.yaw = x, y, up, math.radians(yaw_deg)
        self.v = [0.0, 0.0, 0.0]

    def clock(self) -> float:
        return self.t

    def state(self) -> State:
        return State(x=self.x, y=self.y, up=self.up, yaw_deg=math.degrees(self.yaw))

    def capture(self) -> Capture:
        return Capture(self.t, Pose(self.x, self.y, self.up, self.yaw), JPG, JPG)

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
    return Path(tempfile.mkdtemp(prefix="vlafly_"))


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


# =========================================================================== #
# 1. Hints
# =========================================================================== #
def test_the_goto_phrase_is_the_goal_bearing_from_the_capture_pose():
    """From the mission start (42, -18) the default goal (43, 40) lies 89 deg
    east of north: straight ahead facing east, to the right facing north, to
    the left facing south, behind facing west (the -180 seam goes to the
    left rear, semantic_direction's wrap); "" within the arrive radius, and
    the source is always "goal"."""
    src = VM.goto_hint_source(VM.DEFAULT_GOAL_XY, 3.0)
    sx, sy = VM.DEFAULT_START_XY
    cases = {90.0: "straight ahead ", 0.0: "to your right ", 180.0: "to your left ",
             60.0: "forward-right ", 120.0: "forward-left ", -60.0: "to your right rear ",
             -90.0: "to your right rear "}
    for yaw_deg, want in cases.items():
        h = src(Capture(0.0, Pose(sx, sy, 10.0, math.radians(yaw_deg)), JPG, JPG))
        assert h == Hint(want, "goal"), (yaw_deg, h)
    assert src.last == {"t_capture": 0.0, "phrase": "to your right rear ", "source": "goal"}
    gx, gy = VM.DEFAULT_GOAL_XY
    assert src(Capture(1.0, Pose(gx - 2.0, gy + 1.0, 10.0, 0.0), JPG, JPG)) == Hint("", "goal")
    assert src(Capture(1.0, Pose(gx - 3.5, gy, 10.0, 0.0), JPG, JPG)).phrase == "straight ahead "
    assert set(src.counts) <= PHRASES and src.stats()["kind"] == "goal"
    b = math.degrees(math.atan2(gy - sy, gx - sx))
    assert abs(b - 89.0) < 0.1, b
    args = vla_fly.parse_cli(vla_fly.build_arg_parser(), ["--mission", "goto"])
    assert abs(math.degrees(vla_fly.start_heading_rad(args, VM.DEFAULT_START_XY)) - b) < 1e-9


def test_the_per_tick_state_carries_the_heading_and_the_measured_velocity():
    """RemoteVLABackend reads the pose from State.yaw_deg (Pose.from_state);
    follow_vlm's rows built State without it, which here would read every
    pose as facing North and point every phrase the wrong way. A quaternion
    of +90 deg about the down axis is East; NED z velocity -1 is a climb."""
    from semantic_demo import quat_yaw
    c, s = math.cos(math.radians(45)), math.sin(math.radians(45))
    kin = {"pose": {"position": {"x": 42.0, "y": -18.0, "z": -10.0},
                    "orientation": {"w": c, "x": 0.0, "y": 0.0, "z": s}},
           "twist": {"linear": {"x": 0.5, "y": 1.0, "z": -1.0}}}
    st, yaw = vla_fly._state_from_kin(kin, State, quat_yaw)
    assert abs(yaw - math.pi / 2) < 1e-9 and abs(st.yaw_deg - 90.0) < 1e-9
    assert (st.x, st.y, st.up) == (42.0, -18.0, 10.0)
    assert st.velocity == (0.5, 1.0, 1.0), st.velocity
    assert abs(Pose.from_state(st).yaw_rad - math.pi / 2) < 1e-9
    kin.pop("twist")
    assert vla_fly._state_from_kin(kin, State, quat_yaw)[0].velocity is None


def test_the_approach_hint_is_the_bridge_phrase_of_a_fresh_box_and_never_empty():
    """The box's bearing on the detector camera (VLABridge, pinhole at 90
    deg) becomes the phrase; a box older than fresh_s at the VLA capture, or
    none, is NO request (None), never "" (which would ask for LAND where the
    box was lost)."""
    from vla_bridge import VLABridge
    box = {"det": None, "t": None}
    hint = VM.DetectorHint(lambda: (box["det"], box["t"]), VLABridge("an orange car"),
                           fresh_s=1.0)
    src = VM.HintSource(hint, "detector")
    cap = Capture(10.0, Pose(0.0, 0.0, 10.0, 0.0), JPG, JPG)
    assert src(cap) is None and hint.n_none == 1
    box.update(det=(384.0, 200.0, 40.0, 20.0, 0.2, 768, 432, 0.5), t=9.6)
    assert src(cap) == Hint("straight ahead ", "detector")
    assert abs(hint.last_box_age_s - 0.4) < 1e-9
    box.update(det=(700.0, 200.0, 40.0, 20.0, 0.2, 768, 432, 0.5), t=9.9)
    assert src(cap) == Hint("forward-right ", "detector")
    box.update(det=(30.0, 200.0, 40.0, 20.0, 0.2, 768, 432, 0.5), t=9.9)
    assert src(cap) == Hint("forward-left ", "detector")
    box["t"] = 8.5                                    # 1.5 s old: not fresh
    assert src(cap) is None and hint.n_stale == 1
    assert "" not in src.counts and src.n_none == 2
    assert VM.box_fraction((0, 0, 192.0, 0, 0, 768, 432)) == 0.25
    assert VM.box_fraction(None) is None


# =========================================================================== #
# 2. The mission's state machine
# =========================================================================== #
def _goto_monitor(**kw):
    cfg = VM.MonitorConfig(mission="goto", max_s=kw.pop("max_s", 100.0), **kw)
    return VM.MissionMonitor(cfg, t0=0.0)


FAR = Pose(0.0, 0.0, 10.0, 0.0)


def test_land_ends_the_mission_at_once_whatever_else_holds():
    m = _goto_monitor()
    assert m.update(0.1, FAR, _tick(), goal_xy=(50.0, 0.0)) is None
    assert m.update(0.2, FAR, _tick("hold", "land", land=True), goal_xy=(50.0, 0.0)) == "vla_land"
    assert m.update(0.3, FAR, _tick(), goal_xy=(50.0, 0.0)) == "vla_land"   # latched
    s = m.summary()
    assert s["end_reason"] == "vla_land" and not s["vla_failure"] and s["t_end_s"] == 0.2


def test_a_dead_server_hovers_first_and_ends_only_after_the_timeout():
    """The brief: a dead or erroring server -> hover, then land after a
    timeout. Hover on link_lost for 9.9 s is not yet an end; a parsed
    decision in between resets the streak; 10 s without one ends it
    server_dead, and the process's exit is recorded, not an end by itself."""
    m = _goto_monitor(dead_timeout_s=10.0)
    g = (50.0, 0.0)
    m.update(0.0, FAR, _tick(), goal_xy=g)
    t = 0.1
    while t < 5.0:
        assert m.update(t, FAR, _tick("hover", "link_lost", None, None), goal_xy=g,
                        server_alive=False) is None
        t = round(t + 0.1, 6)
    assert m.update(t, FAR, _tick("vla_step", "translate", 7), goal_xy=g) is None
    t0 = t = round(t + 0.1, 6)
    while t - t0 < 9.9:
        assert m.update(t, FAR, _tick("hover", "link_lost", None, None), goal_xy=g) is None
        t = round(t + 0.1, 6)
    end = None
    while end is None and t < 30:
        end = m.update(t, FAR, _tick("hover", "link_lost", None, None), goal_xy=g)
        t = round(t + 0.1, 6)
    assert end == "server_dead" and m.summary()["vla_failure"], end
    assert m.summary()["t_server_exit_s"] == 0.1
    assert any(e["event"] == "server_exited" for e in m.events)


def test_server_errors_and_malformed_answers_end_the_mission_as_their_own_reasons():
    for author, reason, want in (("none", "server_error", "server_error"),
                                 ("hover", "server_error", "server_error"),
                                 ("hold", "parse_fail", "parse_fail_streak")):
        m = _goto_monitor(dead_timeout_s=3.0)
        m.update(0.0, FAR, _tick(), goal_xy=(50.0, 0.0))
        end, t = None, 0.1
        while end is None and t < 10:
            end = m.update(t, FAR, _tick(author, reason, None if author != "hold" else 2),
                           goal_xy=(50.0, 0.0))
            t = round(t + 0.1, 6)
        assert end == want and abs(t - 3.2) < 0.15, (author, reason, end, t)


def test_no_parsed_answer_at_all_ends_the_mission():
    """A server that only ever answered malformed text, or nothing, never
    flew: no_first_decision at first_decision_timeout_s (a held parse failure
    is not a decision)."""
    m = _goto_monitor(first_decision_timeout_s=5.0, dead_timeout_s=50.0)
    end, t = None, 0.0
    while end is None and t < 10:
        tk = _tick("hold", "parse_fail", 3) if t > 2 else _tick("none", "no_decision", None, None)
        end = m.update(t, FAR, tk, goal_xy=(50.0, 0.0))
        t = round(t + 0.1, 6)
    assert end == "no_first_decision" and abs(t - 5.1) < 0.15, (end, t)


def test_arriving_without_land_ends_after_the_grace_and_leaving_resets_it():
    m = _goto_monitor(arrive_radius_m=3.0, arrive_grace_s=8.0)
    g = (10.0, 0.0)
    inside, outside = Pose(8.0, 0.0, 10.0, 0.0), Pose(6.0, 0.0, 10.0, 0.0)
    t = 0.0
    for _ in range(50):                          # 5 s inside
        assert m.update(t, inside, _tick("hold", "step_complete"), goal_xy=g) is None
        t = round(t + 0.1, 6)
    assert m.update(t, outside, _tick(), goal_xy=g) is None
    t = round(t + 0.1, 6)
    t_in = t
    end = None
    while end is None and t < 60:
        end = m.update(t, inside, _tick("hold", "step_complete"), goal_xy=g)
        t = round(t + 0.1, 6)
    assert end == "arrived_no_land" and abs((t - 0.1) - t_in - 8.0) < 0.15, (end, t, t_in)
    ev = [e["event"] for e in m.events]
    assert ev.count("inside_arrive_radius") == 2 and "left_arrive_radius" in ev


def test_the_approach_ends_on_a_large_box_or_a_lost_target_and_the_mission_time():
    cfg = VM.MonitorConfig(mission="approach", max_s=100.0, close_box_frac=0.15,
                           lost_timeout_s=4.0, det_fresh_s=1.0)
    small = (384.0, 200.0, 50.0, 30.0, 0.2, 768, 432, 0.5)
    big = (384.0, 200.0, 130.0, 60.0, 0.2, 768, 432, 0.5)
    m = VM.MissionMonitor(cfg, 0.0)
    assert m.update(1.0, FAR, _tick(), det=small, det_age_s=0.3) is None
    assert m.update(1.1, FAR, _tick(), det=big, det_age_s=1.5) is None   # stale: not seen
    assert m.update(1.2, FAR, _tick(), det=big, det_age_s=0.3) == "close_box"
    m = VM.MissionMonitor(cfg, 0.0)
    assert m.update(1.0, FAR, _tick(), det=small, det_age_s=0.2) is None
    assert m.update(4.9, FAR, _tick(), det=small, det_age_s=3.0) is None
    assert m.update(5.0, FAR, _tick(), det=None) == "target_lost"
    m = VM.MissionMonitor(cfg, 0.0)
    assert m.update(3.9, FAR, _tick(), det=None) is None
    assert m.update(4.0, FAR, _tick(), det=None) == "target_lost"       # never seen
    m = VM.MissionMonitor(VM.MonitorConfig(mission="goto", max_s=2.0), 0.0)
    assert m.update(1.9, FAR, _tick(), goal_xy=(99.0, 0.0)) is None
    assert m.update(2.0, FAR, _tick(), goal_xy=(99.0, 0.0)) == "timeout"
    for bad in ({"mission": "circle"}, {"max_s": 0}, {"close_box_frac": 1.2},
                {"dead_timeout_s": float("nan")}):
        try:
            VM.MonitorConfig(**bad)
        except ValueError:
            continue
        raise AssertionError(f"MonitorConfig accepted {bad}")
    assert set(VM.VLA_FAILURE_ENDS) < set(VM.END_REASONS)


# =========================================================================== #
# 3. The loop and the records, through TCP, classified by the dashboard rule
# =========================================================================== #
def _fly(respond, model, out: Path, *, goal=(43.0, 10.0), shield_on=True, max_s=120.0,
         dead_timeout_s=5.0, cadence="pipelined", on_tick=None):
    """The flight loop of demo/vla_fly.py on a point vehicle, built the way
    the flight builds it: the pilot's altitude clamp from
    vla_mission.pilot_alt_config (the policy's band with the Shield on, the
    simulator safety band with it off), the GOTO's default object words,
    the cadence asked for. `on_tick(res)` sees every LoopTick."""
    from guardrail.replay import rail_shield
    from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S
    from guardrail.vla_backends import make_backend
    policy = load_policy(POLICY)
    sim = _Sim()
    srv = F.FakeVLAServer(respond, model=model)
    srv.start()
    addr = f"127.0.0.1:{srv.port}"
    hint = VM.goto_hint_source(goal, 3.0)
    client = RemoteVLA("127.0.0.1", srv.port, response_timeout_s=2.0, reconnect_backoff_s=0.5,
                       strict_identity=VM.identity_is_vla(model)[0], clock=sim.clock)
    bpol, band, band_rec = VM.pilot_alt_config(shield_on, policy)
    be = make_backend("aerovla", None, bpol, get_capture=sim.capture, get_hint=hint,
                      client=client, object_text=VM.GOTO_OBJECT_TEXT,
                      steps_log=out / "vla_steps.jsonl", cadence=cadence,
                      tracker_config=TrackerConfig(alt_band=band), clock=sim.clock)
    shield = rail_shield(policy, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S)
    mon = VM.MissionMonitor(VM.MonitorConfig(mission="goto", max_s=max_s,
                                             dead_timeout_s=dead_timeout_s), sim.clock())
    loop = VM.MissionLoop(be, shield, mon, mission="goto", shield_on=shield_on, goal_xy=goal,
                          hint_source=hint, clock=sim.clock)
    rows, end = [], None
    try:
        while end is None:
            res = loop.tick(sim.state())
            rows.append(res.row)
            if on_tick is not None:
                on_tick(res)
            end = res.end_reason
            be.client.wait_idle(3.0)
            sim.step(res.flown)
        be.close()
    finally:
        srv.stop()
    stats, records = be.stats(), list(be.records)
    server = {"mode": "attached", "addr": addr}
    metrics = {"tag": out.name, "ticks": len(rows), "end_reason": end, "aborted": None,
               "shield": "on" if shield_on else "off", "loop_hz": VM.loop_rate(rows),
               "mission_s": rows[-1]["t"],
               **VM.mission_scores("goto", rows, goal_xy=goal, arrive_radius_m=3.0),
               "land": VM.land_record(rows, records, goal)}
    metrics["pilot"] = VM.pilot_entry(model, stats, records, rows, server=server,
                                      reported_by=server["addr"])
    manifest = VM.manifest_for(policy.policy_hash, model, 7, CFG / "scene_vla.jsonc")
    VM.write_jsonl(out / "flight_log.jsonl", rows)
    kpis = VM.finish_run(out, rows, metrics, manifest, policy)
    return SimpleNamespace(sim=sim, srv=srv, rows=rows, end=end, metrics=metrics,
                           manifest=manifest, kpis=kpis, records=records, stats=stats,
                           backend=be, loop=loop, band=band_rec)


def test_a_goto_flown_by_the_scripted_model_is_classified_vla_from_its_own_records():
    """The vehicle moves only by what the loop flies; the server answers each
    image like a perfect AerialVLA from the request's pose and phrase alone,
    and LAND on the "" the hint sends inside the arrive radius. Every tick
    names its author; the step ticks carry the decision's phrase, hint
    source and step; metrics.pilot is kind vla with the reported digest;
    the dashboard rule (tools/todo.py vla_evidence + classify_run) reads the
    written folder as a VLA flight."""
    import todo as TODO
    out = _tmp() / "vla_goto_t"
    out.mkdir()
    r = _fly(_with_model(F.goto_responder((43.0, 10.0), 10.0), REAL_MODEL), REAL_MODEL, out)
    assert r.end == "vla_land", (r.end, r.sim.x, r.sim.y)
    assert math.hypot(r.sim.x - 43.0, r.sim.y - 10.0) < 3.0
    assert r.srv.refused == [] and r.srv.overlaps == 0
    for q in r.srv.requests:
        assert q["hint_source"] == "goal" and q["phrase"] in PHRASES
        assert not ({"target", "goal", "xy"} & set(q))
    assert r.srv.requests[-1]["phrase"] == ""
    authors = [x["author"] for x in r.rows]
    assert set(authors) <= set(AUTHORS) and authors.count("vla_step") > len(authors) / 2
    steps = [x for x in r.rows if x["author"] == "vla_step"]
    assert all(x["hint_source"] == "goal" and x["phrase"] in PHRASES and x["step"]
               and x["vla_seq"] is not None and x["vla_age_s"] is not None for x in steps)
    for x in r.rows:
        for k in ("raw", "emitted", "flown_action", "touched", "violations", "repairs",
                  "emitted_violations", "flown", "shield", "goal_dist_m"):
            assert k in x, k
        assert x["flown_action"] == x["emitted"] and x["shield"] == "on"
    p = r.metrics["pilot"]
    assert p["kind"] == "vla" and p["model_id"] == VM.MODEL_ID
    assert p["reported_digest"] == REAL_MODEL["weights_sha256"] and p["hash_source"] == "reported"
    assert p["hash"] == f"{VM.MODEL_ID}@sha256:{REAL_MODEL['weights_sha256']}"
    s = p["summary"]
    assert s["decisions_accepted"] >= 10 and s["decision_hz"] > 0
    assert s["latency_client_s"]["n"] > 0 and s["latency_client_s"]["p95"] is not None
    assert s["land"] == 1 and s["parse_fail"] == 0 and s["server_error"] == 0
    assert s["vla_authored_tick_share"] > 0.5
    assert r.metrics["land"]["requested_by_vla"] and not r.metrics["land"]["with_hint"]
    assert r.metrics["reached_goal"] is True
    assert r.manifest["vla_model_hash"] == p["hash"]
    assert (out / "kpi.json").is_file() and r.kpis["p0_violation_escape_rate"] == 0.0
    ev = TODO.vla_evidence(out)
    cls, why = TODO.classify_run(r.manifest["vla_model_hash"], ev)
    assert (cls, why) == ("vla", None), (cls, why, ev)
    assert ev["source"] == "vla_steps.jsonl" and ev["weights_sha256"] == REAL_MODEL["weights_sha256"]
    import build_dashboard as BD
    fr = BD.flight_record(ROOT, out)
    assert fr["pilot"]["class"] == "vla" and fr["pilot"]["metrics_kind"] == "vla", fr["pilot"]
    assert fr["end_reason"] == "vla_land" and fr["shield"] == "on" and fr["loop_hz"] == 10.0
    assert fr["pilot"]["evidence"]["decisions"] == s["decisions_accepted"]
    raw = (out / "flight_log.jsonl").read_bytes()
    assert b"\r\n" not in raw and len(raw.splitlines()) == len(r.rows)


def test_a_server_that_only_refuses_hovers_lands_and_is_never_classified_vla():
    """Every request refused (ok=false): no tick is vla_step, the mission
    ends server_error after the timeout, and the dashboard rule says
    vla_unverified (no accepted decisions) for the same manifest."""
    import todo as TODO
    out = _tmp() / "vla_err_t"
    out.mkdir()
    r = _fly(lambda req: F.error_reply(req, "inference failed: CUDA out of memory"),
             REAL_MODEL, out, dead_timeout_s=3.0)
    assert r.end == "server_error", r.end
    assert "vla_step" not in {x["author"] for x in r.rows}
    assert r.metrics["pilot"]["summary"]["decisions_accepted"] == 0
    cls, why = TODO.classify_run(r.manifest["vla_model_hash"], TODO.vla_evidence(out))
    assert cls == "vla_unverified" and why.startswith("no_accepted_decisions"), (cls, why)


def test_a_stand_in_without_weights_is_recorded_as_a_stub_never_a_vla():
    """The fake server's "0" * 64 digest is a stand-in (tools/todo.py
    weights_digest_ok): the pilot record is kind stub, the manifest names the
    stand-in, and classify_run never says vla, although it flew."""
    import todo as TODO
    out = _tmp() / "vla_stub_t"
    out.mkdir()
    r = _fly(F.goto_responder((43.0, 10.0), 10.0), dict(F.MODEL), out)
    assert r.end == "vla_land"
    assert r.metrics["pilot"]["kind"] == "stub" and "stand-in" in r.metrics["pilot"]["not_a_vla_because"]
    assert r.manifest["vla_model_hash"].startswith("fake:")
    cls, _ = TODO.classify_run(r.manifest["vla_model_hash"], TODO.vla_evidence(out))
    assert cls != "vla", cls


def test_shield_off_flies_the_raw_action_and_says_so():
    out = _tmp() / "vla_off_t"
    out.mkdir()
    r = _fly(_with_model(F.goto_responder((43.0, 10.0), 10.0), REAL_MODEL), REAL_MODEL, out,
             shield_on=False)
    assert all(x["shield"] == "off" and x["flown_action"] == x["raw"] for x in r.rows)
    assert r.metrics["shield"] == "off"
    k = json.loads((out / "kpi.json").read_text(encoding="utf-8"))
    assert "p0_violation_escape_rate" in k


def _climber(req):
    """A model that climbs 3 m on every step (2 m forward, no turn): with the
    Shield off it should leave the policy's 6-14 m band."""
    return F.step_reply(req, 2.0, -3.0, 0.0)


def test_shield_off_is_clamped_only_by_the_sim_band_and_its_hud_shows_no_repair():
    """Review of the CityLife flight, P1-1 and P1-2, through the loop. A
    model that climbs 3 m a step: with the Shield ON the tracker clamps into
    the policy's band and never passes 14 m. With it OFF, (P1-2) the backend
    is built without the policy and clamps only into SIM_SAFETY_BAND_M, so
    the aircraft climbs past 14 m (and stays below 40) - before the fix the
    off arm was still held inside the policy band by the pilot side; and
    (P1-1) the policy indicator fed `hud_decision` counts no repair and no
    brake but the P0 escapes, while fed the shadow decision - what 46ca291
    passed it - it counted Shield repairs that never flew."""
    from policy_hud import PolicyIndicator
    out = _tmp() / "vla_band_on"
    out.mkdir()
    on = _fly(_with_model(_climber, REAL_MODEL), REAL_MODEL, out, max_s=25.0)
    assert max(x["up"] for x in on.rows) <= 14.0, max(x["up"] for x in on.rows)
    assert on.backend.alt_band_info["source"] == "policy" and on.band["clamp"] == "policy"
    out = _tmp() / "vla_band_off"
    out.mkdir()
    policy = load_policy(POLICY)
    fixed, old = PolicyIndicator(policy), PolicyIndicator(policy)
    seen = []

    def feed(res):
        st = State(x=res.row["x"], y=res.row["y"], up=res.row["up"], yaw_deg=res.row["psi"])
        fixed.update(res.row["t"], res.hud_decision, st)
        old.update(res.row["t"], res.decision, st)
        seen.append(res)
    off = _fly(_with_model(_climber, REAL_MODEL), REAL_MODEL, out, shield_on=False, max_s=25.0,
               on_tick=feed)
    ups = [x["up"] for x in off.rows]
    assert max(ups) > 15.0 and max(ups) <= VM.SIM_SAFETY_BAND_M[1], max(ups)
    info = off.backend.alt_band_info
    assert info["source"] == "configured" and tuple(info["band"]) == VM.SIM_SAFETY_BAND_M, info
    assert off.band["clamp"] == "sim_safety_band" and info["policy"] is None
    # The HUD's decision: no repair, the flown action emitted, shadow violations as escapes.
    hd = [r.hud_decision for r in seen]
    assert all(not d.repairs and not d.braked and not d.touched for d in hd)
    assert all(r.hud_decision.emitted == r.flown for r in seen)
    assert fixed.counts["repaired"] == 0 and fixed.counts["braked"] == 0
    assert fixed.counts["p0_escapes"] > 0, dict(fixed.counts)
    assert old.counts["repaired"] > 0, dict(old.counts)        # the defect, for contrast
    assert off.loop.n_touched == 0 and off.loop.n_shadow_touched > 0
    assert all(not r.hud["shield"] for r in seen)
    k = json.loads((out / "kpi.json").read_text(encoding="utf-8"))
    assert k["p0_escapes"] > 0 and k["p0_violation_escape_rate"] > 0, k["p0_escapes"]


def test_the_frame_store_pairs_front_and_down_and_never_sends_a_lone_front():
    """VLAFrames: no down frame, or one further than max_pair_dt_s by sim
    stamp, is no capture (the server refuses a missing down tile); a pair
    gives JPEG bytes, the pose interpolated at the front frame's stamp and a
    capture time never later than its arrival; once SENT (commit, the
    backend's on_submit) both tiles go to vla_frames/ by SHA-256, and the
    log keeps the camera sizes, the seq and the phrase. A capture that was
    never sent is neither saved nor logged (review of the CityLife flight,
    P3-8: until then every capture was, before the hint check)."""
    clock = {"t": 100.0}

    class Obs:
        def pose_at(self, t, stamp=None):
            return {"x": 1.0, "y": 2.0, "up": 10.0, "yaw": 0.5, "src": "ring-stamp"}

        def wall_at_stamp(self, stamp):
            return 99.95 + 1.0                  # a mapping error: "later than arrival"
    out = _tmp()
    saver = VM.FrameSaver(out / "vla_frames")
    fr = VM.VLAFrames(Obs(), encode=lambda m: m["jpg"], saver=saver, clock=lambda: clock["t"])
    assert fr.capture() is None and fr.counts["no_front"] == 1
    fr.put_front({"jpg": b"F1", "time_stamp": 1_000_000_000, "width": 256, "height": 256})
    assert fr.capture() is None and fr.counts["no_down"] == 1
    fr.put_down({"jpg": b"D1", "time_stamp": 1_400_000_000, "width": 256, "height": 256})
    assert fr.capture() is None and fr.counts["down_unpaired"] == 1
    fr.put_down({"jpg": b"D2", "time_stamp": 1_050_000_000, "width": 256, "height": 256})
    clock["t"] = 100.2
    cap = fr.capture()
    assert cap is not None and (cap.front_jpg, cap.down_jpg) == (b"F1", b"D2")
    assert cap.t_capture <= 100.0 and cap.pose == Pose(1.0, 2.0, 10.0, 0.5)
    assert len(fr.log) == 0 and fr.counts["captures"] == 1      # built, not sent: not kept
    fr.put_front({"jpg": b"F9", "time_stamp": 1_060_000_000, "width": 256, "height": 256})
    assert fr.capture() is not None                              # a newer one, never sent
    fr.put_front({"jpg": b"F1", "time_stamp": 1_000_000_000, "width": 256, "height": 256})
    cap = fr.capture()
    fr.commit(cap, Hint("straight ahead ", "goal"), 7)
    st = saver.close()
    assert st["written"] == 2 and st["errors"] == 0
    names = sorted(p.name for p in (out / "vla_frames").iterdir())
    assert names == sorted(hashlib.sha256(b).hexdigest() + ".jpg" for b in (b"F1", b"D2"))
    assert fr.stats()["front_sizes"] == {"256x256": 1}
    assert len(fr.log) == 1 and fr.counts["committed"] == 1 and fr.counts["captures"] == 3
    assert fr.log[-1]["front_jpg_sha256"] == hashlib.sha256(b"F1").hexdigest()
    assert fr.log[-1]["seq"] == 7 and fr.log[-1]["phrase"] == "straight ahead "
    # Through the backend: only a request that went out is committed.
    from guardrail.vla_backends import RemoteVLABackend
    sent = []

    class Client:
        busy = backing_off = False
        counts: dict = {}
        identity = None

        def __init__(self):
            self.n = 0

        def submit(self, fields):
            self.n += 1
            return self.n if self.n % 2 else None       # every other request refused

        def poll(self):
            return None

        def close(self):
            pass
    sim = _Sim()
    hints = iter([None, Hint("straight ahead ", "goal")] * 5)
    be = RemoteVLABackend(Client(), sim.capture, lambda c: next(hints),
                          on_submit=lambda c, h, s: sent.append((s, h.phrase)))
    for _ in range(10):
        be.act(sim.state())
    assert be.counts["no_hint"] == 5 and be.counts["requests"] == len(sent) >= 2
    assert all(p == "straight ahead " for _, p in sent)


def test_the_tracker_exposes_a_copy_of_the_record_it_flies():
    from guardrail.vla_link import Decision, Step, StepTracker
    tr = StepTracker()
    assert tr.current_record is None
    d = Decision.of(1, 0.0, Pose(0, 0, 10, 0), Step(4.0, 0.0, 0.0))
    d.record["phrase"] = "straight ahead "
    tr.accept(d, 0.1)
    rec = tr.current_record
    assert rec["phrase"] == "straight ahead " and rec["step"]["fwd_m"] == 4.0
    rec["phrase"] = "changed"
    assert tr.current_record["phrase"] == "straight ahead "


def test_stop_and_go_flies_every_step_whole_and_asks_only_after_the_settle():
    """Review of the CityLife flight, P2-3, through MissionLoop. Upstream's
    cadence (the flight's default now): the next image is captured only
    after the step being flown is complete and the tracker has held 0.5 s,
    so no step is ever superseded and each answer's turn and climb are
    applied once. The same goto pipelined supersedes steps (a capture while
    one is flown) - the integration the reviewer measured as +1.77 m against
    +0.51 m of climb over 30 s of a constant answer."""
    from guardrail.vla_link import UPSTREAM_SETTLE_S
    out = _tmp() / "vla_sng"
    out.mkdir()
    r = _fly(_with_model(F.goto_responder((43.0, 10.0), 10.0), REAL_MODEL), REAL_MODEL, out,
             cadence="stop_and_go")
    assert r.end == "vla_land", r.end
    assert r.stats["cadence"] == "stop_and_go" and r.rows[0]["shield"] == "on"
    assert all(x.get("hud") is None for x in r.rows)          # rows are rows, not HUD dicts
    steps = sorted((x for x in r.records if x.get("kind") == "step" and x.get("accepted")),
                   key=lambda x: x["seq"])
    assert len(steps) >= 5
    # Every step flown to completion before the next answer replaced it.
    assert all(x["outcome"]["completed"] for x in steps), [x["outcome"] for x in steps]
    for a, b in zip(steps, steps[1:]):
        done = a["outcome"]["t_complete"]
        assert done is not None and b["t_sent"] >= done + UPSTREAM_SETTLE_S - 1e-6, (a, b)
    assert math.hypot(r.sim.x - 43.0, r.sim.y - 10.0) < 3.0
    out = _tmp() / "vla_pipe"
    out.mkdir()
    p = _fly(_with_model(F.goto_responder((43.0, 10.0), 10.0), REAL_MODEL), REAL_MODEL, out,
             cadence="pipelined")
    cut = [x for x in p.records if (x.get("outcome") or {}).get("end") == "superseded"
           and not x["outcome"]["completed"]]
    assert cut, "pipelined: no step was cut short by a newer answer"
    assert p.stats["decisions_accepted"] > r.stats["decisions_accepted"]


def _lf(kind, seq):
    return {"kind": kind, "seq": seq, "t": float(seq), "detail": ""}


def test_a_slow_model_is_not_a_dead_one_and_its_first_answer_gets_its_own_grace():
    """Review of the CityLife flight, P2-5. Hovering on link_lost after a
    TIMEOUT is `server_slow`, after a refused or closed connection
    `server_dead`; before the first parsed answer a slow model is not ended
    by the dead rule (10 s) but by first_decision_timeout_s - before the fix
    a model slower than the response timeout ended the mission server_dead
    at ~20 s and the 60 s first-answer allowance never applied. Each failure
    kind is counted once per failure."""
    m = _goto_monitor(dead_timeout_s=10.0, first_decision_timeout_s=60.0, max_s=200.0)
    g = (50.0, 0.0)
    t, end, seq = 0.0, None, 0
    while end is None and t < 100:
        if round(t * 10) % 120 == 0:                 # a timeout every 12 s
            seq += 1
        end = m.update(t, FAR, _tick("none", "link_lost", None, None), goal_xy=g,
                       link_failure=_lf("timeout", seq))
        t = round(t + 0.1, 6)
    assert end == "no_first_decision" and abs(t - 60.1) < 0.15, (end, t)
    assert m.summary()["link_failure_kinds"] == {"timeout": seq}
    for kind, want in (("timeout", "server_slow"), ("link", "server_dead"),
                       ("protocol", "server_dead")):
        m = _goto_monitor(dead_timeout_s=10.0)
        m.update(0.0, FAR, _tick("vla_step", "translate", 1), goal_xy=g)
        t, end = 0.1, None
        while end is None and t < 50:
            end = m.update(t, FAR, _tick("hover", "link_lost", None, None), goal_xy=g,
                           link_failure=_lf(kind, 2))
            t = round(t + 0.1, 6)
        assert end == want and abs(t - 10.2) < 0.15, (kind, end, t)
    assert "server_slow" in VM.VLA_FAILURE_ENDS and "server_slow" in VM.END_REASONS
    # A refused connection before any answer is still dead after 10 s.
    m = _goto_monitor(dead_timeout_s=10.0)
    t, end = 0.0, None
    while end is None and t < 50:
        end = m.update(t, FAR, _tick("none", "link_lost", None, None), goal_xy=g,
                       link_failure=_lf("link", 1))
        t = round(t + 0.1, 6)
    assert end == "server_dead" and abs(t - 10.1) < 0.15, (end, t)


def _slow_server(delay_s, model=None):
    """A fake server whose model holds one lock across connections, as
    demo/vla_server_v2.py's does, and records how many requests were in it
    at once (2 = a request queued behind an abandoned one)."""
    import threading
    st = {"inflight": 0, "max": 0, "n": 0}
    lock = threading.Lock()

    def respond(req):
        with lock:
            st["inflight"] += 1
            st["n"] += 1
            st["max"] = max(st["max"], st["inflight"])
        time.sleep(delay_s)
        with lock:
            st["inflight"] -= 1
        return F.step_reply(req, 1.0, 0.0, 0.0)
    srv = F.FakeVLAServer(respond if model is None else _with_model(respond, model),
                          model=model)
    return srv, st


def test_a_timed_out_request_is_waited_for_never_stacked_behind():
    """Review of the CityLife flight, P2-5: the server cannot cancel a
    generate(), so a request sent after a timeout on a NEW connection waited
    behind the abandoned one (two in the model at once). With
    late_reply_wait_s the client posts the timeout at once, keeps the
    connection, refuses to send (`busy`, submit_draining) until the late
    answer arrives, and records it: one request in the model at a time, one
    connection. Without it (the old behaviour) the next request stacks."""
    from guardrail.vla_link import build_infer_fields
    cap = Capture(0.0, Pose(0.0, 0.0, 10.0, 0.0), JPG, JPG)
    fields = build_infer_fields(cap, Hint("straight ahead ", "goal"), "a street")[0]
    for wait, want_max, want_conn in ((5.0, 1, 1), (0.0, 2, 2)):
        srv, st = _slow_server(0.8)
        srv.start()
        c = RemoteVLA("127.0.0.1", srv.port, response_timeout_s=0.3, reconnect_backoff_s=0.0,
                      strict_identity=False, late_reply_wait_s=wait)
        try:
            assert c.submit(dict(fields)) == 1 and c.wait_idle(3.0)
            res = c.poll()
            assert res.failure == "timeout", res
            if wait:
                assert c.busy and c.draining and c.submit(dict(fields)) is None
                assert c.counts["submit_draining"] == 1
                assert c.wait_drained(5.0) and not c.busy
                assert c.counts["late_replies"] == 1 and c.late[-1]["seq"] == 1
                assert c.late[-1]["late_by_s"] > 0.3
            assert c.submit(dict(fields)) == 2
            c.wait_idle(3.0)
            c.poll()
            c.wait_drained(5.0)
            time.sleep(0.9 if not wait else 0.0)
            assert st["max"] == want_max and srv.connections == want_conn, (wait, st,
                                                                            srv.connections)
        finally:
            c.close()
            srv.stop()


def test_the_warm_up_gate_refuses_a_slow_or_silent_model_before_take_off():
    """Review of the CityLife flight, P2-5: one inference on a live pair
    before arming, timed; a round trip over the gate, a timeout or a server
    error refuses the flight, a fast answer passes, and the result says
    what it measured."""
    cap = Capture(0.0, Pose(42.0, -18.0, 0.2, 1.55), JPG, JPG)
    src = VM.goto_hint_source(VM.DEFAULT_GOAL_XY, 3.0)
    h = VM.warmup_hint("goto", src, cap)
    assert h == Hint("straight ahead ", "goal") and not src.counts      # not counted as flown
    assert VM.warmup_hint("approach", None, cap) == Hint("straight ahead ", "goal")
    srv, _ = _slow_server(0.2)
    with srv:
        c = RemoteVLA("127.0.0.1", srv.port, response_timeout_s=2.0, strict_identity=False)
        try:
            res = VM.warmup_infer(c, cap, h, VM.GOTO_OBJECT_TEXT, 2.0)
        finally:
            c.close()
    assert res["answered"] and res["kind"] == "step" and 0.15 < res["rtt_s"] < 2.0, res
    assert VM.warmup_verdict(res, 5.0) == (True, None)
    ok, why = VM.warmup_verdict(res, 0.1)
    assert not ok and "over the 0.1 s gate" in why
    srv, _ = _slow_server(1.0)
    with srv:
        c = RemoteVLA("127.0.0.1", srv.port, response_timeout_s=0.3, strict_identity=False)
        try:
            res = VM.warmup_infer(c, cap, h, VM.GOTO_OBJECT_TEXT, 0.3)
        finally:
            c.close()
    assert not res["answered"] and res["failure"] == "timeout"
    assert VM.warmup_verdict(res, 5.0)[0] is False
    with F.FakeVLAServer(lambda req: F.error_reply(req, "inference failed: OOM")) as srv:
        c = RemoteVLA("127.0.0.1", srv.port, response_timeout_s=2.0, strict_identity=False)
        try:
            res = VM.warmup_infer(c, cap, h, VM.GOTO_OBJECT_TEXT, 2.0)
        finally:
            c.close()
    ok, why = VM.warmup_verdict(res, 5.0)
    assert res["kind"] == "server_error" and not ok and "refused" in why


def test_the_detector_phrase_is_corrected_for_the_turn_since_the_box():
    """Review of the CityLife flight, P2-6. A box dead ahead on a detector
    frame taken at heading 0, and a VLA capture at heading +0.5 rad (28.6
    deg right, one second at the yaw cap): the target is now 28.6 deg to the
    LEFT, "forward-left ", not the "straight ahead " the uncorrected box
    gave. The age, the correction and the detector's latency are kept."""
    from vla_bridge import VLABridge
    box = {"got": ((384.0, 200.0, 40.0, 20.0, 0.2, 768, 432, 0.5), 9.5, 0.0, 9.7)}
    hint = VM.DetectorHint(lambda: box["got"], VLABridge("an orange car"), fresh_s=1.0)
    cap = Capture(10.0, Pose(0.0, 0.0, 10.0, 0.5), JPG, JPG)
    assert hint(cap) == Hint("forward-left ", "detector")
    assert abs(hint.last_yaw_correction_rad - 0.5) < 1e-9
    assert abs(hint.last_box_age_s - 0.5) < 1e-9 and abs(hint.last_detector_latency_s - 0.2) < 1e-9
    box["got"] = box["got"][:2]                            # no heading for the frame
    assert hint(cap) == Hint("straight ahead ", "detector") and hint.n_no_yaw == 1
    box["got"] = ((384.0, 200.0, 40.0, 20.0, 0.2, 768, 432, 0.5), 9.5, -3.0, None)
    # Across the +-pi seam: from -3.0 to +3.0 rad is a 16.2 deg LEFT turn,
    # not 344 deg right, so the box ahead is now 16.2 deg to the right.
    cap2 = Capture(10.0, Pose(0.0, 0.0, 10.0, 3.0), JPG, JPG)
    assert hint(cap2) == Hint("forward-right ", "detector")
    assert abs(math.degrees(hint.last_yaw_correction_rad) + 16.2) < 0.2
    s = hint.stats()
    assert s["fresh_s"] == 1.0 and s["hints"] == 3 and s["no_frame_heading"] == 1
    assert s["detector_latency_s"]["n"] == 1 and s["yaw_correction_abs_deg"]["n"] == 2


def test_flying_past_the_goal_ends_the_mission_as_an_overshoot():
    """Review of the CityLife flight, P3-15: after the aircraft has been
    inside the arrive radius, the goal behind it and receding for
    overshoot_s ends the mission `arrived_overshoot`, with the distance;
    the rear phrases are not flown until the time runs out. Hovering past
    the goal, or passing behind a goal never reached, does not."""
    g = (10.0, 0.0)
    m = _goto_monitor(arrive_radius_m=3.0, arrive_grace_s=8.0, overshoot_s=3.0)
    t, x, end = 0.0, 0.0, None
    while end is None and t < 60:                 # 1 m/s due north, through the goal
        end = m.update(t, Pose(x, 0.0, 10.0, 0.0), _tick(), goal_xy=g)
        t, x = round(t + 0.1, 6), x + 0.1
    s = m.summary()
    assert end == "arrived_overshoot" and s["ever_inside_arrive_radius"], (end, t)
    assert 5.5 < s["overshoot_m"] < 6.5 and s["min_goal_dist_m"] < 0.2, s
    m = _goto_monitor(arrive_radius_m=3.0, overshoot_s=3.0, max_s=30.0)
    for k in range(80):                           # in, out to 4 m past, then hover there
        x = min(14.0, 8.0 + 0.1 * k)
        end = m.update(k * 0.1, Pose(x, 0.0, 10.0, 0.0), _tick(), goal_xy=g)
    assert end is None and m.overshoot_m is None
    m = _goto_monitor(arrive_radius_m=3.0, overshoot_s=3.0)
    for k in range(100):                          # passes 5 m abeam, never inside
        end = m.update(k * 0.1, Pose(5.0 + 0.1 * k, 5.0, 10.0, 0.0), _tick(), goal_xy=g)
    assert end is None


def test_a_reused_tag_loses_its_old_records_but_never_frames_or_a_link():
    """Review of the CityLife flight, P3-9: the replay bundle, the replay
    gate's report and the server's logs (opened for append) of an earlier run
    under the same tag are removed with its other records; vla_frames/ (an
    earlier run's frames) and view/ (the recorder's, usually a junction to
    C:) are never touched, and a record that is a link is left alone."""
    out = _tmp() / "vla_tag"
    (out / "vla_frames").mkdir(parents=True)
    (out / "view" / "fpv").mkdir(parents=True)
    names = list(vla_fly.RUN_FILES) + ["vla_tag.replay.tar.gz"]
    for n in names:
        (out / n).write_text("old", encoding="utf-8")
    for n in ("vla_server.jsonl", "vla_server_stdout.log", "replay_gate.json",
              "vla_tag.replay.tar.gz"):
        assert n in names
    (out / "vla_frames" / ("a" * 64 + ".jpg")).write_bytes(b"x")
    (out / "view" / "fpv" / "000001.jpg").write_bytes(b"x")
    (out / "other.replay.tar.gz").write_bytes(b"x")       # not this tag's
    rep = vla_fly.clear_previous_run(out, "vla_tag")
    assert sorted(rep["removed"]) == sorted(names) and rep["left"] == []
    assert not any((out / n).exists() for n in names)
    assert (out / "vla_frames" / ("a" * 64 + ".jpg")).is_file()
    assert (out / "view" / "fpv" / "000001.jpg").is_file()
    assert (out / "other.replay.tar.gz").is_file()
    try:
        import _winapi
        target = _tmp() / "elsewhere"
        target.mkdir()
        (target / "keep.txt").write_text("keep", encoding="utf-8")
        _winapi.CreateJunction(str(target), str(out / "metrics.json"))
    except (ImportError, OSError, AttributeError):
        return None                                        # no junctions here: done
    rep = vla_fly.clear_previous_run(out, "vla_tag")
    assert (out / "metrics.json").exists() and (target / "keep.txt").is_file()
    assert any(x.startswith("metrics.json") for x in rep["left"])
    os.rmdir(out / "metrics.json")                         # removes the junction only


def test_a_fake_model_id_is_always_a_stand_in():
    """Review of the CityLife flight, P3-13: whatever else a `fake:` id
    contains - here AerialVLA's own adapter name - it is a stub, never vla or
    vla_unverified, in tools/todo.py and so in the dashboard
    (build_dashboard.flight_record calls the same classify_run)."""
    import todo as TODO
    for h in ("fake:aerialvla-lora/aero_vla@196f2f32", "fake:scripted@unresolved",
              "FAKE:openvla/openvla-7b@47a0ec7f", " fake:aerialvla:openvla-7b+aero_vla@sha256:ab"):
        assert TODO.classify_pilot(h) == "stub", h
        assert TODO.classify_run(h, {"source": "vla_steps.jsonl", "decisions": 99})[0] == "stub"
    assert TODO.classify_pilot("aerialvla:openvla-7b+aero_vla@sha256:ab") == "vla"
    ev = {"source": "vla_steps.jsonl", "decisions": 50, "digests": ["ab" * 32],
          "model_ids": ["fake:aerialvla-lora/aero_vla@196f2f32"]}
    assert TODO.evidence_problem(ev).startswith("model_not_a_vla")
    # A stand-in whose hello names AerialVLA's adapter, through the records.
    stub = dict(REAL_MODEL, weights_sha256="0" * 64)
    rec = VM.pilot_entry(stub, {}, [], [], server={}, reported_by="x")
    man = VM.manifest_for("sha256:" + "1" * 64, stub, 1, CFG / "scene_vla.jsonc")
    assert rec["kind"] == "stub" and rec["model_id"].startswith("fake:aerialvla")
    assert TODO.classify_pilot(man["vla_model_hash"]) == "stub"
    out = _tmp() / "fake_named_run"
    out.mkdir()
    (out / "manifest.json").write_text(json.dumps(man), encoding="utf-8")
    import build_dashboard as BD
    assert BD.flight_record(ROOT, out)["pilot"]["class"] == "stub"


# =========================================================================== #
# 4. Configs
# =========================================================================== #
def _jsonc(name):
    from guardrail.manifest import _strip_jsonc
    return json.loads(_strip_jsonc((CFG / name).read_text(encoding="utf-8")))


def test_the_vla_cameras_are_aerialvlas_training_cameras_and_the_airframe_is_the_rails():
    base = _jsonc("robot_semantic_quad.jsonc")
    for scene, robot, det in (("scene_vla.jsonc", "robot_vla_quad.jsonc", False),
                              ("scene_vla_det.jsonc", "robot_vla_quad_det.jsonc", True)):
        sc = _jsonc(scene)
        (actor,) = sc["actors"]
        assert actor["robot-config"] == robot and actor["name"] == "Drone1"
        assert actor["origin"]["xyz"].split()[:2] == ["35.0", "-20.0"], actor["origin"]
        assert sc["clock"]["type"] == "steppable"
        rb = _jsonc(robot)
        for k in ("links", "joints", "controller", "actuators", "physics-type"):
            assert rb[k] == base[k], (robot, k)
        cams = {s["id"]: s for s in rb["sensors"] if s["type"] == "camera"}
        for cid, pitch in (("VLAFront", 0.0), ("VLADown", -90.0)):
            (cs,) = cams[cid]["capture-settings"]
            assert (cs["width"], cs["height"], cs["fov-degrees"], cs["image-type"]) == \
                (256, 256, 90, 0), (cid, cs)
            assert cs["capture-enabled"] and not cs["streaming-enabled"]
            assert float(cams[cid]["origin"]["rpy-deg"].split()[1]) == pitch
            assert cams[cid]["capture-interval"] <= 0.1
        streaming = sorted(c for c, s in cams.items()
                           if any(x.get("streaming-enabled") for x in s["capture-settings"]))
        assert streaming == ["Chase"], streaming
        assert ("FrontCamera" in cams) == det
        if det:
            (cs,) = cams["FrontCamera"]["capture-settings"]
            assert (cs["width"], cs["height"], cs["fov-degrees"]) == (768, 432, 90)
            assert float(cams["FrontCamera"]["origin"]["rpy-deg"].split()[1]) == -20.0
            assert [a["name"] for a in sc["environment-actors"]] == ["SemCarActor"]
        else:
            assert "environment-actors" not in sc
    assert set(vla_fly.SCENES.values()) == {"scene_vla.jsonc", "scene_vla_det.jsonc"}
    assert vla_fly.CAMERAS["front"] == "VLAFront" and vla_fly.CAMERAS["down"] == "VLADown"


def test_the_scenes_feed_the_manifest_and_say_simple_flight():
    from guardrail.manifest import scene_controllers, sim_speedup_from_scene
    for scene in vla_fly.SCENES.values():
        assert sim_speedup_from_scene(CFG / scene) == sim_speedup_from_scene(
            CFG / "scene_semantic.jsonc")
        ctrls, _ = scene_controllers(CFG / scene)
        assert ctrls and all("simple" in str(c).lower() for c in ctrls), ctrls


def test_the_policy_is_flyable_locked_and_its_geometry_is_the_documented_one():
    from guardrail.bundle import check_lock, load_lock, lock_key
    from guardrail.models import AltitudeEnvelope, ObstacleClearance, PolygonFence
    pol = load_policy(POLICY)
    assert pol.flight_problems() == []
    assert lock_key(pol) in load_lock()["policies"]
    assert not [x for x in check_lock() if "vla_goto_citylife" in x]
    (alt,) = pol.by_type(AltitudeEnvelope)
    assert (alt.alt_min_m, alt.alt_max_m) == (6, 14)
    assert VM.DEFAULT_CRUISE_UP == 10.0 and alt.alt_min_m < VM.DEFAULT_CRUISE_UP < alt.alt_max_m
    (clr,) = pol.by_type(ObstacleClearance)
    assert clr.min_clearance_m == 3.0
    (nfz,) = pol.by_type(PolygonFence)
    xs = [v.x for v in nfz.vertices]
    ys = [v.y for v in nfz.vertices]
    gx, gy = VM.DEFAULT_GOAL_XY
    assert not (min(xs) - nfz.margin_m <= gx <= max(xs) + nfz.margin_m
                and min(ys) - nfz.margin_m <= gy <= max(ys) + nfz.margin_m)
    assert min(ys) - gy >= 20.0                        # the zone begins 22 m past the goal
    import build_flight_bundles as BFB
    assert "vla_goto_citylife" in BFB.FLIGHT_POLICIES


def test_the_default_start_and_goal_are_clear_on_the_citylife_map():
    """Read on the Shield's own distance field over the 6-14 m band map
    (demo/out/citymap_citylife, gitignored: SKIP where it is absent). The
    numbers vla_mission.py states: spawn 4.08 m but its eastward lines within
    1.3 m of two trees; start 4.46 m; goal 13.5 m; the start -> goal line
    never under 3.9 m; both on the street mask."""
    import numpy as np
    from guardrail import Shield
    d = ROOT / "demo" / "out" / "citymap_citylife"
    if not (d / "occ_day_flightband_6to14.npz").is_file():
        print("      (no demo/out/citymap_citylife here: the level's map is gitignored)")
        return SKIP
    z = np.load(d / "occ_day_flightband_6to14.npz")
    occ = {"occ": z["occ"], "res": float(z["res"]), "ox": float(z["origin_x"]),
           "oy": float(z["origin_y"])}
    sh = Shield(load_policy(POLICY), obstacle_map=occ, escalation=False)
    (sx, sy), (gx, gy) = VM.DEFAULT_START_XY, VM.DEFAULT_GOAL_XY
    assert abs(sh.clearance_at(*VM.CITYLIFE_SPAWN_XY) - 4.08) < 0.05
    assert abs(sh.clearance_at(sx, sy) - 4.46) < 0.05
    assert sh.clearance_at(gx, gy) > 13.0
    line = [sh.clearance_at(sx + f * (gx - sx), sy + f * (gy - sy)) for f in np.linspace(0, 1, 400)]
    assert min(line) > 3.9, min(line)
    spx, spy = VM.CITYLIFE_SPAWN_XY
    east = [sh.clearance_at(spx + f * (gx - spx), spy + f * (gy - spy))
            for f in np.linspace(0, 0.2, 100)]
    assert min(east) < 1.3, min(east)
    st = np.load(d / "street.npz")["street"]
    for x, y in ((sx, sy), (gx, gy)):
        assert st[int(round((x - occ["ox"]) / 2.0)), int(round((y - occ["oy"]) / 2.0))]


# =========================================================================== #
# 5. The server lifecycle and the launcher
# =========================================================================== #
def test_every_server_start_names_its_whole_configuration():
    a = VM.server_argv("py", **VM.FIDELITY, port=8791, log="L")
    assert a[:3] == ["py", str(VM.SERVER_SCRIPT), "serve"]
    i = a.index("--precision")
    assert a[i:i + 4] == ["--precision", "bf16", "--attn", "sdpa"] and "--no-merge" in a
    assert a[a.index("--port") + 1] == "8791" and a[a.index("--log") + 1] == "L"
    assert a[a.index("--missing-down") + 1] == "refuse"
    b = VM.server_argv("py", **VM.LOW_VRAM)
    assert b[b.index("--precision") + 1] == "nf4" and "--merge" in b and "--no-merge" not in b
    f = VM.server_argv("py", **VM.FIDELITY, fake_output=["96 48 49</s>"], fake_delay_s=0.3)
    assert "--fake-output" in f and "--precision" not in f and "0.3" in f
    for bad in ({"precision": "fp16", "attn": "sdpa", "merge": False},
                {"precision": "bf16", "attn": "flash", "merge": False},
                {"precision": "bf16", "attn": "sdpa", "merge": None}):
        try:
            VM.server_argv("py", **bad)
        except ValueError:
            continue
        raise AssertionError(f"server_argv accepted {bad}")
    ap = vla_fly.build_arg_parser()
    assert vla_fly.parse_cli(ap, []).vla_config == VM.FIDELITY
    assert vla_fly.parse_cli(ap, ["--vla-low-vram"]).vla_config == VM.LOW_VRAM
    assert vla_fly.parse_cli(ap, ["--vla-precision", "nf4"]).vla_config == \
        {"precision": "nf4", "attn": "sdpa", "merge": False}
    assert VM.parse_addr("127.0.0.1:8790") == ("127.0.0.1", 8790)


def test_a_follow_chases_on_a_large_box_and_ends_only_when_lost_or_on_time():
    """M3 follow (2026-10-23, the user's "follow the red car as the previous
    test"): the phrase comes from OWL-ViT's box as in M2, but a large box is
    the point of a follow, not its end - counted, never a stop rule; a lost
    subject and the time limit still end it."""
    cfg = VM.MonitorConfig(mission="follow", max_s=100.0, close_box_frac=0.15,
                           lost_timeout_s=4.0, det_fresh_s=1.0)
    big = (384.0, 200.0, 130.0, 60.0, 0.2, 768, 432, 0.5)
    m = VM.MissionMonitor(cfg, 0.0)
    for k in range(10):
        assert m.update(1.0 + 0.1 * k, FAR, _tick(), det=big, det_age_s=0.3) is None
    assert m.ticks_close_box == 10 and m.summary()["ticks_close_box"] == 10
    assert m.update(5.9, FAR, _tick(), det=None) == "target_lost"
    m = VM.MissionMonitor(VM.MonitorConfig(mission="follow", max_s=2.0, lost_timeout_s=90.0), 0.0)
    assert m.update(2.0, FAR, _tick(), det=big, det_age_s=0.2) == "timeout"
    # The follow needs the detector, as M2 does.
    try:
        VM.MissionLoop(object(), object(), m, mission="follow")
    except ValueError as exc:
        assert "det_source" in str(exc)
    else:
        raise AssertionError("a follow without a det_source was accepted")


def test_follow_scores_read_the_level_car_truth_rows_only():
    rows = [{"t": 0.0, "x": 0, "y": 0}, {"t": 0.1, "x": 0, "y": 0, "subject_dist_m": 12.0},
            {"t": 0.2, "x": 0, "y": 0, "subject_dist_m": 45.0},
            {"t": 0.3, "x": 0, "y": 0, "subject_dist_m": 20.0}]
    s = VM.follow_scores(rows)
    assert s["follow_scored"] and s["ticks_scored"] == 3 and s["ticks"] == 4
    assert s["frac_within"] == round(2 / 3, 3) and s["within_m"] == 30.0
    assert (s["dist_min_m"], s["dist_median_m"], s["dist_max_m"]) == (12.0, 20.0, 45.0)
    assert s["dist_start_m"] == 12.0 and s["dist_end_m"] == 20.0
    assert VM.follow_scores([{"t": 0.0}]) == {"follow_scored": False, "ticks_scored": 0,
                                               "ticks": 1}


def test_the_follow_cli_defaults_to_the_red_car_of_the_city_follow():
    ap = vla_fly.build_arg_parser()
    a = vla_fly.parse_cli(ap, ["--mission", "follow"])
    assert a.object == "a red car" and a.level_car == "Car_10" and a.park_at is None
    assert a.lost_timeout_s == 90.0 and a.start_heading == "street"
    assert a.monitor.mission == "follow" and a.monitor.lost_timeout_s == 90.0
    assert vla_fly.SCENES["follow"] == vla_fly.SCENES["approach"]      # the detector camera
    a = vla_fly.parse_cli(ap, ["--mission", "follow", "--level-car", "Car_3",
                               "--object", "a white van", "--lost-timeout-s", "30"])
    assert (a.level_car, a.object, a.lost_timeout_s) == ("Car_3", "a white van", 30.0)
    assert vla_fly.parse_cli(ap, ["--mission", "approach"]).lost_timeout_s == 15.0
    import contextlib
    import io
    for bad in (["--mission", "follow", "--park-at", "1,2"],
                ["--mission", "follow", "--no-park"],
                ["--mission", "follow", "--goal", "1,2"],
                ["--mission", "goto", "--level-car", "Car_10"],
                ["--mission", "approach", "--level-car", "Car_10"]):
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                vla_fly.parse_cli(ap, bad)
        except SystemExit as exc:
            assert exc.code == 2, bad
            continue
        raise AssertionError(f"accepted {bad}")


def test_the_cli_refuses_what_cannot_fly_honestly_and_reads_negative_xy():
    ap = vla_fly.build_arg_parser()
    for bad in (["--vla-server", "127.0.0.1:8790", "--vla-precision", "bf16"],
                ["--vla-low-vram", "--vla-attn", "eager"],
                ["--mission", "approach", "--goal", "1,2"],
                ["--mission", "goto", "--park-at", "1,2"],
                ["--mission", "approach", "--object", " "],
                ["--policy", "policies/vla_goto_citylife.yaml"],
                ["--start-heading", "north"],
                ["--object", "the goal at x=43 y=40"],          # a position in the words
                ["--object", "a red car "],                     # blanks upstream strips
                ["--start-tolerance-m", "0"],
                ["--cadence", "lockstep"]):
        try:
            import contextlib
            import io
            with contextlib.redirect_stderr(io.StringIO()):
                vla_fly.parse_cli(ap, bad)
        except SystemExit as exc:
            assert exc.code == 2, bad
            continue
        raise AssertionError(f"accepted {bad}")
    a = vla_fly.parse_cli(ap, ["--mission", "goto", "--goal", "-5,10", "--start-xy", "-1,-2"])
    assert a.goal == (-5.0, 10.0) and a.start_xy == (-1.0, -2.0)
    # P3-14: never the empty object slot (no training prompt has one), a
    # fixed neutral description in upstream's style, overridable.
    assert a.object == VM.GOTO_OBJECT_TEXT and a.object.startswith("The target is ")
    from guardrail.aerialvla_codec import build_prompt, validate_object
    assert validate_object(VM.GOTO_OBJECT_TEXT) == VM.GOTO_OBJECT_TEXT
    assert build_prompt("straight ahead ", VM.GOTO_OBJECT_TEXT, None, "off").endswith(
        VM.GOTO_OBJECT_TEXT + "\nAction: ")
    assert vla_fly.parse_cli(ap, ["--object", "a quiet street"]).object == "a quiet street"
    # P2-3: upstream's cadence unless asked; recorded by --check-args too.
    assert a.cadence == "stop_and_go"
    assert vla_fly.parse_cli(ap, ["--cadence", "pipelined"]).cadence == "pipelined"
    rep = vla_fly.check_args_report(a)
    assert rep["ok"] and rep["cadence"] == "stop_and_go" and rep["object"] == VM.GOTO_OBJECT_TEXT
    assert a.vla_hello_timeout_s == VM.HELLO_TIMEOUT_S >= 1800
    a = vla_fly.parse_cli(ap, ["--mission", "approach"])
    assert a.object == "an orange car" and a.park_at == VM.DEFAULT_PARK_XY
    assert a.start_heading == "street" and a.goal is None and a.tag == "vla_approach"
    assert vla_fly.parse_cli(ap, ["--mission", "approach", "--no-park"]).park_at is None


def test_a_started_server_says_hello_and_stops_on_shutdown():
    """The real demo/vla_server_v2.py, with its scripted stand-in (no model,
    no GPU): started as a subprocess, its hello awaited, then shut down -
    exit code 0 by op shutdown, the identity recorded as fake:scripted (not
    a VLA)."""
    port = _free_port()
    out = _tmp()
    argv = VM.server_argv(sys.executable, **VM.FIDELITY, port=port,
                          log=str(out / "srv.jsonl"), fake_output=["96 48 49</s>"])
    srv = VM.VLAServer("127.0.0.1", port, argv=argv, log_path=out / "stdout.log")
    srv.start()
    try:
        hello = srv.wait_hello(60.0, poll_s=0.2)
        assert hello["id"] == "fake:scripted" and srv.alive
        assert VM.identity_is_vla(hello)[0] is False
    finally:
        rec = srv.stop(timeout_s=20.0)
    assert rec["stop"] == "shutdown" and rec["exit_code"] == 0, (rec, srv.log_tail())
    assert rec["mode"] == "started" and rec["identity"]["id"] == "fake:scripted"


def test_a_fake_id_is_never_a_vla_whatever_its_digest():
    """Re-review of the fix round, N4: a stand-in reporting
    `fake:aerialvla...` with a real-looking digest and every identity field
    passed identity_is_vla, and the run was recorded as kind vla (the
    prefix lost under MODEL_ID, so todo's `fake:` rule never saw it). A
    `fake:` id is a stand-in whatever else it carries."""
    fake = dict(REAL_MODEL, id="fake:" + REAL_MODEL["id"])
    ok, why = VM.identity_is_vla(fake)
    assert ok is False and "fake:" in (why or ""), (ok, why)
    assert VM.identity_is_vla(dict(REAL_MODEL)) == (True, None)


def test_the_warm_up_gate_judges_the_last_of_several_requests():
    """Re-review of the fix round, N1: the warm-up was the model's first
    inference ever (the server runs none at start), and a cold first call
    beside CityLife can exceed the 5 s gate, so every launcher run would
    exit 4 before take-off. Several requests are sent and the LAST one is
    judged; every attempt is recorded."""
    cold = {"answered": True, "kind": "step", "rtt_s": 9.0}
    warm = {"answered": True, "kind": "step", "rtt_s": 0.6}
    assert VM.WARMUP_REQUESTS >= 2
    assert VM.warmup_series_verdict([cold, warm, warm], 5.0) == (True, None)
    ok, why = VM.warmup_series_verdict([warm, warm, cold], 5.0)
    assert not ok and "over the 5 s gate" in why
    ok, why = VM.warmup_series_verdict([], 5.0)
    assert not ok and "no warm-up" in why
    silent = {"answered": False, "failure": "timeout", "detail": "no answer"}
    assert VM.warmup_series_verdict([warm, silent], 5.0)[0] is False


def _city_warmup(respond, decision_timeout_s=1.0):
    """demo/vla_fly._warmup on its own event loop, as the flight runs it,
    against the scripted server; the drone, the pose ring and the camera
    pair are stand-ins. The client is the flight's: response timeout = the
    decision timeout, late answers awaited 120 s."""
    import asyncio
    srv = F.FakeVLAServer(respond, model=REAL_MODEL)
    srv.start()
    commits = []
    try:
        client = RemoteVLA("127.0.0.1", srv.port, response_timeout_s=decision_timeout_s,
                           reconnect_backoff_s=0.5, late_reply_wait_s=120.0,
                           strict_identity=True, clock=time.time)
        client.connect()
        frames = SimpleNamespace(
            capture=lambda: Capture(time.time(), Pose(42.0, -18.0, 10.0, 1.57), JPG, JPG),
            commit=lambda *a, **k: commits.append(k), stats=lambda: {})
        kin = {"pose": {"position": {"x": 42.0, "y": -18.0, "z": -10.0},
                        "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}},
               "twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}}, "time_stamp": 1}
        args = SimpleNamespace(mission="goto", decision_timeout_s=decision_timeout_s,
                               first_decision_timeout_s=60.0, warmup_max_latency_s=5.0,
                               object=VM.GOTO_OBJECT_TEXT)
        res = asyncio.run(vla_fly._warmup(
            args, SimpleNamespace(get_ground_truth_kinematics=lambda: kin),
            SimpleNamespace(put_pose_full=lambda *a, **k: None), frames, client,
            VM.goto_hint_source((43.0, 40.0), 3.0), State, lambda q: 0.0,
            SimpleNamespace(quat_pitch=lambda q: 0.0, quat_roll=lambda q: 0.0),
            SimpleNamespace(beat=lambda **k: None)))
        client.close()
    finally:
        srv.stop()
    return res, srv, commits


def test_the_city_warm_up_gives_a_slow_cold_first_answer_its_timeout():
    """Review of the ArduPilot rail's VLA pilot (4a0da6d, P1-1), on the
    MERGED city rail. The first warm-up request was promised
    first_decision_timeout_s (60 s) and recorded so, while the client's
    socket timed out at its response timeout (the decision timeout), then
    drained the late answer: requests 2 and 3 were "not sent" and a healthy
    server with a 3 s cold start refused the flight. Each request now
    carries the timeout recorded for it and waits for a ready client
    (vla_mission.wait_client_ready, shared with the ArduPilot rail)."""
    base = _with_model(F.goto_responder((43.0, 40.0), 10.0), REAL_MODEL)
    n = {"k": 0}

    def respond(req):
        n["k"] += 1
        out = base(req)
        return F.Delay(3.0, out) if n["k"] == 1 else out
    res, srv, commits = _city_warmup(respond)
    assert res["pass"] is True, res["why"]
    assert [a["timeout_s"] for a in res["attempts"]] == [60.0, 1.0, 1.0], res["attempts"]
    assert [a["kind"] for a in res["attempts"]] == ["step"] * 3
    assert len(srv.requests) == 3 and len(commits) == 3


def test_the_city_warm_up_waits_out_the_backoff_after_a_refused_first_request():
    """Review of 4a0da6d, P2-8: after a server error the client backs off,
    and the city rail's warm-up asked again at once - "not sent" twice, the
    flight refused although the model then answered."""
    base = _with_model(F.goto_responder((43.0, 40.0), 10.0), REAL_MODEL)
    n = {"k": 0}

    def respond(req):
        n["k"] += 1
        return F.error_reply(req, "warming up") if n["k"] == 1 else base(req)
    res, srv, _ = _city_warmup(respond, decision_timeout_s=10.0)
    assert res["pass"] is True, res["why"]
    assert [a["kind"] for a in res["attempts"]] == ["server_error", "step", "step"], \
        [(a["kind"], a["failure"]) for a in res["attempts"]]
    assert res["attempts"][1]["client_ready"]["waited_on"].get("backoff", 0) >= 1


def test_an_occupied_port_and_a_dying_server_are_refused_before_take_off():
    with F.FakeVLAServer(F.goto_responder((0, 0), 10.0), model=REAL_MODEL) as fake:
        srv = VM.VLAServer("127.0.0.1", fake.port, argv=[sys.executable, "-c", "pass"])
        try:
            srv.start()
        except RuntimeError as exc:
            assert "already listens" in str(exc)
        else:
            raise AssertionError("started a server on an occupied port")
        attached = VM.VLAServer("127.0.0.1", fake.port)
        attached.start()
        hello = attached.wait_hello(5.0, poll_s=0.1)
        assert hello["weights_sha256"] == REAL_MODEL["weights_sha256"]
        assert VM.identity_is_vla(hello) == (True, None)
        assert attached.alive and attached.stop()["mode"] == "attached"
    out = _tmp()
    dying = VM.VLAServer("127.0.0.1", _free_port(),
                         argv=[sys.executable, "-c", "import sys; print('boom'); sys.exit(3)"],
                         log_path=out / "stdout.log")
    dying.start()
    try:
        dying.wait_hello(30.0, poll_s=0.2)
    except VM.ServerExited as exc:
        assert "code 3" in str(exc) and "boom" in str(exc), exc
    else:
        raise AssertionError("a dead server was waited on")
    finally:
        dying.stop()


def _pwsh():
    return shutil.which("pwsh") or shutil.which("powershell")


def _launch(args: str, *, file_mode: bool = False):
    """scripts/run_vla_fly.ps1 itself, its REAL parameter binding, with
    -PrintArgs (nothing started). `args` is PowerShell command text (so
    -Goal 43,100 binds as PowerShell binds it typed at a prompt), or with
    file_mode the arguments of `powershell -File` (each one a string)."""
    script = ROOT / "scripts" / "run_vla_fly.ps1"
    if file_mode:
        cmd = [_pwsh(), "-NoProfile", "-NonInteractive", "-File", str(script),
               *args.split(), "-PrintArgs"]
    else:
        cmd = [_pwsh(), "-NoProfile", "-NonInteractive", "-Command",
               f"& '{script}' {args} -PrintArgs"]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    return r, [ln.rstrip("\r") for ln in r.stdout.splitlines() if ln.strip()]


def test_the_launcher_assembles_the_arguments_the_flight_parses():
    """scripts/run_vla_fly.ps1 RUN under PowerShell with its real parameter
    binding (-PrintArgs: nothing starts; static checks first, so a host
    without PowerShell still checks the text): the precision maps to the
    explicit server flags, the cadence and the tag pass through, and the
    flight's own parser accepts the list.

    Review of the CityLife flight, P2-4: -Goal 43,100 typed at a prompt binds
    as TWO values; declared [string] it arrived as "43 100" and the flight
    refused it only after the simulator was up (the earlier version of this
    test cut the function out and handed it a pre-joined string, so it
    could not see that). Now both spellings, -Goal 43,100 and -Goal
    "43,100", and -File's string, reach the flight as "43,100"; a value
    that is not two finite numbers is refused by the launcher before
    anything starts, and the real launch runs vla_fly.py --check-args
    before the simulator too."""
    src = (ROOT / "scripts" / "run_vla_fly.ps1").read_text(encoding="utf-8")
    assert "/Game/CityLife/Maps/CityLife_Day" in src
    assert "function Get-VlaFlyArgs" in src and "demo\\vla_fly.py" in src
    assert "OffloadRoot" in src and "Stop-OurGameSim" in src and "Stop-OurVlaServer" in src
    assert "[string[]]$Goal" in src and "[string[]]$ParkAt" in src
    # The flight's parser runs before Start-Sim is first called.
    assert src.index("& $Py @a --check-args") < src.index("if (-not $SkipSim) { Start-Sim }")
    assert src.index("if ($PrintArgs)") < src.index("Resolve-UnrealEngine.ps1\"")
    if _pwsh() is None:
        print("      (no PowerShell on this host: the text checks above ran)")
        return SKIP
    cases = [
        ("-Mission goto -Seconds 120 -Tag t1 -Precision bf16",
         ["--mission", "goto", "--tag", "t1", "--max-s", "120", "--cadence", "stop_and_go"],
         {"precision": "bf16", "attn": "sdpa", "merge": False}, None),
        ("-Mission approach -Seconds 90 -Tag t2 -Precision nf4 -Object 'a red car' -ParkAt -5,12",
         ["--mission", "approach", "--object", "a red car", "--park-at", "-5,12"],
         {"precision": "nf4", "attn": "sdpa", "merge": True}, (-5.0, 12.0)),
        ("-Mission goto -Seconds 60 -Tag t3 -Goal 43,100 -Shield off -VlaServer 127.0.0.1:8790"
         " -Cadence pipelined",
         ["--goal", "43,100", "--shield", "off", "--vla-server", "127.0.0.1:8790",
          "--cadence", "pipelined"], None, (43.0, 100.0)),
        ("-Mission goto -Tag t4 -Goal '43,100'", ["--goal", "43,100"], None, (43.0, 100.0)),
        ("-Mission goto -Tag t5 -Goal 43.5,-7", ["--goal", "43.5,-7"], None, (43.5, -7.0)),
    ]
    for text, want, cfg, xy in cases:
        r, got = _launch(text)
        assert r.returncode == 0, (text, r.stderr[-1500:])
        assert got[0].endswith("vla_fly.py"), got
        for w in want:
            assert w in got, (w, got)
        a = vla_fly.parse_cli(vla_fly.build_arg_parser(), got[1:] + ["--dev-yaml"])
        if cfg is not None:
            assert a.vla_config == cfg, (a.vla_config, cfg)
        if xy is not None:
            assert (a.goal if a.mission == "goto" else a.park_at) == xy, (text, got)
        if "VlaServer" in text:
            assert a.vla_server == "127.0.0.1:8790" and a.shield == "off"
        assert a.cadence == ("pipelined" if "pipelined" in text else "stop_and_go")
    r, got = _launch("-Mission goto -Tag t6 -Goal 43,100", file_mode=True)
    assert r.returncode == 0 and got[got.index("--goal") + 1] == "43,100", (got, r.stderr)
    for bad in ("-Goal 43", "-Goal '1,000,5'", "-Goal 'a,b'", "-Mission approach -ParkAt '48;12'"):
        r, got = _launch(bad)
        assert r.returncode != 0 and "is not X,Y" in (r.stderr + r.stdout), (bad, r.stdout,
                                                                              r.stderr[-800:])


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
        except Exception as e:                        # noqa: BLE001
            failed += 1
            import traceback
            traceback.print_exc()
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
