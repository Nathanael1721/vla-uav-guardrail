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
8. The review of the AerialVLA follow (d6e7123, 2026-10-23): the follow's
   score is at the top of metrics, where kpi and the dashboard read it, with
   follow_vlm's meaning, and tools/rescore_vla_follow.py re-scores a run
   flown before (P1-1); each request and row says whether its hint was a
   fresh box, the memory or the search (P2-2); with identity, a lapse - the
   strict lock's "no held instance" or LAPSE_S unseen - re-acquires at once
   from the last sighting, shown with follow_vlm's real Grounder state,
   strict TargetLock and Reacquirer (P2-3; the unfixed code never took the
   reappearing car back); a perception reset ignores the dropped box and,
   with identity, re-acquires directly (P2-4); the start gate is decided
   after the identity setup and recorded as applied (P2-5); a LAND held as
   arrival is not a landing the VLA asked for (P2-6); a follow that never
   asked is subject_not_acquired, not a VLA failure (P2-7); a close box with
   no request is a stall that resets the perception (P2-8); and the P3s -
   pilot_raw and close-held shares, StuckWatch on what flew, the flags and
   records agreeing, measured outcomes in metrics in mission time, one
   detector read per tick, the depth stream in sim_config, -ExtraArgs.
   Every one failed on df10863 (missing names, or the old records).

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
        # The detector camera's depth stream (2026-10-23) feeds the follow's
        # identity check, as follow_vlm's FrontCamera does; streamed like it.
        assert streaming == (["Chase", "FrontCamera"] if det else ["Chase"]), streaming
        assert ("FrontCamera" in cams) == det
        if det:
            cs, dep = cams["FrontCamera"]["capture-settings"]
            assert (cs["width"], cs["height"], cs["fov-degrees"]) == (768, 432, 90)
            assert cs["image-type"] == 0 and not cs["streaming-enabled"]
            assert (dep["image-type"], dep["width"], dep["height"], dep["fov-degrees"]) == \
                (2, 768, 432, 90) and dep["pixels-as-float"]
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


def test_the_follow_hint_remembers_then_searches_and_asks_nothing_when_close():
    """DetectorHint memory_s / search_s / close (2026-10-23, the follow): a
    box lost keeps the phrase of its last bearing, held in the world frame
    and re-read against the current heading, for memory_s; then a quarter
    turn's phrase toward the side it was last seen on for search_s; then no
    request. A close box (bottom edge low, or wide) asks for no new step.
    The protocol's hint source stays "detector". With both 0: M2's rule."""
    from vla_bridge import VLABridge, bearing_from_box
    br = VLABridge("a red car", hfov_deg=90.0)
    box = {"v": None}

    def latest():
        return box["v"] if box["v"] is not None else (None, None, None, None)

    h = VM.DetectorHint(latest, br, fresh_s=1.0, memory_s=6.0, search_s=20.0,
                        close=VM.BoxClose(0.92, 0.25))

    def cap(t, yaw=0.0):
        return Capture(t, Pose(0.0, 0.0, 10.0, yaw), JPG, JPG)
    right = (700.0, 200.0, 40.0, 30.0, 0.3, 768, 432, 0.5)       # well right of centre
    b = bearing_from_box(right, 90.0)
    assert b > math.radians(30)
    box["v"] = (right, 10.0, 0.0, 10.1)
    hint = h(cap(10.5))
    assert hint is not None and hint.source == "detector" and h.last_kind == "fresh"
    fresh_phrase = hint.phrase
    box["v"] = None
    hint = h(cap(13.0))                                          # 3 s on, same heading
    assert hint.phrase == fresh_phrase and h.last_kind == "memory"
    hint = h(cap(14.0, yaw=b))                                   # turned to face it
    assert hint.phrase == "straight ahead " and h.last_kind == "memory"
    hint = h(cap(20.0))                                          # past memory: search
    assert hint.phrase == "to your right " and h.last_kind == "search"
    assert h(cap(40.0)) is None                                  # past the search
    k = h.stats()["hints_by_kind"]
    assert (k["fresh"], k["memory"], k["search"]) == (1, 2, 1), k
    low = (384.0, 380.0, 100.0, 80.0, 0.3, 768, 432, 0.5)       # bottom at 420 / 432
    box["v"] = (low, 50.0, 0.0, 50.1)
    assert h(cap(50.3)) is None and h.last_kind == "close" and h.stats()["close_no_request"] == 1
    wide = (384.0, 200.0, 200.0, 60.0, 0.3, 768, 432, 0.5)      # 0.26 of the width
    assert VM.BoxClose(0.92, 0.25)(wide) and not VM.BoxClose(0.92, 0.25)(right)
    m2 = VM.DetectorHint(latest, br, fresh_s=1.0)
    box["v"] = (right, 60.0, 0.0, 60.1)
    assert m2(cap(60.5)) is not None and m2(cap(65.0)) is None   # M2: stale, no request


def test_the_follow_start_gate_waits_for_a_near_box_on_successive_frames():
    """DetectorHint start_gate (n, width_frac), follow_vlm's --start-when-seen
    for the VLA follow: no request (and no memory or search) until n
    successive detector frames gave a fresh box at least width_frac wide; a
    far box or a gap resets the streak; then the hint flows as usual."""
    from vla_bridge import VLABridge
    br = VLABridge("a red car", hfov_deg=90.0)
    box = {"v": None}
    h = VM.DetectorHint(lambda: box["v"] or (None, None, None, None), br, fresh_s=1.0,
                        memory_s=6.0, search_s=20.0, start_gate=(3, 0.04))

    def cap(t):
        return Capture(t, Pose(0.0, 0.0, 10.0, 0.0), JPG, JPG)
    near = (384.0, 200.0, 40.0, 30.0, 0.3, 768, 432, 0.5)       # 0.052 wide
    far = (384.0, 200.0, 20.0, 15.0, 0.3, 768, 432, 0.5)        # 0.026 wide
    assert h(cap(0.5)) is None and h.last_kind == "gated"          # no box at all
    for k, b in enumerate((near, near, far, near, near)):        # the far box resets
        box["v"] = (b, 1.0 + k, 0.0, 1.05 + k)
        assert h(cap(1.2 + k)) is None and not h.gate_open, k
    box["v"] = None
    assert h(cap(6.5)) is None and h.last_kind == "gated"          # no memory before the gate
    for k in range(3):
        box["v"] = (near, 10.0 + k, 0.0, 10.05 + k)
        out = h(cap(10.2 + k))
    assert h.gate_open and out is not None and out.phrase == "straight ahead "
    g = h.stats()["start_gate"]
    assert g["open"] and g["n"] == 3 and g["gated_calls"] >= 7 and g["t_open"] == 12.2


def test_a_forgotten_box_can_neither_reseed_the_memory_nor_count_as_seen():
    """Review of d6e7123, P2-4: the stuck reset called forget(), which
    cleared the sighting - but the wrong box stayed fresh up to det_fresh_s
    and the Grounder kept publishing it up to its drop_after (8 s), so the
    next hint re-seeded the memory with the very object just dropped.
    forget(t_reset) now also ignores every box whose frame was captured at
    or before t_reset (counted); a box from a later frame is the hint's
    again."""
    from vla_bridge import VLABridge
    box = {"v": None}
    h = VM.DetectorHint(lambda: box["v"] or (None, None, None, None),
                        VLABridge("a red car", hfov_deg=90.0), fresh_s=1.0, memory_s=6.0,
                        search_s=20.0, record_kind=True)

    def cap(t):
        return Capture(t, Pose(0.0, 0.0, 10.0, 0.0), JPG, JPG)
    right = (700.0, 200.0, 40.0, 30.0, 0.3, 768, 432, 0.5)
    box["v"] = (right, 10.0, 0.0, 10.1)
    assert h(cap(10.2)).kind == "fresh"
    h.forget(10.3)                                   # the reset, at 10.3
    assert h(cap(10.4)) is None and h.last_kind is None, h.last_kind   # same box: ignored
    assert h(cap(14.0)) is None                      # and no memory of it
    assert h.box_ignored(10.0) and h.box_ignored(10.3) and not h.box_ignored(10.31)
    st = h.stats()
    assert st["ignored_after_reset"] == 2 and st["hints_by_kind"]["forgotten"] == 1
    box["v"] = (right, 14.5, 0.0, 14.6)              # a frame after the reset
    got = h(cap(14.7))
    assert got is not None and got.kind == "fresh"
    assert not VM.DetectorHint(lambda: (None, None), VLABridge("a red car")).box_ignored(1.0)


def test_the_follow_start_gate_can_be_armed_after_the_identity_setup():
    """Review of d6e7123, P2-5: the DetectorHint's start gate was decided
    from --no-identity before the identity setup ran, so a follow whose
    identity failed to load flew with no gate at all. The hint is now built
    ungated and armed afterwards (set_start_gate), once the flight knows
    which gate applies; its stats give the gate's opening in mission time
    (P3: it was an epoch time)."""
    from vla_bridge import VLABridge
    box = {"v": None}
    h = VM.DetectorHint(lambda: box["v"] or (None, None, None, None),
                        VLABridge("a red car", hfov_deg=90.0), fresh_s=1.0)
    assert h.gate_open and h.start_gate is None
    h.set_start_gate((2, 0.04))
    assert not h.gate_open and h.stats()["start_gate"]["n"] == 2
    near = (384.0, 200.0, 40.0, 30.0, 0.3, 768, 432, 0.5)

    def cap(t):
        return Capture(t, Pose(0.0, 0.0, 10.0, 0.0), JPG, JPG)
    for k in range(2):
        box["v"] = (near, 1000.0 + k, 0.0, 1000.05 + k)
        out = h(cap(1000.2 + k))
    assert h.gate_open and out is not None
    g = h.stats(t0=990.0)["start_gate"]
    assert g["t_open_s"] == 11.2 and "t_open" not in g, g
    assert h.stats()["start_gate"]["t_open"] == 1001.2          # no t0: as before
    for bad in ((0, 0.04), (2, 1.5)):
        try:
            VM.DetectorHint(lambda: (None, None), VLABridge("a red car")).set_start_gate(bad)
        except ValueError:
            continue
        raise AssertionError(f"start gate {bad} accepted")
    h2 = VM.DetectorHint(lambda: (None, None), VLABridge("a red car"), start_gate=(3, 0.04))
    h2.set_start_gate(None)
    assert h2.gate_open and h2.start_gate is None


def test_a_close_box_with_no_request_for_too_long_is_a_stall():
    """Review of d6e7123, P2-8: a locked WRONG box that counts as close (>=
    0.25 of the width) asks for no step, the close hold zeroes the motion,
    the tracker hovers, the Shield stays quiet (no StuckWatch) and the box
    stays fresh (no target_lost): a hover until max_s. CloseStall fires once
    when a close box has lasted max_s with no request sent; a request sent,
    or the box no longer close, starts the spell again; 0 disables it."""
    w = VM.CloseStall(8.0)
    fired = [k * 0.1 for k in range(200) if w.update(k * 0.1, True, 5)]
    assert len(fired) == 2 and abs(fired[0] - 8.0) < 0.11 and fired[1] - fired[0] >= 8.0, fired
    assert w.as_dict() == {"max_s": 8.0, "stalls": 2}
    w = VM.CloseStall(8.0)
    assert not any(w.update(k * 0.1, True, k // 50) for k in range(200))   # a request every 5 s
    w = VM.CloseStall(8.0)
    assert not any(w.update(k * 0.1, k % 60 != 0, 5) for k in range(200))  # broken spells
    w = VM.CloseStall(0.0)
    assert not any(w.update(k * 0.1, True, 5) for k in range(200))
    try:
        VM.CloseStall(-1.0)
    except ValueError:
        pass
    else:
        raise AssertionError("a negative stall time accepted")


def test_the_detector_scenes_record_the_depth_stream_they_render():
    """Review of d6e7123 (P3): the follow's identity check added a depth
    stream to the detector camera of robot_vla_quad_det.jsonc, which the
    APPROACH shares: an approach flown since renders it unread, so its render
    load is not the earlier approaches'. sim_config_record reads the scene
    and its robot config (hashes, the detector camera's streams) and says
    whether this run read the depth, for metrics.json."""
    rec = VM.sim_config_record(CFG / "scene_vla_det.jsonc", depth_read=False)
    assert rec["scene"] == "scene_vla_det.jsonc" and rec["robot_config"] == "robot_vla_quad_det.jsonc"
    assert len(rec["scene_sha256"]) == 64 and len(rec["robot_config_sha256"]) == 64
    assert rec["detector_camera"] == "FrontCamera" and rec["detector_streams"] == ["scene", "depth"]
    assert rec["detector_depth_rendered"] is True and rec["detector_depth_read"] is False
    assert "d6e7123" in rec["comparability"]
    assert VM.sim_config_record(CFG / "scene_vla_det.jsonc", depth_read=True)["detector_depth_read"]
    bad = VM.sim_config_record(CFG / "no_such_scene.jsonc", depth_read=False)
    assert bad["problems"] and bad["detector_depth_rendered"] is None


def test_the_stuck_watch_fires_once_when_the_shield_pins_the_aircraft():
    """StuckWatch (follow #4: 2447 Shield blocks in 225 s against a building):
    the Shield touching >= 80 % of 4 s of ticks with under 1 m of progress
    fires once, then waits its cooldown; moving, or the Shield quiet, never."""
    w = VM.StuckWatch(window_s=4.0, touched_frac=0.8, min_progress_m=1.0, cooldown_s=6.0)
    fired = [t for t in [k * 0.1 for k in range(100)]
             if w.update(t, 10.0 + 0.001 * t, 5.0, True)]
    assert len(fired) == 2 and 3.7 <= fired[0] <= 4.1 and fired[1] - fired[0] >= 6.0, fired
    assert w.as_dict()["resets"] == 2
    moving = VM.StuckWatch()
    assert not any(moving.update(k * 0.1, k * 0.2, 0.0, True) for k in range(100))
    quiet = VM.StuckWatch()
    assert not any(quiet.update(k * 0.1, 0.0, 0.0, k % 3 == 0) for k in range(100))


def test_the_follow_acquires_with_identity_then_reacquires_after_a_lapse():
    """vla_fly._acquire_step (follow #7, 23 Oct: identity's strict lock was
    never seeded, so no box ever came and AerialVLA was never asked): each
    new detector frame steps the Reacquirer (a frame seen twice, once); its
    pick is committed once reached, at a mission time. Since the review of
    d6e7123 (P2-3) a committed subject unseen past LAPSE_S restarts the
    acquisition at once, anchored at its last sighting - here the
    committing one - not after memory + search (26 s). The real lock and
    Reacquirer: the two tests above."""
    clock = {"t": 100.0}

    class _G:
        def __init__(self):
            self.state = {"seq": 0, "t": clock["t"], "t_det": 0.0, "det": None,
                          "cands": [("c",)], "cand_tiers": ["OK"], "cand_feats": [{}],
                          "inf_cap": {"t_capture": 99.9, "pose": {"yaw": 0.3}}}
            self.commits, self.reacq = [], 0

        def latest(self):
            return dict(self.state)

        def commit(self, cand, yaw=None):
            self.commits.append((cand, yaw))

        def begin_reacquire(self):
            self.reacq += 1

    class _A:
        def __init__(self):
            self.n, self.pick, self.pick_feat, self.anchors = 0, ("pick",), {"P": (5.0, 1.0)}, []

        def step(self, cands, tiers, feats, t_cap):
            self.n += 1
            return self.n >= 3

        def start(self, anchor):
            self.anchors.append(anchor)
    g, a = _G(), _A()
    box = vla_fly.new_acquisition(a, t0=90.0)
    args = SimpleNamespace(follow_memory_s=6.0, follow_search_s=20.0)
    for k in range(5):
        clock["t"] = round(clock["t"] + 0.2, 6)
        g.state.update(seq=k + 1, t=clock["t"])
        vla_fly._acquire_step(g, box, args, clock=lambda: clock["t"])
        vla_fly._acquire_step(g, box, args, clock=lambda: clock["t"])   # same frame: one step
    assert a.n == 3 and box["committed"] and g.commits == [(("pick",), 0.3)]
    assert box["commits"] == [10.6], box["commits"]                     # mission seconds
    # Right after the commit t_det is still 0: no instant re-acquisition.
    clock["t"] = 101.1
    g.state.update(seq=9, t=clock["t"], det=None, t_det=0.0)
    vla_fly._acquire_step(g, box, args, clock=lambda: clock["t"])
    assert box["committed"] and g.reacq == 0
    clock["t"] = 103.4                                                  # 2.8 s unseen
    g.state.update(seq=10, t=clock["t"])
    vla_fly._acquire_step(g, box, args, clock=lambda: clock["t"])
    assert box["committed"] and g.reacq == 0
    clock["t"] = 103.8                                                  # 3.2 s: the lapse
    g.state.update(seq=11, t=clock["t"])
    vla_fly._acquire_step(g, box, args, clock=lambda: clock["t"])
    assert not box["committed"] and g.reacq == 1 and box["reacquires"] == 1
    assert a.anchors[-1] == {"t": 99.9, "P": (5.0, 1.0), "v": None}, a.anchors
    assert box["lapses"][0]["why"].startswith("unseen") and box["lapses"][0]["t"] == 13.8


def _strict_grounder():
    """follow_vlm's REAL Grounder (not started: no OWL-ViT) with its REAL
    strict TargetLock, as vla_fly builds it with identity on."""
    import follow_vlm as FV
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        g = FV.Grounder(SimpleNamespace(pose=None), "a red car",
                        lock=FV.TargetLock(gate_frac=0.12), identity={"hfov": 90.0})
    return g


def _strict_inference(g, now, cands, feats, t_capture, yaw=0.0):
    """One inference's bookkeeping, exactly as follow_vlm.Grounder._worker
    does it on the identity path (every candidate "ok" here): the seed a
    commit() handed over is taken first (the lock reset and seeded), then -
    unless acquiring - TargetLock.select_strict with the estimator prior
    (_prior_at: None without set_prior), then the publication under the
    Grounder's lock, the box dropped after drop_after. Only the detector
    and the identity judging are left out; the times are `now`."""
    if g._seed is not None:
        (seed, seed_yaw), g._seed = g._seed, None
        g.lock.reset()
        g.lock.seed(seed, seed_yaw if seed_yaw is not None else yaw, now)
    acquiring = g.acquiring
    tiers = ["ok"] * len(cands)
    pose = {"x": 40.0, "y": 0.0, "up": 10.0, "yaw": yaw}
    det = det_feat = lock_why = None
    if cands and not acquiring:
        k, lock_why = g.lock.select_strict(cands, tiers, feats, 768, yaw, now,
                                           prior=g._prior_at(pose, 768, 432, t_capture))
        if k is not None:
            det, det_feat = cands[k], feats[k]
    with g._lock:
        g._seq += 1
        g._t = now
        g._cands, g._cand_tiers, g._cand_feats = list(cands), list(tiers), list(feats)
        g._lock_why = lock_why
        g._inf_cap = {"t_cap": t_capture, "t_capture": t_capture, "pose": pose}
        if det is not None:
            g._det, g._t_det, g._det_tier, g._det_feat = det, now, "ok", det_feat
            g._det_cap = {"t_cap": t_capture, "t_capture": t_capture, "pose": pose}
        elif g._t_det and now - g._t_det > g.drop_after:
            g._det = None
    return det, lock_why


def _chase_with_a_gap(gap_cands, t_gap=(3.0, 5.6), t_end=9.0, need=3):
    """A red car driving +x at 5 m/s, seen at the image centre at 5 Hz, then
    hidden for t_gap (the detector giving `gap_cands(t)` instead), then seen
    again where it has driven to. vla_fly._acquire_step drives follow_vlm's
    real Reacquirer (identity on) against the real Grounder state and strict
    lock; a DetectorHint reads the Grounder as vla_fly's latest_box does.
    Returns (frames: [(t, fresh det published?, lock_why, hint kind)], the
    acquisition box, the Reacquirer)."""
    import follow_vlm as FV
    from vla_bridge import VLABridge
    g = _strict_grounder()
    acq = FV.Reacquirer(need=need, max_range_m=30.0)
    acq.start(None)
    g.begin_acquire()
    box = vla_fly.new_acquisition(acq, t0=0.0)
    args = SimpleNamespace(follow_memory_s=6.0, follow_search_s=20.0)
    clock = {"t": 0.0}

    def latest_box():
        gw = g.latest()
        if gw.get("det") is None:
            return None, None, None, None
        cap = gw.get("cap") or {}
        return gw["det"], cap.get("t_capture"), 0.0, gw.get("t_det")
    hint = VM.DetectorHint(latest_box, VLABridge("a red car", hfov_deg=90.0), fresh_s=1.0,
                           memory_s=6.0, search_s=20.0, record_kind=True)
    frames = []
    t = 0.0
    while t < t_end - 1e-9:
        clock["t"] = t
        x = 60.0 + 5.0 * t
        car = (384.0, 200.0, 40.0, 25.0, 0.3, 768, 432, 0.6)
        car_f = {"P": (x, 0.0), "rng_h": 20.0}
        if t_gap[0] <= t < t_gap[1]:
            cands, feats = gap_cands(t)
        else:
            cands, feats = [car] + gap_cands(t)[0], [car_f] + gap_cands(t)[1]
        import contextlib
        import io
        with contextlib.redirect_stdout(io.StringIO()):
            det, why = _strict_inference(g, t, cands, feats, t - 0.1)
            vla_fly._acquire_step(g, box, args, clock=lambda: clock["t"])
        h = hint(Capture(t + 0.05, Pose(40.0, 0.0, 10.0, 0.0), JPG, JPG))
        frames.append((round(t, 3), det is not None, why, None if h is None else h.kind))
        t = round(t + 0.2, 10)
    return frames, box, acq


def test_a_lost_subject_is_reacquired_at_once_where_it_was_last_seen():
    """Review of d6e7123, P2-3 - the follows' longest blackout. With
    identity, follow_vlm's strict lock adopts nothing it was not seeded
    with: after its 2 s hold it answers "no held instance" unless an
    estimator prior stands in, and vla_fly never set one. _acquire_step only
    restarted the acquisition after memory_s + search_s (26 s) unseen, and
    counted "unseen" only once the Grounder dropped the stale box (8 s): #8
    spent 1060 of 1306 detector frames acquiring, #9 hovered 54 % of the
    time. Now the follow does what follow_vlm does on a lapse with no
    estimator: at the lock's "no held instance" (or LAPSE_S unseen) it
    re-acquires at once - begin_reacquire, and the Reacquirer started from
    an anchor at the last sighting (its place, capture time and measured
    speed), so the car is taken back within `need` frames of reappearing
    where it could have driven, with no motion test; the memory and search
    phrases flow meanwhile. Real Grounder state, strict lock, Reacquirer."""
    wrong = (700.0, 200.0, 40.0, 25.0, 0.3, 768, 432, 0.6)      # something else, far off
    frames, box, acq = _chase_with_a_gap(lambda t: ([wrong], [{"P": (75.0, 60.0),
                                                               "rng_h": 28.0}]))
    committed_t = [t for t, d, _, _ in frames if d]
    assert committed_t and committed_t[0] <= 1.4, frames[:10]     # acquired, moving, by 1.2 s
    gap = [f for f in frames if 3.0 <= f[0] < 5.6]
    assert any(w == "no held instance" for _, _, w, _ in gap), gap
    lapse = box["lapses"][0]
    assert lapse["why"] == "lock: no held instance" and lapse["t"] == 5.0, lapse
    assert lapse["P"] == [74.0, 0.0] and abs(lapse["v"] - 5.0) < 0.2, lapse
    assert acq.anchor is not None and acq.anchor["P"] == (74.0, 0.0)
    back = [t for t, d, _, _ in frames if d and t >= 5.6]
    assert back and back[0] <= 6.4, frames[25:]                  # need=3 frames, then the seed
    assert box["reacquires"] == 1 and box["committed"] and len(box["commits"]) == 2
    # Mission time. (The frame at t = 0 is no frame to _acquire_step: a
    # Grounder time of 0 means it never ran - so six frames from 0.2 s.)
    assert box["commits"][0] == 1.2 and box["commits"][1] <= 6.2, box["commits"]
    # While it was gone the phrase kept coming: the remembered sighting (the
    # last box is fresh for det_fresh_s into the gap, then memory).
    kinds = {k for t, _, _, k in frames if 4.0 <= t < 5.6}
    assert kinds == {"memory"}, kinds


def test_a_subject_gone_without_a_trace_is_reacquired_after_the_lapse():
    """P2-3 with nothing in view at all (the lock is never asked, so it never
    says "no held instance"): LAPSE_S unseen (follow_vlm's --lapse-s, past
    the lock's 2 s hold) starts the anchored re-acquisition; the car is back
    within the Reacquirer's frames of reappearing."""
    frames, box, _ = _chase_with_a_gap(lambda t: ([], []), t_gap=(3.0, 6.4))
    assert vla_fly.LAPSE_S == 3.0
    lapse = box["lapses"][0]
    assert lapse["why"].startswith("unseen") and 5.8 < lapse["t"] <= 6.0, lapse
    back = [t for t, d, _, _ in frames if d and t >= 6.4]
    assert back and back[0] <= 7.2, frames[30:]


def test_a_perception_reset_drops_the_subject_everywhere_it_is_held():
    """Review of d6e7123, P2-4 (the stuck reset, and P2-8's close stall,
    which shares it): with identity, lock.reset() left the strict lock empty
    while _acquire_step still believed it was committed - nothing could take
    the subject back for ~26 s. _perception_reset now starts the
    re-acquisition directly (begin_reacquire, the Reacquirer unanchored:
    the box was probably the wrong object, so its place is no anchor;
    committed False), and the hint ignores every box captured up to the
    reset (forget(t_reset)). Without identity the TargetLock forgets its
    instance. Real Grounder state, strict lock and Reacquirer."""
    import follow_vlm as FV
    from vla_bridge import VLABridge
    g = _strict_grounder()
    acq = FV.Reacquirer(need=3, max_range_m=30.0)
    box = vla_fly.new_acquisition(acq, t0=0.0)
    car = (384.0, 200.0, 40.0, 25.0, 0.3, 768, 432, 0.6)
    g.commit(car, yaw=0.0)
    box["committed"] = True
    det, _ = _strict_inference(g, 10.0, [car], [{"P": (70.0, 0.0), "rng_h": 20.0}], 9.9)
    assert det is not None and g.lock.cx == 384.0 and not g.acquiring

    def latest_box():
        gw = g.latest()
        if gw.get("det") is None:
            return None, None, None, None
        return gw["det"], (gw.get("cap") or {}).get("t_capture"), 0.0, gw.get("t_det")
    hint = VM.DetectorHint(latest_box, VLABridge("a red car", hfov_deg=90.0), fresh_s=1.0,
                           memory_s=6.0, search_s=20.0, record_kind=True)
    assert hint(Capture(10.1, Pose(40.0, 0.0, 10.0, 0.0), JPG, JPG)).kind == "fresh"
    st = SimpleNamespace(x=41.234, y=-0.5)
    ev = vla_fly._perception_reset("stuck", 10.2, 7.5, g, hint, box, st)
    assert ev == {"t": 7.5, "why": "stuck", "x": 41.23, "y": -0.5, "reacquiring": True}, ev
    assert g.acquiring and g.latest()["det"] is None and g.lock.cx is None
    assert not box["committed"] and acq.anchor is None
    assert hint.box_ignored(9.9) and not hint.box_ignored(10.3)
    assert hint(Capture(10.4, Pose(40.0, 0.0, 10.0, 0.0), JPG, JPG)) is None   # no memory of it
    # Without identity: the plain TargetLock forgets its instance; the box
    # the Grounder still publishes is ignored by the hint.
    lock = FV.TargetLock(gate_frac=0.12)
    lock.select([car], 768, 0.0, 10.0)
    plain = SimpleNamespace(lock=lock)
    hint2 = VM.DetectorHint(lambda: (car, 9.9, 0.0, 10.0), VLABridge("a red car"), fresh_s=1.0,
                            memory_s=6.0, search_s=20.0, record_kind=True)
    ev = vla_fly._perception_reset("close_stall", 10.2, 7.5, plain, hint2,
                                   vla_fly.new_acquisition(None), st)
    assert ev["why"] == "close_stall" and ev.get("lock_reset") and lock.cx is None, ev
    assert hint2(Capture(10.4, Pose(40.0, 0.0, 10.0, 0.0), JPG, JPG)) is None


def test_the_follow_resets_read_what_flew():
    """vla_fly._follow_watch: StuckWatch reads the decision that FLEW
    (review of d6e7123, P3: it read res.decision.touched, which with the
    Shield off is the shadow's touch - an aircraft nothing pinned was
    "pinned" and its lock reset); CloseStall reads the row's close_box and
    the backend's request count (P2-8)."""
    def res(t, shadow, flown, close=False):
        return SimpleNamespace(row={"t": t, "close_box": close},
                               decision=SimpleNamespace(touched=shadow),
                               hud_decision=SimpleNamespace(touched=flown))
    st = SimpleNamespace(x=10.0, y=5.0)
    stuck = VM.StuckWatch()
    assert not any(vla_fly._follow_watch(res(k * 0.1, True, False), st, stuck, None, 3)
                   for k in range(100))                       # Shield off: the shadow touched
    stuck = VM.StuckWatch()
    got = [vla_fly._follow_watch(res(k * 0.1, True, True), st, stuck, None, 3) for k in range(60)]
    assert got.count("stuck") == 1, got
    close = VM.CloseStall(2.0)
    got = [vla_fly._follow_watch(res(k * 0.1, False, False, True), st, None, close, 3)
           for k in range(30)]
    assert got.count("close_stall") == 1 and got.index("close_stall") == 20, got
    assert vla_fly._follow_watch(res(0.0, True, True, True), st, None, None, 3) is None


def test_the_follow_records_the_perception_it_applied():
    """Review of d6e7123, P2-5 and P3: the gate is decided AFTER the identity
    setup (follow_perception): identity on - follow_vlm's Reacquirer, the
    strict lock; identity requested but failed to load - the DetectorHint's
    gate (it used to fly with none) and the plain lock, the failure named;
    --no-identity --no-lock --start-gate-n 0 - none of them. params.follow
    records exactly that (follow_params: configuration only), and the
    measured outcomes (acquisition, resets, close-hold ticks) are
    metrics.follow's (follow_measured), in mission seconds."""
    ap = vla_fly.build_arg_parser()
    a = vla_fly.parse_cli(ap, ["--mission", "follow"])
    on = vla_fly.follow_perception(a, True)
    assert on == {"lock": True, "lock_gate": 0.12,
                  "identity": {"requested": True, "on": True, "error": None},
                  "start_gate": {"kind": "identity_reacquirer", "need": 5,
                                 "max_range_m": 30.0}}, on
    failed = vla_fly.follow_perception(a, False, "KeyError: 'depth_camera'")
    assert failed["start_gate"] == {"kind": "detector_hint", "n": 5, "width_frac": 0.04}
    assert failed["identity"] == {"requested": True, "on": False,
                                  "error": "KeyError: 'depth_camera'"} and failed["lock"]
    bare = vla_fly.parse_cli(ap, ["--mission", "follow", "--no-identity", "--no-lock",
                                  "--start-gate-n", "0"])
    none = vla_fly.follow_perception(bare, False)
    assert none == {"lock": False, "lock_gate": None,
                    "identity": {"requested": False, "on": False, "error": None},
                    "start_gate": None}, none
    p = vla_fly.follow_params(a, failed)
    assert p["start_gate"] == failed["start_gate"] and p["lock_gate"] == 0.12
    assert p["identity"] == failed["identity"] and p["perception_applied"] is True
    assert p["lapse_s"] == vla_fly.LAPSE_S and p["close_stall_s"] == 8.0 and p["stuck_reset"]
    assert not {"acquisition", "stuck_resets", "close_hold_ticks"} & set(p), p
    assert vla_fly.follow_params(a, None)["perception_applied"] is False
    box = vla_fly.new_acquisition(SimpleNamespace(), t0=100.0)
    box.update(committed=True, commits=[1.2, 6.0], reacquires=1,
               lapses=[{"t": 5.0, "why": "lock: no held instance"}])
    resets = [{"t": 20.0, "why": "stuck"}, {"t": 40.0, "why": "close_stall"},
              {"t": 50.0, "why": "stuck"}]
    m = vla_fly.follow_measured(box, resets, SimpleNamespace(n_close_hold=17))
    assert m["acquisition"] == {"committed": True, "commits_s": [1.2, 6.0], "reacquires": 1,
                                "lapses": [{"t": 5.0, "why": "lock: no held instance"}],
                                "errors": []}, m
    assert m["stuck_resets"] == 2 and m["close_stalls"] == 1 and m["close_hold_ticks"] == 17
    assert m["perception_resets"] == resets
    assert vla_fly.follow_measured(vla_fly.new_acquisition(None), [], None)["acquisition"] is None


def test_an_identity_that_fails_to_load_is_named_and_flown_without():
    """vla_fly._identity_setup (review of d6e7123, P2-5's other half): any
    failure - here a robot config whose detector camera has no depth
    topic - comes back as (None, why), never an exception, so the flight
    goes on with the DetectorHint gate (follow_perception) and says why;
    with the depth topic it returns follow_vlm's identity configuration and
    subscribes the depth stream."""
    import follow_vlm as FV
    args = vla_fly.parse_cli(vla_fly.build_arg_parser(), ["--mission", "follow",
                                                         "--citymap", str(_tmp() / "occ.npz")])
    subs = []
    sim = SimpleNamespace(subscribe=lambda topic, cb: subs.append(topic))
    obs = SimpleNamespace(put_depth=lambda m: None)
    no_depth = SimpleNamespace(sensors={"FrontCamera": {"scene_camera": "det"}})
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        cfg, why = vla_fly._identity_setup(args, sim, no_depth, obs, FV)
    assert cfg is None and "depth_camera" in why and subs == [], (cfg, why)
    depth = SimpleNamespace(sensors={"FrontCamera": {"scene_camera": "det",
                                                     "depth_camera": "det_depth"}})
    with contextlib.redirect_stdout(io.StringIO()):
        cfg, why = vla_fly._identity_setup(args, sim, depth, obs, FV)
    assert why is None and subs == ["det_depth"], why
    assert cfg["hfov"] == FV.CAMERA_HFOV_DEG and len(cfg["hash"]) > 8 and cfg["street"] is None


def test_the_follow_flags_and_their_records_agree():
    """Review of d6e7123 (P3): --no-lock still got a lock when identity was
    on (identity works through the strict lock) while params said lock_gate
    None, and --start-gate-n 0 ("0 disables") still gated with identity
    (need = max(1, n)). Both are refused unless --no-identity; --lock-gate
    (a fraction of the width), --acquire-max-range-m and --close-stall-s are
    validated."""
    import contextlib
    import io
    ap = vla_fly.build_arg_parser()
    for bad in (["--mission", "follow", "--no-lock"],
                ["--mission", "follow", "--start-gate-n", "0"],
                ["--mission", "follow", "--lock-gate", "0"],
                ["--mission", "follow", "--lock-gate", "1.5"],
                ["--mission", "follow", "--lock-gate", "nan"],
                ["--mission", "follow", "--acquire-max-range-m", "0"],
                ["--mission", "follow", "--acquire-max-range-m", "inf"],
                ["--mission", "follow", "--close-stall-s", "-1"]):
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                vla_fly.parse_cli(ap, bad)
        except SystemExit as exc:
            assert exc.code == 2, bad
            continue
        raise AssertionError(f"accepted {bad}")
    a = vla_fly.parse_cli(ap, ["--mission", "follow", "--no-identity", "--no-lock",
                               "--start-gate-n", "0", "--close-stall-s", "0"])
    assert a.no_lock and a.start_gate_n == 0 and a.close_stall_s == 0.0
    # The other missions never read them: no refusal there.
    assert vla_fly.parse_cli(ap, ["--mission", "goto", "--no-lock"]).no_lock


def test_a_close_box_holds_the_follows_horizontal_motion_before_the_shield():
    """MissionLoop close_hold: a fresh close box zeroes the VLA action's
    horizontal velocity (its yaw rate and climb still fly) before the Shield;
    the row says close_hold, the count goes up; a far box flies as asked.

    Review of d6e7123 (P3): the row's `raw` is what the Shield judged (the
    held action), and the tracker's own command is kept as `pilot_raw`;
    `close_box` says the box was close whatever the tracker did; the
    detector is read ONCE per tick (the close check and the row's box were
    two reads, which could disagree), and the tick hands that box on
    (LoopTick.det, the recorder's); pilot_summary counts the held ticks
    apart from vla_step in its shares. P2-4: a box from before a perception
    reset (DetectorHint.forget) holds nothing."""
    from guardrail.replay import rail_shield
    from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S
    from vla_bridge import VLABridge
    shield = rail_shield(load_policy(POLICY), lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S)
    sim = _Sim()
    move = Action4D(vx=0.0, vy=2.0, vz_up=0.3, yaw_rate=0.1)
    box = {"v": None}
    reads = {"n": 0}

    def det_source():
        reads["n"] += 1
        return box["v"]
    hint = VM.DetectorHint(det_source, VLABridge("a red car", hfov_deg=90.0), fresh_s=1.0)
    mon = VM.MissionMonitor(VM.MonitorConfig(mission="follow", max_s=60.0, lost_timeout_s=90.0),
                            sim.clock())
    loop = VM.MissionLoop(_FixedBackend(move, "vla_step", "translate", 1), shield, mon,
                          mission="follow", det_source=det_source, detector_hint=hint,
                          clock=sim.clock, close_hold=VM.BoxClose(0.92, 0.25))
    low = (384.0, 380.0, 100.0, 80.0, 0.3, 768, 432, 0.5)
    far = (384.0, 200.0, 30.0, 20.0, 0.3, 768, 432, 0.5)
    box["v"] = (low, sim.clock() - 0.2, 0.0, sim.clock() - 0.1)
    res = loop.tick(sim.state())
    assert reads["n"] == 1, reads
    assert res.row["close_hold"] and res.row["raw"]["vy"] == 0.0 and res.row["raw"]["vx"] == 0.0
    assert res.row["raw"]["vz_up"] == 0.3 and res.row["raw"]["yaw_rate"] == 0.1
    assert res.row["pilot_raw"] == move.model_dump() and res.row["close_box"]
    assert res.det == low and res.row["det"]["cy"] == 380.0
    assert loop.n_close_hold == 1
    rows = [res.row]
    box["v"] = (far, sim.clock() - 0.2, 0.0, sim.clock() - 0.1)
    res = loop.tick(sim.state())
    assert reads["n"] == 2
    assert not res.row["close_hold"] and res.row["raw"]["vy"] == 2.0 and loop.n_close_hold == 1
    assert not res.row["close_box"] and res.row["pilot_raw"] == res.row["raw"]
    rows.append(res.row)
    # A perception reset at t: the close box captured before it holds nothing.
    sim.step(Action4D())
    box["v"] = (low, sim.clock() - 0.2, 0.0, sim.clock() - 0.1)
    hint.forget(sim.clock())
    res = loop.tick(sim.state())
    assert not res.row["close_hold"] and not res.row["close_box"] and res.row["raw"]["vy"] == 2.0
    rows.append(res.row)
    s = VM.pilot_summary({}, [], rows)
    assert s["ticks_close_held"] == 1 and s["close_held_tick_share"] == round(1 / 3, 4)
    assert s["vla_step_tick_share"] == round(2 / 3, 4), s
    # No close rule (an approach, the other rails): no such row keys, no
    # such summary keys - their records are unchanged.
    m = VM.MissionLoop(_FixedBackend(move, "vla_step", "translate", 1), shield,
                       VM.MissionMonitor(VM.MonitorConfig(mission="approach"), sim.clock()),
                       mission="approach", det_source=det_source, clock=sim.clock)
    r = m.tick(sim.state()).row
    assert not {"close_hold", "close_box", "pilot_raw", "hint_kind"} & set(r)
    assert "ticks_close_held" not in VM.pilot_summary({}, [], [r])


def test_follow_scores_read_the_level_car_truth_rows_only():
    """follow_scores reads each row's subject_dist_m (the level car's truth,
    for scoring only). Since the review of d6e7123 (P1-1) "scored" has
    follow_vlm's meaning (follow_vlm.follow_scored): the truth on at least
    FOLLOW_TRUTH_MIN_COVERAGE of the ticks - 3 of 4 here is not scored, though
    the share within 30 m of the ticks that have it is still reported."""
    rows = [{"t": 0.0, "x": 0, "y": 0}, {"t": 0.1, "x": 0, "y": 0, "subject_dist_m": 12.0},
            {"t": 0.2, "x": 0, "y": 0, "subject_dist_m": 45.0},
            {"t": 0.3, "x": 0, "y": 0, "subject_dist_m": 20.0}]
    s = VM.follow_scores(rows)
    assert not s["follow_scored"] and s["ticks_scored"] == 3 and s["ticks"] == 4
    assert s["truth_coverage"] == 0.75
    assert s["frac_within"] == s["frac_within_30m"] == round(2 / 3, 3) and s["within_m"] == 30.0
    assert (s["dist_min_m"], s["dist_median_m"], s["dist_max_m"]) == (12.0, 20.0, 45.0)
    assert s["dist_start_m"] == 12.0 and s["dist_end_m"] == 20.0
    assert VM.follow_scores(rows[1:])["follow_scored"] is True
    assert VM.follow_scores([{"t": 0.0}]) == {"follow_scored": False, "ticks_scored": 0,
                                               "ticks": 1, "truth_coverage": 0.0,
                                               "frac_within_30m": None, "after_acquire": None}
    assert VM.follow_scores([])["follow_scored"] is False


def test_follow_scores_also_score_from_the_first_acquisition_as_follow_vlm_does():
    """The hand-written reference (citylife_redcar_nfz_motion2_20261019,
    0.963 within 30 m) waited 196 s at its start gate and scored only the
    240 s after it; vla_fly scored from take-off, so every VLA follow's share
    carried the ~200 s the level car spends on the far side of its loop
    (vla_follow_ft_01_20261024: 0.121 from take-off, 0.158 from the first
    acquisition). Both are reported; the KPI key keeps the whole flight."""
    rows = [{"t": 0.0, "subject_dist_m": 150.0}, {"t": 10.0, "subject_dist_m": 140.0},
            {"t": 20.0, "subject_dist_m": 12.0}, {"t": 30.0, "subject_dist_m": 18.0},
            {"t": 40.0, "subject_dist_m": 45.0}]
    s = VM.follow_scores(rows, acquired_at_s=20.0)
    assert s["frac_within_30m"] == round(2 / 5, 3)
    a = s["after_acquire"]
    assert a["first_acquire_s"] == 20.0 and a["ticks_scored"] == 3
    assert a["frac_within_30m"] == round(2 / 3, 3)
    assert "follow_vlm" in a["basis"]
    assert VM.follow_scores(rows)["after_acquire"] is None            # never acquired
    assert VM.follow_scores(rows, acquired_at_s=None)["after_acquire"] is None
    assert VM.follow_scores(rows, acquired_at_s=50.0)["after_acquire"]["ticks_scored"] == 0
    assert VM.follow_scores(rows, acquired_at_s=50.0)["after_acquire"]["frac_within_30m"] is None
    top = VM.follow_kpi_fields(s)
    assert top["frac_within_30m"] == s["frac_within_30m"]
    assert top["frac_within_30m_after_acquire"] == a["frac_within_30m"]


def test_a_follows_records_say_which_hint_each_decision_came_from():
    """Review of d6e7123, P2-2: 57-74 % of the decisions of follows #6, #8
    and #9 came from the remembered sighting or the search turn, and every
    record called them "detector". With record_kind (the follow's
    DetectorHint), each request's vla_steps record says fresh / memory /
    search (`hint_kind`), as does vla_captures.jsonl; each row says the kind
    of the decision being flown and of the latest hint sent; pilot_summary
    counts them. The protocol is unchanged: every request still says
    hint_source "detector" and carries no kind. Closed loop through TCP: a
    box for 3 s, then none (memory 3 s, search 3 s, then no request)."""
    from collections import Counter
    from guardrail.replay import rail_shield
    from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S
    from guardrail.vla_backends import make_backend
    from vla_bridge import VLABridge
    policy = load_policy(POLICY)
    sim = _Sim()
    box = {"v": None}

    def latest():
        return box["v"] or (None, None, None, None)
    det_hint = VM.DetectorHint(latest, VLABridge("a red car", hfov_deg=90.0), fresh_s=1.0,
                               memory_s=3.0, search_s=3.0, record_kind=True)
    hint = VM.HintSource(det_hint, "detector")
    srv = F.FakeVLAServer(_with_model(lambda req: F.step_reply(req, 1.0, 0.0, 0.0), REAL_MODEL),
                          model=REAL_MODEL)
    srv.start()
    frames = VM.VLAFrames(SimpleNamespace())
    try:
        client = RemoteVLA("127.0.0.1", srv.port, response_timeout_s=2.0,
                           reconnect_backoff_s=0.5, strict_identity=True, clock=sim.clock)
        be = make_backend("aerovla", None, policy, get_capture=sim.capture, get_hint=hint,
                          client=client, object_text="a red car", cadence="pipelined",
                          tracker_config=TrackerConfig(), clock=sim.clock,
                          on_submit=frames.commit)
        shield = rail_shield(policy, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S)
        mon = VM.MissionMonitor(VM.MonitorConfig(mission="follow", max_s=60.0,
                                                 lost_timeout_s=90.0), sim.clock())
        loop = VM.MissionLoop(be, shield, mon, mission="follow", det_source=latest,
                              hint_source=hint, detector_hint=det_hint, clock=sim.clock,
                              close_hold=VM.BoxClose(0.92, 0.25))
        far = (384.0, 200.0, 30.0, 20.0, 0.3, 768, 432, 0.5)
        rows = []
        while sim.t < 9.0:
            box["v"] = ((far, sim.t - 0.1, sim.yaw, sim.t - 0.05) if sim.t < 3.0 else None)
            res = loop.tick(sim.state())
            rows.append(res.row)
            be.client.wait_idle(3.0)
            sim.step(res.flown)
        be.close()
    finally:
        srv.stop()
    recs = [r for r in be.records if r.get("kind") == "step"]
    kinds = Counter(r.get("hint_kind") for r in recs)
    assert {"fresh", "memory", "search"} <= set(kinds) and None not in kinds, kinds
    assert all(r["hint_source"] == "detector" for r in recs)
    assert srv.requests and all(q["hint_source"] == "detector" and "hint_kind" not in q
                                for q in srv.requests)
    by_seq = {r["seq"]: r for r in be.records}
    flown = [r for r in rows if r["vla_seq"] is not None]
    assert flown and all(r["hint_kind"] == by_seq[r["vla_seq"]].get("hint_kind") for r in flown)
    assert {r["hint_sent_kind"] for r in rows} >= {"fresh", "memory", "search"}
    s = VM.pilot_summary(be.stats(), list(be.records), rows)
    assert s["hint_kind_counts"]["memory"] == kinds["memory"], s["hint_kind_counts"]
    assert {c.get("hint_kind") for c in frames.log} >= {"fresh", "memory", "search"}
    # Without record_kind (an approach; the ArduPilot rail's DetectorHint) no
    # kind anywhere.
    plain = VM.DetectorHint(latest, VLABridge("a red car", hfov_deg=90.0), fresh_s=1.0)
    box["v"] = (far, 99.9, 0.0, 99.95)
    h = plain(Capture(100.0, Pose(0.0, 0.0, 10.0, 0.0), JPG, JPG))
    assert h is not None and h.kind is None
    assert "hint_kind_counts" not in VM.pilot_summary({}, [{"kind": "step"}], [])


def test_a_land_held_as_arrival_is_not_recorded_as_a_landing_the_vla_asked_for():
    """Review of d6e7123, P2-6: follows #6 and #9 ended on time, yet their
    land record said requested_by_vla true with no time - the model's LAND
    that land_as_arrival HELD read as a landing. Now land_record says no
    landing was requested and counts the held ones (land_as_arrival, held
    true, the first one's seq and time); pilot_summary's land_accepted counts
    only LANDs that landed, land_as_arrival the held ones, and the hold
    ticks (reason land_as_arrival) stay VLA-authored: the model chose to
    stop. A real LAND (goto) is recorded exactly as before."""
    held = {"seq": 7, "kind": "land", "accepted": True, "land_as_arrival": True,
            "phrase": "straight ahead ", "hint_source": "detector",
            "pose_capture": {"x": 50.0, "y": 0.0, "up": 10.0, "yaw_rad": 0.0}}
    rows = [{"t": 0.0, "author": "vla_step", "author_reason": "translate", "land_requested": False},
            {"t": 0.1, "author": "hold", "author_reason": "land_as_arrival", "vla_seq": 7,
             "land_requested": False},
            {"t": 0.2, "author": "hold", "author_reason": "land_as_arrival", "vla_seq": 7,
             "land_requested": False},
            {"t": 0.3, "author": "hover", "author_reason": "no_new_decision",
             "land_requested": False}]
    lr = VM.land_record(rows, [held])
    assert lr["requested_by_vla"] is False, lr
    a = lr["land_as_arrival"]
    assert a["count"] == 1 and a["held"] is True and a["first_seq"] == 7 and a["first_t_s"] == 0.1
    s = VM.pilot_summary({}, [held], rows)
    assert s["land"] == 1 and s["land_accepted"] == 0 and s["land_as_arrival"] == 1, s
    assert s["vla_authored_tick_share"] == 0.75, s
    real = dict(held, land_as_arrival=None)
    real.pop("land_as_arrival")
    lr = VM.land_record([{"t": 4.2, "land_requested": True}], [real], (50.0, 3.0))
    assert lr == {"requested_by_vla": True, "seq": 7, "phrase": "straight ahead ",
                  "hint_source": "detector", "with_hint": True,
                  "pose_capture": real["pose_capture"], "dist_to_goal_m": 3.0, "t_s": 4.2}, lr
    s = VM.pilot_summary({}, [real], [])
    assert s["land_accepted"] == 1 and "land_as_arrival" not in s


def test_a_follow_whose_gate_never_opened_is_not_a_vla_failure():
    """Review of d6e7123, P2-7: a follow whose start gate never let a request
    out (or whose box was close from the start) ended no_first_decision,
    vla_failure true, exit 3 - blaming a model nothing was asked. With no
    request ever sent it ends `subject_not_acquired`, not a VLA failure;
    a request sent and never answered is still no_first_decision; goto and
    approach keep their rule."""
    cfg = VM.MonitorConfig(mission="follow", max_s=500.0, lost_timeout_s=300.0,
                           first_decision_timeout_s=10.0)
    box = (384.0, 200.0, 30.0, 20.0, 0.3, 768, 432, 0.5)
    m = VM.MissionMonitor(cfg, 0.0)
    nd = _tick("none", "no_decision", None, None)
    for k in range(99):
        assert m.update(0.1 * k, FAR, nd, det=box, det_age_s=0.2, requests_sent=0) is None
    assert m.update(10.0, FAR, nd, det=box, det_age_s=0.2, requests_sent=0) == "subject_not_acquired"
    s = m.summary()
    assert not s["vla_failure"] and s["requests_sent"] == 0 and "subject_not_acquired" in VM.END_REASONS
    assert "subject_not_acquired" not in VM.VLA_FAILURE_ENDS
    m = VM.MissionMonitor(cfg, 0.0)
    assert m.update(10.0, FAR, nd, det=box, det_age_s=0.2, requests_sent=2) == "no_first_decision"
    m = VM.MissionMonitor(cfg, 0.0)                    # a caller that cannot say: as before
    assert m.update(10.0, FAR, nd, det=box, det_age_s=0.2) == "no_first_decision"
    g = VM.MissionMonitor(VM.MonitorConfig(mission="goto", first_decision_timeout_s=10.0), 0.0)
    assert g.update(10.0, FAR, nd, goal_xy=(500.0, 500.0), requests_sent=0) == "no_first_decision"
    assert "requests_sent" not in VM.MissionMonitor(cfg, 0.0).summary()
    # MissionLoop reads the count from the backend (RemoteVLABackend.counts).
    from guardrail.replay import rail_shield
    from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S
    shield = rail_shield(load_policy(POLICY), lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S)
    sim = _Sim()
    mon = VM.MissionMonitor(cfg, sim.clock())
    loop = VM.MissionLoop(_FixedBackend(requests=0), shield, mon, mission="follow",
                          det_source=lambda: (box, sim.clock() - 0.2), clock=sim.clock)
    end = None
    while end is None:
        end = loop.tick(sim.state()).end_reason
        sim.step(Action4D())
    assert end == "subject_not_acquired" and sim.t <= 10.2, (end, sim.t)


class _FixedBackend:
    """A backend whose every act() is the same tick (MissionLoop needs act,
    last_tick, tracker and last_link_failure; `counts` like RemoteVLABackend's
    when `requests` is given)."""

    def __init__(self, action=None, author="none", reason="no_decision", seq=None,
                 requests=None):
        self.tracker = None
        self.last_link_failure = None
        self.tick_ = Tick(action or Action4D(), author, reason, seq, None if seq is None else 0.2)
        self.last_tick = None
        if requests is not None:
            from collections import Counter
            self.counts = Counter(requests=requests) if requests else Counter()

    def act(self, state):
        self.last_tick = self.tick_
        return self.tick_.action


def _follow_rows(dist_of, n=60, box=(384.0, 200.0, 30.0, 20.0, 0.3, 768, 432, 0.5)):
    """`n` rows of a follow through the real MissionLoop and the real Shield
    over the CityLife policy (a hovering backend at the mission start), each
    given subject_dist_m = dist_of(k) as vla_fly adds it after loop.tick
    (None: no truth on that tick)."""
    from guardrail.replay import rail_shield
    from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S
    shield = rail_shield(load_policy(POLICY), lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S)
    sim = _Sim()
    mon = VM.MissionMonitor(VM.MonitorConfig(mission="follow", max_s=600.0, lost_timeout_s=90.0),
                            sim.clock())
    loop = VM.MissionLoop(_FixedBackend(), shield, mon, mission="follow",
                          det_source=lambda: (box, sim.clock() - 0.2), clock=sim.clock)
    rows = []
    for k in range(n):
        res = loop.tick(sim.state())
        d = dist_of(k)
        if d is not None:
            res.row["subject_xy"] = [50.0, -18.0]
            res.row["subject_dist_m"] = d
        rows.append(res.row)
        sim.step(res.flown)
    return rows


def test_a_follow_publishes_its_score_where_kpi_and_the_dashboard_read_it():
    """Review of d6e7123, P1-1: vla_fly put the follow's score under
    metrics["follow"] only; guardrail/kpi.py reads frac_within_30m and
    follow_scored at the TOP of metrics (kpi.compute), found neither and
    applied no follow test, so all nine VLA follows of 23 Oct wrote kpi.json
    with mission_success true - #2 and #3 with 0.0 of their ticks within
    30 m. follow_kpi_fields lifts the two, with follow_vlm's meaning: a
    subject kept beyond 30 m fails kpi's own 5 % rule (follow_proxy), one kept
    close succeeds, and truth on too few ticks is not scored (mission_success
    None, follow_not_scored). tools/build_dashboard.py reads the same key."""
    import follow_vlm as FV
    import build_dashboard as BD
    assert VM.FOLLOW_TRUTH_MIN_COVERAGE == FV.FOLLOW_TRUTH_MIN_COVERAGE
    policy = load_policy(POLICY)
    manifest = VM.manifest_for(policy.policy_hash, REAL_MODEL, 7, CFG / "scene_vla_det.jsonc")
    cases = (("near", lambda k: 12.0, True, None),
             ("far", lambda k: 45.0, False, "follow_proxy"),
             ("thin", lambda k: 12.0 if k % 10 else None, None, "follow_not_scored"))
    for name, dist_of, want, reason in cases:
        rows = _follow_rows(dist_of)
        fs = VM.follow_scores(rows)
        top = VM.follow_kpi_fields(fs)
        # + follow_vlm's basis (from the first acquisition), not a KPI key
        assert set(top) == {"frac_within_30m", "follow_scored",
                            "frac_within_30m_after_acquire"}, top
        metrics = {"tag": f"vla_follow_{name}", "shield": "on", "nfz_s": 0.0,
                   "alt_violation_s": 0.0, "end_reason": "timeout", "aborted": None,
                   "mission": {"kind": "follow"}, **top, "follow": fs}
        out = _tmp() / f"vla_follow_{name}"
        out.mkdir()
        k = VM.finish_run(out, rows, metrics, manifest, policy)
        assert k["mission_success"] is want, (name, k["mission_success"], k["mission_fail_reasons"])
        if reason is None:
            assert k["mission_fail_reasons"] == [], (name, k["mission_fail_reasons"])
        else:
            assert reason in k["mission_fail_reasons"], (name, k["mission_fail_reasons"])
        assert k["mission_goal_status"] == "undeclared"
        stored = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
        assert stored["frac_within_30m"] == fs["frac_within_30m"]
        assert stored["follow_scored"] is fs["follow_scored"]
        assert BD.flight_record(ROOT, out)["frac_within_30m"] == fs["frac_within_30m"], name
    # The share kpi reads is the one follow_vlm writes: within 30 m over the
    # ticks with truth, rounded as follow_vlm rounds it.
    assert VM.follow_kpi_fields(VM.follow_scores(_follow_rows(lambda k: 12.0 if k < 15 else 45.0,
                                                              n=20))) == {
        "frac_within_30m": 0.75, "follow_scored": True, "frac_within_30m_after_acquire": None}


def _flown_follow_as_d6e7123_recorded_it(name: str, dist_of) -> tuple[Path, list[dict]]:
    """A follow run folder written the way vla_fly wrote it at d6e7123: the
    follow's score only under metrics["follow"] (its old keys), the policy
    source a YAML path, kpi.json from VM.finish_run over those metrics."""
    policy = load_policy(POLICY)
    rows = _follow_rows(dist_of)
    d = [r["subject_dist_m"] for r in rows if r.get("subject_dist_m") is not None]
    old_follow = {"follow_scored": bool(d), "ticks_scored": len(d), "ticks": len(rows),
                  "within_m": 30.0, "frac_within": round(sum(x <= 30 for x in d) / len(d), 3),
                  "level_car": "Car_10"}
    out = _tmp() / name
    out.mkdir()
    metrics = {"tag": name, "shield": "on", "nfz_s": 0.0, "alt_violation_s": 0.0,
               "end_reason": "timeout", "aborted": None, "interrupted": False,
               "mission": {"kind": "follow", "level_car": "Car_10"}, "reached_goal": None,
               "follow": old_follow, "params": {"cadence": "pipelined"},
               "policy_source": {"kind": "yaml", "path": str(POLICY),
                                 "loaded_policy_hash": policy.policy_hash,
                                 "signature": "unsigned"}}
    manifest = VM.manifest_for(policy.policy_hash, REAL_MODEL, 7, CFG / "scene_vla_det.jsonc")
    VM.write_jsonl(out / "flight_log.jsonl", rows)
    VM.finish_run(out, rows, metrics, manifest, policy)
    return out, rows


def test_the_rescore_tool_gives_a_flown_follow_the_kpi_it_should_have_had():
    """tools/rescore_vla_follow.py: the nine follows flown on 23 Oct carry
    the P1-1 records (kpi.json mission_success true for a subject 45 m away).
    The tool re-reads each run's flight_log.jsonl, publishes the follow's
    score at the top of metrics.json (follow_vlm's meaning), recomputes
    kpi.json against the run's own policy (its hash checked) and refuses to
    write unless the stored P0 figures come back unchanged; the superseded
    values are kept under `rescore`. A dry run writes nothing; a second run
    changes nothing; a run that is not a follow is refused."""
    import rescore_vla_follow as RS
    out, rows = _flown_follow_as_d6e7123_recorded_it("vla_follow_redcar_far", lambda k: 45.0)
    k_old = json.loads((out / "kpi.json").read_text(encoding="utf-8"))
    assert k_old["mission_success"] is True                   # the record P1-1 is about
    before = {p.name: p.read_bytes() for p in out.iterdir() if p.is_file()}
    rep = RS.rescore(out, write=False, rebundle=False)
    assert rep["status"] == "would_change" and rep["kpi"]["mission_success"] == [True, False], rep
    assert {p.name: p.read_bytes() for p in out.iterdir() if p.is_file()} == before
    rep = RS.rescore(out, rebundle=False)
    assert rep["status"] == "rescored", rep
    k = json.loads((out / "kpi.json").read_text(encoding="utf-8"))
    m = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    assert k["mission_success"] is False and "follow_proxy" in k["mission_fail_reasons"]
    assert m["frac_within_30m"] == 0.0 and m["follow_scored"] is True
    assert m["follow"]["frac_within_30m"] == 0.0 and m["follow"]["truth_coverage"] == 1.0
    assert m["follow"]["level_car"] == "Car_10"               # what the flight recorded stays
    for f in ("p0_violation_escape_rate", "p0_escapes", "p0_violation_ticks"):
        assert k[f] == k_old[f], f
    sup = m["rescore"]
    assert sup["tool"] == "tools/rescore_vla_follow.py" and "P1-1" in sup["why"]
    assert sup["superseded"]["kpi"]["mission_success"] is True
    assert sup["superseded"]["metrics"]["frac_within_30m"] is None
    assert k["rescore"]["superseded"]["mission_success"] is True
    assert b"\r\n" not in (out / "metrics.json").read_bytes()
    assert RS.rescore(out, rebundle=False)["status"] == "unchanged"
    near, _ = _flown_follow_as_d6e7123_recorded_it("vla_follow_redcar_near", lambda k: 12.0)
    rep = RS.rescore(near, rebundle=False)
    assert rep["status"] == "rescored" and rep["kpi"]["mission_success"] == [True, True], rep
    goto, _ = _flown_follow_as_d6e7123_recorded_it("vla_goto_x", lambda k: 12.0)
    m = json.loads((goto / "metrics.json").read_text(encoding="utf-8"))
    m["mission"]["kind"] = "goto"
    VM.write_json(goto / "metrics.json", m)
    assert RS.rescore(goto, rebundle=False)["status"] == "refused"
    # Another policy than the run's: refused, nothing written.
    other = ROOT / "policies" / "follow_car_citylife.yaml"
    before = (near / "kpi.json").read_bytes()
    assert RS.rescore(near, policy_path=other, rebundle=False)["status"] == "refused"
    assert (near / "kpi.json").read_bytes() == before
    # The replay bundle is re-written over the re-scored records and still
    # re-derives them; the as-flown bundle is kept beside it.
    from guardrail.replay import write_replay
    b, _ = _flown_follow_as_d6e7123_recorded_it("vla_follow_redcar_b", lambda k: 45.0)
    write_replay(b, load_policy(POLICY), b / "vla_follow_redcar_b.replay.tar.gz",
                 changelog="VLA flight vla_follow_redcar_b")
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()) as said:
        assert RS.main([str(b), "--dry-run"]) == 0
        assert RS.main([str(goto)]) == 1                         # refused: exit 1
    assert '"would_change"' in said.getvalue() and '"refused"' in said.getvalue()
    rep = RS.rescore(b)
    assert rep["status"] == "rescored" and rep["replay"]["verified"], rep
    assert rep["replay"]["as_flown_kept"] == "vla_follow_redcar_b.replay.tar.gz.pre-rescore"
    assert (b / "vla_follow_redcar_b.replay.tar.gz.pre-rescore").is_file()


def test_the_follow_cli_defaults_to_the_red_car_of_the_city_follow():
    ap = vla_fly.build_arg_parser()
    a = vla_fly.parse_cli(ap, ["--mission", "follow"])
    assert a.object == "a red car" and a.level_car == "Car_10" and a.park_at is None
    assert a.lost_timeout_s == 90.0 and a.start_heading == "street"
    assert a.monitor.mission == "follow" and a.monitor.lost_timeout_s == 90.0
    assert vla_fly.SCENES["follow"] == vla_fly.SCENES["approach"]      # the detector camera
    assert a.follow_executor and (a.follow_memory_s, a.follow_search_s) == (6.0, 20.0)
    assert not vla_fly.parse_cli(ap, ["--mission", "follow", "--no-follow-executor"]).follow_executor
    assert not vla_fly.parse_cli(ap, ["--mission", "goto"]).follow_executor
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
    # -ExtraArgs (d6e7123; review test gap): handed on as given, after the
    # rest, and the flight's parser reads them.
    r, got = _launch("-Mission follow -Tag t7 -ExtraArgs '--follow-memory-s','4',"
                     "'--close-stall-s','5','--no-stuck-reset'")
    assert r.returncode == 0, r.stderr[-1500:]
    assert got[-5:] == ["--follow-memory-s", "4", "--close-stall-s", "5", "--no-stuck-reset"], got
    a = vla_fly.parse_cli(vla_fly.build_arg_parser(), got[1:] + ["--dev-yaml"])
    assert a.mission == "follow" and a.follow_memory_s == 4.0 and a.close_stall_s == 5.0
    assert a.no_stuck_reset and a.tag == "t7"
    assert "passed to demo\\vla_fly.py" in src and "demola_fly" not in src
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
