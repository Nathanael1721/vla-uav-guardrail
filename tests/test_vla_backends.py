"""guardrail/vla_backends.py: the swappable slot's factory and RemoteVLABackend.

Run either way:
    pytest tests/test_vla_backends.py -v
    python tests/test_vla_backends.py

WHAT IS BEING CLAIMED

`make_backend("aerovla", ...)` puts AerialVLA in the slot: RemoteVLABackend,
which reaches the out-of-process VLA server over protocol v2
(guardrail/vla_link.py) and never waits for it. act() returns the step
tracker's world-frame action on every tick, whatever the server is doing; the
next image goes out while the current step is flown (pipelined) or after it
is complete (stop-and-go, upstream's cadence); LAND is a flag that stops the
requests; an answer to an image from before a rail hold ended is thrown away;
every request leaves one vla_steps.jsonl line.

CHANGED BY DESIGN ON 2026-10-21. Until then this file tested the skeleton
AeroVLABackend: that it raised NotImplementedError on its first model tick
(test_the_unfilled_skeleton_raises_instead_of_hovering), and its synchronous
"run the model on ticks 1, 1+N, ..." cadence (test_the_model_runs_on_ticks_1_
1_plus_n_and_holds_in_between, test_the_factory_forwards_the_rate). Both went
with the skeleton: the cadence called the model INSIDE act(), which at 2.5-12 s
per AerialVLA answer would have stalled the 10 Hz loop, and a held action is
replaced by the step tracker. The rule those tests guarded - a backend in the
slot must never hover on zero commands while the run looks like a VLA run -
is now carried by Tick.author: a tick no VLA answer authored says `none` or
`hover`, never `vla_step` (tests/test_vla_link.py).

The review fixes of 2026-10-22 (tests citing "Review of 7c7cc50"): a server
that refuses every request never looks like a pilot (no hold ticks, backoff,
decision rate 0); the decision rate counts only parsed answers; a full disk
or a raising callback never stops the loop; the goal phrase needs an explicit
hint="goal"; a policy never widens or drops a configured altitude band and
skips MSL envelopes. At 7c7cc50, with only the new `hint` keyword tolerated,
each of the six fails on behaviour: 299 hold ticks from a refusing server;
decision_hz 9.8 from malformed answers alone; ENOSPC and the ValueError of a
bad hint raised out of act(); the goal phrase chosen with no hint source; the
policy band (6, 14) replacing a configured (8, 30).

The second review (tests citing "Review of the V1b fixes (39c447a)"): the
drain guard is tested, and a reply that cannot be handed to the tracker keeps
its request's line; stats() says None, not 0, for the log counts of a run
with no log, and `log_pending` says whether `log_errors` is final. At 39c447a
both fail: the request's line is missing, and log_errors reads 0 with no log.
Each half of each fix was also reverted alone (the guard narrowed to
ZeroDivisionError, the line not kept, no link_lost, a reply to a seq never
sent counted as an answer, 0 for no log, pending hidden, stats() waiting on
the writer): every one fails one of the two tests.

Shown failing first: at 7c9c9e6 make_backend("aerovla", ...) required
model_dir/get_frame and built the skeleton, so every test below fails there
(TypeError on get_capture, or NotImplementedError on the first act()).
No model, GPU or simulator is needed; the server is tests/vla_fake_server.py.
(tests/test_vla_backend_table.py is a different module: tools/vla_backend_table.py.)
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from guardrail.compiler import Mission                               # noqa: E402
from guardrail.models import Action4D, State, load_policy            # noqa: E402
from guardrail.vla_backends import RemoteVLABackend, make_backend    # noqa: E402
from guardrail.vla_link import (PHRASES, VLA_STEPS_SCHEMA, Capture, Hint,  # noqa: E402
                                Pose, RemoteVLA, TrackerConfig)
import vla_fake_server as F                                          # noqa: E402

SKIP = "SKIP"
JPG = b"\xff\xd8\xff\xe0fake-jpeg\xff\xd9"
GOAL, GOAL_UP = (25.0, 15.0), 12.0


def _mission(goal=GOAL):
    return Mission(task_text="fly to the marked point", target_x=goal[0],
                   target_y=goal[1], cruise_alt_m=GOAL_UP, speed_pref_mps=2.0)


class _Sim:
    """A point multirotor on a simulated clock (velocity lag 0.3 s)."""

    def __init__(self, x=0.0, y=0.0, up=8.0, yaw_deg=0.0):
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


def _backend(srv, sim, policy=None, backoff_s=0.0, **kw):
    client = RemoteVLA("127.0.0.1", srv.port, response_timeout_s=2.0,
                       reconnect_backoff_s=backoff_s, clock=sim.clock)
    if "get_hint" not in kw:
        kw["hint"] = "goal"
    return make_backend("aerovla", _mission(), policy, get_capture=sim.capture,
                        client=client, clock=sim.clock, **kw)


def test_aerovla_is_the_remote_backend_and_refuses_the_skeletons_arguments():
    """make_backend('aerovla') built the skeleton from model_dir/get_frame/
    vla_hz until 2026-10-21. Those arguments are refused by name now (the
    model loads in the server, frames come with their capture pose, the rate
    is the server's), and the backend takes the policy's altitude band and
    the mission's goal phrase."""
    pol = load_policy(ROOT / "policies" / "follow_car_citylife.yaml")
    sim = _Sim()
    be = make_backend("aerovla", _mission(), pol, get_capture=sim.capture, hint="goal",
                      client=SimpleNamespace(counts={}), object_text="a white van")
    assert isinstance(be, RemoteVLABackend)
    assert be.tracker.alt_band == (6.0, 14.0) and be.cadence == "pipelined"
    assert be.object_text == "a white van" and be.csp_mode == "off"
    h = be.get_hint(Capture(0.0, Pose(0.0, 0.0, 8.0, 0.0), JPG))
    assert h == Hint("forward-right ", "goal"), h      # bearing +31 deg
    assert be.get_hint(Capture(0.0, Pose(25.1, 15.1, 8.0, 0.0), JPG)) == Hint("", "goal")
    for old in ({"model_dir": "D:/models"}, {"get_frame": sim.capture}, {"vla_hz": 3.0}):
        try:
            make_backend("aerovla", _mission(), None, get_capture=sim.capture, **old)
        except TypeError as exc:
            assert "VLA server" in str(exc), exc
        else:
            raise AssertionError(f"make_backend accepted the skeleton's {old}")
    for bad in ({"instruction": "x"}, {"cadence": "fast"}, {"csp_mode": "on"}):
        try:
            make_backend("aerovla", _mission(), None, get_capture=sim.capture, hint="goal",
                         client=SimpleNamespace(counts={}), **bad)
        except (TypeError, ValueError):
            continue
        raise AssertionError(f"make_backend accepted {bad}")
    try:
        make_backend("aerovla", _mission(), None)
    except TypeError:
        pass
    else:
        raise AssertionError("no get_capture, and the factory did not say so")
    try:
        make_backend("openvla-in-process", _mission(), None)
    except ValueError:
        pass
    else:
        raise AssertionError("an unknown backend name was accepted")
    assert make_backend("stub", _mission(), None).act(State(x=0, y=0, up=12)).vx > 0


def test_act_never_waits_for_the_model():
    """The server takes 1.5 s per answer (the old GPU took 2.5-12 s). Every
    act() returns at once with an honest label: nothing has been decided, so
    `none` and zero; the step arrives on the first tick after the answer."""
    sim = _Sim()
    with F.FakeVLAServer(lambda req: F.Delay(1.5, F.step_reply(req, 4.0, 0.0, 0.0))) as srv:
        be = _backend(srv, sim)
        try:
            worst = 0.0
            for _ in range(5):
                t0 = time.monotonic()
                a = be.act(sim.state())
                worst = max(worst, time.monotonic() - t0)
                assert a == Action4D() and be.last_tick.author == "none"
                sim.step(a)
            assert worst < 0.25, f"act() blocked for {worst:.3f} s"
            assert len(srv.requests) <= 1 and be.client.busy
            assert be.client.wait_idle(5.0)
            a = be.act(sim.state())
            assert be.last_tick.author == "vla_step" and a.vx > 0.9, be.last_tick
            assert be.last_tick.seq == 1
        finally:
            be.close()


def _fly_goto(cadence: str, tmp: Path):
    sim = _Sim()
    log = tmp / f"vla_steps_{cadence}.jsonl"
    if log.exists():
        log.unlink()
    with F.FakeVLAServer(F.goto_responder(GOAL, GOAL_UP)) as srv:
        be = _backend(srv, sim, cadence=cadence, steps_log=log,
                      tracker_config=TrackerConfig(alt_band=(5.0, 30.0)))
        authors = []
        try:
            while sim.t < 120.0 and not be.land_requested:
                a = be.act(sim.state())
                authors.append(be.last_tick.author)
                be.client.wait_idle(3.0)      # the answer is back by the next tick
                sim.step(a)
            n_at_land = be.counts["requests"]
            assert n_at_land == len(srv.requests)
            for _ in range(20):               # LAND: the pilot stops asking
                a = be.act(sim.state())
                assert a == Action4D() and be.last_tick.land_requested
                sim.step(a)
            assert be.counts["requests"] == n_at_land, "requests continued after LAND"
            assert not be.client.busy
            stats = be.stats()
        finally:
            be.close()
        return sim, srv, be, log, authors, stats


def test_the_backend_flies_a_goto_through_the_link_and_lands():
    """End to end through TCP: the vehicle follows only act()'s actions; the
    fake server answers each image like a perfect model would (quantized to
    99 bins), from the request's pose and phrase alone; the mission's goal
    phrase turns "" within 0.5 m and the answer is LAND. Both cadences."""
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        for cadence in ("pipelined", "stop_and_go"):
            sim, srv, be, log, authors, stats = _fly_goto(cadence, tmp)
            assert be.land_requested, (cadence, sim.t, sim.x, sim.y)
            assert math.hypot(sim.x - GOAL[0], sim.y - GOAL[1]) < 0.8, (cadence, sim.x, sim.y)
            assert abs(sim.up - GOAL_UP) < 0.5, (cadence, sim.up)
            assert srv.refused == [] and srv.overlaps == 0 and srv.connections == 1
            assert set(authors) <= {"none", "vla_step", "hold"}, (cadence, set(authors))
            assert authors.count("vla_step") > len(authors) / 2, cadence
            for req in srv.requests:
                assert req["hint_source"] == "goal" and req["phrase"] in PHRASES
                assert "target" not in req and "goal" not in req
            assert srv.requests[-1]["phrase"] == ""
            raw = log.read_bytes()
            assert b"\r\n" not in raw
            recs = [json.loads(x) for x in raw.decode("utf-8").splitlines()]
            assert len(recs) == len(srv.requests), (cadence, len(recs), len(srv.requests))
            assert sorted(r["seq"] for r in recs) == list(range(1, len(recs) + 1))
            assert all(r["schema"] == VLA_STEPS_SCHEMA for r in recs)
            acc = [r for r in recs if r["accepted"]]
            assert acc and all(r["outcome"] is not None for r in acc)
            assert all(r["target"] is not None for r in acc if r["kind"] == "step")
            assert recs[-1]["kind"] == "land" and recs[-1]["outcome"]["end"] == "land"
            assert stats["decisions_accepted"] == len(acc) and stats["decision_hz"] > 0
            assert stats["identity"]["id"] == F.MODEL["id"]
            if cadence == "stop_and_go":
                assert all(r["outcome"]["completed"] for r in acc if r["kind"] == "step")
                assert len(recs) < 40, len(recs)
            else:
                assert any(not r["outcome"]["completed"] for r in acc if r["kind"] == "step")


def test_an_answer_to_an_image_from_before_a_rail_hold_is_thrown_away():
    """Step 1 is being flown and the image for step 2 went out at t=0.1 when
    the rail takes the aircraft at t=0.2 (an escalation hold) and hands it
    back at t=1.0. Step 1 ends with the hold; the answer to the t=0.1 image is
    thrown away although it arrives after the hold, the next image goes out
    at once, and the tracker hovers until that answer lands."""
    sim = _Sim()
    with F.FakeVLAServer(lambda req: F.Delay(0.3, F.step_reply(req, 4.0, 0.0, 0.0))) as srv:
        be = _backend(srv, sim)
        try:
            a = be.act(sim.state())                   # request 1, captured t=0
            be.client.wait_idle(3.0)
            sim.step(a)
            a = be.act(sim.state())                   # step 1 flown; request 2, t=0.1
            assert be.last_tick.author == "vla_step" and be.last_tick.seq == 1
            sim.step(a)
            be.suspend()                              # t=0.2
            assert be.records[-1]["seq"] == 1
            assert be.records[-1]["outcome"]["end"] == "suspended"
            while sim.t < 1.0:
                sim.step(Action4D())                  # the rail flies; act() not called
            be.client.wait_idle(3.0)
            be.resume()                               # t=1.0
            a = be.act(sim.state())
            assert a == Action4D() and be.last_tick.author == "hover", be.last_tick
            assert be.last_tick.reason == "awaiting_fresh_decision"
            assert be.records[-1]["seq"] == 2
            assert be.records[-1]["refused"] == "invalidated", be.records[-1]
            be.client.wait_idle(3.0)
            assert len(srv.requests) == 3 and srv.requests[2]["t_capture"] == 1.0
            sim.step(a)
            be.act(sim.state())
            assert be.last_tick.author == "vla_step" and be.last_tick.seq == 3
        finally:
            be.close()


def test_link_failures_and_missing_hints_are_counted_and_recorded():
    sim = _Sim()
    calls = {"n": 0}

    def respond(req):
        calls["n"] += 1
        return F.CLOSE if calls["n"] == 2 else F.step_reply(req, 4.0, 0.0, 0.0)
    with F.FakeVLAServer(respond) as srv:
        be = _backend(srv, sim)
        try:
            for _ in range(3):
                a = be.act(sim.state())
                be.client.wait_idle(3.0)
                sim.step(a)
            kinds = [r["kind"] for r in be.records]
            assert "link_failure" in kinds, kinds
            fail = next(r for r in be.records if r["kind"] == "link_failure")
            assert fail["refused"] == "link_failure" and fail["failure"]["kind"] == "link"
            assert be.tracker.counts["link_lost"] == 1
            assert be.last_tick.author in ("hover", "vla_step")
            assert srv.connections == 2
        finally:
            be.close()
    with F.FakeVLAServer(lambda req: F.step_reply(req, 1.0, 0.0, 0.0)) as srv:
        be = make_backend("aerovla", _mission(), None, get_capture=sim.capture,
                          get_hint=lambda cap: None, clock=sim.clock,
                          client=RemoteVLA("127.0.0.1", srv.port, clock=sim.clock))
        try:
            for _ in range(5):
                assert be.act(sim.state()) == Action4D()
            assert srv.requests == [] and be.counts["no_hint"] == 5
            assert be.last_tick.author == "none"
        finally:
            be.close()


def test_close_leaves_a_line_for_the_request_still_in_flight():
    """One line per request sent, even when the run ends mid-inference."""
    sim = _Sim()
    with F.FakeVLAServer(lambda req: F.Delay(2.0, F.step_reply(req, 1.0, 0.0, 0.0))) as srv:
        be = _backend(srv, sim)
        be.act(sim.state())
        assert be.counts["requests"] == 1 and be.client.busy
        t0 = time.monotonic()
        be.close()
        assert time.monotonic() - t0 < 1.5, "close() waited out the inference"
        assert len(be.records) == 1, be.records
        rec = be.records[0]
        assert rec["seq"] == 1 and rec["refused"] == "closed" and not rec["accepted"]


def test_a_hot_applied_policy_moves_the_altitude_band():
    sim = _Sim()
    be = make_backend("aerovla", _mission(), load_policy(ROOT / "policies" / "demo_policy.yaml"),
                      get_capture=sim.capture, hint="goal", client=SimpleNamespace(counts={}))
    assert be.tracker.alt_band == (2.0, 6.0)
    be.refresh_fences(load_policy(ROOT / "policies" / "follow_car_citylife.yaml"))
    assert be.tracker.alt_band == (6.0, 14.0)
    be.refresh_fences(load_policy(ROOT / "policies" / "corridor_survey.yaml"))
    assert be.tracker.alt_band is None


def test_a_configured_band_is_never_widened_or_dropped_by_the_policy():
    """Review of 7c7cc50, P8: a policy band silently REPLACED the caller's
    TrackerConfig.alt_band, and refresh_fences() with a policy that has no
    envelope removed a configured clamp. The tracker now clamps into the
    tighter of the two (stats() says which), a policy without an envelope
    leaves the configured band in force, and an MSL envelope is skipped and
    named. A configured band that misses the policy band entirely is refused
    at construction; on a hot-apply the policy band wins, said so."""
    sim = _Sim()
    kw = dict(get_capture=sim.capture, hint="goal", client=SimpleNamespace(counts={}))
    be = make_backend("aerovla", _mission(), load_policy(ROOT / "policies" / "follow_car_citylife.yaml"),
                      tracker_config=TrackerConfig(alt_band=(8.0, 30.0)), **kw)
    assert be.tracker.alt_band == (8.0, 14.0), be.tracker.alt_band
    ab = be.stats()["alt_band"]
    assert ab["band"] == (8.0, 14.0) and ab["source"] == "intersection", ab
    assert ab["configured"] == (8.0, 30.0) and ab["policy"] == (6.0, 14.0)
    be.refresh_fences(load_policy(ROOT / "policies" / "corridor_survey.yaml"))
    assert be.tracker.alt_band == (8.0, 30.0), "a policy without an envelope dropped the clamp"
    assert be.stats()["alt_band"]["source"] == "configured"
    be.refresh_fences(load_policy(ROOT / "policies" / "demo_policy.yaml"))      # 2-6 m
    assert be.tracker.alt_band == (2.0, 6.0)
    assert be.stats()["alt_band"]["source"] == "policy_disjoint"
    try:
        make_backend("aerovla", _mission(), load_policy(ROOT / "policies" / "demo_policy.yaml"),
                     tracker_config=TrackerConfig(alt_band=(8.0, 30.0)), **kw)
    except ValueError as exc:
        assert "overlap" in str(exc), exc
    else:
        raise AssertionError("a configured band outside the policy band was accepted")
    from guardrail.models import AltitudeEnvelope
    msl = SimpleNamespace(constraints=[AltitudeEnvelope(
        id="msl_ceiling", type="altitude_envelope", alt_min_m=120, alt_max_m=300,
        altitude_ref="MSL")])
    be = make_backend("aerovla", _mission(), msl, **kw)
    assert be.tracker.alt_band is None
    assert be.stats()["alt_band"]["skipped_msl"] == ["msl_ceiling"]


def test_the_goal_hint_is_only_for_an_explicit_goto():
    """Review of 7c7cc50, P7: without get_hint the factory defaulted to the
    phrase toward (mission.target_x, mission.target_y), labelled `goal`, for
    ANY mission. A follow or approach mission wired without its detector hint
    would silently get an oracle direction labelled as a GOTO goal. The brief:
    the hint comes from the goal only for GOTO, otherwise from the detector.
    The goal phrase is now used only when the caller says hint='goal'."""
    sim = _Sim()
    try:
        make_backend("aerovla", _mission(), None, get_capture=sim.capture,
                     client=SimpleNamespace(counts={}))
    except TypeError as exc:
        assert "vla_bridge" in str(exc) and "hint='goal'" in str(exc), exc
    else:
        raise AssertionError("no hint source, and the factory chose the goal phrase")
    for bad in ({"hint": "detector"}, {"hint": "goal", "get_hint": lambda cap: None}):
        try:
            make_backend("aerovla", _mission(), None, get_capture=sim.capture,
                         client=SimpleNamespace(counts={}), **bad)
        except (TypeError, ValueError):
            continue
        raise AssertionError(f"make_backend accepted {bad}")
    be = make_backend("aerovla", _mission(), None, get_capture=sim.capture, hint="goal",
                      client=SimpleNamespace(counts={}))
    assert be.get_hint(Capture(0.0, Pose(0.0, 0.0, 8.0, 0.0), JPG)) == Hint("forward-right ", "goal")
    det = make_backend("aerovla", _mission(), None, get_capture=sim.capture,
                       get_hint=lambda cap: Hint("to your left ", "detector"),
                       client=SimpleNamespace(counts={}))
    assert det.get_hint(None).source == "detector"


def _fly_ticks(be, sim, n):
    authors = []
    for _ in range(n):
        a = be.act(sim.state())
        assert a == be.last_tick.action
        authors.append(be.last_tick.author)
        be.client.wait_idle(2.0)
        sim.step(a)
    return authors


def test_a_server_that_refuses_every_request_never_looks_like_a_pilot():
    """Review of 7c7cc50, P1, reproduced there: a server answering every
    request with ok=false 'CUDA out of memory' for 30 s gave 299 `hold` ticks
    and 0 `hover`, 300 requests (one per tick: no backoff), and stats() said
    decisions_accepted 299, decision_hz 9.97. A dead model reported a 10 Hz
    VLA decision rate and VLA-authored holds; the first V1a/V1b field
    mismatch (the server refusing 'unknown field' on every request) would
    have looked exactly like that. Now: no tick is `hold` or `vla_step`, the
    requests are paced by the 1 s backoff, decision_hz is 0 and the refusals
    are counted and rated on their own."""
    from collections import Counter
    sim = _Sim()
    with F.FakeVLAServer(lambda req: F.error_reply(req, "CUDA out of memory")) as srv:
        be = _backend(srv, sim, backoff_s=1.0)
        try:
            authors = _fly_ticks(be, sim, 300)            # 30 s of sim time
            st = be.stats()
        finally:
            be.close()
    assert set(authors) == {"none"}, Counter(authors)
    assert be.last_tick.reason == "server_error", be.last_tick
    n = len(srv.requests)
    assert 25 <= n <= 31, f"{n} requests in 30 s with a 1 s backoff"
    assert be.counts["requests"] == n and be.client.counts["server_error"] == n
    assert st["decisions_accepted"] == 0 and not st["decision_hz"], st
    assert st["server_error"] == n and 0.8 < st["server_error_hz"] < 1.1, st
    assert st["answers"] == {"server_error": n}, st["answers"]
    assert len(be.records) == n
    assert all(r["kind"] == "server_error" and r["accepted"] is False
               and r["refused"] == "server_error" for r in be.records)
    # A server that fails after one good answer: the step is dropped at the
    # first refusal and every later tick hovers; none is a hold.
    sim = _Sim()
    calls = {"n": 0}

    def respond(req):
        calls["n"] += 1
        if calls["n"] == 1:
            return F.step_reply(req, 4.0, 0.0, 0.0)
        return F.error_reply(req, "CUDA out of memory")
    with F.FakeVLAServer(respond) as srv:
        be = _backend(srv, sim, backoff_s=1.0)
        try:
            authors = _fly_ticks(be, sim, 100)
            st = be.stats()
        finally:
            be.close()
    assert authors[:2] == ["none", "vla_step"], authors[:5]
    assert set(authors[2:]) == {"hover"}, Counter(authors[2:])
    assert be.last_tick.reason == "server_error"
    assert st["decisions_accepted"] == 1 and st["server_error"] == len(srv.requests) - 1
    assert be.records[0]["seq"] == 1 and be.records[0]["outcome"]["end"] == "server_error"


def test_the_decision_rate_counts_only_answers_the_model_parsed():
    """Review of 7c7cc50, P1 (stats): decision_hz counted every accepted
    reply, malformed ones included. It is now the rate of parse_ok answers
    (a step or LAND) the tracker took; malformed answers keep the brief's
    counted zero hold and are rated on their own."""
    sim = _Sim()
    with F.FakeVLAServer(lambda req: F.parse_fail_reply(req)) as srv:
        be = _backend(srv, sim)
        try:
            authors = _fly_ticks(be, sim, 50)
            st = be.stats()
        finally:
            be.close()
    assert set(authors[1:]) == {"hold"} and be.last_tick.reason == "parse_fail"
    n = len(srv.requests) - 1                 # the last tick's answer was not read yet
    assert st["decisions_accepted"] == 0 and st["decision_hz"] == 0.0, st
    assert st["parse_fail"] == n and st["parse_fail_hz"] > 5.0, st
    assert st["answers"] == {"parse_fail": n} and st["server_error"] == 0


def test_a_full_disk_never_stops_the_loop():
    """Review of 7c7cc50, P4, reproduced there: with the log's write raising
    ENOSPC, act() raised '[Errno 28] No space left on device' at tick 2 and
    would have ended the flight loop. The log is now written off the loop
    thread; a failed write is counted (stats()['log_errors']) and the record
    stays in memory (`records`)."""
    import tempfile
    sim = _Sim()
    with tempfile.TemporaryDirectory() as d, \
            F.FakeVLAServer(lambda req: F.step_reply(req, 1.0, 0.0, 0.0)) as srv:
        be = _backend(srv, sim, steps_log=Path(d) / "vla_steps.jsonl")

        def full(s):
            raise OSError(28, "No space left on device")
        be.steps_log._fh.write = full
        try:
            authors = _fly_ticks(be, sim, 50)
            assert "vla_step" in authors
            assert be.steps_log.flush(3.0)
            st = be.stats()
        finally:
            be.close()
    assert be.records and st["log_errors"] >= 1, st
    # Every record was kept in memory, and every write of one failed, counted.
    assert be.steps_log.n == 0 and be.steps_log.errors == len(be.records)


def test_a_raising_callback_never_stops_the_loop():
    """Review of 7c7cc50, P6, reproduced there: an exception from the caller's
    get_capture or get_hint (Hint('', 'detector') raises), or from
    build_infer_fields, went out of act() and stopped the 10 Hz loop. act()
    now returns the tracker's action, counts the error and keeps the last
    one's text in stats(); the next good tick sends the request."""
    sim = _Sim()
    state = {"raise": True}

    def get_hint(cap):
        if state["raise"]:
            return Hint("", "detector")               # raises ValueError
        return Hint("straight ahead ", "detector")
    with F.FakeVLAServer(lambda req: F.step_reply(req, 4.0, 0.0, 0.0)) as srv:
        be = _backend(srv, sim, get_hint=get_hint)
        try:
            for _ in range(5):
                assert be.act(sim.state()) == Action4D()
                assert be.last_tick.author == "none"
            assert be.counts["request_errors"] == 5 and srv.requests == []
            st = be.stats()
            assert st["last_request_error"].startswith("ValueError"), st
            state["raise"] = False
            _fly_ticks(be, sim, 3)
            assert be.last_tick.author == "vla_step" and len(srv.requests) >= 1
        finally:
            be.close()

    def bad_capture():
        raise RuntimeError("camera gone")
    with F.FakeVLAServer(lambda req: F.step_reply(req, 4.0, 0.0, 0.0)) as srv:
        be = make_backend("aerovla", _mission(), None, get_capture=bad_capture, hint="goal",
                          client=RemoteVLA("127.0.0.1", srv.port, clock=sim.clock),
                          clock=sim.clock)
        try:
            assert be.act(sim.state()) == Action4D()
            assert be.counts["request_errors"] == 1
            assert "camera gone" in be.stats()["last_request_error"]
        finally:
            be.close()


def _one_line_per_request(be):
    seqs = sorted(r["seq"] for r in be.records)
    assert seqs == list(range(1, be.counts["requests"] + 1)), seqs


def test_a_reply_that_cannot_be_handed_to_the_tracker_is_a_lost_link():
    """Review of the V1b fixes (39c447a), P1: the drain guard had no test.
    With `except Exception` in RemoteVLABackend._safe_drain narrowed to
    `except ZeroDivisionError` (the guard gone), test_vla_link.py and this
    file still passed. A reply that cannot be handed to the tracker (here
    decision_from_result raising, and a reply to a seq never sent) must not
    leave act(): the tick returns the tracker's action, the error is counted
    (drain_errors, last_drain_error) and treated as a lost link, so the tick
    is `none`/link_lost before any decision and the step being flown is
    dropped (`hover`/link_lost) after one. The request whose reply could not
    be handed over keeps its vla_steps.jsonl line (refused `drain_error`):
    at 39c447a its meta was popped and the line lost, so a run with a drain
    error broke one line per request."""
    import guardrail.vla_link as L
    real = L.decision_from_result

    def broken(meta, res):
        raise RuntimeError("decoder bug")
    reply = lambda req: F.step_reply(req, 4.0, 0.0, 0.0)          # noqa: E731
    # Before any decision: the first reply cannot be handed over.
    sim = _Sim()
    with F.FakeVLAServer(reply) as srv:
        be = _backend(srv, sim)
        try:
            a = be.act(sim.state())
            be.client.wait_idle(3.0)
            sim.step(a)
            L.decision_from_result = broken
            try:
                a = be.act(sim.state())
            finally:
                L.decision_from_result = real
            assert a == Action4D() and a == be.last_tick.action
            assert (be.last_tick.author, be.last_tick.reason) == ("none", "link_lost"), be.last_tick
            assert be.counts["drain_errors"] == 1 and be.tracker.counts["link_lost"] == 1
            assert be.stats()["last_drain_error"] == "RuntimeError: decoder bug", be.stats()
            lost = [r for r in be.records if r["seq"] == 1]
            assert len(lost) == 1 and lost[0]["refused"] == "drain_error", be.records
            assert lost[0]["accepted"] is False and lost[0]["kind"] == "step"
            assert lost[0]["drain_error"] == "RuntimeError: decoder bug"
            # The loop goes on: the next reply is handed over and flown.
            be.client.wait_idle(3.0)
            sim.step(a)
            be.act(sim.state())
            assert be.last_tick.author == "vla_step" and be.last_tick.seq == 2
        finally:
            be.close()
    _one_line_per_request(be)
    # After a decision: the step being flown is dropped, and the tracker hovers.
    sim = _Sim()
    with F.FakeVLAServer(reply) as srv:
        be = _backend(srv, sim)
        try:
            assert _fly_ticks(be, sim, 2) == ["none", "vla_step"]
            L.decision_from_result = broken
            try:
                a = be.act(sim.state())
            finally:
                L.decision_from_result = real
            assert a == Action4D()
            assert (be.last_tick.author, be.last_tick.reason) == ("hover", "link_lost"), be.last_tick
            assert be.counts["drain_errors"] == 1 and be.tracker.counts["link_lost"] == 1
            flown = next(r for r in be.records if r["seq"] == 1)
            assert flown["accepted"] is True and flown["outcome"]["end"] == "link_lost", flown
            assert next(r for r in be.records if r["seq"] == 2)["refused"] == "drain_error"
        finally:
            be.close()
    _one_line_per_request(be)
    # A reply to a seq this backend never sent: no request is owed a line,
    # and it is no answer.
    sim = _Sim()
    late = [L.LinkResult(99, {}, None, 0.0, 0.0, "link", "never sent")]
    stub = SimpleNamespace(counts={}, busy=False, close=lambda: None,
                           poll=lambda: late.pop() if late else None)
    be = make_backend("aerovla", _mission(), None, get_capture=sim.capture,
                      get_hint=lambda cap: None, client=stub, clock=sim.clock)
    assert be.act(sim.state()) == Action4D()
    assert (be.last_tick.author, be.last_tick.reason) == ("none", "link_lost"), be.last_tick
    assert be.counts["drain_errors"] == 1 and be.tracker.counts["link_lost"] == 1
    st = be.stats()
    assert st["last_drain_error"] == "KeyError: 99", st
    assert st["answers"] == {} and be.records == [], (st["answers"], be.records)
    be.close()
    assert be.records == []


def test_the_log_counts_say_when_nothing_was_measured_or_is_still_queued():
    """Review of the V1b fixes (39c447a), P2: stats() said log_errors 0 when
    there was no log at all, the same as a log written without a fault; and
    with a log it read StepLog.errors without waiting for the writer thread,
    so before close() it could say 0 while failing writes were still queued.
    The zero that means 'not measured'. Now: no log is None for every log
    count, and `log_pending` (queued, not yet written or failed) says
    whether log_errors is final; stats() never waits for the writer."""
    import tempfile
    import threading
    sim = _Sim()
    kw = dict(get_capture=sim.capture, hint="goal", client=SimpleNamespace(counts={}))
    st = make_backend("aerovla", _mission(), None, **kw).stats()
    for k in ("log_errors", "last_log_error", "log_pending", "log_written"):
        assert k in st and st[k] is None, (k, st)
    with tempfile.TemporaryDirectory() as d:
        be = make_backend("aerovla", _mission(), None,
                          steps_log=Path(d) / "vla_steps.jsonl", **kw)
        gate = threading.Event()

        def stuck(s):
            gate.wait(5.0)
            raise OSError(28, "No space left on device")
        be.steps_log._fh.write = stuck
        try:
            be.steps_log.write({"seq": 1})
            be.steps_log.write({"seq": 2})
            t0 = time.monotonic()
            st = be.stats()
            assert time.monotonic() - t0 < 0.5, "stats() waited for the writer"
            assert st["log_errors"] == 0 and st["log_pending"] == 2, st
            assert st["log_written"] == 0 and st["last_log_error"] is None, st
            gate.set()
            assert be.steps_log.flush(3.0)
            st = be.stats()
            assert st["log_errors"] == 2 and st["log_pending"] == 0, st
            assert "No space left" in st["last_log_error"] and st["log_written"] == 0, st
        finally:
            gate.set()
            be.steps_log.close()


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
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
