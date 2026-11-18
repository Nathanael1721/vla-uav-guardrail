"""The body-frame action and its arc (WP3-04 / ARCH-04, 2026-11-17).

Run either way:
    pytest tests/test_body_arc_forecast.py -v
    python tests/test_body_arc_forecast.py

The grant's input frame is the BODY frame (vx forward, vy right, vz, yaw_rate),
with body -> local-NED conversion after the projection, in the MAVLink
adapter. A body-frame command holds its forward / right velocity while the
heading turns at its yaw rate, so it flies an ARC. Until this change the
Shield's forecast held the action's world direction from t = 0, so a zone
the arc reaches and the straight line misses was not seen.

Pinned here:
  * the arc forecast is the exact integral of the turning velocity, a
    subnormal or tiny yaw rate included (the line);
  * the arc ONLY ADDS to the straight forecast (review 2026-11-17): every
    straight pose is still judged, a yaw rate alternating each tick is
    repaired no later than on the line, and nothing the line breaks is lost;
  * inside Shield.filter_body(..., arc=True) a zone the arc reaches and the
    straight forecast misses is caught and repaired, and the repaired action
    re-checks clean on the arc;
  * an arced tick is on record (TickRecord.arc, the trace, the audit
    record), and an arced escape from a zone that opened on the aircraft is
    judged on the full line and arc (the 69c27bd behaviour, opt-in arm only);
  * a zero yaw rate, the arc off, and filter() (world frame) are forecast
    and decided BIT-IDENTICALLY to the straight forecast, with and without a
    measured velocity - so the stub pilot's world-frame KPI flights (yaw_rate
    0) and every rail that has not asked for the arc fly as before;
  * the PAS rail's links turn the Shield's emitted BODY action into local
    NED after the projection (`send(..., body=...)`), the PAS rail flies
    --action-frame body end to end on a fake ArduPilot, and the headless
    harness's body-frame arm flies the world-frame arm's rows to within the
    rotations' rounding;
  * a rail campaign can ask the ROS 2 node for the arc
    (--rail-arc-forecast on) and refuses a run whose metrics.json does not
    say it flew it.
"""
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail.frames import from_body, to_body           # noqa: E402
from guardrail.models import Action4D, Policy, State, load_policy  # noqa: E402
from guardrail.shield import ARC_MIN_RATE, Shield, _arc_offset      # noqa: E402

POLICY = ROOT / "policies" / "sim_demo_policy.yaml"


def _policy():
    return load_policy(POLICY)


# nfz-square is about x, y in [7, 23] m (ring [6, 24] with its 1 m margin).
# From (10, 0) heading North, body-forward 4 m/s (the speed cap) turning right
# at 0.4 rad/s (23 deg/s, under the 45 deg/s cap): the arc's centre is (10, 10),
# radius 10 m, and it is inside the ring from about t = 3 s. The straight line
# (North along y = 0) never comes within 6 m of the zone.
ST = State(x=10.0, y=0.0, up=15.0, yaw_deg=0.0)
TURN = Action4D(vx=4.0, vy=0.0, vz_up=0.0, yaw_rate=0.4)


def test_the_arc_offset_is_the_integral_of_the_turning_velocity():
    """_arc_offset against a fine Riemann sum of v(s) = R(w s) v0, at
    several headings, rates (both signs) and times."""
    rng = random.Random(20261117)
    # 40 ordinary rates, then rates where w * t is under 1e-3 (the series
    # branch) down to a subnormal one (review 2026-11-17: 5e-324 read
    # x = 0.0 m at 0.2 s).
    small = [5e-324, 1e-300, 1e-12, 3e-7, 1e-5, 1.9e-4]
    for k in range(40 + len(small)):
        vx, vy = rng.uniform(-5, 5), rng.uniform(-5, 5)
        w = rng.choice([-1, 1]) * (rng.uniform(0.05, 1.2) if k < 40 else small[k - 40])
        t = rng.uniform(0.1, 5.0)
        n = 20000
        h = t / n
        sx = sy = 0.0
        for k in range(n):
            th = w * (k + 0.5) * h
            sx += (vx * math.cos(th) - vy * math.sin(th)) * h
            sy += (vx * math.sin(th) + vy * math.cos(th)) * h
        ax, ay = _arc_offset(vx, vy, w, t)
        assert abs(ax - sx) < 1e-6 and abs(ay - sy) < 1e-6, (vx, vy, w, t, ax, sx, ay, sy)


def test_turning_right_from_north_drifts_east_on_a_circle():
    """Heading North, a right turn (w > 0, clockwise as State.yaw_deg) drifts
    East, and every arc pose is on the circle of radius v / w about the turn
    centre; the heading carried by each pose is the integrated one."""
    sh = Shield(_policy())
    poses = sh.forecast(ST, from_body(4.0, 0.0, 0.0, 0.4, 0.0), arc=True)
    cx, cy, r = 10.0, 10.0, 4.0 / 0.4
    assert len(poses) == 51
    for t, p in poses:
        assert abs(math.hypot(p.x - cx, p.y - cy) - r) < 1e-9, (t, p)
        assert abs(p.yaw_deg - math.degrees(0.4) * t) < 1e-9
    t, p = poses[-1]
    assert p.y > 5.0, p
    straight = sh.forecast(ST, from_body(4.0, 0.0, 0.0, 0.4, 0.0))
    assert all(q.y == 0.0 for _, q in straight), "the straight forecast turned"


def test_the_arc_catches_the_zone_the_straight_forecast_misses():
    """The task's case: filter_body without the arc (as before) passes the
    turning command clean; with it, nfz-square is found and repaired, and
    the repaired action re-checks clean on the arc forecast."""
    straight, em_s = Shield(_policy()).filter_body(ST, TURN)
    assert not straight.violations, straight.violations
    assert em_s == TURN
    sh = Shield(_policy())
    arc, em_a = sh.filter_body(ST, TURN, arc=True)
    ids = [v.rule_id for v in arc.violations]
    assert "nfz-square" in ids, ids
    assert arc.touched and em_a != TURN, (arc.repairs, em_a)
    assert not [v for v in arc.emitted_violations if v.rule_id == "nfz-square"], \
        arc.emitted_violations
    # The sliding window holds the ARC the checker judged.
    tr = sh.history[-1].trajectory
    assert any(p.y > 6.0 for _, p in tr), "the buffer's trajectory is straight"
    # The constructor's default does the same as arc=True.
    d2, _ = Shield(_policy(), arc_forecast=True).filter_body(ST, TURN)
    assert [v.rule_id for v in d2.violations] == ids


def test_filter_never_arcs_a_world_frame_action():
    """filter() takes a WORLD action, which a LOCAL_NED velocity setpoint
    flies straight while the aircraft yaws: it is never arced, even on a
    Shield built with arc_forecast=True."""
    raw = from_body(TURN.vx, TURN.vy, TURN.vz_up, TURN.yaw_rate, ST.yaw_deg)
    d = Shield(_policy(), arc_forecast=True).filter(ST, raw)
    assert not d.violations, d.violations


def _same(a, b):
    assert a.raw == b.raw and a.emitted == b.emitted, (a.raw, b.raw, a.emitted, b.emitted)
    assert a.model_dump() == b.model_dump()


def test_a_zero_yaw_rate_is_bit_identical_with_and_without_the_arc():
    """yaw_rate 0 (the stub pilot of every KPI campaign): the arc IS the
    line, and every decision, every forecast pose and the buffer's
    trajectory are the same bits with the arc on as off - and as filter()
    on the action turned by hand. Random states round nfz-square, with and
    without a measured velocity (the own-velocity branch)."""
    rng = random.Random(1117)
    for k in range(80):
        st = State(x=rng.uniform(-10, 35), y=rng.uniform(-10, 35),
                   up=rng.uniform(8, 22), yaw_deg=rng.uniform(-180, 180))
        if k % 2:
            st = st.model_copy(update={"velocity": (rng.uniform(-4, 4),
                                                    rng.uniform(-4, 4))})
        body = Action4D(vx=rng.uniform(-6, 6), vy=rng.uniform(-6, 6),
                        vz_up=rng.uniform(-3, 3), yaw_rate=0.0)
        a, b, c = Shield(_policy()), Shield(_policy()), Shield(_policy())
        da, ea = a.filter_body(st, body, arc=True)
        db, eb = b.filter_body(st, body, arc=False)
        dc = c.filter(st, from_body(body.vx, body.vy, body.vz_up, 0.0, st.yaw_deg))
        _same(da, db)
        _same(da, dc)
        assert ea == eb, k
        assert a.history[-1].trajectory == b.history[-1].trajectory == \
            c.history[-1].trajectory, k
        raw = da.raw
        assert a.forecast(st, raw, arc=True) == a.forecast(st, raw), k


def test_the_straight_forecast_is_the_expressions_it_always_was():
    """With the arc off - any yaw rate - each pose is state + v * t, the
    expression the forecast has always used, to the bit."""
    sh = Shield(_policy())
    rng = random.Random(7)
    for _ in range(30):
        st = State(x=rng.uniform(-20, 20), y=rng.uniform(-20, 20), up=12.0,
                   yaw_deg=rng.uniform(0, 360))
        a = Action4D(vx=rng.uniform(-4, 4), vy=rng.uniform(-4, 4),
                     vz_up=rng.uniform(-1, 1), yaw_rate=rng.uniform(-0.7, 0.7))
        yaw_dps = math.degrees(a.yaw_rate)
        for t, p in sh.forecast(st, a):
            assert (p.x, p.y, p.up, p.yaw_deg) == (
                st.x + a.vx * t, st.y + a.vy * t, st.up + a.vz_up * t,
                st.yaw_deg + yaw_dps * t)


def test_the_own_velocity_forecast_adds_the_arc_to_both_lines():
    """With a measured velocity and the arc the forecast is FOUR poses a
    grid time: the own-velocity line, its arc, the commanded line, its arc
    (THE ARC ONLY ADDS). The two lines are the poses the arc-off forecast
    yields, to the bit; the commanded arc is the arc exactly; the
    own-velocity arc ends on the arc's side of its line."""
    sh = Shield(_policy())
    st = ST.model_copy(update={"velocity": (4.0, 0.0)})
    raw = from_body(4.0, 0.0, 0.0, 0.4, 0.0)
    straight = list(sh._predict(st, raw))
    sh._local.arc = True
    try:
        poses = list(sh._predict(st, raw))
    finally:
        sh._local.arc = False
    assert len(poses) == 2 * len(straight)
    own, own_arc, cmd, cmd_arc = poses[0::4], poses[1::4], poses[2::4], poses[3::4]
    assert own + cmd == straight[0::2] + straight[1::2]
    for (t, p) in cmd_arc:
        ax, ay = _arc_offset(4.0, 0.0, 0.4, t)
        assert (p.x, p.y) == (st.x + ax, st.y + ay)
    assert own_arc[-1][1].y > own[-1][1].y + 1.0, (own_arc[-1], own[-1])


def test_the_arc_only_adds_to_the_straight_forecast():
    """Review 2026-11-17 (major): the arc REPLACED the line, so a rule only
    the line breaks was lost. Now, with the arc on, every straight pose is
    still yielded (first at each grid time), then the arc's - and on random
    states and turning body commands round nfz-square, every rule the
    straight forecast breaks is broken with the arc on too, at the same or
    an earlier time."""
    rng = random.Random(20261117)
    for k in range(60):
        st = State(x=rng.uniform(-10, 35), y=rng.uniform(-10, 35),
                   up=15.0, yaw_deg=rng.uniform(-180, 180))
        if k % 3 == 0:
            st = st.model_copy(update={"velocity": (rng.uniform(-4, 4), rng.uniform(-4, 4))})
        body = Action4D(vx=rng.uniform(-4, 4), vy=rng.uniform(-2, 2), vz_up=0.0,
                        yaw_rate=rng.choice([-1, 1]) * rng.uniform(0.05, 0.78))
        raw = from_body(body.vx, body.vy, body.vz_up, body.yaw_rate, st.yaw_deg)
        sh = Shield(_policy())
        line = sh._check(st, raw)
        sh._local.arc = True
        try:
            both = sh._check(st, raw)
            poses = list(sh._predict(st, raw))
        finally:
            sh._local.arc = False
        ref = list(Shield(_policy())._predict(st, raw))
        if st.velocity is None:
            assert poses[0::2] == ref, k
        else:
            assert poses[0::4] + poses[2::4] == ref[0::2] + ref[1::2], k
        got = {v.rule_id: v.predicted_at_s for v in both}
        for v in line:
            assert v.rule_id in got and got[v.rule_id] <= v.predicted_at_s + 1e-9, \
                (k, v, both)


def _alternating_yaw_loop(arc: bool, w: float, max_ticks: int = 60):
    """The review's closed loop (2026-11-17), ideal kinematics: a zone whose
    1 m ring starts at x = 19 m, the aircraft at the origin heading North
    (+x), the pilot body-forward 4 m/s with a yaw rate whose sign alternates
    every tick - nearly straight flight. The x of the first tick the Shield
    repairs, or None."""
    pol = Policy.model_validate({"policy_id": "alt-yaw", "constraints": [
        {"id": "caps", "type": "kinematic_envelope", "scope": "mission",
         "constraint_type": "hard", "priority": "P1", "violation_action": "repair",
         "speed_max_mps": 4.0, "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0},
        {"id": "nfz-dead-ahead", "type": "polygon_fence", "scope": "mission",
         "constraint_type": "hard", "priority": "P0", "violation_action": "repair",
         "margin_m": 1.0, "altitude_floor_m": 0.0, "altitude_ceiling_m": 100.0,
         "vertices": [{"x": 20, "y": -10}, {"x": 40, "y": -10}, {"x": 40, "y": 10},
                      {"x": 20, "y": 10}]}]})
    sh = Shield(pol)
    x = y = yaw = 0.0
    for i in range(max_ticks):
        body = Action4D(vx=4.0, vy=0.0, vz_up=0.0, yaw_rate=w if i % 2 == 0 else -w)
        d, eb = sh.filter_body(State(x=x, y=y, up=15.0, yaw_deg=yaw), body, arc=arc)
        if d.touched:
            return x
        wv = from_body(eb.vx, eb.vy, eb.vz_up, eb.yaw_rate, yaw)
        x, y = x + wv.vx * sh.dt, y + wv.vy * sh.dt
        yaw += math.degrees(eb.yaw_rate) * sh.dt
    return None


def test_a_yaw_rate_alternating_each_tick_is_repaired_no_later_with_the_arc():
    """The review's case: a VLA whose yaw output alternates sign flies
    nearly straight at a zone dead ahead. With the arc in place of the line
    the first repair came at x = 9.2 m (+/-0.4 rad/s) and 13.99 m (+/-0.78)
    against x = 0 on the line. With the arc added to the line it comes
    exactly where the line's does."""
    straight = _alternating_yaw_loop(False, 0.4)
    assert straight is not None and straight < 1.0, straight
    for w in (0.4, 0.78):
        line = _alternating_yaw_loop(False, w)
        arc = _alternating_yaw_loop(True, w)
        assert arc is not None and line is not None, (w, arc, line)
        assert arc <= line + 1e-9, (w, arc, line)


def test_a_subnormal_yaw_rate_is_forecast_on_the_line():
    """Review 2026-11-17 (minor): yaw_rate 5e-324 at 4 m/s underflowed w * t
    and the arc read x = 0.0 m at 0.2 s (the line: 0.8 m). Below
    ARC_MIN_RATE the forecast is the line's, bit for bit, and _arc_offset
    itself goes to the line as w * t -> 0 (its series)."""
    for w in (5e-324, 1e-300, 1e-12, -1e-10):
        assert abs(_arc_offset(4.0, 0.0, w, 0.2)[0] - 0.8) < 1e-12, w
        assert abs(_arc_offset(4.0, 1.0, w, 5.0)[1] - 5.0) < 1e-8, w
        raw = Action4D(vx=4.0, yaw_rate=w)
        sh = Shield(_policy())
        assert sh.forecast(ST, raw, arc=True) == sh.forecast(ST, raw), w
    assert abs(ARC_MIN_RATE - 1e-9) < 1e-24
    # Just over the threshold the series still matches the line to 1e-12 m.
    p = Shield(_policy()).forecast(ST, Action4D(vx=4.0, yaw_rate=2e-9), arc=True)
    assert all(abs(q.x - (ST.x + 4.0 * t)) < 1e-12 for t, q in p)


def test_an_arced_tick_is_on_record():
    """Review 2026-11-17 (minor): nothing said whether a tick arced, so a
    rebuild of an arced flight's forecast was silently the line. Now the
    TickRecord (`arc`), the decision trace ("forecast_arc") and the audit
    record (`forecast_arc`, omitted otherwise) say so, and forecast(...,
    arc=rec.arc) rebuilds the arc the tick judged beside the line."""
    from guardrail.audit import AuditLogger
    sh = Shield(_policy())
    d, _ = sh.filter_body(ST, TURN, arc=True)
    rec = sh.history[-1]
    assert rec.arc is True and d.trace["forecast_arc"] is True
    arc = sh.forecast(rec.state, rec.decision.raw, arc=rec.arc)
    line = sh.forecast(rec.state, rec.decision.raw)
    assert [(t, p.x, p.y) for t, p in arc] == [(t, q.x, q.y) for t, q in rec.trajectory[1::2]]
    assert [(t, p.x, p.y) for t, p in line] == [(t, q.x, q.y) for t, q in rec.trajectory[0::2]]
    a = AuditLogger.in_memory(_policy())
    assert AuditLogger.line(a.record(1, d))["forecast_arc"] is True
    # The same command with the arc off, and a zero yaw rate with it on:
    # nothing new on record.
    for kw, body in (({"arc": False}, TURN), ({"arc": True}, Action4D(vx=4.0))):
        sh2 = Shield(_policy())
        d2, _ = sh2.filter_body(ST.model_copy(update={"x": 0.0, "y": 30.0}), body, **kw)
        assert sh2.history[-1].arc is False and "forecast_arc" not in (d2.trace or {})
        r2 = a.record(2, d2, kind="stop_advised")
        assert "forecast_arc" not in AuditLogger.line(r2)


def test_an_arced_escape_from_an_opened_zone_is_judged_on_the_full_lines():
    """Review 2026-11-17 (minor): the escape shaping for a zone whose window
    opened on the aircraft (Shield._shaped_escape, the review of 69c27bd,
    P2) is a straight exit and is off for an arced action, which is judged
    on the full 5 s line and arc instead. Pinned here, in the library's own
    geometry (tests/test_window_opens_on.py, the corridor edge 9 m past the
    school zone's ring): a body command east with yaw rate 0 flies the cap
    out and stays Normal, as the world-frame test does; the same command
    with a small yaw rate has its cap refused (flown at 3.50 m/s, the
    corridor in the cap's 20 m reach), and at (70.5, 21) the corridor is
    raised and the FSM brakes - the 69c27bd behaviour, for this opt-in arm
    only. Nothing is emitted that re-checks dirty in either."""
    from datetime import datetime, timedelta
    survey = load_policy(ROOT / "policies" / "corridor_survey.yaml")
    t0 = datetime(2026, 9, 7, 7, 29, 58)
    out = {}
    for w in (0.0, 0.05):
        clock = [t0]
        sh = Shield(survey, now=lambda: clock[0])
        east = Action4D(vx=2.0, yaw_rate=w)
        sh.filter_body(State(x=60.0, y=21.0, up=15.0), east, t=0.0, arc=True)
        clock[0] = t0 + timedelta(seconds=0.4)
        d1, e1 = sh.filter_body(State(x=61.6, y=21.0, up=15.0), east, t=0.4, arc=True)
        clock[0] = t0 + timedelta(seconds=2.6)
        d2, e2 = sh.filter_body(State(x=70.5, y=21.0, up=15.0), east, t=2.6, arc=True)
        assert d1.trace["window_opens_on"] == ["nfz-school"], d1.trace
        assert d2.trace["window_opened_on"] == ["nfz-school"], d2.trace
        assert d1.emitted_violations == [] and d2.emitted_violations == [], w
        out[w] = (e1, d1, e2, d2)
    e1, d1, e2, d2 = out[0.0]
    assert abs(e1.vx - 4.0) < 1e-9 and abs(e2.vx - 4.0) < 1e-9, (e1, e2)
    assert "cap_refused" not in d1.trace and d2.violations == [] and d2.fsm_state_after == "Normal"
    e1, d1, e2, d2 = out[0.05]
    assert abs(e1.vx - 3.5) < 1e-9 and abs(e1.vy) < 1e-9, e1
    assert d1.trace["cap_refused"] == ("flown at 3.50 m/s; at the cap the floored action "
                                       "breaks corridor-survey-route"), d1.trace
    assert [v.rule_id for v in d2.violations] == ["corridor-survey-route"], d2.violations
    assert d2.fsm_state_after == "Brake", d2.fsm_state_after


# ---- the rails ------------------------------------------------------------- #

def _pas_modules():
    for p in (ROOT / "demo" / "pas_ardupilot", ROOT / "demo", ROOT / "tests"):
        if str(p) not in sys.path:
            sys.path.insert(0, str(p))
    import autopilot as AP
    import mavros_link as ML
    import test_pas_ardupilot as T
    return AP, ML, T


def test_the_pymavlink_link_turns_the_body_action_into_ned_after_projection():
    """--action-frame body on the PAS rail: the link is handed the Shield's
    emitted BODY action and turns it into local NED itself, at the heading it
    last read - the same setpoint the world route sends for the same action.
    `a` is then only the record copy."""
    AP, _, T = _pas_modules()
    fm = T.FakeMavutil([T.AP_HB])
    link = T._link(fm)
    link.connect(timeout=5)
    body = (3.0, -1.0, 0.5, 0.25)
    for heading in (0.0, 37.0, 90.0, 200.0, -75.0):
        link.snap.yaw = math.radians(heading)
        link.send(Action4D(vx=99.0), body=body)
        got = fm.master.mav.sent[-1][1]
        world = from_body(*body, heading)
        link.send(world)
        ref = fm.master.mav.sent[-1][1]
        assert all(abs(x - y) < 1e-9 for x, y in zip(got[8:11], ref[8:11])), (heading, got, ref)
        assert got[15] == 0.25 and got[1:5] == ref[1:5]


def test_the_mavros_link_sends_the_body_action_as_it_is_or_turns_it():
    """On the adapter route the emitted BODY action goes to /shield/setpoint
    as it is (sitl/mavlink_adapter_node.py converts it after the projection);
    on the default route the link turns it into the PositionTarget itself.
    Without `body` both routes send what they always did."""
    _, ML, T = _pas_modules()

    class Node(T.FakeNode):
        def count_subscribers(self, topic):
            return 1

        def count_publishers(self, topic):
            return 1
    body = (2.0, 1.5, -0.3, -0.2)
    node = Node()
    link = ML.MavrosLink(node=node, rclpy=T.FAKE_RCLPY, msgs=T.FAKE_MSGS,
                         clock=T.Clock(0.01), setpoint_via="adapter")
    link.snap.yaw = math.radians(123.0)
    link.send(Action4D(vx=9.0), body=body)
    assert node.pubs[ML.TOPIC_ADAPTER_SETPOINT].msgs[-1].data == [float(v) for v in body]
    assert node.pubs[ML.TOPIC_SAFE_ACTION].msgs[-1].data == [9.0, 0.0, 0.0, 0.0]
    dnode = T.FakeNode()
    direct = ML.MavrosLink(node=dnode, rclpy=T.FAKE_RCLPY, msgs=T.FAKE_MSGS,
                           clock=T.Clock(0.01))
    direct.snap.yaw = math.radians(123.0)
    direct.send(Action4D(vx=9.0), body=body)
    sp = dnode.pubs[ML.TOPIC_SETPOINT].msgs[-1]
    w = from_body(*body, 123.0)
    assert abs(sp.velocity.x - w.vy) < 1e-9 and abs(sp.velocity.y - w.vx) < 1e-9
    assert abs(sp.velocity.z - w.vz_up) < 1e-9 and sp.yaw_rate == 0.2


def test_the_perception_node_hands_the_shield_the_body_action():
    """The PAS rail's --action-frame body path is wired: filter_body with the
    arc, and the link given the emitted body action; the default flies the
    world frame (link.send without `body`), the Shield handed the body
    action through frames.filter_as_body with the arc off (2026-11-18,
    tests/test_rail_body_frame.py)."""
    src = (ROOT / "demo" / "pas_ardupilot" / "perception_node.py").read_text(encoding="utf-8")
    assert '"--action-frame", choices=["world", "body"], default="world"' in src
    assert "shield.filter_body(" in src and "arc=True" in src
    assert "link.send(flown, raw_body=body, body=fb)" in src
    assert "d, emitted_body = filter_as_body(shield, st_sh, raw), None" in src


def test_the_pas_rail_flies_the_emitted_body_action_end_to_end():
    """Review 2026-11-17 (minor): the --action-frame body branch was pinned
    by source strings only. Here it FLIES, through test_pas_ardupilot's fake
    flight (_fly), with a link whose send takes `body` and flies
    from_body(body) at its own heading (0.6 rad, so the rotations are not
    the identity), as PymavlinkLink does. On every tick sent - a repaired
    pass (CAP), an untouched pass and a brake (a fence whose action is
    brake, own-velocity forecast off), a stop-advised brake (the same, the
    forecast on), the Shield off (the raw action) - the body handed over is
    to_body of the flown (record-copy) action at the tick's heading, the
    Shield's emitted body action on a pass. metrics says the frame; with
    the Shield on nothing flown re-checks dirty. Without the flag the link
    is never handed a body action and the frame is "world"."""
    _, _, T = _pas_modules()

    class BodyLink(T.FakeLink):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self.bodies = []

        def velocity(self):
            return (self.v.vx, self.v.vy), "the fake's own"

        def send(self, a, raw_body=None, body=None):
            h = math.degrees(self.yaw)
            self.bodies.append((a, body, h))
            super().send(a if body is None else from_body(*body, h), raw_body)

    brake_pol = T.NFZ_AHEAD_POLICY.replace("violation_action: repair", "violation_action: brake")
    seen = set()
    for shield, pol, extra in (
            ("on", T.CAP_POLICY, ()),
            ("on", brake_pol, ("--own-velocity-forecast", "off")),
            ("on", brake_pol, ("--own-velocity-forecast", "on")),
            ("off", T.CAP_POLICY, ())):
        rc, run, link = T._fly(shield, link_cls=BodyLink, policy=pol, link_kw={"yaw": 0.6},
                               extra=("--action-frame", "body") + extra)
        assert rc == 0, (shield, extra)
        rows = T._rows(run)
        flown = [r for r in rows if r["flown"]]
        assert len(link.bodies) == len(flown) > 0, (len(link.bodies), len(flown))
        for (a, body, h), r in zip(link.bodies, flown):
            assert body is not None, r["t"]
            ref = to_body(a, h)
            assert max(abs(x - y) for x, y in zip(body, ref)) < 1e-9, (body, ref, r["t"])
            assert a.model_dump() == r["flown_action"]
            seen.add((shield, r["fsm_setpoint"], bool(r["stop_advised"]), bool(r["touched"])))
        m = T._json(run, "metrics.json")
        assert m["shield_forecast"]["action_frame"] == "body", m["shield_forecast"]
        if shield == "on":
            assert m["escape_ticks"] == 0, (extra, m["escape_ticks"])
    for kind in (("on", "pass", False, True), ("on", "pass", False, False),
                 ("on", "brake", False, True), ("on", "brake", True, True)):
        assert kind in seen, (kind, seen)
    assert any(k[0] == "off" for k in seen), seen
    rc, run, link = T._fly("on", link_cls=BodyLink, policy=T.CAP_POLICY, link_kw={"yaw": 0.6})
    assert rc == 0 and link.bodies and all(b is None for _, b, _ in link.bodies)
    assert T._json(run, "metrics.json")["shield_forecast"]["action_frame"] == "world"


def test_a_campaign_asks_the_node_for_the_arc_and_refuses_a_run_that_did_not_fly_it():
    """Review 2026-11-17 (major): the ROS 2 rig commands could not be run - a
    campaign passed no extra node arguments and rail_extra_args allowed the
    hot-apply flags only - and a node from an older bundle would drop
    --arc-forecast without a word (parse_known_args). Now
    --rail-arc-forecast on hands every mission's node --arc-forecast=on
    (checked as the hot-apply flags are: declared by the node, a value it
    takes, once), and a run whose metrics.json shield_forecast.arc is not
    true is refused, as is one that arced unasked."""
    sys.path.insert(0, str(ROOT / "experiments"))
    import sweep_scenarios as S
    assert S.rail_extra_args(["--arc-forecast=on"]) == ["--arc-forecast=on"]
    both = ["--hotapply-port=8071", "--arc-forecast=off"]
    assert S.rail_extra_args(both) == both
    for bad in (["--arc-forecast=yes"], ["--arc-forecast", "on"],
                ["--arc-forecast=on", "--arc-forecast=off"], ["--own-velocity-forecast=on"]):
        try:
            S.rail_extra_args(bad)
        except S.UnsupportedScenario:
            pass
        else:
            raise AssertionError(f"{bad} was accepted")
    on = ["--arc-forecast=on"]
    ok = {"shield_forecast": {"mode": "commanded", "arc": True}}
    off = {"shield_forecast": {"mode": "commanded", "arc": False}}
    old = {"shield_forecast": {"mode": "commanded"}}             # a node before the flag
    assert S.arc_forecast_problems(ok, on) == []
    assert S.arc_forecast_problems(off, None) == [] == S.arc_forecast_problems(old, None)
    assert S.arc_forecast_problems({}, None) == []
    for m in (off, old, {}):
        assert "did not fly the arc" in S.arc_forecast_problems(m, on)[0], m
    assert "no --arc-forecast=on was asked" in S.arc_forecast_problems(ok, None)[0]
    # The option, refused off a rail, and on a re-score.
    assert S.main(["--rail-arc-forecast", "on", "--only", "nfz-head-on"]) == 2
    src = (ROOT / "experiments" / "sweep_scenarios.py").read_text(encoding="utf-8")
    assert src.count("extra_args=rail_arc_args or None") == 2
    assert src.count("arc_forecast_problems(metrics, extra_args)") == 2


def test_the_ros2_node_arcs_only_when_asked():
    """The ROS 2 rail (already body frame through filter_body) takes the arc
    behind --arc-forecast, off by default, so the KPI campaigns' stub flights
    and every recorded VLA flight's forecast are unchanged unless asked."""
    sys.path.insert(0, str(ROOT / "experiments"))
    import sweep_scenarios as S
    src = (ROOT / "sitl" / "ros2_shield_node.py").read_text(encoding="utf-8")
    assert '"--arc-forecast", choices=["on", "off"], default="off"' in src
    assert '{"arc_forecast": True} if self.arc else {}' in src
    assert "--arc-forecast" in S.sitl_node_flags()


def test_the_headless_body_arm_flies_the_world_arm():
    """run_episode(action_frame="body"): the Shield is handed the body
    action (filter_body with the arc) and the integrator flies the emitted
    body action turned back after the projection. Its pilots command no yaw
    rate and the vehicle does not yaw, so the rows match the world arm's to
    the rotations' rounding; the arm is on the episode_start record, and an
    unknown frame is refused."""
    sys.path.insert(0, str(ROOT / "experiments"))
    import sweep_scenarios as S
    from guardrail.scenario_spec import Library
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    spec = lib.by_id()["nfz-head-on"].first.spec.model_copy(deep=True)
    # A heading off North, so the two rotations are not the identity.
    spec.mission.start_pose.yaw_deg = 37.0
    w = S.run_episode(spec, lib.defaults, 1001)
    b = S.run_episode(spec, lib.defaults, 1001, action_frame="body")
    assert "action_frame" not in w.events[0] and b.events[0]["action_frame"] == "body"
    assert len(w.rows) == len(b.rows), (len(w.rows), len(b.rows))
    worst = 0.0
    for rw, rb in zip(w.rows, b.rows):
        worst = max(worst, abs(rw["x"] - rb["x"]), abs(rw["y"] - rb["y"]),
                    abs(rw["up"] - rb["up"]))
        assert bool(rw["repairs"]) == bool(rb["repairs"]), rw["t"]
    assert worst < 1e-6, worst
    assert sum(1 for r in b.rows if r["repairs"]) > 100, "the Shield never acted: blind"
    for k in ("breaches", "polygon_entries", "true_p0_flown_ticks", "reached_goal"):
        assert w.extra[k] == b.extra[k], k
    try:
        S.run_episode(spec, lib.defaults, 1001, action_frame="ned")
    except ValueError as e:
        assert "action frame" in str(e)
    else:
        raise AssertionError("an unknown action frame was flown")


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except Exception as e:                      # noqa: BLE001
            failed += 1
            print(f"FAIL  {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
