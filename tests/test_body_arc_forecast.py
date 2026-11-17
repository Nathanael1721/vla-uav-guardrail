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
  * the arc forecast is the exact integral of the turning velocity;
  * inside Shield.filter_body(..., arc=True) a zone the arc reaches and the
    straight forecast misses is caught and repaired, and the repaired action
    re-checks clean on the arc;
  * a zero yaw rate, the arc off, and filter() (world frame) are forecast
    and decided BIT-IDENTICALLY to the straight forecast, with and without a
    measured velocity - so the stub pilot's world-frame KPI flights (yaw_rate
    0) and every rail that has not asked for the arc fly as before;
  * the PAS rail's links turn the Shield's emitted BODY action into local
    NED after the projection (`send(..., body=...)`), and the headless
    harness's body-frame arm flies the world-frame arm's rows to within the
    rotations' rounding.
"""
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail.frames import from_body, to_body           # noqa: E402
from guardrail.models import Action4D, State, load_policy  # noqa: E402
from guardrail.shield import Shield, _arc_offset            # noqa: E402

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
    for _ in range(40):
        vx, vy = rng.uniform(-5, 5), rng.uniform(-5, 5)
        w = rng.choice([-1, 1]) * rng.uniform(0.05, 1.2)
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


def test_the_own_velocity_commanded_pose_arcs_too():
    """With a measured velocity the forecast is two poses a grid time; the
    commanded one is the arc exactly, the own-velocity one ends on the
    arc's side of the line."""
    sh = Shield(_policy())
    st = ST.model_copy(update={"velocity": (4.0, 0.0)})
    raw = from_body(4.0, 0.0, 0.0, 0.4, 0.0)
    sh._local.arc = True
    try:
        poses = list(sh._predict(st, raw))
    finally:
        sh._local.arc = False
    own, cmd = poses[0::2], poses[1::2]
    for (t, p) in cmd:
        ax, ay = _arc_offset(4.0, 0.0, 0.4, t)
        assert (p.x, p.y) == (st.x + ax, st.y + ay)
    assert own[-1][1].y > 1.0, own[-1]


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
    arc, and the link given the emitted body action; the default stays
    world (Shield.filter, link.send without `body`)."""
    src = (ROOT / "demo" / "pas_ardupilot" / "perception_node.py").read_text(encoding="utf-8")
    assert '"--action-frame", choices=["world", "body"], default="world"' in src
    assert "shield.filter_body(" in src and "arc=True" in src
    assert "link.send(flown, raw_body=body, body=fb)" in src
    assert "d, emitted_body = shield.filter(st_sh, raw), None" in src


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
