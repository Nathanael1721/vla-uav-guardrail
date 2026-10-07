"""The MAVLink adapter's conversions, checked without ROS.

Run either way:
    pytest tests/test_mavlink_adapter.py -v
    python tests/test_mavlink_adapter.py

sitl/mavlink_adapter_node.py is the single body -> local-NED crossing the grant
asks for, and the one place where a sign or an axis swap turns into an aircraft
flying somewhere else with no error anywhere. So the tests here model what
MAVROS does to the message (ENU -> NED) and check the whole chain against
guardrail.frames for every heading, and pin what NED values written into a
message MAVROS reads as ENU would do. They also cover the GeoFence side: which
zones reach the autopilot and when, the altitude fence against every policy's
ceiling, and the modes the adapter forwards.
"""
import math
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sitl"))

import mavlink_adapter_node as A                                    # noqa: E402
from guardrail import load_policy                                   # noqa: E402
from guardrail.frames import from_body, to_local_ned                # noqa: E402
from guardrail.models import PolygonFence, XY                       # noqa: E402
from guardrail.projection import LocalProjection                    # noqa: E402

POLICY = ROOT / "policies" / "sim_demo_policy.yaml"
PARM = ROOT / "sitl" / "fence" / "guardrail_fence.parm"
HOME = (-35.363261, 149.165230)        # sitl/start_sitl.sh --home


def _close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def test_the_type_mask_is_the_reference_adapters():
    """Velocity + yaw_rate commanded; position, acceleration and yaw ignored.
    1479 is also what the pymavlink rail sends (VEL_YAWRATE_MASK)."""
    assert A.TYPE_MASK_VEL_YAWRATE == 1479
    m = A.TYPE_MASK_VEL_YAWRATE
    assert not m & (A.IGNORE_VX | A.IGNORE_VY | A.IGNORE_VZ | A.IGNORE_YAW_RATE)
    assert m & A.IGNORE_YAW and m & A.IGNORE_PX


def test_what_mavros_sends_is_the_ned_setpoint_for_every_heading():
    """The whole chain: body action -> this adapter's message -> MAVROS's
    ENU->NED conversion must equal from_body + to_local_ned."""
    body = (3.0, -1.5, 0.7, 0.25)
    for heading in range(0, 360, 15):
        sp = A.setpoint_from_body(body, heading)
        want = to_local_ned(from_body(*body, heading))
        assert _close(A.mavros_sends(sp["fields"]), want), (heading, sp, want)
        assert _close(sp["ned"], want)


def test_forward_at_heading_east_flies_east_and_up_climbs():
    """Two plain facts a reader can check by eye."""
    sent = A.mavros_sends(A.setpoint_from_body((2.0, 0.0, 1.0, 0.0), 90.0)["fields"])
    v_n, v_e, v_d, _ = sent
    assert abs(v_n) < 1e-9 and abs(v_e - 2.0) < 1e-9, sent
    assert abs(v_d + 1.0) < 1e-9, "up must arrive as negative NED down"
    # right at heading North is East
    v_n, v_e, _, _ = A.mavros_sends(A.setpoint_from_body((0, 1.0, 0, 0), 0.0)["fields"])
    assert abs(v_e - 1.0) < 1e-9 and abs(v_n) < 1e-9


def test_the_yaw_rate_reaches_the_autopilot_clockwise_and_unscaled():
    """Action4D's yaw_rate is rad/s clockwise; SET_POSITION_TARGET_LOCAL_NED's
    is rad/s clockwise too. The message carries it counter-clockwise (ENU)
    and MAVROS flips it back. No radians() anywhere (see
    docs/FINDING-the-contract-disagreed-with-itself-about-yaw.md)."""
    sp = A.setpoint_from_body((0, 0, 0, 0.3), 123.0)
    assert sp["fields"]["yaw_rate"] == -0.3
    assert A.mavros_sends(sp["fields"])[3] == 0.3


def test_ned_values_packed_unconverted_would_reach_ardupilot_swapped_and_inverted():
    """Why the adapter packs ENU. (v_north, v_east, v_down) written straight
    into velocity.x/y/z of a message MAVROS reads as ENU: fed through
    MAVROS's conversion, a climb north becomes a descent east. If this test
    ever stops showing that, the model of MAVROS in mavros_sends() has
    changed and the adapter must be re-checked."""
    v_n, v_e, v_d, w = to_local_ned(from_body(2.0, 0.0, 1.0, 0.0, 0.0))
    ref_fields = {"coordinate_frame": A.FRAME_LOCAL_NED,
                  "velocity": (v_n, v_e, v_d), "yaw_rate": w}
    sent = A.mavros_sends(ref_fields)
    assert abs(sent[0]) < 1e-9 and abs(sent[1] - 2.0) < 1e-9, sent   # east
    assert sent[2] > 0, "a commanded climb arrives as a descent"
    ours = A.mavros_sends(A.setpoint_from_body((2.0, 0, 1.0, 0), 0.0)["fields"])
    assert abs(ours[0] - 2.0) < 1e-9 and ours[2] < 0, ours


def test_heading_is_read_from_the_enu_pose_quaternion():
    def q(yaw_enu_deg):
        h = math.radians(yaw_enu_deg) / 2
        return (0.0, 0.0, math.sin(h), math.cos(h))

    def ang(a, b):                       # circular difference, degrees
        return abs((a - b + 180.0) % 360.0 - 180.0)
    for yaw_enu, heading in ((90, 0.0), (0, 90.0), (180, 270.0), (-90, 180.0)):
        got = A.heading_deg_from_enu_quaternion(*q(yaw_enu))
        assert 0.0 <= got < 360.0 and ang(got, heading) < 1e-9, (yaw_enu, got)


def test_a_malformed_or_non_finite_setpoint_is_refused_not_flown():
    for bad in ((1.0, 2.0, 3.0), (float("nan"), 0, 0, 0), (0, float("inf"), 0, 0)):
        try:
            A.setpoint_from_body(bad, 0.0)
        except ValueError:
            continue
        raise AssertionError(f"{bad} was turned into a setpoint")


def test_policy_fences_export_the_no_fly_zone_and_say_what_they_skip():
    pol = load_policy(POLICY)
    pol.constraints.append(PolygonFence(
        id="nfz-high", type="polygon_fence", altitude_floor_m=40.0,
        vertices=[XY(x=50, y=50), XY(x=60, y=50), XY(x=60, y=60)]))
    fences, skipped = A.policy_fences(pol, column_m=30.0)
    assert [f["id"] for f in fences] == ["nfz-square"], fences
    assert skipped and skipped[0]["id"] == "nfz-high" and "stricter" in skipped[0]["why"]


def test_a_circle_fence_is_exported_as_an_exclusion_circle():
    """The reference DSL's circle_fence (centre + radius), read by duck type."""
    class Circle:
        id, type = "nfz-circle", "circle_fence"
        center, radius_m = {"x": 10.0, "y": -5.0}, 12.0
        altitude_floor_m, altitude_ceiling_m = 0.0, 120.0

    class Pol:
        constraints = [Circle()]
    fences, skipped = A.policy_fences(Pol(), column_m=30.0)
    assert fences == [{"id": "nfz-circle", "kind": "circle",
                       "center": [10.0, -5.0], "radius_m": 12.0}], (fences, skipped)
    items = A.fence_items(fences, *HOME)
    assert len(items) == 1 and items[0]["command"] == 5004
    assert items[0]["param1"] == 12.0


def test_fence_vertices_land_where_the_policy_put_them():
    """Projected about home and back with the policy loader's projection: a
    vertex 23 m North and 7 m East of home must come back as (23, 7), not as
    (7, 23) - the axis-order trap guardrail/projection.py warns about."""
    pol = load_policy(POLICY)
    fences, _ = A.policy_fences(pol, column_m=30.0)
    items = A.fence_items(fences, *HOME)
    assert len(items) == 4 and all(i["command"] == 5002 for i in items)
    assert all(i["param1"] == 4.0 for i in items), "vertex count in param1"
    proj = LocalProjection(*HOME)
    back = [proj.to_local(i["x_lat"], i["y_long"]) for i in items]
    want = [(7, 7), (23, 7), (23, 23), (7, 23)]
    for (n, e), (wn, we) in zip(back, want):
        assert abs(n - wn) < 1e-6 and abs(e - we) < 1e-6, (back, want)
    # An offset home: the fence stays put in the local frame.
    shifted = A.fence_items(fences, *HOME, home_local_ne=(5.0, -3.0))
    n, e = proj.to_local(shifted[0]["x_lat"], shifted[0]["y_long"])
    assert abs(n - 2.0) < 1e-6 and abs(e - 10.0) < 1e-6, (n, e)


def test_the_autopilot_fence_column_stays_above_the_policy_ceiling():
    """The backstop must never fire before the Shield. FENCE_ALT_MAX in the
    .parm that start_sitl.sh loads has to sit above every altitude envelope
    the demo policies allow, or ArduPilot would RTL a flight the Shield
    considers legal."""
    alt_max = A.read_fence_alt_max(PARM)
    assert alt_max is not None, f"{PARM} has no FENCE_ALT_MAX"
    pol = load_policy(POLICY)
    from guardrail.models import AltitudeEnvelope
    ceilings = [e.alt_max_m for e in pol.by_type(AltitudeEnvelope)]
    assert ceilings and alt_max > max(ceilings), (alt_max, ceilings)
    fences, _ = A.policy_fences(pol, column_m=alt_max)
    assert fences, "the demo no-fly zone must reach the autopilot fence"


def test_the_fence_parm_enables_the_backstop_with_rtl():
    vals = {}
    for line in PARM.read_text(encoding="utf-8").splitlines():
        parts = line.split("#", 1)[0].split()
        if len(parts) >= 2:
            vals[parts[0]] = float(parts[1])
    assert vals.get("FENCE_ENABLE") == 1.0
    assert vals.get("FENCE_ACTION") == 1.0, "1 = RTL or Land on ArduCopter"
    # bit 0 max altitude + bit 2 polygons and circles
    assert int(vals.get("FENCE_TYPE", 0)) & 0b101 == 0b101, vals.get("FENCE_TYPE")
    # The fence is a backstop, not a second filter: ArduCopter's simple
    # avoidance (on by default, and applied in GUIDED velocity control) would
    # otherwise bend the setpoints near every zone. Flown 2026-10-06 with it
    # on: the shield-off arm stalled 2 m short of the zone for 20 s.
    assert vals.get("AVOID_ENABLE") == 0.0, "simple avoidance must be off"


def test_no_parm_line_overflows_ardupilots_line_buffer():
    """AP_Param reads a defaults file with fgets(line, 99): a longer line is
    split, and its tail is parsed as a NEW line - a comment whose tail began
    with a parameter name would set that parameter. Every line here must fit
    (97 characters plus the newline)."""
    for i, line in enumerate(PARM.read_text(encoding="utf-8").splitlines(), 1):
        assert len(line) <= 97, f"line {i} is {len(line)} chars: {line!r}"


def test_read_fence_alt_max_says_none_when_absent():
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "x.parm"
        p.write_text("FENCE_ENABLE 1\n", encoding="utf-8")
        assert A.read_fence_alt_max(p) is None
        assert A.read_fence_alt_max(Path(d) / "missing.parm") is None


def test_a_zone_over_the_aircraft_is_held_back_until_it_is_clear():
    """Review 2026-10-07: a zone hot-applied on top of the aircraft was
    uploaded at once and would breach the autopilot fence on arrival; ArduPilot
    then RTLs while the Shield is still steering out, and time to safe
    measures the autopilot. Held back while the aircraft is inside or within
    FENCE_MARGIN of it; exported once clear."""
    pol = load_policy(POLICY)                       # nfz-square: 7..23 x 7..23
    for pos, deferred in (((15.0, 15.0), True),     # inside
                          ((5.5, 15.0), True),      # 1.5 m outside, margin 2
                          ((2.0, 15.0), False),     # 5 m clear
                          (None, False)):           # position unknown
        fences, skipped = A.policy_fences(pol, column_m=30.0, position=pos)
        ids = [f["id"] for f in fences]
        held = [s for s in skipped if s.get("deferred")]
        if deferred:
            assert ids == [] and held and held[0]["id"] == "nfz-square", (pos, skipped)
        else:
            assert ids == ["nfz-square"] and not held, (pos, fences, skipped)
    assert A.zone_clearance_m({"kind": "polygon", "vertices": [[7, 7], [23, 7],
                               [23, 23], [7, 23]]}, 15.0, 15.0) == -8.0
    assert A.zone_clearance_m({"kind": "circle", "center": [0, 0],
                               "radius_m": 10.0}, 0.0, 13.0) == 3.0


def test_a_real_circle_fence_reaches_the_autopilot_as_a_circle():
    """guardrail.models.CircleFence (centre + radius, the grant's form) when
    the DSL has it: exported as ArduPilot's exclusion circle with the exact
    radius (the Shield's 32-gon circumscribes it, so the backstop is not
    stricter), and held back while the aircraft is inside."""
    try:
        from guardrail.models import CircleFence
        c = CircleFence(id="nfz-disc", type="circle_fence", center=XY(x=40, y=40),
                        radius_m=8.0)
    except Exception:                                # noqa: BLE001
        return                                       # DSL without circles
    pol = load_policy(POLICY)
    pol.constraints.append(c)
    fences, _ = A.policy_fences(pol, column_m=30.0)
    disc = [f for f in fences if f["id"] == "nfz-disc"]
    assert disc == [{"id": "nfz-disc", "kind": "circle", "center": [40.0, 40.0],
                     "radius_m": 8.0}], fences
    assert A.fence_items(disc, *HOME)[0]["command"] == 5004
    _, skipped = A.policy_fences(pol, column_m=30.0, position=(41.0, 39.0))
    assert any(s["id"] == "nfz-disc" and s.get("deferred") for s in skipped)


def test_a_zone_over_home_is_named_because_an_rtl_lands_in_it():
    """Flown 2026-10-07 with policies/poly_test.yaml: its tri-nfz has a vertex
    on the origin, so the zone was held back at take-off, uploaded once clear,
    and the GeoFence reported a breach when the FSM's RTL brought the aircraft
    home. The node now names such zones (metrics.zones_over_home)."""
    poly = load_policy(ROOT / "policies" / "poly_test.yaml")
    fences, _ = A.policy_fences(poly, column_m=60.0)
    assert A.zones_near(fences, 0.0, 0.0) == ["tri-nfz"], fences
    # On the pad (the node's position before the first pose) it is held back.
    up, held = A.policy_fences(poly, column_m=60.0, position=(0.0, 0.0))
    assert up == [] and held[0]["id"] == "tri-nfz" and held[0]["deferred"]
    demo, _ = A.policy_fences(load_policy(POLICY), column_m=30.0)
    assert A.zones_near(demo, 0.0, 0.0) == []
    src = (ROOT / "sitl" / "ros2_shield_node.py").read_text(encoding="utf-8")
    assert '"zones_over_home": self.zones_over_home' in src


def test_a_static_dynamic_nfz_is_exported_and_a_moving_one_is_not():
    """A mid-flight zone is a dynamic_nfz where the Shield enforces the
    grant's update model. Without motion it is a polygon the autopilot can
    hold; with motion it is not, and the record says the Shield alone holds
    it."""
    class Zone:
        type, altitude_floor_m, altitude_ceiling_m = "dynamic_nfz", 0.0, 1000.0

        def __init__(self, zid, motion):
            self.id, self.motion = zid, motion
            self.vertices = [XY(x=23, y=6), XY(x=31, y=6), XY(x=31, y=14)]

    class Pol:
        constraints = [Zone("nfz-static", None), Zone("nfz-moving", {"vx_mps": 1})]
    fences, skipped = A.policy_fences(Pol(), column_m=30.0)
    assert [f["id"] for f in fences] == ["nfz-static"], fences
    assert fences[0]["kind"] == "polygon" and len(fences[0]["vertices"]) == 3
    assert skipped and skipped[0]["id"] == "nfz-moving" and "static" in skipped[0]["why"]


def test_with_the_fence_column_unknown_no_zone_is_exported():
    fences, skipped = A.policy_fences(load_policy(POLICY), column_m=None)
    assert fences == [] and skipped and "unknown" in skipped[0]["why"]


def test_the_altitude_fence_sits_above_every_policys_ceiling_once_planned():
    """Review 2026-10-07: FENCE_ALT_MAX 30 is above the demo policies' 20 m,
    but the node flies any --policy and 18 policies in policies/ allow more
    than 25 m (up to 80 m), where the backstop would RTL a flight the Shield
    considers legal. The node now
    raises FENCE_ALT_MAX by fence_alt_plan; this checks the plan against
    every policy in the repository, not only the demo one."""
    from guardrail.models import load_policy as _load
    parm = A.read_fence_alt_max(PARM)
    raised, checked = [], 0
    for path in sorted((ROOT / "policies").glob("*.yaml")):
        try:
            pol = _load(path)
        except Exception:                            # noqa: BLE001
            continue                                 # negative fixtures etc.
        ceiling, _ = A.policy_ceiling_m(pol, 15.0)
        plan = A.fence_alt_plan(ceiling, parm)
        final = parm if plan["ok"] else plan["raise_to"]
        assert final >= ceiling + A.FENCE_ALT_HEADROOM_M, (path.name, plan)
        if not plan["ok"]:
            raised.append(path.name)
        checked += 1
    assert checked >= 10, checked
    assert raised, ("no policy needs a higher FENCE_ALT_MAX, so this test "
                    "would pass with the planning deleted")
    assert A.fence_alt_plan(20.0, 30.0) == {"ok": True, "raise_to": None, "why": None}


def test_a_zero_fence_alt_max_is_read_as_zero_not_replaced_by_a_default():
    """The node used `read_fence_alt_max(...) or 30.0`, which turned 0 into 30."""
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "x.parm"
        p.write_text("FENCE_ALT_MAX 0\n", encoding="utf-8")
        assert A.read_fence_alt_max(p) == 0.0
    assert A.fence_alt_plan(20.0, 0.0)["raise_to"] == 25.0
    assert A.fence_alt_plan(20.0, None)["ok"] is False


def test_only_the_modes_the_fsm_can_ask_for_are_forwarded():
    """The adapter used to forward ANY string on /shield/mode_request."""
    for m in ("GUIDED", "loiter", " RTL ", "LAND"):
        mode, refused = A.mode_request(m)
        assert refused is None, (m, refused)
    for m in ("STABILIZE", "ACRO", "AUTO", ""):
        mode, refused = A.mode_request(m)
        assert refused and refused["kind"] == "mode_refused", (m, refused)
    src = (ROOT / "sitl" / "mavlink_adapter_node.py").read_text(encoding="utf-8")
    assert "mode_request(msg.data)" in src


def test_a_second_setpoint_publisher_stops_forwarding():
    """A VLA backend misconfigured to publish on /shield/setpoint would fly
    unshielded actions with nothing recorded. The adapter checks the
    publisher count on every setpoint and drops all of them while it is
    above one (ArduPilot then stops in GUIDED for want of setpoints)."""
    src = (ROOT / "sitl" / "mavlink_adapter_node.py").read_text(encoding="utf-8")
    i = src.index("def _on_setpoint")
    body = src[i:src.index("def _log", i)]
    assert "self.count_publishers(TOPIC_SETPOINT)" in body
    assert body.index("if n_pub > 1:") < body.index("self.pub.publish(pt)")
    assert '"extra_setpoint_publisher"' in body


def test_a_geo_policy_far_from_home_is_refused_by_offset():
    """A policy anchored to lat/lon whose origin is not the autopilot's home
    would put the Shield's zones and the fence off by the offset."""
    assert A.origin_offset_m(None, *HOME) is None
    n, e = A.origin_offset_m(HOME, *HOME)
    assert abs(n) < 1e-6 and abs(e) < 1e-6
    proj = LocalProjection(*HOME)
    far = proj.to_latlon(-30.0, 40.0)          # origin 30 m S, 40 m W... of home
    n, e = A.origin_offset_m(far, *HOME)
    assert abs(math.hypot(n, e) - 50.0) < 0.5 and math.hypot(n, e) > A.ORIGIN_TOLERANCE_M
    src = (ROOT / "sitl" / "mavlink_adapter_node.py").read_text(encoding="utf-8")
    assert "ORIGIN_TOLERANCE_M" in src[src.index("def _upload"):]


def test_the_module_imports_without_ros_and_says_so_at_run_time():
    assert A.HAVE_ROS in (True, False)
    if not A.HAVE_ROS:
        try:
            A.main([])
        except SystemExit as e:
            assert "rclpy" in str(e)
            return
        raise AssertionError("main() ran without rclpy")


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
