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

# The three ConstraintBase fields the grant requires (IR schema 3.0.0, 2026-11-02),
# at the values they defaulted to before (repair = project_fix).
HARD_P0 = {"constraint_type": "hard", "priority": "P0", "violation_action": "project_fix"}

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
        id="nfz-high", type="polygon_fence", **HARD_P0, altitude_floor_m=40.0,
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
    # A plain import: CircleFence is in the DSL since 2026-10-07, and the old
    # `except Exception: return` guard turned any construction error into a
    # silent PASS (review of 613aa50, 2026-10-19).
    from guardrail.models import CircleFence
    c = CircleFence(id="nfz-disc", type="circle_fence", **HARD_P0, center=XY(x=40, y=40),
                    radius_m=8.0)
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


def test_the_fsm_s_loiter_reaches_arducopter_as_brake():
    """REGRESSION (7 Oct SITL ros2fix2_on_fsm; 8 Oct pas_ap_20261008_224440).
    ArduCopter's LOITER takes its climb rate from the throttle stick
    (mode_loiter.cpp, get_pilot_desired_climb_rate): with no pilot input in
    SITL it descended 14.2 m to the ground on 7 Oct and sank 1.4-1.8 m/s in
    every LOITER spell on 8 Oct. The adapter forwarded the FSM's word as the
    autopilot's mode. It now asks for BRAKE, which holds position and
    altitude with no pilot input (mode_brake.cpp: zero velocity target,
    set_pos_target_z_from_climb_rate_cm(0)). The FSM's vocabulary is
    unchanged: the request on /shield/mode_request still says LOITER."""
    assert A.DEFAULT_HOLD_MODE == "BRAKE"
    assert A.autopilot_mode("LOITER") == "BRAKE"
    assert A.autopilot_mode(" loiter ") == "BRAKE"
    assert A.autopilot_mode("LOITER", "LOITER") == "LOITER"
    for m in ("GUIDED", "RTL", "LAND"):
        assert A.autopilot_mode(m) == m and A.autopilot_mode(m, "LOITER") == m
    for bad, hold in (("STABILIZE", "BRAKE"), ("BRAKE", "BRAKE"), ("LOITER", "ALT_HOLD")):
        try:
            A.autopilot_mode(bad, hold)
        except ValueError:
            continue
        raise AssertionError(f"autopilot_mode({bad!r}, {hold!r}) accepted")
    # The node sends ArduCopter's mode, and both ROS rails can name it.
    src = (ROOT / "sitl" / "mavlink_adapter_node.py").read_text(encoding="utf-8")
    body = src[src.index("def _on_mode_request"):]
    body = body[:body.index("\n        def ", 10)]
    assert "SetMode.Request(custom_mode=ap)" in body and "autopilot_mode(mode" in body
    node = (ROOT / "sitl" / "ros2_shield_node.py").read_text(encoding="utf-8")
    assert "SetMode.Request(custom_mode=ap)" in node
    assert "self._asks.explains(mode)" in node
    assert "ap = self._asks.ask(mode, via_adapter=to_adapter)" in node


def test_mode_requests_are_kept_in_both_vocabularies_and_explain_only_the_current_spell():
    """REVIEW (2026-10-09). The Project AirSim node recorded ArduCopter's
    words (BRAKE) and the ROS node the FSM's (LOITER), so one escalation read
    differently on the two rails. And both judged a takeover against every
    mode asked for in the episode, so after one FSM hold a pilot's BRAKE from
    Mission Planner could never read as one. ModeRequests keeps both lists
    and explains a mode change only by a request made since the aircraft last
    came back to GUIDED."""
    m = A.ModeRequests()
    assert m.ask("LOITER") == "BRAKE" and m.explains("BRAKE") and not m.explains("LOITER")
    assert m.ask("GUIDED") == "GUIDED"
    m.entered_guided()
    assert not m.explains("BRAKE"), "a pilot's BRAKE after the hold is a takeover"
    assert m.ask("RTL") == "RTL" and m.explains("RTL") and not m.explains("LAND")
    m.ask("LAND")
    assert m.fsm == ["LOITER", "GUIDED", "RTL", "LAND"]
    assert m.autopilot == ["BRAKE", "GUIDED", "RTL", "LAND"]
    # A hold asked for AFTER the GUIDED request, before GUIDED is reported,
    # survives the report.
    m = A.ModeRequests()
    m.ask("GUIDED")
    m.ask("LOITER")
    m.entered_guided()
    assert m.explains("BRAKE")
    # GUIDED entered with no request of ours (bring-up, a pilot): all spent.
    m = A.ModeRequests()
    m.ask("RTL")
    m.entered_guided()
    assert not m.explains("RTL")
    # Through the adapter the hold mode is the adapter's: either counts.
    m = A.ModeRequests("LOITER")
    assert m.ask("LOITER", via_adapter=True) == "LOITER"
    assert m.explains("BRAKE") and m.explains("LOITER")
    m = A.ModeRequests()
    m.ask("LOITER", via_adapter=False)
    assert not m.explains("LOITER"), "sent as BRAKE: a LOITER now is someone else's"
    for bad in ("ALT_HOLD", ""):
        try:
            A.ModeRequests(bad)
        except ValueError:
            continue
        raise AssertionError(f"hold mode {bad!r} accepted")


def test_the_ros_node_hands_the_fsm_the_depth_and_the_caps_the_shield_applied():
    """REVIEW (2026-10-09). The ROS node's FSM wiring had no test of its own:
    passing unsafe_depth_m=None or caps=None was caught only by an unrelated
    test that was already failing. The node cannot run without rclpy here,
    so its source is checked: the FSM is given the position's illegal depth
    (X6 / X7) and the caps the Shield clamped against on that tick, and the
    row logs the depth."""
    node = (ROOT / "sitl" / "ros2_shield_node.py").read_text(encoding="utf-8")
    i = node.index("inp = tick_input_from_decision(")
    call = node[i:node.index("fsm_out = self.fsm.step(inp)", i)]
    assert "unsafe_depth_m=unsafe_depth" in call, call
    assert "caps=self.shield.applied_caps or False" in call, call
    assert "horizon_s=self.theta_horizon_s" in call, call
    j = node.index("unsafe_depth = self.shield.unsafe_depth(st, stop_illegal)")
    assert j < i
    assert "shield=self.shield, unsafe_depth=unsafe_depth))" in node
    import sys as _sys
    _sys.path.insert(0, str(ROOT / "sitl"))
    import ros2_shield_node as N
    from guardrail.fsm import DEFAULT_THETA_HORIZON_S
    assert N.default_theta_horizon() is DEFAULT_THETA_HORIZON_S is None
    from guardrail import Action4D, Shield, State
    from guardrail.fsm import EscalationFSM, TickInput
    from guardrail.models import Policy
    sh = Shield(Policy.model_validate({"policy_id": "r", "constraints": [
        {"id": "band", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 0, "alt_max_m": 30}]}),
        escalation=False)
    st, a = State(x=0.0, y=0.0, up=10.0), Action4D(vx=1.0)
    d = sh.filter(st, a)
    out = EscalationFSM().step(TickInput(t=1.0, outcome="clean"))
    row = N.build_row(now=1.0, tick=1, state=st, body=a, decision=d, raw_world=a,
                      emitted_world=a, emitted_body=a, shield_on=True, extra={},
                      subject=None, fsm_out=out, stop_illegal=[], shield=sh,
                      unsafe_depth=1.23456)
    assert row["unsafe_depth_m"] == 1.2346, row


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


_SQ = [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 10}, {"x": 0, "y": 10}]


def _timed_policy():
    from guardrail.models import Policy

    def zone(zid, dx, **kw):
        return {"id": zid, "type": "polygon_fence", **HARD_P0, **kw,
                "vertices": [{"x": v["x"] + dx, "y": v["y"]} for v in _SQ]}
    return Policy.model_validate({"policy_id": "timed", "constraints": [
        zone("yard", 0, valid_time={"recurrence": {"start_time": "07:30",
                                                   "end_time": "17:30"}}),
        zone("always", 20, valid_time={}),            # a window that is "always"
        zone("held", 40),
        {"id": "sw-off", "type": "time_window_switch", **HARD_P0, "target_id": "held",
         "active": False, "valid_time": {"recurrence": {"start_time": "08:00",
                                                        "end_time": "08:30"}}},
        zone("on", 60),
        {"id": "sw-on", "type": "time_window_switch", **HARD_P0, "target_id": "on", "active": True},
        zone("plain", 80),
    ]})


def test_a_zone_a_clock_can_turn_off_is_left_to_the_shield():
    """compiler-conform review (2026-10-08): a time_window_switch in a policy
    file now flies, and policy_fences exported every zone whatever its window
    or switch, so the autopilot's GeoFence held a zone the policy had
    switched off - stricter than the Shield, an RTL at the fence edge on a
    flight the Shield allows. A zone with a time window, or one a switch can
    turn off, is now skipped and named; one in force for the whole flight
    (no window, or a switch that only holds it ON) is still exported."""
    fences, skipped = A.policy_fences(_timed_policy(), column_m=30.0)
    assert [f["id"] for f in fences] == ["always", "on", "plain"], fences
    why = {s["id"]: s for s in skipped}
    assert set(why) == {"yard", "held"}, skipped
    assert why["yard"]["time_controlled"] and "time window" in why["yard"]["why"]
    assert why["held"]["time_controlled"] and "sw-off" in why["held"]["why"]
    assert all("stricter than the policy" in s["why"] for s in skipped)


def test_a_switch_hot_applied_mid_flight_takes_its_zone_off_the_next_export():
    """The ROS node re-exports the zones on every generation; a switch that
    arrives by hot-apply must take its target off the next upload."""
    from guardrail.models import Action4D, Policy, State, TimeWindowSwitch
    from guardrail.shield import Shield
    pol = Policy.model_validate({"policy_id": "hot", "constraints": [
        {"id": "plain", "type": "polygon_fence", **HARD_P0, "vertices": _SQ}]})
    assert [f["id"] for f in A.policy_fences(pol, column_m=30.0)[0]] == ["plain"]
    sh = Shield(pol, escalation=False)
    sh.filter(State(x=-50, y=-50, up=10), Action4D())
    sh.hot_apply(TimeWindowSwitch(id="sw", type="time_window_switch", **HARD_P0,
                                  target_id="plain", active=False))
    fences, skipped = A.policy_fences(sh.policy, column_m=30.0)
    assert fences == [] and skipped[0]["id"] == "plain" and "sw" in skipped[0]["why"]


def test_time_controls_reads_only_a_real_window_or_a_switch_off():
    by = {c.id: c for c in _timed_policy().constraints}
    assert A.time_controls(by["yard"]) == ["it has a time window (valid_time)"]
    assert A.time_controls(by["always"]) == []
    assert A.time_controls(by["plain"]) == []
    assert A.time_controls(by["held"], ["sw-off"]) == [
        "time_window_switch sw-off can switch it off"]


# --------------------------------------------------------------------------- #
# The mission style (WP3-09): MISSION_ITEM_INT through MAVROS's mission push
# --------------------------------------------------------------------------- #

def test_a_waypoint_plan_becomes_home_speed_and_one_item_per_waypoint():
    """seq 0 home (ArduPilot overwrites it), seq 1 DO_CHANGE_SPEED at the
    plan's speed, then a NAV_WAYPOINT per waypoint at relative altitude,
    projected about home exactly as the fences are (back to the plan's own
    metres through the same projection)."""
    doc = {"waypoints": [{"x": 0, "y": 30, "up": 15}, {"x": 30, "y": 30, "up": 15}],
           "speed_mps": 4.0}
    items = A.mission_items(doc, *HOME)
    assert [it["command"] for it in items] == [A.MAV_CMD_NAV_WAYPOINT, A.MAV_CMD_DO_CHANGE_SPEED,
                                               A.MAV_CMD_NAV_WAYPOINT, A.MAV_CMD_NAV_WAYPOINT]
    assert A.FIRST_WAYPOINT_SEQ == 2 and A.SPEED_ITEM_SEQ == 1
    assert items[0]["frame"] == A.MAV_FRAME_GLOBAL and items[1]["param2"] == 4.0
    proj = LocalProjection(*HOME)
    for it, (x, y) in zip(items[2:], [(0, 30), (30, 30)]):
        assert it["frame"] == A.MAV_FRAME_GLOBAL_RELATIVE_ALT and it["z_alt"] == 15.0
        assert it["param1"] == 0.0 and it["param4"] == 0.0, it
        n, e = proj.to_local(it["x_lat"], it["y_long"])
        assert abs(n - x) < 1e-6 and abs(e - y) < 1e-6, (n, e, x, y)
    for bad in ({"waypoints": [], "speed_mps": 4}, {"waypoints": doc["waypoints"]},
                {"waypoints": [{"x": 0, "y": 30, "up": 0}], "speed_mps": 4},
                {"waypoints": [{"x": float("nan"), "y": 30, "up": 15}], "speed_mps": 4}):
        try:
            A.mission_items(bad, *HOME)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad}")


def test_a_mission_update_replaces_one_waypoint_and_nothing_else():
    """The grant's "replaces or updates the next waypoint": one NAV_WAYPOINT
    at its seq, with its hold time; never the home or the speed item, never
    past the mission the autopilot holds, never before a mission exists."""
    seq, item = A.mission_update({"seq": 2, "waypoint": {"x": 0, "y": 15.5, "up": 15,
                                                         "hold_s": 600}}, 4, *HOME)
    assert seq == 2 and item["command"] == A.MAV_CMD_NAV_WAYPOINT and item["param1"] == 600.0
    for doc, n in (({"seq": 1, "waypoint": {"x": 0, "y": 1, "up": 15}}, 4),
                   ({"seq": 0, "waypoint": {"x": 0, "y": 1, "up": 15}}, 4),
                   ({"seq": 4, "waypoint": {"x": 0, "y": 1, "up": 15}}, 4),
                   ({"seq": 2, "waypoint": {"x": 0, "y": 1, "up": 15}}, None),
                   ({"seq": 2, "waypoint": {"x": 0, "y": 1, "up": 15,
                                            "hold_s": A.MAX_HOLD_S + 1}}, 4)):
        try:
            A.mission_update(doc, n, *HOME)
        except ValueError:
            continue
        raise AssertionError(f"accepted {doc} with {n} items")


def test_a_read_back_item_matches_to_the_int_protocols_resolution():
    items = A.mission_items({"waypoints": [{"x": 0, "y": 30, "up": 15}], "speed_mps": 4},
                            *HOME)
    wp = items[2]
    assert A.same_item(wp, dict(wp, x_lat=wp["x_lat"] + 1e-7))      # one int unit
    assert not A.same_item(wp, dict(wp, x_lat=wp["x_lat"] + 1e-5))  # about a metre
    assert not A.same_item(wp, dict(wp, param1=600.0))              # a hold it lacks
    assert not A.same_item(items[1], dict(items[1], param2=6.0))    # another speed
    assert A.same_item(items[0], dict(items[0], x_lat=1.0))         # home: the autopilot's


def test_an_updates_verdict_is_the_autopilots_read_back_not_mavros_push_result():
    """Flown 2026-11-03 (mission_wp_20261103c, decoded from the bag): MAVROS
    2.14 writes MISSION_WRITE_PARTIAL_LIST with end_index = start + count,
    which ArduPilot reads inclusive - it applies the item, restarts the
    command it is flying, asks for one item more and times out after 8 s,
    so MAVROS reports the push failed although the item landed. The adapter
    therefore reads the item back after any push that sent it, and THAT
    comparison is the update's `success`; a push that sent nothing fails."""
    src = (ROOT / "sitl" / "mavlink_adapter_node.py").read_text(encoding="utf-8")
    body = src[src.index("def _on_mission_update"):src.index("def close(self)")]
    assert '"push_success": ok' in body and "if not ok and n < 1:" in body
    assert 'rec["success"] = not bad' in body
    assert "set_current" not in body, "ArduCopter restarts a changed item itself"


def test_a_gcs_set_fence_has_the_operators_values_and_a_column_over_the_ceiling():
    """sitl/gcs_backstop.py (WP3-10): the GCS sets the operator's fence file,
    FENCE_ENABLE last, with FENCE_ALT_MAX raised above the policy's ceiling
    when the file's sits below it, and refuses a file that omits one."""
    import gcs_backstop as G
    parm = A.read_parm(PARM)
    pol = load_policy(POLICY)
    vals, plan = G.desired_params(parm, pol, 15.0)
    assert G.PARAM_ORDER[-1] == "FENCE_ENABLE"
    assert set(G.GRANT_PARAMS) <= set(vals)
    assert vals["FENCE_ENABLE"] == 1.0 and vals["FENCE_TYPE"] == 5.0 and vals["FENCE_ACTION"] == 1.0
    assert vals["FENCE_ALT_MAX"] == 30.0 and plan["raise_to"] is None and vals["AVOID_ENABLE"] == 0.0
    low = dict(parm, FENCE_ALT_MAX=20.0)
    vals2, plan2 = G.desired_params(low, pol, 15.0)
    assert vals2["FENCE_ALT_MAX"] == 25.0 and plan2["file_value"] == 20.0
    try:
        G.desired_params({k: v for k, v in parm.items() if k != "FENCE_ACTION"}, pol, 15.0)
    except ValueError as e:
        assert "FENCE_ACTION" in str(e)
    else:
        raise AssertionError("a fence file without FENCE_ACTION was accepted")


def test_the_gcs_compares_the_fence_it_reads_back_item_by_item():
    import gcs_backstop as G
    pol = load_policy(POLICY)
    fences, _ = A.policy_fences(pol, column_m=30.0)
    items = G.int_items(A.fence_items(fences, *HOME))
    assert len(items) == 4 and all(isinstance(it["x"], int) for it in items)
    assert G.items_match(items, [dict(it) for it in items]) == []
    assert G.items_match(items, items[:3])
    moved = [dict(it) for it in items]
    moved[2]["y"] += 50
    assert any("item 2" in p for p in G.items_match(items, moved))


def test_the_mission_shield_replaces_an_illegal_leg_by_its_stop_short_point():
    """sitl/ros2_mission_node.py: a leg is walked with the Shield's own
    position check; a legal plan goes up unchanged, a leg into a zone has its
    waypoint replaced by a held point short of the zone (its margin
    included), and a zone hot-applied across a leg makes it illegal there."""
    import ros2_mission_node as M
    from guardrail.models import DynamicNFZ
    from guardrail.replay import rail_shield
    pol = load_policy(POLICY)
    sh = rail_shield(pol, lookahead_s=5.0, dt=0.1)
    assert M.parse_waypoints("0,30; 30,30") == [(0.0, 30.0), (30.0, 30.0)]
    wps, legs = M.check_plan(sh, (0.0, 0.0, 15.0), [(0.0, 30.0, 15.0), (30.0, 30.0, 15.0)])
    assert [g["legal"] for g in legs] == [True, True]
    assert [(w["x"], w["y"], w["hold_s"]) for w in wps] == [(0.0, 30.0, 0.0), (30.0, 30.0, 0.0)]
    wps, legs = M.check_plan(sh, (0.0, 0.0, 15.0), [(15.0, 15.0, 15.0), (30.0, 30.0, 15.0)])
    assert legs[0]["legal"] is False and legs[0]["rules"] == ["nfz-square"]
    assert wps[0]["hold_s"] == M.HOLD_S and wps[0]["x"] < 6.0 and wps[0]["y"] < 6.0, wps[0]
    assert legs[1]["legal"] is None and wps[1]["hold_s"] == 0.0
    sh.hot_apply(DynamicNFZ(id="hot", type="dynamic_nfz", **HARD_P0,
                            vertices=[XY(x=-4, y=18), XY(x=4, y=18), XY(x=4, y=22),
                                      XY(x=-4, y=22)], margin_m=1.0))
    v = M.leg_check(sh, (0.0, 5.0, 15.0), (0.0, 30.0, 15.0))
    assert v["legal"] is False and v["rules"] == ["hot"] and 16.5 <= v["first_illegal"][1] <= 17.5
    p = M.stop_short((0.0, 5.0, 15.0), v)
    assert abs(p[1] - (v["last_legal"][1] - M.STANDOFF_M)) < 1e-9 and p[0] == 0.0
    assert M.stop_short((0.0, 5.0, 15.0), dict(v, last_legal=(0.0, 5.5, 15.0)))[1] == 5.0


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
