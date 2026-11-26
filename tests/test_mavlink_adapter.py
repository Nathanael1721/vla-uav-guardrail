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
import json
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
from wgs84_fixture import geo, geo_ring              # noqa: E402

# The four ConstraintBase fields the grant requires (IR schema 3.0.0, 2026-11-02;
# scope since 4.0.0, 2026-11-06), at the values they defaulted to before
# (repair = project_fix; an absent scope meant mission).
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}

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


def test_a_raised_fence_read_back_as_float32_is_not_stricter_than_the_policy():
    """msl-dem-zone-ridge (2026-11-19): ceiling 81.725 m, FENCE_ALT_MAX set to
    86.725 read back 86.7249984741211 (float32) and the run was recorded
    stricter_than_policy. The value read back is the value set."""
    need = 81.72500000000002 + A.FENCE_ALT_HEADROOM_M
    back = A.as_float32(need)
    assert back < need and back == 86.7249984741211, back
    assert A.fence_alt_plan(81.72500000000002, back)["ok"] is True
    # a fence a float32 step lower is still stricter
    low = A.as_float32(back - 1e-5)
    assert low < back and A.fence_alt_plan(81.72500000000002, low)["ok"] is False
    assert A.fence_alt_plan(81.72500000000002, 86.0)["raise_to"] == need


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
    # The pose the Shield judged: `seen` is the perceived pose under a GPS-noise
    # / latency cell and the true one otherwise (2026-11-21, WP3-19).
    j = node.index("unsafe_depth = self.shield.unsafe_depth(seen, stop_illegal)")
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
    # The upload judges the frame through fence_frame (2026-11-06), which
    # holds the tolerance (test_the_fences_go_up_about_the_home_the_shield_flies).
    assert "fence_frame(doc, lat, lon, ne)" in src[src.index("def _upload"):]
    assert "ORIGIN_TOLERANCE_M" in src[src.index("def fence_frame"):src.index("def fence_items")]


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
    sh = Shield(pol, escalation=False, legacy_metres=True)
    sh.filter(State(x=-50, y=-50, up=10), Action4D())
    sh.hot_apply(geo(TimeWindowSwitch(id="sw", type="time_window_switch", **HARD_P0,
                                  target_id="plain", active=False), sh))
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
    sh.hot_apply(geo(DynamicNFZ(id="hot", type="dynamic_nfz", **HARD_P0,
                            vertices=[XY(x=-4, y=18), XY(x=4, y=18), XY(x=4, y=22),
                                      XY(x=-4, y=22)], margin_m=1.0), sh))
    v = M.leg_check(sh, (0.0, 5.0, 15.0), (0.0, 30.0, 15.0))
    assert v["legal"] is False and v["rules"] == ["hot"] and 16.5 <= v["first_illegal"][1] <= 17.5
    p = M.stop_short((0.0, 5.0, 15.0), v)
    assert abs(p[1] - (v["last_legal"][1] - M.STANDOFF_M)) < 1e-9 and p[0] == 0.0
    assert M.stop_short((0.0, 5.0, 15.0), dict(v, last_legal=(0.0, 5.5, 15.0)))[1] == 5.0


def test_the_fences_go_up_about_the_home_the_shield_flies():
    """A stress scenario's start (2026-11-06): the Shield node flies the EKF
    origin at --start's policy-frame position and says so in the fences
    document (`home_policy_ne`). The adapter projects the zones about the
    autopilot's home from there, so each vertex lands where the policy put
    it, and refuses a home the frame origin puts anywhere else - a SITL
    booted at CMAC for a scenario that starts 200 m south. The null: a
    document from before (no home_policy_ne) is judged and placed exactly
    as it was (home at the frame origin), and a non-finite home is refused."""
    proj = LocalProjection(*HOME)
    lat, lon = proj.to_latlon(-200.0, 0.0)
    doc = {"origin": {"lat": HOME[0], "lon": HOME[1]}, "home_policy_ne": [-200.0, 0.0]}
    home_local, off, refused = A.fence_frame(doc, lat, lon)
    assert refused is None and home_local == (-200.0, 0.0), (home_local, refused)
    assert math.hypot(off[0] + 200.0, off[1]) < 1e-6, off
    verts = [(-190.0, 0.0), (-190.0, 10.0), (-180.0, 10.0)]
    items = A.fence_items([{"kind": "polygon", "id": "z", "vertices": verts}], lat, lon,
                          home_local)
    for it, (x, y) in zip(items, verts):
        # Within MISSION_ITEM_INT's 1e-7 degree: projected about home rather
        # than the frame origin, the equirectangular scale differs by under a
        # millimetre over these 200 m.
        want = proj.to_latlon(x, y)
        assert abs(it["x_lat"] - want[0]) < 1e-7 and abs(it["y_long"] - want[1]) < 1e-7, \
            (it, want)
    # Home 2 m from the EKF origin (HomePosition.position) shifts the frame.
    hl, _, r = A.fence_frame(doc, *proj.to_latlon(-198.0, 0.0), home_ne=(2.0, 0.0))
    assert r is None and hl == (-198.0, 0.0), (hl, r)
    # SITL at the frame origin instead of the start: refused, naming both.
    _, off2, wrong = A.fence_frame(doc, *HOME)
    assert wrong is not None and "200.0 m from (-200.0, 0.0) m" in wrong["why"], wrong
    assert wrong["home_policy_ne"] == [-200.0, 0.0] and wrong["origin_offset_m"] == [0.0, 0.0]
    # The null: the documents every run before sent.
    old = {"origin": {"lat": HOME[0], "lon": HOME[1]}}
    assert A.fence_frame(old, *HOME) == ((0.0, 0.0), (0.0, 0.0), None)
    _, _, far = A.fence_frame(old, lat, lon)
    assert "the policy's frame origin is 200.0 m from the autopilot's home" in far["why"], far
    assert A.fence_frame({"home_policy_ne": [-200.0, 0.0]}, *HOME)[2] is None, \
        "a metre policy (no origin) is placed by the declared home and never refused"
    assert A.fence_frame(dict(doc, home_policy_ne=[float("nan"), 0.0]), lat, lon)[2]


def test_a_negative_start_and_action_parse_as_values():
    """--start (south or west of a frame origin) and --action (a pilot flying
    south) take a negative first value as theirs, as --target does, and both
    nodes read flags whole: an abbreviation is not another node's flag."""
    assert {"--start", "--action"} <= set(A.XY_FLAGS)
    assert A.join_negative_xy(["--start", "-200,0,15", "--action", "-3,0,0,0"]) == \
        ["--start=-200,0,15", "--action=-3,0,0,0"]
    import ros2_shield_node as N
    import ros2_vla_stub_node as V
    for mk in (N.make_parser, V.make_parser):
        assert mk().allow_abbrev is False, mk
    ns, rest = N.make_parser().parse_known_args(["--action=-3,0,0,0", "--pilot=goto",
                                                 "--start", "-200,0,15"])
    assert ns.start == "-200,0,15" and rest == ["--action=-3,0,0,0", "--pilot=goto"], rest
    vs, rest = V.make_parser().parse_known_args(["--events=e.json", "--reach-m=3",
                                                 "--action", "-3,0,0,0"])
    assert vs.action == "-3,0,0,0" and rest == ["--events=e.json", "--reach-m=3"], rest


# --------------------------------------------------------------------------- #
# The in-flight mission update: a full write, one at a time (2026-11-15)
#
# What MAVROS 2.14 and ArduPilot 4.5.7 put on the wire for a push, modelled
# from the 3 Nov flights' bags (mission_wp_20261103c / _d, decoded with
# tools/bag_mission_protocol.py; CHANGELOG 2026-11-03, Found): MAVROS starts
# a full push with MISSION_COUNT and a partial one with
# MISSION_WRITE_PARTIAL_LIST(start, start + count); ArduPilot reads end_index
# as INCLUDED, asks for each item with MISSION_REQUEST, re-asks for one it is
# not sent, and gives up 8 s after the last item it received with MISSION_ACK
# 15 ("Mission upload timeout"); MAVROS answers a request only for an item it
# was given (MISSION_ITEM_INT), and reports the push failed without an
# accepted ACK. Shown failing first on 478dcc7: update_push and
# MissionPushQueue do not exist, and the node pushed start_index = seq.
# --------------------------------------------------------------------------- #

MAV_MISSION_ACCEPTED = 0
MAV_MISSION_OPERATION_CANCELLED = 15
AP_UPLOAD_TIMEOUT_S = 8.0
AP_REQUEST_RESEND_S = 0.9            # nine requests for the missing item in 8 s


def wire_exchange(held: list[dict], start_index: int, items: list[dict]) -> dict:
    """One /mavros/mission/push against the autopilot's mission `held`
    (changed in place): the messages each side sends, in order, with their
    time from the push, MAVROS's result and the ACK that ended it."""
    t, wire = 0.0, []

    def say(who, name, **f):
        wire.append({"t": round(t, 3), "from": who, "msg": name, **f})
    if start_index == 0:
        say("mavros", "MISSION_COUNT", count=len(items))
        del held[len(items):]
        first, last = 0, len(items) - 1
        serve = dict(enumerate(items))
    else:
        end = start_index + len(items)                  # MAVROS 2.14's end_index
        say("mavros", "MISSION_WRITE_PARTIAL_LIST", start_index=start_index,
            end_index=end)
        if start_index >= len(held) or end > len(held) or end < start_index:
            say("ardupilot", "MISSION_ACK", type=1)       # MAV_MISSION_ERROR
            return {"wire": wire, "success": False, "transferred": 0, "ack": 1, "t": t}
        first, last = start_index, end                  # ArduPilot: end included
        serve = {start_index + i: it for i, it in enumerate(items)}
    seq, t_rx, sent = first, 0.0, 0
    while True:
        say("ardupilot", "MISSION_REQUEST", seq=seq)
        if seq in serve:
            t += 0.01
            say("mavros", "MISSION_ITEM_INT", seq=seq)
            if seq < len(held):
                held[seq] = dict(serve[seq])
            else:
                held.append(dict(serve[seq]))
            sent, t_rx = sent + 1, t
            if seq == last:
                say("ardupilot", "MISSION_ACK", type=MAV_MISSION_ACCEPTED)
                return {"wire": wire, "success": True, "transferred": sent,
                        "ack": MAV_MISSION_ACCEPTED, "t": t}
            seq += 1
            continue
        # MAVROS refuses a request out of what it was given: nothing is sent.
        t += AP_REQUEST_RESEND_S
        if t - t_rx >= AP_UPLOAD_TIMEOUT_S:
            t = t_rx + AP_UPLOAD_TIMEOUT_S
            say("ardupilot", "MISSION_ACK", type=MAV_MISSION_OPERATION_CANCELLED)
            say("ardupilot", "STATUSTEXT", text="Mission upload timeout")
            return {"wire": wire, "success": False, "transferred": sent,
                    "ack": MAV_MISSION_OPERATION_CANCELLED, "t": t}


def _mission():
    doc = {"waypoints": [{"x": 0, "y": 30, "up": 15}, {"x": 30, "y": 30, "up": 15}],
           "speed_mps": 4.0}
    return A.mission_items(doc, *HOME)


def test_an_update_is_written_as_the_whole_mission_with_one_item_replaced():
    items = _mission()
    seq, held_pt = A.mission_update({"seq": 2, "waypoint": {"x": 0, "y": 15.1, "up": 15,
                                                            "hold_s": 600}}, 4, *HOME)
    assert A.DEFAULT_MISSION_UPDATE_WRITE == "full"
    plan = A.update_push(items, seq, held_pt)
    assert plan["write"] == "full" and plan["start_index"] == 0
    assert plan["items"][2] is held_pt and plan["items"][:2] == items[:2]
    assert plan["items"][3] == items[3] and plan["check"] == [0, 1, 2, 3]
    assert items[2] is not held_pt, "the adapter's copy is not changed by planning"
    part = A.update_push(items, seq, held_pt, "partial")
    assert part == {"write": "partial", "start_index": 2, "items": [held_pt], "check": [2]}
    for bad in ((items, 4, held_pt, "full"), ([], 2, held_pt, "full"),
                (items, 2, held_pt, "neither")):
        try:
            A.update_push(*bad)
        except ValueError:
            continue
        raise AssertionError(f"planned {bad[1:]}")


def test_the_full_write_completes_where_mavros_partial_write_times_out():
    """The message sequence. Partial (as flown 3 Nov): the item lands, then
    ArduPilot asks for one item more until its 8 s timeout, ACK 15, MAVROS
    reports failure. Full: MISSION_COUNT, every item asked for and sent as
    MISSION_ITEM_INT, ACK ACCEPTED at once - and the write back right after
    is accepted too, where the partial write's 8 s window refused one."""
    items = _mission()
    _, held_pt = A.mission_update({"seq": 2, "waypoint": {"x": 0, "y": 15.1, "up": 15,
                                                          "hold_s": 600}}, 4, *HOME)
    ap = [dict(i) for i in items]
    p = A.update_push(items, 2, held_pt, "partial")
    got = wire_exchange(ap, p["start_index"], p["items"])
    msgs = [(w["from"], w["msg"], w.get("seq", w.get("start_index"))) for w in got["wire"]]
    assert msgs[:4] == [("mavros", "MISSION_WRITE_PARTIAL_LIST", 2),
                        ("ardupilot", "MISSION_REQUEST", 2),
                        ("mavros", "MISSION_ITEM_INT", 2),
                        ("ardupilot", "MISSION_REQUEST", 3)], msgs
    assert got["wire"][0]["end_index"] == 3, "MAVROS 2.14: start + count"
    asks3 = [m for m in msgs if m == ("ardupilot", "MISSION_REQUEST", 3)]
    assert len(asks3) >= 8 and ("mavros", "MISSION_ITEM_INT", 3) not in msgs
    assert got["ack"] == MAV_MISSION_OPERATION_CANCELLED and got["t"] >= AP_UPLOAD_TIMEOUT_S
    assert got["success"] is False and A.same_item(held_pt, ap[2]), "the item landed anyway"
    ap = [dict(i) for i in items]
    f = A.update_push(items, 2, held_pt)
    got = wire_exchange(ap, f["start_index"], f["items"])
    msgs = [(w["from"], w["msg"], w.get("seq", w.get("count"))) for w in got["wire"]]
    want = [("mavros", "MISSION_COUNT", 4)]
    for s in range(4):
        want += [("ardupilot", "MISSION_REQUEST", s), ("mavros", "MISSION_ITEM_INT", s)]
    want += [("ardupilot", "MISSION_ACK", None)]
    assert msgs == want, msgs
    assert got["ack"] == MAV_MISSION_ACCEPTED and got["success"] and got["t"] < 0.1
    assert "MISSION_WRITE_PARTIAL_LIST" not in {w["msg"] for w in got["wire"]}
    assert all(A.same_item(a, b) for a, b in zip(f["items"], ap)) and len(ap) == 4
    # The write back, at once: accepted, the original flown again.
    back = A.update_push(f["items"], 2, items[2])
    again = wire_exchange(ap, back["start_index"], back["items"])
    assert again["ack"] == MAV_MISSION_ACCEPTED and A.same_item(ap[2], items[2])


def test_updates_are_written_one_at_a_time_in_order():
    q = A.MissionPushQueue()
    assert q.offer({"update_id": "u1"}) is True and q.busy
    assert q.offer({"update_id": "u2"}) is False
    assert q.offer({"update_id": "u3"}) is False and q.n_queued == 2
    assert q.done() == {"update_id": "u2"} and q.busy
    assert q.done() == {"update_id": "u3"} and q.busy
    assert q.done() is None and not q.busy
    assert q.offer({"update_id": "u4"}) is True


# A stand-in ROS stack for the adapter node (tests/test_railinputs_node.py
# builds one for the Shield node the same way).

class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _Srv:
    Request = _Msg


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Future:
    def __init__(self, result):
        self._result, self._cbs, self.done = result, [], False

    def result(self):
        return self._result

    def add_done_callback(self, cb):
        self._cbs.append(cb)
        if self.done:
            cb(self)

    def complete(self):
        self.done = True
        for cb in self._cbs:
            cb(self)


class _Timer:
    def __init__(self, cb):
        self.cb, self.cancelled = cb, False

    def cancel(self):
        self.cancelled = True


class FakeMavros:
    """/mavros/mission/push and /pull against an ArduPilot holding `held`:
    each push runs wire_exchange; futures complete when the test says (so a
    second update can arrive while the first is in flight)."""

    def __init__(self):
        self.held: list[dict] = []
        self.pushes, self.pending = [], []
        self.node = None

    def push(self, req):
        items = [{**self.node._as_dict(w), "param3": w.param3, "param4": w.param4}
                 for w in req.waypoints]
        ex = wire_exchange(self.held, int(req.start_index), items)
        self.pushes.append({"start_index": int(req.start_index), "n": len(items), **ex})
        fut = _Future(_Msg(success=ex["success"], wp_transfered=(
            ex["transferred"] if req.start_index == 0 else max(ex["transferred"], 1))))
        self.pending.append(fut)
        return fut

    def pull(self, req):
        wps = [self.node._wp({"autocontinue": True, **it}) for it in self.held]
        self.node._on_waypoints(_Msg(waypoints=wps))
        fut = _Future(_Msg(success=True, wp_received=len(wps)))
        self.pending.append(fut)
        return fut


class _Client:
    def __init__(self, fn):
        self.fn = fn

    def service_is_ready(self):
        return self.fn is not None

    def call_async(self, req):
        return self.fn(req)


def _adapter_module():
    import importlib.util
    import types
    names = ("rclpy", "rclpy.node", "rclpy.qos", "geometry_msgs", "geometry_msgs.msg",
             "std_msgs", "std_msgs.msg", "mavros_msgs", "mavros_msgs.msg",
             "mavros_msgs.srv")
    m = {n: types.ModuleType(n) for n in names}

    class Node:
        def __init__(self, name):
            self.timers, self.pubs = [], {}
            self.mav = FakeMavros()
            self.mav.node = self

        def get_logger(self):
            return types.SimpleNamespace(info=lambda s: None, warn=lambda s: None,
                                         error=lambda s: None)

        def create_subscription(self, *a, **k):
            return None

        def create_publisher(self, typ, topic, qos):
            self.pubs[topic] = _Pub()
            return self.pubs[topic]

        def create_client(self, typ, name):
            return _Client({"/mavros/mission/push": self.mav.push,
                            "/mavros/mission/pull": self.mav.pull}.get(name))

        def create_timer(self, period, cb):
            t = _Timer(cb)
            self.timers.append(t)
            return t
    m["rclpy.node"].Node = Node
    q = m["rclpy.qos"]
    q.QoSProfile = _Msg
    q.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    q.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    q.qos_profile_sensor_data = None
    for mod, ns in (("geometry_msgs.msg", ("PoseStamped",)),
                    ("std_msgs.msg", ("Float32MultiArray", "String")),
                    ("mavros_msgs.msg", ("HomePosition", "PositionTarget", "Waypoint",
                                         "WaypointList", "Mavlink", "RCIn", "State"))):
        for n in ns:
            setattr(m[mod], n, type(n, (_Msg,), {}))
    pt = m["mavros_msgs.msg"].PositionTarget
    for k in ("IGNORE_PX", "IGNORE_PY", "IGNORE_PZ", "IGNORE_AFX", "IGNORE_AFY",
              "IGNORE_AFZ", "IGNORE_YAW", "FRAME_LOCAL_NED"):
        setattr(pt, k, getattr(A, k))
    for n in ("SetMode", "WaypointPull", "WaypointPush"):
        setattr(m["mavros_msgs.srv"], n, type(n, (_Srv,), {}))
    saved = {k: sys.modules.get(k) for k in m}
    sys.modules.update(m)
    try:
        spec = importlib.util.spec_from_file_location(
            "mavlink_adapter_node_standin", ROOT / "sitl" / "mavlink_adapter_node.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    assert mod.HAVE_ROS, "the stand-in ROS stack did not import"
    return mod


def _settle(node):
    """Complete every outstanding MAVROS call and run the read-back timers,
    until nothing is left. The 1 Hz delivery beat (`_beat`, 2026-11-17)
    never ends, so it is not one of them."""
    for _ in range(200):
        moved = False
        while node.mav.pending:
            node.mav.pending.pop(0).complete()
            moved = True
        for t in list(node.timers):
            if not t.cancelled and getattr(t.cb, "__name__", "") != "_beat":
                t.cb()
                moved = True
        if not moved:
            return
    raise AssertionError("the adapter never settled")


def _statuses(M, node):
    return [json.loads(m.data) for m in node.pubs[M.TOPIC_STATUS].sent]


def test_the_adapter_node_writes_each_update_whole_and_queues_the_next():
    """The node itself, on a stand-in ROS stack: the mission goes up whole,
    an update is a full write (no MISSION_WRITE_PARTIAL_LIST on the wire, ACK
    ACCEPTED), a second update sent while the first is in flight waits and
    then goes, both verdicts are their read-backs; with
    mission_update_write="partial" the 3 Nov exchange comes back."""
    M = _adapter_module()
    for write in ("full", "partial"):
        node = M.MavlinkAdapterNode(None, mission_update_write=write)
        node._on_home(_Msg(geo=_Msg(latitude=HOME[0], longitude=HOME[1]),
                           position=_Msg(x=0.0, y=0.0, z=0.0)))
        node._on_mission(_Msg(data=json.dumps({
            "waypoints": [{"x": 0, "y": 30, "up": 15}, {"x": 30, "y": 30, "up": 15}],
            "speed_mps": 4.0, "generation": 0})))
        _settle(node)
        up = [s for s in _statuses(M, node) if s["kind"] == "mission_upload"]
        assert up and up[-1]["success"] and up[-1]["match"], up
        assert node.mission_n == 4 and len(node.mav.held) == 4
        u1 = {"seq": 2, "waypoint": {"x": 0, "y": 15.1, "up": 15, "hold_s": 600},
              "update_id": "u1", "generation": 1}
        u2 = {"seq": 3, "waypoint": {"x": 20, "y": 30, "up": 15, "hold_s": 0},
              "update_id": "u2", "generation": 1}
        node._on_mission_update(_Msg(data=json.dumps(u1)))
        node._on_mission_update(_Msg(data=json.dumps(u2)))    # while u1 is in flight
        assert len(node.mav.pushes) == 2, "u2 was pushed beside u1"
        queued = [s for s in _statuses(M, node) if s["kind"] == "mission_update_queued"]
        assert [q["update_id"] for q in queued] == ["u2"], queued
        _settle(node)
        ups = [s for s in _statuses(M, node) if s["kind"] == "mission_update"]
        assert [(s["update_id"], s["success"], s["write"]) for s in ups] == \
            [("u1", True, write), ("u2", True, write)], ups
        it1, it2 = A.mission_update(u1, 4, *HOME)[1], A.mission_update(u2, 4, *HOME)[1]
        assert A.same_item(node.mav.held[2], it1) and A.same_item(node.mav.held[3], it2)
        assert abs(ups[0]["read_back_item"]["y_long"] - it1["y_long"]) < 1e-12
        p1, p2 = node.mav.pushes[1:3]
        if write == "full":
            assert (p1["start_index"], p1["n"], p1["ack"]) == (0, 4, MAV_MISSION_ACCEPTED)
            # u2's full write carries u1's item: one write never undoes another.
            assert (p2["start_index"], p2["n"], p2["ack"]) == (0, 4, MAV_MISSION_ACCEPTED)
            assert all(s["push_success"] for s in ups) and p1["t"] < 1.0
            assert not any(w["msg"] == "MISSION_WRITE_PARTIAL_LIST"
                           for p in node.mav.pushes for w in p["wire"])
        else:
            assert (p1["start_index"], p1["n"]) == (2, 1)
            assert p1["ack"] == MAV_MISSION_OPERATION_CANCELLED and p1["t"] >= 8.0
            assert not any(s["push_success"] for s in ups), "MAVROS reports the timeout"
            assert all("upload timeout" in s["push_note"] for s in ups)


class FakeFence:
    """/mavros/geofence/push and /pull as measured 15 Nov 2026 on the AGX
    (MAVROS 2 against ArduPilot 4.5.7 SITL): a push replaces the list and
    answers success; a zero-item push clears it too, but MAVROS answers
    success=False, wp_transfered 1. `take=False`: the autopilot keeps its
    list (the clear did not happen)."""

    def __init__(self, node):
        self.node, self.held, self.take = node, [], True

    def push(self, req):
        if self.take or req.waypoints:
            self.held = list(req.waypoints)
        n = len(req.waypoints)
        fut = _Future(_Msg(success=n > 0, wp_transfered=n if n else 1))
        self.node.mav.pending.append(fut)
        return fut

    def pull(self, req):
        fut = _Future(_Msg(success=True, wp_received=len(self.held)))
        self.node.mav.pending.append(fut)
        return fut


def test_the_adapter_judges_a_fence_clear_by_its_read_back():
    """An expired zone leaves no zone to upload: the empty push clears the
    autopilot's list, and MAVROS 2 answers it with success=False. The
    verdict is the read-back (0 items), never that answer; a clear the list
    did not take stays a failure. Before 2026-11-15 every clear was recorded
    as failed (pas_hil_hotapply_01's generation 2 on the perception rail)."""
    M = _adapter_module()
    # A ROS request's sequence field starts as an empty list.
    M.WaypointPush.Request = staticmethod(lambda **kw: _Msg(**{"waypoints": [], **kw}))
    node = M.MavlinkAdapterNode(None)
    fake = FakeFence(node)
    node.cli_push, node.cli_pull = _Client(fake.push), _Client(fake.pull)
    node._on_home(_Msg(geo=_Msg(latitude=HOME[0], longitude=HOME[1]),
                       position=_Msg(x=0.0, y=0.0, z=0.0)))
    zone = {"kind": "circle", "id": "nfz-rest-1", "center": [30.0, 30.0],
            "radius_m": 5.0}
    node._on_fences(_Msg(data=json.dumps({"fences": [zone], "generation": 1})))
    _settle(node)
    node._on_fences(_Msg(data=json.dumps({"fences": [], "generation": 2})))
    _settle(node)
    fake.held = [object()]
    fake.take = False
    node._on_fences(_Msg(data=json.dumps({"fences": [], "generation": 3})))
    _settle(node)
    ups = {s["generation"]: s for s in _statuses(M, node) if s["kind"] == "fence_upload"}
    assert ups[1]["success"] and ups[1]["read_back"] == 1, ups[1]
    assert "push_success" not in ups[1], "a non-empty push is judged by both"
    assert ups[2]["success"] and ups[2]["read_back"] == 0, ups[2]
    assert ups[2]["push_success"] is False and ups[2]["transferred"] == 1, ups[2]
    assert not ups[3]["success"] and ups[3]["read_back"] == 1, ups[3]


def _ch3(msg) -> int:
    """Channel 3 of an RC_CHANNELS_OVERRIDE the node published (payload64)."""
    import struct
    raw = b"".join(struct.pack("<Q", w) for w in msg.payload64)
    return struct.unpack("<H", raw[4:6])[0]


def test_the_adapter_node_centres_the_throttle_around_its_loiter_request():
    """The node on a stand-in ROS stack (THE LITERAL LOITER, 2026-11-21). By
    default nothing changes: LOITER is sent as BRAKE and nothing is published
    on MAVROS's router sink. With --hold-mode LOITER --loiter-throttle centre
    the FSM's LOITER goes out as LOITER, the override (channel 3 = 1500, system
    255) is published BEFORE the mode request is made, the timer re-sends it,
    and the next request for anything else hands channel 3 back (0) and
    reports what the autopilot read; shutting down releases a hold left on."""
    M = _adapter_module()
    plain = M.MavlinkAdapterNode(None)
    plain._on_mode_request(_Msg(data="LOITER"))
    assert plain.throttle is None and M.TOPIC_MAVLINK_SINK not in plain.pubs
    st = _statuses(M, plain)
    assert [s["kind"] for s in st] == ["mode_request"] and st[0]["autopilot_mode"] == "BRAKE"
    try:
        M.MavlinkAdapterNode(None, hold_mode="BRAKE", loiter_throttle="centre")
    except ValueError:
        pass
    else:
        raise AssertionError("a centred throttle was accepted with BRAKE")

    node = M.MavlinkAdapterNode(None, hold_mode="LOITER", loiter_throttle="centre")
    cfg = _statuses(M, node)[0]
    assert cfg["kind"] == "hold_config" and cfg["loiter_throttle"] == "centre", cfg
    sink = node.pubs[M.TOPIC_MAVLINK_SINK].sent
    assert sink == []
    node._on_mode_request(_Msg(data="LOITER"))
    st = _statuses(M, node)
    assert [s["kind"] for s in st[-2:]] == ["rc_override", "mode_request"], st
    assert st[-2]["action"] == "hold" and st[-1]["autopilot_mode"] == "LOITER"
    assert len(sink) == 1 and (sink[0].sysid, sink[0].compid, sink[0].msgid) == (255, 191, 70)
    assert _ch3(sink[0]) == 1500 and sink[0].checksum == A.rc_override_frame(1500, 0)["checksum"]
    (beat,) = [t for t in node.timers if t.cb == node.throttle.beat]
    beat.cb()
    assert len(sink) == 2 and _ch3(sink[1]) == 1500
    node._on_rc_in(_Msg(channels=[1500, 1500, 1500, 1500]))
    node._on_state(_Msg(mode="LOITER"))
    node._on_mode_request(_Msg(data="GUIDED"))
    st = _statuses(M, node)
    rel = [s for s in st if s["kind"] == "rc_override"][-1]
    assert rel["action"] == "release" and "GUIDED requested" in rel["why"], rel
    assert rel["rc3_at_pwm"] == 1 and rel["autopilot_reported_loiter"] is True
    assert [_ch3(m) for m in sink[2:]] == [0] * A.RC_OVERRIDE_RELEASES
    beat.cb()
    assert len(sink) == 2 + A.RC_OVERRIDE_RELEASES, "nothing is sent once released"
    node._on_mode_request(_Msg(data="LOITER"))
    node.close()
    assert _ch3(sink[-1]) == 0 and not node.throttle.active
    assert "the adapter stopped" in _statuses(M, node)[-1]["why"]


def test_the_mission_update_flag_defaults_to_the_full_write():
    src = (ROOT / "sitl" / "mavlink_adapter_node.py").read_text(encoding="utf-8")
    main = src[src.index("def main("):]
    assert "--mission-update-write" in main and "DEFAULT_MISSION_UPDATE_WRITE" in main
    assert A.MISSION_UPDATE_WRITES == ("full", "partial")


# --------------------------------------------------------------------------- #
# The perception rail's setpoint through the adapter (2026-11-15, opt-in)
# --------------------------------------------------------------------------- #

def _pas_link(setpoint_via, subscribers=1, publishers=1):
    for p in (ROOT / "demo" / "pas_ardupilot", ROOT / "demo", ROOT / "tests"):
        if str(p) not in sys.path:
            sys.path.insert(0, str(p))
    import mavros_link as ML
    import test_pas_ardupilot as T

    class Node(T.FakeNode):
        def count_subscribers(self, topic):
            return subscribers

        def count_publishers(self, topic):
            return publishers
    node = Node()
    link = ML.MavrosLink(node=node, rclpy=T.FAKE_RCLPY, msgs=T.FAKE_MSGS,
                         clock=T.Clock(0.01), setpoint_via=setpoint_via)
    return ML, link, node


def test_the_pas_setpoint_through_the_adapter_reaches_the_same_ned():
    """--setpoint-via adapter: the PAS link publishes the BODY action on the
    adapter's input and no PositionTarget of its own; the adapter's
    conversion at the same heading gives exactly the NED setpoint the direct
    path sends (enu_position_target, through MAVROS's ENU -> NED), for every
    heading: both are guardrail.frames.body_to_local_ned of the same body
    action (since wf21 the link sends only the body frame)."""
    from guardrail.models import Action4D
    ML, direct, dnode = _pas_link("mavros")
    assert ML.TOPIC_SETPOINT in dnode.pubs and ML.TOPIC_ADAPTER_SETPOINT not in dnode.pubs
    assert direct.setpoint_path().startswith(ML.TOPIC_SETPOINT)
    ML, link, node = _pas_link("adapter")
    assert ML.TOPIC_ADAPTER_SETPOINT == A.TOPIC_SETPOINT
    assert ML.TOPIC_SETPOINT not in node.pubs, "the adapter is the only PositionTarget writer"
    assert ML.TOPIC_ADAPTER_SETPOINT in node.pubs
    a = Action4D(vx=2.5, vy=-1.25, vz_up=0.4, yaw_rate=0.3)
    sent_body = (2.5, -1.25, 0.4, 0.3)          # what the rail streams (body frame)
    for heading in range(0, 360, 30):
        link.snap.yaw = direct.snap.yaw = math.radians(heading)
        link.send(a, raw_body=(1, 2, 3, 4), body=sent_body)
        body = node.pubs[ML.TOPIC_ADAPTER_SETPOINT].msgs[-1].data
        assert list(body) == list(sent_body), "the adapter route publishes the body action as is"
        via_adapter = A.mavros_sends(A.setpoint_from_body(body, heading)["fields"])
        direct.send(a, body=sent_body)
        pt = dnode.pubs[ML.TOPIC_SETPOINT].msgs[-1]
        f = {"coordinate_frame": pt.coordinate_frame, "type_mask": pt.type_mask,
             "velocity": (pt.velocity.x, pt.velocity.y, pt.velocity.z),
             "yaw_rate": pt.yaw_rate}
        assert _close(via_adapter, A.mavros_sends(f), 1e-9), (heading, via_adapter)
    assert node.pubs[ML.TOPIC_SAFE_ACTION].msgs[-1].data == [2.5, -1.25, 0.4, 0.3]
    assert link.evidence()["setpoint_path"].startswith(ML.TOPIC_ADAPTER_SETPOINT)


def test_the_pas_adapter_route_refuses_to_arm_without_one_adapter_and_one_publisher():
    for subs, pubs, ok, why in ((1, 1, True, None), (0, 1, False, "nothing subscribes"),
                                (1, 2, False, "2 publishers")):
        ML, link, node = _pas_link("adapter", subs, pubs)
        got = link.check_adapter(wait_s=0.05)
        assert got["ok"] is ok and (why is None or why in got["why"]), got
    try:
        ML.MavrosLink(node=node, rclpy=None, msgs=None, setpoint_via="both")
    except ValueError:
        pass
    else:
        raise AssertionError("an unknown setpoint route was accepted")
    import perception_node as pn
    args = pn.build_parser().parse_args(["--setpoint-via", "adapter"])
    try:
        pn.check_args(args)
    except SystemExit as e:
        assert "--link mavros" in str(e)
    else:
        raise AssertionError("--setpoint-via adapter was accepted with --link pymavlink")
    assert pn.build_parser().parse_args([]).setpoint_via == "mavros"


# --------------------------------------------------------------------------- #
# THE LITERAL LOITER (2026-11-21, WP3-07)
# --------------------------------------------------------------------------- #

# pymavlink's bytes for the same frames (pymavlink 2.4, MAVLINK20=1,
# srcSystem 255, srcComponent 191, seq 7, target 1/1, chan1-8 UINT16_MAX but
# chan3, chan9-18 0), written down so the test needs no pymavlink.
PYMAVLINK_RC3_1500 = "fd12000007ffbf460000ffffffffdc05ffffffffffffffffffff01018d5e"
PYMAVLINK_RC3_RELEASE = "fd12000007ffbf460000ffffffff0000ffffffffffffffffffff010136ac"


def test_the_throttle_override_frame_is_the_one_pymavlink_builds():
    """The adapter builds RC_CHANNELS_OVERRIDE itself (it is published raw on
    MAVROS's router sink, as the ground station's system id): its bytes, CRC
    included, must be MAVLink's, or ArduPilot drops every frame and LOITER
    sinks. Checked against pymavlink's frames, and against pymavlink itself
    for other sequence numbers when it is installed."""
    f = A.rc_override_frame(1500, 7)
    assert f["frame"].hex() == PYMAVLINK_RC3_1500, f["frame"].hex()
    assert A.rc_override_frame(A.RC_OVERRIDE_RELEASE, 7)["frame"].hex() == PYMAVLINK_RC3_RELEASE
    assert (f["sysid"], f["compid"], f["msgid"], f["len"]) == (255, 191, 70, 18)
    assert f["magic"] == 0xFD and f["framing_status"] == 1
    # payload64 carries exactly the payload, in little-endian 64-bit words
    import struct
    raw = b"".join(struct.pack("<Q", w) for w in f["payload64"])[:f["len"]]
    assert raw == f["frame"][10:10 + f["len"]]
    assert int.from_bytes(f["frame"][-2:], "little") == f["checksum"]
    assert A.rc_override_frame(1500, 300)["seq"] == 300 & 0xFF
    try:
        import os
        os.environ.setdefault("MAVLINK20", "1")
        from pymavlink.dialects.v20 import common as m
    except Exception:                                     # noqa: BLE001
        return
    mav = m.MAVLink(None, srcSystem=255, srcComponent=191)
    U = A.RC_OVERRIDE_IGNORE
    for seq, pwm in ((0, 1500), (200, 1500), (255, 0), (12, 1234)):
        mav.seq = seq
        want = m.MAVLink_rc_channels_override_message(
            1, 1, U, U, pwm, U, U, U, U, U, *([0] * 10)).pack(mav)
        assert A.rc_override_frame(pwm, seq)["frame"] == want, (seq, pwm)


def test_the_adapter_flies_brake_unless_the_literal_loiter_is_asked():
    """Default unchanged: BRAKE, no override. VLAGUARD_ADAPTER_HOLD=loiter
    (how --rail-hold loiter reaches the adapter on both rails) means LOITER
    with the throttle centred; the command line wins where it is given; a
    centred throttle with BRAKE, or an unknown value, is refused - main()
    exits before ROS starts."""
    assert A.adapter_hold_config(None, None, {}) == ("BRAKE", "off")
    assert A.adapter_hold_config(None, None) == ("BRAKE", "off")
    env = {A.ADAPTER_HOLD_ENV: "loiter"}
    assert A.adapter_hold_config(None, None, env) == ("LOITER", "centre")
    assert A.adapter_hold_config(None, None, {A.ADAPTER_HOLD_ENV: " Brake "}) == ("BRAKE", "off")
    assert A.adapter_hold_config("LOITER", None, {}) == ("LOITER", "off")
    assert A.adapter_hold_config("LOITER", "off", env) == ("LOITER", "off")
    for bad in (("BRAKE", "centre", {}), ("BRAKE", None, env), (None, "centre", {}),
                (None, None, {A.ADAPTER_HOLD_ENV: "sideways"}), ("HOVER", None, {})):
        try:
            A.adapter_hold_config(*bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad}")
    ns = A.build_parser().parse_args([])
    assert (ns.hold_mode, ns.loiter_throttle, ns.rc_sysid) == (None, None, 255)
    import os
    saved = os.environ.get(A.ADAPTER_HOLD_ENV)
    try:
        os.environ[A.ADAPTER_HOLD_ENV] = "sideways"
        try:
            A.main([])
        except SystemExit as e:
            assert A.ADAPTER_HOLD_ENV in str(e), e
        else:
            raise AssertionError("main() started with an unknown hold")
    finally:
        if saved is None:
            os.environ.pop(A.ADAPTER_HOLD_ENV, None)
        else:
            os.environ[A.ADAPTER_HOLD_ENV] = saved
    assert A.ADAPTER_HOLD_ENV == "VLAGUARD_ADAPTER_HOLD"


def _frames_sent():
    sent = []
    return sent, sent.append


def test_the_throttle_is_held_centred_only_while_loiter_is_requested():
    """hold() sends channel 3 at 1500 as system 255 at once (before SET_MODE
    LOITER goes out), beat() re-sends it while held and nothing otherwise;
    release() hands channel 3 back (0) three times and says how long it was
    held and what the autopilot read; the autopilot leaving LOITER after it
    was in it ends the hold, a GUIDED report before it ever got there does
    not."""
    sent, send = _frames_sent()
    th = A.ThrottleStandIn(send)
    th.beat()
    assert sent == [] and th.release(1.0, "x") is None, "nothing held, nothing sent"
    st = th.hold(10.0)
    assert st["kind"] == "rc_override" and st["action"] == "hold" and st["pwm"] == 1500
    assert st["sysid"] == 255 and len(sent) == 1
    assert sent[0]["frame"] == A.rc_override_frame(1500, 0)["frame"]
    assert th.hold(10.1) is None and len(sent) == 2, "a second LOITER request re-sends"
    th.beat()
    th.beat()
    assert len(sent) == 4 and [f["seq"] for f in sent] == [0, 1, 2, 3]
    assert th.on_mode("GUIDED", 10.2) is None and th.active, "not in LOITER yet"
    th.on_rc_in([1500, 1500, 1500, 1500])
    assert th.on_mode("LOITER", 10.4) is None and th.seen_loiter
    th.on_rc_in([1500, 1500, 1497, 1500])
    th.on_rc_in([1500, 1500, 1100])
    rel = th.on_mode("RTL", 13.0)
    assert rel["action"] == "release" and "left LOITER for RTL" in rel["why"]
    assert rel["held_s"] == 3.0 and rel["autopilot_reported_loiter"] is True
    assert (rel["rc3_reports"], rel["rc3_at_pwm"]) == (3, 2), rel
    releases = sent[4:]
    assert len(releases) == A.RC_OVERRIDE_RELEASES
    assert all(r["frame"][10 + 4:10 + 6] == b"\x00\x00" for r in releases), "ch3 = 0"
    th.beat()
    assert len(sent) == 4 + A.RC_OVERRIDE_RELEASES, "released: nothing more is sent"
    # A new hold, ended by the adapter's own next request.
    th.hold(20.0)
    assert th.release(21.5, "GUIDED requested")["held_s"] == 1.5
    s = th.summary()
    assert [sp["t_end"] for sp in s["spells"]] == [13.0, 21.5] and s["sysid"] == 255


def _rows(spell, before=3, after=2, t0=10.0, dt=0.1):
    """Shield-node rows around one FSM Loiter: `spell` is [(mode, up), ...]."""
    rows, t = [], t0 - before * dt
    for _ in range(before):
        rows.append({"t": round(t, 3), "fsm_state_after": "Normal", "mode": "GUIDED",
                     "up": 15.0})
        t += dt
    for mode, up in spell:
        rows.append({"t": round(t, 3), "fsm_state_after": "Loiter", "mode": mode, "up": up})
        t += dt
    for _ in range(after):
        rows.append({"t": round(t, 3), "fsm_state_after": "Normal", "mode": "LOITER",
                     "up": 15.0})
        t += dt
    return rows


HOLD = [{"kind": "rc_override", "action": "hold"}]


def test_the_literal_loiter_check_passes_only_a_held_loiter():
    """The post-mission check behind metrics.json literal_loiter: a Loiter the
    heartbeat confirmed as LOITER that held its altitude passes; one that
    sank as uncentred LOITER does (1.5 m/s), flew BRAKE (an adapter on its
    default), was asked and never confirmed, or had no throttle stand-in
    fails; no Loiter, or too little LOITER to judge, is not judged - and
    only a pass is counted."""
    held = [("GUIDED", 15.0)] * 3 + [("LOITER", 15.0 + 0.1 * ((i % 3) - 1))
                                     for i in range(30)]
    ok = A.literal_loiter_check(_rows(held), [10.05], HOLD)
    assert ok["verdict"] == "pass" and ok["counted"] is True, ok
    assert ok["loiter_spells"] == 1 and abs(ok["loiter_held_s"] - 2.9) < 1e-6, ok
    assert ok["spells"][0]["alt_dev_max_m"] <= 0.2 + 1e-9 and ok["spells"][0]["asks"] == 1
    sank = [("GUIDED", 15.0)] * 3 + [("LOITER", 15.0 - 0.15 * i) for i in range(30)]
    bad = A.literal_loiter_check(_rows(sank), [10.05], HOLD)
    assert bad["verdict"] == "fail" and not bad["counted"]
    assert "altitude wandered" in bad["problems"][0], bad
    brake = [("GUIDED", 15.0)] * 3 + [("BRAKE", 15.0)] * 30
    bad = A.literal_loiter_check(_rows(brake), [10.05], HOLD)
    assert bad["verdict"] == "fail" and "['BRAKE']" in bad["problems"][0], bad
    never = [("GUIDED", 15.0)] * 33
    bad = A.literal_loiter_check(_rows(never), [10.05], HOLD)
    assert bad["verdict"] == "fail" and "never reported" in bad["problems"][0], bad
    bad = A.literal_loiter_check(_rows(held), [10.05], [])
    assert bad["verdict"] == "fail" and "no throttle stand-in" in bad["problems"][0], bad
    # Fail closed on a row with no altitude.
    gap = held[:10] + [("LOITER", None)] + held[11:]
    assert A.literal_loiter_check(_rows(gap), [10.05], HOLD)["verdict"] == "fail"
    # Nothing to judge: no Loiter; a request too late in a short Loiter to be
    # confirmed; LOITER shorter than LITERAL_LOITER_MIN_HELD_S.
    none = A.literal_loiter_check(_rows([]), [], [])
    assert none["verdict"] == "not_judged" and none["loiter_spells"] == 0
    late = A.literal_loiter_check(_rows([("GUIDED", 15.0)] * 10), [10.5], HOLD)
    assert late["verdict"] == "not_judged", late
    short = [("GUIDED", 15.0)] * 3 + [("LOITER", 15.0)] * 10
    assert A.literal_loiter_check(_rows(short), [10.05], HOLD)["verdict"] == "not_judged"
    # A re-plan: LOITER, a GUIDED transit to a point 2 m up, LOITER again -
    # each stretch judged from where its LOITER began.
    replan = ([("GUIDED", 15.0)] * 2 + [("LOITER", 15.0)] * 16
              + [("GUIDED", 15.0 + 0.4 * i) for i in range(5)] + [("LOITER", 17.0)] * 16)
    got = A.literal_loiter_check(_rows(replan), [10.05, 12.4], HOLD)
    assert got["verdict"] == "pass" and abs(got["loiter_held_s"] - 3.0) < 1e-6, got
    # Over a DEM the row's up is above the ground; the hold is judged on up_home.
    dem = [dict(r, up=r["up"] + (3.0 if i % 2 else 0.0), up_home=15.0)
           for i, r in enumerate(_rows(held))]
    assert A.literal_loiter_check(dem, [10.05], HOLD)["verdict"] == "pass"
    r = ok["rules"]
    assert (r["alt_tol_m"], r["min_held_s"]) == (A.LITERAL_LOITER_ALT_TOL_M,
                                                 A.LITERAL_LOITER_MIN_HELD_S)


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
