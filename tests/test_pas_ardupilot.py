"""The Project AirSim + ArduPilot rail (demo/pas_ardupilot/), checked offline.

Run either way:
    pytest tests/test_pas_ardupilot.py -v
    python tests/test_pas_ardupilot.py

WHY THIS FILE EXISTS

The rail joins three systems this suite cannot start: Unreal + Project AirSim,
ArduPilot SITL in WSL, and MAVROS 2. So every decision that can be made without
them is made in pure functions, and those are tested here with stand-ins for
pymavlink, rclpy and projectairsim:

  * the WSL2 addresses each side must use (a loopback address under NAT drops
    every UDP datagram without an error, so the plan refuses it);
  * the robot config ArduPilot is given (a simple_flight controller, a swapped
    motor pair or a disabled GPS each fly wrong or not at all);
  * the frame conversions (a sign error turns the aircraft the wrong way);
  * the heartbeat filter (Mission Planner's heartbeat must not become the
    command target), and a TCP connect that is retried while the router is
    still starting;
  * the Shield and the escalation FSM sitting between the pilot and the
    autopilot (only their output may reach ArduPilot, and a persistent P0
    violation must reach a mode request);
  * the autopilot acting on its own (a fence RTL must stop the node's
    commands and be recorded, never pass in silence);
  * the topology label and the KPI grade (no run may claim dev / hil / flight
    or KPI grade without the evidence, G1 and a measured real-time clock);
  * gate G1's four evaluators, each fed the broken run it must reject, and the
    run set: a crashed run is a failed run, never a missing one.

Tests that need a file outside this repository (IAMAI's example, the client's
JSON schema, the stored CityLife trajectories) print SKIP when it is absent,
never PASS.
"""
import ast
import collections
import glob
import json
import math
import os
import re
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
PAS = ROOT / "demo" / "pas_ardupilot"
for _p in (ROOT, ROOT / "demo", PAS):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import rail                                                          # noqa: E402
import topology as topo                                              # noqa: E402
import autopilot as ap                                               # noqa: E402
import mavros_link as ml                                             # noqa: E402
import perception_node as pn                                         # noqa: E402
import g1_check as g1                                                # noqa: E402
from guardrail.manifest import (TOPOLOGY_DEV, TOPOLOGY_FLIGHT,       # noqa: E402
                                TOPOLOGY_HIL, TOPOLOGY_PROJECTAIRSIM,
                                build_manifest, check_hil_evidence,
                                is_kpi_grade, sim_speedup_from_scene)
from guardrail.models import Action4D, State                          # noqa: E402

SKIP = "SKIP"
IAMAI = Path("D:/ProjectAirSim/repo/client/python/example_user_scripts/ardupilot")
WIN, WSL = "172.20.0.1", "172.20.5.9"


def _raises(fn, *a, contains: str = "", **k) -> str:
    try:
        fn(*a, **k)
    except (ValueError, RuntimeError) as exc:
        assert contains.lower() in str(exc).lower(), f"wrong reason: {exc}"
        return str(exc)
    raise AssertionError(f"{fn.__name__} accepted {a} {k}")


def _tmp() -> Path:
    return Path(tempfile.mkdtemp(prefix="pas_ap_"))


# =========================================================================== #
# rail.py - configs, addresses, command lines
# =========================================================================== #

def test_jsonc_strips_comments_but_keeps_paths_inside_strings():
    txt = ('{"mesh": "/Drone/Quadrotor1", "url": "a//b", // trailing\n'
           ' /* block */ "n": 1, "s": "say \\"//\\" here"}')
    d = json.loads(rail.strip_jsonc(txt))
    assert d == {"mesh": "/Drone/Quadrotor1", "url": "a//b", "n": 1,
                 "s": 'say "//" here'}, d


def test_the_templates_pass_the_robot_check():
    robot = rail.load_jsonc(rail.ROBOT_TEMPLATE)
    assert rail.check_robot_config(robot) == [], rail.check_robot_config(robot)
    scene = rail.load_jsonc(rail.SCENE_TEMPLATE)
    assert rail.home_geo_point(scene) == (35.6895, 139.6917, 40.0)


def test_the_robot_check_refuses_what_ardupilot_could_not_fly():
    # 1. The simple_flight config every CityLife flight used until now.
    sf = rail.load_jsonc(ROOT / "demo" / "pas_config" / "robot_semantic_quad.jsonc")
    assert any("simple-flight-api" in p for p in rail.check_robot_config(sf))
    good = rail.load_jsonc(rail.ROBOT_TEMPLATE)
    # 2. Two motors swapped.
    bad = json.loads(json.dumps(good))
    order = bad["controller"]["ardupilot-settings"]["actuator-order"]
    order[0], order[2] = order[2], order[0]
    assert any("actuator-order" in p for p in rail.check_robot_config(bad))
    # 3. GPS off (the simple_flight config had it off), and 4. the IMU off.
    for kind, word in (("gps", "GPS"), ("imu", "IMU")):
        bad = json.loads(json.dumps(good))
        for s in bad["sensors"]:
            if s["type"] == kind:
                s["enabled"] = False
        assert any(word in p for p in rail.check_robot_config(bad)), kind
    # 5. A rotor spinning the wrong way.
    bad = json.loads(json.dumps(good))
    for a in bad["actuators"]:
        if a["name"] == "Prop_FR_actuator":
            a["rotor-settings"]["turning-direction"] = "clock-wise"
    assert any("Prop_FR_actuator" in p for p in rail.check_robot_config(bad))
    # 6. A non-default port: SITL is started on 9003/9002.
    bad = json.loads(json.dumps(good))
    bad["controller"]["ardupilot-settings"]["ardupilot-udp-port"] = 9013
    assert any("9003" in p for p in rail.check_robot_config(bad))


def test_the_airframe_is_iamai_s_ardupilot_airframe():
    """The rate-loop tuning is copied from IAMAI's example; that is only
    justified if the airframe it was tuned on is the airframe we fly."""
    ref = IAMAI / "sim_config" / "robot_ardu_quadrotor.jsonc"
    if not ref.is_file():
        return SKIP
    ours, theirs = rail.load_jsonc(rail.ROBOT_TEMPLATE), rail.load_jsonc(ref)
    for key in ("physics-type", "links", "joints", "actuators"):
        assert ours[key] == theirs[key], f"{key} differs from IAMAI's airframe"
    assert ours["controller"]["ardupilot-settings"]["actuator-order"] == \
        theirs["controller"]["ardupilot-settings"]["actuator-order"]


def _params(p):
    out = {}
    for ln in Path(p).read_text(encoding="utf-8").splitlines():
        f = ln.split("#", 1)[0].split()
        if len(f) == 2:
            out[f[0]] = float(f[1])
    return out


def test_the_param_file_carries_iamai_s_tuning_verbatim():
    ref = IAMAI / "project-airsim-quad.param"
    if not ref.is_file():
        return SKIP
    theirs, ours = _params(ref), _params(rail.PARAM_FILE)
    assert theirs, "parsed nothing from IAMAI's file"
    for k, v in theirs.items():
        assert ours.get(k) == v, f"{k}: ours {ours.get(k)} vs IAMAI {v}"
    # ...and the envelope lines that keep AUTO/RTL inside the policy's band.
    assert ours["WPNAV_SPEED"] == 500 and ours["RTL_ALT"] == 1200
    assert ours["FENCE_ENABLE"] == 1 and ours["FENCE_ALT_MAX"] == 20


# The largest horizontal distance from the start point over the stored
# CityLife follow trajectories, measured 2026-10-07 (citylife_redcar_final2).
CITYLIFE_MAX_RANGE_M = 253.7


def test_the_autopilot_fence_leaves_room_for_the_flights_citylife_actually_flies():
    """The autopilot fence is the backstop the Shield must pre-empt. A radius
    inside the range real CityLife follows reach would fire RTL mid-follow.
    The first version said 250 m on the strength of an unsourced '190 m';
    the stored flights reach 253.7 m."""
    radius = _params(rail.PARAM_FILE)["FENCE_RADIUS"]
    assert radius >= 1.25 * CITYLIFE_MAX_RANGE_M, radius
    files = glob.glob(str(ROOT / "demo" / "out" / "citylife*" / "trajectory.json"))
    if not files:
        return SKIP
    worst = 0.0
    for f in files:
        try:
            tr = json.loads(Path(f).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        pts = [(r["x"], r["y"]) for r in tr if isinstance(r, dict) and "x" in r] \
            if isinstance(tr, list) else []
        if pts:
            worst = max(worst, max(math.hypot(x - pts[0][0], y - pts[0][1]) for x, y in pts))
    assert worst <= CITYLIFE_MAX_RANGE_M + 0.05, \
        f"a stored flight reaches {worst:.1f} m: re-size FENCE_RADIUS and this constant"


def test_nat_puts_each_address_on_the_side_that_can_reach_it():
    p = rail.plan_network("nat", WIN, WSL)
    assert p.sim_bind_ip == WIN, "the simulator must bind the address WSL sends PWM to"
    assert p.ardupilot_ip == WSL, "sensor frames must go to WSL's own address"
    assert p.sitl_sim_address == WIN and p.gcs_address == WIN


def test_nat_refuses_addresses_that_would_drop_udp_silently():
    _raises(rail.plan_network, "nat", "127.0.0.1", WSL, contains="TCP only")
    _raises(rail.plan_network, "nat", WIN, "127.0.0.1", contains="loopback")
    _raises(rail.plan_network, "nat", "0.0.0.0", WSL, contains="wildcard")
    _raises(rail.plan_network, "nat", WIN, WIN, contains="cannot be equal")
    _raises(rail.plan_network, "nat", WIN, None, contains="required")
    _raises(rail.plan_network, "nat", "172.20.0.300", WSL, contains="not an IPv4")
    _raises(rail.plan_network, "bridged", WIN, WSL, contains="expected")


def test_mirrored_is_loopback_on_both_sides():
    p = rail.plan_network("mirrored", WIN, WSL)
    assert (p.sim_bind_ip, p.ardupilot_ip, p.gcs_address) == ("127.0.0.1",) * 3


def test_wslconfig_mode_reads_only_an_uncommented_wsl2_setting():
    assert rail.wslconfig_mode("[wsl2]\n# networkingMode=mirrored\n") == "nat"
    assert rail.wslconfig_mode("[wsl2]\nnetworkingMode = mirrored\n") == "mirrored"
    assert rail.wslconfig_mode("[experimental]\nnetworkingMode=mirrored\n") == "nat"
    assert rail.wslconfig_mode(None) == "nat"


def test_rendered_configs_carry_this_run_s_addresses_and_leave_the_template():
    before = rail.ROBOT_TEMPLATE.read_bytes()
    out = _tmp()
    cfg = rail.write_sim_configs(out, rail.plan_network("nat", WIN, WSL))
    robot = json.loads((out / cfg["robot"]).read_text(encoding="utf-8"))
    st = robot["controller"]["ardupilot-settings"]
    assert (st["ardupilot-ip"], st["local-host-ip"]) == (WSL, WIN)
    scene = json.loads((out / cfg["scene"]).read_text(encoding="utf-8"))
    assert scene["actors"][0]["robot-config"] == cfg["robot"]
    assert json.loads((out / "network.json").read_text())["mode"] == "nat"
    assert rail.ROBOT_TEMPLATE.read_bytes() == before, "the template was modified"
    assert cfg["controller_type"] == "ardupilot-api"
    assert cfg["scene_id"] == "SceneCityLifeArduPilot"


def test_attach_configs_record_no_invented_addresses():
    out = _tmp()
    cfg = rail.write_attach_configs(out)
    assert cfg["network"]["mode"] == "attach" and "loaded by another host" in cfg["network"]["note"]
    assert json.loads((out / "network.json").read_text())["mode"] == "attach"
    assert cfg["scene_id"] == "SceneCityLifeArduPilot" and cfg["controller_type"] == "ardupilot-api"


def test_rendered_configs_pass_project_airsim_s_own_loader_and_schema():
    """The client validates every robot config against its JSON schema before
    sending it. Run that loader offline - and show it CAN refuse."""
    try:
        from projectairsim.utils import load_scene_config_as_dict
    except Exception:                                         # noqa: BLE001
        return SKIP
    out = _tmp()
    cfg = rail.write_sim_configs(out, rail.plan_network("nat", WIN, WSL))
    data, _ = load_scene_config_as_dict(cfg["scene"], str(out))
    assert data["actors"][0]["robot-config"]["controller"]["type"] == "ardupilot-api"
    # The node's own inlining (attach mode) gives the loader's shape.
    mine = pn.inline_scene(cfg)
    assert mine["actors"][0]["robot-config"]["sensors"] == \
        data["actors"][0]["robot-config"]["sensors"]
    robot = json.loads((out / cfg["robot"]).read_text(encoding="utf-8"))
    robot["controller"]["type"] = "bogus-api"
    (out / cfg["robot"]).write_text(json.dumps(robot), encoding="utf-8")
    try:
        load_scene_config_as_dict(cfg["scene"], str(out))
    except Exception as exc:                                  # noqa: BLE001
        assert "bogus-api" in str(exc)
    else:
        raise AssertionError("the client's schema accepted a bogus controller")


def test_the_scene_clock_is_real_time_by_the_manifest_s_own_derivation():
    out = _tmp()
    cfg = rail.write_sim_configs(out, rail.plan_network("mirrored"))
    assert cfg["clock_ratio"] == 1.0
    assert sim_speedup_from_scene(out / cfg["scene"]) == 1.0


def test_sitl_args_are_the_airsim_copter_launch_with_our_params_last():
    scene = rail.load_jsonc(rail.SCENE_TEMPLATE)
    a = rail.sitl_args(scene, rail.plan_network("nat", WIN, WSL), ["/x/citylife-quad.param"])
    kv = {a[i]: a[i + 1] for i in range(1, len(a) - 1) if a[i].startswith("--")}
    assert kv["--model"] == "airsim-copter"
    assert kv["--sim-address"] == WIN
    assert (kv["--sim-port-in"], kv["--sim-port-out"]) == ("9003", "9002")
    assert kv["--home"] == "35.6895,139.6917,40.0,0.0", "home must be the scene's"
    d = kv["--defaults"].split(",")
    assert [Path(x).name for x in d] == ["copter.parm", "airsim-quadX.parm",
                                         "citylife-quad.param"], d
    # Stored parameters are wiped, or an eeprom.bin from an earlier session
    # (a value edited in Mission Planner) would override --defaults.
    assert "-w" in a
    assert "-w" not in rail.sitl_args(scene, rail.plan_network("mirrored"), wipe=False)
    eq = rail.sim_vehicle_equivalent(scene, rail.plan_network("mirrored"), "/p")
    assert "-f airsim-copter" in eq and "--custom-location=35.6895" in eq


def test_the_router_fans_out_without_colliding_with_sitl():
    conf = rail.router_conf(WIN, WIN, orin_address="192.168.1.50")
    assert "TcpServerPort = 5790" in conf and "TcpServerPort = 5760" not in conf
    blocks = conf.split("\n\n")

    def block(name):
        return next(b for b in blocks if name in b)
    assert "Port = 5760" in block("[TcpEndpoint sitl]")
    assert f"Address = {WIN}" in block("missionplanner") and "Port = 14550" in block("missionplanner")
    assert "Address = 127.0.0.1" in block("[UdpEndpoint mavros]")
    assert "192.168.1.50" in block("[UdpEndpoint orin]") and "Port = 14555" in block("orin")
    try:
        rail.router_conf("172.20.0")
    except ValueError:
        pass
    else:
        raise AssertionError("a malformed address reached the router config")


def test_the_mavproxy_fallback_serves_the_node_and_mission_planner_separately():
    """A MAVProxy tcpin output accepts ONE client: with one shared port the
    node and Mission Planner locked each other out. And without
    --streamrate=-1 MAVProxy rewrites the SR0_* rates the param file sets."""
    mp = rail.mavproxy_router_args(WIN, WIN)
    tcp = sorted(o for o in mp if o.startswith("--out=tcpin:"))
    assert tcp == [f"--out=tcpin:0.0.0.0:{rail.ROUTER_TCP_PORT}",
                   f"--out=tcpin:0.0.0.0:{rail.GCS_TCP_FALLBACK_PORT}"], tcp
    assert rail.ROUTER_TCP_PORT != rail.GCS_TCP_FALLBACK_PORT
    assert f"--out=udp:{WIN}:14550" in mp and "--streamrate=-1" in mp
    sh = (PAS / "start_ardupilot.sh").read_text(encoding="utf-8")
    assert str(rail.GCS_TCP_FALLBACK_PORT) in sh, "the script must tell the user which TCP port"


def test_windows_paths_map_into_wsl():
    assert rail.to_wsl_path(r"D:\OneDrive\Lab\VLA Drone\x.sh") == "/mnt/d/OneDrive/Lab/VLA Drone/x.sh"
    assert rail.to_wsl_path("/home/n/x") == "/home/n/x"
    cmd = rail.wsl_start_command("Ubuntu", rail.plan_network("nat", WIN, WSL),
                                 repo=r"D:\R", mavros=True)
    assert cmd[:5] == ["wsl.exe", "-d", "Ubuntu", "--", "bash"]
    assert cmd[5] == "/mnt/d/R/demo/pas_ardupilot/start_ardupilot.sh"
    assert cmd[cmd.index("--sim-address") + 1] == WIN and "--mavros" in cmd


def test_the_wsl_script_is_lf_and_takes_its_arguments_from_rail():
    raw = (PAS / "start_ardupilot.sh").read_bytes()
    assert b"\r\n" not in raw, "CRLF breaks bash in WSL"
    src = raw.decode()
    assert "rail.py\" sitl-args" in src and "rail.py\" router-conf" in src


# =========================================================================== #
# autopilot.py - conversions and the pymavlink link
# =========================================================================== #

def test_body_to_world_rotates_by_the_autopilot_heading():
    a = ap.body_to_world(2.0, 0.0, 0.5, 0.3, 0.0)
    assert (round(a.vx, 9), round(a.vy, 9)) == (2.0, 0.0)
    a = ap.body_to_world(2.0, 0.0, 0.5, 0.3, math.pi / 2)    # facing East
    assert abs(a.vx) < 1e-9 and abs(a.vy - 2.0) < 1e-9
    a = ap.body_to_world(0.0, 1.0, 0.0, 0.0, 0.0)            # right = East at heading 0
    assert abs(a.vx) < 1e-9 and abs(a.vy - 1.0) < 1e-9
    assert (a.vz_up, ap.body_to_world(0, 0, 0.5, 0.3, 1.0).yaw_rate) == (0.0, 0.3)


def test_ned_setpoint_negates_up_and_passes_yaw_rate_in_radians():
    f = ap.ned_setpoint(Action4D(vx=1.0, vy=-2.0, vz_up=0.5, yaw_rate=0.2))
    assert (f["vx"], f["vy"], f["vz"], f["yaw_rate"]) == (1.0, -2.0, -0.5, 0.2)
    assert (f["coordinate_frame"], f["type_mask"]) == (1, 1479)


def test_the_rail_crosses_frames_through_the_project_s_one_frame_module():
    """guardrail.frames is 'the one place a body-frame action becomes a
    world-frame one'; tests/test_frame_contract.py pins it against the
    reference. This rail must agree with it at every heading, so the two
    can never drift apart in silence."""
    from guardrail.frames import from_body, to_local_ned
    for hdg in (-170.0, -90.0, -33.0, 0.0, 47.0, 90.0, 135.0, 180.0):
        for body in ((2.0, 0.0, 0.5, 0.3), (0.0, 1.5, -0.4, -0.2), (1.0, -1.0, 0.0, 0.7)):
            ours = ap.body_to_world(*body, math.radians(hdg))
            ref = from_body(*body, hdg)
            assert all(abs(getattr(ours, k) - getattr(ref, k)) < 1e-9
                       for k in ("vx", "vy", "vz_up", "yaw_rate")), (hdg, body)
            f = ap.ned_setpoint(ours)
            assert (f["vx"], f["vy"], f["vz"], f["yaw_rate"]) == to_local_ned(ours)


def test_the_type_mask_is_the_sitl_rail_s():
    tree = ast.parse((ROOT / "sitl" / "run_sitl_demo.py").read_text(encoding="utf-8"))
    vals = [ast.literal_eval(n.value) for n in ast.walk(tree)
            if isinstance(n, ast.Assign) and any(getattr(t, "id", "") == "VEL_YAWRATE_MASK"
                                                for t in n.targets)]
    assert vals == [ap.VEL_YAWRATE_MASK], vals


def test_enu_position_target_swaps_axes_and_flips_yaw_rate():
    f = ap.enu_position_target(Action4D(vx=3.0, vy=1.0, vz_up=0.5, yaw_rate=0.2))
    assert f["velocity"] == (1.0, 3.0, 0.5), "ENU is (East, North, Up)"
    assert f["yaw_rate"] == -0.2, "ENU yaw rate is counter-clockwise positive"


def test_ekf_ready_needs_every_position_flag_and_no_constant_position_mode():
    assert ap.ekf_ready(ap.EKF_READY_MASK)
    assert not ap.ekf_ready(ap.EKF_READY_MASK & ~ap.EKF_POS_HORIZ_ABS)
    assert not ap.ekf_ready(ap.EKF_READY_MASK | ap.EKF_CONST_POS_MODE)
    assert not ap.ekf_ready(None)


def test_the_frame_anchor_prefers_the_policy_origin_and_keeps_north_as_x():
    pol = SimpleNamespace(origin=SimpleNamespace(lat=25.0, lon=121.5))
    a = ap.FrameAnchor.choose(pol, (35.6895, 139.6917, 40.0))
    assert a.source == "policy.origin"
    a = ap.FrameAnchor.choose(SimpleNamespace(origin=None), (35.6895, 139.6917, 40.0))
    assert a.source == "scene.home-geo-point"
    assert ap.FrameAnchor.choose(None, None) is None
    lat, lon = a.to_latlon(10.0, 0.0)
    assert lat > 35.6895 and abs(lon - 139.6917) < 1e-12, "x must be North"
    x, y = a.to_scene(*a.to_latlon(12.5, -7.25))
    assert abs(x - 12.5) < 1e-6 and abs(y + 7.25) < 1e-6


class Msg:
    def __init__(self, kind, src=1, comp=1, **f):
        self._k, self._s, self._c = kind, src, comp
        self.__dict__.update(f)

    def get_type(self):
        return self._k

    def get_srcSystem(self):
        return self._s

    def get_srcComponent(self):
        return self._c


class FakeMav:
    def __init__(self, respond=None):
        self.sent, self.respond = [], respond

    def __getattr__(self, name):
        if name.endswith("_send"):
            def send(*a, **k):
                self.sent.append((name, a, k))
                if self.respond:
                    self.respond(name, a)
            return send
        raise AttributeError(name)


# What ArduPilot Copter-4.5.7 at 2a3dc4b7 reports in AUTOPILOT_VERSION.
AP_VERSION = Msg("AUTOPILOT_VERSION", flight_sw_version=(4 << 24) | (5 << 16) | (7 << 8) | 255,
                 flight_custom_version=list(b"2a3dc4b7"))


class FakeMaster:
    """Answers parameter reads from `params` and AUTOPILOT_VERSION requests
    when `version` is set, the way a vehicle answers - and nothing else."""

    def __init__(self, inbox, params=None, version=None):
        self.mav, self.inbox = FakeMav(self._respond), list(inbox)
        self.params, self.version = params or {}, version

    def _respond(self, name, a):
        if name == "param_request_read_send":
            pid = a[2].decode() if isinstance(a[2], bytes) else a[2]
            if pid in self.params:
                self.inbox.append(Msg("PARAM_VALUE", param_id=pid,
                                      param_value=self.params[pid]))
        elif name == "command_long_send" and a[2] == 512 and a[4] == 148 and self.version:
            self.inbox.append(self.version)

    def recv_match(self, type=None, blocking=False, timeout=None):
        # As pymavlink does: messages are read in arrival order, and one that
        # does not match `type` is DISCARDED, not kept for a later call.
        while self.inbox:
            m = self.inbox.pop(0)
            if type is None or m.get_type() == type:
                return m
        return None


class FakeMavutil:
    """mavlink_connection() as pymavlink 2.4.49 behaves over TCP while the
    router's port is still closed: it raises ConnectionRefusedError
    (`refuse` times) before it opens."""

    def __init__(self, inbox, refuse=0, **kw):
        self.master = FakeMaster(inbox, **kw)
        self.refuse, self.calls = refuse, 0

    def mavlink_connection(self, url, **kw):
        self.kw = kw
        self.calls += 1
        if self.calls <= self.refuse:
            raise ConnectionRefusedError(10061, "No connection could be made")
        return self.master


class Clock:
    def __init__(self, step=0.0):
        self.t, self.step = 1000.0, step

    def __call__(self):
        self.t += self.step
        return self.t

    def sleep(self, dt):
        self.t += max(0.0, dt)


GCS_HB = Msg("HEARTBEAT", src=255, comp=190, autopilot=8, type=6, base_mode=0, custom_mode=0)
AP_HB = Msg("HEARTBEAT", src=1, comp=1, autopilot=3, type=2, base_mode=0, custom_mode=4)


def _link(fm, step=0.01):
    clk = Clock(step)
    return ap.PymavlinkLink("tcp:x", mavutil=fm, clock=clk, sleep=clk.sleep)


def test_mission_planner_s_heartbeat_is_never_the_command_target():
    """With Mission Planner open, its heartbeat can arrive first. pymavlink's
    wait_heartbeat() would latch it; connect() must not."""
    fm = FakeMavutil([GCS_HB, AP_HB])
    link = _link(fm)
    link.connect(timeout=5)
    assert (link.target_system, link.target_component) == (1, 1)
    assert link.ignored_heartbeats == 1
    assert link.snap.mode == "GUIDED"
    assert fm.kw == {"source_system": 1, "source_component": 191, "retries": 0}
    cmds = [s for s in fm.master.mav.sent if s[0] == "command_long_send"]
    assert cmds and all(c[1][0] == 1 for c in cmds), "commands must address system 1"


def test_connect_retries_a_refused_tcp_connect_until_the_router_is_up():
    """The router's TCP port opens 20+ s after `wsl.exe start_ardupilot.sh`
    (pin check 18.5 s, SITL, router). pymavlink gives up after 3 refused
    connects, so one call crashed every --start-sitl run; connect() must keep
    trying until its own deadline."""
    fm = FakeMavutil([AP_HB], refuse=25)
    link = _link(fm, step=0.0)
    link.connect(timeout=180)
    assert fm.calls == 26 and link.connect_attempts == 26
    assert link.target_system == 1
    assert link.evidence()["connect_attempts"] == 26


def test_connect_gives_up_at_its_deadline_and_says_why():
    fm = FakeMavutil([AP_HB], refuse=10 ** 6)
    link = _link(fm, step=0.0)
    try:
        link.connect(timeout=30)
    except TimeoutError as exc:
        assert "ConnectionRefusedError" in str(exc) and "attempt" in str(exc), exc
    else:
        raise AssertionError("connected to a port that never opened")
    assert 25 <= fm.calls <= 31, fm.calls


def test_connect_times_out_rather_than_adopting_a_ground_station():
    link = _link(FakeMavutil([GCS_HB]), step=0.5)
    try:
        link.connect(timeout=3)
    except TimeoutError as exc:
        assert "1 other" in str(exc)
    else:
        raise AssertionError("connected to a GCS heartbeat")


def test_the_link_reads_only_its_vehicle_and_projects_into_the_scene_frame():
    anchor = ap.FrameAnchor(35.6895, 139.6917, 40.0, "scene")
    lat, lon = anchor.to_latlon(35.0, -20.0)
    gpi = dict(lat=int(round(lat * 1e7)), lon=int(round(lon * 1e7)), alt=48000,
               relative_alt=8000, vx=100, vy=0, vz=0)
    fm = FakeMavutil([AP_HB])
    link = _link(fm)
    link.connect(timeout=5)
    # The ground station's position comes AFTER the vehicle's, so only a
    # filter that drops it keeps the vehicle's (the old order let the
    # vehicle's overwrite it, and the test passed with no filter at all).
    fm.master.inbox = [Msg("GLOBAL_POSITION_INT", src=1, **gpi),
                       Msg("GLOBAL_POSITION_INT", src=255, **{**gpi, "relative_alt": 99000}),
                       Msg("ATTITUDE", src=1, roll=0.0, pitch=0.0, yaw=math.pi / 2),
                       Msg("ATTITUDE", src=255, roll=0.0, pitch=0.0, yaw=0.0),
                       Msg("HEARTBEAT", src=255, comp=190, autopilot=8, type=6,
                           base_mode=128, custom_mode=0)]
    link.pump()
    st = link.state(anchor)
    assert abs(st.x - 35.0) < 0.02 and abs(st.y + 20.0) < 0.02, (st.x, st.y)
    assert st.up == 8.0, "a GCS's position must never overwrite the vehicle's"
    assert abs(st.yaw_deg - 90.0) < 1e-6
    assert link.snap.armed is False, "a GCS heartbeat must not change armed state"


def test_the_link_records_fence_breaches_and_requests_modes_without_waiting():
    fm = FakeMavutil([AP_HB])
    link = _link(fm)
    link.connect(timeout=5)
    assert link.snap.fence_breach_count is None, "unmeasured, not zero"
    fm.master.inbox = [Msg("FENCE_STATUS", breach_status=0, breach_count=0, breach_type=0),
                       Msg("FENCE_STATUS", breach_status=1, breach_count=1, breach_type=2)]
    link.pump()
    assert link.snap.fence_breach_count == 1
    assert any(e["kind"] == "fence_breach" for e in link.events)
    n = len(fm.master.mav.sent)
    t = link.clock.t
    assert link.request_mode("RTL") is True
    sent = fm.master.mav.sent[n:]
    assert [s[0] for s in sent] == ["command_long_send"] and sent[0][1][2] == 176
    assert sent[0][1][5] == ap.COPTER_MODES["RTL"]
    assert link.clock.t - t < 1.0, "a mode request from the 10 Hz loop must not block"
    assert ap.STREAMS["FENCE_STATUS"][0] == 162


def test_the_autopilot_names_itself_and_shows_sitl_or_hardware():
    """ARCH-31's rule on this rail too: the firmware that flew is read from
    the autopilot, and SITL is told from hardware by SIM_SPEEDUP."""
    fm = FakeMavutil([AP_HB], params={"SIM_SPEEDUP": 1.0, "SYSID_THISMAV": 1.0},
                     version=AP_VERSION)
    link = _link(fm)
    link.connect(timeout=5)
    link.identify(timeout=1)
    ev = link.evidence()
    assert ev["topology_evidence"]["ardupilot_version"] == "ArduCopter V4.5.7 (2a3dc4b7)"
    assert (link.autopilot_kind, link.sim_speedup) == ("sitl", 1.0)
    assert check_hil_evidence(ev["topology_evidence"]), \
        "a pymavlink link has no MAVROS chain and must not pass its check"
    hw = FakeMavutil([AP_HB], params={"SYSID_THISMAV": 1.0}, version=AP_VERSION)
    link = _link(hw, step=0.05)
    link.connect(timeout=5)
    link.identify(timeout=1)
    assert (link.autopilot_kind, link.sim_speedup) == ("hardware", None)
    mute = _link(FakeMavutil([AP_HB]), step=0.05)
    mute.connect(timeout=5)
    mute.identify(timeout=1)
    assert mute.autopilot_kind is None and mute.evidence()["topology_evidence"][
        "ardupilot_version"] is None, "an unread version must stay None"


def test_a_parameter_read_keeps_what_the_autopilot_says_meanwhile():
    """pymavlink's typed recv_match discards what it skips. A parameter read
    that waited with type="PARAM_VALUE" lost a mode change and a STATUSTEXT
    arriving during the read - the autopilot's own actions, unrecorded."""
    fm = FakeMavutil([AP_HB], params={"FENCE_ALT_MAX": 20.0})
    link = _link(fm)
    link.connect(timeout=5)
    fm.master.inbox = [Msg("STATUSTEXT", text="Fence Breached"),
                       Msg("HEARTBEAT", src=1, comp=1, autopilot=3, type=2,
                           base_mode=128, custom_mode=6),
                       Msg("PARAM_VALUE", src=255, param_id="FENCE_ALT_MAX", param_value=99.0)]
    assert link.read_param("FENCE_ALT_MAX", timeout=1) == 20.0, \
        "a ground station's PARAM_VALUE is not the vehicle's"
    assert "Fence Breached" in link.snap.statustext
    assert link.snap.mode == "RTL" and link.snap.armed is True
    assert any(e["kind"] == "mode" and e["mode"] == "RTL" for e in link.events)


def test_the_autopilot_fence_is_read_from_the_autopilot_and_unanswered_is_none():
    fm = FakeMavutil([AP_HB], params={"FENCE_ENABLE": 1.0, "FENCE_ALT_MAX": 20.0,
                                      "FENCE_RADIUS": 400.0, "FENCE_ACTION": 1.0})
    link = _link(fm, step=0.05)
    link.connect(timeout=5)
    f = link.autopilot_fence()
    assert (f["FENCE_ENABLE"], f["FENCE_ALT_MAX"], f["FENCE_RADIUS"]) == (1.0, 20.0, 400.0)
    assert f["FENCE_TYPE"] is None and f["AVOID_ENABLE"] is None, "unanswered is None"
    assert set(ap.FENCE_PARAMS) <= set(f)


def test_send_emits_ned_velocity_to_the_vehicle():
    fm = FakeMavutil([AP_HB])
    link = _link(fm)
    link.connect(timeout=5)
    link.send(Action4D(vx=1.0, vy=2.0, vz_up=0.5, yaw_rate=0.1))
    name, a, _ = fm.master.mav.sent[-1]
    assert name == "set_position_target_local_ned_send"
    assert a[1:5] == (1, 1, 1, 1479), a[:5]
    assert a[8:11] == (1.0, 2.0, -0.5) and a[15] == 0.1, a


# =========================================================================== #
# mavros_link.py
# =========================================================================== #

class FakePub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class FakeNode:
    def __init__(self):
        self.subs, self.pubs, self.clients = {}, {}, {}

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = cb

    def create_publisher(self, typ, topic, qos):
        self.pubs[topic] = FakePub()
        return self.pubs[topic]

    def create_client(self, typ, name):
        self.clients[name] = FakeClient(name)
        return self.clients[name]


class FakeClient:
    """A ROS 2 service client whose service exists only once `response` is set."""

    def __init__(self, name):
        self.name, self.response, self.requests = name, None, []

    def wait_for_service(self, timeout_sec=None):
        return self.response is not None

    def call_async(self, req):
        self.requests.append(req)
        resp = self.response(req) if callable(self.response) else self.response
        return SimpleNamespace(result=lambda: resp)


class FakePositionTarget:
    def __init__(self):
        self.coordinate_frame = self.type_mask = None
        self.velocity = SimpleNamespace(x=None, y=None, z=None)
        self.yaw_rate = None


class FakeArray:
    def __init__(self):
        self.data = []


_SRV = SimpleNamespace(Request=lambda **k: SimpleNamespace(**k))
FAKE_MSGS = SimpleNamespace(
    PositionTarget=FakePositionTarget, Float32MultiArray=FakeArray, MavState=object,
    NavSatFix=object, Float64=object, PoseStamped=object, RCOut=object, Image=object,
    ExtendedState=object, StatusText=object, TimeReference=object,
    SetMode=_SRV, CommandBool=_SRV, CommandTOL=_SRV, StreamRate=_SRV,
    GetParameters=_SRV, VehicleInfoGet=_SRV, qos_sensor=None)
FAKE_RCLPY = SimpleNamespace(ok=lambda: True, spin_once=lambda node, timeout_sec=0: None,
                             spin_until_future_complete=lambda node, fut, timeout_sec=None: None)


def _param(value):
    """rcl_interfaces ParameterValue: type 0 unset, 3 double-ish, 4 string."""
    if value is None:
        return SimpleNamespace(type=0, double_value=0.0, integer_value=0, string_value="")
    if isinstance(value, str):
        return SimpleNamespace(type=4, double_value=0.0, integer_value=0, string_value=value)
    # MAVROS's own quirk: type INTEGER with the value in double_value.
    return SimpleNamespace(type=2, double_value=float(value), integer_value=0, string_value="")


def _ros_params(table):
    return lambda req: SimpleNamespace(values=[_param(table.get(n)) for n in req.names])


def _mavros():
    node = FakeNode()
    return ml.MavrosLink(node=node, rclpy=FAKE_RCLPY, msgs=FAKE_MSGS, clock=Clock(0.01)), node


def test_mavros_gets_a_position_target_in_enu_and_record_copies_off_the_grant_chain():
    link, node = _mavros()
    link.send(Action4D(vx=3.0, vy=1.0, vz_up=0.5, yaw_rate=0.2), raw_body=(4.0, 0, 0.5, 0.2))
    sp = node.pubs[ml.TOPIC_SETPOINT].msgs[-1]
    assert (sp.velocity.x, sp.velocity.y, sp.velocity.z) == (1.0, 3.0, 0.5)
    assert sp.yaw_rate == -0.2 and (sp.coordinate_frame, sp.type_mask) == (1, 1479)
    assert node.pubs[ml.TOPIC_SAFE_ACTION].msgs[-1].data == [3.0, 1.0, 0.5, 0.2]
    assert node.pubs[ml.TOPIC_RAW_ACTION].msgs[-1].data == [4.0, 0.0, 0.5, 0.2]


def test_the_record_copies_never_feed_the_ros2_shield_chain():
    """/vla/action_4d is sitl/ros2_shield_node.py's input. A raw action
    published there by this node would be filtered by a second Shield, and
    two Shields would command one aircraft."""
    chain = set()
    for f in ("sitl/ros2_shield_node.py", "sitl/mavlink_adapter_node.py",
              "sitl/ros2_vla_stub_node.py"):
        chain |= set(re.findall(r'"(/[A-Za-z0-9_/]+)"',
                                (ROOT / f).read_text(encoding="utf-8")))
    assert "/vla/action_4d" in chain, "the chain's input topic was not found: test is blind"
    for t in (ml.TOPIC_RAW_ACTION, ml.TOPIC_SAFE_ACTION):
        assert t not in chain, f"{t} is a topic of the grant's ROS 2 chain"
    link, node = _mavros()
    assert set(node.pubs) == {ml.TOPIC_SETPOINT, ml.TOPIC_RAW_ACTION, ml.TOPIC_SAFE_ACTION}


def test_mavros_evidence_passes_the_manifest_check_only_when_everything_is_live():
    """The MAVROS link records the keys sitl/ros2_shield_node.py records, and
    the manifest's own check_hil_evidence judges them."""
    old = os.environ.get("ROS_DISTRO")
    os.environ["ROS_DISTRO"] = "jazzy"
    try:
        link, node = _mavros()
        assert check_hil_evidence(link.evidence()["topology_evidence"]), "nothing seen yet"
        node.subs["/mavros/state"](SimpleNamespace(connected=False, armed=False, mode="STABILIZE"))
        assert check_hil_evidence(link.evidence()["topology_evidence"]), "FCU not connected"
        node.subs["/mavros/state"](SimpleNamespace(connected=True, armed=False, mode="GUIDED"))
        miss = check_hil_evidence(link.evidence()["topology_evidence"])
        assert any("ardupilot_version" in m for m in miss), "the firmware must be named"
        node.clients["/mavros/vehicle_info_get"].response = SimpleNamespace(
            success=True, vehicles=[SimpleNamespace(
                available_info=2, flight_sw_version=AP_VERSION.flight_sw_version,
                flight_custom_version=AP_VERSION.flight_custom_version,
                type=2, autopilot=3)])
        node.clients["/mavros/param/get_parameters"].response = _ros_params(
            {"SIM_SPEEDUP": 1.0, "SYSID_THISMAV": 1.0})
        node.clients["/mavros/get_parameters"].response = _ros_params({"fcu_url": "udp://:14555@"})
        link.identify(timeout=1)
        ev = link.evidence()
        tev = ev["topology_evidence"]
        assert check_hil_evidence(tev) == [] and ev["is_ardupilot"], check_hil_evidence(tev)
        assert tev["ardupilot_version"] == "ArduCopter V4.5.7 (2a3dc4b7)"
        assert (tev["autopilot_kind"], link.sim_speedup) == ("sitl", 1.0)
        assert tev["network_link"]["peer_is_loopback"] is True
        node.subs["/mavros/state"](SimpleNamespace(connected=True, armed=False, mode="OFFBOARD"))
        assert not link.evidence()["is_ardupilot"], "PX4's mode names are not ArduPilot's"
    finally:
        if old is None:
            os.environ.pop("ROS_DISTRO", None)
        else:
            os.environ["ROS_DISTRO"] = old


def test_mavros_never_calls_an_autopilot_hardware_on_silence():
    """No answer from the parameter service proves nothing; 'hardware' needs
    SYSID_THISMAV answered AND SIM_SPEEDUP answered as unset."""
    link, node = _mavros()
    assert link.read_sim_speedup(timeout=0.2) is None
    assert link.hil_evidence["autopilot_kind"] is None
    node.clients["/mavros/param/get_parameters"].response = _ros_params(
        {"SYSID_THISMAV": 1.0, "SIM_SPEEDUP": None})
    assert link.read_sim_speedup(timeout=0.2) is None
    assert link.hil_evidence["autopilot_kind"] == "hardware"


def test_mavros_reads_the_whole_fence_not_the_first_parameter_to_appear():
    """MAVROS fills /mavros/param while it pulls the parameter list; the ROS 2
    rail once read FENCE_ENABLE before FENCE_ALT_MAX existed. The read waits
    for all of them, and what never answers stays None."""
    link, node = _mavros()
    calls = {"n": 0}
    full = {"FENCE_ENABLE": 1.0, "FENCE_TYPE": 3.0, "FENCE_ACTION": 1.0,
            "FENCE_ALT_MAX": 20.0, "FENCE_RADIUS": 400.0, "FENCE_MARGIN": 2.0,
            "AVOID_ENABLE": 7.0}

    def filling(req):
        calls["n"] += 1
        table = {"FENCE_ENABLE": 1.0} if calls["n"] < 3 else full
        return SimpleNamespace(values=[_param(table.get(n)) for n in req.names])
    node.clients["/mavros/param/get_parameters"].response = filling
    f = link.autopilot_fence(timeout=5)
    assert calls["n"] >= 3 and f["FENCE_ALT_MAX"] == 20.0 and f["FENCE_RADIUS"] == 400.0, f
    link2, node2 = _mavros()
    node2.clients["/mavros/param/get_parameters"].response = _ros_params({"FENCE_ENABLE": 1.0})
    f2 = link2.autopilot_fence(timeout=0.3)
    assert f2["FENCE_ENABLE"] == 1.0 and f2["FENCE_ALT_MAX"] is None


def test_mavros_records_the_autopilot_s_own_clock_text_and_landing():
    link, node = _mavros()
    assert link.snap.time_boot_ms is None
    node.subs["/mavros/time_reference"](SimpleNamespace(
        time_ref=SimpleNamespace(sec=1_790_000_000, nanosec=250_000_000)))
    assert link.snap.time_boot_ms == 1_790_000_000_250
    assert "time_reference" in link.evidence()["fcu_clock_source"]
    node.subs["/mavros/statustext/recv"](SimpleNamespace(text="Fence breached", severity=2))
    assert any(e["kind"] == "statustext" and "Fence" in e["text"] for e in link.events)
    node.subs["/mavros/extended_state"](SimpleNamespace(landed_state=1))
    assert link.snap.landed_state == 1
    assert link.snap.fence_breach_count is None, "MAVROS has no FENCE_STATUS: unmeasured"


def test_mavros_mode_requests_do_not_wait_and_say_when_nobody_listens():
    link, node = _mavros()
    assert link.request_mode("RTL") is False
    assert link.events[-1]["via"].startswith("nobody")
    node.clients["/mavros/set_mode"].response = SimpleNamespace(mode_sent=True)
    assert link.request_mode("LOITER") is True
    assert node.clients["/mavros/set_mode"].requests[-1].custom_mode == "LOITER"
    try:
        link.request_mode("OFFBOARD")
    except ValueError:
        pass
    else:
        raise AssertionError("a PX4 mode was requested from ArduPilot")


def test_mavros_orientation_becomes_a_ned_heading():
    def q(yaw_enu):
        return SimpleNamespace(w=math.cos(yaw_enu / 2), x=0.0, y=0.0, z=math.sin(yaw_enu / 2))
    assert abs(ml.enu_quat_to_ned_yaw(q(0.0)) - math.pi / 2) < 1e-9, "ENU East = NED 90 deg"
    assert abs(ml.enu_quat_to_ned_yaw(q(math.pi / 2))) < 1e-9, "ENU North = NED 0"


def test_the_ros2_camera_converts_rgb_and_refuses_what_it_cannot_decode():
    hdr = SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=5))
    rgb = SimpleNamespace(encoding="rgb8", width=2, height=1, header=hdr,
                          data=bytes([255, 0, 0, 0, 0, 255]))
    d = ml.Ros2ImageSource.to_frame_dict(rgb)
    assert d["data"] == bytes([0, 0, 255, 255, 0, 0]) and d["time_stamp"] == 2_000_000_005
    _raises(ml.Ros2ImageSource.to_frame_dict,
            SimpleNamespace(encoding="mono8", width=1, height=1, header=hdr, data=b"\0"),
            contains="mono8")


# =========================================================================== #
# topology.py
# =========================================================================== #

DESKTOP = {"host_arch": "AMD64", "device_model": None, "l4t_release": None, "jetpack": None}
ORIN = {"host_arch": "aarch64", "device_model": "NVIDIA Jetson AGX Orin Developer Kit",
        "l4t_release": "R36.3.0", "jetpack": "6.0"}
LOCAL_LINK = {"fcu_url": "udp://:14555@", "peer": None, "peer_is_loopback": True}
REMOTE_LINK = {"fcu_url": "udp://:14555@192.168.1.10:14555", "peer": "192.168.1.10",
               "peer_is_loopback": False, "iface": "eth0", "link_kind": "ethernet"}


def _ev(host=DESKTOP, kind="sitl", net=LOCAL_LINK, fcu=True, **over):
    tev = {"ros_distro": "jazzy", "mavros_node": "/mavros", "fcu_connected": fcu,
           "ardupilot_version": "ArduCopter V4.5.7 (2a3dc4b7)",
           "autopilot_kind": kind, "network_link": net, **host}
    ev = {"camera_source": "projectairsim", "controller_type": "ardupilot-api",
          "sim_same_desktop": True, "link": "mavros",
          "autopilot": {"heartbeat": True, "is_ardupilot": True},
          "topology_evidence": tev, "g1": {"passed": True}}
    ev.update(over)
    return ev


def test_a_simple_flight_run_is_never_labelled_an_ardupilot_run():
    c = topo.classify(_ev(controller_type="simple-flight-api"))
    assert (c["stamp"], c["grant"]) == (TOPOLOGY_PROJECTAIRSIM, None)


def test_pymavlink_is_real_ardupilot_but_not_a_grant_topology():
    c = topo.classify(_ev(link="pymavlink"))
    assert c["stamp"] == topo.TOPOLOGY_PAS_ARDUPILOT_PYMAVLINK and c["grant"] is None
    assert any("MAVROS 2" in r for r in c["reasons"])


def test_dev_needs_live_mavros_sitl_and_one_desktop():
    assert topo.classify(_ev())["grant"] == TOPOLOGY_DEV
    assert topo.classify(_ev(fcu=False))["grant"] is None
    assert topo.classify(_ev(sim_same_desktop=False))["grant"] is None
    assert topo.classify(_ev(kind=None))["grant"] is None
    assert topo.classify(_ev(autopilot={"heartbeat": False, "is_ardupilot": False}))["grant"] is None
    bad = _ev()
    bad["topology_evidence"]["ardupilot_version"] = None
    assert topo.classify(bad)["grant"] is None, "an unnamed firmware is not dev"


def test_hil_needs_the_orin_s_own_evidence_and_the_simulator_elsewhere():
    assert topo.classify(_ev(host=ORIN, net=REMOTE_LINK,
                             sim_same_desktop=False))["grant"] == TOPOLOGY_HIL
    assert topo.classify(_ev(host=ORIN, net=REMOTE_LINK,
                             sim_same_desktop=True))["grant"] is None
    c = topo.classify(_ev(host=ORIN, net=LOCAL_LINK, sim_same_desktop=False))
    assert c["grant"] is None and any("remote desktop" in r for r in c["reasons"])
    fake_orin = {**ORIN, "device_model": "Raspberry Pi 5"}
    assert topo.classify(_ev(host=fake_orin, net=REMOTE_LINK,
                             sim_same_desktop=False))["grant"] is None


def test_flight_needs_a_hardware_autopilot_and_no_simulator():
    c = topo.classify(_ev(host=ORIN, kind="hardware", camera_source="ros2",
                          controller_type=None, sim_same_desktop=None))
    assert c["grant"] == TOPOLOGY_FLIGHT and c["stamp"] == topo.TOPOLOGY_ARDUPILOT_NO_SIM
    c = topo.classify(_ev(host=ORIN, kind="hardware"))
    assert c["grant"] is None and any("hardware autopilot" in r for r in c["reasons"]), \
        "a real autopilot behind a simulator's camera must be refused by name"
    assert topo.classify(_ev(host=DESKTOP, kind="hardware", camera_source="ros2",
                             controller_type=None))["grant"] is None


def test_g1_qualification_is_reported_never_assumed():
    c = topo.classify(_ev(g1={"passed": None}))
    assert c["grant"] == TOPOLOGY_DEV and c["rail_qualified_by_g1"] is None
    assert any("not yet qualified" in r for r in c["reasons"])
    assert topo.classify(_ev(g1={"passed": False}))["rail_qualified_by_g1"] is False


def test_the_manifest_takes_the_rail_s_label_and_keeps_its_old_guard():
    out = _tmp()
    cfg = rail.write_sim_configs(out, rail.plan_network("mirrored"))
    scene = out / cfg["scene"]
    m = build_manifest("ph", "none:hand-written-controller", 1, scene_path=scene,
                       topology=topo.TOPOLOGY_PAS_ARDUPILOT_MAVROS)
    assert m["topology"] == topo.TOPOLOGY_PAS_ARDUPILOT_MAVROS and m["sim_speedup"] == 1.0
    ok, why = is_kpi_grade(m, {})
    assert not ok and any("topology" in r for r in why)
    # Complete dev evidence AND a scene: still refused, on the scene. That is
    # the guard this rail must not get around (docs/DESIGN-...: what
    # manifest.py needs).
    _raises(build_manifest, "ph", "x", 1, scene_path=scene, topology=TOPOLOGY_DEV,
            hil_evidence=_ev()["topology_evidence"], contains="scene")


# =========================================================================== #
# perception_node.py
# =========================================================================== #

class FakeDetections:
    """follow_vlm.Grounder.latest() in shape: a small box right of centre.
    With `misses`, every second inference finds nothing (n_miss counts it)."""

    def __init__(self, clock, cx=576.0, w=20.0, fresh=True, misses=False):
        self.clock, self.cx, self.w, self.fresh, self.n = clock, cx, w, fresh, 0
        self.misses = misses

    def latest(self):
        self.n += 1
        now = self.clock.t
        miss = self.n // 2 if self.misses else 0
        return {"det": (self.cx, 216.0, self.w, 12.0, 0.3, 768, 432, 0.5),
                "t_det": now - (0.05 if self.fresh else 5.0), "seq": self.n,
                "n_seen": self.n - miss, "n_miss": miss}


def test_the_pilot_tracks_a_fresh_box_and_searches_when_it_is_stale():
    clk = Clock()
    p = pn.FollowPilot(FakeDetections(clk), 90.0, cruise_alt=8.0)
    body, info = p.step(clk.t, 5.0)
    assert info["mode"] == "track" and body[0] > 0, "small box: close in"
    assert body[3] > 0, "box right of centre: turn right (clockwise positive)"
    assert body[2] > 0, "below cruise: climb"
    p = pn.FollowPilot(FakeDetections(clk, fresh=False), 90.0)
    body, info = p.step(clk.t, 8.0)
    assert info["mode"] == "search" and body[0] == 0.0


def test_the_subject_is_placed_along_the_heading_without_ground_truth():
    det = (384.0, 216.0, 40.0, 20.0, 0.3, 768, 432, 0.5)       # centred
    st = State(x=10.0, y=5.0, up=8.0)
    x, y, src = pn.subject_estimate(det, None, st, math.pi / 2, 90.0, 4.0)
    assert src == "width" and abs(x - 10.0) < 1e-6 and y > 5.0, (x, y)
    assert pn.subject_estimate(None, None, st, 0.0, 90.0, 4.0) is None
    # Slant range from the camera becomes horizontal range: the height above
    # the subject's centre is taken out (Pythagoras), not ignored.
    r = pn._fv().implied_range_from_width(det, 4.0, 90.0)
    dh = 8.0 - pn.SUBJECT_UP_M
    assert abs((y - 5.0) - math.sqrt(r * r - dh * dh)) < 1e-6, (y, r)


class FakeLink:
    """An ArduPilot that flies whatever velocity it was last sent while in
    GUIDED, descends on its own in RTL / LAND, and can be told to take the
    aircraft at a given time (`takeover_at=(t, mode)`), as a fence would."""
    name = "pymavlink"

    def __init__(self, clock, anchor, yaw=0.0, takeover_at=None, fence=None,
                 clock_rate=1.0):
        self.clock, self.anchor, self.yaw = clock, anchor, yaw
        self.snap = ap.Snapshot()
        self.x, self.y, self.up, self.v = 35.0, -20.0, 8.0, Action4D()
        self.sent, self.sent_t, self.t_last = [], [], None
        self.events, self.requested = [], []
        self.sim_speedup = 1.0
        self.takeover_at, self.fence, self.clock_rate = takeover_at, fence, clock_rate
        self.t_start = clock.t

    def _mode(self, m):
        if m != self.snap.mode:
            self.events.append({"t": self.clock.t, "kind": "mode", "mode": m})
        self.snap.mode = m

    def bring_up(self, alt):
        self._mode("GUIDED")
        self.snap.armed = True
        self.pump()

    def pump(self):
        t = self.clock.t
        if (self.takeover_at and t - self.t_start >= self.takeover_at[0]
                and self.snap.mode == "GUIDED"):
            self._mode(self.takeover_at[1])
        if self.t_last is not None:
            dt = t - self.t_last
            if self.snap.mode == "GUIDED":
                self.x += self.v.vx * dt
                self.y += self.v.vy * dt
                self.up += self.v.vz_up * dt
            elif self.snap.mode in ("RTL", "LAND"):
                self.up = max(0.0, self.up - 1.0 * dt)
                if self.up == 0.0:
                    self.snap.armed, self.snap.landed_state = False, 1
        self.t_last = t
        lat, lon = self.anchor.to_latlon(self.x, self.y)
        s = self.snap
        s.lat, s.lon, s.rel_alt, s.yaw, s.pitch, s.roll = lat, lon, self.up, self.yaw, 0.0, 0.0
        s.time_boot_ms = int((t - 1000.0) * 1000 * self.clock_rate)
        s.t_time_boot = t
        if self.fence is not None:
            s.fence_breach_count = self.fence(t - self.t_start)

    def state(self, anchor):
        x, y = anchor.to_scene(self.snap.lat, self.snap.lon)
        return State(x=x, y=y, up=self.snap.rel_alt, yaw_deg=math.degrees(self.yaw))

    def request_mode(self, name):
        self.requested.append(name)
        self._mode(name)
        return True

    def send(self, a, raw_body=None):
        self.v = a
        self.sent.append(a)
        self.sent_t.append(self.clock.t)

    def land(self):
        self.requested.append("LAND")
        self._mode("LAND")
        return True

    def evidence(self):
        return {"link": "pymavlink", "heartbeat": True, "is_ardupilot": True,
                "topology_evidence": {"autopilot_kind": "sitl", **DESKTOP}}


class FakeCamera:
    collisions_subscribed, collisions = True, []

    def __init__(self, link, log=None):
        self.link, self.log = link, log

    def start(self):
        if self.log is not None:
            self.log.append("camera.start")

    def truth_geo(self):
        lat, lon = self.link.anchor.to_latlon(self.link.x + 0.1, self.link.y)
        return {"lat": lat, "lon": lon, "alt": 48.0}

    def stop(self):
        pass


class FakeObs:
    def put_pose_full(self, *a, **k):
        pass

    def get_depth(self):
        return None


CAP_POLICY = """policy_id: pas-ap-test
version: 0.1.0
constraints:
  - id: kin-caps
    type: kinematic_envelope
    constraint_type: hard
    priority: P1
    violation_action: repair
    speed_max_mps: 2.0
    climb_rate_max_mps: 2.0
    yaw_rate_max_dps: 45.0
  - id: alt-band
    type: altitude_envelope
    constraint_type: hard
    priority: P0
    violation_action: repair
    alt_min_m: 2
    alt_max_m: 30
"""

# The aircraft starts at 8 m and the ceiling is 6 m with a DIRECT RTL action:
# the position itself is illegal (unsafe) from the first tick, and the grant's
# FSM goes Normal -> RTL (G11).
RTL_POLICY = """policy_id: pas-ap-rtl
version: 0.1.0
constraints:
  - id: alt-ceiling
    type: altitude_envelope
    constraint_type: hard
    priority: P0
    violation_action: RTL
    alt_min_m: 1
    alt_max_m: 6
"""

STANDOFF_POLICY = """policy_id: pas-ap-standoff
version: 0.1.0
constraints:
  - id: standoff-car
    type: subject_standoff
    constraint_type: hard
    priority: P0
    violation_action: repair
    subject_class: car
    min_range_m: 120.0
"""


class FakeMavrosLink(FakeLink):
    """The same aircraft behind a live-looking MAVROS 2 on this desktop."""
    name = "mavros"

    def evidence(self):
        return {"link": "mavros", "heartbeat": True, "is_ardupilot": True,
                "topology_evidence": _ev()["topology_evidence"]}


def _fly(shield="on", link_cls=None, policy=CAP_POLICY, extra=(), link_kw=None,
         grounder_kw=None, max_s="3", camera_log=None, popen=None, run_cmd=None):
    out = _tmp()
    pol = out / "policy.yaml"
    pol.write_text(policy, encoding="utf-8")
    args = pn.build_parser().parse_args([
        "--out-root", str(out), "--tag", "run", "--policy", str(pol), "--max-s", max_s,
        "--hfov", "90", "--citymap", str(out / "no_map.npz"), "--network", "nat",
        "--windows-ip", WIN, "--wsl-ip", WSL, "--shield", shield, "--speed-max", "4",
        *extra])
    clk = Clock()
    anchor = ap.FrameAnchor(35.6895, 139.6917, 40.0, "scene.home-geo-point")
    link = (link_cls or FakeLink)(clk, anchor, **(link_kw or {}))
    rc = pn.run(args, link=link, camera=FakeCamera(link, camera_log),
                grounder=FakeDetections(clk, **(grounder_kw or {})),
                obs=FakeObs(), clock=clk, sleep=clk.sleep, popen=popen, run_cmd=run_cmd)
    return rc, out / "run", link


def _rows(run):
    return [json.loads(l) for l in (run / "flight_log.jsonl").read_text().splitlines()]


def _json(run, name):
    return json.loads((run / name).read_text())


def test_only_the_shield_s_output_reaches_the_autopilot():
    rc, run, link = _fly("on")
    assert rc == 0
    rows = _rows(run)
    assert len(rows) >= 25, len(rows)
    raw_speeds = [math.hypot(r["raw"]["vx"], r["raw"]["vy"]) for r in rows]
    assert max(raw_speeds) > 2.5, "the pilot never exceeded the cap: the test proves nothing"
    assert len(link.sent) == len(rows)
    for r, sent in zip(rows, link.sent):
        assert sent.model_dump() == r["flown_action"],             "something other than the Shield's output flew"
        assert r["flown"] is True
        assert math.hypot(sent.vx, sent.vy) <= 2.0 + 1e-6
    assert any(r["touched"] for r in rows)
    m = _json(run, "metrics.json")
    assert m["escape_ticks"] == 0, "with the Shield on, what flew must re-check clean"
    assert m["topology"] == topo.TOPOLOGY_PAS_ARDUPILOT_PYMAVLINK and m["grant_topology"] is None
    assert m["det_hz_grant_target"] == 10.0 and m["det_hz_null"] == 0.0 and m["det_hz"] > 0
    assert m["frac_within_30m"] is None and m["follow_scored"] is False, \
        "no subject truth is read, so no tracking score may be reported"
    assert m["ekf_vs_sim_horiz_m"]["n"] > 0 and abs(m["ekf_vs_sim_horiz_m"]["median"] - 0.1) < 0.01
    man = _json(run, "manifest.json")
    assert man["topology"] == topo.TOPOLOGY_PAS_ARDUPILOT_PYMAVLINK
    assert man["vla_model_hash"].startswith("none:hand-written-controller")
    kpi = _json(run, "kpi.json")
    assert kpi["kpi_grade"] is False
    assert (run / "pas_config" / "robot_citylife_ardupilot.jsonc").is_file()
    # The episode record the KPIs need, as on the two SITL rails.
    assert all(k in rows[0] for k in ("unsafe", "unsafe_rules", "policy_hash",
                                      "generation", "shield_ms", "ap_mode",
                                      "fsm_state_after"))
    assert (run / "events.jsonl").is_file() and (run / "policy_g0.json").is_file()


def test_the_episode_is_one_id_from_the_flight_log_to_the_kpi_table():
    """guardrail.replay binds a flight's records by the episode id
    (EpisodeRecord.episode_id) and refuses to pack an episode with no
    mission_end. Without the id in the rows, metrics and kpi, the bundle of a
    run flown today said 'the flight log carries no episode id (written before
    2026-10-07)' and tied the files together only by their file times."""
    from guardrail.replay import read_replay
    rc, run, _ = _fly("on")
    assert rc == 0
    events = [json.loads(l) for l in (run / "events.jsonl").read_text().splitlines()]
    start = next(e["episode_id"] for e in events if e["kind"] == "episode_start")
    assert sum(e["kind"] == "mission_end" for e in events) == 1, \
        [e["kind"] for e in events]
    rows = _rows(run)
    assert rows and all(r.get("episode_id") == start for r in rows), rows[0].get("episode_id")
    assert _json(run, "metrics.json")["episode_id"] == start
    assert _json(run, "kpi.json")["episode_id"] == start
    idx = read_replay(run / "run.replay.tar.gz")["index"]
    assert idx["log_binding"] == "episode", idx["log_binding"]


def test_a_link_that_dies_mid_flight_ends_the_episode_with_the_real_reason():
    """The loop's finally writes the mission_end. It used to carry the reason
    the loop was initialised with ('max_s reached') when the link raised."""
    class Dying(FakeLink):
        def pump(self):
            if self.clock.t - self.t_start > 1.0:
                raise ConnectionResetError("router went away")
            super().pump()
    out = _tmp()
    (out / "p.yaml").write_text(CAP_POLICY, encoding="utf-8")
    args = pn.build_parser().parse_args([
        "--out-root", str(out), "--tag", "d", "--policy", str(out / "p.yaml"),
        "--hfov", "90", "--network", "mirrored", "--max-s", "5"])
    clk = Clock()
    link = Dying(clk, ap.FrameAnchor(35.6895, 139.6917, 40.0, "s"))
    try:
        pn.run(args, link=link, camera=FakeCamera(link), grounder=FakeDetections(clk),
               obs=FakeObs(), clock=clk, sleep=clk.sleep)
    except ConnectionResetError:
        pass
    else:
        raise AssertionError("a dead link was swallowed")
    events = [json.loads(l) for l in (out / "d" / "events.jsonl").read_text().splitlines()]
    ends = [e for e in events if e["kind"] == "mission_end"]
    assert len(ends) == 1 and ends[0]["why"].startswith("crashed: ConnectionResetError"), ends
    assert not (out / "d" / "kpi.json").exists(), "a crashed flight wrote a KPI table"


def test_the_shield_off_arm_flies_raw_and_records_what_it_violated():
    rc, run, link = _fly("off")
    rows = _rows(run)
    for r, sent in zip(rows, link.sent):
        assert sent.model_dump() == r["raw"]
    assert any(r["emitted_violations"] for r in rows), "the control arm must earn its escapes"
    assert _json(run, "metrics.json")["escape_ticks"] > 0
    assert _json(run, "metrics.json")["fsm"] is None and link.requested == ["LAND"]


def test_a_persistent_p0_violation_reaches_a_mode_request():
    """The grant's Shield escalates through fail-safes; the first version of
    this node ran Shield.filter() only, never requested a mode, and its
    kpi.json said time to safe was not measurable."""
    rc, run, link = _fly("on", policy=RTL_POLICY, max_s="6")
    assert rc == 0
    rows = _rows(run)
    assert link.requested == ["RTL"], link.requested
    assert any(r.get("set_mode") == "RTL" for r in rows)
    assert rows[-1]["fsm_state_after"] == "RTL"
    m = _json(run, "metrics.json")
    assert m["mode_requests"] == ["RTL"] and m["fsm"]["failsafe_triggered"] is True
    # RTL ends at home (G9): the FSM knows the arming point, read from the EKF
    # right after take-off.
    assert m["fsm"]["terminal_reason"] == "home reached", m["fsm"]["terminal_reason"]
    assert m["autopilot_took_over"] is None, "a mode the FSM asked for is no takeover"
    kpi = _json(run, "kpi.json")
    assert kpi["failsafe_instrumented"] is True
    assert not kpi.get("time_to_safe_not_measurable"), kpi.get("time_to_safe_not_measurable")
    events = [json.loads(l) for l in (run / "events.jsonl").read_text().splitlines()]
    assert any(e["kind"] == "mode_request" and e["mode"] == "RTL" for e in events)
    # Once RTL was requested the autopilot owns the aircraft: no setpoints, no LAND.
    t_rtl = next(r["t"] for r in rows if r.get("set_mode") == "RTL")
    assert all(r["flown_action"] is None and r["flown"] is False
               for r in rows if r["t"] > t_rtl)


def test_the_autopilot_taking_the_aircraft_stops_the_commands():
    """The param file's fence RTL is the grant's backstop. If it fires, the
    node must stop sending ignored GUIDED setpoints and say so."""
    rc, run, link = _fly("on", link_kw={"takeover_at": (1.0, "RTL")}, max_s="40")
    rows = _rows(run)
    m = _json(run, "metrics.json")
    assert m["autopilot_took_over"] and m["autopilot_took_over"]["mode"] == "RTL"
    t_take = m["autopilot_took_over"]["t"]
    assert 0.9 <= t_take <= 1.3, t_take
    assert all(t - link.t_start <= t_take + 1e-6 for t in link.sent_t), \
        "setpoints were sent after the autopilot took the aircraft"
    assert link.requested == [], "a mode the autopilot chose must not be overridden"
    assert any(e["kind"] == "mode" and e["mode"] == "RTL" for e in m["autopilot_events"])
    assert m["end_reason"].startswith("autopilot took over")
    assert rows[-1]["ap_mode"] == "RTL" and rows[-1]["setpoint"] == "none"


def _audit(run):
    p = run / "audit.jsonl"
    return [json.loads(l) for l in p.read_text().splitlines()] if p.is_file() else []


def test_one_escalation_fsm_is_on_record_the_node_s():
    """The Shield runs its own escalation FSM unless built without one
    (guardrail.replay.rail_shield). With both, audit.jsonl recorded the
    Shield's state machine and flight_log.jsonl the node's."""
    # Shield off: no FSM governs the control arm, so no audit record may carry
    # an FSM state (a Shield-internal FSM would put one on every record).
    rc, run, _ = _fly("off")
    aud = _audit(run)
    assert aud, "the control arm's violations were not audited"
    assert all(a["fsm_state_after"] is None for a in aud), \
        {a["fsm_state_after"] for a in aud}
    # Shield on: every audit record carries the NODE's FSM verdict for its tick.
    rc, run, _ = _fly("on", policy=RTL_POLICY, max_s="6")
    rows = {r["tick"]: r for r in _rows(run)}
    aud = _audit(run)
    assert aud and any(a["fsm_state_after"] == "RTL" for a in aud)
    for a in aud:
        r = rows[a["tick"]]
        assert a["fsm_state_after"] == r.get("fsm_state_after"), (a["tick"], a, r)


def test_a_tick_nothing_was_sent_on_cannot_escape():
    """guardrail.replay.flown_fields: after the autopilot took the aircraft,
    nothing is sent, so no action flew and none can have escaped. The rule
    used to count the raw action's violations on every shield-off tick, flown
    or not (250 of 283 'escapes' of one SITL run were ArduPilot's RTL)."""
    rc, run, link = _fly("off", link_kw={"takeover_at": (1.0, "RTL")}, max_s="40")
    rows = _rows(run)
    m = _json(run, "metrics.json")
    after = [r for r in rows if r["setpoint"] == "none"]
    assert after, "the takeover never stopped the commands"
    assert all(r["flown"] is False and r["emitted_violations"] == [] for r in after)
    assert any(r["violations"] for r in after), \
        "the raw action still broke a rule after the takeover: the test proves nothing"
    flown_bad = sum(1 for r in rows if r["flown"] and r["emitted_violations"])
    assert m["escape_ticks"] == flown_bad > 0, (m["escape_ticks"], flown_bad)
    assert m["ticks_not_flown"] == len(after) and m["ticks_flown"] == len(rows) - len(after)


NFZ_AHEAD_POLICY = """policy_id: pas-ap-nfz-ahead
version: 0.1.0
constraints:
  - id: nfz-ahead
    type: polygon_fence
    constraint_type: hard
    priority: P0
    violation_action: repair
    vertices:
      - { x: 48, y: -30 }
      - { x: 60, y: -30 }
      - { x: 60, y: -10 }
      - { x: 48, y: -10 }
    altitude_floor_m: 0
    altitude_ceiling_m: 100
    margin_m: 1.0
"""


def test_the_shield_looks_the_grant_s_five_seconds_ahead():
    """The grant's Safety Shield: '5 s of predicted trajectory at 10 Hz = 50
    future poses' (guardrail.shield.LOOKAHEAD_S/DT_S, as both SITL rails
    fly). The node used 3 s at 0.5 s. The zone's margin starts 12 m north
    of the spawn; at the pilot's first speed that is more than 3 s and less
    than 5 s away, so only the grant's horizon sees it on the first tick."""
    rc, run, link = _fly("on", policy=NFZ_AHEAD_POLICY, max_s="1")
    rows = _rows(run)
    first = rows[0]
    eta = 12.0 / first["raw"]["vx"]
    assert 3.2 < eta < 4.9, f"{eta:.2f} s away: the test cannot tell 3 s from 5 s"
    assert any(v["rule_id"] == "nfz-ahead" for v in first["violations"]), first["violations"]
    m = _json(run, "metrics.json")
    assert m["shield_lookahead"] == {"horizon_s": 5.0, "dt_s": 0.1, "poses": 50}


class FencedLink(FakeLink):
    def __init__(self, *a, fence_vals=None, **k):
        super().__init__(*a, **k)
        self.fence_vals = fence_vals or {}

    def autopilot_fence(self):
        return {**{n: self.fence_vals.get(n) for n in ap.FENCE_PARAMS}, "source": "fake"}


def test_the_autopilot_s_own_fence_is_recorded_and_judged_against_the_policy():
    """The grant's backstop, as the autopilot held it. CAP_POLICY allows 30 m;
    a 20 m FENCE_ALT_MAX would RTL a flight the Shield considers legal, and
    the record must say so. Unanswered values stay None, never 'off'."""
    rc, run, _ = _fly("on", FencedLink, link_kw={"fence_vals": {
        "FENCE_ENABLE": 1.0, "FENCE_ALT_MAX": 20.0, "FENCE_RADIUS": 400.0,
        "FENCE_ACTION": 1.0}})
    f = _json(run, "metrics.json")["autopilot_fence"]
    assert f["backstop_active"] is True and f["policy_ceiling_m"] == 30.0
    assert f["stricter_than_policy"] is True and "below" in f["why"]
    assert f["changed_by_node"] is False and f["FENCE_TYPE"] is None
    events = [json.loads(l) for l in (run / "events.jsonl").read_text().splitlines()]
    assert any(e["kind"] == "autopilot_fence" and e["FENCE_ALT_MAX"] == 20.0 for e in events)
    rc, run, _ = _fly("on", FencedLink, link_kw={"fence_vals": {}})
    f = _json(run, "metrics.json")["autopilot_fence"]
    assert f["backstop_active"] is None and f["stricter_than_policy"] is None
    rc, run, _ = _fly("on")                         # a link that reads no parameters
    assert _json(run, "metrics.json")["autopilot_fence"] is None


def test_fence_breaches_are_counted_and_unmeasured_is_none():
    rc, run, _ = _fly("on", link_kw={"fence": lambda t: 2 if t > 1.5 else 0})
    assert _json(run, "metrics.json")["fence_breach_count"] == 2
    rc, run, _ = _fly("on")
    assert _json(run, "metrics.json")["fence_breach_count"] is None, "no FENCE_STATUS is not 0"


def test_the_node_flies_the_heading_it_reads():
    """Facing East, 'forward' is +y in the scene frame. A node that rotated
    by 0 instead of the autopilot's heading would fly North."""
    rc, run, link = _fly("on", link_kw={"yaw": math.pi / 2})
    rows = _rows(run)
    fwd = [r for r in rows if r["body"][0] > 0.5]
    assert fwd and all(r["raw"]["vy"] > 0.4 and abs(r["raw"]["vx"]) < 0.1 for r in fwd), fwd[:2]


def test_the_subject_estimate_reaches_the_shield_s_standoff_rule():
    rc, run, link = _fly("on", policy=STANDOFF_POLICY)
    rows = _rows(run)
    m = _json(run, "metrics.json")
    assert m["subject_known_ticks"] > 0 and m["subject_source"].get("width", 0) > 0
    assert any(v["rule_id"] == "standoff-car" for r in rows for v in r["violations"]), \
        "a subject 96 m away inside a 120 m ring bound nothing: set_subject was not called"


def test_det_hz_counts_every_inference_not_only_the_hits():
    rc, run, _ = _fly("on", grounder_kw={"misses": True})
    m = _json(run, "metrics.json")
    assert abs(m["det_hz"] - m["loop_hz"]) < 0.6, (m["det_hz"], m["loop_hz"])


def test_a_mavros_run_offers_dev_only_when_g1_passed_and_the_clock_was_real_time():
    """The node never stamps a grant label itself, and never offers one for an
    unqualified rail: G1 must have passed and ArduPilot must have run at real
    time. build_manifest then accepts (once manifest.py is taught this rail)
    or refuses because of the scene."""
    out = _tmp()
    verdict = out / "g1_verdict.json"
    verdict.write_text(json.dumps({"gate": "G1 (fallback-gates.md)", "g1_passed": True,
                                   "status": "passed"}), encoding="utf-8")
    rc, run, _ = _fly("on", FakeMavrosLink, extra=("--link", "mavros",
                                                   "--g1-verdict", str(verdict)))
    m = _json(run, "metrics.json")
    man = _json(run, "manifest.json")
    assert m["grant_topology"] == TOPOLOGY_DEV and m["grant_topology_offered"] == TOPOLOGY_DEV
    assert m["ardupilot_rt_factor"] == 1.0 and m["grant_topology_withheld"] == []
    if m["manifest_refusal"] is None:
        assert man["topology"] == TOPOLOGY_DEV
    else:
        assert man["topology"] == topo.TOPOLOGY_PAS_ARDUPILOT_MAVROS
        assert "scene" in m["manifest_refusal"], m["manifest_refusal"]
    assert m["hil_evidence"] == m["topology_evidence"]["topology_evidence"]
    # No verdict: the label is withheld, and the run cannot be KPI-grade.
    rc, run, _ = _fly("on", FakeMavrosLink, extra=("--link", "mavros"))
    m, kpi = _json(run, "metrics.json"), _json(run, "kpi.json")
    assert m["grant_topology"] == TOPOLOGY_DEV and m["grant_topology_offered"] is None
    assert any("G1" in w for w in m["grant_topology_withheld"])
    assert _json(run, "manifest.json")["topology"] == topo.TOPOLOGY_PAS_ARDUPILOT_MAVROS
    assert not kpi["kpi_grade"] and any("gate G1" in r for r in kpi["kpi_grade_reasons"])
    # G1 passed, but ArduPilot's own clock ran at 0.8x wall time.
    rc, run, _ = _fly("on", FakeMavrosLink, link_kw={"clock_rate": 0.8},
                      extra=("--link", "mavros", "--g1-verdict", str(verdict)))
    m, kpi = _json(run, "metrics.json"), _json(run, "kpi.json")
    assert abs(m["ardupilot_rt_factor"] - 0.8) < 0.01 and m["grant_topology_offered"] is None
    assert any("real time" in w for w in m["grant_topology_withheld"])
    assert any("real-time factor" in r for r in kpi["kpi_grade_reasons"])


def test_g1_status_reads_only_a_real_verdict():
    d = _tmp()
    assert pn.g1_status(None)["passed"] is None
    (d / "x.json").write_text(json.dumps({"g1_passed": True}), encoding="utf-8")
    assert pn.g1_status(str(d / "x.json"))["passed"] is None, "a bare flag is not a verdict"
    (d / "v.json").write_text(json.dumps({"gate": "G1 (fallback-gates.md)",
                                          "g1_passed": False, "status": "fired"}))
    assert pn.g1_status(str(d / "v.json"))["passed"] is False
    assert pn.g1_status(str(d / "missing.json"))["passed"] is None


class Spy:
    def __init__(self, log):
        self.log, self.cmds = log, []

    def popen(self, cmd, **kw):
        self.log.append("sitl.popen")
        self.cmds.append(cmd)
        return SimpleNamespace(wait=lambda timeout=None: 0)

    def run(self, cmd, **kw):
        self.log.append("sitl.stop")
        self.cmds.append(cmd)


def test_the_node_loads_the_scene_before_it_starts_sitl_and_stops_it_after():
    """A scene (re)load resets the simulator clock, and ArduPilot's AirSim
    backend adds the negative step to its own clock: SITL must start after."""
    log = []
    spy = Spy(log)
    _fly("on", extra=("--start-sitl",), camera_log=log, popen=spy.popen, run_cmd=spy.run)
    assert log == ["camera.start", "sitl.popen", "sitl.stop"], log
    assert "--mavros" not in spy.cmds[0]
    log.clear()
    spy = Spy(log)
    _fly("on", FakeMavrosLink, extra=("--start-sitl", "--link", "mavros"),
         camera_log=log, popen=spy.popen, run_cmd=spy.run)
    assert "--mavros" in spy.cmds[0], "--link mavros --start-sitl must start MAVROS too"


def test_impossible_combinations_are_refused_before_anything_starts():
    base = ["--network", "mirrored"]
    for extra, word in ((["--scene-mode", "attach", "--start-sitl"], "attach"),
                        (["--camera", "ros2"], "mavros"),
                        (["--scene-only", "--scene-mode", "attach"], "scene-only")):
        args = pn.build_parser().parse_args(base + extra)
        try:
            pn.check_args(args)
        except SystemExit as exc:
            assert word in str(exc), exc
        else:
            raise AssertionError(f"{extra} was accepted")


class FakeWorld:
    loads = []

    def __init__(self, client, scene_config_name="", delay_after_load_sec=0,
                 sim_config_path="sim_config/", sim_instance_idx=-1):
        FakeWorld.loads.append(scene_config_name)
        self.client = client

    def switch_streaming_view(self):
        pass


class FakeDrone:
    def __init__(self, client, world, name):
        data = world.sim_config if hasattr(world, "sim_config") else None
        self.parent = getattr(world, "parent_topic", None)
        self.robot_info = {"collision_info": f"{self.parent}/robots/{name}/collision_info"}
        self.sensors = {"FrontCamera": {"scene_camera": "s", "depth_camera": "d"}}
        self.data = data


class FakeClient2:
    def __init__(self, address="127.0.0.1", topics=None):
        self.address, self.topics_live, self.subs = address, topics, []
        self.topics = None

    def connect(self):
        pass

    def get_topic_info(self):
        self.topics = dict.fromkeys(self.topics_live or [])

    def subscribe(self, topic, cb):
        self.subs.append(topic)

    def disconnect(self):
        pass


def _fake_pas(topics):
    return SimpleNamespace(ProjectAirSimClient=lambda address: FakeClient2(address, topics),
                           World=FakeWorld, Drone=FakeDrone)


def test_attach_never_loads_the_scene_and_refuses_one_that_is_not_live():
    """The hil node on the Orin must not reload the scene under a running
    ArduPilot, and must not attach to some other scene in silence."""
    cfg = rail.write_attach_configs(_tmp())
    FakeWorld.loads.clear()
    live = ["/Sim/SceneCityLifeArduPilot/robots/Drone1/sensors/FrontCamera/scene_camera"]
    cam = pn.ProjectAirSimCamera(FakeObs(), cfg, projectairsim=_fake_pas(live), attach=True)
    cam.start()
    assert FakeWorld.loads == [], "attach loaded a scene"
    assert cam.world.parent_topic == "/Sim/SceneCityLifeArduPilot"
    assert cam.drone.data["actors"][0]["robot-config"]["controller"]["type"] == "ardupilot-api"
    assert cam.scene_record["mode"] == "attach" and cam.scene_record["robot_topics_live"] == 1
    other = ["/Sim/SceneBasicDrone/robots/Drone1/sensors/x"]
    cam = pn.ProjectAirSimCamera(FakeObs(), cfg, projectairsim=_fake_pas(other), attach=True)
    _raises(cam.start, contains="-Step scene")
    # load mode does load
    cam = pn.ProjectAirSimCamera(FakeObs(), rail.write_sim_configs(
        _tmp(), rail.plan_network("mirrored")), projectairsim=_fake_pas(live))
    cam.start()
    assert FakeWorld.loads == ["scene_citylife_ardupilot.jsonc"]


def test_scene_only_loads_and_exits_without_flying():
    out = _tmp()
    args = pn.build_parser().parse_args(["--scene-only", "--network", "mirrored",
                                         "--out-root", str(out), "--tag", "s"])
    log = []
    assert pn.run(args, camera=FakeCamera(None, log)) == 0
    assert log == ["camera.start"]
    rec = json.loads((out / "s" / "scene_load.json").read_text())
    assert "start ArduPilot now" in rec["next"]
    assert not (out / "s" / "flight_log.jsonl").exists()


def test_a_refused_bring_up_still_releases_the_simulator():
    class Refusing(FakeLink):
        def bring_up(self, alt):
            raise RuntimeError("arming refused: PreArm: test")

    class Watched(FakeCamera):
        stopped = False

        def stop(self):
            Watched.stopped = True
    out = _tmp()
    (out / "p.yaml").write_text(CAP_POLICY, encoding="utf-8")
    args = pn.build_parser().parse_args([
        "--out-root", str(out), "--tag", "r", "--policy", str(out / "p.yaml"),
        "--hfov", "90", "--network", "mirrored"])
    clk = Clock()
    link = Refusing(clk, ap.FrameAnchor(35.6895, 139.6917, 40.0, "s"))
    _raises(pn.run, args, link=link, camera=Watched(link), grounder=FakeDetections(clk),
            obs=FakeObs(), clock=clk, sleep=clk.sleep, contains="arming refused")
    assert Watched.stopped, "the simulator connection outlived a failed run"


def test_the_dry_run_names_every_missing_package():
    old = pn.module_available
    try:
        pn.module_available = lambda n: n != "pymavlink"
        args = pn.build_parser().parse_args(["--dry-run", "--network", "mirrored"])
        rep = pn.dry_run(args, _tmp())
        assert not rep["ok"] and any("pymavlink" in p for p in rep["problems"])
        pn.module_available = lambda n: True
        assert pn.dry_run(args, _tmp())["ok"]
        args = pn.build_parser().parse_args(["--dry-run", "--scene-mode", "attach",
                                             "--link", "mavros"])
        rep = pn.dry_run(args, _tmp())
        assert rep["sim_config"]["network"]["mode"] == "attach" and "sitl_args" not in rep
    finally:
        pn.module_available = old


# =========================================================================== #
# g1_check.py - the four conditions
# =========================================================================== #

WAYPOINTS = [(0, 0, 0, 0), (10, 0, 0, 0), (15, 0, 0, 10), (20, 10, 0, 10),
             (25, 10, 10, 10), (30, 0, 10, 10), (35, 0, 0, 10), (40, 0, 0, 0),
             (50, 0, 0, 0)]


def _path(t, wps):
    for a, b in zip(wps, wps[1:]):
        if a[0] <= t <= b[0]:
            f = 0.0 if b[0] == a[0] else (t - a[0]) / (b[0] - a[0])
            return [a[k] + f * (b[k] - a[k]) for k in (1, 2, 3)]
    return list(wps[-1][1:])


def synthetic_run(seed=1, ekf_off=(0.0, 0.0, 0.0), land_x=0.0, rest_drift=0.0,
                  pwm_rise_t=10.0, frozen_truth=False, collision_t=None, fence=0,
                  run=1):
    wps = [w if i < 7 else (w[0], land_x, w[2], w[3]) for i, w in enumerate(WAYPOINTS)]
    truth, ekf, servo = [], [], []
    for i in range(0, 1001):
        t = i * 0.05
        p = [0.0, 0.0, 0.0] if frozen_truth else _path(t, wps)
        if 4.0 <= t <= 9.0:
            p[0] += rest_drift * (t - 4.0) / 5.0
        truth.append([t, *p])
    for i in range(0, 501):
        t = i * 0.1
        p = _path(t, wps)
        n = 0.15 * math.sin(7.0 * t)
        ekf.append([t, p[0] + ekf_off[0] + n, p[1] + ekf_off[1], p[2] + ekf_off[2]])
        servo.append([t] + [1000 if t < pwm_rise_t else 1500] * 4)
    return {"seed": seed, "run": run, "ekf_ready_t": 2.0, "sitl_clock_advanced": True,
            "truth": truth, "ekf": ekf, "servo": servo, "arm_t": 10.0,
            "rest_window": [4.0, 9.0], "frames": {"before": "b.png", "airborne": "a.png"},
            "events": [{"t": 10.5, "kind": "mode", "mode": "AUTO"}],
            "mission": {"uploaded": True, "nav_seqs": [1, 2, 3, 4, 5, 6],
                        "reached": [1, 2, 3, 4, 5, 6],
                        "corners_scene": [[10, 0], [10, 10], [0, 10], [0, 0]],
                        "side_m": 10, "alt_m": 10},
            "landed_t": 42.0, "takeoff_truth": [0.0, 0.0, 0.0],
            "landing_truth": [land_x, 0.0, 0.0], "collisions_subscribed": True,
            "collision_events": ([] if collision_t is None else
                                 [{"t": collision_t, "object": "pole"}]),
            "fence_breach_count": fence}


def test_g1_1_passes_a_tracking_ekf_and_fails_an_offset_one():
    assert g1.check_ekf(synthetic_run())["passed"] is True
    v = g1.check_ekf(synthetic_run(ekf_off=(1.5, 0.0, 0.0)))
    assert v["passed"] is False and v["p95_m"] > 1.0


def test_g1_1_cannot_pass_a_vehicle_the_bridge_never_moved():
    """ArduPilot believes it flies the square; the simulated vehicle never
    left the pad. The frozen-truth null is exactly this case."""
    v = g1.check_ekf(synthetic_run(frozen_truth=True))
    assert v["passed"] is not True, v


def test_g1_1_does_not_pass_jitter_a_fixed_offset_would_also_match():
    """>= 10 m of path, but all of it +/-0.5 m shaking about the start, and an
    EKF sitting 0.6 m off: inside 1 m, yet a vehicle that never moved scores
    nearly the same. Only the frozen-truth null notices."""
    rec = synthetic_run()
    rec["truth"] = [[t, 0.5 * math.sin(2 * math.pi * t), 0.0, 0.0]
                    for t in (i * 0.05 for i in range(1001))]
    rec["ekf"] = [[t, 0.5 * math.sin(2 * math.pi * t) + 0.6, 0.0, 0.0]
                  for t in (i * 0.1 for i in range(501))]
    v = g1.check_ekf(rec)
    assert v["truth_path_m"] >= g1.MIN_TRUTH_PATH_M and v["p95_m"] <= g1.EKF_TOL_M, v
    assert v["passed"] is None and "frozen-truth null" in v["reasons"][0], v


def test_g1_1_needs_the_minimum_travel_even_when_the_null_is_beaten():
    """A clean 8 m hop beats the frozen null easily; it is still below the
    10 m of travel the definition asks for, so it is not measured."""
    rec = synthetic_run()
    hop = [(0, 0, 0, 0), (10, 0, 0, 0), (20, 8, 0, 0), (50, 8, 0, 0)]
    rec["truth"] = [[t, *_path(t, hop)] for t in (i * 0.05 for i in range(1001))]
    rec["ekf"] = [[t, *_path(t, hop)] for t in (i * 0.1 for i in range(501))]
    v = g1.check_ekf(rec)
    assert v["p95_m"] < g1.NULL_RATIO * v["null_frozen_p95_m"], v
    assert v["passed"] is None and v["truth_path_m"] < g1.MIN_TRUTH_PATH_M, v


def test_g1_1_is_not_measured_without_motion_a_ready_ekf_or_a_sitl_clock():
    rec = synthetic_run()
    rec["ekf"] = [e for e in rec["ekf"] if e[0] < 9.0]           # only the parked part
    assert g1.check_ekf(rec)["passed"] is None
    assert g1.check_ekf({**synthetic_run(), "ekf_ready_t": None})["passed"] is None
    assert g1.check_ekf({**synthetic_run(), "sitl_clock_advanced": False})["passed"] is False


def test_g1_1_lets_the_ekf_settle_after_it_first_reports_ready():
    """The first EKF_SETTLE_S after 'ready' are the filter converging; a 5 m
    transient there is not a bridge failure, and the same error later is."""
    rec = synthetic_run()
    rec["ekf"] = [list(e) for e in rec["ekf"] if e[0] < 30.0]
    for e in rec["ekf"]:
        if 2.0 <= e[0] < 4.0:
            e[1] += 5.0
    assert g1.check_ekf(rec)["passed"] is True
    late = synthetic_run()
    late["ekf"] = [list(e) for e in late["ekf"] if e[0] < 30.0]
    for e in late["ekf"]:
        if 20.0 <= e[0] < 22.0:
            e[1] += 5.0
    assert g1.check_ekf(late)["passed"] is False


def test_g1_truth_is_never_interpolated_across_a_gap():
    truth = [[0.0, 0.0, 0.0, 0.0], [10.0, 10.0, 0.0, 0.0]]
    assert g1.interp_truth(truth, 5.0) is None, "a 10 s gap is not a position"
    close = [[0.0, 0.0, 0.0, 0.0], [0.1, 1.0, 0.0, 0.0]]
    assert abs(g1.interp_truth(close, 0.05)[0] - 0.5) < 1e-9


def test_g1_2_requires_motion_after_pwm_and_none_without_it():
    assert g1.check_pwm(synthetic_run())["passed"] is True
    v = g1.check_pwm(synthetic_run(rest_drift=0.6))
    assert v["passed"] is False and "disarmed" in " ".join(v["reasons"])
    v = g1.check_pwm(synthetic_run(pwm_rise_t=13.0))
    assert v["passed"] is False and "BEFORE" in " ".join(v["reasons"])
    assert g1.check_pwm({**synthetic_run(), "servo": []})["passed"] is None
    assert g1.check_pwm({**synthetic_run(), "rest_window": None})["passed"] is None


def test_g1_3_fails_the_hover_null_a_far_landing_and_an_airborne_crash():
    assert g1.check_square(synthetic_run())["passed"] is True
    hover = synthetic_run(frozen_truth=True)          # "lands 0 m away", never flew
    v = g1.check_square(hover)
    assert v["passed"] is False and "corners" in " ".join(v["reasons"])
    assert g1.check_square(synthetic_run(land_x=3.0))["passed"] is False
    assert g1.check_square(synthetic_run(collision_t=22.0))["passed"] is False
    assert g1.check_square(synthetic_run(collision_t=49.0))["passed"] is True, \
        "touching the ground after landing is not a crash"
    assert g1.check_square(synthetic_run(fence=1))["passed"] is False
    assert g1.check_square({**synthetic_run(), "fence_breach_count": None})["passed"] is None
    assert g1.check_square({**synthetic_run(), "collisions_subscribed": False})["passed"] is None
    rec = synthetic_run()
    rec["events"] = []
    assert g1.check_square(rec)["passed"] is False, "no AUTO mode, no pass"


def test_g1_3_needs_every_waypoint_reported_reached():
    rec = synthetic_run()
    rec["mission"]["reached"] = [1, 2, 4, 5, 6]
    v = g1.check_square(rec)
    assert v["passed"] is False and "[3]" in " ".join(v["reasons"]), v["reasons"]


def test_g1_3_counts_a_contact_of_unknown_height_against_the_run():
    """A contact when the truth stream had a gap cannot be shown to be the
    ground; it is counted, not waved through."""
    rec = synthetic_run(collision_t=22.0)
    rec["truth"] = [r for r in rec["truth"] if not 21.0 <= r[0] <= 23.0]
    assert g1.airborne_collisions(rec) == 1
    assert g1.check_square(rec)["passed"] is False


def _summary(rec):
    rec["checks"] = {"1": g1.check_ekf(rec), "2": g1.check_pwm(rec), "3": g1.check_square(rec)}
    return {"run": rec["run"], "seed": rec["seed"], "checks": rec["checks"],
            "landing_truth": rec["landing_truth"], "corner_hits": g1.corner_hits(rec)}


def test_g1_4_needs_three_passing_runs_with_one_seed():
    good = [_summary(synthetic_run(run=i)) for i in (1, 2, 3)]
    assert g1.check_repeat(good)["passed"] is True
    assert g1.check_repeat(good[:2])["passed"] is None
    assert g1.check_repeat(good[:2] + [_summary(synthetic_run(seed=2, run=3))])["passed"] is False
    assert g1.check_repeat(good[:2] + [_summary(synthetic_run(land_x=1.5, run=3))])["passed"] is True
    far = _summary(synthetic_run(run=3))
    far["landing_truth"] = [3.0, 0.0, 0.0]
    assert g1.check_repeat(good[:2] + [far])["passed"] is False
    bad = _summary(synthetic_run(rest_drift=0.6, run=3))
    assert g1.check_repeat(good[:2] + [bad])["passed"] is False


def test_g1_4_fails_corner_passes_that_wander_between_runs():
    """Each run within 2 m of every corner, but run 3 passes corner 1 three
    metres from where runs 1 and 2 did: not the same mission three times."""
    good = [_summary(synthetic_run(run=i)) for i in (1, 2, 3)]
    good[2]["corner_hits"][0] = [good[2]["corner_hits"][0][0] + 3.0,
                                 good[2]["corner_hits"][0][1]]
    v = g1.check_repeat(good)
    assert v["passed"] is False and "corner passes spread" in " ".join(v["reasons"])


def test_g1_never_passes_on_a_condition_that_was_not_measured():
    t = {"passed": True}
    res = g1.overall({"1": t, "2": t, "3": {"passed": None}, "4": t})
    assert res["status"] == "not measured" and res["g1_passed"] is False
    assert g1.overall({"1": t, "2": {"passed": False}, "3": t, "4": t})["status"] == "fired"
    assert g1.overall({"1": t, "2": t, "3": t, "4": t})["g1_passed"] is True


def _write_runs(d, runs, attempted=None, crashed=(), no_record=()):
    """g1_run.json files the way run_once writes them, plus the attempt log."""
    d.mkdir(parents=True, exist_ok=True)
    for i in (attempted if attempted is not None else runs):
        g1.record_attempt(d, i, 1, f"att{i}")
    for i in runs:
        (d / f"run_{i}").mkdir(parents=True, exist_ok=True)
        if i in no_record:
            continue
        rec = synthetic_run(run=i)
        rec.update(attempt_id=f"att{i}", completed=i not in crashed,
                   error=({"type": "RuntimeError", "message": "arming refused"}
                          if i in crashed else None))
        g1.evaluate(rec)
        (d / f"run_{i}" / "g1_run.json").write_text(json.dumps(rec), encoding="utf-8")


def test_summarize_requires_every_run_to_pass_not_just_the_first():
    d = _tmp() / "g1_x"
    _write_runs(d, (1, 2, 3))
    rec = json.loads((d / "run_3" / "g1_run.json").read_text())
    rec["ekf"] = [[e[0], e[1] + 1.5, e[2], e[3]] for e in rec["ekf"]]
    g1.evaluate(rec)
    (d / "run_3" / "g1_run.json").write_text(json.dumps(rec), encoding="utf-8")
    res = g1.summarize(d, 3)
    assert res["checks"]["1"]["per_run"] == [True, True, False]
    assert res["checks"]["1"]["passed"] is False, "run 3 failed condition 1"
    assert res["status"] == "fired" and not res["g1_passed"]
    assert (d / "g1_verdict.md").is_file()
    clean = _tmp() / "g1_ok"
    _write_runs(clean, (1, 2, 3))
    assert g1.summarize(clean, 3)["status"] == "passed", "three good runs must pass"


def test_a_crashed_run_is_a_failed_run_never_a_missing_one():
    """The reviewer's case: runs 1, 3 and 4 pass, run 2 crashed before it
    wrote anything. The old summarize globbed the records that existed and
    returned PASSED."""
    d = _tmp() / "g1_crash"
    _write_runs(d, (1, 2, 3, 4), no_record=(2,))
    res = g1.summarize(d, 4)
    assert res["status"] == "fired" and not res["g1_passed"], res["conditions"]
    assert res["checks"]["1"]["per_run"][1] is False
    assert any("run 2" in r for r in res["checks"]["3"]["reasons"])
    # A run whose record says it did not complete counts the same way.
    d = _tmp() / "g1_err"
    _write_runs(d, (1, 2, 3), crashed=(2,))
    res = g1.summarize(d, 3)
    assert not res["g1_passed"] and "arming refused" in json.dumps(res["runs"][1])
    # The launcher's -Runs: four asked for, three made, is not a pass either.
    d = _tmp() / "g1_short"
    _write_runs(d, (1, 2, 3))
    assert not g1.summarize(d, 4)["g1_passed"]


def test_a_reused_tag_cannot_pass_on_an_old_run():
    d = _tmp() / "g1_reuse"
    _write_runs(d, (1, 2, 3, 4), attempted=(1, 2, 3))     # run_4 is left over
    res = g1.summarize(d, 3)
    assert not res["g1_passed"] and res["run_set_problems"], res
    # Four runs made, three summarised: the fourth is not silently dropped.
    d = _tmp() / "g1_extra"
    _write_runs(d, (1, 2, 3, 4))
    res = g1.summarize(d, 3)
    assert not res["g1_passed"] and any("outside" in p for p in res["run_set_problems"])
    # A record from another attempt of the same run number.
    d = _tmp() / "g1_stale"
    _write_runs(d, (1, 2, 3))
    g1.record_attempt(d, 2, 1, "a-later-attempt")
    res = g1.summarize(d, 3)
    assert not res["g1_passed"] and "another attempt" in json.dumps(res["runs"][1])


class _Boom:
    """A projectairsim whose scene load fails, after the client connected."""

    def __init__(self):
        self.disconnected = False
        boom = self

        class Client:
            def __init__(self, address):
                pass

            def connect(self):
                pass

            def disconnect(self):
                boom.disconnected = True

        def world(*a, **k):
            raise RuntimeError("LoadScene failed: level not open")
        self.ProjectAirSimClient, self.World = Client, world


def _g1_args(out, tag="g1_t", run=1, extra=()):
    return SimpleNamespace(
        tag=tag, run=run, seed=7, out_root=str(out), network="mirrored", windows_ip=None,
        wsl_ip=None, sim_host="127.0.0.1", mavlink_url="tcp:x", start_sitl=False,
        wsl_distro="Ubuntu", side_m=10.0, alt_m=10.0, rest_s=0.1, timeout_s=1.0,
        overwrite=False, **dict(extra))


def test_a_run_that_crashes_writes_its_record_and_raises():
    out = _tmp()
    pas = _Boom()
    try:
        g1.run_once(_g1_args(out), pas=pas)
    except RuntimeError as exc:
        assert "LoadScene failed" in str(exc)
    else:
        raise AssertionError("a failed run reported success")
    rec = json.loads((out / "g1_t" / "run_1" / "g1_run.json").read_text())
    assert rec["error"]["type"] == "RuntimeError" and rec["completed"] is False
    assert rec["checks"]["1"]["passed"] is None and rec["sim_config"] is not None
    assert pas.disconnected, "the simulator connection outlived the failed run"
    assert g1.read_attempts(out / "g1_t")[1]["attempt_id"] == rec["attempt_id"]
    res = g1.summarize(out / "g1_t", 1)
    assert res["checks"]["1"]["passed"] is False
    # The same run number again in the same tag is refused, not overwritten.
    try:
        g1.run_once(_g1_args(out), pas=pas)
    except FileExistsError:
        pass
    else:
        raise AssertionError("a G1 record was silently overwritten")


def test_the_square_mission_flies_where_the_corners_say():
    anchor = ap.FrameAnchor(35.6895, 139.6917, 40.0, "scene")
    items, corners = g1.square_mission(anchor, (35.0, -20.0), 10.0, 10.0)
    assert [i["command"] for i in items] == [16, 22, 16, 16, 16, 16, 21]
    assert items[1]["z"] == 10.0 and items[-1]["z"] == 0.0
    for it, c in zip(items[2:6], corners):
        x, y = anchor.to_scene(it["x"] / 1e7, it["y"] / 1e7)
        assert abs(x - c[0]) < 0.02 and abs(y - c[1]) < 0.02
    assert corners[0] == [45.0, -20.0] and corners[-1] == [35.0, -20.0]
    # Take-off and landing both at the start point: "lands within 2 m of
    # take-off" is judged against the point the mission was flown from.
    x, y = anchor.to_scene(items[-1]["x"] / 1e7, items[-1]["y"] / 1e7)
    assert abs(x - 35.0) < 0.02 and abs(y + 20.0) < 0.02
    assert (items[-1]["x"], items[-1]["y"]) == (items[1]["x"], items[1]["y"])


class MissionLink:
    """Answers the upload handshake like ArduPilot, or refuses it.
    `early_ack` sends an ACCEPTED ack after that many items."""

    def __init__(self, inbox, ack_type=0, bad_seq=None, early_ack=None):
        self.m = SimpleNamespace(mav=FakeMav())
        self.target_system = self.target_component = 1
        self.inbox, self.ack_type, self.bad_seq = inbox, ack_type, bad_seq
        self.early_ack = early_ack
        self.clock = Clock(0.01)
        self.n_items = None

    def pump(self):
        sent = self.m.mav.sent
        if not sent:
            return
        name, a, _ = sent[-1]
        if name == "mission_clear_all_send" and not getattr(self, "_cleared", False):
            self._cleared = True
            self.inbox.append(Msg("MISSION_ACK", type=0))
        elif name == "mission_count_send" and self.n_items is None:
            self.n_items = a[2]
            self.inbox.append(Msg("MISSION_REQUEST_INT",
                                  seq=0 if self.bad_seq is None else self.bad_seq))
        elif name == "mission_item_int_send" and a[2] == len(self._acked()):
            seq = a[2]
            self._acked().append(seq)
            nxt = seq + 1
            if self.early_ack is not None and nxt >= self.early_ack:
                self.inbox.append(Msg("MISSION_ACK", type=0))
                return
            self.inbox.append(Msg("MISSION_ACK", type=self.ack_type) if nxt >= self.n_items
                              else Msg("MISSION_REQUEST_INT", seq=nxt))

    def _acked(self):
        if not hasattr(self, "_seen"):
            self._seen = []
        return self._seen


def test_the_mission_upload_handshake_and_its_refusals():
    anchor = ap.FrameAnchor(35.6895, 139.6917, 40.0, "scene")
    items, _ = g1.square_mission(anchor, (0.0, 0.0), 10.0, 10.0)
    q = collections.deque()
    ok, why = g1.upload_mission(MissionLink(q), items, q, timeout=5)
    assert ok, why
    q = collections.deque()
    ok, why = g1.upload_mission(MissionLink(q, ack_type=1), items, q, timeout=5)
    assert not ok and "ACK type 1" in why
    q = collections.deque()
    ok, why = g1.upload_mission(MissionLink(q, bad_seq=99), items, q, timeout=5)
    assert not ok and "99" in why
    # ACCEPTED after 2 of 7 items is a mission ArduPilot does not hold.
    q = collections.deque()
    ok, why = g1.upload_mission(MissionLink(q, early_ack=2), items, q, timeout=5)
    assert not ok and "2/7" in why, why


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs a file outside this repository)")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
