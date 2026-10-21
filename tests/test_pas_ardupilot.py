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
    assert p.router_tcp_url == f"tcp:127.0.0.1:{rail.ROUTER_TCP_PORT}"


def test_under_nat_windows_reaches_the_router_at_wsl_s_address():
    """mavlink-routerd's TCP server listens on [::]; WSL's NAT localhost
    forwarding does not carry an IPv6 listener to Windows' 127.0.0.1, so G1 run
    1 of 2026-10-08 timed out on tcp:127.0.0.1:5790 with ArduPilot running."""
    p = rail.plan_network("nat", WIN, WSL)
    assert p.router_tcp_host == WSL
    assert p.router_tcp_url == f"tcp:{WSL}:{rail.ROUTER_TCP_PORT}"
    for mod in (g1, pn):
        src = Path(mod.__file__).read_text(encoding="utf-8")
        assert 'add_argument("--mavlink-url", default=None' in src, \
            f"{mod.__name__} still hard-codes 127.0.0.1 for the router"


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
    # ...and a non-real-time clock the right way up: step-ns of sim time per
    # real-time-update-rate of wall time (the reciprocal before 2026-10-19).
    slow = {"clock": {"step-ns": 1500000, "real-time-update-rate": 3000000}}
    assert rail.scene_clock_ratio(slow) == 0.5


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


def test_the_link_says_how_old_its_position_and_heartbeat_are():
    """REGRESSION (2026-10-19). state() answers from the last position the
    link ever received: a SITL that died behind mavlink-router (the socket
    stays open and goes quiet) read as an aircraft hovering in place, and
    the node flew on it until max_s. link_age() gives the ages the node now
    judges each tick (perception_node.link_staleness)."""
    anchor = ap.FrameAnchor(35.6895, 139.6917, 40.0, "scene")
    fm = FakeMavutil([AP_HB])
    link = _link(fm)
    link.connect(timeout=5)
    a = link.link_age()
    assert a["pos_age_s"] is None and a["heartbeat_age_s"] is not None, a
    assert "no position fix received" in pn.link_staleness(link)[0]
    lat, lon = anchor.to_latlon(35.0, -20.0)
    fm.master.inbox = [Msg("GLOBAL_POSITION_INT", src=1, lat=int(round(lat * 1e7)),
                           lon=int(round(lon * 1e7)), alt=48000, relative_alt=8000,
                           vx=0, vy=0, vz=0)]
    link.pump()
    assert pn.link_staleness(link)[0] is None, "a fix just received is live"
    for _ in range(150):                         # 1.5 s on the link's clock, nothing arrives
        link.clock()
    link.pump()
    why, ages = pn.link_staleness(link)
    assert why and "no position fix for" in why and ages["pos_age_s"] > pn.POS_STALE_S, why
    assert "heartbeat" not in why, "a 1.5 s old heartbeat (1 Hz) is not yet stale"
    assert link.state(anchor) is not None, \
        "state() still answers from the frozen fix: the reason the age is checked"
    for _ in range(200):                         # 3.5 s: the heartbeat too
        link.clock()
    assert "no vehicle heartbeat for" in pn.link_staleness(link)[0]


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
    RCIn=object, OverrideRCIn=SimpleNamespace, Waypoint=SimpleNamespace,
    SetMode=_SRV, CommandBool=_SRV, CommandTOL=_SRV, StreamRate=_SRV,
    GetParameters=_SRV, VehicleInfoGet=_SRV, WaypointPush=_SRV, WaypointPull=_SRV,
    qos_sensor=None)
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


def test_mavros_says_how_old_its_state_is_and_when_the_fcu_dropped():
    """REGRESSION (2026-10-19). /mavros/state's `connected` turned False when
    MAVROS lost the FCU and nothing in flight read it; state() went on
    answering from the last fix. And a run whose link dropped late was
    described as 'no ArduPilot heartbeat was received'."""
    link, node = _mavros()
    node.subs["/mavros/state"](SimpleNamespace(connected=True, armed=True, mode="GUIDED"))
    node.subs["/mavros/global_position/global"](
        SimpleNamespace(latitude=35.6895, longitude=139.6917, altitude=40.0))
    a = link.link_age()
    assert a["connected"] is True and a["pos_age_s"] < 0.1 and a["heartbeat_age_s"] < 0.1, a
    assert pn.link_staleness(link)[0] is None
    for _ in range(150):                         # 1.5 s on the link's clock
        link.clock()
    assert "no position fix for" in pn.link_staleness(link)[0]
    node.subs["/mavros/global_position/global"](
        SimpleNamespace(latitude=35.6895, longitude=139.6917, altitude=40.0))
    node.subs["/mavros/state"](SimpleNamespace(connected=False, armed=True, mode="GUIDED"))
    assert pn.link_staleness(link)[0] == "MAVROS reports the flight controller disconnected"
    ev = link.evidence()
    assert (ev["heartbeat"], ev["heartbeat_seen"], ev["fcu_connected_at_end"]) == \
        (False, True, False), ev
    assert ev["ardupilot_seen"] is True and ev["is_ardupilot"] is False, ev
    keys = ("heartbeat", "is_ardupilot", "heartbeat_seen", "ardupilot_seen")
    c = topo.classify(_ev(autopilot={k: ev[k] for k in keys}))
    assert c["grant"] is None
    assert any("ArduPilot's heartbeat was received" in r and "link dropped" in r
               for r in c["reasons"]), c["reasons"]
    assert not any("no ArduPilot heartbeat was received" in r for r in c["reasons"])
    # Never connected at all is still "no heartbeat".
    c = topo.classify(_ev(autopilot={"heartbeat": False, "is_ardupilot": False,
                                     "heartbeat_seen": False}))
    assert any("no ArduPilot heartbeat was received" in r for r in c["reasons"])


def test_a_px4_that_connected_and_dropped_is_not_called_ardupilot():
    """REVIEW (2026-10-19). MAVROS's heartbeat_seen is ANY flight controller
    that connected, and is_ardupilot needs `connected`, so after a drop the
    reason said "ArduPilot's heartbeat was received ... the link dropped"
    for a PX4 too. Only an FCU seen connected with an ArduCopter mode name
    (ardupilot_seen) is called ArduPilot. Failed before: the PX4 drop read
    "ArduPilot's heartbeat was received"."""
    link, node = _mavros()
    node.subs["/mavros/state"](SimpleNamespace(connected=True, armed=True, mode="OFFBOARD"))
    node.subs["/mavros/state"](SimpleNamespace(connected=False, armed=True, mode="OFFBOARD"))
    ev = link.evidence()
    assert (ev["heartbeat"], ev["heartbeat_seen"], ev["ardupilot_seen"]) == \
        (False, True, False), ev
    keys = ("heartbeat", "is_ardupilot", "heartbeat_seen", "ardupilot_seen")
    c = topo.classify(_ev(autopilot={k: ev[k] for k in keys}))
    assert c["grant"] is None
    assert not any("ArduPilot's heartbeat was received" in r for r in c["reasons"]), \
        c["reasons"]
    assert any("nothing on record shows it was ArduPilot" in r and "link dropped" in r
               for r in c["reasons"]), c["reasons"]
    # Evidence without ardupilot_seen (an older record) is not called ArduPilot either.
    c = topo.classify(_ev(autopilot={"heartbeat": False, "is_ardupilot": False,
                                     "heartbeat_seen": True}))
    assert not any("ArduPilot's heartbeat was received" in r for r in c["reasons"])
    # A disconnected FCU's mode name does not count: connected False with GUIDED.
    link, node = _mavros()
    node.subs["/mavros/state"](SimpleNamespace(connected=False, armed=False, mode="GUIDED"))
    assert link.evidence()["ardupilot_seen"] is False
    # pymavlink: an ArduPilot heartbeat is what sets the target at all.
    fm = FakeMavutil([AP_HB])
    plink = _link(fm)
    plink.connect(timeout=5)
    pev = plink.evidence()
    assert pev["ardupilot_seen"] is True and pev["heartbeat_seen"] is True


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


def test_the_manifest_takes_the_rail_s_label_and_the_grant_label_on_evidence():
    out = _tmp()
    cfg = rail.write_sim_configs(out, rail.plan_network("mirrored"))
    scene = out / cfg["scene"]
    m = build_manifest("ph", "none:hand-written-controller", 1, scene_path=scene,
                       topology=topo.TOPOLOGY_PAS_ARDUPILOT_MAVROS)
    assert m["topology"] == topo.TOPOLOGY_PAS_ARDUPILOT_MAVROS and m["sim_speedup"] == 1.0
    ok, why = is_kpi_grade(m, {})
    assert not ok and any("topology" in r for r in why)
    # Complete dev evidence beside this rail's scene: accepted since
    # 2026-10-12, because guardrail.manifest READS the robot config the
    # simulator was given and finds ArduCopterApi there (docs/DESIGN-...:
    # what manifest.py needs). Until then refused, on the scene.
    m = build_manifest("ph", "x", 1, scene_path=scene, topology=TOPOLOGY_DEV,
                       hil_evidence=_ev()["topology_evidence"])
    assert m["topology"] == TOPOLOGY_DEV, m
    # The guard this rail must not get around: the same files with the
    # simulator's own controller are refused, on the controller.
    robot = out / cfg["robot"]
    doc = json.loads(robot.read_text(encoding="utf-8"))
    doc["controller"]["type"] = "simple-flight-api"
    robot.write_text(json.dumps(doc), encoding="utf-8")
    _raises(build_manifest, "ph", "x", 1, scene_path=scene, topology=TOPOLOGY_DEV,
            hil_evidence=_ev()["topology_evidence"], contains="simple-flight-api")


# =========================================================================== #
# perception_node.py
# =========================================================================== #

class FakeDetections:
    """follow_vlm.Grounder.latest() in shape: a small box right of centre.
    With `misses`, every second inference finds nothing (n_miss counts it)."""

    def __init__(self, clock, cx=576.0, w=20.0, fresh=True, misses=False, cy=216.0,
                 h=12.0):
        self.clock, self.cx, self.w, self.fresh, self.n = clock, cx, w, fresh, 0
        self.misses, self.cy, self.h = misses, cy, h

    def latest(self):
        self.n += 1
        now = self.clock.t
        miss = self.n // 2 if self.misses else 0
        return {"det": (self.cx, self.cy, self.w, self.h, 0.3, 768, 432, 0.5),
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


class LosableDetections(FakeDetections):
    """A box at column `cx` that goes stale when `lost` is set."""

    def __init__(self, clock, cx):
        super().__init__(clock, cx=cx)
        self.lost = False

    def latest(self):
        out = super().latest()
        if self.lost:
            out["t_det"] = self.clock.t - 5.0
        return out


def test_the_search_sweep_starts_toward_the_side_the_box_was_lost_on():
    """REGRESSION (2026-10-19). follow_vlm.search_sweep_rate always starts
    turning RIGHT, so a box lost off the LEFT of the frame - the servo
    turning left after it - got a yaw step of over 1 rad/s at the hand-over
    and a first half-swing away from where it went. The first swing now goes
    toward the side the box was last tracked on, held for the whole search,
    and the sweep still nets zero rotation. Failed before: the left loss
    swung right (+0.685 rad/s)."""
    for cx, side in ((192.0, -1.0), (576.0, 1.0)):
        clk = Clock()
        dets = LosableDetections(clk, cx)
        p = pn.FollowPilot(dets, 90.0)
        body, info = p.step(clk.t, 8.0)
        assert info["mode"] == "track" and math.copysign(1.0, body[3]) == side, body
        dets.lost = True
        rates = []
        for _ in range(40):                       # one 4 s sweep period at 10 Hz
            clk.t += 0.1
            body, info = p.step(clk.t, 8.0)
            assert info["mode"] == "search" and info["sweep_side"] == side, info
            rates.append(body[3])
        assert math.copysign(1.0, rates[0]) == side and abs(rates[0]) > 0.6, (cx, rates[:3])
        assert abs(sum(r * 0.1 for r in rates)) < 1e-6, "the sweep walked the nose off"
        # Found again on the other side, then lost: the next search follows it.
        dets.lost, dets.cx = False, 768.0 - cx
        p.step(clk.t + 0.1, 8.0)
        dets.lost = True
        body, info = p.step(clk.t + 0.2, 8.0)
        assert info["sweep_side"] == -side and math.copysign(1.0, body[3]) == -side
    # Never seen at all: right, as before.
    p = pn.FollowPilot(FakeDetections(Clock(), fresh=False), 90.0)
    body, info = p.step(1000.0, 8.0)
    assert body[3] > 0.6 and info["sweep_side"] == 1.0


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


def _depth_image(r):
    import numpy as np
    return np.full((432, 768), r, dtype=np.float32)


def test_a_depth_return_nearer_than_the_road_below_is_not_the_subject():
    """REGRESSION (pas_ap_20261008_224440, tick 857: the first of five P0
    escape ticks). The state and the box are the flight's; the depth inside
    the box read less than the camera's 7.2 m height above a car's centre.
    The old subject_estimate turned that slant range into a horizontal range
    of 0 and put the car exactly under the aircraft - where FrontCamera,
    20 deg down with a half vertical field of about 29 deg, cannot see - and
    the Shield then held a 5 m P0 ring centred on the aircraft. The depth is
    now refused and the box width ranges the car instead."""
    fv = pn._fv()
    det = (391.8, 155.5, 50.5, 24.1, 0.0211, 768, 432, 0.11)
    st = State(x=36.847, y=-48.38, up=7.962, yaw_deg=-99.78)
    yaw = math.radians(-99.78)
    for r in (4.0, 6.0, 7.0):                    # all under the 7.2 m height
        why = pn.depth_implausible(det, r, st.up, 90.0, fv)
        assert why, r
        x, y, src = pn.subject_estimate(det, _depth_image(r), st, yaw, 90.0, 1.9, fv)
        assert src == "width:depth-rejected", (r, src)
        assert math.hypot(x - st.x, y - st.y) > 5.0, (r, x, y)
    # Nearer than the camera's height above any subject: refused whatever the
    # ray - even for a box with no frame size, where no ray can be drawn.
    assert "whatever the ray" in pn.depth_implausible(det, 4.0, st.up, 90.0, fv)
    assert "whatever the ray" in pn.depth_implausible(det[:5], 4.0, st.up, 90.0, fv)
    assert pn.depth_implausible(det[:5], 6.0, st.up, 90.0, fv) is None
    # A box near the horizon, where the base-ray check has no ground range to
    # compare with: a 7 m return along that ray is 8 m above the road.
    high = (384.0, 60.0, 40.0, 10.0, 0.3, 768, 432, 0.5)
    assert fv.ground_range_m(high, 8.0, 0.0, hfov_deg=90.0) is None
    assert "cannot see under" in pn.depth_implausible(high, 7.0, 8.0, 90.0, fv)
    # Short of where the box's base ray meets the road: in front of the subject.
    assert "in front of the subject" in pn.depth_implausible(det, 10.0, st.up, 90.0, fv)
    # A range that agrees with the box's base ray is the subject's, and kept.
    x, y, src = pn.subject_estimate(det, _depth_image(34.0), st, yaw, 90.0, 1.9, fv)
    dh = st.up - pn.SUBJECT_UP_M
    assert src == "depth" and abs(math.hypot(x - st.x, y - st.y)
                                  - math.sqrt(34.0 ** 2 - dh * dh)) < 1e-6
    # A ray steep enough to see under the aircraft (nose 35 deg down, box at
    # the frame's bottom, its lower edge 83 deg down) may still put a subject
    # close below: 0.85 m away at 7.3 m. Not straight below at 7.0 m, though:
    # that centre would be 90 deg down, under the box.
    low = (384.0, 400.0, 60.0, 40.0, 0.3, 768, 432, 0.5)
    assert math.degrees(pn.ray_depression_rad(400.0, 768, 432, 90.0,
                                              math.radians(-35.0))) > 80.0
    assert pn.depth_implausible(low, 7.3, 8.0, 90.0, fv, math.radians(-35.0)) is None
    assert "would put" in pn.depth_implausible(low, 7.0, 8.0, 90.0, fv,
                                               math.radians(-35.0))
    # Nose up is LESS depression (MAVLink ATTITUDE pitch is positive nose up).
    assert pn.ray_depression_rad(216.0, 768, 432, 90.0, math.radians(5.0)) < \
        pn.ray_depression_rad(216.0, 768, 432, 90.0, 0.0)


def test_the_flight_log_keeps_a_refused_depth_and_the_rule_that_refused_it():
    """The 2026-10-12 re-fly refused 43 depth returns and logged only the
    width range that replaced them (56-156 m, up to 122 m off the nearest
    depth-ranged tick), so which rule refused them could not be told
    afterwards. depth_check records the depth range and the refusal; the row
    carries it."""
    fv = pn._fv()
    det = (391.8, 155.5, 50.5, 24.1, 0.0211, 768, 432, 0.11)
    st = State(x=36.847, y=-48.38, up=7.962, yaw_deg=-99.78)
    bad = pn.depth_check(det, _depth_image(10.0), st, 90.0, fv)
    assert bad["r_m"] == 10.0 and "in front of the subject" in bad["refused"], bad
    good = pn.depth_check(det, _depth_image(34.0), st, 90.0, fv)
    assert good == {"r_m": 34.0, "refused": None}, good
    assert pn.depth_check(det, None, st, 90.0, fv) is None
    assert pn.depth_check(None, _depth_image(34.0), st, 90.0, fv) is None
    # It agrees with subject_estimate on what was refused.
    for r in (4.0, 10.0, 34.0):
        _, _, src = pn.subject_estimate(det, _depth_image(r), st, math.radians(-99.78),
                                        90.0, 1.9, fv)
        refused = pn.depth_check(det, _depth_image(r), st, 90.0, fv)["refused"]
        assert (src == "width:depth-rejected") == bool(refused), (r, src, refused)
    src = (ROOT / "demo" / "pas_ardupilot" / "perception_node.py").read_text(encoding="utf-8")
    assert '"subject_depth": subj_depth' in src


def test_a_depth_that_would_place_the_subject_under_its_own_box_is_refused():
    """REVIEW (2026-10-09). The first three rules let a box low in the frame
    through: at 7.96 m up, a box at 0.85-0.9 of the frame height (its bottom
    edge 43-46 deg below the horizon) with a depth of 7.5-8.5 m passed them
    all, and Pythagoras then placed the car 2.1-4.5 m away - 58-74 deg below
    the horizon, outside FrontCamera's view and inside the 5 m P0 ring. The
    fourth rule refuses a range whose placement would be under the box's
    bottom edge by more than DEPRESSION_SLACK_DEG. Failed on the code before
    it (every cell below returned None from the old rules)."""
    fv = pn._fv()
    up, H = 7.96, 432
    for frac in (0.85, 0.875, 0.9):
        for bh in (20.0, 30.0):
            det = (384.0, frac * H, 40.0, bh, 0.3, 768, H, 0.5)
            for r in (7.5, 8.0, 8.5):
                why = pn.depth_implausible(det, r, up, 90.0, fv)
                assert why, (frac, bh, r)
                old = pn.DEPRESSION_SLACK_DEG
                pn.DEPRESSION_SLACK_DEG = 1e9          # the fourth rule off
                try:
                    before = pn.depth_implausible(det, r, up, 90.0, fv)
                finally:
                    pn.DEPRESSION_SLACK_DEG = old
                if before is None:
                    assert "would put" in why, (frac, bh, r, why)
                    h = math.sqrt(r * r - (up - pn.SUBJECT_UP_M) ** 2)
                    assert h < 5.0, h                  # inside the P0 ring
    # Within the slack (3 deg) a range is kept: pitch error is not a refusal.
    det = (384.0, 0.9 * H, 40.0, 24.0, 0.3, 768, H, 0.5)
    bottom = pn.ray_depression_rad(0.9 * H + 12.0, 768, H, 90.0)
    dh = up - pn.SUBJECT_UP_M
    r_edge = dh / math.sin(bottom + math.radians(pn.DEPRESSION_SLACK_DEG - 0.5))
    assert pn.depth_implausible(det, r_edge, up, 90.0, fv) is None, r_edge
    # The tick-857 box at the range the box's base ray agrees with: kept.
    det857 = (391.8, 155.5, 50.5, 24.1, 0.0211, 768, 432, 0.11)
    assert pn.depth_implausible(det857, 34.0, 7.962, 90.0, fv) is None


class FakeLink:
    """An ArduPilot that flies whatever velocity it was last sent while in
    GUIDED, descends on its own in RTL / LAND, and can be told to take the
    aircraft at a given time (`takeover_at=(t, mode)`), as a fence would."""
    name = "pymavlink"

    def __init__(self, clock, anchor, yaw=0.0, takeover_at=None, fence=None,
                 clock_rate=1.0, pitch=0.0):
        self.clock, self.anchor, self.yaw, self.pitch = clock, anchor, yaw, pitch
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
        s.lat, s.lon, s.rel_alt, s.yaw, s.pitch, s.roll = (lat, lon, self.up, self.yaw,
                                                           self.pitch, 0.0)
        s.time_boot_ms = int((t - 1000.0) * 1000 * self.clock_rate)
        s.t_time_boot = t
        # A live link: a fresh position fix and heartbeat on every pump.
        s.t_pos = s.t_heartbeat = t
        if self.fence is not None:
            s.fence_breach_count = self.fence(t - self.t_start)

    def state(self, anchor):
        x, y = anchor.to_scene(self.snap.lat, self.snap.lon)
        return State(x=x, y=y, up=self.snap.rel_alt, yaw_deg=math.degrees(self.yaw))

    def link_age(self):
        """As the real links' (PymavlinkLink / MavrosLink.link_age)."""
        t, s = self.clock.t, self.snap
        return {"pos_age_s": None if s.t_pos is None else t - s.t_pos,
                "heartbeat_age_s": None if s.t_heartbeat is None else t - s.t_heartbeat,
                "connected": None}

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
    def __init__(self, depth_m=None):
        self.depth_m = depth_m

    def put_pose_full(self, *a, **k):
        pass

    def get_depth(self):
        return None if self.depth_m is None else _depth_image(self.depth_m)


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


# The last _fly's link and run folder: a test whose run() raises reads them
# here (_fly_raising).
_LAST: dict = {}


def _fly(shield="on", link_cls=None, policy=CAP_POLICY, extra=(), link_kw=None,
         grounder_kw=None, max_s="3", camera_log=None, popen=None, run_cmd=None,
         depth_m=None, clk=None, grounder_cls=None, pre=None):
    out = _tmp()
    pol = out / "policy.yaml"
    pol.write_text(policy, encoding="utf-8")
    args = pn.build_parser().parse_args([
        "--out-root", str(out), "--tag", "run", "--policy", str(pol), "--max-s", max_s,
        "--hfov", "90", "--citymap", str(out / "no_map.npz"), "--network", "nat",
        "--windows-ip", WIN, "--wsl-ip", WSL, "--shield", shield, "--speed-max", "4",
        # No start gate unless a test asks for one (`extra` comes after, and
        # argparse keeps the last value): these stand-ins test the loop.
        "--dev-yaml", "--start-when-seen", "0", *extra])
    if pre is not None:
        pre(out / "run")                   # the run folder as a test leaves it
    clk = clk or Clock()
    anchor = ap.FrameAnchor(35.6895, 139.6917, 40.0, "scene.home-geo-point")
    link = (link_cls or FakeLink)(clk, anchor, **(link_kw or {}))
    _LAST.update(link=link, run=out / "run")
    rc = pn.run(args, link=link, camera=FakeCamera(link, camera_log),
                grounder=(grounder_cls or FakeDetections)(clk, **(grounder_kw or {})),
                obs=FakeObs(depth_m), clock=clk, sleep=clk.sleep, popen=popen,
                run_cmd=run_cmd)
    return rc, out / "run", link


def _fly_raising(exc_type, **kw):
    """_fly for a run() that must raise `exc_type`: (the exception, the run
    folder, the link as run() left it)."""
    try:
        _fly(**kw)
    except exc_type as exc:
        return exc, _LAST["run"], _LAST["link"]
    raise AssertionError(f"run() did not raise {exc_type.__name__}")


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
        "--hfov", "90", "--network", "mirrored", "--max-s", "5", "--dev-yaml",
        "--start-when-seen", "0"])
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


# The aircraft starts at 8 m under a 6 m ceiling whose action is a DIRECT
# loiter: a stop is illegal there, so the Shield recovers and the FSM goes
# Normal -> Loiter (X4) on the first tick.
LOITER_POLICY = RTL_POLICY.replace("pas-ap-rtl", "pas-ap-loiter").replace(
    "violation_action: RTL", "violation_action: loiter")


def test_the_fsm_s_loiter_is_flown_as_arducopter_s_brake():
    """REGRESSION (7 Oct SITL, 8 Oct pas_ap_20261008_224440). ArduCopter's
    LOITER takes its climb rate from the throttle stick; with no pilot input
    it sank 1.4-1.8 m/s in every LOITER spell of the 8 Oct follow. The FSM
    still says LOITER (the grant's word, on record), and the autopilot is
    asked for BRAKE, which holds position and altitude with no pilot input.
    BRAKE is a mode the node asked for, so it is no takeover. The old node
    sent LOITER. `--hold-mode LOITER` restores the literal mapping."""
    rc, run, link = _fly("on", policy=LOITER_POLICY, max_s="3")
    assert rc == 0
    assert link.requested == ["BRAKE", "LAND"], link.requested
    rows = _rows(run)
    hold = [r for r in rows if r.get("set_mode")]
    assert [(r["set_mode"], r["autopilot_set_mode"]) for r in hold] == [("LOITER", "BRAKE")]
    m = _json(run, "metrics.json")
    assert m["autopilot_took_over"] is None, "a mode the FSM asked for is no takeover"
    # Both vocabularies on record, as on the ROS rail: the FSM's words and
    # ArduCopter's (review 2026-10-09: this node used to record only BRAKE).
    assert m["mode_requests"] == ["LOITER", "LAND"], m["mode_requests"]
    assert m["autopilot_mode_requests"] == ["BRAKE", "LAND"], m["autopilot_mode_requests"]
    assert m["fsm"]["hold_mode"] == {"fsm_mode": "LOITER", "autopilot_mode": "BRAKE"}
    events = [json.loads(l) for l in (run / "events.jsonl").read_text().splitlines()]
    req = [e for e in events if e["kind"] == "mode_request"]
    assert req and (req[0]["mode"], req[0]["autopilot_mode"]) == ("LOITER", "BRAKE"), req
    rc, run, link = _fly("on", policy=LOITER_POLICY, max_s="3",
                         extra=("--hold-mode", "LOITER"))
    assert link.requested == ["LOITER", "LAND"], link.requested
    # The pymavlink link can ask for BRAKE and name it in a heartbeat.
    assert ap.COPTER_MODES["BRAKE"] == 17 and ap.COPTER_MODE_NAMES[17] == "BRAKE"


# The aircraft starts 2 m over a 6 m ceiling whose action is `repair`: a stop
# is illegal there, so the Shield recovers (a descent) and the FSM sits in
# Brake while the position stays illegal.
CEILING_POLICY = RTL_POLICY.replace("pas-ap-rtl", "pas-ap-ceiling").replace(
    "violation_action: RTL", "violation_action: repair")


class SluggishLink(FakeLink):
    """An autopilot that follows only 40 % of the commanded climb rate: the
    recovery works, slower than the Shield asked (about 0.2 m/s)."""

    def pump(self):
        v = self.v
        self.v = Action4D(vx=v.vx, vy=v.vy, vz_up=0.4 * v.vz_up, yaw_rate=v.yaw_rate)
        try:
            super().pump()
        finally:
            self.v = v


def test_the_node_hands_the_fsm_the_position_s_illegal_depth():
    """REVIEW (2026-10-09). X6 leaves a working recovery alone only if it is
    TOLD how far from legal the position is; a node passing
    unsafe_depth_m=None passed every test. Here a slow descent out of a
    ceiling takes over 5 s (T): with the depth the FSM sees it working and
    holds X6; without it, X6 sent the aircraft home at 5.0 s."""
    rc, run, link = _fly("on", link_cls=SluggishLink, policy=CEILING_POLICY, max_s="9")
    assert rc == 0
    rows = _rows(run)
    depths = [r["unsafe_depth_m"] for r in rows if r.get("stop_illegal")]
    assert depths[0] == 2.0 and depths[-1] < 1.0, (depths[0], depths[-1])
    late = [r for r in rows if r["t"] >= 5.0 and r.get("stop_illegal")]
    assert late, "the recovery finished before T: the test proves nothing"
    for r in late:
        prog = r["fsm"]["x6_progress"]
        assert prog is not None and prog["slope_mps"] is not None, (r["t"], prog)
        assert prog["progressing"] is True, (r["t"], prog)
    edges = [r["fsm_edge"] for r in rows if r.get("fsm_edge")]
    assert "X6" not in edges and link.requested == ["LAND"], (edges, link.requested)


WATCHED_CAP_POLICY = """policy_id: pas-ap-watched-cap
version: 0.1.0
constraints:
  - id: watched-speed
    type: kinematic_envelope
    constraint_type: soft
    priority: P2
    violation_action: monitor_only
    speed_max_mps: 2.0
    climb_rate_max_mps: 2.0
    yaw_rate_max_dps: 45.0
  - id: nfz-ahead
    type: polygon_fence
    constraint_type: hard
    priority: P0
    violation_action: repair
    margin_m: 1.0
    vertices:
      - {x: 45, y: -40}
      - {x: 70, y: -40}
      - {x: 70, y: 0}
      - {x: 45, y: 0}
"""


def test_a_watched_only_speed_cap_does_not_fault_the_fsm_on_either_theta_basis():
    """REVIEW (2026-10-09). A monitor_only speed cap the raw action exceeds,
    and a slide off a zone on the same ticks. On the one-period reading the
    FSM measures from the raw action clamped to the caps the Shield applied
    (none here); read off the whole policy, the watched cap made every tick
    "a decision made under other caps": a fault, a hold, nothing streamed.
    The run records the basis it flew, and its KPI table is judged on it."""
    for extra, horizon in (((), None), (("--theta-horizon-s", "0.1"), 0.1)):
        rc, run, link = _fly("on", policy=WATCHED_CAP_POLICY, max_s="4", extra=extra)
        assert rc == 0
        rows = _rows(run)
        both = [r for r in rows
                if any(v["rule_id"] == "watched-speed" for v in r["violations"])
                and any(op["operator"] == "GeofenceSlide" for op in r["repairs"])]
        assert both, "no tick had both: the test proves nothing"
        m = _json(run, "metrics.json")
        assert m["fsm"]["fault"] is None, m["fsm"]["fault"]
        assert m["fsm"]["theta_basis"]["horizon_s"] == horizon, m["fsm"]["theta_basis"]
        assert _json(run, "kpi.json")["repair_theta_horizon_s"] == horizon
    src = (ROOT / "demo" / "pas_ardupilot" / "perception_node.py").read_text(encoding="utf-8")
    i = src.index("fsm_out = fsm.step(tick_input_from_decision(")
    call = src[i:src.index("setpoint = fsm_out.setpoint", i)]
    assert "unsafe_depth_m=unsafe_depth" in call and \
        "caps=shield.applied_caps or False" in call, call


def test_an_fsm_fault_holds_the_aircraft_as_brake_not_loiter():
    """REVIEW (2026-10-09). When the FSM refuses a tick the node holds the
    aircraft and stops streaming. That hold must be flown as BRAKE too (the
    LOITER this unit exists to avoid sinks with no pilot); a node sending the
    literal LOITER there passed every test."""
    real = pn.tick_input_from_decision

    def refuse(*a, **k):
        raise ValueError("forced: a decision the FSM refuses")
    pn.tick_input_from_decision = refuse
    try:
        rc, run, link = _fly("on", max_s="2")
    finally:
        pn.tick_input_from_decision = real
    assert rc == 0
    assert link.requested[:1] == ["BRAKE"], link.requested
    m = _json(run, "metrics.json")
    assert "forced" in m["fsm"]["fault"]
    assert m["mode_requests"][:1] == ["LOITER"] and m["autopilot_mode_requests"][:1] == ["BRAKE"]
    assert m["autopilot_took_over"] is None, "the node's own hold is no takeover"


def test_the_node_reads_the_body_pitch_it_flies_at():
    """The depth rules read the body pitch (MAVLink ATTITUDE, nose up
    positive). Nose 34 deg down, a box low in the frame and a depth of 7.4 m
    from 8 m up: a subject almost below, which that steep a camera does see,
    so the depth is kept. Read with the sign flipped, the camera would look
    14 deg down and the same depth would be refused. The row logs the pitch,
    so a replay can judge the rules."""
    rc, run, link = _fly("on", link_kw={"pitch": -0.6},
                         grounder_kw={"cx": 384.0, "cy": 400.0, "w": 60.0, "h": 40.0},
                         depth_m=7.4, max_s="2")
    assert rc == 0
    m = _json(run, "metrics.json")
    assert m["subject_source"].get("depth", 0) > 0, m["subject_source"]
    assert "width:depth-rejected" not in m["subject_source"], m["subject_source"]
    rows = _rows(run)
    assert all(abs(r["pitch_deg"] - math.degrees(-0.6)) < 0.01 for r in rows)
    # Level, the same box and depth are refused: the pitch is what decided.
    rc, run, link = _fly("on", grounder_kw={"cx": 384.0, "cy": 400.0, "w": 60.0, "h": 40.0},
                         depth_m=7.4, max_s="2")
    m = _json(run, "metrics.json")
    assert m["subject_source"].get("width:depth-rejected", 0) > 0, m["subject_source"]


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


STANDOFF_ANY_POLICY = (STANDOFF_POLICY.replace("pas-ap-standoff", "pas-ap-standoff-any")
                       .replace("standoff-car", "standoff-any")
                       .replace("subject_class: car", 'subject_class: "*"'))


def test_an_unclassified_subject_still_binds_the_catch_all_standoff():
    """REGRESSION (2026-10-19). "a child", "a jogger", "people" name no class
    in follow_vlm's tables (subject_width -> 4.0 m, None). The node then gave
    the Shield NO subject (`subject_class is not None` gated set_subject), so
    even the catch-all "*" stand-off - written for "a subject that could not
    be classified" - never bound, while metrics still reported its ring.
    Failed before: subject_known_ticks 0, no violation, no intervention."""
    rc, run, link = _fly("on", policy=STANDOFF_ANY_POLICY, extra=("--object", "a child"))
    assert rc == 0
    rows, m = _rows(run), _json(run, "metrics.json")
    assert m["subject_class"] is None and m["subject_known_ticks"] > 0, m["subject_known_ticks"]
    assert any(v["rule_id"] == "standoff-any" for r in rows for v in r["violations"]), \
        "a subject 96 m away inside a 120 m catch-all ring bound nothing"
    assert m["interventions"] > 0
    assert m["subject_standoff_rules"] == ["standoff-any"]
    assert m["subject_unclassified"] == {"object": "a child", "width_m": 4.0,
                                         "binding_rules": ["standoff-any"]}
    events = [json.loads(l) for l in (run / "events.jsonl").read_text().splitlines()]
    assert any(e["kind"] == "subject_unclassified" and e["object"] == "a child"
               for e in events)
    # Where the grade is decided, too (review 2026-10-19): a width-ranged tick
    # assumed a car's 4.0 m, ~8x too far for a child. Failed before: no reason.
    kpi = _json(run, "kpi.json")
    assert kpi["kpi_grade"] is False
    assert any("'a child' names no class" in w and "standoff-any" in w and "4 m" in w
               for w in kpi["kpi_grade_reasons"]), kpi["kpi_grade_reasons"]
    # A rule for a class still binds only that class.
    rc, run, _ = _fly("on", policy=STANDOFF_POLICY, extra=("--object", "a child"))
    m = _json(run, "metrics.json")
    assert m["subject_standoff_rules"] == [] and m["interventions"] == 0
    assert not any(v["rule_id"] == "standoff-car" for r in _rows(run) for v in r["violations"])
    assert any("names no class: no stand-off rule binds it" in w
               for w in _json(run, "kpi.json")["kpi_grade_reasons"])
    # A phrase that names a class carries no such reason.
    rc, run, _ = _fly("on", policy=STANDOFF_POLICY, extra=("--object", "a red car"))
    assert not any("names no class" in w for w in _json(run, "kpi.json")["kpi_grade_reasons"])


class CaptureDetections(FakeDetections):
    """FakeDetections with the capture follow_vlm.Grounder publishes beside
    the box (latest()["cap"]): the pose the frame was taken at, from the
    node's own pose ring, and the range read from that frame's depth."""

    def __init__(self, clock, cap_pose=None, range_m=20.0, **kw):
        super().__init__(clock, **kw)
        self.cap_pose, self.range_m = cap_pose, range_m

    def latest(self):
        out = super().latest()
        now = self.clock.t
        out["cap"] = {"t_cap": now - 0.25, "t_capture": now - 0.3,
                      "pose": (None if self.cap_pose is None
                               else dict(self.cap_pose, src="ring")),
                      "depth_src": "capture", "depth_dt_ms": 5.0, "range_m": self.range_m}
        return out


def test_the_subject_is_placed_from_its_own_capture_not_from_this_tick():
    """REGRESSION (2026-10-19). The node placed a box from a frame up to a
    second old with the CURRENT tick's position and heading, and read its
    range from the depth frame that had arrived LAST (pas_ap runs: heading
    turned a p95 of 17-29 deg since the capture). The box here was captured
    at (35, -20) facing North with its own depth frame reading 20 m; the
    aircraft has since turned 30 deg and moved, and the newest depth frame
    reads 60 m (both pass the depth rules). The subject is where the capture
    puts it. Failed before: placed 60 m out along the current heading."""
    pose = {"t": 0.0, "x": 35.0, "y": -20.0, "up": 8.0, "yaw": 0.0, "pitch": 0.0,
            "roll": 0.0}
    box = {"cx": 384.0, "cy": 224.0, "w": 40.0, "h": 24.0}
    rc, run, link = _fly("on", link_kw={"yaw": math.radians(30.0)}, depth_m=60.0,
                         grounder_cls=CaptureDetections, grounder_kw={**box, "cap_pose": pose})
    assert rc == 0
    rows = [r for r in _rows(run) if r["subject"] is not None]
    assert len(rows) >= 25, len(rows)
    assert max(math.hypot(r["x"] - 35.0, r["y"] + 20.0) for r in rows) > 2.0, \
        "the aircraft never left the capture pose: the test cannot tell them apart"
    horiz = math.sqrt(20.0 ** 2 - (8.0 - pn.SUBJECT_UP_M) ** 2)
    for r in rows:
        x, y, src = r["subject"]
        assert src == "depth" and abs(x - (35.0 + horiz)) < 0.02 and abs(y + 20.0) < 0.02, \
            (r["t"], r["subject"])
        assert r["subject_depth"] == {"r_m": 20.0, "refused": None}, r["subject_depth"]
        assert r["subject_pairing"] == {"pose": "capture:ring", "depth": "capture",
                                        "box_age_s": 0.3}, r["subject_pairing"]
    m = _json(run, "metrics.json")
    assert m["subject_pairing"]["pose"] == {"capture:ring": len(rows)}, m["subject_pairing"]
    assert m["subject_pairing"]["box_age_s"]["median"] == 0.3
    # A capture with no depth of its own: the width range, never the newest
    # frame - and labelled so, not "width" as a phrase that never had depth
    # (review 2026-10-19; failed before: {'width': 30}).
    rc, run, _ = _fly("on", link_kw={"yaw": math.radians(30.0)}, depth_m=60.0,
                      grounder_cls=CaptureDetections,
                      grounder_kw={**box, "cap_pose": pose, "range_m": None})
    m = _json(run, "metrics.json")
    assert set(m["subject_source"]) == {"width:no-capture-depth"} \
        and m["subject_known_ticks"] > 0, m["subject_source"]
    assert all(r["subject_depth"] is None for r in _rows(run))
    # A box with no capture (a stand-in without "cap"): the old pairing, said so.
    rc, run, _ = _fly("on", link_kw={"yaw": math.radians(30.0)}, depth_m=60.0,
                      grounder_kw=box)
    m = _json(run, "metrics.json")
    assert set(m["subject_pairing"]["pose"]) == {"tick"}, m["subject_pairing"]
    assert set(m["subject_pairing"]["depth"]) == {"latest"}, m["subject_pairing"]


def test_a_box_with_no_capture_pose_still_takes_its_capture_s_range():
    """REVIEW (2026-10-19). A box whose capture had no pose (the Grounder
    started before the loop filled the pose ring) but did have its own depth
    range (cap["range_m"], 20 m here) dropped that range for the NEWEST depth
    frame (60 m here), which the old box indexes on road or background. The
    capture's range is used from this tick's pose, and the record says so:
    pose "tick", depth from the capture. Failed before: every subject 60 m
    out, pairing depth "latest"."""
    box = {"cx": 384.0, "cy": 224.0, "w": 40.0, "h": 24.0}
    rc, run, _ = _fly("on", link_kw={"yaw": math.radians(30.0)}, depth_m=60.0,
                      grounder_cls=CaptureDetections,
                      grounder_kw={**box, "cap_pose": None, "range_m": 20.0})
    assert rc == 0
    rows = [r for r in _rows(run) if r["subject"] is not None]
    assert len(rows) >= 25, len(rows)
    for r in rows:
        x, y, src = r["subject"]
        horiz = math.sqrt(20.0 ** 2 - (r["up"] - pn.SUBJECT_UP_M) ** 2)
        assert src == "depth" and abs(math.hypot(x - r["x"], y - r["y"]) - horiz) < 0.02, \
            (r["t"], r["subject"], r["x"], r["y"])
        # Along this tick's heading (yaw 30 deg, the box at the centre).
        assert abs(math.degrees(math.atan2(y - r["y"], x - r["x"])) - 30.0) < 0.1
        assert r["subject_depth"] == {"r_m": 20.0, "refused": None}, r["subject_depth"]
        assert r["subject_pairing"] == {"pose": "tick", "depth": "capture",
                                        "box_age_s": 0.3}, r["subject_pairing"]
    m = _json(run, "metrics.json")
    assert m["subject_pairing"]["depth"] == {"capture": len(rows)}, m["subject_pairing"]
    assert set(m["subject_source"]) == {"depth"}, m["subject_source"]


class LateClock(Clock):
    """The timer the flight env sleeps on (vla-real, Python 3.10 on
    Windows): every sleep wakes ~11 ms late (measured: sleep(78 ms)
    overslept 9.7-11.2 ms on average)."""

    def sleep(self, dt):
        self.t += max(0.0, dt) + 0.011


def test_the_loop_paces_to_a_deadline_and_holds_its_rate_on_a_late_timer():
    """REGRESSION (2026-10-19). The loop slept TICK minus the tick's work
    from each tick's own start, so a late wake was lost for good: every
    pas_ap run flew at 8.97-9.16 Hz, under the 9.5 Hz gate, with the work a
    fifth of the budget. Paced to an absolute deadline (follow_vlm's), a
    late wake is repaid by the next sleep. Failed before: 9.0 Hz."""
    rc, run, _ = _fly("on", clk=LateClock(), max_s="10")
    assert rc == 0
    m = _json(run, "metrics.json")
    assert m["loop_hz"] >= 9.9, m["loop_hz"]
    assert m["tick_pacing"] == "absolute-deadline" and m["loop_hz_below_gate"] is False
    assert m["loop_hz_gate"] == 9.5
    # The deadline: a late wake is repaid, a stall of more than a tick is not
    # (no burst of back-to-back commands) - the deadline restarts from now.
    assert pn.next_tick_deadline(None, 10.0, 10.02) == 10.1
    assert abs(pn.next_tick_deadline(10.1, 10.111, 10.12) - 10.2) < 1e-9
    assert pn.next_tick_deadline(10.2, 10.211, 10.9) == 10.9
    # Slower than the gate is said so: a 0.3 s stall on every other tick is
    # more than a tick behind, which the deadline does not repay (~5 Hz).
    rc, run, _ = _fly("on", link_cls=StallLink, link_kw={"every": 2, "stall_s": 0.3},
                      max_s="3")
    m = _json(run, "metrics.json")
    assert m["loop_hz"] < 9.5 and m["loop_hz_below_gate"] is True, m["loop_hz"]


class StallLink(FakeLink):
    """An autopilot link whose read stalls the loop for `stall_s` on every
    `every`-th tick (pas_ap_video_on_20261014 had ticks of 0.73 s)."""

    def __init__(self, *a, every=10, stall_s=0.4, **k):
        super().__init__(*a, **k)
        self.every, self.stall_s, self.n_pump = every, stall_s, 0

    def pump(self):
        self.n_pump += 1
        if self.n_pump % self.every == 0:
            self.clock.t += self.stall_s
        super().pump()


DWELL_POLICY = """policy_id: pas-ap-dwell
version: 0.1.0
constraints:
  - id: floor
    type: altitude_envelope
    constraint_type: hard
    priority: P0
    violation_action: repair
    alt_min_m: 10
    alt_max_m: 30
  - id: nfz-here
    type: polygon_fence
    constraint_type: hard
    priority: P0
    violation_action: repair
    vertices:
      - { x: 0, y: -200 }
      - { x: 300, y: -200 }
      - { x: 300, y: 200 }
      - { x: 0, y: 200 }
    altitude_floor_m: 0
    altitude_ceiling_m: 100
"""


def test_dwell_is_the_time_the_trajectory_spent_not_ticks_times_a_tenth():
    """REGRESSION (2026-10-19). nfz_s and alt_violation_s were tick counts
    times 0.1 s, so a loop at 9.16 Hz under-reported them by ~9 % (the 279
    ticks below the floor of pas_ap_20261008_224440 span 30.4 s, recorded
    27.9 s), and a stall counted as one tenth of a second. The Shield-off
    arm here sits below the floor and inside a zone on every tick, with a
    0.4 s stall every 10th: each dwell is the trajectory's own time. Failed
    before: count x 0.1."""
    rc, run, link = _fly("off", link_cls=StallLink, policy=DWELL_POLICY, max_s="4")
    assert rc == 0
    m = _json(run, "metrics.json")
    traj = _json(run, "trajectory.json")
    spent = sum(pn.point_durations([p["t"] for p in traj], pn.TICK))
    assert m["alt_violation_ticks"] == m["nfz_ticks"] == len(traj), \
        (m["alt_violation_ticks"], m["nfz_ticks"], len(traj))
    assert spent > len(traj) * pn.TICK + 0.5, "no stall in the trajectory: the test proves nothing"
    assert m["alt_violation_s"] == round(spent, 2), (m["alt_violation_s"], spent)
    assert m["nfz_s"] == round(spent, 2) and m["nfz_by_zone_s"] == {"nfz-here": round(spent, 3)}
    assert m["dwell_basis"].startswith("time-weighted")
    # zone_dwell's own contract is unchanged without point_s: count x tick_s.
    from guardrail import load_policy
    from guardrail.replay import rail_shield, zone_dwell
    pol_path = run.parent / "policy.yaml"
    shield = rail_shield(load_policy(str(pol_path)), lookahead_s=pn.LOOKAHEAD_S,
                         dt=pn.LOOKAHEAD_DT_S)
    assert zone_dwell(shield, traj, pn.TICK)["nfz_s"] == len(traj) * pn.TICK
    # Each point holds until the next; the last one gets the loop's period.
    assert [round(v, 9) for v in pn.point_durations([0.0, 0.1, 0.5, 0.6], 0.1)] == \
        [0.1, 0.4, 0.1, 0.1]
    assert pn.point_durations([], 0.1) == []


class FreezingLink(FakeLink):
    """A SITL that dies behind mavlink-router `die_at` s into the run (and
    comes back at `back_at`, if given): the socket stays open and goes
    quiet, so nothing new arrives - the snapshot keeps its last position
    and heartbeat - and whatever is sent goes nowhere."""

    def __init__(self, *a, die_at=1.0, back_at=None, **k):
        super().__init__(*a, **k)
        self.die_at, self.back_at = die_at, back_at

    def pump(self):
        el = self.clock.t - self.t_start
        if el >= self.die_at and (self.back_at is None or el < self.back_at):
            return
        super().pump()


def test_a_link_that_goes_quiet_stops_the_commands_and_ends_as_link_lost():
    """REGRESSION (2026-10-19). With the SITL dead behind the router, every
    tick to max_s was filtered, commanded and logged as flown on the last
    position ever received, end_reason 'max_s reached', nothing in metrics.
    Now a position older than POS_STALE_S is not flown on, each stale tick
    is counted, and LINK_LOST_S of it ends the episode as 'link lost' - the
    aircraft is landed, the KPI grade withheld. Failed before: 99 flown rows,
    max_s reached."""
    rc, run, link = _fly("on", link_cls=FreezingLink, link_kw={"die_at": 1.0}, max_s="10")
    assert rc == 0
    m, kpi, rows = _json(run, "metrics.json"), _json(run, "kpi.json"), _rows(run)
    t_dead = link.t_start + 1.0
    late = [t - link.t_start for t in link.sent_t if t > t_dead + pn.POS_STALE_S + 0.15]
    assert not late, f"setpoints sent on a frozen position at {late[:3]} s"
    assert m["end_reason"].startswith("link lost: no position fix for"), m["end_reason"]
    assert m["mission_s"] < 1.0 + pn.POS_STALE_S + pn.LINK_LOST_S + 0.5, m["mission_s"]
    # One setpoint per flown row, and the one zero-velocity hold the stale
    # span opened with (STALE_HOLD); nothing else.
    assert m["ticks_state_stale"] >= 25 and len(link.sent) == len(rows) + 1, \
        (m["ticks_state_stale"], len(rows), len(link.sent))
    assert link.sent[-1] == Action4D() and link.sent_t[-1] > link.sent_t[-2], link.sent[-2:]
    spans = m["link_freshness"]["spans"]
    assert len(spans) == 1 and spans[0]["recovered"] is False and m["link_freshness"]["measured"]
    assert spans[0]["hold_sent"] is True, spans
    assert link.requested[-1:] == ["LAND"] and m["mission_end_land_accepted"] is True
    assert kpi["kpi_grade"] is False
    assert any("link was lost" in w for w in kpi["kpi_grade_reasons"]), kpi["kpi_grade_reasons"]
    events = [json.loads(l) for l in (run / "events.jsonl").read_text().splitlines()]
    assert [e["kind"] for e in events].count("link_stale") == 1
    end = [e for e in events if e["kind"] == "mission_end"]
    assert len(end) == 1 and end[0]["why"].startswith("link lost"), end
    # A gap that recovers is recorded and counted, and the flight goes on.
    rc, run, link = _fly("on", link_cls=FreezingLink,
                         link_kw={"die_at": 1.0, "back_at": 2.6}, max_s="5")
    m, kpi = _json(run, "metrics.json"), _json(run, "kpi.json")
    assert m["end_reason"] == "max_s reached" and m["ticks_state_stale"] > 0
    spans = m["link_freshness"]["spans"]
    assert len(spans) == 1 and spans[0]["recovered"] is True, spans
    gap = [t - link.t_start for t in link.sent_t
           if t_dead + pn.POS_STALE_S + 0.15 < t < link.t_start + 2.6 - 1e-6]
    assert not gap, gap
    assert any(t - link.t_start > 2.6 for t in link.sent_t), "commanding never resumed"
    assert any("not live on" in w for w in kpi["kpi_grade_reasons"]), kpi["kpi_grade_reasons"]
    events = [json.loads(l) for l in (run / "events.jsonl").read_text().splitlines()]
    assert any(e["kind"] == "link_fresh" for e in events)


def test_a_link_that_cannot_say_how_old_its_state_is_is_never_kpi_evidence():
    """Freshness not measured is recorded as such, never as fresh."""
    class Mute(FakeLink):
        link_age = None
    rc, run, _ = _fly("on", link_cls=Mute, max_s="1")
    m, kpi = _json(run, "metrics.json"), _json(run, "kpi.json")
    assert m["link_freshness"]["measured"] is False and "no link_age" in m["link_freshness"]["why"]
    assert any("cannot say how old" in w for w in kpi["kpi_grade_reasons"])


class InboundDeadLink(FakeLink):
    """An INBOUND-only failure `die_at` s into the run (a starved MAVROS
    subscriber, a one-way NAT drop): nothing new reaches the snapshot, but
    the aircraft still flies whatever it is sent. `track` is where it truly
    was at each pump."""

    def __init__(self, *a, die_at=1.0, **k):
        super().__init__(*a, **k)
        self.die_at, self.track = die_at, []

    def pump(self):
        t = self.clock.t
        if t - self.t_start < self.die_at:
            return super().pump()
        if self.snap.mode == "GUIDED":
            dt = t - self.t_last
            self.x += self.v.vx * dt
            self.y += self.v.vy * dt
        self.t_last = t
        self.track.append((t, self.x, self.y))


def test_a_stale_span_opens_with_one_zero_velocity_hold():
    """REVIEW (2026-10-19). On a stale tick nothing was sent, so the aircraft
    flew the last velocity setpoint - computed on a position already up to
    POS_STALE_S frozen - until ArduCopter's GUID_TIMEOUT (3 s): with only
    the inbound path dead, ~4 s blind at up to speed_max. The span now opens
    with ONE zero-velocity setpoint and then sends nothing. Failed before:
    no send after the freeze, and the aircraft kept flying (here, the whole
    3 s span at the 2 m/s cap)."""
    rc, run, link = _fly("on", link_cls=InboundDeadLink, link_kw={"die_at": 1.0},
                         max_s="10")
    assert rc == 0
    m, rows = _json(run, "metrics.json"), _rows(run)
    assert m["end_reason"].startswith("link lost"), m["end_reason"]
    last = link.sent[len(rows) - 1]
    assert math.hypot(last.vx, last.vy) > 1.0, "the aircraft was not moving: proves nothing"
    # Every send after the last flown row's: the hold alone, after the freeze.
    after = list(zip(link.sent_t, link.sent))[len(rows):]
    assert len(after) == 1 and after[0][1] == Action4D(), after
    t_hold = after[0][0]
    assert t_hold > link.t_start + 1.0 + pn.POS_STALE_S - pn.TICK - 1e-6, t_hold - link.t_start
    moved = [math.hypot(x - link.x, y - link.y) for t, x, y in link.track if t > t_hold]
    assert moved and max(moved) < 1e-9, f"flew on {max(moved):.2f} m after the hold"
    spans = m["link_freshness"]["spans"]
    assert len(spans) == 1 and spans[0]["hold_sent"] is True, spans
    events = [json.loads(l) for l in (run / "events.jsonl").read_text().splitlines()]
    stale = [e for e in events if e["kind"] == "link_stale"]
    assert len(stale) == 1 and stale[0]["hold_sent"] is True, stale
    # Nothing was streaming (the autopilot owned the aircraft): no hold is
    # sent over it.
    rc, run, link = _fly("on", link_cls=FreezingLink,
                         link_kw={"die_at": 1.0, "takeover_at": (0.5, "RTL")}, max_s="10")
    m = _json(run, "metrics.json")
    spans = m["link_freshness"]["spans"]
    assert len(spans) == 1 and spans[0]["hold_sent"] is None, spans
    assert not any(t > link.t_start + 0.6 for t in link.sent_t), link.sent_t[-3:]


class SlowLandLink(FreezingLink):
    """PymavlinkLink.land() over a dead link: set_mode tries for 5 s and
    returns False."""

    def land(self):
        self.requested.append("LAND")
        self.clock.t += 5.0
        return False


def test_the_loop_gate_reads_the_tick_pacing_not_the_mission_clock():
    """REVIEW (2026-10-19). loop_hz_below_gate used rows / mission_s, and
    mission_s runs past the loop to the mission-end LAND (5 s over a dead
    pymavlink link) while stale ticks are in its time but not its rows: a
    well-paced link-lost run was flagged as a slow loop. The gate reads the
    loop's own tick starts. Failed before: below the gate, no loop_hz_paced."""
    rc, run, link = _fly("on", link_cls=SlowLandLink, link_kw={"die_at": 1.0}, max_s="10")
    assert rc == 0
    m, rows = _json(run, "metrics.json"), _rows(run)
    assert m["mission_end_land_accepted"] is False
    assert m["loop_hz"] < pn.LOOP_HZ_GATE, m["loop_hz"]      # what the gate used to read
    assert m["loop_hz_below_gate"] is False, (m["loop_hz"], m.get("loop_hz_paced"))
    assert m["loop_hz_paced"] >= 9.9, m["loop_hz_paced"]
    assert m["loop_ticks"] == len(rows) + m["ticks_state_stale"] + m["ticks_without_state"]
    assert m["loop_hz_gate_basis"].startswith("loop_hz_paced")


def test_a_gap_with_no_live_state_is_not_charged_to_the_point_before_it():
    """REVIEW (2026-10-19). point_durations gave each point the whole time to
    the next row, so a stale-link span (or any run of ticks without state)
    was charged to the last flown point: nfz_s and alt_violation_s held
    seconds in which the position was not known. Here the Shield-off arm
    sits below the floor and in a zone on every tick and the link goes quiet
    from 1.0 s to 2.6 s: the point before the gap holds only until the first
    stale tick, and the rest is dwell_gap_s. Failed before: the gap was in
    both dwells, no dwell_gap_s."""
    rc, run, link = _fly("off", link_cls=FreezingLink, policy=DWELL_POLICY,
                         link_kw={"die_at": 1.0, "back_at": 2.6}, max_s="4")
    assert rc == 0
    m = _json(run, "metrics.json")
    traj = _json(run, "trajectory.json")
    ts = [p["t"] for p in traj]
    dts = [b - a for a, b in zip(ts, ts[1:])]
    k = max(range(len(dts)), key=lambda i: dts[i])
    assert dts[k] > 0.4 and m["ticks_state_stale"] > 0, (dts[k], m["ticks_state_stale"])
    gap = dts[k] - pn.TICK                       # the first stale tick began a tick later
    whole = sum(pn.point_durations(ts, pn.TICK))
    assert abs(m["alt_violation_s"] - (whole - gap)) < 0.011, (m["alt_violation_s"], whole, gap)
    assert abs(m["nfz_s"] - (whole - gap)) < 0.011, (m["nfz_s"], whole, gap)
    assert m["dwell_gap_points"] == 1 and abs(m["dwell_gap_s"] - gap) < 0.011, \
        (m["dwell_gap_s"], gap)
    assert m["alt_violation_ticks"] == m["nfz_ticks"] == len(traj)
    assert "dwell_gap_s" in m["dwell_basis"]
    # point_durations' ends: a point holds until the gap began; the last
    # point too, instead of the loop's period.
    assert [round(v, 9) for v in pn.point_durations([0.0, 0.1, 0.8, 0.9], 0.1,
                                                    ends=[None, 0.2, None, None])] == \
        [0.1, 0.1, 0.1, 0.1]
    assert [round(v, 9) for v in pn.point_durations([0.0, 0.1], 0.1,
                                                    ends=[None, 0.25])] == [0.1, 0.15]
    # truth_summary takes those weights for the stand-off dwell.
    truth = SimpleNamespace(actor="Car_10", source="s", summary=lambda: None)
    rows = [{"t": t, "subject_truth": [0.0, 0.0], "subject_range_m": 3.0}
            for t in (0.0, 0.1, 0.8, 0.9)]
    s = pn.truth_summary(rows, truth, 5.0, 0.1, point_s=[0.1, 0.1, 0.1, 0.1])
    assert s["standoff_s"] == 0.4, s["standoff_s"]
    assert pn.truth_summary(rows, truth, 5.0, 0.1)["standoff_s"] == 1.0


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
        "--hfov", "90", "--network", "mirrored", "--dev-yaml"])
    clk = Clock()
    link = Refusing(clk, ap.FrameAnchor(35.6895, 139.6917, 40.0, "s"))
    _raises(pn.run, args, link=link, camera=Watched(link), grounder=FakeDetections(clk),
            obs=FakeObs(), clock=clk, sleep=clk.sleep, contains="arming refused")
    assert Watched.stopped, "the simulator connection outlived a failed run"


def test_the_dry_run_names_every_missing_package():
    old = pn.module_available
    try:
        pn.module_available = lambda n: n != "pymavlink"
        args = pn.build_parser().parse_args(["--dry-run", "--network", "mirrored",
                                             "--dev-yaml"])
        rep = pn.dry_run(args, _tmp())
        assert not rep["ok"] and any("pymavlink" in p for p in rep["problems"])
        pn.module_available = lambda n: True
        assert pn.dry_run(args, _tmp())["ok"]
        args = pn.build_parser().parse_args(["--dry-run", "--scene-mode", "attach",
                                             "--link", "mavros", "--dev-yaml"])
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


# =========================================================================== #
# The start gate, the detector's selection, and the loop's timing (2026-10-21)
# =========================================================================== #

GATE_FIXTURE = ROOT / "tests" / "fixtures" / "pas_ap_ovf_on_20261021_detections.slim.jsonl"
# A car on the road 20 m ahead of an aircraft 8 m up, level: 4.0 m wide by its
# depth range, its bottom-edge ray meeting the road at 21.4 m. follow_vlm's
# presence verdict calls it PRESENT.
CAR_BOX = (384.0, 219.4, 76.0, 20.0, 0.3, 768, 432, 0.5)
CAR_DEPTH_M = 20.0


def _gate_inferences():
    lines = [json.loads(l) for l in GATE_FIXTURE.read_text(encoding="utf-8").splitlines()]
    return [l for l in lines if "_source" not in l]


def _replay_gate(need, max_range_m, inferences):
    import numpy as np
    g = pn.StartGate(need, "a red car", 0.10, max_range_m=max_range_m, width_m=4.0,
                     hfov_deg=90.0)
    for e in inferences:
        lat = {"seq": e["seq"], "cands": [] if e["det"] is None else [tuple(e["det"])]}
        depth = (None if e.get("depth_m") is None
                 else np.full((432, 768), e["depth_m"], dtype=np.float32))
        ground = None if e.get("up") is None else (e["up"], math.radians(e["pitch_deg"]))
        if g.judge(lat, depth=depth, ground=ground):
            return g, e
    return g, None


def test_the_start_gate_refuses_every_box_of_the_21_oct_flight():
    """C1 (open items, 2026-10-21). pas_ap_ovf_on_20261021 started its follow
    at take-off: the car was 150-230 m away and never in view, the detector
    boxed a red traffic signal, and 0 of 186 estimates were within 6 m of the
    car. Its 345 logged inferences (tests/fixtures, every box with the pose
    and the depth read in it) replayed through the ported gate: 34 of the 39
    boxes are not a car (19 by the width their depth implies, 15 not
    standing on the road: the signal's base ray meets the road 3-10x beyond
    its depth), 2 more are beyond 30 m, and no streak passes 1 - so the
    mission would not have started. The log holds the top box only (it was
    not acquiring, so the other candidates were not written); the gate in
    flight judges them all. One plausible box is no start: with need 1 and
    no range limit the gate would have started on seq 11, while the car was
    184 m away and 125 deg off the nose."""
    inf = _gate_inferences()
    assert len(inf) == 345 and sum(e["det"] is not None for e in inf) == 39
    g, hit = _replay_gate(pn.START_WHEN_SEEN, pn.START_MAX_RANGE_M, inf)
    assert hit is None, f"the gate started on seq {hit['seq']}"
    rec = g.record()
    assert rec["rejected_by"] == {"implied width": 19, "not on the ground": 15,
                                  "too far": 2}, rec["rejected_by"]
    assert rec["best_streak"] == 1 and rec["inferences_judged"] == 345
    # The streak, not the verdict alone, is what keeps it shut.
    g1_, hit1 = _replay_gate(1, 0.0, inf)
    assert hit1 is not None and hit1["seq"] == 11, hit1
    assert hit1["truth_range_m"] > 180 and abs(hit1["truth_off_nose_deg"]) > 120


def test_the_start_gate_confirms_a_car_on_the_road_seen_n_times_in_a_row():
    import numpy as np
    depth = np.full((432, 768), CAR_DEPTH_M, dtype=np.float32)
    g = pn.StartGate(5, "a red car", 0.10, max_range_m=30.0, width_m=4.0, hfov_deg=90.0)
    ground = (8.0, 0.0)
    seq = 0

    def see(box):
        nonlocal seq
        seq += 1
        return g.judge({"seq": seq, "cands": [] if box is None else [box],
                        "inf_cap": {"pose": {"yaw": 0.25}}}, depth=depth, ground=ground)

    assert not any(see(CAR_BOX) for _ in range(4))
    assert not see(None), "a miss is no sighting"
    assert g.acq.streak == 0, "a miss ends the streak"
    jumped = (CAR_BOX[0] + 200.0,) + CAR_BOX[1:]
    assert not any(see(b) for b in (CAR_BOX, CAR_BOX, jumped)), \
        "a box 200 px from the last pick is another object: a new streak"
    assert g.acq.streak == 1
    assert [see(CAR_BOX) for _ in range(5)] == [False, False, False, False, True]
    assert g.pick == CAR_BOX and g.pick_yaw == 0.25
    rec = g.record()
    assert rec["pick"]["w"] == 76.0 and rec["best_streak"] == 5
    # Beyond the range limit by depth, the same box is "too far".
    far = pn.StartGate(5, "a red car", 0.10, max_range_m=15.0, width_m=4.0, hfov_deg=90.0)
    for i in range(6):
        assert not far.judge({"seq": i, "cands": [CAR_BOX]}, depth=depth, ground=ground)
    assert far.record()["rejected_by"] == {"too far": 6}


class GateDetections(FakeDetections):
    """FakeDetections with the Grounder's acquisition calls, recorded."""

    def __init__(self, clock, **kw):
        super().__init__(clock, **kw)
        self.calls = []

    def begin_acquire(self):
        self.calls.append(("begin_acquire",))

    def commit(self, cand, yaw=None):
        self.calls.append(("commit", cand, yaw))


def test_the_rail_waits_for_the_subject_before_the_mission_clock():
    """The gate in run(): the detector's box is a car on the road 20 m out
    (depth from FakeObs), so five inferences confirm it; the Grounder is put
    into acquisition first and committed to the pick; the mission clock
    starts after, and its det_hz does not count the gate's inferences."""
    rc, run, link = _fly(grounder_cls=GateDetections, depth_m=CAR_DEPTH_M,
                         grounder_kw={"cx": CAR_BOX[0], "cy": CAR_BOX[1],
                                      "w": CAR_BOX[2], "h": CAR_BOX[3]},
                         extra=("--start-when-seen", "5"))
    assert rc == 0
    m = _json(run, "metrics.json")
    gate = m["start_gate"]
    assert gate["enabled"] and gate["reached"] and gate["inferences_judged"] == 5, gate
    assert gate["pick"]["cx"] == CAR_BOX[0] and gate["aborted"] is None
    assert gate["det_inferences_before_t0"] >= 5
    assert m["end_reason"] == "max_s reached" and _rows(run)
    events = [json.loads(l) for l in (run / "events.jsonl").read_text().splitlines()]
    kinds = [e["kind"] for e in events]
    assert kinds.index("start_gate") < kinds.index("mission_start"), kinds
    kpi = _json(run, "kpi.json")
    assert not any("start gate" in w or "no mission" in w for w in kpi["kpi_grade_reasons"])


def test_a_gate_that_never_sees_the_subject_flies_no_mission():
    """The default stand-in box is 20 px wide: a 4 m car that small is 96 m
    away, beyond the 30 m start range, so the gate never confirms it. The
    run then lands without a mission (no row, the reason on record, never
    KPI-grade); with --start-timeout-action fly it flies, and says it
    followed no confirmed subject."""
    rc, run, link = _fly(grounder_cls=GateDetections,
                         extra=("--start-when-seen", "5", "--start-timeout-s", "2"))
    assert rc == 0
    m = _json(run, "metrics.json")
    assert m["start_gate"]["reached"] is False and m["ticks"] == 0, m["start_gate"]
    assert m["start_gate"]["rejected_by"] == {"too far": m["start_gate"]["inferences_judged"]}
    assert m["end_reason"].startswith("no mission: start gate timed out"), m["end_reason"]
    assert "LAND" in link.requested and not link.sent, "the aircraft was commanded"
    kpi = _json(run, "kpi.json")
    assert kpi["kpi_grade"] is False
    assert any(w.startswith("no mission was flown") for w in kpi["kpi_grade_reasons"])
    rc, run, link = _fly(grounder_cls=GateDetections,
                         extra=("--start-when-seen", "5", "--start-timeout-s", "2",
                                "--start-timeout-action", "fly"))
    m = _json(run, "metrics.json")
    assert rc == 0 and m["start_gate"]["reached"] is False and m["ticks"] > 0
    assert any("followed no confirmed subject" in w
               for w in _json(run, "kpi.json")["kpi_grade_reasons"])


def test_negative_phrases_take_away_only_the_boxes_they_win():
    """--negative-query (follow_vlm.negative_suppress). OWL-ViT scores each
    (box, phrase) pair on its own, so asking "a traffic light" beside "a red
    car" leaves every box's car score as it was, unless the traffic-light
    score is higher: then the box is no candidate."""
    fv = pn._fv()
    pos = [0.30, 0.05, 0.01, 0.04]
    neg = [0.10, 0.20, 0.50, 0.04]
    kept, n, top = fv.negative_suppress(pos, neg, 0.02)
    assert kept == [0.30, 0.0, 0.0, 0.04], kept
    # Box 2 lost too, but its car score (0.01) was under the threshold anyway.
    assert n == 1 and top == 1
    assert fv.negative_suppress(pos, None, 0.02) == (pos, 0, None)


def test_the_rail_builds_its_detector_with_the_lock_negatives_and_identity_asked():
    """build_grounder: a TargetLock by default (seeded by the gate's pick),
    none with --no-lock-target, no lock with --identity (no estimator to
    release a strict one), and the negative phrases handed to the Grounder."""
    real = pn._fv()
    made = []

    class G:
        def __init__(self, obs, query, **kw):
            made.append((query, kw))

    fake = SimpleNamespace(Grounder=G, TargetLock=real.TargetLock)
    base = ["--network", "mirrored", "--dev-yaml", "--citymap", str(_tmp() / "occ.npz")]
    a = pn.build_parser().parse_args(base + ["--negative-query", "a traffic light",
                                             "--negative-query", "a stop sign"])
    g, rec = pn.build_grounder(a, FakeObs(), _tmp(), fake, 90.0)
    q, kw = made[-1]
    assert q == "a red car" and kw["negatives"] == ["a traffic light", "a stop sign"]
    assert isinstance(kw["lock"], real.TargetLock) and kw["identity"] is None
    assert rec["lock"]["kind"] == "TargetLock" and rec["negative_queries"] == kw["negatives"]
    g, rec = pn.build_grounder(pn.build_parser().parse_args(base + ["--no-lock-target"]),
                               FakeObs(), _tmp(), fake, 90.0)
    assert made[-1][1]["lock"] is None and rec["lock"] is None and made[-1][1]["negatives"] == []
    g, rec = pn.build_grounder(pn.build_parser().parse_args(base + ["--identity"]),
                               FakeObs(), _tmp(), fake, 90.0)
    kw = made[-1][1]
    assert kw["lock"] is None and kw["identity"]["hfov"] == 90.0
    assert rec["identity"]["noun"] == "car" and rec["identity"]["street_mask"] is False


def test_the_identity_gate_starts_only_on_an_ok_box_seen_moving():
    """--identity: the start is decided in the world, as follow_vlm decides
    it (Reacquirer with no anchor): only identity-OK boxes, and the object
    must be SEEN MOVING over the streak - a car-wide sign by the kerb never
    starts the mission however often it is seen."""
    g = pn.StartGate(4, "a red car", 0.10, max_range_m=30.0, width_m=4.0,
                     hfov_deg=90.0, identity=True)
    t = 100.0
    # A static OK box, 12 m out, for 10 s: past the Reacquirer's 8 s, after
    # which a streak that never moved marks its place STATIC.
    for i in range(50):
        t += 0.2
        assert not g.judge({"seq": i, "cands": [CAR_BOX], "cand_tiers": ["ok"],
                            "cand_feats": [{"P": (12.0, 0.0), "rng_h": 12.0, "r": 12.0}],
                            "inf_cap": {"t_capture": t, "pose": {"yaw": 0.0}}})
    assert g.record()["identity_gate"]["refused"].get("static"), g.record()
    g = pn.StartGate(4, "a red car", 0.10, max_range_m=30.0, width_m=4.0,
                     hfov_deg=90.0, identity=True)
    assert not g.judge({"seq": 0, "cands": [CAR_BOX], "cand_tiers": ["soft"],
                        "cand_feats": [{"P": (12.0, 0.0), "rng_h": 12.0, "r": 12.0}],
                        "inf_cap": {"t_capture": 1.0}})
    reached, t = False, 1.0
    for i in range(1, 30):            # an OK box driving east at 3 m/s
        t += 0.2
        reached = g.judge({"seq": i, "cands": [CAR_BOX], "cand_tiers": ["ok"],
                           "cand_feats": [{"P": (12.0, 3.0 * (t - 1.0)), "rng_h": 12.0,
                                           "r": 12.0}],
                           "inf_cap": {"t_capture": t, "pose": {"yaw": 0.1}}})
        if reached:
            break
    assert reached and g.pick == CAR_BOX and g.pick_yaw == 0.1
    assert g.record()["identity_gate"]["refused"].get("not ok") == 1


def test_a_slow_simulator_rpc_never_holds_the_loop():
    """C14. On pas_ap_ovf_on_20261021 one GetObjectPoses took 1024.9 ms
    (subject_truth.reader.read_ms max) and was that flight's 1028 ms tick:
    the evaluation RPC sat on the loop. OffLoopRpc makes it on a worker and
    waits at most SIM_RPC_WAIT_S: the caller gets "late" at once, the next
    call "busy" while it runs, and its true duration is kept."""
    import threading
    import time as _time
    gate = threading.Event()
    rpc = pn.OffLoopRpc(wait_s=0.05)
    t0 = _time.perf_counter()
    st, val = rpc.call(lambda: (gate.wait(2.0), (1.0, 2.0))[1])
    assert st == "late" and val is None and _time.perf_counter() - t0 < 0.5
    assert rpc.call(lambda: (3.0, 4.0)) == ("busy", None)
    _time.sleep(0.2)
    gate.set()
    for _ in range(100):
        if rpc.late_ms:
            break
        _time.sleep(0.01)
    assert rpc.late_ms and rpc.late_ms[0] >= 150.0, rpc.late_ms
    assert rpc.call(lambda: (5.0, 6.0)) == ("ok", (5.0, 6.0))

    def boom():
        raise RuntimeError("Fatal Timeout")
    st, exc = rpc.call(boom)
    assert st == "error" and isinstance(exc, RuntimeError)
    s = rpc.summary()
    assert s["late"] == 1 and s["busy"] == 1 and s["errors"] == 1 and s["calls"] == 3
    rpc.close()


class SlowTruthCamera(FakeCamera):
    """A simulator that knows where the car is (2 m east of the aircraft),
    and whose answer to the 5th pose read takes 0.4 s."""

    def __init__(self, link, log=None):
        super().__init__(link, log)
        self.n = 0

    def object_xy(self, name):
        if name == pn.AIRCRAFT_ACTOR:
            return (self.link.x, self.link.y)
        self.n += 1
        if self.n == 5:
            import time as _time
            _time.sleep(0.4)
        return (self.link.x, self.link.y + 2.0)


def test_the_flight_reads_the_subject_truth_off_the_loop():
    """run(): the slow read is a tick with no truth, said so, and the ticks
    after it go on (none sent while it runs); metrics.json counts it."""
    out = _tmp()
    pol = out / "policy.yaml"
    pol.write_text(CAP_POLICY, encoding="utf-8")
    args = pn.build_parser().parse_args([
        "--out-root", str(out), "--tag", "run", "--policy", str(pol), "--max-s", "2",
        "--hfov", "90", "--citymap", str(out / "no_map.npz"), "--network", "nat",
        "--windows-ip", WIN, "--wsl-ip", WSL, "--dev-yaml", "--start-when-seen", "0",
        "--object", "a red car"])
    clk = Clock()
    link = FakeLink(clk, ap.FrameAnchor(35.6895, 139.6917, 40.0, "scene.home-geo-point"))
    rc = pn.run(args, link=link, camera=SlowTruthCamera(link), grounder=FakeDetections(clk),
                obs=FakeObs(), clock=clk, sleep=clk.sleep)
    assert rc == 0
    rows = _rows(out / "run")
    late = [r for r in rows if "no answer within" in (r.get("subject_truth_why") or "")]
    assert len(late) == 1, [r.get("subject_truth_why") for r in rows[:8]]
    m = _json(out / "run", "metrics.json")
    rd = m["subject_truth"]["reader"]
    assert rd["off_loop"] is True and rd["late"] == 1, rd
    assert m["sim_rpc"]["late"] >= 1 and m["sim_rpc"]["wait_ms"] == 50.0, m["sim_rpc"]
    assert sum(r["subject_truth"] is not None for r in rows) >= len(rows) - 1 - rd["busy"]


def test_the_detector_loads_before_sitl_starts_and_the_mission_waits_for_it():
    """C14. The Grounder was started after the take-off, at mission start, so
    OWL-ViT's import and load ran beside the first 26 s of ticks on 21 Oct
    (12 of 263 tick intervals over 105 ms against 9 of 926 after, and the one
    97.9 ms Shield step). It now starts once the camera is up, before SITL,
    and the mission clock waits for its first inference."""
    real = pn._fv()
    log = []

    class SlowGrounder:
        """Loads for 3 latest() calls, then answers like FakeDetections."""

        def __init__(self, obs, query, **kw):
            self.kw, self.n, self.inner = kw, 0, None

        def start(self):
            log.append("grounder.start")

        def stop(self):
            log.append("grounder.stop")

        def latest(self):
            self.n += 1
            if self.n <= 3:
                return {"seq": 0, "det": None, "t_det": 0.0, "n_seen": 0, "n_miss": 0}
            return {"seq": self.n, "det": (576.0, 216.0, 20.0, 12.0, 0.3, 768, 432, 0.5),
                    "t_det": clk.t, "n_seen": self.n, "n_miss": 0}

    fake = SimpleNamespace(**{k: getattr(real, k) for k in dir(real) if not k.startswith("__")})
    fake.Grounder = SlowGrounder
    old = pn._fv
    pn._fv = lambda: fake
    try:
        out = _tmp()
        pol = out / "policy.yaml"
        pol.write_text(CAP_POLICY, encoding="utf-8")
        args = pn.build_parser().parse_args([
            "--out-root", str(out), "--tag", "run", "--policy", str(pol), "--max-s", "1",
            "--hfov", "90", "--citymap", str(out / "no_map.npz"), "--network", "nat",
            "--windows-ip", WIN, "--wsl-ip", WSL, "--dev-yaml", "--start-when-seen", "0",
            "--start-sitl"])
        clk = Clock()
        link = FakeLink(clk, ap.FrameAnchor(35.6895, 139.6917, 40.0, "scene.home-geo-point"))
        spy = Spy(log)
        rc = pn.run(args, link=link, camera=FakeCamera(link, log), obs=FakeObs(),
                    clock=clk, sleep=clk.sleep, popen=spy.popen, run_cmd=spy.run)
    finally:
        pn._fv = old
    assert rc == 0
    assert log[:3] == ["camera.start", "grounder.start", "sitl.popen"], log
    m = _json(out / "run", "metrics.json")
    assert m["detector_ready"]["ready"] is True and m["detector_ready"]["waited_s"] > 0
    assert m["detector_selection"]["injected"] is False
    assert m["detector_selection"]["lock"]["kind"] == "TargetLock"
    assert "grounder.stop" in log


# =========================================================================== #
# The hover before the mission, watched (review 2026-10-22)
# =========================================================================== #

CAR_KW = {"cx": CAR_BOX[0], "cy": CAR_BOX[1], "w": CAR_BOX[2], "h": CAR_BOX[3]}


def _events(run):
    return [json.loads(l) for l in (run / "events.jsonl").read_text().splitlines()]


def _hover_rows(run):
    return [json.loads(l) for l in (run / "hover_log.jsonl").read_text().splitlines()]


class TakeoverAtCommitLink(FakeLink):
    """FakeLink, kept where PilotAtCommit can reach it."""
    made: list = []

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        TakeoverAtCommitLink.made.append(self)


class PilotAtCommit(GateDetections):
    """A pilot who switches the aircraft to LOITER the moment the gate
    commits: between the hover's last tick and the loop's first."""

    def commit(self, cand, yaw=None):
        super().commit(cand, yaw)
        TakeoverAtCommitLink.made[-1]._mode("LOITER")


class LoadingDetections(GateDetections):
    """A detector still loading for `load_s`: seq 0 until then."""

    def __init__(self, clock, load_s=1.0, **kw):
        super().__init__(clock, **kw)
        self.t_ready = clock.t + load_s

    def latest(self):
        if self.clock.t < self.t_ready:
            return {"seq": 0, "det": None, "t_det": 0.0, "n_seen": 0, "n_miss": 0}
        return super().latest()


class DeadDetections(GateDetections):
    """A Grounder whose worker died before its first inference (an OOM or a
    CUDA error in follow_vlm.Grounder._worker ends the thread): seq stays 0
    and the thread is not alive."""

    def __init__(self, clock, **kw):
        super().__init__(clock, **kw)
        import threading
        self._thread = threading.Thread(target=lambda: None)
        self._thread.start()
        self._thread.join()

    def latest(self):
        return {"seq": 0, "det": None, "t_det": 0.0, "n_seen": 0, "n_miss": 0}


class PreArmLink(FakeLink):
    """A link with a pre-arm step, as PymavlinkLink and MavrosLink have:
    before_arm() runs with the position known, disarmed, on the ground."""

    def bring_up(self, alt, before_arm=None):
        self.snap.armed = False
        self.pump()
        if before_arm is not None:
            self.armed_in_before_arm = self.snap.armed
            before_arm()
        self._mode("GUIDED")
        self.snap.armed = True
        self.pump()


def test_a_pilot_who_takes_the_aircraft_during_the_gate_hover_keeps_it():
    """P1 (review 2026-10-22). The gate's hover read no mode, and the loop's
    takeover test (a CHANGE of mode) read its baseline AFTER the gate: a
    pilot who switched to LOITER 0.5 s into the hover became that baseline,
    and the node asked for LAND over the pilot with autopilot_took_over None.
    The hover now watches the mode on every tick (HoverWatch): the takeover
    ends it at once, no mission is flown, nothing is asked of the autopilot
    or sent to it, and the takeover is on record."""
    rc, run, link = _fly(grounder_cls=GateDetections,
                         link_kw={"takeover_at": (0.5, "LOITER")},
                         extra=("--start-when-seen", "5", "--start-timeout-s", "2"))
    assert rc == 0
    m = _json(run, "metrics.json")
    assert not link.requested and not link.sent, (link.requested, len(link.sent))
    tk = m["autopilot_took_over"]
    assert tk is not None and tk["mode"] == "LOITER" and tk["previous"] == "GUIDED", tk
    # Before the mission clock (mission time; 0.0 here, where the stand-in
    # clock does not move between the hover's last tick and t0).
    assert tk["phase"] == "start gate" and tk["t"] <= 0 and tk["t_hover"] == 0.5, tk
    g = m["start_gate"]
    assert g["outcome"] == "takeover" and g["reached"] is False and g["wait_s"] < 1.0, g
    assert m["end_reason"].startswith("no mission: the autopilot took the aircraft"), \
        m["end_reason"]
    assert m["mission_end_land_accepted"] is None and m["ticks"] == 0
    assert m["pre_mission_hover"]["outcome"] == "takeover"
    assert _json(run, "kpi.json")["kpi_grade"] is False
    assert any(e["kind"] == "autopilot_took_over" for e in _events(run))


def test_a_takeover_before_the_gate_opens_is_never_flown_over():
    """P1, the second case the review reproduced: the pilot took the aircraft
    0.1 s into the hover with the car in view, the gate opened on the 5th
    sighting anyway, and the mission sent 30 setpoints while the pilot held
    LOITER, then asked for LAND. The hover now ends at the takeover."""
    rc, run, link = _fly(grounder_cls=GateDetections, depth_m=CAR_DEPTH_M, grounder_kw=CAR_KW,
                         link_kw={"takeover_at": (0.1, "LOITER")},
                         extra=("--start-when-seen", "5"))
    assert rc == 0
    m = _json(run, "metrics.json")
    assert not link.sent and not link.requested, (len(link.sent), link.requested)
    assert m["ticks"] == 0 and m["start_gate"]["outcome"] == "takeover", m["start_gate"]
    assert m["start_gate"]["inferences_judged"] < 5
    assert m["autopilot_took_over"]["mode"] == "LOITER"


def test_a_takeover_as_the_gate_opens_is_caught_by_the_loop():
    """P1: the loop's mode baseline is the hover's last mode, not a fresh
    read after the gate. A pilot's switch reported in between (here: as the
    gate commits) became the baseline, and the loop flew its setpoints over
    the pilot."""
    TakeoverAtCommitLink.made.clear()
    rc, run, link = _fly(link_cls=TakeoverAtCommitLink, grounder_cls=PilotAtCommit,
                         depth_m=CAR_DEPTH_M, grounder_kw=CAR_KW,
                         extra=("--start-when-seen", "5"))
    assert rc == 0
    m = _json(run, "metrics.json")
    assert m["start_gate"]["reached"] is True and m["start_gate"]["outcome"] == "reached"
    assert not link.sent and "LAND" not in link.requested, (len(link.sent), link.requested)
    tk = m["autopilot_took_over"]
    assert tk["mode"] == "LOITER" and tk["previous"] == "GUIDED" and tk["phase"] == "mission", tk


def test_a_start_gate_that_crashes_still_lands_and_writes_the_episode():
    """P2 (review 2026-10-22). The detector wait, StartGate.judge (identity
    tiers, the Reacquirer, the presence verdict on live detector output),
    the truth verdict and the commit ran between the take-off and the loop's
    try/finally: an exception there left the aircraft holding in GUIDED,
    with no LAND, no mission_end and no metrics or KPI file (on HIL or
    attach, with nothing commanding it). It now ends the episode with no
    mission, landed and on record, and the run exits non-zero; Ctrl+C in
    the hover lands it the same way."""
    old = pn.StartGate.judge

    def boom(self, *a, **k):
        raise RuntimeError("judge broke")
    pn.StartGate.judge = boom
    try:
        rc, run, link = _fly(grounder_cls=GateDetections,
                             extra=("--start-when-seen", "5", "--start-timeout-s", "2"))
    finally:
        pn.StartGate.judge = old
    assert rc == 1, rc
    assert link.requested == ["LAND"] and link.snap.mode == "LAND", link.requested
    m = _json(run, "metrics.json")
    assert m["end_reason"].startswith("no mission: start gate crashed: RuntimeError: judge broke"), \
        m["end_reason"]
    assert m["start_gate"]["outcome"] == "crashed" and "judge broke" in m["start_gate"]["error"]
    assert m["mission_end_land_accepted"] is True and not link.sent
    kpi = _json(run, "kpi.json")
    assert kpi["kpi_grade"] is False
    assert any("start gate crashed" in w for w in kpi["kpi_grade_reasons"])
    assert "mission_end" in [e["kind"] for e in _events(run)]

    def interrupt(self, *a, **k):
        raise KeyboardInterrupt
    pn.StartGate.judge = interrupt
    try:
        rc, run, link = _fly(grounder_cls=GateDetections,
                             extra=("--start-when-seen", "5", "--start-timeout-s", "2"))
    finally:
        pn.StartGate.judge = old
    assert rc == 0 and link.requested == ["LAND"], (rc, link.requested)
    m = _json(run, "metrics.json")
    assert m["end_reason"] == "no mission: interrupted during the start gate", m["end_reason"]
    assert m["start_gate"]["outcome"] == "interrupted"


# --------------------------------------------------------------------------- #
# No way out of run() from the arming on without a LAND (review 3, 2026-10-22)
# --------------------------------------------------------------------------- #

def _mission_end(run):
    ends = [e for e in _events(run) if e["kind"] == "mission_end"]
    assert len(ends) == 1, ends
    return ends[0]


def test_a_hover_log_that_cannot_be_written_still_lands():
    """Review 3, P1 (a). The hover's finally wrote hover_log.jsonl with no
    guard, and outside the loop's try/finally: a write that failed (here a
    directory in its place; on the lab machine a full D: or a file OneDrive
    holds open under a reused tag) raised PermissionError out of run() with
    the aircraft holding in GUIDED - no LAND, no mission_end, no metrics.
    The hover's bookkeeping is best effort now: the run goes on to its
    no-mission end, lands, and says the log was not written."""
    def dir_in_its_place(run):
        (run / "hover_log.jsonl").mkdir(parents=True)
    rc, run, link = _fly(grounder_cls=GateDetections, pre=dir_in_its_place,
                         extra=("--start-when-seen", "5", "--start-timeout-s", "2"))
    assert rc == 0 and link.requested == ["LAND"] and not link.sent, (rc, link.requested)
    m = _json(run, "metrics.json")
    assert m["end_reason"].startswith("no mission: start gate timed out"), m["end_reason"]
    hv = m["pre_mission_hover"]
    assert hv["log"] is None and "hover_log.jsonl" in hv["log_error"], hv
    assert m["mission_end_land_accepted"] is True
    reasons = _json(run, "kpi.json")["kpi_grade_reasons"]
    assert any("hover_log.jsonl was not written" in w for w in reasons), reasons
    _mission_end(run)


def test_a_disk_that_fills_during_the_hover_still_lands():
    """Review 3, P1 (b). EpisodeRecord.event raising (errno 28, a full disk)
    from the gate's own record on: the hover's handler caught the first
    OSError, then raised a second one writing its own hover_crashed event,
    and that one left run() with the aircraft in GUIDED and nothing asked.
    The handler's write is best effort now, and what raises after the hover
    (mission_start's write, on the same full disk) meets the net that runs
    from the arming to the end of run(): LAND first, then the error."""
    import guardrail.replay as R
    orig = R.EpisodeRecord.event
    full = {"on": False}

    def event(self, kind, /, t=None, **data):
        if kind == "start_gate":
            full["on"] = True
        if full["on"]:
            raise OSError(28, "No space left on device")
        return orig(self, kind, t=t, **data)
    R.EpisodeRecord.event = event
    try:
        exc, run, link = _fly_raising(OSError, grounder_cls=GateDetections,
                                      extra=("--start-when-seen", "5", "--start-timeout-s", "2"))
    finally:
        R.EpisodeRecord.event = orig
    assert exc.errno == 28, exc
    assert link.requested == ["LAND"] and link.snap.mode == "LAND", link.requested
    assert not link.sent


def test_a_hover_that_starts_outside_guided_is_a_takeover():
    """Review 3, P2. HoverWatch took the mode the bring-up left as its
    takeover baseline and never checked that it was GUIDED: a switch
    reported in the bring-up's last pump (the end of the climb, where an
    altitude-fence breach from overshoot fires RTL) became the baseline, and
    neither the hover nor the loop ever saw a change - 30 setpoints flown
    over the pilot's LOITER, then a LAND asked over the pilot. The baseline
    is GUIDED, what both links' bring_up leaves (or raises): a hover that
    starts in a mode this node did not ask for is a takeover at the
    bring-up, with or without a start gate."""
    for gate in ("5", "0"):
        rc, run, link = _fly(grounder_cls=GateDetections, depth_m=CAR_DEPTH_M,
                             grounder_kw=CAR_KW, link_kw={"takeover_at": (0.0, "LOITER")},
                             extra=("--start-when-seen", gate))
        assert rc == 0
        assert not link.sent and link.requested == [], (gate, len(link.sent), link.requested)
        m = _json(run, "metrics.json")
        tk = m["autopilot_took_over"]
        assert tk is not None and tk["mode"] == "LOITER" and tk["previous"] == "GUIDED", (gate, tk)
        assert tk["phase"] == "bring-up" and tk["t_hover"] == 0.0 and tk["t"] <= 0, tk
        assert m["end_reason"].startswith("no mission: the autopilot took the aircraft"), \
            m["end_reason"]
        hv = m["pre_mission_hover"]
        assert hv["mode_at_start"] == "LOITER" and hv["outcome"] == "takeover", hv
        assert m["ticks"] == 0 and m["mission_end_land_accepted"] is None
    # A mode not reported yet is no baseline either: the first one the link
    # reports is judged against GUIDED.
    clk = Clock()
    anchor = ap.FrameAnchor(35.6895, 139.6917, 40.0, "scene.home-geo-point")
    link = FakeLink(clk, anchor)
    link.pump()
    assert link.snap.mode is None
    watch = pn.HoverWatch(link, anchor, FakeObs(), None, pn.ModeRequests(), t0=clk.t)
    assert watch.ended is None and watch.observe(clk.t, "detector") is not None
    clk.sleep(0.1)
    link._mode("LOITER")
    assert watch.observe(clk.t, "detector") is None and watch.ended == "takeover", watch.why
    assert watch.takeover["previous"] == "GUIDED" and watch.takeover["mode"] == "LOITER"


class ClimbFailsLink(FakeLink):
    """Armed in GUIDED and climbing, but never at the take-off altitude:
    PymavlinkLink.bring_up raises 'take-off did not reach N m' after its
    60 s wait, with the aircraft armed and in the air (a ceiling, a weak
    motor). `pilot` switches the aircraft first, as a pilot in Mission
    Planner (or the fence) would during a stalled climb."""

    def __init__(self, *a, pilot=None, **k):
        super().__init__(*a, **k)
        self.pilot = pilot

    def bring_up(self, alt):
        super().bring_up(alt)
        self.up = 3.1
        if self.pilot:
            self._mode(self.pilot)
        raise RuntimeError(f"take-off did not reach {alt} m (rel_alt 3.1)")


class ArmRefusedLink(FakeLink):
    """GUIDED accepted, arming refused: never in the air."""

    def bring_up(self, alt):
        self._mode("GUIDED")
        self.snap.armed = False
        self.pump()
        raise RuntimeError("arming refused: ['PreArm: Throttle below failsafe']")


def test_nothing_from_the_arming_to_the_loop_leaves_the_aircraft_unlanded():
    """Review 3, P4 (open before this wave). From the arming to the hover's
    try, an exception met only the outer finally, which stops the recorder,
    the detector, the camera and SITL and asks nothing of the autopilot: a
    take-off that stalled under a ceiling, a mission_start_check write on a
    full disk, a FollowPilot or SubjectTruth that raised, a crash between
    the hover and the loop - each left the aircraft in GUIDED mid-climb or
    holding (on HIL or attach, with no SITL to stop). One net now runs from
    the arming to the end of run(): unless the autopilot or a pilot has the
    aircraft, it asks for LAND, then writes the mission_end, then lets the
    error go."""
    import guardrail.replay as R
    exc, run, link = _fly_raising(RuntimeError, link_cls=ClimbFailsLink)
    assert "take-off did not reach" in str(exc)
    assert link.requested == ["LAND"] and not link.sent, link.requested
    end = _mission_end(run)
    assert "take-off did not reach" in end["why"] and end["land_accepted"] is True, end
    # A pilot who took the stalled climb keeps it.
    exc, run, link = _fly_raising(RuntimeError, link_cls=ClimbFailsLink,
                                  link_kw={"pilot": "LOITER"})
    assert link.requested == [], link.requested
    end = _mission_end(run)
    assert end["autopilot_owns"] is True and end["land_accepted"] is None, end
    # A refused arm never flew: nothing is asked of a disarmed aircraft, and
    # the bring-up is refused as before (no episode end).
    exc, run, link = _fly_raising(RuntimeError, link_cls=ArmRefusedLink)
    assert link.requested == [] and "mission_end" not in [e["kind"] for e in _events(run)]

    def raising(*a, **k):
        raise RuntimeError("constructor broke")
    orig_event = R.EpisodeRecord.event

    def start_check_on_full_disk(self, kind, /, t=None, **data):
        if kind == "mission_start_check":
            raise OSError(28, "No space left on device")
        return orig_event(self, kind, t=t, **data)
    for name, owner, attr, value, err in (
            ("mission_start_check", R.EpisodeRecord, "event", start_check_on_full_disk, OSError),
            ("FollowPilot", pn, "FollowPilot", raising, RuntimeError),
            ("SubjectTruth", pn, "SubjectTruth", raising, RuntimeError),
            ("after the hover", pn.HoverWatch, "summary", raising, RuntimeError)):
        orig = getattr(owner, attr)
        setattr(owner, attr, value)
        try:
            exc, run, link = _fly_raising(err, grounder_cls=GateDetections,
                                          extra=("--start-when-seen", "5",
                                                 "--start-timeout-s", "2"))
        finally:
            setattr(owner, attr, orig)
        assert link.requested == ["LAND"] and not link.sent, (name, link.requested)
        end = _mission_end(run)
        assert type(exc).__name__ in end["why"] and end["land_accepted"] is True, (name, end)
    # A crash after a takeover in the hover asks nothing of the pilot.
    orig = pn.HoverWatch.summary
    pn.HoverWatch.summary = raising
    try:
        exc, run, link = _fly_raising(RuntimeError, grounder_cls=GateDetections,
                                      link_kw={"takeover_at": (0.5, "LOITER")},
                                      extra=("--start-when-seen", "5", "--start-timeout-s", "2"))
    finally:
        pn.HoverWatch.summary = orig
    assert link.requested == [] and not link.sent, link.requested
    assert _mission_end(run)["autopilot_owns"] is True


def test_the_hover_before_the_mission_is_checked_against_the_rules():
    """P3 (review 2026-10-22). For up to 630 s airborne (the gate's 330 s
    and the detector's 300 s) no Shield query, FSM step or fence recheck
    ran and no row was written: a zone that opened over the hover point,
    a moving zone or the subject's stand-off ring was neither enforced nor
    measured, and KPIs start at the mission clock. DWELL_POLICY puts the
    hover point inside a zone and under the floor. The hover's first tick
    is unsafe; the hover ends there and the loop starts at once, so the
    Shield and the escalation FSM act; the gate says why; the tick is in
    hover_log.jsonl and the run is not KPI-grade."""
    rc, run, link = _fly(grounder_cls=GateDetections, policy=DWELL_POLICY,
                         extra=("--start-when-seen", "5", "--start-timeout-s", "2"))
    assert rc == 0
    m = _json(run, "metrics.json")
    hv = m["pre_mission_hover"]
    assert hv["outcome"] == "unsafe" and hv["ticks_checked"] == 1 and hv["unsafe_ticks"] == 1, hv
    assert hv["unsafe_rules"] == {"floor": 1, "nfz-here": 1}, hv
    assert hv["first_unsafe"]["phase"] == "start gate", hv
    g = m["start_gate"]
    assert g["outcome"] == "unsafe" and g["reached"] is False and g["unsafe_ticks"] == 1, g
    assert m["ticks"] > 0, "the loop did not take the aircraft"
    rows = _hover_rows(run)
    assert len(rows) == 1 and rows[0]["unsafe"] == ["floor", "nfz-here"], rows
    reasons = _json(run, "kpi.json")["kpi_grade_reasons"]
    assert any(w.startswith("the hover before the mission broke") for w in reasons), reasons
    assert any("the Shield and the escalation FSM" in w for w in reasons), reasons
    # The Shield-off arm records the hover, as it records the loop, and does
    # not act on it: the gate times out with every tick unsafe.
    rc, run, link = _fly("off", grounder_cls=GateDetections, policy=DWELL_POLICY,
                         extra=("--start-when-seen", "5", "--start-timeout-s", "2"))
    m = _json(run, "metrics.json")
    hv = m["pre_mission_hover"]
    assert hv["act_on_unsafe"] is False and m["start_gate"]["outcome"] == "timed_out", hv
    assert hv["unsafe_ticks"] == hv["ticks_checked"] >= 15, hv
    assert m["end_reason"].startswith("no mission: start gate timed out")


def test_the_gate_s_candidate_binds_the_subject_standoff_while_it_hovers():
    """P3: the Shield is given the gate's current candidate as the subject,
    as the loop gives it the pilot's estimate (set_subject): a car placed
    20 m out (depth) is inside STANDOFF_POLICY's 120 m ring, so the first
    sighting makes the hover unsafe - before the gate's 5 sightings."""
    rc, run, link = _fly(grounder_cls=GateDetections, policy=STANDOFF_POLICY,
                         depth_m=CAR_DEPTH_M, grounder_kw=CAR_KW,
                         extra=("--start-when-seen", "5"))
    assert rc == 0
    m = _json(run, "metrics.json")
    hv = m["pre_mission_hover"]
    assert hv["unsafe_rules"] == {"standoff-car": 1} and hv["subject_ticks"] == 1, hv
    assert m["start_gate"]["outcome"] == "unsafe" and m["start_gate"]["inferences_judged"] == 1
    row = _hover_rows(run)[0]
    assert row["subject"] is not None and row["subject"][2] == "depth", row


def test_a_clean_hover_is_counted_and_no_hover_is_none_not_zero():
    """P3: a clean hover is measured (every tick checked, 0 unsafe); a run
    with no hover at all says None, not 0 - no check ran."""
    rc, run, link = _fly(grounder_cls=GateDetections, depth_m=CAR_DEPTH_M, grounder_kw=CAR_KW,
                         extra=("--start-when-seen", "5"))
    m = _json(run, "metrics.json")
    hv = m["pre_mission_hover"]
    assert hv["outcome"] is None and hv["ticks_checked"] == 5 and hv["unsafe_ticks"] == 0, hv
    assert hv["s"] > 0 and m["start_gate"]["hover_s"] == hv["s_by_phase"]["start gate"], hv
    rows = _hover_rows(run)
    assert len(rows) == 5 and {r["phase"] for r in rows} == {"start gate"}
    assert all(r["unsafe"] == [] for r in rows) and rows[-1]["det_seq"] is not None
    rc, run, link = _fly()
    hv = _json(run, "metrics.json")["pre_mission_hover"]
    assert hv["ticks"] == 0 and hv["unsafe_ticks"] is None and hv["s"] == 0.0, hv
    assert _hover_rows(run) == []


def test_the_hover_before_the_mission_is_capped():
    """The hover is capped at --start-timeout-s whichever wait it is spent
    in: on a link with no pre-arm step the detector's wait is airborne, and
    comes out of the gate's budget. A timeout that is not finite, or above
    START_TIMEOUT_MAX_S, is refused before anything starts."""
    base = ["--network", "mirrored", "--dev-yaml", "--citymap", str(_tmp() / "occ.npz")]
    for bad in ("inf", "nan", str(pn.START_TIMEOUT_MAX_S + 1), "0"):
        try:
            pn.check_args(pn.build_parser().parse_args(base + ["--start-timeout-s", bad]))
        except SystemExit as exc:
            assert "--start-timeout-s" in str(exc), exc
        else:
            raise AssertionError(f"--start-timeout-s {bad} accepted")
    pn.check_args(pn.build_parser().parse_args(base + ["--start-timeout-s",
                                                       str(pn.START_TIMEOUT_MAX_S)]))
    rc, run, link = _fly(grounder_cls=LoadingDetections, grounder_kw={"load_s": 1.0},
                         extra=("--start-when-seen", "5", "--start-timeout-s", "2"))
    m = _json(run, "metrics.json")
    dr, hv, g = m["detector_ready"], m["pre_mission_hover"], m["start_gate"]
    assert dr["ready"] and dr["airborne"] and dr["waited_s"] >= 1.0, dr
    assert g["timeout_s"] <= 2.0 - dr["waited_s"] + 1e-6 and g["outcome"] == "timed_out", g
    assert hv["s"] <= 2.0 + 2 * pn.TICK, hv
    assert set(hv["s_by_phase"]) == {"detector", "start gate"}, hv


def test_the_detector_is_waited_for_on_the_ground_when_the_link_can():
    """The detector's first inference is waited for in the pre-arm step,
    disarmed, on a link that has one (PymavlinkLink, MavrosLink): a slow
    model load is no airborne hover at all."""
    rc, run, link = _fly(link_cls=PreArmLink, grounder_cls=LoadingDetections,
                         grounder_kw={"load_s": 1.0})
    assert rc == 0
    m = _json(run, "metrics.json")
    dr = m["detector_ready"]
    assert dr["ready"] and dr["phase"] == "before arming" and dr["airborne"] is False, dr
    assert dr["waited_s"] >= 1.0 and link.armed_in_before_arm is False
    assert m["pre_mission_hover"]["ticks"] == 0, m["pre_mission_hover"]


def test_no_mission_is_flown_on_a_detector_that_died():
    """P5 (review 2026-10-22). wait_for_detector waited its full 300 s on a
    Grounder whose thread had died (seq stays 0), the gate 330 s more, and
    with --start-when-seen 0 or --start-timeout-action fly the mission flew
    on a detector that never produced an inference, with detector_ready not
    among the KPI-grade reasons. The wait now stops on the dead thread; no
    mission is flown on it (on the ground the aircraft is not armed), and
    the reason is on record."""
    clk = Clock()
    link = FakeLink(clk, ap.FrameAnchor(35.6895, 139.6917, 40.0, "scene.home-geo-point"))
    dr = pn.wait_for_detector(DeadDetections(clk), link, clock=clk, sleep=clk.sleep)
    assert dr["outcome"] == "detector_dead" and dr["ready"] is False and dr["waited_s"] == 0, dr
    for gate in ("5", "0"):
        rc, run, link = _fly(grounder_cls=DeadDetections, extra=("--start-when-seen", gate))
        assert rc == 0
        m = _json(run, "metrics.json")
        assert m["detector_ready"]["outcome"] == "detector_dead", m["detector_ready"]
        assert m["end_reason"].startswith("no mission: the detector thread died"), m["end_reason"]
        assert link.requested == ["LAND"] and not link.sent
        reasons = _json(run, "kpi.json")["kpi_grade_reasons"]
        assert any(w.startswith("the detector produced no inference") for w in reasons), reasons
    # On a link with a pre-arm step the aircraft never arms on a dead detector.
    try:
        _fly(link_cls=PreArmLink, grounder_cls=DeadDetections)
    except RuntimeError as exc:
        assert "detector thread died" in str(exc), exc
    else:
        raise AssertionError("armed on a dead detector")


def test_a_run_that_flew_no_mission_measures_no_rate_and_no_dwell():
    """P4 (review 2026-10-22). With no tick flown mission_s was max(1e-6,
    ...), so det_hz read 1000000.0 Hz, and nfz_s, alt_violation_s,
    interventions, escape_ticks and dwell_gap_s read 0: a 1 MHz detector
    and clean dwell for a run that flew nothing, for anything reading
    metrics.json directly (the tracker, the decks). Now None, with why."""
    rc, run, link = _fly(grounder_cls=GateDetections,
                         extra=("--start-when-seen", "5", "--start-timeout-s", "2"))
    m = _json(run, "metrics.json")
    assert m["ticks"] == 0 and m["end_reason"].startswith("no mission"), m["end_reason"]
    for k in ("det_hz", "loop_hz", "loop_hz_below_gate", "nfz_s", "alt_violation_s",
              "nfz_ticks", "alt_violation_ticks", "interventions", "brakes",
              "escape_ticks", "dwell_gap_s", "dwell_gap_points"):
        assert m[k] is None, (k, m[k])
    assert m["not_measured"].startswith("no mission flown: start gate timed out"), \
        m["not_measured"]
    rc, run, link = _fly()
    m = _json(run, "metrics.json")
    assert m["not_measured"] is None and m["det_hz"] > 0 and m["nfz_s"] == 0.0


def test_one_tick_waits_for_the_simulator_at_most_once():
    """P8 (review 2026-10-22). SIM_RPC_WAIT_S bounded each call, not each
    tick: a truth read and a frame check each answered just inside 50 ms
    were 100 ms, a whole tick. A tick's calls now share one budget
    (OffLoopRpc.begin_tick): a call waits only what is left, and one with
    nothing left is not sent ("spent")."""
    import threading
    import time as _time
    now = [0.0]
    rpc = pn.OffLoopRpc(wait_s=0.05, timer=lambda: now[0])
    rpc.begin_tick()
    assert rpc.call(lambda: 1) == ("ok", 1)
    now[0] = 0.06                         # the tick's first RPC used the budget
    gate = threading.Event()
    ran = []
    assert rpc.call(lambda: ran.append(1)) == ("spent", None)
    rpc.end_tick()
    _time.sleep(0.05)
    assert not ran, "a spent call was sent"
    assert rpc.call(lambda: 2) == ("ok", 2), "outside a tick each call has its own wait"
    rpc.begin_tick()                      # 10 ms of it left below
    now[0] += 0.04
    t0 = _time.perf_counter()
    st, _ = rpc.call(lambda: gate.wait(2.0))
    took = _time.perf_counter() - t0
    assert st == "late" and took < 0.045, (st, took)
    rpc.end_tick()
    gate.set()
    s = rpc.summary()
    assert s["spent"] == 1 and s["late"] == 1 and s["ticks"] == 2, s
    rpc.close()

    made = []

    class SpyRpc(pn.OffLoopRpc):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self.opened = self.closed = 0
            made.append(self)

        def begin_tick(self):
            self.opened += 1
            super().begin_tick()

        def end_tick(self):
            self.closed += 1
            super().end_tick()

    old = pn.OffLoopRpc
    pn.OffLoopRpc = SpyRpc
    try:
        rc, run, link = _fly()
    finally:
        pn.OffLoopRpc = old
    n = _json(run, "metrics.json")["ticks"]
    assert made and made[0].opened == made[0].closed == n > 0, (made[0].opened, n)


def test_the_negative_phrase_scores_come_from_the_subject_column():
    """P9 (review 2026-10-22). Grounder._worker's use of the negative phrases
    (the logits sliced into the subject's column and the negatives' max,
    and the fallback on post_process's labels when it dropped a box) had no
    test, and a wrong column would fail silently, on the GPU only. It is now
    follow_vlm.subject_scores_with_negatives, checked here on CPU arrays."""
    fv = pn._fv()
    import numpy as np

    def sig(v):
        return 1.0 / (1.0 + math.exp(-v))
    # Three boxes x (subject, "a traffic light"): box 0 is the car's, box 1
    # the light's (its car score 0.5 is over the threshold: suppressed and
    # counted), box 2 low for both.
    lg = np.array([[1.0, -2.0], [0.0, 2.0], [-6.0, -5.0]], dtype=np.float32)
    post = [max(sig(a), sig(b)) for a, b in lg.tolist()]
    labels = [0, 1, 1]
    kept, rec, n = fv.subject_scores_with_negatives(lg, post, labels, 0.02)
    assert [round(v, 6) for v in kept] == [round(sig(1.0), 6), 0.0, 0.0], kept
    assert n == 1 and rec["suppressed"] == 1 and rec["top_index"] == 1, rec
    assert abs(rec["top_pos"] - 0.5) < 1e-6 and abs(rec["top_neg"] - sig(2.0)) < 1e-6
    # A negative that never wins leaves every box's subject score as it was.
    lg2 = np.array([[2.0, -3.0], [0.5, -1.0]], dtype=np.float32)
    kept2, rec2, n2 = fv.subject_scores_with_negatives(
        lg2, [sig(2.0), sig(0.5)], [0, 0], 0.02)
    assert [round(v, 6) for v in kept2] == [round(sig(2.0), 6), round(sig(0.5), 6)]
    assert n2 == 0 and rec2["suppressed"] == 0 and rec2["top_index"] is None
    # post_process dropped a box (2 kept for 3 logits rows): the logits
    # cannot be paired, so its labels decide, and the count is unknown.
    kept3, rec3, n3 = fv.subject_scores_with_negatives(lg, [0.7, 0.9], [0, 1], 0.02)
    assert kept3 == [0.7, 0.0] and n3 == 0 and rec3["suppressed"] is None, (kept3, rec3)
    assert rec3["count_unknown"] == "2 boxes for 3 logits", rec3
    # Very negative logits do not overflow; a CPU tensor reads the same.
    k4, _, _ = fv.subject_scores_with_negatives(np.array([[-1e4, -1e4]]), [0.0], [0], 0.02)
    assert k4 == [0.0]
    try:
        import torch
    except ImportError:
        return
    kt, rt, nt = fv.subject_scores_with_negatives(torch.tensor(lg), torch.tensor(post),
                                                  torch.tensor(labels), 0.02)
    assert [round(v, 6) for v in kt] == [round(v, 6) for v in kept] and nt == 1


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
