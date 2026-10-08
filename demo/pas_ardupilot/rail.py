"""The wiring of the Project AirSim -> ArduPilot SITL -> Mission Planner rail.

Everything here is a pure function of the scene file, the robot template and
two IP addresses, so the whole plan can be checked offline
(tests/test_pas_ardupilot.py) and printed before anything starts.

THE DATA PATH (verified against the shipped binaries, 2026-10-06)

    Project AirSim (Windows)                       ArduPilot SITL (WSL2)
    ArduCopterApi, "ardupilot-api" controller       --model airsim-copter
      every physics step: one JSON line  --UDP-->   binds 0.0.0.0:9003
        {timestamp, imu, pose, gps, velocity}        (SIM_AirSim.cpp keytable)
      binds local-host-ip:9002          <--UDP--    11 x uint16 PWM to
                                                     --sim-address:9002

The JSON keys were read out of PASBlocks' own multirotor_api.lib and match the
key table of ArduPilot's SIM_AirSim.cpp at the pinned Copter-4.5.7. Ports 9003
and 9002 are ArduPilot's built-in defaults (SITL_cmdline.cpp SIM_IN_PORT /
SIM_OUT_PORT), so neither side needs a non-default port.

Mission Planner never talks to Project AirSim. It talks MAVLink to ArduPilot,
through mavlink-router's fan-out (the grant's "Mission Planner via
mavlink-router fan-out, parallel to MAVROS"), and sees whatever ArduPilot
flies - here, the Project AirSim vehicle.

WSL2 NETWORKING, WHICH DECIDES EVERY ADDRESS

  NAT (this PC's setting: ~/.wslconfig leaves networkingMode commented out).
    WSL has its own address on a virtual switch. Windows forwards TCP from
    localhost into WSL ("localhost forwarding"), but NOT UDP. So:
      * Project AirSim -> ArduPilot (UDP 9003) must target WSL's eth0 address;
      * ArduPilot -> Project AirSim (UDP 9002) must target the Windows end of
        the switch, which WSL sees as its default gateway, and Project AirSim
        must bind THAT address (127.0.0.1 would never see a datagram from WSL);
      * mavlink-router -> Mission Planner (UDP 14550) also targets that address;
      * a Windows process reaching WSL over TCP (the router's TCP server 5790)
        may use 127.0.0.1, because TCP is forwarded.
    The WSL address changes on every WSL restart, so it is rendered per run.

  Mirrored (networkingMode=mirrored). WSL shares the Windows interfaces and
    127.0.0.1 reaches across in both directions for TCP and UDP. Every address
    above becomes 127.0.0.1.

CLI (stdlib only, so WSL's system python3 runs it without the project's envs):

    python3 rail.py home        --scene S                  -> lat,lon,alt,yaw
    python3 rail.py sitl-args   --scene S --sim-address IP -> one argument per line
    python3 rail.py router-conf --gcs IP --node IP [--orin IP]
    python3 rail.py render      --out DIR --network nat --windows-ip A --wsl-ip B
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import ipaddress
import json
import sys
from dataclasses import dataclass
from pathlib import Path, PureWindowsPath

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

ROBOT_TEMPLATE = HERE / "robot_citylife_ardupilot.jsonc"
SCENE_TEMPLATE = HERE / "scene_citylife_ardupilot.jsonc"
PARAM_FILE = HERE / "citylife-quad.param"
ROBOT_NAME = "Drone1"

CONTROLLER_TYPE = "ardupilot-api"

# Ports. The first two are ArduPilot's own defaults for the AirSim backend.
SITL_SENSOR_PORT = 9003     # ArduPilot binds 0.0.0.0:9003; the simulator sends JSON here
SIM_SERVO_PORT = 9002       # the simulator binds local-host-ip:9002; ArduPilot sends PWM here
SITL_SERIAL0_TCP = 5760     # ArduPilot SITL SERIAL0, mavlink-router's input
# mavlink-router's own TCP server. NOT its default 5760: SITL already holds
# 5760 in the same network namespace, and the router would fail to bind.
# mavlink-router serves several TCP clients on this one port; the MAVProxy
# fallback serves ONE client per tcpin port, so there Mission Planner gets its
# own port (GCS_TCP_FALLBACK_PORT) and the node keeps this one.
ROUTER_TCP_PORT = 5790
GCS_TCP_FALLBACK_PORT = 5791
# Mission Planner's default UDP listen port. In the grant's interface table
# UDP 14550 sits on the MAVROS 2 -> ArduPilot row; this rail gives MAVROS
# 14555 and Mission Planner 14550, the layout of the reference docker-compose
# and of sitl/mavlink-router/main.conf, so every rail in this repository uses
# the same two numbers (docs/DESIGN-projectairsim-ardupilot.md records it).
GCS_UDP_PORT = 14550
NODE_UDP_PORT = 14551       # the perception node, when it listens on UDP instead of TCP
MAVROS_UDP_PORT = 14555     # MAVROS 2: fcu_url udp://:14555@

# ArduCopter's quad-X motor numbering: (actuator, spin direction) for outputs
# 1..4. The directions are those of IAMAI's ArduPilot example, which flies.
ARDUCOPTER_QUAD_X = (
    ("Prop_FR_actuator", "counter-clock-wise"),
    ("Prop_RL_actuator", "counter-clock-wise"),
    ("Prop_FL_actuator", "clock-wise"),
    ("Prop_RR_actuator", "clock-wise"),
)

# ArduPilot's own default parameter files for `-f airsim-copter`
# (Tools/autotest/pysim/vehicleinfo.py at Copter-4.5.7).
AP_DEFAULT_PARAMS = ("copter.parm", "airsim-quadX.parm")
AP_BINARY = "~/ardupilot/build/sitl/bin/arducopter"
AP_PARAM_DIR = "~/ardupilot/Tools/autotest/default_params"


# --------------------------------------------------------------------------- #
# JSONC
# --------------------------------------------------------------------------- #

def strip_jsonc(text: str) -> str:
    """Remove // and /* */ comments that sit OUTSIDE string literals.

    A line-anchored regex (what manifest.sim_speedup_from_scene uses) is enough
    for full-line comments only; the robot template also has comments inside
    objects, and a mesh path such as "/Drone/Quadrotor1" must survive intact.
    """
    out = []
    i, n = 0, len(text)
    in_str = False
    while i < n:
        c = text[i]
        if in_str:
            out.append(c)
            if c == "\\" and i + 1 < n:
                out.append(text[i + 1])
                i += 2
                continue
            if c == '"':
                in_str = False
            i += 1
            continue
        if c == '"':
            in_str = True
            out.append(c)
            i += 1
        elif text.startswith("//", i):
            j = text.find("\n", i)
            i = n if j < 0 else j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            if j < 0:
                raise ValueError("unterminated /* comment")
            i = j + 2
        else:
            out.append(c)
            i += 1
    return "".join(out)


def load_jsonc(path: str | Path) -> dict:
    return json.loads(strip_jsonc(Path(path).read_text(encoding="utf-8")))


def config_digest(obj) -> str:
    """sha256 of the canonical JSON of a config dict (key order independent)."""
    blob = json.dumps(obj, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(blob).hexdigest()


# --------------------------------------------------------------------------- #
# Scene facts
# --------------------------------------------------------------------------- #

def home_geo_point(scene: dict) -> tuple[float, float, float]:
    """(lat, lon, alt) of the scene origin. Refuses a scene without one: a
    default would put ArduPilot's home somewhere the simulator is not."""
    h = scene.get("home-geo-point")
    if not h or any(k not in h for k in ("latitude", "longitude", "altitude")):
        raise ValueError("scene has no complete home-geo-point; ArduPilot's "
                         "--home cannot be derived from it")
    return float(h["latitude"]), float(h["longitude"]), float(h["altitude"])


def spawn_yaw_deg(scene: dict, robot: str = ROBOT_NAME) -> float:
    for a in scene.get("actors", []):
        if a.get("type") == "robot" and a.get("name") == robot:
            rpy = str(a.get("origin", {}).get("rpy-deg", "0 0 0")).split()
            return float(rpy[2]) if len(rpy) == 3 else 0.0
    raise ValueError(f"scene has no robot actor named {robot!r}")


def sitl_home(scene: dict) -> str:
    """ArduPilot's --home "lat,lon,alt,yaw", equal to the scene's home point.

    ArduPilot's AirSim backend computes its internal position as the distance
    from its origin (= --home) to each GPS fix. With the default home (CMAC,
    Australia) that distance is thousands of kilometres; with this one it is
    the vehicle's few-metre offset from the scene origin.
    """
    lat, lon, alt = home_geo_point(scene)
    return f"{lat},{lon},{alt},{spawn_yaw_deg(scene)}"


def scene_clock_ratio(scene: dict) -> float | None:
    """real-time-update-rate / step-ns: 1.0 means the clock is meant to run in
    real time (the same derivation as manifest.sim_speedup_from_scene)."""
    clock = scene.get("clock") or {}
    step, rate = clock.get("step-ns"), clock.get("real-time-update-rate")
    if not step or not rate:
        return None
    return round(float(rate) / float(step), 4)


# --------------------------------------------------------------------------- #
# Network plan
# --------------------------------------------------------------------------- #

def _ipv4(label: str, value: str | None) -> str:
    if not value:
        raise ValueError(f"{label} is required under WSL2 NAT networking")
    try:
        ip = ipaddress.IPv4Address(str(value).strip())
    except ValueError as exc:
        raise ValueError(f"{label} {value!r} is not an IPv4 address") from exc
    if ip.is_loopback:
        raise ValueError(
            f"{label} {value} is loopback. Under WSL2 NAT, localhost forwarding "
            f"carries TCP only: a UDP datagram sent to 127.0.0.1 on one side "
            f"never reaches the other, so the sensor frames (9003) and the PWM "
            f"(9002) would both vanish without an error. Use the real address, "
            f"or switch WSL to networkingMode=mirrored.")
    if ip.is_unspecified:
        raise ValueError(f"{label} 0.0.0.0 is a bind wildcard, not a destination")
    return str(ip)


@dataclass(frozen=True)
class NetworkPlan:
    """Which address each side must use. See the module docstring."""
    mode: str            # "nat" | "mirrored"
    windows_ip: str      # the Windows host, as seen from WSL
    wsl_ip: str          # WSL, as seen from Windows

    @property
    def sim_bind_ip(self) -> str:
        """Project AirSim's local-host-ip: where it receives ArduPilot's PWM."""
        return self.windows_ip

    @property
    def ardupilot_ip(self) -> str:
        """Project AirSim's ardupilot-ip: where it sends sensor frames."""
        return self.wsl_ip

    @property
    def sitl_sim_address(self) -> str:
        """ArduPilot's --sim-address: where it sends PWM."""
        return self.windows_ip

    @property
    def gcs_address(self) -> str:
        """Where the router sends Mission Planner's UDP 14550 stream."""
        return self.windows_ip

    @property
    def router_tcp_host(self) -> str:
        """Where a Windows process connects to the router's TCP server.

        Under NAT this must be WSL's own address, not 127.0.0.1: mavlink-routerd
        listens on [::] (IPv6, dual stack), and WSL's localhost forwarding
        does not carry an IPv6 listener to Windows' 127.0.0.1 (measured
        2026-10-08: [::] refused on 127.0.0.1, accepted on the WSL address; an
        IPv4 0.0.0.0 listener such as the MAVProxy fallback works on both).
        Under mirrored networking WSL shares the host's addresses."""
        return self.wsl_ip if self.mode == "nat" else "127.0.0.1"

    @property
    def router_tcp_url(self) -> str:
        return f"tcp:{self.router_tcp_host}:{ROUTER_TCP_PORT}"

    def to_dict(self) -> dict:
        return {"mode": self.mode, "windows_ip": self.windows_ip,
                "wsl_ip": self.wsl_ip, "sim_bind_ip": self.sim_bind_ip,
                "ardupilot_ip": self.ardupilot_ip,
                "sitl_sim_address": self.sitl_sim_address,
                "gcs_address": self.gcs_address,
                "ports": {"sitl_sensor_in": SITL_SENSOR_PORT,
                          "sim_servo_in": SIM_SERVO_PORT,
                          "sitl_serial0_tcp": SITL_SERIAL0_TCP,
                          "router_tcp": ROUTER_TCP_PORT,
                          "gcs_tcp_fallback": GCS_TCP_FALLBACK_PORT,
                          "gcs_udp": GCS_UDP_PORT, "node_udp": NODE_UDP_PORT,
                          "mavros_udp": MAVROS_UDP_PORT}}


def plan_network(mode: str, windows_ip: str | None = None,
                 wsl_ip: str | None = None) -> NetworkPlan:
    mode = (mode or "").lower()
    if mode == "mirrored":
        return NetworkPlan("mirrored", "127.0.0.1", "127.0.0.1")
    if mode != "nat":
        raise ValueError(f"network mode {mode!r}: expected 'nat' or 'mirrored'")
    w = _ipv4("windows_ip (the WSL default gateway)", windows_ip)
    s = _ipv4("wsl_ip (WSL eth0)", wsl_ip)
    if w == s:
        raise ValueError(f"windows_ip and wsl_ip are both {w}; under NAT they are "
                         f"two ends of a virtual switch and cannot be equal")
    return NetworkPlan("nat", w, s)


def wslconfig_mode(text: str | None) -> str:
    """'mirrored' if an UNCOMMENTED networkingMode=mirrored is in the [wsl2]
    section of a .wslconfig, else 'nat' (WSL2's default)."""
    section = None
    for raw in (text or "").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith("[") and line.endswith("]"):
            section = line[1:-1].strip().lower()
            continue
        if section == "wsl2" and "=" in line:
            k, v = (p.strip().lower() for p in line.split("=", 1))
            if k == "networkingmode":
                return "mirrored" if v == "mirrored" else "nat"
    return "nat"


# --------------------------------------------------------------------------- #
# Robot and scene configs
# --------------------------------------------------------------------------- #

def check_robot_config(cfg: dict) -> list[str]:
    """Everything that would make ArduPilot fail to fly this robot. Empty = ok."""
    problems: list[str] = []
    ctl = cfg.get("controller") or {}
    if ctl.get("type") != CONTROLLER_TYPE:
        problems.append(f"controller type is {ctl.get('type')!r}, not "
                        f"{CONTROLLER_TYPE!r}: the simulator's own controller "
                        f"would fly, and ArduPilot would fly nothing")
    st = ctl.get("ardupilot-settings")
    if not isinstance(st, dict):
        problems.append("controller has no ardupilot-settings")
        st = {}
    for key, want in (("ardupilot-udp-port", SITL_SENSOR_PORT),
                      ("local-host-udp-port", SIM_SERVO_PORT)):
        if st.get(key) != want:
            problems.append(f"{key} is {st.get(key)!r}; ArduPilot SITL is "
                            f"started with its default {want}")
    for key in ("ardupilot-ip", "local-host-ip"):
        try:
            ipaddress.IPv4Address(str(st.get(key)))
        except ValueError:
            problems.append(f"{key} {st.get(key)!r} is not an IPv4 address")
    order = [a.get("id") for a in st.get("actuator-order", [])]
    want_order = [a for a, _ in ARDUCOPTER_QUAD_X]
    if order != want_order:
        problems.append(f"actuator-order {order} is not ArduCopter quad-X "
                        f"{want_order} (motor 1..4); a swapped pair flips the "
                        f"yaw torque")
    spins = {a.get("name"): (a.get("rotor-settings") or {}).get("turning-direction")
             for a in cfg.get("actuators", [])}
    for name, spin in ARDUCOPTER_QUAD_X:
        if spins.get(name) != spin:
            problems.append(f"{name} turns {spins.get(name)!r}; quad-X needs {spin!r}")
    enabled = {s.get("type") for s in cfg.get("sensors", []) if s.get("enabled", True)}
    if "imu" not in enabled:
        problems.append("no enabled IMU: ArduCopterApi refuses to start without one")
    if "gps" not in enabled:
        problems.append("no enabled GPS: the sensor frame would carry no position "
                        "and ArduPilot's EKF could never leave its origin")
    return problems


def render_robot_config(template: dict, plan: NetworkPlan) -> dict:
    """The template with this run's two addresses; refuses a config ArduPilot
    could not fly rather than handing it to the simulator."""
    cfg = copy.deepcopy(template)
    st = cfg.setdefault("controller", {}).setdefault("ardupilot-settings", {})
    st["ardupilot-ip"] = plan.ardupilot_ip
    st["local-host-ip"] = plan.sim_bind_ip
    st["ardupilot-udp-port"] = SITL_SENSOR_PORT
    st["local-host-udp-port"] = SIM_SERVO_PORT
    problems = check_robot_config(cfg)
    if problems:
        raise ValueError("robot config cannot be flown by ArduPilot: "
                         + "; ".join(problems))
    return cfg


def render_scene(template: dict, robot_config_name: str) -> dict:
    scene = copy.deepcopy(template)
    robots = [a for a in scene.get("actors", []) if a.get("type") == "robot"]
    if len(robots) != 1:
        raise ValueError(f"expected exactly one robot actor, found {len(robots)}")
    robots[0]["robot-config"] = robot_config_name
    home_geo_point(scene)               # refuses early if absent
    return scene


def write_sim_configs(out_dir: str | Path, plan: NetworkPlan,
                      scene_path: str | Path = SCENE_TEMPLATE,
                      robot_path: str | Path = ROBOT_TEMPLATE) -> dict:
    """Render this run's scene + robot config into `out_dir` (plain JSON, so
    any JSONC reader accepts them) and return where they went and their hashes.

    The directory is what `World(client, scene_name, sim_config_path=...)`
    reads, so the configs the simulator receives are the files kept with the
    run - evidence, not a reconstruction.
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    scene_t = load_jsonc(scene_path)
    robot_t = load_jsonc(robot_path)
    robot_name = Path(robot_path).name
    robot = render_robot_config(robot_t, plan)
    scene = render_scene(scene_t, robot_name)
    scene_file = out / Path(scene_path).name
    robot_file = out / robot_name
    for path, obj in ((scene_file, scene), (robot_file, robot)):
        with open(path, "w", encoding="utf-8", newline="\n") as fh:
            json.dump(obj, fh, indent=2)
            fh.write("\n")
    net_file = out / "network.json"
    with open(net_file, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(plan.to_dict(), fh, indent=2)
        fh.write("\n")
    return {"dir": str(out), "scene": scene_file.name, "robot": robot_name,
            "scene_id": scene.get("id"),
            "scene_sha256": config_digest(scene),
            "robot_sha256": config_digest(robot),
            "controller_type": robot["controller"]["type"],
            "home": list(home_geo_point(scene)),
            "clock_ratio": scene_clock_ratio(scene),
            "network": plan.to_dict()}


def write_attach_configs(out_dir: str | Path,
                         scene_path: str | Path = SCENE_TEMPLATE,
                         robot_path: str | Path = ROBOT_TEMPLATE) -> dict:
    """The configs a node that ATTACHES to an already-loaded scene keeps.

    The hil node on the Jetson Orin must not load the scene: a reload resets
    the simulator clock under a running ArduPilot (see the scene file). The
    desktop loaded the rendered copy (`-Step scene`) and knows the addresses;
    the Orin does not, so its record keeps the template with the network
    marked "attach" instead of inventing addresses. The scene id, home point,
    clock and sensors - everything the node reads - are the template's.
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    robot_name = Path(robot_path).name
    robot = load_jsonc(robot_path)
    problems = check_robot_config(robot)
    if problems:
        raise ValueError("robot template cannot be flown by ArduPilot: "
                         + "; ".join(problems))
    scene = render_scene(load_jsonc(scene_path), robot_name)
    net = {"mode": "attach",
           "note": "the scene was loaded by another host (run_pas_ardupilot.ps1 "
                   "-Step scene), which rendered the addresses; this node only "
                   "attached to it"}
    for path, obj in ((out / Path(scene_path).name, scene), (out / robot_name, robot),
                      (out / "network.json", net)):
        with open(path, "w", encoding="utf-8", newline="\n") as fh:
            json.dump(obj, fh, indent=2)
            fh.write("\n")
    return {"dir": str(out), "scene": Path(scene_path).name, "robot": robot_name,
            "scene_id": scene.get("id"),
            "scene_sha256": config_digest(scene),
            "robot_sha256": config_digest(robot),
            "controller_type": robot["controller"]["type"],
            "home": list(home_geo_point(scene)),
            "clock_ratio": scene_clock_ratio(scene),
            "network": net}


# --------------------------------------------------------------------------- #
# ArduPilot SITL and the router
# --------------------------------------------------------------------------- #

def sitl_args(scene: dict, plan: NetworkPlan, param_files=(),
              binary: str = AP_BINARY, param_dir: str = AP_PARAM_DIR,
              instance: int = 0, wipe: bool = True) -> list[str]:
    """The arducopter command line `sim_vehicle.py -v ArduCopter -f
    airsim-copter` would build, with our param file appended.

    The binary is run directly (as sitl/start_sitl.sh does) so the pinned
    build is what flies and no MAVProxy sits on the MAVLink path; the router
    is mavlink-router, as the grant names.

    `wipe` adds -w: the stored parameters (eeprom.bin in the run directory)
    are erased at every start, so --defaults - our param file last - is what
    flies, and a value changed in Mission Planner in an earlier session does
    not silently carry into a G1 repeat (sitl/start_sitl.sh wipes by default
    for the same reason).
    """
    defaults = [f"{param_dir}/{p}" for p in AP_DEFAULT_PARAMS] + [str(p) for p in param_files]
    return ([binary, "--model", "airsim-copter", "--speedup", "1",
             "--sim-address", plan.sitl_sim_address,
             "--sim-port-in", str(SITL_SENSOR_PORT),
             "--sim-port-out", str(SIM_SERVO_PORT),
             "--home", sitl_home(scene),
             "--defaults", ",".join(defaults)]
            + (["-w"] if wipe else [])
            + [f"-I{int(instance)}"])


def sim_vehicle_equivalent(scene: dict, plan: NetworkPlan, param_file: str) -> str:
    """The upstream one-liner (IAMAI's ardupilot example, PI's note) for the
    same launch, for people who already know sim_vehicle.py."""
    return ("sim_vehicle.py -v ArduCopter -f airsim-copter --no-rebuild "
            f"--no-mavproxy --sim-address={plan.sitl_sim_address} "
            f"--custom-location={sitl_home(scene)} "
            f"--add-param-file={param_file}")


def router_conf(gcs_address: str, node_address: str | None = None,
                mavros_address: str = "127.0.0.1",
                orin_address: str | None = None) -> str:
    """mavlink-router main.conf: SITL in, Mission Planner + node + MAVROS out.

    The TCP server (5790) is the firewall-free door from Windows: TCP is
    forwarded from Windows' localhost into WSL even under NAT, UDP is not.
    """
    for label, ip in (("gcs", gcs_address), ("node", node_address),
                      ("mavros", mavros_address), ("orin", orin_address)):
        if ip is not None:
            ipaddress.IPv4Address(ip)              # raises on a typo
    parts = [
        "# mavlink-router main.conf - rendered by demo/pas_ardupilot/rail.py",
        "[General]",
        f"TcpServerPort = {ROUTER_TCP_PORT}",
        "ReportStats = false",
        "MavlinkDialect = ardupilotmega",
        "",
        "[TcpEndpoint sitl]",
        "Address = 127.0.0.1",
        f"Port = {SITL_SERIAL0_TCP}",
        "",
        "[UdpEndpoint missionplanner]",
        "Mode = Normal",
        f"Address = {gcs_address}",
        f"Port = {GCS_UDP_PORT}",
        "",
        "[UdpEndpoint mavros]",
        "Mode = Normal",
        f"Address = {mavros_address}",
        f"Port = {MAVROS_UDP_PORT}",
    ]
    if node_address:
        parts += ["", "[UdpEndpoint perception]", "Mode = Normal",
                  f"Address = {node_address}", f"Port = {NODE_UDP_PORT}"]
    if orin_address:
        # hil: MAVROS on the Jetson Orin, the reference layout and the one the
        # flight topology keeps. The router initiates, so NAT is no obstacle.
        parts += ["", "[UdpEndpoint orin]", "Mode = Normal",
                  f"Address = {orin_address}", f"Port = {MAVROS_UDP_PORT}"]
    return "\n".join(parts) + "\n"


def mavproxy_router_args(gcs_address: str, node_address: str | None = None,
                         mavros_address: str = "127.0.0.1",
                         orin_address: str | None = None) -> list[str]:
    """The fallback fan-out when mavlink-routerd is not installed. It works,
    and it is NOT the grant's router; start_ardupilot.sh says so on screen
    and in endpoints.json.

    Two things differ from mavlink-router and are handled here:
      * a MAVProxy `tcpin` output accepts ONE client. With one shared port,
        whichever of the node and Mission Planner connected second sat unserved
        in the backlog, so the node keeps ROUTER_TCP_PORT and Mission Planner's
        TCP door is GCS_TCP_FALLBACK_PORT;
      * --streamrate=-1 keeps MAVProxy from rewriting the SR0_* stream rates
        the param file sets (sitl/start_router.sh passes it for the same reason).
    """
    outs = [f"udp:{gcs_address}:{GCS_UDP_PORT}",
            f"udp:{mavros_address}:{MAVROS_UDP_PORT}",
            f"tcpin:0.0.0.0:{ROUTER_TCP_PORT}",
            f"tcpin:0.0.0.0:{GCS_TCP_FALLBACK_PORT}"]
    if node_address:
        outs.append(f"udp:{node_address}:{NODE_UDP_PORT}")
    if orin_address:
        outs.append(f"udp:{orin_address}:{MAVROS_UDP_PORT}")
    return (["mavproxy.py", f"--master=tcp:127.0.0.1:{SITL_SERIAL0_TCP}"]
            + [f"--out={o}" for o in outs]
            + ["--streamrate=-1", "--daemon", "--non-interactive"])


# --------------------------------------------------------------------------- #
# Starting the WSL side from Windows
# --------------------------------------------------------------------------- #

def to_wsl_path(path: str | Path) -> str:
    """D:\\a\\b -> /mnt/d/a/b. A path that is already POSIX passes through."""
    s = str(path)
    if s.startswith("/"):
        return s
    p = PureWindowsPath(s)
    if not p.drive or not p.drive.endswith(":"):
        raise ValueError(f"{s!r} has no drive letter to map into /mnt")
    rest = "/".join(p.parts[1:])
    return f"/mnt/{p.drive[0].lower()}/{rest}"


def wsl_start_command(distro: str, plan: NetworkPlan, repo: str | Path = ROOT,
                      orin_address: str | None = None,
                      mavros: bool = False) -> list[str]:
    script = to_wsl_path(Path(repo) / "demo" / "pas_ardupilot" / "start_ardupilot.sh")
    cmd = ["wsl.exe", "-d", distro, "--", "bash", script,
           "--sim-address", plan.sitl_sim_address,
           "--gcs-address", plan.gcs_address,
           "--node-address", plan.windows_ip]
    if orin_address:
        cmd += ["--orin", orin_address]
    if mavros:
        cmd += ["--mavros"]
    return cmd


def wsl_stop_command(distro: str, repo: str | Path = ROOT) -> list[str]:
    script = to_wsl_path(Path(repo) / "demo" / "pas_ardupilot" / "start_ardupilot.sh")
    return ["wsl.exe", "-d", distro, "--", "bash", script, "--stop"]


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p_home = sub.add_parser("home")
    p_home.add_argument("--scene", default=str(SCENE_TEMPLATE))
    p_args = sub.add_parser("sitl-args")
    p_args.add_argument("--scene", default=str(SCENE_TEMPLATE))
    p_args.add_argument("--sim-address", required=True)
    p_args.add_argument("--param", action="append", default=[])
    p_args.add_argument("--binary", default=AP_BINARY)
    p_args.add_argument("--param-dir", default=AP_PARAM_DIR)
    p_args.add_argument("--no-wipe", action="store_true",
                        help="keep SITL's stored parameters (default: wipe, -w)")
    p_conf = sub.add_parser("router-conf")
    p_conf.add_argument("--gcs", required=True)
    p_conf.add_argument("--node", default=None)
    p_conf.add_argument("--mavros", default="127.0.0.1")
    p_conf.add_argument("--orin", default=None)
    p_mp = sub.add_parser("mavproxy-args")
    p_mp.add_argument("--gcs", required=True)
    p_mp.add_argument("--node", default=None)
    p_mp.add_argument("--orin", default=None)
    p_r = sub.add_parser("render")
    p_r.add_argument("--out", required=True)
    p_r.add_argument("--network", default="nat")
    p_r.add_argument("--windows-ip", default=None)
    p_r.add_argument("--wsl-ip", default=None)
    a = ap.parse_args(argv)

    if a.cmd == "home":
        print(sitl_home(load_jsonc(a.scene)))
    elif a.cmd == "sitl-args":
        # The sim address is used verbatim here: inside WSL the caller already
        # resolved it (gateway under NAT, 127.0.0.1 when mirrored).
        ipaddress.IPv4Address(a.sim_address)
        plan = NetworkPlan("given", a.sim_address, "127.0.0.1")
        for x in sitl_args(load_jsonc(a.scene), plan, a.param,
                           binary=a.binary, param_dir=a.param_dir,
                           wipe=not a.no_wipe):
            print(x)
    elif a.cmd == "router-conf":
        sys.stdout.write(router_conf(a.gcs, a.node, a.mavros, a.orin))
    elif a.cmd == "mavproxy-args":
        for x in mavproxy_router_args(a.gcs, a.node, orin_address=a.orin):
            print(x)
    elif a.cmd == "render":
        plan = plan_network(a.network, a.windows_ip, a.wsl_ip)
        print(json.dumps(write_sim_configs(a.out, plan), indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
