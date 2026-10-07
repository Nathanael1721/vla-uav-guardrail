"""deploy/: one image definition, three topologies, and every number agreeing.

Run either way:
    pytest tests/test_deploy_configs.py -v
    python tests/test_deploy_configs.py

WHAT IS BEING CLAIMED

deploy/ says the same stack runs in the grant's dev, hil and flight topologies
and that only a topology file (deploy/topologies/<name>.env) changes between
them. Nothing here has been through `docker build` yet, so these tests check
everything that can be checked without building - and each check is one that
a plausible edit would break:

  * the three topology files define the same keys, and every value a compose
    file reads is defined in each topology file it is used with; the ROS 2
    graph settings are the same in all three;
  * hil and flight on the Orin name the same images (promotion is config only);
  * a mission in a container starts exactly the nodes the native rail starts
    (sitl/run_ros2_demo.sh and its launch file: VLA stub, MAVLink adapter,
    Shield node), records the same topics, and nothing starts a second stub;
  * the router IS the native rail's (sitl/start_router.sh rendering
    sitl/mavlink-router/main.conf), and its ports agree with MAVROS's FCU_URL
    and with Mission Planner's 14550;
  * a run on the Orin with hil.env would EARN the hil label from
    guardrail.manifest: the FCU_URL names the desktop, the containers that
    gather the Orin's evidence can read the device tree and the L4T release,
    and the mission's process scan covers the whole host (pid: host);
  * what the Orin measures goes to a git-ignored folder, so the checkout the
    manifests read stays clean;
  * SITL in a container starts like the native scripts (EEPROM wiped, GeoFence
    backstop; for Project AirSim the scene's home and the airframe file), and
    refuses the CMAC home with Project AirSim;
  * the pins in the images are the pins of the environments the stored runs
    used, and no build context can carry the private signing key;
  * the entrypoint REFUSES what it must: an unknown topology, MAVROS with no
    FCU_URL, the arming mission in flight, a tag that escapes demo/out;
  * scripts keep LF endings (a CRLF shebang breaks every container).

The script tests need a POSIX bash. On Windows that is Git Bash; the WSL
launcher (System32\\bash.exe) is never used. Without one they print SKIP. The
compose tests that call `docker compose config` (no daemon, no pull) print
SKIP without the Docker CLI.
"""
from __future__ import annotations

import ipaddress
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import types
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import manifest as M                                  # noqa: E402
from guardrail.manifest import (TOPOLOGY_DEV, TOPOLOGY_FLIGHT,       # noqa: E402
                                TOPOLOGY_HIL)

SKIP = "SKIP"
DEPLOY = ROOT / "deploy"
TOPO_DIR = DEPLOY / "topologies"
COMPOSE_DIR = DEPLOY / "compose"
DOCKER_DIR = DEPLOY / "docker"
ENTRYPOINT = DOCKER_DIR / "entrypoint.sh"
SITL_ENTRYPOINT = DOCKER_DIR / "sitl-entrypoint.sh"
ROUTER_CONF = ROOT / "sitl" / "mavlink-router" / "main.conf"
START_ROUTER = ROOT / "sitl" / "start_router.sh"
START_SITL = ROOT / "sitl" / "start_sitl.sh"
RUN_ROS2_DEMO = ROOT / "sitl" / "run_ros2_demo.sh"
LAUNCH = ROOT / "sitl" / "ros2_ws" / "src" / "safety_shield" / "launch" / "guardrail_rail.launch.py"
CITYLIFE_PARAM = ROOT / "demo" / "pas_ardupilot" / "citylife-quad.param"
DDS = DEPLOY / "dds" / "cyclonedds.xml"
TOPOLOGIES = ("dev", "hil", "flight")
MISSION_PLANNER_PORT = 14550          # Mission Planner's default UDP listen port
GRANT_MAVLINK_PORT = 14550            # Architecture constraints p.2: "MAVLink UDP 14550"
CMAC_HOME = "-35.363261,149.165230,584,0"
ORIN_FILES = {"/etc/nv_tegra_release": "# R36 (release), REVISION: 4.0, GCID: 37537400, "
                                       "BOARD: generic, EABI: aarch64",
              "/proc/device-tree/model": "NVIDIA Jetson AGX Orin Developer Kit"}

_VAR = re.compile(r"\$\{([A-Z0-9_]+)(?::?[-?][^}]*)?\}")


def read_env(name: str) -> dict:
    out = {}
    for line in (TOPO_DIR / f"{name}.env").read_text(encoding="utf-8").splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        assert "=" in s, f"{name}.env: not KEY=VALUE: {line!r}"
        k, v = s.split("=", 1)
        assert " #" not in v, f"{name}.env: inline comment in {k} (compose parses it oddly)"
        out[k.strip()] = v.strip()
    return out


def compose_files() -> dict:
    return {p.name: p for p in sorted(COMPOSE_DIR.glob("docker-compose.*.yml"))}


def load_compose(p: Path) -> dict:
    return yaml.safe_load(p.read_text(encoding="utf-8"))


def companion_services(doc: dict) -> dict:
    return {n: s for n, s in doc["services"].items()
            if str(s.get("image", "")).startswith("${VLAGUARD_IMAGE")}


def entrypoint_roles() -> set[str]:
    text = ENTRYPOINT.read_text(encoding="utf-8")
    body = text.split('case "$role" in', 2)[-1]
    return set(re.findall(r"^\s{2}([a-z][a-z-]*)\)", body, flags=re.M))


def fcu_port(url: str) -> int:
    m = re.fullmatch(r"udp://[^:@]*:(\d+)@.*", url)
    assert m, f"FCU_URL {url!r} is not udp://[bind]:PORT@[remote]"
    return int(m.group(1))


def parse_router_conf(text: str) -> dict:
    sections, cur = {}, None
    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        if s.startswith("["):
            cur = s.strip("[]")
            sections[cur] = {}
        elif cur is not None and "=" in s:
            k, v = (x.strip() for x in s.split("=", 1))
            sections[cur][k] = v
    return sections


def find_bash() -> str | None:
    if os.name == "nt":
        for cand in (r"C:\Program Files\Git\bin\bash.exe",
                     r"C:\Program Files\Git\usr\bin\bash.exe"):
            if Path(cand).is_file():
                return cand
        return None                      # never System32\bash.exe: that starts WSL
    return shutil.which("bash")


def _base_env(env: dict, dry: bool = True) -> dict:
    full = {k: v for k, v in os.environ.items()
            if k not in ("VLAGUARD_TOPOLOGY", "ROS_DISTRO", "VLAGUARD_DRY_RUN")}
    full.update(env)
    full.update({"VLAGUARD_ROOT": ROOT.as_posix(),
                 "VLAGUARD_PYTHON": Path(sys.executable).as_posix(),
                 "VLAGUARD_RUNTIME_DIR": Path(tempfile.mkdtemp()).as_posix()})
    if dry:
        full["VLAGUARD_DRY_RUN"] = "1"
    return full


def dry_run(script: Path, args: list[str], env: dict) -> subprocess.CompletedProcess:
    return subprocess.run([find_bash(), script.as_posix(), *args], env=_base_env(env),
                          capture_output=True, text=True, timeout=60)


def printed(r: subprocess.CompletedProcess, tag: str = "EXEC") -> list[str]:
    """The argv a dry run printed after `TAG:` (shell-quoted with %q)."""
    for line in r.stdout.splitlines():
        if line.startswith(f"{tag}:"):
            return shlex.split(line[len(tag) + 1:])
    raise AssertionError(f"no {tag}: line in {r.stdout!r} / {r.stderr!r}")


def options(argv: list[str]) -> dict:
    """--flag value pairs of an argv (flags without a value map to True)."""
    out, i = {}, 0
    while i < len(argv):
        a = argv[i]
        if a.startswith("--") and i + 1 < len(argv) and not argv[i + 1].startswith("--"):
            out[a] = argv[i + 1]
            i += 2
        else:
            if a.startswith("-"):
                out[a] = True
            i += 1
    return out


# --------------------------------------------------------------------------- #
# topology files
# --------------------------------------------------------------------------- #
def test_three_topology_files_with_the_same_keys_and_the_grants_names():
    envs = {t: read_env(t) for t in TOPOLOGIES}
    keys = {t: set(e) for t, e in envs.items()}
    assert keys["dev"] == keys["hil"] == keys["flight"], \
        {t: sorted(k ^ keys["dev"]) for t, k in keys.items()}
    for t, e in envs.items():
        assert e["VLAGUARD_TOPOLOGY"] == t, (t, e["VLAGUARD_TOPOLOGY"])
    assert {e["VLAGUARD_TOPOLOGY"] for e in envs.values()} == \
        {TOPOLOGY_DEV, TOPOLOGY_HIL, TOPOLOGY_FLIGHT}, "names differ from guardrail.manifest"
    assert sorted(p.stem for p in TOPO_DIR.glob("*.env")) == sorted(TOPOLOGIES)


def test_the_ros_graph_is_configured_the_same_in_every_topology():
    """Only addresses may differ: a different domain or middleware between
    hil and flight would be a change the Orin's nodes notice."""
    envs = [read_env(t) for t in TOPOLOGIES]
    for key in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "MAVROS_UDP_PORT",
                "SITL_MAVLINK_PORT", "GCS_UDP_PORT", "VLAGUARD_BAG"):
        assert len({e[key] for e in envs}) == 1, (key, [e[key] for e in envs])
    assert envs[0]["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"


def test_hil_and_flight_run_the_same_images_and_dev_the_same_companion():
    dev, hil, flight = (read_env(t) for t in TOPOLOGIES)
    for key in ("VLAGUARD_IMAGE", "VLAGUARD_VLA_IMAGE", "VLA_BASE_IMAGE"):
        assert hil[key] == flight[key], f"{key}: hil {hil[key]} != flight {flight[key]}"
    assert dev["VLAGUARD_IMAGE"] == hil["VLAGUARD_IMAGE"], \
        "the companion image is one multi-arch definition; dev must name the same tag"
    assert "l4t" in hil["VLA_BASE_IMAGE"], "the Orin VLA image must be built on L4T"


def test_hil_puts_mavros_on_the_orin_and_names_the_desktop_as_its_autopilot():
    hil = read_env("hil")
    d, o = ipaddress.ip_address(hil["DESKTOP_IP"]), ipaddress.ip_address(hil["ORIN_IP"])
    assert d != o
    assert ipaddress.ip_network(f"{d}/24", strict=False) == \
        ipaddress.ip_network(f"{o}/24", strict=False), "desktop and Orin not on one /24"
    assert hil["MAVROS_HOST"] == hil["ORIN_IP"], "in hil the router must feed the Orin's MAVROS"
    assert {hil["VLAGUARD_DDS_PEER_A"], hil["VLAGUARD_DDS_PEER_B"]} == \
        {hil["DESKTOP_IP"], hil["ORIN_IP"]}
    assert M.fcu_peer(hil["FCU_URL"]) == hil["DESKTOP_IP"], \
        f"hil FCU_URL {hil['FCU_URL']!r} does not name the desktop as the autopilot"
    assert fcu_port(hil["FCU_URL"]) == int(hil["MAVROS_UDP_PORT"])
    dev = read_env("dev")
    assert dev["MAVROS_HOST"] == "127.0.0.1", "dev shares the router's network namespace"
    assert M.fcu_peer(dev["FCU_URL"]) is None, "dev's MAVROS link is local"


def test_flight_has_no_link_until_someone_sets_it_and_never_arms_by_default():
    for t in TOPOLOGIES:
        e = read_env(t)
        assert e["VLAGUARD_ALLOW_FLIGHT_MISSION"] == "no", t
        assert e["FCU_DEVICE"] == "", f"{t}: no device is mapped until flight names one"
    f = read_env("flight")
    assert f["FCU_URL"] == "", "flight must not ship a guessed autopilot link"


# --------------------------------------------------------------------------- #
# the label: what guardrail.manifest will say about a run made this way
# --------------------------------------------------------------------------- #
def _orin_evidence(fcu_url: str, autopilot_kind: str, iface: str = "eth0") -> dict:
    ev = {"ros_distro": "jazzy", "mavros_node": True, "fcu_connected": True,
          "ardupilot_version": "ArduCopter V4.5.7 (2a3dc4b7)",
          **M.collect_host_evidence(read=lambda p: ORIN_FILES.get(p),
                                    machine=lambda: "aarch64", jetpack=lambda: None),
          "autopilot_kind": autopilot_kind,
          "network_link": M.network_link_evidence(fcu_url, route_iface=lambda p: iface)}
    return ev


def test_a_hil_run_on_the_orin_with_hil_env_earns_the_hil_label():
    """The Shield node labels a run from evidence. With hil.env's FCU_URL on
    an Orin, talking to SITL across the cable, the evidence must support hil
    even where the host's processes could not be scanned; with the
    empty-remote URL of dev and no scan it must not (that was the defect:
    every Orin run would have been stamped dev). The desktop's address in the
    URL also gives the link record its interface (Ethernet), which an empty
    remote never does."""
    hil = read_env("hil")
    ev = _orin_evidence(hil["FCU_URL"], "sitl")
    assert M.check_topology_evidence("hil", ev) == [], M.check_topology_evidence("hil", ev)
    assert M.detect_topology(ev) == "hil"
    assert ev["network_link"]["peer"] == hil["DESKTOP_IP"]
    assert ev["network_link"]["link_kind"] == "ethernet"
    local = _orin_evidence(read_env("dev")["FCU_URL"], "sitl")
    why = M.check_topology_evidence("hil", local)
    assert any("names no remote desktop" in w for w in why), why
    assert M.detect_topology(local) == "dev"


def test_the_mission_scans_the_whole_host_for_sitl_and_mavros():
    """The Shield node reads which MAVLink actors run on its host from /proc
    (guardrail.manifest.scan_local_processes): an ArduPilot SITL there makes
    the run dev, and `mavros_on` records where MAVROS ran. A container sees
    only its own PID namespace, so without `pid: host` the mission would miss
    a SITL started on the Orin itself and record MAVROS, which runs in the
    sibling mavros container, as remote. Only the mission scans, so no other
    service gets the host's PID namespace."""
    for fname in ("docker-compose.orin.yml", "docker-compose.dev.yml"):
        doc = load_compose(COMPOSE_DIR / fname)
        for name, svc in doc["services"].items():
            if name == "mission":
                assert svc.get("pid") == "host", \
                    f"{fname}: the mission would scan only its own container's processes"
            else:
                assert svc.get("pid") is None, \
                    f"{fname}: {name} shares the host's PID namespace without needing it"
    ev = _orin_evidence(read_env("hil")["FCU_URL"], "sitl")
    scan = {"scanned": "/proc", "local_sitl": [], "local_mavros": True, "local_router": []}
    # The Orin as the mission sees it with pid: host: MAVROS beside it, no SITL.
    on_host = {**ev, "local_processes": scan}
    assert M.mavros_location(on_host) == "shield_host"
    assert M.check_topology_evidence("hil", on_host) == [], M.check_topology_evidence("hil", on_host)
    # A SITL started on the Orin by mistake is seen, and the run is not hil.
    stray = {**ev, "local_processes": {**scan, "local_sitl": ["arducopter"]}}
    why = M.check_topology_evidence("hil", stray)
    assert any("SITL runs on the Shield's host" in w for w in why), why
    assert M.detect_topology(stray) == "dev"
    # Its own PID namespace only: the sibling MAVROS would be recorded as remote.
    own_ns = {**ev, "local_processes": {**scan, "local_mavros": False}}
    assert M.mavros_location(own_ns) == "remote"


def test_a_flight_run_with_a_serial_link_earns_the_flight_label():
    ev = _orin_evidence("serial:///dev/ttyACM0:115200", "hardware")
    assert M.check_topology_evidence("flight", ev) == [], M.check_topology_evidence("flight", ev)
    assert M.detect_topology(ev) == "flight"


def test_the_containers_that_gather_evidence_can_read_it():
    """guardrail.manifest reads the board name from the device tree and the
    L4T release from /etc/nv_tegra_release. Docker masks /sys/firmware (where
    /proc/device-tree points) in a default container, so every service that
    gathers evidence must lift the masking and mount the release file; the
    others must not lift it (least privilege)."""
    doc = load_compose(COMPOSE_DIR / "docker-compose.orin.yml")
    ev_services = doc["x-vlaguard"]["evidence_services"]
    assert set(ev_services) == {"mission", "profile", "facts", "vla-bench"}, ev_services
    assert all(p.startswith(("/proc/device-tree", "/sys/firmware")) for p in M._DT_MODEL_PATHS), \
        "the manifest reads the board name somewhere Docker does not mask: revisit the opt-out"
    for name, svc in doc["services"].items():
        unconfined = "systempaths=unconfined" in (svc.get("security_opt") or [])
        if name in ev_services:
            assert unconfined, f"orin/{name} cannot read /proc/device-tree/model"
            vols = svc["volumes"]
            assert any(v == f"{M._TEGRA_RELEASE}:{M._TEGRA_RELEASE}:ro" for v in vols), name
        else:
            assert not unconfined, f"orin/{name} lifts Docker's masking without needing to"
    assert doc["services"]["vla-bench"].get("runtime") == "nvidia"


def test_what_the_orin_measures_goes_where_git_ignores_it():
    """A file written into a tracked path on the Orin makes the checkout dirty,
    and every later manifest there says `-dirty`, which the KPI gate refuses."""
    doc = load_compose(COMPOSE_DIR / "docker-compose.orin.yml")
    want = "/opt/vlaguard/deploy/evidence/incoming"
    assert doc["x-companion-env"]["VLAGUARD_EVIDENCE_OUT"] == want
    assert doc["services"]["vla-bench"]["environment"]["VLAGUARD_EVIDENCE_OUT"] == want
    for svc in companion_services(doc).values():
        assert svc["environment"]["VLAGUARD_EVIDENCE_OUT"] == want
    if not shutil.which("git"):
        return SKIP
    for rel in ("deploy/evidence/incoming/hil/shield_tick_profile.json",
                "deploy/evidence/incoming/flight/vla_rate_stub.json"):
        r = subprocess.run(["git", "-C", str(ROOT), "check-ignore", "-q", rel])
        assert r.returncode == 0, f"{rel} is not git-ignored"
    r = subprocess.run(["git", "-C", str(ROOT), "check-ignore", "-q",
                        "deploy/evidence/incoming/.gitignore"])
    assert r.returncode == 1, "the ignore file itself must stay tracked"


# --------------------------------------------------------------------------- #
# compose files
# --------------------------------------------------------------------------- #
def test_every_value_a_compose_file_reads_is_in_each_of_its_topology_files():
    files = compose_files()
    assert set(files) == {"docker-compose.dev.yml", "docker-compose.hil-desktop.yml",
                          "docker-compose.orin.yml", "docker-compose.orin.flight.yml"}, \
        sorted(files)
    for name, p in files.items():
        doc = load_compose(p)
        topos = doc["x-vlaguard"]["topologies"]
        used = set(_VAR.findall(p.read_text(encoding="utf-8")))
        for t in topos:
            env = read_env(t)
            missing = sorted(v for v in used if v not in env)
            assert not missing, f"{name} with {t}.env: undefined {missing}"


def test_the_orin_file_serves_hil_and_flight_and_only_flight_maps_a_device():
    docs = {n: load_compose(p) for n, p in compose_files().items()}
    assert docs["docker-compose.orin.yml"]["x-vlaguard"]["topologies"] == ["hil", "flight"]
    assert docs["docker-compose.dev.yml"]["x-vlaguard"]["topologies"] == ["dev"]
    assert docs["docker-compose.hil-desktop.yml"]["x-vlaguard"]["topologies"] == ["hil"]
    fl = docs["docker-compose.orin.flight.yml"]
    assert fl["x-vlaguard"]["topologies"] == ["flight"]
    assert fl["x-vlaguard"]["overrides"] == "docker-compose.orin.yml"
    assert set(fl["services"]) == {"mavros"} and set(fl["services"]["mavros"]) == {"devices"}
    assert fl["name"] == docs["docker-compose.orin.yml"]["name"], "an override must share the project"
    orin = docs["docker-compose.orin.yml"]["services"]
    assert not any("sitl" in n for n in orin), "no simulator runs on the Orin"
    for n, s in orin.items():
        assert "devices" not in s, f"orin/{n} maps a device in hil too"
        if n in ("profile", "facts", "vla-bench"):
            continue
        assert s.get("network_mode") == "host", f"orin/{n} is not on host networking"


def test_every_companion_service_runs_a_role_the_entrypoint_knows():
    roles = entrypoint_roles()
    assert roles == {"router", "mavros", "mission", "profile", "facts", "vla-table",
                     "shell"}, roles
    for name, p in compose_files().items():
        doc = load_compose(p)
        for svc_name, svc in companion_services(doc).items():
            if "command" not in svc:
                continue                       # an override adds settings only
            role = svc["command"][0]
            assert role in roles, f"{name}/{svc_name}: role {role!r} unknown to entrypoint.sh"


def test_every_file_the_entrypoint_starts_exists():
    text = ENTRYPOINT.read_text(encoding="utf-8")
    paths = set(re.findall(r'\$ROOT/([\w/.-]+\.(?:py|sh))', text))
    paths |= set(re.findall(r'^LAUNCH_FILE="([^"]+)"', text, flags=re.M))
    assert {"sitl/start_router.sh", "tools/profile_shield_tick.py",
            "sitl/ros2_ws/src/safety_shield/launch/guardrail_rail.launch.py"} <= paths, paths
    for rel in paths:
        assert (ROOT / rel).is_file(), f"entrypoint.sh starts {rel}, which does not exist"


def test_pending_services_are_still_pending_and_nothing_runs_them():
    doc = load_compose(COMPOSE_DIR / "docker-compose.orin.yml")
    text = (COMPOSE_DIR / "docker-compose.orin.yml").read_text(encoding="utf-8")
    for item in doc["x-vlaguard"].get("pending") or []:
        assert not (ROOT / item["needs"]).exists(), \
            f"{item['needs']} exists now: add the {item['service']} service and drop it from pending"
        assert item["service"] not in doc["services"]
        body = text.split("\nservices:", 1)[1]
        assert item["needs"] not in body, f"a service references pending {item['needs']}"


def test_ros_nodes_share_one_network_namespace_in_dev():
    doc = load_compose(COMPOSE_DIR / "docker-compose.dev.yml")
    s = doc["services"]
    for n in ("sitl", "mavros", "mission"):
        assert s[n].get("network_mode") == "service:router", n
        assert "ports" not in s[n], f"{n} publishes ports from a shared namespace"
    assert "9003:9003/udp" in s["router"]["ports"], "Project AirSim cannot reach SITL's 9003"


def test_the_repository_is_mounted_read_only_and_outputs_are_writable():
    for name, p in compose_files().items():
        doc = load_compose(p)
        for svc_name, svc in doc["services"].items():
            vols = svc.get("volumes") or []
            repo = [v for v in vols if v.startswith("../..:")]
            if not repo:
                continue
            assert repo[0].endswith(":ro"), f"{name}/{svc_name} mounts the checkout writable"
    dev = load_compose(COMPOSE_DIR / "docker-compose.dev.yml")["services"]
    assert "../..:/opt/vlaguard:ro" in dev["sitl"]["volumes"], \
        "SITL needs the fence and airframe parameter files from the checkout"


def _compose_config(files: list[str], env_file: str, extra_env: dict | None = None):
    docker = shutil.which("docker")
    if not docker:
        return None
    cmd = [docker, "compose", "--env-file", str(TOPO_DIR / env_file)]
    for f in files:
        cmd += ["-f", str(COMPOSE_DIR / f)]
    env = {k: v for k, v in os.environ.items() if not k.startswith(("VLAGUARD", "FCU_"))}
    env.update(extra_env or {})
    return subprocess.run(cmd + ["config", "--quiet"], capture_output=True, text=True,
                          timeout=120, env=env)


def test_docker_compose_accepts_every_file_and_flight_refuses_an_unnamed_device():
    """`docker compose config` only parses and interpolates: no daemon, no
    image, no network."""
    r = _compose_config(["docker-compose.dev.yml"], "dev.env")
    if r is None:
        return SKIP
    assert r.returncode == 0, r.stderr
    for files, env in ((["docker-compose.hil-desktop.yml"], "hil.env"),
                       (["docker-compose.orin.yml"], "hil.env"),
                       (["docker-compose.orin.yml"], "flight.env")):
        r = _compose_config(files, env)
        assert r.returncode == 0, (files, env, r.stderr)
    both = ["docker-compose.orin.yml", "docker-compose.orin.flight.yml"]
    r = _compose_config(both, "flight.env")
    assert r.returncode != 0 and "FCU_DEVICE" in r.stderr, (r.returncode, r.stderr)
    r = _compose_config(both, "flight.env", {"FCU_DEVICE": "/dev/ttyACM0"})
    assert r.returncode == 0, r.stderr


# --------------------------------------------------------------------------- #
# the mission is the native rail's mission
# --------------------------------------------------------------------------- #
def _launch_nodes() -> set[str]:
    return set(re.findall(r'sitl / "(\w+\.py)"', LAUNCH.read_text(encoding="utf-8")))


def _native_nodes() -> set[str]:
    return set(re.findall(r'"\$DIR/(\w+_node\.py)"', RUN_ROS2_DEMO.read_text(encoding="utf-8")))


def _topics(text: str) -> list[str]:
    return re.findall(r"(?<![\w$])(/[a-z_]+(?:/[a-z_0-9]+)+)", text)


def test_the_mission_starts_every_node_the_native_rail_starts():
    """The launch file is the native rail's (run_ros2_demo.sh, USE_LAUNCH=1).
    Its nodes must be exactly the ones the rail's fallback path starts, so the
    MAVLink adapter cannot go missing from one of them, and the entrypoint must
    reach the nodes only through it."""
    nodes = _launch_nodes()
    assert nodes == {"ros2_vla_stub_node.py", "mavlink_adapter_node.py",
                     "ros2_shield_node.py"}, nodes
    assert nodes == _native_nodes(), (nodes, _native_nodes())
    assert "guardrail_rail.launch.py" in RUN_ROS2_DEMO.read_text(encoding="utf-8")
    text = ENTRYPOINT.read_text(encoding="utf-8")
    assert not re.search(r"sitl/\w+_node\.py", text), "the entrypoint starts a node directly"
    declared = set(re.findall(r'DeclareLaunchArgument\(\s*"(\w+)"', LAUNCH.read_text(encoding="utf-8")))
    passed = set(re.findall(r'"(\w+):=\$', text.split("launch=(", 1)[1].split(")", 1)[0]))
    assert passed and passed <= declared, (passed, declared)
    assert {"python", "repo_root", "out", "shield_args"} <= passed, passed


def test_the_mission_records_the_native_rails_topics():
    native = RUN_ROS2_DEMO.read_text(encoding="utf-8")
    block = native.split('ros2 bag record -s mcap -o "$OUT/bag"', 1)[1].split(">", 1)[0]
    mine = ENTRYPOINT.read_text(encoding="utf-8").split("BAG_TOPICS=(", 1)[1].split(")", 1)[0]
    assert _topics(block) and _topics(mine) == _topics(block), (_topics(mine), _topics(block))


def test_nothing_starts_a_second_vla_stub():
    """The launch file runs the stub. A compose service that ran one too would
    put two publishers on /vla/action_4d."""
    for name, p in compose_files().items():
        for svc_name, svc in load_compose(p)["services"].items():
            cmd = svc.get("command") or []
            assert "vla-stub" not in cmd and "vla-stub" not in svc_name, (name, svc_name)
    assert "vla-stub" not in entrypoint_roles()


# --------------------------------------------------------------------------- #
# the router is the native rail's router
# --------------------------------------------------------------------------- #
def test_the_native_router_config_has_the_ports_every_topology_uses():
    conf = parse_router_conf(ROUTER_CONF.read_text(encoding="utf-8"))
    assert conf["UdpEndpoint sitl"]["Mode"] == "Server"
    assert conf["UdpEndpoint mavros"]["Mode"] == "Normal"
    assert conf["General"]["TcpServerPort"] == "0"
    for t in ("dev", "hil"):
        env = read_env(t)
        assert int(conf["UdpEndpoint sitl"]["Port"]) == int(env["SITL_MAVLINK_PORT"]) \
            == GRANT_MAVLINK_PORT, t
        assert int(conf["UdpEndpoint mission_planner"]["Port"]) == int(env["GCS_UDP_PORT"]) \
            == MISSION_PLANNER_PORT, t
        assert fcu_port(env["FCU_URL"]) == int(env["MAVROS_UDP_PORT"]), t
    assert "@MAVROS_HOST@" in ROUTER_CONF.read_text(encoding="utf-8")


def _run_router(t: str) -> subprocess.CompletedProcess:
    """The entrypoint's router role, for real, with a stand-in mavlink-routerd
    on PATH that prints the configuration it was handed."""
    d = Path(tempfile.mkdtemp())
    fake = d / "bin"
    fake.mkdir()
    (fake / "mavlink-routerd").write_bytes(
        b'#!/usr/bin/env bash\necho "FAKE-ROUTERD $*"\n'
        b'while [ $# -gt 0 ]; do [ "$1" = "-c" ] && cat "$2"; shift; done\n')
    os.chmod(fake / "mavlink-routerd", 0o755)
    env = _base_env({**read_env(t), "GCS_HOST": "10.0.0.9"}, dry=False)
    env["PATH"] = str(fake) + os.pathsep + env.get("PATH", "")
    env["HOME"] = str(d)
    return subprocess.run([find_bash(), ENTRYPOINT.as_posix(), "router"], env=env,
                          capture_output=True, text=True, timeout=60)


def test_the_router_role_runs_the_native_router_with_the_topologys_addresses():
    if not find_bash():
        return SKIP
    for t in ("dev", "hil"):
        env = read_env(t)
        r = _run_router(t)
        assert r.returncode == 0, (t, r.stderr)
        assert "FAKE-ROUTERD -c" in r.stdout, r.stdout
        conf = parse_router_conf(r.stdout.split("FAKE-ROUTERD", 1)[1].split("\n", 1)[1])
        native = parse_router_conf(ROUTER_CONF.read_text(encoding="utf-8"))
        assert conf["General"] == native["General"], "not the native rail's router config"
        assert conf["UdpEndpoint mavros"]["Address"] == env["MAVROS_HOST"], (t, conf)
        assert int(conf["UdpEndpoint mavros"]["Port"]) == fcu_port(env["FCU_URL"]), t
        assert conf["UdpEndpoint mission_planner"]["Address"] == "10.0.0.9", t
        for ep in ("sitl", "mavros", "mission_planner"):
            assert conf[f"UdpEndpoint {ep}"]["Mode"] == native[f"UdpEndpoint {ep}"]["Mode"]
    assert parse_router_conf(_run_router("hil").stdout.split("FAKE-ROUTERD", 1)[1]
                             .split("\n", 1)[1])["UdpEndpoint mavros"]["Address"] \
        == read_env("hil")["ORIN_IP"]


def test_the_router_role_refuses_a_name_it_cannot_resolve():
    if not find_bash():
        return SKIP
    r = dry_run(ENTRYPOINT, ["router"], {**read_env("hil"), "GCS_HOST": "no-such-host.invalid"})
    assert r.returncode == 68 and "cannot resolve" in r.stderr, (r.returncode, r.stderr)


def test_dds_peers_are_values_every_topology_and_compose_file_provides():
    root = ET.parse(DDS).getroot()
    ns = {"c": "https://cdds.io/config"}
    peers = [p.get("address") for p in root.iterfind(".//c:Peer", ns)]
    names = {m for a in peers for m in _VAR.findall(a)}
    assert names == {"VLAGUARD_DDS_PEER_A", "VLAGUARD_DDS_PEER_B"}, names
    for t in TOPOLOGIES:
        env = read_env(t)
        for n in names:
            ipaddress.ip_address(env[n])
    for fname in ("docker-compose.dev.yml", "docker-compose.orin.yml"):
        text = (COMPOSE_DIR / fname).read_text(encoding="utf-8")
        for n in names:
            assert f"{n}: ${{{n}" in text, f"{fname} does not pass {n} to the containers"


# --------------------------------------------------------------------------- #
# images and pins
# --------------------------------------------------------------------------- #
def _dockerignore_allowed(dockerfile: Path) -> list[str]:
    ign = Path(str(dockerfile) + ".dockerignore")
    lines = [l.strip() for l in ign.read_text(encoding="utf-8").splitlines()
             if l.strip() and not l.strip().startswith("#")]
    assert lines[0] == "*", f"{ign.name}: must exclude everything first"
    return [l[1:] for l in lines[1:] if l.startswith("!")]


def _copy_sources(dockerfile: Path) -> list[str]:
    out = []
    for line in dockerfile.read_text(encoding="utf-8").splitlines():
        s = line.strip()
        if s.startswith("COPY ") and "--from=" not in s:
            out += s.split()[1:-1]
    return out


def test_build_contexts_hold_only_what_each_dockerfile_copies():
    for df in sorted(DOCKER_DIR.glob("*.Dockerfile")):
        allowed = _dockerignore_allowed(df)
        for src in _copy_sources(df):
            assert src in allowed, f"{df.name} copies {src}, which its .dockerignore excludes"
            assert (ROOT / src).is_file(), f"{df.name} copies missing {src}"
        for a in allowed:
            assert not a.startswith("policies"), f"{df.name}: policies/ holds the private key"
            assert a in _copy_sources(df), f"{df.name}: lets in {a} but never copies it"


def test_the_companion_image_is_one_multiarch_ros2_definition():
    text = (DOCKER_DIR / "companion.Dockerfile").read_text(encoding="utf-8")
    m = re.search(r"^ARG BASE_IMAGE=(\S+)", text, flags=re.M)
    assert m and m.group(1).startswith("ros:jazzy-"), "base is not the official ROS 2 Jazzy image"
    assert "ros-${ROS_DISTRO}-mavros" in text and "rmw-cyclonedds-cpp" in text
    assert "rosbag2-storage-mcap" in text, "the mission role records with -s mcap"
    assert "install_geographiclib_datasets.sh" in text, "MAVROS exits without the geoids"
    assert "safe.directory" in text, "git would refuse the mounted checkout: 'unversioned'"
    assert "mavlink-routerd" in text
    assert "-r /tmp/req/requirements.txt" in text, "not the guardrail's own requirement list"
    assert not re.search(r"^COPY\s+(guardrail|sitl|policies|tools)\b", text, flags=re.M), \
        "code baked into the image: code_revision would not name the commit that flew"
    # A build argument that changes what is installed would make "the same
    # image" in hil and flight a matter of how it was built.
    args = set(re.findall(r"^ARG (\w+)", text, flags=re.M))
    assert args <= {"BASE_IMAGE", "MAVLINK_ROUTER_REF", "ROS_DISTRO"}, args
    assert "torch" not in text.lower().replace("pytorch is not here", ""), \
        "PyTorch belongs to the vla image"


def test_the_sitl_image_builds_the_pinned_ardupilot():
    df = (DOCKER_DIR / "sitl.Dockerfile").read_text(encoding="utf-8")
    sh = (ROOT / "sitl" / "setup_sitl.sh").read_text(encoding="utf-8")
    for key in ("ARDUPILOT_REF", "ARDUPILOT_SHA"):
        want = re.search(rf'{key}="\$\{{{key}:-([^}}]+)\}}"', sh).group(1)
        got = re.search(rf"^ARG {key}=(\S+)", df, flags=re.M).group(1)
        assert got == want, f"{key}: sitl.Dockerfile {got} != sitl/setup_sitl.sh {want}"
    assert 'test "$(git rev-parse HEAD)" = "${ARDUPILOT_SHA}"' in df, "the SHA is not checked"
    assert "airsim-quadX.parm" in df and "copter.parm" in df


def _design_row(env_name: str) -> str:
    doc = (ROOT / "docs" / "DESIGN-python-versions.md").read_text(encoding="utf-8")
    for line in doc.splitlines():
        if line.startswith("|") and env_name in line.split("|")[1]:
            return line
    raise AssertionError(f"no row for {env_name} in DESIGN-python-versions.md")


def _pins(path: Path) -> dict:
    out = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        s = line.split("#", 1)[0].strip()
        if "==" in s:
            k, v = s.split("==")
            out[k.strip()] = v.strip()
    return out


def test_companion_pins_are_the_kpi_rails_versions():
    row = _design_row("venv-ros").lower()
    pins = _pins(DOCKER_DIR / "constraints-companion.txt")
    assert pins, "no pins"
    for k, v in pins.items():
        assert f"{k.lower()} {v}" in row, f"{k}=={v} is not what ~/venv-ros ran"


def test_vla_pins_are_vla_reals_versions():
    row = _design_row("vla-real").lower()
    pins = _pins(DOCKER_DIR / "requirements-vla.txt")
    assert {"transformers", "tokenizers", "timm", "accelerate", "peft"} <= set(pins)
    for k, v in pins.items():
        assert f"{k.lower()} {v}" in row, f"{k}=={v} is not what vla-real ran"


# --------------------------------------------------------------------------- #
# files themselves
# --------------------------------------------------------------------------- #
def test_deploy_text_files_have_lf_endings_and_scripts_a_bash_shebang():
    text_ext = {".sh", ".yml", ".env", ".xml", ".md", ".txt", ".json", ".dockerignore",
                ".Dockerfile"}
    checked = 0
    for p in DEPLOY.rglob("*"):
        if not p.is_file() or (p.suffix not in text_ext and p.name != ".gitignore"):
            continue
        data = p.read_bytes()
        assert b"\r\n" not in data, f"{p.relative_to(ROOT)} has CRLF line endings"
        checked += 1
    assert checked >= 15, checked
    for sh in (ENTRYPOINT, SITL_ENTRYPOINT, START_ROUTER):
        assert sh.read_text(encoding="utf-8").startswith("#!/usr/bin/env bash\n"), sh.name
        assert b"\r\n" not in sh.read_bytes(), f"{sh.name} would fail in a container"


# --------------------------------------------------------------------------- #
# the entrypoint, dry-run
# --------------------------------------------------------------------------- #
def _env_for(t: str, **over) -> dict:
    env = read_env(t)
    env.update({"GCS_HOST": "10.0.0.9", "AIRSIM_HOST": "10.0.0.8"})   # no DNS needed
    env.update(over)
    return env


def test_entrypoint_starts_each_role_with_the_topologys_values():
    if not find_bash():
        return SKIP
    hil = read_env("hil")
    r = dry_run(ENTRYPOINT, ["mavros"], _env_for("hil"))
    assert r.returncode == 0, r.stderr
    argv = printed(r)
    assert argv[:4] == ["ros2", "run", "mavros", "mavros_node"], argv
    assert f"fcu_url:={hil['FCU_URL']}" in argv, argv
    r = dry_run(ENTRYPOINT, ["profile"], _env_for("hil"))
    argv = printed(r)
    assert argv[1].endswith("tools/profile_shield_tick.py") and \
        argv[2:5] == ["profile", "--topology", "hil"], argv
    r = dry_run(ENTRYPOINT, ["router"], _env_for("hil"))
    assert r.returncode == 0, r.stderr
    argv = printed(r)
    assert argv[0] == "env" and f"MAVROS_HOST={hil['ORIN_IP']}" in argv, argv
    assert argv[-1].endswith("sitl/start_router.sh"), argv


def test_entrypoint_mission_runs_the_launch_file_then_packs_the_replay():
    if not find_bash():
        return SKIP
    r = dry_run(ENTRYPOINT, ["mission", "--shield", "on", "--tag", "hil_smoke_01"],
                _env_for("hil"))
    assert r.returncode == 0, r.stderr
    launch = printed(r)
    root = ROOT.as_posix()
    assert launch[:3] == ["ros2", "launch", f"{root}/{LAUNCH.relative_to(ROOT).as_posix()}"], launch
    kv = dict(a.split(":=", 1) for a in launch[3:])
    assert kv["repo_root"] == root and kv["out"] == f"{root}/demo/out/hil_smoke_01", kv
    assert shlex.split(kv["shield_args"]) == ["--shield", "on", "--tag", "hil_smoke_01"], kv
    assert shlex.split(kv["vla_args"]) == ["--tag", "hil_smoke_01"], kv
    bag = printed(r, "BAG")
    assert bag[:7] == ["ros2", "bag", "record", "-s", "mcap", "-o",
                       f"{root}/demo/out/hil_smoke_01/bag"], bag
    pack = printed(r, "THEN")
    assert pack[1:] == ["-m", "guardrail.replay", "pack", f"{root}/demo/out/hil_smoke_01"], pack
    r = dry_run(ENTRYPOINT, ["mission", "--shield", "on"], _env_for("dev", VLAGUARD_BAG="0"))
    assert "BAG:" not in r.stdout, r.stdout


def test_entrypoint_mission_tags_follow_the_shield_nodes_rule():
    if not find_bash():
        return SKIP
    for args, tag in ((["mission"], "ros2_shield_on"),
                      (["mission", "--shield", "off", "--dynamic"], "ros2_shield_off_dynamic")):
        r = dry_run(ENTRYPOINT, args, _env_for("dev"))
        kv = dict(a.split(":=", 1) for a in printed(r)[3:])
        assert kv["out"].endswith(f"/demo/out/{tag}"), (args, kv["out"])
        if "--dynamic" in args:
            assert "--dynamic" in shlex.split(kv["shield_args"])
            assert "--dynamic" not in shlex.split(kv["vla_args"] or "''")
    for bad in (["mission", "--tag", "../escape"], ["mission", "--tag"],
                ["mission", "--shield", "maybe"]):
        r = dry_run(ENTRYPOINT, bad, _env_for("dev"))
        assert r.returncode == 64, (bad, r.returncode, r.stderr)


def test_entrypoint_refuses_an_unknown_topology():
    if not find_bash():
        return SKIP
    r = dry_run(ENTRYPOINT, ["mavros"], _env_for("dev", VLAGUARD_TOPOLOGY="canonical-hil"))
    assert r.returncode == 64 and "not dev, hil or flight" in r.stderr, (r.returncode, r.stderr)


def test_entrypoint_refuses_mavros_with_no_autopilot_link():
    if not find_bash():
        return SKIP
    r = dry_run(ENTRYPOINT, ["mavros"], _env_for("flight"))
    assert r.returncode == 64 and "FCU_URL is empty" in r.stderr, (r.returncode, r.stderr)
    r = dry_run(ENTRYPOINT, ["mavros"],
                _env_for("flight", FCU_URL="serial:///dev/ttyACM0:115200"))
    assert r.returncode == 0 and "fcu_url:=serial:///dev/ttyACM0:115200" in printed(r), r


def test_entrypoint_refuses_the_arming_mission_in_flight_unless_told_yes():
    if not find_bash():
        return SKIP
    r = dry_run(ENTRYPOINT, ["mission", "--shield", "on"], _env_for("flight"))
    assert r.returncode == 77 and "arms a real aircraft" in r.stderr, (r.returncode, r.stderr)
    r = dry_run(ENTRYPOINT, ["mission", "--shield", "on"],
                _env_for("flight", VLAGUARD_ALLOW_FLIGHT_MISSION="yes"))
    assert r.returncode == 0, r.stderr
    r = dry_run(ENTRYPOINT, ["mission", "--shield", "on"], _env_for("hil"))
    assert r.returncode == 0, r.stderr


def test_entrypoint_refuses_a_missing_checkout_and_an_unknown_role():
    if not find_bash():
        return SKIP
    env = _env_for("dev")
    for role in ("no-such-role", "shield", "vla-stub"):
        r = dry_run(ENTRYPOINT, [role], env)
        assert r.returncode == 64, (role, r)
    full = {**_base_env(env), "VLAGUARD_ROOT": Path(tempfile.mkdtemp()).as_posix()}
    r = subprocess.run([find_bash(), ENTRYPOINT.as_posix(), "mavros"], env=full,
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 66 and "no repository" in r.stderr, (r.returncode, r.stderr)


# --------------------------------------------------------------------------- #
# SITL starts like the native scripts
# --------------------------------------------------------------------------- #
def _sitl(env: dict) -> subprocess.CompletedProcess:
    base = {"SITL_MODEL": "quad", "SITL_MAVLINK_PORT": "14550", "SITL_HOME": CMAC_HOME}
    return dry_run(SITL_ENTRYPOINT, [], {**base, **env})


def test_sitl_quad_starts_like_start_sitl_sh():
    """sitl/start_sitl.sh: EEPROM wiped at each start, the GeoFence backstop as
    a second defaults file, SERIAL0 a TCP server, SERIAL2 to the router."""
    if not find_bash():
        return SKIP
    native = START_SITL.read_text(encoding="utf-8")
    for s in ('SITL_WIPE:-1}" = "1" ] && WIPE=(-w)', 'GUARDRAIL_FENCE:-1', "--serial0 tcp:0",
              "fence/guardrail_fence.parm", f"--home {CMAC_HOME}", "--speedup 1",
              '--serial2 "udpclient:127.0.0.1:'):
        assert s in native, f"sitl/start_sitl.sh changed ({s!r}); re-check the container's start"
    r = _sitl({})
    assert r.returncode == 0, r.stderr
    argv = printed(r)
    opt = options(argv)
    assert opt["--model"] == "quad" and opt["--speedup"] == "1" and opt["--home"] == CMAC_HOME
    assert "-w" in argv, "the EEPROM is not wiped: a zone from the last flight arms at boot"
    assert opt["--defaults"].split(",") == ["/opt/ardupilot/params/copter.parm",
                                            f"{ROOT.as_posix()}/sitl/fence/guardrail_fence.parm"]
    assert opt["--serial0"] == "tcp:0" and opt["--serial2"] == "udpclient:127.0.0.1:14550"
    off = printed(_sitl({"GUARDRAIL_FENCE": "0", "SITL_WIPE": "0"}))
    assert "-w" not in off and "guardrail_fence" not in options(off)["--defaults"], off
    for bad in ({"SITL_WIPE": "2"}, {"GUARDRAIL_FENCE": "yes"}, {"SITL_MODEL": "gazebo"}):
        assert _sitl(bad).returncode == 64, bad
    gone = _sitl({"SITL_FENCE_PARM": "/nonexistent/guardrail_fence.parm"})
    assert gone.returncode == 66 and "GeoFence backstop" in gone.stderr, gone.stderr


def test_sitl_airsim_copter_refuses_the_cmac_home_and_a_missing_airframe():
    if not find_bash():
        return SKIP
    base = {"SITL_MODEL": "airsim-copter", "AIRSIM_HOST": "10.0.0.8",
            "SITL_AIRFRAME_PARM": CITYLIFE_PARAM.as_posix()}
    r = _sitl({**base, "SITL_HOME": CMAC_HOME})
    assert r.returncode == 64 and "CMAC" in r.stderr, (r.returncode, r.stderr)
    r = _sitl({**base, "SITL_HOME": ""})
    assert r.returncode == 64 and "home-geo-point" in r.stderr, r.stderr
    r = _sitl({**base, "SITL_HOME": "35.6895,139.6917,40.0,0.0", "SITL_AIRFRAME_PARM": ""})
    assert r.returncode == 64 and "SITL_AIRFRAME_PARM" in r.stderr, r.stderr
    r = _sitl({**base, "SITL_HOME": "35.6895,139.6917,40.0,0.0",
               "SITL_AIRFRAME_PARM": "/nonexistent.param"})
    assert r.returncode == 66, r.stderr


def test_sitl_airsim_copter_starts_like_the_project_airsim_rail():
    """demo/pas_ardupilot/rail.py builds the native Project AirSim launch. With
    the scene's home and the CityLife airframe file, the container's command
    must carry the same model, home, defaults (in order) and simulator ports."""
    if not (find_bash() and CITYLIFE_PARAM.is_file()):
        return SKIP
    sys.path.insert(0, str(ROOT / "demo" / "pas_ardupilot"))
    import rail
    scene = rail.load_jsonc(rail.SCENE_TEMPLATE)
    home = rail.sitl_home(scene)
    want = options(rail.sitl_args(scene, types.SimpleNamespace(sitl_sim_address="10.0.0.8"),
                                  [CITYLIFE_PARAM.as_posix()], binary="arducopter",
                                  param_dir="/opt/ardupilot/params"))
    r = _sitl({"SITL_MODEL": "airsim-copter", "AIRSIM_HOST": "10.0.0.8", "SITL_HOME": home,
               "SITL_AIRFRAME_PARM": CITYLIFE_PARAM.as_posix()})
    assert r.returncode == 0, r.stderr
    got = options(printed(r))
    for flag in ("--model", "--speedup", "--home", "--defaults", "--sim-address",
                 "--sim-port-in", "--sim-port-out"):
        assert got[flag] == want[flag], (flag, got.get(flag), want[flag])
    assert got["--home"] != CMAC_HOME


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs a POSIX bash, git or the Docker CLI)")
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
