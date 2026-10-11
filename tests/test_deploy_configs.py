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

JETPACK 6.2 (the Orin is being set up with L4T r36.4.x, Ubuntu 22.04):

  * the companion image is ROS 2 Jazzy on an Ubuntu 24.04 (noble) base and
    REFUSES to build when rclpy would run below the grant's Python 3.11 (the
    floor check is executed here on 3.10 and 3.11 interpreters); its
    containers on the Orin run with runc, so no L4T GPU library enters the
    safety path;
  * the Orin's VLA base is a published r36.4 image carrying the desktop's
    torch / torchvision pair, the fallback is named the same everywhere, and
    the runbook builds exactly what hil.env names;
  * the VLA install keeps the base image's Jetson torch, runs pip against PyPI
    only (--isolated, PIP_CONFIG_FILE=/dev/null so no config file of the base
    applies either, and a stop if pip still reports a setting), and on
    aarch64 builds bitsandbytes at the pinned version for sm_87 and refuses a
    library without sm_87 code (PyPI's aarch64 wheels have none: they import,
    then the first 4-bit kernel fails; 10451bc's Dockerfile installed that
    wheel). The build is CMake configure and compile with
    COMPUTE_CAPABILITY=87, then pip, in that order: the source's
    pyproject.toml sets wheel.cmake = false, so pip alone compiles nothing.
    The plan is read from a dry run with a stand-in interpreter, so no torch
    is needed on the host running the tests;
  * deploy/orin/first_boot_check.sh reports what the person on the Orin sends
    back and says FAIL for what would break the deployment (not an Orin, not
    JetPack 6, no nvidia runtime, no buildx, no daemon) and WARN for what to
    know (JetPack 6.1 or 6.2.2+, another Ubuntu, CUDA / cuDNN / TensorRT not
    JetPack 6.2's, too little disk, Docker 28 on r36.4.3 without the raw
    table, ROS on the host, with its rclpy version); each of these verdicts is
    driven to its failing side at least once. Its power verdict reads the
    module's own mode table: the IDs, the default and the MAXN mode differ
    between an AGX Orin, an Orin NX and an Orin Nano, with or without JetPack
    6.2's Super flash. The runbook and the design doc assign the timing modes
    in one "Which modes:" list each, and only its "Not MAXN" item names MAXN;
  * deploy/orin/gpu_check.py refuses a GPU stack that only imports: a CPU
    torch, no GPU, a result not computed on a CUDA device, a 4-bit kernel
    that fails, an NF4 round trip that did not quantize (error 0) or returned
    nothing (error about 1). Its collect() runs here against stand-in torch
    and bitsandbytes modules whose GPU path carries known errors.

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
VLA_INSTALL = DOCKER_DIR / "vla-install.sh"
ORIN_DIR = DEPLOY / "orin"
FIRST_BOOT = ORIN_DIR / "first_boot_check.sh"
GPU_CHECK = ORIN_DIR / "gpu_check.py"
RUNBOOK = ROOT / "docs" / "RUNBOOK-orin-hil.md"
DESIGN_TOPOLOGIES = ROOT / "docs" / "DESIGN-topologies.md"
# Container images for L4T r36.4 (JetPack 6.2), each checked on Docker Hub on
# 2026-10-08 (docs/DESIGN-topologies.md, "JetPack 6.2"). The Orin's VLA base
# must be one of these; the first is the choice, the second the fallback.
ORIN_VLA_BASE = "dustynv/torchvision:0.21.0-r36.4.0-cu128-24.04"
ORIN_VLA_FALLBACK = "dustynv/l4t-pytorch:r36.4.0"
COMPANION_BASE = "ros:jazzy-ros-base-noble"
# Further interpreters that run the companion image's Python-floor check, so
# that one below the grant's 3.11 and one at it both run: VLA_FLOOR_PYTHONS
# (paths joined with os.pathsep), else the conda environments next to this
# interpreter's (see _floor_pythons). Absent ones are skipped.
FLOOR_PYTHONS = tuple(p for p in os.environ.get("VLA_FLOOR_PYTHONS", "").split(os.pathsep) if p)


def _floor_pythons() -> list[str]:
    """FLOOR_PYTHONS if set; otherwise, among the sibling conda environments
    of this interpreter, the first whose Python is on the other side of 3.11
    from this one. Empty when there is none (the test then skips visibly)."""
    if FLOOR_PYTHONS:
        return [p for p in FLOOR_PYTHONS if Path(p).is_file()]
    envs = Path(sys.prefix).parent
    if envs.name != "envs":
        return []
    exe = "python.exe" if os.name == "nt" else "bin/python"
    here = sys.version_info >= (3, 11)
    for cand in sorted(envs.glob("*/" + exe)):
        if cand.resolve() == Path(sys.executable).resolve():
            continue
        try:
            out = subprocess.run([str(cand), "-c", "import sys; print(sys.version_info >= (3, 11))"],
                                 capture_output=True, text=True, timeout=30).stdout.strip()
        except (OSError, subprocess.TimeoutExpired):
            continue
        if out in ("True", "False") and (out == "True") != here:
            return [str(cand)]
    return []
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
    """Every role of every `case "$role" in` block (`versions` is answered
    before the topology check, the others after it)."""
    text = ENTRYPOINT.read_text(encoding="utf-8")
    body = text.split('case "$role" in', 1)[-1]
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


def run_with_fakes(script: Path, args: list[str], env: dict, fakebin: Path,
                   timeout: int = 120) -> subprocess.CompletedProcess:
    """Run `script` with `fakebin` FIRST on PATH. Git Bash puts its own
    /usr/bin ahead of an inherited PATH, which would shadow stand-ins for
    uname or df, so the directory is prepended inside bash itself."""
    wrapper = ('d="$(cygpath -u "$1" 2>/dev/null || printf %s "$1")"; '
               'PATH="$d:$PATH"; shift; exec bash "$@"')
    return subprocess.run([find_bash(), "-c", wrapper, "_", str(fakebin), script.as_posix(),
                           *args], env=env, capture_output=True, text=True, timeout=timeout)


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
    assert re.search(r"-r36\.4\.\d+", hil["VLA_BASE_IMAGE"]) or \
        hil["VLA_BASE_IMAGE"].endswith(":r36.4.0"), \
        f"the Orin VLA image must be built on an L4T r36.4 (JetPack 6.2) base: {hil['VLA_BASE_IMAGE']}"
    assert "cuda" in dev["VLA_BASE_IMAGE"] and "r36" not in dev["VLA_BASE_IMAGE"], \
        "the desktop's VLA base is a CUDA build for amd64"


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
                     "versions", "shell"}, roles
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
    assert m and m.group(1) == COMPANION_BASE, \
        f"base is {m and m.group(1)}, not ROS 2 Jazzy on Ubuntu 24.04 ({COMPANION_BASE})"
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
    # The distribution is the base image's own ROS_DISTRO: a second argument
    # could name another distribution than the base installs.
    args = set(re.findall(r"^ARG (\w+)", text, flags=re.M))
    assert args <= {"BASE_IMAGE", "MAVLINK_ROUTER_REF"}, args
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
                ".Dockerfile", ".py"}
    checked = 0
    for p in DEPLOY.rglob("*"):
        if not p.is_file() or (p.suffix not in text_ext and p.name != ".gitignore"):
            continue
        data = p.read_bytes()
        assert b"\r\n" not in data, f"{p.relative_to(ROOT)} has CRLF line endings"
        checked += 1
    assert checked >= 18, checked
    for sh in (ENTRYPOINT, SITL_ENTRYPOINT, START_ROUTER, VLA_INSTALL, FIRST_BOOT):
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


# --------------------------------------------------------------------------- #
# JetPack 6.2 on the Orin
# --------------------------------------------------------------------------- #
def _floor_check_code() -> str:
    """The Python-floor one-liner the companion image runs at build time."""
    text = (DOCKER_DIR / "companion.Dockerfile").read_text(encoding="utf-8")
    m = re.search(r"/opt/venv/bin/python -c '([^']+)'", text)
    assert m, "the companion image runs no Python-floor check on the Shield's interpreter"
    return m.group(1)


def test_the_companion_image_refuses_rclpy_below_the_grants_python_floor():
    """Safety Shield p.1: "Python 3.11+ as a ROS 2 node (rclpy)". On JetPack
    6.2 the native ROS 2 is Humble, whose rclpy runs on Python 3.10; the image
    keeps Jazzy (3.12) and its build must STOP on a base below 3.11. The
    check's own code is executed here, with a stand-in rclpy, on a 3.10 and a
    3.11 interpreter: it must refuse the first with the reason and pass the
    second."""
    text = (DOCKER_DIR / "companion.Dockerfile").read_text(encoding="utf-8")
    venv = text.index("python3 -m venv --system-site-packages /opt/venv")
    floor = text.index("/opt/venv/bin/python -c '")
    assert venv < floor, "the floor is checked before the Shield's interpreter exists"
    assert 'source "/opt/ros/${ROS_DISTRO}/setup.bash"' in text[floor - 200:floor], \
        "rclpy is not on the path when the floor is checked"
    assert "VLAGUARD_PYTHON=/opt/venv/bin/python" in text
    code = _floor_check_code()
    stub = Path(tempfile.mkdtemp())
    (stub / "rclpy").mkdir()
    (stub / "rclpy" / "__init__.py").write_text("", encoding="utf-8")
    pythons = [sys.executable] + _floor_pythons()
    seen = set()
    for py in pythons:
        r = subprocess.run([py, "-c", code], capture_output=True, text=True, timeout=60,
                           env={**os.environ, "PYTHONPATH": str(stub)})
        v = subprocess.run([py, "-c", "import sys; print(sys.version_info >= (3, 11))"],
                           capture_output=True, text=True, timeout=60).stdout.strip()
        seen.add(v)
        if v == "True":
            assert r.returncode == 0, (py, r.stderr)
        else:
            assert r.returncode != 0 and "below the grant floor 3.11" in r.stderr, (py, r)
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                       timeout=60, env={**os.environ, "PYTHONPATH": ""})
    assert r.returncode != 0, "the floor check passed without rclpy importable"
    if seen != {"True", "False"}:
        return SKIP          # only one side of the floor could be executed here


def test_companion_containers_on_the_orin_take_no_gpu_libraries():
    """jetson-containers' set-up makes nvidia the daemon's default runtime;
    on JetPack 6 that runtime mounts L4T's GPU libraries into every container.
    The safety path names runc; only the VLA bench names nvidia."""
    doc = load_compose(COMPOSE_DIR / "docker-compose.orin.yml")
    for name, svc in doc["services"].items():
        want = "nvidia" if name == "vla-bench" else "runc"
        assert svc.get("runtime") == want, f"orin/{name}: runtime {svc.get('runtime')!r}, want {want}"
    for fname in ("docker-compose.dev.yml", "docker-compose.hil-desktop.yml"):
        for name, svc in load_compose(COMPOSE_DIR / fname)["services"].items():
            assert svc.get("runtime") != "nvidia", f"{fname}/{name}: the desktop uses deploy.resources"


def test_the_orin_vla_base_is_a_published_r36_4_image_with_the_desktops_torch():
    hil, flight = read_env("hil"), read_env("flight")
    assert hil["VLA_BASE_IMAGE"] == flight["VLA_BASE_IMAGE"] == ORIN_VLA_BASE, \
        (hil["VLA_BASE_IMAGE"], flight["VLA_BASE_IMAGE"])
    assert ORIN_VLA_BASE.endswith("-24.04"), "the Orin's guardrail code would run below Python 3.11"
    assert "r36.4" in hil["VLAGUARD_VLA_IMAGE"], hil["VLAGUARD_VLA_IMAGE"]
    # The base's torchvision is the desktop's (vla-real) - the pair torch 2.6 /
    # torchvision 0.21 - so the R1 table compares one framework version.
    tv = re.search(r"torchvision:(\d+\.\d+\.\d+)", ORIN_VLA_BASE).group(1)
    assert f"torchvision {tv}" in _design_row("vla-real"), tv
    # The fallback, the Dockerfile's Orin build and the runbook's all agree.
    vla_df = (DOCKER_DIR / "vla.Dockerfile").read_text(encoding="utf-8")
    hil_text = (TOPO_DIR / "hil.env").read_text(encoding="utf-8")
    runbook = RUNBOOK.read_text(encoding="utf-8")
    design = DESIGN_TOPOLOGIES.read_text(encoding="utf-8")
    gpu = GPU_CHECK.read_text(encoding="utf-8")
    for name, text in (("vla.Dockerfile", vla_df), ("hil.env", hil_text),
                       ("RUNBOOK-orin-hil.md", runbook), ("DESIGN-topologies.md", design),
                       ("gpu_check.py", gpu)):
        assert ORIN_VLA_FALLBACK in text, f"{name} does not name the fallback {ORIN_VLA_FALLBACK}"
    build = (f"--build-arg BASE_IMAGE={hil['VLA_BASE_IMAGE']} "
             f"-t {hil['VLAGUARD_VLA_IMAGE']} .")
    for name, text in (("vla.Dockerfile", vla_df), ("RUNBOOK-orin-hil.md", runbook)):
        flat = re.sub(r"\s*\\\n\s*#?\s*", " ", text)
        assert build in flat, f"{name}: the Orin build is not `{build}`"
    for name, text in (("RUNBOOK-orin-hil.md", runbook), ("vla.Dockerfile", vla_df)):
        assert not re.search(r"vlaguard/vla:l4t-r36(?![.\w-])", text), \
            f"{name} still names the old tag vlaguard/vla:l4t-r36"
    for tag in (ORIN_VLA_BASE, ORIN_VLA_FALLBACK, COMPANION_BASE):
        assert tag in design, f"DESIGN-topologies.md does not list {tag} among the checked tags"
    # The fallback carries torch 2.4.0 / torchvision 0.19.0 (its image history
    # on Docker Hub), not the desktop's pair: whoever switches to it must be
    # told the R1 columns then differ in framework version.
    for name, text in (("RUNBOOK-orin-hil.md", runbook), ("DESIGN-topologies.md", design),
                       ("vla.Dockerfile", vla_df)):
        assert "torch 2.4.0" in text and "0.19.0" in text, \
            f"{name} does not say the fallback base runs torch 2.4.0 / torchvision 0.19.0"


def _req_dir() -> Path:
    d = Path(tempfile.mkdtemp())
    shutil.copy(ROOT / "requirements.txt", d / "requirements.txt")
    shutil.copy(DOCKER_DIR / "requirements-vla.txt", d / "requirements-vla.txt")
    return d


def _vla_install(args: list[str], env: dict) -> subprocess.CompletedProcess:
    full = _base_env(env)
    return subprocess.run([find_bash(), VLA_INSTALL.as_posix(), *args], env=full,
                          capture_output=True, text=True, timeout=120)


def _plans(r: subprocess.CompletedProcess) -> list[list[str]]:
    return [shlex.split(l[len("PLAN:"):]) for l in r.stdout.splitlines() if l.startswith("PLAN:")]


def _base_python(freeze: str = "torch==2.6.0\ntorchvision==0.21.0") -> str:
    """A stand-in for the base image's interpreter in a dry run: it answers
    the install's torch freeze (`python -`, a script on stdin) like the
    chosen Orin base, and nothing else. The plan then needs no torch on this
    host (the dry run executes only the freeze)."""
    py = Path(tempfile.mkdtemp()) / "base-python"
    py.write_bytes(('#!/usr/bin/env bash\n'
                    'if [ "$1" = "-" ]; then cat > /dev/null; printf "%s\\n" '
                    + " ".join(shlex.quote(l) for l in freeze.splitlines())
                    + '; exit 0; fi\nexit 1\n').encode())
    os.chmod(py, 0o755)
    return py.as_posix()


def test_the_vla_install_keeps_the_base_torch_and_builds_bitsandbytes_for_sm87():
    if not find_bash():
        return SKIP
    df = (DOCKER_DIR / "vla.Dockerfile").read_text(encoding="utf-8")
    assert "RUN bash /tmp/req/vla-install.sh /tmp/req" in df
    bnb = _pins(DOCKER_DIR / "requirements-vla.txt")["bitsandbytes"]
    py = _base_python()
    # aarch64: the Orin.
    d = _req_dir()
    r = _vla_install([d.as_posix()], {"VLA_ARCH": "aarch64", "VLA_PYTHON": py})
    assert r.returncode == 0, r.stderr
    frozen = (d / "base-torch.txt").read_text(encoding="utf-8").split()
    assert frozen == ["torch==2.6.0", "torchvision==0.21.0"], frozen
    plans = _plans(r)
    pips = [p for p in plans if p[1:4] == ["-m", "pip", "--isolated"]
            or (p[0] == "env" and "--isolated" in p)]
    assert len(pips) == 2, plans
    for p in pips:
        assert p[p.index("--index-url") + 1] == "https://pypi.org/simple", p
        assert p[p.index("-c") + 1].endswith("/base-torch.txt"), p
        # --isolated drops PIP_* variables and the per-user file, but pip
        # still reads /etc/pip.conf and a virtual environment's pip.conf;
        # PIP_CONFIG_FILE set to the null device makes it skip every file.
        assert p[:6] == ["env", "PIP_CONFIG_FILE=/dev/null", py, "-m", "pip", "--isolated"], p
    assert ["check_pip_config"] in plans and \
        plans.index(["check_pip_config"]) < plans.index(pips[0]), \
        ("nothing shows, before the first install, that pip reads no setting", plans)
    reqs = [pips[0][i + 1] for i, a in enumerate(pips[0]) if a == "-r"]
    assert [Path(x).name for x in reqs] == ["requirements.txt", "requirements-vla-jetson.txt"], reqs
    jetson = _pins(d / "requirements-vla-jetson.txt")
    assert "bitsandbytes" not in jetson and \
        jetson == {k: v for k, v in _pins(DOCKER_DIR / "requirements-vla.txt").items()
                   if k != "bitsandbytes"}, jetson
    clone = next(p for p in plans if p[:2] == ["git", "clone"])
    assert clone[clone.index("--branch") + 1] == bnb, (clone, bnb)
    src = clone[-1]
    # bitsandbytes' pyproject.toml (tag 0.49.2) sets `wheel.cmake = false` under
    # [tool.scikit-build]: pip does NOT run CMake, so a CMAKE_ARGS handed to pip
    # builds no CUDA library at all. Its install guide compiles first
    # (`cmake -DCOMPUTE_BACKEND=cuda -S .`, `make`) and then runs `pip install .`,
    # which packages the library CMake wrote into the source tree. The plan
    # must do the same, with the Orin's capability, in that order.
    configure = next((p for p in plans if p[:1] == ["cmake"] and "-S" in p), None)
    assert configure, f"no CMake configure of the bitsandbytes source: {plans}"
    assert configure[configure.index("-S") + 1] == src, (configure, src)
    for flag in ("-DCOMPUTE_BACKEND=cuda", "-DCOMPUTE_CAPABILITY=87"):
        assert flag in configure, (flag, configure)
    assert any(a.startswith("-DCMAKE_CUDA_COMPILER=") for a in configure), configure
    compile_ = next((p for p in plans if p[:2] == ["cmake", "--build"]), None)
    assert compile_, f"the configured library is never compiled: {plans}"
    build = pips[1]
    assert build[-1] == src, ("pip installs something other than the built source", build)
    assert not any(a.startswith("CMAKE_ARGS=") for a in build), \
        ("CMAKE_ARGS reach pip, which runs no CMake for bitsandbytes", build)
    assert "--no-deps" in build, "the source build may not pull a second torch"
    assert ["check_sm", "87"] in plans, "nothing checks the library for sm_87 code"
    order = [plans.index(p) for p in (clone, configure, compile_, build, ["check_sm", "87"])]
    assert order == sorted(order), ("clone, configure, compile, pip, check: out of order", order)
    # The source's floor is cmake_minimum_required(VERSION 3.22.1). Both Orin
    # bases put a pip-installed CMake ahead of the apt one on PATH (their
    # layer history); apt installs one in case a base has none. Which of them
    # ran is recorded in the build log just before the configure step.
    apt = next(p for p in plans if p[:2] == ["apt-get", "install"])
    assert "cmake" in apt and plans.index(apt) < plans.index(configure), apt
    assert ["cmake", "--version"] in plans, "the build log would not show which CMake ran"
    assert plans.index(apt) < plans.index(["cmake", "--version"]) < plans.index(configure), plans
    # x86_64: the desktop takes PyPI's wheel, which carries its GPUs' code.
    d = _req_dir()
    r = _vla_install([d.as_posix()], {"VLA_ARCH": "x86_64", "VLA_PYTHON": py})
    assert r.returncode == 0, r.stderr
    plans = _plans(r)
    assert plans[0] == ["check_pip_config"], plans
    assert len(plans) == 2 and Path(plans[1][-1]).name == "requirements-vla.txt", plans
    assert plans[1][:2] == ["env", "PIP_CONFIG_FILE=/dev/null"], plans[1]
    # Refusals.
    assert _vla_install([_req_dir().as_posix()],
                        {"VLA_ARCH": "riscv64", "VLA_PYTHON": py}).returncode == 64
    d = _req_dir()
    (d / "requirements-vla.txt").write_text("transformers==4.40.1\n", encoding="utf-8")
    r = _vla_install([d.as_posix()], {"VLA_ARCH": "aarch64", "VLA_PYTHON": py})
    assert r.returncode == 64 and "bitsandbytes" in r.stderr, r.stderr
    fake = Path(tempfile.mkdtemp()) / "python-without-torch"
    fake.write_bytes(b"#!/usr/bin/env bash\ncat > /dev/null\n")
    os.chmod(fake, 0o755)
    r = _vla_install([_req_dir().as_posix()], {"VLA_ARCH": "aarch64", "VLA_PYTHON": fake.as_posix()})
    assert r.returncode == 64 and "no torch" in r.stderr, r.stderr


def test_the_vla_install_stops_when_pip_replaced_the_base_torch():
    """Run for real (not dry) with a stand-in interpreter whose pip 'installs'
    and whose torch version reads differently afterwards, as when PyPI's CPU
    wheel replaced the Jetson build. The install must stop with exit 65; with
    the version unchanged it must finish."""
    if not find_bash():
        return SKIP
    for after, want_rc in (("2.6.0", 0), ("2.6.0+cpu", 65)):
        tmp = Path(tempfile.mkdtemp())
        py = tmp / "fake-python"
        py.write_bytes(
            ("#!/usr/bin/env bash\n"
             f'state="{tmp.as_posix()}/calls"\n'
             'if [ "$1" = "-" ]; then\n'
             '  cat > /dev/null\n'
             '  if [ -f "$state" ]; then echo "torch==' + after + '"; else echo "torch==2.6.0"; fi\n'
             '  echo x >> "$state"; exit 0\n'
             "fi\n"
             'if [ "$1" = "-m" ] && [ "$2" = "pip" ]; then echo "PIP $*" >&2; exit 0; fi\n'
             "exit 1\n").encode())
        os.chmod(py, 0o755)
        d = _req_dir()
        env = _base_env({"VLA_ARCH": "x86_64", "VLA_PYTHON": py.as_posix()}, dry=False)
        r = subprocess.run([find_bash(), VLA_INSTALL.as_posix(), d.as_posix()], env=env,
                           capture_output=True, text=True, timeout=60)
        assert "PIP -m pip --isolated install" in r.stderr, r.stderr
        assert r.returncode == want_rc, (after, r.returncode, r.stderr)
        if want_rc:
            assert "changed the base image's torch stack" in r.stderr, r.stderr


def test_the_vla_install_reads_no_pip_config_from_the_base_image():
    """Both Orin bases set PIP_INDEX_URL, and a base may carry a pip.conf
    (/etc/pip.conf, or one inside its virtual environment). --isolated drops
    the variable but still reads those files (pip's own configuration code;
    shown on this desktop with a venv whose pip.ini named an index:
    `pip --isolated config list` printed it, and printed nothing once
    PIP_CONFIG_FILE was the null device). Run for real with a stand-in pip
    that behaves that way: a base with a config file must install with no
    setting left; a pip that still reports one stops the build with exit
    65; and every install call carries the null config file."""
    if not find_bash():
        return SKIP
    for honours_null, want_rc in ((True, 0), (False, 65)):
        tmp = Path(tempfile.mkdtemp())
        py = tmp / "fake-python"
        null_check = 'if [ "$PIP_CONFIG_FILE" = /dev/null ]; then exit 0; fi\n' if honours_null else ""
        py.write_bytes(
            ("#!/usr/bin/env bash\n"
             'if [ "$1" = "-" ]; then cat > /dev/null; echo "torch==2.6.0"; exit 0; fi\n'
             'if [ "$*" = "-m pip config debug" ]; then\n'
             '  echo "global:"; echo "  /etc/pip.conf, exists: True"; exit 0\n'
             "fi\n"
             'if [ "$*" = "-m pip --isolated config list" ]; then\n'
             + null_check +
             "  echo \"global.index-url='https://pypi.jetson-ai-lab.dev/jp6/cu128'\"; exit 0\n"
             "fi\n"
             'if [ "$1" = "-m" ] && [ "$2" = "pip" ]; then\n'
             '  echo "PIP cfg=${PIP_CONFIG_FILE:-unset} $*" >&2; exit 0\n'
             "fi\n"
             "exit 1\n").encode())
        os.chmod(py, 0o755)
        env = _base_env({"VLA_ARCH": "x86_64", "VLA_PYTHON": py.as_posix()}, dry=False)
        r = subprocess.run([find_bash(), VLA_INSTALL.as_posix(), _req_dir().as_posix()], env=env,
                           capture_output=True, text=True, timeout=60)
        assert r.returncode == want_rc, (honours_null, r.returncode, r.stderr)
        assert "/etc/pip.conf, exists: True" in r.stderr, "the build log does not show pip's config files"
        installs = [l for l in r.stderr.splitlines() if l.startswith("PIP ") and " install " in l]
        if want_rc:
            assert "pip still reads settings" in r.stderr and not installs, r.stderr
        else:
            assert installs and all(l.startswith("PIP cfg=/dev/null -m pip --isolated install")
                                    for l in installs), installs


def test_the_sm87_check_refuses_a_bitsandbytes_without_orin_code():
    """PyPI's aarch64 bitsandbytes carries no sm_87 code (issue #1930):
    it imports on an Orin and fails at the first 4-bit kernel. The build's
    check reads the library's cubins and must say no to exactly that."""
    if not find_bash():
        return SKIP
    tmp = Path(tempfile.mkdtemp())
    pkg = tmp / "site" / "bitsandbytes"
    pkg.mkdir(parents=True)
    (pkg / "libbitsandbytes_cuda128.so").write_bytes(b"\x7fELF")
    fakebin = tmp / "bin"
    fakebin.mkdir()
    py = fakebin / "python3"
    py.write_bytes(f'#!/usr/bin/env bash\necho "{pkg.as_posix()}"\n'.encode())

    def run(archs: str, with_cuobjdump: bool = True) -> subprocess.CompletedProcess:
        cu = fakebin / "cuobjdump"
        if with_cuobjdump:
            lines = "".join(f"ELF file    {i + 1}: libbitsandbytes_cuda128.{i + 1}.{a}.cubin\\n"
                            for i, a in enumerate(archs.split()))
            cu.write_bytes(f'#!/usr/bin/env bash\nprintf "{lines}"\n'.encode())
        elif cu.exists():
            cu.unlink()
        for f in fakebin.iterdir():
            os.chmod(f, 0o755)
        env = _base_env({"VLA_PYTHON": py.as_posix()}, dry=False)
        return run_with_fakes(VLA_INSTALL, ["--check-sm", "87"], env, fakebin, timeout=60)

    r = run("sm_75 sm_80 sm_90")                       # PyPI's aarch64 wheel
    assert r.returncode == 65 and "no sm_87" in r.stderr, (r.returncode, r.stderr)
    r = run("sm_87")                                   # built for the Orin
    assert r.returncode == 0, r.stderr
    if not shutil.which("cuobjdump"):
        r = run("sm_87", with_cuobjdump=False)
        assert r.returncode == 65 and "cuobjdump not found" in r.stderr, r.stderr
    (pkg / "libbitsandbytes_cuda128.so").unlink()
    r = run("sm_87")
    assert r.returncode == 65 and "no libbitsandbytes_cuda" in r.stderr, r.stderr


def test_entrypoint_versions_role_needs_no_topology_and_reports_every_key():
    """The record the runbook asks for after the build: the ROS 2
    distribution and the Python rclpy runs on, among others. It must answer
    without a topology file or a checkout, and never print an empty value."""
    if not find_bash():
        return SKIP
    env = _base_env({}, dry=True)
    env.pop("VLAGUARD_TOPOLOGY", None)
    env["VLAGUARD_ROOT"] = Path(tempfile.mkdtemp()).as_posix()      # no checkout there
    r = subprocess.run([find_bash(), ENTRYPOINT.as_posix(), "versions"], env=env,
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 0 and printed(r) == ["print_versions"], (r.returncode, r.stdout, r.stderr)
    env.pop("VLAGUARD_DRY_RUN")
    env["VLAGUARD_PYTHON"] = Path(sys.executable).as_posix()
    r = subprocess.run([find_bash(), ENTRYPOINT.as_posix(), "versions"], env=env,
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stderr
    kv = dict(l.split("=", 1) for l in r.stdout.splitlines() if "=" in l)
    for key in ("image_os", "ros_distro", "ros_setup", "python[python3]", "mavlink-routerd",
                "pip[pydantic]", "pip[shapely]", "deb[ros-none-mavros]", "deb[ros-none-rclpy]"):
        assert key in kv, (key, sorted(kv))
    assert all(v.strip() for v in kv.values()), kv
    assert kv["ros_distro"] == "none" and kv["deb[ros-none-mavros]"] == "none", kv
    assert kv[f"python[{Path(sys.executable).as_posix()}]"].startswith(
        "%d.%d." % sys.version_info[:2]), kv
    assert kv["pip[pydantic]"] != "none", "the guardrail's own packages were not read"


# --- deploy/orin/first_boot_check.sh ---------------------------------------- #
_JP62_RELEASE = ("# R36 (release), REVISION: 4.3, GCID: 38968081, BOARD: generic, "
                 "EABI: aarch64, DATE: Wed Jan  8 01:49:37 UTC 2025")
_JP62_PKGS = {"nvidia-jetpack": "6.2+b77", "nvidia-l4t-core": "36.4.3-20250107174145",
              "cuda-cudart-12-6": "12.6.68-1", "libcudnn9-cuda-12": "9.3.0.75-1",
              "tensorrt": "10.3.0.30-1+cuda12.5", "nvidia-container-toolkit": "1.16.2-1"}


# /etc/nvpmodel.conf's mode table, as Jetson Linux 36.4.3's power guide lists
# the modes per module (IDs differ between modules; JetPack 6.2's Super modes
# exist only on an Orin NX / Nano flashed with a -super configuration).
_NVPMODEL_AGX_64GB = ("< POWER_MODEL ID=0 NAME=MAXN >\n"
                      "< POWER_MODEL ID=1 NAME=MODE_15W >\n"
                      "< POWER_MODEL ID=2 NAME=MODE_30W >\n"
                      "< POWER_MODEL ID=3 NAME=MODE_50W >\n"
                      "< PM_CONFIG DEFAULT=2 >\n")


def _fake_orin(release=_JP62_RELEASE, model="NVIDIA Jetson AGX Orin Developer Kit",
               pkgs=None, docker="27.5.1", runtimes="io.containerd.runc.v2 nvidia runc",
               buildx=True, ros=(), raw_builtin=False, daemon=True, kernel="5.15.148-tegra",
               nvpmodel_conf=_NVPMODEL_AGX_64GB, os_version="22.04",
               df_avail_kb=900000000, ip_tables_names=None):
    """A JetPack 6.2 Orin as files under a sysroot plus stand-in commands."""
    root = Path(tempfile.mkdtemp())
    for rel, text in (("etc/nv_tegra_release", release),
                      ("proc/device-tree/model", (model + "\x00") if model else None),
                      ("etc/os-release", f'NAME="Ubuntu"\nID=ubuntu\nVERSION_ID="{os_version}"\n'),
                      ("etc/nvpmodel.conf", nvpmodel_conf),
                      ("proc/net/ip_tables_names", ip_tables_names),
                      ("sys/devices/system/cpu/online", "0-11\n")):
        if text is None:
            continue
        p = root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(text.encode())
    for distro in ros:
        (root / "opt" / "ros" / distro).mkdir(parents=True)
    if raw_builtin:
        p = root / "lib" / "modules" / kernel / "modules.builtin"
        p.parent.mkdir(parents=True)
        p.write_text("kernel/net/ipv4/netfilter/iptable_raw.ko\n", encoding="utf-8")
    pkgs = dict(_JP62_PKGS if pkgs is None else pkgs)
    bin_ = root / "fakebin"
    bin_.mkdir()
    scripts = {
        "uname": f'case "$1" in -m) echo aarch64;; -r) echo {kernel};; '
                 f'*) echo "Linux {kernel} #1 SMP PREEMPT aarch64 GNU/Linux";; esac',
        "nvpmodel": 'echo "NV Power Mode: MAXN"; echo 0',
        "df": 'echo "Filesystem 1024-blocks Used Available Capacity Mounted on"; '
              f'echo "/dev/nvme0n1p1 960000000 60000000 {df_avail_kb} 7% /"',
        "dpkg-query": "case \"$*\" in\n" + "".join(
            f'  *"{k}"*) [[ "$*" == *Version* ]] && printf %s "{v}" || echo "{k} {v}";;\n'
            for k, v in pkgs.items()) + "  *) exit 1;;\nesac",
        "nvidia-ctk": 'echo "NVIDIA Container Toolkit CLI version 1.16.2"',
        # This host's kernel modules must not answer for the fake Orin's.
        "modinfo": "exit 1",
    }
    if docker:
        fmt = {"ServerVersion": docker if daemon else "", "Runtimes": runtimes + " ",
               "DefaultRuntime": "runc", "DockerRootDir": "/var/lib/docker"}
        info = "".join(f'  *{k}*) [ -n "{v}" ] && echo "{v}" || exit 1;;\n' for k, v in fmt.items())
        scripts["docker"] = (
            'if [ "$1" = info ] && [ "$2" = --format ]; then case "$3" in\n' + info +
            "  esac; exit 0; fi\n"
            + ('if [ "$1" = info ]; then echo "Server Version: ' + docker + '"; exit 0; fi\n'
               if daemon else 'if [ "$1" = info ]; then echo "permission denied" >&2; exit 1; fi\n')
            + 'if [ "$1" = compose ]; then echo 2.29.7; exit 0; fi\n'
            + ('if [ "$1" = buildx ]; then echo "github.com/docker/buildx v0.17.1"; exit 0; fi\n'
               if buildx else 'if [ "$1" = buildx ]; then echo "unknown command" >&2; exit 1; fi\n')
            + "exit 1")
    for name, body in scripts.items():
        f = bin_ / name
        f.write_bytes(("#!/usr/bin/env bash\n" + body + "\n").encode())
        os.chmod(f, 0o755)
    return root, bin_


def _first_boot(root: Path, bin_: Path) -> tuple[subprocess.CompletedProcess, dict]:
    env = {k: v for k, v in os.environ.items() if k not in ("ROS_DISTRO",)}
    env["VLAGUARD_SYSROOT"] = root.as_posix()
    r = run_with_fakes(FIRST_BOOT, [], env, bin_)
    verdicts = {}
    for line in r.stdout.split("===== VERDICTS =====", 1)[-1].splitlines():
        m = re.match(r"(PASS|WARN|FAIL)\s+(\w+): (.*)", line)
        if m:
            verdicts[m.group(2)] = (m.group(1), m.group(3))
    return r, verdicts


def test_first_boot_check_passes_a_jetpack_62_orin_and_shows_what_to_send_back():
    if not find_bash():
        return SKIP
    r, v = _first_boot(*_fake_orin())
    assert r.returncode == 0, (r.stdout[-3000:], r.stderr)
    want = {"l4t", "board", "arch", "os", "jetpack", "compute", "power", "ros_host",
            "docker", "nvidia_runtime", "compose", "buildx", "disk"}
    assert want <= set(v), sorted(v)
    bad = {k: x for k, x in v.items() if x[0] != "PASS"}
    assert not bad, bad
    assert "R36.4.3 = JetPack 6.2" in v["l4t"][1], v["l4t"]
    assert "CUDA 12.6" in v["compute"][1] and "TensorRT 10.3" in v["compute"][1], v["compute"]
    # The power verdict names this module's mode table, its default and its
    # MAXN mode by ID: runbook step 4 sets the timing modes by these IDs.
    for part in ("0=MAXN", "1=MODE_15W", "3=MODE_50W", "default ID 2 = MODE_30W",
                 "MAXN mode ID 0 = MAXN", "highest budgeted mode ID 3 = MODE_50W"):
        assert part in v["power"][1], (part, v["power"])
    # The outputs the person is asked to send back are all in the report.
    for shown in ("$ cat /etc/nv_tegra_release", "REVISION: 4.3", "$ nvpmodel -q",
                  "NV Power Mode: MAXN", "POWER_MODEL ID=0 NAME=MAXN", "$ ls /opt/ros",
                  "$ docker info", "Server Version: 27.5.1", "$ docker buildx version"):
        assert shown in r.stdout, shown
    assert "0 FAIL, 0 WARN" in r.stdout


def test_first_boot_check_fails_what_would_break_the_deployment():
    if not find_bash():
        return SKIP
    cases = (
        ({"release": "# R35 (release), REVISION: 4.1, GCID: 1, BOARD: t186ref"}, "l4t"),
        ({"release": "# R36 (release), REVISION: 3.0, GCID: 1, BOARD: generic"}, "l4t"),
        ({"release": None}, "l4t"),
        ({"model": "NVIDIA Jetson Xavier NX Developer Kit"}, "board"),
        ({"runtimes": "io.containerd.runc.v2 runc"}, "nvidia_runtime"),
        ({"buildx": False}, "buildx"),
        ({"daemon": False}, "docker"),
    )
    for kw, key in cases:
        r, v = _first_boot(*_fake_orin(**kw))
        assert r.returncode == 1, (kw, r.stdout[-2000:])
        assert v.get(key, ("?",))[0] == "FAIL", (kw, v.get(key))


def test_first_boot_check_warns_what_the_setup_person_must_know():
    if not find_bash():
        return SKIP
    cases = (
        ({"release": "# R36 (release), REVISION: 4.0, GCID: 1, BOARD: generic"}, "l4t", "JetPack 6.1"),
        ({"release": "# R36 (release), REVISION: 5.0, GCID: 1, BOARD: generic"}, "l4t", "R36.5"),
        ({"ros": ("humble",)}, "ros_host", "humble"),
        ({"pkgs": {k: v for k, v in _JP62_PKGS.items() if k != "nvidia-jetpack"}},
         "jetpack", "not installed"),
        ({"pkgs": {**_JP62_PKGS, "nvidia-jetpack": "6.1+b123"}}, "jetpack", "6.1+b123, not 6.2"),
        ({"docker": "28.0.1"}, "docker28", "iptable_raw"),
        # A table list without "raw" does not count as the table being there.
        ({"docker": "28.0.1", "ip_tables_names": "filter\nnat\n"}, "docker28", "iptable_raw"),
        # The failing side of verdicts the passing Orin above only ever PASSes.
        ({"os_version": "24.04"}, "os", "'ubuntu-24.04'"),
        ({"pkgs": {**_JP62_PKGS, "tensorrt": "10.7.0.23-1+cuda12.6"}}, "compute",
         "TensorRT '10.7.0.23-1+cuda12.6'"),
        ({"pkgs": {**_JP62_PKGS, "libcudnn9-cuda-12": "9.6.0.74-1"}}, "compute",
         "cuDNN '9.6.0.74-1'"),
        ({"pkgs": {k: v for k, v in _JP62_PKGS.items() if k != "cuda-cudart-12-6"}}, "compute",
         "cudart-12-6 'none'"),
        ({"df_avail_kb": 10 * 1024 * 1024}, "disk", "10 GB free"),
        ({"df_avail_kb": 39 * 1024 * 1024}, "disk", "39 GB free"),
    )
    for kw, key, text in cases:
        r, v = _first_boot(*_fake_orin(**kw))
        assert r.returncode == 0, (kw, r.stdout[-2000:])
        assert v.get(key, ("?",))[0] == "WARN" and text in v[key][1], (kw, v.get(key))
    r, v = _first_boot(*_fake_orin(df_avail_kb=40 * 1024 * 1024))
    assert v["disk"][0] == "PASS" and "40 GB free" in v["disk"][1], v["disk"]
    # Docker 28 is fine where the kernel has the raw table (built in, or
    # loaded: /proc/net/ip_tables_names lists it), and on r36.4.4.
    for kw in ({"docker": "28.0.1", "raw_builtin": True},
               {"docker": "28.0.1", "ip_tables_names": "filter\nnat\nraw\n"},
               {"docker": "28.0.1", "release": "# R36 (release), REVISION: 4.4, GCID: 1"}):
        r, v = _first_boot(*_fake_orin(**kw))
        assert "docker28" not in v and r.returncode == 0, (kw, v.get("docker28"))
    r, v = _first_boot(*_fake_orin(release="# R36 (release), REVISION: 4.4, GCID: 1"))
    assert v["l4t"][0] == "PASS" and "6.2.1" in v["l4t"][1], v["l4t"]


def test_first_boot_check_reports_the_host_ros_version():
    """"ROS version" is one of the outputs the person on the Orin sends back.
    With no ROS on the host the verdict says so; with a distribution there,
    the report shows its rclpy package version and the verdict names it."""
    if not find_bash():
        return SKIP
    r, v = _first_boot(*_fake_orin())
    assert v["ros_host"][0] == "PASS" and "Jazzy runs inside the companion image" in v["ros_host"][1]
    rclpy = "3.3.16-1jammy.20250325.054128"
    r, v = _first_boot(*_fake_orin(ros=("humble",),
                                   pkgs={**_JP62_PKGS, "ros-humble-rclpy": rclpy}))
    assert "$ dpkg-query -W ros-humble-rclpy" in r.stdout, r.stdout[-3000:]
    assert f"ros-humble-rclpy {rclpy}" in r.stdout, r.stdout[-3000:]
    assert v["ros_host"][0] == "WARN" and f"rclpy {rclpy}" in v["ros_host"][1], v["ros_host"]
    # A directory under /opt/ros with no packages behind it is reported as such.
    r, v = _first_boot(*_fake_orin(ros=("jazzy",)))
    assert v["ros_host"][0] == "WARN" and "rclpy not installed" in v["ros_host"][1], v["ros_host"]


def test_first_boot_check_reads_this_modules_power_modes():
    """Mode IDs differ between Orin modules (Jetson Linux 36.4.3 power guide):
    an AGX Orin's MAXN is ID 0 and its default 30W is ID 2; an Orin NX flashed
    for JetPack 6.2's Super modes has MAXN_SUPER at 0 and defaults to 40W (4);
    an Orin Nano Super has MAXN_SUPER at 2; a Nano flashed without Super has no
    MAXN mode at all. The verdict must read the IDs from the module's own
    table, and must not PASS a table it could not read."""
    if not find_bash():
        return SKIP
    nx_super = ("< POWER_MODEL ID=0 NAME=MAXN_SUPER >\n< POWER_MODEL ID=1 NAME=10W >\n"
                "< POWER_MODEL ID=2 NAME=15W >\n< POWER_MODEL ID=3 NAME=25W >\n"
                "< POWER_MODEL ID=4 NAME=40W >\n< PM_CONFIG DEFAULT=4 >\n")
    nano_super = ("< POWER_MODEL ID=0 NAME=15W >\n< POWER_MODEL ID=1 NAME=25W >\n"
                  "< POWER_MODEL ID=2 NAME=MAXN_SUPER >\n< PM_CONFIG DEFAULT=1 >\n")
    nano_plain = ("< POWER_MODEL ID=0 NAME=15W >\n< POWER_MODEL ID=1 NAME=7W >\n"
                  "< PM_CONFIG DEFAULT=0 >\n")
    # The timing runs use the highest mode WITH a power budget: NVIDIA's guide
    # calls MAXN "an unconstrained power mode" and "an experimental mode",
    # with hardware throttling once the module exceeds its TDP. Its ID is the
    # largest wattage in the table, whatever the name's prefix.
    nano_4gb = ("< POWER_MODEL ID=0 NAME=10W >\n< POWER_MODEL ID=1 NAME=7W_AI >\n"
                "< POWER_MODEL ID=2 NAME=7W_CPU >\n< PM_CONFIG DEFAULT=0 >\n")
    cases = (
        ("NVIDIA Jetson Orin NX Engineering Reference Developer Kit", nx_super,
         ("default ID 4 = 40W", "MAXN mode ID 0 = MAXN_SUPER", "Super",
          "highest budgeted mode ID 4 = 40W")),
        ("NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super", nano_super,
         ("default ID 1 = 25W", "MAXN mode ID 2 = MAXN_SUPER",
          "highest budgeted mode ID 1 = 25W")),
        ("NVIDIA Jetson Orin Nano Engineering Reference Developer Kit", nano_plain,
         ("default ID 0 = 15W", "no MAXN mode", "highest budgeted mode ID 0 = 15W")),
        ("NVIDIA Jetson Orin Nano Engineering Reference Developer Kit", nano_4gb,
         ("highest budgeted mode ID 0 = 10W",)),
    )
    for model, conf, parts in cases:
        r, v = _first_boot(*_fake_orin(model=model, nvpmodel_conf=conf))
        assert v["power"][0] == "PASS", (model, v["power"])
        for part in parts:
            assert part in v["power"][1], (model, part, v["power"])
    # The table unreadable, or with no default: the IDs are unknown.
    for conf in (None, "< POWER_MODEL ID=0 NAME=MAXN >\n"):
        r, v = _first_boot(*_fake_orin(nvpmodel_conf=conf))
        assert v["power"][0] == "WARN" and "nvpmodel.conf" in v["power"][1], (conf, v["power"])


def test_the_runbook_asks_for_the_first_boot_check_and_its_raw_outputs():
    text = RUNBOOK.read_text(encoding="utf-8")
    assert "bash deploy/orin/first_boot_check.sh" in text
    assert "deploy/evidence/incoming/hil/first_boot_check.txt" in text
    for raw in ("cat /etc/nv_tegra_release", "nvpmodel -q", "docker info", "ls /opt/ros"):
        assert raw in text, f"the runbook's paste-back list lacks {raw!r}"
    for setting in ("jetson_clocks", "nvpmodel -m", "tegrastats"):
        assert setting in text, f"the runbook's timing steps never mention {setting}"
    design = DESIGN_TOPOLOGIES.read_text(encoding="utf-8")
    for ver in ("36.4.3", "12.6", "10.3", "9.3"):
        assert ver in text and ver in design, ver
    # Timing runs use the module's highest mode WITH a power budget. The Jetson
    # Linux 36.4.3 power guide calls MAXN "an unconstrained power mode" and "an
    # experimental mode", with hardware throttling above the TDP: a long run in
    # it can change speed part-way. No step may send a measurement to MAXN.
    assert "highest budgeted mode" in text and "highest mode with a power budget" in design
    for name, doc in (("RUNBOOK-orin-hil.md", text), ("DESIGN-topologies.md", design)):
        m = re.search(r"(?i)\b(?:measure|run)[^.]*?\bin \**MAXN\b", doc)
        assert not m, f"{name} sends a timing run to MAXN: {m.group(0)!r}"
        # The one place each document assigns the modes is its "Which modes:"
        # list: the budgeted mode first, the default second, and a "Not
        # MAXN" item; no other item may mention MAXN at all, however phrased.
        bullets = _which_modes(doc, name)
        assert bullets[0].startswith("- **The highest mode with a power budget**"), (name, bullets[0])
        assert bullets[1].startswith("- **The default mode**"), (name, bullets[1])
        not_maxn = [b for b in bullets if b.startswith("- **Not MAXN**")]
        assert len(not_maxn) == 1, (name, "no single '**Not MAXN**' item", bullets)
        for b in bullets:
            if b not in not_maxn:
                assert "MAXN" not in b.upper(), (name, "an item other than 'Not MAXN' names MAXN", b)


def test_the_docs_name_who_publishes_the_jetson_base_images():
    """The dustynv images come from jetson-containers, the Jetson AI Lab
    container project, published under a personal Docker Hub account and not
    on NGC. External material states true facts only, so no document may
    present them as NVIDIA's own images."""
    for p in (RUNBOOK, DESIGN_TOPOLOGIES, DEPLOY / "README.md", DOCKER_DIR / "vla.Dockerfile",
              DOCKER_DIR / "companion.Dockerfile", COMPOSE_DIR / "docker-compose.orin.yml"):
        flat = re.sub(r"[\s#]+", " ", p.read_text(encoding="utf-8"))
        m = re.search(r"(?i)NVIDIA(?:'s)?(?: own)? jetson-containers", flat)
        assert not m, f"{p.name} presents jetson-containers as NVIDIA's: {m.group(0)!r}"
    for p in (RUNBOOK, DESIGN_TOPOLOGIES):
        flat = re.sub(r"\s+", " ", p.read_text(encoding="utf-8"))
        assert "published under the dustynv Docker Hub account" in flat and \
            "not an NGC image" in flat, f"{p.name} does not say where the dustynv images come from"


def _which_modes(doc: str, name: str) -> list[str]:
    """The items of a document's "Which modes:" list: from that line to the
    first blank-line paragraph that does not start with '-'."""
    assert doc.count("Which modes:") == 1, f"{name}: not exactly one 'Which modes:' list"
    block = []
    for para in re.split(r"\n[ \t]*\n", doc.split("Which modes:", 1)[1]):
        if not para.strip():
            continue
        if not para.lstrip().startswith("-"):
            break
        block.append(para.strip())
    items = re.split(r"\n(?=- )", "\n".join(block))
    assert len(items) >= 3 and all(i.startswith("- ") for i in items), (name, items)
    return items


# --- deploy/orin/gpu_check.py ------------------------------------------------ #
def _gpu_check_module():
    import importlib.util
    spec = importlib.util.spec_from_file_location("vlaguard_gpu_check", GPU_CHECK)
    mod = importlib.util.module_from_spec(spec)
    old = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec.loader.exec_module(mod)
    finally:
        sys.dont_write_bytecode = old
    return mod


def test_gpu_check_refuses_a_stack_that_only_imports():
    """Run on the Orin inside the vla image. A working stack must compute, and
    each way a Jetson stack fails after a clean import must be named. The
    NF4 error has two nulls: 0 (a pass-through that quantized nothing) and
    about 1 (a kernel that returned zeros)."""
    G = _gpu_check_module()
    good = {"python": "3.12.3", "machine": "aarch64", "torch": "2.6.0",
            "torch_cuda": "12.8", "cuda_available": True, "device_name": "Orin",
            "capability": [8, 7], "bf16_supported": True, "matmul_device": "cuda:0",
            "matmul_rel_err": 2e-4, "bitsandbytes": "0.49.2", "nf4_device": "cuda:0",
            "nf4_rel_err": 0.094}
    assert G.problems(good, G.ORIN_CAPABILITY) == []
    assert G.problems({**good, "bitsandbytes": "skipped", "nf4_rel_err": None,
                       "nf4_device": None}, G.ORIN_CAPABILITY) == []
    bad = (
        ({"torch_cuda": None}, "CPU build"),
        ({"cuda_available": False}, "sees no GPU"),
        ({"capability": [8, 6]}, "expected (8, 7)"),
        ({"matmul_rel_err": 0.5}, "differs from the CPU"),
        # A product that never left the CPU matches the CPU reference exactly.
        ({"matmul_device": "cpu", "matmul_rel_err": 0.0}, "not on a CUDA device"),
        ({"matmul_device": None}, "not on a CUDA device"),
        ({"nf4_device": "cpu"}, "kernels were not exercised"),
        ({"nf4_device": None}, "kernels were not exercised"),
        ({"gpu_error": "RuntimeError: CUDA error"}, "on the GPU failed"),
        ({"bnb_error": "RuntimeError: Error named symbol not found at line 233"}, "sm_87"),
        ({"nf4_rel_err": 0.0}, "nothing was quantized"),
        ({"nf4_rel_err": 1.0}, "returned nothing"),
        ({"nf4_rel_err": None}, "NF4"),
    )
    for patch, text in bad:
        found = G.problems({**good, **patch}, G.ORIN_CAPABILITY)
        assert found and any(text in p for p in found), (patch, found)
    assert G.problems({"torch_error": "ModuleNotFoundError: torch"})
    # The desktop has another GPU: no capability is demanded there.
    assert G.problems({**good, "capability": [8, 9]}, None) == []
    assert "dustynv/l4t-pytorch:r36.4.0" in " ".join(
        G.problems({**good, "cuda_available": False}, G.ORIN_CAPABILITY))


# The known errors the stand-in GPU adds, so the test can tell a result that
# came from the "GPU" path from one computed anywhere else.
_STUB_MATMUL_ERR = 2.0 ** -10        # every product on "cuda" is scaled by 1 + this
_STUB_NF4_ERR = 0.0625               # dequantize returns x * (1 - this)


def _stub_torch(*, available: bool = True, matmul_err: float = _STUB_MATMUL_ERR,
                cuda_error: str | None = None) -> types.ModuleType:
    """Just enough of torch for gpu_check.collect(): 2-D tensors of floats
    that carry a device, refuse to mix devices (as torch does), and whose
    products on 'cuda:0' carry a known relative error."""
    import math
    import random
    torch = types.ModuleType("torch")
    torch.__version__ = "2.6.0"
    torch.version = types.SimpleNamespace(cuda="12.8")

    class T:
        def __init__(self, rows, device="cpu"):
            self.rows, self.device = [list(r) for r in rows], device

        def _same(self, other):
            if self.device != other.device:
                raise RuntimeError("Expected all tensors to be on the same device, but found "
                                   f"at least two devices, {self.device} and {other.device}!")

        def _map(self, f):
            return T([[f(v) for v in r] for r in self.rows], self.device)

        def __matmul__(self, o):
            self._same(o)
            out = T([[sum(a * o.rows[k][j] for k, a in enumerate(row))
                      for j in range(len(o.rows[0]))] for row in self.rows], self.device)
            return out._map(lambda v: v * (1.0 + matmul_err)) if self.device != "cpu" else out

        def __sub__(self, o):
            self._same(o)
            return T([[a - b for a, b in zip(r, s)] for r, s in zip(self.rows, o.rows)],
                     self.device)

        def abs(self):
            return self._map(abs)

        def max(self):
            return max(max(r) for r in self.rows)

        def norm(self):
            return math.sqrt(sum(v * v for r in self.rows for v in r))

        def half(self):
            return self._map(float)

        float = half

        def cuda(self):
            if cuda_error:
                raise RuntimeError(cuda_error)
            return T(self.rows, "cuda:0")

        def cpu(self):
            return T(self.rows, "cpu")

    class Generator:
        def manual_seed(self, seed):
            self.rng = random.Random(seed)
            return self

    torch.Tensor, torch.Generator = T, Generator
    torch.randn = lambda n, m, generator: T([[generator.rng.gauss(0.0, 1.0) for _ in range(m)]
                                             for _ in range(n)])
    torch.cuda = types.SimpleNamespace(
        is_available=lambda: available, get_device_name=lambda i: "Orin (stand-in)",
        get_device_capability=lambda i: (8, 7), is_bf16_supported=lambda: True,
        synchronize=lambda: None)
    return torch


def _stub_bitsandbytes(*, nf4_err: float = _STUB_NF4_ERR, kernel_error: str | None = None):
    bnb = types.ModuleType("bitsandbytes")
    bnb.__version__ = "0.49.2"
    F = types.ModuleType("bitsandbytes.functional")

    def quantize_4bit(x, quant_type):
        assert quant_type == "nf4", quant_type
        if kernel_error:
            raise RuntimeError(kernel_error)
        return x, {"quant_type": quant_type}

    def dequantize_4bit(q, state, quant_type):
        return q._map(lambda v: v * (1.0 - nf4_err))

    F.quantize_4bit, F.dequantize_4bit = quantize_4bit, dequantize_4bit
    bnb.functional = F
    return bnb, F


def _collect_with(torch_mod, bnb=None, skip_bnb: bool = False):
    """gpu_check.collect() with stand-ins in sys.modules (None = not
    installed: `import` then raises), restored afterwards."""
    G = _gpu_check_module()
    names = ("torch", "bitsandbytes", "bitsandbytes.functional")
    saved = {n: sys.modules[n] for n in names if n in sys.modules}
    try:
        sys.modules["torch"] = torch_mod
        sys.modules["bitsandbytes"], sys.modules["bitsandbytes.functional"] = \
            bnb if bnb is not None else (None, None)
        return G, G.collect(skip_bnb=skip_bnb, size=8)
    finally:
        for n in names:
            sys.modules.pop(n, None)
        sys.modules.update(saved)


def test_gpu_check_collect_reports_what_the_gpu_computed():
    """problems() judges facts; collect() is what produces them on the Orin.
    Run it against stand-in torch and bitsandbytes modules whose GPU path
    carries a known error, so a collect() that computed somewhere else, or
    wrote a number instead of measuring one, is seen: the reported errors
    must be the stand-ins' errors and the devices the GPU's."""
    G, facts = _collect_with(_stub_torch(), _stub_bitsandbytes())
    assert facts["matmul_device"] == "cuda:0", facts
    assert abs(facts["matmul_rel_err"] - _STUB_MATMUL_ERR) < 1e-9, facts
    assert facts["nf4_device"] == "cuda:0", facts
    assert abs(facts["nf4_rel_err"] - _STUB_NF4_ERR) < 1e-9, facts
    assert facts["capability"] == [8, 7] and facts["bitsandbytes"] == "0.49.2", facts
    assert G.problems(facts, G.ORIN_CAPABILITY) == [], G.problems(facts, G.ORIN_CAPABILITY)
    # PyPI's aarch64 wheel on an Orin: the kernel call raises; the report keeps it.
    symbol = "Error named symbol not found at line 233 in file /src/csrc/ops.cu"
    G, facts = _collect_with(_stub_torch(), _stub_bitsandbytes(kernel_error=symbol))
    assert symbol in facts.get("bnb_error", ""), facts
    assert any("sm_87" in p for p in G.problems(facts, G.ORIN_CAPABILITY)), facts
    # A dequantize that hands its input back quantized nothing: error 0, refused.
    G, facts = _collect_with(_stub_torch(), _stub_bitsandbytes(nf4_err=0.0))
    assert facts["nf4_rel_err"] == 0.0, facts
    assert any("nothing was quantized" in p for p in G.problems(facts, G.ORIN_CAPABILITY))
    # A GPU whose product is wrong.
    G, facts = _collect_with(_stub_torch(matmul_err=0.5), _stub_bitsandbytes())
    assert abs(facts["matmul_rel_err"] - 0.5) < 1e-9, facts
    assert any("differs from the CPU" in p for p in G.problems(facts, G.ORIN_CAPABILITY))
    # No GPU; a GPU that fails on first use; no torch; no bitsandbytes.
    G, facts = _collect_with(_stub_torch(available=False), _stub_bitsandbytes())
    assert facts["cuda_available"] is False and "matmul_device" not in facts, facts
    assert any("sees no GPU" in p for p in G.problems(facts, G.ORIN_CAPABILITY))
    G, facts = _collect_with(_stub_torch(cuda_error="CUDA error: no kernel image"),
                             _stub_bitsandbytes())
    assert "no kernel image" in facts.get("gpu_error", ""), facts
    assert any("on the GPU failed" in p for p in G.problems(facts, G.ORIN_CAPABILITY))
    G, facts = _collect_with(None)
    assert "torch_error" in facts and G.problems(facts), facts
    G, facts = _collect_with(_stub_torch(), None)
    assert "bnb_error" in facts and G.problems(facts, G.ORIN_CAPABILITY), facts
    # --skip-bnb (the base image, before the build): torch and the GPU only.
    G, facts = _collect_with(_stub_torch(), None, skip_bnb=True)
    assert facts["bitsandbytes"] == "skipped" and "nf4_rel_err" not in facts, facts
    assert G.problems(facts, G.ORIN_CAPABILITY) == [], G.problems(facts, G.ORIN_CAPABILITY)


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
