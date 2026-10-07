"""Which topology label a run on the Project AirSim + ArduPilot rail may wear.

THE GRANT'S THREE TOPOLOGIES (Architecture constraints, p3-4)

    dev     everything on one desktop: simulators, ArduPilot SITL, MAVROS 2,
            Shield, VLA stub or quantized backend. Not for reported KPIs.
    hil     the desktop runs the simulators + MAVROS + GCS; a Jetson Orin runs
            the VLA + Shield over the network. "Canonical KPI configuration."
    flight  the Orin runs the VLA + Shield as a companion computer alongside a
            real ArduPilot autopilot.

This rail is the first in the repository that can be any of them: Project
AirSim renders and simulates, ArduPilot (not simple_flight) flies, and the node
is one process whose camera source and autopilot link are launch options. The
label is decided from evidence the node collects, never from a flag.

WHOSE RULES. The evidence for the MAVROS chain, the Orin and the autopilot kind
is judged by guardrail.manifest.check_topology_evidence - the same function the
ROS 2 Shield node's runs are judged by - so the two rails cannot disagree about
what "hil" means. This module adds only what that function cannot know about a
simulator: whose controller flew the simulated vehicle, and whether the
simulator sat on the node's own desktop.

TWO LABELS PER RUN

  `grant`  dev / hil / flight, or None when the evidence supports none of them.
  `stamp`  what goes into manifest.json today. build_manifest() refuses `dev`,
           `hil` and `flight` together with a Project AirSim scene file - a
           guard written when every scene meant simple_flight. On this rail the
           scene's controller is ArduPilot, so that guard needs teaching
           (docs/DESIGN-projectairsim-ardupilot.md, "What manifest.py needs");
           until it is, this rail stamps its own label and records the grant
           label beside it in metrics.json. Nothing here loosens the guard.

G1. Whether the rail itself is qualified - gate G1's closed-loop checks
(g1_check.py) - is separate from where the processes ran. Both are recorded;
`rail_qualified_by_g1` is None until a verdict file exists. `grant` says what
the evidence supports; perception_node.py OFFERS it to build_manifest only
when G1 passed and ArduPilot's measured real-time factor was in band, and
otherwise stamps `stamp` and records why the label was withheld.
"""
from __future__ import annotations

import ipaddress
import platform
import socket
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail.manifest import (TOPOLOGY_DEV, TOPOLOGY_FLIGHT,       # noqa: E402
                                TOPOLOGY_HIL, TOPOLOGY_PROJECTAIRSIM,
                                check_topology_evidence)

CONTROLLER_TYPE = "ardupilot-api"

# This rail's own manifest labels, in the style of TOPOLOGY_ARDUPILOT_SITL:
# named for what ran, so the difference from the grant's labels is never
# blurred in either direction.
TOPOLOGY_PAS_ARDUPILOT_MAVROS = "projectairsim-ardupilot-mavros"
TOPOLOGY_PAS_ARDUPILOT_PYMAVLINK = "projectairsim-ardupilot-pymavlink"
TOPOLOGY_ARDUPILOT_NO_SIM = "ardupilot-mavros-no-simulator"


def is_wsl() -> bool:
    return "microsoft" in platform.release().lower() or \
        Path("/proc/sys/fs/binfmt_misc/WSLInterop").exists()


def default_gateway_linux(route_file: str | Path = "/proc/net/route") -> str | None:
    """The IPv4 default gateway from /proc/net/route (in WSL: the Windows host)."""
    try:
        lines = Path(route_file).read_text().splitlines()[1:]
    except OSError:
        return None
    for ln in lines:
        f = ln.split()
        if len(f) >= 3 and f[1] == "00000000":
            return str(ipaddress.IPv4Address(int(f[2], 16).to_bytes(4, "little")))
    return None


def sim_on_this_desktop(sim_address: str | None) -> bool | None:
    """Is the simulator on the machine this node runs on?

    True for loopback, or - inside WSL - for the Windows host at WSL's default
    gateway (one desktop, two kernels). None when it cannot be told.
    """
    if not sim_address:
        return None
    try:
        ip = ipaddress.IPv4Address(socket.gethostbyname(sim_address))
    except (OSError, ValueError):
        return None
    if ip.is_loopback:
        return True
    if is_wsl() and default_gateway_linux() == str(ip):
        return True
    try:
        mine = {a[4][0] for a in socket.getaddrinfo(socket.gethostname(), None)}
    except OSError:
        return None
    return str(ip) in mine


def _on_orin(tev: dict) -> bool:
    return (str(tev.get("host_arch") or "").lower() in ("aarch64", "arm64")
            and "orin" in str(tev.get("device_model") or "").lower())


def classify(ev: dict) -> dict:
    """{stamp, grant, rail_qualified_by_g1, reasons} from a run's evidence.

    Evidence keys (all gathered live by the node; see perception_node.py):
      camera_source       "projectairsim" | "ros2"
      controller_type     the controller of the robot config the simulator was
                          given (None when there is no simulator)
      sim_same_desktop    sim_on_this_desktop(<simulator address>)
      link                "mavros" | "pymavlink"
      autopilot           {"heartbeat": bool, "is_ardupilot": bool}
      topology_evidence   the dict check_topology_evidence() reads: the MAVROS
                          chain, ardupilot_version, autopilot_kind,
                          network_link and collect_host_evidence()'s host keys
      g1                  {"passed": bool | None, "verdict": path | None}
    """
    reasons: list[str] = []
    cam, link = ev.get("camera_source"), ev.get("link")
    apv = ev.get("autopilot") or {}
    tev = ev.get("topology_evidence") or {}
    g1 = ev.get("g1") or {}
    qualified = g1.get("passed") if isinstance(g1.get("passed"), bool) else None

    if cam == "projectairsim" and ev.get("controller_type") != CONTROLLER_TYPE:
        reasons.append(
            f"the simulator's robot config uses controller "
            f"{ev.get('controller_type')!r}, so the simulator's own controller "
            f"flew the aircraft; a MAVLink link to an ArduPilot that flies "
            f"nothing does not make this an ArduPilot run")
        return {"stamp": TOPOLOGY_PROJECTAIRSIM, "grant": None,
                "rail_qualified_by_g1": None, "reasons": reasons}

    if link == "pymavlink":
        stamp = TOPOLOGY_PAS_ARDUPILOT_PYMAVLINK
    elif link == "mavros":
        stamp = (TOPOLOGY_PAS_ARDUPILOT_MAVROS if cam == "projectairsim"
                 else TOPOLOGY_ARDUPILOT_NO_SIM)
    else:
        raise ValueError(f"unknown autopilot link {link!r}")

    def out(grant=None):
        if grant in (TOPOLOGY_DEV, TOPOLOGY_HIL) and qualified is not True:
            reasons.append("the perception rail is not yet qualified by gate G1 "
                           + ("(no verdict file)" if qualified is None else "(G1 FAILED)"))
        return {"stamp": stamp, "grant": grant, "rail_qualified_by_g1": qualified,
                "reasons": reasons}

    if not (apv.get("heartbeat") and apv.get("is_ardupilot")):
        reasons.append("no ArduPilot heartbeat was received, so nothing shows "
                       "ArduPilot was in the loop")
        return out()
    if link == "pymavlink":
        reasons.append("the Shield commanded ArduPilot over pymavlink; the "
                       "grant's safety path is MAVROS 2 (Architecture "
                       "constraints, 'MAVROS 2 first'), so this is none of "
                       "dev / hil / flight")
        return out()

    same, orin, kind = ev.get("sim_same_desktop"), _on_orin(tev), tev.get("autopilot_kind")
    if cam != "projectairsim":
        miss = check_topology_evidence(TOPOLOGY_FLIGHT, tev)
        if not miss:
            reasons.append("hardware ArduPilot, no simulator, VLA + Shield on a "
                           "Jetson Orin: the grant's flight topology")
            return out(TOPOLOGY_FLIGHT)
        reasons.append("not flight: " + "; ".join(miss))
        return out()
    if kind == "hardware":
        reasons.append("a hardware autopilot was flown while the camera came from "
                       "a simulator: no grant topology mixes the two")
        return out()
    if orin:
        miss = check_topology_evidence(TOPOLOGY_HIL, tev)
        if same is not False:
            miss.append(f"the simulator is {'on this host' if same else 'at an unknown place'}"
                        f"; hil keeps it on the desktop")
        if not miss:
            reasons.append("Project AirSim + ArduPilot SITL on the desktop, VLA + "
                           "Shield on a Jetson Orin over the network: the grant's "
                           "hil topology")
            return out(TOPOLOGY_HIL)
        reasons.append("not hil: " + "; ".join(miss))
        return out()
    miss = check_topology_evidence(TOPOLOGY_DEV, tev)
    if kind != "sitl":
        miss.append(f"autopilot_kind is {kind!r}: dev flies ArduPilot SITL")
    if same is not True:
        miss.append(f"the simulator is {'elsewhere' if same is False else 'at an unknown place'}"
                    f"; dev keeps everything on one desktop")
    if not miss:
        reasons.append("simulator, ArduPilot SITL, MAVROS 2 and the node on one "
                       "desktop: the grant's dev topology")
        return out(TOPOLOGY_DEV)
    reasons.append("not dev: " + "; ".join(miss))
    return out()
