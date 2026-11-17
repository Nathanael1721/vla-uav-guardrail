"""
The MAVROS chain's evidence and the six-field manifest, gathered the same way
by every ROS 2 rail node that flies through MAVROS 2.

WHY ONE MODULE

The grant's WP4 spec has every episode emit a six-field manifest
(code_revision, vla_model_hash, policy_hash, random_seed, sim_speedup,
topology; contract item WP4-12), and guardrail/manifest.py stamps a grant
topology (dev / hil / flight) only on EVIDENCE the node gathers from the live
system: the MAVROS node seen on the graph, a flight controller reporting
connected, the autopilot naming itself (AUTOPILOT_VERSION), the MAVLink link
MAVROS flies over, where SITL and MAVROS run, the host's device tree, and
SIM_SPEEDUP read from the autopilot - never a literal. Until 2026-11-06 that
gathering lived inside the GUIDED Shield node (sitl/ros2_shield_node.py),
and the mission-style node (sitl/ros2_mission_node.py, 2026-11-03) gathered
none of it and wrote no manifest: mission_wp_20261104 carried no
code_revision, sim_speedup or topology at all, so its revision was only in
the flight driver's log. Both nodes now gather it here.

ROS-FREE ON PURPOSE

Every reader takes the node's own service call (or its answer) and a spin
as callables, and the node's clock as `now`. So:
  * it is tested without rclpy (tests/test_sitl_rails.py);
  * each node keeps its method names (read_sim_speedup,
    read_autopilot_version, read_link), which the stand-in ROS stack of
    tests/test_railinputs_node.py replaces in its tests;
  * a node's virtual clock and its stand-in rclpy (the tests patch the node
    module's `time` and `rclpy`) reach the readers through those callables,
    exactly as when the code lived in the node.

The evidence dict is the one guardrail.manifest.check_topology_evidence reads
(its keys are that function's, not this module's).
"""
from __future__ import annotations

import os
import time
from typing import Any, Callable

from guardrail.manifest import (TOPOLOGY_ARDUPILOT_SITL, build_manifest,
                                collect_host_evidence, decode_custom_version,
                                describe_autopilot_version, detect_topology,
                                mavros_location, network_link_evidence,
                                ros2_security_evidence, scan_local_processes,
                                write_stack_revisions)

# How long a reader polls MAVROS before it gives up and records what it has.
SPEEDUP_WAIT_S = 20.0
VERSION_WAIT_S = 15.0
# MAVROS's VehicleInfo.available_info bit for AUTOPILOT_VERSION received.
HAVE_INFO_AUTOPILOT_VERSION = 2
# rcl_interfaces/ParameterType: PARAMETER_NOT_SET and PARAMETER_STRING.
PARAMETER_NOT_SET = 0
PARAMETER_STRING = 4


def initial_evidence(*, vla_node: bool, environ=None,
                     host: dict | None = None) -> dict[str, Any]:
    """The evidence a MAVROS rail node starts with, before it has seen MAVROS.

    The MAVROS chain's fields are None until the node sees them (they are
    filled by note_mavros_state, poll_autopilot_version and record_link), so
    a node that never reached its autopilot records exactly that. The host is
    read (collect_host_evidence), never declared; SROS 2 is recorded only
    when it is on (`ros2_security`), so a run without it records what it
    always did.

    `vla_node`: whether a SEPARATE VLA node flies this run and reports its
    own host on /vla/identity (the GUIDED Shield node). The key `vla_host`
    is then present and None until the VLA node says where it runs, which
    hil and flight refuse (guardrail.manifest._vla_host_missing). The
    mission-style node has no VLA at all, so the key is left out: absent
    means no separate VLA node, the reading the manifest gives it.
    """
    env = os.environ if environ is None else environ
    ev: dict[str, Any] = {
        "ros_distro": env.get("ROS_DISTRO"),
        "mavros_node": None,
        "fcu_connected": None,
        "ardupilot_version": None,
        "ardupilot_version_source": None,
    }
    if vla_node:
        ev["vla_host"] = None
    ev.update(collect_host_evidence() if host is None else host)
    security = ros2_security_evidence(env)
    if security is not None:
        ev["ros2_security"] = security
    return ev


def note_mavros_state(evidence: dict, connected) -> None:
    """A /mavros/state message: MAVROS is on the graph, and whether it says
    the flight controller is connected (the load-bearing field of
    check_hil_evidence: MAVROS publishes connected: false forever with
    nothing on the other end of its link)."""
    evidence["mavros_node"] = "/mavros"
    evidence["fcu_connected"] = bool(connected)


def param_value(v) -> Any:
    """One rcl_interfaces/ParameterValue as Python: None when the parameter
    is not set, the string for a string, else the number. MAVROS sometimes
    reports SIM_SPEEDUP as INTEGER with the value in double_value, so the
    number is whichever field carries one."""
    if v.type == PARAMETER_NOT_SET:
        return None
    if v.type == PARAMETER_STRING:
        return v.string_value
    return float(v.double_value) or float(v.integer_value)


def get_params(call: Callable, client, request: Callable, names: list[str]) -> dict:
    """{name: value} from a GetParameters service through the node's own
    `call(client, req, timeout)`; {} when the call raises, and only the names
    answered when it returns nothing."""
    try:
        res = call(client, request(names=names), 5.0)
    except Exception:                                 # noqa: BLE001
        return {}
    if res is None:
        return {}
    return {n: param_value(v) for n, v in zip(names, res.values)}


def poll_sim_speedup(get: Callable[[list[str]], dict], spin: Callable[[], None],
                     evidence: dict, *, now: Callable[[], float] = time.time,
                     wait_s: float = SPEEDUP_WAIT_S,
                     warn: Callable[[str], None] | None = None) -> float | None:
    """SIM_SPEEDUP from the autopilot, or None - and whether the autopilot is
    SITL at all (`evidence["autopilot_kind"]`).

    MAVROS 2 surfaces autopilot parameters as node parameters of /mavros/param
    (rcl_interfaces/GetParameters, `get`); it populates them itself on FCU
    connect, and a forced pull raced the refill and returned 0.0, so this
    polls and accepts the first positive answer. A hardware autopilot has no
    SIM_ parameters: SYSID_THISMAV present and SIM_SPEEDUP absent marks it
    `hardware`. Never assumed: an unread speedup is None, which the manifest
    writes as "unresolved", never as 1.0.

    Read it before the mission's tick timer starts: a service call from a
    timer callback is a nested spin and never completes.
    """
    deadline = now() + wait_s
    vals: dict = {}
    while now() < deadline:
        vals = get(["SIM_SPEEDUP", "SYSID_THISMAV"])
        sp = vals.get("SIM_SPEEDUP")
        if isinstance(sp, float) and sp > 0.0:
            evidence["autopilot_kind"] = "sitl"
            return round(sp, 4)
        spin()
    # Only after the whole wait: while MAVROS is still loading the parameter
    # set, SYSID_THISMAV can be present before SIM_* is.
    hardware = bool(vals.get("SYSID_THISMAV")) and \
        "SIM_SPEEDUP" in vals and vals["SIM_SPEEDUP"] is None
    evidence["autopilot_kind"] = "hardware" if hardware else None
    if not hardware and warn is not None:
        warn("[speedup] SIM_SPEEDUP never resolved")
    return None


def poll_autopilot_version(vehicle_info: Callable[[], Any], spin: Callable[[], None],
                           evidence: dict, *, now: Callable[[], float] = time.time,
                           wait_s: float = VERSION_WAIT_S, environ=None,
                           warn: Callable[[str], None] | None = None) -> None:
    """AUTOPILOT_VERSION as MAVROS received it (/mavros/vehicle_info_get,
    `vehicle_info()` returning the service's answer or None; ARCH-31); the
    setup script's banner (ARDUPILOT_VERSION, run_ros2_demo.sh) only as a
    labelled fallback. check_hil_evidence refuses a run whose autopilot is
    not named."""
    deadline = now() + wait_s
    while now() < deadline:
        try:
            res = vehicle_info()
        except Exception:                             # noqa: BLE001
            res = None
        if res is not None and res.success and res.vehicles:
            v = res.vehicles[0]
            if v.available_info & HAVE_INFO_AUTOPILOT_VERSION:
                custom = decode_custom_version(v.flight_custom_version)
                evidence.update({
                    "ardupilot_version": describe_autopilot_version(
                        v.flight_sw_version, custom, v.type, v.autopilot),
                    "ardupilot_version_source":
                        "AUTOPILOT_VERSION via /mavros/vehicle_info_get",
                    "flight_sw_version": f"{int(v.flight_sw_version):08x}",
                    "flight_custom_version_raw": v.flight_custom_version,
                })
                return
        spin()
    env = os.environ if environ is None else environ
    banner = env.get("ARDUPILOT_VERSION")
    if banner:
        evidence.update({
            "ardupilot_version": banner,
            "ardupilot_version_source":
                "sitl/setup_sitl.sh --verify binary banner (the "
                "autopilot itself did not answer)",
            "ardupilot_pin_ok": env.get("ARDUPILOT_PIN_OK") == "1"})
    elif warn is not None:
        warn("autopilot version not read")


def record_link(fcu_url: str | None, evidence: dict, *, environ=None,
                scan: Callable[[], Any] = scan_local_processes) -> None:
    """The MAVLink link MAVROS flies over: MAVROS's own fcu_url parameter
    (`fcu_url`, read from /mavros by the node), else the URL the launch
    script started it with (MAVROS_FCU_URL, labelled so). Then which MAVLink
    actors run on THIS host (None off Linux): where SITL and MAVROS are
    decides dev vs hil, whatever the URL says."""
    env = os.environ if environ is None else environ
    src = "/mavros fcu_url parameter"
    if not fcu_url:
        fcu_url, src = env.get("MAVROS_FCU_URL"), "MAVROS_FCU_URL (launch script)"
    evidence["network_link"] = {**network_link_evidence(fcu_url),
                                "source": src if fcu_url else None}
    evidence["mavlink_router"] = env.get("MAVLINK_ROUTER")
    evidence["local_processes"] = scan()
    evidence["mavros_on"] = mavros_location(evidence)


def topology_manifest(*, policy_hash: str, model_id: str, seed: int,
                      sim_speedup: float | None, evidence: dict,
                      reported_digest: str | None = None,
                      warn: Callable[[str], None] | None = None
                      ) -> tuple[dict, str | None]:
    """The run's six-field manifest, and why its evidence did not hold (None
    when it did).

    The topology is the one the evidence supports (detect_topology: flight,
    hil or dev), and build_manifest checks the evidence again. A run whose
    evidence supports none of the three - MAVROS never connected, the
    autopilot never named - is refused the grant label and stamped
    TOPOLOGY_ARDUPILOT_SITL instead; the refusal is returned for metrics.json
    `topology_refused`, never a silent upgrade. `policy_hash` is the hash in
    force at the END of the episode (the last generation after any
    hot-apply): guardrail.replay's pack binds the bundle to it, and
    resolve_run_generation_chain resolves it through policy_g0.json.
    `sim_speedup` must come from a read such as poll_sim_speedup, never a
    literal (build_manifest).
    """
    topology = detect_topology(evidence)
    common = dict(policy_hash=policy_hash, model_id=model_id, seed=seed,
                  scene_path=None, sim_speedup=sim_speedup,
                  reported_digest=reported_digest)
    try:
        return build_manifest(**common, topology=topology, hil_evidence=evidence), None
    except ValueError as e:
        if warn is not None:
            warn(f"not the {topology} topology: {e}")
        return build_manifest(**common, topology=TOPOLOGY_ARDUPILOT_SITL), str(e)
