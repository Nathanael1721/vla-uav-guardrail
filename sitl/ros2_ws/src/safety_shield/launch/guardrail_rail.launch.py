"""Launch the Guardrail's ROS 2 nodes on the MAVROS 2 rail.

    vla_stub --/vla/action_4d--> safety_shield --/shield/setpoint--> mavlink_adapter --> MAVROS 2

The three nodes of the grant's interface (Safety Shield Outputs table: "ROS 2
node package"), started together; the mission ends when the Shield node
exits, and the launch then shuts the other two down. ArduPilot SITL, the
MAVLink router and MAVROS 2 are started by sitl/run_ros2_demo.sh before this
file (on the Orin, MAVROS runs there and SITL / the router on the desktop).

The nodes run with the interpreter that has the Guardrail's dependencies
(pydantic, shapely, PyYAML, jinja2) next to rclpy: `python:=`, default
$GUARDRAIL_PYTHON or ~/venv-ros/bin/python. The node code is the repository's
sitl/*.py, found from `repo_root:=` (default $GUARDRAIL_ROOT, else the
repository this file sits in - which is also where a `colcon build
--symlink-install` points back to).

    ros2 launch sitl/ros2_ws/src/safety_shield/launch/guardrail_rail.launch.py \\
        shield_args:="--shield on --dynamic" out:=demo/out/ros2_shield_on_dynamic
    # after colcon build + source install/setup.bash:
    ros2 launch safety_shield guardrail_rail.launch.py repo_root:=/path/to/repo

SROS 2 (2026-10-19). `enclave_root:=/guardrail` gives each node its own
enclave, <root>/<process name> (/guardrail/safety_shield,
/guardrail/mavlink_adapter, /guardrail/vla_stub - sitl/sros2_enclaves.txt),
through ROS_SECURITY_ENCLAVE_OVERRIDE in that process's environment: the
three share this launch's environment otherwise, and one enclave for three
processes would make their identities indistinguishable. Security itself is
switched on by ROS_SECURITY_KEYSTORE / ROS_SECURITY_ENABLE /
ROS_SECURITY_STRATEGY in the environment `ros2 launch` runs in, which
sitl/run_ros2_demo.sh sets under SROS2=1 (sitl/sros2_env.sh). The default,
empty, sets no override: the processes start exactly as before.

THE VLA SLOT (2026-10-23). `vla:=stub` (the default) starts
sitl/ros2_vla_stub_node.py as before; `vla:=bridge` starts
sitl/ros2_vla_bridge_node.py instead - AerialVLA through its server, the
to-do VLA-08 - with the same vla_args and `--out` the Shield's own episode
directory (out:=, else the Shield's rule from its shield_args: shield_out),
so its vla_steps.jsonl lands beside the Shield's records; the Shield node
gets --vla-ready-timeout-s <vla_ready_timeout_s:=, default 3300> unless
shield_args already gives one, so it arms only after the bridge's warm-up;
and the bridge gets 15 s after each stop signal to write its records. Never
both: one publisher on /vla/action_4d. The bridge runs in the VLA slot's enclave,
<root>/vla_stub: an enclave of its own would make every keystore built before
it refuse SROS2=1 until rebuilt (sros2_prepare checks every enclave in
sitl/sros2_enclaves.txt). Since 2026-11-03 the slot's permissions are per
topic (sitl/sros2_policy.xml, profiles vla_stub and vla_bridge): either may
publish /vla/action_4d and /vla/identity and nothing else of the rail's.

MISSION STYLE (2026-11-03, WP3-09). `style:=mission` flies the grant's
pre-planned waypoint mission instead of live setpoints: the Shield process
runs sitl/ros2_mission_node.py (the mission-style Shield: it checks the
plan, sends it to the adapter as MISSION_ITEM_INT and replaces the next
waypoint in flight) with shield_args, in the Shield's enclave, and no VLA
node starts (the plan is the pilot's whole output). `style:=guided`, the
default, is every run before it. The rail still ends with the Shield process.
"""
import os
import shlex
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

SENTINEL = ("guardrail", "shield.py")
# What may fill the VLA slot (vla:=), and the enclave every one of them runs
# in (THE VLA SLOT above).
VLA_KINDS = ("stub", "bridge")
# The grant's two mission styles (style:=; MISSION STYLE above).
STYLES = ("guided", "mission")
VLA_ENCLAVE = "vla_stub"
# vla:=bridge: how long the Shield node waits for the bridge's ready identity
# before it refuses to arm (--vla-ready-timeout-s; vla_ready_timeout_s:=).
# More than the bridge's hello timeout (vla_mission.HELLO_TIMEOUT_S, 2700 s:
# a cold bf16 load took over 15 min) plus its warm-up (three inferences and
# their camera waits, under 3 min), so a late hello still wins the race;
# run_ros2_demo.sh uses the same default and the tests pin both.
VLA_READY_TIMEOUT_S = "3300"
# The bridge writes its records when it is stopped: launch's SIGINT, then
# SIGTERM after this long, then SIGKILL after as long again (launch's
# default is 5 s each).
BRIDGE_STOP_S = "15"


def shield_out(root: Path, shield_args: list[str]) -> str:
    """The Shield node's episode directory from its arguments: demo/out/<--tag>,
    else ros2_shield_<on|off>[_dynamic] (sitl/ros2_shield_node.default_tag)."""
    tag, shield, dynamic = None, "on", False
    for i, a in enumerate(shield_args):
        nxt = shield_args[i + 1] if i + 1 < len(shield_args) else None
        if a == "--tag" and nxt:
            tag = nxt
        elif a.startswith("--tag="):
            tag = a.split("=", 1)[1]
        elif a == "--shield" and nxt:
            shield = nxt
        elif a.startswith("--shield="):
            shield = a.split("=", 1)[1]
        elif a == "--dynamic":
            dynamic = True
    tag = tag or f"ros2_shield_{shield}" + ("_dynamic" if dynamic else "")
    return str(root / "demo" / "out" / tag)


def find_repo_root(start: str | None = None) -> str:
    """$GUARDRAIL_ROOT, else the first parent of this file holding the Shield."""
    env = os.environ.get("GUARDRAIL_ROOT")
    if env:
        return env
    here = Path(start or __file__).resolve()
    for p in here.parents:
        if (p.joinpath(*SENTINEL)).is_file() and (p / "sitl").is_dir():
            return str(p)
    return ""


def _nodes(context):
    py = LaunchConfiguration("python").perform(context)
    root = Path(LaunchConfiguration("repo_root").perform(context) or ".")
    if not root.joinpath(*SENTINEL).is_file():
        raise RuntimeError(f"repo_root {str(root)!r} holds no guardrail/shield.py; "
                           f"pass repo_root:= or set GUARDRAIL_ROOT")
    sitl = root / "sitl"
    out = LaunchConfiguration("out").perform(context)
    env = {"GUARDRAIL_ROOT": str(root)}
    enclave_root = LaunchConfiguration("enclave_root").perform(context).strip().rstrip("/")
    if enclave_root and not enclave_root.startswith("/"):
        raise RuntimeError(f"enclave_root {enclave_root!r} must be an absolute enclave "
                           f"path such as /guardrail")

    def args(name):
        return shlex.split(LaunchConfiguration(name).perform(context))

    def env_for(name):
        """The process's environment: its own SROS 2 enclave when
        enclave_root is given, nothing added otherwise."""
        if not enclave_root:
            return env
        return {**env, "ROS_SECURITY_ENCLAVE_OVERRIDE": f"{enclave_root}/{name}"}

    kind = LaunchConfiguration("vla").perform(context).strip() or "stub"
    if kind not in VLA_KINDS:
        raise RuntimeError(f"vla:={kind!r} is not one of {VLA_KINDS}")
    style = LaunchConfiguration("style").perform(context).strip() or "guided"
    if style not in STYLES:
        raise RuntimeError(f"style:={style!r} is not one of {STYLES}")
    shield_args = args("shield_args")
    if style == "mission":
        # The mission-style Shield in the Shield's place and enclave; no VLA.
        adapter = ExecuteProcess(
            cmd=[py, str(sitl / "mavlink_adapter_node.py"),
                 *(["--out", out] if out else []), *args("adapter_args")],
            name="mavlink_adapter", output="screen",
            additional_env=env_for("mavlink_adapter"))
        shield = ExecuteProcess(
            cmd=[py, str(sitl / "ros2_mission_node.py"), *shield_args],
            name="safety_shield", output="screen", additional_env=env_for("safety_shield"))
        end = RegisterEventHandler(OnProcessExit(
            target_action=shield,
            on_exit=[LogInfo(msg="mission-style Shield finished - shutting the rail down"),
                     EmitEvent(event=Shutdown(reason="mission over"))]))
        return [adapter, shield, end]
    if kind == "bridge":
        # Beside the Shield's records, and the Shield arms only once the
        # bridge's warm-up has passed - on this path too, not only through
        # run_ros2_demo.sh (review of 339f23a, P2-2, P3).
        vla = ExecuteProcess(
            cmd=[py, str(sitl / "ros2_vla_bridge_node.py"), *args("vla_args"),
                 "--out", out or shield_out(root, shield_args)],
            name="vla_bridge", output="screen", additional_env=env_for(VLA_ENCLAVE),
            sigterm_timeout=BRIDGE_STOP_S, sigkill_timeout=BRIDGE_STOP_S)
        if not any(a.split("=", 1)[0] == "--vla-ready-timeout-s" for a in shield_args):
            shield_args += ["--vla-ready-timeout-s",
                            LaunchConfiguration("vla_ready_timeout_s").perform(context)]
    else:
        vla = ExecuteProcess(
            cmd=[py, str(sitl / "ros2_vla_stub_node.py"), *args("vla_args")],
            name="vla_stub", output="screen", additional_env=env_for(VLA_ENCLAVE))
    adapter = ExecuteProcess(
        cmd=[py, str(sitl / "mavlink_adapter_node.py"),
             *(["--out", out] if out else []), *args("adapter_args")],
        name="mavlink_adapter", output="screen", additional_env=env_for("mavlink_adapter"))
    shield = ExecuteProcess(
        cmd=[py, str(sitl / "ros2_shield_node.py"), *shield_args],
        name="safety_shield", output="screen", additional_env=env_for("safety_shield"))
    end = RegisterEventHandler(OnProcessExit(
        target_action=shield,
        on_exit=[LogInfo(msg="safety_shield finished - shutting the rail down"),
                 EmitEvent(event=Shutdown(reason="mission over"))]))
    return [vla, adapter, shield, end]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "python", default_value=os.environ.get(
                "GUARDRAIL_PYTHON", os.path.expanduser("~/venv-ros/bin/python")),
            description="interpreter with rclpy and the Guardrail's dependencies"),
        DeclareLaunchArgument("repo_root", default_value=find_repo_root(),
                              description="the repository holding guardrail/ and sitl/"),
        DeclareLaunchArgument("out", default_value="",
                              description="episode directory (adapter_log.jsonl)"),
        DeclareLaunchArgument("shield_args", default_value="--shield on",
                              description="arguments for sitl/ros2_shield_node.py"),
        DeclareLaunchArgument("vla", default_value="stub",
                              description="what fills the VLA slot: stub "
                                          "(sitl/ros2_vla_stub_node.py) or bridge "
                                          "(sitl/ros2_vla_bridge_node.py, AerialVLA)"),
        DeclareLaunchArgument("style", default_value="guided",
                              description="the grant's mission style: guided (live "
                                          "setpoints, sitl/ros2_shield_node.py) or mission "
                                          "(a waypoint mission, sitl/ros2_mission_node.py)"),
        DeclareLaunchArgument("vla_args", default_value="",
                              description="arguments for the VLA node (the stub or the bridge)"),
        DeclareLaunchArgument("vla_ready_timeout_s", default_value=VLA_READY_TIMEOUT_S,
                              description="vla:=bridge: the Shield node's --vla-ready-timeout-s "
                                          "(unless shield_args gives one)"),
        DeclareLaunchArgument("adapter_args", default_value="",
                              description="arguments for sitl/mavlink_adapter_node.py"),
        DeclareLaunchArgument("enclave_root", default_value="",
                              description="SROS 2: run each node in the enclave "
                                          "<enclave_root>/<node> (e.g. /guardrail); "
                                          "empty = no enclave override"),
        OpaqueFunction(function=_nodes),
    ])
