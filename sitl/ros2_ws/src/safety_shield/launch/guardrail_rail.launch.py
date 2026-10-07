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

    def args(name):
        return shlex.split(LaunchConfiguration(name).perform(context))

    vla = ExecuteProcess(
        cmd=[py, str(sitl / "ros2_vla_stub_node.py"), *args("vla_args")],
        name="vla_stub", output="screen", additional_env=env)
    adapter = ExecuteProcess(
        cmd=[py, str(sitl / "mavlink_adapter_node.py"),
             *(["--out", out] if out else []), *args("adapter_args")],
        name="mavlink_adapter", output="screen", additional_env=env)
    shield = ExecuteProcess(
        cmd=[py, str(sitl / "ros2_shield_node.py"), *args("shield_args")],
        name="safety_shield", output="screen", additional_env=env)
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
        DeclareLaunchArgument("vla_args", default_value="",
                              description="arguments for sitl/ros2_vla_stub_node.py"),
        DeclareLaunchArgument("adapter_args", default_value="",
                              description="arguments for sitl/mavlink_adapter_node.py"),
        OpaqueFunction(function=_nodes),
    ])
