"""Launch the MAVLink adapter alone (e.g. on the Orin, beside MAVROS 2).

    ros2 launch mavlink_adapter mavlink_adapter.launch.py out:=/path/to/episode

Runs sitl/mavlink_adapter_node.py with `python:=` (default $GUARDRAIL_PYTHON
or ~/venv-ros/bin/python) from `repo_root:=` (default $GUARDRAIL_ROOT, else
the repository this file sits in).
"""
import os
import shlex
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

SENTINEL = ("guardrail", "shield.py")


def find_repo_root(start: str | None = None) -> str:
    env = os.environ.get("GUARDRAIL_ROOT")
    if env:
        return env
    for p in Path(start or __file__).resolve().parents:
        if p.joinpath(*SENTINEL).is_file() and (p / "sitl").is_dir():
            return str(p)
    return ""


def _node(context):
    py = LaunchConfiguration("python").perform(context)
    root = Path(LaunchConfiguration("repo_root").perform(context) or ".")
    if not root.joinpath(*SENTINEL).is_file():
        raise RuntimeError(f"repo_root {str(root)!r} holds no guardrail/shield.py")
    out = LaunchConfiguration("out").perform(context)
    extra = shlex.split(LaunchConfiguration("adapter_args").perform(context))
    return [ExecuteProcess(
        cmd=[py, str(root / "sitl" / "mavlink_adapter_node.py"),
             *(["--out", out] if out else []), *extra],
        name="mavlink_adapter", output="screen",
        additional_env={"GUARDRAIL_ROOT": str(root)})]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("python", default_value=os.environ.get(
            "GUARDRAIL_PYTHON", os.path.expanduser("~/venv-ros/bin/python"))),
        DeclareLaunchArgument("repo_root", default_value=find_repo_root()),
        DeclareLaunchArgument("out", default_value=""),
        DeclareLaunchArgument("adapter_args", default_value=""),
        OpaqueFunction(function=_node),
    ])
