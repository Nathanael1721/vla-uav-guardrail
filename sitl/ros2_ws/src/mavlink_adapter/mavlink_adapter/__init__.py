"""ROS 2 entry point for the Guardrail's MAVLink adapter node.

The node code is the repository's sitl/mavlink_adapter_node.py; this package
makes it installable (`ros2 run mavlink_adapter mavlink_adapter`) without a
second copy that could drift.
"""
import importlib
import os
import sys
from pathlib import Path

SENTINEL = ("guardrail", "shield.py")


def repo_root(start: str | None = None) -> Path:
    """$GUARDRAIL_ROOT, else the first parent of this file holding the Shield."""
    env = os.environ.get("GUARDRAIL_ROOT")
    if env:
        root = Path(env)
        if not root.joinpath(*SENTINEL).is_file():
            raise RuntimeError(f"GUARDRAIL_ROOT={env!r} holds no guardrail/shield.py")
        return root
    here = Path(start or __file__).resolve()
    for p in here.parents:
        if p.joinpath(*SENTINEL).is_file() and (p / "sitl").is_dir():
            return p
    raise RuntimeError("cannot find the Guardrail repository: set GUARDRAIL_ROOT "
                       "to the directory holding guardrail/ and sitl/")


def load_adapter(start: str | None = None):
    root = repo_root(start)
    for p in (root, root / "sitl"):
        if str(p) not in sys.path:
            sys.path.insert(0, str(p))
    return importlib.import_module("mavlink_adapter_node")
