"""ROS 2 entry points for the Guardrail's Safety Shield and VLA stub nodes.

The node code is the repository's sitl/ros2_shield_node.py and
sitl/ros2_vla_stub_node.py; this package makes it installable and
launchable (`ros2 run safety_shield shield_node`, `ros2 launch safety_shield
guardrail_rail.launch.py`) without a second copy that could drift.
"""
import os
import sys
from pathlib import Path

SENTINEL = ("guardrail", "shield.py")


def repo_root(start: str | None = None) -> Path:
    """$GUARDRAIL_ROOT, else the first parent of this file holding the Shield
    (a `colcon build --symlink-install` resolves back into the source tree)."""
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


def load_sitl_module(name: str, start: str | None = None):
    """Import sitl/<name>.py from the repository, with guardrail/ importable."""
    root = repo_root(start)
    for p in (root, root / "sitl"):
        if str(p) not in sys.path:
            sys.path.insert(0, str(p))
    import importlib
    return importlib.import_module(name)
