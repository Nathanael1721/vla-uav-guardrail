"""`ros2 run safety_shield vla_bridge_node` -> sitl/ros2_vla_bridge_node.py main()."""
from . import load_sitl_module


def main(argv=None) -> None:
    raise SystemExit(load_sitl_module("ros2_vla_bridge_node").main(argv))


if __name__ == "__main__":
    main()
