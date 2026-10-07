"""`ros2 run safety_shield shield_node` -> sitl/ros2_shield_node.py main()."""
from . import load_sitl_module


def main(argv=None) -> None:
    load_sitl_module("ros2_shield_node").main(argv)


if __name__ == "__main__":
    main()
