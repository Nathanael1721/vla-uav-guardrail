"""`ros2 run safety_shield vla_stub_node` -> sitl/ros2_vla_stub_node.py main()."""
from . import load_sitl_module


def main(argv=None) -> None:
    load_sitl_module("ros2_vla_stub_node").main(argv)


if __name__ == "__main__":
    main()
