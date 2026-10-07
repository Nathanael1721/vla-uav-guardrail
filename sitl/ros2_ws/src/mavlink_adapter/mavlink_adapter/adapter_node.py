"""`ros2 run mavlink_adapter mavlink_adapter` -> sitl/mavlink_adapter_node.py main()."""
from . import load_adapter


def main(argv=None) -> None:
    load_adapter().main(argv)


if __name__ == "__main__":
    main()
