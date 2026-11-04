"""Print a signed flight bundle's frame origin as "lat,lon", or an empty line
for a policy written in metres (it has no origin).

run_ros2_demo.sh boots SITL there (SITL_HOME, 2026-11-04), so a geographic
policy is flown where it is and the autopilot's GeoFence lands on its zones.
Exit 2 when the bundle does not load: a flight must not start somewhere it
cannot name.

    python sitl/bundle_origin.py bundles/corridor-survey-v0.1.2.tar.gz
    35.6895,139.6917
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def origin_of(bundle: str) -> str:
    """"lat,lon" of the bundle's frame origin, or "" for a metre policy."""
    from guardrail.bundle import load_for_flight
    policy = load_for_flight(bundle, None)[0]
    o = policy.frame_origin
    return "" if o is None else f"{o[0]!r},{o[1]!r}"


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__, file=sys.stderr)
        return 64
    try:
        print(origin_of(argv[0]))
    except Exception as e:  # noqa: BLE001 - any failure to load refuses the flight
        print(f"cannot read the frame origin of {argv[0]}: {type(e).__name__}: {e}",
              file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
