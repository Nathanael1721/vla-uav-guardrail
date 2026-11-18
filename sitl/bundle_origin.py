"""Print a signed flight bundle's frame origin as "lat,lon", or an empty line
for a policy written in metres (it has no origin).

run_ros2_demo.sh boots SITL there (SITL_HOME, 2026-11-04), so a geographic
policy is flown where it is and the autopilot's GeoFence lands on its zones.
Exit 2 when the bundle does not load: a flight must not start somewhere it
cannot name.

    python sitl/bundle_origin.py bundles/corridor-survey-v0.1.3.tar.gz
    35.6895,139.6917

A DECLARED GROUND (X-11, 2026-11-18). A bundle with an MSL rule passes the
flight gate only over the ground it flies over, so until this date this
script refused one and the runbook had SITL_HOME typed by hand. It takes the
flight's ground by name, as the Shield node does:

    python sitl/bundle_origin.py bundles/msl-dem-sitl-v0.1.0.tar.gz \\
        --ground experiments/worlds/dem_cmac_wgs84.json --ground-sha256 <hex>

(--ground-sha256 optional; when given the file must hash to it). The bundle
is then loaded through the same gate with that ground; nothing else changes.
The ground is never guessed: without --ground an MSL bundle is still
refused (exit 2).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def origin_of(bundle: str, ground: str | None = None,
              ground_sha256: str | None = None) -> str:
    """"lat,lon" of the bundle's frame origin, or "" for a metre policy.
    `ground` / `ground_sha256`: the flight's declared ground (module
    docstring), loaded in the bundle's frame and handed to the flight gate."""
    from guardrail.bundle import load_for_flight
    from guardrail.terrain import ground_from_args
    named = argparse.Namespace(ground=ground, ground_sha256=ground_sha256)
    g, _ = ground_from_args(named, bundle, None, root=ROOT)
    policy = load_for_flight(bundle, None, **({} if g is None else {"ground": g}))[0]
    o = policy.frame_origin
    return "" if o is None else f"{o[0]!r},{o[1]!r}"


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    ap = argparse.ArgumentParser(prog="bundle_origin.py", add_help=False)
    ap.add_argument("bundle", nargs="?")
    ap.add_argument("--ground", default=None)
    ap.add_argument("--ground-sha256", default=None)
    try:
        args, rest = ap.parse_known_args(argv)
    except SystemExit:
        print(__doc__, file=sys.stderr)
        return 64
    if not args.bundle or rest:
        print(__doc__, file=sys.stderr)
        return 64
    try:
        print(origin_of(args.bundle, args.ground, args.ground_sha256))
    except Exception as e:  # noqa: BLE001 - any failure to load refuses the flight
        print(f"cannot read the frame origin of {args.bundle}: {type(e).__name__}: {e}",
              file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
