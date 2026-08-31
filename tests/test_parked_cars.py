"""Parked vehicles: where they may stand, and what they may not be.

Run either way:
    pytest tests/test_parked_cars.py -v
    python tests/test_parked_cars.py

These are placement rules, so they are checked offline against the placement
function rather than judged by eye on a video. Two of them are the difference
between a street and a bug:

* a parked car in a driving lane is not scenery, it is an obstruction that the
  moving fleet will drive through;
* a parked `taxi.glb` is not a distractor, it is a SECOND CORRECT ANSWER to the
  demo's standing query "a yellow car", and no amount of tuning would recover
  the hit rate afterwards.

The pedestrian work established the rest of the discipline: static scenery costs
no per-tick RPC, and `det_hit_rate` is the gate on anything added to this scene.
"""
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

import numpy as np                                          # noqa: E402

import city_traffic                                         # noqa: E402
import parked_cars as PC                                    # noqa: E402
from pedestrians import _dist_to_route, pavement_spots      # noqa: E402

CITY = ROOT / "demo" / "out" / "citymap"
ROUTE = [(38.0, -8.0), (38.0, 40.0), (-10.0, 40.0)]


def _world():
    from build_street_mask import load_street
    sm = load_street(CITY / "street.npz")
    bld = np.load(CITY / "occ_day_highband_15to55.npz")["occ"]
    return sm, bld


def test_the_target_model_is_never_parked():
    """taxi.glb is the tracked subject. A parked one is a second right answer."""
    assert PC.TARGET_MODEL == "taxi.glb"
    assert PC.TARGET_MODEL not in PC.PARKED_PALETTE, (
        "the target's own model is in the parked palette; the demo query is "
        "'a yellow car' and det_hit_rate would fall for a reason no tuning fixes")


def test_nothing_parked_is_yellow():
    """Colour, not just model. The gate scores yellow-ness."""
    for name in PC.PARKED_PALETTE:
        assert "taxi" not in name, name
        assert "yellow" not in name.lower(), name


def test_parked_cars_keep_clear_of_both_driving_lines():
    sm, bld = _world()
    circ = city_traffic.circuit()
    spots = PC.kerb_spots(sm, bld, 14, random.Random(20260828), ROUTE, circ)
    assert spots, "no kerb spots found at all"
    for x, y in spots:
        assert _dist_to_route(x, y, ROUTE) >= PC.ROAD_CLEARANCE_M, (
            f"parked car at ({x}, {y}) is in the target's lane")
        assert _dist_to_route(x, y, circ) >= PC.ROAD_CLEARANCE_M, (
            f"parked car at ({x}, {y}) is on the background circuit")


def test_parked_cars_do_not_share_a_cell_with_a_pedestrian():
    """Both draw from the same kerb, so the allocation has to be partitioned."""
    sm, bld = _world()
    peds = pavement_spots(sm, bld, 12, random.Random(20260825), avoid=ROUTE)
    spots = PC.kerb_spots(sm, bld, 14, random.Random(20260828), ROUTE,
                          city_traffic.circuit(), taken=peds)
    for x, y in spots:
        near = min((math.hypot(x - px, y - py) for px, py in peds), default=99)
        assert near >= 5.0, f"parked car at ({x}, {y}) is {near:.1f} m from a person"


def test_parked_cars_do_not_stack_on_each_other():
    sm, bld = _world()
    spots = PC.kerb_spots(sm, bld, 14, random.Random(20260828), ROUTE,
                          city_traffic.circuit())
    for i, (x, y) in enumerate(spots):
        for j, (a, b) in enumerate(spots):
            if i >= j:
                continue
            assert math.hypot(x - a, y - b) >= 5.0, f"{(x, y)} and {(a, b)} overlap"


def test_every_parked_spot_is_on_a_street_cell():
    from build_street_mask import is_street
    sm, bld = _world()
    spots = PC.kerb_spots(sm, bld, 14, random.Random(20260828), ROUTE,
                          city_traffic.circuit())
    for x, y in spots:
        assert is_street(sm, x, y), f"parked car at ({x}, {y}) is off the street"


def test_it_degrades_when_the_asset_pack_is_absent():
    """The repository stays runnable without third-party models."""
    sm, bld = _world()
    p = PC.ParkedCars(None, sm, bld, count=6, models_dir="D:/definitely/not/here")
    assert p.available() is False
    p.spawn()                       # must print and return, not raise


def test_a_zero_count_spawns_nothing_and_touches_no_world():
    """Default is 0, and it must not so much as look at the simulator."""
    sm, bld = _world()
    p = PC.ParkedCars(None, sm, bld, count=0)
    p.spawn()
    assert p.cars == []


def test_the_moving_palette_still_excludes_the_target():
    """--traffic distractors must not include the taxi either."""
    names = [f for f, _ in city_traffic.GLB_PALETTE]
    assert city_traffic.GLB_TARGET[0] not in names
    assert len(names) == len(set(names)), "duplicate model in the palette"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
