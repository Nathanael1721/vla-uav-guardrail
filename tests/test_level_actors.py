"""Truth read from the level, and what happens when it is not there.

Run either way:
    pytest tests/test_level_actors.py -v
    python tests/test_level_actors.py

The defect this guards against is the one this repo keeps meeting: a flight
that produces a video, a metrics file and no tracking number, because the
subject's ground truth was an empty list on every tick and nothing said so.
`demo/level_actors.py` therefore has to fail LOUDLY when no name resolves, and
it must never invent a position for one that does not.
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

import level_actors as LA                                   # noqa: E402

NAN = float("nan")


class FakeWorld:
    """Answers GetObjectPoses the way the simulator does: NaN for a miss."""

    def __init__(self, table):
        self.table = table            # name -> (x, y, z) or None for missing
        self.calls = 0
        self.asked = []

    def get_object_poses(self, names):
        self.calls += 1
        self.asked.append(list(names))
        out = []
        for n in names:
            p = self.table.get(n)
            x, y, z = (NAN, NAN, NAN) if p is None else p
            out.append({"translation": {"x": x, "y": y, "z": z},
                        "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}})
        return out


def test_names_match_the_level_tag_convention():
    assert LA.names("Ped_", 3) == ["Ped_00", "Ped_01", "Ped_02"]
    assert LA.names("Car_", 2, start=6) == ["Car_06", "Car_07"]


def test_a_resolved_actor_carries_its_ned_position():
    w = FakeWorld({"Ped_00": (12.5, -3.25, 0.0)})
    a = LA.LevelActors(w, ["Ped_00"])
    assert a.resolve() == 1
    f = a.figures[0]
    assert (round(f.x, 2), round(f.y, 2)) == (12.5, -3.25)
    assert f.seen and f.stale == 0


def test_nothing_resolving_refuses_the_flight():
    """An empty truth must stop the run, not quietly score as unscorable."""
    w = FakeWorld({})
    a = LA.LevelActors(w, ["Ped_00", "Ped_01"])
    try:
        a.resolve()
    except SystemExit as e:
        assert "Ped_00" in str(e) and "tags" in str(e)
    else:
        raise AssertionError("resolve() accepted a truth that is always empty")


def test_a_missing_name_is_dropped_not_carried_as_a_zero():
    w = FakeWorld({"Ped_00": (1.0, 2.0, 0.0), "Ped_01": None})
    a = LA.LevelActors(w, ["Ped_00", "Ped_01"])
    assert a.resolve() == 1
    assert [f.name for f in a.figures] == ["Ped_00"]
    assert all((f.x, f.y) != (0.0, 0.0) for f in a.figures)


def test_a_nan_keeps_the_last_position_and_is_counted():
    """(0, 0) is a real place on this map; a truth that teleports there would
    score the detector WRONG rather than unscorable."""
    w = FakeWorld({"Ped_00": (5.0, 5.0, 0.0)})
    a = LA.LevelActors(w, ["Ped_00"], min_period_s=0.0)
    a.resolve()
    w.table["Ped_00"] = None                     # the actor goes away
    a.update(1.0)
    f = a.figures[0]
    assert (f.x, f.y) == (5.0, 5.0)
    assert f.stale == 1 and a.stats()["nan_reads"] == 1


def test_polling_is_throttled_to_min_period():
    w = FakeWorld({"Ped_00": (0.0, 1.0, 0.0)})
    a = LA.LevelActors(w, ["Ped_00"], min_period_s=0.5)
    a.resolve()                                   # poll 1
    a.update(0.10)                                # too soon
    a.update(0.20)                                # too soon
    assert w.calls == 1
    a.update(0.60)                                # far enough
    assert w.calls == 2


def test_every_name_goes_in_one_call():
    """One RPC per poll, not one per actor - the flight has a 10 Hz budget."""
    w = FakeWorld({n: (0.0, 0.0, 0.0) for n in LA.names("Ped_", 16)})
    a = LA.LevelActors(w, LA.names("Ped_", 16))
    a.resolve()
    assert w.calls == 1 and len(w.asked[0]) == 16


def test_heading_comes_back_in_radians():
    w = FakeWorld({"Car_00": (0.0, 0.0, 0.0)})
    a = LA.LevelActors(w, ["Car_00"], kind="car")
    a.resolve()
    assert abs(a.figures[0].heading) < 1e-9       # identity quaternion
    q = {"w": math.cos(math.pi / 4), "x": 0.0, "y": 0.0, "z": math.sin(math.pi / 4)}
    assert abs(LA._yaw_from_quat(q) - math.pi / 2) < 1e-9


def test_truth_points_read_the_same_way_follow_vlm_reads_them():
    """`subject_truth_pts` does `[[f.x, f.y] for f in people.figures]`."""
    w = FakeWorld({"Ped_00": (3.0, 4.0, 0.0), "Ped_01": (7.0, 8.0, 0.0)})
    a = LA.LevelActors(w, ["Ped_00", "Ped_01"])
    a.resolve()
    pts = [[round(f.x, 2), round(f.y, 2)] for f in a.figures]
    assert pts == [[3.0, 4.0], [7.0, 8.0]]


def test_destroy_does_not_touch_the_level():
    w = FakeWorld({"Ped_00": (0.0, 0.0, 0.0)})
    a = LA.LevelActors(w, ["Ped_00"])
    a.resolve()
    before = w.calls
    a.destroy()
    assert w.calls == before        # no DestroyObject, no extra RPC


def test_a_level_car_stands_in_for_the_scripted_car():
    """`subject_truth_pts` reads `[car.pos[0], car.pos[1]]` - one point, so a
    car mission is scored against one instance however many cars are around."""
    w = FakeWorld({"Car_10": (37.5, -20.0, 0.0)})
    car = LA.LevelCar(w, "Car_10", desc="a red car")
    assert car.spawn() == 1
    assert car.pos == (37.5, -20.0)
    assert car.actual_name is None             # nothing to teleport or destroy
    assert car.spec.desc_match == "a red car"
    w.table["Car_10"] = (40.0, -20.0, 0.0)
    car.update(1.0)
    assert car.pos == (40.0, -20.0)
    assert car.stats()["tag"] == "Car_10"


def test_a_level_car_that_does_not_resolve_refuses_the_flight():
    w = FakeWorld({})
    car = LA.LevelCar(w, "Car_10")
    try:
        car.spawn()
    except SystemExit as e:
        assert "Car_10" in str(e)
    else:
        raise AssertionError("a car with no truth was accepted")


def test_a_level_car_keeps_its_last_position_through_a_nan():
    w = FakeWorld({"Car_10": (5.0, 6.0, 0.0)})
    car = LA.LevelCar(w, "Car_10", min_period_s=0.0)
    car.spawn()
    w.table["Car_10"] = None
    car.update(0.5)
    assert car.pos == (5.0, 6.0)
    assert car.stats()["nan_reads"] == 1


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
