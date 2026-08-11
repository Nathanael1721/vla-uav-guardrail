"""Fleet geometry and update budget — everything about demo/city_traffic.py that
can be checked without a simulator.

Run either way:
    pytest tests/test_city_traffic.py -v
    python tests/test_city_traffic.py

What these guard against is a fleet that LOOKS like traffic in the config but
degenerates in the scene: vehicles stacked on each other, every car in the same
place at t=0, a distractor wearing the target's paint, or an RPC bill that eats
the control loop. None of those crash anything. They just quietly turn the
discrimination experiment into a coin flip, which is the worst failure mode
because it still produces a number.
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

import city_traffic as ct                                        # noqa: E402
from moving_car import CarSpec                                   # noqa: E402


class _FakeWorld:
    """MovingCar only touches the world on spawn/update/destroy, and Traffic
    construction does none of those, so geometry is testable with a stub."""

    def spawn_object(self, *a, **k):
        raise AssertionError("no test here should spawn")

    def set_object_pose(self, *a, **k):
        raise AssertionError("no test here should teleport")


def _traffic(**kw):
    return ct.Traffic(_FakeWorld(), fleet=ct.default_fleet(**kw))


# ------------------------------------------------------------------ identity

def test_exactly_one_vehicle_is_the_target_and_it_is_the_painted_one():
    for n in (1, 3, 4):
        t = _traffic(n_background=n)
        targets = [v for v in t.fleet if v.is_target]
        assert len(targets) == 1, f"{len(targets)} targets with {n} background"
        assert targets[0].painted, "the target must be the painted vehicle"
        assert not any(v.painted for v in t.fleet if not v.is_target), (
            "a distractor is painted -- it would compete with the target"
        )


def test_only_the_target_carries_a_colour_word():
    """Distractors ask for 'a car'. If a distractor also claimed the colour, the
    experiment would be testing nothing."""
    t = _traffic(n_background=4)
    for vs in t.fleet:
        spec = vs.car_spec()
        if vs.is_target:
            assert spec.materials, "target has no material to paint with"
            assert "white" in spec.desc_match
        else:
            assert spec.materials == [], "distractor would be painted"
            assert spec.desc_match == "a car"


def test_every_vehicle_is_the_same_mesh():
    """The whole design: shape held constant so colour is the only variable."""
    assets = {vs.car_spec().asset for vs in _traffic(n_background=4).fleet}
    assert len(assets) == 1, f"mixed meshes would confound shape with colour: {assets}"
    assert assets == {CarSpec().asset}


# ------------------------------------------------------------------ geometry

def test_no_two_vehicles_start_on_top_of_each_other():
    """Two meshes in the same place read as one object to the detector."""
    t = _traffic(n_background=4)
    # The lane pitch is 3 m, so two vehicles abreast in adjacent lanes ARE 3 m
    # apart and that is intended. What must not happen is vehicles stacked.
    assert t.min_separation(0.0) > 2.5, (
        f"vehicles {t.min_separation(0.0):.1f} m apart at t=0; the car is 4.3 m long"
    )


def test_vehicles_stay_apart_for_the_whole_run():
    t = _traffic(n_background=4)
    worst, worst_t = float("inf"), None
    for i in range(0, 1200):
        tt = i * 0.1
        d = t.min_separation(tt)
        if d < worst:
            worst, worst_t = d, tt
    assert worst > 2.5, f"vehicles came within {worst:.1f} m at t={worst_t:.1f}s"


def test_every_lane_is_inside_the_clearance_envelope():
    """The corridor is free for x in [30,50], but the obstacle map holds
    buildings only -- a flight at x=48 hit street furniture it cannot see. So
    nothing is placed outside [32,46]."""
    for vs in _traffic(n_background=4).fleet:
        assert 32.0 <= vs.lane_x <= 46.0, f"{vs.name} in lane x={vs.lane_x}"


def test_the_target_owns_the_middle_lane():
    t = _traffic(n_background=4)
    assert t.fleet[0].lane_x == ct.TARGET_LANE_X
    assert all(v.lane_x != ct.TARGET_LANE_X for v in t.fleet[1:]), (
        "a distractor shares the target's lane and will occlude it"
    )


def test_routes_run_the_full_street_in_both_directions():
    fleet = _traffic(n_background=4).fleet
    for vs in fleet:
        (x0, y0), (x1, y1) = vs.route()
        assert x0 == x1 == vs.lane_x
        assert {abs(y0), abs(y1)} == {abs(ct.ROUTE_Y0), abs(ct.ROUTE_Y1)}
    assert any(v.reverse for v in fleet[1:]), "no oncoming traffic"
    assert any(not v.reverse for v in fleet[1:]), "all traffic is oncoming"


# ------------------------------------------------------------------- motion

def test_phases_are_spread_so_the_fleet_is_not_one_clump():
    phases = [v.phase_s for v in _traffic(n_background=4).fleet]
    assert len(set(phases)) == len(phases), f"repeated phase offsets: {phases}"
    assert max(phases) - min(phases) >= 20.0, "phases too tightly bunched"


def test_speeds_differ_so_the_scene_is_not_a_rigid_body():
    speeds = [round(v.speed_mps, 3) for v in _traffic(n_background=4).fleet]
    assert len(set(speeds)) >= 3, f"only {len(set(speeds))} distinct speeds: {speeds}"
    assert all(s > 0.2 for s in speeds)


# --------------------------------------------------------------- rpc budget

def test_the_target_updates_every_tick():
    """It is the thing being closed on and scored against."""
    t = _traffic(n_background=4)
    assert t.fleet[0].update_every == 1


def test_background_vehicles_are_staggered_and_the_bill_stays_sane():
    t = _traffic(n_background=4, bg_every=2)
    assert all(v.update_every == 2 for v in t.fleet[1:])
    # 10 for the target + 4 * 5 for the background.
    assert abs(t.rpc_per_second(10.0) - 30.0) < 1e-6, t.rpc_per_second(10.0)
    assert t.rpc_per_second(10.0) < 60.0, "RPC budget would crowd the control loop"


def test_a_skipped_update_costs_no_accuracy():
    """pose_at is a pure function of time, so staggering must not drift. This is
    what makes the whole stagger safe rather than merely cheap."""
    t = _traffic(n_background=3, bg_every=3)
    car = t.cars[1]
    stepped = [car.pose_at(i * 0.1)[:2] for i in range(0, 300)]
    sparse = [car.pose_at(i * 0.1)[:2] for i in range(0, 300, 3)]
    for k, p in enumerate(sparse):
        assert stepped[k * 3] == p, "pose_at is not time-pure"


def test_update_stagger_actually_skips_and_keeps_truth_current():
    """Traffic.update must skip the RPC but still advance ground truth."""
    class _CountingWorld(_FakeWorld):
        def __init__(self):
            self.n = 0

        def set_object_pose(self, *a, **k):
            self.n += 1

    w = _CountingWorld()
    t = ct.Traffic(w, fleet=ct.default_fleet(n_background=3, bg_every=3))
    for c in t.cars:                       # pretend they spawned
        c.actual_name = c.name
    for tick in range(30):
        t.update(tick * 0.1, tick)
    assert t.n_skipped > 0, "nothing was staggered"
    # target 30, three background at every 3rd tick = 10 each.
    assert w.n == 30 + 3 * 10, w.n
    for vs, car in zip(t.fleet, t.cars):
        assert car.pos == car.pose_at(2.9)[:2], f"{vs.name} truth went stale"


def test_summary_reports_what_an_experiment_needs_to_be_checked():
    t = _traffic(n_background=4)
    s = t.summary()
    assert s["n_vehicles"] == 5
    assert len(s["vehicles"]) == 5
    assert sum(v["is_target"] for v in s["vehicles"]) == 1
    for v in s["vehicles"]:
        for k in ("name", "painted", "material", "lane_x", "speed_mps", "asset"):
            assert k in v, f"summary missing {k}"


# --------------------------------------------------------------------- runner

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
