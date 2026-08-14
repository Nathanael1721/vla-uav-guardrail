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

def test_exactly_one_vehicle_is_the_target():
    for n in (1, 2, 3, 4):
        t = _traffic(n_background=n)
        targets = [v for v in t.fleet if v.is_target]
        assert len(targets) == 1, f"{len(targets)} targets with {n} distractors"


def test_every_vehicle_LOOKS_different_in_demo_mode():
    """The point of this rebuild. With every car looking the same the noun cannot
    separate them and neither can the colour gate, so the tracker has nothing to
    hold and wanders between them -- which is exactly what it was doing.

    The invariant is APPEARANCE, not material path: M_Orange renders white on the
    sports car and genuinely orange on the offroad body, so the same material on
    two meshes is two different-looking vehicles."""
    t = _traffic(n_background=3, mode="demo")
    looks = [(v.asset, v.material) for v in t.fleet]
    assert len(set(looks)) == len(looks), f"two vehicles look identical: {looks}"


def test_the_target_is_the_only_one_answering_the_demo_phrase():
    t = _traffic(n_background=3)
    target = t.fleet[0]
    assert target.is_target and target.colour_word == ct.TARGET_COLOUR_WORD
    for vs in t.fleet[1:]:
        assert vs.colour_word != target.colour_word, (
            f"{vs.name} shares the target's colour word"
        )


def test_experiment_mode_holds_the_mesh_constant():
    """The controlled version. Mixing meshes makes the scene readable but
    confounds shape with colour - a correct lock could be the paint or could be
    the silhouette. Experiment mode gives that up on purpose."""
    assets = {vs.car_spec().asset
              for vs in _traffic(n_background=3, mode="experiment").fleet}
    assert assets == {ct.TARGET_ASSET}, f"experiment mode mixed meshes: {assets}"


def test_demo_mode_deliberately_does_not():
    """Stated as a test so the trade is visible rather than implied."""
    assets = {vs.car_spec().asset for vs in _traffic(n_background=3, mode="demo").fleet}
    assert len(assets) > 1, "demo mode gained nothing over experiment mode"


# ------------------------------------------------------------------- motion

def test_the_heading_never_turns_faster_than_a_car_can():
    """The complaint that started this rebuild. Per-vehicle 1 m loops with a
    0.6 m corner radius turned the heading at 109 deg/s; a real car manages
    about 30."""
    t = _traffic(n_background=3)
    for vs, car in zip(t.fleet, t.cars):
        hs = [car.pose_at(i * 0.1)[2] for i in range(1400)]
        worst = max(abs(math.degrees(math.atan2(math.sin(hs[i + 1] - hs[i]),
                                                math.cos(hs[i + 1] - hs[i]))))
                    for i in range(len(hs) - 1)) * 10.0
        assert worst <= 35.0, f"{vs.name} turns at {worst:.0f} deg/s"


def test_the_step_between_ticks_stays_small():
    """Teleport steps are what the eye reads as stutter."""
    t = _traffic(n_background=3)
    for vs, car in zip(t.fleet, t.cars):
        step = max(math.dist(car.pose_at(i * 0.1)[:2], car.pose_at((i + 1) * 0.1)[:2])
                   for i in range(600))
        assert step <= 0.35, f"{vs.name} jumps {step:.2f} m per tick"


def test_distractors_share_one_speed_so_they_cannot_collide():
    """They share a circuit. Unequal speeds mean a faster car catches a slower
    one and drives through it -- measured at 0.0 m separation before this."""
    speeds = {round(v.speed_mps, 4) for v in _traffic(n_background=3).fleet[1:]}
    assert len(speeds) == 1, f"distractors have differing speeds: {speeds}"


def test_no_two_vehicles_ever_meet():
    t = _traffic(n_background=3)
    worst, worst_t = float("inf"), None
    for i in range(1400):
        d = t.min_separation(i * 0.1)
        if d < worst:
            worst, worst_t = d, i * 0.1
    assert worst > 3.0, f"vehicles came within {worst:.1f} m at t={worst_t:.1f}s"


def test_phases_are_spread_around_the_lap():
    phases = [v.phase_frac for v in _traffic(n_background=3).fleet]
    assert len(set(phases)) == len(phases), f"repeated phases: {phases}"


def test_every_distractor_is_still_moving_late_in_a_long_flight():
    """A parked distractor stops being a distractor exactly when the selection
    problem should be hardest."""
    t = _traffic(n_background=3)
    for vs, car in zip(t.fleet, t.cars):
        if vs.is_target:
            continue                     # the target parks; that is the demo
        assert car.pose_at(110.0)[:2] != car.pose_at(100.0)[:2], (
            f"{vs.name} is stationary scenery by t=100 s"
        )


# ------------------------------------------------------------------ geometry

def test_no_distractor_crosses_the_stretch_the_target_drives():
    """A distractor sweeping through x=38 between y=-8 and y=58 occludes the
    very vehicle the aircraft is trying to select. The circuit's cross-legs are
    pushed 8 m beyond each end of the target's route for exactly this reason."""
    t = _traffic(n_background=3)
    for vs, car in zip(t.fleet, t.cars):
        if vs.is_target:
            continue
        for i in range(2000):
            x, y = car.pose_at(i * 0.1)[:2]
            if ct.ROUTE_Y0 <= y <= ct.ROUTE_Y1:
                assert abs(x - ct.TARGET_LANE_X) >= 3.0, (
                    f"{vs.name} reached ({x:.1f},{y:.1f}), {abs(x - 38):.1f} m "
                    "from the target lane on the stretch the target drives"
                )


def test_a_distractor_never_comes_close_to_the_target_itself():
    t = _traffic(n_background=3)
    tgt = t.cars[0]
    worst = min(math.dist(tgt.pose_at(i * 0.1)[:2], car.pose_at(i * 0.1)[:2])
                for i in range(1400) for car in t.cars[1:])
    assert worst > 3.0, f"a distractor passed {worst:.1f} m from the target"


def test_everything_stays_inside_the_clearance_envelope():
    """The obstacle map holds buildings only -- a flight at x=48 hit street
    furniture it cannot see."""
    t = _traffic(n_background=3)
    for vs, car in zip(t.fleet, t.cars):
        xs = [car.pose_at(i * 0.5)[0] for i in range(400)]
        assert 32.0 <= min(xs) and max(xs) <= 46.0, (
            f"{vs.name} spans x {min(xs):.1f}..{max(xs):.1f}, outside [32,46]"
        )


def test_the_target_drives_a_straight_run_and_distractors_a_closed_lap():
    fleet = _traffic(n_background=3).fleet
    assert len(fleet[0].route()) == 2, "the target should not loop; it parks"
    for vs in fleet[1:]:
        assert vs.route() == ct.circuit(), f"{vs.name} is not on the shared circuit"


# --------------------------------------------------------------- rpc budget

def test_the_target_updates_every_tick():
    assert _traffic(n_background=3).fleet[0].update_every == 1


def test_the_rpc_bill_stays_sane():
    t = _traffic(n_background=3)
    assert t.rpc_per_second(10.0) <= 45.0, t.rpc_per_second(10.0)


def test_a_skipped_update_costs_no_accuracy():
    """pose_at is a pure function of time, so staggering must not drift."""
    t = _traffic(n_background=3, bg_every=3)
    car = t.cars[1]
    stepped = [car.pose_at(i * 0.1)[:2] for i in range(300)]
    sparse = [car.pose_at(i * 0.1)[:2] for i in range(0, 300, 3)]
    for k, q in enumerate(sparse):
        assert stepped[k * 3] == q, "pose_at is not time-pure"


def test_update_stagger_skips_the_rpc_but_keeps_truth_current():
    class _CountingWorld(_FakeWorld):
        def __init__(self):
            self.n = 0

        def set_object_pose(self, *a, **k):
            self.n += 1

    w = _CountingWorld()
    t = ct.Traffic(w, fleet=ct.default_fleet(n_background=3, bg_every=3))
    for c in t.cars:
        c.actual_name = c.name
    for tick in range(30):
        t.update(tick * 0.1, tick)
    assert t.n_skipped > 0, "nothing was staggered"
    assert 30 <= w.n <= 30 + 3 * 10
    for vs, car in zip(t.fleet, t.cars):
        want = car.pose_at(29 * 0.1)[:2]
        assert math.dist(car.pos, want) < 1e-6, f"{vs.name} truth went stale"


def test_summary_records_the_colour_ground_truth():
    """Without this an experiment cannot be checked after the fact."""
    t = _traffic(n_background=3)
    out = t.summary()
    assert out["n_vehicles"] == 4
    for v in out["vehicles"]:
        for k in ("name", "is_target", "material", "colour_word", "speed_mps", "asset"):
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
