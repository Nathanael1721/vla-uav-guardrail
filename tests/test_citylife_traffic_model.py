"""The CityLife traffic with signals: tools/citylife_traffic_model.py.

Run either way:
    pytest tests/test_citylife_traffic_model.py -v
    python tests/test_citylife_traffic_model.py

The model is the reference the Blueprint DSL will mirror, so these checks pin
two things: the rules, one situation at a time (a car at its line on red, on
yellow, behind its own platoon leader in a turn, behind a stopped car on its
exit, a turner that waited out the green), and a short run of all 24 cars
against the gates - no red run, nobody waiting on a zebra, no two cars closer
than 6 m, nobody stuck. Every zero is also shown to be able to be non-zero:
the red detector fires on a car driven through a red, and today's logic
(legacy mode) does put waiting cars on the zebra.
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools import citylife_signals as S                 # noqa: E402
from tools import citylife_traffic_model as T           # noqa: E402
from tools.citylife_mcp.apply_routes import plan        # noqa: E402

DATA = plan()
A_J = (4100.0, -4100.0)          # loop A comes SOUTH down lane y = -3750 and turns left (east)
T_RED_NS = 10.0                  # (0, -1) has offset 27: NS red over [0, 27), green [27, 47)
T_GREEN_NS = 30.0
T_YELLOW_NS = 27.0 + 21.0


def _car(name, x, y, yaw, v=0.0):
    """A car from the level's plan, moved to (x, y) facing yaw, ready to update."""
    loops = T.build_loops(DATA, frozenset())
    car = next(c for c in T.build_cars(DATA, loops) if c.name == name)
    L = car.loop
    i = min(range(L.n), key=lambda k: math.hypot(L.path.pts[k][0] - x, L.path.pts[k][1] - y))
    car.st.x, car.st.y, car.st.yaw_deg, car.st.v, car.st.idx = x, y, yaw, v, (i + 1) % L.n
    car.ticks = 1                                  # the next update_eff is an even tick: it runs
    car.dt = 0.1
    T._set_fwd(car)
    return car


def _at_line(extra=0.0, v=0.0, name="Car_10"):
    """A loop-A car on the approach to (4100, -4100), centre STOP_CM + extra out."""
    return _car(name, A_J[0] + T.STOP_CM + extra, -3750.0, 180.0, v)


def test_the_stop_line_is_behind_the_zebra_and_the_old_target_is_on_it():
    assert T.STOP_CM == 1730.0
    assert T.STOP_CM - T.CAR_HALF_LEN_CM == T.ZEBRA_OUTER_CM + T.LINE_GAP_CM   # front 100 cm short
    at_line = _at_line()
    assert not T._on_zebra(at_line)
    old = _car("Car_10", A_J[0] + T.BOX_HALF_CM + 300.0, -3750.0, 180.0)      # JEdge - 300
    assert T._on_zebra(old), "the legacy target must be ON the zebra - problem (b)"


def test_red_holds_at_the_line_and_green_lets_go():
    assert S.state(T_RED_NS, *A_J, S.NS) == "red" and S.state(T_GREEN_NS, *A_J, S.NS) == "green"
    car = _at_line(300.0)
    T.update_eff(car, T_RED_NS, [car], [], "signals")
    assert car.why == "red" and abs(car.stop - 300.0) < 1.0, (car.why, car.stop)
    car = _at_line(300.0)
    T.update_eff(car, T_GREEN_NS, [car], [], "signals")
    assert car.why == "" and car.stop > 90000.0
    car = _at_line(-60.0, v=250.0)                              # 60 cm past its line: committed
    T.update_eff(car, T_RED_NS, [car], [], "signals")
    assert car.why != "red"


def test_yellow_stops_a_car_that_can_and_not_one_that_cannot():
    assert S.state(T_YELLOW_NS, *A_J, S.NS) == "yellow"
    far = _at_line(300.0, v=320.0)                              # 320^2/500 - 50 = 155 < 300
    T.update_eff(far, T_YELLOW_NS, [far], [], "signals")
    assert far.why == "yellow"
    near = _at_line(100.0, v=320.0)
    T.update_eff(near, T_YELLOW_NS, [near], [], "signals")
    assert near.why == ""


def test_my_platoon_leader_turning_in_the_box_is_not_crossing_traffic():
    follower = _at_line()
    # 60 deg round A's 7 m left turn (centre (5150, -3050)): past the 45.6 deg
    # at which the old rule's par < 0.7 starts calling it crossing traffic
    th = math.radians(60.0)
    leader = _car("Car_11", 5150.0 - 700.0 * math.sin(th), -3050.0 - 700.0 * math.cos(th), 120.0, 300.0)
    T.update_eff(follower, T_GREEN_NS, [follower, leader], [], "signals")
    assert follower.why not in ("box", "late"), follower.why
    follower = _at_line()
    crossing = _car("Car_06", 3750.0, -3600.0, 270.0, 300.0)    # loop B westbound, in the box
    T.update_eff(follower, T_GREEN_NS, [follower, crossing], [], "signals")
    assert follower.why == "box"
    follower = _at_line()                                       # legacy: the leader DOES hold it
    T.update_eff(follower, T_GREEN_NS, [follower, leader], [], "legacy")
    assert follower.why == "box" and abs(follower.stop - (T.STOP_CM - T.BOX_HALF_CM - 300.0)) < 1.0


def test_a_stopped_car_on_my_exit_holds_me_at_the_line_a_moving_one_does_not():
    ahead = T.BOX_HALF_CM + 700.0                               # 18 m past the centre < 2280
    for v, want in ((0.0, "exit"), (300.0, "")):
        me = _at_line()
        other = _car("Car_11", 4450.0, A_J[1] + ahead, 90.0, v)   # on A's exit lane, eastbound
        T.update_eff(me, T_GREEN_NS, [me, other], [], "signals")
        assert me.why == want, (v, me.why)
    me = _at_line()                                             # the spec-literal variant counts it
    other = _car("Car_11", 4450.0, A_J[1] + ahead, 90.0, 300.0)
    T.update_eff(me, T_GREEN_NS, [me, other], [], "signals", box_rule="static")
    assert me.why == "exit"


def test_a_turner_that_waited_out_the_green_goes_on_early_yellow_unless_oncoming_cannot_stop():
    J = (12300.0, -4100.0)                                      # loop B gives way here (z = 1)
    t = S.offset(*J) + S.GREEN_S + 0.5
    assert S.state(t, *J, S.NS) == "yellow"

    def turner(held):
        c = _car("Car_06", J[0] - T.STOP_CM, -4450.0, 0.0)     # B northbound, at its line
        c.held_turn = held
        return c
    b = turner(0.0)
    T.update_eff(b, t, [b], [], "signals")
    assert b.why == "yellow"
    b = turner(T.SNEAK_HELD_S + 1.0)
    T.update_eff(b, t, [b], [], "signals")
    assert b.why == "" and b.counters.get("sneaks") == 1 and b.sneak_j >= 0
    b = turner(T.SNEAK_HELD_S + 1.0)
    fast = _car("Car_09", J[0] + T.STOP_CM + 100.0, -3750.0, 180.0, 450.0)   # A, can't stop
    T.update_eff(b, t, [b, fast], [], "signals")
    assert b.why == "yellow" and not b.counters.get("sneaks")


def test_a_sneak_held_again_before_its_line_does_not_carry_into_the_red():
    """Yellow: the turner sneaks; an oncoming car that cannot stop holds it at
    its line; the light turns red and that car has gone. A latched sneak
    skipped the red rule here and drove through the red from 20 cm short."""
    J = (12300.0, -4100.0)
    off = S.offset(*J)
    b = _car("Car_06", J[0] - T.STOP_CM - 20.0, -4450.0, 0.0)
    b.held_turn = T.SNEAK_HELD_S + 1.0

    def upd(t, others):
        b.ticks = 1                                             # every call is an update tick
        T.update_eff(b, t, [b] + others, [], "signals")
        return b.why
    assert S.state(off + 20.5, *J, S.NS) == "yellow" and S.state(off + 23.5, *J, S.NS) == "red"
    assert upd(off + 20.5, []) == "" and b.sneak_j >= 0         # sneaks
    fast = _car("Car_09", J[0] + T.STOP_CM + 100.0, -3750.0, 180.0, 450.0)
    assert upd(off + 21.0, [fast]) == "giveway"                 # held at the line again
    assert b.sneak_j == -1
    assert upd(off + 23.5, []) == "red" and abs(b.stop - 20.0) < 1.0


def test_the_red_detector_fires_on_a_car_driven_through_a_red():
    car = _at_line(70.0, v=300.0)
    car.line_k = 0                                              # A's junction 0 is (4100, -4100)
    assert (car.loop.jinfo[0].x, car.loop.jinfo[0].y) == A_J
    car.stop = car.gap = 99999.0
    car.eff = 320.0
    M = T.Metrics(signals=True)
    T.drive(car, 0.5, T_RED_NS, M)                              # 1.5 m on, no rule holding it
    assert M.red_viol == 1, M.red_viol
    M = T.Metrics(signals=True)
    car = _at_line(70.0, v=300.0)
    car.line_k, car.stop, car.gap, car.eff = 0, 99999.0, 99999.0, 320.0
    T.drive(car, 0.5, T_GREEN_NS, M)
    assert M.red_viol == 0


def test_jexit_rows_point_from_each_junction_to_the_next():
    rows = T.jexit_rows(DATA)
    for k, L in DATA["loops"].items():
        js = L["junctions"]
        assert len(rows[k]) == len(js)
        for i, (ex, ey, yaw) in enumerate(rows[k]):
            nx, ny = js[(i + 1) % len(js)][0] - js[i][0], js[(i + 1) % len(js)][1] - js[i][1]
            assert abs(math.degrees(math.atan2(ny, nx)) - yaw) < 1e-6, (k, i)
            assert abs(math.hypot(ex - js[i][0], ey - js[i][1]) - 350.0) < 1e-6   # the lane offset
            # keep-left: the exit lane is LEFT of the exit heading (X north, Y
            # east, so the left of (fx, fy) is (fy, -fx)); a right-side point
            # would put the rule on the oncoming lane
            fx, fy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            ox, oy = ex - js[i][0], ey - js[i][1]
            assert ox * fy - oy * fx > 349.0, (k, i, ox, oy, yaw)


def test_a_run_of_no_frames_or_an_unknown_rule_is_refused():
    for kw in ({"minutes": 0.0}, {"fps": 0.0}, {"fps": -10.0}, {"mode": "x"}, {"box_rule": "x"}):
        args = {"minutes": 1.0, "fps": 10.0, "data": DATA}
        args.update(kw)
        try:
            T.simulate(**args)
            assert False, kw
        except ValueError:
            pass


def test_a_short_run_meets_every_gate():
    r = T.simulate(minutes=5.0, fps=10.0, mode="signals", data=DATA)
    assert r["red_viol"] == 0, r["red_viol_log"]
    assert r["zebra_wait_s"] == 0.0, r["zebra_wait_log"]
    assert r["min_gap_cm"] >= 600.0, (r["min_gap_cm"], r["min_gap_at"])
    assert r["deadlocks"] == [], r["deadlocks"]
    assert r["ped_hits"] == 0
    # the zeros are not silence: the signals held cars, walkers crossed, cars yielded
    assert r["red_front_min_cm"] is not None and r["red_front_min_cm"] >= T.ZEBRA_OUTER_CM
    assert r["peds"] > 0 and r["counters"].get("ped_ticks", 0) > 0
    assert r["yellow_entries"] > 0
    assert len(r["car10_laps_s"]) >= 1


def test_a_short_run_at_3_fps_meets_every_gate():
    r = T.simulate(minutes=5.0, fps=3.0, mode="signals", data=DATA)
    assert r["red_viol"] == 0, r["red_viol_log"]
    assert r["zebra_wait_s"] == 0.0, r["zebra_wait_log"]
    assert r["min_gap_cm"] >= 600.0, (r["min_gap_cm"], r["min_gap_at"])
    assert r["deadlocks"] == []
    assert r["red_front_min_cm"] is not None and r["red_front_min_cm"] >= T.ZEBRA_OUTER_CM


def test_todays_logic_does_wait_on_the_zebra():
    r = T.simulate(minutes=5.0, fps=10.0, mode="legacy", data=DATA)
    assert r["zebra_wait_s"] > 0.0 and r["zebra_wait_events"] > 0, r
    assert r["red_viol"] == 0                                   # nothing judged: no signals


def test_a_run_is_deterministic():
    a = T.simulate(minutes=2.0, fps=10.0, mode="signals", data=DATA)
    b = T.simulate(minutes=2.0, fps=10.0, mode="signals", data=DATA)
    assert a == b


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
