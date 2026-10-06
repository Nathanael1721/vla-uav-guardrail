"""Pedestrian routes and the kerb-side state machine: tools/citylife_peds.py.

Run either way:
    pytest tests/test_citylife_peds.py -v
    python tests/test_citylife_peds.py

The level's figures used to roam to random navmesh points, so they idled ON
zebras and stepped out in front of cars already inside their braking distance
(docs/FINDING-crowd-pedestrians-and-traffic.md). These tests pin what replaces
that: a route never crosses a carriageway except kerb to opposite kerb at a
crossing, and a figure never starts across on don't-walk or with a car under
4 s away, never stands still on the zebra, and does cross once it is clear.

The street mask (demo/out/citymap_citylife/street.npz) is gitignored. Every
test here holds with or without it; the mask-only check does nothing when the
file is absent.

Frames: UE centimetres, X = north, Y = east (tools/citylife_routes.py).
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import citylife_peds as P                                    # noqa: E402
import citylife_routes as R                                  # noqa: E402

_G = {}


def _graph():
    if "g" not in _G:
        _G["g"] = P.build_graph()
    return _G["g"]


def _tours():
    if "t" not in _G:
        _G["t"] = P.plan_tours(_graph(), 40, 40, P.SEED)
    return _G["t"]


def _samples(a, b, step=20.0):
    n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
    return [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n + 1)]


# ------------------------------------------------------------------ the graph

def test_the_seven_level_crossings_five_usable():
    """(3000, -4100) and (5200, -4100) cross the Y = -4100 street, whose far
    kerb (Y = -5100) is outside the nav bounds (Y >= -5000)."""
    for g in (_graph(), P.build_graph(mask=None)):
        assert [c.id for c in g.crossings] == [0, 1, 2, 5, 6], [c.id for c in g.crossings]
        bad = g.report["unusable_crossings"]
        assert set(bad) == {(3000.0, -4100.0), (5200.0, -4100.0)}, bad
        assert all("outside the pedestrian area" in v for v in bad.values()), bad
        assert sum(1 for k in g.xing_at if k >= 0) == 10


def test_every_crossing_edge_joins_the_two_kerbs_of_one_crossing():
    g = _graph()
    seen = set()
    for i, j in g.edges():
        if not g.is_crossing_edge(i, j):
            continue
        c = g.crossing(g.xing_at[i])
        seen.add(c.id)
        (ai, bi), (aj, bj) = c.local(*g.pos[i]), c.local(*g.pos[j])
        assert ai * aj < 0, f"crossing {c.id}: both ends on one side ({ai}, {aj})"
        for a, b in ((ai, bi), (aj, bj)):
            assert P.KERB_OFF_CM - 60 <= abs(a) <= P.KERB_OFF_CM + 110, (c.id, a)
            assert abs(b) <= 110, (c.id, b)
        assert {g.pos[i], g.pos[j]} == {c.kerb_a, c.kerb_b}
    assert seen == {c.id for c in g.crossings}


def test_no_edge_crosses_a_carriageway_except_at_a_crossing():
    g = _graph()
    for i, j in g.edges():
        pts = _samples(g.pos[i], g.pos[j])
        if not g.is_crossing_edge(i, j):
            hit = [p for p in pts if P.in_carriageway(p[0], p[1], g.xs, g.ys)]
            assert not hit, f"edge {g.pos[i]}->{g.pos[j]} enters a carriageway at {hit[0]}"
            continue
        c = g.crossing(g.xing_at[i])
        own = (c.x,) if c.axis == 0 else (c.y,)
        others_x = tuple(v for v in g.xs if c.axis != 0 or v != c.x)
        others_y = tuple(v for v in g.ys if c.axis != 1 or v != c.y)
        for p in pts:
            _, along = c.local(*p)
            assert abs(along) <= P.ZEBRA_HALF_CM - P.AGENT_RADIUS_CM, (c.id, p)
            assert not P.in_carriageway(p[0], p[1], others_x, others_y), \
                f"crossing {c.id} also cuts another street at {p} (own {own})"


def test_the_crossing_network_is_one_connected_graph():
    """Everything reachable through the level's crossings is one component:
    the four blocks round junction (4100, 4100). BFS from one kerb reaches every
    node of it and every kerb."""
    g = _graph()
    nets = g.crossing_components()
    assert len(nets) == 1, nets
    main = [i for i in range(len(g)) if g.comp[i] == nets[0]]
    seen, stack = {main[0]}, [main[0]]
    while stack:
        u = stack.pop()
        for v in g.adj[u]:
            if v not in seen:
                seen.add(v)
                stack.append(v)
    assert seen == set(main)
    assert all(g.comp[i] == nets[0] for i in range(len(g)) if g.xing_at[i] >= 0)
    assert len({g.block[i] for i in main}) == 4
    assert all(g.adj[i] for i in range(len(g))), "an isolated node survived"
    for c, nodes in g.components().items():          # every component is itself whole
        seen, stack = {nodes[0]}, [nodes[0]]
        while stack:
            u = stack.pop()
            for v in g.adj[u]:
                if v not in seen:
                    seen.add(v)
                    stack.append(v)
        assert seen == set(nodes), c


def test_islands_are_the_level_not_the_code():
    """A crossing added over the X = 12300 street joins the block north of it
    to the network with no other change: the islands exist only because the
    level has no crossing there."""
    before = P.build_graph(mask=None)
    after = P.build_graph(mask=None, crossings=list(R.CROSSINGS) + [(12300.0, 0.0)])
    nb = len([i for i in range(len(before)) if before.comp[i] in before.crossing_components()])
    na = len([i for i in range(len(after)) if after.comp[i] in after.crossing_components()])
    assert len(after.crossing_components()) == 1
    assert 7 in [c.id for c in after.crossings]
    assert na > nb + 10, (nb, na)


def test_every_node_is_in_the_area_and_on_a_pavement():
    for g in (_graph(), P.build_graph(mask=None)):
        for (x, y) in g.pos:
            assert P.in_area(x, y, g.area), (x, y)
            assert not P.in_carriageway(x, y, g.xs, g.ys), (x, y)
            assert all(abs(x - c) >= P.PAVEMENT_MIN_CM for c in g.xs), (x, y)
            assert all(abs(y - c) >= P.PAVEMENT_MIN_CM for c in g.ys), (x, y)


def test_every_node_is_on_a_street_cell_of_the_mask():
    mask = P.load_mask()
    if mask is None:
        print("      (street mask not built here: nothing to check)")
        return
    g = _graph()
    assert g.report["mask"] is True
    off = [p for p in g.pos if not P.on_street(mask, p[0], p[1])]
    assert not off, off[:5]


# ------------------------------------------------------------------ routes

def test_routes_are_deterministic_per_seed():
    g = _graph()
    a = P.make_routes(40, 40, seed=7, graph=g)
    b = P.make_routes(40, 40, seed=7, graph=g)
    c = P.make_routes(40, 40, seed=8, graph=g)
    assert a == b
    assert a != c
    assert len(a) == 40 and all(len(r) >= 40 for r in a)


def test_tours_are_closed_walks_with_no_immediate_backtracking():
    """Consecutive nodes - including last to first - are graph neighbours, and
    a tour turns straight back only at a dead end, where nothing else exists."""
    g = _graph()
    for t in _tours():
        n = len(t)
        for i in range(n):
            a, b = t[i], t[(i + 1) % n]
            assert b in g.adj[a], f"{g.pos[a]} -> {g.pos[b]} is not an edge"
            if t[i - 1] == b:
                assert len(g.adj[a]) == 1, f"U-turn at {g.pos[a]}, degree {len(g.adj[a])}"


def test_every_crossing_leg_goes_kerb_to_opposite_kerb():
    g = _graph()
    by_kerbs = {}
    for c in g.crossings:
        by_kerbs[(c.kerb_a, c.kerb_b)] = c
        by_kerbs[(c.kerb_b, c.kerb_a)] = c
    for r in P.make_routes(40, 40, graph=g):
        n = len(r)
        for i in range(n):
            (x0, y0, k0), (x1, y1, k1) = r[i], r[(i + 1) % n]
            wet = any(P.in_carriageway(p[0], p[1], g.xs, g.ys)
                      for p in _samples((x0, y0), (x1, y1)))
            if not wet:
                assert (k0, k1) != (P.KIND_KERB_WAIT, P.KIND_CROSS_EXIT), r[i]
                continue
            assert (k0, k1) == (P.KIND_KERB_WAIT, P.KIND_CROSS_EXIT), (r[i], r[(i + 1) % n])
            c = by_kerbs.get(((x0, y0), (x1, y1)))
            assert c is not None, f"leg {(x0, y0)}->{(x1, y1)} is not kerb to kerb"
            assert c.local(x0, y0)[0] * c.local(x1, y1)[0] < 0


def test_kind_codes_pair_up():
    for r in P.make_routes(40, 40, graph=_graph()):
        n = len(r)
        for i in range(n):
            k = r[i][2]
            if k == P.KIND_KERB_WAIT:
                assert r[(i + 1) % n][2] == P.KIND_CROSS_EXIT
            if k == P.KIND_CROSS_EXIT:
                assert r[i - 1][2] == P.KIND_KERB_WAIT


def test_network_routes_cross_and_together_use_every_crossing():
    g = _graph()
    net = g.crossing_components()[0]
    used = set()
    on_net = 0
    for t in _tours():
        legs = [(t[i], t[(i + 1) % len(t)]) for i in range(len(t))]
        xs = [g.xing_at[a] for a, b in legs if g.is_crossing_edge(a, b)]
        if g.comp[t[0]] == net:
            on_net += 1
            assert xs, "a tour on the crossing network never crosses"
        else:
            assert not xs
        used.update(xs)
    assert on_net >= 10, on_net
    assert used == {c.id for c in g.crossings}, used


def test_nobody_is_placed_where_a_tour_can_only_pace():
    """A component with no cycle (the 63 m runs along the far edges, dead ends
    at both ends) admits only a back-and-forth tour - the old roaming look.
    Nobody is routed there; on a block ring a tour never turns round, and over
    all tours a turn-round (only ever at a network stub's dead end) is rare."""
    g = _graph()
    comps = g.components()
    pacing = [c for c, v in comps.items() if sum(len(g.adj[i]) for i in v) // 2 < len(v)]
    assert pacing, "the level has path-shaped components; the test would prove nothing"
    net = g.crossing_components()[0]
    uturns = legs = 0
    for t in _tours():
        c = g.comp[t[0]]
        assert c not in pacing, f"a tour on the path-shaped component {c}"
        n = len(t)
        u = sum(1 for i in range(n) if t[i - 1] == t[(i + 1) % n])
        if c != net:
            assert u == 0, f"a ring tour turned round {u} times"
        uturns += u
        legs += n
    assert uturns < 0.05 * legs, (uturns, legs)


def test_empty_and_off_map_inputs():
    g = _graph()
    assert P.plan_tours(g, 0, 40) == [] and P.make_routes(0, graph=g) == []
    off = P.build_graph(area=(100000.0, 101000.0, 100000.0, 101000.0), mask=None)
    assert len(off) == 0 and off.crossings == [] and off.edges() == []
    assert len(off.report["unusable_crossings"]) == len(R.CROSSINGS)
    assert P.make_routes(40, graph=off) == []
    assert P.route_stats(off, [])["routes"] == 0
    bare = P.build_graph(crossings=[], mask=None)
    assert bare.crossings == [] and bare.crossing_components() == []
    assert all(k == P.KIND_PAVEMENT for r in P.make_routes(10, graph=bare) for _, _, k in r)
    for bad in ([(0.0, 0.0, 0)],                                 # one point
                [(3150.0, 0.0, 0), (3100.0, -3000.0, 1), (3150.0, 500.0, 0)]):  # wait, no exit
        try:
            P.PedModel(bad, g.crossings)
            raise AssertionError(f"accepted {bad}")
        except ValueError:
            pass
    ped = P.PedModel(_xing_route(g), g.crossings, walk_ok=None, start=1)
    ped.step(0.1, 0.1, [])                                       # no cars at all
    assert ped.state == P.CROSS


def test_route_points_stay_in_the_area_off_the_carriageway():
    g = _graph()
    for r in P.make_routes(40, 40, graph=g):
        for x, y, _ in r:
            assert P.in_area(x, y, g.area)
            assert not P.in_carriageway(x, y, g.xs, g.ys)


# ------------------------------------------------------------------ the state machine

def _xing_route(g):
    """South pavement -> across crossing 0 -> north pavement -> back across
    crossing 1 -> south pavement. Index 1 is crossing 0's south kerb."""
    c0, c1 = g.crossing(0), g.crossing(1)
    return [(3150.0, 0.0, 0), (c0.kerb_a[0], c0.kerb_a[1], 1), (c0.kerb_b[0], c0.kerb_b[1], 2),
            (5050.0, 0.0, 0), (c1.kerb_b[0], c1.kerb_b[1], 1), (c1.kerb_a[0], c1.kerb_a[1], 2)]


def _run(ped, seconds, cars_at=lambda t: (), dt=0.1, each=None):
    t = 0.0
    for _ in range(int(round(seconds / dt))):
        t += dt
        cars = cars_at(t)
        ped.step(t, dt, cars)
        if each:
            each(t, ped, cars)


def _car_a(s):
    p = R.build_all()["A"]
    x, y, yaw = P.path_point(p, s)
    return p, x, y, yaw


def test_never_crosses_on_dont_walk():
    g = _graph()
    ped = P.PedModel(_xing_route(g), g.crossings, walk_ok=lambda t: False, seed=1)
    wet = []
    _run(ped, 200.0, each=lambda t, p, c: wet.append(t) if P.in_carriageway(
        p.x, p.y, g.xs, g.ys) else None)
    assert ped.cross_log == [], ped.cross_log
    assert ped.state == P.WAIT
    assert not wet, f"on the carriageway at t={wet[0]}"
    assert ped.counters.kerb_wait_max_s > 150.0


def test_does_not_start_with_a_car_two_seconds_away_then_crosses_when_clear():
    g = _graph()
    c0 = g.crossing(0)
    path = R.build_all()["A"]
    s_x = P.crossing_s(path, c0)
    assert s_x is not None, "loop A drives over crossing 0"
    v = 500.0
    reach = P.ZEBRA_HALF_CM + P.CAR_LEN_CM / 2
    s0 = s_x - reach - 2.0 * v                       # 2 s from the zebra at t = 0

    def cars(t):
        s = s0 + v * t
        x, y, yaw = P.path_point(path, s)
        return [P.CarView(x, y, yaw, v, path, s)]

    assert abs(P.time_to_zebra(cars(0.0)[0], c0) - 2.0) < 1e-6
    ped = P.PedModel(_xing_route(g), g.crossings, walk_ok=None, start=1, seed=2)
    assert ped.state == P.WAIT
    _run(ped, 20.0, cars)
    assert ped.cross_log, "never crossed once the car had gone"
    t_go, xid, walk, tta = ped.cross_log[0]
    t_clear = (s_x + reach - s0) / v                 # the car's tail leaves the zebra
    assert xid == 0 and walk and tta >= P.GAP_S, ped.cross_log[0]
    assert t_go >= t_clear - 1e-6, (t_go, t_clear)
    assert t_go <= t_clear + 0.5, (t_go, t_clear)
    assert ped.counters.gap_viol == 0 and ped.counters.walk_viol == 0


def test_never_starts_into_a_stream_of_cars_two_seconds_apart():
    g = _graph()
    c0 = g.crossing(0)
    path = R.build_all()["A"]
    v, head = 500.0, 2.0
    total = P.path_arc(path)[1]
    n = int(total // (v * head))                     # the whole loop, 10 m apart

    def cars(t):
        out = []
        for k in range(n):
            s = (v * t - k * v * head) % total
            x, y, yaw = P.path_point(path, s)
            out.append(P.CarView(x, y, yaw, v, path, s))
        return out

    worst = []
    ped = P.PedModel(_xing_route(g), g.crossings, walk_ok=None, start=1, seed=3)
    _run(ped, 120.0, cars, each=lambda t, p, cs: worst.append(
        min(P.time_to_zebra(c, c0) for c in cs)))
    assert max(worst) < P.GAP_S, "the stream left a 4 s gap; the test proves nothing"
    assert ped.cross_log == [], ped.cross_log


def test_a_moving_car_on_the_zebra_blocks_a_standing_one_does_not():
    g = _graph()
    c0 = g.crossing(0)
    ped = P.PedModel(_xing_route(g), g.crossings, walk_ok=None, start=1)
    on = P.CarView(c0.x + 350.0, c0.y, 90.0, 300.0)          # eastbound lane, on the zebra
    go, walk, tta, busy = ped.clear_to_cross(0.0, c0, [on])
    assert busy and not go and tta == 0.0
    parked = P.CarView(c0.x + 350.0, c0.y, 90.0, 0.0)
    go, walk, tta, busy = ped.clear_to_cross(0.0, c0, [parked])
    assert go and not busy and tta == math.inf


def test_never_idles_on_the_zebra_even_with_a_car_parked_across_it():
    """Once across it keeps walking; a car standing in its way is walked
    through (and counted), never waited for on the painted stripes."""
    g = _graph()
    c0 = g.crossing(0)
    parked = [P.CarView(c0.x - 350.0, c0.y, 90.0, 0.0)]   # westbound lane, on the line
    slow = []

    def watch(t, p, cars):
        if c0.in_zebra(p.x, p.y) and p.v < P.STILL_PED_CMS:
            slow.append(t)

    ped = P.PedModel(_xing_route(g), g.crossings, walk_ok=None, start=1, seed=4)
    _run(ped, 30.0, lambda t: parked, each=watch)
    assert ped.cross_log and ped.state != P.WAIT
    assert not slow, f"stood on the zebra at t={slow[0]}"
    assert ped.counters.zebra_idle_s == 0.0
    assert ped.counters.car_overlap >= 1, "the car across its line was not counted"


def test_the_counters_fire_when_the_checks_are_off():
    """checks=False crosses on arrival: the same two situations must then be
    counted, or a zero from the checked model would mean nothing."""
    g = _graph()
    c0 = g.crossing(0)
    path = R.build_all()["A"]
    s_x = P.crossing_s(path, c0)
    v = 500.0
    s0 = s_x - P.ZEBRA_HALF_CM - P.CAR_LEN_CM / 2 - 2.0 * v

    def cars(t):
        s = s0 + v * t
        x, y, yaw = P.path_point(path, s)
        return [P.CarView(x, y, yaw, v, path, s)]

    ped = P.PedModel(_xing_route(g), g.crossings, walk_ok=lambda t: False, start=1,
                     checks=False)
    _run(ped, 3.0, cars)
    assert ped.counters.walk_viol == 1 and ped.counters.gap_viol == 1, ped.counters


def test_the_crossing_is_walked_faster_but_capped():
    g = _graph()
    assert P.PedModel(_xing_route(g), g.crossings, speed_cms=140.0).cross_speed == 175.0
    assert P.PedModel(_xing_route(g), g.crossings, speed_cms=154.0).cross_speed == 180.0


def test_time_to_zebra_along_the_path_and_along_the_heading_agree_on_a_straight():
    """Crossing 1 (4100, 3000) sits on loop A's eastbound straight. (Crossing 0
    does not: A's corner arc at (4100, -4100) ends 50 cm before it, which is
    exactly why the time is measured along the path when the path is known.)"""
    g = _graph()
    c1 = g.crossing(1)
    path = R.build_all()["A"]
    s_x = P.crossing_s(path, c1)
    v = 400.0
    reach = P.ZEBRA_HALF_CM + P.CAR_LEN_CM / 2
    s = s_x - reach - 1200.0                         # 3 s out, on the straight
    x, y, yaw = P.path_point(path, s)
    assert abs(yaw - 90.0) < 1e-6, yaw               # heading east
    on_path = P.time_to_zebra(P.CarView(x, y, yaw, v, path, s), c1)
    by_heading = P.time_to_zebra(P.CarView(x, y, yaw, v), c1)
    assert abs(on_path - 3.0) < 1e-6, on_path
    assert abs(by_heading - 3.0) < 0.2, by_heading
    x, y, yaw = P.path_point(path, s_x + reach + 50.0)       # just past it
    assert P.time_to_zebra(P.CarView(x, y, yaw, v, path, s_x + reach + 50.0), c1) == math.inf
    loop_c = R.build_all()["C"]                              # never drives over it
    x, y, yaw = P.path_point(loop_c, 0.0)
    assert P.time_to_zebra(P.CarView(x, y, yaw, v, loop_c, 0.0), c1) == math.inf


def test_signals_resolve_by_id_or_centre_and_the_stand_in_alternates():
    g = _graph()
    c0, c5 = g.crossing(0), g.crossing(5)
    assert P.walk_fn({0: lambda t: False}, c0)(0.0) is False
    assert P.walk_fn({(c0.x, c0.y): lambda t: False}, c0)(0.0) is False
    try:
        P.walk_fn({1: lambda t: True}, c0)
        raise AssertionError("a missing signal must not default to walk")
    except KeyError:
        pass
    sig = P.fixed_time_signals(g.crossings)
    for t in range(0, 120):
        assert not (sig[0](t) and sig[5](t)), t      # the two directions never walk together
    assert any(sig[0](t) for t in range(60)) and any(sig[5](t) for t in range(60))
    assert c0.axis != c5.axis


def test_straightness_is_one_on_a_line_and_near_zero_pacing():
    c = P.PedCounters()
    for k in range(0, 1201):
        c.observe(k * 0.1, 0.1, 140.0 * k * 0.1, 0.0, 140.0, (), ())
    st = c.straightness()
    assert len(st) == 2 and all(abs(r - 1.0) < 1e-9 for r in st), st
    c = P.PedCounters()
    for k in range(0, 1201):
        t = k * 0.1
        x = 140.0 * (t % 30.0) if (t // 30.0) % 2 == 0 else 140.0 * (30.0 - t % 30.0)
        c.observe(t, 0.1, x, 0.0, 140.0, (), ())
    st = c.straightness()
    assert len(st) == 2 and all(r < 0.05 for r in st), st


def test_a_short_simulated_run_is_clean_and_the_blind_one_is_not():
    g = _graph()
    ok = P.simulate(120.0, graph=g)["counters"]
    assert ok["WalkViol"] == 0 and ok["GapViol"] == 0, ok
    assert ok["ZebraIdleS"] == 0.0 and ok["CarOverlap"] == 0, ok
    assert ok["Crossings"] > 0, ok
    blind = P.simulate(120.0, graph=g, checks=False)["counters"]
    assert blind["WalkViol"] > 0, blind


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
