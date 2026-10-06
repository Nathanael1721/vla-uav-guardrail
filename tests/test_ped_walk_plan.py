"""The walk test the pedestrians run is the walk phase of citylife_signals,
and their car test is citylife_peds.time_to_zebra.

tools/citylife_mcp/ped_walk.py writes, per crossing leg of each route, a
WOff the Blueprint turns into "may I start?" with one fmod; this holds that
equal to citylife_signals.walk() for the crossing's junction and road. The
car test reads each car's own path (Crossings, Idx, NPts): `car_blocks` is
that arithmetic in Python, held here to block every car on every loop that
the model puts under GAP_S from a usable crossing; and the rendered graph is
RUN (tests/test_drive_signals_dsl.py's evaluator) against it, and against
the stuck rule, which used to send a figure across without either test.

Run either way:
    pytest tests/test_ped_walk_plan.py -v
    python tests/test_ped_walk_plan.py
"""
import bisect
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from tools import citylife_peds as P                        # noqa: E402
from tools import citylife_routes as R                      # noqa: E402
from tools import citylife_signals as S                     # noqa: E402
from tools.citylife_mcp import drive_tick as DT             # noqa: E402
from tools.citylife_mcp import ped_walk as W                # noqa: E402
import test_drive_signals_dsl as DSL                        # noqa: E402  (parser, evaluator)

PLAN = W.plan()
PATHS = R.build_all()
SPEEDS = (60.0, 200.0, 320.0, 450.0, 580.0, 800.0)          # the level's cars run 320-580 cm/s


def test_forty_routes_and_every_crossing_leg_has_an_offset():
    assert len(PLAN) == 40
    n = 0
    for p in PLAN:
        r = p["route"]
        for i, q in enumerate(r):
            if int(q[2]) == P.KIND_CROSS_EXIT:
                n += 1
                assert int(r[i - 1][2]) == P.KIND_KERB_WAIT, (p["tag"], i)
    assert n >= 40


def test_the_blueprint_walk_test_equals_the_signal_plan():
    for p in PLAN:
        r = p["route"]
        for i, q in enumerate(r):
            if int(q[2]) != P.KIND_CROSS_EXIT:
                continue
            (kx, ky, _), (ex, ey, _) = r[i - 1], q
            cx, cy = 0.5 * (kx + ex), 0.5 * (ky + ey)
            jx, jy = S.nearest_junction(cx, cy)
            road = S.crossing_road_axis(cx, cy, jx, jy)
            for k in range(0, 400):
                t = 0.25 * k
                assert W.is_walk(t, p["woff"][i]) == S.walk(t, jx, jy, road), (p["tag"], i, t)


def test_the_rendered_graph_is_complete():
    src = W.render()
    assert "@" not in src and "{" not in src and "}" not in src and "__CAR__" not in src
    assert src.count("(") == src.count(")")
    forms = DSL.parse(src)                                   # raises on an unbalanced graph
    assert [f[:2] for f in forms] == [["event", "EventBeginPlay"],
                                      ["event", "Collision|EventActorBeginOverlap"],
                                      ["event", "EventTick"]], [f[:2] for f in forms]
    assert DSL.structure_errors(forms) == [], DSL.structure_errors(forms)[:5]
    assert "CastToBP_CityCar" in src and "GetCurSpeed" in src
    assert "(- (Variables|Default|GetCNp) 4)" in src         # an int, compared with an int


def test_every_variable_the_graph_uses_exists():
    """Its own: NEW_VARS or the ones the figure was found with (ped_roam_v1.dsl's
    header; bFoo is read as Foo). A car's: declared by drive_tick.py or used by
    its graphs. A misspelt name compiles to nothing, not to an error here."""
    src = W.render()
    roam = (W.HERE / "ped_roam_v1.dsl").read_text(encoding="utf-8")
    found = [v.strip() for v in re.search(r"; variables: (.*)", roam).group(1).split(",")]
    own = {v[1:] if re.match(r"b[A-Z]", v) else v for v in found} | {v[0] for v in W.NEW_VARS}
    assert DSL.var_names(src) <= own, sorted(DSL.var_names(src) - own)
    car_src = Path(DT.__file__).read_text(encoding="utf-8")
    car = (DSL.var_names(car_src) | set(re.findall(r"\{([A-Za-z][A-Za-z0-9_]*)\}", car_src))
           | {v[0] for v in DT.NEW_VARS})
    read = set(re.findall(r"Class\|BPCityCar\|Get([A-Za-z0-9_]+)", src))
    assert read == {"CurSpeed", "NPts", "Crossings", "Idx"}, read
    assert read <= car, sorted(read - car)


# ------------------------------------------------------------------ the car test

_G = {}


def _usable():
    """[(crossing, zebra centre as the Blueprint computes it: kerb/exit midpoint)]."""
    if "u" not in _G:
        _G["u"] = [(c, (0.5 * (c.kerb_a[0] + c.kerb_b[0]), 0.5 * (c.kerb_a[1] + c.kerb_b[1])))
                   for c in P.build_graph().crossings]
    return _G["u"]


def _crossing_at(xy):
    return next((c, xc) for c, xc in _usable() if math.hypot(xc[0] - xy[0], xc[1] - xy[1]) < 1.0)


def _path_over(c):
    """A loop whose path drives over crossing c."""
    return next(p for p in PATHS.values() if (c.x, c.y) in [(x, y) for x, y, _ in R.crossings_on(p)])


def _state(path, s):
    """A car at arc length s: (x, y, unit heading, Idx) - Idx the point it drives
    toward, as DriveTick and citylife_routes.follow_step advance it."""
    cum, total = P.path_arc(path)
    i = max(0, bisect.bisect_right(cum, s % total) - 1)
    x, y, yaw = P.path_point(path, s)
    return x, y, (math.cos(math.radians(yaw)), math.sin(math.radians(yaw))), (i + 1) % len(path.pts)


def _view(path, s, v):
    x, y, f, _ = _state(path, s)
    return P.CarView(x, y, math.degrees(math.atan2(f[1], f[0])), v, path, s)


def _rows(path):
    """The car's Crossings as apply_routes.py writes them."""
    return [(x, y, float(i)) for x, y, i in R.crossings_on(path)]


def test_the_zebra_a_figure_computes_is_the_one_the_cars_carry():
    """XC is the kerb/exit midpoint of a leg: the CROSSINGS point a car's
    Crossings row holds, to well inside MATCH_CM, and no other zebra's."""
    legs = 0
    for p in PLAN:
        r = p["route"]
        for i, q in enumerate(r):
            if int(q[2]) != P.KIND_CROSS_EXIT:
                continue
            xc = (0.5 * (r[i - 1][0] + q[0]), 0.5 * (r[i - 1][1] + q[1]))
            d = sorted(math.hypot(cx - xc[0], cy - xc[1]) for cx, cy in R.CROSSINGS)
            assert d[0] < 1.0 and d[1] > 10 * W.MATCH_CM, (p["tag"], i, d[:2])
            legs += 1
    assert legs >= 40


def test_a_loop_drives_over_a_zebra_at_most_once():
    """One Crossings row per zebra is all the path rule reads: a second pass
    over the same zebra would be invisible to it (and to time_to_zebra)."""
    for k, path in PATHS.items():
        for c, _ in _usable():
            near = [math.hypot(x - c.x, y - c.y) <= 450.0 for x, y in path.pts]
            runs = sum(1 for i in range(len(near)) if near[i] and not near[i - 1])
            assert runs <= 1, (k, c.id, runs)


def test_the_path_rule_blocks_every_car_the_model_puts_under_the_gap():
    """Every loop, a car every 25 cm, six speeds, against every usable crossing:
    wherever time_to_zebra(path) < GAP_S the Blueprint's test blocks. And it
    does not simply block everything: an extra block is at most 400 cm further
    out than the model's line (one point of deliberate slack, one for where
    the car is on its segment, and the loops' 145.8-156.8 cm spacing: 366
    measured) or 100 cm further past it (78 measured)."""
    miss, extra, n, nb = [], [], 0, 0
    for k, path in PATHS.items():
        _, total = P.path_arc(path)
        rows, npts = _rows(path), len(path.pts)
        for c, xc in _usable():
            s_x = P.crossing_s(path, c)
            for v in SPEEDS:
                s = 0.0
                while s < total:
                    x, y, f, idx = _state(path, s)
                    tta = P.time_to_zebra(_view(path, s, v), c)
                    b = W.car_blocks(xc, c.walk_dir(), (x, y), f, v, rows, idx, npts)
                    n, nb = n + 1, nb + b
                    if tta < P.GAP_S and not b:
                        miss.append((k, c.id, v, round(s), round(tta, 2)))
                    elif b and tta >= P.GAP_S:
                        d = (s_x - s) % total if s_x is not None else None
                        if d is None or not (d - (W.REACH_CM + P.GAP_S * v) <= 400.0
                                             or (total - d) - W.REACH_CM <= 100.0):
                            extra.append((k, c.id, v, round(s), d and round(d)))
                    s += 25.0
    assert not miss, (len(miss), miss[:5])
    assert not extra, (len(extra), extra[:5])
    assert n > 100000 and 0.01 < nb / n < 0.1, (n, nb)


def test_a_car_turning_into_the_zebra_is_seen_on_its_arc():
    """The case the heading rule missed: loop A turns at (4100, -4100) onto the
    zebra at (4100, -3000). While the car still points along the road it is
    leaving, the path rule blocks it everywhere the model does - from the full
    4 s out, where the heading rule first counted it 0.4 s out at 3.2 m/s."""
    path = PATHS["A"]
    c, xc = _crossing_at((4100.0, -3000.0))
    _, total = P.path_arc(path)
    s_x = P.crossing_s(path, c)
    seen = []
    for v in (320.0, 560.0):
        for back in range(0, 3000, 10):
            s = (s_x - back) % total
            x, y, f, idx = _state(path, s)
            tta = P.time_to_zebra(_view(path, s, v), c)
            across = abs(f[0] * c.walk_dir()[0] + f[1] * c.walk_dir()[1]) > 0.7
            if across and tta < P.GAP_S:
                assert W.car_blocks(xc, c.walk_dir(), (x, y), f, v, _rows(path), idx, len(path.pts)), (v, back)
                seen.append(tta)
    assert seen and max(seen) > 1.0, sorted(seen)[-3:]


def test_a_car_whose_path_misses_the_zebra_blocks_only_on_it():
    """A car with no Crossings row for this zebra blocks only while its body is
    on the box - time_to_zebra's 0-or-inf for such a car - and the two-axis
    box test is never smaller than the exact overlap (car_on_zebra)."""
    for c, xc in _usable()[:2]:
        for yaw in range(0, 360, 15):
            f = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)))
            for dx in range(-1400, 1401, 50):
                for dy in range(-1400, 1401, 50):
                    x, y = c.x + dx, c.y + dy
                    on = P.car_on_zebra(P.CarView(x, y, yaw, 300.0), c)
                    b = W.car_blocks(xc, c.walk_dir(), (x, y), f, 300.0, [], 0, 100)
                    assert b or not on, (c.id, yaw, dx, dy)
                    assert not b or (abs(dx) < 1100 and abs(dy) < 1100), (c.id, yaw, dx, dy)
    c, xc = _usable()[0]
    assert W.car_blocks(xc, c.walk_dir(), (c.x, c.y), (1.0, 0.0), 300.0, [], 0, 100)
    assert not W.car_blocks(xc, c.walk_dir(), (c.x, c.y), (1.0, 0.0), 40.0, [], 0, 100)   # standing


# ------------------------------------------------------------------ the graph, run

TICK = next(f for f in DSL.parse(W.render()) if f[:2] == ["event", "EventTick"])


def _ped(route, idx, pstate, loc, woff, vel=(0.0, 0.0, 0.0), **kw):
    v = dict(UseNavMesh=False, Route=[tuple(q) for q in route], Idx=idx, PState=pstate,
             WaitT=0.0, KerbWaitMax=0.0, WOff=list(woff), NextMoveTime=0.0, StuckT=0.0,
             Crossings=0, StuckN=0, DetT=0.0, DetS=0.0, Detours=0, Teleports=0)
    v.update(kw)
    return DSL.Obj("BP_CityPed", v, loc=(loc[0], loc[1], 90.0), vel=vel)


def _car(path, s, v):
    x, y, f, idx = _state(path, s)
    return DSL.Obj("BP_CityCar", {"CurSpeed": v, "Crossings": _rows(path), "Idx": idx,
                                  "NPts": len(path.pts)},
                   loc=(x, y, 5.0), fwd=(f[0], f[1], 0.0))


def _tick(ped, t, cars=(), dt=0.1):
    DSL.Machine(ped, {"t": t, "actors": {W.CAR: list(cars)}}).run(TICK[3:], {"DeltaSeconds": dt})
    return ped.vars


def _moves(ped, t, dt=0.1):
    """One tick; the directions it asked AddMovementInput for."""
    m = DSL.Machine(ped, {"t": t, "actors": {W.CAR: []}})
    m.run(TICK[3:], {"DeltaSeconds": dt})
    return [c[1] for c in m.calls if c[0] == "move"]


def _legs():
    """(tag, route, woff, exit index) of every crossing leg in the plan."""
    for p in PLAN:
        for i, q in enumerate(p["route"]):
            if int(q[2]) == P.KIND_CROSS_EXIT:
                yield p["tag"], p["route"], p["woff"], i


def _walk_time(woff, on=True):
    return next(0.25 * k for k in range(400) if W.is_walk(0.25 * k, woff) == on)


def _leg_geometry(route, i):
    (kx, ky, _), (ex, ey, _) = route[i - 1], route[i]
    L = math.hypot(ex - kx, ey - ky)
    return (kx, ky), (0.5 * (kx + ex), 0.5 * (ky + ey)), ((ex - kx) / L, (ey - ky) / L)


def test_the_rendered_car_test_is_car_blocks():
    """The graph's EventTick, run for a figure at its kerb in its walk phase
    with one car (every 300 cm of every loop, three speeds, one leg per
    crossing): it starts across exactly when car_blocks says the car is clear."""
    seen, runs, blocked = set(), 0, 0
    for tag, route, woff, i in _legs():
        kerb, xc, xw = _leg_geometry(route, i)
        if xc in seen:
            continue
        seen.add(xc)
        t = _walk_time(woff[i])
        for k, path in PATHS.items():
            _, total = P.path_arc(path)
            for v in (0.0, 200.0, 450.0):
                s = 0.0
                while s < total:
                    car = _car(path, s, v)
                    got = _tick(_ped(route, i, 2, kerb, woff), t, [car])["PState"]
                    want = W.car_blocks(xc, xw, car.loc[:2], car.fwd[:2], v, car.vars["Crossings"],
                                        car.vars["Idx"], car.vars["NPts"])
                    assert got == (2 if want else 3), (tag, k, v, round(s), got, want)
                    runs, blocked = runs + 1, blocked + want
                    s += 300.0
    assert len(seen) == 5 and runs > 1000 and blocked > 50, (len(seen), runs, blocked)


def test_every_car_must_be_clear_and_the_walk_phase_on():
    tag, route, woff, i = next(_legs())
    kerb, xc, _ = _leg_geometry(route, i)
    c, _ = _crossing_at(xc)
    path = _path_over(c)
    s_x = P.crossing_s(path, c)
    near, far = _car(path, s_x - 900.0, 400.0), _car(path, s_x - 9000.0, 400.0)
    t = _walk_time(woff[i])
    assert _tick(_ped(route, i, 2, kerb, woff), t, [far])["PState"] == 3
    assert _tick(_ped(route, i, 2, kerb, woff), t, [far, near, far])["PState"] == 2
    assert _tick(_ped(route, i, 2, kerb, woff), _walk_time(woff[i], on=False), [far])["PState"] == 2


def test_a_figure_stuck_short_of_its_kerb_waits_there_and_is_checked():
    """Stuck within NEAR_CM of a kerb-wait node (a crowd already on the spot)
    it goes to WAIT (Idx on to the exit, WaitT 0), as an arrival would; a rule
    once skipped to the exit in WALK and crossed with no walk or gap test.
    From there it starts only when clear, judging the zebra of its leg, not
    the spot where it stopped."""
    tag, route, woff, i = next(_legs())
    kerb, xc, xw = _leg_geometry(route, i)
    stand = (kerb[0] - 250.0 * xw[0], kerb[1] - 250.0 * xw[1])     # 2.5 m back from the kerb node
    ped = _ped(route, i - 1, 0, stand, woff, StuckT=W.STUCK_S - 0.05)
    v = _tick(ped, 0.0)
    assert (v["PState"], v["Idx"], v["WaitT"], v["StuckT"]) == (2, i, 0.0, 0.0), v
    c, _ = _crossing_at(xc)
    path = _path_over(c)
    blocker = _car(path, P.crossing_s(path, c) - W.REACH_CM - 400.0, 400.0)   # 1 s out
    t = _walk_time(woff[i])
    assert _tick(ped, t, [blocker])["PState"] == 2
    assert ped.vars["XC"][:2] == xc
    assert _tick(ped, t, [])["PState"] == 3



def test_arriving_at_the_kerb_waits_there_and_the_exit_ends_the_crossing():
    """Arrival reads the kind of the point REACHED, not of the next one. The
    kind was a pure bind read after SetIdx - Blueprint re-evaluates a pure
    node at each use - so a figure reaching its kerb node took the EXIT's
    kind and walked across in WALK with no walk-phase and no gap test
    (Simulate, 2026-09-30: two figures mid-carriageway in PState 0)."""
    tag, route, woff, i = next(_legs())
    kerb, xc, xw = _leg_geometry(route, i)
    near = (kerb[0] - 30.0 * xw[0], kerb[1] - 30.0 * xw[1])      # inside the 80 cm arrival
    v = _tick(_ped(route, i - 1, 0, near, woff), 0.0)
    assert (v["PState"], v["Idx"]) == (2, i), v
    ex = route[i]
    v = _tick(_ped(route, i, 3, (ex[0] - 30.0 * xw[0], ex[1] - 30.0 * xw[1]), woff), 0.0)
    assert (v["PState"], v["Idx"]) == (0, (i + 1) % len(route)), v
    # The evaluator re-evaluates pure binds, as Blueprint does: the old graph
    # (kind bound, then Idx moved, then the kind read) fails the same case.
    old_src = (W.render()
               .replace("(Variables|Default|SetNKind (Math|Float|Truncate (.z _tgt)))",
                        "(bind _kind (Math|Float|Truncate (.z _tgt)))")
               .replace("(Variables|Default|GetNKind)", "_kind"))
    old = next(f for f in DSL.parse(old_src) if f[:2] == ["event", "EventTick"])
    ped = _ped(route, i - 1, 0, near, woff)
    DSL.Machine(ped, {"t": 0.0, "actors": {W.CAR: []}}).run(old[3:], {"DeltaSeconds": 0.1})
    assert ped.vars["PState"] == 0 and ped.vars["Idx"] == i, ped.vars

def test_verify_peds_counts_a_figure_on_a_zebra_that_is_not_crossing():
    """verify_peds.py's per-read check, by distinct tag; property names read
    case-blind; a figure-read without Spd is checked on PState only and
    counted as such, so a 0 can be told from nothing measured."""
    from tools.citylife_mcp import verify_peds as VP
    zx, zy = R.CROSSINGS[1]
    reads = [{"peds": {"Ped_00": {"x": zx, "y": zy, "PState": 3, "Spd": 140.0},        # crossing
                       "Ped_01": {"x": zx, "y": zy, "PState": 0, "Spd": 120.0},        # walking on it
                       "Ped_02": {"x": zx + 100, "y": zy, "PState": 3, "Spd": 2.0},    # standing on it
                       "Ped_03": {"x": zx + 5000, "y": zy, "PState": 1, "Spd": 0.0}}},  # elsewhere
             {"peds": {"Ped_01": {"x": zx, "y": zy + 50, "pState": 2},                 # no Spd
                       "Ped_04": {"x": zx, "y": zy, "pState": 3}}}]
    z = VP.on_zebra_not_crossing(reads)
    why = z.pop("why")
    assert z == {"tags": ["Ped_01", "Ped_02"], "figure_reads_in_a_box": 5,
                 "state_unread": 0, "speed_unread": 2}, z
    # ...and says what each was doing: walking on it (twice), standing on it
    assert [(w["read"], w["pstate"]) for w in why["Ped_01"]] == [(0, 0), (1, 2)], why
    assert why["Ped_02"] == [{"read": 0, "pstate": 3, "idx": None, "spd": 2.0,
                              "x": zx + 100, "y": zy}], why
    assert VP.on_zebra_not_crossing([{"peds": {}}])["figure_reads_in_a_box"] == 0


def test_verify_peds_finds_a_figure_off_its_route():
    """The stuck cascade left figures in CROSS 55 m from their zebra with every
    zebra count clean. off_route measures each figure against its own leg."""
    from tools.citylife_mcp import verify_peds as VP
    route = [{"x": 0.0, "y": 0.0, "z": 0.0}, {"x": 2000.0, "y": 0.0, "z": 1.0},
             {"x": 2000.0, "y": 2000.0, "z": 2.0}]
    reads = [{"peds": {
        "Ped_00": {"x": 1000.0, "y": 100.0, "PState": 0, "Idx": 1, "Route": route},    # on its leg
        "Ped_01": {"x": 2050.0, "y": 1000.0, "PState": 3, "Idx": 2, "Route": route},   # crossing
        "Ped_02": {"x": 8000.0, "y": 5000.0, "PState": 3, "Idx": 2, "Route": route},   # lost
        "Ped_03": {"x": 1000.0, "y": 0.0, "PState": 2, "Idx": 2, "Route": route},      # waits 10 m off its kerb
        "Ped_04": {"x": 1000.0, "y": 0.0, "PState": 0, "Idx": 1}}}]                    # no route read
    o = VP.off_route(reads)
    assert sorted(o["tags"]) == ["Ped_02", "Ped_03"] and o["checked"] == 4, o
    assert o["tags"]["Ped_03"] == {"read": 0, "pstate": 2, "idx": 2, "cm": 1000}, o


def test_stuck_near_its_point_it_arrives_and_anywhere_else_it_keeps_the_target():
    """The rule that cascaded (Simulate 2026-09-30, 21 of 40 figures frozen in
    CROSS far from any zebra): stuck short of a pavement node it SKIPPED to
    the next point - round a building corner from where it stood - stalled
    again, skipped again, until a kerb node sent it to WAIT tens of metres
    from its zebra. Now Idx moves only at the point it names."""
    tag, route, woff, i = next(_legs())
    n = len(route)
    j = next(k for k in range(n) if int(route[k][2]) == P.KIND_PAVEMENT)
    st = W.STUCK_S - 0.05
    # stuck 2 m from its pavement node: that is arriving
    v = _tick(_ped(route, j, 0, (route[j][0] + 200.0, route[j][1]), woff, StuckT=st), 0.0)
    assert (v["PState"], v["Idx"], v["StuckN"]) == (0, (j + 1) % n, 0), v
    # stuck 5 m from it: the target is kept, a detour starts
    ped = _ped(route, j, 0, (route[j][0] + 500.0, route[j][1]), woff, StuckT=st)
    mv = _moves(ped, 10.0)
    v = ped.vars
    assert (v["PState"], v["Idx"], v["StuckN"], v["Detours"], v["StuckT"]) == (0, j, 1, 1, 0.0), v
    assert v["DetT"] == 10.0 + W.DETOUR_S and v["DetS"] == 1.0, v
    # ...stepping mostly to one side: target due -x (south), DetS +1 is -y (west)
    assert len(mv) == 1 and mv[0][1] < -0.9 and mv[0][0] < 0.0, mv
    # the next detour goes the other way
    mv = _moves(_ped(route, j, 0, (route[j][0] + 500.0, route[j][1]), woff, StuckT=st, StuckN=1), 10.0)
    assert mv[0][1] > 0.9, mv
    # the same in CROSS, 5 m short of the exit: it stays crossing, to the exit
    v = _tick(_ped(route, i, 3, (route[i][0] + 500.0, route[i][1]), woff, StuckT=st), 0.0)
    assert (v["PState"], v["Idx"], v["StuckN"]) == (3, i, 1), v
    # ...and far short of a kerb node it does NOT wait where it stands
    kerb, _, xw = _leg_geometry(route, i)
    v = _tick(_ped(route, i - 1, 0, (kerb[0] - 900.0 * xw[0], kerb[1] - 900.0 * xw[1]), woff,
                   StuckT=st), 0.0)
    assert (v["PState"], v["Idx"], v["StuckN"]) == (0, i - 1, 1), v
    # not yet stuck: the clock runs; moving: it resets
    v = _tick(_ped(route, j, 0, (route[j][0] + 500.0, route[j][1]), woff, StuckT=0.5), 0.0)
    assert (v["Idx"], round(v["StuckT"], 3), v["StuckN"]) == (j, 0.6, 0), v
    v = _tick(_ped(route, j, 0, (route[j][0] + 500.0, route[j][1]), woff, vel=(-130.0, 0.0, 0.0),
                   StuckT=1.0), 0.0)
    assert v["StuckT"] == 0.0, v


def test_after_its_last_detour_a_figure_is_set_down_on_its_point():
    """Bounded: the TELEPORT_N-th detour puts it on the point, which is then
    reached - for a kerb node that is WAIT at the kerb, not in the road."""
    tag, route, woff, i = next(_legs())
    n = len(route)
    j = next(k for k in range(n) if int(route[k][2]) == P.KIND_PAVEMENT)
    ped = _ped(route, j, 0, (route[j][0] + 500.0, route[j][1]), woff,
               StuckT=W.STUCK_S - 0.05, StuckN=W.TELEPORT_N - 1)
    v = _tick(ped, 0.0)
    assert ped.loc[:2] == (route[j][0], route[j][1]) and ped.loc[2] == 90.0, ped.loc
    assert (v["Idx"], v["PState"], v["Teleports"], v["StuckN"]) == ((j + 1) % n, 0, 1, 0), v
    kerb, _, xw = _leg_geometry(route, i)
    ped = _ped(route, i - 1, 0, (kerb[0] - 900.0 * xw[0], kerb[1] - 900.0 * xw[1]), woff,
               StuckT=W.STUCK_S - 0.05, StuckN=W.TELEPORT_N - 1)
    v = _tick(ped, 0.0)
    assert ped.loc[:2] == kerb and (v["PState"], v["Idx"], v["WaitT"]) == (2, i, 0.0), (ped.loc, v)


def test_a_stuck_figure_walks_round_a_post_instead_of_skipping_ahead():
    """Closed loop in the evaluator, with a round post on the line between two
    pavement nodes and movement integrated at 130 cm/s: the figure detours,
    passes the post and reaches the node it was heading for - Idx moving by
    one, never skipping - without being set down. The toy collision slides
    a blocked step along the post, as a capsule does: straight at it, dead
    centre, it goes nowhere, which is the stall a detour must break."""
    tag, route, woff, i = next(_legs())
    n = len(route)
    j = next(k for k in range(1, n) if int(route[k][2]) == P.KIND_PAVEMENT
             and int(route[k - 1][2]) == P.KIND_PAVEMENT)
    (ax, ay, _), (bx, by, _) = route[j - 1], route[j]
    L = math.hypot(bx - ax, by - ay)
    ux, uy = (bx - ax) / L, (by - ay) / L
    post, R_POST = (ax + 0.5 * L * ux, ay + 0.5 * L * uy), 60.0 + 35.0
    ped = _ped(route, j, 0, (ax + 100.0 * ux, ay + 100.0 * uy), woff)
    t, dt, speed = 0.0, 0.1, 130.0
    for _ in range(int(60.0 / dt)):
        mv = _moves(ped, t, dt)
        if ped.vars["Idx"] != j:
            break
        x, y, z = ped.loc
        if mv:
            dx, dy = mv[0][0] * speed * dt, mv[0][1] * speed * dt
            nx, ny = x + dx, y + dy
            h = math.hypot(nx - post[0], ny - post[1])
            if h < R_POST:              # a capsule slides along what it hits: pushed out radially
                nx, ny = post[0] + (nx - post[0]) * R_POST / h, post[1] + (ny - post[1]) * R_POST / h
            ped.loc = (nx, ny, z)
            ped.vel = ((nx - x) / dt, (ny - y) / dt, 0.0)
        else:
            ped.vel = (0.0, 0.0, 0.0)
        t += dt
    v = ped.vars
    assert v["Idx"] == (j + 1) % n and v["Teleports"] == 0 and v["Detours"] >= 1, (v["Idx"], v)
    assert math.hypot(ped.loc[0] - bx, ped.loc[1] - by) < W.NEAR_CM, ped.loc


def test_pavement_nodes_are_spread_across_the_pavement_and_the_zebras_kept():
    """Every figure on one line met the ones walking the other way exactly
    head-on. Pavement nodes move by lane_jitter(k) away from their road;
    kerb-wait and exit nodes stay where the zebra is."""
    raw = P.make_routes(n_peds=40)
    seen = set()
    for k, (p, r) in enumerate(zip(PLAN, raw)):
        j = W.lane_jitter(k)
        assert abs(j) <= W.LANE_JITTER_CM
        seen.add(j)
        for (x, y, kind), (x0, y0, k0) in zip(p["route"], r):
            assert int(kind) == int(k0)
            if int(k0) != P.KIND_PAVEMENT:
                assert (x, y) == (round(x0, 1), round(y0, 1)), (p["tag"], x, y, x0, y0)
            else:
                assert abs(x - x0) <= abs(j) + 0.05 and abs(y - y0) <= abs(j) + 0.05
                moved = (x, y) != (round(x0, 1), round(y0, 1))
                assert moved or j == 0.0 or (W._away(x0) == 0 and W._away(y0) == 0), (p["tag"], x0, y0)
    assert len(seen) == 5, seen


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
