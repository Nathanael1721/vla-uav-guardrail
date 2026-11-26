"""
obstacle_clearance golden cases — one test per failure mode the review found.

Run either way:
    pytest tests/test_clearance.py -v
    python tests/test_clearance.py

The invariant behind every case: the Shield must never answer a clearance
problem with a FREEZE. A stationary vehicle inside the clearance ring never
recedes, so a brake there re-raises the identical violation on the next tick and
every tick after it — the fail-safe becomes the deadlock.
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import Action4D, Shield, State, load_policy       # noqa: E402
from guardrail.models import (KinematicEnvelope, ObstacleClearance,  # noqa: E402
                              Policy)
from guardrail.shield import BRAKE                               # noqa: E402

# The four ConstraintBase fields the grant requires (IR schema 3.0.0, 2026-11-02;
# scope since 4.0.0, 2026-11-06), at the values they defaulted to before
# (repair = project_fix; an absent scope meant mission).
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}

CITYMAP = ROOT / "demo" / "out" / "citymap" / "occ_day.npz"
URBAN = ROOT / "policies" / "urban_clearance.yaml"

# Synthetic maps mirror the shipped ones: 80 x 80 cells at 2 m, origin (-80,-80).
RES, OX, OY, N = 2.0, -80.0, -80.0, 80


def _ci(x):                      # world x -> row index (city_planner's mapping)
    return int(round((x - OX) / RES))


def _cj(y):                      # world y -> column index
    return int(round((y - OY) / RES))


def _shield(occ, min_clearance=6.0, soft=2.0, cap=4.0):
    pol = Policy(policy_id="clr-test", constraints=[
        ObstacleClearance(id="clr", type="obstacle_clearance", **HARD_P0,
                          min_clearance_m=min_clearance, soft_margin_m=soft),
        KinematicEnvelope(id="kin", type="kinematic_envelope", **HARD_P0, speed_max_mps=cap,
                          climb_rate_max_mps=2.0, yaw_rate_max_dps=45.0)])
    return Shield(pol, lookahead_s=3.0, dt=0.5,
                  obstacle_map={"occ": occ, "res": RES, "ox": OX, "oy": OY})


def _city_shield():
    import city_planner
    cm = city_planner.load_occ(str(CITYMAP))
    sh = Shield(load_policy(URBAN), lookahead_s=3.0, dt=0.5,
                obstacle_map={"occ": cm["occ"], "res": cm["res"],
                              "ox": cm["ox"], "oy": cm["oy"]})
    return sh, cm


def _step_dist(sh, st, a, t=0.5):
    """Distance one control step along `a` — "is it actually getting away?"."""
    return sh._distance_at(st.x + a.vx * t, st.y + a.vy * t)


def _moving(a, floor=0.5):
    return math.hypot(a.vx, a.vy) > floor


# ------------------------------------------------------- distance field basics

def test_distance_field_is_signed_inside_buildings():
    """The unsigned field read 0 across a whole footprint, so its gradient there
    was flat and "the way out" was undefined."""
    occ = np.zeros((N, N), np.uint8)
    occ[_ci(-10):_ci(10) + 1, _cj(-10):_cj(10) + 1] = 1     # 20 m square block
    sh = _shield(occ)
    assert sh._distance_at(0.0, 0.0) < 0.0                  # dead centre: inside
    assert sh._distance_at(0.0, 30.0) > 0.0                 # out in the open
    assert sh._distance_at(0.0, 0.0) < sh._distance_at(0.0, -8.0)  # deeper = lower
    assert sh._away_dir(0.0, -8.0, 6.0) != (0.0, 0.0)       # always a way out


def test_away_dir_never_returns_no_direction_on_the_city_map():
    """Finding: 39% of occupied cells on the shipped map answered "no direction",
    which fell through to BRAKE and froze the drone in mid-air over a building
    (the demo's ESCAPE mode climbs onto exactly those cells on purpose)."""
    sh, cm = _city_shield()
    occ = np.asarray(cm["occ"])
    dead = 0
    for i, j in zip(*np.nonzero(occ)):
        x = cm["ox"] + int(i) * cm["res"]
        y = cm["oy"] + int(j) * cm["res"]
        if sh._away_dir(x, y, 5.0) == (0.0, 0.0):
            dead += 1
    assert dead == 0, f"{dead} occupied cells still have no escape bearing"


def test_no_freeze_on_a_building_footprint():
    """The reported dead cell: the old Shield emitted (0,0,0) braked=True here,
    forever, at 45 m altitude."""
    sh, _ = _city_shield()
    st = State(x=-74.0, y=-18.0, up=45.0)
    assert sh._distance_at(st.x, st.y) < 0.0                # over a building
    d = sh.filter(st, Action4D(vx=3.0, vy=0.0, vz_up=1.0))
    assert not d.braked
    assert _moving(d.emitted)
    assert _step_dist(sh, st, d.emitted) > sh._distance_at(st.x, st.y)


def test_a_building_on_the_map_border_is_not_a_wall_to_infinity():
    """citylife_redcar_ground/_carpolicy: past this 160 m map's edge the field
    extrapolated `edge value - distance past the edge`, and the edge cell at
    (48, 78) is a building - so (47.2, 112.7), the entrance of a CityLife
    junction, read -35.16 m and the clearance repair reversed a +3 m/s follow
    at 5 m/s for up to 187 s. Off the map the bound is the distance to the last
    cell centre, less that cell's half-width."""
    sh, cm = _city_shield()
    edge_y = cm["oy"] + (cm["occ"].shape[1] - 1) * cm["res"]
    d = sh._distance_at(47.2, 112.7)
    assert d >= 112.7 - edge_y - cm["res"] / 2 - 1e-6, d
    # and nothing is repaired when the aircraft flies on, off the map
    st = State(x=45.0, y=110.0, up=8.0)
    dec = sh.filter(st, Action4D(vx=3.0, vy=0.5, vz_up=0.0))
    assert not any(r.operator == "ClearanceFix" for r in dec.repairs), dec.repairs


def test_the_half_cell_past_a_border_building_is_still_the_building():
    """Between the last cell centre and the map's edge, a border building
    must read as inside - and the field must not jump there. The first off-map
    fix returned +off from the last centre on, so (78.2, 0) read clear inside
    a building and a push out through it counted as receding (review,
    2026-09-24)."""
    occ = np.zeros((N, N), np.uint8)
    occ[77:, 30:50] = 1                      # 3 rows deep on the North border
    sh = _shield(occ)
    inside = sh._distance_at(78.0, 0.0)
    assert inside < 0.0
    for x in (78.1, 78.5, 78.9):
        d = sh._distance_at(x, 0.0)
        assert d < 0.0, (x, d)                # still inside the building
        assert abs(d - inside) <= (x - 78.0) + 1e-9, (x, d)   # 1-Lipschitz
    far = sh._distance_at(140.0, 0.0)          # 62 m past it: not a wall
    assert far > 50.0, far
    free = sh._distance_at(90.0, -70.0)        # off a FREE stretch of edge
    assert free >= (90.0 - 78.0) - RES / 2 - 1e-9, free


def test_off_the_map_is_reported_not_passed_as_clear():
    occ = np.zeros((N, N), np.uint8)
    sh = _shield(occ)
    assert not sh.off_map(0.0, 0.0)
    assert not sh.off_map(OX, OY)                        # on the first cell
    assert sh.off_map(0.0, OY + N * RES + 5.0)           # past the far edge
    assert sh.off_map(OX - 5.0, 0.0)
    no_map = Shield(Policy(policy_id="none", constraints=[]), lookahead_s=3.0, dt=0.5)
    assert not no_map.off_map(500.0, 500.0)              # no map: nothing to be off
    # ...which is why a caller must be able to tell "no map" from "on the map"
    assert sh.has_obstacle_map and not no_map.has_obstacle_map


# ------------------------------------------------------------ the repair chain

def test_geofence_tangent_is_re_repaired_for_clearance():
    """Repair ordering: _repair_geofence ran AFTER _repair_clearance and its
    output was never re-repaired, so the anti-stall tangent tripped the clearance
    rule and the P0 guard braked — permanently, at the demo's own spawn."""
    sh, _ = _city_shield()
    st = State(x=36.4, y=-18.8, up=45.0)
    d = sh.filter(st, Action4D(vx=-0.05, vy=4.0))
    assert not d.braked
    assert _moving(d.emitted)
    assert not sh._check(st, d.emitted)


def test_escape_direction_comes_from_the_forecast_not_the_present():
    """The push used the gradient at the CURRENT pose while the depth came from
    the forecast. With a different building nearest right now, "outward" pointed
    straight AT the future obstacle and the repair accelerated into it."""
    occ = np.zeros((N, N), np.uint8)
    occ[:, :_cj(-12) + 1] = 1                 # building A, face at y = -12
    occ[:, _cj(6):] = 1                       # building B, face at y = +6
    sh = _shield(occ, min_clearance=6.0, soft=2.0)
    st = State(x=0.0, y=-4.0, up=40.0)
    raw = Action4D(vx=0.0, vy=4.0)            # heading at B, but A is nearer NOW
    assert sh._distance_at(st.x, st.y) >= 6.0             # legal right now...
    assert sh._check(st, raw)                             # ...forecast is not
    before = sh._clear_min_dist(st, raw)
    d = sh.filter(st, raw)
    assert d.emitted.vy <= 0.0                            # never accelerate at B
    assert sh._clear_min_dist(st, d.emitted) > before      # repairs must improve
    assert not d.braked


def test_repair_never_lowers_the_forecast_clearance():
    """The invariant the old code had no way to state, swept over headings."""
    occ = np.zeros((N, N), np.uint8)
    occ[_ci(8):_ci(20) + 1, _cj(-30):_cj(-6) + 1] = 1
    occ[_ci(-20):_ci(-8) + 1, _cj(6):_cj(30) + 1] = 1
    sh = _shield(occ, min_clearance=6.0, soft=2.0)
    checked = 0
    for x in range(-24, 25, 4):
        for y in range(-24, 25, 4):
            st = State(x=float(x), y=float(y), up=40.0)
            for k in range(8):
                ang = k * math.pi / 4.0
                raw = Action4D(vx=4.0 * math.cos(ang), vy=4.0 * math.sin(ang))
                if not sh._check(st, raw):
                    continue
                d = sh.filter(st, raw)
                if d.braked:
                    continue
                checked += 1
                assert (sh._clear_min_dist(st, d.emitted)
                        >= sh._clear_min_dist(st, raw) - 1e-9), (
                    f"repair made it worse at ({x},{y}) heading {k}")
    assert checked > 50, f"only {checked} repaired samples exercised"


# ------------------------------------------------------------ the trend-aware check

def test_receding_does_not_green_light_the_whole_horizon():
    """`if d_now < min and receding: continue` dropped the ENTIRE lookahead for
    that rule, so a drone inside the ring got a free pass on every obstacle on
    the map — including a forecast that ended inside a wall."""
    occ = np.zeros((N, N), np.uint8)
    occ[:, :_cj(-2) + 1] = 1                  # west wall, face at y = -2
    occ[:, _cj(6):] = 1                       # east wall, face at y = +6
    sh = _shield(occ, min_clearance=5.0, soft=2.0)
    st = State(x=0.0, y=0.0, up=40.0)
    raw = Action4D(vx=0.0, vy=4.0)
    d_now = sh._distance_at(st.x, st.y)
    assert d_now < 5.0                                     # inside the ring...
    assert _step_dist(sh, st, raw) > d_now                 # ...and receding
    viol = sh._check(st, raw)
    assert viol, "forecast crosses the far wall but was not flagged"
    assert viol[0].predicted_at_s > 0.0                    # caught in the FUTURE
    d = sh.filter(st, raw)
    assert d.touched and not d.braked
    assert not sh._check(st, d.emitted)


def test_escaping_the_ring_is_still_not_flagged():
    """The other half of trend-awareness: retreating from a wall must stay a
    clean passthrough, or the Shield fights its own recovery."""
    occ = np.zeros((N, N), np.uint8)
    occ[_ci(10):, :] = 1                                   # wall, face at x = 10
    sh = _shield(occ, min_clearance=6.0, soft=2.0)
    st = State(x=6.0, y=0.0, up=40.0)
    assert sh._distance_at(st.x, st.y) < 6.0               # inside the ring
    out = sh.filter(st, Action4D(vx=-3.0, vy=0.0))
    assert not out.violations and out.emitted == out.raw and not out.braked
    deeper = sh.filter(st, Action4D(vx=3.0, vy=0.0))
    assert any(v.category == "clearance" for v in deeper.violations)
    hover = sh.filter(st, Action4D())
    assert any(v.category == "clearance" for v in hover.violations)


# ------------------------------------------------------------------- the brake

def test_brake_is_never_emitted_when_stopping_is_itself_illegal():
    """filter() returned BRAKE without ever re-checking it. Inside the ring a
    zero action leaves d_next == d_now, so `receding` is false and the identical
    violation is raised forever."""
    occ = np.zeros((N, N), np.uint8)
    occ[_ci(0):_ci(6) + 1, _cj(0):_cj(6) + 1] = 1
    sh = _shield(occ, min_clearance=6.0, soft=2.0)
    st = State(x=2.0, y=-3.0, up=40.0)
    assert sh._check(st, BRAKE), "test premise: stopping here IS a violation"
    for raw in (Action4D(), Action4D(vx=0.0, vy=4.0), Action4D(vx=4.0, vy=0.0),
                Action4D(vx=-2.8, vy=2.8)):
        d = sh.filter(st, raw)
        assert not d.braked, f"froze on raw {raw}"
        assert _moving(d.emitted), f"emitted a standstill on raw {raw}"
        assert _step_dist(sh, st, d.emitted) > sh._distance_at(st.x, st.y)


def test_wedged_between_a_wall_and_a_zone_still_recovers():
    """The reported deadlock: ClearanceFix pushes into the NFZ, GeofenceSlide
    cancels it, the P0 guard brakes, and at rest the clearance violation
    persists. Four consecutive ticks of (0,0) that never terminate."""
    from guardrail.models import PolygonFence, XY
    occ = np.zeros((N, N), np.uint8)
    occ[:, _cj(-16):_cj(-8) + 1] = 1                       # wall face at y = -8
    pol = Policy(policy_id="wedge", constraints=[
        ObstacleClearance(id="clr", type="obstacle_clearance", **HARD_P0,
                          min_clearance_m=6.0, soft_margin_m=2.0),
        PolygonFence(id="nfz", type="polygon_fence", **HARD_P0,
                     vertices=[XY(x=-40, y=1), XY(x=40, y=1),
                               XY(x=40, y=40), XY(x=-40, y=40)],
                     altitude_floor_m=0, altitude_ceiling_m=60),
        KinematicEnvelope(id="kin", type="kinematic_envelope", **HARD_P0, speed_max_mps=4.0,
                          climb_rate_max_mps=2.0, yaw_rate_max_dps=45.0)])
    sh = Shield(pol, lookahead_s=3.0, dt=0.5,
                obstacle_map={"occ": occ, "res": RES, "ox": OX, "oy": OY}, legacy_metres=True)
    x, y = 0.0, -4.0
    frozen = 0
    for _ in range(40):                                    # 4 s at 10 Hz
        st = State(x=x, y=y, up=40.0)
        d = sh.filter(st, Action4D(vx=0.0, vy=2.0))        # commanded at the NFZ
        if not _moving(d.emitted, 0.2):
            frozen += 1
        x += d.emitted.vx * 0.1
        y += d.emitted.vy * 0.1
    assert frozen == 0, f"stood still on {frozen}/40 ticks in the sliver"


# --------------------------------------------------------------- the standoff

def test_head_on_standoff_is_braking_sized_not_lookahead_sized():
    """A constant-velocity scan over the full 3 s geofence horizon turns a 5 m
    rule into a 17 m standoff at 4 m/s: the drone stopped dead 15.75 m from the
    wall and never got closer."""
    occ = np.zeros((N, N), np.uint8)
    occ[_ci(20):, :] = 1                                   # wall, face at x = 20
    sh = _shield(occ, min_clearance=5.0, soft=2.0)
    x, y, brakes = -20.0, 0.0, 0
    closest = 1e9
    for _ in range(200):                                   # 20 s at 10 Hz
        st = State(x=x, y=y, up=40.0)
        d = sh.filter(st, Action4D(vx=4.0, vy=0.0))        # straight at the wall
        brakes += d.braked
        x += d.emitted.vx * 0.1
        y += d.emitted.vy * 0.1
        closest = min(closest, sh._distance_at(x, y))
    gap = sh._distance_at(x, y)
    assert brakes == 0, f"braked {brakes}/200 ticks on a head-on approach"
    assert closest >= 5.0 - 1.2, f"breached the ring: closest {closest:.2f} m"
    assert gap < 12.0, f"still parked {gap:.2f} m out — standoff not fixed"


def test_planned_routes_are_flyable_with_the_shipped_policy():
    """End to end on the shipped artefacts: urban_clearance.yaml over
    occ_day.npz, from the demo's own spawn, following the demo's own planner.
    The measured brake rate before the fix was 95%, and 11/11 routes stalled."""
    import city_planner
    from shapely.geometry import Point

    from guardrail.geometry import fence_polygon
    from guardrail.models import PolygonFence

    sh, cm = _city_shield()
    # Same planning grid the demo builds: surveyed buildings + every policy NFZ
    # stamped in, then inflated once. Planning against buildings ALONE routes
    # straight through the no-fly zones, and the resulting stalls are the
    # geofence slide's, not the clearance rule's.
    occ, res = np.asarray(cm["occ"]).copy(), cm["res"]
    ox, oy = cm["ox"], cm["oy"]
    for f in sh.policy.by_type(PolygonFence):
        buf = fence_polygon(f).buffer(f.margin_m)
        minx, miny, maxx, maxy = buf.bounds
        for i in range(max(0, int((minx - ox) / res)),
                       min(occ.shape[0], int((maxx - ox) / res) + 1)):
            for j in range(max(0, int((miny - oy) / res)),
                           min(occ.shape[1], int((maxy - oy) / res) + 1)):
                if occ[i, j] == 0 and buf.contains(Point(ox + i * res, oy + j * res)):
                    occ[i, j] = 1
    grid = city_planner.inflate(occ, res, 6.0)
    rng = np.random.default_rng(11)
    start = (35.0, -20.0)

    # Sample goals, keep what the planner can actually reach, and fly the TEN
    # LONGEST routes — short hops would not exercise the rule enough to mean
    # anything. (Most of the map is unreachable from the spawn once the 6 m
    # inflation and the NFZs are in, so goals cannot simply be drawn far away.)
    def _plen(p):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p[:-1], p[1:]))

    found = []
    for _ in range(150):
        goal = (float(rng.integers(-70, 70)), float(rng.integers(-70, 70)))
        path = city_planner.plan(grid, res, ox, oy, start, goal)
        if path is None or len(path) < 2:
            continue
        pts = [(float(a), float(b)) for a, b in path]
        found.append((_plen(pts), pts))
    found.sort(key=lambda t: -t[0])
    assert len(found) >= 10, f"only {len(found)} routes planned — map/planner problem"

    def fly(drift):
        """Carrot follower at 4 m/s, 10 Hz, Shield in the loop. `drift` bends the
        commanded heading away from the planned corridor — the VLA wander the
        rule actually exists to catch (at drift = 0 the planner's own 6 m
        inflation should keep the Shield silent)."""
        brake = ticks = stalled = touched = frozen = 0
        closest = 1e9
        for _, pts in found[:10]:
            x, y, up, seg = pts[0][0], pts[0][1], 45.0, 1
            for k in range(900):                           # 90 s at 10 Hz
                while seg < len(pts) and math.hypot(pts[seg][0] - x, pts[seg][1] - y) < 3.0:
                    seg += 1
                if seg >= len(pts):
                    break
                ticks += 1
                gx, gy = pts[seg]
                ang = math.atan2(gy - y, gx - x) + drift * math.sin(k / 60.0)
                d = sh.filter(State(x=x, y=y, up=up),
                              Action4D(vx=4.0 * math.cos(ang), vy=4.0 * math.sin(ang)))
                brake += d.braked
                touched += d.touched
                frozen += not _moving(d.emitted, 0.2)
                x += d.emitted.vx * 0.1
                y += d.emitted.vy * 0.1
                up += d.emitted.vz_up * 0.1
                closest = min(closest, sh._distance_at(x, y))
            if seg < len(pts):
                stalled += 1
        return brake, ticks, stalled, touched, frozen, closest

    # 1) nominal: on its own planned corridor the Shield must stay out of the way
    brake, ticks, stalled, touched, frozen, closest = fly(0.0)
    print(f"    nominal: {ticks} ticks, brake {brake}, touched {touched}, "
          f"stalled {stalled}, closest {closest:.2f} m")
    assert ticks > 800, f"only {ticks} ticks flown — the sweep is too thin"
    assert brake == 0, f"braked {brake}/{ticks} ticks on planner-clean routes"
    assert stalled == 0, f"{stalled}/10 planned routes stalled"
    assert closest > 0.0, "flew through a building"

    # 2) drifting off the corridor: now the rule DOES fire, and it must steer
    #    rather than stop — this is where the old code braked 95% of ticks.
    brake, ticks, stalled, touched, frozen, closest = fly(1.5)
    rate = 100.0 * brake / max(1, ticks)
    print(f"    drifting: {ticks} ticks, brake {brake} ({rate:.1f}%), "
          f"touched {touched}, frozen {frozen}, closest {closest:.2f} m")
    assert touched > 500, f"only {touched} interventions — the drift did not bite"
    assert rate < 5.0, f"brake rate {rate:.1f}% >= 5%"
    assert frozen < 0.05 * ticks, f"stood still on {frozen}/{ticks} ticks"
    assert closest > 3.0, f"steered too close to a building ({closest:.2f} m)"


# ------------------------------------------------------------------ inertness

def test_no_map_leaves_clearance_rules_inert():
    """A policy that names obstacle_clearance but a Shield built without a map:
    the rule must never fire and never raise."""
    occ = np.zeros((N, N), np.uint8)
    occ[_ci(10):, :] = 1
    armed = _shield(occ, min_clearance=6.0)
    inert = Shield(armed.policy, lookahead_s=3.0, dt=0.5, obstacle_map=None)
    st = State(x=8.0, y=0.0, up=40.0)
    for a in (Action4D(vx=2.0), Action4D(vx=-2.0), Action4D(vy=3.0), Action4D()):
        d = inert.filter(st, a)
        assert not d.violations and not d.braked and d.emitted == a
    assert armed.filter(st, Action4D(vx=2.0)).touched     # ...but armed it fires


# ---- distance_envelope on the hil rail (de_hil1, 2026-11-19) ----

DE_POLICY = ROOT / "policies" / "distance_envelope_sitl.yaml"
DE_WORLD = ROOT / "experiments" / "worlds" / "de_block.yaml"


def _de_rail_shield(**kw):
    from guardrail.replay import rail_shield
    from guardrail.world_map import load_world
    world = load_world(DE_WORLD, ROOT)
    return rail_shield(load_policy(str(DE_POLICY)), lookahead_s=5.0, dt=0.1,
                       **world.shield_kwargs(), **kw)


def test_clearance_fix_converges_at_a_field_cell_boundary():
    """de_hil1_001 tick 176: the goto pilot rides the buildings envelope's
    5.0 contour at the building's NW corner (x = 40, a field cell boundary),
    where the gradient at the forecast hit is skewed to (+0.19, -0.98). The
    raw action already leaves along that normal at 0.45 m/s and its t+0.1
    sample reads 4.994 m.

    Before: ClearanceFix raised the outward part to the 0.5 m/s floor only
    (push = max(0.5, depth / lookahead_s), 5 s) and kept the tangential
    part, so the fix fell 0.5 mm short; pass 2 was a no-op ('5.00->5.00');
    ClearanceEscape rescued with (0, -4) and the FSM read it as 'no operator
    converged' (G3 Brake -> Loiter, then G5 RTL through the envelope). Now
    the push is sized to clear the shortfall at the offending sample's time
    and the first pass converges (one ClearanceFix). Tick 175's state is a
    guard only: it already converged on 0f4269a. (38.05, -3.55) pins
    _CLEAR_PUSH_MARGIN_MPS: with the exact first-order push (margin 0) the
    pass there falls short on the field's slope and the chain ends in
    ClearanceEscape; 0.05 m/s clears it (5 of 28,182 grid probes round the
    corner failed at 0, none at 0.05).

    The same no-op pass on a rule judged from the STOP POINT (_committed,
    a state with a measured velocity, the own-velocity forecast pas_ardupilot
    flies by default): the stop point is inside the ring and the action
    leaves it, so ClearanceFix raised the outward part to 0.50 m/s, which
    it already flew ('leaving at 0.50 m/s -> raised to 0.50'), while the
    check also judges the rule from the position on the commanded forecast,
    which enters the ring at t+0.6 s. Now the repair steers by that
    forecast, as it already did for the own-velocity one."""
    for xy, once in (((39.571, -3.438), True), ((39.171, -3.47), False),
                     ((38.05, -3.55), True)):
        sh = _de_rail_shield()
        st = State(x=xy[0], y=xy[1], up=15.0)
        d = sh.filter(st, Action4D(vx=3.99, vy=0.32))
        ops = [r.operator for r in d.repairs]
        assert "ClearanceEscape" not in ops, (xy, [(r.operator, r.detail) for r in d.repairs])
        assert not any("not converged" in (r.detail or "") for r in d.repairs), d.repairs
        assert "ClearanceFix" in ops, ops
        if once:
            assert ops.count("ClearanceFix") == 1, ops      # converged in pass 1
        assert not d.emitted_violations, d.emitted_violations
        assert not sh._check(st, d.emitted), (xy, d.emitted)
        # The pilot's progress is kept: still flying on along the contour.
        assert d.emitted.vx > 3.9, d.emitted
    from guardrail.response import ARDUCOPTER_GUIDED
    sh = _de_rail_shield(velocity_response=ARDUCOPTER_GUIDED)
    st = State(x=33.6, y=-3.6, up=15.0, velocity=(4.0, 0.32, 0.0))
    d = sh.filter(st, Action4D(vx=3.99, vy=0.32))
    ops = [r.operator for r in d.repairs]
    assert "ClearanceEscape" not in ops and ops.count("ClearanceFix") == 1, \
        [(r.operator, r.detail) for r in d.repairs]
    assert not d.emitted_violations and not sh._check(st, d.emitted), d.emitted
    assert d.emitted.vx > 3.9, d.emitted


def test_clear_horizon_is_sized_for_a_lagging_vehicle():
    """The commanded clearance forecast's horizon (_clear_horizon_s).

    No clear_response (every headless ideal vehicle, every caller before
    2026-11-19): unchanged, dt + v / (2 * 3 m/s^2) = 0.767 s at 4 m/s.
    With the ArduPilot response (0.45 s reaction, 1.75 m/s^2), as the
    ArduPilot rails now pass: dt + 0.45 + v / (2 * 1.75) = 1.69 s, about
    6.8 m, which covers the vehicle's 6.4 m stop (before: 0.77 s / 3.1 m,
    and both de_hil1 lanes entered their envelopes). A slowed repair is
    sized at the tick's request speed, so it cannot shrink its own horizon."""
    from guardrail.response import ARDUCOPTER_GUIDED, VelocityResponse
    plain = _de_rail_shield()
    assert plain.clear_response is None
    assert abs(plain._clear_horizon_s(Action4D(vx=4.0, vy=0.0)) - (0.1 + 4.0 / 6.0)) < 1e-9
    assert plain._clear_horizon_s(Action4D(vx=0.0, vy=0.0)) == 0.0
    lag = _de_rail_shield(clear_response=VelocityResponse(
        reaction_s=0.45, accel_mps2=1.75, source="test"))
    h = lag._clear_horizon_s(Action4D(vx=4.0, vy=0.0))
    assert h >= 1.6, h
    assert abs(h - (0.1 + 0.45 + 4.0 / 3.5)) < 1e-9, h
    assert lag._clear_horizon_s(Action4D(vx=0.0, vy=0.0)) == 0.0
    # Inside a tick whose request is 4 m/s, a 0.5 m/s repair keeps the
    # request's horizon.
    tick = lag._new_tick(None)
    tick.clear_v = 4.0
    with lag._scope(tick):
        assert abs(lag._clear_horizon_s(Action4D(vx=0.5, vy=0.0)) - h) < 1e-9
    # The rails' value is the Shield's own fitted response.
    assert (ARDUCOPTER_GUIDED.reaction_s, ARDUCOPTER_GUIDED.accel_mps2) == (0.45, 1.75)
    try:
        _de_rail_shield(clear_response=(0.45, 1.75))
    except TypeError:
        pass
    else:
        raise AssertionError("a clear_response that is not a VelocityResponse was accepted")


def test_clear_horizon_reads_the_setpoints_still_in_the_pipe():
    """filter() sizes the clearance horizon (_Tick.clear_v) at the larger of
    the request and EVERY setpoint the vehicle still flies within its
    reaction time (pipe_segments over note_flown's record, or over what
    filter() emitted when the rail notes nothing), never at the Shield's own
    slowed repair: three 4 m/s ticks, then a 0.5 m/s request at x = 15.1,
    whose own horizon (0.69 s, 0.35 m) stops short of the buildings
    envelope. Sized at 4 m/s (1.69 s, 0.85 m at 0.5 m/s) it enters it: the
    vehicle is still flying 4 m/s. Once the 4 m/s setpoints have left the
    pipe (one reaction time after the last), and after reset_episode, the
    request's own horizon again."""
    from guardrail.response import ARDUCOPTER_GUIDED as R
    h4 = 0.1 + R.reaction_s + 4.0 / (2.0 * R.accel_mps2)
    h05 = 0.1 + R.reaction_s + 0.5 / (2.0 * R.accel_mps2)
    near, slow = State(x=15.1, y=-1.0, up=15.0), Action4D(vx=0.5)
    fresh = _de_rail_shield(clear_response=R)
    assert not fresh.filter(near, slow, t=0.3).violations      # its own horizon: clean
    for noted in (False, True):
        sh = _de_rail_shield(clear_response=R)
        seen: list = []
        own = sh._clear_horizon_s
        sh._clear_horizon_s = lambda a, _o=own, _s=seen: _s.append(_o(a)) or _s[-1]
        for t in (0.0, 0.1, 0.2):
            d = sh.filter(State(x=0.0, y=-1.0, up=15.0), Action4D(vx=4.0), t=t)
            assert not d.touched, d.repairs
            if noted:
                sh.note_flown(d.emitted, t=t)
        seen.clear()
        d = sh.filter(near, slow, t=0.3)
        assert [v.rule_id for v in d.violations] == ["de-buildings"], (noted, d.violations)
        assert abs(max(seen) - h4) < 1e-9, (noted, max(seen), h4)
        # The repair slowed it; a later 0.5 m/s request is still sized at
        # 4 m/s while the 0.2 s setpoint is in the pipe ...
        if noted:
            sh.note_flown(d.emitted, t=0.3)
        seen.clear()
        sh.filter(State(x=0.0, y=-1.0, up=15.0), slow, t=0.4)
        assert abs(max(seen) - h4) < 1e-9, (noted, max(seen))
        if noted:
            sh.note_flown(slow, t=0.4)
        # ... and not after it: the 0.2 s setpoint acted until 0.75 s.
        seen.clear()
        sh.filter(State(x=0.0, y=-1.0, up=15.0), slow, t=1.0)
        assert max(seen) < h4 - 0.5, (noted, max(seen))
        sh.reset_episode()
        seen.clear()
        assert not sh.filter(near, slow, t=0.3).violations, noted
        assert abs(max(seen) - h05) < 1e-9, (noted, max(seen), h05)


def test_clearance_theta_reads_the_depth_an_instant_vehicle_reaches():
    """ClearanceFix's magnitude_m, which the FSM's theta (2.0 m lateral)
    reads, under a clear_response: the depth over the horizon of a vehicle
    that obeys at once (_clear_theta_depth), not over the longer one. At
    (13.3, -1) the 4 m/s goto pilot's forecast over the lagging horizon
    (1.69 s, 6.8 m) reaches 2.23 m into the buildings envelope, over theta,
    with the aircraft still about 3 m short of the ring: the headless rail
    arm took G3 there, a Loiter the same reading never let G8 hand back and
    G5 RTL on a lane that should pass. Without a clear_response the reading
    is the one every caller had (here 0.50 m)."""
    from guardrail.response import ARDUCOPTER_GUIDED
    st, req = State(x=13.3, y=-1.0, up=15.0), Action4D(vx=4.0)
    lag = _de_rail_shield(clear_response=ARDUCOPTER_GUIDED)
    d = lag.filter(st, req)
    fix = [r for r in d.repairs if r.operator == "ClearanceFix"]
    assert len(fix) == 1 and not fix[0].recovery, d.repairs
    f = lag._field_for(lag._clear[0])
    assert 5.0 - lag._clear_min_dist(st, req, f) > 2.0      # the long horizon's depth
    assert fix[0].magnitude_m < 0.5, fix[0]
    plain = _de_rail_shield()
    pfix = [r for r in plain.filter(st, req).repairs if r.operator == "ClearanceFix"]
    assert len(pfix) == 1 and abs(pfix[0].magnitude_m - 0.495) < 0.01, pfix
    assert fix[0].magnitude_m <= pfix[0].magnitude_m + 1e-9, (fix, pfix)


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
            print(f"FAIL  {fn.__name__}  {e}")
        except Exception as e:                       # noqa: BLE001
            # Any crash is a fail, not the end of the run. Without this arm
            # an ImportError or a numpy error in one test propagates out of
            # the loop, every remaining test is silently skipped, and the
            # summary line never prints - so the file looks like it ran
            # clean when most of it never executed. pytest runs them all, so
            # the two invocation modes disagreed about coverage.
            failed += 1
            print(f"ERROR {fn.__name__}  {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
