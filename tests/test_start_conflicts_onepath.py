"""models.Policy.start_conflicts asks the Shield (X-13, 2026-11-27).

Run either way:
    pytest tests/test_start_conflicts_onepath.py -v
    python tests/test_start_conflicts_onepath.py

WHY THIS FILE EXISTS

The grant's 'Violation checking' section says there is exactly one
rule-evaluation code path in the system. Until 2026-11-27
`Policy.start_conflicts` (guardrail/models.py) made its own point-in-zone and
margin test: the even-odd ring test with the boundary counted, the edge
distance against the margin with `<=`, bands and schedules ignored. Through
it tools/geo_to_policy.py refuses a take-off point inside a zone, and every
flight loader records the frame origin's conflicts in `runtime_notes`.

It now asks guardrail/shield.py's `zone_footprint_reached`, the closed form of
`zone_breached`'s footprint half. That verdict must give the answer the old
test gave everywhere, or a policy that loaded cleanly would start refusing
take-off points (or stop refusing them). So the old code is kept HERE, only
here, as the oracle, and the two are compared:

- on every hard keep-out zone of every shipped policy (policies/*.yaml,
  experiments/policies/*.yaml), at its vertices, on its edges, exactly on its
  margin ring (off each edge's interior along the normal, and round each
  vertex), a hair either side of the ring, and over a grid that covers the
  zone and its ring;
- at every take-off point the scenario library flies (experiments/
  scenarios.yaml, expanded) and the CityLife spawn, against every zone;
- on random rings with random margins, 0 included, where the boundary test
  (an edge within 1e-9) decides;
- and `start_conflicts()` itself, string for string, on every shipped policy.

Nulls: the grid holds points where `<` instead of `<=` would answer
differently (it reaches the ring exactly), and where the old boundary test
decides a margin-0 zone; and the closed verdict is never less strict than the
Shield's `zone_breached` at a height inside the band.
"""
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

from guardrail.ir import compile_fence  # noqa: E402
from guardrail.models import DynamicNFZ, Policy, PolygonFence, load_policy  # noqa: E402
from guardrail.shield import zone_breached, zone_footprint_reached  # noqa: E402

HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}


# --------------------------------------------------------------- the oracle
#
# guardrail/models.py at 78b50fd, as start_conflicts ran it: _orient,
# _on_segment, _segments_meet, _dedupe_ring, _point_in_ring (whose even-odd
# half is planar.point_in_ring_xy, written out) and _dist_to_ring
# (planar.ring_edge_distance, written out), and the method itself.

def _old_orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _old_on_segment(a, b, p, eps):
    return (min(a[0], b[0]) - eps <= p[0] <= max(a[0], b[0]) + eps
            and min(a[1], b[1]) - eps <= p[1] <= max(a[1], b[1]) + eps)


def _old_segments_meet(a, b, c, d, eps=1e-9):
    o1, o2 = _old_orient(a, b, c), _old_orient(a, b, d)
    o3, o4 = _old_orient(c, d, a), _old_orient(c, d, b)
    if ((o1 > eps and o2 < -eps) or (o1 < -eps and o2 > eps)) and \
            ((o3 > eps and o4 < -eps) or (o3 < -eps and o4 > eps)):
        return True
    return ((abs(o1) <= eps and _old_on_segment(a, b, c, eps))
            or (abs(o2) <= eps and _old_on_segment(a, b, d, eps))
            or (abs(o3) <= eps and _old_on_segment(c, d, a, eps))
            or (abs(o4) <= eps and _old_on_segment(c, d, b, eps)))


def _old_dedupe_ring(pts):
    out = []
    for p in pts:
        if not out or out[-1] != p:
            out.append(p)
    while len(out) >= 2 and out[0] == out[-1]:
        out.pop()
    return out


def _old_even_odd(x, y, verts):
    inside = False
    n = len(verts)
    for i in range(n):
        (x1, y1), (x2, y2) = verts[i], verts[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xc = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xc:
                inside = not inside
    return inside


def _old_point_in_ring(x, y, ring):
    n = len(ring)
    if any(_old_segments_meet(ring[i], ring[(i + 1) % n], (x, y), (x, y)) for i in range(n)):
        return True
    return _old_even_odd(x, y, ring)


def _old_segment_distance(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def _old_dist_to_ring(x, y, verts):
    best = float("inf")
    n = len(verts)
    for i in range(n):
        (ax, ay), (bx, by) = verts[i], verts[(i + 1) % n]
        best = min(best, _old_segment_distance(x, y, ax, ay, bx, by))
    return best


def old_reached(c, x, y, cmp=lambda d, m: d <= m):
    """The old per-zone test at (x, y), or None where it skipped the zone
    (fewer than three distinct vertices). `cmp` is the margin comparison
    (`<=` as shipped; the `<` null swaps it)."""
    ring = _old_dedupe_ring([(v.x, v.y) for v in c.vertices])
    if len(ring) < 3:
        return None
    m = max(c.margin_m, 0.0)
    return _old_point_in_ring(x, y, ring) or cmp(_old_dist_to_ring(x, y, ring), m)


def old_start_conflicts(pol):
    out = []
    for c in pol.constraints:
        if not isinstance(c, (PolygonFence, DynamicNFZ)) or c.constraint_type != "hard":
            continue
        ring = _old_dedupe_ring([(v.x, v.y) for v in c.vertices])
        if len(ring) < 3:
            continue
        m = max(c.margin_m, 0.0)
        if _old_point_in_ring(0.0, 0.0, ring) or _old_dist_to_ring(0.0, 0.0, ring) <= m:
            out.append(
                f"{c.id}: the frame origin (0, 0) lies inside this hard "
                f"keep-out zone's footprint (margin {m:g} m, band "
                f"{c.altitude_floor_m:g}-{c.altitude_ceiling_m:g} m, schedule "
                f"ignored); a vehicle that starts at the frame origin (the "
                f"ArduPilot SITL rail: home) starts inside the zone at every "
                f"height of its band")
    return out


# --------------------------------------------------------------- the inputs

def shipped_policy_files():
    return sorted(list((ROOT / "policies").glob("*.yaml"))
                  + list((ROOT / "experiments" / "policies").glob("*.yaml")))


_POLICIES = None


def shipped_policies():
    """{relative path: Policy} for every shipped policy, loaded as written
    (runtime=False: bundling and the lint load them so; a policy a flight
    refuses still states its zones)."""
    global _POLICIES
    if _POLICIES is None:
        _POLICIES = {}
        for f in shipped_policy_files():
            _POLICIES[f.relative_to(ROOT).as_posix()] = load_policy(f, runtime=False)
    return _POLICIES


def hard_zones(pol):
    return [c for c in pol.constraints
            if isinstance(c, (PolygonFence, DynamicNFZ)) and c.constraint_type == "hard"]


def probe_points(c, grid_n=30):
    """Vertices; quarter, mid and three-quarter points of every edge; each
    of those moved along the edge's normal, both sides, by exactly the
    margin, by the margin +/- 1e-6 m and by the margin + 1e-12 m; 16 points
    exactly on the margin circle round every vertex; a (grid_n + 1)^2 grid
    over the zone's box grown by the margin and 2 m; and the frame origin."""
    ring = _old_dedupe_ring([(v.x, v.y) for v in c.vertices])
    m = max(c.margin_m, 0.0)
    n = len(ring)
    out = [(0.0, 0.0)]
    for i in range(n):
        (ax, ay), (bx, by) = ring[i], ring[(i + 1) % n]
        out.append((ax, ay))
        L = math.hypot(bx - ax, by - ay)
        nx, ny = (by - ay) / L, -(bx - ax) / L
        for t in (0.25, 0.5, 0.75):
            px, py = ax + t * (bx - ax), ay + t * (by - ay)
            out.append((px, py))
            for s in (1.0, -1.0):
                for d in (m, m + 1e-6, m - 1e-6, m + 1e-12):
                    out.append((px + s * d * nx, py + s * d * ny))
        for k in range(16):
            th = 2.0 * math.pi * k / 16
            out.append((ax + m * math.cos(th), ay + m * math.sin(th)))
    xs, ys = [p[0] for p in ring], [p[1] for p in ring]
    lo_x, hi_x = min(xs) - m - 2.0, max(xs) + m + 2.0
    lo_y, hi_y = min(ys) - m - 2.0, max(ys) + m + 2.0
    for i in range(grid_n + 1):
        for j in range(grid_n + 1):
            out.append((lo_x + (hi_x - lo_x) * i / grid_n, lo_y + (hi_y - lo_y) * j / grid_n))
    return out


# The CityLife (classic AirSim) scenarios' spawn (demo/gen_random_scenario.py
# SPAWN; Policy.start_conflicts' docstring).
CITYLIFE_SPAWN = (35.0, -20.0)


def library_takeoff_points():
    """{(x, y)} of every start_pose the scenario library expands to (the
    local frame; a lat/lon pose would need the policy's origin)."""
    from guardrail.scenario_spec import Library, expand
    eps, _ = expand(Library(ROOT / "experiments" / "scenarios.yaml"))
    pts = set()
    for e in eps:
        p = e.cell.spec.mission.start_pose
        assert p.x is not None and p.y is not None, (e.episode_id, p)
        pts.add((float(p.x), float(p.y)))
    return pts


# --------------------------------------------------------------- the tests

def test_every_shipped_policy_loads_and_states_hard_zones():
    files = shipped_policy_files()
    pols = shipped_policies()
    assert len(pols) == len(files) and len(files) >= 30, (len(pols), len(files))
    n = sum(len(hard_zones(p)) for p in pols.values())
    assert n >= 50, n
    print(f"      {len(files)} shipped policies, {n} hard keep-out zones")


def test_the_shields_verdict_is_the_old_test_on_every_shipped_zone():
    """Point for point, on every hard zone of every shipped policy: the
    vertices, the edges, exactly on the margin ring and a hair either side,
    and a grid over the zone and its ring."""
    n = 0
    bad = []
    for rel, pol in shipped_policies().items():
        for c in hard_zones(pol):
            for x, y in probe_points(c):
                want = old_reached(c, x, y)
                got = zone_footprint_reached(c, x, y)
                n += 1
                if (want is True) != got:
                    bad.append(f"{rel} {c.id} ({x!r}, {y!r}): old {want}, Shield {got}")
    assert not bad, f"{len(bad)} of {n} points differ:\n      " + "\n      ".join(bad[:20])
    assert n >= 50_000, n
    print(f"      {n} points agree")


def test_the_grid_reaches_the_margin_ring_exactly():
    """Null for the test above: on the shipped zones' probe points, `<` in
    place of `<=` answers differently somewhere (so the points sit exactly on
    the ring, where the comparison matters), and the closed verdict reaches
    points the Shield's open margin ring leaves clear."""
    strict = 0
    looser = 0
    for pol in shipped_policies().values():
        for c in hard_zones(pol):
            rec = compile_fence(0, c)
            for x, y in probe_points(c):
                if old_reached(c, x, y) != old_reached(c, x, y, cmp=lambda d, m: d < m):
                    strict += 1
                if zone_footprint_reached(c, x, y) and not zone_breached(
                        c, x, y, c.altitude_floor_m, rec.polygon, rec.buffered):
                    looser += 1
    assert strict > 0, "no probe point lies exactly on a margin ring"
    assert looser > 0
    print(f"      {strict} points where `<` would differ, {looser} reached only by "
          f"the closed verdict")


def test_the_closed_verdict_is_never_less_strict_than_zone_breached():
    """At a height inside the band (floor, middle, ceiling), every point the
    Shield's zone_breached reports breached is reached by
    zone_footprint_reached, on every shipped zone's probe points, up to float
    rounding on the ring itself: a point of the 16 round each vertex can sit
    an ulp inside a vertex of the margin ring GEOS builds while its exact
    edge distance rounds an ulp over the margin. Those are counted and held
    within 1e-12 m of the ring."""
    n = 0
    rounding = 0
    bad = []
    for rel, pol in shipped_policies().items():
        for c in hard_zones(pol):
            rec = compile_fence(0, c)
            ring = _old_dedupe_ring([(v.x, v.y) for v in c.vertices])
            m = max(c.margin_m, 0.0)
            ups = (c.altitude_floor_m, 0.5 * (c.altitude_floor_m + c.altitude_ceiling_m),
                   c.altitude_ceiling_m)
            for x, y in probe_points(c):
                if zone_footprint_reached(c, x, y):
                    continue
                over = _old_dist_to_ring(x, y, ring) - m
                for up in ups:
                    n += 1
                    if zone_breached(c, x, y, up, rec.polygon, rec.buffered):
                        if over <= 1e-12:
                            rounding += 1
                        else:
                            bad.append(f"{rel} {c.id} ({x!r}, {y!r}, {up!r}): {over} m out")
    assert not bad, "zone_breached but not reached:\n      " + "\n      ".join(bad[:20])
    assert n > 0
    print(f"      {n} (point, height) pairs outside the footprint; {rounding} breached "
          f"by float rounding on the ring (<= 1e-12 m), none further out")


def test_the_scenario_librarys_takeoff_points_get_the_old_answer():
    """Every take-off point the scenario library flies, and the CityLife
    spawn, against every hard zone of every shipped policy."""
    pts = library_takeoff_points() | {CITYLIFE_SPAWN, (0.0, 0.0)}
    assert len(pts) >= 20, len(pts)
    n = 0
    hits = 0
    bad = []
    for rel, pol in shipped_policies().items():
        for c in hard_zones(pol):
            for x, y in sorted(pts):
                want = old_reached(c, x, y)
                got = zone_footprint_reached(c, x, y)
                n += 1
                hits += bool(got)
                if (want is True) != got:
                    bad.append(f"{rel} {c.id} ({x}, {y}): old {want}, Shield {got}")
    assert not bad, "\n      ".join(bad)
    print(f"      {len(pts)} take-off points x every zone: {n} answers agree "
          f"({hits} inside a footprint)")


def test_start_conflicts_is_the_old_list_on_every_shipped_policy():
    n = 0
    for rel, pol in shipped_policies().items():
        assert pol.start_conflicts() == old_start_conflicts(pol), rel
        n += len(old_start_conflicts(pol))
    assert n > 0, "no shipped policy has a zone over the frame origin"
    print(f"      {len(shipped_policies())} policies, {n} start conflicts, string for string")


def _zone(ring, margin, zid="z"):
    return Policy.model_validate({"policy_id": "p", "constraints": [{
        "id": zid, "type": "polygon_fence", **HARD_P0, "margin_m": margin,
        "altitude_floor_m": 0.0, "altitude_ceiling_m": 120.0,
        "vertices": [{"x": x, "y": y} for x, y in ring]}]}).constraints[0]


def _rand_ring(rng):
    n = rng.randint(3, 9)
    cx, cy = rng.uniform(-30, 30), rng.uniform(-30, 30)
    angs = sorted(rng.uniform(0, 2 * math.pi) for _ in range(n))
    return [(cx + r * math.cos(a), cy + r * math.sin(a))
            for a, r in ((a, rng.uniform(3, 20)) for a in angs)]


def test_random_rings_and_margins_get_the_old_answer_boundary_included():
    """Random star-shaped rings, margins 0 to 8 m (a third of them 0): at
    the probe points and at points within 1e-10 m of an edge, where on a
    margin-0 zone only the old boundary test (an edge within 1e-9) reaches
    them. The null: that case occurs."""
    rng = random.Random(1127)
    n = 0
    edge_decided = 0
    for k in range(150):
        ring = _rand_ring(rng)
        margin = 0.0 if k % 3 == 0 else round(rng.uniform(0.1, 8.0), 3)
        try:
            c = _zone(ring, margin)
        except ValueError:
            continue                     # a ring the lint refuses (not simple)
        extra = []
        for i in range(len(ring)):
            (ax, ay), (bx, by) = ring[i], ring[(i + 1) % len(ring)]
            L = math.hypot(bx - ax, by - ay)
            nx, ny = (by - ay) / L, -(bx - ax) / L
            for t in (0.1, 0.5, 0.9):
                px, py = ax + t * (bx - ax), ay + t * (by - ay)
                extra += [(px + 1e-10 * nx, py + 1e-10 * ny),
                          (px - 1e-10 * nx, py - 1e-10 * ny)]
        for x, y in probe_points(c, grid_n=12) + extra:
            want = old_reached(c, x, y)
            assert zone_footprint_reached(c, x, y) == (want is True), (k, x, y, margin)
            n += 1
            if margin == 0.0 and want and not _old_even_odd(x, y, _old_dedupe_ring(ring)) \
                    and _old_dist_to_ring(x, y, _old_dedupe_ring(ring)) > 0.0:
                edge_decided += 1
    assert n > 10_000, n
    assert edge_decided > 0, "no point where only the boundary test decides"
    print(f"      {n} points agree, {edge_decided} decided by the boundary test alone")


def test_a_ring_of_fewer_than_three_distinct_vertices_is_no_footprint():
    c = PolygonFence.model_construct(
        id="deg", type="polygon_fence", vertices=[
            type("V", (), {"x": 0.0, "y": 0.0})(), type("V", (), {"x": 1.0, "y": 0.0})(),
            type("V", (), {"x": 1.0, "y": 0.0})(), type("V", (), {"x": 0.0, "y": 0.0})()],
        margin_m=5.0)
    assert old_reached(c, 0.0, 0.0) is None
    assert zone_footprint_reached(c, 0.0, 0.0) is False


def test_start_conflicts_asks_the_shield():
    """The method holds no geometry of its own: it imports the Shield's
    verdict (lazily: shield imports models) and calls it."""
    import ast
    import inspect
    import textwrap
    src = textwrap.dedent(inspect.getsource(Policy.start_conflicts))
    tree = ast.parse(src)
    called = {n.func.id if isinstance(n.func, ast.Name) else getattr(n.func, "attr", None)
              for n in ast.walk(tree) if isinstance(n, ast.Call)}
    assert "zone_footprint_reached" in called, called
    assert not called & {"_point_in_ring", "_dist_to_ring", "ring_edge_distance",
                         "point_in_ring_xy", "_dedupe_ring"}, called
    imports = [n for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
    assert any(n.module == "shield" and n.level == 1 for n in imports), \
        [(n.module, n.level) for n in imports]


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
