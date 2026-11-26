"""X-11 (2026-11-17): MSL rules over a gridded DEM, and MSL in the CSP.

guardrail/terrain.py GridGround is a ground whose elevation varies with
position (a JSON grid, bilinear). The Shield converts an MSL band at the
QUERY POINT over the lowest / highest ground within the DEM's reach
(Shield._local_runtime): a keep-in band narrows, a keep-out band widens, so a
pose anywhere in reach is judged at least as strictly as the true MSL rule.
The CSP compiles an MSL rule on a compiler given a ground (the opt-in) and
says, in its sentence, the declared MSL band and the ground used.

Opt-in: an AGL policy, or an MSL policy over a flat ground, is unchanged
(the same decisions as the master code's conversion), and a CSP compiled
without a ground still refuses an MSL rule.

Run: python tests/test_msl_dem.py
"""
from __future__ import annotations

import itertools
import json
import math
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from guardrail import terrain                                           # noqa: E402
from guardrail.compiler import ConstraintCompiler, Mission              # noqa: E402
from guardrail.csp import CSPUnenforcedRules                            # noqa: E402
from guardrail.models import Action4D, State, policy_from_raw           # noqa: E402
from guardrail.shield import Shield                                     # noqa: E402
from wgs84_fixture import geo, geo_ring, wgs84_doc             # noqa: E402

H = {"scope": "mission", "constraint_type": "hard", "priority": "P0",
     "violation_action": "project_fix"}

# The frame origin every fixture states (the Project AirSim home-geo-point):
# since 2026-11-18 (WP1-14) an event's metres are written as WGS84 about it
# and a policy with no frame origin refuses them; since 2026-11-26 a policy
# document is loaded in WGS84 only, so the fixtures, laid out in metres, are
# written in WGS84 about it (wgs84_doc) and their DEM is placed in it.
FRAME = (35.6895, 139.6917)


def _slope_dem(reach=10.0, origin="frame"):
    """Ground rising 0.5 m per metre east from 100 m MSL at x = 0; `origin`
    the WGS84 frame it is placed in (default FRAME, the fixtures' frame since
    2026-11-26; None: a metre-only world)."""
    origin = FRAME if origin == "frame" else origin
    xs = [-50.0 + 10.0 * i for i in range(21)]
    elev = [[100.0 + 0.5 * (x + 50.0) for x in xs] for _ in range(11)]
    return terrain.GridGround(-50.0, -50.0, 10.0, elev, "test slope", reach_m=reach,
                              origin=origin)


def _raw(ref="MSL", env=(150.0, 250.0), zone=(0.0, 180.0), origin="frame"):
    env_r = {"id": "env", "type": "altitude_envelope", **H,
             "alt_min_m": env[0], "alt_max_m": env[1]}
    zone_r = {"id": "zone", "type": "polygon_fence", **H, "margin_m": 1.0,
              "vertices": [{"x": 60, "y": -10}, {"x": 80, "y": -10},
                           {"x": 80, "y": 10}, {"x": 60, "y": 10}],
              "altitude_floor_m": zone[0], "altitude_ceiling_m": zone[1]}
    if ref:
        env_r["altitude_ref"] = zone_r["altitude_ref"] = ref
    out = {"policy_id": "msl-dem-test", "constraints": [env_r, zone_r]}
    origin = FRAME if origin == "frame" else origin
    if origin is not None:
        out["origin"] = {"lat": origin[0], "lon": origin[1]}
    return out


def test_grid_ground_is_bilinear_bounded_and_round_trips():
    g = _slope_dem()
    assert g.elevation_at(0.0, 0.0) == 125.0
    assert abs(g.elevation_at(3.0, 7.0) - 126.5) < 1e-9
    assert g.elevation_at(-500.0, 0.0) == 100.0, "outside the grid: the edge node"
    assert g.bounds() == (100.0, 200.0)
    # Reach 10 m about x = 0: the nodes x -10 ... 10 -> ground 120 ... 130.
    assert g.band_at(0.0, 0.0) == (120.0, 130.0)
    assert g.band_at(1.0, 0.0) == (120.0, 135.0), "a reach box touching a cell takes its nodes"
    # Rounded OUTWARD to the 0.5 m quantum.
    h = terrain.GridGround(0, 0, 1.0, [[10.2, 10.7], [10.3, 11.1]], "q", reach_m=0.0)
    assert h.band_at(0.0, 0.0) == (10.0, 10.5)
    assert h.bounds() == (10.0, 11.5)
    d = Path(tempfile.mkdtemp()) / "dem.json"
    d.write_text(json.dumps(g.to_dict()), encoding="utf-8")
    g2 = terrain.GridGround.load(d)
    assert g2.record() == g.record() and g2.record()["kind"] == "grid"
    for bad in ([[1.0]], [[1.0, float("nan")], [1.0, 1.0]]):
        try:
            terrain.GridGround(0, 0, 1.0, bad, "x")
        except ValueError:
            continue
        raise AssertionError(f"accepted a bad grid {bad}")


def test_keep_in_narrows_and_keep_out_widens_over_the_band():
    pol = policy_from_raw(wgs84_doc(_raw()), ground=_slope_dem())
    env, zone = pol.constraints
    e = terrain.agl_rule(env, _slope_dem(), (120.0, 130.0))
    z = terrain.agl_rule(zone, _slope_dem(), (120.0, 130.0))
    assert (e.alt_min_m, e.alt_max_m) == (30.0, 120.0)        # 150-120, 250-130
    assert (z.altitude_floor_m, z.altitude_ceiling_m) == (-130.0, 60.0)  # 0-130, 180-120
    assert e.altitude_ref is None and e.id == "env"
    # Flat ground: both readings are the plain shift (the master conversion).
    flat = terrain.FlatGround(120.0, "t")
    assert (terrain.agl_rule(env, flat).alt_min_m, terrain.agl_rule(zone, flat)
            .altitude_ceiling_m) == (30.0, 60.0)


def test_a_dem_shield_is_never_looser_than_the_true_msl_rule():
    """Every (pose, action) whose TRUE MSL forecast leaves the envelope over
    the DEM, the aircraft holding its MSL height, is flagged by the Shield at
    the query point; and none of those it passes breaks the rule."""
    g = _slope_dem(reach=12.0)
    sh = Shield(policy_from_raw(wgs84_doc(_raw()), ground=g), ground=g, up_ref="terrain", escalation=False,
                lookahead_s=2.0, dt=0.5)
    missed = flagged = 0
    for x, z, vx, vz in itertools.product((-40.0, -10.0, 0.0, 15.0, 30.0),
                                          (152.0, 175.0, 230.0, 248.0),
                                          (-3.0, 0.0, 3.0), (-2.0, 0.0, 2.0)):
        up = z - g.elevation_at(x, 30.0)
        d = sh.filter(State(x=x, y=30.0, up=up), Action4D(vx=vx, vz_up=vz))
        hit = any(v.rule_id == "env" for v in d.violations)
        flagged += hit
        # Truth: the MSL height over the forecast, under either physics.
        bad = any(not (150.0 <= z + vz * t <= 250.0) for t in (0.5, 1.0, 1.5, 2.0))
        if bad and not hit:
            missed += 1
    assert missed == 0 and flagged > 0, (missed, flagged)


def test_the_band_is_read_at_the_query_point_not_the_whole_dem():
    """Over the low end of the slope the ceiling is far higher than over the
    whole DEM's highest ground: a pose legal there is not flagged."""
    g = _slope_dem(reach=10.0)
    sh = Shield(policy_from_raw(wgs84_doc(_raw()), ground=g), ground=g, up_ref="terrain",
                escalation=False)
    # 240 m MSL at x = -40 (ground 105): up 135; the local ceiling is
    # 250 - 115 = 135 (legal), the whole DEM's 250 - 200 = 50 (illegal).
    st = State(x=-40.0, y=0.0, up=240.0 - g.elevation_at(-40.0, 0.0))
    assert not sh.state_is_unsafe(st), "judged over the whole DEM, not the query point"
    assert sh._rt.alts[0].alt_max_m == 50.0, "the base runtime is the whole DEM's"
    d = sh.filter(st, Action4D())
    assert not d.violations, d.violations
    # Pure and cached: the same point twice is the same runtime object.
    a = sh._local_runtime(sh._rt, -40.0, 0.0)
    assert a is sh._local_runtime(sh._rt, -40.0, 0.0)
    assert a.ir.policy_hash == sh.policy.policy_hash


def test_an_msl_zone_band_follows_the_ground_and_hot_apply_works_over_a_dem():
    g = _slope_dem(reach=5.0, origin=FRAME)
    sh = Shield(policy_from_raw(wgs84_doc(_raw(origin=FRAME)), ground=g), ground=g, up_ref="terrain",
                escalation=False)
    # Zone 0-180 m MSL over x 60..80 (ground 155..165): at 170 MSL the
    # aircraft is in the band and must be stopped short; at 190 it overflies.
    for z, blocked in ((170.0, True), (190.0, False)):
        st = State(x=50.0, y=0.0, up=z - g.elevation_at(50.0, 0.0))
        d = sh.filter(st, Action4D(vx=3.0))
        assert any(v.rule_id == "zone" for v in d.violations) is blocked, (z, d.violations)
    from guardrail.models import DynamicNFZ
    zone = DynamicNFZ(id="dz", type="dynamic_nfz", **H, altitude_ref="MSL",
                      vertices=[{"x": -30, "y": -5}, {"x": -20, "y": -5},
                                {"x": -20, "y": 5}, {"x": -30, "y": 5}],
                      altitude_floor_m=0.0, altitude_ceiling_m=150.0)
    sh.hot_apply(geo(zone, sh))
    # Ground ~110 there: 140 MSL is inside the zone's band, 160 above it.
    inside = {r["id"]: r for r in sh.rule_status(State(x=-25.0, y=0.0, up=30.0))}
    above = {r["id"]: r for r in sh.rule_status(State(x=-25.0, y=0.0, up=50.0))}
    assert inside["dz"]["breach"] and not above["dz"]["breach"], (inside["dz"], above["dz"])


def test_agl_and_flat_msl_policies_decide_as_before():
    """Opt-in: no DEM, no local runtime - the runtime has no MSL entries and
    every decision is the plain one (flat MSL = the AGL policy shifted)."""
    flat = terrain.FlatGround(100.0, "t")
    s_flat = Shield(policy_from_raw(wgs84_doc(_raw()), ground=flat), ground=flat, escalation=False)
    s_agl = Shield(policy_from_raw(wgs84_doc(_raw(ref=None, env=(50.0, 150.0), zone=(-100.0, 80.0)))),
                   escalation=False)
    assert s_flat._rt.msl == () and s_agl._rt.msl == ()
    n = 0
    for x, up, vx, vz in itertools.product((40.0, 55.0, 70.0), (45.0, 60.0, 79.0, 149.0),
                                           (0.0, 3.0), (-2.0, 0.0, 2.0)):
        st, a = State(x=x, y=0.0, up=up), Action4D(vx=vx, vz_up=vz)
        d1, d2 = s_flat.filter(st, a), s_agl.filter(st, a)
        assert d1.emitted == d2.emitted and \
            [v.rule_id for v in d1.violations] == [v.rule_id for v in d2.violations], (st, a)
        n += 1
    assert n == 72


def test_the_csp_compiles_msl_over_a_ground_and_states_it():
    g = _slope_dem()
    pol = policy_from_raw(wgs84_doc(_raw()), ground=g)
    m = Mission(task_text="fly east", start_x=-40.0, start_y=0.0, target_x=40.0,
                target_y=0.0, cruise_alt_m=60.0, speed_pref_mps=3.0)
    try:
        ConstraintCompiler(pol).compile_csp(m)
    except CSPUnenforcedRules as e:
        assert set(e.rules) == {"env", "zone"}, e.rules
    else:
        raise AssertionError("an MSL rule compiled without a ground")
    csp = ConstraintCompiler(pol, ground=g).compile_csp(m)
    assert csp.policy_hash == pol.policy_hash, "the declared policy's hash"
    text = csp.natural_language_prompt
    # The ground the mission can reach (x -40 ... 40 by the 10 m reach: ground
    # 100-150 m), not the whole DEM's 100-200 m: keep-in 150-250 MSL -> 50-100.
    assert "Stay between 50 m and 100 m altitude (heights above ground: declared " \
           "150-250 m MSL over ground at 100-150 m MSL under the mission)" in text, text
    assert "declared 0-180 m MSL over ground at 100-150 m MSL under the mission" in text, text
    # A flat ground states its one elevation.
    flat = terrain.FlatGround(100.0, "t")
    csp2 = ConstraintCompiler(policy_from_raw(wgs84_doc(_raw()), ground=flat), ground=flat).compile_csp(m)
    assert "Stay between 50 m and 150 m altitude (heights above ground: declared " \
           "150-250 m MSL over ground at 100 m MSL)" in csp2.natural_language_prompt
    # An AGL policy's CSP is byte for byte what a compiler without a ground makes.
    agl = policy_from_raw(wgs84_doc(_raw(ref=None)))
    a1 = ConstraintCompiler(agl).compile_csp(m, issued_at="2026-11-17T00:00:00")
    a2 = ConstraintCompiler(agl, ground=g).compile_csp(m, issued_at="2026-11-17T00:00:00")
    assert a1.model_dump() == a2.model_dump()


def test_the_csp_compiles_a_keep_in_band_narrower_than_the_relief():
    """Review 2026-11-17: an MSL keep-in band narrower than the relief under
    the mission (150-180 m MSL over 100-150 m ground) has no AGL band over
    all of it; until then compile_csp raised CSPAltitudeConflict (and over
    the whole DEM even for 150-250). Now it states the band near the start,
    and says so; the Shield still converts at each point."""
    g = _slope_dem()
    m = Mission(task_text="fly east", start_x=-40.0, start_y=0.0, target_x=40.0,
                target_y=0.0, cruise_alt_m=60.0, speed_pref_mps=3.0)
    for kind in ("env", "corridor"):
        raw = _raw(env=(150.0, 180.0))
        if kind == "corridor":
            raw["constraints"][0] = {
                "id": "env", "type": "corridor", **H, "altitude_ref": "MSL",
                "centerline": [{"x": -45, "y": 0}, {"x": 45, "y": 0}],
                "width_m": 20.0, "altitude_floor_m": 150.0, "altitude_ceiling_m": 200.0}
        pol = policy_from_raw(wgs84_doc(raw), ground=g)
        csp = ConstraintCompiler(pol, ground=g).compile_csp(m, budget_tokens=512)
        lo, hi = csp.allowed_action_set.altitude_band_m_agl
        # Near the start (x -40 by 10 m: ground 100-110): floor 50, ceiling
        # 180-110 = 70 (envelope) or 200-110 = 90 (corridor).
        assert (lo, hi) == (50.0, 70.0 if kind == "env" else 90.0), (kind, lo, hi)
        assert "over ground at 100-110 m MSL near the mission start" in \
            csp.natural_language_prompt, csp.natural_language_prompt


def test_a_dem_needs_a_declared_terrain_height_reference():
    """Review 2026-11-17: a DEM converts correctly only when State.up is the
    height above the DEM surface; the rails' up is height above home. The
    Shield refuses a GridGround without up_ref='terrain', and
    terrain_relative_up converts a height above home."""
    g = _slope_dem()
    pol = policy_from_raw(wgs84_doc(_raw()), ground=g)
    for ref in (None, "home"):
        try:
            Shield(pol, ground=g, up_ref=ref, escalation=False)
        except ValueError as e:
            assert "up_ref" in str(e), e
        else:
            raise AssertionError(f"a DEM Shield built with up_ref={ref!r}")
    # A flat ground needs no declaration (every caller before 2026-11-17).
    flat = terrain.FlatGround(100.0, "t")
    Shield(policy_from_raw(wgs84_doc(_raw()), ground=flat), ground=flat, escalation=False)
    # Home at x = -40 (ground 105 m MSL), aircraft 30 m above home at x = 20
    # (ground 135): 105 + 30 = 135 m MSL, i.e. 0 m above the ground there.
    assert terrain.terrain_relative_up(30.0, 105.0, g, 20.0, 0.0) == 0.0
    assert terrain.terrain_relative_up(30.0, 105.0, g, -40.0, 0.0) == 30.0
    # The converted height and the Shield agree with the MSL truth: 240 m MSL
    # at x = -40 is legal (local ceiling 250 - 115 = 135 AGL; up = 135).
    sh = Shield(pol, ground=g, up_ref="terrain", escalation=False)
    up = terrain.terrain_relative_up(240.0 - 105.0, 105.0, g, -40.0, 0.0)
    assert up == 135.0 and not sh.state_is_unsafe(State(x=-40.0, y=0.0, up=up))


def test_a_dem_is_read_only_in_its_own_frame():
    """Review 2026-11-17: a DEM carries the WGS84 origin of its frame; a
    policy (or a loader) in another frame refuses it."""
    g0 = _slope_dem()
    g = terrain.GridGround(g0.x0, g0.y0, g0.cell_m, g0.z.tolist(), "t",
                           reach_m=10.0, origin=(25.0, 121.5))
    assert g.record()["origin"] == [25.0, 121.5]
    assert terrain.GridGround.from_dict(g.to_dict()).origin == (25.0, 121.5)
    raw = _raw()
    try:
        policy_from_raw(wgs84_doc(raw), ground=g)             # a policy in another frame
    except ValueError as e:
        assert "frame" in str(e), e
    else:
        raise AssertionError("a georeferenced DEM read in a policy of another frame")
    raw["origin"] = {"lat": 25.0, "lon": 121.5}
    pol = policy_from_raw(wgs84_doc(raw), ground=g)
    Shield(pol, ground=g, up_ref="terrain", escalation=False)
    raw["origin"] = {"lat": 25.001, "lon": 121.5}
    pol2 = policy_from_raw(wgs84_doc(raw), ground=terrain.FlatGround(100.0, "t"))
    for ground in (g, g0):
        try:
            Shield(pol2, ground=ground, up_ref="terrain", escalation=False)
        except ValueError as e:
            assert "frame" in str(e), e
        else:
            raise AssertionError(f"a DEM in another frame accepted: {ground!r}")


def test_an_msl_corridor_over_a_dem_reuses_its_tube():
    """Review 2026-11-17: each band makes new corridor objects; the tube is
    keyed on the rule's id and geometry, so it is buffered once, and the
    cache stays bounded on a long flight across many bands."""
    g = _slope_dem(reach=5.0)
    raw = {"policy_id": "msl-cor", "constraints": [{
        "id": "cor", "type": "corridor", **H, "altitude_ref": "MSL",
        "centerline": [{"x": -45, "y": 0}, {"x": 45, "y": 0}],
        "width_m": 20.0, "altitude_floor_m": 130.0, "altitude_ceiling_m": 260.0}]}
    sh = Shield(policy_from_raw(wgs84_doc(raw), ground=g), ground=g, up_ref="terrain",
                escalation=False)
    bands = set()
    for k in range(400):
        x = -44.0 + (k % 89)
        bands.add(g.band_at(x, 0.0))
        sh.filter(State(x=x, y=0.0, up=200.0 - g.elevation_at(x, 0.0)), Action4D(vx=1.0))
        rt = sh._local_runtime(sh._rt, x, 0.0)
        assert rt.corridors[0].altitude_floor_m == 130.0 - g.band_at(x, 0.0)[0]
        sh._corridor_tube(rt.corridors[0])       # what the segment check reads
    tubes = {id(v) for k, v in sh._tubes.items() if isinstance(k, tuple)}
    assert len(bands) > 5 and len(tubes) == 1, (len(bands), len(tubes))
    assert len(sh._tubes) <= 512


def test_the_family_tool_meets_the_kpis_on_its_scenarios():
    """tools/msl_dem_kpi.py's family (no timing here): P0 escapes 0 against
    an oracle of the declared MSL rules, every fail-safe label right, the
    untouched flights unhindered, and both nulls see violations."""
    import msl_dem_kpi as T
    fam = T.family(T.load_dem())
    s = fam["summary"]
    assert s["episodes"] == 35 and s["p0_requested_ticks"] > 0, s
    assert s["p0_true_escape_ticks"] == 0 and s["shield_emitted_p0_ticks"] == 0, s
    assert s["failsafe_trigger_correctness"] == 1.0, s
    assert 0 < s["failsafe_expected"] < s["episodes"], s
    assert s["untouched_flights_unhindered"] and s["untouched_flights"] == 10, s
    assert s["null_shield_off_true_p0_ticks"] > 0 and s["null_msl_as_agl_true_p0_ticks"] > 0, s
    assert T.load_dem().record()["sha256"] == T.ridge_dem().record()["sha256"], \
        "experiments/worlds/dem_ridge.json is the tool's ridge"


def test_the_kpi_tool_runs_its_dsl_kpis_in_a_stated_frame():
    """tools/msl_dem_kpi.py dsl_kpis, end to end with tiny n (no timing
    asserted beyond the budgets' shape). Every policy it builds states a
    WGS84 origin (FRAME) and stores its points in lat/lon, and the ridge
    (a metre grid with no origin on disk) is read in that frame (WP1-14,
    review 2026-11-18): with no origin, the hot-apply phase - an MSL
    dynamic_nfz written into the 50-rule MSL policy - was refused
    (models.NoFrameOrigin) and the tool crashed."""
    import msl_dem_kpi as T
    from guardrail.models import policy_from_raw
    dem = T.load_dem()
    assert dem.origin == T.FRAME and terrain.GridGround.load(T.DEM_PATH).origin is None
    for raw in (T.fifty_rule_msl_raw(), T.family_policy()):
        pol = policy_from_raw(wgs84_doc(raw), ground=dem)
        assert pol.point_form == "wgs84" and pol.frame_origin == T.FRAME, raw["policy_id"]
    with tempfile.TemporaryDirectory() as td:
        kp = T.dsl_kpis(dem, 0, 1, 4, 2, Path(td))
    assert kp["frame_origin"] == list(T.FRAME)
    assert kp["hot_apply_msl_dynamic_nfz"]["n"] == 2, kp["hot_apply_msl_dynamic_nfz"]
    assert kp["query_check_msl_corridor_local_runtime_cold"]["n"] == 4
    assert kp["msl_corridor_tubes_buffered"] == 1, kp


if __name__ == "__main__":
    import traceback
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    ok = 0
    for name, fn in fns:
        try:
            fn()
            print(f"PASS  {name}")
            ok += 1
        except Exception as e:                                # noqa: BLE001
            print(f"FAIL  {name}\n      {type(e).__name__}: {e}")
            traceback.print_exc()
    print(f"\n{ok}/{len(fns)} passed")
    sys.exit(0 if ok == len(fns) else 1)
