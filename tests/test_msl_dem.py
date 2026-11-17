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

H = {"scope": "mission", "constraint_type": "hard", "priority": "P0",
     "violation_action": "project_fix"}


def _slope_dem(reach=10.0):
    """Ground rising 0.5 m per metre east from 100 m MSL at x = 0."""
    xs = [-50.0 + 10.0 * i for i in range(21)]
    elev = [[100.0 + 0.5 * (x + 50.0) for x in xs] for _ in range(11)]
    return terrain.GridGround(-50.0, -50.0, 10.0, elev, "test slope", reach_m=reach)


def _raw(ref="MSL", env=(150.0, 250.0), zone=(0.0, 180.0)):
    env_r = {"id": "env", "type": "altitude_envelope", **H,
             "alt_min_m": env[0], "alt_max_m": env[1]}
    zone_r = {"id": "zone", "type": "polygon_fence", **H, "margin_m": 1.0,
              "vertices": [{"x": 60, "y": -10}, {"x": 80, "y": -10},
                           {"x": 80, "y": 10}, {"x": 60, "y": 10}],
              "altitude_floor_m": zone[0], "altitude_ceiling_m": zone[1]}
    if ref:
        env_r["altitude_ref"] = zone_r["altitude_ref"] = ref
    return {"policy_id": "msl-dem-test", "constraints": [env_r, zone_r]}


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
    pol = policy_from_raw(_raw(), ground=_slope_dem())
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
    sh = Shield(policy_from_raw(_raw(), ground=g), ground=g, escalation=False,
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
    sh = Shield(policy_from_raw(_raw(), ground=g), ground=g, escalation=False)
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
    g = _slope_dem(reach=5.0)
    sh = Shield(policy_from_raw(_raw(), ground=g), ground=g, escalation=False)
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
    sh.hot_apply(zone)
    # Ground ~110 there: 140 MSL is inside the zone's band, 160 above it.
    inside = {r["id"]: r for r in sh.rule_status(State(x=-25.0, y=0.0, up=30.0))}
    above = {r["id"]: r for r in sh.rule_status(State(x=-25.0, y=0.0, up=50.0))}
    assert inside["dz"]["breach"] and not above["dz"]["breach"], (inside["dz"], above["dz"])


def test_agl_and_flat_msl_policies_decide_as_before():
    """Opt-in: no DEM, no local runtime - the runtime has no MSL entries and
    every decision is the plain one (flat MSL = the AGL policy shifted)."""
    flat = terrain.FlatGround(100.0, "t")
    s_flat = Shield(policy_from_raw(_raw(), ground=flat), ground=flat, escalation=False)
    s_agl = Shield(policy_from_raw(_raw(ref=None, env=(50.0, 150.0), zone=(-100.0, 80.0))),
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
    pol = policy_from_raw(_raw(), ground=g)
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
    # Whole DEM 100-200 m: keep-in 150-250 MSL -> 50-50 AGL; stated with the ground.
    assert "Stay between 50 m and 50 m altitude (heights above ground: declared " \
           "150-250 m MSL over ground at 100-200 m MSL)" in text, text
    assert "declared 0-180 m MSL over ground at 100-200 m MSL" in text, text
    # A flat ground states its one elevation.
    flat = terrain.FlatGround(100.0, "t")
    csp2 = ConstraintCompiler(policy_from_raw(_raw(), ground=flat), ground=flat).compile_csp(m)
    assert "Stay between 50 m and 150 m altitude (heights above ground: declared " \
           "150-250 m MSL over ground at 100 m MSL)" in csp2.natural_language_prompt
    # An AGL policy's CSP is byte for byte what a compiler without a ground makes.
    agl = policy_from_raw(_raw(ref=None))
    a1 = ConstraintCompiler(agl).compile_csp(m, issued_at="2026-11-17T00:00:00")
    a2 = ConstraintCompiler(agl, ground=g).compile_csp(m, issued_at="2026-11-17T00:00:00")
    assert a1.model_dump() == a2.model_dump()


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
