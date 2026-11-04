"""distance_envelope enforced (WP1-07) and MSL as an opt-in altitude
reference with a ground-elevation source (X-11), both since 2026-11-03.

Run either way:
    pytest tests/test_distance_envelope_msl.py -v
    python tests/test_distance_envelope_msl.py

WHAT IS PINNED

The grant's distance_envelope ("min distance to typed object class: people,
buildings, roads") was declarable and refused by every flight loader. It is
now enforced through the runtime the Shield already had for each class
(models.DistanceEnvelope.runtime_rule), so the tests below do not re-test the
clearance and stand-off machinery; they show that a distance_envelope IS that
machinery, decision for decision, on the map or subject of its class:

  * buildings == an obstacle_clearance on the same obstacle map;
  * roads     == an obstacle_clearance on a map that is the road mask (the
                 road field is read exactly as the obstacle field is);
  * people    == a subject_standoff binding the tracked person, and not a
                 car;
  * a buildings or roads envelope without its map is REFUSED at
    construction - never inert;
  * the compiler renders it, and a policy file carrying it loads for flight.

MSL: a rule with altitude_ref MSL is refused without a ground (loaders, the
Shield, hot-apply), and with a flat ground at elevation E it is enforced
exactly as the AGL policy whose bands are shifted by -E (756 decisions, no
difference). A non-flat ground is refused, not approximated.

Each assertion names what the pre-change code did ("before:").
"""
import itertools
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np                                                      # noqa: E402

from guardrail import terrain                                           # noqa: E402
from guardrail.compiler import ConstraintCompiler, Mission              # noqa: E402
from guardrail.models import (RUNTIME_TYPES, Action4D, DynamicNFZ,      # noqa: E402
                              State, policy_from_raw)
from guardrail.shield import ENFORCED_TYPES, HotApplyRefused, Shield    # noqa: E402

H = {"constraint_type": "hard", "priority": "P0", "violation_action": "project_fix"}


def _refused(fn, *needles):
    try:
        fn()
    except ValueError as e:
        msg = str(e)
        for n in needles:
            assert n in msg, (n, msg)
        return msg
    raise AssertionError(f"accepted; expected a refusal mentioning {needles}")


def _grid_map(cells, n=40, res=2.0, origin=-20.0):
    occ = np.zeros((n, n), dtype=np.uint8)
    for sl in cells:
        occ[sl] = 1
    return {"occ": occ, "res": res, "ox": origin, "oy": origin}


ROADS = _grid_map([(slice(None), 10)])                     # a road along y ~ 0
BUILDINGS = _grid_map([(slice(28, 33), slice(18, 23))])    # a block at x 36..44, y 16..24


def _policy(*rules, pid="p"):
    return policy_from_raw({"policy_id": pid, "constraints": list(rules)})


def _de(cls, d, rid=None):
    return {"id": rid or f"de-{cls}", "type": "distance_envelope", **H,
            "object_class": cls, "min_distance_m": d}


def _same_decisions(s1, s2, states_actions) -> tuple[int, int]:
    n = diff = 0
    for st, a in states_actions:
        r1, r2 = s1.filter(st, a), s2.filter(st, a)
        n += 1
        if r1.emitted != r2.emitted or \
                [(v.rule_id, v.category) for v in r1.violations] != \
                [(v.rule_id, v.category) for v in r2.violations]:
            diff += 1
    return n, diff


def _sweep(xs, ys):
    for x, y, vx, vy in itertools.product(xs, ys, (-4.0, 0.0, 4.0), (-4.0, 0.0, 4.0)):
        yield State(x=float(x), y=float(y), up=10.0), Action4D(vx=vx, vy=vy)


# --------------------------------------------------------------------------- #
# distance_envelope (WP1-07)
# --------------------------------------------------------------------------- #

def test_distance_envelope_is_a_runtime_type_and_loads_for_flight():
    """Before: declarable only - RUNTIME_TYPES and the Shield's ENFORCED_TYPES
    left it out and load_policy refused it ("the Safety Shield does not
    enforce it")."""
    assert "distance_envelope" in RUNTIME_TYPES and "distance_envelope" in ENFORCED_TYPES
    pol = _policy(_de("people", 10), _de("buildings", 4), _de("roads", 3))
    assert pol.flight_problems() == [], pol.flight_problems()
    assert Shield.unenforceable(pol) == []


def test_a_buildings_or_roads_envelope_without_its_map_is_refused_never_inert():
    pol = _policy(_de("buildings", 4), _de("roads", 3))
    _refused(lambda: Shield(pol), "de-buildings", "obstacle_map", "de-roads", "road_map")
    _refused(lambda: Shield(pol, obstacle_map=BUILDINGS), "de-roads", "road_map")
    _refused(lambda: Shield(pol, road_map=ROADS), "de-buildings", "obstacle_map")
    Shield(pol, obstacle_map=BUILDINGS, road_map=ROADS)
    # people needs no map: it binds to whatever person is tracked.
    Shield(_policy(_de("people", 10)))


def test_buildings_is_obstacle_clearance_decision_for_decision():
    de = Shield(_policy(_de("buildings", 4, "rule")), obstacle_map=BUILDINGS)
    oc = Shield(_policy({"id": "rule", "type": "obstacle_clearance", **H,
                         "min_clearance_m": 4.0}), obstacle_map=BUILDINGS)
    n, diff = _same_decisions(de, oc, _sweep(range(26, 50, 2), range(8, 32, 2)))
    assert n == 1296 and diff == 0, (n, diff)
    # ... and it acts there (the sweep is not all clear).
    r = de.filter(State(x=33.0, y=20.0, up=10.0), Action4D(vx=4.0, vy=0.0))
    assert [v.rule_id for v in r.violations] == ["rule"] and r.emitted.vx < 4.0, r


def test_roads_reads_the_road_map_as_clearance_reads_buildings():
    """The road field is built and read exactly as the obstacle field is:
    the same rule on the same grid, given as the road map or as the
    obstacle map, decides the same."""
    de = Shield(_policy(_de("roads", 3, "rule")), road_map=ROADS)
    oc = Shield(_policy({"id": "rule", "type": "obstacle_clearance", **H,
                         "min_clearance_m": 3.0}), obstacle_map=ROADS)
    n, diff = _same_decisions(de, oc, _sweep(range(-6, 7, 3), [y * 0.5 for y in range(-20, 21)]))
    assert n == 1845 and diff == 0, (n, diff)
    r = de.filter(State(x=0.0, y=4.0, up=10.0), Action4D(vx=0.0, vy=-4.0))
    assert [v.rule_id for v in r.violations] == ["rule"], r
    assert r.emitted.vy > -4.0 and any(x.operator == "ClearanceFix" for x in r.repairs), r
    # The obstacle map does not leak into the road rule, nor the road map into
    # a building rule: two rules, two maps.
    both = Shield(_policy(_de("roads", 3), _de("buildings", 4)),
                  obstacle_map=BUILDINGS, road_map=ROADS)
    st = {r["id"]: r for r in both.rule_status(State(x=0.0, y=1.0, up=10.0))}
    assert st["de-roads"]["breach"] and not st["de-buildings"]["breach"], st
    st = {r["id"]: r for r in both.rule_status(State(x=40.0, y=20.0, up=10.0))}
    assert st["de-buildings"]["breach"] and not st["de-roads"]["breach"], st


def test_people_is_a_standoff_from_the_tracked_person_and_not_a_car():
    de = Shield(_policy(_de("people", 10, "rule")))
    so = Shield(_policy({"id": "rule", "type": "subject_standoff", **H,
                         "subject_class": "pedestrian", "min_range_m": 10.0}))
    for sh in (de, so):
        sh.set_subject(0.0, 0.0, "pedestrian")
    n, diff = _same_decisions(de, so, _sweep(range(-12, 13, 4), range(-12, 13, 4)))
    assert n == 441 and diff == 0, (n, diff)
    for cls, binds in (("pedestrian", True), ("person", True), ("car", False), (None, False)):
        de.set_subject(0.0, 0.0, cls)
        r = de.filter(State(x=0.0, y=12.0, up=10.0), Action4D(vx=0.0, vy=-4.0))
        assert bool(r.violations) == binds, (cls, r.violations)


def test_the_compiler_renders_every_class_and_the_cost_map_says_what_it_cannot_draw():
    """Before: compile_csp refused the policy (CSPUnenforcedRules)."""
    pol = _policy(_de("people", 10), _de("buildings", 4), _de("roads", 3))
    m = Mission(task_text="follow the person", start_x=0, start_y=20, target_x=50,
                target_y=20, cruise_alt_m=10, speed_pref_mps=3)
    text = ConstraintCompiler(pol).compile_csp(m).natural_language_prompt
    for want in ("Keep at least 10 m from the person you are following.",
                 "Keep at least 4 m from any building.",
                 "Keep at least 3 m from any road."):
        assert want in text, text
    from guardrail.compiler import rasterise_cost_map
    cm = rasterise_cost_map(pol, obstacle_map=BUILDINGS, subject=(0.0, 0.0, "pedestrian"))
    whys = {d["id"]: d["why"] for d in cm.header["not_drawn"]}
    assert "de-roads" in whys and "road map" in whys["de-roads"], whys
    assert "de-buildings" not in whys and "de-people" not in whys, whys


def test_a_layer_may_tighten_a_distance_envelope_but_not_relax_it():
    from guardrail.models import merge_layers
    reg = _policy({**_de("people", 10, "d"), "layer": "regulation"}, pid="reg")
    tighter = _policy({**_de("people", 15, "d"), "layer": "mission"}, pid="m")
    looser = _policy({**_de("people", 5, "d"), "layer": "mission"}, pid="m")
    assert merge_layers([reg, tighter]).constraints[0].min_distance_m == 15
    _refused(lambda: merge_layers([reg, looser]), "reduces min_distance_m")


# --------------------------------------------------------------------------- #
# MSL (X-11)
# --------------------------------------------------------------------------- #

def _msl_raw(ref="MSL", lo=600.0, hi=640.0, floor=0.0, ceil=700.0):
    rr = {"altitude_ref": ref} if ref else {}
    return {"policy_id": "msl", "constraints": [
        {"id": "alt", "type": "altitude_envelope", **H, "alt_min_m": lo, "alt_max_m": hi, **rr},
        {"id": "nfz", "type": "polygon_fence", **H, **rr,
         "vertices": [{"x": 20, "y": -5}, {"x": 30, "y": -5}, {"x": 30, "y": 5},
                      {"x": 20, "y": 5}],
         "altitude_floor_m": floor, "altitude_ceiling_m": ceil}]}


def test_msl_is_refused_without_a_ground_everywhere():
    """As before 2026-11-03: no ground, no MSL - at the loader, the Shield."""
    _refused(lambda: policy_from_raw(_msl_raw()), "altitude_ref MSL", "ground")
    pol = policy_from_raw(_msl_raw(), runtime=False)
    _refused(lambda: Shield(pol), "altitude_ref MSL", "ground")


def test_msl_over_a_flat_ground_is_the_agl_policy_shifted_by_its_elevation():
    g = terrain.SITL_CMAC
    assert g.elevation_msl_m == 584.0
    msl = policy_from_raw(_msl_raw(), ground=g)
    agl = policy_from_raw(_msl_raw(ref=None, lo=16.0, hi=56.0, floor=-584.0, ceil=116.0))
    s1, s2 = Shield(msl, ground=g), Shield(agl)
    assert s1.ground is g and s1.policy.policy_hash == msl.policy_hash, \
        "the declared policy (and its hash) is what the Shield holds"
    cases = [(State(x=float(x), y=0.0, up=float(up)),
              Action4D(vx=float(vx), vy=0.0, vz_up=float(vz)))
             for x, up, vx, vz in itertools.product(range(0, 40, 3), (5, 15, 30, 55, 60, 120),
                                                    (-3, 0, 3), (-2, 0, 2))]
    n, diff = _same_decisions(s1, s2, cases)
    assert n == 756 and diff == 0, (n, diff)
    # The same heights judged as AGL (no conversion) would be another policy:
    # 600 m above GROUND forbids every case here.
    wrong = Shield(policy_from_raw(_msl_raw(ref=None)))
    assert _same_decisions(wrong, s2, cases)[1] > 100


def test_a_ground_that_is_not_flat_is_refused_not_approximated():
    class Hilly:
        flat = False

        def elevation_at(self, x=None, y=None):
            return 100.0 + 0.1 * (x or 0.0)
    pol = policy_from_raw(_msl_raw(), runtime=False)
    _refused(lambda: Shield(pol, ground=Hilly()), "not a flat ground")
    _refused(lambda: terrain.FlatGround(float("nan"), "x"), "not finite")
    _refused(lambda: terrain.FlatGround(10.0, ""), "say where")


def test_an_msl_hot_apply_follows_the_shields_ground():
    """Before (2026-10-19): an MSL hot-apply was refused outright. Now it is
    refused by a Shield with no ground and accepted by one with a ground."""
    base = policy_from_raw(_msl_raw(ref=None, lo=5.0, hi=60.0))
    zone = DynamicNFZ(id="dz", type="dynamic_nfz", **H, altitude_ref="MSL",
                      vertices=[{"x": 50, "y": -5}, {"x": 60, "y": -5}, {"x": 60, "y": 5},
                                {"x": 50, "y": 5}],
                      altitude_floor_m=0.0, altitude_ceiling_m=650.0)
    try:
        Shield(base).hot_apply(zone)
    except HotApplyRefused as e:
        assert "MSL" in str(e), e
    else:
        raise AssertionError("an MSL zone was hot-applied to a Shield with no ground")
    sh = Shield(base, ground=terrain.FlatGround(600.0, "test"))
    sh.hot_apply(zone)
    # 650 m MSL over 600 m ground is a 50 m AGL ceiling: 40 m is inside, 55 m above.
    st = {r["id"]: r for r in sh.rule_status(State(x=55.0, y=0.0, up=40.0))}
    assert st["dz"]["breach"], st
    st = {r["id"]: r for r in sh.rule_status(State(x=55.0, y=0.0, up=55.0))}
    assert not st["dz"]["breach"], st


def test_the_rails_state_their_ground_and_a_flight_records_it():
    from guardrail.bundle import load_for_flight
    assert terrain.RAIL_GROUND["ardupilot-sitl"].elevation_msl_m == 584.0
    scene = ROOT / "demo" / "pas_ardupilot" / "scene_citylife_ardupilot.jsonc"
    assert terrain.scene_ground(scene).elevation_msl_m == 40.0
    assert terrain.PAS_SCENE.elevation_msl_m == 40.0
    import json
    import tempfile
    d = Path(tempfile.mkdtemp())
    f = d / "msl.yaml"
    f.write_text(json.dumps(_msl_raw()), encoding="utf-8")
    _refused(lambda: load_for_flight(yaml_path=f), "altitude_ref MSL")
    pol, rec = load_for_flight(yaml_path=f, ground=terrain.SITL_CMAC)
    assert rec["ground"] == {"kind": "flat", "elevation_msl_m": 584.0,
                             "source": terrain.SITL_CMAC.source}, rec


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
