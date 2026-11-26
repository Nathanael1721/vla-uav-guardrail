"""X-11: MSL rules over a non-flat ground (a gridded DEM), measured.

    python tools/msl_dem_kpi.py                       # writes docs/data/msl_dem_kpi.json
    python tools/msl_dem_kpi.py --out x.json --fresh-runs 5
    python tools/msl_dem_kpi.py --write-dem           # (re)writes experiments/worlds/dem_ridge.json

WHAT THE GRANT ASKS FOR

Policy DSL page: every geometry primitive carries `altitude_ref` (AGL
default, MSL opt-in). Until 2026-11-17 an MSL rule was enforced over a FLAT
ground only (guardrail/terrain.py FlatGround) and the CSP refused it. Since
then the Shield converts an MSL band at the query point over a gridded DEM
(terrain.GridGround, Shield._local_runtime) and the CSP compiles it over a
ground it is given (ConstraintCompiler(policy, ground=...)).

WHAT THIS MEASURES

1. A headless scenario family over a non-flat DEM (a ridge 40 -> 100 m MSL
   with a cross slope; experiments/worlds/dem_ridge.json): an MSL altitude
   envelope (keep-in, 110-200 m MSL) and an MSL no-fly band over the crest
   (keep-out, 0-160 m MSL), flown by dumb pilots from five starts each:
   climb into the ceiling, descend into the floor, cross the ridge level,
   fly into the zone below its ceiling, overfly the zone above it, and two
   variants whose envelope's violation_action is RTL (the rubric's E1, a
   fail-safe expected). The vehicle holds its MSL height (an ideal
   integrator of the commanded velocity); the Shield is told its height
   above the ground under it, as a rail's rangefinder / terrain database
   would. P0 escapes are judged by an ORACLE that does not use the Shield:
   the true MSL height against the DECLARED MSL rules, every tick
   (`true_p0_ticks`), beside the Shield's own re-check of what it emitted.
   Fail-safe labels: the FSM entering RTL or Land (docs/RUBRIC-failsafe.md
   section 1), against the label each scenario carries (E5, E1 or N1).
   CAVEAT: the labels were set after a first run that used other labels,
   matched to how the library labels its AGL twin altitude-ceiling-hold
   (E5). The fail-safe figure is therefore not pre-registered and is NOT an
   independent fail-safe KPI result; the P0 figure (oracle) is.
   Two nulls fly the same episodes: the Shield OFF (the pilot flown raw:
   the oracle must see the violations) and MSL READ AS AGL (the same
   numbers with altitude_ref dropped, what the DSL did before an MSL height
   was refused: the oracle must see escapes).
2. The locked DSL KPIs on an MSL policy over the DEM (Policy DSL page,
   acceptance KPIs): cold start <= 1 s (fresh processes, imports included:
   read the DEM and the policy, load, build the Shield, first tick),
   hot-apply <= 50 ms (an MSL dynamic_nfz into the 50-rule MSL Shield), and
   query <= 5 ms (Shield._check at 50 rules at the query point, the local
   runtime built from cold for every sample - the worst case - and warm).
   The 50 rules are experiments/bench_shield_50rules.py's, every banded rule
   made MSL (heights + 40 m, the ridge's foot). That set has no corridor, so
   the MSL corridor path (its tube reused across bands, review 2026-11-17)
   is timed on its own: an MSL corridor + envelope over the ridge, _check
   at points across many bands, the local runtime built from cold, on a
   Shield with segment_check=True (the only path that reads the tube).

FRAME (WP1-14, 2026-11-18): every policy here states a WGS84 origin
(FRAME, the bench's BENCH_ORIGIN) and stores its points in lat/lon, since
the grant's DSL "never carries projected coordinates", and the DEM is read
in that frame (load_dem / in_frame place the metre grid of
experiments/worlds/dem_ridge.json, which states no origin, at FRAME). The
geometry is laid out in metres on the micrometre grid, so the Shield's view
of every point is the metre value it was laid out at. Until 2026-11-18 the
policies were metre-only with no origin; the hot-apply phase then stopped
working, because a policy with no frame origin refuses an event with points.

SCOPE (what this does NOT show): no rail passes a ground yet; the rails'
State.up is height above home, which a DEM Shield refuses (up_ref); the DEM
is synthetic; no SITL, hil or PAS flight has carried an MSL rule.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import statistics
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

from bench_shield_50rules import BENCH_ORIGIN                        # noqa: E402
from guardrail import terrain                                        # noqa: E402
from guardrail.models import Action4D, State, policy_from_raw         # noqa: E402
from guardrail.projection import raw_points_in_wgs84                 # noqa: E402
from guardrail.shield import Shield                                  # noqa: E402

DEM_PATH = ROOT / "experiments" / "worlds" / "dem_ridge.json"
OUT = ROOT / "docs" / "data" / "msl_dem_kpi.json"
DT = 0.1
# The WGS84 frame every policy here states and the DEM is read in (module
# docstring, FRAME): the bench's, since the 50-rule MSL set is its grid.
FRAME = BENCH_ORIGIN
H = {"scope": "mission", "constraint_type": "hard", "priority": "P0",
     "violation_action": "project_fix"}

# The family's rules, declared in MSL.
ENV_LO, ENV_HI = 110.0, 200.0                   # keep-in envelope, m MSL
ZONE = [(70.0, -50.0), (130.0, -50.0), (130.0, 50.0), (70.0, 50.0)]
ZONE_LO, ZONE_HI, ZONE_MARGIN = 0.0, 160.0, 2.0  # keep-out band over the crest


def ridge_elevation(x: float, y: float) -> float:
    """40 m MSL plain, a ridge along y at x = 100 rising 60 m (Gaussian,
    120 m wide: steepest slope 0.43 m/m), tilted 0.02 m/m in y."""
    return 40.0 + 60.0 * math.exp(-((x - 100.0) / 120.0) ** 2) + 0.02 * (y + 300.0) - 6.0


def ridge_dem(x0=-300.0, y0=-300.0, nx=81, ny=101, cell=10.0, reach_m=20.0):
    xs = [x0 + i * cell for i in range(nx)]
    ys = [y0 + j * cell for j in range(ny)]
    elev = [[round(ridge_elevation(x, y), 3) for x in xs] for y in ys]
    return terrain.GridGround(x0, y0, cell, elev,
                              "synthetic ridge (tools/msl_dem_kpi.py ridge_elevation)",
                              reach_m=reach_m)


def in_frame(dem: terrain.GridGround) -> terrain.GridGround:
    """`dem` placed in FRAME: a metre grid that states no origin (the ridge
    file) gets FRAME as its origin, the nodes unchanged; a grid already in
    FRAME comes back as it is; one in another frame is refused."""
    if dem.origin is None:
        d = dem.to_dict()
        d["origin"] = list(FRAME)
        return terrain.GridGround.from_dict(d)
    why = terrain.origin_problem(dem, FRAME)
    if why:
        raise ValueError(why)
    return dem


def load_dem() -> terrain.GridGround:
    """The ridge (experiments/worlds/dem_ridge.json, else ridge_dem()) read in
    FRAME. The file itself stays a metre grid with no origin."""
    return in_frame(terrain.GridGround.load(DEM_PATH) if DEM_PATH.exists() else ridge_dem())


def framed(raw: dict) -> dict:
    """A raw policy laid out in metres, stored as the DSL stores it: the
    origin stated (FRAME) and every point in WGS84 about it."""
    return {**raw, "origin": {"lat": FRAME[0], "lon": FRAME[1]},
            "constraints": raw_points_in_wgs84(raw["constraints"], FRAME)}


def family_policy(env_action: str = "project_fix", ref: str | None = "MSL") -> dict:
    env = {"id": "env-msl", "type": "altitude_envelope", **H,
           "violation_action": env_action, "alt_min_m": ENV_LO, "alt_max_m": ENV_HI}
    zone = {"id": "crest-msl", "type": "polygon_fence", **H,
            "vertices": [{"x": x, "y": y} for x, y in ZONE],
            "altitude_floor_m": ZONE_LO, "altitude_ceiling_m": ZONE_HI,
            "margin_m": ZONE_MARGIN}
    if ref:
        env["altitude_ref"] = zone["altitude_ref"] = ref
    kin = {"id": "kin", "type": "kinematic_envelope", **H, "speed_max_mps": 3.0,
           "climb_rate_max_mps": 2.5, "yaw_rate_max_dps": 60.0}
    return framed({"policy_id": f"msl-ridge-{env_action}", "version": "1.0.0",
                   "constraints": [env, zone, kin]})


# name -> (pilot (vx, vz), start MSL height, start y, start xs, envelope
# violation_action, expected fail-safe, rubric clause, unhindered?).
# Labels follow docs/RUBRIC-failsafe.md section 2, as the library labels its
# AGL twins: a pilot that pushes through the ceiling (floor) for the whole
# 40 s flight keeps requesting a repair over theta past the operator's Loiter
# timeout - E5, a fail-safe expected (experiments/templates/
# altitude_envelope.yaml altitude-ceiling-hold); a rule whose action is RTL,
# violated by the request - E1; a pilot the Shield can stop short of a zone
# and hold there without a timeout (Brake has none) - N1; a flight the rules
# never touch - N1, and it must go unhindered (`unhindered`: the full 120 m).
# The vertical cells start at y = 120, north of the crest zone, over the
# ridge's foot, flank and crest (x -100 ... 160).
STARTS = (-100.0, 0.0, 60.0, 100.0, 160.0)
WEST = (-100.0, -50.0, 0.0, 30.0, 50.0)
SCENARIOS = {
    "msl-ceiling-climb": ((0.0, 2.0), 185.0, 120.0, STARTS, "project_fix", True, "E5", False),
    "msl-floor-descend": ((0.0, -2.0), 125.0, 120.0, STARTS, "project_fix", True, "E5", False),
    "msl-level-cross-ridge": ((3.0, 0.0), 150.0, 120.0, WEST, "project_fix", False, "N1", True),
    "msl-zone-below-ceiling": ((3.0, 0.0), 140.0, 0.0, WEST, "project_fix", False, "N1", False),
    "msl-zone-overfly": ((3.0, 0.0), 185.0, 0.0, WEST, "project_fix", False, "N1", True),
    "msl-ceiling-climb-rtl": ((0.0, 2.0), 185.0, 120.0, STARTS, "RTL", True, "E1", False),
    "msl-floor-descend-rtl": ((0.0, -2.0), 125.0, 120.0, STARTS, "RTL", True, "E1", False),
}


def truth_p0(x: float, y: float, z: float) -> list[str]:
    """The DECLARED MSL rules, judged at the true MSL height (no Shield)."""
    out = []
    if not (ENV_LO - 1e-9 <= z <= ENV_HI + 1e-9):
        out.append("env-msl")
    if ZONE[0][0] <= x <= ZONE[1][0] and ZONE[0][1] <= y <= ZONE[2][1] \
            and ZONE_LO <= z <= ZONE_HI:
        out.append("crest-msl")
    return out


def run_episode(dem, name: str, x0: float, arm: str = "on", seconds: float = 40.0) -> dict:
    (vx, vz), z0, y0, _, action, expected, clause, free = SCENARIOS[name]
    dem = in_frame(dem)
    if arm == "msl_as_agl":
        pol = policy_from_raw(family_policy(action, ref=None))
        sh = Shield(pol)
    else:
        pol = policy_from_raw(family_policy(action), ground=dem)
        sh = Shield(pol, ground=dem, up_ref="terrain")
    x, y, z = x0, y0, z0
    if truth_p0(x, y, z):
        raise ValueError(f"{name} x0={x0}: starts in violation of a declared rule")
    true_ticks = shield_escapes = requested = 0
    triggered, states = False, set()
    progress = x
    for k in range(int(round(seconds / DT))):
        up = z - dem.elevation_at(x, y)
        st = State(x=x, y=y, up=up)
        raw = Action4D(vx=vx, vy=0.0, vz_up=vz)
        if arm == "off":
            fly = raw
        else:
            d = sh.filter(st, raw, t=k * DT)
            states.add(str(d.fsm_state_after))
            if any(v.rule_id in ("env-msl", "crest-msl") for v in d.violations):
                requested += 1
            if (d.command is not None and d.command == d.emitted
                    and any(v.rule_id in ("env-msl", "crest-msl")
                            for v in d.emitted_violations)):
                shield_escapes += 1
            if str(d.fsm_state_after).lower() in ("rtl", "land"):
                triggered = True
                break
            fly = d.command if d.command is not None else Action4D()
        x, y, z = x + fly.vx * DT, y + fly.vy * DT, z + fly.vz_up * DT
        progress = max(progress, x)
        if truth_p0(x, y, z):
            true_ticks += 1
    return {"scenario": name, "x0": x0, "y0": y0, "arm": arm,
            "expected_failsafe": expected, "failsafe_clause": clause,
            "failsafe_triggered": triggered,
            "unhindered": abs(x - (x0 + vx * seconds)) < 0.5 and abs(z - z0) < 0.5,
            "must_be_unhindered": free,
            "label_correct": triggered == expected,
            "true_p0_ticks": true_ticks, "shield_emitted_p0_ticks": shield_escapes,
            "requested_p0_ticks": requested, "end_x": round(x, 3),
            "end_msl_m": round(z, 3), "fsm_states": sorted(states)}


def family(dem) -> dict:
    dem = in_frame(dem)
    eps = {"on": [], "off": [], "msl_as_agl": []}
    for name, spec in SCENARIOS.items():
        for x0 in spec[3]:
            for arm in eps:
                eps[arm].append(run_episode(dem, name, x0, arm))
    on = eps["on"]
    req = sum(e["requested_p0_ticks"] for e in on)
    esc = sum(e["true_p0_ticks"] for e in on)
    summary = {
        "episodes": len(on),
        "p0_requested_ticks": req,
        "p0_true_escape_ticks": esc,
        "p0_escape_rate": (esc / req) if req else None,
        "shield_emitted_p0_ticks": sum(e["shield_emitted_p0_ticks"] for e in on),
        "failsafe_trigger_correctness": sum(e["label_correct"] for e in on) / len(on),
        "failsafe_expected": sum(e["expected_failsafe"] for e in on),
        "failsafe_triggered": sum(e["failsafe_triggered"] for e in on),
        "untouched_flights_unhindered": all(e["unhindered"] for e in on
                                            if e["must_be_unhindered"]),
        "untouched_flights": sum(1 for e in on if e["must_be_unhindered"]),
        "null_shield_off_true_p0_ticks": sum(e["true_p0_ticks"] for e in eps["off"]),
        "null_shield_off_episodes_violating": sum(1 for e in eps["off"] if e["true_p0_ticks"]),
        "null_msl_as_agl_true_p0_ticks": sum(e["true_p0_ticks"] for e in eps["msl_as_agl"]),
        "null_msl_as_agl_episodes_violating": sum(
            1 for e in eps["msl_as_agl"] if e["true_p0_ticks"]),
    }
    return {"summary": summary, "episodes": eps}


# ---------------------------------------------------------------- DSL KPIs

def fifty_rule_msl_raw() -> dict:
    """bench_shield_50rules.fifty_rule_policy() with every banded rule MSL
    (heights + 40 m, the ridge's foot), in its stored form: origin FRAME,
    points in lat/lon only."""
    import bench_shield_50rules as bench
    pol = bench.fifty_rule_policy()
    assert pol.frame_origin == FRAME, (pol.frame_origin, FRAME)
    data = pol.model_dump(mode="json", exclude_none=True, context={"points": "stored"})
    for c in data["constraints"]:
        if c["type"] == "altitude_envelope":
            c["alt_min_m"] += 40.0
            c["alt_max_m"] += 40.0 + 60.0
            c["altitude_ref"] = "MSL"
        elif c["type"] == "polygon_fence":
            c["altitude_floor_m"] += 40.0
            c["altitude_ceiling_m"] += 100.0
            c["altitude_ref"] = "MSL"
    data["policy_id"] = "bench-50-msl"
    return data


def _stats(ms: list[float]) -> dict:
    s = sorted(ms)
    p = lambda q: s[max(0, math.ceil(q / 100.0 * len(s)) - 1)]   # noqa: E731
    return {"n": len(s), "median_ms": round(statistics.median(s), 4),
            "p95_ms": round(p(95), 4), "p99_ms": round(p(99), 4),
            "max_ms": round(s[-1], 4)}


_CHILD = r"""
import time; t0 = time.perf_counter()
import json, sys
sys.path.insert(0, sys.argv[1])
from guardrail import terrain
from guardrail.models import Action4D, State, policy_from_raw
from guardrail.shield import Shield
g = terrain.GridGround.load(sys.argv[2])
raw = json.loads(open(sys.argv[3], encoding="utf-8").read())
pol = policy_from_raw(raw, ground=g)
sh = Shield(pol, ground=g, up_ref="terrain")
d = sh.filter(State(x=20.0, y=20.0, up=20.0), Action4D(vx=1.0))
print(json.dumps({"ms": (time.perf_counter() - t0) * 1e3, "rules": len(pol.constraints),
                  "msl": len(sh._rt.msl)}))
"""


def dsl_kpis(dem, fresh_runs: int, warm_runs: int, n_query: int, n_hot: int,
             tmp: Path) -> dict:
    import bench_shield_50rules as bench
    from guardrail.models import DynamicNFZ, wgs84_from_metres
    dem = in_frame(dem)
    raw = fifty_rule_msl_raw()
    tmp.mkdir(parents=True, exist_ok=True)
    pol_file = tmp / "bench50_msl.json"
    pol_file.write_text(json.dumps(raw), encoding="utf-8")
    dem_file = tmp / "dem.json"
    dem_file.write_text(json.dumps(dem.to_dict()), encoding="utf-8")
    # cold start, fresh processes (imports included)
    fresh = []
    for _ in range(fresh_runs):
        out = subprocess.run([sys.executable, "-X", "utf8", "-c", _CHILD, str(ROOT),
                              str(dem_file), str(pol_file)],
                             capture_output=True, text=True, check=True)
        rec = json.loads(out.stdout.strip().splitlines()[-1])
        assert rec["rules"] == 50 and rec["msl"] == 49, rec
        fresh.append(rec["ms"])
    # cold start, warm interpreter
    warm = []
    for _ in range(warm_runs):
        t0 = time.perf_counter()
        g = terrain.GridGround.load(dem_file)
        pol = policy_from_raw(json.loads(pol_file.read_text(encoding="utf-8")), ground=g)
        sh = Shield(pol, ground=g, up_ref="terrain")
        sh.filter(State(x=20.0, y=20.0, up=20.0), Action4D(vx=1.0))
        warm.append((time.perf_counter() - t0) * 1e3)
    # query: _check at the query point, local runtime from cold and warm
    pol = policy_from_raw(raw, ground=dem)
    sh = Shield(pol, ground=dem, up_ref="terrain")
    pairs = []
    for scen in ("grid", "near"):
        pairs += bench.samples(scen, n_query // 2, 0)
    cold_q, warm_q = [], []
    for st, a in pairs:
        sh.__dict__.pop("_msl_rts", None)
        dem._cache.clear()
        t0 = time.perf_counter()
        with sh._at(st):
            sh._check(st, a)
        cold_q.append((time.perf_counter() - t0) * 1e3)
        t0 = time.perf_counter()
        with sh._at(st):
            sh._check(st, a)
        warm_q.append((time.perf_counter() - t0) * 1e3)
    flt = []
    for st, a in pairs[: n_query // 4]:
        sh.__dict__.pop("_msl_rts", None)
        t0 = time.perf_counter()
        sh.filter(st, a)
        flt.append((time.perf_counter() - t0) * 1e3)
    # hot-apply: an MSL dynamic_nfz into the 50-rule MSL Shield, then the
    # first tick at the query point (its local runtime built from cold)
    hot, hot_tick = [], []
    rng = random.Random(0)
    for i in range(n_hot):
        sh = Shield(policy_from_raw(raw, ground=dem), ground=dem, up_ref="terrain")
        cx, cy = rng.uniform(-150.0, -60.0), rng.uniform(0.0, 200.0)
        zone = DynamicNFZ(id=f"hot-{i}", type="dynamic_nfz", **H, altitude_ref="MSL",
                          vertices=[{"x": cx, "y": cy}, {"x": cx + 10, "y": cy},
                                    {"x": cx + 10, "y": cy + 10}, {"x": cx, "y": cy + 10}],
                          altitude_floor_m=40.0, altitude_ceiling_m=160.0)
        # WGS84-only entry (WP1-14, 2026-11-26): converted tool-side, outside
        # the timed hot-apply.
        zone = wgs84_from_metres(zone, sh.policy.frame_origin)
        t0 = time.perf_counter()
        sh.hot_apply(zone)
        t1 = time.perf_counter()
        sh.filter(State(x=cx - 15.0, y=cy + 5.0, up=20.0), Action4D(vx=2.0))
        t2 = time.perf_counter()
        hot.append((t1 - t0) * 1e3)
        hot_tick.append((t2 - t0) * 1e3)
    # the MSL corridor path: an MSL corridor across the ridge + an MSL
    # envelope, _check at points spread over many bands, each from cold
    cor_raw = framed({"policy_id": "msl-corridor-bench", "constraints": [
        {"id": "cor-msl", "type": "corridor", **H, "altitude_ref": "MSL",
         "centerline": [{"x": -250.0, "y": 20.0}, {"x": 0.0, "y": 40.0},
                        {"x": 250.0, "y": 20.0}],
         "width_m": 30.0, "altitude_floor_m": 110.0, "altitude_ceiling_m": 220.0},
        {"id": "env-msl", "type": "altitude_envelope", **H, "altitude_ref": "MSL",
         "alt_min_m": 100.0, "alt_max_m": 240.0}]})
    # segment_check=True: the only path that reads the corridor's tube
    # (Shield._corridor_segment_exit), so the tube cache is exercised.
    sh = Shield(policy_from_raw(cor_raw, ground=dem), ground=dem, up_ref="terrain",
                segment_check=True)
    rng = random.Random(1)
    cor_q = []
    for _ in range(n_query):
        x = rng.uniform(-240.0, 240.0)
        y = 20.0 + 20.0 * (1.0 - abs(x) / 250.0) + rng.uniform(-10.0, 10.0)
        st = State(x=x, y=y, up=170.0 - dem.elevation_at(x, y))
        a = Action4D(vx=rng.uniform(-3.0, 3.0), vy=rng.uniform(-1.0, 1.0),
                     vz_up=rng.uniform(-1.0, 1.0))
        sh.__dict__.pop("_msl_rts", None)
        dem._cache.clear()
        t0 = time.perf_counter()
        with sh._at(st):
            sh._check(st, a)
        cor_q.append((time.perf_counter() - t0) * 1e3)
    n_tubes = len({id(v) for k, v in sh._tubes.items() if isinstance(k, tuple)})
    return {
        "policy": "bench_shield_50rules.fifty_rule_policy, 49 banded rules MSL",
        "frame_origin": list(FRAME),
        "query_check_msl_corridor_local_runtime_cold": _stats(cor_q),
        "msl_corridor_tubes_buffered": n_tubes,
        "cold_start_fresh_process": _stats(fresh) if fresh else None,
        "cold_start_warm_interpreter": _stats(warm),
        "query_check_local_runtime_cold": _stats(cold_q),
        "query_check_local_runtime_warm": _stats(warm_q),
        "filter_tick_local_runtime_cold": _stats(flt),
        "hot_apply_msl_dynamic_nfz": _stats(hot),
        "hot_apply_plus_first_tick": _stats(hot_tick),
        "budgets_ms": {"cold_start": 1000.0, "hot_apply": 50.0, "query": 5.0},
    }


def verdict(fam: dict, kp: dict) -> dict:
    s = fam["summary"]
    cold = (kp["cold_start_fresh_process"] or kp["cold_start_warm_interpreter"])["p95_ms"]
    return {
        "p0_true_escape_ticks_zero": s["p0_true_escape_ticks"] == 0,
        "p0_requested_nonzero": s["p0_requested_ticks"] > 0,
        "nulls_see_violations": (s["null_shield_off_true_p0_ticks"] > 0
                                 and s["null_msl_as_agl_true_p0_ticks"] > 0),
        "failsafe_labels_all_correct": s["failsafe_trigger_correctness"] == 1.0,
        "untouched_flights_unhindered": s["untouched_flights_unhindered"],
        "cold_start_p95_le_1s": cold <= 1000.0,
        "hot_apply_p99_le_50ms": kp["hot_apply_msl_dynamic_nfz"]["p99_ms"] <= 50.0,
        "query_p99_le_5ms": kp["query_check_local_runtime_cold"]["p99_ms"] <= 5.0,
        "msl_corridor_query_p99_le_5ms":
            kp["query_check_msl_corridor_local_runtime_cold"]["p99_ms"] <= 5.0,
    }


CAVEATS = {
    "failsafe_labels": "labels set after a first run that used other labels, matched to "
                       "the library's altitude-ceiling-hold labelling (E5); the 35/35 "
                       "fail-safe figure is not pre-registered and is not an independent "
                       "fail-safe KPI result",
    "scope": "headless only: no rail passes a ground; a DEM Shield requires "
             "up_ref='terrain' and the rails' State.up is height above home; the DEM is "
             "synthetic (a metre grid with no origin on disk, read in the policies' "
             "stated WGS84 frame); no SITL, hil or PAS flight has carried an MSL rule",
}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--write-dem", action="store_true")
    ap.add_argument("--fresh-runs", type=int, default=10)
    ap.add_argument("--warm-runs", type=int, default=30)
    ap.add_argument("--n-query", type=int, default=400)
    ap.add_argument("--n-hot", type=int, default=100)
    ap.add_argument("--no-kpis", action="store_true", help="the family only")
    args = ap.parse_args(argv)
    if args.write_dem:
        DEM_PATH.parent.mkdir(parents=True, exist_ok=True)
        DEM_PATH.write_text(json.dumps(ridge_dem().to_dict(), separators=(",", ":")) + "\n",
                            encoding="utf-8", newline="\n")
        print(f"wrote {DEM_PATH.relative_to(ROOT)}")
    dem = load_dem()
    fam = family(dem)
    print(json.dumps(fam["summary"], indent=1))
    out = {"command": "python tools/msl_dem_kpi.py " + " ".join(argv or sys.argv[1:]),
           "caveats": CAVEATS, "dem": dem.record(), "family": fam}
    if not args.no_kpis:
        import tempfile
        tmp = Path(tempfile.mkdtemp(prefix="msl_dem_"))
        kp = dsl_kpis(dem, args.fresh_runs, args.warm_runs, args.n_query, args.n_hot, tmp)
        out["dsl_kpis"] = kp
        out["verdict"] = verdict(fam, kp)
        print(json.dumps({k: v for k, v in kp.items() if k != "policy"}, indent=1))
        print(json.dumps(out["verdict"], indent=1))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8", newline="\n")
    print(f"wrote {args.out}")
    return 0 if all(out.get("verdict", {"x": True}).values()) else 1


if __name__ == "__main__":
    sys.exit(main())
