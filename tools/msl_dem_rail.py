"""X-11: an MSL rule flown by the headless harness over a WGS84 DEM.

    python tools/msl_dem_rail.py                  # writes docs/data/msl_dem_rail.json
    python tools/msl_dem_rail.py --write-dem      # (re)writes experiments/worlds/dem_cmac_wgs84.json

WHAT THIS RUNS

The stress harness's own episode runner (experiments/sweep_scenarios.py
run_episode) on scenarios that name a declared ground (ScenarioSpec.ground,
2026-11-18): experiments/worlds/dem_cmac_wgs84.json, a WGS84 elevation grid
about CMAC (the ArduPilot SITL home), read and resampled into the policy's
frame by guardrail/terrain.py (load_ground -> Wgs84Grid.to_ground). The
policy is the SITL profile's, experiments/policies/msl_dem_sitl.yaml: one
MSL rule (msl-ceiling, 590-624 m MSL), an AGL terrain clearance (agl-band,
8-60 m above the ground under the aircraft) and kinematic caps. Every Shield
of each episode (the flying one and the harness's judges) gets the ground
with up_ref="terrain"; the vehicle holds its MSL height, so its height above
the ground changes as the ground does.

The family: four pilots flying east (+y) from three starts (x = -60, 0,
60 m north of home) across the DEM's ridge (crest 604 m MSL at 100 m east):

  level-cross     constant 3 m/s east, level, from 12 m: the ridge rises
                  into the AGL floor (the Shield must climb over it);
  climb-cross     constant 3 m/s east, climbing 1 m/s, from 12 m: into the
                  MSL ceiling (the Shield must hold it down);
  terrain-follow  a goto pilot to 200 m east holding 30 m above the ground:
                  over the crest that asks for 634 m MSL, above the ceiling;
  descend         constant 3 m/s east, descending 1 m/s, from 20 m: into
                  the AGL floor over the plain.

P0 ESCAPES, TWO WAYS. (1) The harness's own count (`true_p0_flown_ticks` in each
run's extra: the flown command judged at the true state by a Shield of the
harness's). (2) An ORACLE that uses no Shield: each row's true MSL height
(row `msl_m`, which the integrator keeps) against the DECLARED rules - the
MSL band as written, and the AGL band against the height above the SOURCE
DEM surface (the WGS84 grid's own bilinear surface at the row's lat/lon,
not the resampled one the Shield reads) - every tick.

Two nulls fly the same episodes: the Shield OFF (the oracle must see
violations) and MSL READ AS AGL (msl-ceiling with altitude_ref dropped, the
same numbers, what the DSL did before an MSL height was refused; the oracle
must see escapes of the MSL band).

FINDING (AGL OVER A DEM). Until 2026-11-18 (wf20) the Shield's forecast
held the ground flat over its lookahead: an AGL band was judged on the
height above the ground at the query point plus the commanded climb, so a
level command over RISING ground was let through while the ground ahead came
up under it (189 oracle ticks, at most 0.081 m under the floor, level-cross
only; the first SITL flight of the profile, msl_dem_sitl_01, went 0.49 m
under it for 2.0 s). Since wf20 each forecast pose's up is the height above
the ground under THAT pose and the altitude repair climbs on that forecast
(guardrail/shield.py _over_ground, _repair_altitude_over_ground), aiming
DEM_BAND_MARGIN_M (0.25 m) inside the band; the MSL rule keeps the flat
forecast and is judged in MSL, as before. The run reports, not gates, the
oracle's AGL count and the deepest shortfall below the 8 m floor. The
harness's vehicle is ideal: the margin is the repair's aim, not a tighter
rule, so where nothing is repaired (a crest) the aircraft may settle onto
the floor itself, and a SITL re-flight with baro or EKF noise may still
show sub-cm agl-band ticks. Since wf20 the harness's RTL over the declared
ground climbs to RTL_ALT above home (sweep_scenarios._autopilot, up_home)
and flies the leg home level in MSL, as ArduCopter's RTL does; until then
it climbed to RTL_ALT above the ground under the aircraft.

SCOPE (what this does NOT show): headless only - the harness's ideal
vehicle, not SITL, hil or PAS; the DEM is synthetic (a declared ground: no
public DEM file is in the repository and this run uses no network), though
georeferenced in WGS84 and read through the same loader a published grid
would be; no rail has flown it.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "experiments"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import terrain                                       # noqa: E402
from guardrail.projection import projection_for                     # noqa: E402

DEM_REL = "experiments/worlds/dem_cmac_wgs84.json"
DEM_PATH = ROOT / DEM_REL
POLICY_REL = "experiments/policies/msl_dem_sitl.yaml"
OUT = ROOT / "docs" / "data" / "msl_dem_rail.json"

# The SITL home (sitl/start_sitl.sh, CMAC) and its altitude: the policy's
# frame origin and the DEM's elevation at it.
HOME = (-35.363261, 149.16523)
HOME_MSL = 584.0
# The grid: 91 x 111 nodes every 0.0001 deg (about 11.1 m north, 9.1 m
# east), centred on home: +-500 m north, +-500 m east.
LAT0, LON0, DDEG, NR, NC = HOME[0] - 0.0045, HOME[1] - 0.0055, 0.0001, 91, 111
# The declared rules (experiments/policies/msl_dem_sitl.yaml), for the oracle.
MSL_BAND = (590.0, 624.0)
AGL_BAND = (8.0, 60.0)


AGL_FINDING = ("until 2026-11-18 (wf20) the Shield forecast held the ground under the "
               "aircraft flat over the lookahead: a level command over rising ground "
               "passed the AGL floor check at the query point and the next tick's "
               "ground was higher (189 oracle ticks, max 0.081 m under the floor, "
               "level-cross only). Since wf20 every forecast pose stands over its own "
               "ground and the altitude repair climbs on that forecast, aiming 0.25 m "
               "inside the band (DEM_BAND_MARGIN_M); oracle_agl_ticks is this run's "
               "count, on the harness's ideal vehicle (a SITL re-flight with baro/EKF "
               "noise may still show sub-cm ticks: the margin is the repair's aim, not "
               "a tighter rule). MSL rules keep the flat forecast (judged in MSL)")


def ridge(north_m: float, east_m: float) -> float:
    """The DEM's surface, m MSL, at (north, east) metres from home: 584 m at
    home, a ridge along north at 100 m east rising 20 m (Gaussian, 60 m
    wide: steepest slope 0.29 m/m), on a 0.005 m/m rise to the north."""
    g = 20.0 * (math.exp(-((east_m - 100.0) / 60.0) ** 2) - math.exp(-(100.0 / 60.0) ** 2))
    return HOME_MSL + g + 0.005 * north_m


def cmac_grid() -> terrain.Wgs84Grid:
    p = projection_for(*HOME)
    elev = []
    for r in range(NR):
        row = []
        for c in range(NC):
            n, e = p.to_local(LAT0 + r * DDEG, LON0 + c * DDEG)
            row.append(round(ridge(n, e), 3))
        elev.append(row)
    return terrain.Wgs84Grid(round(LAT0, 9), round(LON0, 9), DDEG, DDEG, elev,
                             "synthetic ridge about CMAC (tools/msl_dem_rail.py ridge), "
                             "a declared ground: not survey data")


def write_dem() -> None:
    d = cmac_grid().to_dict()
    d["description"] = ("X-11 declared ground for the SITL MSL profile "
                        "(experiments/policies/msl_dem_sitl.yaml): WGS84 grid, rows "
                        "south->north, columns west->east, metres MSL; 584 m at the "
                        "SITL home (CMAC)")
    DEM_PATH.parent.mkdir(parents=True, exist_ok=True)
    DEM_PATH.write_text(json.dumps(d, separators=(",", ":")) + "\n", encoding="utf-8",
                        newline="\n")


STARTS = (-60.0, 0.0, 60.0)
# name -> (pilot, start up (m above the ground at the start), target east or None)
PILOTS = {
    "level-cross": ({"type": "constant", "action": {"vx": 0.0, "vy": 3.0, "vz_up": 0.0}},
                    12.0, None),
    "climb-cross": ({"type": "constant", "action": {"vx": 0.0, "vy": 3.0, "vz_up": 1.0}},
                    12.0, None),
    "terrain-follow": ({"type": "goto", "speed": 3.0}, 30.0, 200.0),
    "descend": ({"type": "constant", "action": {"vx": 0.0, "vy": 3.0, "vz_up": -1.0}},
                20.0, None),
}
TICKS = 600


def spec_for(name: str, x0: float, policy_rel: str, shield: bool = True):
    from guardrail.scenario_spec import ScenarioSpec
    pilot, up0, tgt = PILOTS[name]
    mission = {"task_prompt": f"msl-dem {name}",
               "start_pose": {"x": x0, "y": 0.0, "up": up0}, "pilot": pilot}
    if tgt is not None:
        mission["target"] = {"x": x0, "y": tgt, "up": up0}
    return ScenarioSpec.model_validate({
        "scenario_id": f"msl-dem-{name}", "template": "altitude_envelope",
        "description": f"X-11 headless MSL over a WGS84 DEM: {name} from x = {x0:g}",
        "mission": mission, "policy": {"path": policy_rel}, "ground": DEM_REL,
        "ticks": TICKS, "shield": shield,
        "expected_kpis": {"P0_escape_rate": 0}})


def oracle(rows: list[dict], grid: terrain.Wgs84Grid) -> dict:
    """Declared-rule violations per tick, without a Shield: the MSL band at
    the row's true MSL height, the AGL band at its height above the SOURCE
    grid's surface (and, beside it, above the resampled one)."""
    p = projection_for(*HOME)
    msl_t = agl_t = agl_res_t = 0
    lo_agl = math.inf
    hi_msl = -math.inf
    short = over = 0.0
    for r in rows:
        z = float(r["msl_m"])
        lat, lon = p.to_latlon(float(r["x"]), float(r["y"]))
        agl = z - grid.elevation_at_latlon(lat, lon)
        agl_res = float(r["up"])
        lo_agl, hi_msl = min(lo_agl, agl), max(hi_msl, z)
        short = max(short, AGL_BAND[0] - agl)
        over = max(over, z - MSL_BAND[1], MSL_BAND[0] - z)
        msl_t += not (MSL_BAND[0] - 1e-9 <= z <= MSL_BAND[1] + 1e-9)
        agl_t += not (AGL_BAND[0] - 1e-9 <= agl <= AGL_BAND[1] + 1e-9)
        agl_res_t += not (AGL_BAND[0] - 1e-9 <= agl_res <= AGL_BAND[1] + 1e-9)
    return {"msl_ticks": msl_t, "agl_ticks": agl_t, "agl_resampled_ticks": agl_res_t,
            "p0_ticks": sum(1 for r in rows if _viol(r, grid, p)),
            "min_agl_m": round(lo_agl, 3), "max_msl_m": round(hi_msl, 3),
            "max_agl_floor_shortfall_m": round(short, 4),
            "max_msl_band_excess_m": round(over, 4)}


def _viol(r, grid, p) -> bool:
    z = float(r["msl_m"])
    lat, lon = p.to_latlon(float(r["x"]), float(r["y"]))
    agl = z - grid.elevation_at_latlon(lat, lon)
    return (not (MSL_BAND[0] - 1e-9 <= z <= MSL_BAND[1] + 1e-9)
            or not (AGL_BAND[0] - 1e-9 <= agl <= AGL_BAND[1] + 1e-9))


def _agl_policy(tmp: Path) -> str:
    """The null: the SITL profile's policy with msl-ceiling's altitude_ref
    dropped (the same numbers read as AGL). agl-band is left out of it - the
    DSL lint refuses two hard altitude bands with no height in common (590-624
    and 8-60 m, both AGL) - so this null measures the MSL band alone (its
    oracle count is `msl_ticks`)."""
    import yaml
    raw = yaml.safe_load((ROOT / POLICY_REL).read_text(encoding="utf-8"))
    raw["policy_id"] = "msl-dem-sitl-as-agl"
    raw["constraints"] = [c for c in raw["constraints"] if c["id"] != "agl-band"]
    for c in raw["constraints"]:
        c.pop("altitude_ref", None)
    out = tmp / "msl_dem_sitl_as_agl.yaml"
    out.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return str(out)


def run(grid: terrain.Wgs84Grid) -> dict:
    import sweep_scenarios as S
    from guardrail.scenario_spec import Library
    dfl = Library(ROOT / "experiments" / "scenarios.yaml").defaults
    tmp = Path(tempfile.mkdtemp(prefix="msl_dem_rail_"))
    arms = {"on": (POLICY_REL, True), "off": (POLICY_REL, False),
            "msl_as_agl": (_agl_policy(tmp), True)}
    eps: dict[str, list] = {a: [] for a in arms}
    t0 = time.perf_counter()
    for name in PILOTS:
        for x0 in STARTS:
            for arm, (pol, shield) in arms.items():
                spec = spec_for(name, x0, pol, shield)
                run_ = S.run_episode(spec, dfl, 1001)
                ex = run_.extra
                o = oracle(run_.rows, grid)
                st = run_.rows[-1]
                fsm = sorted({r.get("fsm_state_after") for r in run_.rows
                              if r.get("fsm_state_after")})
                eps[arm].append({
                    "pilot": name, "x0": x0, "arm": arm, "ticks": len(run_.rows),
                    "harness_true_p0_ticks": ex.get("true_p0_flown_ticks"),
                    "requested_ticks": {
                        rid: sum(1 for r in run_.rows if any(
                            v.get("rule_id") == rid for v in r.get("violations", [])))
                        for rid in ("msl-ceiling", "agl-band")},
                    "oracle": o, "fsm_states": fsm,
                    "end": {"x": round(st["x"], 2), "y": round(st["y"], 2),
                            "up": round(st["up"], 2), "msl_m": round(st["msl_m"], 2)},
                    "policy_hash": run_.policy_hash_start})
    wall = time.perf_counter() - t0
    on = eps["on"]
    summ = {
        "episodes_per_arm": len(on),
        "on_requested_ticks_msl_ceiling": sum(e["requested_ticks"]["msl-ceiling"] for e in on),
        "on_requested_ticks_agl_band": sum(e["requested_ticks"]["agl-band"] for e in on),
        "on_harness_true_p0_ticks": sum(e["harness_true_p0_ticks"] or 0 for e in on),
        "on_oracle_p0_ticks": sum(e["oracle"]["p0_ticks"] for e in on),
        "on_oracle_msl_ticks": sum(e["oracle"]["msl_ticks"] for e in on),
        "on_oracle_agl_ticks": sum(e["oracle"]["agl_ticks"] for e in on),
        "on_oracle_agl_ticks_by_pilot": {
            n: sum(e["oracle"]["agl_ticks"] for e in on if e["pilot"] == n) for n in PILOTS},
        "on_max_agl_floor_shortfall_m": max(e["oracle"]["max_agl_floor_shortfall_m"]
                                            for e in on),
        "on_max_msl_band_excess_m": max(e["oracle"]["max_msl_band_excess_m"] for e in on),
        "on_min_agl_m": min(e["oracle"]["min_agl_m"] for e in on),
        "on_max_msl_m": max(e["oracle"]["max_msl_m"] for e in on),
        "null_off_oracle_p0_ticks": sum(e["oracle"]["p0_ticks"] for e in eps["off"]),
        "null_off_episodes_violating": sum(1 for e in eps["off"] if e["oracle"]["p0_ticks"]),
        "null_msl_as_agl_oracle_msl_ticks": sum(e["oracle"]["msl_ticks"]
                                                for e in eps["msl_as_agl"]),
        "null_msl_as_agl_episodes_violating": sum(
            1 for e in eps["msl_as_agl"] if e["oracle"]["p0_ticks"]),
        "wall_s": round(wall, 1),
    }
    return {"summary": summ, "episodes": eps}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--write-dem", action="store_true")
    args = ap.parse_args(argv)
    if args.write_dem:
        write_dem()
        print(f"wrote {DEM_REL}")
    grid = terrain.load_wgs84_grid(DEM_PATH)
    ground = terrain.load_ground(DEM_REL, HOME, root=ROOT)
    res = run(grid)
    print(json.dumps(res["summary"], indent=1))
    # Gated: the MSL rule (msl-ceiling, X-11's subject), the harness's own
    # count and the nulls. NOT gated, and never folded into a pass: the
    # independent oracle's P0 count over the whole declared policy, which
    # includes agl-band's ticks under the floor (FINDING in the module
    # docstring; 189 ticks before wf20, when the harness's judge, a Shield
    # with the same flat-ground forecast, could not see them). Both counts
    # are stated side by side in `p0_counts`.
    s = res["summary"]
    gated = {
        "oracle_msl_ticks_zero": s["on_oracle_msl_ticks"] == 0,
        "harness_true_p0_zero": s["on_harness_true_p0_ticks"] == 0,
        "msl_rule_bound": s["on_requested_ticks_msl_ceiling"] > 0,
        "null_off_sees_violations": s["null_off_oracle_p0_ticks"] > 0,
        "null_msl_as_agl_sees_msl_escapes": s["null_msl_as_agl_oracle_msl_ticks"] > 0,
    }
    verdict = {
        "scope": ("gated: the MSL rule msl-ceiling (oracle), the harness's own true-P0 "
                  "count and the two nulls; NOT the declared policy as a whole - "
                  "agl-band's oracle ticks are in not_gated"),
        "gated": gated,
        "not_gated": {
            "oracle_p0_zero": s["on_oracle_p0_ticks"] == 0,
            "oracle_p0_ticks": s["on_oracle_p0_ticks"],
            "oracle_p0_ticks_by_rule": {"msl-ceiling": s["on_oracle_msl_ticks"],
                                        "agl-band": s["on_oracle_agl_ticks"]},
        },
    }
    by = {n: k for n, k in s["on_oracle_agl_ticks_by_pilot"].items() if k}
    p0_counts = (f"harness true-P0 {s['on_harness_true_p0_ticks']}; independent oracle "
                 f"{s['on_oracle_p0_ticks']} P0 ticks (agl-band, max "
                 f"{s['on_max_agl_floor_shortfall_m']:.3f} m under the floor, "
                 f"{(', '.join(by) + ' only') if by else 'no pilot'}); MSL-band ticks "
                 f"{s['on_oracle_msl_ticks']}")
    print(json.dumps(verdict, indent=1))
    print(p0_counts)
    out = {"command": "python tools/msl_dem_rail.py " + " ".join(argv or sys.argv[1:]),
           "policy": POLICY_REL, "ground_file": DEM_REL,
           "ground": ground.record(), "verdict": verdict, "p0_counts": p0_counts,
           "finding_agl_over_dem": {
               "oracle_agl_ticks": res["summary"]["on_oracle_agl_ticks"],
               "by_pilot": res["summary"]["on_oracle_agl_ticks_by_pilot"],
               "max_floor_shortfall_m": res["summary"]["on_max_agl_floor_shortfall_m"],
               "mechanism": AGL_FINDING},
           "scope": ("headless harness (experiments/sweep_scenarios.py run_episode, ideal "
                     "vehicle, seed 1001); synthetic WGS84 DEM; no SITL, hil or PAS "
                     "flight"),
           **res}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8", newline="\n")
    print(f"wrote {args.out}")
    return 0 if all(gated.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
