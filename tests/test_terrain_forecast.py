"""X-11 / wf20 (2026-11-18): the Shield's forecast over a DEM is terrain-aware.

The first SITL flight of the MSL/DEM profile (demo/out/msl_dem_sitl_01 in the
lab checkout) flew level into a rising ridge and went 0.49 m under the AGL
floor (agl-band, 8 m above the grid) for 2.0 s: the Shield's forecast held
the ground under the aircraft flat over its 5 s lookahead, so the first
violation it saw was "alt 7.7m < floor 8.0m, not climbing" with
predicted_at_s 0.0 - after the floor was crossed. Headless, the
independent oracle of tools/msl_dem_rail.py counted 189 such ticks.

WHAT IS PINNED

  (a) Over a DEM (Shield(ground=GridGround, up_ref="terrain")) every forecast
      pose's up is the predicted MSL height less the ground under THAT pose:
      level flight at a constant MSL height towards rising ground is flagged
      on agl-band with predicted_at_s > 0 while the aircraft is still above
      the floor; a climb slower than the ground rises is flagged too (it is
      coming down onto the floor, whatever its vz).
  (b) The altitude repair sizes vz on the same forecast: the emitted climb
      keeps every forecast pose on or above the floor, and re-checks clean.
  (c) An MSL rule stays in MSL: it is judged on the flat forecast (the
      predicted MSL height less the ground under the aircraft now, against
      the band converted over the reach), so a level flight downhill 8 m
      under an MSL ceiling is not refused, and a climb into it still is.
  (d) Without a DEM (no ground, a flat ground) every pose is the flat
      forecast's, the same floats.
  (e) The stored rows of the SITL flight replayed through this Shield: the
      forecast flags agl-band 50 ticks (5.0 s) before tick 156, where the
      flight's own Shield first flagged it. SKIP when the lab checkout's
      demo/out/msl_dem_sitl_01 is absent.
  (f) The ROS 2 node's RTL watch (RTLFailureWatch.assess) is fed the height
      above home, the reference RTL_ALT is in (up_home), while the Shield's
      band stays in the terrain reference: the climb to RTL_ALT above home is
      judged at its height above the ground under the aircraft.

Run: python tests/test_terrain_forecast.py
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "sitl", ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import terrain                                           # noqa: E402
from guardrail.models import Action4D, State, policy_from_raw           # noqa: E402
from guardrail.shield import Shield                                     # noqa: E402

H = {"scope": "mission", "constraint_type": "hard", "priority": "P0",
     "violation_action": "project_fix"}
# The lab checkout's first SITL flight of experiments/policies/msl_dem_sitl.yaml.
SITL_RUN = Path("D:/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone/demo/out/msl_dem_sitl_01")
CMAC = (-35.363261, 149.16523)
DEM = "experiments/worlds/dem_cmac_wgs84.json"


def _ramp(slope: float = 0.2, base: float = 100.0) -> terrain.GridGround:
    """Ground rising `slope` m per metre north (+y) from `base` m MSL at
    y = 0, flat across x; 10 m cells from -200 to 200 m."""
    ys = [-200.0 + 10.0 * j for j in range(41)]
    elev = [[base + slope * y] * 41 for y in ys]
    return terrain.GridGround(-200.0, -200.0, 10.0, elev, "test ramp")


def _agl_policy(lo: float = 8.0, hi: float = 60.0, msl=None):
    cons = [{"id": "agl-band", "type": "altitude_envelope", **H,
             "alt_min_m": lo, "alt_max_m": hi},
            {"id": "kin", "type": "kinematic_envelope", **H, "priority": "P1",
             "speed_max_mps": 6.0, "climb_rate_max_mps": 3.0, "yaw_rate_max_dps": 45.0}]
    if msl is not None:
        cons.insert(0, {"id": "msl-ceiling", "type": "altitude_envelope", **H,
                        "alt_min_m": msl[0], "alt_max_m": msl[1], "altitude_ref": "MSL"})
    return policy_from_raw({"policy_id": "terrain-forecast-test", "constraints": cons},
                           runtime=False)


def _shield(policy, ground):
    return Shield(policy, ground=ground, up_ref="terrain")


def _agl(vs):
    return [v for v in vs if v.rule_id == "agl-band"]


def test_level_flight_towards_rising_ground_is_flagged_before_the_floor():
    """(a) 10 m above the ground, level at 3 m/s north up a 0.2 slope: the
    ground under the forecast rises 0.6 m/s, so the floor (8 m) is met
    3.33 s ahead. The flat forecast saw nothing until the aircraft was
    under the floor."""
    sh = _shield(_agl_policy(), _ramp())
    st = State(x=0.0, y=0.0, up=10.0, yaw_deg=0.0)
    level = Action4D(vx=0.0, vy=3.0, vz_up=0.0, yaw_rate=0.0)
    v = _agl(sh.recheck_commanded(st, level))
    assert v, "level flight into rising ground was not flagged"
    assert 3.3 < v[0].predicted_at_s <= 3.5, v[0]
    assert st.up > 8.0
    # The poses the rule judged: each over the ground under it.
    poses = sh.forecast(st, level)
    for t, p in poses:
        assert abs(p.up - (10.0 - 0.2 * 3.0 * t)) < 1e-9, (t, p.up)
    # A climb slower than the ground rises is coming down onto the floor:
    # from 9 m, 0.3 m/s up over 0.6 m/s of rise meets it 3.33 s ahead.
    slow = Action4D(vx=0.0, vy=3.0, vz_up=0.3, yaw_rate=0.0)
    v = _agl(sh.recheck_commanded(State(x=0.0, y=0.0, up=9.0, yaw_deg=0.0), slow))
    assert v and 3.3 < v[0].predicted_at_s <= 3.5, "a 0.3 m/s climb over a 0.6 m/s rise passed"
    # A climb that keeps pace passes; level flight downhill passes.
    keep = Action4D(vx=0.0, vy=3.0, vz_up=0.7, yaw_rate=0.0)
    assert not _agl(sh.recheck_commanded(st, keep))
    down = Action4D(vx=0.0, vy=-3.0, vz_up=0.0, yaw_rate=0.0)
    assert not _agl(sh.recheck_commanded(st, down))


def test_below_the_floor_a_recovery_is_judged_against_the_ground():
    """Already under the floor (7 m): climbing 1 m/s over ground rising
    0.6 m/s gains height above the ground - a recovery, passed; climbing
    0.5 m/s over it loses height above the ground - refused."""
    sh = _shield(_agl_policy(), _ramp())
    st = State(x=0.0, y=0.0, up=7.0, yaw_deg=0.0)
    assert not _agl(sh.recheck_commanded(st, Action4D(vx=0.0, vy=3.0, vz_up=1.0,
                                                      yaw_rate=0.0)))
    assert _agl(sh.recheck_commanded(st, Action4D(vx=0.0, vy=3.0, vz_up=0.5,
                                                  yaw_rate=0.0)))


def test_the_repair_climbs_early_enough_and_re_checks_clean():
    """(b) The level command is repaired to the slowest climb that keeps
    every forecast pose on or above the floor: max_t (8 - 10 + 0.6 t) / t =
    0.2 m/s (at t = 5 s), and the emitted action re-checks clean."""
    sh = _shield(_agl_policy(), _ramp())
    st = State(x=0.0, y=0.0, up=10.0, yaw_deg=0.0)
    d = sh.filter(st, Action4D(vx=0.0, vy=3.0, vz_up=0.0, yaw_rate=0.0), t=0.0)
    vz = d.emitted.vz_up
    assert abs(vz - 0.2) < 1e-6, (vz, d.repairs)
    assert any(r.operator == "AltitudeFix" for r in d.repairs), d.repairs
    assert not sh.hard_violations(sh.recheck_commanded(st, d.emitted))
    assert min(p.up for _, p in sh.forecast(st, d.emitted)) >= 8.0 - 1e-9


def test_an_msl_rule_stays_in_msl():
    """(c) Ground falling 0.3 m per metre north; an MSL ceiling of 624 m.
    Level at 616 m MSL (8 m under it) at 5 m/s downhill: the MSL height
    never changes, so the MSL rule must not refuse it (on the forecast over
    the ground, judged against the band converted over the 20 m reach, it
    read 7.5 m of falling ground as a climb into the ceiling: the RTL leg
    of the terrain-follow cell, a false harness true-P0). Climbing 2 m/s
    from 620 m is still refused."""
    g = _ramp(slope=-0.3, base=600.0)
    pol = _agl_policy(lo=8.0, hi=60.0, msl=(590.0, 624.0))
    sh = _shield(pol, g)
    g0 = g.elevation_at(0.0, 0.0)
    st = State(x=0.0, y=0.0, up=616.0 - g0, yaw_deg=0.0)
    level = Action4D(vx=0.0, vy=5.0, vz_up=0.0, yaw_rate=0.0)
    vs = sh.recheck_commanded(st, level)
    assert not [v for v in vs if v.rule_id == "msl-ceiling"], vs
    up = State(x=0.0, y=0.0, up=620.0 - g0, yaw_deg=0.0)
    climb = Action4D(vx=0.0, vy=5.0, vz_up=2.0, yaw_rate=0.0)
    assert [v for v in sh.recheck_commanded(up, climb) if v.rule_id == "msl-ceiling"]
    # The AGL band, on the same Shield, still reads the ground under each pose.
    for t, p in sh.forecast(st, level):
        assert abs(p.up - (st.up + (g0 - g.elevation_at(p.x, p.y)))) < 1e-9


def test_without_a_dem_the_forecast_is_the_flat_one():
    """(d) No ground, and a flat ground: the same poses as the flat
    forecast, float for float, and the level command over the 'ramp' is not
    flagged (there is no ramp)."""
    pol = _agl_policy()
    st = State(x=0.0, y=0.0, up=10.0, yaw_deg=0.0)
    level = Action4D(vx=0.0, vy=3.0, vz_up=-0.1, yaw_rate=0.0)
    for sh in (Shield(pol), Shield(pol, ground=terrain.FlatGround(100.0, "flat"))):
        assert sh._dem is None
        assert list(sh._predict(st, level)) == list(sh._predict_flat(st, level))
        assert not _agl(sh.recheck_commanded(st, Action4D(vx=0.0, vy=3.0, vz_up=0.0,
                                                          yaw_rate=0.0)))


def test_the_sitl_rows_replayed_flag_agl_band_50_ticks_earlier():
    """(e) msl_dem_sitl_01's stored rows (lab checkout; SKIP if absent): each
    tick's state (its up above the grid) and raw command through a Shield
    with the flight's policy and ground. The flight's Shield first flagged
    agl-band at tick 156 (predicted_at_s 0.0, the floor already crossed);
    this forecast flags it at tick 106 (predicted_at_s 5.0)."""
    rows_f = SITL_RUN / "flight_log.jsonl"
    if not rows_f.is_file():
        print(f"SKIP  {rows_f} absent")
        return "skip"
    from guardrail.models import load_policy
    g = terrain.load_ground(DEM, CMAC, root=ROOT)
    pol = load_policy(ROOT / "experiments" / "policies" / "msl_dem_sitl.yaml", ground=g)
    sh = _shield(pol, g)
    rows = [json.loads(line) for line in rows_f.read_text(encoding="utf-8").splitlines()
            if line.strip()]
    flown = next(r["tick"] for r in rows
                 if any(v["rule_id"] == "agl-band" for v in r["violations"]))
    assert flown == 156
    first = None
    for r in rows:
        if r["tick"] > flown:
            break
        st = State(x=r["x"], y=r["y"], up=r["up"], yaw_deg=r["yaw_deg"])
        v = _agl(sh.recheck_commanded(st, Action4D(**r["raw"])))
        if v:
            first = (r["tick"], v[0].predicted_at_s, r["up"])
            break
    assert first is not None, "the replay never flagged agl-band"
    n = flown - first[0]
    print(f"  replay: agl-band first flagged at tick {first[0]} (predicted_at_s "
          f"{first[1]:.2f}, up {first[2]:.3f} m above the grid), {n} ticks before {flown}")
    assert first[1] > 0 and first[2] > 8.0
    assert n >= 50, n


def test_the_rtl_watch_reads_rtl_alt_above_home():
    """(f) 60 m from home over ground 20 m below home: 30 m above the ground
    is 10 m above home. RTL climbs to RTL_ALT (15 m) ABOVE HOME first: 35 m
    above the ground here, over a band whose ceiling is 32 m - a blocked
    leg. Fed the terrain height alone (the gap), the watch read 30 m as
    above RTL_ALT and the leg as clear. The progress test counts the 5 m
    climb above home."""
    import ros2_shield_node as N
    from guardrail.rtl_path import Band
    band = Band("agl-band", 8.0, 32.0)
    old = N.RTLFailureWatch((0.0, 0.0)).assess(0.0, 60.0, 0.0, 30.0, "RTL", [], band)
    assert not old.failed, old
    new = N.RTLFailureWatch((0.0, 0.0)).assess(0.0, 60.0, 0.0, 30.0, "RTL", [], band,
                                               up_home=10.0)
    assert new.failed and "blocked_path" in new.kinds, new
    assert "35" in new.note and "agl-band" in new.note, new.note
    # Without up_home, or with the same height, nothing changes.
    same = N.RTLFailureWatch((0.0, 0.0)).assess(0.0, 60.0, 0.0, 30.0, "RTL", [], band,
                                                up_home=30.0)
    assert same == old
    # The progress test: 10 m above home, holding position over home's
    # distance - the 5 m climb is still to fly.
    from guardrail.rtl_path import remaining_to_home
    w = N.RTLFailureWatch((0.0, 0.0))
    seen = []
    w._progress.update = lambda t, rem: (seen.append(rem), (False, ""))[1]
    w.assess(0.0, 60.0, 0.0, 30.0, "RTL", None, None, up_home=10.0)
    assert seen == [remaining_to_home(60.0, 0.0, 10.0, (0.0, 0.0))] == [65.0], seen


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = skipped = 0
    for t in tests:
        try:
            r = t()
            if r == "skip":
                skipped += 1
            print(f"{'SKIP' if r == 'skip' else 'PASS'}  {t.__name__}")
        except Exception as e:                                      # noqa: BLE001
            failed += 1
            import traceback
            traceback.print_exc()
            print(f"FAIL  {t.__name__}: {type(e).__name__}: {e}")
    print(f"{len(tests) - failed}/{len(tests)} passed ({skipped} skipped)")
    sys.exit(1 if failed else 0)
