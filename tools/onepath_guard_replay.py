"""How much FenceGuard's decisions would move if its building half read the
Shield's distance field (--guard-field shield; cards WP3-22 / X-13).

Run:  python tools/onepath_guard_replay.py [RUN ...] [--root DIR] [--json OUT]

The city controller's FenceGuard (demo/follow_vlm.py) measures building
clearance with a grid scan of its own by default (guard_field="grid",
follow_vlm.grid_scan_clearance: the nearest blocked cell centre). The Shield
measures the same rule (obstacle_clearance, min_clearance_m) on its signed
chamfer distance field, read bilinearly (guardrail/shield.py field_distance).
guard_field="shield" makes the guard read the Shield's field; it is opt-in
until flown. This replays stored CityLife follow flights' logged ticks
through the WHOLE guard decision - gate() (fence and building brake) and,
where the flight loop calls it, slide() (the sidestep) - once per measure,
and counts the ticks whose decision differs.

What is replayed, honestly:
  * the logged position and the commanded velocity before the guard, which
    the log does not hold directly. Where the guard added no sidestep
    (fence_mode clear / near / hold) it is `guard_in / fence_scale`, exact up
    to the log's rounding (scale >= 0.05 only). Where it skirted, guard_in is
    `cv * scale + slide_speed * urgency * side` with side perpendicular to
    cv; the replay solves that for cv with the default --slide-speed 2.5 (two
    mirror solutions) and keeps the one whose grid-scan replay gives back the
    logged decision (skirt). A skirt tick neither solution reproduces is
    counted as not reconstructed and left out;
  * the decision compared per tick is the flight loop's: the mode it would
    log (clear / near / hold / skirt), the sidestep's side, and the building
    brake (gate's building half scale < 1);
  * the grid replay of the reconstructed ticks is also checked against the
    logged fence_mode: the share that agree is printed (the null);
  * the policy is the repository's policies/follow_car_citylife_nfz.yaml
    (the flights flew that file or a signed bundle of it) and the map
    demo/out/citymap_citylife/occ_day.npz banded as follow_vlm selects it,
    with the street mask beside it as fly() loads it;
  * it is an open-loop replay: a changed decision would have changed the
    flight from that tick on, which no replay of a stored log can show.

Nothing here flies, and nothing it prints is a flight result.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

# The follow runs flown on the AGX on 3 Nov 2026 (the current map and
# controller) plus the first one, 2 Nov.
DEFAULT_RUNS = ("agx_follow_01", "agx_manual_20261103", "agx_hier_r1_20261103",
                "agx_hier_r2_20261103", "agx_hier_r3_20261103", "agx_hier_r4_20261103",
                "agx_hier_aerovla_20261103", "agx_hier_dronecats_20261103",
                "agx_hier_racevla_20261103", "agx_hier_fixed12_20261103")

SLIDE_SPEED = 2.5            # follow_vlm --slide-speed default


def flight_setup(lab: Path, policy_path: Path):
    """(policy, map, guard on the grid scan, guard on the Shield field) as
    follow_vlm.fly() builds them for this policy over the CityLife day map."""
    import city_planner
    import occ_bands
    from follow_vlm import FenceGuard, flight_shield
    from guardrail.models import AltitudeEnvelope, ObstacleClearance, load_policy
    pol = load_policy(policy_path)
    cdir = lab / "demo" / "out" / "citymap_citylife"
    cmap = city_planner.load_occ(cdir / "occ_day.npz")
    if cmap is None:
        raise SystemExit(f"no map at {cdir / 'occ_day.npz'}")
    alt = pol.by_type(AltitudeEnvelope)
    if pol.by_type(ObstacleClearance) and alt:
        sel = occ_bands.select_for_band(cdir, alt[0].alt_min_m, alt[0].alt_max_m)
        if sel["map"] is not None:
            cmap = sel["map"]
    smap = {k: cmap[k] for k in ("occ", "res", "ox", "oy")}
    street = None
    if (cdir / "street.npz").is_file():
        from build_street_mask import load_street
        street = load_street(cdir / "street.npz")
    shield = flight_shield(pol, smap)
    clr = pol.by_type(ObstacleClearance)
    mc = clr[0].min_clearance_m if clr else 5.0
    grid = FenceGuard(pol, obstacle_map=smap, min_clearance_m=mc, street_mask=street,
                      shield=shield, guard_field="grid")
    field = FenceGuard(pol, obstacle_map=smap, min_clearance_m=mc, street_mask=street,
                       shield=shield, guard_field="shield")
    return pol, smap, grid, field


def decide(guard, x: float, y: float, cvx: float, cvy: float):
    """The flight loop's guard decision for one tick (follow_vlm.fly):
    (mode, side, building_brake), side = (sx, sy) rounded, or None."""
    fscale, fdist, fblocked = guard.gate(x, y, cvx, cvy)
    ob = guard.last_cause == "obstacle" and fscale < 1.0
    approaching = fdist is not None and fdist < guard.brake_m and fscale < 1.0
    if fblocked or approaching:
        sx, sy, _ = guard.slide(x, y, cvx, cvy)
        if sx or sy:
            return "skirt", (round(sx, 6), round(sy, 6)), ob, fscale
        return ("hold" if fblocked else "near"), None, ob, fscale
    return ("clear" if fdist is None or fdist > 20 else "near"), None, ob, fscale


def skirt_candidates(gi, sc):
    """The commanded velocities a skirt tick's guard_in admits (module doc)."""
    urg = min(1.0, max(0.33, 1.0 - sc))
    lat = SLIDE_SPEED * urg
    g2 = gi[0] ** 2 + gi[1] ** 2
    if g2 <= lat * lat:
        return []
    fc = math.sqrt(g2 - lat * lat)
    ang_g = math.atan2(gi[1], gi[0])
    off = math.atan2(lat, fc)
    c = fc / sc
    # gi = fc * u + lat * sgn * left(u): u is gi's direction turned back by sgn*off
    return [(c * math.cos(ang_g - sgn * off), c * math.sin(ang_g - sgn * off), sgn)
            for sgn in (1.0, -1.0)]


def replay_run(run_dir: Path, grid, field) -> dict:
    rows = [json.loads(line) for line in (run_dir / "flight_log.jsonl").open(encoding="utf-8")]
    n = n_skirt = unrec = agree = 0
    mode_d = side_d = ms_d = brake_d = scale_d = 0
    skirt_now = skirt_old = 0
    by_logged = {}
    for r in rows:
        gi, sc, fm = r.get("guard_in"), r.get("fence_scale"), r.get("fence_mode")
        if gi is None or sc is None or sc < 0.05:
            continue
        x, y = float(r["x"]), float(r["y"])
        if fm == "skirt":
            got = None
            for cvx, cvy, _ in skirt_candidates(gi, sc):
                o = decide(grid, x, y, cvx, cvy)
                if o[0] == "skirt":
                    got = (cvx, cvy, o)
                    break
            if got is None:
                unrec += 1
                continue
            cvx, cvy, o = got
            n_skirt += 1
        else:
            cvx, cvy = gi[0] / sc, gi[1] / sc
            o = decide(grid, x, y, cvx, cvy)
        w = decide(field, x, y, cvx, cvy)
        n += 1
        agree += int(o[0] == fm)
        ob, wb = o[2], w[2]
        brake_d += int(ob != wb)
        scale_d += int(abs(o[3] - w[3]) > 1e-6)
        mode_d += int(o[0] != w[0])
        side_d += int(o[1] != w[1])
        ms_d += int(o[0] != w[0] or o[1] != w[1])
        skirt_old += int(o[0] == "skirt")
        skirt_now += int(w[0] == "skirt")
        k = by_logged.setdefault(fm, [0, 0])
        k[0] += 1
        k[1] += int(o[0] != w[0] or o[1] != w[1])
    return {"run": run_dir.name, "ticks_logged": len(rows), "ticks_replayed": n,
            "skirt_reconstructed": n_skirt, "skirt_not_reconstructed": unrec,
            "grid_replay_agrees_with_log": agree,
            "scale_changed": scale_d, "building_brake_on_off_changed": brake_d,
            "mode_changed": mode_d, "side_changed": side_d, "mode_or_side_changed": ms_d,
            "skirt_ticks_grid": skirt_old, "skirt_ticks_shield": skirt_now,
            "by_logged_mode": {k: {"replayed": v[0], "decision_changed": v[1]}
                               for k, v in sorted(by_logged.items())}}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("runs", nargs="*", default=list(DEFAULT_RUNS))
    ap.add_argument("--root", default=str(ROOT),
                    help="the checkout whose demo/out holds the runs and the map "
                         "(the lab checkout: demo/out is gitignored)")
    ap.add_argument("--policy", default=str(ROOT / "policies" / "follow_car_citylife_nfz.yaml"))
    ap.add_argument("--json", default=None, help="write the per-run table here")
    a = ap.parse_args(argv)
    lab = Path(a.root)
    _, _, grid, field = flight_setup(lab, Path(a.policy))
    out = []
    for name in a.runs:
        rd = lab / "demo" / "out" / name
        if not (rd / "flight_log.jsonl").is_file():
            print(f"{name}: no flight_log.jsonl, skipped")
            continue
        res = replay_run(rd, grid, field)
        out.append(res)
        print(f"{name}: replayed {res['ticks_replayed']}/{res['ticks_logged']} ticks "
              f"({res['skirt_reconstructed']} skirt reconstructed, "
              f"{res['skirt_not_reconstructed']} not); grid replay = logged mode on "
              f"{res['grid_replay_agrees_with_log']}; decision (mode or side) changed "
              f"{res['mode_or_side_changed']}, building brake on/off changed "
              f"{res['building_brake_on_off_changed']}, skirt ticks grid "
              f"{res['skirt_ticks_grid']} -> shield {res['skirt_ticks_shield']}")
    keys = ("ticks_replayed", "skirt_reconstructed", "skirt_not_reconstructed",
            "grid_replay_agrees_with_log", "scale_changed", "building_brake_on_off_changed",
            "mode_changed", "side_changed", "mode_or_side_changed", "skirt_ticks_grid", "skirt_ticks_shield")
    tot = {k: sum(r[k] for r in out) for k in keys}
    bym = {}
    for r in out:
        for m, v in r["by_logged_mode"].items():
            b = bym.setdefault(m, {"replayed": 0, "decision_changed": 0})
            b["replayed"] += v["replayed"]
            b["decision_changed"] += v["decision_changed"]
    tot["by_logged_mode"] = bym
    n = tot["ticks_replayed"]
    if n:
        print(f"ALL {len(out)} runs: replayed {n} ticks ({tot['skirt_not_reconstructed']} skirt "
              f"ticks not reconstructed); grid replay = logged mode on "
              f"{tot['grid_replay_agrees_with_log'] / n:.2%}")
        print(f"  shield field vs grid scan: building brake on/off differs on "
              f"{tot['building_brake_on_off_changed']} ({tot['building_brake_on_off_changed'] / n:.2%}), "
              f"guard scale on {tot['scale_changed']} ({tot['scale_changed'] / n:.2%}), "
              f"decision (mode or sidestep side) on {tot['mode_or_side_changed']} "
              f"({tot['mode_or_side_changed'] / n:.2%}); skirt ticks {tot['skirt_ticks_grid']} -> "
              f"{tot['skirt_ticks_shield']}")
        for m, v in sorted(bym.items()):
            print(f"  logged {m}: {v['decision_changed']}/{v['replayed']} decisions differ "
                  f"({v['decision_changed'] / max(1, v['replayed']):.2%})")
    if a.json:
        Path(a.json).write_text(json.dumps({"runs": out, "total": tot}, indent=1),
                                encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
