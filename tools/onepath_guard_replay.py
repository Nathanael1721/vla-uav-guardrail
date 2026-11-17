"""How much FenceGuard's decisions moved when its building half began reading
the Shield's distance field (2026-11-17, cards WP3-22 / X-13).

Run:  python tools/onepath_guard_replay.py [RUN ...] [--root DIR] [--json OUT]

Until 2026-11-17 the city controller's FenceGuard (demo/follow_vlm.py)
measured building clearance with a grid scan of its own: the distance from a
point to the nearest blocked cell centre. The Shield measures the same rule
(obstacle_clearance, min_clearance_m) on its signed chamfer distance field,
read bilinearly (guardrail/shield.py field_distance). FenceGuard now reads
the Shield's field. This replays stored CityLife follow flights' logged
ticks through the building half (`FenceGuard._obstacle_gate`, gate()'s
obstacle brake) twice - once with the old grid scan (kept HERE only, as the
replay's null) and once as the guard now measures - and counts the ticks
whose verdict differs.

What is replayed, honestly:
  * the logged position and the commanded velocity before the guard, which
    the log does not hold directly: it is `guard_in / fence_scale`, exact
    only where the guard scaled without a sidestep (fence_mode not 'skirt')
    and the scale is at least 0.05. Other ticks are not replayed; the count
    of replayed ticks is printed;
  * the policy is the repository's policies/follow_car_citylife_nfz.yaml
    (the flights flew that file or a signed bundle of it) and the map
    demo/out/citymap_citylife/occ_day.npz banded as follow_vlm selects it;
  * it is an open-loop replay: a changed verdict would have changed the
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


def grid_scan_clearance(occ_map: dict, px: float, py: float, cap_m: float) -> float:
    """FenceGuard.clearance as it was until 2026-11-17: the nearest blocked
    cell centre within `cap_m` (cap_m when none). The replay's null only."""
    occ, res = occ_map["occ"], occ_map["res"]
    ox, oy = occ_map["ox"], occ_map["oy"]
    n, m = occ.shape
    r = int(math.ceil(cap_m / res))
    i0 = int(round((px - ox) / res))
    j0 = int(round((py - oy) / res))
    best = cap_m
    for i in range(max(0, i0 - r), min(n, i0 + r + 1)):
        for j in range(max(0, j0 - r), min(m, j0 + r + 1)):
            if occ[i, j]:
                d = math.hypot(ox + i * res - px, oy + j * res - py)
                if d < best:
                    best = d
    return best


def flight_setup(lab: Path, policy_path: Path):
    """(policy, obstacle map, flight Shield, FenceGuard) as follow_vlm.fly()
    builds them for this policy over the CityLife day map."""
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
    shield = flight_shield(pol, smap)
    clr = pol.by_type(ObstacleClearance)
    guard = FenceGuard(pol, obstacle_map=smap,
                       min_clearance_m=(clr[0].min_clearance_m if clr else 5.0), shield=shield)
    return pol, smap, shield, guard


def replay_run(run_dir: Path, smap: dict, guard) -> dict:
    rows = [json.loads(line) for line in (run_dir / "flight_log.jsonl").open(encoding="utf-8")]
    now_fn = guard.clearance
    old_fn = (lambda px, py, cap: grid_scan_clearance(smap, px, py, cap))
    n = scale_d = blocked_d = brake_d = 0
    worst = 0.0
    pose_d = []
    for r in rows:
        gi, sc = r.get("guard_in"), r.get("fence_scale")
        if gi is None or sc is None:
            continue
        x, y = float(r["x"]), float(r["y"])
        pose_d.append(now_fn(x, y, 12.0) - old_fn(x, y, 12.0))
        if sc < 0.05 or r.get("fence_mode") == "skirt":
            continue
        vx, vy = gi[0] / sc, gi[1] / sc
        guard.clearance = old_fn
        try:
            o = guard._obstacle_gate(x, y, vx, vy)
        finally:
            guard.clearance = now_fn
        w = guard._obstacle_gate(x, y, vx, vy)
        n += 1
        if abs(o[0] - w[0]) > 1e-6:
            scale_d += 1
            worst = max(worst, abs(o[0] - w[0]))
        blocked_d += int(o[2] != w[2])
        brake_d += int((o[0] < 1.0) != (w[0] < 1.0))
    pose_d.sort()
    med = pose_d[len(pose_d) // 2] if pose_d else None
    return {"run": run_dir.name, "ticks_logged": len(rows), "ticks_replayed": n,
            "scale_changed": scale_d, "blocked_changed": blocked_d,
            "brake_on_off_changed": brake_d, "worst_scale_change": round(worst, 3),
            "pose_clearance_now_minus_old_median_m": None if med is None else round(med, 3),
            "pose_clearance_now_minus_old_min_m": round(pose_d[0], 3) if pose_d else None,
            "pose_clearance_now_minus_old_max_m": round(pose_d[-1], 3) if pose_d else None}


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
    _, smap, _, guard = flight_setup(lab, Path(a.policy))
    out = []
    for name in a.runs:
        rd = lab / "demo" / "out" / name
        if not (rd / "flight_log.jsonl").is_file():
            print(f"{name}: no flight_log.jsonl, skipped")
            continue
        res = replay_run(rd, smap, guard)
        out.append(res)
        print(f"{name}: replayed {res['ticks_replayed']}/{res['ticks_logged']} ticks; building "
              f"half scale changed {res['scale_changed']}, blocked changed "
              f"{res['blocked_changed']}, brake on/off changed {res['brake_on_off_changed']} "
              f"(worst scale change {res['worst_scale_change']}); clearance now-old "
              f"median {res['pose_clearance_now_minus_old_median_m']} m, range "
              f"[{res['pose_clearance_now_minus_old_min_m']}, "
              f"{res['pose_clearance_now_minus_old_max_m']}] m")
    tot = {k: sum(r[k] for r in out) for k in ("ticks_replayed", "scale_changed",
                                                "blocked_changed", "brake_on_off_changed")}
    if tot["ticks_replayed"]:
        print(f"ALL {len(out)} runs: replayed {tot['ticks_replayed']} ticks; scale changed on "
              f"{tot['scale_changed']} ({tot['scale_changed'] / tot['ticks_replayed']:.2%}), "
              f"blocked on {tot['blocked_changed']} "
              f"({tot['blocked_changed'] / tot['ticks_replayed']:.2%}), brake on/off on "
              f"{tot['brake_on_off_changed']} "
              f"({tot['brake_on_off_changed'] / tot['ticks_replayed']:.2%})")
    if a.json:
        Path(a.json).write_text(json.dumps({"runs": out, "total": tot}, indent=1),
                                encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
