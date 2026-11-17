"""Replay logged VLA follow flights through the Shield WITH the subject that
demo/vla_fly.py now feeds it (shield_subject), at four forecasts:

  g_still   5 s / 0.1 s (the grant's; vla_fly's), the subject held still
  g_vel     5 s / 0.1 s, the subject's measured velocity given (as flown now)
  c_still   3 s / 0.5 s (follow_vlm's city default), held still
  c_vel     3 s / 0.5 s, velocity given

Usage:
    python tools/replay_standoff_feed.py [--out-root demo/out] [--jobs 4] [run ...]

With no runs it takes every VLA follow under --out-root whose identity was on
(the ones with a perception subject to feed), --gate-truth flights apart as a
what-if: their box IS the level car's truth, which vla_fly does not feed.

WHY. Until 2026-11-01 vla_fly never set a subject, so the policy's stand-off
rules never ran (CHANGELOG 2026-11-01 "late"). Feeding one raises the
question follow_vlm answered for itself on 2026-10-19: held still, the
grant's 5 s forecast flies the aircraft 25 m on while the car stands, and
StandoffHold fired on 44 % of its replayed ticks. This measures the same on
the VLA's own commands.

HOW. Open loop: each tick's logged raw command (`raw`, what the Shield
judged) at its logged pose, the aircraft's velocity from the pose
differences (the flight used the simulator's kinematics, which the log does
not keep). The subject is rebuilt from detections.jsonl as vla_fly's
_acquire_step keeps it - each held box's map point (the identity record
matching the box), its capture time, the commits, lapses and perception
resets of metrics.follow - and handed to vla_fly.shield_subject itself. Wall
time is mapped to mission time through vla_steps.jsonl (each step's receipt
against the first row that flies it). The level car's truth
(subject_dist_m) only classifies the ticks afterwards.

Each run prints one JSON line; the summary follows. No simulator or GPU.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics as st
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

POLICY = ROOT / "policies" / "follow_car_citylife_nfz.yaml"
VARIANTS = {"g_still": (5.0, 0.1, False), "g_vel": (5.0, 0.1, True),
            "c_still": (3.0, 0.5, False), "c_vel": (3.0, 0.5, True)}
SO_OPS = {"StandoffHold", "StandoffRecover"}
# --path-repair on: PathRepair (the Shield's default chain since 2026-11-17)
# supersedes StandoffHold's record where it detours round the ring, so its
# ticks count as stand-off repairs too. Off (default): the chain every
# replayed flight flew, so the published figures reproduce.
SO_OPS_PR = SO_OPS | {"PathRepair"}


def _jl(p: Path) -> list[dict]:
    with p.open(encoding="utf-8") as fh:
        return [json.loads(ln) for ln in fh if ln.strip()]


def follow_runs(out_root: Path) -> list[tuple[str, bool]]:
    """(run, gate_truth) for every vla_fly follow with the identity on and
    the three logs this replay reads."""
    found = []
    for f in sorted(out_root.glob("*/metrics.json")):
        try:
            m = json.loads(f.read_text(encoding="utf-8"))
        except Exception:                                         # noqa: BLE001
            continue
        mis = m.get("mission")
        if not isinstance(mis, dict) or mis.get("kind") != "follow" \
                or "object_words_are_inert" not in mis:
            continue
        d = f.parent
        if not all((d / x).is_file() for x in
                   ("detections.jsonl", "flight_log.jsonl", "vla_steps.jsonl")):
            continue
        fp = (m.get("params") or {}).get("follow") or {}
        if (fp.get("identity") or {}).get("on"):
            found.append((d.name, bool(fp.get("gate_truth"))))
    return found


def replay(run_dir: str, citymap_dir: str, jump_gate: bool = True,
           path_repair: bool = False) -> dict:
    import occ_bands
    import vla_fly as VF
    from guardrail.models import Action4D, State, load_policy
    from guardrail.replay import rail_shield

    d = Path(run_dir)
    pol = load_policy(POLICY)
    cm = occ_bands.select_for_band(Path(citymap_dir), 6, 14)["map"]
    smap = {"occ": cm["occ"], "res": cm["res"], "ox": cm["ox"], "oy": cm["oy"]}
    m = json.loads((d / "metrics.json").read_text(encoding="utf-8"))
    rows = _jl(d / "flight_log.jsonl")
    steps = _jl(d / "vla_steps.jsonl")
    acq = (m.get("follow") or {}).get("acquisition") or {}
    first_row: dict = {}
    for r in rows:
        s = r.get("vla_seq")
        if s is not None and s not in first_row:
            first_row[s] = r["t"]
    offs = [s["t_received"] - first_row[s["seq"]] for s in steps
            if s.get("t_received") and s.get("seq") in first_row]
    if not offs:
        return {"run": d.name, "error": "no flown VLA step to place the mission clock"}
    t0 = st.median(offs) - 0.05                  # flown from the tick after receipt
    ev = [(c, "commit") for c in acq.get("commits_s") or []]
    ev += [(la["t"], "lapse") for la in acq.get("lapses") or []]
    ev += [(e["t"], "reset") for e in (m.get("follow") or {}).get("perception_resets") or []]
    ev.sort()
    held = []
    for r in _jl(d / "detections.jsonl"):
        if r.get("det") is None or r.get("acquiring"):
            continue
        dd = r["det"]
        P = next((c.get("P") for c in r.get("id") or []
                  if abs(c["cx"] - dd["cx"]) < 0.06 and abs(c["cy"] - dd["cy"]) < 0.06), None)
        if P is not None:
            held.append((r["t"], r.get("t_capture") or r.get("t_cap") or r["t"],
                         r.get("tier"), P))
    held.sort()

    box = VF.new_acquisition(None)
    shields = {k: rail_shield(pol, lookahead_s=a, dt=b, obstacle_map=smap,
                              path_repair=path_repair)
               for k, (a, b, _) in VARIANTS.items()}
    so_ops = SO_OPS_PR if path_repair else SO_OPS
    res = {k: {"so_rep": 0, "cut": [], "truth_lt10": 0, "other_rep": 0, "p0_escapes": 0,
               "pr": 0}
           for k in VARIANTS}
    prio = {c.id: c.priority for c in pol.constraints}
    n = fed = t5 = t5_fed = t5_seen = 0
    err = []
    ei = hi = 0
    prev = None
    first = (acq.get("commits_s") or [math.inf])[0]
    for r in rows:
        t, now = r["t"], t0 + r["t"]
        while ei < len(ev) and ev[ei][0] <= t:
            if ev[ei][1] == "commit":
                box.update(committed=True, hist=[], vxy=None, last=None, seen_t=None,
                           t_ok=-math.inf, v=None)
            else:
                box["committed"] = False
            ei += 1
        while hi < len(held) and held[hi][0] <= now:
            tp, tc, tier, P = held[hi]
            hi += 1
            if not box["committed"]:
                continue
            if jump_gate:
                # vla_fly's own measurement step, its jump gate included.
                VF.take_held_box(box, tp, tc, tier, P)
            elif tier == "ok" or tp - box["t_ok"] < 3.0:
                # As flown until 2026-11-04: every held box taken.
                if tier == "ok":
                    box["t_ok"] = tp
                box["seen_t"] = tp
                box["hist"].append((tp, float(P[0]), float(P[1])))
                del box["hist"][:-40]
                box["vxy"] = VF.measured_velocity(box["hist"])
                box["last"] = {"t": float(tc), "P": (float(P[0]), float(P[1]))}
        subj = VF.shield_subject(box, now, vmax=15.0)
        vel = None
        if prev is not None and t > prev["t"]:
            dt = t - prev["t"]
            vel = ((r["x"] - prev["x"]) / dt, (r["y"] - prev["y"]) / dt,
                   (r["up"] - prev["up"]) / dt)
        prev = r
        state = State(x=r["x"], y=r["y"], up=r["up"], yaw_deg=r.get("psi") or 0.0,
                      velocity=vel)
        a = Action4D(**r["raw"])
        td = r.get("subject_dist_m")
        scored = t >= first
        if scored:
            n += 1
            fed += subj is not None
            if td is not None and td < 5.0:
                t5 += 1
                if subj is not None:
                    t5_fed += 1
                    t5_seen += math.hypot(r["x"] - subj[0], r["y"] - subj[1]) < 5.0
            if subj is not None and r.get("subject_xy"):
                err.append(math.hypot(subj[0] - r["subject_xy"][0],
                                      subj[1] - r["subject_xy"][1]))
        for k, (_, _, use_v) in VARIANTS.items():
            sh = shields[k]
            if subj is None:
                sh.set_subject(None)
            else:
                sh.set_subject(subj[0], subj[1], "car", velocity=subj[2] if use_v else None)
            dec = sh.filter(state, a)
            if not scored:
                continue
            ops = {p.operator for p in dec.repairs}
            rr = res[k]
            rr["pr"] += "PathRepair" in ops
            if ops & so_ops:
                rr["so_rep"] += 1
                rr["cut"].append(math.hypot(a.vx, a.vy)
                                 - math.hypot(dec.emitted.vx, dec.emitted.vy))
                rr["truth_lt10"] += td is not None and td < 10.0
            rr["other_rep"] += bool(ops - so_ops)
            # The Shield's own re-check of what it emitted (kpi.py's escape):
            # a P0 rule still broken after the repair.
            rr["p0_escapes"] += any(prio.get(v.rule_id, "P0") == "P0" for v in dec.violations) \
                and any(prio.get(v.rule_id, "P0") == "P0" for v in dec.emitted_violations)
    return {"run": d.name, "ticks": n, "fed": fed, "jump_gate": jump_gate,
            "path_repair": path_repair,
            "held_jumps_refused": box.get("jumps", 0),
            "truth_lt5": t5, "truth_lt5_fed": t5_fed,
            "truth_lt5_perceived_lt5": t5_seen,
            "subject_err_m": round(st.median(err), 2) if err else None,
            "variants": {k: {"so_rep": rr["so_rep"],
                             "so_rep_frac": round(rr["so_rep"] / max(1, n), 4),
                             "cut_median_mps": (round(st.median(rr["cut"]), 2)
                                                if rr["cut"] else None),
                             "so_rep_truth_lt10": rr["truth_lt10"],
                             "other_rep": rr["other_rep"],
                             "path_repair_ticks": rr["pr"],
                             "p0_escapes": rr["p0_escapes"]} for k, rr in res.items()}}


def summary(results: list[dict], label: str) -> list[str]:
    ok = [r for r in results if "error" not in r]
    n = sum(r["ticks"] for r in ok)
    if not n:
        return [f"{label}: nothing replayed"]
    fed = sum(r["fed"] for r in ok)
    out = [f"{label}: {len(ok)} flights, {n} ticks from the first acquisition; "
           f"{sum(r.get('held_jumps_refused', 0) for r in ok)} held boxes refused as jumps "
           f"(in {sum(1 for r in ok if r.get('held_jumps_refused'))} flights); a subject "
           f"fed on {fed} ({fed / n:.3f}); truth within 5 m on "
           f"{sum(r['truth_lt5'] for r in ok)}, a subject fed on "
           f"{sum(r['truth_lt5_fed'] for r in ok)} of them, placed within 5 m on "
           f"{sum(r['truth_lt5_perceived_lt5'] for r in ok)}",
           f"  {'forecast':8}  {'stand-off repair ticks':>24}  {'flights >5 %':>12}  "
           f"{'worst':>6}  {'median cut':>10}  {'truth <10 m':>11}  {'other repairs':>13}"
           f"  {'P0 escapes':>10}"]
    for k in VARIANTS:
        rep = sum(r["variants"][k]["so_rep"] for r in ok)
        fl = [r["variants"][k]["so_rep_frac"] for r in ok]
        cuts = [r["variants"][k]["cut_median_mps"] for r in ok
                if r["variants"][k]["cut_median_mps"] is not None]
        out.append(f"  {k:8}  {rep:>14} ({rep / n:7.4f})  {sum(f > 0.05 for f in fl):>12}  "
                   f"{max(fl):6.3f}  {(st.median(cuts) if cuts else float('nan')):7.2f} m/s"
                   f"  {sum(r['variants'][k]['so_rep_truth_lt10'] for r in ok):>11}"
                   f"  {sum(r['variants'][k]['other_rep'] for r in ok):>13}"
                   f"  {sum(r['variants'][k].get('p0_escapes', 0) for r in ok):>10}"
                   + (f"  PathRepair {sum(r['variants'][k].get('path_repair_ticks', 0) for r in ok)}"
                      if any(r.get("path_repair") for r in ok) else ""))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("runs", nargs="*")
    ap.add_argument("--out-root", default=str(ROOT / "demo" / "out"))
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--no-jump-gate", action="store_true",
                    help="take every held box, as vla_fly did until 2026-11-04 "
                         "(take_held_box's jump gate off)")
    ap.add_argument("--path-repair", choices=["on", "off"], default="off",
                    help="off (default): the chain every replayed flight flew; on: "
                         "PathRepair in the chain (the Shield's default since "
                         "2026-11-17), its ticks counted as stand-off repairs - the "
                         "open-loop A/B of WP3-06 on the follow flights")
    a = ap.parse_args(argv)
    out_root = Path(a.out_root)
    citymap = out_root / "citymap_citylife"
    known = dict(follow_runs(out_root))
    runs = a.runs or list(known)
    with ProcessPoolExecutor(max_workers=max(1, a.jobs)) as ex:
        results = list(ex.map(replay, [str(out_root / r) for r in runs],
                              [str(citymap)] * len(runs), [not a.no_jump_gate] * len(runs),
                              [a.path_repair == "on"] * len(runs)))
    for r in results:
        print(json.dumps(r))
    for err in (r for r in results if "error" in r):
        print(f"skipped {err['run']}: {err['error']}")
    print()
    for line in summary([r for r in results if not known.get(r["run"])], "PERCEPTION"):
        print(line)
    truth = [r for r in results if known.get(r["run"])]
    if truth:
        for line in summary(truth, "GATE-TRUTH (what-if: vla_fly feeds none)"):
            print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
