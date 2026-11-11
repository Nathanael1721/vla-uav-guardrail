"""Numbers for the 11 Nov 2026 progress deck, read from the hil campaigns' own
files: every value on a slide comes from here (tools/deck/build_progress_1111_deck.js).

    python tools/deck/build_progress_1111_data.py [--labels DIR]

Sources
  demo/out/hil_<campaign>/sweep.json        the campaign's own summary (sweep_scenarios.py)
  demo/out/hil_smoke/<campaign>_*/flight_log.jsonl, manifest.json   per-tick timing, manifests
  <labels>/<campaign>.json                  `sweep_scenarios.py --rescore-rail` output with the
                                            fail-safe readings (a) / (r) / (r2) (branch
                                            adopt-rail-labels, docs/data/failsafe_labels_rail.json)
Writes docs/share/2026-11-11-Progress/_build/progress_1111.json.
"""
import argparse
import glob
import json
import math
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "docs/share/2026-11-11-Progress/_build/progress_1111.json"
STORE = REPO / "demo/out/hil_smoke"
CAMPAIGNS = {  # campaign -> what it is
    "rh03": "smoke, 7 Nov (fb96fe5)",
    "sm1110": "smoke, 10 Nov (9eed35b)",
    "sm1111": "smoke, 11 Nov (22d434b)",
    "ge1110": "grant example sweep, 10 Nov (9eed35b)",
}


def pct(v, q):
    s = sorted(v)
    return s[min(len(s) - 1, int(q * (len(s) - 1) + 0.5))] if s else None


def ticks(campaign):
    tick, e2e, rows, over = [], [], 0, 0
    for fl in glob.glob(str(STORE / f"{campaign}_*" / "flight_log.jsonl")):
        for line in open(fl, encoding="utf-8"):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            rows += 1
            t = r.get("tick_ms")
            if isinstance(t, (int, float)):
                tick.append(t)
                over += t > 100.0
            if isinstance(r.get("e2e_ms"), (int, float)):
                e2e.append(r["e2e_ms"])
    return {"rows": rows, "tick_n": len(tick), "tick_p50": pct(tick, .5), "tick_p99": pct(tick, .99),
            "tick_max": max(tick) if tick else None, "tick_over_100": over,
            "e2e_p50": pct(e2e, .5), "e2e_p99": pct(e2e, .99)}


def manifests(campaign):
    revs, topo, speed = set(), set(), set()
    for m in glob.glob(str(STORE / f"{campaign}_*" / "manifest.json")):
        d = json.load(open(m, encoding="utf-8"))
        revs.add(d.get("code_revision")); topo.add(d.get("topology")); speed.add(d.get("sim_speedup"))
    return {"code_revisions": sorted(r for r in revs if r), "topologies": sorted(t for t in topo if t),
            "sim_speedups": sorted(s for s in speed if s is not None)}


def sweep(campaign):
    d = json.load(open(REPO / f"demo/out/hil_{campaign}/sweep.json", encoding="utf-8"))
    on = [r for r in d["results"] if (r.get("kpi") or {}).get("p0_violation_escape_rate") is not None]
    shield_on = [r for r in on if "control" not in r["scenario_id"]]
    control = [r for r in on if "control" in r["scenario_id"]]
    p0_on = [r["kpi"]["p0_violation_escape_rate"] for r in shield_on]
    fl = d.get("_failsafe_labels") or {}
    ts = d.get("_true_state") or {}
    tts = [r["kpi"].get("mean_time_to_safe_s") for r in shield_on
           if isinstance((r.get("kpi") or {}).get("mean_time_to_safe_s"), (int, float))]
    tts_max = [r["kpi"].get("max_time_to_safe_s") for r in shield_on
               if isinstance((r.get("kpi") or {}).get("max_time_to_safe_s"), (int, float))]
    outcomes = [r["kpi"].get("outcome") for r in shield_on]
    reps = [r["kpi"].get("repair_count") for r in shield_on if isinstance(r["kpi"].get("repair_count"), (int, float))]
    mags = [r["kpi"].get("mean_repair_magnitude_mps") for r in shield_on
            if isinstance(r["kpi"].get("mean_repair_magnitude_mps"), (int, float))]
    return {
        "outcomes": {o: outcomes.count(o) for o in sorted(set(o for o in outcomes if o))},
        "mission_success_rate": round(outcomes.count("success") / len(outcomes), 3) if outcomes else None,
        "repairs_per_episode": round(sum(reps) / len(reps), 1) if reps else None,
        "repair_magnitude_mps": round(sum(mags) / len(mags), 2) if mags else None,
        "tts_max_s": max(tts_max) if tts_max else None,
        "counts": d["_counts"], "runtime_h": round(d["_runtime_s"] / 3600, 2),
        "missions_flown": sum(v for k, v in d["_counts"].items() if k != "skipped"),
        "shield_on_scored": len(shield_on),
        "p0_shield_on_max": max(p0_on) if p0_on else None,
        "p0_control": [round(r["kpi"]["p0_violation_escape_rate"], 3) for r in control],
        "true_state_entries": ts.get("episodes_entering_polygon"),
        "breach_episodes": ts.get("episodes_with_breach"),
        "failsafe_ideal": {k: fl.get(k) for k in ("labelled", "correct", "false_triggers", "missed_triggers",
                                                  "censored_in_loiter")},
        "tts_mean_s": round(sum(tts) / len(tts), 2) if tts else None, "tts_episodes": len(tts),
    }


def readings(labels_dir, campaign):
    p = Path(labels_dir) / f"{campaign}.json"
    if not p.exists():
        return None
    fl = json.load(open(p, encoding="utf-8"))["failsafe_labels"]
    out = {"adopted": fl["adopted_kpi_reading"], "artefact": fl["artefact"]}
    for key, r in fl["readings"].items():
        out[key] = {"correct": r["correct"], "scored": r["scored"],
                    "rate": r["failsafe_trigger_correctness"], "wilson_low": r["wilson_low_95"],
                    "null_never": r["null_never_trigger"],
                    "false": [x["run"] for x in r.get("false", [])],
                    "missed": [x["run"] for x in r.get("missed", [])]}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--labels", default=str(OUT.parent / "labels"),
                    help="folder of <campaign>.json files written by sweep_scenarios.py --rescore-rail")
    a = ap.parse_args()
    data = {"campaigns": {}}
    for c, what in CAMPAIGNS.items():
        data["campaigns"][c] = {"what": what, "sweep": sweep(c), "ticks": ticks(c),
                                "manifests": manifests(c), "readings": readings(a.labels, c)}
    for c, v in data["campaigns"].items():
        for x in json.dumps(v):
            pass
        if any(isinstance(n, float) and math.isnan(n) for n in v["ticks"].values() if n is not None):
            raise SystemExit(f"{c}: NaN in tick stats")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(data, indent=1), encoding="utf-8")
    for c, v in data["campaigns"].items():
        s, t, r = v["sweep"], v["ticks"], v["readings"] or {}
        print(c, "| flown", s["missions_flown"], "| P0 on max", s["p0_shield_on_max"], "control", s["p0_control"],
              "| tick max", t["tick_max"], "over100", t["tick_over_100"], "| revs", v["manifests"]["code_revisions"],
              v["manifests"]["topologies"],
              "|", {k: f"{r[k]['correct']}/{r[k]['scored']}" for k in ("a", "r", "r2") if k in r})
    print("->", OUT)


if __name__ == "__main__":
    main()
