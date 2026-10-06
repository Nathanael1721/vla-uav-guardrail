"""Numbers for the 2026-09-30 progress deck, gathered in one file.

    C:/Users/natha/.conda/envs/pas/python.exe tools/deck/build_progress_0930_data.py

Writes docs/data/progress_0930.json. The flight rows are read from each
flight's own metrics.json / kpi.json / flight_log.jsonl, the replay figures
from demo/out/identity_replay/{pipeline,report}.json and the signal survey
from docs/data/citylife_signals.json - nothing about a flight is typed in.

The Simulate counters are the one exception: they were read live from the
running editor (tools/citylife_mcp/verify_{drive,signals,peds}.py) and exist
only as those tools' printed output, recorded in docs/WORKLOG.md
(2026-09-30 entries). They are copied below with that source named, so the
deck says where each came from.
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "demo" / "out"
sys.path.insert(0, str(ROOT / "demo"))
import track_truth as T                                          # noqa: E402

RED_NEW = ["citylife_redcar_id1", "citylife_redcar_id2", "citylife_redcar_id3",
           "citylife_redcar_id4"]
RED_OLD = ["citylife_redcar_trail", "citylife_redcar_trail2", "citylife_redcar_trail3",
           "citylife_redcar_final1", "citylife_redcar_final2", "citylife_redcar_final3",
           "citylife_redcar_final4"]
OTHER = ["citylife_ped_id", "citylife_ped_0930", "citylife_ped_final",
         "demo_follow_identity", "demo_follow_trail2"]

SIMULATE = {
    "source": "docs/WORKLOG.md, 2026-09-30 entries (verify_drive/verify_signals/verify_peds output)",
    "before_fix_t_s": 1933, "before_fix_off_route": 25, "before_fix_figures": 40,
    "before_fix_in_cross": 20, "before_fix_median_move_m": 1.8,
    "after_fix_t_s": [676, 1972], "after_fix_off_route": [0, 0], "after_fix_reads": [120, 120],
    "after_fix_teleports": [0, 0], "after_fix_detours": [210, 553],
    "after_fix_crossings": [100, 294], "after_fix_median_move_m": [65.9, 69.9],
    "kerb_wait_max_s": 194.4, "figures_waited_over_100_s": 8,
    "cars_red_violations": 0, "cars_zebra_wait": 0, "cars_ped_violations": 0,
    "cars_min_gap_cm": 650, "cars_longest_standstill_s": 56.0,
    "lamps_checked": 33, "lamps_ok": 33, "lamps_driven": 258,
    "zebra_figures_before_nkind": 6, "zebra_figures_after_nkind": [1, 0],
    "suite_tests": 793,
}


def jload(p):
    return json.loads(Path(p).read_text(encoding="utf-8"))


def max_sep(rows):
    best = None
    for r in rows:
        pts = T.truth_points(r)
        if pts:
            d = min(math.hypot(r["x"] - a, r["y"] - b) for a, b in pts)
            best = d if best is None else max(best, d)
    return None if best is None else round(best, 1)


def flight(tag):
    d = OUT / tag
    m = jload(d / "metrics.json")
    kpi = jload(d / "kpi.json") if (d / "kpi.json").exists() else {}
    rows = T.load_rows(d)
    sg = m.get("start_gate") or {}
    rq = m.get("reacquisition") or {}
    est = m.get("estimate_on_subject") or {}
    land = m.get("landing") or {}
    col = m.get("collisions") or {}
    return {
        "tag": tag,
        "object": m.get("object"),
        "within_30m": m.get("frac_within_30m"),
        "sep_mean_m": m.get("sep_mean_m"), "sep_min_m": m.get("sep_min_m"),
        "sep_max_m": max_sep(rows),
        "det_hz": m.get("det_hz"),
        "loop_hz": round((len(rows) - 1) / (rows[-1]["t"] - rows[0]["t"]), 2) if len(rows) > 1 else None,
        "estimate_on_subject": est.get("on_subject_frac"),
        "estimate_err_median_m": est.get("err_median_m"),
        "lapses": rq.get("lapses"), "reacquired": rq.get("reacquired"),
        "reacq_events": [{k: e.get(k) for k in ("t", "event", "after_s", "rng_h")}
                         for e in (rq.get("events") or [])],
        "refused_by": rq.get("refused_by"),
        "far_approach_ticks": rq.get("far_approach_ticks"),
        "mode_frac": m.get("mode_frac"),
        "gate_wait_s": sg.get("wait_s"), "gate_range_m": sg.get("subject_range_m"),
        "p0": kpi.get("p0_violation_escape_rate", m.get("p0_violation_escape_rate")),
        "interventions": m.get("interventions"),
        "collisions_mission": col.get("mission"),
        "landing_reached": land.get("reached"), "landing_final_landable": land.get("final_landable"),
        "landing_path_m": land.get("path_m"), "landing_reasons": land.get("final_reasons"),
        "presence_blocked_ticks": m.get("presence_blocked_ticks"),
        "frac_ticks_seen": m.get("frac_ticks_seen"),
        "frac_on_target": m.get("frac_on_target"),
        "frac_on_target_chance": m.get("frac_on_target_chance"),
        "code_revision": (m.get("manifest") or kpi.get("manifest") or {}).get("code_revision"),
    }


def main():
    pipe = jload(OUT / "identity_replay" / "pipeline.json")
    rep = jload(OUT / "identity_replay" / "report.json")
    sig = jload(ROOT / "docs" / "data" / "citylife_signals.json")
    old_as_flown = {t: {"within_30m": jload(OUT / t / "metrics.json")["frac_within_30m"],
                        "estimate_on_car_as_flown": pipe[t]["old_as_flown"]["on_frac_of_served"]}
                    for t in RED_OLD}
    P = pipe["_pooled"]
    data = {
        "generated_by": "tools/deck/build_progress_0930_data.py",
        "red_new": [flight(t) for t in RED_NEW],
        "red_old": old_as_flown,
        "other": {t: flight(t) for t in OTHER if (OUT / t / "metrics.json").exists()},
        "replay": {
            "flights": len([k for k in pipe if not k.startswith("_")]),
            "ticks": P["new"]["n_ticks"],
            "as_flown_on_car_of_served": P["old_as_flown"]["on_frac_of_served"],
            "as_flown_false_post_gap": P["old_as_flown"]["false_post_gap_accepts"],
            "as_flown_post_gap": P["old_as_flown"]["post_gap_accepts"],
            "new_on_car_of_served": P["new"]["on_frac_of_served"],
            "new_seeds": P["new"]["seeds"], "new_seeds_start": P["new"]["seeds_start"],
            "new_seeds_reacq": P["new"]["seeds_reacq"], "new_false_seeds": P["new"]["false_seeds"],
            "new_served_frac": P["new"]["served_frac"],
            "as_flown_served_frac": P["old_as_flown"]["served_frac"],
        },
        "identity_gate": {
            "thresholds_hash": rep["thresholds_hash"],
            "n_wrong_boxes": rep["pooled"]["n_off"], "n_true_boxes": rep["pooled"]["n_on"],
            "in_sample": {k: rep["pooled"][k] for k in ("off_notok_rate", "on_notok_rate", "on_hard_rate")},
            "held_out": {k: rep["loo"]["pooled_held_out"][k]
                         for k in ("off_notok_rate", "on_notok_rate", "on_hard_rate")},
            "gate": {"off_notok_min": 0.70, "on_notok_max": 0.15, "on_hard_max": 0.03},
        },
        "signals": {"heads_surveyed": len(sig["heads"]), "by_kind": sig["counts"],
                    "junctions": sig["n_junctions"]},
        "simulate": SIMULATE,
    }
    out = ROOT / "docs" / "data" / "progress_0930.json"
    out.write_text(json.dumps(data, indent=1, default=str) + "\n", encoding="utf-8", newline="\n")
    print("wrote", out.relative_to(ROOT))
    for f in data["red_new"]:
        print(f["tag"], f["within_30m"], f["estimate_on_subject"], f["sep_max_m"], f["lapses"], f["reacquired"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
