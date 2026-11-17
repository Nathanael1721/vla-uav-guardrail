"""Before / after figures for PathRepair joining the default repair chain
(WP3-06, 2026-11-17; docs/data/pathrepair_default_20261117/README.md).

    python tools/pathrepair_default_summary.py \\
        --pair smoke BEFORE/smoke/sweep.json AFTER/smoke/sweep.json smoke_diff.json \\
        --pair library BEFORE/lib/sweep.json AFTER/lib/sweep.json library_diff.json \\
        --out summary.json

Each pair is two `experiments/sweep_scenarios.py` outputs (before: the chain
without PathRepair; after: this branch) and the episode diff
`tools/wgs84_bind_compare.py diff` made of them. The summary reads every
figure from the sweeps' own Shield-on rollup (`_rollup._all_shield_on`) - the
P0 escapes, the fail-safe trigger correctness on the labelled episodes, the
repair success rate, the mean repair magnitude, mission success, PathRepair's
own operator-class figures - and lists every differing episode with the
fields that say what it flew (FSM edges, outcome, PathRepair ticks,
true-state P0 ticks, closest true distance to a fence polygon). It computes
nothing the sweeps did not measure.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

ROLLUP_FIELDS = ("episodes", "p0_escapes", "p0_escape_rate", "repair_success_rate",
                 "repair_attempt_ticks", "repair_success_ticks", "mean_repair_magnitude_mps",
                 "mission_success_rate", "mission_successes", "episodes_mission_scored")
EPISODE_FIELDS = ("kpi.repair_magnitude_by_operator_class.PathRepair.ticks",
                  "kpi.true_p0_flown_ticks", "kpi.min_true_dist_to_fence_m",
                  "kpi.repair_success_rate", "kpi.mean_repair_magnitude_mps",
                  "kpi.p0_violation_ticks", "kpi.polygon_entries")


def _load(p):
    return json.loads(Path(p).read_text(encoding="utf-8"))


def figures(sweep: dict) -> dict:
    r = sweep["_rollup"]["_all_shield_on"]
    out = {k: r.get(k) for k in ROLLUP_FIELDS}
    fs = r.get("failsafe_labelled_episodes") or {}
    out["failsafe_trigger_correctness"] = fs.get("failsafe_trigger_correctness")
    out["failsafe_scored"] = fs.get("scored")
    out["false_triggers"] = fs.get("false_triggers")
    out["missed_triggers"] = fs.get("missed_triggers")
    out["path_repair_class"] = (r.get("mean_repair_magnitude_by_operator_class") or {}
                                ).get("PathRepair")
    out["repair_outcomes"] = r.get("repair_outcomes")
    return out


def episodes(diff: dict) -> list:
    rows = []
    for e in diff.get("episodes") or []:
        f = {x["field"]: [x["before"], x["after"]] for x in e["fields"]}
        fl = e.get("flown") or {}
        rows.append({
            "id": e["id"],
            "mission_outcome": fl.get("mission_outcome"),
            "mission_outcome_changed": fl.get("mission_outcome_changed"),
            "status_changed": fl.get("status_changed"),
            "fsm_edges_changed": fl.get("fsm_edges_changed"),
            "fsm_edges_before": fl.get("fsm_edges_before"),
            "fsm_edges_after": fl.get("fsm_edges_after"),
            "final_position_moved_m": fl.get("final_position_moved_m"),
            **{k.removeprefix("kpi."): f[k] for k in EPISODE_FIELDS if k in f},
        })
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--pair", nargs=4, action="append", required=True,
                    metavar=("NAME", "BEFORE", "AFTER", "DIFF"))
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    out = {"_what": __doc__.split("\n")[0], "runs": {}}
    for name, before, after, diff in a.pair:
        b, f, d = _load(before), _load(after), _load(diff)
        out["runs"][name] = {
            "before": figures(b), "after": figures(f),
            "headline_identical": d.get("headline_identical"),
            "headline_fields_differing": d.get("headline_fields_differing"),
            "episodes_compared": d.get("episodes_compared"),
            "episodes_differing": d.get("episodes_differing"),
            "episodes_fsm_outcome_status_or_hold_changed":
                d.get("episodes_fsm_outcome_status_or_hold_changed"),
            "episodes": episodes(d),
        }
        fb, fa = out["runs"][name]["before"], out["runs"][name]["after"]
        print(f"{name}: P0 escapes {fb['p0_escapes']} -> {fa['p0_escapes']}; fail-safe "
              f"{fb['failsafe_trigger_correctness']} -> {fa['failsafe_trigger_correctness']}; "
              f"repair success {fb['repair_success_rate']} -> {fa['repair_success_rate']}; "
              f"mean |emitted-raw| {fb['mean_repair_magnitude_mps']} -> "
              f"{fa['mean_repair_magnitude_mps']} m/s; mission {fb['mission_success_rate']} -> "
              f"{fa['mission_success_rate']}; PathRepair ticks "
              f"{(fb['path_repair_class'] or {}).get('ticks')} -> "
              f"{(fa['path_repair_class'] or {}).get('ticks')}; "
              f"{d.get('episodes_differing')} of {d.get('episodes_compared')} episodes differ")
    Path(a.out).write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
