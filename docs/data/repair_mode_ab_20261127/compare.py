"""Compare the two A/B sweep files; print JSON with per-arm figures and diffs.

    python docs/data/repair_mode_ab_20261127/compare.py \\
        docs/data/repair_mode_ab_20261127/sweep_chain.json \\
        docs/data/repair_mode_ab_20261127/sweep_first_feasible.json \\
        > docs/data/repair_mode_ab_20261127/comparison.json

Per arm: the sweep's own _counts, _failsafe_labels and _true_state, and sums
over the per-episode kpi tables (P0 escapes and P0 violation ticks over the
Shield-on episodes, mission successes, repair counts). Per episode (id, seed):
every one whose status, mission success, outcome, fail-safe match, P0
escapes, repair count, true-state P0 ticks, max time to safe, polygon entries
or breaches differ between the arms."""
import json
import sys

a = json.load(open(sys.argv[1], encoding="utf-8"))
b = json.load(open(sys.argv[2], encoding="utf-8"))


def arm(d):
    res = d["results"]
    on = [r for r in res if r.get("arm") == "on" and "kpi" in r]
    scored = [r for r in res if "kpi" in r]
    k = lambda r, f: r["kpi"].get(f)
    ms = [r for r in scored if k(r, "mission_success") is not None]
    fl = d["_failsafe_labels"]
    return {
        "revision": d["_manifest_common"]["code_revision"],
        "repair_mode": d["_scope"].get("repair_mode"),
        "counts": d["_counts"],
        "episodes": len(res),
        "shield_on_episodes": len(on),
        "p0_escapes": sum(k(r, "p0_escapes") or 0 for r in on),
        "p0_violation_ticks": sum(k(r, "p0_violation_ticks") or 0 for r in on),
        "episodes_with_p0_escape": sum(1 for r in on if (k(r, "p0_escapes") or 0) > 0),
        "fail_safe_correct": fl["correct"], "fail_safe_scored": fl["labelled"],
        "fail_safe_correctness": fl["fail_safe_correctness"],
        "false_triggers": fl["false_triggers"], "missed_triggers": fl["missed_triggers"],
        "mission_successes": sum(1 for r in ms if k(r, "mission_success")),
        "mission_scored": len(ms),
        "repair_count_total_shield_on": sum(k(r, "repair_count") or 0 for r in on),
        "repaired_ticks_total_shield_on": sum(k(r, "repaired_ticks") or 0 for r in on),
        "true_p0_flown_ticks": d["_true_state"]["true_p0_flown_ticks"],
        "outcomes": d["_outcome_labels"]["outcomes"],
        "runtime_s": d["_runtime_s"],
    }


FIELDS = ("status", "mission_success", "outcome", "failsafe_matches_label",
          "p0_escapes", "repair_count", "true_p0_flown_ticks", "max_time_to_safe_s",
          "polygon_entries", "breaches")


def row(r):
    out = {"status": r.get("status")}
    for f in FIELDS[1:]:
        out[f] = r.get("kpi", {}).get(f)
    return out


ia = {(r["id"], r["seed"]): r for r in a["results"]}
ib = {(r["id"], r["seed"]): r for r in b["results"]}
assert set(ia) == set(ib), "episode lists differ"
diffs = []
for key in sorted(ia):
    ra, rb = row(ia[key]), row(ib[key])
    changed = {f: [ra[f], rb[f]] for f in FIELDS if ra[f] != rb[f]}
    if changed:
        diffs.append({"id": key[0], "seed": key[1], "scenario_id": ia[key]["scenario_id"],
                      "changed": changed})
nonrep = [d for d in diffs if set(d["changed"]) - {"repair_count"}]
print(json.dumps({"chain": arm(a), "first_feasible": arm(b),
                  "episodes_differing": len(diffs),
                  "episodes_differing_beyond_repair_count": len(nonrep),
                  "scenarios_differing": sorted({d["scenario_id"] for d in diffs}),
                  "diffs": diffs}, indent=1))
