"""
WP1-14 / WP4-11: what binding hot-applied geometry to the WGS84 frame changed.

    python tools/wgs84_bind_compare.py diff BEFORE/sweep.json AFTER/sweep.json \
        --before-runs BEFORE/episodes --after-runs AFTER/episodes --out OUT.json
    python tools/wgs84_bind_compare.py tick-cost --out OUT.json

Since 2026-11-17 the Shield enforces the lazy projection of the stored WGS84
geometry for every rule an event writes into a policy with a frame
(Shield._in_frame -> models.rule_in_wgs84, which binds the rule to the
policy's frame). Before that, on integrate/1117, the live Shield enforced the
metres it was given while the stored snapshot held lat/lon, up to 0.6 um
apart under one hash, so a replay from the stored policy_g<N>.json was not
bit-for-bit the flight. Binding moves computed (off-grid) rings by up to
0.6 um, which the Shield reacts to; this tool documents what that moved.

`diff` compares two sweeps of the same profile (experiments/
sweep_scenarios.py: smoke, the library, nightly) field by field: every KPI
field of every episode, the episode status and failed gates, and the sweep's
aggregates (only whole wall-clock, path and revision keys left out). Its
headline includes mission success (grant KPI 1, Shield-on and per family),
and each differing episode carries a `flown` summary: FSM edge sequence or
only edge times moved, mission outcome, status, hold points, end position. For each episode that differs it also aligns the
two shield_audit.jsonl logs by monitor tick and names the first tick whose
emitted action or violation set differs, the largest emitted-action
difference, and every tick where the set of violated rules changed.

`tick-cost` times Shield.filter on a hot-applied, rotated (off-grid) zone:
metre writes (`wgs84_writes = False`, the master behaviour: no projection)
against the default (WGS84 written, bound, metres computed lazily from
lat/lon). Interleaved rounds, median and p95 per mode, plus the commit
(hot-apply) time, where the projection is actually computed, and how many
times the ticks read a point's lazily projected view at all
(projection.local_metres' cache counters).
"""
from __future__ import annotations

import argparse
import copy
import gzip
import json
import math
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

# Sweep fields that are wall clock, paths or revision, not flown behaviour.
# Matched on the WHOLE last key (and two wall-clock suffixes), never on any
# ending: until 2026-11-17 evening a bare endswith() on 'ts', 'dir', 'path'
# and 'wall' also dropped kpi.hold_points, kpi.harness_events,
# kpi.collision_contacts / _objects, the *_distinct_flights fail-safe figures,
# cell_seed_slots and the wind-gusts family row (review of wf17b-bind).
IGNORED_TOP = {"_runtime_s", "_command", "_runner", "_manifest_common", "results"}
IGNORED_EXACT = frozenset({"_runtime_s", "runtime", "runtime_s", "seconds", "code_revision",
                           "ts", "path", "dir", "bundle", "issued_at", "wall"})
IGNORED_SUFFIX = ("_s_wall", "_wall_s")
UM = 1e-6


def _ignored(path: str) -> bool:
    last = path.rsplit(".", 1)[-1].split("[", 1)[0]
    return last in IGNORED_EXACT or last.endswith(IGNORED_SUFFIX)


def flatten(node, prefix: str = "") -> dict:
    out: dict = {}
    if isinstance(node, dict):
        for k, v in node.items():
            out.update(flatten(v, f"{prefix}.{k}" if prefix else str(k)))
    elif isinstance(node, list) and node and all(isinstance(v, (dict, list)) for v in node):
        for i, v in enumerate(node):
            out.update(flatten(v, f"{prefix}[{i}]"))
    else:
        out[prefix] = node
    return out


def _kind(a, b) -> str:
    if isinstance(a, bool) or isinstance(b, bool) or a is None or b is None:
        return "changed"
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        d = abs(float(a) - float(b))
        if isinstance(a, int) and isinstance(b, int):
            return "count"
        return "um_level" if d <= UM else "magnitude"
    return "changed"


def diff_dicts(a: dict, b: dict) -> list[dict]:
    fa, fb = flatten(a), flatten(b)
    out = []
    for k in sorted(set(fa) | set(fb)):
        if _ignored(k):
            continue
        va, vb = fa.get(k, "<absent>"), fb.get(k, "<absent>")
        if va != vb:
            row = {"field": k, "before": va, "after": vb, "kind": _kind(va, vb)}
            if isinstance(va, (int, float)) and isinstance(vb, (int, float)) \
                    and not isinstance(va, bool):
                row["abs_diff"] = abs(float(vb) - float(va))
            out.append(row)
    return out


def _episode_dir(runs: Path | None, result: dict) -> Path | None:
    """The episode folder of `result` under `runs`: matched on the folder
    name after its timestamp (scenario, arm and seed), since the two sweeps
    ran at different times."""
    if runs is None or not runs.is_dir() or not result.get("bundle"):
        return None
    key = Path(str(result["bundle"])).name.split("--", 1)[-1]
    hits = [d for d in runs.iterdir() if d.is_dir() and d.name.split("--", 1)[-1] == key]
    return hits[0] if len(hits) == 1 else None


def _audit(d: Path) -> dict[int, dict]:
    p = d / "shield_audit.jsonl"
    if not p.is_file():
        p = d / "shield_audit.jsonl.gz"
        if not p.is_file():
            return {}
        lines = gzip.open(p, "rt", encoding="utf-8").read().splitlines()
    else:
        lines = p.read_text(encoding="utf-8").splitlines()
    out = {}
    for ln in lines:
        if ln.strip():
            r = json.loads(ln)
            out[int(r.get("monitor_tick", r.get("tick")))] = r
    return out


def _act(r: dict) -> dict:
    return r.get("emitted_action_world") or r.get("emitted_action") or {}


def _viol(r: dict) -> list:
    return sorted((v.get("rule_id"), v.get("category")) for v in r.get("violations") or [])


def _vdetail(r: dict) -> list:
    return [{"rule_id": v.get("rule_id"), "detail": v.get("detail"),
             "predicted_at_s": v.get("predicted_at_s")} for v in r.get("violations") or []]


def audit_diff(d0: Path, d1: Path) -> dict:
    """Align two shield_audit.jsonl logs by monitor tick. An emitted-action
    difference is `um_level` when every component is within 1e-6 m/s, and
    `mirrored` when the two actions are each other's negation (a slide
    direction picked the other way at a tie)."""
    a, b = _audit(d0), _audit(d1)
    first = None
    small, max_small = 0, 0.0
    large = []
    viol_changes = []
    one_side = []
    for t in sorted(set(a) | set(b)):
        ra, rb = a.get(t), b.get(t)
        if ra is None or rb is None:
            r = ra or rb
            one_side.append({"tick": t, "audited_in": "before" if rb is None else "after",
                             "fsm": [r.get("fsm_state_before"), r.get("fsm_state_after")],
                             "violations": _vdetail(r)})
            first = t if first is None else first
            continue
        ea, eb = _act(ra), _act(rb)
        keys = sorted(set(ea) | set(eb))
        dd = max((abs(float(ea.get(k, 0.0)) - float(eb.get(k, 0.0))) for k in keys), default=0.0)
        if dd and dd <= UM:
            small += 1
            max_small = max(max_small, dd)
        elif dd:
            mir = max(abs(float(ea.get(k, 0.0)) + float(eb.get(k, 0.0))) for k in keys)
            large.append({"tick": t, "abs_diff_mps": dd, "mirrored": mir <= UM,
                          "before": ea, "after": eb})
        va, vb = _viol(ra), _viol(rb)
        if va != vb:
            viol_changes.append({"tick": t, "before": _vdetail(ra), "after": _vdetail(rb)})
        if (dd or va != vb) and first is None:
            first = t
    return {"audited_ticks_before": len(a), "audited_ticks_after": len(b),
            "first_differing_tick": first,
            "ticks_emitted_differs_um_level": small,
            "emitted_um_level_max_abs_diff_mps": max_small,
            "ticks_emitted_differs_large": len(large),
            "ticks_emitted_differs_large_all_mirrored": all(x["mirrored"] for x in large),
            "large": large, "violation_set_changes": viol_changes,
            "ticks_audited_in_one_only": one_side}


def _mission(row: dict | None) -> dict:
    row = row or {}
    return {k: row.get(k) for k in ("mission_success_rate", "mission_successes",
                                    "episodes_mission_scored", "mission_goal", "outcomes")}


def headline(sweep: dict) -> dict:
    """The figures the smoke profile is read for: what tools/ci_local.py
    prints, the status counts, P0 escapes summed over episodes, the
    true-state P0 figures, and the grant's KPI 1, mission success
    (grant-overview.md; stress-testing.md's table): the Shield-on rate, its
    goal and outcome counts, per family too, and the outcome labels' counts.
    Until 2026-11-17 evening mission success was left out, so a library diff
    whose success rate fell read `headline_identical: true`."""
    fl = sweep.get("_failsafe_labels") or {}
    ts = sweep.get("_true_state") or {}
    ro = sweep.get("_rollup") or {}
    res = sweep.get("results") or []
    status: dict = {}
    for r in res:
        status[r.get("status")] = status.get(r.get("status"), 0) + 1
    return {"episodes": len(res), "status": dict(sorted(status.items(), key=str)),
            "mission_shield_on": _mission(ro.get("_all_shield_on")),
            "mission_by_family": {k: _mission(v) for k, v in sorted(ro.items())
                                  if not k.startswith("_") and isinstance(v, dict)},
            "outcome_label_counts": (sweep.get("_outcome_labels") or {}).get("outcomes"),
            "p0_escapes_sum": sum(int((r.get("kpi") or {}).get("p0_escapes") or 0) for r in res),
            "fail_safe_correctness": fl.get("fail_safe_correctness"),
            "failsafe_correct_of_labelled": [fl.get("correct"), fl.get("labelled")],
            "false_triggers": fl.get("false_triggers"),
            "missed_triggers": fl.get("missed_triggers"),
            "null_never_trigger": fl.get("null_never_trigger"),
            "null_always_trigger": fl.get("null_always_trigger"),
            "true_p0_flown_ticks": ts.get("true_p0_flown_ticks"),
            "episodes_entering_polygon": ts.get("episodes_entering_polygon"),
            "episodes_with_breach": ts.get("episodes_with_breach"),
            "autopilot_p0_flown_ticks": ts.get("autopilot_p0_flown_ticks")}


def _edges(kpi: dict) -> list:
    fsm = (kpi or {}).get("fsm") or {}
    return [(t.get("edge"), t.get("from"), t.get("to"), t.get("t_s"))
            for t in fsm.get("transitions") or []]


def flown_change(x: dict, y: dict) -> dict:
    """What a reader of the episode needs first: did its FSM edge SEQUENCE
    (edge, from, to) change, or only the times of the same edges; its
    mission outcome, its status, its hold points; and where did it end."""
    kx, ky = x.get("kpi") or {}, y.get("kpi") or {}
    fx, fy = (kx.get("final") or {}), (ky.get("final") or {})
    end = None
    if all(isinstance(d.get(k), (int, float)) for d in (fx, fy) for k in ("x", "y")):
        end = math.hypot(fy["x"] - fx["x"], fy["y"] - fx["y"])
    return {"status_changed": x.get("status") != y.get("status"),
            "fsm_edges_changed": [e[:3] for e in _edges(kx)] != [e[:3] for e in _edges(ky)],
            "fsm_edge_times_moved": _edges(kx) != _edges(ky),
            "fsm_edges_before": _edges(kx), "fsm_edges_after": _edges(ky),
            "mission_outcome_changed": kx.get("mission_outcome") != ky.get("mission_outcome"),
            "mission_outcome": [kx.get("mission_outcome"), ky.get("mission_outcome")],
            "hold_points_changed": kx.get("hold_points") != ky.get("hold_points"),
            "harness_events": [kx.get("harness_events"), ky.get("harness_events")],
            "final_position_moved_m": end}


def diff_sweeps(before: dict, after: dict, before_runs: Path | None,
                after_runs: Path | None) -> dict:
    ra = {r["id"]: r for r in before["results"]}
    rb = {r["id"]: r for r in after["results"]}
    episodes = []
    for eid in sorted(set(ra) | set(rb)):
        x, y = ra.get(eid), rb.get(eid)
        if x is None or y is None:
            episodes.append({"id": eid, "only_in": "after" if x is None else "before"})
            continue
        fields = diff_dicts({"status": x.get("status"), "gates_failed": x.get("gates_failed"),
                             "kpi": x.get("kpi")},
                            {"status": y.get("status"), "gates_failed": y.get("gates_failed"),
                             "kpi": y.get("kpi")})
        if not fields:
            continue
        row = {"id": eid, "scenario_id": x.get("scenario_id"), "seed": x.get("seed"),
               "fields": fields, "flown": flown_change(x, y)}
        d0 = _episode_dir(before_runs, x)
        d1 = _episode_dir(after_runs, y)
        if d0 and d1:
            row["audit"] = audit_diff(d0, d1)
        episodes.append(row)
    top = diff_dicts({k: v for k, v in before.items() if k not in IGNORED_TOP},
                     {k: v for k, v in after.items() if k not in IGNORED_TOP})
    hb, ha = headline(before), headline(after)
    fsm_or_outcome = [e["id"] for e in episodes
                      if e.get("flown") and (e["flown"]["fsm_edges_changed"]
                                             or e["flown"]["mission_outcome_changed"]
                                             or e["flown"]["status_changed"]
                                             or e["flown"]["hold_points_changed"])]
    return {"headline_before": hb, "headline_after": ha, "headline_identical": hb == ha,
            "headline_fields_differing": diff_dicts(hb, ha),
            "episodes_fsm_outcome_status_or_hold_changed": fsm_or_outcome,
            "episodes_fsm_edge_times_moved": [e["id"] for e in episodes if e.get("flown")
                                             and e["flown"]["fsm_edge_times_moved"]],
            "episodes_compared": len(set(ra) & set(rb)),
            "episodes_differing": len(episodes), "episodes": episodes,
            "aggregates_differing": top}


# ------------------------------------------------------------------ tick cost

def tick_cost(ticks: int = 400, rounds: int = 7) -> dict:
    from guardrail import api
    from guardrail.models import Action4D, DynamicNFZ, State, load_policy, wgs84_from_metres
    from guardrail.shield import Shield
    sitl = ROOT / "policies" / "sim_demo_policy.yaml"
    rule = {"id": "nfz-bench", "scope": "mission", "constraint_type": "hard",
            "priority": "P0", "violation_action": "project_fix",
            "vertices": [{"x": 23.1234567, "y": 6}, {"x": 31, "y": 6.7654321},
                         {"x": 31, "y": 14}, {"x": 23, "y": 14}],
            "altitude_floor_m": 0, "altitude_ceiling_m": 100, "margin_m": 1.0}

    def build(metres: bool):
        sh = Shield(load_policy(sitl))
        if metres:
            sh.wgs84_writes = False
        t0 = time.perf_counter_ns()
        if metres:
            # The metre arm (comparison switch): the Shield stores the metres
            # as given. The REST entry takes lat/lon only since 2026-11-26
            # (WP1-14), so the rule goes to the Shield directly.
            sh.spawn_nfz(DynamicNFZ.model_validate({**copy.deepcopy(rule),
                                                    "type": "dynamic_nfz"}))
        else:
            # The WGS84 arm: converted tool side, as every metre caller does
            # since 2026-11-26, inside the timed commit, where the ingest
            # converted it until then (the same projection.metres_to_wgs84).
            api.apply_dynamic_nfz(sh, wgs84_from_metres(copy.deepcopy(rule),
                                                        sh.policy.frame_origin))
        api.apply_nfz_edit(sh, rule["id"], {"op": "rotate", "angle_deg": 17})
        return sh, (time.perf_counter_ns() - t0) / 1e3

    def fly(sh) -> list[float]:
        out = []
        s = State(x=0.0, y=10.0, up=20.0)
        for k in range(ticks):
            a = Action4D(vx=4.0 * math.cos(k * 0.01), vy=0.3, vz_up=0.0, yaw_rate=0.0)
            t0 = time.perf_counter_ns()
            d = sh.filter(s, a, t=k * 0.1)
            out.append((time.perf_counter_ns() - t0) / 1e3)
            e = d.emitted
            s = State(x=s.x + e.vx * 0.1, y=s.y + e.vy * 0.1, up=20.0)
            if k % 100 == 99:
                s = State(x=0.0, y=10.0, up=20.0)
        return out

    from guardrail.projection import local_metres
    per = {"metres": [], "wgs84_bound": []}
    commit = {"metres": [], "wgs84_bound": []}
    proj = {"metres": [0, 0], "wgs84_bound": [0, 0]}    # view reads, cache misses
    fly(build(True)[0])                                   # warm-up
    for r in range(rounds):
        for mode in (("metres", "wgs84_bound") if r % 2 == 0 else ("wgs84_bound", "metres")):
            sh, c_us = build(mode == "metres")
            commit[mode].append(c_us)
            c0 = local_metres.cache_info()
            per[mode].extend(fly(sh))
            c1 = local_metres.cache_info()
            proj[mode][0] += (c1.hits + c1.misses) - (c0.hits + c0.misses)
            proj[mode][1] += c1.misses - c0.misses

    def stats(xs):
        xs = sorted(xs)
        return {"n": len(xs), "median_us": round(statistics.median(xs), 1),
                "p95_us": round(xs[int(0.95 * (len(xs) - 1))], 1),
                "mean_us": round(statistics.fmean(xs), 1)}
    out = {"ticks_per_round": ticks, "rounds": rounds,
           "filter": {m: stats(v) for m, v in per.items()},
           "hot_apply_spawn_plus_rotate": {m: stats(v) for m, v in commit.items()},
           "projection_reads_during_ticks": {m: {"view_reads": v[0], "computed": v[1]}
                                             for m, v in proj.items()}}
    fm, fb = out["filter"]["metres"]["median_us"], out["filter"]["wgs84_bound"]["median_us"]
    out["filter_median_ratio_bound_over_metres"] = round(fb / fm, 3)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("diff")
    d.add_argument("before")
    d.add_argument("after")
    d.add_argument("--before-runs")
    d.add_argument("--after-runs")
    d.add_argument("--out", required=True)
    t = sub.add_parser("tick-cost")
    t.add_argument("--ticks", type=int, default=400)
    t.add_argument("--rounds", type=int, default=7)
    t.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    if a.cmd == "diff":
        res = diff_sweeps(json.loads(Path(a.before).read_text(encoding="utf-8")),
                          json.loads(Path(a.after).read_text(encoding="utf-8")),
                          Path(a.before_runs) if a.before_runs else None,
                          Path(a.after_runs) if a.after_runs else None)
    else:
        res = tick_cost(a.ticks, a.rounds)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(res, indent=1) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps(res if a.cmd != "diff" else
                     {k: v for k, v in res.items() if k != "episodes"}, indent=1)[:4000])
    return 0


if __name__ == "__main__":
    sys.exit(main())
