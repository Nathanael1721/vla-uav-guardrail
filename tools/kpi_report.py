"""Roll many episodes up into the grant's KPI report, as Markdown and JSON.

    python tools/kpi_report.py                                  # print it
    python tools/kpi_report.py --out <stem>                     # <stem>.md + .json
    python tools/kpi_report.py --runs 'demo/out/ros2_*' --no-sweep

WHAT THE GRANT ASKS FOR

Stress Testing p6, "KPI report": "The auto-report has two sections: 1.
Per-scenario-family stats table. One row per (scenario template, parameter
cell) tuple. Columns: episode count, P0 escape rate, fail-safe correctness,
mission success rate, mean repair count, mean repair magnitude. 2. Top-K
failure cases. K=10 by default. Each row links to its episode bundle ...
columns: scenario_id, parameters, failure category, raw vs repaired action.
Output formats: Markdown ... and JSON". Until 2026-10-06 nothing in this
repository added KPIs up across episodes: `guardrail/kpi.py` scored one flight
at a time and `tools/build_mideval_report.py` is a hand-picked four-run table
(audit card WP4-07).

This tool reads what is already on disk:

  * flown run folders (`flight_log.jsonl`, `metrics.json`, `manifest.json`),
    re-scored from the log with the SAME `guardrail.kpi.compute` the flights
    use, so the report and the artefacts cannot disagree about what an escape
    is;
  * the stress harness's episode bundles (`demo/out/stress_*/episode-*`:
    `manifest.json`, `kpi.json`, `harness_events.jsonl`, no flight log unless
    `--keep-logs`). Without per-tick rows they cannot be re-scored, so their
    stored table is used and every such episode says so (`kind:
    stored-table`);
  * the headless sweep's aggregated JSON (`docs/data/scenario_sweep.json`).

WHAT IT REFUSES TO BLUR

  * KPI-grade or not. The grant allows contractual numbers only from runs that
    pass `guardrail.manifest.is_kpi_grade`; every other episode is functional
    evidence. They are tabulated SEPARATELY and never pooled, and every
    episode carries the reasons it was refused.
  * Not measurable vs zero. A rate whose denominator is empty is printed
    `n/m` or `none attempted`, never 0 - see the `silence-reads-as-success`
    memory: four of this project's zeros turned out to be rules that never ran.
  * The null beside every score: what a passthrough Shield, an always-brake
    Shield, or a vehicle that never moves would have scored on the same
    episodes. A success rate a hover also achieves is not evidence.
  * Different parameter cells. One family row is one (template, cell) - the
    grant's key - plus the Shield arm, so two scenarios of one template, or a
    static run and a hot-applied one, never share a row.
  * The stored figure vs the recomputed one. Where they differ the report
    says so (e.g. `ros2_ped_off`'s time to safe, corrected 2026-10-06), and
    `tools/rescore_kpis.py` is what writes corrections back.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import shlex
import subprocess
import sys
from datetime import date
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from guardrail import kpi as K                                     # noqa: E402
from guardrail.manifest import code_revision, is_kpi_grade         # noqa: E402
import rescore_kpis as R                                           # noqa: E402

DEFAULT_RUNS = ["demo/out/*", "sitl/out/*", "demo/out/stress_*/episode-*"]
DEFAULT_SWEEP = "docs/data/scenario_sweep.json"
LIBRARY = ROOT / "experiments" / "scenarios.yaml"

RAIL = {
    "canonical-hil": "ros2-mavros-sitl",      # the pre-2026-10-06 label of `dev`
    "dev": "ros2-mavros-sitl",
    "ardupilot-sitl-pymavlink": "sitl-pymavlink",
    "projectairsim-single-host": "airsim",
    "headless-kinematic": "headless-kinematic",
}

# Fields whose value depends on rule PRIORITIES. compute() treats an unknown
# rule as P0, so for a run whose policy cannot be resolved a recomputation
# would count every P1 tick as a P0 one - ros2_shield_on_dynamic read 309 P0
# ticks against the 217 it scored in flight. For such a run these are taken
# from the stored kpi.json (computed in flight against the real policy), or
# left None; never recomputed against {}. Same list in spirit as
# tools/rescore_kpis.NEEDS_PRIORITIES, which refuses to WRITE them.
PRIORITY_FIELDS = (
    "p0_violation_escape_rate", "p0_escapes", "p0_violation_ticks",
    "p0_ticks_not_measurable", "failsafe_trigger_correctness",
    "failsafe_expected_ticks", "failsafe_correct_ticks",
    "failsafe_permitted_ticks", "violations_by_risk_level", "repair_outcomes",
    "outcome", "mission_success", "mission_fail_reasons", "unsafe_p0_ticks",
    "mission_started_unsafe",
)

# Headline fields compared between the stored kpi.json and the recomputation.
_COMPARE = ("p0_escapes", "p0_violation_ticks", "outcome", "mission_success",
            "mean_time_to_safe_s", "time_to_safe_episodes",
            "time_to_safe_censored", "time_to_safe_not_measurable")


def json_safe(o, path: str = "", found: list | None = None):
    """Replace non-finite floats with the strings "NaN" / "Infinity" /
    "-Infinity", recording where.

    The scenario library itself contains one on purpose (`nonfinite-action`
    flies `vx: .nan`), and a raw action in a log can carry one. Strict JSON has
    no token for either, and `allow_nan=False` is kept on the writer because a
    bare NaN once made an artefact unreadable to everything but Python. A
    string keeps the fact visible; dropping it to null would not.
    """
    if isinstance(o, float) and not math.isfinite(o):
        if found is not None:
            found.append(path)
        return "NaN" if o != o else ("Infinity" if o > 0 else "-Infinity")
    if isinstance(o, dict):
        return {k: json_safe(v, f"{path}/{k}", found) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [json_safe(v, f"{path}/{i}", found) for i, v in enumerate(o)]
    return o


def _rel(p: Path) -> str:
    try:
        return p.resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return p.as_posix()


def _read_json(p: Path) -> dict:
    try:
        return json.loads(p.read_text(encoding="utf-8")) if p.is_file() else {}
    except (OSError, json.JSONDecodeError):
        return {}


def _arm(v) -> str | None:
    if v is True or (isinstance(v, str) and v.lower() == "on"):
        return "on"
    if v is False or (isinstance(v, str) and v.lower() == "off"):
        return "off"
    return None


def cell_name(scenario_id: str, params: dict | None) -> str:
    """The grant's parameter cell, spelled the way the stress harness names its
    episode folders: `altitude-recovery-depth@start_up=0.5`."""
    if not params:
        return str(scenario_id)
    return f"{scenario_id}@" + ",".join(f"{k}={params[k]}" for k in sorted(params))


def shield_off_table(k: dict) -> dict:
    """A Shield-off control arm's table, with the repairs it never flew removed.

    The headless sweep logs the repairs the Shield WOULD have made and flies
    the raw action (sweep_scenarios.py: `flown = d.emitted if on else raw`).
    A stored table computed before kpi.py knew that scores the control arm as
    a repair success of 0.0 over N "attempts" and a repair magnitude of 0.000
    over N "repaired" ticks. None of those repairs was applied, and "Average
    repair count" counts applied repairs. Idempotent.
    """
    k = dict(k)
    if (k.get("repair_count") or 0) and "repair_count_counterfactual" not in k:
        k["repair_count_counterfactual"] = k["repair_count"]
        k["repair_count"] = 0
    k.update({"repair_success_rate": None, "repair_success_status": "shield_off",
              "repair_attempt_ticks": 0, "repair_success_ticks": 0,
              "repair_unmeasured_ticks": 0, "repair_outcomes": None,
              "repair_success_basis": None, "repair_theta_unchecked_ticks": 0,
              "mean_repair_magnitude_mps": None, "max_repair_magnitude_mps": None,
              "repaired_ticks": 0})
    return k


# --------------------------------------------------------------------------- #
# one flown episode
# --------------------------------------------------------------------------- #

def load_run(run: Path, by_hash: dict, by_id: dict) -> dict:
    """Score one run folder. Returns an episode dict, or {"skipped": reason}."""
    log = run / "flight_log.jsonl"
    if not log.is_file():
        if (run / "kpi.json").is_file():
            return load_stored(run)
        return {"id": run.name, "source": _rel(run),
                "skipped": "no flight_log.jsonl and no kpi.json"}
    rows = [json.loads(x) for x in log.read_text(encoding="utf-8").splitlines()
            if x.strip()]
    if not rows:
        return {"id": run.name, "source": _rel(run), "skipped": "empty flight_log.jsonl"}

    metrics = _read_json(run / "metrics.json")
    manifest = _read_json(run / "manifest.json")
    stored = _read_json(run / "kpi.json")
    if isinstance(stored.get("kpi"), dict):          # a harness bundle with --keep-logs
        stored = stored["kpi"]
    pol, how, note = R.resolve_policy(run, manifest, by_hash, by_id)

    recon = None
    logged_unsafe = any("unsafe" in r for r in rows)
    basis = "logged in flight" if logged_unsafe else "n/m: policy not resolvable"
    if (pol is not None and not logged_unsafe
            and any(r.get("x") is not None for r in rows)):
        info = R.reconstruct_unsafe(rows, pol, metrics)
        cross = R.dwell_crosscheck(info, metrics)
        recon = {"subject_source": info["subject_source"], "dwell": cross}
        basis = time_to_safe_basis(info["subject_source"], pol)
        if how == "policy_id" and cross["agree"] is False:
            # The id-matched file does not describe the rules that flew; a
            # time to safe computed against it would be a guess.
            for r in rows:
                r.pop("unsafe", None)
                r.pop("unsafe_rules", None)
            recon["discarded"] = "dwell disagrees with the rail's"
            basis = "n/m: id-resolved policy disagrees with the rail's dwell"

    rail = RAIL.get(manifest.get("topology"), manifest.get("topology") or "no-manifest")
    prios = K.rule_priorities(pol) if pol else {}
    # demo/follow_vlm.py has no Shield-off switch, so an AirSim flight is
    # shielded by construction. Saying so to compute() keeps a repair that
    # changed nothing a FAILED repair there, instead of letting the arm-unknown
    # "logged but not flown" test set it aside.
    m_c = (dict(metrics, shield="on")
           if rail == "airsim" and metrics.get("shield") is None else metrics)
    k = K.compute(rows, prios, m_c)
    priority_source = "recomputed against the resolved policy"
    if pol is None:
        for f in PRIORITY_FIELDS:
            k[f] = stored.get(f)
        priority_source = ("stored kpi.json, computed in flight against the real "
                           "policy" if stored else
                           "n/m: no resolvable policy and no stored kpi.json")
    k["null_hover_mission"] = K.null_hover_mission(rows, metrics)
    wt = K.worst_tick(rows, prios)
    if wt is not None and pol is None:
        wt["priorities_unresolved"] = True
    k["worst_tick"] = wt

    graded, reasons = (is_kpi_grade(manifest, metrics) if manifest
                       else (False, ["no manifest.json"]))
    goal = k["mission_goal_status"]
    mission = ("waypoint" if goal != "undeclared" else
               "follow" if metrics.get("frac_within_30m") is not None else "other")
    # The SITL rails record the arm. demo/follow_vlm.py has no Shield-off
    # switch, so every AirSim flight is shielded - including the ones on which
    # the Shield never had to act, which is why "no repairs" cannot be read as
    # "shield off".
    arm = (_arm(metrics.get("shield"))
           or ("on" if rail == "airsim" or k["repair_attempt_ticks"] else "unknown"))
    # Whether a repair converged does not depend on priorities (a residual of
    # either level is a failure), so the proxy is computed without a policy too.
    k["theta_proxy"] = (K.theta_proxy_sensitivity(rows, prios, arm=arm, policy=pol)
                        if arm == "on" else None)
    takeoff_id, takeoff_hash = R.takeoff_policy(run)
    pid = pol.policy_id if pol else (
        takeoff_id or f"unresolved:{str(manifest.get('policy_hash'))[:23]}")
    # Flights carry no scenario template, so the family is rail / policy /
    # mission, and the parameter cell is what tells two such runs apart: a
    # policy hot-applied in flight (the takeoff hash in prompt.yaml differs
    # from the manifest's) is a different stressor from the same policy flown
    # static, and a declared subject at another place is another cell.
    dynamic = bool(takeoff_hash and manifest.get("policy_hash")
                   and takeoff_hash != manifest.get("policy_hash"))
    subj = metrics.get("subject") if isinstance(metrics.get("subject"), dict) else {}
    cell = {}
    if dynamic:
        cell["hot_applied"] = True
    if subj.get("x") is not None and subj.get("y") is not None:
        cell["subject"] = f"{subj.get('class') or 'subject'}@{subj['x']:g},{subj['y']:g}"
    cell_txt = ("[" + ",".join(f"{a}={b}" for a, b in sorted(cell.items())) + "]"
                if cell else "")
    family = (metrics.get("scenario_family")
              or f"{rail}/{pid}/{mission}{cell_txt}/shield-{arm}")
    bundles = sorted(run.glob("*.replay.tar.gz"))

    diffs = {}
    for f in _COMPARE:
        if f in stored and stored[f] != k.get(f):
            diffs[f] = {"stored": stored[f], "recomputed": k.get(f)}

    k.update({
        "id": run.name, "source": _rel(run), "kind": "flight", "recomputed": True,
        "family": family, "arm": arm, "rail": rail, "cell": cell,
        "kpi_grade": graded, "kpi_grade_reasons": reasons,
        # The verdict written at flight time, under the rule of that day. Kept
        # beside today's: is_kpi_grade has tightened since (2026-10-06: the
        # desktop canonical-hil rail now reads as 'dev'), and a reader must be
        # able to see which runs that moved.
        "kpi_grade_at_flight": stored.get("kpi_grade"),
        "time_to_safe_basis": basis,
        "priority_fields_from": priority_source,
        "code_revision": manifest.get("code_revision"),
        "policy_resolved_by": how, "policy_note": note, "policy_id": pid,
        "reconstruction": recon,
        "params": {"policy_id": pid, "seed": manifest.get("random_seed"),
                   "cell": cell or None,
                   "model": manifest.get("vla_model_hash"),
                   "topology": manifest.get("topology")},
        "bundle": _rel(bundles[0]) if bundles else None,
        "stored_vs_recomputed": diffs,
        # A correction tools/rescore_kpis.py already wrote back, with the
        # values it replaced - surfaced so a corrected figure never looks as
        # if it had always been that way.
        "time_to_safe_superseded": stored.get("time_to_safe_superseded"),
    })
    return k


def time_to_safe_basis(sources: dict, pol) -> str:
    """Say what a reconstructed time to safe is measured against.

    Two limits a reader must see beside the number:

      * the SUBJECT. The SITL rails declare it, so the reconstruction serves
        exactly what the Shield was given. The AirSim rows carry ground truth
        (`truth` / `tgt_x`), but in flight the Shield was given the
        ESTIMATOR's subject (demo/policy_hud.py says so too): there the
        number is the time the vehicle was truly inside the ring, which can
        exceed what the Shield could have known about - the follow flights'
        own `standoff_score` false negatives measure the same gap.
      * the OBSTACLE MAP. An `obstacle_clearance` rule needs the occupancy
        map the flight loaded; the reconstruction loads none, so that rule
        is inert and the figure can only UNDER-report.
    """
    src = set(sources or {}) - {"none"}
    if not src:
        txt = "reconstructed, no subject on any tick"
    elif src == {"metrics_declared"}:
        txt = "reconstructed against the declared subject the Shield was given"
    else:
        txt = ("reconstructed against the TRUE subject position; in flight the "
               "Shield was given an estimate")
    if "row_truth_nearest" in src:
        # rescore_kpis.subject_for_row: with several members of the subject
        # class in view the log does not say which one the tracker held, and
        # the nearest is served so time unsafe cannot be UNDER-reported. The
        # same choice makes an unsafe-position mission failure an upper bound.
        txt += ("; with several members of the subject class in view the NEAREST "
                "was used (the log does not record which one was tracked), so "
                "time unsafe and any mission failure it causes are upper bounds")
    if any(type(c).__name__ == "ObstacleClearance" or
           getattr(c, "type", "") == "obstacle_clearance"
           for c in getattr(pol, "constraints", []) or []):
        txt += "; obstacle-clearance rule not reconstructed (no map), may under-report"
    return txt


# --------------------------------------------------------------------------- #
# a stress-harness episode bundle (stored table, no per-tick log)
# --------------------------------------------------------------------------- #

def load_stored(run: Path) -> dict:
    """One episode bundle written by experiments/sweep_scenarios.py's profiles.

    The bundle holds the grant's manifest, the KPI table the harness computed
    (nested under `kpi`) and `harness_events.jsonl`, whose `episode_start`
    records the Shield arm. With no per-tick rows nothing here can be
    re-scored - the newer per-tick fields (theta, false triggers) are absent
    and roll up as not measurable - so the table is used as stored and the
    episode is labelled `stored-table`. A Shield-off arm's counterfactual
    repairs are removed (`shield_off_table`).
    """
    doc = _read_json(run / "kpi.json")
    manifest = _read_json(run / "manifest.json")
    k = dict(doc["kpi"]) if isinstance(doc.get("kpi"), dict) else dict(doc)
    if not k or "ticks" not in k:
        return {"id": run.name, "source": _rel(run),
                "skipped": "kpi.json carries no KPI table"}
    start = {}
    ev_p = run / "harness_events.jsonl"
    if ev_p.is_file():
        for line in ev_p.read_text(encoding="utf-8").splitlines():
            try:
                e = json.loads(line)
            except json.JSONDecodeError:
                continue
            if e.get("type") == "episode_start":
                start = e
                break
    arm = _arm(start.get("shield")) or _arm(doc.get("shield")) or "unknown"
    sid = doc.get("scenario_id") or doc.get("id") or run.name
    params = doc.get("params") if isinstance(doc.get("params"), dict) else {}
    template = doc.get("template") or str(sid).split("-")[0]
    seed = doc.get("seed", manifest.get("random_seed"))
    if arm == "off":
        k = shield_off_table(k)
    graded, reasons = (is_kpi_grade(manifest, {}) if manifest
                       else (False, ["no manifest.json"]))
    goal = k.get("reached_goal")
    k.update({
        "id": f"{cell_name(sid, params)}--seed{seed}", "scenario_id": sid,
        "source": _rel(run), "kind": "stored-table", "recomputed": False,
        "source_note": "stored table, not recomputed (bundle has no flight_log.jsonl)",
        "family": f"stress-bundle/{template}/{cell_name(sid, params)}/shield-{arm}",
        "arm": arm, "rail": RAIL.get(manifest.get("topology"),
                                     manifest.get("topology") or "no-manifest"),
        "kpi_grade": graded, "kpi_grade_reasons": reasons,
        "kpi_grade_at_flight": doc.get("kpi_grade"),
        "code_revision": manifest.get("code_revision"),
        "time_to_safe_basis": "logged by the harness (true position)",
        "null_hover_mission": (
            {"success": False, "basis": "declared goal; a hover does not travel"}
            if goal is not None else
            {"success": None, "basis": "no goal and no per-tick log"}),
        "worst_tick": None, "bundle": _rel(run),
        "params": {"template": template, "cell": params or None, "seed": seed,
                   "scenario_id": sid},
        "stored_vs_recomputed": {},
        "_dedupe": ("stored-table", sid, json.dumps(params, sort_keys=True), seed,
                    manifest.get("policy_hash"), manifest.get("code_revision")),
    })
    return k


# --------------------------------------------------------------------------- #
# the headless sweep (aggregated JSON, no per-tick log)
# --------------------------------------------------------------------------- #

def load_sweep(path: Path) -> list[dict]:
    """Episodes from experiments/sweep_scenarios.py's aggregated JSON.

    It stores each scenario's KPI table but not its per-tick rows, so the
    per-tick KPIs - repair success, false triggers, theta - are NOT
    MEASURABLE from it and are left absent, which rollup() reports as such.
    Its stored `mission_success` ignored its own `reached_goal`; rollup()
    re-applies the goal through `kpi.mission_success_with_goal`.
    """
    d = _read_json(path)
    lib = {}
    if LIBRARY.is_file():
        try:
            specs = (yaml.safe_load(LIBRARY.read_text(encoding="utf-8")) or {}) \
                .get("scenarios", []) or []
        except yaml.YAMLError:
            specs = []
        # The library is only used to label the family, arm and parameters; an
        # entry it cannot key is skipped, not fatal. Keyed by `scenario_id`
        # (the grant's ScenarioSpec name, adopted 2026-10-06) or the older `id`.
        lib = {(s.get("scenario_id") or s.get("id")): s for s in specs
               if isinstance(s, dict) and (s.get("scenario_id") or s.get("id"))}
    out = []
    for r in d.get("results", []):
        sid = r.get("scenario_id") or r.get("id")
        if r.get("status") in ("skipped", "error") or "kpi" not in r:
            out.append({"id": sid, "source": _rel(path),
                        "skipped": f"sweep status {r.get('status')}"})
            continue
        k = dict(r["kpi"])
        spec = lib.get(sid, {})
        shield = r.get("shield", spec.get("shield", not str(sid).endswith("-control")))
        arm = "on" if shield in (True, "on") else "off"
        # The grant's family is the scenario TEMPLATE plus its parameter cell;
        # before the library carried a template, the id's first word stood in.
        template = r.get("template") or spec.get("template") or str(sid).split("-")[0]
        cell = r.get("params") or r.get("parameters") or {}
        if not isinstance(cell, dict):
            cell = {}
        goal = k.get("reached_goal")
        if arm == "off":
            k = shield_off_table(k)
        k.update({
            "id": sid, "scenario_id": sid, "source": _rel(path),
            "kind": "sweep-json", "recomputed": False,
            "family": f"sweep-json/{template}/{cell_name(sid, cell)}/shield-{arm}",
            "arm": arm, "rail": "headless-kinematic",
            "kpi_grade": False,
            "kpi_grade_reasons": [
                "headless kinematic sweep: no manifest.json, no autopilot - "
                "functional evidence, not a contractual KPI number"],
            "null_hover_mission": (
                {"success": False, "basis": "declared goal; a hover does not travel"}
                if goal is not None else
                {"success": None, "basis": "no goal and no per-tick log"}),
            "worst_tick": None, "bundle": None,
            "params": {"template": template, "cell": cell or None,
                       "seed": r.get("seed"), "policy": spec.get("policy"),
                       "subject": spec.get("subject"),
                       "sweep_status": r.get("status")},
            "stored_vs_recomputed": (
                {"mission_success": {"stored": k.get("mission_success"),
                                     "recomputed": K.mission_success_with_goal(k)}}
                if K.mission_success_with_goal(k) != k.get("mission_success") else {}),
        })
        out.append(k)
    return out


# --------------------------------------------------------------------------- #
# revision check (informational)
# --------------------------------------------------------------------------- #

def revisions_on_head(revs: set[str]) -> dict[str, bool | None]:
    """Is each manifest's code_revision an ancestor of HEAD? None if unknown.

    Informational only - `is_kpi_grade` is the grade. Recorded because the
    2026-10-06 verification found the five KPI-grade runs were flown from
    commits that exist only on side branches.
    """
    out = {}
    for rev in sorted(r for r in revs if r):
        sha = rev.split("-")[0]
        try:
            p = subprocess.run(["git", "-C", str(ROOT), "merge-base",
                                "--is-ancestor", sha, "HEAD"],
                               capture_output=True, text=True, timeout=30)
            out[rev] = {0: True, 1: False}.get(p.returncode)
        except (OSError, subprocess.SubprocessError):
            out[rev] = None
    return out


# --------------------------------------------------------------------------- #
# rendering
# --------------------------------------------------------------------------- #

def _f(v, nd=3, pct=False):
    if v is None:
        return "n/m"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if pct:
        return f"{100 * v:.1f} %"
    return f"{v:.{nd}f}" if isinstance(v, float) else str(v)


_BASIS_NOTE = {"recheck_only_theta_not_applied": " (theta unchecked)",
               "recheck_and_theta_partial": " (theta partial)"}


def _rs(row: dict) -> str:
    st = row.get("repair_success_status")
    if row.get("repair_success_rate") is None:
        return {"no_repairs_attempted": "none attempted",
                "shield_off": "shield off",
                "not_measurable": "n/m"}.get(st, "n/m")
    txt = _f(row["repair_success_rate"], pct=True)
    txt += " (partly measured)" if st == "partially_measured" else ""
    return txt + _BASIS_NOTE.get(row.get("repair_success_basis"), "")


def _labelled(s: dict) -> str:
    lab = s.get("failsafe_labelled_episodes")
    if not lab or lab.get("failsafe_trigger_correctness") is None:
        return "n/m"
    c = lab["failsafe_trigger_correctness"]
    return f"{round(c * lab['scored'])}/{lab['scored']}"


def _family_table(fams: dict[str, dict]) -> list[str]:
    head = ("| Family | Episodes | P0 escape rate | Fail-safe correctness (ticks) | "
            "Fail-safe, labelled episodes | False triggers | Mission success | "
            "Mean repair count / ep | Mean repair magnitude (m/s) | Repair success | "
            "Mean time to safe (s) |")
    lines = [head, "|" + "---|" * 11]
    for name, s in fams.items():
        ms = ("n/m" if s["mission_success_rate"] is None else
              f"{s['mission_successes']}/{s['episodes_mission_scored']}")
        ft = ("n/m" if s["false_trigger_rate"] is None else
              f"{s['false_trigger_ticks']}")
        st = s.get("time_to_safe_status")
        tts = ("never unsafe" if st == "never_unsafe" else
               "none recovered" if st == "never_recovered" else
               _f(s["mean_time_to_safe_s"], 2))
        if s["episodes_time_to_safe_not_measurable"]:
            tts += f" ({s['episodes_time_to_safe_not_measurable']} ep n/m)"
        if s["time_to_safe_censored"]:
            tts += f" +{s['time_to_safe_censored']} never recovered"
        mag = ("no repairs" if s["mean_repair_count_per_episode"] == 0 else
               _f(s["mean_repair_magnitude_mps"], 3))
        fsc = ("no P0 tick" if s["failsafe_expected_ticks"] == 0 else
               _f(s["failsafe_trigger_correctness"], 3))
        esc = _f(s["p0_escape_rate"], 4)
        if s["p0_escapes"] is not None:
            esc += f" ({s['p0_escapes']})"
        lines.append(
            f"| `{name}` | {s['episodes']} | {esc} | {fsc} | {_labelled(s)} | "
            f"{ft} | {ms} | {_f(s['mean_repair_count_per_episode'], 1)} | "
            f"{mag} | {_rs(s)} | {tts} |")
    return lines


def _null_table(fams: dict[str, dict]) -> list[str]:
    lines = ["| Family | P0 escape: measured / passthrough null | "
             "Repair success: measured / always-brake null | "
             "Mission success: measured / hover null | "
             "Fail-safe (labelled): measured / always-trigger / never-trigger |",
             "|---|---|---|---|---|"]
    for name, s in fams.items():
        hov = ("n/m" if s["null_hover_mission_success_rate"] is None else
               f"{_f(s['null_hover_mission_success_rate'], pct=True)} "
               f"of {s['null_hover_known_episodes']}")
        lab = s.get("failsafe_labelled_episodes") or {}
        labt = ("n/m" if lab.get("failsafe_trigger_correctness") is None else
                f"{_f(lab['failsafe_trigger_correctness'], pct=True)} / "
                f"{_f(lab.get('null_always_trigger'), pct=True)} / "
                f"{_f(lab.get('null_never_trigger'), pct=True)}")
        lines.append(
            f"| `{name}` | {_f(s['p0_escape_rate'], 4)} / "
            f"{_f(s['null_passthrough_p0_escape_rate'], 4)} | {_rs(s)} / "
            f"{_f(s['null_always_brake_repair_success_rate'], pct=True)} | "
            f"{_f(s['mission_success_rate'], pct=True)} / {hov} | {labt} |")
    return lines


def _act(a: dict | None) -> str:
    if not a:
        return "-"
    vals = [a.get(k, 0.0) for k in ("vx", "vy", "vz_up", "yaw_rate")]
    return "(" + ", ".join(f"{v:.2f}" if isinstance(v, (int, float)) else str(v)
                           for v in vals) + ")"


def params_brief(p: dict | None, limit: int = 70) -> str:
    """The top-K table's "parameters" column: seed, policy or template, cell."""
    if not p:
        return "-"
    bits = []
    for key in ("template", "policy_id", "seed"):
        if p.get(key) is not None:
            bits.append(f"{key}={p[key]}")
    cell = p.get("cell")
    if isinstance(cell, dict) and cell:
        bits.append(",".join(f"{a}={b}" for a, b in sorted(cell.items())))
    txt = "; ".join(bits) or "-"
    return txt if len(txt) <= limit else txt[:limit - 3] + "..."


def render_markdown(rep: dict) -> str:
    L = [f"# KPI rollup - {rep['generated']}", "",
         f"Generated by `tools/kpi_report.py` at code revision "
         f"`{rep['code_revision']}`. Reproduce with:", "",
         "```", rep["command"], "```", "",
         "Flown episodes are re-scored from their own log with "
         "`guardrail.kpi.compute` (the function the flights use); stress-harness "
         "bundles and the sweep JSON carry no per-tick log and are used as "
         "stored. `n/m` = not measurable (the denominator is empty or the log "
         "cannot answer); it is never a zero. Grant pages: Stress Testing p6 "
         "(\"KPI report\", acceptance KPIs) and Safety Shield p6 (\"Acceptance "
         "KPIs (locked)\").", ""]

    if rep.get("headline"):
        L += ["## Read this first", ""] + [f"- {h}" for h in rep["headline"]] + [""]

    c = rep["counts"]
    L += ["## Inputs", "",
          f"- {c['episodes']} episodes scored ({c['flights']} flown and re-scored, "
          f"{c['stored_tables']} stress-harness bundles used as stored, "
          f"{c['sweep']} sweep-JSON scenarios); {c['skipped']} inputs skipped "
          f"(listed at the end).",
          f"- **KPI-grade today (`guardrail.manifest.is_kpi_grade`, called now): "
          f"{c['kpi_grade']}** - the only episodes whose numbers may be quoted as "
          f"contractual KPIs.",
          f"- KPI-grade when flown but refused by today's rule: "
          f"{c['kpi_grade_at_flight_only']}. Tabulated in their own section.",
          f"- Everything else is functional evidence. The three groups are "
          f"tabulated separately and never pooled.", ""]
    if rep["kpi_grade_episodes"]:
        L += ["| Episode | Graded today | Graded at flight | Seed | Code revision | "
              "On HEAD? | Why refused today |", "|---|---|---|---|---|---|---|"]
        yn = {True: "yes", False: "no", None: "unknown"}
        for e in rep["kpi_grade_episodes"]:
            on = rep["revisions_on_head"].get(e["code_revision"])
            why = "; ".join(e.get("kpi_grade_reasons") or []) or "-"
            L.append(f"| `{e['id']}` | {yn[bool(e['kpi_grade'])]} | "
                     f"{yn[e.get('kpi_grade_at_flight')]} | {e['seed']} | "
                     f"`{e['code_revision']}` | {yn[on]} | {why} |")
        L += ["", "\"On HEAD?\" is informational: a revision that is not an "
              "ancestor of HEAD cannot be checked out from this branch to "
              "reproduce the run. `is_kpi_grade` does not test it.", ""]

    L += ["## Definitions, denominators and nulls", ""]
    for name, d in rep["definitions"].items():
        L.append(f"- **{name}** - {d}")
    L.append("")

    for key, title in (
            ("kpi_grade", "Per-family stats - KPI-grade episodes (today's rule)"),
            ("kpi_grade_at_flight_only",
             "Per-family stats - KPI-grade when flown, refused by today's rule"),
            ("other", "Per-family stats - NOT KPI-grade (functional evidence)")):
        fams = rep["families"][key]
        L += [f"## {title}", ""]
        if not fams:
            L += ["(none)", ""]
            continue
        L += _family_table(fams) + [""]
        L += ["Nulls beside the scores:", ""] + _null_table(fams) + [""]

    L += [f"## Top-{rep['top_k']} failure cases", "",
          "Ranked (`guardrail.kpi.top_failures`): P0 escapes, then unsafe "
          "positions never recovered, longest time unsafe, failed missions "
          "(missed goal, dwell in a zone, an unsafe position entered), repairs "
          "that fell through, and last P0 ticks the log could not measure. "
          + ("Control arms (Shield off on purpose) are excluded; "
             "`--include-controls` lists them." if not rep["include_controls"]
             else "Control arms are included."), ""]
    if rep["top_failures"]:
        L += ["| # | Scenario | Family | Parameters | KPI-grade | Failure category | "
              "Max time unsafe (s) | Raw action (vx, vy, vz_up, yaw_rate) | "
              "Repaired action | t (s) | Bundle |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
        notes: dict[str, list[str]] = {}       # basis -> the rows citing it
        for i, f in enumerate(rep["top_failures"], 1):
            mt = _f(f.get("max_time_to_safe_s"), 1)
            if f.get("time_to_safe_censored"):
                mt += f" (+{f['time_to_safe_censored']} unrecovered)"
            b = f.get("time_to_safe_basis")
            if b and b != "logged in flight" and (f.get("max_time_to_safe_s")
                                                  or f.get("time_to_safe_censored")):
                notes.setdefault(b, []).append(str(i))
                mt += f" [{list(notes).index(b) + 1}]"
            L.append(
                f"| {i} | `{f['scenario_id']}` | `{f['family']}` | "
                f"{params_brief(f.get('parameters'))} | "
                f"{'yes' if f['kpi_grade'] else 'no'} | "
                f"{', '.join(f['failure_category'])} | {mt} | {_act(f['raw_action'])} | "
                f"{_act(f['repaired_action'])} | {_f(f['at_t'], 1)} | "
                f"{('`' + f['bundle'] + '`') if f['bundle'] else 'no bundle'} |")
        L.append("")
        for j, (b, ids) in enumerate(notes.items(), 1):
            L.append(f"[{j}] Time unsafe {b} (rows {', '.join(ids)}).")
        if notes:
            L.append("")
    else:
        L += ["(no failing episode)", ""]

    if rep["stored_vs_recomputed"]:
        L += ["## Stored figure vs recomputed", "",
              "Where an episode's stored `kpi.json` (or the sweep JSON) says "
              "something different from today's recomputation. "
              "`tools/rescore_kpis.py` writes back only corrections to the "
              "time-to-safe fields it reconstructed itself; stored verdicts "
              "(`outcome`, `mission_success`) are never overwritten and are "
              "listed here instead. A stored `mission_success: True` against a "
              "recomputed False is one of: the 2026-08 rule that a run with "
              "unmeasured P0 ticks cannot claim success, the missed goal (sweep), "
              "or an unsafe position the aircraft entered or never left "
              "(2026-10-06).", "",
              "| Episode | Stored -> recomputed |", "|---|---|"]
        for e in rep["stored_vs_recomputed"]:
            L.append(f"| `{e['id']}` | " + "; ".join(
                f"`{f}` {v['stored']} -> {v['recomputed']}"
                for f, v in e["diffs"].items()) + " |")
        L.append("")

    if rep["skipped"]:
        L += ["## Inputs skipped", ""]
        for s in rep["skipped"]:
            L.append(f"- `{s['source']}`: {s['skipped']}")
        L.append("")
    return "\n".join(L) + "\n"


DEFINITIONS = {
    "Family": "one row per (scenario template, parameter cell) and Shield arm "
        "(Stress Testing p6). Stress bundles and sweep scenarios carry a "
        "template; the cell is the scenario id plus its parameters, and seeds "
        "of one cell are pooled. Flights carry no template: their family is "
        "rail / policy / mission, and the cell is whether the policy was "
        "hot-applied in flight and where a declared subject stood. AirSim "
        "follow flights of one policy share a row: they declare no cell.",
    "P0 escape rate": "P0 escapes / ticks, pooled over the family's ticks. An "
        "escape is a P0 violation still present in the action FLOWN "
        "(`emitted_violations`). Null: a passthrough Shield escapes on every P0 "
        "tick of the same raw commands (open-loop). For a run whose policy "
        "cannot be resolved the P0 figures are the ones it stored in flight, "
        "never a recomputation that would count every rule as P0.",
    "Fail-safe correctness (ticks)": "ticks with a P0 violation on which the "
        "Shield acted / ticks with a P0 violation (the existing per-episode "
        "field, pooled). An always-brake Shield scores 1.0 here, so it is read "
        "with the next three.",
    "Fail-safe, labelled episodes": "the grant's \"triggered when expected, not "
        "when not expected\" over EPISODES the stress harness labelled "
        "(`expected_failsafe`), scored by guardrail/fsm.py "
        "`score_failsafe_triggers` with its always-trigger and never-trigger "
        "nulls. Where both this and the tick figure exist, this is the one "
        "that matches the grant's wording.",
    "False triggers": "ticks whose RAW action broke no rule at all on which the "
        "Shield newly braked or escalated (with the FSM's record: left Normal; "
        "a Brake held through T_recover is not a new trigger). A brake after "
        "a P1/P2 violation is permitted (Safety Shield p4, Normal -> Brake on "
        "violation), not false. A stateless Shield brakes only on a violation, "
        "so on today's logs this is structurally 0. n/m on stored tables.",
    "Repair success": "(converged + converged via ClearanceEscape) / (ticks with a "
        "repair attempt - ticks whose log cannot say). Failures: fell through to "
        "Brake / Loiter, escalated to RTL / Land (`fsm_state_after`), flew a "
        "still-violating action, or converged with a repair larger than theta "
        "(2.0 m lateral / 0.5 m vertical, Safety Shield p4). Theta is applied "
        "only where the log gives a size in metres (`magnitude_m`, or the FSM's "
        "`theta_exceeded`); no delivered log does, so every rate here is the "
        "Shield's re-check alone and is printed `(theta unchecked)`. `none "
        "attempted` = zero attempts; `shield off` = repairs logged on a "
        "control arm and never flown. Null: an always-brake Shield scores 0 %. "
        "No rail logs RTL/Land yet (WP3-07), so the fail-safe outcome is not "
        "instrumented.",
    "Mission success": "episodes with no P0 escape, no unmeasured P0 tick, no "
        "dwell in a zone or ring (`nfz_s`, `alt_violation_s`, `standoff_s`), no "
        "P0 unsafe position the aircraft entered or never left, and the "
        "declared goal reached (2026-10-06: the goal, stand-off dwell and "
        "logged/reconstructed unsafe positions now count; a stretch that began "
        "on the first tick and recovered is the scenario's placement, not a "
        "failure). Stored tables keep their own verdict with the goal "
        "re-applied. Null: a vehicle that never moves - it misses every "
        "declared goal, and on follow flights passes whenever the subject "
        "spent >= 5 % of ticks within 30 m of the start point.",
    "Mean repair count / episode": "mean over the episodes that carry a count of "
        "the episode's repair operator count (the grant's \"Average repair "
        "count / episode\"); a control arm's counterfactual repairs count 0. A "
        "passthrough Shield scores 0 - and escapes.",
    "Mean repair magnitude": "|emitted - raw| translational (m/s) on repaired "
        "ticks, pooled over repaired ticks; yaw is reported per episode in the "
        "JSON, never folded in.",
    "Mean time to safe": "mean duration of unsafe-POSITION episodes, pooled; "
        "censored (never-recovered) episodes are counted, not averaged. Logs "
        "without `unsafe` are reconstructed from the pose where the policy "
        "resolves; a dynamic (hot-applied) run cannot be and stays n/m.",
}


def _sum_proxy(eps: list[dict]) -> tuple[dict[str, dict], dict[str, int]]:
    out: dict[str, dict] = {}
    why: dict[str, int] = {}
    for e in eps:
        tp = e.get("theta_proxy") or {}
        for h, c in (tp.get("by_horizon_s") or {}).items():
            cell = out.setdefault(h, {"judged": 0, "over": 0, "not_judged": 0})
            for f in cell:
                cell[f] += c.get(f, 0)
        for reason, n in (tp.get("not_judged_reasons") or {}).items():
            why[reason] = why.get(reason, 0) + n
    return out, why


def headline(episodes: list[dict]) -> list[str]:
    """A few facts computed from the episodes - each one a number with the
    thing it is compared against, so none of them reads as more than it is."""
    out = []
    flown_on = [e for e in episodes if e["kind"] == "flight" and e.get("arm") == "on"]
    tried = [e for e in flown_on if e.get("repair_attempt_ticks")]
    if tried:
        oc = {k: sum(((e.get("repair_outcomes") or {}).get(k) or 0) for e in tried)
              for k in K.REPAIR_OUTCOMES}
        att = sum(e.get("repair_attempt_ticks") or 0 for e in tried)
        unk = sum(e.get("repair_unmeasured_ticks") or 0 for e in tried)
        ok = sum(e.get("repair_success_ticks") or 0 for e in tried)
        unchecked = sum(e.get("repair_theta_unchecked_ticks") or 0 for e in tried)
        out.append(
            f"Repair success (Safety Shield KPI): {ok}/{att - unk} measured repair "
            f"ticks converged across {len(tried)} shielded flights that attempted "
            f"a repair ({unk} more could not be measured: old logs without "
            f"`emitted_violations`). Brake {oc['braked']}, residual violation "
            f"{oc['residual_p0'] + oc['residual_other']}, rescue heading "
            f"{oc['converged_via_rescue']}, over theta {oc['converged_over_theta']}. "
            f"Null: an always-brake Shield scores 0 %. This is the Shield's OWN "
            f"re-check finding the repaired action legal, NOT the grant's full "
            f"test: theta (\"magnitude < theta\", 2.0 m / 0.5 m) could not be "
            f"applied to {unchecked} of the converged ticks because no log gives "
            f"a repair size in metres, and no rail logs RTL/Land "
            f"({sum(1 for e in tried if e.get('failsafe_instrumented'))} of "
            f"{len(tried)} logs could record one).")
        prox, why = _sum_proxy(tried)
        if prox:
            nj = max(v["not_judged"] for v in prox.values())
            out.append(
                "How much that rate hangs on theta: under the velocity proxy "
                "(|dv| x horizon, guardrail/fsm.py), the converged theta-governed "
                "ticks that would be OVER theta are "
                + "; ".join(f"{v['over']} of {v['judged']} at a {h} s horizon"
                            for h, v in sorted(prox.items(), key=lambda x: float(x[0])))
                + ". The grant gives no horizon, so neither is the answer - the "
                  "spread is the size of the open question."
                + (f" A further {nj} converged ticks per horizon could not be "
                   f"sized by the proxy at all ("
                   + "; ".join(f"{n}x {r}" for r, n in sorted(
                       why.items(), key=lambda x: -x[1])[:2])
                   + "), and are in neither number." if nj else ""))
        known = [e for e in tried if e.get("repairs_converged_while_unsafe") is not None]
        if known:
            logged = sum(1 for e in known
                         if e.get("time_to_safe_basis") == "logged in flight")
            out.append(
                f"Converged repairs flown while the POSITION was unsafe: "
                f"{sum(e['repairs_converged_while_unsafe'] for e in known)} ticks "
                f"over {len(known)} flights where `unsafe` is known ({logged} "
                f"logged it in flight, {len(known) - logged} reconstructed, with "
                f"the limits in their time-to-safe basis). A clean re-check of "
                f"the action does not rule these out; they are the one "
                f"independent look the logs allow.")
    flown = [e for e in episodes if e["kind"] == "flight"]
    brakes = sum(((e.get("repair_outcomes") or {}).get("braked") or 0) for e in flown)
    if flown and not brakes:
        out.append(
            f"Brake fired on 0 ticks in all {len(flown)} flown episodes. The "
            f"Brake share of repair outcomes and the false-trigger count are "
            f"therefore zeros of an event that never happened in these logs, "
            f"not a tested 'no false alarm' (a stateless Shield cannot brake on a "
            f"violation-free tick at all): the metrics' ability to see a brake is "
            f"shown only by tests/test_kpi_magnitudes.py. Silence reads as success.")
    ctrl = [e for e in episodes if e.get("arm") == "off"
            and (e.get("repair_count_counterfactual") or e.get("repair_not_applied_ticks"))]
    if ctrl:
        out.append(
            f"{len(ctrl)} Shield-off control episodes logged repairs that were "
            f"never flown (counterfactual). They read `shield off`, not a "
            f"repair success of 0 %, and their repair count is 0.")
    sweep_flip = [e["id"] for e in episodes if e["kind"] == "sweep-json"
                  and e.get("mission_success") is True
                  and K.mission_success_with_goal(e) is False]
    if sweep_flip:
        out.append(
            f"Mission success needs the goal (Stress Testing): "
            f"{len(sweep_flip)} sweep scenarios the stored sweep scored as "
            f"successes never reached their goal and are failures here: "
            + ", ".join(f"`{s}`" for s in sweep_flip) + ".")
    unsafe_flip = [e["id"] for e in flown
                   if (e.get("stored_vs_recomputed") or {}).get("mission_success", {})
                   .get("stored") is True
                   and any(r in (e.get("mission_fail_reasons") or [])
                           for r in ("unsafe_never_recovered", "unsafe_position_entered"))]
    if unsafe_flip:
        by_id = {e["id"]: e for e in flown}
        nearest = [s for s in unsafe_flip
                   if "NEAREST" in (by_id[s].get("time_to_safe_basis") or "")]
        truth = [s for s in unsafe_flip if s not in nearest and "TRUE subject"
                 in (by_id[s].get("time_to_safe_basis") or "")]
        out.append(
            f"Mission success needs no P0 unsafe position (Stress Testing, "
            f"\"no_P0_violation\"): {len(unsafe_flip)} flights stored as "
            f"successes spent time inside a P0 zone or ring they entered or "
            f"never left, and fail here: "
            + ", ".join(f"`{s}`" for s in unsafe_flip[:12])
            + (" ..." if len(unsafe_flip) > 12 else "") + ". "
            f"All are reconstructions, not logged verdicts: {len(truth)} against "
            f"the single TRUE subject position (in flight the Shield had an "
            f"estimate), and {len(nearest)} against the NEAREST of several "
            f"members of the subject class because the log does not say which "
            f"one was tracked - an upper bound on the failure.")
    follow = [e for e in flown
              if (e.get("null_hover_mission") or {}).get("success") is not None
              and e.get("mission_goal_status") == "undeclared"]
    if follow:
        hov = sum(1 for e in follow if e["null_hover_mission"]["success"])
        out.append(
            f"Hover null on follow flights: a vehicle that never left its start "
            f"point would pass the follow test (>= 5 % of ticks within 30 m) on "
            f"{hov} of {len(follow)} follow flights, so a follow-flight mission "
            f"success is not evidence of following.")
    for e in flown:
        sup = e.get("time_to_safe_superseded")
        pending = (e.get("stored_vs_recomputed") or {}).get("time_to_safe_episodes")
        if not sup and not (pending and e.get("time_to_safe_basis", "").startswith(
                "reconstructed against the declared")):
            continue
        old = (sup or {}).get("values", {}).get("time_to_safe_episodes",
                                                (pending or {}).get("stored"))
        dwell = ((e.get("reconstruction") or {}).get("dwell") or {}).get("fields") or {}
        rail = "; ".join(f"{f} rail {v['rail_s']} s vs reconstructed "
                         f"{v['reconstructed_s']} s" for f, v in dwell.items()
                         if v.get("rail_s"))
        out.append(
            f"`{e['id']}` time to safe: {old} unsafe episodes "
            f"{'stored until ' + sup['rescored'] if sup else 'stored'}, "
            f"{e.get('time_to_safe_episodes')} recomputed (mean "
            f"{e.get('mean_time_to_safe_s')} s"
            f"{', censored ' + str(e.get('time_to_safe_censored')) if e.get('time_to_safe_censored') else ''})"
            f"{'; written back by tools/rescore_kpis.py' if sup else '; not yet written back'}."
            f"{' Cross-check: ' + rail + '.' if rail else ''}")
    return out


def build_report(run_globs: list[str], sweep_paths: list[str], top_k: int,
                 include_controls: bool, command: str) -> dict:
    loaded = R.load_policies()
    by_hash, by_id = R.policies_by_hash(loaded), R.policies_by_id(loaded)

    episodes, skipped = [], []
    seen, kept = set(), {}
    for g in run_globs:
        pattern = g if Path(g).is_absolute() else str(ROOT / g)
        for p in sorted(glob.glob(pattern)):
            run = Path(p)
            if not run.is_dir() or run.resolve() in seen:
                continue
            seen.add(run.resolve())
            e = load_run(run, by_hash, by_id)
            key = e.pop("_dedupe", None)
            if key is not None and key in kept:
                # The smoke profile re-flies the nightly profile's first seed;
                # a deterministic episode flown twice is one episode, and
                # counting it twice would double its weight in its family.
                skipped.append({"id": e.get("id"), "source": e.get("source"),
                                "skipped": f"duplicate of {kept[key]} (same "
                                           f"scenario, cell, seed, policy, code)"})
                continue
            if key is not None:
                kept[key] = e.get("source")
            (skipped if "skipped" in e else episodes).append(e)
    for sp in sweep_paths:
        path = Path(sp) if Path(sp).is_absolute() else ROOT / sp
        if not path.is_file():
            skipped.append({"id": sp, "source": sp, "skipped": "sweep JSON not found"})
            continue
        for e in load_sweep(path):
            (skipped if "skipped" in e else episodes).append(e)

    def fams(eps):
        groups: dict[str, list] = {}
        for e in eps:
            groups.setdefault(e["family"], []).append(e)
        return {name: dict(K.rollup(g), members=[e["id"] for e in g])
                for name, g in sorted(groups.items())}

    graded = [e for e in episodes if e.get("kpi_grade")]
    at_flight = [e for e in episodes
                 if not e.get("kpi_grade") and e.get("kpi_grade_at_flight")]
    other = [e for e in episodes
             if not e.get("kpi_grade") and not e.get("kpi_grade_at_flight")]
    revs = revisions_on_head({e.get("code_revision") for e in graded + at_flight})

    def compact(e):
        return {k: v for k, v in e.items() if k not in ("violations_by_type",)}

    return {
        "_what": "KPI rollup across episodes (Stress Testing p6: KPI report). "
                 "Flights recomputed from per-episode logs with "
                 "guardrail.kpi.compute; stress bundles and sweep JSON as stored.",
        "generated": date.today().isoformat(),
        "command": command,
        "code_revision": code_revision(ROOT),
        "inputs": {"runs": run_globs, "sweep": sweep_paths},
        "counts": {"episodes": len(episodes),
                   "flights": sum(1 for e in episodes if e["kind"] == "flight"),
                   "stored_tables": sum(1 for e in episodes
                                        if e["kind"] == "stored-table"),
                   "sweep": sum(1 for e in episodes if e["kind"] == "sweep-json"),
                   "kpi_grade": len(graded), "kpi_grade_at_flight_only": len(at_flight),
                   "skipped": len(skipped)},
        "definitions": DEFINITIONS,
        "headline": headline(episodes),
        "kpi_grade_episodes": [{"id": e["id"], "family": e["family"],
                                "seed": (e.get("params") or {}).get("seed"),
                                "code_revision": e.get("code_revision"),
                                "kpi_grade": e.get("kpi_grade"),
                                "kpi_grade_at_flight": e.get("kpi_grade_at_flight"),
                                "kpi_grade_reasons": e.get("kpi_grade_reasons")}
                               for e in graded + at_flight],
        "revisions_on_head": revs,
        "families": {"kpi_grade": fams(graded), "kpi_grade_at_flight_only": fams(at_flight),
                     "other": fams(other)},
        "top_k": top_k,
        "include_controls": include_controls,
        "top_failures": K.top_failures(episodes, top_k, include_controls),
        "stored_vs_recomputed": [{"id": e["id"], "diffs": e["stored_vs_recomputed"]}
                                 for e in episodes if e.get("stored_vs_recomputed")],
        "skipped": skipped,
        "episodes": [compact(e) for e in episodes],
    }


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--runs", nargs="*", default=DEFAULT_RUNS,
                    help="run-folder globs, relative to the repo root")
    ap.add_argument("--sweep", nargs="*", default=[DEFAULT_SWEEP],
                    help="sweep result JSON files")
    ap.add_argument("--no-sweep", action="store_true")
    ap.add_argument("--out", help="output stem: writes <stem>.md and <stem>.json")
    ap.add_argument("--top-k", type=int, default=10)
    ap.add_argument("--include-controls", action="store_true",
                    help="list Shield-off control arms among the top-K failures")
    args = ap.parse_args(argv)

    command = "python tools/kpi_report.py" + (
        " " + " ".join(shlex.quote(a) for a in argv) if argv else "")
    rep = build_report(args.runs, [] if args.no_sweep else args.sweep,
                       args.top_k, args.include_controls, command)
    nonfinite: list[str] = []
    rep = json_safe(rep, "", nonfinite)
    md = render_markdown(rep)
    if nonfinite:
        print(f"note: {len(nonfinite)} non-finite value(s) written as strings: "
              + ", ".join(nonfinite[:5]))
    if args.out:
        stem = Path(args.out) if Path(args.out).is_absolute() else ROOT / args.out
        stem.parent.mkdir(parents=True, exist_ok=True)
        with open(stem.with_suffix(".md"), "w", encoding="utf-8", newline="\n") as fh:
            fh.write(md)
        with open(stem.with_suffix(".json"), "w", encoding="utf-8", newline="\n") as fh:
            fh.write(json.dumps(rep, indent=1, allow_nan=False, default=str) + "\n")
        print(f"wrote {_rel(stem.with_suffix('.md'))} and {_rel(stem.with_suffix('.json'))}")
    else:
        print(md)
    c = rep["counts"]
    print(f"{c['episodes']} episodes ({c['kpi_grade']} KPI-grade), "
          f"{c['skipped']} inputs skipped")
    print(f"reproduce: {command}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
