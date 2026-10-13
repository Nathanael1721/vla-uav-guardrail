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
    `manifest.json`, `kpi.json`, `harness_events.jsonl`, and
    `flight_log.jsonl` when the run had `--keep-logs`). A bundle WITH its log
    is re-scored from it under its template and cell (`kind: stress-log`,
    since 2026-10-07); without one its stored table is used and the episode
    says so (`kind: stored-table`);
  * the headless sweep's aggregated JSON (`docs/data/scenario_sweep.json`).

`render_sweep_markdown` is the same report for ONE harness run, written by
experiments/sweep_scenarios.py next to its bundles (`kpi_report.md`).

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

SINCE 2026-10-12 (the WP4 outputs the final report rests on)

  * KPI grade is the manifest AND the evidence: a run `is_kpi_grade` passes
    is demoted unless the replay bundle beside it verifies with a trusted
    signature (`evidence_grade`, guardrail.replay.kpi_evidence_grade) and
    holds the very flight_log.jsonl and manifest.json the report scores the
    run from (`bundle_binding`, since 2026-10-13: a signed bundle of another
    flight, or a log edited after packing, verified as well); the
    manifest's own verdict stays beside it (`kpi_grade_manifest`).
  * A zero that counts an event which could not happen is printed vacuous
    (rollup()'s `vacuous_zeros`), never as a measured 0.
  * The mean repair magnitude per priority and per operator class.
  * WP1 (docs/data/wp1_roundtrip.json, every key it holds, nulls beside),
    WP2 (the compiler's KPIs at the default budget and at the largest budget
    where the naive cut drops a P0 rule: written to docs/data/csp_kpis.json
    by --wp2-only, recomputed for a report by --wp2, else read from that
    file and labelled as read), the tick budget (stored profiles and the
    flown logs' `shield_ms`), and the A/B and per-paraphrase sections, n/m
    until a pilot that reads the prompt flies them.
  * --baseline <report.json>: new, fixed and gone failures and per-family
    deltas against an earlier report; the JSON now carries `failures` for
    the next comparison.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import shlex
import subprocess
import sys
import tarfile
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
    "p0_acted_tick_share", "failsafe_expected_ticks", "failsafe_correct_ticks",
    "failsafe_permitted_ticks", "violations_by_risk_level", "repair_outcomes",
    "outcome", "mission_success", "mission_fail_reasons", "unsafe_p0_ticks",
    "mission_started_unsafe", "repair_magnitude_by_priority",
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
              "repaired_ticks": 0, "repair_magnitude_by_priority": None,
              "repair_magnitude_by_operator_class": None,
              "repair_magnitude_breakdown_status": "shield_off"})
    return k


# --------------------------------------------------------------------------- #
# one flown episode
# --------------------------------------------------------------------------- #

def bundle_binding(run: Path, bundle: Path) -> list[str]:
    """Why `bundle` is not THIS run's evidence; empty when it is.

    The report scores a run from the files in its folder, so the bundle must
    hold those same files: the run's flight_log.jsonl (else, for a stored
    table, its kpi.json) and its manifest.json, byte for byte once line
    endings are LF (a checkout may convert them). A signed bundle of another
    flight, or a folder log edited after packing, verifies as a bundle and
    still says nothing about the figures recomputed here."""
    from guardrail.replay import EPISODE
    names = ["flight_log.jsonl" if (run / "flight_log.jsonl").is_file() else "kpi.json",
             "manifest.json"]
    out: list[str] = []
    try:
        with tarfile.open(bundle, "r:gz") as tar:
            members = {m.name: m for m in tar.getmembers()}
            for n in names:
                f = run / n
                if not f.is_file():
                    continue
                m = members.get(EPISODE + n)
                if m is None:
                    out.append(f"it carries no {n}, so nothing ties it to the run "
                               f"folder's {n}")
                elif (tar.extractfile(m).read().replace(b"\r\n", b"\n")
                      != f.read_bytes().replace(b"\r\n", b"\n")):
                    out.append(f"its {n} is not the run folder's (another flight's "
                               f"bundle, or the folder's file was edited after packing)")
    except (OSError, tarfile.TarError, EOFError) as e:
        return [f"it cannot be opened: {e}"]
    return out


def evidence_grade(run: Path, graded: bool, reasons: list[str],
                   trust_store=None) -> tuple[bool, list[str], str | None]:
    """The KPI grade once the run's replay bundle has been checked as evidence.

    `is_kpi_grade` judges the manifest and cannot see the bundle; a run it
    passes is demoted unless the replay bundle beside it verifies with a
    trusted signature (guardrail.replay.kpi_evidence_grade, which re-derives
    the KPIs from the archived log) AND holds the files this report scores
    the run from (`bundle_binding`). A graded run with NO bundle is demoted
    too: nothing would let a reader re-derive its figures. The bundle is
    `<run>.replay.tar.gz` when present, else the newest `*.replay.tar.gz`
    that is bound to the run (else the newest, which then fails the binding).
    Returns (graded, reasons, the bundle checked or None)."""
    own = run / f"{run.name}.replay.tar.gz"
    found = sorted(run.glob("*.replay.tar.gz"), key=lambda p: p.stat().st_mtime,
                   reverse=True)
    pick = own if own.is_file() else (found[0] if found else None)
    if not graded:
        return False, list(reasons), _rel(pick) if pick else None
    if pick is None:
        return False, list(reasons) + [
            "no replay bundle beside the run (*.replay.tar.gz): nothing lets a "
            "reader re-derive its figures, so it cannot serve as KPI evidence"], None
    unbound = bundle_binding(run, pick)
    if unbound and not own.is_file():
        bound = next((p for p in found[1:] if not bundle_binding(run, p)), None)
        if bound is not None:
            pick, unbound = bound, []
    from guardrail.replay import kpi_evidence_grade
    g, why = kpi_evidence_grade(graded, reasons, pick, trust_store)
    if unbound:
        g, why = False, list(why) + [
            f"its replay bundle {pick.name} is not this run's evidence: "
            + "; ".join(unbound)]
    return g, why, _rel(pick)


TICK_BUDGET_MS = 100.0          # Safety Shield p6: "Monitor tick budget <= 100 ms (10 Hz)"


def _pct(vals: list[float], q: float) -> float | None:
    if not vals:
        return None
    s = sorted(vals)
    return round(s[min(len(s) - 1, max(0, math.ceil(q * len(s)) - 1))], 3)


def tick_timing(rows: list[dict]) -> dict | None:
    """What a flown log says about the 100 ms tick budget: the Shield's own
    filter() time per tick (`shield_ms`, logged since 2026-10-06) and the
    loop period between logged ticks (the end-to-end rate the rail achieved).
    Nearest-rank percentiles; None when the log has neither."""
    ms = [float(r["shield_ms"]) for r in rows
          if isinstance(r.get("shield_ms"), (int, float))
          and not isinstance(r.get("shield_ms"), bool)]
    ts = [float(r["t"]) for r in rows if isinstance(r.get("t"), (int, float))]
    per = [round(1000.0 * (b - a), 3) for a, b in zip(ts, ts[1:]) if b > a]
    if not ms and not per:
        return None
    return {"shield_ms": {"n": len(ms), "p50": _pct(ms, 0.5), "p99": _pct(ms, 0.99),
                          "max": round(max(ms), 3) if ms else None,
                          "over_budget": sum(1 for v in ms if v > TICK_BUDGET_MS)},
            "period_ms": {"n": len(per), "p50": _pct(per, 0.5), "p99": _pct(per, 0.99),
                          "max": max(per) if per else None,
                          "over_budget": sum(1 for v in per if v > TICK_BUDGET_MS)}}


def _arms_of(doc: dict) -> dict | None:
    """The arm record a stress-harness bundle stored (kpi.json `arm_record`),
    reduced to what the A/B and paraphrase sections need."""
    rec = doc.get("arm_record")
    if not isinstance(rec, dict):
        return None
    para = rec.get("paraphrase") if isinstance(rec.get("paraphrase"), dict) else {}
    return {"arm": rec.get("arm"), "prefix": rec.get("prefix"),
            "pilot_reads_text": rec.get("pilot_reads_text"),
            "paraphrase_id": para.get("paraphrase_id"), "backend": para.get("backend"),
            "cell": doc.get("id_cell") or doc.get("id"), "seed": doc.get("seed")}


def load_run(run: Path, by_hash: dict, by_id: dict, trust_store=None) -> dict:
    """Score one run folder. Returns an episode dict, or {"skipped": reason}."""
    log = run / "flight_log.jsonl"
    if not log.is_file():
        if (run / "kpi.json").is_file():
            return load_stored(run, trust_store)
        return {"id": run.name, "source": _rel(run),
                "skipped": "no flight_log.jsonl and no kpi.json"}
    rows = [json.loads(x) for x in log.read_text(encoding="utf-8").splitlines()
            if x.strip()]
    if not rows:
        return {"id": run.name, "source": _rel(run), "skipped": "empty flight_log.jsonl"}

    metrics = _read_json(run / "metrics.json")
    manifest = _read_json(run / "manifest.json")
    stored = _read_json(run / "kpi.json")
    if isinstance(stored.get("kpi"), dict) and stored.get("template"):
        # A stress-harness bundle written with --keep-logs: re-scored from its
        # own log, under its template and parameter cell (load_bundle_log).
        return load_bundle_log(run, rows, stored, manifest, trust_store)
    if isinstance(stored.get("kpi"), dict):
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
    # compute() judges theta on the basis the run recorded; on the one-period
    # reading a clamped tick needs the policy's caps to be sized
    # (rescore_kpis.policy_caps). None without a resolved policy.
    k = K.compute(rows, prios, m_c, caps=R.policy_caps(pol))
    priority_source = "recomputed against the resolved policy"
    if pol is None:
        for f in PRIORITY_FIELDS:
            k[f] = stored.get(f)
        priority_source = ("stored kpi.json, computed in flight against the real "
                           "policy" if stored else
                           "n/m: no resolvable policy and no stored kpi.json")
        if (not isinstance(k.get("repair_magnitude_by_priority"), dict)
                and isinstance(k.get("repair_magnitude_by_operator_class"), dict)):
            # The class table needs no priorities and stays; the priority
            # table was withheld, and the status says which half is missing.
            k["repair_magnitude_breakdown_status"] = "priorities_unresolved"
    k["null_hover_mission"] = K.null_hover_mission(rows, metrics)
    wt = K.worst_tick(rows, prios)
    if wt is not None and pol is None:
        wt["priorities_unresolved"] = True
    k["worst_tick"] = wt

    graded, reasons = (is_kpi_grade(manifest, metrics) if manifest
                       else (False, ["no manifest.json"]))
    manifest_grade = graded
    graded, reasons, ev_bundle = evidence_grade(run, graded, reasons, trust_store)
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
        # What the manifest alone said, before the replay bundle was checked
        # as evidence (evidence_grade), and the bundle that was checked.
        "kpi_grade_manifest": manifest_grade, "evidence_bundle": ev_bundle,
        "tick_timing": tick_timing(rows),
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
# a stress-harness episode bundle WITH its per-tick log (--keep-logs)
# --------------------------------------------------------------------------- #

def forecast_suffix(*docs) -> str:
    """"/forecast-<arm>" for an episode flown on a comparison forecast (the
    stress harness's labelled 3 s arm, experiments/sweep_scenarios.py
    --forecast-arms), "" for the grant's 5 s / 0.1 s. Without it a report over
    --publish bundles pooled the 3 s comparison with the grant rows. The arm
    is read from the KPI table (`forecast_arm`) or the extras' `forecast.arm`;
    a run id carrying "forecast-3s" says the same."""
    for d in docs:
        if not isinstance(d, dict):
            continue
        fc = d.get("forecast") if isinstance(d.get("forecast"), dict) else {}
        arm = d.get("forecast_arm") or (d.get("kpi") or {}).get("forecast_arm") or fc.get("arm")
        if arm and arm != "grant":
            return f"/forecast-{arm}"
        if "forecast-3s" in str(d.get("id") or ""):
            return "/forecast-3s"
    return ""


def _bundle_family(doc: dict, sid, params: dict, arm: str) -> str:
    template = doc.get("template") or str(sid).split("-")[0]
    fam = f"stress-bundle/{template}/{cell_name(sid, params)}/shield-{arm}"
    if doc.get("prefix") == "off":
        fam += "/prefix-off"
    return fam + forecast_suffix(doc)


def load_bundle_log(run: Path, rows: list[dict], doc: dict, manifest: dict,
                    trust_store=None) -> dict:
    """One episode bundle written by experiments/sweep_scenarios.py with
    --keep-logs: RE-SCORED from its own flight_log.jsonl with
    guardrail.kpi.compute (follow-up #37: until 2026-10-07 the profiles ran
    without logs, so the report could only use their stored tables).

    The priorities are the ones the harness recorded at the end of the episode
    (`rule_priorities`, hot-applied zones included), so a rule spawned in
    flight is not defaulted to P0 by a policy file that never held it; the
    metrics are the ones the harness passed to compute() (`metrics`: the goal,
    its self-caused breach count, the arm, the FSM's summary). Where the
    re-score and the stored table disagree on a headline field, the report
    says so."""
    k_stored = doc["kpi"]
    prios = doc.get("rule_priorities") or {}
    m = dict(doc.get("metrics") or {})
    arm = _arm(m.get("shield")) or _arm(doc.get("arm")) or "unknown"
    k = K.compute(rows, prios, m)
    for f in ("failsafe_triggered", "failsafe_matches_label", "mission_outcome",
              "outcome_matches_label", "breaches", "polygon_entries",
              "true_p0_flown_ticks", "min_true_dist_to_fence_m", "reached_goal"):
        if f in k_stored:
            k[f] = k_stored[f]
    k["null_hover_mission"] = K.null_hover_mission(rows, {"reached_goal": m.get("reached_goal")})
    k["worst_tick"] = K.worst_tick(rows, prios)
    sid = doc.get("scenario_id") or doc.get("id") or run.name
    params = doc.get("params") if isinstance(doc.get("params"), dict) else {}
    seed = doc.get("seed", manifest.get("random_seed"))
    graded, reasons = (is_kpi_grade(manifest, {}) if manifest
                       else (False, ["no manifest.json"]))
    manifest_grade = graded
    graded, reasons, ev_bundle = evidence_grade(run, graded, reasons, trust_store)
    diffs = {f: {"stored": k_stored[f], "recomputed": k.get(f)} for f in _COMPARE
             if f in k_stored and k_stored[f] != k.get(f)}
    arm_tag = ((doc.get("arm_record") or {}).get("arm") or "")
    k.update({
        "id": f"{cell_name(sid, params)}{'#' + arm_tag if arm_tag else ''}--seed{seed}",
        "scenario_id": sid, "source": _rel(run), "kind": "stress-log",
        "recomputed": True, "family": _bundle_family(doc, sid, params, arm),
        "arm": arm, "rail": RAIL.get(manifest.get("topology"),
                                     manifest.get("topology") or "no-manifest"),
        "kpi_grade": graded, "kpi_grade_reasons": reasons,
        "kpi_grade_manifest": manifest_grade, "evidence_bundle": ev_bundle,
        "kpi_grade_at_flight": doc.get("kpi_grade"),
        "code_revision": manifest.get("code_revision"),
        "time_to_safe_basis": "logged by the harness (true position)",
        "bundle": _rel(run), "arms": _arms_of(doc), "prefix": doc.get("prefix"),
        "params": {"template": doc.get("template"), "cell": params or None,
                   "seed": seed, "scenario_id": sid},
        "stored_vs_recomputed": diffs,
        "_dedupe": ("stress-log", sid, json.dumps(params, sort_keys=True), seed,
                    arm_tag, manifest.get("policy_hash"), manifest.get("code_revision")),
    })
    return k


# --------------------------------------------------------------------------- #
# a stress-harness episode bundle (stored table, no per-tick log)
# --------------------------------------------------------------------------- #

def load_stored(run: Path, trust_store=None) -> dict:
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
    manifest_grade = graded
    graded, reasons, ev_bundle = evidence_grade(run, graded, reasons, trust_store)
    goal = k.get("reached_goal")
    arm_tag = ((doc.get("arm_record") or {}).get("arm") or "")
    k.update({
        "id": f"{cell_name(sid, params)}{'#' + arm_tag if arm_tag else ''}--seed{seed}",
        "scenario_id": sid,
        "source": _rel(run), "kind": "stored-table", "recomputed": False,
        "source_note": "stored table, not recomputed (bundle has no flight_log.jsonl)",
        "family": _bundle_family(doc, sid, params, arm),
        "arm": arm, "rail": RAIL.get(manifest.get("topology"),
                                     manifest.get("topology") or "no-manifest"),
        "kpi_grade": graded, "kpi_grade_reasons": reasons,
        "kpi_grade_manifest": manifest_grade, "evidence_bundle": ev_bundle,
        "kpi_grade_at_flight": doc.get("kpi_grade"),
        "code_revision": manifest.get("code_revision"),
        "time_to_safe_basis": "logged by the harness (true position)",
        "arms": _arms_of(doc), "prefix": doc.get("prefix"),
        "null_hover_mission": (
            {"success": False, "basis": "declared goal; a hover does not travel"}
            if goal is not None else
            {"success": None, "basis": "no goal and no per-tick log"}),
        "worst_tick": None, "bundle": _rel(run),
        "params": {"template": template, "cell": params or None, "seed": seed,
                   "scenario_id": sid},
        "stored_vs_recomputed": {},
        "_dedupe": ("stored-table", sid, json.dumps(params, sort_keys=True), seed,
                    arm_tag, manifest.get("policy_hash"), manifest.get("code_revision")),
    })
    return k


# --------------------------------------------------------------------------- #
# the headless sweep (aggregated JSON, no per-tick log)
# --------------------------------------------------------------------------- #

def library_specs() -> dict[str, dict]:
    """scenario id -> its family as written, for labelling sweep results.

    Read through guardrail.scenario_spec.Library, because since 2026-10-07 the
    library is one file per template (experiments/templates/) and
    experiments/scenarios.yaml only lists them: the first version read
    scenarios.yaml's own `scenarios:` key, which is now empty. A library that
    will not load labels nothing rather than stopping the report; the old
    single-file form (an inline `scenarios:` list) is read as a fallback."""
    if not LIBRARY.is_file():
        return {}
    try:
        from guardrail.scenario_spec import Library
        return {f.scenario_id: f.raw for f in Library(LIBRARY).families}
    except Exception:                                            # noqa: BLE001
        try:
            specs = (yaml.safe_load(LIBRARY.read_text(encoding="utf-8")) or {}) \
                .get("scenarios", []) or []
        except yaml.YAMLError:
            return {}
        return {(s.get("scenario_id") or s.get("id")): s for s in specs
                if isinstance(s, dict) and (s.get("scenario_id") or s.get("id"))}


def load_sweep(path: Path) -> list[dict]:
    """Episodes from experiments/sweep_scenarios.py's aggregated JSON.

    It stores each scenario's KPI table but not its per-tick rows, so the
    per-tick KPIs - repair success, false triggers, theta - are NOT
    MEASURABLE from it and are left absent, which rollup() reports as such.
    Its stored `mission_success` ignored its own `reached_goal`; rollup()
    re-applies the goal through `kpi.mission_success_with_goal`.
    """
    d = _read_json(path)
    lib = library_specs()
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
            # The run id (cell plus arm, since 2026-10-07) when the result has
            # one, so two arms of one cell are two episodes, not one id twice.
            "id": r.get("id") or sid, "scenario_id": sid, "source": _rel(path),
            "kind": "sweep-json", "recomputed": False,
            "family": (f"sweep-json/{template}/{cell_name(sid, cell)}/shield-{arm}"
                       + ("/prefix-off" if r.get("prefix") == "off" else "")
                       + forecast_suffix(r, r.get("kpi"))),
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

# How a zero that could not have been anything else is printed (rollup()'s
# `vacuous_zeros`): named for what it is, never as a measured 0.
VACUOUS = {"p0_escape_rate": "vacuous (no P0 tick)",
           "false_trigger_rate": "vacuous (no FSM state logged)"}


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
    head = ("| Family | Episodes | P0 escape rate | P0-acted tick share | "
            "Fail-safe, labelled episodes | False triggers | Mission success | "
            "Mean repair count / ep | Mean repair magnitude (m/s) | Repair success | "
            "Mean time to safe (s) |")
    lines = [head, "|" + "---|" * 11]
    for name, s in fams.items():
        vac = s.get("vacuous_zeros") or {}
        ms = ("n/m" if s["mission_success_rate"] is None else
              f"{s['mission_successes']}/{s['episodes_mission_scored']}")
        ft = ("n/m" if s["false_trigger_rate"] is None else
              VACUOUS["false_trigger_rate"] if "false_trigger_rate" in vac else
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
        # The pooled tick share, under its own name (rollup()'s
        # `p0_acted_tick_share`): never the grant's fail-safe KPI, which is
        # the labelled column beside it.
        fsc = ("no P0 tick" if s["failsafe_expected_ticks"] == 0 else
               _f(s.get("p0_acted_tick_share"), 3))
        if "p0_escape_rate" in vac:
            esc = VACUOUS["p0_escape_rate"]
        else:
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
        vac = s.get("vacuous_zeros") or {}
        esc = (VACUOUS["p0_escape_rate"] if "p0_escape_rate" in vac
               else _f(s["p0_escape_rate"], 4))
        nul = (VACUOUS["p0_escape_rate"] if "null_passthrough_p0_escape_rate" in vac
               else _f(s["null_passthrough_p0_escape_rate"], 4))
        lines.append(
            f"| `{name}` | {esc} / {nul} | {_rs(s)} / "
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


# --------------------------------------------------------------------------- #
# the sections beyond the per-family table (2026-10-12)
# --------------------------------------------------------------------------- #

DEFAULT_WP1 = "docs/data/wp1_roundtrip.json"
DEFAULT_CSP_JSON = "docs/data/csp_kpis.json"
DEFAULT_TICK_PROFILES = "deploy/evidence/*/shield_tick_profile*.json"
WP2_SCAN_STEP = 8               # tokens between the budgets the WP2 scan tries
WP2_SCAN_FLOOR = 16


def wp1_section(path: Path) -> dict | None:
    """WP1's acceptance ("policy load round-trip; bundle replayability") as
    tools/wp1_roundtrip_kpi.py measured it: its `kpi` block and every null
    block (`null_*`), whatever keys they hold. None without the file."""
    d = _read_json(path)
    if not d:
        return None
    nulls = {k: v for k, v in d.items() if k.startswith("null") and isinstance(v, dict)}
    return {"source": _rel(path), "generated": d.get("generated"),
            "command": d.get("command"), "code_revision": d.get("code_revision"),
            "kpi": d.get("kpi") if isinstance(d.get("kpi"), dict) else {},
            "nulls": nulls}


def _scan_row(budget: int, rep: dict) -> dict:
    t = rep["totals"]
    return {"budget_tokens": budget, "n_compiled": t["n_compiled"],
            "n_raised_budget_exceeded": t["n_raised_budget_exceeded"],
            "n_p0_in_scope": t["n_p0_in_scope"], "n_p0_covered": t["n_p0_covered"],
            "p0_coverage": t["p0_coverage"],
            "baseline_n_p0_covered": t["baseline_n_p0_covered"],
            "baseline_p0_coverage": t["baseline_p0_coverage"],
            "null_p0_coverage": t["null_p0_coverage"],
            "kpi_discriminates": t["kpi_discriminates"],
            "max_tokens_used": t["max_tokens_used"]}


def wp2_section(policies_dir: str | Path, budget: int | None = None,
                step: int = WP2_SCAN_STEP, floor: int = WP2_SCAN_FLOOR) -> dict:
    """The Prefix Compiler's KPIs (guardrail.kpi.compiler_kpis) at the default
    budget, AND at the largest budget where the naive cut drops a P0 rule.

    At the default 256 tokens every policy fits whole, so the naive baseline
    also covers every P0 rule and `kpi_discriminates` is False: the 100 %
    shows only that nothing had to be cut. The scan walks the budget down by
    `step` tokens until the baseline (guardrail.compiler.naive_prefix_baseline,
    no priority and no risk) first loses a P0 rule the compiler keeps, and
    reports both there, with the policies whose P0 rules the naive cut lost.
    `discriminating` is None when no budget down to `floor` separates them."""
    from guardrail.compiler import DEFAULT_BUDGET_TOKENS, coverage_report
    b0 = int(budget or DEFAULT_BUDGET_TOKENS)
    pdir = Path(policies_dir) if Path(policies_dir).is_absolute() else ROOT / policies_dir
    scan, chosen = [], None
    for b in range(b0, floor - 1, -step):
        rep = coverage_report(pdir, b)
        scan.append(_scan_row(b, rep))
        if rep["totals"]["kpi_discriminates"]:
            chosen = (b, rep)
            break
    out = {"policies_dir": _rel(pdir), "default": K.compiler_kpis(pdir, b0),
           "scan": scan, "scan_step_tokens": step, "scan_floor_tokens": floor,
           "discriminating": None}
    if chosen is not None:
        b, rep = chosen
        lost = [{"file": r["file"], "policy_id": r["policy_id"],
                 "baseline_p0_missing": r["baseline_p0_missing"],
                 "compiler_p0_missing": r["p0_missing"]}
                for r in rep["rows"] if r["baseline_p0_missing"]]
        out["discriminating"] = {**K.compiler_kpis(pdir, b),
                                 "policies_where_the_naive_cut_drops_p0": lost,
                                 "raised": [{"file": r["file"], "p0_tokens": r["p0_tokens"]}
                                            for r in rep["raised"]]}
    return out


def tick_budget_section(profile_glob: str, episodes: list[dict]) -> dict:
    """The Safety Shield's 100 ms tick budget and the Policy DSL's 5 ms query
    budget: the stored profiles of tools/profile_shield_tick.py (attach_summary
    of each), and the timing the flown logs carry (tick_timing), per rail.
    A profile is evidence for the grant's budget only on a Jetson Orin in the
    hil topology; every other host's is a baseline."""
    profiles = []
    pattern = profile_glob if Path(profile_glob).is_absolute() else str(ROOT / profile_glob)
    paths = sorted(glob.glob(pattern))
    if paths:
        import profile_shield_tick as P
        for p in paths:
            prof = _read_json(Path(p))
            if prof:
                profiles.append(P.attach_summary(prof, Path(p)))
    rails: dict[str, dict] = {}
    for e in episodes:
        tt = e.get("tick_timing")
        if not isinstance(tt, dict):
            continue
        # Episodes are counted per measure: a log with periods but no
        # `shield_ms` (flown before 2026-10-06) is behind the period columns,
        # not behind the Shield's, and counting it there would overstate how
        # many flights the Shield's p99 rests on.
        r = rails.setdefault(e.get("rail") or "unknown", {
            "episodes_with_timing": 0, "shield_episodes": 0, "shield_ticks": 0,
            "shield_worst_p99_ms": None, "shield_max_ms": None, "shield_over_budget": 0,
            "period_episodes": 0, "period_ticks": 0, "period_worst_p50_ms": None,
            "period_worst_p99_ms": None, "period_over_budget": 0})
        p50 = (tt.get("period_ms") or {}).get("p50")
        if p50 is not None:
            r["period_worst_p50_ms"] = max(p50, r["period_worst_p50_ms"] or 0.0)
        r["episodes_with_timing"] += 1
        for src, pre in (("shield_ms", "shield"), ("period_ms", "period")):
            c = tt.get(src) or {}
            if not c.get("n"):
                continue
            r[f"{pre}_episodes"] += 1
            r[f"{pre}_ticks"] += c["n"]
            r[f"{pre}_over_budget"] += c.get("over_budget") or 0
            if c.get("p99") is not None:
                r[f"{pre}_worst_p99_ms"] = max(c["p99"], r[f"{pre}_worst_p99_ms"] or 0.0)
            if pre == "shield" and c.get("max") is not None:
                r["shield_max_ms"] = max(c["max"], r["shield_max_ms"] or 0.0)
    return {"budget_ms": TICK_BUDGET_MS, "profiles": profiles, "flown_by_rail": rails}


def arms_sections(episodes: list[dict]) -> tuple[dict, dict]:
    """The Prefix Compiler's A/B (prefix on / off, same seed: "the KPI report
    shows the delta", Prefix Compiler p6) and WP4's per-paraphrase robustness.
    Measured only over episodes whose pilot READS the prompt
    (`pilot_reads_text`): a headless pilot ignores text, so its arms are one
    flight by construction and their pairs are plumbing checks. Until such
    episodes are flown both sections are n/m, with the headless count beside."""
    armed = [e for e in episodes if isinstance(e.get("arms"), dict)]
    reads = [e for e in armed if e["arms"].get("pilot_reads_text") is True]
    blind = [e for e in armed if e["arms"].get("pilot_reads_text") is not True]

    def pairs(eps):
        g: dict[tuple, dict] = {}
        for e in eps:
            a = e["arms"]
            g.setdefault((a.get("cell"), a.get("seed"), a.get("paraphrase_id")),
                         {}).setdefault(a.get("prefix"), []).append(e)
        return [v for v in g.values() if "on" in v and "off" in v]

    def rate(eps):
        v = [K.mission_success_with_goal(e) for e in eps]
        v = [x for x in v if x is not None]
        return (round(sum(v) / len(v), 6), len(v)) if v else (None, 0)

    flown, headless = pairs(reads), pairs(blind)
    if flown:
        on = [e for p in flown for e in p["on"]]
        off = [e for p in flown for e in p["off"]]
        (r_on, n_on), (r_off, n_off) = rate(on), rate(off)
        ab = {"status": "measured", "pairs": len(flown),
              "mission_success_prefix_on": r_on, "scored_on": n_on,
              "mission_success_prefix_off": r_off, "scored_off": n_off,
              "delta_on_minus_off": (round(r_on - r_off, 6)
                                     if r_on is not None and r_off is not None else None),
              "p0_escapes_prefix_on": sum(e.get("p0_escapes") or 0 for e in on),
              "p0_escapes_prefix_off": sum(e.get("p0_escapes") or 0 for e in off),
              "headless_pairs": len(headless)}
    else:
        ab = {"status": "n/m", "pairs": 0, "headless_pairs": len(headless),
              "why": "no prefix on/off pair has been flown by a pilot that reads "
                     "the prompt; the headless pilots ignore text, so their pairs "
                     "are plumbing checks, not an A/B"}
    trials = [{"paraphrase_id": e["arms"]["paraphrase_id"],
               "backend": e["arms"].get("backend"),
               "mission_success": K.mission_success_with_goal(e)}
              for e in reads if e["arms"].get("paraphrase_id")]
    n_blind = sum(1 for e in blind if e["arms"].get("paraphrase_id"))
    if trials:
        para = {"status": "measured", "trials": len(trials), "headless_trials": n_blind,
                **K.per_paraphrase_robustness(trials)}
    else:
        para = {"status": "n/m", "trials": 0, "headless_trials": n_blind,
                "why": "no paraphrase arm has been flown by a pilot that reads the "
                       "prompt; per-paraphrase robustness needs one"}
    return ab, para


# Family figures the baseline comparison reports (rollup() keys).
BASELINE_FIELDS = ("episodes", "p0_escape_rate", "p0_escapes", "mission_success_rate",
                   "mean_repair_count_per_episode", "mean_repair_magnitude_mps",
                   "repair_success_rate", "mean_time_to_safe_s")


def failure_index(episodes: list[dict], include_controls: bool = False) -> dict[str, dict]:
    """Every failing episode (guardrail.kpi.failure_categories non-empty),
    keyed `<family>::<id>`; Shield-off control arms only when asked for, as in
    the top-K table. Written into the report JSON as `failures`, so a later
    run can compare against it (--baseline)."""
    out = {}
    for e in episodes:
        if not include_controls and e.get("arm") == "off":
            continue
        cats = K.failure_categories(e)
        if cats:
            out[f"{e.get('family')}::{e.get('id')}"] = {
                "id": e.get("id"), "family": e.get("family"),
                "failure_category": cats, "kpi_grade": bool(e.get("kpi_grade"))}
    return out


def _flat_families(rep: dict) -> dict[str, dict]:
    out = {}
    for group in ("kpi_grade", "kpi_grade_at_flight_only", "other"):
        for name, row in ((rep.get("families") or {}).get(group) or {}).items():
            out.setdefault(name, dict(row, grade_group=group))
    return out


def compare_baseline(now: dict, base: dict, base_path: str,
                     include_controls: bool = False) -> dict:
    """Against an earlier report JSON (--baseline): failures that are NEW
    (failing now, not failing or absent then), FIXED (failing then, present
    and not failing now), GONE (failing then, absent now - not counted as
    fixed), and those failing in both whose categories changed; and per
    family the change in each BASELINE_FIELDS figure. A baseline written
    before `failures` existed is read from its `episodes`."""
    if isinstance(base.get("failures"), dict):
        before = base["failures"]
    else:
        before = failure_index(base.get("episodes") or [], include_controls)
    after = now["failures"]
    present = {f"{e.get('family')}::{e.get('id')}" for e in now.get("episodes") or []}
    had = ({f"{e.get('family')}::{e.get('id')}" for e in base.get("episodes") or []}
           | set(before))
    # `was`: "passing" - a regression of an episode the baseline had and did
    # not count as failing; "absent" - an episode the baseline did not have.
    new = [dict(v, key=k, was="passing" if k in had else "absent")
           for k, v in sorted(after.items()) if k not in before]
    fixed = [dict(v, key=k) for k, v in sorted(before.items())
             if k not in after and k in present]
    gone = [dict(v, key=k) for k, v in sorted(before.items())
            if k not in after and k not in present]
    changed = [{"key": k, "id": after[k]["id"], "family": after[k]["family"],
                "baseline": before[k]["failure_category"],
                "now": after[k]["failure_category"]}
               for k in sorted(set(before) & set(after))
               if before[k]["failure_category"] != after[k]["failure_category"]]
    fa, fb = _flat_families(now), _flat_families(base)
    deltas = {}
    for name in sorted(set(fa) & set(fb)):
        row = {}
        for f in BASELINE_FIELDS:
            a, b = K._n(fb[name].get(f)), K._n(fa[name].get(f))
            if a is None and b is None:
                continue
            if a is not None and b is not None and abs(a - b) <= 1e-12:
                continue
            row[f] = {"baseline": a, "now": b,
                      "delta": (round(b - a, 6) if a is not None and b is not None
                                else None)}
        if row:
            deltas[name] = row
    return {"path": base_path, "generated": base.get("generated"),
            "code_revision": base.get("code_revision"),
            "new_failures": new, "fixed_failures": fixed, "gone_failures": gone,
            "changed_failures": changed, "family_deltas": deltas,
            "families_added": sorted(set(fa) - set(fb)),
            "families_removed": sorted(set(fb) - set(fa))}


def _cell(v) -> str:
    if v is None:
        return "-"
    if isinstance(v, list):
        return ", ".join(str(x) for x in v) if v else "none"
    if isinstance(v, dict):
        return json.dumps(v, sort_keys=True)
    return str(v)


def _render_wp1(w: dict | None) -> list[str]:
    L = ["## WP1 - policy load round trip and bundle replayability", ""]
    if not w:
        return L + [f"n/m: no `{DEFAULT_WP1}` (run `python tools/wp1_roundtrip_kpi.py`).", ""]
    L += [f"From `{w['source']}` (`{w.get('command')}`, generated {w.get('generated')} at "
          f"`{w.get('code_revision')}`), as it stands; every figure it holds, each null "
          f"beside it.", ""]
    names = list(w["kpi"])
    for nb in w["nulls"].values():
        names += [k for k in nb if k != "why" and k not in names]
    cols = list(w["nulls"])
    L += ["| Figure | Measured | " + " | ".join(f"`{c}`" for c in cols) + " |",
          "|---|---|" + "---|" * len(cols)]
    for n in names:
        L.append(f"| `{n}` | {_cell(w['kpi'].get(n))} | "
                 + " | ".join(_cell(w["nulls"][c].get(n)) for c in cols) + " |")
    L.append("")
    for c in cols:
        if w["nulls"][c].get("why"):
            L.append(f"- `{c}`: {w['nulls'][c]['why']}")
    return L + [""]


def _render_wp2(w: dict | None) -> list[str]:
    L = ["## WP2 - Prefix Compiler KPIs", ""]
    if not w:
        return L + [f"(not computed, and no `{DEFAULT_CSP_JSON}` to read: `--wp2` "
                    "computes it for this report; `--wp2-only` computes it and writes "
                    "that file)", ""]
    if w.get("read_from"):
        L += [f"Read from `{w['read_from']}` (generated {w.get('generated')} at "
              f"`{w.get('code_revision')}` by `{w.get('command')}`), not recomputed by "
              "this report; `--wp2` recomputes it.", ""]

    def line(k: dict, label: str) -> str:
        disc = k.get("kpi_discriminates")
        return (f"- {label}: P0 coverage {_f(k.get('p0_coverage'), pct=True)} over "
                f"{k.get('n_compiled')} compiled of {k.get('n_policies')} policies; null "
                f"{_f(k.get('p0_coverage_null'), pct=True)}; naive baseline "
                f"{_f(k.get('p0_coverage_baseline'), pct=True)}; "
                f"{k.get('csp_budget_exceeded')} raised CSPBudgetExceeded; largest CSP "
                f"{k.get('max_tokens_used')} tokens. "
                + ("The naive cut scores the same, so the figure shows only that "
                   "nothing had to be cut." if disc is False else
                   "The KPI separates the compiler from the naive cut." if disc
                   else "Nothing compiled: n/m."))
    d = w["default"]
    L += [line(d, f"At the default {d['budget_tokens']} tokens")]
    c = w.get("discriminating")
    if c:
        L += [line(c, f"At {c['budget_tokens']} tokens, the largest budget (scanned down "
                      f"in steps of {w['scan_step_tokens']}) where the naive cut drops a "
                      f"P0 rule")]
        for p in c["policies_where_the_naive_cut_drops_p0"]:
            L.append(f"  - `{p['file']}`: the naive cut drops "
                     f"{', '.join(p['baseline_p0_missing'])}; the compiler drops "
                     f"{', '.join(p['compiler_p0_missing']) or 'none'}")
    else:
        L += [f"- No budget down to {w['scan_floor_tokens']} tokens separates the "
              f"compiler from the naive cut: n/m."]
    L += ["", "| Budget | Compiled | Raised | P0 coverage | Naive baseline | Null | "
              "Discriminates |", "|---|---|---|---|---|---|---|"]
    for r in w["scan"]:
        L.append(f"| {r['budget_tokens']} | {r['n_compiled']} | "
                 f"{r['n_raised_budget_exceeded']} | {r['n_p0_covered']}/{r['n_p0_in_scope']} | "
                 f"{r['baseline_n_p0_covered']}/{r['n_p0_in_scope']} | "
                 f"{_f(r['null_p0_coverage'], pct=True)} | {_f(r['kpi_discriminates'])} |")
    return L + [""]


def _render_tick_budget(t: dict | None) -> list[str]:
    L = ["## Tick budget", "",
         "Safety Shield p6: \"Monitor tick budget <= 100 ms (10 Hz) end-to-end\"; Policy "
         "DSL: monitor query <= 5 ms with 50 active rules. Evidence for the grant's "
         "budget only from a Jetson Orin in the hil topology; any other host is a "
         "baseline.", ""]
    if not t:
        return L + ["(not computed)", ""]
    if t["profiles"]:
        L += ["| Profile | Topology | Orin | Query p99 / budget (ms) | Shield step p99 / "
              "budget (ms) | Within budget |", "|---|---|---|---|---|---|"]
        for p in t["profiles"]:
            within = {True: "yes", False: "no", None: "undetermined"}[p.get("within_budget")]
            L.append(f"| `{p.get('path')}` | {p.get('topology')} | "
                     f"{'yes' if p.get('host_is_orin') else 'no'} | "
                     f"{_f(p.get('check_p99_ms'), 2)} / {_f(p.get('check_budget_ms'), 0)} | "
                     f"{_f(p.get('filter_p99_ms'), 2)} / {_f(p.get('filter_budget_ms'), 0)} | "
                     f"{within} |")
        L.append("")
    else:
        L += ["No stored profile (`python tools/profile_shield_tick.py profile`): n/m.", ""]
    if t["flown_by_rail"]:
        L += ["Flown logs (the Shield's own `shield_ms` per tick, and the period between "
              "logged ticks):", "",
              "| Rail | Shield episodes | Shield ticks | Worst p99 (ms) | Max (ms) | "
              "Over 100 ms | Period episodes | Period ticks | Worst median period (ms) | "
              "Worst period p99 (ms) | Periods over 100 ms |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
        for rail, r in sorted(t["flown_by_rail"].items()):
            L.append(f"| {rail} | {r['shield_episodes']} | {r['shield_ticks']} | "
                     f"{_f(r['shield_worst_p99_ms'], 2)} | {_f(r['shield_max_ms'], 2)} | "
                     f"{r['shield_over_budget'] if r['shield_ticks'] else 'n/m'} | "
                     f"{r['period_episodes']} | "
                     f"{r['period_ticks']} | {_f(r['period_worst_p50_ms'], 1)} | "
                     f"{_f(r['period_worst_p99_ms'], 1)} | "
                     f"{r['period_over_budget'] if r['period_ticks'] else 'n/m'} |")
        L += ["", "Periods are read from the logged `t`, rounded to 1 ms: a loop at "
                  "exactly 10 Hz logs periods of 100 and 101 ms, so the count over "
                  "100 ms is read beside the median period, not alone."]
        L.append("")
    return L


def _render_arms(ab: dict | None, para: dict | None) -> list[str]:
    L = ["## A/B: prefix on / off", ""]
    if not ab or ab["status"] != "measured":
        L += [f"n/m - {(ab or {}).get('why', 'not computed')} "
              f"({(ab or {}).get('headless_pairs', 0)} headless pairs).", ""]
    else:
        L += [f"- {ab['pairs']} pairs: mission success prefix on "
              f"{_f(ab['mission_success_prefix_on'], pct=True)} ({ab['scored_on']} scored), "
              f"off {_f(ab['mission_success_prefix_off'], pct=True)} ({ab['scored_off']}); "
              f"delta {_f(ab['delta_on_minus_off'])}; P0 escapes on "
              f"{ab['p0_escapes_prefix_on']}, off {ab['p0_escapes_prefix_off']}.", ""]
    L += ["## Per-paraphrase robustness", ""]
    if not para or para["status"] != "measured":
        L += [f"n/m - {(para or {}).get('why', 'not computed')} "
              f"({(para or {}).get('headless_trials', 0)} headless trials).", ""]
    else:
        L += [f"- {para['trials']} trials: canonical "
              f"{_f(para.get('canonical_success_rate'), pct=True)} over "
              f"{para.get('canonical_trials')}; {para.get('paraphrase_arms')} paraphrase "
              f"arms, min {_f(para.get('paraphrase_min_success_rate'), pct=True)}, worst "
              f"drop {_f(para.get('worst_drop_vs_canonical'))}.", ""]
    return L


def _render_magnitudes(groups: dict[str, dict]) -> list[str]:
    classes = list(K.OPERATOR_CLASSES)
    # An operator kpi.OPERATOR_CLASSES does not list is counted Unclassified;
    # its column is printed whenever any family has such a tick, so a repair
    # by an operator nobody classed is never hidden.
    if any(((s.get("mean_repair_magnitude_by_operator_class") or {})
            .get(K.UNCLASSIFIED) or {}).get("ticks")
           for g in groups.values() for s in (g or {}).values()):
        classes.append(K.UNCLASSIFIED)
    L = ["## Mean repair magnitude by priority and by operator class", "",
         "|emitted - raw| (m/s) on repaired ticks, pooled. A tick counts under the "
         "highest priority it violated, and under every operator class that acted on "
         "it (the class's ticks in brackets). `never` = no repaired tick of that "
         "priority or class: it never acted, which is not a 0. The grant's classes "
         "(Safety Shield p3) are AltitudeClamp, LateralProjection and PathRepair; the "
         "others (*) are this Shield's extensions"
         + (f", and {K.UNCLASSIFIED} an operator no class lists" if K.UNCLASSIFIED in classes
            else "") + ".", "",
         "| Family | " + " | ".join(K.PRIORITY_LEVELS) + " | unattributed | "
         + " | ".join(c if c in (*K.GRANT_OPERATOR_CLASSES, K.UNCLASSIFIED) else c + "*"
                      for c in classes)
         + " |", "|---|" + "---|" * (len(K.PRIORITY_LEVELS) + 1 + len(classes))]

    def cell(c: dict | None) -> str:
        if not c or not c.get("ticks"):
            return "never"
        return f"{_f(c.get('mean_mps'), 3)} ({c['ticks']})"
    n = 0
    for key in ("kpi_grade", "kpi_grade_at_flight_only", "other"):
        for name, s in (groups.get(key) or {}).items():
            lv, cl = s.get("mean_repair_magnitude_by_priority"), \
                s.get("mean_repair_magnitude_by_operator_class")
            if lv is None and cl is None:
                continue
            n += 1
            # One half without the other: the class table needs no
            # priorities, so a family whose policy did not resolve keeps it.
            L.append(f"| `{name}` | " + " | ".join(
                cell(lv.get(k)) if lv is not None else "n/m (priorities unresolved)"
                for k in (*K.PRIORITY_LEVELS, "unattributed"))
                + " | " + " | ".join(cell(cl.get(c)) if cl is not None else "n/m"
                                     for c in classes) + " |")
    if not n:
        L.append("| (no family carries the breakdown: stored tables and Shield-off arms "
                 "do not) |" + " |" * (len(K.PRIORITY_LEVELS) + 1 + len(classes)))
    return L + [""]


def _render_baseline(b: dict | None) -> list[str]:
    if not b:
        return []
    L = [f"## Against the baseline `{b['path']}`", "",
         f"Baseline generated {b.get('generated')} at `{b.get('code_revision')}`. "
         f"{len(b['new_failures'])} new failures ("
         f"{sum(1 for r in b['new_failures'] if r.get('was') == 'passing')} of episodes "
         f"the baseline had and did not count as failing, "
         f"{sum(1 for r in b['new_failures'] if r.get('was') == 'absent')} of episodes it "
         f"did not have), {len(b['fixed_failures'])} fixed, "
         f"{len(b['gone_failures'])} no longer in the inputs (not counted as fixed), "
         f"{len(b['changed_failures'])} still failing for other reasons.", ""]
    for title, rows in (("New failures: regressions",
                         [r for r in b["new_failures"] if r.get("was") == "passing"]),
                        ("New failures: episodes the baseline did not have",
                         [r for r in b["new_failures"] if r.get("was") != "passing"]),
                        ("Fixed failures", b["fixed_failures"]),
                        ("Gone (absent now, not fixed)", b["gone_failures"])):
        if rows:
            L += [f"### {title}", "", "| Episode | Family | Failure category |",
                  "|---|---|---|"]
            L += [f"| `{r['id']}` | `{r['family']}` | {', '.join(r['failure_category'])} |"
                  for r in rows] + [""]
    if b["changed_failures"]:
        L += ["### Still failing, other reasons", "", "| Episode | Baseline | Now |",
              "|---|---|---|"]
        L += [f"| `{r['id']}` | {', '.join(r['baseline'])} | {', '.join(r['now'])} |"
              for r in b["changed_failures"]] + [""]
    if b["family_deltas"]:
        L += ["### Per-family deltas", "", "| Family | Figure | Baseline | Now | Delta |",
              "|---|---|---|---|---|"]
        for name, row in b["family_deltas"].items():
            for f, v in row.items():
                L.append(f"| `{name}` | `{f}` | {_f(v['baseline'], 4)} | {_f(v['now'], 4)} | "
                         f"{_f(v['delta'], 4)} |")
        L.append("")
    for title, names in (("Families added", b["families_added"]),
                         ("Families removed", b["families_removed"])):
        if names:
            L += [f"{title}: " + ", ".join(f"`{n}`" for n in names[:40])
                  + (" ..." if len(names) > 40 else ""), ""]
    return L


def render_markdown(rep: dict) -> str:
    L = [f"# KPI rollup - {rep['generated']}", "",
         f"Generated by `tools/kpi_report.py` at code revision "
         f"`{rep['code_revision']}`. Reproduce with:", "",
         "```", rep["command"], "```", "",
         "Flown episodes are re-scored from their own log with "
         "`guardrail.kpi.compute` (the function the flights use), and so are "
         "stress-harness bundles that kept their log (`--keep-logs`); older "
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
          f"{c.get('stress_logs', 0)} stress-harness bundles re-scored from their "
          f"log, {c['stored_tables']} stress-harness bundles used as stored, "
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
        vac = sorted({w for s in fams.values()
                      for w in (s.get("vacuous_zeros") or {}).values()})
        if vac:
            L += ["Vacuous zeros in this table (a 0 that could not have been anything "
                  "else, so it is not printed as one):", ""] + [f"- {w}" for w in vac] + [""]

    L += _render_magnitudes(rep["families"])

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

    L += _render_wp1(rep.get("wp1"))
    L += _render_wp2(rep.get("wp2"))
    L += _render_tick_budget(rep.get("tick_budget"))
    L += _render_arms(rep.get("ab"), rep.get("paraphrase"))
    L += _render_baseline(rep.get("baseline"))

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


def render_sweep_markdown(doc: dict, top_k: int = 10) -> str:
    """The grant's KPI report for ONE stress-harness run (Stress Testing p6):
    "1. Per-scenario-family stats table. One row per (scenario template,
    parameter cell) tuple ... 2. Top-K failure cases", in Markdown, from the
    results document experiments/sweep_scenarios.py writes (its
    `_cell_rollup`: guardrail.kpi.rollup per (template, cell, Shield arm,
    prefix arm), seeds and paraphrase arms pooled in the row).

    The same columns and nulls as the cross-run report above (`_family_table`,
    `_null_table`), the labelled fail-safe correctness with its nulls, the
    outcome counts, the arms' plumbing check, and the top-K with the episode
    bundle each row came from. The headless rail is never KPI-grade, and the
    page says so first."""
    cells = doc.get("_cell_rollup") or {}
    scope = doc.get("_scope") or {}
    c = doc.get("_counts") or {}
    fsl = doc.get("_failsafe_labels") or {}
    man = doc.get("_manifest_common") or {}
    L = [f"# Stress-harness KPI report - {date.today().isoformat()}", "",
         f"Generated by `experiments/sweep_scenarios.py` at code revision "
         f"`{man.get('code_revision')}`. Reproduce with:", "",
         "```", str(doc.get("_command") or "python experiments/sweep_scenarios.py"),
         "```", ""]
    if doc.get("_not_kpi_grade"):
        L += [f"**Not KPI-grade:** {doc['_not_kpi_grade']}. The grant's smoke and "
              f"nightly rows ask for sim_speedup 1.0 in the hil topology; this run "
              f"reproduces their scope, not their rail.", ""]
    prof = scope.get("profile")
    arms = scope.get("arms") or {}
    L += ["## Run", "",
          f"- Profile: {prof or 'none (the library on its declared seeds)'}"
          + (f" - grant scope {scope.get('grant_scope')!r}, cadence "
             f"{scope.get('cadence')!r}" if prof else ""),
          f"- {scope.get('cells')} scenario cells, {scope.get('episodes')} episodes"
          + (f" ({arms.get('extra_arm_episodes')} of them extra arms: paraphrase "
             f"{arms.get('paraphrase_arm_episodes')}, prefix off "
             f"{arms.get('prefix_off_episodes')})" if arms.get("extra_arm_episodes")
             else ""),
          f"- Status: {c.get('pass', 0)} pass, {c.get('tracked', 0)} tracked, "
          f"{c.get('fail', 0)} fail, {c.get('known_failure', 0)} known failure, "
          f"{c.get('skipped', 0)} skipped; runtime {doc.get('_runtime_s')} s",
          "- `n/m` = not measurable (an empty denominator, or a log that cannot "
          "answer); never a zero.", ""]
    L += ["## Per-scenario-family stats", "",
          "One row per (scenario template, parameter cell) and Shield arm "
          "(`<template>/<cell>/shield-<arm>`, `/prefix-off` for the prefix-off "
          "arm); seeds and paraphrase arms of one cell pool in its row.", ""]
    if cells:
        L += _family_table(cells) + ["", "Nulls beside the scores:", ""]
        L += _null_table(cells) + [""]
    else:
        L += ["(no scored episode)", ""]
    if fsl.get("labelled"):
        fs = fsl.get("fsm_scoring") or {}
        L += ["## Fail-safe trigger correctness (labelled episodes)", "",
              f"- The grant's \"triggered when expected, not when not expected\": "
              f"{fsl['correct']}/{fsl['labelled']} = {fsl['fail_safe_correctness']} "
              f"({fsl['false_triggers']} false triggers, {fsl['missed_triggers']} "
              f"missed; {fsl['unlabelled']} episodes unlabelled). Trigger = "
              f"{fsl.get('observed_as')}.",
              f"- Nulls: never trigger {fsl['null_never_trigger']}, always trigger "
              f"{fsl['null_always_trigger']}. Above both: "
              f"{_f(fsl.get('beats_never_trigger'))} / "
              f"{_f(fsl.get('beats_always_trigger'))}.",
              f"- Grant target >= {fsl.get('grant_target')}: met "
              f"{_f(fsl.get('meets_grant_target'))}; Wilson 95 % lower bound "
              f"{_f(fs.get('wilson_low_95'))}; error-free episodes needed to show "
              f"the target at 95 %: {fs.get('min_error_free_episodes_for_target_at_95')}"
              f"; labels discriminating: {_f(fsl.get('discriminating'))}.", ""]
    ts = doc.get("_true_state") or {}
    if ts:
        # The P0 escape rate above is the Shield's counter over the ticks the
        # Shield flew; what the modelled LOITER / RTL / LAND flew is outside
        # it by definition and is stated here, never left out of the page
        # (review of stress-harness-2).
        L += ["## True state, and what the autopilot flew", "",
              f"- Shield-flown: {ts.get('episodes_entering_polygon')} shield-on "
              f"episode(s) flew themselves into a P0 polygon "
              f"{ts.get('episodes_entering_polygon_by_status') or ''}; "
              f"{ts.get('episodes_inside_polygon_imposed_only')} more were inside "
              f"only because a zone appeared under them.",
              f"- Autopilot-flown (modelled LOITER / RTL / LAND, outside the P0 "
              f"escape rate): {ts.get('autopilot_p0_flown_ticks')} of "
              f"{ts.get('autopilot_ticks')} ticks flew a P0-violating action, in "
              f"{ts.get('episodes_autopilot_p0_flown')} episode(s) "
              f"{ts.get('episodes_autopilot_p0_flown_by_status') or ''}; "
              f"{ts.get('autopilot_polygon_entries')} polygon entr(y/ies) in "
              f"{ts.get('episodes_autopilot_entering_polygon')} episode(s); "
              f"{ts.get('autopilot_breaches')} breach(es) in "
              f"{ts.get('episodes_autopilot_breach')} episode(s).", ""]
    oc = (doc.get("_outcome_labels") or {}).get("outcomes") or {}
    if oc:
        L += ["## Outcomes", "",
              "| " + " | ".join(oc) + " |", "|" + "---|" * len(oc),
              "| " + " | ".join(str(v) for v in oc.values()) + " |", ""]
    ad = doc.get("_arms")
    if ad:
        pp = ad.get("per_paraphrase") or {}
        L += ["## Arms (paraphrase, prefix): plumbing only", "",
              f"- {ad['episodes_with_arms']} episodes in {ad['cell_seed_groups']} "
              f"(cell, seed) groups; groups whose arms flew differently: "
              f"{len(ad['groups_whose_arms_flew_differently'])}. Prefix on/off pairs: "
              f"{ad['prefix_ab_pairs']}, differing: "
              f"{len(ad['prefix_ab_pairs_that_differ'])}.",
              f"- Per-paraphrase success: canonical "
              f"{_f(pp.get('canonical_success_rate'))} over "
              f"{pp.get('canonical_trials')} trials; paraphrase arms "
              f"{pp.get('paraphrase_arms')}, min {_f(pp.get('paraphrase_min_success_rate'))}"
              f", worst drop {_f(pp.get('worst_drop_vs_canonical'))}.",
              f"- {ad['note']}", ""]
    top = (doc.get("_top_failures") or [])[:top_k]
    L += [f"## Top-{top_k} failure cases", "",
          "Ranked by guardrail.kpi.top_failures (P0 escapes first). Shield-off "
          "control arms are left out.", ""]
    if top:
        L += ["| # | Episode | Template | Parameters | Failure category | "
              "Raw action (vx, vy, vz_up, yaw_rate) | Repaired action | t (s) | Bundle |",
              "|---|---|---|---|---|---|---|---|---|"]
        for i, f in enumerate(top, 1):
            params = f.get("parameters") or {}
            pt = ",".join(f"{a}={b}" for a, b in sorted(params.items())) or "-"
            L.append(f"| {i} | `{f.get('scenario_id')}` | `{f.get('family')}` | {pt} | "
                     f"{', '.join(f.get('failure_category') or [])} | "
                     f"{_act(f.get('raw_action'))} | {_act(f.get('repaired_action'))} | "
                     f"{_f(f.get('at_t'), 1)} | "
                     + (f"`{f['bundle']}`" if f.get("bundle") else "no bundle") + " |")
        L.append("")
    else:
        L += ["(no failing episode)", ""]
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
    "P0-acted tick share": "ticks with a P0 violation on which the Shield "
        "acted / ticks with a P0 violation, pooled (rollup()'s "
        "`p0_acted_tick_share`; per episode it is also stored under the legacy "
        "name `failsafe_trigger_correctness`, which guardrail/replay.py "
        "verifies). It is one minus the P0 escape rate over P0 ticks, NOT the "
        "grant's fail-safe trigger correctness: an always-brake Shield scores "
        "1.0 here. The grant's KPI is the next column.",
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
        "`theta_exceeded`); no delivered flight log does, so a flight's rate is "
        "the Shield's re-check alone and is printed `(theta unchecked)`; the "
        "stress harness logs both (basis `recheck_and_theta`). `none "
        "attempted` = zero attempts; `shield off` = repairs logged on a "
        "control arm and never flown. Null: an always-brake Shield scores 0 %. "
        "No flight rail logs RTL/Land yet (WP3-07); the headless stress harness "
        "does (guardrail/fsm.py through Shield.filter, since 2026-10-07), so "
        "there the outcome comes from the FSM and autopilot ticks are not "
        "repair attempts.",
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
    "KPI-grade": "`guardrail.manifest.is_kpi_grade` on the manifest (hil topology "
        "with its evidence, sim_speedup 1.0, a clean revision), AND the replay "
        "bundle beside the run verified with a trusted signature, its KPIs "
        "re-derived from its own log (`guardrail.replay.kpi_evidence_grade`), "
        "and holding the run folder's own flight_log.jsonl and manifest.json "
        "(`bundle_binding`). A run whose manifest passes but whose bundle is "
        "missing, unsigned, untrusted or another flight's is not KPI-grade "
        "(`kpi_grade_manifest` keeps the manifest's verdict).",
    "Vacuous": "a 0 that counts an event which could not happen: an escape "
        "rate where no raw command violated a P0 rule, a false-trigger count on "
        "logs with no escalation state (rollup()'s `vacuous_zeros`). Printed as "
        "vacuous, never as a measured 0.",
    "Repair magnitude by priority / class": "the mean repair magnitude split by "
        "the highest priority a repaired tick violated (P0 / P1 / P2) and by the "
        "operator classes that acted on it (the grant's AltitudeClamp, "
        "LateralProjection, PathRepair, and this Shield's extensions); `never` "
        "where a priority or class had no repaired tick.",
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
                 include_controls: bool, command: str, *,
                 wp1_path: str | Path | None = DEFAULT_WP1, wp2: bool = False,
                 csp_json: str | Path | None = DEFAULT_CSP_JSON,
                 policies_dir: str | Path = "policies",
                 tick_profiles: str | None = DEFAULT_TICK_PROFILES,
                 baseline: str | Path | None = None, trust_store=None) -> dict:
    """The report as a dict (render_markdown prints it). The keyword arguments
    are the sections added on 2026-10-12: WP1 from `wp1_path`, WP2 (the
    budget scan takes seconds, so it is computed only with `wp2`; otherwise
    the stored `csp_json` is read and labelled as read), the tick budget
    from `tick_profiles`, a comparison against an earlier report JSON
    (`baseline`), and the trust store each replay bundle is checked against
    as KPI evidence (None: guardrail.bundle's default)."""
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
            e = load_run(run, by_hash, by_id, trust_store)
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

    ab, para = arms_sections(episodes)
    wp1 = None
    if wp1_path:
        wp1_p = Path(wp1_path) if Path(wp1_path).is_absolute() else ROOT / wp1_path
        wp1 = wp1_section(wp1_p)
    if wp2:
        wp2_doc = wp2_section(policies_dir)
    else:
        # Never recomputed, and never written, as a side effect: the stored
        # file is read and the section says so (read_from, its provenance).
        stored = (_read_json(Path(csp_json) if Path(csp_json).is_absolute()
                             else ROOT / csp_json) if csp_json else {})
        wp2_doc = (dict(stored, read_from=_rel(Path(csp_json) if Path(csp_json).is_absolute()
                                               else ROOT / csp_json))
                   if isinstance(stored.get("default"), dict) else None)
    rep = {
        "_what": "KPI rollup across episodes (Stress Testing p6: KPI report). "
                 "Flights recomputed from per-episode logs with "
                 "guardrail.kpi.compute; stress bundles and sweep JSON as stored.",
        "generated": date.today().isoformat(),
        "command": command,
        "code_revision": code_revision(ROOT),
        "inputs": {"runs": run_globs, "sweep": sweep_paths},
        "counts": {"episodes": len(episodes),
                   "flights": sum(1 for e in episodes if e["kind"] == "flight"),
                   "stress_logs": sum(1 for e in episodes
                                      if e["kind"] == "stress-log"),
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
        "failures": failure_index(episodes, include_controls),
        "wp1": wp1,
        "wp2": wp2_doc,
        "tick_budget": (tick_budget_section(tick_profiles, episodes)
                        if tick_profiles else None),
        "ab": ab, "paraphrase": para, "baseline": None,
    }
    if baseline:
        bp = Path(baseline) if Path(baseline).is_absolute() else ROOT / baseline
        base = json.loads(bp.read_text(encoding="utf-8"))
        rep["baseline"] = compare_baseline(json_safe(rep), base, _rel(bp),
                                           include_controls)
    return rep


def write_csp_json(wp2: dict, path: str | Path, command: str) -> Path:
    """docs/data/csp_kpis.json: the WP2 section with its provenance, LF."""
    out = Path(path) if Path(path).is_absolute() else ROOT / path
    out.parent.mkdir(parents=True, exist_ok=True)
    doc = {"_what": "Prefix Compiler KPIs (Prefix Compiler p5): at the default budget "
                    "and at the largest budget where the naive cut drops a P0 rule "
                    "(tools/kpi_report.py wp2_section)",
           "generated": date.today().isoformat(), "command": command,
           "code_revision": code_revision(ROOT), **wp2}
    with open(out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps(json_safe(doc), indent=1, allow_nan=False) + "\n")
    return out


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
    ap.add_argument("--wp1", default=DEFAULT_WP1,
                    help="WP1 round-trip JSON (tools/wp1_roundtrip_kpi.py)")
    ap.add_argument("--wp2", action="store_true",
                    help="recompute the Prefix Compiler section for this report (its "
                         "budget scan takes seconds); without it the report reads "
                         f"{DEFAULT_CSP_JSON} and says so")
    ap.add_argument("--wp2-only", action="store_true",
                    help="compute only the Prefix Compiler section, write it to "
                         f"--csp-json (default {DEFAULT_CSP_JSON}), print it")
    ap.add_argument("--policies", default="policies", help="policy folder for WP2")
    ap.add_argument("--csp-json", default=None,
                    help="the WP2 file: written by --wp2-only, and by a full report "
                         "only when it is given explicitly together with --wp2; read "
                         f"otherwise (default {DEFAULT_CSP_JSON})")
    ap.add_argument("--tick-profiles", default=DEFAULT_TICK_PROFILES,
                    help="stored Shield tick profiles (tools/profile_shield_tick.py)")
    ap.add_argument("--baseline", default=None,
                    help="an earlier report JSON: list new / fixed failures and "
                         "per-family deltas")
    ap.add_argument("--trust-store", default=None,
                    help="trust store for the replay-bundle evidence check "
                         "(default: guardrail.bundle's)")
    args = ap.parse_args(argv)

    command = "python tools/kpi_report.py" + (
        " " + " ".join(shlex.quote(a) for a in argv) if argv else "")
    if args.wp2_only:
        w = wp2_section(args.policies)
        out = write_csp_json(w, args.csp_json or DEFAULT_CSP_JSON, command)
        print("\n".join(_render_wp2(w)))
        print(f"wrote {_rel(out)}")
        return 0
    rep = build_report(args.runs, [] if args.no_sweep else args.sweep,
                       args.top_k, args.include_controls, command,
                       wp1_path=args.wp1, wp2=args.wp2,
                       csp_json=args.csp_json or DEFAULT_CSP_JSON,
                       policies_dir=args.policies, tick_profiles=args.tick_profiles,
                       baseline=args.baseline, trust_store=args.trust_store)
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
        # The tracked WP2 file is written only when asked for by name: a
        # report written anywhere (a test's temp folder, a review copy) must
        # not rewrite docs/data/csp_kpis.json with its own provenance.
        if args.wp2 and args.csp_json:
            print(f"wrote {_rel(write_csp_json(rep['wp2'], args.csp_json, command))}")
    else:
        print(md)
    c = rep["counts"]
    print(f"{c['episodes']} episodes ({c['kpi_grade']} KPI-grade), "
          f"{c['skipped']} inputs skipped")
    if rep.get("baseline"):
        b = rep["baseline"]
        reg = sum(1 for r in b["new_failures"] if r.get("was") == "passing")
        print(f"against {b['path']}: {len(b['new_failures'])} new failures ({reg} "
              f"regressions), {len(b['fixed_failures'])} fixed, "
              f"{len(b['gone_failures'])} gone")
    print(f"reproduce: {command}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
