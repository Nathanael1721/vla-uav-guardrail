"""Roll many episodes up into the grant's KPI report, as Markdown and JSON.

    python tools/kpi_report.py                                  # print it
    python tools/kpi_report.py --out <stem>                     # <stem>.md + .json
    python tools/kpi_report.py --runs 'demo/out/ros2_*' --no-sweep
    python tools/kpi_report.py --campaign demo/out/hil_sm1111 \
        --rescore-rail <rescore.json> --out <stem>              # a hil campaign
    python tools/kpi_report.py --campaign demo/out/hil_ntFIN2 \
        --campaign demo/out/hil_ntFIN2s --rescore-rail <main.json> \
        --rescore-rail <supplement.json> --out <stem>           # + a supplement

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
    until a pilot that reads the prompt flies them. Since 2026-11-16 the A/B
    is also measured over pairs flown by a pilot that CONSUMES the CSP
    (`pilot_reads_prefix`, guardrail/prefix_consumer.py; experiments/
    sweep_scenarios.py, THE PREFIX CONSUMER): mission success, P0 escapes,
    repairs per episode, mean repair magnitude and time to safe, each on,
    off and on minus off, with the A/A pairs (prefix on flown twice on one
    seed) beside them as the noise floor. The per-paraphrase section is not
    touched by such a pilot: it reads no words. A campaign flown by such a
    pilot is another pilot's campaign, and half its flights carry no Prefix:
    its Campaign section says so (`prefix_consumer`), its rows are
    `.../prefix-consumer-on|off`, and its rail flights never share a report
    with flights of the library's own pilots (mixed_pilots_refusal). Since
    2026-11-18 also over pairs a LEARNED VLA flew reading the CSP as text on
    the PAS follow (`vla_reads_prefix`; perception_node.py --vla-prefix
    --ab-pair, deploy/orin/run_pas_prefix_ab.sh; `--runs` over the run
    folders): paired by their --ab-pair label (vla_prefix_pairs: one seed,
    object words, model, prompt slot, policy bundle and scene), the same
    measures plus the follow's share of ticks within 30 m (from acquisition
    and from take-off), whether mission success separates the arms at all,
    the A/A repeats as the noise floor and the neutral arm where flown; an
    arm refused before take-off (its P0 rules over the model's prompt slot)
    is listed with why among the skipped inputs. Such flights never share a
    report with another pilot's prefix A/B (mixed_pilots_refusal).
  * --baseline <report.json>: new, fixed and gone failures and per-family
    deltas against an earlier report; the JSON now carries `failures` for
    the next comparison.

SINCE 2026-10-20: COMPARISON CONFIGURATIONS ARE NOT HEADLINE FAILURES

  * An episode flown on a comparison configuration - the 3 s forecast
    (`/forecast-3s`), the open-loop wind (`/wind-open_loop`), the measured
    ArduPilot vehicle response (`/response-ardupilot`) - keeps its labelled
    family row but is ranked in its own top-K (`comparison_top_failures`)
    and indexed in `comparison_failures`, never in the grant's top-K,
    `failures` or the headline facts (`comparison_label`,
    `split_comparison`).
  * experiments/sweep_scenarios.py writes such a run to
    `demo/out/cmp_<profile>_<label>/` unless told otherwise, outside the
    `demo/out/stress_*/episode-*` glob this tool reads by default; give that
    folder in --runs to read it.

SINCE 2026-11-11: A HIL CAMPAIGN IS THE KPI REPORT (WP4, due 30 Nov)

The grant's "KPI report from a nightly stress run, with the P0 violation
escape rate at 0" is written from a hil campaign (experiments/
sweep_scenarios.py --backend hil), which leaves its flights under the hil
store (demo/out/hil_smoke/<campaign>_NNN_<cell>) and its summary in
demo/out/hil_<campaign>/sweep.json - neither of which the default globs
reach. So:

  * --campaign <dir or sweep.json> reads the campaign's sweep.json (its
    command, revision, profile, counts) and takes the runs from its store
    (`_scope.hil.config.store`/<campaign>_*) unless --runs is given; the
    headless sweep JSON is not read unless --sweep names it. A run folder
    the campaign did not ACCEPT (its result is an `error`: e.g. the cell's
    stress was not realised) or does not list is demoted from KPI grade,
    with the campaign's reason; the gate itself (is_kpi_grade plus the
    signed replay bundle bound to the run, evidence_grade) is unchanged.
  * --rescore-rail <json> (experiments/sweep_scenarios.py --rescore-rail
    ... --out <json>) applies the adopted fail-safe label reading (r2 since
    2026-11-10) to every run it names - the family table's fail-safe column
    and the top-K then read r2 - and the report prints fail-safe trigger
    correctness under every reading, (a), (r) and (r2), side by side, over
    the KPI-grade Shield-on runs: the point estimate (the KPI; the grant
    says ">= 99 %"), the Wilson 95 % lower bound (guardrail/fsm.py
    score_failsafe_triggers: meets_target / meets_target_at_95), both nulls,
    and by name the runs left out and the false and missed triggers.
  * The tick budget ("<= 100 ms end-to-end", Safety Shield p6) is read from
    every flown row over the WHOLE flight (a rail run's scenario window
    cuts the performance KPIs, never this): `tick_ms` (tick start to the
    setpoint's publication) and `e2e_ms` (an action's receipt to the first
    setpoint flown from it) on the hil Shield node, with `e2e_with_wait_ms`
    beside it (since 2026-11-23: e2e_ms plus the wait behind the previous
    tick's post-setpoint hot-apply bookkeeping; a tick is over the budget
    when that is over 100 ms), and, on the perception
    rail's VLA pilot, `cam_age_ms` / `cam_to_setpoint_ms` (the camera frame
    behind the flown action at publication) - p50 / p95 / p99 / max
    (nearest rank) and the ticks over 100 ms, pooled per rail; a measure no
    row and no metrics.json summary carries is "not measured", never 0.
  * A KPI summary leads the report (latency_section, kpi_summary): the P0
    escape rate against the Shield-off control, fail-safe correctness on
    the adopted reading, the tick budget, and the tracked KPIs - time to
    safe, mission success, repairs per episode, repair magnitude, repair
    success - over the KPI-grade Shield-on runs and per scenario template.

SINCE 2026-11-13: A CAMPAIGN AND ITS SUPPLEMENTS

The final nightly ntFIN2 lost 11 missions to the rig (8 never flown after
the desktop ran out of memory, 3 gps-dropout flights whose stress was not
realised); they were flown again, at the same commit, as the campaign
ntFIN2s. --campaign is repeatable: the first is the main campaign, every
later one a supplement, and the rules (campaign_merge, EPISODE_CHOICE_RULES)
are printed in the report's Inputs section:

  * every campaign is at the same code revision (its sweep.json
    `_manifest_common`, and the manifest of every run it lists) and the same
    profile and backend - refused otherwise, by name;
  * an episode is (cell id, seed, Shield arm); the main campaign's ACCEPTED
    episode wins, and a supplement's is counted only where no earlier
    campaign has an accepted one (not flown, errored, not accepted);
  * a supplement flight of an episode an earlier campaign accepted is
    superseded: listed by name, never read, scored or counted;
  * the report says which episodes came from which campaign;
  * --rescore-rail is repeatable too (sweep_scenarios.py writes one table
    per campaign): the tables must agree on the label artefact, its build,
    the adopted reading and the rail gates; they are merged per run, and the
    table's own figure is recomputed over the same episode choice;
  * the KPI-grade gate (is_kpi_grade, the signed replay bundle bound to the
    run) is unchanged and applies to every counted flight.

SINCE 2026-11-14: A LENGTHENED CAMPAIGN IS NEVER MERGED

experiments/sweep_scenarios.py --hil-max-s-add S flies every mission of a
hil campaign S seconds past the rail's cap, so a cell censored in Loiter at
the profile's length can show its fail-safe outcome (docs/RUBRIC-failsafe.md
section 3). Its sweep.json says so (`_scope.hil.max_s_add`), as does every
run's scenario record (`max_s_add`). It flies another flight than the
profile's, so the report keeps it apart: its runs' rows end in
"/flown-longer-<S>s" (scenario_family), the Campaign section and the Inputs
(`campaign_max_s_add`) name the length, and a campaign, supplement or
re-score table of another length is refused (campaign_merge, load_rescores,
build_report). It is reported on its own, beside the KPI campaign.

SINCE 2026-11-21: ONE SHA ACROSS THE STACK, REPORTED (ARCH-31)

Every hil run writes stack_revisions.json beside its manifest (since
2026-11-17, guardrail.manifest STACK_FILE): each host's revision and
`single_sha`, true only when every host the topology needs ran the
manifest's commit on a clean tree. Nothing read it until now. load_run
keeps each flight's record as written (guardrail.manifest.stack_summary,
the episode's `stack_revisions`), and stack_section counts, per campaign
(`by_campaign`), over the KPI-grade episodes (`kpi_grade`) and over every
flight read (`all_flights`), the runs whose single_sha is true / false /
pending (None: a host the topology needs never reported) / missing (no
record: flown before 2026-11-17) / unreadable, with the distinct companion,
desktop, component and autopilot revisions their records name. JSON:
`stack_revisions`, and `single_sha` on every `kpi_grade_episodes` entry;
Markdown: "Stack revisions by campaign" under Campaign and a line under the
KPI-grade table.

SINCE 2026-11-27: ONE SHA ACROSS THE STACK, GATED (ARCH-31)

The grant asks for "a single git SHA across the entire stack"
(03-simulation/layer-requirements.md) and rejects from the KPI report a run
that cannot produce one (data-flow.md). Every loader (load_run,
load_bundle_log, load_stored) now judges the run's record after
is_kpi_grade and the evidence check (stack_grade, guardrail.manifest
.stack_grade_reasons): single_sha false refuses a run on any topology; on
hil, a record that is pending or unreadable refuses it, and so does no
record beside a flight whose writer writes one (its metrics.json carries
`setpoint_delivery`, written by the ROS 2 Shield node since 2026-11-17,
after the record's writer). A hil run flown before the record keeps its
grade, and a harness bundle (where no rail writes the record) is judged
only on a record that is beside it. The refusal is in
`kpi_grade_reasons`; is_kpi_grade itself is unchanged. The stack section's
tallies are the same, and its `gate` says the record is now a gate.

SINCE 2026-11-28: A MATRIX ROW WITH NO P0 TICK TESTED NOTHING

The release smoke smREL072 flew near-edge-at-12mps with its school-yard zone
out of force (no scenario clock, so the hil rail read the wall clock, Fri
18:15), and both cells printed "vacuous (no P0 tick)" in the family table
while the gate asked only that every matrix row flew. shield_matrix_section
states, per row of the Shield's test matrix (SHIELD_MATRIX: high-speed
near-edge, dyn-nfz spawn, time-window switch, three simultaneous violations,
GPS noise / latency), whether it has a non-vacuous cell, and whether every
cell of near-edge-at-12mps is non-vacuous (JSON `shield_matrix`, Markdown
"Shield test matrix" after the KPI summary): the release gate's line,
docs/RUNBOOK-release.md step 5. With a campaign, every at-speed cell it
planned must be in the report and non-vacuous (a missing one is not met); a
report with no matrix scenario episode and no campaign planning one (a
follow flight, a CityLife or PAS run) prints one line and no verdict.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import re
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
from guardrail.delivery import with_stored_delivery                # noqa: E402
from guardrail.manifest import (STACK_FILE, STACK_STATES,           # noqa: E402
                                code_revision, is_kpi_grade, stack_grade_reasons,
                                stack_summary)
import rescore_kpis as R                                           # noqa: E402

DEFAULT_RUNS = ["demo/out/*", "sitl/out/*", "demo/out/stress_*/episode-*"]
DEFAULT_SWEEP = "docs/data/scenario_sweep.json"
LIBRARY = ROOT / "experiments" / "scenarios.yaml"

RAIL = {
    "canonical-hil": "ros2-mavros-sitl",      # the pre-2026-10-06 label of `dev`
    "dev": "ros2-mavros-sitl",
    "ardupilot-sitl-pymavlink": "sitl-pymavlink",
    "projectairsim-single-host": "airsim",
    "projectairsim-remote-sim": "airsim",       # the same rail, simulator on another host
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


# The grant's end-to-end measures a flown row can carry (2026-11-11), in the
# order the report prints them. Each is milliseconds on ONE clock per rail:
# the hil Shield node's time.monotonic_ns() (sitl/ros2_shield_node.py
# latency_fields), the perception node's loop clock (demo/pas_ardupilot/
# perception_node.py publish_latency_fields).
LATENCY_FIELDS = (
    ("tick_ms", "tick start to the setpoint's publication: the node's own work per tick"),
    ("e2e_ms", "an action's receipt to the first setpoint flown from it (hil Shield "
               "node; since 2026-11-11 the tick runs on the action's arrival: a deferred or "
               "timer tick adds the wait for its poll)"),
    ("e2e_with_wait_ms", "e2e_ms plus the wait the action may have had in the DDS queue "
                         "behind the previous tick's hot-apply bookkeeping (hil Shield "
                         "node since 2026-11-23, blocked_by_deferred_ms, an upper "
                         "bound): the budget's end-to-end time, so work moved out of the "
                         "tick still counts"),
    ("cam_age_ms", "age of the camera frame behind the flown action at its publication "
                   "(perception rail, VLA pilot), every tick a VLA decision's action flew"),
    ("cam_to_setpoint_ms", "camera frame to the FIRST setpoint flown from its decision "
                           "(perception rail, VLA pilot): camera input to action output"),
)
# Where a run's own metrics.json summarises them, when its rows do not.
_METRICS_LATENCY = ("tick_latency", "camera_to_setpoint")


def _num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def latency_stats(vals: list[float], budget_ms: float = TICK_BUDGET_MS) -> dict:
    """n, p50, p95, p99, max (nearest rank) and the values over the budget."""
    return {"n": len(vals), "p50": _pct(vals, 0.5), "p95": _pct(vals, 0.95),
            "p99": _pct(vals, 0.99), "max": round(max(vals), 3) if vals else None,
            "over_budget": sum(1 for v in vals if v > budget_ms)}


def _e2e_with_wait(r: dict):
    """A row's e2e_with_wait_ms: its own; on a hil row from before the
    hot-apply bookkeeping left the tick (no tick_phases_ms: nothing ran
    after the setpoint to wait behind) its e2e_ms; None on a row of the
    days the bookkeeping ran after the setpoint unmeasured (tick_phases_ms
    and no e2e_with_wait_ms), which is not measured rather than e2e_ms."""
    if "e2e_with_wait_ms" in r:
        return r["e2e_with_wait_ms"]
    if "tick_phases_ms" in r:
        return None
    return r.get("e2e_ms")


def latency_values(rows: list[dict]) -> dict[str, list[float]]:
    """Every LATENCY_FIELDS value the rows carry, per field (absent: none).
    e2e_with_wait_ms falls back to e2e_ms where nothing ran after the
    setpoint (_e2e_with_wait)."""
    out: dict[str, list[float]] = {}
    for key, _ in LATENCY_FIELDS:
        if key == "e2e_with_wait_ms":
            got = [_e2e_with_wait(r) for r in rows]
        else:
            got = [r.get(key) for r in rows]
        v = [float(x) for x in got if _num(x)]
        if v:
            out[key] = v
    return out


def _metrics_latency(metrics: dict | None, key: str) -> dict | None:
    """A run's own metrics.json summary of `key` (the hil node's
    `tick_latency`, the perception rail's `camera_to_setpoint`), or None."""
    for block in _METRICS_LATENCY:
        b = (metrics or {}).get(block)
        s = b.get(key) if isinstance(b, dict) else None
        if isinstance(s, dict) and _num(s.get("n")) and s["n"] > 0:
            return {"n": int(s["n"]), "p50": s.get("p50"), "p95": s.get("p95"),
                    "p99": s.get("p99"), "max": s.get("max"),
                    "over_budget": s.get("over_budget"),
                    "source": f"metrics.json {block} (no per-tick values)"}
    return None


def tick_timing(rows: list[dict], metrics: dict | None = None) -> dict | None:
    """What a flown log says about the 100 ms tick budget: the Shield's own
    filter() time per tick (`shield_ms`, logged since 2026-10-06) and the
    loop period between logged ticks (the end-to-end rate the rail achieved).
    Nearest-rank percentiles; None when the log has neither.

    Since 2026-11-11 also the grant's end-to-end measures (LATENCY_FIELDS)
    the rows carry, else the run's own metrics.json summary of them (its
    `source` says so; it has no p95). A measure neither carries is absent,
    never a 0. load_run passes the WHOLE flight's rows: a rail run's
    scenario window cuts its performance KPIs, never the tick budget."""
    ms = [float(r["shield_ms"]) for r in rows
          if isinstance(r.get("shield_ms"), (int, float))
          and not isinstance(r.get("shield_ms"), bool)]
    ts = [float(r["t"]) for r in rows if isinstance(r.get("t"), (int, float))]
    per = [round(1000.0 * (b - a), 3) for a, b in zip(ts, ts[1:]) if b > a]
    lat: dict[str, dict] = {}
    vals = latency_values(rows)
    for key, _ in LATENCY_FIELDS:
        if key in vals:
            lat[key] = dict(latency_stats(vals[key]), source="flight_log.jsonl")
        else:
            s = _metrics_latency(metrics, key)
            if s is not None:
                lat[key] = s
    if not ms and not per and not lat:
        return None
    return {"shield_ms": {"n": len(ms), "p50": _pct(ms, 0.5), "p95": _pct(ms, 0.95),
                          "p99": _pct(ms, 0.99),
                          "max": round(max(ms), 3) if ms else None,
                          "over_budget": sum(1 for v in ms if v > TICK_BUDGET_MS)},
            "period_ms": {"n": len(per), "p50": _pct(per, 0.5), "p95": _pct(per, 0.95),
                          "p99": _pct(per, 0.99),
                          "max": max(per) if per else None,
                          "over_budget": sum(1 for v in per if v > TICK_BUDGET_MS)},
            **lat}


def _arms_of(doc: dict) -> dict | None:
    """The arm record a stress-harness bundle stored (kpi.json `arm_record`),
    reduced to what the A/B and paraphrase sections need."""
    rec = doc.get("arm_record")
    if not isinstance(rec, dict):
        return None
    para = rec.get("paraphrase") if isinstance(rec.get("paraphrase"), dict) else {}
    out = {"arm": rec.get("arm"), "prefix": rec.get("prefix"),
           "pilot_reads_text": rec.get("pilot_reads_text"),
           "paraphrase_id": para.get("paraphrase_id"), "backend": para.get("backend"),
           "cell": doc.get("id_cell") or doc.get("id"), "seed": doc.get("seed")}
    if rec.get("pilot_reads_prefix") is True:
        # A pilot that consumes the CSP (2026-11-16): only such a record
        # carries these keys, so every other episode's `arms` is unchanged.
        c = rec.get("consumer") if isinstance(rec.get("consumer"), dict) else {}
        out.update(pilot_reads_prefix=True, consumer=c.get("id"),
                   adapter=c.get("adapter"), repeat=int(rec.get("repeat") or 0))
    return out


def consumer_suffix(arm_record) -> str:
    """The family term of an episode flown by a pilot that consumes the CSP:
    "/prefix-consumer-on" / "-off" after the Shield arm (experiments/
    sweep_scenarios.py prefix_family_suffix writes the same), "" for every
    other episode. Another pilot than the cell's own, and its two arms are
    two experiments: neither pools with the cell's row or with the other."""
    if not (isinstance(arm_record, dict) and arm_record.get("pilot_reads_prefix") is True):
        return ""
    return f"/prefix-consumer-{arm_record.get('prefix')}"


# A rail run of the stress library (experiments/sweep_scenarios.py --backend
# hil / sitl) carries the harness's verdict beside it (2026-11-06).
SCENARIO_SCORE = "scenario_score.json"
# The fields the harness measured that a flight log alone cannot give
# compute() - the label fields load_bundle_log keeps from a stress bundle's
# table, plus the label itself.
SCENARIO_FIELDS = ("failsafe_triggered", "failsafe_matches_label", "mission_outcome",
                   "outcome_matches_label", "breaches", "polygon_entries",
                   "true_p0_flown_ticks", "min_true_dist_to_fence_m", "reached_goal",
                   "failsafe_label", "failsafe_clause", "failsafe_censored",
                   # Who triggered the fail-safe (2026-11-15): "fsm", or
                   # "autopilot_geofence" for the GeoFence backstop taking the
                   # aircraft while the FSM held (sweep_scenarios.BACKSTOP_RULE).
                   # Absent from a score written before that rule.
                   "failsafe_by",
                   # What the autopilot flew, counted apart (2026-11-07): its
                   # keep-out breaches and polygon entries (gated), its
                   # corridor / altitude-band departures (reported).
                   "autopilot_breaches", "autopilot_keepout_breaches",
                   "autopilot_polygon_entries", "autopilot_departures")


def scenario_score_for(run: Path, scenario) -> tuple[dict | None, str | None]:
    """(the run's scenario_score.json, None) when it is THIS run's: its
    run id and nonce are the ones the node recorded in metrics.json
    `scenario` (the harness's --scenario-meta). (None, why) otherwise; (None,
    None) for a run that flew no scenario or has no score yet."""
    p = run / SCENARIO_SCORE
    if not isinstance(scenario, dict) or not p.is_file():
        return None, None
    doc = _read_json(p)
    if not doc:
        return None, f"{SCENARIO_SCORE} unreadable"
    if (doc.get("run_id"), doc.get("run_nonce")) != (scenario.get("run_id"),
                                                     scenario.get("run_nonce")):
        return None, (f"{SCENARIO_SCORE} is for {doc.get('run_id')!r} / nonce "
                      f"{doc.get('run_nonce')!r}, the run flew {scenario.get('run_id')!r} "
                      f"/ {scenario.get('run_nonce')!r}: ignored")
    if not isinstance(doc.get("metrics"), dict) or not isinstance(doc.get("kpi"), dict):
        return None, f"{SCENARIO_SCORE} carries no metrics / kpi"
    return doc, None


def rail_window_cap(sc_doc: dict | None) -> float | None:
    """The time cap a rail run's scenario_score.json was read up to
    (`rail_window` `max_s`, since 2026-11-07), None for a score written
    before (it read every row) or none at all."""
    win = (sc_doc or {}).get("rail_window")
    cap = win.get("max_s") if isinstance(win, dict) else None
    return float(cap) if isinstance(cap, (int, float)) else None


# The FSM edges that end a mission (sweep_scenarios.TERMINAL_EDGES).
TERMINAL_EDGES = ("G9", "G10", "X5")


def rail_scored_rows(rows: list[dict], cap: float) -> list[dict]:
    """The rows the harness read a rail run's performance KPIs over
    (sweep_scenarios.rail_scored_rows, the same cut, tested against it): up
    to and including the FSM's terminal edge, before the scenario's cap."""
    k = next((i for i, r in enumerate(rows) if r.get("fsm_edge") in TERMINAL_EDGES), None)
    mission = rows if k is None else rows[:k + 1]
    return mission[:next((i for i, r in enumerate(mission)
                          if float(r.get("t") or 0.0) >= cap - 1e-9), len(mission))]


def max_s_add(record: dict | None) -> float:
    """How many seconds past the rail's cap a rail run (its scenario record,
    `max_s_add`) or a hil campaign (its `_scope.hil`, `max_s_add`) was flown
    (experiments/sweep_scenarios.py --hil-max-s-add, flight_max_s_add): 0.0
    at the profile's own length, which every record without the key is."""
    v = (record or {}).get("max_s_add")
    try:
        x = float(v) if v is not None else 0.0
    except (TypeError, ValueError):
        return 0.0
    return x if math.isfinite(x) and x > 0.0 else 0.0


def scenario_family(scenario: dict, arm: str) -> str:
    """The KPI report's row of a rail run of the stress library:
    "<backend>-stress/<template>/<cell>/shield-<arm>" (hil-stress/... for the
    hil rail), the stress bundles' row with the rail named where the
    harness's "stress-bundle" stands. A run of a LENGTHENED hil campaign
    (2026-11-14, `max_s_add` in its scenario record) gets a row of its own,
    "/flown-longer-<S>s" after the arm (sweep_scenarios.lengthened_suffix):
    it flew another flight than the profile's, so it never pools with it.
    A mission of the Gazebo functional rail (2026-11-17, `rail` "gazebo" in
    its scenario record) is filed "gazebo-stress/...": another physics than
    the SITL rail's, so it never pools with it either."""
    backend = scenario.get("rail") or scenario.get("backend") or "hil"
    add = max_s_add(scenario)
    return (f"{backend}-stress/{scenario.get('template')}/"
            f"{scenario.get('cell') or scenario.get('scenario_id')}/shield-{arm}"
            + consumer_suffix(scenario.get("arm_record"))
            + (f"/flown-longer-{add:g}s" if add else ""))


def stack_grade(graded: bool, reasons: list[str], stack: dict, manifest: dict,
                metrics: dict | None) -> tuple[bool, list[str]]:
    """The KPI grade once the run's stack record has been judged (ARCH-31,
    since 2026-11-27; guardrail.manifest.stack_grade_reasons): single_sha
    false refuses a run on any topology; on hil a record that is pending or
    unreadable, or none beside a run whose writer writes one, refuses it
    too. The reasons are added whatever the grade was, so a run refused for
    two causes names both."""
    why = stack_grade_reasons(stack, manifest, metrics)
    return (graded and not why), list(reasons) + why


def load_run(run: Path, by_hash: dict, by_id: dict, trust_store=None) -> dict:
    """Score one run folder. Returns an episode dict, or {"skipped": reason}.

    A RAIL RUN OF THE STRESS LIBRARY (2026-11-06): the node recorded the
    harness's scenario in metrics.json (`scenario`), and the harness wrote
    its verdict beside the run (scenario_score.json, sweep_scenarios.
    write_scenario_score). When the two match (scenario_score_for), the run
    is re-scored as a stress bundle with its log is (load_bundle_log): with
    the metrics the harness gave compute() - the goal, the breaches it
    measured (a zone spawned onto the aircraft, a window opening over it, is
    not one), the arm, the FSM record - and the rule priorities it recorded
    (hot-applied zones included); its label fields come from the harness's
    table; and its row is the scenario's (scenario_family:
    "hil-stress/<template>/<cell>/shield-<arm>"). The run's own dwell figures
    stay beside it (`rail_dwell`)."""
    log = run / "flight_log.jsonl"
    if not log.is_file():
        if (run / "kpi.json").is_file():
            return load_stored(run, trust_store)
        refused = run / VLA_PREFIX_REFUSED
        if refused.is_file():
            # A prefix-A/B arm refused before take-off (guardrail/vla_prefix.py:
            # the P0 rules over the model's prompt slot, never truncated): no
            # flight, and the report says why rather than "no log".
            doc = _read_json(refused)
            return {"id": run.name, "source": _rel(run), "prefix_refused": doc,
                    "skipped": (f"prefix A/B arm {doc.get('arm')!r} of pair "
                                f"{doc.get('ab_pair')!r} refused before take-off: "
                                f"{doc.get('why')}")}
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
    scenario = metrics.get("scenario") if isinstance(metrics.get("scenario"), dict) else None
    sc_doc, sc_note = scenario_score_for(run, scenario)
    cap = rail_window_cap(sc_doc)
    rows_all = rows
    if cap is not None:
        # The windows the harness scored (2026-11-07, experiments/
        # sweep_scenarios.py sitl_extra): the performance KPIs over the
        # scenario's window - the mission's rows (to the FSM's terminal edge)
        # before its time cap - and the P0 escapes, a safety invariant, over
        # the whole flight (below). Past the cap the rail flew on so a
        # fail-safe could show its outcome, which the FSM record carries.
        rows = rail_scored_rows(rows, cap)
    sc_prios = sc_doc.get("rule_priorities") if sc_doc is not None else None
    if isinstance(sc_prios, dict) and sc_prios:
        # The harness's: the take-off policy's and every hot-applied rule's.
        prios = dict(prios, **sc_prios)
    # demo/follow_vlm.py has no Shield-off switch, so an AirSim flight is
    # shielded by construction. Saying so to compute() keeps a repair that
    # changed nothing a FAILED repair there, instead of letting the arm-unknown
    # "logged but not flown" test set it aside.
    m_c = (dict(metrics, shield="on")
           if rail == "airsim" and metrics.get("shield") is None else metrics)
    if sc_doc is not None:
        # The metrics the harness gave compute() (a stress bundle's way,
        # load_bundle_log), not the rail's dwell figures: a zone the scenario
        # spawned onto the aircraft is dwell, not a failure of the mission.
        m_c = dict(sc_doc["metrics"])
    # compute() judges theta on the basis the run recorded; on the one-period
    # reading a clamped tick needs the policy's caps to be sized
    # (rescore_kpis.policy_caps). None without a resolved policy.
    k = K.compute(rows, prios, m_c, caps=R.policy_caps(pol))
    if cap is not None and len(rows_all) != len(rows):
        # A safety invariant is the whole flight's (the harness's ruling).
        kw = K.compute(rows_all, prios, m_c, caps=R.policy_caps(pol))
        k.update({f: v for f, v in kw.items() if f.startswith("p0_")})
    if sc_doc is not None:
        for f in SCENARIO_FIELDS:
            if f in sc_doc["kpi"]:
                k[f] = sc_doc["kpi"][f]
    priority_source = ("the stress harness's (scenario_score.json): the policy's and "
                       "every hot-applied rule's" if isinstance(sc_prios, dict) and sc_prios
                       else "recomputed against the resolved policy")
    if pol is None and not (isinstance(sc_prios, dict) and sc_prios):
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
    k["null_hover_mission"] = K.null_hover_mission(rows, m_c if sc_doc is not None
                                                   else metrics)
    wt = K.worst_tick(rows, prios)
    if wt is not None and pol is None and not (isinstance(sc_prios, dict) and sc_prios):
        wt["priorities_unresolved"] = True
    k["worst_tick"] = wt

    # A run flown before the Shield recorded setpoint delivery (2026-11-17)
    # is graded on the record its two logs give (guardrail.delivery
    # .stored_record): ntFIN2_175's 245 unsent setpoints are not KPI-grade.
    graded, reasons = (is_kpi_grade(manifest, with_stored_delivery(metrics, run))
                       if manifest
                       else (False, ["no manifest.json"]))
    manifest_grade = graded
    graded, reasons, ev_bundle = evidence_grade(run, graded, reasons, trust_store)
    # One SHA across the stack (ARCH-31, gated since 2026-11-27): the run's
    # stack record, judged against the metrics.json the run wrote (never the
    # delivery-filled copy above, which would read every older run as one
    # whose writer writes the record).
    stack = stack_summary(run)
    graded, reasons = stack_grade(graded, reasons, stack, manifest, metrics)
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
    if scenario is not None:
        # A rail run of the stress library: the scenario's row, with or
        # without the harness's score.
        family = scenario_family(scenario, arm)
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
        # The whole flight's (rows_all): the tick budget holds on every tick,
        # the scenario window's cut is the performance KPIs' (2026-11-11).
        "tick_timing": tick_timing(rows_all, metrics),
        # The per-tick values behind it, pooled by build_report and dropped
        # from the episode before the report is written.
        "_latency_values": latency_values(rows_all),
        # The verdict written at flight time, under the rule of that day. Kept
        # beside today's: is_kpi_grade has tightened since (2026-10-06: the
        # desktop canonical-hil rail now reads as 'dev'), and a reader must be
        # able to see which runs that moved.
        "kpi_grade_at_flight": stored.get("kpi_grade"),
        "time_to_safe_basis": basis,
        "priority_fields_from": priority_source,
        "code_revision": manifest.get("code_revision"),
        # The run's stack record (stack_revisions.json, ARCH-31), as written:
        # single_sha and the revisions it names (2026-11-21; reported per
        # campaign and over the KPI-grade runs by stack_section; a gate since
        # 2026-11-27, stack_grade above).
        "stack_revisions": stack,
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
    if scenario is not None:
        k.update({
            "scenario_id": scenario.get("scenario_id"),
            "params": {"template": scenario.get("template"),
                       "cell": scenario.get("params") or None,
                       "seed": scenario.get("seed", manifest.get("random_seed")),
                       "scenario_id": scenario.get("scenario_id"),
                       "policy_id": pid, "model": manifest.get("vla_model_hash"),
                       "topology": manifest.get("topology")},
            # Where the harness's verdict came from, or why it was not used.
            "scenario_score": (SCENARIO_SCORE if sc_doc is not None
                               else sc_note or f"no {SCENARIO_SCORE} beside the run"),
            "scenario_status": (sc_doc or {}).get("status"),
            "metrics_from": (f"{SCENARIO_SCORE}: the metrics the stress harness gave "
                             f"compute()" if sc_doc is not None else "metrics.json"),
            "rail_dwell": {f: metrics.get(f) for f in ("nfz_s", "alt_violation_s",
                                                      "standoff_s")},
            # The window the KPIs above were read over (the scenario's cap;
            # None: every row) and the gates the verdict used (2026-11-07).
            "rail_window": (sc_doc or {}).get("rail_window"),
            "scenario_gates_basis": (sc_doc or {}).get("gates_basis"),
            # Seconds flown past the rail's cap (a lengthened hil campaign,
            # 2026-11-14; 0.0 at the profile's length): its family says so too.
            "max_s_add": max_s_add(scenario),
        })
        if isinstance(scenario.get("arm_record"), dict):
            # A prefix-consumer mission (2026-11-16): the arm its scenario
            # record names, for the A/B section. No other rail run has one.
            k["arms"] = _arms_of({"arm_record": scenario["arm_record"],
                                  "id_cell": scenario.get("cell"),
                                  "seed": scenario.get("seed")})
            k["prefix"] = scenario["arm_record"].get("prefix")
    vp = metrics.get("vla_prefix")
    if scenario is None and isinstance(vp, dict) and vp.get("in_ab") is True:
        # A prefix-A/B arm flown by a VLA that reads the prefix as text
        # (2026-11-18; demo/pas_ardupilot/perception_node.py --vla-prefix,
        # --ab-pair): paired by arms_sections, never with any other flight.
        # The pair must also share the policy bundle that flew (the
        # manifest's hash, on the off arm too) and the scene (review of
        # 94a7d7b).
        sc = metrics.get("sim_config") if isinstance(metrics.get("sim_config"), dict) else {}
        k["arms"] = vla_prefix_arms(vp, policy_hash=manifest.get("policy_hash"),
                                    scene=sc.get("scene_id"))
        k["prefix"] = vp.get("prefix")
        # The follow's share of ticks within 30 m of the car: from take-off,
        # and from the start gate's acquisition (perception_node's
        # frac_within_30m_after_acquire; None when the gate never opened),
        # the fair-follow scoring a VLA follow is compared on.
        k["follow_within_30m"] = _val(metrics.get("frac_within_30m"))
        k["follow_within_30m_after_acquire"] = _val(
            metrics.get("frac_within_30m_after_acquire"))
    return k


# A refused prefix-A/B arm's record (perception_node.VLA_PREFIX_REFUSED_FILE).
VLA_PREFIX_REFUSED = "vla_prefix_refused.json"


def vla_prefix_arms(vp: dict, *, policy_hash=None, scene=None) -> dict:
    """The A/B arm of a flight whose VLA read the prefix as text (metrics.json
    `vla_prefix`, guardrail/vla_prefix.py): `vla_reads_prefix`, a key apart
    from `pilot_reads_text` (the stress harness's) and `pilot_reads_prefix`
    (the hand-written consumer's), so it pools with neither. `policy_hash`
    (manifest.json's) and `scene` (sim_config.scene_id) are the flight's, for
    the pairing (VLA_PAIR_SAME)."""
    slot = vp.get("slot") if isinstance(vp.get("slot"), dict) else {}
    return {"arm": vp.get("arm"), "prefix": vp.get("prefix"), "vla_reads_prefix": True,
            "ab_pair": vp.get("ab_pair"), "model": vp.get("model"),
            "slot_model": slot.get("model"), "object": vp.get("object"),
            "seed": vp.get("seed"), "prefix_sha256": vp.get("prefix_sha256"),
            "csp_content_hash": vp.get("csp_content_hash"),
            "policy_hash": policy_hash, "scene": scene,
            "cell": f"pas-follow/{vp.get('object')}", "paraphrase_id": None}


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
    --forecast-arms), "" for one LABELLED with the grant's 5 s / 0.1 s, and
    "/forecast-unlabelled" for one that carries no label at all. Without the
    suffix a report over --publish bundles pooled the 3 s comparison with the
    grant rows. The arm is read from the KPI table (`forecast_arm`, top level
    or under `kpi`), the extras' or the KPI table's `forecast.arm`, or the
    harness's `episode_start` event; a run id carrying "forecast-3s" says
    the same.

    Unlabelled is not grant (2026-10-19 review). The harness has written
    these labels only since 2026-10-13 (93dcb10); every bundle before that
    was flown on the old 3 s / 0.5 s forecast and open-loop wind (the
    CHANGELOG of 2026-10-13 retracts the 12 Oct sweep for that reason), and
    some without the FSM. Read as "", all 2396 stored stress bundles - two
    code revisions, FSM-less and FSM - filed under the grant family names,
    and a new grant-config profile run would have pooled with them."""
    labelled = False
    for d in docs:
        if not isinstance(d, dict):
            continue
        kpi = d.get("kpi") if isinstance(d.get("kpi"), dict) else {}
        fc = (d.get("forecast") if isinstance(d.get("forecast"), dict)
              else kpi.get("forecast") if isinstance(kpi.get("forecast"), dict) else {})
        arm = d.get("forecast_arm") or kpi.get("forecast_arm") or fc.get("arm")
        if arm and arm != "grant":
            return f"/forecast-{arm}"
        if "forecast-3s" in str(d.get("id") or ""):
            return "/forecast-3s"
        if arm == "grant":
            labelled = True
    return "" if labelled else "/forecast-unlabelled"


def wind_suffix(*docs) -> str:
    """"/wind-<model>" for an episode flown on a wind model other than the
    harness's default `guided` (experiments/sweep_scenarios.py
    `--wind-model open_loop`, "kept for comparison"), read from the KPI
    table's or the episode_start event's `wind_model`; "" for guided or no
    label (an unlabelled bundle is already apart, under
    /forecast-unlabelled). Until 2026-10-19 no family term read it, so an
    open_loop run pooled with the guided rows - or, at the same code
    revision, was dropped as a "duplicate" of them (`_run_labels`)."""
    for d in docs:
        if not isinstance(d, dict):
            continue
        kpi = d.get("kpi") if isinstance(d.get("kpi"), dict) else {}
        m = d.get("wind_model") or kpi.get("wind_model")
        if m and m != "guided":
            return f"/wind-{m}"
    return ""


def response_suffix(*docs) -> str:
    """"/response-<arm>" for an episode flown on a vehicle response other
    than the harness's ideal one (experiments/sweep_scenarios.py
    `--response-arms ideal ardupilot`, the measured ArduPilot response,
    docs/DESIGN-harness-vehicle-response.md), read from the KPI table's or
    the episode_start event's `vehicle_response.arm`, a top-level or KPI
    `response_arm`, or a run id carrying "response-"; "" for the ideal
    vehicle or no label. Unlike the forecast, an unlabelled bundle IS the
    headline configuration here: every bundle written before the arm existed
    (2026-10-19) was flown on the ideal vehicle, the only one there was.
    Without the suffix a report over a response-arm run would pool the
    ArduPilot copies with the ideal rows of the same cell."""
    for d in docs:
        if not isinstance(d, dict):
            continue
        kpi = d.get("kpi") if isinstance(d.get("kpi"), dict) else {}
        vr = (d.get("vehicle_response") if isinstance(d.get("vehicle_response"), dict)
              else kpi.get("vehicle_response")
              if isinstance(kpi.get("vehicle_response"), dict) else {})
        arm = d.get("response_arm") or kpi.get("response_arm") or vr.get("arm")
        if arm and arm != "ideal":
            return f"/response-{arm}"
        # Only the arm tag's own spelling (after "#" or ","), so a scenario
        # whose id merely contains the word is not read as an arm.
        hit = re.search(r"[#,]response-([A-Za-z0-9_]+)", str(d.get("id") or ""))
        if hit and hit.group(1) != "ideal":
            return f"/response-{hit.group(1)}"
    return ""


def _run_labels(*docs) -> str:
    """The configuration a stress episode was flown in that its scenario,
    cell and seed do not name: forecast arm, wind model and (since
    2026-10-19) vehicle response. Part of the family name AND of the
    duplicate key - two episodes of one cell and seed at one revision on
    different forecasts, winds or vehicles are two experiments, and the
    duplicate key once dropped the comparison arm as a copy of the grant one
    (build_arm records no arm for a forecast-only arm, so the `arm_tag` in
    the key was "" for both)."""
    return forecast_suffix(*docs) + wind_suffix(*docs) + response_suffix(*docs)


def _bundle_family(doc: dict, sid, params: dict, arm: str, start: dict | None = None) -> str:
    template = doc.get("template") or str(sid).split("-")[0]
    fam = f"stress-bundle/{template}/{cell_name(sid, params)}/shield-{arm}"
    if consumer_suffix(doc.get("arm_record")):
        fam += consumer_suffix(doc.get("arm_record"))
    elif doc.get("prefix") == "off":
        fam += "/prefix-off"
    return fam + _run_labels(doc, start)


# The family terms _run_labels appends that name a COMPARISON configuration:
# a forecast other than the grant's (`/forecast-3s`), a wind model other than
# the harness's default (`/wind-open_loop`, "kept for comparison") and a
# vehicle other than the ideal one (`/response-ardupilot`). Not
# `/forecast-unlabelled`: those are the stress bundles written before the
# labels (before 2026-10-13), every one of them flown as the headline run of
# its day, and they stay where they always were.
_COMPARISON_TERM = re.compile(r"(forecast|wind|response)-[A-Za-z0-9_.]+")
_STRESS_FAMILY_PREFIXES = ("stress-bundle/", "sweep-json/")


def comparison_label(family) -> str:
    """The comparison configuration a stress family was flown in - its
    `/forecast-<arm>` (not `/forecast-unlabelled`), `/wind-<model>` and
    `/response-<arm>` terms, in that order - or "" for the headline
    configuration (and for every flight: the rails fly the grant's Shield on
    a real autopilot, never a comparison arm).

    Read back from the family NAME on purpose: `_run_labels` appends exactly
    these terms after the Shield arm (and `/prefix-off`) of every stress
    family, so the name is the one place that carries the label on an episode
    loaded today, on a stored table, on a sweep row AND on a `failures` entry
    of an earlier report JSON (--baseline), which keeps only `id`, `family`
    and its categories. Terms are looked for only after the last
    `shield-<arm>` segment, so a cell value that happens to contain one of
    the words is not read as a label.

    Until 2026-10-20 nothing read it, and the comparison copies went into the
    grant's top-K and `failures` beside the headline episodes they shadow
    (review of T1b-harness-vehicle, P1: over the smoke profile flown with
    both vehicles, 3 of the 10 top-failure rows were /response-ardupilot
    copies; a nightly run with the arm adds 105 failing copies, enough to
    push headline failures off a ten-row table)."""
    fam = str(family or "")
    if not fam.startswith(_STRESS_FAMILY_PREFIXES):
        return ""
    segs = fam.split("/")
    arm_at = [i for i, s in enumerate(segs) if s.startswith("shield-")]
    if not arm_at:
        return ""
    return "".join(f"/{s}" for s in segs[arm_at[-1] + 1:]
                   if _COMPARISON_TERM.fullmatch(s) and s != "forecast-unlabelled")


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
    # A harness bundle is not where a rail writes its stack record (the rail
    # run is), so a missing one says nothing here; a record that IS beside
    # it is judged as a flight's (stack_grade, no metrics).
    graded, reasons = stack_grade(graded, reasons, stack_summary(run), manifest, None)
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
                    arm_tag, manifest.get("policy_hash"), manifest.get("code_revision"),
                    _run_labels(doc)),
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
    # A harness bundle is not where a rail writes its stack record (the rail
    # run is), so a missing one says nothing here; a record that IS beside
    # it is judged as a flight's (stack_grade, no metrics).
    graded, reasons = stack_grade(graded, reasons, stack_summary(run), manifest, None)
    goal = k.get("reached_goal")
    arm_tag = ((doc.get("arm_record") or {}).get("arm") or "")
    k.update({
        "id": f"{cell_name(sid, params)}{'#' + arm_tag if arm_tag else ''}--seed{seed}",
        "scenario_id": sid,
        "source": _rel(run), "kind": "stored-table", "recomputed": False,
        "source_note": "stored table, not recomputed (bundle has no flight_log.jsonl)",
        "family": _bundle_family(doc, sid, params, arm, start),
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
                    arm_tag, manifest.get("policy_hash"), manifest.get("code_revision"),
                    _run_labels(doc, start)),
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
                       + (consumer_suffix(r.get("arm_record"))
                          or ("/prefix-off" if r.get("prefix") == "off" else ""))
                       + _run_labels(r, r.get("kpi"))),
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
# a hil campaign and its fail-safe re-score (2026-11-11)
# --------------------------------------------------------------------------- #

GRANT_FAILSAFE_TARGET = 0.99    # Stress Testing p6: fail-safe trigger correctness >= 99 %
RESCORE_SCHEMA = "rail-rescore/1"
# experiments/sweep_scenarios.py RAIL_LABEL_READINGS, in its order: the
# adopted reading is named by the table itself (`adopted_kpi_reading`).
READING_ORDER = ("a", "r", "r2")
# Why a Shield-on run has no verdict under a reading (rail_label_scores'
# lists): never counted as correct, and named.
READING_EXCLUDED = ("not_accepted", "not_measured", "censored", "undecided", "unlabelled")
# THE GEOFENCE BACKSTOP (2026-11-15; docs/RUBRIC-failsafe.md sections 1 and 9,
# experiments/sweep_scenarios.py BACKSTOP_RULE). A run in which the autopilot's
# GeoFence took the aircraft to RTL / LAND while the FSM held is a fail-safe
# triggered by the backstop: scored against its label under every reading,
# named (`backstop`), counted beside the KPI wherever the KPI is printed, and
# read beside it the two earlier ways - as before the rule (left out as
# censored) and counted as not triggered. A re-score table written before the
# rule carries no `backstop_rule`; its runs were scored the earlier way, and
# the report says "not recorded" for the count instead of printing a 0.
FAILSAFE_BY_GEOFENCE = "autopilot_geofence"
BACKSTOP_SINCE = "2026-11-15"
_MISSION_IN_ERROR = re.compile(r"\bmission (\S+?):")


def load_campaign(path: str | Path) -> dict:
    """A campaign of experiments/sweep_scenarios.py (--backend hil) from its
    sweep.json (`path`, or the folder holding it): what was flown and how,
    the run folders it ACCEPTED (a result with its `mission_tag`) and those
    it did not (an `error` naming its mission: the flight happened, a check
    of it failed - e.g. the cell's stress was not realised), and the glob of
    its store, `_scope.hil.config.store`/<campaign>_* (else, the store
    having moved, the folder of that name beside the summary's)."""
    p = Path(path) if Path(path).is_absolute() else ROOT / path
    sw = p / "sweep.json" if p.is_dir() else p
    doc = _read_json(sw)
    if not isinstance(doc.get("results"), list):
        raise SystemExit(f"{sw}: not an experiments/sweep_scenarios.py result (no `results`)")
    scope = doc.get("_scope") if isinstance(doc.get("_scope"), dict) else {}
    hil = scope.get("hil") if isinstance(scope.get("hil"), dict) else {}
    name = hil.get("campaign")
    store = (hil.get("config") or {}).get("store")
    accepted: dict[str, dict] = {}
    not_accepted: dict[str, str] = {}
    run_dirs: list[str] = []
    # Every result as an episode (campaign_episode_key) and what became of
    # it here, for the episode choice across campaigns (campaign_merge).
    episodes: list[dict] = []
    for r in doc["results"]:
        tag = r.get("mission_tag") or (Path(r["run"]).name if r.get("run") else None)
        ep = {"key": campaign_episode_key(r), "tag": None, "status": r.get("status"),
              "state": "not flown", "why": r.get("reason")}
        if tag and r.get("status") != "error":
            accepted[tag] = {"status": r.get("status"), "cell": r.get("id"),
                             "gates_failed": r.get("gates_failed") or [],
                             "expect": r.get("expect")}
            ep.update(tag=tag, state="accepted", why=None)
            if r.get("run"):
                run_dirs.append(str(r["run"]))
        elif r.get("status") == "error":
            m = _MISSION_IN_ERROR.search(str(r.get("error") or ""))
            ep["why"] = str(r.get("error"))
            if m:
                not_accepted[m.group(1)] = str(r.get("error"))
                ep.update(tag=m.group(1), state="not accepted")
            else:
                ep["state"] = "errored before flight"
        episodes.append(ep)
    globs: list[str] = []
    if name and store:
        st = Path(store)
        if not st.is_dir() and (sw.parent.parent / st.name).is_dir():
            st = sw.parent.parent / st.name
        globs = [(st / f"{name}_*").as_posix()]
    elif run_dirs:
        globs = run_dirs
    statuses: dict[str, int] = {}
    for r in doc["results"]:
        statuses[str(r.get("status"))] = statuses.get(str(r.get("status")), 0) + 1
    common = doc.get("_manifest_common") if isinstance(doc.get("_manifest_common"), dict) else {}
    arms = scope.get("arms") if isinstance(scope.get("arms"), dict) else {}
    out = {"path": _rel(sw), "campaign": name, "backend": doc.get("_backend"),
            "profile": scope.get("profile"), "grant_scope": scope.get("grant_scope"),
            "cadence": scope.get("cadence"), "command": doc.get("_command"),
            "code_revision": common.get("code_revision"),
            # How long its missions flew (2026-11-14): 0.0 at the profile's own
            # length (every campaign before --hil-max-s-add existed), else the
            # seconds past the rail's cap of a LENGTHENED campaign, which is
            # never merged with a campaign of another length (campaign_merge).
            "max_s_add": max_s_add(hil),
            "flight_length": hil.get("flight_length"),
            "runtime_s": doc.get("_runtime_s"), "results": len(doc["results"]),
            "statuses": statuses, "flyable": hil.get("flyable"),
            "refused_cells": len(hil.get("refused") or {}),
            "runs_glob": globs, "accepted": accepted, "not_accepted": not_accepted,
            # The campaign's own fail-safe figure, on the labels it flew with
            # (reading (a) as of 2026-11-11): beside the re-scored readings.
            # `triggered_by` (2026-11-15): who triggered the fail-safes it scored,
            # the GeoFence backstop's among them; None in a summary written
            # before that rule, whose figure left such runs out as censored.
            "failsafe_at_flight": {k: (doc.get("_failsafe_labels") or {}).get(k) for k in (
                "labelled", "correct", "fail_safe_correctness", "false_triggers",
                "missed_triggers", "triggered_by")},
            # Internal (keys of tuples; not written to the report).
            "_episodes": episodes}
    if arms.get("prefix_consumer"):
        # A campaign flown by a pilot that consumes the CSP (2026-11-16,
        # `_scope.arms.prefix_consumer`): only such a campaign carries this
        # key, so every other campaign's record is the one it always was.
        out["prefix_consumer"] = {"adapter": arms["prefix_consumer"],
                                  "prefix_off_episodes": arms.get("prefix_off_episodes"),
                                  "aa_episodes": arms.get("prefix_aa_episodes")}
    return out


def campaign_episode_key(r: dict) -> tuple:
    """An episode's identity across campaigns of one profile and revision:
    (cell id, seed, Shield arm). The cell id is the result's `id` - the cell
    with its text-arm label where the family has one (`nfz-head-on#canonical,
    prefix-on`), and a comparison arm's tag (`/forecast-3s`) - so one key is
    one flight of the expansion."""
    return (str(r.get("id") or r.get("id_cell")), r.get("seed"), r.get("arm"))


# The episode choice across a main campaign and its supplements (2026-11-13),
# as the report prints it.
EPISODE_CHOICE_RULES = (
    "every campaign is flown at the same code revision (its sweep.json "
    "`_manifest_common` and the manifest of every run it lists) and the same profile "
    "and backend; checked, and the report refused otherwise",
    "an episode is (cell id, seed, Shield arm); the main campaign's ACCEPTED episode "
    "wins, and a supplement's is counted only where no earlier campaign has an accepted "
    "one (not flown, errored before flight, or not accepted)",
    "a supplement flight of an episode an earlier campaign accepted is superseded: "
    "listed by name, not read, scored or counted",
    "a not-accepted flight stays what it was (not KPI-grade, functional evidence), "
    "with the supplement flight that replaced it named",
    "every campaign flew the same length: a LENGTHENED campaign (experiments/"
    "sweep_scenarios.py --hil-max-s-add) flies another flight than the profile's and "
    "is reported beside it, never merged into its figure; refused otherwise",
    "the KPI-grade gate is unchanged: is_kpi_grade plus the signed replay bundle bound "
    "to the run, for every counted flight",
    "several --rescore-rail tables must share the label artefact, its build, the "
    "adopted reading and the rail gates; they are merged per run, and the tables' own "
    "figure is recomputed over the same episode choice")


def _campaign_name(c: dict) -> str:
    return str(c.get("campaign") or c.get("path"))


def campaign_merge(camps: list[dict]) -> dict:
    """A main campaign (`camps[0]`, load_campaign) and its supplements as one
    campaign for the report (EPISODE_CHOICE_RULES). Refused (SystemExit) when
    a supplement is the main campaign again, or flew at another code revision,
    profile or backend - or one of them cannot be checked. Returns the main
    campaign's record with the merged acceptance (`accepted`: the counted
    flights only, each with its `campaign`; `not_accepted` of every campaign;
    `superseded`: run -> what superseded it), every campaign's store
    (`runs_glob`), `supplements` (their records) and `episode_choice` (the
    rules, the count per campaign, the episodes taken from a supplement and
    what the main campaign had for each, the superseded flights, and the
    not-accepted flights a supplement replaced)."""
    main = camps[0]
    seen: dict[str, str] = {}
    for c in camps:
        for ident in (c["path"], f"campaign {c.get('campaign')}" if c.get("campaign") else None):
            if ident and ident in seen:
                raise SystemExit(f"--campaign {c['path']}: {ident} is given twice (as "
                                 f"{seen[ident]} too): a supplement is another campaign")
            if ident:
                seen[ident] = c["path"]
    for c in camps[1:]:
        for field, what in (("code_revision", "code revision"), ("profile", "profile"),
                            ("backend", "backend")):
            a, b = main.get(field), c.get(field)
            if a is None or b is None or a != b:
                raise SystemExit(
                    f"--campaign {c['path']} cannot supplement {main['path']}: its {what} is "
                    f"{b!r}, the main campaign's {a!r}. Every campaign of one report must be "
                    f"flown at the same code revision and profile"
                    + (" (a missing value cannot be checked)" if a is None or b is None
                       else ""))
        a, b = max_s_add(main), max_s_add(c)
        if a != b:
            def said(x):
                return (f"LENGTHENED, {x:g} s past the rail's cap (--hil-max-s-add)" if x
                        else "the profile's own length")
            raise SystemExit(
                f"--campaign {c['path']} cannot supplement {main['path']}: it was flown at "
                f"{said(b)}, the main campaign at {said(a)}. A lengthened campaign flies "
                f"another flight than the profile's: report it on its own, beside the KPI "
                f"campaign, never merged into its figure")
    names = [_campaign_name(c) for c in camps]
    winner: dict[tuple, tuple[int, str]] = {}
    for i, c in enumerate(camps):
        for ep in c["_episodes"]:
            if ep["state"] == "accepted" and ep["key"] not in winner:
                winner[ep["key"]] = (i, ep["tag"])
    main_by_key: dict[tuple, dict] = {}
    for ep in main["_episodes"]:
        main_by_key.setdefault(ep["key"], ep)
    accepted: dict[str, dict] = {}
    not_accepted: dict[str, str] = {}
    superseded: dict[str, dict] = {}
    tag_campaign: dict[str, str] = {}
    from_supplements: list[dict] = []
    replaced: dict[str, str] = {}
    counted = {n: 0 for n in names}
    for i, c in enumerate(camps):
        for ep in c["_episodes"]:
            tag = ep["tag"]
            if tag is None:
                continue
            cell, seed, arm = ep["key"]
            tag_campaign[tag] = names[i]
            w = winner.get(ep["key"])
            if w is not None and (w[0] < i or (w[0] == i and w[1] != tag
                                                and ep["state"] == "accepted")):
                superseded[tag] = {"campaign": names[i], "cell": cell, "seed": seed,
                                   "arm": arm, "status": ep["status"], "by": w[1],
                                   "by_campaign": names[w[0]]}
            elif ep["state"] == "accepted":
                accepted[tag] = dict(c["accepted"][tag], campaign=names[i])
                counted[names[i]] += 1
                if i > 0:
                    m = main_by_key.get(ep["key"])
                    had = ("not in the main campaign" if m is None else
                           m["state"] + (f": {m['why']}" if m.get("why") else ""))
                    from_supplements.append({"run": tag, "campaign": names[i], "cell": cell,
                                             "seed": seed, "arm": arm, "main": had,
                                             "main_run": (m or {}).get("tag")})
            else:
                not_accepted[tag] = c["not_accepted"].get(tag) or str(ep.get("why"))
                if w is not None:
                    replaced[tag] = w[1]
    supplements = [{k: v for k, v in c.items() if not k.startswith("_")} for c in camps[1:]]
    out = dict(main)
    out.update(
        accepted=accepted, not_accepted=not_accepted, superseded=superseded,
        _tag_campaign=tag_campaign,
        runs_glob=[g for c in camps for g in c["runs_glob"]],
        main_accepted=len(main["accepted"]), supplements=supplements,
        episode_choice={
            "rules": list(EPISODE_CHOICE_RULES), "campaigns": names,
            "counted_by_campaign": counted, "from_supplements": from_supplements,
            "superseded": [dict(v, run=k) for k, v in sorted(superseded.items())],
            "not_accepted_replaced": replaced})
    return out


def load_campaigns(paths: list) -> dict | None:
    """--campaign, once or more: one campaign (load_campaign) as it always
    was, or a main campaign and its supplements merged (campaign_merge)."""
    if not paths:
        return None
    camps = [load_campaign(p) for p in paths]
    return camps[0] if len(camps) == 1 else campaign_merge(camps)


def campaign_gate(e: dict, camp: dict) -> None:
    """Demote a flown episode the campaign did not accept (or does not list)
    from KPI grade, with the campaign's reason; record its status and, with
    supplements, the campaign it came from."""
    name = Path(str(e.get("source") or e.get("id"))).name
    who = (camp.get("_tag_campaign") or {}).get(name) or _campaign_name(camp)
    got = camp["accepted"].get(name)
    if got is not None:
        e["campaign_status"] = got["status"]
        if got.get("campaign"):
            e["campaign"] = got["campaign"]
        return
    why = camp["not_accepted"].get(name)
    e["campaign_status"] = "not accepted" if why else "not in the campaign"
    if camp.get("supplements"):
        e["campaign"] = who if why else None
        names = " + ".join([_campaign_name(camp)] + [_campaign_name(s)
                                                     for s in camp["supplements"]])
        repl = (camp.get("episode_choice") or {}).get("not_accepted_replaced", {}).get(name)
        reason = (f"campaign {who} did not accept this flight: {why}"
                  + (f" (replaced by {repl})" if repl else "") if why else
                  f"not one of campaigns {names}'s flights (none of their summaries "
                  f"lists it)")
    else:
        reason = (f"campaign {who} did not accept this flight: {why}"
                  if why else f"not one of campaign {who}'s "
                              f"flights ({camp['path']} does not list it)")
    e["kpi_grade"] = False
    e["kpi_grade_reasons"] = list(e.get("kpi_grade_reasons") or []) + [reason]


def load_rescore(path: str | Path) -> dict:
    """The table of experiments/sweep_scenarios.py --rescore-rail ... --out
    <path>: per run its re-scored fail-safe verdict and, per label reading
    ((a), (r), (r2); READING_ORDER), its status there - "correct", "false",
    "missed" (scored), or why it is left out (READING_EXCLUDED) - rebuilt
    from the reading's named lists exactly as rail_label_scores made them.
    Refused when it is not such a table or carries no label table."""
    p = Path(path) if Path(path).is_absolute() else ROOT / path
    doc = _read_json(p)
    if doc.get("schema") != RESCORE_SCHEMA or not isinstance(doc.get("runs"), list):
        raise SystemExit(f"{p}: not a {RESCORE_SCHEMA} table (experiments/sweep_scenarios.py "
                         f"--rescore-rail <runs> --out <this file>)")
    fl = doc.get("failsafe_labels")
    if not isinstance(fl, dict) or not isinstance(fl.get("readings"), dict):
        raise SystemExit(f"{p}: carries no fail-safe label table (the re-score found no "
                         f"label artefact): no reading to apply")
    readings = fl["readings"]
    keys = ([k for k in READING_ORDER if k in readings]
            + [k for k in readings if k not in READING_ORDER])
    runs: dict[str, dict] = {}
    adds: dict[float, list[str]] = {}
    for it in doc["runs"]:
        if "error" in it:
            runs[str(it.get("run"))] = {"error": it["error"]}
            continue
        d = it.get("doc") or {}
        kpi = d.get("kpi") or {}
        add = max_s_add(d.get("scenario"))
        adds.setdefault(add, []).append(str(it.get("run")))
        runs[str(it.get("run"))] = {
            "cell": it.get("cell"), "seed": it.get("seed"),
            "shield": (d.get("metrics") or {}).get("shield"),
            "triggered": kpi.get("failsafe_triggered"),
            "accepted": it.get("before") is not None,
            # The prefix-on arm of a pilot that consumes the CSP (2026-11-16):
            # the cell's labels are its own pilot's, so the table scores it
            # under no reading (sweep_scenarios.rescore_rail_main). Absent
            # from every reading's lists, it must not read as "correct".
            "labels_not_applied": bool((d.get("scenario") or {}).get("labels_not_applied")),
            "readings": {}}
    declared = (doc.get("flight_length") or {}).get("max_s_add")
    if len(adds) > 1 or (declared is not None and adds
                         and set(adds) != {max_s_add({"max_s_add": declared})}):
        # One flight length per table (sweep_scenarios.rescore_rail_main
        # refuses to write another since 2026-11-14).
        raise SystemExit(f"{p}: its runs were flown at more than one flight length "
                         f"({sorted(adds)} s past the rail's cap; declared {declared!r}): a "
                         f"lengthened campaign is re-scored and reported on its own")
    table_add = next(iter(adds), max_s_add({"max_s_add": declared}))
    for key in keys:
        rd = readings[key]
        named: dict[str, tuple[str, list]] = {}
        for kind in ("false", "missed") + READING_EXCLUDED:
            for t in rd.get(kind) or []:
                named[str(t.get("run"))] = (kind, t.get("label") or [])
        longer = {str(t.get("run")) for t in rd.get("label_flown_longer") or []}
        # Triggered by the GeoFence backstop (2026-11-15): named by the
        # reading, with its takeover and what the reading before the rule
        # made of it (`fsm_censored`: left out as censored).
        backstop = {str(t.get("run")): t for t in rd.get("backstop") or []}
        for name, rec in runs.items():
            if "error" in rec or rec["shield"] != "on" or rec["labels_not_applied"]:
                continue
            kind, clauses = named.get(name, ("correct", []))
            if kind in ("correct", "false", "missed"):
                trig = bool(rec["triggered"])
                rec["readings"][key] = {
                    "status": kind, "triggered": trig, "clauses": clauses,
                    "expected": {"false": False, "missed": True}.get(kind, trig),
                    "label_flown_longer": name in longer}
                if name in backstop:
                    b = backstop[name]
                    rec["readings"][key]["backstop"] = {
                        k: b.get(k) for k in ("mode", "t", "fsm_state", "fsm_censored")}
            else:
                rec["readings"][key] = {"status": kind, "expected": None,
                                        "triggered": rec["triggered"]}
    return {"path": _rel(p), "paths": [_rel(p)], "schema": RESCORE_SCHEMA,
            "rail_gates": doc.get("rail_gates"), "max_s_add": table_add,
            "artefact": fl.get("artefact"), "built": fl.get("built"),
            "adopted": fl.get("adopted_kpi_reading"), "keys": keys,
            # The GeoFence backstop rule the table was scored under (None: a
            # table written before 2026-11-15, scored the earlier way).
            "backstop_rule": fl.get("backstop_rule"),
            "shield_off_runs_left_out": fl.get("shield_off_runs_left_out"),
            "readings": {k: {f: readings[k].get(f) for f in (
                "reading", "adopted_kpi_reading", "runs", "scored", "correct",
                "failsafe_trigger_correctness", "wilson_low_95", "at_scenario_length")}
                for k in keys},
            "runs": runs}


_TABLE_FIGURE = ("runs", "scored", "correct", "failsafe_trigger_correctness", "wilson_low_95")


def load_rescores(paths: list) -> dict | None:
    """--rescore-rail, once or more (experiments/sweep_scenarios.py
    --rescore-rail writes one table per campaign): one table (load_rescore)
    as it always was, or several merged per run. Refused (SystemExit) when
    the tables disagree on what a verdict means - the label artefact, its
    build, the adopted reading, the readings, the rail gates - or when one
    run carries two different verdicts. The merged table's `path` names
    every file (`paths` lists them) and `tables` keeps each file's own
    figure; the figure over the runs the report counts is recomputed by
    failsafe_readings_section, with the campaigns' episode choice."""
    if not paths:
        return None
    tables = [load_rescore(p) for p in paths]
    first = tables[0]
    if len(tables) == 1:
        return first
    given = [t["path"] for t in tables]
    twice = sorted({p for p in given if given.count(p) > 1})
    if twice:
        raise SystemExit(f"--rescore-rail {twice[0]} is given twice")
    for t in tables[1:]:
        for f, what in (("artefact", "label artefact"), ("built", "label build"),
                        ("adopted", "adopted KPI reading"), ("keys", "label readings"),
                        ("rail_gates", "rail gates"),
                        ("backstop_rule", "GeoFence backstop rule (a table written before "
                                          "2026-11-15 has none)"),
                        ("max_s_add", "flight length (seconds past the rail's cap, "
                                      "--hil-max-s-add)")):
            if t[f] != first[f]:
                raise SystemExit(
                    f"--rescore-rail {t['path']} and {first['path']} disagree on the {what} "
                    f"({t[f]!r} vs {first[f]!r}): re-score every campaign with one checkout "
                    f"and one label artefact before merging them")
    runs: dict[str, dict] = {}
    origin: dict[str, str] = {}
    for t in tables:
        for name, rec in t["runs"].items():
            if name in runs and runs[name] != rec:
                raise SystemExit(f"run {name} is in --rescore-rail {origin[name]} and "
                                 f"{t['path']} with different verdicts")
            if name not in runs:
                runs[name], origin[name] = rec, t["path"]
    out = dict(first)
    out.update(
        path=" + ".join(t["path"] for t in tables), paths=[t["path"] for t in tables],
        runs=runs,
        shield_off_runs_left_out=sum(1 for r in runs.values()
                                     if "error" not in r and r["shield"] != "on"),
        readings={k: dict(first["readings"][k], **{f: None for f in _TABLE_FIGURE})
                  for k in first["keys"]},
        tables=[{"path": t["path"], "runs": len(t["runs"]),
                 "readings": {k: {f: t["readings"][k].get(f) for f in _TABLE_FIGURE}
                              for k in t["keys"]}} for t in tables])
    return out


def rescore_table_figure(rs: dict, key: str, superseded: set) -> dict:
    """The re-score tables' own figure under reading `key`, recomputed from
    their per-run verdicts as rail_label_scores counts them (every Shield-on
    run re-scored without error; scored = correct, false or missed), leaving
    out the runs the campaigns' episode choice superseded and those whose
    labels are not applied (rescore_rail_main drops them before
    rail_label_scores counts its runs)."""
    pairs, n, na = [], 0, 0
    for name, rec in rs["runs"].items():
        if name in superseded or "error" in rec or rec.get("shield") != "on":
            continue
        if rec.get("labels_not_applied"):
            na += 1
            continue
        n += 1
        rd = (rec.get("readings") or {}).get(key)
        if rd is not None and rd["status"] in ("correct", "false", "missed"):
            pairs.append((rd["expected"], rd["triggered"]))
    sc = failsafe_score(pairs)
    return {"runs": n, "scored": sc["scored"],
            "correct": (sc["true_triggers"] or 0) + (sc["true_non_triggers"] or 0),
            "failsafe_trigger_correctness": sc["failsafe_trigger_correctness"],
            "wilson_low_95": sc["wilson_low_95"],
            "basis": (f"{len(rs.get('paths') or [])} table(s) merged per run"
                      + (f", {sum(1 for x in rs['runs'] if x in superseded)} superseded "
                         f"run(s) left out" if superseded else "")
                      + (f", {na} run(s) with labels not applied left out" if na else ""))}


LABELS_NOT_APPLIED_WHY = ("the prefix-on arm of a pilot that consumes the CSP: the cell's "
                          "fail-safe label was derived for its own pilot, so it is not "
                          "applied (experiments/sweep_scenarios.py LABELS_NOT_APPLIED)")


def apply_rescore(e: dict, rs: dict) -> None:
    """The adopted reading's label on one flown episode (its row's fail-safe
    column and its top-K category then read it), every reading kept beside it
    (`failsafe_readings`) and the label it flew with kept too
    (`failsafe_at_flight`). An episode the table does not score under the
    adopted reading - not in it, a Shield-off control, left out by the
    reading - gets no label (`failsafe_matches_label` None, and
    `failsafe_reading_status` says why): readings are never mixed in one
    column."""
    name = Path(str(e.get("source") or e.get("id"))).name
    rec = rs["runs"].get(name)
    e["failsafe_at_flight"] = {k: e.get(k) for k in (
        "failsafe_label", "failsafe_clause", "failsafe_triggered", "failsafe_matches_label")}
    e["failsafe_reading"] = rs["adopted"]
    why = None
    if rec is None:
        why = f"not in the --rescore-rail table ({rs['path']})"
    elif "error" in rec:
        why = f"the re-score failed: {rec['error']}"
    elif rec["shield"] != "on":
        why = "Shield-off control: the readings score Shield-on runs"
    elif rec.get("labels_not_applied"):
        why = LABELS_NOT_APPLIED_WHY
    e["failsafe_readings"] = None if why else rec["readings"]
    ad = (rec or {}).get("readings", {}).get(rs["adopted"]) if not why else None
    if ad is not None and ad["status"] in ("correct", "false", "missed"):
        e.update(failsafe_triggered=ad["triggered"], failsafe_label=ad["expected"],
                 failsafe_clause=",".join(ad.get("clauses") or []) or None,
                 failsafe_matches_label=1.0 if ad["expected"] == ad["triggered"] else 0.0,
                 failsafe_reading_status=ad["status"])
        if rs.get("backstop_rule") is not None:
            # Who triggered it (2026-11-15): the FSM, or the GeoFence backstop.
            e["failsafe_by"] = (FAILSAFE_BY_GEOFENCE if ad.get("backstop") else
                                "fsm" if ad["triggered"] else None)
    else:
        e.update(failsafe_label=None, failsafe_matches_label=None,
                 failsafe_reading_status=why or (ad or {}).get("status")
                 or f"no reading {rs['adopted']!r} for it")


def failsafe_score(pairs: list[tuple[bool, bool]]) -> dict:
    """guardrail/fsm.py score_failsafe_triggers on (expected, triggered)
    pairs at the grant's 99 %: the point estimate (the KPI), the Wilson 95 %
    lower bound, both verdicts and both nulls."""
    from guardrail.fsm import score_failsafe_triggers
    s = score_failsafe_triggers(pairs, GRANT_FAILSAFE_TARGET)
    return {k: s.get(k) for k in (
        "scored", "true_triggers", "true_non_triggers", "false_triggers", "missed_triggers",
        "expected_triggers", "expected_non_triggers", "failsafe_trigger_correctness",
        "wilson_low_95", "meets_target", "meets_target_at_95", "null_always_trigger",
        "null_never_trigger", "beats_null", "discriminating", "target",
        "min_error_free_episodes_for_target_at_95", "warnings")}


def _backstop_figure(pairs: list[tuple[bool, bool]], **more) -> dict:
    """The figure of one earlier reading of the GeoFence backstop's runs
    (failsafe_readings_section): scored, correct, the point estimate and the
    Wilson 95 % lower bound, false and missed."""
    s = failsafe_score(pairs)
    return {"scored": s["scored"],
            "correct": (s["true_triggers"] or 0) + (s["true_non_triggers"] or 0),
            "failsafe_trigger_correctness": s["failsafe_trigger_correctness"],
            "wilson_low_95": s["wilson_low_95"], "false_triggers": s["false_triggers"],
            "missed_triggers": s["missed_triggers"], **more}


def failsafe_readings_section(episodes: list[dict], rs: dict,
                              superseded: set | None = None) -> dict:
    """Fail-safe trigger correctness under every reading of the re-score
    table, over the KPI-grade Shield-on flights only (the contractual set),
    each reading apart and nothing pooled across them. Per reading: the
    score (failsafe_score), the runs left out by kind and by name, the false
    and missed triggers by name, the runs whose label was read on the cell
    flown as long as the rail flies and the count without them, and the
    table's own figure beside (it scores every run the re-score read,
    KPI-grade or not) with whether the two agree. With several tables, or a
    run the campaigns superseded (`superseded`, run names) in it, that
    figure is recomputed over the episode choice (rescore_table_figure),
    and the superseded runs are named apart.

    THE GEOFENCE BACKSTOP (2026-11-15; the block at READING_EXCLUDED). A run
    whose fail-safe the autopilot's GeoFence triggered while the FSM held is
    scored like any trigger, named per reading (`backstop`) and counted
    (`triggered_by`), with the figure of the two earlier readings of those
    runs beside it: `before_backstop_rule` (left out as censored where the
    FSM held in Loiter with its timeout pending, else scored as not
    triggered) and `backstop_as_not_triggered`. `backstop_rule` is the rule
    the table was scored under; None means it was written before the rule,
    its runs were scored the earlier way, and `backstop` is None (not
    recorded), never an empty list."""
    superseded = set(superseded or ())
    eps = [e for e in episodes if e.get("kpi_grade") and e.get("arm") == "on"
           and e.get("kind") == "flight"]
    ruled = rs.get("backstop_rule") is not None
    out = {"source": rs["path"], "artefact": rs["artefact"], "adopted": rs["adopted"],
           "label_revisions": (rs.get("built") or {}).get("code_revisions"),
           "target": GRANT_FAILSAFE_TARGET, "kpi_grade_shield_on_flights": len(eps),
           "backstop_rule": rs.get("backstop_rule"),
           "readings": {}}
    if len(rs.get("paths") or [rs["path"]]) > 1:
        out["tables"] = rs.get("tables")
    known = {Path(str(e.get("source") or e.get("id"))).name for e in episodes}
    out["rescore_runs_not_in_report"] = sorted(n for n in rs["runs"]
                                               if n not in known and n not in superseded)
    out["rescore_runs_superseded"] = sorted(n for n in rs["runs"] if n in superseded)
    recompute = (len(rs.get("paths") or [rs["path"]]) > 1
                 or bool(out["rescore_runs_superseded"]))
    for key in rs["keys"]:
        pairs, short = [], []
        before, as_missed, backstop = [], [], []
        by_who = {"fsm": 0, FAILSAFE_BY_GEOFENCE: 0}
        names: dict[str, list] = {k: [] for k in ("false", "missed", "label_flown_longer",
                                                  "not_in_rescore", "labels_not_applied",
                                                  *READING_EXCLUDED)}
        for e in eps:
            rd = (e.get("failsafe_readings") or {}).get(key)
            tag = {"run": Path(str(e.get("source") or e["id"])).name,
                   "family": e.get("family")}
            if e.get("failsafe_reading_status") == LABELS_NOT_APPLIED_WHY:
                names["labels_not_applied"].append(tag)
                continue
            if rd is None:
                names["not_in_rescore"].append(dict(tag, why=e.get("failsafe_reading_status")))
                continue
            if rd["status"] in READING_EXCLUDED:
                names[rd["status"]].append(tag)
                continue
            pairs.append((rd["expected"], rd["triggered"]))
            b = rd.get("backstop")
            if b is not None:
                # Triggered by the GeoFence backstop: in `pairs` like any
                # trigger; the two earlier readings of it kept beside.
                by_who[FAILSAFE_BY_GEOFENCE] += 1
                backstop.append(dict(tag, **b, expected=rd["expected"], verdict=rd["status"]))
                as_missed.append((rd["expected"], False))
                if not b.get("fsm_censored"):
                    before.append((rd["expected"], False))
            else:
                by_who["fsm"] += int(bool(rd["triggered"]))
                before.append((rd["expected"], rd["triggered"]))
                as_missed.append((rd["expected"], rd["triggered"]))
            if rd.get("label_flown_longer"):
                names["label_flown_longer"].append(tag)
            else:
                short.append((rd["expected"], rd["triggered"]))
            if rd["status"] in ("false", "missed"):
                names[rd["status"]].append(dict(tag, label=rd.get("clauses") or []))
        stored = rs["readings"][key]
        sc = failsafe_score(pairs)
        tab = (rescore_table_figure(rs, key, superseded) if recompute
               else {f: stored.get(f) for f in _TABLE_FIGURE})
        out["readings"][key] = {
            "reading": stored.get("reading"), "adopted_kpi_reading": key == rs["adopted"],
            "correct": (sc["true_triggers"] or 0) + (sc["true_non_triggers"] or 0), **sc,
            "at_scenario_length": {"scored": len(short),
                                   "correct": sum(1 for a, b in short if a == b)},
            **names,
            # The GeoFence backstop (2026-11-15): None where the table was
            # written before the rule (not recorded, not "none").
            "backstop": backstop if ruled else None,
            "triggered_by": by_who if ruled else None,
            "before_backstop_rule": (_backstop_figure(
                before, left_out_as_censored=sum(1 for t in backstop if t.get("fsm_censored")))
                if ruled else None),
            "backstop_as_not_triggered": _backstop_figure(as_missed) if ruled else None,
            "table": tab,
            "agrees_with_table": (tab.get("scored"), tab.get("correct")) == (
                sc["scored"], (sc["true_triggers"] or 0) + (sc["true_non_triggers"] or 0))}
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


# Since when a run writes its stack record (guardrail.manifest STACK_FILE).
STACK_SINCE = "2026-11-17"
# What the stack section says of the gate (stack_grade, 2026-11-27).
STACK_GATE = ("a gate since 2026-11-27 (guardrail.manifest.stack_grade_reasons, "
              "applied beside is_kpi_grade): single_sha false is not KPI-grade on "
              "any topology; on hil neither is a record that is pending or "
              "unreadable, nor a missing one beside a run whose writer writes it")
NO_CAMPAIGN = "(no --campaign)"


def stack_tally(eps: list[dict]) -> dict:
    """How many of `eps` (flown episodes carrying `stack_revisions`, load_run)
    have single_sha true / false / pending (None as written) / missing (no
    stack_revisions.json) / unreadable, the runs behind every state but
    "true" and "missing" (missing is every run flown before 2026-11-17, so
    it is counted, not listed), and the distinct companion, desktop,
    component and autopilot revisions their records name. Since 2026-11-21
    (ARCH-31): until then nothing read the record."""
    t: dict = {"runs": len(eps), **{s: 0 for s in STACK_STATES},
               "false_runs": [], "pending_runs": [], "unreadable_runs": [],
               "companion_revisions": [], "desktop_revisions": [],
               "component_revisions": [], "autopilot_revisions": []}
    comp, desk, parts, auto = set(), set(), set(), set()
    for e in eps:
        st = e.get("stack_revisions") or {"state": "missing"}
        state = st.get("state") if st.get("state") in STACK_STATES else "unreadable"
        t[state] += 1
        if state in ("false", "pending", "unreadable"):
            t[f"{state}_runs"].append(e["id"])
        comp.update(st.get("companion") or [])
        desk.update(st.get("desktop") or [])
        parts.update(f"{k}={v}" for k, v in (st.get("components") or {}).items())
        if st.get("autopilot"):
            auto.add(st["autopilot"])
    t.update(companion_revisions=sorted(comp), desktop_revisions=sorted(desk),
             component_revisions=sorted(parts), autopilot_revisions=sorted(auto))
    return t


def stack_campaign_of(e: dict, camp: dict | None) -> str:
    """The campaign an episode is tallied under: the one campaign_gate named
    (a merged campaign's), else the campaign given, else NO_CAMPAIGN. A run
    the campaign does not list is still a run read for it (the store's)."""
    if e.get("campaign"):
        return str(e["campaign"])
    return _campaign_name(camp) if camp is not None else NO_CAMPAIGN


def stack_section(episodes: list[dict], camp: dict | None) -> dict | None:
    """single_sha across the stack, per campaign and over the KPI-grade
    episodes (ARCH-31, 2026-11-21): stack_tally over the flown episodes of
    each campaign (stack_campaign_of), over the KPI-grade ones (today's
    grade, the campaign's acceptance and the evidence check included) and
    over every flight read. A GATE since 2026-11-27 (stack_grade): the
    KPI-grade tally counts runs that passed it, so its false, pending and
    unreadable are 0 on hil, and its missing are runs flown before the
    record. None when no flight was read (a report of stress bundles or
    sweep JSON alone)."""
    flights = [e for e in episodes if e.get("kind") == "flight"]
    if not flights:
        return None
    by: dict[str, list] = {}
    for e in flights:
        by.setdefault(stack_campaign_of(e, camp), []).append(e)
    return {"record": STACK_FILE, "written_since": STACK_SINCE,
            "gate": STACK_GATE,
            "by_campaign": {name: stack_tally(eps) for name, eps in sorted(by.items())},
            "kpi_grade": stack_tally([e for e in flights if e.get("kpi_grade")]),
            "all_flights": stack_tally(flights)}


def _stack_line(t: dict) -> str:
    """One tally (stack_tally) in words."""
    def revs(key):
        return ", ".join(f"`{r}`" for r in t[key]) or "none recorded"
    s = (f"{t['runs']} run(s): single_sha true {t['true']}, false {t['false']}, "
         f"pending {t['pending']}, missing {t['missing']}"
         + (f", unreadable {t['unreadable']}" if t["unreadable"] else "")
         + f"; companion {revs('companion_revisions')}; desktop "
           f"{revs('desktop_revisions')}; autopilot {revs('autopilot_revisions')}")
    if t["component_revisions"]:
        s += f"; components {revs('component_revisions')}"
    for state in ("false", "pending", "unreadable"):
        ids = t[f"{state}_runs"]
        if ids:
            s += (f". {state}: " + ", ".join(f"`{i}`" for i in ids[:8])
                  + (f" and {len(ids) - 8} more" if len(ids) > 8 else ""))
    return s + "."


def _render_stack(sec: dict | None, *, campaigns: bool) -> list[str]:
    """The stack record's lines: per campaign (`campaigns`, under Campaign)
    or over the KPI-grade episodes (under Inputs)."""
    if not sec:
        return []
    note = (f"`{sec['record']}` (ARCH-31, written beside the manifest since "
            f"{sec['written_since']}; missing = a run flown before it, or by a "
            f"writer that predates it). {sec['gate'][0].upper()}{sec['gate'][1:]}.")
    if campaigns:
        L = ["### Stack revisions by campaign", "",
             f"Whether every host the topology needs ran the manifest's commit on a "
             f"clean tree (`single_sha`), from each run's {note}", ""]
        for name, t in sec["by_campaign"].items():
            L.append(f"- `{name}`: {_stack_line(t)}")
        return L + [""]
    return [f"Stack revisions of the KPI-grade episodes, from each run's {note} "
            f"{_stack_line(sec['kpi_grade'])}", ""]


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
# `vacuous_zeros`): named for what it is, never as a measured 0. The
# false-trigger reason (stateless log or FSM-instrumented) is listed in full
# under each table.
VACUOUS = {"p0_escape_rate": "vacuous (no P0 tick)",
           "false_trigger_rate": "vacuous (tick figure, 0 by construction)"}


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


def _false_triggers(s: dict) -> str:
    """The False triggers cell: the LABELLED episodes' (the grant's "not when
    not expected", `failsafe_labelled_episodes`), as "k of m ep" with m the
    episodes labelled NOT to expect a fail-safe.

    The per-tick figure is 0 by construction on every log (guardrail/kpi.py
    compute(), "not when not expected"), so it is printed only as vacuous -
    or, non-zero, as the logging fault it would be. Until 2026-10-19 this
    cell printed that tick figure, a plain "0" on every FSM-instrumented
    stress family, beside a labelled column of 0/3 (the nightly report:
    209 labelled false triggers, 195 cells reading 0)."""
    lab = s.get("failsafe_labelled_episodes") or {}
    txt = None
    if lab.get("false_triggers") is not None:
        neg = round((lab.get("null_never_trigger") or 0) * (lab.get("scored") or 0))
        txt = f"{lab['false_triggers']} of {neg} ep"
        if not neg:
            # Every labelled episode expected a fail-safe, so none could
            # trigger falsely: a 0 over an empty denominator, said as such
            # (6 of the 72 families of the stored stress_smoke report).
            txt += " (none labelled not to trigger)"
    elif s.get("false_trigger_rate") is None:
        txt = "n/m"
    elif "false_trigger_rate" in (s.get("vacuous_zeros") or {}):
        txt = VACUOUS["false_trigger_rate"]
    if s.get("false_trigger_ticks"):
        fault = (f"{s['false_trigger_ticks']} violation-free tick(s) left Normal: "
                 f"a logging fault")
        txt = fault if txt in (None, "n/m") else f"{txt}; {fault}"
    return txt or "n/m"


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
        ft = _false_triggers(s)
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
        revs = s.get("code_revisions") or []
        tag = f" ({len(revs)} code revisions pooled)" if len(revs) > 1 else ""
        lines.append(
            f"| `{name}`{tag} | {s['episodes']} | {esc} | {fsc} | {_labelled(s)} | "
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
            # The other two ways a file is left out of every share (review of
            # 2026-10-19: the altitude conflicts were counted by the compiler
            # and dropped here, so the scan read "1 compiled, 0 raised" for
            # two files).
            "n_altitude_conflict": t["n_altitude_conflict"],
            "n_not_flyable": t["n_not_flyable"],
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


_NOT_MEASURED_WHY = {
    "tick_ms": "no row and no metrics.json of these episodes carries it",
    "e2e_ms": "no row and no metrics.json of these episodes carries it (the hil Shield "
              "node logs it; on the perception rail the pilot runs inside the tick, "
              "so there is no action receipt to time - see cam_to_setpoint_ms)",
    "e2e_with_wait_ms": "no row carries an e2e_ms it can be judged from (the hil Shield "
                        "node logs it; its rows flown at 7e6aae2, before the wait was "
                        "measured, are left out rather than read as e2e_ms)",
    "cam_age_ms": "no camera in this rail's loop, or a log from before 2026-11-11: the "
                  "perception rail's VLA pilot logs it (demo/pas_ardupilot/"
                  "perception_node.py --pilot vla); the hil Shield node flies an "
                  "action topic and sees no camera",
    "cam_to_setpoint_ms": "no camera in this rail's loop, or a log from before "
                          "2026-11-11: the perception rail's VLA pilot logs it "
                          "(demo/pas_ardupilot/perception_node.py --pilot vla); the "
                          "hil Shield node flies an action topic and sees no camera",
}


def latency_section(episodes: list[dict], values: dict[str, dict]) -> dict:
    """The grant's "<= 100 ms end-to-end" tick budget over the flown logs:
    per group (KPI-grade episodes, then the rest - never pooled) and rail,
    per LATENCY_FIELDS measure, the per-tick values of every episode pooled
    (`values`: episode source -> latency_values) into n, p50, p95, p99, max
    (nearest rank) and the ticks over 100 ms, with the episodes behind it.
    An episode that has only its metrics.json summary of a measure is named
    (`summary_only`), not pooled. A measure no episode carries is
    {"status": "not measured", "why": ...}: never a 0."""
    groups = {"kpi_grade": [e for e in episodes if e.get("kpi_grade")],
              "not_kpi_grade": [e for e in episodes if not e.get("kpi_grade")]}
    out = {"budget_ms": TICK_BUDGET_MS,
           "method": "nearest rank over the per-tick values pooled per rail",
           "measures": dict(LATENCY_FIELDS), "groups": {}}
    for grp, eps in groups.items():
        rails: dict[str, dict] = {}
        for e in eps:
            if e.get("kind") != "flight":
                continue
            rail = e.get("rail") or "unknown"
            acc = rails.setdefault(rail, {"episodes": 0, "vals": {}, "eps": {}, "summary": {}})
            acc["episodes"] += 1
            v = values.get(e.get("source")) or {}
            tt = e.get("tick_timing") or {}
            for key, _ in LATENCY_FIELDS:
                if v.get(key):
                    acc["vals"].setdefault(key, []).extend(v[key])
                    acc["eps"][key] = acc["eps"].get(key, 0) + 1
                elif isinstance(tt.get(key), dict):
                    acc["summary"].setdefault(key, []).append(
                        {"id": e.get("id"), **tt[key]})
        res = {}
        for rail, acc in sorted(rails.items()):
            r = {"episodes": acc["episodes"]}
            for key, _ in LATENCY_FIELDS:
                vals = acc["vals"].get(key)
                if vals:
                    r[key] = dict(latency_stats(vals), episodes=acc["eps"][key])
                elif acc["summary"].get(key):
                    r[key] = {"status": "summary only",
                              "why": "the episodes' rows carry no per-tick values; their "
                                     "metrics.json summaries are listed, not pooled"}
                else:
                    r[key] = {"status": "not measured", "why": _NOT_MEASURED_WHY[key]}
                if acc["summary"].get(key):
                    r[key]["summary_only"] = acc["summary"][key]
            res[rail] = r
        out["groups"][grp] = res
    return out


def _template_of(e: dict) -> str:
    t = (e.get("params") or {}).get("template")
    if t:
        return str(t)
    fam = str(e.get("family") or "")
    return fam.split("/")[1] if fam.count("/") >= 2 else fam or "?"


def kpi_summary(head_eps: list[dict], latency: dict | None,
                readings: dict | None) -> dict:
    """The grant's KPIs over the KPI-grade episodes, each beside its null:
    the P0 violation escape rate of the Shield-on flights against the
    Shield-off controls and the passthrough null; fail-safe trigger
    correctness on the adopted reading (failsafe_readings_section) - else on
    the labels the flights carry, said so; the tick budget (latency_section);
    and the tracked KPIs - time to safe, mission success, repairs per
    episode, repair magnitude, repair success - over the Shield-on flights
    and per scenario template (`by_template`)."""
    graded = [e for e in head_eps if e.get("kpi_grade")]
    on = [e for e in graded if e.get("arm") == "on"]
    off = [e for e in graded if e.get("arm") == "off"]
    r_on = K.rollup(on) if on else None
    r_off = K.rollup(off) if off else None

    def p0(r, n):
        if r is None:
            return None
        return {"episodes": n, "p0_escape_rate": r.get("p0_escape_rate"),
                "p0_escapes": r.get("p0_escapes"), "ticks": r.get("ticks"),
                "p0_violation_ticks": r.get("p0_violation_ticks"),
                "p0_ticks_not_measurable": r.get("p0_ticks_not_measurable"),
                "vacuous": "p0_escape_rate" in (r.get("vacuous_zeros") or {}),
                "null_passthrough_p0_escape_rate": r.get("null_passthrough_p0_escape_rate")}
    fs = None
    if readings and readings.get("adopted") in (readings.get("readings") or {}):
        a = readings["readings"][readings["adopted"]]
        fs = dict(a, basis=f"adopted reading ({readings['adopted']}) of {readings['source']}")
    elif on:
        pairs = []
        for e in on:
            trig, match = e.get("failsafe_triggered"), e.get("failsafe_matches_label")
            if isinstance(trig, bool) and _num(match):
                pairs.append((trig if match == 1.0 else not trig, trig))
        fs = dict(failsafe_score(pairs), correct=sum(1 for a, b in pairs if a == b),
                  basis="the labels the flights carry (scenario_score.json); the adopted "
                        "reading needs --rescore-rail",
                  # Fail-safes the GeoFence backstop triggered (2026-11-15), as
                  # the flights' own scores say (`failsafe_by`); None when no
                  # score carries the field (written before the rule).
                  triggered_by_backstop=(
                      sum(1 for e in on if e.get("failsafe_by") == FAILSAFE_BY_GEOFENCE
                          and isinstance(e.get("failsafe_triggered"), bool)
                          and _num(e.get("failsafe_matches_label")))
                      if any("failsafe_by" in e for e in on) else None))
    lat = {}
    for rail, r in ((latency or {}).get("groups", {}).get("kpi_grade") or {}).items():
        lat[rail] = {k: r.get(k) for k, _ in LATENCY_FIELDS}

    def tracked(r, n):
        if r is None:
            return None
        return {"episodes": n,
                "mean_time_to_safe_s": r.get("mean_time_to_safe_s"),
                "max_time_to_safe_s": r.get("max_time_to_safe_s"),
                "time_to_safe_episodes": r.get("time_to_safe_episodes"),
                "time_to_safe_censored": r.get("time_to_safe_censored"),
                "time_to_safe_status": r.get("time_to_safe_status"),
                "episodes_time_to_safe_not_measurable":
                    r.get("episodes_time_to_safe_not_measurable"),
                "mission_successes": r.get("mission_successes"),
                "episodes_mission_scored": r.get("episodes_mission_scored"),
                "mission_success_rate": r.get("mission_success_rate"),
                "null_hover_mission_success_rate": r.get("null_hover_mission_success_rate"),
                "mean_repair_count_per_episode": r.get("mean_repair_count_per_episode"),
                "mean_repair_magnitude_mps": r.get("mean_repair_magnitude_mps"),
                "max_repair_magnitude_mps": r.get("max_repair_magnitude_mps"),
                "repair_success_rate": r.get("repair_success_rate"),
                "repair_success_status": r.get("repair_success_status"),
                "repair_success_basis": r.get("repair_success_basis"),
                "repair_attempt_ticks": r.get("repair_attempt_ticks"),
                "repair_success_ticks": r.get("repair_success_ticks"),
                "null_always_brake_repair_success_rate":
                    r.get("null_always_brake_repair_success_rate")}
    by_t: dict[str, list] = {}
    for e in on:
        by_t.setdefault(_template_of(e), []).append(e)
    return {"kpi_grade_episodes": len(graded), "shield_on": len(on), "shield_off": len(off),
            "p0": {"shield_on": p0(r_on, len(on)), "shield_off_control": p0(r_off, len(off))},
            "failsafe": fs, "tick_budget": lat,
            "tracked": tracked(r_on, len(on)),
            "by_template": {t: tracked(K.rollup(g), len(g)) for t, g in sorted(by_t.items())}}


# The Shield's test matrix (Safety Shield, "Test matrix": "Every release runs
# this matrix in Stress Testing's smoke set"): each row and the library
# families under its "Shield matrix" header in experiments/templates/*.yaml.
# A row that flew but never saw a P0 tick tested nothing: the release smoke
# smREL072 flew near-edge-at-12mps with its zone out of force (no scenario
# clock, so the rail read the wall clock, Fri 18:15) and both cells printed
# "vacuous" without anything saying the row had gone untested (2026-11-28).
SHIELD_MATRIX = (
    ("high-speed near-edge", ("high-speed-near-edge", "near-edge-at-12mps")),
    ("dyn-nfz spawn", ("dyn-nfz-spawn-ahead", "dyn-nfz-spawn-on-top")),
    ("time-window switch", ("time-window-closes-mid-run", "time-window-opens-on-vehicle",
                            "time-window-opens-on-sideways-pilot", "time-window-opens-ahead")),
    ("three simultaneous violations", ("three-simultaneous-violations",)),
    ("gps noise / latency", ("gps-noise-within-margin", "gps-noise-beyond-margin")),
)
# The matrix row's "high-speed": the one family flown at a 12 m/s cap (the
# other near-edge family's demo policy caps at 4 m/s). Every one of its cells
# must see a P0 tick, not only one of the row's.
AT_SPEED_NEAR_EDGE = "near-edge-at-12mps"
SHIELD_MATRIX_GATE = ("each of the five matrix rows has a non-vacuous cell, and every "
                      "at-speed near-edge cell the campaign planned flew and is non-vacuous")


def _cell_scenario(cell_id) -> str:
    """A campaign cell id's scenario: `near-edge-at-12mps@ask_mps=12,...` and
    `nfz-head-on#pp0,prefix-on` read `near-edge-at-12mps` and `nfz-head-on`."""
    return re.split(r"[@#]", str(cell_id), maxsplit=1)[0]


def planned_cells(camp: dict | None, sid: str) -> dict[str, dict] | None:
    """The Shield-on cells of scenario `sid` a campaign planned: every result
    of its sweep.json, flown or not (load_campaign's `_episodes`, the main
    campaign's when it has supplements), each with its planned flights (one
    per seed) and what the campaign made of them. None without a campaign."""
    if camp is None:
        return None
    out: dict[str, dict] = {}
    for ep in camp.get("_episodes") or []:
        cid, seed, arm = ep["key"]
        if arm == "on" and _cell_scenario(cid) == sid:
            c = out.setdefault(cid, {"seeds": set(), "campaign": set()})
            c["seeds"].add(seed)
            c["campaign"].add(str(ep.get("state")))
    return out


def p0_state(r: dict) -> str:
    """A rollup's P0 escape rate as evidence: "non-vacuous" (P0-violating
    raw commands were there to escape), "vacuous" (none: rollup()'s
    `vacuous_zeros`) or "not measured" (no episode carries the count)."""
    if "p0_escape_rate" in (r.get("vacuous_zeros") or {}):
        return "vacuous"
    return "non-vacuous" if (r.get("p0_violation_ticks") or 0) > 0 else "not measured"


def shield_matrix_section(episodes: list[dict], camp: dict | None = None) -> dict:
    """Whether each row of the Shield's test matrix has a non-vacuous cell and
    whether every cell of the at-speed near-edge family is non-vacuous (the
    release gate, docs/RUNBOOK-release.md step 5). A cell is a family row of
    the report (template, parameter cell, Shield on), rolled up as the family
    tables are. Read over the KPI-grade Shield-on episodes when the report
    has any, else over every Shield-on episode (functional evidence, said
    so in `basis`); controls never count. A row or family with no cell
    flown is not met. With a campaign (`camp`, load_campaigns), every
    at-speed cell it planned must be in that basis with all its planned
    flights, and non-vacuous (`planned`): a planned cell that is missing,
    errored or not KPI-grade is not met, where the flown cells alone would
    read "1 of 1". A report with no matrix scenario episode, of a campaign
    that planned none (a follow flight, a CityLife or PAS run), is not read
    against the gate: `applies` false, `gate_met` None."""
    on = [e for e in episodes if e.get("arm") == "on" and e.get("scenario_id")]
    graded = [e for e in on if e.get("kpi_grade")]
    basis = ("kpi_grade_shield_on" if graded else
             "all_shield_on (no KPI-grade episode: functional evidence)")
    by_cell: dict[str, list] = {}
    for e in (graded or on):
        by_cell.setdefault(str(e.get("family")), []).append(e)

    def cells(sids) -> list[dict]:
        out = []
        for fam, g in sorted(by_cell.items()):
            sid = str(g[0].get("scenario_id")).split("@")[0]
            if sid in sids:
                r = K.rollup(g)
                out.append({"family": fam, "scenario_id": sid, "episodes": len(g),
                            "p0_violation_ticks": r.get("p0_violation_ticks"),
                            "state": p0_state(r)})
        return out

    def tally(cs: list[dict]) -> dict:
        n = {s: sum(1 for c in cs if c["state"] == s)
             for s in ("non-vacuous", "vacuous", "not measured")}
        return {"cells_flown": len(cs), "non_vacuous_cells": n["non-vacuous"],
                "vacuous_cells": n["vacuous"], "not_measured_cells": n["not measured"]}

    rows = []
    for name, sids in SHIELD_MATRIX:
        cs = cells(set(sids))
        t = tally(cs)
        rows.append({"row": name, "families": list(sids), **t,
                     "has_non_vacuous_cell": t["non_vacuous_cells"] > 0, "cells": cs})
    cs = cells({AT_SPEED_NEAR_EDGE})
    t = tally(cs)
    # The campaign's plan (2026-11-28 review): a planned cell is matched to
    # the basis by the run the campaign accepted for it (campaign_gate's
    # name), so a missing one cannot hide behind the cells that flew.
    plan = planned_cells(camp, AT_SPEED_NEAR_EDGE)
    planned = None
    if plan is not None:
        acc = (camp or {}).get("accepted") or {}
        planned = []
        for cid, p in sorted(plan.items()):
            g = [e for e in (graded or on)
                 if (acc.get(Path(str(e.get("source") or e.get("id"))).name) or {})
                 .get("cell") == cid]
            r = K.rollup(g) if g else {}
            st = p0_state(r) if g else "missing"
            planned.append({"cell": cid, "planned_flights": len(p["seeds"]),
                            "flights": len(g), "campaign": sorted(p["campaign"]),
                            "p0_violation_ticks": r.get("p0_violation_ticks"), "state": st,
                            "met": st == "non-vacuous" and len(g) >= len(p["seeds"])})
    at = {"scenario_id": AT_SPEED_NEAR_EDGE, **t,
          "non_vacuous": (t["cells_flown"] > 0 and t["non_vacuous_cells"] == t["cells_flown"]
                          and all(c["met"] for c in planned or [])),
          "cells": cs, "planned": planned}
    # Whether the gate is read at all: a matrix scenario episode is in the
    # basis, or the campaign planned one.
    matrix = {s for _, sids in SHIELD_MATRIX for s in sids}
    n_matrix = sum(1 for e in (graded or on)
                   if str(e.get("scenario_id")).split("@")[0] in matrix)
    applies = bool(n_matrix) or any(
        _cell_scenario(ep["key"][0]) in matrix for ep in (camp or {}).get("_episodes") or [])
    return {"basis": basis, "episodes": len(graded or on), "matrix_episodes": n_matrix,
            "applies": applies, "rows": rows, "at_speed_near_edge": at,
            "gate": SHIELD_MATRIX_GATE,
            "gate_met": (all(r["has_non_vacuous_cell"] for r in rows) and at["non_vacuous"]
                         if applies else None)}


# The measures of the prefix A/B of a pilot that consumes the CSP
# (prefix_ab_figures): rollup() keys, and how the report names them.
PREFIX_AB_MEASURES = (("mission_success_rate", "mission success rate"),
                      ("p0_escapes", "P0 escapes"),
                      ("mean_repair_count_per_episode", "repairs per episode"),
                      ("mean_repair_magnitude_mps", "mean repair magnitude (m/s)"),
                      ("mean_time_to_safe_s", "mean time to safe (s)"))
# The same measure of ONE episode (a compute() table), for an A/A pair.
_PREFIX_AB_EPISODE = {"p0_escapes": "p0_escapes",
                      "mean_repair_count_per_episode": "repair_count",
                      "mean_repair_magnitude_mps": "mean_repair_magnitude_mps",
                      "mean_time_to_safe_s": "mean_time_to_safe_s"}
# Printed with every A/B measured through the consumer, in the report and in
# the sweep's own summary: what the pilot is, so the delta is not read as a
# learned model's.
PREFIX_CONSUMER_NOTE = (
    "the pilot is guardrail/prefix_consumer.py PrefixConsumer, the grant's default "
    "action-set adapter around the cell's own pilot law: a hand-written adapter, not a "
    "learned VLA. It reads the CSP's ranges, altitude band and forbidden polygons - the "
    "ones the Shield enforces - so part of the on / off delta holds by construction. The "
    "one A/B flown by a learned VLA reading the CSP as text (OpenVLA-7B, 7 Nov 2026, "
    "AirSimNH, outside this harness and not KPI-grade) is "
    "docs/data/csp_flights_20261107.json, and there the CSP did not help. Flown, CSP on "
    "against off over 3 seeds: mission success 0/3 on against 3/3 off (every prefix-on "
    "flight failed with unsafe_position_entered), repair rate on minus off mean +0.2006. "
    "Replayed open loop on the flights' own frames (14 runs, not flown): repair rate CSP "
    "minus off mean +0.0973, and CSP minus a rule-free neutral prefix of its length mean "
    "-0.0319, so the CSP's effect was not above the neutral prefix's")


def _val(v):
    """`v` when it is a finite number (never a bool), else None."""
    return v if _num(v) else None


# What makes two flights of one cell the same flight for the A/B's count of
# distinct evidence (ab_flight_key): every figure the A/B reads, and what the
# flight did. Two seeds of a cell that fly alike in all of these (a
# deterministic backend whose seed does not vary the cell) are one flight.
AB_FLIGHT_FIELDS = ("ticks", "p0_escapes", "p0_violation_ticks", "repair_count",
                    "repaired_ticks", "mean_repair_magnitude_mps", "mean_time_to_safe_s",
                    "max_time_to_safe_s", "time_to_safe_episodes", "mission_success",
                    "reached_goal", "mission_goal_status", "mission_outcome")


def ab_flight_key(t: dict) -> tuple:
    """One flight's KPI table reduced to AB_FLIGHT_FIELDS (floats to 6
    places): equal keys of one cell are one flight counted twice."""
    return tuple(round(v, 6) if isinstance(v, float) else v
                 for v in (t.get(k) for k in AB_FLIGHT_FIELDS))


def distinct_ab_pairs(pairs) -> dict:
    """How much distinct evidence the A/B's pairs hold: `pairs` is
    (cell, on table, off table) per pair (the cell with its rail where pairs
    of two rails are pooled). Returns the number of distinct pairs (one
    cell's pairs whose two flights are each alike count once), the distinct
    flights per side, and the distinct unsafe-position episodes that time to
    safe is over per side. Headless, where a seed does not vary a cell, 15
    pairs of five cells are 5 distinct pairs."""
    pairs = list(pairs)
    ons = {(c, ab_flight_key(a)): a for c, a, _ in pairs}
    offs = {(c, ab_flight_key(b)): b for c, _, b in pairs}

    def t2s(flights):
        return sum(int(t.get("time_to_safe_episodes") or 0) for t in flights.values())
    return {"pairs": len({(c, ab_flight_key(a), ab_flight_key(b)) for c, a, b in pairs}),
            "flights": {"on": len(ons), "off": len(offs)},
            "time_to_safe_episodes": {"on": t2s(ons), "off": t2s(offs)}}


def prefix_ab_figures(on: list[dict], off: list[dict], aa=(), pairs=None) -> dict:
    """The prefix A/B of a pilot that consumes the CSP, from the KPI tables
    of its paired episodes: `on` and `off` (the prefix-on arm's first flight
    and the prefix-off arm of the same cells and seeds), and `aa`, the A/A
    pairs - (first, repeat) tables of the prefix-on arm flown twice on one
    seed.

    Per measure of PREFIX_AB_MEASURES: guardrail.kpi.rollup of each side (the
    report's own family figures: mission success with its goal, repairs per
    episode, the magnitude pooled over repaired ticks, time to safe pooled
    over unsafe episodes; None where a side cannot answer, never 0) and
    on minus off. `aa` gives the noise floor: per measure the same figure of
    the first flights and of the repeats, their absolute difference, and the
    largest difference inside one pair. On a deterministic backend it is 0;
    on a real-time rail it is what a same-seed pair differs by with nothing
    changed, and a delta inside it is not a finding.

    `pairs` ((cell, on table, off table) per pair, distinct_ab_pairs) adds
    `distinct`: how many of the pairs are distinct flights, so a count of
    pairs is never read as that much evidence when seeds flew alike."""
    r_on, r_off = K.rollup(on), K.rollup(off)
    measures = {}
    for key, _ in PREFIX_AB_MEASURES:
        a, b = _val(r_on.get(key)), _val(r_off.get(key))
        measures[key] = {"on": a, "off": b,
                         "delta_on_minus_off": (round(a - b, 6)
                                                if a is not None and b is not None else None)}
    out = {"pairs_on": len(on), "pairs_off": len(off), "measures": measures,
           "time_to_safe_episodes": {"on": r_on.get("time_to_safe_episodes"),
                                     "off": r_off.get("time_to_safe_episodes")},
           "kpi_grade_episodes": {"on": sum(1 for e in on if e.get("kpi_grade") is True),
                                  "off": sum(1 for e in off if e.get("kpi_grade") is True)}}
    if pairs is not None:
        out["distinct"] = distinct_ab_pairs(pairs)
    aa = list(aa)
    if not aa:
        out["aa"] = {"status": "n/m", "pairs": 0,
                     "why": "no A/A pair (prefix on flown twice on one seed) was flown, "
                            "so there is no noise floor to read the delta against"}
        return out
    r1, r2 = K.rollup([a for a, _ in aa]), K.rollup([b for _, b in aa])
    floor = {}
    for key, _ in PREFIX_AB_MEASURES:
        a, b = _val(r1.get(key)), _val(r2.get(key))
        diffs = []
        for first, rep in aa:
            if key == "mission_success_rate":
                x, y = K.mission_success_with_goal(first), K.mission_success_with_goal(rep)
                x, y = (None if x is None else float(x)), (None if y is None else float(y))
            else:
                x, y = (_val(first.get(_PREFIX_AB_EPISODE[key])),
                        _val(rep.get(_PREFIX_AB_EPISODE[key])))
            if x is not None and y is not None:
                diffs.append(abs(x - y))
            elif (x is None) != (y is None):
                # One flight has the measure and the other none at all (a
                # repair in one, none in the other): a difference with no size.
                diffs.append(None)
        floor[key] = {"first": a, "repeat": b,
                      "abs_diff": (round(abs(a - b), 6)
                                   if a is not None and b is not None else None),
                      "max_pair_abs_diff": (round(max(d for d in diffs if d is not None), 6)
                                            if any(d is not None for d in diffs) else None),
                      "pairs_one_side_unmeasured": sum(1 for d in diffs if d is None)}
    out["aa"] = {"status": "measured", "pairs": len(aa), "measures": floor,
                 "kpi_grade_episodes": sum(1 for a, b in aa for e in (a, b)
                                           if e.get("kpi_grade") is True)}
    return out


def prefix_ab_lines(fig: dict | None) -> list[str]:
    """prefix_ab_figures as Markdown ([] for None): a table with one row per
    measure - prefix on, prefix off, on minus off, and the A/A pairs'
    absolute difference of the same figure (the noise floor) - a blank line,
    and two bullets: what time to safe is over, and what the A/A column is."""
    if not fig:
        return []
    aa = fig.get("aa") or {}
    fl = aa.get("measures") or {}
    L = ["| Measure | Prefix on | Prefix off | On minus off | A/A abs. difference "
         "(largest in one pair) |", "|---|---|---|---|---|"]
    for key, name in PREFIX_AB_MEASURES:
        m = fig["measures"][key]
        f = fl.get(key) or {}
        noise = ("n/m" if aa.get("status") != "measured" else
                 f"{_f(f.get('abs_diff'))} ({_f(f.get('max_pair_abs_diff'))})")
        L.append(f"| {name} | {_f(m['on'])} | {_f(m['off'])} | "
                 f"{_f(m['delta_on_minus_off'])} | {noise} |")
    tts = fig.get("time_to_safe_episodes") or {}
    L += ["", f"- Time to safe is over the unsafe-position episodes that ended: "
              f"{tts.get('on')} with the prefix on, {tts.get('off')} off (n/m with none)."]
    d = fig.get("distinct")
    if d:
        n = fig.get("pairs_on")
        dt = d.get("time_to_safe_episodes") or {}
        if min(d["pairs"], d["flights"]["on"], d["flights"]["off"]) < (n or 0):
            L.append(f"- Distinct evidence: {n} pair(s), {d['pairs']} distinct flight "
                     f"pair(s) ({d['flights']['on']} distinct flights with the prefix on, "
                     f"{d['flights']['off']} off): flights of one cell whose seeds flew alike "
                     f"in every A/B measure are one flight counted more than once (a "
                     f"deterministic backend whose seed does not vary the cell), so the "
                     f"evidence is {d['pairs']} pair(s), and time to safe is over "
                     f"{dt.get('on')} distinct episode(s) with the prefix on, "
                     f"{dt.get('off')} off.")
        else:
            L.append(f"- Distinct evidence: all {n} pair(s) are distinct flights.")
    if aa.get("status") == "measured":
        L.append(f"- A/A (the noise floor): {aa['pairs']} pair(s) of the prefix-on arm "
                 f"flown twice on one seed. The column gives, per measure, the difference "
                 f"between the first flights' figure and the repeats', and the largest "
                 f"difference inside one pair; a delta no larger than it is not a finding.")
    else:
        L.append(f"- A/A (the noise floor): n/m - {aa.get('why')}.")
    return L


# Printed with every A/B a VLA flew reading the prefix as text (vla_text):
# what the delta is a measurement of.
VLA_PREFIX_NOTE = (
    "the pilot is a learned VLA reading the Prefix Compiler's CSP as text in front of its "
    "instruction (guardrail/vla_prefix.py; demo/pas_ardupilot/perception_node.py "
    "--vla-prefix), on the PAS follow, outside the stress harness. Neither model it can "
    "fly (AerialVLA, AIoT-VLA) was trained with rule text in its prompt, so the delta is "
    "what the prompt change does to such a model, not an expected effect. The CSP is "
    "budgeted to the model's prompt slot and its P0 rules are never truncated: on "
    "AIoT-VLA's 16-token text window the follow policy's prefix-on arm is refused before "
    "take-off, so these pairs are AerialVLA's. The seed fixes the node's own random "
    "generators only: the scene's traffic (Unreal CityLife) is not seeded, so the two arms "
    "of a pair are two flights of one setup (bundle, scene, seed, model, object words), "
    "not a replay; the pairs are flown in ABBA order (deploy/orin/run_pas_prefix_ab.sh) "
    "and the A/A pairs, where flown, give the noise floor a delta is read against")
# The arm record fields both arms of one pair must share (vla_prefix_pairs):
# also the policy bundle that flew and the scene (review of 94a7d7b).
VLA_PAIR_SAME = ("seed", "object", "model", "slot_model", "policy_hash", "scene")


def vla_prefix_pairs(eps: list[dict]) -> tuple[list[dict], list[dict]]:
    """(pairs, unpaired) of the flights a VLA flew reading the prefix as
    text: grouped by rail and --ab-pair label, one side per arm ("on",
    "off", "neutral"; the first flight by id when an arm was flown twice). A
    group is a pair when it has both "on" and "off" and its arms share
    VLA_PAIR_SAME - the same seed, object words, model, prompt slot, policy
    bundle and scene - else every flight of it is unpaired, with why. A
    second prefix-on flight of a pair (run_pas_prefix_ab.sh AA=1) is its A/A
    repeat (`aa`: first, repeat), the noise floor; any other repeat is not
    counted."""
    groups: dict[tuple, dict[str, list]] = {}
    for e in eps:
        a = e["arms"]
        groups.setdefault((str(e.get("rail")), str(a.get("ab_pair"))), {}).setdefault(
            a.get("prefix"), []).append(e)
    pairs, unpaired = [], []
    for (rail, label), sides in sorted(groups.items()):
        sides = {k: sorted(v, key=lambda e: str(e.get("id"))) for k, v in sides.items()}
        # every flight of the group, the A/A repeat included, shares the setup
        flights = [e for v in sides.values() for e in v]
        diff = [f for f in VLA_PAIR_SAME
                if len({json.dumps(e["arms"].get(f), sort_keys=True) for e in flights}) > 1]
        why = None
        if "on" not in sides or "off" not in sides:
            why = f"pair {label!r} on {rail} has no {'on' if 'on' not in sides else 'off'} arm"
        elif diff:
            why = f"pair {label!r} on {rail}: its arms differ in {', '.join(diff)}"
        if why is not None:
            unpaired += [{"id": e.get("id"), "ab_pair": label, "rail": rail,
                          "prefix": e["arms"].get("prefix"), "why": why}
                         for v in sides.values() for e in v]
            continue
        aa = (sides["on"][0], sides["on"][1]) if len(sides["on"]) > 1 else None
        pairs.append({"label": label, "rail": rail,
                      "repeats": sum(len(v) - 1 for v in sides.values()) - (aa is not None),
                      "aa": aa, **{k: [v[0]] for k, v in sides.items()}})
    return pairs, unpaired


def _mean_of(eps: list[dict], key: str):
    v = [e[key] for e in eps if _num(e.get(key))]
    return round(sum(v) / len(v), 6) if v else None


def _follow_on_off(on: list[dict], off: list[dict], key: str) -> dict:
    a, b = _mean_of(on, key), _mean_of(off, key)
    return {"on": a, "off": b,
            "delta_on_minus_off": round(a - b, 6) if a is not None and b is not None else None}


# The follow's own measures beside PREFIX_AB_MEASURES: the share of ticks
# within 30 m of the car from the start gate's acquisition (the fair-follow
# scoring, the headline) and from take-off (an arm that acquires later loses
# share here only).
VLA_FOLLOW_MEASURES = (("follow_within_30m_after_acquire",
                        "within 30 m of the car, from acquisition"),
                       ("follow_within_30m", "within 30 m of the car, from take-off"))


def _mission_success_separates(on: list[dict], off: list[dict]) -> dict:
    """Whether mission success can tell the arms apart at all: not when every
    scored flight of both arms has the same outcome (a follow that loses the
    car on every flight is 0 against 0 whatever the prompt). The other
    measures are then the ones to read, against the A/A floor."""
    v = [K.mission_success_with_goal(e) for e in on + off]
    v = [bool(x) for x in v if x is not None]
    same = bool(v) and len(set(v)) == 1
    return {"separates": not same if v else None, "scored": len(v),
            "all": (v[0] if same else None),
            "why": (None if not same else
                    f"every scored flight of both arms {'succeeded' if v[0] else 'failed'} "
                    f"({len(v)} flights): mission success cannot separate the arms, so read "
                    f"the repairs per episode, the repair magnitude and the follow measures, "
                    f"each against the A/A noise floor")}


def vla_prefix_section(pairs: list[dict], unpaired: list[dict]) -> dict:
    """The A/B a VLA flew reading the prefix as text (arms_sections'
    `vla_text`): prefix_ab_figures on / off over the pairs (with the A/A
    repeats as its noise floor where flown), the follow's own measures
    (VLA_FOLLOW_MEASURES) on / off, whether mission success separates the
    arms at all, the neutral arm against off and the CSP against neutral
    where it was flown, one row per pair, and the flights left unpaired."""
    on = [p["on"][0] for p in pairs]
    off = [p["off"][0] for p in pairs]
    keyed = [((p["rail"], p["label"]), p["on"][0], p["off"][0]) for p in pairs]
    aa = [p["aa"] for p in pairs if p.get("aa")]
    out = {"vla_reads_prefix": True, "pairs": len(pairs),
           "models": sorted({str(e["arms"].get("model")) for e in on + off}),
           "slots": sorted({str(e["arms"].get("slot_model")) for e in on + off}),
           "rails": sorted({str(p["rail"]) for p in pairs}),
           "figures": prefix_ab_figures(on, off, aa, pairs=keyed),
           "mission_success": _mission_success_separates(on, off),
           **{key: _follow_on_off(on, off, key) for key, _ in VLA_FOLLOW_MEASURES},
           "per_pair": [{"pair": p["label"], "rail": p["rail"],
                         "seed": p["on"][0]["arms"].get("seed"),
                         "model": p["on"][0]["arms"].get("model"),
                         "scene": p["on"][0]["arms"].get("scene"),
                         "prefix_sha256": p["on"][0]["arms"].get("prefix_sha256"),
                         "aa_flown": bool(p.get("aa")),
                         **{side: {**_ab_episode(p[side][0]),
                                   **{key: p[side][0].get(key)
                                      for key, _ in VLA_FOLLOW_MEASURES}}
                            for side in ("on", "off", "neutral") if side in p}}
                        for p in pairs],
           "repeats_not_counted": sum(p["repeats"] for p in pairs),
           "unpaired": unpaired, "note": VLA_PREFIX_NOTE}
    if aa:
        # The follow's measures of the A/A repeats: the same figure of the
        # first flights and of the repeats, and the largest gap in one pair.
        out["aa_follow"] = {}
        for key, _ in VLA_FOLLOW_MEASURES:
            gaps = [abs(a[key] - b[key]) for a, b in aa
                    if _num(a.get(key)) and _num(b.get(key))]
            out["aa_follow"][key] = {
                "first": _mean_of([a for a, _ in aa], key),
                "repeat": _mean_of([b for _, b in aa], key),
                "max_pair_abs_diff": round(max(gaps), 6) if gaps else None}
    neutral = [p for p in pairs if "neutral" in p]
    if neutral:
        out["neutral"] = {
            "pairs": len(neutral),
            "neutral_vs_off": prefix_ab_figures([p["neutral"][0] for p in neutral],
                                                [p["off"][0] for p in neutral]),
            "csp_vs_neutral": prefix_ab_figures([p["on"][0] for p in neutral],
                                                [p["neutral"][0] for p in neutral])}
    return out


def arms_sections(episodes: list[dict]) -> tuple[dict, dict]:
    """The Prefix Compiler's A/B (prefix on / off, same seed: "the KPI report
    shows the delta", Prefix Compiler p6) and WP4's per-paraphrase robustness.
    Measured only over episodes whose pilot READS the prompt
    (`pilot_reads_text`): a headless pilot ignores text, so its arms are one
    flight by construction and their pairs are plumbing checks. Until such
    episodes are flown both sections are n/m, with the headless count beside.

    SINCE 2026-11-16 the A/B (never the per-paraphrase section) is also
    measured over episodes whose pilot CONSUMES the CSP (`pilot_reads_prefix`,
    a key apart from `pilot_reads_text`: guardrail/prefix_consumer.py reads
    the action sets, not words). Its pairs are per rail, cell and seed - the
    prefix-on arm's first flight against the prefix-off arm - and the A/B
    then also carries `consumer` (prefix_ab_figures: every measure on, off
    and on minus off, and the A/A pairs as the noise floor)."""
    armed = [e for e in episodes if isinstance(e.get("arms"), dict)]
    # SINCE 2026-11-18 also the pairs a learned VLA flew reading the prefix as
    # text (`vla_reads_prefix`, vla_prefix_arms: the PAS follow's --ab-pair),
    # paired by their pair label (vla_prefix_pairs) and kept out of the
    # other groups.
    vla_eps = [e for e in armed if e["arms"].get("vla_reads_prefix") is True]
    armed = [e for e in armed if e["arms"].get("vla_reads_prefix") is not True]
    reads = [e for e in armed if e["arms"].get("pilot_reads_text") is True]
    eats = [e for e in armed if e["arms"].get("pilot_reads_prefix") is True
            and e["arms"].get("pilot_reads_text") is not True]
    blind = [e for e in armed if e["arms"].get("pilot_reads_text") is not True
             and e["arms"].get("pilot_reads_prefix") is not True]

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

    # The consuming pilot's pairs: one rail's flights only (a headless pair
    # and a hil pair of one cell are two pairs), the A/A repeat set apart.
    eaten: dict[tuple, dict] = {}
    for e in eats:
        a = e["arms"]
        side = "aa" if a.get("repeat") else a.get("prefix")
        eaten.setdefault((e.get("rail"), a.get("cell"), a.get("seed")),
                         {}).setdefault(side, []).append(e)
    eat_pairs = [v for _, v in sorted(eaten.items(), key=lambda kv: str(kv[0]))
                 if "on" in v and "off" in v]
    eat_aa = [(v["on"][0], v["aa"][0]) for _, v in sorted(eaten.items(),
                                                          key=lambda kv: str(kv[0]))
              if "on" in v and "aa" in v]

    vla_pairs, vla_unpaired = vla_prefix_pairs(vla_eps)
    # The headline below pools every flown pair: a report never holds two of
    # these pilots' A/Bs at once (mixed_pilots_refusal, before this is
    # reached), so it is one pilot's.
    flown, headless = pairs(reads) + eat_pairs + vla_pairs, pairs(blind)
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
        if eat_pairs:
            # Only when a consuming pilot flew pairs, so an A/B measured by a
            # pilot that reads text alone keeps exactly these keys.
            c_on = [e for p in eat_pairs for e in p["on"]]
            c_off = [e for p in eat_pairs for e in p["off"]]
            ab["consumer"] = {
                "pilot_reads_prefix": True,
                "consumers": sorted({str(e["arms"].get("consumer")) for e in eats}),
                "rails": sorted({str(e.get("rail")) for p in eat_pairs
                                 for e in p["on"] + p["off"]}),
                "pairs": len(eat_pairs),
                "figures": prefix_ab_figures(
                    c_on, c_off, eat_aa,
                    pairs=[((p["on"][0].get("rail"), p["on"][0]["arms"].get("cell")),
                            p["on"][0], p["off"][0]) for p in eat_pairs]),
                "per_cell": [{"cell": p["on"][0]["arms"].get("cell"),
                              "seed": p["on"][0]["arms"].get("seed"),
                              "rail": p["on"][0].get("rail"),
                              "on": _ab_episode(p["on"][0]), "off": _ab_episode(p["off"][0])}
                             for p in eat_pairs],
                "note": PREFIX_CONSUMER_NOTE}
        if vla_pairs:
            # Only when a VLA flew pairs reading the prefix as text
            # (2026-11-18), so every other A/B keeps exactly its keys.
            ab["vla_text"] = vla_prefix_section(vla_pairs, vla_unpaired)
    else:
        ab = {"status": "n/m", "pairs": 0, "headless_pairs": len(headless),
              "why": "no prefix on/off pair has been flown by a pilot that reads "
                     "the prompt; the headless pilots ignore text, so their pairs "
                     "are plumbing checks, not an A/B"}
    if vla_unpaired and "vla_text" not in ab:
        # flights of a VLA prefix A/B with no complete pair (2026-11-18)
        ab["vla_text_unpaired"] = vla_unpaired
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


def mixed_pilots_refusal(episodes: list[dict]) -> str | None:
    """Why these episodes cannot make one report, or None: one report, one
    pilot per rail scenario flight (2026-11-16). A prefix-consumer campaign
    flies ANOTHER PILOT than the stress library's own, and half its flights
    without the Prefix, so its rail flights never pool with a campaign of the
    library's own pilots, whatever globs brought them in (two campaigns of
    one report are also of one profile: campaign_merge). Rail flights of the
    stress library only (`scenario_id`): a report of other runs, or of a
    headless sweep, is the one it always was."""
    eats, plain, vla, reads = [], [], [], []
    for e in episodes:
        arms = e.get("arms") if isinstance(e.get("arms"), dict) else {}
        if arms.get("vla_reads_prefix") is True:
            vla.append(str(e.get("id")))
        elif arms.get("pilot_reads_text") is True:
            reads.append(str(e.get("id")))
        if e.get("kind") != "flight" or e.get("scenario_id") is None:
            continue
        (eats if arms.get("pilot_reads_prefix") is True else plain).append(str(e.get("id")))
    # (2026-11-18) the A/B headline (mission success on / off and its delta)
    # pools every pair it is given: the PAS follow's VLA reading the prefix
    # as text, the hand-written consumer and the stress harness's
    # text-reading pilot are three pilots on different missions, so a VLA
    # prefix A/B never shares a report with either of the other two
    # (review of 94a7d7b: the text-reading pilot's pairs were let in).
    for other, who in ((eats, "the pilot that consumes the CSP"),
                       (reads, "the stress harness's text-reading pilot")):
        if vla and other:
            return (f"these runs hold two prefix A/Bs by two pilots: {len(vla)} flights of "
                    f"a VLA reading the prefix as text on the PAS follow (e.g. {vla[0]}) and "
                    f"{len(other)} of {who} (e.g. {other[0]}); the A/B headline would pool "
                    f"them, so report each on its own")
    if not (eats and plain):
        return None
    return (f"these rail runs were flown by two pilots: {len(eats)} by a pilot that "
            f"consumes the CSP (guardrail/prefix_consumer.py; e.g. {eats[0]}) and "
            f"{len(plain)} by the stress library's own (e.g. {plain[0]}). A prefix-consumer "
            f"campaign is reported on its own, beside the KPI campaign, never in one "
            f"report with its flights")


def _ab_episode(e: dict) -> dict:
    """One episode of a consumer A/B pair, as the per-cell table prints it."""
    return {"id": e.get("id"), "mission_success": K.mission_success_with_goal(e),
            "mission_outcome": e.get("mission_outcome"),
            "p0_escapes": e.get("p0_escapes"), "repair_count": e.get("repair_count"),
            "mean_repair_magnitude_mps": e.get("mean_repair_magnitude_mps"),
            "mean_time_to_safe_s": e.get("mean_time_to_safe_s"),
            "kpi_grade": e.get("kpi_grade")}


# Family figures the baseline comparison reports (rollup() keys).
BASELINE_FIELDS = ("episodes", "p0_escape_rate", "p0_escapes", "mission_success_rate",
                   "mean_repair_count_per_episode", "mean_repair_magnitude_mps",
                   "repair_success_rate", "mean_time_to_safe_s")


def failure_index(episodes: list[dict], include_controls: bool = False) -> dict[str, dict]:
    """Every failing episode (guardrail.kpi.failure_categories non-empty),
    keyed `<family>::<id>`; Shield-off control arms only when asked for, as in
    the top-K table. Written into the report JSON as `failures` (over the
    headline episodes) and `comparison_failures` (over the comparison
    configurations, `split_comparison`), so a later run can compare against
    it (--baseline)."""
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


def split_comparison(episodes: list[dict]) -> tuple[list[dict], list[dict]]:
    """(headline episodes, comparison-configuration episodes), by
    `comparison_label` of each family. The grant's "Top-K failure cases"
    (Stress Testing p6), the `failures` index --baseline compares against,
    the headline facts and the prefix A/B are the headline configuration's;
    a comparison copy shadows a headline episode of the same cell and seed
    (sweep_scenarios._twin_id), so pooling it there would count one scenario
    twice under two vehicles or forecasts. The comparison episodes get their
    own top-K and index (`comparison_top_failures`, `comparison_failures`)
    and keep their labelled rows in the family tables."""
    head, cmp_ = [], []
    for e in episodes:
        (cmp_ if comparison_label(e.get("family")) else head).append(e)
    return head, cmp_


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
    before `failures` existed is read from its `episodes`.

    Headline against headline: `now["failures"]` holds no comparison
    configuration since 2026-10-20 (split_comparison), and a baseline written
    before that holds them in its `failures`. Kept, every comparison copy of
    such a baseline that is still present would read as FIXED."""
    if isinstance(base.get("failures"), dict):
        before = base["failures"]
    else:
        before = failure_index(base.get("episodes") or [], include_controls)
    before = {k: v for k, v in before.items()
              if not comparison_label((v or {}).get("family"))}
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

    def left_out(k: dict) -> str:
        # Every file that is not in the shares, by why (review of 2026-10-19:
        # CSPAltitudeConflict files were missing from this line, so "1 compiled
        # of 2, 0 raised" did not add up). A csp_kpis.json written before then
        # has no altitude count: said so, never printed as 0.
        alt = k.get("n_altitude_conflict")
        files = k.get("altitude_conflict_files") or []
        return (f"{k.get('csp_budget_exceeded')} raised CSPBudgetExceeded; "
                + ("CSPAltitudeConflict not counted (written before 2026-10-19)"
                   if alt is None else
                   f"{alt} raised CSPAltitudeConflict"
                   + (f" ({', '.join(f'`{f}`' for f in files)})" if files else ""))
                + (f"; {k['n_not_flyable']} not flyable as written"
                   if k.get("n_not_flyable") else ""))

    def line(k: dict, label: str) -> str:
        disc = k.get("kpi_discriminates")
        return (f"- {label}: P0 coverage {_f(k.get('p0_coverage'), pct=True)} over "
                f"{k.get('n_compiled')} compiled of {k.get('n_policies')} policies; null "
                f"{_f(k.get('p0_coverage_null'), pct=True)}; naive baseline "
                f"{_f(k.get('p0_coverage_baseline'), pct=True)}; "
                f"{left_out(k)}; largest CSP "
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
    L += ["", "| Budget | Compiled | Raised | Altitude conflict | P0 coverage | "
              "Naive baseline | Null | Discriminates |", "|---|---|---|---|---|---|---|---|"]
    for r in w["scan"]:
        L.append(f"| {r['budget_tokens']} | {r['n_compiled']} | "
                 f"{r['n_raised_budget_exceeded']} | {_f(r.get('n_altitude_conflict'))} | "
                 f"{r['n_p0_covered']}/{r['n_p0_in_scope']} | "
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


_LAT_NAMES = {"tick_ms": "Tick to setpoint", "e2e_ms": "Action receipt to setpoint",
              "e2e_with_wait_ms": "Action receipt to setpoint, with the wait behind "
                                  "post-setpoint work",
              "cam_age_ms": "Camera frame age at setpoint",
              "cam_to_setpoint_ms": "Camera to first setpoint"}


def _lat_cells(s: dict | None) -> str:
    """p50 / p95 / p99 / max and the count over budget, or why not."""
    if not isinstance(s, dict):
        return "n/m"
    if s.get("status"):
        return f"{s['status']}: {s.get('why')}"
    return (f"p50 {_f(s['p50'], 1)} / p95 {_f(s.get('p95'), 1)} / p99 {_f(s['p99'], 1)} / "
            f"max {_f(s['max'], 2)} ms; {s['over_budget']} of {s['n']} ticks over 100 ms")


def _lat_verdict(s: dict | None) -> str:
    if not isinstance(s, dict) or s.get("status"):
        return "n/m"
    if not s["over_budget"]:
        return "within (max <= 100 ms)"
    if (s.get("p99") or 0) <= TICK_BUDGET_MS:
        return f"p99 within; {s['over_budget']} tick(s) over"
    return f"p99 OVER 100 ms ({s['over_budget']} tick(s) over)"


def _tts(s: dict) -> str:
    st = s.get("time_to_safe_status")
    txt = ("never unsafe" if st == "never_unsafe" else
           "none recovered" if st == "never_recovered" else
           f"{_f(s.get('mean_time_to_safe_s'), 2)} (max {_f(s.get('max_time_to_safe_s'), 2)}, "
           f"{s.get('time_to_safe_episodes')} unsafe ep)")
    if s.get("episodes_time_to_safe_not_measurable"):
        txt += f" ({s['episodes_time_to_safe_not_measurable']} ep n/m)"
    if s.get("time_to_safe_censored"):
        txt += f" +{s['time_to_safe_censored']} never recovered"
    return txt


def _tracked_row(name: str, s: dict) -> str:
    ms = ("n/m" if s.get("mission_success_rate") is None else
          f"{s['mission_successes']}/{s['episodes_mission_scored']}")
    reps = s.get("mean_repair_count_per_episode")
    mag = "no repairs" if reps == 0 else _f(s.get("mean_repair_magnitude_mps"), 3)
    rs_ = _rs(s)
    if s.get("repair_success_rate") is not None:
        rs_ += f" ({s.get('repair_success_ticks')}/{s.get('repair_attempt_ticks')} ticks)"
    return (f"| {name} | {s['episodes']} | {ms} | {_tts(s)} | {_f(reps, 1)} | {mag} | "
            f"{rs_} |")


def _render_kpi_summary(rep: dict) -> list[str]:
    """The KPI summary that leads the report (kpi_summary)."""
    s = rep.get("kpi_summary")
    if not s:
        return []
    L = ["## KPI summary", "",
         f"The grant's acceptance KPIs over the {s['kpi_grade_episodes']} KPI-grade "
         f"episodes ({s['shield_on']} Shield on, {s['shield_off']} Shield-off control"
         f"{'' if s['shield_off'] == 1 else 's'}), each beside its null. Only KPI-grade "
         f"episodes (`is_kpi_grade`, the signed replay bundle bound to the run"
         + (", and the campaign's acceptance" if rep.get("campaign") else "")
         + ") give contractual numbers; the rest is tabulated apart below.", ""]
    if not s["kpi_grade_episodes"]:
        return L + ["No KPI-grade episode: no contractual figure.", ""]
    L += ["| KPI (grant) | Target | Measured | Null / control | Verdict |",
          "|---|---|---|---|---|"]
    on, off = s["p0"]["shield_on"], s["p0"]["shield_off_control"]
    if on:
        meas = ("vacuous (no P0 tick)" if on["vacuous"] else
                f"{_f(on['p0_escape_rate'], 4)} ({on['p0_escapes']} escapes on "
                f"{on['p0_violation_ticks']} P0-violation ticks, {on['episodes']} "
                f"Shield-on flights)")
        if on.get("p0_ticks_not_measurable"):
            meas += f"; {on['p0_ticks_not_measurable']} P0 tick(s) not measurable"
        ctrl = (f"Shield-off control {_f(off['p0_escape_rate'], 4)} ({off['p0_escapes']} "
                f"escapes on {off['p0_violation_ticks']} P0-violation ticks, "
                f"{off['episodes']} flight{'' if off['episodes'] == 1 else 's'})"
                if off else "no Shield-off control flown")
        ctrl += f"; passthrough null {_f(on.get('null_passthrough_p0_escape_rate'), 4)}"
        verdict = ("vacuous" if on["vacuous"] else
                   "meets: 0 escapes" if on["p0_escapes"] == 0 and not
                   on.get("p0_ticks_not_measurable") else
                   "not shown: P0 ticks not measurable" if on["p0_escapes"] == 0 else
                   f"FAILS: {on['p0_escapes']} escape(s)")
        L.append(f"| P0 violation escape rate (Stress Testing p6) | 0 | {meas} | {ctrl} | "
                 f"{verdict} |")
    fs = s.get("failsafe")
    if fs:
        name = ("Fail-safe trigger correctness, reading "
                f"({rep['failsafe_readings']['adopted']}) adopted"
                if rep.get("failsafe_readings") else
                "Fail-safe trigger correctness, labels as flown")
        if fs.get("failsafe_trigger_correctness") is None:
            meas, verdict = "n/m (no scored run)", "n/m"
        else:
            meas = (f"{_f(fs['failsafe_trigger_correctness'], 3)} ({fs['correct']}/"
                    f"{fs['scored']}): the point estimate, the KPI; Wilson 95 % lower "
                    f"bound {_f(fs.get('wilson_low_95'), 3)}" + _backstop_note(fs))
            mt, m95 = fs.get("meets_target"), fs.get("meets_target_at_95")
            verdict = ("not judged (the labels do not discriminate)" if mt is None else
                       ("meets" if mt else "BELOW") + " 99 % (point); "
                       + ("meets" if m95 else "not shown") + " at 95 % confidence"
                       + ("" if m95 else " (needs "
                          f"{fs.get('min_error_free_episodes_for_target_at_95')} "
                          f"error-free episodes)"))
        L.append(f"| {name} | >= 99 % | {meas} | always-trigger "
                 f"{_f(fs.get('null_always_trigger'), 3)}, never-trigger "
                 f"{_f(fs.get('null_never_trigger'), 3)} | {verdict} |")
    for rail, r in sorted((s.get("tick_budget") or {}).items()):
        for key, _ in LATENCY_FIELDS:
            null = ("a tick run on the action's arrival is the node's own work; a "
                    "deferred tick or a --tick-trigger timer run adds the wait for its "
                    "poll or timer" if key in ("e2e_ms", "e2e_with_wait_ms") else
                    "frame ARRIVAL at the node, not render: render and transport before "
                    "it are not in it" if key.startswith("cam") else "-")
            got = isinstance(r.get(key), dict) and not r[key].get("status")
            L.append(f"| {_LAT_NAMES[key]} `{key}` ({rail}) | <= 100 ms end to end | "
                     f"{_lat_cells(r.get(key))} | {null if got else '-'} | "
                     f"{_lat_verdict(r.get(key))} |")
    L += ["", "Tracked KPIs (Shield on, KPI-grade):", "",
          "| Scope | Episodes | Mission success | Mean time to safe (s) | Mean repairs / ep | "
          "Mean repair magnitude (m/s) | Repair success |",
          "|---|---|---|---|---|---|---|"]
    if s.get("tracked"):
        L.append(_tracked_row("all Shield-on flights", s["tracked"]))
    for t, row in (s.get("by_template") or {}).items():
        L.append(_tracked_row(f"`{t}`", row))
    tr = s.get("tracked") or {}
    L += ["", f"Nulls: a hover would pass {_f(tr.get('null_hover_mission_success_rate'), pct=True)}"
              f" of the missions; an always-brake Shield's repair success is "
              f"{_f(tr.get('null_always_brake_repair_success_rate'), pct=True)}. "
              f"Per (template, cell) the family tables below have every figure.", ""]
    return L


def _render_shield_matrix(m: dict | None) -> list[str]:
    """The Shield test matrix's non-vacuity (shield_matrix_section)."""
    if not m:
        return []
    if not m.get("applies", True):
        # No matrix scenario episode and none planned: no gate verdict.
        return ["## Shield test matrix: a P0 tick in every row", "",
                "No Shield matrix scenario episode (Shield on) is in this report, and no "
                "campaign of it planned one: the release gate's matrix line "
                "(docs/RUNBOOK-release.md step 5) is not read here.", ""]
    basis = ("the KPI-grade Shield-on episodes" if m["basis"] == "kpi_grade_shield_on"
             else "every Shield-on episode (no KPI-grade episode: functional evidence)")
    L = ["## Shield test matrix: a P0 tick in every row", "",
         f"Safety Shield, \"Test matrix\", read over {basis} ({m['episodes']}). A cell "
         f"is non-vacuous when its raw commands had P0-violating ticks for the Shield "
         f"to act on; a vacuous cell's escape rate of 0 says nothing about the Shield. "
         f"Release gate (docs/RUNBOOK-release.md step 5): {m['gate']}.", "",
         "| Matrix row | Families | Cells flown | Non-vacuous | Vacuous | Not measured | "
         "Verdict |", "|---|---|---|---|---|---|---|"]
    for r in m["rows"]:
        verdict = ("has a non-vacuous cell" if r["has_non_vacuous_cell"] else
                   "NOT FLOWN" if not r["cells_flown"] else "NO non-vacuous cell")
        L.append(f"| {r['row']} | " + ", ".join(f"`{f}`" for f in r["families"])
                 + f" | {r['cells_flown']} | {r['non_vacuous_cells']} | "
                 f"{r['vacuous_cells']} | {r['not_measured_cells']} | {verdict} |")
    a = m["at_speed_near_edge"]
    verdict = ("non-vacuous" if a["non_vacuous"] else
               "NOT FLOWN" if not a["cells_flown"] else "VACUOUS" if not
               a["non_vacuous_cells"] else "PARTLY VACUOUS" if
               a["non_vacuous_cells"] < a["cells_flown"] else "A PLANNED CELL NOT MET")
    L += ["", f"At-speed near-edge (`{a['scenario_id']}`): **{verdict}** - "
              f"{a['non_vacuous_cells']} of {a['cells_flown']} cell(s) non-vacuous"
          + ("" if not a["cells"] else ": " + "; ".join(
              f"`{c['family']}` {c['state']} ({c['p0_violation_ticks']} P0 tick(s))"
              for c in a["cells"])) + "."]
    pl = a.get("planned")
    if pl is not None:
        unmet = [c for c in pl if not c["met"]]
        L += ["", f"Planned by the campaign: {len(pl)} cell(s), {len(pl) - len(unmet)} "
                  f"flown with every planned flight and non-vacuous"
              + ("" if not unmet else "; not met: " + "; ".join(
                  f"`{c['cell']}` {c['state']} ({c['flights']} of {c['planned_flights']} "
                  f"planned flight(s) in the report; campaign: {', '.join(c['campaign'])})"
                  for c in unmet)) + "."]
    L += ["", f"Gate: **{'met' if m['gate_met'] else 'NOT MET'}**.", ""]
    return L


def _backstop_note(fs: dict) -> str:
    """What the fail-safe KPI's row says about the GeoFence backstop
    (2026-11-15; the block at READING_EXCLUDED): how many of the scored
    fail-safes it triggered and the figure with those runs read the two
    earlier ways - or that the count was not recorded, never a 0 for a table
    or score written before the rule."""
    what = "triggered by the autopilot's GeoFence backstop while the FSM held"
    if "backstop" in fs:                         # the adopted reading of a re-score table
        bs = fs["backstop"]
        if bs is None:
            return (f"; fail-safes {what}: not recorded (the re-score table predates the "
                    f"rule of {BACKSTOP_SINCE}; such runs are left out as censored here)")
        if not bs:
            return f"; 0 of the scored fail-safes {what} (counted since {BACKSTOP_SINCE})"
        b0, b1 = fs["before_backstop_rule"], fs["backstop_as_not_triggered"]
        return (f"; {len(bs)} of the scored fail-safes {what} (counted since "
                f"{BACKSTOP_SINCE}). The same runs read as before that rule "
                f"({b0['left_out_as_censored']} left out as censored): "
                f"{_f(b0['failsafe_trigger_correctness'], 3)} ({b0['correct']}/"
                f"{b0['scored']}), Wilson 95 % low {_f(b0['wilson_low_95'], 3)}; counted "
                f"as not triggered: {_f(b1['failsafe_trigger_correctness'], 3)} "
                f"({b1['correct']}/{b1['scored']}), Wilson 95 % low "
                f"{_f(b1['wilson_low_95'], 3)}")
    n = fs.get("triggered_by_backstop")
    if n is None:
        return (f"; fail-safes {what}: not recorded (the flights' scores predate the "
                f"rule of {BACKSTOP_SINCE})")
    return f"; {n} of the scored fail-safes {what} (counted since {BACKSTOP_SINCE})"


def _render_campaign(c: dict | None, episodes: list[dict]) -> list[str]:
    if not c:
        return []
    st = ", ".join(f"{n} {k}" for k, n in sorted(c["statuses"].items()))
    na = [e for e in episodes if e.get("campaign_status") in ("not accepted",
                                                               "not in the campaign")]
    fa = c.get("failsafe_at_flight") or {}
    ch = c.get("episode_choice")
    L = ["## Campaign", "",
         f"- {'Main campaign' if ch else 'Campaign'} `{c['campaign']}`: "
         f"experiments/sweep_scenarios.py --backend "
         f"{c['backend']}, profile {c['profile']} (grant scope {c['grant_scope']!r}, "
         f"cadence {c['cadence']!r}), flown at code revision `{c['code_revision']}` in "
         f"{_f(c['runtime_s'], 0)} s.",
         f"- Summary `{c['path']}`: {c['results']} results ({st}); {c['flyable']} flyable "
         f"on the rail, {c['refused_cells']} cell(s) the rail refused before flight.",
         (f"- **LENGTHENED campaign**: every mission flew {max_s_add(c):g} s past the "
          f"rail's cap (experiments/sweep_scenarios.py --hil-max-s-add), so a cell censored "
          f"in Loiter at the profile's length could show its fail-safe outcome. Not the "
          f"profile's flight: its rows are `.../flown-longer-{max_s_add(c):g}s`, and it is "
          f"reported beside the KPI campaign, never merged into its figure."
          if max_s_add(c) else
          "- Flight length: the profile's own (each mission to the rail's cap: the "
          "scenario's max_s + the Loiter operator timeout + 5 s).")]
    pc = c.get("prefix_consumer")
    if pc:
        # Only a campaign flown by a pilot that consumes the CSP has the key.
        L.append(
            f"- **PREFIX-CONSUMER campaign**: every mission's pilot law was flown through "
            f"guardrail/prefix_consumer.py PrefixConsumer (adapter `{pc.get('adapter')}`), "
            f"with the Prefix Compiler's CSP on the prefix-on arm and without it on the "
            f"prefix-off arm ({pc.get('prefix_off_episodes')} prefix-off episode(s), "
            f"{pc.get('aa_episodes')} A/A repeat(s) in the expansion). Another pilot than "
            f"the stress library's own, and half its flights carry no Prefix: its rows are "
            f"`.../prefix-consumer-on` and `.../prefix-consumer-off`, the headline figures "
            f"below pool both arms and are NOT the KPI campaign's, and the measurement "
            f"this campaign exists for is the section \"A/B: prefix on / off\".")
    for s in c.get("supplements") or []:
        sst = ", ".join(f"{n} {k}" for k, n in sorted(s["statuses"].items()))
        L.append(f"- Supplement `{s['campaign']}`: profile {s['profile']}, code revision "
                 f"`{s['code_revision']}`, {_f(s['runtime_s'], 0)} s; summary `{s['path']}`: "
                 f"{s['results']} results ({sst}), {len(s['accepted'])} flight(s) it "
                 f"accepted, {len(s['not_accepted'])} it did not.")
    L += [f"- Runs read from: " + ", ".join(f"`{g}`" for g in c["runs_glob"]) + ".",
          (f"- Flights counted: {len(c['accepted'])} ("
           + ", ".join(f"{n} from `{name}`" for name, n in ch["counted_by_campaign"].items())
           + f"; the main campaign accepted {c.get('main_accepted')}). Run folders not "
             f"accepted or not listed: {len(na)} - NOT KPI-grade here, whatever their "
             f"manifest says:" if ch else
           f"- Flights the campaign accepted: {len(c['accepted'])}. Run folders it did not "
           f"accept or does not list: {len(na)} - NOT KPI-grade here, whatever their "
           f"manifest says:")]
    for e in na:
        L.append(f"  - `{e['id']}`: {(e.get('kpi_grade_reasons') or ['?'])[-1]}")
    if fa.get("labelled") is not None:
        L.append(f"- The {'main ' if ch else ''}campaign's own fail-safe figure, on the "
                 f"labels it flew with: "
                 f"{fa.get('correct')}/{fa.get('labelled')} = "
                 f"{_f(fa.get('fail_safe_correctness'), 3)} ({fa.get('false_triggers')} false, "
                 f"{fa.get('missed_triggers')} missed); "
                 + (f"{(fa['triggered_by'] or {}).get(FAILSAFE_BY_GEOFENCE, 0)} of its "
                    f"fail-safes triggered by the autopilot's GeoFence backstop."
                    if isinstance(fa.get("triggered_by"), dict) else
                    f"scored before the GeoFence backstop rule of {BACKSTOP_SINCE}: a run "
                    f"the backstop took while the FSM held is left out of it as censored."))
    L += ["", "```", str(c.get("command")), "```", ""]
    for s in c.get("supplements") or []:
        L += [f"Supplement `{s['campaign']}`:", "", "```", str(s.get("command")), "```", ""]
    if ch:
        L += ["### Episodes by campaign", "",
              "Which campaign each counted episode comes from (the rules are under "
              "Inputs). Every counted flight not listed here is the main campaign's "
              f"(`{c['campaign']}`).", ""]
        if ch["from_supplements"]:
            L += ["| Episode (cell) | Seed | Shield | Counted run | From | The main campaign "
                  "had |", "|---|---|---|---|---|---|"]
            for x in ch["from_supplements"]:
                had = str(x["main"]).replace("|", "/")
                if x.get("main_run"):
                    had = f"`{x['main_run']}`: {had}"
                L.append(f"| `{x['cell']}` | {x['seed']} | {x['arm']} | `{x['run']}` | "
                         f"`{x['campaign']}` | {had} |")
            L.append("")
        else:
            L += ["No episode was taken from a supplement.", ""]
        if ch["superseded"]:
            L += [f"Superseded, not counted ({len(ch['superseded'])}): a supplement flight "
                  f"of an episode an earlier campaign accepted - not read, not scored.", ""]
            for x in ch["superseded"]:
                L.append(f"- `{x['run']}` (`{x['campaign']}`, {x['cell']} seed {x['seed']}, "
                         f"Shield {x['arm']}, {x['status']}): superseded by `{x['by']}` "
                         f"(`{x['by_campaign']}`)")
            L.append("")
    return L


def _render_failsafe_readings(fr: dict | None) -> list[str]:
    if not fr:
        return []
    L = ["## Fail-safe trigger correctness by label reading", "",
         f"Stress Testing p6: \"Fail-safe trigger correctness >= 99 %\" - triggered when "
         f"expected, not when not expected. Over the {fr['kpi_grade_shield_on_flights']} "
         f"KPI-grade Shield-on flights, under each reading of the label artefact "
         f"`{fr['artefact']}` (labels derived at "
         + ", ".join(f"`{str(r)[:12]}`" for r in fr.get("label_revisions") or ["?"])
         + f"), re-scored in `{fr['source']}`. The adopted reading is the KPI; the others "
         f"are printed beside it, never pooled. The point estimate is the KPI (the "
         f"grant's wording); the Wilson 95 % lower bound says what the sample shows at "
         f"95 % confidence (guardrail/fsm.py `score_failsafe_triggers`). A run a reading "
         f"leaves out (not accepted, no verdict, censored in Loiter, label undecided or "
         f"absent) is named, never counted as correct.", ""]
    rule = fr.get("backstop_rule")
    if rule:
        said = str(rule.get("rule") or "?")
        L += [f"**The GeoFence backstop (since {rule.get('since')}).** {said[0].upper()}"
              f"{said[1:]}: correct where the label expects a fail-safe, a "
              f"false trigger where it does not. Decided {rule.get('decided')} "
              f"({rule.get('doc')}). Before that, {rule.get('before')}. Each reading below "
              f"counts such runs (\"By the GeoFence backstop\") and gives the figure with "
              f"them read as before the rule and with them counted as not triggered; they "
              f"are named under the table.", ""]
    else:
        L += [f"**The GeoFence backstop: not recorded.** The re-score table was written "
              f"before the rule of {BACKSTOP_SINCE} (a run the autopilot's GeoFence took to "
              f"RTL / LAND while the FSM held counts as a fail-safe triggered by the "
              f"backstop): here such a run is left out as censored where the FSM held in "
              f"Loiter. Re-score the campaign (experiments/sweep_scenarios.py "
              f"--rescore-rail) to count them.", ""]
    L += ["| Reading | Correct / scored | Correctness (point, the KPI) | Wilson 95 % low | "
          ">= 99 % (point) | >= 99 % at 95 % | Nulls: always / never | Left out | "
          "Label read on the longer flight | By the GeoFence backstop | Those runs as "
          f"before {BACKSTOP_SINCE} (left out as censored) | Those runs counted as not "
          "triggered | Re-score table's own |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    yn = {True: "yes", False: "no", None: "not judged"}

    def fig(b):
        return (f"{b['correct']}/{b['scored']} = {_f(b['failsafe_trigger_correctness'], 3)} "
                f"(Wilson 95 % low {_f(b['wilson_low_95'], 3)})")
    for key, r in fr["readings"].items():
        out = sum(len(r[k]) for k in (*READING_EXCLUDED, "not_in_rescore",
                                      "labels_not_applied"))
        s0 = r["at_scenario_length"]
        lon = (f"{len(r['label_flown_longer'])} (without them {s0['correct']}/{s0['scored']})"
               if r["label_flown_longer"] else "0")
        bs = r.get("backstop")
        if bs is None:
            bcols = "not recorded | not recorded | not recorded"
        else:
            bcols = (f"{len(bs)} ({sum(1 for t in bs if t['verdict'] == 'correct')} correct, "
                     f"{sum(1 for t in bs if t['verdict'] == 'false')} false) | "
                     f"{fig(r['before_backstop_rule'])}; "
                     f"{r['before_backstop_rule']['left_out_as_censored']} left out | "
                     f"{fig(r['backstop_as_not_triggered'])}")
        tab = r["table"]
        own = (f"{tab.get('correct')}/{tab.get('scored')} over {tab.get('runs')} runs"
               + (f" ({tab['basis']})" if tab.get("basis") else "")
               + ("" if r["agrees_with_table"] else " (differs: it scores another set "
                  "of runs - runs not KPI-grade here, or runs this report leaves out)"))
        mark = " **adopted**" if r["adopted_kpi_reading"] else ""
        L.append(f"| ({key}){mark} | {r['correct']}/{r['scored']} | "
                 f"{_f(r['failsafe_trigger_correctness'], 3)} | {_f(r['wilson_low_95'], 3)} | "
                 f"{yn[r['meets_target']]} | {yn[r['meets_target_at_95']]} | "
                 f"{_f(r['null_always_trigger'], 3)} / {_f(r['null_never_trigger'], 3)} | "
                 f"{out} | {lon} | {bcols} | {own} |")
    L.append("")
    for key, r in fr["readings"].items():
        L.append(f"- ({key}) {r.get('reading')}:")
        for t in r.get("backstop") or []:
            L.append(f"  - triggered by the GeoFence backstop: `{t['run']}` ({t.get('mode')} "
                     f"at {t.get('t')} s, the FSM in {t.get('fsm_state')}; "
                     + ("a fail-safe expected: correct" if t.get("expected") else
                        "no fail-safe expected: a false trigger") + ")")
        for kind, word in (("false", "false trigger"), ("missed", "missed trigger"),
                           *((k, f"left out ({k.replace('_', ' ')})")
                             for k in (*READING_EXCLUDED, "not_in_rescore")),
                           ("label_flown_longer", "label read on the cell flown as long "
                                                  "as the rail flies")):
            for t in r[kind]:
                lab = f" (label {', '.join(t['label'])})" if t.get("label") else ""
                L.append(f"  - {word}: `{t['run']}`{lab}")
        if r.get("labels_not_applied"):
            L.append(f"  - left out, labels not applied ({LABELS_NOT_APPLIED_WHY}): "
                     f"{len(r['labels_not_applied'])} run(s), "
                     + ", ".join(f"`{t['run']}`" for t in r["labels_not_applied"]))
    if fr.get("rescore_runs_not_in_report"):
        L.append("- Runs in the re-score table that this report did not read: "
                 + ", ".join(f"`{n}`" for n in fr["rescore_runs_not_in_report"]))
    if fr.get("rescore_runs_superseded"):
        L.append("- Runs in the re-score tables that the campaigns' episode choice "
                 "superseded (not counted, not in the tables' own figure above): "
                 + ", ".join(f"`{n}`" for n in fr["rescore_runs_superseded"]))
    for t in fr.get("tables") or []:
        figs = "; ".join(f"({k}) {v.get('correct')}/{v.get('scored')} over {v.get('runs')} "
                         f"runs" for k, v in t["readings"].items())
        L.append(f"- Re-score table `{t['path']}` on its own ({t['runs']} runs): {figs}.")
    return L + [""]


def _render_latency(lat: dict | None) -> list[str]:
    if not lat:
        return []
    L = ["## End-to-end tick budget, per tick", "",
         "Safety Shield p6: \"Monitor tick budget <= 100 ms (10 Hz) end-to-end\". Every "
         "flown row's own timing over the whole flight, pooled per rail (nearest rank), "
         "KPI-grade episodes apart from the rest. A measure no row and no metrics.json "
         "carries is \"not measured\", never 0.", ""]
    for key, what in LATENCY_FIELDS:
        L.append(f"- `{key}`: {what}.")
    L += ["", "Clocks: each measure is the difference of two readings of ONE clock - the "
              "hil Shield node's `time.monotonic_ns()`, the perception node's loop clock "
              "(the frame's arrival stamp, the tick and the publication all read it). On "
              "the perception rail the camera frame's time is its ARRIVAL at the node (the "
              "pose ring there carries no simulator stamps), so render and transport "
              "before it are not in `cam_*`: a lower bound of camera to action.", ""]
    for grp, title in (("kpi_grade", "KPI-grade episodes"),
                       ("not_kpi_grade", "NOT KPI-grade episodes (functional evidence)")):
        rails = lat["groups"].get(grp) or {}
        if not rails:
            continue
        L += [f"{title}:", "",
              "| Rail | Measure | Episodes | Ticks | p50 (ms) | p95 (ms) | p99 (ms) | "
              "Max (ms) | Over 100 ms |", "|---|---|---|---|---|---|---|---|---|"]
        for rail, r in rails.items():
            for key, _ in LATENCY_FIELDS:
                s = r.get(key) or {}
                if s.get("status"):
                    L.append(f"| {rail} | `{key}` | - | - | {s['status']}: {s['why']} "
                             f"| | | | |")
                    continue
                L.append(f"| {rail} | `{key}` | {s['episodes']} of {r['episodes']} | "
                         f"{s['n']} | {_f(s['p50'], 2)} | {_f(s['p95'], 2)} | "
                         f"{_f(s['p99'], 2)} | {_f(s['max'], 2)} | {s['over_budget']} |")
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
        L += _render_consumer_ab(ab.get("consumer"))
        L += _render_vla_text_ab(ab.get("vla_text"))
    for u in (ab or {}).get("vla_text_unpaired") or []:
        L.append(f"- Unpaired VLA prefix-A/B flight `{u['id']}` ({u['prefix']}): {u['why']}.")
    if (ab or {}).get("vla_text_unpaired"):
        L.append("")
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


def _render_vla_text_ab(v: dict | None) -> list[str]:
    """The A/B a VLA flew reading the prefix as text (vla_prefix_section):
    the models and slots, the KPI-grade count, the follow's own measures
    with their A/A floor, whether mission success separates the arms, the
    measures on / off, the neutral arm where flown, one row per pair, the
    unpaired flights. [] when no such pair was flown."""
    if not v:
        return []
    fig = v["figures"]
    kg = fig.get("kpi_grade_episodes") or {}
    n_on, n_off = fig.get("pairs_on"), fig.get("pairs_off")
    graded = (kg.get("on") == n_on and kg.get("off") == n_off)
    L = [f"### Flown by a VLA that reads the prefix ({', '.join(v['models'])})", "",
         f"- {v['pairs']} pair(s) (one --ab-pair label, seed, object words, model, policy "
         f"bundle and scene each), prefix on against prefix off, on "
         f"{', '.join(v['rails'])}; prompt slot "
         f"{', '.join(v['slots'])}. KPI-grade flights: {kg.get('on')} of {n_on} with the "
         f"prefix on, {kg.get('off')} of {n_off} off"
         + ("." if graded else " - FUNCTIONAL EVIDENCE, NOT KPI-GRADE, wherever a flight "
                               "is not."),
         f"- {v['note'][0].upper()}{v['note'][1:]}."]
    for key, label in VLA_FOLLOW_MEASURES:
        fw = v.get(key) or {}
        floor = (v.get("aa_follow") or {}).get(key)
        L.append(f"- The follow's share of ticks {label}: on {_f(fw.get('on'))}, off "
                 f"{_f(fw.get('off'))}, on minus off {_f(fw.get('delta_on_minus_off'))}"
                 + ("" if not floor else
                    f"; A/A first {_f(floor['first'])}, repeat {_f(floor['repeat'])}, "
                    f"largest gap in one A/A pair {_f(floor['max_pair_abs_diff'])}")
                 + ".")
    ms = v.get("mission_success") or {}
    if ms.get("separates") is False:
        L.append(f"- MISSION SUCCESS DOES NOT SEPARATE THE ARMS: {ms['why']}.")
    if v.get("repeats_not_counted"):
        L.append(f"- {v['repeats_not_counted']} repeat flight(s) of an arm already flown in "
                 f"its pair are not counted (the first by run id is; a second prefix-on "
                 f"flight is the pair's A/A repeat).")
    L += [""] + prefix_ab_lines(fig) + [""]
    nt = v.get("neutral")
    if nt:
        L += [f"#### The neutral arm (rule-free text of the CSP's length), {nt['pairs']} "
              f"pair(s)", "", "Neutral against off:", ""]
        L += prefix_ab_lines(nt["neutral_vs_off"])[:len(PREFIX_AB_MEASURES) + 2]
        L += ["", "CSP against neutral (the rules beyond any text of their length):", ""]
        L += prefix_ab_lines(nt["csp_vs_neutral"])[:len(PREFIX_AB_MEASURES) + 2] + [""]
    L += ["| Pair | Seed | Rail | Repairs on / off | Repair magnitude on / off (m/s) | "
          "P0 escapes on / off | Within 30 m from acquisition on / off | "
          "Within 30 m from take-off on / off | Mission success on / off | "
          "Outcome on / off | A/A flown | Prefix SHA-256 |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for p in v.get("per_pair") or []:
        a, b = p["on"], p["off"]
        L.append(f"| `{p['pair']}` | {p['seed']} | {p['rail']} | "
                 f"{_f(a['repair_count'])} / {_f(b['repair_count'])} | "
                 f"{_f(a['mean_repair_magnitude_mps'])} / "
                 f"{_f(b['mean_repair_magnitude_mps'])} | "
                 f"{_f(a['p0_escapes'])} / {_f(b['p0_escapes'])} | "
                 f"{_f(a.get('follow_within_30m_after_acquire'))} / "
                 f"{_f(b.get('follow_within_30m_after_acquire'))} | "
                 f"{_f(a['follow_within_30m'])} / {_f(b['follow_within_30m'])} | "
                 f"{_f(a['mission_success'])} / {_f(b['mission_success'])} | "
                 f"{a['mission_outcome']} / {b['mission_outcome']} | "
                 f"{'yes' if p.get('aa_flown') else 'no'} | "
                 f"`{str(p['prefix_sha256'])[:12]}` |")
    for u in v.get("unpaired") or []:
        L.append(f"- Unpaired: `{u['id']}` ({u['prefix']}): {u['why']}.")
    return L + [""]


def _render_consumer_ab(c: dict | None) -> list[str]:
    """The A/B flown by a pilot that consumes the CSP (arms_sections'
    `consumer`): who the pilot is, how many of its flights are KPI-grade,
    the measures on / off / on minus off with the A/A noise floor, and one
    row per pair. [] when no such pair was flown."""
    if not c:
        return []
    fig = c["figures"]
    kg = fig.get("kpi_grade_episodes") or {}
    n_on, n_off = fig.get("pairs_on"), fig.get("pairs_off")
    graded = (kg.get("on") == n_on and kg.get("off") == n_off)
    d = fig.get("distinct") or {}
    L = [f"### Flown by a pilot that consumes the CSP ({', '.join(c['consumers'])})", "",
         f"- {c['pairs']} pair(s) of one cell and seed"
         + (f" ({d['pairs']} distinct flight pair(s))" if d else "")
         + f", prefix on against prefix off, on "
         f"{', '.join(c['rails'])}. KPI-grade flights: {kg.get('on')} of {n_on} with the "
         f"prefix on, {kg.get('off')} of {n_off} off"
         + ("." if graded else " - FUNCTIONAL EVIDENCE, NOT KPI-GRADE, wherever a flight "
                               "is not."),
         f"- {c['note'][0].upper()}{c['note'][1:]}.",
         "- A cell's outcome and fail-safe labels were derived for its own rule-ignorant "
         "pilot and are not applied to the prefix-on arm; its gates (the safety "
         "invariants) are.", ""]
    L += prefix_ab_lines(fig) + [""]
    L += ["| Cell | Seed | Rail | Repairs on / off | Repair magnitude on / off (m/s) | "
          "Time to safe on / off (s) | P0 escapes on / off | Mission success on / off | "
          "Outcome on / off |", "|---|---|---|---|---|---|---|---|---|"]
    for p in c.get("per_cell") or []:
        a, b = p["on"], p["off"]
        L.append(f"| `{p['cell']}` | {p['seed']} | {p['rail']} | "
                 f"{_f(a['repair_count'])} / {_f(b['repair_count'])} | "
                 f"{_f(a['mean_repair_magnitude_mps'])} / "
                 f"{_f(b['mean_repair_magnitude_mps'])} | "
                 f"{_f(a['mean_time_to_safe_s'])} / {_f(b['mean_time_to_safe_s'])} | "
                 f"{_f(a['p0_escapes'])} / {_f(b['p0_escapes'])} | "
                 f"{_f(a['mission_success'])} / {_f(b['mission_success'])} | "
                 f"{a['mission_outcome']} / {b['mission_outcome']} |")
    return L + [""]


def _render_magnitudes(groups: dict[str, dict]) -> list[str]:
    classes = list(K.OPERATOR_CLASSES)
    # An operator kpi.OPERATOR_CLASSES does not list is counted Unclassified;
    # its column is printed whenever any family has such a tick, so a repair
    # by an operator nobody classed is never hidden.
    if any(((s.get("mean_repair_magnitude_by_operator_class") or {})
            .get(K.UNCLASSIFIED) or {}).get("ticks")
           for g in groups.values() for s in (g or {}).values()):
        classes.append(K.UNCLASSIFIED)
    # A tick the Shield floored with nothing violated (kpi._floored, since
    # 2026-10-19) has no priority to sit under; it has its own level, and
    # its column is printed whenever any family has one, like Unclassified.
    levels = [*K.PRIORITY_LEVELS, "unattributed"]
    if any(((s.get("mean_repair_magnitude_by_priority") or {}).get("floored") or {})
           .get("ticks") for g in groups.values() for s in (g or {}).values()):
        levels.append("floored")
    L = ["## Mean repair magnitude by priority and by operator class", "",
         "|emitted - raw| (m/s) on repaired ticks, pooled. A tick counts under the "
         "highest priority it violated, and under every operator class that acted on "
         "it (the class's ticks in brackets). `never` = no repaired tick of that "
         "priority or class: it never acted, which is not a 0. The grant's classes "
         "(Safety Shield p3) are AltitudeClamp, LateralProjection and PathRepair; the "
         "others (*) are this Shield's extensions"
         + (f", and {K.UNCLASSIFIED} an operator no class lists" if K.UNCLASSIFIED in classes
            else "") + "."
         + (" `floored` = ticks the Shield floored a legal action from an illegal "
            "position with nothing violated (not repair attempts)."
            if "floored" in levels else ""), "",
         "| Family | " + " | ".join(levels) + " | "
         + " | ".join(c if c in (*K.GRANT_OPERATOR_CLASSES, K.UNCLASSIFIED) else c + "*"
                      for c in classes)
         + " |", "|---|" + "---|" * (len(levels) + len(classes))]

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
                for k in levels)
                + " | " + " | ".join(cell(cl.get(c)) if cl is not None else "n/m"
                                     for c in classes) + " |")
    if not n:
        L.append("| (no family carries the breakdown: stored tables and Shield-off arms "
                 "do not) |" + " |" * (len(levels) + len(classes)))
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


def _top_failure_table(rows: list[dict]) -> list[str]:
    """A top-K table (guardrail.kpi.top_failures rows) with its time-unsafe
    basis notes, or "(no failing episode)"; the headline's and the comparison
    configurations' tables are the same table over different episodes."""
    if not rows:
        return ["(no failing episode)", ""]
    L = ["| # | Scenario | Family | Parameters | KPI-grade | Failure category | "
         "Max time unsafe (s) | Raw action (vx, vy, vz_up, yaw_rate) | "
         "Repaired action | t (s) | Bundle |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    notes: dict[str, list[str]] = {}       # basis -> the rows citing it
    for i, f in enumerate(rows, 1):
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
    return L


def _render_rail_autopilot(episodes: list[dict]) -> list[str]:
    """Rail runs of the stress library whose AUTOPILOT made a move into an
    unsafe position (2026-11-07, the user's ruling): its keep-out breaches
    and polygon entries, gated by the harness like the Shield's, and its
    departures from a keep-in corridor or an altitude band while a fail-safe
    mode flew, reported and not gated (ArduPilot's RTL, LAND and hold do not
    follow mission bounds)."""
    rows = []
    for e in episodes:
        if e.get("scenario_status") is None:
            continue
        dep = e.get("autopilot_departures") or {}
        vals = (e.get("autopilot_keepout_breaches"), e.get("autopilot_polygon_entries"),
                dep.get("corridor"), dep.get("altitude_band"))
        if any(v for v in vals):
            rows.append(f"| `{e['id']}` | " + " | ".join(
                "n/m" if v is None else str(v) for v in vals) + " |")
    if not rows:
        return []
    return (["## What the autopilot flew on the rail runs", "",
             "Moves the autopilot made into an unsafe position (after the FSM handed "
             "the aircraft to RTL, LAND or the hold). Keep-out breaches and polygon "
             "entries are gated like the Shield's; corridor and altitude-band "
             "departures are reported, not gated: the fail-safe modes are the grant's "
             "escape and ArduPilot's RTL / LAND do not follow mission bounds.", "",
             "| Run | Keep-out breaches (gated) | Polygon entries (gated) | "
             "Corridor departures | Altitude-band departures |",
             "|---|---|---|---|---|"] + rows + [""])


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

    # 2026-11-11: the grant's KPIs first, then the campaign they come from
    # and the fail-safe readings behind the adopted one.
    L += _render_kpi_summary(rep)
    L += _render_shield_matrix(rep.get("shield_matrix"))
    L += _render_campaign(rep.get("campaign"), rep.get("episodes") or [])
    if rep.get("campaign"):
        L += _render_stack(rep.get("stack_revisions"), campaigns=True)
    L += _render_failsafe_readings(rep.get("failsafe_readings"))

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
          f"{c['kpi_grade_at_flight_only']}. Tabulated in their own section."
          + (f" Today's rule includes the campaign's acceptance: "
             f"{c['campaign_not_accepted']} run(s) it did not accept are refused "
             f"(listed under Campaign)." if c.get("campaign_not_accepted") else ""),
          f"- Everything else is functional evidence. The three groups are "
          f"tabulated separately and never pooled.", ""]
    camp = rep.get("campaign") or {}
    ch = camp.get("episode_choice")
    if ch:
        sup = camp.get("supplements") or []
        L += [f"Campaigns: main `{camp['campaign']}` (`{camp['path']}`), "
              + ", ".join(f"supplement `{s['campaign']}` (`{s['path']}`)" for s in sup)
              + f"; all at code revision `{camp['code_revision']}`, profile "
                f"{camp['profile']}. One episode is counted once, by these rules:", ""]
        L += [f"{i}. {r}." for i, r in enumerate(ch["rules"], 1)]
        L += ["", f"Flights the episode choice takes: "
              + ", ".join(f"{n} from `{name}`" for name, n in
                          ch["counted_by_campaign"].items())
              + f"; {len(ch['from_supplements'])} episode(s) taken from a supplement and "
                f"{len(ch['superseded'])} supplement flight(s) superseded (listed under "
                f"Campaign).", ""]
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
        L += _render_stack(rep.get("stack_revisions"), campaigns=False)

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
    L += _render_rail_autopilot(rep.get("episodes") or [])

    L += [f"## Top-{rep['top_k']} failure cases", "",
          "Ranked (`guardrail.kpi.top_failures`): P0 escapes, then unsafe "
          "positions never recovered, longest time unsafe, failed missions "
          "(missed goal, dwell in a zone, an unsafe position entered), repairs "
          "that fell through, and last P0 ticks the log could not measure. "
          + ("Control arms (Shield off on purpose) are excluded; "
             "`--include-controls` lists them." if not rep["include_controls"]
             else "Control arms are included.")
          + (f" Episodes flown on a comparison configuration "
             f"({rep['counts']['comparison']}: `/forecast-3s`, `/wind-open_loop`, "
             f"`/response-ardupilot`) are not ranked here; they have their own "
             f"table below." if (rep.get("counts") or {}).get("comparison") else ""), ""]
    L += _top_failure_table(rep["top_failures"])
    if (rep.get("counts") or {}).get("comparison"):
        # Only when a comparison configuration was read, so a headline-only
        # report is exactly what it was.
        L += [f"## Top-{rep['top_k']} failure cases on comparison configurations "
              f"(not headline numbers)", "",
              "The same ranking over the episodes flown on a comparison "
              "configuration: the 3 s forecast (`/forecast-3s`), the open-loop "
              "wind model (`/wind-open_loop`) or the measured ArduPilot vehicle "
              "response (`/response-ardupilot`). Each shadows a headline episode "
              "of the same cell and seed, so it is kept out of the table above "
              "and out of `failures` (`comparison_failures` in the JSON).", ""]
        L += _top_failure_table(rep.get("comparison_top_failures") or [])

    L += _render_wp1(rep.get("wp1"))
    L += _render_wp2(rep.get("wp2"))
    L += _render_tick_budget(rep.get("tick_budget"))
    L += _render_latency(rep.get("latency"))
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


def _render_response_comparison(rc: dict | None) -> list[str]:
    """The vehicle-response comparison of a sweep run (experiments/
    sweep_scenarios.py response_comparison), or nothing when the arm did not
    fly - so a headline-only run's page is exactly what it was. Per family
    (shield-on), the ideal vehicle beside the measured ArduPilot response over
    the same (cell, seed) pairs: unsafe-position entries, true-state polygon
    entries (Shield-flown and autopilot-flown), the Shield's P0 escapes, the
    true-state P0 ticks twice - of the COMMANDED action and of the velocity
    the airframe flew (`true_p0_vehicle_ticks`, since 2026-10-20: braking
    carry counts only there; the ideal vehicle's airframe flies its command,
    so its two are one count) - and the labelled fail-safe score."""
    if not rc:
        return []
    ti = (rc.get("totals") or {}).get("ideal") or {}
    ta = (rc.get("totals") or {}).get("ardupilot") or {}
    fi, fa = ti.get("failsafe_labels") or {}, ta.get("failsafe_labels") or {}
    c = rc.get("counts") or {}
    params = (rc.get("response") or {}).get("params") or {}
    L = ["## Vehicle-response comparison (not a headline number)", "",
         f"- The same episodes flown on the measured ArduPilot vehicle response "
         f"({', '.join(f'{k} {v}' for k, v in params.items())}; "
         f"docs/DESIGN-harness-vehicle-response.md) instead of the ideal vehicle. "
         f"Reported, never gated: {rc.get('paired_episodes')} paired episodes; "
         f"{c.get('pass', 0)} pass, {c.get('tracked', 0)} tracked, "
         f"{c.get('fail', 0) + c.get('error', 0)} fail, "
         f"{c.get('known_failure', 0)} known, {c.get('unexpected_pass', 0)} "
         f"pinned-but-passing; {rc.get('episodes_moved')} episode(s) moved; "
         f"largest velocity lag {_f(rc.get('max_velocity_lag_mps'), 2)} m/s.",
         f"- Shield-on totals, ideal -> ArduPilot: unsafe-position entries "
         f"{ti.get('unsafe_position_entries')} -> {ta.get('unsafe_position_entries')}; "
         f"polygon entries {ti.get('polygon_entries')} -> {ta.get('polygon_entries')}; "
         f"autopilot polygon entries {ti.get('autopilot_polygon_entries')} -> "
         f"{ta.get('autopilot_polygon_entries')}; P0 escapes {ti.get('p0_escapes')} -> "
         f"{ta.get('p0_escapes')}; true-state P0 ticks of the commanded action "
         f"{ti.get('true_p0_flown_ticks')} -> {ta.get('true_p0_flown_ticks')}, of the "
         f"velocity the airframe flew {ti.get('true_p0_vehicle_ticks')} -> "
         f"{ta.get('true_p0_vehicle_ticks')}; fail-safe labels "
         f"{fi.get('correct')}/{fi.get('labelled')} -> {fa.get('correct')}/"
         f"{fa.get('labelled')} ({fi.get('false_triggers')} -> "
         f"{fa.get('false_triggers')} false, {fi.get('missed_triggers')} -> "
         f"{fa.get('missed_triggers')} missed).", "",
         "| Family | Episodes | Unsafe-position entries | Polygon entries | "
         "Autopilot polygon entries | P0 escapes | True-state P0 ticks (command) | "
         "True-state P0 ticks (airframe) | Fail-safe correct | "
         "Deepest true distance (m) |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for fam, row in (rc.get("families") or {}).items():
        a, b = row.get("ideal") or {}, row.get("ardupilot") or {}
        if not a:
            continue

        def pair(k):
            return f"{a.get(k)} -> {b.get(k)}"
        L.append(f"| `{fam}` | {a.get('episodes')} | {pair('unsafe_position_entries')} | "
                 f"{pair('polygon_entries')} | {pair('autopilot_polygon_entries')} | "
                 f"{pair('p0_escapes')} | {pair('true_p0_flown_ticks')} | "
                 f"{pair('true_p0_vehicle_ticks')} | "
                 f"{a.get('failsafe_correct')}/{a.get('failsafe_labelled')} -> "
                 f"{b.get('failsafe_correct')}/{b.get('failsafe_labelled')} | "
                 f"{_f(a.get('min_true_dist_to_fence_m'), 2)} -> "
                 f"{_f(b.get('min_true_dist_to_fence_m'), 2)} |")
    return L + [""]


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
    # A run whose rollup or top-K failed says so first (review of 2026-10-19,
    # harness-tools/rollup-failure-swallowed): until then `_rollup_note` was
    # never rendered, and a missing `_top_failures` read "(no failing
    # episode)" - a crash shown as a clean result.
    note = doc.get("_rollup_note")
    L = [f"# Stress-harness KPI report - {date.today().isoformat()}", ""]
    if note:
        L += [f"> **INCOMPLETE.** {note}. The tables below are missing what "
              f"could not be computed.", ""]
    L += [
         f"Generated by `experiments/sweep_scenarios.py` at code revision "
         f"`{man.get('code_revision')}`. Reproduce with:", "",
         "```", str(doc.get("_command") or "python experiments/sweep_scenarios.py"),
         "```", ""]
    not_bearing = scope.get("grant_profile_kpi_bearing") is False
    if doc.get("_not_kpi_grade") and not_bearing and scope.get("profile") == "broad":
        # The broad sweep (2026-11-17): its grant row is outside the KPIs on
        # any rail, so "reproduces their scope, not their rail" would mislead.
        L += [f"**Not KPI-grade:** {doc['_not_kpi_grade']}. This profile's row in the "
              f"grant's schedule is not KPI-bearing on any rail: the broad sweep is "
              f"tracked for coverage, and its results are not part of the final KPI "
              f"numbers.", ""]
    elif doc.get("_not_kpi_grade") and not_bearing:
        # Another profile with `grant_kpi_bearing: false` (prefix_ab,
        # fsm_edges): not a smoke or nightly reproduction, and not the broad
        # sweep either; its own scope line says what it is.
        gs = scope.get("grant_scope")
        L += [f"**Not KPI-grade:** {doc['_not_kpi_grade']}. This profile is not one of "
              f"the grant's KPI-bearing rows"
              + (f" (its scope: {gs})" if gs else "")
              + "; keep its runs out of the KPI figures.", ""]
    elif doc.get("_not_kpi_grade"):
        L += [f"**Not KPI-grade:** {doc['_not_kpi_grade']}. The grant's smoke and "
              f"nightly rows ask for sim_speedup 1.0 in the hil topology; this run "
              f"reproduces their scope, not their rail.", ""]
    if doc.get("_resume"):
        r = doc["_resume"]
        L += [f"Merged from per-episode checkpoints (`{r.get('checkpoints')}`): "
              f"{r.get('episodes_replayed')} replayed, "
              f"{r.get('episodes_flown_by_this_process')} flown by the merging run; "
              f"{r.get('episode_wall_s_total')} s of episode time "
              f"({r.get('s_per_episode')} s per episode)"
              + (f"; the sweep's wall span {r.get('sweep_wall_span_s')} s, from "
                 f"{r.get('sweep_started_utc')} to {r.get('last_checkpoint_utc')} "
                 f"(pauses between runs included)"
                 if r.get("sweep_wall_span_s") is not None else "")
              + (f". The invocation that wrote this file: `{r.get('command_as_run')}`"
                 if r.get("command_as_run") else "") + ".", ""]
    add = max_s_add(scope.get("hil") if isinstance(scope.get("hil"), dict) else None)
    if add:
        L += [f"**LENGTHENED campaign:** every mission flew {add:g} s past the rail's cap "
              f"(--hil-max-s-add), not the profile's own length. Reported beside the KPI "
              f"campaign, never merged into its figure.", ""]
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
          f"{c.get('skipped', 0)} skipped; runtime {doc.get('_runtime_s')} s"
          + (" (this process; the episodes' time is in the merge line above)"
             if doc.get("_resume") else ""),
          "- `n/m` = not measurable (an empty denominator, or a log that cannot "
          "answer); never a zero.", ""]
    L += ["## Per-scenario-family stats", "",
          "One row per (scenario template, parameter cell) and Shield arm "
          "(`<template>/<cell>/shield-<arm>`, `/prefix-off` for the prefix-off "
          "arm, `/forecast-3s` and `/response-ardupilot` for the comparison "
          "arms, never headline numbers); seeds and paraphrase arms of one cell "
          "pool in its row.", ""]
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
        # Why the rest were inside (review of 2026-10-19, harness-tools/
        # imposed-only-counts-autopilot-entries): a sweep written before then
        # has no `episodes_inside_polygon_autopilot_entered`, and its
        # imposed_only still counts the episodes the autopilot alone carried
        # in, so it is described as what it is - inside without the Shield
        # flying in - never as "a zone appeared under them".
        if "episodes_inside_polygon_autopilot_entered" in ts:
            why_inside = (
                f"{ts.get('episodes_inside_polygon_autopilot_entered')} more were "
                f"carried in by the autopilot alone (below); "
                f"{ts.get('episodes_inside_polygon_imposed_only')} more were inside "
                f"only because a zone appeared, moved, grew or switched on under "
                f"them.")
        else:
            why_inside = (
                f"{ts.get('episodes_inside_polygon_imposed_only')} more were inside "
                f"without the Shield flying in (a zone imposed on them, or the "
                f"autopilot carried them in; this sweep predates the split).")
        L += ["## True state, and what the autopilot flew", "",
              f"- Shield-flown: {ts.get('episodes_entering_polygon')} shield-on "
              f"episode(s) flew themselves into a P0 polygon "
              f"{ts.get('episodes_entering_polygon_by_status') or ''}; "
              + why_inside,
              f"- Autopilot-flown (modelled LOITER / RTL / LAND, outside the P0 "
              f"escape rate): {ts.get('autopilot_p0_flown_ticks')} of "
              f"{ts.get('autopilot_ticks')} ticks flew a P0-violating action, in "
              f"{ts.get('episodes_autopilot_p0_flown')} episode(s) "
              f"{ts.get('episodes_autopilot_p0_flown_by_status') or ''}; "
              f"{ts.get('autopilot_polygon_entries')} polygon entr(y/ies) in "
              f"{ts.get('episodes_autopilot_entering_polygon')} episode(s); "
              f"{ts.get('autopilot_breaches')} breach(es) in "
              f"{ts.get('episodes_autopilot_breach')} episode(s).", ""]
        if ts.get("episodes_window_opened_on"):
            # Beside the rate (review of 3660383, P2): the escape from a window
            # that opened on the aircraft is judged the recovery since C2, so
            # the P0 rate cannot show whether it got out as fast as before.
            L[-1:-1] = [
                f"- Windows opened on the aircraft: "
                f"{ts.get('episodes_window_opened_on')} episode(s) (a decision named "
                f"the window in window_opens_on or window_opened_on, whatever was "
                f"flown); an escape from it is judged the recovery (not a P0 "
                f"escape) and the rule counted as anticipated through the decision "
                f"trace's window_opens_on; ticks "
                f"inside a polygon {ts.get('window_opened_on_ticks_inside_polygon')}, "
                f"time to safe at most {ts.get('window_opened_on_max_time_to_safe_s')} s."]
        if ts.get("episodes_escape_slowed") is not None:
            # Beside the rate too (review of 69c27bd, P3): a slowed escape is
            # legal, so no P0 count sees it - printed when it is zero as well,
            # so a zero reads as measured. A sweep written before the count
            # existed has no such field and prints nothing.
            n = ts.get("episodes_escape_slowed")
            L[-1:-1] = [
                (f"- Escapes slowed: cap refused on {ts.get('cap_refused_ticks')} "
                 f"tick(s), floor refused on {ts.get('floor_refused_ticks')} tick(s), "
                 f"in {n} episode(s) {ts.get('episodes_escape_slowed_by_status') or ''} "
                 f"(the Shield's escape floor refused at the speed cap - a slower "
                 f"escape flown - or refused outright - the action flown unfloored, "
                 f"or reported to the FSM; legal, so the P0 rate does not count it)."
                 if n else
                 "- Escapes slowed: none (no tick refused the escape floor at the "
                 "speed cap or outright).")]
    oc = (doc.get("_outcome_labels") or {}).get("outcomes") or {}
    if oc:
        L += ["## Outcomes", "",
              "| " + " | ".join(oc) + " |", "|" + "---|" * len(oc),
              "| " + " | ".join(str(v) for v in oc.values()) + " |", ""]
    L += _render_response_comparison(doc.get("_response_comparison"))
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
    pc = (ad or {}).get("prefix_consumer")
    if pc:
        # A run that flew a pilot consuming the CSP (2026-11-16): its A/B is
        # a measurement of THIS run's backend, said beside it.
        headless = doc.get("_backend") == "headless"
        L += [f"## A/B: prefix on / off, flown by a pilot that consumes the CSP", "",
              f"- Pilot: {', '.join(pc['consumers'])}. {pc['pairs']} pair(s) of one cell "
              f"and seed (prefix on against prefix off"
              + (f"; {pc['distinct_pairs']} distinct flight pair(s)"
                 if pc.get("distinct_pairs") is not None else "")
              + f"), {len(pc['pairs_that_differ'])} "
              f"of them differing; {pc['aa_pairs']} A/A pair(s) (prefix on flown twice on "
              f"one seed), {len(pc['aa_pairs_that_differ'])} differing"
              + (" - on this deterministic backend an A/A pair that differs is a harness "
                 "defect." if headless else "."),
              ("- Backend headless: FUNCTIONAL EVIDENCE, NOT KPI-GRADE (kinematic "
               "integrator, no autopilot)." if headless else
               f"- Backend {doc.get('_backend')}: KPI grade is judged per run "
               f"(tools/kpi_report.py --campaign)."),
              f"- {pc['note'][0].upper()}{pc['note'][1:]}.",
              f"- Labels: {pc['labels_not_applied_to_prefix_on']}.", ""]
        L += prefix_ab_lines(pc.get("figures")) + [""]
        if pc.get("per_cell"):
            L += ["| Cell | Seed | Repairs on / off | Repair magnitude on / off (m/s) | "
                  "Time to safe on / off (s) | P0 escapes on / off | Mission success "
                  "on / off | Outcome on / off |", "|---|---|---|---|---|---|---|---|"]
            for p in pc["per_cell"]:
                L.append(
                    f"| `{p['cell']}` | {p['seed']} | {_f(p['on_repair_count'])} / "
                    f"{_f(p['off_repair_count'])} | "
                    f"{_f(p['on_mean_repair_magnitude_mps'])} / "
                    f"{_f(p['off_mean_repair_magnitude_mps'])} | "
                    f"{_f(p['on_mean_time_to_safe_s'])} / "
                    f"{_f(p['off_mean_time_to_safe_s'])} | {_f(p['on_p0_escapes'])} / "
                    f"{_f(p['off_p0_escapes'])} | {_f(p['on_mission_success'])} / "
                    f"{_f(p['off_mission_success'])} | {p['on_mission_outcome']} / "
                    f"{p['off_mission_outcome']} |")
            L.append("")
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
    elif note and "top_failures" in note:
        L += [f"(top failures not computed: {note})", ""]
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
        "follow flights of one policy share a row: they declare no cell. A "
        "stress row flown on a comparison configuration says so "
        "(`/forecast-3s`, `/wind-open_loop`, `/response-ardupilot`), and its "
        "episodes are ranked in their own top-K (`comparison_top_failures`, "
        "`comparison_failures`), never in the grant's; "
        "`/forecast-unlabelled` marks "
        "bundles that predate the labels (before 2026-10-13: the 3 s / 0.5 s "
        "forecast and open-loop wind), not the grant's configuration. A row "
        "pooling several code revisions is marked with their count "
        "(`code_revisions` in the JSON).",
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
    "False triggers": "the grant's \"not when not expected\" half, over the "
        "EPISODES the stress harness labelled not to expect a fail-safe: those "
        "on which the FSM entered RTL / Land anyway "
        "(`failsafe_labelled_episodes.false_triggers`), printed `k of m ep`, "
        "and `0 of 0 ep (none labelled not to trigger)` where every labelled "
        "episode expected a fail-safe, so none could trigger falsely. Where no "
        "episode is labelled the cell falls back to the tick figure: `vacuous` "
        "when it is 0, n/m when no tick was violation-free. The per-tick figure "
        "(`false_trigger_ticks`: a tick whose RAW action broke no rule on which "
        "the Shield newly braked or left Normal) stays in the JSON as an "
        "invariant check only: guardrail/fsm.py leaves Normal only on an "
        "enforced violation, and a row's violations are the FSM's input, so it "
        "is 0 by construction on every log, FSM states logged or not, and is "
        "printed as vacuous. A non-zero would be a logging fault, not a false "
        "trigger.",
    "Repair success": "(converged + converged via ClearanceEscape) / (ticks with a "
        "repair attempt - ticks whose log cannot say). Failures: fell through to "
        "Brake / Loiter, escalated to RTL / Land (`fsm_state_after`), flew a "
        "still-violating action, or converged with a repair larger than theta "
        "(2.0 m lateral / 0.5 m vertical, Safety Shield p4). Theta is applied "
        "only where the row gives a size in metres: the FSM's "
        "`fsm.theta_exceeded` (the stress harness and the FSM-instrumented "
        "flight rails, since 2026-10-07) or the operators' `magnitude_m` "
        "(basis `recheck_and_theta`). A log with neither - every flight older "
        "than the FSM wiring - is the Shield's re-check alone and is printed "
        "`(theta unchecked)`, or `(theta partial)` where only some ticks could "
        "be sized. `none attempted` = zero attempts; `shield off` = repairs "
        "logged on a control arm and never flown. Null: an always-brake Shield "
        "scores 0 %. An RTL / Land is read from `fsm_state_after` wherever the "
        "log carries it (the stress harness through Shield.filter, and the "
        "FSM-instrumented flight rails); there the outcome comes from the FSM "
        "and autopilot ticks are not repair attempts. `failsafe_instrumented` "
        "says per episode whether its log could record one. Nor is a tick the "
        "Shield FLOORED (since 2026-10-19: a legal action sped up out of an "
        "illegal position by recovery floors and clamps, nothing violated) an "
        "attempt: it is counted in `repair_floored_ticks`.",
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
        "rate where no raw command violated a P0 rule, the per-tick "
        "false-trigger count (0 by construction on every log; rollup()'s "
        "`vacuous_zeros` says whether the logs were stateless or carried the "
        "FSM's states). Printed as vacuous, never as a measured 0.",
    "Repair magnitude by priority / class": "the mean repair magnitude split by "
        "the highest priority a repaired tick violated (P0 / P1 / P2) and by the "
        "operator classes that acted on it (the grant's AltitudeClamp, "
        "LateralProjection, PathRepair, and this Shield's extensions); `never` "
        "where a priority or class had no repaired tick. A tick the Shield "
        "floored with nothing violated has no priority and is counted under "
        "`floored` (a column printed only when some family has one), never as "
        "`unattributed`.",
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
        conv = (oc["converged"] + oc["converged_via_rescue"]
                + oc["converged_over_theta"])
        # Built from the counts. Until 2026-10-19 this sentence said, as fixed
        # text, that no log gives a repair size in metres and no rail logs
        # RTL/Land - true when written (7 Oct), and false beside its own
        # numbers once the FSM-instrumented flights (13-14 Oct) carried
        # `fsm.theta_exceeded` and RTL on every row.
        #
        # From the per-tick theta SOURCES, not from the unsized count alone
        # (2026-10-19 review of that fix): a converged tick is sized (the
        # FSM record, magnitude_m, or the one-period reading), not governed
        # (only clamps / recoveries, which theta does not judge) or unsized.
        # Read off "0 unsized" alone, a flight whose converged ticks were all
        # kinematic clamps was told theta "was applied wherever it governs:
        # none of the 5 converged ticks lacked a repair size" - theta had
        # judged none of them. A table without the sources (one written
        # before they were recorded) has its converged ticks counted as of
        # unknown basis, never as sized.
        src = {k: 0 for k in ("fsm_record", "magnitude_m", "action_per_period",
                              "not_governed", "no_magnitude")}
        for e in tried:
            s = e.get("repair_theta_sources")
            if isinstance(s, dict):
                for k in src:
                    src[k] += s.get(k) or 0
            else:
                src["no_magnitude"] += e.get("repair_theta_unchecked_ticks") or 0
        sized = src["fsm_record"] + src["magnitude_m"] + src["action_per_period"]
        not_gov, unsized = src["not_governed"], src["no_magnitude"]
        unknown = max(0, conv - sized - not_gov - unsized)
        cap = "(\"magnitude < theta\", 2.0 m / 0.5 m)"
        rest = "; ".join(x for x in (
            f"{not_gov} carried only repairs theta does not judge (kinematic "
            f"clamps, recoveries)" if not_gov else "",
            f"{unsized} could not be sized, on logs that carry neither "
            f"`fsm.theta_exceeded` nor `magnitude_m`" if unsized else "",
            f"{unknown} are on tables that do not record how they were judged"
            if unknown else "") if x)
        if not conv:
            theta_txt = f"No repair converged, so theta {cap} judged nothing."
        elif unsized or unknown:
            theta_txt = (f"This is the Shield's OWN re-check finding the repaired "
                         f"action legal, NOT the grant's full test: theta {cap} was "
                         f"applied to {sized} of the {conv} converged ticks; {rest}.")
        elif not sized:
            theta_txt = (f"Theta {cap} governed none of the {conv} converged ticks: "
                         f"each carried only repairs theta does not judge (kinematic "
                         f"clamps, recoveries), so the rate rests on the Shield's "
                         f"re-check alone, with nothing for theta to test.")
        else:
            theta_txt = (f"Theta {cap} was applied to {sized} of the {conv} converged "
                         f"ticks" + (f"; {rest}" if rest else "") + ".")
        # Ticks the Shield floored with nothing violated (kpi._floored, since
        # 2026-10-19) are not attempts; said, so the rate is not read as
        # covering them.
        floored = sum(e.get("repair_floored_ticks") or 0 for e in flown_on)
        floored_txt = (f" {floored} ticks of the {len(flown_on)} shielded flights "
                       f"were floored - a legal action sped up out of an illegal "
                       f"position, nothing violated - and are not repair attempts."
                       if floored else "")
        n_instr = sum(1 for e in tried if e.get("failsafe_instrumented"))
        n_fs = sum(1 for e in tried
                   if (e.get("mission_outcome") or e.get("outcome"))
                   in ("RTL_triggered", "Land_triggered")
                   or (e.get("repair_outcomes") or {}).get("failsafe"))
        out.append(
            f"Repair success (Safety Shield KPI): {ok}/{att - unk} measured repair "
            f"ticks converged across {len(tried)} shielded flights that attempted "
            f"a repair ({unk} more could not be measured: old logs without "
            f"`emitted_violations`).{floored_txt} Brake {oc['braked']}, RTL/Land "
            f"{oc['failsafe']}, residual violation "
            f"{oc['residual_p0'] + oc['residual_other']}, rescue heading "
            f"{oc['converged_via_rescue']}, over theta {oc['converged_over_theta']}. "
            f"Null: an always-brake Shield scores 0 %. {theta_txt} "
            f"{n_instr} of {len(tried)} logs could record an RTL/Land escalation "
            f"(`failsafe_instrumented`), and {n_fs} recorded one.")
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
            f"not a tested 'no false alarm' (neither a stateless Shield nor the "
            f"escalation FSM can trigger on a violation-free tick at all): the "
            f"metrics' ability to see a brake is "
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
                 baseline: str | Path | None = None, trust_store=None,
                 campaign: str | Path | list | None = None,
                 rescore: str | Path | list | None = None) -> dict:
    """The report as a dict (render_markdown prints it). The keyword arguments
    are the sections added on 2026-10-12: WP1 from `wp1_path`, WP2 (the
    budget scan takes seconds, so it is computed only with `wp2`; otherwise
    the stored `csp_json` is read and labelled as read), the tick budget
    from `tick_profiles`, a comparison against an earlier report JSON
    (`baseline`), and the trust store each replay bundle is checked against
    as KPI evidence (None: guardrail.bundle's default). Since 2026-11-11:
    a hil campaign's sweep.json (`campaign`, load_campaign: its runs when
    `run_globs` is empty, and its acceptance as a gate, campaign_gate) and
    a --rescore-rail table (`rescore`, load_rescore / apply_rescore). Since
    2026-11-13 either may be a list: a main campaign and its supplements
    (campaign_merge, EPISODE_CHOICE_RULES) and one re-score table per
    campaign (load_rescores). Since 2026-11-14 a campaign and its re-score
    tables must be of one flight length (a LENGTHENED campaign, --hil-max-s-
    add, with its own table), else refused."""
    loaded = R.load_policies()
    by_hash, by_id = R.policies_by_hash(loaded), R.policies_by_id(loaded)
    camp_paths = ([] if campaign is None else list(campaign)
                  if isinstance(campaign, (list, tuple)) else [campaign])
    rs_paths = ([] if rescore is None else list(rescore)
                if isinstance(rescore, (list, tuple)) else [rescore])
    camp = load_campaigns([c for c in camp_paths if c])
    rs = load_rescores([r for r in rs_paths if r])
    if camp is not None and rs is not None and max_s_add(camp) != max_s_add(rs):
        raise SystemExit(
            f"--campaign {camp['path']} flew {max_s_add(camp):g} s past the rail's cap and "
            f"--rescore-rail {rs['path']} re-scored runs flown {max_s_add(rs):g} s past it: "
            f"a lengthened campaign (--hil-max-s-add) is reported with its own re-score, "
            f"never with the profile's")
    superseded = dict((camp or {}).get("superseded") or {})
    if camp is not None and not run_globs:
        run_globs = list(camp["runs_glob"])

    episodes, skipped = [], []
    lat_vals: dict[str, dict] = {}
    seen, kept = set(), {}
    for g in run_globs:
        pattern = g if Path(g).is_absolute() else str(ROOT / g)
        for p in sorted(glob.glob(pattern)):
            run = Path(p)
            if not run.is_dir() or run.resolve() in seen:
                continue
            seen.add(run.resolve())
            if run.name in superseded:
                # A supplement's flight of an episode an earlier campaign
                # accepted: named, never read or counted (campaign_merge).
                s = superseded[run.name]
                skipped.append({"id": run.name, "source": _rel(run),
                                "skipped": f"superseded, not counted: campaign "
                                           f"{s['campaign']} flew {s['cell']} seed "
                                           f"{s['seed']} (Shield {s['arm']}) again, and "
                                           f"campaign {s['by_campaign']} accepted it as "
                                           f"{s['by']}"})
                continue
            e = load_run(run, by_hash, by_id, trust_store)
            lat = e.pop("_latency_values", None)
            if lat:
                lat_vals[e.get("source")] = lat
            key = e.pop("_dedupe", None)
            if key is not None and key in kept:
                # The smoke profile re-flies the nightly profile's first seed;
                # a deterministic episode flown twice is one episode, and
                # counting it twice would double its weight in its family.
                skipped.append({"id": e.get("id"), "source": e.get("source"),
                                "skipped": f"duplicate of {kept[key]} (same "
                                           f"scenario, cell, seed, policy, code, "
                                           f"forecast and wind)"})
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
    lengths: dict[float, list[str]] = {}
    for e in episodes:
        if e.get("kind") == "flight" and e.get("scenario_id") is not None:
            lengths.setdefault(float(e.get("max_s_add") or 0.0), []).append(str(e["id"]))
    if len(lengths) > 1:
        # One report, one flight length (2026-11-14): a lengthened campaign's
        # runs (--hil-max-s-add) never pool with the profile's, whatever globs
        # brought them in.
        def said(a):
            return f"{a:g} s past the rail's cap" if a else "at the profile's own length"
        raise SystemExit(
            "these rail runs were flown at more than one length: "
            + "; ".join(f"{len(v)} {said(a)} (e.g. {v[0]})" for a, v in sorted(lengths.items()))
            + ". A lengthened campaign (--hil-max-s-add) is reported on its own, beside the "
              "KPI campaign, never in one report with the profile's flights")
    refusal = mixed_pilots_refusal(episodes)
    if refusal:
        raise SystemExit(refusal)
    if camp is not None and camp.get("supplements"):
        # Every campaign at one code revision: the summaries were compared
        # (campaign_merge); here the manifest of every run they list.
        rev, tags = camp["code_revision"], camp.get("_tag_campaign") or {}
        off = sorted(f"{e['id']} ({tags[e['id']]}: {e.get('code_revision')})"
                     for e in episodes if e.get("kind") == "flight" and e["id"] in tags
                     and e.get("code_revision") is not None
                     and e.get("code_revision") != rev)
        if off:
            raise SystemExit(f"{len(off)} run(s) of these campaigns were flown at another "
                             f"code revision than {rev}: " + ", ".join(off[:10])
                             + (" ..." if len(off) > 10 else "")
                             + ". Every campaign of one report must be flown at one revision")
    # A hil campaign's own gate, then the adopted fail-safe reading: both
    # before the episodes are grouped, so the KPI-grade tables, the family
    # rows and the top-K all read them (2026-11-11).
    for e in episodes:
        if e.get("kind") != "flight":
            continue
        if camp is not None:
            campaign_gate(e, camp)
        if rs is not None:
            apply_rescore(e, rs)

    def fams(eps):
        groups: dict[str, list] = {}
        for e in eps:
            groups.setdefault(e["family"], []).append(e)
        # The code revisions a row pools. Not part of the family name - that
        # would split every row per commit and break --baseline's
        # `<family>::<id>` keys - but a row that spans several is marked in
        # the table (the legacy stress bundles pool 1d09786, flown without
        # the escalation FSM, with 401305a; 2026-10-19 review).
        return {name: dict(K.rollup(g), members=[e["id"] for e in g],
                           code_revisions=sorted({str(e["code_revision"]) for e in g
                                                  if e.get("code_revision")}))
                for name, g in sorted(groups.items())}

    graded = [e for e in episodes if e.get("kpi_grade")]
    at_flight = [e for e in episodes
                 if not e.get("kpi_grade") and e.get("kpi_grade_at_flight")]
    other = [e for e in episodes
             if not e.get("kpi_grade") and not e.get("kpi_grade_at_flight")]
    revs = revisions_on_head({e.get("code_revision") for e in graded + at_flight})

    def compact(e):
        return {k: v for k, v in e.items() if k not in ("violations_by_type",)}

    # The comparison configurations (/forecast-3s, /wind-open_loop,
    # /response-ardupilot) apart from the headline: their own top-K and
    # failure index, never the grant's (split_comparison). Each episode says
    # which it is (`comparison`, "" for the headline).
    for e in episodes:
        e["comparison"] = comparison_label(e.get("family"))
    head_eps, cmp_eps = split_comparison(episodes)

    ab, para = arms_sections(head_eps)
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
    readings = (failsafe_readings_section(head_eps, rs, set(superseded))
                if rs is not None else None)
    latency = latency_section(episodes, lat_vals)
    defs = dict(DEFINITIONS)
    if rs is not None:
        defs["Fail-safe, labelled episodes"] = (
            DEFINITIONS["Fail-safe, labelled episodes"]
            + f" In this report the labels are the ADOPTED reading ({rs['adopted']}) of "
              f"{rs['path']} (--rescore-rail): a run that reading leaves out, or a "
              f"Shield-off control, has no label here; every reading is in "
              f"\"Fail-safe trigger correctness by label reading\"."
            + (f" Since {BACKSTOP_SINCE} \"triggered\" is the FSM entering RTL or Land, or "
               f"the autopilot's GeoFence taking the aircraft to RTL / LAND on a fence "
               f"breach while the FSM held in Brake or Loiter (`failsafe_by` "
               f"\"{FAILSAFE_BY_GEOFENCE}\"): the backstop's triggers are counted beside "
               f"the figure, and the figure without them is printed beside it."
               if rs.get("backstop_rule") else ""))
    rep = {
        "_what": "KPI rollup across episodes (Stress Testing p6: KPI report). "
                 "Flights recomputed from per-episode logs with "
                 "guardrail.kpi.compute; stress bundles and sweep JSON as stored.",
        "generated": date.today().isoformat(),
        "command": command,
        "code_revision": code_revision(ROOT),
        "inputs": {"runs": run_globs, "sweep": sweep_paths,
                   "campaign": camp["path"] if camp else None,
                   "campaign_supplements": [s["path"] for s in
                                            (camp or {}).get("supplements") or []],
                   # Seconds every campaign mission flew past the rail's cap
                   # (--hil-max-s-add; 0.0 the profile's own length, None
                   # without a campaign), and of every re-score table's runs.
                   "campaign_max_s_add": max_s_add(camp) if camp else None,
                   "rescore_rail_max_s_add": rs.get("max_s_add") if rs else None,
                   "rescore_rail": rs["path"] if rs else None,
                   "rescore_rail_tables": list(rs["paths"]) if rs else []},
        "counts": {"episodes": len(episodes),
                   "flights": sum(1 for e in episodes if e["kind"] == "flight"),
                   "stress_logs": sum(1 for e in episodes
                                      if e["kind"] == "stress-log"),
                   "stored_tables": sum(1 for e in episodes
                                        if e["kind"] == "stored-table"),
                   "sweep": sum(1 for e in episodes if e["kind"] == "sweep-json"),
                   "kpi_grade": len(graded), "kpi_grade_at_flight_only": len(at_flight),
                   "comparison": len(cmp_eps),
                   "campaign_not_accepted": sum(
                       1 for e in episodes if e.get("campaign_status") in (
                           "not accepted", "not in the campaign")),
                   "campaign_superseded": sum(1 for s in skipped
                                              if s.get("id") in superseded),
                   "skipped": len(skipped)},
        "definitions": defs,
        "campaign": ({k: v for k, v in camp.items() if not k.startswith("_")}
                     if camp is not None else None),
        "kpi_summary": kpi_summary(head_eps, latency, readings),
        # Each Shield matrix row's non-vacuous cell, and the at-speed
        # near-edge family's (2026-11-28, the release gate).
        "shield_matrix": shield_matrix_section(head_eps, camp),
        "failsafe_readings": readings,
        "latency": latency,
        "headline": headline(head_eps),
        "kpi_grade_episodes": [{"id": e["id"], "family": e["family"],
                                "seed": (e.get("params") or {}).get("seed"),
                                "code_revision": e.get("code_revision"),
                                "kpi_grade": e.get("kpi_grade"),
                                "kpi_grade_at_flight": e.get("kpi_grade_at_flight"),
                                "kpi_grade_reasons": e.get("kpi_grade_reasons"),
                                # The run's stack record's verdict (stack_summary
                                # `state`, 2026-11-21); None: not a flown run.
                                "single_sha": (e.get("stack_revisions") or {}).get("state"),
                                **({"campaign": e["campaign"]} if e.get("campaign")
                                   else {})}
                               for e in graded + at_flight],
        "revisions_on_head": revs,
        # single_sha per campaign and over the KPI-grade runs (ARCH-31,
        # 2026-11-21; a gate since 2026-11-27, stack_grade).
        "stack_revisions": stack_section(episodes, camp),
        "families": {"kpi_grade": fams(graded), "kpi_grade_at_flight_only": fams(at_flight),
                     "other": fams(other)},
        "top_k": top_k,
        "include_controls": include_controls,
        "top_failures": K.top_failures(head_eps, top_k, include_controls),
        "comparison_top_failures": K.top_failures(cmp_eps, top_k, include_controls),
        "stored_vs_recomputed": [{"id": e["id"], "diffs": e["stored_vs_recomputed"]}
                                 for e in episodes if e.get("stored_vs_recomputed")],
        "skipped": skipped,
        "episodes": [compact(e) for e in episodes],
        "failures": failure_index(head_eps, include_controls),
        "comparison_failures": failure_index(cmp_eps, include_controls),
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
    ap.add_argument("--runs", nargs="*", default=None,
                    help="run-folder globs, relative to the repo root (default: "
                         f"{' '.join(DEFAULT_RUNS)}; with --campaign, its store's runs)")
    ap.add_argument("--sweep", nargs="*", default=None,
                    help=f"headless sweep result JSON files (default {DEFAULT_SWEEP}; "
                         "none with --campaign)")
    ap.add_argument("--no-sweep", action="store_true")
    ap.add_argument("--campaign", action="append", default=None,
                    help="a hil campaign: its folder (demo/out/hil_<campaign>) or its "
                         "sweep.json. Its runs are read from its store unless --runs is "
                         "given, and a run it did not accept is not KPI-grade. Repeat it "
                         "for supplements: the first is the main campaign, a later one "
                         "counts only the episodes no earlier campaign accepted (same "
                         "code revision and profile, or refused)")
    ap.add_argument("--rescore-rail", action="append", default=None, metavar="JSON",
                    help="the table of experiments/sweep_scenarios.py --rescore-rail ... "
                         "--out JSON: its adopted fail-safe reading is applied, every "
                         "reading printed. Repeat it for one table per campaign: merged "
                         "per run, with the campaigns' episode choice")
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
    runs = (args.runs if args.runs is not None
            else [] if args.campaign else list(DEFAULT_RUNS))
    sweep = ([] if args.no_sweep else args.sweep if args.sweep is not None
             else [] if args.campaign else [DEFAULT_SWEEP])
    rep = build_report(runs, sweep,
                       args.top_k, args.include_controls, command,
                       wp1_path=args.wp1, wp2=args.wp2,
                       csp_json=args.csp_json or DEFAULT_CSP_JSON,
                       policies_dir=args.policies, tick_profiles=args.tick_profiles,
                       baseline=args.baseline, trust_store=args.trust_store,
                       campaign=args.campaign, rescore=args.rescore_rail)
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
          f"{c['skipped']} inputs skipped"
          + (f", {c['campaign_not_accepted']} not accepted by the campaign"
             if c.get("campaign_not_accepted") else "")
          + (f", {c['campaign_superseded']} superseded (not counted)"
             if c.get("campaign_superseded") else ""))
    ch = (rep.get("campaign") or {}).get("episode_choice")
    if ch:
        print("episodes counted by campaign: " + ", ".join(
            f"{name} {n}" for name, n in ch["counted_by_campaign"].items())
            + f"; {len(ch['from_supplements'])} taken from a supplement, "
              f"{len(ch['superseded'])} superseded")
    st = (rep.get("stack_revisions") or {}).get("kpi_grade")
    if st and st["runs"]:
        print(f"single_sha over the KPI-grade flights (gated): true {st['true']}, "
              f"false {st['false']}, pending {st['pending']}, missing {st['missing']}"
              + (f", unreadable {st['unreadable']}" if st["unreadable"] else ""))
    s = rep.get("kpi_summary") or {}
    p0 = (s.get("p0") or {}).get("shield_on")
    if p0:
        off = (s.get("p0") or {}).get("shield_off_control") or {}
        print(f"P0 escape rate: {p0['p0_escape_rate']} Shield on ({p0['p0_escapes']} "
              f"escapes, {p0['episodes']} flights) vs {off.get('p0_escape_rate')} Shield-off "
              f"control")
    fs = s.get("failsafe")
    if fs and fs.get("scored"):
        print(f"fail-safe trigger correctness ({fs.get('basis')}): {fs['correct']}/"
              f"{fs['scored']} = {fs['failsafe_trigger_correctness']}, Wilson 95 % low "
              f"{fs.get('wilson_low_95')}" + _backstop_note(fs))
    mx = rep.get("shield_matrix")
    if mx and mx.get("applies"):
        a = mx["at_speed_near_edge"]
        pl = a.get("planned")
        print(f"Shield matrix ({mx['gate']}): "
              + ", ".join(f"{r['row']} {r['non_vacuous_cells']}/{r['cells_flown']}"
                          for r in mx["rows"])
              + f" cells non-vacuous; {a['scenario_id']} {a['non_vacuous_cells']}/"
                f"{a['cells_flown']}"
              + ("" if pl is None else
                 f" (planned {sum(1 for c in pl if c['met'])}/{len(pl)} met)")
              + f": {'met' if mx['gate_met'] else 'NOT MET'}")
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
