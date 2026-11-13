"""Per-scenario perception metrics of the Project AirSim perception-stress runs.

    python tools/perception_stress_report.py demo/out/pas_hil_aiot_night_* demo/out/pas_hil_aiot_day_*
    python tools/perception_stress_report.py RUN... --json report.json --md report.md

WP4-18 asks for lighting / weather stressors on Project AirSim "with a
per-scenario perception metric", WP4-16 for perception-stress scenarios run
on the PAS + ArduPilot rail and labelled non-KPI. The stressors are
demo/pas_ardupilot/perception_stress.py (perception_node.py --stress); this
tool scores what each run already logs and groups the runs by stressor.
Nothing is flown and nothing in a run folder is written.

WHAT IT READS, per run folder (perception_node.py writes all three)

  metrics.json       perception_stress (the stressor, its scenario, the
                     weather as set or read back; absent on a run flown
                     without --stress, which is grouped as "unstressed"),
                     det_hz, end_reason, mission_s
  flight_log.jsonl   one row per tick: t, det_seq, subject (the estimate the
                     Shield flew on), subject_truth / subject_range_m /
                     subject_gap_m (the subject's simulator pose, record only)
  detections.jsonl   one record per detector inference: seq, det (the box and
                     its score, or null), switched, luma (the frame's mean
                     grey level, since this tool; older runs have none)
  kpi.json           kpi_grade, only to check a stressed run was not graded

THE METRICS, per run (each None when what it needs was not logged)

  detector, over the inferences of the MISSION: those whose seq lies between
  the first and the last det_seq a flight-log row carries (no clock is
  matched: detections.jsonl's t is wall time, the rows' t mission time)
    det_hit_rate      inferences that returned a box / inferences
    det_conf_mean     mean score of the returned boxes (OWL-ViT's, 0-1)
    det_conf_median   their median
    det_switches      inferences on which the box switched object
    luma_mean         mean of the frames' grey level (0-255): the check that
                      a dusk / night run was really darker than the baseline
    det_hz            metrics.json's (inferences per mission second)
  tracking, over the flight-log rows
    estimate_frac     ticks with a subject estimate / ticks
    lock_frac         TARGET LOCK: ticks whose estimate lies within
                      ESTIMATE_NEAR_M (6 m) of the subject's true position /
                      ticks whose truth was read (a tick with no estimate, or
                      one on something else, is not locked)
    estimate_on_subject_frac  the same ticks / ticks with both an estimate and
                      the truth (perception_node's truth_view
                      estimate_on_subject: is an estimate, when there is one,
                      the subject?)
    track_losses      times the lock was lost for at least LOSS_MIN_S (1 s,
                      by the rows' own t) after it had been held; a loss still
                      open at the end counts
    longest_loss_s    the longest span without the lock after it had been
                      held, of any length (0.0: never lost)
    estimate_dropouts the same count on estimate presence alone (no truth
                      needed)
  following, over the ticks whose truth was read
    within_30m_frac   ticks within FOLLOW_RANGE_M (30 m) of the subject
                      (perception_node's follow proxy, of the ticks read)
    range_median_m, range_p90_m           the true horizontal range
    estimate_err_median_m, _p90_m          |estimate - truth| (subject_gap_m)

PER SCENARIO: the mean of each metric over the scenario's runs, its delta
from the baseline scenario (day, else the unstressed runs), and checks that
refuse to let a run pass in silence: a dusk / night scenario whose frames
are not darker than the baseline's, a weather stressor whose read-back did
not verify, a stressed run graded KPI. Every figure is non-KPI evidence.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path

# perception_node.ESTIMATE_NEAR_M: an estimate within this of the truth is
# "on the subject" (track_truth.score_estimate's near_m).
ESTIMATE_NEAR_M = 6.0
# perception_node.FOLLOW_RANGE_M: the follow proxy's range.
FOLLOW_RANGE_M = 30.0
# A lock or an estimate gone for at least this long is a loss: the detector's
# own staleness (perception_node.DETECTION_STALE_S, follow_vlm's "fresh").
LOSS_MIN_S = 1.0
# A dusk / night scenario whose mean frame luma is not below this share of
# the baseline's is flagged: the exposure may not have reached the cameras.
DARK_MAX_RATIO = 0.85
BASELINE = "day"
UNSTRESSED = "unstressed"
LABEL = "non-KPI (perception-stress evidence, WP4-16 / WP4-18)"
DARK_STRESSORS = ("dusk", "night")
WEATHER_STRESSORS = ("rain", "fog")

# The headline metrics: (key, what) in the report's order.
HEADLINE = (
    ("det_hit_rate", "detector hit rate"),
    ("det_conf_mean", "mean box score"),
    ("det_hz", "detector Hz"),
    ("luma_mean", "frame luma"),
    ("estimate_frac", "estimate share"),
    ("lock_frac", "target lock"),
    ("estimate_on_subject_frac", "estimate on subject"),
    ("track_losses", "track losses"),
    ("longest_loss_s", "longest loss s"),
    ("within_30m_frac", "within 30 m"),
    ("range_median_m", "range median m"),
    ("estimate_err_median_m", "estimate error m"),
)


def _read_json(p: Path):
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def _read_jsonl(p: Path) -> list[dict] | None:
    """The records of a .jsonl file; None when it is absent. A line that is
    not JSON (a run killed mid-write) is skipped."""
    if not p.is_file():
        return None
    out = []
    for line in p.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except ValueError:
            continue
        if isinstance(rec, dict):
            out.append(rec)
    return out


def _num(v) -> float | None:
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(v) else None


def _r(v, nd: int = 4):
    return None if v is None else round(v, nd)


def _pct(vals: list[float], q: float) -> float | None:
    """Nearest rank (track_truth.score_estimate's)."""
    if not vals:
        return None
    s = sorted(vals)
    return s[min(len(s) - 1, int(q * len(s)))]


def detector_metrics(dets: list[dict] | None, rows: list[dict]) -> dict:
    """The detector's figures over the mission's inferences (module
    docstring): those whose seq lies in the rows' det_seq span; all of them
    when no row carries one."""
    if dets is None:
        return {"inferences": None, "why": "no detections.jsonl"}
    seqs = [r["det_seq"] for r in rows if isinstance(r.get("det_seq"), int)
            and not isinstance(r.get("det_seq"), bool)]
    if seqs:
        lo, hi = min(seqs), max(seqs)
        win = [d for d in dets if isinstance(d.get("seq"), int) and lo <= d["seq"] <= hi]
        window = f"seq {lo}-{hi} (the flight log's det_seq span)"
    else:
        win, window = list(dets), "every inference (no flight-log row carries det_seq)"
    hits = [d for d in win if isinstance(d.get("det"), dict)]
    scores = [s for s in (_num(d["det"].get("score")) for d in hits) if s is not None]
    lumas = [v for v in (_num(d.get("luma")) for d in win) if v is not None]
    n = len(win)
    return {"window": window, "inferences": n, "hits": len(hits),
            "det_hit_rate": _r(len(hits) / n) if n else None,
            "det_conf_mean": _r(statistics.fmean(scores)) if scores else None,
            "det_conf_median": _r(statistics.median(scores)) if scores else None,
            "det_switches": sum(1 for d in win if d.get("switched")),
            "luma_n": len(lumas),
            "luma_mean": _r(statistics.fmean(lumas), 1) if lumas else None,
            "luma_median": _r(statistics.median(lumas), 1) if lumas else None}


def _losses(rows: list[dict], state) -> tuple[int | None, float | None]:
    """(count, longest s) of the spans in which `state(row)` was False for at
    least LOSS_MIN_S after it had been True; a row where it is None (not
    judged) neither starts nor ends a span. (None, None) when it was never
    True."""
    held = False
    start = None
    spans: list[float] = []
    last_t = None
    for r in rows:
        t = _num(r.get("t"))
        s = state(r)
        if t is None or s is None:
            continue
        last_t = t
        if s:
            if start is not None:
                spans.append(t - start)
                start = None
            held = True
        elif held and start is None:
            start = t
    if start is not None and last_t is not None:
        spans.append(last_t - start)                  # still lost at the end
    if not held:
        return None, None
    long_ = [d for d in spans if d >= LOSS_MIN_S]
    return len(long_), _r(max(spans), 2) if spans else 0.0


def _on_subject(r: dict) -> bool | None:
    """Is this tick locked: an estimate within ESTIMATE_NEAR_M of the truth?
    None when the truth was not read."""
    tr = r.get("subject_truth")
    if not (isinstance(tr, (list, tuple)) and len(tr) >= 2):
        return None
    s = r.get("subject")
    if not (isinstance(s, (list, tuple)) and len(s) >= 2):
        return False
    try:
        return math.hypot(float(s[0]) - float(tr[0]), float(s[1]) - float(tr[1])) \
            <= ESTIMATE_NEAR_M
    except (TypeError, ValueError):
        return False


def track_metrics(rows: list[dict]) -> dict:
    n = len(rows)
    if not n:
        return {"ticks": 0}
    has_est = [isinstance(r.get("subject"), (list, tuple)) for r in rows]
    judged = [(r, _on_subject(r)) for r in rows]
    truth = [(r, on) for r, on in judged if on is not None]
    both = [(r, on) for (r, on), e in zip(judged, has_est) if on is not None and e]
    locked = sum(1 for _, on in truth if on)
    losses, longest = _losses(rows, _on_subject)
    drops, _ = _losses(rows, lambda r: isinstance(r.get("subject"), (list, tuple)))
    ranges = [v for v in (_num(r.get("subject_range_m")) for r, _ in truth) if v is not None]
    gaps = [v for v in (_num(r.get("subject_gap_m")) for r, _ in truth) if v is not None]
    return {"ticks": n, "truth_ticks": len(truth),
            "estimate_frac": _r(sum(has_est) / n),
            "lock_frac": _r(locked / len(truth)) if truth else None,
            "estimate_on_subject_frac": (_r(sum(1 for _, on in both if on) / len(both))
                                         if both else None),
            "track_losses": losses, "longest_loss_s": longest,
            "estimate_dropouts": drops,
            "within_30m_frac": (_r(sum(1 for d in ranges if d <= FOLLOW_RANGE_M)
                                   / len(ranges)) if ranges else None),
            "range_median_m": _r(statistics.median(ranges), 2) if ranges else None,
            "range_p90_m": _r(_pct(ranges, 0.9), 2),
            "estimate_err_median_m": _r(statistics.median(gaps), 2) if gaps else None,
            "estimate_err_p90_m": _r(_pct(gaps, 0.9), 2),
            "near_m": ESTIMATE_NEAR_M, "loss_min_s": LOSS_MIN_S}


def run_metrics(run: str | Path) -> dict:
    """One run folder's perception metrics (module docstring). ValueError
    when it holds no metrics.json."""
    run = Path(run)
    metrics = _read_json(run / "metrics.json")
    if not isinstance(metrics, dict):
        raise ValueError(f"{run}: no readable metrics.json")
    rows = _read_jsonl(run / "flight_log.jsonl") or []
    dets = _read_jsonl(run / "detections.jsonl")
    kpi = _read_json(run / "kpi.json")
    ps = metrics.get("perception_stress") if isinstance(
        metrics.get("perception_stress"), dict) else None
    name = ps.get("name") if ps else None
    weather = (ps or {}).get("weather") or {}
    det = detector_metrics(dets, rows)
    det["det_hz"] = _num(metrics.get("det_hz"))
    return {"run": run.name, "path": str(run),
            "stress": name or UNSTRESSED,
            "scenario_id": (ps or {}).get("scenario_id"),
            "label": LABEL,
            "kpi_grade": (kpi or {}).get("kpi_grade") if isinstance(kpi, dict) else None,
            "scene_id": ((metrics.get("sim_config") or {}).get("scene_id")
                         if isinstance(metrics.get("sim_config"), dict) else None),
            "weather_ok": weather.get("ok") if isinstance(weather, dict) else None,
            "weather_read_back": weather.get("read_back") if isinstance(weather, dict) else None,
            "end_reason": metrics.get("end_reason"),
            "mission_s": _num(metrics.get("mission_s")),
            "detector": det, "track": track_metrics(rows)}


def flat(rm: dict) -> dict:
    """A run's HEADLINE metrics, one level deep."""
    src = {**rm.get("detector", {}), **rm.get("track", {})}
    return {k: _num(src.get(k)) for k, _ in HEADLINE}


def scenario_table(runs: list[dict]) -> dict:
    """The runs grouped by stressor: each scenario's mean metrics, its delta
    from the baseline (day, else the unstressed runs), and its checks."""
    groups: dict[str, list[dict]] = {}
    for rm in runs:
        groups.setdefault(rm["stress"], []).append(rm)
    scen = {}
    for name, rms in groups.items():
        means = {}
        for k, _ in HEADLINE:
            vals = [v for v in (flat(r)[k] for r in rms) if v is not None]
            means[k] = _r(statistics.fmean(vals)) if vals else None
        scen[name] = {"stress": name,
                      "scenario_id": next((r["scenario_id"] for r in rms
                                           if r.get("scenario_id")), None),
                      "runs": len(rms), "run_names": [r["run"] for r in rms],
                      "mean": means}
    base = BASELINE if BASELINE in scen else (UNSTRESSED if UNSTRESSED in scen else None)
    for name, s in scen.items():
        b = scen[base]["mean"] if base is not None and name != base else None
        s["baseline"] = None if b is None else base
        s["delta_vs_baseline"] = (None if b is None else
                                  {k: (None if s["mean"][k] is None or b[k] is None
                                       else _r(s["mean"][k] - b[k])) for k, _ in HEADLINE})
        s["checks"] = _checks(name, s, groups[name], scen.get(base) if base else None)
    order = [n for n in ("day", "dusk", "night", "rain", "fog", UNSTRESSED) if n in scen]
    order += sorted(n for n in scen if n not in order)
    return {"label": LABEL, "baseline": base,
            "definitions": {"near_m": ESTIMATE_NEAR_M, "follow_range_m": FOLLOW_RANGE_M,
                            "loss_min_s": LOSS_MIN_S, "dark_max_ratio": DARK_MAX_RATIO},
            "scenarios": [scen[n] for n in order], "runs": runs}


def _checks(name: str, s: dict, rms: list[dict], base: dict | None) -> list[str]:
    out = []
    if name != UNSTRESSED:
        graded = [r["run"] for r in rms if r.get("kpi_grade") is True]
        if graded:
            out.append(f"KPI-graded although stressed: {graded} (a stressed run must not be)")
    if name in DARK_STRESSORS:
        lum, blum = s["mean"]["luma_mean"], None if base is None else base["mean"]["luma_mean"]
        if lum is None:
            out.append("no frame luma recorded: whether the frames were dark is not measured")
        elif blum is None:
            out.append("no baseline luma to compare with: fly the day scenario too")
        elif lum > DARK_MAX_RATIO * blum:
            out.append(f"frames not darker than the baseline's (luma {lum:g} vs {blum:g}): "
                       f"was the scene loaded with -Stress {name}?")
    if name in WEATHER_STRESSORS:
        bad = [r["run"] for r in rms if r.get("weather_ok") is not True]
        if bad:
            out.append(f"weather not verified by read-back on {bad}")
    no_truth = [r["run"] for r in rms if not r["track"].get("truth_ticks")]
    if no_truth:
        out.append(f"no subject truth read on {no_truth}: lock and range not measured")
    return out


def _fmt(v, k: str) -> str:
    if v is None:
        return "-"
    if k in ("track_losses",):
        return f"{v:.1f}"
    if k.endswith("_m") or k.endswith("_s") or k in ("luma_mean", "det_hz"):
        return f"{v:.1f}"
    return f"{v:.3f}"


def to_markdown(table: dict) -> str:
    cols = HEADLINE
    lines = [f"Perception stress on Project AirSim, per scenario - {table['label']}.",
             f"Baseline: {table['baseline'] or 'none'}. Means over each scenario's runs; "
             f"lock = estimate within {ESTIMATE_NEAR_M:g} m of the truth, loss = lock gone "
             f">= {LOSS_MIN_S:g} s.", "",
             "| scenario | runs | " + " | ".join(w for _, w in cols) + " |",
             "|---|---|" + "---|" * len(cols)]
    for s in table["scenarios"]:
        lines.append(f"| {s['scenario_id'] or s['stress']} | {s['runs']} | "
                     + " | ".join(_fmt(s["mean"][k], k) for k, _ in cols) + " |")
    checks = [(s["scenario_id"] or s["stress"], c) for s in table["scenarios"]
              for c in s["checks"]]
    if checks:
        lines += ["", "Checks:"] + [f"- {sid}: {c}" for sid, c in checks]
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("runs", nargs="+", help="run folders (demo/out/<tag>)")
    ap.add_argument("--json", default=None, help="write the whole table here")
    ap.add_argument("--md", default=None, help="write the markdown table here")
    a = ap.parse_args(argv)
    runs, skipped = [], []
    for p in a.runs:
        try:
            runs.append(run_metrics(p))
        except ValueError as exc:
            skipped.append(str(exc))
    for s in skipped:
        print(f"skipped: {s}", file=sys.stderr)
    if not runs:
        print("no run folder with a metrics.json", file=sys.stderr)
        return 2
    table = scenario_table(runs)
    md = to_markdown(table)
    print(md, end="")
    if a.json:
        with open(a.json, "w", encoding="utf-8", newline="\n") as fh:
            json.dump(table, fh, indent=2)
            fh.write("\n")
    if a.md:
        with open(a.md, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(md)
    return 0


if __name__ == "__main__":
    sys.exit(main())
