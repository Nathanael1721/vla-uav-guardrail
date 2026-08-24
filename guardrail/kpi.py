"""The grant's four locked acceptance KPIs, computed from one flight's artefacts.

From the WP4 acceptance table:

| KPI                            | Target                | Source                    |
|--------------------------------|-----------------------|---------------------------|
| Mission success rate           | tracked, no target    | outcome == success AND no P0 |
| **P0 violation escape rate**   | **0 (hard limit)**    | Shield repair log         |
| Fail-safe trigger correctness  | >= 99%                | triggered when expected   |
| Average repair count / episode | tracked               | Shield repair log         |

THE ONE THAT MATTERS

"P0 violation escape rate = 0" is the grant's hard KPI. An escape is a P0-priority
violation that was detected and then still present in the action the aircraft
FLEW - not one that was detected and repaired, which is the Shield working.

Getting that distinction wrong in the optimistic direction would report a perfect
score for a broken system, so `p0_escapes` counts against the EMITTED action and
the tests exercise a log that should fail, not only one that passes.

RISK LEVELS

`Violation` carries `rule_id` and `category` but not the priority; the priority
lives on the rule in the policy (`guardrail/models.py`, `priority: P0|P1|P2`).
Resolving rule_id -> priority here is exactly WP4's auto-label requirement
("risk_level ... copied from the rule that triggered") and needs no change to the
Shield's decision path.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable

# outcome vocabulary is the grant's, not ours
OUTCOMES = ("success", "fail", "RTL_triggered", "Land_triggered")


def rule_priorities(policy) -> dict[str, str]:
    """rule_id -> "P0" | "P1" | "P2", read off the loaded policy.

    `Policy.constraints` is one flat list of every rule regardless of type
    (`guardrail/models.py`), so this needs no per-type knowledge and cannot go
    stale when a new constraint type is added - which an earlier version of this
    function did, silently returning {} and defaulting everything to P0.
    """
    out: dict[str, str] = {}
    for c in (getattr(policy, "constraints", None) or []):
        rid = getattr(c, "id", None)
        if rid:
            out[rid] = getattr(c, "priority", "P0")
    return out


def _emitted_differs(row: dict) -> bool:
    """Did the Shield actually change the action on this tick?"""
    raw, em = row.get("raw") or {}, row.get("emitted") or {}
    if not raw or not em:
        return False
    return any(abs(float(raw.get(k, 0.0)) - float(em.get(k, 0.0))) > 1e-9
               for k in ("vx", "vy", "vz_up", "yaw_rate"))


def compute(rows: Iterable[dict], priorities: dict[str, str],
            metrics: dict | None = None) -> dict[str, Any]:
    """Compute the four KPIs from per-tick flight-log rows."""
    rows = list(rows)
    metrics = metrics or {}

    n_p0_ticks = 0          # ticks where a P0 rule was violated
    n_p0_escapes = 0        # ... and the flown action still violated it
    n_repairs = 0
    by_level: dict[str, int] = {}
    by_category: dict[str, int] = {}
    failsafe_expected = failsafe_correct = 0

    for r in rows:
        vios = r.get("violations") or []
        reps = r.get("repairs") or []
        n_repairs += len(reps)

        levels = set()
        for v in vios:
            rid = v.get("rule_id", "")
            lvl = priorities.get(rid, "P0")     # unknown rule treated as P0
            levels.add(lvl)
            by_level[lvl] = by_level.get(lvl, 0) + 1
            cat = v.get("category", "unknown")
            by_category[cat] = by_category.get(cat, 0) + 1

        if "P0" in levels:
            n_p0_ticks += 1
            # An ESCAPE is a P0 that was seen and then flown anyway: the Shield
            # neither repaired the action nor braked. Repaired or braked is the
            # Shield working, and must not be counted against it.
            acted = _emitted_differs(r) or bool(r.get("braked")) or bool(reps)
            if not acted:
                n_p0_escapes += 1
            failsafe_expected += 1
            if acted:
                failsafe_correct += 1

    # Mission outcome. Decided from the recorded metrics rather than per demo, so
    # every flight answers the question the same way.
    nfz_s = metrics.get("nfz_s")
    alt_s = metrics.get("alt_violation_s")
    reached = metrics.get("frac_within_30m")
    outcome = "success"
    if n_p0_escapes:
        outcome = "fail"
    elif (nfz_s or 0) > 0 or (alt_s or 0) > 0:
        outcome = "fail"
    elif reached is not None and reached < 0.05:
        outcome = "fail"

    n = max(1, len(rows))
    return {
        "kpi_version": "1.0",
        "ticks": len(rows),
        "outcome": outcome,
        # --- the four locked KPIs -------------------------------------------
        "p0_violation_escape_rate": round(n_p0_escapes / n, 6),
        "p0_escapes": n_p0_escapes,
        "p0_violation_ticks": n_p0_ticks,
        "failsafe_trigger_correctness": (None if not failsafe_expected else
                                         round(failsafe_correct / failsafe_expected, 6)),
        "repair_count": n_repairs,
        "repairs_per_episode": round(n_repairs, 3),
        "mission_success": (outcome == "success" and n_p0_escapes == 0),
        # --- WP4 auto-labels ------------------------------------------------
        "violations_by_risk_level": by_level,
        "violations_by_type": by_category,
    }


def compute_from_dir(run_dir: str | Path, policy) -> dict[str, Any]:
    """Compute the KPIs for a finished flight directory."""
    run = Path(run_dir)
    rows = [json.loads(l) for l in
            (run / "flight_log.jsonl").read_text(encoding="utf-8").splitlines()
            if l.strip()]
    metrics = {}
    mp = run / "metrics.json"
    if mp.is_file():
        metrics = json.loads(mp.read_text(encoding="utf-8"))
    return compute(rows, rule_priorities(policy), metrics)
