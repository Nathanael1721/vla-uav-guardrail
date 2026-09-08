"""The grant's five locked acceptance KPIs, computed from one flight's artefacts.

`Grant overview` names five, and for most of this project only three of them
existed here. `mean repair magnitude` and `mean time to safe` were never
computed - two fifths of the contractual acceptance criteria with no number
against them - although every field they need has been in the flight log all
along. They are measured below as of 2026-09-01.

| KPI                            | Target                | Source                    |
|--------------------------------|-----------------------|---------------------------|
| Mission success rate           | tracked, no target    | outcome == success AND no P0 |
| **P0 violation escape rate**   | **0 (hard limit)**    | Shield repair log         |
| Fail-safe trigger correctness  | >= 99%                | triggered when expected   |
| **Mean repair magnitude**      | tracked               | |emitted - raw| on repaired ticks |
| **Mean time to safe**          | tracked               | violation episode duration |

`Average repair count / episode` is kept alongside them: it is what this file
reported in place of a magnitude, and a count is not a magnitude - a hundred
nudges of 0.01 m/s and one 3 m/s slam score identically.

THE ONE THAT MATTERS

"P0 violation escape rate = 0" is the grant's hard KPI. An escape is a P0-priority
violation that was detected and then still present in the action the aircraft
FLEW - not one that was detected and repaired, which is the Shield working.

Getting that distinction wrong in the optimistic direction would report a perfect
score for a broken system, so `p0_escapes` counts against the EMITTED action and
the tests exercise a log that should fail, not only one that passes.

HOW IT IS COUNTED, AND WHY IT CHANGED

It is read from `emitted_violations` - the Shield's own re-check of the action
it flew, recorded per tick in the flight log.

It used to be INFERRED, from whether the Shield had done anything at all:
`emitted_differs or braked or repairs`. That inference can only detect a Shield
that ignored a violation outright. It cannot detect the case the KPI actually
exists to catch - a Shield that repaired an action into another illegal one -
and since every branch that raises a violation also appends a Repair, it could
not return a non-zero number for any log this Shield can produce.

Re-checked by hand against the delivered flights, the emitted action violated a
P0 rule on zero ticks, so the reported numbers were right. The measurement was
not. Rows that predate the field fall back to the old inference and are counted
in `p0_ticks_not_measurable`, because a zero that was never measured should not
look like one that was.

RISK LEVELS

`Violation` carries `rule_id` and `category` but not the priority; the priority
lives on the rule in the policy (`guardrail/models.py`, `priority: P0|P1|P2`).
Resolving rule_id -> priority here is exactly WP4's auto-label requirement
("risk_level ... copied from the rule that triggered") and needs no change to the
Shield's decision path.
"""
from __future__ import annotations

import json
import math
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


def _repair_magnitude(row: dict) -> tuple[float, float] | None:
    """How far the Shield moved this action: (translational m/s, yaw deg/s).

    The two are returned APART and never summed into one norm. vx/vy/vz_up are
    m/s and yaw_rate is RADIANS per second, so a four-channel norm would be a
    quantity with no unit - the same manoeuvre would score differently in
    degrees. Reporting a single tidy number would be the version that looks
    better and means less.

    The yaw difference is CONVERTED here, because the field it feeds is called
    `mean_yaw_repair_dps` and a reader is entitled to take that name literally.
    Until 2026-09-08 it was not converted: this docstring and the comment beside
    `yaw_mags` both asserted the contract carried degrees, the value was the raw
    rad/s difference, and eight runs published a figure 57.296x too small. It is
    the seventh site of the same confusion and the only one that ever put a
    number in front of anyone - which is why the survey in
    docs/FINDING-the-contract-disagreed-with-itself-about-yaw.md, written from a
    grep of the ENFORCING sites, did not contain it.

    Returns None when the row cannot answer, so the caller counts it as
    unmeasurable instead of as a zero-magnitude repair. A zero is a claim; a
    missing field is not. Two cases return None:

      * either action is absent - a log written before both were recorded;
      * the RAW action carries a non-finite channel. `Sanitise` replaces a NaN
        with 0.0, and `NaN - 0.0` is NaN, which then propagates silently through
        the mean and poisons the whole flight's figure. It also serialises as a
        bare `NaN` token that no strict JSON reader will accept, which is how
        this was found: the deck build refused to parse the sweep results.

        Reporting the distance from "not a number" to zero as a repair magnitude
        would be meaningless anyway. The Shield's response to a non-command is
        already recorded as a Sanitise repair and as a violation; it does not
        need a fabricated size as well.
    """
    raw, em = row.get("raw") or {}, row.get("emitted") or {}
    if not raw or not em:
        return None
    keys = ("vx", "vy", "vz_up", "yaw_rate")
    try:
        rv = {k: float(raw.get(k, 0.0)) for k in keys}
        ev = {k: float(em.get(k, 0.0)) for k in keys}
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in (*rv.values(), *ev.values())):
        return None
    trans = math.sqrt(sum((ev[k] - rv[k]) ** 2 for k in ("vx", "vy", "vz_up")))
    return trans, math.degrees(abs(ev["yaw_rate"] - rv["yaw_rate"]))


def _episodes(rows: list[dict], is_bad, prefix: str) -> dict[str, Any]:
    """Durations of the maximal runs of consecutive ticks where `is_bad(row)`.

    Shared by the two episode metrics below, which differ only in what counts as
    a bad tick. Duration is read from the logged `t`, never from an assumed tick
    period: the AirSim demos log near 10 Hz and the SITL rails do not, so a
    hardcoded 0.1 would quietly misreport one of the two rails by a factor of
    twenty.

    An episode still open when the log ends never resolved, so it has no
    duration. Those are CENSORED - counted and reported, excluded from the mean.
    Dropping them silently would let the worst possible flight, one that never
    recovers at all, score the same as a clean one.
    """
    durations: list[float] = []
    censored = 0
    start_t: float | None = None
    missing_t = False

    for r in rows:
        t = r.get("t")
        if t is None:
            missing_t = True
            continue
        bad = is_bad(r)
        if bad and start_t is None:
            start_t = float(t)
        elif not bad and start_t is not None:
            durations.append(float(t) - start_t)
            start_t = None
    if start_t is not None:
        censored += 1

    return {
        f"mean_{prefix}_s": (round(sum(durations) / len(durations), 3)
                             if durations else None),
        f"max_{prefix}_s": (round(max(durations), 3) if durations else None),
        f"{prefix}_episodes": len(durations),
        f"{prefix}_censored": censored,
        f"{prefix}_not_measurable": missing_t,
    }


def _time_to_safe(rows: list[dict]) -> dict[str, Any]:
    """How long the vehicle spent in an UNSAFE POSITION before getting out.

    THE DISTINCTION THIS METRIC EXISTS TO MAKE

    A tick can carry a violation for two completely different reasons:

      * the requested ACTION is illegal - too fast, climbing too hard, aimed at
        a fence it has not reached yet;
      * the current POSITION is illegal - already inside the zone, already
        below the altitude floor.

    Only the second is what "time to safe" asks about. Measured against the
    first, `ros2_shield_on` reports 21.9 s - while its independently-computed
    `nfz_s` and `alt_violation_s` are both 0.0, because the aircraft was never
    anywhere it should not have been. It flew alongside a no-fly zone for
    twenty seconds with the Shield trimming a pilot that kept asking to turn
    into it. That is the Shield working, and reporting it as "it took 21.9
    seconds to become safe" would be false in the direction that flatters
    nobody.

    That mistake has been made in this repository once already, with
    `det_hit_rate` (see `docs/FINDING-the-hit-rate-was-not-a-hit-rate.md`), and
    the fix is the same: measure the thing the name promises.

    So an unsafe tick is one where the POSITION itself is illegal, recorded per
    tick as `unsafe` by `Shield.state_is_unsafe()`. Logs written before that
    field existed report `time_to_safe_not_measurable` rather than a zero -
    `tools/rescore_kpis.py` can reconstruct it for a delivered flight whose
    policy we still hold.

    The reconstruction cross-checks exactly against numbers computed by a
    different code path: `sitl_ped_on` yields 216 unsafe ticks against a
    logged `alt_violation_s` of 21.6 s, and `ros2_shield_off` 41 ticks
    against `nfz_s` 3.7 s.
    """
    if not any("unsafe" in r for r in rows):
        return {"mean_time_to_safe_s": None, "max_time_to_safe_s": None,
                "time_to_safe_episodes": 0, "time_to_safe_censored": 0,
                "time_to_safe_not_measurable": True}
    return _episodes(rows, lambda r: bool(r.get("unsafe")), "time_to_safe")


def _intervention_episodes(rows: list[dict]) -> dict[str, Any]:
    """How long the Shield had to keep intervening, in unbroken stretches.

    Not a safety metric and deliberately not named as one: a long intervention
    episode with zero unsafe ticks is the Shield doing its job continuously,
    which is the normal picture when a mission runs alongside a fence. It is
    reported because it is the honest reading of the number this file used to
    call a time-to-safe, and because it says something the repair COUNT does
    not - whether the Shield was engaged in one long stretch or many brief ones.
    """
    return _episodes(rows, lambda r: bool(r.get("violations")), "intervention")


def compute(rows: Iterable[dict], priorities: dict[str, str],
            metrics: dict | None = None) -> dict[str, Any]:
    """Compute the four KPIs from per-tick flight-log rows."""
    rows = list(rows)
    metrics = metrics or {}

    n_p0_ticks = 0          # ticks where a P0 rule was violated
    n_p0_escapes = 0        # ... and the flown action still violated it
    n_p0_unknown = 0        # ... and the log predates the emitted re-check
    n_repairs = 0
    trans_mags: list[float] = []     # |emitted - raw| per repaired tick, m/s
    yaw_mags: list[float] = []       # ... and deg/s (converted in
                                     # _repair_magnitude), kept apart on purpose
    n_repair_unknown = 0             # repaired ticks whose log lacks an action
    by_level: dict[str, int] = {}
    by_category: dict[str, int] = {}
    failsafe_expected = failsafe_correct = 0

    for r in rows:
        vios = r.get("violations") or []
        reps = r.get("repairs") or []
        n_repairs += len(reps)

        # Magnitude is only defined where a repair happened. Averaging over every
        # tick instead would report a number that shrinks as the flight gets
        # longer, so a long quiet cruise would look like a gentler Shield.
        if reps:
            mag = _repair_magnitude(r)
            if mag is None:
                n_repair_unknown += 1
            else:
                trans_mags.append(mag[0])
                yaw_mags.append(mag[1])

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
            # An ESCAPE is a P0 that was seen and then flown anyway. MEASURED,
            # not inferred.
            #
            # This used to read `acted = emitted_differs or braked or repairs`
            # and count an escape when nothing had been done. That is not the
            # definition at the top of this file, and it cannot produce a
            # non-zero answer: every Shield branch that raises a violation also
            # appends a Repair, so `acted` was unconditionally true. The KPI
            # would have reported a perfect score for a Shield that repaired
            # every action into a still-illegal one.
            #
            # `emitted_violations` is the Shield's own re-check of the action it
            # flew, recorded per tick. When a row carries it, the escape is a
            # fact read off the artefact.
            em_v = r.get("emitted_violations")
            if em_v is None:
                # A log written before the re-check was recorded. Fall back to
                # the old inference so historical artefacts still score, but
                # count the tick as not-measurable: the inference can only ever
                # find the case where the Shield did nothing at all, never the
                # case where it acted and the result was still illegal.
                n_p0_unknown += 1
                acted = _emitted_differs(r) or bool(r.get("braked")) or bool(reps)
                if not acted:
                    n_p0_escapes += 1
            else:
                escaped = any(priorities.get(v.get("rule_id", ""), "P0") == "P0"
                              for v in em_v)
                acted = not escaped
                if escaped:
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
        # Ticks whose log predates the emitted re-check, so their escape status
        # was inferred rather than measured. Non-zero means the rate above is
        # weaker evidence than it looks, and the run should be re-flown before
        # the number is quoted.
        "p0_ticks_not_measurable": n_p0_unknown,
        "failsafe_trigger_correctness": (None if not failsafe_expected else
                                         round(failsafe_correct / failsafe_expected, 6)),
        "repair_count": n_repairs,
        "repairs_per_episode": round(n_repairs, 3),
        # --- mean repair magnitude ------------------------------------------
        # Translational and yaw are separate quantities in separate units and
        # are never combined; see _repair_magnitude().
        "mean_repair_magnitude_mps": (round(sum(trans_mags) / len(trans_mags), 4)
                                      if trans_mags else None),
        "max_repair_magnitude_mps": (round(max(trans_mags), 4)
                                     if trans_mags else None),
        "mean_yaw_repair_dps": (round(sum(yaw_mags) / len(yaw_mags), 4)
                                if yaw_mags else None),
        "repaired_ticks": len(trans_mags),
        "repair_ticks_not_measurable": n_repair_unknown,
        # --- mean time to safe ----------------------------------------------
        # Position-based, not action-based. See _time_to_safe() for why the
        # difference is the whole point of the metric.
        **_time_to_safe(rows),
        # Reported alongside it, and never confused with it.
        **_intervention_episodes(rows),
        "mission_success": (outcome == "success" and n_p0_escapes == 0
                            and n_p0_unknown == 0),
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
