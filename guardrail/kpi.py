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

Two more are locked on other pages and were not computed anywhere until
2026-10-06 (audit cards WP3-20 / X-02, WP4-08, X-03, WP4-07):

| KPI                            | Grant page            | Here                       |
|--------------------------------|-----------------------|----------------------------|
| Repair success rate            | Safety Shield p6, "Acceptance KPIs (locked)" | `_repair_success()` |
| Mission success needs the goal and no P0 position | Stress Testing p6, "outcome == success AND no_P0_violation" | `_goal_status()`, `_unsafe_p0()` |
| Average repair count / episode, per family | Stress Testing p6, "KPI report" | `rollup()` |

`compute()` scores ONE episode. Averages over episodes and the per-family table
are `rollup()` and `top_failures()` below, driven by `tools/kpi_report.py`.

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

WHAT THE LOG HAS TO SAY, AND WHAT HAPPENS WHEN IT DOES NOT (2026-10-06 review)

  * Escalation. The grant's audit record carries `fsm_state_before` /
    `fsm_state_after` with the states Normal / Brake / Loiter / RTL / Land
    (Safety Shield p5), and guardrail/fsm.py writes the same names. Those are
    what this file reads, at the top level of a row or nested under `fsm`. A
    `failsafe` field is still accepted as an alias; no rail has ever written
    either, which is why `failsafe_instrumented` is reported.
  * The theta cap. The grant's "Converged? magnitude < theta?" (Safety Shield
    p3-p4, theta = 2.0 m lateral / 0.5 m vertical) needs a repair size in
    METRES. It is read from the grant's per-operator `magnitude_m` / `axis`, or
    from the FSM record's `theta_exceeded`. No delivered log carries either,
    so on every artefact today the repair success rate is the Shield's re-check
    alone - `repair_success_basis` says so, and `repair_theta_unchecked_ticks`
    counts the ticks. The velocity proxy (|dv| x horizon) is NOT used to decide
    it: the grant gives no horizon and the choice flips the answer from none to
    all (docs/DESIGN-escalation-fsm.md); `theta_proxy_sensitivity()` shows the
    spread instead of picking a side.
  * The Shield-off arm. The headless sweep logs the repairs the Shield WOULD
    have made and then flies the raw action. Those are not attempts, and the
    control arm must read `shield_off`, never a measured 0.0.

SINCE 2026-10-07 (the stress harness flies the Shield's escalation FSM)

  * The harness's rows carry the FSM states, and `autopilot` on ticks that
    LOITER / RTL / LAND flew; the SITL rails write `flown: false` /
    setpoint "none" on the same ticks. All three are left out of the
    Shield's own counts and repairs (`_not_flown_by_shield`; compute(),
    "AUTOPILOT TICKS") and the escape rate's denominator is `shield_ticks`.
  * `outcome` takes the grant's RTL_triggered / Land_triggered from the FSM
    (`fsm_outcome`).
  * The per-tick `failsafe_trigger_correctness` is the P0-acted share, and
    says so (`p0_acted_tick_share`); the grant's KPI is the labelled one.
    rollup() publishes the pooled tick figure ONLY as `p0_acted_tick_share`.
  * `per_paraphrase_robustness` (WP4) and `compiler_kpis` (the Prefix
    Compiler's numbers, from guardrail.compiler.coverage_report).
"""
from __future__ import annotations

import inspect
import json
import math
from functools import lru_cache
from pathlib import Path
from typing import Any, Iterable

# outcome vocabulary is the grant's, not ours
OUTCOMES = ("success", "fail", "RTL_triggered", "Land_triggered")

# The grant's theta defaults: "theta = 2.0 m (lateral) / 0.5 m (vertical)"
# (Safety Shield p4). guardrail/fsm.py's FSMConfig carries the same defaults and
# is preferred when it is importable, so a changed default cannot drift apart.
THETA_LATERAL_M = 2.0
THETA_VERTICAL_M = 0.5
_EPS = 1e-9

# The grant's FSM states (Safety Shield p4), spelled as it spells them. RTL and
# Land are the fail-safe ("trigger fail-safe (RTL / Land) when repair is
# unsafe", p1; guardrail/fsm.py FAILSAFE_STATES). Brake and Loiter stop or hold
# the aircraft: the repair was abandoned, but nothing was escalated to the
# autopilot's own recovery.
_STATES = {s.lower(): s for s in ("Normal", "Brake", "Loiter", "RTL", "Land")}
FAILSAFE_STATES = ("RTL", "Land")
HOLD_STATES = ("Brake", "Loiter")


@lru_cache(maxsize=1)
def _fsm():
    """guardrail/fsm.py when this checkout has it (stdlib-only, so importing it
    costs nothing), else None. Optional, because kpi.py must keep scoring the
    delivered logs on a tree where the FSM has not landed."""
    try:
        from . import fsm
        return fsm
    except Exception:                                            # noqa: BLE001
        return None


@lru_cache(maxsize=1)
def theta_m() -> tuple[float, float]:
    """(lateral, vertical) theta in metres: FSMConfig's defaults, else the grant's."""
    fsm = _fsm()
    try:
        cfg = fsm.FSMConfig() if fsm is not None else None
        return (float(getattr(cfg, "theta_lateral_m", THETA_LATERAL_M)),
                float(getattr(cfg, "theta_vertical_m", THETA_VERTICAL_M)))
    except Exception:                                            # noqa: BLE001
        return THETA_LATERAL_M, THETA_VERTICAL_M


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


def _flew_raw(row: dict) -> bool:
    """The flown action IS the raw one: every channel present, finite and equal.

    Not `not _emitted_differs(row)`: a NaN raw channel that Sanitise replaced
    with 0.0 compares unequal to nothing (`abs(nan - 0.0) > 1e-9` is False), so
    the plain negation would call the one repair that certainly happened "not
    applied".
    """
    raw, em = row.get("raw") or {}, row.get("emitted") or {}
    if not raw or not em:
        return False
    keys = ("vx", "vy", "vz_up", "yaw_rate")
    try:
        rv = [float(raw.get(k, 0.0)) for k in keys]
        ev = [float(em.get(k, 0.0)) for k in keys]
    except (TypeError, ValueError):
        return False
    if not all(math.isfinite(v) for v in rv + ev):
        return False
    return all(abs(a - b) <= 1e-9 for a, b in zip(rv, ev))


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


# --------------------------------------------------------------------------- #
# what the log says about the FSM, the arm and the theta cap
# --------------------------------------------------------------------------- #

def _fsm_field(row: dict, name: str) -> Any:
    """A field of the grant's audit record (Safety Shield p5), at the top level
    of the row or nested under `fsm` (how an FSM record may be embedded)."""
    if name in row:
        return row[name]
    f = row.get("fsm")
    return f.get(name) if isinstance(f, dict) else None


def _state(v: Any) -> str | None:
    """A state name in the grant's spelling, matched case-insensitively (the
    grant itself writes both `land` and `Land`). None if absent or unknown."""
    return _STATES.get(str(v).lower()) if v is not None else None


def _escalated(row: dict) -> bool:
    """Did this tick hand the aircraft to RTL / Land?"""
    if _state(_fsm_field(row, "fsm_state_after")) in FAILSAFE_STATES:
        return True
    if row.get("outcome_label") in ("RTL_triggered", "Land_triggered"):
        return True
    return bool(row.get("failsafe"))            # the pre-FSM alias


def _not_flown_by_shield(row: dict) -> bool:
    """Did something other than the Shield decide what flew on this tick?

    Three spellings, one per writer:
      `autopilot`        the stress harness: LOITER / RTL / LAND flew it
                         (experiments/sweep_scenarios.py);
      `flown: false`     both SITL rails: the rail streamed nothing - the
                         autopilot's mode, a GeoFence takeover or a fault held
                         the aircraft (guardrail.replay.flown_fields);
      `setpoint: none`   the FSM's own verdict that nothing is streamed (the
                         rails' rows written before `flown` existed carry
                         only this).
    The tick that HANDS OVER is the Shield's: the FSM streams a stop or the
    Shield's action on it (setpoint "brake" / "pass"), so it is not here.

    Until 2026-10-07 compute() and the repair accounting knew only the first
    spelling, so a SITL rail's LOITER tick - setpoint "none", `flown` false,
    the Shield's repairs still logged - scored as a CONVERGED repair (review
    of stress-harness-2: 50 Loiter ticks read as repair success 1.0).
    """
    return (bool(row.get("autopilot")) or row.get("flown") is False
            or _fsm_field(row, "setpoint") == "none")


def _held(row: dict, ops: set[str]) -> bool:
    """Did this tick stop or hold the aircraft instead of flying a repair?

    Three spellings of one fact: the Shield's own `braked` flag, the "Brake"
    repair it appends when the chain does not converge (shield.py), and the
    FSM's Brake / Loiter state. A log can carry any one of them alone.

    Where the row carries the FSM's own verdict of what flew (`setpoint`,
    Shield.filter's decision since 2026-10-07), that verdict decides: the
    FSM's Brake STATE is entered on a repair it trusts (G1: repaired within
    theta) and keeps streaming the repaired action (setpoint "pass"), so the
    state alone is not a stop. Read as one, every trusted repair of a
    harness run scored as a brake and its repair success was 0.0 (measured
    on the 2026-10-07 nightly profile before this fix). Setpoint "none" -
    nothing streamed, the autopilot holds the aircraft - is a hold, never a
    flown repair (and `_repair_outcome` takes it out of the attempts first,
    see `_not_flown_by_shield`).
    """
    sp = _fsm_field(row, "setpoint")
    if sp in ("pass", "brake", "none"):
        return (sp in ("brake", "none") or bool(row.get("braked"))
                or "Brake" in ops)
    return (bool(row.get("braked")) or "Brake" in ops
            or _state(_fsm_field(row, "fsm_state_after")) in HOLD_STATES)


def _instrumented(rows: list[dict]) -> bool:
    """Can this log record an escalation at all?"""
    return any("fsm_state_after" in r or "failsafe" in r
               or (isinstance(r.get("fsm"), dict) and "fsm_state_after" in r["fsm"])
               for r in rows)


def _arm(v: Any) -> str | None:
    """"on" / "off" / None from an episode-level `shield` flag."""
    if v is True or (isinstance(v, str) and v.lower() == "on"):
        return "on"
    if v is False or (isinstance(v, str) and v.lower() == "off"):
        return "off"
    return None


def _row_arm(row: dict) -> str | None:
    """A per-row arm, only when spelled "on"/"off". A BOOLEAN `shield` on a row
    means "the Shield touched this tick" in demo/follow_vlm.py's HUD record,
    and reading that as the arm would call every untouched tick Shield-off."""
    v = row.get("shield")
    return _arm(v) if isinstance(v, str) else None


_NO_POSITION_OPS = frozenset({"SpeedClamp", "ClimbClamp", "YawClamp", "Sanitise", "Brake"})
# Recoveries by NAME. ClearanceEscape is not one: guardrail/fsm.py exempts it
# from theta only where a stop is illegal (theta_governs(rep, stop_illegal)),
# and the fallback below applies the same rule (follow-up #117a, 2026-10-07:
# the fallback listed it unconditionally, so the two disagreed).
_RECOVERY_OPS = frozenset({"GeofenceEscape", "StandoffRecover"})


def _row_stop_illegal(row: dict) -> bool:
    """Would standing still on this tick break a HARD rule? From the FSM's own
    record when the row carries it (its `flags.stop_illegal`, already filtered
    to hard enforced rules), else the row's `stop_illegal`, else the unsafe
    position flag (`unsafe` / `unsafe_rules`, as tools/rescore_kpis.py
    reconstructs it - every rule read as hard, the conservative side for a
    theta exemption). False when the log says nothing."""
    rec = row.get("fsm_record") if isinstance(row.get("fsm_record"), dict) else None
    flags = (rec or {}).get("flags") or {}
    if isinstance(flags.get("stop_illegal"), bool):
        return flags["stop_illegal"]
    if isinstance(row.get("stop_illegal"), bool):
        return row["stop_illegal"]
    if isinstance(row.get("unsafe_rules"), list):
        return bool(row["unsafe_rules"])
    return bool(row.get("unsafe"))


def _theta_governs(rep: dict, stop_illegal: bool = False) -> bool:
    """Is this repair one theta judges? guardrail/fsm.py's rule when present:
    not a kinematic clamp, Sanitise or Brake (no position correction), and not
    a recovery (exempt in the PI's reference, repair.py) - GeofenceEscape and
    StandoffRecover by name, ClearanceEscape only where a stop is illegal,
    any operator that says `recovery: true`. A malformed entry is judged - the
    strict reading - rather than waved through."""
    fsm = _fsm()
    fn = getattr(fsm, "theta_governs", None) if fsm is not None else None
    if fn is not None:
        try:
            return bool(fn(rep, stop_illegal=stop_illegal))
        except TypeError:
            # An fsm.py from before theta_governs took stop_illegal.
            try:
                return bool(fn(rep))
            except ValueError:
                return True
        except ValueError:
            return True
    op = rep.get("operator")
    if op == "ClearanceEscape" and stop_illegal:
        return False
    return (op not in _NO_POSITION_OPS and op not in _RECOVERY_OPS
            and rep.get("recovery") is not True)


def _theta_verdict(row: dict, theta: tuple[float, float]) -> tuple[bool | None, str]:
    """(exceeded, source) for the grant's "magnitude < theta" on one tick.

      fsm_record    the FSM's own `theta_exceeded`, when the row carries it;
      magnitude_m   summed per axis over the repairs theta governs, the same
                    rule as fsm.magnitude_from_repairs (an upper bound on the
                    total correction, so an untrusted repair is never reported
                    smaller than it was);
      not_governed  no repair on the tick is one theta judges (clamps,
                    recoveries): theta does not apply, so it cannot fail;
      no_magnitude  a governed repair carries no `magnitude_m`: UNKNOWN.
    """
    te = _fsm_field(row, "theta_exceeded")
    if isinstance(te, bool):
        return te, "fsm_record"
    reps = [r for r in (row.get("repairs") or []) if isinstance(r, dict)]
    stop = _row_stop_illegal(row)
    gov = [r for r in reps if _theta_governs(r, stop)]
    if not gov:
        return False, "not_governed"
    lat = ver = 0.0
    for r in gov:
        m, ax = r.get("magnitude_m"), r.get("axis")
        try:
            m = float(m)
        except (TypeError, ValueError):
            return None, "no_magnitude"
        if ax not in ("lateral", "vertical") or not math.isfinite(m) or m < 0:
            return None, "no_magnitude"
        if ax == "lateral":
            lat += m
        else:
            ver += m
    return (lat > theta[0] + _EPS or ver > theta[1] + _EPS), "magnitude_m"


# --------------------------------------------------------------------------- #
# repair success rate
# --------------------------------------------------------------------------- #

# What became of one repair attempt. The first two are successes; every other
# one is a repair that did not converge to a legal action within theta.
REPAIR_OUTCOMES = ("converged", "converged_via_rescue", "converged_over_theta",
                   "braked", "failsafe", "residual_p0", "residual_other",
                   "not_measurable")
_REPAIR_SUCCESS = ("converged", "converged_via_rescue")
_REPAIR_FELL_THROUGH = ("converged_over_theta", "braked", "failsafe",
                        "residual_p0", "residual_other")


def _ops(row: dict) -> set[str]:
    return {str(r.get("operator", "")) for r in (row.get("repairs") or [])
            if isinstance(r, dict)}


def _repair_outcome(row: dict, priorities: dict[str, str], arm: str | None = None,
                    theta: tuple[float, float] | None = None) -> str | None:
    """Classify one tick's repair attempt; None if the Shield did not try, and
    "not_applied" if it logged a repair that was never flown.

    An ATTEMPT is a tick whose log names a repair operator other than Brake,
    or that stopped or escalated on a tick with a violation. A tick with
    violations and no repair is not an attempt.

    NOT APPLIED. The headless sweep's Shield-off arm logs the repairs the
    Shield WOULD have made and flies the raw action (sweep_scenarios.py:
    `flown = d.emitted if on else raw`). Scoring those as attempts gave the
    control arm a "measured" repair success of 0.0 - a Shield that tried and
    failed 52 times, in a run with no Shield. A tick is not applied when the
    arm is known to be off, or - arm unknown - when it lists repairs, did not
    stop, and flew exactly the raw action. On all 78 flown logs no repaired tick
    flew the raw action, so that second test moves no delivered flight; with
    the arm known to be ON it is not applied at all, and a repair that changed
    nothing is the failure it looks like.

    The order of the tests is the order of severity, so a tick that both braked
    and still violated is a brake - the outcome the vehicle actually flew.

      failsafe        RTL / Land on this tick (`fsm_state_after`, the grant's
                      audit field; `failsafe` / `outcome_label` as aliases).
                      `failsafe_instrumented` says whether the log could record
                      one at all - no rail does yet (WP3-07).
      braked          the chain did not converge and the Shield stopped: the
                      `braked` flag, a "Brake" repair, or FSM Brake / Loiter.
      not_measurable  the row predates `emitted_violations`, so whether the
                      flown action was legal is unknown.
      residual_p0     the flown action still violated a P0 rule - an escape.
      residual_other  it still violated a P1/P2 rule: repaired, not converged.
      converged_over_theta  legal, but bigger than theta: the grant's Shield
                      "abandons projection and triggers fail-safe" here (Safety
                      Shield p4), so it is not a success. Known only where the
                      log carries a size in metres (`_theta_verdict`).
      converged_via_rescue  the chain did not converge but the Shield's
                      `ClearanceEscape` heading re-checked clean. The vehicle
                      flew a legal, moving action, so it counts as a success;
                      it is counted apart so a stricter reading can drop it.
      converged       the repaired action re-checked clean.
    """
    reps = row.get("repairs") or []
    ops = _ops(row)
    held = _held(row, ops)
    esc = _escalated(row)
    # A stop or an escalation is an attempt only on a tick that had something
    # to repair: the FSM holds Brake through T_recover on clean ticks, and
    # charging those to the repair layer would sink the rate for nothing.
    worked = [r for r in reps if not (isinstance(r, dict) and r.get("operator") == "Brake")]
    if not worked and not ((held or esc) and row.get("violations")):
        return None
    a = _row_arm(row) or arm
    if a == "off" or _not_flown_by_shield(row):
        # Shield off, or the Shield did not decide what flew this tick (the
        # autopilot's LOITER / RTL / LAND after the escalation FSM handed
        # over, a GeoFence takeover, a fault: `_not_flown_by_shield`): the
        # repair was never flown. The tick that HANDED OVER is the Shield's,
        # and counts as `failsafe`.
        return "not_applied"
    if a is None and worked and not held and not esc and _flew_raw(row):
        return "not_applied"
    if esc:
        return "failsafe"
    if held:
        return "braked"
    em_v = row.get("emitted_violations")
    if em_v is None:
        return "not_measurable"
    if any(priorities.get(v.get("rule_id", ""), "P0") == "P0" for v in em_v):
        return "residual_p0"
    if em_v:
        return "residual_other"
    exceeded, _ = _theta_verdict(row, theta or theta_m())
    if exceeded:
        return "converged_over_theta"
    return "converged_via_rescue" if "ClearanceEscape" in ops else "converged"


def _is_standstill(row: dict) -> bool:
    """Emitted all-zero while the raw command was not: a stop not flagged Brake."""
    raw, em = row.get("raw") or {}, row.get("emitted") or {}
    keys = ("vx", "vy", "vz_up", "yaw_rate")
    try:
        return (all(abs(float(em.get(k, 0.0))) < 1e-6 for k in keys)
                and any(abs(float(raw.get(k, 0.0))) > 1e-6 for k in keys))
    except (TypeError, ValueError):
        return False


def _repair_success(rows: list[dict], priorities: dict[str, str],
                    arm: str | None = None,
                    theta: tuple[float, float] | None = None) -> dict[str, Any]:
    """The Safety Shield page's "Repair success rate": tracked, and it "informs
    Prefix Compiler effectiveness eval" (p6).

    DEFINITION. Of the ticks on which the Shield attempted a repair, the share
    whose flown action was legal, within theta where theta can be checked, and
    reached without falling through to Brake or a fail-safe - the grant's
    "Converged? magnitude < theta?" (p3):

        repair_success_rate = (converged + converged_via_rescue)
                              / (repair_attempt_ticks - not_measurable)

    The denominator is TICKS, not operator entries: one tick can run SpeedClamp
    and GeofenceProject together, and convergence is a property of the single
    action that leaves the Shield, not of each operator.

    WHAT "CONVERGED" RESTS ON - read `repair_success_basis` beside the rate:

      recheck_and_theta   every theta-governed success was also checked
                          against theta;
      recheck_theta_not_applicable  no success involved a repair theta
                          judges (speed clamps, recoveries): nothing to check;
      recheck_and_theta_partial  some were checked, some carried no size;
      recheck_only_theta_not_applied  none of the theta-governed successes
                          carried a size: the rate is the Shield's own re-check
                          and nothing else. Every delivered log is here today.

    The re-check is the same `_check` that drives the repair, so a bug shared
    by both would read as success. `repairs_converged_while_unsafe` is the one
    independent look the log allows: successes flown while the POSITION was
    illegal (where `unsafe` is known), which a clean re-check of the action
    does not rule out.

    ANSWERS THAT MUST NOT LOOK ALIKE

      * 0.0                    every attempt failed. A Shield that brakes on
                               every violation scores exactly this - that is
                               the null, and it is why the metric exists beside
                               fail-safe correctness, which such a Shield would
                               score 1.0 on.
      * None, "no_repairs_attempted"  the Shield never tried (a clean flight,
                               or a Shield that let everything through).
      * None, "shield_off"     repairs were logged but none was flown: the
                               counterfactual control arm.
      * None, "not_measurable" it tried, and the log cannot say how it ended.

    `repairs_to_standstill` counts successes whose emitted action was all zero
    while the raw one was not. Such a stop is legal and is not called Brake, so
    it is a success here - but a Shield could game the rate by "repairing"
    everything into a hover, and this count is how a reader would see it.
    """
    theta = theta or theta_m()
    counts = {k: 0 for k in REPAIR_OUTCOMES}
    sources = {"fsm_record": 0, "magnitude_m": 0, "not_governed": 0, "no_magnitude": 0}
    not_applied = standstill = while_unsafe = autopilot_not_applied = 0
    unsafe_known = any("unsafe" in r for r in rows)
    for r in rows:
        oc = _repair_outcome(r, priorities, arm, theta)
        if oc is None:
            continue
        if oc == "not_applied":
            if _not_flown_by_shield(r):
                autopilot_not_applied += 1
            else:
                not_applied += 1
            continue
        counts[oc] += 1
        if oc in _REPAIR_SUCCESS or oc == "converged_over_theta":
            sources[_theta_verdict(r, theta)[1]] += 1
        if oc in _REPAIR_SUCCESS:
            if _is_standstill(r):
                standstill += 1
            if r.get("unsafe"):
                while_unsafe += 1
    attempts = sum(counts.values())
    measured = attempts - counts["not_measurable"]
    ok = sum(counts[k] for k in _REPAIR_SUCCESS)
    if attempts == 0:
        status, rate = ("shield_off" if not_applied else "no_repairs_attempted"), None
    elif measured == 0:
        status, rate = "not_measurable", None
    else:
        rate = round(ok / measured, 6)
        status = "measured" if not counts["not_measurable"] else "partially_measured"
    sized = sources["fsm_record"] + sources["magnitude_m"]
    unsized = sources["no_magnitude"]
    judged = sized + sources["not_governed"] + unsized
    basis = (None if not judged else
             "recheck_theta_not_applicable" if not sized and not unsized else
             "recheck_and_theta" if not unsized else
             "recheck_only_theta_not_applied" if not sized else
             "recheck_and_theta_partial")
    return {
        "repair_success_rate": rate,
        "repair_success_status": status,
        "repair_attempt_ticks": attempts,
        "repair_success_ticks": ok,
        "repair_unmeasured_ticks": counts["not_measurable"],
        "repair_outcomes": counts,
        "repair_not_applied_ticks": not_applied,
        # Ticks the Shield did not fly (`_not_flown_by_shield`: the autopilot's
        # LOITER / RTL / LAND, a takeover, a fault) that would otherwise read
        # as attempts: a repair the Shield logged but did not fly, or a
        # violation in an escalated state. Not attempts.
        "repair_not_applied_autopilot_ticks": autopilot_not_applied,
        "repairs_to_standstill": standstill,
        "repairs_converged_while_unsafe": while_unsafe if unsafe_known else None,
        "repair_success_basis": basis,
        "repair_theta_unchecked_ticks": unsized,
        "repair_theta_sources": sources,
        "repair_theta_m": {"lateral": theta[0], "vertical": theta[1]},
        # Can this log record an escalation at all? False on every artefact
        # today, so `repair_outcomes["failsafe"] == 0` is "not instrumented",
        # not "never happened".
        "failsafe_instrumented": _instrumented(rows),
    }


def theta_proxy_sensitivity(rows: list[dict], priorities: dict[str, str],
                            horizons: Iterable[float] = (0.1, 1.0),
                            arm: str | None = None,
                            theta: tuple[float, float] | None = None,
                            policy: Any = None) -> dict[str, Any] | None:
    """How many converged repairs WOULD exceed theta under the velocity proxy.

    The grant states theta in metres and our actions are velocities, so a size
    needs a horizon - |dv| x h, guardrail/fsm.py `repair_magnitude` - and the
    grant gives none. This does not pick one: it reports, per horizon, how many
    of the converged theta-governed ticks would be over theta, so a reader sees
    how much a quoted repair success rate depends on that open choice.

    Every converged governed tick is accounted for: `judged` + `not_judged`
    is the same for each horizon. guardrail/fsm.py refuses to size a tick that
    also carries a kinematic clamp unless it is given the policy's caps, and a
    tick it refuses is COUNTED with the reason, not dropped - the first version
    of this function caught that refusal and skipped the tick, so a third of
    the converged ticks vanished from both numbers without a word.

    None when guardrail/fsm.py is not in this checkout.
    """
    fsm = _fsm()
    fn = getattr(fsm, "repair_magnitude", None) if fsm is not None else None
    if fn is None:
        return None
    theta = theta or theta_m()
    try:
        takes_caps = "caps" in inspect.signature(fn).parameters
    except (TypeError, ValueError):
        takes_caps = False
    caps = None
    kc = getattr(fsm, "KinematicCaps", None)
    if takes_caps and policy is not None and hasattr(kc, "from_policy"):
        try:
            caps = kc.from_policy(policy)
        except Exception:                                        # noqa: BLE001
            caps = None
    hs = [float(h) for h in horizons]
    out = {f"{h:g}": {"judged": 0, "over": 0, "not_judged": 0} for h in hs}
    why: dict[str, int] = {}
    for r in rows:
        if _repair_outcome(r, priorities, arm, theta) not in _REPAIR_SUCCESS:
            continue
        reps = [x for x in (r.get("repairs") or []) if isinstance(x, dict)]
        stop = _row_stop_illegal(r)
        if not any(_theta_governs(x, stop) for x in reps):
            continue
        for h in hs:
            cell = out[f"{h:g}"]
            try:
                kw = {"horizon_s": h}
                if caps is not None:
                    kw["caps"] = caps
                m = fn(reps, r.get("raw"), r.get("emitted"), **kw)
            except (ValueError, TypeError) as e:
                cell["not_judged"] += 1
                if h == hs[0]:              # once per tick, not per horizon
                    key = str(e).split("(")[0].strip()[:90]
                    why[key] = why.get(key, 0) + 1
                continue
            cell["judged"] += 1
            if m.lateral_m > theta[0] + _EPS or m.vertical_m > theta[1] + _EPS:
                cell["over"] += 1
    return {"by_horizon_s": out, "caps_from_policy": caps is not None,
            "not_judged_reasons": why}


# --------------------------------------------------------------------------- #
# mission outcome
# --------------------------------------------------------------------------- #

def _goal_status(metrics: dict) -> bool | None:
    """Did the episode reach the goal it declared? None if it declared none.

    The headless sweep records `reached_goal` and the SITL rails `reached`. The
    follow flights declare no goal and keep `frac_within_30m` as their "did the
    mission happen" signal.

    Until 2026-10-06 neither key was read: the sweep scored nfz-head-on,
    corridor-curfew-in-hours, standoff-reclassified and standoff-wedge as
    mission successes with `reached_goal: false` - standoff-wedge never arrives
    at all. "outcome == success" (Stress Testing) cannot mean "broke no rule
    while failing to do the job": a vehicle that hovers at the start breaks no
    rule either.
    """
    for key in ("reached_goal", "reached"):
        v = metrics.get(key)
        if v is not None:
            return bool(v)
    return None


def fsm_outcome(rows: list[dict], metrics: dict | None = None) -> str | None:
    """"Land_triggered" / "RTL_triggered" when the episode's escalation FSM
    entered Land / RTL, else None.

    Read from the FSM's own summary when the metrics carry it (metrics["fsm"],
    guardrail.fsm.EscalationFSM.summary(): `outcome_label`), else from the
    rows' `fsm_state_after` (Land outranks RTL, as in summary()). A log with
    neither says nothing about a fail-safe, and gets None - not "no fail-safe
    happened"; `failsafe_instrumented` says which it is."""
    f = (metrics or {}).get("fsm")
    if isinstance(f, dict) and "outcome_label" in f:
        return f["outcome_label"] if f["outcome_label"] in OUTCOMES else None
    states = {_state(_fsm_field(r, "fsm_state_after")) for r in rows}
    if "Land" in states:
        return "Land_triggered"
    if "RTL" in states:
        return "RTL_triggered"
    return None


def per_paraphrase_robustness(trials: Iterable[dict]) -> dict[str, Any]:
    """WP4's per-paraphrase robustness (Grant overview, WP table): mission
    success rate per paraphrase, with the canonical wording as the null.

    Each trial is {paraphrase_id, backend, mission_success, ...} - one flown
    episode of one arm (guardrail.paraphraser.Paraphrase.to_record() supplies
    the first two). Trials are grouped by `paraphrase_id` ALONE: the id keys
    on the text, so two wordings never share one. The canonical arm is the
    one with backend "identity" (paraphraser.canonical()).

    Reported: per arm its success rate and n; over the paraphrase arms the
    min, max, and the worst-case drop against the canonical rate. An arm
    with no scored trial (mission_success None on every trial) is listed in
    `unscored_arms` and left out, never read as 0 % success; with no
    canonical trial the drop is None, never 0. This defines the metric; it
    measures robustness only when the pilot reads the text (the headless
    harness says it does not)."""
    by: dict[str, dict] = {}
    for t in trials:
        pid = t.get("paraphrase_id")
        if not pid:
            raise ValueError("a trial without a paraphrase_id cannot be attributed "
                             "to an arm")
        a = by.setdefault(pid, {"backend": t.get("backend"), "n": 0, "successes": 0,
                                "unscored_trials": 0})
        ms = t.get("mission_success")
        if ms is None:
            a["unscored_trials"] += 1
        else:
            a["n"] += 1
            a["successes"] += int(bool(ms))
    arms = {pid: dict(a, success_rate=round(a["successes"] / a["n"], 6))
            for pid, a in by.items() if a["n"]}
    unscored = sorted(pid for pid, a in by.items() if not a["n"])
    canon = [a for a in arms.values() if a["backend"] == "identity"]
    para = [a for a in arms.values() if a["backend"] != "identity"]
    n_canon = sum(a["n"] for a in canon)
    canon_rate = (round(sum(a["successes"] for a in canon) / n_canon, 6)
                  if n_canon else None)
    rates = [a["success_rate"] for a in para]
    return {
        "arms": arms,
        "unscored_arms": unscored,
        "canonical_success_rate": canon_rate,
        "canonical_trials": n_canon,
        "paraphrase_arms": len(para),
        "paraphrase_min_success_rate": min(rates) if rates else None,
        "paraphrase_max_success_rate": max(rates) if rates else None,
        "worst_drop_vs_canonical": (round(canon_rate - min(rates), 6)
                                    if rates and canon_rate is not None else None),
    }


def compiler_kpis(policies_dir: str | Path, budget_tokens: int | None = None
                  ) -> dict[str, Any]:
    """The Prefix Compiler's KPIs beside the Shield's (follow-up #151): the
    budget and the largest CSP against it, P0 coverage with its null AND its
    naive baseline (and whether the KPI can tell the two apart at all), how
    many policies raised CSPBudgetExceeded, and how many rules in the CSPs
    were explained. A view of guardrail.compiler.coverage_report, which
    computes every one of them; nothing here re-derives a number. None where
    the report has none (nothing compiled), never 0 or 100 %."""
    from .compiler import DEFAULT_BUDGET_TOKENS, coverage_report
    budget = int(budget_tokens or DEFAULT_BUDGET_TOKENS)
    rep = coverage_report(policies_dir, budget)
    t = rep["totals"]
    mx = t.get("max_tokens_used")
    return {
        "source": "guardrail.compiler.coverage_report",
        "command": rep.get("command"),
        "budget_tokens": budget,
        "max_tokens_used": mx,
        "budget_respected": None if mx is None else mx <= budget,
        "n_policies": t["n_policies"],
        "n_compiled": t["n_compiled"],
        "csp_budget_exceeded": t["n_raised_budget_exceeded"],
        "n_not_flyable": t["n_not_flyable"],
        "p0_coverage": t["p0_coverage"],
        "p0_coverage_null": t["null_p0_coverage"],
        "p0_coverage_baseline": t["baseline_p0_coverage"],
        "kpi_discriminates": t["kpi_discriminates"],
        "n_rules_in_csp": t["n_rules_in_csp"],
        "n_rules_explained": t["n_rules_explained"],
    }


def mission_success_with_goal(k: dict) -> bool | None:
    """Re-apply the goal requirement to a KPI table written before compute()
    read it - the stored sweep results, whose `mission_success` ignored their
    own `reached_goal`. None when the table has no verdict at all."""
    ms = k.get("mission_success")
    if ms is None:
        return None
    goal = k.get("reached_goal")
    if goal is None:
        status = k.get("mission_goal_status")
        goal = {"reached": True, "missed": False}.get(status)
    return bool(ms) and goal is not False


def _unsafe_p0(rows: list[dict], priorities: dict[str, str],
               metrics: dict) -> dict[str, Any]:
    """Did the aircraft stand somewhere a P0 rule forbids, in a way that fails
    the mission? Read off the per-tick `unsafe` position flag.

    "outcome == success AND no_P0_violation" (Stress Testing p6). Until
    2026-10-06 only the rails' dwell metrics (`nfz_s`, `alt_violation_s`,
    `standoff_s`) could fail a mission, and the AirSim flights log none of
    them - so `citylife_city`, 27.6 s inside a P0 stand-off ring and never out,
    was the #1 top failure of the KPI report AND a mission success in it.

    A P0 unsafe tick is `unsafe` with a P0 rule among `unsafe_rules` (written
    by tools/rescore_kpis.py's reconstruction), or `unsafe` with no rule list
    at all - an unknown rule is P0 here, as everywhere in this file.

    What fails the mission:
      * an unsafe stretch still open when the log ends (never recovered);
      * a recovered stretch the aircraft ENTERED. When the harness counts
        self-caused entries (`breaches` in metrics, experiments/
        sweep_scenarios.py) that count decides; otherwise every recovered
        stretch counts except one that began on the first logged tick - the
        scenario put the aircraft there, and getting it out is what time to
        safe measures. A zone hot-applied on top of the aircraft mid-flight
        cannot be told apart from an entry without `breaches`, and counts.
    """
    known = [r for r in rows if "unsafe" in r]
    if not known:
        return {"ticks": 0, "fails": False, "reason": None, "started_unsafe": None}

    def p0(r):
        if not r.get("unsafe"):
            return False
        ids = r.get("unsafe_rules")
        if not isinstance(ids, list) or not ids:
            return True
        return any(priorities.get(str(i), "P0") == "P0" for i in ids)

    flags = [p0(r) for r in known]
    runs, start = [], None
    for i, f in enumerate(flags):
        if f and start is None:
            start = i
        elif not f and start is not None:
            runs.append((start, i, True))
            start = None
    if start is not None:
        runs.append((start, len(flags), False))
    started = bool(flags[0])
    if any(not recovered for _, _, recovered in runs):
        return {"ticks": sum(flags), "fails": True,
                "reason": "unsafe_never_recovered", "started_unsafe": started}
    breaches = metrics.get("breaches")
    if isinstance(breaches, int) and not isinstance(breaches, bool):
        entered = breaches > 0
    else:
        entered = any(s > 0 for s, _, _ in runs)
    return {"ticks": sum(flags), "fails": entered,
            "reason": "unsafe_position_entered" if entered else None,
            "started_unsafe": started}


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
            metrics: dict | None = None, *,
            theta: tuple[float, float] | None = None) -> dict[str, Any]:
    """Score ONE episode from its per-tick flight-log rows.

    `metrics` is the episode's metrics.json: dwell inside zones / rings
    (`nfz_s`, `alt_violation_s`, `standoff_s`), the declared goal
    (`reached_goal`, or the SITL rails' `reached`), the follow proxy
    (`frac_within_30m`) and the harness's `breaches` decide the mission
    outcome; `shield: "off"` marks a control arm whose logged repairs were
    never flown; `fsm` (guardrail.fsm's summary()) gives the episode's
    fail-safe outcome. `theta` overrides the (lateral, vertical) cap in
    metres. Across episodes, use `rollup()`.

    AUTOPILOT TICKS (2026-10-07). A row the Shield did not fly
    (`_not_flown_by_shield`: `autopilot` set by the stress harness, `flown`
    false or setpoint "none" on the SITL rails - LOITER / RTL / LAND after
    the escalation FSM handed over, a GeoFence takeover, a fault) is left out
    of the Shield's own counts - P0 ticks and escapes, fail-safe ticks,
    repairs and their size - and counted in `autopilot_ticks`; position-based
    figures (time to safe, unsafe P0 positions) still read it, because where
    the aircraft was does not depend on who flew it. The escape rate's
    denominator is the Shield's ticks (`shield_ticks`), on every rail alike.
    (Until the stress-harness-2 review only the harness's `autopilot` key was
    read, so a SITL rail's not-flown ticks still sat in its denominator.)
    What the AUTOPILOT flew through is not the Shield's KPI and is not here:
    the harness counts it apart (sweep_scenarios.py, `autopilot_p0_flown_ticks`
    and `autopilot_polygon_entries`).

    OUTCOME. The grant's vocabulary is success | fail | RTL_triggered |
    Land_triggered (Stress Testing p5). An episode whose FSM entered Land or
    RTL takes that label (metrics' `fsm.outcome_label`, else the rows'
    `fsm_state_after`), whatever else went wrong; the reasons stay listed."""
    rows = list(rows)
    metrics = metrics or {}
    theta = theta or theta_m()
    arm = _arm(metrics.get("shield"))
    n_autopilot = 0

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
    # The other half of "triggered when expected, NOT WHEN NOT EXPECTED"
    # (Stress Testing p6). Counted apart rather than folded into
    # failsafe_trigger_correctness: that field is compared bit-for-bit by
    # guardrail/replay.py, so changing its meaning would fail every bundle
    # already written, and a Shield that brakes on every tick would score 1.0
    # on the expected half alone.
    #
    # Which ticks are "not expected": those whose raw action broke NO rule.
    # A tick with only a P1/P2 violation is neither - the grant's FSM goes
    # Normal -> Brake on any violation it cannot repair (Safety Shield p4), so
    # a brake there is permitted, not false. (The first version of this field
    # called it false; there were no such brakes to miscount.)
    violation_free = permitted = false_trig = 0
    prev_state = "Normal"

    for r in rows:
        if _not_flown_by_shield(r):
            # See AUTOPILOT TICKS above. The FSM's state still advances, so a
            # later return to Normal is not mistaken for a fresh trigger.
            n_autopilot += 1
            after = _state(_fsm_field(r, "fsm_state_after"))
            if after is not None:
                prev_state = after
            continue
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

        # A NEW trigger on this tick. With the FSM's record, a move out of
        # Normal; staying in Brake for T_recover after the violation clears is
        # the FSM working, not a fresh alarm. Without it, the stateless
        # Shield's brake (or an escalation alias) on the tick itself.
        after = _state(_fsm_field(r, "fsm_state_after"))
        if after is not None:
            before = _state(_fsm_field(r, "fsm_state_before")) or prev_state
            new_trigger = before == "Normal" and after != "Normal"
            prev_state = after
        else:
            new_trigger = (bool(r.get("braked")) or "Brake" in _ops(r)
                           or _escalated(r))

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
        elif vios:
            permitted += 1
        else:
            violation_free += 1
            if new_trigger:
                false_trig += 1

    # Mission outcome. Decided from the recorded metrics and the log rather
    # than per demo, so every flight answers the question the same way. Every
    # reason is kept: a reader should not have to re-derive why a run failed.
    reasons: list[str] = []
    if n_p0_escapes:
        reasons.append("p0_escape")
    # Time inside a stand-off ring is a position violation exactly like time
    # inside a zone. The SITL rails have logged it since the pedestrian runs
    # (2026-08-31) and nothing read it until 2026-10-06.
    for key in ("nfz_s", "alt_violation_s", "standoff_s"):
        if (_n(metrics.get(key)) or 0) > 0:
            reasons.append(f"dwell:{key}")
    unsafe = _unsafe_p0(rows, priorities, metrics)
    if unsafe["fails"]:
        reasons.append(unsafe["reason"])
    goal = _goal_status(metrics)
    if goal is False:
        reasons.append("goal_missed")
    reached = metrics.get("frac_within_30m")
    if reached is not None and reached < FOLLOW_MIN_FRAC:
        reasons.append("follow_proxy")
    outcome = "fail" if reasons else "success"
    fs_label = fsm_outcome(rows, metrics)
    if fs_label:
        outcome = fs_label
        reasons.append(f"failsafe:{fs_label}")
    if n_p0_unknown:
        # Not an outcome failure - nothing was seen to go wrong - but a run
        # with unmeasured P0 ticks cannot claim mission success (2026-08 rule).
        reasons.append("p0_not_measurable")

    n = max(1, len(rows) - n_autopilot)
    p0_acted = (None if not failsafe_expected else
                round(failsafe_correct / failsafe_expected, 6))
    return {
        "kpi_version": "1.0",
        "ticks": len(rows),
        # The ticks the Shield decided what flew: the escape rate's denominator.
        "shield_ticks": len(rows) - n_autopilot,
        "autopilot_ticks": n_autopilot,
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
        # The share of P0 ticks the Shield acted on - one minus the P0 escape
        # rate over P0 ticks. NOT the grant's fail-safe trigger correctness
        # ("triggered when expected, not when not expected" is a question
        # about labelled EPISODES: rollup()'s `failsafe_labelled_episodes`,
        # guardrail.fsm.score_failsafe_triggers). Kept under its old name too,
        # because guardrail/replay.py verifies that key bit for bit in every
        # bundle already written (follow-up #117b, 2026-10-07).
        "p0_acted_tick_share": p0_acted,
        "failsafe_trigger_correctness": p0_acted,
        "failsafe_trigger_correctness_is": "p0_acted_tick_share (legacy name)",
        # The raw counts, so rollup() can pool across episodes instead of
        # averaging ratios of different-sized denominators.
        "failsafe_expected_ticks": failsafe_expected,
        "failsafe_correct_ticks": failsafe_correct,
        # The "not when not expected" half. A stateless Shield brakes only on a
        # violation, so on its logs this is structurally 0; it becomes
        # informative once the FSM's states are logged (fsm_state_before/after).
        "violation_free_ticks": violation_free,
        "false_trigger_ticks": false_trig,
        "false_trigger_rate": (None if not violation_free else
                               round(false_trig / violation_free, 6)),
        "failsafe_permitted_ticks": permitted,
        "repair_count": n_repairs,
        # THIS episode's count, despite the name; kept because artefacts and
        # readers already carry it. The grant's "Average repair count /
        # episode" is a mean OVER episodes and is `rollup()`'s
        # `mean_repair_count_per_episode` (audit card X-03).
        "repairs_per_episode": round(n_repairs, 3),
        # --- repair success rate --------------------------------------------
        **_repair_success(rows, priorities, arm, theta),
        # --- mean repair magnitude ------------------------------------------
        # Translational and yaw are separate quantities in separate units and
        # are never combined; see _repair_magnitude(). Computed over every tick
        # that LISTS a repair, as it always has been: replay bundles verify
        # these two fields bit-for-bit (guardrail/replay.py). On a Shield-off
        # arm those ticks flew the raw action and measure 0.0; read
        # `repair_not_applied_ticks` before quoting a magnitude.
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
        "mission_fail_reasons": reasons,
        "unsafe_p0_ticks": unsafe["ticks"],
        "mission_started_unsafe": unsafe["started_unsafe"],
        # "undeclared" is a follow flight or a scenario with no destination: a
        # vehicle hovering at the start can pass those on rules alone, which is
        # what null_hover_mission() measures.
        "mission_goal_status": {True: "reached", False: "missed",
                                None: "undeclared"}[goal],
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


# --------------------------------------------------------------------------- #
# nulls and failure cases for one episode
# --------------------------------------------------------------------------- #

FOLLOW_RANGE_M = 30.0          # compute()'s frac_within_30m radius
FOLLOW_MIN_FRAC = 0.05         # ... and the share below which it fails


def _subject_xy(row: dict) -> tuple[float, float] | None:
    if row.get("tgt_x") is not None and row.get("tgt_y") is not None:
        return float(row["tgt_x"]), float(row["tgt_y"])
    pts = (row.get("truth") or {}).get("pts") or []
    if len(pts) == 1:
        return float(pts[0][0]), float(pts[0][1])
    return None


def null_hover_mission(rows: list[dict], metrics: dict) -> dict[str, Any]:
    """Would a vehicle that never left its start point pass this mission test?

    The null for mission success. `compute()` fails an episode on escapes,
    dwell inside a zone, a missed goal, or - on follow flights, which declare
    no goal - spending under 5 % of the flight within 30 m of the subject. A
    hover commits no escape and no dwell, so its fate is decided by the last
    two alone, and both can be read off the log:

      * a declared goal: a hover does not travel, so it misses (assumes the
        goal is not the start point, true of every mission in this repo);
      * the follow proxy: the share of ticks the logged subject spent within
        30 m of the START position. If that is >= 5 %, a hover passes the same
        test the flown controller passed, and the success says nothing about
        following.

    An upper bound: it assumes the hover position itself is legal. Returns
    `success: None` where the log cannot answer.
    """
    goal = _goal_status(metrics)
    if goal is not None:
        return {"success": False, "basis": "declared goal; a hover does not travel",
                "frac_within_30m": None}
    if metrics.get("frac_within_30m") is None:
        return {"success": None, "basis": "no mission test in the metrics",
                "frac_within_30m": None}
    start = next(((float(r["x"]), float(r["y"])) for r in rows
                  if r.get("x") is not None and r.get("y") is not None), None)
    seps = []
    if start is not None:
        for r in rows:
            s = _subject_xy(r)
            if s is not None:
                seps.append(math.hypot(s[0] - start[0], s[1] - start[1]))
    if not seps:
        return {"success": None,
                "basis": "follow proxy, but the log carries no subject position",
                "frac_within_30m": None}
    frac = sum(d <= FOLLOW_RANGE_M for d in seps) / len(seps)
    return {"success": frac >= FOLLOW_MIN_FRAC,
            "basis": f"follow proxy: subject within {FOLLOW_RANGE_M:.0f} m of the "
                     f"start point on {frac:.1%} of ticks",
            "frac_within_30m": round(frac, 4)}


def worst_tick(rows: list[dict], priorities: dict[str, str]) -> dict | None:
    """The tick a failure-case row shows ("raw vs repaired action", Stress
    Testing p6, top-K columns): the first P0 escape, else the first repair that
    fell through, else the first unsafe position. None for a clean episode."""
    def pick(why, r):
        return {"why": why, "t": r.get("t"), "tick": r.get("tick"),
                "raw": r.get("raw"), "emitted": r.get("emitted"),
                "rules": sorted({v.get("rule_id", "") for v in
                                 (r.get("violations") or [])}),
                "emitted_rules": sorted({v.get("rule_id", "") for v in
                                         (r.get("emitted_violations") or [])})}

    for r in rows:
        em_v = r.get("emitted_violations") or []
        if any(priorities.get(v.get("rule_id", ""), "P0") == "P0" for v in em_v) \
                and any(priorities.get(v.get("rule_id", ""), "P0") == "P0"
                        for v in (r.get("violations") or [])):
            return pick("p0_escape", r)
    for r in rows:
        oc = _repair_outcome(r, priorities)
        if oc in _REPAIR_FELL_THROUGH:
            return pick(f"repair_{oc}", r)
    for r in rows:
        if r.get("unsafe"):
            return pick("unsafe_position", r)
    return None


def failure_categories(k: dict) -> list[str]:
    """Why an episode is a failure case, from its KPI table alone. Empty for a
    clean one. Works on stored tables that predate the newer fields: a field
    that is absent contributes nothing rather than a guess."""
    out = []
    if (k.get("p0_escapes") or 0) > 0:
        out.append("p0_escape")
    if (k.get("p0_ticks_not_measurable") or 0) > 0:
        out.append("p0_not_measurable")
    if (k.get("time_to_safe_censored") or 0) > 0:
        out.append("unsafe_never_recovered")
    if (k.get("time_to_safe_episodes") or 0) > 0:
        out.append("unsafe_position")
    if k.get("mission_goal_status") == "missed" or k.get("reached_goal") is False:
        out.append("goal_missed")
    oc = k.get("repair_outcomes") or {}
    if sum(oc.get(x, 0) or 0 for x in _REPAIR_FELL_THROUGH):
        out.append("repair_fell_through")
    if (k.get("false_trigger_ticks") or 0) > 0:
        out.append("false_fail_safe")
    fs = k.get("mission_outcome") or k.get("outcome")
    if fs in ("RTL_triggered", "Land_triggered"):
        # The mission was aborted to the autopilot. A failure CASE whether or
        # not it was expected: the top-K table is where a reader looks first.
        out.append(fs)
    if k.get("failsafe_matches_label") == 0.0:
        out.append("failsafe_label_mismatch")
    # A stored table written before compute() read unsafe positions can say
    # "success" for a run that ended inside a ring. Say so on the row rather
    # than letting the two columns contradict each other silently.
    if (k.get("time_to_safe_censored") or 0) > 0 and mission_success_with_goal(k):
        out.append("mission_success_despite_unrecovered_unsafe")
    if not out and mission_success_with_goal(k) is False:
        out.append("mission_fail")
    return out


# --------------------------------------------------------------------------- #
# across episodes: the KPI report's per-family row
# --------------------------------------------------------------------------- #

def _n(v) -> float | None:
    """A finite number, or None - bools and NaN are not numbers here."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    return v if math.isfinite(v) else None


def _carrying(episodes: list[dict], *fields: str) -> list[dict]:
    """The episodes on which every one of `fields` is a number."""
    return [e for e in episodes if all(_n(e.get(f)) is not None for f in fields)]


def _labelled_failsafe(episodes: list[dict]) -> dict[str, Any] | None:
    """Episode-level fail-safe correctness where the harness labelled episodes.

    The grant's ">= 99 %, triggered when expected, not when not expected"
    (Stress Testing p6) is a question about EPISODES with a label, and
    guardrail/fsm.py's `score_failsafe_triggers` answers it with its two nulls
    (always trigger, never trigger). The stress harness stores
    `failsafe_triggered` and `failsafe_matches_label`, from which the label
    follows. None when no episode carries a label.
    """
    eps = []
    for e in episodes:
        trig, match = e.get("failsafe_triggered"), _n(e.get("failsafe_matches_label"))
        if isinstance(trig, bool) and match is not None:
            eps.append({"expected_failsafe": trig if match == 1.0 else not trig,
                        "triggered": trig})
    if not eps:
        return None
    fsm = _fsm()
    fn = getattr(fsm, "score_failsafe_triggers", None) if fsm is not None else None
    if fn is not None:
        s = fn(eps)
        return {k: s.get(k) for k in ("failsafe_trigger_correctness", "scored",
                                       "false_triggers", "missed_triggers",
                                       "null_always_trigger", "null_never_trigger",
                                       "discriminating")}
    tp = sum(1 for e in eps if e["expected_failsafe"] and e["triggered"])
    tn = sum(1 for e in eps if not e["expected_failsafe"] and not e["triggered"])
    pos = sum(1 for e in eps if e["expected_failsafe"])
    return {"failsafe_trigger_correctness": round((tp + tn) / len(eps), 6),
            "scored": len(eps),
            "false_triggers": sum(1 for e in eps if e["triggered"] and not e["expected_failsafe"]),
            "missed_triggers": pos - tp,
            "null_always_trigger": round(pos / len(eps), 6),
            "null_never_trigger": round((len(eps) - pos) / len(eps), 6),
            "discriminating": 0 < pos < len(eps)}


def rollup(episodes: list[dict]) -> dict[str, Any]:
    """One row of the Stress Testing KPI report's "per-scenario-family stats
    table" (p6): episode count, P0 escape rate, fail-safe correctness, mission
    success rate, mean repair count, mean repair magnitude - plus the repair
    success rate and time to safe the Safety Shield page locks, and the null
    beside every score.

    Each episode is a `compute()` table, optionally carrying
    `null_hover_mission` (from `null_hover_mission()`). Older stored tables are
    accepted. A field an episode lacks leaves that episode out of BOTH the
    numerator and the denominator of that one figure and is counted in
    `episodes_<figure>_not_measurable`; a figure no episode carries is None,
    never 0. (The first version summed `e.get(f) or 0`, so a table with no
    escape count at all pooled as a measured zero escape rate.)

    POOLED, NOT AVERAGED, where the grant's figure is a rate over ticks: two
    episodes of 50 and 5000 ticks do not deserve equal weight in an escape
    rate. "Average repair count / episode" is the one figure the grant defines
    per episode, and it is a plain mean over the episodes that carry a count.
    """
    n = len(episodes)
    ticks = sum(e["ticks"] for e in _carrying(episodes, "ticks"))

    def shield_ticks(e: dict) -> float:
        # The escape rate's denominator: the ticks the Shield decided what
        # flew (compute()'s `shield_ticks`); every tick on a table without it.
        st = _n(e.get("shield_ticks"))
        return e["ticks"] if st is None else st

    # --- P0 escape, and its passthrough null ------------------------------
    p0_eps = _carrying(episodes, "ticks", "p0_escapes")
    p0_ticks = sum(shield_ticks(e) for e in p0_eps)
    esc = sum(e["p0_escapes"] for e in p0_eps)
    pt_eps = _carrying(episodes, "ticks", "p0_violation_ticks")
    pt_ticks = sum(shield_ticks(e) for e in pt_eps)
    p0t = sum(e["p0_violation_ticks"] for e in pt_eps)
    unk_eps = _carrying(episodes, "p0_ticks_not_measurable")

    # --- fail-safe, expected half (pooled counts) --------------------------
    exp = corr = fs_n = 0
    for e in episodes:
        x, c = _n(e.get("failsafe_expected_ticks")), _n(e.get("failsafe_correct_ticks"))
        if x is None or c is None:
            # An older table: rebuild the counts from the ratio, and only when
            # the ratio is there to rebuild them from.
            x = _n(e.get("p0_violation_ticks"))
            rate = _n(e.get("failsafe_trigger_correctness"))
            if x is None or (x and rate is None):
                continue
            c = round(rate * x) if x else 0
        exp += x
        corr += c
        fs_n += 1
    # ... and the false-trigger half
    ft_eps = _carrying(episodes, "violation_free_ticks", "false_trigger_ticks")
    ft_n = sum(e["violation_free_ticks"] for e in ft_eps)
    ft_bad = sum(e["false_trigger_ticks"] for e in ft_eps)

    # --- mission success, with the goal requirement applied to old tables ---
    verdicts = [mission_success_with_goal(e) for e in episodes]
    scored = [v for v in verdicts if v is not None]
    goal_counts = {"reached": 0, "missed": 0, "undeclared": 0}
    for e in episodes:
        st = e.get("mission_goal_status")
        if st is None and "reached_goal" in e:
            st = {True: "reached", False: "missed", None: "undeclared"}[
                None if e["reached_goal"] is None else bool(e["reached_goal"])]
        if st in goal_counts:
            goal_counts[st] += 1

    # --- repair count and magnitude ----------------------------------------
    rc_eps = _carrying(episodes, "repair_count")
    att_eps = _carrying(episodes, "repair_attempt_ticks")
    mag_w = [(e["mean_repair_magnitude_mps"], e["repaired_ticks"])
             for e in _carrying(episodes, "mean_repair_magnitude_mps", "repaired_ticks")
             if e["repaired_ticks"] > 0]
    mag_n = sum(w for _, w in mag_w)
    mags_max = [e["max_repair_magnitude_mps"]
                for e in _carrying(episodes, "max_repair_magnitude_mps")]

    # --- repair success -----------------------------------------------------
    rs_eps = [e for e in _carrying(episodes, "repair_attempt_ticks", "repair_success_ticks")
              if e.get("repair_success_status") is not None]
    rs_att = sum(e["repair_attempt_ticks"] for e in rs_eps)
    rs_unk = sum(_n(e.get("repair_unmeasured_ticks"))
                 if _n(e.get("repair_unmeasured_ticks")) is not None
                 else ((e.get("repair_outcomes") or {}).get("not_measurable") or 0)
                 for e in rs_eps)
    rs_ok = sum(e["repair_success_ticks"] for e in rs_eps)
    rs_measured = rs_att - rs_unk
    oc_eps = [e for e in rs_eps if isinstance(e.get("repair_outcomes"), dict)]
    outcomes = ({k: sum((e["repair_outcomes"].get(k) or 0) for e in oc_eps)
                 for k in REPAIR_OUTCOMES} if oc_eps else None)
    if not rs_eps:
        rs_status, rs_rate = "not_measurable", None
    elif rs_att == 0:
        rs_status = ("shield_off" if all(e["repair_success_status"] == "shield_off"
                                         for e in rs_eps) else "no_repairs_attempted")
        rs_rate = None
    elif rs_measured == 0:
        rs_status, rs_rate = "not_measurable", None
    else:
        rs_rate = round(rs_ok / rs_measured, 6)
        rs_status = ("measured" if not rs_unk and len(rs_eps) == n
                     else "partially_measured")
    th_eps = _carrying(rs_eps, "repair_theta_unchecked_ticks")
    th_unchecked = sum(e["repair_theta_unchecked_ticks"] for e in th_eps)
    # What the pooled successes rest on. A stored table from before theta was
    # read has no basis field, and its successes were never judged against
    # theta: it counts as re-check only, whatever the other episodes say.
    # An episode whose successes needed no theta check says nothing about the
    # others, so it only decides the basis when it is the only kind there is.
    kinds = {e.get("repair_success_basis") or "recheck_only_theta_not_applied"
             for e in rs_eps if e["repair_success_ticks"]}
    if len(kinds) > 1:
        kinds.discard("recheck_theta_not_applicable")
    rs_basis = (None if not kinds else kinds.pop() if len(kinds) == 1
                else "recheck_and_theta_partial")
    wu_eps = _carrying(rs_eps, "repairs_converged_while_unsafe")

    # --- time to safe, pooled over unsafe episodes ---------------------------
    t_eps = [e for e in episodes if e.get("time_to_safe_not_measurable") is False]
    t_w = [(e["mean_time_to_safe_s"], e.get("time_to_safe_episodes") or 0)
           for e in t_eps if _n(e.get("mean_time_to_safe_s")) is not None]
    t_n = sum(w for _, w in t_w)
    t_max = [e["max_time_to_safe_s"] for e in t_eps
             if _n(e.get("max_time_to_safe_s")) is not None]

    # Four different answers that a bare `mean_time_to_safe_s: None` would
    # collapse into one: measured, measured-and-never-unsafe, never recovered,
    # and could not be measured at all.
    t_cens = sum(e.get("time_to_safe_censored") or 0 for e in t_eps)
    if not t_eps:
        t_status = "not_measurable"
    elif t_n:
        t_status = "measured"
    elif t_cens:
        t_status = "never_recovered"
    else:
        t_status = "never_unsafe"

    hover = [((e.get("null_hover_mission") or {}).get("success")) for e in episodes]
    hover_known = [h for h in hover if h is not None]

    fs_outcomes = {o: sum(1 for e in episodes
                          if (e.get("mission_outcome") or e.get("outcome")) == o)
                   for o in OUTCOMES}
    return {
        "episodes": n,
        "ticks": ticks,
        "autopilot_ticks": sum(e["autopilot_ticks"] for e in
                               _carrying(episodes, "autopilot_ticks")),
        # The grant's outcome vocabulary, counted (success | fail |
        # RTL_triggered | Land_triggered).
        "outcomes": fs_outcomes,
        # --- P0 escape ------------------------------------------------------
        "p0_escape_rate": (round(esc / p0_ticks, 6) if p0_ticks else None),
        "p0_escapes": esc if p0_eps else None,
        "episodes_with_p0_escape": sum(1 for e in p0_eps if e["p0_escapes"] > 0),
        "episodes_p0_escape_not_measurable": n - len(p0_eps),
        "p0_violation_ticks": p0t if pt_eps else None,
        "p0_ticks_not_measurable": (sum(e["p0_ticks_not_measurable"] for e in unk_eps)
                                    if unk_eps else None),
        # --- fail-safe ------------------------------------------------------
        # The share of P0 ticks the Shield acted on, pooled: one minus the
        # P0 escape rate over P0 ticks. NOT the grant's fail-safe trigger
        # correctness, which is `failsafe_labelled_episodes` below. Published
        # under the grant KPI's name until the stress-harness-2 review, where
        # it read 1.0 beside the labelled 0.747 of the same run; only the
        # per-episode table keeps the legacy name, for guardrail/replay.py.
        "p0_acted_tick_share": (round(corr / exp, 6) if exp else None),
        "failsafe_expected_ticks": exp if fs_n else None,
        "episodes_failsafe_not_measurable": n - fs_n,
        "false_trigger_rate": (round(ft_bad / ft_n, 6) if ft_n else None),
        "false_trigger_ticks": ft_bad if ft_eps else None,
        "violation_free_ticks": ft_n if ft_eps else None,
        "episodes_false_trigger_not_measurable": n - len(ft_eps),
        "failsafe_labelled_episodes": _labelled_failsafe(episodes),
        # --- mission --------------------------------------------------------
        "mission_success_rate": (round(sum(scored) / len(scored), 6)
                                 if scored else None),
        "mission_successes": sum(scored),
        "episodes_mission_scored": len(scored),
        "mission_goal": goal_counts,
        # --- repairs --------------------------------------------------------
        "mean_repair_count_per_episode": (
            round(sum(e["repair_count"] for e in rc_eps) / len(rc_eps), 3)
            if rc_eps else None),
        "episodes_repair_count_not_measurable": n - len(rc_eps),
        "mean_repair_attempt_ticks_per_episode": (
            round(sum(e["repair_attempt_ticks"] for e in att_eps) / len(att_eps), 3)
            if att_eps else None),
        "mean_repair_magnitude_mps": (round(sum(m * w for m, w in mag_w) / mag_n, 4)
                                      if mag_n else None),
        "max_repair_magnitude_mps": (round(max(mags_max), 4) if mags_max else None),
        "repair_success_rate": rs_rate,
        "repair_success_status": rs_status,
        "repair_success_basis": rs_basis,
        "repair_attempt_ticks": rs_att,
        "repair_success_ticks": rs_ok,
        "repair_theta_unchecked_ticks": th_unchecked if th_eps else None,
        "repairs_converged_while_unsafe": (
            sum(e["repairs_converged_while_unsafe"] for e in wu_eps) if wu_eps else None),
        "repair_outcomes": outcomes,
        "episodes_repair_success_not_measurable": n - len(rs_eps),
        # --- time to safe ---------------------------------------------------
        "mean_time_to_safe_s": (round(sum(m * w for m, w in t_w) / t_n, 3)
                                if t_n else None),
        "max_time_to_safe_s": (round(max(t_max), 3) if t_max else None),
        "time_to_safe_status": t_status,
        "time_to_safe_episodes": t_n,
        "time_to_safe_censored": t_cens,
        "episodes_time_to_safe_measured": len(t_eps),
        "episodes_time_to_safe_not_measurable": n - len(t_eps),
        # --- nulls ----------------------------------------------------------
        # A passthrough Shield flies every raw command, so it escapes on every
        # P0 tick. Open-loop: in closed loop its trajectory, and so its count,
        # would differ - but it is the floor the Shield's 0 is measured against.
        "null_passthrough_p0_escape_rate": (round(p0t / pt_ticks, 6) if pt_ticks else None),
        # A Shield that brakes on every violation converges on none of them.
        "null_always_brake_repair_success_rate": (0.0 if (rs_att or p0t) else None),
        # A vehicle that never moves: see null_hover_mission().
        "null_hover_mission_success_rate": (round(sum(hover_known) / len(hover_known), 6)
                                            if hover_known else None),
        "null_hover_known_episodes": len(hover_known),
    }


def _severity(e: dict) -> tuple:
    """Sort key for the top-K list, most severe first: P0 escapes, unsafe
    positions never recovered, longest time in an unsafe position, a failed
    mission (missed goal, dwell in a zone), repairs that fell through - and
    only then P0 ticks the log could not measure, which is a gap in the
    evidence rather than an observed failure."""
    oc = e.get("repair_outcomes") or {}
    fell = sum(oc.get(x, 0) or 0 for x in _REPAIR_FELL_THROUGH)
    return ((e.get("p0_escapes") or 0) > 0, e.get("p0_escapes") or 0,
            e.get("time_to_safe_censored") or 0,
            _n(e.get("max_time_to_safe_s")) or 0.0,
            mission_success_with_goal(e) is False, fell,
            e.get("p0_ticks_not_measurable") or 0)


def top_failures(episodes: list[dict], k: int = 10,
                 include_controls: bool = False) -> list[dict]:
    """The Stress Testing KPI report's "Top-K failure cases" (p6, K=10 default).

    Each episode dict is a `compute()` table plus `id`, `family`, `arm`,
    `params`, `bundle` and `worst_tick`. Control arms - flown with the Shield
    OFF on purpose, so that their escapes can be compared against - are left
    out unless asked for: they fail by design, and listing them would push the
    Shield's own failures off a ten-row table.
    """
    rows = []
    for e in episodes:
        if not include_controls and e.get("arm") == "off":
            continue
        cats = failure_categories(e)
        if cats:
            rows.append((e, cats))
    rows.sort(key=lambda ec: _severity(ec[0]), reverse=True)
    out = []
    for e, cats in rows[:k]:
        wt = e.get("worst_tick") or {}
        out.append({
            "scenario_id": e.get("id"),
            "family": e.get("family"),
            "parameters": e.get("params"),
            "failure_category": cats,
            "p0_escapes": e.get("p0_escapes"),
            "mission_success": mission_success_with_goal(e),
            "time_to_safe_censored": e.get("time_to_safe_censored"),
            "max_time_to_safe_s": e.get("max_time_to_safe_s"),
            "time_to_safe_basis": e.get("time_to_safe_basis"),
            "raw_action": wt.get("raw"),
            "repaired_action": wt.get("emitted"),
            "at_t": wt.get("t"),
            "worst_tick_why": wt.get("why"),
            "bundle": e.get("bundle"),
            "kpi_grade": e.get("kpi_grade"),
        })
    return out
