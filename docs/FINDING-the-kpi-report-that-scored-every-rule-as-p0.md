# The KPI report that scored every rule as P0

**Date:** 2026-10-06
**Found by:** review of the first version of `tools/kpi_report.py`, while it was
being built (the KPIs-from-logs work package).
**Status:** fixed in `tools/kpi_report.py`; the stored per-run `kpi.json` files
were never affected.

## What it was for

`tools/kpi_report.py` re-scores every recorded run from its own flight log with
`guardrail.kpi.compute`, so a KPI rollup can be produced without trusting the
numbers each flight wrote about itself. To decide whether a violation is a P0
one, `compute()` needs the run's rule priorities. They live in the policy, and
the report finds the policy from the hash in the run's manifest.

## What went wrong

`compute()` treats a rule id it cannot look up as P0:

```python
lvl = priorities.get(rid, "P0")     # unknown rule treated as P0
```

That default is the safe one for a single flight scored against its own policy:
an unknown rule should never make an escape disappear. The first version of the
report called `compute(rows, {}, metrics)` for every run whose policy it could
not resolve. With an empty table **every** rule is unknown, so every P1 and P2
violation, a speed cap, an altitude band, was counted as a P0 tick. No error,
no warning; the run simply looked worse.

## The evidence

Re-scoring the flight logs with an empty priority table, against the figures
each run computed in flight with its real policy:

| Run | P0 ticks, empty table | P0 ticks, in flight |
|---|---|---|
| `ros2_shield_on_dynamic` | 309 | 217 |
| `sitl_shield_on_dynamic` | 219 | 131 |

Reproduced on this machine:

```python
import json, guardrail.kpi as K
rows = [json.loads(l) for l in open("demo/out/ros2_shield_on_dynamic/flight_log.jsonl")]
K.compute(rows, {}, {})["p0_violation_ticks"]       # 309
```

Both runs fly a hot-applied policy, whose hash matched no file on disk until
the policy-identity work of the same day, which is why the lookup failed for
exactly these runs.

## The fix

- Every field whose value depends on priorities (`PRIORITY_FIELDS` in
  `tools/kpi_report.py`: escape rate, P0 ticks, fail-safe figures, repair
  outcomes, mission outcome, unsafe P0 ticks) is **never recomputed against an
  empty table**. For a run whose policy cannot be resolved it is taken from the
  stored `kpi.json`, computed in flight against the real policy, or left None,
  and the row says which (`priority_fields_from`).
- `tools/rescore_kpis.py` refuses to *write* those fields without a policy, by
  the same list.
- The policy-identity work made 76 of 76 stored manifests traceable to a
  policy (from 34), so far fewer runs need the fallback at all.

## The general lesson

A default that is safe for one caller can be wrong for another. "Unknown means
P0" protects a flight from losing an escape; inside a bulk re-scorer the same
default inflates every figure it touches. A recomputation that cannot get its
inputs must say "not measurable" or use the value computed with them, never a
plausible substitute.
