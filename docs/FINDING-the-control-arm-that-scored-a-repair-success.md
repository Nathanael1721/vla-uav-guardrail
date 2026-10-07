# The control arm that scored a repair success

**Date:** 2026-10-06
**Found by:** the KPIs-from-logs work package, reading the stress-harness
control arms beside their shielded twins while building `tools/kpi_report.py`.
**Status:** fixed in `guardrail/kpi.py` (scoring) and `tools/kpi_report.py`
(stored tables). The three stress-harness `kpi.json` files written before the
fix still hold the old figure; they are listed below.

## What it was for

Every stress scenario that tests a rule has a control arm: the same flight
with the Shield switched off. `nfz-head-on-control` exists to FAIL its safety
gate. If an unguarded run into a no-fly zone scored clean, the harness would be
measuring nothing. The control arm's escape rate is the null for the shielded
arm's.

The headless sweep still runs the Shield on the control arm, to log the repair
it WOULD have made, and then flies the raw action
(`experiments/sweep_scenarios.py`: `flown = d.emitted if on else raw`).

## What went wrong

`guardrail.kpi.compute` scored those logged repairs as attempts. The re-check
of each one found the raw action, which is what flew, still violating the
zone. Every logged repair was therefore an attempt that failed, and the
control arm reported:

| Field (stored `kpi.json`) | Value |
|---|---|
| `repair_count` | 52 |
| `repair_attempt_ticks` | 52 |
| `repair_outcomes.residual_p0` | 52 |
| `repair_success_rate` | **0.0** |
| `repair_success_status` | **`measured`** |
| `mean_repair_magnitude_mps` | 0.0, over 52 "repaired" ticks |

These are the values in
`demo/out/stress_nightly/episode-2026-10-06T04-20-45Z--nfz-head-on-control--seed1001/kpi.json`,
and the same in the `seed1002` and `seed1003` bundles.

None of the 52 repairs was applied. A repair success rate of 0.0 says the
Shield tried 52 times and failed 52 times. In fact it never acted. The same
table also gave a mean repair magnitude of 0.000 m/s over 52 "repaired" ticks
and a repair count of 52 per episode, where the grant's "Average repair count /
episode" counts repairs that were applied. There was no error and no warning.
The figure was a real-looking measurement. Pooled into a family with the
shielded arm, it would have pulled the family's repair success down. Read on
its own, it says the Shield's repair layer fails every time.

## The evidence

The KPI rollup of 6 Oct (`docs/data/kpi_rollup_2026-10-06.json`,
`python tools/kpi_report.py --out docs/data/kpi_rollup_2026-10-06`) lists four
control episodes with logged, unflown repairs: the three stress bundles above
and the sweep's own `nfz-head-on-control` (`docs/data/scenario_sweep.json`).
Each carried 52 counterfactual repairs. After the fix each reads:

| Field | Before | After |
|---|---|---|
| `repair_success_status` | `measured` | `shield_off` |
| `repair_success_rate` | 0.0 | None |
| `repair_count` | 52 | 0 (`repair_count_counterfactual`: 52) |
| `mean_repair_magnitude_mps` | 0.0 | None |
| `p0_violation_escape_rate` | 0.173333 | 0.173333 (unchanged) |

The escape rate is unchanged, and it is the number the control arm exists for.

## The fix

- `guardrail/kpi.py`: a repair logged on a Shield-off arm, or on a row that
  flew the raw action, is `not_applied`, not an attempt. With attempts = 0 and
  such repairs present, the status is `shield_off` and the rate is None, as
  the docstring's "answers that must not look alike" table now lists. 0.0
  still means every attempt failed, which is what an always-brake Shield scores
  (the null).
- `tools/kpi_report.py` `shield_off_table()`: a stored table from before the
  fix (the stress bundles carry no per-tick log to re-score) has the unflown
  repairs moved to `repair_count_counterfactual`. Its repair success, outcomes
  and magnitudes become None. The rollup prints `shield off` for such a row and
  leaves control arms out of the top-failure ranking unless
  `--include-controls` is given.

## What is still open

The three `kpi.json` files in `demo/out/stress_nightly/*nfz-head-on-control*`
still hold `repair_success_rate: 0.0, "measured"`. They are corrected only
where the report reads them. Re-running the nightly profile with
`--keep-logs` lets them be re-scored from their own rows.

## The general lesson

A control arm that logs what the treatment WOULD have done looks, to a scorer
that does not know which arm it is reading, exactly like a treatment that
tried and failed. The scorer has to know which arm it is reading. "Did not
act" and "acted and failed" must be different values, never both 0.0.
