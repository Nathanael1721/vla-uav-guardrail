# Repair selection A/B: chain vs first_feasible (headless library, 27 Nov 2026)

Row WP3-06. The grant's safety-shield.md says the repair operators are tried
in priority order and the first that produces a feasible action within the
threshold wins. The Shield's default is the iterated chain
(`repair_mode="chain"`, guardrail/repair.py THE MODES). The grant's rule
(`repair_mode="first_feasible"`) existed but no flying or library harness
could select it. Since commit abff166 three harnesses can:
`--repair-mode` on sitl/ros2_shield_node.py, sitl/run_sitl_demo.py and the
headless sweep, `--rail-repair-mode` on the sweep's rails. The default stays
`chain`.

This folder is the first measurement of first_feasible on the scenario
library since the 17 Nov PathRepair redesign. In that mode PathRepair is
ungated and detours round keep-out zones as well
(guardrail/repair.py PathRepair docstring).

## What was run

Revision `abff166efc2d` (clean tree, recorded in each file's
`_manifest_common.code_revision`). The scenario library and seeds are those
of docs/data/scenario_sweep.json (experiments/scenarios.yaml, its declared
seeds, no profile), on the headline arm only (grant forecast, ideal vehicle,
guided wind model, moved hold points, cap_all recovery). `--publish` also flies
the 3 s comparison forecast, and it refuses `--repair-mode`, so the two arms
were run as plain library runs:

    python experiments/sweep_scenarios.py --out demo/out/ab_repair_1127/sweep_chain.json --runs demo/out/ab_repair_1127/runs_chain --no-bundles
    python experiments/sweep_scenarios.py --repair-mode first_feasible --out demo/out/ab_repair_1127/sweep_first_feasible.json --runs demo/out/ab_repair_1127/runs_first_feasible --no-bundles

The JSONs were copied here unchanged. `comparison.json` is
`compare.py sweep_chain.json sweep_first_feasible.json`.

Headless-kinematic topology: functional evidence, never a KPI-grade number
(each file's `_not_kpi_grade`).

## Per arm (180 episodes each, 179 Shield-on)

| | chain (default) | first_feasible |
|---|---|---|
| verdicts pass / tracked / fail / known | 155 / 22 / 0 / 3 | 149 / 22 / 6 / 3 |
| P0 escapes (Shield log, grant KPI) | 0 of 11385 P0 violation ticks | 0 of 12452 P0 violation ticks |
| fail-safe trigger correctness | 179/179 = 1.0 (Wilson 95 % low 0.97899) | 177/179 = 0.988827 (2 missed, 0 false; Wilson low 0.960183) |
| mission success | 96 / 180 | 98 / 180 |
| repairs (sum of repair_count, Shield-on) | 18116 | 20921 |
| repaired ticks (Shield-on) | 16100 | 17080 |
| true-state P0 ticks flown | 1326 | 1333 |
| outcomes success / fail / RTL / Land | 96 / 21 / 58 / 5 | 98 / 21 / 56 / 5 |

## Where they differ

19 of 180 episodes differ in at least one of status, mission success,
outcome, fail-safe match, P0 escapes, repair count, true-state P0 ticks, max
time to safe, polygon entries or breaches. 13 of them differ beyond the repair
count. Scenarios: corridor-drift-angles, dyn-nfz-rotate-spin,
dyn-nfz-spawn-on-top, gps-noise-within-margin, rule-activated-mid-run,
three-simultaneous-violations, time-window-opens-ahead,
time-window-opens-on-sideways-pilot, time-window-opens-on-vehicle.

The six first_feasible failures (pass on chain):

- corridor-drift-angles ask 4 m/s and ask 8 m/s, drift -60 deg: the label
  expects RTL. first_feasible kept the aircraft in the corridor and the
  mission succeeded, so no fail-safe was triggered. These are the 2 missed
  triggers. Repairs went from 78 to 1031 and from 206 to 1903.
- time-window-opens-on-vehicle, three clocks (07:29:58, 07:29:58.5,
  07:29:59; vehicle 1.5 m/s): 41, 46 and 51 ticks inside the zone (the gate
  allows 15), time to safe 6.7 s (the gate allows 2.7 s). On chain each
  episode had 26 repairs and a time to safe of 2.6 s; on first_feasible each
  had 0 repairs.
- time-window-opens-on-sideways-pilot: 30 ticks inside (gate 7), time to
  safe 5.5 s (gate 2.9 s). On chain the time to safe was 2.8 s. Repairs went
  from 300 to 245.

Other differences: three-simultaneous-violations (six cells) went from 8
repairs to 27-147 and from 19 to 20 true-state P0 ticks.
dyn-nfz-spawn-on-top (size 20 and 40) went from 28 and 53 repairs to 0.
corridor-drift-angles ask 8, drift -30 went from 0 to 1 true-state P0 tick.
The rest differ in repair count only. Per-episode values are in
`comparison.json` `diffs`.

## Reading

- The P0 escape count is 0 on both arms.
- first_feasible regresses where a zone opens on the aircraft (the
  time-window-opens-on-vehicle and -on-sideways-pilot cells). There it
  leaves the zone more slowly, which fails the recovery gates. On
  time-window-opens-on-vehicle and dyn-nfz-spawn-on-top it records no repair
  at all. It also misses the two RTLs that the corridor drift-out cells are
  labelled to expect.
- This is the ungated mode measured once on the headless harness. It is
  evidence for the PI's question on the deviation (chain as default, the
  grant's selection available but off). It is not a case for changing the
  default. No rail flight has been flown on first_feasible.
