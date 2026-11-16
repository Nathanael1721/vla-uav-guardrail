# Prefix A/B on hil, campaign pab1 (16 Nov 2026)

33 missions of the `prefix_ab` profile (5 cells x 3 seeds x prefix on/off, plus 3 A/A repeats) flown on the hil rig at c7257f1 (Project AirSim + ArduPilot SITL on the desktop, Shield and pilot on the AGX Orin). The pilot is guardrail.prefix_consumer.PrefixConsumer, a hand-written CSP-consuming adapter, not a learned VLA. Part of any on/off delta holds by construction because the adapter reads the geometry the Shield enforces.

Files:

- `rescore_r2_pab1.json` / `.txt`: rail re-score.
  - Command: `experiments/sweep_scenarios.py --rescore-rail "demo/out/hil_smoke/pab1_*" --profile prefix_ab`.
  - The 18 prefix-on runs carry `labels_not_applied`. Each cell's fail-safe label was derived for its own pilot, so it is not scored on the CSP-consuming one.
- `kpi_report_pab1_2026-11-16.md` / `.json` / `kpi_report_stdout.txt`: `tools/kpi_report.py --campaign demo/out/hil_pab1 --rescore-rail rescore_r2_pab1.json`, regenerated with 336355f.
  - The first report, generated before that fix, read the unlabelled prefix-on runs as correct and showed 33/33.

## Results

- **KPIs:**
  - 33 episodes, all KPI-grade.
  - P0 escapes 0 on 2223 P0 ticks.
  - Tick p50 5.1 / p99 15.4 / max 24.23 ms, 0 of 14756 ticks over 100 ms; end-to-end max 24.32 ms.
  - Fail-safe (adopted reading r2): 15/15 on the labelled prefix-off arm (Wilson 95 % low 0.796). Reading (a) gives 12/15: the 3 nfz-head-on prefix-off runs are the same rail pattern as nightly ntFIN2.
- **A/B, on vs off:**

  | Measure | On | Off | A/A, largest in one pair |
  |---|---|---|---|
  | Mission success | 0.600 | 0.600 | – |
  | Repairs per episode | 6.6 | 130.5 | 7 |
  | Mean repair magnitude | 1.78 m/s | 9.38 m/s | – |

## Finding: the corridor cell's prefix-on arm ended in RTL 3/3

On corridor-drift-out the prefix-on arm ended in RTL 3/3; the off arm succeeded (the Shield held it at the corridor edge in Brake). A headless reproduction of pab1_008 matches the logged repairs to 5e-4 m. Two causes compounded.

1. **The consumer sampled the path too coarsely.** It tested admissibility only every 0.5 s. From (26.30, 12.48) the 60-degree heading (3.46, 2.0) m/s cuts the dog-leg's inner corner, vertex (40, 20), for 0.21 s and about 0.25 m. That cut falls between the 3.5 s and 4.0 s samples. The Shield's 0.1 s forecast catches it at t+3.8 s.
2. **CorridorReturn overstates the excursion at a corner.** It sizes the excursion as a straight line along the nearest leg: d + outward x L - half = 2.48 m, about 10x the true excursion. That is over theta 2.0 m, so the chain was G3 Loiter, then X1, then G5 RTL on the 30 s loiter timeout.

**Fix on branch kpi-prefix-labels (554e35d and its review fixes):**

- The consumer now tests the whole straight path to the lookahead against the corridor's tube and the zones' margin rings.
- Headless results:
  - corridor-drift-out prefix-on: 0 repairs and no FSM edge.
  - nfz-head-on prefix-on: the same mechanism, 19 repairs and 2 Loiters before the fix, 0 after.

**The Shield is unchanged:**

- Its corner sizing errs in the safe direction (it over-escalates and never under-repairs). A test pins it as found.
- Changing it would move the certified ntFIN2 and hil readings. It is left for after 30 Nov.

This campaign was flown before the fix; a re-fly of the prefix-on arms is owed.
