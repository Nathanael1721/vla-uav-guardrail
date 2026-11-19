# Prefix A/B on hil, campaign pab3 (19 Nov 2026)

The prefix A/B re-flown on the code the hil Shield runs since 06518b7
(clearance horizon sized for the ArduPilot vehicle's response). Same profile
as pab2 (docs/data/kpi_prefix_ab_20261117/): 33 missions of `prefix_ab`,
5 cells x 3 seeds x prefix on/off, plus 3 A/A repeats.

- **Commit flown:** 1239619 (clean tree, detached worktree).
- **Rig (hil):** Project AirSim + ArduPilot SITL on the desktop; Shield and
  pilot on the AGX Orin.
- **Pilot:** guardrail.prefix_consumer.PrefixConsumer, a hand-written
  CSP-consuming adapter, not a learned VLA. Part of any on/off delta holds by
  construction, because the adapter reads the geometry the Shield enforces.
- **Duration:** 4500 s.

## Files

| File | What it is | Command |
|---|---|---|
| `rescore_r2_pab3.json` / `.txt` | Rail re-score | `experiments/sweep_scenarios.py --profile prefix_ab --rescore-rail "demo/out/hil_smoke/pab3_*" --out docs/data/kpi_prefix_ab_20261119/rescore_r2_pab3.json` |
| `kpi_report_pab3_2026-11-19.md` / `.json`, `kpi_report_stdout.txt` | KPI report, generated at cd29a19 (clean) | `tools/kpi_report.py --campaign demo/out/hil_pab3 --rescore-rail docs/data/kpi_prefix_ab_20261119/rescore_r2_pab3.json --baseline docs/data/kpi_prefix_ab_20261117/kpi_report_pab2_2026-11-17.json --out docs/data/kpi_prefix_ab_20261119/kpi_report_pab3_2026-11-19` |

- **Baseline comparison:** it matches runs by name. pab2 and pab3 are
  different campaigns, so its "new / gone" counts compare nothing. The report
  shows 0 regressions.
- **Track videos:** `demo/out/hil_smoke/pab3_*/pab3_*_track.mp4`.

## Results

**KPIs** (33 episodes, all KPI-grade)
- P0 escapes: 0 on 2141 P0 ticks (passthrough null 0.312).
- Tick: p50 4.9 / p99 14.7 / max 18.10 ms; 0 of 15783 ticks over 100 ms.
- End-to-end (action receipt to setpoint): p99 14.8 / max 36.92 ms.
- Fail-safe, adopted reading r2: 15/15 on the labelled prefix-off arm
  (Wilson 95 % low 0.796).
- Fail-safe, reading as flown (ideal-vehicle labels): 12/15. The 3
  nfz-head-on prefix-off runs (pab3_002, _005, _007) end in RTL, the same
  pattern as pab1 and pab2; on the rail-vehicle labels they are read on the
  cell flown as long as the rail flies.

**A/B, on vs off**

| Measure | On | Off | A/A, largest in one pair |
|---|---|---|---|
| Mission success | 0.800 | 0.600 | 0.000 |
| Repairs per episode | 2.333 | 130.400 | 0 |
| Mean repair magnitude | 0.000 m/s | 9.380 m/s | n/m |
| Mean time to safe | 4.830 s | 4.833 s | 0.096 s |

**Against pab2 (47b3fde, before the clearance change)**

| Cell, prefix-on arm | pab2 | pab3 |
|---|---|---|
| corridor-drift-out | success 3/4 (pab2_011 RTL, "no operator converged") | success 4/4, 0 repairs |
| nfz-head-on | 4/4 success, 0 repairs | 4/4 success, 0 repairs |
| altitude-floor-recovery | success | 4/4 success, 0 repairs |
| altitude-ceiling-hold | RTL in both arms | RTL in both arms (off arm labelled to expect it) |
| speed-cap | success | 3/3 success, 0 repairs |

- Mission success with the prefix on rises from 0.733 to 0.800: the
  corridor-drift-out flight that ended in RTL on pab2 (pab2_011) did not
  recur. One campaign does not show which change removed it; pab2's open
  question is closed only in the sense that it was not reproduced in 4
  flights.
- The prefix-on arm repaired nothing on 4 of 5 cells; altitude-ceiling-hold
  repairs 8-14 times per run (mean translational repair magnitude
  0.000 m/s) and ends in RTL in both arms.
