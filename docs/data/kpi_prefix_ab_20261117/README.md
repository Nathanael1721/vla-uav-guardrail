# Prefix A/B on hil, campaign pab2 (17 Nov 2026)

The re-fly that pab1 (docs/data/kpi_prefix_ab_20261116/) owed. Same profile: 33 missions of `prefix_ab`, which is 5 cells x 3 seeds x prefix on/off, plus 3 A/A repeats.

- **Commit:** 47b3fde, which includes the PrefixConsumer path-test fix 554e35d.
- **Rig (hil):** Project AirSim + ArduPilot SITL on the desktop; Shield and pilot on the AGX Orin.
- **Pilot:** guardrail.prefix_consumer.PrefixConsumer, a hand-written CSP-consuming adapter, not a learned VLA. Part of any on/off delta holds by construction, because the adapter reads the geometry the Shield enforces.

## Files

| File | What it is | Command |
|---|---|---|
| `rescore_r2_pab2.json` / `.txt` | Rail re-score | `experiments/sweep_scenarios.py --rescore-rail "demo/out/hil_smoke/pab2_*" --profile prefix_ab` |
| `kpi_report_pab2_2026-11-17.md` / `.json`, `kpi_report_stdout.txt` | KPI report | `tools/kpi_report.py --campaign demo/out/hil_pab2 --rescore-rail rescore_r2_pab2.json --baseline ../kpi_prefix_ab_20261116/kpi_report_pab1_2026-11-16.json` |

- **Re-score:** the 18 prefix-on runs carry `labels_not_applied`.
- **Baseline comparison:** it matches runs by name. pab1 and pab2 are different campaigns, so its "new / gone" counts compare nothing. The report shows 0 regressions.
- **Track videos:** `demo/out/hil_smoke/pab2_*/pab2_*_track.mp4`.

## Results

**KPIs**
- 33 episodes, all KPI-grade.
- P0 escapes: 0 on 2172 P0 ticks.
- Tick: p50 4.8 / p99 14.4 / max 26.29 ms; 0 of 15464 ticks over 100 ms.
- End-to-end: max 39.45 ms.
- Fail-safe, adopted reading r2: 15/15 on the labelled prefix-off arm (Wilson 95 % low 0.796).
- Fail-safe, reading as flown: 12/15. The 3 nfz-head-on prefix-off runs are the pattern pab1 and ntFIN2 show.

**A/B, on vs off**

| Measure | On | Off | A/A, largest in one pair |
|---|---|---|---|
| Mission success | 0.733 | 0.600 | 0.000 |
| Repairs per episode | 3.133 | 130.600 | 3 |
| Mean repair magnitude | 0.505 m/s | 9.370 m/s | – |
| Mean time to safe | 4.833 s | 4.900 s | 0.002 s |

**Against pab1 (flown before the fix)**

| Cell, prefix-on arm | pab1 | pab2 |
|---|---|---|
| nfz-head-on | success, but 9-16 repairs per run | 4/4 success, 0 repairs |
| corridor-drift-out | RTL 3/3 | success 3/4 |
| altitude-ceiling-hold | – | RTL in both arms; the off arm is labelled to expect it |

## Open: one corridor-drift-out prefix-on flight still ended in RTL

The run is pab2_011 (seed 1002).

- **Why it differs from pab1:** pab1's escalation was "repair over theta". pab2_011's is "no operator converged".
- **Chain:**
  - 24.8 s: X1 to Brake.
  - 25.5 s: G3 to Loiter.
  - 33.1 s: G4 to RTL ("N-in-T persists: 3 onsets in 5 s since Loiter entry").
  - 45.9 s: home reached.
- **A related run:** pab2_010 (the A/A repeat of seed 1001) entered Loiter twice on the same message, then recovered and succeeded.
- **Status:** the cause is not established yet. It is investigated headless from these logs. The Shield is unchanged.
