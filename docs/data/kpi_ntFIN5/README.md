# ntFIN5: the KPI-grade nightly at release 0.7.0 (23-24 Nov 2026)

The nightly profile on the hil rig (AGX Orin + ArduCopter 4.5.7 SITL over
MAVROS 2) at release 0.7.0, master 21c2771: 59e19a5 (ntFIN4) plus the changes
of CHANGELOG "2026-11-23 (code)", among them the hot-apply tick fix (7e6aae2,
review fixes a0eb767) that answers ntFIN4's one tick over 100 ms.
`--hil-controls`, from the detached worktree flight-21c2771, launched 23 Nov
20:12 after its gate, the smoke smREL070 (`docs/data/kpi_smREL070/`);
77,356.0 s (21.5 h). 744 episodes planned, 654 missions flown (651 Shield on,
3 Shield-off controls), 90 refused before take-off (as in ntFIN4: paraphrase
and prefix arms the stub pilot cannot read, and the stressors the hil rail
cannot realise).

All 654 KPI-grade; single_sha 654/654 (companion and desktop 21c277170001,
ArduCopter V4.5.7 2a3dc4b7).

## Files

| File | What |
|---|---|
| `kpi_report_ntFIN5_2026-11-24.md` / `.json` | `tools/kpi_report.py --campaign demo/out/hil_ntFIN5 --rescore-rail docs/data/kpi_ntFIN5/rescore_r2_ntFIN5.json --baseline docs/data/kpi_ntFIN4/kpi_report_ntFIN4_2026-11-23.json`, made from a clean tree at b171c57 |
| `kpi_report_stdout.txt` | that command's console output |
| `rescore_r2_ntFIN5.json` / `.txt` | `experiments/sweep_scenarios.py --profile nightly --rescore-rail` over the 654 runs |
| `replay_at_revision_ntFIN5.json` / `.per_run.jsonl.gz` | `tools/replay_at_revision.py --root demo/out/hil_smoke --only ntFIN5_` |

## Figures, against ntFIN4

| Figure | ntFIN4 (59e19a5) | ntFIN5 (21c2771) |
|---|---|---|
| KPI-grade missions | 653 (650 Shield on, 3 controls) | **654** (651 Shield on, 3 controls) |
| P0 violation escape rate, Shield on | 0 on 17,078 P0-violation ticks | **0** on 17,194 P0-violation ticks |
| Shield-off control / passthrough null | 0.6500 / 0.1935 | 0.6500 / 0.1938 |
| Fail-safe trigger correctness, (r2) adopted | 648/648 = 1.000, Wilson 95 % low 0.994 | **649/649 = 1.000, Wilson 95 % low 0.994** |
| Same, as before the GeoFence backstop rule | 630/630 | 631/631 |
| Same, backstop runs counted as not triggered | 630/648 = 0.972 | 631/649 = 0.972 |
| (r) / (a) | 636/648 = 0.981 / 573/648 = 0.884 | 637/649 = 0.982 / 574/649 = 0.884 |
| Triggered by the GeoFence backstop | 18 (all expected, all correct) | 18 |
| Left out of (r2) | 2 (censored in Loiter) | 2 (censored in Loiter: ntFIN5_373, ntFIN5_374 corridor-swap-shifted) |
| `tick_ms` (hil) | p99 28.4 / max 105.50 ms, 1 of 183,909 over 100 ms | p99 28.3 / **max 62.08 ms, 0 of 183,961 over 100 ms** |
| `e2e_ms` (hil) | p99 28.5 / max 105.60 ms, 1 over | p99 28.4 / max 62.18 ms, 0 over |
| `e2e_with_wait_ms` (hil, the wait behind post-setpoint work counted) | - | p99 28.4 / max 62.18 ms, 0 over |
| Mission success, Shield on | 287/650 | 288/651 |
| Repair success | 98.1 % (27,097/27,615) | 98.2 % (27,206/27,716) |
| Mean time to safe | 3.39 s (max 9.70 s; 95 never recovered) | 3.49 s (max 9.90 s; 94 never recovered) |
| Repairs per Shield-on mission | 50.2 | 50.2 |
| single_sha | 653/653 | 654/654 |

Verdicts: P0 escape rate 0 (meets); fail-safe trigger correctness 1.000
(meets 99 % as the point estimate and at 95 % confidence); the monitor tick
budget of 100 ms end to end is met on every tick, with the wait behind the
deferred post-setpoint work counted (`e2e_with_wait_ms`, an upper bound).

The report's "against the baseline" section matches episodes by run name, so
it lists every run as new or gone; its per-family table is the comparison.

## Replay at the flown revision

654 runs: 352 identical, 301 last float digits (109 plain, 192 through the
body-frame rotation; at most 3.7e-14), 0 not replayable, 1 differs in a
repair's pass count (`ntFIN5_306_dyn-nfz-rotate-spin`: a GeofenceSlide
convergence test on the last float digit, the same emitted action to
6.9e-14; as ntFIN4_304 and ntFIN3_306).

## How the files were made

The re-score and the replay ran in the lab checkout after the campaign; the
KPI report was made from a clean tree at b171c57 after the inputs were
committed. Local absolute paths in stored records are repo-relative, and the
scratchpad is written `<scratchpad>`.
