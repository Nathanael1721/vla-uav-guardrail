# smREL070: the hil smoke at release 0.7.0 (23 Nov 2026)

The smoke profile on the hil rig (AGX Orin + ArduCopter 4.5.7 SITL over
MAVROS 2) at release 0.7.0, master 21c2771 (59e19a5 plus the changes of
CHANGELOG "2026-11-23 (code)", among them the hot-apply tick fix 7e6aae2 /
a0eb767), `--hil-controls`, from the detached worktree flight-21c2771,
launched 23 Nov 18:27; 6274.9 s. 64 episodes planned, 52 missions flown (51
Shield on, 1 Shield-off control), 12 refused before take-off. All 52
KPI-grade; single_sha 52/52. It was the gate for the nightly ntFIN5
(`docs/data/kpi_ntFIN5/`): P0 escape 0, no ERROR, (r2) at least 0.98.

## Files

| File | What |
|---|---|
| `kpi_report_smREL070_2026-11-24.md` / `.json` | `tools/kpi_report.py --campaign demo/out/hil_smREL070 --rescore-rail docs/data/kpi_smREL070/rescore_r2_smREL070.json --baseline docs/data/kpi_smFIN2/kpi_report_smFIN2_2026-11-23.json`, from a clean tree at 4a3b207 |
| `kpi_report_stdout.txt` | that command's console output |
| `rescore_r2_smREL070.json` / `.txt` | `experiments/sweep_scenarios.py --profile smoke --rescore-rail` over the 52 runs |
| `replay_at_revision_smREL070.json` / `.per_run.jsonl.gz` | `tools/replay_at_revision.py --root demo/out/hil_smoke --only smREL070_` |

## Figures, against smFIN2

| Figure | smFIN2 (59e19a5) | smREL070 (21c2771) |
|---|---|---|
| P0 escape rate, Shield on | 0 (2239 P0-violation ticks) | **0** (2299 P0-violation ticks, 51 flights; control 0.65, null 0.3166) |
| Fail-safe (r2) | 51/51 = 1.000, Wilson 95 % low 0.930 | **50/50 = 1.000**, Wilson 95 % low 0.929 (1 censored) |
| (r) / (a) | 50/51 / 45/51 | 49/50 = 0.980 / 44/50 = 0.880 |
| `tick_ms` max | 72.91 ms (p99 25.5) | **57.38 ms** (p99 24.6), 0 of 16,002 ticks over 100 ms |
| `e2e_ms` / `e2e_with_wait_ms` max | 73.01 ms / - | 57.49 ms / 57.49 ms (p99 24.7) |
| Mission success, Shield on | 24/51 | 24/51 |
| Repair success | 98.3 % (2329/2369) | 98.1 % (2358/2404) |
| Mean time to safe | - | 3.32 s (max 5.30 s; 4 never recovered) |
| Replay at the flown revision | 43 identical, 9 last digits, 0 not replayable | 43 identical, 9 last digits (7 through the body-frame rotation, 2 plain; at most 3.0e-14), 0 not replayable |

Left out of (r2): `smREL070_033_corridor-swap-shifted`, censored (ended still
holding in Loiter), as in every campaign since ntFIN3.

## How the files were made

As for ntFIN4: re-score and replay in the lab checkout after the flight, the
KPI report from a clean tree after the inputs were committed; local paths
repo-relative, the scratchpad written `<scratchpad>`.
