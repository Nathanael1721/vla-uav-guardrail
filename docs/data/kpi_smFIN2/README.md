# smFIN2: the hil smoke at 59e19a5 (22 Nov 2026)

The smoke profile on the hil rig at master 59e19a5 (71aabb3 plus the eleven
changes of CHANGELOG "2026-11-22 (code)"), `--hil-controls`, from the detached
worktree flight-59e19a5, launched 22 Nov 13:56 just before the nightly ntFIN4
(`docs/data/kpi_ntFIN4/`); 6345.9 s. 64 episodes planned, 52 missions flown
(51 Shield on, 1 Shield-off control), 12 refused before take-off (three fewer
than before: the GPS-noise / latency cells now fly on the rail). All 52
KPI-grade; single_sha 52/52.

## Files

| File | What |
|---|---|
| `kpi_report_smFIN2_2026-11-23.md` / `.json` | `tools/kpi_report.py --campaign demo/out/hil_smFIN2 --rescore-rail docs/data/kpi_smFIN2/rescore_r2_smFIN2.json --baseline docs/data/kpi_smfin1120/kpi_report_smfin1120_2026-11-21.json`, from a clean tree at 2957677 |
| `rescore_r2_smFIN2.json` / `.txt` | `experiments/sweep_scenarios.py --profile smoke --rescore-rail` over the 52 runs |
| `replay_at_revision_smFIN2.json` / `.per_run.jsonl.gz` | `tools/replay_at_revision.py --root demo/out/hil_smoke --only smFIN2_` |

## Figures, against smfin1120 and rel060

| Figure | rel060 (a85dc91) | smfin1120 (71aabb3) | smFIN2 (59e19a5) |
|---|---|---|---|
| P0 escape rate, Shield on | 0 (2220 P0 ticks) | 0 (2201) | **0** (2239 P0-violation ticks, 51 flights; control 0.65, null 0.3099) |
| Fail-safe (r2) | 48/48 | 46/47 (1 false trigger, 1 censored) | **51/51 = 1.000**, Wilson 95 % low 0.930 |
| (r) / (a) | 47/48 / 42/48 | 45/47 / 40/47 | 50/51 / 45/51 |
| `tick_ms` / `e2e_ms` max | 70.89 / 70.98 ms | 78.84 / 78.93 ms | 72.91 / 73.01 ms (p99 25.5 / 25.6), 0 of 15,662 ticks over 100 ms |
| Mission success, Shield on | 22/48 | 22/48 | 24/51 |
| Repair success | 98.2 % | 98.0 % | 98.3 % (2329/2369) |
| Replay at the flown revision | 37 identical, 10 last digits, 2 not replayable | 33 identical, 14 last digits, 2 not replayable | 43 identical, 9 last digits (at most 8.9e-16), 0 not replayable |

dyn-nfz-rotate-spin@3, smfin1120's false trigger, flew without a fail-safe.
failsafe-rtl-zone ended in Land (the RTL watch finds the leg home through the
zone), which its outcome gate, written for RTL, fails; the fail-safe is
correct either way.

## How the files were made

As for ntFIN4: re-score and replay in the lab checkout after the flight (the
replay with a436d32, which rebuilds the pose the Shield judged under GPS
noise), the KPI report from a clean tree after the inputs were committed;
local paths repo-relative.
