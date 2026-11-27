# broad_REL072: the full broad sweep at release 0.7.2 (27-28 Nov 2026)

The grant's broad sweep profile (`experiments/profiles/broad.yaml`, ~3000
scenarios x 5 seeds) run in full on the headless backend at release 0.7.2
(418b151, tag v0.7.2), from the clean detached worktree flight-418b151:
2,990 cells x 5 seeds = **14,950 episodes**, every family of the scenario
library.

- **How:** 12 shards side by side (`--shard i/12 --resume <store>/broad_REL072/ck
  --runs <store>/broad_REL072/runs`), then `--merge-only` into
  `<store>/broad_REL072/sweep.json`. Started 27 Nov 23:16 TST, shards done in
  2,506 s, total 2,628 s (43.8 min wall; 1.98 s per episode per process,
  29,570 s of episode time).
- **Python:** 3.10.20 (vla-real). `_manifest_common.code_revision`
  418b1517da9f, topology `headless-kinematic`.
- **Not KPI-grade, by design:** the grant keeps broad-sweep results out of
  the final KPI numbers ("tracked but not part of the final KPI numbers");
  the KPI figure of record stays the hil nightly ntFIN5 (`../kpi_ntFIN5/`).

## Figures

| Figure | Value |
|---|---|
| Episodes | 14,950 (2,990 cells x 5 seeds), 0 skipped |
| Status | 14,370 pass, 110 tracked, 0 fail, 470 known failures |
| P0 escape (Shield log) | **0.0** over the Shield-flown ticks of 14,945 Shield-on episodes (null passthrough 0.2512; Shield-off control 0.24) |
| True state | 0 Shield-on episodes flew into a P0 polygon; 10 carried in by the modelled autopilot alone (LOITER / RTL / LAND); 970 inside only because a zone appeared, moved, grew or switched on under them |
| Fail-safe labels (FSM RTL / Land) | **14,945 / 14,945 = 1.0** (0 false triggers, 0 missed; Wilson 95 % low 0.99974); 5 unlabelled; 0 censored |
| Outcomes | success 4,555, fail 2,345, RTL_triggered 8,025, Land_triggered 25 |
| Distinct KPI signatures | 286 of 14,950 episodes (14,840 deterministic episodes repeat on another seed by construction) |

Per family (from `merge_stdout.txt`):

| Family | Episodes | Pass | Tracked | Known failures |
|---|---|---|---|---|
| dynamic_nfz_movement | 7,840 | 7,840 | 0 | 0 |
| subject_standoff | 2,775 | 2,310 | 0 | 465 |
| three_simultaneous_violations | 2,760 | 2,760 | 0 | 0 |
| failsafe_escalation | 940 | 940 | 0 | 0 |
| high_speed_near_edge | 180 | 180 | 0 | 0 |
| sensor_degradation | 140 | 60 | 80 | 0 |
| time_window_switch | 85 | 85 | 0 | 0 |
| corridor_keep_in | 70 | 70 | 0 | 0 |
| altitude_envelope | 45 | 45 | 0 | 0 |
| wind_disturbance | 40 | 10 | 30 | 0 |
| radius_scaling | 25 | 25 | 0 | 0 |
| static_nfz | 15 | 15 | 0 | 0 |
| corridor_swap | 15 | 10 | 0 | 5 |
| time_window_static | 10 | 10 | 0 | 0 |
| kinematic_envelope | 5 | 5 | 0 | 0 |
| action_contract | 5 | 5 | 0 | 0 |

"Known failures" are the scenarios' own recorded open behaviours (each
scenario's `description` names it); "tracked" cells are scored but carry no
pass/fail gate.

## Files

| File | What |
|---|---|
| `broad_REL072_summary.json` | `sweep.json` without the per-episode `results` and the 12 MB `_cell_rollup` (both kept with the run store, `<store>/broad_REL072/sweep.json`, 186 MB) |
| `merge_stdout.txt` | the `--merge-only` run's console output |

`<store>` is the offload drive (F:/vla_drone_offload).
