# broad_REL073: the full broad sweep at release 0.7.3 (28 Nov 2026)

The grant's broad sweep profile (`experiments/profiles/broad.yaml`, ~3000
scenarios x 5 seeds) run in full on the headless backend at release 0.7.3
(c44a360, tag v0.7.3), from the clean detached worktree flight-c44a360:
2,990 cells x 5 seeds = **14,950 episodes**, every family of the scenario
library. It repeats `../broad_REL072/` at the delivered release.

- **How:** 12 shards side by side (`--shard i/12 --resume <store>/broad_REL073/ck
  --runs <store>/broad_REL073/runs`), then `--merge-only` into
  `<store>/broad_REL073/sweep.json`. Started 28 Nov 09:10 TST; shards done in
  2,473 s, total 2,592 s (43.2 min wall; 1.959 s per episode per process,
  29,286 s of episode time; `_resume` in the summary).
- **Python:** 3.10.20 (vla-real). `_manifest_common.code_revision`
  c44a360618e1, topology `headless-kinematic`, clean tree.
- **Not KPI-grade, by design:** the grant keeps broad-sweep results out of
  the final KPI numbers ("tracked but not part of the final KPI numbers").

## Figures

| Figure | Value |
|---|---|
| Episodes | 14,950 (2,990 cells x 5 seeds), 0 skipped |
| Status | 14,370 pass, 110 tracked, 0 fail, 470 known failures |
| P0 escape (Shield log) | **0.0** over the Shield-flown ticks of 14,945 Shield-on episodes (null passthrough 0.2512; Shield-off control 0.24) |
| True state | 0 Shield-on episodes flew into a P0 polygon; 10 carried in by the modelled autopilot alone (LOITER / RTL / LAND); 970 inside only because a zone appeared, moved, grew or switched on under them |
| Fail-safe labels (FSM RTL / Land) | **14,945 / 14,945 = 1.0** (0 false triggers, 0 missed; Wilson 95 % low 0.99974); 5 unlabelled; 0 censored |
| Outcomes | success 4,555, fail 2,345, RTL_triggered 8,025, Land_triggered 25 |
| Distinct KPI signatures | 286 of 14,950 episodes |

Against release 0.7.2 (`../broad_REL072/`): every one of the 14,950
episodes has the same status, outcome and KPI record (matched by scenario,
parameter cell and seed over the two run stores' `sweep.json`), and the
summary figures above are the same. The changes of 0.7.3 (the scenario
clock pinned on the rails, the Shield-matrix gate, the manifest start frame
at arming, the setpoint path in report.md) act on the flying rails and the
KPI reader, not on the headless sweep's episodes.

Per family the counts equal 0.7.2's (`merge_stdout.txt`; the table in
`../broad_REL072/README.md`).

## Files

| File | What |
|---|---|
| `broad_REL073_summary.json` | `sweep.json` without the per-episode `results` and the `_cell_rollup` (both kept with the run store, `<store>/broad_REL073/sweep.json`, 187 MB) |
| `merge_stdout.txt` | the `--merge-only` run's console output |

`<store>` is the offload drive (F:/vla_drone_offload).
