# ntFIN3: the KPI-grade nightly at the final revision 71aabb3 (21 Nov 2026)

The nightly profile flown on the hil rig (AGX Orin + ArduCopter 4.5.7 SITL over
MAVROS 2) at master 71aabb3, the revision the final report describes, with
`--hil-controls`, from the detached worktree flight-71aabb3. Launched 21 Nov
01:04, 72,478.6 s (20.1 h). 744 episodes planned, 594 missions flown (591
Shield on, 3 Shield-off controls), 150 refused before take-off (paraphrase and
prefix arms the rail's stub pilot cannot read, and stressors the hil rail
cannot realise, as in ntFIN2). All 594 KPI-grade.

The KPI report of record until now, ntFIN2 (`docs/data/kpi_ntFIN2_20261117_delivery/`),
flew 116214f on 13 Nov. Since then the ROS 2 rail's default chain gained
PathRepair (17 Nov), the clearance horizon a lagging vehicle needs (19 Nov),
the RTL watch (14 Nov) and the body-frame hand-over (19 Nov). ntFIN3 measures
the code that is delivered.

## Files

| File | What |
|---|---|
| `kpi_report_ntFIN3_2026-11-21.md` / `.json` | `tools/kpi_report.py --campaign demo/out/hil_ntFIN3 --rescore-rail docs/data/kpi_ntFIN3/rescore_r2_ntFIN3.json --baseline docs/data/kpi_ntFIN2_20261117_delivery/kpi_report_ntFIN2_2026-11-17.json` |
| `rescore_r2_ntFIN3.json` / `.txt` | `experiments/sweep_scenarios.py --profile nightly --rescore-rail` over the 594 runs |
| `replay_at_revision_ntFIN3.json` / `.per_run.jsonl.gz` | `tools/replay_at_revision.py --root demo/out/hil_smoke --only ntFIN3_` |

## Figures, against ntFIN2

| Figure | ntFIN2 (116214f, regraded 17 Nov) | ntFIN3 (71aabb3) |
|---|---|---|
| KPI-grade missions | 588 (585 Shield on, 3 controls) | 594 (591 Shield on, 3 controls) |
| P0 violation escape rate, Shield on | 0 on 16,746 P0-violation ticks | **0** on 17,457 P0-violation ticks |
| Shield-off control / passthrough null | 0.6375 / 0.2131 | 0.6458 / 0.2220 |
| Fail-safe trigger correctness, (r2) adopted | 583/584 = 0.998, Wilson 95 % low 0.990 (meets at 95 % too) | **587/589 = 0.997**, Wilson 95 % low 0.988 |
| Same, as before the GeoFence backstop rule | 565/566 | 569/571 |
| Same, backstop runs counted as not triggered | 565/584 = 0.967 | 569/589 = 0.966 |
| (r) / (a) | 571/584 / 513/584 | 575/589 = 0.976 / 516/589 = 0.876 |
| The campaign's own labels | - | 528/589 = 0.896 |
| Triggered by the GeoFence backstop | 18 (all correct) | 18 (all dyn-nfz-spawn-ahead, all expected, all correct) |
| Left out of (r2) | 1 | 2 (censored in Loiter: rule-activated-mid-run) |
| `tick_ms` (hil) | p99 29.5 / max 74.06 ms | p99 28.9 / max 74.52 ms, 0 of 174,186 ticks over 100 ms |
| `e2e_ms` (hil) | p99 30.3 / max 121.93 ms, 13 of 172,724 ticks over 100 ms | p99 29.0 / max 74.62 ms, **0** of 174,186 ticks over 100 ms |
| Mission success, Shield on | 252/585 | 252/591 |
| Repair success | 98.1 % (26,508/27,011) | 98.1 % (26,747/27,258) |
| Mean time to safe | 3.35 s (172 spells, max 10.90 s; 75 never recovered) | 3.38 s (170 spells, max 9.50 s; 95 never recovered) |
| Repairs per Shield-on mission | 54.7 | 54.4 |

The verdicts: P0 escape rate 0 (meets); fail-safe trigger correctness 0.997
(meets 99 % as the point estimate, the grant's figure; the Wilson 95 % lower
bound 0.988 does not clear 0.99, where ntFIN2's 0.990 did by 0.0004);
`tick_ms` and `e2e_ms` within 100 ms on every tick.

The report's "against the baseline" section lists 409 new failures and 411
gone because it matches episodes by run name (ntFIN2_NNN never matches
ntFIN3_NNN); its per-family figure table is the comparison.

## The two errors and the two left out (r2)

- Missed: `ntFIN3_164` and `ntFIN3_168` dyn-nfz-spawn-ahead (label E5), as
  ntFIN2_177 was in ntFIN2.
- Left out, censored in Loiter: `ntFIN3_352` and `ntFIN3_354`
  rule-activated-mid-run@at=2 (the Loiter 3.2 s and 3.9 s into its 30 s
  operator timeout when the flight ended). Were both errors, 587/591 = 0.993.

## What flew differently from ntFIN2

- failsafe-rtl-zone (ahead 6 m and 12 m) now ends in **Land** on most seeds
  where it ended in RTL: the node's RTL watch (14 Nov) finds the leg home
  through the zone and lands (G6). A fail-safe triggers either way, so the
  fail-safe figure is unchanged; the scenario's outcome gate, written for
  RTL, fails them ("labelled RTL_triggered, flew Land_triggered").
- rotate-spin: no false trigger on any of its seeds (the smoke smfin1120's
  one, `docs/data/kpi_smfin1120/`, did not recur).

## Replay at the flown revision

594 runs: 277 identical; 292 differ only in the last float digits (111 plain,
181 through the body-frame rotation; at most 6.9e-14); 24 not replayable
(gps-dropout: the pipe the Shield dead-reckons on is not logged per tick); 1
differs in a repair's pass count: `ntFIN3_306_dyn-nfz-rotate-spin`, on 4
ticks the flight ran GeofenceSlide twice and the replay once (or the
reverse), with the same emitted action (to 6.9e-14) and the same Brake. The
convergence test there sits on the last float digit, and the Orin
(aarch64) and the desktop (x86-64) round it differently. The replay ran on
the desktop.

The stack revisions (ARCH-31, `tools/kpi_report.py` since 22 Nov): single_sha
true on 594 of 594 KPI-grade runs.

## How the files were made

The re-score and the replay ran in the lab checkout at 71aabb3 after the
campaign (the replay again on 21 Nov with `--root demo/out/hil_smoke --only
ntFIN3_`, as 594 run folders do not fit one command line); the KPI report was
made again on 22 Nov from a clean tree at f761387, after the inputs were
committed. Local absolute paths in the stored records are written
repo-relative, and the scratchpad as `<scratchpad>`.
