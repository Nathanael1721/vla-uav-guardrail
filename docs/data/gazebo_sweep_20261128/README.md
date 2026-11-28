# Gazebo Harmonic functional sweep at release 0.7.2, 27-28 Nov 2026 (WP4-04)

The full `gazebo_functional` profile flown again on the Gazebo functional
rail, at release 0.7.2: 44 cells x 3 seeds (1001-1003) = 132 missions, every
family the Gazebo rail admits, each mission flown end to end on Gazebo's
physics behind ArduPilot SITL's JSON backend, with the stub pilot through the
ROS 2 Shield node and MAVROS 2. Grant (02-implementation/stress-testing.md):
"Where it runs (Gazebo Harmonic for functional sweeps, Project AirSim for
perception stress)". It replaces the 24-25 Nov sweep at 0.7.0
(`../gazebo_sweep_20261124/`) as the rail's record.

- **Code:** release 0.7.2, 418b151, from the detached worktree
  `.claude/worktrees/flight-418b151` (clean). **Every flown run's own
  `manifest.json` records `code_revision` 418b1517da9f** (131 of 131),
  where the 24 Nov runs recorded `unversioned`; `sweep.json` records the same
  revision.
- **Command:** `python experiments/sweep_scenarios.py --profile
  gazebo_functional --backend sitl --out demo/out/gzsweep_rel072/sweep.json`
  (17,613 s), then the re-flights below.
- **Rail:** `gazebo-harmonic`, world `iris_runway.sdf`, SITL model JSON with
  `gazebo-iris.parm`, ArduCopter V4.5.7; topology `dev`, sim_speedup 1.0;
  pilot `guardrail.vla_stub.GotoPilot` (or the cell's constant action).
- **Not KPI-grade, by design:** every mission's record is `functional_only`
  (KPI-grade runs come from hil only), so `guardrail.manifest.is_kpi_grade`
  refuses it. The figures below are functional evidence, never pooled with
  the hil KPI figures.

## Re-flown missions

3 of the 132 missions ended in ERROR ("the rail wrote no flight_log.jsonl"),
each on the rail's known home-check refusal: the run's `events.jsonl` ends in
a `home_check` with `ok: false` (no `/mavros/home_position/home` within the
node's 90 s wait, 30 home requests), so the node refused to arm; no flight,
no Shield decision. Each was re-flown on its own seed with `--seed <n>
--only <cell>` (`refly/`): corridor-curfew-out-of-hours and standoff-approach
(seed 1003) flew on the first try and passed;
standoff-approach-angles@offset_m=12,subject_class=pedestrian,variant=0 seed
1003 was refused the same way on all three re-flights. **131 of 132 missions
flown**; the other two seeds of that cell flew.

## Files

| File | What it is |
|---|---|
| `sweep.json` / `sweep_stdout.txt` | the sweep's record and console output (132 planned, 3 ERROR) |
| `kpi_report_sweep.md` | the per-family report the sweep wrote |
| `refly/refly_*.json`, `refly/refly_*_stdout.txt` | the re-flights' records and console output, the refused attempts included |
| `rescore_rail_gz.json` / `rescore_rail_stdout.txt` | `python experiments/sweep_scenarios.py --profile gazebo_functional --backend sitl --rescore-rail "<worktree>/demo/out/gzsweep_*" --out docs/data/gazebo_sweep_20261128/rescore_rail_gz.json` over the 131 flown run folders |
| `kpi_report_gz_2026-11-28.md` / `.json`, `kpi_report_stdout.txt` | `python tools/kpi_report.py --runs '.claude/worktrees/flight-418b151/demo/out/gzsweep_*' --no-sweep --rescore-rail docs/data/gazebo_sweep_20261128/rescore_rail_gz.json`, from a clean tree at fdc8970 after these inputs were committed |

`<worktree>` is `.claude/worktrees/flight-418b151`; WSL home paths are
written `~/`. Run folders (worktree flight-418b151, gitignored):
`demo/out/gzsweep_<cell>--seed<n>/` and the episode bundles
`demo/out/stress_gazebo_functional/episode-*` with the grant's six names
(131).

## Result

131 missions: 128 Shield on, 3 Shield-off controls (nfz-head-on-control).

| Figure | Value |
|---|---|
| Gates (ideal-vehicle labels, the sweep's own) | 106 pass, 3 tracked, 16 fail, 6 known failures |
| P0 escape, Shield on | **0** over the Shield-flown ticks of 128 Shield-on missions |
| P0 escape, Shield-off control | family rate 0.659 (3 of 3 missions) |
| Outcomes | success 65, fail 9, RTL 47, Land 10 |
| Fail-safe trigger, ideal-vehicle labels | 119/128 (9 false triggers, 0 missed) |
| Repair success | 9,448/9,551 measured repair ticks converged (98.9 %), 104 missions |
| `tick_ms` (desktop, dev topology) | p50 1.66 / p99 6.48 / max 25.06 ms over 35,118 ticks, 0 over 100 ms |
| `e2e_with_wait_ms` (desktop) | p99 6.71 / max 97.36 ms over 35,049 ticks, 0 over 100 ms |

The timing is the desktop's, not the Orin's: a baseline, not the grant's
tick-budget evidence (that is the hil campaigns on the AGX Orin).

The 16 failed gates are on the same cells as the 0.7.0 sweep and fail the
same gates on the hil rig: nfz-head-on and dyn-nfz-rotate-step (3 each:
fail-safe labelled not expected, triggered; the adopted rail-vehicle reading
(r2) expects that fail-safe), time-window-opens-on-vehicle at both speeds (6:
time to safe 3.7-3.8 s > 2.7 s, 16-17 ticks in the polygon > 15),
time-window-opens-on-sideways-pilot (3: 18-137 ticks > 7, time to safe
4.0-4.1 s > 2.9 s, Land on all 3, seed 1002's time to safe censored) and failsafe-rtl-zone seed 1002 (Land instead
of RTL). The 6 known failures (standoff-wedge x3, corridor-swap-shifted x3)
and the 3 tracked runs (gps-noise-beyond-margin) are the library's
documented open cells.

The Shield-matrix line of the KPI report reads near-edge-at-12mps 0/2
non-vacuous: release 0.7.2 predates the scenario clock pin of 0.7.3 that
puts that pass beside an active zone. The release gate reads the hil smoke
of each release (`../kpi_smREL073/`: 2/2 non-vacuous); this profile does
not include the three-simultaneous-violations cell.
