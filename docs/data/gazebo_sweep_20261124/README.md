# Gazebo Harmonic functional sweep at release 0.7.0, 24-25 Nov 2026 (WP4-04, ARCH-16)

The full `gazebo_functional` profile flown on the Gazebo functional rail at
release 0.7.0: 44 cells x 3 seeds (1001-1003) = 132 missions, every family
the Gazebo rail admits, each mission flown end to end on Gazebo's physics
behind ArduPilot SITL's JSON backend, with the stub pilot through the ROS 2
Shield node and MAVROS 2. Grant (02-implementation/stress-testing.md):
"Where it runs (Gazebo Harmonic for functional sweeps, Project AirSim for
perception stress)". The first sweep (20 Nov, `../gazebo_sweep_20261120/`)
flew 8 missions on one seed.

- **Code:** release 0.7.0, 21c2771, from the detached worktree
  `.claude/worktrees/flight-21c2771` (clean). `sweep.json` records
  `code_revision` 21c277170001. Each run's own `manifest.json` records
  `unversioned` (see Open: git in WSL cannot read a Windows worktree's
  `.git` file), so the revision of these runs is the sweep's record, not the
  run manifests'.
- **Command:** `python experiments/sweep_scenarios.py --profile
  gazebo_functional --backend sitl --out demo/out/gzsweep_rel070/sweep.json`,
  launched 24 Nov 19:59 after the release's nightly ntFIN5; 17,132 s.
- **Rail (each run's physics.json):** `gazebo-harmonic`, gz_sim 8.11.0,
  world `iris_runway.sdf`, SITL model JSON with `gazebo-iris.parm`,
  ArduCopter V4.5.7 (2a3dc4b7); topology `dev`, sim_speedup 1.0; pilot
  `guardrail.vla_stub.GotoPilot` (or the cell's constant action).
- **Not KPI-grade, by design:** every mission's record is `functional_only`
  (the lab's choice of 17 Nov: KPI-grade runs come from hil only), so
  `guardrail.manifest.is_kpi_grade` refuses it. The figures below are
  functional evidence and are never pooled with the hil KPI figures
  (ntFIN5, `../kpi_ntFIN5/`).

## Re-flown missions

The sweep flew 132 missions; 6 ended in ERROR ("the rail wrote no
flight_log.jsonl"). Each of those runs' `events.jsonl` ends in a
`home_check` with `ok: false`: MAVROS never received the autopilot's home
(`/mavros/home_position/home`) within the node's wait, although the EKF was
using GPS, so the node refused to arm. In passing runs the home arrived
about 8 s after "EKF3 IMU0 is using GPS". No flight, no Shield decision.

The 6 cells were re-flown on their own seed with `--seed <n> --only <cell>`
(`refly/refly_stdout.txt`); 4 errored again in the same way. A second pass
re-flew each remaining cell up to three times (`refly/refly2_stdout.txt`):
3 flew on the first try, nfz-head-on seed 1003 on the second. In all, 143
attempts gave 132 flown missions and 11 home-check refusals. The 11
refused run folders are kept, unscored, beside the runs
(`demo/out/_gz_errors_rel070/`, listed in `refly/errored_run_folders.txt`).
Not seen on the hil rig: ntFIN5 flew 654 missions with no ERROR.

## Files

| File | What it is |
|---|---|
| `sweep.json` / `sweep_stdout.txt` | the sweep's record and console output (132 planned, 6 ERROR) |
| `kpi_report_sweep.md` | the per-family report the sweep wrote (126 scored) |
| `refly/refly_*.json`, `refly/refly*_stdout.txt` | the re-flights' records and console output, including the attempts that errored |
| `refly/errored_run_folders.txt` | the 11 refused run folders kept aside |
| `rescore_rail_gz.json` / `rescore_rail_stdout.txt` | `python experiments/sweep_scenarios.py --profile gazebo_functional --backend sitl --rescore-rail "<worktree>/demo/out/gzsweep_*" --out docs/data/gazebo_sweep_20261124/rescore_rail_gz.json` over the final 132 run folders, at 0d3dcda |
| `kpi_report_gz_<date>.md` / `.json` | `python tools/kpi_report.py --runs '.claude/worktrees/flight-21c2771/demo/out/gzsweep_*' --no-sweep --rescore-rail docs/data/gazebo_sweep_20261124/rescore_rail_gz.json`, from a clean tree after these inputs were committed |

Run folders (worktree flight-21c2771, gitignored):
`demo/out/gzsweep_<cell>--seed<n>/` (flight log, rosbag2 `bag/`, events,
`physics.json`, signed `<tag>.replay.tar.gz`, trajectory plot, flight-log
replay video `<tag>_track.mp4` rendered after the sweep by
tools/render_track_video.py), and
`demo/out/stress_gazebo_functional/episode-2026-11-24T11-59-15Z--<cell>--seed<n>/`
with the grant's six names (manifest.json, policy_bundle.tar.gz,
episode.bag2, shield_audit.jsonl, harness_events.jsonl, kpi.json).

## Result

132 missions: 129 Shield on, 3 Shield-off controls (nfz-head-on-control).

| Figure | Value |
|---|---|
| Gates (ideal-vehicle labels, the sweep's own) | 107 pass, 3 tracked, 16 fail, 6 known failures |
| P0 escape, Shield on | **0** on 6,419 P0-violation ticks, 129 missions; 0 missions with a P0 escape |
| P0 escape, Shield-off control | 158 escapes, family rate 0.656 (3 of 3 missions) |
| Outcomes, Shield on | success 57, fail 15, RTL 47, Land 10 |
| Fail-safe trigger, ideal-vehicle labels | 120/129 = 0.930 (9 false triggers, 0 missed) |
| Repair success | 9,678/9,779 measured repair ticks converged (99.0 %), 103 missions |
| `tick_ms` (desktop, dev topology) | p50 1.65 / p99 6.66 / max 24.04 ms over 39,273 ticks, 0 over 100 ms |
| `e2e_with_wait_ms` (desktop) | p99 6.74 / max 83.51 ms over 39,251 ticks, 0 over 100 ms |
| GUIDED hold handed to BRAKE (`hold_forced_handover`) | 5 of 132 missions |

The timing is the desktop's, not the Orin's: a baseline, not the grant's
tick-budget evidence (that is ntFIN5 on the AGX Orin).

### The 16 failed gates, against the hil rig

Every failed gate is on a cell that fails the same gate on the hil rig in
ntFIN5 (`../kpi_ntFIN5/rescore_r2_ntFIN5.json`, the same release):

| Cell | Gazebo (3 seeds) | ntFIN5, hil (same cell) |
|---|---|---|
| nfz-head-on | 3 fail: fail-safe labelled not expected, triggered (RTL) | 3 fail, the same gate |
| dyn-nfz-rotate-step | 3 fail: the same | 3 fail, the same gate |
| time-window-opens-on-vehicle (2 speeds) | 6 fail: time to safe 3.79-3.80 s > 2.7 s; 4 also 16 ticks in the polygon > 15 | 6 fail: 3.8-3.9 s > 2.7 s, 16-17 ticks > 15 |
| time-window-opens-on-sideways-pilot | 3 fail: 18-19 ticks > 7, time to safe 4.0-4.1 s > 2.9 s, and the run ended in Land (ended_safe false) | 3 fail: 18-20 ticks > 7, 4.1 s > 2.9 s; Land on all 3 |
| failsafe-rtl-zone (12 m ahead) | seed 1002: Land instead of RTL | seed 1002: Land instead of RTL |

The fail-safe gates fail on ideal-vehicle labels. The adopted rail-vehicle
reading (r2, `docs/data/failsafe_labels_rail.json`) expects a fail-safe on
nfz-head-on and dyn-nfz-rotate-step, so there the trigger is correct. The
re-score does not apply r2 to these runs (all 129 "not accepted": a
sitl-backend run writes no `scenario_score.json`), and it should not be
applied unexamined: r2's vehicle was fitted to the hil rig's Project
AirSim quad, not to Gazebo's iris. The time-window cells' limits (2.7 s,
2.9 s, 15 and 7 ticks) come from the headless library; a zone that opens
on the aircraft takes about 3.8-4.1 s to leave on both rails. No failed
gate on Gazebo is new to the hil rig.

The 6 known failures (standoff-wedge x3: goal not reached; corridor-swap-
shifted x3: time to safe censored, final offset 11.3-11.4 m > 10 m) and the
3 tracked runs (gps-noise-beyond-margin) are the library's documented open
cells.

## Open

- **Home-check refusals on this rail:** 11 of 143 attempts (7.7 %), MAVROS
  never got the home position. Each was refused before arming, as designed,
  and flown again; cause not investigated.
- **Run manifests say `unversioned`** when the rail is launched from a
  Windows git worktree: the Shield node runs in WSL, where
  `git rev-parse` fails on the worktree's `.git` file (`gitdir: D:/...`).
  The sweep's own record names 21c2771. The 20 Nov sweep, flown from the lab
  checkout, recorded 28a594d in each manifest.
- **GUIDED hold handed to BRAKE** in 5 of 132 missions (the setpoint
  stream faltered), as on 20 Nov (2 of 8). Not seen on the hil rig.
- No vehicle-response fit for Gazebo's iris, so no rail-vehicle labels for
  this rail.
- The grant's broad sweep (~3000 scenarios x 5 seeds at speed > 1) is not
  flown: the Shield node runs on wall time, so speed > 1 is refused.
