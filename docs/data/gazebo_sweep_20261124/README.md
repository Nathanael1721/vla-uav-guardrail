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
| `kpi_report_gz_2026-11-25.md` / `.json` | `python tools/kpi_report.py --runs '.claude/worktrees/flight-21c2771/demo/out/gzsweep_*' --no-sweep --rescore-rail docs/data/gazebo_sweep_20261124/rescore_rail_gz.json`, from a clean tree at 96a5a03 after these inputs were committed |

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

- **Home-check refusals on this rail:** 11 of 143 attempts (7.7 %), the
  Shield node never got the home position. Each was refused before arming,
  as designed, and flown again. Read from the kept folders on 27 Nov (next
  section): the node's home requests never reached the autopilot; why
  MAVROS dropped them is not established.
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

## The 11 home-check refusals, read from the kept folders (27 Nov)

Read offline from the 11 folders in `refly/errored_run_folders.txt`
(worktree flight-21c2771, `demo/out/_gz_errors_rel070/`) and, for contrast,
the 132 flown run folders: each run's `events.jsonl` and its rosbag2 `bag/`
(MCAP, read with `rosbags`), including the raw MAVLink MAVROS recorded
(`/uas1/mavlink_sink` to the autopilot, `/uas1/mavlink_source` from it). The
Shield node code is the release's, 21c2771 (`sitl/ros2_shield_node.py`,
`check_home` and `bring_up`).

| | 132 flown | 11 refused |
|---|---|---|
| `COMMAND_LONG` 410 (MAV_CMD_GET_HOME_POSITION) on the wire | exactly 1, after the node started | exactly 1, before the node's `episode_start` |
| its `COMMAND_ACK` | ACCEPTED (132) | FAILED (11) |
| node's `/mavros/home_position/req_update` calls (`home_check.home_requests`) | 1 | 30 (one every 3 s for 90 s) |
| `HOME_POSITION` (242) from the autopilot | 2 (121 runs) or 3 (11 runs) | 1 |
| `/mavros/home_position/home` messages in the bag | 2-3 | 1, published 1.47-3.65 s after the bag started |
| node's first home request, against "EKF3 IMU0 is using GPS" | 6.90 to 10.24 s after | 11.04 s before to 6.17 s after |

What the records show:

- **The autopilot did send a home, once, at boot.** In every refused run
  ArduPilot sent one `HOME_POSITION` 1.47-3.65 s into the bag, 21.4-21.9 s
  before "EKF3 IMU0 is using GPS", and MAVROS published it (offered QoS: RELIABLE,
  TRANSIENT_LOCAL). The Shield node subscribes to
  `/mavros/home_position/home` with `qos_profile_sensor_data` (best effort,
  volatile), so it never receives a sample published before it subscribed:
  in all 132 flown runs the node's `autopilot_home` event falls -0.03 to
  +0.06 s from the start of its own request loop (computed from
  `waited_s`, which is rounded to 0.1 s), and the 11 refused runs have none. In 2 of the 11 the adapter (`sitl/mavlink_adapter_node.py`),
  subscribed earlier, did take that boot-time home and uploaded the fence
  before GPS; in the other 9 it had no home either and uploaded nothing.
- **The node's requests never reached the autopilot.** In every flown run
  the node's one `req_update` call put a `COMMAND_LONG` 410 on the wire,
  ArduPilot answered ACCEPTED and sent `HOME_POSITION`, and the check passed
  with `waited_s` 0.0 (125 runs) to 0.7 (7 runs). In every refused run the
  node called
  `req_update` 30 times and the wire carries no `COMMAND_LONG` 410 from any
  of them; the only one is the request sent before the node started (the
  node is the only code in this repository that calls `req_update`), which
  ArduPilot answered FAILED because it held no home yet.
- **The difference is timing.** The node asks for the home after waiting
  up to 15 s for the adapter's fence upload (`bring_up`), and the adapter
  uploads only once it has a home. In the flown runs that wait timed out
  (15.0-15.4 s after the fences went out) 6.9-10.2 s after the EKF took
  GPS, and the node's request was answered. In
  the refused runs the fences went out 8.9-11.1 s before GPS (4.8-8.1 s in
  the flown runs), the wait ran its 15 s in 9 of them and ended after
  0.06-0.10 s in the 2 with an early upload, so the node's first request
  left at most 6.17 s after
  GPS, or before it, and none of its requests, then or later, reached the
  autopilot. The bag cannot say why MAVROS sent none of them (its home
  plugin's handling of `req_update` is not recorded), so that step is not
  established.

Not changed here: the node's subscription QoS and its request loop. A
transient-local subscription would deliver the boot-time home, but that
home predates the EKF's GPS, so the check would then rest on a home set
before the position estimate; that change needs its own review and
flights. No Shield decision is involved: every refusal is before arming.

## Six-name bundles and the manifest revision on Gazebo

Read from the run folders on 27 Nov (`manifest.json` and the six names in
each episode folder):

| Gazebo runs | Episodes | All six names | `manifest.json` `code_revision` |
|---|---|---|---|
| 20 Nov sweep, lab checkout (`demo/out/gzsweep_20261120/episode-*`, `../gazebo_sweep_20261120/`) | 8 | 7 (corridor-along has no `shield_audit.jsonl`: 0 repairs) | `28a594dd0c43` (8/8) |
| 25 Nov mid-term gate, worktree midterm-demo (`demo/out/stress_gazebo_midterm/episode-*`, `../gazebo_midterm_20261125/`) | 3 | 3 | `75a6d45be758` (3/3) |
| this sweep, worktree flight-21c2771 (`demo/out/stress_gazebo_functional/episode-*`) | 132 | 132 | `unversioned` (132/132) |

The `unversioned` record is the Windows-worktree case of Open above; its
fix, 6a78190 ("manifest.code_revision: read a Windows git worktree from
WSL"), is an ancestor of 75a6d45 and not of 21c2771, and the mid-term runs
at 75a6d45 record their revision from a worktree.
