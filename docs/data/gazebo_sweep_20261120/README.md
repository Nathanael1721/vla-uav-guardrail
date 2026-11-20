# Gazebo Harmonic functional sweep, 20 Nov 2026 (WP4-04, ARCH-16)

The first sweep flown on the Gazebo functional rail: the `gazebo_functional`
profile (8 missions, one seed), each mission flown end to end on Gazebo's
physics behind ArduPilot SITL's JSON backend, with the stub pilot through the
ROS 2 Shield node and MAVROS 2. Grant (02-implementation/stress-testing.md):
"Where it runs (Gazebo Harmonic for functional sweeps, Project AirSim for
perception stress)".

- **Code:** 28a594d, the lab checkout right after the fast-forward of
  integrate/1120 (clean tree; full CI 185/185 there and in the worktree).
- **Command:** `python experiments/sweep_scenarios.py --profile
  gazebo_functional --backend sitl --out demo/out/gzsweep_20261120/sweep.json
  --runs demo/out/gzsweep_20261120` (1074 s).
- **Rail (each run's physics.json):** `gazebo-harmonic`, gz_sim 8.11.0 (the
  ROS 2 Jazzy vendor build that `sitl/run_ros2_demo.sh` runs; WSL also has
  the system package 8.14.0), world `iris_runway.sdf`, SITL model JSON with
  `gazebo-iris.parm`, ArduCopter 4.5.7; topology `dev`, sim_speedup 1.0;
  pilot `guardrail.vla_stub.GotoPilot`; seed 1001.
- **Not KPI-grade, by design:** every mission's record is `functional_only`
  (the lab's choice of 17 Nov: KPI-grade runs come from hil only), so
  `guardrail.manifest.is_kpi_grade` refuses it and the KPI report files it
  apart from the hil figures.

## Files

| File | What it is |
|---|---|
| `sweep.json` | the sweep's record (per mission: gates, KPIs, FSM, tags) |
| `sweep_stdout.txt` | its console output |
| `kpi_report.md` | the per-family KPI report the sweep wrote |
| `rescore_rail_gz.json` / `rescore_rail_stdout.txt` | `python experiments/sweep_scenarios.py --profile gazebo_functional --backend sitl --rescore-rail "demo/out/gzsweep_*--seed1001" --out docs/data/gazebo_sweep_20261120/rescore_rail_gz.json` |

Run folders (lab checkout, gitignored): `demo/out/gzsweep_<cell>--seed1001/`
(flight log, rosbag2 `bag/`, events, `physics.json`, signed
`<tag>.replay.tar.gz`, trajectory plot, flight-log replay video
`<tag>_track.mp4` rendered after the sweep by tools/render_track_video.py),
and `demo/out/gzsweep_20261120/episode-2026-11-20T09-36-03Z--<cell>--seed1001/`,
each with the grant's six names (manifest.json, policy_bundle.tar.gz,
episode.bag2, shield_audit.jsonl, harness_events.jsonl, kpi.json), except
corridor-along, whose bundle has five: its run (0 repairs) wrote no
shield_audit.jsonl, as that bundle's kpi.json `bundle_files` records.

## Result

**7 of 8 missions passed their gates.**

| Mission | Gate | Outcome | Repairs | Note |
|---|---|---|---|---|
| nfz-head-on | **fail** | RTL | 4 | fail-safe labelled not expected, triggered |
| nfz-head-on-control (Shield off) | pass | fail | 0 | P0 escape 0.654, entered the zone |
| speed-cap | pass | success | 276 | |
| altitude-floor-recovery | pass | success | 45 | |
| corridor-along | pass | success | 0 | |
| dyn-nfz-translate (zone 2 m/s) | pass | success | 6 | |
| time-window-switch-event | pass | success | 2 | |
| failsafe-rtl-zone (12 m ahead) | pass | RTL | 1 | RTL expected |

- **P0 escape:** 0.0 over the Shield-flown ticks of the 7 Shield-on
  missions (passthrough null 0.047); the Shield-off control 0.654. No
  Shield-on mission entered a P0 polygon; the autopilot flew 444 ticks (FSM
  Loiter / RTL; flown as a companion-held GUIDED hold, ArduPilot BRAKE after
  the two handovers below, or RTL) with 0 P0-violating actions.
- **Fail-safe trigger correctness, ideal-vehicle labels (the sweep's own
  gate):** 6/7 = 0.857 (1 false trigger, 0 missed).

### The one failure: nfz-head-on

Flown straight at `nfz-square`. The Shield repaired the first approach
(G1, 0.31 m), then a repair of 2.26 m against theta 2.00 m sent the FSM to
Loiter (G3, t = 3.4 s); Loiter timed out after 30 s (G5) and the FSM
returned home (RTL, G9). No breach; P0 escape 0.

This is the same sequence the hil rig flies on this cell: every
Shield-on nfz-head-on mission of ntFIN and ntFIN2 (6 of 6), nt1107 (3),
nt1111 (3), hs07 and hs09 ends Brake -> Loiter -> RTL on a repair over theta
(2.12-2.30 m; 4.00 m on nt1111_001, which enters by X1). The prefix-on arms
of pab1-pab3 (CSP-consuming pilot) take other paths.

The FSM sequence matches, but the hold was flown differently. On hil, each
of those 14 runs held the Loiter in GUIDED, the companion streaming
setpoints. On Gazebo that stream faltered 4.1 s into the Loiter (no setpoint
for 0.41 s, over the 0.3 s limit; `hold_forced_handover` at t = 7.48 s) and
the hold was handed to ArduPilot BRAKE, which flew it until the G5 timeout
sent RTL at t = 33.5 s. time-window-switch-event shows the same handover
(0.44 s gap, BRAKE for 2.9 s). None of the listed hil runs had a
setpoint-falter handover.

The ideal-vehicle
label (no fail-safe expected) assumes a vehicle that follows the commanded
velocity at once. The adopted rail-vehicle reading r2
(docs/data/failsafe_labels_rail.json) labels this cell `expected: true`
(clause E5): under r2 the trigger is correct and the sweep would read 7/7.
The rescore does not apply r2 here (all 7 Shield-on runs "not accepted": a
sitl-backend sweep writes no `scenario_score.json`), and it should not be
applied unexamined: r2's vehicle was fitted to the hil rig (rh01,
Project AirSim physics), not to Gazebo's iris. Both readings are stated;
neither is a KPI figure.

## Open

- **GUIDED-hold setpoint stream faltered** on Gazebo in 2 of 7 Shield-on
  missions (nfz-head-on 0.41 s, time-window-switch-event 0.44 s) and was
  handed to BRAKE. Not seen on the hil rig. A health finding for this rail,
  cause not investigated.

- Each mission is one seed; the grant's broad sweep (~3000 scenarios x 5
  seeds at speed > 1) is not flown: the Shield node runs on wall time, so
  speed > 1 is refused.
- No vehicle-response fit for Gazebo's iris, so no rail-vehicle labels for
  this rail.
