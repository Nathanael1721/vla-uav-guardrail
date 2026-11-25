# hil, 25 Nov 2026: X-11 zone-ridge re-flown with the terrain feed, WP3-07 battery G6 and FSM profile

Three small campaigns on the canonical hil topology (AGX Orin companion
nathan-agxiot@10.42.0.3 + ArduCopter 4.5.7 SITL on the desktop), all at
release 0.7.0 (21c2771, clean on both hosts), flown one after the other on
25 Nov between 21:20 and 21:40 +0800 with the hil stub pilot
(`sweep_scenarios --backend hil --hil-campaign`). The run folders are on
the offload disk (`F:/vla_drone_offload/hil_runs/<run>/`, bags, replay
tarballs and track videos included); this folder keeps each campaign's
sweep.json and log and, per run, kpi.json, metrics.json, report.md,
scenario_score.json, manifest.json, stack_revisions.json and (battery)
sitl_stress.json. None of the three profiles is KPI-bearing
(`grant_kpi_bearing: false`), so none of these runs enters a KPI report of
record (ntFIN5, smREL070).

| Campaign | Profile / cell | Missions | Result |
|---|---|---|---|
| x11r1125 | msl_dem `msl-dem-zone-ridge` (target_x -45, -20) | 2 | 2 passed, 0 failed (287 s) |
| bat1125 | fsm_edges `failsafe-low-battery` (rail_battery) | 1 | 1 passed (142 s) |
| fsmp1125 | fsm_edges `failsafe-rtl-from-hold`, `--rail-fsm-config experiments/fsm_profiles.yaml --rail-fsm-profile grant` | 1 | 1 passed (140 s) |

## X-11: the hil zone-ridge cells with the terrain feed (x11r1125)

On 19 Nov (msl_dem_hil2, e56d3e7, before the goto pilot's terrain
feed-forward 27cbeee) both zone-ridge cells were false triggers. Re-flown
here with the feed on (`--terrain-feed=on` on the mission line):

- both reached the pad, outcome success, fail-safe labels 2/2 (0 false
  triggers, 0 missed), P0 escape 0 over the Shield-flown ticks (null
  passthrough 0.108), 0 P0 ticks flown at the true state, 0 polygon entries;
- FSM max state Normal (target_x -45) and Brake (target_x -20), as on the
  SITL rail on 22 Nov;
- each run's metrics.json `ground` names the pinned DEM
  `experiments/worlds/dem_cmac_wgs84.json` (sha256 44ed8a16...31e0,
  `pinned: true`, up_ref terrain); events.jsonl carries a `terrain_feed`
  event; the replay tarball carries `ground/dem_cmac_wgs84.json` and
  `python -m guardrail.replay verify` passes on both
  (`x11r1125_replay_verify.txt`); kpi.json `kpi_grade` true;
- e2e tick (pilot message received to setpoint) p99 6.5 / 11.6 ms, max
  7.8 / 12.3 ms;
- `tools/replay_from_bag.py`: ticks, trajectory, KPI and manifest_in_bag
  identical 2/2;
- `tools/replay_at_revision.py`: before the fix of 25 Nov (a75f1f5)
  not_replayable 2/2, because the open-loop replay built the Shield with no
  ground and the MSL rule refused it (`x11r1125_replay_at_revision_before_fix.json`);
  with the fix (the Shield rebuilt over the ground file the run named)
  identical 2/2 (`x11r1125_replay_at_revision_a75f1f5.json`), and the six
  19 Nov msl_dem_hil2 runs identical 6/6
  (`msl_dem_hil2_replay_at_revision_a75f1f5.json`).

The (r2) rail-vehicle reading (`x11r1125_rescore_r2.txt`) leaves both runs
unlabelled: the msl_dem cells are not in the r2 label set; the in-campaign
labels above are the cell's own (N1, no fail-safe expected).

## WP3-07: G6's battery reason (bat1125)

`SIM_BATT_VOLTAGE` 12.6 V at boot, 10.0 V at t = 13 s (BATT_LOW_VOLT 10.5,
BATT_CRT_VOLT 10.1, both fail-safe actions none, so the FSM, not ArduPilot,
acts). ArduPilot's BATTERY_STATUS turned CRITICAL at t = 14.19 s with the
FSM in RTL; edges G11, G6, G10; outcome Land_triggered (expected, E1);
gate passed; P0 escape 0; e2e tick max 8.6 ms. The stressor's record
(`sitl_stress.json`) and its log end in `SITL_STRESS RESTORED`: every
BATT_* parameter and SIM_BATT_VOLTAGE written back and read back.
Replay at revision identical 1/1, from the bag identical 1/1. The bundle it
flew is the AGX's fase3-sim-demo-v0.1.4 rebuilt that evening with a
changelog diff (docs/data/x10_bundle_rebuild_20261125/).

## WP3-07: a profile's FSM YAML selected on the rail (fsmp1125)

The first flight with `--rail-fsm-config / --rail-fsm-profile` (5934168,
in 0.7.0): metrics.json `fsm_config` = experiments/fsm_profiles.yaml,
profile `grant`, digest sha256:b9103b0d98c7 (N 3, T 5 s, theta 2.0 / 0.5 m,
T_recover 2 s, loiter timeout 30 s); edges G1, G3, X2, G9; outcome
RTL_triggered (expected); gate passed; P0 escape 0; e2e tick max 12.0 ms.
Replay at revision and from the bag identical 1/1. It is a separate
campaign, never merged with one on the default thresholds.

## How it was run

From Git Bash, the flight worktree at 21c2771, with the rig config's
`store` and the sweep's `--out` / `--runs` on the offload disk:

    experiments/sweep_scenarios.py --profile msl_dem --backend hil --hil-campaign x11r1125 --only msl-dem-zone-ridge ...
    experiments/sweep_scenarios.py --profile fsm_edges --backend hil --hil-campaign bat1125 --only failsafe-low-battery ...
    experiments/sweep_scenarios.py --profile fsm_edges --backend hil --hil-campaign fsmp1125 --only failsafe-rtl-from-hold \
        --rail-fsm-config experiments/fsm_profiles.yaml --rail-fsm-profile grant ...

then per campaign: `--rescore-rail`, `tools/replay_at_revision.py`,
`tools/replay_from_bag.py`, `python -m guardrail.replay verify` and
`tools/render_track_video.py` (one track video per run, in the run folder).
