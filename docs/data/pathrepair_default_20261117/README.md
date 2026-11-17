# PathRepair in the default repair chain (WP3-06), 17 Nov 2026

## What changed (branch wf18-pathrepair)

`Shield(path_repair=True)` is the default. The iterated chain runs
ClearanceProjection, StandoffProjection, CorridorProjection,
LateralProjection, **PathRepair**, KinematicClamp (KinematicClamp and
AltitudeClamp before the loop): the grant's priority order AltitudeClamp ->
LateralProjection -> PathRepair (Safety Shield p3), behind the same
`RepairOperator` Protocol (`guardrail/repair.py`). `path_repair=False` is the
chain flown before this date.

PathRepair was redesigned so it no longer escalates by itself:

1. **Magnitude = the correction of the action it emits.** `magnitude_m` is
   |emitted - raw| (horizontal; the raw command taken at the emitted speed,
   since SpeedClamp reports the speed change) over one control period
   (0.1 s, `fsm.THETA_HORIZON_S`), in metres: the grant's
   ||action_repaired - action_original|| (p4) on the one-period reading the
   FSM already offers. Theta (2.0 m lateral) reads it as it reads every other
   repair's. It was the depth the raw forecast reached into the ring - the
   projection's correction, already logged by StandoffHold / GeofenceSlide,
   so theta's per-axis sum counted it twice (the 2 Nov measurement with that
   magnitude: library fail-safe correctness 150/171).
2. **Superseded projections.** The lateral projection records the detour
   replaces are removed from the decision's `repairs` (the pass ledger keeps
   them; the detour's detail names them), since their action is not flown.
3. **Where chain mode takes a detour** (`PathRepair.repair`, violation None):
   - round **stand-off rings only** (`CHAIN_CATEGORIES`); a keep-out zone is
     left to the projection, whose anti-stall slide keeps moving along it
     (see "Narrowed" below). `PathRepair(chain_categories=("geofence",
     "standoff"))` in a Shield's `repair_operators` puts zone detours back;
     first-feasible mode, where PathRepair runs only after the projection
     failed, detours round both;
   - not when the projection is already **over theta** - abandoning it for
     the fail-safe is theta's call (the Shield's own FSM's theta; a rail
     Shield, which has none, reads `Shield.theta_config`, which
     `sitl/ros2_shield_node.py` sets to its FSM's config);
   - not during a **recovery** (the aircraft already inside a rule);
   - not without a **position fix** (`State.fix_ok` False);
   - not where one control period of it takes the **request's forecast
     deeper** into the ring.
   - It must still re-check clean under every rule (unchanged).

**This changes flown behaviour by design.** Every run flown before
2026-11-17 had PathRepair off; the next hil campaign on the ROS 2 rail flies
it (the rail Shield is built by `guardrail/replay.py rail_shield`, which
takes the default). The stub pilot of the KPI campaigns flies the same
chain as here. `tools/prefix_eval.py` rebuilds `path_repair=False` for a
flight record without the key.

## Headline (Shield-on, the grant's figures)

| figure | smoke 64 ep | library 180 ep | nightly cells, seed 1001, 248 ep |
|---|---|---|---|
| P0 escapes (Shield log) | 0 -> 0 | 0 -> 0 | 0 -> 0 |
| fail-safe trigger correctness | 63/63 = 1.0 -> 1.0 | 179/179 = 1.0 -> 1.0 | 247/247 = 1.0 -> 1.0 |
| repair success rate | 0.993384 -> 0.993384 | 0.994260 -> 0.994264 | 0.992012 -> 0.992027 |
| mean repair magnitude, mean \|emitted - raw\| (m/s) | 6.95 -> 6.95 | 7.0141 -> 7.0071 | 6.3486 -> 6.3319 |
| mission success | 0.460317 -> 0.460317 | 0.536313 -> 0.536313 | 0.469636 -> 0.469636 |
| PathRepair ticks (operator class) | 0 -> 0 | 0 -> 25 | 0 -> 75 |
| episodes that fly differently | 0 | 1 | 3 |

In all three diffs `headline_identical` is true (status, mission success per
family, outcome labels, fail-safe labels, P0, true-state P0 ticks, polygon
entries, breaches), and no episode's FSM edge sequence, mission outcome,
status or hold points changed (`episodes_fsm_outcome_status_or_hold_changed`
empty). Every figure is in `summary.json`, read from the sweeps' own
`_rollup._all_shield_on`.

## Every changed episode

`standoff-approach-angles@offset_m=9,subject_class=pedestrian` (library:
variant 0; nightly: variants 0, 1, 2 - the same flight each time). The
pedestrian stands 9 m off the route inside its 10 m ring's reach. Before:
13 StandoffHold ticks (largest magnitude 1.0 m, within theta), the closing
component removed. After: 25 PathRepair ticks, each superseding the hold,
detouring round the ring's 12 m tangent circle (magnitude 0.071 m mean,
0.082 m max). Same FSM edges (G1 at 0.0 s, G7 back to Normal at 4.5 s
instead of 3.3 s: the raw forecast still crosses the ring while the detour is
flown, so the Brake lasts 1.2 s longer), same outcome (success), closest range
10.012 m -> 10.020 m (ring 10 m), end position 0.32 m away. Mean
|emitted - raw| on that episode 3.61 -> 0.71 m/s: the detour keeps the
heading the hold took away.

Smoke: no episode changed (no smoke cell has a stand-off projection within
theta that loses half the progress).

## Narrowed, and why (the measurements on the way)

The first versions in this branch detoured round zones in chain mode as well
(magnitude and supersede as above; the gates were added one at a time, each
after the measurement that called for it). Episodes with a worse figure, all
zones':

- `nfz-head-on` (6 arms): the detour toward a corner of the square
  deepened the raw request's forecast until the slide's projection was 3.59 m
  over theta; the FSM held in Loiter, censored at the episode's end. Fixed by
  the depth gate (no detour that takes the request deeper).
- `gps-noise-within-margin` cells (version with the theta gate only): more
  true-state margin ticks (library 1326 -> 1396), closest true approach
  1.24-1.43 m vs 1.32-1.67 m before (polygon never entered).
- rail-vehicle `gps-dropout` lane 23.6 (tests/test_failsafe_rubric.py,
  tests/test_position_quality.py): closest true approach to the fence
  2.9 m -> 1.1 m (margin 1.0 m). Fixed by the fix_ok gate.
- `gps-dropout@lane_y=23.6,window=6-13` (ideal vehicle): a detour along the
  ring in the 2 s before the fix froze left the aircraft at the ring; the
  frozen slide then chattered in place and X8 (position lost -> RTL), which
  the cell's E3 label expects, never came: library fail-safe correctness
  179/179 -> 178/179 (0.994, still >= 0.99, but worse). A no-hug rule (the
  detour's forecast 1 m clear of the margin ring) did not bring it back.

So zone detours were taken out of chain mode (`CHAIN_CATEGORIES =
("standoff",)`): the figures above are that version's, with no KPI worse.

## Reproduce

    # before: master d91a2e7 (git archive d91a2e7)
    # after:  git archive d91a2e7 + this branch's guardrail/, sitl/ and
    #         tools/prefix_eval.py (byte-identical to commit 26be716)
    # in each tree:
    python experiments/sweep_scenarios.py --profile smoke --out OUT/smoke/sweep.json --runs OUT/smoke/episodes
    python experiments/sweep_scenarios.py --out OUT/lib/sweep.json --runs OUT/lib/episodes
    python experiments/sweep_scenarios.py --profile nightly --seed 1001 --out OUT/n3/sweep.json --runs OUT/n3/episodes
    # then, for each of smoke / lib / n3:
    python tools/wgs84_bind_compare.py diff BEFORE/X/sweep.json AFTER/X/sweep.json \
        --before-runs BEFORE/X/episodes --after-runs AFTER/X/episodes --out X_diff.json
    python tools/pathrepair_default_summary.py \
        --pair smoke BEFORE/smoke/sweep.json AFTER/smoke/sweep.json smoke_diff.json \
        --pair library BEFORE/lib/sweep.json AFTER/lib/sweep.json library_diff.json \
        --pair nightly_seed1001 BEFORE/n3/sweep.json AFTER/n3/sweep.json nightly_seed1001_diff.json \
        --out summary.json

Produced on the desktop, 17 Nov 2026, from scratch trees (so every sweep
records `code_revision: unversioned`). The episode folders are scratch data
and are not kept. The nightly profile flies three seeds; only 1001 was run.
No flight, SITL, hil or rig run was made for this measurement.

## Files

| file | what |
|---|---|
| `summary.json` | before / after figures per run and every differing episode (`tools/pathrepair_default_summary.py`) |
| `smoke_diff.json`, `library_diff.json`, `nightly_seed1001_diff.json` | `tools/wgs84_bind_compare.py diff` of each run, every field |
