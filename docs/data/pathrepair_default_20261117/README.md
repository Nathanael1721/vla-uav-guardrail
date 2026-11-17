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

1. **Magnitude on theta's decided basis (reading A).** `magnitude_m` is a
   forecast depth in metres, as for every projection operator
   (docs/DESIGN-escalation-fsm.md decision 1, 2026-11-02;
   `fsm.DEFAULT_THETA_HORIZON_S = None`): the depth the raw forecast reached
   into the ring the detour avoids, or, if larger, the summed magnitudes of
   the projections it supersedes (`PathRepair.depth_magnitude`). Theta
   (2.0 m lateral) therefore reads a detour tick no lower than the
   projection tick it replaces, and caps a detour whose raw forecast goes
   more than 2.0 m deep, as it caps a projection. In the one changed
   episode below the PathRepair magnitudes are 0.556 m mean, 1.0 m max (the
   hold's were 0.539 m mean, 1.0 m max). The one-period |emitted - raw| x
   0.1 s is only written in the detour's detail: it is the reading
   guardrail/fsm.py rejects as the default (at the 4-5 m/s caps it can never
   reach theta). A first draft of this branch used it as `magnitude_m`; the
   review of 17 Nov rejected that, and these figures are re-measured with
   the depth basis.
2. **Superseded projections.** The lateral projection records the detour
   replaces are removed from the decision's `repairs` (the pass ledger keeps
   them; the detour's detail names them), since their action is not flown.
   This is what keeps the depth from being counted twice: the 2 Nov version
   reported the same depth but kept the projection's record beside it, and
   with zone detours on the library's fail-safe correctness fell to 150/171.
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

**Where it flies, and where it does not.** Every run flown before
2026-11-17 had PathRepair off.

- **On:** the library Shield (`Shield()` default) and the KPI rail,
  `sitl/ros2_shield_node.py` (`--path-repair on`, the default; `off` is the
  old chain; the metrics record `shield_forecast.path_repair`). On the rail
  it can act only round a `--subject` declared stand-off ring. The next hil
  campaign flies it; the stub pilot of the KPI campaigns flies the same
  chain as here.
- **Off (pinned, `guardrail/replay.py FOLLOW_RAIL_PATH_REPAIR = False`):**
  every follow / perception rail - `demo/vla_fly.py` (the AerialVLA,
  OpenVLA and AIoT-VLA follows, the stand-off-feed flights),
  `demo/follow_vlm.py`, `demo/pas_ardupilot/perception_node.py` and
  `sitl/run_sitl_demo.py`. In chain mode PathRepair acts only round a
  subject's stand-off ring, which is exactly what these rails feed, and it
  does change what they fly: on tick 201 of the 8 Oct follow-car flight it
  supersedes StandoffHold and flies a 2.64 m/s detour (+5.5 m forward,
  +1.5 m right) in place of the hold's slowdown to 1.35 m/s
  (tests/test_shield.py `test_tick_201_with_the_default_chain_flies_a_stand_off_detour`).
  No follow figure (stand-off feed, agx_follow, follow scoring) has been
  re-measured closed-loop with it, so these rails keep the old chain until
  one is.
- **Unaffected either way:** `demo/real_vla_demo.py`,
  `demo/aerialvla_demo.py`, `demo/aerialvla_pas_demo.py` and
  `sitl/ros2_mission_node.py` set no subject, so there is no stand-off ring
  for chain-mode PathRepair to act on.
- **Replay tools** rebuild the chain each run flew: `tools/prefix_eval.py`
  and `tools/replay_q3.py` read the record (no key: off),
  `tools/replay_standoff_feed.py`, `tools/rescore_kpis.py` and
  `tools/rerender_policy_hud.py` pin it off (`replay_standoff_feed.py
  --path-repair on` is the A/B below).

### Follow flights, open-loop A/B (`tools/replay_standoff_feed.py`)

The logged VLA follow flights under the lab checkout's `demo/out` (79
perception flights, 224 255 ticks from the first acquisition; 39 gate-truth
what-ifs), each tick's logged raw command at its logged pose, with
PathRepair off and on. Open loop: the pose does not follow the detour, so
this counts the ticks a detour would be taken, not what the flight would
then have done.

| forecast | stand-off repair ticks off -> on | PathRepair ticks (on) | P0 escapes off -> on |
|---|---|---|---|
| g_vel (5 s / 0.1 s, subject velocity given: vla_fly as flown) | 2261 -> 2261 (1.01 %) | 76 (0.034 %) | 10 -> 10 |
| g_still (5 s / 0.1 s, subject held still) | 19252 -> 19263 | 1715 | 0 -> 0 |
| c_still (3 s / 0.5 s) | 2540 -> 2544 | 33 | 0 -> 0 |
| c_vel (3 s / 0.5 s, velocity) | 1057 -> 1057 | 12 | 10 -> 10 |

(PERCEPTION flights; with the default off, today's replay gives 8.58 % ->
1.01 % for g_still -> g_vel on these 79 flights, against the 9.1 % -> 1.1 %
published on 1 Nov on the flights of that day.) The P0 escape counts do not
move; whether the detours help or hurt the follow is a closed-loop question
this does not answer.

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
13 StandoffHold ticks (magnitude 0.539 m mean, 1.0 m max, within theta),
the closing component removed. After: 25 PathRepair ticks, each superseding
the hold, detouring round the ring's 12 m tangent circle (magnitude, depth
basis, 0.556 m mean, 1.0 m max). Same FSM edge SEQUENCE (G1 to Brake at
0.0 s, then G7 back to Normal) but not the same timing: G7 at 4.5 s instead
of 3.3 s - the raw forecast still crosses the ring while the detour is
flown, so the Brake lasts 1.2 s longer. Same outcome (success), closest
range 10.012 m -> 10.020 m (ring 10 m), end position 0.32 m away. Mean
|emitted - raw| on that episode 3.61 -> 0.71 m/s: the detour keeps the
heading the hold took away.

Costs on that episode: `kpi.p0_violation_ticks` (ticks on which the RAW
command's forecast breaks a P0 rule) rose 13 -> 25, because the detour keeps
the pilot's request pointing at the ring for longer, and the Brake state
lasts 1.2 s longer (above). No P0 is flown (P0 escapes 0, true-state P0
ticks unchanged), and no headline figure moves.

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
("standoff",)`): the figures above are that version's, with no headline KPI
worse.

## Deviations from the grant's wording (stated, not hidden)

- **Boundary detours are opt-in in chain mode.** The grant's PathRepair is
  "if a predicted segment crosses a boundary, insert a detour waypoint"
  (Safety Shield, Priority 3). In the default chain mode it acts only round
  subject stand-off rings; keep-out zones need
  `PathRepair(chain_categories=("geofence", "standoff"))`. Reasons, measured
  (above): lost RTL on gps-dropout lane 23.6 (fail-safe correctness
  179/179 -> 178/179), the rail vehicle's closest true fence approach
  2.9 m -> 1.1 m, more true-state margin ticks on gps-noise cells.
- **Follow rails keep it off** until a closed-loop follow A/B is flown.
- **First-feasible mode** (off by default) runs PathRepair ungated (none of
  the chain-mode gates) and has not been measured since this redesign.
- **Headless only.** Nothing here was flown on SITL or hil.

## Reproduce

    # before: master d91a2e7 (git archive d91a2e7)
    # after:  git archive of this branch + its guardrail/, sitl/ and tools/
    #         (the depth-basis magnitude; re-measured 17 Nov after review,
    #         every figure and diff file identical to the first measurement)
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
    # the follow A/B (open loop, lab checkout's logged flights):
    python tools/replay_standoff_feed.py --out-root "<lab checkout>/demo/out" --path-repair off
    python tools/replay_standoff_feed.py --out-root "<lab checkout>/demo/out" --path-repair on

Produced on the desktop, 17 Nov 2026, from scratch trees (so every sweep
records `code_revision: unversioned`). The episode folders are scratch data
and are not kept. The nightly profile flies three seeds; only 1001 was run.
No flight, SITL, hil or rig run was made for this measurement.

## Files

| file | what |
|---|---|
| `summary.json` | before / after figures per run and every differing episode (`tools/pathrepair_default_summary.py`) |
| `smoke_diff.json`, `library_diff.json`, `nightly_seed1001_diff.json` | `tools/wgs84_bind_compare.py diff` of each run, every field |
