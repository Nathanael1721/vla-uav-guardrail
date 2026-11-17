# Binding hot-applied geometry to the WGS84 frame by default (WP1-14, WP4-11), 17 Nov 2026

## What changed

Branch wf17-wgs84 (merged into integrate/1117) made every write path into a
policy with a frame store WGS84 only (the REST hot-apply endpoint, the
Shield's zone edits, the harness's spawned zones, `bundle.derive`,
`tools/geo_to_policy.py ingest`). By default the live Shield still enforced
the metres an event GAVE for such a point, while the generation's stored and
signed snapshot (`policy_g<N>.json`), reloaded, enforces the micrometre view
of the stored lat/lon: the same floats for metres on the micrometre grid,
else up to 0.6 um away, under the same hash. A run with a hot-applied,
computed (off-grid) ring therefore did not replay bit for bit from what it
stored: `tests/test_replay_q3.py` passed 8/16 by default (16/16 only with the
opt-in `VLAGUARD_WGS84_BIND=1`).

On branch wf17b-bind:

1. **Binding is the only mode.** The Shield binds every rule an event writes
   to the policy's frame (`Shield._in_frame` -> `models.rule_in_wgs84`, which
   now always binds): what is enforced is the lazy projection of what is
   stored and signed, bit for bit what a reload reads. The opt-in
   (`Shield.wgs84_bind_frame`, `VLAGUARD_WGS84_BIND`) is removed: a policy
   written only in metres (the legacy form) has no frame, and its events keep
   writing metres either way (`test_metre_policy_keeps_metre_writes`).
   `tests/test_replay_q3.py` is 16/16 with no environment variable.
2. **An exact tie on a bound dynamic zone is judged at the geometry's
   resolution.** A computed ring is enforced within 0.6 um of the metres the
   edit computed (0.87 um over a chain of edits), so a tie built into it - a
   zone spawned exactly centred on the aircraft, its edges equally near - was
   broken by that rounding instead of by the heading
   (`shield.escape_direction`). Measured on the first version of this branch
   (5654b53, tie tolerance 1e-9 m for every zone), the nightly profile's
   sampled cells of `dyn-nfz-spawn-on-top` changed in 5 of 6: size 20 m x3
   went G1 Brake 5.0 s -> G7 11.9 s and missed the goal; size 40 m variants
   1 (approach 182.376 deg) and 2 (111.198 deg) went G1 Brake 5.0 s -> G3
   Loiter 10.3 s and ended the episode holding at the zone edge (status
   changed, a Loiter hold point, 118 m and 99 m short of the old end). That
   version's own evidence flew variant 0 only and reported one changed cell.
   Now `shield.tie_tolerance_m(rule)` is 2e-6 m (`_TIE_BOUND_M`) for a
   `dynamic_nfz` whose points are stored in WGS84, and stays 1e-9 m
   (`_TIE_M`) for every fence and for a `dynamic_nfz` in a metre-only policy.
   It is read from what is stored (the class and the lat/lon points), so a
   snapshot reloaded by `tools/replay_q3.py` (which installs generations
   without `_in_frame`) judges the same.
3. **A lat/lon body with inconsistent metres is refused.** Before, a rule
   given wholly in lat/lon came back from `rule_in_wgs84` unchanged, and x/y
   given beside the lat/lon stayed its live view, so the Shield enforced
   those x/y under the lat/lon's hash. Now every such rule is bound: x/y more
   than 1 um from the frame's projection of the lat/lon raise ValueError,
   which `Shield._in_frame` turns into `HotApplyRefused` (HTTP 422 on the
   REST endpoint). `test_a_lat_lon_body_with_inconsistent_metres_is_refused`.

Metres on the micrometre grid (every REST body, every hand-written scenario)
are enforced exactly as given.

What binding does NOT do for WP4-11: it removes one obstacle to replaying a
hot-applied run bit for bit from its stored `policy_g<N>.json`.
`tools/replay_q3.py` re-runs TODAY's Shield, open loop, on stored inputs; it
is not the grant's replay at the revision the run flew, and no flown SITL or
hil run has been replayed at its revision. WP4-11 stays PARTIAL.

## Files

| file | what |
|---|---|
| `smoke_diff.json` | the headless smoke profile (64 episodes), old default vs this branch, every field |
| `library_diff.json` | the default library (`experiments/scenarios.yaml`, one variant per family, 180 episodes) |
| `nightly_seed1001_diff.json` | the nightly profile's cells (`samples_per_family: 3`, every hot-apply family sampled three times) on one seed, 1001: 225 cells, 248 episodes |
| `tick_cost.json` | `Shield.filter` time per tick, metre writes vs bound WGS84 writes |
| `alternative_tie_2e-6/` | this branch against a GLOBAL 2e-6 m tie tolerance (every zone, fences included): NOT adopted, see the last section |

Each diff has `headline_identical`, `headline_fields_differing`,
`episodes_fsm_outcome_status_or_hold_changed` (an episode whose FSM edge
sequence, mission outcome, status or hold points changed),
`episodes_fsm_edge_times_moved` (the same edges at other times) and, per
differing episode, every changed KPI field, a `flown` summary and the
shield_audit tick alignment.

Produced on the desktop, 17 Nov 2026, from scratch copies of the trees
(`git archive`, so every sweep records `code_revision: unversioned`):

    # before: integrate/1117 4afd2a1 (git archive 4afd2a1)
    # after:  git archive 5654b53 + this branch's changed files (guardrail/,
    #         experiments/ and sitl/ byte-identical to the commit that adds
    #         this README; only tools/wgs84_bind_compare.py, tests/ and docs/,
    #         which no sweep reads, were edited after)
    # in each tree:
    python experiments/sweep_scenarios.py --profile smoke --out OUT/smoke/sweep.json --runs OUT/smoke/episodes
    python experiments/sweep_scenarios.py --out OUT/lib/sweep.json --runs OUT/lib/episodes
    python experiments/sweep_scenarios.py --profile nightly --seed 1001 --out OUT/n3/sweep.json --runs OUT/n3/episodes
    # then, for each of smoke / lib / n3:
    python tools/wgs84_bind_compare.py diff BEFORE/X/sweep.json AFTER/X/sweep.json \
        --before-runs BEFORE/X/episodes --after-runs AFTER/X/episodes --out X_diff.json
    python tools/wgs84_bind_compare.py tick-cost --ticks 400 --rounds 9 --out tick_cost.json

The episode folders are scratch data and are not kept; every number below is
in the JSON files. The comparison tool was corrected in the same commit (see
"Corrections" below), and every diff here was made with the corrected tool.

The nightly profile flies three seeds; only 1001 was run here (a seed changes
only families that draw noise, gusts or jitter, none of which hot-applies a
zone). The hil rail has NOT flown this branch (rig checks below).

## Headline: unchanged in all three runs, mission success included

| figure | smoke (64) | library (180) | nightly cells, seed 1001 (248) |
|---|---|---|---|
| status | pass 56, tracked 6, known_failure 2 | pass 155, tracked 22, known_failure 3 | pass 221, tracked 22, known_failure 5 |
| **mission success, Shield-on** (grant KPI 1) | 0.460317 (29/63) | 0.536313 (96/179) | 0.469636 (116/247) |
| goal reached / missed / undeclared | 21 / 30 / 12 | 82 / 64 / 33 | 102 / 100 / 45 |
| mission success, dynamic_nfz_movement | 0.571429 | 0.333333 | 0.342466 |
| outcome labels success / fail / RTL / Land | 29 / 11 / 20 / 4 | 96 / 21 / 58 / 5 | 116 / 35 / 92 / 5 |
| fail-safe labels (FSM RTL/Land) | 63/63 = 1.0 | 179/179 = 1.0 | 247/247 = 1.0 |
| nulls: never / always trigger | 0.619048 / 0.380952 | 0.648045 / 0.351955 | 0.607287 / 0.392713 |
| P0 escapes, all episodes | 72, all `nfz-head-on-control` (the Shield-off control) | 72, same | 72, same |
| true-state P0 ticks flown / by the autopilot | 1058 / 1008 | 1326 / 1237 | 1559 / 1465 |
| polygon entries, breaches | 0, 0 | 0, 0 | 0, 0 |

Mission success is every per-family rate and count in `_rollup`, not only
the Shield-on total (`headline.mission_by_family`). In all three diffs
`headline_identical` is true and `headline_fields_differing` is empty, and no
episode's FSM edge sequence, mission outcome, status or hold points changed.

(The first version of this README said "headline unchanged" for the library
while its own diff showed mission success 0.536313 -> 0.530726 Shield-on and
0.333333 -> 0.296296 for dynamic_nfz_movement: the diff tool's headline left
mission success out. With the tie judged at the geometry's resolution that
drop is gone; it was the 20 m centred spawn's missed goal.)

## Every changed episode

All of them hot-apply a dynamic zone at a computed, off-grid position (a
spawn at the vehicle's position, a translate of a spawned zone, a fail-safe
cell's zone). Ticks are monitor ticks (dt 0.1 s).

**Smoke, 8 of 64** (the library and nightly runs have each of these with the
same figures):

1. `dyn-nfz-spawn-ahead@ahead_m=12,variant=0,vehicle_speed_mps=4`:
   `max_` / `mean_intervention_s` 31.9 -> 32.0, `phase1_violation_ticks_dyn-nfz`
   319 -> 320, `repair_not_applied_autopilot_ticks` 317 -> 318: one extra
   violation tick at the t+5.0 s lookahead sample while the autopilot flies
   RTL (the Shield's output is not applied); emitted actions differ by at
   most 5e-8 m/s, one RTL tick slides the other way (mirrored).
2. `dyn-nfz-spawn-on-top@size_m=40,variant=0`: the zone is spawned centred on
   the aircraft, the tie goes to the heading as before (same FSM record, same
   end position), but the escape direction is now from the bound ring, about
   1e-8 rad off the pilot's own direction, so a GeofenceEscape repair is
   recorded (and audited) on each of the 53 ticks inside, 3e-6 m/s in all
   (Recovery ticks 0 -> 53; `repair_count`, `repair_floored_ticks` and the
   repair aggregates move with them). Before, no tick was audited, so the
   audit alignment has nothing to compare these ticks with.
3. `dyn-nfz-translate@variant=0,zone_mps=2` (arms canonical, pp0, pp1, one
   flight): `max_intervention_s` 2.9 -> 3.0, `min_true_dist_to_fence_m`
   1.37 -> 1.371, LateralProjection `sum_mps` 175.174911 -> 175.174966. The
   slide against the moving zone alternates between the edge's two tangents;
   from tick 77 the alternation runs in the other phase for 13 ticks
   (mirrored actions), because the anti-stall side ("keep the raw action's
   turn side") is the sign of a head-on dot product, which the bound edge's
   sub-um rotation decides (14 large-difference ticks in all; tick 125
   differs by 7e-5 m/s because the aircraft is where the other phase left
   it).
4. `failsafe-loiter-timeout`: one violation tick more at t+5.0 s, in RTL
   (318 -> 319; not applied).
5. `failsafe-low-battery`: one more, in RTL (19 -> 20; not applied).
6. `failsafe-rtl-zone@ahead_m=12,variant=0`: one fewer, in RTL (20 -> 19;
   not applied).

**Library, 14 of 180**: the 8 above and

- `dyn-nfz-spawn-on-top@size_m=20,variant=0`: as 2. (28 Recovery records,
  3e-6 m/s in all), no FSM edge, goal reached. (On 5654b53, at 1e-9 m: G1
  Brake 5.0 s, G7 11.9 s, goal missed.)
- `dyn-nfz-translate@variant=0,zone_mps=1` (three arms): the same
  tangent-phase swap as 3., ticks 77-104 mirrored; no KPI headline field.
- `dyn-nfz-spawn-ahead@ahead_m=30,variant=0,vehicle_speed_mps=4` and `=8`:
  the same tangent-phase swap from tick 77; the episode ends 0.80 m from
  where it ended before (`final` -222.07, -19.12 -> -222.59, -18.51); same
  FSM edge (G1 at 7.3 s / 0.0 s), same outcome (fail, as before).

**Nightly cells, seed 1001, 29 of 248**: the 14 library ones and 15 more:

- `dyn-nfz-spawn-on-top` @20 and @40, variants 1 and 2: as 2. (no FSM
  edge, goal reached, only the Recovery records, 28 or 53 ticks); @40
  variant 1 also counts one tick more inside the zone's polygon
  (`ticks_inside_fence_polygon` 50 -> 51: the zone, imposed on the
  aircraft, is left at its edge, which moved by a micrometre).
- FSM edges at other TIMES, same edges (`episodes_fsm_edge_times_moved`),
  each with the anti-stall slide in the other tangent phase (mirrored
  actions):
  `dyn-nfz-spawn-ahead@ahead_m=12,variant=1,vehicle_speed_mps=2` G1 5.6 ->
  5.5 s (`intervention_episodes` 0 -> 1, 0.1 s);
  `@ahead_m=30,variant=2,vehicle_speed_mps=2` G1 14.5 -> 14.6 s
  (`intervention_episodes` 1 -> 0); `dyn-nfz-translate@variant=1,zone_mps=1`
  (three arms) G7 22.9 -> 22.7 s, P0-violation ticks (all repaired and
  converged) 135 -> 133, `min_true_dist_to_fence_m` 1.276 -> 1.087, end
  position 0.80 m apart.
- one violation tick more or fewer at the t+5.0 s sample while the
  autopilot flies RTL (`repair_not_applied_autopilot_ticks` moving with
  it): `dyn-nfz-spawn-ahead` @12 variant 2 (4 m/s) and @6 variant 2
  (2 m/s), where the Shield's unapplied output also slides the other way
  (mirrored, 305 and 309 ticks; the end position is unchanged), @6 variant 1
  (2 m/s), @3 variant 1 (2 and 4 m/s; there also `intervention_censored`
  1 -> 0 and `intervention_episodes` 0 -> 1), and
  `failsafe-rtl-zone@ahead_m=12,variant=1` (19 -> 20).

No mission outcome, status, FSM edge sequence or hold point changed in any
of the 248 episodes.

So the path changes that remain are tangent-side swaps: where the commanded
velocity is exactly head-on to a bound zone's edge, which way the slide goes
is decided by the edge's sub-micrometre rotation, as it was decided by the
1e-14 m rounding of the computed ring before. They are path changes of up to
0.8 m at the episode's end in these cells and FSM edges up to 0.2 s apart,
not micrometre-level. A tolerance on that side (1e-5 rad: head-on keeps the
first tangent) was tried on the three spawn/translate families and did not
give back the unbound choice: against the unbound runs it changed 38 of 69
episodes instead of 25 and moved FSM edge times in 14 instead of 5, so it is
not in the code.

## Per-tick cost (`tick_cost.json`)

`Shield.filter` on a hot-applied, rotated (off-grid) zone, 9 interleaved
rounds of 400 ticks per mode, desktop: median 413.9 us with metre writes vs
409.7 us with bound WGS84 writes (ratio 0.99; earlier runs of the same
measurement gave +1.2 % and +4 %, so this is run-to-run noise). The ticks
read a point's projected view 0 times in either mode
(`projection_reads_during_ticks`): the projection is computed once, when the
zone is committed. The cost is at hot-apply time: a spawn plus a rotate
commits in a median 1.09 ms bound vs 0.86 ms with metre writes (+0.22 ms
per event, the lat/lon conversion and the bind).

## Pinned digests and tests

- `tests/test_hold_points.py` `C77_IN_PLACE_DIGESTS`
  `dyn-nfz-rotate-spin@deg_per_s=9`: 89cf8aa54cdee7d4 -> 1cac6a6572420378
  (164 rows either way). Positions and emitted actions agree to 1e-14, every
  FSM edge at the same time; only the repair RECORD of 8 Loiter ticks
  (t 11.9-13.2 s, emitted 0 either way) gained or lost a second GeofenceSlide
  pass removing a 0.00 m/s into-zone component. Not a tie: unchanged by the
  tie tolerance.
- `tests/test_sweep.py`: the centred-spawn premise is integrate/1117's again
  (no FSM edge); the first version of this branch had re-pinned it to G1/G7.
- `tests/test_wgs84_writes.py`: the single bound mode; the tie tolerance
  (`test_a_zone_centred_on_the_aircraft_escapes_toward_the_heading_once_bound`,
  `test_the_tie_tolerance_is_a_property_of_what_is_stored`); every sampled
  `dyn-nfz-spawn-on-top` cell flies as the metre writes did
  (`test_centred_spawns_fly_as_the_metre_writes_did`, which fails at 1e-9 m);
  the refused inconsistent lat/lon body; the diff tool's filter and headline;
  this README's counts (`test_the_bind_evidence_matches_its_readme`).

No `docs/data` digest or `tests/fixtures` file pins hot-applied geometry:
`tests/test_replay_identity.py`, `test_replay_q3.py`, `test_bundle.py`,
`test_harnesstruth_ci.py` pass unchanged. Stored policies' hashes are
unchanged (`test_stored_policy_hashes_unchanged`).

## Corrections to the first version of this evidence

- The diff tool's filter (`IGNORED_LEAF`, matched with `endswith`) dropped
  flown-behaviour fields along with the wall-clock ones: `kpi.hold_points`,
  `kpi.harness_events`, `kpi.collision_contacts` / `_objects`, the
  `*_distinct_flights` fail-safe figures, `cell_seed_slots` and the
  `wind-gusts` family row. It now drops only whole keys (`runtime_s`,
  `code_revision`, `ts`, `bundle`, ...) and `*_s_wall`.
- Its headline left out mission success (above).
- It said one exact-tie cell changed; the sampled variants showed five,
  two of them ending in a Loiter hold (above).
- Its cross-check "the old tree with `VLAGUARD_WGS84_BIND=1` equals the new
  default (0 episodes differ)" rested on the faulty filter and no longer
  applies (the new default adds the tie tolerance); it was not re-run.

## Not adopted: a global tie tolerance (`alternative_tie_2e-6/`)

The tolerance at the geometry's resolution could apply to every zone: every
stored policy has been WGS84 since 2026-11-03, so a static fence is the
micrometre view of its lat/lon too. Raising `shield._TIE_M` itself to 2e-6 m
was flown against this branch (the diffs in `alternative_tie_2e-6/` are
this branch -> global): it changes static-zone cells this item does not
touch - `wind-beyond-authority@lane_y=15,wind_speed_mps=12` (an aircraft
0.18 um off a zone's bisector: one RTL tick fewer, true-state P0 ticks
1058 -> 1057 in smoke) and three `gps-noise-within-margin` cells (latency 2:
FSM G1/G7 -> G1/G7/G1/G7). So the tolerance is scoped to the zone class whose
rings events compute; a global one is a decision of its own.

## Rig checks for the integrator (not run here)

Nothing here was flown on SITL or hil. To see the change on the rail:

1. Fly the three hot-apply families that changed, on hil, at this branch's
   commit (the AGX checkout synced to it): first
   `python experiments/sweep_scenarios.py --backend hil --profile nightly --seed 1001 --only dyn-nfz-spawn-on-top dyn-nfz-spawn-ahead dyn-nfz-translate --hil-dry-run`,
   then the same without `--hil-dry-run`. Expect the centred spawns
   (`dyn-nfz-spawn-on-top`, all six cells) to take no FSM edge, as on
   integrate/1117. A centred spawn at the MEASURED vehicle position on hil is
   the same kind of tie and is judged the same way.
2. Replay what was flown: `python tools/replay_q3.py --root demo/out --since 2026-11-17 --until 2026-12-31 --only <campaign tag> --out <dir>`.
   The default `--until` is 2026-10-31, which leaves out every run from 17
   Nov on (the tool then exits "no run could be replayed", not "identical"),
   so pass `--until`. Run folders given on the command line bypass the date
   window altogether.
3. Comparing those hil runs with integrate/1117 flights of the same cells:
   expect sub-um action differences and one-tick shifts at the t+5.0 s
   lookahead sample, and, where the command is head-on to a moving or
   spawned zone's edge, the anti-stall slide in the other tangent phase -
   metre-level path differences (0.8 m at the end in the headless runs) and
   FSM edges 0.1-0.2 s apart, with the same edges and outcomes.
