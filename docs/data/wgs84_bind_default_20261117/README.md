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

On branch wf17b-bind the Shield binds every rule an event writes to the
policy's frame (`Shield._in_frame` -> `models.rule_in_wgs84`, which now always
binds): what is enforced is the lazy projection of what is stored and signed,
bit for bit what a reload reads. The opt-in (`Shield.wgs84_bind_frame`,
`VLAGUARD_WGS84_BIND`) is removed: no policy needs an opt-out, because a
policy written only in metres (the legacy form) has no frame, and its events
keep writing metres either way (`test_metre_policy_keeps_metre_writes`).
`tests/test_replay_q3.py` is 16/16 with no environment variable.

Metres on the micrometre grid (every REST body, every hand-written scenario)
are enforced exactly as given. A computed ring (rotate, scale, a spawn at an
off-grid vehicle position) is enforced within 0.6 um of the metres the edit
computed; over a chain of edits each computed from the previous bound view
the offsets compound (0.87 um worst in `test_live_view_is_the_snapshot_view_bit_for_bit`).

## Files

| file | what |
|---|---|
| `smoke_diff.json` | the headless smoke profile, old default vs new default, every field |
| `library_diff.json` | the same for the whole default library (`experiments/scenarios.yaml`, 180 episodes) |
| `tick_cost.json` | `Shield.filter` time per tick, metre writes vs bound WGS84 writes |
| `alternative_tie_2e-6/` | the smoke and library diffs with `shield._TIE_M` = 2e-6 m as well (NOT adopted; see the last section) |

Produced with `tools/wgs84_bind_compare.py` on the desktop, 17 Nov 2026:

    # old default: integrate/1117 HEAD 4afd2a1 (clean tree for the smoke run;
    # the library run used `git archive 4afd2a1`, so it records `unversioned`)
    python experiments/sweep_scenarios.py --profile smoke --out BEFORE/sweep.json --runs BEFORE/episodes
    python experiments/sweep_scenarios.py --out LIB_BEFORE/sweep.json --runs LIB_BEFORE/episodes
    # new default: this branch's tree (4afd2a1-dirty at the time of the run)
    python experiments/sweep_scenarios.py --profile smoke --out AFTER/sweep.json --runs AFTER/episodes
    python experiments/sweep_scenarios.py --out LIB_AFTER/sweep.json --runs LIB_AFTER/episodes
    python tools/wgs84_bind_compare.py diff BEFORE/sweep.json AFTER/sweep.json \
        --before-runs BEFORE/episodes --after-runs AFTER/episodes --out smoke_diff.json
    python tools/wgs84_bind_compare.py diff LIB_BEFORE/sweep.json LIB_AFTER/sweep.json \
        --before-runs LIB_BEFORE/episodes --after-runs LIB_AFTER/episodes --out library_diff.json
    python tools/wgs84_bind_compare.py tick-cost --ticks 400 --rounds 9 --out tick_cost.json

As a cross-check, the smoke profile flown on the OLD tree with
`VLAGUARD_WGS84_BIND=1` gives a sweep identical to the new default in every
field the diff reads (0 episodes differ): the new default is exactly the old
opt-in.

The episode folders are scratch data and are not kept; every number below is
in the JSON files.

## Headline: unchanged

Smoke (64 episodes) and library (180 episodes), before = after:

| figure | smoke | library |
|---|---|---|
| status counts | pass 56, tracked 6, known_failure 2 | unchanged |
| fail-safe labels (FSM RTL/Land) | 63/63 = 1.0, 0 false, 0 missed | unchanged |
| nulls: never / always trigger | 0.619048 / 0.380952 | unchanged |
| P0 escapes, Shield-on episodes | 0 | 0 |
| P0 escapes, all episodes | 72 (all `nfz-head-on-control`, the Shield-off control) | unchanged |
| true-state P0 ticks flown / by the autopilot | 1058 / 1008 | unchanged |
| polygon entries, breaches | 0, 0 | unchanged |

`headline_identical: true` in both files. No status changed, no gate failed
that passed before, no violation was missed or newly flown.

## Every changed field in the smoke profile (8 of 64 episodes)

All 8 episodes hot-apply a zone at a computed, off-grid position (a spawn at
the vehicle's position, a translate of a spawned zone). Ticks are monitor
ticks (dt 0.1 s).

1. **dyn-nfz-spawn-ahead@ahead_m=12,variant=0,vehicle_speed_mps=4**:
   `max_intervention_s` / `mean_intervention_s` 31.9 -> 32.0,
   `phase1_violation_ticks_dyn-nfz` 319 -> 320,
   `repair_not_applied_autopilot_ticks` 317 -> 318. From tick 50 (the spawn)
   the emitted action differs by at most 5.0e-8 m/s on 318 ticks (the ring is
   0.6 um elsewhere). One extra violation tick, 369, is at the far end of the
   5 s lookahead: the predicted position at t+5.0 s (the last sample) lands
   inside the ring by less than the shift. The vehicle is in RTL there - the
   autopilot flies, the Shield's output is not applied (that is the +1 in
   `repair_not_applied_autopilot_ticks`). Tick 353 (RTL as well) slides the
   other way along the edge (emitted (+2.58, -3.06) vs (-2.58, +3.06) m/s:
   two tangents equally good, picked by a sub-um difference); not applied
   either, being RTL.
2. **dyn-nfz-spawn-on-top@size_m=40,variant=0**: `repair_count`,
   `repaired_ticks`, `repairs_per_episode`, `repair_floored_ticks`,
   Recovery `ticks` / `sole_ticks` 0 -> 53; Recovery `sum_mps` 0.0 -> 3e-06;
   `max_` / `mean_repair_magnitude_mps`, `mean_yaw_repair_dps` None -> 0.0;
   `repair_magnitude_breakdown_status` no_repaired_tick -> measured. The zone
   is spawned centred on the aircraft and the pilot already flies straight
   out at the 4 m/s cap. Before, the escape direction the Shield computed
   from the given metres was the pilot's own direction to the last bit, so
   no repair was recorded; bound, the edge is rotated by about 1e-8 rad and
   the GeofenceEscape "repair" changes the action by 4.5e-8 m/s on each of
   the 53 ticks inside (ticks 50-102), 3e-6 m/s in all. Same path, same exit.
   The `_rollup` / `_cell_rollup` repair aggregates (`_all_shield_on`
   `mean_repair_count_per_episode` 104.698 -> 105.54, Recovery and `floored`
   ticks 106 -> 159 and 54 -> 107, their means, `dynamic_nfz_movement`
   rollups) and `_top_failures[4].repaired_action` (this episode's tick, the
   same 4e-8 m/s) move only because of these 53 ticks.
3. **dyn-nfz-translate@variant=0,zone_mps=2** (arms canonical, pp0, pp1; the
   headless pilot reads no text, so the three are one flight):
   `max_intervention_s` 2.9 -> 3.0, `min_true_dist_to_fence_m` 1.37 -> 1.371
   (1 mm FURTHER from the zone), LateralProjection / P0 `sum_mps` 175.174911
   -> 175.174966. The aircraft brakes against the moving zone and the slide
   alternates between the edge's two tangents (+/-3.66, -/+1.61 m/s) every
   few ticks. From tick 77 the alternation runs in the other phase (13 ticks
   mirrored exactly, ticks 77-101), because the tie between the tangents is
   decided by sub-um geometry. The violation window ends one tick earlier
   (102 instead of 103: both at the t+5.0 s lookahead sample), and the
   intervention, counted to the end of the episode's last window, reads
   0.1 s longer. Tick 125 (Normal, no repair) differs by 7e-5 m/s because
   the aircraft is where the other phase left it.
4. **failsafe-loiter-timeout**: `max_` / `mean_intervention_s` 31.8 -> 31.9,
   `phase1_violation_ticks_loiter-zone` 318 -> 319,
   `repair_not_applied_autopilot_ticks` 317 -> 318. One extra violation tick,
   348, at t+5.0 s, in RTL (autopilot flying, not applied).
5. **failsafe-low-battery**: `mean_intervention_s` 4.65 -> 4.7,
   `phase1_violation_ticks_rtl-zone` 19 -> 20,
   `repair_not_applied_autopilot_ticks` 92 -> 93. One extra violation tick,
   119, at t+5.0 s, in RTL.
6. **failsafe-rtl-zone@ahead_m=12,variant=0**: `max_` / `mean_intervention_s`
   2.0 -> 1.9, `phase1_violation_ticks_rtl-zone` 20 -> 19,
   `repair_not_applied_autopilot_ticks` 19 -> 18. One violation tick FEWER,
   49, at t+5.0 s, in RTL.

Every violation count that moved by one (episodes 1, 4, 5, 6) is a predicted
position at the last sample of the 5 s lookahead (t+5.0 s) within a
micrometre of the ring, and all four ticks are in RTL, where the Shield's
output is not applied; the translate's window (3.) shifts by one tick, also
at the t+5.0 s sample, without changing its count. Every emitted
difference while the Shield flies is under 5e-8 m/s, except where two
equally good tangents swap (mirrored actions). Fail-safe labels, P0 figures
and statuses are unchanged.

## The whole library (14 of 180 episodes)

The 8 smoke episodes above (with the same figures) and 6 more:

- **dyn-nfz-translate@variant=0,zone_mps=1** (three arms): no KPI field
  changed; the same tangent-phase swap as in 3. (ticks 77-104 mirrored).
- **dyn-nfz-spawn-ahead@ahead_m=30,variant=0,vehicle_speed_mps=4 and =8**:
  `final.x` / `final.y` -222.07, -19.12 -> -222.59, -18.51 (the episode's end
  position, 0.8 m apart); the same tangent-phase swap from tick 77; no
  status, P0 or fail-safe figure changed.
- **dyn-nfz-spawn-on-top@size_m=20,variant=0**: a real change of what the
  Shield does, at an exact tie. The 20 m zone is spawned exactly centred on
  the aircraft. As written, its four edges are equally near and
  `shield.escape_direction` breaks the tie toward the commanded heading, so
  the pilot's forward command is the way out (no violation, no FSM edge).
  Bound, the edges are up to about 1.2 um apart in distance, above the tie
  tolerance (`shield._TIE_M`, 1e-9 m), so the escape takes the side edge that
  is nearer by about a micrometre; the forward command is no longer
  escaping: G1 Brake at 5.0 s, G7 back to Normal at 11.9 s, 49 P0-violation
  ticks, all repaired and converged (`repair_success_rate` 1.0,
  `failsafe_trigger_correctness` 1.0 on 49/49; 28 of them converged while
  the aircraft was still inside, `repairs_converged_while_unsafe`),
  `p0_escapes` 0, `true_p0_flown_ticks` 0 before and after (the zone was
  imposed on the aircraft), status still `pass`. The detour costs about 5 s, so the 30 s episode ends 9 m short of
  its goal: `mission_outcome` success -> fail (`goal_missed`). It is the only
  episode whose outcome changed. `tests/test_sweep.py`
  (`test_what_the_autopilot_flew_through_is_counted_apart_and_reported`)
  pins this centred run's new edges.

## Per-tick cost (`tick_cost.json`)

`Shield.filter` on a hot-applied, rotated (off-grid) zone, 9 interleaved
rounds of 400 ticks per mode, desktop: median 417.2 us with metre writes vs
422.4 us with bound WGS84 writes (+1.2 %; an earlier run of the same
measurement gave +4 % on the median and -0.6 % on the mean, so this is
run-to-run noise). The ticks read a point's projected view 0 times in either
mode (`projection_reads_during_ticks`): the projection is computed once,
when the zone is committed, and the tick works on the compiled rings. The
cost is at hot-apply time: a spawn plus a rotate commits in a median 1.08 ms
bound vs 0.89 ms with metre writes (+0.19 ms per event, the lat/lon
conversion and the bind).

## Pinned digests re-baselined

- `tests/test_hold_points.py` `C77_IN_PLACE_DIGESTS`
  `dyn-nfz-rotate-spin@deg_per_s=9`: 89cf8aa54cdee7d4 -> 1cac6a6572420378
  (164 rows either way). Positions and emitted actions agree to 1e-14, every
  FSM edge at the same time; only the repair RECORD of 8 Loiter ticks
  (t 11.9-13.2 s, emitted 0 either way) gained or lost a second GeofenceSlide
  pass removing a 0.00 m/s into-zone component.
- `tests/test_sweep.py`: the centred-spawn premise above.
- `tests/test_wgs84_writes.py`: rewritten for the single (bound) mode.

No `docs/data` digest or `tests/fixtures` file pins hot-applied geometry:
`tests/test_replay_identity.py`, `test_replay.py`, `test_replay_pipeline.py`,
`test_replay_from_bag.py`, `test_bundle.py`, `test_policy_hash.py` pass
unchanged. Stored policies' hashes are unchanged
(`test_stored_policy_hashes_unchanged`).

## Not adopted: a tie tolerance at the geometry's resolution

Raising `shield._TIE_M` to 2e-6 m (above the 1.2 um a bound zone's edge
distances can differ by) restores the 20 m centred spawn (success, no FSM
edge) and was tried. It also changes static-zone cells, whose geometry is
the micrometre view of a stored WGS84 policy since 2026-11-03:
wind-beyond-authority@lane_y=15,wind_speed_mps=12 (an aircraft 0.18 um off a
zone's bisector: one RTL tick fewer, `true_p0_flown_ticks` 137 -> 136, in
smoke) and three gps-noise-within-margin cells (new FSM edges at latency 2).
Those are flown changes outside this item, so the tolerance stays 1e-9 m;
the two diffs are kept in `alternative_tie_2e-6/` for that decision.
