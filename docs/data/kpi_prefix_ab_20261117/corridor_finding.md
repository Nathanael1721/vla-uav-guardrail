# Finding: corridor-drift-out prefix-on ended in RTL on pab2_011 (17 Nov 2026)

Contract row: WP4-15 (the Prefix A/B). Campaign: pab2, hil rig, master 47b3fde. Pilot: `guardrail.prefix_consumer.PrefixConsumer` around the constant law `vy = 4 m/s`.

Everything below is a headless reproduction from the run folders in `demo/out/hil_smoke/pab2_0{08,10,11,13}_corridor-drift-out/` (`flight_log.jsonl`, `shield_audit.jsonl`, `vla_prefix/csp_g0.json`). Nothing was flown for this note.

## Summary

- **What escalated:** the Shield found no repair for the consumer's action and braked (X1, "no operator converged"). Each time, the action's straight 5 s path left the corridor through the round cap at its END, vertex (60, 60), by 0.005 to 0.30 m.
- **Consumer side (fixed on this branch):**
  - The consumer picks the nearest admissible fan heading. By construction, that heading's path grazes the corridor wall.
  - It decides on the pose it received. The Shield judges the action from the pose of its own tick, which is the same tick or up to one 0.1 s tick newer, so the aircraft has flown on.
  - Its wall test also used the Shield's own tube. That tube *contains* the true tube (20.024 m for a 20 m half-width), which is the wrong side for an admissibility test.
- **Shield side (not changed, pinned):**
  - CorridorReturn judges the drift by the rate toward the *nearest* centerline point. Past the last leg, that point is the end vertex.
  - A path can close on the vertex first and leave the cap later. The operator then sees "returning" and leaves the action unchanged, so the chain falls through to Brake.
- **Fix:**
  - A path that starts inside the tube must also stay `CORRIDOR_MARGIN_M = 1.0` m inside the wall. The inner tube is buffered by `half - 1.0` with plain chords, so it lies inside the true tube.
  - A path that starts in the 1 m band next to the wall must end back inside the margin, and must never pass the true wall on the way (a tube buffered by the plain half-width, plain chords). Without that second rule a band start at the dog-leg's inner corner could pass the corner up to 0.024 m past the true wall (added after review, see below).
  - The margin is the larger of 1.0 m and the CSP's horizontal speed cap times 0.25 s (`corridor_margin_m`; 1.0 m for the 4 m/s cells flown). The consumer record carries it as `corridor_margin_m`, and the harness refuses a flight whose announced margin differs from the arm's.
  - The consumer record's `corridor_path_test` is now `"segment_margin"` (was `"segment"`).

## pab2_011 as logged

| t (s) | tick | Pose (x, y) | Consumer action (world) | Shield | FSM |
|---|---|---|---|---|---|
| 24.686 | 247 | (56.700, 70.873) | (4.0, 0.0) | pass | Normal |
| 24.780 | 248 | (57.096, 70.909) | (4.0, -0.0006) | violation "20.3 m off at t+5.0 s"; CorridorProjection "left the action unchanged" x3 passes; Brake | X1 |
| 25.484 | 255 | (59.847, 70.924) | (3.865, -1.030) | the same, "20.0 m at t+5.0 s"; Brake | G3 Loiter |
| 28.781, 29.883 | 288, 299 | in the cap | the request still exits the cap | Brake (in Loiter) | - |
| 33.083 | 331 | (59.843, 70.923) | (3.864, -1.035) | Brake | G4 RTL: "N-in-T persists: 3 onsets in 5 s since Loiter entry" |

The corridor is `(0,0) -> (60,0) -> (60,60)`, half-width 20 m, so its cap reaches y = 80. nfz-school was out of force (the rule clock read Saturday 09:00).

## Reproduction

- **Shield:** `guardrail.replay.rail_shield(policy, lookahead_s=5, dt=0.1)`. At each logged pose and measured velocity, it brakes on all 5 Brake-fallback ticks of pab2_011 (248, 255, 288, 299, 331) with the same operators the audit shows. It does the same with the velocity dropped.
- **Consumer:** at the pose of tick 247, the wall-only consumer (as flown, `"segment"`) asked for `vy = 4`. It flies exactly the logged `(4.0, 0.0)`, the 90-degree fan heading:
  - From tick 247's pose, that path stays inside the 20.024 m tube.
  - From tick 248's pose, it ends 20.28 m from the centerline.
- **Pose order:** the consumer's action changed on 25 ticks. On 22 of them the logged action is reproduced from the pose of the same tick, and on 12 from the tick before (some match both). Two ticks in the first 2.1 s match neither.

## The same in every prefix-on corridor flight of pab2

| Run | Seed | Brake-fallback ticks | X1 "no operator converged" | Ended |
|---|---|---|---|---|
| pab2_008 | 1001 | 6 | 2 | time cap, success |
| pab2_010 (A/A) | 1001 | 11 | 4 | time cap, success |
| pab2_011 | 1002 | 5 | 1 | RTL (G4), then home (G9) |
| pab2_013 | 1003 | 12 | 4 | time cap, success |

- **Where the paths exit:** all 34 Brake-fallback ticks are exits through the end cap. Their worst point is at t+5.0 s, 0.005 to 0.30 m out, and the nearest centerline point is (60, 60).
- **pab2_013 tick 177 is on the leg itself:** (42.3, 49.4), asking (0, 4). Its radial rate is zero, but the path ends 20.026 m from the end vertex.
- **One more X1 in pab2_010 (tick 67, 6.7 s), "repair over theta 3.62 m", at the dog-leg's inner corner:**
  - From (28.895, 13.623), the 60-degree heading passes 20.021 m from the centerline at t+3.2 s.
  - The wall test admitted it (inside 20.024 m).
  - The Shield's distance check flags anything past 20.0 m. CorridorReturn sized it 3.62 m along the nearest leg, which is pab1's corner overstatement and is already pinned.
  - No pose lag is involved. This is the tube-superset gap.

## The Shield's blind spot (documented, not changed)

`Shield._repair_corridor` sizes a forecast exit as `d + outward x L - half`, with "outward" taken from the direction to the nearest centerline point at the current pose.

- **At the round end cap**, that direction points at the end vertex. A straight path past the vertex first closes on it (inward rate > 0) and leaves the disc later.
  - At pab2_011 tick 248, the rate is +1.03 m/s inward while the endpoint is 20.28 m out.
  - The own-velocity branch does not fire either, because the inward test `inward >= need` passes with `need = 0`.
  - So the operator leaves the action unchanged, the chain does not converge, and the Shield brakes (X1, then G3 or G2, then RTL via G4 on a persisting N-in-T).
- **Why this is safe, and its cost:** it is safe, because it is a stop and never an escape. P0 escapes were 0 on all four flights. But it is an escalation a lateral repair could have avoided.
- **The pin:** `tests/test_prefix_consumer.py::test_the_shields_end_cap_blind_spot_is_pinned_as_found` pins it at pab2_011 tick 248 and pab2_013 tick 177.
- **Why it is left alone:** changing it would move the certified ntFIN2 and hil readings, so it waits until after 30 Nov, like pab1's corner sizing.
- **A possible repair, not built:** read the excursion and its direction off the forecast's deepest point (the direction to the centerline from that point), as the own-velocity branch already reads its depth.

## The fix and its evidence (headless only)

`guardrail/prefix_consumer.py`: `CORRIDOR_MARGIN_M = 1.0`, `CORRIDOR_LAG_BUDGET_S = 0.25`, `corridor_margin_m`, `CORRIDOR_PATH_TEST = "segment_margin"`, and the band rule. `experiments/sweep_scenarios.py`: `prefix_flight_problems` also compares `corridor_margin_m`. Zone rings are unchanged. Of the five prefix_ab cells, only corridor-drift-out has a corridor, so the other cells fly the same as before.

### Tests (`tests/test_prefix_consumer.py`, 43/43 pass)

- `test_the_turn_keeps_a_margin_off_the_corridors_end_cap`: pab2_011 tick 248.
  - **Null:** the wall-only consumer flies the logged action, and the Shield brakes on it from tick 248's pose.
  - **With the margin:** the consumer refuses that action and flies a 105-degree turn instead. The Shield leaves the new action alone from both poses and from 24 poses 0.5 m and 1.0 m away, each tested with no velocity and with tick 248's measured velocity (the Shield's own-velocity forecast). That is one measured velocity, not every one: the margin's guarantee is geometric, for the commanded straight path.
- `test_from_the_band_at_the_inner_corner_the_path_keeps_the_true_wall` (added after review): from (39.5, 19.4) the 52-degree heading starts in the band, stays in the Shield's tube, ends inside the margin, and passes the inner corner (40, 20) 20.017 m out. The end-point rule alone admitted it. The Shield flags the corridor (its only violation) and CorridorReturn sizes it 15.16 m, over theta 2.0. The band rule now refuses it.
  - The review's own probes, (39.5, 19.8) at 24 degrees, (37.5, 19.2) at 18 and (39.0, 19.8) at 12, are the same geometry and are refused now too. But there the Shield's violation is `nfz-school` (a zone this cell's CSP does not list), and its 0.1 s-sampled corridor check stays under 20.0 m. Their CorridorReturn repairs belong to the zone's repair chain, not to the corridor.
- `test_the_margin_follows_the_speed_cap_and_is_recorded`: 1.0 m at the 4 m/s cap, 2.0 m at 8 m/s, never under 1.0 m, and in the record.
- `test_the_wall_test_admitted_what_the_shields_distance_check_flags`: pab2_010 tick 67.
- `test_from_the_margin_band_the_path_ends_back_inside_it`: the band rule.
  - A path parallel to the wall is refused.
  - A path heading back inside is admitted.
  - Stopping stays admissible.
  - The inner tube's vertices are no further than `half - margin` from the centerline, and the true tube's no further than `half`.
- `test_closed_loop_a_late_pose_holds_no_loiter`: the corridor arm closed loop with the consumer fed a pose 1 or 2 ticks old.
  - **Null:** the wall-only consumer at 1 tick of lag hits X1 and repairs.
  - **With the margin:** no repair and no FSM edge.
- `test_the_shields_end_cap_blind_spot_is_pinned_as_found`: the Shield half.
- **Unchanged:** `test_closed_loop_the_corridor_arm_holds_no_loiter`, with no lag, still flies 0 repairs and no edge.

**Lag sweep.** Headless corridor-drift-out, prefix-on, seed 1001, 650 ticks (65 s, the hil cap). The consumer is fed a pose `lag` ticks old. Cells are repairs per episode; P0 escapes were 0 in every run.

| Response | Margin | lag 0 | 0.1 s | 0.2 s | 0.3 s |
|---|---|---|---|---|---|
| ideal | none (as flown) | 0 | 33 | 38 | 41 |
| ideal | 0.5 m | 0 | 0 | 28 | 32 |
| ideal | **1.0 m** | 0 | 0 | 0 | 17 |
| ideal | 1.5 m | 0 | 0 | 0 | 0 |
| ardupilot | none (as flown) | 0 | 13 | 19 | 25 |
| ardupilot | 0.5 m | 0 | 0 | 7 | 13 |
| ardupilot | **1.0 m** | 0 | 0 | 0 | 4 |
| ardupilot | 1.5 m | 0 | 0 | 0 | 0 |

- **Every repair column with repairs also carries FSM edges:** X1 or G1, then G3 or G2 Loiter.
- **What the margin covers:** a margin covers pose lag up to margin / speed (1.0 m / 4 m/s = 0.25 s).
- **The lag on the rail:** pab2's logs show the deciding pose 0 to 1 ticks behind the Shield's.

**Open-loop log replay** (scratch script, not a test). For each logged tick, the consumer decides on the previous tick's pose, and the replay Shield judges the action at the logged pose, velocity and rule clock.

| Run | Brake fallbacks, wall-only | Brake fallbacks, margin 1.0 |
|---|---|---|
| pab2_008 | 28 | 0 |
| pab2_010 | 19 | 0 |
| pab2_011 | 10 | 0 |
| pab2_013 | 22 | 0 |

- **What remains with the margin:** 2 to 7 CorridorReturn repairs per run, under 0.6 m, at t of about 3 to 4 s. There, the logged velocity is the old flight's reversal, so the own-velocity forecast disagrees with a different command.
- **Limit of this replay:** it replays states the new consumer would not have flown, so it is not a claim about a new flight.

## What this does not show

- **No flight:** nothing here was flown. The hil result of the fix is unknown until the re-fly below.
- **The tables above predate the band's true-wall rule:** the lag sweep and the open-loop replay were run with the end-point band rule alone. They were not re-run; `test_closed_loop_a_late_pose_holds_no_loiter` (1 and 2 ticks of lag, no repair, no FSM edge) passes with the final rule.
- **The Shield's own-velocity forecast:** the margin is a guarantee for the commanded straight path only. The end-cap test checks one measured velocity (tick 248's); the open-loop replay's remaining CorridorReturn repairs come from that branch.
- **Pose lag above 0.25 s:** pose lag longer than that at 4 m/s, or a response lag that carries the aircraft further than 1 m off the commanded path, can still reach the Shield's blind spot. The 0.3 s column above shows it.
- **The rig checkout:** the AGX checkout must carry this commit. A rig still on 47b3fde announces `corridor_path_test: "segment"`, and the harness refuses its flights (`prefix_flight_problems`, "path test").

## Rig steps owed (integrator)

1. **Sync the AGX checkout** (git bundle, as for pab2) to the merged commit.
2. **Check the record:** `python experiments/sweep_scenarios.py --profile prefix_ab --backend hil --hil-dry-run --only corridor-drift-out`. Confirm the prefix-on arm's record says `corridor_path_test: segment_margin` and `corridor_margin_m: 1.0`.
3. **Re-fly:** `python experiments/sweep_scenarios.py --profile prefix_ab --backend hil --hil-campaign pab3`.
   - The full profile is preferred: the consumer's source pin changed for every prefix-on arm.
   - The minimum is `--only corridor-drift-out`.
4. **Re-score and report:** follow `docs/data/kpi_prefix_ab_20261117/README.md`, using `pab3_*` and baseline pab2.
5. **Read the corridor prefix-on runs:** look for X1 "no operator converged" and the end of each flight.
