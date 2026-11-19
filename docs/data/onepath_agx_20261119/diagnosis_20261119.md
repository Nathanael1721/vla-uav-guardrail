# Diagnosis of the 19 Nov 2026 AGX CityLife follow runs (code bfb4973)

Produced by an independent read-only investigation (three investigators, each claim re-checked by two adversarial verifiers, one synthesis) over the run logs; no file was changed and nothing was flown for it.

**Sources and how to read this.** Every number below comes from claims that two adversarial verifiers re-checked. Where a verifier refuted or corrected a claim, the corrected version is used. Each statement is marked measured (re-computed from the run files) or inferred (an interpretation, or something the logs cannot show directly).

Run folders are in `demo/out/<tag>/` of the lab checkout (gitignored). Frames, PIP videos and the full runs are in `F:/vla_drone_offload/jetson_runs/<tag>/`. The 3 Nov and 4 Nov baselines are in `D:/` and `E:/vla_drone_offload/jetson_runs/`. The investigation's scratch scripts were not kept in the repository.

**One fact I checked myself (read-only git).** The FenceGuard stand-off rule already existed in the 4 Nov code (989f967). In `git show 989f96724b74:demo/follow_vlm.py`, `FenceGuard.gate` contains `d_min = d` (line 2445) and `if d <= self.stand_off_m:` (line 2457). These are the same lines as 2703/2714 at bfb4973. So the gate stall described in question 2 is not new in the 18 Nov change.

---

## 1. Why agx_onepath_shield_03 scored 0.472 (01/02/04/05 scored 0.917-0.969)

### Answer
The cause was perception and identity: the strict identity lock rejected the correct car box. A search and reacquisition sequence then lasted almost a full lap of the car. The guard (FenceGuard on the Shield ruler) was not active when the car was lost.

### Surviving evidence
All files below are in `agx_onepath_shield_03/` unless another path is given.

**Size of the loss (measured; `flight_log.jsonl`, `metrics.json`)**
- 2217 of 4195 ticks were beyond 30 m, so 0.4715 were within 30 m. `metrics.json` reports 0.472, with sep_mean 72.4 m.
- 2131 of those ticks form one block, t=192.7-405.7 s, peaking at 224 m.
- The other 86 ticks are:
  - 64 at t=0.4-6.6 s
  - 21 in short pieces at t=74.05-76.85 s (30.3-31.5 m)
  - 1 at t=192.19 s (30.07 m)
- The drone was back within 30 m from 405.79 s and ended at 11.0 m.

**The lock drop (measured; `detections.jsonl`, `metrics.json` reacquisition.events)**
- The lock dropped at t=186.19 s (seq887, lock_why "nothing where the subject should be").
- Car_10 had finished its westward turn at the corner (201.5,-37.5) about 1.6-4 s earlier. It was at (189.3,-37.5), 17.3 m from the drone.
- Mode sequence: coast 186.29, search 188.30, plan 193.20.
- The lapse event is at 187.78 s with anchor P (199.4,-33.6). That anchor is about 15 m east of the true car position.
- Reacquired at 407.31 s (after_s 219.53) on the correct car: rng_h 25.0 m (truth 25.7 m), streak 8.
- This was the only lapse event. There was an earlier 1.8 s lock gap at t=122.3-124.1 (seq769-771) that recovered without a lapse.

**The rejected boxes were the real car (measured)**
- Seq887 had an ok-tier box: score 0.40, cx 243.7, w 103.3, P (194.65,-36.37). It was 1.08 m from Car_10's truth at capture time t≈184.55.
- Seq888 was 1.52 m from truth and seq889 1.19 m.
- These errors match the accepted boxes just before (seq885 1.52 m, seq886 1.23 m).

**Why it was rejected: the pixel gate**
- The lock was still fresh at seq887: 1.50 s since the last pick, under the 2.0 s hold.
- The prediction from the held box (seq886: cx 323.8, yaw -1.581 rad) at seq887's capture yaw (-1.617 rad) is cx 337.9.
- The car box was 94.2 px from that prediction, and its duplicate 94.0 px. The gate is 92.16 px (0.12 × 768), so both missed by about 2 px. The width ratio, 1.21, was within the limit.
- At seq888 (still fresh, 1.89 s) the box was 226.6 px off.
- From seq889 (2.21 s) the lock was stale, so the reference was the estimator prior. That prior is not logged, so the earlier "260.8 px" figure is not what the flight tested.
- **Inferred:** this assumes the defaults `--lock-gate 0.12`, no `--linear-bearing`, and a 90° camera on the Jetson. metrics.json does not record these.

**Detector timing at the corner (measured)**
- Run 03 had 10 inferences over 700 ms in a row (seq878-887, 772-1050 ms) with capture gaps of 0.93-1.33 s.
- No other run had more than 4 in a row.
- The whole-run median was 271-277 ms depending on counting, similar to the other runs.
- Single slow gaps did not break the lock elsewhere:
  - Run 02: 913 ms and a 1.33 s gap; the box was 58.4 px from the prediction.
  - Run 01, second pass: 826 ms and a 1.07 s gap; 40.7 px.
- The comparison runs were not uniformly fast at the corner either: 04 had an 898 ms inference with a 1.20 s gap just before the turn, and 05 had 561-633 ms.

**Same scene in every run (measured)**
- Car_10 makes the same west turn in every run.
- On the first pass the drone entered the corner 16.4-20.4 m behind the car (01, 02, 03, 04, 05, agx_follow_01, hier_wgs84).
- Every run except 03 stayed in track mode with no lock refusal from 10 s before to 30 s after the corner. Even so, 01, 02, 04, 05 and agx_follow_01 briefly reached 30.7-32.0 m in that window.

**Slow inferences across runs (measured)**
- All five onepath runs had inferences over 700 ms: 01 20, 02 37, 03 47, 04 33, 05 52. Medians were 250-274 ms.
- This is not a whole-day effect. The hier run flown afterwards on the same AGX and code (agx_hier_aerovla_wgs84_20261119, about 18:50) had 0 slow inferences and a 223 ms median. The nfz3b_wgs84 run had 5 (median 255 ms).
- On 3 Nov, agx_follow_01 had 3 (median 225 ms), and other 3 and 4 Nov runs had medians of 228-294 ms with 0-6 slow inferences.
- The extra time is in the GPU forward pass (about 740-800 ms on slow inferences); preprocessing is about 40-52 ms.
- Shield-step median: 3.5 ms for agx_follow_01 and onepath 01-03, 3.9 ms (04) and 3.7 ms (05). The p95 rose from 7.6 to 8.4-9.8 ms.

**Reacquirer replay (measured; `Reacquirer(need=4, max_range_m=45)` replayed on `detections.jsonl`)**
- The replay matches the logged streak and n on all 597 inferences.
- First streak, seq910-914 (captured about 195.2-197.5 s, rng_h 36.0-44.5 m): refused as "short pass". The search reach had passed 60 m (61.6 m at seq912), so 8 sightings were needed. At seq915 (46.5 m) the car was "too far".
- Second streak of 10, seq996-1005 (captured about 229.1-234.1 s): refused as "short pass" at 4-7, then "not moving" at 8-10 (shift 0.24/0.56/2.30 m against bars of 3.04/3.13/2.95 m).
- Truth shows the car standing at (58.3,-37.5) from about 227.6 s to about 231.6-232.2 s. At seq1006 (45.5 m) "too far" reset the streak.
- Commit at seq1487 with streak 8 (capture about 406.8-407.1 s).
- With seq886 as the anchor time, the replay also matches every logged far value. With seq885, one verifier found 37 far-count mismatches.

**Search behaviour (measured; inferred where marked)**
- The planner target stayed at the junction (205,-41) from 194 s to about 406 s, in hold mode from about 204 s.
- From about 201 to 243.7 s the drone flew west to x≈80, away from that target, toward car sightings beyond 45 m (far_approach_ticks 498). That this was the far-lead override is inferred from the code plus the unchanged plan target.
- Best truth distance on that flight: 40.2 m at 231.5 s.
- After the last far sighting (about 242.0-242.1 s) it went back to the junction, within 2 m by 298.1 s.
- At about 389 s it saw the car standing at (202,58), about 97 m away. It left the junction at 392.1 s, flew north to about (205,17) by about 404-406 s, and the car drove toward it from 401.8 s.
- Separation dropped under 30 m at 405.8 s. The car never came back past the junction.
- Car stops (truth): (58,-38) 227.6-232.2 s; (44,24) about 254.8-281.6 s; (106,120) 330.6-338.8 s; (202,58) about 387.3-401.9 s.

**Guard state at the loss (measured)**
- t 180-193 s (130 ticks): fence_mode "clear" on all ticks, fence_scale 1.0, fence_hold false.
- The Shield touched only 7 ticks in that window, all kin-caps YawClamps (search spin of 46-59°/s cut to 45°/s, vx/vy unchanged).
- `braked` is false on every tick of the run. It is the Shield's flag, not FenceGuard's, so it says nothing either way.
- 193-400 s: fence modes were clear 1786, near 126, skirt 157, hold 1. The building brake was on for 211 ticks, all after the drone was already beyond 30 m.

**Old guard vs new guard, open-loop (measured; `tools/onepath_guard_replay.py`)**
- There are 28 brake-changed ticks, all "grid free, Shield brake":
  - 5 before the loss (t=73.9, 100.9, 123.0, 130.6, 170.4), in track mode, 14.9-28.9 m from the car
  - 22 at t=201.4-288.1, in plan mode, car 45.6-158.7 m away
  - 1 at t=415.2, back in track, 13.6 m
- None falls in 170.4-201.4 s.
- Run 01 has 8 such ticks, all in track mode at the same kinds of corners.
- Commanded travel removed by the Shield-ruler guard compared with the grid guard:
  - Run 03: 6.63 m over 130 ticks, always slower and never faster. By window: 2.67 m before 183 s, 0.00 m in 183-193 s, 3.49 m in 193-406 s (of 370 m commanded), 0.47 m after.
  - Run 01: 7.59 m over 157 ticks.
- Caveats: open-loop; commanded rather than flown; sideways skirt changes are not counted; the replay uses the repo yaml, not the signed bundle.

### Is the one-path change (guard_field "shield") a regression here?
**No, as far as the logs can show.**
- The loss was set off by the identity lock's pixel gate during a sustained run of slow detector cycles, while the guard was clear at scale 1.0.
- No guard difference falls in the loss window.
- The same code passed the same corner in 01, 02, 04 and 05.
- Not excluded: closed-loop knock-on effects, because the replay is open-loop.

### CHANGELOG sentences
- "agx_onepath_shield_03 (0.472) lost the red car once, at t=186 s just after its west turn at (201.5,-37.5): during ten consecutive ~1 s detector cycles the strict identity lock rejected the correct car box by 2 px over its 92 px pixel gate, while FenceGuard was clear at scale 1.0."
- "The reacquirer then refused the car as 'short pass', 'not moving' (it was stopped) or 'too far' until t=407 s; a replay of the Reacquirer on the logged detections reproduces every logged streak and the final commit."
- "None of the 28 ticks where the Shield-ruler brake differs from the old grid guard falls in t=170-201 s, so this is not a regression of the one-path change."

---

## 2. Why nfz3b_agx_hier_aerovla_wgs84_20261119 scored 0.345 (controller_fallback 53 %)

### Answer
The car was lost while FenceGuard held the drone still beside nfz-north-street. Once the drone was inside its own 3 m stand-off from that zone's margin ring, `FenceGuard.gate()` returned scale 0 for every velocity, and `slide()` offered no sidestep along the commanded aim. The drone sat there while the car drove about 100 m on and turned the corner. The search planner then waited at a junction until the car came round its loop.

This was neither a Safety Shield failure nor the guardrail correctly refusing to follow into a zone. The car never entered a zone, and the way along the zone was open: the VLA later flew it with no Shield violations. controller_fallback is a result of the loss, not its cause.

### Surviving evidence
All files below are in `nfz3b_agx_hier_aerovla_wgs84_20261119/` unless another path is given.

**Size of the loss (measured)**
- 1447 of 4189 ticks were within 30 m, so 0.3454. Of the 2742 far ticks, 2609 (95.1 %) form one stretch, t=89.1-350.2 s (minimum 30.9 m, peak 224.2 m).
- The rest are 58 ticks at 1.3-6.8 s, 2 at 412.3-412.4 s and 73 at 412.8-419.9 s.

**The policy (measured)**
- `policies/follow_car_citylife_nfz3b.yaml` v0.1.3, which matches the flown bundle (hash sha256:5097…b681, signature verified).
- Three hard P0 polygon zones, each with a 1 m margin and altitude 0-60 m:
  - nfz-frontage: x 45-62, y 62-92
  - nfz-north-street: x 100-124, y 122-134
  - nfz-south-block: x 145-175, y -34 to -26
- Other rules: standoff-pedestrian 10 m, standoff-any 5 m, bld-clearance 3 m, alt-band 6-14 m, kin-caps (priority P1, constraint_type hard).

**The car and the zones (measured)**
- The car's centre point is never inside a zone polygon in any of the three runs.
- Its lane runs 0.5 m from frontage (inside the 1 m margin on 204 ticks), 2.5 m from north-street and 3.5 m from south-block.
- On 19 Nov the car was inside the drone's 4 m keep-out band on 749 ticks: frontage 235, north-street 407, south-block 107. The drone was within 30 m on 315 of those (113, 202 and 0 respectively).
- The 3 m stand-off is the code default (`--fence-standoff 3.0`); the run files do not record it.

**Line of sight across a zone (measured)**
- The drone-to-car line crosses a raw zone polygon on 220 ticks (19 Nov), 232 (4 Nov) and 1333 (3 Nov). None of these ticks is within 30 m, and all come after the car was already lost. So a zone behind the car does not explain the loss.
- The zone did cause the loss, through the band hold described below.

**Whether following past the zones is possible (measured)**
- Frontage was passed within 30 m on the first lap in all three runs. The drone flew about 5 m from the zone, 13-19 m behind the car with a few metres of sideways offset. It did not fly beside the car.
- North-street was passed within 30 m only on 4 Nov.
- No run followed the car past south-block.

**The hold (measured)**
- fence_hold was true, mode hold, cause fence, on 403 consecutive ticks (793-1195, t=79.79-119.95). fence_d was 2.62-3.00.
- From 82.5 to 114.4 s (320 ticks) the drone sat at x 100.35-100.55, y≈118.3, under controller_follow authority, with the car seen on every tick, fence_scale 0 and emitted velocity 0.
- In that time the car drove from (109.7,119.5) to (201.5,106.5) and separation grew from 9.4 to 101.6 m.
- On the last 55 ticks (114.5-120.0 s) vla_search had authority. The VLA command was emitted at up to 4.42 m/s even though guard_in was 0. The drone flew east along the zone to x 113.4 with no Shield repairs while fence_hold stayed true.
- Over the whole hold, controller_follow had authority on 327 of 403 ticks.

**Why it stalled (measured: code, plus open-loop replay with `onepath_guard_replay.flight_setup`)**
- `gate()` seeds `d_min` with the current distance, so once d ≤ 3 m it returns 0 for any speed ≥ 1e-3, including moving straight away from the zone. The fence branch has no retreat allowance; the building branch does.
- Along the commanded clear_aim (-7.7°) the way ahead is clear: probes at 4/8/12 m are 3.19/3.72/4.26 m from the zone. So `slide()` returns no sidestep and the flight loop holds.
- Moving straight away also holds, but through `slide()`'s other branch: buildings breach clearance 8-12 m ahead and no side is found.
- 7 of 16 headings, including straight along the street (+x), would have skirted.
- FenceGuard never released the hold itself.

**How the drone got inside the stand-off (measured; inferred where marked)**
- The controller brought the drone to about 3.19 m from the zone by 75.06 s.
- The VLA (vla_search, 75.54-77.67 s) asked for up to 2.13 m/s toward the zone. The Shield removed that component with GeofenceSlide on 20 ticks, and zone distance went from 3.18 to 3.30 m.
- The building skirt (77.78-78.66 s, cause obstacle) did not reduce zone distance during its own ticks (3.31-3.36 m). The logged fence_d of 5.79-6.81 there is building distance, not zone distance.
- On the "near" ticks (78.76-79.66 s) guard_in pointed away from the zone, but the emitted vy was still +0.78 falling to +0.03. That this is the smoother's leftover of the skirt command plus vehicle lag is inferred. Zone distance went from 3.34 to 3.03 m.
- The hold began at 79.79 s (3.00 m).
- At 79.90-80.00 the Shield's ClearanceFix/StandoffRecover pushed +1.25 m/s toward the zone ("bld-clearance dist 2.69m < 3.0m"). The drone then drifted to about 2.65 m.

**After the loss (measured)**
- The lapse was at 115.02 s. The car estimate was (199.4,118.0) while the drone was still held, 101.6 m behind.
- The planner pursued the junction (205,123) from 121.55 s and held there from 151.5 to 351.9 s.
- From 184.1 to 316.1 s (1321 ticks) the drone moved less than 3 m, all plan mode with controller_fallback "expired".
- Reacquired at 352.02 s (after_s 237.0), 26 m away, after the car had driven its loop.

**controller_fallback (measured)**
- 2213 ticks (52.8 %): "expired" 2170 and "no_answer" 43. All 43 no_answer ticks are at take-off (0.4-4.6 s).
- The main fallback block runs 138.6-354.5 s (2158 ticks). It starts about 24 s after the car was last seen (114.5 s) and about 50 s after the separation passed 30 m (89.1 s).
- No VLA request was sent between 135.7 and 352.8 s (a 217 s gap; 174 requests in total).
- Whether fallback flying lengthened the loss cannot be told from the logs.

**Shield record (measured)**
- p0_escapes 0. There were 49 P0 violation ticks, all corrected: north-street lookahead 20, bld-clearance 27, standoff-any 2.
- nfz_entered false, nfz_s 0.0, mission contacts 0.
- Closest approach to each zone: north-street 3.62 m (403 ticks under 4 m, none under 1 m), frontage 4.67 m, south-block 44.62 m (never tested).
- **This is the Shield's own count, not ground truth.** Against the true car position, standoff-any (5 m, horizontal only) was breached on 102 ticks, minimum 0.22 m, and flagged on 2 (metrics.json standoff_score: fn 100, recall 0.02). The drone was at about 8 m altitude.
- standoff-pedestrian never applied (bound_ticks 0).

**Comparison with 4 Nov, 0.494 (measured; `D:/vla_drone_offload/jetson_runs/nfz3b_agx_hier_aerovla_20261104`, code 989f967)**
- Same pattern: one loss, then a wait for a lap. The loss was not zone-related.
- The car was lost at t=129.5 s at the stop at (201.5,58.3). That point is 87.8-88.4 m from the nearest zone (south-block), not 77 m. The drone was about 5-6 m from the car, then overhead.
- From 140 to 153 s the undetected car passed the drone at 3-10 m.
- Hold at (205.0,41.9) from 165.0 to 355.6 s (about 1905 ticks, controller_fallback, no fence holds). Reacquired at 362.66 s.
- Fence-cause holds totalled only 49 ticks (t 40-42, 78-80, 86); obstacle-cause holds 171.
- At north-street the drone stayed outside the 4 m band (closest 4.12 m at 71.45 s) while the car stopped at (105.5,119.5).
- The 19 Nov run's second pass of the same leg-3 stop also lost the car, at 6.3 m (t 384.6 s, 22 s unseen).

**Comparison with 3 Nov, 0.152 (measured)**
- The car was last seen at 52.47 s, within the window holding the run's only 23 fence-cause hold ticks (t 40-55).
- Off the old obstacle map from 68.04 s, for 3484 of 4160 ticks.
- Plan mode on 78 % of ticks. The drone was in box (41-47,142-145) on 2070 ticks.
- From about 340 s it sat at (64.2,145.8) against BP_jctBldSetK33_6: 1919 mission contacts from 351.53 s, plus 657 after the mission.
- The old map plausibly prevented recovery. It did not cause the first loss, and whether the fence holds did is not established.

**Counterfactual (inferred, not flown)**
- Without the hold, the drone would probably have passed north-street as on 4 Nov.
- The 101 m gap explains the loss at the corner, not why it lasted a lap: 4 Nov lost the car from 6 m and still waited 233 s.
- The likely gain is therefore uncertain, probably toward the 4 Nov level rather than a full follow.

### Is the one-path change a regression here?
**Not shown, as far as an open-loop replay can tell.**
- All 403 stall ticks are cause "fence".
- The fence half of `gate()` is shared by both guards, and its stand-off rule is identical in the 4 Nov code (my git check above).
- Building clearance along y 117-118.3 is at least 3.5 m on both guards.
- On the commanded clear_aim, both guards hold on all 7660 probes, and the building skirt chose the same side under both.
- The guards are not identical everywhere: at 210-250° (pointing away from the car) grid would skirt where Shield holds (2.1 % of probes). Over the whole flight there are 41 mode or side differences (for example t 1.93, 33.5, 62.7). A closed-loop contribution from those cannot be excluded.
- Why 19 Nov had 403 fence-hold ticks against 49 on 4 Nov is a path difference (VLA requests, skirt carry-over, Shield push). Whether the guard change shaped that path is undetermined.

### CHANGELOG sentences
- "nfz3b_agx_hier_aerovla_wgs84_20261119 (0.345, 0 P0 escapes, no zone entered) lost the car while FenceGuard held the drone for 40 s (403 ticks, cause 'fence') 2.65 m from nfz-north-street's margin ring: inside the 3 m stand-off gate() returns 0 for every velocity, including retreat, and slide() finds no detour along the commanded aim, while the car drove 100 m on."
- "This stand-off rule is unchanged since at least 989f967 (4 Nov), and an open-loop replay gives the same hold under the grid and Shield guards; controller_fallback (53 %) is the planner waiting at a junction after the loss, with no VLA request for 217 s."
- "The Shield's '0 P0 escapes' is its own assessment: against ground truth the 5 m standoff-any rule was breached on 102 ticks (minimum 0.22 m horizontal) and flagged on 2."

---

## 3. After-mission contacts (landing)

### Answer
All 7 flights on 19 Nov had 0 contacts inside the mission window. The after-mission contacts all fall in the after-mission bucket. That bucket covers four stages:
1. the Shield-filtered flight to the landing site
2. the descent, flown without the Shield
3. land_async and touchdown
4. time on the ground until metrics are written

Per-contact times are not saved, so I cannot say which stage, or which pedestrian, produced which contact. I also cannot say whether a pedestrian walked into the drone or the drone came down on one.

Two flights list a pedestrian among their after-mission objects. Both landed at site (192,30), where level pedestrians walked within a few metres while the drone was in its last 2-4 m. The landing phase is outside the guardrail's pedestrian awareness. That is a real, open safety gap, but it predates 18 Nov.

### Surviving evidence

**Contact counts per run (measured; `metrics.json` collisions)**

| Run | pre_t0 | mission | after_mission | After-mission objects |
|---|---|---|---|---|
| nfz3b_wgs84 | 1 | 0 | 132 | BP_CityPed_F3_C_4, BP_jcRoad22mC_a68_17 |
| agx_hier_wgs84 | 1 | 0 | 1 | SM_jcBldBase60b_97 |
| onepath_01 to 04 | 1 | 0 | 1 each | SM_jcBldBase60b_97 |
| onepath_05 | 1 | 0 | 6 | BP_CityPed_M2_C_5, SM_jcBldBase60b_97 |

**What is and is not logged (measured; `demo/follow_vlm.py` 512-542 and 5243-5253)**
- `_on_collision` keeps each contact (t_conn, object, penetration, impact) only in memory.
- `collisions_summary` writes:
  - pre_t0 as a count only
  - mission and after_mission as counts
  - the object-name sets for mission and after_mission, with no per-object count
  - mission_first: times for the first 10 mission contacts only
- No other run file holds contact data.

**Landing-site choice and descent (measured)**
- `demo/landing.py` choose_site/is_landable check only off_map, map_edge, street, 2-4 m clutter, 6-14 m obstacles and car-lane centrelines (2.1 m). There is no pedestrian input.
- follow_vlm calls it with prefer="pavement", which ranks sites at least 9.2 m from a road centreline first. The level's pedestrian walking line is at 9.5 m ± 0.25 (`tools/citylife_peds.py` PAVEMENT_OFF_CM 950).
- Both 19 Nov sites, (192,30) and (194,-28), are 11.0 m from a centreline and about 1.5 m from modelled pedestrian route 24. Final positions were 1.1-2.2 m from it. This uses the planned route model, not live positions.
- The transit to the site goes through the Shield only (filter_world_as_body, set_subject(None)). shield_touched was 77 in nfz3b and 0 in hier.
- The descent loop (until 1.2 m) and land_async go straight to the simulator without the Shield. The HUD reads "Shield not in this loop".
- Neither policy has a people distance_envelope. standoff-pedestrian had bound_ticks 0 on all 7 runs. The 40 level pedestrians were polled once, for scoring only.

**nfz3b landing record (measured)**
- Start (201.81,8.59) was not landable (6-14 m obstacle 2.84 m < 3.0; lane 0.31 m < 2.1).
- Site (192,30), path 23.6 m, reached. Pre-descent final position (191.89,29.32), 0.69 m from the site, final_landable true.

**nfz3b recorder timeline (measured; `view/recorder.json`, FPV HUD)**
- frame_t is the HUD time, not the capture time.
- mission_end is at 420.314 s, and there are 597 post-mission frames (8399-8995).
- Transit to the site runs to frame 8768 (438.731 s, 8.1 m).
- The descent HUD starts at frame 8769. Readings: 7.7 m at 439.4 s, 3.7 m at 444.1, 2.1 m at 446.8, 1.5 m at 447.9, 1.2 m at 448.4.
- The HUD is then frozen at 448.393 s for the last 42 frames (8954-8995), which are still live camera images.
- **Touchdown is not recorded.** Between TPS 8988 and 8989 the scene changes (pedestrians and truck vanish; mean pixel difference 22.3 against about 4 between other frames). So touchdown falls in an unrecorded gap.

**nfz3b: pedestrians seen near the drone (measured from frames, with pixel-scale estimates)**
- A pedestrian crossed the touchdown spot at about frames 8860-8902, when the drone was 4.4-3.0 m up.
- From 2.1 m (8930) to the last airborne frame (8988, 1.2 m), the nearest pedestrians were about 2-3 m to the side (estimate), and none was under the drone.
- In FPV, a woman in a dark high-neck top and slacks is visible from about frame 8890-8915 and passes close on the right at about 8945-8962 (1.5-1.2 m). Her look matches the doc's BP_CityPed_F3 class (`docs/FINDING-crowd-pedestrians-and-traffic.md` line 94).
- A second pedestrian, in a grey blazer and wide trousers, is at the right edge at about 8968-8990.
- The F3 class covers more than one instance, so linking BP_CityPed_F3_C_4 to the visible woman is appearance-only.

**onepath_05 (measured from frames, with estimates)**
- Same site as onepath_02 (final 192.59,30.23 against 192.63,30.21). onepath_02 had 1 contact (SM_jcBldBase60b_97 only).
- Two pedestrians walked along the pavement to the corner and passed the descending drone's position at frames 8619-8650 (HUD 433.7-435.3 s, 2.6-1.7 m), an estimated 0.5-1 m in front of its ground point. No contact is visible.
- The HUD is frozen at 435.8 s / 1.4 m.
- A third pedestrian was walking toward the drone, about 9 m away, at the end of the recording.

**Base rates across runs (measured; scan of 318 demo/out runs with a collisions block)**
- 94 runs have only a road tile after the mission; 93 of them have exactly 1 contact. The exception is agx_e2e_r4_20261103: 52 contacts on a road tile only, landing in place (path 0).
- SM_jcBldBase60b_97 alone with 1 contact: 97 runs.
- So a road-only touchdown can produce dozens of contacts, and the 132 cannot be split between pedestrian and road.
- All 7 runs with a pedestrian after the mission have 6-202 contacts.

**History (measured)**
- On 19 Nov, pedestrian landing contacts came in 2 of 7 flights, both at (192,30), a site used by 3 flights. The 4 flights at (194,-28) had 1 SM_jcBldBase60b_97 contact each.
- Earlier runs with pedestrian after-mission contacts (totals include road contacts):
  - aiot_v8r_01, 30 Oct: 72
  - aiot_tch_w06, 31 Oct: 202
  - aiot_tch_k04, 1 Nov: 20
  - aiot_v10kw_02, 1 Nov: 80
  - agx_e2e_aerovla_01, 3 Nov: 67
- Two of those (aiot_v8r_01, aiot_v10kw_02) were at (194,-28).
- pre_t0 = 1 on 302 of 318 runs. The other 16 (cspab/cspsmoke) come from `demo/real_vla_demo.py` and have no pre_t0 field. The pre_t0 object is never saved.

**KPI treatment (measured)**
- `guardrail/kpi.py` collision_contacts deliberately counts only mission contacts.
- kpi.json shows collision_contacts 0, outcome success and mission_success true for nfz3b_wgs84 and onepath_05.
- kpi.json has no after-mission field.

**Code change history (measured)**
- `demo/landing.py` was last changed in 470b9a2 (6 Oct). The landing maps are dated 4 Nov. `tools/citylife_routes.py` has no diff in the range.
- The descent block is byte-identical between 4d0838b and bfb4973.
- In `_fly_to_landing_site` only the command frame changed (WP3-04).
- `guardrail/shield.py` itself changed heavily in that range, so transit behaviour is not guaranteed identical.

### Is the one-path change a regression here?
**No.**
- The site choice and the descent are unchanged and run outside the Shield.
- Pedestrian landing contacts already occurred in 5 runs between 30 Oct and 3 Nov.
- Mission contacts are 0 on all 7 runs of 19 Nov.

### CHANGELOG sentences
- "All 7 AGX CityLife flights on 19 Nov (bfb4973) had 0 simulator contacts in the mission window; after the mission, 5 recorded a single touchdown contact and 2 (agx_onepath_shield_05: 6; nfz3b_agx_hier_aerovla_wgs84_20261119: 132) list a level pedestrian, at landing site (192,30), where pedestrians walked within a few metres of the descending aircraft."
- "The landing phase is outside the guardrail's pedestrian awareness: landing.choose_site prefers pavement and uses only static maps and car lanes, the vertical descent and land_async bypass the Shield, and per-contact times after the mission are not logged; making landing pedestrian-aware is an open safety item, not a regression of the 18 Nov change (the same kind of contact appears in five runs from 30 Oct to 3 Nov)."

---

## What stays undetermined
1. **Slow inference in the onepath window.** Why OWL-ViT inference was slower during the five onepath runs (about 17:33-18:30, 20-52 inferences over 700 ms each) but not in the hier run at 18:50. No 19 Nov jetson_state was recorded.
2. **Lock settings on the Jetson.** The `--lock-gate`, `--linear-bearing` and camera HFOV values used on the Jetson are assumed to be the defaults (0.12, off, 90°); none is logged. The estimator prior that the stale lock used at seq889+ is not logged either.
3. **Closed-loop effects of the guard change.** In both runs 03 and nfz3b, closed-loop effects of the guard change cannot be excluded: the replays are open-loop, and nfz3b has 41 earlier guard-decision differences.
4. **Why 19 Nov nfz3b stalled when 4 Nov did not.** The stand-off code is the same. 19 Nov had 403 fence-hold ticks against 49 on 4 Nov, at a different code revision with VLA-driven path differences.
5. **Untested counterfactuals.**
   - Whether re-centring the run-03 search planner on far sightings would have recovered the car sooner.
   - How much an un-held nfz3b drone would have scored.
6. **Whether fallback flying lengthened the nfz3b loss.** The logs show the order of events, not this.
7. **Landing-contact timing and attribution.**
   - For all after-mission contacts: which phase (transit, descent, touchdown, on the ground), which object each contact was, and how the 132 or 6 split between pedestrian and ground.
   - Whether the visible woman is BP_CityPed_F3_C_4.
   - Whether any pedestrian touched the drone during the unrecorded touchdown gap or after the recording ended.
8. **The pre_t0 contact object** on every run.
9. **nfz-south-block** was never exercised in any run.
10. **Standoff ground truth.** Whether the standoff-any ground-truth breaches (102 ticks, horizontal at about 8 m altitude) indicate a rule-geometry gap or only the rule's horizontal-only definition. That is a policy question, not settled by these logs.
