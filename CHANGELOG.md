# Changelog

All notable changes to the Guardrail safety layer.

Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/);
versioning is [Semantic Versioning](https://semver.org/spec/v2.0.0.html), with
the caveat that this is pre-1.0 research software and the public surface is
still moving.

A note peculiar to this project: several entries below are **retractions**. The
grant names five acceptance KPIs, *P0 violation escape rate* (target 0) among them, and a safety number that is
wrong is worse than one that is missing, so a corrected claim is treated as a
shipped change and recorded as one.

---

## [Unreleased]

### 2026-10-24 (later) — the VLA's decision rate, not its training, is what keeps it behind: ceiling flights, the renderer's share of the GPU, faster VLA flights

The fine-tuned follows kept losing the car, so the question was asked
directly: how well does the hand-written follow itself do when it must act
at the VLA's rate? Badly - and the slowness was mostly the simulator's
renderer sharing the GPU. Still not as good as the hand-written follow; the
third model is training.

#### Found
- **Ceiling flights** (`follow_vlm --decide-every-s P --decide-delay-s D`,
  merge a92f338: the expert's own command, D s old, held for P s): within 30 m
  on 0.122 of the ticks at P 1.0 / D 0.8 (the VLA's rate at the time; it
  overshot onto the car, minimum 0.5 m, and saw it on 16 % of the ticks),
  0.908 at 0.5 / 0.4, 0.956 at 0.3 / 0.3, against 0.963 at full rate. The VLA
  flights (0.158-0.238 from the first acquisition) were no worse than the
  expert held to their rate.
- **The renderer, not the model, made the VLA slow.** Alone the merged bf16
  server answers in 235 ms (generation 0.233 s, 9 tokens). Pinged through an
  expert flight it took 625-713 ms from the moment the simulator moved the
  camera; with the aircraft hovering in four simulator configurations it
  stayed at 230-242 ms. OWL-ViT is not the cause: alone it takes 30.5 ms
  (fp32), and the server stays at 248 ms beside it.
  `-SimOffscreen` (`-RenderOffscreen`) with `-SimMaxFps 30` (`t.MaxFPS`)
  brought the chase-phase latency to 370-397 ms with the expert's own score
  0.945 (commit 0a352b5; both switches default off).

#### Flown (from the first acquisition, within 30 m)
- **follow_v3_pf_r1/epoch1** (r2, the prev_front tile; its training stopped at
  epoch 2 step 1320 when the session ended - held-out epoch 1: forward step
  error 0.340 m, turn error 0.0173 rad): `vla_follow_ft_03_pf` 0.173 (it
  overshot a stopped car, which left the bottom of the frame, then flew on
  with no car in view), `vla_follow_ft_04_pf_fps30` 0.148,
  `vla_follow_ft_05_pf_off` 0.125 (capture-to-answer p50 0.43 s, decisions
  every 0.4-0.5 s: it answered forward 0.0 for about 7 s while the car pulled
  away from 8 m).
- **follow_v2_r1/epoch4** (r1) offscreen: `vla_follow_ft_06_r1_off` 0.169
  (latency p50 0.46 s); it chased for about 190 s with three re-acquisitions
  but mostly 30-40 m behind.

#### Added
- **Memory and search samples** (merge 564901c): one hint rule,
  `vla_mission.follow_hint_step`, for the flight's DetectorHint and
  `build_follow_dataset.py --hint-modes fresh,memory,search`, plus the
  flight's close rule (`--close-bottom-frac 1.0`). They add few samples: the
  expert rarely loses the car, and no capture shows the car behind the
  aircraft.
- **Five DAgger flights with r2** (ft_dagger_pf_1-5): within 30 m on 0.844,
  0.807, 0.808, 0.788 and 0.813; 52, 54, 53, 52 and 53 windows.
- **Dataset follow_v4_pf_hm** (19 recordings): 18570 train (9285 real) and
  1641 held-out samples (ft_expert_p03, ft_dagger_03, ft_dagger_pf_3); 309
  memory and 22 search samples.
- **Speed-change oversampling** (`--change-oversample N`, 0a352b5): 2370 of
  the 18570 train samples change the forward step by 1 m or more within 1 s.
  Training follow_v4_r3 (from r2, 3 epochs, changes x3, turns x2) is in
  progress.
- Videos of every flight above in `docs/video/` (`*_small.mp4`).

### 2026-10-24 — AerialVLA fine-tuned on the hand-written follow: held-out step error 2.47 m to 0.36 m, the first fine-tuned flights still lose the car, DAgger and a temporal tile next

The user's goal: the VLA-piloted follow as good as, or better than, the
hand-written follow. AerialVLA was fine-tuned on the expert's own flights
(the 23 Oct FT-D / FT-T / DART tools, merged 5269143 and ce1a61c), flown,
and the two failures it showed were turned into the next round's data and
inputs. Not yet as good as the hand-written follow; the second model is
training.

#### Added
- **Expert recordings for the fine-tune** (`follow_vlm --record-vla`, with
  `--record-perturb` DART on the p runs): ft_expert_01, 02b, 03 and p01-p06,
  within 30 m on 0.953, 0.948, 0.939, 0.972, 0.934, 0.969, 0.928, 0.985 and
  0.95 of their ticks. p06 kept 339 of its frames: C: filled during the
  batch (5096 write errors; frames now go to D:, commit 40e079e).
- **Dataset follow_v2**: 5706 train samples (2853 real, 2853 mirrored) and
  345 held-out (ft_expert_p03), labels the expert's motion over 1.0 s.
- **Fine-tune follow_v2_r1** (bf16 LoRA from upstream's aero_vla, 4 epochs,
  116.5 min). Held-out (345 samples, never trained on), original adapter
  then epochs 1-4: forward step error 2.467, 0.432, 0.377, 0.396, 0.359 m;
  turn error 0.103, 0.056, 0.053, 0.050, 0.053 rad; turn sign agreeing
  0.699, 0.791, 0.817, 0.826, 0.835. Epoch 4 merged in bf16 (the flying
  configuration): 0.360 m, 0.0526 rad, 0.838 - merging costs nothing here.
- **DAgger recording** (merge c9dc0a1): `follow_vlm --record-vla
  --record-perturb-vla HOST:PORT` (`-RecordPerturbVla`) lets the fine-tuned
  AerialVLA fly seeded 2.5-4.0 s windows every 6-10 s of the expert's
  recording, through vla_fly's own follow tracker, after the controller and
  before the Shield; the expert's recovery is the label (the DART rule). A
  window with no fresh step is flown by the expert and says so.
  `StepTracker.note_flown` starts the tracker's smoothing from the command
  in force, so the hand-over does not jump.
- **Five DAgger flights** with follow_v2_r1/epoch4 (ft_dagger_01-05): 52,
  53, 52, 50 and 51 windows; within 30 m on 0.803, 0.879, 0.799, 0.811 and
  0.846 of their ticks; capture-to-answer p50 0.78-0.83 s. Against
  follow_v2, ft_dagger_01's labels sit farther from the car (median 23.9 m
  against 20.4 m) and twice as often at full speed (fwd >= 4.5 m on 0.15
  against 0.08): the states the VLA creates and the expert's catch-up.
- **The front frame 1 s earlier as the lower tile** (merge ee4762d, off by
  default): `--bottom-tile prev_front` (builder) and `--vla-bottom-tile
  prev_front` (flight) put the front frame nearest t - 1.0 s (within
  0.25 s) where the down camera was; no such frame is no sample and no
  request, never a fallback. Checkpoints record the tile and label horizon
  they learned; the server announces both and vla_fly / the DAgger recorder
  refuse to fly a mismatch.
- **Rate execution** (`vla_fly --step-as-rate H`, off by default): a step
  flies at fwd / H and dpsi / H until the next answer replaces it - what a
  1.0 s label means - instead of flying to an endpoint at cruise and braking.
- **Dataset follow_v3_pf**: the 9 expert and 5 DAgger recordings with the
  prev_front tile, stride 0.25 s: 13762 train samples (6881 real, 6881
  mirrored) and 1076 held-out (ft_expert_p03, ft_dagger_03); measured tile
  dt p50 1.0082 s. Training follow_v3_pf_r1 on it is in progress.
- A follow's score from its first acquisition (`metrics.follow.after_acquire`,
  top-level `frac_within_30m_after_acquire`; commit 3cd51d3), see Found.

#### Flown
- **vla_follow_ft_01_20261024** (follow_v2_r1/epoch4, merged bf16, 8 m - the
  expert's altitude; the dataset's median is 8.06 m and vla_fly's default
  10 m): within 30 m on 0.121 of the flight, 0.158 from the first acquisition
  (229.3 s). The car stopped at a light about 9 m ahead and the model held,
  as the expert does; the car drove off and the model answered forward 0.0
  for about 6 s while the car went from 9.8 m to 27.6 m - one frame cannot
  tell a stopped car from one pulling away. It then matched the car's speed
  about 30 m behind and lost it.
- **vla_follow_ft_02_rate_20261024** (as ft_01 with `--step-as-rate 1.0` and
  `--acquire-max-range-m 60`): 0.196 of the flight, 0.238 from the first
  acquisition. It closed from 33.0 m to 5.5 m behind a stopping car, lagged
  again when the car pulled away, and lost the car at a corner where it had
  turned toward the car and the Shield's 3 m building clearance held it at
  0.5 m/s. P0 escape rate 0.0 in both.

#### Found
- **The VLA-to-hand-written comparison was on unequal bases.** follow_vlm
  starts its clock and score at its start gate: the reference
  citylife_redcar_nfz_motion2_20261019 (0.963) waited 196.1 s and scored
  the 2400 rows after it. vla_fly scored from take-off, and in every flight
  the level car spends about 200 s on the far side of its loop first. From
  the first acquisition: ft_01 0.158, ft_02_rate 0.238, follow #10 0.235
  (0.17 of its whole flight). Still far from 0.963.

#### Fixed
- The trainer refused the builder's whole first dataset (6051 of 6051 lines
  "no_target": the builder writes the answer as action_text); a test now
  feeds the builder's rows to the trainer's loader (f8476a3, 1762e9f).
- tests/test_track_truth.py knows a tie at the frame centre: the clean
  expert recordings tie the constant-centre null (3.4 px against 3.4, 3.0
  against 3.0, 3.3 against 3.3) while their perturbed twins win, 27.0-30.5 px
  against 37.6-39.3 (896b50d); the tie may be within 5 % + 0.1 px of the
  null (ft_dagger_01: 5.5 px against 5.3, its four siblings win).

#### Tests
- `tests/test_vla_fly.py` 62/62, `tests/test_vla_dagger.py` 17/17,
  `tests/test_vla_prev_front_rate.py` 21/24 (3 need Pillow / torch),
  `tests/test_follow_dataset.py` 35/39 (39/39 with Pillow),
  `tests/test_track_truth.py` 36/36.
- The CI list, 120 files: 2 lab-only files fail, both stale decks
  (`tests/test_explainer_deck.py`: the 14 Oct pack cites a shield.py
  sentence rewritten on 21 Oct; `tests/test_build_progress_oct_data.py`:
  the ITRI pack was not rebuilt after its data changed on 22 Oct). Left for
  a separate change: the PI chooses which deck is sent.

### 2026-10-23 (follow, reviewed) — the follow's score now counts, a lost car is re-acquired at once, follow #10

An independent review of the follow work (no P0; truth never reaches control)
found the follow's records overstating, and a 26 s blackout after a lapse.
Fixed (merge d9dbbd8), then follow #10 flown.

#### Retracted
- **Every VLA follow's kpi.json said mission_success true**: the follow score
  sat under `metrics.follow`, where guardrail/kpi.py does not look, so no follow
  test applied. Re-scored from each run's own flight log with
  `tools/rescore_vla_follow.py`: #2 and #3 (0.0 within 30 m) now fail
  `follow_proxy`; the other seven pass kpi's 5 % rule. The within-30-m shares
  quoted in the section below were computed from the flight logs and stand.

#### Fixed
- **A lost car is re-acquired at once**: with identity on, the strict lock
  lapsed after 2 s and nothing re-acquired for memory + search (26 s); now the
  lapse, or 3 s unseen, restarts follow_vlm's Reacquirer anchored at the last
  sighting (in the test with the real lock and Reacquirer: back 0.6 s after the
  car reappears, never within 20 s before).
- The follow's records: each decision's hint kind (fresh / memory / search);
  a LAND held as arrival is no longer a VLA landing; a follow that never asked
  ends `subject_not_acquired`, not as a VLA failure; the start gate is the one
  applied after the identity setup; perception resets drop the stale box.

#### Flown
- **#10** (`demo/out/vla_follow_redcar_10_20261023`, as #9 with the fixes):
  within 30 m on 0.926 of the ticks AerialVLA flew (median 12.78 m, the box on
  the car 133 of 151), the best chase yet - but 0.17 of the whole flight: the
  car started 41.77 m ahead, the drone waited for it to come round, chased it,
  and lost it when the car outran it before the corner; the re-acquisition
  range (30 m) never took it back.

#### Tests
- `tests/test_vla_fly.py` 60/61, the CI list 115 files, 0 failing.

### 2026-10-23 (follow) — AerialVLA chases the red car: nine follow flights, each fix measured on the next

The user asked for AerialVLA to follow the target as the hand-written
controller does. AerialVLA still decides every step; what changed is which box
is the car (perception) and how a step is executed, all for `--mission follow`
only (commit d6e7123). CityLife simple_flight, Car_10 as truth for scoring
only, not KPI-grade. Within 30 m, whole flight / while the VLA flew:

| # | what changed | within 30 m | while flying | closest | box on the car |
|---|---|---|---|---|---|
| 1 | first follow (3 m/s goto policy) | 0.348 | 0.506 | 0.49 m | 586 of 814 |
| 2 | fresh scene (the car started 35 m ahead) | 0.0 | 0.0 | 35.29 m | 374 of 1412 |
| 3 | the city follows' 5 m/s policy, a lock, memory and search | 0.0 | 0.0 | 41.76 m | 517 of 2384 |
| 4 | a start gate, steps anchored where the answer arrives | 0.252 | 0.195 | 0.93 m | 968 of 3834 |
| 5 | a reset when the Shield pins the aircraft | 0.315 | 1.0 | 0.08 m | 174 of 194 |
| 6 | the model's LAND held as arrival | 0.216 | 0.254 | 0.12 m | 182 of 1061 |
| 7 | identity check on (its lock never seeded: no request) | 0.139 | - | 2.74 m | none |
| 8 | identity acquisition (follow_vlm's Reacquirer) | 0.199 | 0.646 | 0.1 m | 344 of 451 |
| 9 | as #8, bf16 merged model (latency p50 0.7914 s) | 0.308 | 0.552 | 0.07 m | 473 of 763 |

The hand-written controller's 19 Oct flight: 0.963.

#### Flown
- **#9, the best**: two chases - up the street and, for the first time,
  round the right-hand corner and on along the next street; within 30 m on
  0.552 of the ticks AerialVLA flew, speed while chasing p90 3.46 m/s. Between
  the chases the drone hovered until the car came round its loop again, which
  is most of the whole-flight shortfall.
- **#5**: the closest chase - within 30 m on every tick the VLA flew, down to
  0.08 m above the car, until AerialVLA answered LAND (its training: target
  found, land), which ended that flight; #6 onwards hold instead.
- Every follow: P0 escape 0.0 and 0.0 s in the no-fly zone; #4 was pinned 225 s
  against a building by a wrong box (2510 Shield interventions).

#### Added
- Perception for the follow: follow_vlm's TargetLock and identity check (the
  VLA detector camera gains follow_vlm's depth stream), follow_vlm's
  Reacquirer as the start gate and the re-acquisition after a lapse; the box
  matched the car on 0.17-0.26 of the ticks with a box without them (#2, #3,
  #4, #6) and 0.62-0.76 with them (#8, #9).
- The follow's hint: the last sighting's bearing for 6 s after the box is
  lost, then a quarter-turn phrase toward its side for 20 s; a close box asks
  for no new step and holds the horizontal motion; the Shield pinning the
  aircraft resets the lock.
- The follow's executor (TrackerConfig, all default off): turn while moving,
  the altitude held (AerialVLA's 2-4 m descents), steps anchored where the
  answer arrives (#3 flew 1.54 m/s at a 4.5 m/s cruise), LAND as arrival.

#### Found
- The remaining limit is AerialVLA itself: steps of at most 5 m at about one
  answer per 0.8-1.3 s leave it below the car's 3.4-4 m/s on a straight, and
  it was never trained to pursue; when the car leaves the view at a corner,
  nothing but the car's own loop brings it back.

#### Tests
- `tests/test_vla_fly.py` 45/45, `tests/test_vla_link.py` 33/33; the CI list
  115 files, 0 failing.

#### Not done
- Training AerialVLA on following (expert demonstrations from the
  hand-written controller with step labels, matched to the model's decision
  latency, as the July campaign learned) is the next step and has not
  started. The merged model (#9) agrees with the upstream-precision reference
  on 88 % of the probe, not 96 %.

### 2026-10-23 (flown) — AerialVLA flies the drone: a goal reached and landed by the model, and the red car chased then lost

The first two flights piloted by AerialVLA (OpenVLA-7B + the original
adapter, bf16, on the RTX 4090) behind the Shield, on the CityLife
simple_flight rail with the signed `vla-goto-citylife` bundle. Not KPI-grade
(the simple_flight rail is none of the grant's topologies). The memory that
blocked them was freed by the user.

#### Flown
- **M1 GOTO** (`demo/out/vla_goto_01_20261023`, stop-and-go cadence): from
  the mission start (42, -18) AerialVLA flew 58 m up the street on the
  goal-derived phrase and answered LAND itself, with the empty phrase ("you
  have arrived"), 0.85 m from the goal (43, 40); closest approach 0.23 m,
  reached_goal true. 14 decisions, capture-to-answer latency p50 1.0125 s;
  0.9895 of the mission ticks authored by the VLA; 0 Shield interventions; 0
  collisions in the mission. The Dashboard now counts 1 VLA flight of 100.
- **M3 follow, the red car** (`demo/out/vla_follow_redcar_01_20261023`,
  240 s, pipelined cadence at 2.5 m/s, labelled): OWL-ViT's box gives only
  the direction phrase, AerialVLA flies, the level's Car_10 (the earlier city
  follows' red car) is truth for scoring only. The drone waited about 45 s for
  the car to come round its loop, then chased it 111 m up the street and came
  within 0.66 m (horizontally, flying over it) at 70 s; the car then pulled
  away at about 3 m/s against the drone's 1.8 m/s and turned off the street,
  the box was lost and the mission ended target_lost. Within 30 m of the car
  for 0.348 of the flight (the hand-written controller's 19 Oct flight:
  0.963); median distance 58 m. The box pointed within 10 deg of the car's
  bearing on 0.72 of the ticks with a box (150 of 151 at 40-60 s, 200 of 200
  at 80-100 s, 99 of 251 after the turn); at times it sat on something in line
  with the car, such as a road arrow. Shield: P0 escape 0.0, 0.0 s in the
  no-fly zone, 177 corrected ticks (altitude band 111 - AerialVLA kept asking
  for 2-4 m descents - no-fly-zone frontage 64, building clearance 26).

#### Added
- **`demo/vla_fly.py --mission follow`**: M2's detector phrase without M2's
  stop on a large box; ends on the time limit, a lost subject (90 s default)
  or a VLA failure; `--level-car` (default Car_10) scored as `subject_dist_m`
  per row and `metrics.follow` (the same within-30-m share as follow_vlm).
  OWL-ViT now loads beside the warm-up and the take-off.

#### Tests
- The CI list runs 115 test files, 0 failing.

#### Not done
- Following a moving car is far weaker than the hand-written controller:
  the policy's 3 m/s speed cap is at the car's speed, a rotation step stops
  translation, and nothing re-acquires the car after a turn. No Shield-off
  control arm was flown, so these flights show the Shield acting, not its
  margin over none.

### 2026-10-23 (later) — AerialVLA in the pilot slot of the ArduPilot rail and of the ROS 2 rail; none flown

Wave V2, branches b and c, each reviewed, fixed and re-reviewed (the ROS 2
branch twice), merged 37292d1 and d0eb3be. Nothing here has run in Unreal,
SITL, ROS or with the model; the ROS 2 rail has no camera publisher yet.

#### Added
- **`perception_node.py --pilot vla`** (Project AirSim + ArduPilot rail): the
  step tracker's action goes through the same Shield and escalation FSM as the
  detector pilot; FSM holds and stale-link spans suspend the tracker and pause
  the mission's stop rules; the model's LAND is landed by the node's own
  mission-end LAND. GOTO (`--goal`, scored on reaching it) and APPROACH (the
  OWL-ViT box's phrase, scored on the subject's true range). The warm-up runs
  disarmed; a refusal is recorded and exits 4. Stop-and-go only, unless
  pipelined is explicitly allowed (this rail's captures are timed by arrival).
  The default `--pilot servo` is unchanged.
- The rail's robot config carries AerialVLA's VLAFront / VLADown cameras; they
  render only in a VLA run (or with `-VlaCameras`), and the run records which.
- **`sitl/ros2_vla_bridge_node.py`**: AerialVLA in the ROS 2 / MAVROS 2 rail's
  VLA slot (`VLA=bridge` / `vla:=bridge`), publishing `/vla/action_4d` at 10 Hz
  with the decision's age, seq, author and LAND/END flags; the stub stays the
  default and its 4-value stream is read as before.

#### Changed
- **The ROS 2 Shield node refuses any VLA message that breaks the contract and
  holds the aircraft**: a NaN anywhere, a partial tail, a missing or wrong
  layout label, a seq going backwards, motion no decision authored, or 4 values
  after the bridge's 8. A sound bridge's stall is left to the receipt watchdog
  (LOITER, then GUIDED when messages return); a broken pilot ends the mission
  through the mission-end LAND after one 10 s hold or 3 holds in 30 s, never
  over the FSM's own Loiter or fail-safe. A lasting second publisher on
  `/vla/action_4d` makes the run not KPI-grade.
- `--vla-ready-timeout-s` (3300 s, added by the launch file for the bridge)
  arms only after the bridge's warm-up passes and its layout and decision
  timeout match the Shield's; arming then gets its full budget after the wait.

#### Fixed
- **The warm-up gate on both rails, the merged city rail included**: each
  request now carries the answer timeout it is recorded with (the cold first
  one gets 60 s, where the socket used to cut it at 10 s) and waits for a ready
  client after a late answer or a backoff. Before, a cold first inference over
  10 s, or one refused first request, refused a healthy server.

#### Tests
- The CI list runs 115 test files on the merged tree, 0 failing.

#### Not done
- No flight on either rail. The ROS 2 rail needs a camera publisher; the Orin
  entrypoint and compose do not yet select the bridge or serve AerialVLA.
  Small record gaps from the last reviews remain (an approach refusal's
  start-gate record, a reused scene's camera load, the Dashboard row reading
  quiet during a long warm-up).

### 2026-10-23 — the CityLife VLA flight is ready behind the Shield, and AerialVLA re-measured at its own precision; not yet flown

Wave V2, branch a: built, reviewed twice, fixed twice, merged (988c447). The
first AerialVLA-piloted flight has **not** flown: with the bf16 server loaded
the machine was at 99.8 of 104 GB committed memory, too little to start the
simulator safely, so the server was stopped and the flight waits for memory.

#### Added
- **`demo/vla_fly.py`: AerialVLA flies CityLife behind the Shield.** M1 GOTO
  (the direction phrase from the goal's bearing at each capture; the empty
  phrase within 3 m) and M2 APPROACH (the phrase from OWL-ViT's box through
  `vla_bridge`, rotated by the turn since the detector's frame). Every action
  goes through `Shield.filter`; `--shield off` is a labelled control arm
  clamped only to a recorded 2-40 m simulator band. LAND ends the mission in
  the script's own landing (never a mode request from the model).
- **Upstream's stop-and-go cadence is the default** (`-Cadence pipelined` is a
  labelled arm). On the point-vehicle probe, pipelining a constant answer for
  30 s climbed +1.77 m against +0.51 m and drifted -54.6 deg against -12.9 deg.
- **A warm-up gate before arming**: three inferences on live camera pairs, the
  last judged against 5 s; a slow or silent model refuses the flight (exit 4).
  A slow model is told apart from a dead one (`server_slow` / `server_dead`),
  and no request is stacked behind a timed-out one.
- **Every way out lands and is recorded**: an exception, a missed start, an
  overshoot or Ctrl+C (exit 130, marked interrupted); a server the flight
  started is always stopped; a records fault still leaves `metrics.json`.
- **`tools/vla_replay_gate.py`**: the flight's own model on at least 10 recorded
  frames, all seven phrases, descents judged; exit 3 when the server is not
  the flight's model, 4 for a stand-in (never PASS), 2 when it could not run.
- `policies/vla_goto_citylife.yaml` (6-14 m band, 3 m clearance, the city
  no-fly zone) and its signed bundle; VLA cameras at 256x256, 90 deg, pitch 0
  and -90, as upstream; `scripts/run_vla_fly.ps1`.

#### Measured (offline, RTX 4090)
- **AerialVLA's direction phrase in bf16** (`docs/data/aerialvla_bf16_phrase_probe_20261023.json`,
  section "Addendum, 2026-10-23" of `docs/FINDING-what-drives-aerialvla.md`):
  the mean turn is ordered by the phrase (-0.651 to +0.288 rad), but facing
  the target every phrase gives about 4.9 m forward and no turn; the phrase
  steers only when the target is off to one side. The 4 Aug effect was
  measured on a 4-bit loader. 36 requests, every answer recorded.
- Idle latency p50 391 ms per decision; about 15.8 GB of VRAM. A bf16 cold
  load took 26 min with the machine short of memory.

#### Fixed
- `tools/check_claims.py` registers the two 22 Oct retractions; the 22 Oct log
  had failed `test_every_changelog_retraction_is_registered`.
- A `fake:` model id never counts as a VLA, whatever digest it reports.

#### Tests
- The CI list runs 113 test files, 0 failing. `tests/test_vla_fly.py` 35/35,
  `tests/test_vla_replay_gate.py` 8/8; `tests/test_vla_fly_fakesim.py` 10/10 in
  the lab environment (it skips without OpenCV, as in CI).

#### Not done
- No VLA flight yet (blocked by memory, above). Open from the second review:
  the abort landing caps `land_async` at 60 s with no descent first (the
  control arm can be up to 40 m high); an aborted run's decision rate includes
  the time after landing; the replay gate can wait a full timeout per request
  on a slow server and checks its frame count only after the server is up.

### 2026-10-22 — AerialVLA becomes a pilot the Guardrail can fly, the tracker becomes the progress monitor, and open WP items closed

Wave V1: five branches, each reviewed adversarially two or three times and
fixed after each review, merged on 21-22 Oct (2aa03b1 for V1a-V1c, 01f7820
for V1d, fb8f87c for V1e). **Nothing in this wave has flown
a VLA yet**: every AerialVLA number below is offline (the model on the RTX
4090 against recorded frames), and the rail changes are tested headless, not
flown. The goal it serves is the user's of 21 Oct: a VLA that flies the
drone, not a hand-written controller with a detector.

#### Added
- **AerialVLA codec** (`guardrail/aerialvla_codec.py`): the upstream prompt
  byte for byte, the 8-phrase vocabulary, the 224x448 front-over-down mosaic
  (a missing down frame is recorded or refused) and a strict output reader.
  Anything but `FF DD YY[ LAND]</s>` is a parse failure the client holds as
  zero; the old last-three-integers reader had turned `00 49 494 49` into a
  5 m dive. Outputs are per-decision STEPS (forward 0-5 m, down -5..5 m,
  heading change +-1.1 rad), not velocities.
- **VLA server protocol v2** (`demo/vla_server_v2.py`, also
  `demo/vla_server.py --v2`): TCP on 127.0.0.1, newline JSON; refuses unknown
  fields and any field, object word or CSP text that could carry target
  coordinates; answers every request it reads (a failing log write, a
  decompression-bomb frame, a 400-digit number get `ok=false`, not a dropped
  connection); puts the model identity (weights_sha256 over the base shards
  plus the adapter, precision, attention, merge, measured agreement) on every
  answer. **No silent model default**: it refuses to load until
  `--precision`, `--attn` and `--merge`/`--no-merge` are given.
- **AerialVLA serving on the RTX 4090** (`docs/data/aerialvla_serving_4090.json`):
  all 8 precision x attention x merge configurations meet the 1.0 s p50 gate.
  bf16/sdpa/unmerged reproduces the upstream-precision reference on 96 % of
  the probe (p50 486 ms, ~16 GB); bf16/sdpa/merged is fastest (259 ms, 88 %);
  nf4/sdpa/merged is the low-VRAM option (517 ms, ~7.9 GB process VRAM) but
  agrees on only 25-33 %. CityLife loaded leaves ~19.4 GB free, so bf16 is
  expected to fit beside it (not yet measured in flight).
- **The pilot client** (`guardrail/vla_link.py`, `RemoteVLABackend` in
  `guardrail/vla_backends.py`): one outstanding request, pipelined captures, a
  `StepTracker` that turns each step and its capture pose into a 10 Hz
  world-frame action (rotate only when |dpsi| >= 0.25 rad, as upstream; cruise
  cap; altitude inside the tighter of the configured and policy bands), LAND
  reported to the rail (never a raw mode request), hover on a stale or failed
  decision. Every tick names its author (vla_step / hold / hover / none). The
  client re-reads the answer text instead of trusting `parse_ok`, pins the
  model identity (a different model mid-run is refused, `model_changed`), and
  nothing in it can stop the 10 Hz loop (logging on its own thread, callback
  errors counted).
- **Dashboard** (default page of the local tracker, `tools/build_dashboard.py`,
  `GET /api/dashboard`, `GET /api/live`): VLA in the loop, running now (with
  heartbeats from `tools/live_status.py`, written outside OneDrive), WP
  progress, grant KPIs (target / value / null / grade / as_of / source; not
  measured never shows as 0; green only when on target, KPI-grade and better
  than its null), latest flights with H.264 videos, timeline and the open
  to-do list (`tools/todo.py`, `tracker/data/todo.json`, 44 items), in
  EN/ID/zh-Hant. **A run counts as a VLA flight only on the server's own
  answers in its records** (at least 10 accepted parsed decisions carrying one
  real weights digest, and 10 ticks flown toward them); today 0 of 99 recorded
  runs qualify. A red banner shows when a flight and a workflow overlap.
- **ArduPilot rail start gate** (`demo/pas_ardupilot/perception_node.py`):
  after take-off the aircraft hovers until the detector has seen a plausible
  subject 5 times in a row within 30 m; a timeout flies no mission unless
  `--start-timeout-action fly`. The hover is watched tick by tick
  (HoverWatch): a pilot or failsafe takeover ends it at once, the Shield
  checks every hover tick, and an unsafe hover tick starts the loop so the
  Shield and the escalation FSM act. New options `--identity`,
  `--negative-query`, `--lock-gate`. Replayed on the 21 Oct flight's 345
  inferences the gate never opens: no box there was the car. Not flown.
- **Sweep**: a sideways-pilot cell (`time-window-opens-on-sideways-pilot`,
  2 m/s east, bounded at 7 ticks inside, 2.9 s to safe, never censored), and
  per-episode `cap_refused_ticks` / `floor_refused_ticks` printed beside the
  P0 headline, zero included.

#### Changed
- **The city rail flies the signed bundle of its policy by default**
  (ARCH-29); the YAML only under `--dev-yaml`, recorded as a development run.
  Build the bundles first: `python tools/build_flight_bundles.py`.
- **C2, the time-window escape.** When a zone's window opens on the aircraft
  sooner than any action within the hard caps can take it out, the zone is
  judged by the inside rule from the opening, and the escape flies at the
  speed cap until the aircraft is out of the ring. That escape is now
  forecast as flown (out of the ring, then a stop at one forecast step and
  3 m/s^2), so a rule beyond the stop no longer refuses the cap; when one
  lies inside the stop, the Shield tries the fastest clean speed, then
  reports a violation so the FSM escalates. No FSM edge or KPI formula
  changed. The library P0 escape of 0 for these cells is a re-judgement of the
  same flight path: the aircraft is still inside the polygon 3-13 ticks, as on
  7c9c9e6, at most 2.8 s to safe.
- **C8**: the stand-off CSP sentence binds the followed subject ("the
  pedestrian you are following"), not "any pedestrian". **A14**: the
  paraphraser refuses narrowed or widened objects (held-out false accepts
  5/30 -> 0/30 on the dev set; 4/12 on a fresh held-out set). **WP1-22**:
  generated policies get unique ids.
- Evaluation RPCs on the ArduPilot rail run off the loop (the 21 Oct
  flight's 1028 ms tick was a 1024.9 ms simulator truth read), and the
  detector loads before SITL. Neither change has flown yet.

#### Fixed
- **LAND on every way out from the arming** (ArduPilot rail): an exception or
  Ctrl+C anywhere after arming, including the start-gate hover, lands the
  aircraft and writes the episode, unless the autopilot or a pilot has it; the
  net's own prints cannot raise past its LAND.
- A Ctrl+C caught by the start gate's hover ended the Dashboard row "ok"; it
  now ends "stopped" (found by the V1c/V1d merge, failing-first test).
- A run that flew no mission writes None (not a 1 MHz detector rate and zero
  dwell) for its rates and counters.
- The pilot client no longer treats a server error as a decision: a server
  refusing every request for 30 s read as 299 VLA holds at 9.97 Hz, now as
  300 `none` ticks and a decision rate of 0.

#### Retracted
- **The 4 Aug AerialVLA ablation**, read strictly: the original adapter stops
  on 17 of 18 no-hint frames, not 11, and the object-presence effect is not
  supported (`docs/data/aerialvla_ablation_rescore.json`). All its numbers are
  per-step displacements, not rates.
- **"The library has no such zone"** (V1e review notes): the corridor edge at
  x = 80 is 9 m past the school zone's ring at x = 71.

#### Measured (headless sweep, republished on the merged tree)
- Library, grant arm: 140 pass / 22 tracked / 0 fail / 18 known over 180
  episodes; P0 escape 0.0 over 179 Shield-on episodes (null passthrough
  0.234); fail-safe 145/171 = 0.848; mission success 0.603; mean time to safe
  4.31 s; 0 slowed escapes. The fail-safe KPI target (>= 0.99) is still not
  met.

#### Tests
- The CI list runs 110 test files on the merged tree, 0 failing.

#### Not done
- No VLA flight. The CityLife VLA pilot (`demo/vla_fly.py`) is the next
  branch; its first flight is the milestone.
- The 4 LLM-authored stored paraphrase sets stale since C8 are pinned, not
  re-authored. PI decisions, the Orin, the fail-safe gap and Gazebo remain
  open (`docs/CHECKLIST-remaining-work.md`).

### 2026-10-21 — the Shield forecasts from the aircraft's own velocity, legal hold points, a realistic harness vehicle, and a smoother city follow, flown

The to-do list's T1 wave, four branches each reviewed adversarially and again
for the reviewers' notes (merged 0931822, e20544a, 1a9791b, 08cd0be, and the
integration review's fixes in d1e93a9; 9bf5a64..d1e93a9, 25 commits), and
four flights. None of the flights is KPI-grade, and no VLA flies them (a
hand-written controller steers by OWL-ViT's boxes); every harness number
below is headless.

#### Flown
- **The city follow with its new motion, twice** (simple_flight rail, the
  CityLife red car with the no-fly zone, the unsigned YAML policy and the
  3 s / 0.5 s forecast, as on 13 Oct; 240 s each, videos 268-269 s):
  `demo/out/citylife_redcar_nfz_motion_20261019` at `--accel-max 2.0`
  (ece38c1), then `citylife_redcar_nfz_motion2_20261019` after the defaults
  were raised (c77c77d, below). Against `citylife_redcar_nfz_20261013`:

  | | 13 Oct (old motion) | 19 Oct, 2.0 m/s^2 | 19 Oct, 2.5 m/s^2 |
  |---|---|---|---|
  | tilt over 15 deg | 9 episodes | **0** | **0** |
  | max tilt | 19.1 deg | 11.8 deg | 12.8 deg |
  | max pitch rate | 181 deg/s | 90 deg/s | **20.2 deg/s** |
  | max roll rate | 47 deg/s | 97.8 deg/s (2 ticks over 47; p95 8.3) | **15.3 deg/s** |
  | commanded acceleration, max | 11.31 m/s^2 | 8.00 m/s^2 | **5.56 m/s^2** |
  | within 30 m of the car | 0.991 | 0.94 | 0.963 |
  | minimum separation | 12.0 m | 11.0 m | 10.8 m |
  | in the zone / P0 escape / interventions | 0.0 s / 0.0 / 1 | 0.0 s / 0.0 / 0 | 0.0 s / 0.0 / 0 |

  The first flight's shortfall was catching up: 142 of 2377 ticks beyond
  30 m (at most 33.7 m) in four spells, one after the start and three after
  FenceGuard sidesteps. With the raised defaults: 89 of 2400 (at most
  32.0 m), four spells, each after a sidestep, none at the start.
- **The ArduPilot close approach with the own-velocity forecast on and off**,
  build 08cd0be, the 14 Oct arguments (want width 0.5, seed 20261006, the
  signed `follow-car-citylife` bundle, pymavlink; Project AirSim + ArduPilot
  SITL, none of the grant's topologies, not KPI-grade):

  | | on (`pas_ap_ovf_on_20261021`) | off (`pas_ap_ovf_off_20261021`) | 14 Oct (`pas_ap_video_on_20261014`) |
  |---|---|---|---|
  | ticks inside a P0 margin (building clearance) | **0** | 79 (BRAKE 21, RTL 58) | 71 (20 while the Shield flew) |
  | deepest | **0** | 3.18 m | 1.02 m |
  | end | the 120 s limit | RTL (X7) | RTL (X7) |
  | P0 escape ticks (commanded basis) | 0 (0) | 0 (0) | 0 |
  | loop | 9.91 Hz | 9.98 Hz | 8.97 Hz |

  On: 1190 ticks, 1168 flown by the Shield, the measured velocity on every
  tick (LOCAL_POSITION_NED) and never set aside; FSM edges G1 G7 G1 G7 G1 G2
  G8 G1 G7 G1 G7, no X7. Its one Loiter (G2 at 47.3 s, 1.01 m/s) was flown
  as a GUIDED stop for 9 ticks - BRAKE's stop was predicted legal, but the
  aircraft was over 0.5 m/s - and handed to BRAKE at 0.45 m/s; G8 at 50.4 s;
  no forced hand-over, 0 stop-advised ticks; the HUD read "Loiter(GUIDED)"
  on those 9 ticks. Off: G3 at 40.9 s with the aircraft at 2.5 m/s, BRAKE
  carried it into the margin (1.68 m deep), X7 sent it home at 43.0 s and
  the RTL climb reached 3.18 m - the 13 Oct pattern; none of its 79 ticks
  was flown by the Shield. Real-time factor 0.965 / 0.962 (0.935 on 14 Oct);
  `demo/out/pas_ap_ovf_compare_20261021.mp4` stacks the two (120 s).
  **Read with care:** the detector again did not box the car (0 of 186
  subject estimates within 6 m on the on run, 0 of 190 off; the nearest
  180 m away), so these runs test the Shield, not tracking; and the
  trajectories differ run to run (first building-clearance flag at 44.4 s
  on, 40.3 s off, 40.8 s on 14 Oct): one flight each is not an A/B proof.

#### Added
- **The Shield forecasts from the aircraft's own velocity** (T1a;
  `State.velocity`, optional; `guardrail/response.py`;
  `docs/DESIGN-own-velocity-forecast.md`). With a measured velocity each
  forecast pose first flies the setpoints already sent for the reaction time
  (`Shield.note_flown`, the pipe), then turns towards the candidate at the
  acceleration limit. Fitted on the six recorded ArduPilot flights
  (`tools/fit_velocity_response.py`): identified 0.35 s / 2.0 m/s^2; the
  Shield uses 0.45 s / 1.75 m/s^2, the most accurate point whose MEDIAN
  braking error at 1.5 s, in the form the Shield flies, is not short
  (+0.03 m; not a bound: p25 -0.33 m there, the median -0.16 m at 2.0 s).
  The velocity only ADDS caution: every rule is judged on the own-velocity
  forecast and on the old commanded one, and a rule either breaks is broken
  (a seeded property test: nothing the old Shield refused is emitted clean);
  from an illegal position the velocity is set aside and the decision is
  the old Shield's. Without a velocity every decision is byte-identical
  (18,000 seeded decisions against c77c77d). Replayed, the 14 and 13 Oct
  approaches flag the building 1.09 and 0.98 s earlier; in closed loop on
  the identified aircraft both stay outside the 3 m margin (3.28 / 3.48 m)
  where the commanded forecast is inside on 73 ticks of each and ends in X7.
  At 50 rules `_check` takes median 0.87 / p99 1.23 / max 1.51 ms with the
  velocity, against the 5 ms budget.
- **On by default on the Project AirSim + ArduPilot rail** (pymavlink and
  MAVROS, Shield-on arm; `--own-velocity-forecast off` restores the
  commanded forecast and BRAKE hold). The SITL rails log the velocity but
  default off (the response was measured on the Project AirSim quad); the
  harness and `demo/follow_vlm.py` pass none.
- **A hold that stops clear** on that rail: the FSM's Loiter is flown as a
  GUIDED stop through the Shield (`Shield.advise` of a zero action) while
  BRAKE's predicted stop would break a hard rule or the aircraft is over
  0.5 m/s, then handed to BRAKE; capped at 3 s (ArduCopter's GUID_TIMEOUT),
  and a stale link or a 0.3 s gap in the stream hands it over at once
  (`metrics.hold.stop_clear`, `forced_handovers`). The FSM's states, edges
  and inputs are unchanged. A stop-advised tick has its own audit record
  (`decision_kind: "stop_advised"`), rows carry `stop_advice` and
  `p0_escape_commanded_basis`, and the HUD names a deferred Loiter.
- **A vehicle-response comparison arm in the stress harness** (T1b;
  `--response-arms ideal ardupilot`; `docs/DESIGN-harness-vehicle-response.md`):
  each episode flown again on ArduPilot's measured response (delay 0.08 s,
  2.4 m/s^2, jerk 5.7 m/s^3, lag 0.03 s; `tools/measure_vehicle_response.py`
  over the recorded flights), labelled `/response-ardupilot`, ranked in its
  own top-K, written to `demo/out/cmp_*`, never in the headline counts, pins
  or exit code. Nightly, ideal -> ArduPilot: unsafe-position entries 6 -> 12,
  autopilot polygon entries 12 -> 84, P0 escapes 18 -> 18, true-state P0
  ticks 8894 -> 12189 judged on the command and 8894 -> 16382 on what the
  airframe flew, fail-safe labels 532/705 -> 490/705. Flying the arm changes
  nothing of the headline run, field for field.
- **Legal hold and landing points** (T1c; `guardrail/holdpoint.py`,
  `EscalationFSM.needs_hold` / `hold_at`; `docs/DESIGN-hold-points.md`).
  Entering Loiter or Land the FSM asks for a point and the Shield's own rule
  code answers: in place where a stop there is legal through the lookahead
  (every legal hold of the library is unchanged), else the nearest legal
  point within 60 m, reached by a straight transit judged at every instant,
  not at sampled poses - it never enters a rule in force that the aircraft
  is outside of, and leaves one it is in only by getting shallower, by the
  Shield's own depth (160 random transits in 40 seeded worlds: none called
  legal enters or dips). The search spends at most 600 units of about
  0.1 ms (a count, not a clock); a fruitless one costs 29-41 ms on the
  desktop. No FSM edge, guard or setpoint moved; Loiter / Land records gain
  `hold_point`, `hold_legal`, `hold_needed`, and the audit record (version
  3) `fsm_hold_point`. The harness's modelled autopilot flies a moved point
  as a GUIDED transit; `--autopilot-hold in_place` flies what the rails fly,
  c77c77d's rows exactly. **No flight rail flies a moved hold yet.**
- **The DSL's first lint WARNING class** (`python -m guardrail.compiler
  lint`, `ConstraintCompiler.lint_warnings`; never refuses): a hard zone
  whose window can open on an aircraft that cannot leave its margin ring
  within the lookahead, or, for a mission start, before it opens. No shipped
  policy warns at the policy level (29 files); the three
  time-window-opens-on-vehicle cells carry one each.
- **The WP1 cold start, timed and published** (T1d; `docs/data/
  wp1_roundtrip.json`, `docs/DESIGN-wp1-cold-start.md`): a signed 50-rule
  bundle to a ready Shield in fresh interpreters, p95 376 ms against the
  grant's 1000 ms (worst bundle, imports included; whole process p95
  504 ms), every run verified, and a tampered, an untrusted and a
  bad-signature bundle refused. Desktop only, not the Orin.
- **Parquet export** (`tools/export_episodes.py`, pyarrow optional): one
  typed column per field, read back against the JSONL before a file is kept
  (a missing column fails it); 6 stored episodes, 4616 step rows, equal. The
  3.11 extras CI job installs pyarrow 26.0.0.
- **SROS 2 behind `SROS2=1`** (`sitl/run_ros2_demo.sh`,
  `tools/sros2_setup.sh`, `tools/sros2_check.sh`): one enclave per ROS 2
  process under Enforce, on the lab CA; the live check 6/6 in WSL Jazzy. Not
  flown; the default `SROS2=0` changes no command line.

#### Changed
- **Stress-harness results with hold points** (grant arm; the modelled
  autopilot flies moved holds, which no rail does): library pass / tracked /
  fail / known 132 / 22 / 0 / 22 -> 133 / 22 / 0 / 21, fail-safe labels
  140/167 -> 141/167, autopilot-flown P0 ticks 1738 -> 1153; nightly
  508 / 66 / 0 / 158 -> 511 / 66 / 0 / 155 and 532/705 -> 535/705; smoke
  unchanged (52 / 6 / 0 / 4). These differences are by design, and only one
  fail-safe fix is robust: `dyn-nfz-rotate-spin` at 9 deg/s (pin removed;
  its 3 nightly seeds too). The six `three-simultaneous-violations` cells
  first read as passes only because the 300-tick run ended 0.2 s before the
  Loiter timeout; flown 400 ticks, G5 sends every one home, so they stay
  pinned on both labels (what is real: X6 no longer fires, and the climb
  into the band flies 1-2 autopilot P0 ticks where 72-78 were flown). The 6
  Shield P0 escape ticks of the time-window-opens-on-vehicle cells are
  unchanged (flown before G3); their hold moved to (60, 31.9), autopilot P0
  38 / 33 / 28 -> 18 / 13 / 8.
- With the velocity, the rail's P0 escape re-check is made with it, as the
  decision is: never laxer than the commanded basis (property test), and it
  can be stricter; rows and `metrics.shield_forecast` carry both.
- Bundle extras are `sweep-episode-extras/3` (both `arm.response` and
  `arm.autopilot_hold`); `rerun_from_extras` reads /1, /2 and /3, and a
  bundle from before the hold points is re-flown in place.
- The city follow's defaults after the first flight (c77c77d):
  `--accel-max` and `--guard-release-accel` 2.0 -> 2.5 m/s^2, `--catchup-mps`
  2.5 -> 3.0.

#### Fixed
- `tools/build_eval_data.py` counted the commits since the 2 Sept meeting
  with a bare `--since 2026-09-02`, which git reads as that date at the
  current time of day: run after 22:58, the meeting day's 22:58 commit
  (96746a9) dropped out (15 instead of 16). It now passes midnight, and a
  test checks the argument git receives (c57e007).
- Found by the wave's reviews, each shown failing first: the episode
  exporter split a run's audit at every stop-advised tick (40 of 60 rows
  joined on a test run; now 60 of 60, one flight); the Parquet read-back
  kept a file missing a whole column; the cold start read a loader that
  never verified the signature as within budget; the hold search could fly
  through a zone closing on the aircraft and judged a transit only at poses
  1 m apart (a fuzz: 27 of 195 moved holds entered a zone they started
  outside of); a candidate inside k nested zones was charged k zone reads
  (with 8-10 nested squares the hold stayed inside where 51.5 m out was
  legal; now moved, 14 ms); a speed cap's reason called the default an
  enforced cap; a zone with `valid_time: {}` was no wall to the lint; and a
  lint call with no legal exit stepped a week at 0.05 s (12.6 s; now
  0.03 s).
- At the merge: the hold plan read the tick's momentum-aware stop check and
  moved 24 of 337 seeded Loiter holds the FSM had just called legal (it now
  reads the FSM's own position check); the response arm flew a moved hold
  as the ideal vehicle's command, passing a point 10 m away by 4.4 m (now
  ArduPilot's position controller, under a metre).

#### Not fixed
- No flight rail flies a moved hold point, so the harness's autopilot
  numbers with them are modelled (`autopilot_hold_model`); the 2 Shield
  escape ticks per opens-on-vehicle cell remain (flown before G3; the lint
  names the cause).
- The response model is one form fitted to six flights of one simulated
  airframe: on an aircraft slower than the Shield's numbers (0.55 s,
  1.5 m/s^2) the closed loop's approaches entered the margin 0.8-1.0 m deep
  on 26-33 ticks. BRAKE is an envelope of two recorded entries, neither
  clean (a SITL BRAKE step test is open), and the velocity set-aside drops
  the momentum for every rule when any hard rule is broken.
- Not wired: the own-velocity forecast in `demo/follow_vlm.py` and in the
  harness's Shield, `land_site_ok` in `demo/follow_vlm.py`; the SITL rails
  default off. The cold start and the hold search are measured on the
  desktop only, not the Orin; SROS 2 has not flown, and no per-topic
  permissions are written. The ArduPilot rail still has no start gate.
- For the PI: the velocity in the P0 escape re-check; a GUIDED transit
  inside the grant's "Loiter"; whether `three-simultaneous-violations`'
  label or our 30 s Loiter timeout is wrong; a lint warning class.

#### Tests
- The CI loop runs 103 test files (98 before): new
  `tests/test_own_velocity_forecast.py` (27/27),
  `tests/test_harnesstruth_response.py` (19/19),
  `tests/test_hold_points.py` (33/33), `tests/test_hold_transit.py` (14/15,
  the opt-in wall-clock test skipped) and `tests/test_window_lead_lint.py`
  (11/11); `tests/test_build_eval_data.py` (10/10) checks the `--since`
  argument.

### 2026-10-19 — an October bug hunt: 100 verified defects fixed, a smoother city follow (not yet flown), and a complete CI list

A nine-area review on 18-19 Oct found 107 candidate defects; independent
verifiers confirmed 100 (6 refuted, 1 uncertain), fixed in nine groups, each
reviewed adversarially and again for the reviewers' notes (1b95383..2e81f21,
33 commits). Nothing here has flown: the city follow's new motion is tested
offline only, and no VLA flies the current demos. No pinned digest, decisions
fixture or published `docs/data` file was regenerated; the stress harness's
smoke profile scores 52 pass / 6 tracked / 0 fail / 4 known failures before
and after, and the smoke and nightly sweeps score as before apart from the
added `p0_violation_escape_rate_status` field.

#### Fixed
Safety Shield (`guardrail/shield.py`):
- **A crawl out of a P0 breach is floored even with nothing else wrong.** A
  legal but too-slow escape (from a zone, a stand-off or clearance ring, or
  under an altitude floor) gets the repair floors when nothing else, or only a
  monitor_only rule, is violated; before, 1 m inside an NFZ leaving at
  0.15 m/s took 6.7 s alone (0.5 s beside a yaw breach), and a 1e-6 m/s climb
  from under a floor passed every tick. The floored action flies only if it
  re-checks clean and carries only theta-exempt repairs (else the raw action
  flies, traced `floor_refused`). `filter()` can now return repairs with no
  violation, which the FSM reads as clean; `ShieldDecision.touched` counts
  them, so rails that log it show these ticks as interventions.
- **Hot-apply refuses what it would not enforce or cannot build, atomically.**
  An `altitude_ref: MSL` event (enforced as metres above ground before) is
  refused (`HotApplyRefused`, REST 422), and one whose runtime fails to
  compile changes nothing and returns 422; before, it returned 500 with the
  live policy a generation on and the rule listed but not enforced.
- A NaN or inf subject is dropped instead of silently disabling the stand-off;
  a `subject-finite` violation is raised only while a hard, enforced stand-off
  binds it. `Shield.forecast()` turned headings 57.3x too slowly (rad/s added
  to degrees); first-feasible mode judges theta on the per-axis repair sum.

Policy DSL, compiler, manifest and replay:
- A higher layer that re-declares a lower layer's hard rule can no longer
  switch it off or swap it wider, at ingest or by hot-apply; the winner
  records the overridden layers in a new optional `locked_by` (11 additive
  lock pins, no stored hash moved).
- Schedules refuse `days: []` and clocks like '10:75'. CSP altitude bands are
  intersected per instant, so day/night bands no longer crash `compile_csp`;
  a real clash raises `CSPAltitudeConflict`, now listed and counted in the
  coverage and KPI reports. A class-specific stand-off's reason binds it to
  the followed subject, not "any pedestrian" (the sentence: see Not fixed).
- An empty flight log is refused by `write_replay` / `verify_replay` (it
  verified as a perfect run) and scores outcome fail (`no_rows`); a log the
  autopilot flew entirely reads escape rate None, not 0.0 (none of 1835
  stored flight logs or 55 replay bundles is one).
- Manifest: an ambiguous HuggingFace snapshot or unreadable weights record
  'unresolved'; `sim_speedup` was inverted. `AeroVLABackend`'s skeleton
  raises on its first tick instead of hovering.

Escalation FSM and KPI report:
- A `brake` rule where a stop is legal is answered by the Shield's stop: a
  soft P1 brake beside a hard NFZ no longer takes G3 into LOITER.
- `failsafe_instrumented` reads the value, not the key (Shield-off arms log
  nulls); the per-tick false-trigger count, 0 by construction, is always a
  vacuous zero, and the report shows labelled episodes (`k of m ep`).
- A floored tick is not a repair attempt (`repair_floored_ticks`; a scan of
  644,234 stored rows found none). Bundles with no forecast label file under
  `/forecast-unlabelled`, so a comparison arm is no longer dropped.

REST / gRPC API and harness:
- **REST mTLS allow-list bypass closed.** A lab-CA certificate not on the
  hot-apply listener's common-name allow-list could get a POST applied to the
  flying Shield by sending it in the same write as its TLS Finished (only the
  response was dropped); the gate now aborts before uvicorn parses anything.
- The harness's session forward speaks mTLS (`serve --rest-pki`). StopSession
  waits for a flying episode, stops a session once, and still stops it if it
  raises. Bad lat/lon: 422 / INVALID_ARGUMENT, not 500; NaN or Infinity in a
  gRPC payload: INVALID_ARGUMENT.

The city follow (`demo/follow_vlm.py`):
- **Smoother motion against the pitch/roll jerks** when the car turns or pulls
  away: a dt-aware vector slew limiter (speed-up 2 m/s^2, braking never softer
  than the old 0.4 m/s step, turns 2.5 m/s^2, yaw-rate slew 2 rad/s^2), a
  guard smoother (urgent brakes immediate), catch-up capped at 2.5 m/s and a
  line-of-sight yaw feed-forward (0.7, capped 0.4 rad/s). Offline closed loop:
  speed-up peak 3.11 -> 1.62 m/s^2, jerk peak 8.35 -> 3.44 m/s^3, turn lag
  5.0 -> 1.4 deg, range max 24.1 -> 23.8 m. `--legacy-motion` restores the
  old motion; `params.motion` records it. Not yet flown.
- City manifests file `vla_model_hash` = `none:hand-written-controller`, with
  OWL-ViT under `pilot.detector`. Without `--identity` the estimator also gets
  each box at its capture's pose and depth; `det_hit_rate` counts mission
  inferences (13 Oct: 0.993; over all inferences, 0.495); a follow without
  subject truth reads mission_success null; an abort before the first tick
  exits non-zero.
- FenceGuard's brake bites at once inside a building's clearance ring with a
  no-fly zone in range (the ring was measured against the zone, 13 m instead
  of 4.7 m); car missions with `--level-peds` stop polling the 40-name crowd
  (429 polls a flight).

Project AirSim + ArduPilot rail (`demo/pas_ardupilot/`):
- **A frozen autopilot link fails safe.** SITL dead behind mavlink-router, a
  closed socket or MAVROS losing its FCU no longer counts as live: one
  zero-velocity setpoint, then nothing; 3 s stale ends the episode as "link
  lost" and withholds the KPI grade.
- An unclassified subject ("a child") binds the catch-all "*" stand-off
  instead of turning every stand-off off; the subject is placed from its own
  capture's pose and depth; the loop paces to a 10 Hz deadline (8.97-9.16 Hz
  before) and dwell is time-weighted from timestamps.

SITL and ROS 2 rails, deploy:
- **The pymavlink rail flew on a pose 3-6.6 s old** (pymavlink's oldest queued
  LOCAL_POSITION_NED); it now takes the newest, on a 10 Hz deadline. Every
  stored `sitl_*` and `run_gazebo_demo.sh` run was measured on the delayed
  pose (`sitl_ped_on`'s 21.6 s `alt_violation_s` and its X6 escalation too).
- **ROS 2 node: a stale pose fails safe.** A watchdog (`--pose-timeout-s`,
  1.0 s) holds the aircraft; the FSM is not stepped on a stale pose (from
  Loiter, G8 could return to GUIDED on a frozen position), and
  `EscalationFSM.unobserved()` makes G7 / G8 wait T_recover after the gap.
- ROS 2 node: a pilot or failsafe leaving the node's hold is a takeover (no
  GUIDED, no LAND after it); mode requests go to the adapter only when it
  subscribes. All rails accept `--target -20,15`. Both Docker images keep
  numpy 1.26.4: neither could be built (ResolutionImpossible), nor has been.

Stress harness, tools and tracker server:
- **Known-failure pins name their gates.** Any other gate failing in a pinned
  cell is NEW and exits 1; before, a P0 escape, a breach or an unsafe end
  there read KNOWN with exit 0. A pin can bound its gate: the three
  time-window-opens-on-vehicle cells are bounded at the measured 2 escapes
  (0.0073-0.0076).
- `--backend sitl` scores a rail flight from its own files (every gate read
  'not measured'); a failed KPI rollup exits 4 and writes no partial report.
- `tools/check_claims.py` passes on a clean clone and scans the code packages
  and root guides (288 files clean); `tracker/serve.py` refuses a foreign
  Host, Origin or non-JSON save, so a web page can no longer wipe the board.

#### Changed
- Every city flight's default motion changed; a recorded flight needs
  `--legacy-motion` (plus `--legacy-tick-pose`, `--legacy-servo-width`) for
  its old motion, and its tracking, guard-hold and intervention counts are not
  comparable with new flights'.
- Perception-rail flights should run at about 10 Hz, not 9.15 Hz, with
  time-weighted dwell (about 8-9 % above tick count x 0.1 s on a 9.16 Hz run).
- The KPI-report families of the 2396 stored legacy stress bundles are
  renamed on regeneration (a `--baseline` diff shows each as new and as gone);
  `expect: known_failure` without `known_failure_fails` is refused at load.
- `Shield.set_subject` takes an optional `velocity=(vx, vy)` and then
  forecasts the subject at constant velocity; without it every decision is
  byte-identical (18,000 seeded stand-off decisions compared). No rail passes
  it yet.
- README and `docs/index.md` say no VLA flies the recorded demos (a
  hand-written controller steers by OWL-ViT's boxes, the SITL rails fly
  StubVLA, and the last VLA-commanded flights were on 4 Aug 2026), carry the
  13/14 Oct Shield-off controls (727/1100, 736/1099), and test their sample.

#### Corrected
- The site said the Shield forecasts 5 s at 0.1 s steps; the city
  demonstration flies 3 s at 0.5 s by default, the 14 Oct recording
  (`citylife_redcar_nfz_20261013`) included, which also flew the unsigned
  YAML policy. Its P0 escape 0.0 is on that forecast; separation and time in
  the zone do not depend on it.
- The 14 Oct start range is about 151-152 m, not 119-152 m; the preamble no
  longer calls P0 escape "the hard KPI"; `docs/aerialvla-ft-report.md` headed
  all rows "pure VLA", but its three 100 % rows flew goal_blend 0.55.
- The August and September reports open with a dated note on the
  retractions; the 13 Oct BRAKE ticks were ArduPilot's BRAKE mode under the
  FSM's Loiter; the perception rail's 27.9 s is 279 ticks x 0.1 s (30.4 s by
  timestamp); RUNBOOK's Rail 3 is the grant's dev topology flown by the stub.

#### Not fixed
- The altitude and corridor-band trend monitors accept any climb and the
  clearance monitor any forecast gain, so where a floor is refused a crawl
  flies with no violation; raising them to the reference's 0.1 m/s changes
  pinned `_check` answers and needs a deliberate regeneration and re-score.
- The CSP stand-off sentence still says "any pedestrian": rewording it needs
  the LLM-written paraphrase sets and the OpenVLA tokenizer table regenerated.
- No rail passes the subject velocity (`follow_vlm`, `perception_node`,
  `ros2_shield_node`). The city rail stays on 3 s / 0.5 s until it does and
  the moving-subject stand-off repairs are measured against theta with the FSM
  on: held still, the grant's forecast fired StandoffHold on 529 of 1190
  replayed city ticks (44 %).
- FenceGuard's hysteresis, continuous k and clear_aim margin are unchanged
  (pinned single-call values); `follow_vlm` still matches "a delivery man" to
  the van class.
- Off CI: 6 test files that need OpenCV, Pillow, projectairsim or node
  (`test_range_and_lock` holds most city-follow regressions);
  `test_deck_scorer_parity` counts its JavaScript checks as passes without it.
- The ROS 2 node does not check at mission start that the adapter is alive;
  during a stale-action hold the FSM's clean streaks keep running.
- Documents citing `sitl_*` runs are not yet marked as delayed-pose; stored
  KPI reports and `docs/data/csp_kpis.json` show old cells until rebuilt; the
  sweep's SITL command passes the subject unrotated.

#### Tests
- The CI loop runs 98 test files (40 before). The 6 left out are listed with
  measured reasons, and `tests/test_harnesstruth_ci.py` fails on a file in
  neither list, a listed test that can return None early, or a runner that
  does not count SKIP; verified on Linux 3.10.22 / 3.11.17 venvs, a clean
  `git archive` copy and a depth-1 clone. The 3.11 extras job installs
  uvicorn and jsonschema and fails on a SKIP from the REST-listener tests.
- 7 new test files, and regression tests in the existing ones (53 test files
  changed in all).

### 2026-10-14 — recorded videos of both rails, the ground-truth question answered, an English explainer pack

#### Flown
- **The usual demo video, re-flown on today's code**
  (`demo/out/citylife_redcar_nfz_20261013`, simple_flight rail, the CityLife
  red car with the no-fly zone and the policy indicator): P0 escape 0.0,
  0.0 s in the zone, 0.991 of the time within 30 m of the car, minimum
  separation 12.0 m; 5011 frames at 19.28 Hz. Video
  `citylife_redcar_nfz_20261013_demo.mp4` (260 s).
- **The ArduPilot rail recorded for the first time**, the close approach
  with the Shield on and off (Project AirSim + ArduPilot SITL over pymavlink,
  none of the grant's topologies, not KPI-grade):
  - Shield on (`demo/out/pas_ap_video_on_20261014`): 517 ticks, 0 P0 escape
    ticks in 388 Shield-flown ticks; the FSM braked, then sent the aircraft
    home (RTL); real-time factor 0.935 (under the 0.95 band); 1105 frames
    at 19.19 Hz.
  - Shield off (`demo/out/pas_ap_video_off_20261014`): 1099 ticks, 736 P0
    escape ticks (rate 0.6697), the raw command flown into the
    building-clearance margin; 2372 frames at 19.75 Hz.
  - `demo/out/pas_ap_video_compare_20261014.mp4` stacks the two (120 s).
  - Where the aircraft was, not only what was flown: with the Shield on it
    was inside a P0 margin (building clearance) on 71 of 517 ticks, 20 of
    them while the Shield flew (up to 1.0 m deep, no emitted command
    breaking a rule) and 51 under RTL; with the Shield off, on 739. The
    Shield's forecast starts from the commanded velocity, not the aircraft's
    own (`_predict`; `State` carries no velocity), so the aircraft's carry
    while it slowed from about 3.2 m/s went unseen. Next: a forecast from the
    aircraft's own velocity and a hold that stops clear of the margin.
- **The ground-truth question is answered: the truth was right, the detector
  was not on the car.** The 13 Oct gap between the truth (Car_10) and the
  detector was not a frame error: the rail's Car_10 poses lie on
  follow_vlm's validated Car_10 track (median 0.52-0.54 m). The recorded
  Shield-on video shows the detector's box on a red traffic signal; across
  the close flights 0 estimates came within 6 m of the car, and Car_10 was
  in the camera's view on 10 of the 149 estimate ticks. The rail starts with
  the car about 151-152 m away (`subject_range_m` on the first tick of each close flight: 150.8-151.9 m; the 119-152 m first written here took in the Shield-off flights' nearest approach, 117-119 m, and was corrected on 2026-10-19) and has no start gate (follow_vlm waits until it
  sees the car); until one is ported, this rail's follow scores stay at 0
  and its flights test the Shield, not tracking.

#### Added
- The perception node records like the usual demos (`--save-view`,
  `--record-hz`, `--record-height`; `run_pas_ardupilot.ps1 -SaveView`): the
  drone's camera with the policy indicator (FSM state, Shield-off banner)
  and the chase camera, off the control loop (loop 9.15 -> 9.12 Hz beside a
  CPU-bound detector stand-in). `tools/make_compare_video.py` stacks two
  runs and refuses to overwrite an existing file.
- A frame check for the subject's ground truth (the aircraft read through
  the same simulator call as the car). On the CityLife build the aircraft's
  actor name (UnrealRobot) is not found, so the check reports partial: only
  the EKF against the simulator's kinematics was measured (median 0.388 m).
- An English explainer pack (`docs/share/2026-10-14-Explainer/`, local):
  a 22-slide deck built from `tools/deck/build_explainer_deck.js` on the usual
  template, an English and an Indonesian cheatsheet, and likely questions
  with model answers; five figures drawn from code
  (`tools/deck/build_explainer_figures.py`, `docs/img/explainer_*`): the
  end-to-end flowchart, the components, the topologies, the Shield per tick
  and the evidence chain.

#### Changed
- The perception node's report says what its counters count: "flown-violation
  ticks" (any priority) beside the P0 escape ticks from kpi.json.

#### Retracted
- The 749 P0 escape ticks of the 13 Oct Shield-off close approach (13 Oct
  (later) entry): 749 counted ticks that flew a violation of any priority,
  the 22 P1 speed-cap ticks included; the P0 escape ticks were 727 (building
  clearance 705, stand-off 17, both 5), which is what the 0.661 rate was
  computed from.

#### Tests
- New: `tests/test_railvideo_{recording,compare,truth_frame}.py`,
  `tests/test_explainer_{figures,deck}.py`; each failed on a pristine copy of
  5cf7a08 and passes now.

### 2026-10-13 (later) — G1 holds after the fence change; the perception rail flown close with the Shield on and off

Flights on Project AirSim + ArduPilot SITL (pymavlink, none of the grant's
topologies, not KPI-grade), after the wave-A commit, plus two CI fixes.

#### Flown
- **Gate G1 passed again** after `citylife-quad.param` changed (FENCE_TYPE 7,
  AVOID_ENABLE 0; `demo/out/g1_20261013_203404`): 3/3 runs, all four
  conditions. EKF p95 0.82-0.83 m against the 1.0 m limit (frozen-position
  null 18.96-18.98 m), climb 14.47 m from rest, landing 0.06-0.09 m from
  take-off, 0 fence breaches, 0 airborne collisions.
- **The red-car follow with an aggressive pilot** (`--want-width 0.5`, the
  pilot closes until the car fills half the frame), 120 s, flying the signed
  bundle of `follow_car_citylife` (signature verified; the first signed
  flight on this rail):
  - Shield on (`demo/out/pas_ap_close_on`): 532 ticks, 0 P0 escape ticks,
    173 interventions (AltitudeFix 103, ClearanceFix 85, YawClamp 19,
    StandoffHold 3). The FSM escalated twice to Loiter (a stand-off repair
    of 2.08 m and a building-clearance repair of 2.14 m, over theta 2.0 m);
    in the second hold the illegal depth kept growing (2.95 m in 2.07 s,
    fitted) and X7 sent the aircraft home; RTL reached home at 58.1 s. The
    hold flew as ArduCopter BRAKE and held 7.99-8.01 m over 42 ticks: the
    first flight evidence for the BRAKE hold, which had rested on a SITL
    probe.
  - Shield off, the same pilot and route (`demo/out/pas_ap_close_off`): 1100
    ticks, 749 P0 escape ticks (rate 0.661): building clearance 710,
    stand-off 22, kinematic caps 22. This is the rail's first Shield-off
    control.
  - ArduPilot real-time factor 0.966 (on) and 0.946 (off, just under the
    0.95 band); 0 fence breaches on both; the follow policy declares no
    keep-out zone, so no GeoFence zone was uploaded.

#### Found
- **The subject's ground truth and the detector disagree.** The truth reader
  logged Car_10 (the level's one red car) on every tick, 119-235 m from the
  aircraft to the north-east, while every estimate the detector produced sat
  45-164 m to the south-east; the median gap is 258-264 m. Either the truth
  pose is in another frame than the Shield's, or the detector follows
  something other than Car_10. Unresolved: no truth-based number from these
  runs is used.

#### Fixed
- Two tests added to CI with wave A failed on GitHub only:
  `tests/test_env_pins.py`'s stand-in `waf` lost its executable bit on Linux
  (the later `git add -A` re-read the file mode; Windows ignores it), and
  `tests/test_harnesstruth_bundles.py` needed the gitignored
  `bundles/fase3-sim-demo-v0.1.1.tar.gz`; it now skips visibly without it.
  `test_env_pins` was run on Linux (WSL): the eight setup and verify tests
  that failed there pass; the one left failing is that WSL venv's own numpy
  2.5.0 against the 2.4.6 pin.

#### Changed
- `scripts/run_pas_ardupilot.ps1 -NodeArgs` passes extra arguments to the
  perception node as given.

#### Tests
- `tests/test_env_pins.py` 37/37 on Windows, 36/37 on Linux (WSL's numpy);
  `tests/test_harnesstruth_bundles.py` 11/11 here, 10/11 + 1 skip in a clean
  copy.

### 2026-10-13 — the grant's forecast in the harness, rule clocks and hot-apply on the flying rails, a faster Shield, and the WP4 outputs

Four packages built in parallel on disjoint files from a re-verified inventory
of everything still open (the grant pages, the 5 Oct audit, the tracker and
the KPI evidence chain), each challenged by two reviewers who tried to break
it, then fixed. Nothing here has flown: every number is headless or a
desktop measurement. The Jetson Orin and the drone are not available.

#### Fixed
- **The stress harness flew a Shield the grant does not describe.** Its
  flying, oracle and checker Shields used a 3 s forecast at 0.5 s (6 poses);
  the grant locks 5 s at 0.1 s (50 poses). All three now use
  `guardrail.shield.LOOKAHEAD_S` / `LOOKAHEAD_DT_S` unless a spec overrides
  them, and a labelled 3 s comparison arm is kept out of every headline.
  Cells written around the 3 s lookahead were re-cut for their purpose at
  5 s, each mechanism in its description.
- **GUIDED setpoints flew open-loop under wind.** Every velocity the harness
  flies now goes through one closed-loop model of ArduCopter's position
  controller (Copter-4.5.7 `mode_guided.cpp`, `AC_PosControl`): steady wind
  is rejected up to 6.4 m/s, derived from the SITL default frame, not
  measured. `--wind-model open_loop` keeps the old model.
- **The FSM's RTL -> Land edge (G6) had no input.** `guardrail/rtl_path.py`
  judges every RTL tick: the straight leg home against the P0 zones and band,
  progress over 5 s, and a battery hook.
- **Time windows and switches were inert in flight.** No rail gave its
  Shield or its CSP a clock. Both SITL rails and the perception node now pass
  one rule clock to both (`--clock-start ISO8601` scripts it; otherwise the
  wall clock in +08:00, recorded either way), and zone dwell skips time in a
  zone the clock had switched off.
- **A YAML policy flew with no acknowledgement.** The flight rails fly the
  signed bundle of their policy (`tools/build_flight_bundles.py`, into the
  gitignored `bundles/`); a YAML flies only with `--dev-yaml`, is recorded so
  and is never KPI-grade. The stress harness passes `--dev-yaml` for YAML
  references and pins every bundle with `--expected-hash`, which the ROS 2
  node now declares and checks before take-off (it had dropped the flag).
- **A pilot whose action stream stopped kept being flown** by the ROS 2
  node. A stale-action watchdog (0.5 s, five missed 10 Hz ticks) now holds
  the aircraft through the FSM's path.
- **The Shield's moving-zone path was over the grant's 5 ms query budget**
  on GitHub's runner (5.71 ms) and close to it on a loaded desktop: 70 % of
  the time went into rebuilding every zone's polygon on every check.
  `guardrail/ir.py anchor_reach` now builds a zone only where the forecast
  can meet it; in 200 checks the Shield builds 199 zones instead of 9600.
  `_check` median at the grant's load, same session: moving 2.53-2.64 ->
  0.45 ms, static 0.35 -> 0.30 ms; decisions identical on every compared
  sample; the per-fence loop (the null) takes 222 ms.
- **The audit's per-repair iteration count was wrong** on 1324 of 4257
  seeded multi-pass records (keyed by operator name), and a relaxed chain's
  failed run vanished from the record. Both fixed.
- **A follow the perception rail could not score claimed mission success**:
  with no subject ground truth, `frac_within_30m` and `standoff_s` were None,
  which `guardrail/kpi.py` read as no failure. It is now not measurable.

#### Added
- **The REST hot-apply API on the flying Shield** (`--hotapply-port` on both
  SITL rails): loopback by default, mutual TLS with the harness CA off it,
  rules swapped under the rail's lock between ticks, each event recorded as a
  new policy generation with its receipt-to-swap time.
- **The perception rail**: a follow mission so a CSP is written per
  generation; the policy's clock-proof zones uploaded to ArduPilot's GeoFence
  before arming (`citylife-quad.param` now FENCE_TYPE 7, AVOID_ENABLE 0; G1
  not re-run since); the followed car's simulator pose logged for evaluation
  only (`--subject-actor`, Car_10 for a red car), with the true nearest range
  and the stand-off dwell; `--hold-mode LOITER` flying the literal LOITER with
  an RC3 override of 1500 on SITL (BRAKE stays the default); the MAVROS path
  brought to parity, and `dev` offered for Project AirSim + ArduPilot over
  live MAVROS evidence (never over pymavlink).
- **The audit record as a Pydantic v2 model** (`AuditRecord`,
  `policies/audit_record.schema.json`) with body-frame actions in the grant's
  keys, every operator considered per pass with its result and skip reason, a
  measured iteration count and a `time-window` category; a reader upgrades
  every stored shape (22 123 records in 73 files).
- **Repair operators as a pluggable library** (`guardrail/repair.py`, a
  `RepairOperator` Protocol), decision-identical by default, plus three modes
  off by default: the grant's first-feasible selection, PathRepair (frees
  standoff-wedge: goal reached, minimum range 10.022 m against the 10 m
  ring), and a segment check between forecast poses (catches a 0.5 m ring
  between two poses at 12 m/s).
- **Self-contained harness bundles** (the expanded cell, events, arm, seed,
  policy and generations in `manifest_extras.json`) and
  `tools/rerun_episode.py`, which re-flies one and compares the flight log by
  hash (a changed seed is the null).
- **Episode exporters** (`tools/export_episodes.py`): step and episode
  records with the grant's names, JSONL always, Parquet when pyarrow is
  installed (it is not, in either env); audit records joined only to the
  flight they belong to.
- **The KPI report's missing sections**: WP1 and WP2 (written to
  `docs/data/csp_kpis.json`: at 120 tokens the compiler keeps 105/105 P0
  rules where a naive cut keeps 104/105; null 0/105), the 100 ms tick budget
  per rail, A/B and paraphrase printed n/m until flown, repair magnitude by
  priority and operator class, vacuous zeros named, a `--baseline` diff, and
  evidence bound to the run's own signed replay bundle.
- **The grant's 10 gRPC RPCs** (StartSession, StopSession, InjectPolicy,
  IssueTask, StreamTelemetry, StreamEvents added), a live mode driving the
  episode runner, HotApply forwarded to a bound REST endpoint, idle sessions
  expiring. Headless only.
- `tools/sros2_setup.sh`: the SROS 2 keystore and enclaves seeded with the
  harness CA, checked with `bash -n` and `--dry-run`. SROS 2 is not enabled.
- Pins: shapely 2.1.2; numpy 1.26.4 on Python 3.10, 2.4.6 on 3.11+.
- `.github/workflows/nightly.yml` (the nightly profile, headless). It cannot
  run yet: GitHub fires scheduled workflows only from the default branch,
  which on this repository is `main`, while the workflows are on `master`.

#### Changed
- `docs/data/scenario_sweep.json` republished on the grant's forecast and the
  closed-loop wind model: 132 pass, 22 tracked, 0 fail, 22 known failures over 176 episodes (165
  cells; the 3 s arm reported apart); P0 escape 0.000133 over 175 Shield-on
  episodes (6 Shield-flown ticks, all in the time-window-opens-on-vehicle
  known failures; passthrough null 0.236); fail-safe trigger correctness
  0.838 (140/167; 26 false, 1 missed; 100/122 = 0.820 over distinct flights),
  below the never-trigger null 0.940 and the grant's 0.99; autopilot-flown
  P0 ticks 1738 of 14 287 (LAND 1004, LOITER 529, RTL 205) in 17 episodes,
  RTL refused (G6's input) in 3. Profiles: library 165 cells, smoke 55
  (62 episodes), nightly 221 (732).
- The KPI report keeps the 3 s comparison arm in its own `/forecast-3s`
  rows instead of pooling it with the grant's.
- `sitl/setup_sitl.sh` pins numpy 2.4.6 (as requirements.txt does for
  3.11+) and installs fastapi / uvicorn for `run_sitl_demo.py
  --hotapply-port`; `start_ardupilot.sh --mavros` starts MAVROS as system id
  255 (SYSID_MYGCS, so ArduPilot takes its RC override) and records its fcu
  url. Neither was re-run in WSL.
- The ROS 2 node's rows carry the end-to-end tick latency; no flight value
  exists yet.

#### Retracted
- The published library sweep of 12 Oct (152 pass, P0 escape 0.0 over 194
  Shield-on episodes, fail-safe correctness 0.823) was flown on a 3 s / 0.5 s
  forecast the grant does not describe; on the grant's 5 s / 0.1 s with the
  closed-loop wind model it reads as above.

#### Not fixed (found, for the FSM and Shield work next)
- When a time window opens over the aircraft and no action clears the
  forecast, the Shield flies its best effort out with the P0 violation on it:
  6 ticks over the library's three time-window-opens-on-vehicle cells, all
  known failures. This is why the library's P0 escape headline is no longer
  0.0.
- The FSM can hold LOITER on a spot about to become illegal, or one a sweep
  returns to; a hold can drift through a zone that clears before any Loiter
  -> RTL edge; G6 lands in place, which can be inside a zone.

#### Tests
- New test files: `test_harnesstruth_{forecast,rtl,wind,bundles,ci}`,
  `test_railinputs_{fence_export,clock,policy,manifest,node,hotapply,perception}`,
  `test_shieldcore_{repair,audit,identity}`,
  `test_wp4outputs_{kpi_report,exporters,harness}`; each failed on a pristine
  copy of 4418ae0 and passes now. Deliberate defects applied to copies of
  the code: harness 38 (36 caught, 2 equivalent), rail inputs 45, Shield 30,
  WP4 outputs 48, all caught. `tests/fixtures/` lets
  the 8 Oct regression run without `demo/out`.
- Full suite on Python 3.10: 2526 of 2536 over 92 test files. On 3.11 the
  guardrail package's tests pass (2494 of 2534 overall: demo tests that need
  cv2, Pillow or projectairsim cannot run there, plus visible skips). On
  3.10 the ten not passing are nine visible skips (FastAPI is not installed
  in that environment; one sweep test) and the ITRI deck-pack check, which
  passes once the deck is rebuilt from a clean tree.

### 2026-10-12 (later) — CI green: a clamp that repaired rounding errors, and a timing test fit for shared runners

GitHub's test workflow had failed on every push since 8 Oct, so its smoke
step never ran. Two causes, both fixed here.

#### Fixed
- **SpeedClamp logged "repairs" of float-rounding excesses.** The kinematic
  check allows 1e-9 over a cap, but the repair operator compared strictly, so
  an action another operator had already scaled onto the cap and that came
  out at 4.000000000000001 m/s was "clamped" to 4.0 and logged as a repair.
  Whether that happened depended on the platform's floating point: on Windows
  nfz-head-on logged 82 such repairs, on the Linux runners none, so
  `tests/test_sweep.py`'s pinned repair counts held here and failed on
  GitHub. The clamp (and the FSM's mirror of it, `KinematicCaps.clamp`) now
  has the check's tolerance. Repair counts move (nfz-head-on 346 -> 264,
  corridor-curfew-in-hours 412 -> 300, standoff-reclassified 360 -> 300);
  every final position, status and fail-safe figure is unchanged, and
  `docs/data/scenario_sweep.json` is republished with only `repair_count`
  moving (97 of 195 episodes).
- `tests/test_shield.py`'s 50-rule timing test asserted the desktop's 5 ms
  budget on GitHub's shared runners, where the 48-moving-zone path measured
  5.71 ms. On a runner its bound is now 15 ms, still well under the 31 ms of
  the per-fence loop it guards against; the 5 ms bound stays for the
  desktop. Its docstring claimed a margin of at least 10x; that holds for the
  static and window-edge samples (0.36 ms), not for the moving zones (2.6 ms
  on a quiet desktop, so about 2x). The same code at 10451bc measures the
  same, so this is no regression, but it is a risk for the slower Orin.

#### Tests
- New: `test_a_float_rounding_excess_over_a_cap_is_not_a_repair`
  (`tests/test_shield.py`), failing on the strict clamp. `tests/test_sweep.py`
  89/90 (1 visible skip) with the Linux runner's repair counts.

### 2026-10-12 — the 8 Oct follow's defects fixed in replay, the compiler and harness conform, gRPC over mTLS, the Orin defined for JetPack 6.2

Work of 9-12 Oct, in six packages and four integration packages, each
challenged by a reviewer who broke its tests on purpose, then fixed, and one
re-fly of the perception node with all of it. The Jetson Orin and the drone
are not available, so no hil or flight run exists.

#### Flown
- **The perception node re-flew the 8 Oct red-car follow, 120 s, with these changes**
  (`demo/out/pas_ap_20261012_033329`, Project AirSim + ArduPilot SITL over
  pymavlink, none of the grant's topologies, not KPI-grade): 1098 ticks at
  9.14 Hz, detector 4.26 Hz, 43 Shield interventions (yaw caps and building
  clearance), 0 escape ticks, 0 GeoFence breaches, ArduPilot real-time factor
  0.962, EKF p95 0.62 m. The FSM went Normal -> Brake 3 times and back each
  time; it never reached Loiter, and the altitude stayed at 7.99-8.15 m (on
  8 Oct, LOITER spells took it down to 0.36 m).
- What this run does not show: the car was tracked on 129 ticks (269 on
  8 Oct) and never came nearer than 16.5 m, so the 5 m stand-off ring was
  never approached. Its 0 escapes do not test the range-0 fix, and the BRAKE
  hold was never commanded in flight; both rest on the replay and the SITL
  probe above.

#### Fixed
- **The five P0 escapes of the 8 Oct follow** (`docs/FINDING-the-escape-that-stayed-inside-the-ring.md`).
  On every escape tick the perception node had placed the car under the
  aircraft (a depth return shorter than the camera's height became a range
  of 0), and at range 0 the stand-off check saw no action opening the range.
  The check now reads horizontal speed as the opening speed at range 0,
  `StandoffRecover` leaves along the commanded heading, and the node refuses
  depth returns the camera geometry rules out. A replay of the 1101 logged
  ticks gives 0 escapes (the old Shield reproduces the 5).
- **Over-escalation**: `StandoffHold.magnitude_m` is the forecast's closest
  approach inside the ring; on the replayed flight Brake -> Loiter falls from
  8 to 2. X6 leaves a recovery alone while the illegal depth is shown to
  fall (least-squares slope), a new edge X7 sends a recovery that loses
  ground to RTL, and N-in-T counts at most one onset per second.
- **The FSM's Loiter is flown as ArduCopter BRAKE** on every rail
  (`--hold-mode LOITER` keeps the literal mode): with no RC input, LOITER fell
  from 15 m to the ground in 6.7 s in SITL while BRAKE held 14.97-15.01 m.
  Mode requests are recorded in the FSM's and in ArduCopter's words, so a
  pilot's takeover after a hold is no longer read as the node's own.
- `demo/run_demo.py` died on its first shielded tick (it assigned the
  read-only `AuditLogger.policy_hash` since 26 Aug), and `--dynamic`
  hot-applied a fixed polygon fence, refused since 7 Oct; it now hot-applies
  the zone as a `dynamic_nfz`.
- The ArduPilot GeoFence export no longer uploads a zone that a clock can
  switch off (a `valid_time` window or a `time_window_switch`).
- `EpisodeRunner` no longer hangs after a tick raises.
- `scripts/run_pas_ardupilot.ps1 -Step check` reported mavlink-router as
  missing and the MAVProxy fallback as in use, although the router built into
  `~/.local/bin` is what the rail starts; it now looks there too.

#### Added
- **The three hot-apply rule types compile** (templates for `dynamic_nfz`,
  `time_window_switch`, `corridor_swap`, `circle_fence`); a policy file with
  a switch or a swap flies. A file declaring a `dynamic_nfz` is still held
  for flight (`models.FLIGHT_HELD_TYPES`): the rails count it now, but offline
  analysis and the BC pilots' features read polygon fences only.
- **Cost-map adapter** (`compiler.rasterise_cost_map`,
  `docs/DESIGN-cost-map-adapter.md`) with the grant's conventions; 0 missed
  under corridor_outside = 50 on 24 shipped policies; no planner reads it.
  Replay bundles carry each generation's map; `--cost-map` on both SITL rails
  and `demo/real_vla_demo.py`.
- **Both hashes in every bundle** (ours and the PI reference's); a forged
  reference entry is refused 29/29; no stored hash moved.
- **The stress harness's gRPC control plane over mutual TLS**
  (`guardrail/harness_service.py`, `guardrail/protos/harness.proto`,
  `tools/harness_client.py`, `docs/DESIGN-harness-grpc.md`): StartEpisode,
  HotApply, EmitEvent, GetStatus, EndEpisode on the headless harness, with a
  lab root CA meant to be shared with SROS 2 (not yet enabled).
- **One zone-dwell count on every rail** (`guardrail.replay.zone_dwell`).
- **The Orin deployment made concrete for JetPack 6.2** (Jetson Linux
  36.4.3): ROS 2 Jazzy in an Ubuntu 24.04 container so rclpy runs on Python
  3.12 (the grant asks 3.11+), the VLA image on an r36.4 base,
  bitsandbytes built for sm_87, and `deploy/orin/first_boot_check.sh` for the
  person setting the Orin up. Nothing has been built or run on an Orin.
- `tests/test_tracker_content.py` checks the local tracker's content when it
  is present.
- The perception node logs, per tick, the depth range inside the box and the
  rule that refused it (`subject_depth`, `depth_check`).

#### Found
- On the re-fly, 43 depth returns were refused and the car ranged by its box
  width instead, at 56-156 m. On the 31 of them with a depth-ranged tick
  within 5 ticks, the two ranges disagree by up to 122 m (the width range
  farther on 22, nearer on 9). The log kept neither the refused depth nor the
  rule that refused it, so whether the refusals were right cannot be told
  from this run; the next flight logs both. The depth image is also
  quantised to 1 m (the simulator does not honour the float encoding asked
  for).

#### Changed
- The public progress site is re-curated to 12 Oct (Gate G1 with its numbers
  and frame, and this entry's progress); README and `docs/index.md` scope the P0-escape figure to the 41
  mid-evaluation flights and state the 8 Oct follow's 5 escapes.
- `docs/data/scenario_sweep.json` is republished on today's FSM: 152 pass,
  20 tracked, 23 known failures, 0 fail; P0 escape 0.0 over 194 Shield-on
  episodes; fail-safe trigger correctness 0.823 (33 false, 0 missed), below
  the never-trigger null 0.968 and the grant's 0.99.
- `policies/policy_dsl.schema.json` regenerated: only the hot-apply types'
  descriptions changed.

#### Retracted
- The escape repair did not leave the ring in one tick (8 Oct entry): on all
  five ticks the node had placed the subject at the aircraft's own position
  (range 0), so no action could open the range.
- Brake to Loiter seven times (8 Oct entry): the log has eight Brake -> Loiter
  edges, five on stand-off repairs and three where no operator converged.
- Dev topology on one desktop (8 Oct entry), for the follow run: it commanded
  ArduPilot over pymavlink, which is none of the grant's topologies.
- The published library sweep's fail-safe correctness of 0.747 was measured
  on the 10451bc FSM; republished on today's FSM it is 0.823.

#### Tests
- New: `tests/test_flight_pas_ap_20261008.py` (replays the 8 Oct log on both
  theta readings; SKIP without the log), `tests/test_harness_service.py`,
  `tests/test_tracker_content.py`, and cases in `test_shield`, `test_fsm`,
  `test_csp`, `test_sweep`, `test_sitl_rails`, `test_replay`,
  `test_pas_ardupilot` and others. Each package's new tests were shown
  failing on the code before its fix, and the reviewers' deliberate defects
  were caught.
- Full suite on Python 3.10: 2291 of 2296 over 74 test files. On 3.11 the
  guardrail package's tests pass (2254 of 2294 overall: demo tests that need
  cv2, Pillow or projectairsim cannot run there, and five skips are visible).
  On 3.10 the five not passing are four visible skips (FastAPI is not
  installed in that environment; one sweep test) and the ITRI deck-pack
  check, which passes once the deck is rebuilt from a clean tree.

### 2026-10-08 — Gate G1 passes: ArduPilot flies Project AirSim, Mission Planner sees it

The first runs of the perception rail the PI asked about on 6 Oct: ArduPilot
SITL as the flight controller on Project AirSim's physics and sensors,
through the simulator's native `ardupilot-api` controller, with
mavlink-router fanning MAVLink out to Mission Planner and the perception
node. Dev topology on one desktop; not KPI-grade.

#### Flown
- **Gate G1 passed** (`demo/out/g1_20261008_223259`, the four conditions of
  the reference fallback-gates page, three runs, same seed): ArduPilot's EKF
  tracked the simulator's position with a p95 error of 0.81-0.82 m (limit
  1.0 m; a frozen-position null gives 18.9-19.0 m); its PWM lifted the
  vehicle 14.5 m from rest; each AUTO square mission (10 m sides, 15 m up)
  reached all 5 mission items with no airborne collision and landed within
  0.2 m of take-off; ArduPilot ran at 0.958-0.967 of real time. Frame:
  `docs/img/g1_pas_ardupilot_airborne.png`.
- **Perception node, red-car follow, 120 s** (`demo/out/pas_ap_20261008_224440`):
  camera, detector, Shield and escalation FSM commanding ArduPilot. 1101
  ticks at 9.16 Hz, detector 3.75 Hz, 85 Shield interventions, 0 GeoFence
  breaches, ArduPilot real-time factor 0.951. **5 ticks let a P0 stand-off
  violation through** (escape rate 0.0053): each time the aircraft was
  already inside the stand-off ring, stopping was illegal, and the escape
  repair did not leave the ring in one tick. The FSM went Brake to Loiter
  seven times because stand-off repairs of 3.6-9.8 m exceed theta (2.0 m).
  Both are open defects.
- **Mission Planner's port carried the flight**: the router's UDP stream to
  Windows port 14550 delivered GLOBAL_POSITION_INT and ATTITUDE at 19.0 Hz
  for the whole 120 s mission, largest gap 0.1 s (`sitl/gcs_listen.py`).

#### Fixed (each found on the rail on 8 Oct)
- A Windows client could not reach the router: mavlink-routerd listens on
  `[::]`, and WSL's NAT localhost forwarding does not carry an IPv6
  listener to Windows' 127.0.0.1 (measured: refused on 127.0.0.1, accepted
  on the WSL address). Under NAT the node and G1 now connect to WSL's own
  address (`NetworkPlan.router_tcp_url`).
- After a G1 run's ArduPilot stopped, the next scene load never returned:
  the steppable clock waits for ArduPilot's PWM. The simulator is now
  restarted for each G1 run.
- ArduPilot refused to arm ("Main loop slow (222Hz < 300Hz)"): CityLife
  delivers about 222 sensor frames a second against the 300 Hz loop
  airsim-quadX.parm sets, and 3 ms steps at that rate ran the clock at about
  0.67 of real time. The scene steps 5 ms and the param file sets
  SCHED_LOOP_RATE 200.
- The G1 square at 10 m flew through a street light (48-60 collisions a
  run); the passing runs fly it at 15 m.
- `start_ardupilot.sh` and `sitl/start_router.sh` now find a mavlink-routerd
  installed without sudo in `~/.local/bin`; the runner no longer exits 1
  when nothing is left to stop.

#### Changed
- The ITRI deck's Project AirSim slide states the G1 result from the newest
  verdict file (`tools/deck/build_progress_oct_data.py`, section `g1`).
- Installed for these runs: pymavlink and grpcio in the flight environment,
  cryptography, jsonschema and psutil in the 3.11 environment, jinja2 in the
  WSL ArduPilot venv, and mavlink-routerd built from source (2362c62) into
  `~/.local`.

#### Tests
- `tests/test_pas_ardupilot.py` 97/97 (the router-address test fails on the
  old code); `tests/test_build_progress_oct_data.py` gains a G1 case.
- Full suite on Python 3.10: 2075 of 2081 over 70 test files. The six not
  passing are four visible skips (FastAPI is absent in that environment; one
  sweep test) and the two deck-pack checks, which pass once the deck data and
  the pack are rebuilt from this commit.

### 2026-10-07 — the grant's own policy form, a Shield that escalates, ArduPilot on every rail

Nine packages following the PI's answers of 6 Oct: the KPI campaign moves to
the Jetson Orin (the grant's hil topology), the perception rail follows best
practice on Project AirSim, and deviations are closed by conforming to the
grant. Each package was challenged by an adversarial reviewer and fixed.
Short SITL runs were flown in WSL (dev topology, not KPI-grade); Project
AirSim, Unreal and any GPU model were not started.

#### Added
- **The grant's policy form** (`guardrail/models.py`, `guardrail/bundle.py`):
  the grant's demo bundle and the Policy DSL page's worked example load as
  written (geometry blocks, grant field names, scope, layer, altitude_ref,
  issued_at, lat/lon without origin). All nine grant rule types are
  declarable; circle fences are enforced. Layered policies
  (regulation/site/mission) merge and refuse any override that loosens a
  lower layer's hard rule. Bundles cross-load with the PI's reference loader
  in both directions (3/3). `tools/geo_to_policy.py` converts GeoJSON/KML and
  ingests REST payloads.
- **Escalation in the Shield** (`guardrail/shield.py`): every `filter()` tick
  feeds the escalation FSM (Brake, Loiter, RTL, Land); repairs report their
  size, so theta (2.0 m / 0.5 m) is applied; breach actions, hard/soft and
  priority now change behaviour.
- **Mid-flight events**: moving no-fly zones (spawn, move, translate, rotate,
  scale, expire), time-window switches and corridor swaps, each bumping the
  generation, checked against the layer rule, and exposed over REST
  (`guardrail/api.py`). Hot-applying a fixed polygon fence after take-off is
  refused, as the grant locks that class.
- **One rule checker**: `Shield.rule_status()`; the policy HUD and FenceGuard
  read it and their own geometry is deleted.
- **The grant's ROS 2 interface**: body-frame actions end to end, a separate
  `sitl/mavlink_adapter_node.py` that converts to local NED after projection
  and publishes `mavros_msgs/PositionTarget`, ROS 2 packages with launch files
  (`sitl/ros2_ws/`), a MAVLink router config for Mission Planner (UDP 14550)
  beside MAVROS, the ArduPilot GeoFence as backstop (`sitl/fence/`), and per
  episode a fresh audit file, a CSP per policy generation, an events log and
  a rosbag2 recording.
- **Project AirSim flown by ArduPilot** (`demo/pas_ardupilot/`,
  `scripts/run_pas_ardupilot.ps1`, `docs/DESIGN-projectairsim-ardupilot.md`):
  Project AirSim's native ArduPilot controller (`ardupilot-api`, ArduPilot
  SITL with `-f airsim-copter`), a one-process perception node (camera,
  detector, Shield, FSM, ArduPilot), Mission Planner through the router, and
  a script that records the four Gate G1 checks. Built and tested offline;
  not yet flown.
- **Jetson Orin portability** (`deploy/`, `docs/RUNBOOK-orin-hil.md`,
  `docs/DESIGN-topologies.md`): one image definition for dev, hil and flight
  where only a topology file differs; `tools/profile_shield_tick.py` (the
  grant's tick budgets on real flight ticks, runnable on the Orin unchanged)
  and `tools/vla_backend_table.py` (one protocol to compare VLA backends).
  No image has been built yet.
- **Stress harness**: every grant event type runs mid-flight; RTL and Land
  outcomes occur; paraphrase arms and a prefix on/off key; one YAML per
  template under `experiments/templates/` (16 files); a broad profile
  (15,025 episodes, defined, not run); a per-template KPI report.
- `tools/check_claims.py`: fails when a withdrawn claim is stated again;
  `tools/prefix_eval.py` (WP2-09 offline replay, not yet run on the model).
- A public progress site, preview only (`docs/progress/`,
  `tools/build_public_site.py`): 21 pages in English and Traditional Chinese,
  built from an explicit allow-list of fields. Not published.
- Finding notes for the silent defects found on 6-7 Oct (`docs/FINDING-*.md`).

#### Changed
- WGS84 is the canonical stored form for geographic policies; 28 of 29 policy
  hashes are unchanged and the moved one keeps its old hash verifiable.
- Policy validation is strict: the negative corpus goes from 14/25 to 35/35
  refused (10 cases added).
- The Shield's default lookahead is the grant's 5 s at 0.1 s, with checks on
  both sides of every time-window edge. At the grant's load (50 rules, 50
  poses) a check near a fence takes 0.39 ms and `filter()` 2.5 ms on the
  development desktop.
- The topology label accepts `hil` and `flight` only on evidence (an Orin
  host, the VLA host, the router layout), and records the ArduPilot version.

#### Fixed
- A window ending 17:30 stayed in force until 17:30:59.
- A mission-layer event could switch off a hard regulation-layer zone.
- The HUD showed rules the Shield did not correct as "corrected".
- A SITL tick the autopilot held was scored as a converged repair.
- A shield-off tick counted as a P0 escape even when nothing was flown.
- `parse_command("altitude: (40, 40)")` read the target as the altitude.
- `experiments/bench_shield_50rules.py` compares the fence code with the
  escalation FSM off on both sides, and times the Shield with it on.

#### Retracted
- The `guardrail/bundle.py` docstring said the two implementations could read
  each other's bundles. Until 7 Oct only the tar.gz container matched.
- The sweep's fail-safe figure (triggered = any brake). Measured with the FSM
  it is 0.747 (library), 0.726 (nightly) and 0.875 (smoke), below the grant's
  0.99 and below the never-trigger null.
- The sweep rollup published the P0-acted tick share (1.0) under the fail-safe
  KPI's name; it is the escape rate restated.
- The fine-tune result "100 % reached, efficiency 0.996" was stated without
  its goal assist: the run flew with goal_blend 0.55, under which the
  original adapter also reached 5/5 (0.942); without the assist the
  fine-tune reached 0/5 and 1/5.
- `docs/data/deck_sept_extra.json` carried stored detector rates (3.76, 5.15,
  4.03 Hz); the mission rates are 2.77, 4.06 and 3.11 Hz.
- The September deck's detector rate sentence used the stored 5.15 Hz; the
  mission rate is 4.06 Hz.
- The August midterm report's "the detector half of the threshold is met on
  every scenario" used stored rates; the flights on disk read 3.02-3.11 Hz.
- "Detections arrive at roughly 4 Hz while the control loop runs at 10 Hz":
  the detector ran 3.02-3.11 Hz and the loop 7.5-8.4 Hz.

#### Tests
- New test files: `test_policy_dsl_grant_form`, `test_geo_to_policy`,
  `test_shield_events`, `test_api`, `test_mavlink_adapter`,
  `test_ros2_package`, `test_pas_ardupilot`, `test_deploy_configs`,
  `test_profile_shield_tick`, `test_vla_backend_table`, `test_check_claims`,
  `test_prefix_eval`, `test_build_public_site`, `test_build_progress_oct_data`
  and others.
- Full suite on Python 3.10: 2073 of 2078 over 70 test files. The five not
  passing are four visible skips (FastAPI is not installed in that
  environment; one sweep test) and the ITRI deck-pack check, which passes once
  the deck is rebuilt from a clean tree. On 3.11 the guardrail package's tests
  pass, with visible skips only where an optional dependency is absent
  (cryptography, jsonschema, tokenizers, psutil); demo tests that need cv2,
  Pillow or projectairsim cannot run there (2032 of 2076 overall).

### 2026-10-06 — ten work packages toward the grant's locked spec

Ten packages taken from the re-verified audit, built in parallel on disjoint
files, then each challenged by an adversarial reviewer (tests reverted or
mutated on purpose) and fixed. Nothing here was flown; no simulator, SITL or
GPU model was started.

#### Added
- **Shield speed: a compiled fence index** (`guardrail/ir.py`). Each fence's
  margin ring is built once and a shapely STRtree answers which fences a
  forecast can reach. At the grant's 50-rule load (48 fences), on this desktop:
  `_check` near a fence 14-33 ms -> 0.13-0.26 ms median; `filter()` near a
  fence 172 ms median (p99 255 ms) -> 0.55-1.0 ms (p99 1.0-1.8 ms), against the
  grant's 5 ms check and 100 ms tick. `Shield.history` keeps the last 50 ticks
  (Safety Shield, sliding-window buffer). `experiments/bench_shield_50rules.py`
  reproduces the numbers. Not measured on a Jetson Orin.
- **Typed Constraint Summary Pack** (`guardrail/csp.py`, `guardrail/templates/`,
  `guardrail/csp.schema.json`). The 14 locked fields; time and region
  filtering; risk grading 0.5/0.3/0.2; P0 always kept, P1/P2 cut by risk inside
  a configurable 256-token budget; `CSPBudgetExceeded` when P0 alone is over;
  per-rule reasons; Jinja2 sentence templates; the action-set adapter; coarse
  and fine geometry with `geometry_ref`. `ConstraintCompiler.compile_csp()`
  returns it; `build_prompt()` keeps all its keys.
- **The CSP can reach a VLA**: `demo/real_vla_demo.py --csp on|off` (default
  off) puts the CSP before OpenVLA's instruction, logs the exact prompt, its
  token count, CSP hash and policy hash per inference, and refuses a prompt
  over budget before the model loads. 54-106 OpenVLA tokens over 29/29
  policies. Not yet flown.
- **Paraphraser** (`guardrail/paraphraser.py`, `experiments/paraphrases/`):
  112 free-form paraphrases (14 instruction sets x 8), generated once by
  claude-opus-5-5 with the prompt recorded, a seeded template fallback, and a
  validator that refuses a paraphrase changing a mission slot. Measured false
  accepts: 0/28 on the review set, 5/30 on a held-out set it was never tuned
  on.
- **Escalation state machine** (`guardrail/fsm.py`): Normal -> Brake -> Loiter
  -> RTL -> Land with the grant's defaults (N=3 violations in T=5 s, theta
  2.0 m lateral / 0.5 m vertical, T_recover 2 s), the mapping from
  violation_action, hard/soft and priority, and the fail-safe trigger
  correctness scorer with its always-trigger and never-trigger nulls. Pure
  module; not yet wired into the Shield.
- **Stress harness groundwork** (`guardrail/scenario_spec.py`,
  `experiments/profiles/`): ScenarioSpec and ScenarioEvent copied field for
  field from the Stress Testing page, expected-outcome and expected-fail-safe
  labels, seeds, a smoke profile (52 scenarios x 1 seed) and a nightly profile
  (211 x 3 seeds), stressors for all five Shield smoke-matrix rows plus wind,
  gusts, start jitter and GPS dropout, and a six-field manifest per episode.
  Smoke: 52 episodes in 15.9 s; nightly: 633 episodes in 223 s; both headless
  and never KPI-grade. `.github/workflows/tests.yml` (never run on GitHub).
- **Policy identity** (`guardrail/models.py`, `guardrail/bundle.py`):
  Ed25519-signed bundles (lab development key; private half gitignored),
  pure-Python verification so the 3.11 and SITL environments verify without
  `cryptography`, `issued_at` in the manifest, `policies/policy.lock.json`
  pinning one hash per `policy_id@version`, `--bundle` on
  `sitl/ros2_shield_node.py`, `sitl/run_sitl_demo.py` and `demo/follow_vlm.py`,
  `tools/wp1_roundtrip_kpi.py` with 25 negative policies, and
  `policies/policy_dsl.schema.json`.
- **KPIs from logs on disk** (`guardrail/kpi.py`, `tools/kpi_report.py`):
  repair success with the theta cap, mission success that requires the goal
  and no P0 unsafe position, repair count per episode and family, false
  triggers, and a rollup that never prints a zero for "not measurable".
  `docs/data/kpi_rollup_2026-10-06.md` covers 742 episodes, 0 KPI-grade.
- `pyproject.toml` for the guardrail package (requires Python >= 3.11);
  `sitl/setup_sitl.sh` pins ArduPilot to Copter-4.5.7 (2a3dc4b7) and gains a
  read-only `--verify`. `docs/DESIGN-python-versions.md` records what runs
  where.
- Design notes: `docs/DESIGN-{escalation-fsm,paraphraser,policy-identity,
  prefix-compiler,python-versions}.md`.

#### Changed
- `Policy.policy_hash` is the full 64-hex SHA-256 over canonical JSON with
  None omitted (`sha256-canonical-v2`), so adding an optional field no longer
  moves every hash. Old 16-hex forms still verify: stored manifests traceable
  to a policy 34/76 -> 76/76.
- The topology label `canonical-hil` is now written as the grant's word,
  `dev`. `is_kpi_grade()` refuses `dev` runs without a written PI waiver, so
  stored KPI-grade runs go from 5 to 0 until the hil runs exist.
- Time-windowed rules now carry their window in the prompt ("in force
  Mon-Fri 07:30-17:30") and drop out of the CSP outside it; before, the
  model was told they were permanent.
- `load_bundle` refuses unsigned bundles unless `require_signature=False`.
- README, `docs/index.md`, `docs/scope-clarification.md`,
  `docs/prof-repo-study.md`, `docs/CHECKLIST-remaining-work.md` (now marked
  superseded by the 5 Oct audit) and the progress note no longer state the
  claims retracted below.

#### Fixed
- `demo/real_vla_demo.py` sent OpenVLA's forward/right deltas straight into
  North/East: facing East, "forward" flew North. They are rotated at the
  capture heading now, and the loop reads the heading from the pose.
- A stripped or forged bundle signature could read as "unsigned" or
  "unverifiable here" and fly with `--allow-unverified-bundle`; both are now
  refused as bad-signature.
- A Shield-off control arm's counterfactual repairs scored as a measured
  repair success of 0.0; they now read `shield off`.
- `setup_sitl.sh --verify` reported PIN OK for an edited file inside a
  submodule, and for a crashed `git status`.
- `--backend sitl` in the sweep passed an option the SITL scripts reject.

#### Retracted
- "canonical HIL" for the desktop SITL + MAVROS 2 rail: it is the grant's
  `dev` topology. KPI numbers count from `hil` (Jetson Orin) runs.
- "P0 escape 0.0 on all 41 shielded flights" as the contract KPI: the 41 are
  34 Project AirSim + 4 SITL/pymavlink + 3 SITL/MAVROS 2 flights, and none is
  a contract KPI figure.
- Fail-safe trigger correctness target "1.0": the grant says >= 99 %.
- The in-flight detector rate on the five flights the mid-evaluation quoted:
  3.76-5.22 Hz -> 2.77-4.31 Hz, recomputed over the mission.
- "11713/11713 repairs converged" as the grant's repair success KPI: it is the
  Shield's own re-check; theta could not be applied to the 7660
  theta-governed ticks because no log records a repair size in metres.

#### Tests
- New: `test_ir` 14, `test_csp` 65, `test_real_vla_prompt` 43,
  `test_paraphraser` 82, `test_fsm` 95, `test_scenario_spec` 29,
  `test_policy_hash` 24, `test_wp1_roundtrip_kpi` 6, `test_kpi_report` 31,
  `test_env_pins` 34. Extended: `test_shield` 39, `test_sweep` 63,
  `test_kpi_magnitudes` 65, `test_bundle` 51, `test_manifest` 39,
  `test_replay` 22.
- Full suite on Python 3.10: 1408 of 1408 over 53 test files. On 3.11 the
  guardrail package's tests pass; the demo tests that need `cv2` or
  `projectairsim` cannot run there.

### 2026-10-05 (later) — a tracker website, and an update log on it

Record: the tracker's own Updates page (`tracker/data/updates.json`, entry #15).

#### Added
- `tracker/`: a local website that explains the project against the grant and tracks it. It is served by `tracker/serve.py` (Python standard library, 127.0.0.1:8765, HTTP Range so videos seek) and started with `tracker/start_tracker.bat`. It has seven pages in English, Indonesian and Traditional Chinese:
  - an overview;
  - clickable diagrams of the pipeline as built and as the grant specifies it;
  - the grant's work packages, KPIs, topologies and timeline;
  - a kanban board of the 148 audit items;
  - examples with what they do and do not show;
  - a glossary;
  - an Updates page.
- Board state (column, notes, history) is written to `tracker/data/state.json`. The file is gitignored as personal.
- Updates page: a versioned log, `tracker/data/updates.json`, maintained with `tools/add_update.py` (`new`, `add`, `check`, `list`, `extract`).
  - `seq` numbers the site's updates.
  - `version` copies a CHANGELOG release, or `<release>+<date>` for unreleased work, so no project release is invented.
  - It holds 15 entries: 12 backfilled from this file's sections, one from git for the 14-16 Sept work this file does not cover, and the two 2026-10-05 entries.
- Board activity: card moves and note edits with their times, from the board state.

#### Fixed (found while designing, each shown failing before the fix)
- `/data//_batches.json` and `/data/./_batches.json` served the site's working files. The hidden-file check ran on the raw path, and the static handler normalises it afterwards; the check now runs on the normalised path.
- A non-numeric `Content-Length` on a board save dropped the connection. It now gets a 400.
- A failed board write (a file held by OneDrive) left a temp file behind and dropped the connection. The temp file is now removed, the answer is a JSON 500, and the page keeps the board in the browser.

#### Fixed (found in the screenshots taken for this entry)
- Checklist: each column head was meant to stay at the top of the page while scrolling. The board scrolls sideways, so the head stuck 56 px down inside its own column instead and covered the first card. The board now scrolls in its own box (the window height minus 160 px, at least 360 px), and the heads stick to the top of that box.
- Updates timeline: releases a day apart (0.2.0 to 0.5.0, 7 to 10 Sept) printed their labels over each other. Each label now sits on the lowest free row, up to four rows.

#### Tests
- `tests/test_tracker_serve.py`: 15 cases, all pass.
- `tests/test_add_update.py`: 20 cases, all pass. One of them runs `check` on the real log.
- Full suite: 847 of 847 pass, over 43 test files. `python tools/add_update.py check` reports 0 problems.
- Checked in a browser against a second server on port 8766 with a scratch board file, so the real `tracker/data/state.json` was not touched:
  - the content API returns 148 cards and 15 updates, with no load error;
  - a board save reads back, a video answers a byte range with 206, and hidden working files answer 404;
  - moving a card and writing a note show on Board activity as 1 move and 1 note;
  - the Release filter shows 6 entries, and a link to one update (`#updates?v=12`) opens it;
  - all seven pages fit a 375 px phone screen, and requests go only to the local server.
- Screenshots in `docs/img/tracker/`: 27 desktop views (nine pages or states in three languages) and three phone views.

### 2026-10-05 — the grant audit, and claims corrected against it

Record in Indonesian: `docs/AUDIT-KONTRAK-2026-10-05.md`. Data: `docs/data/grant_audit_2026-10-05.json`.

#### Found
- Every obligation in the seven grant pages (`reference/*.pdf`) was audited, with the PI's reference design docs as the detailed spec: 136 items.
  - Result: 6 done, 56 partial, 39 missing, 35 drift (built differently from the grant, or a claim about it does not match).
  - Method: five auditors (WP1 to WP4, and architecture), each challenged by a skeptical verifier, and a completeness critic who reread every page.
- Final-gate items with no artefact:
  - perception-rail integration;
  - the KPI report from stress testing in the hil topology;
  - the Paraphraser (a Q3 deliverable);
  - the stress-test harness;
  - the signed final report.
- Prefix Compiler: the CSP is built but never put in any VLA's prompt, and its two KPIs were never measured.
- WP3 Shield:
  - There is no escalation beyond Brake (no Loiter, RTL or Land) and no ArduPilot GeoFence backstop.
  - `fail-safe trigger correctness` in `guardrail/kpi.py` restates the escape KPI.
  - `mean time to safe` has no Shield-on value.
  - The stored KPI numbers (runs of 28 to 31 Aug) were not re-measured after the Shield changes of 1 and 29 Sept.
- Deviations from the locked spec:
  - a 3 s lookahead against the grant's 5 s;
  - world-frame actions against the locked body frame;
  - `violation_action` limited to repair and brake;
  - no spatial index (14 to 36 ms per check with 50 polygon rules, against a 5 ms budget).
- `docs/CHECKLIST-remaining-work.md` (last updated 8 Sept) is out of date: it lists two open items.

#### Retracted
- "The P0-escape count on screen is the contract KPI" (`docs/GAMBARAN-SISTEM.md`, 3 Oct). The HUD uses the KPI's definition. But a Project AirSim flight is not a contractual KPI number: the grant takes KPIs only from stress-test runs in the hil topology.
- "Most compliance is done by a pilot that knows the rules, and the Shield is the backstop, exactly as the contract designs it" (`docs/GAMBARAN-SISTEM.md` and the `demo/policy_hud.py` docstring). In the grant, upstream compliance comes from the Prefix Compiler's CSP in the VLA prompt, and the backstop behind the Shield is ArduPilot's GeoFence. FenceGuard is our addition.
- "Gazebo: no artefacts in the repository" (`docs/GAMBARAN-SISTEM.md`). The scripts exist (`sitl/setup_gazebo.sh`, `sitl/run_gazebo_demo.sh`); no run was kept.
- "The city demos may use a hand-written pilot because the contract makes the VLA backend swappable" (`docs/GAMBARAN-SISTEM.md`). The clause covers VLA backends that output the 4-D action, and it names stubs only for unit tests. The text now says the demos test the Shield, not a constrained VLA.

#### Changed
- `docs/GAMBARAN-SISTEM.md` (and its PDF) corrected as above. The `demo/policy_hud.py` docstring was reworded.

### 2026-10-03 — the policy on screen, a no-fly zone in the city, and the share pack

Record in Indonesian: `docs/WORKLOG.md` (the 2026-10-03 entry). Asked for at
the 2026-09-30 meeting: show on screen when a guardrail rule acts, and send the
material for ITRI's monthly report.

#### Flown
Red car (`Car_10`), 240 s, `--identity`:

| | policy | cruise | within 30 m | estimate on the car | zone entered | min clearance | detector (mission) |
|---|---|---|---|---|---|---|---|
| `citylife_redcar_nfz1` | NFZ | 8 m | 0.977 | 0.998 | no | 3.24 m | 2.20 Hz |
| `citylife_redcar_alt10` | default | 10 m | 0.987 | 0.997 | - | 3.21 m | 2.43 Hz |
| `citylife_redcar_alt10_nohud` | default, old HUD | 10 m | 0.994 | 1.000 | - | - | 2.89 Hz |
| `citylife_redcar_nfz2` | NFZ, cached overlay | 10 m | **0.985** | **1.000** | **no** | 3.25 m | 2.79 Hz |

- P0 was 0 on every flight, the Shield made 0 corrections, and there were no
  mission collisions.
- Beside the zone the drone flew x 37.9-41.4. `clear_aim` re-aimed it on 91
  ticks, and it was 21.2 m behind the car at the first corner (24.1 m on
  nfz1, against 42.7 m in the no-re-aim replay).
- At 10 m cruise the altitude stayed 10.0-10.6 m against the 14 m ceiling.
- Drawing the overlay on every 20 Hz frame cost the detector (2.89 -> 2.43
  Hz, flown A/B). `OverlayCache` draws it at 5 Hz and pastes the tiles in
  between, which brings it back to 2.79 Hz.
- All of today's flights ran the detector slower than the 30 Sept flights,
  including the old-HUD one (2.89 vs 3.26 Hz in the mission; start-gate rates
  are equal). OBS Studio and Mission Planner were running today and were not
  on 30 Sept. This is suspected, not tested.

#### Added
- **`demo/policy_hud.py`: the policy indicator.**
  - One row per rule in the loaded policy, with its limit, the value the Shield
    itself measures (the estimator's subject, its obstacle distance field) and
    a status: idle / OK / NEAR / AVOID (the controller steering round it) /
    HOLD / ACTING (the Shield repaired the command) / BREACH (past the limit,
    or a P0 still violated by the command actually sent) / BRAKE.
  - A banner for the most important event of the last 1.5 s, ranked by event.
  - A north-up map with the zones, the aircraft's track and the target
    estimate.
  - Running counts. The P0-escape count uses `guardrail/kpi.py`'s definition.
  - On by default in `demo/follow_vlm.py`; `--no-policy-hud` restores the old
    line. It stays live for the Shield-filtered flight to the landing site and
    says plainly that the vertical descent is outside the Shield.
  - Its totals are written to `metrics.json` as `policy_hud`.
- **`policies/follow_car_citylife_nfz.yaml`**: a part-width no-fly zone on the
  red car's first leg (x 45-62, y 62-92). `run_citylife_follow.ps1
  -PolicyFile` selects it.
- **`FenceGuard.clear_aim()`.** A forward aim point that falls inside a zone's
  band (polygon + margin + stand-off) is moved to the band's edge plus 1 m, so
  a zone beside the subject's path is passed alongside. Without it, a kinematic
  replay with the real gate/slide/Shield left the drone 42.7 m behind the car
  at the next corner (16.1 m with no zone); with it, 18.7 m, with the zone never
  entered. `fence_aim_ticks` in metrics.
- `FenceGuard.last_cause` ("fence" / "obstacle"), logged per tick as
  `fence_cause`.
- `Shield.subject`, `Shield.subject_class`, `Shield.clearance_at()`:
  read-only views for the indicator.
- `frame_t` in `recorder.json`: the flight time each frame showed.
- `tools/rerender_policy_hud.py` redraws the indicator onto a recorded flight
  from its own log. It warns when the obstacle map is newer than the flight.
- `tools/deck/build_flowcharts.py` draws the tracking and whole-system
  flowcharts as SVG and PNG (`docs/img/flowchart_*`).
- `docs/GAMBARAN-SISTEM.md`: system overview in Indonesian.
- `demo/identity.TIER_LABEL`: the identity tiers are shown as REJECT /
  DOUBTFUL / OK. The stored values stay hard / soft / ok.

#### Changed
- The identity tiers are relabelled everywhere a reader sees them: the deck
  builder, the progress note, the HUD box label and the flowcharts. At the
  meeting, "HARD / SOFT" was heard as manoeuvre types, and the grant's Policy
  DSL already uses hard/soft for rule types.
- `nfz_hold_ticks` counts only holds a fence caused. With a fence declared
  anywhere, building holds used to count too.
- The legacy HUD names the hazard from `last_cause` rather than from whether
  the policy has a fence.

#### Fixed (found by a three-lens review with verification, before any flight)
- The banner named the first repair operator, so a P0 escape could read as
  "backing off from target". It now names the last operator acting for the
  violated rules' kind, and an escape outranks everything.
- A controller hold masked Shield repairs, on both the row and the banner.
- No state for "past the limit": rows showed NEAR, and the banner said "all
  rules satisfied".
- Rules the Shield repaired for, but that were not violated by the raw command,
  were not marked.
- Off the obstacle map, clearance read as OK.
- The stand-off binding test differed from `SubjectStandoff.binds`.
- Zone distance ignored the margin and the altitude band.
- The minimap was shifted half a cell.
- The zone warning distance was hard-coded.
- A display exception could abort a flight.
- The HUD froze through the landing.
- `frame_t` flooded the console.
- `-PolicyFile` refused absolute paths.
- Re-renders reused stale frames.

#### Share pack
`docs/share/2026-09-30-ITRI/` (gitignored). It holds:
- the 30 Sept deck as shown (14 slides; tiers relabelled; flowcharts
  redrawn) and its PDF;
- the 16 Sept deck and its PDF;
- the 30 Sept follow video and the before/after clip;
- the 24 Sept reference flight, re-encoded from 166 MB to 22 MB with the same
  duration;
- the flowcharts;
- the progress note;
- a README with the cover note and six corrections.

#### Tests
`tests/test_policy_hud.py` 36/36 (with `OverlayCache`). Full suite 812/812.

### 2026-09-30 — a re-review, pedestrians that no longer get lost, and the red car flown

Record in Indonesian: `docs/WORKLOG.md` (the 2026-09-30 entries).

#### Flown
The red-car mission with `--identity`. The reference flight
(`citylife_redcar_trail`) ended TARGET LOCKED on a pedestrian signal.

| | within 30 m | estimate on the car | lapses / re-acquired |
|---|---|---|---|
| `citylife_redcar_id1` | **0.992** | **1.000** | 0 / 0 |
| `citylife_redcar_id2` | **0.980** | **1.000** | 0 / 0 |
| `citylife_redcar_id3` | 0.495 | **0.996** | 2 / 1 |
| `citylife_redcar_id4` (after both fixes) | **0.970** | **1.000** | 0 / 0 |
| before (7 trail/final flights) | 0.088-0.436 | 0.013-0.697 (as flown) | - |

"Estimate on the car" is the share of ticks with a served estimate that sit
within 6 m of the car. P0 was 0 on every flight, and no flight collided
during its mission. On `_id3` the estimate never moved to anything else. It
lost the car twice. The first time it re-acquired it in 1.35 s. The second
time it saw the car standing at a light 52 m away but never took it back,
because re-acquisition stops at 45 m. That is fixed below.

#### Retracted / corrected
- **The identity gate did not pass held out.** Re-run after the bottom-clip
  guard moved into `identity.features_for` (thresholds unchanged, hash
  `567a2c79dc53`): leave-one-flight-out, 15.2 % of true boxes are not OK,
  over the 15 % gate; in-sample 14.1 % meets it. The 09-29 entry is
  corrected in place.
- **The replay's "old" arm was not what flew.** It reset and re-seeded after
  3 s. Against the estimator as flown the replay reads 22.9 % -> 94.5 % on
  the car, and 210 of 241 wrong first accepts after a gap -> 0 of 12 wrong
  seeds. The 09-29 entry is corrected in place.
- **Pinhole rescore:** `_final1`, `_final3` and `_carpolicy` (a tie) fall
  below chance; `_trail3` and `_far` were below it already.
- **"Nobody standing on a zebra"** (09-29, Simulate 1) was measured between
  two samples of positions only; the per-read check (state and speed) found
  six figures on zebras in WALK - the pure-node bug below.

#### Fixed
- **Pedestrians crossed from the kerb in WALK, with no walk-phase or gap
  test.** A `bind` of a pure node is re-evaluated at every use in Blueprint;
  the point's kind was read after `SetIdx` and so was the NEXT point's.
  Latched into `NKind`; the DSL evaluator in `tests/test_drive_signals_dsl.py`
  now re-evaluates pure binds the same way, and a test runs the old graph
  into the bug.
- **Half the pedestrians froze after ~30 minutes.** Stuck 3 s short of a
  pavement node a figure SKIPPED to the next point; from off its line that
  point was often round a building corner, so it stalled, skipped, stalled,
  until a kerb node sent it to WAIT tens of metres from its zebra, then
  across, through a building, for ever (CROSS had no stuck rule). At
  t = 1933 s, 25 of 40 figures were off their routes while every zebra
  counter read clean. Now a stall within 300 cm of the point is arriving;
  anywhere else the figure keeps its target and steps aside, alternating
  sides; after six detours it is set down on the point. Pavement nodes are
  spread +-25 cm per figure so opposite streams no longer meet dead
  head-on. Simulate over 30 minutes: 0 figures off route at
  t = 1972 s, 0 set-downs.
- **Re-review of the drone side (10 of 11 findings confirmed):** without an
  estimator the lapse anchored where the subject was committed (Reacquirer
  then refused the car as out of reach for good), and SOFT boxes kept the
  lapse clock alive indefinitely; the pedestrian centre height never applied
  ("person" vs the canonical "pedestrian"); a retarget during an inference
  could re-seed the lock with the old subject; `_prior_at` gave a column to
  a point behind or under the aircraft; the Reacquirer's fixed 2 m motion bar
  let a sign with 1.5 m of map-point jitter through on 186 of 200 seeded
  runs (now 2 m + 3 standard errors: 0 of 200, a car still 200 of 200);
  `_pooled_reproduced` could count the `_pooled` row as a flight. Logs:
  `depth_dt_ms` of the depth actually paired, `t_capture` and `stamp` per
  detection.

- **A car in plain view beyond the re-acquisition range was never taken
  back** (`_id3`, 65 "too far" refusals of OK boxes within reach). A
  candidate refused for its range alone is now a far lead: after three
  consistent sightings the search flies toward it (HUD "APPROACHING") until
  the ordinary gate can judge it. The gate still decides.
  `reacquisition.far_approach_ticks` counts the ticks. Replayed on `_id3`'s
  own candidates after the second loss, it fires at t = 126 s, 4.3 m from
  the car, with the drone 51 m away. `_id4` never lost the car, so the
  steering has not flown yet.
- **Landing sites were chosen with no room for drift.** `_id3` reached the
  nearest cell that passed and touched down 0.67 m off it, 2.94 m from an
  obstacle that needs 3.0 m. A site to fly to now keeps every clearance
  1 m wider (`SITE_MARGIN_M`), and the bare rules are tried only when no such
  cell is in reach. Staying put needs the rules alone: the first version
  also asked it of the hover point and sent `citylife_ped_id` 25 m into a
  crowd, where the Shield held it.

- **Both replays swept in the new flights.** `tools/replay_identity.py`
  globbed every `citylife_redcar_*`, so the `_id1..4` flights, whose boxes
  the identity rules had chosen, entered the tuning set and the pipeline
  replay (28,796 -> 37,551 ticks). A flight flown with `--identity` is now
  left out. Both replays reproduce their published numbers exactly, and the
  far-lead change moves neither.

#### Changed
- **`run_citylife_follow.ps1` passes `--identity` only for the car
  mission** (`-Identity` forces it for a person). The identity rules were
  tuned on car flights only. For people they supersede the presence gate's
  0.2-1.5 m width test, and on `citylife_ped_id` person boxes ranged on the
  facades behind them (40-130 m) pulled the estimate within 6 m of anyone on
  only 46 % of ticks. A person-width SOFT rule was tried and withdrawn: it
  demoted 41 % of the boxes near people, with labels too loose to tune it
  on. `--land-site` stays on for both missions.

#### Known limitations
- **Range to a person comes from depth, and a thin box measures the
  background.** The presence gate blocks those boxes, 1,502 of 1,788 ticks
  on `citylife_ped_0930`. That flight was within 30 m 98.2 %, where the
  09-23 flight managed 100 %. Its pedestrians walk their routes now instead
  of pacing. The proposed fixes are the nearest surface in the box or the
  ground ray through the box bottom. Neither is done.
- **`run_follow_vlm.ps1 -Identity` (Demo_day) has no start gate.** Acquiring
  with no anchor means waiting to see the car move, which took 7.1 s. All of
  the 6.4 % outside 30 m fell in the catch-up after it (t = 8.1-12.5 s). The
  default Demo_day run, without `-Identity`, is unchanged.
- At the zebra where loop A turns north (y = -3000 on junction (4100, -4100))
  a figure can wait at the kerb for minutes (194 s longest, 8 of 40 figures
  over 100 s in 33 minutes). The likely cause (inferred, not measured):
  loop A's turning platoon uses the whole 8 s walk window, and the figures
  yield to cars. The fix is pedestrian priority
  on the car side, and it is not done.

#### Added
- `verify_peds.py`: `off_route` (a figure more than 5 m from its own leg),
  `detours`, `teleports`, and per-figure reasons for any figure on a zebra
  that is not crossing.
- `docs/video/citylife_redcar_identity.mp4`: `citylife_redcar_id1`, the
  first-person view with the HUD beside the chase camera (262 s, 1920x540
  H.264, 23.6 MB). The duration is checked against the master.
- Tests: 793 in the suite (41 files).

### 2026-09-29 (evening) — the lock that could not let go, lights that change, people who wait

Day-by-day record in Indonesian: `docs/WORKLOG.md`. The write-ups are
`docs/FINDING-the-lock-that-could-not-let-go.md` and the fifth part of
`docs/FINDING-crowd-pedestrians-and-traffic.md`.

#### Retracted
- **Every `det_hz` published before today is inflated.** metrics.json divided
  EVERY inference since the detector loaded - including minutes of start-gate
  waiting - by ticks x 0.1 s, which is shorter than the mission whenever the
  loop ran below 10 Hz. Recomputed from the flight logs (the inference seq
  numbers the ticks consumed, over the ticks' own span):
  `citylife_follow2` 7.60 -> **3.06**, `citylife_follow3` 4.63 -> 3.18,
  `citylife_city` 4.82 -> 2.96, `people_final` 4.04 -> 3.26,
  `fixed_kpi` 4.41 -> 3.53, `city_kpi` 5.15 -> 4.06, and on the red car
  6.88-7.50 -> 3.46-3.99. So:
  - "`det_hz` clears its 4.0 Hz gate in all four" (09-23, the four
    `citylife_follow*`/`_city` flights) is wrong: none of the three whose logs
    survive does (2.96-3.18); `citylife_follow`'s log is deleted.
  - "The start gate can fire while the level is still streaming (detector at
    3.8 Hz instead of 7)" (09-29, morning) is wrong. Every red-car flight ran
    its detector at 3.5-4.0 Hz during the mission; the "7" was the gate's
    inferences counted over mission time. There is no streaming effect.
  - `docs/FINDING-decorative-pedestrians.md` (4.04 "clears 4.0") and
    `docs/FINDING-five-defects-found-by-audit.md` (4.41) inherit the same
    error. `tools/build_eval_data.py` now recomputes the rate for old flights
    (`det_hz_mission`) and keeps the reported one as `det_hz_reported`.
- **The tracking score's projection was linear, the camera is a pinhole.**
  `track_truth.project_target_cx` mapped angle to pixel linearly (off by up to
  ~4 deg between centre and edge). Re-scored with the pinhole
  (`PROJECTION = "pinhole"`), the red-car flights' `frac_on_target` moves by
  less than 0.02, but the chance floor rises by 0.02-0.05, so the margin over
  chance shrinks everywhere. Three flights drop below it: `_final1` (0.356 vs
  0.373), `_final3` (0.274 vs 0.288) and `_carpolicy` (0.442 vs 0.443, a
  tie). `_trail3` (0.059 vs 0.094) and `_far` (0.083 vs 0.097) were below it
  already on the linear score. (A first draft of this entry named only
  `_final3` and `_trail3`, and `_trail3` as a new casualty.) The default
  stays linear so published numbers reproduce; the table is in the WORKLOG.

#### Fixed
- **The drone locked onto a red pedestrian signal 110 m from the car**
  (`citylife_redcar_trail`, t = 180 s). Three causes, three fixes:
  - `TargetLock` could not say "none of these": with nothing near its
    prediction it took the best-scoring box. `select_strict` (with
    `--identity`) answers None instead; only a start gate or a re-acquisition
    seeds it.
  - The ground check only rejected box bottoms above ~2.7 m; a kerbside signal
    at 2-3 m passed. Every candidate is now judged physically at its OWN
    frame's pose and depth (`demo/identity.py`: width, aspect, bottom height,
    distance from the street; HARD / SOFT / OK). Thresholds grid-searched on
    the 12 flights: in-sample 86.4 % of wrong boxes not OK, 14.1 % of true
    ones, 0.3 % of true ones HARD. Leave-one-flight-out, 86.5 % / **15.2 %** /
    0.6 %: the held-out true-box rate is just over the 15 % gate, which only
    the in-sample figure meets. (Drafts said 15.4 % and "passed"; the 15.4
    predates the bottom-clip guard moving into `identity.features_for`.)
  - After a loss the estimator's gate widened until a far box could re-seed
    it. Now the estimate lapses after 3 s and only `Reacquirer` restarts it:
    4 OK sightings in a row, <= 45 m, within reach of the last measured
    position, and - when nothing constrains where it is - seen MOVING (the
    replay found a red fire-hydrant sign by the launch point seeding it).
  Replayed open-loop on the 12 recorded red-car flights with the same boxes
  (`tools/replay_pipeline.py`, pooled integers): the estimate is on the car on
  **94.5 %** of the ticks it is served, against 22.9 % for the estimator as
  flown, and **0 of 12** seeds (7 starts, 5 re-acquisitions) are on something
  else, where the flown estimator's first accepted box after a gap was
  wrong after 210 of 241 gaps. It is served on 7.6 % of the ticks (flown:
  45.1 %), because the replay only has the box the old lock picked - a car
  the old lock passed over is not in the log. Earlier drafts compared
  against an "old" arm that reset and re-seeded, which the flown code never
  did (95.5 % vs 22.4 %, 0 of 18 vs 260 of 293).
- **Bearings used a linear pixel-to-angle map** (up to ~4 deg off; physical
  widths ~21 % small). One pinhole model, `demo/camera_model.py`, everywhere;
  `--linear-bearing` reproduces older flights, and an AST test keeps the
  linear formula out.
- **A detection met the tick's pose and the latest depth frame**, 0.2-0.4 s
  after its image was captured. `SemanticObs.get_front_capture` now pairs it
  with the pose interpolated at capture (sim stamps when both sides have
  them) and the nearest depth frame (none beyond 70 ms), and the estimator
  folds each measurement in at its capture time (posterior kept at the last
  update; `predict` no longer moves it).
- **"NFZ AHEAD - HOLDING" at a building.** The HUD says OBSTACLE unless a
  no-fly polygon caused the hold; TARGET LOCKED only on a fresh identity-OK
  box, else TRACKING (PREDICTED) / RE-ACQUIRING n/4 / PURSUING ....
- **Search after a loss rotated in place** (the reference flight, against a
  building corner). `--search-planner`: pursue the car's street to the next
  junction, watch each exit, hold over it (`demo/search.py`).
- **Landings in a hedge and on a parked car.** `--land-site` flies, through
  the Shield, to a pavement cell `demo/landing.py` calls landable first.
- **Cars waited with their front half on the zebra** (give-way target
  JEdge - 300 put the nose 1170 cm from the junction centre, inside the
  800-1400 band). They now stop at a line 1730 cm out, 100 cm behind it.
- **The runner started flights at 43.9 m.** `-StartMaxRange` 45 -> 30 and
  `-StartTimeout` 240 -> 330 (red lights lengthen the car's lap).

#### Added
- **Traffic signals that change and are obeyed.** `BP_SignalController`
  switches 258 of the 292 heads (27 vehicle, 231 pedestrian) by a fixed-time
  plan (`tools/citylife_signals.py`: green 20 / yellow 3 / all-red 2 s per
  axis, 50 s cycle, offsets spread by grid index); `BP_CityCar.UpdateEffSpeed`
  obeys the same plan (`tools/citylife_mcp/drive_signals.py`, a node-for-node
  port of `tools/citylife_traffic_model.py`, which held RedViol 0, ZebraWait 0,
  MinGap 650 cm over 30 simulated minutes at 3/10/30 fps). Simulate, ~7 min:
  RedViol 0, ZebraWait 0, MinGap 650 cm, longest stand-still 78 s. (That
  ZebraWait could not have fired; see 2026-09-30.)
- **Pedestrians with somewhere to go.** 40 figures walk closed tours over a
  pavement graph whose only road-crossing edges are the zebras
  (`tools/citylife_peds.py`), wait at the kerb for the walk phase and a 4 s
  gap, and cross without stopping (`tools/citylife_mcp/ped_walk.py`;
  `--roam` restores the old graph, saved in `ped_roam_v1.dsl`). Simulate:
  51 crossings started, kerb wait max 42 s, nobody standing on a zebra.
  (The zebra check was too loose, and two pedestrian bugs were behind it;
  see 2026-09-30.)
- Metrics: `identity`, `reacquisition`, `estimate_on_subject`
  (`track_truth.score_estimate`), `landing`, per-tick `tier`, `lock_why`,
  `reacq`, `plan`, `est_xy`; `detections.jsonl` logs every candidate with its
  features and tier.
- Tools: `tools/replay_identity.py`, `tools/replay_pipeline.py`,
  `tools/citylife_mcp/{inspect_signals,signals,drive_signals,ped_walk,
  verify_signals,verify_peds}.py`; survey `docs/data/citylife_signals.json`.

### 2026-09-29 — a map of this city, a trail to follow, 10 Hz, and four reviews

The write-up is the fourth part of
`docs/FINDING-crowd-pedestrians-and-traffic.md`.

#### Retracted
- **"The corner stall was the aircraft pressing into a tree and a signal
  pole"** was already corrected before the 09-23 commit; the phantom wall it
  describes is now fixed at both ends (below).
- **"9.3 Hz is the Windows 15.6 ms timer"** (09-23). Half of it: the pacing
  slept `TICK - work` from each tick's own start, so every timer round-up was
  lost. An absolute deadline alone reaches 10.0 Hz at 15.6 ms.
- **`start_heading_err_deg: null` in every metrics.json since it was added.**
  It was measured and then reset to None before any metric read it.

#### Fixed
- **The Shield's phantom wall.** Off the obstacle map `_distance_at`
  extrapolated a border building into a wall to infinity (-35.16 m at a
  CityLife junction entrance). The distance off the map now stays within
  [v - off, v + off] and a border building extends past the edge only as far as
  it reaches into the map. Against HEAD, 12 of 28,800 sampled monitor answers
  in `test_check_contract` change, all on forecasts that leave the map; the
  fixture now also pins four policies it never covered.
- **The control loop: 9.99-10.0 Hz** on every flight (was 9.31-9.37 on the red
  car, 8.38 on the August Demo_day reference): absolute-deadline pacing plus
  `timeBeginPeriod(1)`; `timer_resolution_ms` measures the wait the loop does.
- **The estimator was re-fed the Grounder's held box** - up to 8 s old - on
  every inference that found nothing, paired with the current pose. It now
  takes each detection once, fresh.
- **The ground-contact check rejected the car it protects** (34 boxes on the
  red car through a corner). A box on the estimated subject is exempt, while a
  ground-checked box fed the estimate in the last 3 s.
- **A car on a zebra drove on whoever stepped out.** It now stops where it is
  for a pedestrian in its own path. Final Simulate, 4.4 min: 14 emergency-stop
  ticks, none above 50 cm/s; walking figure within 4 m at speed 0; closest two
  cars 650 cm (the design value).
- The runner fails loudly: a failed flight no longer points at the previous
  run's metrics.json, a throw or Ctrl+C stops the simulator, the map check
  runs before any simulator starts, and the teardown kills only the `-game`
  process.

#### Added
- `demo/trail.py` + `--trail-follow` (on in the car runner): fly where the
  subject DROVE - breadcrumbs from the estimator, a carrot along them in
  track, the last sighting and the subject's last direction in coast and
  search, never nearer a stopped subject than the follow's stand-off and never
  nearer anything than the camera's blind spot plus 2 m.
- `demo/build_voxel_map.py`: any rectangle, several bands from one query, raw
  voxels saved (`--from-voxels`); `demo/out/citymap_citylife/` for CityLife
  (180 x 100 cells; the +-80 m cube rebuilt through it agrees on all 5,600
  overlapping cells). `build_street_mask.py --dir`.
- Evidence in metrics.json: `collisions` (the simulator's contact reports,
  split into before t0, mission and after it), `off_map_ticks` (null with no
  map), `trail`, `presence_block_reasons`, `ground_check_waiver`,
  `timer_resolution_ms`; per tick `gate_why`, `pitch_deg`, `roll_deg`, `trail`.
- `tests/test_trail.py` (18), `tests/test_voxel_map.py` (5), new cases in
  `test_clearance.py` and `test_range_and_lock.py` (100).
- `test_track_truth`'s median test knows a third losing shape: a crowd flight
  (`citylife_ped_final`, 12.0 px against 7.4) where the class-level null is the
  distance to the nearest of ~40 figures. Allowed only with per-figure truth
  and a one-figure score at least 0.05 over its own null (0.58 vs 0.48); it
  excuses no other flight. Suite: 553/553.

#### Flown
Seven red-car flights with the trail (240 s each), a pedestrian flight on the
CityLife map (180 s), and DEMO 1 on Demo_day with and without the trail.
`p0_violation_escape_rate` 0.0 and no collision during any mission.

| red car | within 30 m | longest < 30 m | estimate on the car |
|---|---|---|---|
| `_ground` (09-23, no trail) | 0.314 | 48.8 s / 146 m | 0.55 |
| `_trail` | **0.436** | **103.4 s / 305 m, two corners** | **0.74** |
| `_trail2` / `_trail3` / `_final1..4` | 0.088-0.282 | 12.8-59.0 s | 0.11-0.36 |

Each of `_trail2`, `_trail3` and `_final1` exposed a defect fixed after it (see
the FINDING). Pedestrian: within 30 m 1.00, instance on target 0.58 against a
null of 0.48. Demo_day DEMO 1: within 30 m 1.00 with and without the trail,
mean separation 16.9-17.3 m, loop 10.0 Hz (8.38 in August).

#### Found, not fixed
- **Which red thing.** Across the seven trail flights the estimator served
  the car on 36 % of its ticks (74 % on `_trail`); the mission in this level is
  limited by target identity, not by the corner.
- Loop B waits up to 42 s to give way at its junctions.
- The 10 m pedestrian ring binds only to the tracked subject (839 ticks with
  another pedestrian inside 10 m, by design).
- The start gate can fire while the level is still streaming (detector at
  3.8 Hz instead of 7).

### 2026-09-23 — the right frame width, the left side of the road, cars that yield, and the red car

The level's logic is reproduced by `tools/citylife_routes.py` and
`tools/citylife_mcp/` (it is gitignored); the write-up is the third part of
`docs/FINDING-crowd-pedestrians-and-traffic.md`.

#### Retracted
- **The 2026-09-22 tracking columns, and "`frac_on_target` no longer measures
  anything on this level".** `demo/track_truth.py` fell back to a 400 px frame
  on a 768 x 432 camera. Re-scored with the width recovered from
  `detections.jsonl`: `citylife_follow2` 0.255 on target against a null of 0.802
  (in-shot median 128.2 px vs 31.9), `citylife_follow3` 0.365 against 0.808
  (113.5 vs 23.3) - on those two the box was mostly NOT on a person, worse than
  chance, which the video had shown and the numbers had not. `citylife_city`
  1.000 against 1.000, median 9.2 px vs 10.1: at chance. "45.5 px against 5.2 px"
  was an artefact of the width. `citylife_follow` cannot be re-scored (its log
  is deleted), and none of these flights can be instance-scored (their logs
  lack `truth.names`).
- **"Lanes sit 350 cm left of each centre line"** (2026-09-22). They sat on the
  RIGHT of travel - eastbound at X = 3750, the south half, with X = north and
  Y = east - against the level's own keep-left lane markings.

#### Fixed
- The scorer's frame width: rows carry `det.img_w`, `track_truth.load_rows`
  recovers it for older logs (commit 37514c3).
- The controller steered on boxes its own presence check rejected.
  `--presence-gates-control` (on in the CityLife runner) turns an ABSENT box into
  a miss.
- The control loop slept a full tick after its work, so it could never reach
  10 Hz. It now sleeps to a 0.1 s deadline (`--legacy-tick-sleep` keeps the old
  pacing): **9.31-9.37 Hz** on every red-car flight, against 4.04-6.89 Hz before
  on the same level. Still under the 9.5 gate.
- Cars keep left, on dense lane-centre polylines with filleted corners (loop A
  616 m, B 334 m, C 288 m; 7 m and 13 m arcs; speed limit per point from
  1.8 m/s^2 lateral acceleration and a 1.5 m/s^2 braking ramp).
- **Cars turned by pivoting.** `UpdateAim` and the fixed-rate `RInterpTo` yaw are
  replaced by `DriveTick`: path-curvature feed-forward plus lateral and heading
  correction (L = 5 m, zeta = 0.9), frames cut into <= 50 ms sub-steps. Pure
  pursuit was tried first in simulation and rejected (66 cm corner cut at
  3.2 m/s^2). In Simulate: worst lane error **10.1 cm**, worst lateral
  acceleration **1.93 m/s^2**.
- Cars crept into stopped cars (a 60 cm/s floor) and ignored junctions and
  pedestrians. Now they stop 6.5 m behind a queue, give way where their path
  really crosses another loop's (loop B, at (4100, 4100) and (12300, -4100)), and
  stop for a pedestrian on a crossing on their own path. In Simulate: closest
  approach between any two cars **650 cm**, longest stand-still 22.3 s.
- **The start gate could be captured by a distractor**: a red fire-hydrant sign
  held the jump gate and the instance lock for 300 s and the car was never a
  candidate. The gate now suspends both until it confirms a subject (`Acquirer`),
  and accepts only one within 45 m.
- **A level car crashed the flight loop** on its first tick
  (`'LevelCar' object has no attribute 'pose_at'`).

#### Added
- `policies/follow_car_citylife.yaml`: the car mission's policy - both
  stand-offs of `follow_pedestrian.yaml`, the envelope of `follow_car.yaml`. The
  runner used the pedestrian policy, whose 3.0 m/s cap is below the car's
  3.2 m/s.
- A ground-contact check in `presence_verdict` for things that stand on the
  road: the ray through a box's bottom edge must meet the road near the measured
  range. A red traffic signal passed every other check as a car.
- `tools/citylife_routes.py` (+ `follow_step`/`follow`, the reference the
  Blueprint mirrors, `crossings_on`, `give_way_junctions`) and
  `tests/test_citylife_routes.py` (22 tests); `tools/citylife_mcp/` -
  `ue_rpc.py`, `drive_tick.py`, `rewire_tick.py`, `apply_routes.py`,
  `one_red_car.py`, `verify_drive.py`.
- Exactly one red car: `Car_04` and `Car_22` repainted blue and white; `Car_10`
  drives loop A at 3.2 m/s.
- `--level-car`, `--start-when-seen`, `--start-max-range-m`,
  `--start-timeout-s` (`demo/follow_vlm.py`); gate snapshots and a truth trace in
  `metrics.json`; the red-car mode of `scripts/run_citylife_follow.ps1`.

#### Flown
Six red-car flights, 240 s after the start gate
(`-Object "a red car" -LevelCar Car_10`), each isolating one change:

| run | on target (null) | median px (null) | in shot | within 30 m | outcome |
|---|---|---|---|---|---|
| `citylife_redcar_far` | 0.083 (0.097) | 180.3 (145.9) | 0.262 | 0.095 | success |
| `citylife_redcar_pedpolicy` | 0.320 (0.250) | 57.3 (105.1) | 0.505 | 0.118 | success |
| `citylife_redcar_carpolicy` | 0.418 (0.392) | 55.5 (81.6) | 0.674 | 0.256 | success |
| `citylife_redcar_ground` | **0.532 (0.414)** | **39.0 (84.9)** | **0.762** | **0.314** | success |
| `citylife_redcar_high` (12 m) | 0.547 (0.487) | 20.0 (37.6) | 0.617 | 0.247 | **fail** |

(The first flight crashed and was deleted.) `p0_violation_escape_rate` 0.0 in
all. The best flight held the car within 30 m for **48.8 s over 133 m of
street**, then lost it where it turned the first corner. Video:
`docs/video/citylife_redcar_ground.mp4`. The ground-contact check did not fire
in that flight, so its improvement over `_carpolicy` is run-to-run variation.

#### Found, not fixed
- **Corners - a wall the Shield invented.** In `_carpolicy` and `_ground` the
  aircraft stalled at the same junction entrance for 40-187 s: every obstacle
  map the Guardrail loads covers only NED [-80, 78] m (the 25 August Demo_day
  survey), and past its edge `Shield._distance_at` extrapolated a border building
  into an ever-deeper wall (-35 m at the junction). The Shield swapped the
  follower's +3 m/s north for 5 m/s south, in bursts.
- **The aircraft left its altitude envelope** in `_high`: 3.1 s above the 14 m
  ceiling, peak 14.32 m, while every command the Shield emitted asked it to
  descend - each climb coincides with one of those 5 m/s phantom-wall reversals.
  12 m cruise was reverted to 8 m.
- 5 half-rate ticks in 4 min of Simulate where a car was on a zebra with a
  walking pedestrian on it; cause not established.

### 2026-09-22 — hands, hair, corners, and a city instead of a street

The CityLife level lives in the gitignored `PASBlocks/`, so what reproduces it
is `docs/FINDING-crowd-pedestrians-and-traffic.md`.

#### Fixed
- **The crowd figures had no hands.** `m_tal_nrw_base`, the mesh they were
  built on, is a 3-vertex stub whose only material slot is called `M_Hide`: it
  drives the pose and renders nothing. Epic pairs it with `m_tal_nrw_body`
  (10,648 vertices, skin, hands), which was never attached. Added as a `Body`
  component on all six variants and leader-posed like the clothes. Leader pose
  matches bones by NAME, so the different skeleton (`metahuman_base_skel`) is
  not an obstacle — the face already proved that.
- **The hair trailed the head.** It was parented to the mesh COMPONENT with no
  socket, so it followed the capsule and not the `head` bone. `BeginPlay` now
  attaches it to that bone with `KeepWorld`, which preserves the authored
  placement without computing an offset.
- **Cars pivoted at corners.** New `UpdateAim` slides the aim point up to 4.5 m
  into the next leg as the car arrives and scales the corner speed, so the
  heading target moves continuously. Measured: cars corner at 64-70 % of their
  own top speed and arc rather than turn on the spot.
- **Gap keeping deadlocked the fleet** in the first version of that change: at
  a corner two cars can each be "ahead" of the other, so both braked to zero and
  stayed there. A car now only brakes for cars facing the same way, and
  `EffSpeed` has a 60 cm/s floor.

#### Added
- The environment is a city rather than one street: **two car loops on the map's
  own 82 m junction grid (684 m and 300 m), 16 cars, 40 pedestrians**, nav
  bounds from 48 x 104 m to 200 x 185 m, and six new no-walk bands over the
  carriageways. Lanes sat 350 cm from each centre line, so the two loops read
  as two-way traffic. *(Corrected 2026-09-23: they sat on the RIGHT of travel,
  eastbound at X = 3750, against the level's own keep-left lane markings - never
  "left" as written here. Cars keep left since 2026-09-23.)* Measured in Simulate: 32 of 40 walking, **0 in any
  carriageway**, nearest car-to-car 567-1544 cm, **0 ticks under 4 m**.
- `demo/level_actors.py` + `--level-peds` in `demo/follow_vlm.py`: ground truth
  for actors the LEVEL owns, read back from the simulator with
  `World.get_object_poses`. Without it a CityLife flight logs an empty truth on
  every tick and scores unscorable while looking complete.
  `tests/test_level_actors.py` covers it (10 tests).
- `scripts/run_citylife_follow.ps1`: the follow mission against the level's own
  crowd, with no client-side pedestrians, parked cars or scripted car.
- Every pedestrian and car carries its name as an actor TAG.

#### Retracted
- **"Names are load-bearing: an actor's name IS its segmentation class"** in
  `docs/FINDING-citylife-level.md`. The plugin reads `GetOwner()->GetName()`,
  which for a placed Blueprint is `BP_CityPed_M1_C_1`. `Ped_00` is the editor
  LABEL, and labels do not exist in a `-game` build. The tags above are what
  make the intended names resolvable.

#### Added, later the same day
- Pedestrian crossings: the no-walk bands are cut by 600 cm at each painted
  crosswalk, so the navmesh stays walkable across the carriageway. A
  `NavLinkProxy` would need a struct-array write, which this toolset does not do
  reliably. Measured: 3 of 40 figures mid-crossing, 0 anywhere else in a road.
- 24 cars on three loops (a third, west of the corridor), each pedestrian on its
  own `GlobalAnimRateScale` (0.93-1.07, 27 distinct values) so 40 people no
  longer share a footfall, and the gap scan runs on every other tick.
- Measured after all of it: nearest car-to-car **430 cm, 0 ticks under 4 m**
  across 24 cars and ~1,900 ticks.

#### Flown
Four 180 s flights, `scripts/run_citylife_follow.ps1`:

| run | truth | poll | det_hz | loop_hz | on target | chance |
|---|---|---|---|---|---|---|
| `citylife_follow` | 16 of 40 | 0.1 s | 5.82 | 5.29 | ~~0.063~~ n/a | ~~0.571~~ n/a |
| `citylife_follow2` | 40 | 0.1 s | 7.60 | 4.04 | ~~0.739~~ 0.255 | ~~0.922~~ 0.802 |
| `citylife_follow3` | 40 | 0.5 s | 4.63 | 6.89 | ~~0.721~~ 0.365 | ~~0.811~~ 0.808 |
| `citylife_city` | 40 | 0.5 s | 4.82 | 6.17 | ~~0.821~~ 1.000 | **1.000** |

*(Corrected 2026-09-23: tracking columns re-scored with the frame width recovered from
`detections.jsonl`; the first scoring fell back to 400 px on a 768 px camera.
`citylife_follow` cannot be re-scored: its log is deleted.)*

- `det_hz` clears its 4.0 Hz gate in all four; the control loop clears 9.5 Hz in
  none, and neither did the reference flight on the old level (8.33 Hz).
- `p0_violation_escape_rate` **0.0** in all four. Not KPI-grade: the topology is
  `projectairsim-single-host` by construction.

#### Retracted, about our own measurement
- **`frac_on_target` no longer measures anything on this level.** Its chance
  baseline - a box placed with no skill, scored against "any pedestrian" - is
  **1.000** in the last flight, because 40 people fill the frame. The median
  error says the same: 45.5 px against a chance of 5.2 px. Tracking claims about
  CityLife have to be scored against the LOCKED instance (`target_lock`: 515
  ticks held, 19 switches), not the class. `subject_truth_pts` returning every
  pedestrian was right for 12 and is wrong for 40.

#### Not measured
- The control loop is under its gate (6.17-6.89 Hz vs 9.5) and the split between
  "this level" and "the truth polling" is not separated: that needs the same
  mission flown with `--level-peds 0` as a control.

#### Earlier, 2026-09-21

The CityLife level lives in the gitignored `PASBlocks/`, so what reproduces it
is `docs/FINDING-crowd-pedestrians-and-traffic.md`. Backup of the pre-change
assets: `PASBlocks/_backup/citylife_2026-09-21/`.

#### Added
- CityLife pedestrians are six City Sample Crowd variants (3 male, 3 female)
  instead of Manny. They reuse `BP_CityPed_Human`, built 2026-09-16 and never
  placed, and walk on the existing animation with no retarget because
  `SK_Base` is registered compatible with `SK_Mannequin`. Face RigLogic off,
  face forced to LOD3, hair as cards instead of grooms. Measured: 12 and 15 of
  16 walking, 0 in the carriageway.
- Gap keeping in `BP_CityCar`: target speed falls with the square of
  gap / (700 cm + 1.8 s x speed). Measured inside the engine: nearest other car
  621-771 cm, 0 ticks under 4 m.
- `citylife_level` in `docs/data/eval_sep2026.json` carries the pedestrian
  model, the car gap and the second finding.

#### Fixed
- `M_CarPaint` did not compile (two empty texture samples), so every
  `MIC_Paint_*` rendered in the default grey. Textures assigned; three cars now
  carry Gold, Purple and Black next to the five `VehicleVarietyPack` bodies in
  their own paint. Six of eight colours are confirmed by measured hue in one frame; the
  white truck and red sports car sat in shadow.
- The crowd figure's clothes and head followed the template's mesh, not their
  own body: `LeaderPoseComponent` on the template is a weak pointer to the CDO.
  It is now set in `BeginPlay`.

#### Retracted
- **"Paint is set per instance from the eight `MIC_Paint_*`"** in
  `docs/FINDING-citylife-level.md`. The saved level referenced no
  `MIC_Paint_*`, and their parent material did not compile, so no car could
  have shown one.
- **"Cars sat within 4 m of each other"**, same file. The positions were read
  one tool call at a time, and each call lets the game advance a frame, so the
  distance is not reliable. The same method reported 12 cm with gap keeping
  already on, while the in-engine meter read 621 cm minimum.

#### Not measured
- OWL-ViT `a person` did not improve with the realistic figures (10 m up:
  Manny 0.062, crowd 0.054 / 0.105; 20 m up: 0.027 vs 0.012 / 0.013; n = 1
  per cell). The frame-rate cost of the crowd is unknown: the editor world ran
  at 2.8 ticks/s with or without the figures. Needs `-game` plus a flight.

## [0.5.1] — 2026-09-14

### Added
- `tools/build_eval_data.py` → `docs/data/eval_sep2026.json`: every number the
  September decks and the mid-evaluation report quote, computed once from the
  artefacts, reusing `demo/track_truth.py` so there is one scorer.
- A visibility breakdown of stand-off ring positives (in frame / outside the
  horizontal FOV / under the nose), and a check of the served range against the
  pedestrian under the detection box.

### Retracted
- **"The rule saw 31 of 516 because the position the Shield was served was wrong
  by tens of metres"** — in the README, the Pages site, the 0.4.0 notes, the
  meeting pack and the September deck. Two errors in one sentence:
  - `standoff_score` counts a tick positive when *any* pedestrian is within
    10 m. `SubjectStandoff` protects only the subject being followed. On **498 of
    the 516** ticks the person inside 10 m was outside the camera's ±45°
    horizontal field of view; 6 were under the nose; 12 were in frame.
  - The "37 m served-range error" compared the subject's estimate with the
    *nearest* pedestrian — a bystander beside the aircraft. Against the
    pedestrian under the detection box (ticks 340–700) the estimate was 43.3 m
    and that person 49.4 m away; against the nearest visible pedestrian the gap
    is 1.8 m. The estimate was roughly right about the person being followed.
- **"Depth sampled through an 8-pixel box is the background"** was an inference
  from that comparison, never a measurement, and falls with it.

- **"P0 escape rate is 0.0 on every flight recorded here"** (README, Pages). It
  is 0.0 on all 41 shielded flights; the five unshielded control flights read
  0.63, as they are designed to.

### Still true
- The ring fired 94 times; on 63 of them no real pedestrian was within 10 m.
- The pedestrian-phase detector does not beat a centre constant (7.0 px against
  5.9 px).
- A real pedestrian came within 4.70 m of the aircraft while
  `p0_violation_escape_rate` read 0.0. The gap it exposes is **policy scope**
  (no rule protects non-subject pedestrians) and **sensor coverage** (a forward
  camera cannot see beside the aircraft) — not a wrong range estimate.

---

## [0.5.0] — 2026-09-10

### Added
- `CAMERA_HFOV_DEG`, read from the simulator config at import, and used as the
  default for `servo`, `implied_width_m`, `range_from_depth` and `TargetLock`.
- `det_gt_err_deg_median_in_shot` and its centre-constant null, plus
  `frame_widths_seen`, in `track_truth.score_rows`.
- Two regression tests: one changes the config and asserts the reader follows;
  one walks the AST for a literal `45.0`/`90.0` handed to `radians()`.

### Changed
- **FrontCamera raised from 400×225 to 768×432**, scene and depth together.
  `OwlViTProcessor` resizes every input to 768×768, so the forward pass does not
  depend on capture resolution — measured on an RTX 4090, median of 20, one
  query: 400×225 → 12.6 ms, 768×432 → 12.5 ms, 1280×720 → 13.2 ms. A 400×225
  frame was being upscaled 1.9× into the model. End to end 768×432 is *faster*
  than what it replaces (30.7 ms against 35.2 ms).

### Fixed
- `math.radians(45.0)` was inlined in the estimator feed — the camera's
  half-FOV, as a literal — while `servo()` three hundred lines away took
  `hfov_deg` as a parameter. Re-aiming the camera would have scaled every
  estimator bearing wrong, served the Shield a subject in the wrong direction,
  and raised nothing.

### Known limitations
- **The 768×432 change has not been flown.** Offline measurement says it costs
  nothing; behaviour under Unreal's GPU contention is unmeasured, and that
  contention is severe — across five recorded flights the forward pass runs
  13.8–21.5× slower in flight while preprocessing runs only 1.8–2.6× slower.
  The detector is GPU-starved, not CPU-bound.
- Raising resolution buys *detail*, not *size*: model pixels are
  `(angular width / hfov) × 768` regardless of capture resolution. A 0.5 m
  pedestrian still occupies 0.48 of one 32×32 patch at 16 m. Only a narrower
  FOV changes that, and it is not yet taken.

---

## [0.4.0] — 2026-09-09

### Added
- `SUBJECT_VMAX_MPS` and a post-update velocity clamp in `TargetState`. A
  constant-velocity filter cannot know that a person does not travel at
  13 m/s; once the estimate ran away, its own 4σ gate rejected 149 of the next
  154 measurements and it coasted the runaway velocity to the end of the
  flight. Pedestrian phase on replay: estimate served 55/331 → 257/331, error
  to the nearest real pedestrian 31.08 m → 9.15 m.
- `yaw_command` — one yaw cap in one place. The coast branch clipped to
  1.1 rad/s and the estimator branch clipped to nothing; `TargetState.observe()`
  can return a bearing behind the aircraft, so it reached 130.4 deg/s.
- `track_truth.score_standoff_firings` — TP/FP/FN and precision/recall per
  `SubjectStandoff` ring, surfaced in `metrics.json` as `standoff_score`.

### Retracted
- **The claim that the 10 m pedestrian stand-off "fired six times" was the
  demonstration the rule finally worked.** Scored against ground truth from the
  same flight log: 0 true positives, 6 false, 49 false negatives, precision
  0.00. All six fired against a diverged estimate 3.81–9.39 m away while the
  nearest real pedestrian was 16.23–16.53 m away.
- Three candidate fixes were measured and dropped rather than shipped: a
  presence-gated selection filter (the verdict marks the *car* phase's most
  accurate boxes as absent), an implied-width gate on the estimator (its
  apparent win is the speed clamp's, and together they are worse than the clamp
  alone), and an implied-width gate on selection.

### Fixed
- The re-flight comparison quoted whole-flight figures for two flights of
  different length (69.95 s against 119.95 s). On the matched window every
  figure is better than published — median |yaw| 11.5 → 1.3 deg/s, peak
  130.4 → 9.6 — and so is the thing that matters less: the aircraft came 4.70 m
  from a real person against the previous flight's 9.43 m.

### Known limitations
- `p0_violation_escape_rate` reads 0.0 on a flight where a real pedestrian came
  within 10 m on 516 ticks and the stand-off rule fired on 31. *(Corrected in
  0.5.1: 498 of the 516 were people outside the camera's view whom the
  subject-only rule does not protect — a scope and coverage gap, not a range
  error.)*

---

## [0.3.0] — 2026-09-08

### Added
- `guardrail/replay.py` — WP4 replay bundles that re-derive their own KPIs.
- `guardrail/frames.py` — the body/world Action4D boundary, with `yaw_rate`
  documented as rad/s and pinned by test.
- `demo/occ_bands.py` — altitude-band-aware obstacle map selection.

### Changed
- The headline tracking statistic moved from `frac_on_target` to the median
  pixel error against a null family (uniform draw, centre constant, best fixed
  column, lag-1 persistence).

### Retracted
- **`frac_on_target` at a 100 px tolerance is a statistic a constant can pass.**
  A "detector" that emits the frame centre and never opens the image scores
  1.000 on `city_locked` — margin exactly 0.000 — and *beats* the real detector
  on `city_full`.
- `replay.py`'s `VERIFIED_KPI_FIELDS` named two keys nothing emits, so it
  compared five fields and reported seven; one of them was a grant KPI. Its
  policy guard was also skipped whenever `manifest.json` was absent, so a real
  flight bundled cleanly under all 26 foreign policies.
- Test runners printed `PASS` for fixture-less tests: "8/8 passed" for a run in
  which seven asserted nothing.

---

## [0.2.0] — 2026-09-01 → 2026-09-07

### Added
- Scenario sweep harness (`experiments/sweep_scenarios.py`).
- Two previously unmeasured grant KPIs, and two previously unbuilt rules.
- Signed policy bundles, WGS84 policy origins, and a start-up refusal for a
  rule no `--object` phrase can reach.
- September deck built from the artefacts rather than from memory, and the
  meeting pack.

### Fixed
- **A `SubjectStandoff` rule that was hashed, audited, and completely inert.**
  The policy named class `pedestrian`; the phrase `a person` produced `person`;
  `binds()` compares exact strings. Across a whole flight with a human subject
  it bound zero times and the 5 m catch-all applied instead. No violation is
  indistinguishable from no rule.
- `--want-width` is an *angular* target, so the stand-off it asks for scales
  with the subject's width — 15.83 m for a 4 m car, 1.98 m for a 0.5 m person.
  The retarget block updated the width and not the set-point derived from it,
  so the aircraft station-kept at the car's distance and was commanded
  *backwards* at −1.18 m/s while doing so.

---

## [0.1.0] — 2026-08-24 → 2026-08-31

Initial Guardrail layer: the policy DSL (WP1), the prefix compiler (WP2), the
suffix Safety Shield (WP3) and stress testing (WP4); the first KPI-grade
results on the ArduPilot SITL rail; occupancy maps built from the simulator
rather than inferred from empty space; scripted city scenes with traffic,
parked vehicles and pedestrians; and the two-view recorder.

---

## Publication note

This history was rewritten once, at first publication on 2026-09-10, to remove
`meeting notes/` and `docs/CORRECTIONS-2026-09-02.md` — internal records naming
colleagues, withheld deliberately. Every commit SHA therefore differs from the
pre-publication working copy, and four commit hashes cited in
`docs/CHECKLIST-remaining-work.md`, `docs/FINDING-the-kpi-was-never-measured.md`
and `docs/MIDTERM-REPORT-Aug2026.md` refer to that earlier history. One commit
("Record the 2026-09-02 meeting…") touched only the removed paths and was
pruned as empty, so the published history is 50 commits where the working copy
has 51.
