# AIOT-VLA: the lab's own small VLA

Named NTUT-VLA until 30 Oct 2026 (package `training/ntutvla`, family
`ntutvla`); renamed for the AIoT Lab. The flights flown under the old name
keep their tags (`demo/out/ntut_*`) and their records' family `ntutvla`,
which the codec still classifies as this model's.

Status: v10b, 2026-11-01 (within 30 m after acquisition, three flights each:
a white car 0.926, a box truck 0.843, the red car 0.944 and 0.985 with a third
flight lost to a simulator camera stall). v10, with a person's data as well:
the black car 0.940 / 0.261 / 0.855 when it starts ahead and driving away
(on a slowed level), a person 0.25-0.37 on the one acquired. v9: red 0.956,
truck 0.901, white 0.811; v8: red 0.978, white 0.907.
Code in `training/aiotvla/`, served by `demo/vla_server_foreign.py --model
aiotvla`, tested by `tests/test_aiotvla.py` and `tests/test_vla_lateral_step.py`.

## Why build one

The 27-29 Oct matrix (CHANGELOG 2026-10-28, 2026-10-29) flew every VLA we
have on the red-car follow. No VLA beat the hand-written follow
consistently, and every pure (end-to-end) VLA scored 0.017-0.539 within 30 m
after acquisition, against 0.972 for the hand-written follow (n=5). Two causes were measured:

- **Latency.** The follow needs decisions every 0.3-0.4 s
  (`vla-latency-ceiling`, 24 Oct ceiling flights). AerialVLA (7B) answers in
  0.24 s alone and 0.4-0.7 s in a flight; DroneCATS 2B int4 takes 9.6 s on
  the Orin Nano.
- **Training.** None of them was trained on this task in this world; the
  AerialVLA fine-tunes (r1-r4) learned the labels but flew worse than the
  original (decision-latency domain shift, `ft-campaign-2`).

The grant also asks for a learned backend that emits the 4-D action at
10 Hz into the Shield (ARCH-01, ARCH-03, ARCH-21, ARCH-25), which no 7B
model reaches on an edge board.

## What it is

```
VLAFront frame (256x256) --+
                            |  frozen OWL-ViT B/32 (google/owlvit-base-patch32,
object words "a red car" --+  the flight detector's own weights), fp16
                            |
                            v
   grounding map G (24x24: OWL-ViT's per-patch logit for the phrase)
   appearance map A (24x24x8: patch class embeddings on 8 PCA components)
                            |
   history ring: 8 slots at 0, 0.2, 0.4, 0.7, 1.0, 1.4, 1.9, 2.5 s back
   + proprio per slot (own motion in the newest body frame, from poses)
                            |
                            v
   trained head (1.68 M parameters): shared conv encoder per slot ->
   2-layer transformer over the slots -> 3 distributions
   (fwd_m 61 cells, down_m 41, dpsi_rad 65) + 2 auxiliary outputs
                            |
                            v
   step over 0.5 s (fwd, down, heading change) -> protocol v2 step text,
   flown by vla_fly --step-as-rate 0.5 through the Shield
```

The language reaches the action only through the backbone's grounding:
no detector box, no compass hint and no hand-written servo is an input. The
compass phrase the client sends is recorded as ignored. Because the head
reads a grounding map rather than the words, a phrase never trained on
(for example "a blue car") is grounded by the same formula. Whether the
policy then follows that car is **not yet shown**: v0 is trained on red-car
flights only.

Shared code: `core.py` (stdlib + numpy) holds the history picking, the
backward-only velocity, the proprio, the label grids, soft targets,
decoding and the mirror. The trainer and the server's `core.Ring` both call
it, and `tests/test_aiotvla.py` checks that the ring's slot inputs equal the
trainer's for the same captures.

## Data and labels

- 20 recordings of the hand-written follow with `--record-vla`: 4 clean
  expert, 6 DART-perturbed, 10 DAgger (AerialVLA flew windows and the expert
  relabelled). 66,148 captures at 10 Hz.
- v0 labels: `tools/build_follow_dataset.py --no-images --horizon-s 0.5
  --stride-s 0.1 --hint-modes fresh,memory,search`, the expert's own motion
  over the next 0.5 s. It drops perturbed and VLA-flown horizons, which
  leaves 30,091 training and 4,419 hold-out samples (hold-out runs
  `ft_expert_03`, `ft_dagger_pf_5`).
- v1 labels: `python -m training.aiotvla.build_labels`, the expert's
  ASKED command at the capture's tick, in step units over H = 0.5 s
  (fwd = forward speed x H, dpsi = yaw rate x H, down = -climb x H). Why:
  vla_fly flies a step as a velocity command, and the airframe lags its
  command. A model that asks for the displacement the expert *achieved*
  therefore asks for less than the expert did, and its own speed decays
  (ntut_v0b_01 fell from 9 m to 47 m behind the car in 50 s). `asked` is the
  expert's command before any DART offset or DAgger window, so every
  perturbed or VLA-flown capture keeps its label. That gives 54,814 training
  and 6,831 hold-out samples.
- Backbone maps are cached once per run (`build_cache.py`, 5.3 ms per frame
  plus its mirror on the RTX 4090), including the maps of the frame flipped
  left-right for mirror augmentation.

## Numbers so far (hold-out, offline)

| model | fwd MAE (m) | dpsi MAE (rad) | dpsi corr |
|---|---|---|---|
| constant (training median) | 0.753 | 0.0199 | - |
| v0 | 0.0675 | 0.0034 | 0.992 |
| v0b (motion dropout 0.5) | 0.0773 | 0.0036 | 0.991 |
| v1 (command labels, dropout 0.5; its own hold-out labels, constant 0.782 / 0.0201) | 0.0888 | 0.0054 | 0.957 |

Ablations on the hold-out:

| inputs removed | v0 fwd / dpsi MAE | v0b fwd / dpsi MAE |
|---|---|---|
| own-motion proprio | 1.116 / 0.0101 | 0.085 / 0.0037 |
| all vision | 0.256 / 0.0176 | 0.580 / 0.0541 |
| grounding map only | 0.122 / 0.0082 | 0.179 / 0.0076 |
| history (newest slot only) | 0.217 / 0.0084 | 0.304 / 0.0101 |

v1, on its own (command) labels: without the motion proprio fwd/dpsi MAE
is 0.089 / 0.0056 (unchanged); without vision 0.870 / 0.0231 (dpsi
correlation 0.23); without the grounding map 0.153 / 0.0118; with the newest
slot only 0.323 / 0.0099.

v0 leaned on its own recent motion. With that motion zeroed, its forward
error grew 16x: a "copycat" that repeats its last motion. v0b drops the
motion fields on half the training samples, and it loses almost nothing
without them. It is the model that flies.

Serving check: replaying hold-out `ft_expert_03` (1,500 captures) through
the running server gives dpsi MAE 0.0027 and correlation 0.994, the same as
offline. The server path is the training path. Model time is 10.3 ms p50
(14.6 ms p95) on the RTX 4090, and the round trip is 11.9 ms.

## On the Jetson Orin Nano

v1, served on the Orin Nano Super (JetPack 6.2, L4T R36.5, MAXN_SUPER,
~/vla-drone-env: torch 2.8, transformers 4.40.1). The first 600 captures of
the hold-out `ft_expert_03` were replayed through it on the board itself:
model time 48.4 ms p50, 52.0 ms p95, 71.5 ms max, round trip 52.3 ms, i.e.
19.1 decisions per second, against the grant's 10 Hz action rate
(ARCH-03, ARCH-25; DroneCATS 2B int4 takes 9.6 s per decision on the same
board). Its answers match the PC's: dpsi correlation with the command
labels 0.989, dpsi MAE 0.0039. That figure is the server alone. A flight
with vla_fly's own detector on the same board has not been flown. Logs:
D:/vla_drone_offload/jetson_runs/aiotvla_latency_20261029.

## Closed loop (CityLife, Car_10, vla_fly e2e, 420 s, Shield on)

The matrix's scenario and score: `vla_fly --mission follow --cadence pipelined
--step-as-rate 0.5`, policy `follow_car_citylife_nfz`, scored within 30 m
after the start gate's acquisition (`fair-follow-scoring`). Runner:
the session scratchpad's `ourvla/fly_ntut.ps1`.

| flight | model | within 30 m after acquisition | car last within 30 m | VLA answers | latency p50 (capture to answer) | P0 escapes |
|---|---|---|---|---|---|---|
| ntut_v0b_01 | v0b | 0.161 | 39 s after acquisition | 813 | 0.091 s | 0.0 |
| ntut_v1_01 | v1 | 0.291 | 64 s after acquisition | 792 | 0.088 s | 0.0 |
| ntut_v1_search600_01 | v1, `--follow-search-s 600` | 0.031 (early start: the car began 29 m away and drove off; acquired at 62 m, 3.6 s; never caught) | - | 590 | 0.094 s | 0.0 |
| ntut_v2_01 / 02 / 03 | v2 (+5 recordings) | 0.243 / 0.208 / 0.185, mean 0.212 | 61 / 41 / 38 s after acquisition | 631 / 771 / 692 | 0.090-0.098 s | 0.0 |
| ntut_v2_rate_01 / 02 / 03 | v2, `--rate-yaw rate` | 0.355 / 0.253 / 0.0, mean 0.203 (03: early start, the car began 34 m away and drove off; acquired at 60 m, never caught) | 119 / 62 s after acquisition / - | 1254 / 619 / 495 | 0.089-0.091 s | 0.0 |
| cmp2_e2e_aerovla_20261028 | AerialVLA 7B | 0.153 | 30 s after acquisition | 74 | 0.620 s | 0.0 |
| cmp2_e2e_dronecats_20261028 | DroneCATS 2B | 0.086 (early start: acquired at 61 m, 4.0 s) | - | 13 | 1.200 s | 0.0 |
| hand-written follow (rep3-5 + cmp, n=5) | - | 0.972 mean | - | - | - | 0.0 |

**The turn that was never flown.** Through v2 the aircraft held a yaw
rate of 0.00 while the car's bearing swept from -2 deg to -110 deg at a
corner (ntut_v2_01, 263-280 s). The model did answer turns (|dpsi| up to
0.20 rad). vla_fly's rate executor reads dpsi as a *target heading*
(capture heading + dpsi), turns to it and holds it within a 0.05 rad
tolerance. A model answering 10 times a second asks for 0.02-0.2 rad per
step, and most of that lies inside the tolerance or was already flown since
the capture. `vla_fly --rate-yaw rate` (TrackerConfig.rate_yaw, default
unchanged) flies dpsi/H as the yaw rate itself until the next answer. That
is the meaning of AIOT-VLA's label. With it the same v2 weights turned at
corners, followed the car for up to 119 s after acquisition, and scored
0.355 and 0.253.

**Two kinds of start (corrected 30 Oct).** An earlier version of this
document called two flights (ntut_v1_search600_01, ntut_v2_rate_03)
"false start-gate seeds" and left them out. That was wrong. The level car
drives from the moment the scene loads, so where it is at take-off depends
on how long the simulator took to start. In those two flights, and in the
28 Oct DroneCATS and r1 e2e flights, the car began 15-34 m away and was
driving off. vla_fly's start gate (`--acquire-max-range-m 60`, the
matrix's setting) committed to it correctly, at about 60 m. The other
flights started with the car about 200 m away; the gate waited for it to come round and committed at
15-17 m. A late start begins the chase close behind the car. An early
start begins it 60 m behind a car that is driving away at up to the
drone's own speed cap, and no pure VLA has closed that gap.

v2 with `--rate-yaw rate` scored 0.355 and 0.253 on late starts and 0.0 on
an early start (mean 0.203, n=3); it is not a ranking. Every pure VLA, ours included, loses the car within about a
minute, at a corner. After `--follow-search-s` (20 s) without a detection,
vla_fly stops asking the VLA and hovers, so no pure VLA gets a second
chance. The next data are DAgger recordings with v1 as the learner (the
corners it fails at) and stronger DART (cars lost and found).

## More data (29-30 Oct)

Five new recordings of the hand-written follow with `--record-vla`
(scratchpad `ourvla/record_campaign.ps1`):
- three DAgger recordings, v1 flying 3-6 s windows every 9-13 s
  (`ntut_dagger_02` to `_04`; 26 windows and 1,106 VLA-flown ticks in the
  first);
- two strong DART recordings, the executed yaw replaced by 0.6-0.9 rad/s
  for 1.0-1.5 s every 5-8 s (`ntut_dart_01`, `_02`).

Together they add 12,164 captures. The first DAgger recording aborted on a
simulator ConnectionReset before its first tick. v2 trains on all 24
recordings: 66,974 training samples. The DAgger windows were flown in the
`heading` mode, because the rate-yaw finding came after them; their labels
are the expert's own commands either way. `--perturb-vla-rate-yaw rate`
(follow_vlm, run_citylife_follow.ps1 `-PerturbVlaRateYaw`) flies the next
round's windows the way the model is flown.

## v3 and on: the FrontCamera, a longer memory, the hand-written follow's start gate (30 Oct)

**Why the FrontCamera.** In the v2 flights the model's own estimate of the
car's bearing (its auxiliary output) read -5 deg while the truth was
-20 deg at the corners it lost, and its turn followed that. v3 on reads the
detector's FrontCamera (768x432, pitched -20 deg, the frame the hand-written
follow's detector reads), recorded with `follow_vlm --record-vla-front-source
detector` and sent with `vla_fly --vla-front-source detector`. v2 and earlier
read VLAFront (256x256, level). The history grew to 10 slots back to 5.5 s
(`train --long-history`).

**Which grounder.** On 571 recorded FrontCamera frames with the car in view,
the car's TRUE position was projected into the frame (pose, 90 deg HFOV,
pitch -20 deg; the scratchpad's `aiot/eval_backbones.py`), so the
reference is not the expert's own OWL-ViT detector. A hit is a peak within
3 deg of bearing and 12 % of the frame height:

| backbone | hit rate | bearing error p50 / p90 | model time (RTX 4090) |
|---|---|---|---|
| OWL-ViT B/32 (in use) | 0.751 | 2.2 / 3.5 deg | 32 ms |
| OWLv2 B/16 | 0.690 | 1.5 / 4.4 deg | 139 ms |
| Grounding DINO tiny | 0.800 | 0.5 / 3.0 deg | 141 ms |
| Grounding DINO base | 0.797 | 0.5 / 3.1 deg | 171 ms |

Grounding DINO is the most precise but over four times slower. OWL-ViT's
2.2 deg median is mostly its 3.75 deg patch grid. OWL-ViT stays; OWLv2 is
wired in as an option (`build_cache --backbone`).

**The start gate.** The hand-written follow's start gate accepts a car
within 30 m: every flight of 27-29 Oct records max_range_m 30.0 and refused
24-44 candidates as too far. The matrix flew every pure-VLA flight at
`--acquire-max-range-m 60`, which lets a flight begin 60 m behind a car
that is driving away. From v3 on, AIOT-VLA flies vla_fly's default,
30 m, the hand-written follow's gate. aiot_v3_01 was flown at 60 m: early
start, acquired at 62.7 m, score 0.0.

**Data.** Batch R1 (6 recordings of 600 s on the FrontCamera: 2 clean
expert, 4 strong DART) and batch R2 (6 DAgger, v3 flying 4-8 s windows,
about 46 % of the ticks): 56,005 captures. The planner ticks of the
hand-written follow (`plan` mode, its street-map search after it lost the
car; 3,066 of 4,660 ticks in aiot_rec_d04, where DART knocked it off for
good) are left out (`build_labels --exclude-modes plan`).

| flight | model, data | within 30 m after acquisition (all late starts) | car lost |
|---|---|---|---|
| aiot_v3g_01 / 02 / 03 | v3, R1 (19,211 samples) | 0.279 / 0.172 / 0.307, mean 0.253 | 62-64 s after acquisition |
| aiot_v4_01 / 02 / 03 | v4, R1+R2 (46,071) | 0.230 / 0.264 / 0.298, mean 0.264 | 62-63 s after acquisition |

All six flights lost the car at the same place: the aircraft near (52, 106),
the car turning at (80, 120), beside a roadside tree. The hand-written
follow passes that corner 21-24 m behind the car, already in the
intersection. AIOT-VLA arrives about 30 m behind and 15 m short of it,
because when the car stops at the light before the corner it stops too,
about 30 m back, where the expert closes to about 20 m. The car then turns
behind the tree. Offline, on the hold-out expert flight, v4 has no speed bias
by range (at 30-40 m: label 4.06 m/s, prediction 4.05 m/s). The expert
never stands 30 m behind a stopped car, so the model has no label for it.
That is the covariate shift DAgger exists for; batch R3 lets v4 fly 15-25 s
windows (about 70 % of the ticks).

| flight | model, data | within 30 m after acquisition (all late starts) | car lost |
|---|---|---|---|
| aiot_v5_01 / 02 / 03 | v5, R1+R2+R3 (64,874), on-policy samples x2 | 0.211 / 0.189 / 0.191, mean 0.197 | 39-40 s after acquisition, earlier, on the straight before the light (aircraft (44, 53)) |
| aiot_v6_01 / 02 / 03 | v6, R1+R2 (as v4) + the backbone's box per slot | 0.258 / 0.269 / 0.268, mean 0.265 | 60-63 s after acquisition, the same corner |

**R3 made it worse.** In 15-25 s windows v4 often lost the car, and the
expert's labels after that are search; 3,855 of aiot_rec_h06's 4,660 ticks
were planner ticks and were left out. v5, trained on it with the on-policy
samples weighted x2, lost the car sooner.

**Box features.** OWL-ViT's own box at the phrase's best patch (cx, cy, w,
h) now enters each slot (`train --box-features`, `HeadConfig.n_stats` 8). Its
width falls with range: median 0.072 at 15-20 m, 0.057 at 20-25 m, 0.049 at
25-30 m, 0.045 at 30-40 m (aiot_rec_e02; 1/width against true range,
correlation 0.6). On v4's data it gave the same mean as v4 but three
flights within 0.011 of each other.

**What still fails, at the one corner.** aiot_v6_01, set against the clean
expert flight aiot_rec_e02 at the same place:
- Expert: 4.4-4.7 m/s, 20-22 m behind, no Shield repair. AIOT-VLA: slows
  to 1.1-1.9 m/s while the car keeps 3.5 m/s, with the Shield repairing
  four times, so the gap opens from 20 m to 35 m.
- When the car turns at the intersection, the expert speeds up to
  4.4-4.7 m/s and cuts the corner. AIOT-VLA stops (0.1 m/s) 28 m back.
- AIOT-VLA then heads for the corner late, the Shield holds it off the
  roadside tree there, and the car is gone.

## v7: a lateral axis and a target memory (30 Oct)

**What the expert does that v3-v6 could not.** The hand-written follow flies a
holonomic aircraft. In the two clean expert recordings (aiot_rec_e01, e02;
11,997 ticks) its commanded velocity, in the body frame:

| fence mode (its keep-out logic) | share of ticks | forward p50 | sideways p50 / p90 | bearing to the car p50 |
|---|---|---|---|---|
| clear | 65 % | 3.30 m/s | 0.01 / 0.84 m/s | 0.2 deg |
| near | 14 % | 3.52 m/s | 0.80 / 1.20 m/s | 0.3 deg |
| skirt | 21 % | 1.65 m/s | 1.34 / 1.66 m/s | 0.7 deg |

On a third of its ticks it slides sideways at about 1.3 m/s with the camera on
the car, and it backs up on 12 % (median -0.55 m/s). v3-v6's labels kept the
forward and yaw components only, so along a building their lesson was "slow
down, do not turn": aiot_v6_01 crept along a building at 1.2 m/s with the
Shield's building-clearance repairs (`bld-clearance`, 3.0 m) while the car
pulled away. Its yaw rate never exceeded the executor's 0.5 rad/s cap that
the expert itself never reaches (max 0.44 rad/s).

**The lateral step.** Protocol v2 gains a LATERAL step: a fourth bin,
`right_m` (-2.5..2.5 m, 99 bins), written by the server
(`encode_step(right_m=)`), read by the codec only when asked
(`parse_action(lateral=True)`), checked by the client against the text and
bins, and flown by the step tracker as a body-frame velocity (fwd_m / H,
right_m / H) on the heading at each tick, the pair capped at the speed cap
as a vector (`guardrail/vla_link.py`; `tests/test_vla_lateral_step.py`).
Upstream's three-bin step is unchanged and AerialVLA's reader still refuses a
fourth group. A v7 checkpoint answers `right_m = v_right x H` over a 41-cell
grid (-1..1 m, +-2 m/s). Backing up is still not represented (`fwd_m` >= 0).

**The target memory.** Every v3/v4/v6 flight lost the car where it turned out
of the frame. v7 keeps an estimate of where it is:

```
OWL-ViT box at the phrase's best patch (cx, cy, w, h)
  -> ground point under its bottom edge (pinhole, pitch -20 deg, flat ground)
  -> constant-velocity Kalman filter on the car's world position
     (demo/target_state.TargetState's model; gate 4 sigma; a track starts on
      two agreeing boxes; five agreeing gated-out boxes move it; dropped after
      8 s unseen)
  -> 17 numbers in the newest capture's body frame (core.MEM_FIELDS)
  -> one more token for the head's transformer
```

A box counts only when OWL-ViT's logit is above -3.0 AND "a red car" beats its
contrast phrases ("a white car", "a blue car") at that patch. Against the
car's true position on the 18 recordings (80,797 frames):

| measure | value |
|---|---|
| boxes within 3 deg of the true bearing (car in frame) | 0.897 |
| bearing error of those, p50 | 0.4-0.5 deg |
| range error from the bottom edge, p50, 15-25 m | -11 to -12 % (the car's near end, not its centre) |
| frames without the car in view that pass the logit alone / with the contrast | 0.220 / 0.026 |
| memory position error, clean expert flights, p50 / p90 | 2.9-3.0 / 6.8-7.2 m |
| memory position error, all 18, p50 / p90 (share > 15 m) | 3.8 / 10.9 m (0.050) |

Where the long DAgger windows lost the car for minutes (aiot_rec_h01, h06), the
memory says "no track" most of the time (valid 0.28 in h06) rather than track
something else.

**Training.** v7 = v6's recipe (labels R1+R2, 46,071 samples, box features, 10
slots, motion dropout 0.5) with `--lateral --memory` (memory zeroed on 10 % of
the samples). 1,728,658 parameters. Hold-out (aiot_rec_e02):

| | v6 | v7 | constant baseline |
|---|---|---|---|
| fwd_m MAE | 0.072 m | 0.076 m | 0.765 m |
| dpsi_rad MAE | 0.0062 rad | 0.0056 rad | 0.0198 rad |
| right_m MAE (corr) | - | 0.037 m (0.974) | 0.252 m |

**Flown.** `vla_fly` e2e, CityLife, Car_10, 420 s, start gate 30 m, Shield on,
`--step-as-rate 0.5 --rate-yaw rate --vla-front-source detector` (the v7
checkpoint answers lateral steps):

| flight | within 30 m after acquisition | median distance after | farthest | Shield repair ticks |
|---|---|---|---|---|
| aiot_v7_01 | 0.961 | 20.3 m | 31.8 m | 9 |
| aiot_v7_02 | 0.983 | 19.8 m | 31.4 m | 10 |
| aiot_v7_03 | 0.961 | 20.9 m | 32.2 m | 33 |

Mean 0.968, against 0.972 for the hand-written follow (n=5) on the same
scenario and gate, and 0.265 for v6. All three started late (the car 16-17 m
away at acquisition), the model flew 2,006-2,008 of the 2,015-2,016 ticks
after acquisition, and all three passed the corner every v3/v4/v6 flight lost
the car at (at most 25.4 m behind it there) and had it within 30 m at the end
of the mission. It slid sideways
above 0.5 m/s on 40 % of the ticks of aiot_v7_01 (the expert: 39 %). P0
escapes 0.0.

**What v7 is, and is not.** The action is the trained head's: no hand-written
servo, no detector box from the flight's own detector and no compass hint is
an input. The memory is a classical filter inside the VLA package (the server),
fed only by the backbone's own box of the object words and the capture pose.

## v8: other targets by their words (30-31 Oct)

**The teacher.** The hand-written follow is AIOT-VLA's expert, and it is a
red-car follow. Flown on the level's white car (Car_22) it stayed within 30 m
on 0.182 of its ticks and spent 0.609 of them lost in its planner; on the black
car (Car_07), 0.069 and 0.784. "A white car" also grounds on white facades.
A model cannot learn a target from a teacher that does not follow it, so the
other targets' labels come from a PRIVILEGED teacher, `follow_vlm
--teacher-truth`: each detector frame's boxes are replaced by one box drawn
where the level car truly is (its ground point and a point its class height
above it, through the capture's camera), with its true range and bearing as
identity's features. Everything after the detector - the estimator, the
stand-off and yaw laws, the fence, the Shield - flies as in any follow. Its
flights are never follow results (metrics.json `teacher_truth`). The student
never sees the truth: its inputs are the frames, the object words and the
pose, as before.

| recording (600 s) | target | kind | teacher within 30 m |
|---|---|---|---|
| aiot_tch_w01 / w02 | a white car (Car_22) | teacher | 0.865 / 0.878 |
| aiot_tch_w03 / w04 | a white car | teacher + DART | 0.901 / 0.905 |
| aiot_tch_k01 | a black car (Car_07) | teacher | 0.695 |
| aiot_tch_t01 / t02 | a box truck (Car_00) | teacher | 0.797 / 0.808 |
| aiot_tch_t03 / t04 | a box truck | teacher + DART | 0.630 / 0.677 |
| aiot_teach_smoke1 (180 s) | a white car | teacher | 0.997 |

The black car drives at 4.83 m/s (p50; p90 5.61), above the aircraft's 5 m/s
policy cap, so even the teacher falls 30-70 m behind; the white car drives at
3.73 m/s and the red car at 3.20. One black recording was made; slowing Car_07
is a level change for the user to decide.

**Which phrases to contrast.** On those recordings, against each target's
truth (a hit: the box at the phrase's best patch within 3 deg of it; the
frames without the target in view are few, because the teacher keeps it in
view, so the second column is rough):

| phrase | logit alone: hits / frames without it | contrast phrases | with them |
|---|---|---|---|
| a white car | 0.957 / 0.752 | a blue car, a black car, a box truck | 0.629 / 0.155 |
| a box truck | 0.874 / 0.890 | a red car, a blue car, a black car | 0.743 / 0.161 |
| a black car | 0.883 / 0.706 | a red car, a blue car | 0.653 / 0.059 |
| a red car | 0.958 / 0.219 | a white car, a blue car (v7's) | 0.950 / 0.026 |

The box truck is not contrasted against "a white car": the trucks are white.
A checkpoint records the table it trained with, and the server grounds that.

**Training.** v8 = v7's recipe on 22 runs (the 12 red-car recordings of v7 and
the 10 teacher recordings above), with the object words' OWL-ViT text
embedding as one more token (`train --text`), and samples whose target
memory is more than 15 m from the expert's own estimate of its subject left
out (`--memory-agree-m 15`: 6,754 of them, the memory on another white car or
truck or a facade). 74,688 training and 13,563 hold-out samples (aiot_rec_e02,
aiot_tch_w02, aiot_tch_t02); 1,827,730 parameters. Hold-out: fwd_m MAE 0.062 m,
dpsi_rad 0.0049 rad, right_m 0.033 m (correlation 0.971).

**The start gate for the new targets.** vla_fly's start gate runs the
hand-written detector, which does not find the white or the black car. The
new targets' flights use `vla_fly --gate-truth`: the gate judges the teacher's
box of the level car, so the flight starts when the target is within 30 m and
in view. From then on the VLA flies on its own perception. The red car keeps
the detector gate.

**Flown.** `vla_fly` e2e, 420 s, start gate 30 m, Shield on, three flights per
target. The headline is the share of the ticks after acquisition within 30 m
of the instance acquired (the candidate nearest at acquisition), so a drift
onto another truck lowers it; beside it, any candidate and the run's intended
tag:

| target (`--level-car`) | acquired, per flight | mean | any | intended |
|---|---|---|---|---|
| a red car (Car_10; detector gate) | 0.986, 0.972, 0.977 | 0.978 | 0.978 | 0.978 |
| a white car (Car_22, Car_15) | 0.958, 0.874, 0.889 | 0.907 | 0.913 | 0.907 |
| a box truck (Car_00, Car_23, Car_14) | 0.634, 0.560, 0.651 | 0.615 | 0.649 | 0.000 |
| a black car (Car_07) | 0.042, 0.133, 0.137 | 0.104 | 0.104 | 0.104 |

P0 escapes 0.0 in all twelve. The white car was Car_22 in all three flights.
The box truck was Car_23 in all three (the nearest truck when the gate
passed). On the red car v8 does as well as v7 (0.968) or better.

What holds the other two back:
- **The box truck follows its teacher's slack.** The teacher held the truck
  23-27 m behind at the median against its 15.8 m stand-off, and v8 holds
  26.9-28.1 m, so 35-44 % of the ticks are over 30 m. Car_23 drove at 3.79 m/s,
  slower than the teacher's Car_00 (4.19): not speed. The next teacher
  recordings for the trucks use a tighter stand-off (`-ObjectWidthM 3`, 11.9 m)
  and DAgger windows.
- **The black car is not learned.** One recording of a car faster than the
  aircraft. In aiot_v8k_01 it drove south and stopped behind the aircraft, and
  v8 flew north along the route the other targets take: the route is what the
  data teaches when one phrase has one recording.

## v9: a closer teacher for the trucks, and DAgger windows (31 Oct)

The box truck copied its teacher's slack, so the trucks were recorded again
with a tighter stand-off (`run_citylife_follow.ps1 -ObjectWidthM 3`: 11.9 m;
the teacher's median distance 19.6-22.5 m, from 23-27 m), on Car_00 and on
Car_23, clean, with DART, and with v8 flying DAgger windows (aiot_tch_t05-t10).
The white car got two DAgger recordings (w05, w06). v9 is v8's recipe on 26
runs (the trucks at 15.8 m, t01-t04, left out): 92,153 training samples,
hold-out dpsi_rad MAE 0.0043 rad, right_m 0.026 m.

| target | v9, within 30 m of the instance acquired | mean | v8 |
|---|---|---|---|
| a box truck (Car_23 each time) | 0.913, 0.837, 0.952 | 0.901 | 0.615 |
| a white car (Car_22 each time) | 0.282, 0.937, 0.884, 0.896, 0.938, 0.927 | 0.811 | 0.907 (3 flights) |
| a red car | 0.938, 0.967, 0.963 | 0.956 | 0.978 |

**Where the car stops.** In the 0.282 flight the white car stopped and v9 flew
to within 0.5 m of it, out of the camera's view; it drifted toward the other
white car, lost both, and when the gate's subject lapsed vla_fly stopped asking
the model (hold, then hover) for the rest of the mission. Every white-car
flight of v8 and v9 came within 0-1.4 m of the car at least once. The step has
no reverse (fwd_m >= 0), while the expert backs off on 12 % of its ticks: a
backward axis is the next change to the step.

## v10: the black car and a person (1 Nov)

**Data.** The level's black car (Car_07) drove faster than the aircraft's
5 m/s cap, so it was slowed to 320 cm/s for one window (Environment Builder,
CHANGELOG 2026-11-01; restored afterwards). In that window: six teacher
recordings of the black car (two plain, two DART, two with v9 flying DAgger
windows; within 30 m on 0.933-1.000 of their ticks) and six of a person.
For the person, `--level-car Ped_17` makes ONE figure the level walks the
teacher's subject and the truth (`level_actors.kind_of_tags`, CHANGELOG
2026-11-01 afternoon); each run follows a different figure that walks into
the start camera's view within 30 m. The person teacher holds 12 m and keeps
its person on 0.801-0.993 of its ticks (1.000, 1.000 and 0.981 over the
first 420 s of the three plain runs; each lost its person between 488 and
507 s, in p02 while the building fence held the aircraft back).

**v10** is v9's recipe on all 38 runs; **v10b** the same without the person
runs (a control, below).

**Flown** (vla_fly e2e, `--gate-truth` for every target but the red car; the
black and person flights with Car_07 slowed):

| target | v10 | v10b | v9 |
|---|---|---|---|
| black car, start heading 90 | 0.140, 0.079, 0.144 | - | - |
| black car, start heading 270 | 0.940, 0.261, 0.855 | - | - |
| a person (the figure acquired) | 0.334, 0.252, 0.371 | - | - |
| box truck | 0.509, 0.751, 0.940 | 0.738, 0.824, 0.968 | 0.913, 0.837, 0.952 |
| white car | 0.888, 0.900, 0.258 | 0.912, 0.908, 0.959 | 0.811 (6 flights) |
| red car | 0.861, 0.421*, 0.955 | 0.944, 0.985, 0.202* | 0.938, 0.967, 0.963 |

\* the simulator's camera stream stopped for 10.9 and 8.7 s with the car 20-25 m
ahead, and the car drove out of reach (CHANGELOG 2026-11-01 evening).

**The person data costs the vehicles.** Without it (v10b) the white car is
above 0.9 in all three flights and the trucks closer to v9; with it (v10,
about a sixth of the samples, a 12 m stand-off at walking pace) both fall.
v10b is the model to fly for vehicles; a person needs more than these
recordings (below).

**The black car's start.** Loop B runs against the follow's start heading:
at 90 the car is acquired ahead driving toward the aircraft and passes under
it, and v10 lost it in all three (in aiot_v10k_01 the memory reseeded onto
something moving ahead, most likely another car).
Facing 270 the black car starts as every other target's car does, ahead and
driving away.

**A person.** v10 stays among people (within 30 m of some figure on
0.95-1.00 of the ticks) but not with one: the nearest figure changes 37-54
times a flight. The target memory is position-only, and on the pavements
figures pass within a few metres of each other; following one person needs
an appearance cue in the memory.

**Where the car stops** (every vehicle target): v10 closes into the
camera's 6.9 m blind spot, the truth gate's subject lapses, vla_fly stops
asking after its 26 s memory and search, and a car that drives off without
re-entering the view is lost (aiot_v10kw_02, aiot_v10t_01). The step has no
reverse.

**The stand-off rules did not run.** vla_fly gave the Shield no subject, so
the policy's subject stand-offs (10 m for a pedestrian, 5 m for anything)
were inert in every VLA flight up to v10b; against the truth, the white-car and truck
flights of v9 and v10 came within 0.1-3.9 m of their car. Their follow
scores stand; their "P0 escapes 0.0" does not cover the stand-off. Since
2026-11-01 (CHANGELOG "late") vla_fly feeds the perception's subject every
tick; a `--gate-truth` flight still has none to feed (its client-side box is
the truth) and says INERT.

## Limits

- Offline accuracy is not closed-loop skill. The recordings are mostly
  straight chasing, with 69 search samples in 34,510, so the model has
  seen very little of a car lost at a corner. That is the failure the
  matrix found.
- One level. v10b follows the red car, the white car and a box truck; the
  black car only when it starts ahead and driving away, flown on a slowed
  level (v10 section); a person it stays near but does not keep. The level
  has two white cars and three box trucks, so those phrases are ambiguous by
  construction (`vla_fly --level-car` takes all of them).
- The other targets' labels come from a privileged teacher, and their
  flights start on a truth gate; the student never sees the truth.
- Behaviour cloning compounds its own errors. DAgger rounds with this model
  as the learner are the next step (`follow_vlm --record-perturb-vla`
  accepts the aiotvla server as a foreign family).
- Up to v6 the step had no lateral axis (AerialVLA's codec). v7 answers
  protocol v2's lateral step (a fourth bin); backing up is still not in the
  step (fwd_m >= 0), and that is where the vehicle flights are lost: closing
  into the camera's blind spot behind a stopped car.
- Every flight listed here was flown with the policy's subject stand-offs
  inert (v10 section). Since 2026-11-01 vla_fly feeds the perception's
  subject on a red-car follow; the `--gate-truth` targets (white car, box
  truck, black car, person) still have none and are reported INERT, and the
  camera's 6.9 m blind spot is wider than the 5 m ring.
- Not KPI-grade: the sim is simple_flight on the city rail, and no hil
  topology is used.
