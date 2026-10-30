# AIOT-VLA: the lab's own small VLA

Named NTUT-VLA until 30 Oct 2026 (package `training/ntutvla`, family
`ntutvla`); renamed for the AIoT Lab. The flights flown under the old name
keep their tags (`demo/out/ntut_*`) and their records' family `ntutvla`,
which the codec still classifies as this model's.

Status: v2, 2026-10-30. Code in `training/aiotvla/`, served by
`demo/vla_server_foreign.py --model aiotvla`, tested by `tests/test_aiotvla.py`.

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
| ntut_v1_search600_01 | v1, `--follow-search-s 600` | 0.031 (not comparable: the start gate acquired at 3.6 s, a false seed; the aircraft chased it north and lost it) | - | 590 | 0.094 s | 0.0 |
| ntut_v2_01 / 02 / 03 | v2 (+5 recordings) | 0.243 / 0.208 / 0.185, mean 0.212 | 61 / 41 / 38 s after acquisition | 631 / 771 / 692 | 0.090-0.098 s | 0.0 |
| ntut_v2_rate_01 / 02 | v2, `--rate-yaw rate` | 0.355 / 0.253, mean 0.304 | 119 / 62 s after acquisition | 1254 / 619 | 0.089 s | 0.0 |
| ntut_v2_rate_03 | v2, `--rate-yaw rate` | 0.0 (not comparable: false start-gate seed at 2.4 s) | - | 495 | 0.091 s | 0.0 |
| cmp2_e2e_aerovla_20261028 | AerialVLA 7B | 0.153 | 30 s after acquisition | 74 | 0.620 s | 0.0 |
| cmp2_e2e_dronecats_20261028 | DroneCATS 2B | 0.086 (start gate at 4.0 s) | - | 13 | 1.200 s | 0.0 |
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

**False start-gate seeds.** Two of the nine AIOT-VLA flights were not comparable:
vla_fly's start gate acquired something other than Car_10 within the first
3 s (the 28 Oct DroneCATS e2e flight did the same at 4.0 s), and the score
is counted from that acquisition. This is the client's gate, not the
model, but it decides the flight.

v2 with `--rate-yaw rate` is the best pure VLA flown on this scenario, but
n=2 is not a ranking. Every pure VLA, ours included, loses the car within about a
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

## Limits

- Offline accuracy is not closed-loop skill. The recordings are mostly
  straight chasing, with 69 search samples in 34,510, so the model has
  seen very little of a car lost at a corner. That is the failure the
  matrix found.
- Trained on one target (the red car) in one level. Language
  generalisation is untested.
- Behaviour cloning compounds its own errors. DAgger rounds with this model
  as the learner are the next step (`follow_vlm --record-perturb-vla`
  accepts the aiotvla server as a foreign family).
- The step format has no lateral axis (AerialVLA's codec). The model turns
  and moves forward; it cannot strafe.
- Not KPI-grade: the sim is simple_flight on the city rail, and no hil
  topology is used.
