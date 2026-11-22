# BitVLA's real loader on the AGX Orin, 22 Nov 2026 (ARCH-06)

ARCH-06 asks for a switchable backend slot: "CognitiveDrone, OpenVLA
generic, BitVLA, in-house stubs, all conforming to 4-D". Until today BitVLA
had an adapter (demo/vla_server_foreign.py, family `bitvla`, rule
`bitvla_chunk_velocity4/1`, 17 Nov) that had never loaded a model. This run
loads BitVLA's published checkpoint once on the Jetson AGX Orin with
BitVLA's own model code and sends its output through this repository's 4-D
slot to a headless Shield.

**What this is and is not.** The checkpoint is
`hongyuw/ft-bitvla-bitsiglipL-224px-libero_spatial-bf16` (MIT): a LIBERO
robot-arm fine-tune. Its 7 numbers per row are end-effector deltas plus a
gripper command; no BitVLA drone checkpoint exists. The adapter reads row 0
as a body-frame velocity (vx, vy, vz = a[0..2], yaw rate = a[5]). So the 4-D
actions below are a **conformance check** (BitVLA's real output has the
shape and range the slot and the Shield accept). They are **not** a policy
that means anything in flight. Nothing was flown, and no hil campaign ran.

## Result in one table

| | Value |
|---|---|
| Machine | Jetson AGX Orin, JetPack 7.2.1, aarch64, Python 3.12.3 |
| Torch / CUDA | 2.14.1+cu132 / 13.2 (the Jetson build of `~/vla-drone-env`, read-only) |
| transformers | 4.51.0.dev0, BitVLA's fork (`BitVLA/transformers`); tokenizers 0.21.4 |
| BitVLA code | github.com/ustcwhy/BitVLA @ a8d62dfe398bfcce01283278cd716d4d6eae3c5a |
| Model class run | `openvla-oft/bitvla/model/bitvla_for_action_prediction.py`, sha256 f4b5e55b...e1ef8 |
| Identity | `bitvla/ft-bitvla-libero_spatial@38604209` (probe), `bitvla/bitvla-libero-spatial@38604209` (CLI server); weights_sha256 386042090e8c...0cefc |
| Load (warm page cache) | 5.94 s: processor 1.96 s, model 3.25 s, action head 0.72 s (CLI server: 6.49 s) |
| GPU memory | 5,585 MiB allocated after load; peak 6,195 MiB during inference (torch) |
| System RAM (unified) | 9,082 MB before load, 13,664 MB peak (tegrastats.log) |
| Latency per answer | median **1.479 s** (min 1.466, max 1.490; n = 20, each frame's first call excluded); model forward 1.457 s, input build 8-19 ms; very first call 3.04 s |
| Over TCP (CLI server + real client) | 12 decisions in 20 s, 0.60 Hz; 188 Action4D ticks, all finite |
| Output | an 8 x 7 chunk per call, the same answer on 5 of 5 repeats per frame |
| 4-D slot | 5 of 5 frames -> finite Action4D; the client's strict reader agrees with the server's step on all 5 |
| Shield | `Shield.filter` took all 15 (frame, state) cases; it changed only vz, under `alt-band`: -0.408 -> -0.400 at 4 m, to -0.040 at 0.2 m above the 2 m floor (table below) |
| This repository's code | commit 1c42f4a (the loader fix), run before its last edits, which touched docstrings and the `UPSTREAM` text only (so serve_requests.jsonl's hello still shows the older `upstream.code` text) |

1.48 s per answer is about 0.68 Hz, far below the 10 Hz the grant's
decision loop wants. That is PyTorch eager with BitVLA's online quantisation:
the checkpoint holds bf16 master weights and quantises them to ternary in
every forward, so the 5.6 GB of weights take 5.6 GB on the card. BitVLA's
README points to robot.cpp (I2S GGUF) for the real speed and memory savings.
robot.cpp was not tried here.

## The loader had to change (the 17 Nov adapter could not have run)

The 17 Nov `BitVLAModel` was written from the remote code before any
download. Against the real checkpoint it had five faults. Each would have
stopped it or changed its answers:

1. **No `set_constant`.** `predict_action` reads `image_token_idx`,
   `stop_index` and the other token ids, which only `set_constant` sets.
   Without it, AttributeError. Now it gets bitvla/constants.py's `BITNET_*`
   ids (`BITVLA_TOKEN_IDS`).
2. **`norm_stats` is null in config.json.** The evaluator loads
   dataset_statistics.json into `norm_stats`. Without it,
   `_unnormalize_actions` fails.
3. **Wrong prompt.** OpenVLA's `In: ... Out:` prompt has no image token.
   The remote code counts image tokens against image features and refuses a
   mismatch. Now the evaluator's user turn is used,
   `<|image_pad|><|image_pad|><proprio_pad>What action should the robot take to fly to the red car?`,
   through the tokenizer's chat template, with each image token expanded
   to 256 (one per 14 px patch of 224 px).
4. **Pixel values must be 5-D** (batch, images, 3, 224, 224). The
   processor's 4-D tensor fails the remote code's unpacking.
5. **The checkpoint's model file is stale.** BitVLA's evaluator
   (`check_model_logic_mismatch`) overwrites the checkpoint's
   `bitvla_for_action_prediction.py` with the repository's copy. The two
   differ: the repository's copy passes `use_bi_attn=True`, the
   bidirectional attention OFT's parallel decoding was trained with. The
   checkpoint's copy runs causal attention. `BitVLAModel` now runs the
   repository's class (`--bitvla-code DIR`, BitVLA's `openvla-oft` folder)
   and records its sha256 in the identity's facts (`code`). The weight
   digest no longer hashes the checkpoint's unused .py.

Two input choices, both recorded in the identity (`image_pipeline`):

- **Two image slots, one camera.** The LIBERO fine-tunes take two cameras
  (third-person + wrist, `--num_images_in_input 2`). The drone's one frame
  fills both slots.
- **No proprio.** The checkpoint was trained with an 8-D arm state; a drone
  has none, so the `<proprio_pad>` slot keeps its token embedding.

**Image pipeline without TensorFlow.** The evaluator resizes the image to
224 (lanczos) and takes a 0.9-area centre crop, using TensorFlow, which has
no build in this aarch64 / Python 3.12 environment.
`bitvla_prepare_image` does the same with Pillow. Pillow's resampling is not
bit-identical to TensorFlow's.

**prismatic without its `__init__`.** openvla-oft's `prismatic` package
imports its training stack (TensorFlow, RLDS, draccus, rich) from its
`__init__` files. `import_bitvla` registers the packages the loader reads
by path, the way `_import_codec` does for guardrail, so only
`prismatic.vla.constants`, `prismatic.training.train_utils` and
`prismatic.models.action_heads` run. The constants resolve to LIBERO's
(chunk 8, action dim 7, BOUNDS_Q99), and the loader checks them.

The fix is covered by two new tests in tests/test_vla_backends_cd_bitvla.py:
`test_bitvla_asks_in_its_evaluators_words_not_openvlas` (the prompt
against a transcription of the evaluator, the token expansion, the token
ids) and `test_bitvla_frame_is_resized_and_centre_cropped_as_its_evaluator_does`.
Both fail on the 17 Nov file (10/13 there: the two new tests and the updated
digest assertion); 13/13 here.

### Do the two choices matter? (ablation.json)

The same loaded model, the same frames (Pillow frames fed straight in, no
JPEG; see the note below):

| Frame | max abs diff, causal vs bidirectional (chunk) | max abs diff, one vs two image slots |
|---|---|---|
| protocol_present_front | 0.396 | 0.059 |
| protocol_absent_front | 0.328 | 0.061 |
| citylife_fpv_before | 0.498 | 0.170 |
| citylife_fpv_after | 0.282 | 0.056 |
| synthetic_grey | 0.311 | 0.079 |

Causal attention (what the checkpoint's own file would run) changes every
row 0 a lot. It flips the sign of a[0] on all five frames and of a[2] on
three of them. Running the repository's class is therefore a real choice,
not a formality. One vs two slots moves the answer about a tenth as much.
After the causal pass, the bidirectional answer came back bit-identical
(`restored_equals_first` true on all five).

Note: ablation.json's bidirectional rows differ slightly from
measurements.json's (e.g. a[0] -0.195 vs -0.210 on protocol_present_front).
The probe sends each frame as a JPEG (quality 90), as a flight's request
does. The ablation feeds the decoded frame directly. synthetic_grey, which
JPEG leaves unchanged, is identical in both.

## Real output through the 4-D slot (measurements.json, shield_check.json)

Frames: two 224 x 224 front tiles of the protocol test frames
(deploy/evidence/vla_protocol_frames, top half), two CityLife FPV frames
(docs/img/progress_0930_*_fpv.jpg, 1280 x 720) and a flat grey 640 x 480.
Object words: "a red car". Instruction: "fly to the red car" (the BitVLA
family's default). Horizon 1 s.

| Frame | chunk row 0 (a0..a6) | (vx, vy, vz, omega) read | step flown (fwd, right, down, dpsi) | Action4D (vx, vy, vz_up, yaw_rate) |
|---|---|---|---|---|
| protocol_present_front | -0.210, 0.025, -0.403, -0.013, -0.010, -0.211, 0.582 | -0.210, 0.025, -0.403, -0.211 | 0, 0, 0.408, 0.202 | 0, 0, -0.408, 0.202 |
| protocol_absent_front | -0.210, -0.048, -0.284, 0.011, 0.035, -0.157, 0.523 | -0.210, -0.048, -0.284, -0.157 | 0, 0.051, 0.306, 0.157 | 0, 0.050, -0.306, 0.157 |
| citylife_fpv_before | -0.274, 0.081, -0.362, -0.011, -0.028, -0.054, 0.520 | -0.274, 0.081, -0.362, -0.054 | 0, -0.102, 0.408, 0.045 | 0, -0.102, -0.408, 0 |
| citylife_fpv_after | -0.220, 0.144, 0.212, 0.017, -0.087, -0.117, 0.252 | -0.220, 0.144, 0.212, -0.117 | 0, -0.153, -0.204, 0.112 | 0, -0.152, 0.204, 0.112 |
| synthetic_grey | -0.066, 0.036, -0.197, -0.022, -0.013, -0.011, 0.535 | -0.066, 0.036, -0.197, -0.011 | 0, -0.051, 0.204, 0.022 | 0, -0.051, -0.204, 0 |

What the slot loses here, as the 17 Nov tests predicted:

- **a[0] < 0 on all five frames, so forward is flown as 0.** The AerialVLA
  step codec has no backward step (`clipped: ["fwd_m"]` on every answer).
- Steps are quantised to the codec's bins: about 0.05 m on right_m, 0.10 m
  on down_m, 0.02 rad on dpsi.
- Yaw rates under the tracker's 0.05 rad heading tolerance become 0
  (citylife_fpv_before, synthetic_grey).

Action4D is what the StepTracker hands the Shield, in the hil launcher's
rate execution for `FAMILY=bitvla`: H 1 s, cruise 4.5 m/s, cap 5 m/s, turn
while moving. That is the same configuration as the tests'
`LAUNCHER_TRACKER`.

### Shield.filter on that Action4D (policies/demo_policy.yaml)

| Frame | open air (-20, -20, 4 m) | 2.5 m south of the no-fly square, heading North | 2.2 m (0.2 m above the 2 m floor) |
|---|---|---|---|
| protocol_present_front | vz -0.408 -> -0.400, alt-band | vz -0.408 -> -0.400, alt-band | vz -0.408 -> -0.040, alt-band |
| protocol_absent_front | passed unchanged | passed unchanged | vz -0.306 -> -0.040, alt-band |
| citylife_fpv_before | vz -0.408 -> -0.400, alt-band | vz -0.408 -> -0.400, alt-band | vz -0.408 -> -0.040, alt-band |
| citylife_fpv_after | passed unchanged (climb) | passed unchanged | passed unchanged (climb) |
| synthetic_grey | passed unchanged | passed unchanged | vz -0.204 -> -0.040, alt-band |

The no-fly square never acts: with forward flown as 0, no answer moves
North into it. That case is covered by the 17 Nov sweep, not by this
checkpoint.

### Over TCP (tcp_client.json, serve.log, serve_requests.jsonl)

The CLI server ran on loopback (127.0.0.1:8797, not a campaign port):
`serve --model bitvla --bitvla-code ...`. The real client connected to it:
`RemoteVLA` with strict identity, then `RemoteVLABackend.act()` with the
launcher's StepTracker, then `Shield.filter`. The run lasted 20 s at a 10 Hz
tick on the citylife_fpv_before frame.

- Identity check passed.
- 12 decisions at 0.60 Hz. The first inference took 3.0 s (warm-up).
- 188 ticks, all finite.
- Distinct Action4D values: the zero action before the first answer, then
  (0, 0, -0.408, 0) and (0, -0.102, -0.408, 0).
- The Shield flagged `alt-band` and nothing else.

## Environment (built for this run, nothing else touched)

`~/vla-drone-env`, `~/vla-dronecats-env`, the campaign checkout
`~/vla-drone` (still at 71aabb34, clean) and the container
`vlaguard-orin-mavros-1` were not modified. The new venv reads the Jetson
torch from `~/vla-drone-env` through a `.pth` line. Its own site-packages
come first, so the fork shadows transformers 4.40.1 only inside the new
venv. `pip uninstall` there would not reach the old env: pip does not
uninstall outside its environment.

```bash
# on the AGX (network installs approved for this environment)
mkdir -p ~/bitvla-src && cd ~/bitvla-src
git clone --depth 1 https://github.com/ustcwhy/BitVLA.git      # a8d62df
python3 -m venv ~/bitvla-env
echo "$HOME/vla-drone-env/lib/python3.12/site-packages" \
  > ~/bitvla-env/lib/python3.12/site-packages/zz_vla_drone_env_readonly.pth
~/bitvla-env/bin/python -m pip install --no-deps -e ~/bitvla-src/BitVLA/transformers
~/bitvla-env/bin/python -m pip install --no-deps "tokenizers>=0.21,<0.22" "diffusers==0.31.0"
```

The venv's own packages are only those three: transformers 4.51.0.dev0
(editable, the fork), tokenizers 0.21.4 and diffusers 0.31.0. diffusers is
needed by `prismatic.models.action_heads`. diffusers 0.43.0 was tried first
and failed to import against huggingface_hub 0.36.2; draccus was tried and
removed once prismatic was loaded by path. Neither BitVLA's `openvla-oft`
nor its `bitvla` package is pip-installed. The loader puts them on the path
(`--bitvla-code`).

The checkpoint was downloaded earlier on 22 Nov with the user's approval
(~/bitvla_dl.sh, huggingface_hub snapshot_download) to
`~/models/bitvla-libero-spatial` (5.6 GB).

## Commands

The worktree's demo/, guardrail/, policies/ and the frames were copied to
`~/bitvla-run/repo` on the AGX (not the campaign checkout). Then:

```bash
cd ~/bitvla-run/repo
# 1. the probe: digest, load, 5 frames x 5 calls through ForeignServer.handle
tegrastats --interval 1000 --logfile ~/bitvla-run/out/tegrastats.log &
~/bitvla-env/bin/python docs/data/bitvla_loader_20261122/probe_agx.py \
    --weights ~/models/bitvla-libero-spatial \
    --code ~/bitvla-src/BitVLA/openvla-oft --out ~/bitvla-run/out
# 2. the two loader choices
~/bitvla-env/bin/python docs/data/bitvla_loader_20261122/ablation_agx.py \
    --weights ~/models/bitvla-libero-spatial \
    --code ~/bitvla-src/BitVLA/openvla-oft --out ~/bitvla-run/out
# 3. the CLI server on loopback and the real client, then the server stopped
~/bitvla-env/bin/python demo/vla_server_foreign.py serve --model bitvla \
    --weights ~/models/bitvla-libero-spatial --bitvla-code ~/bitvla-src/BitVLA/openvla-oft \
    --host 127.0.0.1 --port 8797 --log ~/bitvla-run/out/serve_requests.jsonl \
    --digest-sidecar ~/bitvla-run/out/digest_sidecar.json &
~/bitvla-env/bin/python docs/data/bitvla_loader_20261122/tcp_client_agx.py \
    --port 8797 --out ~/bitvla-run/out --seconds 20
```

On the desktop, from the copied responses.jsonl, in a Python with
guardrail's requirements (pydantic, shapely):

```bash
python docs/data/bitvla_loader_20261122/shield_check.py
```

All AGX processes started for this run (probe, ablation, server, client,
tegrastats) exited or were stopped by 08:13 CST. `ps` and `nvidia-smi`
showed none left. The AGX is free for the afternoon campaign.

## Files

| File | What |
|---|---|
| measurements.json | versions, digest, load timing, memory, per-frame chunks, mapping, step, timing, latency summary, prompt text (image tokens shown as `x256`) |
| responses.jsonl | every protocol answer of the probe (25), identity included |
| shield_check.json | client reader, Action4D and Shield.filter per frame and state |
| ablation.json | causal vs bidirectional, one vs two image slots |
| tcp_client.json, serve.log, serve_requests.jsonl | the CLI server and the real client over loopback (the request log stores each JPEG's length and sha256, not the image) |
| tegrastats.log, tegrastats_serve.log | 1 s tegrastats during the probe and the server run |
| probe.log | the probe's console output (the fork prints each layer's quantisation) |
| probe_agx.py, ablation_agx.py, tcp_client_agx.py, shield_check.py | the scripts that produced the above |

AGX home paths are written as `~/` in every file here.

## What this does not close

- **No drone checkpoint, no flight.** The LIBERO output is an arm's motion
  read as a velocity. A BitVLA drone fine-tune (its released fine-tuning
  code, on drone data) is what would make it a pilot.
- **0.6-0.7 Hz in PyTorch** on the Orin. The 10 Hz loop needs robot.cpp or
  another runtime with offline ternary weights; that was not tried.
- tools/vla_backend_table.py still lists bitvla as unavailable on the
  desktop (true there) and says nothing about this AGX load. The table and
  the report rows were left for the caller to update.
