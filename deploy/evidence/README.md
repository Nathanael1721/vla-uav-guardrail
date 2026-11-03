# deploy/evidence - what each host measured

One folder per topology. A file here is a measurement, with the host facts and
the command that produced it inside it; regenerate it with that command, never
by hand.

| Path | Produced by | What it is |
|---|---|---|
| `shield_tick_pack.json` | `python tools/profile_shield_tick.py pack` (desktop, needs `demo/out`) | The portable input for the Shield tick profile: the states, actions, subjects, policies and obstacle map of four real flights on two rails, so the Orin replays exactly what the desktop replays. Each flight's `rail` is today's name for it (`dev` for the three ArduPilot SITL + MAVROS 2 flights, which recorded the retired label `canonical-hil`, kept as `rail_recorded`). None of them is KPI-bearing. |
| `vla_protocol_frames/` | copied from `experiments/out/frames` | The two camera inputs of the R1 rate protocol (`tools/vla_backend_table.py protocol`), with their poses and SHA-256. |
| `dev/shield_tick_profile.json` | `python tools/profile_shield_tick.py profile --topology dev --cpus 0,2,4,6,8,10,12,14` | The desktop baseline (performance cores, one logical CPU per core) of the Shield's per-tick cost against the grant's 5 ms query and 100 ms tick budgets. |
| `dev/shield_tick_profile-ecores.json` | the same with `--cpus 16-27` | The same on the desktop's efficiency cores. |
| `dev/vla_rate_<backend>.json` | `python tools/vla_backend_table.py measure --backend <b> --topology dev` | R1 inference rate on the desktop. |
| `hil/shield_tick_profile.json` | the profile on the Orin (`docker compose ... run --rm profile`), copied from `incoming/hil/` | **Not yet measured.** The number the grant's "Orin profiling" asks for. |
| `hil/vla_rate_<backend>.json` | `docker compose ... --profile vla-bench run --rm vla-bench` on the Orin, copied from `incoming/hil/` | **Not yet measured.** |
| `hil/first_boot_check.txt` | `bash deploy/orin/first_boot_check.sh` on the Orin (runbook step 1) | The Orin as set up: L4T release, board, JetPack's CUDA / cuDNN / TensorRT, power modes, ROS on the host, Docker and its runtimes, with a verdict per expectation. Run on the AGX Orin (JetPack 7.2.1) on 2026-11-03 at 81793ff: 7 PASS; WARN no `nvidia-jetpack` meta-package; FAIL Docker, whose daemon refuses the user (not in the `docker` group). |
| `hil/gpu_check_agx_native.json` | `python deploy/orin/gpu_check.py --skip-bnb` in the AGX's native environment (`deploy/orin/agx_vla_env.sh`) | 2026-11-03 at 81793ff: torch 2.14.1+cu132 computes on the Orin (capability 8.7, matrix product within 5.7e-7 of the CPU). No bitsandbytes there. |
| `hil/vla_bf16_check.json` | `python deploy/orin/vla_bf16_check.py --model ~/models/openvla-7b` in the same environment | 2026-11-03 at 81793ff, MAXN setting, clocks not locked: OpenVLA-7B in bf16 on `cuda:0`, eight identical answers of seven finite numbers, 0.97 to 1.00 s each after the first, peak 15.47 GB. A gate, not R1. |
| `hil/aerialvla_serving_bench_agx.json` | `python demo/vla_server_v2.py bench --base ~/models/openvla-7b --adapter ~/models/aerialvla-lora/aero_vla --precision bf16 --attn sdpa --no-merge --out ...` in the same environment | 2026-11-03 at 07cee03, MAXN setting, clocks not locked: AerialVLA in the desktop's flying configuration, 5 warm-up and 50 timed requests, p50 1520.3 ms, p95 1537.8 ms, 0 parse failures, peak 15.934 GB. Not R1. Its `identity.agreement` is the RTX 4090's probe, not measured on the AGX. |
| `hil/aerialvla_serving_bench_agx_merged.json` | the same with `--merge` instead of `--no-merge`, at 6da813d | The configuration the 28 Oct comparison matrix served (bf16, sdpa, adapter merged): p50 1073.9 ms, p95 1099.6 ms, 0 parse failures, peak 15.471 GB, load 12.97 s. MAXN setting, clocks not locked; not R1. |
| `hil/companion_versions.txt` | `docker run --rm vlaguard/companion:jazzy versions` (step 8) | **Not yet run.** The ROS 2 distribution, rclpy's Python and the MAVROS build in the image that flew. |
| `hil/gpu_check_base.json`, `hil/gpu_check.json` | `deploy/orin/gpu_check.py` in the VLA base and in the built VLA image (step 8) | **Not yet run.** Whether the GPU stack computes on the Orin, 4-bit kernels included. |
| `hil/power_<date>.txt`, `hil/tegrastats_<date>.log` | `nvpmodel -q`, `jetson_clocks --show` and `tegrastats` on the Orin (step 4) | **Not yet run.** The power mode and locked clocks of each timing session, and the temperatures during it. |
| `incoming/` | the Orin's containers (`VLAGUARD_EVIDENCE_OUT`) | Git-ignored landing folder, so a measurement on the Orin never modifies a tracked file. Reviewed files are moved to `<topology>/` on the desktop and committed. |
| `vla_backend_table.json`, `.md` | `python tools/vla_backend_table.py table` | The backend comparison table: what is on disk, under which conditions, and which cells wait for the Orin. |

`tools/profile_shield_tick.py` refuses to label a result `hil` or `flight` on
a host that guardrail.manifest's Orin rule does not accept. To attach a
profile to a run, put the output of
`python tools/profile_shield_tick.py attach <profile.json>` beside the
manifest in the run's `metrics.json`; the manifest itself stays the grant's six
fields.
