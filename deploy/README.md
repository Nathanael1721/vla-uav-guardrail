# deploy/ - one stack for the grant's three topologies

This folder holds what it takes to run the same software in the grant's three
deployment topologies - **dev** (one desktop), **hil** (desktop + Jetson Orin,
the configuration every reported KPI number comes from) and **flight** (the
Orin as companion computer on a real ArduPilot autopilot) - and switch between
them with a configuration file, never a code change.

## Why a new top-level folder

`CONTRIBUTING.md` asks for a reason before a new top-level folder. None of the
existing ones fits:

- `sitl/` holds the ArduPilot SITL rail as it runs natively in WSL. Most of
  what is here is not SITL: the Orin side has no simulator at all, and the
  flight topology has none anywhere. (The containers reuse that rail's
  scripts and configuration rather than copying them.)
- `scripts/` holds Windows launchers (`.ps1`, `.bat`) for the desktop.
- `tools/` holds Python utilities.

What is here is deployment configuration that has to travel together to a
second machine: images, compose files, topology files, network configuration,
and the evidence those hosts produce. It is the one part of the repository that
the person setting up the Orin needs, and nothing else.

## Layout

| Path | What it is |
|---|---|
| `docker/companion.Dockerfile` | ROS 2 Jazzy on Ubuntu 24.04 (`ros:jazzy-ros-base-noble`) + MAVROS 2 + mavlink-router + rosbag2 + the guardrail's Python environment (3.12); no GPU. One multi-arch definition with no install-changing build arguments: amd64 on the desktop, arm64 on the Orin, the same arm64 image for hil and flight. The build stops if rclpy would run below the grant's Python 3.11. |
| `docker/vla.Dockerfile` | The GPU runtime for the OpenVLA-based backends, and where every R1 rate is measured. Base image is a build argument: CUDA 12.4 on the desktop, an L4T r36.4 PyTorch image for JetPack 6.2 on the Orin (`dustynv/torchvision:0.21.0-r36.4.0-cu128-24.04`; fallback `dustynv/l4t-pytorch:r36.4.0`). hil and flight use the same arm64 build. |
| `docker/vla-install.sh` | The VLA image's install: keeps the base image's Jetson torch (constraints, compared again at the end), runs pip against PyPI only (the Jetson bases set their own `PIP_INDEX_URL`: every pip call runs `--isolated` with `PIP_CONFIG_FILE=/dev/null`, so neither that variable nor a `pip.conf` applies, and the build stops if pip still reports a setting), and on aarch64 builds the pinned bitsandbytes for the Orin's sm_87 (CMake configure and compile, then pip, as bitsandbytes' guide says) and refuses a library without sm_87 code. |
| `docker/sitl.Dockerfile` | ArduPilot SITL at the pin in `sitl/setup_sitl.sh` (Copter-4.5.7). Desktop only. |
| `docker/entrypoint.sh` | Roles of the companion image: `router` (the native `sitl/start_router.sh`), `mavros`, `mission` (the native launch file: VLA stub, MAVLink adapter, Shield node; rosbag2; replay bundle), `profile`, `facts`, `vla-table`, `versions` (what the image holds: Ubuntu, ROS 2 distribution, rclpy's Python, MAVROS), `shell`. Nothing branches on the host. |
| `orin/first_boot_check.sh` | Run on the Orin right after JetPack 6.2 or 7.2.1 is installed (runbook step 1): read-only report of the L4T release, board, JetPack's CUDA / cuDNN / TensorRT, the power mode and this module's mode table (IDs, default, highest budgeted mode, MAXN), any ROS on the host with its version, and Docker, with a PASS / WARN / FAIL line per expectation; the release picks the expected Ubuntu and compute stack. The file is sent back. |
| `orin/gpu_check.py` | Run inside the VLA image (or its base) on the Orin (runbook step 8), or in the AGX's native environment with `--skip-bnb`: a GPU matrix product against the CPU and an NF4 4-bit round trip through bitsandbytes, each with the device it ran on, so a stack that only imports, or a result not computed on the GPU, is refused. |
| `orin/agx_vla_env.sh` | The AGX Orin's native VLA and follow-controller environment on JetPack 7.2.1 (`docs/DESIGN-topologies.md`, decision 11): a venv on the host's Python 3.12 with torch 2.14.1 + torchvision 0.29.1 from PyTorch's CUDA 13.2 index (held by a constraints file), `requirements-vla.txt` minus bitsandbytes, the follow's packages and the Project AirSim client. |
| `orin/vla_bf16_check.py` | Run after the GPU check in that environment: OpenVLA-7B loaded unquantized in bf16 answers the R1 protocol frame a few times; refused unless the weights are bf16 on the GPU and every answer is seven finite numbers. A gate, not an R1 rate. |
| `orin/sync_follow_controller.ps1`, `run_follow_controller.sh`, `run_vla_fly.sh`, `start_sim_for_jetson.ps1` | The Jetson flights against the desktop's Project AirSim: the checkout sent as a git bundle with the map and policy bundles, the follow controller or `vla_fly` started on the Jetson, the simulator started for a remote client. `-JetsonHost nathan-agxiot@10.42.0.3` for the AGX. |
| `orin/run_pas_perception.sh` | The perception rail's hil follow flown by AIOT-VLA on the AGX Orin: `demo/pas_ardupilot/perception_node.py --pilot vla --mission follow` natively on the host (ROS 2 Jazzy's rclpy and mavros_msgs from apt, `~/vla-drone-env`), on the ROS 2 graph of the companion container's MAVROS (domain, RMW, Cyclone DDS peers and FCU URL read from `topologies/hil.env`), attached to the desktop's scene and SITL; never `--dev-yaml` or `--start-sitl`. Not flown yet (`docs/DESIGN-projectairsim-ardupilot.md`, hil). |
| `docker/sitl-entrypoint.sh` | Starts SITL as `sitl/start_sitl.sh` does (EEPROM wiped, GeoFence backstop) or, for Project AirSim, as `demo/pas_ardupilot/start_ardupilot.sh` does (scene home, airframe file). |
| `docker/*.dockerignore` | Build contexts limited to the files each Dockerfile copies. The repository root is ~34 GB and holds the private signing key; neither enters a build. |
| `topologies/dev.env`, `hil.env`, `flight.env` | The only thing that differs between topologies. Same keys in all three. They select configuration; the label a run carries comes from evidence (`guardrail.manifest.detect_topology`). |
| `compose/docker-compose.dev.yml` | dev: router, SITL, MAVROS and the mission on one desktop. |
| `compose/docker-compose.hil-desktop.yml` | hil, desktop side: SITL and mavlink-router. |
| `compose/docker-compose.orin.yml` | hil **and** flight, Orin side: MAVROS, the mission, profiling, VLA bench. The companion services run with `runtime: runc` (no L4T GPU library in the safety path), the VLA bench with `runtime: nvidia`. |
| `compose/docker-compose.orin.flight.yml` | flight only: maps the autopilot's serial device into MAVROS. |
| `dds/cyclonedds.xml` | Cyclone DDS with the two hosts as unicast peers. |
| `evidence/` | What each host measures and the manifest can attach (`evidence/README.md`); `evidence/incoming/` is where the Orin writes, git-ignored. |

The code is never copied into an image. The checkout is mounted read-only at
`/opt/vlaguard`, so `guardrail.manifest.code_revision()` reads the real git
history and every run names the commit that flew. The desktop and the Orin
must therefore be on the same commit, and the Orin must never write a tracked
file (`docs/RUNBOOK-orin-hil.md`, step 7).

## What runs where

| | dev (desktop) | hil desktop | hil Orin | flight Orin |
|---|---|---|---|---|
| ArduPilot | SITL container | SITL container | - | real autopilot |
| Physics / camera | SITL built-in or Project AirSim (Windows) | same | - | the world |
| mavlink-router | container | container | - | - |
| Mission Planner | Windows, UDP 14550 | Windows, UDP 14550 | - | telemetry radio to the autopilot |
| MAVROS 2 | container | - | container | container (serial `FCU_URL`) |
| Mission: VLA stub, MAVLink adapter, Shield node | container | - | container | container (refused without `VLAGUARD_ALLOW_FLIGHT_MISSION=yes`) |
| VLA rate bench | vla image | - | vla image | same as hil |

## The Orin: JetPack 6.2

The Orin is being installed with JetPack 6.2 (Jetson Linux 36.4.3, Ubuntu
22.04, CUDA 12.6.10, cuDNN 9.3.0, TensorRT 10.3.0). The host keeps only the
driver, Docker and the NVIDIA container runtime; nothing ROS is installed on
it. The companion image is ROS 2 Jazzy on an Ubuntu 24.04 user space (the
grant's Python 3.11+ for the rclpy Shield; Humble on 22.04 would be 3.10), and
the VLA image is built on an r36.4 PyTorch base. Timing measurements fix the
nvpmodel power mode (the module's highest mode with a power budget, and its
default; not the unconstrained MAXN) and lock the clocks with `jetson_clocks`.
`docs/DESIGN-topologies.md` (decisions 2 and 10) has the reasoning and the
container tags that were checked; `docs/RUNBOOK-orin-hil.md` steps 1 to 4 the
set-up.

## The AGX Orin: JetPack 7.2.1 (since 2026-11-03)

The hil machine is now an AGX Orin 64 GB on JetPack 7.2.1 (L4T r39.2.1,
Ubuntu 24.04, CUDA 13.2.2, cuDNN 9.20, TensorRT 10.16). The r36.4 VLA bases
above do not apply there and no published image carries a JetPack 7.2 Orin
torch, so the VLA and the follow controller run natively in the venv that
`orin/agx_vla_env.sh` builds (torch 2.14.1 + torchvision 0.29.1, CUDA 13.2;
OpenVLA-7B in bf16, no bitsandbytes). The companion image needs no GPU and is
unchanged. On the board, in order:

```bash
bash deploy/orin/first_boot_check.sh 2>&1 | tee deploy/evidence/incoming/hil/first_boot_check.txt
bash deploy/orin/agx_vla_env.sh
~/vla-drone-env/bin/python deploy/orin/gpu_check.py --skip-bnb
~/vla-drone-env/bin/python deploy/orin/vla_bf16_check.py --model ~/models/openvla-7b
```

`docs/DESIGN-topologies.md`, decision 11, has what the board reported, why
the environment is native, what that costs (the R1 columns no longer share
one torch version) and the first measurements.

## Status (2026-10-08)

**Written, not built, not run.** Nothing in this folder has been through
`docker build` or `docker compose up`: that needs network downloads and the
Orin, and neither has been used yet. `tests/test_deploy_configs.py` checks
what can be checked without building: every topology file defines the same
keys, every value a compose file reads is defined (`docker compose config`
accepts every file), every port agrees across router, MAVROS and Mission
Planner, a mission starts the same nodes and records the same topics as the
native rail, a run on the Orin with `hil.env` would earn the hil label from
`guardrail.manifest`, SITL starts like the native scripts, the pins match the
environments the stored runs used, the entrypoint refuses what it must
refuse, and nothing in a build context can carry the signing key. For JetPack
6.2 it also runs the companion image's Python-floor check on 3.10 and 3.11,
the VLA install's plan (bitsandbytes compiled by CMake for sm_87 before pip
packages it) and its sm_87, torch and pip-config checks with stand-ins, the
first-boot check against stand-in Orins (an AGX Orin, an Orin NX and an Orin
Nano, with and without the Super flash; each verdict driven to its failing
side too), and the GPU check, both its verdicts and its measuring code
(against stand-in torch and bitsandbytes modules).

numpy (2026-10-19): `requirements.txt` pins numpy 2.4.6 for Python 3.11+
(vla-drone, pas, venv-ap), but both images keep numpy 1.26.4: the companion
venv sees Ubuntu's, which ROS Jazzy's Python packages are built against (as
in `~/venv-ros`), and the VLA stack needs 1.x (`requirements-vla.txt`). So
both installs drop that file's numpy line (`requirements-companion.txt`,
`requirements-core.txt`, written at build time); with both pins pip stopped
with ResolutionImpossible and neither image could be built. The tests now
resolve every install the images run, not each pin file alone.

What has run: `tools/profile_shield_tick.py` and `tools/vla_backend_table.py`
on the desktop, producing the dev baselines in `evidence/`.

See `docs/DESIGN-topologies.md` for the decisions and
`docs/RUNBOOK-orin-hil.md` for the step-by-step.
