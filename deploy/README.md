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
| `docker/companion.Dockerfile` | ROS 2 Jazzy + MAVROS 2 + mavlink-router + rosbag2 + the guardrail's Python environment. One multi-arch definition with no install-changing build arguments: amd64 on the desktop, arm64 on the Orin, the same arm64 image for hil and flight. |
| `docker/vla.Dockerfile` | The GPU runtime for the OpenVLA-based backends, and where every R1 rate is measured. Base image is a build argument: CUDA 12.4 on the desktop, L4T (JetPack 6) on the Orin. hil and flight use the same arm64 build. |
| `docker/sitl.Dockerfile` | ArduPilot SITL at the pin in `sitl/setup_sitl.sh` (Copter-4.5.7). Desktop only. |
| `docker/entrypoint.sh` | Roles of the companion image: `router` (the native `sitl/start_router.sh`), `mavros`, `mission` (the native launch file: VLA stub, MAVLink adapter, Shield node; rosbag2; replay bundle), `profile`, `facts`, `vla-table`, `shell`. Nothing branches on the host. |
| `docker/sitl-entrypoint.sh` | Starts SITL as `sitl/start_sitl.sh` does (EEPROM wiped, GeoFence backstop) or, for Project AirSim, as `demo/pas_ardupilot/start_ardupilot.sh` does (scene home, airframe file). |
| `docker/*.dockerignore` | Build contexts limited to the files each Dockerfile copies. The repository root is ~34 GB and holds the private signing key; neither enters a build. |
| `topologies/dev.env`, `hil.env`, `flight.env` | The only thing that differs between topologies. Same keys in all three. They select configuration; the label a run carries comes from evidence (`guardrail.manifest.detect_topology`). |
| `compose/docker-compose.dev.yml` | dev: router, SITL, MAVROS and the mission on one desktop. |
| `compose/docker-compose.hil-desktop.yml` | hil, desktop side: SITL and mavlink-router. |
| `compose/docker-compose.orin.yml` | hil **and** flight, Orin side: MAVROS, the mission, profiling, VLA bench. |
| `compose/docker-compose.orin.flight.yml` | flight only: maps the autopilot's serial device into MAVROS. |
| `dds/cyclonedds.xml` | Cyclone DDS with the two hosts as unicast peers. |
| `evidence/` | What each host measures and the manifest can attach (`evidence/README.md`); `evidence/incoming/` is where the Orin writes, git-ignored. |

The code is never copied into an image. The checkout is mounted read-only at
`/opt/vlaguard`, so `guardrail.manifest.code_revision()` reads the real git
history and every run names the commit that flew. The desktop and the Orin
must therefore be on the same commit, and the Orin must never write a tracked
file (`docs/RUNBOOK-orin-hil.md`, step 5).

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

## Status (2026-10-06)

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
refuse, and nothing in a build context can carry the signing key.

What has run: `tools/profile_shield_tick.py` and `tools/vla_backend_table.py`
on the desktop, producing the dev baselines in `evidence/`.

See `docs/DESIGN-topologies.md` for the decisions and
`docs/RUNBOOK-orin-hil.md` for the step-by-step.
