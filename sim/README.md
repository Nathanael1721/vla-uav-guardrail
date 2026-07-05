# Simulation — functional rail (dev topology)

This brings up the **functional rail** from the design docs
(`docs/03-simulation/dual-rail.md`): Gazebo Harmonic + `ardupilot_gazebo` +
ArduPilot SITL, with MAVROS 2 (Humble) on the safety path and `mavlink-router`
fanning out to a GCS. It is the canonical CI / KPI configuration; the **perception
rail** (Project AirSim + HIL bridge) is deferred per the 2026-06-30 fallback gate.

## Prerequisites

- Docker + docker compose on an **Ubuntu 22.04** host (ROS 2 Humble parity).
- A discrete GPU is *not* required for the functional rail (Gazebo Harmonic runs
  headless for CI).

## One-time image builds

The SITL+Gazebo combo has no off-the-shelf image; build it once:

```bash
docker compose -f sim/docker-compose.dev.yml build
```

This builds:
- `vlaguard/sitl-gazebo:harmonic` — ArduPilot SITL + Gazebo Harmonic + the
  `ArduPilot/ardupilot_gazebo` plugin (pinned in `sim/ardupilot_gazebo/`).
- `vlaguard/ros2:humble` — ROS 2 Humble + the colcon overlay (`ros2_ws/`) with the
  Shield + adapter nodes, and a pip install of the uv workspace packages so the
  nodes can `import policy_dsl / safety_shield / vlaguard_common`.
- `vlaguard/mavros:humble`, `vlaguard/mavlink-router:latest`.

## Run

```bash
docker compose -f sim/docker-compose.dev.yml up
```

Then the Shield loads the demo bundle, the VLA stub publishes `/vla/action_4d` at
10 Hz, and `/episodes/dev/shield_audit.jsonl` accumulates intercept records.

## Status

Phase-1 skeleton. The `Dockerfile`s and the `ardupilot_gazebo/` world/SDF assets
are stubbed pending a Humble host to validate against; the node logic itself is
exercised today by the offline `make demo` (no ROS required).
