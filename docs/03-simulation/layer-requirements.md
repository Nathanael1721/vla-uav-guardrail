# Layer requirements

Every simulator candidate is judged against five layer requirements. A candidate that satisfies only some layers must compose with one that fills the rest — most decisions in this report are about **L2 (world/perception)** with AP SITL providing **L1 (inner loop)**.

## The five layers

| Layer | Requirement | Why load-bearing |
|---|---|---|
| **L1 — Inner-loop FDM + autopilot** | ArduPilot SITL bit-identical to flight firmware | The grant title is "ArduPilot UAVs". Safety Shield KPIs (P0 escape rate, fail-safe trigger correctness) only mean something if the *actual* AP firmware is the safety net being measured. |
| **L2 — World / sensor synthesis** | RGB camera ≥10 Hz; depth; IMU/GPS truth; dynamic actors (moving obstacles, wind, GPS denial) | The VLA observation rate sets a floor; dynamic actors are required by stress scenarios. |
| **L3 — ROS 2 plumbing** | MAVROS 2 first-class; Shield as a ROS 2 node; rosbag2 recording for replay | Determinism contract for stress-test KPI runs depends on rosbag2 capture. |
| **L4 — GCS** | Mission Planner via `mavlink-router` fan-out (TCP 5760 / UDP 14550), coexisting with MAVROS | Mission Planner is the operator's UI; AI Wings real-flight uses it. Fan-out (not serial chain) avoids a single-point bottleneck. |
| **L5 — VLA contract** | Surface obs at ≥10 Hz; consume 4-D `(vx, vy, vz, yaw_rate)` at ≥10 Hz; deterministic with locked seed; `sim_speedup=1` for HIL KPI runs | The KPI numbers in the final report must be reproducible bit-for-bit from a single git SHA + seed + policy hash. |

## What "load-bearing" means in practice

- **L1 has no realistic substitute** for this grant. JSBSim and PX4 SITL are excluded by name in [Inner-loop candidates](inner-loop.md).
- **L2 is the actual decision space.** Every simulator survey decision is downstream of L2 quality, ArduPilot integration quality, and license/hardware risk.
- **L3 is non-negotiable.** A simulator without ROS 2 first-class support either gets dropped or requires custom bridge code that becomes maintenance debt.
- **L4 is a coexistence requirement, not a feature.** Mission Planner must be reachable *while* MAVROS is active — solved at the wire level by `mavlink-router`, not at the sim level.
- **L5 is the hidden hard requirement.** Determinism is what makes KPIs reproducible. Any simulator that can't pause / can't lock seeds / can't run at `sim_speedup=1` will pollute KPI reporting.

## Determinism contract (referenced throughout)

Every KPI run records:

- `code_revision` — single git SHA across the entire stack.
- `vla_model_hash` — SHA-256 of model weights.
- `policy_hash` — hash of the policy bundle.
- `random_seed` — uniform across simulator, ROS, VLA sampling.
- `sim_speedup` — must be `1.0` for any HIL or reported number.
- `topology` — `dev` | `hil` | `flight`.

A simulator that can't honour all six is unfit for L5.
