# Comparison matrix

Six-axis scoring of every L2 candidate. Tier symbols are deliberately coarse so the table reads on a projector without squinting.

> **Legend.** ✓✓ = strong / best-in-class. ✓ = adequate / acceptable. ✗ = weak or absent. — = not applicable.

## L2 (world / perception) candidates

| Candidate | Photo-fidelity | ArduPilot integration | ROS 2 native | Scriptability | Hardware floor | License risk |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| **A. Gazebo Harmonic** + `ardupilot_gazebo` | ✓ | ✓✓ | ✓✓ | ✓✓ | ✓✓ (laptop OK) | ✓✓ (Apache 2.0) |
| **B. Project AirSim** | ✓✓ | ✗ (HIL adapter required) | ✓ | ✓✓ | ✗ (RTX 3070+) | ✓ (commercial — review) |
| **C. Colosseum** | ✓✓ | ✓✓ (native AP HIL) | ✓ | ✓ | ✗ (RTX 3070+) | ✓✓ (MIT) |
| **D. Microsoft AirSim (original)** | ✓ | ✓ | ✓ | ✓ | ✗ | ✗ (archived 2022) |
| **E. Isaac Sim + Pegasus** | ✓✓ | ✓ (newer) | ✓✓ | ✓ | ✗ (RTX 4090 ideal) | ✓ (NVIDIA EULA) |
| **F. Flightmare** | ✓ | ✗ | ✓ | ✓ | ✓ | ✓ |
| **G. VIVID** | ✓✓ | ✗ (NTUT-built) | ✗ | ✓ | ✗ | ✓✓ (lab-owned) |
| **H. Webots** | ✗ | ✓ (community) | ✓ | ✓ | ✓✓ | ✓✓ |
| **I. CoppeliaSim** | ✓ | ✗ | ✗ | ✓ | ✓ | ✗ (commercial) |
| **J. X-Plane / RealFlight** | ✓✓ | ✓ | ✗ | ✗ | ✗ | ✗ (commercial) |
| **K. Custom UE5** | ✓✓ | — (build-it) | — | — | ✗ | ✓✓ |
| **L. CARLA** | ✓✓ | ✗ (no UAV path) | ✓ | ✓ | ✗ | ✓✓ |

## Reading the matrix

The **rails** that emerge from this scoring are:

1. **Functional/regression rail** — needs ✓✓ on ArduPilot integration, ROS 2 native, scriptability, and license risk; can accept ✓ on photo-fidelity. Only **Gazebo Harmonic** clears that bar.
2. **Perception/photoreal rail** — needs ✓✓ on photo-fidelity; everything else negotiable but ArduPilot integration cannot be ✗ without a remediation plan. Three candidates remain in contention: **Project AirSim** (with a Q1 HIL bridge as the remediation plan), **Colosseum** (native AP, identical fidelity, MIT-licensed — the strongest alternative), and **Isaac Sim + Pegasus** (NVIDIA-aligned, the largest counterfactual).
3. **Demo overlay** — VIVID is the only candidate with VR/HMD support. Not a rail; an overlay.

## Why two rails

A single candidate that maxes every column doesn't exist. Gazebo Harmonic wins on cost, speed, license, and ArduPilot integration but trails on photo-fidelity. The Unreal-based contenders win photo-fidelity but trail on cost, speed, and (in Project AirSim's case) ArduPilot integration. Picking both — one for KPI throughput, one for perception stress — is the only configuration that covers all four components without compromise.

For the architectural diagram showing how the two rails share a single ArduPilot SITL, see [Dual-rail decision](dual-rail.md).
