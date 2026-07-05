# World/perception candidates (L2)

This page is the long-form survey. Each candidate is scored against six axes:

> *photo-fidelity · ArduPilot integration · ROS 2 native · scriptability / stress-testing · hardware floor · license / support risk*

For a side-by-side scoring grid see the [Comparison matrix](comparison-matrix.md).

---

## A. Gazebo Harmonic + `ardupilot_gazebo` plugin — *locked, functional rail*

**Pros**

- First-class ArduPilot plugin maintained under [`ArduPilot/ardupilot_gazebo`](https://github.com/ArduPilot/ardupilot_gazebo). The lineage descends from Khancyr's fork, which is the canonical bridge used by ArduPilot CI itself.
- First-class ROS 2 (`ros_gz_bridge`, `gz_ros2_control`).
- Apache 2.0 / open source. No commercial-licensing risk.
- Headless mode → fast CI for Policy DSL / Prefix Compiler regression suites. Tens of episodes per minute on a laptop.
- Plug-in physics with PBR materials → "good enough" perception for many scenarios.
- All scripting in standard SDF/world XML — easy to author thousands of stress scenarios.
- Wind, GPS-noise, camera-noise plugins all exist out of the box.

**Cons**

- Photorealism gap vs Unreal-based sims. Won't stress real-world VLA perception robustness as hard as a photoreal rail can.
- Gazebo Harmonic's ROS 2 ecosystem is still maturing — some plugins from Gazebo Classic haven't ported.
- Limited dynamic-actor library compared to Unreal asset stores.

**Verdict.** Locked as the **functional rail**. Owns Policy DSL GeoFence regression, prefix-constraint sweep, Safety Shield unit/integration tests, functional stress scenarios. The one rail that would never be dropped under any plausible course-correction.

---

## B. Project AirSim (Microsoft) — *locked, perception rail*

**Pros**

- Photoreal Unreal Engine 5 perception → stresses VLA models on textures, lighting, shadows close to real-flight conditions.
- Service-oriented (TypeScript core, Python client). Fits a remote-orchestrator architecture cleanly.
- JSON scene-as-data → scriptable scenario generation.
- Active commercial development at Microsoft.

**Cons**

- **Weak ArduPilot support — the canonical risk for this grant.** Native vehicle controllers are PX4-only; AP is reachable only via a HIL bridge (HIL_GPS / HIL_SENSOR + actuator return). This is a **Q1 critical-path adapter** the project must build.
- Closed-source commercial license. Academic-use terms must be reviewed before stress-testing ramp.
- Unreal Engine 5 hardware floor: discrete NVIDIA GPU on desktop (RTX 3070 minimum, RTX 4090 ideal). Doesn't run on Orin.
- TypeScript core means cross-language stack-tracing is harder when the bridge misbehaves.
- Tied to one host (the desktop). Never on Orin.

**Risk.** If the HIL bridge is not closed-loop by the **2026-06-30 fallback gate**, the perception rail swaps to Colosseum (option C). Gazebo Harmonic remains the KPI-bearing rail in the interim.

---

## C. Colosseum (community AirSim fork) — *strongest fallback for B*

**Pros**

- Direct successor to original Microsoft AirSim (which was archived July 2022). MIT-licensed, fully open source.
- **Native ArduPilot vehicle controller already wired** — the AirSim ArduPilot HIL path is upstream code, not an adapter the team has to build.
- Unreal Engine 5 perception, comparable in fidelity to Project AirSim.
- No commercial-licensing risk.

**Cons**

- Smaller community than Project AirSim or original AirSim.
- Less Microsoft-driven feature velocity.
- C++ codebase — heavier modification cost than Project AirSim's TypeScript core.

**Verdict.** **The fallback** if Project AirSim's HIL bridge proves unworkable. The 2026-06-30 gate decision rule is documented in [Fallback gates](../04-risks-and-fallbacks/fallback-gates.md).

---

## D. Microsoft AirSim (original) — *rejected*

**Pros.** Mature, widely cited.

**Cons.**

- Archived July 2022. No upstream maintenance.
- Unreal Engine 4 only. The UE5 path forward *is* Colosseum.

**Verdict.** Rejected. Colosseum is the live successor with the same code lineage.

---

## E. NVIDIA Isaac Sim + Pegasus Simulator — *strongest road-not-taken*

**Pros**

- Photoreal Omniverse rendering, USD scenes, RTX path-traced, sensor-grade.
- Strong ROS 2 native bridge.
- [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator) (open source, IST Lisbon) provides PX4 + ArduPilot integration on top of Isaac Sim.
- Excellent for synthetic data / domain-randomization workflows.
- Free for individual / research use.
- **Aligns end-to-end with the chosen Jetson Orin inference hardware** — same vendor for sim and inference.

**Cons**

- High hardware floor: RTX 3070 minimum, 4090 strongly recommended; 64 GB system RAM.
- Pegasus's ArduPilot support is newer than Khancyr's Gazebo plugin — less battle-tested.
- Larger ecosystem learning curve than Gazebo (USD vs SDF, Omniverse vs vanilla ROS 2).
- Closed source under NVIDIA EULA (free but EULA'd). License review needed.

**Verdict.** Could replace Project AirSim if NVIDIA-vertical alignment is preferred. Worth a one-day spike before re-confirming the lock — it is the single largest counterfactual not surfaced in the original 2026-04-27 decision.

---

## F. Flightmare (ETH Zurich) — *rejected*

**Pros.** Lightweight, Unity-based, fast batched RL training; widely used in agile-flight literature.

**Cons.**

- PX4-focused; ArduPilot integration not first-class.
- Less active maintenance since 2022.
- Unity ecosystem (vs Unreal) creates an asset/import split with the rest of the photoreal world.

**Verdict.** Rejected. No ArduPilot path of record, and the racing/agile bias is a poor match for the safety-shield emphasis.

---

## G. VIVID — *third rail (demo overlay only)*

**Pros**

- The lab's own VR/sim environment. ACM MM 2018 Best Open-Source Software award. Institutional / political fit.
- VR / HMD support — the only candidate here that does. Useful for human-in-the-loop demos at ITRI demo days.
- Unreal-based → photoreal class.
- Institutional reuse pattern — VIVID is dropped into multiple grants over time.

**Cons**

- Originally a VR environment, not an autonomy-stack sim. ROS 2 / MAVLink integration would be NTUT-built.
- Smaller user base means less community testing.

**Verdict.** Use as **demo overlay** (mid-term and final demos), not a KPI-bearing rail. Adds no KPI cost and preserves institutional reuse.

---

## H. Webots (Cyberbotics) — *rejected*

**Pros.** Open source, lightweight, runs on laptops without a discrete GPU.

**Cons.** ArduPilot integration is community-maintained and less active than Gazebo's; perception fidelity below Gazebo Harmonic; smaller robotics ecosystem.

**Verdict.** Rejected. Gazebo Harmonic dominates it on every axis except install simplicity, and that's not a deciding factor.

---

## I. CoppeliaSim (formerly V-REP) — *rejected*

**Pros.** Mature robotics sim, Lua scripting.

**Cons.** No first-class ArduPilot integration; commercial license for non-academic use; not ROS 2 first-class.

**Verdict.** Rejected.

---

## J. X-Plane / RealFlight + ArduPilot SITL bridge — *rejected*

**Pros.** Very high-fidelity FDM (X-Plane especially); ArduPilot has a maintained bridge.

**Cons.** Commercial licenses; no scriptable perception API; no ROS 2; built for piloted flight, not autonomy/AI workloads.

**Verdict.** Rejected — wrong tool category for an AI-on-drone grant.

---

## K. Custom Unreal Engine 5 + ArduPilot HIL (DIY) — *rejected*

**Pros.** Maximum control; no licensing entanglement.

**Cons.** Order-of-magnitude more engineering than picking Colosseum, with no advantage that justifies it.

**Verdict.** Rejected — reinventing the wheel.

---

## L. CARLA (autonomous-driving sim) — *rejected*

**Pros.** Photoreal, well-funded, large ecosystem.

**Cons.** Ground-vehicle focus. UAV support is community / experimental, no ArduPilot path of record.

**Verdict.** Rejected — wrong vehicle class.
