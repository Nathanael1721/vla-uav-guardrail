# Inner-loop candidates (L1)

The inner-loop layer is where flight dynamics meet the autopilot firmware. For this grant the choice is effectively forced — ArduPilot SITL is required by the grant title and by the Safety Shield's KPI definitions — but the rejected alternatives are documented here so reviewers can confirm the reasoning.

## 1. ArduPilot SITL — *required, no realistic alternative*

**Pros**

- Bit-identical to flight firmware — the same C++ binary that runs on Pixhawk runs in SITL. Behavioural drift between simulation and real flight is approximately zero.
- Matches the AI Wings real-flight stack (the lab's existing ArduPilot + Android companion + cloud system from [IEEE Systems Journal 2022](../references.md)).
- Full GeoFence / `FENCE_ACTION` semantics. `RTL`, `Land`, `Loiter`, `SmartRTL`, `Brake` all behave exactly as on hardware — critical for measuring fail-safe trigger correctness.
- Stable MAVLink contract — the wire format is identical to flight, so MAVROS 2 / Mission Planner integration tests transfer.
- Headless mode runs in CI; tens of episodes per minute on a laptop.
- Same parameter file moves between SITL and the real airframe — operationally meaningful for the AI Wings deployment.

**Cons**

- Zero perception by itself. Provides only flight dynamics, sensor truth, and MAVLink — no camera, no images. Must be paired with an L2 world (Gazebo, Project AirSim, etc.).
- Default FDM is approximate — fine for autonomy testing, not for aerodynamics research.

**Verdict.** Required by the grant. Not optional. See [ArduPilot SITL docs](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html).

## 2. JSBSim (alternative FDM) — *rejected*

**Pros**

- High-fidelity flight dynamics; well-validated against real aircraft data.
- Already shipped as one of the FDM options inside Project AirSim's drone configurations.

**Cons**

- **Defeats AP SITL purity.** If JSBSim runs the dynamics, the AP firmware code path that fires breach actions is no longer the path being exercised. The Safety Shield's P0 escape-rate KPI becomes meaningless — the safety net being measured is not the safety net that flies.
- ArduPilot does have a JSBSim bridge, but it routes JSBSim's state into AP — ambiguity about which side actually owns the inner loop.

**Verdict.** Rejected. Safety Shield KPIs require AP SITL as the ultimate backstop, not as a passenger.

## 3. PX4 SITL — *rejected*

**Pros**

- Strong DDS support (`micro-XRCE-DDS`) — closer to ROS 2 native than ArduPilot's MAVROS-via-MAVLink path.
- Matches most academic VLA-on-drone literature (CognitiveDrone, RaceVLA, AutoFly, etc. typically prototype on PX4).
- Active development pace.

**Cons**

- Not ArduPilot. The grant title and AI Wings real-flight target are both AP. Parameter files, mission semantics, breach-action behaviour all differ.
- PX4-trained policies and PX4-tuned PIDs do not transfer to AP without significant retuning.

**Verdict.** Rejected — out of grant scope. Useful as a literature reference only.

## Why the field is so narrow at L1

The grant explicitly names ArduPilot. The KPI list explicitly names `FENCE_ACTION`-style behaviours. The lab's deployable platform (AI Wings) is ArduPilot. There is no scenario where switching the inner loop helps — every gain (DDS richness, FDM fidelity) costs the Safety Shield measurement validity.

L1 is settled. The interesting decisions are at [L2](world-perception.md).
