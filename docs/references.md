# References

Grouped by topic. URLs are provided where the project page is well-known and stable. For academic papers where a definitive URL was not on hand at the time of writing, the citation is given by title and a TODO marker — the user is expected to fill these in rather than have URLs fabricated.

## ArduPilot stack

- **ArduPilot SITL — Software in the Loop simulator.** ArduPilot Dev Wiki. <https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html>
- **`ardupilot_gazebo` plugin.** Official ArduPilot organisation, lineage from Khancyr's fork. <https://github.com/ArduPilot/ardupilot_gazebo>
- **ArduPilot ROS 2 (AP_DDS).** ArduPilot Dev Wiki. <https://ardupilot.org/dev/docs/ros2.html>
- **Mission Planner.** <https://ardupilot.org/planner/>
- **`mavlink-router`.** <https://github.com/mavlink-router/mavlink-router>
- **MAVROS** — MAVLink ↔ ROS 2 bridge. <https://github.com/mavlink/mavros>
- **MAVLink protocol.** <https://mavlink.io/>
- **ArduPilot GeoFence (parameters and FENCE_ACTION).** ArduPilot Dev Wiki. <https://ardupilot.org/copter/docs/common-geofencing-landing-page.html>
- **MAVLink message — `MISSION_ITEM_INT`.** <https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT>
- **MAVLink message — `SET_POSITION_TARGET_LOCAL_NED`.** <https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED>
- **MAVLink message — `SET_MODE`.** <https://mavlink.io/en/messages/common.html#SET_MODE>
- **MAVLink HIL messages — `HIL_GPS`, `HIL_SENSOR`** (used by Project AirSim ↔ ArduPilot HIL bridge). <https://mavlink.io/en/messages/common.html#HIL_GPS>

## Simulators surveyed

- **Gazebo Harmonic.** Open Source Robotics Foundation. <https://gazebosim.org/docs/harmonic>
- **Project AirSim.** Microsoft. <https://github.com/microsoft/ProjectAirSim>
- **Microsoft AirSim (archived July 2022).** <https://github.com/microsoft/AirSim>
- **Colosseum** — community AirSim fork (MIT-licensed successor). Codex Labs LLC. <https://github.com/CodexLabsLLC/Colosseum>
- **NVIDIA Isaac Sim.** <https://developer.nvidia.com/isaac-sim>
- **Pegasus Simulator** — PX4 + ArduPilot integration on Isaac Sim. IST Lisbon. <https://github.com/PegasusSimulator/PegasusSimulator>
- **Flightmare** — ETH Zurich Robotics and Perception Group. <https://github.com/uzh-rpg/flightmare>
- **Webots** — Cyberbotics. <https://cyberbotics.com/>
- **CoppeliaSim.** <https://www.coppeliarobotics.com/>
- **CARLA** — autonomous driving simulator. <https://carla.org/>

## VLA models

- **OpenVLA** — open-source vision-language-action model (the base model for CognitiveDrone). <https://openvla.github.io/>
- **CognitiveDrone** — OpenVLA-7B fine-tune for drones with 4-D `(vx, vy, vz, yaw_rate)` action space. *Citation TBD by the user — URL to fill in.*
- **BitVLA** — quantized / bit-efficient VLA. *Citation TBD by the user.*
- **AutoFly, RaceVLA** — academic VLA-on-drone literature. *Citations TBD by the user.*

## NTUT lab projects (referenced in the report)

- **VIVID** — VR/sim environment, ACM Multimedia 2018 Best Open-Source Software award. <https://github.com/kuanting/vivid>
- **AI Wings** — ArduPilot + Android companion + cloud, IEEE Systems Journal 2022. <https://github.com/kuanting/aiwings>

## Algorithms and standards used in the implementation plan

- **Douglas-Peucker polygon simplification.** Candidate algorithm for the [Prefix Compiler's](02-implementation/prefix-compiler.md) multi-scale geometry compression. Original 1973 paper: Douglas & Peucker, *Algorithms for the Reduction of the Number of Points Required to Represent a Digitized Line or its Caricature*.
- **R-tree spatial index.** Candidate index for the [Policy DSL](02-implementation/policy-dsl.md) ingest pipeline's runtime representation, for fast point-in-polygon and segment-intersection queries. Original 1984 paper: Guttman, *R-Trees: A Dynamic Index Structure for Spatial Searching*.
- **WGS84 / MSL / AGL.** World Geodetic System 1984; Mean Sea Level; Above Ground Level. ICAO standard for aviation altitude references.
- **NOTAM** (Notice to Airmen) — basis for the temporal "temporary ban" constraint class. <https://www.faa.gov/air_traffic/publications/atpubs/notam_html/>

## Documentation tools used to build this report

- **MkDocs.** <https://www.mkdocs.org/>
- **Material for MkDocs.** <https://squidfunk.github.io/mkdocs-material/>
- **PyMdown Extensions.** <https://facelessuser.github.io/pymdown-extensions/>
- **Mermaid.** <https://mermaid.js.org/>

## Grant / institutional

- **ITRI ICL** (Information & Communications Research Laboratories, Industrial Technology Research Institute). <https://www.itri.org.tw/>

## On URL fabrication

This report deliberately leaves "Citation TBD" markers rather than guessing URLs for academic papers. URL accuracy matters more than citation completeness for a working architecture document — a wrong URL is worse than no URL.
