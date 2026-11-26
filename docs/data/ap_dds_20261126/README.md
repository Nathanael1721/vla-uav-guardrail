# AP_DDS migration, stage 1 (ARCH-08), 26 Nov 2026

The grant makes AP_DDS "a Q3 optional migration, not a blocker" (Architecture
constraints p1). This folder is the first run of the AP_DDS path on ArduPilot:
docs/RUNBOOK-ap-dds.md steps 1-6, on the desktop (WSL Ubuntu 24.04, ROS 2
Jazzy, `dev` topology, so not KPI-grade: the grant reports KPIs from `hil`
only). The KPI rail stays on MAVROS 2, as the grant has it.

Build (step 1): `sitl/ap_dds/setup_ap_dds.sh` - ArduCopter SITL Copter-4.5.7
(2a3dc4b7, the rig's pin) in a separate checkout `~/ardupilot-dds`, configured
with `--enable-dds` (`AP_DDS_ENABLED=1`, 30 `AP_DDS_Client` symbols in the
binary), ArduPilot's Micro-XRCE-DDS-Gen, and a colcon workspace with the
micro-ROS agent and `ardupilot_msgs` from the same checkout. The first build
found four faults in the scripts, fixed on this branch: the waf option is
`--enable-dds` (lower case); the generator's Gradle 7.6 cannot run on Java 21;
CMake picked a user-level python3 without `catkin_pkg`; the agent needs the
checkout's `dds_xrce_profile.xml` (`-r`), else "Profile 'participant_profile'
not found" and no `/ap/*` topic.

Run: micro-ROS agent on UDP 2019, ROS domain 42, localhost discovery. Twelve
`/ap/*` topics on the graph: battery/battery0, clock, cmd_gps_pose, cmd_vel,
geopose/filtered, joy, navsat/navsat0, pose/filtered, tf, tf_static, time,
twist/filtered.

| Folder | Step | What it shows |
|---|---|---|
| `ap_dds_verify_01/ap_dds_verify.json` | 4 | AP_DDS topics at a rate; GUIDED and arm over `/ap/mode_switch` and `/ap/arm_motors`; take-off; 1.0 m/s east on `/ap/cmd_vel` for 6 s flown and tracked (measured 1.018 / -0.008 m/s, velocity_error 0.02 m/s, bearing error 0.1 deg); LAND. PASS |
| `ap_dds_verify_02/` (`ap_dds_verify.json`, `ap_dds_adapter_log.jsonl`) | 5 | The same check through `sitl/ap_dds_adapter_node.py` (`--via-adapter`): body `[1, 0, 0, 0]` on `/shield/setpoint` rotated by the adapter and flown along the heading (velocity_error 0.016 m/s, bearing error 0.1 deg). PASS; 70 adapter lines. `ap_receives_ned == ned` holds on every one, but only as a consistency check: `ap_receives_ned` is the adapter's own model of ArduPilot's ENU -> NED (`ap_dds_receives`, sitl/ap_dds_adapter_node.py), equal to `ned` by construction, so it is not evidence of what the autopilot received; the autopilot evidence is the measured velocity and track |
| `ap_dds_stage1_01/` (events, metrics, kpi, manifest, adapter log, report) | 6, diagnostic only | The default stub scenario flies straight at `nfz-square`: X1 Brake on tick 1, G3 Loiter held in GUIDED through `/ap/cmd_vel`, G5 RTL; 2 of 636 ticks flown - the same as this scenario on the MAVLink rail (hil_smoke_03). Delivery: 244 published, 244 adapter reports, 101 beats, no gap. The hold went to BRAKE at 24.6 s because one tick spent 296 ms writing its records to the OneDrive-synced drive the WSL run wrote to (`post_publish`), not because of the adapter. Code revision 9a5239c-dirty: not the evidence run |
| `ap_dds_stage1_02/` (run folder, replay bundle, report, trajectory) | 6, evidence | `sitl/ros2_shield_node.py` (state and mode changes through MAVROS on SERIAL2) -> `/shield/setpoint` -> `ap_dds_adapter_node.py` -> `/ap/cmd_vel`; no MAVLink setpoint adapter running. Signed bundle fase3-sim-demo v0.1.4 (signature verified), route `--target=-30,30`. 113 of 113 ticks flown, target reached at 11.3 s, LAND; P0 escapes 0, NFZ 0 s, altitude 0 s; 109 kinematic repairs (6 m/s requested, capped to 4 m/s); delivery 113 published / 113 reports / 48 beats, no gap, no shortfall; 113 adapter lines (`ap_receives_ned == ned` on every one, the same consistency check, equal by construction; the autopilot evidence is the flown trajectory and state velocity tracking the emitted command). Code revision 21493e67f945 (clean) |

What stage 1 does not move: the Shield still reads state (pose, velocity,
mode, armed, home, parameters) from MAVROS, because ArduCopter 4.5.7's DDS
client publishes no mode, armed or landed state and no parameter service
(the coverage table in docs/RUNBOOK-ap-dds.md). That is the grant's "uneven
coverage" and stage 2 of the runbook.

The delivery contract is also checked without ROS:
`tests/test_ap_dds_adapter.py`
`test_the_shields_delivery_monitor_accepts_the_adapters_reports`.

Local paths in the stage1_01 records are written `<repo>`.
