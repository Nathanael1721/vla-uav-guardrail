# Project AirSim flown by ArduPilot, with Mission Planner attached

**Date:** 2026-10-06, revised 2026-10-07 after review, and 2026-10-08 after the first live runs.
**Status:** **flown on this desktop on 2026-10-08.** Gate G1 passed (3 of 3 runs, `demo/out/g1_20261008_223259`), and a 120 s red-car follow flew through the Shield and the escalation FSM on this rail (`demo/out/pas_ap_20261008_224440`) and showed three defects that were open when it flew. The follow commanded ArduPilot over pymavlink, so it is none of the grant's topologies; the MAVROS 2 paths (`dev` in WSL, `hil` on the Orin) have not been run. Numbers, the fixes the runs needed, and the defects are in "Measured 2026-10-08" below.
**Cards:** ARCH-15 (bridge, gate G1), ARCH-02 (ArduPilot is the camera rail's autopilot), ARCH-18 (perception-rail integration), WP4-16 (city flights on ArduPilot), ARCH-20 (plumbing for a real VLA on ArduPilot), ARCH-25 (a camera on the ArduPilot rail).
**Code:** `demo/pas_ardupilot/`, `scripts/run_pas_ardupilot.ps1`, `tests/test_pas_ardupilot.py`.
**Supersedes** the bridge premise of [DESIGN-hil-perception-bridge.md](DESIGN-hil-perception-bridge.md) (2026-09-07), whose options A and B assumed that a `HIL_GPS` / `HIL_SENSOR` bridge would have to be written. See "The PI's question" below.

## The PI's question, answered

> Can Project AirSim be connected to ArduPilot and to Mission Planner?

**Yes. Demonstrated on this desktop on 2026-10-08: gate G1 passed on 3 of 3 runs, and the UDP stream mavlink-router sends to Mission Planner's port 14550 carried position and attitude at 19.0 Hz through a 120 s flight.** That stream was measured with a listener standing in for Mission Planner on its port (`sitl/gcs_listen.py`), which shows what Mission Planner receives, not its display.

The IAMAI fork of Project AirSim ships an ArduPilot flight controller (`"controller": {"type": "ardupilot-api"}`, class `ArduCopterApi`). With it, Project AirSim stops flying the vehicle itself. It sends its sensor readings to ArduPilot SITL and applies the motor commands ArduPilot sends back. ArduPilot upstream supports this interface (`sim_vehicle.py -v ArduCopter -f airsim-copter`).

**Mission Planner never talks to Project AirSim.** It talks MAVLink to ArduPilot through `mavlink-router`, as the grant specifies ("Mission Planner via mavlink-router fan-out, parallel to MAVROS"). Because ArduPilot flies the simulated vehicle, Mission Planner shows that flight: position, attitude, mode, mission and fence. A real aircraft is connected to Mission Planner the same way.

This changes the premise of the earlier scoping. The reference design (`dual-rail.md`) and `DESIGN-hil-perception-bridge.md` both assumed Project AirSim had no ArduPilot controller, so a `HIL_GPS` / `HIL_SENSOR` bridge had to be written. The binaries this project already runs contain one, so there is no bridge to write. What remains is to wire it correctly across WSL2, and to prove that it closes the loop. Gate G1 asks for exactly that proof.

## What runs where

```
 Windows                                               WSL2 (Ubuntu 24.04)
 ------------------------------------------------      ----------------------------------------
 Unreal Engine + Project AirSim (CityLife_Day)
   robot "Drone1", controller ardupilot-api
     every physics step (5 ms): one JSON line --UDP 9003-->  ArduPilot SITL Copter-4.5.7 (pinned)
       {timestamp, imu, pose, gps, velocity}                   --model airsim-copter, -w
     binds <local-host-ip>:9002  <--UDP 9002-- 11 x PWM ---    --home = scene home-geo-point
   FrontCamera / Chase streams (TCP 8989/8990)                  SERIAL0 tcp:5760
          |                                                          |
          v                                                     mavlink-router
 perception node (one process)                                  TCP server 5790 ---------+
   camera -> OWL-ViT -> servo pilot -> Shield + FSM -MAVLink->   UDP -> Windows:14550     |
   (vla-real env; pymavlink link today)          (TCP 5790)      UDP -> 127.0.0.1:14555 (MAVROS 2)
                                                                 UDP -> Orin:14555 (hil)  |
 Mission Planner  <-- UDP 14550 (or TCP <WSL address>:5790) ------------------------------+
```

`mavlink-routerd` is built from source (2362c62) into `~/.local` in WSL on this desktop (2026-10-08). Where it is not installed, a MAVProxy fan-out stands in. It is announced on screen and in `endpoints.json` as "not the grant's router". A MAVProxy TCP output serves one client, so under the fallback the node keeps TCP 5790 and Mission Planner's TCP port is **5791**. MAVProxy also runs with `--streamrate=-1`, so it does not rewrite the stream rates set in the param file.

## The data path, and how it was verified without running it

| Fact | Evidence (read 2026-10-06) |
|---|---|
| The CityLife project's own plugin has the ArduPilot controller | `PASBlocks/Plugins/ProjectAirSim/SimLibs/multirotor_api/include/arducopter_api.hpp` (identical to the Linux plugin's) and the `ArduCopterApi` symbols in `multirotor_api.lib` |
| The settings keys the binary reads | strings in `multirotor_api.lib`: `ardupilot-ip`, `ardupilot-udp-port`, `local-host-ip`, `local-host-udp-port`, `use-sensor-distance` |
| The sensor frame it sends | strings in the same library: `"timestamp"`, `"imu": {"angular_velocity", "linear_acceleration"}`, `"pose": {roll, pitch, yaw}`, `"gps": {lat, lon, alt}`, `"velocity": {"world_linear_velocity"}`, `"rng": {"distances"}` |
| ArduPilot parses exactly that | `libraries/SITL/SIM_AirSim.cpp` keytable at the pinned Copter-4.5.7 (read in WSL, read-only) |
| Ports | ArduPilot binds `0.0.0.0:9003` and sends 11 PWM values to `--sim-address:9002` (`SIM_IN_PORT` / `SIM_OUT_PORT` in `SITL_cmdline.cpp`); no non-default port is needed |
| `-f airsim-copter` defaults | `vehicleinfo.py`: `copter.parm` + `airsim-quadX.parm`; our `citylife-quad.param` is loaded after both |
| The rendered configs are acceptable to the simulator | Project AirSim's own client loader and JSON schema (`load_scene_config_as_dict`) accept them offline, and reject a broken controller (test) |

Barometer and compass are not in the JSON frame. ArduPilot's SITL synthesises them from the GPS altitude and the attitude it receives. So ArduPilot's EKF runs on Project AirSim's IMU, attitude, GPS and velocity, plus a SITL barometer and compass derived from them.

One upstream inconsistency was found. IAMAI's example config and the client schema spell the distance-sensor key `use-distance-sensor`, but the binary reads `use-sensor-distance`. Our airframe has no distance sensor, so the key is omitted.

## WSL2 networking: which address each side must use

This PC runs WSL2 in **NAT** mode (`~/.wslconfig` leaves `networkingMode` commented out). Under NAT, Windows forwards **TCP** from its localhost into WSL, but **not UDP**. Both simulator links are UDP, so loopback silently drops them. `rail.plan_network()` refuses loopback under NAT for that reason.

| Address | NAT (this PC) | Mirrored |
|---|---|---|
| Project AirSim sends sensor frames to (`ardupilot-ip`) | WSL eth0 (`wsl hostname -I`) | 127.0.0.1 |
| Project AirSim binds for PWM (`local-host-ip`) | Windows end of the WSL switch (WSL's default gateway) | 127.0.0.1 |
| ArduPilot `--sim-address` | the same gateway address | 127.0.0.1 |
| mavlink-router -> Mission Planner UDP 14550 | the same gateway address | 127.0.0.1 |
| Windows process -> router TCP 5790 | WSL eth0. TCP is forwarded from 127.0.0.1, but not to mavlink-routerd's IPv6 `[::]` listener (measured 2026-10-08: refused on 127.0.0.1, accepted on the WSL address); an IPv4 listener such as the MAVProxy fallback works on both (`NetworkPlan.router_tcp_host`) | 127.0.0.1 |

The WSL address changes on every WSL restart. So the robot config in `demo/pas_ardupilot/` is a template, and each run gets its own rendered copy, with that day's addresses, in `demo/out/<tag>/pas_config/`. The client sends these configs to the simulator, so the files kept with the run are exactly what the simulator received.

**Firewall.** Under NAT, Windows Defender Firewall can drop inbound UDP 9002 (PWM) and UDP 14550 (Mission Planner) arriving on the WSL adapter. `run_pas_ardupilot.ps1 -Step check` looks for an allow rule and prints the `New-NetFirewallRule` command for an administrator to run. The check is read-only: the launcher changes no firewall setting. It looks for port rules and lists program rules (for example one Windows created for `UnrealEditor.exe`) separately, because whether a program rule covers the port cannot be told from the rule list. If UDP 14550 is blocked, Mission Planner can connect by TCP instead (5790, or 5791 under the MAVProxy fallback), which needs no rule.

**Order.** Reloading a scene resets the simulator clock. ArduPilot's AirSim backend adds `new timestamp - previous timestamp` to its own clock, with no guard against a negative step. So SITL must start **after** the scene loads. `perception_node.py` and `g1_check.py` do this themselves with `--start-sitl`. A test checks that the node starts the camera, then SITL, then stops SITL. For the hil sequence, `-Step scene` loads the scene, then `-Step sitl` starts ArduPilot, and the node on the Orin only attaches.

**The TCP connect is retried.** The router's TCP port opens only after WSL starts, the pin check runs (18.5 s measured), SITL starts and the router follows. pymavlink 2.4.49 tries a refused TCP connect three times, about a second apart, and then raises. The first version made a single call, so every `--start-sitl` run would have crashed before its heartbeat timeout began. `PymavlinkLink.connect()` now retries the connect itself until the same deadline as the heartbeat wait.

## Best-practice choices

| Choice | Why |
|---|---|
| Native `ardupilot-api` controller, not a hand-written HIL_GPS/HIL_SENSOR bridge | It ships in the binary we already run. ArduPilot upstream supports the backend (`-f airsim-copter`, the same backend Colosseum uses). Nothing new has to be written or maintained |
| Run the pinned `arducopter` binary directly; `sim_vehicle.py` given as the equivalent | The pinned build flies (`sitl/setup_sitl.sh --verify` is recorded at start-up), and no MAVProxy sits on the MAVLink path |
| `-w` at every SITL start | Stored parameters (`eeprom.bin`) would otherwise override `--defaults`, so a value edited in Mission Planner in one session would silently fly in the next G1 repeat. `sitl/start_sitl.sh` wipes by default for the same reason. `SITL_WIPE=0` keeps them |
| `mavlink-router` for the fan-out, its TCP server on 5790 | The grant names it; 5760 (its default) is already SITL's SERIAL0 |
| ArduPilot geofence on (`FENCE_ALT_MAX 20`, 400 m circle, RTL) | The grant: "the autopilot's own GeoFence / FENCE_ACTION is the ultimate backstop"; the Shield acts first (policy ceiling 14 m). The radius is sized from flown data: over the 26 stored CityLife follow trajectories, the largest horizontal distance from the start point is 253.7 m (`citylife_redcar_final2`). The first version's 250 m circle, justified by an unsourced "190 m", would have fired RTL in the middle of such a follow |
| `WPNAV_SPEED 500`, `RTL_ALT 1200` | `airsim-quadX.parm` sets 20 m/s and 25 m; the CityLife policy caps 5 m/s and 6-14 m |
| Rate-loop tuning copied from IAMAI's `project-airsim-quad.param` | Our airframe is identical to theirs (links, mass, rotors; a test checks) |
| MAVROS output as `mavros_msgs/PositionTarget` on `/mavros/setpoint_raw/local` | The grant's interface table names PositionTarget, not Twist |
| Position from ArduPilot's EKF (GLOBAL_POSITION_INT), projected about the policy origin or the scene home | On the real drone there is no simulator, and the node must not change |
| One heartbeat filter | mavlink-router also delivers Mission Planner's heartbeat. Latching it would address every command to the ground station |

## Where this rail differs from the grant's wording, and why

The PI asked to conform rather than ask for waivers. Two numbers and one placement on this rail differ from the grant's Architecture constraints text, so they are recorded here rather than left implicit.

| Grant text | This rail | Why |
|---|---|---|
| Interface table: MAVROS 2 -> ArduPilot over "UDP 14550" | MAVROS on UDP 14555, Mission Planner on UDP 14550 | Both cannot hold 14550 on one host. 14550 is Mission Planner's default listen port, and 14555 for MAVROS is the reference docker-compose's layout and `sitl/mavlink-router/main.conf`'s, so every rail in this repository uses the same two numbers |
| hil: "Desktop runs sims + MAVROS + GCS; Jetson Orin runs VLA + Shield" | MAVROS runs on the Orin, next to the Shield | The reference repository's Topologies page puts MAVROS on the Orin in hil, and `docs/DESIGN-topologies.md` chose that layout for the whole project. hil to flight is then an `fcu_url` change only, and the Shield-to-MAVROS hop stays inside one host. `guardrail.manifest`'s hil evidence check also requires it today (a non-loopback `fcu_url` remote) |

Running the grant's wording instead is possible: MAVROS in WSL with mirrored networking, Cyclone DDS unicast peers across the cable (`deploy/dds/cyclonedds.xml`), and the same ROS distribution on both sides. It would also need a change to `guardrail/manifest.py`, outside this unit: hil evidence would have to accept a desktop-side MAVROS and take the Orin link from DDS instead of from `fcu_url`.

## The perception node

`demo/pas_ardupilot/perception_node.py` is one process. On each 10 Hz tick it does the following:

1. It reads ArduPilot's EKF state, and the mode and arm state ArduPilot reports.
2. It feeds the camera frame to `follow_vlm.Grounder`.
3. It turns the box into a body-frame 4-D action with `follow_vlm.servo()`.
4. It rotates that action once into the scene frame, at the Shield's input, using ArduPilot's heading. The rotation is `guardrail.frames.from_body`, the project's one body-to-world crossing, which `tests/test_frame_contract.py` pins against the reference implementation.
5. It runs `Shield.filter()` with the grant's lookahead: 5 s of predicted trajectory at 10 Hz, 50 future poses (`guardrail.shield.LOOKAHEAD_S` / `LOOKAHEAD_DT_S`, as both SITL rails fly). The Shield is built without an escalation FSM of its own (`guardrail.replay.rail_shield`).
6. With the Shield on, it steps the grant's escalation FSM (`guardrail.fsm.EscalationFSM`: Brake, Loiter, RTL, Land) on the Shield's decision, as `sitl/run_sitl_demo.py` and `sitl/ros2_shield_node.py` do. Mode requests go to ArduPilot without blocking the loop. This is the only FSM: the audit log records its verdict (`audit_tick`), the same one the flight log does. With the Shield's own FSM also running, the two logs would describe two different state machines.
7. It sends only the Shield's output, or the FSM's brake, to ArduPilot. When the FSM hands the aircraft to an autopilot mode, it sends nothing. What was flown, and the Shield's check on it, follow the rule both SITL rails share (`guardrail.replay.flown_fields`): a tick on which nothing was sent cannot count as an escape.

Stand-off rules get the subject's position from depth (or apparent width) and bearing, never from ground truth. Every row carries the fields the KPIs need (`unsafe`, `unsafe_rules`, `policy_hash`, `generation`, `shield_ms`, the FSM state). The episode directory is written through `guardrail.replay.EpisodeRecord`, so time to safe is measurable and the replay bundle carries every policy generation. A follow mission has no compiler target, so its CSP is recorded as "not compiled" (an event), not skipped in silence.

Every row, `metrics.json` and `kpi.json` carry the episode id (`EpisodeRecord.episode_id`), and the episode ends with exactly one `mission_end` (`EpisodeRecord.end()`), so `guardrail.replay` binds the files to one episode by id (`log_binding: episode`) rather than by file times. If the link or anything else raises mid-flight, the `mission_end` names the crash and no KPI table is written. Until 2026-10-07 the rows carried no id, and the replay bundle of a stand-in run written that day wrongly called its log one "written before 2026-10-07".

**What the autopilot does on its own is recorded.** `metrics.json` carries ArduPilot's mode and arm changes, its STATUSTEXT messages, its FENCE_STATUS breach count, and the node's own mode requests. A breach count that was never received is `None`, not 0. MAVROS 2 has no FENCE_STATUS topic, so on that link a fence action shows up only as a mode change. If ArduPilot leaves GUIDED without the node asking (its fence fired RTL, a failsafe, a pilot in Mission Planner), the node stops commanding at once, records the takeover, and ends the episode once the aircraft has landed, or after 25 s. It does not override the autopilot's mode with a LAND.

After take-off the node also reads the GeoFence parameters back from the autopilot (`FENCE_ENABLE`, `FENCE_TYPE`, `FENCE_ACTION`, `FENCE_ALT_MAX`, `FENCE_RADIUS`, `FENCE_MARGIN`, `AVOID_ENABLE`), records them in `events.jsonl` and `metrics.json`, and judges `FENCE_ALT_MAX` against the policy's ceiling by the SITL rails' rule (`fence_alt_plan`: ceiling plus 5 m). A fence lower than that would fire RTL on a flight the Shield considers legal, and the record says so. A value the autopilot did not answer stays `None`. Unlike the SITL rails, this node never raises `FENCE_ALT_MAX` itself: the param file's 20 m fits the CityLife policy's 14 m ceiling, and a policy that needs more must be given a param file to match.

**The VLA-to-Shield hop is in-process** on this node. With `--link mavros`, record copies of the raw and filtered actions go on `/pas/raw_action_4d` and `/pas/safe_action_4d`. They are deliberately not on `/vla/action_4d`, the input of `sitl/ros2_shield_node.py`: if that chain were up on the same ROS graph, a second Shield would filter the action and two Shields would command one aircraft.

**Not reused yet.** `follow_vlm.fly()`'s target estimator, identity tiers, re-acquisition, trail following, fence guard and landing-site choice live inside that function. Reusing them needs a refactor of `follow_vlm.py`. Until then the node tracks with the servo law on the detector's box. Its runs carry no tracking score: `frac_within_30m` is `None`, never 0, because no subject ground truth is read.

## Topology and KPI grade: what this rail is, and how it moves to the drone

| Where the node runs | Link | Grant label the evidence supports (`topology.classify`) |
|---|---|---|
| Windows desktop, `--link pymavlink` (runnable today) | pymavlink | none: the grant's safety path is MAVROS 2. Stamped `projectairsim-ardupilot-pymavlink` |
| WSL on the desktop, `--link mavros --scene-mode attach` | MAVROS 2 in WSL (`fcu_url udp://:14555@`) | `dev` |
| Jetson Orin, simulator on the desktop, `--scene-mode attach --sim-host <desktop>` | MAVROS 2 on the Orin (`fcu_url udp://:14555@<desktop>:14555`) | `hil` |
| Jetson Orin on the drone, `--camera ros2` | MAVROS 2 to the flight controller's serial port | `flight` |

The label comes from evidence, judged by the same `guardrail.manifest.check_topology_evidence` that the ROS 2 Shield node uses. The evidence is the MAVROS chain, the autopilot's own `AUTOPILOT_VERSION`, SITL or hardware (from `SIM_SPEEDUP` / `SYSID_THISMAV`), the Orin's device tree and L4T release, and the MAVLink peer. This module adds two facts only it can know: whose controller flew the simulated vehicle, and whether the simulator ran on the node's own desktop.

**The grant label is offered only to a qualified rail.** For `dev` or `hil` the node offers the label to `build_manifest` only when both of these hold:

- gate G1 has passed (`--g1-verdict`; the launcher passes the newest `demo\out\g1_*\g1_verdict.json`);
- ArduPilot ran at real time. This is measured from ArduPilot's own clock (SYSTEM_TIME, or MAVROS's `time_reference`) against the wall clock, and must fall within 0.95-1.05.

Otherwise the node stamps the rail's own label and records why. The scene only declares `sim_speedup` 1.0; the measured factor is what shows whether the run kept it. Either failure also makes `kpi.json` say "not KPI-grade", with the reason. On the MAVROS path the clock comes from `/mavros/time_reference` (SYSTEM_TIME's UNIX time, which on SITL advances with simulated time). That has not been checked on a live MAVROS.

**Moving to the drone needs no code change.** Only `--camera ros2 --image-topic ...`, the MAVROS `fcu_url`, and the policy (WGS84-anchored, `origin:`) change. The node reads position from the EKF, which the drone has too.

**What `manifest.py` needs (outside this unit).** `build_manifest()` refuses `dev`, `hil` and `flight` together with any Project AirSim scene file. That guard was written when every scene meant simple_flight, and its message still says "A run with a Project AirSim scene is 'projectairsim-single-host'." On this rail that is no longer true. So the node offers the grant label (when qualified), and stamps its own label when the manifest refuses, keeping the refusal in `metrics.json`. Once `build_manifest()` accepts a scene whose robot config uses controller `ardupilot-api` (read from the file the simulator was given), and still refuses a simple_flight scene, the same node stamps `dev` or `hil` with no change here.

## Gate G1, measured: the four conditions

The results of the runs of 2026-10-08 are in the next section.

`demo/pas_ardupilot/g1_check.py` turns the gate's four conditions into evidence files. Each condition answers `True`, `False` or `None` (not measured), and `None` never counts as a pass.

1. **The EKF converges on the simulator's position.** The 95th percentile of the 3-D error between ArduPilot's EKF (GLOBAL_POSITION_INT) and Project AirSim's ground-truth geo position, paired in time, must be within 1 m. **Null:** the same EKF track against a *frozen* truth. The run must beat the null by half and travel at least 10 m. Otherwise it cannot tell a working bridge from a parked vehicle under a converged EKF. **Transport note:** the frames travel as Project AirSim's JSON sensor frames, not as MAVLink `HIL_GPS` / `HIL_SENSOR`. What the condition measures is unchanged.
2. **ArduPilot's PWM moves the simulated vehicle.** **Null:** while disarmed, the vehicle moves under 0.3 m. After arming, the motor PWM rises above 1100 and the vehicle then climbs at least 2 m within 30 s, lifting off *after* the PWM rise. Chase-camera frames before and during the climb are saved as the viewport evidence.
3. **A square mission in AUTO** (10 m side, 10 m up by default, uploaded through the MAVLink mission protocol; the passing runs of 2026-10-08 fly it 15 m up, because at 10 m it crosses a street light). The conditions:
   - AUTO is observed, and take-off and every corner report `MISSION_ITEM_REACHED`;
   - the vehicle lands, and ground truth passes within 2 m of every corner. The corner check is what fails a vehicle that never left the pad, which would "land within 2 m" trivially;
   - landing is within 2 m of take-off;
   - there are zero airborne collisions and zero fence breaches.
4. **Repeatability:** at least three runs with one seed, each passing conditions 1-3, with landing points and corner passes each spread by no more than 2 m. The seed fixes nothing random on our side of a scripted mission. The simulator's IMU noise is seeded inside Project AirSim; the record says so.

**A crashed run is a failed run, never a missing one.** Each `run` first logs its attempt (`g1_attempts.jsonl`), then writes `g1_run.json` from a `finally` block, with an `error` field if the run did not complete. `summarize --runs N` expects runs 1..N of one invocation. A run that started and left no record, crashed, or carries another attempt's record counts as failing conditions 1-3. A run folder left over from an earlier use of the tag fails condition 4. The launcher uses a fresh `g1_<timestamp>` tag and refuses to reuse one. The first version simply globbed the records that existed. Three passing runs plus one crashed run therefore read as PASSED (shown in review, and now a test).

## Measured 2026-10-08

All runs on one desktop: Windows 11 with Project AirSim (CityLife_Day in the Unreal editor), WSL2 in NAT mode with ArduPilot SITL `ArduCopter V4.5.7 (2a3dc4b7)` and mavlink-routerd built from source (2362c62). None of it is KPI-grade: the G1 runs carry no Shield, and the follow run commanded ArduPilot over pymavlink (stamped `projectairsim-ardupilot-pymavlink`, none of `dev`, `hil` or `flight`) from a dirty tree (`b23b5fd331b8-dirty`).

### Gate G1: passed

`demo/out/g1_20261008_223259/g1_verdict.json`, status `passed`: 3 runs expected, 3 completed, seed 20261006. The square flew 10 m sides at 15 m (`-AltM 15`), not at the 10 m default (fix 4 below).

| | Run 1 | Run 2 | Run 3 | Limit, null |
|---|---|---|---|---|
| C1 EKF vs truth, p95 3-D error | 0.814 m | 0.819 m | 0.815 m | at most 1.0 m. Null, the same EKF track against a frozen truth: 18.94 / 18.96 / 18.97 m |
| C1 median / max | 0.479 / 0.866 m | 0.462 / 0.874 m | 0.502 / 0.875 m | |
| C1 pairs, truth path length | 1288, 65.75 m | 1289, 65.98 m | 1268, 65.96 m | at least 10 m travelled |
| C2 motion at rest (disarmed, PWM 1000) | 0.0 m | 0.0 m | 0.0 m | null: under 0.3 m |
| C2 PWM rise, then lift-off | 87.99 s, 91.25 s | 88.40 s, 91.89 s | 87.58 s, 90.81 s | lift-off after the PWM rise |
| C2 climb | 14.49 m | 14.50 m | 14.49 m | at least 2 m within 30 s |
| C3 MISSION_ITEM_REACHED | items 1-5 | items 1-5 | items 1-5 | every item (7 uploaded, 5 navigation items) |
| C3 nearest pass per corner | 0.77 / 0.98 / 0.97 / 0.10 m | 0.76 / 0.98 / 0.86 / 0.08 m | 0.78 / 0.98 / 0.92 / 0.01 m | at most 2 m. This is what fails a vehicle that never left the pad |
| C3 landing error | 0.178 m | 0.180 m | 0.055 m | at most 2 m (a parked vehicle scores 0 here) |
| C3 airborne collisions, fence breaches | 0, 0 | 0, 0 | 0, 0 | 0, 0 |
| ArduPilot real-time factor | 0.966 | 0.958 | 0.967 | 0.95-1.05 for a run to be offered a grant label |

C4, repeatability: landing points spread 0.13 m, corner passes 0.022 / 0.079 / 0.134 / 0.115 m (limit 2 m). The seed fixes nothing random on our side of a scripted mission; the simulator's IMU noise is seeded inside Project AirSim.

`docs/img/g1_pas_ardupilot_airborne.png` is byte-identical to run 1's `chase_airborne.png` (sha256 `fab3bf99f9186492...`).

What G1 settles: the native `ardupilot-api` controller closes the loop on this desktop near real time; ArduPilot's EKF converges on Project AirSim's IMU, attitude, GPS and velocity to well inside 1 m; the rate-loop tuning copied from IAMAI flies an AUTO square in CityLife. What it does not touch: MAVROS 2 on this rail, the Orin, a VLA, or the Shield.

### The four rail fixes (each found on the rail on 2026-10-08)

1. **A Windows client could not reach the router.** mavlink-routerd listens on `[::]`, and WSL's NAT localhost forwarding does not carry an IPv6 listener to Windows' 127.0.0.1 (refused on 127.0.0.1, accepted on the WSL address). Under NAT the node and G1 now connect to WSL's own address (`NetworkPlan.router_tcp_host`, `router_tcp_url`). The router-address test in `tests/test_pas_ardupilot.py` fails on the old code.
2. **The next scene load never returned after a G1 run's ArduPilot stopped.** The steppable clock waits for ArduPilot's PWM. `run_pas_ardupilot.ps1 -Step g1` now restarts the simulator for each run.
3. **ArduPilot refused to arm: "Main loop slow (222Hz < 300Hz)".** CityLife delivers about 222 sensor frames a second against the 300 Hz loop `airsim-quadX.parm` sets, and 3 ms steps at that frame rate ran the clock at about 0.67 of real time. The scene steps 5 ms (`scene_citylife_ardupilot.jsonc`) and `citylife-quad.param` sets `SCHED_LOOP_RATE 200`.
4. **The G1 square at 10 m flew through a street light** (48-60 airborne collisions a run). The passing runs fly it at 15 m. The default is still 10 m in both `g1_check.py` (`--alt-m`) and `run_pas_ardupilot.ps1` (`-AltM`), so `-Step g1` without `-AltM 15` flies the failing square again.

A fifth change is tooling: `start_ardupilot.sh` and `sitl/start_router.sh` find a mavlink-routerd installed without sudo in `~/.local/bin`, and the runner no longer exits 1 when nothing is left to stop.

### The follow run: a red car for 120 s

`demo/out/pas_ap_20261008_224440`, launched as `-Step node -Object "a red car" -Seconds 120`. OWL-ViT base-patch32; cruise 8 m; policy `policies/follow_car_citylife.yaml` (`sha256:4f7ebad6...`, loaded from YAML, unsigned): stand-off 5 m to any subject and 10 m to a pedestrian, building clearance 3 m and altitude band 6-14 m (all P0), speed 5 m/s, climb 2 m/s and yaw 45 deg/s (P1). Shield lookahead 5 s at 0.1 s (50 poses); the grant's FSM defaults; pymavlink to `tcp:<WSL address>:5790`; the G1 verdict above attached, so the rail counted as qualified.

| Measure | Value | Reference or null |
|---|---|---|
| Ticks, mission time, loop rate | 1101 in 120.14 s, 9.16 Hz | 10 Hz |
| Detector rate | 3.75 Hz | the grant's 10 Hz; null 0 |
| Ticks flown by the node / handed to ArduPilot by the FSM (nothing sent) | 937 / 164 | |
| Ticks tracking the car / searching | 269 / 832 | |
| Shield interventions, brakes | 85, 0 | |
| ArduPilot real-time factor | 0.951 | 0.95-1.05 (inside by 0.001) |
| EKF vs simulator, horizontal p95 (221 pairs) | 0.627 m | |
| Shield time per tick, median / p99 / max | 0.32 / 2.43 / 104.2 ms | the grant's budgets are 5 ms per query and 100 ms per tick; 6 ticks took over 5 ms in all, 1 over 100 ms |
| GeoFence breaches (FENCE_STATUS received) | 0 | fence read back: ENABLE 1, TYPE 3, ACTION 1 (RTL), ALT_MAX 20 m, RADIUS 400 m, MARGIN 2 m |
| Mission Planner's UDP 14550 | GLOBAL_POSITION_INT and ATTITUDE at 19.0 Hz, largest gap 0.1 s | `sitl/gcs_listen.py check` over the mission window. The capture is not archived with the run (see below) |
| P0 escape rate | 0.0053 (5 of 937 flown ticks) | target 0. No Shield-off control has been flown on this rail, so there is no null yet |
| Repair success (re-check and theta) | 0.776 (52 of 67) | |
| Time to safe, mean / max | 2.81 / 6.42 s over 12 episodes | |
| Time below the policy's 6 m floor | 27.9 s (279 ticks) | |
| Mission success | false: P0 escape, altitude dwell, unsafe position entered | |
| Simulator contacts | 1 | take-off and landing included; the record has no time for it |

The 19.0 Hz row comes from `gcs_node.json`, a 300 s `sitl/gcs_listen.py listen` capture taken while the follow flew (21816 packets from one source, sha256 `2b2bb6a898ba75e5...`). It was saved in a session's temporary folder, not in `demo/out/pas_ap_20261008_224440/`, so the run folder alone cannot re-derive the row. Re-checked for this record with `python sitl/gcs_listen.py check gcs_node.json demo/out/pas_ap_20261008_224440/events.jsonl`: PASS, 2285 of each message in 120.1 s (19.0 Hz), largest gap 0.1 s. Until the capture is copied into the run folder as `gcs.json`, treat the row as resting on a file outside the repository.

`kpi.json`'s `failsafe_trigger_correctness` (0.881) is the P0-acted tick share under its legacy name, as `failsafe_trigger_correctness_is` says. It is not the grant's fail-safe trigger correctness and must not be quoted as such.

### Defects open when the follow flew

(a)-(c) are what the follow run showed. (d) was measured the day before by the stress harness's headless kinematic sweep (manifest topology `headless-kinematic`: no simulator and no ArduPilot; Loiter, RTL and Land are modelled as ideal position control, so the sinking LOITER of (c) is not in it). It is listed because it bears on the same escalation FSM, and a figure measured with a real autopilot's Loiter could be lower. This section records the run as flown at `b23b5fd331b8-dirty`. A fix made after it is recorded in `CHANGELOG.md` and, for a defect that was silent, in a `docs/FINDING-*.md` note; it does not change the numbers below, and only a new flight on this rail shows whether it worked. Until such a flight, these numbers are the rail's latest.

**(a) Five P0 stand-off escapes.** Ticks 857, 927, 928, 999 and 1100 sent a command that still violated `standoff-any` (5 m to the car, P0). On each, the aircraft was already inside the ring by the logged estimate, standing still would itself break the rule (`stop_illegal`), no repair operator converged, and the best-effort `ClearanceEscape` did not leave the ring within the tick. So the Shield flew its own best action, as designed when stopping is illegal, and the FSM stepped up.

What the log adds: on all 5 ticks the logged subject position equals the aircraft's own horizontal position (0.00-0.01 m apart), while on the neighbouring ticks that have an estimate it is 3.5-54.6 m away. `perception_node.subject_estimate()`, as it was at that revision, turns the depth range r into a horizontal range as `sqrt(max(0, r² - dh²))` with dh = altitude - 0.75 m (about 7 m here), so any range r ≤ dh puts the car directly under the aircraft instead of being rejected. On tick 857 the detection the tick carried (`det_seq` 324) was a 50 x 24 px box in the upper half of the 768 x 432 frame, which is not how a car under the aircraft looks to a forward camera. The depth range itself is not logged, so this is the likeliest reading, not a measurement. If it holds, there are two defects: a subject estimate that silently collapses onto the aircraft, and a Shield that cannot leave the ring in one tick once it is inside. Neither raised an error: the run reports five escapes and nothing about the estimate.

**(b) Brake to Loiter on stand-off repairs.** The FSM went Brake to Loiter 8 times. 7 were on `standoff-any`: on 5 of them the `StandoffHold` repair was larger than theta (3.63, 5.29, 8.08, 9.23 and 9.81 m lateral against 2.0 m; ticks 480, 750, 202, 1001, 282), and on 2 no operator converged (ticks 857 and 928, both escape ticks). The 8th, at tick 1100, was building clearance. The Normal to Brake edges before them carried hold repairs of 2.54-9.78 m. The subject estimate jumped just before several of them (59.6 m to 8.3 m at tick 201, 108.8 m to 15.4 m at tick 735, 48.5 m to 5.6 m at tick 841), and a 5 s forecast against a jumping subject asks for a large hold. This node tracks with the servo law on the raw detector box; `follow_vlm.fly()`'s target estimator is not reused here yet (see "The perception node").

**(c) LOITER sinks out of the altitude band.** ArduCopter LOITER with no RC input descends; the SITL rails showed it on 2026-10-07 ("LOITER descended from 14.2 m to the ground", `docs/DESIGN-ros2-interface.md`). Here the 7 Loiters before the last tick lasted 2.1-3.3 s each and lost altitude at 1.35-1.77 m/s from 7.6-8.0 m (down to 1.80-4.99 m when ArduPilot handed back), and every one was followed by a stretch below the policy's 6 m floor: 7 stretches, 279 ticks (27.9 s), lowest points 0.36-3.43 m. (The 8th Loiter began on the final tick.) The lowest, 0.36 m at tick 322, came in GUIDED after the second Loiter, with the node already commanding the 1.8 m/s climb limit. These are 279 of the run's 310 unsafe P0 ticks (the rest: 25 building clearance, 6 stand-off) and the `dwell:alt_violation_s` mission failure. The escalation meant to hold the aircraft safe takes it out of the band instead.

**(d) Fail-safe trigger correctness in the headless harness** was 0.747 (library), 0.726 (nightly) and 0.875 (smoke) against the grant's 0.99, and below the never-trigger null (CHANGELOG 2026-10-07, Retracted). Every stored sweep and nightly manifest names the topology `headless-kinematic`; the harness's ArduPilot path (`--backend sitl`) refuses every scenario with events (`sitl_refusal` in `experiments/sweep_scenarios.py`). It was not measured on this rail or on ArduPilot SITL.

## Re-flown 2026-10-12, with the fixes

The same follow (red car, 120 s, same policy and seed) flown with the 9-12 Oct fixes: `demo/out/pas_ap_20261012_033329`, pymavlink, same topology stamp, not KPI-grade.

| | 8 Oct | 12 Oct |
|---|---|---|
| ticks, loop rate | 1101, 9.16 Hz | 1098, 9.14 Hz |
| detector rate | 3.75 Hz | 4.26 Hz |
| ticks with the car tracked | 269 | 129 |
| nearest the car came (logged estimate) | 0.0 m (the range-0 placements) | 16.5 m |
| Shield interventions | 85 | 43 (yaw caps, building clearance) |
| P0 escape ticks | 5 | 0 |
| FSM | 27 edges, 8 Brake to Loiter | 6 edges, Normal to Brake 3 times and back; never Loiter |
| mode requests | 8 LOITER, 7 GUIDED, LAND | LAND |
| altitude | 0.36-8.15 m, 27.9 s below the 6 m floor | 7.99-8.15 m, 0 s below it |
| ArduPilot real-time factor, GeoFence breaches | 0.951, 0 | 0.962, 0 |

Against the defects above: (a) is not tested by this run, because the car never came near the 5 m ring; (b) and (c) did not recur, but no stand-off repair arose and the FSM never asked for a hold, so the BRAKE hold was not flown either; (d) is unchanged in kind (republished at 0.823, still headless). Only a run that brings the aircraft close to the car tests the range-0 fix and the hold on this rail.

Found on this run: 43 depth returns were refused and the car ranged by box width at 56-156 m, up to 122 m off the nearest depth-ranged tick (`docs/FINDING-the-escape-that-stayed-inside-the-ring.md`, "The re-fly of 2026-10-12"). Rows now carry `subject_depth` (the depth range and the rule that refused it).

## How to run it

Steps 0-3 below were run on 2026-10-08 (see "Measured 2026-10-08"). The `dev` and `hil` paths after them have not been run.

**Since 2026-10-13** (rail inputs, CHANGELOG):
- The node flies the signed bundle of its policy from `bundles/` (gitignored; build it with `python tools/build_flight_bundles.py` where the lab key is, and again after any policy edit, since a stale bundle is refused). A YAML policy flies only with `--dev-yaml`; `run_pas_ardupilot.ps1` passes the bundle when it is present and otherwise `--policy <yaml> --dev-yaml`, and says which. The MAVROS commands for dev and hil below need the same: the bundle on that host, or `--dev-yaml`.
- `--clock-start ISO8601` / `--clock-tz` give the Shield and the CSP a rule clock (wall clock in +08:00 without them, recorded either way); `-ClockStart` on the script.
- Before arming, the policy's clock-proof zones go up to ArduPilot's GeoFence over the node's own link (`--no-fence-upload` turns it off); `citylife-quad.param` now sets FENCE_TYPE 7 (the polygon bit) and AVOID_ENABLE 0, as `sitl/fence/guardrail_fence.parm` does. G1 has not been re-run since that parameter change.
- `--subject-actor` (default `auto`, Car_10 for a red car) logs the followed car's simulator pose for evaluation; the Shield still uses the estimate.
- `--hold-mode LOITER` (SITL) holds the FSM's Loiter as the literal LOITER mode with an RC3 override of 1500; BRAKE stays the default. Over MAVROS this needs `mavros_node -p system_id:=255` (SYSID_MYGCS), which `start_ardupilot.sh --mavros` now sets.

```powershell
# 0. once: pymavlink in the flight env (pin as in sitl/setup_sitl.sh), and optionally
#    mavlink-router in WSL (otherwise the MAVProxy fallback is used and labelled).
#    Both are installed on this desktop as of 2026-10-08.
<vla-real python> -m pip install pymavlink==2.4.49

# 1. addresses, firewall rules, packages, rendered configs - starts nothing
.\scripts\run_pas_ardupilot.ps1 -Step check
#    expect: "WSL2 networking: nat | Windows (as WSL sees it) 172.x.x.1 | WSL 172.x.x.y"
#    and a dry-run JSON with "ok": true once pymavlink is installed.
#    The ArduPilot pin line, as measured on 2026-10-06, is PIN FAIL: the binary is
#    "ArduCopter V4.5.7 (2a3dc4b7)" but the modules/mavlink/pymavlink submodule
#    has drifted (sitl/setup_sitl.sh explains it and how to restore the pin).
#    The flight still runs; the record says which firmware flew.

# 2. gate G1: three square-mission runs and the verdict (close the Unreal editor first).
#    -AltM 15: the 10 m default flies the square through a street light.
.\scripts\run_pas_ardupilot.ps1 -Step g1 -AltM 15
#    per run: "[G1] condition 1: True", "condition 2: True", "condition 3: True"
#    a run that cannot complete prints "[G1] run N did NOT complete: ..." and is
#    counted as failed. Verdict: demo\out\g1_<timestamp>\g1_verdict.md,
#    status PASSED, or which condition fired.

# 3. the follow flight, ArduPilot flying, Mission Planner watching. This is the
#    pymavlink node in the Windows env: a functional run, stamped
#    projectairsim-ardupilot-pymavlink, not one of the grant's topologies.
#    The node picks up the newest G1 verdict.
.\scripts\run_pas_ardupilot.ps1 -Step node -Object "a red car" -Seconds 120
```

`-Step node -Link mavros` is refused before anything starts: the Windows env has no ROS 2. The MAVROS node, which is what the grant's `dev` and `hil` labels need, runs in WSL or on the Orin and attaches to a scene the desktop loaded.

**dev, the MAVROS node in WSL** (needs one WSL environment with ROS 2 Jazzy + MAVROS + `rclpy`, and `torch`, `transformers` and the `projectairsim` client; none exists yet):

```powershell
# desktop, in this order
.\scripts\run_pas_ardupilot.ps1 -Step scene                 # loads CityLife + the ArduPilot robot, leaves it running
.\scripts\run_pas_ardupilot.ps1 -Step sitl -Mavros          # ArduPilot AFTER the scene, the router, MAVROS 2 (fcu_url udp://:14555@)
```

```bash
# WSL, a second terminal; <windows-ip> is WSL's default gateway under NAT, 127.0.0.1 when mirrored
python3 demo/pas_ardupilot/perception_node.py --link mavros --scene-mode attach \
    --sim-host <windows-ip> --g1-verdict <the desktop's g1_verdict.json, as a /mnt/... path>
```

Under NAT the client in WSL reaches the simulator's TCP 8989/8990 at the gateway address, so Windows Firewall must let that in from the WSL adapter. The node counts the simulator as on its own desktop because it sits at WSL's default gateway (`topology.sim_on_this_desktop`).

Expected in `~/sitl-run/pas/sitl.log` (ArduPilot's own lines):

- `Starting SITL Airsim type 0`
- `Bind SITL sensor input at 127.0.0.1:9003` (it binds 0.0.0.0; the text says 127.0.0.1)
- `AirSim control interface set to <gateway>:9002`
- then periodic `FPS avg=...` near 200 at real time (one sensor frame per 5 ms step).

**A repeating `No sensor message received in last 1s` means the simulator's frames are not reaching WSL.** Check the rendered `ardupilot-ip`, and the firewall on UDP 9002 for the return path.

**Mission Planner:** choose UDP, port 14550, and Connect. Alternatively use TCP to WSL's own address (`wsl hostname -I`) on 5790: under NAT, mavlink-routerd's `[::]` listener is not reachable on 127.0.0.1 (measured 2026-10-08), although `start_ardupilot.sh` still prints 127.0.0.1. Under the MAVProxy fallback, an IPv4 listener, TCP 127.0.0.1:5791 works. Expect:

- a 3D GPS fix near 35.6895 N, 139.6917 E (the scene's home-geo-point);
- `ArduCopter V4.5.7 (2a3dc4b7)` in Messages;
- the mode changes the node, the FSM or the G1 script makes.

Leave Mission Planner passive during a run. A mode change there is recorded as the autopilot taking the aircraft, and the node then stops commanding.

**hil, the node on the Jetson Orin** (after the Orin is set up as in [RUNBOOK-orin-hil.md](RUNBOOK-orin-hil.md); MAVROS on the Orin):

```powershell
# desktop, in this order
.\scripts\run_pas_ardupilot.ps1 -Step scene                 # loads CityLife + the ArduPilot robot, leaves it running
.\scripts\run_pas_ardupilot.ps1 -Step sitl -Orin <orin-ip>  # ArduPilot AFTER the scene; router adds a UDP output to the Orin
```

```bash
# Jetson Orin
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14555@<desktop-ip>:14555 &
python3 demo/pas_ardupilot/perception_node.py --link mavros --scene-mode attach \
    --sim-host <desktop-ip> --g1-verdict <copy of the desktop's g1_verdict.json>
```

In attach mode the node never loads the scene. It checks that the simulator serves `/Sim/SceneCityLifeArduPilot/robots/Drone1/...` topics, and refuses with "Load it on the desktop first" if not. The named remote in `fcu_url` is what the hil evidence check reads. MAVROS's UDP transport replies to the address the router's packets come from, so replies reach the router even though its source port is not 14555; this has not been run here. The runbook recommends WSL mirrored networking when ArduPilot runs natively in WSL, so the Orin can reach it. Under NAT the router's outbound UDP to the Orin should pass, but that is unverified. The desktop must allow inbound TCP 8989/8990 from the Orin for the camera streams.

## What has not been verified yet

Settled by the runs of 2026-10-08 and moved to "Measured 2026-10-08": whether the level holds the physics step with ArduPilot in the loop (not at 3 ms; at 5 ms it ran at 0.951-0.967 of real time), whether the EKF converges on Project AirSim's sensors (p95 0.81-0.82 m), whether the airframe tuning flies in CityLife (an AUTO square within 1 m of every corner), whether the default G1 square clears the buildings (it does not at 10 m; it does at 15 m), and whether the stream to Mission Planner's port carries a flight (19.0 Hz). Still open:

- Mission Planner itself on the Windows side: the measurement is a listener on its UDP port, and Mission Planner's display was not recorded. Inbound UDP from the WSL adapter did reach Windows that day (ArduPilot's PWM on 9002 moved the vehicle in G1 condition 2; the listener on 14550 received the stream); whether a firewall rule or the default policy let it through was not recorded, and the launcher changes none.
- The MAVROS link (including `request_mode`, `time_reference`, `statustext` and `extended_state`) and the ROS 2 camera source were exercised only against stand-ins, never against a live MAVROS.
- The `dev` path: no WSL environment holds ROS 2 + MAVROS together with `torch`, `transformers` and the `projectairsim` client yet, so the MAVROS node has not been started in WSL either.
- The Orin path (`hil`): attach mode against a live simulator, the projectairsim client on aarch64, MAVROS's reply path through the router under WSL NAT, and DDS.
- The KPI-evidence step (`guardrail.replay.kpi_evidence_grade`, which demotes a graded run whose replay bundle lacks a trusted signature) is wired as on the SITL rails but has never been reached. G1 has passed, but the only live run so far used pymavlink, which is offered no grant label, and `build_manifest()` still refuses `dev`/`hil` beside a Project AirSim scene.
- The escalation FSM on this rail reached Loiter 8 times on the live follow and never RTL or Land, so those two edges are tested with stand-ins only. What Loiter did to the altitude is defect (c) above.
- A Shield-off control run on this rail, which the P0 escape rate of the follow needs as its null.
