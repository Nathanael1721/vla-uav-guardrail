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

`demo/pas_ardupilot/perception_node.py` is one process. On each tick (nominal 10 Hz, `TICK = 0.1` s; every recorded run measured 8.97-9.16 Hz, see the tables below) it does the following:

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

**The start gate (2026-10-21, not flown yet).** After the take-off the node hovers (nothing commanded; ArduCopter's own GUIDED hold) until follow_vlm's own `Acquirer` has confirmed the subject: 5 fresh inferences in a row, each with a box its presence verdict does not call ABSENT (colour, wall-sized, the width its depth implies, standing on the road), within 30 m, continuous in the frame (`StartGate`, `--start-when-seen`, `--start-max-range-m`, `--start-timeout-s`). The Grounder publishes every candidate while it acquires and is committed to the pick, which seeds a `TargetLock`. A gate that times out (330 s) flies no mission unless `--start-timeout-action fly`; `metrics.json` `start_gate` records what was judged and refused and, read after the decision for evaluation only, whether the pick was the subject. Replayed on the 345 logged inferences of `pas_ap_ovf_on_20261021` (`tests/fixtures`), it never opens: 34 of the 39 boxes are not a car (19 by the width their depth implies, 15 not standing on the road), 2 more are beyond 30 m, and no streak passes 1. `--identity` adds follow_vlm's identity tiers (only an OK box may start or steer; the start is decided in the world and must be seen moving; no strict lock, since this node has no estimator to release one), and `--negative-query "a traffic light"` drops a box that scores higher for that phrase than for the subject. The detector now starts before SITL and the mission waits for its first inference, and the simulator's evaluation RPCs run off the loop (`OffLoopRpc`, at most 50 ms waited per tick, all of that tick's RPCs together). On 21 Oct one 1024.9 ms truth read was the 1028 ms tick (measured). The detector loaded beside the first 26 s of ticks, and the flight's one 97.9 ms Shield step, at 19.4 s, fell inside that load; that the load caused it is inferred from the timing, not profiled. Neither change has flown yet.

**The hover before the mission is watched (review 2026-10-22, not flown yet).** As first written, the gate's hover (up to 330 s, plus up to 300 s waiting for the detector after the take-off) read no mode and checked no rule, and ran outside the loop's `try/finally`. A pilot who took the aircraft during it became the loop's baseline mode, so the node asked for LAND over the pilot (or, had the gate then opened, flew its setpoints over the pilot's LOITER). An exception in the gate left the aircraft holding in GUIDED with no LAND and no episode record. Now every hover tick is a `HoverWatch` tick. The autopilot's mode is read by the loop's own takeover test (`autopilot_took_over`); a takeover ends the hover, flies no mission, asks nothing of the autopilot and fills `autopilot_took_over`. A link that stays stale `LINK_LOST_S` ends it. The Shield checks the position (`state_is_unsafe`), with the gate's current candidate given as the subject, so the stand-off rings bind. With the Shield on, the first unsafe tick starts the loop at once, so the Shield and the escalation FSM act. The hover's ticks go to `hover_log.jsonl` and its counts to `metrics.json` `pre_mission_hover`; it is in no KPI, and an unsafe hover tick makes the run not KPI-grade. The whole hover runs inside a `try` that turns an exception or Ctrl+C into a no-mission end: landed, recorded, and, for a crash, a non-zero exit. The hover's own records are best effort, because a write is the likeliest thing to fail on this machine (a full D:, a file OneDrive holds open under a reused tag). An event or `hover_log.jsonl` that cannot be written is kept in `pre_mission_hover` (`io_errors`, `log_error`) and named among the KPI-grade reasons. Before review 3 (2026-10-22), that second exception, raised from the hover's handler or `finally`, left `run()` with no LAND. The takeover test's baseline is GUIDED, which both links' `bring_up` ends in (or raises). A hover that starts in any other mode is a takeover at the bring-up (phase `bring-up`): a fence RTL or a pilot's switch reported in the climb's last pump had become the baseline, and the loop flew its setpoints over it. The gate's record names its `outcome` (reached, timed_out, takeover, link_lost, detector_dead, unsafe, crashed, interrupted). The detector's first inference is now waited for in the pre-arm step, disarmed, on links that have one. On a link without one, that wait is airborne, watched, and comes out of `--start-timeout-s`, which is capped at 600 s (`START_TIMEOUT_MAX_S`). A detector thread that died is not waited for, and no mission is flown on it.

**No way out of `run()` from the arming without a LAND (review 3, 2026-10-22, not flown yet).** The loop's `finally` asks for LAND, and the hover reaches it as a no-mission end. Until review 3, anything else that raised between the arming and that `finally` met only `run()`'s outer `finally`, which stops the recorder, the detector, the camera and SITL but asks nothing of the autopilot. Examples: a take-off that stalled (`take-off did not reach N m`, armed and climbing), the `mission_start_check` write on a full disk, a `FollowPilot` or `SubjectTruth` that raised, or `mission_start`'s write. On SITL the aircraft went with SITL. On HIL or attach it would have been left in GUIDED mid-climb or holding, with nothing commanding it. The outer `finally` now starts with a net (`land_on_the_way_out`). If the aircraft was armed (the bring-up returned, or the autopilot reports it armed) and the loop never made its LAND decision, the net hands the throttle back to the radio and asks for LAND, unless the autopilot or a pilot has the aircraft (a takeover the hover saw, an FSM fail-safe, or any mode the node did not ask for). It then writes the `mission_end`, naming the stage, the error, the mode and whether LAND was accepted, and lets the error propagate. No `metrics.json` or KPI table is written for such a run, as for a crash in the loop. A refused arm or an EKF that never converged is not armed, so nothing is asked and no episode end is written, as before. The loop's own LAND decision also takes one last look at the mode, with the loop's takeover test against the last mode judged. On a no-mission end no loop tick runs, and the gate pumps the link once more after the hover's last tick, for its truth verdict. A pilot's switch reported in that pump was judged by nobody, and LAND was asked over the pilot's LOITER. It is now a takeover (phase `before the mission`), and nothing is asked. Tested with the rail's stand-ins only.

**Not reused yet.** `follow_vlm.fly()`'s target estimator, strict lock and re-acquisition, trail following, fence guard and landing-site choice live inside that function. Reusing them needs a refactor of `follow_vlm.py`. Until then the node tracks with the servo law on the detector's box. Since 2026-10-13 the node logs the followed car's simulator pose for evaluation (`--subject-actor`), so its runs carry a tracking score; on every close flight of 2026-10-13 and 2026-10-14 it is 0 (`frac_within_30m` 0.0), because the detector was not on the car: 0 estimates came within 6 m of it, and the 2026-10-14 video shows the box on a red traffic signal (see "Flown 2026-10-13"). Before 2026-10-13 `frac_within_30m` was `None`, because no subject ground truth was read.

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

**What `manifest.py` needed (done 2026-10-13, 93dcb10).** Until then `build_manifest()` refused `dev`, `hil` and `flight` beside any Project AirSim scene file, a guard written when every scene meant simple_flight. It now accepts `dev` and `hil` beside a scene whose scene file and robot config it reads as controller `ardupilot-api` (`scene_controllers`; its docstring dates the change 2026-10-12), but only on the same evidence as without a scene: the live MAVROS chain for `dev`, the Orin's evidence for `hil`, and for both MAVROS's own link (an `fcu_url`, and no pymavlink link). It still refuses a simple_flight scene, a pymavlink link, and `flight` beside any scene (`tests/test_railinputs_manifest.py`). The node needed no change (`topology.py`): it offers the grant label when the rail is qualified, stamps its own label when the label is withheld or refused, and keeps any refusal in `metrics.json` (`manifest_refusal`). No live MAVROS run has exercised the acceptance yet.

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
| Ticks, mission time, loop rate | 1101 in 120.14 s, 9.16 Hz | 10 Hz, and the project's 9.5 Hz loop gate. The shortfall was the pacing, not the work: the loop slept 0.1 s minus each tick's work from that tick's own start, so every round-up to the OS timer (about 10 ms a tick on the flight env's Python 3.10 on Windows; per-tick period median 0.110 s) was lost for good. Every recorded run of this rail (8 to 14 Oct) was paced that way and ran at 8.97-9.16 Hz. Since 2026-10-19 the node paces to an absolute deadline (`perception_node.next_tick_deadline`, with a measured 1 ms OS timer) and judges the gate on the loop's own tick starts (`metrics.json` `loop_hz_paced`, `loop_hz_below_gate`; `loop_hz` keeps the mission-clock rate). No flight has run on the deadline pacing yet |
| Detector rate | 3.75 Hz | the grant's 10 Hz; null 0 |
| Ticks flown by the node / handed to ArduPilot by the FSM (nothing sent) | 937 / 164 | |
| Ticks tracking the car / searching | 269 / 832 | |
| Shield interventions, brakes | 85, 0 | |
| ArduPilot real-time factor | 0.951 | 0.95-1.05 (inside by 0.001) |
| EKF vs simulator, horizontal p95 (221 pairs) | 0.627 m | |
| Shield time per tick, median / p99 / max | 0.32 / 2.43 / 104.2 ms | the grant's budgets are 5 ms per query and 100 ms per tick; 6 ticks took over 5 ms in all, 1 over 100 ms |
| GeoFence breaches (FENCE_STATUS received) | 0 | fence read back: ENABLE 1, TYPE 3, ACTION 1 (RTL), ALT_MAX 20 m, RADIUS 400 m, MARGIN 2 m |
| Mission Planner's UDP 14550 | GLOBAL_POSITION_INT and ATTITUDE at 19.0 Hz, largest gap 0.1 s | `sitl/gcs_listen.py check` over the mission window. The capture is not archived with the run (see below) |
| P0 escape rate | 0.0053 (5 of 937 flown ticks) | target 0. As of 8 Oct no Shield-off control had been flown on this rail, so there was no null; the controls were flown on 13 and 14 Oct (see "Flown 2026-10-13") |
| Repair success (re-check and theta) | 0.776 (52 of 67) | |
| Time to safe, mean / max | 2.81 / 6.42 s over 12 episodes | |
| Time below the policy's 6 m floor | 27.9 s (279 ticks) | as recorded: 279 ticks x 0.1 s. The trajectory's own timestamps give 30.4 s, because the loop ran at 9.16 Hz; the node time-weights dwell from the timestamps since 2026-10-19 |
| Mission success | false: P0 escape, altitude dwell, unsafe position entered | |
| Simulator contacts | 1 | take-off and landing included; the record has no time for it |

The 19.0 Hz row comes from `gcs_node.json`, a 300 s `sitl/gcs_listen.py listen` capture taken while the follow flew (21816 packets from one source, sha256 `2b2bb6a898ba75e5...`). It was saved in a session's temporary folder, not in `demo/out/pas_ap_20261008_224440/`, so the run folder alone cannot re-derive the row. Re-checked for this record with `python sitl/gcs_listen.py check gcs_node.json demo/out/pas_ap_20261008_224440/events.jsonl`: PASS, 2285 of each message in 120.1 s (19.0 Hz), largest gap 0.1 s. Until the capture is copied into the run folder as `gcs.json`, treat the row as resting on a file outside the repository.

`kpi.json`'s `failsafe_trigger_correctness` (0.881) is the P0-acted tick share under its legacy name, as `failsafe_trigger_correctness_is` says. It is not the grant's fail-safe trigger correctness and must not be quoted as such.

### Defects open when the follow flew

(a)-(c) are what the follow run showed. (d) was measured the day before by the stress harness's headless kinematic sweep (manifest topology `headless-kinematic`: no simulator and no ArduPilot; Loiter, RTL and Land are modelled as ideal position control, so the sinking LOITER of (c) is not in it). It is listed because it bears on the same escalation FSM, and a figure measured with a real autopilot's Loiter could be lower. This section records the run as flown at `b23b5fd331b8-dirty`. A fix made after it is recorded in `CHANGELOG.md` and, for a defect that was silent, in a `docs/FINDING-*.md` note; it does not change the numbers below, and only a new flight on this rail shows whether it worked. Until such a flight, these numbers are the rail's latest.

**(a) Five P0 stand-off escapes.** Ticks 857, 927, 928, 999 and 1100 sent a command that still violated `standoff-any` (5 m to the car, P0). On each, the aircraft was already inside the ring by the logged estimate, standing still would itself break the rule (`stop_illegal`), no repair operator converged, and the best-effort `ClearanceEscape` did not leave the ring within the tick. So the Shield flew its own best action, as designed when stopping is illegal, and the FSM stepped up.

What the log adds: on all 5 ticks the logged subject position equals the aircraft's own horizontal position (0.00-0.01 m apart), while on the neighbouring ticks that have an estimate it is 3.5-54.6 m away. `perception_node.subject_estimate()`, as it was at that revision, turns the depth range r into a horizontal range as `sqrt(max(0, r² - dh²))` with dh = altitude - 0.75 m (about 7 m here), so any range r ≤ dh puts the car directly under the aircraft instead of being rejected. On tick 857 the detection the tick carried (`det_seq` 324) was a 50 x 24 px box in the upper half of the 768 x 432 frame, which is not how a car under the aircraft looks to a forward camera. The depth range itself is not logged, so this is the likeliest reading, not a measurement. If it holds, there are two defects: a subject estimate that silently collapses onto the aircraft, and a Shield that cannot leave the ring in one tick once it is inside. Neither raised an error: the run reports five escapes and nothing about the estimate.

**(b) Brake to Loiter on stand-off repairs.** The FSM went Brake to Loiter 8 times. 7 were on `standoff-any`: on 5 of them the `StandoffHold` repair was larger than theta (3.63, 5.29, 8.08, 9.23 and 9.81 m lateral against 2.0 m; ticks 480, 750, 202, 1001, 282), and on 2 no operator converged (ticks 857 and 928, both escape ticks). The 8th, at tick 1100, was building clearance. The Normal to Brake edges before them carried hold repairs of 2.54-9.78 m. The subject estimate jumped just before several of them (59.6 m to 8.3 m at tick 201, 108.8 m to 15.4 m at tick 735, 48.5 m to 5.6 m at tick 841), and a 5 s forecast against a jumping subject asks for a large hold. This node tracks with the servo law on the raw detector box; `follow_vlm.fly()`'s target estimator is not reused here yet (see "The perception node").

**(c) LOITER sinks out of the altitude band.** ArduCopter LOITER with no RC input descends; the SITL rails showed it on 2026-10-07 ("LOITER descended from 14.2 m to the ground", `docs/DESIGN-ros2-interface.md`). Here the 7 Loiters before the last tick lasted 2.1-3.3 s each and lost altitude at 1.35-1.77 m/s from 7.6-8.0 m (down to 1.80-4.99 m when ArduPilot handed back), and every one was followed by a stretch below the policy's 6 m floor: 7 stretches, 279 ticks (27.9 s as 279 x 0.1 s; 30.4 s by the trajectory's timestamps), lowest points 0.36-3.43 m. (The 8th Loiter began on the final tick.) The lowest, 0.36 m at tick 322, came in GUIDED after the second Loiter, with the node already commanding the 1.8 m/s climb limit. These are 279 of the run's 310 unsafe P0 ticks (the rest: 25 building clearance, 6 stand-off) and the `dwell:alt_violation_s` mission failure. The escalation meant to hold the aircraft safe takes it out of the band instead.

**(d) Fail-safe trigger correctness in the headless harness** was 0.747 (library), 0.726 (nightly) and 0.875 (smoke) against the grant's 0.99, and below the never-trigger null (CHANGELOG 2026-10-07, Retracted; the current figure is under "Re-flown 2026-10-12"). Every stored sweep and nightly manifest names the topology `headless-kinematic`; the harness's ArduPilot path (`--backend sitl`) refuses every scenario with events (`sitl_refusal` in `experiments/sweep_scenarios.py`). It was not measured on this rail or on ArduPilot SITL.

## Re-flown 2026-10-12, with the fixes

The same follow (red car, 120 s, same policy and seed) flown with the 9-12 Oct fixes: `demo/out/pas_ap_20261012_033329`, pymavlink, same topology stamp, not KPI-grade.

| | 8 Oct | 12 Oct |
|---|---|---|
| ticks, loop rate (both under the 9.5 Hz loop gate; the cause is in the 8 Oct table) | 1101, 9.16 Hz | 1098, 9.14 Hz |
| detector rate | 3.75 Hz | 4.26 Hz |
| ticks with the car tracked | 269 | 129 |
| nearest the detector's estimate came (logged) | 0.0 m (the range-0 placements) | 16.5 m |
| Shield interventions | 85 | 43 (yaw caps, building clearance) |
| P0 escape ticks | 5 | 0 |
| FSM | 27 edges, 8 Brake to Loiter | 6 edges, Normal to Brake 3 times and back; never Loiter |
| mode requests | 8 LOITER, 7 GUIDED, LAND | LAND |
| altitude | 0.36-8.15 m, 27.9 s (279 ticks x 0.1 s; 30.4 s by timestamps) below the 6 m floor | 7.99-8.15 m, 0 s below it |
| ArduPilot real-time factor, GeoFence breaches | 0.951, 0 | 0.962, 0 |

Against the defects above: (a) is not tested by this run, because the car never came near the 5 m ring; (b) and (c) did not recur, but no stand-off repair arose and the FSM never asked for a hold, so the BRAKE hold was not flown either; (d) is unchanged in kind (republished at 0.823, still headless; that figure was itself retracted on 2026-10-13, because it was measured on a 3 s / 0.5 s forecast: on the grant's 5 s / 0.1 s, over the 176-episode library, it is 0.838, 140 of 167 labelled episodes, against a never-trigger null of 0.940, `docs/data/scenario_sweep.json` and CHANGELOG 2026-10-13; republished on 2026-10-21 with the FSM's hold points: 0.844, 141 of 167, 25 false and 1 missed trigger, the null unchanged at 0.940, still headless and still below it). Only a run that brings the aircraft close to the car tests the range-0 fix and the hold on this rail.

Found on this run: 43 depth returns were refused and the car ranged by box width at 56-156 m, up to 122 m off the nearest depth-ranged tick (`docs/FINDING-the-escape-that-stayed-inside-the-ring.md`, "The re-fly of 2026-10-12"). Rows now carry `subject_depth` (the depth range and the rule that refused it).

## Flown 2026-10-13: G1 again, and the follow flown close with the Shield on and off

G1 passed again after the fence parameter change (`demo/out/g1_20261013_203404`: 3/3 runs, EKF p95 0.82-0.83 m, frozen-position null 18.96-18.98 m, 0 fence breaches).

The red-car follow with `--want-width 0.5` (the pilot closes until the car fills half the frame), from the signed `follow_car_citylife` bundle:

| | Shield on (`pas_ap_close_on`) | Shield off (`pas_ap_close_off`) |
|---|---|---|
| ticks | 532 (FSM sent it home at 58.1 s) | 1100 (120 s) |
| P0 escape ticks (rate) | 0 (0.0) | 727 (0.661): building clearance alone 705, stand-off alone 17, both 5. A further 22 ticks broke only the P1 kinematic caps; the 749 first written here counted them too and was retracted on 2026-10-14 |
| FSM | Brake -> Loiter twice (repairs of 2.08 m and 2.14 m over theta 2.0 m), X7 in the second hold (illegal depth grew 2.95 m in 2.07 s), RTL home | not run (Shield off) |
| hold | BRAKE, 7.99-8.01 m over 42 ticks | - |
| ArduPilot real-time factor, fence breaches | 0.966, 0 | 0.946, 0 |

This is the first flight evidence of the BRAKE hold and the rail's first Shield-off control.

**The subject's ground truth was right; the detector was not on the car** (resolved 2026-10-14, CHANGELOG). The truth (Car_10, read every tick) sat a median 258-264 m from every detector estimate, and the question was which of the two was wrong. The rail's Car_10 poses lie on follow_vlm's validated Car_10 track (median 0.52-0.54 m), so the truth frame is right. The recorded Shield-on video of 2026-10-14 shows the detector's box on a red traffic signal; across the four close flights 0 estimates came within 6 m of the car. On the 13 Oct Shield-on flight the car was in the camera's view on 10 of its 149 estimate ticks; on the 14 Oct Shield-on and Shield-off flights, on 4 of 156 and 0 of 132 (`metrics.json`, `subject_truth.view`). The rail starts with the car about 151-152 m away (`subject_range_m` on the first tick of each close flight's `flight_log.jsonl`: 150.8-151.9 m) and has no start gate (follow_vlm waits until it sees the car). Until one is ported, this rail's follow scores stay at 0 and its flights test the Shield, not tracking. The Shield never reads the truth; it is logged for evaluation only.

### Recorded 2026-10-14

The same close approach (`--want-width 0.5`, the signed bundle) flown again with `--save-view` at `5cf7a08e5eee-dirty`, pymavlink, not KPI-grade:

| | Shield on (`pas_ap_video_on_20261014`) | Shield off (`pas_ap_video_off_20261014`) |
|---|---|---|
| ticks | 517 (the run ended at 57.6 s, when RTL reached home) | 1099 (120 s) |
| P0 escape ticks (rate) | 0 in 388 Shield-flown ticks (0.0) | 736 (0.6697): building clearance alone 731, stand-off alone 2, both 3 |
| FSM | Normal -> Brake, then X7 -> RTL from Brake (the position stayed illegal and its depth grew); no hold flown | not run (Shield off) |
| ticks inside a P0 margin (unsafe position) | 71: 20 while the Shield flew its corrected command, 51 under RTL | 739 |
| ArduPilot real-time factor, fence breaches | 0.935 (under the 0.95 band), 0 | 0.954, 0 |
| frames recorded | 1105 at 19.19 Hz | 2372 at 19.75 Hz |

- **Inside the margin while the Shield flew.** Unlike 2026-10-13, where every unsafe-position tick was flown by ArduPilot (its BRAKE mode under the escalation FSM's Loiter, and RTL), 20 of these came while the Shield was still flying its corrected commands in Brake, up to 1.0 m deep. kpi.json still counts 0 P0 escapes, because every emitted command was legal by the forecast.
- **How.** At 40.7 s the Shield held the stand-off from the detector's target (StandoffHold, edge G1 to Brake); from the next tick ClearanceFix pushed each command away from the building at 0.50-0.86 m/s (horizontal speed of the emitted command). The aircraft was flying at 3.2 m/s and carried 3.3 m on while it slowed.
- **Why the forecast did not see it.** The forecast is constant-velocity from the commanded action (`guardrail/shield.py` `_predict`; `State` has no velocity), so each push-away read as recovering (2.70 m to 4.64 m on the first) while the aircraft was still stopping. The position stayed illegal and its depth grew, so X7 sent the aircraft home from Brake.
- **Not fixed:** a forecast that starts from the aircraft's own velocity, beside the hold that stops clear of the margin.
- **The subject.** The detector boxed a red traffic signal, not the car, and the rail has no start gate, so this pair shows the Shield, not tracking.
- **The videos** (local, not committed; `demo/out` is gitignored):
  - `pas_ap_video_on_20261014_demo.mp4` (58 s) and `pas_ap_video_off_20261014_demo.mp4` (120 s) in their run folders: the drone's camera with the policy HUD (the FSM state, or SHIELD OFF on the control arm) beside the chase camera. MPEG-4 Part 2, which browsers do not play.
  - A `_demo_phone.mp4` H.264 re-encode beside each.
  - `demo/out/pas_ap_video_compare_20261014.mp4` (120 s, H.264): the two stacked from take-off, with its phone re-encode `pas_ap_video_compare_20261014_phone.mp4` beside it.

## How to run it

Steps 0-3 below were run on 2026-10-08 (see "Measured 2026-10-08"). The `dev` and `hil` paths after them have not been run.

**Since 2026-10-13** (rail inputs, CHANGELOG):
- The node flies the signed bundle of its policy from `bundles/` (gitignored; build it with `python tools/build_flight_bundles.py` where the lab key is, and again after any policy edit, since a stale bundle is refused). A YAML policy flies only with `--dev-yaml`; `run_pas_ardupilot.ps1` passes the bundle when it is present and otherwise `--policy <yaml> --dev-yaml`, and says which. The MAVROS commands for dev and hil below need the same: the bundle on that host, or `--dev-yaml`.
- `--clock-start ISO8601` / `--clock-tz` give the Shield and the CSP a rule clock (wall clock in +08:00 without them, recorded either way); `-ClockStart` on the script.
- Before arming, the policy's clock-proof zones go up to ArduPilot's GeoFence over the node's own link (`--no-fence-upload` turns it off); `citylife-quad.param` now sets FENCE_TYPE 7 (the polygon bit) and AVOID_ENABLE 0, as `sitl/fence/guardrail_fence.parm` does. G1 passed again after that change (2026-10-13, `demo/out/g1_20261013_203404`).
- `--subject-actor` (default `auto`, Car_10 for a red car) logs the followed car's simulator pose for evaluation; the Shield still uses the estimate.
- `--hold-mode LOITER` (SITL) holds the FSM's Loiter as the literal LOITER mode with an RC3 override of 1500; BRAKE stays the default. Over MAVROS this needs `mavros_node -p system_id:=255` (SYSID_MYGCS), which `start_ardupilot.sh --mavros` now sets.

**Since 2026-10-14** (recording, CHANGELOG):
- `-SaveView` on `-Step node` passes `--save-view` and records the demo video as `demo/follow_vlm.py` does: the drone's camera with the policy HUD (rule panel, the Shield / escalation-FSM banner, minimap) to `demo\out\<tag>\view\fpv`, the chase camera to `view\tps`, and `view\recorder.json` with the rate the recorder measured. The recorder runs off the control loop. The node's own flags are `--record-hz` (the 2026-10-14 pair used 20) and `--record-height` (720).
- `view\` is a junction to `-OffloadRoot\<tag>\view` on C: (default `C:\Users\natha\vla_drone_offload\demo_out`), because D: is near full; `-OffloadRoot ""` keeps the frames under `demo\out`. The 120 s Shield-off flight of 2026-10-14 wrote 0.71 GB of frames, the 58 s Shield-on flight 0.33 GB (the 260 s city flight of 2026-10-13, 1.5 GB).
- `python tools\make_demo_video.py --tag <tag> --height 720` turns a recorded run into `<tag>_demo.mp4` at the measured rate; the launcher prints that line when the flight ends.
- `python tools\make_compare_video.py --a <Shield-on tag> --b <Shield-off tag> --out <file>.mp4` stacks two recorded runs (`--layout side` puts them side by side, `--panel fpv|tps` keeps one view), both from take-off, each half at its own run's measured rate. It refuses an existing output unless `--force`.

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

# 4. the close approach recorded on video with the Shield on and off (the pair of
#    2026-10-14), then the stacked video. Each flight writes demo\out\<tag>\view.
.\scripts\run_pas_ardupilot.ps1 -Step node -Seconds 120 -SaveView -Tag <on tag> -NodeArgs "--want-width","0.5"
.\scripts\run_pas_ardupilot.ps1 -Step node -Seconds 120 -SaveView -Tag <off tag> -NodeArgs "--want-width","0.5","--shield","off"
python tools\make_demo_video.py --tag <on tag> --height 720
python tools\make_demo_video.py --tag <off tag> --height 720
python tools\make_compare_video.py --a <on tag> --b <off tag> --out demo\out\<name>_compare.mp4
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

**hil, the node on the Jetson Orin** (after the Orin is set up as in [RUNBOOK-orin-hil.md](RUNBOOK-orin-hil.md)).

The layout on the AGX Orin (2026-11-06): MAVROS 2 runs in the companion container (`deploy/compose/docker-compose.orin.yml`, service `mavros`, its `FCU_URL` from `deploy/topologies/hil.env`), as on the ROS 2 rail's hil runs. The perception node runs natively on the Orin's host, beside it on one ROS 2 graph: its detector (OWL-ViT) and the VLA client's frame pipeline need the GPU and torch, and the Project AirSim client, which the companion image never carries ([DESIGN-topologies.md](DESIGN-topologies.md), decision 2). On the host the node uses ROS 2 Jazzy's rclpy and mavros_msgs from apt, and the `~/vla-drone-env` environment (`deploy/orin/agx_vla_env.sh`). `deploy/orin/run_pas_perception.sh` takes the graph from `hil.env` (ROS_DOMAIN_ID, RMW_IMPLEMENTATION, the Cyclone DDS peers, and MAVROS's FCU URL as `MAVROS_FCU_URL`, which the link record falls back to) and starts the node.

The VLA follow flown by the lab's own AIOT-VLA (`--pilot vla --mission follow`), in this order:

```powershell
# desktop, in this order
.\scripts\run_pas_ardupilot.ps1 -Step scene -VlaCameras        # CityLife + the ArduPilot robot with the VLA cameras, left running
                                                               # (add -FrontCaptureIntervalS 0.066 for FrontCamera at about 15 Hz)
.\scripts\run_pas_ardupilot.ps1 -Step sitl -Orin 10.42.0.3     # ArduPilot AFTER the scene; the router adds a UDP output to the Orin
```

```bash
# AGX Orin, in this order
docker compose --env-file deploy/topologies/hil.env \
    -f deploy/compose/docker-compose.orin.yml up -d mavros
~/vla-drone-env/bin/python demo/vla_server_foreign.py serve --model aiotvla \
    --weights ~/models/aiotvla/v10b --port 8791 --front-source detector --horizon-s 0.5 &
G1_VERDICT=~/g1_verdict.json TAG=pas_hil_aiot_01 bash deploy/orin/run_pas_perception.sh
#   FRONT_CAPTURE_INTERVAL_S=0.066 as well when the scene step was given it
```

The script runs `perception_node.py --link mavros --scene-mode attach --sim-host <hil.env DESKTOP_IP> --g1-verdict ... --pilot vla --mission follow --vla-server 127.0.0.1:8791 --vla-front-source detector --vla-step-as-rate 0.5 --vla-rate-yaw rate --cadence pipelined --allow-unstamped-pipelined --vla-cruise-mps 4.5 --vla-speed-cap-mps 5`, the last two being `vla_fly`'s follow speeds. It never passes `--dev-yaml` or `--start-sitl` and refuses both. Before it starts the node it checks the G1 verdict, the maps, ROS 2 and the RMW library, the imports, and the simulator's and the server's ports. The detector pilot runs on the Orin the same way, without the VLA flags: `perception_node.py --link mavros --scene-mode attach --sim-host <desktop-ip> --g1-verdict <copy of the desktop's g1_verdict.json>`, with the same environment.

What the VLA follow records beside the rail's usual records (CHANGELOG 2026-11-06, "perception VLA"):

- `vla_host`, the server's host by its own hello. The VLA is a separate process, so the run is hil only when that host is an Orin too.
- The object words, recorded as AIOT-VLA's grounding phrase.
- The follow's configuration, which is `vla_fly`'s own.
- `vla_observation`: the distinct FrontCamera frames that reached the VLA, per wall second, per sim second and while the pilot was being asked, with the duplicates, the gaps and the hint-gated spans counted apart. This is the grant's 10 Hz observation, measured from what was sent.

**Not flown yet.**

In attach mode the node never loads the scene. It checks that the simulator serves `/Sim/SceneCityLifeArduPilot/robots/Drone1/...` topics, and refuses with "Load it on the desktop first" if not. The named remote in `fcu_url` is what the hil evidence check reads. MAVROS's UDP transport replies to the address the router's packets come from, so replies reach the router even though its source port is not 14555; this has not been run here. The runbook recommends WSL mirrored networking when ArduPilot runs natively in WSL, so the Orin can reach it. Under NAT the router's outbound UDP to the Orin should pass, but that is unverified. The desktop must allow inbound TCP 8989/8990 from the Orin for the camera streams.

**Perception stress (since 2026-11-14; WP4-16, WP4-18; not flown yet).** One stressor per run, named in the suite as `pas-follow-redcar-<stressor>`: `day` (the baseline, the weather reset), `dusk` and `night` (every scene capture exposed 2 and 4 EV down), `rain` (RAIN 0.8) and `fog` (FOG 0.6). CityLife_Day has no `SunSky` actor, so Project AirSim's time-of-day and sun calls do nothing there ("No SunSky objects found" in the server log); darkness is therefore the cameras' exposure (`auto-exposure-bias` in the robot config's capture settings), and the level's lamps and car lights stay off. Rain and fog are the plugin's weather effects, set after the load and read back. A stressed scene has its own scene id (`SceneCityLifeArduPilotStressNight`, ...), so an attaching node told another stressor, or none, refuses it. `demo/pas_ardupilot/perception_stress.py` names every client and server source line behind this. A stressed run is never KPI-grade (kpi.json says why). `tools/perception_stress_report.py` scores the runs per scenario from what they log: detector hit rate and box score, the frames' mean grey level (`luma`, new in `detections.jsonl`), target lock (an estimate within 6 m of the truth), track losses, the range and the estimate's error.

```powershell
# desktop: one stressor per scene load (night here; day for the baseline)
.\scripts\run_pas_ardupilot.ps1 -Step scene -VlaCameras -Stress night
.\scripts\run_pas_ardupilot.ps1 -Step sitl -SkipSim -Orin 10.42.0.3
```

```bash
# AGX Orin, with the server and MAVROS up as above
G1_VERDICT=~/g1_verdict.json STRESS=night TAG=pas_stress_night_01 bash deploy/orin/run_pas_perception.sh
python tools/perception_stress_report.py demo/out/pas_stress_* --md demo/out/pas_stress_report.md
```

## What has not been verified yet

Settled by the runs of 2026-10-08 and moved to "Measured 2026-10-08": whether the level holds the physics step with ArduPilot in the loop (not at 3 ms; at 5 ms it ran at 0.951-0.967 of real time), whether the EKF converges on Project AirSim's sensors (p95 0.81-0.82 m), whether the airframe tuning flies in CityLife (an AUTO square within 1 m of every corner), whether the default G1 square clears the buildings (it does not at 10 m; it does at 15 m), and whether the stream to Mission Planner's port carries a flight (19.0 Hz). Still open:

- Mission Planner itself on the Windows side: the measurement is a listener on its UDP port, and Mission Planner's display was not recorded. Inbound UDP from the WSL adapter did reach Windows that day (ArduPilot's PWM on 9002 moved the vehicle in G1 condition 2; the listener on 14550 received the stream); whether a firewall rule or the default policy let it through was not recorded, and the launcher changes none.
- The MAVROS link (including `request_mode`, `time_reference`, `statustext` and `extended_state`) and the ROS 2 camera source were exercised only against stand-ins, never against a live MAVROS.
- The `dev` path: no WSL environment holds ROS 2 + MAVROS together with `torch`, `transformers` and the `projectairsim` client yet, so the MAVROS node has not been started in WSL either.
- The Orin path (`hil`): attach mode against a live simulator, the projectairsim client on aarch64, MAVROS's reply path through the router under WSL NAT, and DDS.
- The VLA follow on hil (2026-11-06, `deploy/orin/run_pas_perception.sh`). Four things have not run:
  - the perception node on the AGX Orin's host joining the companion container's MAVROS on one ROS 2 graph, with host ROS 2 Jazzy from apt;
  - AIOT-VLA's server on the Orin answering it;
  - FrontCamera's frames carried over the cable at 10 to 15 Hz;
  - whether ArduPilot's real-time factor stays in the band at a 0.066 s capture interval.
  It is tested with the rail's stand-ins and a scripted protocol-v2 server only.
- The KPI-evidence step (`guardrail.replay.kpi_evidence_grade`, which demotes a graded run whose replay bundle lacks a trusted signature) is wired as on the SITL rails but has never been reached. G1 has passed, but every live run so far (2026-10-08 to 2026-10-14) used pymavlink, which is offered no grant label. Since 93dcb10 (2026-10-13) `build_manifest()` accepts `dev`/`hil` beside a scene whose robot config it reads as `ardupilot-api`, but only on live MAVROS evidence (an `fcu_url`, no pymavlink link; `hil` also needs the Orin's evidence), and no MAVROS run exists on this rail yet.
- Land on this rail. On the live follow of 2026-10-08 the escalation FSM reached Loiter 8 times and never RTL or Land (what Loiter did to the altitude is defect (c) above). RTL has flown since: X7 sent the aircraft home on 2026-10-13 and 2026-10-14. Land is still tested with stand-ins only.
- Tracking on this rail. The Shield-off control, which the follow's P0 escape rate needed as its null, was flown on 2026-10-13 and 2026-10-14. What no run has shown yet is the detector on the car: it boxed a red traffic signal, and the rail had no start gate (see "Flown 2026-10-13"). The gate exists since 2026-10-21 (see "The perception node") and has not flown.
