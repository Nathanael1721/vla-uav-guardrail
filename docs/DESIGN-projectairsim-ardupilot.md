# Project AirSim flown by ArduPilot, with Mission Planner attached

**Date:** 2026-10-06, revised 2026-10-07 after review.
**Status:** designed, built and tested offline only. **Not yet flown.** No simulator, SITL, MAVROS or Mission Planner was started for this work. The commands to verify it live, and what they should print, are in "How to verify" below.
**Cards:** ARCH-15 (bridge, gate G1), ARCH-02 (ArduPilot is the camera rail's autopilot), ARCH-18 (perception-rail integration), WP4-16 (city flights on ArduPilot), ARCH-20 (plumbing for a real VLA on ArduPilot), ARCH-25 (a camera on the ArduPilot rail).
**Code:** `demo/pas_ardupilot/`, `scripts/run_pas_ardupilot.ps1`, `tests/test_pas_ardupilot.py`.
**Supersedes** the bridge premise of [DESIGN-hil-perception-bridge.md](DESIGN-hil-perception-bridge.md) (2026-09-07), whose options A and B assumed that a `HIL_GPS` / `HIL_SENSOR` bridge would have to be written. See "The PI's question" below.

## The PI's question, answered

> Can Project AirSim be connected to ArduPilot and to Mission Planner?

**Yes, by design and according to the shipped binaries. It has not yet been demonstrated on this desktop: gate G1 has not been run.**

The IAMAI fork of Project AirSim ships an ArduPilot flight controller (`"controller": {"type": "ardupilot-api"}`, class `ArduCopterApi`). With it, Project AirSim stops flying the vehicle itself. It sends its sensor readings to ArduPilot SITL and applies the motor commands ArduPilot sends back. ArduPilot upstream supports this interface (`sim_vehicle.py -v ArduCopter -f airsim-copter`).

**Mission Planner never talks to Project AirSim.** It talks MAVLink to ArduPilot through `mavlink-router`, as the grant specifies ("Mission Planner via mavlink-router fan-out, parallel to MAVROS"). Because ArduPilot flies the simulated vehicle, Mission Planner shows that flight: position, attitude, mode, mission and fence. A real aircraft is connected to Mission Planner the same way.

This changes the premise of the earlier scoping. The reference design (`dual-rail.md`) and `DESIGN-hil-perception-bridge.md` both assumed Project AirSim had no ArduPilot controller, so a `HIL_GPS` / `HIL_SENSOR` bridge had to be written. The binaries this project already runs contain one, so there is no bridge to write. What remains is to wire it correctly across WSL2, and to prove that it closes the loop. Gate G1 asks for exactly that proof.

## What runs where

```
 Windows                                               WSL2 (Ubuntu 24.04)
 ------------------------------------------------      ----------------------------------------
 Unreal Engine + Project AirSim (CityLife_Day)
   robot "Drone1", controller ardupilot-api
     every physics step (3 ms): one JSON line --UDP 9003-->  ArduPilot SITL Copter-4.5.7 (pinned)
       {timestamp, imu, pose, gps, velocity}                   --model airsim-copter, -w
     binds <local-host-ip>:9002  <--UDP 9002-- 11 x PWM ---    --home = scene home-geo-point
   FrontCamera / Chase streams (TCP 8989/8990)                  SERIAL0 tcp:5760
          |                                                          |
          v                                                     mavlink-router
 perception node (one process)                                  TCP server 5790 ---------+
   camera -> OWL-ViT -> servo pilot -> Shield + FSM -MAVLink->   UDP -> Windows:14550     |
   (vla-real env; pymavlink link today)          (TCP 5790)      UDP -> 127.0.0.1:14555 (MAVROS 2)
                                                                 UDP -> Orin:14555 (hil)  |
 Mission Planner  <-- UDP 14550 (or TCP 127.0.0.1:5790) ----------------------------------+
```

Until `mavlink-routerd` is installed in WSL, a MAVProxy fan-out stands in. It is announced on screen and in `endpoints.json` as "not the grant's router". A MAVProxy TCP output serves one client, so under the fallback the node keeps TCP 5790 and Mission Planner's TCP port is **5791**. MAVProxy also runs with `--streamrate=-1`, so it does not rewrite the stream rates set in the param file.

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
| Windows process -> router TCP 5790 | 127.0.0.1 (TCP is forwarded) | 127.0.0.1 |

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

## Gate G1, measured

`demo/pas_ardupilot/g1_check.py` turns the gate's four conditions into evidence files. Each condition answers `True`, `False` or `None` (not measured), and `None` never counts as a pass.

1. **The EKF converges on the simulator's position.** The 95th percentile of the 3-D error between ArduPilot's EKF (GLOBAL_POSITION_INT) and Project AirSim's ground-truth geo position, paired in time, must be within 1 m. **Null:** the same EKF track against a *frozen* truth. The run must beat the null by half and travel at least 10 m. Otherwise it cannot tell a working bridge from a parked vehicle under a converged EKF. **Transport note:** the frames travel as Project AirSim's JSON sensor frames, not as MAVLink `HIL_GPS` / `HIL_SENSOR`. What the condition measures is unchanged.
2. **ArduPilot's PWM moves the simulated vehicle.** **Null:** while disarmed, the vehicle moves under 0.3 m. After arming, the motor PWM rises above 1100 and the vehicle then climbs at least 2 m within 30 s, lifting off *after* the PWM rise. Chase-camera frames before and during the climb are saved as the viewport evidence.
3. **A square mission in AUTO** (10 m side, 10 m up, uploaded through the MAVLink mission protocol). The conditions:
   - AUTO is observed, and take-off and every corner report `MISSION_ITEM_REACHED`;
   - the vehicle lands, and ground truth passes within 2 m of every corner. The corner check is what fails a vehicle that never left the pad, which would "land within 2 m" trivially;
   - landing is within 2 m of take-off;
   - there are zero airborne collisions and zero fence breaches.
4. **Repeatability:** at least three runs with one seed, each passing conditions 1-3, with landing points and corner passes each spread by no more than 2 m. The seed fixes nothing random on our side of a scripted mission. The simulator's IMU noise is seeded inside Project AirSim; the record says so.

**A crashed run is a failed run, never a missing one.** Each `run` first logs its attempt (`g1_attempts.jsonl`), then writes `g1_run.json` from a `finally` block, with an `error` field if the run did not complete. `summarize --runs N` expects runs 1..N of one invocation. A run that started and left no record, crashed, or carries another attempt's record counts as failing conditions 1-3. A run folder left over from an earlier use of the tag fails condition 4. The launcher uses a fresh `g1_<timestamp>` tag and refuses to reuse one. The first version simply globbed the records that existed. Three passing runs plus one crashed run therefore read as PASSED (shown in review, and now a test).

## How to verify (run by the user; not run for this document)

```powershell
# 0. once: pymavlink in the flight env (pin as in sitl/setup_sitl.sh), and optionally
#    mavlink-router in WSL (otherwise the MAVProxy fallback is used and labelled)
<vla-real python> -m pip install pymavlink==2.4.49

# 1. addresses, firewall rules, packages, rendered configs - starts nothing
.\scripts\run_pas_ardupilot.ps1 -Step check
#    expect: "WSL2 networking: nat | Windows (as WSL sees it) 172.x.x.1 | WSL 172.x.x.y"
#    and a dry-run JSON with "ok": true once pymavlink is installed.
#    The ArduPilot pin line, as measured on 2026-10-06, is PIN FAIL: the binary is
#    "ArduCopter V4.5.7 (2a3dc4b7)" but the modules/mavlink/pymavlink submodule
#    has drifted (sitl/setup_sitl.sh explains it and how to restore the pin).
#    The flight still runs; the record says which firmware flew.

# 2. gate G1: three square-mission runs and the verdict (close the Unreal editor first)
.\scripts\run_pas_ardupilot.ps1 -Step g1
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
- then periodic `FPS avg=...` near 333 at real time.

**A repeating `No sensor message received in last 1s` means the simulator's frames are not reaching WSL.** Check the rendered `ardupilot-ip`, and the firewall on UDP 9002 for the return path.

**Mission Planner:** choose UDP, port 14550, and Connect. Alternatively use TCP 127.0.0.1:5790, or 5791 under the MAVProxy fallback (`start_ardupilot.sh` prints which). Expect:

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

## What could not be verified here

- **No live run of any kind.** Unmeasured:
  - whether the CityLife level holds the 3 ms physics step with ArduPilot in the loop (the node now measures the real-time factor);
  - whether the EKF converges on Project AirSim's IMU noise;
  - whether the airframe tuning flies well in CityLife.

  G1 exists to measure exactly these.
- The firewall on this PC's WSL adapter, and Mission Planner's connection.
- `pymavlink` is not installed in any Windows environment, and `mavlink-routerd` is not installed in WSL (checked 2026-10-06). MAVProxy is present in `~/venv-ap`, so the labelled fallback router is used until mavlink-router is installed. `wslinfo --networking-mode` reports `nat`.
- The MAVROS link (including `request_mode`, `time_reference`, `statustext` and `extended_state`) and the ROS 2 camera source were exercised only against stand-ins, never against a live MAVROS.
- The `dev` path: no WSL environment holds ROS 2 + MAVROS together with `torch`, `transformers` and the `projectairsim` client yet, so the MAVROS node has not been started in WSL either.
- The Orin path (`hil`): attach mode against a live simulator, the projectairsim client on aarch64, MAVROS's reply path through the router under WSL NAT, and DDS.
- The KPI-evidence step (`guardrail.replay.kpi_evidence_grade`, which demotes a graded run whose replay bundle lacks a trusted signature) is wired as on the SITL rails but is never reached by the offline tests: no stand-in run is KPI-grade, because `build_manifest()` refuses `dev`/`hil` beside a Project AirSim scene and G1 has not passed.
- The escalation FSM on this rail was tested with stand-ins only. Whether a live follow ever reaches Loiter or RTL depends on the policy and the flight.
- The default G1 square (10 m north and east of the spawn, 10 m up) was not checked against CityLife's buildings. `--side-m` and `--alt-m` move it.
