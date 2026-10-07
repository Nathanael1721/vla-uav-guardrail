# The grant's ROS 2 interface on the ArduPilot rail

**Date:** 2026-10-06, updated 2026-10-07
**Cards:** WP3-03, WP3-04, WP3-09, WP3-10, WP3-11, WP3-17, WP3-24, ARCH-04,
ARCH-05, ARCH-09, ARCH-10, ARCH-11, ARCH-22, ARCH-31, WP1-18, WP1-24, WP2-11,
WP2-15, WP2-17, WP4-11, WP4-13, WP4-20
**Code:** `sitl/ros2_shield_node.py`, `sitl/mavlink_adapter_node.py`,
`sitl/ros2_vla_stub_node.py`, `sitl/ros2_ws/`, `sitl/fence/`,
`sitl/mavlink-router/`, `sitl/start_router.sh`, `sitl/run_ros2_demo.sh`,
`sitl/gcs_listen.py`, `guardrail/replay.py`, `guardrail/manifest.py`
**Tests:** `tests/test_mavlink_adapter.py`, `tests/test_ros2_package.py`,
`tests/test_sitl_rails.py`, `tests/test_replay.py`, `tests/test_manifest.py`

This is the design of the ROS 2 side of the Safety Shield as the grant fixes it,
and the record of the SITL flights that exercised it. It follows the grant
wherever the grant speaks, and the PI's reference implementation
(`kuanting-vla-uav-guardrail/`) where the grant is silent.

## The graph

```
vla node   --/vla/action_4d------> safety_shield --/shield/setpoint----> mavlink_adapter --/mavros/setpoint_raw/local--> MAVROS 2
             Float32MultiArray                     Float32MultiArray                       mavros_msgs/PositionTarget
             body [vx, vy, vz,                     body, post-Shield                       FRAME_LOCAL_NED,
             yaw_rate] 10 Hz                                                               type_mask 1479

vla node   --/vla/identity (latched JSON: model id, kind, frame, policy hash, host)--> safety_shield
safety_shield --/shield/mode_request (String)----> mavlink_adapter --> /mavros/set_mode
safety_shield --/shield/fences (latched JSON)----> mavlink_adapter --> /mavros/geofence/push
mavlink_adapter --/mavlink_adapter/status (latched JSON)--> safety_shield (events.jsonl)
safety_shield --/shield/intercept (String JSON)--> rosbag2

ArduPilot SITL --SERIAL2 udp--> mavlink-router --+--> MAVROS 2 (udp 14555)
                                                 +--> Mission Planner (Windows, udp 14550)
```

The topic names, the message types and the type mask are the reference's
(`ros2_ws/src/safety_shield_node`, `ros2_ws/src/mavlink_adapter`).

## The frame chain

The grant locks the action as "a = (vx, vy, vz, yaw_rate) # body-frame
velocities + yaw rate" (Architecture constraints p2) and puts the conversion to
NED in the MAVLink adapter, after the Shield. Our Shield's checks are written
in the world frame (a fence is a polygon on the ground), so the chain is:

1. The VLA node publishes body frame. The stub plans in the world frame and
   rotates its answer with `guardrail.frames.to_body` and the heading of
   `/mavros/local_position/pose`.
2. The Shield node rotates the body action once into the world frame
   (`frames.from_body`, same heading source), lets the Shield check and repair
   it, and rotates the result back (`frames.to_body`). With the Shield off the
   body action passes through untouched. The Shield therefore takes a body
   action and returns a body action; the rotation is exact.
3. The adapter turns `/shield/setpoint` into local NED (`from_body`, then
   `to_local_ned`): the single body -> NED crossing.
4. MAVROS expects ENU in `PositionTarget` and converts it to NED itself: for
   `FRAME_LOCAL_NED`, setpoint_raw applies `ftf::transform_frame_enu_ned`
   ((x, y, z) -> (y, x, -z)) to the velocity and flips the yaw-rate sign
   before it sends `SET_POSITION_TARGET_LOCAL_NED`. So the adapter fills the
   message in ENU (x = east, y = north, z = up, yaw rate counter-clockwise),
   and MAVROS sends exactly the NED setpoint.

`test_what_mavros_sends_is_the_ned_setpoint_for_every_heading` checks the
adapter on a model of MAVROS's conversion for every heading in 15 degree
steps, and `test_ned_values_packed_unconverted_would_reach_ardupilot_swapped_and_inverted`
pins why the packing matters.

On ArduPilot SITL the flown displacement followed the commanded world
direction. Comparing each tick's Shield output (`emitted`) with the
displacement over the next 1 s (windows with at least 1 m/s commanded and
0.5 m flown), the median angular error was 0.38 deg over 199 windows
(`ros2fix_on_dyn_yaw`), 0.57 deg over 192 (`ros2fix_on_unsafe_start`), and
0.48 deg and 1.54 deg on the two yaw runs of 2026-10-06. A random direction
would give a median of 90 deg. The commands pointed away from the north-east
diagonal in nearly every window (199 of 199, 192 of 192, 185 of 195, 213 of
224), where a north/east swap would show as an error of twice that offset.
Commanded climbs flew as climbs: +0.53 m/s commanded against +0.49 m/s flown,
mean over 6 windows, the sign agreeing in all 6 (`ros2fix_on_dyn_yaw`).

## Packaging

Two `ament_python` packages under `sitl/ros2_ws/src`, named as in the
reference:

| Package | Executables | Launch |
|---|---|---|
| `safety_shield` | `shield_node`, `vla_stub_node` | `guardrail_rail.launch.py`: the stub, the Shield and the adapter; the rail shuts down when the Shield node (the mission) exits |
| `mavlink_adapter` | `mavlink_adapter` | `mavlink_adapter.launch.py`: the adapter alone (e.g. on the Orin beside MAVROS) |

The entry points import the repository's `sitl/*.py` (found from
`GUARDRAIL_ROOT`, or from the source tree a symlink install resolves to), so
there is one copy of the node code. The launch files run the nodes with the
interpreter that has the Guardrail's dependencies next to `rclpy`
(`python:=`, default `~/venv-ros/bin/python`).

Built on 2026-10-06 in WSL with `colcon build --symlink-install --base-paths
sitl/ros2_ws/src --build-base ~/guardrail_ws/build --install-base
~/guardrail_ws/install`: both packages finished (13.3 s), `ros2 pkg
executables` lists the three executables, and the installed module resolves
the repository root. `sitl/run_ros2_demo.sh` starts the same launch file from
the source tree, so no build is needed to fly.

## Mission Planner and mavlink-router

`sitl/mavlink-router/main.conf` is the reference's fan-out (dev compose:
`mavlink-routerd -e mavros:14555 -e host:14556 0.0.0.0:14550`) with Mission
Planner on its default UDP port 14550 on the Windows host:

- ArduPilot SITL SERIAL2 -> `udpclient:127.0.0.1:14550` (the router listens);
- router -> MAVROS 2 on UDP 14555 (`fcu_url udp://:14555@`);
- router -> Mission Planner, `<Windows host>:14550`.

`start_router.sh` runs `mavlink-routerd` when it is installed and otherwise the
same fan-out on MAVProxy 1.8.74 (already in `~/venv-ap`), with
`--streamrate=-1` so it never changes the stream rates MAVROS asks for. The run
record names the one used (`hil_evidence.mavlink_router`); in every flight
below it was MAVProxy. MAVProxy is itself a ground station: it downloads the
autopilot's parameter set (1381 parameters over MAVLink FTP) at start, and the
replies reach every output.

**Does the operator's port carry the flight?** `sitl/gcs_listen.py` stands in
for Mission Planner: it listens on UDP 14550 on Windows, never sends, records
when each message type arrived, and `check` compares that with the mission
window in the run's own `events.jsonl`. It passes only if GLOBAL_POSITION_INT
and ATTITUDE (what Mission Planner draws the aircraft from) arrived throughout
the mission with no gap over 2 s. On 2026-10-07:

| Run | GLOBAL_POSITION_INT | ATTITUDE | Largest gap |
|---|---|---|---|
| `ros2fix_off_fence` | 284 in 28.4 s (10.0 Hz) | 284 (10.0 Hz) | 0.1 s |
| `ros2fix_on_unsafe_start` | 210 in 21.0 s (10.0 Hz) | 210 (10.0 Hz) | 0.1 s |

The listener has to run under an interpreter that Windows Firewall lets
receive inbound UDP; Mission Planner has an inbound Allow rule on this PC.

**The PI's question - can Project AirSim connect to Mission Planner?** Mission
Planner connects to ArduPilot, not to the simulator. Project AirSim (the IAMAI
fork at `D:/ProjectAirSim/repo`) has a native ArduPilot controller: the robot
config sets `"controller": {"type": "ardupilot-api", "ardupilot-settings":
{...}}` with UDP ports 9002/9003 (`client/python/example_user_scripts/
ardupilot/sim_config/robot_ardu_quadrotor.jsonc`), and ArduPilot SITL is
started with `sim_vehicle.py -v ArduCopter -f airsim-copter` (plus the
`project-airsim-quad.param` gains). Project AirSim then provides the physics,
sensors and rendering; ArduPilot flies the drone and speaks MAVLink like any
SITL, so Mission Planner attaches to it over UDP 14550 - directly through
`sim_vehicle.py`'s MAVProxy output, or through this router beside MAVROS.
The IAMAI example script's own docstring says Mission Planner can be used to
control the drone. That combination has not been run in this unit (Project
AirSim was not started here); the router side has, with the results above.

## The GeoFence backstop

"The autopilot's own GeoFence / FENCE_ACTION is the ultimate backstop"
(Architecture constraints p1).

- `sitl/fence/guardrail_fence.parm`, loaded by `start_sitl.sh` as a second
  defaults file: `FENCE_ENABLE 1`, `FENCE_TYPE 5` (max altitude + polygons and
  circles), `FENCE_ACTION 1` (RTL or Land), `FENCE_ALT_MAX 30`, and
  `AVOID_ENABLE 0` (below). Every line stays under 98 characters: ArduPilot
  reads a defaults file 98 characters at a time and would parse the rest of a
  longer line as a new one.
- **Read back, not assumed.** At take-off the Shield node reads
  `FENCE_ENABLE`, `FENCE_TYPE`, `FENCE_ACTION`, `FENCE_ALT_MAX`,
  `FENCE_MARGIN` and `AVOID_ENABLE` from the autopilot through `/mavros/param`
  and records them (`hil_evidence.autopilot_fence`, with `backstop_active`).
  When `FENCE_ALT_MAX` sits below the policy's ceiling + 5 m it is raised
  through `/mavros/param/set_parameters`: 18 of the policies in `policies/`
  allow more than 25 m (most 55 m, one 80 m), and with 30 m the backstop
  would fire on a flight the Shield considers legal. The pymavlink rail does
  the same over PARAM_VALUE / PARAM_SET.
- The zones come from the policy. The Shield node publishes each generation's
  keep-out zones on `/shield/fences`; the adapter converts them to lat/lon
  about the autopilot's home (the policy loader's equirectangular projection)
  and uploads them as exclusion polygons (5002) and circles (5004) through
  `/mavros/geofence/push`, then reads them back. The zones go up as authored,
  without the Shield's margin. A zone is exported only if its altitude band
  covers 0 .. `FENCE_ALT_MAX`, because ArduPilot's polygons are 2-D. Keep-in
  corridors are not exported. A `circle_fence` (centre + radius) goes up as an
  exclusion circle with its exact radius.
- **A zone over the aircraft is held back.** A zone the aircraft is inside, or
  within `FENCE_MARGIN` of, is not uploaded until the aircraft is clear of it
  and the Shield no longer finds the position unsafe under that rule. A zone
  hot-applied on top of the aircraft is then the Shield's to resolve, and the
  GeoFence fires only if the Shield fails to (the grant: "fire only if the
  Shield itself crashes or fails to emit"). Before the first pose the aircraft
  is taken to be on the pad, at home. Flown in `ros2fix_on_unsafe_start`
  below. A zone that covers home is named in the record (`zones_over_home`):
  any RTL, the FSM's or the GeoFence's own, flies back into it.
- A policy anchored to lat/lon whose frame origin is more than 5 m from the
  autopilot's home is refused at upload, with the offset recorded: its zones
  would sit in the wrong place in the Shield and in the fence alike.
- The EEPROM is wiped at each SITL start, because uploaded fences persist
  there and a zone left by an earlier hot-apply would be armed from boot.
- **Simple avoidance is off.** ArduCopter's `AVOID_ENABLE` (default on, bit 0
  "UseFence") also uses the fence and, in GUIDED velocity control
  (`ArduCopter/mode_guided.cpp`, `copter.avoid.adjust_velocity`), bends every
  setpoint that points within `FENCE_MARGIN` of a zone. A shield-off flight
  with avoidance on stopped about 2 m short of the zone and hung there for
  20 s before it drifted in and the fence fired. `AVOID_ENABLE 0` keeps the
  fence a backstop that acts on a breach only, and keeps Shield-on runs free
  of a second filter behind the Shield.

## What every run records

| Record | Cards | How |
|---|---|---|
| `unsafe`, `unsafe_rules` per tick | WP3-17 | `Shield.state_is_unsafe` (`guardrail.replay.episode_row_fields`) |
| `policy_hash`, `generation`, `episode_id` per tick | WP4-13, WP1-24 | same |
| `flown` per tick, and `emitted_violations` = the check on what was flown | WP3-17 | `guardrail.replay.flown_fields` |
| `shield_ms` per tick | WP3-13 | `Shield.history[-1].elapsed_ms` |
| declared subject per tick (`tgt_x`, `tgt_y`) | WP3-17 | what `guardrail/kpi.py` reads |
| both frames per tick (`raw_body`, `emitted_body`; `raw`, `emitted` world) | WP3-04 | `build_row` |
| FSM state per tick (`fsm_state_before/after`, `fsm_edge`, `failsafe`), and the same FSM's verdict in every audit record | WP3-07, WP3-11 | `guardrail.fsm`, `guardrail.replay.audit_tick` |
| `policy_g<N>.json`, `csp_g<N>.json` per generation | WP2-11, WP2-15 | `EpisodeRecord` |
| a fresh `audit.jsonl` per episode; each record carries the policy hash, the generation, the episode id and the tick number of its log row | WP4-20, WP3-11 | `rotate_stale_episode`, `AuditLogger(..., episode_id=...)` |
| `events.jsonl`: generations, hot-applies, mode requests/changes, fence uploads and deferrals, FSM transitions, autopilot text, `mission_end` | WP4-13 | `EpisodeRecord.event` / `end` |
| `adapter_log.jsonl`: body in, heading, NED, ENU message, what MAVROS sends | WP4-13 | the adapter |
| the autopilot's fence parameters as read back | WP3-10, ARCH-09 | `read_autopilot_fence` |
| `bag/`: rosbag2 (mcap) of `/vla/action_4d`, `/shield/*`, `/mavros/setpoint_raw/*`, `/mavros/state`, `/mavros/local_position/pose` | ARCH-22 | `run_ros2_demo.sh` |
| `<tag>.replay.tar.gz` with all of the above | WP4-11, WP1-18 | `write_replay`, re-packed after the bag closes |
| autopilot version as the autopilot reports it | ARCH-31 | `AUTOPILOT_VERSION` via `/mavros/vehicle_info_get` |
| pilot record (what flew, from `/vla/identity`) | ARCH-06, X-16 | `guardrail.manifest.pilot_record` |

**What was flown.** `flown` is false on a tick the rail sent nothing because
the autopilot (a GeoFence RTL) or a fault held the aircraft; such a tick's
`emitted_violations` is empty, so it cannot count as an escape. On a Brake tick
the check is that of the zero action actually sent.

**One episode, not two.** A run tag is reused on purpose. Starting an episode
moves every file an earlier flight wrote (log, metrics, KPI table, manifest,
report, bundle, audit, events, generations) to `_previous/<stamp>/`; every
row, `metrics.json` and `kpi.json` carry the episode id (the `episode_start`
time); and `write_replay` / `verify_replay` refuse a log, metrics or KPI table
carrying another episode's id, and an episode whose `events.jsonl` has no
`mission_end`. `python -m guardrail.replay pack`, which `run_ros2_demo.sh`
runs after every launch, therefore refuses the directory of a flight whose
Shield node died, instead of bundling the previous flight's records with it.

Audit records that carry an episode id must carry this one too (records
written before 2026-10-07 carry none and bind nothing).

`write_replay` and `verify_replay` also refuse an episode whose log or audit
names a policy hash that none of its archived generations has, and a CSP
compiled from another generation. `write_replay` leaves out a rosbag that is
still open or does not overlap the episode's start, and the index says why;
`verify_replay` names rosbag topics that recorded nothing.

**KPI evidence is signed evidence.** Both rails pass a run that `is_kpi_grade`
accepts through `guardrail.replay.kpi_evidence_grade`, which demotes it unless
its replay bundle verifies with a trusted signature
(`verify_replay(..., require_signature=True)`), and rewrites `kpi.json` and
the bundle to say so.

## Topology from evidence: desktop today, Orin next, drone later

The PI's decision of 2026-10-06: the Jetson Orin is the `hil` machine, and the
stack must move onto the drone (`flight`) with no code change. So the Shield
node gathers the evidence and `guardrail.manifest.detect_topology` picks the
label; there is no topology flag.

| Label | Evidence the node must record (`metrics.json` -> `hil_evidence`) |
|---|---|
| `dev` | ROS distro, a MAVROS node, `fcu_connected`, the autopilot's own version |
| `hil` | the above + the Shield's host is an Orin (`host_arch` aarch64, a device tree naming an Orin, L4T or JetPack read) + the VLA node's host is an Orin (`vla_host`, from `/vla/identity`) + `autopilot_kind` sitl (SIM_SPEEDUP readable) + the SITL autopilot runs off the Shield's host |
| `flight` | the Orin evidence (Shield and VLA) + `autopilot_kind` hardware (SYSID_THISMAV present, no SIM_ parameter) |

"Off the Shield's host" is read from the host itself: the node scans `/proc`
for ArduPilot SITL, MAVROS and router processes (`local_processes`) and
records where MAVROS ran (`mavros_on`). Both layouts qualify:

- the grant's table, "Desktop runs sims + MAVROS + GCS; Jetson Orin runs VLA +
  Shield over network": no MAVROS process on the Orin (`mavros_on: remote`);
- `docs/RUNBOOK-orin-hil.md` and `deploy/topologies/hil.env`: MAVROS on the
  Orin with `fcu_url udp://:14555@` (`mavros_on: shield_host`), fed by
  `start_router.sh` on the desktop with `MAVROS_HOST=<Orin IP>`.

A SITL process on the Shield's host, or MAVROS there naming a loopback
autopilot, keeps the label at `dev`. A Shield node inside a container sees only
its own PID namespace; run it with `pid: host` (or MAVROS in the same
container) for `mavros_on` to describe the host. `build_manifest` refuses
`hil` and `flight` without the evidence, and `is_kpi_grade` re-checks it for
`hil`, so a hand-edited label is not quoted. JetPack 6 is Ubuntu 22.04, i.e.
ROS 2 Humble; the nodes use only rclpy APIs common to Humble and Jazzy, and
the Guardrail package runs on Python 3.10.

## Escalation FSM wiring

With the Shield on, each decision goes through `guardrail.fsm.EscalationFSM`
as `docs/DESIGN-escalation-fsm.md` ("Integration") specifies; its mode
requests go to the adapter on `/shield/mode_request`, which forwards only the
modes the FSM can ask for (GUIDED, LOITER, RTL, LAND) and refuses any other.
The mission-end LAND is never marked a fail-safe and is not sent when the FSM
or the autopilot already has the aircraft in RTL / Land. Theta is applied to
`Repair.magnitude_m` when `guardrail/shield.py` reports it, and to the
velocity proxy (h = 0.1 s, which cannot fire on these flights) when it does
not; the node picks the source by itself and writes it into `metrics.json`
(`fsm.theta_basis`); the pymavlink rail keeps the proxy. `magnitude_m` arrived
in the Shield during 2026-10-07, so the flights below say which source each
one used. If the autopilot leaves GUIDED without being asked (a GeoFence
breach), the node records the takeover, stops streaming and stops stepping the
FSM. The adapter forwards setpoints only while the Shield node is the single
publisher on `/shield/setpoint`.

**One FSM per rail.** Since 2026-10-07 the Shield can also run the escalation
FSM inside `filter()` (`Shield(escalation=...)`, on by default), and
`AuditLogger.log` records that FSM's verdict unless the caller passes its own.
Only the node sees the autopilot (its mode, home reached, landed, a takeover),
so the rails build their Shield without an FSM (`guardrail.replay.rail_shield`)
and write each audit record after the node's FSM stepped, with its verdict
(`guardrail.replay.audit_tick`). Before that, both FSMs ran on the same
decisions and the audit recorded the Shield's: on `ros2fix_on_dyn_nfz`,
`audit.jsonl` and `flight_log.jsonl` gave different FSM states on 3 of 83
ticks (the Shield's FSM entered RTL one tick early and never saw home
reached). On `ros2fix2_on_proxy`, flown after the change, all 217 audit
records match their log rows.

Two flights with `magnitude_m` (below) took the FSM past Brake on ArduPilot:
Brake -> Loiter [G3] and Loiter -> RTL [X6], each mode set through the adapter
and confirmed by `/mavros/state`. On the default mission the Shield's first
geofence repair carries a forecast penetration of 8.1 m (5 s ahead at the
cruise speed, through the middle of `nfz-square`) against theta 2.0 m, so
the FSM leaves Normal at the first tick. In ArduCopter's LOITER with no RC
input, as on this SITL, the aircraft descended from 14.5 m to the ground in
about 6.5 s.

## Shield horizon

Both SITL rails run the Shield with the grant's horizon: "5 s of predicted
trajectory at 10 Hz = 50 future poses" (Safety Shield page), recorded per run
as `shield_lookahead`. The Shield's own time per tick on the nine 2026-10-07
flights: p50 1.0-2.0 ms, p99 1.3-3.9 ms against the 100 ms tick budget. The
first mission tick took 116 ms on `ros2fix_on_dyn_yaw` and 33 ms on the
pymavlink run `sitlfix_on_dyn`. The ROS 2 node now makes one Shield call
before the mission, on a throwaway Shield, and records its time
(`shield_warmup`). On the seven ROS 2 flights since, that call took between
1.2 ms and 202.7 ms, and the first mission tick 1.0-2.2 ms: the slow call is
the process's first Shield call, not the mission's. What makes it slow is not
established. The pymavlink rail has no warm-up call yet.

## Flown (WSL, ArduCopter V4.5.7 (2a3dc4b7), dev topology)

All runs: ArduPilot SITL at speedup 1.0 (read from the autopilot), MAVProxy
as the router, MAVROS 2 over UDP, the GeoFence parm loaded, policy
`fase3-sim-demo` v0.1.1 from YAML (unsigned), StubVLA. None is KPI-grade: the
topology is `dev` and the working tree was uncommitted (the manifest says
both). Distances are from the `flight_log.jsonl` rows (10 Hz) to the zone's
authored edge.

2026-10-07, after one FSM per rail (one tick number per decision from
`ros2fix2_on_proxy` on, audit episode ids on `ros2fix2_off_fence`), on the
Shield as it stood after that morning's escalation and `magnitude_m` changes:

| Tag | Setup | Result |
|---|---|---|
| `ros2fix2_on_proxy` | Shield on, no hot-apply; theta on the velocity proxy (`--theta-horizon-s 0.1`) | target reached in 22.4 s; 0.0 s inside any zone, never closer than 1.68 m to `nfz-square`; P0 escapes 0 of 126 P0 ticks, all 222 ticks flown; FSM Normal -> Brake [G1] at the first tick; the 217 audit records give the same FSM state, edge and tick number as their log rows; Shield p50 1.6 ms, p99 3.0 ms; flown versus commanded direction: median 0.5 deg over 207 one-second windows |
| `ros2fix2_on_fsm` | Shield on, no hot-apply; theta on `magnitude_m` (the default) | Normal -> Brake [X1] -> Loiter [G3] at 0.2 s (geofence repair magnitude 8.1 m > theta 2.0 m); LOITER descended from 14.2 m to the ground; Loiter -> RTL [X6] at 8.1 s, home reached [G9]; a setpoint (Brake) was flown on 2 of 82 ticks and the FSM's LOITER / RTL held the other 80; audit and log agree on all 82 records (flown before the tick-number fix, so audit tick k is log row k + 1) |
| `ros2fix2_off_fence` | Shield off | "Fence Breached", GUIDED -> RTL at 3.35 s; 6.3 s inside the zone, at most 5.7 m deep; of 283 ticks the VLA's action was flown on 33 (all 33 P0 escapes) and the RTL flew 250 (`flown: false`); time to safe 6.7 s (1 episode) plus 1 censored; all 283 audit records carry this episode's id |

2026-10-07, earlier, after the fixes above (5 s horizon, `flown`, episode ids,
fence read-back, zone deferral). The FSM's theta source is given per run:

| Tag | Setup | Result |
|---|---|---|
| `ros2fix_on_dyn_yaw` | Shield on, NFZ hot-applied at 8 s (polygon_fence), VLA yaw rate 0.2 rad/s; theta on the velocity proxy | target reached in 22.6 s; 0.0 s inside any zone, never closer than 1.72 m to `nfz-square` or 14.2 m to `nfz-dynamic`; P0 escapes 0 of 125 P0 ticks; turned 254 deg; altitude 14.2-15.0 m; fence g0 (4 items) and g1 (8 items) uploaded and read back; read back from the autopilot: FENCE_ENABLE 1, FENCE_ALT_MAX 30 (policy ceiling 20 m), AVOID_ENABLE 0; pose 5.9 Hz |
| `ros2fix_off_fence` | Shield off | "Fence Breached", GUIDED -> RTL at 3.36 s; 6.3 s inside the zone, at most 5.9 m deep; of 284 ticks, the VLA's action was flown on 33 (all 33 P0 escapes) and the autopilot's RTL flew 251 (`flown: false`); time to safe 6.8 s (1 episode) plus 1 censored (the RTL descent below the 10 m floor when the episode ended); GCS port check passed |
| `ros2fix_on_unsafe_start` | Shield on, a 10 x 10 m zone hot-applied centred on the aircraft at 8 s (polygon_fence); theta on the velocity proxy | the Shield steered out: time to safe 3.2 s (1 episode, none censored), 2.6 s inside the zone; P0 escapes 0 of 114 P0 ticks; the zone was held back from the autopilot fence and uploaded 3.7 s later, once clear; no fence breach and no autopilot takeover; target reached in 21.0 s; GCS port check passed |
| `ros2fix_on_policy55` | Shield on, `policies/poly_test.yaml` (altitude band 35-55 m, flown at the mission's 15 m); theta on `magnitude_m` | FENCE_ALT_MAX raised from 30 to 60 m and read back (policy ceiling 55 m); `tri-nfz`, whose vertex is on home, held back on the pad, uploaded 2.0 s into the mission once clear, and named as covering home (`zones_over_home`); the FSM went Normal -> Brake [G1] -> Loiter [G3] at 1.4 s -> RTL [X6] at 5.1 s (position below the 35 m floor) -> home reached [G9]; the GeoFence reported "Fence Breached" at 26.5 s, after the RTL had brought the aircraft home into `tri-nfz`; of 102 ticks, a setpoint was flown on 14 (0 P0 escapes) and the FSM's LOITER / RTL held the other 88 (`flown: false`) |
| `ros2fix_on_dyn_nfz` | Shield on, NFZ hot-applied at 8 s, sent as a dynamic_nfz (the Shield's mid-flight update model); theta on `magnitude_m` | Brake -> Loiter [G3] at 0.2 s (geofence repair magnitude 8.1 m > theta 2.0 m), LOITER descended to the ground, Loiter -> RTL [X6] at 8.2 s; target not reached; the dynamic_nfz uploaded to the autopilot fence (8 items read back); generation 1's CSP not compiled (the compiler does not yet render dynamic_nfz), which the bundle states |
| `sitlfix_on_dyn` (pymavlink rail, `--seed 3`) | Shield on, NFZ hot-applied at 8 s (polygon_fence); theta on the velocity proxy | target reached in 25.6 s; 0.0 s inside any zone; P0 escapes 0 of 122 P0 ticks; fence parameters read back over PARAM_VALUE (FENCE_ENABLE 1, FENCE_ALT_MAX 30, AVOID_ENABLE 0); generations 0 and 1 written, their CSPs not compiled because `~/venv-ap` has no jinja2, which the record and the bundle say |

2026-10-06 (3 s horizon at 0.5 s; rows without `flown` or episode ids):

| Tag | Setup | Result |
|---|---|---|
| `ros2if_on_dyn_yaw` | Shield on, NFZ hot-applied at 8 s, yaw rate 0.2 rad/s; simple avoidance still on | target reached in 24.6 s; 0.0 s inside any zone, never closer than 2.84 m to `nfz-square`; P0 escapes 0 of 135 P0 ticks; turned 261 deg; altitude 14.2-15.0 m; the adapter's NED direction matched the Shield's world output within 1.6 deg (median 0.0); fence g0 (4 items) and g1 (8 items) uploaded and read back |
| `ros2if_off_fence` | Shield off, simple avoidance on | the autopilot's avoidance held the aircraft about 2 m short of the zone for 20 s; then "Fence Breached", GUIDED -> RTL at 23.8 s, recorded as an autopilot takeover. This run is why `AVOID_ENABLE 0` is in the parm |
| `ros2if_off_fence_noavoid` | Shield off, avoidance off | "Fence Breached", GUIDED -> RTL at 3.4 s; 6.4 s inside the zone, at most 5.9 m deep, all of it under the RTL; P0 escape rate 1.0 over the 33 ticks on which the VLA's action was flown, the RTL flying the remaining 250; time to safe 6.9 s (1 episode) plus 1 censored (the RTL descent below the 10 m floor) |
| `ros2if_on_dyn_yaw_noavoid` | Shield on, NFZ hot-applied at 8 s, yaw rate 0.2 rad/s, avoidance off | target reached in 21.5 s; 0.0 s inside any zone, never closer than 1.78 m to `nfz-square`; P0 escapes 0 of 125 P0 ticks; the fence never fired; turned 246 deg; Shield time p99 0.9 ms; pose 9.8 Hz |
| `sitl_if_on_dyn` (pymavlink rail, `--seed 3`) | Shield on, NFZ hot-applied at 8 s | target reached in 23.2 s; 0.0 s inside any zone; P0 escapes 0 |

Fixed on the way: `sitl/run_sitl_demo.py` printed `verify_replay`'s notes as
the reasons a run was not KPI-grade (pinned by
`test_the_kpi_grade_reasons_are_not_overwritten_before_the_report`); both
rails counted a shield-off tick as an escape whether or not anything was
flown on it (`test_a_tick_the_autopilot_flew_is_not_an_escape`); a re-run
that died before writing its log left the previous flight's records to be
bundled (`test_a_crashed_rerun_does_not_bundle_the_previous_flights_log`);
once the Shield gained its own escalation FSM, both rails ran two FSMs and the
audit recorded the one that never saw the autopilot
(`test_a_rail_runs_one_escalation_fsm_and_its_audit_records_that_one`); and
the ROS 2 node numbered each audit record one below its log row
(`test_a_ticks_audit_record_and_log_row_carry_the_same_tick_number`).

## Scope and next steps

- The router runs on MAVProxy until `mavlink-routerd` is built and installed;
  `start_router.sh` switches by itself and the run record names the router.
- Mission Planner itself has not yet been attached to a run; `gcs_listen.py`
  stands in for it on the same port.
- The first Orin run follows `docs/RUNBOOK-orin-hil.md`; the hil and flight
  evidence rules are unit-tested.
- The pymavlink rail uploads no zones (its altitude fence applies).
- Audit records written before 2026-10-07 carry the policy hash only; the
  generation files map it to a generation.
- `/mavros/setpoint_raw/target_local` is recorded, and ArduPilot does not
  stream POSITION_TARGET_LOCAL_NED at the requested rates, so it is empty.
- `--start X,Y` (fly to a scenario's start point first) is next; `--target`,
  `--speed`, `--seed`, `--max-s` and `--dynamic-zone` are in.
