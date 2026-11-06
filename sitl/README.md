# ArduPilot SITL rails

Same Guardrail, real autopilot. Two ArduPilot rails live in this folder, beside
the perception rail on Project AirSim (`demo/`):

| Rail | Entry point | Path to ArduPilot | Topology label |
|---|---|---|---|
| ROS 2 (the grant's interface) | `run_ros2_demo.sh` | VLA node -> Shield node -> MAVLink adapter -> MAVROS 2 -> mavlink-router -> SITL | `dev` on one desktop; `hil` / `flight` on a Jetson Orin, from evidence |
| pymavlink | `run_sitl_demo.py` | in-process pymavlink, `SET_POSITION_TARGET_LOCAL_NED` | `ardupilot-sitl-pymavlink` |

The design, the topics and what was flown are in
`docs/DESIGN-ros2-interface.md`.

## One-time setup (WSL Ubuntu 24.04)

```bash
bash setup_sitl.sh          # ArduPilot at the pinned Copter-4.5.7 (2a3dc4b7), venv ~/venv-ap, build (~15 min)
bash setup_sitl.sh --verify # read-only: does ~/ardupilot match the pin?
sudo bash setup_ros2.sh     # ROS 2 Jazzy + MAVROS 2, venv ~/venv-ros
```

`setup_sitl.sh` checks out the pinned tag Copter-4.5.7 = 2a3dc4b7 (it no
longer clones master). `--verify` changes nothing and fails on a wrong commit,
a submodule at another commit, edited tracked files (submodules included), or
a binary not built from the pin. On this PC it currently reports PIN FAIL for
one reason: `modules/mavlink/pymavlink` is at ec06837a, not the commit the pin
records; the binary itself is the pinned firmware ("ArduCopter V4.5.7
(2a3dc4b7)"). Every ROS 2 run records the autopilot's own answer
(AUTOPILOT_VERSION) in `metrics.json` -> `hil_evidence.ardupilot_version`.

Optional: build the two ROS 2 packages (`safety_shield`, `mavlink_adapter`)
into a workspace outside the repository:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths "$REPO/sitl/ros2_ws/src" \
  --build-base ~/guardrail_ws/build --install-base ~/guardrail_ws/install
source ~/guardrail_ws/install/setup.bash
ros2 launch safety_shield guardrail_rail.launch.py shield_args:="--shield on"
```

`run_ros2_demo.sh` does not need the build: it starts the same launch file
from the source tree.

## Run the ROS 2 rail

```bash
cd "/mnt/d/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone"
bash sitl/run_ros2_demo.sh on                      # Shield on
bash sitl/run_ros2_demo.sh off                     # control arm: the ArduPilot GeoFence is the only backstop
bash sitl/run_ros2_demo.sh on --dynamic            # NFZ hot-applied at t = 8 s
bash sitl/run_ros2_demo.sh on --dynamic --dynamic-zone on-aircraft   # ... on top of the aircraft: starts unsafe
bash sitl/run_ros2_demo.sh on - --yaw-rate 0.2     # the VLA turns while it flies
bash sitl/run_ros2_demo.sh on - --bundle bundles/fase3-sim-demo-v0.1.1.tar.gz --tag my_run
```

What it starts, in order: SITL with the GeoFence backstop
(`start_sitl.sh`, `fence/guardrail_fence.parm`), the MAVLink router
(`start_router.sh`, `mavlink-router/main.conf`), MAVROS 2 on UDP 14555, a
rosbag2 recording, then the three Guardrail nodes through
`ros2_ws/src/safety_shield/launch/guardrail_rail.launch.py`. When the Shield
node ends the mission, the bag is closed and the replay bundle is re-packed
with it (`python -m guardrail.replay pack`).

Switches: `ROUTER=0` (MAVROS straight to SITL tcp 5760, as before
2026-10-06), `BAG=0`, `USE_LAUNCH=0`, `GUARDRAIL_FENCE=0` (stock autopilot),
`SITL_WIPE=0` (keep the EEPROM), `GCS_HOST=...`, `VLA=bridge` (AerialVLA in
the VLA slot instead of the stub, below), `GCS_BACKSTOP=1`, `STYLE=mission`
and `ADAPTER_ARGS=...` (below).

### The GCS-set GeoFence (`GCS_BACKSTOP=1`, WP3-10)

```bash
GCS_BACKSTOP=1 bash sitl/run_ros2_demo.sh off - --speed 3.5 --tag my_fence_off
```

SITL boots without `fence/guardrail_fence.parm`; the router's GCS endpoint
goes to `127.0.0.1:${GCS_UDP_PORT:-14560}`, where `gcs_backstop.py` - the
ground station's own process, before MAVROS and the Shield start - sets
FENCE_TYPE, FENCE_ACTION, FENCE_ALT_MAX, FENCE_MARGIN, AVOID_ENABLE and then
FENCE_ENABLE over that link (system 255, as Mission Planner) and uploads the
signed bundle's keep-out zones with the mission protocol, reading every
value and item back into `<out>/gcs_backstop.json`. The run is refused if it
fails. The adapter then uploads no zone (`--fences-by-gcs`), the Shield node
changes no fence parameter and records the GCS's read-back beside its own
(`hil_evidence.autopilot_fence.gcs`). Mission Planner is not on the GCS
endpoint for such a run.

### The simulator's stressors (`SITL_STRESS_PLAN`, X-14, 2026-11-07)

A stress-harness cell with wind or a GPS dropout (`experiments/
sweep_scenarios.py --backend sitl | hil`) gets a plan, and `sitl_stressors.py`,
a process of the simulator's side on SITL's SERIAL0 (tcp:5760, free with
`ROUTER=1`), sets SITL's own parameters: SIM_WIND_DIR (where the wind comes
FROM), SIM_WIND_T 1 and SIM_WIND_TURB 0 at boot, read back before MAVROS
starts (the run is refused otherwise), then SIM_WIND_SPD at the episode's
t = 0 and SIM_GPS_DISABLE 1 / 0 at the window's edges, on the clock the
Shield node announces in a STATUSTEXT (`GRDL T0 <nonce> t=<s>`). The Shield
node never sets a SIM_* parameter; it reads them back at take-off and records
every change the autopilot broadcasts (metrics.json `sim_params`), and hands
the Shield its GPS fix (`/mavros/global_position/raw/fix`, `fix_ok`).
`run_sitl_demo.py --sim-param` shares the parameter rule, the read-back test
and the record (`sitl_stressors.parse_sim_params`, `stressor_entry`), and
since 2026-11-07 a wind speed brings SIM_WIND_T 1 there too: SITL's default
square-law profile halved the 2 Nov runs' 8 m/s at their 15 m.

### The mission style (`STYLE=mission`, WP3-09)

```bash
STYLE=mission GCS_BACKSTOP=1 bash sitl/run_ros2_demo.sh on - \
  --waypoints "0,30;30,30" --speed 6 --hotapply-port 8071 --tag my_mission
```

`ros2_mission_node.py` takes the Shield's place (no VLA node): it checks the
plan leg by leg with the Shield's own rules, clamps the speed to the policy's
cap, and sends it to the adapter, which writes it to ArduPilot as
MISSION_ITEM_INT through MAVROS (`/mavros/mission/push`). In AUTO, at 10 Hz,
it re-checks the leg to the next waypoint under the rules in force (a REST
hot-apply may have changed them) and replaces that one item in flight - a
partial write, MISSION_WRITE_PARTIAL_LIST then MISSION_ITEM_INT - with a held
point short of the zone, writing the original back once the leg has been
legal for 2 s. The run folder gains `mission.json`; the bag gains
`/shield/mission`, `/shield/mission_update` and MAVROS's mission topics.
Since 2026-11-06 it also writes the grant's six-field `manifest.json` beside
`metrics.json`, from the same evidence the GUIDED Shield node gathers
(`mavros_evidence.py`): `vla_model_hash` is `none:hand-written-controller`
(no VLA flies this style), `policy_hash` the final generation's (the take-off
hash is in metrics), `random_seed` from `--seed` (default 0), `sim_speedup`
read from the autopilot, and `topology` from the evidence (also in metrics).
MAVROS 2.14's partial push ends in an ArduPilot upload timeout after the item
has landed (`mavlink_adapter_node.py` says why), so the adapter's verdict on
an update is its read-back of the item.

### AerialVLA in the VLA slot (`VLA=bridge`)

`sitl/ros2_vla_bridge_node.py` (to-do VLA-08) replaces the VLA stub on the
same topic: it reads the front and down cameras and the pose, asks the
AerialVLA server (protocol v2, `guardrail/vla_link.py`), flies each answer
through the step tracker and publishes `/vla/action_4d` at 10 Hz in the body
frame. The stub stays the default; without `VLA=bridge` nothing changes.

```bash
# on the GPU host first (Windows, vla-real env), the server with its whole configuration:
#   python demo/vla_server_v2.py serve --precision bf16 --attn sdpa --no-merge --missing-down refuse --host <an address WSL reaches> --port 8790
#   (the server warns when it listens off loopback; with WSL's mirrored networking 127.0.0.1 is shared)
VLA=bridge bash sitl/run_ros2_demo.sh on - --vla-server <windows-host-ip>:8790 --tag ros2_vla_01
VLA=bridge bash sitl/run_ros2_demo.sh on - --vla-server <host>:8790 --allow-stand-in   # a fake: server, recorded as a stub
```

| | |
|---|---|
| Message | `[vx_fwd, vy_right, vz_up, yaw_rate, decision_age_s, decision_seq, author, flags]`; the first four are the grant's, so a node reading `d[:4]` flies it unchanged. `decision_age_s` / `decision_seq` are -1 when no decision is behind the tick; author 0 none, 1 vla_step, 2 hold, 3 hover; flags 1 = the model answered LAND, 2 = the bridge's mission ended. The names are in `layout.dim[0].label`. |
| Mission | GOTO only: the hint is the compass phrase toward the Shield node's own mission target (`--command` / `--target`), `""` inside 3 m. No detector topic exists on this rail, so no APPROACH. |
| When it flies | Only once the Shield node flies the stream (armed, GUIDED, its first `/shield/setpoint`); out of GUIDED (the Shield's hold, an FSM fail-safe, a takeover) suspends the step tracker, GUIDED again resumes it and throws away every decision captured before. |
| Zero | A hover or no decision is all-zero; a tick that fails, or whose latest pose is older than `--pose-max-age-s` (1 s), publishes nothing, so the Shield's 0.5 s receipt watchdog holds the aircraft. The capture time is the camera's header stamp when it is on this host's clock, else the frame's arrival. |
| Shield side | A message that is not a whole valid tail is refused (not flown, not a receipt) and held: a NaN or infinity anywhere, an out-of-set field, another length, a wrong layout label (or none, once the bridge announced its layout), a decision seq going backwards, motion that no decision authored (only a vla_step moves horizontally or turns), and - once the bridge has shown itself - the grant's four values alone (a stub left running); more than one publisher on `/vla/action_4d` on 3 consecutive once-a-second checks is recorded with their names and makes the run not KPI-grade. A vla_step whose decision is older than `--decision-timeout-s` (10 s) plus a 1 s grace, or a hold older than twice that timeout plus the grace, is held like a quiet stream; the next valid message that is not itself stale ends the hold, and silence ages a decision only up to the 0.5 s receipt timeout (a sound bridge's stall is the receipt watchdog's: LOITER, then GUIDED when it is back). One hold of the Shield's that lasts `--decision-hold-max-s` (10 s), or 3 holds within 30 s, end the mission through the mission-end LAND, never over the FSM's own Loiter or fail-safe. LAND or the end flag ends the mission through the Shield node's own mission-end LAND, in place (never a mode request from the VLA; where, in which state, and any keep-out rule the position breaks are in metrics `vla_pilot.land_request`). A kind `vla` identity's `reported_digest` becomes metrics `pilot` (kind vla) and the manifest's `vla_model_hash`; a `fake:` id is a stub, and a bridge that never became ready is `<id>@unresolved`. `--vla-ready-timeout-s` (3300 s; the launch file adds it for `vla:=bridge`) arms only after the bridge's warm-up passed and only if its layout and decision timeout are the Shield's; an identity with no `ready` (a stub) does not pass, and arming gets its whole 180 s budget after the wait. The Shield's own 2 m reach rule still ends the mission first if the aircraft gets there before the model answers LAND. |
| Warm-up | Three inferences on live camera pairs before `/vla/identity` is published; the last is judged against `--warmup-max-latency-s` (5 s). Not ready: the identity says why, and the Shield does not arm. |
| Records | `vla_steps.jsonl`, `vla_frames/`, `vla_captures.jsonl`, `vla_bridge_ticks.jsonl`, `vla_bridge.json` beside the Shield's records (without `--out`, the Shield's own directory rule); an earlier run's copies go to `_previous/`. Written when the bridge stops: launch's SIGINT/SIGTERM, or the SIGTERM `run_ros2_demo.sh` sends and waits for on `USE_LAUNCH=0`. |
| SROS 2 | The bridge runs in the VLA slot's enclave, `/guardrail/vla_stub` (an enclave of its own would make every existing keystore refuse `SROS2=1`). |

**Cameras.** The bridge subscribes to `/camera/front/image_raw` and
`/camera/down/image_raw` (`--front-topic`, `--down-topic`; `sensor_msgs/Image`,
rgb8 or bgr8; AerialVLA's training cameras are 256x256, 90 deg, pitch 0 and
-90). Nothing on this SITL-only rail publishes them yet: ArduPilot SITL has no
camera, so until a camera publisher runs (Project AirSim's, or a driver on the
drone) the warm-up finds no camera pair, the bridge reports not ready and the
Shield node refuses to arm. The bridge has not been run under ROS; its tests
(`tests/test_ros2_vla_bridge.py`) drive it with a scripted server, a point
vehicle and a stand-in rclpy.

### Mission Planner

Mission Planner connects to ArduPilot, not to a simulator. Start it on Windows
and choose **UDP**, port **14550**: the router sends every MAVLink message the
autopilot emits to the Windows host on that port, beside MAVROS. Mission
Planner sets stream rates for the link it shares with MAVROS; the Shield node
re-asserts 10 Hz every 5 s and records the pose rate it actually got
(`metrics.json` -> `pose_hz`). `pose_hz` is a whole-flight average and can
hide a stall. Since 2026-10-19 each row carries `pose_age_ms` and
`pose_stale`, and `metrics.json` -> `pose_watchdog` counts the stale spells.
A pose older than `--pose-timeout-s` (default 1.0 s) holds the aircraft (or,
with the FSM already holding it, stops the FSM stepping) and re-requests the
streams at once. A run with such a spell is not KPI-grade.

To check that the Windows port carries a whole flight (not only a packet
count), run the stand-in listener on Windows during a flight, then check it
against the run's own mission window:

```bash
python sitl/gcs_listen.py listen --seconds 270 --out gcs.json      # Windows, before the run
python sitl/gcs_listen.py check gcs.json demo/out/<tag>/events.jsonl
```

`check` passes only if GLOBAL_POSITION_INT and ATTITUDE arrived throughout the
mission with no gap over 2 s. On 2026-10-07 it passed on two flights
(`ros2fix_off_fence`, `ros2fix_on_unsafe_start`): both at 10.0 Hz, largest gap
0.1 s. Run the listener with a Python interpreter that Windows Firewall
allows to receive inbound UDP. An interpreter with an inbound Block rule
receives nothing and the check fails; Mission Planner normally has an Allow
rule of its own.

`mavlink-routerd` is the grant's router and is not packaged for Ubuntu 24.04;
until it is built and installed, `start_router.sh` runs the same fan-out on
MAVProxy (`~/venv-ap`) and the run record says which one it was
(`hil_evidence.mavlink_router`).

## Run the pymavlink rail

Terminal 1 (WSL) - the autopilot:
```bash
bash start_sitl.sh                    # listens on tcp:127.0.0.1:5760
```

Terminal 2 (WSL) - the Guardrail mission:
```bash
cd "/mnt/d/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone"
~/venv-ap/bin/python sitl/run_sitl_demo.py --shield off     # A
~/venv-ap/bin/python sitl/run_sitl_demo.py --shield on      # B
~/venv-ap/bin/python sitl/run_sitl_demo.py --shield on --dynamic --seed 3
```

`~/venv-ap` has no jinja2 yet, so this rail cannot render `prompt.yaml` or
the CSP files; it flies anyway and records why in `events.jsonl`
(`~/venv-ap/bin/pip install jinja2==3.1.6` fixes it, no rebuild).

## What every run writes (`demo/out/<tag>/`)

`flight_log.jsonl` (one row per tick, incl. `unsafe`, `policy_hash`,
`generation`, `shield_ms`, the FSM state, the episode id, and `flown`: false
on a tick the autopilot held the aircraft and nothing was sent),
`policy_g<N>.json` and `csp_g<N>.json` for every policy generation,
`shield_audit.jsonl` (`audit.jsonl` before 2026-11-02; each record with the
episode id, its log row's tick number and the rail's own escalation-FSM
verdict: the rails build the Shield without an FSM of its own; record
version 4, the grant's audit fields with the grant's content), `events.jsonl`
(ending with `mission_end`),
`adapter_log.jsonl` (ROS rail), `metrics.json` (incl. the autopilot's fence
parameters as read back), `manifest.json`, `kpi.json`, `report.md`,
`trajectory.png`, `bag/` (ROS rail: actions, mode changes, MAVROS's sensor
topics and raw MAVLink, `/uas1/mavlink_source` and `/uas1/mavlink_sink`) and
`<tag>.replay.tar.gz`, which carries
all of it. Starting a new episode in a used tag moves EVERY earlier file to
`_previous/<stamp>/` first, and the bundle refuses records that carry another
episode's id or an episode that never ended.

The Shield looks 5 s ahead at 0.1 s steps (50 poses) on both rails, as the
grant's Safety Shield page sets it.

## Notes

- SITL boot -> EKF ready takes ~20-40 s; both rails retry arming until ACK.
- The EEPROM is wiped at each SITL start: fence polygons are stored there,
  and a zone left by an earlier hot-apply would otherwise be armed from boot.
- The GeoFence is a backstop: a zone the aircraft is inside or near is held
  back from the autopilot until the aircraft is clear (a zone hot-applied on
  top of the aircraft is the Shield's to resolve), and FENCE_ALT_MAX is raised
  at take-off when the policy allows more than 25 m.
- Keep every line of `fence/guardrail_fence.parm` under 98 characters:
  ArduPilot reads defaults files 98 characters at a time and parses the rest
  of a longer line as a new line.
- Coordinates: the policy is local metres, x North, y East, up positive.
  MAVROS topics are ENU; the adapter fills `PositionTarget` in ENU because
  MAVROS converts ENU -> NED itself (see `mavlink_adapter_node.py`).
