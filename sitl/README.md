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
`SITL_WIPE=0` (keep the EEPROM), `GCS_HOST=...`.

### Mission Planner

Mission Planner connects to ArduPilot, not to a simulator. Start it on Windows
and choose **UDP**, port **14550**: the router sends every MAVLink message the
autopilot emits to the Windows host on that port, beside MAVROS. Mission
Planner sets stream rates for the link it shares with MAVROS; the Shield node
re-asserts 10 Hz every 5 s and records the pose rate it actually got
(`metrics.json` -> `pose_hz`).

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
`audit.jsonl` (each record with the episode id, its log row's tick number and
the rail's own escalation-FSM verdict: the rails build the Shield without an
FSM of its own), `events.jsonl` (ending with `mission_end`),
`adapter_log.jsonl` (ROS rail), `metrics.json` (incl. the autopilot's fence
parameters as read back), `manifest.json`, `kpi.json`, `report.md`,
`trajectory.png`, `bag/` (ROS rail) and `<tag>.replay.tar.gz`, which carries
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
