# Runbook: the AP_DDS path on SITL (ARCH-08)

The grant makes MAVROS 2 the Year-1 bridge and AP_DDS "a Q3 optional migration,
not a blocker" (Architecture constraints p1), because "AP_DDS coverage is
improving but uneven across topics needed by the Shield". This runbook shows
the AP_DDS path on ArduCopter SITL without touching the rig:

| Piece | File | What it does |
|---|---|---|
| Build | `sitl/ap_dds/setup_ap_dds.sh` | A SEPARATE ArduPilot checkout (`~/ardupilot-dds`, default `Copter-4.5.7`, the rig's tag) built with `--enable-DDS`; ArduPilot's Micro-XRCE-DDS-Gen; a colcon workspace (`~/ap_dds_ws`) with the micro-ROS agent and `ardupilot_msgs` from the same checkout. Refuses to build into `~/ardupilot`. Installs nothing system-wide. |
| Run | `sitl/ap_dds/start_ap_dds_sitl.sh` | micro-ROS agent on UDP 2019, then the DDS SITL (`-I0`, EEPROM wiped, run dir `~/ap-dds-run`). Refuses to start while tcp:5760 is taken (the rig's SITL). |
| Check | `sitl/ap_dds/verify_ap_dds.py` | `/ap/time`, `/ap/pose/filtered`, `/ap/navsat`, `/ap/geopose/filtered` at a rate; GUIDED, arm, take off; a velocity through `/ap/cmd_vel` (or through the adapter, `--via-adapter`); flown velocity and track against the command; LAND. Writes `ap_dds_verify.json`, exit 0 only on PASS. |
| Adapter | `sitl/ap_dds_adapter_node.py` | The Shield's `/shield/setpoint` (body) to `/ap/cmd_vel` (TwistStamped, frame `map`, ENU) with the MAVLink adapter's own rotation; `--forward-modes` sends `/shield/mode_request` to `/ap/mode_switch`. Opt-in: nothing on the KPI rail launches it. |
| Test | `tests/test_ap_dds_adapter.py` | No ROS, no simulator. |

## What is measured without the rig

`python tests/test_ap_dds_adapter.py` prints the equivalence sweep: for 360
headings x 108 body setpoints (38,880), the NED command ArduPilot takes from the
adapter's `map`-frame TwistStamped (AP_DDS_External_Control's ENU -> NED,
modelled by `ap_dds_receives`) against the NED command MAVROS sends on the
MAVLink path (`mavlink_adapter_node.mavros_sends`). Measured 2026-11-17 on
this branch: **max difference 0** (the two paths use one rotation,
`setpoint_from_body`, and one ENU packing). The node itself runs on a stand-in
ROS stack in the same test. That is a model of ArduPilot's conversion, not a
flight: the SITL steps below are what show it on the autopilot.

## Steps (WSL Ubuntu 24.04, ROS 2 Jazzy; the operator runs these)

Stop the rig's SITL / MAVROS first (`pkill -f arducopter`; the DDS SITL uses
the same tcp:5760). Do not run this beside a KPI campaign.

### 0. Prerequisites (once, operator, sudo)

```bash
sudo apt-get update
sudo apt-get install -y default-jre python3-colcon-common-extensions python3-rosdep
```

`setup_ap_dds.sh` checks for these and prints this line if one is missing.

### 1. Build (once; ~45-75 min on the desktop, mostly downloads and compiles)

```bash
cd /mnt/d/<repo>            # the lab checkout, as for sitl/setup_sitl.sh
bash sitl/ap_dds/setup_ap_dds.sh
```

Expected tail (`--check` prints the same, read-only):

```
checkout : /home/<user>/ardupilot-dds (Copter-4.5.7)
binary   : /home/<user>/ardupilot-dds/build/sitl/bin/arducopter
banner   : ArduCopter V4.5.7 (2a3dc4b7)
DDS      : compiled in (DDS_ENABLE parameter present)
gen      : /home/<user>/Micro-XRCE-DDS-Gen/scripts/microxrceddsgen
workspace: /home/<user>/ap_dds_ws/install (ardupilot_msgs micro_ros_agent micro_ros_msgs ...)
```

If rosdep reports missing dependencies the script stops and prints
`rosdep install --from-paths ~/ap_dds_ws/src --ignore-src -y` for the
operator. If `waf configure` rejects the generator version, check out the
generator tag named in `~/ardupilot-dds/libraries/AP_DDS/README.md` for that
ref and re-run. `AP_DDS_REF=Copter-4.6 bash sitl/ap_dds/setup_ap_dds.sh` builds
4.6 instead (into the same `~/ardupilot-dds`), which adds the DDS takeoff
service; the default stays 4.5.7 so the AP_DDS path is the autopilot the MAVROS
path flies.

### 2. Run (terminal A, leave running)

```bash
bash sitl/ap_dds/start_ap_dds_sitl.sh
```

Expected: `micro-ROS agent pid ... on udp4 2019`, then SITL's boot lines. The
agent log (`~/ap-dds-run/agent.log`) shows `session established` once the
autopilot's DDS client connects (a few seconds after boot).

### 3. Topics (terminal B)

```bash
source /opt/ros/jazzy/setup.bash && source ~/ap_dds_ws/install/setup.bash
ros2 topic list | grep '^/ap/'
ros2 topic hz /ap/pose/filtered        # Ctrl-C after a few lines
ros2 topic echo --once /ap/geopose/filtered
```

Expected: `/ap/time`, `/ap/pose/filtered`, `/ap/twist/filtered`,
`/ap/geopose/filtered`, the NavSatFix topic (`/ap/navsat/navsat0` on 4.5,
`/ap/navsat` on later firmware), `/ap/cmd_vel` and others; the exact set
depends on the ref, and `verify_ap_dds.py` writes the list it saw
(`ap_topics_seen`).

### 4. Velocity straight through /ap/cmd_vel

```bash
~/venv-ros/bin/python sitl/ap_dds/verify_ap_dds.py --out demo/out/ap_dds_verify_01
```

GUIDED (mode 4) and arming go over `/ap/mode_switch` and `/ap/arm_motors`. On
4.5.7 there is no DDS takeoff service and GUIDED velocity does not lift a
landed copter, so the script takes off with MAV_CMD_NAV_TAKEOFF over MAVLink
(tcp:5760); on 4.6+ it uses `/ap/experimental/takeoff`. It then sends 1.0 m/s
east for 6 s at 10 Hz (frame `map`) and compares. Expected last line:
`PASS  -> demo/out/ap_dds_verify_01/ap_dds_verify.json`, with
`velocity.velocity_error` <= 0.3 m/s and `velocity.bearing_error_deg` <= 15.
The same by hand:

```bash
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}"
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
# take off (4.5.7: MAVProxy or Mission Planner on tcp:5760, `takeoff 10`)
ros2 topic pub -r 10 /ap/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: map}, twist: {linear: {x: 1.0}}}"
```

### 5. Through the adapter (the Shield's output path on AP_DDS)

Terminal C:

```bash
~/venv-ros/bin/python sitl/ap_dds_adapter_node.py --out demo/out/ap_dds_verify_02
```

Terminal B (restart terminal A first so the copter is on the ground again):

```bash
~/venv-ros/bin/python sitl/ap_dds/verify_ap_dds.py --via-adapter --out demo/out/ap_dds_verify_02
```

The script publishes body `[1.0, 0, 0, 0]` on `/shield/setpoint`; the adapter
rotates it with the heading from `/ap/pose/filtered` and sends `/ap/cmd_vel`.
Expected: PASS, the track along the vehicle's heading, and
`ap_dds_adapter_log.jsonl` with `ap_receives_ned` equal to `ned` on every line.

Total for steps 2-5 after the build: ~10 min.

## What the migration does not cover yet (the grant's "uneven coverage")

The Shield node (`sitl/ros2_shield_node.py`) still reads MAVROS. Its inputs and
their AP_DDS counterparts on 4.5.7, to be confirmed against `ap_topics_seen`
on the built firmware:

| Shield input / output (MAVROS) | AP_DDS |
|---|---|
| `/mavros/local_position/pose` | `/ap/pose/filtered` |
| `/mavros/local_position/velocity_local` | `/ap/twist/filtered` |
| `/mavros/global_position/raw/fix` | `/ap/navsat/navsat0` |
| `/mavros/setpoint_raw/local` | `/ap/cmd_vel` (this adapter) |
| `/mavros/set_mode`, `/mavros/cmd/arming` | `/ap/mode_switch`, `/ap/arm_motors` |
| `/mavros/cmd/takeoff` | none on 4.5.7 (`/ap/experimental/takeoff` later) |
| `/mavros/state`, `/mavros/extended_state` (mode, armed, landed) | none on 4.5.7 |
| `/mavros/home_position/home` | `/ap/gps_global_origin/filtered` is the EKF origin, not home |
| `/mavros/param/*` (fence, RTL, battery parameters the Shield reads) | none on 4.5.7 |
| `/mavros/geofence/push`, `/mavros/mission/push` | none: the GCS-set fence (`sitl/gcs_backstop.py`) stays |
| EKF and battery status over MAVLink | battery topic only |

So the staged path is: (1) the output path on AP_DDS with the Shield still
reading MAVROS for state (run the adapter with
`--pose-topic /mavros/local_position/pose` so both rotate with one heading,
and do not start `mavlink_adapter_node.py`); (2) the Shield's state inputs
moved once the firmware publishes mode, armed and landed state. Neither stage
changes the KPI rail, which stays on MAVROS 2 as the grant has it.
