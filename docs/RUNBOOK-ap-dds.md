# Runbook: the AP_DDS path on SITL (ARCH-08)

The grant makes MAVROS 2 the Year-1 bridge and AP_DDS "a Q3 optional migration,
not a blocker" (Architecture constraints p1), because "AP_DDS coverage is
improving but uneven across topics needed by the Shield". This runbook shows
the AP_DDS path on ArduCopter SITL without touching the rig:

| Piece | File | What it does |
|---|---|---|
| Build | `sitl/ap_dds/setup_ap_dds.sh` | A SEPARATE ArduPilot checkout (`~/ardupilot-dds`, default `Copter-4.5.7`, the rig's tag) built with `--enable-dds`; ArduPilot's Micro-XRCE-DDS-Gen; a colcon workspace (`~/ap_dds_ws`) with the micro-ROS agent and `ardupilot_msgs` from the same checkout. Refuses to build into `~/ardupilot`. Installs nothing system-wide. |
| Run | `sitl/ap_dds/start_ap_dds_sitl.sh` | micro-ROS agent on UDP 2019, then the DDS SITL (`-I0`, EEPROM wiped, run dir `~/ap-dds-run`) on its own ROS graph (domain 42, localhost discovery). `AP_DDS_MAVROS=1` adds a MAVROS endpoint (SERIAL2, udp 14550) for stage 1. Refuses to start while tcp:5760 is taken (the rig's SITL). |
| Check | `sitl/ap_dds/verify_ap_dds.py` | `/ap/time`, `/ap/pose/filtered`, `/ap/navsat`, `/ap/geopose/filtered` at a rate; GUIDED, arm, take off; a velocity through `/ap/cmd_vel` (or through the adapter, `--via-adapter`); flown velocity and track against the command; LAND. Writes `ap_dds_verify.json`, exit 0 only on PASS. |
| Adapter | `sitl/ap_dds_adapter_node.py` | The Shield's `/shield/setpoint` (body) to `/ap/cmd_vel` (TwistStamped, frame `map`, ENU) with the MAVLink adapter's own rotation; `--forward-modes` sends `/shield/mode_request` to `/ap/mode_switch`. Opt-in: nothing on the KPI rail launches it. |
| Test | `tests/test_ap_dds_adapter.py` | No ROS, no simulator. |

## Row status (ARCH-08)

**PARTIAL** (independent review, 26 Nov, 3 of 3). Steps 1-6 ran on
ArduCopter 4.5.7 DDS SITL on 26 Nov (desktop WSL, `dev` topology, not
KPI-grade; docs/data/ap_dds_20261126/): steps 4 and 5 PASS
(`ap_dds_verify_01`, `_02`: velocity_error 0.02 / 0.016 m/s, bearing error
0.1 deg), and one clean step-6 flight (`ap_dds_stage1_02`, 21493e6) in which
the Shield's setpoints reached the autopilot only through
`ap_dds_adapter_node.py` -> `/ap/cmd_vel`. That flight was a route with no
scenario: closest zone 9.90 m, no P0 engagement, so no safety intervention
has gone through the adapter on a clean revision (`ap_dds_stage1_01`, the
zone-engaging run, is diagnostic only: dirty tree, hold lost to BRAKE). State,
mode changes, arming, take-off, home, parameters and the fence stay on
MAVROS 2; stage 2 is not possible on 4.5.7 (no mode/armed/landed state, no
param service). The grant names no KPI for this optional row.

| After | Row text |
|---|---|
| 17 Nov (b316a05) | PARTIAL: AP_DDS output adapter, scripts and runbook exist; Shield on MAVROS |
| steps 4 and 5 PASS (26 Nov) | still PARTIAL: "AP_DDS output path shown on SITL; the Shield's state inputs still on MAVROS". Neither run has the Shield in the loop (step 5 publishes `/shield/setpoint` itself) |
| step 6 (stage 1) flown (26 Nov, `ap_dds_stage1_02`) | still PARTIAL: the output half of an optional migration. This runbook once called step 6 "the earliest DONE candidate"; that is the project's own bar, not the grant's wording |

To close the row: a recorded lab/PI decision that stage 1 meets this
optional item, or that the row closes at PARTIAL (not yet recorded); a
clean-revision scenario run with the P0 rule engaged through
`ap_dds_adapter_node.py` would strengthen the case. The evidence to cite is
the SITL output only: `ap_dds_verify.json` (`velocity.velocity_error`,
`velocity.bearing_error_deg`, `climb`, `ap_topics_seen`) and, for step 6,
the Shield's run directory and the flown trajectory. `ap_receives_ned` in
`ap_dds_adapter_log.jsonl` is the adapter's own model output, equal to `ned`
by construction: a consistency check, not evidence of what the autopilot
received.

## What the unit test checks (a model consistency check, not a measurement)

`python tests/test_ap_dds_adapter.py` runs a sweep over 360 headings x 108
body setpoints (38,880): the NED command our model of ArduPilot's ENU -> NED
(`ap_dds_receives`) takes from the adapter's `map`-frame TwistStamped, against
the NED command MAVROS sends on the MAVLink path
(`mavlink_adapter_node.mavros_sends`). The difference is 0 by construction:
both paths share one rotation (`setpoint_from_body`) and the ENU packing and
its inverse are hand-written in the same repo. So the sweep shows the adapter
and the MAVLink path agree with each other; it cannot catch a difference
between our model and ArduPilot's own map-frame handling. It is not a KPI
value and is not cited as evidence. Only the SITL steps below show the
autopilot's behaviour.

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
pin      : Copter-4.5.7 = 2a3dc4b7bf2507120f7378a7b2fde73185e0c325 (same commit as sitl/setup_sitl.sh)
...
checkout : /home/<user>/ardupilot-dds (Copter-4.5.7)
binary   : /home/<user>/ardupilot-dds/build/sitl/bin/arducopter
banner   : ArduCopter V4.5.7 (2a3dc4b7)
DDS      : compiled in (AP_DDS_Client symbols present)
gen      : /home/<user>/Micro-XRCE-DDS-Gen/scripts/microxrceddsgen
workspace: /home/<user>/ap_dds_ws/install (ardupilot_msgs micro_ros_agent micro_ros_msgs ...)
```

The `pin` line appears only on the build (not `--check`); if the tag moved,
the build stops with `PIN FAIL`. The DDS line looks for the `AP_DDS_Client`
symbols, else `AP_DDS_ENABLED 1` in `build/sitl/ap_config.h`: confirm it
reads "compiled in" on the first build, and note here if neither marker is
present on a binary that does serve `/ap/*` topics.

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

Expected: `micro-ROS agent pid ... on udp4 2019`, `ROS_DOMAIN_ID=42 ...`, then
SITL's boot lines. The agent log (`~/ap-dds-run/agent.log`) shows `session
established` once the autopilot's DDS client connects (a few seconds after
boot).

The DDS SITL runs on its own ROS graph: DDS domain 42 (`AP_DDS_DOMAIN_ID`,
ArduPilot parameter `DDS_DOMAIN_ID` written to `~/ap-dds-run/dds_domain.parm`)
with localhost-only discovery, so a Shield or adapter still up on the AGX
(10.42.0.3, domain 0) never sees this run's `/shield/setpoint` or
`/ap/cmd_vel`. **Every other terminal below starts with the same exports**
(`ENV` below). A mismatch shows up as missing `/ap/*` topics, not as a pass.

```bash
# ENV: paste at the top of terminals B, C, D
source /opt/ros/jazzy/setup.bash && source ~/ap_dds_ws/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

### 3. Topics (terminal B)

```bash
# ENV first
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
# ENV first
~/venv-ros/bin/python sitl/ap_dds/verify_ap_dds.py --out demo/out/ap_dds_verify_01
```

GUIDED (mode 4) and arming go over `/ap/mode_switch` and `/ap/arm_motors`,
both retried for up to `--arm-timeout` (90 s) because ArduCopter refuses
GUIDED until the EKF has a position. Once armed, the script always ends with a
zero setpoint and LAND, also after a failed takeoff or an error, and always
writes `ap_dds_verify.json`. The takeoff must reach 0.9 x `--alt` (`climb.ok`)
or the run fails before the velocity step. On
4.5.7 there is no DDS takeoff service and GUIDED velocity does not lift a
landed copter, so the script takes off with MAV_CMD_NAV_TAKEOFF over MAVLink
(tcp:5760: close MAVProxy / Mission Planner first, SITL's tcp serial takes one
client; a refused link is written to `takeoff.error`); on 4.6+ it uses
`/ap/experimental/takeoff`. It then sends 1.0 m/s
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
# ENV first
~/venv-ros/bin/python sitl/ap_dds_adapter_node.py --out demo/out/ap_dds_verify_02
```

Terminal B (restart terminal A first so the copter is on the ground again;
the GUIDED retry covers the EKF settling after the restart):

```bash
# ENV first
~/venv-ros/bin/python sitl/ap_dds/verify_ap_dds.py --via-adapter --out demo/out/ap_dds_verify_02
```

The script publishes body `[1.0, 0, 0, 0]` on `/shield/setpoint`; the adapter
rotates it with the heading from `/ap/pose/filtered` and sends `/ap/cmd_vel`.
Expected: PASS, the track along the vehicle's heading, and
`ap_dds_adapter_log.jsonl` with `ap_receives_ned` equal to `ned` on every line.

Total for steps 2-5 after the build: ~10 min. Steps 4 and 5 show ArduPilot's
DDS command interface and the adapter's rotation; the Shield is not in the
loop in either, so they leave the row PARTIAL (see "Row status").

### 6. Stage 1: the Shield flies through the AP_DDS adapter

The Shield node reads state and arms / takes off / changes mode through
MAVROS, as on the rig; its setpoints leave on `/shield/setpoint` and reach the
autopilot through `ap_dds_adapter_node.py` on `/ap/cmd_vel`.
`mavlink_adapter_node.py` is NOT started, so no setpoint goes over MAVROS.
Without a `mavlink_adapter` node on `/shield/mode_request` the Shield uses its
direct-MAVROS mode fallback (`adapter_listening` in `ros2_shield_node.py`), so
the AP_DDS adapter runs WITHOUT `--forward-modes` (one mode path, not two).

Terminal A (restart, with the MAVROS endpoint):

```bash
AP_DDS_MAVROS=1 bash sitl/ap_dds/start_ap_dds_sitl.sh
```

Terminal B, MAVROS on SITL's SERIAL2 (udp 14550):

```bash
# ENV first, plus the rig's ROS env that has mavros
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14550@
```

Terminal C, the AP_DDS adapter with the Shield's heading source (MAVROS pose,
so the adapter and the Shield rotate with one heading):

```bash
# ENV first
~/venv-ros/bin/python sitl/ap_dds_adapter_node.py \
  --pose-topic /mavros/local_position/pose --out demo/out/ap_dds_stage1_01
```

Terminal D, the stub pilot and the Shield (the KPI rail's pair, default
signed policy bundle):

```bash
# ENV first
~/venv-ros/bin/python sitl/ros2_vla_stub_node.py &
~/venv-ros/bin/python sitl/ros2_shield_node.py --shield on --tag ap_dds_stage1_01
kill %1
```

Expected: the Shield arms, takes off and flies; its run directory
(`demo/out/ap_dds_stage1_01`) has its usual events, and
`ap_dds_adapter_log.jsonl` there has one line per forwarded setpoint with
`ap_receives_ned` equal to `ned`.

The Shield's delivery watchdog accepts the AP_DDS adapter's
`/mavlink_adapter/sent` reports and beat (same topic, QoS depth and JSON as
the MAVLink adapter; `tests/test_ap_dds_adapter.py`
`test_the_shields_delivery_monitor_accepts_the_adapters_reports`). Flown on
`ap_dds_stage1_01` (26 Nov): metrics.json `setpoint_delivery` 244 published,
244 reports, 101 beats, no gap, no shortfall.

The default scenario above is the stub flying straight at `nfz-square`: the
Shield escalates on the first tick (X1 Brake, G3 Loiter held in GUIDED,
G5 RTL after 30 s), so only 2 ticks are "flown" (`flown` counts the pilot's
action or the FSM's stop, not the hold's own stream). The MAVLink rail does
the same with this scenario (hil_smoke_03, 2 of 303 ticks). To fly a route
through the AP_DDS adapter, give both nodes a target clear of the zone and
the bundle, as `sitl/run_ros2_demo.sh` does (`=`: argparse reads a bare
`-30,30` as an option):

```bash
# ENV first
B=bundles/fase3-sim-demo-v0.1.4.tar.gz
~/venv-ros/bin/python sitl/ros2_vla_stub_node.py --bundle $B --target=-30,30 &
~/venv-ros/bin/python sitl/ros2_shield_node.py --shield on --bundle $B \
  --target=-30,30 --tag ap_dds_stage1_02
kill %1
```

Run records on `/mnt/d` (an OneDrive folder through WSL's drvfs): a single
append there can stall for 300 ms (measured 26 Nov: p99 77 ms, max 324 ms,
against under 1 ms on WSL's own disk). The Shield writes its records on the
tick thread, so such a stall during a GUIDED hold exceeds
`HOLD_STREAM_GAP_S` (0.3 s) and hands the hold to BRAKE ("the stream that
flies the GUIDED hold faltered"), as on `ap_dds_stage1_01` at 24.6 s
(`post_publish` 296 ms on tick 244). It is not a delivery fault: the adapter
sent every setpoint, at most 0.115 s apart. The MAVLink rail on this
desktop shows the same (Gazebo sweep, 20 Nov: 0.41 s and 0.44 s gaps).

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
reading MAVROS for state (step 6: `AP_DDS_MAVROS=1` on the DDS SITL, the
adapter with `--pose-topic /mavros/local_position/pose` so both rotate with
one heading, and no `mavlink_adapter_node.py`); (2) the Shield's state inputs
moved once the firmware publishes mode, armed and landed state. Neither stage
changes the KPI rail, which stays on MAVROS 2 as the grant has it.
