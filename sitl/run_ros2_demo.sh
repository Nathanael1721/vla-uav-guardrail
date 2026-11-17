#!/usr/bin/env bash
# The ROS 2 rail, end to end, on one desktop (the grant's `dev` topology):
#
#   ArduPilot SITL --SERIAL2--> mavlink-router --+--> MAVROS 2 (udp 14555)
#        (GeoFence backstop)                    +--> Mission Planner (Windows, udp 14550)
#   vla_stub --/vla/action_4d--> safety_shield --/shield/setpoint--> mavlink_adapter
#            --/mavros/setpoint_raw/local--> MAVROS 2
#   ros2 bag record of the grant's topics into demo/out/<tag>/bag: actions,
#   mode changes, MAVROS's sensor topics and raw MAVLink (see the command)
#
# Usage (WSL):  bash run_ros2_demo.sh [on|off] [--dynamic | -] [extra args ...]
#               VLA=bridge bash run_ros2_demo.sh on - --vla-server HOST:PORT [...]
#
# THE VLA SLOT (2026-10-23). VLA=stub (the default) runs the VLA stub, as
# every run before. VLA=bridge runs sitl/ros2_vla_bridge_node.py in its place:
# AerialVLA through a protocol-v2 server that must already be running where
# the GPU is (demo/vla_server_v2.py serve ...; --vla-server HOST:PORT is
# required, e.g. the Windows host's address from WSL), reading the cameras on
# /camera/front/image_raw and /camera/down/image_raw - which nothing on this
# SITL-only rail publishes yet, so its warm-up then refuses and the Shield
# does not arm. With VLA=bridge the bridge's arguments are checked
# (--check-args) before SITL starts, it gets `--shield <on|off>` and
# `--out <the episode directory>` beside the extra args, and the Shield node
# gets --vla-ready-timeout-s ${VLA_READY_TIMEOUT_S:-3300} (through the launch
# file's vla_ready_timeout_s:=, which adds it for vla:=bridge on any path):
# it arms only once the bridge's warm-up has passed - more than the bridge's
# 2700 s hello timeout (a cold bf16 load took over 15 min) plus its warm-up.
# When the mission ends the bridge is stopped with SIGTERM and waited for
# (stop_vla) before the replay pack, so its records are complete.
#
# Extra args go to BOTH the Shield node and the VLA node; each takes its own
# and ignores the rest: --policy P | --bundle B, --subject X,Y, --target X,Y,
# --speed V, --yaw-rate R (VLA), --seed N, --tag T, --max-s S,
# --dynamic-zone route|on-aircraft (Shield, with --dynamic), and the Shield
# node's --dev-yaml, --clock-start ISO8601, --clock-tz TZ, --hotapply-port N,
# --action-timeout-s S; with VLA=bridge also the bridge's --vla-server,
# --cadence, --object, --front-topic / --down-topic, ... and
# --decision-timeout-s S, which both nodes read (the bridge's step tracker,
# the Shield's decision watchdog).
#
# The policy (2026-10-12): a YAML flies only with --dev-yaml (a development
# run, recorded so). With no --policy / --bundle the signed bundle of
# policies/sim_demo_policy.yaml is given to both nodes when bundles/ holds it
# (python tools/build_flight_bundles.py, on a machine with the lab key);
# without it the run is refused here, before SITL starts.
#
# After the launch the replay bundle is re-packed with the closed bag; pack
# REFUSES an episode that never ended (a Shield node that died) rather than
# bundling a previous flight's records, and says so.
#
# Environment switches (defaults first):
#   ROUTER=1           SITL -> router -> MAVROS over UDP; 0 = MAVROS on SITL
#                      tcp 5760 as every run before 2026-10-06
#   BAG=1              record the rosbag; 0 = no bag
#   BAG_MANIFEST=0     1 = the manifest fields known before the recorder
#                      starts (code revision, seed, the launch's policy hash;
#                      sitl/bag_custom_data.py) go into the bag's
#                      metadata.yaml as custom_data; 0 records as before
#   USE_LAUNCH=1       start the nodes with ros2 launch
#                      (sitl/ros2_ws/src/safety_shield/launch/guardrail_rail.launch.py);
#                      0 = plain background processes
#   GUARDRAIL_FENCE=1  load the ArduPilot GeoFence backstop (start_sitl.sh)
#   PHYSICS=sitl       gazebo = Gazebo Harmonic's physics (a headless gz sim
#                      server, iris_runway) behind SITL's JSON backend: the
#                      grant's functional Gazebo rail (ARCH-16) over MAVROS 2.
#                      The run writes physics.json beside its records.
#   SITL_SPEEDUP=1     only 1 is accepted, on every physics: the Shield node
#                      ticks, caps the mission, fires its events and clocks
#                      its rules on wall time, so a faster simulator would fly
#                      another scenario than the one recorded. Refused until
#                      the node runs on the autopilot's clock.
#   GCS_HOST           Mission Planner's address (default: the Windows host)
#   VLA=stub           bridge = AerialVLA in the VLA slot (THE VLA SLOT above);
#                      it runs in the slot's SROS 2 enclave, vla_stub
#   VLA_READY_TIMEOUT_S=3300  with VLA=bridge: how long the Shield node waits
#                      for the bridge's warm-up before it refuses to arm
#   GCS_BACKSTOP=0     1 = the ArduPilot GeoFence is GCS-set (WP3-10): SITL
#                      boots without sitl/fence/guardrail_fence.parm
#                      (GUARDRAIL_FENCE=0), the router's GCS endpoint goes to
#                      127.0.0.1:${GCS_UDP_PORT:-14560}, and there
#                      sitl/gcs_backstop.py - the ground station's step, a
#                      process of its own before MAVROS and the Shield start -
#                      sets FENCE_* and AVOID_ENABLE and uploads the signed
#                      bundle's keep-out zones, reading each back into
#                      <out>/gcs_backstop.json; the run is refused if it
#                      fails. The adapter then uploads no zone
#                      (--fences-by-gcs). Needs ROUTER=1 and a signed bundle
#                      (not --dev-yaml). Mission Planner is not on the GCS
#                      endpoint for such a run.
#   ADAPTER_ARGS=""    extra arguments for sitl/mavlink_adapter_node.py
#   STYLE=guided       mission = the grant's pre-planned waypoint mission
#                      (WP3-09): sitl/ros2_mission_node.py, the mission-style
#                      Shield, takes the Shield's place (the extra args are
#                      its own: --waypoints "x,y;x,y", --speed, --bundle,
#                      --hotapply-port, --max-s, --tag), no VLA node starts,
#                      the adapter writes the plan as MISSION_ITEM_INT and
#                      replaces the next waypoint in flight, and the bag adds
#                      /shield/mission, /shield/mission_update and MAVROS's
#                      mission topics. Shield `on` only.
#   SITL_STRESS_PLAN / SITL_STRESS_RECORD (2026-11-07; experiments/
#                      sweep_scenarios.py run_sitl sets both through WSLENV
#                      for a cell with wind or a GPS dropout): the
#                      simulator's stressors. Once SITL and the router are
#                      up, sitl/sitl_stressors.py connects to SITL's SERIAL0
#                      (tcp:5760, free with ROUTER=1; refused with ROUTER=0,
#                      where MAVROS has it), sets the plan's boot SIM_*
#                      parameters and reads them back - the run is refused
#                      before MAVROS starts unless it says "SITL_STRESS BOOT
#                      OK" - and applies the scheduled ones at the episode's
#                      t = 0 the Shield node announces, writing its record to
#                      SITL_STRESS_RECORD. Its Python: SITL_STRESS_PYTHON
#                      (default ~/venv-ap/bin/python, pymavlink).
#   SROS2=0            1 = SROS 2 enforced on every ROS 2 process of the rail
#                      (sitl/sros2_env.sh): the keystore SROS2_KEYSTORE (default
#                      ~/sros2_keystore, built by tools/sros2_setup.sh from the
#                      harness's lab CA) must hold every enclave of
#                      sitl/sros2_enclaves.txt or the run is refused before
#                      SITL starts; MAVROS, the bag recorder and each Guardrail
#                      node then run in their own enclave under
#                      ROS_SECURITY_STRATEGY=Enforce. 0 changes nothing.
set -e
SHIELD="${1:-on}"
DYN="${2:-}"
[ "$DYN" = "-" ] && DYN=""
EXTRA=("${@:3}")
DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/.." && pwd)"
VLA="${VLA:-stub}"
case "$VLA" in
  stub | bridge) ;;
  *) echo "[vla] REFUSED: VLA must be stub or bridge, not '$VLA'"; exit 2 ;;
esac
STYLE="${STYLE:-guided}"
BAG_EXTRA=()
case "$STYLE" in
  guided) ;;
  mission)
    if [ "$SHIELD" != "on" ] || [ -n "$DYN" ] || [ "$VLA" != "stub" ]; then
      echo "[style] REFUSED: STYLE=mission flies the mission-style Shield: shield on, no --dynamic, no VLA=bridge"
      exit 2
    fi
    BAG_EXTRA=(/shield/mission /shield/mission_update /mavros/mission/waypoints /mavros/mission/reached)
    ;;
  *) echo "[style] REFUSED: STYLE must be guided or mission, not '$STYLE'"; exit 2 ;;
esac

# The episode directory, by the Shield node's own rule (or --tag).
TAG="ros2_shield_${SHIELD}"
[ "$DYN" = "--dynamic" ] && TAG="${TAG}_dynamic"
for ((i = 0; i < ${#EXTRA[@]}; i++)); do
  [ "${EXTRA[$i]}" = "--tag" ] && TAG="${EXTRA[$((i + 1))]}"
done
OUT="$ROOT/demo/out/$TAG"
mkdir -p "$OUT" "$HOME/sitl-run"

BAG_PID=""
VLA_PID=""
VLA_READY_TIMEOUT_S="${VLA_READY_TIMEOUT_S:-3300}"
# Cleanup ALWAYS runs (success, failure, Ctrl+C) - leftover arducopter/mavros
# processes hold their ports and silently break every later run. The bag is
# stopped FIRST and with SIGTERM, so rosbag2 closes the file and writes
# metadata.yaml. Not SIGINT: bash starts background jobs of a non-interactive
# script with SIGINT ignored, and a recorder sent SIGINT kept recording into
# the next run's directory (seen 2026-10-06).
stop_bag() {
  if [ -n "$BAG_PID" ] && kill -0 "$BAG_PID" 2> /dev/null; then
    kill -TERM "$BAG_PID" 2> /dev/null || true
    for _ in $(seq 1 20); do kill -0 "$BAG_PID" 2> /dev/null || break; sleep 0.5; done
    kill -9 "$BAG_PID" 2> /dev/null || true
  fi
  BAG_PID=""
}
# The VLA bridge writes its records (vla_steps.jsonl, vla_bridge.json, ...)
# when it is stopped: SIGTERM - not SIGINT, which a background job here
# ignores (see above) - and a wait, BEFORE anything is SIGKILLed (until the
# review of 339f23a the trap's kill -9 was the only stop on USE_LAUNCH=0, and
# the bridge's records were lost). With the launch file, ros2 launch stops it.
stop_vla() {
  if [ -n "$VLA_PID" ] && kill -0 "$VLA_PID" 2> /dev/null; then
    kill -TERM "$VLA_PID" 2> /dev/null || true
    for _ in $(seq 1 40); do kill -0 "$VLA_PID" 2> /dev/null || break; sleep 0.5; done
  fi
  VLA_PID=""
}
cleanup() {
  stop_bag
  stop_vla
  # The stressor process with TERM, so its record says how far it got.
  pkill -TERM -f "[s]itl_stressors.py" 2> /dev/null || true
  pkill -TERM -f "[r]os2 bag record -s mcap" 2> /dev/null || true
  pkill -9 -f "[g]uardrail_rail.launch.py"     2> /dev/null || true
  pkill -9 -f "[r]os2_vla_stub_node"           2> /dev/null || true
  pkill -9 -f "[r]os2_vla_bridge_node"         2> /dev/null || true
  pkill -9 -f "[m]avlink_adapter_node"         2> /dev/null || true
  pkill -9 -f "[g]cs_backstop.py"              2> /dev/null || true
  pkill -9 -f "[r]os2_shield_node"             2> /dev/null || true
  pkill -9 -f "[r]os2_mission_node"            2> /dev/null || true
  pkill -9 -f "mavros_nod[e]"                  2> /dev/null || true
  pkill -9 -f "[m]avproxy.py --master=udpin"   2> /dev/null || true
  pkill -9 -f "[m]avlink-routerd"              2> /dev/null || true
  pkill -9 -f "[a]rducopter"                   2> /dev/null || true
  pkill -9 -f "gz si[m] -s -r"                 2> /dev/null || true
}
trap cleanup EXIT

source /opt/ros/jazzy/setup.bash
PY=~/venv-ros/bin/python

HAS_POLICY=0
HAS_DEV_YAML=0
for a in "${EXTRA[@]}"; do
  case "$a" in
    --policy | --policy=* | --bundle | --bundle=*) HAS_POLICY=1 ;;
    --dev-yaml) HAS_DEV_YAML=1 ;;
  esac
done
if [ "$HAS_POLICY" = "0" ] && [ "$HAS_DEV_YAML" = "0" ]; then
  DEFAULT_BUNDLE="$(cd "$ROOT" && "$PY" -c 'from guardrail.manifest import flight_bundle_path; print(flight_bundle_path("policies/sim_demo_policy.yaml"))')"
  if [ -f "$DEFAULT_BUNDLE" ]; then
    echo "[policy] signed bundle $DEFAULT_BUNDLE"
    EXTRA+=(--bundle "$DEFAULT_BUNDLE")
  else
    echo "[policy] REFUSED: no --policy / --bundle, and no signed bundle at"
    echo "         $DEFAULT_BUNDLE. Build it (python tools/build_flight_bundles.py),"
    echo "         give --bundle B, or pass --dev-yaml to fly the YAML unsigned."
    exit 2
  fi
fi

# WHERE THE FLIGHT IS (2026-11-04). A geographic bundle (WGS84 since IR schema
# 3.1.0) is flown at its frame origin: SITL boots there (start_sitl.sh,
# SITL_HOME), so MAVROS's local frame and the policy's metres share an origin
# and the adapter's GeoFence upload - refused beyond 5 m of the autopilot's
# home - lands on the zones. Until then SITL always booted at CMAC, and a
# bundle anchored elsewhere (corridor-survey 0.1.2: the Project AirSim
# scenes' home-geo-point) flew its geometry about CMAC with no GeoFence
# uploaded. A bundle whose origin is CMAC changes nothing, a flight without a
# bundle keeps CMAC, and SITL_HOME set by hand wins. Altitude 584 m is the
# SITL ground every rail assumes (guardrail/terrain.py, FlatGround).
if [ -z "${SITL_HOME:-}" ]; then
  FLIGHT_BUNDLE=""
  for ((i = 0; i < ${#EXTRA[@]}; i++)); do
    case "${EXTRA[$i]}" in
      --bundle) FLIGHT_BUNDLE="${EXTRA[$((i + 1))]}" ;;
      --bundle=*) FLIGHT_BUNDLE="${EXTRA[$i]#--bundle=}" ;;
    esac
  done
  if [ -n "$FLIGHT_BUNDLE" ]; then
    ORIGIN="$(cd "$ROOT" && "$PY" "$DIR/bundle_origin.py" "$FLIGHT_BUNDLE")" || {
      echo "[home] REFUSED: could not read the frame origin of $FLIGHT_BUNDLE"; exit 2; }
    if [ -n "$ORIGIN" ]; then
      export SITL_HOME="$ORIGIN,584,0"
      echo "[home] SITL boots at the bundle's frame origin: $SITL_HOME"
    fi
  fi
fi

# GCS_BACKSTOP=1: the ground station sets the GeoFence (see the header). Its
# bundle is the flight's signed bundle; a YAML flight has none to upload.
GCS_BACKSTOP="${GCS_BACKSTOP:-0}"
ADAPTER_EXTRA=()
read -r -a ADAPTER_EXTRA <<< "${ADAPTER_ARGS:-}"
case "$GCS_BACKSTOP" in
  0)
    # A GCS record left by an earlier flight under this tag would read as
    # this flight's (the Shield node records it when present).
    if [ -f "$OUT/gcs_backstop.json" ]; then
      mkdir -p "$OUT/_previous"
      mv "$OUT/gcs_backstop.json" "$OUT/_previous/gcs_backstop-$(date -u +%Y%m%dT%H%M%SZ).json"
    fi
    ;;
  1)
    GCS_BUNDLE=""
    for ((i = 0; i < ${#EXTRA[@]}; i++)); do
      case "${EXTRA[$i]}" in
        --bundle) GCS_BUNDLE="${EXTRA[$((i + 1))]}" ;;
        --bundle=*) GCS_BUNDLE="${EXTRA[$i]#--bundle=}" ;;
      esac
    done
    if [ -z "$GCS_BUNDLE" ] || [ "$HAS_DEV_YAML" = "1" ]; then
      echo "[gcs] REFUSED: GCS_BACKSTOP=1 uploads the signed bundle's zones; this run has no --bundle"
      exit 2
    fi
    if [ "${ROUTER:-1}" != "1" ]; then
      echo "[gcs] REFUSED: GCS_BACKSTOP=1 needs ROUTER=1 (the GCS speaks on the router's GCS endpoint)"
      exit 2
    fi
    if [ -n "${GCS_HOST:-}" ] && [ "$GCS_HOST" != "127.0.0.1" ]; then
      echo "[gcs] GCS_HOST=$GCS_HOST ignored: with GCS_BACKSTOP=1 the GCS endpoint is the local GCS step"
    fi
    export GCS_HOST=127.0.0.1 GCS_UDP_PORT="${GCS_UDP_PORT:-14560}"
    export GUARDRAIL_FENCE=0
    ADAPTER_EXTRA+=(--fences-by-gcs)
    ;;
  *) echo "[gcs] REFUSED: GCS_BACKSTOP must be 0 or 1, not '$GCS_BACKSTOP'"; exit 2 ;;
esac

# VLA=bridge: the bridge's own refusals (no --vla-server, bad object words, a
# policy it cannot load) come here, before SITL starts, not after take-off.
if [ "$VLA" = "bridge" ]; then
  if ! "$PY" "$DIR/ros2_vla_bridge_node.py" --check-args "${EXTRA[@]}" \
      --shield "$SHIELD" --out "$OUT" > ~/sitl-run/vla_check.json; then
    echo "[vla] REFUSED: the VLA bridge's arguments do not resolve (see above)"
    exit 2
  fi
  echo "[vla] AerialVLA bridge: $(tr -d '\n' < ~/sitl-run/vla_check.json | head -c 300)"
fi

# SROS 2 (SROS2=1): refused here, before anything starts, when the keystore
# lacks an enclave; the arrays stay empty with SROS2=0, so every command
# below is then exactly what it was.
source "$DIR/sros2_env.sh"
sros2_prepare || exit 2
sros2_prefix ENC_MAVROS mavros
sros2_prefix ENC_BAG bag_recorder
sros2_prefix ENC_VLA vla_stub
sros2_prefix ENC_ADAPTER mavlink_adapter
sros2_prefix ENC_SHIELD safety_shield
sros2_launch_args LAUNCH_SROS2

echo "=== clean slate ==="
cleanup
sleep 2

# The firmware about to fly, as the setup script reads it. The Shield node
# records what the AUTOPILOT reports (AUTOPILOT_VERSION) and uses this banner
# only if the autopilot does not answer, saying so.
VERIFY="$(bash "$DIR/setup_sitl.sh" --verify 2>&1 || true)"
printf '%s\n' "$VERIFY" | sed 's/^/[pin] /'
ARDUPILOT_VERSION="$(printf '%s\n' "$VERIFY" | sed -n 's/^binary   : //p')"
ARDUPILOT_PIN_OK=0
printf '%s\n' "$VERIFY" | grep -qx 'PIN OK' && ARDUPILOT_PIN_OK=1
export ARDUPILOT_VERSION ARDUPILOT_PIN_OK

PHYSICS="${PHYSICS:-sitl}"
# SITL_SPEEDUP (2026-11-17, WP4-04): unset or 1, every run as before, and
# nothing else. The grant lets the broad sweep run faster than real time on
# the functional rail, but SITL's --speedup (and gz sim's rate) would speed up
# the simulator only: ros2_shield_node.py's tick (create_timer), its mission
# cap, its event schedule and its rule clock run on time.monotonic, so at 2x
# the Shield would decide at half the sim rate with half its lookahead margin
# and switch events at twice their declared sim time. Refused here, before
# anything starts, until the node runs on the autopilot's clock.
# speedup-check begin
SPEEDUP="${SITL_SPEEDUP:-1}"
if ! [[ "$SPEEDUP" =~ ^1(\.0+)?$ ]]; then
  echo "[speedup] REFUSED: SITL_SPEEDUP='$SPEEDUP': only 1 is flown - the Shield node's tick, mission cap, event schedule and rule clock run on wall time"
  exit 2
fi
# speedup-check end
case "$PHYSICS" in
  sitl) ;;
  gazebo)
    GZ_WORLD="$HOME/ardupilot_gazebo/worlds/iris_runway.sdf"
    GZ_IRIS_PARM="$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm"
    [ -f "$GZ_WORLD" ] || { echo "PHYSICS=gazebo: $GZ_WORLD missing (sitl/setup_gazebo.sh)"; exit 1; }
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
    export GZ_SIM_RESOURCE_PATH="$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
    echo "=== gz sim (headless server, $GZ_WORLD) ==="
    nohup gz sim -s -r "$GZ_WORLD" > ~/sitl-run/gz.log 2>&1 &
    sleep 6
    export SITL_MODEL=JSON SITL_EXTRA_DEFAULTS="$GZ_IRIS_PARM"
    ;;
  *) echo "PHYSICS must be sitl or gazebo, got '$PHYSICS'"; exit 1 ;;
esac

echo "=== fresh SITL ==="
echo "[physics] $PHYSICS"
nohup bash "$DIR/start_sitl.sh" > ~/sitl-run/sitl.log 2>&1 &
sleep 8

if [ "${ROUTER:-1}" = "1" ]; then
  echo "=== MAVLink router ==="
  rm -f ~/sitl-run/router.kind
  nohup bash "$DIR/start_router.sh" > ~/sitl-run/router.log 2>&1 &
  sleep 3
  MAVLINK_ROUTER="$(cat ~/sitl-run/router.kind 2> /dev/null || echo unknown)"
  FCU_URL="udp://:${MAVROS_UDP_PORT:-14555}@"
  head -3 ~/sitl-run/router.log | sed 's/^/[router] /'
else
  MAVLINK_ROUTER="none (MAVROS on SITL tcp 5760)"
  FCU_URL="tcp://127.0.0.1:5760"
fi
export MAVLINK_ROUTER
export MAVROS_FCU_URL="$FCU_URL"

# THE SIMULATOR'S STRESSORS (SITL_STRESS_PLAN, see the header): before MAVROS
# and the Shield exist, the boot parameters set and read back, or no flight.
if [ -n "${SITL_STRESS_PLAN:-}" ]; then
  if [ "${ROUTER:-1}" != "1" ]; then
    echo "[stress] REFUSED: SITL_STRESS_PLAN needs ROUTER=1 (with ROUTER=0 MAVROS holds SITL's tcp:5760)"
    exit 2
  fi
  if [ -z "${SITL_STRESS_RECORD:-}" ] || [ ! -f "$SITL_STRESS_PLAN" ]; then
    echo "[stress] REFUSED: SITL_STRESS_RECORD unset or no plan at $SITL_STRESS_PLAN"
    exit 2
  fi
  STRESS_PY="${SITL_STRESS_PYTHON:-$HOME/venv-ap/bin/python}"
  echo "=== simulator stressors ($SITL_STRESS_PLAN) ==="
  "$STRESS_PY" "$DIR/sitl_stressors.py" --plan "$SITL_STRESS_PLAN" \
    --record "$SITL_STRESS_RECORD" > ~/sitl-run/stress.log 2>&1 &
  STRESS_PID=$!
  STRESS_OK=0
  for _ in $(seq 1 120); do
    if grep -q "^SITL_STRESS BOOT OK" ~/sitl-run/stress.log 2> /dev/null; then
      STRESS_OK=1
      break
    fi
    grep -q "^SITL_STRESS BOOT FAILED" ~/sitl-run/stress.log 2> /dev/null && break
    kill -0 "$STRESS_PID" 2> /dev/null || break
    sleep 0.5
  done
  # Once more: it may have said so just before the last check.
  grep -q "^SITL_STRESS BOOT OK" ~/sitl-run/stress.log 2> /dev/null && STRESS_OK=1
  sed 's/^/[stress] /' ~/sitl-run/stress.log 2> /dev/null || true
  if [ "$STRESS_OK" != "1" ]; then
    echo "[stress] REFUSED: the boot parameters were not set and read back ($SITL_STRESS_RECORD)"
    exit 2
  fi
fi

if [ "$GCS_BACKSTOP" = "1" ]; then
  # Before MAVROS and the Shield exist: the GCS's own process, on the
  # router's GCS endpoint, sets the backstop and reads it back.
  echo "=== GCS: GeoFence set over the GCS link (udpin:127.0.0.1:$GCS_UDP_PORT) ==="
  GCS_RC=0
  "$PY" "$DIR/gcs_backstop.py" --bundle "$GCS_BUNDLE" \
    --link "udpin:127.0.0.1:$GCS_UDP_PORT" \
    --router-endpoint "$MAVLINK_ROUTER, GCS endpoint (mission_planner) -> 127.0.0.1:$GCS_UDP_PORT" \
    --out "$OUT" > ~/sitl-run/gcs.log 2>&1 || GCS_RC=$?
  sed 's/^/[gcs] /' ~/sitl-run/gcs.log
  if [ "$GCS_RC" -ne 0 ]; then
    echo "[gcs] REFUSED (exit $GCS_RC): the GCS could not set and read back the GeoFence ($OUT/gcs_backstop.json)"
    exit 2
  fi
fi

echo "=== mavros ($FCU_URL) ==="
nohup "${ENC_MAVROS[@]}" ros2 run mavros mavros_node --ros-args -p fcu_url:="$FCU_URL" \
  > ~/sitl-run/mavros.log 2>&1 &
sleep 10

if [ "${BAG:-1}" = "1" ]; then
  echo "=== rosbag2 -> $OUT/bag ==="
  if [ -e "$OUT/bag" ]; then
    mkdir -p "$OUT/_previous"
    mv "$OUT/bag" "$OUT/_previous/bag-$(date -u +%Y%m%dT%H%M%SZ)"
  fi
  # The grant's episode bag (Stress Testing, "Episode bundle": "rosbag2
  # capture: sensors, actions, MAVLink, mode changes"; ARCH-22). Actions: the
  # VLA's, the Shield's and what MAVROS was sent (/mavros/setpoint_raw/local,
  # which tools/export_episodes.py reads as each step's `executed`). Mode
  # changes: /mavros/state, /mavros/extended_state, the Shield's mode
  # requests and the autopilot's status text. Sensors (since 2026-11-02): the
  # pose, velocity, IMU, GPS fix and relative altitude, altitude, battery and
  # home that MAVROS reports, and the two camera topics the VLA bridge reads
  # (nothing publishes them on the SITL-only rail; recorded when present).
  # Raw MAVLink (since 2026-11-02): MAVROS 2's own link to its router, both
  # directions - /uas1/mavlink_source (from the autopilot) and
  # /uas1/mavlink_sink (to it, SET_POSITION_TARGET_LOCAL_NED among them), as
  # mavros_msgs/Mavlink. Keep deploy/docker/entrypoint.sh's BAG_TOPICS equal
  # (tests/test_deploy_configs.py).
  # BAG_MANIFEST=1 (off by default; ARCH-22, 2026-11-16): what the launch
  # knows of the manifest, as KEY=VALUE pairs after --custom-data, which
  # rosbag2 writes into metadata.yaml when it closes the bag. The option takes
  # every argument after it, so it stays LAST. A field that cannot be
  # computed is left out (bag_custom_data.log); none at all records as before.
  BAG_CUSTOM=()
  if [ "${BAG_MANIFEST:-0}" = "1" ]; then
    BAG_DYN=()
    [ -n "$DYN" ] && BAG_DYN=("$DYN")
    mapfile -t BAG_CUSTOM < <("$PY" "$DIR/bag_custom_data.py" -- --shield "$SHIELD" \
      "${BAG_DYN[@]}" "${EXTRA[@]}" 2> ~/sitl-run/bag_custom_data.log || true)
    if [ "${#BAG_CUSTOM[@]}" -gt 0 ]; then
      echo "[bag] custom_data: ${BAG_CUSTOM[*]}"
      BAG_CUSTOM=(--custom-data "${BAG_CUSTOM[@]}")
    else
      echo "[bag] BAG_MANIFEST=1 but no field could be computed (~/sitl-run/bag_custom_data.log): recording without custom_data"
    fi
  fi
  nohup "${ENC_BAG[@]}" ros2 bag record -s mcap -o "$OUT/bag" \
    /vla/action_4d /vla/identity /shield/setpoint /shield/intercept \
    /shield/mode_request /shield/fences /mavlink_adapter/status \
    /mavros/setpoint_raw/local /mavros/setpoint_raw/target_local \
    /mavros/state /mavros/local_position/pose \
    /mavros/extended_state /mavros/statustext/recv \
    /mavros/local_position/velocity_local /mavros/imu/data \
    /mavros/global_position/global /mavros/global_position/rel_alt \
    /mavros/altitude /mavros/battery /mavros/home_position/home \
    /camera/front/image_raw /camera/down/image_raw \
    /uas1/mavlink_source /uas1/mavlink_sink "${BAG_EXTRA[@]}" "${BAG_CUSTOM[@]}" \
    > ~/sitl-run/bag.log 2>&1 &
  BAG_PID=$!
  sleep 2
fi

SHIELD_ARGS=(--shield "$SHIELD")
[ -n "$DYN" ] && SHIELD_ARGS+=("$DYN")
SHIELD_ARGS+=("${EXTRA[@]}")
VLA_NODE_ARGS=("${EXTRA[@]}")
if [ "$VLA" = "bridge" ]; then
  # The bridge clamps its altitude targets by the arm it flies behind. (The
  # Shield's ready gate: vla_ready_timeout_s:= below, or the fallback path.)
  VLA_NODE_ARGS+=(--shield "$SHIELD")
fi
VLA_ARGS=""
[ "${#VLA_NODE_ARGS[@]}" -gt 0 ] && VLA_ARGS="$(printf '%q ' "${VLA_NODE_ARGS[@]}")"

LAUNCH_ADAPTER=()
[ "${#ADAPTER_EXTRA[@]}" -gt 0 ] && LAUNCH_ADAPTER=(adapter_args:="${ADAPTER_EXTRA[*]}")
echo "=== Guardrail nodes (shield $SHIELD, VLA $VLA, style $STYLE) ==="
if [ "${USE_LAUNCH:-1}" = "1" ]; then
  ros2 launch "$DIR/ros2_ws/src/safety_shield/launch/guardrail_rail.launch.py" \
    python:="$PY" repo_root:="$ROOT" out:="$OUT" vla:="$VLA" style:="$STYLE" \
    vla_ready_timeout_s:="$VLA_READY_TIMEOUT_S" \
    shield_args:="$(printf '%q ' "${SHIELD_ARGS[@]}")" \
    vla_args:="$VLA_ARGS" "${LAUNCH_ADAPTER[@]}" "${LAUNCH_SROS2[@]}"
elif [ "$STYLE" = "mission" ]; then
  nohup "${ENC_ADAPTER[@]}" "$PY" "$DIR/mavlink_adapter_node.py" --out "$OUT" "${ADAPTER_EXTRA[@]}" \
    > ~/sitl-run/adapter.log 2>&1 &
  sleep 2
  "${ENC_SHIELD[@]}" "$PY" "$DIR/ros2_mission_node.py" "${SHIELD_ARGS[@]}"
else
  if [ "$VLA" = "bridge" ]; then
    nohup "${ENC_VLA[@]}" "$PY" "$DIR/ros2_vla_bridge_node.py" "${VLA_NODE_ARGS[@]}" --out "$OUT" > ~/sitl-run/vla.log 2>&1 &
    VLA_PID=$!
    SHIELD_ARGS+=(--vla-ready-timeout-s "$VLA_READY_TIMEOUT_S")
  else
    nohup "${ENC_VLA[@]}" "$PY" "$DIR/ros2_vla_stub_node.py" "${EXTRA[@]}" > ~/sitl-run/vla.log 2>&1 &
  fi
  nohup "${ENC_ADAPTER[@]}" "$PY" "$DIR/mavlink_adapter_node.py" --out "$OUT" "${ADAPTER_EXTRA[@]}" \
    > ~/sitl-run/adapter.log 2>&1 &
  sleep 2
  "${ENC_SHIELD[@]}" "$PY" "$DIR/ros2_shield_node.py" "${SHIELD_ARGS[@]}"
  stop_vla
fi

echo "=== close the bag, re-pack the replay bundle with it ==="
stop_bag
# physics-record begin
if [ "$PHYSICS" = "gazebo" ]; then
  # sim_speedup_requested (2026-11-17): what the run was asked to fly at (1,
  # speedup-check); the manifest's sim_speedup is what the autopilot read.
  printf '{"physics": "gazebo-harmonic", "gz_sim": "%s", "world": "%s", "sitl_model": "JSON", "extra_defaults": "%s", "sim_speedup_requested": %s}\n' \
    "$(gz sim --version 2> /dev/null | head -1 | sed 's/.*version //')" "$GZ_WORLD" "$GZ_IRIS_PARM" \
    "$SPEEDUP" > "$OUT/physics.json"
else
  printf '{"physics": "ardupilot-sitl-builtin", "sitl_model": "quad"}\n' > "$OUT/physics.json"
fi
# physics-record end
cd "$ROOT"
"$PY" -m guardrail.replay pack "$OUT" || echo "[replay] re-pack FAILED (see above)"

echo "=== done (cleanup runs via trap) ==="
