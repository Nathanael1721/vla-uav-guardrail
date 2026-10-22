#!/usr/bin/env bash
# The ROS 2 rail, end to end, on one desktop (the grant's `dev` topology):
#
#   ArduPilot SITL --SERIAL2--> mavlink-router --+--> MAVROS 2 (udp 14555)
#        (GeoFence backstop)                    +--> Mission Planner (Windows, udp 14550)
#   vla_stub --/vla/action_4d--> safety_shield --/shield/setpoint--> mavlink_adapter
#            --/mavros/setpoint_raw/local--> MAVROS 2
#   ros2 bag record of the grant's topics into demo/out/<tag>/bag
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
#   USE_LAUNCH=1       start the nodes with ros2 launch
#                      (sitl/ros2_ws/src/safety_shield/launch/guardrail_rail.launch.py);
#                      0 = plain background processes
#   GUARDRAIL_FENCE=1  load the ArduPilot GeoFence backstop (start_sitl.sh)
#   GCS_HOST           Mission Planner's address (default: the Windows host)
#   VLA=stub           bridge = AerialVLA in the VLA slot (THE VLA SLOT above);
#                      it runs in the slot's SROS 2 enclave, vla_stub
#   VLA_READY_TIMEOUT_S=3300  with VLA=bridge: how long the Shield node waits
#                      for the bridge's warm-up before it refuses to arm
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
  pkill -TERM -f "[r]os2 bag record -s mcap" 2> /dev/null || true
  pkill -9 -f "[g]uardrail_rail.launch.py"     2> /dev/null || true
  pkill -9 -f "[r]os2_vla_stub_node"           2> /dev/null || true
  pkill -9 -f "[r]os2_vla_bridge_node"         2> /dev/null || true
  pkill -9 -f "[m]avlink_adapter_node"         2> /dev/null || true
  pkill -9 -f "[r]os2_shield_node"             2> /dev/null || true
  pkill -9 -f "mavros_nod[e]"                  2> /dev/null || true
  pkill -9 -f "[m]avproxy.py --master=udpin"   2> /dev/null || true
  pkill -9 -f "[m]avlink-routerd"              2> /dev/null || true
  pkill -9 -f "[a]rducopter"                   2> /dev/null || true
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

echo "=== fresh SITL ==="
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
  nohup "${ENC_BAG[@]}" ros2 bag record -s mcap -o "$OUT/bag" \
    /vla/action_4d /vla/identity /shield/setpoint /shield/intercept \
    /shield/mode_request /shield/fences /mavlink_adapter/status \
    /mavros/setpoint_raw/local /mavros/setpoint_raw/target_local \
    /mavros/state /mavros/local_position/pose \
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

echo "=== Guardrail nodes (shield $SHIELD, VLA $VLA) ==="
if [ "${USE_LAUNCH:-1}" = "1" ]; then
  ros2 launch "$DIR/ros2_ws/src/safety_shield/launch/guardrail_rail.launch.py" \
    python:="$PY" repo_root:="$ROOT" out:="$OUT" vla:="$VLA"     vla_ready_timeout_s:="$VLA_READY_TIMEOUT_S" \
    shield_args:="$(printf '%q ' "${SHIELD_ARGS[@]}")" \
    vla_args:="$VLA_ARGS" "${LAUNCH_SROS2[@]}"
else
  if [ "$VLA" = "bridge" ]; then
    nohup "${ENC_VLA[@]}" "$PY" "$DIR/ros2_vla_bridge_node.py" "${VLA_NODE_ARGS[@]}" --out "$OUT" > ~/sitl-run/vla.log 2>&1 &
    VLA_PID=$!
    SHIELD_ARGS+=(--vla-ready-timeout-s "$VLA_READY_TIMEOUT_S")
  else
    nohup "${ENC_VLA[@]}" "$PY" "$DIR/ros2_vla_stub_node.py" "${EXTRA[@]}" > ~/sitl-run/vla.log 2>&1 &
  fi
  nohup "${ENC_ADAPTER[@]}" "$PY" "$DIR/mavlink_adapter_node.py" --out "$OUT" > ~/sitl-run/adapter.log 2>&1 &
  sleep 2
  "${ENC_SHIELD[@]}" "$PY" "$DIR/ros2_shield_node.py" "${SHIELD_ARGS[@]}"
  stop_vla
fi

echo "=== close the bag, re-pack the replay bundle with it ==="
stop_bag
cd "$ROOT"
"$PY" -m guardrail.replay pack "$OUT" || echo "[replay] re-pack FAILED (see above)"

echo "=== done (cleanup runs via trap) ==="
