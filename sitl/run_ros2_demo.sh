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
#
# Extra args go to BOTH the Shield node and the VLA node; each takes its own
# and ignores the rest: --policy P | --bundle B, --subject X,Y, --target X,Y,
# --speed V, --yaw-rate R (VLA), --seed N, --tag T, --max-s S,
# --dynamic-zone route|on-aircraft (Shield, with --dynamic).
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
set -e
SHIELD="${1:-on}"
DYN="${2:-}"
[ "$DYN" = "-" ] && DYN=""
EXTRA=("${@:3}")
DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/.." && pwd)"

# The episode directory, by the Shield node's own rule (or --tag).
TAG="ros2_shield_${SHIELD}"
[ "$DYN" = "--dynamic" ] && TAG="${TAG}_dynamic"
for ((i = 0; i < ${#EXTRA[@]}; i++)); do
  [ "${EXTRA[$i]}" = "--tag" ] && TAG="${EXTRA[$((i + 1))]}"
done
OUT="$ROOT/demo/out/$TAG"
mkdir -p "$OUT" "$HOME/sitl-run"

BAG_PID=""
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
cleanup() {
  stop_bag
  pkill -TERM -f "[r]os2 bag record -s mcap" 2> /dev/null || true
  pkill -9 -f "[g]uardrail_rail.launch.py"     2> /dev/null || true
  pkill -9 -f "[r]os2_vla_stub_node"           2> /dev/null || true
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
nohup ros2 run mavros mavros_node --ros-args -p fcu_url:="$FCU_URL" \
  > ~/sitl-run/mavros.log 2>&1 &
sleep 10

if [ "${BAG:-1}" = "1" ]; then
  echo "=== rosbag2 -> $OUT/bag ==="
  if [ -e "$OUT/bag" ]; then
    mkdir -p "$OUT/_previous"
    mv "$OUT/bag" "$OUT/_previous/bag-$(date -u +%Y%m%dT%H%M%SZ)"
  fi
  nohup ros2 bag record -s mcap -o "$OUT/bag" \
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
VLA_ARGS=""
[ "${#EXTRA[@]}" -gt 0 ] && VLA_ARGS="$(printf '%q ' "${EXTRA[@]}")"

echo "=== Guardrail nodes (shield $SHIELD) ==="
if [ "${USE_LAUNCH:-1}" = "1" ]; then
  ros2 launch "$DIR/ros2_ws/src/safety_shield/launch/guardrail_rail.launch.py" \
    python:="$PY" repo_root:="$ROOT" out:="$OUT" \
    shield_args:="$(printf '%q ' "${SHIELD_ARGS[@]}")" \
    vla_args:="$VLA_ARGS"
else
  nohup "$PY" "$DIR/ros2_vla_stub_node.py" "${EXTRA[@]}" > ~/sitl-run/vla.log 2>&1 &
  nohup "$PY" "$DIR/mavlink_adapter_node.py" --out "$OUT" > ~/sitl-run/adapter.log 2>&1 &
  sleep 2
  "$PY" "$DIR/ros2_shield_node.py" "${SHIELD_ARGS[@]}"
fi

echo "=== close the bag, re-pack the replay bundle with it ==="
stop_bag
cd "$ROOT"
"$PY" -m guardrail.replay pack "$OUT" || echo "[replay] re-pack FAILED (see above)"

echo "=== done (cleanup runs via trap) ==="
