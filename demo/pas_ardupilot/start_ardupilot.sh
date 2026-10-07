#!/usr/bin/env bash
# ArduPilot SITL as the flight controller of a Project AirSim vehicle, plus the
# MAVLink fan-out to Mission Planner, the perception node and MAVROS 2.
# Runs in WSL. Every address and argument comes from rail.py (tested offline);
# this script only starts processes and says what it started.
#
#   bash start_ardupilot.sh --sim-address <win ip> [--gcs-address <ip>]
#        [--node-address <ip>] [--orin <ip>] [--mavros] [--router auto|mavlink-router|mavproxy|none]
#   bash start_ardupilot.sh --stop
#
# SITL_WIPE=0 keeps SITL's stored parameters (eeprom.bin); the default wipes
# them at every start (-w), so the param file - not a value edited in Mission
# Planner in an earlier session - is what flies, as sitl/start_sitl.sh does.
#
# ORDER MATTERS: load the Project AirSim scene FIRST, then run this. Reloading a
# scene resets the simulator clock, and ArduPilot's AirSim backend adds the
# (negative) timestamp step to its own clock. perception_node.py and
# g1_check.py --start-sitl keep that order themselves.
#
# Expected output once the scene is loaded (ArduPilot's own lines):
#   Starting SITL Airsim type 0
#   Bind SITL sensor input at 127.0.0.1:9003      (it binds 0.0.0.0; the text says 127.0.0.1)
#   AirSim control interface set to <win ip>:9002
#   ... then NO repeating "No sensor message received in last 1s" lines:
#   those mean the simulator's frames are not arriving (see the design doc).
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
RUN_DIR="$HOME/sitl-run/pas"
AP_DIR="$HOME/ardupilot"
VENV_AP="$HOME/venv-ap"

stop_all() {
  pkill -9 -f "[a]rducopter --model airsim" 2>/dev/null || true
  pkill -9 -f "[m]avlink-routerd -c $RUN_DIR" 2>/dev/null || true
  pkill -9 -f "[m]avproxy.py --master=tcp:127.0.0.1:5760" 2>/dev/null || true
  pkill -9 -f "[m]avros_node.*14555" 2>/dev/null || true
}

SIM_ADDRESS=""; GCS_ADDRESS=""; NODE_ADDRESS=""; ORIN=""; MAVROS=0; ROUTER="auto"
while [ $# -gt 0 ]; do
  case "$1" in
    --sim-address)  SIM_ADDRESS="$2"; shift 2 ;;
    --gcs-address)  GCS_ADDRESS="$2"; shift 2 ;;
    --node-address) NODE_ADDRESS="$2"; shift 2 ;;
    --orin)         ORIN="$2"; shift 2 ;;
    --mavros)       MAVROS=1; shift ;;
    --router)       ROUTER="$2"; shift 2 ;;
    --stop)         stop_all; echo "stopped"; exit 0 ;;
    *) echo "unknown argument $1"; exit 2 ;;
  esac
done

# Under WSL2 NAT the Windows host is WSL's default gateway; under mirrored
# networking it is 127.0.0.1. Only guessed when the caller did not say.
if [ -z "$SIM_ADDRESS" ]; then
  if command -v wslinfo >/dev/null && [ "$(wslinfo --networking-mode 2>/dev/null)" = "mirrored" ]; then
    SIM_ADDRESS="127.0.0.1"
  else
    SIM_ADDRESS="$(ip route show default | awk '{print $3; exit}')"
  fi
fi
GCS_ADDRESS="${GCS_ADDRESS:-$SIM_ADDRESS}"
NODE_ADDRESS="${NODE_ADDRESS:-$SIM_ADDRESS}"

BIN="$AP_DIR/build/sitl/bin/arducopter"
[ -x "$BIN" ] || { echo "SITL binary missing: run sitl/setup_sitl.sh first"; exit 1; }
mkdir -p "$RUN_DIR"
stop_all
trap stop_all EXIT

# The pinned firmware, read-only (sitl/setup_sitl.sh --verify). Recorded, not
# enforced. The verdict comes from the exit status: on a failure the LAST line
# printed is the binary banner, which reads like success (it did on
# 2026-10-06: a drifted pymavlink submodule under a V4.5.7 banner).
if PIN_OUT="$(bash "$REPO/sitl/setup_sitl.sh" --verify 2>&1)"; then PIN="PIN OK"; else PIN="PIN FAIL"; fi
BANNER="$(printf '%s\n' "$PIN_OUT" | grep -m1 '^binary' || true)"
PIN="$PIN | ${BANNER:-no binary banner}"
echo "pin      : $PIN"
if [ "${PIN#PIN FAIL}" != "$PIN" ]; then
  printf '%s\n' "$PIN_OUT" | grep 'PIN FAIL' | sed 's/^/           /' || true
fi

PARAM="$HERE/citylife-quad.param"
WIPE_ARGS=(); [ "${SITL_WIPE:-1}" = "1" ] || WIPE_ARGS=(--no-wipe)
mapfile -t ARGS < <(python3 "$HERE/rail.py" sitl-args --sim-address "$SIM_ADDRESS" \
    --scene "$HERE/scene_citylife_ardupilot.jsonc" --param "$PARAM" \
    --binary "$BIN" --param-dir "$AP_DIR/Tools/autotest/default_params" "${WIPE_ARGS[@]}")
# A failure inside <( ) does not trip set -e; an empty array would run nothing
# and the next lines would report a router for an autopilot that never started.
[ "${#ARGS[@]}" -gt 0 ] || { echo "rail.py sitl-args produced nothing"; exit 1; }
echo "sitl     : ${ARGS[*]}"
cd "$RUN_DIR"
# tail -f /dev/null keeps stdin open: SITL's console exits on stdin EOF
# (sitl/start_sitl.sh learned this the hard way).
tail -f /dev/null | "${ARGS[@]}" > "$RUN_DIR/sitl.log" 2>&1 &
SITL_PID=$!
sleep 3

# The fan-out. mavlink-router is what the grant names; MAVProxy works too and
# is NOT it, so a fallback says so on screen and in endpoints.json.
# Mission Planner's TCP door differs: mavlink-router serves every TCP client
# on 5790, while a MAVProxy tcpin output serves one, so under the fallback the
# node keeps 5790 and Mission Planner gets 5791 (rail.GCS_TCP_FALLBACK_PORT).
USED="none"
GCS_TCP=5790
ORIN_ARGS=(); [ -n "$ORIN" ] && ORIN_ARGS=(--orin "$ORIN")
if [ "$ROUTER" = "auto" ] || [ "$ROUTER" = "mavlink-router" ]; then
  if command -v mavlink-routerd >/dev/null; then
    python3 "$HERE/rail.py" router-conf --gcs "$GCS_ADDRESS" --node "$NODE_ADDRESS" \
        "${ORIN_ARGS[@]}" > "$RUN_DIR/main.conf"
    mavlink-routerd -c "$RUN_DIR/main.conf" > "$RUN_DIR/router.log" 2>&1 &
    USED="mavlink-router"
  elif [ "$ROUTER" = "mavlink-router" ]; then
    echo "mavlink-routerd is not installed (see docs/DESIGN-projectairsim-ardupilot.md)"; exit 1
  fi
fi
if [ "$USED" = "none" ] && { [ "$ROUTER" = "auto" ] || [ "$ROUTER" = "mavproxy" ]; }; then
  mapfile -t MP < <(python3 "$HERE/rail.py" mavproxy-args --gcs "$GCS_ADDRESS" \
      --node "$NODE_ADDRESS" "${ORIN_ARGS[@]}")
  MP[0]="$VENV_AP/bin/mavproxy.py"
  echo "router   : *** mavlink-routerd not installed - MAVProxy fan-out instead."
  echo "           It carries the same traffic; it is NOT the grant's mavlink-router."
  "${MP[@]}" > "$RUN_DIR/router.log" 2>&1 &
  USED="mavproxy (fallback, not the grant's router)"
  GCS_TCP=5791
fi
echo "router   : $USED"
echo "           Mission Planner: UDP port 14550 (sent to $GCS_ADDRESS), or TCP 127.0.0.1:$GCS_TCP"

if [ "$MAVROS" = "1" ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  nohup ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14555@ \
      > "$RUN_DIR/mavros.log" 2>&1 &
  echo "mavros   : fcu_url udp://:14555@  (log $RUN_DIR/mavros.log)"
fi

python3 - "$RUN_DIR/endpoints.json" <<EOF
import json, sys
json.dump({"sim_address": "$SIM_ADDRESS", "gcs_address": "$GCS_ADDRESS",
           "node_address": "$NODE_ADDRESS", "orin": "$ORIN" or None,
           "router": "$USED", "gcs_tcp_port": $GCS_TCP,
           "mavros": bool($MAVROS), "pin": """$PIN""",
           "sitl_args": """${ARGS[*]}"""}, open(sys.argv[1], "w"), indent=2)
EOF
echo "running  : SITL pid $SITL_PID; logs in $RUN_DIR (Ctrl+C stops everything)"
wait "$SITL_PID"
