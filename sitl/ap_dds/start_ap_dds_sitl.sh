#!/usr/bin/env bash
# Start the AP_DDS SITL (ARCH-08): the micro-ROS agent on UDP 2019 in the
# background, then ArduCopter from the SEPARATE DDS checkout in the foreground.
# Ctrl-C stops both. Build first with sitl/ap_dds/setup_ap_dds.sh.
#
#   bash sitl/ap_dds/start_ap_dds_sitl.sh
#
# MAVLink stays on tcp:5760 (SERIAL0), which verify_ap_dds.py uses for the
# takeoff on 4.5.7. The rig's SITL (sitl/start_sitl.sh) uses the same port:
# stop it first; this script refuses to start beside it.
#
# Stage 1 of the migration (docs/RUNBOOK-ap-dds.md: the Shield reads state from
# MAVROS, its output goes out on AP_DDS) needs MAVROS beside the DDS client.
# SITL's tcp serial takes one client, so that is opt-in on a second serial, as
# sitl/start_sitl.sh does it:
#
#   AP_DDS_MAVROS=1 bash sitl/ap_dds/start_ap_dds_sitl.sh
#     -> --serial2 udpclient:127.0.0.1:${SITL_ROUTER_PORT:-14550}
#        (MAVROS: fcu_url:=udp://:14550@)
#
# ROS graph isolation: the AP_DDS client joins DDS domain AP_DDS_DOMAIN_ID
# (default 42, ArduPilot parameter DDS_DOMAIN_ID, written to a defaults file in
# the run dir) and the agent runs with ROS_DOMAIN_ID and a localhost discovery
# range, so a Shield or adapter still up on the AGX (10.42.0.3, domain 0)
# never sees /shield/setpoint or /ap/cmd_vel from this run. Every terminal
# that talks to this SITL exports the same values:
#   export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# A domain mismatch shows up as missing /ap/* topics (verify_ap_dds.py FAIL),
# never as a silent pass.
set -eo pipefail
AP_DDS_DIR="${AP_DDS_DIR:-$HOME/ardupilot-dds}"
WS="${AP_DDS_WS:-$HOME/ap_dds_ws}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DDS_PORT="${DDS_UDP_PORT:-2019}"
RUN_DIR="${AP_DDS_RUN_DIR:-$HOME/ap-dds-run}"
HOME_LOC="${SITL_HOME:--35.363261,149.165230,584,0}"
DOMAIN="${AP_DDS_DOMAIN_ID:-42}"
MAVROS_SERIAL=()
if [ "${AP_DDS_MAVROS:-0}" = "1" ]; then
  MAVROS_SERIAL=(--serial2 "udpclient:127.0.0.1:${SITL_ROUTER_PORT:-14550}")
fi
BIN="$AP_DDS_DIR/build/sitl/bin/arducopter"
PARM="$AP_DDS_DIR/Tools/autotest/default_params/copter.parm"

[ -x "$BIN" ] || { echo "no DDS SITL binary at $BIN - run setup_ap_dds.sh"; exit 1; }
[ -f "$WS/install/setup.bash" ] || { echo "no workspace at $WS - run setup_ap_dds.sh"; exit 1; }
if ss -ltn 2>/dev/null | grep -q ':5760 '; then
  echo "tcp:5760 is in use (the rig's SITL?). Stop it first."
  exit 1
fi

# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
mkdir -p "$RUN_DIR"
cd "$RUN_DIR"
printf 'DDS_DOMAIN_ID %s\n' "$DOMAIN" > "$RUN_DIR/dds_domain.parm"

nohup ros2 run micro_ros_agent micro_ros_agent udp4 -p "$DDS_PORT" \
  > "$RUN_DIR/agent.log" 2>&1 &
AGENT=$!
trap 'kill $AGENT 2>/dev/null || true' EXIT INT TERM
echo "micro-ROS agent pid $AGENT on udp4 $DDS_PORT (log $RUN_DIR/agent.log)"
echo "SITL: $BIN  home $HOME_LOC  (EEPROM wiped, run dir $RUN_DIR)"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID (DDS_DOMAIN_ID $DOMAIN), discovery LOCALHOST: export both in every terminal"
if [ "${#MAVROS_SERIAL[@]}" -gt 0 ]; then
  echo "MAVROS endpoint: ${MAVROS_SERIAL[*]}  (fcu_url:=udp://:${SITL_ROUTER_PORT:-14550}@)"
fi
# -w wipes the EEPROM: each start boots the defaults, DDS_ENABLE=1 among them.
# tail -f /dev/null keeps stdin open, as sitl/start_sitl.sh does: SITL's
# console exits on stdin EOF.
tail -f /dev/null | "$BIN" --model quad --speedup 1 -I0 -w \
  --defaults "$PARM,$RUN_DIR/dds_domain.parm" \
  --home "$HOME_LOC" --serial0 tcp:0 "${MAVROS_SERIAL[@]}"
