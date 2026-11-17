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
set -eo pipefail
AP_DDS_DIR="${AP_DDS_DIR:-$HOME/ardupilot-dds}"
WS="${AP_DDS_WS:-$HOME/ap_dds_ws}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DDS_PORT="${DDS_UDP_PORT:-2019}"
RUN_DIR="${AP_DDS_RUN_DIR:-$HOME/ap-dds-run}"
HOME_LOC="${SITL_HOME:--35.363261,149.165230,584,0}"
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
mkdir -p "$RUN_DIR"
cd "$RUN_DIR"

nohup ros2 run micro_ros_agent micro_ros_agent udp4 -p "$DDS_PORT" \
  > "$RUN_DIR/agent.log" 2>&1 &
AGENT=$!
trap 'kill $AGENT 2>/dev/null || true' EXIT INT TERM
echo "micro-ROS agent pid $AGENT on udp4 $DDS_PORT (log $RUN_DIR/agent.log)"
echo "SITL: $BIN  home $HOME_LOC  (EEPROM wiped, run dir $RUN_DIR)"
# -w wipes the EEPROM: each start boots the defaults, DDS_ENABLE=1 among them.
# tail -f /dev/null keeps stdin open, as sitl/start_sitl.sh does: SITL's
# console exits on stdin EOF.
tail -f /dev/null | "$BIN" --model quad --speedup 1 -I0 -w --defaults "$PARM"   --home "$HOME_LOC" --serial0 tcp:0
