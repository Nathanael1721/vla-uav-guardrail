#!/usr/bin/env bash
# Launch ArduPilot SITL (copter), headless.
# Run in WSL:  bash start_sitl.sh
#
#   SERIAL0  tcp:5760   for the pymavlink rail (sitl/run_sitl_demo.py) and as
#                       a fallback MAVROS link (ROUTER=0). No longer ":wait":
#                       SITL starts without a client on it.
#   SERIAL2  udpclient  -> 127.0.0.1:14550, where mavlink-router (or its
#                       MAVProxy fallback, sitl/start_router.sh) fans MAVLink
#                       out to MAVROS 2 and Mission Planner.
#
# The ArduPilot GeoFence backstop (sitl/fence/guardrail_fence.parm) is loaded
# as a second defaults file; GUARDRAIL_FENCE=0 starts the stock autopilot, as
# every run before 2026-10-06 did. The EEPROM is wiped at each start
# (SITL_WIPE=0 keeps it): fence polygons are stored there, and a zone left by
# the previous flight's hot-apply would otherwise be armed from boot.
set -e
BIN="$HOME/ardupilot/build/sitl/bin/arducopter"
PARM="$HOME/ardupilot/Tools/autotest/default_params/copter.parm"
DIR="$(cd "$(dirname "$0")" && pwd)"
FENCE_PARM="$DIR/fence/guardrail_fence.parm"
[ -x "$BIN" ] || { echo "SITL binary missing - run setup_sitl.sh first"; exit 1; }

# SITL_MODEL=JSON with SITL_EXTRA_DEFAULTS=<gazebo-iris.parm> flies Gazebo's
# physics instead of SITL's built-in quad (run_ros2_demo.sh PHYSICS=gazebo).
MODEL="${SITL_MODEL:-quad}"
DEFAULTS="$PARM${SITL_EXTRA_DEFAULTS:+,$SITL_EXTRA_DEFAULTS}"
if [ "${GUARDRAIL_FENCE:-1}" = "1" ]; then
  [ -f "$FENCE_PARM" ] || { echo "fence parm missing: $FENCE_PARM"; exit 1; }
  # Appended, not assigned (2026-11-04): assigning "$PARM,$FENCE_PARM" here
  # dropped SITL_EXTRA_DEFAULTS, so PHYSICS=gazebo with the fence on (the
  # default) booted without gazebo-iris.parm.
  DEFAULTS="$DEFAULTS,$FENCE_PARM"
  echo "GeoFence backstop: $FENCE_PARM"
else
  echo "GeoFence backstop OFF (GUARDRAIL_FENCE=0)"
fi
echo "defaults: $DEFAULTS"

# SITL_HOME (lat,lon,alt_m,heading_deg): where the vehicle boots. CMAC by
# default, as every run before 2026-11-04. run_ros2_demo.sh sets it to a
# geographic bundle's frame origin, so the policy is flown where it is
# (WGS84 since IR schema 3.1.0) and the autopilot's GeoFence lands on its
# zones; the container's entrypoint takes the same variable.
HOME_LOC="${SITL_HOME:--35.363261,149.165230,584,0}"
NUM='-?[0-9]+(\.[0-9]+)?'
[[ "$HOME_LOC" =~ ^$NUM,$NUM,$NUM,$NUM$ ]] || {
  echo "SITL_HOME must be lat,lon,alt,heading - got '$HOME_LOC'"; exit 64; }
echo "home: $HOME_LOC"
WIPE=()
[ "${SITL_WIPE:-1}" = "1" ] && WIPE=(-w)

mkdir -p "$HOME/sitl-run"
cd "$HOME/sitl-run"          # eeprom.bin / logs live here, not in the repo
# tail -f /dev/null keeps stdin open forever: SITL's console exits on stdin
# EOF, which kills headless/background runs otherwise (learned the hard way).
tail -f /dev/null | "$BIN" --model "$MODEL" --speedup 1 --defaults "$DEFAULTS" \
     --home "$HOME_LOC" -I0 "${WIPE[@]}" \
     --serial0 tcp:0 --serial2 "udpclient:127.0.0.1:${SITL_ROUTER_PORT:-14550}"
