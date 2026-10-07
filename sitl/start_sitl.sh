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

DEFAULTS="$PARM"
if [ "${GUARDRAIL_FENCE:-1}" = "1" ]; then
  [ -f "$FENCE_PARM" ] || { echo "fence parm missing: $FENCE_PARM"; exit 1; }
  DEFAULTS="$PARM,$FENCE_PARM"
  echo "GeoFence backstop: $FENCE_PARM"
else
  echo "GeoFence backstop OFF (GUARDRAIL_FENCE=0)"
fi
WIPE=()
[ "${SITL_WIPE:-1}" = "1" ] && WIPE=(-w)

mkdir -p "$HOME/sitl-run"
cd "$HOME/sitl-run"          # eeprom.bin / logs live here, not in the repo
# tail -f /dev/null keeps stdin open forever: SITL's console exits on stdin
# EOF, which kills headless/background runs otherwise (learned the hard way).
tail -f /dev/null | "$BIN" --model quad --speedup 1 --defaults "$DEFAULTS" \
     --home -35.363261,149.165230,584,0 -I0 "${WIPE[@]}" \
     --serial0 tcp:0 --serial2 "udpclient:127.0.0.1:${SITL_ROUTER_PORT:-14550}"
