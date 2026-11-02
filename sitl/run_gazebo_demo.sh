#!/usr/bin/env bash
# Gazebo Harmonic rail: gz physics world + ArduPilot SITL (JSON backend) +
# Guardrail demo over pymavlink.
# Usage (WSL):  bash run_gazebo_demo.sh [on|off] [--dynamic]
#
# pipefail (2026-10-19): the mission is piped through grep, and without it the
# script's exit status was grep's - 0 for a mission that refused its policy,
# crashed or timed out in bring-up - as setup_sitl.sh and start_router.sh
# already guard against. (A KPI FAIL still exits 0: run_sitl_demo.py returns
# 0 whatever the verdict.)
set -eo pipefail
SHIELD="${1:-on}"
DYN="${2:-}"
DIR="$(cd "$(dirname "$0")" && pwd)"

# Cleanup ALWAYS runs — a JSON-model arducopter without gz sim behind it
# wedges on port 5760 and ignores SIGTERM (hence -9).
cleanup() {
  pkill -9 -f "[a]rducopter" 2>/dev/null || true
  pkill -9 -f "gz si[m]"     2>/dev/null || true
}
trap cleanup EXIT

export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"

echo "=== gz sim (headless server) ==="
# Before the first redirect into it: on a fresh user the gz log's folder did
# not exist yet, the background redirect failed, gz never started, and the
# JSON SITL then waited for a physics backend that was not there.
mkdir -p "$HOME/sitl-run"
cleanup                       # clear any wedged leftovers from prior runs
sleep 1
nohup gz sim -s -r "$HOME/ardupilot_gazebo/worlds/iris_runway.sdf" \
  > ~/sitl-run/gz.log 2>&1 &
sleep 6

echo "=== SITL with JSON backend (talks to gazebo plugin) ==="
# The ArduPilot GeoFence backstop, loaded as sitl/start_sitl.sh loads it
# (GUARDRAIL_FENCE=0 starts the stock autopilot), and the EEPROM wiped at each
# start (SITL_WIPE=0 keeps it). Until 2026-11-02 this rail loaded neither: its
# first run read FENCE_ENABLE 0, so the last-line backstop behind the Shield
# was off on the Gazebo rail.
DEFAULTS="$HOME/ardupilot/Tools/autotest/default_params/copter.parm,$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm"
FENCE_PARM="$DIR/fence/guardrail_fence.parm"
if [ "${GUARDRAIL_FENCE:-1}" = "1" ]; then
  [ -f "$FENCE_PARM" ] || { echo "fence parm missing: $FENCE_PARM"; exit 1; }
  DEFAULTS="$DEFAULTS,$FENCE_PARM"
  echo "GeoFence backstop: $FENCE_PARM"
else
  echo "GeoFence backstop OFF (GUARDRAIL_FENCE=0)"
fi
WIPE=""
[ "${SITL_WIPE:-1}" = "1" ] && WIPE="-w"
mkdir -p "$HOME/sitl-run" && cd "$HOME/sitl-run"
nohup bash -c "tail -f /dev/null | $HOME/ardupilot/build/sitl/bin/arducopter \
  --model JSON --speedup 1 \
  --defaults '$DEFAULTS' \
  --home -35.363261,149.165230,584,0 -I0 $WIPE" > ~/sitl-run/sitl.log 2>&1 &
sleep 8

echo "=== guardrail mission (pymavlink rail) ==="
cd "$DIR/.."                  # the repository root, found from this script
~/venv-ap/bin/python sitl/run_sitl_demo.py --shield "$SHIELD" $DYN \
  --tag "gazebo_shield_${SHIELD}$( [ -n "$DYN" ] && echo _dynamic )" 2>&1 \
  | { grep -v "EOF on TCP" || true; }   # a fully filtered output is not a failure

echo "=== done (cleanup runs via trap) ==="
