#!/usr/bin/env bash
# Entrypoint of the ArduPilot SITL image (deploy/docker/sitl.Dockerfile).
# Desktop only: dev and hil. The flight topology has no simulator.
#
# It starts the autopilot the way the native rails do, so a run in a
# container and one in WSL differ only in where the process runs:
#
#   SITL_MODEL=quad           as sitl/start_sitl.sh: ArduPilot's own physics;
#                             --defaults copter.parm, then the GeoFence
#                             backstop sitl/fence/guardrail_fence.parm
#                             (GUARDRAIL_FENCE=1, the default; 0 = the stock
#                             autopilot); the CMAC home of every stored run.
#   SITL_MODEL=airsim-copter  as demo/pas_ardupilot/start_ardupilot.sh
#                             (rail.py sitl-args): Project AirSim computes the
#                             physics and sensors through its ardupilot-api
#                             controller; --defaults copter.parm,
#                             airsim-quadX.parm, then the airframe file
#                             SITL_AIRFRAME_PARM (for the CityLife scene
#                             demo/pas_ardupilot/citylife-quad.param, which
#                             carries that rail's own fence). SITL_HOME must be
#                             the scene's home-geo-point
#                             (python3 demo/pas_ardupilot/rail.py home): the
#                             AirSim backend measures position from --home to
#                             each GPS fix, so the CMAC default would put the
#                             vehicle thousands of kilometres from the scene.
#                             SITL binds UDP 9003 for sensor input and sends
#                             motor outputs to AIRSIM_HOST:9002.
#
# In both, the EEPROM is wiped at every start (SITL_WIPE=1, the default; 0
# keeps it), as sitl/start_sitl.sh does: the MAVLink adapter uploads the
# policy's keep-out zones as fence polygons, which ArduPilot stores in the
# EEPROM, and a zone left by the previous flight would otherwise be armed from
# boot. `docker compose restart sitl` between KPI missions relies on this.
# SERIAL0 is a TCP server (5760) and SERIAL2 sends to mavlink-router on UDP
# SITL_MAVLINK_PORT, as in sitl/start_sitl.sh. --speedup is fixed at 1:
# sim_speedup = 1.0 is mandatory for a KPI-bearing run (grant, Stress Testing
# p.1), so it is not a setting.
#
# The repository is mounted read-only at VLAGUARD_ROOT (default /opt/vlaguard)
# for the fence and airframe parameter files. VLAGUARD_DRY_RUN=1 prints the
# command, shell-quoted, instead of running it.
#
# Exit codes: 64 bad configuration, 66 a file it needs is missing, 68 a host
# name did not resolve.
set -eo pipefail

: "${SITL_MODEL:=quad}"
: "${SITL_MAVLINK_PORT:=14550}"
: "${SITL_ROUTER_HOST:=127.0.0.1}"
CMAC_HOME="-35.363261,149.165230,584,0"
ROOT="${VLAGUARD_ROOT:-/opt/vlaguard}"
PARAM_DIR="${SITL_PARAM_DIR:-/opt/ardupilot/params}"
FENCE_PARM="${SITL_FENCE_PARM:-$ROOT/sitl/fence/guardrail_fence.parm}"
DRY="${VLAGUARD_DRY_RUN:-0}"

die() { echo "sitl-entrypoint: $2" >&2; exit "$1"; }

resolve_ip() {
  local host="$1" ip
  if [[ "$host" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "$host"
    return 0
  fi
  ip="$(getent ahostsv4 "$host" 2>/dev/null | awk 'NR==1 {print $1}')" || true
  [ -n "$ip" ] || die 68 "cannot resolve '$host' to an IPv4 address"
  echo "$ip"
}

wipe=()
case "${SITL_WIPE:-1}" in
  1) wipe=(-w) ;;
  0) ;;
  *) die 64 "SITL_WIPE='${SITL_WIPE}' is not 1 or 0" ;;
esac

extra=()
case "$SITL_MODEL" in
  quad)
    home="${SITL_HOME:-$CMAC_HOME}"
    params="$PARAM_DIR/copter.parm"
    case "${GUARDRAIL_FENCE:-1}" in
      1) [ -f "$FENCE_PARM" ] || die 66 "GeoFence backstop $FENCE_PARM not found (is the repository mounted at $ROOT?)"
         params="$params,$FENCE_PARM" ;;
      0) echo "sitl-entrypoint: GeoFence backstop OFF (GUARDRAIL_FENCE=0)" >&2 ;;
      *) die 64 "GUARDRAIL_FENCE='${GUARDRAIL_FENCE}' is not 1 or 0" ;;
    esac
    ;;
  airsim-copter)
    home="${SITL_HOME:-}"
    case "$home" in
      "") die 64 "SITL_MODEL=airsim-copter needs SITL_HOME = the scene's home-geo-point (python3 demo/pas_ardupilot/rail.py home)" ;;
      -35.363261,149.165230*) die 64 "SITL_HOME is the CMAC default; with airsim-copter it must be the scene's home-geo-point (python3 demo/pas_ardupilot/rail.py home)" ;;
    esac
    [ -n "${SITL_AIRFRAME_PARM:-}" ] \
      || die 64 "SITL_MODEL=airsim-copter needs SITL_AIRFRAME_PARM, e.g. $ROOT/demo/pas_ardupilot/citylife-quad.param"
    [ -f "$SITL_AIRFRAME_PARM" ] || die 66 "airframe parameter file $SITL_AIRFRAME_PARM not found"
    airsim_ip="$(resolve_ip "${AIRSIM_HOST:?AIRSIM_HOST is not set}")"
    params="$PARAM_DIR/copter.parm,$PARAM_DIR/airsim-quadX.parm,$SITL_AIRFRAME_PARM"
    extra=(--sim-address "$airsim_ip" --sim-port-in 9003 --sim-port-out 9002)
    ;;
  *)
    die 64 "SITL_MODEL='$SITL_MODEL' is not quad or airsim-copter"
    ;;
esac

cmd=(arducopter --model "$SITL_MODEL" --speedup 1 --defaults "$params"
     --home "$home" -I0 "${wipe[@]}" "${extra[@]}"
     --serial0 tcp:0 --serial2 "udpclient:${SITL_ROUTER_HOST}:${SITL_MAVLINK_PORT}")

if [ "$DRY" = "1" ]; then
  printf 'EXEC:'
  printf ' %q' "${cmd[@]}"
  printf '\n'
  exit 0
fi

mkdir -p /sitl-run
cd /sitl-run            # eeprom.bin and dataflash logs live here (a volume)
# SITL's console exits on stdin EOF; tail -f /dev/null keeps stdin open
# (sitl/start_sitl.sh, learned the hard way).
tail -f /dev/null | "${cmd[@]}"
