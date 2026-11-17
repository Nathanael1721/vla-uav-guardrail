#!/usr/bin/env bash
# The hil rig's desktop side, one mission at a time: ArduPilot SITL and
# mavlink-routerd in WSL, restarted at a scenario's start for each mission of
# the stress harness's hil campaign (experiments/sweep_scenarios.py --backend
# hil, docs/RUNBOOK-orin-hil.md step 14). Run in WSL; from Windows:
#
#   wsl.exe -d Ubuntu -- bash /mnt/d/.../sitl/hil_desktop.sh start \
#       --home -35.36505763,149.16523,584,220.12 --mavros-host 10.42.0.3
#   wsl.exe -d Ubuntu -- bash /mnt/d/.../sitl/hil_desktop.sh stop
#
# (from Git Bash, set MSYS_NO_PATHCONV=1 first, or the /mnt/... path is
# rewritten into a Windows one.)
#
#   start   kill any SITL and router left over, start sitl/start_sitl.sh with
#           SITL_HOME (fresh EEPROM, the GeoFence backstop loaded: its own
#           defaults) and sitl/start_router.sh with MAVROS_HOST, wait until
#           SITL is up and the EKF has had --settle-s seconds, print
#           "HIL_DESKTOP READY ..." and stay in the foreground until the two
#           exit or `stop` kills them - so the harness's wsl.exe keeps the
#           distro up for the whole mission.
#   stop    kill SITL, its stdin keeper, the router (mavlink-routerd, or
#           the MAVProxy fallback) and the stressor process (TERM first, so
#           it writes its record's last state).
#   status  what is running.
#
# Options of start: --home LAT,LON,ALT,HEADING (default: start_sitl.sh's
# CMAC home), --mavros-host ADDR (default 10.42.0.3, the AGX Orin on the
# cable: deploy/topologies/hil.env), --settle-s S (default 20, the runbook's
# EKF wait), --ready-timeout-s S (default 60). The Shield node checks the
# home SITL reports against the scenario's start before it arms
# (sitl/ros2_shield_node.py --start), so a wrong --home is refused there.
#
# The simulator's stressors (2026-11-07): --stress-plan FILE --stress-record
# FILE (both, WSL paths; experiments/sweep_scenarios.py rail_stress_plan).
# Once SITL and the router are up, sitl/sitl_stressors.py connects to SITL's
# SERIAL0 (tcp:5760; the router is on SERIAL2), sets the plan's boot SIM_*
# parameters and reads them back; READY is printed only after it says
# "SITL_STRESS BOOT OK", else start fails (exit 1) and nothing flies. It
# stays to apply the scheduled changes at the episode's t = 0 the Shield node
# announces, writing its record to FILE; `stop` ends it with the rest. Its
# Python: SITL_STRESS_PYTHON, default ~/venv-ap/bin/python (pymavlink, as
# sitl/setup_sitl.sh pins it).
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
RUN="$HOME/sitl-run"
CMD="${1:-}"
[ "$#" -gt 0 ] && shift

HOME_LOC=""
MAVROS_HOST="10.42.0.3"
SETTLE_S=20
READY_TIMEOUT_S=60
STRESS_PLAN=""
STRESS_RECORD=""
STRESS_PY="${SITL_STRESS_PYTHON:-$HOME/venv-ap/bin/python}"
while [ "$#" -gt 0 ]; do
  case "$1" in
    --home) HOME_LOC="${2:?--home needs LAT,LON,ALT,HEADING}"; shift 2 ;;
    --mavros-host) MAVROS_HOST="${2:?--mavros-host needs an address}"; shift 2 ;;
    --settle-s) SETTLE_S="${2:?--settle-s needs seconds}"; shift 2 ;;
    --ready-timeout-s) READY_TIMEOUT_S="${2:?--ready-timeout-s needs seconds}"; shift 2 ;;
    --stress-plan) STRESS_PLAN="${2:?--stress-plan needs a file}"; shift 2 ;;
    --stress-record) STRESS_RECORD="${2:?--stress-record needs a file}"; shift 2 ;;
    *) echo "hil_desktop: unknown option '$1'" >&2; exit 64 ;;
  esac
done
NUM='-?[0-9]+(\.[0-9]+)?'
if [ -n "$HOME_LOC" ] && ! [[ "$HOME_LOC" =~ ^$NUM,$NUM,$NUM,$NUM$ ]]; then
  echo "hil_desktop: --home must be lat,lon,alt,heading - got '$HOME_LOC'" >&2
  exit 64
fi
if { [ -n "$STRESS_PLAN" ] && [ -z "$STRESS_RECORD" ]; } \
    || { [ -z "$STRESS_PLAN" ] && [ -n "$STRESS_RECORD" ]; }; then
  echo "hil_desktop: --stress-plan and --stress-record go together" >&2
  exit 64
fi
[[ "$SETTLE_S" =~ ^[0-9]+(\.[0-9]+)?$ ]] || { echo "hil_desktop: --settle-s '$SETTLE_S'" >&2; exit 64; }
[[ "$READY_TIMEOUT_S" =~ ^[0-9]+$ ]] || { echo "hil_desktop: --ready-timeout-s '$READY_TIMEOUT_S'" >&2; exit 64; }

stop_all() {
  # The stressor process first, with TERM: it writes its record's last
  # state (sitl/sitl_stressors.py; the record is rewritten at every step)
  # and sets back what it must restore (a battery cell's BATT_*), while SITL
  # still runs: up to 5 s.
  if pgrep -f "[s]itl_stressors.py" > /dev/null 2>&1; then
    pkill -TERM -f "[s]itl_stressors.py" 2> /dev/null || true
    for _ in $(seq 1 25); do
      pgrep -f "[s]itl_stressors.py" > /dev/null 2>&1 || break
      sleep 0.2
    done
    pkill -9 -f "[s]itl_stressors.py" 2> /dev/null || true
  fi
  pkill -9 -f "[s]tart_router.sh"            2> /dev/null || true
  pkill -9 -f "[m]avlink-routerd"            2> /dev/null || true
  pkill -9 -f "[m]avproxy.py --master=udpin" 2> /dev/null || true
  pkill -9 -f "[s]tart_sitl.sh"              2> /dev/null || true
  pkill -9 -f "[a]rducopter"                 2> /dev/null || true
  # start_sitl.sh feeds SITL's console from `tail -f /dev/null`; it outlives
  # a killed SITL. Only the one whose parent shell is gone.
  for p in $(pgrep -f "^tail -f /dev/null$" 2> /dev/null || true); do
    [ "$(ps -o ppid= -p "$p" 2> /dev/null | tr -d ' ')" = "1" ] && kill -9 "$p" 2> /dev/null || true
  done
  for _ in $(seq 1 20); do
    pgrep -f "[a]rducopter|[m]avlink-routerd" > /dev/null 2>&1 || return 0
    sleep 0.5
  done
  echo "hil_desktop: SITL or the router is still running after stop" >&2
  return 1
}

case "$CMD" in
  stop)
    stop_all
    echo "hil_desktop: stopped"
    exit 0
    ;;
  status)
    pgrep -af "[a]rducopter|[m]avlink-routerd|[m]avproxy.py --master=udpin|[s]itl_stressors.py" || echo "nothing running"
    exit 0
    ;;
  start) ;;
  *)
    echo "usage: hil_desktop.sh start|stop|status [--home LAT,LON,ALT,HDG] [--mavros-host ADDR] [--settle-s S] [--stress-plan FILE --stress-record FILE]" >&2
    exit 64
    ;;
esac

stop_all
mkdir -p "$RUN"
rm -f "$RUN/router.kind"
if [ -n "$HOME_LOC" ]; then
  SITL_HOME="$HOME_LOC" bash "$DIR/start_sitl.sh" > "$RUN/sitl.log" 2>&1 &
else
  bash "$DIR/start_sitl.sh" > "$RUN/sitl.log" 2>&1 &
fi
SITL_PID=$!
# SITL is up once its console has printed the home and it is still alive.
t0=$SECONDS
until grep -q "^home: " "$RUN/sitl.log" 2> /dev/null && pgrep -f "[a]rducopter" > /dev/null; do
  if ! kill -0 "$SITL_PID" 2> /dev/null; then
    echo "hil_desktop: SITL exited at start:" >&2
    tail -20 "$RUN/sitl.log" >&2
    exit 1
  fi
  if [ $((SECONDS - t0)) -ge "$READY_TIMEOUT_S" ]; then
    echo "hil_desktop: SITL not up within ${READY_TIMEOUT_S} s:" >&2
    tail -20 "$RUN/sitl.log" >&2
    stop_all || true
    exit 1
  fi
  sleep 0.5
done
MAVROS_HOST="$MAVROS_HOST" bash "$DIR/start_router.sh" > "$RUN/router.log" 2>&1 &
ROUTER_PID=$!
sleep 2
if ! kill -0 "$ROUTER_PID" 2> /dev/null; then
  echo "hil_desktop: the router exited at start:" >&2
  tail -20 "$RUN/router.log" >&2
  stop_all || true
  exit 1
fi
STRESS_PID=""
if [ -n "$STRESS_PLAN" ]; then
  # The simulator's stressors: boot parameters set and read back before
  # READY, or nothing flies (the header says what follows).
  [ -f "$STRESS_PLAN" ] || { echo "hil_desktop: no stress plan at $STRESS_PLAN" >&2; stop_all || true; exit 1; }
  [ -x "$STRESS_PY" ] || { echo "hil_desktop: no Python for the stressors at $STRESS_PY (SITL_STRESS_PYTHON)" >&2; stop_all || true; exit 1; }
  "$STRESS_PY" "$DIR/sitl_stressors.py" --plan "$STRESS_PLAN" --record "$STRESS_RECORD" \
    > "$RUN/stress.log" 2>&1 &
  STRESS_PID=$!
  t0=$SECONDS
  until grep -q "^SITL_STRESS BOOT OK" "$RUN/stress.log" 2> /dev/null; do
    if grep -q "^SITL_STRESS BOOT FAILED" "$RUN/stress.log" 2> /dev/null \
        || { ! kill -0 "$STRESS_PID" 2> /dev/null \
             && ! grep -q "^SITL_STRESS BOOT OK" "$RUN/stress.log" 2> /dev/null; }; then
      echo "hil_desktop: the stressors did not set and read back their boot parameters:" >&2
      tail -20 "$RUN/stress.log" >&2
      stop_all || true
      exit 1
    fi
    if [ $((SECONDS - t0)) -ge "$READY_TIMEOUT_S" ]; then
      echo "hil_desktop: the stressors were not ready within ${READY_TIMEOUT_S} s:" >&2
      tail -20 "$RUN/stress.log" >&2
      stop_all || true
      exit 1
    fi
    sleep 0.5
  done
  echo "hil_desktop: $(grep -m1 '^SITL_STRESS BOOT OK' "$RUN/stress.log")"
fi
echo "hil_desktop: $(grep -m1 '^home: ' "$RUN/sitl.log"), router $(cat "$RUN/router.kind" 2> /dev/null || echo unknown) -> MAVROS $MAVROS_HOST; EKF settling ${SETTLE_S} s"
sleep "$SETTLE_S"
if ! kill -0 "$SITL_PID" 2> /dev/null; then
  echo "hil_desktop: SITL died while the EKF settled:" >&2
  tail -20 "$RUN/sitl.log" >&2
  exit 1
fi
if [ -n "$STRESS_PID" ] && ! kill -0 "$STRESS_PID" 2> /dev/null \
    && ! grep -q "^SITL_STRESS DONE" "$RUN/stress.log" 2> /dev/null; then
  # Gone before the mission with changes still to apply: they never will be.
  echo "hil_desktop: the stressors exited while the EKF settled:" >&2
  tail -20 "$RUN/stress.log" >&2
  stop_all || true
  exit 1
fi
echo "HIL_DESKTOP READY sitl_pid=$SITL_PID router_pid=$ROUTER_PID home=${HOME_LOC:-default} stress=${STRESS_PLAN:-none}"
# In the foreground until SITL and the router exit (`stop`, or the next
# `start`), so the wsl.exe that started this keeps the distro up.
wait
