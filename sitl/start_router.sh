#!/usr/bin/env bash
# MAVLink fan-out for the dev topology: one ArduPilot SITL, two ground
# stations side by side - MAVROS 2 (the Shield's path) and Mission Planner on
# Windows (the operator's view). Run in WSL after start_sitl.sh:
#
#   bash start_router.sh            # foreground; run_ros2_demo.sh backgrounds it
#
# Config: sitl/mavlink-router/main.conf, copied to ~/sitl-run with
#   @GCS_HOST@     Mission Planner's host. Default: the Windows host as WSL 2
#                  (NAT networking) sees it, the default gateway. Set GCS_HOST
#                  to override (e.g. 127.0.0.1 with mirrored networking).
#   @MAVROS_HOST@  where MAVROS runs. 127.0.0.1 on one desktop (dev); the
#                  Orin's address in the hil topology.
#
# mavlink-routerd is the grant's router. It is not packaged for Ubuntu 24.04
# and is not installed on this PC yet; until it is, the SAME fan-out runs on
# MAVProxy (~/venv-ap, already installed), and the choice is printed and
# exported so the run record says which one carried the flight.
# MAVProxy is started with --streamrate=-1 so it never changes the stream
# rates MAVROS asks for on the shared SITL link.
set -eo pipefail
# mavlink-routerd is built without sudo into ~/.local (see sitl/README.md);
# a non-login shell does not have that on PATH.
export PATH="$HOME/.local/bin:$PATH"
DIR="$(cd "$(dirname "$0")" && pwd)"
RUN="$HOME/sitl-run"
mkdir -p "$RUN"
GCS_HOST="${GCS_HOST:-$(ip route show default | awk '/default/ {print $3; exit}')}"
MAVROS_HOST="${MAVROS_HOST:-127.0.0.1}"
SITL_PORT="${SITL_ROUTER_PORT:-14550}"
MAVROS_PORT="${MAVROS_UDP_PORT:-14555}"
GCS_PORT="${GCS_UDP_PORT:-14550}"
CONF="$RUN/mavlink-router.conf"
sed -e "s/@GCS_HOST@/$GCS_HOST/" -e "s/@MAVROS_HOST@/$MAVROS_HOST/" \
    -e "s/^Port = 14555/Port = $MAVROS_PORT/" "$DIR/mavlink-router/main.conf" > "$CONF"

echo "router: SITL udp:$SITL_PORT -> MAVROS $MAVROS_HOST:$MAVROS_PORT + GCS $GCS_HOST:$GCS_PORT"
if command -v mavlink-routerd > /dev/null 2>&1; then
  echo "router: mavlink-routerd -c $CONF"
  echo "mavlink-routerd" > "$RUN/router.kind"
  exec mavlink-routerd -c "$CONF"
fi

MAVPROXY="${MAVPROXY:-$HOME/venv-ap/bin/mavproxy.py}"
[ -x "$MAVPROXY" ] || { echo "router: neither mavlink-routerd nor $MAVPROXY found"; exit 1; }
echo "router: mavlink-routerd not installed - MAVProxy fallback ($MAVPROXY)"
echo "mavproxy" > "$RUN/router.kind"
cd "$RUN"
exec "$MAVPROXY" --master="udpin:0.0.0.0:$SITL_PORT" \
  --out="udp:$MAVROS_HOST:$MAVROS_PORT" --out="udp:$GCS_HOST:$GCS_PORT" \
  --streamrate=-1 --daemon --non-interactive --state-basedir="$RUN"
