#!/usr/bin/env bash
# The perception rail's hil follow flown by the lab's own VLA, AIOT-VLA, ON THE
# AGX ORIN (docs/DESIGN-projectairsim-ardupilot.md, "hil"): the perception node
# (demo/pas_ardupilot/perception_node.py --pilot vla --mission follow) runs
# natively on the Orin - ROS 2 Jazzy's rclpy and mavros_msgs from apt, and
# ~/vla-drone-env for torch, the detector and the Project AirSim client, which
# the companion image never carries - beside MAVROS 2 in the companion
# container on the same Orin and the AIOT-VLA server on 127.0.0.1; Project
# AirSim CityLife and ArduPilot SITL run on the desktop.
#
#   G1_VERDICT=~/g1_verdict.json TAG=pas_hil_aiot_01 bash deploy/orin/run_pas_perception.sh [node args...]
#
# First, in this order (the design doc has each command):
#   desktop  .\scripts\run_pas_ardupilot.ps1 -Step scene -VlaCameras
#            .\scripts\run_pas_ardupilot.ps1 -Step sitl -SkipSim -Orin <the Orin's address>
#   Orin     docker compose --env-file deploy/topologies/hil.env \
#                -f deploy/compose/docker-compose.orin.yml up -d mavros
#            ~/vla-drone-env/bin/python demo/vla_server_foreign.py serve --model aiotvla \
#                --weights ~/models/aiotvla/v10b --port 8791 --front-source detector --horizon-s 0.5
#
# The ROS 2 graph is hil.env's, read from it here and never copied: the domain,
# the RMW, the Cyclone DDS peers (deploy/dds/cyclonedds.xml expands them), and
# MAVROS's FCU_URL as MAVROS_FCU_URL, which the node's link record falls back
# to; the simulator is hil.env's DESKTOP_IP. Refused here: --dev-yaml (an
# unsigned YAML policy is never KPI-grade), --start-sitl (ArduPilot runs on
# the desktop, after its scene) and the flags that fix the topology (--link,
# --scene-mode / --scene-only, --sim-host, --camera), in any abbreviation
# argparse would take. Environment knobs:
#   G1_VERDICT  the desktop's demo/out/g1_*/g1_verdict.json, copied here (required)
#   TAG         the run folder under demo/out/ (default pas_hil_aiot_<date>)
#   SECONDS_    mission length in s (default 420)
#   BUNDLE      a signed policy bundle (default: the node's own, follow_car_citylife's)
#   FRONT_CAPTURE_INTERVAL_S  FrontCamera's capture interval the desktop's scene
#               step was given (-FrontCaptureIntervalS), recorded with the run
#   VLA_PORT    the AIOT-VLA server's port on 127.0.0.1 (default 8791)
#   VENV        the Python environment (default ~/vla-drone-env)
#   ROS_SETUP   ROS 2's setup script (default /opt/ros/jazzy/setup.bash)
# VLAGUARD_DRY_RUN=1 prints the graph's values (ENV:) and the command (EXEC:)
# and starts nothing (tests/test_deploy_configs.py).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HIL_ENV="$ROOT/deploy/topologies/hil.env"
DRY="${VLAGUARD_DRY_RUN:-0}"

die() { echo "*** $2" >&2; exit "$1"; }

# One value of hil.env (KEY=VALUE lines; the last one wins).
topo() {
  local v
  v="$(sed -n "s/^$1=//p" "$HIL_ENV" | tail -n 1 | tr -d '\r')"
  [ -n "$v" ] || die 64 "$HIL_ENV sets no $1"
  printf '%s' "$v"
}

for a in "$@"; do
  case "$a" in
    --dev*)
      die 64 "$a: a YAML policy flown with --dev-yaml is a development run, never KPI-grade; this is the hil run" ;;
    --start-s*)
      die 64 "$a: ArduPilot SITL runs on the desktop in hil (run_pas_ardupilot.ps1 -Step sitl -Orin), after its scene" ;;
    --lin*|--scene*|--sim*|--cam*)
      die 64 "$a: the hil topology fixes it here (--link mavros --scene-mode attach --sim-host <hil.env DESKTOP_IP>)" ;;
  esac
done

# (No apostrophe in this message: inside "${...:?word}" bash reads one as a quote.)
: "${G1_VERDICT:?set G1_VERDICT to a copy of the desktop g1_verdict.json (demo/out/g1_*/)}"
[ -f "$G1_VERDICT" ] || die 66 "no G1 verdict at $G1_VERDICT (copy the desktop's demo/out/g1_*/g1_verdict.json here)"
TAG="${TAG:-pas_hil_aiot_$(date +%Y%m%d_%H%M%S)}"
case "$TAG" in
  ""|*/*|.*) die 64 "TAG '$TAG' must be a plain directory name under demo/out" ;;
esac
SECONDS_="${SECONDS_:-420}"
VLA_PORT="${VLA_PORT:-8791}"
VENV="${VENV:-$HOME/vla-drone-env}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

# The ROS 2 graph MAVROS in the companion container is on (hil.env, compose's
# companion environment), for this host's node.
ROS_DOMAIN_ID="$(topo ROS_DOMAIN_ID)"
RMW_IMPLEMENTATION="$(topo RMW_IMPLEMENTATION)"
VLAGUARD_DDS_PEER_A="$(topo VLAGUARD_DDS_PEER_A)"
VLAGUARD_DDS_PEER_B="$(topo VLAGUARD_DDS_PEER_B)"
MAVROS_FCU_URL="$(topo FCU_URL)"
DESKTOP_IP="$(topo DESKTOP_IP)"
CYCLONEDDS_URI="file://$ROOT/deploy/dds/cyclonedds.xml"
export ROS_DOMAIN_ID RMW_IMPLEMENTATION VLAGUARD_DDS_PEER_A VLAGUARD_DDS_PEER_B
export MAVROS_FCU_URL CYCLONEDDS_URI

args=(demo/pas_ardupilot/perception_node.py
      --link mavros --scene-mode attach --sim-host "$DESKTOP_IP"
      --g1-verdict "$G1_VERDICT" --tag "$TAG" --max-s "$SECONDS_"
      --pilot vla --mission follow --vla-server "127.0.0.1:$VLA_PORT"
      --vla-front-source detector --vla-step-as-rate 0.5 --vla-rate-yaw rate
      --cadence pipelined --allow-unstamped-pipelined
      --vla-cruise-mps 4.5 --vla-speed-cap-mps 5)
if [ -n "${BUNDLE:-}" ]; then
  args+=(--bundle "$BUNDLE")
fi
if [ -n "${FRONT_CAPTURE_INTERVAL_S:-}" ]; then
  args+=(--front-capture-interval-s "$FRONT_CAPTURE_INTERVAL_S")
fi
args+=("$@")

cd "$ROOT"
if [ "$DRY" = "1" ]; then
  for k in ROS_DOMAIN_ID RMW_IMPLEMENTATION VLAGUARD_DDS_PEER_A VLAGUARD_DDS_PEER_B \
           MAVROS_FCU_URL CYCLONEDDS_URI; do
    printf 'ENV: %s=%s\n' "$k" "${!k}"
  done
  printf 'EXEC: python'
  printf ' %q' "${args[@]}"
  printf '\n'
  exit 0
fi

for f in demo/out/citymap_citylife/occ_day.npz demo/out/citymap_citylife/occ_day_flightband_6to14.npz; do
  [ -f "$f" ] || die 2 "missing $f (deploy/orin/sync_follow_controller.ps1 copies it)"
done
[ -f "$ROS_SETUP" ] || die 2 "no ROS 2 at $ROS_SETUP (apt: ros-jazzy-ros-base ros-jazzy-mavros-msgs ros-jazzy-rmw-cyclonedds-cpp)"
rmw_lib="$(dirname "$ROS_SETUP")/lib/lib${RMW_IMPLEMENTATION}.so"
[ -e "$rmw_lib" ] || die 2 "hil.env's RMW $RMW_IMPLEMENTATION is not installed beside ROS 2 ($rmw_lib)"
[ -f "$VENV/bin/activate" ] || die 2 "no Python environment at $VENV (deploy/orin/agx_vla_env.sh builds it)"
# ROS's setup script and the venv's activate read unset variables.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
set -u

python - <<'EOF'
import importlib
import sys
bad = []
for m in ("rclpy", "mavros_msgs.msg", "torch", "transformers", "projectairsim"):
    try:
        importlib.import_module(m)
    except Exception as exc:                                      # noqa: BLE001
        bad.append(f"{m} ({type(exc).__name__}: {exc})")
if bad:
    sys.exit("*** not importable with ROS 2 sourced and the venv active: " + "; ".join(bad)
             + " (ROS 2 from apt: ros-jazzy-ros-base, ros-jazzy-mavros-msgs; the rest: "
               "deploy/orin/agx_vla_env.sh)")
print(f"==> rclpy, mavros_msgs, torch, transformers and projectairsim import "
      f"(Python {sys.version.split()[0]})")
EOF
python - "$DESKTOP_IP" "$VLA_PORT" <<'EOF'
import socket
import sys
desk, vla_port = sys.argv[1], int(sys.argv[2])
for host, port, what in ((desk, 8989, "Project AirSim (topics)"),
                         (desk, 8990, "Project AirSim (services)"),
                         ("127.0.0.1", vla_port, "the AIOT-VLA server")):
    s = socket.socket()
    s.settimeout(3)
    try:
        s.connect((host, port))
    except OSError as exc:
        sys.exit(f"*** {what} {host}:{port} unreachable ({exc}): is it up (and, for the "
                 f"simulator, does the desktop's firewall let the Orin in)?")
    finally:
        s.close()
print(f"==> Project AirSim {desk} (8989, 8990) and the VLA server 127.0.0.1:{vla_port} reachable")
EOF

echo "==> ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION peers" \
     "$VLAGUARD_DDS_PEER_A $VLAGUARD_DDS_PEER_B; MAVROS fcu_url $MAVROS_FCU_URL"
rm -f "demo/out/$TAG/metrics.json"
echo "==> python ${args[*]}"
exec python "${args[@]}"
