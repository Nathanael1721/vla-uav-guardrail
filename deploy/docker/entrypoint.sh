#!/usr/bin/env bash
# Entrypoint of the companion image (deploy/docker/companion.Dockerfile).
#
# One image, every topology. The role is the first argument; VLAGUARD_TOPOLOGY
# (from deploy/topologies/<name>.env through compose) selects CONFIGURATION
# only. The label a run's manifest carries is not taken from it: the Shield
# node reads the host and the MAVLink link it flies over and
# guardrail.manifest.detect_topology decides dev, hil or flight from that
# evidence. Nothing in here branches on the host.
#
#   router     sitl/start_router.sh - the native rail's own mavlink-router
#              start and configuration (sitl/mavlink-router/main.conf), with
#              the topology's addresses (dev and hil, desktop side)
#   mavros     MAVROS 2 against FCU_URL
#   mission    one mission, started the way the native rail starts it
#              (sitl/run_ros2_demo.sh with USE_LAUNCH=1): rosbag2 of the
#              grant's topics, then the launch file
#              sitl/ros2_ws/src/safety_shield/launch/guardrail_rail.launch.py
#              (VLA stub, MAVLink adapter, Shield node), then the replay
#              bundle re-packed with the closed bag.
#                mission [--shield on|off] [--dynamic] [args for the nodes ...]
#              Refused in flight unless VLAGUARD_ALLOW_FLIGHT_MISSION=yes.
#   profile    tools/profile_shield_tick.py profile --topology $VLAGUARD_TOPOLOGY
#   facts      tools/profile_shield_tick.py facts
#   vla-table  tools/vla_backend_table.py
#   shell      an interactive shell with ROS sourced
#
# The repository is NOT baked into the image. It is mounted at VLAGUARD_ROOT
# (read-only), so guardrail.manifest.code_revision() reads the real git
# checkout and a run on the Orin names the commit that flew.
#
# VLAGUARD_DRY_RUN=1 prints the commands, shell-quoted, instead of running
# them; the tests use it to check every role against every topology without
# Docker or ROS.
#
# Exit codes: 64 bad configuration, 66 repository not mounted, 68 a host name
# did not resolve, 77 refused for safety.
set -eo pipefail

ROOT="${VLAGUARD_ROOT:-/opt/vlaguard}"
PY="${VLAGUARD_PYTHON:-/opt/venv/bin/python}"
RUNTIME_DIR="${VLAGUARD_RUNTIME_DIR:-/tmp}"
DRY="${VLAGUARD_DRY_RUN:-0}"
LAUNCH_FILE="sitl/ros2_ws/src/safety_shield/launch/guardrail_rail.launch.py"

# The topics the native rail records (sitl/run_ros2_demo.sh); a test keeps the
# two lists equal.
BAG_TOPICS=(/vla/action_4d /vla/identity /shield/setpoint /shield/intercept
            /shield/mode_request /shield/fences /mavlink_adapter/status
            /mavros/setpoint_raw/local /mavros/setpoint_raw/target_local
            /mavros/state /mavros/local_position/pose)

die() { echo "vlaguard-entrypoint: $2" >&2; exit "$1"; }

run() {
  if [ "$DRY" = "1" ]; then
    printf 'EXEC:'
    printf ' %q' "$@"
    printf '\n'
    exit 0
  fi
  exec "$@"
}

# An IPv4 literal passes through; a name is resolved once, here, because
# mavlink-router wants addresses. A name that does not resolve stops the
# container rather than routing to nowhere.
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

role="${1:-help}"
[ "$#" -gt 0 ] && shift

case "$role" in
  help|-h|--help)
    awk 'NR == 1 { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "$0"
    exit 0
    ;;
esac

case "${VLAGUARD_TOPOLOGY:-}" in
  dev|hil|flight) ;;
  *) die 64 "VLAGUARD_TOPOLOGY='${VLAGUARD_TOPOLOGY:-}' is not dev, hil or flight; pass --env-file deploy/topologies/<name>.env" ;;
esac

[ -f "$ROOT/guardrail/shield.py" ] \
  || die 66 "no repository at $ROOT (mount the checkout there; see deploy/README.md)"

if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ "$DRY" != "1" ]; then
  case "$role" in
    mavros|mission|shell)
      die 64 "ROS 2 is not installed in this image (ROS_DISTRO='${ROS_DISTRO:-}')" ;;
  esac
fi

case "$role" in
  router)
    MAVROS_IP="$(resolve_ip "${MAVROS_HOST:?MAVROS_HOST is not set}")"
    GCS_IP="$(resolve_ip "${GCS_HOST:?GCS_HOST is not set}")"
    # One fan-out definition for the WSL rail and the containers: the native
    # script renders sitl/mavlink-router/main.conf with these addresses and
    # execs mavlink-routerd (installed in this image).
    run env GCS_HOST="$GCS_IP" MAVROS_HOST="$MAVROS_IP" \
        MAVROS_UDP_PORT="${MAVROS_UDP_PORT:?}" SITL_ROUTER_PORT="${SITL_MAVLINK_PORT:?}" \
        GCS_UDP_PORT="${GCS_UDP_PORT:?}" bash "$ROOT/sitl/start_router.sh"
    ;;
  mavros)
    [ -n "${FCU_URL:-}" ] \
      || die 64 "FCU_URL is empty. In flight set it to the autopilot link, e.g. serial:///dev/ttyACM0:115200, and FCU_DEVICE to the same device"
    if [[ "$FCU_URL" == serial://* ]] && [ "$DRY" != "1" ]; then
      dev="${FCU_URL#serial://}"
      dev="${dev%%:*}"
      [ -e "$dev" ] || die 64 "FCU_URL names $dev, which is not present in the container (add deploy/compose/docker-compose.orin.flight.yml and set FCU_DEVICE)"
    fi
    run ros2 run mavros mavros_node --ros-args \
      -p "fcu_url:=${FCU_URL}" -p "tgt_system:=1" -p "tgt_component:=1"
    ;;
  mission)
    if [ "$VLAGUARD_TOPOLOGY" = "flight" ] && [ "${VLAGUARD_ALLOW_FLIGHT_MISSION:-no}" != "yes" ]; then
      die 77 "refusing to run the Shield mission in the flight topology: it arms a real aircraft and takes off. Set VLAGUARD_ALLOW_FLIGHT_MISSION=yes only after the bench checks in docs/RUNBOOK-orin-hil.md"
    fi
    shield="on"; dynamic=""; extra=()
    while [ "$#" -gt 0 ]; do
      case "$1" in
        --shield) [ "$#" -ge 2 ] || die 64 "--shield needs on or off"; shield="$2"; shift 2 ;;
        --dynamic) dynamic="--dynamic"; shift ;;
        *) extra+=("$1"); shift ;;
      esac
    done
    case "$shield" in on|off) ;; *) die 64 "--shield '$shield' is not on or off" ;; esac
    # The episode directory, by the Shield node's own rule (default_tag) or --tag.
    tag="ros2_shield_${shield}"
    [ -n "$dynamic" ] && tag="${tag}_dynamic"
    for ((i = 0; i < ${#extra[@]}; i++)); do
      [ "${extra[$i]}" = "--tag" ] && tag="${extra[$((i + 1))]:-}"
    done
    case "$tag" in
      ""|*/*|.*) die 64 "--tag '$tag' must be a plain directory name" ;;
    esac
    out="$ROOT/demo/out/$tag"
    shield_args=(--shield "$shield")
    [ -n "$dynamic" ] && shield_args+=("$dynamic")
    shield_args+=("${extra[@]}")
    vla_args=""
    [ "${#extra[@]}" -gt 0 ] && vla_args="$(printf '%q ' "${extra[@]}")"
    launch=(ros2 launch "$ROOT/$LAUNCH_FILE" "python:=$PY" "repo_root:=$ROOT" "out:=$out"
            "shield_args:=$(printf '%q ' "${shield_args[@]}")" "vla_args:=$vla_args")
    pack=("$PY" -m guardrail.replay pack "$out")
    bag=(ros2 bag record -s mcap -o "$out/bag" "${BAG_TOPICS[@]}")
    # What the Shield node records as the link (it reads MAVROS's own fcu_url
    # parameter first and falls back to this) and which router carried it.
    export MAVROS_FCU_URL="${FCU_URL:-}"
    export MAVLINK_ROUTER="${MAVLINK_ROUTER:-mavlink-routerd (vlaguard/companion image)}"
    export GUARDRAIL_ROOT="$ROOT" GUARDRAIL_PYTHON="$PY"
    if [ "$DRY" = "1" ]; then
      [ "${VLAGUARD_BAG:-1}" = "1" ] && { printf 'BAG:'; printf ' %q' "${bag[@]}"; printf '\n'; }
      printf 'EXEC:'; printf ' %q' "${launch[@]}"; printf '\n'
      printf 'THEN:'; printf ' %q' "${pack[@]}"; printf '\n'
      exit 0
    fi
    mkdir -p "$out"
    bag_pid=""
    if [ "${VLAGUARD_BAG:-1}" = "1" ]; then
      if [ -e "$out/bag" ]; then
        mkdir -p "$out/_previous"
        mv "$out/bag" "$out/_previous/bag-$(date -u +%Y%m%dT%H%M%SZ)"
      fi
      "${bag[@]}" > "$RUNTIME_DIR/bag.log" 2>&1 &
      bag_pid=$!
      sleep 2
    fi
    set +e
    "${launch[@]}"
    rc=$?
    set -e
    # SIGTERM, not SIGINT, so rosbag2 closes the file and writes metadata.yaml
    # (sitl/run_ros2_demo.sh, stop_bag).
    if [ -n "$bag_pid" ] && kill -0 "$bag_pid" 2>/dev/null; then
      kill -TERM "$bag_pid" 2>/dev/null || true
      for _ in $(seq 1 20); do kill -0 "$bag_pid" 2>/dev/null || break; sleep 0.5; done
      kill -9 "$bag_pid" 2>/dev/null || true
    fi
    cd "$ROOT"
    "${pack[@]}" || echo "vlaguard-entrypoint: [replay] re-pack FAILED (see above)" >&2
    exit "$rc"
    ;;
  profile)
    run "$PY" "$ROOT/tools/profile_shield_tick.py" profile --topology "$VLAGUARD_TOPOLOGY" "$@"
    ;;
  facts)
    run "$PY" "$ROOT/tools/profile_shield_tick.py" facts
    ;;
  vla-table)
    run "$PY" "$ROOT/tools/vla_backend_table.py" "$@"
    ;;
  shell)
    run bash
    ;;
  *)
    die 64 "unknown role '$role' (router, mavros, mission, profile, facts, vla-table, shell)"
    ;;
esac
