#!/usr/bin/env bash
# AP_DDS SITL setup (ARCH-08) for WSL Ubuntu 24.04 + ROS 2 Jazzy - run as user.
# Builds a SEPARATE ArduPilot checkout with DDS enabled, the Micro-XRCE-DDS
# code generator it needs, and a ROS 2 workspace with the micro-ROS agent and
# ardupilot_msgs. The rig's ~/ardupilot (Copter-4.5.7, sitl/setup_sitl.sh) is
# never touched: this script refuses to build into it.
#
#   bash sitl/ap_dds/setup_ap_dds.sh            # all steps, skips finished ones
#   bash sitl/ap_dds/setup_ap_dds.sh --check    # read-only: what is built?
#
# It installs nothing system-wide. When a prerequisite is missing it prints the
# apt command for the operator and stops (docs/RUNBOOK-ap-dds.md, step 0).
#
# Environment (defaults):
#   AP_DDS_DIR=~/ardupilot-dds      the separate ArduPilot checkout
#   AP_DDS_REF=Copter-4.5.7         the rig's firmware tag, so the AP_DDS path is
#                                   the same autopilot as the MAVROS path; set
#                                   Copter-4.6 for the DDS takeoff service
#   GEN_DIR=~/Micro-XRCE-DDS-Gen    ArduPilot's fork of the code generator
#   AP_DDS_WS=~/ap_dds_ws           colcon workspace: micro_ros_agent, ardupilot_msgs
set -eo pipefail

AP_URL="${ARDUPILOT_URL:-https://github.com/ArduPilot/ardupilot.git}"
AP_DDS_DIR="${AP_DDS_DIR:-$HOME/ardupilot-dds}"
AP_DDS_REF="${AP_DDS_REF:-Copter-4.5.7}"
# The rig's pin (sitl/setup_sitl.sh ARDUPILOT_REF / ARDUPILOT_SHA);
# tests/test_ap_dds_adapter.py checks the two files agree.
RIG_REF="Copter-4.5.7"
RIG_SHA="2a3dc4b7bf2507120f7378a7b2fde73185e0c325"
GEN_DIR="${GEN_DIR:-$HOME/Micro-XRCE-DDS-Gen}"
WS="${AP_DDS_WS:-$HOME/ap_dds_ws}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

if [ "$(realpath -m "$AP_DDS_DIR")" = "$(realpath -m "$HOME/ardupilot")" ]; then
  echo "REFUSED: AP_DDS_DIR is the rig's ~/ardupilot; use a separate checkout"
  exit 64
fi

check() {
  local bin="$AP_DDS_DIR/build/sitl/bin/arducopter"
  echo "checkout : $AP_DDS_DIR ($(git -C "$AP_DDS_DIR" describe --tags --always 2>/dev/null || echo missing))"
  if [ -x "$bin" ]; then
    echo "binary   : $bin"
    echo "banner   : $(strings "$bin" | grep -m1 -E '^ArduCopter V[0-9]' || echo '?')"
    # Parameter names are stored split by group ("DDS" + "_ENABLE"), so the
    # literal DDS_ENABLE need not be in the binary. Look for the client class
    # (SITL binaries keep their symbols) or the configure-time define.
    if nm -C "$bin" 2>/dev/null | grep -q 'AP_DDS_Client'; then
      echo "DDS      : compiled in (AP_DDS_Client symbols present)"
    elif grep -qs 'AP_DDS_ENABLED 1' "$AP_DDS_DIR/build/sitl/ap_config.h"; then
      echo "DDS      : compiled in (AP_DDS_ENABLED 1 in build/sitl/ap_config.h)"
    else
      echo "DDS      : NOT compiled in (no AP_DDS_Client symbol, no AP_DDS_ENABLED 1)"
    fi
  else
    echo "binary   : missing"
  fi
  echo "gen      : $(command -v microxrceddsgen || echo "missing ($GEN_DIR/scripts not on PATH)")"
  if [ -f "$WS/install/setup.bash" ]; then
    echo "workspace: $WS/install ($(ls "$WS/install" | tr '\n' ' '))"
  else
    echo "workspace: missing"
  fi
}

if [ "${1:-}" = "--check" ]; then
  export PATH="$PATH:$GEN_DIR/scripts"
  check
  exit 0
fi

# ---- 0. prerequisites (no sudo here: print what to install) ----------------
missing=()
command -v java >/dev/null || missing+=(default-jre)
command -v git >/dev/null || missing+=(git)
[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ] || missing+=("ros-$ROS_DISTRO-ros-base")
command -v colcon >/dev/null || missing+=(python3-colcon-common-extensions)
command -v rosdep >/dev/null || missing+=(python3-rosdep)
if [ "${#missing[@]}" -gt 0 ]; then
  echo "Missing prerequisites. As the operator, run:"
  echo "  sudo apt-get update && sudo apt-get install -y ${missing[*]}"
  exit 2
fi
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"

# ---- 1. Micro-XRCE-DDS-Gen (ArduPilot's fork; ArduPilot's waf calls it) -----
if [ ! -x "$GEN_DIR/scripts/microxrceddsgen" ] || [ ! -d "$GEN_DIR/share" ]; then
  [ -d "$GEN_DIR/.git" ] || git clone --recurse-submodules \
      https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git "$GEN_DIR"
  (cd "$GEN_DIR" && ./gradlew assemble)
fi
export PATH="$PATH:$GEN_DIR/scripts"
command -v microxrceddsgen >/dev/null || { echo "microxrceddsgen not on PATH"; exit 1; }

# ---- 2. the separate ArduPilot checkout, built with DDS ---------------------
if [ ! -d "$AP_DDS_DIR/.git" ]; then
  git clone "$AP_URL" "$AP_DDS_DIR"
fi
git -C "$AP_DDS_DIR" fetch --tags --quiet origin
git -C "$AP_DDS_DIR" checkout --quiet "$AP_DDS_REF"
echo "AP_DDS checkout: $(git -C "$AP_DDS_DIR" describe --tags --always) = $(git -C "$AP_DDS_DIR" rev-parse HEAD)"
# The default ref is the rig's firmware: hold it to the same commit
# sitl/setup_sitl.sh pins (ARDUPILOT_SHA there), so "the same autopilot as the
# MAVROS path" is checked, not assumed. A moved tag stops the build.
if [ "$AP_DDS_REF" = "$RIG_REF" ]; then
  head="$(git -C "$AP_DDS_DIR" rev-parse HEAD)"
  if [ "$head" != "$RIG_SHA" ]; then
    echo "PIN FAIL: $AP_DDS_REF resolved to $head, expected $RIG_SHA (sitl/setup_sitl.sh)"
    exit 3
  fi
  echo "pin      : $AP_DDS_REF = $RIG_SHA (same commit as sitl/setup_sitl.sh)"
fi
git -C "$AP_DDS_DIR" submodule update --init --recursive --quiet
# ArduPilot's waf needs its Python modules (empy, pexpect, future); the rig's
# ~/venv-ap from sitl/setup_sitl.sh has them, so it is read, not changed.
PYBIN=python3
[ -x "$HOME/venv-ap/bin/python" ] && PYBIN="$HOME/venv-ap/bin/python"
(cd "$AP_DDS_DIR" && "$PYBIN" ./waf configure --board sitl --enable-DDS \
   && "$PYBIN" ./waf copter) 2>&1 | tail -n 25

# ---- 3. ROS 2 workspace: micro-ROS agent + ardupilot_msgs -------------------
mkdir -p "$WS/src"
[ -d "$WS/src/micro-ROS-Agent" ] || git clone -b "$ROS_DISTRO" \
    https://github.com/micro-ROS/micro-ROS-Agent.git "$WS/src/micro-ROS-Agent"
[ -d "$WS/src/micro_ros_msgs" ] || git clone -b "$ROS_DISTRO" \
    https://github.com/micro-ROS/micro_ros_msgs.git "$WS/src/micro_ros_msgs"
# ardupilot_msgs from the SAME checkout the firmware was built from, so the
# service and message definitions match what AP_DDS serves.
ln -sfn "$AP_DDS_DIR/Tools/ros2/ardupilot_msgs" "$WS/src/ardupilot_msgs"
if ! rosdep check --from-paths "$WS/src" --ignore-src -y >/dev/null 2>&1; then
  echo "rosdep reports missing system dependencies. As the operator, run:"
  echo "  rosdep install --from-paths $WS/src --ignore-src -y"
  exit 2
fi
(cd "$WS" && colcon build --symlink-install --packages-up-to micro_ros_agent ardupilot_msgs) \
  2>&1 | tail -n 15

check
