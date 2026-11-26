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
    # No `grep -q` / `grep -m1` straight on a pipe here: under pipefail the
    # writer's SIGPIPE makes the pipeline fail after a match (a false '?' and
    # a false "NOT compiled in" on 26 Nov). Collect the output first.
    local banner syms
    banner="$(strings "$bin" | grep -E '^ArduCopter V[0-9]' || true)"
    echo "banner   : ${banner%%$'
'*}"
    # Parameter names are stored split by group ("DDS" + "_ENABLE"), so the
    # literal DDS_ENABLE need not be in the binary. Look for the client class
    # (SITL binaries keep their symbols) or the configure-time define.
    syms="$(nm -C "$bin" 2>/dev/null | grep -c 'AP_DDS_Client' || true)"
    if [ "${syms:-0}" -gt 0 ]; then
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
  # The fork pins Gradle 7.6, which cannot run on Java 21 (Ubuntu 24.04's
  # default-jre: "Unsupported class file major version 65"). With Java >= 20
  # move the wrapper in this separate clone to Gradle 8.5 (the first release
  # that runs on Java 21) and replace the `classifier` property Gradle 8
  # removed. Only $GEN_DIR is changed; nothing system-wide.
  java_major="$(java -version 2>&1 | sed -nE '1s/.*version "([0-9]+).*/\1/p')"
  if [ "${java_major:-0}" -ge 20 ] \
     && grep -q 'gradle-7\.' "$GEN_DIR/gradle/wrapper/gradle-wrapper.properties"; then
    echo "Java $java_major: Micro-XRCE-DDS-Gen wrapper -> Gradle 8.5 (Gradle 7 cannot run on Java >= 20)"
    sed -i 's#gradle-7\.[0-9.]*-bin\.zip#gradle-8.5-bin.zip#' \
      "$GEN_DIR/gradle/wrapper/gradle-wrapper.properties"
    find "$GEN_DIR" -name build.gradle -exec \
      sed -i -E 's/^(\s*)classifier( *=)? */\1archiveClassifier = /' {} +
  fi
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
# waf's option is --enable-dds (lower case; --enable-DDS is refused as
# "no such option"). The log goes to a file so a failure is not hidden behind
# a pipe; the binary is checked afterwards.
build_log="$AP_DDS_DIR/build_ap_dds.log"
if ! (cd "$AP_DDS_DIR" && "$PYBIN" ./waf configure --board sitl --enable-dds \
      && "$PYBIN" ./waf copter) >"$build_log" 2>&1; then
  tail -n 25 "$build_log"
  echo "BUILD FAIL: ArduCopter SITL with DDS (full log: $build_log)"
  exit 1
fi
tail -n 5 "$build_log"
[ -x "$AP_DDS_DIR/build/sitl/bin/arducopter" ] \
  || { echo "BUILD FAIL: $AP_DDS_DIR/build/sitl/bin/arducopter missing"; exit 1; }

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
# ROS 2 Jazzy's CMake helpers import catkin_pkg from the system Python
# (/usr/bin/python3, 3.12). A user-level python3 earlier on PATH (e.g.
# ~/.local/bin/python3.11) lacks it and fails with "No module named
# 'catkin_pkg'", so the interpreter is pinned. The log goes to a file so a
# failed package stops this script.
ws_log="$WS/colcon_build_ap_dds.log"
if ! (cd "$WS" && colcon build --symlink-install \
      --packages-up-to micro_ros_agent ardupilot_msgs \
      --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
                   -DPYTHON_EXECUTABLE=/usr/bin/python3) >"$ws_log" 2>&1; then
  tail -n 25 "$ws_log"
  echo "BUILD FAIL: colcon workspace $WS (full log: $ws_log)"
  exit 1
fi
tail -n 5 "$ws_log"

check
