#!/usr/bin/env bash
# First-boot check of the Jetson Orin for the hil topology, on JetPack 6.2
# (L4T r36.4.x, Ubuntu 22.04). docs/RUNBOOK-orin-hil.md, step 1.
#
#   mkdir -p deploy/evidence/incoming/hil
#   bash deploy/orin/first_boot_check.sh 2>&1 | tee deploy/evidence/incoming/hil/first_boot_check.txt
#
# Send the whole output back. It prints what the person setting up the Orin
# would otherwise paste one command at a time (the L4T release, the board,
# JetPack and its CUDA / cuDNN / TensorRT, the power mode and the modes this
# module offers, ROS on the host, Docker and its runtimes, memory, disks,
# network, time), then a VERDICTS block with one PASS, WARN or FAIL line per
# expectation of the deployment (deploy/, docs/DESIGN-topologies.md). It exits
# 1 when any line is FAIL.
#
# Read-only: it installs nothing, downloads nothing, changes no setting and
# asks for no password. The one reading that needs root (jetson_clocks
# --show) uses `sudo -n`; run `sudo -v` first to include it.
#
# VLAGUARD_SYSROOT (default empty) is put in front of every file it reads, so
# tests can describe an Orin without being one.
set -o pipefail

SYS="${VLAGUARD_SYSROOT:-}"
MIN_FREE_GB=40
FAILS=0
WARNS=0
VERDICTS=()

pass() { VERDICTS+=("PASS  $1: $2"); }
warn() { VERDICTS+=("WARN  $1: $2"); WARNS=$((WARNS + 1)); }
fail() { VERDICTS+=("FAIL  $1: $2"); FAILS=$((FAILS + 1)); }
have() { command -v "$1" >/dev/null 2>&1; }

section() { printf '\n----- %s\n' "$1"; }

show() {                                   # show FILE: its contents, or why not
  if [ -r "$SYS$1" ]; then
    printf '$ cat %s\n' "$1"
    tr -d '\000' < "$SYS$1"
    echo
  else
    echo "(cannot read $1)"
  fi
}

cmd() {                                    # cmd ARGS...: the command and its output
  printf '$ %s\n' "$*"
  if have "$1"; then
    "$@" 2>&1
    local rc=$?
    [ "$rc" -eq 0 ] || echo "(exit $rc)"
  else
    echo "(command not found: $1)"
  fi
}

dpkg_version() {                           # the installed version of PACKAGE, or empty
  have dpkg-query || return 0
  dpkg-query -W -f='${Version}' "$1" 2>/dev/null || true
}

echo "===== vlaguard first-boot check (target: JetPack 6.2, L4T r36.4.x) ====="
echo "date_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
rev="$(git -C "$(dirname "$0")/../.." rev-parse --short HEAD 2>/dev/null)" || rev=""
echo "script_commit: ${rev:-unknown}"

# --------------------------------------------------------------------------- #
section "L4T release"
show /etc/nv_tegra_release
rel="$(tr -d '\000' < "$SYS/etc/nv_tegra_release" 2>/dev/null)"
if [[ "$rel" =~ R([0-9]+)\ \(release\),\ REVISION:\ ([0-9]+)\.([0-9]+) ]]; then
  L4T_MAJOR="${BASH_REMATCH[1]}"; L4T_MINOR="${BASH_REMATCH[2]}"; L4T_PATCH="${BASH_REMATCH[3]}"
  L4T="R$L4T_MAJOR.$L4T_MINOR.$L4T_PATCH"
  if [ "$L4T_MAJOR" = "36" ] && [ "$L4T_MINOR" = "4" ]; then
    case "$L4T_PATCH" in
      3) pass l4t "$L4T = JetPack 6.2, the release deploy/ is written for" ;;
      4) pass l4t "$L4T = JetPack 6.2.1 (the same CUDA 12.6 / cuDNN 9.3 / TensorRT 10.3 as 6.2, plus the Docker 28 fix)" ;;
      0) warn l4t "$L4T = JetPack 6.1, not the planned 6.2: the same r36.4 line and CUDA 12.6, so the images still fit; say so when you send this back" ;;
      *) pass l4t "$L4T: the r36.4 line the Orin images are built for" ;;
    esac
  elif [ "$L4T_MAJOR" = "36" ] && [ "$L4T_MINOR" -ge 5 ]; then
    warn l4t "$L4T is newer than planned (R36.5 = JetPack 6.2.2 / 6.2.3, same CUDA 12.6 / TensorRT 10.3 per NVIDIA); the r36.4.0 base images were built for r36.4: deploy/orin/gpu_check.py decides whether they run"
  elif [ "$L4T_MAJOR" = "36" ]; then
    fail l4t "$L4T is JetPack 6.0 or older; flash JetPack 6.2 (L4T r36.4.3)"
  else
    fail l4t "$L4T is not JetPack 6 (L4T R36); flash JetPack 6.2"
  fi
else
  L4T=""
  fail l4t "no readable /etc/nv_tegra_release: this is not a Jetson running L4T"
fi

section "Board"
show /proc/device-tree/model
model="$(tr -d '\000' < "$SYS/proc/device-tree/model" 2>/dev/null)"
if [[ "$model" == *Orin* ]]; then
  pass board "$model"
else
  fail board "the device tree names '${model:-nothing}', not a Jetson Orin; guardrail.manifest would label every run on it dev"
fi

section "Kernel and architecture"
cmd uname -srvmo
arch="$(uname -m 2>/dev/null)"
if [ "$arch" = "aarch64" ]; then
  pass arch "aarch64"
else
  fail arch "'$arch', not aarch64"
fi
kver="$(uname -r 2>/dev/null)"

section "Ubuntu"
show /etc/os-release
os_id="$(. "$SYS/etc/os-release" 2>/dev/null && echo "${ID:-}-${VERSION_ID:-}")"
if [ "$os_id" = "ubuntu-22.04" ]; then
  pass os "Ubuntu 22.04, as JetPack 6.2 ships it"
else
  warn os "'${os_id:-unknown}': JetPack 6.2 ships Ubuntu 22.04"
fi

# --------------------------------------------------------------------------- #
section "JetPack and its compute stack on the host"
cmd dpkg-query -W nvidia-jetpack nvidia-l4t-core cuda-cudart-12-6 libcudnn9-cuda-12 tensorrt \
    nvidia-container-toolkit
show /usr/local/cuda/version.json
jp="$(dpkg_version nvidia-jetpack)"
case "$jp" in
  6.2*) pass jetpack "nvidia-jetpack $jp" ;;
  "")   warn jetpack "the nvidia-jetpack meta-package is not installed, so the host has no CUDA / cuDNN / TensorRT packages; the images bring their own and need only the driver, so nothing in deploy/ needs them (install with: sudo apt install nvidia-jetpack)" ;;
  *)    warn jetpack "nvidia-jetpack $jp, not 6.2" ;;
esac
cudart="$(dpkg_version cuda-cudart-12-6)"
trt="$(dpkg_version tensorrt)"
cudnn="$(dpkg_version libcudnn9-cuda-12)"
if [ -n "$jp" ]; then
  if [ -n "$cudart" ] && [[ "$trt" == 10.3* ]] && [[ "$cudnn" == 9.3* ]]; then
    pass compute "CUDA 12.6 ($cudart), cuDNN $cudnn, TensorRT $trt"
  else
    warn compute "expected JetPack 6.2's CUDA 12.6 / cuDNN 9.3 / TensorRT 10.3; found cudart-12-6 '${cudart:-none}', cuDNN '${cudnn:-none}', TensorRT '${trt:-none}'"
  fi
fi

# --------------------------------------------------------------------------- #
section "Power mode"
cmd nvpmodel -q
if [ -r "$SYS/etc/nvpmodel.conf" ]; then
  echo "\$ grep -E 'POWER_MODEL|PM_CONFIG' /etc/nvpmodel.conf"
  grep -E '< *POWER_MODEL|< *PM_CONFIG' "$SYS/etc/nvpmodel.conf"
else
  echo "(cannot read /etc/nvpmodel.conf)"
fi
show /sys/devices/system/cpu/online
printf '$ sudo -n jetson_clocks --show\n'
if have sudo && have jetson_clocks; then
  sudo -n jetson_clocks --show 2>&1 || echo "(needs root: run 'sudo -v' first, then this script again)"
else
  echo "(sudo or jetson_clocks not available)"
fi
# This module's mode table. IDs differ between Orin modules (Jetson Linux
# 36.4.3 power guide: AGX Orin MAXN = 0 and default 30W = 2; an Orin NX
# flashed for JetPack 6.2's Super modes has MAXN_SUPER = 0 and default 40W = 4;
# an Orin Nano Super has MAXN_SUPER = 2; a Nano without Super has no MAXN), so
# runbook step 4 sets the timing modes by the IDs read here.
modes=""; def_id=""; def_name=""; maxn_id=""; maxn_name=""; super=""
re_model='<[[:space:]]*POWER_MODEL[[:space:]]+ID=([0-9]+)[[:space:]]+NAME=([^[:space:]>]+)'
re_default='<[[:space:]]*PM_CONFIG[[:space:]]+DEFAULT=([0-9]+)'
if [ -r "$SYS/etc/nvpmodel.conf" ]; then
  while IFS= read -r line || [ -n "$line" ]; do
    if [[ "$line" =~ $re_model ]]; then
      modes+="${BASH_REMATCH[1]}=${BASH_REMATCH[2]} "
      if [ -z "$maxn_id" ] && [[ "${BASH_REMATCH[2]}" == MAXN* ]]; then
        maxn_id="${BASH_REMATCH[1]}"; maxn_name="${BASH_REMATCH[2]}"
      fi
      [[ "${BASH_REMATCH[2]}" == *SUPER* ]] && super="yes"
    elif [[ "$line" =~ $re_default ]]; then
      def_id="${BASH_REMATCH[1]}"
    fi
  done < "$SYS/etc/nvpmodel.conf"
fi
# The highest mode WITH a power budget: the largest wattage in a mode's name
# (MODE_50W, 40W, 25W, 7W_AI ...). Timing runs use it rather than MAXN, which
# NVIDIA's guide calls "an unconstrained power mode" and "an experimental
# mode", throttled by hardware once the module exceeds its TDP.
top_id=""; top_name=""; top_w=-1
re_watts='([0-9]+)W'
for m in $modes; do
  [ "${m%%=*}" = "$def_id" ] && def_name="${m#*=}"
  if [[ "${m#*=}" != MAXN* ]] && [[ "${m#*=}" =~ $re_watts ]] \
     && [ "${BASH_REMATCH[1]}" -gt "$top_w" ]; then
    top_w="${BASH_REMATCH[1]}"; top_id="${m%%=*}"; top_name="${m#*=}"
  fi
done
if have nvpmodel; then
  mode="$(nvpmodel -q 2>/dev/null | grep -i 'power mode' | head -n 1)"
  mode="${mode:-nvpmodel answered, mode line not found}"
  if [ -z "$modes" ] || [ -z "$def_name" ]; then
    warn power "current '$mode', but no mode table with a default in /etc/nvpmodel.conf: the mode IDs for runbook step 4 are unknown; send the file's POWER_MODEL and PM_CONFIG lines"
  else
    if [ -n "$maxn_id" ]; then
      ceiling="MAXN mode ID $maxn_id = $maxn_name"
    else
      ceiling="no MAXN mode on this module: the highest-power mode is its ceiling"
    fi
    note=""
    if [ -n "$super" ]; then
      note="; JetPack 6.2's Super modes are flashed"
    elif [[ "$model" == *"Orin NX"* || "$model" == *"Orin Nano"* ]]; then
      note="; flashed without JetPack 6.2's Super modes (a -super flash configuration adds MAXN_SUPER)"
    fi
    if [ -n "$top_id" ]; then
      budget="highest budgeted mode ID $top_id = $top_name"
    else
      budget="no mode with a power budget in its name"
    fi
    pass power "current '$mode'; modes ${modes% }; default ID $def_id = $def_name; ${budget}; ${ceiling}${note}; timing runs set the mode by ID and lock the clocks (runbook step 4)"
  fi
else
  fail power "nvpmodel not found: not a JetPack image"
fi

# --------------------------------------------------------------------------- #
section "ROS 2 on the host"
printf '$ ls /opt/ros\n'
ros_host=""
if [ -d "$SYS/opt/ros" ]; then
  ros_host="$(ls -1 "$SYS/opt/ros" 2>/dev/null | tr '\n' ' ')"
fi
echo "${ros_host:-(none)}"
echo "ROS_DISTRO=${ROS_DISTRO:-}"
# The version of each distribution found: its rclpy package (what a node on
# the host would run), its ros-base and MAVROS if present.
ros_versions=""
for d in $ros_host; do
  cmd dpkg-query -W "ros-$d-rclpy" "ros-$d-ros-base" "ros-$d-mavros"
  v="$(dpkg_version "ros-$d-rclpy")"
  ros_versions+="$d (rclpy ${v:-not installed}), "
done
if [ -z "${ros_host// /}" ]; then
  pass ros_host "no ROS 2 on the host: ROS 2 Jazzy runs inside the companion image (docs/DESIGN-topologies.md, decision 2)"
else
  warn ros_host "ROS 2 on the host: ${ros_versions%, }. Harmless unless a host node joins ROS_DOMAIN_ID 42; the rail never uses it (ROS 2 Jazzy runs inside the companion image)"
fi

# --------------------------------------------------------------------------- #
section "Docker"
cmd docker info
cmd docker compose version
cmd docker buildx version
cmd nvidia-ctk --version
if ! have docker; then
  fail docker "Docker is not installed (JetPack 6.2 flashed from SDK Manager has none; runbook step 2)"
else
  dinfo() { docker info --format "$1" 2>/dev/null; }
  server="$(dinfo '{{.ServerVersion}}')"
  if [ -z "$server" ]; then
    fail docker "the Docker daemon does not answer this user (is it running, and is the user in the docker group?)"
  else
    major="${server%%.*}"
    if [[ "$major" =~ ^[0-9]+$ ]] && [ "$major" -ge 23 ]; then
      pass docker "Docker $server"
    else
      warn docker "Docker $server: BuildKit is the default builder from Docker 23; older needs DOCKER_BUILDKIT=1"
    fi
    if [[ "$major" =~ ^[0-9]+$ ]] && [ "$major" -ge 28 ] \
       && [ "${L4T_MAJOR:-}" = "36" ] && [ "${L4T_MINOR:-}" = "4" ] && [ "${L4T_PATCH:-9}" -lt 4 ]; then
      raw="no"
      if have modinfo && modinfo iptable_raw >/dev/null 2>&1; then raw="yes"
      elif grep -qs 'iptable_raw' "$SYS/lib/modules/$kver/modules.builtin"; then raw="yes"
      elif grep -qsx 'raw' "$SYS/proc/net/ip_tables_names"; then raw="yes"
      fi
      if [ "$raw" = "no" ]; then
        warn docker28 "Docker $server on $L4T, whose kernel has no iptable_raw: published ports fail. The Orin's compose files publish none, but pin Docker 27.5.1 or move to JetPack 6.2.1 (runbook step 2)"
      fi
    fi
    runtimes="$(dinfo '{{range $k, $v := .Runtimes}}{{$k}} {{end}}')"
    if [[ " $runtimes " == *" nvidia "* ]]; then
      pass nvidia_runtime "runtimes: ${runtimes% }; default: $(dinfo '{{.DefaultRuntime}}') (the companion services name runc themselves)"
    else
      fail nvidia_runtime "no nvidia runtime (runtimes: ${runtimes:-none}): sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker"
    fi
    if docker compose version >/dev/null 2>&1; then
      pass compose "$(docker compose version --short 2>/dev/null || echo present)"
    else
      fail compose "the docker compose plugin is missing (docker-compose-plugin)"
    fi
    if docker buildx version >/dev/null 2>&1; then
      pass buildx "$(docker buildx version 2>/dev/null | head -n 1)"
    else
      fail buildx "the buildx plugin is missing: docker build would fall back to the legacy builder, which ignores the per-Dockerfile .dockerignore files and would send the whole checkout, policies/keys/ included, as build context"
    fi
    root="$(dinfo '{{.DockerRootDir}}')"
    avail_kb="$(df -Pk "${root:-/var/lib/docker}" 2>/dev/null | awk 'NR == 2 {print $4}')"
    if [[ "$avail_kb" =~ ^[0-9]+$ ]]; then
      gb=$((avail_kb / 1024 / 1024))
      if [ "$gb" -ge "$MIN_FREE_GB" ]; then
        pass disk "${gb} GB free under ${root:-/var/lib/docker}"
      else
        warn disk "${gb} GB free under ${root:-/var/lib/docker}; the images and OpenVLA-7B's weights need about ${MIN_FREE_GB} GB (an NVMe drive is preferred)"
      fi
    else
      warn disk "could not read the free space under ${root:-/var/lib/docker}"
    fi
  fi
fi

# --------------------------------------------------------------------------- #
section "Memory, swap and disks"
cmd free -h
cmd swapon --show
cmd df -h /
cmd lsblk -d -o NAME,SIZE,MODEL,TRAN

section "Network"
cmd ip -br addr

section "Time"
cmd timedatectl
cmd chronyc tracking

section "git"
cmd git --version

# --------------------------------------------------------------------------- #
echo
echo "===== VERDICTS ====="
for v in "${VERDICTS[@]}"; do
  echo "$v"
done
echo "===== ${FAILS} FAIL, ${WARNS} WARN, $(( ${#VERDICTS[@]} - FAILS - WARNS )) PASS ====="
[ "$FAILS" -eq 0 ]
