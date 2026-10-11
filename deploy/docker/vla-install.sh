#!/usr/bin/env bash
# Installs the VLA runtime's Python packages into the vla image
# (deploy/docker/vla.Dockerfile), on top of a base image that already carries
# PyTorch built for its GPU: CUDA 12.4 on the desktop, a Jetson build for
# L4T r36.4 (JetPack 6.2) on the Orin.
#
# Three things a plain `pip install -r` gets wrong on the Orin, each of which
# builds an image that imports fine and fails later:
#
#   1. The torch stack belongs to the base image. PyPI's aarch64 torch and
#      torchvision wheels are not built for the Jetson GPU; a requirement that
#      pulled one in would replace the Jetson build. The installed torch,
#      torchvision and torchaudio versions are written into a constraints file
#      first, so pip either keeps them or stops with a conflict, and they are
#      compared again at the end.
#   2. Base images for Jetson can set their own package index: both Orin
#      bases set PIP_INDEX_URL in their environment, and a pip.conf file could
#      set one too. Every pip call here runs with --isolated (no PIP_*
#      environment variables, no per-user config file), with
#      PIP_CONFIG_FILE=/dev/null (pip then skips every config file, the global
#      /etc/pip.conf and the one inside a virtual environment included, which
#      --isolated alone still reads) and with an explicit --index-url for
#      PyPI. Before installing, the build log shows `pip config debug` (which
#      config files the base carries) and the build stops if pip, run with
#      those settings, still reports any setting (`pip config list`).
#   3. bitsandbytes (4-bit NF4, how OpenVLA-7B is loaded) publishes aarch64
#      wheels with no sm_87 code (its build script targets sm_75, sm_80 and
#      sm_90 on aarch64, plus sm_100 and sm_120 with CUDA 12.8 or 12.9; an
#      sm_80 cubin does not run on a Tegra GPU). A Jetson Orin is sm_87: the
#      wheel installs and imports, and the first 4-bit kernel fails with
#      "named symbol not found" (bitsandbytes issue #1930, JetPack 6.2). On
#      aarch64 the pinned version is therefore built from its source tag for
#      COMPUTE_CAPABILITY=87, and the build stops unless the library it
#      installed carries sm_87 code (cuobjdump).
#      The build follows bitsandbytes' own install guide: CMake configures and
#      compiles the CUDA library into the source tree FIRST, then pip packages
#      the tree. pip alone would not compile it: the source's pyproject.toml
#      sets `wheel.cmake = false` under [tool.scikit-build], so CMake options
#      handed to pip (CMAKE_ARGS) are never used and no CUDA library is built.
#      The source needs CMake >= 3.22.1. Both Orin bases carry a pip-installed
#      CMake ahead of the apt one on PATH (their Docker Hub layer history runs
#      `pip3 install --upgrade ... cmake`); either meets the floor (the apt
#      CMake is 3.28 on Ubuntu 24.04, 3.22.1 on the 22.04 fallback). apt
#      installs one in case a base has none, and the build log records
#      `cmake --version` just before the configure step, so it shows which
#      CMake ran.
#
# The pins stay in requirements-vla.txt (bitsandbytes too): the Orin builds
# the same version the desktop installs from PyPI.
#
#   vla-install.sh [REQ_DIR]        REQ_DIR defaults to /tmp/req
#   vla-install.sh --check-sm 87    only the check: does the installed
#                                   bitsandbytes carry sm_87 code?
#
# VLA_PYTHON (default python3) is the interpreter; VLA_ARCH (default uname -m)
# picks the path; VLA_BUILD_JOBS (default 4) caps the parallel nvcc jobs of
# the bitsandbytes build, so it fits a smaller Orin's memory.
# VLAGUARD_DRY_RUN=1 writes and prints the constraints, then
# prints each command (PLAN: lines, shell-quoted) instead of running it; the
# tests use it.
#
# Exit codes: 64 bad input, 65 a check failed.
set -eo pipefail

REQ_DIR="${1:-/tmp/req}"
PY="${VLA_PYTHON:-python3}"
ARCH="${VLA_ARCH:-$(uname -m)}"
DRY="${VLAGUARD_DRY_RUN:-0}"
INDEX="https://pypi.org/simple"
BNB_REPO="https://github.com/bitsandbytes-foundation/bitsandbytes.git"
BNB_SRC="${VLA_BNB_SRC:-/tmp/bitsandbytes}"
JETSON_SM="87"

die() { echo "vla-install: $2" >&2; exit "$1"; }

step() {
  if [ "$DRY" = "1" ]; then
    printf 'PLAN:'
    printf ' %q' "$@"
    printf '\n'
    return 0
  fi
  "$@"
}

freeze_torch() {
  "$PY" - <<'EOF'
import importlib.metadata as m
for dist in ("torch", "torchvision", "torchaudio"):
    try:
        print(f"{dist}=={m.version(dist)}")
    except m.PackageNotFoundError:
        pass
EOF
}

# The installed bitsandbytes must carry code for this GPU architecture.
check_sm() {
  local sm="$1" dir elfs lib
  local libs=()
  dir="$("$PY" -c 'import importlib.util as u; s = u.find_spec("bitsandbytes"); print(s.submodule_search_locations[0])')" \
    || die 65 "bitsandbytes is not installed"
  command -v cuobjdump >/dev/null 2>&1 \
    || die 65 "cuobjdump not found (no CUDA toolkit in the base image): cannot show sm_$sm code"
  shopt -s nullglob
  libs=("$dir"/libbitsandbytes_cuda*.so)
  shopt -u nullglob
  [ "${#libs[@]}" -gt 0 ] || die 65 "no libbitsandbytes_cuda*.so in $dir"
  for lib in "${libs[@]}"; do
    elfs="$(cuobjdump --list-elf "$lib")" || die 65 "cuobjdump could not read $lib"
    grep -q "sm_$sm" <<<"$elfs" \
      || die 65 "$lib carries no sm_$sm code: every 4-bit kernel would fail on the Orin"
    echo "vla-install: $lib carries sm_$sm" >&2
  done
}

# pip reads no package index or other setting from the base image: show the
# config files the base carries, then stop if pip, with the settings every
# install below uses, still reports any setting.
check_pip_config() {
  local left
  "$PY" -m pip config debug >&2 || echo "vla-install: (pip config debug failed)" >&2
  left="$("${PIP_NO_CONFIG[@]}" config list)" || die 65 "pip config list failed"
  [ -z "$left" ] \
    || die 65 "pip still reads settings with PIP_CONFIG_FILE=/dev/null --isolated: $left"
  echo "vla-install: pip reads no config file and no PIP_* variable" >&2
}

if [ "${1:-}" = "--check-sm" ]; then
  [[ "${2:-}" =~ ^[0-9]+$ ]] || die 64 "--check-sm needs a compute capability such as 87"
  check_sm "$2"
  exit 0
fi

[ -f "$REQ_DIR/requirements.txt" ] || die 64 "no $REQ_DIR/requirements.txt"
[ -f "$REQ_DIR/requirements-vla.txt" ] || die 64 "no $REQ_DIR/requirements-vla.txt"

# 1. The base image's torch stack, as constraints.
freeze_torch > "$REQ_DIR/base-torch.txt"
grep -q '^torch==' "$REQ_DIR/base-torch.txt" \
  || die 64 "the base image has no torch: use a PyTorch base (deploy/docker/vla.Dockerfile)"
echo "vla-install: keeping the base image's $(tr '\n' ' ' < "$REQ_DIR/base-torch.txt")" >&2

PIP_NO_CONFIG=(env PIP_CONFIG_FILE=/dev/null "$PY" -m pip --isolated)
PIP=("${PIP_NO_CONFIG[@]}" install --no-cache-dir --index-url "$INDEX"
     -c "$REQ_DIR/base-torch.txt")

BNB="$(sed -n 's/^bitsandbytes==\([0-9][0-9.]*\)[[:space:]]*$/\1/p' "$REQ_DIR/requirements-vla.txt")"
[ -n "$BNB" ] || die 64 "requirements-vla.txt pins no bitsandbytes==X.Y.Z"

# 2. No package index or other pip setting from the base image.
step check_pip_config

case "$ARCH" in
  x86_64|amd64)
    step "${PIP[@]}" -r "$REQ_DIR/requirements.txt" -r "$REQ_DIR/requirements-vla.txt"
    ;;
  aarch64|arm64)
    grep -v '^bitsandbytes==' "$REQ_DIR/requirements-vla.txt" > "$REQ_DIR/requirements-vla-jetson.txt"
    step "${PIP[@]}" -r "$REQ_DIR/requirements.txt" -r "$REQ_DIR/requirements-vla-jetson.txt"
    # 3. bitsandbytes for the Orin's GPU, at the desktop's version: configure
    # and compile the CUDA library with CMake (it lands in
    # $BNB_SRC/bitsandbytes/libbitsandbytes_cuda<ver>.so), then let pip
    # package the tree. nvcc comes from the base image's CUDA toolkit.
    step apt-get update
    step apt-get install -y --no-install-recommends cmake build-essential
    step git clone --depth 1 --branch "$BNB" "$BNB_REPO" "$BNB_SRC"
    step cmake --version            # which CMake runs: the base's pip one or apt's
    step cmake -S "$BNB_SRC" -B "$BNB_SRC/build" \
         -DCOMPUTE_BACKEND=cuda "-DCOMPUTE_CAPABILITY=$JETSON_SM" \
         "-DCMAKE_CUDA_COMPILER=${CUDACXX:-/usr/local/cuda/bin/nvcc}" \
         -DCMAKE_BUILD_TYPE=Release
    step cmake --build "$BNB_SRC/build" --config Release --parallel "${VLA_BUILD_JOBS:-4}"
    step "${PIP[@]}" --no-deps "$BNB_SRC"
    step check_sm "$JETSON_SM"
    step rm -rf "$BNB_SRC"
    ;;
  *)
    die 64 "unknown architecture '$ARCH' (x86_64 or aarch64)"
    ;;
esac

# 1 (again). Nothing replaced the base image's torch stack.
if [ "$DRY" != "1" ]; then
  diff -u "$REQ_DIR/base-torch.txt" <(freeze_torch) >&2 \
    || die 65 "the install changed the base image's torch stack (above): a requirement pulled in another build"
fi
exit 0
