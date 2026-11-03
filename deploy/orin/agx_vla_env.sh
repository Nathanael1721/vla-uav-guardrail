#!/usr/bin/env bash
# The VLA and follow-controller environment on the AGX Orin 64 GB, natively on
# JetPack 7.2.1 (docs/DESIGN-topologies.md, decision 11). No published
# container carries a JetPack 7.2 Orin build of torch, so this builds a venv
# on the host's own Python 3.12 instead of the vla image (decision 10).
#
#   bash deploy/orin/agx_vla_env.sh [VENV]        # default ~/vla-drone-env
#
# What it installs, in this order, each pip call against one named index and
# with --isolated (no PIP_* variable or user config applies):
#   1. torch / torchvision from PyTorch's CUDA 13.2 index (aarch64 wheels with
#      sm_80 code, which the Orin's sm_87 runs; what NVIDIA's forum staff point
#      to for JetPack 7.2), pinned in a constraints file that every later call
#      reads, so no requirement can replace them with another build;
#   2. deploy/docker/requirements-vla.txt minus bitsandbytes: the 64 GB module
#      runs OpenVLA-7B unquantized in bf16, and PyPI's aarch64 bitsandbytes
#      has no sm_87 code (decision 10);
#   3. the follow controller's packages (demo/follow_vlm.py and the Project
#      AirSim client's imports) at the desktop vla-real env's versions, except
#      opencv-contrib-python: its 5.x needs numpy 2 and the VLA stack pins
#      numpy 1.26.4, so 4.11.0.86;
#   4. the Project AirSim client, editable and without dependencies, from
#      $PAS_CLIENT (default ~/ProjectAirSim/client/python/projectairsim: copy
#      client/python/projectairsim from the desktop's checkout, fa404d0).
# It ends by printing the versions it installed and stops unless torch sees
# the GPU. Needs internet (the AGX's Wi-Fi) and no sudo.
set -euo pipefail
VENV="${1:-$HOME/vla-drone-env}"
PAS_CLIENT="${PAS_CLIENT:-$HOME/ProjectAirSim/client/python/projectairsim}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TORCH_INDEX="https://download.pytorch.org/whl/cu132"
PYPI="https://pypi.org/simple"

[ -d "$PAS_CLIENT" ] || { echo "*** no Project AirSim client at $PAS_CLIENT (set PAS_CLIENT)"; exit 2; }
python3 -m venv "$VENV"
PIP=("$VENV/bin/pip" --isolated --disable-pip-version-check)
CONS="$VENV/constraints-agx.txt"
cat > "$CONS" <<'EOF'
torch==2.14.1
torchvision==0.29.1
numpy==1.26.4
transformers==4.40.1
EOF
"${PIP[@]}" install --index-url "$PYPI" -U pip wheel
"${PIP[@]}" install --index-url "$TORCH_INDEX" -c "$CONS" torch torchvision
grep -vE '^[[:space:]]*(#|$)|^bitsandbytes' "$ROOT/deploy/docker/requirements-vla.txt" \
    > "$VENV/requirements-vla-agx.txt"
"${PIP[@]}" install --index-url "$PYPI" -c "$CONS" -r "$VENV/requirements-vla-agx.txt"
"${PIP[@]}" install --index-url "$PYPI" -c "$CONS" \
    pynng==0.9.0 "msgpack>=1.0.5" opencv-contrib-python==4.11.0.86 matplotlib==3.10.9 \
    commentjson==0.9.0 jsonschema==4.26.0 inputs==0.5 pqdict==1.5.1 cryptography==49.0.0 \
    pykml==0.2.0 junit-xml==1.9 jsonlines==4.0.0 shapely==2.1.2 pydantic==2.13.4 \
    PyYAML==6.0.3 Jinja2==3.1.6 scipy==1.15.3
"${PIP[@]}" install --no-deps -e "$PAS_CLIENT"
"$VENV/bin/python" - <<'EOF'
import cv2, numpy, torch, transformers
import projectairsim  # noqa: F401
print(f"torch {torch.__version__} (CUDA {torch.version.cuda}), torchvision "
      f"{__import__('torchvision').__version__}, transformers {transformers.__version__}, "
      f"numpy {numpy.__version__}, opencv {cv2.__version__}")
if not torch.cuda.is_available():
    raise SystemExit("*** torch sees no GPU: run deploy/orin/gpu_check.py --skip-bnb for the reason")
print(f"GPU: {torch.cuda.get_device_name(0)}, capability {torch.cuda.get_device_capability(0)}")
EOF
echo "==> $VENV ready; next: python deploy/orin/gpu_check.py --skip-bnb, then deploy/orin/vla_bf16_check.py"
