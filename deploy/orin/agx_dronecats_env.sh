#!/usr/bin/env bash
# DroneCATS' environment on the AGX Orin 64 GB, natively on JetPack 7.2.1, beside
# the VLA environment (deploy/orin/agx_vla_env.sh). DroneCATS-Cosmos3-Edge-2B
# needs transformers >= 5.18 (demo/vla_server_foreign.py), and the VLA stack
# pins transformers 4.40.1, so the two cannot share one venv - as on the desktop,
# where DroneCATS has its own conda env (dronecats).
#
#   bash deploy/orin/agx_dronecats_env.sh [VENV]   # default ~/vla-dronecats-env
#
#   1. torch / torchvision from PyTorch's CUDA 13.2 index, the same build as the
#      VLA environment, pinned in a constraints file every later call reads;
#   2. demo/vla_server_foreign.py's packages at the desktop dronecats env's
#      versions (transformers 5.21.0, tokenizers 0.23.3, accelerate 1.15.0,
#      huggingface_hub 2.3.0, safetensors 0.8.0, numpy 2.4.6, pillow 12.3.0).
# The 64 GB module serves the 2B model in bf16, the model card's precision, so
# neither torchao nor bitsandbytes is installed (the Orin Nano needed torchao's
# int4). It ends by printing the versions and stops unless torch sees the GPU.
# Needs internet (the AGX's Wi-Fi) and no sudo.
set -euo pipefail
VENV="${1:-$HOME/vla-dronecats-env}"
TORCH_INDEX="https://download.pytorch.org/whl/cu132"
PYPI="https://pypi.org/simple"

python3 -m venv "$VENV"
PIP=("$VENV/bin/pip" --isolated --disable-pip-version-check)
CONS="$VENV/constraints-agx.txt"
cat > "$CONS" <<'EOF'
torch==2.14.1
torchvision==0.29.1
numpy==2.4.6
transformers==5.21.0
EOF
"${PIP[@]}" install --index-url "$PYPI" -U pip wheel
"${PIP[@]}" install --index-url "$TORCH_INDEX" -c "$CONS" torch torchvision
"${PIP[@]}" install --index-url "$PYPI" -c "$CONS" \
    transformers==5.21.0 tokenizers==0.23.3 accelerate==1.15.0 huggingface_hub==2.3.0 \
    safetensors==0.8.0 numpy==2.4.6 pillow==12.3.0 Jinja2==3.1.6 PyYAML==6.0.3 \
    msgpack==1.2.4 pynng==0.9.0 psutil==7.2.2
"$VENV/bin/python" - <<'EOF'
import numpy, torch, transformers
print(f"torch {torch.__version__} (CUDA {torch.version.cuda}), torchvision "
      f"{__import__('torchvision').__version__}, transformers {transformers.__version__}, "
      f"numpy {numpy.__version__}")
if not torch.cuda.is_available():
    raise SystemExit("*** torch sees no GPU: run deploy/orin/gpu_check.py --skip-bnb for the reason")
print(f"GPU: {torch.cuda.get_device_name(0)}, capability {torch.cuda.get_device_capability(0)}")
EOF
echo "==> $VENV ready; serve with: $VENV/bin/python demo/vla_server_foreign.py serve --model dronecats ..."
