# syntax=docker/dockerfile:1
#
# The VLA backend image: the GPU runtime for OpenVLA-7B and the AerialVLA
# adapters, kept apart from the companion image on purpose.
#
#   * The safety path must not depend on the CUDA / PyTorch stack: a model that
#     runs out of memory or crashes must not take the Shield or MAVROS with it.
#   * The docstring of demo/vla_server.py records a desktop measurement (no
#     timing file was kept): the same model took about 2.5 s per action in its
#     own process and about 12 s inside the 10 Hz flight process. The VLA
#     belongs in a process of its own.
#   * GPU user space has to suit the host's driver stack. On the Orin that is
#     L4T r36.4 (JetPack 6.2); on the desktop, CUDA 12.4 as in the vla-real
#     environment. The companion image needs neither, which is what lets it be
#     one multi-arch definition.
#
# So this definition takes its base as an argument, and hil and flight use the
# same arm64 build of it (deploy/topologies/hil.env = flight.env):
#
#   desktop: docker build -f deploy/docker/vla.Dockerfile \
#              --build-arg BASE_IMAGE=pytorch/pytorch:2.6.0-cuda12.4-cudnn9-runtime \
#              -t vlaguard/vla:desktop-cu124 .
#   Orin:    docker build -f deploy/docker/vla.Dockerfile \
#              --build-arg BASE_IMAGE=dustynv/torchvision:0.21.0-r36.4.0-cu128-24.04 \
#              -t vlaguard/vla:l4t-r36.4-cu128 .
#
# THE ORIN BASE (JetPack 6.2: L4T r36.4.3, CUDA 12.6, cuDNN 9.3, TensorRT 10.3,
# Ubuntu 22.04). Published container tags exist per L4T minor release, and the
# newest for JetPack 6 is r36.4.0, built for the r36.4 line that JetPack 6.1,
# 6.2 and 6.2.1 share. dustynv/torchvision:0.21.0-r36.4.0-cu128-24.04
# (jetson-containers; checked on Docker Hub 2026-10-08) carries torch 2.6 and
# torchvision 0.21.0, the pair the desktop's vla-real flies with, on Ubuntu
# 24.04 with Python 3.12 (the grant's 3.11+ for the guardrail code this image
# runs) and CUDA 12.8 with its forward-compatibility libraries
# (/usr/local/cuda/compat), plus the CUDA toolkit the bitsandbytes build needs.
# Fallback, if that image fails the GPU check (deploy/orin/gpu_check.py):
# dustynv/l4t-pytorch:r36.4.0 (Ubuntu 22.04, Python 3.10, CUDA 12.6 as on
# JetPack 6.2, torch 2.4.0 + torchvision 0.19.0), which puts the guardrail
# code in this image on Python 3.10 and the Orin's R1 rates on another torch
# than the desktop's: deviations to record. No r36.4 transformers image is published
# (dustynv/transformers stops at r36.2.0), so transformers and the other pins
# are installed here on top of the PyTorch base. TensorRT is on the host but
# unused: OpenVLA runs in PyTorch with bitsandbytes 4-bit.
# docs/DESIGN-topologies.md ("JetPack 6.2") has the reasoning and the list of
# tags that were checked.
#
# Written on 2026-10-06, revised for JetPack 6.2 on 2026-10-08; NOT yet built.
# On aarch64 deploy/docker/vla-install.sh builds bitsandbytes from source for
# the Orin's sm_87 (PyPI's aarch64 wheels have no sm_87 code) and refuses to
# finish if the result does not carry it; it also keeps pip from replacing the
# base image's Jetson torch build, and from reading a package index the base
# sets (PIP_INDEX_URL, or a pip.conf: every pip call runs --isolated with
# PIP_CONFIG_FILE=/dev/null and PyPI named explicitly). The previous version
# of this file (commit 10451bc) ran a plain `pip install -r` on the Orin too,
# which installs PyPI's aarch64 bitsandbytes: it imports, then fails at its
# first 4-bit kernel.
#
# The pins are vla-real's, the only stack on record that loads OpenVLA-7B in
# 4-bit (docs/DESIGN-python-versions.md); torch and torchvision come from the
# base image. Weights are mounted at /models, the repository at /opt/vlaguard.
# Every R1 rate on a host is measured in this image, the CPU backends (stub,
# bc_v3) included, so the companion image stays one build for hil and flight.

ARG BASE_IMAGE=pytorch/pytorch:2.6.0-cuda12.4-cudnn9-runtime
FROM ${BASE_IMAGE}
# 1 adds the Project AirSim client (needed once a camera VLA flies from the
# Orin and reads frames from the desktop's simulator; not for the R1 bench).
ARG WITH_PROJECTAIRSIM=0
ARG PROJECTAIRSIM_REF=fa404d0e9aa72955823708ec146e7871a7c7d217
ENV DEBIAN_FRONTEND=noninteractive PIP_NO_CACHE_DIR=1
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update \
 && apt-get install -y --no-install-recommends git ca-certificates libgl1 libglib2.0-0 \
 && rm -rf /var/lib/apt/lists/*
COPY requirements.txt deploy/docker/requirements-vla.txt deploy/docker/vla-install.sh /tmp/req/
RUN bash /tmp/req/vla-install.sh /tmp/req \
 && if [ "${WITH_PROJECTAIRSIM}" = "1" ]; then \
      PIP_CONFIG_FILE=/dev/null python3 -m pip --isolated install --no-cache-dir \
        --index-url https://pypi.org/simple \
        -c /tmp/req/base-torch.txt \
        "projectairsim @ git+https://github.com/iamaisim/ProjectAirSim.git@${PROJECTAIRSIM_REF}#subdirectory=client/python/projectairsim"; \
    fi \
 && rm -rf /var/lib/apt/lists/*
RUN git config --system --add safe.directory '*'
ENV VLAGUARD_ROOT=/opt/vlaguard \
    VLAGUARD_MODEL_ROOT=/models \
    GIT_OPTIONAL_LOCKS=0 \
    HF_HUB_OFFLINE=1 \
    TRANSFORMERS_OFFLINE=1 \
    PYTHONDONTWRITEBYTECODE=1
WORKDIR /opt/vlaguard
ENTRYPOINT ["python3", "tools/vla_backend_table.py"]
CMD ["protocol"]
