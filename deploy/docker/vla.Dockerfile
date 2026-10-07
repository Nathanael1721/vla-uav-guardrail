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
#   * GPU user space has to match the host's driver stack. On the Orin that is
#     L4T (JetPack); on the desktop, CUDA 12.4 as in the vla-real environment.
#     The companion image needs neither, which is what lets it be one
#     multi-arch definition.
#
# So this definition takes its base as an argument, and hil and flight use the
# same arm64 build of it (deploy/topologies/hil.env = flight.env):
#
#   desktop: docker build -f deploy/docker/vla.Dockerfile \
#              --build-arg BASE_IMAGE=pytorch/pytorch:2.6.0-cuda12.4-cudnn9-runtime \
#              -t vlaguard/vla:desktop-cu124 .
#   Orin:    docker build -f deploy/docker/vla.Dockerfile \
#              --build-arg BASE_IMAGE=dustynv/l4t-pytorch:r36.4.0 \
#              -t vlaguard/vla:l4t-r36 .
#
# The Orin base must match the Orin's L4T release (cat /etc/nv_tegra_release);
# jetson-containers' `autotag l4t-pytorch` prints the matching tag. Written on
# 2026-10-06 and NOT yet built: bitsandbytes (4-bit NF4) on Jetson is the step
# most likely to need a different wheel - docs/RUNBOOK-orin-hil.md, step 11.
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
RUN apt-get update \
 && apt-get install -y --no-install-recommends git ca-certificates libgl1 libglib2.0-0 \
 && rm -rf /var/lib/apt/lists/*
COPY requirements.txt deploy/docker/requirements-vla.txt /tmp/req/
RUN python3 -m pip install -r /tmp/req/requirements.txt -r /tmp/req/requirements-vla.txt \
 && if [ "${WITH_PROJECTAIRSIM}" = "1" ]; then \
      python3 -m pip install "projectairsim @ git+https://github.com/iamaisim/ProjectAirSim.git@${PROJECTAIRSIM_REF}#subdirectory=client/python/projectairsim"; \
    fi
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
