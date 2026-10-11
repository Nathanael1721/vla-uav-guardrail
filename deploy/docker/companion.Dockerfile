# syntax=docker/dockerfile:1
#
# The companion image: ROS 2 Jazzy + MAVROS 2 + mavlink-router + the Python
# environment the Shield node, the VLA stub node and the profiling tools run in.
#
# ONE definition for every topology (grant, Architecture constraints p.3: "The
# same code base must run in three configurations"; reference
# docs/03-simulation/topologies.md: "The identical Docker image runs in all
# three. Topology selection is a config flag, not a rebuild."):
#
#   dev     desktop, linux/amd64
#   hil     Jetson Orin, linux/arm64   } the same arm64 image, unchanged
#   flight  Jetson Orin, linux/arm64   }
#
# Roles (deploy/docker/entrypoint.sh): router, mavros, mission (the native
# rail's launch file: VLA stub + MAVLink adapter + Shield node), profile,
# facts, vla-table, versions, shell.
#
# Build (written on 2026-10-06, NOT yet built or run - see deploy/README.md):
#   docker buildx build --platform linux/amd64,linux/arm64 \
#       -f deploy/docker/companion.Dockerfile -t vlaguard/companion:jazzy .
# or natively on each host:
#   docker build -f deploy/docker/companion.Dockerfile -t vlaguard/companion:jazzy .
#
# THE ORIN RUNS JETPACK 6.2 (L4T r36.4.x, Ubuntu 22.04, kernel 5.15). ROS 2
# Humble is the distribution native to that Ubuntu; this image keeps ROS 2
# JAZZY on an Ubuntu 24.04 (noble) user space instead, in a container on the
# 22.04 host (docs/DESIGN-topologies.md, decision 2, has the full case):
#   * The grant's Safety Shield page: "Python 3.11+ as a ROS 2 node (rclpy)".
#     rclpy is built for its distribution's own python3: Jazzy's is 3.12,
#     Humble's 3.10, below that floor. The PI asked (6 Oct) to conform rather
#     than ask for waivers. The build refuses a base whose rclpy runs below
#     3.11 (the check after the venv below), so a Humble base cannot slip in.
#   * This image uses no GPU: no CUDA, cuDNN, TensorRT or VPI, nothing from
#     L4T. Only the host's kernel is shared, and an Ubuntu 24.04 user space
#     runs on the 5.15 L4T kernel. The GPU work lives in vla.Dockerfile, which
#     does follow L4T r36.4.
#   * One distribution on every host: the desktop's dev rail (demo/out/ros2_*)
#     flew on Jazzy + MAVROS 2, and ROS 2 does not support mixing
#     distributions in one DDS graph (the optional desktop-Orin monitoring).
#   * ros:jazzy-ros-base-noble is an official multi-arch image (amd64 and
#     arm64v8); the tag names the 24.04 user space explicitly.
# On the Orin the compose file runs these containers with `runtime: runc`, so
# a Docker daemon whose default runtime is nvidia (as jetson-containers' set-up
# recommends) does not inject L4T's GPU libraries into the safety path.
#
# The ROS 2 distribution is the base image's own ROS_DISTRO (set by every
# official ros image), not a second build argument that could disagree with
# the base.
#
# The repository is mounted at /opt/vlaguard at run time, read-only, never
# copied in: guardrail.manifest.code_revision() then reads the real checkout,
# and a run names the commit that flew. Only requirements.txt and the
# deploy/docker scripts are in the build context
# (companion.Dockerfile.dockerignore).

ARG BASE_IMAGE=ros:jazzy-ros-base-noble

# --------------------------------------------------------------------------- #
# mavlink-router, built from source (no Ubuntu package exists).
# --------------------------------------------------------------------------- #
FROM ${BASE_IMAGE} AS router-build
ARG MAVLINK_ROUTER_REF=v4
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      git ca-certificates meson ninja-build pkg-config g++ python3 \
 && rm -rf /var/lib/apt/lists/*
RUN git clone --depth 1 --branch "${MAVLINK_ROUTER_REF}" --recurse-submodules \
      --shallow-submodules https://github.com/mavlink-router/mavlink-router.git /src \
 && cd /src \
 && meson setup build . --buildtype=release -Dsystemdsystemunitdir=/usr/lib/systemd/system \
 && ninja -C build \
 && install -m 0755 build/src/mavlink-routerd /usr/local/bin/mavlink-routerd

# --------------------------------------------------------------------------- #
# Runtime
# --------------------------------------------------------------------------- #
FROM ${BASE_IMAGE}
# No build arguments change what is installed: hil and flight must run the
# very same image. PyTorch is not here; the behaviour-cloned policy's R1 rate
# is measured in the vla image (deploy/docker/vla.Dockerfile), which has it.
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# MAVROS 2 for the base's distribution (ros-jazzy-mavros, -mavros-extras),
# Cyclone DDS and rosbag2's MCAP storage (the mission role records the
# grant's topics, as sitl/run_ros2_demo.sh does) from the ROS apt repository
# the base image carries. python3-numpy/-yaml/-jinja2/-cryptography are the
# same Ubuntu 24.04 packages the WSL rail's venv sees
# (docs/DESIGN-python-versions.md, ~/venv-ros). The ROS apt repository keeps
# only the newest build of each package, so versions are not pinned here; the
# `versions` role prints what an image holds, and the runbook records it.
RUN test -n "${ROS_DISTRO:-}" \
      || { echo "the base image sets no ROS_DISTRO: not an official ros image" >&2; exit 1; } \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-rosbag2-storage-mcap \
      geographiclib-tools wget ca-certificates \
      python3-venv python3-pip python3-numpy python3-yaml python3-jinja2 \
      python3-cryptography \
      git iproute2 iputils-ping \
 && rm -rf /var/lib/apt/lists/*

# MAVROS needs the geoid datasets at start-up or it exits.
RUN /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

COPY --from=router-build /usr/local/bin/mavlink-routerd /usr/local/bin/mavlink-routerd

# The guardrail's own dependency list (requirements.txt, kept equal to
# pyproject.toml by tests/test_env_pins.py), pinned to the versions the KPI
# rail ran (constraints-companion.txt).
COPY requirements.txt deploy/docker/constraints-companion.txt /tmp/req/
RUN python3 -m venv --system-site-packages /opt/venv \
 && /opt/venv/bin/pip install --no-cache-dir \
      -r /tmp/req/requirements.txt -c /tmp/req/constraints-companion.txt

# The grant's floor for the Shield as an rclpy node (Safety Shield p.1:
# "Python 3.11+ as a ROS 2 node (rclpy)"), checked on the interpreter that
# runs the Shield node (VLAGUARD_PYTHON below) with the distribution's rclpy
# imported. A Humble base (Ubuntu 22.04, Python 3.10) stops here with the
# reason, instead of building an image that flies below the floor.
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && /opt/venv/bin/python -c 'import sys, rclpy; v = sys.version_info[:2]; sys.exit(0 if v >= (3, 11) else "rclpy runs on Python %d.%d here, below the grant floor 3.11 (Safety Shield p.1); see docs/DESIGN-topologies.md, decision 2" % v)'

# The checkout is mounted with the host user's ownership; without this git
# refuses it ("dubious ownership") and every manifest would say "unversioned".
RUN git config --system --add safe.directory '*'

ENV VLAGUARD_ROOT=/opt/vlaguard \
    VLAGUARD_PYTHON=/opt/venv/bin/python \
    GIT_OPTIONAL_LOCKS=0 \
    PYTHONDONTWRITEBYTECODE=1 \
    ROS_LOG_DIR=/tmp/ros_log

COPY deploy/docker/entrypoint.sh /usr/local/bin/vlaguard-entrypoint
RUN chmod 0755 /usr/local/bin/vlaguard-entrypoint

WORKDIR /opt/vlaguard
ENTRYPOINT ["/usr/local/bin/vlaguard-entrypoint"]
CMD ["help"]
