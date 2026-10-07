# syntax=docker/dockerfile:1
#
# ArduPilot SITL (ArduCopter) for the desktop side of dev and hil.
# Never runs on the Orin and never in flight: there the autopilot is real.
#
# The firmware is the pin in sitl/setup_sitl.sh (ARCH-31): Copter-4.5.7 at
# 2a3dc4b7bf2507120f7378a7b2fde73185e0c325. The clone is checked against the
# SHA, so a moved tag fails the build instead of building another autopilot.
# tests/test_deploy_configs.py keeps the two pins equal.
#
# The entrypoint starts the autopilot as sitl/start_sitl.sh does (EEPROM wiped
# at each start, the GeoFence backstop sitl/fence/guardrail_fence.parm) or as
# demo/pas_ardupilot/start_ardupilot.sh does for Project AirSim; both parameter
# files come from the repository, which the compose files mount read-only at
# /opt/vlaguard. Nothing of the repository is copied in.
#
# Build (written on 2026-10-06, NOT yet built or run):
#   docker build -f deploy/docker/sitl.Dockerfile -t vlaguard/sitl:copter-4.5.7 .

FROM ubuntu:22.04 AS build
ARG ARDUPILOT_REF=Copter-4.5.7
ARG ARDUPILOT_SHA=2a3dc4b7bf2507120f7378a7b2fde73185e0c325
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      git ca-certificates g++ make pkg-config libtool \
      python3 python3-dev python3-pip python-is-python3 \
 && rm -rf /var/lib/apt/lists/*
# waf's code generators: empy must stay at 3.3.4 (4.x is not supported by the
# ArduPilot build), the others as in ArduPilot's install-prereqs script.
RUN pip3 install --no-cache-dir empy==3.3.4 future pexpect lxml dronecan
RUN git clone --branch "${ARDUPILOT_REF}" https://github.com/ArduPilot/ardupilot.git /ardupilot \
 && cd /ardupilot \
 && test "$(git rev-parse HEAD)" = "${ARDUPILOT_SHA}" \
 && git submodule update --init --recursive \
 && ./waf configure --board sitl \
 && ./waf copter

FROM ubuntu:22.04
RUN apt-get update \
 && apt-get install -y --no-install-recommends libstdc++6 iproute2 \
 && rm -rf /var/lib/apt/lists/*
COPY --from=build /ardupilot/build/sitl/bin/arducopter /usr/local/bin/arducopter
COPY --from=build /ardupilot/Tools/autotest/default_params/copter.parm \
                  /ardupilot/Tools/autotest/default_params/airsim-quadX.parm \
                  /opt/ardupilot/params/
COPY deploy/docker/sitl-entrypoint.sh /usr/local/bin/sitl-entrypoint
RUN chmod 0755 /usr/local/bin/sitl-entrypoint
VOLUME ["/sitl-run"]
ENTRYPOINT ["/usr/local/bin/sitl-entrypoint"]
