# Design: one stack for dev, hil and flight

**Date:** 2026-10-06; revised 2026-10-08 for JetPack 6.2 (decisions 1, 2
and 10)
**Files:** `deploy/` (images, compose files, topology files, the Orin's
first-boot and GPU checks in `deploy/orin/`), `tools/profile_shield_tick.py`,
`tools/vla_backend_table.py`, and their tests `tests/test_deploy_configs.py`,
`tests/test_profile_shield_tick.py`, `tests/test_vla_backend_table.py`. The
step-by-step for the Orin is [RUNBOOK-orin-hil.md](RUNBOOK-orin-hil.md).

## What the grant asks for

The grant (Architecture constraints, p.3) fixes three deployment topologies and
says "The same code base must run in three configurations. All three are
first-class". Its table, verbatim:

| Topology | What runs where | Purpose |
|---|---|---|
| dev | Everything on a single desktop (incl. ArduPilot SITL, Gazebo, Project AirSim, MAVROS, Shield, VLA stub or quantized backend) | Fast iteration, CI, unit/integration testing |
| hil | Desktop runs sims + MAVROS + GCS; Jetson Orin runs VLA + Shield over network | **Canonical KPI configuration** — what every reported number is measured against |
| flight | Orin runs VLA + Shield as a companion computer alongside real ArduPilot autopilot | Closest to the AI Wings real-flight stack; final demos |

Below the table the grant points to the reference design for the details: "See
Topologies for the diagram and per-topology component layout". That page
(`kuanting-vla-uav-guardrail/docs/03-simulation/topologies.md`) adds the rule
this design is built around: "The identical Docker image runs in all three.
Topology selection is a config flag, not a rebuild." Its hil diagram places
MAVROS 2 on the Orin, next to the Shield and the VLA, with mavlink-router on
the desktop feeding it "UDP / Ethernet", and it says "promoting from hil to
flight is a configuration change, not a code change."

A Jetson Orin is the hil machine, and the same stack is to move onto the drone
once the airframe is ready. The Orin is being installed with **JetPack 6.2**
(Jetson Linux / L4T r36.4.x on Ubuntu 22.04); decision 10 lists what that
fixes.

## Decisions

### 1. Two images, each identical across topologies

| Image | Contents | dev | hil | flight |
|---|---|---|---|---|
| `vlaguard/companion:jazzy` (`deploy/docker/companion.Dockerfile`) | ROS 2 Jazzy on Ubuntu 24.04, MAVROS 2, mavlink-router, Cyclone DDS, rosbag2 (MCAP), the guardrail's Python environment (3.12); no GPU | amd64 build | arm64 build | **the same** arm64 build |
| `vlaguard/vla:*` (`deploy/docker/vla.Dockerfile`) | PyTorch, transformers and bitsandbytes for the OpenVLA-based backends; every R1 rate measurement | CUDA 12.4 base (`vlaguard/vla:desktop-cu124`) | L4T r36.4 base for JetPack 6.2 (`vlaguard/vla:l4t-r36.4-cu128`, decision 10) | **the same** L4T build |

The companion image has no build argument that changes what is installed, so
"the same image in hil and flight" cannot depend on how it was built (a test
checks the Dockerfile's arguments). A third image, `vlaguard/sitl`
(`deploy/docker/sitl.Dockerfile`), is ArduPilot SITL at the version
`sitl/setup_sitl.sh` pins (Copter-4.5.7). It runs on the desktop only; in
flight the autopilot is real.

Why the VLA is not inside the companion image:

- The safety path (MAVROS, Shield) must not share a process, or a dependency
  stack, with a 7 B model. A model that runs out of memory must not take the
  Shield down with it.
- The docstring of `demo/vla_server.py` records a desktop measurement: the same
  model took about 2.5 s per action in its own process and about 12 s inside
  the 10 Hz flight process.
- GPU user space must suit the host's driver stack: L4T r36.4 on the Orin,
  CUDA 12.4 on the desktop. The companion image needs no GPU, so it can be one
  multi-arch definition built from the official `ros:jazzy-ros-base-noble`
  image. Only the VLA image has a per-architecture base, and hil and flight
  still share one build.

### 2. ROS 2 Jazzy inside the containers, on JetPack 6.2's Ubuntu 22.04

The grant leaves the ROS 2 distribution open ("Humble (LTS, AI Wings parity),
Iron, or Jazzy?", Safety Shield p.7). The PI's answer of 6 Oct was to choose
by what the Orin's JetPack supports, keep the code distribution-agnostic, and
conform to the grant rather than ask for waivers. JetPack 6.2 is Ubuntu 22.04,
so the question became concrete:

| | ROS 2 Humble, native on the host | ROS 2 Jazzy in an Ubuntu 24.04 container (chosen) |
|---|---|---|
| How it runs on JetPack 6.2 | apt packages on the 22.04 host, or `ros:humble-ros-base-jammy` | `ros:jazzy-ros-base-noble` (official, amd64 and arm64v8; checked 2026-10-08); only the kernel is shared |
| Python of rclpy, so of the Shield node | 3.10, **below the grant's "Python 3.11+ as a ROS 2 node (rclpy)"** (Safety Shield p.1) | 3.12 |
| Same distribution as the desktop's dev rail (`demo/out/ros2_*`, WSL Ubuntu 24.04) | no | yes |
| One DDS graph across the cable (optional monitoring, decision 4) | not with the desktop's Jazzy: ROS 2 does not support mixing distributions in one graph | yes |
| Needs the GPU or anything from L4T | no | no |
| Support ends | May 2027 | May 2029 |
| AI Wings / reference implementation parity | yes (the reference repository's nodes are Humble) | no |

Jazzy is chosen. The deciding reason is the grant's Python floor: rclpy is
built for its distribution's own `python3`, so a Humble Shield node runs on
3.10, a deviation the PI asked us not to request. The companion image needs
no GPU (MAVROS, the Shield, mavlink-router and rosbag2 are CPU only), so the
container's Ubuntu does not have to match L4T: an Ubuntu 24.04 user space runs
on L4T's 5.15 kernel, and jetson-containers (the Jetson AI Lab container
project, published under the dustynv Docker Hub account; not an NGC image)
publishes Jazzy for r36.4 on a 24.04 user space in the same way
(`dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04`, checked 2026-10-08; this
image is not used, because the companion needs no CUDA). Humble's advantage,
parity with the AI Wings stack and the reference implementation, matters only
where two ROS graphs exchange DDS traffic; the interface to the autopilot is
MAVLink over UDP or serial, which no ROS distribution touches. The rail's
nodes are plain rclpy, and the guardrail package also imports on Python 3.10
(`docs/DESIGN-python-versions.md`), so the code stays distribution-agnostic.

**What changes in the companion image for JetPack 6.2** (2026-10-08):

- The base is pinned as `ros:jazzy-ros-base-noble`, naming the 24.04 user
  space instead of leaving it to the `jazzy-ros-base` alias.
- The separate `ROS_DISTRO` build argument is gone. The distribution is the
  base image's own `ROS_DISTRO`, so the packages installed (`ros-jazzy-mavros`,
  `ros-jazzy-mavros-extras`, `ros-jazzy-rmw-cyclonedds-cpp`,
  `ros-jazzy-rosbag2-storage-mcap`, MAVROS's GeographicLib datasets) can never
  name another distribution than the base carries.
- The build checks the grant's floor: with ROS sourced, the Shield node's
  interpreter (`/opt/venv/bin/python`) must import rclpy on Python 3.11 or
  newer, or the build stops with the reason. A Humble base therefore cannot
  produce an image; switching to Humble would take the PI's written acceptance
  of a 3.10 Shield and an edit of that check, nothing else.
- On the Orin the companion services run with `runtime: runc`
  (`deploy/compose/docker-compose.orin.yml`), so a Docker daemon whose default
  runtime is `nvidia` (jetson-containers' recommended set-up) does not mount
  L4T's GPU libraries into the safety path.
- A `versions` role prints what an image holds (its Ubuntu, `ros_distro`,
  rclpy's Python, MAVROS, Cyclone DDS, rosbag2 MCAP, mavlink-routerd, the
  guardrail's packages), for the record the runbook keeps. The ROS apt
  repository keeps only the newest build of each package, so these versions
  are recorded per image rather than pinned.

Nothing ROS is installed on the Orin's own Ubuntu; the first-boot check warns
if something is.

### 3. MAVROS 2 on the Orin in hil

The hil layout follows the per-topology layout the grant points to (the
reference Topologies page): MAVROS 2, the Shield and the VLA on the Orin;
ArduPilot SITL, mavlink-router, Project AirSim and Mission Planner on the
desktop. The grant's one-line table entry ("Desktop runs sims + MAVROS + GCS")
puts MAVROS on the desktop instead; the two differ only in that, and this
design takes the page the table refers to for its layout. Reasons:

- In flight there is no desktop, so MAVROS runs on the Orin there. Keeping it
  on the Orin in hil makes hil and flight the same set of containers on the
  same host; only the topology file changes.
- The Shield to MAVROS hop is part of the safety path, and it stays inside one
  host.
- The cable carries MAVLink over UDP, which does not depend on the ROS
  distribution on each side.
- `guardrail.manifest` accepts both layouts as hil (`check_topology_evidence`
  reads where MAVROS ran from the Orin's processes and requires the SITL
  autopilot to run off the Orin either way), so the choice does not change
  what a run can claim.

The table's wording can be run too, if it is ever wanted: start MAVROS on the
desktop, start only the mission on the Orin, and use `deploy/dds/cyclonedds.xml`
so the two ROS graphs see each other across the cable. That variant needs the
same ROS distribution on both sides, and no files for it are written here.

### 4. mavlink-router feeds MAVROS and Mission Planner in parallel, with the native rail's configuration

The grant: Mission Planner gets MAVLink "via mavlink-router fan-out (parallel
to MAVROS), not as a serial bottleneck" (Grant overview). The containers do
not carry a second router configuration: the companion image's `router` role
runs the native rail's own `sitl/start_router.sh`, which renders
`sitl/mavlink-router/main.conf` with the topology's addresses and starts
`mavlink-routerd`. ArduPilot's SERIAL2 sends to the router on UDP 14550 (the
MAVLink port in the grant's interface table), and the router forwards every
message to MAVROS 2 (UDP 14555) and to Mission Planner (UDP 14550 on the
desktop), and their commands back.

| Link | Protocol | dev | hil |
|---|---|---|---|
| SITL to mavlink-router | UDP 14550 | inside one container network | on the desktop |
| mavlink-router to MAVROS 2 | UDP 14555 | 127.0.0.1 | to the Orin (`ORIN_IP`) |
| MAVROS 2 `fcu_url` | | `udp://:14555@` (as the WSL rail) | `udp://:14555@<DESKTOP_IP>:14555` |
| mavlink-router to Mission Planner | UDP 14550 | the Windows host | the Windows host |
| Project AirSim to SITL (sensors) | UDP 9003 | published on the desktop | published on the desktop |
| SITL to Project AirSim (motor outputs) | UDP 9002 | to the Windows host | to the Windows host |
| Project AirSim client API (camera frames for a VLA on the Orin) | TCP 8989/8990 | local | Orin to desktop |
| ROS 2 DDS (optional monitoring) | Cyclone DDS, unicast peers | local | desktop and Orin |

In hil, MAVROS names the desktop as its remote, part of the evidence the
manifest reads (decision 7). MAVROS then answers whichever address the
router's MAVLink actually arrives from, so no MAVLink port has to be published
on the desktop. `tests/test_deploy_configs.py` runs the router role with a
stand-in `mavlink-routerd` and checks the rendered endpoints against each
topology file and `FCU_URL`.

### 5. Project AirSim and Mission Planner

Project AirSim (the IAMAI fork used here) has a native ArduPilot controller:
a robot configuration with `"controller": {"type": "ardupilot-api", ...}`
exchanges sensor and motor packets with ArduPilot SITL over UDP 9002/9003, and
SITL runs with ArduPilot's AirSim model (`sim_vehicle.py -v ArduCopter -f
airsim-copter`). In that mode Project AirSim computes the physics and the
sensors, and ArduPilot runs its own controllers on them; every command reaches
the aircraft as MAVLink, which is the grant's control path. The native rail
for it, its addresses across WSL 2 and its acceptance gate are in
[DESIGN-projectairsim-ardupilot.md](DESIGN-projectairsim-ardupilot.md).

**Can Project AirSim connect to Mission Planner?** Yes, through ArduPilot,
which is how a real aircraft is connected too. Mission Planner never talks to
the simulator; it talks to the autopilot. The chain is Project AirSim to
ArduPilot SITL (UDP 9002/9003), then ArduPilot to mavlink-router, then Mission
Planner on UDP 14550, in parallel with MAVROS 2.

In the SITL container, `SITL_MODEL=airsim-copter` starts the autopilot as
`demo/pas_ardupilot/start_ardupilot.sh` does: `copter.parm`,
`airsim-quadX.parm`, then the airframe file `SITL_AIRFRAME_PARM` (for the
CityLife scene `demo/pas_ardupilot/citylife-quad.param`), with `--home` set to
the scene's home-geo-point (`python3 demo/pas_ardupilot/rail.py home`). The
entrypoint refuses the mode with the default CMAC home or without an airframe
file: ArduPilot's AirSim backend measures position from `--home` to each GPS
fix, so the wrong home puts the vehicle thousands of kilometres from the scene.
A test compares the container's command with `rail.py`'s.

### 6. The code is mounted, not copied

The checkout is mounted read-only at `/opt/vlaguard` in every container.
`guardrail.manifest.code_revision()` then reads the real git history, and a run
on the Orin names the commit that flew. An image with the code baked in would
report `unversioned`, and `is_kpi_grade()` refuses that. The desktop and the
Orin must be on the same commit; the runbook checks it.

A modified checkout records `-dirty`, which the KPI gate also refuses. So
nothing measured on the Orin is written into a tracked path: the Orin compose
file sets `VLAGUARD_EVIDENCE_OUT=deploy/evidence/incoming/`, which git ignores
(`deploy/evidence/incoming/.gitignore`), and runs land in `demo/out/`, also
ignored. Reviewed files are copied to the desktop and committed there.

### 7. The label comes from evidence; the topology file only selects configuration

`VLAGUARD_TOPOLOGY` in a topology file picks addresses, images and refusals.
It is not the label a run carries. The Shield node gathers evidence from the
machine it runs on and the link it flies over, and
`guardrail.manifest.detect_topology` decides `dev`, `hil` or `flight` from it
(`check_topology_evidence`): the MAVROS chain, the autopilot's own version,
SITL or a hardware autopilot, the board and L4T release, the VLA node's host,
the processes running on the Shield's host, and the `fcu_url` remote.
`deploy/` supplies what that evidence needs on the Orin:

| Evidence the manifest reads | What `deploy/` provides |
|---|---|
| The SITL autopilot runs off the Shield's host (hil): the `fcu_url` names a remote desktop, or the host's processes were scanned and show no SITL | `FCU_URL=udp://:14555@192.168.50.1:14555` in `hil.env`, which holds even where no scan could be made and gives the link record its interface (Ethernet); and `pid: host` on the `mission` service, so the Shield node's `/proc` scan covers the whole Orin: it sees MAVROS in the sibling container (`mavros_on: shield_host`) and would see a SITL started on the Orin itself, which keeps the run `dev` |
| The device tree names an Orin | `security_opt: systempaths=unconfined` on the services that gather evidence (`mission`, `profile`, `facts`, `vla-bench`): Docker masks `/sys/firmware`, where `/proc/device-tree` points, in a default container |
| The L4T release | `/etc/nv_tegra_release` mounted read-only into the same services |
| The VLA node's host is an Orin (`vla_host`) | the VLA stub runs in the `mission` container, with the same access to the device tree and the release file |
| aarch64 | the arm64 build of the companion image |

`tools/profile_shield_tick.py` and `tools/vla_backend_table.py` use the same
rule: they call `guardrail.manifest.collect_host_evidence` and its Orin check,
so a profile can never say `hil` on a host whose runs would say `dev` (a Jetson
Xavier, a Nano or a bare aarch64 board is refused). The entrypoint refuses any
topology name that is not `dev`, `hil` or `flight`.

### 8. A mission in a container is the native rail's mission

The `mission` role does what `sitl/run_ros2_demo.sh` does with `USE_LAUNCH=1`:
it records the grant's topics with rosbag2, starts
`sitl/ros2_ws/src/safety_shield/launch/guardrail_rail.launch.py` (the VLA stub,
the MAVLink adapter and the Shield node), and re-packs the replay bundle with
the closed bag. The MAVLink adapter is the node that turns the Shield's
body-frame setpoints into `/mavros/setpoint_raw/local`, forwards its mode
requests to `/mavros/set_mode` and uploads the policy's zones as the
autopilot's fence; without it nothing the Shield decides would reach
ArduPilot. Tests keep the node set equal to the native rail's, the topic list
equal to its bag list, and no compose service starts a second VLA stub.

The SITL container starts the autopilot as `sitl/start_sitl.sh` does: the
EEPROM is wiped at every start (a fence zone uploaded in the previous flight
would otherwise be armed from boot, and the KPI loop restarts SITL between
missions), and the GeoFence backstop `sitl/fence/guardrail_fence.parm` is
loaded after `copter.parm`.

### 9. Safety defaults for flight

- `deploy/topologies/flight.env` ships with `FCU_URL` and `FCU_DEVICE` empty.
  The entrypoint refuses to start MAVROS until `FCU_URL` names the autopilot
  link, and the flight-only override `docker-compose.orin.flight.yml` (which
  maps the serial device) refuses to start until `FCU_DEVICE` names it. hil
  maps no device at all.
- The Shield node arms the vehicle and takes off. In the flight topology the
  entrypoint refuses the mission unless `VLAGUARD_ALLOW_FLIGHT_MISSION=yes`,
  which the runbook sets only after the bench checks.
- ArduPilot's own geofence stays enabled on the airframe. The grant: the
  autopilot's GeoFence "is the ultimate backstop -- the Shield does not replace
  it, it pre-empts it."

### 10. JetPack 6.2 on the Orin (2026-10-08)

**What JetPack 6.2 ships** (NVIDIA's JetPack 6.2 and 6.2.1 pages and release
notes, read 2026-10-08):

| | JetPack 6.2 | JetPack 6.2.1 | JetPack 6.2.2 / 6.2.3 |
|---|---|---|---|
| Jetson Linux (L4T) | 36.4.3 | 36.4.4 | 36.5 / 36.5.2 |
| `/etc/nv_tegra_release` | `R36 (release), REVISION: 4.3` | `REVISION: 4.4` | `REVISION: 5.x` |
| Ubuntu, kernel | 22.04, 5.15 | same | same |
| CUDA | 12.6.10 | 12.6.10 | 12.6.10 |
| cuDNN | 9.3.0 | 9.3.0 | 9.3.0 |
| TensorRT | 10.3.0 | 10.3.0 | 10.3.0 |
| VPI | 3.2 | 3.2 | |
| Docker 28 | breaks published ports (no `iptable_raw` in the kernel) | fixed | |

The deployment is written for r36.4.3. r36.4.4 needs nothing different.
r36.5 (a JetPack 6.2.x released after the plan was made) has the same
compute stack per NVIDIA, but the container tags below were built for r36.4,
so there the GPU check decides.

**The VLA image's base on the Orin.** NVIDIA's own L4T containers are
published per L4T minor release, and the newest for JetPack 6 is r36.4.0
(NGC `nvcr.io/nvidia/l4t-jetpack:r36.4.0`; no r36.4.3 tag exists): the r36.4
line is what JetPack 6.1, 6.2 and 6.2.1 share. For PyTorch on JetPack 6.2,
NVIDIA's "PyTorch for Jetson" release notes list its iGPU framework container
releases 25.02 to 25.06 (torch 2.7 and 2.8 pre-releases, Ubuntu 24.04, CUDA
12.8), and NVIDIA staff on the developer forum point to the Jetson AI Lab
wheel index (torch 2.6.0 for JetPack 6 / CUDA 12.6). jetson-containers'
builds are used here because one of them carries the desktop's exact torch /
torchvision pair. They are third-party images, not NVIDIA's: the Jetson AI
Lab container project, published under the dustynv Docker Hub account, not
on NGC, and referenced by tag (runbook step 8 records the image IDs that were
built on). NVIDIA's iGPU containers are the next alternative if both images
below fail. Tags checked on Docker Hub on 2026-10-08:

| Image | What it is | Used |
|---|---|---|
| `dustynv/torchvision:0.21.0-r36.4.0-cu128-24.04` | torch 2.6 + torchvision 0.21.0, Ubuntu 24.04, Python 3.12 (in `/opt/venv`), CUDA 12.8 with `/usr/local/cuda/compat` on `LD_LIBRARY_PATH`, the CUDA toolkit (nvcc, cuobjdump), `CUDAARCHS=87` | **the Orin's VLA base** (`hil.env` = `flight.env`) |
| `dustynv/l4t-pytorch:r36.4.0` | torch 2.4.0 + torchvision 0.19.0 for r36.4, Ubuntu 22.04, Python 3.10, CUDA 12.6 (12.6.1, JetPack 6.2's CUDA line, so no forward compatibility is needed), cuDNN 9.4, the CUDA toolkit | **fallback**, if the GPU check fails on the first |
| `dustynv/pytorch:2.6-r36.4.0-cu128-24.04`, `2.7-r36.4.0`, `2.7-r36.4.0-cu128-24.04` | torch without torchvision | no: timm needs torchvision, and PyPI's aarch64 torchvision is not a Jetson build |
| `dustynv/transformers` | newest tag `r36.2.0` (JetPack 6.0) | no r36.4 image exists; transformers 4.40.1 is installed on the PyTorch base |
| `dustynv/bitsandbytes:0.45.4-r36.4.0-cu128-24.04` | bitsandbytes 0.45.4 built for the Orin | no: the desktop flies 0.49.2, which is built from source instead |
| `ros:jazzy-ros-base-noble`, `ros:humble-ros-base-jammy` | official ROS 2 images, amd64 and arm64v8 | the first is the companion's base (decision 2) |
| `dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04` | Jazzy with CUDA on r36.4 | no; it shows the same Jazzy-on-24.04 arrangement on r36.4 |

The base chosen carries the desktop's torch / torchvision pair (vla-real:
torch 2.6.0, torchvision 0.21.0), so the R1 table compares one framework
version across the desktop and Orin columns, and Python 3.12, so the
guardrail code in this image (the backend table, the CSP compiler) runs on
the grant's 3.11+. The PI's plan of 6 Oct named exactly this: "VLA runtime in
a 3.11 container on the Orin". Its CUDA 12.8 is newer than JetPack 6.2's 12.6;
it runs on the r36.4 driver through the forward-compatibility libraries the
image carries, which is the one assumption of this choice not yet seen on the
Orin. `deploy/orin/gpu_check.py --skip-bnb` on the pulled base tests exactly
that before anything is built on it (runbook step 8). If it fails, the
fallback runs JetPack 6.2's own CUDA 12.6 line, at two prices to record: Python
3.10 in the VLA image (below the grant's 3.11; the companion image and the
Shield are not affected), and torch 2.4.0 / torchvision 0.19.0 instead of the
desktop's 2.6.0 / 0.21.0, so the R1 table's desktop and Orin columns would no
longer share one framework version. bitsandbytes 0.49.2 asks for torch 2.3 or
newer, so the pin holds on either base.

Both jetson-containers images set `PIP_INDEX_URL` in their environment (the
chosen base to `pypi.jetson-ai-lab.dev/jp6/cu128`, the fallback to a
`jetson.webredirect.org` mirror over plain HTTP), which is why the install
keeps pip from reading any index the base sets (below).

**bitsandbytes on an Orin.** OpenVLA-7B is loaded in 4-bit NF4 through
bitsandbytes. Its PyPI wheels for aarch64 carry no sm_87 code (its
`.github/scripts/build-cuda.sh` targets sm_75, sm_80 and sm_90 on aarch64,
plus sm_100 and sm_120 with CUDA 12.8 or 12.9, and the CUDA guide's binary
compatibility of cubins does not extend to Tegra GPUs, so the sm_80 code does
not run on an Orin); an Orin is sm_87. The wheel installs and imports, and the first
4-bit kernel fails with "named symbol not found" (bitsandbytes issue #1930,
reported on JetPack 6.2, open on 2026-10-08). The Dockerfile of commit
10451bc installed that wheel on the Orin (a plain `pip install -r` of
requirements-vla.txt on every architecture), so its 4-bit kernels would have
failed there. `deploy/docker/vla-install.sh` now builds the pinned
version (0.49.2, from requirements-vla.txt, so the desktop and the Orin run
the same version) from its source tag with `COMPUTE_CAPABILITY=87`, and the
build stops unless `cuobjdump` finds sm_87 code in the library it installed.
`gpu_check.py` then runs an NF4 round trip on the GPU (runbook step 8).

The source build follows bitsandbytes' own install guide: `cmake
-DCOMPUTE_BACKEND=cuda -DCOMPUTE_CAPABILITY=87` configures and `cmake --build`
compiles `libbitsandbytes_cuda<ver>.so` into the source tree, and only then
does pip package the tree. pip alone would compile nothing: the tag's
`pyproject.toml` sets `wheel.cmake = false` under `[tool.scikit-build]`. The
tag needs CMake 3.22.1 or newer. The base images carry a pip-installed CMake
ahead of the apt one on PATH (their Docker Hub layer history runs `pip3
install --upgrade ... cmake`, and the chosen base's PATH starts with
`/opt/venv/bin`); either meets the source's floor 3.22.1 (the apt CMake is
3.28 on Ubuntu 24.04 and 3.22.1 on the 22.04 fallback). The script installs
the apt one in case a base has none, and prints `cmake --version` just before
the configure step, so the build log shows which CMake ran. With CUDA 12.8 in
the base, the library is `libbitsandbytes_cuda128.so`, the name bitsandbytes
looks for when `torch.version.cuda` is 12.8.

**pip on a Jetson base.** The install freezes the base image's torch,
torchvision and torchaudio versions into a constraints file before installing
anything, so a requirement that wanted another torch stops the build instead
of replacing the Jetson build with PyPI's (which has no Orin GPU code), and
compares them again at the end. Every pip call names PyPI with `--index-url`
and runs with `--isolated` and `PIP_CONFIG_FILE=/dev/null`. `--isolated`
alone drops the `PIP_*` variables and the per-user config file but still
reads the global `/etc/pip.conf` and a virtual environment's `pip.conf`
(shown on the desktop on 2026-10-09: with a venv whose `pip.ini` named an
index, `pip --isolated config list` printed it); with `PIP_CONFIG_FILE` set to
the null device, pip skips every config file. Whether the bases carry such a
file is not known (their layer history does not show it), so the build log
prints `pip config debug` first and the build stops if pip, with those
settings, still reports any setting. That is the whole guarantee: no index
or option a base image sets reaches these pip calls.

**Docker on JetPack 6.2.** JetPack 6 flashed from SDK Manager installs no
Docker. The runbook installs Docker Engine from Docker's repository with the
compose and buildx plugins (buildx is required: without it `docker build`
uses the legacy builder, which ignores the per-Dockerfile ignore files and
would send the whole checkout, the private signing key included, as build
context), pins 27.5.1 on r36.4.3 because Docker 28 needs a kernel table r36.4.3
lacks (the Orin's compose files publish no port, so they would run either
way), and registers the NVIDIA runtime with `nvidia-ctk`. The compose file
names each service's runtime (decision 2).

**Power mode for timing.** The Shield profile and the VLA rates depend on the
nvpmodel power mode and on dynamic clock scaling. Every timing session fixes
the mode by its ID (read from `/etc/nvpmodel.conf`, since IDs differ between
modules: on an AGX Orin 64GB 0 = MAXN, 1 = 15W, 2 = 30W (default),
3 = 50W), locks the clocks with `jetson_clocks`, records `nvpmodel -q` and
`jetson_clocks --show`, and logs `tegrastats`.

Which modes:

- **The highest mode with a power budget** (50W, ID 3, on an AGX Orin 64GB),
  clocks locked: the Shield profile, the R1 rates and the KPI missions.
- **The default mode** (30W, ID 2, on an AGX Orin 64GB), clocks locked: the
  Shield profile and the R1 rates once more, the conservative figure until
  the drone's power budget is known.
- **Not MAXN** (or MAXN_SUPER). The Jetson Linux 36.4.3 power guide calls it
  "an unconstrained power mode" and "an experimental mode", advises against
  prolonged heavy workloads in it, and engages hardware throttling once the
  module exceeds its TDP, so a long run in it can change speed part-way.

A timing budget is claimed for flight only from the mode the drone flies in.
nvpmodel is not visible inside the containers, so the mode is recorded on the
host and in the profile's note. The tests hold both this list and the
runbook's to that shape: the budgeted mode first, the default second, and no
item but "Not MAXN" naming MAXN.

**What the person setting up the Orin runs and sends back.**
`deploy/orin/first_boot_check.sh` (runbook step 1) is read-only. It prints
the raw outputs (L4T release, board, JetPack and its CUDA / cuDNN / TensorRT,
`nvpmodel -q` and the module's modes, `ls /opt/ros` and each host
distribution's rclpy / ros-base / MAVROS packages, `docker info` and its
plugins, memory, disks, network, time) and a verdict per expectation: FAIL for
what breaks the deployment (not an Orin, not JetPack 6, no nvidia runtime, no
buildx, no Docker daemon), WARN for what to know (JetPack 6.1 or 6.2.2+, no
`nvidia-jetpack` packages, ROS on the host with its rclpy version, Docker 28
on r36.4.3, a mode table it cannot read). The `power` verdict reads the mode
table from the module's own `/etc/nvpmodel.conf` and names its default, its
highest mode with a power budget (the largest wattage in a mode name) and its
MAXN mode by ID, because the IDs differ between modules (Jetson Linux 36.4.3
power guide: an AGX Orin 64GB has MAXN at 0, its default 30W at 2 and 50W at
3; an Orin NX flashed for the Super modes has MAXN_SUPER at 0 and defaults to
its 40W at 4; an Orin Nano Super has MAXN_SUPER at 2 and defaults to its 25W
at 1; a Nano without the Super flash has no MAXN mode).

## Measurements so far (desktop, dev)

### Shield time per tick, on real flight states

`tools/profile_shield_tick.py` replays 1,301 ticks of four flights on two rails
(the dev rail's static and hot-applied no-fly zones and subject stand-off, and
a Project AirSim city flight with a no-fly zone, a per-tick subject and an
obstacle map) through the Shield.

Replay fidelity, with its null: the replay reproduces the violated-rule set the
rail logged on 688 of the 688 ticks whose log names a violation. The other 613
ticks logged no violation, and a replay that never flags anything would match
those as well, so they show nothing. That includes all 599 packed ticks of the
city flight, which logged no violation in its 2,396 ticks: its agreement does
not verify the replay of its obstacle map or its subject. The stand-off flight
(`ros2_ped_on`) does verify the subject path: 160 of 160 logged ticks agree.

The `design_load` arm adds no-fly zones along each path until there are 50
rules, the load the grant states its budget at; every flight's forecasts reach
those zones on some ticks, which the tool checks. Results, pinned to one
logical CPU per core and with the host-speed reference steady before and
after:

| Condition | `_check` p99 (budget 5 ms) | `_check` max, ticks over 5 ms | `filter` p99 (budget 100 ms) | `filter` max | Host-speed reference |
|---|---|---|---|---|---|
| Rules as flown, 3 s horizon at 0.5 s (what the rails fly), performance cores | 0.22 ms | 0.29 ms, 0 of 1,301 | 0.78 ms | 1.05 ms | 39.0 to 38.1 ms (same run) |
| 50 rules, 5 s horizon at 0.1 s (the grant's stated load), performance cores | 3.38 ms | 4.24 ms, 0 of 1,301 | 11.9 ms | 27.4 ms | 39.0 to 38.1 ms |
| 50 rules, 5 s horizon at 0.1 s, efficiency cores | 7.60 ms | 9.01 ms, 148 of 1,301 | 27.5 ms | 45.8 ms | 62.9 to 62.6 ms |
| Floor: rules as flown minus every fence, 5 s horizon, performance cores | 0.30 ms | 0.43 ms, 0 of 1,301 | 1.10 ms | 1.83 ms | 39.0 to 38.1 ms (same run) |

Source: `deploy/evidence/dev/shield_tick_profile.json` (performance cores,
`--cpus 0,2,4,6,8,10,12,14`) and `shield_tick_profile-ecores.json` (efficiency
cores, `--cpus 16-27`). Two further runs of each, not committed (the second on
2026-10-07, with the same tool and pack), gave `_check` p99 3.17 and 3.21 ms
(max 4.56 and 4.28 ms, 0 over) on the performance cores and 7.63 and 7.73 ms
(157 and 165 ticks over) on the efficiency cores. `filter` varies more from run
to run: 11.9 and 16.3 ms on the performance cores, 27.0 and 40.4 ms on the
efficiency cores, all well inside 100 ms. Other processes were running on the
desktop throughout. The verdict is p99 against each budget; the grant states a budget,
not a percentile, so p99 is this project's choice, and the maximum and the
count of ticks over budget stand beside it.

The desktop is an Intel Core i7-14700: 8 performance cores with two hardware
threads each (logical CPUs 0 to 15) and 12 efficiency cores (16 to 27).
Unpinned runs moved from the first kind to the second mid-run, and the same
code measured 2 to 3 times apart. The tool therefore pins the CPUs (`--cpus`),
times a fixed reference workload before and after each run, and FAILS a run
whose reference moved by more than 1.5 times: such timings mix two host speeds,
and `attach` then reports the verdict as undetermined.

What this means for the Orin: the rails as flown use a few percent of the
budgets. At the grant's stated load (50 rules, 50 future poses) the 5 ms query
budget holds on the desktop's performance cores, with every tick under it, and
not on its efficiency cores. The grant measures it on the Orin, whose cores are
expected to be slower per core than this desktop's, so the Orin profile
(runbook step 12) decides whether the monitor needs the faster path the
reference design names as its fallback ("Rust hot-path ... if profiling
fails", reference `docs/02-implementation/overview.md`). The host-speed
reference in each profile makes the Orin's numbers comparable with these.

### VLA backends

`tools/vla_backend_table.py` fixes one protocol (R1: inference rate on fixed
inputs, 5 warm-up and 50 timed inferences, nothing else on the GPU; C1: closed
loop on `policies/urban_demo_policy.yaml`, 5 seeds, no assist, with hover and
the stub as nulls) and builds the table from files on disk, citing each one
(`deploy/evidence/vla_backend_table.md`). Measured with R1 on the desktop CPU
so far: the stub (p50 0.0013 ms) and the behaviour-cloned state policy (p50
0.307 ms, p95 0.373 ms). The OpenVLA-based backends have desktop numbers taken
under other conditions (a two-inference load probe: 0.86 s per inference warm
in 4-bit; in-flight rates of 0.11 to 0.15 Hz with the simulator on the same
GPU), which the table lists with their conditions. OpenVLA's R1 uses the flight
script's default prompt and records its token count; a `--csp on` arm times
the longer prompt with the Constraint Summary Pack. Every Orin cell is still to
be measured (runbook step 13). CognitiveDrone and BitVLA, both named by the
grant, have no checkpoint here. No backend is the default: the grant defers
that choice to this measurement set.

## Status

| Item | Written | Built | Run |
|---|---|---|---|
| Companion, VLA and SITL images (`deploy/docker/`) | yes | not yet | not yet |
| dev, hil-desktop, Orin and flight-override compose files | yes | - | `docker compose config` only |
| Topology files and DDS configuration | yes | - | checked by tests |
| Entrypoint roles and refusals | yes | - | dry-run in tests; the router role with a stand-in `mavlink-routerd`; the `versions` role for real (no ROS on this desktop) |
| Companion image's Python-floor check (JetPack 6.2, decision 2) | yes | - | its code executed in tests on Python 3.10 (refused) and 3.11 (passed) |
| VLA install for the r36.4 base (`vla-install.sh`) | yes | - | dry-run plans in tests (with a stand-in interpreter, no torch needed on the host); the sm_87 check, the torch comparison and the pip-config check with stand-ins |
| First-boot check (`deploy/orin/first_boot_check.sh`) | yes | - | against stand-in Orins in tests (AGX Orin, Orin NX and Orin Nano mode tables); not yet on the Orin |
| GPU check (`deploy/orin/gpu_check.py`) | yes | - | its verdicts, and its measuring code against stand-in torch and bitsandbytes modules, in tests; a real GPU needs the Orin |
| Shield tick profile | yes | - | desktop baseline committed; Orin next |
| VLA backend protocol and table | yes | - | stub and BC on the desktop CPU; Orin next |

`tests/test_deploy_configs.py` checks the configuration without building
anything; a mutation run broke the configuration in 35 different ways (the hil
`FCU_URL`, the device-tree access, the mission's view of the host's processes,
the evidence output folders, the launch file, a bag topic, the router
addresses, the SITL wipe, fence, home and airframe file, the ROS domain, a
build argument, a device mapping in hil, the flight device guard, and others),
and each was caught. The same run broke the
two tools 26 ways (the grant's budgets, horizon and headline condition, the
timed window, the subject, the Orin rule, the agreement null, the host-speed
check, the claim checks and the correction's null, among others); all were
caught once one test was tightened.

The JetPack 6.2 revision (2026-10-08) was checked the same way: 32 mutants of
the new safeguards on a scratch copy (the Python floor lowered to 3.10 or run
without rclpy on the path, the companion base back to the `jazzy-ros-base`
alias, a second `ROS_DISTRO` argument, `runc` or `nvidia` dropped from an Orin
service, the hil or flight VLA image back to the old tags, the runbook building
the old tag, pip without `--isolated` or without the torch constraints, sm 80
instead of 87, the sm_87 check removed or made to pass anything, bitsandbytes
from PyPI on aarch64, the final torch comparison or the "no torch" refusal
removed, the Dockerfile not running the install script, the `versions` role
renamed or missing `ros_distro`, the first-boot check passing a missing buildx
or nvidia runtime, a Xavier, R36.3 or R36.5, losing the Docker 28 warning, its
exit code or the `docker info` output, and the GPU check losing its lower NF4
bound, the capability check, the CPU-torch check or its upper bound). All 32
were caught.

Two things in commit 10451bc were defects, not only missing JetPack 6.2
requirements. The arm64 VLA image ran a plain `pip install -r` of
requirements-vla.txt, which installs PyPI's aarch64 bitsandbytes 0.49.2; it
has no sm_87 code, so every 4-bit kernel would have failed on the Orin. It is
now built from source for sm_87 and checked with `cuobjdump`. And the
runbook's Orin profile pinned `--cpus 4-7`, cores that an AGX Orin's 15W mode
and an Orin NX's 10W mode switch off; it now pins 1-3, with the instruction
to compare `/sys/devices/system/cpu/online` in each mode measured.

A second pass the same day checked the (uncommitted, never built) JetPack 6.2
draft against its sources and found two errors in it, both corrected before
anything was committed: its install script handed the bitsandbytes CMake
options to pip as `CMAKE_ARGS`, which builds no CUDA library (the build would
have stopped at the sm_87 check), and its runbook sent the timing runs to
MAXN. The plan test now requires CMake to configure and compile with
`COMPUTE_CAPABILITY=87` before pip, and failed on the draft. The same pass
moved the timing mode to the highest mode with a power budget (above, "Power
mode for timing"), and made the first-boot check
report what step 4 needs: the module's mode table by ID, its default, its
highest budgeted mode and its MAXN mode (an AGX Orin, an Orin NX and an Orin
Nano with and without the Super flash), and, for any ROS on the host, the
distribution's rclpy version. Each new verdict was shown failing on the draft
first. The tags in the table above, the fallback base's torch version and the
power-mode IDs in the runbook were re-read from Docker Hub's image histories
and the Jetson Linux 36.4.3 power guide. A mutation run (each file edited,
tested and restored) broke the new safeguards 15 ways (the capability, the
compile step, the order, the CMake options sent to pip again, the compiler
path; the default, the warning without a table, MAXN_SUPER recognition, the
Super note, the highest budgeted mode picked as the lowest or left out of the
verdict, the host rclpy version and its listing; the fallback's torch left
unstated, and a timing run sent to MAXN again, in the runbook); all 15 were
caught.

A review on 2026-10-09 broke the result 20 more ways on a scratch copy, and
9 survived. This pass closed each gap and corrected what the review found
overstated:

- `gpu_check.py`'s measuring code was untested; only its verdicts were. A
  "GPU" product computed on the CPU and a fixed NF4 error written instead of
  the measured one both left every test passing. Each result now records the
  device it was computed on and a non-CUDA device is refused, and a test runs
  the measuring code against stand-in torch and bitsandbytes modules whose
  GPU path carries known errors.
- Five first-boot verdicts were tested on their passing side only (Ubuntu,
  CUDA / cuDNN / TensorRT, disk space, the JetPack version, and the
  `/proc/net/ip_tables_names` branch of the Docker 28 check). Each is now
  driven to its failing side too.
- The rule that timing never uses MAXN was held by one phrase pattern; two
  rewrites of the runbook's list passed it. Both documents now assign the
  modes in a single list, and the test holds its shape (budgeted mode first,
  default second, and only a "Not MAXN" item naming MAXN).
- The install script claimed `--isolated` made pip independent of an index a
  base sets; `--isolated` still reads `/etc/pip.conf` and a virtual
  environment's `pip.conf`. It now adds `PIP_CONFIG_FILE=/dev/null`, logs
  `pip config debug` and stops if any setting remains.
- The script and this document said the apt CMake builds bitsandbytes; the
  bases' pip-installed CMake comes first on PATH. Corrected, and the build
  log now records `cmake --version`.
- Two places called jetson-containers NVIDIA's; its images are third-party,
  published under the dustynv Docker Hub account, not on NGC.
- The plan test ran the install's torch freeze with the test's own
  interpreter, so it needed torch installed on the host; it now uses a
  stand-in interpreter.

Each new check failed on the code before this pass first (6 of the 58 tests;
the new first-boot cases pass there, because the script was right and only
untested). The review's 20 mutants (one rewritten where the code moved, plus
a variant of it) and 20 new ones on this pass's safeguards: 41 of 41 caught. Run
against the `deploy/` files, the runbook and this document of commit 10451bc
(every other file as in the working tree), the current test file passes 38 of
58; the 20 that fail are the JetPack 6.2 requirements and the two defects
above (measured 2026-10-09).

## Next steps

1. Run the first-boot check on the Orin and read what comes back (runbook
   step 1), then Docker and power mode (steps 2 and 4).
2. Build the images and bring up hil on the Orin (runbook steps 8 to 11),
   with the base image's GPU check before the VLA build.
3. Run the Shield profile and the R1 rates on the Orin (steps 12 and 13), in
   the highest budgeted mode and in the default mode, clocks locked.
4. `sitl/ros2_shield_node.py`: put the Orin's profile summary
   (`tools/profile_shield_tick.py attach`) into each run's `metrics.json`.
5. `tools/profile_shield_tick.py`: record the power mode inside the container
   from `/var/lib/nvpmodel/status` and the CPU frequency limits from sysfs,
   so the mode no longer depends on the `--note` text.
6. `sitl/ros2_vla_bridge_node.py`: let a VLA in the `vla` image publish
   `/vla/action_4d`, so a camera VLA can be the pilot on the Orin
   (`deploy/compose/docker-compose.orin.yml` lists it as pending). The VLA
   image is Ubuntu 24.04 like the companion, so the bridge can use Jazzy's
   rclpy there too.
