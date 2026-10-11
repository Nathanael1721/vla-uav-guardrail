# Runbook: the Jetson Orin (JetPack 6.2) as the hil machine, then on the drone

**For:** whoever sets up the Jetson Orin and runs the hil topology, the
configuration the grant takes every reported KPI number from.
**Date:** 2026-10-06; revised 2026-10-08 for **JetPack 6.2** (Jetson Linux /
L4T r36.4.x, Ubuntu 22.04), the release the Orin is being installed with. The
design and its reasons are in [DESIGN-topologies.md](DESIGN-topologies.md);
the files are in `deploy/`.

**Status of this procedure.** It was written before the Orin was available.
The images and compose files have not been built or run yet; the
configuration is checked by `tests/test_deploy_configs.py`. When a step does
not behave as written, fix the step here in the same commit as the fix.

## The picture

```
 DESKTOP (Windows 11 + Docker Desktop)                 JETSON ORIN (JetPack 6.2, Docker)
 +------------------------------------------+          +------------------------------------+
 | Project AirSim (optional)  UDP 9002/9003 |          |  companion containers (host net)   |
 |      |                                   |          |   MAVROS 2                         |
 | ArduPilot SITL ---UDP 14550--> mavlink-  |  cable   |   fcu_url udp://:14555@<desktop>   |
 |   (container)                  router  --+--UDP---->+      |                             |
 |                                  |       |  14555   |   mission: VLA stub -> Shield ->   |
 | Mission Planner <---UDP 14550----+       |          |            MAVLink adapter -> MAVROS|
 +------------------------------------------+          +------------------------------------+
   192.168.50.1                                         192.168.50.2
```

Only MAVLink crosses the cable (plus Project AirSim's client API when a camera
VLA runs on the Orin). Everything on the Orin is the same software that will
later fly on the drone; only `deploy/topologies/flight.env` replaces `hil.env`.

**Where things are written.** On the Orin, measurements go to
`deploy/evidence/incoming/` and runs to `demo/out/`; git ignores both. Nothing
on the Orin may write a tracked file: a modified checkout makes every later
manifest there say `-dirty`, and the KPI gate refuses those runs. Reviewed
files are copied to the desktop and committed there (step 14).

## What JetPack 6.2 gives, and what runs where

JetPack 6.2 is Jetson Linux 36.4.3 (JetPack 6.2.1 is 36.4.4 with the same
compute stack; NVIDIA's JetPack 6.2 and 6.2.1 pages, read 2026-10-08). The
host keeps only the driver, Docker and the NVIDIA container runtime; each
container brings its own user space.

| | JetPack 6.2 host | companion container (`vlaguard/companion:jazzy`) | VLA container (`vlaguard/vla:l4t-r36.4-cu128`) |
|---|---|---|---|
| L4T / Jetson Linux | 36.4.3 (`R36 (release), REVISION: 4.3`) | - (shares only the kernel) | base built for r36.4.0 |
| Ubuntu | 22.04, kernel 5.15 | 24.04 (noble) | 24.04 |
| Python | 3.10 (not used by the rail) | 3.12 | 3.12 |
| ROS 2 | none | Jazzy, MAVROS 2, Cyclone DDS, rosbag2 | none |
| CUDA | 12.6.10 (its driver is what the containers use) | none | 12.8, with the forward-compatibility libraries |
| cuDNN | 9.3.0 | none | from the base image |
| TensorRT | 10.3.0 | none | not used (OpenVLA runs in PyTorch, 4-bit with bitsandbytes) |
| PyTorch | none | none | torch 2.6 + torchvision 0.21.0 (the desktop's pair) |

## What you need

- The Jetson Orin flashed with **JetPack 6.2** (or 6.2.1), with network access
  for the first image builds (packages and base images are downloaded once).
- An Ethernet cable between the Orin and the desktop (a switch also works).
- On the desktop: Docker Desktop with the WSL 2 backend, Mission Planner, and
  this repository at the same commit as the Orin.
- About 40 GB free on the Orin for images and, for the VLA measurements, the
  model weights (OpenVLA-7B is 15 GB on disk). An NVMe drive is strongly
  preferred over the eMMC or SD card.

## 1. First boot: run the check and send it back

Right after JetPack is installed, and again after step 2, run the check and
send the whole file back. It is read-only: it installs nothing, downloads
nothing and changes no setting.

```bash
git clone <the lab repository URL> ~/vla-drone
cd ~/vla-drone
mkdir -p deploy/evidence/incoming/hil
sudo -v            # lets the check read `jetson_clocks --show`; it never asks for a password itself
bash deploy/orin/first_boot_check.sh 2>&1 | tee deploy/evidence/incoming/hil/first_boot_check.txt
```

Send `deploy/evidence/incoming/hil/first_boot_check.txt`. It holds the raw
output of `cat /etc/nv_tegra_release`, `cat /proc/device-tree/model`,
`uname`, `/etc/os-release`, the JetPack, CUDA, cuDNN and TensorRT packages,
`nvpmodel -q` and the power modes this module offers, `jetson_clocks --show`,
`ls /opt/ros` and the rclpy / ros-base / MAVROS package versions of any ROS
on the host, `docker info`, `docker compose version`,
`docker buildx version`, `nvidia-ctk --version`, memory, swap, disks,
network addresses and time sync, and ends with a VERDICTS block:

| Verdict line | Expected on a JetPack 6.2 Orin | If not |
|---|---|---|
| `l4t` | PASS `R36.4.3 = JetPack 6.2` (or `R36.4.4 = JetPack 6.2.1`) | R36.4.0 (JetPack 6.1) warns; R36.5 (JetPack 6.2.2 / 6.2.3) warns and step 8's GPU check decides; anything older fails: flash JetPack 6.2. |
| `board` | PASS, the device tree names an **Orin** | A Xavier or Nano is not the grant's hil machine; `guardrail.manifest` labels its runs `dev`. |
| `arch`, `os` | aarch64; Ubuntu 22.04 | |
| `jetpack`, `compute` | `nvidia-jetpack 6.2...`; CUDA 12.6, cuDNN 9.3, TensorRT 10.3 | A warning only: the images bring their own user space and need only the driver. |
| `power` | the current mode, this module's mode table by ID, its default, its highest mode with a power budget and its MAXN mode (on an AGX Orin 64GB: `0=MAXN ... default ID 2 = MODE_30W; highest budgeted mode ID 3 = MODE_50W; MAXN mode ID 0 = MAXN`) | A warning when `/etc/nvpmodel.conf` has no readable mode table: the IDs step 4 needs are then unknown. |
| `ros_host` | PASS: no ROS on the host | A warning naming each distribution and its rclpy version: harmless, but the rail never uses it (step 3). |
| `docker`, `compose`, `buildx`, `nvidia_runtime` | all PASS | Step 2. Without buildx, `docker build` falls back to the legacy builder, which ignores the per-Dockerfile ignore files and would send the whole checkout, `policies/keys/` included, as build context: that is a FAIL. |
| `docker28` (only when it applies) | absent | Docker 28 or newer on r36.4.3, whose kernel lacks `iptable_raw`: published ports fail. Step 2. |
| `disk` | 40 GB or more free under Docker's root | Move Docker's data root to the NVMe drive. |

**If the repository cannot be cloned yet**, paste the output of these instead:

```bash
cat /etc/nv_tegra_release; cat /proc/device-tree/model; echo
uname -srvmo; grep -E '^(ID|VERSION_ID)=' /etc/os-release
dpkg-query -W nvidia-jetpack cuda-cudart-12-6 libcudnn9-cuda-12 tensorrt nvidia-container-toolkit
nvpmodel -q; grep -E 'POWER_MODEL|PM_CONFIG' /etc/nvpmodel.conf
ls /opt/ros; dpkg-query -W 'ros-*-rclpy' 'ros-*-mavros'     # ROS on the host and its version, if any
docker info; docker compose version; docker buildx version
free -h; df -h /
```

(`dpkg-query` printing "no packages found matching" for the ROS line is the
expected answer: nothing ROS belongs on the host.)

## 2. Docker and the NVIDIA container runtime on JetPack 6.2

JetPack 6 flashed from SDK Manager does not install Docker (the SD-card image
for the Orin Nano does). If step 1 says `docker`, `compose`, `buildx` and
`nvidia_runtime` PASS, skip to step 3.

Install Docker Engine from Docker's own apt repository (it carries the
compose and buildx plugins):

```bash
sudo apt-get update && sudo apt-get install -y ca-certificates curl nvidia-container
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

Then, **on JetPack 6.2 (r36.4.3)**, install Docker 27.5.1 and hold it. Docker
28 needs the kernel's `iptable_raw`, which the r36.4.3 kernel does not have,
and published ports then fail; JetPack 6.2.1 (r36.4.4) fixed this. The Orin's
compose files publish no port (host networking or none), but the pin keeps
Docker usable for anything else:

```bash
apt-cache madison docker-ce | grep 27.5.1     # the exact version string, expected below
V=5:27.5.1-1~ubuntu.22.04~jammy
sudo apt-get install -y docker-ce=$V docker-ce-cli=$V containerd.io docker-buildx-plugin docker-compose-plugin
sudo apt-mark hold docker-ce docker-ce-cli
```

On JetPack 6.2.1 or later, install the current release instead:
`sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin`.

Register the NVIDIA runtime with Docker and let your user run Docker:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
sudo usermod -aG docker $USER      # then log out and back in
docker info | grep -i -E "server version|runtimes|default runtime"
```

`runtimes` must list `nvidia`. Leave the default runtime as it is: the Orin
compose file names the runtime of every service itself, `runc` for the
companion containers (the safety path takes no GPU library) and `nvidia` for
the VLA bench. jetson-containers' own set-up makes `nvidia` the default; that
is fine for this repository too.

Run step 1's check again and send the new file.

## 3. ROS 2: Jazzy in the container, nothing on the host

JetPack 6.2 is Ubuntu 22.04, whose native ROS 2 is Humble. This stack runs
**ROS 2 Jazzy inside the companion container** on an Ubuntu 24.04 user space
instead, and nothing ROS is installed on the Orin's own Ubuntu. The reasons
(DESIGN-topologies.md, decision 2):

- The grant: "Python 3.11+ as a ROS 2 node (rclpy)" (Safety Shield p.1).
  rclpy runs on its distribution's own Python: Jazzy's is 3.12, Humble's
  3.10, below that floor. The image's build stops on a base below 3.11.
- The companion container uses no GPU, so it needs nothing from L4T; it shares
  only the kernel, and an Ubuntu 24.04 user space runs on L4T's 5.15 kernel.
  jetson-containers (the Jetson AI Lab container project, published under
  the dustynv Docker Hub account; not an NGC image) publishes Jazzy for
  r36.4 the same way (`dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04`).
- The desktop's dev rail flew on Jazzy, and ROS 2 does not support mixing
  distributions in one DDS graph.

MAVROS 2 is installed in the image from the ROS apt repository
(`ros-jazzy-mavros`, `ros-jazzy-mavros-extras`, plus the GeographicLib geoid
datasets MAVROS needs at start-up). After step 8, record what the image holds:

```bash
docker run --rm vlaguard/companion:jazzy versions | tee deploy/evidence/incoming/hil/companion_versions.txt
```

It prints the image's Ubuntu, `ros_distro=jazzy`, the Python rclpy and the
Shield node run on, and the installed versions of rclpy, MAVROS, Cyclone DDS,
rosbag2's MCAP storage, mavlink-routerd and the guardrail's Python packages.

## 4. Power mode and clocks for every timing measurement

Every timing number (the Shield profile, step 12; the VLA rates, step 13; the
KPI missions, step 14) depends on the power mode and on dynamic clock
scaling. Fix both, and record them with each measurement.

1. **Find the mode IDs on this module.** Step 1's `power` verdict lists
   them. They differ between modules (Jetson Linux 36.4.3 power guide):

   | Module | Highest mode with a power budget | Default mode | MAXN-class mode (no budget) |
   |---|---|---|---|
   | AGX Orin 64GB | 3 = 50W | 2 = 30W | 0 = MAXN |
   | AGX Orin 32GB | 3 = 40W | 2 = 30W | 0 = MAXN |
   | Orin NX 16GB / 8GB, flashed for JetPack 6.2's Super modes | 4 = 40W | 4 = 40W | 0 = MAXN_SUPER |
   | Orin NX 16GB / 8GB, standard flash | 3 = 25W / 20W | 2 = 15W | 0 = MAXN |
   | Orin Nano 8GB / 4GB, flashed for Super modes | 1 = 25W | 1 = 25W | 2 = MAXN_SUPER |
   | Orin Nano 8GB / 4GB, standard flash | 0 = 15W / 10W | 0 = 15W / 10W | none |

   The Super modes exist only on an Orin NX or Nano flashed with a
   `-super` configuration; the module's own table is what counts:

   ```bash
   grep -E 'POWER_MODEL|PM_CONFIG' /etc/nvpmodel.conf
   nvpmodel -q
   ```

2. **Set the mode** by its ID. If nvpmodel asks for a reboot, reboot and check
   again; the mode persists across reboots:

   ```bash
   sudo nvpmodel -m <ID>
   nvpmodel -q
   cat /sys/devices/system/cpu/online     # the cores this mode keeps on
   ```

3. **Lock the clocks** at the mode's maximum for the session (this does not
   survive a reboot) and record the state:

   ```bash
   sudo jetson_clocks --store ~/l4t_clocks_before.conf
   sudo jetson_clocks
   S=deploy/evidence/incoming/hil/power_$(date +%F_%H%M).txt
   { nvpmodel -q; sudo jetson_clocks --show; } | tee "$S"
   ```

4. **Log temperatures and clocks** while measuring, to show nothing
   throttled:

   ```bash
   sudo tegrastats --interval 1000 --logfile deploy/evidence/incoming/hil/tegrastats_$(date +%F_%H%M).log &
   # ... the measurement ...
   sudo tegrastats --stop
   ```

5. **Afterwards**, `sudo jetson_clocks --restore ~/l4t_clocks_before.conf`.

Which modes:

- **The highest mode with a power budget** (step 1's "highest budgeted
  mode"; 50W, ID 3, on an AGX Orin 64GB), clocks locked: the Shield profile,
  the VLA rates and the KPI missions. It is the fastest mode the module holds
  within a stated budget, so long runs stay repeatable.
- **The default mode** (`PM_CONFIG DEFAULT` in `/etc/nvpmodel.conf`; 30W on
  an AGX Orin), clocks locked: the Shield profile and the VLA rates once more,
  as the conservative figure until the drone's power budget is known.
- **Not MAXN** (or MAXN_SUPER) for these. NVIDIA's power guide for Jetson
  Linux 36.4.3 calls MAXN "an unconstrained power mode" and "an experimental
  mode", advises against heavy workloads for prolonged periods in it, and
  hardware throttling engages once the module exceeds its TDP, so a long run
  can change speed part-way. A MAXN figure, if wanted as a ceiling, is a short
  separate run with tegrastats showing no throttling, and is labelled as such.

Quote a timing budget for flight only from the mode the drone will fly in
(step 16). Put the mode name, its ID and "jetson_clocks on" in each profile's
`--note`.

## 5. Network

Use a direct cable and a private subnet. The topology file assumes
`192.168.50.1` for the desktop and `192.168.50.2` for the Orin. If you use
other addresses, change `DESKTOP_IP`, `ORIN_IP`, `MAVROS_HOST`, the desktop
address inside `FCU_URL` and the two `VLAGUARD_DDS_PEER_*` values in
`deploy/topologies/hil.env` together (the tests check that they agree), commit
the change on the desktop and pull it on the Orin.

On the Orin (the connection name may differ; `nmcli con show` lists them):

```bash
sudo nmcli con mod "Wired connection 1" ipv4.method manual ipv4.addresses 192.168.50.2/24
sudo nmcli con up "Wired connection 1"
```

On the desktop: Settings > Network > Ethernet > IP assignment > Manual,
`192.168.50.1`, mask `255.255.255.0`, no gateway. Then:

```bash
ping 192.168.50.1      # from the Orin
ping 192.168.50.2      # from the desktop
```

Windows firewall: allow inbound UDP 14550 for Mission Planner (Windows asks the
first time Mission Planner listens; answer for private networks), and, only if
a camera VLA on the Orin will read Project AirSim frames, inbound TCP 8989 and
8990 from `192.168.50.0/24`. The Orin's Ubuntu has no firewall enabled by
default; MAVROS listens on UDP 14555 there.

A quick check that UDP reaches the Orin, before any container exists:

```bash
nc -ul 14555                                   # on the Orin, leave it running
echo hello | nc -u -w1 192.168.50.2 14555      # in WSL on the desktop
```

## 6. Time synchronisation

The Shield's tick timing and every KPI are measured on one host's monotonic
clock, so a clock offset between the two machines does not change a KPI. It
does matter for lining up logs from both machines, so keep the offset small
and record it.

- **Both machines online:** leave Windows Time on its default server and let
  the Orin's `chrony` (or `systemd-timesyncd`) use its default pool.
- **Cable only, no internet on the Orin:** make the desktop the server. In an
  administrator PowerShell:

  ```powershell
  reg add HKLM\SYSTEM\CurrentControlSet\Services\W32Time\TimeProviders\NtpServer /v Enabled /t REG_DWORD /d 1 /f
  w32tm /config /reliable:yes /update
  net stop w32time; net start w32time
  New-NetFirewallRule -DisplayName "NTP from Orin" -Direction Inbound -Protocol UDP -LocalPort 123 -RemoteAddress 192.168.50.0/24 -Action Allow
  ```

  and on the Orin put `server 192.168.50.1 iburst` in `/etc/chrony/chrony.conf`
  (install `chrony` if missing) and `sudo systemctl restart chrony`.

Record `chronyc tracking` at the start and end of each session:
`chronyc tracking >> deploy/evidence/incoming/hil/chrony_$(date +%F).txt`.

## 7. The code and the weights on the Orin

```bash
cd ~/vla-drone                # cloned in step 1
git fetch && git checkout <the commit the desktop is on>
git rev-parse HEAD            # must print the same commit as on the desktop
git status --porcelain        # must print nothing, now and before every KPI run
```

The containers mount this checkout read-only, and every run's manifest records
its commit. A modified checkout records `-dirty`, which the KPI gate refuses, so
never edit files on the Orin: change them on the desktop, commit, and pull.
Everything this runbook writes on the Orin goes to `deploy/evidence/incoming/`
or `demo/out/`, which git ignores, so it does not dirty the checkout.

Copy what git does not carry, from the desktop, only if you will run step 13:
`D:\models\openvla-7b`, `D:\models\aerialvla-lora\aero_vla` and
`D:\models\aerialvla-ft\run2\epoch1` into `/opt/models/` with the same
sub-paths (`VLAGUARD_MODEL_ROOT` in `hil.env`), and `models/vla_policy_v2.pt`
(the behaviour-cloned policy, 2 MB) into `~/vla-drone/models/`.

## 8. Build the images

BuildKit is needed for the per-Dockerfile ignore files (it is the default from
Docker 23, through the buildx plugin step 1 checks). Run from the repository
root.

**On the Orin.** First check the VLA base image's GPU stack on this Orin
before building on it (this pulls the base, about 3.8 GB, and checks torch
and the GPU only):

```bash
docker run --rm --runtime nvidia -v "$PWD/deploy/orin:/chk:ro" \
    dustynv/torchvision:0.21.0-r36.4.0-cu128-24.04 python3 /chk/gpu_check.py --skip-bnb \
    | tee deploy/evidence/incoming/hil/gpu_check_base.json
```

`"verdict": "ok"` means torch 2.6 sees the Orin's GPU (compute capability
8.7) and a matrix product on it matches the CPU's. Then build both images
natively (arm64):

```bash
docker build -f deploy/docker/companion.Dockerfile -t vlaguard/companion:jazzy .
docker build -f deploy/docker/vla.Dockerfile --progress=plain \
    --build-arg BASE_IMAGE=dustynv/torchvision:0.21.0-r36.4.0-cu128-24.04 \
    -t vlaguard/vla:l4t-r36.4-cu128 . 2>&1 | tee deploy/evidence/incoming/hil/vla_build.log
```

The VLA build compiles bitsandbytes (the pinned version) for the Orin's
sm_87 GPU, because PyPI's aarch64 wheels carry no sm_87 code (they import,
then the first 4-bit kernel fails). It follows bitsandbytes' own guide: CMake
configures and compiles the CUDA library with `COMPUTE_CAPABILITY=87`, then
pip packages the source tree (pip alone compiles nothing for bitsandbytes).
This is the slow part of the build; it runs 4 compiler jobs in parallel so
that it fits in a smaller Orin's memory. The build stops if the library it
built lacks sm_87, if anything tried to replace the base image's torch, and
if pip still reports a setting with the install's options. The pipe through
`tee` hides `docker build`'s exit status, so read the end of the log for
whether it finished. In `vla_build.log`, check two things once and send the
file back:

- `pip config debug` (the config files the base carries), then `vla-install:
  pip reads no config file and no PIP_* variable`. Every pip call runs
  `--isolated` with `PIP_CONFIG_FILE=/dev/null` and names PyPI, so neither the
  base's `PIP_INDEX_URL` nor a `pip.conf` decides what is installed.
- `cmake --version` just before the bitsandbytes configure step: the CMake
  that ran. The bases put a pip-installed CMake ahead of the apt one on PATH;
  either meets the source's floor, 3.22.1.

Then check the built image on the GPU, 4-bit kernels included:

```bash
docker compose --env-file deploy/topologies/hil.env \
    -f deploy/compose/docker-compose.orin.yml --profile vla-bench \
    run --rm -T --entrypoint python3 vla-bench deploy/orin/gpu_check.py \
    | tee deploy/evidence/incoming/hil/gpu_check.json
```

`"verdict": "ok"` adds that an NF4 4-bit round trip through bitsandbytes'
CUDA kernels quantized (relative error above 0) and returned the data
(relative error below 0.2). Each result names the device it was computed on
(`matmul_device`, `nf4_device`); anything but a CUDA device is refused. **If the base check fails** with "torch sees no
GPU" (CUDA 12.8's forward compatibility not working on this L4T), use the
fallback base, JetPack 6.2's own CUDA 12.6 on Ubuntu 22.04:

```bash
docker build -f deploy/docker/vla.Dockerfile \
    --build-arg BASE_IMAGE=dustynv/l4t-pytorch:r36.4.0 -t vlaguard/vla:l4t-r36.4-cu126 .
```

and set `VLA_BASE_IMAGE=dustynv/l4t-pytorch:r36.4.0` and
`VLAGUARD_VLA_IMAGE=vlaguard/vla:l4t-r36.4-cu126` in both `hil.env` and
`flight.env` (on the desktop, committed). That base runs Python 3.10, so the
guardrail code in the VLA image then runs below the grant's 3.11, and it
carries torch 2.4.0 / torchvision 0.19.0 rather than the desktop's 2.6.0 /
0.21.0: record both as deviations beside the R1 rates. The companion image is
not affected.

**On the desktop:**

```bash
docker build -f deploy/docker/companion.Dockerfile -t vlaguard/companion:jazzy .
docker build -f deploy/docker/sitl.Dockerfile -t vlaguard/sitl:copter-4.5.7 .
```

The two `vlaguard/companion:jazzy` builds come from the same file for two
architectures, with no build arguments. (`docker buildx build --platform
linux/amd64,linux/arm64 ...` with a registry produces both from one machine,
if you prefer.) The SITL build checks out ArduPilot Copter-4.5.7 and stops if
the tag does not resolve to the pinned commit.

Record the image IDs on both machines, and the companion's versions (step 3):

```bash
for i in vlaguard/companion:jazzy vlaguard/vla:l4t-r36.4-cu128; do
  docker image inspect --format "$i {{.Id}} {{.Architecture}}" "$i"
done >> deploy/evidence/incoming/hil/images.txt
docker run --rm vlaguard/companion:jazzy versions > deploy/evidence/incoming/hil/companion_versions.txt
```

## 9. Start the desktop side

```bash
docker compose --env-file deploy/topologies/hil.env \
    -f deploy/compose/docker-compose.hil-desktop.yml up -d
docker compose --env-file deploy/topologies/hil.env \
    -f deploy/compose/docker-compose.hil-desktop.yml logs router sitl
```

The router container runs the native rail's own `sitl/start_router.sh`; its
log shows `router: SITL udp:14550 -> MAVROS 192.168.50.2:14555 + GCS ...` and
mavlink-router opening the three endpoints. SITL starts with its EEPROM wiped
and the GeoFence backstop loaded, as `sitl/start_sitl.sh` does. Start Mission
Planner and connect with UDP on port 14550: it should show the SITL vehicle at
its home position. Mission Planner is then on the same MAVLink stream that
MAVROS on the Orin gets, through mavlink-router.

For the perception rail (Project AirSim physics and camera under ArduPilot),
read [DESIGN-projectairsim-ardupilot.md](DESIGN-projectairsim-ardupilot.md)
first. In a copy of `hil.env` set:

```bash
SITL_MODEL=airsim-copter
SITL_HOME=35.6895,139.6917,40.0,0.0       # python3 demo/pas_ardupilot/rail.py home (the CityLife scene)
SITL_AIRFRAME_PARM=/opt/vlaguard/demo/pas_ardupilot/citylife-quad.param
```

The SITL container refuses `airsim-copter` with the default CMAC home or
without an airframe file. Load the scene in Project AirSim first, then start
SITL (a scene reload resets the simulator clock). The robot configuration's
`ardupilot-api` controller must send its sensor packets to the port the router
container publishes (UDP 9003 on the desktop) and listen on UDP 9002 for the
motor outputs SITL sends to `AIRSIM_HOST`. This container variant has not been
run; the native WSL rail in that document is the tested reference.

## 10. Start the Orin side

```bash
cd ~/vla-drone
docker compose --env-file deploy/topologies/hil.env \
    -f deploy/compose/docker-compose.orin.yml up -d mavros
docker compose --env-file deploy/topologies/hil.env \
    -f deploy/compose/docker-compose.orin.yml exec mavros \
    bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash && ros2 topic echo --once /mavros/state'
docker compose --env-file deploy/topologies/hil.env \
    -f deploy/compose/docker-compose.orin.yml run --rm -T facts \
    > deploy/evidence/incoming/hil/host_facts.json
grep -A3 '"is_orin"' deploy/evidence/incoming/hil/host_facts.json
```

`connected: true` means MAVLink flows from SITL on the desktop, through
mavlink-router and the cable, into MAVROS on the Orin. `connected: false`
means it does not: check step 5 (`MAVROS_HOST` must be the Orin's address) and
the router log.

`"is_orin": true` with `"missing": []` means the containers can read the
evidence a run needs to be labelled hil: the device tree's board name and the
L4T release (`R36.4.3` on JetPack 6.2). If `missing` says the device model is
`None`, the container cannot read `/proc/device-tree/model`; the Orin compose
file lifts Docker's masking of it with `security_opt: systempaths=unconfined`,
which needs Docker 19.03 or newer. (`-T` keeps a terminal from writing CRLF
into the file.)

## 11. A first mission

```bash
docker compose --env-file deploy/topologies/hil.env \
    -f deploy/compose/docker-compose.orin.yml run --rm mission \
    mission --shield on --tag hil_smoke_01
```

The mission container does what `sitl/run_ros2_demo.sh` does on the desktop:
it records the grant's topics with rosbag2, starts the launch file (the VLA
stub's "fly to the northeast pad", the MAVLink adapter and the Shield node),
and packs the replay bundle. The Shield node switches SITL to GUIDED, arms,
takes off and flies the mission; Mission Planner should show it live. The
run's files land in `~/vla-drone/demo/out/hil_smoke_01/` on the Orin. Restart
SITL between missions (`docker compose ... restart sitl` on the desktop): the
vehicle stays where it landed, and the restart also wipes the fence zones the
adapter uploaded.

**Label.** The Shield node decides the label from evidence, not from the
topology file. Check `demo/out/hil_smoke_01/manifest.json`: `"topology": "hil"`
means the evidence held (an Orin, SITL on a remote desktop, a connected
MAVROS chain, the autopilot's version). `metrics.json` also records
`mavros_on`, which should be `shield_host` (the mission container scans the
Orin's processes with `pid: host` and sees the mavros container). If the label
says anything else, `metrics.json` has a `topology_refused` entry naming what
was missing: usually the device tree (step 10), an ArduPilot SITL running on
the Orin itself, or an `FCU_URL` without the desktop's address.

## 12. Profile the Shield on the Orin

This is the measurement the grant asks for by name ("Orin profiling") against
the 100 ms tick and 5 ms query budgets. Set the power mode and lock the
clocks first (step 4); nvpmodel is not visible inside the containers, so the
mode goes into `--note`, and the mode file of step 4 sits beside the profile.

```bash
nvpmodel -q                      # on the Orin itself; then step 4's jetson_clocks
docker compose --env-file deploy/topologies/hil.env \
    -f deploy/compose/docker-compose.orin.yml run --rm profile \
    profile --cpus 1-3 --note "Orin <module>, JetPack 6.2, nvpmodel <mode name> (<ID>), jetson_clocks on, nothing else running"
```

It replays the same 1,301 real flight ticks the desktop baseline used
(`deploy/evidence/shield_tick_pack.json`) and writes
`deploy/evidence/incoming/hil/shield_tick_profile.json`. It refuses the hil
label unless guardrail.manifest's Orin rule passes inside the container, and
it fails, by design, if the 50-rule arm never reached a fence or if the
host-speed reference moved by more than 1.5 times during the run (then re-run
with nothing else on the Orin). `--cpus` pins it to fixed cores. The Orin's
Arm cores have one hardware thread each; lower power modes switch cores off,
so pin to cores that stay on in every mode you measure (compare
`/sys/devices/system/cpu/online` in each mode; cores 1-3 normally are, and
core 0 takes most interrupts), and use the same ones every time. Run it in
the highest budgeted mode and in the default mode (step 4), renaming the
first file before the second run.

Compare with `deploy/evidence/dev/shield_tick_profile.json` (desktop
performance cores) and `shield_tick_profile-ecores.json` (desktop efficiency
cores). The `host_speed` block in each file is the same reference workload on
each machine. The headline is `_check` p99 against 5 ms and `filter` p99
against 100 ms at 50 rules and a 5 s horizon at 0.1 s, with the maximum and
the count of ticks over budget beside each.

## 13. VLA inference rate on the Orin (protocol R1)

Every backend is measured in the `vla` image, one at a time, with nothing else
using the GPU, after step 8's `gpu_check.json` said ok, in the power mode of
step 4 with the clocks locked and tegrastats logging:

```bash
for b in stub bc_v3 openvla_7b_4bit aerialvla_lora aerialvla_ft_run2; do
  VLA_BACKEND=$b docker compose --env-file deploy/topologies/hil.env \
      -f deploy/compose/docker-compose.orin.yml --profile vla-bench run --rm vla-bench
done
```

Each writes `deploy/evidence/incoming/hil/vla_rate_<backend>.json`. Keep the
same power mode for every backend, and keep step 4's mode file with them. For
OpenVLA, the default is the flight script's default prompt; to time the
longer prompt with the Constraint Summary Pack as well, run
`docker compose ... --profile vla-bench run --rm vla-bench measure --backend
openvla_7b_4bit --allow-gpu --model-root /models --topology hil --csp on`.

## 14. Running the KPI campaign from the desktop

Once steps 10 to 12 work and step 11's manifest says `hil`, the campaign is a
loop run from the desktop that starts one mission at a time on the Orin and
resets SITL between missions:

```bash
for n in $(seq -w 1 20); do
  ssh orin@192.168.50.2 "cd ~/vla-drone && docker compose --env-file deploy/topologies/hil.env \
      -f deploy/compose/docker-compose.orin.yml run --rm mission mission --shield on --tag hil_kpi_$n"
  docker compose --env-file deploy/topologies/hil.env \
      -f deploy/compose/docker-compose.hil-desktop.yml restart sitl
  sleep 20                       # EKF settles before the next arming
done
rsync -a orin@192.168.50.2:~/vla-drone/demo/out/hil_kpi_* demo/out/
rsync -a orin@192.168.50.2:~/vla-drone/deploy/evidence/incoming/hil/ deploy/evidence/hil/
python tools/kpi_report.py --runs "demo/out/hil_kpi_*" --no-sweep --out demo/out/hil_kpi_report
python tools/vla_backend_table.py table        # moves the Orin's R1 rates into the Orin column
```

(`orin` is a placeholder for the Orin's user name.) Lock the clocks on the
Orin (step 4) before the loop. Review and commit `deploy/evidence/hil/` on the
desktop; the Orin then pulls it like any other commit, and since its own new
measurements go to `incoming/`, they never overwrite a tracked file.
`sim_speedup` is fixed at 1 in the SITL image, and every run reads it back
from SITL, as the grant requires for a KPI-bearing run. The scenario library
of the stress harness (`experiments/sweep_scenarios.py`) drives this rail once
the Shield node accepts a scenario's start, target, pilot and seed; until then
the campaign is the node's own mission, repeated.

## 15. Evidence each hil run must carry

| Evidence | Where it comes from | Where it goes |
|---|---|---|
| Six-field manifest (code revision, model hash, policy hash, seed, `sim_speedup` 1.0, topology `hil`) | the Shield node | `demo/out/<tag>/manifest.json` |
| ROS distribution, MAVROS node seen, `fcu_connected: true`, ArduPilot version, the Orin's board and L4T release, the VLA node's host, the `fcu_url` and the route to the desktop, the MAVLink processes on the Orin (`local_processes`, `mavros_on`) | the Shield node's `hil_evidence` | `metrics.json` |
| The Orin as set up: L4T, JetPack, CUDA / cuDNN / TensorRT, power modes, Docker and runtimes | step 1 | `deploy/evidence/hil/first_boot_check.txt` |
| The Orin as the containers see it | step 10 (`facts`) | `deploy/evidence/hil/host_facts.json` |
| What the companion image holds (ROS 2 distribution, rclpy's Python, MAVROS) | step 8 (`versions`) | `deploy/evidence/hil/companion_versions.txt` |
| The VLA image works on this GPU (4-bit included) | step 8 (`gpu_check.py`) | `deploy/evidence/hil/gpu_check.json` |
| Power mode and locked clocks of each session, temperatures during it | step 4 | `deploy/evidence/hil/power_<date>.txt`, `tegrastats_<date>.log` |
| Shield time per tick on this Orin | step 12 | `deploy/evidence/hil/shield_tick_profile.json`; its summary (`python tools/profile_shield_tick.py attach <file>`) beside each run |
| Same commit on both hosts, clean tree | step 7 | the manifest's `code_revision` (no `-dirty`) |
| Same image on both hosts | step 8 | `deploy/evidence/hil/images.txt` |
| Clock offset | step 6 | `deploy/evidence/hil/chrony_<date>.txt` |
| Router endpoints | step 9 | the router log, saved beside the session's evidence |

## 16. Moving to the drone (flight)

Nothing is rebuilt. The same `vlaguard/companion:jazzy` arm64 image and the
same `deploy/compose/docker-compose.orin.yml` run with
`deploy/topologies/flight.env` instead of `hil.env`, plus the small override
`docker-compose.orin.flight.yml` that maps the autopilot's serial device.

1. **Wire the autopilot.** Connect a TELEM port of the autopilot (e.g. TELEM2)
   to the Orin, by USB or by a UART on the carrier board. In Mission Planner,
   connected to the autopilot directly, set the port to MAVLink 2 at a high
   rate, e.g. `SERIAL2_PROTOCOL = 2` and `SERIAL2_BAUD = 921` for TELEM2.
2. **Name the link.** In a copy of `flight.env`, set `FCU_URL`, e.g.
   `serial:///dev/ttyACM0:115200` for USB or `serial:///dev/ttyTHS1:921600`
   for a UART (the device name depends on the carrier board: `ls /dev/tty*`
   before and after plugging in), and set `FCU_DEVICE` to the same device.
   With `FCU_URL` empty the entrypoint refuses to start MAVROS; with
   `FCU_DEVICE` empty the override refuses to start at all. (An autopilot
   reached over Ethernet needs no device: set a `udp://` `FCU_URL` and leave
   the override out.)
3. **Bring up MAVROS only, propellers off:**

   ```bash
   F="--env-file <your flight.env> -f deploy/compose/docker-compose.orin.yml \
      -f deploy/compose/docker-compose.orin.flight.yml"
   docker compose $F up -d mavros
   docker compose $F exec mavros \
       bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash && ros2 topic echo --once /mavros/state'
   ```

   Expect `connected: true` and the autopilot's mode. Mission Planner keeps its
   own link through the telemetry radio.
4. **Profile the Shield on the drone's Orin** (step 12 with `flight.env`), in
   the power mode the drone will fly in, with the clocks set as they will be
   in flight. The Orin may be powered differently on the airframe than on the
   bench, and a timing budget is claimed for flight only from this profile.
5. **Keep the autopilot's own protections on.** ArduPilot's geofence
   (`FENCE_ENABLE`, `FENCE_ACTION`), RC failsafe and a safety pilot with a mode
   switch stay in place: the grant names the autopilot's GeoFence as the
   ultimate backstop that the Shield pre-empts, not replaces.
6. **The mission.** The Shield node's mission today is the SITL test mission.
   A mission for a real site is separate work, reviewed with the PI. Only then,
   with the bench checks above done, set `VLAGUARD_ALLOW_FLIGHT_MISSION=yes` in
   your flight file; without it the entrypoint refuses to start the arming
   mission in the flight topology.
7. **Camera.** In flight the VLA's frames come from the onboard camera instead
   of Project AirSim; that frame source belongs to the VLA bridge node
   (DESIGN-topologies.md, next steps).

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `connected: false` in `/mavros/state` | No MAVLink reaching the Orin: wrong `MAVROS_HOST`, cable or address (step 5), or the router container not running. |
| `manifest.json` says `dev` on the Orin | Read `topology_refused` in `metrics.json`. Usually the device tree is not readable in the container (step 10, `facts`), a SITL process runs on the Orin itself (`local_processes` in `metrics.json` names it; stop it), or `FCU_URL` lost the desktop's address. |
| `manifest.json` has `-dirty` in `code_revision` | Something wrote a tracked file on the Orin. `git status --porcelain` names it; restore it with `git checkout -- <file>`, and keep outputs in `deploy/evidence/incoming/` and `demo/out/`. |
| The companion build stops with "below the grant floor 3.11" | The base image is not ROS 2 Jazzy on Ubuntu 24.04 (for example a Humble base on 22.04). Use the default `ros:jazzy-ros-base-noble`; a Humble build needs the PI's written acceptance of a Python 3.10 Shield first (DESIGN-topologies.md, decision 2). |
| The VLA build stops with "carries no sm_87 code" or "cuobjdump not found" | The bitsandbytes source build did not target the Orin, or the base has no CUDA toolkit. Use the base named in `hil.env`; do not install bitsandbytes from PyPI on the Orin. |
| The VLA build stops in the bitsandbytes CMake step | CMake's message says why; usually no `nvcc` at `/usr/local/cuda/bin/nvcc` (a base without the CUDA toolkit). Use the base named in `hil.env`. |
| The VLA build stops with "no libbitsandbytes_cuda*.so" | pip packaged a bitsandbytes source tree that holds no CUDA library: the CMake configure and compile steps did not run before it (pip does not compile one for bitsandbytes; its `pyproject.toml` sets `wheel.cmake = false`). |
| The VLA build stops with "pip still reads settings" | pip, run with `--isolated` and `PIP_CONFIG_FILE=/dev/null`, still reported a setting (printed after the message). The `pip config debug` lines above it show where it comes from; send the log back rather than editing the base. |
| The VLA build stops with "changed the base image's torch stack" | A requirement pulled in another torch or torchvision. Read the diff it printed; the Jetson build must stay. |
| `gpu_check.py`: "torch sees no GPU" | The container was not started with the nvidia runtime (`runtime: nvidia` is set only on `vla-bench`), or CUDA 12.8's forward compatibility does not work on this L4T: use the fallback base (step 8). |
| `gpu_check.py`: "4-bit kernels fail ... sm_87" | A bitsandbytes without Orin code got into the image; rebuild it (step 8). |
| `apt-get install nvidia-container` reports unmet dependencies on `nvidia-container-toolkit-base` | Two repositories offer the container toolkit at different versions. `apt-cache policy nvidia-container nvidia-container-toolkit-base` shows which; install the versions JetPack's own repository (`repo.download.nvidia.com/jetson`) carries. |
| Containers that publish a port fail with an iptables "raw" table error | Docker 28 or newer on JetPack 6.2 (r36.4.3). Pin Docker 27.5.1 (step 2) or move to JetPack 6.2.1. The Orin compose files publish no ports. |
| MAVROS exits at start-up mentioning GeographicLib | The image was built without the geoid datasets; rebuild the companion image (the Dockerfile installs them). |
| MAVROS connected but no position | ArduPilot streams nothing until a stream rate is requested; the Shield node requests it at bring-up. |
| `manifest.json` says `unversioned` | git could not read the mounted checkout. The image sets `safe.directory`; check that the checkout is a git clone and is mounted at `/opt/vlaguard`. |
| A build sends gigabytes of context | The legacy builder is in use, so the per-Dockerfile ignore file is not read. Install the buildx plugin (step 2), or use `DOCKER_BUILDKIT=1`. |
| `bash\r: No such file` in a container | The checkout converted line endings. Clone with `git config --global core.autocrlf false` on Windows. |
| `profile` refuses `hil` | The container is not seen as an Orin: run `facts` (step 10) and read `missing`. |
| `profile` fails with "host-speed reference moved" | Something else ran during the profile, or the clocks were not locked (step 4); re-run with nothing else on the host. |
| Timings differ between sessions | A different power mode, or clocks not locked: compare the sessions' `power_<date>.txt` files (step 4). |
