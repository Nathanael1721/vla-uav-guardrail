# Runbook: the Jetson Orin as the hil machine, then on the drone

**For:** whoever sets up the Jetson Orin and runs the hil topology, the
configuration the grant takes every reported KPI number from.
**Date:** 2026-10-06. The design and its reasons are in
[DESIGN-topologies.md](DESIGN-topologies.md); the files are in `deploy/`.

**Status of this procedure.** It was written before the Orin was available.
The images and compose files have not been built or run yet; the configuration
is checked by `tests/test_deploy_configs.py`. When a step does not behave as
written, fix the step here in the same commit as the fix.

## The picture

```
 DESKTOP (Windows 11 + Docker Desktop)                 JETSON ORIN (JetPack, Docker)
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
files are copied to the desktop and committed there (step 12).

## What you need

- The Jetson Orin with JetPack installed and network access for the first
  image build (packages and base images are downloaded once).
- An Ethernet cable between the Orin and the desktop (a switch also works).
- On the desktop: Docker Desktop with the WSL 2 backend, Mission Planner, and
  this repository at the same commit as the Orin.
- About 25 GB free on the Orin for images and, for the VLA measurements, the
  model weights (OpenVLA-7B is 15 GB on disk). An NVMe drive is strongly
  preferred over the eMMC or SD card.

## 1. Check the Orin

```bash
cd ~/vla-drone && mkdir -p deploy/evidence/incoming/hil      # after step 5's clone
{
cat /etc/nv_tegra_release           # e.g. "# R36 (release), REVISION: 4.0, ..."  -> L4T R36.4.0
cat /proc/device-tree/model; echo   # e.g. "NVIDIA Jetson AGX Orin Developer Kit"
dpkg-query --show nvidia-jetpack    # JetPack version, if the meta-package is installed
sudo nvpmodel -q                    # power mode
free -h; df -h /
docker info | grep -i -E "server version|runtimes|default runtime"
} | tee deploy/evidence/incoming/hil/orin_check.txt
```

What to look for:

| Check | Expected | If not |
|---|---|---|
| Device model | names an **Orin** | A Xavier or Nano is not the grant's hil machine; `guardrail.manifest` labels its runs `dev`. |
| L4T release | R36.x (JetPack 6.x) | Another release needs a VLA base image built for it (step 6); the companion image does not care. |
| Docker runtimes | `nvidia` listed | Install NVIDIA Container Toolkit from the JetPack repository (`sudo apt install nvidia-container-toolkit`), then restart Docker. |
| Docker version | 19.03 or newer | The containers that gather evidence use `--security-opt systempaths=unconfined`. |
| Memory | 8 GB or more for the 4-bit VLA measurements | The Shield and MAVROS fit in far less; record the model and memory either way. |

## 2. ROS 2 distribution: nothing to install on the Orin

ROS 2 runs inside the companion container (ROS 2 Jazzy, Ubuntu 24.04,
Python 3.12), not on the Orin's own Ubuntu. Reasons (DESIGN-topologies.md,
decision 2): the grant asks for the Shield as an rclpy node on Python 3.11+,
which Jazzy gives and Humble does not; the stored dev-rail runs used Jazzy; and
the companion container needs no GPU, so it does not have to match JetPack's
Ubuntu 22.04. Do not install ROS on the Orin host; a second ROS graph on the
host would only cause confusion about which one is running.

## 3. Network

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

## 4. Time synchronisation

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

## 5. The code and the weights on the Orin

```bash
git clone <the lab repository URL> ~/vla-drone
cd ~/vla-drone
git checkout <the commit the desktop is on>
git rev-parse HEAD            # must print the same commit as on the desktop
git status --porcelain        # must print nothing, now and before every KPI run
```

The containers mount this checkout read-only, and every run's manifest records
its commit. A modified checkout records `-dirty`, which the KPI gate refuses, so
never edit files on the Orin: change them on the desktop, commit, and pull.
Everything this runbook writes on the Orin goes to `deploy/evidence/incoming/`
or `demo/out/`, which git ignores, so it does not dirty the checkout.

Copy what git does not carry, from the desktop, only if you will run step 11:
`D:\models\openvla-7b`, `D:\models\aerialvla-lora\aero_vla` and
`D:\models\aerialvla-ft\run2\epoch1` into `/opt/models/` with the same
sub-paths (`VLAGUARD_MODEL_ROOT` in `hil.env`), and `models/vla_policy_v2.pt`
(the behaviour-cloned policy, 2 MB) into `~/vla-drone/models/`.

## 6. Build the images

BuildKit is needed for the per-Dockerfile ignore files (it is the default from
Docker 23). Run from the repository root.

On the Orin (native arm64 builds):

```bash
docker build -f deploy/docker/companion.Dockerfile -t vlaguard/companion:jazzy .
# for the R1 rate measurements (every backend, the CPU ones included); the
# base tag must match the Orin's L4T release (step 1):
docker build -f deploy/docker/vla.Dockerfile \
    --build-arg BASE_IMAGE=dustynv/l4t-pytorch:r36.4.0 -t vlaguard/vla:l4t-r36 .
```

On the desktop:

```bash
docker build -f deploy/docker/companion.Dockerfile -t vlaguard/companion:jazzy .
docker build -f deploy/docker/sitl.Dockerfile -t vlaguard/sitl:copter-4.5.7 .
```

The two `vlaguard/companion:jazzy` builds come from the same file for two
architectures, with no build arguments. (`docker buildx build --platform
linux/amd64,linux/arm64 ...` with a registry produces both from one machine,
if you prefer.) The SITL build checks out ArduPilot Copter-4.5.7 and stops if
the tag does not resolve to the pinned commit.

If the base image tag in the VLA build does not exist for your L4T release,
`jetson-containers`' `autotag l4t-pytorch` prints the matching one. If
`bitsandbytes` fails to import inside the VLA image, 4-bit loading is not
available on that build: use a Jetson build of bitsandbytes (jetson-containers
provides one) as the base instead, and note which in the evidence.

Record the image IDs on both machines (on the desktop, into a session file of
your choice):

```bash
for i in vlaguard/companion:jazzy vlaguard/vla:l4t-r36; do
  docker image inspect --format "$i {{.Id}} {{.Architecture}}" "$i"
done >> deploy/evidence/incoming/hil/images.txt
```

## 7. Start the desktop side

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

## 8. Start the Orin side

```bash
cd ~/vla-drone
docker compose --env-file deploy/topologies/hil.env \
    -f deploy/compose/docker-compose.orin.yml up -d mavros
docker compose --env-file deploy/topologies/hil.env \
    -f deploy/compose/docker-compose.orin.yml exec mavros \
    bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic echo --once /mavros/state'
docker compose --env-file deploy/topologies/hil.env \
    -f deploy/compose/docker-compose.orin.yml run --rm -T facts \
    > deploy/evidence/incoming/hil/host_facts.json
grep -A3 '"is_orin"' deploy/evidence/incoming/hil/host_facts.json
```

`connected: true` means MAVLink flows from SITL on the desktop, through
mavlink-router and the cable, into MAVROS on the Orin. `connected: false`
means it does not: check step 3 (`MAVROS_HOST` must be the Orin's address) and
the router log.

`"is_orin": true` with `"missing": []` means the containers can read the
evidence a run needs to be labelled hil: the device tree's board name and the
L4T release. If `missing` says the device model is `None`, the container
cannot read `/proc/device-tree/model`; the Orin compose file lifts Docker's
masking of it with `security_opt: systempaths=unconfined`, which needs
Docker 19.03 or newer. (`-T` keeps a terminal from writing CRLF into the file.)

## 9. A first mission

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
was missing: usually the device tree (step 8), an ArduPilot SITL running on
the Orin itself, or an `FCU_URL` without the desktop's address.

## 10. Profile the Shield on the Orin

This is the measurement the grant asks for by name ("Orin profiling") against
the 100 ms tick and 5 ms query budgets.

```bash
sudo nvpmodel -q                 # on the Orin itself (not visible in the containers):
                                 # put it in --note, and use the same mode for every run
docker compose --env-file deploy/topologies/hil.env \
    -f deploy/compose/docker-compose.orin.yml run --rm profile \
    profile --cpus 4-7 --note "Orin, power mode <mode>, nothing else running"
```

It replays the same 1,301 real flight ticks the desktop baseline used
(`deploy/evidence/shield_tick_pack.json`) and writes
`deploy/evidence/incoming/hil/shield_tick_profile.json`. It refuses the hil
label unless guardrail.manifest's Orin rule passes inside the container, and
it fails, by design, if the 50-rule arm never reached a fence or if the
host-speed reference moved by more than 1.5 times during the run (then re-run
with nothing else on the Orin). `--cpus` pins it to fixed cores; the Orin's
Arm cores have one hardware thread each, so any four cores will do, but use
the same ones every time.

Compare with `deploy/evidence/dev/shield_tick_profile.json` (desktop
performance cores) and `shield_tick_profile-ecores.json` (desktop efficiency
cores). The `host_speed` block in each file is the same reference workload on
each machine. The headline is `_check` p99 against 5 ms and `filter` p99
against 100 ms at 50 rules and a 5 s horizon at 0.1 s, with the maximum and
the count of ticks over budget beside each.

## 11. VLA inference rate on the Orin (protocol R1)

Every backend is measured in the `vla` image, one at a time, with nothing else
using the GPU:

```bash
for b in stub bc_v3 openvla_7b_4bit aerialvla_lora aerialvla_ft_run2; do
  VLA_BACKEND=$b docker compose --env-file deploy/topologies/hil.env \
      -f deploy/compose/docker-compose.orin.yml --profile vla-bench run --rm vla-bench
done
```

Each writes `deploy/evidence/incoming/hil/vla_rate_<backend>.json`.
`nvpmodel` is not visible inside the containers, so record the power mode in
the session's `orin_check.txt`, and keep the same power mode for every backend.
For OpenVLA, the default is the flight script's default prompt; to time the
longer prompt with the Constraint Summary Pack as well, run
`docker compose ... --profile vla-bench run --rm vla-bench measure --backend
openvla_7b_4bit --allow-gpu --model-root /models --topology hil --csp on`.

## 12. Running the KPI campaign from the desktop

Once steps 8 to 10 work and step 9's manifest says `hil`, the campaign is a
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

(`orin` is a placeholder for the Orin's user name.) Review and commit
`deploy/evidence/hil/` on the desktop; the Orin then pulls it like any other
commit, and since its own new measurements go to `incoming/`, they never
overwrite a tracked file. `sim_speedup` is fixed at 1 in the SITL image, and
every run reads it back from SITL, as the grant requires for a KPI-bearing
run. The scenario library of the stress harness
(`experiments/sweep_scenarios.py`) drives this rail once the Shield node
accepts a scenario's start, target, pilot and seed; until then the campaign is
the node's own mission, repeated.

## 13. Evidence each hil run must carry

| Evidence | Where it comes from | Where it goes |
|---|---|---|
| Six-field manifest (code revision, model hash, policy hash, seed, `sim_speedup` 1.0, topology `hil`) | the Shield node | `demo/out/<tag>/manifest.json` |
| ROS distribution, MAVROS node seen, `fcu_connected: true`, ArduPilot version, the Orin's board and L4T release, the VLA node's host, the `fcu_url` and the route to the desktop, the MAVLink processes on the Orin (`local_processes`, `mavros_on`) | the Shield node's `hil_evidence` | `metrics.json` |
| The Orin as the containers see it | step 8 (`facts`) | `deploy/evidence/hil/host_facts.json` |
| Shield time per tick on this Orin | step 10 | `deploy/evidence/hil/shield_tick_profile.json`; its summary (`python tools/profile_shield_tick.py attach <file>`) beside each run |
| Same commit on both hosts, clean tree | step 5 | the manifest's `code_revision` (no `-dirty`) |
| Same image on both hosts | step 6 | `deploy/evidence/hil/images.txt` |
| Clock offset | step 4 | `deploy/evidence/hil/chrony_<date>.txt` |
| Router endpoints | step 7 | the router log, saved beside the session's evidence |

## 14. Moving to the drone (flight)

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
       bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic echo --once /mavros/state'
   ```

   Expect `connected: true` and the autopilot's mode. Mission Planner keeps its
   own link through the telemetry radio.
4. **Profile the Shield on the drone's Orin** (step 10 with `flight.env`), in
   the power mode the drone will fly in. The Orin may be powered differently
   on the airframe than on the bench.
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
| `connected: false` in `/mavros/state` | No MAVLink reaching the Orin: wrong `MAVROS_HOST`, cable or address (step 3), or the router container not running. |
| `manifest.json` says `dev` on the Orin | Read `topology_refused` in `metrics.json`. Usually the device tree is not readable in the container (step 8, `facts`), a SITL process runs on the Orin itself (`local_processes` in `metrics.json` names it; stop it), or `FCU_URL` lost the desktop's address. |
| `manifest.json` has `-dirty` in `code_revision` | Something wrote a tracked file on the Orin. `git status --porcelain` names it; restore it with `git checkout -- <file>`, and keep outputs in `deploy/evidence/incoming/` and `demo/out/`. |
| MAVROS exits at start-up mentioning GeographicLib | The image was built without the geoid datasets; rebuild the companion image (the Dockerfile installs them). |
| MAVROS connected but no position | ArduPilot streams nothing until a stream rate is requested; the Shield node requests it at bring-up. |
| `manifest.json` says `unversioned` | git could not read the mounted checkout. The image sets `safe.directory`; check that the checkout is a git clone and is mounted at `/opt/vlaguard`. |
| A build sends gigabytes of context | BuildKit is off, so the per-Dockerfile ignore file is not read. Use Docker 23 or newer, or `DOCKER_BUILDKIT=1`. |
| `bash\r: No such file` in a container | The checkout converted line endings. Clone with `git config --global core.autocrlf false` on Windows. |
| `profile` refuses `hil` | The container is not seen as an Orin: run `facts` (step 8) and read `missing`. |
| `profile` fails with "host-speed reference moved" | Something else ran during the profile; re-run with nothing else on the host. |
