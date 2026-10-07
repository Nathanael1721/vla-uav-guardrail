# Design: one stack for dev, hil and flight

**Date:** 2026-10-06
**Files:** `deploy/` (images, compose files, topology files),
`tools/profile_shield_tick.py`, `tools/vla_backend_table.py`, and their tests
`tests/test_deploy_configs.py`, `tests/test_profile_shield_tick.py`,
`tests/test_vla_backend_table.py`. The step-by-step for the Orin is
[RUNBOOK-orin-hil.md](RUNBOOK-orin-hil.md).

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
once the airframe is ready.

## Decisions

### 1. Two images, each identical across topologies

| Image | Contents | dev | hil | flight |
|---|---|---|---|---|
| `vlaguard/companion:jazzy` (`deploy/docker/companion.Dockerfile`) | ROS 2 Jazzy, MAVROS 2, mavlink-router, Cyclone DDS, rosbag2 (MCAP), the guardrail's Python environment | amd64 build | arm64 build | **the same** arm64 build |
| `vlaguard/vla:*` (`deploy/docker/vla.Dockerfile`) | PyTorch, transformers and bitsandbytes for the OpenVLA-based backends; every R1 rate measurement | CUDA 12.4 base | L4T (JetPack 6) base | **the same** L4T build |

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
- GPU user space must match the host's driver stack: L4T on the Orin, CUDA 12.4
  on the desktop. The companion image needs no GPU, so it can be one multi-arch
  definition built from the official `ros:jazzy-ros-base` image. Only the VLA
  image has a per-architecture base, and hil and flight still share one build.

### 2. ROS 2 Jazzy inside the containers

The grant leaves the ROS 2 distribution open ("Humble (LTS, AI Wings parity),
Iron, or Jazzy?", Safety Shield p.7). Jazzy is chosen because:

- The grant also says "Python 3.11+ as a ROS 2 node (rclpy)" (Safety Shield
  p.1). Jazzy's rclpy runs on Python 3.12; Humble's on 3.10, below that floor.
- The stored dev-rail runs (`demo/out/ros2_*`) flew on Jazzy with MAVROS 2.
- Jazzy is supported until May 2029; Iron has reached end of life.
- A container's Ubuntu is independent of the host's. If the Orin runs
  JetPack 6 (Ubuntu 22.04), the companion container is still Ubuntu 24.04 on
  top of it, which works because the companion image does not use the GPU.
  Only the VLA image has to follow the Orin's L4T release.

Humble's advantage, parity with the AI Wings stack, matters only where two ROS
graphs exchange DDS traffic. In the layout below the only link between the
desktop and the Orin is MAVLink over UDP, which is independent of the ROS
distribution.

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
(runbook step 10) decides whether the monitor needs the faster path the
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
be measured (runbook step 11). CognitiveDrone and BitVLA, both named by the
grant, have no checkpoint here. No backend is the default: the grant defers
that choice to this measurement set.

## Status

| Item | Written | Built | Run |
|---|---|---|---|
| Companion, VLA and SITL images (`deploy/docker/`) | yes | not yet | not yet |
| dev, hil-desktop, Orin and flight-override compose files | yes | - | `docker compose config` only |
| Topology files and DDS configuration | yes | - | checked by tests |
| Entrypoint roles and refusals | yes | - | dry-run in tests; the router role with a stand-in `mavlink-routerd` |
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

## Next steps

1. Build the images and bring up hil on the Orin (runbook steps 6 to 9).
2. Run the Shield profile and the R1 rates on the Orin (steps 10 and 11).
3. `sitl/ros2_shield_node.py`: put the Orin's profile summary
   (`tools/profile_shield_tick.py attach`) into each run's `metrics.json`.
4. `sitl/ros2_vla_bridge_node.py`: let a VLA in the `vla` image publish
   `/vla/action_4d`, so a camera VLA can be the pilot on the Orin
   (`deploy/compose/docker-compose.orin.yml` lists it as pending).
