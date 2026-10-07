# Project AirSim — Setup & Status (verified working 2026-07-07)

**Project AirSim v0.2.0** (IAMAI, UE 5.2): the actively maintained successor to
classic AirSim, and since September the camera rail every CityLife flight runs on
(`demo/follow_vlm.py`). Proven flying on this machine since 2026-07-07.

## What's installed where

| Piece | Location |
|---|---|
| Neighborhood world (UE5) | `D:\ProjectAirSim\Neighborhood\Neighborhood-Windows-UE5.2-PAS_v0.2.0\AirSimNH.exe` |
| Client repo (sparse: client dir only) | `D:\ProjectAirSim\repo` |
| Python client env | conda **`pas`** (Python 3.11.15, `projectairsim` installed editable); the flight scripts run in `vla-real` (3.10.20), see `docs/DESIGN-python-versions.md` |
| Server log | `<world dir>\AirSimNH\projectairsim_server.log` |

> Their `Blocks-Windows-UE5.2` release zip is MISLABELED (contains the Linux
> UE5.7 plugin). Use Neighborhood; report/re-check Blocks upstream later.

## How to run (verified sequence)

```powershell
# 1. start the world (UE5 — first launch compiles shaders, be patient)
D:\ProjectAirSim\Neighborhood\Neighborhood-Windows-UE5.2-PAS_v0.2.0\AirSimNH.exe

# 2. server is ready when ports 8989 (topics) + 8990 (services) listen:
Test-NetConnection 127.0.0.1 -Port 8990

# 3. fly the official example:
conda activate pas
cd D:\ProjectAirSim\repo\client\python\example_user_scripts
python hello_drone.py     # takeoff -> up -> down -> land
```

## Key differences vs our current AirSim rail

| | AirSim 1.8.1 (current) | Project AirSim v0.2.0 |
|---|---|---|
| Protocol | msgpack-RPC, port 41451 | **pynng (NNG)**, ports 8989/8990 |
| Client | `airsim` pip pkg | `projectairsim` (from repo, editable install) |
| World/robot config | `settings.json` (global) | **JSONC scene + robot configs, sent BY THE CLIENT** (`World(client, "scene_x.jsonc")`) |
| Engine | UE4 | UE 5.2 |
| Empty world at start | drone auto-spawns | **no vehicle until a client creates the world** — that's why no drone is visible on launch |
| Maintenance | archived 2022 | active (2026 commits) |

## Adapter plan (next step when we want the guardrail here)

The client API has direct equivalents for everything our adapter needs:
- state: pose topics / `drone.get_ground_truth_kinematics()`
- velocity control: `drone.move_by_velocity_async(v_north, v_east, v_down, duration)`
- camera: image topics (subscribed callbacks)

So `demo/run_demo.py`'s AirSim calls map ~1:1; guardrail core untouched (adapter
isolation, proven 5× if we count this). Estimated effort: one working session.

## ArduPilot, Project AirSim and Mission Planner

*Added 2026-10-06, answering the PI's question "can Project AirSim be connected to
Mission Planner?". Written from the Project AirSim repository on this machine
(`D:\ProjectAirSim\repo`); this chain has not been run here yet.*

**Short answer: yes, through ArduPilot.** Mission Planner never talks to Project
AirSim. It talks MAVLink to ArduPilot. Project AirSim has a native ArduPilot
controller, so ArduPilot SITL can fly the Project AirSim drone, and Mission
Planner then sees and commands that drone like any ArduPilot vehicle. The
example script says so in its own docstring ("Mission Planner can be used to
control the drone").

```
Project AirSim (UE 5.2, desktop GPU)                 Windows
   robot controller "ardupilot-api"
        |  UDP 9003 -> ArduPilot,  9002 <- ArduPilot
        |  (sensors out, motor outputs back, in lockstep)
ArduPilot SITL  sim_vehicle.py -v ArduCopter -f airsim-copter
        |  MAVLink
  mavlink-router fan-out
        |-----------------------------|
   MAVROS 2 -> Safety Shield node     Mission Planner (GCS)
```

That is the grant's own layout: "GCS: Mission Planner via mavlink-router
fan-out (parallel to MAVROS)" (Grant overview p.3; Architecture constraints
p.3), with Project AirSim as the simulator, which the grant keeps on the
desktop in every topology because Unreal needs the discrete GPU.

**The pieces in the Project AirSim repo**
(`client/python/example_user_scripts/ardupilot/`):

| File | What it holds |
|---|---|
| `sim_config/robot_ardu_quadrotor.jsonc` | `"controller": {"type": "ardupilot-api", "ardupilot-settings": {"ardupilot-ip": "127.0.0.1", "ardupilot-udp-port": 9003, "local-host-ip": "127.0.0.1", "local-host-udp-port": 9002, ...}}` |
| `sim_config/scene_ardu_quadrotor.jsonc` | the scene that loads that robot |
| `project-airsim-quad.param` | ArduCopter gains for this airframe; its header gives the launch line `sim_vehicle.py -v ArduCopter -f airsim-copter --add-param-file=<this file>` |
| `ardupilot_quadrotor.py` | connects, loads the scene, subscribes to the cameras; ArduPilot (Iris, FRAME_CLASS 1, FRAME_TYPE 1) flies it |

**Bring-up, in outline.** This section is the short answer for the PI. The
runnable version is `scripts/run_pas_ardupilot.ps1` (`-Step check`, then
`-Step g1` or `-Step scene` + `-Step sitl`). Its design, the address table and
the gate that proves the loop closes (G1) are in
[DESIGN-projectairsim-ardupilot.md](DESIGN-projectairsim-ardupilot.md). That
document also records its status: built and tested offline, not yet flown on
this desktop.

1. Start the Project AirSim world (as in "How to run" above) and load the
   scene that carries the `ardupilot-api` robot (`ardupilot_quadrotor.py` from
   the `pas` env, or the launcher's `-Step scene`). **The scene loads first.**
   Reloading a scene resets the simulator clock, and ArduPilot's AirSim backend
   adds the (negative) step to its own clock, so SITL starts after the scene.
2. In WSL, start ArduPilot SITL with the AirSim backend, the param file and
   the address that Project AirSim listens on for motor outputs:
   `sim_vehicle.py -v ArduCopter -f airsim-copter --add-param-file=<path>/project-airsim-quad.param --sim-address=<Windows end of the WSL switch>`.
   SITL sends its PWM outputs to `--sim-address` on UDP 9002, and the default
   is 127.0.0.1. With SITL in WSL under NAT networking, that default never
   reaches Windows: WSL forwards TCP from Windows' localhost but not UDP.
   Our pinned tree is `~/ardupilot` at Copter-4.5.7 (`sitl/setup_sitl.sh`).
3. Point the robot config at it. Under WSL NAT (this PC): `ardupilot-ip` is
   WSL's own address (`wsl hostname -I`), and `local-host-ip` is the Windows
   end of the WSL switch (WSL's default gateway), the same address as
   `--sim-address`. Under WSL mirrored networking all three are 127.0.0.1.
   The WSL address changes on every WSL restart, so the launcher renders the
   config per run.
4. Fan MAVLink out with mavlink-router: one endpoint for MAVROS 2 (the Shield
   node), one UDP endpoint for Mission Planner on Windows (14550, at the
   gateway address), and a TCP server for Windows clients (5790). The PI's
   reference compose file does the same with
   `mavlink-routerd -e mavros:14555 -e host.docker.internal:14556`.
5. In Mission Planner, connect UDP 14550. If Windows Firewall drops the UDP
   stream from the WSL adapter, connect TCP to 127.0.0.1:5790 instead; TCP is
   forwarded and needs no firewall rule.

**Where this goes next.** The same chain is the perception-rail integration the
final delivery names: Project AirSim imagery and physics, ArduPilot flight
control, MAVROS 2 into the Shield, Mission Planner beside it. For the KPI
campaign the VLA and the Shield move to the Jetson Orin (the grant's hil
topology) while Project AirSim, ArduPilot SITL, MAVROS 2 and Mission Planner
stay on the desktop; later the same stack moves onto the drone (flight
topology).
