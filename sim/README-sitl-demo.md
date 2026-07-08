# SITL + ROS 2 bring-up (dev topology, no Docker)

`run_sitl_ros_demo.sh` runs the Phase-1 ROS 2 nodes (`safety_shield_node`,
`mavlink_adapter`, `ros_vla_stub`) against a real **ArduPilot SITL** on the same
host. This is the `dev` topology from `docs/03-simulation/topologies.md` minus
the Gazebo/Docker layer — SITL alone is enough to exercise the MAVLink path the
Shield sits on.

## Run (from WSL)

```bash
bash sim/run_sitl_ros_demo.sh on     # shield ON  -> /shield/intercept events + audit log
bash sim/run_sitl_ros_demo.sh off    # shield OFF -> passthrough, no filtering
```

Verified output on this host (ROS 2 Jazzy, mavros, SITL from `~/ardupilot`):
the stack connects end-to-end and the Shield's audit log (`episodes/sitl_demo/
shield_audit.jsonl`) accumulates records carrying the loaded bundle's
`policy_hash` — the same reproducibility property `make demo` asserts offline.

## Why not `ros2 run` / colcon

This host is ROS 2 Jazzy with no `colcon`, and the nodes are plain `rclpy`
scripts with `__main__` blocks. The script therefore:

1. **Makes the workspace importable via a user-site `.pth` file** rather than a
   colcon overlay. Exporting `PYTHONPATH` breaks `rclpy` under Ubuntu's
   externally-managed ROS Python (the ROS site-packages `.pth` machinery is
   order-sensitive); a user-site `.pth` keeps rclpy's path intact while still
   letting the nodes `import policy_dsl / safety_shield / vlaguard_common`.
2. **Runs the nodes as `python3 -m <module>`** with `--ros-args -p ...` for
   parameters, instead of `ros2 run <pkg>` (no colcon overlay = no executables
   registered).

One-time host setup (already done on this lab machine):

```bash
# ROS Python needs the workspace deps (PEP 668 -> --break-system-packages in the lab env)
pip3 install --break-system-packages shapely pydantic pyyaml
```

## What is *not* here yet

The VLA stub does not arm/takeoff the vehicle, so this validates the **software
plumbing** (MAVROS ↔ Shield ↔ VLA, audit trail, policy-hash matching) rather than
a flown A/B mission. A full flown A/B over SITL needs the bring-up's arming/
takeoff state-machine (cf. the in-house prototype's `sitl/run_sitl_demo.py`),
which is the natural next step. The Shield logic itself is already fully
exercised by `make sim` (kinematic) and the unit suite.
