# Scene motion belongs to the simulator, not to a client thread

**Date:** 2026-08-25
**Status:** cause established, first approach tried and **abandoned on evidence**,
correct mechanism identified and not yet built.

## The complaint

Car motion in the demo videos looks choppy and unnatural.

## The cause is the sampling rate, not the motion model

`demo/moving_car.py` already carries a proper vehicle model: rounded corners,
cornering speed capped by a lateral-acceleration limit, and a forward/backward
pass that brakes into corners and accelerates out of them under a longitudinal
limit. The speed it produces is continuous. Nothing about the path is wrong.

What was wrong is how often that continuous model got *sampled*. Measured on the
delivered `demo_follow` flight:

| | measured |
|---|---|
| Car pose updates while moving | **8.69 Hz**, median step **54 cm** |
| Recorder capture rate | **15.57 Hz** |
| Frames during motion repeating a position | **44 %** |
| Background traffic (`update_every = 2`) | **4.4 Hz**, ~72 % repeated |

The car jumped 54 cm and then held still for a frame. In the code the single car
was stepped only on **even** ticks (`tick % 2 == 0`), so it moved at roughly half
an already-slow control loop.

## What was tried, and why it was abandoned

The obvious fix, by analogy with `demo/recorder.py`, was to move scene updates
onto their own fixed-rate thread. In isolation it worked perfectly — 19.96 Hz
achieved against a 20 Hz target, zero late slots.

In flight it **aborted the mission**:

```
pynng.exceptions.ConnectionReset: Connection reset
[car] teleport failed (Canceled: Operation canceled)
[warn] flight aborted: BadState: Incorrect state
```

The Project AirSim client is **not thread-safe**. The driver thread's
`set_object_pose` calls collided with the control loop's RPCs on the same
connection and corrupted it. This is documented in this project's own history —
the real-VLA demo runs inference on a background thread with *its own* AirSim
client for exactly this reason.

Giving the driver its own connection does not work either. `World.__init__`
loads the scene, which would destroy everything already spawned; passing an empty
scene name skips the load (`if scene_config_name:`) but then hangs in
`import_ned_trajectory` against the default topic.

**A client thread is the wrong place for this.** The approach was reverted whole,
and a flight re-run to confirm the repository is back to working: 266 ticks,
hit rate 1.000, 47 Shield interventions, no abort.

## The right mechanism: env actors with trajectories

Project AirSim already moves scene objects itself, and the client API for it is
sitting unused:

```
World.import_ned_trajectory(name, time[], x[], y[], z[], roll[], pitch[], yaw[], ...)
EnvActor.set_trajectory(traj_name, to_loop=True, time_offset=..., x_offset=...)
```

The whole path is uploaded **once**. The simulator then interpolates and moves
the actor at its own render rate. That removes the judder by construction rather
than by out-running it, costs essentially no per-frame RPC, and therefore takes
nothing from the detector — which matters, because `det_hz` only just cleared its
4.0 Hz gate.

One trajectory can drive several actors with per-actor time and position offsets,
which is how a small fleet gets staggered without any client-side scheduling.

### And it is articulated, which is what the pedestrian needs

An env actor is defined by **links and joints**, not a single mesh:

```
EnvActor.set_link_rotation_angle(link_name, angle_deg)
EnvActor.set_link_rotation_angles({link: angle, ...})
EnvActor.set_link_rotation_rate(link_name, deg_per_sec)
```

The shipped quad-tiltrotor example rotates four shroud links to tilt its rotors.
The same mechanism applied to a figure with upper-leg, lower-leg and arm links is
a **real gait**, driven by joint angles, rather than the pose-swapping workaround
planned earlier.

That workaround — baking N static GLBs at N phases of a walk cycle and cycling
which one sits at the walking position — remains technically valid, because a
spawned glTF genuinely cannot animate (`AssimpToProcMesh` produces a procedural
mesh with no bones, and there is no animation API). But it is strictly worse than
articulating an env actor, and it was only chosen because env actors had not yet
been found.

## What this costs

Env actors are **declared in the scene configuration**, not spawned at runtime.
Their links reference geometry, and our own `robot_semantic_quad.jsonc` shows
both forms in use: `"type": "unreal_mesh"` by asset path, and `"type":
"geometry"` with a `box` primitive.

So there are two routes, and they differ in how much Unreal content work they
need:

1. **Primitive links.** A figure assembled from box links — torso, thighs,
   shins, arms, head — needs no imported assets at all. It would be blocky. At a
   9 m cruise a person occupies roughly 10 px of the 400 x 225 detector frame, so
   the *detector* would likely be unaffected, but that has to be measured rather
   than assumed: a box figure that OWL-ViT does not read as "a pedestrian" fails
   the actual mission.
2. **Imported limb meshes.** Human-looking, and needs the limbs brought into
   PASBlocks as separate meshes through the Unreal editor.

The same choice applies to vehicles, except vehicles need no articulation at all
— a trajectory alone fixes them, with the existing packaged or glTF mesh.

## Recommended order

1. Move the **car** to an env actor with an imported trajectory. No new assets,
   no articulation, and it settles the complaint that was actually raised.
   Measure the duplicate-frame fraction: it is 44 % now and should approach 0.
2. Then decide the pedestrian's appearance, having seen whether a primitive
   figure is detected as a person.
