# The city can hold its own traffic

`/Game/CityLife/Maps/CityLife_Day` is a copy of `JapaneseCity/Demo_day` with 16 walking
pedestrians and 8 driving cars built INTO the level. They cost no RPC and need no client:
the drone's camera sees a moving city whether or not `demo/pedestrians.py` and
`demo/city_traffic.py` ever run.

That is the whole point. The figures the demo shows today are pushed in at run time over
the ProjectAirSim RPC as bone-less baked GLB props, they cannot walk (`AssimpToProcMesh`
discards bones), and one car costs about 10 RPC/s against a detector whose `det_hz` only
just cleared its 4.0 Hz gate.

Built 2026-09-10 through the editor's MCP server. `Demo_day.umap` was hashed before,
during, and after: `2e9f91a4…802efa83` all three times. It was never opened.

## What is in the level

| Actor | Count | Where it came from |
|---|---|---|
| `Ped_00…Ped_15`, tag `citylife.ped`, folder `CityLife/Peds` | 16 | pavement cells from `street.npz` |
| `Car_00…Car_07`, tag `citylife.car`, folder `CityLife/Cars` | 8 | the `city_traffic.py` circuit |
| `CityLife_NavBounds` (+ `RecastNavMesh-Default`) | 1 | corridor only, not the whole map |

Names are load-bearing: `demo/pas_config/scene_guardrail.jsonc` sets
`"segmentation": {"use-owner-name": true}`, so an actor's name IS its segmentation class.
`Ped_NN` / `Car_NN` do not collide with the client-side `Person{k}` / `BgCar{i}`, so a
flight that spawns both keeps two distinguishable sets.

## The two Blueprints

`BP_CityPed` (parent `Character`) walks on a NavMesh. `SKM_Manny` / `SKM_Quinn` with
`ABP_CityPed_Manny` / `ABP_CityPed_Quinn`, `MaxWalkSpeed` 126-154, `DetourCrowdAIController`
so they avoid each other. `BeginPlay` records `HomeLocation` and starts a 1 s looping timer
on the function `Roam`, which decides when and where the figure walks next and calls
`SimpleMoveToLocation`. What `Roam` does now is in
[They walked like robots](#they-walked-like-robots-and-the-animation-was-not-why).

The plan for `Roam` was a latent `AIMoveTo` re-armed from its own `OnSuccess`/`OnFail`.
That needs a custom event, and **the graph DSL cannot write custom events** — `(event Roam …)`
fails with `AddEvent|Roam does not exist`, and pre-creating the node with `add_event` does
not help. A named function graph plus `SetTimerByFunctionName` reaches the same behaviour
with nodes the DSL can write.

`BP_CityCar` (parent `Actor`, root `Body` = `SM_AutomotiveTP_Car`, `Movable`, `QueryOnly`)
follows `Route` — a `TArray<FVector>`, not a spline — at `SpeedCmS` 250, wrapping on `Idx`
and turning through `RInterpTo` so corners round rather than snap. Paint is set per
instance from the eight `MIC_Paint_*`, deliberately fixed rather than random so the colour
test in `city_traffic.py` keeps a stable ground truth.

Variant B (`bUseNavMesh = false`, route following off Tick through `AddMovementInput`) is
compiled into `BP_CityPed` and unused. It exists so a navmesh failure is a bool flip.

## The walk looked robotic, and the blend spaces say why

Bound to the pack's own `ABP_Manny`, the figures animated but read as stiff and dragged.
`BS_MM_WalkRun` is authored with three samples - **0 (`MM_Walk_InPlace`), 230
(`MM_Walk_Fwd`), 500 (`MM_Run_Fwd`) cm/s**. At `MaxWalkSpeed` 140 that is 61% forward walk
blended with **39% marching on the spot**: the ground speed comes out right, because the
blend's implied speed is the weighted average, but the stride is squashed to 61% at full
cadence. A shuffle, which is exactly what "robotic" looked like. `BS_MF_Unarmed_WalkRun`
is authored the same way.

### The retraction: there was no stale grid

An earlier revision of this file claimed the fix was impossible because "a blend space does
not blend from `SampleData`; it blends from `GridSamples`", which MCP cannot rebuild. **That
was wrong, and the property that disproves it is readable:** `bInterpolateUsingGrid` is
`false` on both blend spaces - *"If true then interpolation is done via a grid at runtime.
If false the interpolation uses the triangulation."* These blend spaces interpolate from
`SampleData` directly, so editing a sample value through `ObjectTools.set_properties` takes
effect with no grid rebuild at all. The intermediate workaround that claim produced - move
the pedestrians to 230 cm/s so the blend lands exactly on the walk sample - was both
unnecessary and, per the next section, wrong for this repo.

### What is shipped

Two settings, both on our own copies in `/Game/CityLife/Animations/`; the pack's assets are
untouched.

1. **The walk sample is retimed to the speed it is actually used at**: `MM_Walk_Fwd` /
   `MF_Walk_Fwd` moved to **140 cm/s with `rateScale` 0.6087** (= 140/230). Playing the
   cycle at 61% rate covers 61% of the distance in the same number of steps, so the stride
   stays full-length and only the cadence slows. At 140 the blend is then 100% walk - no
   in-place pose mixed in, which is what removes the shuffle.
2. **The blend space scales its own playback to the requested speed**: `bScaleAnimation`
   true plus **`axisToScaleAnimation = BSA_X`** - *"the speed axis will scale the animation
   speed in order to make up the difference between the target and the result of blending
   the samples."* This is the engine's built-in speed matching, and it is what makes
   per-instance speed variation safe: every pedestrian gets its own `MaxWalkSpeed`
   (**126-154 cm/s**, scattered around `demo/pedestrians.py`'s `WALK_SPEED_MPS = 1.4`) and
   the blend space time-scales each one instead of letting the feet slip.

`RotationRate` yaw is 250 (was 360) so turns are not snapped, and `Roam` issues a path only
when the figure has actually stopped (`GetVelocity` < 10) instead of re-pathing blindly
every 4 s - that was its own source of stutter.

### Why not simply walk them at 230 cm/s

Because the repo already fixed a pedestrian speed, and it is not a cosmetic choice.
`demo/follow_vlm.py:242` sets `SUBJECT_VMAX_MPS["pedestrian"] = 2.0`, with the reasoning at
`:206-209` that these are *"ceilings that only a diverged track can reach, not expected
speeds"*. 2.3 m/s is above that ceiling, so `demo/target_state.py:180-190` would clamp a
healthy track on essentially every update: `n_clamped` stops being a divergence signal and
the speed estimate becomes a systematic under-report feeding `range_rate`. The one recorded
flight that carries the field (`demo/out/retarget_smooth/metrics.json`) already shows 66 of
192 updates clamped against a 1.4 m/s truth - a single flight, so weak evidence on its own,
but the inequality 2.3 > 2.0 needs no measurement. `tests/test_target_state.py:291` asserts
the 2.0 ceiling holds.

### Why not stride warping

`AnimationWarping` is now enabled in `Blocks.uproject` (backup in `Blocks.uproject.bak`) and
its `StrideWarping` / `OrientationWarping` nodes CAN be created in the top-level AnimGraph,
between `Slot 'DefaultSlot'` and `ControlRig` - outside the state machine that refuses new
nodes. They were deliberately not wired in:

- **Stride warping fights `bScaleAnimation`.** Both correct the same speed mismatch, one by
  rescaling playback, one by stretching the stride. Stacked, the correction is applied
  twice and the feet slip the other way. Warping only earns its place with blend-space
  scaling switched off.
- **The range is too narrow to show.** At 126-154 cm/s against a 140 cm/s sample,
  `StrideScale` would be 0.9-1.1; a ±10% playback rate is not visible from a drone.
- **Orientation warping has nothing to correct.** `bOrientRotationToMovement` keeps the
  figures facing their velocity, so the angle it warps is ~0.
- **Its setup is not writable here.** The node needs pelvis, IK-foot-root and per-foot bone
  definitions, and `set_properties` has already been caught dropping struct fields.

Measured after the change: **16 of 16 walking**, each at its own `MaxWalkSpeed`.

## They walked like robots, and the animation was not why

With the gait fixed, the figures still read as mechanical. Two causes, both in movement
rather than animation, both found by reading state rather than looking at frames.

### Speed was binary

One Simulate snapshot: **15 of 16 pedestrians at exactly `MaxWalkSpeed` or exactly 0.** The
CharacterMovement component had `NavMovementProperties.bUseAccelerationForPaths = false`, so
path following wrote velocity directly - full walking pace on the first frame of a move, dead
stop on the last. Real people take a step or two to get going.

Now, on the `BP_CityPed` template AND on each of the 16 placed instances (placed actors did
not pick up the template change, even after a compile):

| Property | Was | Now | Why |
|---|---|---|---|
| `bUseAccelerationForPaths` | false | true | paths drive acceleration, not velocity |
| `MaxAcceleration` | 900 | 200 | ~0.7 s from standstill to 1.4 m/s |
| `BrakingDecelerationWalking` | 2048 | 260 | a stop takes a stride, not a frame |
| `bUseFixedBrakingDistanceForPaths` / `FixedPathBrakingDistance` | false / 0 | true / 90 | eases off before the goal instead of braking on it |
| `BrakingFrictionFactor` | 2 | 0.5 | friction x 2 x `GroundFriction` 8 would out-brake the deceleration above |

`bUseSeparateBrakingFriction` was the first choice for that last row. It writes on the
template but `set_properties` refuses it on a placed instance, so the factor was lowered
instead.

Measured after: intermediate speeds (75, 95, 133 cm/s) appear in snapshots, which a
direct-velocity mover never produces.

### `Roam` paced, never paused, and mostly failed

The old `Roam` re-pathed the instant a figure stopped, to a random point within 12 m of its
SPAWN, so a figure walked a few metres, turned round, and walked back, with no pause ever.
It also pulled the pure `GetRandomReachablePointInRadius` twice, once for the branch and once
for the move, and a pure node re-rolls on every pull - the point walked to was never the point
checked.

The first rewrite aimed at a point 9 m ahead and asked `GetRandomReachablePointInRadius`
around it. **12 of 16 `Goal`s came back exactly 900 cm away**: the query fails outright when
its origin is not on the navmesh, and a point 9 m ahead of someone on a pavement is usually in
the road or a building. The second rewrite projected that point with
`ProjectPointToNavigation`, and 5 of 16 `Goal`s came back as **(0, 0, 0)**, 5-8 km away: a
failed projection returns the zero vector, not the input.

What is shipped:

- **Pause.** On stopping, the figure waits - 70% a beat (0.2-1 s), 30% a real stop (2-6 s).
- **Next leg.** 14 m within 30 deg of where it faces (80%), or any direction (20%) so a
  figure at the end of a pavement turns round rather than stalling. Past `RoamRadius`
  (now 2000) it heads home instead.
- **Goal.** `select(projected_ok, projected, GetRandomReachablePointInRadius(own location,
  12 m))`. The fallback's origin is the pawn's own position, which is on the mesh by
  construction. Both pure nodes feed one `select` consumed by one `Set Goal`, so `Goal` comes
  from a single evaluation; the move reads `Goal`.
- **Grace.** `NextMoveTime = now + 1.5` after a move is issued, so a figure that has not yet
  accelerated past 10 cm/s is not mistaken for one that has arrived.

Measured, Simulate, two snapshots after 20 s warmup: **13 and 11 of 16 walking**, every
`Goal` within 25 m (the largest are figures heading home), **0 of 16 in the carriageway**.

### What still looks artificial

The figures are `SKM_Manny` / `SKM_Quinn`: chrome mannequins. Motion is now plausible; the
people are not. The only other humans on disk are the Quaternius "Animated Men Pack"
(`D:/models/quaternius_people/glb`, CC0, rigged, low-poly and stylised). Photoreal figures
means MetaHuman or a Fab crowd pack, which is an asset decision, not a tuning one.

## Pedestrians are fenced off the carriageway, and the kerb is not what does it

The requirement is that a figure never steps into the lane a car drives. The kerb cannot
enforce it: the drop from the block-base apron (z = 15) to the asphalt (z = 0) is 15 cm
against `RecastNavMesh-Default`'s `AgentMaxStepHeight = 35`, so Recast links pavement and
carriageway into one continuous walkable surface. There is no tuning-only fix - 15 cm is
below any sane step height.

Traced at 5 cm intervals across the west kerb, the surface reads
`X 2900-2995 -> z 15` (apron), `3000-3295 -> z 10` (the road tile's own 3 m sidewalk),
`3300 -> z 9` (kerb face), `3305-3400 -> z 0` (carriageway), and mirrored on the east side.
So the carriageway is a plus-shape: **N-S X in [3300, 4900], E-W Y in [3300, 4900]**, with
the junction's corner kerbs cut back about 450 cm further in Y.

Two `NavModifierVolume`s with `AreaClass = /Script/NavigationSystem.NavArea_Null` cover
exactly that, sized by scale on the stock 200 cm cube brush the way `CityLife_NavBounds`
already was:

| Actor | Location | Scale | Resulting bounds |
|---|---|---|---|
| `CityLife_NoWalk_NS` | (4100, 2500, 100) | (8, 52, 2) | X 3300-4900, Y -2700-7700 |
| `CityLife_NoWalk_EW` | (3900, 4100, 100) | (24, 12.5, 2) | X 1500-6300, Y 2850-5350 |

`NavArea_Null` is already in the navmesh's `SupportedAreas` (id 0), and
`RuntimeGeneration = Dynamic` rebuilds the affected tiles by itself. Because the navmesh is
eroded by `AgentRadius = 35`, figures keep clear of the boundary rather than hugging it.

Measured in Simulate, twice, minutes apart: **0 of 16 pedestrians inside the carriageway
band**. The cost is deliberate - west and east pavements are now separate navmesh islands,
so nobody crosses the road. Crossings would need `NavLinkProxy` actors at the crosswalks.

Note what this does NOT do: a navmesh governs pathing, not physics. A car sweep or a
DetourCrowd shove could still push a capsule off the pavement, because `BP_CityCar` moves
with an unswept `SetActorLocation`.

## The cars had no wheels, which is why they slid

`SM_AutomotiveTP_Car` is a single rigid StaticMesh: 1 LOD, 124,818 triangles, and its tyres
are the `TireRubber` **material slot**, not separate geometry. `BP_CityCar` had exactly one
component and one asset dependency. Wheels could not rotate, so 2.5 m/s of pure translation
read as a prop on a conveyor - and no amount of route or speed tuning would have fixed it.

The project already contained a better body: `/Rover/OffroadCar/SM_Offroad_Body` plus
`/Rover/OffroadCar/SM_Offroad_Tire`, a standalone tyre mesh (radius 51.1 cm, spin axis Y).
`BP_CityCar` is now that body with four `StaticMeshComponent` wheels - `Wheel_FL/FR/BL/BR`
at (+-120/-110, +-80, 51.1) - and a Tick that:

- ramps `CurSpeed` toward the target with `FInterpTo` instead of starting at full speed,
- drops to 40% cruise within 900 cm of a waypoint, so corners are braked for,
- spins every wheel by `speed x dt / radius` in degrees, wrapped at 360,
- steers the front pair by the clamped yaw error (+-35 deg),
- gives each car its own `SpeedCmS` (390-560), so the spacing in the convoy actually changes.

`/Rover/SportsCar/SKM_SportsCar` is the richer option - it has `Phys_Wheel_*` bones and four
dampers - but driving those bones needs an AnimBP or Control Rig for a vehicle skeleton, and
node creation inside a state machine is refused by this toolset, so the four-component route
is the one that can actually be built here.

**Two toolset traps worth recording.** `ObjectTools.set_properties` writes only the **X**
component of an `FVector` on a component: `{"x":120,"y":-80,"z":51.1}` lands as
`(120, 0, 0)`, silently. And a `UserConstructionScript` written through `write_graph_dsl`
did not take effect on already-placed actors even after a compile. The wheel offsets are
therefore applied in `EventBeginPlay`, where they verifiably stick - confirmed in Simulate:
`Wheel_FL (120, -80, 51.1)`, `Wheel_FR (120, 80, 51.1)`, `Wheel_BL (-110, -80, 51.1)`,
`Wheel_BR (-110, 80, 51.1)`, each with a changing pitch between samples.

## Placement comes from the survey, not from guesses

`tools/citylife_layout.py` reads `demo/out/citymap/street.npz` and
`occ_day_highband_15to55.npz` and reuses `pavement_spots`' rule — street AND touching a
building — banded 7–20 m off the route line. 51 cells qualify; 16 are used.
`NedToUnrealLinear` (`UnrealTransforms.h:25`) has no origin offset, so
`UE_X = ned_x*100`, `UE_Y = ned_y*100`, and the corridor lands at UE X ∈ [3200, 4600].

Verified before placing anything: traces down at (3400, −1600), (3800, 2500), (4400, 6600)
all hit road at z = 0, and a pavement cell at (2200, 2600) hits kerb at z = 15.

**A downward trace from 3000 cm is not the ground.** Three pedestrians (`Ped_03`, `_04`,
`_15`) landed at z = 1194, 722 and 726 — on the roofs and awnings of the very buildings
that made their cells count as pavement. Re-tracing from 250 cm put all three at z = 15.
The rule that finds a pavement is the rule that guarantees a roof overhead.

## Measured, not eyeballed

Simulate-In-Editor, 5 s warmup, when the level was first built:

- `Car_00` spawned at y = −1600, read y = 1683, then y = 5933. About 2.5 m/s, which was
  `SpeedCmS` before the wheeled rebuild.
- `Ped_00` spawned at (2200, 2600), read (2520, 3032), then (2414, 2988). It walks and it
  turns around, so the navmesh built and `Roam` re-armed.

Re-measured 2026-09-15 after the wheeled rebuild: every car's `CurSpeed` equals its own
`SpeedCmS` (390-560) except one braking into a corner (268 of 560), and `Wheel_FL` pitch
differs car to car, so the wheels turn.

**Open: cars do not keep their distance.** Each car holds its own speed on a shared route with
no gap-keeping, so faster cars catch slower ones: in that same snapshot `BP_CityCar_C_2`, `_C_3`
and `_C_4` (object names, not labels) sat within 4 m of each other on one leg. `BP_CityCar` is `QueryOnly` and moves with
an unswept `SetActorLocation`, so they overlap rather than collide.
- `LogNavigation` has one warning — `Recreating dtNavMesh instance … maxTiles` — which is
  runtime regeneration doing its job, not a failure.
- No `Blueprint Runtime Error` and no `Accessed None` in the log.

A zero delta would have been the thing to chase: a pedestrian that never moves and a
pedestrian with no navmesh look identical from outside.

## Running it

```powershell
& "C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor.exe" `
  "PASBlocks\Blocks.uproject" /Game/CityLife/Maps/CityLife_Day -game -windowed -ResX=1280 -ResY=720
```

Close the authoring editor first: the demo scripts match on `Blocks.uproject` and will kill it
(`TUTORIAL.md:176`).

To re-derive the placements: `python tools/citylife_layout.py` writes
`demo/out/citylife/placement.json`. `PASBlocks/` is gitignored, so the level itself is not
in the repository — this file and that script are what reproduce it.
