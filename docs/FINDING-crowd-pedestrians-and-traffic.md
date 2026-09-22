# The light people were already built, and the cars were silver because their paint never compiled

Follow-up to [FINDING-citylife-level.md](FINDING-citylife-level.md), done 2026-09-21 through
the editor's MCP server. Two complaints drove it: the MetaHuman figures tried on 2026-09-16
looked real but moved badly and made the editor crawl, and the cars did not read as traffic
(the 16 Sep meeting: *"all one colour, which makes tracking hard"*).

`PASBlocks/` is gitignored, so there is no git undo for any of this. Before the first edit,
`CityLife/` (13 files) and `Car/` (16 files) were copied to
`PASBlocks/_backup/citylife_2026-09-21/`, outside `Content/` so the asset registry never sees
them, SHA-256 checked.

## MetaHuman was rejected for its pipeline, not its polygon count

Sixteen people is nothing for an RTX 4090. What cost the frame time was how the MetaHuman
figure was wired, one of each per pedestrian:

- a second actor (`BP_Ada`) spawned in `BeginPlay` and attached to the pawn,
- a live `RetargetPoseFromMesh`, i.e. a second skeleton evaluated every frame,
- the face's `Face_PostProcess_AnimBP` running RigLogic on a face nobody can see from a drone,
- groom strands (`Hair_S_AfroFade` 37 MB, `Hair_S_Coil` 48 MB).

Nine presets came to 8.8 GB on disk. The motion faults seen on 2026-09-16 (feet sliding, then
an attach to the actor root instead of the mesh component) came from the same wiring, and
fixing them would still leave every cost above in place.

## `BP_CityPed_Human` existed since 2026-09-16 and was never placed

Read from the `.uasset` name tables before writing the plan, then confirmed in the editor:

- `BP_CityPed_Human` is a City Sample Crowd figure: `m_tal_nrw_base` body plus `Top`,
  `Bottom`, `Shoes`, `Head` skeletal mesh components following it. **Zero references to
  `SKM_Manny`.** It is a child of `BP_CityPed`, so it inherits `Roam`, the gait retime and the
  five-property acceleration table from the parent finding.
- **`SK_Base` carries a `CompatibleSkeletonList` that names `SK_Mannequin`.** This is the
  load-bearing fact and it was written down nowhere. Because of it, `ABP_CityPed_HumanM` and
  `BS_CityWalk_HumanM` (target skeleton `SK_Mannequin`, samples the crowd's own
  `MTN_N_Walk_F`, `MTN_N_WalkQuickly_F`, `MTN_N_Walk_InPlace`) drive the crowd body directly.
  No IK retarget, no second skeleton.
- The female path is complete too: `ABP_CityPed_HumanF` → `BS_CityWalk_HumanF` →
  `FTN_N_Walk_F`, `FTN_N_Walk_F_Quickly`, `FTN_N_Idle_Base`.

All 16 placed pedestrians were still `BP_CityPed` (Manny). Nothing needed downloading.

Validated before building on it: one instance in Simulate moved about 280 cm in 2 s
(`MaxWalkSpeed` 140), legs stepping in consecutive frames, not a bind pose. Its
CharacterMovement read back the parent's tuning exactly (`MaxAcceleration` 200,
`BrakingDecelerationWalking` 260, `BrakingFrictionFactor` 0.5, `bUseAccelerationForPaths`,
`FixedPathBrakingDistance` 90).

## The follower meshes followed the template, not their own body

Silent defect: after a recompile the clothes and head of a freshly spawned figure were not on
the figure. The SCS template's `LeaderPoseComponent` is a weak pointer, and it pointed at the
**CDO's** `CharacterMesh0`. Instancing the template does not remap it to the instance's own
mesh, so every follower tracked a mesh that never moves.

Fix: `LeaderPoseComponent` is `None` on all four templates, and `BeginPlay` sets it at run
time, after the parent call so `Roam` still starts:

```
(event EventBeginPlay
  (bind _mesh (Variables|Character|GetMesh))
  (|Parent:BeginPlay)
  (Components|SkinnedMesh|SetLeaderPoseComponent (Variables|Default|GetTop) _mesh)
  ... GetBottom, GetShoes, GetHead)
```

Actors placed before the fix kept the stale pointer and had to be deleted and re-spawned;
recompiling did not repair them.

## What one figure costs now

| Component | Setting | Why |
|---|---|---|
| `Head` | `bDisablePostProcessBlueprint = true` | RigLogic off. The face is a few pixels from the air. |
| `Head` | `forcedLodModel = 4` (LOD3 of 8) | Blendshapes live only at LOD0. |
| `Hair` (new) | static hair-card mesh `..._CardsMesh_Group0_LOD2`, parent `CharacterMesh0`, `NoCollision` | Under 1 MB instead of a 37-48 MB groom; no collision so it cannot intercept a drone trace. |

`VisibilityBasedAnimTickOption` was left alone: Manny was already `OnlyTickPoseWhenRendered`,
so it is not a difference between the two.

## Six variants

Duplicates of `BP_CityPed_Human` in `/Game/CityLife/Blueprints/`, parent still `BP_CityPed`.

| BP | Body / AnimBP | Face | Hair | Top / Bottom / Shoes |
|---|---|---|---|---|
| `BP_CityPed_M1` | `m_tal_nrw_base` / `HumanM` | `m_001` | AfroFade | buttonDown_tie_blazer / slacks_belt / oxfords |
| `BP_CityPed_M2` | `m_tal_nrw_base` / `HumanM` | `m_002` | CurlyFade | crewneck / jeans_belt / loafers |
| `BP_CityPed_M3` | `m_tal_nrw_base` / `HumanM` | `m_003` | SideSweptFringe | turtleneck_blazer / slacks / oxfords |
| `BP_CityPed_F1` | `f_tal_nrw_base` / `HumanF` | `f_001` | Coil | scoopneck_croppedJacket / jeans_belt / loafers |
| `BP_CityPed_F2` | `f_tal_nrw_base` / `HumanF` | `f_002` | Pixie | buttonDown_blazer / skirt / dressFlats |
| `BP_CityPed_F3` | `f_tal_nrw_base` / `HumanF` | `f_003` | LowPonytail | turtleneck / slacks_belt / oxfords |

`Ped_00…Ped_15` were swapped round-robin M1, F1, M2, F2, M3, F3. Name, tag `citylife.ped`,
folder `CityLife/Peds` and transform were kept (the name is the segmentation class).
`MaxWalkSpeed` per figure: 126, 140, 154, 133, 147, 130, 144, 151, 128, 137, 149, 135, 142,
131, 153, 139 cm/s.

Measured in Simulate, two snapshots: **12 and 15 of 16 walking, 0 in the carriageway, 0
fallen**, with speeds between 0 and `MaxWalkSpeed` (67, 97 cm/s) that only an accelerating
mover produces. Before, on Manny: 13 and 11 of 16. Same order, so the swap did not break
`Roam`. It is not evidence of an improvement either.

**Not done:** the plan's random `SetGlobalAnimRateScale` (0.93-1.07) to break step phase-lock
between figures.

## The cars were silver because `M_CarPaint` did not compile

Silent defect. `/Game/Car/Assets/M_CarPaint` is the parent of every `MIC_Paint_*`, and it
failed to compile (`Missing input texture`): two `TextureSample` nodes had no texture. A
material that fails to compile renders with the engine default, so every paint instance came
out the same grey whatever `PaintColor` said. Setting a colour per car could never have
worked until the parent compiled. The fix:

- `TextureSample_0` (Normal sampler) = `T_CarPaintFlakes_N`, `TextureSample_2` (LinearColor)
  = `T_Smudges_R`, recompile.
- `MIC_Paint_Orange` had its `PaintColor` override switched off, so it showed the parent's
  pale blue. Now on, (0.85, 0.35, 0.02).
- New `/Game/CityLife/Materials/MIC_Paint_Purple` (0.55, 0.02, 0.5) and `MIC_Paint_Gold`
  (0.85, 0.5, 0.02), scalars copied from the stock instances.

`Car_05/06/07` (body slot 1 `CarPaint`) now carry Gold, Purple and Black. `Car_00-04` are the
`VehicleVarietyPack` bodies in their own paint.

Hue measured from one overhead frame (5200 cm), OpenCV scale 0-179, median over a
28 x 28 px box on each body, against `demo/follow_vlm.py` `COLOUR_HUE`:

| Car | Paint | Measured | Band |
|---|---|---|---|
| `Car_00` | box truck, white | 106.7 | not confirmed: in building shadow, and white has no hue |
| `Car_01` | hatchback, blue | 106.2 | blue |
| `Car_02` | pickup, orange | 21.7 | orange |
| `Car_03` | SUV, green | 52.1 | green |
| `Car_04` | sports car, red | 169.3 | not confirmed: 0.7 below red's 170, in building shadow |
| `Car_05` | Gold | 31.6 | yellow |
| `Car_06` | Purple | 143.6 | purple |
| `Car_07` | Black | 3 of 784 px chromatic, 581 dark | black |

So six of eight are confirmed from one frame. The picker's hue is not the rendered hue:
`MIC_Paint_Yellow` (hue 25.8 by its sRGB value) rendered at 37.5 from overhead, inside the
green band, which is why `Car_05` got the new Gold. The new purple, 151.6 by value, rendered
at 143.6. Choose paint from a measured frame.

## Cars keep their distance

The parent finding's open bug: faster cars caught slower ones on the shared loop and
overlapped. `BP_CityCar` now has a function `UpdateEffSpeed`, called first on every Tick, that
feeds a gap-limited target speed to the existing `FInterpTo` in place of `SpeedCmS`:

```
gap    = nearest car ahead: dot(d, forward) > 0 and lateral^2 < 250^2
sStar  = 700 + 1.8 * CurSpeed          # standstill gap + ~1.8 s headway, cm
f      = clamp(gap / sStar, 0, 1)
EffSpeed = SpeedCmS * f * f
```

It is IDM-lite: no explicit braking term, just a target speed that falls off with the square
of the gap ratio. The move direction is now `GetActorForwardVector` rather than the normalised
vector to the waypoint, so a car travels where its nose points and corners on its turn rate.

Every intermediate is latched into a member variable by an impure `Set`, because a pure node
feeding several pins is re-evaluated at each pull, and inside a `ForEach` that makes the value
depend on evaluation order.

Measured **inside the engine**, one Simulate of about 40 s, 112-123 ticks per car: nearest
other car per car **621-771 cm**, and **0 ticks under 400 cm** for any car. No car stopped for
a whole sample, and no car strayed more than 155 cm from the route line.

Two caveats that go with the number. The world was ticking at about 2.8 Hz (see below), so
this is 2.8 Hz sampling; cars in one lane close at no more than 170 cm/s relative, about 60 cm
between ticks, so a sub-400 cm approach would still show. And there is no control run with
gap keeping off using the same in-engine meter, so "before" is only the parent finding's
snapshot.

### That "before" number is suspect

The parent finding said three cars sat "within 4 m of each other". Measuring with the same
method here gave a minimum of 12 cm and 15 pair-samples under 4 m **with gap keeping already
on**, and the cars' own computed gaps disagreed with the snapshot's by 140-335 cm. The cause was the
meter, not the cars: `execute_tool_script` is not atomic. **Each `execute_tool` call lets the
game advance a frame**, so a "snapshot" of eight positions spans eight frames. At 400-560 cm/s
that is metres of error. The 4 m figure was taken the same way and should be read as "cars
bunch up", not as a distance.

## What is not measured

- **Frame cost.** The tick rate read 2.8/s with the 16 figures visible, 2.8/s with them
  hidden, and 2.8/s with `bThrottleCPUWhenNotForeground` switched off in memory (then
  restored). Something else paces this world, probably the ProjectAirSim clock, so the A/B
  says nothing about the crowd. The real test is `-game` plus a flight, reading `det_hz` (gate
  4.0) and the control loop (gate 9.5 Hz) from `metrics.json`, two runs per condition.
- **Detection.** OWL-ViT `a person` on one figure at a fixed ground point, camera at the same
  pose, n = 1 per cell, figures in their editor A-pose:

  | Figure | 10 m up | 20 m up |
  |---|---|---|
  | Manny | 0.062 | 0.027 |
  | crowd `M1` | 0.054 | 0.012 |
  | crowd `F2` | 0.105 | 0.013 |

  OWLv2 scored all six cells 0.008-0.022. The plan's gate was "at least 2x Manny"; that fails.
  A realistic figure did not make OWL-ViT see people. The numbers are consistent with
  `demo/detectors.py`: its scores on our pedestrians were already 0.03-0.07.
- **Wheels.** Whether the four `SM_Offroad_Tire` components from the template still render on
  the `VehicleVarietyPack` bodies was not checked in this pass.

## Toolset traps met on the way

- **Array properties.** `set_properties` refuses to change an array's length and its elements
  in one write (`ArrayAdd: elements changed alongside the size change`), and writing `"None"`
  does not clear an element. What works for `overrideMaterials`: `[]`, then
  `["None","None"]`, then `["None", <mic>]`.
- **Struct properties.** For `bodyInstance`, read the whole struct, change
  `collisionEnabled` / `collisionProfileName`, write it back, and check no field went missing.
- **A failed property inside a script fails the whole script**, even inside try/except. Ask
  only for properties the class has.
- **`CaptureViewport` with a `captureTransform`** gives a frozen or artefacted pose, and several
  captures in one script dropped meshes. Use `SetCameraTransform`, capture the main viewport
  with a null transform, and give it a moment. Capture with no annotations when measuring
  colour: the annotation text is yellow.
- **The graph DSL** cannot recreate IDs it decompiles, such as `Math|Vector|vector-vector` and
  `|Parent:BeginPlay`. Use the generic operator `(- a b)`, and splice into existing graphs with
  `create_node` / `connect_pins`. Literals become `MakeLiteralFloat` nodes, and the decompiler
  only prints nodes reachable from an entry.
- **`AssetTools.exists` / `get_asset_class` / `get_dependencies`** report existing assets as
  missing; `load_asset` is reliable. SCS components are not on the CDO; address them as
  `<BP>_C:<Comp>_GEN_VARIABLE`.
- **The MCP client can go stale.** After the editor restarts, the session's `unreal-mcp`
  client may stay `ConnectionRefused` while the endpoint answers. Plain JSON-RPC to
  `http://127.0.0.1:8000/mcp` (`initialize`, `notifications/initialized`, `tools/call`) works.

## Left behind

- `BP_CityCar`'s EventGraph holds 111 unreachable nodes from earlier edits. They do nothing but
  make the graph hard to read.
- Debug variables on `BP_CityCar`: `DbgMe`, `DbgFwd`, `DbgN`, `MinNow`, `MinEver`,
  `CloseTicks`, `Ticks`. They cost a few float writes per tick and are what the in-engine meter
  above reads, so they stay until a flight has been measured.
- `BP_CityPed_Ada` (the MetaHuman attempt) is still on disk, unused.
