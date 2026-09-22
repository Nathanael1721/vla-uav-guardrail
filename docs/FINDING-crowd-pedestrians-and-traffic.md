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

---

# Second pass, 2026-09-22: the figures had no hands, and the city was one street wide

The first pass put crowd figures in the level. Flying it showed two faults and
one limit: the people had no hands, their hair trailed behind their heads, and
the whole environment was a single 80 x 10 m block of street.

## The body was never there: `m_tal_nrw_base` is a 3-vertex stub

`BP_CityPed_Human` drove `CharacterMesh0` with `m_tal_nrw_base` and dressed it
in `Top`, `Bottom`, `Shoes` and `Head`. Nothing in that list is skin. The base
mesh is **3 vertices at every one of its 4 LODs**, and its one material slot is
named `M_Hide`, carrying `M_DebugPink`; an asset thumbnail of it renders empty.
It is a pose driver, not a body - which is why the arms ended at the wrist,
where the sleeves stop.

Epic's own `BP_CrowdCharacter` pairs that stub with a second mesh, and so does
`CrowdCharacterDataAsset`: every body definition carries `base`
(`m_tal_nrw_base`) *and* `body` (`m_tal_nrw_body`, 10,648 vertices, real skin,
hands included). We had only ever attached the first.

Fixed by adding a `Body` SkeletalMeshComponent to the six variants and the
template, parented under `CharacterMesh0`, and leader-posed in `BeginPlay`
exactly like the clothes:

| Component | Mesh | Skeleton |
|---|---|---|
| `CharacterMesh0` (leader) | `m_tal_nrw_base` / `f_tal_nrw_base` | `SK_Base` |
| `Body` (new) | `m_tal_nrw_body` / `f_tal_nrw_body` | `metahuman_base_skel` |
| `Head` | `m_001_nrw_FaceMesh` and siblings | `Face_Archetype_Skeleton` |
| `Top` / `Bottom` / `Shoes` | garment meshes | `SK_Base` |

**Leader pose does not require the same skeleton.** It maps bones by NAME. The
face was already riding a different skeleton (`Face_Archetype_Skeleton`) and
following correctly, which is the evidence that settled it before the body mesh
was attached. `m_tal_nrw_body` is on `metahuman_base_skel` and follows too -
verified walking in Simulate, hands visible and moving.

There is no separate hand or glove mesh anywhere in the pack; searching for one
is a dead end.

## The hair was parented to the mesh, not to the head

The `Hair` static mesh hung off `CharacterMesh0` with an identity relative
transform and no socket, so it followed the component - the capsule, in effect -
and not the `head` bone. Standing still that looks right; walking, the head bobs
and turns and the hair does not, which reads as lag.

`BeginPlay` now calls `AttachComponentToComponent(Hair, GetMesh(), "head",
KeepWorld, KeepWorld, KeepWorld)`. `SK_Base` has no head SOCKET - Epic attaches
to the bone named `head` directly - and the hair cards are authored in character
space, so `KeepWorld` is what makes this work without hand-computing an offset:
at `BeginPlay` the mesh is in its reference pose, the hair is already in the
right place, and re-parenting under that rule keeps it there.

## The name a segmentation mask carries is not the name in the outliner

*Retraction.* `docs/FINDING-citylife-level.md` said "an actor's name IS its
segmentation class" and treated `Ped_NN` / `Car_NN` as load-bearing. Half of
that is right. The plugin's `GetSegmentationName` returns
`mesh->GetOwner()->GetName()` - the INTERNAL object name. For a Blueprint placed
in a level that is `BP_CityPed_M1_C_1`, never `Ped_00`, because `Ped_00` is the
editor LABEL, and labels do not exist in a `-game` build at all.

The same applies to the simulator's pose lookup: `WorldSimApi::getObjectPose`
resolves a name through `UnrealHelpers::FindActor`, which matches
`GetName().Contains(name)` **or an actor TAG**. So every pedestrian and car now
carries its intended name as a tag (`Ped_00` to `Ped_39`, `Car_00` to
`Car_15`), and `world.get_object_poses(["Ped_07", ...])` resolves. A miss is not
silent: `getObjectPose` returns NaN for an actor it cannot find.

## Ground truth for a level that owns its own crowd

`demo/follow_vlm.py` scored pedestrians against `people.figures` - objects the
CLIENT spawned and teleports. A level that walks its own figures has no such
object, so `subject_truth_pts` returned `[]` on every tick and the whole
pedestrian phase scored UNSCORABLE, with nothing saying so.

`demo/level_actors.py` (new) asks the simulator instead:
`World.get_object_poses(names)` returns global NED, which is what the flight log
already uses. It duck-types `Pedestrians` closely enough to drop in
(`figures`, `update`, `stats`, `destroy`), refuses the flight when no name
resolves, keeps the last position on a NaN rather than teleporting truth to the
origin, and throttles polling (`--level-truth-period`, default 0.1 s) because
the simulator loops the names on the game thread. `tests/test_level_actors.py`
covers all of that: 10 tests.

`follow_vlm.py` gains `--level-peds N`, refuses it together with
`--pedestrians`, and records where the truth came from in
`metrics.json.pedestrian_truth`. `scripts/run_citylife_follow.ps1` runs the
mission against the level's own crowd with no client-side scenery at all.

## Corners: the aim point moves, so the heading does not jump

`BP_CityCar` steered at `Route[Idx]` and switched waypoint inside
`ArriveRadius`, so at a corner the target yaw stepped by up to 90 degrees and
`RInterpTo` swung the car through it - a pivot, not a turn. The flat
"40 % speed within 900 cm" rule braked the same distance out whatever the speed.

New function `UpdateAim`, called from Tick right after `UpdateEffSpeed`:

```
blend    = 300 + 2.2 * CurSpeed          # lead-in distance, cm
CornerF  = clamp(1 - dist_to_waypoint / blend, 0, 1)
AimPt    = waypoint + normalize(next - waypoint) * (450 * CornerF)
TargetSpeed = EffSpeed * (1 - 0.5 * CornerF)
```

Tick's look-at now reads `AimPt` and its `FInterpTo` reads `TargetSpeed`. The
aim slides up to 4.5 m into the next leg as the car arrives, so the heading
target moves continuously and the car arcs.

**The first version of this deadlocked the whole fleet**, and the reason is
worth keeping. `AimPt` was `waypoint + (next - waypoint) * 0.45 * CornerF` - a
fraction of the WHOLE leg, which on a 170 m leg is a 75 m shortcut. Cars cut
across the corner, piled into each other, and then the gap rule did the rest:
every car saw another within 250 cm laterally and 0 cm ahead, so every car
braked to a stop and stayed there. Two fixes, both kept:

- the lead-in is a fixed distance (450 cm), not a fraction of the leg;
- a car only brakes for cars **facing the same way**
  (`dot(other.forward, my.forward) > 0.5`), and `EffSpeed` has a 60 cm/s floor.
  At a corner two cars can each be geometrically "ahead" of the other; without
  the heading gate that is a mutual stop with no way out.

## The city was never one street wide

The map is a modular grid: **66 junctions, 11 columns x 6 rows on an 82 m
spacing**, about 1000 x 590 m of built road. The old circuit was a hand-typed
80 x 10 m rectangle that touched none of them, and `tools/citylife_layout.py`
only ever saw a 160 x 160 m window of occupancy data, which is why its pavement
pool was 51 cells.

Now, derived from that grid rather than typed:

| | Before | Now |
|---|---|---|
| Car routes | one 80 x 10 m rectangle | two loops on the junction grid: 684 m and 300 m |
| Cars | 8 | 16 |
| Pedestrians | 16 | 40 |
| Nav bounds | 48 x 104 m | 200 x 185 m |
| No-walk bands | 2, hand-fitted to the old loop | 8 (3 N-S + 3 E-W carriageways + the original 2) |

Lanes are offset 350 cm to the LEFT of travel from the junction centre line,
because this is a Japanese city and its traffic keeps left; the two loops share
streets in opposite directions 7 m apart, which is what makes them read as
two-way traffic. Pavement positions come from tracing the ground 1000 cm either
side of each centre line and keeping hits between 6 and 40 cm - the road tile's
own sidewalk sits at z = 7-10, the carriageway at z = 0.

Measured in Simulate, 45 s, after the change:

- **40 pedestrians, 32 walking, 0 in any carriageway**, speeds spread 36-154 cm/s.
- **16 cars, nearest other car 567-1544 cm, 0 ticks under 4 m**, each at or
  under its own `SpeedCmS`, cars in corners at 64-70 % of it.
- The editor world ticked at about 5.8 Hz during that run with 56 actors
  driving, against 2.8 Hz measured in the first pass with 24. That number says
  nothing about `-game` throughput; it is an editor with a viewport, and the
  earlier 2.8 Hz was measured with the window in the background.

## Still not measured, still not done

- **No flight yet.** Everything above is Simulate-in-editor. `det_hz`, the
  control loop and the crowd's real frame cost need `-game` plus
  `scripts/run_citylife_follow.ps1`.
- `UpdateEffSpeed` still calls `GetAllActorsOfClass` per car per tick, and that
  walks every actor in the world (about 5,900) before the 16-car loop. At 16
  cars that is roughly 95,000 class tests a tick. It has not been profiled in
  `-game`; caching the array in `BeginPlay` is the obvious fix if it shows up.
- Nobody crosses a road: the no-walk bands split the pavements into islands.
  `SM_jcGrdCrosswalkA` exists in the content and the junctions have crosswalk
  markings painted, but no `NavLinkProxy` is placed, so a crossing is a path the
  navmesh does not have.
- Eyebrows are absent from the crowd figures (they are a separate groom in
  Epic's pipeline). Unverified whether that reads at 10-20 m.
- The wheel components on `BP_CityCar` carry no mesh at all, so the "are the
  buggy tyres still there" question from the first pass is answered: they are
  not rendering anything.
