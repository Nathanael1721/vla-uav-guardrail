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

*Since 2026-09-23* `Car_04` and `Car_22` (the red VVP sports cars) are TP bodies painted
blue and white, so `Car_10` (TP body, `MIC_Paint_Red`, loop A at 3.2 m/s) is the only red
car in the level. The hue table below is the 2026-09-21 frame.

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

*Superseded in part 2026-09-23:* `UpdateEffSpeed` still runs (every other tick), but it
feeds `DriveTick`, not an `FInterpTo`, and a car ahead going the same way now stops the car
6.5 m behind it, centre to centre. Heading follows the path's curvature, not a fixed turn
rate. See the third part of this document.

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
`Car_15`; `Car_23` since the 24-car change later that day), and `world.get_object_poses(["Ped_07", ...])` resolves. A miss is not
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

*Superseded 2026-09-23.* `UpdateAim` is deleted, and the Tick below (corner-waypoint
steering, fixed-rate `RInterpTo` yaw, `ArriveRadius`) is replaced by `DriveTick`:
path-curvature feed-forward plus lateral/heading correction on dense lane-centre polylines
from `tools/citylife_routes.py`. Of the two deadlock fixes below, the 450 cm lead-in went
with `UpdateAim`; the heading gate survives, joined by a queue stop, junction give-way and
pedestrian yield, and the 60 cm/s floor now applies only to a crossing car in the corridor.
What follows is the 2026-09-22 model, kept as a record; the third part of this document
describes the new one.

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

| | Before | 2026-09-22 (first pass) |
|---|---|---|
| Car routes | one 80 x 10 m rectangle | two loops on the junction grid: 684 m and 300 m |
| Cars | 8 | 16 |
| Pedestrians | 16 | 40 |
| Nav bounds | 48 x 104 m | 200 x 185 m |
| No-walk bands | 2, hand-fitted to the old loop | 8 (3 N-S + 3 E-W carriageways + the original 2) |

Since later on 2026-09-22 there are 24 cars on three loops. Since 2026-09-23 the loops are
dense lane-centre polylines from `tools/citylife_routes.py`: A 616 m / 408 points / four 7 m
left-turn arcs, B 334 m / 224 points / four 13 m arcs, C 288 m / 192 points / 7 m arcs, each
point with a speed limit from 1.8 m/s^2 lateral acceleration and a 1.5 m/s^2 braking ramp.

~~Lanes are offset 350 cm to the LEFT of travel from the junction centre line,
because this is a Japanese city and its traffic keeps left;~~ *Retracted
2026-09-23:* the 2026-09-22 lanes sat 350 cm to the RIGHT of travel. UE axes here
are X = north, Y = east, and the eastbound lane was at X = 3750, the south half of
the road, so the cars drove on the right, against the level's own lane markings:
on the western approach to junction (4100, 4100) the stop line and lane arrows are
on the north half and point east, which is keep-left. Since 2026-09-23 cars keep
left (`tools/citylife_routes.py`, `DRIVE_SIDE = "left"`). The two loops share
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

## It was flown, four times, and the first tracking scores were computed in the wrong frame width

Four flights on 2026-09-22, `scripts/run_citylife_follow.ps1`, 180 s each,
following `"a person"` with the level's own crowd as ground truth. The Unreal
editor has to be closed first; the runner refuses to kill it.

| run | truth | poll | ticks | det_hz | loop_hz | on target | chance | median err px in shot (chance) | sep min |
|---|---|---|---|---|---|---|---|---|---|
| `citylife_follow` (deleted, see below) | 16 of 40 | 0.1 s | 952 | 5.82 | 5.29 | ~~0.063~~ n/a | ~~0.571~~ n/a | ~~332.6~~ n/a | 0.8 m |
| `citylife_follow2` | 40 | 0.1 s | 728 | 7.60 | 4.04 | 0.255 | 0.802 | 128.2 (31.9) | 4.0 m |
| `citylife_follow3` | 40 | 0.5 s | 1239 | 4.63 | 6.89 | 0.365 | 0.808 | 113.5 (23.3) | 4.5 m |
| `citylife_city` (24 cars, crossings) | 40 | 0.5 s | 1110 | 4.82 | 6.17 | 1.000 | **1.000** | 9.2 (10.1) | 5.7 m |

Tracking columns re-scored 2026-09-23 with the frame width recovered from
`detections.jsonl` (`track_truth.load_rows`). The first scoring fell back to a 400 px
frame while the camera was 768 x 432, and gave 0.739 / 0.721 / 0.821 on target. The error
column is now the in-shot median beside its null; the first version of this table showed
the whole-flight median. `citylife_follow` was scored in the same wrong frame, and its log
is deleted, so its struck figures cannot be re-scored and should not be quoted.

**`det_hz` clears its 4.0 Hz gate in all four. The control loop clears 9.5 Hz in
none of them** - and it did not in the reference flight either
(`retarget_smooth`, 8.33 Hz, on the old level).

Three things the table says that are worth more than the numbers:

1. **The first run asked for the wrong truth.** The
   runner defaulted to 16 level pedestrians while the level walks 40, so the
   subject the detector locked was not in the truth list and every tick scored
   as a miss. The fix was a flag, not a model: `-LevelPeds 40`. A default that
   silently describes a smaller world than the one being flown is exactly the
   failure this repo keeps writing findings about.

   **Its artefacts were then deleted.** The flight log is not wrong about what
   the aircraft did; it is wrong about the world, because it carries truth for
   16 figures out of 40. Anything that scores `demo/out/` - the eval generator,
   `tests/test_track_truth.py` - would have read it as a detector that missed,
   and produced a plausible, wrong number from a complete-looking artefact. The
   row above is the record; the 887 MB of frames are not. Its 0.063 was also
   computed in a 400 px frame on a 768 px camera, like every CityLife score of that
   day, and with the log deleted it cannot be re-scored: it says nothing about the
   detector. The wrong truth list is established by the runner's default, not by
   that number.
2. **Polling the truth is not free.** 40 names at 10 Hz is 400 game-thread
   round trips a second, and the control loop paid for it: 4.04 Hz at a 0.1 s
   period against 6.89 Hz at 0.5 s. The runner now defaults to 0.5 s, which is
   still finer than a 1.4 m/s subject moves between polls.
3. **The tracking columns were first scored in the wrong frame width.**
   `demo/track_truth.py` fell back to `img_w` = 400 while the camera was
   768 x 432. Re-scored (table above): on `citylife_follow2` and `_follow3` the box
   was mostly NOT on a person - 0.255 and 0.365 on target against nulls of 0.802
   and 0.808, medians 128.2 and 113.5 px against 31.9 and 23.3 - worse than chance,
   which the video had shown and the first numbers had not. On `citylife_city`
   on-target and its null both saturate at 1.000 and the median, 9.2 px against
   10.1, is at chance: there the class-level score cannot tell skill from chance.
   The earlier 0.821 and "45.5 px against a chance of 5.2 px" were artefacts of the
   width. These logs carry no `truth.names`, so no instance-level score exists for
   them; `target_lock`'s 515 ticks held and 19 switches count the lock, not whether
   it was on a person. **Any future tracking claim about this level has to be
   scored against the LOCKED instance, not the class**: rows logged since
   2026-09-23 carry names and `track_truth.score_instance` does that scoring.
   `tests/test_track_truth.py` briefly excused these flights as a "crowded
   scene"; that exception was written from the wrong-width numbers and has been
   replaced. Its second known shape is now a flight whose own presence check
   called the box ABSENT on most ticks (61 % and 63 % on `citylife_follow2` /
   `_follow3`), and `citylife_city` is no longer excused. `subject_truth_pts` returns
   every pedestrian, which was right for a scene with 12 and is wrong for one
   with 40.

The guardrail numbers are unaffected by that, because they do not use the
class truth: `p0_violation_escape_rate` **0.0** in all four runs, 649 P0 ticks
repaired in the last one, 1,070 repairs, no NFZ or altitude escape. None of it
is KPI-grade - `topology` is `projectairsim-single-host`, so it is
functional-rail evidence by construction.

Artefacts: `demo/out/citylife_city/` (metrics, kpi, flight log, replay bundle,
`citylife_city_demo.mp4`).

## Still not measured, still not done

- ~~**The control loop is under its gate.**~~ *Found 2026-09-23 (commit 37514c3):*
  every tick slept a full 0.1 s AFTER its work, so the period was work + 100 ms and
  no flight - including the 8.33 Hz reference - could reach 10 Hz. It now sleeps to
  a 0.1 s deadline and logs per-stage milliseconds; see the third part for the
  measured rate.
- **Scored against the class, not the instance.** For the 2026-09-22 flights it
  cannot be done: their logs lack `truth.names`. The class-level numbers in the
  table were re-scored in the correct frame width on 2026-09-23. Rows logged since
  then carry names and `track_truth.score_instance` scores the locked figure.
- `UpdateEffSpeed` calls `GetAllActorsOfClass`, which walks every actor in the
  world (about 5,900) before the per-car loop. It now runs on every OTHER tick,
  which halves that, but at 24 cars it is still roughly 70,000 class tests a
  tick. Caching the array in `BeginPlay` needs an array-of-object variable, and
  this toolset's `add_object_variable` makes single references only. Since
  2026-09-23 it also scans `BP_CityPed` when a crossing on the car's path is within
  17 points ahead; that extra cost has not been measured.
- ~~Nobody crosses a road~~ *Done 2026-09-22:* rather than place `NavLinkProxy`
  actors - whose `PointLinks` is a struct array, and struct-array writes through
  this toolset are unreliable - the no-walk bands are CUT. A 600 cm gap at each
  painted crosswalk (1100 cm either side of a junction centre) leaves the
  navmesh walkable straight across the carriageway, which is what a crossing is.
  Five gaps on the demo corridor's two junctions; measured in Simulate, 3 of 40
  figures were mid-crossing and 0 were anywhere else in a carriageway.
  ~~Nothing yields: a car drives through a crossing pedestrian, because
  `BP_CityCar` does not look for them.~~ *Since 2026-09-23* a car stops for a
  pedestrian on a crossing on its own path, measured along the path. In-engine
  (Simulate, editor throttled to ~3 fps, 4 min) cars logged 780 pedestrian-yield
  half-ticks, and 5 in which a car was on a zebra at > 50 cm/s with a MOVING
  pedestrian on it; the cause is not established.
- Eyebrows are absent from the crowd figures (they are a separate groom in
  Epic's pipeline). Unverified whether that reads at 10-20 m.
- ~~Cars do not yield at intersections, so two loops that CROSS drive through each
  other.~~ *Since 2026-09-23* cars give way at junctions where their path crosses
  another loop's, and loops A and B DO cross, at (4100, 4100) and (12300, -4100),
  where loop B gives way. Loop C was moved a block west on 2026-09-22, when no
  priority rule existed yet.
- The wheel components on `BP_CityCar` carry no mesh at all, so the "are the
  buggy tyres still there" question from the first pass is answered: they are
  not rendering anything.

---

# Third pass, 2026-09-23: the cars drove on the wrong side, turned by pivoting, and yielded to nobody

What the second pass left: cars that swung round corners rather than arcing,
nothing that gave way at a junction or a crossing, a tracking score computed in
the wrong frame width, and a flight in which the aircraft chased building
facades while its own presence check said ABSENT. Asked for: natural turns,
the loose ends closed, and a mission the drone can actually be judged on -
follow the red car, still through the VLA, OWL-ViT and the guardrail.

## The traffic drove on the wrong side, and a sentence here said it did not

The second pass wrote that lanes sit "350 cm to the LEFT of travel ... because
this is a Japanese city and its traffic keeps left". The intent was right; the
geometry was not. In this level X is NORTH and Y is EAST, so the right-hand side
of a heading (dX, dY) is (-dY, dX). The formula used for "left" put an
eastbound car on the SOUTH half of the street - its right. Every car drove on
the right, against the markings.

The markings were read, not assumed: a top-down capture of the western approach
to junction (4100, 4100) shows the stop line and the lane arrows (straight;
straight-and-right) on the NORTH half of the carriageway, pointing east.
Eastbound traffic keeps north, i.e. left. `tools/citylife_routes.py` now has
`DRIVE_SIDE = "left"` read off that image, and a test that right-hand geometry
reproduces exactly the lanes the 2026-09-22 cars drove, which is how we know
they were on the wrong side.

Keeping left changes more than the side. Loop A turns left at every corner, so
it now runs INSIDE its block on 7 m arcs (616 m a lap, was 684 m on the lane
rectangle); loop B turns right, so it runs outside on 13 m arcs (334 m) - and
B's right turns now cross the oncoming lane, which is A's. The loops that were
built "never to cross" cross at two junctions, (4100, 4100) and (12300, -4100).
That is ordinary traffic, and it is the reason the cars had to learn to give
way (below).

## Why the corners looked wrong, and what a car does now

`BP_CityCar` steered its body with a fixed-rate `RInterpTo` toward an aim point
(`UpdateAim`, second pass). The yaw rate therefore had nothing to do with the
path or the speed: the car turned at whatever rate the interpolator allowed,
and the corner speed came from the distance to a waypoint, not from how sharp
the turn was. A car moves along an arc and turns at v/R.

So the path became geometry, computed offline and tested
(`tools/citylife_routes.py`, `tests/test_citylife_routes.py`): each loop is a
dense lane-centre polyline, a point every 150 cm, whose corners are circular
fillets (7 m for a turn toward your own kerb, 13 m across the oncoming lane,
both checked to stay inside the carriageway and the junction box), with a speed
limit per point from 1.8 m/s^2 lateral acceleration and a 1.5 m/s^2 braking
ramp, and a curvature per point.

How to follow it was decided in Python before any Blueprint was written, with
`follow_step` as the reference the Blueprint mirrors node for node:

| follower | worst off-lane | worst v^2 kappa |
|---|---|---|
| pure pursuit, look-ahead 2.5 m + 0.6 s | 66 cm | 3.2 m/s^2 |
| curvature feed-forward + correction, segment advanced 120 cm early | 90-102 cm | 4.6 m/s^2 |
| same, advanced only when a point is PASSED | 5 cm | 2.3 m/s^2 |
| same, heading error against the arc TANGENT, not the chord | **4.7 cm** | **1.85 m/s^2** |

(Loops A-C at 60 Hz, L = 5 m; at 10 Hz the last row is 19 cm and 2.0 m/s^2.) Pure pursuit turns
early because its look-ahead point enters the arc before the car does. The law
kept is

    kappa = kappa_path - e_y / L^2 - 2 zeta e_psi / L,   L = 5 m, zeta = 0.9

a second-order correction in DISTANCE, so its behaviour does not depend on speed
or frame rate; on an arc the feed-forward alone turns the car at exactly v/R.
The front wheels are steered 2.7 m x kappa (radians, small-angle), capped at 35 degrees.

`DriveTick` replaces the old Tick chain (154 nodes of it, and `UpdateAim`, were
deleted). Measured INSIDE the engine - counters each car keeps, because an
outside snapshot of 24 cars takes 70 editor calls and is not a snapshot:

| Simulate run | worst off-lane | worst v^2 kappa |
|---|---|---|
| first DriveTick | 63 cm | 2.52 m/s^2 (pinned at the curvature clamp) |
| with frames cut into <= 50 ms sub-steps | **10.1 cm** | **1.93 m/s^2** |

The first run was wrong for a reason Python then reproduced exactly: the editor
hitches, and one 0.5 s frame at the end of an arc drives 1.8 m on the wrong
curvature. `follow` (the sub-stepping frame loop) and a test with every 20th
frame at 0.5 s now pin that down.

## Giving way

`UpdateEffSpeed` (every other tick, it walks every car) now decides three
limits, and `DriveTick` shrinks the stop distances by the distance driven in
every sub-step, so a slow frame cannot carry a car past a stop line before the
next measurement:

- **Queue.** Stop 6.5 m centre to centre behind a car going my way, on a
  2.5 m/s^2 ramp. The old rule never went below 60 cm/s and so crept into a
  stopped car. A car NOT going my way counts only inside my 2.5 m corridor: a
  "within 9 m" allowance meant for curves once caught oncoming cars in the
  other lane and held the subject at 0.6 m/s.
- **Junction.** Wait at the box edge while a non-parallel car is inside the
  box. Where my path crosses another loop's and I am the one turning across
  (`give_way_junctions`: only loop B, only at the two real crossings), also wait
  for a non-parallel car within 20 m of the box. A car already inside a box
  never waits, so the waiting condition always clears: no deadlock by
  construction. Giving way at every far turn was tried first; it held loop B at
  (4100, -4100), where its path never meets A's, behind a queue that was itself
  waiting.
- **Crossing.** Stop 6.5 m short of a crossing on MY PATH - measured along the
  path by index (`crossings_on`), because a car about to turn left must stop
  for the crossing round the corner, not the one straight ahead that it never
  reaches (the first version did exactly that). The pedestrian scan runs only
  within 25 m of such a crossing.

Pedestrians roam to random nav points and the crossings are nav, so a figure
sometimes stops ON a zebra, idles 2-6 s or sticks; one froze a junction for
33 s. A standing pedestrian therefore holds a car for 6 s at most; a walking
one holds it as long as it walks.

Measured in Simulate, 4 minutes, 24 cars (editor throttled to about 3 fps):

- worst lane error 10.1 cm, worst lateral acceleration 1.93 m/s^2;
- **closest approach between any two cars 650 cm** centre to centre - the
  queue distance; no car went through another;
- longest stand-still 22.3 s (a loop-B car queued at its give-way junction), no deadlock;
- 780 half-rate ticks of cars waiting for pedestrians, and **5 where a car was
  on a zebra, moving, with a walking pedestrian on it**. Not zero, and the cause
  is not established. The likely one is that the figures do not look: they step
  out in front of a car already inside its braking distance. It is reported, not
  fixed.

## Exactly one red car

Three cars on loop A were red: `Car_10` (the TP body in `MIC_Paint_Red`) and
`Car_04`, `Car_22` (the Vehicle Variety Pack sports car, red by default).
"Follow the red car" with three is ambiguous and the colour gate cannot choose.
`Car_04` is now a blue TP car and `Car_22` a white one
(`tools/citylife_mcp/one_red_car.py`). Paint hue, OpenCV scale, from each paint's
`PaintColor`: Red 0, Orange 18, Gold 22.5, Yellow 26, Green 71, Cyan 92, Blue 112,
Purple 152; the gate's red band is 0-10 and 170-179, so Red is the only paint in
it. `Car_10` drives loop A at 3.2 m/s, under the aircraft's 4 m/s `--speed-max`.

## The red-car mission: six flights, and what each one isolated

`scripts/run_citylife_follow.ps1 -Object "a red car" -LevelCar Car_10`. The
steering input is still only where OWL-ViT puts the box; the VLA follow loop,
the colour gate, the instance lock and the Guardrail Shield are the same ones
the pedestrian mission uses. Ground truth is the one car, by tag, so the score
has a single subject and its null cannot saturate the way forty pedestrians
did.

| run | what it isolated | start | on target (null) | median err px in shot (null) | in shot | within 30 m | outcome |
|---|---|---|---|---|---|---|---|
| (first, deleted) | the start gate as written | timed out after 300 s | - | - | - | - | crashed |
| `citylife_redcar_far` | acquisition mode, any range | 4 s, car at **140 m** | 0.083 (0.097) | 180.3 (145.9) | 0.262 | 0.095 | success |
| `citylife_redcar_pedpolicy` | acquire only within 45 m | 161 s, 13.8 m | 0.320 (0.250) | 57.3 (105.1) | 0.505 | 0.118 | success |
| `citylife_redcar_carpolicy` | the car's own policy | 153 s, 13.1 m | 0.418 (0.392) | 55.5 (81.6) | 0.674 | 0.256 | success |
| `citylife_redcar_ground` | + ground-contact check | 167 s, 12.9 m | **0.532 (0.414)** | **39.0 (84.9)** | **0.762** | **0.314** | success |
| `citylife_redcar_high` | cruise 12 m instead of 8 | 150 s, 16.8 m | 0.547 (0.487) | 20.0 (37.6) | 0.617 | 0.247 | **fail** |

All 240 s after the start gate; `p0_violation_escape_rate` 0.0 in every one;
control loop 9.31-9.37 Hz; detector 7.0-7.5 Hz except `_far` (4.27). "Outcome"
is the KPI file's verdict, and `_high` failed it on altitude (below). Scores are
`track_truth` at the real 768 px width, instance-level (one subject), so the
null is meaningful. None of it is KPI-grade: the topology is
`projectairsim-single-host`.

What the best flight (`_ground`, video `docs/video/citylife_redcar_ground.mp4`)
actually did: it held the red car **continuously within 30 m for 48.8 s from
acquisition, median 19.4 m behind it, over 133 m of street**. Then the car
turned left at junction (4100, 12300) and the aircraft did not. For the rest of
the flight it searched, found the car again when it came round the loop, and
lost it again. `_carpolicy` and `_high` show the same shape (47.5 s / 127 m and
46.6 s / 135 m). **Following along a street works; following round a corner
does not yet.**

### What each failure was

1. **The gate was captured by a fire-hydrant sign.** The first flight waited
   300 s and never saw the car. `detections.jsonl` shows one box for all 1,227
   inferences: a stationary red object 22 deg right of the nose, which the
   gate snapshot later showed to be a red 消火栓 (fire hydrant) sign on the
   kerb. The presence check rejected it every time (50 px at > 78 m is no
   car), which is right - but the Grounder's jump gate then dropped every
   candidate more than 35 % of the frame away from it, and the instance lock
   held it for 1,101 inferences. The car was never even a candidate. Before a
   subject is acquired there is nothing to be continuous WITH, so the gate
   now runs the Grounder in acquisition mode: no jump gate, no lock, every
   colour-passing candidate published, each judged by the presence verdict,
   N sightings in a row that stay pixel-continuous (`Acquirer`), and the lock
   seeded on the one confirmed. The same flight then crashed on its first
   tick: the truth update fell through to `car.pose_at`, which a scripted car
   has and a level car does not. Its artefacts were deleted, as with
   `citylife_follow`: an empty flight log would be scored as a flight.
2. **A car at 140 m is not a start.** `_far` acquired the right car in 4 s - at
   140 m, 16 px, driving away at 3.2 m/s from an aircraft that then flew at
   most 4 m/s. It never closed. The gate now accepts only a subject within
   45 m (depth, or the width prior without depth); the car passes the start
   point once a lap, so the gate waits for that pass (~150-170 s).
3. **The policy decided the result before the controller did.** `_pedpolicy`
   flew under `follow_pedestrian.yaml` for its 5 m catch-all stand-off, and
   inherited a person's envelope: a 3.0 m/s cap against a 3.2 m/s car (the
   Shield clamped speed on 406 ticks) and a 5 m clearance ring that a mapped
   obstacle 4.99 m from the start point violates (238 violations there; the
   controller's fence guard held the aircraft at 18 % of commanded speed for
   the first 800 ticks). `policies/follow_car_citylife.yaml` keeps both stand-offs and takes
   `follow_car.yaml`'s envelope: 5 m/s, 3 m clearance, 6-14 m.
4. **A red traffic signal passed for a red car.** In `_carpolicy` the lock
   left the car at the first junction for a red signal: 21 % red, ~35 px,
   plausible by width at ~40 m. It is not on the ground, and a car is: the
   ray through the bottom of its box meets the road about where the depth
   image says it is, while a signal 4-5 m up meets the road near twice as far.
   `presence_verdict` now checks that for things that stand on the road
   (car, truck, bus, van, person), with the camera's 20 deg mount and the
   body's pitch read from the pose each tick. `_ground` is the best flight on
   every tracking number, but **the check never fired in it** (no tick gives
   it as the reason), so that improvement over `_carpolicy` is run-to-run
   variation - traffic, pedestrians, where the car was when the gate opened -
   and not the check. Its corner loss has a different cause, next.
5. **At the corner the Shield held the aircraft against a wall that is not
   there.** In `_carpolicy` and `_ground` the aircraft stopped at the same
   place, the entrance to junction (4100, 12300), NED y = 111-114 m, for ~187 s
   and ~40 s of broken-up ticks respectively. The repair text says why:
   `bld-clearance: dist -35.06m < 3.0m -> push (-1.00,-0.03) at 5.00 m/s` -
   the obstacle map put the aircraft 35 m INSIDE an obstacle. Every map the
   Guardrail loads (`demo/out/citymap/*.npz`) is 80 x 80 cells of 2 m, NED
   [-80, 78] m: the Demo_day survey of 25 August, which never covered where
   CityLife's loops go. Beyond the edge `Shield._distance_at` extrapolates
   `edge value - distance past the edge`, and the edge cell at (48, 78) is a
   building, so everything north of x = 47 past the edge reads as ever-deeper
   inside it. The Shield swapped the follower's +3 m/s north for 5 m/s south,
   in bursts, and the aircraft oscillated between x = 40 and 47. (This part
   was first written up as the aircraft pressing into a corner tree and a
   signal pole, which the chase camera happened to show; the sim logged no
   collision there, and the repair text rules it out.)
6. **At 12 m the aircraft left its altitude envelope.** It overshot the car
   policy's 14 m ceiling for 3.1 s, peaking at **14.32 m**, although every
   command the Shield emitted was compliant (P0 escapes 0) and asked to
   DESCEND (vz -0.5 to -1.2 m/s throughout) - so the KPI file calls the flight
   a fail. Each climb coincides with a phantom-wall repair of point 5: a 5 m/s
   reversal of horizontal velocity, which pitches the airframe hard, and the
   pitch climbs it. The detector's hit rate also fell from 0.71 to 0.32 at the
   extra height, so 12 m was reverted to 8.

### Still open

- **Corners.** Two causes: the phantom wall of point 5, and a controller with
  no memory of where the car went - the estimator predicts along the last
  velocity (straight) for 3 s, then coast and search fly along the nose.
- **The maps do not cover the city**, and the Shield's off-map extrapolation
  turns a building on the map border into an endless wall (point 5).
- **A repair can reverse the aircraft at full speed** (point 6).
- The ground-contact check is untested against its own false-reject rate in
  flight: `_ground` logged no rejection by it, and offline, without the body
  pitch the logs do not carry, it cannot be evaluated.

### The control loop reaches 9.3 Hz

Every red-car flight ran at 9.31-9.37 Hz with a median 5.3-6.5 ms of work per tick, on
the same level where the 2026-09-22 flights managed 4.04-6.89 Hz. Nothing about
the level changed: the loop slept a full 0.1 s AFTER its work (commit
37514c3), so no flight - including the 8.33 Hz reference - could reach 10 Hz.
It now sleeps to a deadline. 9.3 is still under the 9.5 gate: the period is
~107 ms, and the extra ~7 ms is outside every timed stage - the likely cause
is the 15.6 ms default timer granularity of `asyncio.sleep` on Windows, which is
not measured yet.

## The level is rebuilt from scripts, because it is not in git

Everything above lives in `PASBlocks/`, which is gitignored. The record is the
scripts that build it, run in order through the editor's MCP endpoint
(`tools/citylife_mcp/`): `drive_tick.py` (variables, `DriveTick`,
`UpdateEffSpeed`), `rewire_tick.py` (EventGraph: Tick -> UpdateEffSpeed ->
DriveTick -> wheels; the old chain deleted), `apply_routes.py` (per-car path,
speeds, curvatures, junction flags, crossings, placement) and `one_red_car.py`;
`verify_drive.py` reads the counters. That was tested the hard way: the first
build was lost (below) and the second was produced from these scripts alone.

## Toolset traps met on the way

- **A full disk turns into a locked file.** `D:` filled while the editor was
  saving. The save failed, and every save after it failed too, with
  `MoveFile ... Error Code 32` - the editor itself kept the package files open.
  `save_assets` still answered `saved: true`; the files on disk kept their old
  date. The complete new packages were sitting in `PASBlocks/Saved/*.tmp`. Only
  a restart releases the handles. Check the file date, not the return value.
- A new Blueprint variable is invisible to the object API until it is
  instance-editable: `set_properties` refuses it by name.
- `write_graph_dsl`: `(Transformation|SetActorRotation x)` binds x to the
  target pin and fails; pass `:NewRotation`. Another actor's getter needs
  `:self actor`.
- A function call node's `type_id` reads back as `|Name`, while `create_node`
  wants `CallFunction|Name`.

# Fourth pass, 2026-09-24 to 09-29: a map of this city, a trail to follow, and three reviews

What the third pass left: every red-car flight lost the car at its first
corner; the maps the Shield checks clearance against were Demo_day's, not
CityLife's; the control loop ran at 9.3 Hz against a 9.5 Hz gate; nothing
recorded a collision; a car already on a zebra drove on whoever stepped out.
Asked for: finish the remaining bugs, keep testing the new level, and track
the vehicle the way the Demo_day demos did.

## The phantom wall was the Shield extrapolating a map that ended too soon

Point 5 of the third pass. Every obstacle map under `demo/out/citymap/` is the
160 m cube surveyed on Demo_day on 25 August: NED [-80, 78] m. CityLife's loop A
runs to y = 123 and x = 205. Past the edge `Shield._distance_at` returned
`edge value - distance past the edge`, and where the edge cell is a building
that is a wall reaching to infinity, deeper the further one flies - the
junction entrance at (47.2, 112.7) read **-35.16 m**, "35 m inside a building".

Two changes, and the first one was wrong in a way a review caught:

- **The bound.** Off the map the distance now lies between `v - off` and
  `v + off` (the field is 1-Lipschitz) and takes `off - res/2` once past a
  built-on border: a border building extends past the edge as far as it
  reaches into the map, not to infinity. The first version, `max(v - off,
  off)`, jumped from inside-the-building to +off at the last cell centre, so
  the half cell of building beyond it read as clear and a push "out" through
  it counted as receding. `tests/test_clearance.py` pins both.
  Against HEAD, `test_check_contract`'s sampled monitor answers changed on
  **12 of 28,800** states, every one a forecast that leaves the map: 7 lose a
  phantom `bld-clearance` violation, 5 keep their rules with a different
  distance in the text. The fixture now also pins the four policies it never
  covered, `follow_car_citylife.yaml` among them.
- **The map.** `demo/build_voxel_map.py` takes any rectangle and cuts several
  bands from one voxel query. The simulator's indexing, read from
  `WorldSimApi::createVoxelGrid`, is `idx = i + nx*(k + nz*j)` with cell
  centres at `centre + (i - n//2)*res`; on a cube the wrong reshape still
  returns the right shape, so `tests/test_voxel_map.py` builds a non-cubic grid
  with the simulator's own formula and checks a block lands where it is.
  `demo/out/citymap_citylife/` covers NED x -140..218, y -60..138 (180 x 100
  cells of 2 m): 6-14 m band 6,021 cells occupied (33.5 %), 15-55 m 6,020,
  2-4 m 5,043, the 0-2 m slice 100 % (the ground, rejected as before); street
  mask 10,642 cells, 415 of them canopy. **Regression:** the old +-80 m cube
  rebuilt through the new code, in the same simulator session, agrees with the
  rectangle on all **5,600 overlapping cells**. The spawn (35, -20) and the
  old phantom point (47.2, 112.7) are free. `run_citylife_follow.ps1` now
  refuses to fly without this map (before it starts a simulator).

On the new map 12.5-13.7 % of each loop's lane points lie within 3 m of
something at 6-14 m: 9 m runs just past most junctions, where kerbside trees
and signal arms overhang the lane. Those are real, and the Shield steering
round them is the Shield working.

## Following the car's trail, not its bearing

The follow flew every horizontal command along the nose, and the nose at the
car, so at a corner it cut across toward a car that had already turned; once
the car was out of sight the estimator predicted it straight on for 3 s and
coast and search carried the aircraft straight past the junction.
`demo/trail.py` (+ `--trail-follow`, on in the car runner) keeps breadcrumbs
of the ESTIMATOR's position after each accepted update - no ground truth - and:

- **track:** forward is toward a carrot 10 m along the trail; speed is the same
  stand-off law, the yaw stays on the car. On a straight street the two
  coincide. A trail direction pointing away from the estimated subject (an old
  leg after a re-acquisition) is refused and the nose wins.
- **coast/search:** fly the trail to the last sighting and look 8 m past it
  along the car's last direction; the search sweep is centred there instead of
  integrated from wherever the nose was. A car last seen MOVING is followed on
  past the sighting at its measured speed; one last seen STOPPED is held off
  at the follow's own stand-off. Never nearer the last sighting than the
  camera's blind spot plus 2 m (6.9 m at 8 m altitude: the 20 deg-down camera
  cannot see the road closer than that).
- **prediction is not evidence:** for up to 3 s after the last accepted box the
  estimator serves a constant-velocity guess; once that guess is half a second
  stale the aircraft does not close on the last MEASURED position past the
  blind spot plus 2 m.
- "stopped" is measured from the displacement of accepted positions over 2 s,
  not the filter's velocity, which keeps the pre-stop speed for seconds.
- the estimator, the trail and the stop test take only FRESH detections (see
  round 4 below); a trail steers nothing until it is 8 m long; a jump of more
  than 30 m restarts it; a retarget clears it.

The flights below found three of these the hard way and four review rounds
found the rest; `tests/test_trail.py` has 18 tests, `test_range_and_lock.py`
covers the blind spot and the stop measure.

## The loop reaches 10.0 Hz: the pacing lost every timer round-up

The 9.3 Hz was blamed on Windows' 15.6 ms timer. Probed in the flight
environment (Python 3.10, proactor loop), that is half of it: a 1 ms
`asyncio.sleep` takes 15.5 ms by default and 2.5 ms under `timeBeginPeriod(1)`,
but a loop that steps an ABSOLUTE deadline reaches 10.0 Hz even at 15.6 ms,
because a late tick is repaid by the next one's shorter sleep. The loop slept
`TICK - work` from each tick's own start, so every round-up was lost for good.
Both are fixed (a stall longer than a tick restarts the deadline rather than
bursting). Measured: **9.99-10.0 Hz** on every flight since, against 8.38 Hz on
the August Demo_day reference and 9.31-9.37 on the third pass. The metric
`timer_resolution_ms` now times the wait the loop actually does
(`asyncio.sleep`), in both arms of `--coarse-timer`.

## Collisions are recorded

The plugin logged every contact to the simulator's own log; the client never
subscribed to `collision_info`. It does now, and `metrics.json` splits the
reports at the mission clock: before t0 (take-off, the start-gate hover), the
mission, and after it (the descent and the landing). Every flight since shows
one contact before t0 and none in the mission; the landings touch a road tile,
and one (`citylife_redcar_trail2`) came down in a hedge - 2,525 contact
reports, all after the mission. `off_map_ticks` counts ticks flown off the
obstacle map (0 on every flight) and is null, not 0, when no map was loaded.

## The ground-contact check rejected the car it was built to protect

`citylife_redcar_trail` (below) lost the car after its second corner with the
presence gate blocking **891 of 2,399 ticks**. The gate's reason was not
logged; reconstructed offline from `detections.jsonl`, 146 of the 176 blocked
inferences pass every check except ground contact, and 34 of those boxes were
ON the red car. The check's inputs are fragile exactly at a corner: a ray a
few degrees below the horizon, a box bottom a few pixels off, and a body pitch
read now for a frame 150-250 ms old. Now:

- the gate's reason is logged per tick and counted by rule
  (`presence_block_reasons`), with pitch and roll per row;
- a box that lands where the estimator predicts the subject (6 deg, 35 % range)
  is not asked to prove it stands on the road - but only while a box that
  PASSED the ground check, with the check actually evaluated, fed the estimate
  in the last 3 s, so a signal beside a stopped car cannot be waved in and
  hold the estimate on itself;
- `ground_check_waiver` in metrics.json counts, per fresh detection, the boxes
  waived and those the check would have REFUSED (`rescued_detections`), and
  says whether it could arm at all (`measured`, `have_depth`).

## The zebra

A car already on a crossing (path index from 3 points before it to 4 after)
now stops where it is for a pedestrian in its own path - 3 m either side, from
1 m behind its centre to 9 m ahead - under the 6 s rule for one standing
still. It took four versions; each of the first three was broken in a way a
review found and an emulation of the Blueprint reproduced:

1. the 6 s release used `StopRun`, which DriveTick zeroes at 10 cm/s, so it
   re-armed after a centimetre and held the car as long as the figure idled;
   and the window closed the moment the car's centre passed the crossing's
   path point, cancelling a stop half-way through braking;
2. its own clock (`StillRun`) was shared by every crossing in the window, so a
   release earned at one zebra carried over to the next, 7-16 path points on;
3. latching the crossing LAST in array order (not path order) re-armed the
   near crossing from the far one's figure, and a crossing behind the car
   could take the latch and hold the car for good.

Now the latch is the nearest crossing AHEAD with a standing figure, a release
exempts only the crossing it was earned at (`StillDone`, forgotten when that
crossing leaves the window), and a figure anywhere in the crossing's box
counts in both windows. Before pushing it to the level, an emulation of the
final logic (UpdateEffSpeed every other tick, DriveTick sub-steps, the real
loop A and B paths) gave: two idle figures 7, 15 and 16 points apart - two
separate 6.1 s stops at 60, 30 and 5 fps; a figure toggling still/moving every
1, 4 or 5 s in the box behind a car waiting at the next line - one 6.1 s stop,
the same as with no such figure; a walker stepping out from the car's side as
it reaches the zebra - an emergency stop on the zebra, released after 3 s.

Measured in Simulate, 4.4 min per version (one sample each; the pedestrians
roam at random, so single counts are noisy):

| | 09-23 | v1 | v2 | v3 | **final** |
|---|---|---|---|---|---|
| PedViol: moving figure on the zebra, car > 50 cm/s (0-2 points) | 5 | 9 | 29 * | 15 | **4** |
| on-the-zebra emergency stops (half-rate ticks) | - | 32 | 37 | 105 | **14** |
| ...of them still above 50 cm/s | - | 2 | 2 | 2 | **0** |
| PedClose: moving figure within 4 m ahead at > 50 cm/s | - | - | 2 | 2 | **0** |
| closest two cars | 650 cm | 621 | 581 | 513 | **650** |
| longest stand-still | 22.3 s | 19.0 | 38.3 | 66.7 | **42.0** |

\* v2 counted over a wider window, so it does not compare. The longest
stand-stills in v2-final are loop B waiting to give way at its two junctions
(polled in Simulate: every car stopped over 8 s had `Conflict` set and its
stop at the box edge); the pedestrian rule is not what holds them, but cars
pausing on zebras near the box make loop A's gaps rarer. PedClose also counts
a figure stepping out inside braking distance (1.3 m from 3.2 m/s), so it is
read beside the emergency stops, not as a pass/fail gate.

## The missions

All on the finished level and the CityLife obstacle map, 240 s after the start
gate (180 s for the pedestrian), `p0_violation_escape_rate` 0.0 and altitude
escape 0.0 s in every one, no collision reported during any mission. Video:
`docs/video/citylife_redcar_trail.mp4`, `citylife_ped_final.mp4`,
`demo_follow_trail2.mp4` (gitignored, like the others).

### The red car

`scripts/run_citylife_follow.ps1 -Object "a red car" -LevelCar Car_10`, now
with `--trail-follow` and the policy's 5 m/s. `_ground` is the third pass's best
flight, without the trail, for comparison. "Estimate on the car" is the share
of ticks the estimator served a position within max(10 m, 30 %) of the car's
true range - whether the thing being followed was the car at all.

| run | start (wait, range) | det Hz | loop Hz | within 30 m | longest < 30 m | on target (null) | in shot | estimate on the car |
|---|---|---|---|---|---|---|---|---|
| `_ground` (09-23, no trail) | 167 s, 12.9 m | 7.5 | 9.31 | 0.314 | 48.8 s / 146 m | 0.532 (0.414) | 0.761 | 0.55 |
| `_trail` (first version) | 176 s, 14.0 m | 6.91 | 9.99 | **0.436** | **103.4 s / 305 m** | **0.779 (0.688)** | **0.879** | **0.74** |
| `_trail2` | 196 s, 13.6 m | 6.88 | 9.99 | 0.282 | 59.0 s / 147 m | 0.447 (0.309) | 0.59 | 0.36 |
| `_trail3` | 1 s, 43.9 m | 4.04 | 9.96 | 0.111 | 26.4 s / 19 m | 0.059 (0.094) | 0.221 | 0.11 |
| `_final1` | 173 s, 13.2 m | 7.12 | 9.99 | 0.091 | 12.8 s / 18 m | 0.361 (0.357) | 0.494 | 0.15 |
| `_final2` | 1 s, 43.9 m | 3.82 | 10.00 | 0.088 | 20.8 s / 35 m | 0.619 (0.57) | 0.7 | 0.33 |
| `_final3` | 2 s, 43.9 m | 3.83 | 10.00 | 0.195 | 35.5 s / 50 m | 0.261 (0.24) | 0.517 | 0.26 |
| `_final4` | 182 s, 13.7 m | 7.18 | 10.00 | 0.255 | 33.4 s / 71 m | 0.231 (0.204) | 0.505 | 0.25 |

What the trail does when the car is the thing being followed: `_trail` held it
within 30 m for **103.4 s over 305 m, through two corners** - the corner every
earlier flight lost it at, and the next one - against 48.8 s / 146 m on a
straight street before. It lost the car after the second corner with 34 of the
car's own boxes refused by the ground-contact check (fixed since). `_final4`
followed it round the first corner too (t = 53-67 s).

What each other flight isolated, and what was changed after it:

- `_trail2`: the car stopped at a zebra, its boxes were lost, and the aircraft
  closed on the estimator's prediction into the camera's blind spot (the
  estimate read 4.9 m/s for a stationary car: the held box was being re-fed).
  Fixed: fresh detections only, a measured stop test, the blind-spot floor.
- `_trail3`: a four-point trail from 40 m gave the lookout a heading 119 deg
  off and the aircraft turned away. Fixed: a trail steers nothing under 8 m.
- `_final1`: a moving car pulled away from 13 to 25 m while the prediction
  window's cap throttled the chase to 1.4 m/s. Fixed: the cap only for a car
  measured stopped. `_final3` and `_final4` flew with that fix.

**What the table says about the rest.** The last column orders the flights
almost exactly as the within-30 m column does. Across the seven trail flights
the estimator served the car on **36 %** of its ticks (2,568 of 7,226); on
`_trail`, 74 %. On the others it was following other red things - boxes that
pass the colour gate, the size check, the ground check and the instance lock -
including false detections 130-180 m away that re-seeded the estimate at its
15 m/s speed clamp after a loss. In `_final4` the aircraft chased one of those
while the real car stood at a zebra 25 m away, and flew directly over it
(0.7 m horizontally, 8 m up; no contact). The red-car mission in this level is
now limited by **whose box it is**, not by what the controller does with it at
a corner. A second confound: the three flights whose gate fired within 2 s of
the simulator starting (the car happened to be passing) ran the detector at
3.8-4.0 Hz against 6.9-7.5 for the rest - the level is still streaming in
its first minutes.

### The pedestrian

`-Object "a person" -Seconds 180 -Tag citylife_ped_final`: within 30 m of a
pedestrian 100 % of the time, mean 10.7 m. Instance-level on target **0.58
against a null of 0.48**, 12 re-associations between figures; the class-level
score is 1.00 against 1.00, meaningless in a crowd. Its box-to-nearest-figure median (12.0 px)
loses to the null (7.4 px) for the same reason - the null is the distance to
the nearest of ~40 people - and `test_track_truth` now names that shape,
allowed only with per-figure truth and a one-figure margin of at least 0.05. The 10 m pedestrian ring
**fired 0 times** while some pedestrian was within 10 m on 839 ticks: it stands
off the TRACKED subject, which the aircraft held at ~12 m; the others are not
its business. The landing came down on a car (40 contact reports, after the
mission).

### Demo_day: vehicle tracking as before

DEMO 1 of `scripts/run_follow_vlm.ps1` (yellow car, turn route), 70 s:

| run | within 30 m | mean separation | on target (null) | Shield interventions | loop Hz |
|---|---|---|---|---|---|
| `demo_follow` (August reference) | 1.0 | 17.3 m | - | 0 | 8.38 |
| `demo_follow_nose` (09-25, no trail) | 1.0 | 16.9 m | 1.0 (0.974) | 0 | 10.0 |
| `demo_follow_trail` (09-25, first trail) | 1.0 | 17.3 m | 1.0 (0.961) | 0 | 10.0 |
| `demo_follow_trail2` (09-29, finished) | 1.0 | 17.3 m | 1.0 (0.961) | 0 | 10.0 |

The trail changes nothing where the nose-follow already worked.

## Three reviews, and what they found

Each round: independent finders per area, each finding checked by three
skeptics (trace it, reproduce it, judge its consequence), majority rules.
Several rounds lost agents to the session limit; findings their verifiers never
reached were checked by hand against the code before being acted on.

What changed because of them, beyond the zebra above:

- **Round 1** (one of three finders finished): the runner's teardown left a
  simulator running on a throw or Ctrl+C; the zebra release defect 1.
- **Round 2** (six finders): the trail pointed BACK at the last sighting once
  past it (3/3), so search oscillated there; a retarget kept the old
  subject's trail; `remaining()` ignored the street before the first
  breadcrumb; nothing stopped the trail direction pointing away from the
  subject; the coast approach could enter the stand-off ring with the
  Shield's subject unset; the zebra window closing mid-stop (high).
- **Round 3**: the first off-map bound read the half cell of a border building
  as clear; `off_map_ticks` read 0 with no map at all; `start_heading_err_deg`
  was measured and then reset to null before any metric read it (every
  metrics.json since it was added); the ground-check waiver could let a red
  signal beside a stopped car capture the estimate; the timer metric measured
  `time.sleep`, which Python 3.11 no longer ties to the timer it was checking;
  landing contacts were bucketed as mission collisions; `--from-voxels` wrote
  to Demo_day's map folder by default.
- **Round 4** (39 agents, all verified): the estimator was re-fed the Grounder's
  HELD box - up to 8 s old - on every inference that found nothing, paired
  with the current pose (this is what laid breadcrumbs 15-190 m from the car
  and put a stopped car's estimate at 4.9 m/s in `_trail2`); the prediction
  window's cap parked the aircraft short of junctions; the waiver's anchor
  lapsed in steady tracking; `verify_drive`'s "not measured" path could never
  run; a failed flight left the previous run's metrics.json as its result.

Rejected by the skeptics (and not acted on): a fixture coverage gap that
predates this work; `--from-voxels` overwriting as operator error (fixed
anyway); PedClose blind at 3-4 points before the zebra; two claims about the
stop test that did not reproduce.

## Still open

- **Target identity in CityLife.** The estimator follows the wrong red thing on
  most ticks of most flights (above). Candidates, none built: an appearance
  embedding checked against the confirmed subject before an estimator update;
  refusing to re-seed a lapsed estimate from a box far from the last sighting;
  a range ceiling on re-acquisition like the start gate's 45 m.
- **Loop B waits up to 42 s to give way** at its two junctions, behind loop A's
  13 cars; cars pausing on zebras near the box make the gaps rarer.
- **The pedestrian ring binds only to the tracked subject**; 839 ticks with a
  different pedestrian inside 10 m went unguarded by design, and the policy
  has no rule for "any pedestrian".
- The start gate can fire while the simulator is still streaming (3.8 Hz
  detector); the runner should wait for the level to settle first.
- PedClose does not see a figure stepping onto the near half of the zebra at 3-4
  path points (review, rejected as low; documented instead).
- `D:` filled up during this pass (a WSL disk image and a Steam update, not
  the project); frames are now written to `C:` through a junction.
