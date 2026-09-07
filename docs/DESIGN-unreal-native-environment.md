# Direction: build the scene in Unreal directly, instead of spawning into it

**Raised:** 2026-09-02, by Nathan
**Status:** recorded; the scripted path is now **proven** (2026-09-04), the
placement work itself still deferred

> **Update, 2026-09-04 — the central objection below is weaker than it was.**
> This note argues the trade is visual fidelity *against* reproducibility, on the
> assumption that authoring means hand-authoring in the editor. That was never
> tested. It has now been: a Python commandlet duplicated the map, placed an
> actor, saved, reloaded from disk and found it again at exactly the requested
> transform — headless, in 37 seconds, with the original map byte-identical
> afterwards. See `FINDING-the-level-can-be-scripted.md`.
>
> So the environment can be authored as **code**, with the level as build output
> rather than source. Read the cost section below with that in mind: what remains
> genuinely un-scriptable is judgement — whether a placement looks right, whether
> the lighting reads — not the placement itself.
>
> Two other corrections to this note. The engine is **UE 5.8**, not 5.7: the
> project was migrated on 2026-09-03 and eleven launcher scripts were left
> pointing at 5.7, which would have rebuilt its modules against the wrong engine.
> They now resolve the version from `Blocks.uproject` (`tools/ue_engine.py`). And
> Unreal MCP was already wired up before any of this — it is live and speaking
> the protocol.
**Prompted by:** the scene looks noticeably worse than Unreal renders natively,
and the added vehicles and pedestrians look "pasted on" rather than placed.

## The observation is correct, and the cause is structural

Nothing in the current pipeline builds a scene. It **spawns objects into one at
run time** through Project AirSim's `spawn_object_from_file`, from free GLB
assets, positioned by a seeded random placer (`demo/parked_cars.py`,
`demo/pedestrians.py`).

That approach was chosen for reasons that were right at the time and are still
right for what it does:

- placement is **reproducible from a seed**, so a KPI run can be repeated;
- the scene costs nothing per tick once spawned, which is what let 29 objects sit
  in a scene whose control loop only just clears its detector gate;
- no large binary assets enter the repository.

But it also explains every visual complaint. Run-time spawned meshes get no
baked lighting, no lightmaps, no reflection capture, no shadow proxies, and no
material tuning against the scene's own lighting. They are lit by whatever the
level's dynamic lighting happens to do to them. Nothing is placed by eye against
the geometry, so a car can sit slightly off the road camber and read as floating.
Free assets also arrive at inconsistent scale, polycount and texel density — the
pedestrian mesh needed a `4.81` unit-height correction and a full re-export
before it was even the right size.

Native Unreal authoring fixes all of that by construction: assets placed in the
editor, lighting built, materials adjusted, everything saved into the level.

## What it would buy

- A scene that looks like the Japan city map does when opened directly.
- Better assets, placed deliberately rather than sampled onto a street mask.
- A materially more convincing video for the 18 September sharing meeting and for
  the final deliverable.

## What it would cost — the part worth deciding with open eyes

**The scene stops being reproducible from code.** Right now a KPI run can be
re-created from a seed and a policy hash. A hand-authored level is a binary
artefact: it cannot go in the repository (the no-large-assets rule), it cannot be
diffed, and `is_kpi_grade()` has no way to assert that the world was the same as
last time. That is not fatal — the canonical KPI rail has **no renderer at all**,
so the contractual numbers do not depend on the scene — but the AirSim evidence
would become harder to reproduce, not easier.

**It is Unreal work, not Python work.** Level editing, lighting build, asset
import. Different skills and a different iteration loop from everything else here,
and the iteration is slow: a lighting build is minutes, not seconds.

**Asset licences become a real question.** Better models usually means paid or
licence-restricted ones. Anything used in a deliverable for an ITRI subcontract
needs its licence checked first — the same question already open on YOLO-World's
AGPL.

**It does not move any acceptance KPI.** Not one of the five improves. The
contractual gap is perception on the KPI-grade rail, and prettier geometry does
nothing for it.

## Recommendation

Worth doing, **after** 18 September, and framed honestly as what it is:
presentation quality for the funder, not a technical result.

A sensible split that keeps what the current approach is good at:

1. Author the **static** city in Unreal — buildings, roads, street furniture,
   parked vehicles, lighting — and save it as the level. These never move, so
   they lose nothing by being baked in, and they are most of what looks wrong.
2. Keep **spawning the few objects the mission depends on** — the tracked
   subject, any pedestrian used as a stand-off subject — from code with a seed.
   Those are the ones a KPI run has to be able to reproduce.
3. **Rebuild the occupancy map from the new level.** Any level edit invalidates
   the map, so this is not optional housekeeping — it is the step that keeps
   `ObstacleClearance` describing the world the aircraft is actually in.

Point 3 is what keeps a cosmetic task from becoming a safety regression.

> **Correction, 2026-09-07.** An earlier version of point 3 said the rebuild
> would "close the real map gap: tall street furniture at cruise height". There
> is no such gap. `occ_day.npz` was rebuilt over the 6–14 m band on 25 August and
> the cell holding the 9 m collision at (48.3, −0.9) is occupied in it. The
> reason to rebuild after editing a level is that the old map no longer matches
> the level — not that something is missing from it today.

## Not to be confused with

The decorative scenery being absent from the cruise-band occupancy map is **not**
a defect: pedestrians are 1.75 m and vehicles 2–3 m, and the map covers 6–14 m.
An earlier note of mine got that wrong. See
`meeting notes/2026-09-02-pedestrians-id-lock-and-detector-comparison.md`.
