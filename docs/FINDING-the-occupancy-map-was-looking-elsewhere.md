# The occupancy map was sampling a band the aircraft never flies in

**Date:** 2026-08-25
**Status:** cause found and fixed; the corrected map is NOT yet the default, and
this note says why.

## The incident

A 9 m flight struck street furniture at **(48.3, −0.9)** while the Shield's
occupancy map reported **7.4 m of clearance** there, against a 5 m requirement.
The Shield was not wrong about the map; the map was wrong about the city.

## Cause

`demo/build_voxel_map.py` collapses the simulator's ground-truth voxel grid to a
2-D grid over an altitude band, and that band defaulted to **15–55 m AGL**. These
demos cruise at **9 m**. Everything shorter than 15 m — traffic lights, signs,
poles, tree canopies — was therefore structurally invisible.

"The occupancy map contains buildings only", repeated in several documents
including the midterm report, was a description of the sampling band rather than
of the world. Nothing was missing from the simulator's geometry.

Rebuilt over **6–14 m**, matching the altitude envelope in the policy that
consumes it:

| | 15–55 m band | 6–14 m band |
|---|---|---|
| Occupied fraction | 0.346 | 0.315 |
| Clearance at (48.3, −0.9) | **7.4 m** | **1.8 m** |
| Cells newly blocked | — | 564 |
| Cells freed | — | 461 |

The struck obstacle survives even an 8–14 m band, so it is tall: a genuine
hazard at cruise altitude, not kerb clutter. The 461 freed cells are the mirror
image — building tops that exist at 15–55 m and nothing at all at 9 m.

**Under the corrected map the Shield would have prevented the collision.**

## Why it is not the default yet

Installing the corrected map alone made the demos worse, and the first flight
said so immediately: detector hit rate fell **1.000 → 0.48**, the target was held
on 68 % of ticks instead of 100 %, and mean separation went from 16 m to 56 m.
The aircraft was not avoiding obstacles, it was being pushed off the road.

With street furniture present a 5 m clearance ring is not satisfiable on these
streets — at y = 20 the widest free point across the road carries **5.2 m**, and
six of twenty-five route waypoints fell below the minimum. Lowering
`min_clearance_m` to 3.0 fixed it: hit rate back to 1.000, target held 100 % of
ticks, P0 escape rate 0.0, and **52 Shield interventions where the same demo
previously recorded 0** — because there had been nothing in the map to avoid.

That pairing works. What blocks adopting it is a third consequence, and it is
conceptual rather than numeric.

### The map is not a road map

Two tests fail under the corrected map, and both fail for the same reason.

- `test_the_turn_route_stays_on_mapped_road` flags the car's route at
  **(38.0, 22.1)**, where the corrected map is fully occupied. That cell is a
  canopy **over** the road. It blocks a drone at 9 m and does not inconvenience a
  car driving underneath it. The test treats an aerial obstacle map as a
  driveability map, which was harmless while the map held only buildings — roads
  were always free by construction — and is wrong now.

- `test_a_detour_must_stay_on_the_road_not_merely_outside_the_fence` fails from
  the opposite direction: the 461 freed cells open detours over low structures
  that the old map forbade, so "on the road" can no longer be inferred from
  free space either.

Both need a **street mask** — a separate layer saying where a route may run —
rather than inferring roadness from the absence of obstacles. That is a design
change, not a threshold.

### And the gap policy's gap has moved

`policies/follow_car_gap.yaml` describes a corridor at x 43–50. Under the
corrected map its eastern half is solid: **0.0 m at x 48–50, y 0**. The western
end (x 43–45) still carries 3.7–7.8 m, so the geometry is not unflyable — the
gap moved and the policy still describes the old one.

Worth recording: lowering `stand_off_m` does **not** recover it. Tried at 3.0,
2.0, 1.5 and 1.0 m, all report no gap. This is not a threshold that can be tuned
until the demo passes; it is an obstacle.

## What is in the repository

| File | Band | Role |
|---|---|---|
| `occ_day.npz` | 15–55 m | Current default. Every measured result to date used it. |
| `occ_day_flightband_6to14.npz` | 6–14 m | The corrected map. Ready, evidenced, not yet wired in. |
| `occ_day_highband_15to55.npz` | 15–55 m | Explicit copy of the default, so the swap is reversible. |

## To adopt it

1. Add a street mask and rewrite the two "stays on the road" tests against it
   instead of against the obstacle map.
2. Move `follow_car_gap.yaml`'s corridor west to where the gap actually is.
3. Set `min_clearance_m: 3.0` in both follow policies. This reads as a weaker
   rule and is the opposite — the old 5 m was a larger number measured against a
   map with nothing short in it, and 3 m around a pole that is really there
   beats 5 m around a pole that is not.
4. Copy `occ_day_flightband_6to14.npz` over `occ_day.npz` and re-fly all three
   demos, since every number in the midterm report is conditioned on the map.

Doing (4) without (1)–(3) is what produced the 0.48 hit rate.
