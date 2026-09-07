# The obstacle map was true — for an altitude the policy does not fly at

**Date:** 2026-09-07
**Found by:** trying to close what the checklist called a tidiness item.
**Status:** measured, with a checked selector and tests. The map itself cannot be
rebuilt without the simulator running, so the gap is now **reported** rather than
closed.

## What the checklist said

> `follow_pedestrian.yaml` permits descent to **4 m** but loads the 6–14 m band
> map, while `ground_2to4.npz` exists for that altitude and nothing chooses it.

Written as housekeeping: the right map exists, wire it up. Both halves turn out
to be wrong — and the thing underneath them is worse than either.

## What is actually true

The obstacle map is **2-D**. `demo/build_voxel_map.py` asks the simulator for a
voxel cube and collapses everything inside one altitude band into a plane, so a
map is only meaningful for the band it was flattened over. Four exist:

| map | band | occupied cells |
|---|---|---|
| `ground_0to2` | 0–2 m | 6400 — **every cell** |
| `ground_2to4` | 2–4 m | 1391 |
| `occ_day_flightband_6to14` (= `occ_day.npz`) | 6–14 m | 2015 |
| `occ_day_highband_15to55` | 15–55 m | 2212 |

**`ground_2to4` would not have helped.** It ends at exactly 4 m, which is where
the policy's band *begins*. It never applies to a 4–10 m flight at all.

**Nothing maps 4–6 m.** The bands are not contiguous, and the hole sits inside
the band this policy permits. That was not in the checklist, in the report, or in
anyone's head — including mine, and I had written the item.

**Neither map contains the other.** 300 cells are occupied at 2–4 m and clear at
6–14 m; the building at grid cell (64, 40) — the one in the documented 9 m
collision — is in the cruise map and open ground at 2–4 m. So "pick the right
map" is not a well-formed instruction for a policy that spans bands. The only
sound reading of a set of 2-D projections is their **union**: a wall present at
3 m and absent at 8 m is still a wall, somewhere inside a band the aircraft is
allowed to occupy.

**The ground plane is not an obstacle map.** `ground_0to2` has all 6400 cells
occupied — it is the ground seen from above. Union it in and every cell is
blocked, and a Shield that vetoes everything looks exactly like a Shield working
very hard.

## What was built

`demo/occ_bands.py` — `select_for_band(map_dir, alt_min, alt_max)`:

- unions every band map overlapping the policy's altitude envelope;
- refuses to union maps on different grids, rather than silently shifting
  obstacles by an origin difference;
- rejects a band that is more than 90 % occupied, with the reason;
- and **names the uncovered slices**, because the failure this replaces is a
  plausible grid returned for the wrong altitude with nothing saying so.

On the pedestrian policy it now prints:

```
[occ] occ_day_flightband_6to14 (6-14 m), 2015 cells
[occ] *** 4-6 m of the permitted band has NO map. Obstacles there are
      invisible to ObstacleClearance.
```

`strict=True` raises instead, naming the slice. It is off by default: the honest
response to a partial map is to fly with it and say so, not to refuse to fly.

Pinned by `tests/test_occ_bands.py` — fourteen tests, including the 300-cell
asymmetry and the (64, 40) cell, asserted against the map files themselves so
this document cannot drift from them.

## A framing this corrects

`docs/MEETING-PACK-Sept2026.md` (A2 item 2, and Q&A D8) says the parked cars and
pedestrians added on 31 August are missing from `occ_day.npz` and calls it an
overdue debt closed by a map rebuild. The fact is right and the framing is not.

They are spawned at run time, per seed, *after* any map was built, so they are in
no band map and a rebuild would only ever match one seed. More to the point, at
the 8 m cruise this policy actually flies, a 1.5 m parked car is not a collision
candidate — its absence from a 6–14 m map is correct behaviour, not a gap.

The real exposure is the one above: the policy permits descent to 4 m, and
between 4 m and 6 m there is no map of anything at all.

## What this does not fix

The map cannot be rebuilt from here — `build_voxel_map.py` needs Project AirSim
running with the scene loaded. Two rebuilds would close it properly: one over
**4–6 m**, and one over **2–14 m** as a single band for policies that span it.
Until then the aircraft flies the 6–14 m map and is told, on every start-up,
exactly which two metres of its permitted envelope are unmapped.

The union is also conservative in a way worth stating: merging 2–4 into a 4–10 m
flight would repel the aircraft from low walls it could safely fly over at 8 m.
That is the price of a 2-D map, it is paid in the safe direction, and the
selector only pays it for bands the policy actually permits.
