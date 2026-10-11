# Design: the cost-map adapter

**Date:** 2026-10-08. **Cards:** WP2-07 (the cost-map half; the action-set half
was built on 2026-10-06), deviation table item 4 in
`docs/PI-DECISIONS-2026-10-06.md`. **Code:** `guardrail/compiler.py`
(`rasterise_cost_map`, `CostMap`, `load_cost_map`, `cost_map_agreement`,
`cost_map_report`; CLI `python -m guardrail.compiler costmap` and
`costmap-report`). **Tests:** `tests/test_csp.py` (the cost-map block at the
end).

## Why it exists now

The Prefix Compiler page lists a "Cost-map adapter (path planners only)"
among its per-consumer adapters and its Outputs table, and gives the CSP a
`cost_map_ref` field for it. The page's own open question says it is only
useful once a path planner joins the system, and on 6 Oct the recommendation
to the PI was to accept it as out of scope (deviation 4). The PI's answer was
to follow the grant wherever that is technically possible. Producing the
artefact needs no planner, so it is built; nothing in this repository reads
it yet, and that is stated wherever it is mentioned.

## What it is

`rasterise_cost_map(policy, now=..., zone_age_s=..., altitude_m=...,
resolution_m=5, bounds=..., subject=..., obstacle_map=...)` returns a
`CostMap`: a 2-D float32 grid, one boolean mask per layer, and a JSON-ready
header. `CostMap.save(stem)` writes `stem.npz` (the grid and the masks) and
`stem.json` (the header). `load_cost_map(stem.json, policy, ref=...)` reads
it back and refuses a grid, a mask or a header that is not the one written, a
map of another policy or generation, and - given a CSP's `cost_map_ref` - a
map that is not the one that CSP names (two maps of one generation differ by
clock, height, subject or zone age). `ConstraintCompiler.compile_csp(...,
cost_map=cm)` puts `cm.ref` into the CSP's `cost_map_ref`; without the
argument the field stays `None`, as the grant's default, so no CSP compiled
before this change is different after it. `compile_csp` accepts only the
map of the CSP it compiles: the same policy and generation, the same clock
(`now`, or no clock on both), and drawn at the mission's cruise height
(`altitude_m = mission.cruise_alt_m`; a map drawn at no particular height is
refused, see below). The same policy and generation alone is not enough:
in review a CSP issued Mon 08:15 for a 15 m cruise accepted a map drawn at
Mon 12:30, in which a fence the Shield enforces at 08:15 was missing and a
fence a switch holds off was drawn, and a map drawn at 200 m.
`write_csp(path, ..., cost_map=cm)` also exports the map beside the CSP as
`costmap_<csp stem>.npz` + `.json`, so the reference travels with a file
that resolves; the prefix keeps it out of the `csp_g*.json` pattern the
replay bundle reads. A map of another policy, clock or height is refused
before anything is written. What a CSP cannot check - the subject's
position, a moving zone's age, the obstacle map - is in the map's header
and is the caller's to match.

## Conventions

The grant's example (Prefix Compiler page) gives the keys and four values;
the rest is this project's, chosen to match what already exists here.

| Item | Value | Source |
|---|---|---|
| resolution | 5 m by default (`resolution_m`) | grant example |
| origin | `cost_map.origin: {lat, lon}` of cell (0, 0)'s centre; `null` for a policy written in metres with no `origin` | grant example (key); the WGS84 value is computed with the policy's own projection |
| layers | `cost_map.layers: [{name, source, cost, cells}]` | grant example (`cells` added) |
| `nfz_hard` | `inf` (written `"inf"` in JSON, `np.inf` in the grid) | grant example |
| `corridor_pref` | 0, one layer per keep-in tube | grant example |
| `corridor_outside` | 50, every cell not inside ALL tubes in force | grant example; "all" because the Shield checks each corridor on its own |
| soft layers | 50 (`nfz_soft`, `standoff_soft`, `clearance_soft`, `altitude_soft`) | this project's; a soft rule is discouraged, never impassable |
| `standoff_hard`, `clearance_hard`, `altitude_hard` | `inf` | this project's names for the remaining rule types |
| frame | local metres, x North, y East of the policy frame origin | `guardrail/models.py` `XY` |
| indexing | `cost[i, j]` is the cell centred on `(ox + i*res, oy + j*res)`; `origin_xy_m: [ox, oy]` | the Shield's obstacle-map convention (`{"occ", "res", "ox", "oy"}`) |
| combination | maximum over the layers a cell is in; 0 in none | this project's |
| keep-out cell rule | a cell is in a keep-out layer when its square MEETS the zone, margin ring included | conservative side |
| keep-in cell rule | a cell is inside a tube only when its square lies WHOLLY in it | conservative side |
| header format | JSON (a YAML 1.2 document too: PyYAML reads each of the 24 headers of the shipped policies to the same values, and `tests/test_csp.py` checks it) | the grant's Outputs table says "emits YAML"; JSON has no ambiguity about `inf` and is what the rest of this repository writes |
| identity | `ref = "costmap:<policy_id>@v<version>/g<generation>#sha256:<digest of the header>"`; the header carries the SHA-256 of the grid and of every mask | as `geometry_ref`, plus a content digest |

## One instant, read from the Shield

A cost map is the rules in force at one instant. Which rules are in force is
not decided here: a Shield is built over the same policy and clock and its
`rule_status()["in_force"]` is read (both sides of the instant, the Shield's
own rule), so a time window, a `time_window_switch` and a `corridor_swap` are
applied exactly as the Shield applies them. Keep-out zones are the IR's
compiled margin rings (`guardrail/ir.py`, the polygons the Shield tests); a
moving `dynamic_nfz` is drawn at `ring_at(age)` through the same
`ir.compile_fence` the Shield materialises it with, and the age used is in
the header. A stand-off disc is drawn as a polygon that contains the circle.
Clearance is measured to each blocked map cell's whole square, which is never
farther than the Shield's measure (to the cell's centre, through a chamfer
field), by up to half a map cell.

What cannot be drawn is listed, not dropped: `not_drawn` names each kinematic
cap, each switch, an altitude envelope the chosen height respects, and a
stand-off or clearance rule whose input (subject position, obstacle map) was
not given - the Shield holds those inert too. `not_in_force` and
`out_of_band` name the rest. With a height, a zone whose band misses it is
left out, a tube whose band misses it has no inside (every cell is
`corridor_outside`), and a height outside a hard altitude envelope makes the
whole grid `inf`. With `altitude_m=None` (the 2-D reading of a planner that
does not know its height) every keep-out zone is drawn whatever its band,
which is conservative, but every tube is read laterally, ignoring its band,
which is NOT: at a height outside a corridor's band the Shield forbids the
whole corridor, while this map prices its interior 0. No altitude envelope
is priced either. That is why `compile_csp` refuses a map drawn at `None`.

## The score, and its two nulls

`cost_map_agreement(cm, policy)` draws points uniformly over the grid
(seeded) and asks the Shield, through `rule_status`, which rules standing at
each point breaks. A point the Shield forbids needs `inf` (a hard rule) or
at least 50 (soft rules, or outside a corridor):

- **missed**: the Shield forbids the point and the cell is cheaper. A sound
  map has 0; a planner trusting it would route through a place the Shield
  then repairs away from.
- **hard-forbidden but priced finite** (`hard_rule_priced_finite`): a hard
  rule forbids the point, a hard corridor included, and the cell's cost is
  finite. `missed` asks only 50 of a point outside a corridor, which is the
  grant's `corridor_outside` price, and that holds for a hard corridor too;
  so `missed` = 0 does not mean the map never prices a place the Shield
  refuses. This count says how many it does. It is reported beside `missed`
  everywhere, never folded into it (added after review, 2026-10-08).
- **over-cost**: the Shield allows the point and the cell costs more than 0,
  the price of the conservative cell rule.

Both nulls are scored the same way: an all-free grid (misses every forbidden
point) and an all-`inf` grid (misses none, over-costs every allowed point).
The tests also free one zone's cells in a real map and require the scorer to
report missed points, and score a moving zone's map at the wrong age and
require the same.

Measured on 2026-10-08 (`python -m guardrail.compiler costmap-report
--points 1000`, 5 m cells, no clock, zones at their declared position, no
subject or obstacle map, no height - so corridors are read laterally, see
above; code 10451bc plus this change, not yet committed). Every forbidden
point in this run is forbidden by a hard rule:

| Grid | Points (forbidden by the Shield) | Missed | Hard-forbidden, priced finite | Over-cost |
|---|---|---|---|---|
| this map | 24 000 over 24 policies (4 181) | 0 (0 %) | 602 (14 %), all outside the hard corridor of `corridor_survey`, priced 50 | 2 116 of 19 819 allowed (11 %) |
| null: all free | same | 4 181 (100 %) | 4 181 (100 %) | 0 |
| null: all `inf` | same | 0 | 0 | 19 819 (100 %) |

So the headline is: **0 missed under the grant's `corridor_outside` = 50;
602 points outside a hard corridor are priced 50, not `inf`.** Scored with
hard corridors requiring `inf`, the same 602 points would read as missed
(the reviewer's rerun: 602 of 4 181, 14 %).

At 2 m cells (`--resolution 2`) the same run reads 0 missed of 7 327
forbidden, 479 hard-forbidden points priced finite (7 %, all in
`corridor_survey`) and 11 % over-cost; the grids differ in extent (two cells of
padding each), so the two over-cost shares are not a resolution study. Five
policies are listed and not scored: `follow_car`, `follow_car_citylife`,
`follow_pedestrian`, `orbit_building`, `sitl_pedestrian` have nothing with a
footprint without a subject position or an obstacle map (stand-off and
clearance rules, envelopes), and the report says so instead of scoring an
empty grid. Stand-off and clearance layers are scored in
`tests/test_csp.py` with a subject and a 30 x 30 obstacle map (0 missed).
The shipped-policy report itself is a test there
(`test_the_shipped_policies_cost_map_report_misses_nothing`), and a second
test samples only a zone's margin ring: a map drawing zones without their
ring passed every other test in review, and fails both of these.

## What is not done

- No planner reads the map. The adapter is an output, as the grant's table
  lists it; whether a planner backend joins the system is the grant's own
  open question.
- One instant per map. A planner over a horizon with moving zones or
  changing windows needs a map per step (call it again with `now` and
  `zone_age_s` advanced); no time-indexed stack is produced.
- The agreement score cannot model per-zone ages that differ (the Shield
  moves every authored zone on one clock); it refuses such a map rather than
  scoring it.
- `corridor_outside` costs 50 for a HARD corridor too, following the grant's
  example (an `inf` outside would leave a planner that starts outside the
  corridor, on the take-off pad, with no finite path in). A planner may
  therefore cross outside a hard corridor at a price; the Shield still
  enforces it. The score counts those points (`hard_rule_priced_finite`:
  602 on the shipped set at 5 m). A stricter planner can read the
  `corridor_pref` masks and the rule's `constraint_type` from the policy.
- The `altitude_m=None` reading is permissive for a corridor's altitude band
  (above); the shipped-policy report is computed at `None`, so its numbers
  are lateral for corridors. A CSP never names such a map.
- 2-D only, as the grant's example. Height is a parameter of the map, not an
  axis of it.
- Nothing here was flown; no simulator, SITL or GPU model was started.
