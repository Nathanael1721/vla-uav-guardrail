# DESIGN - the Policy DSL in the grant's own form

2026-10-06, revised 2026-10-07 after review. Audit cards WP1-01, WP1-02,
WP1-05, WP1-06 (model part), WP1-07, WP1-09 / WP1-15 (types), WP1-13, WP1-14,
WP1-16, WP1-17, X-11.
Code: `guardrail/models.py`, `guardrail/projection.py`, `guardrail/bundle.py`,
`tools/geo_to_policy.py`, `tools/wp1_roundtrip_kpi.py`.
Tests: `tests/test_policy_dsl_grant_form.py`, `tests/test_geo_to_policy.py`,
`tests/test_policy_hash.py`, `tests/test_bundle.py`, `tests/test_wp1_roundtrip_kpi.py`.

## The problem

On 2026-10-05 this repository's loader refused the grant's own example. The
reference repository's `bundles/itri-icl-2026-demo.yaml` (the policy printed
on the grant's Policy DSL page) failed on `issued_at`, and with that line
removed on four more fields: `project_fix`, `geometry.vertices`,
`altitude_min_m`, `altitude_max_m`. The reference's bundle of the same policy
failed the same four ways. Meanwhile 11 of 25 deliberately broken policies in
`policies/negative/` loaded without a word, among them a misspelt ceiling that
fell back to 1000 m, a bow-tie fence, an infinite speed cap and `version:
banana`.

The PI's ruling of 2026-10-06: follow the grant; conform rather than ask for
waivers. Where the grant is silent, the reference implementation
(`kuanting-vla-uav-guardrail/`) is the detailed specification.

## Decisions

### 1. Two surface forms, one IR

A rule may be written in the grant's form or in this project's earlier form.
Both load into the same model; the IR keeps this project's field names, so no
stored hash moved.

| Grant form | This project's form (still loads) |
|---|---|
| `geometry: {vertices, altitude_floor_m, altitude_ceiling_m, altitude_ref}` | the same keys at the rule's top level |
| `geometry: {centerline, width_m, ...}` (corridor, corridor_swap) | top level |
| `geometry: {center, radius_m, ...}` (circle_fence; `centre` accepted) | top level |
| `altitude_min_m` / `altitude_max_m` (worked example), `alt_min` / `alt_max` (taxonomy table) | `alt_min_m` / `alt_max_m` |
| `speed_max`, `climb_rate_max`, `turn_rate_max` | `speed_max_mps`, `climb_rate_max_mps`, `yaw_rate_max_dps` |
| `violation_action`: the six grant actions | `repair` (kept, an alias of `project_fix`) |
| `scope`, `layer`, `altitude_ref`, `issued_at`, `layers_merged` | absent |

Writing one field twice (under both names, or inside and beside `geometry`) is
refused, not resolved. `Policy.to_grant_form()` produces the grant's form
(`python -m guardrail.bundle export`).

A `geometry:` block is filled with the reference's defaults for what it omits -
altitude band 0-200 m and no margin ring - because that is what the same block
means to the reference. The flat form keeps this project's defaults (0-1000 m,
1 m margin), so no existing policy changed meaning.

### 2. WGS84 is canonical; metres are the legacy form

The grant: "WGS84 latitude / longitude is canonical ... the DSL itself never
carries projected coordinates." A point authored in lat/lon now keeps lat/lon
in the IR and in the hash; x/y are derived at load for the Shield
(`projection.project_raw`, called by `Policy` itself). A metre point is stored
as x/y exactly as before.

This is hash scheme `sha256-canonical-v3`. It differs from v2 only for a policy
with WGS84 points. Of the 29 policies in `policies/`, 28 hash exactly as
before; `wgs84_taipei.yaml` moved from `sha256:ed22f684...` (v2, projected
metres) to `sha256:7d6e1955...` (v3, lat/lon). The old hash is still its hash
under the named prior form (`Policy.prior_hashes()`, `hash_form()`), the lock
entry keeps it and records the v3 form beside it (`hashes`), and no stored run
manifest used it. Every stored manifest that existed when this change was
made still resolves: 81 (the 76 runs counted on 2026-10-05 plus 5 added on
2026-10-06), of which 34 resolved under the formula used before the identity
release. Runs keep arriving from other units; at the last measurement
(2026-10-07 10:25) all 86 runs in `demo/out/*/` resolved, and 2 more that
another unit set aside in `demo/out/_hot_apply_not_in_lock/` do not - see
Measurements.

Lock entries written from 2026-10-07 on record their own `hash_scheme`; older
entries carry none and are under the lock's top-level `sha256-canonical-v2`
(the lock's `_doc` says so). The IR schema pins now list every field of every
model: `lock --update` adds a new field whose omission means "absent" (the
one change that moves no hash) and never alters an existing field's pin. The
2.1.0 fields were added to the pre-existing models' pins that way
(`ir_schema.additive_pins`).

**The frame.** The grant's form has no origin. The reference derives its local
plane from the mean of the first polygon_fence's vertices
(`policy_dsl/ir.py`, `_choose_origin`); `projection.derive_frame_origin` does
the same with the same float arithmetic (checked to the last bit against the
reference's printed origin). Where the reference falls back to (0, 0) - no
polygon - this code uses the first geographic rule instead. An explicit
`origin:` still wins. The frame matters at the Shield's interface, because
`State` is metres from the frame origin, and the rail that flies geographic
policies (ArduPilot SITL) puts the vehicle's start - the autopilot's home - at
that origin. A derived origin is the centroid of the first fence, not the
take-off point, so flying a grant-form policy as written would place every
zone wrong relative to the vehicle with nothing failing. The flight loaders therefore refuse a geographic
policy with no stated origin (`Policy.frame_problem`); the DSL loads it as a
declaration, and stating `origin: {lat, lon}` = the take-off position makes it
flyable (`tools/geo_to_policy.py convert --origin`). The reference solves the
same problem with a marker polygon at the spawn point
(`bundles/projectairsim-demo.yaml`).

**What the gate guarantees is a stated frame, not a correct one.** It cannot
know where the vehicle is: an `origin` written inside a keep-out zone passes
it. The first version of this change "proved" the gate with a take-off point
that was the school yard's own centroid, so the policy it called flyable
started the vehicle inside `nfz-school-yard` - the failure the gate was cited
for (2026-10-07 review). Now:

- `Policy.start_conflicts()` names every hard keep-out zone whose footprint
  (polygon plus margin ring, any height, any schedule) holds the frame origin,
  and `load_for_flight` records each in the run's `runtime_notes`;
- `tools/geo_to_policy.py convert --origin` refuses a take-off point inside a
  zone it creates;
- the tests use a take-off point ~100 m south of the yard and assert the start
  is safe.

It is recorded, not refused. The frame origin is the start on the ArduPilot
SITL rail (metres about home; its MAVLink adapter will not upload the zones of
a policy whose `origin` is not at home), but not on every rail: the CityLife
scenarios spawn at (35, -20) in their frame (`demo/gen_random_scenario.py`).
8 of the 29 policies in `policies/` have a hard zone over (0, 0): six
generated CityLife scenarios (`hard_11`, `hard_21`, `hp_8`, `hp_14`, `hp_30`,
`hp_42`, whose generator keeps zones clear of the real spawn), `poly_test` (a
fence drawn from the origin) and `wgs84_taipei.yaml` (its origin is the yard's
centroid; it is used by `experiments/scenarios.yaml`). A refusal would turn
all eight away, most of them for a start they do not have; the note says "a
vehicle that starts at the frame origin" for that reason.

Mixing frames inside one rule is refused. Mixing across rules is allowed: a
geographic policy with a metre fence hot-applied mid-flight is exactly that,
and its bundle must load again.

### 3. issued_at is part of the document when written

`docs/DESIGN-policy-identity.md` section 5 kept `issued_at` out of the policy
(refused in the file, recorded in the bundle manifest) so re-issuing never
moved a hash. The grant's worked example and the reference document put it in
the document, and the reference hashes it. Conforming means: a policy that
writes `issued_at` keeps it and hashes it (coerced to text exactly as the
reference does); the bundle manifest carries it; an explicit `--issued-at` for
another instant is refused. A policy that does not write it behaves as before.

### 4. Strict validation, in two places

Model level, always: `extra="forbid"` and no inf/NaN on every IR model;
`policy_id` non-empty; semver `version`; `generation >= 0`; lat/lon ranges and
pairing; a simple polygon (no crossing edges, non-zero area). The polygon
check is pure Python, so models.py does not import shapely. A vertex repeated
in a row is read once (shapely accepts it, and GIS exports produce it; the
first version refused it as "a zero-length edge"). Edges are compared after a
sweep along x, so a 2000-vertex ring takes milliseconds rather than the
3.5 s of the first, pairwise version. What it is checked against is limited
and stated: shapely's `is_valid` plus non-zero area on 300 seeded random
rings, 150 of them again with one vertex doubled, and every fence in
`policies/`, with no disagreement. That is not a proof of agreement on every
input.

Lint level (`Policy.lint()`), run by every DSL entry point - `load_policy`, the
layer merge, `check_bundle`: at least one rule, unique rule ids, no negative
margin, no conflicting hard altitude limits, switch and swap targets that
exist (a swap must target a corridor, nothing may target itself or another
event), and `layers_merged` consistent with the rules. A conflict is any two
of the hard envelopes and hard corridor bands with no height in common while
their schedules can overlap (weekly intervals, wrap-around included). Two
corridors count: the Shield checks every active corridor on its own, so the
vehicle must be inside all of them at once (until 2026-10-07 corridor pairs
were skipped). Two corridors whose tubes never meet horizontally are just as
unsatisfiable; that needs geometry the lint does not have and is not checked.
Rules with different altitude references are not compared. These are lint
rather than model checks because existing callers construct policies outside
them on purpose (a test fence with a negative margin, an empty policy a
hot-apply fills).

Duplicate YAML keys are refused by the loader (PyYAML keeps the last silently).

### 5. All nine rule types; the runtime refuses what it cannot enforce

Declarable now: `circle_fence`, `distance_envelope`, `dynamic_nfz`,
`time_window_switch`, `corridor_swap`, beside the four that existed.

`circle_fence` is enforced: it is a `PolygonFence` subclass whose vertices are
the 32-gon circumscribing the circle (at most 0.48 % of the radius beyond it),
so the Shield, its STRtree and the CSP handle it unchanged. The IR stores centre
and radius; the polygon is never stored.

The other four are declarable only for the flight loaders (`RUNTIME_TYPES`).
When this change was written (2026-10-06) the Shield could not move a zone,
toggle a rule or swap a corridor, and it has no map of people or roads. A rule
the Shield silently ignored would report a clean flight while enforcing
nothing, so every flight loader refuses a policy holding one, naming each rule
and why (`Policy.unenforced_rules`): `load_policy`, `load_layered` and
`bundle.load_bundle` (each `runtime=True` by default) and
`bundle.load_for_flight`. `runtime=False` loads the declaration for bundling,
schema and export. `load_bundle` gained the gate on 2026-10-07: before, a
bundle with a dynamic_nfz loaded through it and the Shield of that morning
reported the zone's centre as safe, and `guardrail/scenario_spec.py` loads
its `bundle_path` scenarios through it.

Later on 2026-10-07 the Shield itself (`guardrail/shield.py`,
`ENFORCED_TYPES`; another unit's change) began enforcing dynamic_nfz (motion
included), time_window_switch and corridor_swap, and refusing at construction
any rule it does not enforce. `RUNTIME_TYPES` also decides what the constraint
compiler renders (`guardrail/compiler.py`; `tests/test_csp.py` demands a
template per type), and there are no templates for those three yet, so the
loaders still refuse them: an over-refusal, never a rule silently dropped.
They join `RUNTIME_TYPES` in the change that adds the templates. A test keeps
`RUNTIME_TYPES` inside the Shield's `ENFORCED_TYPES`, checks that every type
the loaders refuse is either refused or enforced by the Shield, and prints
the over-refused set. distance_envelope is enforced by neither.

`altitude_ref: MSL` is declarable and refused for flight, by the loaders and
by the Shield's constructor: the Shield measures height above ground and has
no ground-elevation source.

The six breach actions are accepted and stored as written. Since 2026-10-07
the Shield dispatches on them (`guardrail/shield.py`, ENFORCEMENT): repair /
project_fix repair, monitor_only is recorded and never repaired, brake skips
projection and stops, and loiter / RTL / land stop the same way while the
escalation FSM (`guardrail/fsm.py`) requests the mode. What the Shield cannot
complete on its own - flying the requested mode, which takes a rail that
passes `ShieldDecision.set_mode` to an autopilot, and a soft rule's mode
action, which the FSM caps at brake - `load_for_flight` records as
`runtime_notes` in the run's policy source (`models.MODE_ACTIONS`), so a run
never reads as having flown an RTL it did not. A test checks each note
against what the Shield actually does on a tick. (Until then the note said
the Shield repaired, then braked, for every rule, which was true of the
Shield of 2026-10-06.)

### 6. Layered policies

`merge_layers` / `load_layered` / `python -m guardrail.bundle merge`. Each
rule's layer is its `layer` (absent = mission). Precedence is mission > site >
regulation: a rule id in several layers is taken from the highest. A higher
layer may tighten a lower layer's HARD rule, never relax it: a wider altitude
band, a smaller keep-out area (margin ring included), a narrower fence band, a
wider corridor or corridor band, a raised speed / climb / turn cap, a reduced
clearance or stand-off, a soft instead of hard rule, a lower priority, a
weaker breach action, an added or changed schedule, a changed altitude
reference, a changed type, any change to a dynamic_nfz / time_window_switch /
corridor_swap rule, a switch that turns it off, or a corridor swap that widens
it. What cannot be shown to be at least as strict is refused, and each of
these cases has its own test (since 2026-10-07; a review found half of them
untested). A lower layer's soft rule may be relaxed. Layers must state the
same origin, and layers mixing metre and lat/lon points must state one: the
metre rules would otherwise sit in a frame derived from whichever fence the
merge puts first. The result records `layers_merged` and is hashed
post-merge, as the grant says.

The grant states the precedence twice and the two read differently: the
layered-authoring section says "mission > site > regulation", the ingest
diagram "Layer merge regulation > site > mission precedence". Both are met,
each read as what it governs: the HIGHER layer's rule wins a same-id conflict
(mission over site over regulation), and a LOWER layer's hard rule prevails
over any attempt to relax it (regulation over site over mission).

### 7. Cross-loading with the reference (WP1-05)

Reading: `check_bundle` accepts a reference bundle when the hash of its
`ir.json` equals `Policy.reference_hash()` of the policy rebuilt from it -
which, both being SHA-256, means the bytes are exactly what
`Policy.reference_ir()` rebuilds; the hash is recognised as the named form
`reference-policy-dsl-0.1`. Writing: `write_bundle(..., ir_form="reference")`
ships the reference's document and hash. Measured with the reference's own
loader in its 3.11 environment (`tools/wp1_roundtrip_kpi.py`, read-only): it
computes the same hash as `reference_hash()` for 3/3 of its documents, and
reads 3/3 bundles written here.

**A reference-form bundle is an exchange format, not a flight artefact.** Its
IR writes every default out (`scope` and `layer` "mission", `altitude_ref`
"AGL", margin 0) and spells `repair` as `project_fix`, so the policy rebuilt
from it has a third hash: neither the manifest's nor the source policy's.
Measured on the one-envelope policy of
`tests/test_policy_dsl_grant_form.py` (no scope or layer, `repair`): source
`sha256:4bc4c5eb...`, manifest `sha256:b6fa8935...`, rebuilt
`sha256:0f52850e...`. A run flown from it would record a `loaded_policy_hash`
that resolves to no policy. So the flight loaders (`load_bundle`,
`load_for_flight`) refuse it and say why; `check_bundle` still reads it; the source policy still
recognises the manifest's hash (`hash_form` -> `reference-policy-dsl-0.1`);
and every flight's bundle source record now carries both
`declared_policy_hash` (the manifest's) and `loaded_policy_hash`. Bundles for
ITRI may be written in this form; bundles to fly are written in this
project's.

What still differs, precisely:

1. **The IR and its hash.** The reference hashes its authored document (grant
   field names, `geometry:` blocks, every default written out, `issued_at:
   null` when absent); this code hashes its own IR. The same policy therefore
   has two hashes. This code reproduces the reference's for every policy the
   reference can represent; the reference does not reproduce this code's, and
   reads only bundles written with `ir_form="reference"`.
2. **Rule types.** The reference implements polygon_fence and
   altitude_envelope only, and has no `valid_time` field. Its loader refuses
   the grant's worked example (the corridor) and keeps a document with a
   `valid_time` - dropping the schedule, same hash as without. A policy with a
   schedule, a margin, an origin or any other type is not written in the
   reference form (`Policy.reference_problems()` says why).
3. **Unknown keys and lint.** The reference ignores unknown keys (a misspelt
   `altitude_cieling_m: 40` is dropped and the ceiling stays at its 200 m
   default) and accepts `version: banana`, duplicate rule ids and MSL. This
   code refuses the first three and refuses MSL for flight.
4. **Defaults of an omitted field in an authored file.** priority: P0 here, P1
   in the reference. constraint_type and violation_action are required by the
   reference and default to hard / repair here. Bundled IRs carry every value,
   so cross-loaded bundles are unaffected.
5. **Margin.** A fence here may carry `margin_m`, a detection ring; the
   reference has none (its repair pushes 1 m outward). A grant-shaped fence
   gets margin 0 here, so both detect at the drawn boundary.
6. **Frame.** With no polygon the reference uses (0, 0); this code uses the
   first geographic rule. The reference's plane is (east, north); this
   project's is (north, east).
7. **Signature.** The reference writes a placeholder signer and never reads
   `signature.txt`; this code writes and checks an Ed25519 line. A reference
   bundle loads here as "unsigned".
8. **Python.** The reference needs 3.11 (`enum.StrEnum`); this code runs on
   3.10 and 3.11.
9. **Identity through an exchange.** A policy written in the reference form
   and read back here carries every default explicitly, so its hash here is
   not the source policy's (above). The source still recognises the bundle's
   hash; the rebuilt policy is read, not flown.

### 8. GeoJSON / KML and the ingest tool (WP1-16)

`tools/geo_to_policy.py convert` turns every Polygon / MultiPolygon in a
GeoJSON or KML file into a grant-form polygon_fence, validated by the DSL
before anything is written. It refuses holes, non-polygons, a non-WGS84 CRS,
duplicate ids and an out-of-range coordinate (the visible half of the
longitude/latitude swap; the test pins the other half with a zone longer
east-west than north-south). A vertex written twice in a row is read once. A
band not given by the feature or the command line is left out of the rule,
so the DSL's geometry-block default applies (0-200 m AGL, the reference's);
until 2026-10-07 the converter wrote an undocumented 120 m ceiling of its
own. `--origin` (the take-off point) inside a converted zone is refused.
`ingest` reads a policy file, GeoJSON / KML, or a REST payload in the shapes
`guardrail/api.py` speaks (a zone body of POST /nfz or POST /dynamic_nfz, a
typed POST /hot_apply rule, a GET /policy body), from a file, stdin, or the
endpoint itself (an http(s) URL fetched with GET, since 2026-10-07; the
grant's Outputs table says the tool "reads files + REST endpoint"), appends
rules to `--base` exactly as the API's hot-apply would (one generation per
rule; the test compares the hash with the API's own handlers) and emits a
bundle. A zone body becomes a dynamic_nfz: since 2026-10-07 the API spawns
POST /nfz as one, because the grant locks polygon_fence at mission start, and
until then this tool appended a polygon_fence. Such a bundle is a declaration
until dynamic_nfz joins `RUNTIME_TYPES` (decision 5). An endpoint that fails or returns an empty body is
refused by name. Geographic zones are refused onto a metre base
with no origin.

### 9. The published JSON Schema is the authoring form

The grant: the DSL is "the hand-authored surface form ... what an operator (or
an external GIS tool) writes", and its "Pydantic JSON Schema export is the
published contract for external tooling" (`policy_dsl.schema.json`). Until
2026-10-07 that file described this project's IR only, and it rejected the
grant's own documents (0/3 of the reference's; the 2026-10-05 schema also
rejected `wgs84_taipei.yaml`, 28/29 authored policies).

Now its root is the authoring form, built from the same tables the loader
reads (`GRANT_ALIASES`, `GRANT_GEOMETRY`), so the two cannot disagree about
which names exist: the grant's names and this project's, `geometry:` blocks,
`issued_at`, `layers_merged`, lat/lon with or without `origin`. Two names for
one field, a key both inside and beside `geometry`, an unknown key and an
empty rule list are refused, as the loader refuses them. The canonical IR -
what a bundle's `ir.json` holds - is published beside it as
`$defs/CanonicalIR` (its models under `$defs/IR.<Model>`); every canonical IR
is also a valid authoring document, and the grant's documents as written are
not canonical IRs. Measured: 29/29 authored policies and 3/3 reference
documents (plus the worked example, in the tests) validate against the root;
29/29 canonical IRs against `CanonicalIR`.

The schema checks shape only. It refuses 20 of the 35 negative cases by itself;
polygon validity, unique ids, conflicting limits and the like are beyond a
shape, and `load_policy` remains the authority.

## Measurements

`python tools/wp1_roundtrip_kpi.py` (writes `docs/data/wp1_roundtrip.json`):

Two nulls per column where they differ. "Accept everything" is the zero-skill
reader. "401305a" is this repository's code before this change (committed
2026-10-06, the state of 2026-10-05 for the DSL): measured where the code
existed (its loader on today's negative corpus, its published schema on the
reference documents - 2026-10-07), and 0 where the function a column calls
did not exist (`to_grant_form`, `reference_hash`, the reference IR form).

| Column | Now (2026-10-07) | Null: 401305a | Null: accept everything |
|---|---|---|---|
| round trip, policies/*.yaml | 29/29 | 29/29 | 29/29 |
| tamper refusal | 29/29 | 29/29 | 0/29 |
| negative corpus refused | 35/35 | 20/35 (14/25 on the original 25) | 0/35 |
| grant form round trip | 29/29 | 0/29 | - |
| reference documents loaded | 3/3 | 0/3 | 3/3 |
| reference bundle loaded | 1/1 | 0/1 | 1/1 |
| reference hash agrees (reference loader run) | 3/3 | 0/3 | - |
| reference loader reads bundles written here | 3/3 | 0/3 | - |
| schema: authored policies valid (root) | 29/29 | 28/29 | - |
| schema: reference documents valid (root) | 3/3 | 0/3 | - |
| schema: canonical IRs valid (`CanonicalIR`) | 29/29 | 29/29 (then the root) | - |
| schema alone refuses (negative corpus) | 20/35 | - | 0/35 |
| stored manifests resolving (`demo/out/*/`) | 86/86 (81/81 on 2026-10-06; 83/84 at 2026-10-07 06:30) | 34/86 (the formula before the identity release) | 86/86 |
| set aside in `demo/out/_*/`, listed not counted | 2, resolving 0/2 | - | - |

The negative corpus is not the corpus of 2026-10-05: 14/25 then and 35/35
now compare two corpora. On the original 25 slots it is 25/25 (2 replaced);
10 cases were added; the 401305a loader refuses 20 of today's 35. Two cases
were replaced because the grant's form made them valid: lat/lon with no
origin (now the grant's own form; replaced by a rule mixing frames) and
`circle_fence` as an unknown type (now defined; replaced by `cylinder_fence`).

The stored manifests are the 76 counted on 2026-10-05, 5 runs added on
2026-10-06 and the SITL runs other units wrote on 2026-10-07 (at 10:25: 42
resolve under `legacy16-exclude-none`, 34 under `legacy16-include-defaults`,
10 under `sha256-canonical-v3`). At 06:30 one did not resolve. By 10:25
another unit had moved it, with a second hot-apply run
(`ros2fix_on_dyn_nfz`), into `demo/out/_hot_apply_not_in_lock/`, out of the
census glob, and the census read 85/85 with nothing said. It now lists
set-aside runs separately with whether each resolves (2, resolving 0/2;
`tests/test_wp1_roundtrip_kpi.py` shows the old census dropping them). The
run, `ros2fix_on_unsafe_start`, recorded the policy hash AFTER a fence
was hot-applied at the aircraft's live position (`nfz-on-aircraft`, vertices
in fractional metres from the vehicle state). The lock's `derived` recipes
rebuild hot-applied policies from a fixed rule; a rule placed where the
aircraft happened to be differs on every run, so no recipe can rebuild it.
The run directory holds the snapshots that would (`policy_g0.json`, which
hashes to `sim_demo_policy.yaml`, and `policy_g1.json`, which hashes to the
recorded `sha256:06a67859...`); resolving a run against its own snapshots is
`guardrail/manifest.py`'s to do, not the lock's.

## Not done here

- distance_envelope is enforced by nothing (no map of people or roads) and
  MSL has no conversion; both are refused by every flight loader and by the
  Shield's constructor. dynamic_nfz, time_window_switch and corridor_swap
  are enforced by the Shield but refused by the loaders until the compiler
  can render them (decision 5). A requested flight mode is flown only where
  a rail passes `set_mode` to an autopilot; flights record that.
- The flight rails do not project a vehicle's GPS into a derived frame, so a
  grant-form policy must state its take-off origin to fly (refused otherwise).
  Whether the stated origin is the real take-off point is not something the
  DSL can know; a start inside a zone is recorded, not refused (decision 2).
- The lint does not check that hard corridors meet horizontally.
- `docs/DESIGN-policy-identity.md` section 5 still describes `issued_at` as a
  manifest-only field; see decision 3.
