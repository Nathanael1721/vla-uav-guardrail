# DESIGN — Policy identity: one hash, one version, one signature

*2026-10-06. Audit cards WP1-22, X-09, X-10, WP1-04, ARCH-29, WP1-23,
WP3-21 (node side), ARCH-13, WP1-17, WP1-16 (schema export).*

The grant makes `policy_hash` the load-bearing reproducibility property:
"Replaying a flight means loading the bundle whose policy_hash matches — never
'the latest'" (Policy DSL, *Versioning and signing*). This document records how
that property had quietly stopped holding, what replaced it, and the rules that
keep it from breaking the same way again.

## 1. What was wrong, measured

`Policy.policy_hash` digested `model_dump()`, which writes every optional field
as `null`. The IR schema gained optional fields after the Q1 freeze, and each
one changed the bytes of every policy — with no YAML edited and no version
raised (28 of 29 policies still say `0.1.0`).

| Commit | Date | IR change | Effect on the old hash |
|---|---|---|---|
| `2fef997` | 2026-08-24 | first tracked `guardrail/models.py` | — |
| `dba9e48` | 2026-08-25 | `SubjectStandoff` (new rule type) | none for existing policies |
| `4950be1` | 2026-09-01 | `Corridor`; `valid_time` on every rule | every policy re-hashed |
| `445bdbe` | 2026-09-01 | `origin` (WGS84 anchor) on the policy | every policy re-hashed again |
| this release | 2026-10-06 | canonical form v2, IR schema 2.0.0 | see §2 |

Earlier changes (for example obstacle clearance becoming P0 on 3 August)
predate the tracked history of `models.py` and cannot be dated from git.

Measured with the code at `HEAD` (1d09786) against every
`demo/out/*/manifest.json` — 76 manifests, the only stored manifests on this
machine (`experiments/out` holds none; the four tracked replay bundles wrap
runs that are also in `demo/out`):

| | Manifests matching a policy on disk | The five ArduPilot + MAVROS 2 runs |
|---|---|---|
| Before (old hash) | **34 / 76** | **0 / 5** |
| After (this release) | **76 / 76** | **5 / 5** |

Example, `policies/sim_demo_policy.yaml`, a file not edited since 2026-08-24:
`sha256:8a311f9d22d22600` in July, `sha256:9a3782ae46925a46` from
2026-09-01, and now
`sha256:8a311f9d22d2260000eb2b24775c3e6cbfeb70416c8299b3354e47227a2476af`.

Reproduce: `python tools/wp1_roundtrip_kpi.py` (section `stored_runs` of
`docs/data/wp1_roundtrip.json`) and `python tests/test_policy_hash.py`.

## 2. The canonical form (v2)

`Policy.canonical_ir()` is `model_dump(mode="json", exclude_none=True)`;
`canonical_bytes()` serialises it with sorted keys, no whitespace, ASCII only
(the same serializer as the reference's `canonicalize()`); `policy_hash` is
`sha256:` plus **all 64 hex digits** of that (the reference keeps the full
digest too). The bundle's `ir.json` is exactly those bytes, so
`sha256sum ir.json` is the hash.

**Deviation from the reference, recorded rather than implied.** The same
serializer does not give the same hash. The reference's `build_ir`
(`policy_dsl/ir.py`) digests `doc.model_dump(mode="json")` of the *authored*
document, which keeps None as `null` (`issued_at: null` among them) and keeps
lat/lon. This code digests the *projected* IR in local metres with None left
out. So the same policy hashed by both implementations gives two different
strings, even where the field names agree. An earlier version of the
`canonical_json` docstring said "both halves of the grant agree on bytes"; that
was false and is withdrawn. Omitting None is the decision this whole release
rests on (it is what stops an additive schema change from re-fingerprinting
every policy), so it is kept and listed as a deviation for the PI to accept,
next to "policies are stored in local metres plus an origin" (PI question Q4).

Two decisions carry the design:

* **None is left out.** A field whose value is None means "absent", so a new
  optional field defaulting to None cannot change any existing policy's bytes.
* **Non-None defaults stay in.** If `margin_m` defaulted to 2.0 tomorrow, a
  policy that omits it would fly differently, and its hash must say so.

That makes the rule a property of the *schema*, so it is enforced on the
schema: `policies/policy.lock.json` pins every IR model's fields and what an
omitted field means (`guardrail.models.ir_schema_defaults()`). The only change
allowed without raising `IR_SCHEMA_VERSION` is a new field defaulting to None.
A new field with a value default, a changed default, or a removed field is
reported by `python -m guardrail.bundle lock` and fails
`tests/test_policy_hash.py`. The models are found by walking `Policy`'s fields
(`guardrail.models.ir_models`), not from a hand-written list, so a new
constraint type is seen the day it joins the `Constraint` union. A model the
lock has not pinned yet is reported until `lock --update` pins it; that adds
the pin and never rewrites an existing one.

**Superseded on 2026-11-02 (WP1-22 / X-09, the grant's freeze rule).** The
additive exception above is gone: any difference from the pin - a new
None-default field and a new model included - is reported until
`IR_SCHEMA_VERSION` is raised, and `lock --update` then re-pins the schema and
appends what changed to `ir_schema.migrations`. docs/IR-MIGRATIONS.md has the
rule and a row per version; 2.2.0 versions the 2026-10-19 `locked_by` change.
The None-omission itself stays: it is what keeps such a bump from moving any
stored hash.

A useful accident: for the None-free schema of July and August, the old 16-hex
hash and the new 64-hex hash are the same SHA-256 over the same bytes. The old
hash is literally the first 16 hex digits of the new one
(`Policy.policy_hash_short`).

## 3. Legacy verification

Nothing stored is rewritten. `Policy.hash_form(recorded)` names the form a
recorded hash is in, or returns None:

| Form | Bytes hashed | Carried by |
|---|---|---|
| `sha256-canonical-v2` | None-free canonical JSON, 64 hex | runs from 2026-10-06 |
| `legacy16-exclude-none` | the same, 16 hex | 42 stored runs (to 2026-08-31) |
| `legacy16-include-defaults` | `model_dump()` with `valid_time`, `recurrence`, `origin` as null | 34 stored runs (2026-09-01 to 2026-10-06) |
| `legacy16-valid-time` | as above without `origin` | the hours between `4950be1` and `445bdbe`; no stored run |

A run that hot-applied a rule mid-flight recorded the hash *after* the change,
which no file reproduces. The lock's `derived` list holds the recipe: the
`--dynamic` runs (`ros2_shield_on_dynamic`, `sitl_shield_on_dynamic`, both
`sha256:77d64d2e5e94ac39`) are `sim_demo_policy.yaml` plus the `nfz-dynamic`
fence, generation 1. A test reads `DYNAMIC_FENCE` out of both SITL rails and
requires it to equal the recipe, so the two cannot drift apart.

`guardrail.manifest.resolve_run_policy(manifest, guardrail.bundle.policy_candidates())`
is the one resolver: every policy file plus every derivation, every form. It
returns `(None, None, None)` rather than the nearest policy.

`guardrail.replay` binds an episode under any known form and writes the form
into the index (`flown_hash_form`); `guardrail.bundle.check_bundle` loads a
pre-release bundle (16-hex, IR with nulls, placeholder signer) and reports it as
legacy and unsigned. A legacy hash digests the *rebuilt* object, so it cannot
see a key the model drops; for legacy bundles `check_bundle` therefore also
compares the archived IR with what the policy dumps back to, and refuses an
unknown key there too (v2 bundles catch it by hashing the bytes).

## 4. Versions: the lock

`policies/policy.lock.json` maps `policy_id@version` to one content hash, with
its legacy forms beside it — the old→new table the audit asked for, for every
policy. The rule: an `id@version` names exactly one content hash, forever.

* Change a policy without raising `version:` → `check_lock` reports *content
  changed but version did not*; the bundle CLI refuses to publish it.
* Raise the version → the new pair must be locked:
  `python -m guardrail.bundle lock --update`. That only *adds* entries; an
  existing entry is never rewritten, because old entries are how a stored run's
  hash stays traceable after the policy moves on.

No policy version was raised in this release: the contents did not change, only
the hash algorithm, and bumping `version:` would itself change the content hash
and orphan the runs this release exists to recover. The lock starts from today's
`(id, version)` pairs and enforces bumps from here on.

**Known defect, recorded rather than hidden (resolved 2026-10-21, WP1-22):**
nine generated scenario files (`hard_*.yaml`, `hp_*.yaml`,
`random_scenario.yaml`) shared `random-scenario@0.1.0` with nine different
contents. The lock lists them under `shared_by_files` (each file still pinned)
and refuses a tenth claimant. The fix was a distinct `policy_id` per file:
`demo/gen_random_scenario.py` now gives each generated file its own id and
refuses one another file claims, and the eight `hard_*` / `hp_*` files were
re-id'd as `random-scenario-<stem>` (`hard_3.yaml` -> `random-scenario-hard-3`;
no rule changed, but the id is part of the content, so each hash moved). Each
gained its own lock entry carrying `previously: random-scenario@0.1.0`, which
`check_lock` requires of a file that left the shared identity. Only
`random_scenario.yaml` still claims `random-scenario@0.1.0`. The shared entry
is deliberately NOT rewritten (an existing entry never is: it is how a run
flown under the old id still traces to its content), so its `known_defect`
string is historical: it describes the 2026-10-06 state, and its "left to
whoever owns policies/*.yaml" was done on 2026-10-21.

## 5. The signature

`signature.txt` keeps the reference's first line, `<policy_hash> <signed_by>`,
and adds a second: `ed25519:<hex>`, an Ed25519 detached signature over the exact
`manifest.json` bytes. The manifest carries `policy_hash`, and `policy_hash` is
the SHA-256 of `ir.json`, so one signature covers the chain. The manifest now
also records `issued_at`, `hash_scheme` and `ir_schema_version`.

* **Key.** A lab *development* key, `lab-dev:ed25519:6ec75b535ef5c533`. Its
  public half is tracked in `policies/keys/trusted_signers.json`; the private
  half is `policies/keys/lab-dev-ed25519.pem`, gitignored (with every other
  private-key spelling under `policies/keys/`). Override with
  `VLAGUARD_SIGNING_KEY` / `VLAGUARD_TRUST_STORE`. *Since 27 Nov 2026* this key
  and the AGX Orin's are certified by the lab root CA (authority `lab-ca`, a lab
  decision, not the PI's) and sign as `lab-ca:71d1e071d3985bc6:ed25519:<key>`;
  the bare `lab-dev` identities were retired at 2026-11-27T00:10:37Z and verify
  only what they signed before (docs/RUNBOOK-bundle-ca.md,
  docs/data/bundle_ca_20261127/).
* **The CA stays swappable.** The grant leaves "Lab-internal CA or ITRI CA?"
  open. The trust store is data: since 16 Nov an authority is a CA certificate
  entry and each key a certified signer entry (`issue-cert` / `register-cert`);
  moving to ITRI's CA is one more entry, a re-sign with `--authority itri-ca`,
  and `retired_on` on the lab-ca signer entries (docs/RUNBOOK-bundle-ca.md,
  'Later: moving to ITRI's CA'). No code changes.
* **Verified on every rail.** `cryptography` is optional. The 3.11 `vla-drone`
  env lacks it, and the setup scripts of the two SITL rails' WSL venvs
  (`sitl/setup_ros2.sh`, `sitl/setup_sitl.sh`) never install it (read from the
  scripts; the venvs themselves were not inspected). The first cut of this release answered
  `unverifiable-here` there, so two of three flight rails could never verify a
  signature, and under `--allow-unverified-bundle` a forged signature flew
  reading the same as a real one (review, 2026-10-06). Verification now falls
  back to a pure-Python Ed25519 verifier (`guardrail.bundle.py_ed25519_verify`,
  RFC 8032 §5.1.7, following the RFC's §6 reference code). Verifying uses only
  public data, so a plain big-integer implementation leaks nothing; it takes
  about 3 ms per bundle. It is pinned to the RFC's test vectors on every
  interpreter and checked against `cryptography` where both exist. Signing
  still needs `cryptography` and the private key.
* **Never a silent pass.** Each check returns one status: `verified`;
  `unsigned`, *only* for the two layouts that honestly name no key (the
  reference placeholder, and `unsigned:no-signing-key`); `untrusted-signer`,
  decided from the trust store before any cryptography runs, so it is reported
  in every environment; `bad-signature`, always refused; and
  `unverifiable-here`, now reachable only with the fallback switched off.
  `load_bundle` refuses everything except `verified` unless the caller passes
  `require_signature=False`, and the flight entry points record the status.
* **A stripped signature is refused.** Dropping the Ed25519 line while keeping
  the key's identity used to read as `unsigned`, with the trusted signer's name
  attached and a detail claiming it "names no key". No writer produces that
  layout, so it is `bad-signature` now, in policy and replay bundles alike, and
  `--allow-unverified-bundle` does not fly it. The mirror case (an Ed25519 line
  under a keyless identity) is refused the same way.
* **Reproducible.** Ed25519 is deterministic, so the same policy, key and
  `issued_at` give the same bytes. `issued_at` comes from the caller, else
  `SOURCE_DATE_EPOCH`, else now; it never touches `ir.json` or `policy_hash`.
* **`issued_at` is a manifest field: a deviation.** The grant's worked DSL
  example (Policy DSL p.3) and the reference `PolicyDoc` carry `issued_at`
  inside the hashed policy document. Here it lives in the bundle manifest, so
  re-issuing an unchanged policy never moves its hash, and the manifest is
  covered by the signature instead. Pydantic would drop a top-level
  `issued_at:` in a YAML without a word, so `load_policy` refuses one and says
  where it belongs (`--issued-at`).
* **Replay bundles** are signed the same way over `replay.json`. The four
  tracked ones predate the key: they verify, and `verify_replay` says
  `signature UNSIGNED` every time. The honest limit that remains: dropping the
  Ed25519 line *and* replacing the signer with the keyless placeholder makes a
  new bundle read as keyless — loud, not refusable, because keyless bundles are
  real.

## 6. Flights fly the bundle

`sitl/ros2_shield_node.py`, `sitl/run_sitl_demo.py` and `demo/follow_vlm.py`
take `--bundle PATH` (and `--allow-unverified-bundle`) and load through
`guardrail.bundle.load_for_flight`. YAML stays the fallback, recorded as
`signature: unsigned`. Both at once is refused. The `policy_source` record goes
into `metrics.json`, `kpi.json` and the replay index — *beside* the manifest,
not in it, because the grant fixes the determinism manifest at six fields. The
ROS 2 node logs its policy hash at start-up and now writes a replay bundle.
The policy is loaded before the run's output folder is touched: `follow_vlm`
used to clear the previous run's flight log first, so a refused bundle on a
re-used tag destroyed the old log and orphaned its manifest and `kpi.json`.

No flight has been flown from a bundle yet (that needs the simulators), and
`sitl/ros2_vla_stub_node.py` still reads its own YAML, so the two-node ROS 2
rail does not yet load one bundle on both sides.

## 7. Topology: `dev`, not "canonical HIL"

The grant has three topologies (reference `vlaguard_common.manifest.Topology`):
`dev` (one desktop), `hil` (VLA and Shield on a Jetson Orin, "the canonical KPI
configuration") and `flight`. Our ArduPilot SITL + MAVROS 2 rail is `dev`; the
code called it `canonical-hil` and graded it KPI-grade. Now:

* every writer emits `dev`; stored `canonical-hil` is *read* as `dev`
  (`normalize_topology`) and never rewritten;
* `build_manifest` refuses `hil` and `flight` (no rail here runs an Orin);
* `is_kpi_grade` refuses `dev` unless a written PI waiver is recorded in
  `guardrail.manifest.DEV_KPI_WAIVER` (question PQ1). Under today's gate the
  five stored MAVROS runs are **not** KPI-grade (5 → 0); their stored
  `kpi.json` still says what the gate said at flight time, and
  `tools/build_eval_data.py` now reports both. The WP1 census calls them
  `mavros_dev_five`, not "KPI-grade".

## 8. The WP1 round-trip KPI

`python tools/wp1_roundtrip_kpi.py` → `docs/data/wp1_roundtrip.json`:

| Measure | Result | Null: a loader that checks nothing |
|---|---|---|
| Round trip (load → bundle → load, same hash, IR and dump) | **29 / 29** | 29 / 29 |
| Tampered bundle refused | **29 / 29** | 0 / 29 |
| Negative corpus refused (`policies/negative/`) | **14 / 25** | 0 / 25 |
| Schema validation against `policy_dsl.schema.json` | 29 / 29 | — |

The round trip alone cannot tell a real loader from one that verifies nothing;
the refusal columns can. The 11 broken policies the loader **accepts** are
findings, not fixed here (validation strictness is out of scope for this unit):
a misspelled rule key (the fence silently keeps the 1000 m default ceiling), a
bow-tie polygon, a zero-area polygon, a duplicate rule id, a duplicate YAML key,
an empty rule list, an empty `policy_id`, a non-semver version, an infinite
speed cap, a NaN altitude bound and a negative fence margin. Each file states
its defect and how the loader treats it, and a test fails if that header goes
stale. WP1-17 therefore stays *partial*: the card asks for every negative case
to be refused (11 of 25 are not) and for the N/N to be quoted in the KPI
report (no report quotes it yet).

`python -m guardrail.bundle schema` writes `policies/policy_dsl.schema.json`
(the grant's "DSL JSON Schema export"; it describes the canonical IR in local
metres — authored lat/lon is projected by `load_policy` first).

## 9. Not done here

* No GeoJSON/KML converter and no REST ingest (WP1-16's other parts).
* No `layers_merged` (there is no layer merge to record) and no generated
  changelog diff; `changelog` is still free text. (Since closed: the diff
  against the previous version, `changelog_diff`, from 2026-10-29 (X-10) in
  `write_bundle(previous=...)`, and in every flight bundle from 2026-11-21
  (X-10b): `tools/build_flight_bundles.py` diffs each policy against its
  highest lower published version in `bundles/` and its `--check` flags a
  bundle without the diff. The flight bundles were rebuilt with it on 25 Nov
  and re-signed under the Lab CA on 27 Nov, 17 of them; older versions in
  `bundles/` keep their bare-key signature.)
* `sitl/ros2_vla_stub_node.py` and `sitl/run_ros2_demo.sh` still read their own
  YAML (WP1-23's stub side); they are outside this unit.
* No flight was re-flown from a bundle; that needs the simulator. The
  pure-Python verifier is tested on Windows 3.10 and 3.11, not yet inside the
  WSL venvs themselves.
* The delivered mid-term and mid-evaluation reports were not regenerated; only
  their generators were corrected.
* `load_bundle` now refuses an unsigned bundle by default. Two callers outside
  this unit write-then-read bundles and break on a machine without the lab key
  (`tools/build_deck_data.py` `bundle_facts`) or would refuse an unsigned
  scenario bundle (`guardrail/scenario_spec.py` `PolicyRef.load`); the change
  each needs is listed for their owners.

## 10. The WGS84 conversion (2026-11-03, IR schema 3.1.0, WP1-14)

Until this date 31 of the 32 policy files were written in local metres; the
grant makes WGS84 canonical. Each was rewritten by
`tools/convert_policies_wgs84.py` with a stated `origin` and every point in
lat/lon - new content, so each raised its patch version, as section 4
requires. What keeps every stored run traceable:

* **The local view is exact.** A WGS84 point's metres are computed when first
  read (`models.XY`), from its policy's frame, and rounded to the micrometre
  (`projection.local_metres`). The projection is sub-metre over a mission's
  area, so the rounding resolves nothing it could; it makes a converted
  point give back exactly the metres it was written from (all 257, checked),
  so the Shield's geometry is the same floats.
* **The metre form is the old policy.** `Policy.metre_form()` writes the
  converted policy back in metres about its origin, without the origin, rings
  in the order given. At the old version its canonical bytes are the old
  file's (checked per file by the tool), so it carries the old 64-hex hash,
  IR 2.x forms and 16-hex forms.
* **The lock links the two, and checks the link.** Each new entry says
  `migrated_from: <id>@<metre version>` with `migration.old_hash_form`
  `sha256-canonical-v3/local-metres` (`bundle.FRAME_FORM`). `bundle.rewind`
  reads an old version back - the same content at that version (an IR change,
  section 3) or the metre form at it (this one) - and `check_lock` re-proves
  every link on every run. `policy_candidates` walks the chain step by step
  (WGS84 -> metres at 3.0.0 -> metres at 2.x), so `resolve_run_policy` finds
  a run flown under any of them, and the `derived` recipes rebuild on the
  metre base they were written for.
* **Earlier schemes of a WGS84 policy hash the unrounded metres.** v2 and the
  16-hex eras stored projected metres; `prior_hashes` and `legacy_hashes`
  compute them with the unrounded projection of their day (the "era" dump), so
  `wgs84_taipei.yaml`'s old forms are unchanged. A pre-3.1.0 bundle of a WGS84
  policy shipped unrounded multi-scale geometry; it is read within half a
  micrometre of the rounded view.

Measured (4 Nov): 386 of the 396 stored manifests in the lab checkout resolve
directly with the base code and with this one, each to the same id@version
and content; the other 10 (ros2_hotapply_*, ros2_action_*,
gazebo_shield_on_dynamic_20261102) changed their policy in flight and resolve
through their own generation chain (`manifest.resolve_run_generation_chain`),
10 of 10 with either code. Every one of the 19 bundles there verifies.

## 11. Scope required (2026-11-06, IR schema 4.0.0, WP1-01)

The grant's ConstraintBase requires `scope`; it was optional here, absent in
every shipped policy, and an absent scope meant "mission". Making it required
is a breaking change for every writer (IR schema 4.0.0), and every policy
file's content changed (each states `scope: mission` on every rule now), so
each raised its patch version, as section 4 requires. What keeps every stored
document readable, and every stored run traceable:

* **A stored document is read in its own era.** `models.era_context(v)` is
  the validation context under which a document of IR schema `v` before 4.0.0
  - or of a format that records no version - may leave `scope` out; it stays
  absent (None), so the canonical bytes, and every hash form, are the ones
  recorded. `effective_scope` reads the absence as "mission". The readers of
  stored artefacts pass it: `check_bundle` (the manifest's
  `ir_schema_version`), `derive` (the recipe's), `read_replay` (the replay's
  policy manifest), the run readers (`generation_table`,
  `resolve_run_generation_chain`: a run's policy_g<N>.json records no
  version). Without that context - every ingest, every rule built in code - a
  missing or null scope is refused.
* **The old version is the file with scope left out.** `Policy.ir3_form()`
  drops every rule's scope and changes nothing else (the given ring order and
  the points' frame are kept). At the old version its canonical bytes are the
  old file's, checked for all 32: the old entry's 64-hex hash, IR 2.x forms
  and 16-hex forms. Each new lock entry says `migrated_from` the old one with
  `migration.old_hash_form` `sha256-canonical-v3/ir-3.x` (`bundle.IR3_FORM`),
  `bundle.rewind` reads it back that way, and `check_lock` re-proves every
  link. `policy_candidates` walks the chain one more step (scope stated ->
  WGS84 without scope -> metres at 3.0.0 -> metres at 2.x).
* **A manifest names the schema its IR is a document of.** Since 4.0.0
  `manifest_for` records `Policy.ir_schema_version`: 4.0.0 for a policy that
  states every scope, 3.1.0 for one of an earlier era that leaves it out (the
  two shapes differ only in that requirement). A run of an earlier era packed
  again (a replay bundle, a re-bundled old policy) is then read in its era,
  where recording the writer's version would have made it unreadable.

Measured (6 Nov, read-only, the lab checkout's gitignored stores with this
code): 36 of its 36 bundles open - 31 flight bundles written under IR
schema 2.1.0 (8), 3.0.0 (10) and 3.1.0 (13), every signature verified, each
under the hash its manifest records (24 under the current scheme, 8 under
their IR 2.x form), and 5 replay bundles (the 4 legacy ones, 16-hex;
hil_smoke_03) - 171 rules, every scope absent as written. Every one of the
405 stored manifests there resolves with this code exactly as with the base
code (master 30667cc): 392 directly, each to the same id@version and the
same canonical bytes under the same hash form, and 13 through their own
generation chain, whose base the label now names as the old version of its
file (`corridor_survey.yaml as corridor-survey@0.1.2 -> policy_g4`).
`tools/wp1_roundtrip_kpi.py` counts 404/404 of them (docs/data/wp1_roundtrip.json).
