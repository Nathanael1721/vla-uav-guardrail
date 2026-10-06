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

**Known defect, recorded rather than hidden:** nine generated scenario files
(`hard_*.yaml`, `hp_*.yaml`, `random_scenario.yaml`) share
`random-scenario@0.1.0` with nine different contents. The lock lists them under
`shared_by_files` (each file still pinned) and refuses a tenth claimant. The fix
— a distinct `policy_id` per file — changes their hashes and belongs to whoever
owns `policies/*.yaml`.

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
  `VLAGUARD_SIGNING_KEY` / `VLAGUARD_TRUST_STORE`.
* **The CA stays swappable.** The grant leaves "Lab-internal CA or ITRI CA?"
  open. The trust store is data: to switch, add the chosen authority's public
  key to `trusted_signers.json`, sign with `--key`, and (when the dev key should
  stop being accepted) remove its entry. No code changes.
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
  changelog diff; `changelog` is still free text.
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
