# IR migration register

*Started 2026-10-29 for audit items WP1-22 and X-09. One row per change of
the Policy IR's shape after the grant's Q1 freeze (2026-04-30).*

The grant asks that the IR shape freeze at the end of Q1 and that "every
later IR-shape change carries a semver bump and a migration note" (Policy
DSL page, R6 freeze gate). This file is that register. The IR shape is the
set of models `policies/policy.lock.json` pins under `ir_schema`
(`guardrail.models.ir_models`, found by walking `Policy`'s fields), and
`guardrail.models.IR_SCHEMA_VERSION` is its version.

How to read a row: **version** is `IR_SCHEMA_VERSION` after the change
("-" where it did not exist yet); **stored hashes** says whether any
policy_hash already recorded by a run or a bundle moved; **migration** is
what a policy, run or bundle from before the change needs to be read today.

| Date | Commit | Version | Change | Stored hashes | Migration |
|---|---|---|---|---|---|
| 2026-08-25 | `dba9e48` | - | `SubjectStandoff`, a new rule type | none moved (no existing policy carried it) | none |
| 2026-09-01 | `4950be1` | - | `Corridor`, a new rule type; `valid_time` on every rule | every policy re-hashed (`model_dump()` wrote the new field as `null`) | the old hash is recognised as `legacy16-valid-time` (`Policy.hash_form`, docs/DESIGN-policy-identity.md section 3); nothing rewritten |
| 2026-09-01 | `445bdbe` | - | `origin` (the WGS84 anchor) on the policy | every policy re-hashed again | recognised as `legacy16-include-defaults` (34 stored runs, 2026-09-01 to 2026-10-06) |
| 2026-10-06 | `401305a` | 2.0.0 | canonical form v2: the hash digests the IR with None left out, all 64 hex digits (`sha256-canonical-v2`); the schema pinned in the lock for the first time | the canonical form itself is new; every stored run still verifies (76 of 76 manifests, docs/DESIGN-policy-identity.md section 1) | 16-hex runs verify as `legacy16-exclude-none` or `legacy16-include-defaults` |
| 2026-10-07 | `9bdc331` | 2.1.0 | new rule types `circle_fence`, `corridor_swap`, `distance_envelope`, `dynamic_nfz`, `time_window_switch` (and their `Motion`); new optional fields `altitude_ref`, `layer`, `scope` on the rules, `issued_at` and `layers_merged` on the policy, `lat` / `lon` on a vertex (19 fields, pinned as additive) | none moved: every new field defaults to None and the canonical form leaves None out | none; 2.0.0 is in the lock's `previous_versions` |
| 2026-10-19 | `2385f84` | 2.1.0 (not raised) | `locked_by` on every rule, the layer-merge provenance (`merge_layers` writes it), 11 fields pinned as additive | none moved: absent on every rule no layer overrode | none |
| 2026-11-02 | this release | 2.2.0 | no field changed on this date: the 2026-10-19 `locked_by` change, versioned under the freeze rule (below); the lock records it in `ir_schema.migrations` (2.1.0 -> 2.2.0, no difference, with why) | none moved | none; an artefact recording `ir_schema_version` 2.1.0 reads unchanged |
| 2026-11-02 | this release | 3.0.0 | WP1-01 / WP1-02: `constraint_type`, `priority` and `violation_action` required on every rule (33 pins, default -> `<required>`; they defaulted to hard / P0 / `repair`); `violation_action` holds the grant's six only - `repair` is read as `project_fix` on input and never stored; keep-out rings (`polygon_fence`, `dynamic_nfz`) stored in canonical order (positive winding over the stored pairs, smallest pair first; `models.canonical_ring_order`), and a derived frame origin averages the ring in that order | every shipped policy moved (all 32 spelt `repair`): each raised its patch version (`fase3-sim-demo` 0.1.1 -> 0.1.2, the rest 0.1.0 -> 0.1.1) and its new lock entry names the old in `migrated_from`; the old entries are unchanged | IR 2.x hashes are reproduced, not rewritten: `Policy.ir2_hashes` (rings as given, `repair` - forms `sha256-canonical-v3/ir-2.x` and `.../ir-2.x-project_fix`), the v2 and 16-hex legacy forms over the same view; `check_lock` re-proves every `migrated_from`; `policy_candidates` offers each file at its old version, so a run recorded under it resolves; `check_bundle` reads a 2.x bundle under its 2.x form; a 2.x hot-apply recipe that omits the three fields is read with the 2.x defaults (`bundle.derive`). A rule written without the three fields is now refused - 382 construction sites in code and tests state them |

Earlier changes (for example obstacle clearance becoming P0 on 3 August)
predate the tracked history of `guardrail/models.py` and cannot be dated
from git. Between 2026-10-19 and 2026-11-02 no IR-shape change was made: the
lock's commits of 2026-10-22 to 2026-10-28 pin new policies, not new fields.

## The freeze rule (since 2026-11-02)

Every change to the IR shape - additive ones included - needs a semver bump
of `guardrail.models.IR_SCHEMA_VERSION` and a row here, in the same commit.
`python -m guardrail.bundle lock` (and `tests/test_policy_hash.py`) report any
difference between the code's schema and the lock's pin while the version is
unchanged, and `lock --update` refuses to pin it; under a higher version
`lock --update` re-pins the schema and appends what changed to the lock's
`ir_schema.migrations` (from, to, on, changes). A version never goes back.
`tests/test_policy_hash.py` also fails when the current version, or any
version the lock has pinned, has no row here.

Until 2026-11-02 this project had its own exception: since the canonical
form v2 a new field that defaults to None cannot change any existing
policy's bytes, so the lock let exactly that change (and a new model) through
without a bump. The grant makes no such exception, and the 2026-10-19 row
went in under it; 2.2.0 versions it. The None-omission stays: it is what
keeps a bump for an optional field from moving stored hashes.

## Adding a row

Any change `python -m guardrail.bundle lock` reports under the IR schema is
an IR-shape change: raise `IR_SCHEMA_VERSION`, give it a row here in the same
commit, with the version after it, whether a stored hash moved (`python
tools/wp1_roundtrip_kpi.py`, section `stored_runs`), and what reads the old
form; then run `python -m guardrail.bundle lock --update`.
