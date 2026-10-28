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

Earlier changes (for example obstacle clearance becoming P0 on 3 August)
predate the tracked history of `guardrail/models.py` and cannot be dated
from git. No IR-shape change has been made since 2026-10-19: the lock's
later commits (2026-10-22 to 2026-10-28) pin new policies, not new fields.

## The deviation in this register

The 2026-10-19 row added fields without raising the version. That follows
this project's rule, not the grant's: since the canonical form v2 a new
field that defaults to None cannot change any existing policy's bytes, so
the lock allows exactly that change without a bump and refuses every other
one (a new field with a value default, a changed default, a removed field:
`python -m guardrail.bundle lock`, `tests/test_policy_hash.py`;
docs/DESIGN-policy-identity.md section 2). The grant asks for a bump on
every change. Either the PI accepts the additive exception, or the next
release raises `IR_SCHEMA_VERSION` to 2.2.0 and this row names it. Open
with the PI (WP1-22).

## Adding a row

Any change `python -m guardrail.bundle lock` reports is an IR-shape change:
give it a row here in the same commit, with the version after it, whether a
stored hash moved (`python tools/wp1_roundtrip_kpi.py`, section
`stored_runs`), and what reads the old form.
