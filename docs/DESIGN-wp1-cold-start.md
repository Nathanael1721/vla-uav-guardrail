# WP1 cold start - a signed 50-rule bundle to a ready Shield, timed

**Date:** 2026-10-19
**Status:** measured and published. `tools/wp1_roundtrip_kpi.py` has a `cold_start`
section, written into `docs/data/wp1_roundtrip.json` (its `kpi` block carries
`cold_start_50_rules` and `cold_start_within_budget`, so `tools/kpi_report.py`'s WP1
table shows them beside the 2026-10-05 null). `tests/test_wp1_roundtrip_kpi.py` pins
the budget. Desktop figures; the onboard computer (Jetson Orin) is not measured.
**Audit:** docs/AUDIT-KONTRAK-2026-10-05.md, "KPI WP1 (... cold-start <= 1 s,
hot-apply <= 50 ms) belum dilaporkan sebagai KPI".

## What the grant asks for

Policy DSL page, "Acceptance KPIs (locked)": **"Cold-start ingest of a typical 50-rule
bundle <= 1 s"**. The ingest diagram on the same page names the cold-start path: parse
and schema validation, geometry normalisation, lint, "Build runtime IR (R-tree / grid
spatial index ...)", sign and version, and "Cold-start path ... runs once per mission
start".

Until today only one piece had a number: Shield construction at 50 rules, median 4.7 ms
(2.7 ms at 401305a; docs/DESIGN-shield-enforcement.md). The 50-rule bench calls that
"only the IR part of a cold start - no YAML parse, no bundle signature check".

## What is timed

The path a flight rail takes at mission start, through the rails' own entry points:

1. `guardrail.bundle.load_for_flight(bundle, trust_store=...)`: the tar.gz read; `ir.json`
   parsed and validated by the pydantic models (WGS84 points projected); `policy_hash`
   recomputed over the canonical IR and compared with the manifest; both hash forms
   verified; the DSL lint; the Ed25519 signature verified against a trust store read from
   disk; the flight gate; `runtime_notes`.
2. `guardrail.replay.rail_shield(policy, lookahead_s=5, dt=0.1)`: the Shield every rail
   builds, i.e. the runtime IR (compiled margin rings, the STRtree, schedules,
   corridors).

In this process (warm: imports done) and in **fresh interpreters**: one process per run
that times its own imports of `guardrail.bundle`, `guardrail.replay` and
`guardrail.shield`, then the first call of the path, then the same path again. The
parent also times each child from spawn to exit (interpreter start-up included). The KPI
reading is the fresh process with imports, worst of the two bundles; the whole-process
wall time is printed beside it.

Two bundles of exactly 50 rules, each signed with a fixed bench key whose public half
sits in a trust store file beside them, so the status is `verified` on any machine:

| Bundle | Rules |
|---|---|
| `typical` | WGS84 with an origin (projected at load): 32 polygon fences of 6 to 12 vertices with margin rings, 10 circle fences (each a derived 32-gon), 2 keep-in corridors, an altitude and a kinematic envelope, obstacle clearance, a pedestrian stand-off, 2 time-window switches; 20 rules on weekly schedules. 42 zones to compile. |
| `grid` | `experiments/bench_shield_50rules.fifty_rule_policy()`: 2 envelopes + 48 square fences in metres, the load every monitor budget here is quoted against. 48 zones. |

## Why a fast number is not enough

A path that skipped the signature, the hash or the IR would post the best time. So
every timed run must also return a verified signature, the issued `policy_hash`, 50
rules and every zone compiled; one Shield per bundle must raise a geofence violation
from inside its first fence; and the same path must refuse a tampered bundle, one
signed by a key the trust store does not hold, and one whose Ed25519 signature has one
hex digit changed (`bad_signature`). `verdict.within_budget` is true only when all of
that holds and the p95 fits. The test
`test_a_cold_start_that_skips_the_work_is_not_within_budget` swaps in a loader that
verifies nothing, a Shield that compiles no zone, and a loader that checks the hash and
the signer's identity but never verifies the signature: all three are faster, and all
three read as a failure (each refusal it lets through is named in the record).

**The third refusal (added 2026-10-20, branch review).** With only the first two, nothing
showed that the signature itself is verified: the tampered IR is refused by the
`policy_hash` comparison, the stranger's bundle by the trust-store lookup, so the
identity-only loader above still read `checks_hold` true and `within_budget` true. The
`bad_signature` fixture is the issued bundle with one digit of its `ed25519:` line
changed (the low nibble of the first byte of S, so it still parses and S stays canonical);
its IR, manifest, hash and signer are as issued. It counts as refused only when the
loader's refusal is the verification's own ("Ed25519 verification failed", a plain
`ValueError`; guardrail/bundle.py raises `BundleSignatureError` only for a signature that
is absent or untrusted). `test_the_bad_signature_refusal_is_the_signature_check_and_nothing_else`
checks the fixture differs in that one character and is refused with both verifiers
(`cryptography` and the pure-Python RFC 8032 one).

**Provenance and hygiene (same review).** The file's `command` is the line that
reproduces it from a full checkout; since 2026-10-20 it also records `command_args`, the
flags it was generated with (paths relative to the checkout, or reduced to
`<main checkout>/...` when a worktree read the main checkout's runs and reference
repository), and `command_env`, whether `VLAGUARD_SIGNING_KEY` was set. `cold_start`
removes its fixture folder when it ends (it left a `wp1cold_*` folder per run), the
tool's other bundle folders are removed at exit, and the tests register theirs.

## Measured, 2026-10-19

Desktop (Intel64 Family 6 Model 183, Windows), Python 3.11.15 (`vla-drone`,
`cryptography` 50.0.2 as the verifier), `docs/data/wp1_roundtrip.json` generated at
bf255b1 by `python tools/wp1_roundtrip_kpi.py` (20 fresh interpreters and 100 in-process
runs per bundle). The desktop was shared with other work while measuring.

**Fresh interpreter, one process per run** (median / p95 / max, ms):

| | typical (WGS84, mixed) | grid (metres, 48 squares) |
|---|---|---|
| imports (`guardrail.bundle`, `.replay`, `.shield`) | 344 / 361 / 363 | 343 / 360 / 379 |
| `load_for_flight` (read, parse, validate, hash, lint, verify, gate) | 12.0 / 12.9 / 13.5 | 7.4 / 7.9 / 7.9 |
| `rail_shield` (runtime IR) | 6.0 / 6.5 / 6.7 | 4.5 / 4.7 / 4.9 |
| path, first call | 18.1 / 19.1 / 19.7 | 11.8 / 12.3 / 12.6 |
| **ingest with imports (the KPI)** | **362 / 379 / 382** | **354 / 371 / 391** |
| whole process, spawn to exit | 453 / 580 / 648 | 446 / 496 / 600 |
| the same path again in that process | 19.4 / 21.1 / 22.0 | 7.8 / 8.2 / 9.3 |

**KPI: p95 379 ms against the grant's 1000 ms** (worst bundle, imports included); the
whole process including the interpreter's own start-up stays under 1 s at every one of
the 40 runs (max 648 ms). Every run verified the signature, returned the issued hash,
50 rules and every zone (42 / 48); the tampered and the untrusted bundle were refused.
Imports are about 95 % of the cold start; the ingest proper is 12-19 ms, 2 % of the
budget.

In this process (after the tool's other sections, so with more garbage-collection
pressure; median / p95): typical 24.5 / 32.6 ms, grid 14.6 / 16.0 ms. Of the warm
typical path, the IR parse and validation is 5.6 ms, the tar read 0.6 ms and the
Ed25519 check 0.4 ms. With the pure-Python RFC 8032 verifier instead (what a WSL venv
without `cryptography` uses): 28.1 / 30.1 ms and 20.7 / 21.4 ms. The author's side of
the grant's diagram (YAML -> parse, normalise, lint -> canonical IR, hash -> signed
bundle) for the typical policy: 87 / 98 ms, same `policy_hash` as the bundle timed.

Other interpreters, from the test's 3-run measurement the same day (p95, imports +
path, ms): Python 3.10.20 (`vla-real`) typical 385 + 17 (wall 489), grid 376 + 10 (wall
468); 3.11 with `cryptography` blocked in the parent 338 + 17. Not in the published file.

**The test** (`test_the_cold_start_fits_the_grants_budget_and_proves_it_did_the_work`)
pins the path (fresh process, first call) at <= 1 s everywhere and the import-inclusive
ingest at <= 1 s on the desktop; on a shared CI runner (`GITHUB_ACTIONS`) the
import-inclusive bound is 2 s and the numbers are printed, as the 50-rule monitor test
does with its 15 ms CI bound.

## What else moved in the regenerated file

`docs/data/wp1_roundtrip.json` was last written on 2026-10-08 (10451bc, Python 3.10.20).
Regenerated from a git worktree with `--runs-root` and `--reference-repo` pointing at the
main checkout (its gitignored flight runs and the reference repository, read-only) and
`VLAGUARD_SIGNING_KEY` at its lab development key, so the round-trip bundles are signed
as before (`verified` 29/29). Besides the new `cold_start` section and the two KPI
lines: `python` 3.11.15; the stored-run census 97/97 (89/89 on 10-08; the 8 new runs
are all `sha256-canonical-v3`, and the 2 set-aside runs are unchanged); and one
negative case's refusal reason reads as the October validator words it ("time '25:99'
out of range (00:00-23:59; 24:00 only as an end_time)"), still refused. Every other
figure is unchanged.

## What this does not give you

- **The Orin.** The grant's budget is for the onboard computer. Re-run `python
  tools/wp1_roundtrip_kpi.py` there (ROS 2 container, its Python) before quoting a
  margin for it.
- **A reboot-cold disk.** One discarded fresh run first warms the OS file cache and the
  `.pyc` files, as on any second mission of a day.
- **Hot-apply.** The grant's other WP1 latency (dynamic_nfz <= 50 ms) is measured by
  `experiments/bench_shield_50rules.py` (`hot_apply`), not here.
