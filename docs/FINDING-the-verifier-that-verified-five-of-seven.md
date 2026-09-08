# The bundle verifier checked five fields and reported seven

**Date:** 2026-09-08
**Found by:** an adversarial review of the replay bundle written the day before.
**Status:** fixed, with regression tests, and one defect deliberately left open
because it cannot be closed without a key.

## What it was for

`guardrail/replay.py` closes WP4's last named artefact. Its whole claim is that
"replayable" means *re-derivable*: `verify_replay()` reloads the policy from the
archived IR, recomputes the KPIs from the archived log, and compares them to the
archived KPI table. `docs/CHECKLIST-remaining-work.md` marks item 7 CLOSED on the
strength of that property.

Six defects. Two of them cut straight through the claim.

## 1. Two of the seven verified fields do not exist

```python
VERIFIED_KPI_FIELDS = (
    "p0_violation_escape_rate", "p0_ticks", "p0_ticks_not_measurable",
    "fail_safe_correctness", ...
)
```

`p0_ticks` and `fail_safe_correctness` are not keys that `guardrail/kpi.py` ever
emits. The real names are **`p0_violation_ticks`** and
**`failsafe_trigger_correctness`**. And the compare loop began:

```python
if field not in stored:
    continue
```

So both were skipped in silence. Five fields were compared where seven were
claimed, and one of the two missing was **the grant's fail-safe correctness
KPI**, target ≥ 99 %.

Demonstrated: edit a bundle's `episode/kpi.json` so
`failsafe_trigger_correctness` reads 0.10 and `p0_violation_ticks` reads 0,
rebuild the index and the signature — `verify_replay` returns `(True, [])`.

Two typos, no error, and the module reported a property it was not checking.
That is this project's recurring failure mode appearing *inside the code written
to catch it*.

**Fixed.** The names are corrected, `p0_escapes` and `max_time_to_safe_s` are
added, and a name that neither the stored table nor `compute()` produces is now
a **hard failure of verification** rather than a skip: a typo in that tuple can
no longer hide.

## 2. The policy guard was skipped for any run without a manifest

`manifest.json` is optional in `EPISODE_FILES`, and both the write-side and
read-side policy checks were gated on its presence. For a run directory without
one, **any policy bundled with any episode**.

The KPI recompute is not a backstop, and this is the part worth keeping:
`kpi.compute` treats an unknown `rule_id` as P0, so substituting a foreign policy
changes **no KPI field at all**. Measured — a real scored flight, manifest
removed, bundled under each of the 26 other policies in `policies/`:

```
corridor_survey.yaml   bundled OK -> verify True []
follow_car.yaml        bundled OK -> verify True []
wgs84_taipei.yaml      bundled OK -> verify True []
... 26 of 26 identical ...
fields that DIFFER under the wrong policy: {}
```

Two runs on disk today (`vlm_stopgo`, `vlm_nfz_smooth`) have no manifest, so this
was reachable without touching a fixture.

`tests/test_replay.py` had a test named
`test_it_refuses_a_policy_that_did_not_govern_the_episode`, whose docstring
describes this exact failure. It passed, because its fixture happens to have a
manifest. **A green test named after the hole, beside the open hole.**

**Fixed.** A run carrying `kpi.json` must carry the manifest that binds those
figures to a policy — bundling one without is refused, naming the reason. An
episode with no KPI table may still be bundled unbound, and the index records
`policy_binding: "unverified"`, which `verify_replay` reports out loud.

## 3–4. The archive was trusted beyond what it checked

`read_replay` iterated `index["members"]`, never `tar.getmembers()`, while its
docstring said "check every digest". An extra member in the archive was neither
digested nor rejected — a second KPI table, another policy IR, an operator's
note, all uncovered. And a bundle whose index omitted the log died on
`KeyError: 'episode/flight_log.jsonl'`, naming no bundle and no reason, which is
the exact failure `bundle.py`'s `_read` was written to prevent.

**Fixed.** Every archive member must be declared or the bundle is refused, and
required members are checked before use so the error names them.

## 5. The signature stops nothing, and a test said otherwise

`signature.txt` is `sha256(replay.json)` with **no key**. Anyone who edits a
member can recompute the index and then the signature in one line. The module
docstring admitted the log-plus-index case; the *test* claimed the opposite:

> `test_rewriting_the_index_to_match_a_forged_member_still_fails`
> "The signature covers the index, so consistency is not enough."

It passed only because its helper replaced exactly one member and forgot
`signature.txt`. Staged properly, the forgery sails through:

```
read_replay ACCEPTED; index['run'] = a-flight-that-never-happened
verify_replay -> (True, [])
```

**Not fixed, and cannot be** until there is a real key — which is an open
question for the lab or ITRI, recorded the same way in `guardrail/bundle.py`.
What is fixed is the claim: the docstring now states the limit plainly, and the
test asserting the false property has been replaced by
`test_a_CONSISTENT_forgery_passes_because_the_signature_has_no_key`, which
asserts the true one. It will start failing the day a key exists, which is
exactly when someone should look at it.

## 6. The deliverable existed in one working tree, and the suite hid it

Replay bundles were written to `demo/out/<tag>/` (ignored by `demo/out/*`) and to
`bundles/` (ignored by `bundles/*.tar.gz`). Unlike a policy bundle — byte-derivable
from tracked YAML, which is why `bundle.py` ignores its output — a replay bundle
wraps a flight log that is itself ignored. **It cannot be rebuilt from a clone.**
A reviewer cloning the repository to collect the WP4 artefact would find the code
that writes bundles and no bundles.

Worse, the test suite concealed it. `tests/test_replay.py` short-circuits on a
missing fixture, and the runner printed `PASS` for every short-circuit. On a
clean clone it reported **"8/8 passed"** for a run in which seven tests asserted
nothing.

**Fixed both ways.** `.gitignore` now keeps `bundles/*.replay.tar.gz`, and the
two bundles that exist are retained (~100 KiB each, which is what an episode
compresses to). And every runner distinguishes SKIP from PASS:

```
SKIP  test_one_flipped_byte_in_the_log_is_caught_and_named (fixture missing)
...
1/15 passed, 14 skipped
```

The same runner change was applied to `test_occ_bands.py`,
`test_frame_contract.py`, `test_deck_scorer_parity.py`, `test_track_truth.py`
and `test_range_and_lock.py`, all of which had the same shape.

## What this round is really about

Every one of these six was in code or tests written **the previous day**, by me,
specifically to close the gap where a claim outruns what is checked. Three of
them are that same gap in a new place: a field list that names what it does not
compare, a guard skipped on a condition its own test never exercises, and a
runner that reports success for work it did not do.

The lesson that generalises is narrower than "be careful":

> **A check that cannot fail is not a check.** Before trusting one, make it fail
> on purpose — feed it a forged bundle, a wrong policy, a missing fixture — and
> watch it say so. Everything above was found by trying to break something and
> discovering it did not break.
