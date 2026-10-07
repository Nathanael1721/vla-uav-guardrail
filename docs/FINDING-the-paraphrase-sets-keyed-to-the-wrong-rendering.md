# The paraphrase sets were keyed to the wrong rendering

**Date:** 2026-10-06
**Found by:** the independent review of the Paraphraser (`guardrail/paraphraser.py`).
**Status:** fixed: the stored sets are keyed to the CSP text the compiler
actually emits and also matched by rule set; a test checks that every in-scope
text is served.

## What was claimed

The Paraphraser serves stored, validated paraphrases of every instruction the
system issues, so the per-paraphrase robustness KPI (WP4) can be measured on
fixed, reproducible wordings. The first version reported: "a stored set exists
for every in-scope instruction". Six of the stored sets cover rule text: the
natural-language sentences of a policy.

## What went wrong

The compiler renders a policy's rules in two ways:

- `build_prompt()` (the older surface, still used by `demo/run_demo.py` and the
  SITL rails): one sentence per rule, in rule **type** order;
- `compile_csp()` (the grant's Prefix Compiler): the same sentences, filtered
  and in **risk** order, P0 first.

The six rule-text sets were keyed to `build_prompt`'s text. The CSP that a
flight puts in the model's prompt is `compile_csp`'s. Same sentences, different
order, so an exact-text lookup found nothing: the stored backend raised
`UnknownSource` for **all six** CSPs as `compile_csp` emits them. The claim was
true only of the rendering that no constrained VLA reads.

Nothing failed loudly, because the coverage check enumerated `build_prompt`'s
texts. A run asking for a paraphrase of its actual CSP would have been refused
at flight time, or, under `backend="auto"`, served template text instead of
the stored free-form set.

## The fix

- The six sets were re-keyed to `compile_csp`'s rendering. Their sentences are
  unchanged and the paraphrases were not regenerated; each set records
  `generated_from` and a `rekeyed` note.
- Lookup also matches by rule set (`rule_set_key`: the sorted sentences), so
  both renderings of the same rules get the same set under the same ids.
- `python -m guardrail.paraphraser stale` lists every in-scope text (each
  scenario cell's `compile_csp` with and without its clock, `build_prompt`, the
  city policies, the mission texts) and the set that serves it.
  `test_every_instruction_in_scope_has_a_stored_set` enumerates the same list:
  21 in-scope texts, all served by 14 sets of 8.
- `auto` falls back to templates only with a logged WARNING and
  `provenance.fallback_from = "stored"`, so a fallback can never be measured as
  if it were the stored set.

## The general lesson

"Covered" has to be checked against what the system emits, not against what a
neighbouring function renders. Two renderings of one object are two artefacts;
a guarantee about one says nothing about the other until a test enumerates the
one that ships.
