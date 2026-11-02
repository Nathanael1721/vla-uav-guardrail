# heldout_2026_11_02: paraphrase-validator probes

94 hand-written probes: 50 meaning changes and 44 faithful paraphrases. Each
one is a `(source, candidate, meaning_changed, label)` row in `probes.json`, in
the format of `experiments/paraphrases/probes.json`.

## Who wrote it

Claude (an Anthropic model, `claude-opus-5-5`), running as a coding agent on
2026-11-02. It was the same agent that was then asked to fix the validator.
No person wrote or edited the probes. The set is therefore **not independent of
the fixer**: the author knew a fix would follow. It was committed on its own
before the agent read any validator code or measured anything, so the commit
history shows that it predates the fix.

## What the author had read when it wrote the set

- The grant pages in `kuanting-vla-uav-guardrail/docs`:
  `02-implementation/prefix-compiler.md` (the CSP's
  `natural_language_prompt` is "templated, never free-form"; free-form
  paraphrasing is the paraphraser's job, a separate Q3 service; P0 coverage
  100 %), `01-context/grant-overview.md` (WP4 KPI: per-paraphrase robustness)
  and `03-simulation/data-flow.md` (the paraphraser service feeds the
  orchestrator, which issues tasks and paraphrased variants to the VLA).
- The sentence and summary templates in `guardrail/templates/`, including their
  comments (the C8 note: a named-class stand-off binds only to the followed
  subject).
- The current templated NL. The author rendered it by running
  `ConstraintCompiler.compile_csp` on the policy fixtures, and read the 14
  `source_text` strings of the paraphrase store.
- The **structure** of `probes.json` (keys, set names and each set's `status`
  text) and a few lines of CHANGELOG.md found by grepping for "held-out".
  Those status texts and lines name some earlier mechanisms: "'with' phrases,
  place nouns, 'in uniform', narrowing adjectives, prepositions read as
  narrowings". Those were not used as a checklist.

It had **not** read `guardrail/paraphraser.py`, `tests/test_paraphraser.py`,
`docs/DESIGN-paraphraser.md`, `docs/FINDING-the-paraphrase-sets-keyed-to-the-wrong-rendering.md`,
any stored paraphrase or any candidate in an earlier probe set.

## Sources

There are 17 sources, all the compiler's current rendering:

- the 8 CSP prompts of the paraphrase store;
- 3 policy fixtures the store has never paraphrased (`urban_clearance`,
  `orbit_building`, `follow_car_citylife_nfz3`), to test generality;
- the 6 task texts.

The four pedestrian CSPs use the post-C8 wording "the pedestrian you are
following", not the stale "any pedestrian" still stored in the paraphrase
files.

## Labelling rule

A candidate is **faithful** only if it keeps every rule of the source and adds
none. For each rule it must also keep:

- each number's value and unit (spelled-out numbers, "metres", "≤" and "<" are
  fine);
- each comparator's direction (a ceiling stays a ceiling, a minimum stays a
  minimum);
- each zone or corridor id and whether it is keep-in or keep-out;
- the schedule ("Mon-Fri 07:30-17:30" may be written as weekdays, 7:30 am to
  5:30 pm);
- the subject each stand-off binds to (anything followed, the followed
  pedestrian);
- the task's object class, colour, count, direction and target.

Reordering, merging, splitting and telegraphic lists are faithful.

Anything else is a meaning change, **even when it is stricter**. For example,
adding a zone, adding a schedule to a permanent zone, or narrowing "any
building" all change the meaning.

The `label` field names the mechanism, for example
`changed: half-width 20 m read as a 20 m total width`.

The author avoided cases whose label it could not defend, such as
strict-versus-non-strict swaps ("at or below" to "under") or "person" to
"pedestrian". It kept one hard faithful case on purpose: zones
'nfz-edge-1' **through** 'nfz-edge-4'. That candidate keeps the meaning but
does not spell out two of the ids.

## How it is used

The set is measured in three stages:

1. **Before any change.** Only the counts are looked at, not which probes
   failed.
2. **After the fixes drawn from heldout_2026_10_28's misses alone.** This is
   the honest held-out figure: no fix has seen this set's misses yet.
3. **Analysis and fixes.** Its own misses are then analysed and fixed by
   mechanism. From then on it is a **dev set**, and its later figures are
   regression floors, not estimates.

CHANGELOG.md (2026-11-02) records the figures at each stage.
