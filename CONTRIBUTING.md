# Contributing

This file writes down what this project already does, so it is a rule rather
than a habit only one person remembers. If you are about to fix something,
add something, or move something, this is the checklist before you commit.

## The rule the whole project is built around

**A check that cannot fail is not a check.** Nine separate defects in this
repo were each a working program that produced a plausible, wrong number: a
safety rule that matched no real flight (`docs/FINDING-the-standoff-rule-that-never-armed.md`),
a tracking metric a frame-centre constant could pass
(`docs/FINDING-the-tracking-metric-a-constant-could-pass.md`), a verifier that
checked five of the seven fields it claimed to
(`docs/FINDING-the-verifier-that-verified-five-of-seven.md`). None of them
crashed. All of them looked done. Read the memory file
`silence-reads-as-success` (or any `docs/FINDING-*.md`) before assuming a
green run means the thing you built is correct rather than merely running.

Before trusting a check, try to break it on purpose: feed it the wrong input,
the tampered file, the zero-skill answer, and confirm it actually says no.

## Definition of done, for any change

1. **A regression test.** New module `foo.py` gets `tests/test_foo.py`
   (the project's own convention — every file in `guardrail/`, `demo/`,
   `tools/` that does real work has a same-named test file in `tests/`).
   A fix for a live bug gets a test that fails on the old code and passes on
   the new one — not a test that only exercises the happy path.
2. **A `CHANGELOG.md` entry**, under `### Added` / `### Changed` / `### Fixed`
   as appropriate. If the change corrects something already published — a
   number in a report, a claim in a doc, a figure in a deck — it goes under
   `### Retracted` with what was wrong and what the right value is, not just
   a silent overwrite. See the 0.4.0 and 0.5.1 entries for the pattern.
3. **A finding document, if the defect was silent** — i.e. it produced no
   error and no test failure, just a wrong answer. Name it
   `docs/FINDING-<what-was-wrong>.md`, following the existing 28: what was
   claimed, how it was found, the measured evidence, the fix. If it's a new
   design decision rather than a bug, it's `docs/DESIGN-<name>.md` instead.
4. **Every score needs its null.** A metric with no stated floor invites the
   question "what would something with zero skill score here" — answer it in
   the same commit, the way `demo/track_truth.py` reports a centre-constant
   and lag-1 baseline beside every tracking number.
5. **A manifest for anything that produces a number that gets quoted.**
   Flights write code revision, weights hash, policy hash, seed and topology
   to `manifest.json` precisely so a number can be traced back to what
   produced it. New tooling that emits a headline figure should do the same,
   or at minimum print the command that reproduces it (see
   `tools/build_eval_data.py`, which prints every value it writes).

## Where things go

| Kind of file | Goes in |
|---|---|
| Guardrail package code (policy DSL, compiler, shield, replay) | `guardrail/` |
| Flight controller, detector, scene scripting, recorder | `demo/` |
| One-off analysis, benchmarks, probes | `experiments/` |
| Build scripts, deck/report generators, repo utilities | `tools/` |
| ArduPilot SITL rail | `sitl/` |
| Tests (one file per module under test) | `tests/` |
| Policies (`.yaml`) | `policies/` |
| Launcher scripts (`.ps1` / `.bat`) | `scripts/` |
| Findings, design docs, reports, decks | `docs/` |
| Grant/reference PDFs, not ours to edit | `reference/` |

Do not invent a new top-level folder for one file. If nothing above fits,
that's worth a sentence in the PR/commit message explaining why, not a
silent new directory.

`docs/` doubles as the source for the published GitHub Pages site
(`docs/_config.yml`, served at the repo's Pages URL) — moving or renaming
files inside it changes live public URLs. Do that deliberately, not as a
side effect of a tidy-up.

## Commit messages

Narrative, not a one-line label. State what was wrong (or what was built),
how you know, and what the fix actually changes — the existing `git log` is
the style guide. `git log --oneline -20` before writing one if you're unsure.

## Line endings

The repo has mixed LF/CRLF per file (`core.autocrlf=false`). When editing a
file programmatically, preserve its existing ending — read it, check for
`\r\n`, and write back the same way. `Path.write_text` on Windows silently
converts LF files to CRLF and turns a one-line fix into a whole-file diff;
see the `repo-line-endings` project memory.
