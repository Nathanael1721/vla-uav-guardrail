# GitHub Actions runs on the fork (WP4-06, WP4-14)

`run_45497539573.json`: the `tests` workflow (.github/workflows/tests.yml) on
the push of master 339cacf to the fork (Nathanael1721/vla-uav-guardrail),
24 Nov 2026 22:05-22:55 UTC, exported with `gh run view 45497539573 --json
...`. Conclusion success on all three jobs (Python 3.10 extras, 3.11
extras, 3.11 requirements only); in each, "Standalone test files" and
"Smoke profile (stress harness, headless)" succeeded and the smoke results
were kept as the run's artifacts smoke-py3.10, smoke-py3.11 and
smoke-py3.11-nosigning (held by GitHub, not copied here).

The last green run before it was 13 Oct (38471368810); the 22-23 Nov runs
(44976329320, 44991484912, 45156163225) failed in the test step before the
smoke. 339cacf carries the release 0.7.0 code (21c2771) plus the 25 Nov
fixes (CHANGELOG 2026-11-25 (code)).
