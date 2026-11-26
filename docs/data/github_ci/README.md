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

## Release 0.7.1 and after (26 Nov)

Every push from 339cacf (24 Nov) through 9a5239c (45654591293, 25 Nov) was
green. Exported the same way on 26 Nov:

- `run_45762447982.json` (master) and `run_45762446649.json` (tag v0.7.1):
  the release 0.7.1 push of a3d616e, 26 Nov 05:34 UTC. Conclusion failure.
  The job "guardrail (3.11, extras)" failed in "Standalone test files" and
  its smoke step was skipped; "guardrail (3.10, extras)" and "guardrail
  (3.11, requirements only)" succeeded, smoke included. The failing file
  was tests/test_railinputs_node.py: two tests
  (test_a_hot_apply_over_rest_reaches_the_flying_shield_and_its_records,
  test_a_stranger_cannot_hot_apply_by_sending_its_request_with_its_finished)
  still sent REST hot-apply bodies with `{x, y}` metre points, which the
  release's own WP1-14 change (909804b) answers with HTTP 422 by design.
  The tests were stale; the code behaved as intended. tools/ci_local.py
  did not see it because its vla-real env has no fastapi and those tests
  skip there. Fixed in 35a2039 (the tests send lat/lon via
  `guardrail.models.wgs84_from_metres`), merged into master at 849b21a.
- `run_45838850365.json`: the push of master 220be42 (849b21a plus the
  lab-checkout ci_local record), 26 Nov 14:25-15:04 UTC. Conclusion
  success on all three jobs; "Standalone test files" and "Smoke profile
  (stress harness, headless)" succeeded in each, artifacts smoke-py3.10,
  smoke-py3.11 and smoke-py3.11-nosigning.

So the `tests` workflow was green per push from 24 to 25 Nov, red on the
release 0.7.1 push, and green again from 220be42.
