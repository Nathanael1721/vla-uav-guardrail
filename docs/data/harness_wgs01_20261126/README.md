# wgs01: a harness-driven hil episode whose REST bodies carry lat/lon only (26 Nov 2026)

One episode of the stress harness (`python -m guardrail.harness_service serve
--rail hil --rail-campaign wgs01 ...` and `tools/harness_hil_episode.py`) on
the hil rig (AGX Orin + ArduCopter 4.5.7 SITL over MAVROS 2) at release
0.7.1, a3d616e, sim_speedup 1.0, KPI-grade (kpi.json `kpi_grade` true,
manifest `code_revision` a3d616ec6ec8). Scenario dyn-nfz-rotate-step, seed
1001, signed bundle fase3-sim-demo v0.1.4. Its purpose: the first flight
since WP1-14 (909804b, 26 Nov: the entries refuse a point in metres; a
nested geometry.vertices / centerline REST body is not yet checked, see
CHANGELOG 2026-11-26, corrections (b)) in which a policy change reaches the flying Shield over the Policy
DSL's REST hot-apply, so the lat/lon-only REST form is shown on the rig, not
only in tests. It is the harness twin of hh02_001 (14 Nov, the same scenario
and seed).

## What the harness sent (harness_events.jsonl)

| t (harness) | Request | Body | Shield (rail) | Generation |
|---|---|---|---|---|
| 3.016 s | POST https://10.42.0.3:8071/hot_apply (mutual TLS) | dynamic_nfz `bar`, four `{lat, lon}` vertices, no x/y | applied at rail t 3.312 s, receipt to swap 9.837 ms, round trip 172 ms | 1 |
| 8.000 s | PATCH https://10.42.0.3:8071/dynamic_nfz/bar | `{"op": "rotate", "angle_deg": 90}` (a relative edit, in degrees) | applied at rail t 8.280 s, receipt to swap 5.464 ms, round trip 47 ms | 2 |

Neither was refused. `policy_g1.json` and `policy_g2.json`, the policies the
Shield flew after each, hold no `x` key (lat/lon only). The node's
`events_hot_apply.jsonl` extract shows the listener (mutual TLS, client
`harness-client`), both `policy_generation` and `hot_apply_rest` records and
the fences re-published to the autopilot for each generation.

## Outcome

P0 escape 0 (null passthrough 0.0606); `tick_ms` max 49.89 ms, 0 of the 399
ticks with a measured `tick_ms` over 100 ms (650 ticks in the episode); the grant's six bundle names written (manifest.json,
policy_bundle.tar.gz complete with generations 0-2, episode.bag2/,
shield_audit.jsonl, harness_events.jsonl, kpi.json). The harness's verdict
was `fail` on its ideal-vehicle fail-safe label ("labelled not expected,
triggered": the FSM triggered RTL); the same cell in the release smoke
(smREL071_020) reads "false" under the ideal-vehicle reading (a) and correct
under the adopted rail reading (r2) (docs/data/kpi_smREL071/rescore_r2_smREL071.txt).

Not shown by this flight: a metre body refused by the rig's listener. The
HTTP 422 for {x, y} / [x, y] / x-y-beside-lat-lon bodies is tested
(tests/test_api.py and the 26 Nov WP1-14 tests), not flown.

## The first attempt

The first launch (26 Nov 15:52 TST) died at bring-up before take-off:
`ModuleNotFoundError: No module named 'uvicorn'` in the Shield node's
`serve_hot_apply` (`first_attempt_mission_console_excerpt.txt`; the client's
refusal in `first_attempt_client_stdout.txt`). The companion image
rebuilt on 22 Nov (`vlaguard/companion:jazzy`) did not carry the REST layer of
docs/RUNBOOK-orin-hil.md section 15a. The 22 Nov image was kept as
`jazzy-noapi-20261122`, the layer rebuilt on it from the Orin's
`~/wheelhouse-api` (fastapi 0.162.0, uvicorn 0.54.0; numpy 1.26.4 and
pydantic 2.13.4 unchanged) and tagged `jazzy`; the runbook now says to
re-apply it after every base rebuild. The failed run folder was moved aside
by the rail to `<store>/hil_runs/wgs01_001_dyn-nfz-rotate-step/_previous/`.

## Files

harness_events.jsonl, harness_request.json, kpi.json, manifest.json,
manifest_extras.json, metrics.json, policy_bundle.tar.gz (copied from the
harness bundle); policy_g1.json, policy_g2.json, scenario_score.json and
events_hot_apply.jsonl (the hot-apply, policy_generation and fence lines of
the run's events.jsonl, with the fence uploads they caused) from the run folder
`<store>/hil_runs/wgs01_001_dyn-nfz-rotate-step`; the client's console output
of both attempts, and an excerpt of the first attempt's mission console. The bag, shield_audit.jsonl and the replay tarball stay in
the run folder and the harness bundle (`<store>` = F:/vla_drone_offload).
