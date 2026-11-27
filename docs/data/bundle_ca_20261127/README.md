# The policy bundles under the Lab CA, 27 Nov 2026 (WP1-04, X-10)

The grant's Policy DSL page ("Versioning and signing") asks every published
bundle to carry `signed_by`, a "Lab CA or ITRI CA reference", and lists
"Lab-internal CA or ITRI CA?" as an open question. On 27 Nov the user chose
the **Lab CA** (a lab decision; the PI has not named a CA). This folder is
the state after docs/RUNBOOK-bundle-ca.md steps 1-7 ran on both hosts. The
state before is docs/data/bundle_ca_20261126/. A later move to ITRI's CA is
written down in the runbook ("Later: moving to ITRI's CA") and tested
(tests/test_bundle.py,
`test_moving_to_another_ca_retires_the_old_ca_identity_and_keeps_its_bundles`).

## What was done

| Step | Commit | What |
|---|---|---|
| 1-2 | 7144b7f | `python tools/build_flight_bundles.py --issue-cert` on the desktop: authority `lab-ca` (CN=VLA Guardrail Lab Root CA, CA certificate SHA-256 `71d1e071d3985bc66ee1b310a6e12e2f2c3892ecf733e9e2aeaf6143cef718f4`, valid to 2036-10-30), one codeSigning leaf per bundle key, issued from its public half: `lab-ca:71d1e071d3985bc6:ed25519:6ec75b535ef5c533` (desktop) and `lab-ca:71d1e071d3985bc6:ed25519:715f70c4647a9701` (AGX Orin; its private key never left it). The bare `lab-dev` identities were retired at 2026-11-27T00:10:37Z. |
| 2 | (gitignored bundles/) | 17 flight bundles re-signed on the desktop: the tool's 12 defaults, plus `policies/sitl_pedestrian.yaml`, `experiments/policies/follow_car_citylife_people.yaml` and the three `experiments/policies/msl_dem_*.yaml` (with `--ground experiments/worlds/dem_cmac_wgs84.json`). Which policies the hil rail flies came from dry runs of the smoke, nightly, fsm_edges, distance_envelope, msl_dem, grant_example, prefix_ab and prefix_dyn profiles; the CityLife ones are the perception rail's. |
| 5 | cda127e | `VLAGUARD_REQUIRE_CHAIN=1` in deploy/topologies/hil.env and flight.env; dev.env 0. |
| 4 | at cda127e on the AGX | the same 17 bundles rebuilt there (issued_at 2026-11-27T00:17:47-49Z) and signed by its own key under its `lab-ca` identity; the AGX was moved to 9589623 for the record and the flight, with no change to bundles/. |
| - | 988434b, 9589623 | `tools/bundle_ca_evidence.py --drill` fixed (it died on test_bundle.py's `wgs84_fixture` import since 26 Nov); the ITRI move documented and tested. |

Every bundle was copied before the re-sign: `<store>/bundles_pre_ca_20261127`
on the desktop (56 files, SHA256SUMS; `<store>` = F:/vla_drone_offload) and
`~/bundles_pre_ca_20261127` on the AGX (56 files, SHA256SUMS).

In each of the 17, policy_hash, policy_id, version and the IR are the ones the
bundle had before; the manifest's `signed_by`, `signer` (CA fingerprint,
leaf serial and subject, certificate SHA-256) and `issued_at` changed (the
re-sign is issued now, after the bare identities were retired). Four
bundles first built before X-10b (msl-dem-sitl, -zone, -corridor,
follow-car-citylife-people) now carry its first-version changelog text.

## The records

| File | Host | How |
|---|---|---|
| `bundle_ca_desktop.json` | lab desktop (Windows, Python 3.10.20, cryptography 49.0.0) | `python tools/bundle_ca_evidence.py --drill --out ...` at 9589623 (clean), 27 Nov 00:28 UTC |
| `bundle_ca_agx.json` | AGX Orin (aarch64, Python 3.12.3, cryptography 49.0.0) | the same tool without `--drill`, written to the git-ignored `deploy/evidence/incoming/` at 9589623 (clean), 27 Nov 00:28 UTC, copied back |

Both hosts show the same: **1 authority** (`lab-ca`, fingerprint matching
its record), **2 certified signers, both with chain verified here, 0 bare
signers in force, 2 retired**; policy bundles: **17 verified with chain
verified**, 31 verified with chain none (older versions signed by the bare
keys before their retirement, which still verify for what they signed then
and are refused by `--require-chain`); replay bundles 4 verified, 4 unsigned
legacy. The desktop drill: **10 of 10 PASS**.

Also checked by hand on both hosts: 17/17 pass
`python -m guardrail.bundle verify <bundle> --require-chain`; a bundle still
signed by a bare key (fase3-sim-demo-v0.1.3) fails it with rc 1; on the
desktop the 8 tracked replay bundles still verify, and
`tools/build_flight_bundles.py --check` prints OK for all 17.

## The verification mission: ca1127_001_nfz-head-on

One mission on the hil rig (AGX Orin + ArduCopter 4.5.7 SITL over MAVROS 2),
flown from the detached worktree at 9589623 with both hosts at that commit
and clean, `smoke` profile `--only nfz-head-on`, seed 1001, sim_speedup 1.0
(the other five arms of the cell are refused by the profile's design, as in
every smoke).

| Field (metrics.json `policy_source`) | Value |
|---|---|
| kind / path | bundle, `bundles/fase3-sim-demo-v0.1.4.tar.gz` (the Orin's copy) |
| declared = loaded = expected policy_hash | `sha256:b665adbaac29186240d5a74a8fa36a94b69adae2db6035ca69d45c8e93e7f358` (the same hash ntFIN5 flew) |
| signed_by | `lab-ca:71d1e071d3985bc6:ed25519:715f70c4647a9701` |
| signature / chain | verified / verified ("its certificate chains to authority 'lab-ca'") |
| require_chain | true |
| issued_at | 2026-11-27T00:17:47Z |

The run's own replay bundle (`ca1127_001_nfz-head-on.replay.tar.gz`, in the
run folder) is signed as the same `lab-ca` identity and passes
`python -m guardrail.replay verify`.

Outcome: KPI-grade (kpi.json `kpi_grade` true, manifest `code_revision`
9589623a55d6, topology hil, sim_speedup 1.0). P0 escape 0 (6 P0-violation
ticks, 0 escapes, 6 repairs); the FSM went Normal -> Brake -> Loiter -> RTL
and reached home. Fail-safe: correct under the adopted rail reading (r2) and
under (r), 1/1; the launcher's "FAIL ... labelled not expected, triggered" is
the ideal-vehicle reading (a), as for this cell in the release smokes
(`rescore_r2_ca1127.txt`). Ticks: `tick_ms` max 16.29 ms, e2e max 16.39 ms,
0 of 333 measured over 100 ms (650 ticks in the episode). The track video is
`ca1127_001_nfz-head-on_track.mp4` in the run folder (1.0 MB).

Files here from the run folder `<repo>/demo/out/hil_smoke/ca1127_001_nfz-head-on`:
`kpi.json`, `manifest.json`, `scenario_score.json`, `ca1127_001_metrics.json`
(its metrics.json), `ca1127_sweep.json` (the campaign's sweep.json) and
`rescore_r2_ca1127.txt` (`experiments/sweep_scenarios.py --rescore-rail`),
paths sanitised.

## What this does not show

- The KPI runs of record (ntFIN5 at 0.7.0, smREL070, smREL071) flew before
  27 Nov, signed by the bare keys, with the chain requirement off; their
  records say so and are unchanged. Only ca1127_001 flew under the CA.
- The Lab CA is the lab's own root (it also issues the harness's mutual-TLS
  certificates). Its private key is an unencrypted file on the lab desktop;
  docs/RUNBOOK-bundle-ca.md says to move it to lab-controlled storage at
  hand-over. No revocation list exists: retiring or removing a trust-store
  entry is the mechanism.
- Whether the Lab CA is acceptable to the PI or ITRI is not recorded; the
  move to ITRI's CA needs ITRI's CA certificate and two leaves.
