# Runbook: put the policy bundles under the CA the PI names (WP1-04, X-10)

The grant's Policy DSL page ("Versioning and signing") asks for each bundle's
`signed_by` to be a "Lab CA or ITRI CA reference". The code path is built and
tested since 16 Nov (guardrail/bundle.py; the swap drill in
tests/test_bundle.py), but which CA signs is the PI's decision (12 Oct, Q5;
section 9 of docs/FINAL-REPORT.md, decision 1), and on 22 Nov the user
decided to wait for it. Until then every bundle is signed by a bare lab key
(`lab-dev:ed25519:...`, chain `none`); the state before the CA is recorded in
docs/data/bundle_ca_20261126/.

This runbook applies the PI's answer in one session: certify the two bundle
keys, re-sign the flight bundles on both hosts, check them, require the chain
on `hil`, fly one verification mission, and commit the evidence. Every
command below exists at the time of writing; the flags are those of
`tools/build_flight_bundles.py --help`, `python -m guardrail.bundle
issue-cert --help`, `python -m guardrail.bundle register-cert --help` and
`tools/bundle_ca_evidence.py --help`.

**Applied on 27 Nov 2026 with the Lab CA** (a lab decision, taken by the user
on 27 Nov; the PI has not named a CA): steps 1-7 ran on both hosts; the
record is docs/data/bundle_ca_20261127/. The grant's Policy DSL page lists
"Lab-internal CA or ITRI CA?" as an open question and accepts either as the
`signed_by` reference. What remains is the move to ITRI's CA if the PI or
ITRI asks for it: section "Later: moving to ITRI's CA" below. Nothing here
may invent an authority: an ITRI CA entry is added only from the CA
certificate and leaves ITRI issues.

## 0. Before you start

- The two hosts: the lab desktop (holds the lab root CA's key and the
  desktop's bundle key) and the AGX Orin (`nathan-agxiot@10.42.0.3`, checkout
  `~/vla-drone`, holds its own bundle key; its private key never leaves it).
- The two bundle keys the trust store pins
  (`policies/keys/trusted_signers.json`): `lab-dev:ed25519:6ec75b535ef5c533`
  (desktop) and `lab-dev:ed25519:715f70c4647a9701` (AGX Orin).
- Nothing else on the rig: no campaign running (docs/RUNBOOK-orin-hil.md,
  step 14), the desktop on a branch, not on master.
- `cryptography` on both hosts (the chain is checked only where it is
  installed; desktop `vla-real` and the Orin's `~/vla-drone-env` have
  49.0.0, docs/data/bundle_ca_20261126/).

## 1. Put the named authority in the trust store (desktop)

The trust store is tracked in git, so this is one commit that both hosts
then carry.

**If the PI names the Lab CA** (its key is on the desktop, in the PKI folder
of `python -m guardrail.harness_service gen-pki`, by default
`%LOCALAPPDATA%\vla-guardrail\harness_pki\`): step 2 does it in one command
(`--issue-cert`), which lists the CA as authority `lab-ca`, issues a signer
certificate for BOTH keys from their public halves, pins each under its new
identity `lab-ca:<CA>:ed25519:<key>`, and retires the bare `lab-dev`
identities for what is issued from then on. Only public data is written.

**If the PI names ITRI's CA** (its key is not here): ITRI issues one signer
certificate (a leaf) for each lab public key. Get the CA certificate and the
two leaves as PEM files, then:

```bash
python -m guardrail.bundle register-cert --authority itri-ca --ca-cert ITRI_CA.pem \
    --cert LEAF_DESKTOP.pem --identity lab-dev:ed25519:6ec75b535ef5c533
python -m guardrail.bundle register-cert --authority itri-ca \
    --cert LEAF_AGX.pem --identity lab-dev:ed25519:715f70c4647a9701
```

The first call lists the authority; the second finds it listed. A leaf that
certifies another key than the one named is refused.

## 2. Re-sign the flight bundles on the desktop

Lab CA:

```bash
python tools/build_flight_bundles.py --issue-cert
```

(`--pki DIR` if the CA's folder is not the default one.) ITRI CA, after
step 1:

```bash
python tools/build_flight_bundles.py
```

If a key is certified by more than one authority, the tool refuses to pick
one by file order: add `--authority <name>` (or set
`VLAGUARD_SIGNING_AUTHORITY`). The policies are unchanged, so each bundle's
`policy_hash` must be the one it had before (compare with the records in
docs/data/bundle_ca_20261126/); the signer, the shipped certificate and the
signature change.

Then commit the trust store change (and nothing else) on the branch:

```bash
git add policies/keys/trusted_signers.json
git commit -m "WP1-04: bundle keys certified by <authority> (PI decision of <date>)"
```

## 3. Check on the desktop

```bash
python tools/build_flight_bundles.py --check
python -m guardrail.bundle verify bundles/fase3-sim-demo-v0.1.4.tar.gz --require-chain
```

`--check` must print OK for every bundle of its default list (12/12);
`verify --require-chain` exits 1 unless the signer's certificate chained to a
listed authority on this machine. Repeat `verify --require-chain` for each
bundle a campaign flies.

## 4. Bring the commit to the AGX Orin and rebuild there

Bring the branch's commit to `~/vla-drone` the way every campaign does
(docs/RUNBOOK-orin-hil.md, step 7: same commit on both hosts, `git status
--porcelain` empty). Then, on the Orin, in `~/vla-drone`, rebuild WITHOUT
`--issue-cert` (the CA's key is not there; its own key is now pinned under
the new identity in the trust store it just received):

```bash
~/vla-drone-env/bin/python tools/build_flight_bundles.py
~/vla-drone-env/bin/python tools/build_flight_bundles.py --check
~/vla-drone-env/bin/python -m guardrail.bundle verify \
    bundles/fase3-sim-demo-v0.1.4.tar.gz --require-chain
```

The Orin's bundles are signed by its own key under the new identity; the
desktop's by the desktop's. Both must pass `--check` and `verify
--require-chain`.

## 5. Require the chain on `hil`

In `deploy/topologies/hil.env` set

```
VLAGUARD_REQUIRE_CHAIN=1
```

and rewrite its comment (the "0 until the bundle keys are certified"
reason no longer holds). Two tests pin the shipped value to 0 and must be
changed in the same commit, with the reason:
`tests/test_deploy_configs.py` `test_the_chain_requirement_reaches_the_companion_and_ships_switched_off`
(every topology file reads 0) and
`test_the_perception_rail_hil_script_hands_on_the_chain_requirement`
(hil.env reads 0). On 27 Nov `flight.env` was set to 1 as well (the real
aircraft is the deployment a signed bundle is for, and no flight run exists
yet) and `dev.env` kept 0 (development topology); the two tests now pin
dev 0 / hil 1 / flight 1. Run `python tests/test_deploy_configs.py`, commit, and
bring that commit to the Orin as in step 4.

The compose companion reads the value from the topology file
(`${VLAGUARD_REQUIRE_CHAIN:-0}` in deploy/compose/docker-compose.orin.yml),
the PAS perception rail from hil.env through deploy/orin/run_pas_perception.sh,
and the stress harness passes the desktop's `VLAGUARD_REQUIRE_CHAIN` (or a
`--hil-config` file's `require_chain`) to each mission container
(experiments/sweep_scenarios.py, `HilConfig.mission_env`).

## 6. One `hil` verification mission

From the desktop, with both hosts at the step-5 commit and the rig set up as
for any campaign (docs/RUNBOOK-orin-hil.md, step 14):

```bash
python experiments/sweep_scenarios.py --profile smoke --backend hil --hil-dry-run \
    --only nfz-head-on
python experiments/sweep_scenarios.py --profile smoke --backend hil \
    --hil-campaign ca<MMDD> --only nfz-head-on
```

The dry run must list one mission and refuse none for a signer reason. In
the fetched run (`demo/out/hil_smoke/ca<MMDD>_001_nfz-head-on/metrics.json`),
`policy_source` must read `signature` "verified", `chain` "verified",
`require_chain` true and `signed_by` the new CA identity, and the mission
must be KPI-grade like any other. A refusal before take-off for the chain
means a host still has a bundle signed by a bare key: go back to step 3 or 4.

## 7. Commit the evidence

On the desktop, on a clean tree (the record names its code revision), in
the checkout whose `bundles/` holds the re-signed bundles:

```bash
python tools/bundle_ca_evidence.py --drill \
    --out docs/data/bundle_ca_<date>/bundle_ca_desktop.json
```

On the Orin, into a git-ignored folder, then copied back
(`scp nathan-agxiot@10.42.0.3:vla-drone/deploy/evidence/incoming/bundle_ca_agx.json ...`):

```bash
~/vla-drone-env/bin/python tools/bundle_ca_evidence.py \
    --out deploy/evidence/incoming/bundle_ca_agx.json
```

(`--drill` leaves its throw-away CA folders in the system temp folder; on the
Orin, add it only if that is acceptable.) The records must show one
authority, both signers certified with chain `verified` here, and every
flight bundle chain `verified`. Write a README beside them as in
docs/data/bundle_ca_20261126/README.md, naming the verification mission.

Then the report text: WP1-04 and X-10 in Appendix A of both
docs/FINAL-REPORT-outline.md and docs/FINAL-REPORT.md, decision 1 in
section 9, a CHANGELOG entry and the tracker entry (tools/add_update.py).
Whether the rows move to DONE is for an independent review against the
grant's words, not for this runbook.

## Undo

Revert the step-1/2 and step-5 commits on both hosts and rebuild the bundles
(`python tools/build_flight_bundles.py` on each). A bare identity retired by
`issue-cert` or `register-cert` stays accepted only for bundles issued before
its `retired_on` (tests/test_bundle.py,
`test_a_retired_bare_key_is_accepted_only_for_what_was_issued_before`);
`--keep-legacy` on either command does not retire it.

## What ran on 27 Nov 2026 (Lab CA)

- Step 1-2 on the desktop: `python tools/build_flight_bundles.py --issue-cert`
  (authority `lab-ca`, CA sha256 `71d1e071d3985bc6...`, valid to 2036-10-30;
  leaves for both keys; the bare `lab-dev` identities retired at
  2026-11-27T00:10:37Z), then the bundles the hil profiles and the perception
  rail fly that are not in the tool's default list:
  `policies/sitl_pedestrian.yaml`,
  `experiments/policies/follow_car_citylife_people.yaml`, and the three
  `experiments/policies/msl_dem_*.yaml` with
  `--ground experiments/worlds/dem_cmac_wgs84.json` (section 11a of
  docs/RUNBOOK-orin-hil.md). 17 flight bundles in all; which policies the hil
  profiles fly was read from a dry run of each (smoke, nightly, fsm_edges,
  distance_envelope, msl_dem, grant_example, prefix_ab, prefix_dyn).
- Every bundle was copied first: `<store>/bundles_pre_ca_20261127` on the
  desktop and `~/bundles_pre_ca_20261127` on the AGX, each with SHA256SUMS.
  In all 17 the policy_hash, policy_id, version and IR are unchanged; the
  signer, its certificate and issued_at changed.
- Step 4 on the AGX at the same commit, the same 17 bundles signed by its own
  key under `lab-ca:71d1e071d3985bc6:ed25519:715f70c4647a9701`.
- Step 3/4 checks: 17/17 pass `verify --require-chain` on each host; a bundle
  still signed by a bare key fails it (rc 1); the 8 replay bundles still
  verify.
- Step 5: hil.env and flight.env 1, dev.env 0.
- Step 6-7: docs/data/bundle_ca_20261127/README.md.

## Later: moving to ITRI's CA

No code changes. ITRI receives the two PUBLIC keys
(`public_key_hex` of the two `lab-ca:...` entries in
policies/keys/trusted_signers.json) and returns its CA certificate and one
codeSigning leaf per key, as PEM. Then, on a branch, with nothing flying:

1. Register both leaves (section 1 above, "If the PI names ITRI's CA"). Each
   key is then certified twice (lab-ca and itri-ca), and the tools refuse to
   choose between them by file order.
2. Re-sign the same bundle list as on 27 Nov on both hosts with
   `--authority itri-ca` (or `VLAGUARD_SIGNING_AUTHORITY=itri-ca`), then
   step 3-4's checks.
3. Stop the Lab CA from vouching for new bundles while keeping what it signed
   verifiable: give each of the two `lab-ca:...` signer entries a
   `retired_on` (the instant of the re-sign) and a `superseded_by` (its
   `itri-ca:...` identity). guardrail.bundle applies `retired_on` to any
   signer entry: a lab-ca bundle issued before that instant still verifies,
   one issued after is refused, and the key signs only as itri-ca from then
   on. Do NOT remove the `lab-ca` authority: every run and bundle signed
   between 27 Nov and the move would then be refused on replay
   (tests/test_bundle.py, test_swapping_the_authority_is_one_registry_entry_and_a_re_sign).
4. Step 6-7 again (one verification mission, evidence), and the report rows
   WP1-04 / X-10.

`retired_on` compares against the bundle's own `issued_at`, which the signer
writes: it is a naming rule for honest tools, not a cryptographic revocation.
A key whose private half may have leaked is handled by removing its signer
entries and re-keying, not by retirement.

## The Lab CA's private key

`ca.key.pem` in the lab PKI folder (`%LOCALAPPDATA%\vla-guardrail\harness_pki\`
on the lab desktop) is not encrypted, and the same root also issues the
harness's mutual-TLS certificates (the grant's Stress Testing page reuses one
root CA across the control plane). Whoever holds that file can certify new
bundle signers. At hand-over the lab should move it to storage it controls
(offline media or a password-protected keystore) and keep only the public
`ca.cert.pem` on the desktop; nothing in the flight path needs the CA's
private key, only `issue-cert` does.
