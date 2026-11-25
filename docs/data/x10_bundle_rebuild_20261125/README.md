# X-10 item (2): the flight bundles rebuilt with a changelog diff (25 Nov 2026)

App. A row X-10 asks for a bundle changelog that is a diff against the
previous version. The builder has written one since c928a23 (23 Nov, in
release 0.7.0 = 21c2771): `tools/build_flight_bundles.py` passes the policy's
previous published bundle to `write_bundle(previous=...)` and `--check` flags
a bundle whose manifest has no `changelog_diff`. The bundles in `bundles/`
(a gitignored build product) were built before that, so none of them carried
one. This record is the rebuild.

What was done, on both hosts, at release 0.7.0 (21c2771):

- Desktop (lab checkout, key lab-dev:ed25519:6ec75b535ef5c533): the 12
  default policies and sitl_pedestrian rebuilt one at a time with each
  delivered bundle's own `issued_at` (`--issued-at`), so only the changelog
  fields change. `desktop_rebuild_report.json` compares every rebuilt
  manifest with the delivered one, key by key: 13/13 rc 0; policy_hash,
  `hashes` and the ir.json bytes unchanged on 13/13; the manifest keys that
  changed are `changelog` and `changelog_diff` only; 12/13 carry
  `changelog_diff` (distance-envelope-sitl v0.1.0 is a first version and says
  so). `desktop_check_before.txt` / `desktop_check_after.txt`: `--check`
  11/12 BAD before, 12/12 OK after.
- AGX Orin (`~/vla-drone`, its own key lab-dev:ed25519:715f70c4647a9701, the
  copies the hil rail loads): the same rebuild with `--issued-at` from each
  bundle's manifest. `agx_check_after.txt`: `--check` 12/12 OK, and for the
  four bundles ntFIN5 and smREL070 flew, the file sha256 after and before,
  the unchanged policy_hash, issued_at and signer, and `changelog_diff`
  present.
- The bundles as delivered before the rebuild are kept: on the desktop in
  `F:/vla_drone_offload/bundles_pre_x10b_20261125/` (56 files), on the AGX in
  `~/bundles_pre_x10b_20261125/` (56 files). Nothing was deleted.

Flown: bat1125_001 (hil, 25 Nov, 21:30 +0800, WP3-07 battery cell) loaded
the rebuilt `bundles/fase3-sim-demo-v0.1.4.tar.gz` on the AGX (file sha256
526b2f634d36..., replaced at 21:28:54; mission_console.log: "from bundle
bundles/fase3-sim-demo-v0.1.4.tar.gz - signature verified", policy_hash
b665adba...), whose changelog is "against fase3-sim-demo 0.1.3
(sha256:1a777); changed alt-band (scope); changed kin-caps (scope); changed
nfz-square (scope); policy version".

Not done here: `signed_by` is still a lab-dev key, not a Lab or ITRI CA
reference (X-10 item 1; the CA is the PI's to name, and the user decided on
22 Nov to wait). Re-signing under the CA will rebuild these bundles again.
