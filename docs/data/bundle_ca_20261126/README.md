# Who signs the policy bundles: the PRE-CA state, 26 Nov 2026 (WP1-04, X-10)

These two records are the state BEFORE any certificate authority signs
anything. The grant's Policy DSL page asks for `signed_by` to be a "Lab CA or
ITRI CA reference"; which CA is the PI's call (12 Oct, Q5), and on 22 Nov the
user decided to wait for the PI to name it. So no key is certified here and
no bundle is signed under a CA. Nothing in this folder invents one.

`tools/bundle_ca_evidence.py` reads public data only (the trust store and the
bundles); it issues nothing and signs nothing. Its `--drill` runs the CA
tests of tests/test_bundle.py in the same process, on throw-away CAs it makes
in the system temp folder.

## The records

| File | Host | How | Code |
|---|---|---|---|
| `bundle_ca_desktop.json` | the lab desktop (Windows, Python 3.10.20, cryptography 49.0.0) | `python tools/bundle_ca_evidence.py --drill --out docs/data/bundle_ca_20261126/bundle_ca_desktop.json`, 26 Nov 02:15 UTC | 3a92819 (clean; the trust store, guardrail/bundle.py, guardrail/replay.py, tests/test_bundle.py and the tool unchanged since release 0.7.0) |
| `bundle_ca_agx.json` | the AGX Orin (JetPack 7.2.1, aarch64, Python 3.12.3 in `~/vla-drone-env`, cryptography 49.0.0) | the tool's own `collect()` run read-only in the campaign checkout `~/vla-drone` and its JSON printed to stdout (nothing written there), WITHOUT `--drill`, 26 Nov 01:59 UTC | 21c2771 (release 0.7.0, clean) |

The desktop record reads the 48 policy bundles of the lab checkout's
`bundles/` (copied byte for byte into the worktree's git-ignored `bundles/`
for the run) and the 8 tracked replay bundles.

Why no drill on the AGX: tests/test_bundle.py makes its throw-away CAs with
`tempfile.mkdtemp()` and does not remove them, so the drill would leave files
outside its output on the rig. The drill is host-independent Python
(`cryptography` 49.0.0 on both hosts); the AGX record shows what that host
sees in its own trust store and bundles, which is the part that differs.

## What they show (both hosts the same)

- **Authorities: none.** `policies/keys/trusted_signers.json` lists no
  `authorities` entry.
- **Signers: two bare lab keys,** `lab-dev:ed25519:6ec75b535ef5c533` (the
  desktop's) and `lab-dev:ed25519:715f70c4647a9701` (the AGX Orin's), both
  accepted, chain `none`, none retired.
- **Policy bundles: 48, all `verified`, all chain `none`.** On the desktop all
  48 are signed by the desktop key. On the AGX, 36 by the desktop key and 12
  by its own, which it built itself: corridor-survey 0.1.3, fase3-sim-demo
  0.1.4, follow-car 0.1.3, follow-car-nfz 0.1.3, follow-car-citylife 0.1.3,
  follow-car-citylife-nfz / -nfz3 / -nfz3b 0.1.3, follow-pedestrian 0.1.3,
  sitl-pedestrian-standoff 0.1.3, vla-goto-citylife 0.1.3 and
  wgs84-taipei-demo 0.1.2.
- **Replay bundles: 8.** 4 verified (aiot_v10bs_01-03 by the desktop key,
  hil_smoke_03 by the AGX key), 4 unsigned legacy replays
  (retarget_*, `signed_by` "lab-ca:dev-placeholder", a placeholder label
  from before signing, not a CA).
- **Drill (desktop): 9 of 9 PASS** - a key under a CA signs as a CA
  reference and its chain verifies; the manifest carries the five grant
  fields with a CA reference; swapping the authority is one registry entry
  and a re-sign; a certificate that does not chain to a listed authority is
  refused; a retired bare key is accepted only for what was issued before;
  without `cryptography` the chain is recorded as not checked, never as
  verified; a CA reference without its signer record is refused; an
  authority vouches only for pinned keys; a key certified twice signs under
  the authority named, never by file order.

## What this does not show

No CA has signed a flight bundle and no flight has required a chain
(`VLAGUARD_REQUIRE_CHAIN=0` in deploy/topologies/hil.env). The steps that
apply the PI's answer in one session are in docs/RUNBOOK-bundle-ca.md; its
last step writes the post-CA records next to these.
