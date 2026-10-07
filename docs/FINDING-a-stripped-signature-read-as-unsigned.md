# A stripped signature read as "unsigned"

**Date:** 2026-10-06
**Found by:** the adversarial review of the signed-bundle work, the same day the
real Ed25519 signature replaced the placeholder.
**Status:** fixed in `guardrail/bundle.py` (`verify_signature`), with
regression tests in `tests/test_bundle.py`.

## What it was for

A policy bundle is `ir.json` + `manifest.json` + `signature.txt`. Since 6 Oct the
second line of `signature.txt` is an Ed25519 signature over the manifest, and
the manifest carries the SHA-256 of the IR, so one signature covers the whole
chain. Every reader reports exactly one status: `verified`, `unsigned`,
`untrusted-signer`, `unverifiable-here` or `bad-signature`. A flight refuses
anything but `verified` unless it is started with `--allow-unverified-bundle`,
and then it records the status it flew with.

## What went wrong

Two layouts are honestly unsigned: the reference's placeholder signer, and what
`write_bundle` writes on a machine with no key. Both name no key.

The first version decided "unsigned" by the **absence of the signature line**,
not by the signer's identity. So a forger could take a bundle signed by the lab
key, edit the IR (a fence ceiling lowered to 5 m), recompute the hash, keep the
trusted signer's name in the manifest, and delete only the Ed25519 line. The
reader returned:

- status `unsigned`, with the **trusted** signer's name attached and the detail
  "names no key", which was false;
- and `load_for_flight(..., allow_unverified=True)` flew it.

A second path had the same effect. On an interpreter without the
`cryptography` package (the 3.11 environment, the WSL flight venvs) a forged
signature read `unverifiable-here`, exactly like a genuine one, and also flew
under the flag.

Neither produced an error. Both looked like the system handling an ordinary,
expected case.

## The fix

- `unsigned` is reserved for the two identities that name no key. A bundle
  whose signer names a key but which carries no signature is
  `bad-signature`: that is what stripping looks like, and no tool here writes
  it. Every reader refuses `bad-signature`, with or without the flag.
- The mirror image, an Ed25519 line under a keyless identity, is refused the
  same way.
- The trust store is consulted before any cryptography, so an unknown signer
  is `untrusted-signer` in every environment.
- Verification no longer depends on `cryptography`: a pure-Python RFC 8032
  verifier, pinned to the RFC's own test vectors, runs where the library is
  missing. A forged signature is now refused there and a genuine one verifies.

Regression tests: `test_a_stripped_signature_that_keeps_its_signer_is_refused`
(the review's forgery, verbatim) and
`test_a_signature_under_a_keyless_identity_is_refused`. On the pre-fix code the
first returned `unsigned` with the trusted identity and `load_for_flight` flew
the bundle.

## The general lesson

A status is a claim about the artefact. "Unsigned" must mean "nobody claims to
have signed this", not "we did not find a signature". When the two are allowed
to read the same, the weakest status becomes a way around the strongest check.
