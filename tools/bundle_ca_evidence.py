"""Who signs the policy bundles, as THIS machine sees it: the evidence record
for WP1-04 / X-10 (the signer under a certificate authority).

    python tools/bundle_ca_evidence.py                          # print; write nothing
    python tools/bundle_ca_evidence.py --out docs/data/bundle_ca_<date>_<host>.json
    python tools/bundle_ca_evidence.py --drill --out ...        # also run the swap drill

WHAT IT RECORDS

  machine      host, platform, Python, the `cryptography` version (or none),
               which Ed25519 verifier runs here, the code revision
  authorities  each certificate authority the trust store lists: name, the
               SHA-256 of its CA certificate (compare it with
               `python -m guardrail.harness_service gen-pki`'s fingerprint, or
               `openssl x509 -fingerprint -sha256`), subject, notAfter
  signers      each identity the store pins: its key id, issuer, certificate
               digest and serial, whether it is retired and since when, and
               what THIS machine establishes about it (guardrail.bundle.
               signer_standing): accepted or not, and the chain status - "verified"
               only where `cryptography` chained the certificate to its
               authority here, "not-checked-here" where it is absent
  bundles      every bundle in bundles/: for a policy bundle its identity,
               signer, signature status and chain status (check_bundle); for
               a replay bundle its signer, signature status and chain status
               (read_replay); a retired identity accepted for either is named
               (`retired_on`, a policy bundle's in `signer_retired_on`)
  drill        with --drill: the CA tests of tests/test_bundle.py run in this
               process on throw-away CAs (the swap of authority, the refused
               certificates, the retired key, the no-cryptography record),
               each PASS, SKIP or FAIL. Without --drill: not run, and said so.

It reads public data only (the trust store, the bundles) and never a private
key; it issues nothing and signs nothing. Nothing here is a result until it
has been run on the machine it describes: the record names its host.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import platform
import sys
import tarfile
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail import bundle as B                                     # noqa: E402

# The tests the drill runs, in tests/test_bundle.py.
DRILL_TESTS = (
    "test_a_key_under_a_ca_signs_as_a_ca_reference_and_its_chain_verifies",
    "test_the_manifest_carries_the_five_grant_fields_with_a_ca_reference",
    "test_swapping_the_authority_is_one_registry_entry_and_a_re_sign",
    "test_moving_to_another_ca_retires_the_old_ca_identity_and_keeps_its_bundles",
    "test_a_certificate_that_does_not_chain_to_a_listed_authority_is_refused",
    "test_a_retired_bare_key_is_accepted_only_for_what_was_issued_before",
    "test_without_cryptography_the_chain_is_recorded_as_not_checked_never_as_verified",
    "test_a_ca_reference_without_its_signer_record_is_refused",
    "test_an_authority_vouches_only_for_pinned_keys_unless_its_entry_says_otherwise",
    "test_a_key_certified_twice_signs_under_the_authority_named_never_by_file_order",
)


def _shown(p: Path) -> str:
    p = Path(p).resolve()
    return p.relative_to(ROOT).as_posix() if p.is_relative_to(ROOT) else p.as_posix()


def machine() -> dict:
    try:
        import cryptography
        crypto = cryptography.__version__ if B.HAVE_CRYPTO else None
    except ImportError:
        crypto = None
    try:
        from guardrail.manifest import code_revision
        rev = code_revision()
    except Exception as exc:                                  # noqa: BLE001
        rev = f"unknown ({type(exc).__name__})"
    return {"host": platform.node(), "platform": platform.platform(),
            "machine": platform.machine(), "python": platform.python_version(),
            "cryptography": crypto, "ed25519_verifier": B.verifier_name(),
            "code_revision": rev}


def authorities(trust_store=None) -> list[dict]:
    rows = []
    for name, ent in sorted(B.load_authorities(trust_store).items()):
        row = {"name": name, "recorded_sha256": ent.get("sha256"),
               "subject": ent.get("subject"), "not_after": ent.get("not_after"),
               "registered": ent.get("registered")}
        try:
            row["sha256"] = B.certificate_sha256(ent.get("certificate_pem"))
            row["reference"] = B.ca_reference(name, row["sha256"])
            row["fingerprint_matches_record"] = row["sha256"] == ent.get("sha256")
        except ValueError as exc:
            row["sha256"], row["problem"] = None, str(exc)
        rows.append(row)
    return rows


def signers(trust_store=None) -> list[dict]:
    rows = []
    for ident, ent in sorted(B.load_trust_store(trust_store).items()):
        # No document at hand, so no issue time: a retired identity is in
        # standing here for what was issued before `retired_on` (the row
        # carries the date) and for nothing issued since.
        st = B.signer_standing(ident, trust_store)
        row = {"identity": ident, "key_id": B._split_identity(ident)[2],
               "issuer": ent.get("issuer"),
               "certificate_sha256": ent.get("certificate_sha256"),
               "leaf_serial": ent.get("leaf_serial"),
               "leaf_subject": ent.get("leaf_subject"),
               "not_after": ent.get("not_after"),
               "retired_on": ent.get("retired_on"),
               "superseded_by": ent.get("superseded_by"),
               "accepted_here": st.ok}
        if st.ok:
            row["chain"] = st.chain
        else:
            row["refused_as"], row["why"] = B.SIG_UNTRUSTED, st.detail
        rows.append(row)
    return rows


def bundles(bundles_dir: Path, trust_store=None) -> dict:
    policy, replay, counts = [], [], Counter()
    for p in sorted(Path(bundles_dir).glob("*.tar.gz")):
        if p.name.endswith(".replay.tar.gz"):
            try:
                from guardrail.replay import read_replay
                sig = read_replay(p, trust_store)["signature"]
                row = {"file": p.name, "signed_by": sig["signed_by"],
                       "signature": sig["status"], "chain": sig["chain"],
                       "authority": sig["authority"], "retired_on": sig["retired_on"],
                       "detail": sig["detail"]}
            except (ValueError, KeyError, OSError, tarfile.TarError) as exc:
                row = {"file": p.name, "signature": "REFUSED", "chain": "none",
                       "detail": str(exc)}
            replay.append(row)
            counts[f"replay / {row['signature']} / chain {row['chain']}"] += 1
            continue
        try:
            chk = B.check_bundle(p, trust_store)
            row = {"file": p.name, "policy_id": chk.policy.policy_id,
                   "version": chk.policy.version,
                   "policy_hash": chk.manifest.get("policy_hash"),
                   "issued_at": chk.manifest.get("issued_at"),
                   "signed_by": chk.signed_by, "signature": chk.signature,
                   "chain": chk.chain, "authority": chk.authority,
                   "signer_retired_on": chk.retired_on,
                   "signer": chk.manifest.get("signer"), "detail": chk.detail}
        except (ValueError, OSError, tarfile.TarError) as exc:
            row = {"file": p.name, "signature": "REFUSED", "chain": "none",
                   "detail": str(exc)}
        policy.append(row)
        counts[f"policy / {row['signature']} / chain {row['chain']}"] += 1
    return {"dir": _shown(bundles_dir), "counts": dict(sorted(counts.items())),
            "policy": policy, "replay": replay}


def drill() -> dict:
    """The CA tests of tests/test_bundle.py, run here on throw-away CAs.

    tests/ goes on sys.path while the module loads: test_bundle.py imports
    its sibling helpers (wgs84_fixture since 26 Nov, 909804b) as a test run
    from tests/ finds them, and without it `--drill` died on the import."""
    tests_dir = str(ROOT / "tests")
    path = ROOT / "tests" / "test_bundle.py"
    spec = importlib.util.spec_from_file_location("_bundle_ca_drill", path)
    mod = importlib.util.module_from_spec(spec)
    added = tests_dir not in sys.path
    if added:
        sys.path.insert(0, tests_dir)
    try:
        spec.loader.exec_module(mod)
    finally:
        if added:
            sys.path.remove(tests_dir)
    out = {}
    for name in DRILL_TESTS:
        try:
            out[name] = "SKIP (needs cryptography)" if getattr(mod, name)() == mod.SKIP \
                else "PASS"
        except Exception as exc:                              # noqa: BLE001
            out[name] = f"FAIL {type(exc).__name__}: {str(exc)[:300]}"
    return out


def collect(trust_store=None, bundles_dir: Path | None = None, *,
            run_drill: bool = False) -> dict:
    store = Path(trust_store) if trust_store else B.trust_store_path()
    rec = {
        "what": "policy-bundle signing authority as this machine sees it "
                "(WP1-04 / X-10); tools/bundle_ca_evidence.py",
        "generated_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "machine": machine(),
        "trust_store": _shown(store),
        "authorities": authorities(store),
        "signers": signers(store),
        "bundles": bundles(bundles_dir or ROOT / "bundles", store),
        "drill": drill() if run_drill else "not run (pass --drill)",
    }
    certified = [s for s in rec["signers"] if s["issuer"]]
    rec["summary"] = {
        "authorities": len(rec["authorities"]),
        "certified_signers": len(certified),
        "certified_signers_chain_verified_here":
            sum(1 for s in certified if s.get("chain") == B.CHAIN_VERIFIED),
        "bare_signers_in_force": sum(1 for s in rec["signers"]
                                     if not s["issuer"] and not s["retired_on"]),
        "bare_signers_retired": sum(1 for s in rec["signers"]
                                    if not s["issuer"] and s["retired_on"]),
        "bundles": rec["bundles"]["counts"],
    }
    return rec


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--trust", default=None, help="trust store (default: the repository's)")
    ap.add_argument("--bundles", default=None, help="bundles folder (default: bundles/)")
    ap.add_argument("--out", default=None, help="write the record here (JSON)")
    ap.add_argument("--drill", action="store_true",
                    help="also run the CA tests of tests/test_bundle.py here")
    a = ap.parse_args(argv)
    rec = collect(a.trust, Path(a.bundles) if a.bundles else None, run_drill=a.drill)
    m, s = rec["machine"], rec["summary"]
    print(f"{m['host']}  python {m['python']}  cryptography {m['cryptography']}  "
          f"verifier {m['ed25519_verifier']}  code {m['code_revision']}")
    for au in rec["authorities"]:
        print(f"authority  {au['name']}  sha256 {au['sha256']}  {au['subject']}  "
              f"to {au['not_after']}")
    if not rec["authorities"]:
        print("authority  none listed: every signer is a bare key")
    for sg in rec["signers"]:
        state = (f"chain {sg['chain']}" if sg["accepted_here"]
                 else f"{sg['refused_as']}: {sg['why']}")
        print(f"signer     {sg['identity']}  "
              + (f"retired {sg['retired_on']}  " if sg["retired_on"] else "") + state)
    for k, v in s["bundles"].items():
        print(f"bundles    {v:3d}  {k}")
    if a.drill:
        for name, res in rec["drill"].items():
            print(f"drill      {res.split(' ')[0]:4s}  {name}")
    if a.out:
        out = Path(a.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(rec, indent=2) + "\n", encoding="utf-8", newline="\n")
        print(f"wrote {out}")
    failed = a.drill and any(v.startswith("FAIL") for v in rec["drill"].values())
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
