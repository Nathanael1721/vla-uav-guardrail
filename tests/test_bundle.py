"""The signed policy bundle, the WGS84 loader, and the Constraint Summary Pack.

Run either way:
    pytest tests/test_bundle.py -v
    python tests/test_bundle.py

WHY THIS FILE EXISTS

Three WP1/WP2 deliverables that the grant names and this repository had never
produced: a signed policy bundle, WGS84 as the authoring frame, and the CSP.

The bundle tests are mostly about REFUSAL. A round-trip test alone would pass on
a bundle that verifies nothing at all - write, read back, hashes agree, because
nothing in between ever checked. So each test below breaks the bundle in a
different way and requires the loader to notice.

Since 2026-10-06 the signature is a real Ed25519 signature, so the refusals now
include the ones a placeholder could never make: an edited manifest, a
consistent forgery by someone without the key, an untrusted signer, and a
signature stripped from a bundle that still names its key. Signature tests use
a throw-away key and trust store, never the lab key. Where `cryptography` is
missing (the 3.11 env, the WSL flight venvs) the key is the pure-Python RFC
8032 test key and verification is the pure-Python fallback - so these tests
RUN there, which is the point: those are the environments the fallback is for.
The fallback is pinned to the RFC's own test vectors on every interpreter.

The projection tests are mostly about the axis order. The reference
implementation returns (east, north); this project's frame is x = North,
y = East. A straight copy of its call would rotate every constraint ninety
degrees with nothing in the schema to object.
"""
import hashlib
import io
import json
import os
import subprocess
import sys
import tarfile
import tempfile
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import guardrail.bundle as B                                       # noqa: E402
from guardrail import load_policy                                  # noqa: E402
from guardrail.bundle import (IR_NAME, MANIFEST_NAME,              # noqa: E402
                              SIGNATURE_NAME, BundleSignatureError,
                              check_bundle, load_bundle, write_bundle)
from guardrail.compiler import ConstraintCompiler                  # noqa: E402
from guardrail.models import State                                 # noqa: E402
from guardrail.projection import LocalProjection, project_raw      # noqa: E402
from guardrail.shield import Shield                                # noqa: E402

DEMO = ROOT / "policies" / "sim_demo_policy.yaml"
WGS84 = ROOT / "policies" / "wgs84_taipei.yaml"
PED = ROOT / "policies" / "sitl_pedestrian.yaml"
ISSUED = "2026-10-06T00:00:00Z"
SKIP = "SKIP"


def _tmp(name="p.tar.gz"):
    return Path(tempfile.mkdtemp()) / name


def _repack(src: Path, dst: Path, edits: dict):
    """Rewrite named members of a bundle, leaving the rest alone."""
    members = {}
    with tarfile.open(src, "r:gz") as tar:
        for m in tar.getmembers():
            members[m.name] = tar.extractfile(m).read()
    members.update(edits)
    with tarfile.open(dst, "w:gz") as tar:
        for name, data in members.items():
            info = tarfile.TarInfo(name=name)
            info.size = len(data)
            info.mtime = 0
            tar.addfile(info, io.BytesIO(data))
    return dst


def _members(path: Path) -> dict:
    with tarfile.open(path, "r:gz") as tar:
        return {m.name: tar.extractfile(m).read() for m in tar.getmembers()}


def _unsigned(pol, name="p.tar.gz", **kw):
    """An integrity-only bundle: written UNSIGNED on purpose."""
    return write_bundle(pol, _tmp(name), signer=None, issued_at=ISSUED, **kw)


def _test_key(kind: str = "auto"):
    """A throw-away signer and a trust store that trusts only it.

    `cryptography` makes the key where it is installed; elsewhere (or with
    kind="py") the pure-Python RFC 8032 test key does, so every signature test
    runs on the interpreters that lack `cryptography` instead of skipping
    there."""
    d = Path(tempfile.mkdtemp())
    trust = d / "trust.json"
    if B.HAVE_CRYPTO and kind != "py":
        signer = B.generate_signing_key(d / "k.pem", authority="test-lab",
                                        trust_store=trust)
    else:
        signer = B.Signer(authority="test-lab",
                          private_key=B._PyEd25519TestKey(os.urandom(32)))
        B.register_public_key(signer, trust)
    return signer, trust


class _no_crypto:
    """Run a block as an interpreter without `cryptography` would; with
    fallback=False, as one with no verifier at all."""

    def __init__(self, fallback: bool = True):
        self.fallback = fallback

    def __enter__(self):
        self.saved = (B.HAVE_CRYPTO, B.PY_VERIFY_FALLBACK)
        B.HAVE_CRYPTO, B.PY_VERIFY_FALLBACK = False, self.fallback

    def __exit__(self, *exc):
        B.HAVE_CRYPTO, B.PY_VERIFY_FALLBACK = self.saved
        return False


def _strip_signature(signed: Path, dst: Path, forge_ir: bool = True) -> Path:
    """The 2026-10-06 review's forgery: edit the IR, recompute the hash, keep
    the trusted signer's identity, and drop only the Ed25519 line."""
    m = _members(signed)
    ir_b, man = m[IR_NAME], json.loads(m[MANIFEST_NAME])
    if forge_ir:
        ir = json.loads(ir_b)
        ir["constraints"][0]["altitude_ceiling_m"] = 5.0
        ir_b = json.dumps(ir, sort_keys=True, separators=(",", ":"),
                          ensure_ascii=True).encode()
        man["policy_hash"] = "sha256:" + hashlib.sha256(ir_b).hexdigest()
    head = m[SIGNATURE_NAME].decode().splitlines()[0].split(" ", 1)[1]
    return _repack(signed, dst, {
        IR_NAME: ir_b,
        MANIFEST_NAME: json.dumps(man, indent=2, sort_keys=True).encode(),
        SIGNATURE_NAME: f"{man['policy_hash']} {head}\n".encode()})


# --------------------------------------------------------------------------- #
# bundle: round trip
# --------------------------------------------------------------------------- #

def test_a_bundle_round_trips_to_an_identical_policy():
    pol = load_policy(DEMO)
    got = load_bundle(_unsigned(pol), require_signature=False)
    assert got.policy_hash == pol.policy_hash
    assert got.policy_id == pol.policy_id
    assert got.canonical_ir() == pol.canonical_ir()
    assert len(got.constraints) == len(pol.constraints)


def test_the_bundle_carries_the_three_named_members():
    """The reference implementation's container layout. The container alone
    did not make the bundles exchangeable (the IRs differ); the cross-load is
    tested in tests/test_policy_dsl_grant_form.py."""
    with tarfile.open(_unsigned(load_policy(DEMO)), "r:gz") as tar:
        names = sorted(tar.getnames())
    assert names == ["ir.json", "manifest.json", "signature.txt"], names


def test_ir_json_is_exactly_the_bytes_the_hash_digests():
    """WP3-21: hash the stored IR bytes, not a re-dump. A reader can now check
    a bundle with sha256sum alone, and a schema change in this code can never
    make an old bundle's own bytes disagree with its own manifest."""
    pol = load_policy(DEMO)
    m = _members(_unsigned(pol))
    digest = "sha256:" + hashlib.sha256(m[IR_NAME]).hexdigest()
    man = json.loads(m[MANIFEST_NAME])
    assert digest == man["policy_hash"] == pol.policy_hash, (digest, man)
    assert len(pol.policy_hash) == len("sha256:") + 64, pol.policy_hash


def test_the_same_policy_produces_byte_identical_bundles():
    """Member mtimes are pinned to 0 so a bundle is reproducible.

    Without that, two builds of an unchanged policy differ, and a hash of the
    ARCHIVE stops being a statement about the policy. Since issued_at entered
    the manifest, "the same policy" means the same policy at the same issue
    time; ir.json never depends on it (next test).
    """
    pol = load_policy(DEMO)
    a = write_bundle(pol, _tmp("a.tar.gz"), signer=None, issued_at=ISSUED).read_bytes()
    b = write_bundle(pol, _tmp("b.tar.gz"), signer=None, issued_at=ISSUED).read_bytes()
    assert a == b


def test_a_later_issue_changes_the_manifest_but_never_the_policy_hash():
    pol = load_policy(DEMO)
    a = _members(write_bundle(pol, _tmp(), signer=None, issued_at="2026-10-06T00:00:00Z"))
    b = _members(write_bundle(pol, _tmp(), signer=None, issued_at="2026-11-30T12:00:00Z"))
    assert a[IR_NAME] == b[IR_NAME]
    ma, mb = json.loads(a[MANIFEST_NAME]), json.loads(b[MANIFEST_NAME])
    assert ma["policy_hash"] == mb["policy_hash"]
    assert (ma["issued_at"], mb["issued_at"]) == ("2026-10-06T00:00:00Z",
                                                  "2026-11-30T12:00:00Z")


def test_the_manifest_records_what_a_reviewer_needs():
    p = write_bundle(load_policy(DEMO), _tmp(), changelog="added school curfew",
                     signer=None, issued_at=ISSUED)
    man = json.loads(_members(p)[MANIFEST_NAME])
    for f in ("policy_id", "version", "generation", "policy_hash",
              "signed_by", "changelog", "issued_at", "hash_scheme",
              "ir_schema_version"):
        assert f in man, f"manifest missing {f}"
    assert man["changelog"] == "added school curfew"
    assert man["issued_at"] == ISSUED
    assert man["signed_by"] == B.UNSIGNED, (
        "an unsigned bundle must say so in the manifest, not carry a "
        "placeholder that reads like a signer")


# --------------------------------------------------------------------------- #
# bundle: refusal (integrity - no key involved)
# --------------------------------------------------------------------------- #

def test_a_tampered_IR_is_refused():
    """The test that gives the round-trip its meaning.

    Widen the no-fly zone's altitude ceiling in the shipped IR and leave the
    manifest alone. The bundle still opens, still parses, still describes a
    valid policy - and must be refused, because it is not the policy that was
    issued.
    """
    src = _unsigned(load_policy(DEMO), "ok.tar.gz")
    ir = json.loads(_members(src)[IR_NAME])
    ir["constraints"][0]["altitude_ceiling_m"] = 9999.0
    bad = _repack(src, _tmp("bad.tar.gz"),
                  {IR_NAME: json.dumps(ir, sort_keys=True,
                                       separators=(",", ":")).encode()})
    try:
        load_bundle(bad, require_signature=False)
    except ValueError as e:
        assert "policy_hash mismatch" in str(e), e
        return
    raise AssertionError("a tampered IR was accepted")


def test_a_manifest_from_another_bundle_is_refused():
    """Swap in a manifest whose hash belongs to a different policy.

    Caught by the signature line, which binds a hash to the identity that signed
    it - so the manifest and the signature must agree as well as the IR.
    """
    a = _unsigned(load_policy(DEMO), "a.tar.gz")
    b = _unsigned(load_policy(PED), "b.tar.gz")
    bad = _repack(a, _tmp("mixed.tar.gz"),
                  {MANIFEST_NAME: _members(b)[MANIFEST_NAME]})
    try:
        load_bundle(bad, require_signature=False)
    except ValueError:
        return
    raise AssertionError("a foreign manifest was accepted")


def test_a_truncated_bundle_is_refused():
    src = _unsigned(load_policy(DEMO), "ok.tar.gz")
    dst = _tmp("short.tar.gz")
    with tarfile.open(src, "r:gz") as tin, tarfile.open(dst, "w:gz") as tout:
        for m in tin.getmembers():
            if m.name == MANIFEST_NAME:
                continue
            data = tin.extractfile(m).read()
            info = tarfile.TarInfo(name=m.name)
            info.size, info.mtime = len(data), 0
            tout.addfile(info, io.BytesIO(data))
    try:
        load_bundle(dst, require_signature=False)
    except ValueError as e:
        assert "missing required member" in str(e), e
        return
    raise AssertionError("a bundle missing its manifest was accepted")


def test_an_IR_with_an_unknown_key_is_refused_even_when_its_hash_matches():
    """Pydantic ignores unknown keys. An IR carrying one hashes correctly - the
    bytes are the bytes - and then rebuilds into a DIFFERENT policy than the
    one those bytes describe. Before 2026-10-06 the loader only compared the
    rebuilt object's hash, so this case could not even be expressed; now that
    the bytes are hashed directly it must be refused explicitly."""
    pol = load_policy(DEMO)
    src = _unsigned(pol, "ok.tar.gz")
    m = _members(src)
    ir = json.loads(m[IR_NAME])
    ir["constraints"][0]["altitude_cieling_m"] = 40.0          # sic
    ir_bytes = json.dumps(ir, sort_keys=True, separators=(",", ":")).encode()
    h = "sha256:" + hashlib.sha256(ir_bytes).hexdigest()
    man = json.loads(m[MANIFEST_NAME])
    man["policy_hash"] = h
    bad = _repack(src, _tmp("extra.tar.gz"), {
        IR_NAME: ir_bytes,
        MANIFEST_NAME: json.dumps(man, indent=2, sort_keys=True).encode(),
        SIGNATURE_NAME: f"{h} {B.UNSIGNED}\n".encode()})
    try:
        check_bundle(bad)
    except ValueError as e:
        assert "cannot represent" in str(e), e
        return
    raise AssertionError("an IR the model silently truncates was accepted")


def test_a_re_serialised_IR_is_refused():
    """Same policy, prettier bytes: the artefact is no longer what was hashed."""
    src = _unsigned(load_policy(DEMO), "ok.tar.gz")
    ir = json.loads(_members(src)[IR_NAME])
    bad = _repack(src, _tmp("pretty.tar.gz"),
                  {IR_NAME: json.dumps(ir, indent=2, sort_keys=True).encode()})
    try:
        check_bundle(bad)
    except ValueError as e:
        assert "canonical encoding" in str(e), e
        return
    raise AssertionError("a re-serialised ir.json was accepted")


def test_a_hot_applied_rule_is_carried_into_the_next_bundle():
    """Replayability: a bundle must describe the policy actually in force.

    `hot_apply` deliberately mutates the loaded policy - it appends the rule and
    bumps `generation`, which is what makes every artefact after that instant
    carry a different hash. So a bundle written afterwards has to contain the
    dynamic zone; one that did not would document a flight that never happened.
    """
    from guardrail.models import PolygonFence, XY
    pol = load_policy(DEMO)
    before_hash, before_n = pol.policy_hash, len(pol.constraints)

    sh = Shield(pol, lookahead_s=3.0, dt=0.5)
    sh.hot_apply(PolygonFence(
        id="nfz-dynamic", type="polygon_fence",
        vertices=[XY(x=-25, y=-25), XY(x=-15, y=-25),
                  XY(x=-15, y=-15), XY(x=-25, y=-15)], margin_m=1.0))

    assert pol.policy_hash != before_hash, "the hash must restamp"
    assert pol.generation == 1, pol.generation

    got = load_bundle(_unsigned(pol, changelog="dynamic NFZ at t+8s"),
                      require_signature=False)
    assert len(got.constraints) == before_n + 1
    assert any(c.id == "nfz-dynamic" for c in got.constraints)
    assert got.generation == 1, "the generation counter did not survive"


# --------------------------------------------------------------------------- #
# bundle: the signature (2026-10-06)
# --------------------------------------------------------------------------- #

def test_an_unsigned_bundle_is_refused_by_default_and_says_why():
    """No signature is a fact to record, not a pass. load_bundle refuses it
    unless the caller explicitly accepts an unsigned policy."""
    p = _unsigned(load_policy(DEMO))
    assert check_bundle(p).signature == B.SIG_UNSIGNED
    try:
        load_bundle(p)
    except BundleSignatureError as e:
        assert "unsigned" in str(e), e
    else:
        raise AssertionError("an unsigned bundle loaded as if signed")
    assert load_bundle(p, require_signature=False).policy_id == "fase3-sim-demo"


def test_a_signed_bundle_verifies_against_its_trust_store():
    signer, trust = _test_key()
    pol = load_policy(DEMO)
    p = write_bundle(pol, _tmp(), signer=signer, issued_at=ISSUED)
    chk = check_bundle(p, trust)
    assert chk.signature == B.SIG_VERIFIED, chk.detail
    assert chk.signed_by == signer.identity and signer.identity.startswith("test-lab:ed25519:")
    assert load_bundle(p, trust_store=trust).policy_hash == pol.policy_hash
    sig = _members(p)[SIGNATURE_NAME].decode().splitlines()
    assert sig[0] == f"{pol.policy_hash} {signer.identity}", sig
    assert sig[1].startswith("ed25519:") and len(sig[1]) == len("ed25519:") + 128


def test_signing_is_deterministic_so_signed_bundles_stay_reproducible():
    signer, _ = _test_key()
    pol = load_policy(DEMO)
    a = write_bundle(pol, _tmp("a.tar.gz"), signer=signer, issued_at=ISSUED)
    b = write_bundle(pol, _tmp("b.tar.gz"), signer=signer, issued_at=ISSUED)
    assert a.read_bytes() == b.read_bytes()


def test_an_edited_manifest_breaks_the_signature():
    """The changelog is not covered by the policy hash - only by the signature."""
    signer, trust = _test_key()
    p = write_bundle(load_policy(DEMO), _tmp(), signer=signer, issued_at=ISSUED)
    man = json.loads(_members(p)[MANIFEST_NAME])
    man["changelog"] = "approved by ITRI"
    bad = _repack(p, _tmp("edited.tar.gz"),
                  {MANIFEST_NAME: json.dumps(man, indent=2, sort_keys=True).encode()})
    try:
        check_bundle(bad, trust)
    except ValueError as e:
        assert "Ed25519 verification failed" in str(e), e
        return
    raise AssertionError("an edited manifest kept a valid signature")


def test_a_consistent_forgery_without_the_key_is_refused():
    """What the placeholder could never catch: edit the IR, recompute the hash,
    rewrite the manifest and the signature's first line consistently. Without
    the private key the Ed25519 line cannot follow."""
    signer, trust = _test_key()
    p = write_bundle(load_policy(DEMO), _tmp(), signer=signer, issued_at=ISSUED)
    m = _members(p)
    ir = json.loads(m[IR_NAME])
    ir["constraints"][0]["altitude_ceiling_m"] = 5.0
    ir_b = json.dumps(ir, sort_keys=True, separators=(",", ":")).encode()
    h = "sha256:" + hashlib.sha256(ir_b).hexdigest()
    man = json.loads(m[MANIFEST_NAME])
    man["policy_hash"] = h
    old_sig = m[SIGNATURE_NAME].decode().splitlines()[1]
    bad = _repack(p, _tmp("forged.tar.gz"), {
        IR_NAME: ir_b,
        MANIFEST_NAME: json.dumps(man, indent=2, sort_keys=True).encode(),
        SIGNATURE_NAME: f"{h} {signer.identity}\n{old_sig}\n".encode()})
    try:
        check_bundle(bad, trust)
    except ValueError as e:
        assert "Ed25519" in str(e), e
        return
    raise AssertionError("a consistent forgery passed a keyed signature")


def test_a_key_the_trust_store_does_not_list_is_refused():
    signer, _ = _test_key()
    _, other_trust = _test_key()
    p = write_bundle(load_policy(DEMO), _tmp(), signer=signer, issued_at=ISSUED)
    assert check_bundle(p, other_trust).signature == B.SIG_UNTRUSTED
    try:
        load_bundle(p, trust_store=other_trust)
    except BundleSignatureError as e:
        assert "untrusted" in str(e), e
        return
    raise AssertionError("a bundle from an unlisted key loaded as trusted")


# RFC 8032 section 7.1, TEST 1-3: (secret seed, public key, message, signature).
# Cross-checked against `cryptography` when these lines were written.
RFC8032 = [
    ("9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60",
     "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a", "",
     "e5564300c360ac729086e2cc806e828a84877f1eb8e5d974d873e06522490155"
     "5fb8821590a33bacc61e39701cf9b46bd25bf5f0595bbe24655141438e7a100b"),
    ("4ccd089b28ff96da9db6c346ec114e0f5b8a319f35aba624da8cf6ed4fb8a6fb",
     "3d4017c3e843895a92b70aa74d1b7ebc9c982ccf2ec4968cc0cd55f12af4660c", "72",
     "92a009a9f0d4cab8720e820b5f642540a2b27b5416503f8fb3762223ebdb69da"
     "085ac1e43e15996e458f3613d0f11d8c387b2eaeb4302aeeb00d291612bb0c00"),
    ("c5aa8df43f9f837bedb7442f31dcb7b166d38535076f094b85ce3a2e0b4458f7",
     "fc51cd8e6218a1a38da47ed00230f0580816ed13ba3303ac5deb911548908025", "af82",
     "6291d657deec24024827e69c3abe01a30ce548a284743a445e3680d7db5ac3ac"
     "18ff9b538d16f290ae67f760984dc6594a7c15e9716ed28dc027beceea1ec40a"),
]


def test_the_pure_python_verifier_passes_the_rfc_8032_vectors():
    """The fallback the SITL rails depend on, pinned to the standard's own
    numbers on every interpreter - and made to say no: a changed message, a
    flipped bit in the signature, another key, and an S >= L must all be
    refused."""
    for i, (seed, pub, msg, sig) in enumerate(RFC8032):
        pub_b, msg_b, sig_b = bytes.fromhex(pub), bytes.fromhex(msg), bytes.fromhex(sig)
        assert B.py_ed25519_verify(pub_b, msg_b, sig_b), pub
        key = B._PyEd25519TestKey(bytes.fromhex(seed))
        assert key.public_raw == pub_b and key.sign(msg_b) == sig_b, pub
        assert not B.py_ed25519_verify(pub_b, msg_b + b"\x00", sig_b)
        flipped = bytearray(sig_b)
        flipped[5] ^= 1
        assert not B.py_ed25519_verify(pub_b, msg_b, bytes(flipped))
        other = bytes.fromhex(RFC8032[(i + 1) % len(RFC8032)][1])
        assert not B.py_ed25519_verify(other, msg_b, sig_b)
        s = int.from_bytes(sig_b[32:], "little") + B._Q        # same S mod L
        assert not B.py_ed25519_verify(pub_b, msg_b, sig_b[:32] + s.to_bytes(32, "little"))
    assert not B.py_ed25519_verify(b"\x00" * 31, b"", b"\x00" * 64)


def test_the_pure_python_verifier_agrees_with_cryptography():
    """Where both exist, they must give the same answer, both ways round (each
    verifies the other's signatures, each refuses the same tampering)."""
    if not B.HAVE_CRYPTO:
        return SKIP
    from cryptography.hazmat.primitives import serialization as ser
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
    for i in range(12):
        data = os.urandom(40 + i * 17)
        k = Ed25519PrivateKey.generate()
        pub = k.public_key().public_bytes(ser.Encoding.Raw, ser.PublicFormat.Raw)
        sig = k.sign(data)
        assert B.py_ed25519_verify(pub, data, sig)
        assert B._ed25519_ok("cryptography", pub, sig, data)
        bad = data[:-1] + bytes([data[-1] ^ 0x80])
        assert not B.py_ed25519_verify(pub, bad, sig)
        assert not B._ed25519_ok("cryptography", pub, sig, bad)
        py = B._PyEd25519TestKey(os.urandom(32))
        assert B._ed25519_ok("cryptography", py.public_raw, py.sign(data), data)


def test_without_cryptography_a_signature_is_still_checked():
    """The 2026-10-06 review: the SITL rails (WSL venvs without `cryptography`)
    could never verify, so every --bundle there was refused or flown as
    "unverifiable-here" - and a forged signature flew under the flag reading
    the same as a real one. With the fallback the same bundle verifies there,
    and an edited one is refused there."""
    signer, trust = _test_key()
    p = write_bundle(load_policy(DEMO), _tmp(), signer=signer, issued_at=ISSUED)
    man = json.loads(_members(p)[MANIFEST_NAME])
    man["changelog"] = "approved by ITRI"
    edited = _repack(p, _tmp("edited.tar.gz"),
                     {MANIFEST_NAME: json.dumps(man, indent=2, sort_keys=True).encode()})
    with _no_crypto():
        assert B.verifier_name() == "pure-python"
        chk = check_bundle(p, trust)
        assert chk.signature == B.SIG_VERIFIED and "pure-python" in chk.detail, chk.detail
        _, src = B.load_for_flight(p, trust_store=trust)
        assert src["signature"] == B.SIG_VERIFIED, src
        try:
            check_bundle(edited, trust)
        except ValueError as e:
            assert "Ed25519 verification failed" in str(e), e
        else:
            raise AssertionError("an edited manifest verified under the fallback")


def test_with_no_verifier_at_all_a_signature_is_UNVERIFIABLE_not_passed():
    """The failure this project keeps finding: a check that cannot run must not
    look like a check that passed. Reachable now only with the fallback off."""
    signer, trust = _test_key()
    p = write_bundle(load_policy(DEMO), _tmp(), signer=signer, issued_at=ISSUED)
    with _no_crypto(fallback=False):
        assert B.verifier_name() is None
        chk = check_bundle(p, trust)
        assert chk.signature == B.SIG_UNVERIFIABLE, chk.signature
        assert "NOT checked" in chk.detail, chk.detail
        for load in (lambda: load_bundle(p, trust_store=trust),
                     lambda: B.load_for_flight(p, trust_store=trust)):
            try:
                load()
            except BundleSignatureError as e:
                assert "unverifiable" in str(e), e
            else:
                raise AssertionError("an unverifiable signature was treated as verified")


def test_an_unknown_signer_is_named_before_any_cryptography_runs():
    """The trust lookup needs no crypto, so it must not hide behind a missing
    library: an unlisted key is "untrusted-signer" even with no verifier."""
    signer, _ = _test_key()
    _, other_trust = _test_key()
    p = write_bundle(load_policy(DEMO), _tmp(), signer=signer, issued_at=ISSUED)
    with _no_crypto(fallback=False):
        assert check_bundle(p, other_trust).signature == B.SIG_UNTRUSTED


def test_a_stripped_signature_that_keeps_its_signer_is_refused():
    """The review's forgery, verbatim: edit the IR (fence ceiling 5 m), redo
    the hash, keep the TRUSTED signer's identity, drop only the Ed25519 line.
    It used to read "unsigned" - with the trusted name attached and the false
    detail "names no key" - and flew under --allow-unverified-bundle. No tool
    writes that layout, so it is refused outright, flag or no flag.

    Shown failing on the pre-fix code: check_bundle returned signature
    'unsigned' with the trusted identity, and load_for_flight flew it."""
    signer, trust = _test_key()
    good = write_bundle(load_policy(DEMO), _tmp(), signer=signer, issued_at=ISSUED)
    for forge_ir in (True, False):
        bad = _strip_signature(good, _tmp("stripped.tar.gz"), forge_ir=forge_ir)
        for load in (lambda: check_bundle(bad, trust),
                     lambda: B.load_for_flight(bad, allow_unverified=True,
                                               trust_store=trust)):
            try:
                load()
            except BundleSignatureError:
                raise AssertionError("refused as merely unverified; it is forged")
            except ValueError as e:
                assert "stripped" in str(e), e
            else:
                raise AssertionError("a stripped signature was accepted")


def test_a_signature_under_a_keyless_identity_is_refused():
    """The mirror image of a strip: an Ed25519 line under `unsigned:...` or the
    placeholder. No writer produces it, and reading it as "unsigned" would let
    a bundle carry a signature nobody checks."""
    pol = load_policy(DEMO)
    for who in (B.UNSIGNED, B.LEGACY_PLACEHOLDER_SIGNER):
        src = _unsigned(pol)
        man = json.loads(_members(src)[MANIFEST_NAME])
        man["signed_by"] = who
        bad = _repack(src, _tmp("keyless_sig.tar.gz"), {
            MANIFEST_NAME: json.dumps(man, indent=2, sort_keys=True).encode(),
            SIGNATURE_NAME: f"{pol.policy_hash} {who}\ned25519:{'ab' * 64}\n".encode()})
        try:
            check_bundle(bad)
        except ValueError as e:
            assert "names no key" in str(e), e
        else:
            raise AssertionError(f"a signature under {who!r} was accepted")


def test_a_signature_line_naming_another_hash_is_refused():
    """signature.txt line 1 binds a hash to a signer. One naming any hash other
    than the manifest's belongs to another bundle."""
    src = _unsigned(load_policy(DEMO))
    bad = _repack(src, _tmp("otherhash.tar.gz"), {
        SIGNATURE_NAME: f"sha256:{'0' * 64} {B.UNSIGNED}\n".encode()})
    try:
        check_bundle(bad)
    except ValueError as e:
        assert "signature covers" in str(e), e
    else:
        raise AssertionError("a signature line for another hash was accepted")


def test_a_manifest_naming_another_signer_than_the_signature_is_refused():
    """The manifest's signed_by is covered by the signature only when there IS
    one; without one it must still agree with the signature line, or an
    unsigned bundle could carry the lab key's name in its manifest."""
    src = _unsigned(load_policy(DEMO))
    man = json.loads(_members(src)[MANIFEST_NAME])
    man["signed_by"] = "lab-dev:ed25519:6ec75b535ef5c533"
    bad = _repack(src, _tmp("whosigned.tar.gz"), {
        MANIFEST_NAME: json.dumps(man, indent=2, sort_keys=True).encode()})
    try:
        check_bundle(bad)
    except ValueError as e:
        assert "manifest says signed_by" in str(e), e
    else:
        raise AssertionError("a manifest naming another signer was accepted")


def _legacy_bundle(pol, ir_obj=None) -> Path:
    """The pre-2026-10-06 format: 16-hex hash over model_dump() with nulls, IR
    with nulls, placeholder signer."""
    ir = json.dumps(ir_obj if ir_obj is not None else pol.model_dump(mode="json"),
                    sort_keys=True, separators=(",", ":")).encode()
    old = pol.legacy_hashes()["legacy16-include-defaults"]
    man = {"policy_id": pol.policy_id, "version": pol.version,
           "generation": pol.generation, "policy_hash": old,
           "signed_by": B.LEGACY_PLACEHOLDER_SIGNER, "changelog": "initial"}
    p = _tmp("legacy.tar.gz")
    with tarfile.open(p, "w:gz") as tar:
        for name, data in ((IR_NAME, ir),
                           (MANIFEST_NAME, json.dumps(man, indent=2, sort_keys=True).encode()),
                           (SIGNATURE_NAME, f"{old} {B.LEGACY_PLACEHOLDER_SIGNER}\n".encode())):
            info = tarfile.TarInfo(name=name)
            info.size, info.mtime = len(data), 0
            tar.addfile(info, io.BytesIO(data))
    return p


def test_a_bundle_written_before_the_release_still_loads_as_legacy():
    """It must load (refusing it would orphan whatever it documents), report
    its form, and still refuse to pass as signed."""
    pol = load_policy(WGS84)
    p = _legacy_bundle(pol)
    chk = check_bundle(p)
    assert chk.hash_form == "legacy16-include-defaults", chk.hash_form
    assert chk.signature == B.SIG_UNSIGNED
    assert chk.policy.policy_hash == pol.policy_hash
    try:
        load_bundle(p)
    except BundleSignatureError:
        pass
    else:
        raise AssertionError("a placeholder-signed bundle loaded as signed")


def test_a_legacy_bundle_with_an_unknown_key_is_refused():
    """A legacy hash digests the REBUILT object, so a key pydantic drops never
    reaches it. The v2 check hashes bytes; the legacy branch must compare the
    archived content instead. Shown failing on the pre-fix code: this bundle
    loaded under legacy16-include-defaults."""
    pol = load_policy(WGS84)
    ir = pol.model_dump(mode="json")
    ir["constraints"][0]["altitude_cieling_m"] = 40.0          # sic
    try:
        check_bundle(_legacy_bundle(pol, ir))
    except ValueError as e:
        # Since 2026-10-06 the model itself refuses the key ("cannot
        # represent"); the legacy branch's own comparison ("does not
        # represent") stays as the second line of defence.
        assert "represent" in str(e), e
    else:
        raise AssertionError("a legacy IR with an unknown key was accepted")


def test_issued_at_in_a_policy_file_is_kept_not_dropped():
    """The grant's worked example puts issued_at in the policy document. Until
    2026-10-06 it was refused here (it had been silently dropped before that).
    Now it is part of the document as the grant has it: kept, hashed, and
    carried to the bundle manifest - never dropped, which was the original
    defect. Shown failing on the pre-fix code: the file was refused."""
    f = Path(tempfile.mkdtemp()) / "with_issued_at.yaml"
    f.write_text(DEMO.read_text(encoding="utf-8")
                 + "\nissued_at: 2026-04-28T09:00:00Z\n", encoding="utf-8")
    pol = load_policy(f)
    assert pol.issued_at == "2026-04-28T09:00:00+00:00", pol.issued_at
    assert pol.policy_hash != load_policy(DEMO).policy_hash
    man = json.loads(_members(write_bundle(pol, _tmp(), signer=None))[MANIFEST_NAME])
    assert man["issued_at"] == "2026-04-28T09:00:00Z", man


def test_the_bundles_already_on_disk_still_open():
    """bundles/*.tar.gz written before the release (gitignored, local)."""
    old = [p for p in sorted((ROOT / "bundles").glob("*.tar.gz"))
           if not p.name.endswith(".replay.tar.gz")]
    if not old:
        return SKIP
    for p in old:
        chk = check_bundle(p)
        assert chk.hash_form, p.name
        assert chk.signature in (B.SIG_UNSIGNED, B.SIG_VERIFIED,
                                 B.SIG_UNVERIFIABLE), (p.name, chk.signature)


def test_the_lab_key_is_public_in_git_and_private_out_of_it():
    """The trust anchor must ship with a clone; the private key never may."""
    store = json.loads((ROOT / "policies" / "keys" / "trusted_signers.json")
                       .read_text(encoding="utf-8"))
    labs = {k: v for k, v in store["signers"].items() if k.startswith("lab-dev:ed25519:")}
    assert labs, store
    for v in labs.values():
        assert len(bytes.fromhex(v["public_key_hex"])) == 32
    try:
        ign = subprocess.run(["git", "-C", str(ROOT), "check-ignore", "-q",
                              "policies/keys/lab-dev-ed25519.pem"],
                             capture_output=True, timeout=20)
        tracked = subprocess.run(["git", "-C", str(ROOT), "ls-files",
                                  "policies/keys"], capture_output=True,
                                 text=True, timeout=20)
    except (OSError, subprocess.SubprocessError):
        return SKIP
    assert ign.returncode == 0, "the private key path is not gitignored"
    assert not any(x.endswith((".pem", ".key")) for x in tracked.stdout.split()), (
        f"a private key is tracked: {tracked.stdout}")


def test_the_schema_command_writes_the_published_schema():
    out = _tmp("policy_dsl.schema.json")
    assert B._main(["schema", "-o", str(out)]) == 0
    got = json.loads(out.read_text(encoding="utf-8"))
    assert got == B.policy_json_schema()
    assert got["x-hash-scheme"] == "sha256-canonical-v3"
    assert "PolygonFence" in got["$defs"] and "Corridor" in got["$defs"]


def test_a_flight_gets_its_policy_from_the_bundle_and_says_where_from():
    """ARCH-29 / WP3-21: the flight entry points load through load_for_flight.
    A YAML is recorded as unsigned; a bundle records its signature status; an
    unverified bundle does not fly unless the caller accepts that on record;
    and a run handed both is refused rather than left to pick one."""
    pol, src = B.load_for_flight(None, DEMO)
    assert src["kind"] == "yaml" and src["signature"] == "unsigned", src
    assert src["loaded_policy_hash"] == pol.policy_hash

    unsigned = _unsigned(load_policy(DEMO))
    try:
        B.load_for_flight(unsigned)
    except BundleSignatureError as e:
        assert "allow-unverified-bundle" in str(e), e
    else:
        raise AssertionError("an unsigned bundle flew as if signed")
    pol2, src2 = B.load_for_flight(unsigned, allow_unverified=True)
    assert src2["kind"] == "bundle" and src2["signature"] == B.SIG_UNSIGNED
    assert src2["accepted_unverified"] is True
    assert pol2.policy_hash == pol.policy_hash

    try:
        B.load_for_flight(unsigned, DEMO)
    except ValueError as e:
        assert "not both" in str(e), e
    else:
        raise AssertionError("a run with two candidate policies was accepted")

    signer, trust = _test_key()
    signed = write_bundle(load_policy(DEMO), _tmp(), signer=signer, issued_at=ISSUED)
    pol3, src3 = B.load_for_flight(signed, trust_store=trust)
    assert src3["signature"] == B.SIG_VERIFIED and "accepted_unverified" not in src3
    assert src3["issued_at"] == ISSUED and src3["signed_by"] == signer.identity


def test_every_flight_entry_point_can_fly_a_bundle():
    """Checked by reading the source: none of the three imports on this host
    (rclpy, pymavlink, Project AirSim)."""
    for rel in ("sitl/ros2_shield_node.py", "sitl/run_sitl_demo.py",
                "demo/follow_vlm.py"):
        src = (ROOT / rel).read_text(encoding="utf-8")
        assert '"--bundle"' in src, f"{rel} has no --bundle option"
        assert '"--allow-unverified-bundle"' in src, rel
        assert "load_for_flight(" in src, f"{rel} does not load through load_for_flight"
        assert "load_policy(args.policy)" not in src and "load_policy(policy_path" not in src, (
            f"{rel} still builds its rules straight from YAML")
        assert src.count('"policy_source"') >= 2, (
            f"{rel} must record the policy source in metrics.json AND kpi.json")
        assert "policy_source=policy_source" in src or \
            "policy_source=self.policy_source" in src, (
                f"{rel} does not hand the policy source to its replay bundle")


def test_a_refused_policy_leaves_the_previous_run_untouched():
    """follow_vlm used to delete demo/out/<tag>/flight_log.jsonl BEFORE loading
    the policy, so a refused bundle (or --bundle with --policy) on a re-used
    tag destroyed the old log and orphaned its manifest and kpi.json. The
    policy is now loaded before the output folder is touched. Checked by
    source order, since neither script imports on this host. Shown failing on
    the pre-fix code: the unlink came first in follow_vlm, mkdir in
    run_sitl_demo."""
    for rel, marker in (("demo/follow_vlm.py", "(out / f).unlink()"),
                        ("sitl/run_sitl_demo.py", "out.mkdir(")):
        src = (ROOT / rel).read_text(encoding="utf-8")
        assert src.count("load_for_flight(") == 1, rel
        assert src.index("load_for_flight(") < src.index(marker), (
            f"{rel}: the output folder is touched before the policy is loaded")


def test_the_cli_refuses_new_content_under_a_locked_version():
    """`python -m guardrail.bundle` is a release: it must not publish changed
    content under a version the lock already gives to other content."""
    src = DEMO.read_text(encoding="utf-8").replace("alt_max_m: 20", "alt_max_m: 25")
    assert src != DEMO.read_text(encoding="utf-8")
    f = Path(tempfile.mkdtemp()) / "edited.yaml"
    f.write_text(src, encoding="utf-8")
    assert B._main([str(f), "-o", str(_tmp()), "--unsigned"]) == 2


# --------------------------------------------------------------------------- #
# WGS84
# --------------------------------------------------------------------------- #

def test_the_projection_returns_north_then_east():
    """The axis order, which is the easy thing to get wrong.

    One degree of latitude north is ~111 km of X (North). One degree of
    longitude east is ~111 km * cos(lat) of Y (East). If these are transposed
    every constraint lands ninety degrees off with nothing to complain.
    """
    proj = LocalProjection(25.0, 121.0)
    x, y = proj.to_local(25.01, 121.0)          # north only
    assert x > 1000.0 and abs(y) < 1e-6, (x, y)
    x, y = proj.to_local(25.0, 121.01)          # east only
    assert y > 900.0 and abs(x) < 1e-6, (x, y)


def test_the_projection_round_trips():
    proj = LocalProjection(25.04245, 121.5314)
    lat, lon = proj.to_latlon(*proj.to_local(25.0428, 121.5318))
    assert abs(lat - 25.0428) < 1e-9 and abs(lon - 121.5318) < 1e-9


def test_a_wgs84_policy_loads_into_local_metres():
    pol = load_policy(WGS84)
    fence = pol.constraints[0]
    for v in fence.vertices:
        assert abs(v.x) < 200.0 and abs(v.y) < 200.0, (
            f"vertex {v} was not projected into the local frame")
    assert pol.origin is not None and abs(pol.origin.lat - 25.04245) < 1e-9


def test_a_wgs84_policy_actually_works_in_the_shield():
    """Projected or not, a fence is a fence."""
    pol = load_policy(WGS84)
    when = datetime(2026, 9, 7, 9, 0)                 # Monday, curfew in force
    sh = Shield(pol, lookahead_s=3.0, dt=0.5, now=lambda: when)
    assert sh.state_is_unsafe(State(x=0.0, y=0.0, up=20.0)), (
        "the origin sits inside the school-yard polygon and should be unsafe")
    assert not sh.state_is_unsafe(State(x=300.0, y=300.0, up=20.0))


def test_geographic_coordinates_without_an_origin_get_the_references_frame_never_0_0():
    """An ASSUMED origin does not fail - it relocates the policy and validates,
    which is why (0, 0) is never used. The grant's form has no origin at all,
    so since 2026-10-06 the frame is DERIVED from the geometry the way the
    reference derives it (mean of the first polygon's vertices): every rule
    stays where its author put it, and the frame is near the rules.

    Shown failing on the pre-fix code: the policy was refused."""
    raw = {"policy_id": "p", "constraints": [
        {"id": "f", "type": "polygon_fence",
         "vertices": [{"lat": 25.0, "lon": 121.0}, {"lat": 25.1, "lon": 121.0},
                      {"lat": 25.1, "lon": 121.1}]}]}
    out = project_raw(raw)
    pts = out["constraints"][0]["vertices"]
    assert all(abs(p["x"]) < 20_000 and abs(p["y"]) < 20_000 for p in pts), pts
    from guardrail.models import Policy
    pol = Policy.model_validate(raw)
    assert pol.frame_origin == ((25.0 + 25.1 + 25.1) / 3, (121.0 + 121.0 + 121.1) / 3)
    corridor_only = {"policy_id": "p", "constraints": [
        {"id": "c", "type": "corridor", "width_m": 10,
         "centerline": [{"lat": 25.0, "lon": 121.0}, {"lat": 25.0, "lon": 121.001}]}]}
    assert Policy.model_validate(corridor_only).frame_origin == (25.0, 121.0005), \
        "with no polygon the reference would use (0, 0); this uses the first geometry"


def test_mixing_frames_in_one_point_is_refused():
    raw = {"policy_id": "p", "origin": {"lat": 25.0, "lon": 121.0},
           "constraints": [
               {"id": "f", "type": "polygon_fence",
                "vertices": [{"lat": 25.0, "lon": 121.0, "x": 1.0, "y": 2.0}]}]}
    try:
        project_raw(raw)
    except ValueError as e:
        assert "mix frames" in str(e), e
        return
    raise AssertionError("a point in two frames at once was accepted")


def test_every_existing_metre_policy_still_loads_untouched():
    """The support is ADDITIVE. If this fails, the change was a migration."""
    n = 0
    for p in sorted((ROOT / "policies").glob("*.yaml")):
        pol = load_policy(p)
        assert pol.constraints, p.name
        n += 1
    assert n >= 20, f"only {n} policies loaded"


def test_a_wgs84_policy_bundles_and_reloads():
    pol = load_policy(WGS84)
    got = load_bundle(_unsigned(pol), require_signature=False)
    assert got.policy_hash == pol.policy_hash
    assert got.origin is not None, "the anchor for the metres was lost"


# --------------------------------------------------------------------------- #
# Constraint Summary Pack
# --------------------------------------------------------------------------- #

def test_the_pack_contains_every_rule_not_just_the_ones_with_shorthands():
    """REGRESSION: the stand-off the Shield enforces was invisible to the pilot.

    `build_prompt` used to emit fences, the altitude band and the speed cap and
    nothing else, so a policy whose headline rule is a 10 m pedestrian stand-off
    described none of it. The planner could only discover that rule by being
    repaired against it.
    """
    pack = ConstraintCompiler(load_policy(PED)).summary_pack()
    assert pack["n_rules"] == 4, pack["rules_by_type"]
    assert pack["rules_by_type"].get("subject_standoff") == 2, pack
    ids = {r["id"] for r in pack["rules"]}
    assert {"standoff-pedestrian", "standoff-any"} <= ids, ids


def test_the_prompt_mentions_the_standoff_in_words():
    """Checked against the sentence list, not the rendered YAML: safe_dump wraps
    long lines, so a substring test on the document fails on formatting rather
    than on content."""
    c = ConstraintCompiler(load_policy(PED))
    said = " ".join(c._sentences())
    assert "10 m away from any pedestrian" in said, said
    assert "5 m away from anything you are following" in said, said


def test_the_prompt_describes_a_corridor():
    c = ConstraintCompiler(load_policy(ROOT / "policies" / "corridor_survey.yaml"))
    said = " ".join(c._sentences())
    assert "Stay inside corridor" in said, said
    assert "20 m of its centerline" in said, said


def test_the_pack_is_pinned_to_the_policy_hash():
    """A summary that cannot be traced to a policy is a summary of nothing."""
    pol = load_policy(PED)
    assert ConstraintCompiler(pol).summary_pack()["policy_hash"] == pol.policy_hash


def test_the_pack_can_be_written_alongside_a_flight():
    out = Path(tempfile.mkdtemp()) / "csp.json"
    ConstraintCompiler(load_policy(PED)).write_summary_pack(out)
    got = json.loads(out.read_text(encoding="utf-8"))
    assert got["n_rules"] == 4


def test_the_prompt_keeps_the_shorthands_the_demos_already_read():
    """The pack is added underneath, not swapped in - this stays a drop-in."""
    import yaml as _y
    c = ConstraintCompiler(load_policy(DEMO))
    doc = _y.safe_load(c.build_prompt(c.parse_command("go to (30, 30) altitude 15")))
    cons = doc["constraints"]
    for f in ("no_fly_zones", "altitude_band_m", "speed_max_mps", "policy_hash"):
        assert f in cons, f"{f} disappeared from the prompt"
    assert cons["summary_pack"]["n_rules"] == len(load_policy(DEMO).constraints)


if __name__ == "__main__":
    # A test that cannot run here (no `cryptography`) prints SKIP, never PASS.
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs cryptography or git)")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
