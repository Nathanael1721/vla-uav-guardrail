"""The signed policy bundle — WP1's deployable artefact.

    write_bundle(policy, "bundles/demo.tar.gz", changelog="added school curfew")
    policy = load_bundle("bundles/demo.tar.gz")     # refuses if tampered or unsigned
    check  = check_bundle("bundles/demo.tar.gz")    # same checks, status returned

WHAT THE GRANT ASKS FOR

WP1's deliverable line reads: "YAML/JSON DSL -> IR/AST; **signed policy bundle**
(hash + semver) covering GeoFence, envelope, corridor, time windows, breach
actions", accepted on "Policy load round-trip; bundle replayability". The Policy
DSL page lists what every published bundle carries: policy_id, version (semver),
generation, policy_hash ("SHA-256 over the canonicalised IR, post-merge,
pre-sign"), signed_by ("Lab CA or ITRI CA reference") and changelog; its worked
example adds issued_at.

THE CONTAINER IS THE REFERENCE'S; THE CONTENTS NOW CROSS-LOAD

`packages/policy-dsl/src/policy_dsl/ingest.py` writes `ir.json`,
`manifest.json`, `signature.txt`, member mtimes pinned to 0, and so does this
module. The first line of `signature.txt` is the reference's `<policy_hash>
<signed_by>`; since 2026-10-06 a second line carries a real signature.

Matching the container is NOT matching the contents, and until 2026-10-06 this
docstring implied it was: neither loader accepted the other's bundle (audit card
WP1-05). The two IRs differ - the reference stores its authored document
(grant field names, `geometry:` blocks, every default written out), this code
stores its own IR - so the same policy hashes differently. Now:

  * reading: `check_bundle` accepts a reference bundle. Its ir.json must be
    exactly what `Policy.reference_ir()` rebuilds, and its hash is recognised
    as the named form REFERENCE_FORM (`hash_form`);
  * writing: `write_bundle(..., ir_form="reference")` ships `reference_ir()`
    and the reference's hash, which the reference loader reads (checked with
    its own code in tests/test_policy_dsl_grant_form.py where its 3.11
    environment exists). Only policies the reference can represent qualify:
    WGS84 polygon fences without a margin ring and altitude envelopes, no
    schedules (`Policy.reference_problems`).

A reference-form bundle's IR writes every default out, so the policy rebuilt
from it has a third hash - neither the manifest's nor the source policy's (the
source policy recognises the manifest's hash as REFERENCE_FORM). Until
2026-10-08 the flight loaders therefore refused every reference-form bundle.

BOTH HASHES IN EVERY MANIFEST (2026-10-08, deviation 10)

Every manifest written since names the policy under both canonical forms,
`hashes: {sha256-canonical-v3: ..., reference-policy-dsl-0.1: ...}`, and
`check_bundle` verifies both (`_verify_both_hashes`). The reference's entry is
null, with `reference_problems`, where its Phase-1 DSL cannot hold the policy -
measured: 0 of the 29 shipped policies (metre vertices, margin rings, types it
lacks), 3 of its own 3 documents. A reference-form manifest also carries
`reference_delta`, which rebuilds the source policy exactly
(`Policy.from_reference`): such a bundle is read by the reference's loader
(which checks `policy_hash`, the reference hash) and by this one (which checks
both, and flies the source policy under the hash its manifest records). A
reference-form bundle without the delta - the reference's own - is still
refused for flight. `policy_hash` of a bundle in this project's form is what
it always was: no stored hash moved.

docs/DESIGN-policy-dsl-grant-form.md lists what still differs.

THE SIGNATURE (2026-10-06)

Until this date the signer was `lab-ca:dev-placeholder`, copied from the
reference, and the docstring here said so. It is now an Ed25519 detached
signature over the exact `manifest.json` bytes, made with a lab DEVELOPMENT
key. The manifest carries `policy_hash`, and `policy_hash` is the SHA-256 of
the exact `ir.json` bytes, so one signature covers the whole chain:

    signature.txt line 2  --Ed25519-->  manifest.json  --sha256-->  ir.json

Who signs is still the PI's decision (grant open question: "Lab-internal CA or
ITRI CA?"). That is why the trust anchor is DATA, not code:
`policies/keys/trusted_signers.json` maps a `signed_by` identity to a public
key. Swapping to the chosen CA means adding its key there and signing with it;
no line of this module changes. The private key never enters git
(`policies/keys/*.pem` is ignored).

VERIFYING EVERYWHERE, SIGNING WHERE THE KEY IS

`cryptography` is an optional import. Neither WSL setup script installs it
(sitl/setup_ros2.sh, sitl/setup_sitl.sh): ~/venv-ap has none, ~/venv-ros sees
cryptography 41.0.7 only through the system site-packages, and the 3.11
`vla-drone` environment has none. The first cut of this release answered a
missing library with the status "unverifiable-here", which meant a flight rail
without it could never verify a signature - every --bundle there was refused,
or flown under --allow-unverified-bundle with a forged signature reading the
same as a real one (2026-10-06 review). So verification now has a pure-Python
fallback (RFC 8032, `py_ed25519_verify` below), and no environment depends on
which site-packages it happens to see. Verifying handles only public
data, so a big-integer implementation leaks nothing; signing still needs
`cryptography` and the private key, and is never done any other way for a
real key.

NEVER A SILENT PASS

Each check returns exactly one status. "verified" is the only good one.
"unsigned" is reserved for the two layouts that honestly name no key (the
reference's placeholder, and what write_bundle writes without a key). A bundle
that names a key but carries no signature is "bad-signature" - that is what
stripping the Ed25519 line from a signed bundle looks like, and no tool here
writes it. "untrusted-signer" is decided before any cryptography runs, so it
is reported in every environment. "unverifiable-here" survives only for an
interpreter with neither verifier (the fallback switched off), and
`load_bundle` refuses it like any other unverified status unless the caller
says `require_signature=False` and records the policy as unsigned (the flight
entry points do, in metrics.json and kpi.json).

Integrity checks are not optional and need no cryptography: the manifest hash
must match the IR bytes, the signature line must name the same hash and
signer, and every required member must be present. Those raise ValueError.
"""
from __future__ import annotations

import gzip
import hashlib
import io
import json
import os
import tarfile
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .models import (HASH_SCHEME, IR2_FORMS, IR_SCHEMA_VERSION, PRIOR_SCHEME_V2, SHORT_FORM,
                     REFERENCE_FORM, Policy, _prune_nulls, canonical_json,
                     ir2_rule_view, load_policy)

try:                                    # optional: see "VERIFYING EVERYWHERE"
    from cryptography.exceptions import InvalidSignature
    from cryptography.hazmat.primitives import serialization
    from cryptography.hazmat.primitives.asymmetric.ed25519 import (
        Ed25519PrivateKey, Ed25519PublicKey)
    HAVE_CRYPTO = True
except ImportError:                     # pragma: no cover - env dependent
    HAVE_CRYPTO = False

ROOT = Path(__file__).resolve().parents[1]

IR_NAME = "ir.json"
MANIFEST_NAME = "manifest.json"
SIGNATURE_NAME = "signature.txt"
# The multi-scale geometry (WP2-16; guardrail/ir.py "The bundle ships them"),
# since 2026-11-02. Optional on read: a bundle written before has none.
MULTISCALE_NAME = "multiscale.json"

# The reference's placeholder identity. Bundles written before 2026-10-06 carry
# it; they still load (integrity intact) but report signature "unsigned".
LEGACY_PLACEHOLDER_SIGNER = "lab-ca:dev-placeholder"
# What a bundle written with no key says about itself. Never dressed up.
UNSIGNED = "unsigned:no-signing-key"
# Kept for importers of the old name; it is the legacy placeholder, not a key.
_SIGNED_BY = LEGACY_PLACEHOLDER_SIGNER

KEY_DIR = ROOT / "policies" / "keys"
DEFAULT_PRIVATE_KEY = KEY_DIR / "lab-dev-ed25519.pem"
DEFAULT_TRUST_STORE = KEY_DIR / "trusted_signers.json"
LOCK_PATH = ROOT / "policies" / "policy.lock.json"
SCHEMA_PATH = ROOT / "policies" / "policy_dsl.schema.json"

# Signature statuses. Exactly one is "good"; every other one is a reason.
SIG_VERIFIED = "verified"
SIG_UNVERIFIABLE = "unverifiable-here"
SIG_UNSIGNED = "unsigned"
SIG_UNTRUSTED = "untrusted-signer"
SIG_BAD = "bad-signature"


class BundleSignatureError(ValueError):
    """The bundle is intact but its signature could not be verified as trusted."""


# --------------------------------------------------------------------------- #
# Ed25519 without `cryptography` (RFC 8032)
# --------------------------------------------------------------------------- #
#
# This follows the RFC's own Python reference (section 6) and its verification
# procedure (section 5.1.7). It checks [S]B = R + [k]A', which section 5.1.7
# permits in place of the cofactored equation, as the RFC's sample code does.
# Speed is irrelevant here (one check per bundle, a few milliseconds), and
# constant time is irrelevant to VERIFYING: every input is public.
#
# tests/test_bundle.py checks it against the RFC's test vectors on every
# interpreter, and against `cryptography` wherever both are present.

_P = 2 ** 255 - 19
_Q = 2 ** 252 + 27742317777372353535851937790883648493   # group order ("L")
_D = -121665 * pow(121666, _P - 2, _P) % _P
_SQRT_M1 = pow(2, (_P - 1) // 4, _P)


def _recover_x(y: int, sign: int) -> int | None:
    if y >= _P:
        return None
    x2 = (y * y - 1) * pow(_D * y * y + 1, _P - 2, _P) % _P
    if x2 == 0:
        return None if sign else 0
    x = pow(x2, (_P + 3) // 8, _P)
    if (x * x - x2) % _P:
        x = x * _SQRT_M1 % _P
    if (x * x - x2) % _P:
        return None
    if (x & 1) != sign:
        x = _P - x
    return x


_GY = 4 * pow(5, _P - 2, _P) % _P
_GX = _recover_x(_GY, 0)
_G = (_GX, _GY, 1, _GX * _GY % _P)          # base point, extended coordinates


def _pt_add(a, b):
    pa = (a[1] - a[0]) * (b[1] - b[0]) % _P
    pb = (a[1] + a[0]) * (b[1] + b[0]) % _P
    pc = 2 * a[3] * b[3] * _D % _P
    pd = 2 * a[2] * b[2] % _P
    e, f, g, h = pb - pa, pd - pc, pd + pc, pb + pa
    return (e * f % _P, g * h % _P, f * g % _P, e * h % _P)


def _pt_mul(s: int, pt):
    acc = (0, 1, 1, 0)
    while s > 0:
        if s & 1:
            acc = _pt_add(acc, pt)
        pt = _pt_add(pt, pt)
        s >>= 1
    return acc


def _pt_equal(a, b) -> bool:
    return ((a[0] * b[2] - b[0] * a[2]) % _P == 0
            and (a[1] * b[2] - b[1] * a[2]) % _P == 0)


def _pt_decompress(s: bytes):
    if len(s) != 32:
        return None
    y = int.from_bytes(s, "little")
    sign = y >> 255
    y &= (1 << 255) - 1
    x = _recover_x(y, sign)
    return None if x is None else (x, y, 1, x * y % _P)


def _pt_compress(pt) -> bytes:
    zinv = pow(pt[2], _P - 2, _P)
    x, y = pt[0] * zinv % _P, pt[1] * zinv % _P
    return (y | ((x & 1) << 255)).to_bytes(32, "little")


def _sha512_modq(data: bytes) -> int:
    return int.from_bytes(hashlib.sha512(data).digest(), "little") % _Q


def py_ed25519_verify(public: bytes, message: bytes, signature: bytes) -> bool:
    """RFC 8032 section 5.1.7 verification in plain Python. True only for a
    valid signature by `public` over `message`; malformed input is False."""
    if len(public) != 32 or len(signature) != 64:
        return False
    a_pt = _pt_decompress(public)
    r_pt = _pt_decompress(signature[:32])
    if a_pt is None or r_pt is None:
        return False
    s = int.from_bytes(signature[32:], "little")
    if s >= _Q:
        return False
    k = _sha512_modq(signature[:32] + public + message)
    return _pt_equal(_pt_mul(s, _G), _pt_add(r_pt, _pt_mul(k, a_pt)))


class _PyEd25519TestKey:
    """RFC 8032 signing in plain Python, FOR TESTS AND TEST VECTORS ONLY.

    It exists so the signature tests run on the interpreters this fallback is
    for (3.11, the WSL venvs), where `cryptography` cannot make a key. It is not
    constant-time, it takes a raw 32-byte seed rather than a PEM, and nothing
    in this module ever hands it a real key: `load_signer` and the CLI sign
    through `cryptography` only.
    """

    def __init__(self, seed: bytes):
        if len(seed) != 32:
            raise ValueError("an Ed25519 seed is 32 bytes")
        h = hashlib.sha512(seed).digest()
        a = int.from_bytes(h[:32], "little")
        a &= (1 << 254) - 8
        a |= 1 << 254
        self._a, self._prefix = a, h[32:]
        self.public_raw = _pt_compress(_pt_mul(a, _G))

    def sign(self, message: bytes) -> bytes:
        r = _sha512_modq(self._prefix + message)
        r_enc = _pt_compress(_pt_mul(r, _G))
        k = _sha512_modq(r_enc + self.public_raw + message)
        return r_enc + ((r + k * self._a) % _Q).to_bytes(32, "little")


# Switched off only by tests, to reach the "unverifiable-here" status that an
# interpreter with no verifier at all would report.
PY_VERIFY_FALLBACK = True


def verifier_name() -> str | None:
    """Which Ed25519 verifier this interpreter will use, or None for none."""
    if HAVE_CRYPTO:
        return "cryptography"
    return "pure-python" if PY_VERIFY_FALLBACK else None


def _ed25519_ok(engine: str, public: bytes, signature: bytes, data: bytes) -> bool:
    if engine == "cryptography":
        try:
            Ed25519PublicKey.from_public_bytes(public).verify(signature, data)
        except (InvalidSignature, ValueError):
            return False
        return True
    return py_ed25519_verify(public, data, signature)


# --------------------------------------------------------------------------- #
# Keys and the trust store
# --------------------------------------------------------------------------- #

def _key_id(public_raw: bytes) -> str:
    return "ed25519:" + hashlib.sha256(public_raw).hexdigest()[:16]


@dataclass(frozen=True)
class Signer:
    """A private key plus the identity it signs as (`<authority>:ed25519:<id>`).

    The authority is a label from the trust store - "lab-dev" today - not a
    claim the key makes about itself.
    """
    authority: str
    private_key: Any = field(repr=False)

    @property
    def public_raw(self) -> bytes:
        raw = getattr(self.private_key, "public_raw", None)   # _PyEd25519TestKey
        if raw is not None:
            return raw
        return self.private_key.public_key().public_bytes(
            serialization.Encoding.Raw, serialization.PublicFormat.Raw)

    @property
    def key_id(self) -> str:
        return _key_id(self.public_raw)

    @property
    def identity(self) -> str:
        return f"{self.authority}:{self.key_id}"

    def sign(self, data: bytes) -> str:
        # Ed25519 is deterministic (RFC 8032): the same key and bytes give the
        # same signature, so a signed bundle stays byte-reproducible.
        return self.private_key.sign(data).hex()


def _require_crypto(what: str) -> None:
    if not HAVE_CRYPTO:
        raise RuntimeError(
            f"{what} needs the 'cryptography' package, which this interpreter "
            f"does not have. Signing is never faked; use an environment that "
            f"has it (vla-real does).")


def trust_store_path() -> Path:
    env = os.environ.get("VLAGUARD_TRUST_STORE")
    return Path(env) if env else DEFAULT_TRUST_STORE


def load_trust_store(path: str | Path | None = None) -> dict[str, dict]:
    """identity -> {"public_key_hex", "authority", ...}. Missing file = empty.

    An empty store is not an error here; it makes every signed bundle
    "untrusted-signer", which is the loud answer.
    """
    p = Path(path) if path else trust_store_path()
    if not p.is_file():
        return {}
    data = json.loads(p.read_text(encoding="utf-8"))
    return dict(data.get("signers") or {})


def generate_signing_key(out: str | Path | None = None, authority: str = "lab-dev",
                         trust_store: str | Path | None = None) -> Signer:
    """Create an Ed25519 key, save it (PEM, PKCS8), and register its public half.

    Refuses to overwrite an existing key: replacing the lab key silently would
    make every bundle signed with the old one untrusted at once.
    """
    _require_crypto("generating a signing key")
    out = Path(out) if out else DEFAULT_PRIVATE_KEY
    if out.exists():
        raise FileExistsError(f"{out} exists; refusing to replace a signing key")
    out.parent.mkdir(parents=True, exist_ok=True)
    key = Ed25519PrivateKey.generate()
    out.write_bytes(key.private_bytes(serialization.Encoding.PEM,
                                      serialization.PrivateFormat.PKCS8,
                                      serialization.NoEncryption()))
    try:
        os.chmod(out, 0o600)
    except OSError:                                     # Windows: best effort
        pass
    signer = Signer(authority=authority, private_key=key)
    register_public_key(signer, trust_store)
    return signer


def register_public_key(signer: Signer, trust_store: str | Path | None = None,
                        note: str = "") -> Path:
    """Add the signer's PUBLIC key to the trust store (creating it if needed)."""
    p = Path(trust_store) if trust_store else trust_store_path()
    data = (json.loads(p.read_text(encoding="utf-8")) if p.is_file()
            else {"_doc": _TRUST_DOC, "signers": {}})
    data.setdefault("signers", {})[signer.identity] = {
        "authority": signer.authority,
        "algorithm": "ed25519",
        "public_key_hex": signer.public_raw.hex(),
        "registered": datetime.now(timezone.utc).strftime("%Y-%m-%d"),
        "note": note or ("lab DEVELOPMENT key - not a CA. Replace with the CA "
                         "the PI names (lab or ITRI) before final delivery."),
    }
    p.parent.mkdir(parents=True, exist_ok=True)
    # newline="\n": the same bytes from Windows and WSL, or every run from the
    # other OS rewrites the whole file (see the repo-line-endings memory).
    p.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n",
                 encoding="utf-8", newline="\n")
    return p


_TRUST_DOC = ("Public keys whose signatures guardrail.bundle accepts, keyed by "
              "the signed_by identity. Data, not code: swapping the signing "
              "authority (lab CA or ITRI CA, the PI's decision) means adding its "
              "key here and signing with it. Private keys never go in git.")


def load_signer(path: str | Path | None = None,
                trust_store: str | Path | None = None) -> Signer:
    """Load a private key; its authority label comes from the trust store.

    A key the store does not list still signs - as `unregistered` - so the
    resulting bundle fails verification loudly instead of passing as trusted.
    """
    _require_crypto("signing")
    p = Path(path) if path else Path(os.environ.get("VLAGUARD_SIGNING_KEY")
                                     or DEFAULT_PRIVATE_KEY)
    key = serialization.load_pem_private_key(p.read_bytes(), password=None)
    if not isinstance(key, Ed25519PrivateKey):
        raise ValueError(f"{p} is not an Ed25519 private key")
    kid = _key_id(key.public_key().public_bytes(serialization.Encoding.Raw,
                                                serialization.PublicFormat.Raw))
    authority = "unregistered"
    for ident, entry in load_trust_store(trust_store).items():
        if ident.endswith(":" + kid):
            authority = entry.get("authority") or ident.rsplit(":", 2)[0]
            break
    return Signer(authority=authority, private_key=key)


def default_signer() -> Signer | None:
    """The lab key if this machine has it and can use it, else None."""
    if not HAVE_CRYPTO:
        return None
    p = Path(os.environ.get("VLAGUARD_SIGNING_KEY") or DEFAULT_PRIVATE_KEY)
    if not p.is_file():
        return None
    return load_signer(p)


def _resolve_signer(signer) -> Signer | None:
    if signer == "auto":
        return default_signer()
    if signer is None or isinstance(signer, Signer):
        return signer
    return load_signer(signer)


def verify_signature(signed_by: str, sig_hex: str | None, data: bytes,
                     trust_store: str | Path | dict | None = None
                     ) -> tuple[str, str]:
    """(status, detail) for one detached signature. Never raises for a bad one;
    the caller decides, and the status says exactly what happened.

    The order matters, and each step is a finding from the 2026-10-06 review:

    1. "unsigned" only for the two identities that honestly name no key. A
       bundle that keeps a key's identity and drops the Ed25519 line is a
       stripped signature: it used to read as "unsigned" with the TRUSTED
       signer's name attached, and then flew under --allow-unverified-bundle.
       It is "bad-signature" now, which every reader refuses.
    2. The trust store is consulted before any cryptography, so an unknown
       signer is "untrusted-signer" in every environment.
    3. Then the signature is checked - by `cryptography` where installed, else
       by the RFC 8032 fallback above.
    """
    if signed_by in (LEGACY_PLACEHOLDER_SIGNER, UNSIGNED):
        if sig_hex:
            return SIG_BAD, (f"a signature is attached but the signer "
                             f"{signed_by!r} names no key; no tool here writes "
                             f"that layout")
        return SIG_UNSIGNED, (f"no signature: signer is {signed_by!r}, which "
                              f"names no key")
    if not signed_by:
        return SIG_BAD, "the signature line names no signer at all"
    if not sig_hex:
        return SIG_BAD, (f"signer {signed_by!r} names a key, but the bundle "
                         f"carries no signature: the Ed25519 line was "
                         f"stripped (no tool here writes a keyed identity "
                         f"without one)")
    if isinstance(trust_store, dict):
        trust, where = trust_store, "the supplied trust store"
    else:
        trust = load_trust_store(trust_store)
        where = str(trust_store or trust_store_path())
    entry = trust.get(signed_by)
    if entry is None:
        return SIG_UNTRUSTED, f"{signed_by!r} is not in the trust store ({where})"
    engine = verifier_name()
    if engine is None:
        return SIG_UNVERIFIABLE, ("signature present but NOT checked: this "
                                  "interpreter has neither 'cryptography' nor "
                                  "the pure-Python verifier enabled")
    try:
        public = bytes.fromhex(entry["public_key_hex"])
        signature = bytes.fromhex(sig_hex)
    except (KeyError, TypeError, ValueError) as exc:
        return SIG_BAD, (f"Ed25519 verification failed for {signed_by!r}: "
                         f"unreadable key or signature ({type(exc).__name__})")
    if not _ed25519_ok(engine, public, signature, data):
        return SIG_BAD, f"Ed25519 verification failed for {signed_by!r} ({engine})"
    return SIG_VERIFIED, f"Ed25519 signature by {signed_by!r} verified ({engine})"


def signature_text(digest: str, signer: Signer | None, data: bytes) -> bytes:
    """`<digest> <signed_by>` (the reference's line) + `ed25519:<sig>` if signed."""
    if signer is None:
        return f"{digest} {UNSIGNED}\n".encode()
    return f"{digest} {signer.identity}\ned25519:{signer.sign(data)}\n".encode()


def parse_signature(text: str) -> tuple[str, str, str | None]:
    """(digest, signed_by, sig_hex or None) from a signature.txt body."""
    lines = [ln.strip() for ln in text.strip().splitlines() if ln.strip()]
    if not lines:
        return "", "", None
    head = lines[0].split(" ", 1)
    digest, signed_by = head[0], (head[1] if len(head) > 1 else "")
    sig = None
    for ln in lines[1:]:
        if ln.startswith("ed25519:"):
            sig = ln[len("ed25519:"):]
    return digest, signed_by, sig


# --------------------------------------------------------------------------- #
# Writing
# --------------------------------------------------------------------------- #

def _issued_at(value=None) -> str:
    """ISO-8601 UTC, second resolution.

    Explicit value first; then SOURCE_DATE_EPOCH, the reproducible-builds
    convention, so a rebuilt bundle can carry the original issue time; then now.
    A bundle written twice at different times therefore differs ONLY in
    issued_at and the signature over it - never in ir.json or policy_hash.
    """
    if value is None:
        sde = os.environ.get("SOURCE_DATE_EPOCH")
        dt = (datetime.fromtimestamp(int(sde), tz=timezone.utc) if sde
              else datetime.now(timezone.utc))
    elif isinstance(value, datetime):
        dt = value if value.tzinfo else value.replace(tzinfo=timezone.utc)
    else:
        dt = datetime.fromisoformat(str(value).replace("Z", "+00:00"))
        dt = dt if dt.tzinfo else dt.replace(tzinfo=timezone.utc)
    return dt.astimezone(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


IR_FORMS = ("ours", "reference")


def _ir_for(policy: Policy, ir_form: str) -> tuple[bytes, str, str]:
    """(ir.json bytes, policy_hash, hash scheme name) for one IR form."""
    if ir_form == "ours":
        return policy.canonical_bytes(), policy.policy_hash, HASH_SCHEME
    if ir_form == "reference":
        ref = policy.reference_ir()
        if ref is None:
            raise ValueError(
                f"{policy.policy_id}: the reference implementation cannot "
                f"represent this policy: " + "; ".join(policy.reference_problems()))
        return canonical_json(ref), policy.reference_hash(), REFERENCE_FORM
    raise ValueError(f"ir_form must be one of {IR_FORMS}, not {ir_form!r}")


def policy_diff(previous: Policy, policy: Policy) -> dict[str, Any]:
    """What changed from `previous` to `policy`, rule by rule (grant, Policy
    DSL page: the bundle's changelog is "a diff against the previous
    version"; audit item X-10). Rules are matched by id: `added` and
    `removed` list ids, `changed` maps an id kept on both sides to the
    fields whose values differ (its `type` among them when the rule was
    replaced by another type under the same id), and `policy_fields` names
    the policy-level fields that differ (version, generation, layers_merged,
    origin - not the rules, not issued_at). `against` names the previous version exactly,
    by its policy_hash. Deterministic: every list sorted."""
    def rules(p: Policy) -> dict[str, dict]:
        return {c.id: c.model_dump(mode="json") for c in p.constraints}

    old, new = rules(previous), rules(policy)
    changed = {}
    for rid in sorted(old.keys() & new.keys()):
        a, b = old[rid], new[rid]
        fields = sorted(k for k in a.keys() | b.keys() if a.get(k) != b.get(k))
        if fields:
            changed[rid] = fields
    pa = previous.model_dump(mode="json", exclude={"constraints", "issued_at"})
    pb = policy.model_dump(mode="json", exclude={"constraints", "issued_at"})
    return {
        "against": {"policy_id": previous.policy_id, "version": previous.version,
                    "policy_hash": previous.policy_hash},
        "added": sorted(new.keys() - old.keys()),
        "removed": sorted(old.keys() - new.keys()),
        "changed": changed,
        "policy_fields": sorted(k for k in pa.keys() | pb.keys() if pa.get(k) != pb.get(k)),
    }


def diff_text(diff: dict[str, Any]) -> str:
    """One line a person reads for a policy_diff: what was added, removed and
    changed against which version."""
    a = diff["against"]
    parts = [f"against {a['policy_id']} {a['version']} ({a['policy_hash'][:12]})"]
    if diff["added"]:
        parts.append("added " + ", ".join(diff["added"]))
    if diff["removed"]:
        parts.append("removed " + ", ".join(diff["removed"]))
    for rid, fields in diff["changed"].items():
        parts.append(f"changed {rid} ({', '.join(fields)})")
    if diff["policy_fields"]:
        parts.append("policy " + ", ".join(diff["policy_fields"]))
    if len(parts) == 1:
        parts.append("no rule changed")
    return "; ".join(parts)


def _previous_policy(previous) -> Policy | None:
    """`previous` as a Policy: None, a Policy, a bundle (.tar.gz, read with
    the signature checks load_bundle makes) or a policy YAML/JSON file."""
    if previous is None or isinstance(previous, Policy):
        return previous
    p = Path(previous)
    if p.name.endswith(".tar.gz"):
        return load_bundle(p)
    from guardrail import load_policy
    return load_policy(p, runtime=False)


def manifest_for(policy: Policy, changelog: str = "initial", *,
                 issued_at: str | None = None,
                 signed_by: str | None = None,
                 ir_form: str = "ours",
                 previous: Policy | None = None,
                 multiscale_sha256: str | None = None) -> dict[str, Any]:
    """The bundle's identity card: the grant's six fields, plus how the hash was
    made (`hash_scheme`, `ir_schema_version`) and, for a published bundle, when
    it was issued. `issued_at=None` leaves the key out (a replay bundle embeds a
    policy it did not issue). `ir_form="reference"` describes a bundle whose
    ir.json is the reference implementation's document (see _ir_for).

    BOTH HASHES (2026-10-08, card WP1-05, deviation 10). `hashes` names the
    policy under both canonical forms: this project's (HASH_SCHEME, the one
    every stored run records) and the reference implementation's
    (REFERENCE_FORM, `Policy.reference_hash`). `policy_hash` is still the one
    the archived ir.json bytes hash to, so the reference's own loader - which
    reads `policy_hash` and nothing else - verifies a reference-form bundle,
    and `check_bundle` verifies both entries of `hashes`. Where the reference
    cannot represent the policy its entry is null and `reference_problems`
    says why: a hash of a document its loader would refuse is not written.
    A reference-form bundle also carries `reference_delta`, the record that
    rebuilds this project's policy from the reference's document exactly
    (Policy.from_reference). No stored hash moves: `policy_hash` of a bundle
    in this project's form is what it was.

    THE CHANGELOG AS A DIFF (2026-10-29, X-10). With `previous` (the version
    this one replaces) the manifest also carries `changelog_diff`
    (policy_diff), signed with the rest, and a default `changelog` of
    "initial" is replaced by its one-line reading (diff_text); a changelog
    written by hand is kept beside it. Without `previous` the manifest is
    exactly what it was, so no stored bundle changes.

    THE REFERENCE PROJECTION (2026-11-02, WP1-05). `reference_projection`
    is the second hash of the two-hash contract for every policy: the
    reference implementation's policy_hash of the document its Phase-1 DSL
    can hold - the policy's exactly-convertible rules
    (reference_convert.to_reference) - with the rules it holds, those left
    out and why, and `complete` when that document is the whole policy
    (then its hash is hashes[REFERENCE_FORM]). check_bundle recomputes it.

    THE MULTI-SCALE GEOMETRY (2026-11-02, WP2-16). `multiscale_sha256` is the
    digest of the bundle's multiscale.json, so the signature over the
    manifest covers the shipped scales too; absent on every manifest whose
    bundle ships none (replay bundles, and every bundle before that date)."""
    _, digest, scheme = _ir_for(policy, ir_form)
    ref = policy.reference_hash()
    m = {
        "policy_id": policy.policy_id,
        "version": policy.version,
        "generation": policy.generation,
        "policy_hash": digest,
        "hash_scheme": scheme,
        "hashes": {HASH_SCHEME: policy.policy_hash, REFERENCE_FORM: ref},
        "ir_schema_version": IR_SCHEMA_VERSION,
        "signed_by": signed_by or UNSIGNED,
        "changelog": changelog,
    }
    if previous is not None:
        diff = policy_diff(previous, policy)
        m["changelog_diff"] = diff
        if changelog == "initial":
            m["changelog"] = diff_text(diff)
    if ref is None:
        m["reference_problems"] = policy.reference_problems()
    # The reference's hash over what it CAN represent (WP1-05, the two-hash
    # contract; guardrail/reference_convert.py). Exact conversion only, in
    # the policy's own frame: the record names every rule left out and why,
    # and says whether the projection is the whole policy (`complete`).
    from .reference_convert import to_reference
    m["reference_projection"] = to_reference(policy).record()
    if ir_form == "reference":
        m["reference_delta"] = policy.reference_delta()
    if issued_at is not None:
        m["issued_at"] = issued_at
    if multiscale_sha256 is not None:
        m["multiscale_sha256"] = multiscale_sha256
    return m


def _same_instant(a: str, b: str) -> bool:
    return _issued_at(a) == _issued_at(b)


def _add(tar: tarfile.TarFile, name: str, data: bytes) -> None:
    info = tarfile.TarInfo(name=name)
    info.size = len(data)
    info.mtime = 0          # deterministic archive: same inputs -> same bytes
    tar.addfile(info, io.BytesIO(data))


def _read(tar: tarfile.TarFile, name: str) -> bytes:
    """Read a required member, or say which one is missing.

    `extractfile` raises KeyError for a name that is not in the archive and
    returns None for one that is not a regular file. Letting the KeyError escape
    would surface a truncated bundle as a bare `KeyError: "filename 'x' not
    found"` from inside tarfile, which tells the operator nothing about what
    they are holding.
    """
    try:
        member = tar.extractfile(name)
    except KeyError:
        member = None
    if member is None:
        raise ValueError(f"bundle missing required member: {name}")
    return member.read()


def write_bundle(policy: Policy, out_path: str | Path,
                 changelog: str = "initial", *, issued_at=None,
                 signer="auto", ir_form: str = "ours", previous=None) -> Path:
    """Write a tar.gz containing the canonical IR, the manifest and the signature.

    `previous`: the version this bundle replaces (a Policy, a bundle path,
    read with load_bundle's checks, or a policy file), recorded in the
    manifest as `changelog_diff` (manifest_for); None, no diff.

    `ir.json` is `policy.canonical_bytes()` - the bytes `policy_hash` digests,
    not a prettier rendering of them - so `sha256(ir.json)` IS the hash.
    `ir_form="reference"` writes the reference implementation's document and
    hash instead, so its loader can read the bundle (see the module docstring).

    `signer`: "auto" signs with the lab key when this machine has it (and
    writes an honestly UNSIGNED bundle when it does not); a `Signer` or a key
    path signs with that key; None writes unsigned on purpose.

    The issue time: a policy that WRITES `issued_at` (the grant's form) carries
    it into the manifest; an explicit `issued_at` that names another instant is
    refused, because the bundle would then contradict the document it ships.
    """
    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    sgn = _resolve_signer(signer)

    if policy.issued_at is not None:
        if issued_at is not None and not _same_instant(issued_at, policy.issued_at):
            raise ValueError(
                f"{policy.policy_id}: the policy says issued_at {policy.issued_at}, "
                f"the bundle was asked for {issued_at}; a bundle may not contradict "
                f"the document it carries")
        issued_at = policy.issued_at
    ir_bytes, digest, _ = _ir_for(policy, ir_form)
    # The scales are computed once, here at ingest, by the IR (the only place
    # that computes them) and shipped; readers verify, never recompute.
    from .ir import multiscale_document                 # lazy: shapely
    ms_bytes = canonical_json(multiscale_document(policy))
    manifest = manifest_for(policy, changelog, issued_at=_issued_at(issued_at),
                            signed_by=sgn.identity if sgn else UNSIGNED,
                            ir_form=ir_form, previous=_previous_policy(previous),
                            multiscale_sha256="sha256:" + hashlib.sha256(ms_bytes).hexdigest())
    manifest_bytes = json.dumps(manifest, indent=2, sort_keys=True).encode()
    signature = signature_text(digest, sgn, manifest_bytes)

    # Reproducibility needs all three of these, and the first two are not
    # enough on their own:
    #
    #   * member mtimes pinned to 0 (see _add);
    #   * mtime=0 on the GZIP WRAPPER - `tarfile.open(path, "w:gz")` stamps the
    #     current time into the gzip header, so two builds of an unchanged
    #     policy differ in their first few bytes;
    #   * filename="" - GzipFile otherwise copies the OUTPUT PATH into the
    #     header, so the same policy written to a.tar.gz and b.tar.gz produces
    #     different archives. That one is invisible until you compare two
    #     bundles built under different names, which is exactly what a
    #     reviewer re-deriving a delivered bundle would do.
    #
    # Since 2026-10-06 the manifest also carries issued_at, so "same policy ->
    # same bytes" holds for the same issue time (pass issued_at, or set
    # SOURCE_DATE_EPOCH). ir.json and policy_hash never depend on it.
    with open(out, "wb") as fh:
        with gzip.GzipFile(fileobj=fh, mode="wb", mtime=0, filename="") as gz:
            with tarfile.open(fileobj=gz, mode="w") as tar:
                _add(tar, IR_NAME, ir_bytes)
                _add(tar, MANIFEST_NAME, manifest_bytes)
                _add(tar, SIGNATURE_NAME, signature)
                _add(tar, MULTISCALE_NAME, ms_bytes)
    return out


# --------------------------------------------------------------------------- #
# Reading
# --------------------------------------------------------------------------- #

@dataclass
class BundleCheck:
    """Everything `check_bundle` established about one bundle."""
    path: Path
    policy: Policy
    manifest: dict
    hash_form: str              # HASH_SCHEME, or the legacy form's name
    signature: str              # one of the SIG_* statuses
    signed_by: str
    detail: str
    # Every hash form the manifest names and this check VERIFIED, form ->
    # hash (None: the manifest records that the form cannot represent the
    # policy, and that was checked too). Bundles written before 2026-10-08
    # name one form, `hash_form`.
    hashes: dict = field(default_factory=dict)
    # A reference-form bundle with a `reference_delta`: this project's policy
    # rebuilt from it, its hash verified against `hashes[HASH_SCHEME]`.
    source_policy: Policy | None = None

    @property
    def verified(self) -> bool:
        return self.signature == SIG_VERIFIED

    @property
    def flight_policy(self) -> Policy | None:
        """The policy a flight loader hands the Shield: the rebuilt policy for
        a bundle in this project's form; for a reference-form bundle the
        source policy its delta rebuilds (whose hash the manifest records), or
        None when it carries no delta - see _refuse_for_flight."""
        if self.hash_form == REFERENCE_FORM:
            return self.source_policy
        return self.policy

    def source_record(self) -> dict:
        """What a flight writes about where its policy came from.

        `declared_policy_hash` is the hash the bundle's manifest (and its
        signature) names; `loaded_policy_hash` is the hash of the policy
        object the flight gets. They are equal for every bundle this project
        writes in its own form, and for the legacy forms `hash_form` names the
        relation. A reference-form bundle declares the reference's hash and
        flies this project's policy rebuilt from it, whose hash the manifest
        names in `hashes` (both verified). Both are recorded so a run never
        has to infer one."""
        flown = self.flight_policy or self.policy
        return {"kind": "bundle", "path": str(self.path),
                "loaded_policy_hash": flown.policy_hash,
                "declared_policy_hash": self.manifest.get("policy_hash"),
                "hash_form": self.hash_form,
                "hashes": dict(self.hashes),
                "issued_at": self.manifest.get("issued_at"),
                "signed_by": self.signed_by,
                "signature": self.signature,
                "signature_detail": self.detail}


def check_bundle(path: str | Path,
                 trust_store: str | Path | dict | None = None) -> BundleCheck:
    """Open a bundle, prove its integrity, and report its signature status.

    Raises ValueError for anything that means "this is not the policy that was
    issued": a missing member, an IR whose bytes do not hash to the manifest's
    policy_hash, an IR that does not rebuild to the same policy (an unknown
    key the model would silently drop, or a non-canonical encoding - checked
    for v2 and legacy bundles alike), a signature line naming another hash or
    signer, an Ed25519 signature that is present and WRONG, or one stripped
    from a bundle that still names its key. Returns a BundleCheck for
    everything else, with the signature status stated - including "unsigned",
    "untrusted-signer" and "unverifiable-here", which are facts, not passes.
    """
    p = Path(path)
    with tarfile.open(p, "r:gz") as tar:
        ir_bytes = _read(tar, IR_NAME)
        manifest_bytes = _read(tar, MANIFEST_NAME)
        sig_text = _read(tar, SIGNATURE_NAME).decode()
        try:
            ms_bytes = _read(tar, MULTISCALE_NAME)
        except ValueError:
            ms_bytes = None
    manifest = json.loads(manifest_bytes)
    declared = manifest.get("policy_hash")

    # model_validate, not load_policy: the IR is the validated document, and
    # the YAML loader's file handling has no business here. Geographic points
    # are projected again by the model, deterministically, in the same frame.
    # Since 2026-10-06 the model refuses an unknown key outright (it used to
    # drop it, and the hash comparison below was what caught that).
    try:
        policy = Policy.model_validate(json.loads(ir_bytes))
    except ValueError as exc:
        first = str(exc).strip().splitlines()
        raise ValueError(
            f"{p.name}: ir.json holds content this code cannot represent as a "
            f"policy IR, refusing rather than enforcing a different policy: "
            f"{' / '.join(first[:3])}") from None

    digest = "sha256:" + hashlib.sha256(ir_bytes).hexdigest()
    if declared == digest:
        # The bytes are what was hashed. Now prove the OBJECT is too: an IR
        # whose bytes are fine can still rebuild into a different policy (a
        # non-canonical encoding, a coerced value).
        ir2 = {h: n for n, h in policy.ir2_hashes().items()}
        if policy.policy_hash == declared:
            form = HASH_SCHEME
        elif declared in ir2:
            # A bundle written under IR schema 2.x (before 2026-11-02): its
            # ir.json spells project_fix `repair` and keeps rings as written.
            # `declared` is the SHA-256 of the archived bytes (checked above)
            # AND of the 2.x view of the policy rebuilt from them, so the two
            # byte strings are the same, as for the reference form below.
            form = ir2[declared]
        elif declared in (policy.reference_hash(), policy.reference_hash(given=True)):
            # The reference implementation's bundle (WP1-05). `declared` is
            # the SHA-256 of the archived bytes (checked just above) AND of
            # canonical_json(reference_ir()), the document rebuilt from the
            # model - so the two byte strings are the same and nothing in
            # ir.json went unread. A separate byte comparison stood here until
            # 2026-10-07; it could not fail without a SHA-256 collision, and
            # a check that cannot fail is not kept as if it were one.
            form = REFERENCE_FORM
        else:
            raise ValueError(
                f"{p.name}: ir.json hashes to the manifest's policy_hash, but "
                f"rebuilds to {policy.policy_hash}: it holds content this code "
                f"cannot represent (an unknown key, or a non-canonical "
                f"encoding). Refusing rather than enforcing a different policy.")
    else:
        # A bundle written before 2026-10-06: 16-hex hash over the old dump.
        form = policy.hash_form(declared)
        if form == SHORT_FORM:
            raise ValueError(
                f"{p.name}: the manifest declares {declared}, the 16-hex display "
                f"form of the policy's hash; no bundle format records it, so the "
                f"bundle was not written by this code. Refusing.")
        if form in (HASH_SCHEME, PRIOR_SCHEME_V2, REFERENCE_FORM, *IR2_FORMS):
            # Same policy, different bytes: someone re-serialised ir.json. The
            # object is right but the artefact no longer is what was hashed and
            # signed, and a reader checking it with sha256sum would be told no.
            raise ValueError(
                f"{p.name}: ir.json is not the canonical encoding of its policy "
                f"(its bytes hash to {digest}, not {declared}); it was "
                f"re-serialised after issue.")
        if form is None:
            raise ValueError(
                f"{p.name}: policy_hash mismatch - manifest says {declared}, the "
                f"IR in this bundle hashes to {policy.policy_hash}. The bundle "
                f"is corrupt or was edited.")
        # A legacy hash digests the REBUILT object, not the archived bytes, so
        # a key pydantic drops (a typo, an injected field) would pass under it
        # unseen. The v2 branch catches that by hashing the bytes; here the
        # archived content must equal what the policy dumps back to (null
        # meaning absent on both sides, as in every legacy form).
        # Both sides in the IR 3.0.0 view: the archived 2.x rules spell
        # `repair` and keep rings as written (models.ir2_rule_view).
        archived = _prune_nulls(json.loads(ir_bytes), frozenset())
        if isinstance(archived, dict) and isinstance(archived.get("constraints"), list):
            archived["constraints"] = [ir2_rule_view(r) if isinstance(r, dict) else r
                                       for r in archived["constraints"]]
        if archived != _prune_nulls(policy.model_dump(mode="json"), frozenset()):
            raise ValueError(
                f"{p.name}: ir.json carries content its policy does not "
                f"represent (an unknown key, or a value the model coerced). "
                f"The {form} hash cannot see it; refusing rather than "
                f"enforcing a different policy than the one archived.")

    # A bundle is a release of a DSL-valid policy: the same lint load_policy
    # runs (duplicate ids, no rules, conflicting hard limits, ...).
    lint = policy.lint()
    if lint:
        raise ValueError(f"{p.name}: the policy in this bundle does not pass the "
                         f"DSL lint: " + "; ".join(lint))

    hashes, source = _verify_both_hashes(p.name, manifest, ir_bytes, policy, form,
                                         declared)
    _attach_multiscale(p.name, manifest, ms_bytes, policy, source, form)
    _verify_projection(p.name, manifest, source or policy, hashes)

    sig_hash, signed_by, sig_hex = parse_signature(sig_text)
    if sig_hash != declared:
        raise ValueError(
            f"{p.name}: signature covers {sig_hash!r} but the manifest declares "
            f"{declared!r} - the manifest does not belong to this signature.")
    if manifest.get("signed_by") and manifest["signed_by"] != signed_by:
        raise ValueError(
            f"{p.name}: manifest says signed_by {manifest['signed_by']!r} but "
            f"the signature line names {signed_by!r}.")
    status, detail = verify_signature(signed_by, sig_hex, manifest_bytes, trust_store)
    if status == SIG_BAD:
        why = (" The manifest was altered after signing, or the signature "
               "belongs to another bundle." if "verification failed" in detail
               else " Refusing: this is not a layout any signer here writes.")
        raise ValueError(f"{p.name}: {detail}.{why}")
    return BundleCheck(path=p, policy=policy, manifest=manifest, hash_form=form,
                       signature=status, signed_by=signed_by, detail=detail,
                       hashes=hashes, source_policy=source)


def multiscale_problems(doc, policy: Policy) -> list[str]:
    """Why a bundle's multiscale.json does not describe `policy`, or [].

    Checks, not a recomputation (so a reader needs no shapely and no
    agreement with the writer's GEOS build): the document names this policy
    (hash, generation); every zone and tube of the policy has an entry and
    nothing else does; a zone's fine scale IS its vertices and a tube's IS its
    centerline, float for float; a zone's bbox IS the bounding rectangle of
    those vertices; its coarse ring is convex and contains every vertex
    (boundary included), so - being convex - it contains the whole zone. A
    coarse ring that cut a corner would tell the model a smaller zone than
    the Shield enforces."""
    from .models import _dist_to_ring, _point_in_ring
    from .ir import FENCE_TYPES, MULTISCALE_FORMAT, TUBE_TYPES
    out: list[str] = []
    if not isinstance(doc, dict) or doc.get("format") != MULTISCALE_FORMAT:
        return [f"not a {MULTISCALE_FORMAT} document"]
    if doc.get("policy_hash") != policy.policy_hash or \
            doc.get("generation") != policy.generation:
        return [f"it describes {doc.get('policy_hash')} g{doc.get('generation')}, not "
                f"this policy ({policy.policy_hash} g{policy.generation})"]
    rules = doc.get("rules")
    if not isinstance(rules, dict):
        return ["no `rules` mapping"]
    want = {c.id: c for c in policy.constraints if c.type in FENCE_TYPES + TUBE_TYPES}
    for rid in sorted(set(rules) ^ set(want)):
        out.append(f"{rid}: " + ("no entry" if rid in want else "no such zone or tube"))

    def pts(v):
        return [(float(x), float(y)) for x, y in v]
    for rid in sorted(set(rules) & set(want)):
        c, ent = want[rid], rules[rid]
        try:
            if c.type in TUBE_TYPES:
                if set(ent) != {"fine"} or pts(ent["fine"]) != [tuple(p) for p in c.points()]:
                    out.append(f"{rid}: the fine scale is not the centerline")
                continue
            if set(ent) != {"fine", "coarse", "bbox"}:
                out.append(f"{rid}: scales {sorted(ent)}, not fine / coarse / bbox")
                continue
            fine = pts(ent["fine"])
            if fine != [(float(v.x), float(v.y)) for v in c.vertices]:
                out.append(f"{rid}: the fine scale is not the zone's vertices")
                continue
            xs, ys = [p[0] for p in fine], [p[1] for p in fine]
            box = [(min(xs), min(ys)), (max(xs), min(ys)), (max(xs), max(ys)),
                   (min(xs), max(ys))]
            if pts(ent["bbox"]) != box:
                out.append(f"{rid}: the bbox is not the vertices' bounding rectangle")
            coarse = pts(ent["coarse"])
            n = len(coarse)
            turns = [(coarse[(i + 1) % n][0] - coarse[i][0]) * (coarse[(i + 2) % n][1] - coarse[i][1])
                     - (coarse[(i + 1) % n][1] - coarse[i][1]) * (coarse[(i + 2) % n][0] - coarse[i][0])
                     for i in range(n)] if n >= 3 else []
            if n < 3 or not (all(t >= -1e-9 for t in turns) or all(t <= 1e-9 for t in turns)):
                out.append(f"{rid}: the coarse ring is not convex")
            # inside or on it; 1 micrometre allows the float rounding of a
            # minimum rotated rectangle's corners
            elif not all(_point_in_ring(x, y, coarse) or _dist_to_ring(x, y, coarse) <= 1e-6
                         for x, y in fine):
                out.append(f"{rid}: the coarse ring does not contain the zone")
        except (TypeError, ValueError, KeyError):
            out.append(f"{rid}: unreadable entry")
    return out


def _verify_projection(name: str, manifest: dict, policy: Policy, hashes: dict) -> None:
    """The manifest's `reference_projection` (manifest_for) is what the
    converter gives for the policy this bundle flies, exactly; and when it
    claims to be the whole policy, its hash is the reference form's. Absent on
    a bundle written before 2026-11-02."""
    rec = manifest.get("reference_projection")
    if rec is None:
        return
    from .reference_convert import to_reference
    want = to_reference(policy).record()
    if rec != want:
        diff = sorted(k for k in set(rec) | set(want) if rec.get(k) != want.get(k)) \
            if isinstance(rec, dict) else ["(not a mapping)"]
        raise ValueError(f"{name}: the manifest's reference_projection does not "
                         f"describe the policy in this bundle (differs in {diff})")
    if want["complete"] and REFERENCE_FORM in hashes and \
            hashes[REFERENCE_FORM] not in (None, want["policy_hash"]):
        raise ValueError(f"{name}: the complete reference projection hashes to "
                         f"{want['policy_hash']}, the manifest's reference form to "
                         f"{hashes[REFERENCE_FORM]}")


def _attach_multiscale(name: str, manifest: dict, ms_bytes: bytes | None,
                       policy: Policy, source: Policy | None, form: str) -> None:
    """Verify a bundle's multiscale.json and hand it on with the policy it
    describes (`Policy.shipped_multiscale`), or refuse. A member the manifest
    does not name is unsigned content, and is refused too. A reference-form
    bundle whose delta was dropped describes a source policy nothing here
    rebuilds: its digest is still verified, and the geometry is not used
    (such a bundle is an exchange format, refused for flight)."""
    rec = manifest.get("multiscale_sha256")
    if rec is None:
        if ms_bytes is not None:
            raise ValueError(f"{name}: carries {MULTISCALE_NAME} but the manifest "
                             f"names no digest for it; unsigned geometry is refused")
        return
    if ms_bytes is None:
        raise ValueError(f"{name}: the manifest names {MULTISCALE_NAME} "
                         f"({str(rec)[:23]}...) but the bundle has none")
    got = "sha256:" + hashlib.sha256(ms_bytes).hexdigest()
    if got != rec:
        raise ValueError(f"{name}: {MULTISCALE_NAME} hashes to {got}, the manifest "
                         f"says {rec}: edited after issue")
    doc = json.loads(ms_bytes)
    target = source if (source is not None and isinstance(doc, dict)
                        and doc.get("policy_hash") == source.policy_hash) else policy
    if form == REFERENCE_FORM and source is None:
        return
    bad = multiscale_problems(doc, target)
    if bad:
        raise ValueError(f"{name}: {MULTISCALE_NAME} does not describe the policy "
                         f"in this bundle: " + "; ".join(bad[:5]))
    target._shipped_multiscale = doc["rules"]


def _verify_both_hashes(name: str, manifest: dict, ir_bytes: bytes, policy: Policy,
                        form: str, declared: str) -> tuple[dict, Policy | None]:
    """Check every entry of the manifest's `hashes` (see manifest_for) against
    the policy this bundle rebuilds; return the verified forms and, for a
    reference-form bundle with a delta, this project's policy rebuilt from it.

    `declared` (the manifest's `policy_hash`) was verified by the caller. A
    manifest without `hashes` was written before 2026-10-08 and names one
    form only. One that has it must name both forms: an entry dropped from a
    manifest would otherwise read as "not recorded" and pass unchecked.

      this project's form   hashes[HASH_SCHEME] is the declared hash, and
                            hashes[REFERENCE_FORM] is what the rebuilt policy's
                            reference_hash() gives - null exactly when the
                            reference cannot represent it;
      the reference's form  hashes[REFERENCE_FORM] is the declared hash, and
                            the source policy rebuilt with `reference_delta`
                            (Policy.from_reference, which also proves it writes
                            back the archived document) hashes to
                            hashes[HASH_SCHEME].
    """
    rec = manifest.get("hashes")
    if rec is None:
        return {form: declared}, None
    if not isinstance(rec, dict) or set(rec) != {HASH_SCHEME, REFERENCE_FORM}:
        raise ValueError(
            f"{name}: the manifest's `hashes` must name exactly the two forms "
            f"{HASH_SCHEME} and {REFERENCE_FORM}; it names "
            f"{sorted(rec) if isinstance(rec, dict) else type(rec).__name__}. An "
            f"entry dropped from the record would pass unchecked.")
    if form == REFERENCE_FORM:
        if rec[REFERENCE_FORM] != declared:
            raise ValueError(f"{name}: hashes[{REFERENCE_FORM}] is "
                             f"{rec[REFERENCE_FORM]}, but the bundle's policy_hash "
                             f"is {declared}")
        if "reference_delta" not in manifest:
            raise ValueError(f"{name}: a reference-form manifest that records "
                             f"{HASH_SCHEME} must carry the reference_delta that "
                             f"rebuilds that policy")
        try:
            source = Policy.from_reference(json.loads(ir_bytes), manifest["reference_delta"])
        except ValueError as exc:
            raise ValueError(f"{name}: reference_delta does not rebuild a policy "
                             f"from this document: {str(exc).splitlines()[0]}") from None
        # Written under IR 2.x, the record names the source's 2.x hash.
        if source.hash_form(rec[HASH_SCHEME]) not in (HASH_SCHEME, *IR2_FORMS):
            raise ValueError(
                f"{name}: hashes[{HASH_SCHEME}] is {rec[HASH_SCHEME]}, but the "
                f"policy rebuilt from the reference document and its delta "
                f"hashes to {source.policy_hash}")
        return {HASH_SCHEME: rec[HASH_SCHEME], REFERENCE_FORM: declared}, source
    if form not in (HASH_SCHEME, *IR2_FORMS):
        raise ValueError(f"{name}: a {form} bundle cannot carry the 2026-10-08 "
                         f"`hashes` record; it was written before that form "
                         f"existed, so the record was added afterwards")
    if rec[HASH_SCHEME] != declared:
        raise ValueError(f"{name}: hashes[{HASH_SCHEME}] is {rec[HASH_SCHEME]}, "
                         f"but the bundle's policy_hash is {declared}")
    ref = policy.reference_hash()
    if form in IR2_FORMS and rec[REFERENCE_FORM] is not None and \
            rec[REFERENCE_FORM] == policy.reference_hash(given=True):
        # An IR 2.x bundle recorded the reference hash of its rings as written.
        ref = rec[REFERENCE_FORM]
    if rec[REFERENCE_FORM] != ref:
        raise ValueError(
            f"{name}: hashes[{REFERENCE_FORM}] is {rec[REFERENCE_FORM]}, but the "
            f"policy in this bundle "
            + (f"hashes to {ref} under the reference's canonical form"
               if ref is not None else "cannot be represented by the reference")
            + ". The manifest does not describe this policy.")
    return {HASH_SCHEME: declared, REFERENCE_FORM: ref}, None


def _refuse_for_flight(chk: BundleCheck, name: str) -> None:
    """What no flight loader accepts from a bundle, whatever its signature.

    1. A rule the Shield would not enforce as written (models.
       _refuse_unenforced: a declarable-only type, MSL, a derived frame).
    2. A bundle in the reference implementation's IR form WITHOUT a
       `reference_delta` (the reference's own bundles, and any written here
       before 2026-10-08). Its ir.json writes every default out (scope, layer,
       altitude_ref, margin 0) and spells `repair` as `project_fix`, so the
       policy rebuilt from it hashes to a THIRD value - neither the manifest's
       hash nor the source policy's. A run flown from it would record a
       loaded_policy_hash that resolves to no policy (2026-10-07 review).
       Since 2026-10-08 a reference-form bundle written here carries the
       delta and this project's hash beside the reference's, both verified by
       check_bundle; it flies the source policy, under the hash its manifest
       names, and the reference's own loader reads the same file."""
    from .models import _refuse_unenforced
    if chk.hash_form == REFERENCE_FORM and chk.source_policy is None:
        raise ValueError(
            f"{name}: a bundle in the reference implementation's IR form "
            f"({REFERENCE_FORM}) with no reference_delta is an exchange format, "
            f"not a flight artefact: the policy rebuilt from it hashes to "
            f"{chk.policy.policy_hash[:23]}..., not the manifest's "
            f"{str(chk.manifest.get('policy_hash'))[:23]}..., so a run could not "
            f"be traced back to it. Write the bundle with this code (`python -m "
            f"guardrail.bundle POLICY`, either --ir-form) to fly it.")
    _refuse_unenforced(chk.flight_policy, name)


def load_bundle(path: str | Path, *, require_signature: bool = True,
                trust_store: str | Path | dict | None = None,
                runtime: bool = True) -> Policy:
    """Load a bundle, rebuild the Policy, and REFUSE it if it is not the policy
    that was issued.

    Rebuilding from the canonical IR is not re-parsing the DSL: the IR is the
    post-validation single source of truth, which is the whole point of
    shipping it rather than the YAML.

    With `require_signature=True` (the default) a bundle whose signature is
    anything but verified - unsigned, untrusted, or unverifiable - raises
    BundleSignatureError. Passing False loads it anyway; use `check_bundle`
    instead when the status must be recorded, which it should be. A bad or
    stripped signature raises ValueError either way.

    `runtime=True` (the default) applies the same flight gate as
    `load_policy` and `load_for_flight`: a bundle holding a rule the Shield
    would not enforce (a declarable-only type, MSL, a derived frame), or one
    in the reference's form without a reference_delta, is refused; a
    reference-form bundle with one returns the source policy it rebuilds
    (`BundleCheck.flight_policy`). Until 2026-10-07 this
    function skipped that gate, so a bundle with a dynamic_nfz loaded here and
    the Shield then ignored the zone without a word - and
    guardrail/scenario_spec.py loads its `bundle_path` scenarios through it.
    `runtime=False` returns the declaration, for tools that only read it.

    The signature default changed on 2026-10-06 (it used to accept the
    placeholder signer). A caller that writes a bundle and reads it straight
    back on a machine without the lab key - tools/build_deck_data.py does -
    gets an UNSIGNED bundle and must say `require_signature=False` explicitly.
    """
    chk = check_bundle(path, trust_store)
    if runtime:
        _refuse_for_flight(chk, Path(path).name)
    if require_signature and not chk.verified:
        raise BundleSignatureError(
            f"{Path(path).name}: signature {chk.signature}: {chk.detail}. "
            f"Refusing to treat it as signed. Load with "
            f"require_signature=False only if the run is recorded as UNSIGNED.")
    return chk.flight_policy if runtime else chk.policy


def load_for_flight(bundle: str | Path | None = None,
                    yaml_path: str | Path | None = None, *,
                    allow_unverified: bool = False,
                    trust_store: str | Path | dict | None = None
                    ) -> tuple[Policy, dict]:
    """The one way a flight entry point gets its policy: `(policy, source)`.

    Audit cards ARCH-29 / WP3-21: every flight script built its rules straight
    from YAML, so no run could show which signed bundle was in force - only a
    hash. Now `--bundle` loads the issued artefact (integrity always enforced;
    a signature that is not VERIFIED refuses the flight unless
    `allow_unverified`, and then the record says exactly what it was), and the
    YAML path stays as the fallback, recorded as unsigned. Never both: a run
    with two candidate policies has none.

    `source` is written by the caller beside the manifest - metrics.json,
    kpi.json, the replay index - and its `loaded_policy_hash` is the hash at
    take-off, before any mid-flight hot-apply.

    Both paths refuse a policy holding a rule the Shield would not enforce (a
    declarable-only class, an MSL altitude, a derived frame;
    `Policy.flight_problems`), the bundle path also refuses the reference's
    exchange form (`_refuse_for_flight`), and both record as `runtime_notes`
    the breach actions the Shield cannot complete on its own (a flight mode
    it only requests, a soft rule's action capped at brake) and a start
    inside a hard keep-out zone (`Policy.runtime_notes`), so a run never reads
    as having honoured something it did not.
    """
    from .manifest import policy_source_record
    if bundle and yaml_path:
        raise ValueError("give a policy bundle or a YAML policy, not both - a "
                         "run with two candidate policies cannot say which flew")
    if bundle:
        chk = check_bundle(bundle, trust_store)
        _refuse_for_flight(chk, Path(bundle).name)
        if not chk.verified and not allow_unverified:
            raise BundleSignatureError(
                f"{Path(bundle).name}: signature {chk.signature}: {chk.detail}. "
                f"Not flying an unverified bundle as if signed; pass "
                f"--allow-unverified-bundle to fly it recorded as such.")
        rec = chk.source_record()
        if not chk.verified:
            rec["accepted_unverified"] = True
        pol = chk.flight_policy
    else:
        if yaml_path is None:
            raise ValueError("no policy given: pass a bundle or a YAML file")
        pol = load_policy(yaml_path)
        rec = policy_source_record("yaml", str(yaml_path), pol.policy_hash)
    notes = pol.runtime_notes()
    if notes:
        rec["runtime_notes"] = notes
    return pol, rec


# --------------------------------------------------------------------------- #
# The version lock: a policy's version must change whenever its content does
# --------------------------------------------------------------------------- #

def lock_key(policy: Policy) -> str:
    return f"{policy.policy_id}@{policy.version}"


def _entry(policy: Policy) -> dict:
    """A new lock entry. `hash_scheme` names the canonical scheme its
    policy_hash was made under; entries written before 2026-10-07 carry none
    and are under the lock's top-level `hash_scheme` (v2) - see its `_doc`."""
    return {"policy_hash": policy.policy_hash,
            "hash_scheme": HASH_SCHEME,
            "ir_schema_version": IR_SCHEMA_VERSION,
            "legacy_hashes": policy.legacy_hashes()}


# --------------------------------------------------------------------------- #
# Migrations: an IR schema change that moves a policy's hash (IR 3.0.0)
# --------------------------------------------------------------------------- #
#
# The lock never rewrites an entry, and an id@version names one content hash
# forever. So when an IR change moves a policy's canonical bytes without
# changing what it says (3.0.0: `repair` stored as project_fix, rings in
# canonical order), the file RAISES its version and the new id@version is
# locked beside the old one, with `migrated_from: <old id@version>`. The old
# entry keeps the hash every run, bundle and manifest of that time recorded.
#
# The claim is checked, not trusted: the file's policy, put back at the old
# version (`as_version`), must reproduce the old entry's recorded hash under
# an IR 2.x form (models.Policy.ir2_hashes) or a prior scheme, and its legacy
# 16-hex forms. That is how "old hashes still resolve" holds for every
# migrated policy: `policy_candidates` offers the rewound policy too, so
# `guardrail.manifest.resolve_run_policy` finds a run flown under the old
# version, and a lock `derived` recipe based on it still rebuilds.

MIGRATION_FORMS = (PRIOR_SCHEME_V2, *IR2_FORMS)


def as_version(policy: Policy, version: str) -> Policy:
    """`policy` with another `version` and nothing else changed (its rules,
    with the ring order they were given, are shared)."""
    return policy.model_copy(update={"version": version})


def _pin_for(ent: dict, fname: str) -> dict | None:
    """The pin a lock entry holds for one file: the entry itself, or the
    file's own pin inside a shared entry; None when it pins another file."""
    if "shared_by_files" in ent:
        return (ent.get("shared_by_files") or {}).get(fname)
    return ent if ent.get("file") == fname and "policy_hash" in ent else None


def migration_problem(pol: Policy, old_key: str, pin: dict | None) -> str | None:
    """Why `pol` is NOT the content locked as `old_key` (`pin`), migrated to
    the current IR - or None when it is: put back at the old version it
    reproduces the old recorded hash under a migration form, and the old
    legacy forms."""
    if pin is None:
        return f"{old_key} pins no hash for this file"
    pid, _, ver = old_key.rpartition("@")
    if pid != pol.policy_id:
        return f"{old_key} is another policy id than {pol.policy_id}"
    old = as_version(pol, ver)
    form = old.hash_form(pin.get("policy_hash"))
    if form not in MIGRATION_FORMS:
        return (f"rewound to {ver} it does not reproduce {old_key}'s hash "
                f"{str(pin.get('policy_hash'))[:23]}... under any migration form "
                f"(got {form or 'no form'})")
    later = (pin.get("hashes") or {}).get(HASH_SCHEME)
    if later is not None and old.hash_form(later) not in MIGRATION_FORMS:
        return f"rewound to {ver} it does not reproduce {old_key}'s {HASH_SCHEME} hash"
    if pin.get("legacy_hashes") is not None and pin["legacy_hashes"] != old.legacy_hashes():
        return f"rewound to {ver} it does not reproduce {old_key}'s legacy hashes"
    return None


def _migration_source(pol: Policy, fname: str, entries: dict, key: str) -> tuple | None:
    """(old key, form) of the lock entry this file's content migrates, or None.
    The highest old version that reproduces wins."""
    found = []
    for k, e in entries.items():
        if k == key:
            continue
        pin = _pin_for(e, fname)
        if pin is not None and migration_problem(pol, k, pin) is None:
            ver = k.rpartition("@")[2]
            found.append((_semver_key(ver), k, as_version(pol, ver).hash_form(pin["policy_hash"])))
    if not found:
        return None
    _, k, form = max(found)
    return k, form


def load_lock(path: str | Path | None = None) -> dict:
    p = Path(path) if path else LOCK_PATH
    return json.loads(p.read_text(encoding="utf-8")) if p.is_file() else {}


def _policy_files(policies_dir: str | Path | None = None) -> list[Path]:
    d = Path(policies_dir) if policies_dir else ROOT / "policies"
    return sorted(d.glob("*.yaml"))


# What a lock entry says about a policy's content, given its recorded hash.
_PINNED, _RECORD_NEW_FORM, _CHANGED = "pinned", "record-new-form", "changed"


def _pin_status(pol: Policy, ent: dict) -> str:
    """Whether a lock entry (or one file's pin inside a shared entry) still
    pins `pol`'s content.

    An entry is never rewritten, so an entry locked under an earlier
    full-length hash scheme keeps that hash. Hash scheme v3 (2026-10-06)
    changed the bytes of geographic policies only; for those the entry also
    carries `hashes: {<scheme>: <hash>}`, added by `lock --update` and never
    replacing the original. Same content under a newer scheme is not a
    content change - but it is reported until the new form is recorded, so the
    lock always names the hash a current bundle carries."""
    rec = ent.get("policy_hash")
    if rec == pol.policy_hash:
        return _PINNED
    if pol.hash_form(rec) in pol.prior_hashes():
        later = (ent.get("hashes") or {}).get(HASH_SCHEME)
        if later is None:
            return _RECORD_NEW_FORM
        return _PINNED if later == pol.policy_hash else _CHANGED
    return _CHANGED


def _new_form_msg(name: str, key: str, ent: dict, pol: Policy) -> str:
    return (f"{name}: {key} is locked under {pol.hash_form(ent['policy_hash'])} "
            f"({ent['policy_hash'][:23]}...); the same content hashes to "
            f"{pol.policy_hash[:23]}... under {HASH_SCHEME}, not yet recorded - "
            f"run `python -m guardrail.bundle lock --update` (additive: the old "
            f"hash stays)")


def ir_schema_drift(locked: dict, current: dict | None = None) -> list[str]:
    """Every way today's IR schema differs from the pinned one. Empty = the
    shape is the pinned shape.

    THE FREEZE RULE (grant, Policy DSL page, R6: the IR shape freezes at the
    end of Q1 and "every later IR-shape change carries a semver bump and a
    migration note"). Since 2026-11-02 every difference is reported, and
    `check_lock` refuses it until IR_SCHEMA_VERSION is raised and the lock
    re-pinned (`update_lock`): a removed model or field, a changed default
    (every policy that omits the field now means something else), a new
    field with a value default (it enters every canonical IR and moves every
    hash) - and, the change this rule tightened, a NEW field defaulting to
    None ("<absent>") or a new MODEL. Those two move no stored hash (the
    canonical form omits None, and no existing policy uses a new type), and
    until 2026-11-02 they passed unbumped under this project's own rule; the
    grant makes no such exception, and `locked_by` (2385f84) went in that way.
    """
    from .models import ir_schema_defaults
    cur = ir_schema_defaults() if current is None else current
    out: list[str] = []
    for model, fields in sorted(locked.items()):
        now = cur.get(model)
        if now is None:
            out.append(f"{model}: removed from the IR schema")
            continue
        for name, was in sorted(fields.items()):
            if name not in now:
                out.append(f"{model}.{name}: removed (was {was!r})")
            elif now[name] != was and now[name] == "<required>":
                out.append(f"{model}.{name}: default {was!r} -> '<required>'; "
                           f"every policy that omits it is refused")
            elif now[name] != was:
                out.append(f"{model}.{name}: default {was!r} -> {now[name]!r}; "
                           f"every policy that omits it changes meaning")
        for name in sorted(set(now) - set(fields)):
            if now[name] != "<absent>":
                out.append(f"{model}.{name}: new field with default "
                           f"{now[name]!r} - it enters every canonical IR "
                           f"and moves every hash")
            else:
                out.append(f"{model}.{name}: {NEW_ABSENT_FIELD} (moves no "
                           f"stored hash, but is an IR-shape change)")
    # A new MODEL (a new constraint type) moves no stored hash either. But until
    # it is pinned its defaults can change unseen - the review of 2026-10-06
    # found exactly that hole behind a hand-written class list.
    for model in sorted(set(cur) - set(locked)):
        out.append(f"{model}: {UNPINNED_MODEL}")
    return out


UNPINNED_MODEL = "in the IR schema but not pinned"
NEW_ABSENT_FIELD = "new field defaulting to None"


def _semver_key(v: str | None) -> tuple:
    """(major, minor, patch) of a plain semver string; () when unreadable."""
    try:
        parts = tuple(int(x) for x in str(v).split("-")[0].split("+")[0].split("."))
    except ValueError:
        return ()
    return parts if len(parts) == 3 else ()


def ir_schema_problems(lock: dict, current: dict | None = None) -> list[str]:
    """What check_lock says about the IR schema pin (the freeze rule; see
    ir_schema_drift). The pin names the version it was made under
    (`ir_schema.version`); the code names its own (IR_SCHEMA_VERSION).

      same version, same shape        clean;
      same version, another shape     every difference, each one an IR-shape
                                      change made without a version bump;
      a higher version                not yet re-pinned: `lock --update`
                                      records the change and re-pins;
      a lower or unreadable version   refused (a version never goes back)."""
    irs = lock.get("ir_schema") or {}
    pinned = irs.get("defaults")
    if pinned is None:
        return ["the lock does not pin the IR schema (ir_schema.defaults)"]
    drift = ir_schema_drift(pinned, current)
    have, want = irs.get("version"), IR_SCHEMA_VERSION
    if have == want:
        return [f"IR schema: {x} - an IR-shape change needs a semver bump of "
                f"IR_SCHEMA_VERSION (still {want}) and a row in "
                f"docs/IR-MIGRATIONS.md, then `python -m guardrail.bundle lock "
                f"--update`" for x in drift]
    if not _semver_key(want) or not _semver_key(have) or \
            _semver_key(want) <= _semver_key(have):
        return [f"IR schema: IR_SCHEMA_VERSION {want} is not above the pinned "
                f"{have}; a schema version never goes back"]
    return [f"IR schema: IR_SCHEMA_VERSION is {want}, the lock pins {have} - run "
            f"`python -m guardrail.bundle lock --update` to record the change "
            f"({len(drift)} difference(s)) and re-pin"]


def _previously(ent: dict) -> set[str]:
    """The identities a lock entry says its file claimed before (`previously`:
    one id@version, or a list)."""
    p = ent.get("previously")
    return {p} if isinstance(p, str) else set(p or ())


def check_lock(policies_dir: str | Path | None = None,
               lock: dict | None = None) -> list[str]:
    """Every way the policies on disk disagree with the lock. Empty = clean.

    The rule (X-09, WP1-22): an `id@version` names exactly one content hash,
    forever. Changing a policy without raising its version is reported, and
    so is a new `id@version` nobody has locked yet. A lock entry is never
    rewritten - old entries are how a stored run's hash stays traceable after
    the policy moves on.
    """
    lock = load_lock() if lock is None else lock
    entries = lock.get("policies") or {}
    problems: list[str] = []
    if not entries:
        # An empty or missing lock would make every check below vacuous, and
        # "0 problems" from a lock that pins nothing is the silent pass this
        # project keeps finding. Say it.
        problems.append("the lock pins no policies (missing or empty "
                        "policies/policy.lock.json)")
    problems += ir_schema_problems(lock)
    seen_shared: dict[str, set[str]] = {}
    seen_key: dict[str, str] = {}             # file name -> the id@version it claims
    for f in _policy_files(policies_dir):
        try:
            # runtime=False: the lock is about identity, not flyability; a
            # declarable-only policy still has a version and a hash.
            pol = load_policy(f, runtime=False)
        except Exception as exc:                          # noqa: BLE001
            problems.append(f"{f.name}: does not load ({type(exc).__name__}: {exc})")
            continue
        key = lock_key(pol)
        seen_key[f.name] = key
        ent = entries.get(key)
        if ent is None:
            problems.append(f"{f.name}: {key} is not in the lock - run "
                            f"`python -m guardrail.bundle lock --update`")
            continue
        shared = ent.get("shared_by_files")
        if shared is not None:
            # A known identity collision (several files, one id@version). Each
            # file is pinned on its own so a content change is still caught.
            seen_shared.setdefault(key, set()).add(f.name)
            pin = shared.get(f.name)
            want = (pin or {}).get("policy_hash")
            if want is None:
                problems.append(f"{f.name}: claims {key}, already shared by "
                                f"{sorted(shared)}; give it its own policy_id")
                continue
            status = _pin_status(pol, pin)
            if status == _RECORD_NEW_FORM:
                problems.append(_new_form_msg(f.name, key, pin, pol))
            elif status == _CHANGED:
                problems.append(f"{f.name}: content changed under {key} "
                                f"({want[:23]} -> {pol.policy_hash[:23]})")
            continue
        if ent.get("file") not in (None, f.name):
            problems.append(f"{f.name}: claims {key}, which the lock gives to "
                            f"{ent['file']} - two files must not share an "
                            f"identity")
            continue
        status = _pin_status(pol, ent)
        if status == _RECORD_NEW_FORM:
            problems.append(_new_form_msg(f.name, key, ent, pol))
        elif status == _CHANGED:
            problems.append(
                f"{f.name}: content changed but version did not - {key} is "
                f"locked to {str(ent.get('policy_hash'))[:23]}..., the file now "
                f"hashes to {pol.policy_hash[:23]}.... Raise `version:` and "
                f"lock the new one.")
        # A migrated entry's claim is checked, not trusted (see "Migrations").
        mf = ent.get("migrated_from")
        if mf is not None:
            why = migration_problem(pol, mf, _pin_for(entries.get(mf) or {}, f.name))
            if why:
                problems.append(f"{f.name}: {key} says migrated_from {mf}, but {why}")
    for key, ent in entries.items():
        if "shared_by_files" in ent:
            missing = set(ent["shared_by_files"]) - seen_shared.get(key, set())
            # A file that left the shared identity for one of its own (WP1-22,
            # 2026-10-21: the generated hard_*/hp_* files) is not missing when
            # it is on disk under an identity whose entry names this one as
            # `previously` and gives it that file. The shared entry itself is
            # never rewritten, so its per-file pins still trace a run flown
            # under the old id.
            missing -= {name for name in missing
                        if key in _previously(entries.get(seen_key.get(name, "")) or {})
                        and entries[seen_key[name]].get("file") == name}
            if missing:
                problems.append(f"{key}: lock lists {sorted(missing)} as sharing "
                                f"this identity, but they no longer do - drop "
                                f"them from shared_by_files")
    return problems


def update_lock(policies_dir: str | Path | None = None,
                path: str | Path | None = None,
                locked_on: str | None = None) -> list[str]:
    """Add every unlocked `id@version`. Never rewrites an existing entry.

    Returns what was added. Refuses (ValueError) while check_lock reports a
    content change under an existing version, because "updating" that would
    be exactly the silent re-fingerprinting the lock exists to stop.

    The IR schema pin is re-pinned only under a HIGHER IR_SCHEMA_VERSION (the
    freeze rule, ir_schema_problems). The re-pin replaces `defaults` with
    today's schema and appends to `ir_schema.migrations` what changed (the
    ir_schema_drift lines, from -> to, the day), so every earlier pin can be
    read back from the current one; `previous_versions` keeps the old label.
    A shape change under the SAME version is refused here, as check_lock
    reports it: until 2026-11-02 this function pinned a new None-default
    field or a new model without a bump.
    """
    p = Path(path) if path else LOCK_PATH
    lock = load_lock(p)
    entries = lock.setdefault("policies", {})
    day = locked_on or datetime.now(timezone.utc).strftime("%Y-%m-%d")
    added: list[str] = []
    irs = lock.get("ir_schema") or {}
    if irs.get("defaults") is not None:
        from .models import ir_schema_defaults
        cur = ir_schema_defaults()
        drift = ir_schema_drift(irs["defaults"], cur)
        if irs.get("version") == IR_SCHEMA_VERSION:
            if drift:
                raise ValueError(
                    "lock not updated: the IR shape changed without a version "
                    f"bump (IR_SCHEMA_VERSION is still {IR_SCHEMA_VERSION}):\n  "
                    + "\n  ".join(drift))
        else:
            bad = ir_schema_problems(lock, cur)
            if bad and "lock --update" not in bad[0]:
                raise ValueError("lock not updated: " + bad[0])
            irs.setdefault("migrations", []).append(
                {"from": irs.get("version"), "to": IR_SCHEMA_VERSION, "on": day,
                 "changes": drift})
            irs.setdefault("previous_versions", []).append(irs.get("version"))
            irs["version"] = IR_SCHEMA_VERSION
            irs["defaults"] = cur
            added.append(f"ir_schema:version {IR_SCHEMA_VERSION}")
    for f in _policy_files(policies_dir):
        pol = load_policy(f, runtime=False)
        key = lock_key(pol)
        if key in entries:
            # The one addition an existing entry may receive: the current
            # scheme's hash of UNCHANGED content (see _pin_status).
            ent = entries[key]
            if "shared_by_files" in ent:
                pin = (ent.get("shared_by_files") or {}).get(f.name)
            else:
                pin = ent
            if pin and _pin_status(pol, pin) == _RECORD_NEW_FORM:
                pin.setdefault("hashes", {})[HASH_SCHEME] = pol.policy_hash
                added.append(f"{key}:{HASH_SCHEME}")
            continue
        entries[key] = {"file": f.name, "locked_on": day, **_entry(pol)}
        # A file that a shared (colliding) identity lists, now under an id of
        # its own: the new entry points back, so check_lock knows it moved
        # rather than vanished, and a reader can follow the old pin.
        was = sorted(k for k, e in entries.items() if k != key
                     and f.name in (e.get("shared_by_files") or {}))
        if was:
            entries[key]["previously"] = was[0] if len(was) == 1 else was
        # The same content under a raised version after an IR change that
        # moved its hash (see "Migrations"): the new entry names the old one,
        # and check_lock re-proves the link on every run.
        src = _migration_source(pol, f.name, entries, key)
        if src is not None:
            entries[key]["migrated_from"] = src[0]
            entries[key]["migration"] = {"ir_schema_version": IR_SCHEMA_VERSION,
                                         "old_hash_form": src[1]}
        added.append(key)
    still = [x for x in check_lock(policies_dir, lock)
             if "not in the lock" not in x]
    if still:
        raise ValueError("lock not updated; fix these first:\n  " + "\n  ".join(still))
    p.write_text(json.dumps(lock, indent=2, sort_keys=True) + "\n",
                 encoding="utf-8", newline="\n")
    return added


def derive(base: Policy, hot_apply: list[dict], generations: int | None = None,
           *, ir_schema_version: str | None = None) -> Policy:
    """Rebuild a hot-applied policy exactly as Shield.hot_apply produces it:
    each rule appended, generation bumped once per rule.

    `ir_schema_version` is the IR schema the recipe was recorded under; None
    (every recipe in the lock before 2026-11-02) means 2.x. A 2.x recipe may
    omit constraint_type / priority / violation_action, which then meant
    hard / P0 / repair (models.IR2_RULE_DEFAULTS); those are filled in, so
    the rule is rebuilt as it flew. Under 3.0.0 a recipe states them."""
    from pydantic import TypeAdapter
    from .models import IR2_RULE_DEFAULTS, Constraint
    ir2 = _semver_key(ir_schema_version or "2.0.0") < (3, 0, 0)
    pol = base.model_copy(deep=True)
    ta = TypeAdapter(Constraint)
    for raw in hot_apply:
        if ir2 and isinstance(raw, dict):
            raw = {**IR2_RULE_DEFAULTS, **raw}
        pol.constraints.append(ta.validate_python(raw))
        pol.generation += 1
    if generations is not None and pol.generation != base.generation + generations:
        raise ValueError(f"derivation expected {generations} generation(s), "
                         f"got {pol.generation - base.generation}")
    return pol


def policy_candidates(policies_dir: str | Path | None = None,
                      lock: dict | None = None) -> list[tuple[str, Policy]]:
    """(label, Policy) for every policy a stored run may have flown under:
    each policies/*.yaml, plus each hot-applied derivation the lock records.

    A run that hot-applied a rule mid-flight recorded the hash AFTER the
    change, which no file on disk produces - the dynamic NFZ runs. The lock's
    `derived` list says how to rebuild those, so they verify instead of being
    written off as unmatched.

    A file whose lock entry says `migrated_from` (an IR change moved its hash,
    so its version was raised; see "Migrations") is also offered at each
    older version of that chain, labelled "<file> as <id@version>": a run
    flown under the old version resolves to it, under the IR 2.x form its
    hash was recorded in, and a recipe based on the old version rebuilds.
    """
    lock = load_lock() if lock is None else lock
    entries = lock.get("policies") or {}
    out: list[tuple[str, Policy]] = []
    by_key: dict[str, Policy] = {}
    for f in _policy_files(policies_dir):
        try:
            pol = load_policy(f, runtime=False)
        except Exception:                                 # noqa: BLE001
            continue
        out.append((f.name, pol))
        by_key.setdefault(lock_key(pol), pol)
        ent, seen = entries.get(lock_key(pol)), set()
        while ent and ent.get("migrated_from") and ent["migrated_from"] not in seen:
            old_key = ent["migrated_from"]
            seen.add(old_key)
            old = as_version(pol, old_key.rpartition("@")[2])
            out.append((f"{f.name} as {old_key}", old))
            by_key.setdefault(old_key, old)
            ent = entries.get(old_key)
    for d in lock.get("derived") or []:
        base = by_key.get(d.get("base"))
        if base is None:
            continue
        out.append((d["name"], derive(base, d.get("hot_apply") or [],
                                      d.get("generations"),
                                      ir_schema_version=d.get("ir_schema_version"))))
    return out


# --------------------------------------------------------------------------- #
# Published schema (WP1-16)
# --------------------------------------------------------------------------- #

def ir_json_schema() -> dict:
    """The canonical IR's JSON Schema: exactly what a bundle's `ir.json` may
    hold - this project's field names, a geographic point as {lat, lon}, a
    metre point as {x, y}, unknown keys refused. Published inside
    policy_dsl.schema.json as `$defs/CanonicalIR` (see policy_json_schema)."""
    schema = Policy.model_json_schema()
    # The model requires x/y because the Shield reads them; the IR stores a
    # geographic point WITHOUT them (they are derived at load). So a point is
    # one frame or the other, never neither.
    xy = schema.get("$defs", {}).get("XY")
    if xy is not None:
        xy.pop("required", None)
        xy["anyOf"] = [{"required": ["x", "y"]}, {"required": ["lat", "lon"]}]
        xy["description"] = (
            "A point: {x, y} metres (x = North, y = East of the frame origin), "
            "or {lat, lon} WGS84 - the canonical form of a geographic point.")
    # A DERIVED field (exclude=True: circle_fence's polygon) is never in the IR
    # and is refused when authored, so the published contract does not offer it.
    from .models import ir_models
    for cls in ir_models():
        d = schema.get("$defs", {}).get(cls.__name__)
        for name, f in cls.model_fields.items():
            if f.exclude and d is not None:
                d.get("properties", {}).pop(name, None)
    return schema


def _authoring_rule_def(cls, d_ir: dict) -> dict:
    """One rule model in the authoring form: the IR's fields, the grant's
    names for them (GRANT_ALIASES), and the grant's `geometry:` block
    (GRANT_GEOMETRY) - built from the same tables `_normalise_rule` reads, so
    the schema and the loader cannot disagree about which names exist.

    Encoded as the loader enforces it: a required field may come under any of
    its names or inside `geometry`; two names for one field, or one key
    inside and beside `geometry`, is refused (`not: {required: [...]}`)."""
    import copy
    ir_props = d_ir.get("properties", {})
    props = copy.deepcopy(ir_props)
    aliases = dict(cls.GRANT_ALIASES)
    rules: list[dict] = []
    for grant, ours in sorted(aliases.items()):
        if ours in ir_props:
            s = copy.deepcopy(ir_props[ours])
            s["description"] = f"The grant's name for `{ours}`; give one of the two."
            s.pop("title", None)
            props[grant] = s
            rules.append({"not": {"required": [grant, ours]}})
    geom = sorted(cls.GRANT_GEOMETRY)
    if geom:
        gprops = {k: copy.deepcopy(ir_props[aliases.get(k, k)])
                  for k in geom if aliases.get(k, k) in ir_props}
        gdef: dict = {"type": "object", "additionalProperties": False,
                      "properties": gprops,
                      "description": ("The grant's geometry block. Keys omitted "
                                      "here take the reference's defaults: "
                                      "altitude band 0-200 m and, for a fence, "
                                      "no margin ring.")}
        pairs = [[k, ours] for k, ours in aliases.items() if k in geom and ours in geom]
        if pairs:
            gdef["allOf"] = [{"not": {"required": p}} for p in pairs]
        props["geometry"] = gdef
        for k in geom:
            for top in sorted({k, aliases.get(k, k)}):
                rules.append({"not": {"required": [top, "geometry"],
                                      "properties": {"geometry": {"required": [k]}}}})
    for r in d_ir.get("required", []):
        if r in ("id", "type"):
            continue
        options = [{"required": [r]}]
        options += [{"required": [g]} for g, o in sorted(aliases.items()) if o == r]
        options += [{"required": ["geometry"],
                     "properties": {"geometry": {"required": [k]}}}
                    for k in geom if aliases.get(k, k) == r]
        rules.append(options[0] if len(options) == 1 else {"anyOf": options})
    out = {k: v for k, v in d_ir.items() if k not in ("properties", "required")}
    out["properties"] = props
    out["required"] = [r for r in d_ir.get("required", []) if r in ("id", "type")]
    if rules:
        out["allOf"] = rules
    return out


def _prefix_refs(node, prefix: str):
    """Rewrite every `#/$defs/X` reference to `#/$defs/<prefix>X`."""
    if isinstance(node, list):
        return [_prefix_refs(v, prefix) for v in node]
    if isinstance(node, dict):
        out = {}
        for k, v in node.items():
            if k == "$ref" and isinstance(v, str) and v.startswith("#/$defs/"):
                out[k] = "#/$defs/" + prefix + v[len("#/$defs/"):]
            elif k == "mapping" and isinstance(v, dict):
                out[k] = {mk: ("#/$defs/" + prefix + mv[len("#/$defs/"):]
                               if isinstance(mv, str) and mv.startswith("#/$defs/") else mv)
                          for mk, mv in v.items()}
            else:
                out[k] = _prefix_refs(v, prefix)
        return out
    return node


IR_DEF_PREFIX = "IR."


def policy_json_schema() -> dict:
    """The grant's "DSL JSON Schema export", named `policy_dsl.schema.json` as
    in the reference package: the published contract for external tooling.

    The grant (Policy DSL page): the DSL is "the hand-authored surface form
    ... what an operator (or an external GIS tool) writes", and the Pydantic
    JSON Schema export of it is the published contract. So the ROOT of this
    schema is the AUTHORING form: the grant's names (`altitude_min_m`,
    `speed_max`, ...), `geometry:` blocks, `issued_at`, `layers_merged`,
    lat/lon with or without an `origin` - and this project's own names, which
    keep loading. Until 2026-10-07 the root described the canonical IR only,
    and it rejected the grant's own example document (review finding).

    The canonical IR - what a bundle's ir.json holds, this project's names
    only - is published beside it as `$defs/CanonicalIR` (its own models under
    `$defs/IR.<Model>`), so a reader can validate either. Every canonical IR
    is also a valid authoring document.

    JSON Schema checks the document's shape. The DSL checks more than a shape
    can say - polygon validity, unique ids, conflicting hard limits - and
    `load_policy` remains the authority; tools/wp1_roundtrip_kpi.py reports
    how much of the negative corpus the schema alone refuses. A YAML timestamp
    (`issued_at: 2026-04-28T09:00:00Z` unquoted) is validated as its ISO text,
    which is how the loader reads it.
    """
    from .models import ir_models
    ir = ir_json_schema()
    defs = ir.get("$defs", {})
    root = {k: v for k, v in ir.items() if k != "$defs"}
    out_defs: dict = {}
    for cls in ir_models():
        d = defs.get(cls.__name__)
        if d is None:
            continue
        if getattr(cls, "GRANT_ALIASES", None) is not None and "type" in cls.model_fields:
            out_defs[cls.__name__] = _authoring_rule_def(cls, d)
        else:
            out_defs[cls.__name__] = d
    for name, d in defs.items():
        out_defs.setdefault(name, d)
        out_defs[IR_DEF_PREFIX + name] = _prefix_refs(d, IR_DEF_PREFIX)
    canonical = _prefix_refs(root, IR_DEF_PREFIX)
    canonical["title"] = "CanonicalIR"
    canonical["description"] = (
        "The canonical IR: what a bundle's ir.json holds and policy_hash "
        "digests. This project's field names only; a geographic point as "
        "{lat, lon}, a metre point as {x, y}.")
    out_defs["CanonicalIR"] = canonical
    schema = dict(root)
    # At least one rule: the lint refuses an empty policy at every DSL entry
    # point (load_policy, the merge, check_bundle), so no document either form
    # describes may have none.
    for doc in (schema, canonical):
        doc["properties"] = dict(doc["properties"])
        doc["properties"]["constraints"] = {**doc["properties"]["constraints"],
                                            "minItems": 1}
    schema["title"] = "Policy DSL (authoring form)"
    schema["description"] = (
        "A policy as an operator or GIS tool writes it: the grant's form "
        "(geometry blocks, altitude_min_m / altitude_max_m, speed_max, "
        "issued_at, layers_merged, WGS84 with or without origin) or this "
        "project's earlier form. $defs/CanonicalIR is the bundle IR.")
    schema["$defs"] = out_defs
    schema["$schema"] = "https://json-schema.org/draft/2020-12/schema"
    schema["x-ir-schema-version"] = IR_SCHEMA_VERSION
    schema["x-hash-scheme"] = HASH_SCHEME
    schema["x-canonical-ir"] = {"$ref": "#/$defs/CanonicalIR"}
    return schema


def write_schema(out: str | Path | None = None) -> Path:
    p = Path(out) if out else SCHEMA_PATH
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(policy_json_schema(), indent=2, sort_keys=True) + "\n",
                 encoding="utf-8", newline="\n")
    return p


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def _release_check(pol: Policy, source: str) -> str | None:
    """Why `pol` may not be published under its id@version, or None."""
    ent = (load_lock().get("policies") or {}).get(lock_key(pol))
    if ent and "policy_hash" in ent and _pin_status(pol, ent) == _CHANGED:
        return (f"{lock_key(pol)} is locked to {ent['policy_hash']}, but {source} "
                f"hashes to {pol.policy_hash}: raise its version.")
    return None


def _main(argv: list[str] | None = None) -> int:
    """Emit a bundle from a policy file, or one of six other commands.

        python -m guardrail.bundle policies/wgs84_taipei.yaml -m "initial"
        python -m guardrail.bundle POLICY --ir-form reference   # the reference reads it
        python -m guardrail.bundle verify bundles/wgs84-taipei-demo-v0.1.0.tar.gz
        python -m guardrail.bundle merge reg.yaml site.yaml mission.yaml -o out.tar.gz
        python -m guardrail.bundle export POLICY [--origin LAT,LON] [-o out.yaml]
        python -m guardrail.bundle export POLICY --reference [--conservative]
        python -m guardrail.bundle schema          # policies/policy_dsl.schema.json
        python -m guardrail.bundle lock [--update] # policies/policy.lock.json
        python -m guardrail.bundle keygen          # lab dev key, once per lab

    `merge` is the grant's layer merge (regulation, site, mission - pass them in
    that order); `export` writes a policy in the grant's own form (WGS84,
    `geometry:` blocks, grant field names).

    Bundles are build products, not source: they are byte-derivable from the
    policy (given the issue time), so `bundles/*.tar.gz` is gitignored here
    exactly as it is in the reference repository. A bundle carries the
    declaration as written, so a declarable-only rule may be bundled; the
    flight loaders refuse it (`load_for_flight`).
    """
    import argparse
    import sys

    argv = list(sys.argv[1:] if argv is None else argv)
    cmds = ("schema", "keygen", "verify", "lock", "merge", "export")
    cmd = argv[0] if argv and argv[0] in cmds else None

    if cmd == "schema":
        ap = argparse.ArgumentParser(prog="python -m guardrail.bundle schema")
        ap.add_argument("-o", "--out", default=None)
        a = ap.parse_args(argv[1:])
        p = write_schema(a.out)
        print(f"{p}  ({len(policy_json_schema().get('$defs', {}))} definitions, "
              f"IR schema {IR_SCHEMA_VERSION})")
        return 0

    if cmd == "keygen":
        ap = argparse.ArgumentParser(prog="python -m guardrail.bundle keygen")
        ap.add_argument("--out", default=None)
        ap.add_argument("--authority", default="lab-dev")
        ap.add_argument("--trust", default=None)
        a = ap.parse_args(argv[1:])
        s = generate_signing_key(a.out, a.authority, a.trust)
        print(f"private key  {a.out or DEFAULT_PRIVATE_KEY}  (gitignored - keep it so)")
        print(f"registered   {s.identity}  in  {a.trust or trust_store_path()}")
        return 0

    if cmd == "verify":
        ap = argparse.ArgumentParser(prog="python -m guardrail.bundle verify")
        ap.add_argument("bundle")
        ap.add_argument("--allow-unsigned", action="store_true")
        a = ap.parse_args(argv[1:])
        try:
            chk = check_bundle(a.bundle)
        except (ValueError, OSError, tarfile.TarError) as exc:
            print(f"REFUSED  {exc}")
            return 2
        print(f"{chk.path.name}: {chk.policy.policy_id} v{chk.policy.version} "
              f"gen {chk.policy.generation}")
        print(f"  policy_hash {chk.manifest.get('policy_hash')}  ({chk.hash_form})")
        for scheme, h in sorted(chk.hashes.items()):
            print(f"  verified    {scheme}: "
                  + (h if h is not None else "not representable (recorded and checked)"))
        print(f"  issued_at   {chk.manifest.get('issued_at', '(not recorded)')}")
        print(f"  signature   {chk.signature.upper()}  - {chk.detail}")
        return 0 if (chk.verified or a.allow_unsigned) else 1

    if cmd == "lock":
        ap = argparse.ArgumentParser(prog="python -m guardrail.bundle lock")
        ap.add_argument("--update", action="store_true")
        a = ap.parse_args(argv[1:])
        if a.update:
            added = update_lock()
            print(f"added {len(added)} entr{'y' if len(added) == 1 else 'ies'}: "
                  + (", ".join(added) or "none"))
        problems = check_lock()
        for x in problems:
            print(f"  ! {x}")
        print(f"{len(_policy_files())} policies, {len(problems)} problem(s)")
        return 1 if problems else 0

    if cmd == "export":
        import yaml as _yaml
        ap = argparse.ArgumentParser(prog="python -m guardrail.bundle export")
        ap.add_argument("policy")
        ap.add_argument("-o", "--out", default=None)
        ap.add_argument("--origin", default=None,
                        help="LAT,LON of the local frame's (0, 0), for a metre "
                             "policy that states no origin")
        ap.add_argument("--reference", action="store_true",
                        help="write the PI's reference implementation's form of "
                             "what it can represent (guardrail/reference_convert"
                             ".py) and say what it left out")
        ap.add_argument("--conservative", action="store_true",
                        help="with --reference: fold margin rings into the "
                             "vertices and drop schedules (stricter), recorded")
        a = ap.parse_args(argv[1:])
        pol = load_policy(a.policy, runtime=False)
        org = tuple(float(v) for v in a.origin.split(",")) if a.origin else None
        if a.reference:
            from .reference_convert import to_reference
            conv = to_reference(pol, origin=org, conservative=a.conservative)
            rec = conv.record()
            for rid, why in rec["omitted"].items():
                print(f"# not converted  {rid}: {why}", file=sys.stderr)
            for rid, how in {**rec["conservative"], **rec["reworded"]}.items():
                print(f"# converted      {rid}: {how}", file=sys.stderr)
            for f, why in rec["not_carried"].items():
                print(f"# not carried    {f}: {why}", file=sys.stderr)
            if conv.doc is None:
                print("REFUSED  no rule of this policy is in the reference's DSL; "
                      "a reference document needs at least one")
                return 2
            print(f"# reference policy_hash {rec['policy_hash']} over "
                  f"{len(rec['rules'])} of {len(pol.constraints)} rules "
                  f"({'complete' if rec['complete'] else 'a projection'}); this "
                  f"project's hash of the whole policy {pol.policy_hash}",
                  file=sys.stderr)
            text = _yaml.safe_dump(conv.doc, sort_keys=False, allow_unicode=False)
            if a.out:
                Path(a.out).write_text(text, encoding="utf-8", newline="\n")
                print(f"{a.out}  reference form of {pol.policy_id} v{pol.version}")
            else:
                print(text, end="")
            return 0
        try:
            doc = pol.to_grant_form(origin=org)
        except ValueError as exc:
            print(f"REFUSED  {exc}")
            return 2
        text = _yaml.safe_dump(doc, sort_keys=False, allow_unicode=False)
        if a.out:
            Path(a.out).write_text(text, encoding="utf-8", newline="\n")
            print(f"{a.out}  grant form of {pol.policy_id} v{pol.version}")
        else:
            print(text, end="")
        ext = pol.grant_form_extensions()
        if ext:
            print(f"# kept beyond the grant's form: {', '.join(ext)}", file=sys.stderr)
        return 0

    ap = argparse.ArgumentParser(prog="python -m guardrail.bundle" +
                                 (" merge" if cmd == "merge" else ""))
    if cmd == "merge":
        ap.add_argument("policy", nargs="+")
        ap.add_argument("--policy-id", default=None)
        ap.add_argument("--version", default=None)
    else:
        ap.add_argument("policy")
    ap.add_argument("-o", "--out")
    ap.add_argument("-m", "--changelog", default="initial")
    ap.add_argument("--issued-at", default=None,
                    help="ISO-8601; default the policy's own issued_at, else "
                         "SOURCE_DATE_EPOCH, else now")
    ap.add_argument("--key", default=None, help="signing key (default: lab key)")
    ap.add_argument("--unsigned", action="store_true",
                    help="write an explicitly UNSIGNED bundle")
    ap.add_argument("--ir-form", choices=IR_FORMS, default="ours",
                    help="'reference': ship the reference implementation's IR "
                         "and hash, so its loader reads the bundle")
    ap.add_argument("--previous", default=None,
                    help="the bundle (or policy file) this version replaces: the "
                         "manifest records the diff against it (changelog_diff)")
    args = ap.parse_args(argv[1:] if cmd == "merge" else argv)

    if cmd == "merge":
        from .models import load_layered
        try:
            pol = load_layered(args.policy, policy_id=args.policy_id,
                               version=args.version, runtime=False)
        except ValueError as exc:
            print(f"REFUSED  {exc}")
            return 2
    else:
        pol = load_policy(args.policy, runtime=False)
        # A bundle is a release: refuse to publish content under a version the
        # lock already gives to different content.
        why = _release_check(pol, args.policy)
        if why:
            print(f"REFUSED  {why}")
            return 2
        if (load_lock().get("policies") or {}).get(lock_key(pol)) is None:
            print(f"WARNING  {lock_key(pol)} is not in policies/policy.lock.json")
    out = Path(args.out) if args.out else (
        ROOT / "bundles" / f"{pol.policy_id}-v{pol.version}.tar.gz")
    signer = None if args.unsigned else (args.key or "auto")
    try:
        p = write_bundle(pol, out, args.changelog, issued_at=args.issued_at,
                         signer=signer, ir_form=args.ir_form, previous=args.previous)
    except ValueError as exc:
        print(f"REFUSED  {exc}")
        return 2
    chk = check_bundle(p)               # never ship one that will not reload
    if chk.manifest.get("changelog_diff") is not None:
        print(f"  changelog   {chk.manifest['changelog']}")
    print(f"{p}  {p.stat().st_size} bytes")
    print(f"  policy_id   {chk.policy.policy_id}")
    print(f"  version     {chk.policy.version}  generation {chk.policy.generation}")
    print(f"  policy_hash {chk.manifest.get('policy_hash')}  ({chk.hash_form})")
    for scheme, h in sorted(chk.hashes.items()):
        print(f"  verified    {scheme}: "
              + (h if h is not None else "not representable - "
                 + "; ".join(chk.manifest.get("reference_problems") or [])))
    print(f"  issued_at   {chk.manifest.get('issued_at')}")
    print(f"  rules       {len(chk.policy.constraints)}")
    if chk.policy.layers_merged:
        print(f"  layers      {', '.join(chk.policy.layers_merged)}")
    gaps = chk.policy.flight_problems()
    if gaps:
        print(f"  NOT FLYABLE {len(gaps)} problem(s): "
              + "; ".join(gaps))
    print(f"  signature   {chk.signature.upper()}  - {chk.detail}")
    return 0


if __name__ == "__main__":
    import sys as _sys
    _sys.exit(_main())
