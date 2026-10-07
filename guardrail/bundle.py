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

A reference-form bundle is an EXCHANGE format, not a flight artefact. Its IR
writes every default out, so the policy rebuilt from it has a third hash -
neither the manifest's nor the source policy's (the source policy recognises
the manifest's hash as REFERENCE_FORM). The flight loaders (`load_bundle`,
`load_for_flight`) refuse it; `check_bundle` reads it.

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

from .models import (HASH_SCHEME, IR_SCHEMA_VERSION, PRIOR_SCHEME_V2,
                     REFERENCE_FORM, Policy, _prune_nulls, canonical_json,
                     load_policy)

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


def manifest_for(policy: Policy, changelog: str = "initial", *,
                 issued_at: str | None = None,
                 signed_by: str | None = None,
                 ir_form: str = "ours") -> dict[str, Any]:
    """The bundle's identity card: the grant's six fields, plus how the hash was
    made (`hash_scheme`, `ir_schema_version`) and, for a published bundle, when
    it was issued. `issued_at=None` leaves the key out (a replay bundle embeds a
    policy it did not issue). `ir_form="reference"` describes a bundle whose
    ir.json is the reference implementation's document (see _ir_for)."""
    _, digest, scheme = _ir_for(policy, ir_form)
    m = {
        "policy_id": policy.policy_id,
        "version": policy.version,
        "generation": policy.generation,
        "policy_hash": digest,
        "hash_scheme": scheme,
        "ir_schema_version": IR_SCHEMA_VERSION,
        "signed_by": signed_by or UNSIGNED,
        "changelog": changelog,
    }
    if issued_at is not None:
        m["issued_at"] = issued_at
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
                 signer="auto", ir_form: str = "ours") -> Path:
    """Write a tar.gz containing the canonical IR, the manifest and the signature.

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
    manifest = manifest_for(policy, changelog, issued_at=_issued_at(issued_at),
                            signed_by=sgn.identity if sgn else UNSIGNED,
                            ir_form=ir_form)
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

    @property
    def verified(self) -> bool:
        return self.signature == SIG_VERIFIED

    def source_record(self) -> dict:
        """What a flight writes about where its policy came from.

        `declared_policy_hash` is the hash the bundle's manifest (and its
        signature) names; `loaded_policy_hash` is the hash of the policy
        object rebuilt from it. They are equal for every bundle this project
        writes in its own form, and for the legacy forms `hash_form` names the
        relation. Both are recorded so a run never has to infer one."""
        return {"kind": "bundle", "path": str(self.path),
                "loaded_policy_hash": self.policy.policy_hash,
                "declared_policy_hash": self.manifest.get("policy_hash"),
                "hash_form": self.hash_form,
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
        if policy.policy_hash == declared:
            form = HASH_SCHEME
        elif policy.reference_hash() == declared:
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
        if form in (HASH_SCHEME, PRIOR_SCHEME_V2, REFERENCE_FORM):
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
        if _prune_nulls(json.loads(ir_bytes), frozenset()) != \
                _prune_nulls(policy.model_dump(mode="json"), frozenset()):
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
                       signature=status, signed_by=signed_by, detail=detail)


def _refuse_for_flight(chk: BundleCheck, name: str) -> None:
    """What no flight loader accepts from a bundle, whatever its signature.

    1. A rule the Shield would not enforce as written (models.
       _refuse_unenforced: a declarable-only type, MSL, a derived frame).
    2. A bundle in the reference implementation's IR form. It is an EXCHANGE
       format: its ir.json writes every default out (scope, layer,
       altitude_ref, margin 0) and spells `repair` as `project_fix`, so the
       policy rebuilt from it hashes to a THIRD value - neither the manifest's
       hash nor the source policy's. A run flown from it would record a
       loaded_policy_hash that resolves to no policy (2026-10-07 review).
       Re-issue it in this project's form to fly it."""
    from .models import _refuse_unenforced
    if chk.hash_form == REFERENCE_FORM:
        raise ValueError(
            f"{name}: a bundle in the reference implementation's IR form "
            f"({REFERENCE_FORM}) is an exchange format, not a flight artefact: "
            f"the policy rebuilt from it hashes to {chk.policy.policy_hash[:23]}..., "
            f"not the manifest's {str(chk.manifest.get('policy_hash'))[:23]}..., "
            f"so a run could not be traced back to it. Write the bundle in this "
            f"project's form (`python -m guardrail.bundle POLICY`) to fly it.")
    _refuse_unenforced(chk.policy, name)


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
    in the reference's exchange form, is refused. Until 2026-10-07 this
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
    return chk.policy


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
        pol = chk.policy
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
            "legacy_hashes": policy.legacy_hashes()}


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
    """How today's IR schema differs from the pinned one, in ways that matter.

    Allowed silently: a NEW field defaulting to None ("<absent>") - the one
    change that cannot move any stored hash, because the canonical form omits
    it. Everything else is reported: a new field with a value default (every
    policy's canonical bytes grow), a changed default (every policy that omits
    the field now means something else), a removed field or model - and a
    model nobody has pinned yet (UNPINNED_MODEL), because a pin that never
    sees a model cannot notice when that model's defaults change later.
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
            elif now[name] != was:
                out.append(f"{model}.{name}: default {was!r} -> {now[name]!r}; "
                           f"every policy that omits it changes meaning")
        for name in sorted(set(now) - set(fields)):
            if now[name] != "<absent>":
                out.append(f"{model}.{name}: new field with default "
                           f"{now[name]!r} - it would enter every canonical IR "
                           f"and move every hash; default it to None")
    # A new MODEL (a new constraint type) is additive: no existing policy uses
    # it, so no stored hash moves. But until it is pinned, its defaults can
    # change unseen - the review of 2026-10-06 found exactly that hole behind
    # a hand-written class list. So it is reported until `lock --update`
    # pins it, which adds the pin and never rewrites an existing one.
    for model in sorted(set(cur) - set(locked)):
        out.append(f"{model}: {UNPINNED_MODEL} - run `python -m guardrail.bundle "
                   f"lock --update` (additive: no stored hash moves)")
    return out


UNPINNED_MODEL = "in the IR schema but not pinned"


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
    pinned = (lock.get("ir_schema") or {}).get("defaults")
    if pinned is None:
        problems.append("the lock does not pin the IR schema (ir_schema.defaults)")
    else:
        problems += [f"IR schema: {x}" for x in ir_schema_drift(pinned)]
    seen_shared: dict[str, set[str]] = {}
    for f in _policy_files(policies_dir):
        try:
            # runtime=False: the lock is about identity, not flyability; a
            # declarable-only policy still has a version and a hash.
            pol = load_policy(f, runtime=False)
        except Exception as exc:                          # noqa: BLE001
            problems.append(f"{f.name}: does not load ({type(exc).__name__}: {exc})")
            continue
        key = lock_key(pol)
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
    for key, ent in entries.items():
        if "shared_by_files" in ent:
            missing = set(ent["shared_by_files"]) - seen_shared.get(key, set())
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
    """
    p = Path(path) if path else LOCK_PATH
    lock = load_lock(p)
    entries = lock.setdefault("policies", {})
    day = locked_on or datetime.now(timezone.utc).strftime("%Y-%m-%d")
    added: list[str] = []
    # A new IR model (a new constraint type) is pinned here, the first time it
    # is seen. An existing model's pin is never touched: changing what an
    # omitted field means is a schema bump, not an update.
    pinned = (lock.get("ir_schema") or {}).get("defaults")
    if pinned is not None:
        from .models import ir_schema_defaults
        for model, fields in sorted(ir_schema_defaults().items()):
            if model not in pinned:
                pinned[model] = fields
                added.append(f"ir_schema:{model}")
                continue
            # A NEW field whose omission means "absent" is the one change
            # ir_schema_drift allows unannounced; recording it here keeps the
            # pin a complete list of the model's fields, so a later change to
            # its default is reported as a change. Existing pins are never
            # altered.
            for name, val in sorted(fields.items()):
                if name not in pinned[model] and val == "<absent>":
                    pinned[model][name] = val
                    added.append(f"ir_schema:{model}.{name}")
        irs = lock["ir_schema"]
        if irs.get("version") != IR_SCHEMA_VERSION:
            # The pin's label follows the schema it now pins; the old label is
            # kept, as every other lock record is.
            irs.setdefault("previous_versions", []).append(irs.get("version"))
            irs["version"] = IR_SCHEMA_VERSION
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
        added.append(key)
    still = [x for x in check_lock(policies_dir, lock)
             if "not in the lock" not in x]
    if still:
        raise ValueError("lock not updated; fix these first:\n  " + "\n  ".join(still))
    p.write_text(json.dumps(lock, indent=2, sort_keys=True) + "\n",
                 encoding="utf-8", newline="\n")
    return added


def derive(base: Policy, hot_apply: list[dict], generations: int | None = None) -> Policy:
    """Rebuild a hot-applied policy exactly as Shield.hot_apply produces it:
    each rule appended, generation bumped once per rule."""
    from pydantic import TypeAdapter
    from .models import Constraint
    pol = base.model_copy(deep=True)
    ta = TypeAdapter(Constraint)
    for raw in hot_apply:
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
    """
    lock = load_lock() if lock is None else lock
    out: list[tuple[str, Policy]] = []
    by_key: dict[str, Policy] = {}
    for f in _policy_files(policies_dir):
        try:
            pol = load_policy(f, runtime=False)
        except Exception:                                 # noqa: BLE001
            continue
        out.append((f.name, pol))
        by_key.setdefault(lock_key(pol), pol)
    for d in lock.get("derived") or []:
        base = by_key.get(d.get("base"))
        if base is None:
            continue
        out.append((d["name"], derive(base, d.get("hot_apply") or [],
                                      d.get("generations"))))
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
        a = ap.parse_args(argv[1:])
        pol = load_policy(a.policy, runtime=False)
        org = tuple(float(v) for v in a.origin.split(",")) if a.origin else None
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
                         signer=signer, ir_form=args.ir_form)
    except ValueError as exc:
        print(f"REFUSED  {exc}")
        return 2
    chk = check_bundle(p)               # never ship one that will not reload
    print(f"{p}  {p.stat().st_size} bytes")
    print(f"  policy_id   {chk.policy.policy_id}")
    print(f"  version     {chk.policy.version}  generation {chk.policy.generation}")
    print(f"  policy_hash {chk.manifest.get('policy_hash')}  ({chk.hash_form})")
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
