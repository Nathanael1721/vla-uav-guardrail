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

THE LAYOUT IS THE REFERENCE'S, DELIBERATELY

`packages/policy-dsl/src/policy_dsl/ingest.py` writes `ir.json`,
`manifest.json`, `signature.txt`, member mtimes pinned to 0. Matching that is
worth more than any improvement: the two halves of the same grant must be able
to read each other's bundles. The first line of `signature.txt` is still the
reference's `<policy_hash> <signed_by>`; what changed on 2026-10-06 is that a
second line now carries a real signature.

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

`cryptography` is an optional import: the SITL flight venvs in WSL
(sitl/setup_ros2.sh and sitl/setup_sitl.sh never install it) and the 3.11
`vla-drone` environment do not have it. The first cut of this release answered
that with the status "unverifiable-here", which meant two of the three flight
rails could never verify a signature - every --bundle there was refused, or
flown under --allow-unverified-bundle with a forged signature reading the same
as a real one (2026-10-06 review). So verification now has a pure-Python
fallback (RFC 8032, `py_ed25519_verify` below). Verifying handles only public
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

from .models import (HASH_SCHEME, IR_SCHEMA_VERSION, Policy, _prune_nulls,
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


def manifest_for(policy: Policy, changelog: str = "initial", *,
                 issued_at: str | None = None,
                 signed_by: str | None = None) -> dict[str, Any]:
    """The bundle's identity card: the grant's six fields, plus how the hash was
    made (`hash_scheme`, `ir_schema_version`) and, for a published bundle, when
    it was issued. `issued_at=None` leaves the key out (a replay bundle embeds a
    policy it did not issue)."""
    m = {
        "policy_id": policy.policy_id,
        "version": policy.version,
        "generation": policy.generation,
        "policy_hash": policy.policy_hash,
        "hash_scheme": HASH_SCHEME,
        "ir_schema_version": IR_SCHEMA_VERSION,
        "signed_by": signed_by or UNSIGNED,
        "changelog": changelog,
    }
    if issued_at is not None:
        m["issued_at"] = issued_at
    return m


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
                 signer="auto") -> Path:
    """Write a tar.gz containing the canonical IR, the manifest and the signature.

    `ir.json` is `policy.canonical_bytes()` - the bytes `policy_hash` digests,
    not a prettier rendering of them - so `sha256(ir.json)` IS the hash.

    `signer`: "auto" signs with the lab key when this machine has it (and
    writes an honestly UNSIGNED bundle when it does not); a `Signer` or a key
    path signs with that key; None writes unsigned on purpose.
    """
    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    sgn = _resolve_signer(signer)

    ir_bytes = policy.canonical_bytes()
    manifest = manifest_for(policy, changelog, issued_at=_issued_at(issued_at),
                            signed_by=sgn.identity if sgn else UNSIGNED)
    manifest_bytes = json.dumps(manifest, indent=2, sort_keys=True).encode()
    signature = signature_text(policy.policy_hash, sgn, manifest_bytes)

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
        """What a flight writes about where its policy came from."""
        return {"kind": "bundle", "path": str(self.path),
                "loaded_policy_hash": self.policy.policy_hash,
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

    # model_validate, not load_policy: the IR is already projected and
    # validated, and running it back through the lat/lon loader would be a
    # second chance to move the geometry.
    policy = Policy.model_validate(json.loads(ir_bytes))

    digest = "sha256:" + hashlib.sha256(ir_bytes).hexdigest()
    if declared == digest:
        # The bytes are what was hashed. Now prove the OBJECT is too: pydantic
        # ignores unknown keys, so an IR with an extra field hashes correctly
        # and then rebuilds into a different policy than the one signed.
        if policy.policy_hash != declared:
            raise ValueError(
                f"{p.name}: ir.json hashes to the manifest's policy_hash, but "
                f"rebuilds to {policy.policy_hash}: it holds content this code "
                f"cannot represent (an unknown key, or a non-canonical "
                f"encoding). Refusing rather than enforcing a different policy.")
        form = HASH_SCHEME
    else:
        # A bundle written before 2026-10-06: 16-hex hash over the old dump.
        form = policy.hash_form(declared)
        if form == HASH_SCHEME:
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


def load_bundle(path: str | Path, *, require_signature: bool = True,
                trust_store: str | Path | dict | None = None) -> Policy:
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

    The default changed on 2026-10-06 (it used to accept the placeholder
    signer). A caller that writes a bundle and reads it straight back on a
    machine without the lab key - tools/build_deck_data.py does - gets an
    UNSIGNED bundle and must say `require_signature=False` explicitly.
    """
    chk = check_bundle(path, trust_store)
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
    """
    from .manifest import policy_source_record
    if bundle and yaml_path:
        raise ValueError("give a policy bundle or a YAML policy, not both - a "
                         "run with two candidate policies cannot say which flew")
    if bundle:
        chk = check_bundle(bundle, trust_store)
        if not chk.verified and not allow_unverified:
            raise BundleSignatureError(
                f"{Path(bundle).name}: signature {chk.signature}: {chk.detail}. "
                f"Not flying an unverified bundle as if signed; pass "
                f"--allow-unverified-bundle to fly it recorded as such.")
        rec = chk.source_record()
        if not chk.verified:
            rec["accepted_unverified"] = True
        return chk.policy, rec
    if yaml_path is None:
        raise ValueError("no policy given: pass a bundle or a YAML file")
    pol = load_policy(yaml_path)
    return pol, policy_source_record("yaml", str(yaml_path), pol.policy_hash)


# --------------------------------------------------------------------------- #
# The version lock: a policy's version must change whenever its content does
# --------------------------------------------------------------------------- #

def lock_key(policy: Policy) -> str:
    return f"{policy.policy_id}@{policy.version}"


def _entry(policy: Policy) -> dict:
    return {"policy_hash": policy.policy_hash,
            "legacy_hashes": policy.legacy_hashes()}


def load_lock(path: str | Path | None = None) -> dict:
    p = Path(path) if path else LOCK_PATH
    return json.loads(p.read_text(encoding="utf-8")) if p.is_file() else {}


def _policy_files(policies_dir: str | Path | None = None) -> list[Path]:
    d = Path(policies_dir) if policies_dir else ROOT / "policies"
    return sorted(d.glob("*.yaml"))


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
            pol = load_policy(f)
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
            want = (shared.get(f.name) or {}).get("policy_hash")
            if want is None:
                problems.append(f"{f.name}: claims {key}, already shared by "
                                f"{sorted(shared)}; give it its own policy_id")
            elif want != pol.policy_hash:
                problems.append(f"{f.name}: content changed under {key} "
                                f"({want[:23]} -> {pol.policy_hash[:23]})")
            continue
        if ent.get("file") not in (None, f.name):
            problems.append(f"{f.name}: claims {key}, which the lock gives to "
                            f"{ent['file']} - two files must not share an "
                            f"identity")
            continue
        if ent.get("policy_hash") != pol.policy_hash:
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
    for f in _policy_files(policies_dir):
        pol = load_policy(f)
        key = lock_key(pol)
        if key in entries:
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
            pol = load_policy(f)
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

def policy_json_schema() -> dict:
    """The IR's JSON Schema - the grant's "DSL JSON Schema export", named
    `policy_dsl.schema.json` as in the reference package.

    It describes the canonical IR (local metres, post-projection), which is
    what a bundle ships. Authored YAML may also use lat/lon plus an origin;
    that form is accepted by load_policy() and projected before validation.
    """
    schema = Policy.model_json_schema()
    schema["$schema"] = "https://json-schema.org/draft/2020-12/schema"
    schema["x-ir-schema-version"] = IR_SCHEMA_VERSION
    schema["x-hash-scheme"] = HASH_SCHEME
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

def _main(argv: list[str] | None = None) -> int:
    """Emit a bundle from a policy file, or one of four maintenance commands.

        python -m guardrail.bundle policies/wgs84_taipei.yaml -m "initial"
        python -m guardrail.bundle verify bundles/wgs84-taipei-demo-v0.1.0.tar.gz
        python -m guardrail.bundle schema          # policies/policy_dsl.schema.json
        python -m guardrail.bundle lock [--update] # policies/policy.lock.json
        python -m guardrail.bundle keygen          # lab dev key, once per lab

    Bundles are build products, not source: they are byte-derivable from the
    policy (given the issue time), so `bundles/*.tar.gz` is gitignored here
    exactly as it is in the reference repository.
    """
    import argparse
    import sys

    argv = list(sys.argv[1:] if argv is None else argv)
    cmd = argv[0] if argv and argv[0] in ("schema", "keygen", "verify", "lock") else None

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

    ap = argparse.ArgumentParser(prog="python -m guardrail.bundle")
    ap.add_argument("policy")
    ap.add_argument("-o", "--out")
    ap.add_argument("-m", "--changelog", default="initial")
    ap.add_argument("--issued-at", default=None,
                    help="ISO-8601; default SOURCE_DATE_EPOCH, else now")
    ap.add_argument("--key", default=None, help="signing key (default: lab key)")
    ap.add_argument("--unsigned", action="store_true",
                    help="write an explicitly UNSIGNED bundle")
    args = ap.parse_args(argv)

    pol = load_policy(args.policy)
    # A bundle is a release: refuse to publish content under a version the lock
    # already gives to different content.
    ent = (load_lock().get("policies") or {}).get(lock_key(pol))
    if ent and "policy_hash" in ent and ent["policy_hash"] != pol.policy_hash:
        print(f"REFUSED  {lock_key(pol)} is locked to {ent['policy_hash']}, but "
              f"{args.policy} hashes to {pol.policy_hash}: raise its version.")
        return 2
    if ent is None:
        print(f"WARNING  {lock_key(pol)} is not in policies/policy.lock.json")
    out = Path(args.out) if args.out else (
        ROOT / "bundles" / f"{pol.policy_id}-v{pol.version}.tar.gz")
    signer = None if args.unsigned else (args.key or "auto")
    p = write_bundle(pol, out, args.changelog, issued_at=args.issued_at,
                     signer=signer)
    chk = check_bundle(p)               # never ship one that will not reload
    print(f"{p}  {p.stat().st_size} bytes")
    print(f"  policy_id   {chk.policy.policy_id}")
    print(f"  version     {chk.policy.version}  generation {chk.policy.generation}")
    print(f"  policy_hash {chk.policy.policy_hash}")
    print(f"  issued_at   {chk.manifest.get('issued_at')}")
    print(f"  rules       {len(chk.policy.constraints)}")
    print(f"  signature   {chk.signature.upper()}  - {chk.detail}")
    return 0


if __name__ == "__main__":
    import sys as _sys
    _sys.exit(_main())
