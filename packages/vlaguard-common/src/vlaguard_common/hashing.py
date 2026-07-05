"""``policy_hash`` — the load-bearing reproducibility primitive.

Per ``docs/02-implementation/policy-dsl.md`` ("Versioning and signing"), every
published policy bundle carries a ``policy_hash`` that is a SHA-256 over the
*canonicalised* IR (post-merge, pre-sign). Every downstream artefact (CSP,
Shield audit record, episode bundle) records this hash so a flight can be
replayed by loading the bundle whose hash matches — never "the latest".

Canonicalisation must be deterministic across processes and Python versions, so
we sort object keys and use a compact, ASCII-safe separator set.
"""

from __future__ import annotations

import hashlib
import json
from typing import Any

_HASH_PREFIX = "sha256:"


def canonicalize(obj: Any) -> bytes:
    """Serialize ``obj`` to canonical JSON bytes.

    Keys are sorted, whitespace is stripped, and non-ASCII is escaped so the
    byte string is identical regardless of dict insertion order or platform.
    """
    return json.dumps(
        obj,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        default=_fallback,
    ).encode("ascii")


def policy_hash(canonical_ir: Any) -> str:
    """Return ``sha256:<hex>`` over the canonicalised IR.

    Accepts either an already-canonical ``bytes`` blob or any JSON-serialisable
    object (which is canonicalised first).
    """
    blob = canonical_ir if isinstance(canonical_ir, bytes) else canonicalize(canonical_ir)
    return _HASH_PREFIX + hashlib.sha256(blob).hexdigest()


def _fallback(value: Any) -> Any:
    """Make pydantic models / sets / tuples JSON-serialisable, deterministically."""
    if hasattr(value, "model_dump"):
        return value.model_dump(mode="json")
    if isinstance(value, (set, frozenset)):
        return sorted(value)
    raise TypeError(f"Cannot canonicalize value of type {type(value)!r}")
