"""Cold-start ingest pipeline and the signed policy bundle (policy-dsl.md).

Pipeline (cold-start path only for the Phase-1 slice):
parse + validate -> geometry normalise -> build runtime IR (index + SDF) ->
sign + version -> emit ``tar.gz`` bundle.

The hot-apply path (REST endpoint, generation bump) is Phase 2.
"""

from __future__ import annotations

import io
import json
import tarfile
from pathlib import Path
from typing import Any

import yaml

from policy_dsl.ir import PolicyIR, build_ir
from policy_dsl.models import PolicyDoc

# Placeholder signing identity — the real CA (lab vs ITRI) is an open question
# tracked in policy-dsl.md. Swap this for a detached CA signature in deployment.
_SIGNED_BY = "lab-ca:dev-placeholder"


def parse_doc(raw: str) -> PolicyDoc:
    """Parse YAML/JSON source into a validated :class:`PolicyDoc`."""
    data = yaml.safe_load(raw)
    return PolicyDoc.model_validate(data)


def ingest_text(raw: str) -> PolicyIR:
    return build_ir(parse_doc(raw))


def ingest_file(path: str | Path) -> PolicyIR:
    return ingest_text(Path(path).read_text())


def _manifest(ir: PolicyIR, changelog: str) -> dict[str, Any]:
    return {
        "policy_id": ir.policy_id,
        "version": ir.version,
        "generation": ir.generation,
        "policy_hash": ir.policy_hash,
        "signed_by": _SIGNED_BY,
        "changelog": changelog,
    }


def write_bundle(ir: PolicyIR, out_path: str | Path, changelog: str = "initial") -> Path:
    """Write a ``tar.gz`` bundle: canonical IR + manifest + signature."""
    out = Path(out_path)
    manifest = _manifest(ir, changelog)
    ir_bytes = json.dumps(ir.canonical, sort_keys=True, separators=(",", ":")).encode()
    manifest_bytes = json.dumps(manifest, indent=2, sort_keys=True).encode()
    # Detached signature placeholder: binds policy_hash to the signing identity.
    signature = f"{ir.policy_hash} {_SIGNED_BY}\n".encode()

    with tarfile.open(out, "w:gz") as tar:
        _add(tar, "ir.json", ir_bytes)
        _add(tar, "manifest.json", manifest_bytes)
        _add(tar, "signature.txt", signature)
    return out


def load_bundle(path: str | Path) -> PolicyIR:
    """Load a bundle, rebuild the runtime IR, and verify the hash matches.

    Rebuilding the shapely indices from the canonical IR is *not* re-parsing the
    DSL source — the canonical IR is the post-validation single source of truth.
    """
    with tarfile.open(path, "r:gz") as tar:
        canonical = json.loads(_read(tar, "ir.json"))
        manifest = json.loads(_read(tar, "manifest.json"))

    ir = build_ir(PolicyDoc.model_validate(canonical))
    if ir.policy_hash != manifest["policy_hash"]:
        raise ValueError(
            f"policy_hash mismatch: bundle manifest {manifest['policy_hash']} "
            f"!= rebuilt {ir.policy_hash} (bundle is corrupt or tampered)"
        )
    return ir


def _add(tar: tarfile.TarFile, name: str, data: bytes) -> None:
    info = tarfile.TarInfo(name=name)
    info.size = len(data)
    info.mtime = 0  # deterministic archive
    tar.addfile(info, io.BytesIO(data))


def _read(tar: tarfile.TarFile, name: str) -> bytes:
    member = tar.extractfile(name)
    if member is None:
        raise ValueError(f"bundle missing required member: {name}")
    return member.read()
