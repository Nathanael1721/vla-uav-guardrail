"""The signed policy bundle — WP1's deployable artefact.

    write_bundle(policy, "bundles/demo.tar.gz", changelog="added school curfew")
    policy = load_bundle("bundles/demo.tar.gz")     # refuses if tampered

WHAT THE GRANT ASKS FOR

WP1's deliverable line reads: "YAML/JSON DSL -> IR/AST; **signed policy bundle**
(hash + semver) covering GeoFence, envelope, corridor, time windows, breach
actions", accepted on "Policy load round-trip; bundle replayability". Until now
this project hashed policies in memory and never produced the artefact.

THE FORMAT IS THE REFERENCE'S, DELIBERATELY

`packages/policy-dsl/src/policy_dsl/ingest.py` already writes one, and matching
it byte-for-byte is worth more than any improvement: `ir.json`, `manifest.json`,
`signature.txt`, member mtimes pinned to 0 so the archive is reproducible.
Inventing a second, nicer layout would mean the two halves of the same grant
could not read each other's bundles.

ABOUT THE "SIGNATURE"

It is a placeholder, and so is the reference's - `_SIGNED_BY =
"lab-ca:dev-placeholder"`, with a comment recording that the real CA (lab or
ITRI) is an open question. Carried forward unchanged rather than quietly
upgraded: inventing a key-management story the project has not decided on would
put a real-looking signature on a bundle nobody can actually verify, which is
worse than an honest placeholder that says what it is.

What the bundle DOES guarantee today is integrity against accident and against
silent edits: `load_bundle` rebuilds the policy from the canonical IR and
recomputes its hash, refusing the bundle if the manifest disagrees. That catches
a corrupted archive, a hand-edited `ir.json`, and a manifest copied from another
policy. It does not catch an attacker who can rewrite both, and it does not
claim to.
"""
from __future__ import annotations

import gzip
import io
import json
import tarfile
from pathlib import Path
from typing import Any

from .models import Policy

# Placeholder signing identity, matching the reference implementation exactly.
# The real CA is an open question tracked in policy-dsl.md; swap this for a
# detached CA signature at deployment.
_SIGNED_BY = "lab-ca:dev-placeholder"

IR_NAME = "ir.json"
MANIFEST_NAME = "manifest.json"
SIGNATURE_NAME = "signature.txt"


def manifest_for(policy: Policy, changelog: str = "initial") -> dict[str, Any]:
    """The bundle's identity card: who, what version, which rules, what changed."""
    return {
        "policy_id": policy.policy_id,
        "version": policy.version,
        "generation": policy.generation,
        "policy_hash": policy.policy_hash,
        "signed_by": _SIGNED_BY,
        "changelog": changelog,
    }


def _add(tar: tarfile.TarFile, name: str, data: bytes) -> None:
    info = tarfile.TarInfo(name=name)
    info.size = len(data)
    info.mtime = 0          # deterministic archive: same policy -> same bytes
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
                 changelog: str = "initial") -> Path:
    """Write a tar.gz containing the canonical IR, the manifest and the signature.

    The IR is serialised with sorted keys and no whitespace - the same
    canonicalisation `Policy.policy_hash` hashes - so the bytes in the archive
    are the bytes the hash describes rather than a prettier rendering of them.
    """
    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    ir_bytes = json.dumps(policy.model_dump(mode="json"), sort_keys=True,
                          separators=(",", ":")).encode()
    manifest = manifest_for(policy, changelog)
    manifest_bytes = json.dumps(manifest, indent=2, sort_keys=True).encode()
    signature = f"{policy.policy_hash} {_SIGNED_BY}\n".encode()

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
    with open(out, "wb") as fh:
        with gzip.GzipFile(fileobj=fh, mode="wb", mtime=0, filename="") as gz:
            with tarfile.open(fileobj=gz, mode="w") as tar:
                _add(tar, IR_NAME, ir_bytes)
                _add(tar, MANIFEST_NAME, manifest_bytes)
                _add(tar, SIGNATURE_NAME, signature)
    return out


def load_bundle(path: str | Path) -> Policy:
    """Load a bundle, rebuild the Policy, and REFUSE it if the hash disagrees.

    Rebuilding from the canonical IR is not re-parsing the DSL: the IR is the
    post-validation single source of truth, which is the whole point of shipping
    it rather than the YAML.

    Three things are checked, and each has caught a different kind of mistake in
    other projects: the manifest hash against the rebuilt policy (a tampered or
    corrupt IR), the signature line against the manifest (a manifest swapped in
    from another bundle), and the presence of every required member (a truncated
    archive). A bundle that fails any of them raises rather than loading a
    policy the operator did not authorise.
    """
    p = Path(path)
    with tarfile.open(p, "r:gz") as tar:
        canonical = json.loads(_read(tar, IR_NAME))
        manifest = json.loads(_read(tar, MANIFEST_NAME))
        signature = _read(tar, SIGNATURE_NAME).decode().strip()

    # model_validate, not load_policy: the IR is already projected and
    # validated, and running it back through the lat/lon loader would be a
    # second chance to move the geometry.
    policy = Policy.model_validate(canonical)

    if policy.policy_hash != manifest.get("policy_hash"):
        raise ValueError(
            f"{p.name}: policy_hash mismatch - manifest says "
            f"{manifest.get('policy_hash')}, the IR in this bundle hashes to "
            f"{policy.policy_hash}. The bundle is corrupt or was edited.")

    sig_hash = signature.split(" ", 1)[0] if signature else ""
    if sig_hash != manifest.get("policy_hash"):
        raise ValueError(
            f"{p.name}: signature covers {sig_hash!r} but the manifest declares "
            f"{manifest.get('policy_hash')!r} - the manifest does not belong to "
            f"this signature.")
    return policy


def _main(argv: list[str] | None = None) -> int:
    """Emit a bundle from a policy file.

        python -m guardrail.bundle policies/wgs84_taipei.yaml -m "initial"

    Bundles are build products, not source: they are byte-derivable from the
    policy, so `bundles/` is gitignored here exactly as it is in the reference
    repository. What belongs in version control is the YAML they came from.
    """
    import argparse
    from .models import load_policy

    ap = argparse.ArgumentParser(prog="python -m guardrail.bundle")
    ap.add_argument("policy")
    ap.add_argument("-o", "--out")
    ap.add_argument("-m", "--changelog", default="initial")
    args = ap.parse_args(argv)

    pol = load_policy(args.policy)
    out = Path(args.out) if args.out else (
        Path(__file__).resolve().parents[1] / "bundles" /
        f"{pol.policy_id}-v{pol.version}.tar.gz")
    p = write_bundle(pol, out, args.changelog)
    back = load_bundle(p)               # never ship one that will not reload
    print(f"{p}  {p.stat().st_size} bytes")
    print(f"  policy_id   {back.policy_id}")
    print(f"  version     {back.version}  generation {back.generation}")
    print(f"  policy_hash {back.policy_hash}")
    print(f"  rules       {len(back.constraints)}")
    return 0


if __name__ == "__main__":
    import sys as _sys
    _sys.exit(_main())
