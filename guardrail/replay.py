"""The replay bundle — WP4's deployable artefact, and the last named one missing.

    write_replay("demo/out/city_kpi", policy, "bundles/city_kpi.replay.tar.gz")
    ok, why = verify_replay("bundles/city_kpi.replay.tar.gz")

WHAT THE GRANT ASKS FOR

WP1 is accepted on "policy load round-trip; **bundle replayability**", and WP4's
artefact list names replay bundles beside the determinism manifest.
`guardrail/bundle.py` closed the policy half. This closes the flight half: one
archive holding the episode, the policy that governed it, the determinism
manifest, and the KPI table it produced.

WHAT "REPLAYABLE" HAD BETTER MEAN

Not "the files are in one place". A tar of four files nobody can re-derive
anything from would satisfy the word and none of the intent. So
`verify_replay()` reloads the policy from the archived IR, **recomputes the KPIs
from the archived log**, and compares them to the archived KPI table field by
field. A bundle that cannot reproduce its own headline numbers is refused.

That check has teeth on exactly one day: the day someone changes how a KPI is
computed without re-deriving the bundles that quote the old value. It passes
trivially today because `compute()` is deterministic, which is the point of
writing it now rather than after that day.

INTEGRITY, HONESTLY DESCRIBED

Every member is digested and the digests are listed in `replay.json`, which is
itself digested into `signature.txt` beside the same placeholder signer
`bundle.py` uses. This catches accidental corruption, a truncated download and a
hand-edited log. It does not catch someone who rewrites the log AND the index,
and it does not claim to — see the same note in `guardrail/bundle.py`.
"""
from __future__ import annotations

import gzip
import hashlib
import io
import json
import tarfile
from pathlib import Path
from typing import Any

from .bundle import _SIGNED_BY, manifest_for
from .kpi import compute, rule_priorities
from .models import Policy

INDEX_NAME = "replay.json"
SIGNATURE_NAME = "signature.txt"
EPISODE = "episode/"
POLICY = "policy/"

# The episode files, and whether a bundle may be written without each. The log
# is the one thing a replay bundle cannot be missing — without it there is
# nothing to replay and the artefact is a filing cabinet.
EPISODE_FILES = {
    "flight_log.jsonl": True,
    "metrics.json": False,
    "kpi.json": False,
    "manifest.json": False,
}

# KPI fields compared on verification. Deliberately the contractual figures and
# the counts they derive from, rather than every key: a bundle should not be
# refused because a later version of `compute()` added a field.
VERIFIED_KPI_FIELDS = (
    "p0_violation_escape_rate", "p0_ticks", "p0_ticks_not_measurable",
    "fail_safe_correctness", "mean_repair_magnitude_mps",
    "max_repair_magnitude_mps", "mean_time_to_safe_s",
)


def _digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _add(tar: tarfile.TarFile, name: str, data: bytes) -> None:
    info = tarfile.TarInfo(name=name)
    info.size = len(data)
    info.mtime = 0          # same episode -> same bytes; see bundle.py
    tar.addfile(info, io.BytesIO(data))


def _read(tar: tarfile.TarFile, name: str) -> bytes:
    try:
        member = tar.extractfile(name)
    except KeyError:
        member = None
    if member is None:
        raise ValueError(f"replay bundle missing required member: {name}")
    return member.read()


def write_replay(run_dir: str | Path, policy: Policy, out_path: str | Path,
                 changelog: str = "initial") -> Path:
    """Package a finished flight directory and its policy into one archive.

    The policy is stored as the same canonical IR `bundle.py` hashes, so the
    policy inside a replay bundle and the policy inside a policy bundle are the
    same bytes and reload through the same path.
    """
    run, out = Path(run_dir), Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    members: dict[str, bytes] = {}
    for name, required in EPISODE_FILES.items():
        f = run / name
        if f.is_file():
            members[EPISODE + name] = f.read_bytes()
        elif required:
            raise ValueError(
                f"{run} has no {name}; there is nothing to replay. A replay "
                f"bundle without the episode log is a filing cabinet.")

    # The policy has to be the one that actually governed this flight. Without
    # this, any policy bundles with any episode and `verify_replay` still says
    # yes, because the KPI recompute uses whatever priorities it is handed - so
    # a bundle could ship a clean escape rate derived under rules the aircraft
    # never flew under. The run's own manifest already records the hash.
    run_manifest = members.get(EPISODE + "manifest.json")
    if run_manifest is not None:
        flown = json.loads(run_manifest).get("policy_hash")
        if flown and flown != policy.policy_hash:
            raise ValueError(
                f"{run.name} was flown under policy {flown}, but the policy "
                f"passed here hashes to {policy.policy_hash}. A replay bundle "
                f"must carry the policy that governed the episode.")

    ir = json.dumps(policy.model_dump(mode="json"), sort_keys=True,
                    separators=(",", ":")).encode()
    members[POLICY + "ir.json"] = ir
    members[POLICY + "manifest.json"] = json.dumps(
        manifest_for(policy, changelog), indent=2, sort_keys=True).encode()

    index = {
        "kind": "vla-guardrail-replay",
        "version": 1,
        "run": run.name,
        "policy_id": policy.policy_id,
        "policy_hash": policy.policy_hash,
        "changelog": changelog,
        "members": {k: _digest(v) for k, v in sorted(members.items())},
    }
    index_bytes = json.dumps(index, indent=2, sort_keys=True).encode()
    signature = f"{_digest(index_bytes)} {_SIGNED_BY}\n".encode()

    with open(out, "wb") as fh:
        with gzip.GzipFile(fileobj=fh, mode="wb", mtime=0, filename="") as gz:
            with tarfile.open(fileobj=gz, mode="w") as tar:
                for name, data in sorted(members.items()):
                    _add(tar, name, data)
                _add(tar, INDEX_NAME, index_bytes)
                _add(tar, SIGNATURE_NAME, signature)
    return out


def read_replay(path: str | Path) -> dict[str, Any]:
    """Open a bundle, check every digest, and return its parts.

    Raises `ValueError` on the first member whose bytes do not match the index,
    naming it. Returns `{"index", "policy", "rows", "metrics", "kpi",
    "manifest"}`.
    """
    with tarfile.open(path, "r:gz") as tar:
        index_bytes = _read(tar, INDEX_NAME)
        sig = _read(tar, SIGNATURE_NAME).decode().split()
        if not sig or sig[0] != _digest(index_bytes):
            raise ValueError(
                f"{path}: signature does not match the index; the bundle has "
                f"been modified or truncated.")
        index = json.loads(index_bytes)
        blobs: dict[str, bytes] = {}
        for name, want in index["members"].items():
            data = _read(tar, name)
            got = _digest(data)
            if got != want:
                raise ValueError(
                    f"{path}: member {name!r} does not match its digest "
                    f"(index {want[:12]}..., archive {got[:12]}...).")
            blobs[name] = data

    def _json(name, default=None):
        b = blobs.get(name)
        return default if b is None else json.loads(b)

    # model_validate, not load_policy: the IR is already projected and
    # validated, exactly as load_bundle does it.
    policy = Policy.model_validate(_json(POLICY + "ir.json"))
    if policy.policy_hash != index["policy_hash"]:
        raise ValueError(
            f"{path}: the policy rebuilt from ir.json hashes to "
            f"{policy.policy_hash}, but the index says {index['policy_hash']}.")
    rows = [json.loads(line) for line
            in blobs[EPISODE + "flight_log.jsonl"].decode("utf-8").splitlines()
            if line.strip()]
    return {"index": index, "policy": policy, "rows": rows,
            "metrics": _json(EPISODE + "metrics.json", {}),
            "kpi": _json(EPISODE + "kpi.json"),
            "manifest": _json(EPISODE + "manifest.json")}


def verify_replay(path: str | Path) -> tuple[bool, list[str]]:
    """Can the bundle still produce the numbers it ships? `(ok, reasons)`.

    Digests, the policy round-trip, and then the part that earns the name: the
    KPIs are recomputed from the archived log and compared to the archived KPI
    table. `True` with an empty list means a reader can re-derive every
    contractual figure in the bundle from the bundle alone.
    """
    try:
        got = read_replay(path)
    except (ValueError, KeyError, tarfile.TarError, OSError) as exc:
        return False, [str(exc)]

    flown = (got["manifest"] or {}).get("policy_hash")
    if flown and flown != got["policy"].policy_hash:
        return False, [f"episode flown under policy {flown}, bundle carries "
                       f"{got['policy'].policy_hash}"]

    stored = got["kpi"]
    if stored is None:
        # Honest, and not a failure: an episode may be bundled before it is
        # scored. Say so rather than passing silently.
        return True, ["no kpi.json in the bundle; nothing to re-derive"]

    fresh = compute(got["rows"], rule_priorities(got["policy"]), got["metrics"])
    bad = []
    for field in VERIFIED_KPI_FIELDS:
        if field not in stored:
            continue
        a, b = stored.get(field), fresh.get(field)
        if isinstance(a, float) and isinstance(b, float):
            if abs(a - b) > 1e-9:
                bad.append(f"{field}: bundle {a}, recomputed {b}")
        elif a != b:
            bad.append(f"{field}: bundle {a!r}, recomputed {b!r}")
    return (not bad), bad
