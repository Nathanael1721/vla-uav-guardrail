"""
Recorded policies in the legacy metre form, for the tests of the declared
legacy read (WP1-14, third fix of 2026-11-26).

Since then bundle.load_for_flight / load_yaml_for_flight / load_bundle read a
policy in a projected form under `legacy_read` (or VLAGUARD_LEGACY_READ=1)
only when it is a recorded artefact: one form of its policy_hash is in
policies/policy.lock.json (bundle.recorded_as). A test of the read therefore
needs a policy the lock records, not one made up in the test. These are the
lock's own old versions, as bundle.policy_candidates rewinds them (the
frame migration's metre form, then each older version), so they are the
policies the runs of that time flew.
"""
from __future__ import annotations

import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail import bundle as B                                  # noqa: E402
from guardrail.models import LEGACY_POINT_FORMS, Policy            # noqa: E402

SITL_FILE = "sim_demo_policy.yaml"


def _version_key(v: str) -> tuple:
    return tuple(int(p) if p.isdigit() else -1 for p in v.split("."))


def recorded_metre_versions(file: str = SITL_FILE) -> list[tuple[str, Policy]]:
    """(lock key, policy) of every version of policies/<file> the lock
    records in the legacy metre form, oldest first."""
    out = []
    for label, pol in B.policy_candidates():
        name, sep, key = label.partition(" as ")
        if sep and name == file and pol.point_form in LEGACY_POINT_FORMS:
            out.append((key, pol))
    out.sort(key=lambda kp: _version_key(kp[1].version))
    assert out, f"the lock records no metre version of {file}"
    return out


def recorded_metre_policy(file: str = SITL_FILE) -> tuple[str, Policy]:
    """(lock key, policy) of the newest metre version of policies/<file>
    the lock records (for sim_demo_policy.yaml: fase3-sim-demo@0.1.2, locked
    under IR schema 3.0.0, the last version before the frame migration)."""
    return recorded_metre_versions(file)[-1]


def write_recorded_metre_yaml(folder: Path, file: str = SITL_FILE) -> tuple[str, Path]:
    """(lock key, path): that version written out as a policy YAML, as the
    file was at that version - points in {x, y}, no origin, `scope` left out
    (every document before IR schema 4.0.0 left it out)."""
    key, pol = recorded_metre_policy(file)
    p = Path(folder) / f"{pol.policy_id}-v{pol.version}.yaml"
    p.write_text(yaml.safe_dump(pol.model_dump(mode="json", exclude_none=True),
                                sort_keys=False), encoding="utf-8")
    return key, p


def write_recorded_metre_bundle(folder: Path, file: str = SITL_FILE) -> tuple[str, Path]:
    """(lock key, path): that version as an unsigned bundle, as the lab's
    bundles of the time hold it (bundles/<id>-v<version>.tar.gz)."""
    key, pol = recorded_metre_policy(file)
    p = Path(folder) / f"{pol.policy_id}-v{pol.version}.tar.gz"
    B.write_bundle(pol, p, signer=None, issued_at="2026-11-02T00:00:00Z",
                   allow_legacy_metres=True)
    return key, p
