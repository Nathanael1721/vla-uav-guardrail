"""Shared contracts for the VLA guardrail stack.

This package holds the cross-cutting invariants from the design docs
(``docs/02-implementation/overview.md``, ``docs/03-simulation/data-flow.md``):

* the six-field determinism manifest (:mod:`vlaguard_common.manifest`),
* the ``policy_hash`` canonicalisation + hashing util (:mod:`vlaguard_common.hashing`),
* the 4-D body-frame action contract and body<->local-NED conversion
  (:mod:`vlaguard_common.frames`).

Every other package imports these types; nothing here imports anything else
in the workspace, so it sits at the bottom of the dependency graph.
"""

from vlaguard_common.frames import Action4D, body_to_local_ned
from vlaguard_common.hashing import canonicalize, policy_hash
from vlaguard_common.manifest import DeterminismManifest, Topology

__all__ = [
    "Action4D",
    "body_to_local_ned",
    "canonicalize",
    "policy_hash",
    "DeterminismManifest",
    "Topology",
]
