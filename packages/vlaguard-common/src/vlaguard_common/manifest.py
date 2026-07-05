"""The six-field determinism manifest.

From ``docs/03-simulation/data-flow.md`` ("Determinism contract"): every recorded
episode is reproducible bit-for-bit from six fields, emitted by the orchestrator
at episode start and stored as the first frame of every rosbag. A run that cannot
produce all six values is rejected from the KPI report.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Self

from pydantic import BaseModel, Field, model_validator


class Topology(StrEnum):
    """The three first-class deployment topologies (architecture-constraints.md)."""

    DEV = "dev"
    HIL = "hil"
    FLIGHT = "flight"


class DeterminismManifest(BaseModel):
    """The contract that gates a run into the KPI report.

    ``sim_speedup`` MUST be ``1.0`` for the ``hil`` topology (the canonical KPI
    configuration); the validator enforces this so a non-reproducible HIL run
    cannot be silently accepted.
    """

    model_config = {"frozen": True}

    code_revision: str = Field(description="git SHA across the entire stack")
    vla_model_hash: str = Field(description="SHA-256 of the VLA model weights")
    policy_hash: str = Field(description="hash of the active policy bundle")
    random_seed: int = Field(description="integer seed, applied uniformly")
    sim_speedup: float = Field(default=1.0, gt=0.0)
    topology: Topology

    @model_validator(mode="after")
    def _hil_must_be_realtime(self) -> Self:
        if self.topology == Topology.HIL and self.sim_speedup != 1.0:
            raise ValueError("sim_speedup must be 1.0 for the hil (KPI) topology")
        return self
