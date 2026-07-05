import math

import pytest
from pydantic import ValidationError
from vlaguard_common import (
    Action4D,
    DeterminismManifest,
    Topology,
    body_to_local_ned,
    canonicalize,
    policy_hash,
)


def test_canonicalize_is_order_independent():
    a = {"b": 1, "a": [3, 2, 1], "c": {"y": 1, "x": 2}}
    b = {"c": {"x": 2, "y": 1}, "a": [3, 2, 1], "b": 1}
    assert canonicalize(a) == canonicalize(b)


def test_policy_hash_is_stable_and_prefixed():
    ir = {"constraints": [{"id": "nfz-1", "type": "polygon_fence"}], "version": "0.3.0"}
    h1 = policy_hash(ir)
    h2 = policy_hash(dict(reversed(list(ir.items()))))
    assert h1 == h2
    assert h1.startswith("sha256:")
    assert len(h1) == len("sha256:") + 64


def test_manifest_hil_requires_realtime():
    with pytest.raises(ValidationError):
        DeterminismManifest(
            code_revision="abc",
            vla_model_hash="def",
            policy_hash="sha256:0",
            random_seed=1,
            sim_speedup=4.0,
            topology=Topology.HIL,
        )


def test_manifest_dev_allows_speedup():
    m = DeterminismManifest(
        code_revision="abc",
        vla_model_hash="def",
        policy_hash="sha256:0",
        random_seed=1,
        sim_speedup=8.0,
        topology=Topology.DEV,
    )
    assert m.sim_speedup == 8.0


def test_body_to_ned_identity_heading():
    # heading 0 (North): body-forward maps to +north, body-right to +east
    n, e, d, yr = body_to_local_ned(Action4D(vx=1, vy=0, vz=2, yaw_rate=0.5), yaw_rad=0.0)
    assert math.isclose(n, 1.0)
    assert math.isclose(e, 0.0, abs_tol=1e-9)
    assert math.isclose(d, -2.0)  # body up-positive -> NED down-negative
    assert yr == 0.5


def test_body_to_ned_quarter_turn():
    # heading 90deg East: body-forward maps to +east
    n, e, _, _ = body_to_local_ned(Action4D(vx=1, vy=0, vz=0, yaw_rate=0), yaw_rad=math.pi / 2)
    assert math.isclose(n, 0.0, abs_tol=1e-9)
    assert math.isclose(e, 1.0)
