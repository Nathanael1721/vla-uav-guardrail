"""The grant labels beside a Project AirSim scene that ArduPilot flies
(guardrail.manifest.build_manifest, scene_controllers).

    python tests/test_railinputs_manifest.py      (no pytest needed)

build_manifest refused dev, hil and flight beside any Project AirSim scene: a
guard written when every scene meant simple_flight, the simulator's own
controller. On the Project AirSim + ArduPilot rail the scene's robot config
names Project AirSim's ArduCopterApi ("ardupilot-api"), so ArduPilot SITL
flies the simulated vehicle, and the rail stamped its own label instead
(demo/pas_ardupilot/topology.py, "What manifest.py needs"). Now dev is
accepted there when the controller READ from the scene's files is
ardupilot-api AND the live MAVROS evidence holds; a simple_flight scene, a
pymavlink link, missing MAVROS evidence and an unreadable robot config are
refused, hil still needs the Orin's evidence, and flight never has a scene.

Shown failing first against a pristine HEAD 4418ae0 copy (git archive): the
two acceptance tests raise ValueError ("has no simulator scene file"), and
scene_controllers does not exist.
"""
from __future__ import annotations

import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail.manifest import (TOPOLOGY_DEV, TOPOLOGY_HIL,           # noqa: E402
                                build_manifest)

AP_SCENE = ROOT / "demo" / "pas_ardupilot" / "scene_citylife_ardupilot.jsonc"
SF_SCENE = ROOT / "demo" / "pas_config" / "scene_semantic.jsonc"
LINK = {"fcu_url": "udp://:14555@", "peer": None, "peer_is_loopback": True,
        "iface": None, "link_kind": None}
MAVROS = {"ros_distro": "jazzy", "mavros_node": "/mavros", "fcu_connected": True,
          "ardupilot_version": "ArduCopter V4.5.7 (2a3dc4b7)",
          "network_link": LINK, "autopilot_kind": "sitl"}
ORIN = {"host_arch": "aarch64", "device_model": "NVIDIA Jetson AGX Orin Developer Kit",
        "l4t_release": "R36.4.3", "jetpack": "6.2",
        "network_link": {"fcu_url": "udp://:14555@192.168.10.2:14550",
                         "peer": "192.168.10.2", "peer_is_loopback": False,
                         "iface": "eth0", "link_kind": "ethernet"}}


def _man(scene, topology=TOPOLOGY_DEV, ev=MAVROS):
    return build_manifest(policy_hash="sha256:deadbeefdeadbeef",
                          model_id="guardrail.vla_stub.StubVLA", seed=0,
                          scene_path=scene, topology=topology, hil_evidence=ev)


def _refused(scene, topology=TOPOLOGY_DEV, ev=MAVROS) -> str:
    try:
        _man(scene, topology, ev)
    except ValueError as e:
        return str(e)
    raise AssertionError(f"{topology!r} was accepted beside {scene} with {ev}")


def test_the_controller_is_read_from_the_scenes_own_files():
    from guardrail.manifest import scene_controllers
    found, problems = scene_controllers(AP_SCENE)
    assert problems == [] and [r["controller"] for r in found] == ["ardupilot-api"]
    found, _ = scene_controllers(SF_SCENE)
    assert [r["controller"] for r in found] == ["simple-flight-api"], found
    alone = Path(tempfile.mkdtemp(prefix="scene_")) / AP_SCENE.name
    shutil.copy(AP_SCENE, alone)                # no robot config beside it
    found, problems = scene_controllers(alone)
    assert found == [] and "could not be read" in problems[0], problems


def test_dev_is_accepted_beside_an_ardupilot_flown_scene_with_live_mavros():
    m = _man(AP_SCENE)
    assert m["topology"] == "dev" and len(m) == 6, m
    # sim_speedup still comes from the scene's own clock, never asserted.
    assert m["sim_speedup"] == 1.0, m


def test_hil_beside_the_scene_needs_the_orin_and_is_accepted_with_it():
    why = _refused(AP_SCENE, TOPOLOGY_HIL)
    assert "Orin" in why, why
    m = _man(AP_SCENE, TOPOLOGY_HIL, {**MAVROS, **ORIN})
    assert m["topology"] == "hil", m


def test_a_simple_flight_scene_is_still_refused_whatever_the_evidence():
    why = _refused(SF_SCENE)
    assert "simple-flight-api" in why and "projectairsim-single-host" in why, why
    assert "simple-flight-api" in _refused(SF_SCENE, TOPOLOGY_HIL, {**MAVROS, **ORIN})


def test_pymavlink_or_missing_mavros_evidence_is_refused_beside_the_scene():
    assert "pymavlink" in _refused(AP_SCENE, ev={**MAVROS, "link": "pymavlink"})
    assert "fcu_url" in _refused(AP_SCENE, ev={k: v for k, v in MAVROS.items()
                                               if k != "network_link"})
    assert "fcu_connected" in _refused(AP_SCENE, ev={**MAVROS, "fcu_connected": False})
    assert "no evidence supplied" in _refused(AP_SCENE, ev=None)
    alone = Path(tempfile.mkdtemp(prefix="scene_")) / AP_SCENE.name
    shutil.copy(AP_SCENE, alone)
    assert "could not be read" in _refused(alone)


def test_flight_never_has_a_simulator_scene():
    ev = {**MAVROS, **ORIN, "autopilot_kind": "hardware"}
    assert "real ArduPilot airframe" in _refused(AP_SCENE, "flight", ev)
    assert build_manifest(policy_hash="sha256:x", model_id="guardrail.vla_stub.StubVLA",
                          seed=0, scene_path=None, topology="flight",
                          hil_evidence=ev)["topology"] == "flight"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        # SystemExit too: a rail whose parser refuses a flag exits, and
        # that is this test failing, not the run ending.
        except (Exception, SystemExit) as e:         # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
