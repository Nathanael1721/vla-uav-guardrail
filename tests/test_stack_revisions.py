"""code_revision host by host: the stack record beside the six-field manifest
(ARCH-31, guardrail.manifest STACK_FILE).

Run either way:
    pytest tests/test_stack_revisions.py -v
    python tests/test_stack_revisions.py

The grant's determinism contract names one code_revision, a "single git SHA
across the entire stack". A hil run spans the Orin (the Shield and the VLA),
the desktop (the harness, SITL's helper, the stressors) and ArduPilot's own
firmware, and manifest.json said only the revision of the host that wrote it.
These tests pin the record that says the rest: which host ran which commit,
whether they agree on a clean tree, which firmware build flew - while
manifest.json stays exactly the grant's six fields and a run written before
the record (run manifest schema 1) loads as it always did.
"""
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import manifest as M                                   # noqa: E402

SKIP = "SKIP"
GRANT_FIELDS = ["code_revision", "vla_model_hash", "policy_hash", "random_seed",
                "sim_speedup", "topology"]
REV = "a1b2c3d4e5f6"
ORIN = {"host_arch": "aarch64", "device_model": "NVIDIA Jetson AGX Orin Developer Kit",
        "l4t_release": "R38.4.0", "jetpack": "7.2.1", "hostname": "agx"}
DESKTOP = {"host_arch": "AMD64", "device_model": None, "l4t_release": None,
           "jetpack": None, "hostname": "desktop"}
# The MAVROS path's evidence on the pinned SITL (sitl/mavros_evidence.py).
MAVROS_EV = {**ORIN, "ardupilot_version": "ArduCopter V4.5.7 (2a3dc4b7)",
             "ardupilot_version_source": "AUTOPILOT_VERSION via /mavros/vehicle_info_get",
             "flight_sw_version": "040507ff", "flight_custom_version_raw": "3762336134336132",
             "autopilot_kind": "sitl"}


def _no_describe(repo=None):
    return "v-test-" + REV


def _manifest(rev=REV):
    return {"code_revision": rev, "vla_model_hash": "sha256:m", "policy_hash": "sha256:p",
            "random_seed": 7, "sim_speedup": 1.0, "topology": "hil"}


def _run(man=None, metrics=None) -> Path:
    d = Path(tempfile.mkdtemp(prefix="stack_"))
    if man is not None:
        (d / "manifest.json").write_text(json.dumps(man, indent=2), encoding="utf-8")
    if metrics is not None:
        (d / "metrics.json").write_text(json.dumps(metrics), encoding="utf-8")
    return d


# ------------------------------------------------------------- the autopilot

def test_the_autopilot_is_named_as_it_named_itself():
    a = M.autopilot_revision(MAVROS_EV)
    assert a["firmware"] == "ArduCopter V4.5.7 (2a3dc4b7)"
    assert a["version"] == "4.5.7" and a["git_hash"] == "2a3dc4b7", a
    assert a["flight_sw_version"] == "040507ff" and a["kind"] == "sitl"
    assert "vehicle_info_get" in a["source"] and "pin_ok" not in a


def test_a_dev_build_and_the_banner_fallback_keep_their_source():
    a = M.autopilot_revision({"ardupilot_version": "ArduCopter V4.8.0-dev (5f119834)",
                              "ardupilot_version_source": "sitl/setup_sitl.sh --verify "
                                                          "binary banner",
                              "ardupilot_pin_ok": True})
    assert a["version"] == "4.8.0-dev" and a["git_hash"] == "5f119834", a
    assert a["pin_ok"] is True and "banner" in a["source"]


def test_an_autopilot_that_never_answered_is_not_named():
    """Nothing is filled in from what setup_sitl.sh pins."""
    assert M.autopilot_revision({**ORIN, "ardupilot_version": None}) is None
    assert M.autopilot_revision(None) is None
    assert M.sitl_simulator(None) is None


def test_sitl_is_its_own_simulator_build_and_hardware_is_none():
    a = M.autopilot_revision(MAVROS_EV)
    s = M.sitl_simulator(a)
    assert s["name"] == "ArduPilot SITL" and s["build"] == a["firmware"]
    assert s["git_hash"] == "2a3dc4b7" and s["physics_model"] == M.UNRESOLVED
    assert M.sitl_simulator({**a, "kind": "hardware"}) is None


def test_project_airsim_records_its_client_and_scene_digest():
    import hashlib
    scene = ROOT / "demo" / "pas_config" / "scene_semantic.jsonc"
    s = M.projectairsim_simulator(scene, version=lambda name: "9.9.9")
    assert s["name"] == "Project AirSim" and s["client_version"] == "9.9.9"
    assert s["scene"] == scene.name
    assert s["scene_sha256"] == hashlib.sha256(scene.read_bytes()).hexdigest()
    assert s["server_build"] == M.UNRESOLVED

    def missing(name):
        raise LookupError(name)
    assert M.projectairsim_simulator(scene, version=missing)["client_version"] is None


# ------------------------------------------------------------------ the hosts

def test_the_host_role_is_read_from_the_device_tree():
    assert M.host_role(ORIN) == M.ROLE_COMPANION
    assert M.host_role({**DESKTOP, "l4t_release": "R36.3.0"}) == M.ROLE_COMPANION
    assert M.host_role(DESKTOP) == M.ROLE_DESKTOP and M.host_role(None) == M.ROLE_DESKTOP


def test_a_host_revision_carries_the_dirty_flag_and_describe():
    r = M.host_revision(host=DESKTOP, revision=REV + "-dirty", describe=_no_describe)
    assert r["code_revision"] == REV + "-dirty" and r["dirty"] is True
    assert r["git_describe"] == "v-test-" + REV and r["hostname"] == "desktop"
    assert M.host_revision(host=DESKTOP, revision=REV, describe=_no_describe)["dirty"] is False
    for rev in (REV + "-unknown", "unversioned"):
        assert M.host_revision(host=DESKTOP, revision=rev,
                               describe=_no_describe)["dirty"] is None


def test_git_describe_names_a_repository_that_holds_the_shield():
    if shutil.which("git") is None:
        print("      SKIP: git is not installed")
        return SKIP
    repo = Path(tempfile.mkdtemp(prefix="desc_"))
    try:
        (repo / "guardrail").mkdir()
        (repo / "guardrail" / "shield.py").write_text("# sentinel\n", encoding="utf-8")
        git = ["git", "-C", str(repo), "-c", "user.name=t", "-c", "user.email=t@t",
               "-c", "commit.gpgsign=false"]
        subprocess.run(git[:3] + ["init", "-q"], check=True)
        subprocess.run(git + ["add", "guardrail/shield.py"], check=True)
        subprocess.run(git + ["commit", "-q", "-m", "t"], check=True)
        head = subprocess.run(git[:3] + ["rev-parse", "HEAD"], capture_output=True,
                              text=True, check=True).stdout.strip()
        assert M.git_describe(repo) == head[:12], M.git_describe(repo)
        subprocess.run(git + ["tag", "v9.9.9"], check=True)
        assert M.git_describe(repo) == "v9.9.9", M.git_describe(repo)
        assert M.code_revision(repo) == head[:12]
    finally:
        shutil.rmtree(repo, ignore_errors=True)


# ------------------------------------------------------- the record on disk

def test_the_node_writes_the_record_beside_an_unchanged_manifest():
    man = _manifest()
    d = _run(man)
    try:
        doc = M.write_stack_revisions(d, man, MAVROS_EV, describe=_no_describe)
        on_disk = json.loads((d / M.STACK_FILE).read_text(encoding="utf-8"))
        assert on_disk == doc and doc["schema"] == M.STACK_SCHEMA
        assert doc["run_manifest_schema"] == 2 and doc["code_revision"] == REV
        assert list(doc["hosts"]) == ["companion"], doc["hosts"]
        comp = doc["hosts"]["companion"]
        assert comp["code_revision"] == REV and comp["hostname"] == "agx"
        assert comp["device_model"].startswith("NVIDIA Jetson")
        assert doc["autopilot"]["git_hash"] == "2a3dc4b7"
        assert doc["simulator"]["name"] == "ArduPilot SITL"
        assert doc["single_sha"] is True and doc["problems"] == [], doc["problems"]
        man_disk = json.loads((d / "manifest.json").read_text(encoding="utf-8"))
        assert list(man_disk) == GRANT_FIELDS and man_disk == man
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_the_desktop_joins_and_the_record_says_whether_one_sha_holds():
    man = _manifest()
    for desk_rev, single, said in ((REV, True, None),
                                   ("0123456789ab", False, "desktop runs '0123456789ab'"),
                                   (REV + "-dirty", False, "not a clean tree")):
        d = _run(man)
        try:
            M.write_stack_revisions(d, man, MAVROS_EV, describe=_no_describe)
            doc = M.add_host_revision(d, M.ROLE_DESKTOP, M.host_revision(
                host=DESKTOP, revision=desk_rev, describe=_no_describe))
            assert set(doc["hosts"]) == {"companion", "desktop"}
            assert doc["single_sha"] is single, (desk_rev, doc)
            if said is None:
                assert doc["problems"] == [], doc["problems"]
            else:
                assert any(said in p for p in doc["problems"]), doc["problems"]
            loaded = M.load_run_manifest(d)
            assert loaded["run_manifest_schema"] == 2 and loaded["stack"] == doc
            assert M.revisions_by_host(loaded) == {"companion": REV, "desktop": desk_rev}
        finally:
            shutil.rmtree(d, ignore_errors=True)


def test_a_run_written_before_the_record_loads_as_schema_1():
    man = _manifest("116214f3e6ee")
    d = _run(man, {"hil_evidence": MAVROS_EV})
    try:
        loaded = M.load_run_manifest(d)
        assert loaded == {"run_manifest_schema": 1, "manifest": man, "stack": None}
        assert M.revisions_by_host(loaded) == {"manifest": "116214f3e6ee"}
        # The harness adds the desktop to such a run: the record is built from
        # the manifest and the autopilot the metrics recorded; the host that
        # wrote the manifest is not listed, since only it could say which it was.
        doc = M.add_host_revision(d, M.ROLE_DESKTOP, M.host_revision(
            host=DESKTOP, revision="116214f3e6ee", describe=_no_describe))
        assert list(doc["hosts"]) == ["desktop"] and doc["code_revision"] == "116214f3e6ee"
        assert doc["autopilot"]["firmware"] == "ArduCopter V4.5.7 (2a3dc4b7)"
        assert M.load_run_manifest(d)["run_manifest_schema"] == 2
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_a_run_without_a_manifest_says_so_and_unknown_schemas_are_refused():
    d = _run(None)
    try:
        doc = M.add_host_revision(d, M.ROLE_DESKTOP, M.host_revision(
            host=DESKTOP, revision=REV, describe=_no_describe))
        assert doc["single_sha"] is False and doc["autopilot"] is None
        assert any("manifest's code_revision did not resolve" in p for p in doc["problems"])
        assert any("did not name its firmware" in p for p in doc["problems"])
        (d / "manifest.json").write_text(json.dumps(_manifest()), encoding="utf-8")
        (d / M.STACK_FILE).write_text(json.dumps({"schema": "vlaguard-stack-revisions/99"}),
                                      encoding="utf-8")
        for call in (lambda: M.load_run_manifest(d),
                     lambda: M.add_host_revision(d, "desktop", {})):
            try:
                call()
            except ValueError as e:
                assert "vlaguard-stack-revisions/99" in str(e)
            else:
                raise AssertionError("an unknown stack schema was read")
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_a_failed_write_is_warned_never_raised():
    warned = []
    assert M.write_stack_revisions(Path(tempfile.gettempdir()) / "no" / "such" / "dir",
                                   _manifest(), MAVROS_EV, describe=_no_describe,
                                   warn=warned.append) is None
    assert warned and warned[0].startswith(M.STACK_FILE), warned


def test_the_replay_bundle_carries_the_record_and_a_rerun_moves_it_aside():
    """A reused run tag must not leave the previous flight's record beside a
    new manifest (a run that dies before writing its own), and the replay
    bundle carries the record when the run has one, never requires it."""
    from guardrail.replay import EPISODE_FILES, STALE_PATTERNS, rotate_stale_episode
    assert EPISODE_FILES[M.STACK_FILE] is False and M.STACK_FILE in STALE_PATTERNS
    d = _run(_manifest())
    try:
        M.write_stack_revisions(d, _manifest(), MAVROS_EV, describe=_no_describe)
        moved = rotate_stale_episode(d, stamp="t0")
        assert {M.STACK_FILE, "manifest.json"} <= set(moved), moved
        assert not (d / M.STACK_FILE).exists() and (d / "_previous" / "t0" / M.STACK_FILE).is_file()
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_the_grade_does_not_read_the_record():
    """Evidence, not a gate: is_kpi_grade takes the manifest and metrics it
    always took, so no stored run's grade moves."""
    import inspect
    assert "STACK_FILE" not in inspect.getsource(M.is_kpi_grade)
    assert "stack" not in inspect.signature(M.is_kpi_grade).parameters


# ------------------------------------------------------------ the rails

def test_the_mission_node_writes_the_record_with_its_manifest():
    sys.path.insert(0, str(ROOT / "sitl"))
    import mavros_evidence as E
    import ros2_mission_node as MN
    from guardrail import load_policy
    pol = load_policy(ROOT / "policies" / "demo_policy.yaml")
    out = Path(tempfile.mkdtemp(prefix="mission_stack_"))
    try:
        man = MN.write_run_records(out, {"rail": "ros2_mavros_mission"},
                                   policy_hash=pol.policy_hash, seed=3, sim_speedup=None,
                                   warn=lambda m: None,
                                   evidence=E.initial_evidence(vla_node=False, environ={},
                                                               host=DESKTOP))
        assert list(man) == GRANT_FIELDS
        doc = json.loads((out / M.STACK_FILE).read_text(encoding="utf-8"))
        assert list(doc["hosts"]) == ["desktop"]
        assert doc["hosts"]["desktop"]["code_revision"] == man["code_revision"]
        # MAVROS never connected here: the autopilot is not named, and says so.
        assert doc["autopilot"] is None and doc["simulator"] is None
        assert any("did not name its firmware" in p for p in doc["problems"])
    finally:
        shutil.rmtree(out, ignore_errors=True)


def test_the_hil_harness_adds_the_desktop_once_per_campaign():
    sys.path.insert(0, str(ROOT / "experiments"))
    import sweep_scenarios as S
    calls = []
    saved = (S._DESKTOP_REVISION, M.host_revision)

    def fake(**kw):
        calls.append(kw)
        return {"hostname": "desktop", "code_revision": REV, "dirty": False,
                "git_describe": REV, "device_model": None, "host_arch": "AMD64",
                "source": "test"}
    S._DESKTOP_REVISION = None
    M.host_revision = fake
    dirs = [_run(_manifest(), {"hil_evidence": MAVROS_EV}) for _ in range(2)]
    try:
        for d in dirs:
            M.write_stack_revisions(d, _manifest(), MAVROS_EV, describe=_no_describe)
            doc = S.record_desktop_revision(d)
            assert set(doc["hosts"]) == {"companion", "desktop"} and doc["single_sha"] is True
        # (The node's own write goes through host_revision too, with its host.)
        desk = [c for c in calls if "repo" in c]
        assert len(desk) == 1 and desk[0]["repo"] == S.ROOT, calls
        # A record that cannot be written is printed; the run goes on.
        (dirs[0] / M.STACK_FILE).write_text("{not json", encoding="utf-8")
        assert S.record_desktop_revision(dirs[0]) is None
    finally:
        S._DESKTOP_REVISION, M.host_revision = saved
        for d in dirs:
            shutil.rmtree(d, ignore_errors=True)


def test_the_pas_rail_records_project_airsim_beside_its_manifest():
    for p in (ROOT / "demo", ROOT / "demo" / "pas_ardupilot"):
        if str(p) not in sys.path:
            sys.path.insert(0, str(p))
    import perception_node as pn
    scene = ROOT / "demo" / "pas_config" / "scene_semantic.jsonc"
    out = Path(tempfile.mkdtemp(prefix="pas_stack_"))
    try:
        ev = {"topology_evidence": {**DESKTOP, "ardupilot_version": "ArduCopter V4.5.7 "
                                    "(2a3dc4b7)", "autopilot_kind": "sitl"}}
        doc = pn.write_stack_record(out, _manifest(), ev, scene)
        assert doc["simulator"]["name"] == "Project AirSim"
        assert doc["simulator"]["scene"] == scene.name
        assert list(doc["hosts"]) == ["desktop"] and doc["autopilot"]["version"] == "4.5.7"
        assert (out / M.STACK_FILE).is_file()
        # No scene (the ROS 2 camera): SITL's own build stands for the simulator.
        assert pn.write_stack_record(out, _manifest(), ev, None)["simulator"]["name"] \
            == "ArduPilot SITL"
    finally:
        shutil.rmtree(out, ignore_errors=True)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
