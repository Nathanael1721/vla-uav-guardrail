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

def test_write_stack_revisions_puts_the_record_beside_an_unchanged_manifest():
    """The library call every wired rail makes (the Shield node's own call
    site is pinned by test_the_shield_node_writes_the_record_right_after_its_manifest)."""
    man = _manifest()
    d = _run(man)
    try:
        doc = M.write_stack_revisions(d, man, MAVROS_EV, describe=_no_describe)
        on_disk = json.loads((d / M.STACK_FILE).read_text(encoding="utf-8"))
        assert on_disk == doc and doc["schema"] == M.STACK_SCHEMA
        assert doc["run_manifest_schema"] == 2 and doc["code_revision"] == REV
        assert doc["topology"] == "hil" and doc["required_hosts"] == ["companion", "desktop"]
        assert list(doc["hosts"]) == ["companion"], doc["hosts"]
        comp = doc["hosts"]["companion"]
        assert comp["code_revision"] == REV and comp["hostname"] == "agx"
        assert comp["device_model"].startswith("NVIDIA Jetson")
        assert comp["role"] == "companion" and "device tree model" in comp["role_basis"]
        assert doc["autopilot"]["git_hash"] == "2a3dc4b7"
        assert doc["simulator"]["name"] == "ArduPilot SITL"
        man_disk = json.loads((d / "manifest.json").read_text(encoding="utf-8"))
        assert list(man_disk) == GRANT_FIELDS and man_disk == man
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_a_companion_only_record_never_claims_one_sha_across_the_stack():
    """The Orin's own write on hil (and the copy its signed replay bundle
    carries, or the one left behind when the desktop's add fails or the run
    was launched outside run_hil) saw one host of two: single_sha stays None
    (pending), and problems say the desktop has not reported."""
    for topo in ("hil", "flight"):
        man = {**_manifest(), "topology": topo}
        d = _run(man)
        try:
            doc = M.write_stack_revisions(d, man, MAVROS_EV, describe=_no_describe)
            assert doc["single_sha"] is None, (topo, doc)
            assert doc["single_sha"] is not True and doc["problems"] != []
            assert any(p.startswith("desktop has not reported its revision")
                       for p in doc["problems"]), doc["problems"]
        finally:
            shutil.rmtree(d, ignore_errors=True)
    # What one host CAN rule out it still does: a dirty companion is False.
    man = _manifest(REV + "-dirty")
    d = _run(man)
    try:
        doc = M.write_stack_revisions(d, man, MAVROS_EV, describe=_no_describe)
        assert doc["single_sha"] is False, doc
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_the_real_hil_smoke_run_replayed_reads_pending_not_true():
    """The reviewer's replay, kept: the Orin-side write on the real
    ntFIN2_001 hil_evidence and manifest (lab checkout, read only)."""
    src = ROOT.parents[2] / "demo" / "out" / "hil_smoke" / "ntFIN2_001_nfz-head-on"
    if not (src / "manifest.json").is_file():
        src = ROOT / "demo" / "out" / "hil_smoke" / "ntFIN2_001_nfz-head-on"
    if not (src / "manifest.json").is_file() or not (src / "metrics.json").is_file():
        print("      SKIP: the ntFIN2_001 hil run is not in this checkout")
        return SKIP
    man = json.loads((src / "manifest.json").read_text(encoding="utf-8"))
    ev = json.loads((src / "metrics.json").read_text(encoding="utf-8")).get("hil_evidence")
    d = _run(man)
    try:
        doc = M.write_stack_revisions(d, man, ev, describe=_no_describe)
        assert man["topology"] == "hil" and list(doc["hosts"]) == ["companion"], doc["hosts"]
        assert doc["single_sha"] is not True and doc["problems"] != [], doc
        assert any("desktop has not reported" in p for p in doc["problems"]), doc["problems"]
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_dev_needs_only_the_desktop_and_an_unknown_topology_stays_pending():
    man = {**_manifest(), "topology": "dev"}
    d = _run(man)
    try:
        doc = M.write_stack_revisions(d, man, {**DESKTOP, "ardupilot_version":
                                               MAVROS_EV["ardupilot_version"],
                                               "autopilot_kind": "sitl"},
                                      describe=_no_describe)
        assert doc["required_hosts"] == ["desktop"] and list(doc["hosts"]) == ["desktop"]
        assert doc["single_sha"] is True and doc["problems"] == [], doc["problems"]
    finally:
        shutil.rmtree(d, ignore_errors=True)
    # The legacy label reads as dev.
    assert M.stack_required_roles("canonical-hil") == ("desktop",)
    for topo in ("some-new-rail", None):
        doc = M.stack_document(code_revision=REV, hosts={"desktop": M.host_revision(
            host=DESKTOP, revision=REV, describe=_no_describe)},
            autopilot=M.autopilot_revision(MAVROS_EV), simulator=None, topology=topo)
        assert doc["single_sha"] is None and doc["required_hosts"] is None, doc
        assert any("are not known" in p for p in doc["problems"]), doc["problems"]


def test_a_companion_that_misread_its_role_is_not_overwritten_by_the_desktop():
    """An Orin container without the device-tree mount reads as desktop; the
    harness's own desktop entry goes beside it, never over it, and the
    record stays pending with the clash named."""
    blind_orin = {**ORIN, "device_model": None, "l4t_release": None}
    role, why = M.host_role_basis(blind_orin)
    assert role == "desktop" and "aarch64" in why and "device-tree mount" in why
    man = _manifest()
    d = _run(man)
    try:
        first = M.write_stack_revisions(d, man, {**MAVROS_EV, **blind_orin},
                                        describe=_no_describe)
        assert list(first["hosts"]) == ["desktop"] and first["single_sha"] is None
        desk = M.host_revision(host=DESKTOP, revision=REV, describe=_no_describe)
        doc = M.add_host_revision(d, M.ROLE_DESKTOP, desk)
        assert set(doc["hosts"]) == {"desktop", "desktop#2"}, doc["hosts"]
        assert doc["hosts"]["desktop"]["hostname"] == "agx"
        assert doc["hosts"]["desktop#2"]["hostname"] == "desktop"
        assert doc["single_sha"] is None, doc
        assert any("desktop#2: a second host reported as desktop" in p
                   for p in doc["problems"]), doc["problems"]
        assert any(p.startswith("companion has not reported") for p in doc["problems"])
        # The same machine adding itself again replaces its own entry.
        again = M.add_host_revision(d, M.ROLE_DESKTOP, {**desk, "git_describe": "again"})
        assert set(again["hosts"]) == {"desktop", "desktop#2"}
        assert again["hosts"]["desktop#2"]["git_describe"] == "again"
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
        # hil needs the companion too, and it never reported: pending.
        assert doc["single_sha"] is None and doc["topology"] == "hil"
        assert any(p.startswith("companion has not reported") for p in doc["problems"])
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


# ------------------------------------------------- the reader (2026-11-21)

def test_stack_summary_quotes_single_sha_as_written_with_the_revisions_it_names():
    """The KPI report's reader (tools/kpi_report.py, 2026-11-21): every
    state single_sha can be in, as the run wrote it, with the companion,
    desktop and autopilot revisions the record names. Against 71aabb3 this
    fails: nothing read the record, and guardrail.manifest had no
    stack_summary."""
    man = _manifest()
    cases = (("pending", None, None), ("true", REV, True),
             ("false", "0123456789ab", False))
    for state, desk_rev, single in cases:
        d = _run(man)
        try:
            M.write_stack_revisions(d, man, MAVROS_EV, describe=_no_describe)
            if desk_rev is not None:
                M.add_host_revision(d, M.ROLE_DESKTOP, M.host_revision(
                    host=DESKTOP, revision=desk_rev, describe=_no_describe))
            before = (d / M.STACK_FILE).read_bytes()
            s = M.stack_summary(d)
            assert s["state"] == state and s["single_sha"] is single, (state, s)
            assert s["companion"] == [REV], s
            assert s["desktop"] == ([] if desk_rev is None else [desk_rev]), s
            assert s["autopilot"] == "ArduCopter V4.5.7 (2a3dc4b7)", s
            assert (s["problems"] == []) is (single is True), s["problems"]
            # Read only: the record is not rewritten.
            assert (d / M.STACK_FILE).read_bytes() == before
        finally:
            shutil.rmtree(d, ignore_errors=True)


def test_stack_summary_says_missing_and_unreadable_and_never_true_for_them():
    """A run flown before the record (2026-11-17) is "missing"; a record
    that is not JSON, of another schema, or without a verdict is
    "unreadable" - never counted as one SHA across the stack."""
    d = _run(_manifest())
    try:
        s = M.stack_summary(d)
        assert s["state"] == "missing" and s["single_sha"] is None, s
        assert M.STACK_FILE in s["why"]
        for body, said in (("{not json", "not readable JSON"),
                           (json.dumps({"schema": "vlaguard-stack-revisions/99",
                                        "single_sha": True}), "stack-revisions/99"),
                           (json.dumps({"schema": M.STACK_SCHEMA, "hosts": {}}),
                            "no single_sha"),
                           (json.dumps({"schema": M.STACK_SCHEMA, "single_sha": "yes"}),
                            "no single_sha")):
            (d / M.STACK_FILE).write_text(body, encoding="utf-8")
            s = M.stack_summary(d)
            assert s["state"] == "unreadable" and s["single_sha"] is None, (body, s)
            assert said in s["why"], (body, s["why"])
        assert set(M.STACK_STATES) == {"true", "false", "pending", "missing", "unreadable"}
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_stack_summary_keeps_a_second_claimant_and_the_components():
    """A duplicate role ("desktop#2") and a host's components (the WSL
    helper's checkout) are revisions the record names: kept, not dropped."""
    man = _manifest()
    d = _run(man)
    try:
        M.write_stack_revisions(d, man, MAVROS_EV, describe=_no_describe)
        desk = M.host_revision(host=DESKTOP, revision=REV, describe=_no_describe)
        desk["components"] = {"wsl_helper": {"code_revision": "0123456789ab"}}
        M.add_host_revision(d, M.ROLE_DESKTOP, desk)
        M.add_host_revision(d, M.ROLE_DESKTOP, M.host_revision(
            host=dict(DESKTOP, hostname="other-pc"), revision="fedcba987654",
            describe=_no_describe))
        s = M.stack_summary(d)
        assert s["state"] == "false", s
        assert s["desktop"] == sorted([REV, "fedcba987654"]), s
        assert s["components"] == {"desktop/wsl_helper": "0123456789ab"}, s
    finally:
        shutil.rmtree(d, ignore_errors=True)


# ------------------------------------------------------------ the rails

def test_the_shield_node_writes_the_record_right_after_its_manifest():
    """The KPI rail's call site sits inside `if HAVE_ROS` and no desktop test
    runs it, so its shape is pinned from the source: in the method that
    writes manifest.json, the statement right after that write is
    write_stack_revisions(self.out, manifest, self.hil_evidence, warn=...),
    and it comes before kpi.json is computed."""
    import ast
    src = (ROOT / "sitl" / "ros2_shield_node.py").read_text(encoding="utf-8")
    tree = ast.parse(src)

    def writes_manifest(stmt) -> bool:
        return any(isinstance(n, ast.Constant) and n.value == "manifest.json"
                   for n in ast.walk(stmt)) and "write_text" in ast.unparse(stmt)
    found = []
    for fn in ast.walk(tree):
        if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        for body in [n.body for n in ast.walk(fn) if hasattr(n, "body")
                     and isinstance(n.body, list)]:
            for i, stmt in enumerate(body):
                if not writes_manifest(stmt):
                    continue
                nxt = body[i + 1] if i + 1 < len(body) else None
                found.append((fn.name, nxt, body[i + 2:]))
    assert found, "no manifest.json write found in the Shield node"
    for name, nxt, rest in found:
        assert isinstance(nxt, ast.Expr) and isinstance(nxt.value, ast.Call), (
            name, ast.unparse(nxt) if nxt else None)
        call = nxt.value
        assert ast.unparse(call.func) == "write_stack_revisions", ast.unparse(call)
        assert [ast.unparse(a) for a in call.args] == ["self.out", "manifest",
                                                       "self.hil_evidence"], ast.unparse(call)
        assert [k.arg for k in call.keywords] == ["warn"], ast.unparse(call)
        later = "\n".join(ast.unparse(s) for s in rest)
        assert "compute_from_dir" in later, name
    # And the name it calls is the library's, through mavros_evidence.
    sys.path.insert(0, str(ROOT / "sitl"))
    import mavros_evidence as E
    assert E.write_stack_revisions is M.write_stack_revisions


def test_the_wsl_helper_checkout_is_recorded_when_it_is_another_one():
    sys.path.insert(0, str(ROOT / "experiments"))
    import sweep_scenarios as S

    class _Done:
        def __init__(self, rc, out):
            self.returncode, self.stdout = rc, out
    calls = []

    def run_as(head, status):
        def run(cmd, **kw):
            calls.append(cmd)
            return _Done(0, head + "\n") if "rev-parse" in cmd else _Done(0, status)
        return run
    # This checkout as WSL sees it: nothing more to read, no wsl.exe call.
    same = S.HilConfig(wsl_repo=S._wsl_path(S.ROOT))
    assert S.wsl_helper_revision(same, run=run_as(REV, "")) is None and not calls
    assert S.wsl_helper_revision(S.HilConfig(), run=run_as(REV, "")) is None and not calls
    saved = (dict(S._WSL_HELPER_REVISIONS), S._DESKTOP_REVISION)
    S._DESKTOP_REVISION = M.host_revision(host=DESKTOP, revision=REV, describe=_no_describe)
    try:
        for repo, head, status, single in (("/home/u/other", REV + "0000", "", True),
                                           ("/home/u/stale", "0123456789ab0000", "", False),
                                           ("/home/u/dirty", REV + "0000", " M x.py\n", False)):
            S._WSL_HELPER_REVISIONS.clear()
            cfg = S.HilConfig(wsl_repo=repo)
            man = _manifest()
            d = _run(man)
            try:
                M.write_stack_revisions(d, man, MAVROS_EV, describe=_no_describe)
                doc = S.record_desktop_revision(d, cfg, run=run_as(head, status))
                helper = doc["hosts"]["desktop"]["components"]["wsl_helper"]
                assert helper["repo"] == repo and helper["wsl_distro"] == "Ubuntu"
                assert calls[-1][:6] == ["wsl.exe", "-d", "Ubuntu", "--", "git", "-C"]
                assert doc["single_sha"] is single, (repo, doc)
                if not single:
                    assert any(p.startswith("desktop/wsl_helper") for p in doc["problems"])
            finally:
                shutil.rmtree(d, ignore_errors=True)
    finally:
        S._WSL_HELPER_REVISIONS.clear()
        S._WSL_HELPER_REVISIONS.update(saved[0])
        S._DESKTOP_REVISION = saved[1]


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
