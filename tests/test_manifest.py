"""The WP4 determinism manifest and the four locked acceptance KPIs.

Run either way:
    pytest tests/test_manifest.py -v
    python tests/test_manifest.py

The grant allows contractual KPI numbers only from a run that carries a six-field
determinism manifest at `sim_speedup=1.0`. Our flights carried none of it, so on
the grant's own terms nothing measured here was reportable.

The most important test in this file is the one that makes the hard KPI FAIL. A
P0-escape counter that only ever sees compliant logs proves nothing: it would
report a perfect score for a broken Shield. So a log where a P0 violation is
detected and then flown anyway must come out non-zero.
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import kpi as K                                        # noqa: E402
from guardrail.manifest import (MAX_HEADING_ERR_DEG, MIN_DET_HZ,      # noqa: E402
                                TOPOLOGY_CANONICAL_HIL, TOPOLOGY_DEV,
                                TOPOLOGY_HIL, TOPOLOGY_PROJECTAIRSIM,
                                build_manifest, is_kpi_grade,
                                normalize_topology, sim_speedup_from_scene)

SCENE = ROOT / "demo" / "pas_config" / "scene_semantic.jsonc"
GOOD_METRICS = {"det_hz": 4.5, "start_heading_err_deg": 1.5}


def _man(**kw):
    base = dict(policy_hash="sha256:deadbeefdeadbeef",
                model_id="google/owlvit-base-patch32", seed=42,
                scene_path=str(SCENE))
    base.update(kw)
    return build_manifest(**base)


# ------------------------------------------------------------------ manifest

def test_the_manifest_has_exactly_the_six_grant_fields():
    m = _man()
    assert set(m) == {"code_revision", "vla_model_hash", "policy_hash",
                      "random_seed", "sim_speedup", "topology"}, sorted(m)


def test_the_same_inputs_give_a_byte_identical_manifest():
    a, b = _man(), _man()
    assert json.dumps(a, sort_keys=True) == json.dumps(b, sort_keys=True)


def test_changing_any_field_changes_the_manifest():
    base = json.dumps(_man(), sort_keys=True)
    assert json.dumps(_man(seed=43), sort_keys=True) != base
    assert json.dumps(_man(policy_hash="sha256:0"), sort_keys=True) != base
    assert json.dumps(_man(model_id="other/model"), sort_keys=True) != base


def test_sim_speedup_is_derived_from_the_scene_not_asserted():
    """A hardcoded 1.0 would defeat the requirement it exists to enforce."""
    assert sim_speedup_from_scene(SCENE) == 1.0


def test_a_fast_clock_is_detected_and_disqualifies_the_run(tmp=Path("_t_fastclock.jsonc")):
    """`sim_speedup=1.0` is mandatory for a KPI-bearing run, so a scene running
    faster than real time has to be caught from the config."""
    try:
        tmp.write_text(json.dumps({
            "id": "fast", "clock": {"type": "steppable", "step-ns": 3000000,
                                    "real-time-update-rate": 12000000}}),
            encoding="utf-8")
        assert sim_speedup_from_scene(tmp) == 4.0
        ok, why = is_kpi_grade(_man(scene_path=str(tmp)), GOOD_METRICS)
        assert not ok and any("sim_speedup" in r for r in why), why
    finally:
        tmp.unlink(missing_ok=True)


def test_an_unreadable_scene_is_unresolved_not_assumed_real_time():
    m = _man(scene_path="does/not/exist.jsonc")
    assert m["sim_speedup"] == "unresolved"
    ok, why = is_kpi_grade(m, GOOD_METRICS)
    assert not ok and any("sim_speedup" in r for r in why)


def test_our_rail_may_not_claim_the_canonical_hil_topology():
    """The field exists so the difference cannot be blurred. Our flights are
    Project AirSim; the grant's KPI topology is ArduPilot SITL + MAVROS.

    Tested WITH good evidence, which is the whole point. Without evidence this
    was just a second copy of
    `test_canonical_hil_cannot_be_claimed_without_evidence`, and its failure
    message - "a Project AirSim run was allowed to claim canonical-hil" - named
    a property nothing actually enforced. Supplying `hil_evidence` used to be
    enough to get our own Project AirSim scene stamped `canonical-hil`, the one
    label the grant reads as KPI-grade.

    The scene file is the tell: the canonical rail is ArduPilot SITL and has no
    simulator scene.
    """
    try:
        _man(topology=TOPOLOGY_CANONICAL_HIL, hil_evidence=GOOD_HIL)
    except ValueError as e:
        assert "scene" in str(e).lower(), e
    else:
        raise AssertionError("a Project AirSim run was allowed to claim canonical-hil")


# --------------------------------------------------------------- kpi grading

def test_the_project_airsim_rail_is_never_kpi_grade():
    """Honest ceiling on everything this repo can currently claim."""
    ok, why = is_kpi_grade(_man(), GOOD_METRICS)
    assert not ok
    assert any(TOPOLOGY_PROJECTAIRSIM in r for r in why), why


def test_a_stalled_detector_disqualifies_the_run():
    """The real failure: 29 inferences at 0.52 Hz reported det_hit_rate 1.000 on a
    flight that tracked for 13.6% of ticks."""
    ok, why = is_kpi_grade(_man(), {"det_hz": 0.52, "start_heading_err_deg": 1.0})
    assert not ok and any("Hz" in r for r in why), why


def test_a_missed_start_heading_disqualifies_the_run():
    """The real failure: the heading was never commanded, and swung the traffic hit
    rate 0.740 -> 0.331 on identical configuration."""
    ok, why = is_kpi_grade(_man(), {"det_hz": 4.5, "start_heading_err_deg": 52.1})
    assert not ok and any("heading" in r for r in why), why
    assert MAX_HEADING_ERR_DEG < 52.1 and MIN_DET_HZ > 0.52


def test_an_unresolved_field_disqualifies_the_run():
    ok, why = is_kpi_grade(_man(policy_hash=""), GOOD_METRICS)
    assert not ok and any("policy_hash" in r for r in why), why


# ------------------------------------------------------------- the hard KPI

def _tick(rule="nfz-route", repaired=False, braked=False, moved=False):
    raw = {"vx": 1.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0}
    em = dict(raw)
    if moved:
        em["vy"] = 0.7
    return {"violations": [{"rule_id": rule, "category": "geofence"}],
            "repairs": ([{"operator": "GeofenceProject"}] if repaired else []),
            "braked": braked, "raw": raw, "emitted": em}


PRIOS = {"nfz-route": "P0", "kin-caps": "P1"}


def test_a_p0_flown_anyway_IS_counted_as_an_escape():
    """THE TEST THAT MATTERS. A counter that only sees compliant logs would report
    a perfect score for a broken Shield."""
    k = K.compute([_tick()] * 10, PRIOS)
    assert k["p0_escapes"] == 10
    assert k["p0_violation_escape_rate"] == 1.0
    assert k["outcome"] == "fail" and k["mission_success"] is False


def test_a_repair_that_did_not_fix_it_IS_an_escape():
    """The case the old accounting could not see, and the reason it changed.

    A repair ran, so `repairs` is non-empty and the action changed - the old
    inference (`emitted_differs or braked or repairs`) therefore scored this as
    the Shield working. But the re-check on the flown action still names a P0
    rule, which is the definition of an escape. Because every Shield branch
    that raises a violation also appends a Repair, this shape is the ONLY way
    a real escape can occur, and it was exactly the one being missed.
    """
    row = _tick(repaired=True, moved=True)
    row["emitted_violations"] = [{"rule_id": "nfz-route", "category": "geofence"}]
    k = K.compute([row] * 10, PRIOS)
    assert k["p0_escapes"] == 10, k
    assert k["p0_violation_escape_rate"] == 1.0
    assert k["outcome"] == "fail" and k["mission_success"] is False
    assert k["failsafe_trigger_correctness"] == 0.0


def test_a_measured_clean_repair_is_not_an_escape_and_is_marked_measured():
    """The good case, recorded rather than assumed."""
    row = _tick(repaired=True, moved=True)
    row["emitted_violations"] = []
    k = K.compute([row] * 10, PRIOS)
    assert k["p0_escapes"] == 0
    assert k["p0_ticks_not_measurable"] == 0, "these rows carry the re-check"


def test_a_log_without_the_recheck_is_flagged_as_not_measurable():
    """Old artefacts still score, but must not look like measured zeros."""
    k = K.compute([_tick(repaired=True, moved=True)] * 10, PRIOS)
    assert k["p0_escapes"] == 0
    assert k["p0_ticks_not_measurable"] == 10
    assert k["mission_success"] is False, (
        "a run whose escape rate was never measured must not claim success")


def test_a_p0_that_was_repaired_is_not_an_escape():
    """Detected-and-repaired is the Shield working, and must not count against it."""
    k = K.compute([_tick(repaired=True, moved=True)] * 10, PRIOS)
    assert k["p0_escapes"] == 0 and k["p0_violation_escape_rate"] == 0.0
    assert k["repair_count"] == 10
    assert k["failsafe_trigger_correctness"] == 1.0


def test_braking_also_counts_as_acting_on_a_p0():
    k = K.compute([_tick(braked=True)] * 5, PRIOS)
    assert k["p0_escapes"] == 0


def test_a_p1_violation_is_not_a_p0_escape():
    k = K.compute([_tick(rule="kin-caps")] * 8, PRIOS)
    assert k["p0_escapes"] == 0
    assert k["violations_by_risk_level"] == {"P1": 8}


def test_an_unknown_rule_is_treated_as_p0():
    """Conservative direction on purpose: a rule the policy does not name must not
    silently downgrade the hard KPI."""
    k = K.compute([_tick(rule="who-knows")] * 3, PRIOS)
    assert k["p0_escapes"] == 3


def test_a_clean_flight_scores_zero_escapes_and_succeeds():
    rows = [{"violations": [], "repairs": [], "raw": {}, "emitted": {}}] * 50
    k = K.compute(rows, PRIOS, {"nfz_s": 0.0, "alt_violation_s": 0.0,
                                "frac_within_30m": 1.0})
    assert k["p0_violation_escape_rate"] == 0.0
    assert k["outcome"] == "success" and k["mission_success"] is True
    assert k["failsafe_trigger_correctness"] is None      # nothing to trigger on


def test_time_inside_a_fence_fails_the_mission_even_with_no_escape():
    rows = [{"violations": [], "repairs": [], "raw": {}, "emitted": {}}] * 20
    k = K.compute(rows, PRIOS, {"nfz_s": 3.2, "alt_violation_s": 0.0})
    assert k["outcome"] == "fail" and k["mission_success"] is False


def test_priorities_come_from_the_policy_constraints_list():
    """An earlier version guessed per-type attribute names, silently returned {},
    and defaulted every rule to P0."""
    from guardrail import load_policy
    prios = K.rule_priorities(load_policy(ROOT / "policies" / "follow_car_nfz.yaml"))
    assert prios, "no priorities resolved from the policy"
    assert set(prios.values()) <= {"P0", "P1", "P2"}
    assert "P0" in prios.values()


# -------------------------------------------------- code_revision provenance

def test_code_revision_names_a_repository_that_holds_the_shield():
    """The field must name a commit a reader could check out to get this code.

    It used to name `kuanting-vla-uav-guardrail` - a repository with no
    guardrail/shield.py in it - because that path was searched first and merely
    resolving was accepted as good enough.
    """
    from guardrail.manifest import CODE_SENTINEL, _tracks_our_code, code_revision
    rev = code_revision()
    if rev == "unversioned":
        return                      # no git available; nothing to assert about
    assert _tracks_our_code(ROOT), \
        f"{ROOT} reports a revision but does not track {CODE_SENTINEL}"


def test_a_repository_without_our_code_is_never_accepted():
    """Even asked for by name. Order alone would not have caught the original
    bug: the failure was accepting a repo without checking it holds the code."""
    from guardrail.manifest import _tracks_our_code, code_revision
    fork = ROOT / "kuanting-vla-uav-guardrail"
    if not (fork / ".git").exists():
        return
    assert not _tracks_our_code(fork), \
        "the fork is expected not to contain guardrail/shield.py"
    asked = code_revision(fork)
    ours = code_revision()
    assert asked == ours, \
        f"asking for the fork returned {asked!r}; it must fall through to {ours!r}"


def test_a_dirty_tree_is_not_kpi_grade():
    """A dirty revision does not describe the code that flew: checking out that
    commit gives you something else."""
    ok, why = is_kpi_grade({**_man(), "code_revision": "abc123abc123-dirty"},
                           GOOD_METRICS)
    assert not ok and any("uncommitted" in r for r in why), why


def test_an_unknown_cleanliness_is_not_kpi_grade():
    ok, why = is_kpi_grade({**_man(), "code_revision": "abc123abc123-unknown"},
                           GOOD_METRICS)
    assert not ok and any("clean" in r for r in why), why


def test_a_clean_revision_raises_no_code_revision_objection():
    """Topology will still object - this rail is not canonical HIL - but the
    revision itself must draw no complaint."""
    ok, why = is_kpi_grade({**_man(), "code_revision": "abc123abc123"},
                           GOOD_METRICS)
    assert not any("code_revision" in r for r in why), why


# ------------------------------------------- canonical HIL needs evidence

GOOD_HIL = {"ros_distro": "jazzy", "mavros_node": "/mavros", "fcu_connected": True,
            "ardupilot_version": "ArduCopter V4.5.7 (2a3dc4b7)"}


def test_canonical_hil_cannot_be_claimed_without_evidence():
    """Claiming the canonical topology is claiming KPI-grade eligibility, so it
    may not rest on the caller's word."""
    from guardrail.manifest import TOPOLOGY_CANONICAL_HIL
    for bad in (None, {}, {"ros_distro": "jazzy"}):
        try:
            _man(topology=TOPOLOGY_CANONICAL_HIL, hil_evidence=bad)
        except ValueError:
            continue
        raise AssertionError(f"evidence {bad!r} was accepted")


def test_a_disconnected_flight_controller_is_not_canonical_hil():
    """MAVROS comes up happily with nothing on the other end and publishes
    connected: false forever, so a whole mission can run into the void."""
    from guardrail.manifest import TOPOLOGY_CANONICAL_HIL, check_hil_evidence
    ev = dict(GOOD_HIL, fcu_connected=False)
    assert any("fcu_connected" in m for m in check_hil_evidence(ev))
    try:
        _man(topology=TOPOLOGY_CANONICAL_HIL, hil_evidence=ev)
    except ValueError:
        return
    raise AssertionError("a disconnected FCU was accepted as canonical HIL")


def _dev_run():
    # A REAL MAVROS run: ArduPilot SITL, so no Project AirSim scene file and
    # the speedup read from the flight controller instead of from a scene.
    # The pilot is the source-pinned StubVLA, as on that rail: a HuggingFace
    # id resolves only where its cache exists, which made these topology
    # tests fail in WSL (no OWL-ViT cache) for a reason they do not test.
    return build_manifest(policy_hash="sha256:deadbeefdeadbeef",
                          model_id="guardrail.vla_stub.StubVLA", seed=42,
                          scene_path=None, sim_speedup=1.0,
                          topology=TOPOLOGY_DEV, hil_evidence=GOOD_HIL)


def test_a_dev_run_with_evidence_is_accepted_but_not_kpi_grade_without_a_waiver():
    """ARCH-13. This test used to assert the opposite - that our desktop
    SITL + MAVROS 2 rail, then labelled `canonical-hil`, IS KPI-grade. The
    grant calls that configuration `dev` and says it is not used for reported
    KPI numbers; those come from `hil` (Jetson Orin). Until the PI waives that
    in writing, the gate says so instead of quietly passing."""
    m = _dev_run()
    assert m["topology"] == "dev"
    ok, why = is_kpi_grade({**m, "code_revision": "abc123abc123"}, GOOD_METRICS,
                           dev_waiver="")
    assert not ok
    assert len(why) == 1 and "'hil'" in why[0] and "waiver" in why[0], why


def test_with_a_recorded_waiver_the_dev_gate_can_actually_be_met():
    """And when the waiver exists it must pass, or the gate can never be met
    and the field is decorative."""
    ok, why = is_kpi_grade({**_dev_run(), "code_revision": "abc123abc123"},
                           GOOD_METRICS, dev_waiver="PI email 2026-10-14 (test)")
    assert ok, why


def test_the_legacy_label_is_written_as_dev_and_read_as_dev():
    """Five stored runs say `canonical-hil`. They are read under today's name,
    never rewritten, and no code path emits the old label again."""
    assert TOPOLOGY_CANONICAL_HIL == TOPOLOGY_DEV == "dev"
    assert normalize_topology("canonical-hil") == "dev"
    m = build_manifest(policy_hash="sha256:deadbeefdeadbeef",
                       model_id="guardrail.vla_stub.StubVLA", seed=0,
                       scene_path=None, sim_speedup=1.0,
                       topology="canonical-hil", hil_evidence=GOOD_HIL)
    assert m["topology"] == "dev", m
    stored = {**m, "topology": "canonical-hil", "code_revision": "abc123abc123"}
    ok, why = is_kpi_grade(stored, GOOD_METRICS, dev_waiver="")
    assert not ok and "read as 'dev'" in why[0], why


def test_a_desktop_rail_may_not_claim_hil_or_flight():
    """Both need a Jetson Orin. With only the desktop's MAVROS evidence (no
    device tree naming an Orin, x86_64) both labels are refused, naming the
    Orin."""
    for topo in (TOPOLOGY_HIL, "flight"):
        try:
            build_manifest(policy_hash="sha256:deadbeefdeadbeef",
                           model_id="guardrail.vla_stub.StubVLA", seed=0,
                           scene_path=None, sim_speedup=1.0, topology=topo,
                           hil_evidence={**GOOD_HIL, **DESKTOP_HOST})
        except ValueError as e:
            assert "Orin" in str(e), e
            continue
        raise AssertionError(f"{topo!r} was claimed by a desktop rail")


# An Orin as the node would describe it: device tree, L4T, JetPack, and
# MAVROS talking to SITL on a desktop across a wired link.
DESKTOP_HOST = {"host_arch": "x86_64", "device_model": None,
                "l4t_release": None, "jetpack": None}
ORIN_HOST = {"host_arch": "aarch64",
             "device_model": "NVIDIA Jetson AGX Orin Developer Kit",
             "l4t_release": "R36.3.0", "jetpack": "6.0+b106"}
REMOTE_LINK = {"fcu_url": "udp://:14555@192.168.10.2:14550",
               "peer": "192.168.10.2", "peer_is_loopback": False,
               "iface": "eth0", "link_kind": "ethernet"}
ORIN_HIL = {**GOOD_HIL, **ORIN_HOST, "autopilot_kind": "sitl",
            "network_link": REMOTE_LINK}


def _hil_run(ev=None):
    return build_manifest(policy_hash="sha256:deadbeefdeadbeef",
                          model_id="guardrail.vla_stub.StubVLA", seed=0,
                          scene_path=None, sim_speedup=1.0, topology="hil",
                          hil_evidence=ORIN_HIL if ev is None else ev)


def test_an_orin_with_a_remote_sitl_link_may_claim_hil():
    """The PI's 2026-10-06 decision: the Orin is the hil machine. With its own
    evidence the label is accepted, and the gate then has no topology
    objection. On the code before this change this raised: hil was refused
    outright, so the Orin could never produce a KPI-grade run."""
    m = _hil_run()
    assert m["topology"] == "hil", m
    ok, why = is_kpi_grade({**m, "code_revision": "abc123abc123"},
                           {**GOOD_METRICS, "hil_evidence": ORIN_HIL},
                           dev_waiver="")
    assert ok, why


def test_hil_on_an_orin_with_a_loopback_link_is_refused():
    """MAVROS and SITL on the same machine is dev, wherever that machine is:
    the hil link is desktop -> Orin across the network."""
    local = {**REMOTE_LINK, "fcu_url": "tcp://127.0.0.1:5760",
             "peer": "127.0.0.1", "peer_is_loopback": True}
    for ev in ({**ORIN_HIL, "network_link": local},
               {k: v for k, v in ORIN_HIL.items() if k != "network_link"}):
        try:
            _hil_run(ev)
        except ValueError as e:
            assert "remote desktop" in str(e), e
            continue
        raise AssertionError("an Orin with a local autopilot link claimed hil")


def test_hil_needs_a_sitl_autopilot_and_flight_a_hardware_one():
    for ev, topo in (({**ORIN_HIL, "autopilot_kind": "hardware"}, "hil"),
                     ({**ORIN_HIL, "autopilot_kind": "sitl"}, "flight"),
                     ({**ORIN_HIL, "autopilot_kind": None}, "flight")):
        try:
            build_manifest(policy_hash="sha256:deadbeefdeadbeef",
                           model_id="guardrail.vla_stub.StubVLA", seed=0,
                           scene_path=None, sim_speedup=None, topology=topo,
                           hil_evidence=ev)
        except ValueError as e:
            assert "autopilot_kind" in str(e), e
            continue
        raise AssertionError(f"{topo} accepted autopilot_kind "
                             f"{ev['autopilot_kind']!r}")


def test_a_flight_run_is_accepted_on_evidence_and_never_kpi_grade():
    """The same node on the real drone: same code, label from evidence. The
    grant makes flight qualitative validation, not a KPI topology."""
    ev = {**ORIN_HIL, "autopilot_kind": "hardware",
          "network_link": {"fcu_url": "serial:///dev/ttyTHS1:921600",
                           "peer": None, "peer_is_loopback": True}}
    m = build_manifest(policy_hash="sha256:deadbeefdeadbeef",
                       model_id="guardrail.vla_stub.StubVLA", seed=0,
                       scene_path=None, sim_speedup=None, topology="flight",
                       hil_evidence=ev)
    assert m["topology"] == "flight"
    ok, why = is_kpi_grade({**m, "code_revision": "abc123abc123"},
                           {**GOOD_METRICS, "hil_evidence": ev}, dev_waiver="")
    assert not ok and any("'flight'" in r for r in why), why


def test_a_hand_edited_hil_manifest_is_not_kpi_grade():
    """The label alone is a string. This test used to assert the opposite
    (`a_hil_manifest_draws_no_topology_objection`, passing a desktop dev
    manifest relabelled "hil" with no evidence) - the gate then quoted a
    hand-typed label. Now the evidence beside it is re-checked."""
    m = {**_dev_run(), "topology": "hil", "code_revision": "abc123abc123"}
    ok, why = is_kpi_grade(m, GOOD_METRICS, dev_waiver="")
    assert not ok and any("hil_evidence" in r for r in why), why
    ok, why = is_kpi_grade(m, {**GOOD_METRICS,
                               "hil_evidence": {**GOOD_HIL, **DESKTOP_HOST}},
                           dev_waiver="")
    assert not ok and any("Orin" in r for r in why), why


def test_detect_topology_follows_the_evidence_not_a_flag():
    from guardrail.manifest import detect_topology
    assert detect_topology({**GOOD_HIL, **DESKTOP_HOST}) == "dev"
    assert detect_topology(ORIN_HIL) == "hil"
    assert detect_topology({**ORIN_HIL, "autopilot_kind": "hardware"}) == "flight"
    # An Orin whose MAVROS takes MAVLink from "whoever sends first", with
    # nothing scanned: nothing shows where SITL runs, so dev, not hil.
    assert detect_topology({**ORIN_HIL, "network_link": RUNBOOK_LINK}) == "dev"
    # The same URL with the Orin's processes scanned and no SITL among them:
    # the runbook's hil (MAVROS on the Orin behind mavlink-router).
    assert detect_topology({**ORIN_HIL, "network_link": RUNBOOK_LINK,
                            "local_processes": ORIN_PROCS}) == "hil"
    # SITL found running on the Orin itself: dev, whatever the URL says.
    assert detect_topology({**ORIN_HIL, "local_processes": {
        **ORIN_PROCS, "local_sitl": ["arducopter"]}}) == "dev"


# What MAVROS's fcu_url is on the Orin in docs/RUNBOOK-orin-hil.md and
# deploy/topologies/hil.env: an empty remote, so the router's packets are
# answered wherever they came from.
RUNBOOK_LINK = {"fcu_url": "udp://:14555@", "peer": None,
                "peer_is_loopback": True, "iface": None, "link_kind": None}
# /proc on the Orin in that layout: MAVROS beside the Shield, no SITL.
ORIN_PROCS = {"scanned": "/proc", "local_sitl": [], "local_mavros": True,
              "local_router": []}


def test_the_grants_hil_layout_and_the_runbooks_layout_are_both_hil():
    """Review 2026-10-07. The grant's table: "Desktop runs sims + MAVROS +
    GCS; Jetson Orin runs VLA + Shield over network". The runbook: MAVROS on
    the Orin with fcu_url udp://:14555@. The old check demanded a
    non-loopback fcu_url remote, so it refused BOTH and passed only the PI
    reference's layout: no Orin run made by the runbook could ever be hil."""
    from guardrail.manifest import check_topology_evidence, mavros_location
    grant = {**ORIN_HIL, "network_link": {
                 "fcu_url": "udp://:14555@", "peer": None,
                 "peer_is_loopback": True},
             "local_processes": {**ORIN_PROCS, "local_mavros": False}}
    runbook = {**ORIN_HIL, "network_link": RUNBOOK_LINK,
               "local_processes": ORIN_PROCS}
    for name, ev, where in (("grant", grant, "remote"),
                            ("runbook", runbook, "shield_host")):
        assert check_topology_evidence("hil", ev) == [], (
            name, check_topology_evidence("hil", ev))
        assert mavros_location(ev) == where, name
        assert _hil_run(ev)["topology"] == "hil", name
    # MAVROS on the desktop: its loopback fcu_url is the desktop's business.
    tcp = {**grant, "network_link": {"fcu_url": "tcp://127.0.0.1:5760",
                                     "peer": "127.0.0.1",
                                     "peer_is_loopback": True}}
    assert check_topology_evidence("hil", tcp) == []
    # MAVROS on the Orin naming a loopback autopilot: local link, dev.
    local = {**runbook, "network_link": tcp["network_link"]}
    assert any("loopback" in m for m in check_topology_evidence("hil", local))


def test_sitl_on_the_shields_host_is_never_hil():
    from guardrail.manifest import check_topology_evidence
    for procs in ({**ORIN_PROCS, "local_sitl": ["arducopter"]},
                  {**ORIN_PROCS, "local_mavros": False,
                   "local_sitl": ["sim_vehicle.py"]}):
        miss = check_topology_evidence("hil", {**ORIN_HIL,
                                               "local_processes": procs})
        assert any("SITL runs on the Shield's host" in m for m in miss), miss


def test_a_vla_node_off_the_orin_is_not_hil():
    """The grant puts the VLA on the Orin with the Shield. A separate VLA
    node reports its host on /vla/identity; the record of a node that sent
    none is None, which is refused. Absent means in-process (unchanged)."""
    from guardrail.manifest import check_topology_evidence
    ok = {**ORIN_HIL, "vla_host": dict(ORIN_HOST)}
    assert check_topology_evidence("hil", ok) == []
    for vh, words in ((None, "did not report"),
                      (dict(DESKTOP_HOST), "not on a Jetson Orin")):
        miss = check_topology_evidence("hil", {**ORIN_HIL, "vla_host": vh})
        assert any(words in m for m in miss), (vh, miss)
        miss = check_topology_evidence("flight", {
            **ORIN_HIL, "autopilot_kind": "hardware", "vla_host": vh})
        assert any(words in m for m in miss), (vh, miss)


def test_the_orin_rule_is_public_and_is_the_one_the_manifest_applies():
    """tools/profile_shield_tick.py labels a host as an Orin with
    guardrail.manifest.orin_missing when that name exists (and falls back to
    the private one), so the tick profile and the manifest share one rule.
    Requested by the orin-hil-portability unit, 2026-10-07. The private name
    stays for older importers (tests/test_profile_shield_tick.py)."""
    from guardrail import manifest as M
    assert callable(getattr(M, "orin_missing", None)), "no public orin_missing"
    assert M._orin_missing is M.orin_missing
    assert M.orin_missing(dict(ORIN_HOST)) == []
    miss = M.orin_missing(dict(DESKTOP_HOST))
    assert len(miss) == 3 and any("aarch64" in m for m in miss), miss
    # hil on a desktop host is refused with exactly these reasons among others.
    got = M.check_topology_evidence("hil", {**ORIN_HIL, **DESKTOP_HOST})
    assert all(m in got for m in miss), got


def test_processes_are_named_by_their_executable_not_their_arguments():
    """A rosbag recorder's ARGUMENTS name /mavros topics; it is not MAVROS."""
    from guardrail.manifest import classify_process
    assert classify_process(["/home/u/ardupilot/build/sitl/bin/arducopter",
                             "--model", "quad"]) == "sitl"
    assert classify_process(["python3", "/ap/Tools/autotest/sim_vehicle.py",
                             "-v", "ArduCopter"]) == "sitl"
    assert classify_process(["/opt/ros/jazzy/lib/mavros/mavros_node",
                             "--ros-args", "-p", "fcu_url:=udp://:14555@"]) == "mavros"
    assert classify_process(["/usr/bin/python3", "/opt/ros/jazzy/bin/ros2",
                             "run", "mavros", "mavros_node"]) == "mavros"
    assert classify_process(["mavlink-routerd", "-c", "x.conf"]) == "router"
    assert classify_process(["/usr/bin/python3", "/opt/ros/jazzy/bin/ros2",
                             "bag", "record", "/mavros/state",
                             "/mavros/local_position/pose"]) is None
    assert classify_process(["python", "sitl/ros2_shield_node.py",
                             "--shield", "on"]) is None


def test_the_process_scan_reads_proc_and_says_none_without_it():
    import tempfile
    from guardrail.manifest import scan_local_processes
    d = Path(tempfile.mkdtemp(prefix="proc_"))
    try:
        for pid, argv in ((101, ["/x/arducopter", "--model", "quad"]),
                          (102, ["/opt/ros/jazzy/lib/mavros/mavros_node"]),
                          (103, ["ros2", "bag", "record", "/mavros/state"])):
            (d / str(pid)).mkdir()
            (d / str(pid) / "cmdline").write_bytes(
                "\x00".join(argv).encode() + b"\x00")
        (d / "self").mkdir()
        got = scan_local_processes(d)
        assert got["local_sitl"] == ["arducopter"], got
        assert got["local_mavros"] is True and got["local_router"] == [], got
        assert scan_local_processes(d / "missing") is None, \
            "no /proc is 'not scanned', never 'nothing found'"
    finally:
        import shutil as _sh
        _sh.rmtree(d, ignore_errors=True)


def test_dev_evidence_without_the_autopilot_version_is_refused():
    """ARCH-31: a run must name the firmware that flew. Before 2026-10-06 the
    three-field evidence was enough and no run recorded a version."""
    from guardrail.manifest import check_hil_evidence
    ev = {k: v for k, v in GOOD_HIL.items() if k != "ardupilot_version"}
    assert any("ardupilot_version" in m for m in check_hil_evidence(ev))
    try:
        build_manifest(policy_hash="sha256:deadbeefdeadbeef",
                       model_id="guardrail.vla_stub.StubVLA", seed=0,
                       scene_path=None, sim_speedup=1.0, topology="dev",
                       hil_evidence=ev)
    except ValueError as e:
        assert "ardupilot_version" in str(e), e
        return
    raise AssertionError("dev accepted without the autopilot version")


def test_host_evidence_is_read_from_the_device_tree_and_l4t():
    from guardrail.manifest import collect_host_evidence
    files = {"/proc/device-tree/model":
             "NVIDIA Jetson AGX Orin Developer Kit\x00",
             "/etc/nv_tegra_release":
             "# R36 (release), REVISION: 3.0, GCID: 36191598, BOARD: generic, "
             "EABI: aarch64, DATE: Mon May  6 17:34:21 UTC 2024"}
    ev = collect_host_evidence(read=files.get, machine=lambda: "aarch64",
                               jetpack=lambda: "6.0+b106")
    assert ev["device_model"] == "NVIDIA Jetson AGX Orin Developer Kit", ev
    assert ev["l4t_release"] == "R36.3.0" and ev["jetpack"] == "6.0+b106"
    desk = collect_host_evidence(read=lambda p: None, machine=lambda: "x86_64",
                                 jetpack=lambda: None)
    assert desk["device_model"] is None and desk["l4t_release"] is None


def test_fcu_url_peers_are_parsed_and_loopback_is_recognised():
    from guardrail.manifest import fcu_peer, network_link_evidence
    assert fcu_peer("udp://:14555@192.168.10.2:14550") == "192.168.10.2"
    assert fcu_peer("udp://:14555@") is None
    assert fcu_peer("tcp://127.0.0.1:5760") == "127.0.0.1"
    assert fcu_peer("serial:///dev/ttyTHS1:921600") is None
    link = network_link_evidence("udp://:14555@192.168.10.2:14550",
                                 route_iface=lambda peer: "eth0")
    assert not link["peer_is_loopback"] and link["link_kind"] == "ethernet", link
    wifi = network_link_evidence("udp://:14555@10.0.0.5:14550",
                                 route_iface=lambda peer: "wlan0")
    assert wifi["link_kind"] == "wifi", wifi
    assert network_link_evidence("tcp://127.0.0.1:5760",
                                 route_iface=lambda p: "lo")["peer_is_loopback"]


def test_the_autopilot_version_reads_like_the_firmware_banner():
    from guardrail.manifest import decode_custom_version, describe_autopilot_version
    assert describe_autopilot_version(0x040507FF, "2a3dc4b7") == \
        "ArduCopter V4.5.7 (2a3dc4b7)"
    # The firmware behind the five stored ros2_* runs, as its logs print it.
    assert describe_autopilot_version(0x04080000, "5f119834") == \
        "ArduCopter V4.8.0-dev (5f119834)"
    ascii_bytes = [ord(c) for c in "2a3dc4b7"]
    assert decode_custom_version(ascii_bytes) == "2a3dc4b7"
    le_hex = bytes(ascii_bytes)[::-1].hex()       # MAVROS's uint64 rendering
    assert decode_custom_version(le_hex) == "2a3dc4b7", le_hex
    assert decode_custom_version("2a3dc4b7") == "2a3dc4b7"
    assert decode_custom_version("") is None
    assert decode_custom_version("not-a-hash") == "not-a-hash"


def test_the_autopilot_version_is_read_over_mavlink_or_reported_missing():
    from types import SimpleNamespace
    from guardrail.manifest import ardupilot_version_from_mavlink

    class _Mav:
        def __init__(self): self.sent = []
        def command_long_send(self, *a): self.sent.append(a)

    class _Master:
        target_system = target_component = 1
        mav_type = 2

        def __init__(self, reply):
            self.mav, self._reply = _Mav(), reply

        def recv_match(self, **kw):
            return self._reply

    reply = SimpleNamespace(flight_sw_version=0x040507FF,
                            flight_custom_version=[ord(c) for c in "2a3dc4b7"])
    got = ardupilot_version_from_mavlink(_Master(reply), timeout=0.01)
    assert got["ardupilot_version"] == "ArduCopter V4.5.7 (2a3dc4b7)", got
    assert got["flight_sw_version"] == "040507ff"
    assert ardupilot_version_from_mavlink(_Master(None), timeout=0.01) is None


def _two_weight_dirs():
    import os
    import tempfile
    d = Path(tempfile.mkdtemp(prefix="weights_"))
    a, b = d / "a", d / "b"
    a.mkdir(); b.mkdir()
    (a / "model.safetensors").write_bytes(b"\x01" * 4096)
    (b / "model.safetensors").write_bytes(b"\x02" * 4096)
    t = 1_700_000_000
    for f in (a / "model.safetensors", b / "model.safetensors"):
        os.utime(f, (t, t))
    return d, a, b


def test_weights_are_hashed_by_their_bytes_not_name_size_and_date():
    """X-16. Two different checkpoints with the same file name, size and
    mtime shared one hash under the old code (it hashed name+size+mtime and
    cut to 16 hex). A byte-identical copy written later got a DIFFERENT one."""
    import os
    import shutil as _sh
    from guardrail.manifest import model_hash
    d, a, b = _two_weight_dirs()
    os.environ["GUARDRAIL_DIGEST_CACHE"] = str(d / "cache.json")
    try:
        ha, hb = model_hash("lab/adapter", [a]), model_hash("lab/adapter", [b])
        assert ha != hb, "different weights, same hash"
        assert len(ha.rsplit(":", 1)[1]) == 64, ha
        c = d / "c"
        _sh.copytree(a, c)                       # same bytes, new mtime
        assert model_hash("lab/adapter", [c]) == ha, "same bytes, other hash"
    finally:
        os.environ.pop("GUARDRAIL_DIGEST_CACHE", None)
        _sh.rmtree(d, ignore_errors=True)


def test_weight_digests_are_cached_and_invalidated_by_an_edit():
    import os
    import shutil as _sh
    from guardrail import manifest as M
    d, a, _ = _two_weight_dirs()
    cache = d / "cache.json"
    calls = {"n": 0}
    real = M.file_sha256

    def counting(path, cache=None):
        before = dict(cache or {})
        out = real(path, cache)
        if cache is not None and cache != before:
            calls["n"] += 1
        return out
    M.file_sha256 = counting
    try:
        first = M.weights_digest([a], cache)
        assert calls["n"] == 1 and cache.is_file()
        assert M.weights_digest([a], cache) == first and calls["n"] == 1, \
            "the second launch re-hashed an unchanged file"
        (a / "model.safetensors").write_bytes(b"\x03" * 4096)
        assert M.weights_digest([a], cache) != first, "an edit was served stale"
        assert calls["n"] == 2
    finally:
        M.file_sha256 = real
        _sh.rmtree(d, ignore_errors=True)


def test_a_checkpoint_replaced_with_its_mtime_preserved_is_rehashed():
    """Review 2026-10-07. The cache key was path + size + mtime_ns, so a
    different checkpoint copied over the same path with its mtime preserved
    (`cp -p`, `rsync -a`, a LoRA adapter of the same shape) was served the
    OLD digest. Two ways in: a replacing copy (a new file at the path) and a
    same-size rewrite in place with the date put back."""
    import os
    import shutil as _sh
    import tempfile
    from guardrail import manifest as M
    d = Path(tempfile.mkdtemp(prefix="weights_"))
    cache = d / "cache.json"
    w = d / "adapter"
    w.mkdir()
    f = w / "adapter_model.safetensors"
    size = 3 * (1 << 16) + 123                     # past both 64 KiB samples
    f.write_bytes(b"\x01" * size)
    t = 1_700_000_000
    os.utime(f, (t, t))
    try:
        first = M.weights_digest([w], cache)
        # 1. cp -p: a different file of the same size and date replaces it;
        #    the difference sits in the MIDDLE, outside the end samples.
        other = d / "other.safetensors"
        body = bytearray(b"\x01" * size)
        body[size // 2] = 0x02
        other.write_bytes(bytes(body))
        os.utime(other, (t, t))
        os.replace(other, f)
        assert f.stat().st_mtime_ns == t * 10**9 and f.stat().st_size == size
        second = M.weights_digest([w], cache)
        assert second != first, "a replaced checkpoint was served the old digest"
        # 2. rewritten in place, same size, mtime put back; header changed.
        body = bytearray(f.read_bytes())
        body[0] = 0x03
        with open(f, "r+b") as fh:
            fh.write(bytes(body))
        os.utime(f, (t, t))
        third = M.weights_digest([w], cache)
        assert third != second, "an in-place rewrite was served the old digest"
        # And the cache still works: unchanged, nothing is re-hashed.
        assert M.weights_digest([w], cache) == third
    finally:
        _sh.rmtree(d, ignore_errors=True)


def test_a_code_only_pilot_carries_the_full_sha256_of_its_source():
    from guardrail.manifest import model_hash
    h = model_hash("guardrail.vla_stub.StubVLA")
    assert len(h.split("@src:")[1]) == 64, h


def test_the_pilot_record_names_what_flew_beside_the_six_fields():
    from guardrail.manifest import NO_VLA, model_hash, pilot_record
    assert model_hash(NO_VLA) == NO_VLA, "no VLA is a fact, not unresolved"
    rec = pilot_record(NO_VLA, "none", detector="google/owlvit-base-patch32")
    assert rec["kind"] == "none" and rec["hash"] == NO_VLA, rec
    stub = pilot_record("guardrail.vla_stub.StubVLA", "stub")
    assert "@src:" in stub["hash"]
    for bad in (("x", "none"), ("x", "model")):
        try:
            pilot_record(*bad)
        except ValueError:
            continue
        raise AssertionError(f"pilot_record accepted {bad}")
    # The manifest keeps exactly the grant's six fields.
    m = _dev_run()
    assert "pilot" not in m and len(m) == 6


def test_no_flight_entry_point_writes_the_old_label():
    """`canonical-hil` may be READ (old manifests); it must not be written."""
    for rel in ("sitl/ros2_shield_node.py", "sitl/run_sitl_demo.py",
                "demo/follow_vlm.py"):
        src = (ROOT / rel).read_text(encoding="utf-8")
        assert '"canonical-hil"' not in src and "TOPOLOGY_CANONICAL_HIL" not in src, rel


def test_a_code_only_pilot_is_pinned_by_its_source_not_left_unresolved():
    """StubVLA has no weights and no HuggingFace revision, but it is fully
    determined by its source file. Reporting 'unresolved' would wrongly say the
    run cannot be reproduced."""
    from guardrail.manifest import model_hash
    h = model_hash("guardrail.vla_stub.StubVLA")
    assert "@src:" in h, h
    assert "unresolved" not in h
    assert model_hash("guardrail.vla_stub.StubVLA") == h, "must be stable"


def test_an_unknown_model_id_still_says_unresolved():
    from guardrail.manifest import model_hash
    assert "unresolved" in model_hash("no.such.module.Thing")





def test_an_audit_record_after_a_hot_apply_carries_the_new_policy_hash():
    """hot_apply promises it: "every artefact after this instant carries a
    different policy_hash - the audit trail shows exactly which rules were
    active when". AuditLogger snapshotted the string at construction, so it
    did not. Reproduced: a record whose violation was `nfz-hot`, stamped with
    the hash of a policy that did not contain `nfz-hot`."""
    import json as _json
    import tempfile
    from pathlib import Path as _P
    from guardrail import load_policy as _lp
    from guardrail.audit import AuditLogger
    from guardrail.models import Action4D, PolygonFence, State
    from guardrail.shield import Shield

    pol = _lp(ROOT / "policies" / "demo_policy.yaml")
    sh = Shield(pol, lookahead_s=3.0, dt=0.5)
    path = _P(tempfile.mkdtemp()) / "audit.jsonl"
    al = AuditLogger(path, pol)              # the POLICY, not a snapshot string
    before = pol.policy_hash

    al.log(1, sh.filter(State(x=-20, y=-20, up=4), Action4D(vx=8)))
    zone = PolygonFence(
        id="nfz-hot", type="polygon_fence",
        vertices=[{"x": -25, "y": -25}, {"x": -15, "y": -25},
                  {"x": -15, "y": -15}, {"x": -25, "y": -15}])
    # After the first tick a Shield that enforces the grant's mid-flight
    # update model takes the zone only as a dynamic_nfz (2026-10-07).
    import guardrail.shield as _S
    sh.hot_apply(_S.as_dynamic_nfz(zone) if hasattr(_S, "as_dynamic_nfz") else zone)
    al.log(2, sh.filter(State(x=-20, y=-20, up=4), Action4D(vx=1)))

    recs = [_json.loads(l) for l in path.read_text(encoding="utf-8").splitlines()]
    assert len(recs) == 2, recs
    assert recs[0]["policy_hash"] == before
    assert recs[1]["policy_hash"] != before, "hot-applied rule did not restamp the hash"
    fired = [v["rule_id"] for v in recs[1]["violations"]]
    assert "nfz-hot" in fired, fired


def test_a_plain_hash_string_still_works_for_callers_that_pass_one():
    """Backward compatibility: the string form is a snapshot and stays one."""
    import tempfile
    from pathlib import Path as _P
    from guardrail import load_policy as _lp
    from guardrail.audit import AuditLogger

    pol = _lp(ROOT / "policies" / "demo_policy.yaml")
    al = AuditLogger(_P(tempfile.mkdtemp()) / "audit.jsonl", pol.policy_hash)
    assert al.policy_hash == pol.policy_hash


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
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
