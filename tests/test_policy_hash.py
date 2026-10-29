"""Policy identity: the hash, its legacy forms, the version lock, the schema pin.

Run either way:
    pytest tests/test_policy_hash.py -v
    python tests/test_policy_hash.py

WHY THIS FILE EXISTS

`Policy.policy_hash` used to digest `model_dump()`, which writes every optional
field as `null`. So when `valid_time` and `origin` joined the schema on
2026-09-01, every policy on disk changed fingerprint without one byte of YAML
changing, at version 0.1.0 throughout. Measured on 2026-10-06 with the code as
it then stood: 34 of the 76 stored flight manifests matched a policy in the
repository; 42 matched none, including all five ArduPilot + MAVROS 2 runs the
reports called KPI-grade. Nothing failed. Nothing warned. The runs simply
stopped being traceable, which is the one property the hash exists for.

The tests below pin four things: the new canonical form cannot be moved by an
additive schema change; every old 16-hex form is still recognised, so stored
runs verify again (76/76); a policy's version must change whenever its content
does (policies/policy.lock.json); and the IR schema's defaults are pinned, so
the next silent re-fingerprinting is a test failure instead of a discovery.
"""
import copy
import glob
import hashlib
import json
import re
import shutil
import sys
import tempfile
import typing
from pathlib import Path
from typing import Annotated, Literal, Union

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from pydantic import BaseModel, Field                               # noqa: E402

from guardrail.bundle import (LOCK_PATH, SCHEMA_PATH, check_lock,   # noqa: E402
                              derive, ir_schema_drift, load_lock, lock_key,
                              policy_candidates, policy_json_schema,
                              update_lock)
from guardrail.manifest import resolve_run_generation_chain, resolve_run_policy  # noqa: E402
from guardrail.models import (HASH_SCHEME, PRIOR_SCHEME_V2,         # noqa: E402
                              AltitudeEnvelope, Constraint, Corridor,
                              KinematicEnvelope, ObstacleClearance, Policy,
                              PolygonFence, SubjectStandoff, canonical_json,
                              ir_schema_defaults, load_policy)

POLICIES = ROOT / "policies"
DEMO = POLICIES / "sim_demo_policy.yaml"
PED = POLICIES / "sitl_pedestrian.yaml"
SKIP = "SKIP"

# Recorded in demo/out/ros2_shield_on/manifest.json by commit 4bafc63 (July
# schema) and quoted in audit card X-09; the September form is the hash the
# same unchanged file got from the code of 2026-09-01..2026-10-06.
JULY_SIM_DEMO = "sha256:8a311f9d22d22600"
SEPT_SIM_DEMO = "sha256:9a3782ae46925a46"
# Both --dynamic runs (ROS 2 and pymavlink) recorded this AFTER the mid-flight
# no-fly zone was hot-applied: sim_demo_policy + nfz-dynamic, generation 1.
DYNAMIC_RUNS = "sha256:77d64d2e5e94ac39"


# Fields the grant-form change (2026-10-06, later the same day) added to the
# IR, and `locked_by` (layer-merge provenance, 2026-10-19). Every one is None
# for any policy the old code could have held, so the old formula's dump of
# that era simply did not have them; a geographic point then held x/y only.
GRANT_FORM_FIELDS = frozenset({"scope", "layer", "altitude_ref", "issued_at",
                               "layers_merged", "locked_by"})


def _era_dump(node):
    """model_dump() as the schema of 2026-09-01..2026-10-06 produced it."""
    if isinstance(node, list):
        return [_era_dump(v) for v in node]
    if isinstance(node, dict):
        point = "x" in node and "y" in node
        return {k: _era_dump(v) for k, v in node.items()
                if not (k in GRANT_FORM_FIELDS and v is None)
                and not (point and k in ("lat", "lon"))}
    return node


def _old_policy_hash(pol) -> str:
    """The pre-2026-10-06 property body, verbatim, over that era's dump."""
    canon = json.dumps(_era_dump(pol.model_dump()), sort_keys=True, separators=(",", ":"))
    return "sha256:" + hashlib.sha256(canon.encode()).hexdigest()[:16]


def _policy_files():
    return sorted(POLICIES.glob("*.yaml"))


# --------------------------------------------------------------------------- #
# the hash
# --------------------------------------------------------------------------- #

def test_the_hash_is_the_full_sha256_of_the_canonical_bytes():
    pol = load_policy(DEMO)
    h = pol.policy_hash
    assert re.fullmatch(r"sha256:[0-9a-f]{64}", h), h
    assert h == "sha256:" + hashlib.sha256(pol.canonical_bytes()).hexdigest()
    assert pol.canonical_bytes() == canonical_json(pol.canonical_ir())
    assert b"null" not in pol.canonical_bytes(), "None must be omitted, not written"


def test_adding_an_optional_field_does_not_move_any_hash():
    """THE REGRESSION. A schema that grows an optional field - on the policy
    or on a rule - must leave every existing policy's hash where it was.

    Shown failing on the old code: the same experiment under the old formula
    (`_old_policy_hash`) moves the hash, which is exactly what happened on
    2026-09-01."""
    class PolygonFenceLater(PolygonFence):
        added_later: float | None = None

    class PolicyLater(Policy):
        added_later_too: str | None = None
        constraints: list[Annotated[
            Union[PolygonFenceLater, AltitudeEnvelope, KinematicEnvelope,
                  ObstacleClearance, SubjectStandoff, Corridor],
            Field(discriminator="type")]]

    import yaml
    from guardrail.projection import project_raw
    for f in _policy_files():
        raw = project_raw(yaml.safe_load(f.read_text(encoding="utf-8")))
        now, later = Policy.model_validate(raw), PolicyLater.model_validate(raw)
        assert later.policy_hash == now.policy_hash, (
            f"{f.name}: an unset optional field moved the hash")
    raw = project_raw(yaml.safe_load(DEMO.read_text(encoding="utf-8")))
    assert _old_policy_hash(PolicyLater.model_validate(raw)) != \
        _old_policy_hash(Policy.model_validate(raw)), (
            "the old formula should move under this change - if it does not, "
            "this test is no longer demonstrating anything")


def test_the_old_formula_is_exactly_the_include_defaults_legacy_form():
    """Every manifest written between 2026-09-01 and 2026-10-06 carries the
    old formula's output; the legacy form must reproduce it byte for byte."""
    for f in _policy_files():
        pol = load_policy(f)
        assert pol.legacy_hashes()["legacy16-include-defaults"] == \
            _old_policy_hash(pol), f.name


def test_the_short_hash_is_the_july_hash():
    """For the None-free schema of July/August, old 16-hex and new 64-hex are
    the same SHA-256 over the same bytes, one truncated. For a geographic
    policy that holds for the v2 form (projected metres), which is what every
    old era hashed; v3 stores lat/lon instead (2026-10-06)."""
    for f in _policy_files():
        pol = load_policy(f)
        full = pol.prior_hashes().get(PRIOR_SCHEME_V2, pol.policy_hash)
        assert full[:len("sha256:") + 16] == pol.legacy_hashes()["legacy16-exclude-none"], f.name
        assert pol.policy_hash.startswith(pol.policy_hash_short)
        assert (pol.policy_hash_short == pol.legacy_hashes()["legacy16-exclude-none"]) \
            == (not pol.is_geographic), f.name
    demo = load_policy(DEMO)
    assert demo.policy_hash_short == JULY_SIM_DEMO, demo.policy_hash_short
    assert demo.hash_form(JULY_SIM_DEMO) == "legacy16-exclude-none"
    assert demo.hash_form(SEPT_SIM_DEMO) == "legacy16-include-defaults"
    assert _old_policy_hash(demo) == SEPT_SIM_DEMO
    assert demo.hash_form(demo.policy_hash) == HASH_SCHEME


def test_a_foreign_or_missing_hash_matches_no_form():
    demo, ped = load_policy(DEMO), load_policy(PED)
    for h in [ped.policy_hash, ped.policy_hash_short,
              *ped.legacy_hashes().values(), "", None, "unresolved",
              "sha256:" + "0" * 64]:
        assert demo.hash_form(h) is None, h
        assert not demo.matches_hash(h)


def test_a_hot_apply_still_restamps_the_hash():
    """The property tests/test_shield.py also relies on: generation is content."""
    pol = load_policy(DEMO)
    before = pol.policy_hash
    pol.generation += 1
    assert pol.policy_hash != before


# --------------------------------------------------------------------------- #
# stored runs
# --------------------------------------------------------------------------- #

def test_the_dynamic_runs_rebuild_from_the_lock():
    """No file on disk produces a mid-flight hash; the lock's recipe does."""
    lock = load_lock()
    (d,) = [x for x in lock["derived"] if x["base"] == "fase3-sim-demo@0.1.1"]
    pol = derive(load_policy(DEMO), d["hot_apply"], d["generations"])
    assert pol.generation == 1
    assert pol.hash_form(DYNAMIC_RUNS) == "legacy16-exclude-none"
    assert pol.policy_hash == d["policy_hash"]
    assert pol.legacy_hashes() == d["legacy_hashes"]


def test_the_lock_recipe_is_the_fence_the_rails_hot_apply():
    """The recipe is data; the rails define DYNAMIC_FENCE in code. Read both
    sources and require agreement, or the recipe could drift from the flown
    zone and keep 'verifying' runs it no longer describes."""
    (d,) = load_lock()["derived"]
    want = d["hot_apply"][0]
    for rel in ("sitl/ros2_shield_node.py", "sitl/run_sitl_demo.py"):
        src = (ROOT / rel).read_text(encoding="utf-8")
        m = re.search(r"DYNAMIC_FENCE = PolygonFence\((.*?)\n\)", src, re.S)
        assert m, f"{rel}: DYNAMIC_FENCE not found"
        body = m.group(1)
        verts = [(float(x), float(y)) for x, y in
                 re.findall(r"XY\(x=(-?[\d.]+), y=(-?[\d.]+)\)", body)]
        assert verts == [(v["x"], v["y"]) for v in want["vertices"]], (rel, verts)
        assert f'id="{want["id"]}"' in body, rel
        assert f"margin_m={want['margin_m']}" in body, rel


def test_every_stored_manifest_resolves_to_a_policy():
    """The measurement behind this release, re-run on every test pass.

    Before (the old formula against every policies/*.yaml): 34 of 76, with all
    five MAVROS runs unmatched. After: every one, under a named form."""
    mans = sorted(glob.glob(str(ROOT / "demo" / "out" / "*" / "manifest.json")))
    if not mans:
        return SKIP                         # demo/out is gitignored
    cands = policy_candidates()
    old_index = {_old_policy_hash(p) for n, p in cands if n.endswith(".yaml")}
    before = after = 0
    unmatched = []
    for m in mans:
        man = json.loads(Path(m).read_text(encoding="utf-8"))
        before += man.get("policy_hash") in old_index
        pol, form, label = resolve_run_policy(man, cands)
        if pol is None and resolve_run_generation_chain(Path(m).parent, man.get("policy_hash"),
                                                        cands) is None:
            unmatched.append(Path(m).parent.name)
        else:
            # a run whose policy changed over REST in flight resolves through
            # its own policy_g<N>.json chain (guardrail.manifest, 30 Oct)
            after += 1
    assert not unmatched, f"stored runs that match no policy: {unmatched}"
    assert before < after, (before, after)
    for tag in ("ros2_shield_on", "ros2_shield_off", "ros2_shield_on_dynamic",
                "ros2_ped_on", "ros2_ped_off"):
        p = ROOT / "demo" / "out" / tag / "manifest.json"
        if p.is_file():
            man = json.loads(p.read_text(encoding="utf-8"))
            assert man["policy_hash"] not in old_index, (
                f"{tag} matched under the old code - the premise has changed")
            assert resolve_run_policy(man, cands)[0] is not None, tag


def _geo_policy(**rule_extra):
    """A policy the reference implementation can hold: WGS84 fence with no
    margin ring, an altitude envelope; scope / layer / altitude_ref absent and
    `repair` spelt this project's way, the two things its form writes out."""
    sq = [{"lat": 25.0421, "lon": 121.5310}, {"lat": 25.0428, "lon": 121.5310},
          {"lat": 25.0428, "lon": 121.5318}]
    return Policy.model_validate({"policy_id": "geo", "version": "0.1.0", "constraints": [
        {"id": "f", "type": "polygon_fence", "vertices": sq, "margin_m": 0.0,
         "altitude_floor_m": 0, "altitude_ceiling_m": 120, **rule_extra},
        {"id": "alt", "type": "altitude_envelope", "alt_min_m": 5, "alt_max_m": 90,
         "scope": "global"}]})


def test_the_reference_delta_rebuilds_the_source_exactly_and_nothing_else():
    """Deviation 10: a reference-form bundle carries the reference's hash
    (which its loader checks) and this project's hash of the SOURCE, rebuilt
    from the reference document plus `reference_delta`. The delta may only
    restore an absent default (scope, layer, altitude_ref) or a spelling
    (repair); anything that would change what a rule means fails the round
    trip (the rebuilt policy must write back the same reference document)."""
    pol = _geo_policy()
    doc = json.loads(json.dumps(pol.reference_ir()))
    delta = pol.reference_delta()
    assert delta["rules"]["f"] == {"absent": ["scope", "layer", "altitude_ref"],
                                   "violation_action": "repair"}, delta
    assert delta["rules"]["alt"] == {"absent": ["layer", "altitude_ref"],
                                     "violation_action": "repair"}, delta
    back = Policy.from_reference(doc, delta)
    assert back.policy_hash == pol.policy_hash
    assert Policy.model_validate(doc).policy_hash != pol.policy_hash, \
        "without the delta the document alone is a different policy (the third hash)"
    forged = [
        ("alt's written scope 'global' claimed absent (= mission)",
         {"rules": {**delta["rules"], "alt": {"absent": ["scope", "layer", "altitude_ref"],
                                              "violation_action": "repair"}}}),
        ("a breach action changed", {"rules": {**delta["rules"],
                                               "f": {**delta["rules"]["f"],
                                                     "violation_action": "brake"}}}),
        ("a field the reference does not default", {"rules": {
            "f": {"absent": ["priority"]}}}),
        ("a rule the document lacks", {"rules": {"ghost": {"absent": ["scope"]}}}),
    ]
    for name, bad in forged:
        try:
            Policy.from_reference(doc, bad)
        except ValueError:
            continue
        raise AssertionError(f"a forged delta was accepted: {name}")
    assert _geo_policy(margin_m=1.0).reference_delta() is None, "not representable"


def test_resolve_run_policy_refuses_rather_than_guesses():
    cands = policy_candidates()
    for man in ({}, {"policy_hash": ""}, {"policy_hash": "unresolved"},
                {"policy_hash": "sha256:" + "f" * 16}):
        assert resolve_run_policy(man, cands) == (None, None, None), man


# --------------------------------------------------------------------------- #
# the version lock
# --------------------------------------------------------------------------- #

def test_the_lock_is_clean():
    problems = check_lock()
    assert problems == [], "\n".join(problems)


def test_an_empty_lock_is_not_a_clean_lock():
    """'0 problems' from a lock that pins nothing would be a silent pass.

    The schema pin is present here on purpose: `lock={}` alone also lacks it,
    so the first version of this test passed on the schema message and never
    exercised the empty-policies one (2026-10-06 review, mutant M28)."""
    assert check_lock(lock={}), "an empty lock reported no problems"
    pinned = load_lock()["ir_schema"]
    empty_dir = Path(tempfile.mkdtemp(prefix="nopol_"))
    problems = check_lock(empty_dir, {"ir_schema": pinned, "policies": {}})
    assert any("pins no policies" in x for x in problems), problems


def _copy_policies() -> Path:
    d = Path(tempfile.mkdtemp(prefix="lock_"))
    for f in _policy_files():
        shutil.copy(f, d / f.name)
    return d


def test_changing_content_without_a_version_bump_is_caught():
    d = _copy_policies()
    f = d / DEMO.name
    f.write_text(f.read_text(encoding="utf-8").replace("alt_max_m: 20", "alt_max_m: 25"),
                 encoding="utf-8")
    problems = check_lock(d)
    assert any("content changed but version did not" in x and DEMO.name in x
               for x in problems), problems
    lock_path = d / "lock.json"
    lock_path.write_text(LOCK_PATH.read_text(encoding="utf-8"), encoding="utf-8")
    try:
        update_lock(d, lock_path)
    except ValueError as e:
        assert "content changed" in str(e), e
    else:
        raise AssertionError("update_lock papered over a content change")
    shutil.rmtree(d, ignore_errors=True)


def test_a_version_bump_is_locked_by_adding_never_by_rewriting():
    d = _copy_policies()
    f = d / DEMO.name
    f.write_text(f.read_text(encoding="utf-8")
                 .replace("alt_max_m: 20", "alt_max_m: 25")
                 .replace("version: 0.1.1", "version: 0.2.0"), encoding="utf-8")
    lock_path = d / "lock.json"
    lock_path.write_text(LOCK_PATH.read_text(encoding="utf-8"), encoding="utf-8")
    problems = check_lock(d, load_lock(lock_path))
    assert any("fase3-sim-demo@0.2.0 is not in the lock" in x for x in problems), problems
    assert update_lock(d, lock_path, locked_on="2026-10-06") == ["fase3-sim-demo@0.2.0"]
    new = load_lock(lock_path)
    assert new["policies"]["fase3-sim-demo@0.1.1"] == load_lock()["policies"]["fase3-sim-demo@0.1.1"], (
        "the old entry was rewritten; stored runs flown under 0.1.1 lose their anchor")
    assert check_lock(d, new) == []
    shutil.rmtree(d, ignore_errors=True)


GENERATED = ("hard_3", "hard_11", "hard_21", "hp_5", "hp_8", "hp_14", "hp_30", "hp_42")


def test_the_shared_identity_is_recorded_as_a_defect_not_hidden():
    """Nine generated files shared `random-scenario@0.1.0` with nine different
    contents until 2026-10-21. The lock entry that names them is never
    rewritten (its per-file pins still trace a run flown under the old id);
    random_scenario.yaml is the one file still claiming it, and a second
    claimant is still refused."""
    ent = load_lock()["policies"]["random-scenario@0.1.0"]
    assert len(ent["shared_by_files"]) == 9, sorted(ent["shared_by_files"])
    assert "known_defect" in ent
    d = _copy_policies()
    src = (d / "random_scenario.yaml").read_text(encoding="utf-8")
    (d / "hard_99.yaml").write_text(src + "\n# a second copy\n", encoding="utf-8")
    problems = check_lock(d)
    assert any("hard_99.yaml" in x and "own policy_id" in x for x in problems), problems
    shutil.rmtree(d, ignore_errors=True)


def test_a_content_change_inside_the_shared_identity_is_still_caught():
    """The file still under random-scenario@0.1.0 is pinned on its own, so the
    old collision is not a blind spot: editing it without a version bump is
    reported like any other content change (mutant M34). A file that moved to
    its own identity is judged by that identity's entry."""
    d = _copy_policies()
    for name, want in (("random_scenario.yaml", "content changed under random-scenario@0.1.0"),
                       ("hard_3.yaml", "content changed but version did not - "
                                       "random-scenario-hard-3@0.1.0")):
        f = d / name
        src = f.read_text(encoding="utf-8")
        assert "altitude_ceiling_m: 60\n" in src, src[:400]
        f.write_text(src.replace("altitude_ceiling_m: 60\n", "altitude_ceiling_m: 61\n", 1),
                     encoding="utf-8")
        problems = check_lock(d)
        assert any(name in x and want in x for x in problems), (name, problems)
        f.write_text(src, encoding="utf-8")
    assert check_lock(d) == [], check_lock(d)
    shutil.rmtree(d, ignore_errors=True)


def test_every_policy_file_has_its_own_policy_id():
    """WP1-22 (2026-10-21): an id@version must name one policy. Until then
    nine generated files said `policy_id: random-scenario` (failed then: one
    id, nine files). Now no two files under policies/ share an id."""
    seen: dict[str, list[str]] = {}
    for f in _policy_files():
        seen.setdefault(load_policy(f, runtime=False).policy_id, []).append(f.name)
    shared = {k: v for k, v in seen.items() if len(v) > 1}
    assert not shared, shared
    assert len(seen) == len(_policy_files()) >= 29, len(seen)


def _generator():
    import importlib
    sys.path.insert(0, str(ROOT / "demo"))
    try:
        return importlib.import_module("gen_random_scenario")
    finally:
        sys.path.remove(str(ROOT / "demo"))


def test_the_generator_gives_each_file_its_own_policy_id():
    """WP1-22: demo/gen_random_scenario.py wrote `policy_id: random-scenario`
    into every file it generated (the source of the nine-file collision).
    Now the default output keeps that id and any other file is
    `random-scenario-<stem>`; it refuses an id another file in the output
    folder claims; and its text reproduces each shipped generated file byte
    for byte (line endings aside), so the shipped ids are the generator's.
    Errors before the change: the module had no default_policy_id."""
    import yaml
    G = _generator()
    assert G.default_policy_id(POLICIES / "random_scenario.yaml") == "random-scenario"
    assert G.default_policy_id(POLICIES / "hard_3.yaml") == "random-scenario-hard-3"
    assert G.default_policy_id("out/My Scene_2.yaml") == "random-scenario-my-scene-2"
    for name in GENERATED + ("random_scenario",):
        f = POLICIES / f"{name}.yaml"
        doc = yaml.safe_load(f.read_text(encoding="utf-8"))
        assert doc["policy_id"] == G.default_policy_id(f), (name, doc["policy_id"])
        nfzs, band = [], None
        for c in doc["constraints"]:
            if c["type"] == "polygon_fence":
                v = c["vertices"]
                nfzs.append((v[0]["x"], v[1]["x"], v[0]["y"], v[2]["y"]))
            elif c["type"] == "altitude_envelope":
                band = (c["alt_min_m"], c["alt_max_m"])
        assert G.policy_yaml(doc["policy_id"], nfzs, *band) == \
            f.read_text(encoding="utf-8"), name
    # Read-only questions about policies/ itself (nothing is written):
    # regenerating a file is not a clash; a new file under a taken id is.
    assert G.id_claimed_elsewhere("random-scenario-hard-3", POLICIES / "hard_3.yaml") == []
    assert G.id_claimed_elsewhere("random-scenario-hard-3",
                                  POLICIES / "new.yaml") == ["hard_3.yaml"]
    assert G.id_claimed_elsewhere("random-scenario", POLICIES / "x.yaml") == [
        "random_scenario.yaml"]
    # Review of 3660383 (P7). The default ids are NOT unique by file name -
    # hard-3.yaml maps to hard_3.yaml's id - and the refusal is what keeps
    # them unique.
    assert G.default_policy_id(POLICIES / "hard-3.yaml") == "random-scenario-hard-3"
    assert G.id_claimed_elsewhere(G.default_policy_id(POLICIES / "hard-3.yaml"),
                                  POLICIES / "hard-3.yaml") == ["hard_3.yaml"]
    d = Path(tempfile.mkdtemp())
    try:
        # ... and an --out elsewhere is checked against policies/ too: until
        # the review this minted policies/hard_3.yaml's id for new content.
        assert G.id_claimed_elsewhere("random-scenario-hard-3", d / "hard_3.yaml") == [
            "policies/hard_3.yaml"]
        assert G.id_claimed_elsewhere(G.default_policy_id(d / "hp_99.yaml"),
                                      d / "hp_99.yaml") == []
        (d / "mine.yaml").write_text("policy_id: random-scenario-hp-99\n", encoding="utf-8")
        assert G.id_claimed_elsewhere("random-scenario-hp-99", d / "hp_99.yaml") == [
            "mine.yaml"]
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_a_file_that_left_a_shared_identity_must_be_locked_under_its_own():
    """The lock's rule for the WP1-22 move. A generated file now under its own
    id is clean only because its new entry names the old shared one as
    `previously` (written by `lock --update`, which only adds); the shared
    entry is not rewritten. Without that pointer the file reads as vanished
    from the shared identity; with no entry at all it is unlocked; and a file
    that still claims the old id stays pinned there."""
    lock = load_lock()
    ents = lock["policies"]
    for name in GENERATED:
        key = f"random-scenario-{name.replace('_', '-')}@0.1.0"
        assert ents[key]["file"] == f"{name}.yaml", key
        assert ents[key]["previously"] == "random-scenario@0.1.0", key
        assert f"{name}.yaml" in ents["random-scenario@0.1.0"]["shared_by_files"]
    assert check_lock(POLICIES, lock) == []
    no_ptr = copy.deepcopy(lock)
    del no_ptr["policies"]["random-scenario-hard-3@0.1.0"]["previously"]
    assert any("random-scenario@0.1.0" in x and "hard_3.yaml" in x and "no longer do" in x
               for x in check_lock(POLICIES, no_ptr)), check_lock(POLICIES, no_ptr)
    unlocked = copy.deepcopy(lock)
    del unlocked["policies"]["random-scenario-hp-8@0.1.0"]
    probs = check_lock(POLICIES, unlocked)
    assert any("hp_8.yaml" in x and "not in the lock" in x for x in probs), probs
    assert any("hp_8.yaml" in x and "no longer do" in x for x in probs), probs
    # lock --update writes the pointer itself, from the shared entry's list
    d = _copy_policies()
    try:
        before = copy.deepcopy(lock)
        for name in GENERATED:
            del before["policies"][f"random-scenario-{name.replace('_', '-')}@0.1.0"]
        lp = d / "lock.json"
        lp.write_text(json.dumps(before), encoding="utf-8")
        added = update_lock(d, lp, locked_on="2026-10-21")
        assert len(added) == len(GENERATED), added
        new = load_lock(lp)
        assert new["policies"]["random-scenario@0.1.0"] == lock["policies"]["random-scenario@0.1.0"]
        assert new["policies"]["random-scenario-hp-42@0.1.0"]["previously"] == \
            "random-scenario@0.1.0"
        assert check_lock(d, new) == []
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_the_lock_carries_old_and_new_hash_for_every_policy():
    """X-09: record the old -> new pairs so a stored run's hash is greppable.
    An entry locked under an earlier full-length scheme keeps that hash and
    carries the current one in `hashes` (wgs84_taipei, scheme v3)."""
    lock = load_lock()["policies"]
    for f in _policy_files():
        pol = load_policy(f)
        ent = lock[lock_key(pol)]
        ent = ent["shared_by_files"][f.name] if "shared_by_files" in ent else ent
        if ent["policy_hash"] != pol.policy_hash:
            assert pol.hash_form(ent["policy_hash"]) == PRIOR_SCHEME_V2, f.name
            assert ent["hashes"][HASH_SCHEME] == pol.policy_hash, f.name
        assert ent["legacy_hashes"] == pol.legacy_hashes(), f.name


def test_a_prior_scheme_entry_must_record_the_current_form():
    """The lock may not silently accept a v2 hash for a policy whose current
    hash it has never seen: until `lock --update` records the v3 form, it is a
    problem, and the update only ADDS that form (mutant: _pin_status treating
    any prior-form match as pinned)."""
    d = _copy_policies()
    lock_path = d / "lock.json"
    lock = load_lock()
    key = "wgs84-taipei-demo@0.1.0"
    del lock["policies"][key]["hashes"]
    lock_path.write_text(json.dumps(lock), encoding="utf-8")
    problems = check_lock(d, load_lock(lock_path))
    assert any("wgs84_taipei.yaml" in x and "not yet recorded" in x for x in problems), problems
    added = update_lock(d, lock_path, locked_on="2026-10-06")
    assert added == [f"{key}:{HASH_SCHEME}"], added
    new = load_lock(lock_path)["policies"][key]
    assert new["policy_hash"] == lock["policies"][key]["policy_hash"], "the old hash was rewritten"
    assert check_lock(d, load_lock(lock_path)) == []
    # A WRONG current form is a content change, not something to record.
    bad = load_lock(lock_path)
    bad["policies"][key]["hashes"][HASH_SCHEME] = "sha256:" + "0" * 64
    assert any("content changed but version did not" in x
               for x in check_lock(d, bad)), check_lock(d, bad)
    shutil.rmtree(d, ignore_errors=True)


# --------------------------------------------------------------------------- #
# the IR schema pin
# --------------------------------------------------------------------------- #

def test_the_ir_schema_is_pinned_and_unchanged():
    pinned = load_lock()["ir_schema"]["defaults"]
    assert ir_schema_drift(pinned) == [], ir_schema_drift(pinned)


def test_the_pin_allows_exactly_one_kind_of_change():
    base = ir_schema_defaults()
    add_none = copy.deepcopy(base)
    add_none["PolygonFence"]["added_later"] = "<absent>"
    assert ir_schema_drift(base, add_none) == [], "an optional-None field was refused"

    add_value = copy.deepcopy(base)
    add_value["PolygonFence"]["buffer_m"] = 2.0
    assert any("buffer_m" in x for x in ir_schema_drift(base, add_value))

    changed = copy.deepcopy(base)
    changed["PolygonFence"]["margin_m"] = 2.0
    assert any("margin_m" in x and "changes meaning" in x
               for x in ir_schema_drift(base, changed))

    removed = copy.deepcopy(base)
    del removed["SubjectStandoff"]["soft_margin_m"]
    assert any("soft_margin_m" in x and "removed" in x
               for x in ir_schema_drift(base, removed))


def test_schema_drift_reaches_check_lock_and_the_lock_command():
    """ir_schema_drift is only worth something if check_lock (and so
    `python -m guardrail.bundle lock` and test_the_lock_is_clean) runs it
    against the pin. A pinned default that no longer matches the code must
    surface there (mutant M35)."""
    lock = copy.deepcopy(load_lock())
    lock["ir_schema"]["defaults"]["PolygonFence"]["margin_m"] = 2.0
    problems = check_lock(lock=lock)
    assert any(x.startswith("IR schema: PolygonFence.margin_m") and "changes meaning" in x
               for x in problems), problems


def test_a_new_ir_model_is_reported_until_the_lock_pins_it():
    """A new constraint type moves no stored hash, but until it is pinned its
    defaults could change unseen. So it is reported, and `lock --update` pins
    it (adding, never rewriting an existing model's pin)."""
    pinned = load_lock()["ir_schema"]["defaults"]
    current = copy.deepcopy(ir_schema_defaults())
    current["Geofence3D"] = {"id": "<required>", "ceiling_m": 120.0}
    assert any(x.startswith("Geofence3D:") and "not pinned" in x
               for x in ir_schema_drift(pinned, current)), ir_schema_drift(pinned, current)

    d = _copy_policies()
    lock_path = d / "lock.json"
    lock = load_lock()
    del lock["ir_schema"]["defaults"]["Corridor"]           # as if Corridor were new
    lock_path.write_text(json.dumps(lock), encoding="utf-8")
    assert any(x.startswith("IR schema: Corridor:") and "not pinned" in x
               for x in check_lock(d, load_lock(lock_path)))
    added = update_lock(d, lock_path, locked_on="2026-10-06")
    assert added == ["ir_schema:Corridor"], added
    new = load_lock(lock_path)
    assert new["ir_schema"]["defaults"]["Corridor"] == ir_schema_defaults()["Corridor"]
    assert new["ir_schema"]["defaults"]["PolygonFence"] == pinned["PolygonFence"]
    assert check_lock(d, new) == []
    shutil.rmtree(d, ignore_errors=True)


def test_a_new_lock_entry_names_its_scheme_and_pins_stay_complete():
    """Shown failing before 2026-10-07: `_entry()` recorded no hash scheme, so
    an entry locked after the v3 change would carry a v3 hash under the lock's
    top-level v2 label; and the pins of models that existed before 2.1.0 did
    not list the fields 2.1.0 added, so the pin was not the model's field set.

    A new field defaulting to None is the one change allowed silently; `lock
    --update` records it in the pin (adding), never altering an existing pin."""
    d = _copy_policies()
    f = d / DEMO.name
    f.write_text(f.read_text(encoding="utf-8")
                 .replace("alt_max_m: 20", "alt_max_m: 25")
                 .replace("version: 0.1.1", "version: 0.2.0"), encoding="utf-8")
    lock = load_lock()
    del lock["ir_schema"]["defaults"]["PolygonFence"]["scope"]   # as before the re-pin
    margin_pin = lock["ir_schema"]["defaults"]["PolygonFence"]["margin_m"]
    lock_path = d / "lock.json"
    lock_path.write_text(json.dumps(lock), encoding="utf-8")
    assert not any("scope" in x for x in check_lock(d, load_lock(lock_path))), \
        "a new absent field is allowed without a report"
    added = update_lock(d, lock_path, locked_on="2026-10-07")
    assert added == ["ir_schema:PolygonFence.scope", "fase3-sim-demo@0.2.0"], added
    new = load_lock(lock_path)
    assert new["policies"]["fase3-sim-demo@0.2.0"]["hash_scheme"] == HASH_SCHEME
    assert "hash_scheme" not in new["policies"]["fase3-sim-demo@0.1.1"], \
        "an old entry is never rewritten; it stays under the top-level scheme"
    assert new["ir_schema"]["defaults"]["PolygonFence"]["scope"] == "<absent>"
    assert new["ir_schema"]["defaults"]["PolygonFence"]["margin_m"] == margin_pin
    shutil.rmtree(d, ignore_errors=True)
    # The real lock: every pin lists exactly its model's fields.
    pinned = load_lock()["ir_schema"]["defaults"]
    assert {m: set(fs) for m, fs in pinned.items()} == \
        {m: set(fs) for m, fs in ir_schema_defaults().items()}


def test_the_schema_walk_finds_a_constraint_type_nobody_listed():
    """The pin used to cover a hand-written tuple of eleven classes. A new rule
    type in the Constraint union - with a nested model of its own - must be
    found by walking the fields, not by remembering to edit a list.
    Shown failing on the pre-fix code: ir_schema_defaults took no root and
    knew only the eleven classes it named."""
    from guardrail.models import ConstraintBase, XY, ir_models

    class Ring(BaseModel):
        radius_m: float = 5.0

    class Geofence3D(ConstraintBase):
        type: Literal["geofence_3d"]
        ring: Ring
        ceiling_m: float = 120.0

    class PolicyLater(Policy):
        constraints: list[Annotated[
            Union[PolygonFence, AltitudeEnvelope, KinematicEnvelope,
                  ObstacleClearance, SubjectStandoff, Corridor, Geofence3D],
            Field(discriminator="type")]]

    later = ir_schema_defaults(PolicyLater)
    assert later["Geofence3D"]["ceiling_m"] == 120.0, later.get("Geofence3D")
    assert later["Ring"] == {"radius_m": 5.0}, later.get("Ring")
    names = {m.__name__ for m in ir_models()}
    union = typing.get_args(typing.get_args(Constraint)[0])
    assert {c.__name__ for c in union} <= names, names
    assert {"Policy", "Origin", XY.__name__, "ValidTime", "Recurrence"} <= names
    assert set(ir_schema_defaults()) == set(load_lock()["ir_schema"]["defaults"]), (
        "the walked schema and the pinned schema name different models")


# --------------------------------------------------------------------------- #
# the published schema
# --------------------------------------------------------------------------- #

def test_the_published_schema_is_current():
    """policies/policy_dsl.schema.json is the grant's 'DSL JSON Schema export';
    a schema change without regenerating it would publish a stale contract."""
    have = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    assert have == policy_json_schema(), (
        "policy_dsl.schema.json is stale - run python -m guardrail.bundle schema")


def test_every_policy_validates_against_the_published_schema():
    try:
        import jsonschema
    except ImportError:
        return SKIP
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    for f in _policy_files():
        jsonschema.validate(load_policy(f).canonical_ir(), schema)
    # A grant-form document's IR (WGS84 points, no x/y) validates too.
    ref = ROOT / "kuanting-vla-uav-guardrail" / "bundles" / "itri-icl-2026-demo.yaml"
    if ref.is_file():
        jsonschema.validate(load_policy(ref, runtime=False).canonical_ir(), schema)
    # And it says no: a negative speed cap, a misspelt key, a point in no frame.
    bad_cap = load_policy(DEMO).canonical_ir()
    bad_cap["constraints"][2]["speed_max_mps"] = -1.0
    typo = load_policy(DEMO).canonical_ir()
    typo["constraints"][0]["altitude_cieling_m"] = 40.0
    nowhere = load_policy(DEMO).canonical_ir()
    nowhere["constraints"][0]["vertices"][0] = {"x": 1.0}
    for name, bad in (("negative speed cap", bad_cap), ("misspelt key", typo),
                      ("half a point", nowhere)):
        try:
            jsonschema.validate(bad, schema)
        except jsonschema.ValidationError:
            continue
        raise AssertionError(f"the published schema accepted a {name}")


if __name__ == "__main__":
    # A test that cannot run here prints SKIP, never PASS.
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (fixture or dependency missing)")
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
