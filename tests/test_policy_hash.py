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

# The four ConstraintBase fields the grant requires (IR schema 3.0.0, 2026-11-02;
# scope since 4.0.0, 2026-11-06), at the values they defaulted to before
# (repair = project_fix; an absent scope meant mission).
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}

POLICIES = ROOT / "policies"
DEMO = POLICIES / "sim_demo_policy.yaml"
WGS84_YAML = POLICIES / "wgs84_taipei.yaml"
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
    """The pre-2026-10-06 property body, verbatim, over that era's dump. That
    era's model held the IR 2.x shape (rings as written, `repair`), which is
    what `_ir2_dump` gives back since IR schema 3.0.0. A WGS84 point held the
    UNROUNDED projection of its day; since IR schema 3.1.0 the "era" context
    gives it (a plain dump gives the view, rounded to the micrometre)."""
    canon = json.dumps(_era_dump(pol._ir2_dump(mode="python", exclude_none=False,
                                               context={"points": "era"})),
                       sort_keys=True, separators=(",", ":"))
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
    old era hashed; v3 stores lat/lon instead (2026-10-06). Since IR schema
    3.0.0 (2026-11-02) the bytes those eras hashed are the IR 2.x view
    (`ir2_hashes`), and the July hash belongs to the file's OLD version: the
    migrated file at its new version answers to it only when put back
    (`as_version`, which is what the lock's `migrated_from` records)."""
    from guardrail.bundle import as_version
    from guardrail.models import IR2_FORM
    for f in _policy_files():
        pol = load_policy(f)
        full = (pol.prior_hashes().get(PRIOR_SCHEME_V2)
                or pol.ir2_hashes().get(IR2_FORM, pol.policy_hash))
        assert full[:len("sha256:") + 16] == pol.legacy_hashes()["legacy16-exclude-none"], f.name
        assert pol.policy_hash.startswith(pol.policy_hash_short)
    demo = load_policy(DEMO)
    assert demo.version == "0.1.4" and demo.hash_form(JULY_SIM_DEMO) is None, \
        "the July hash is fase3-sim-demo@0.1.1's, not the 0.1.4's (WGS84, scope stated)"
    # 0.1.1 (IR 2.x) and 0.1.2 (IR 3.0.0) were written in metres; since the
    # WGS84 conversion (IR schema 3.1.0, 2026-11-03) the file answers for them
    # through its metre form (Policy.metre_form, bundle.rewind), and since
    # scope was stated (IR schema 4.0.0, 2026-11-06, 0.1.4) through the metre
    # form of the file with scope left out (Policy.ir3_form).
    assert as_version(demo, "0.1.1").hash_form(JULY_SIM_DEMO) is None
    assert as_version(demo.metre_form(), "0.1.1").hash_form(JULY_SIM_DEMO) is None, \
        "a metre form that states scope is not the July document"
    old = as_version(demo.ir3_form().metre_form(), "0.1.1")
    assert old.ir2_hashes()[IR2_FORM][:len("sha256:") + 16] == JULY_SIM_DEMO
    assert old.hash_form(JULY_SIM_DEMO) == "legacy16-exclude-none"
    assert old.hash_form(SEPT_SIM_DEMO) == "legacy16-include-defaults"
    assert _old_policy_hash(old) == SEPT_SIM_DEMO
    assert old.hash_form(load_lock()["policies"]["fase3-sim-demo@0.1.1"]["policy_hash"]) \
        == IR2_FORM
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
    """No file on disk produces a mid-flight hash; the lock's recipe does. The
    recipe's base is fase3-sim-demo@0.1.1, which sim_demo_policy.yaml left for
    0.1.2 at IR schema 3.0.0, for the WGS84 0.1.3 at 3.1.0 and for 0.1.4 at
    4.0.0 (scope stated): the base is the metre form of the file with scope
    left out, put back at 0.1.1, and the recipe - written under IR 2.x, its
    rule omits the three ConstraintBase fields and scope - is read with the
    2.x defaults (derive, IR2_RULE_DEFAULTS), scope left absent as it flew."""
    from guardrail.bundle import as_version
    from guardrail.models import IR2_FORM
    lock = load_lock()
    (d,) = [x for x in lock["derived"] if x["base"] == "fase3-sim-demo@0.1.1"]
    pol = derive(as_version(load_policy(DEMO).ir3_form().metre_form(), "0.1.1"),
                 d["hot_apply"], d["generations"])
    assert pol.generation == 1
    assert all(c.scope is None for c in pol.constraints), "the 2.x recipe gained a scope"
    assert pol.hash_form(DYNAMIC_RUNS) == "legacy16-exclude-none"
    assert pol.hash_form(d["policy_hash"]) == IR2_FORM
    assert pol.legacy_hashes() == d["legacy_hashes"]
    assert any(n == d["name"] and p.hash_form(DYNAMIC_RUNS) for n, p in policy_candidates())
    # A recipe written under 3.0.0 must state the three fields, as a policy must.
    try:
        derive(load_policy(DEMO), d["hot_apply"], ir_schema_version="3.0.0")
    except ValueError as e:
        assert "Field required" in str(e), e
    else:
        raise AssertionError("a 3.0.0 recipe without constraint_type was accepted")
    # ... and one written under 4.0.0 must state scope too (WP1-01), while the
    # same rule recorded under 3.x reads with scope absent, as it flew.
    rule = {**d["hot_apply"][0], "constraint_type": "hard", "priority": "P0",
            "violation_action": "project_fix"}
    for era in ("4.0.0", "9.0.0"):
        try:
            derive(load_policy(DEMO), [rule], ir_schema_version=era)
        except ValueError as e:
            assert "scope" in str(e) and "Field required" in str(e), e
        else:
            raise AssertionError(f"a {era} recipe without scope was accepted")
    got = derive(load_policy(DEMO), [rule], ir_schema_version="3.1.0")
    assert got.constraints[-1].scope is None and got.constraints[-1].effective_scope == "mission"
    stated = derive(load_policy(DEMO), [{**rule, "scope": "mission"}], ir_schema_version="4.0.0")
    assert stated.constraints[-1].scope == "mission"
    assert stated.policy_hash != got.policy_hash, "an absent scope hashed as a stated one"


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


def _geo_policy(era: str | None = None, **rule_extra):
    """A policy the reference implementation can hold: WGS84 fence with no
    margin ring, an altitude envelope; layer / altitude_ref absent, the things
    its form writes out. The fence states scope mission (required since IR
    schema 4.0.0) and the envelope global. With `era` (an IR schema version
    before 4.0.0) the fence leaves scope out, as a document of that era could,
    and is read in that era (models.era_context). The breach action is
    written `repair`, the spelling read as project_fix since IR schema 3.0.0."""
    from guardrail.models import era_context
    sq = [{"lat": 25.0421, "lon": 121.5310}, {"lat": 25.0428, "lon": 121.5310},
          {"lat": 25.0428, "lon": 121.5318}]
    rep = {**HARD_P0, "violation_action": "repair"}
    fence = {"id": "f", "type": "polygon_fence", **rep, "vertices": sq, "margin_m": 0.0,
             "altitude_floor_m": 0, "altitude_ceiling_m": 120, **rule_extra}
    if era is not None:
        fence.pop("scope")
    return Policy.model_validate({"policy_id": "geo", "version": "0.1.0", "constraints": [
        fence,
        {"id": "alt", "type": "altitude_envelope", **rep, "alt_min_m": 5, "alt_max_m": 90,
         "scope": "global"}]}, context=None if era is None else era_context(era))


def test_the_reference_delta_rebuilds_the_source_exactly_and_nothing_else():
    """Deviation 10: a reference-form bundle carries the reference's hash
    (which its loader checks) and this project's hash of the SOURCE, rebuilt
    from the reference document plus `reference_delta`. The delta may only
    restore an absent default (layer, altitude_ref; scope in a delta written
    before IR 4.0.0, which a policy of that era could leave out) or a spelling
    (repair, in a delta written before IR 3.0.0 - the IR no longer holds it);
    anything that would change what a rule means fails the round trip (the
    rebuilt policy must write back the same reference document)."""
    from guardrail.models import era_context
    pol = _geo_policy()
    assert {c.violation_action for c in pol.constraints} == {"project_fix"}
    doc = json.loads(json.dumps(pol.reference_ir()))
    delta = pol.reference_delta()
    assert delta["rules"]["f"] == {"absent": ["layer", "altitude_ref"]}, delta
    assert delta["rules"]["alt"] == {"absent": ["layer", "altitude_ref"]}, delta
    back = Policy.from_reference(doc, delta)
    assert back.policy_hash == pol.policy_hash
    # A delta written under IR 2.x carried the `repair` spelling; it still
    # rebuilds the same policy.
    old = {"rules": {k: {**v, "violation_action": "repair"} for k, v in delta["rules"].items()}}
    assert Policy.from_reference(doc, old).policy_hash == pol.policy_hash
    assert Policy.model_validate(doc).policy_hash != pol.policy_hash, \
        "without the delta the document alone is a different policy (the third hash)"
    # IR schema 4.0.0: the same document from a policy of an earlier era,
    # whose fence left scope out. Its delta restores the absence, which only
    # that era allows: in its era it rebuilds the source exactly; read as
    # today's, it is refused (scope is required), never rebuilt with a guess.
    pol3 = _geo_policy(era="3.1.0")
    assert pol3.constraints[0].scope is None and pol3.ir_schema_version == "3.1.0"
    assert json.loads(json.dumps(pol3.reference_ir())) == doc, "the reference reads absent as mission"
    delta3 = pol3.reference_delta()
    assert delta3["rules"]["f"] == {"absent": ["scope", "layer", "altitude_ref"]}, delta3
    assert Policy.from_reference(doc, delta3, context=era_context("3.1.0")).policy_hash == \
        pol3.policy_hash != pol.policy_hash
    for ctx in (None, era_context("4.0.0")):
        try:
            Policy.from_reference(doc, delta3, context=ctx)
        except ValueError as e:
            assert "scope" in str(e), e
        else:
            raise AssertionError(f"a delta restoring an absent scope was accepted in {ctx}")
    forged = [
        ("alt's written scope 'global' claimed absent (= mission)",
         {"rules": {**delta3["rules"], "alt": {"absent": ["scope", "layer", "altitude_ref"]}}}),
        ("a breach action changed", {"rules": {**delta["rules"],
                                               "f": {**delta["rules"]["f"],
                                                     "violation_action": "brake"}}}),
        ("a field the reference does not default", {"rules": {
            "f": {"absent": ["priority"]}}}),
        ("a rule the document lacks", {"rules": {"ghost": {"absent": ["layer"]}}}),
    ]
    for name, bad in forged:
        for ctx in (None, era_context("3.1.0")):
            try:
                Policy.from_reference(doc, bad, context=ctx)
            except ValueError:
                continue
            raise AssertionError(f"a forged delta was accepted ({ctx}): {name}")
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
    cur = load_policy(DEMO).version
    f.write_text(f.read_text(encoding="utf-8")
                 .replace("alt_max_m: 20", "alt_max_m: 25")
                 .replace(f"version: {cur}", "version: 0.2.0"), encoding="utf-8")
    lock_path = d / "lock.json"
    lock_path.write_text(LOCK_PATH.read_text(encoding="utf-8"), encoding="utf-8")
    problems = check_lock(d, load_lock(lock_path))
    assert any("fase3-sim-demo@0.2.0 is not in the lock" in x for x in problems), problems
    assert update_lock(d, lock_path, locked_on="2026-10-06") == ["fase3-sim-demo@0.2.0"]
    new = load_lock(lock_path)
    key = f"fase3-sim-demo@{cur}"
    assert new["policies"][key] == load_lock()["policies"][key], (
        f"the old entry was rewritten; stored runs flown under {cur} lose their anchor")
    assert "migrated_from" not in new["policies"]["fase3-sim-demo@0.2.0"], \
        "a real content change was recorded as a migration"
    assert check_lock(d, new) == []
    shutil.rmtree(d, ignore_errors=True)


def test_every_migrated_entry_reproduces_the_hash_it_replaces():
    """IR schema 3.0.0 (2026-11-02) moved every shipped policy's hash (`repair`
    stored as project_fix, rings in canonical order), so each file raised its
    version and its new lock entry names the old one in `migrated_from`. The
    claim is re-proved, not trusted: put back at the old version the file
    reproduces the old entry's recorded hash and legacy hashes, and a run
    recorded under the old version resolves to it (policy_candidates). A
    forged or broken link is reported by check_lock.

    Since 2026-11-03 (IR schema 3.1.0) the 31 metre policies were converted
    to WGS84 and raised their version again: a chain of two migrations, each
    step read back from the step after it (bundle.rewind) - the frame
    migration through the file's metre form (FRAME_FORM), the IR 3.0.0 one
    as before. Since 2026-11-06 (IR schema 4.0.0) all 32 state `scope` and
    raised their version a third time: the scope migration is read back
    through the file with scope left out (Policy.ir3_form, IR3_FORM), which
    is that old file exactly - every rule's scope absent, as it was stored -
    before the older steps."""
    from guardrail.bundle import FRAME_FORM, IR3_FORM, _semver_key, rewind
    from guardrail.models import IR2_FORMS
    lock = load_lock()
    ents = lock["policies"]
    cands = policy_candidates()
    frame_steps = ir2_steps = scope_steps = 0
    for f in _policy_files():
        pol = load_policy(f, runtime=False)
        ent = ents[lock_key(pol)]
        assert ent.get("ir_schema_version") == "4.0.0", f.name
        assert {c.scope for c in pol.constraints} == {"mission"}, f.name
        base = pol
        while ent.get("migrated_from"):
            old_key = ent["migrated_from"]
            assert _semver_key(old_key.rpartition("@")[2]) < _semver_key(base.version), \
                (f.name, old_key)
            old_ent = ents[old_key]
            pin = old_ent["shared_by_files"][f.name] if "shared_by_files" in old_ent else old_ent
            got = rewind(base, old_key, pin)
            assert got is not None, (f.name, old_key)
            old, form = got
            assert form == ent["migration"]["old_hash_form"], (f.name, form, ent["migration"])
            if form == IR3_FORM:
                scope_steps += 1
                assert ent["migration"]["ir_schema_version"] == "4.0.0", f.name
                assert all(c.scope is None for c in old.constraints), f.name
                assert old.ir_schema_version == "3.1.0", f.name
            elif form.startswith(FRAME_FORM):
                frame_steps += 1
                assert ent["migration"]["ir_schema_version"] == "3.1.0", f.name
                assert not old.is_geographic and old.origin is None, f.name
            else:
                ir2_steps += 1
                assert form in (PRIOR_SCHEME_V2, *IR2_FORMS), (f.name, form)
            assert old.legacy_hashes() == pin["legacy_hashes"], f.name
            assert pol.hash_form(pin["policy_hash"]) is None, \
                f"{f.name}: the old hash is not the new"
            got, _, label = resolve_run_policy({"policy_hash": pin["policy_hash"]}, cands)
            assert got is not None and label == f"{f.name} as {old_key}", (f.name, label)
            short = pin["legacy_hashes"]["legacy16-exclude-none"]
            assert resolve_run_policy({"policy_hash": short}, cands)[2] == \
                f"{f.name} as {old_key}", (f.name, old_key)
            ent, base = old_ent, old
            if "shared_by_files" in ent:
                break
    assert (scope_steps, frame_steps, ir2_steps) == (32, 31, 32), \
        (scope_steps, frame_steps, ir2_steps)
    assert check_lock(POLICIES, lock) == []
    # Broken links are caught: a hash the rewound file does not reproduce,
    # and a migration claimed from another policy's entry.
    key = lock_key(load_policy(DEMO))
    bad = copy.deepcopy(lock)
    bad["policies"][bad["policies"][key]["migrated_from"]]["policy_hash"] = "sha256:" + "0" * 64
    assert any(key in x and "migrated_from" in x for x in check_lock(POLICIES, bad)), \
        check_lock(POLICIES, bad)
    forged = copy.deepcopy(lock)
    forged["policies"][key]["migrated_from"] = "follow-car@0.1.0"
    assert any(key in x and "migrated_from" in x for x in check_lock(POLICIES, forged)), \
        check_lock(POLICIES, forged)


def test_a_file_left_at_its_old_version_after_the_ir_change_is_caught():
    """The lock's rule holds across an IR change: a policy whose canonical
    bytes moved must raise its version. Put back at its old version, a
    shipped file is reported as changed - not recorded as 'the same content
    under a newer scheme', which is the treatment a prior hash SCHEME gets.
    The same holds for the WGS84 conversion (IR schema 3.1.0): the converted
    file at its metre version is another content under that version; and for
    scope (IR schema 4.0.0): the file that states it, put back at the version
    that left it out (0.1.3), is another content under that version too."""
    d = _copy_policies()
    f = d / DEMO.name
    src = f.read_text(encoding="utf-8")
    for now, was in (("version: 0.1.4", "version: 0.1.3"), ("version: 0.1.4", "version: 0.1.2"),
                     ("version: 0.1.4", "version: 0.1.1")):
        assert now in src, src[:300]
        f.write_text(src.replace(now, was), encoding="utf-8")
        probs = check_lock(d)
        assert any(DEMO.name in x and "content changed but version did not" in x
                   for x in probs), (was, probs)
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
    # Since IR schema 3.0.0 random_scenario.yaml is random-scenario@0.1.1, an
    # entry of its own, so the second claimant meets the ordinary refusal.
    assert any("hard_99.yaml" in x and ("own policy_id" in x or "must not share" in x)
               for x in problems), problems
    shutil.rmtree(d, ignore_errors=True)


def test_a_content_change_inside_the_shared_identity_is_still_caught():
    """Every file that shared random-scenario@0.1.0 is now judged by an entry
    of its own (the hard_*/hp_* files since 2026-10-21, random_scenario.yaml
    since it moved to 0.1.1 at IR schema 3.0.0), so editing one without a
    version bump is reported like any other content change (mutant M34). The
    shared entry still pins each file's old content."""
    d = _copy_policies()
    # 0.1.2 since the WGS84 conversion (IR schema 3.1.0, 2026-11-03), 0.1.3
    # since scope was stated (IR schema 4.0.0, 2026-11-06).
    for name, want in (("random_scenario.yaml", "content changed but version did not - "
                                                "random-scenario@0.1.3"),
                       ("hard_3.yaml", "content changed but version did not - "
                                       "random-scenario-hard-3@0.1.3")):
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
    G = _generator()
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        import convert_policies_wgs84 as CV
    finally:
        sys.path.remove(str(ROOT / "tools"))
    assert G.default_policy_id(POLICIES / "random_scenario.yaml") == "random-scenario"
    assert G.default_policy_id(POLICIES / "hard_3.yaml") == "random-scenario-hard-3"
    assert G.default_policy_id("out/My Scene_2.yaml") == "random-scenario-my-scene-2"
    for name in GENERATED + ("random_scenario",):
        f = POLICIES / f"{name}.yaml"
        pol = load_policy(f, runtime=False)
        assert pol.policy_id == G.default_policy_id(f), (name, pol.policy_id)
        # Since the WGS84 conversion (2026-11-03) the shipped file was the
        # generator's metre text at its metre version, converted by
        # tools/convert_policies_wgs84.py; since scope was stated (IR schema
        # 4.0.0, 2026-11-06) it is that text with `scope: mission` on every
        # rule, one version later. Read the rules back in metres.
        metre = pol.metre_form()
        nfzs, band = [], None
        for c in metre.constraints:
            if c.type == "polygon_fence":
                v = c.given_vertices()
                nfzs.append((v[0].x, v[1].x, v[0].y, v[2].y))
            elif c.type == "altitude_envelope":
                band = (c.alt_min_m, c.alt_max_m)
        shipped = f.read_text(encoding="utf-8")
        metre_ver = re.search(r"from the\n# metres of (\d+\.\d+\.\d+)\)", shipped).group(1)
        wgs_ver = load_lock()["policies"][lock_key(pol)]["migrated_from"].rpartition("@")[2]

        def unscoped(t):
            return t.replace("    scope: mission\n", "")
        # The 3.1.0 file (wgs_ver): the conversion of the metre text, which
        # stated no scope; the generator writes it now, so it is taken out.
        text = unscoped(G.policy_yaml(pol.policy_id, nfzs, *band, version=metre_ver))
        assert "scope" not in text, name
        assert CV.convert_text(text, CV.anchor_for(f.name))[0] == \
            unscoped(shipped).replace(f"version: {pol.version}\n", f"version: {wgs_ver}\n"), name
        # ... and today's file: the generator writes it itself, scope stated
        # on every rule, at its version.
        assert shipped.count("    scope: mission\n") == len(pol.constraints), name
        assert G.wgs84_policy_yaml(pol.policy_id, nfzs, *band, version=pol.version) \
            .replace("Generated by demo/gen_random_scenario.py in WGS84",
                     f"WGS84 since 2026-11-03 (tools/convert_policies_wgs84.py, from the\n"
                     f"# metres of {metre_ver})") == shipped, name
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
        key = lock_key(load_policy(POLICIES / f"{name}.yaml", runtime=False))
        assert key.startswith(f"random-scenario-{name.replace('_', '-')}@"), key
        assert ents[key]["file"] == f"{name}.yaml", key
        assert ents[key]["previously"] == "random-scenario@0.1.0", key
        assert f"{name}.yaml" in ents["random-scenario@0.1.0"]["shared_by_files"]
    assert check_lock(POLICIES, lock) == []
    hard3 = lock_key(load_policy(POLICIES / "hard_3.yaml", runtime=False))
    hp8 = lock_key(load_policy(POLICIES / "hp_8.yaml", runtime=False))
    no_ptr = copy.deepcopy(lock)
    del no_ptr["policies"][hard3]["previously"]
    assert any("random-scenario@0.1.0" in x and "hard_3.yaml" in x and "no longer do" in x
               for x in check_lock(POLICIES, no_ptr)), check_lock(POLICIES, no_ptr)
    unlocked = copy.deepcopy(lock)
    del unlocked["policies"][hp8]
    probs = check_lock(POLICIES, unlocked)
    assert any("hp_8.yaml" in x and "not in the lock" in x for x in probs), probs
    assert any("hp_8.yaml" in x and "no longer do" in x for x in probs), probs
    # lock --update writes the pointer itself, from the shared entry's list
    d = _copy_policies()
    try:
        before = copy.deepcopy(lock)
        for name in GENERATED:
            del before["policies"][lock_key(load_policy(POLICIES / f"{name}.yaml",
                                                        runtime=False))]
        lp = d / "lock.json"
        lp.write_text(json.dumps(before), encoding="utf-8")
        added = update_lock(d, lp, locked_on="2026-10-21")
        assert len(added) == len(GENERATED), added
        new = load_lock(lp)
        assert new["policies"]["random-scenario@0.1.0"] == lock["policies"]["random-scenario@0.1.0"]
        hp42 = lock_key(load_policy(POLICIES / "hp_42.yaml", runtime=False))
        assert new["policies"][hp42]["previously"] == "random-scenario@0.1.0"
        # 0.1.3 (scope stated, IR 4.0.0, 2026-11-06) migrated from 0.1.2
        # (WGS84, 2026-11-03), itself from 0.1.1 (IR 3.0.0, metres)
        assert new["policies"][hp42]["migrated_from"] == "random-scenario-hp-42@0.1.2"
        assert new["policies"][hp42]["migration"]["old_hash_form"] == \
            "sha256-canonical-v3/ir-3.x"
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
    # Since IR schema 3.0.0 wgs84_taipei.yaml is locked under the current
    # scheme; a v2-era entry for it, as the lock held for 0.1.0, is rebuilt
    # here so the mechanism keeps its test.
    pol = load_policy(WGS84_YAML)
    key = lock_key(pol)
    lock["policies"][key] = {"file": WGS84_YAML.name, "locked_on": "2026-10-06",
                             "policy_hash": pol.prior_hashes()[PRIOR_SCHEME_V2],
                             "legacy_hashes": pol.legacy_hashes()}
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


def test_the_pin_allows_no_change_at_all():
    """The grant's freeze rule (R6): every IR-shape change after the freeze
    carries a semver bump. Until 2026-11-02 a new field defaulting to None
    passed this check silently (failed then: `add_none` reported nothing),
    and that is how `locked_by` went in unbumped (2385f84)."""
    base = ir_schema_defaults()
    assert ir_schema_drift(base, copy.deepcopy(base)) == []
    add_none = copy.deepcopy(base)
    add_none["PolygonFence"]["added_later"] = "<absent>"
    assert any("PolygonFence.added_later" in x for x in ir_schema_drift(base, add_none)), \
        "an optional-None field passed the freeze unreported"

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


def _bumped(version: str) -> str:
    major, minor, patch = (int(x) for x in version.split("."))
    return f"{major}.{minor + 1}.0"


def test_an_additive_change_needs_a_version_bump_and_then_re_pins():
    """THE FREEZE RULE (WP1-22 / X-09, 2026-11-02). A new model and a new
    None-default field move no stored hash - and are still IR-shape changes,
    so check_lock reports them and `lock --update` refuses them until
    IR_SCHEMA_VERSION is raised. Failed before 2026-11-02: check_lock was
    clean for the deleted Corridor pin once `lock --update` had re-added it
    unbumped, and silent for a missing `scope` pin. With a bump, the update
    re-pins and records what changed under `ir_schema.migrations`."""
    import guardrail.bundle as B
    pinned = load_lock()["ir_schema"]["defaults"]
    current = copy.deepcopy(ir_schema_defaults())
    current["Geofence3D"] = {"id": "<required>", "ceiling_m": 120.0}
    assert any(x.startswith("Geofence3D:") and "not pinned" in x
               for x in ir_schema_drift(pinned, current)), ir_schema_drift(pinned, current)

    d = _copy_policies()
    lock_path = d / "lock.json"
    lock = load_lock()
    del lock["ir_schema"]["defaults"]["Corridor"]                  # as if Corridor were new
    del lock["ir_schema"]["defaults"]["PolygonFence"]["scope"]     # as if scope were new
    lock_path.write_text(json.dumps(lock), encoding="utf-8")
    probs = check_lock(d, load_lock(lock_path))
    for name in ("Corridor:", "PolygonFence.scope:"):
        assert any(x.startswith(f"IR schema: {name}") and "semver bump" in x
                   for x in probs), (name, probs)
    try:
        update_lock(d, lock_path, locked_on="2026-11-02")
    except ValueError as e:
        assert "without a version bump" in str(e) and "Corridor" in str(e), e
    else:
        raise AssertionError("lock --update pinned an IR-shape change without a bump")
    assert load_lock(lock_path) == lock, "a refused update wrote the lock"

    # The same change under a raised version: reported until re-pinned, then
    # re-pinned with the change recorded, and clean.
    saved = B.IR_SCHEMA_VERSION
    B.IR_SCHEMA_VERSION = _bumped(saved)
    try:
        probs = check_lock(d, load_lock(lock_path))
        assert any("lock --update" in x and B.IR_SCHEMA_VERSION in x for x in probs), probs
        added = update_lock(d, lock_path, locked_on="2026-11-02")
        assert added == [f"ir_schema:version {B.IR_SCHEMA_VERSION}"], added
        new = load_lock(lock_path)
        irs = new["ir_schema"]
        assert irs["version"] == B.IR_SCHEMA_VERSION
        assert irs["previous_versions"][-1] == saved
        mig = irs["migrations"][-1]
        assert (mig["from"], mig["to"], mig["on"]) == (saved, B.IR_SCHEMA_VERSION, "2026-11-02")
        assert any("Corridor" in x for x in mig["changes"]), mig
        assert any("PolygonFence.scope" in x for x in mig["changes"]), mig
        assert irs["defaults"] == ir_schema_defaults()
        assert new["policies"] == lock["policies"], "a re-pin touched a policy entry"
        assert check_lock(d, new) == []
    finally:
        B.IR_SCHEMA_VERSION = saved
    shutil.rmtree(d, ignore_errors=True)


def test_the_schema_version_never_goes_back():
    import guardrail.bundle as B
    lock = copy.deepcopy(load_lock())
    lock["ir_schema"]["version"] = _bumped(B.IR_SCHEMA_VERSION)
    probs = check_lock(lock=lock)
    assert any("never goes back" in x for x in probs), probs


def test_every_ir_schema_version_has_a_migration_note():
    """The other half of the freeze rule: the bump carries a migration note.
    docs/IR-MIGRATIONS.md must have a row for the current version and for
    every version the lock has pinned."""
    from guardrail.models import IR_SCHEMA_VERSION
    text = (ROOT / "docs" / "IR-MIGRATIONS.md").read_text(encoding="utf-8")
    versions = set()
    for line in text.splitlines():
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if line.startswith("|") and len(cells) >= 3:
            versions.add(cells[2].split(" ")[0])
    irs = load_lock()["ir_schema"]
    for v in {IR_SCHEMA_VERSION, irs["version"], *irs.get("previous_versions", [])}:
        assert v in versions, f"docs/IR-MIGRATIONS.md has no row for IR schema {v}"
    assert irs["version"] == IR_SCHEMA_VERSION


def test_a_new_lock_entry_names_its_scheme_and_pins_stay_complete():
    """Shown failing before 2026-10-07: `_entry()` recorded no hash scheme, so
    an entry locked after the v3 change would carry a v3 hash under the lock's
    top-level v2 label; and the pins of models that existed before 2.1.0 did
    not list the fields 2.1.0 added, so the pin was not the model's field set."""
    d = _copy_policies()
    f = d / DEMO.name
    src = f.read_text(encoding="utf-8")
    old = load_policy(DEMO).version
    f.write_text(src.replace("alt_max_m: 20", "alt_max_m: 25")
                 .replace(f"version: {old}", "version: 9.0.0"), encoding="utf-8")
    lock = load_lock()
    lock_path = d / "lock.json"
    lock_path.write_text(json.dumps(lock), encoding="utf-8")
    added = update_lock(d, lock_path, locked_on="2026-10-07")
    assert added == ["fase3-sim-demo@9.0.0"], added
    new = load_lock(lock_path)
    assert new["policies"]["fase3-sim-demo@9.0.0"]["hash_scheme"] == HASH_SCHEME
    assert "hash_scheme" not in new["policies"]["fase3-sim-demo@0.1.1"], \
        "an old entry is never rewritten; it stays under the top-level scheme"
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
