"""The signed policy bundle, the WGS84 loader, and the Constraint Summary Pack.

Run either way:
    pytest tests/test_bundle.py -v
    python tests/test_bundle.py

WHY THIS FILE EXISTS

Three WP1/WP2 deliverables that the grant names and this repository had never
produced: a signed policy bundle, WGS84 as the authoring frame, and the CSP.

The bundle tests are mostly about REFUSAL. A round-trip test alone would pass on
a bundle that verifies nothing at all - write, read back, hashes agree, because
nothing in between ever checked. So each test below breaks the bundle in a
different way and requires the loader to notice.

The projection tests are mostly about the axis order. The reference
implementation returns (east, north); this project's frame is x = North,
y = East. A straight copy of its call would rotate every constraint ninety
degrees with nothing in the schema to object.
"""
import json
import sys
import tarfile
import tempfile
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import load_policy                                  # noqa: E402
from guardrail.bundle import (IR_NAME, MANIFEST_NAME, load_bundle,  # noqa: E402
                              write_bundle)
from guardrail.compiler import ConstraintCompiler                  # noqa: E402
from guardrail.models import State                                 # noqa: E402
from guardrail.projection import LocalProjection, project_raw      # noqa: E402
from guardrail.shield import Shield                                # noqa: E402

DEMO = ROOT / "policies" / "sim_demo_policy.yaml"
WGS84 = ROOT / "policies" / "wgs84_taipei.yaml"
PED = ROOT / "policies" / "sitl_pedestrian.yaml"


def _tmp(name="p.tar.gz"):
    return Path(tempfile.mkdtemp()) / name


def _repack(src: Path, dst: Path, edits: dict):
    """Rewrite named members of a bundle, leaving the rest alone."""
    members = {}
    with tarfile.open(src, "r:gz") as tar:
        for m in tar.getmembers():
            members[m.name] = tar.extractfile(m).read()
    members.update(edits)
    with tarfile.open(dst, "w:gz") as tar:
        for name, data in members.items():
            info = tarfile.TarInfo(name=name)
            info.size = len(data)
            info.mtime = 0
            tar.addfile(info, __import__("io").BytesIO(data))
    return dst


# --------------------------------------------------------------------------- #
# bundle: round trip
# --------------------------------------------------------------------------- #

def test_a_bundle_round_trips_to_an_identical_policy():
    pol = load_policy(DEMO)
    got = load_bundle(write_bundle(pol, _tmp()))
    assert got.policy_hash == pol.policy_hash
    assert got.policy_id == pol.policy_id
    assert len(got.constraints) == len(pol.constraints)


def test_the_bundle_carries_the_three_named_members():
    """The reference implementation's layout, matched so the two halves of the
    same grant can read each other's bundles."""
    with tarfile.open(write_bundle(load_policy(DEMO), _tmp()), "r:gz") as tar:
        names = sorted(tar.getnames())
    assert names == ["ir.json", "manifest.json", "signature.txt"], names


def test_the_same_policy_produces_byte_identical_bundles():
    """Member mtimes are pinned to 0 so a bundle is reproducible.

    Without that, two builds of an unchanged policy differ, and a hash of the
    ARCHIVE stops being a statement about the policy.
    """
    pol = load_policy(DEMO)
    a = write_bundle(pol, _tmp("a.tar.gz")).read_bytes()
    b = write_bundle(pol, _tmp("b.tar.gz")).read_bytes()
    assert a == b


def test_the_manifest_records_what_a_reviewer_needs():
    p = write_bundle(load_policy(DEMO), _tmp(), changelog="added school curfew")
    with tarfile.open(p, "r:gz") as tar:
        man = json.loads(tar.extractfile(MANIFEST_NAME).read())
    for f in ("policy_id", "version", "generation", "policy_hash",
              "signed_by", "changelog"):
        assert f in man, f"manifest missing {f}"
    assert man["changelog"] == "added school curfew"


# --------------------------------------------------------------------------- #
# bundle: refusal
# --------------------------------------------------------------------------- #

def test_a_tampered_IR_is_refused():
    """The test that gives the round-trip its meaning.

    Widen the no-fly zone's altitude ceiling in the shipped IR and leave the
    manifest alone. The bundle still opens, still parses, still describes a
    valid policy - and must be refused, because it is not the policy that was
    signed.
    """
    src = write_bundle(load_policy(DEMO), _tmp("ok.tar.gz"))
    with tarfile.open(src, "r:gz") as tar:
        ir = json.loads(tar.extractfile(IR_NAME).read())
    ir["constraints"][0]["altitude_ceiling_m"] = 9999.0
    bad = _repack(src, _tmp("bad.tar.gz"),
                  {IR_NAME: json.dumps(ir, sort_keys=True,
                                       separators=(",", ":")).encode()})
    try:
        load_bundle(bad)
    except ValueError as e:
        assert "policy_hash mismatch" in str(e), e
        return
    raise AssertionError("a tampered IR was accepted")


def test_a_manifest_from_another_bundle_is_refused():
    """Swap in a manifest whose hash belongs to a different policy.

    Caught by the signature line, which binds a hash to the identity that signed
    it - so the manifest and the signature must agree as well as the IR.
    """
    a = write_bundle(load_policy(DEMO), _tmp("a.tar.gz"))
    b = write_bundle(load_policy(PED), _tmp("b.tar.gz"))
    with tarfile.open(b, "r:gz") as tar:
        other_manifest = tar.extractfile(MANIFEST_NAME).read()
    bad = _repack(a, _tmp("mixed.tar.gz"), {MANIFEST_NAME: other_manifest})
    try:
        load_bundle(bad)
    except ValueError:
        return
    raise AssertionError("a foreign manifest was accepted")


def test_a_truncated_bundle_is_refused():
    src = write_bundle(load_policy(DEMO), _tmp("ok.tar.gz"))
    dst = _tmp("short.tar.gz")
    with tarfile.open(src, "r:gz") as tin, tarfile.open(dst, "w:gz") as tout:
        for m in tin.getmembers():
            if m.name == MANIFEST_NAME:
                continue
            data = tin.extractfile(m).read()
            info = tarfile.TarInfo(name=m.name)
            info.size, info.mtime = len(data), 0
            tout.addfile(info, __import__("io").BytesIO(data))
    try:
        load_bundle(dst)
    except ValueError as e:
        assert "missing required member" in str(e), e
        return
    raise AssertionError("a bundle missing its manifest was accepted")


def test_a_hot_applied_rule_is_carried_into_the_next_bundle():
    """Replayability: a bundle must describe the policy actually in force.

    `hot_apply` deliberately mutates the loaded policy - it appends the rule and
    bumps `generation`, which is what makes every artefact after that instant
    carry a different hash. So a bundle written afterwards has to contain the
    dynamic zone; one that did not would document a flight that never happened.
    """
    from guardrail.models import PolygonFence, XY
    pol = load_policy(DEMO)
    before_hash, before_n = pol.policy_hash, len(pol.constraints)

    sh = Shield(pol, lookahead_s=3.0, dt=0.5)
    sh.hot_apply(PolygonFence(
        id="nfz-dynamic", type="polygon_fence",
        vertices=[XY(x=-25, y=-25), XY(x=-15, y=-25),
                  XY(x=-15, y=-15), XY(x=-25, y=-15)], margin_m=1.0))

    assert pol.policy_hash != before_hash, "the hash must restamp"
    assert pol.generation == 1, pol.generation

    got = load_bundle(write_bundle(pol, _tmp(), changelog="dynamic NFZ at t+8s"))
    assert len(got.constraints) == before_n + 1
    assert any(c.id == "nfz-dynamic" for c in got.constraints)
    assert got.generation == 1, "the generation counter did not survive"


# --------------------------------------------------------------------------- #
# WGS84
# --------------------------------------------------------------------------- #

def test_the_projection_returns_north_then_east():
    """The axis order, which is the easy thing to get wrong.

    One degree of latitude north is ~111 km of X (North). One degree of
    longitude east is ~111 km * cos(lat) of Y (East). If these are transposed
    every constraint lands ninety degrees off with nothing to complain.
    """
    proj = LocalProjection(25.0, 121.0)
    x, y = proj.to_local(25.01, 121.0)          # north only
    assert x > 1000.0 and abs(y) < 1e-6, (x, y)
    x, y = proj.to_local(25.0, 121.01)          # east only
    assert y > 900.0 and abs(x) < 1e-6, (x, y)


def test_the_projection_round_trips():
    proj = LocalProjection(25.04245, 121.5314)
    lat, lon = proj.to_latlon(*proj.to_local(25.0428, 121.5318))
    assert abs(lat - 25.0428) < 1e-9 and abs(lon - 121.5318) < 1e-9


def test_a_wgs84_policy_loads_into_local_metres():
    pol = load_policy(WGS84)
    fence = pol.constraints[0]
    for v in fence.vertices:
        assert abs(v.x) < 200.0 and abs(v.y) < 200.0, (
            f"vertex {v} was not projected into the local frame")
    assert pol.origin is not None and abs(pol.origin.lat - 25.04245) < 1e-9


def test_a_wgs84_policy_actually_works_in_the_shield():
    """Projected or not, a fence is a fence."""
    pol = load_policy(WGS84)
    when = datetime(2026, 9, 7, 9, 0)                 # Monday, curfew in force
    sh = Shield(pol, lookahead_s=3.0, dt=0.5, now=lambda: when)
    assert sh.state_is_unsafe(State(x=0.0, y=0.0, up=20.0)), (
        "the origin sits inside the school-yard polygon and should be unsafe")
    assert not sh.state_is_unsafe(State(x=300.0, y=300.0, up=20.0))


def test_geographic_coordinates_without_an_origin_are_REFUSED():
    """An assumed origin does not fail - it relocates the policy and validates.

    That is the whole reason this raises instead of defaulting to (0, 0).
    """
    raw = {"policy_id": "p", "constraints": [
        {"id": "f", "type": "polygon_fence",
         "vertices": [{"lat": 25.0, "lon": 121.0}, {"lat": 25.1, "lon": 121.0},
                      {"lat": 25.1, "lon": 121.1}]}]}
    try:
        project_raw(raw)
    except ValueError as e:
        assert "origin" in str(e), e
        return
    raise AssertionError("a geographic policy with no origin was accepted")


def test_mixing_frames_in_one_point_is_refused():
    raw = {"policy_id": "p", "origin": {"lat": 25.0, "lon": 121.0},
           "constraints": [
               {"id": "f", "type": "polygon_fence",
                "vertices": [{"lat": 25.0, "lon": 121.0, "x": 1.0, "y": 2.0}]}]}
    try:
        project_raw(raw)
    except ValueError as e:
        assert "mix frames" in str(e), e
        return
    raise AssertionError("a point in two frames at once was accepted")


def test_every_existing_metre_policy_still_loads_untouched():
    """The support is ADDITIVE. If this fails, the change was a migration."""
    n = 0
    for p in sorted((ROOT / "policies").glob("*.yaml")):
        pol = load_policy(p)
        assert pol.constraints, p.name
        n += 1
    assert n >= 20, f"only {n} policies loaded"


def test_a_wgs84_policy_bundles_and_reloads():
    pol = load_policy(WGS84)
    got = load_bundle(write_bundle(pol, _tmp()))
    assert got.policy_hash == pol.policy_hash
    assert got.origin is not None, "the anchor for the metres was lost"


# --------------------------------------------------------------------------- #
# Constraint Summary Pack
# --------------------------------------------------------------------------- #

def test_the_pack_contains_every_rule_not_just_the_ones_with_shorthands():
    """REGRESSION: the stand-off the Shield enforces was invisible to the pilot.

    `build_prompt` used to emit fences, the altitude band and the speed cap and
    nothing else, so a policy whose headline rule is a 10 m pedestrian stand-off
    described none of it. The planner could only discover that rule by being
    repaired against it.
    """
    pack = ConstraintCompiler(load_policy(PED)).summary_pack()
    assert pack["n_rules"] == 4, pack["rules_by_type"]
    assert pack["rules_by_type"].get("subject_standoff") == 2, pack
    ids = {r["id"] for r in pack["rules"]}
    assert {"standoff-pedestrian", "standoff-any"} <= ids, ids


def test_the_prompt_mentions_the_standoff_in_words():
    """Checked against the sentence list, not the rendered YAML: safe_dump wraps
    long lines, so a substring test on the document fails on formatting rather
    than on content."""
    c = ConstraintCompiler(load_policy(PED))
    said = " ".join(c._sentences())
    assert "10 m away from any pedestrian" in said, said
    assert "5 m away from anything you are following" in said, said


def test_the_prompt_describes_a_corridor():
    c = ConstraintCompiler(load_policy(ROOT / "policies" / "corridor_survey.yaml"))
    said = " ".join(c._sentences())
    assert "Stay inside corridor" in said, said
    assert "20 m of its centerline" in said, said


def test_the_pack_is_pinned_to_the_policy_hash():
    """A summary that cannot be traced to a policy is a summary of nothing."""
    pol = load_policy(PED)
    assert ConstraintCompiler(pol).summary_pack()["policy_hash"] == pol.policy_hash


def test_the_pack_can_be_written_alongside_a_flight():
    out = Path(tempfile.mkdtemp()) / "csp.json"
    ConstraintCompiler(load_policy(PED)).write_summary_pack(out)
    got = json.loads(out.read_text(encoding="utf-8"))
    assert got["n_rules"] == 4


def test_the_prompt_keeps_the_shorthands_the_demos_already_read():
    """The pack is added underneath, not swapped in - this stays a drop-in."""
    import yaml as _y
    c = ConstraintCompiler(load_policy(DEMO))
    doc = _y.safe_load(c.build_prompt(c.parse_command("go to (30, 30) altitude 15")))
    cons = doc["constraints"]
    for f in ("no_fly_zones", "altitude_band_m", "speed_max_mps", "policy_hash"):
        assert f in cons, f"{f} disappeared from the prompt"
    assert cons["summary_pack"]["n_rules"] == len(load_policy(DEMO).constraints)


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
