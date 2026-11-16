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

Since 2026-10-06 the signature is a real Ed25519 signature, so the refusals now
include the ones a placeholder could never make: an edited manifest, a
consistent forgery by someone without the key, an untrusted signer, and a
signature stripped from a bundle that still names its key. Signature tests use
a throw-away key and trust store, never the lab key. Where `cryptography` is
missing (the 3.11 env, the WSL flight venvs) the key is the pure-Python RFC
8032 test key and verification is the pure-Python fallback - so these tests
RUN there, which is the point: those are the environments the fallback is for.
The fallback is pinned to the RFC's own test vectors on every interpreter.

Since 2026-11-16 the bundle key is certified by a root CA (WP1-04 / X-10), so
there is a second thing to refuse: a certificate that the listed authority did
not issue for the key that signed. Those tests make their own CAs in temp
folders with the function that made the lab's root; the lab's CA key is never
read and the repository's trust store is never written. They need
`cryptography` (X.509 is not re-implemented) and SKIP without it; what an
interpreter without it must do is tested by switching it off.

The projection tests are mostly about the axis order. The reference
implementation returns (east, north); this project's frame is x = North,
y = East. A straight copy of its call would rotate every constraint ninety
degrees with nothing in the schema to object.
"""
import contextlib
import hashlib
import io
import json
import os
import subprocess
import sys
import tarfile
import tempfile
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import guardrail.bundle as B                                       # noqa: E402
from guardrail import load_policy                                  # noqa: E402
from guardrail.bundle import (IR_NAME, MANIFEST_NAME,              # noqa: E402
                              SIGNATURE_NAME, BundleSignatureError,
                              check_bundle, load_bundle, write_bundle)
from guardrail.compiler import ConstraintCompiler                  # noqa: E402
from guardrail.models import State                                 # noqa: E402
from guardrail.projection import LocalProjection, project_raw      # noqa: E402
from guardrail.shield import Shield                                # noqa: E402

# The four ConstraintBase fields the grant requires (IR schema 3.0.0, 2026-11-02;
# scope since 4.0.0, 2026-11-06), at the values they defaulted to before
# (repair = project_fix; an absent scope meant mission).
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}

DEMO = ROOT / "policies" / "sim_demo_policy.yaml"
WGS84 = ROOT / "policies" / "wgs84_taipei.yaml"
PED = ROOT / "policies" / "sitl_pedestrian.yaml"
ISSUED = "2026-10-06T00:00:00Z"
SKIP = "SKIP"


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
            tar.addfile(info, io.BytesIO(data))
    return dst


def _members(path: Path) -> dict:
    with tarfile.open(path, "r:gz") as tar:
        return {m.name: tar.extractfile(m).read() for m in tar.getmembers()}


def _unsigned(pol, name="p.tar.gz", **kw):
    """An integrity-only bundle: written UNSIGNED on purpose."""
    return write_bundle(pol, _tmp(name), signer=None, issued_at=ISSUED, **kw)


def _test_key(kind: str = "auto"):
    """A throw-away signer and a trust store that trusts only it.

    `cryptography` makes the key where it is installed; elsewhere (or with
    kind="py") the pure-Python RFC 8032 test key does, so every signature test
    runs on the interpreters that lack `cryptography` instead of skipping
    there."""
    d = Path(tempfile.mkdtemp())
    trust = d / "trust.json"
    if B.HAVE_CRYPTO and kind != "py":
        signer = B.generate_signing_key(d / "k.pem", authority="test-lab",
                                        trust_store=trust)
    else:
        signer = B.Signer(authority="test-lab",
                          private_key=B._PyEd25519TestKey(os.urandom(32)))
        B.register_public_key(signer, trust)
    return signer, trust


class _no_crypto:
    """Run a block as an interpreter without `cryptography` would; with
    fallback=False, as one with no verifier at all."""

    def __init__(self, fallback: bool = True):
        self.fallback = fallback

    def __enter__(self):
        self.saved = (B.HAVE_CRYPTO, B.PY_VERIFY_FALLBACK)
        B.HAVE_CRYPTO, B.PY_VERIFY_FALLBACK = False, self.fallback

    def __exit__(self, *exc):
        B.HAVE_CRYPTO, B.PY_VERIFY_FALLBACK = self.saved
        return False


def _strip_signature(signed: Path, dst: Path, forge_ir: bool = True) -> Path:
    """The 2026-10-06 review's forgery: edit the IR, recompute the hash, keep
    the trusted signer's identity, and drop only the Ed25519 line."""
    m = _members(signed)
    ir_b, man = m[IR_NAME], json.loads(m[MANIFEST_NAME])
    if forge_ir:
        ir = json.loads(ir_b)
        ir["constraints"][0]["altitude_ceiling_m"] = 5.0
        ir_b = json.dumps(ir, sort_keys=True, separators=(",", ":"),
                          ensure_ascii=True).encode()
        man["policy_hash"] = "sha256:" + hashlib.sha256(ir_b).hexdigest()
    head = m[SIGNATURE_NAME].decode().splitlines()[0].split(" ", 1)[1]
    return _repack(signed, dst, {
        IR_NAME: ir_b,
        MANIFEST_NAME: json.dumps(man, indent=2, sort_keys=True).encode(),
        SIGNATURE_NAME: f"{man['policy_hash']} {head}\n".encode()})


# --------------------------------------------------------------------------- #
# bundle: round trip
# --------------------------------------------------------------------------- #

def test_a_bundle_round_trips_to_an_identical_policy():
    pol = load_policy(DEMO)
    got = load_bundle(_unsigned(pol), require_signature=False)
    assert got.policy_hash == pol.policy_hash
    assert got.policy_id == pol.policy_id
    assert got.canonical_ir() == pol.canonical_ir()
    assert len(got.constraints) == len(pol.constraints)


def _next_version(pol, edit):
    """`pol` re-validated as version 9.9.9 with its rules passed through
    `edit(list of rule dicts) -> list of rule dicts`."""
    d = pol.model_dump(mode="json")
    d["version"] = "9.9.9"
    d["constraints"] = edit([dict(c) for c in d["constraints"]])
    return type(pol).model_validate(d)


def test_the_changelog_is_a_diff_against_the_previous_version():
    """X-10 (grant, Policy DSL page): the bundle's changelog is "a diff
    against the previous version". Until 2026-10-29 it was free text
    ("initial" by default). With `previous` the manifest carries
    `changelog_diff` - rules added, removed and changed by id, with the
    fields that changed, against the previous version's policy_hash - signed
    with the rest of the manifest, and the default text becomes its reading.
    Without it the manifest is exactly what it was."""
    prev = load_policy(DEMO)
    ids = [c.id for c in prev.constraints]
    assert {"alt-band", "kin-caps", "nfz-square"} <= set(ids), ids

    def edit(rules):
        out = []
        for r in rules:
            if r["id"] == "nfz-square":
                continue                                     # removed
            if r["id"] == "alt-band":
                r["alt_max_m"] = 25.0                        # changed
            out.append(r)
        out.append({"id": "alt-low", "type": "altitude_envelope", "scope": "mission",
                    "priority": "P0", "violation_action": "project_fix",   # added
                    "constraint_type": "hard", "alt_min_m": 5.0, "alt_max_m": 30.0})
        return out

    new = _next_version(prev, edit)
    assert B.manifest_for(new) == B.manifest_for(new, previous=None)
    assert "changelog_diff" not in B.manifest_for(new)
    path = write_bundle(new, _tmp(), signer=None, issued_at=ISSUED, previous=prev)
    chk = check_bundle(path)
    diff = chk.manifest["changelog_diff"]
    assert diff["against"] == {"policy_id": prev.policy_id, "version": prev.version,
                               "policy_hash": prev.policy_hash}, diff["against"]
    assert diff["added"] == ["alt-low"] and diff["removed"] == ["nfz-square"], diff
    assert diff["changed"] == {"alt-band": ["alt_max_m"]}, diff["changed"]
    assert diff["policy_fields"] == ["version"], diff["policy_fields"]
    text = chk.manifest["changelog"]
    assert text.startswith(f"against {prev.policy_id} {prev.version}"), text
    assert "added alt-low" in text and "removed nfz-square" in text, text
    assert "changed alt-band (alt_max_m)" in text, text
    assert load_bundle(path, require_signature=False).policy_hash == new.policy_hash

    # A hand-written changelog is kept beside the diff; a previous version
    # given as its policy file reads the same; the same policy diffs empty.
    m = B.manifest_for(new, "raised the ceiling", previous=load_policy(DEMO))
    assert m["changelog"] == "raised the ceiling" and m["changelog_diff"] == diff
    assert B._previous_policy(DEMO).policy_hash == prev.policy_hash
    same = B.policy_diff(prev, prev)
    assert (same["added"], same["removed"], same["changed"], same["policy_fields"]) == \
        ([], [], {}, []), same
    assert B.diff_text(same).endswith("no rule changed")
    # A rule replaced by another type under the same id names `type`.
    swapped = _next_version(prev, lambda rs: [
        {"id": "kin-caps", "type": "altitude_envelope", "scope": "mission",
         "priority": "P0", "violation_action": "project_fix", "constraint_type": "hard",
         "alt_min_m": 1.0, "alt_max_m": 200.0}
        if r["id"] == "kin-caps" else r for r in rs])
    assert "type" in B.policy_diff(prev, swapped)["changed"]["kin-caps"]


def test_the_scope_migration_reads_as_a_diff_against_the_previous_version():
    """IR schema 4.0.0 (2026-11-06): every shipped policy states `scope` and
    raised its patch version. Its bundle's changelog against the version
    before - rebuilt from the lock as it was issued, scope absent (Policy.
    ir3_form) - names that change and nothing else: every rule's `scope`,
    and the policy's version. The previous version keeps the hash its own
    bundle recorded."""
    now = load_policy(DEMO)
    lock = B.load_lock()["policies"]
    prev_key = lock[B.lock_key(now)]["migrated_from"]
    prev = B.as_version(now.ir3_form(), prev_key.rpartition("@")[2])
    assert prev.policy_hash == lock[prev_key]["policy_hash"]
    diff = B.policy_diff(prev, now)
    assert (diff["added"], diff["removed"]) == ([], []), diff
    assert diff["changed"] == {c.id: ["scope"] for c in now.constraints}, diff["changed"]
    assert diff["policy_fields"] == ["version"], diff["policy_fields"]
    m = B.manifest_for(now, previous=prev)
    assert m["ir_schema_version"] == "4.0.0" and m["changelog_diff"] == diff
    assert all(f"changed {c.id} (scope)" in m["changelog"] for c in now.constraints), m["changelog"]
    # The previous version, bundled again (a replay would), records the
    # schema its IR is a document of and reads back under it.
    path = write_bundle(prev, _tmp(), signer=None, issued_at=ISSUED)
    chk = check_bundle(path)
    assert chk.manifest["ir_schema_version"] == "3.1.0"
    assert chk.policy.policy_hash == prev.policy_hash == chk.manifest["policy_hash"]


def test_a_previous_version_that_does_not_verify_is_refused():
    """The previous version named by a bundle path is read with load_bundle's
    signature checks: an unsigned one is refused, not diffed against."""
    prev = load_policy(DEMO)
    old = _unsigned(prev, "old.tar.gz")
    new = _next_version(prev, lambda rs: rs)
    try:
        write_bundle(new, _tmp(), signer=None, issued_at=ISSUED, previous=old)
    except ValueError as exc:
        assert "sign" in str(exc).lower(), exc
    else:
        raise AssertionError("an unsigned previous bundle was diffed against")


def test_the_bundle_carries_the_three_named_members():
    """The reference implementation's container layout, plus (since
    2026-11-02, WP2-16) the multi-scale geometry, which the reference's
    loader ignores. The container alone did not make the bundles
    exchangeable (the IRs differ); the cross-load is tested in
    tests/test_policy_dsl_grant_form.py."""
    with tarfile.open(_unsigned(load_policy(DEMO)), "r:gz") as tar:
        names = sorted(tar.getnames())
    assert names == ["ir.json", "manifest.json", "multiscale.json", "signature.txt"], names


def _ms(bundle: Path) -> dict:
    return json.loads(_members(bundle)[B.MULTISCALE_NAME])


def test_the_bundle_ships_the_multiscale_geometry_and_the_compiler_reads_it():
    """WP2-16, the grant's locked choice: "the policy bundle's IR already
    carries pre-computed multi-scale polygons (Policy DSL ingest produces
    them). The Prefix Compiler picks the scale, never re-computes." Until
    2026-11-02 the bundle carried the IR only and every reader recomputed the
    scales at load (failed then: no multiscale.json, no digest). Now the
    scales are computed once, at ingest, shipped under the signed manifest,
    and a policy loaded from the bundle compiles its CSP with the coarse-ring
    computation switched OFF - which a policy loaded from YAML cannot."""
    import guardrail.compiler as K
    import guardrail.ir as IRM
    from guardrail.compiler import Mission
    survey = ROOT / "policies" / "corridor_survey.yaml"
    pol = load_policy(survey)
    b = _unsigned(pol)
    m = _members(b)
    man = json.loads(m[MANIFEST_NAME])
    assert man["multiscale_sha256"] == "sha256:" + hashlib.sha256(m[B.MULTISCALE_NAME]).hexdigest()
    doc = json.loads(m[B.MULTISCALE_NAME])
    assert doc["policy_hash"] == pol.policy_hash and doc["format"] == IRM.MULTISCALE_FORMAT
    assert set(doc["rules"]) == {c.id for c in pol.constraints
                                 if c.type in IRM.FENCE_TYPES + IRM.TUBE_TYPES}
    loaded = check_bundle(b).policy
    assert loaded.shipped_multiscale == doc["rules"]
    mission = Mission(task_text="survey", target_x=60.0, target_y=60.0,
                      cruise_alt_m=15.0, speed_pref_mps=4.0)
    now = datetime(2026, 11, 2, 9, 0)                    # a Monday, school hours
    real = IRM.coarse_ring

    def refuse(*a, **k):
        raise AssertionError("the coarse scale was recomputed")
    K._IR_CACHE.clear()
    IRM.coarse_ring = refuse
    try:
        csp = ConstraintCompiler(loaded).compile_csp(mission, now=now)
        ir = K.compiled_ir(loaded)
        assert set(ir.multiscale_source.values()) == {"bundle"}, ir.multiscale_source
        refs = [p.geometry_ref for p in csp.P0_constraints if not p.geometry_ref.startswith("param:")]
        assert refs, csp.P0_constraints
        for ref in refs:
            scale, rest = ref.split(":", 1)
            rid = rest.rsplit("@v", 1)[0]
            assert K.resolve_geometry_ref(loaded, ref) == \
                [tuple(p) for p in doc["rules"][rid][scale]], ref
        try:
            K._IR_CACHE.clear()
            ConstraintCompiler(load_policy(survey)).compile_csp(mission, now=now)
        except AssertionError:
            pass                                    # YAML: computed at load
        else:
            raise AssertionError("the YAML path never computed a coarse ring: "
                                 "this test proves nothing")
    finally:
        IRM.coarse_ring = real
        K._IR_CACHE.clear()


def test_shipped_geometry_that_does_not_describe_the_policy_is_refused():
    """The shipped scales are covered by the signature (their digest is in the
    manifest) and checked against the policy - without recomputing them: the
    fine scale is the zone, the bbox its bounds, the coarse ring convex and
    containing it. Each forgery below keeps the digest consistent, as someone
    without the signing key could on an unsigned bundle."""
    pol = load_policy(ROOT / "policies" / "corridor_survey.yaml")
    b = _unsigned(pol)
    doc = _ms(b)
    zid = next(c.id for c in pol.constraints if c.type == "polygon_fence")
    fine = doc["rules"][zid]["fine"]

    def forged(edit, name):
        d = json.loads(json.dumps(doc))
        edit(d)
        ms = json.dumps(d, sort_keys=True, separators=(",", ":")).encode()
        mm = _members(b)
        man = json.loads(mm[MANIFEST_NAME])
        man["multiscale_sha256"] = "sha256:" + hashlib.sha256(ms).hexdigest()
        return _repack(b, _tmp(name), {B.MULTISCALE_NAME: ms,
                                       MANIFEST_NAME: json.dumps(man, indent=2,
                                                                 sort_keys=True).encode()})
    cx = sum(p[0] for p in fine) / len(fine)
    cy = sum(p[1] for p in fine) / len(fine)
    cases = {
        "does not contain": lambda d: d["rules"][zid].__setitem__(
            "coarse", [[cx + 0.5 * (x - cx), cy + 0.5 * (y - cy)] for x, y in fine]),
        "not convex": lambda d: d["rules"][zid].__setitem__(
            "coarse", [[cx, cy]] + d["rules"][zid]["coarse"]),
        "not the zone's vertices": lambda d: d["rules"][zid]["fine"][0].__setitem__(0, 99.0),
        "bounding rectangle": lambda d: d["rules"][zid]["bbox"][2].__setitem__(0, 1e6),
        "no entry": lambda d: d["rules"].pop(zid),
        "no such zone": lambda d: d["rules"].__setitem__("ghost", {"fine": [[0, 0]]}),
        "describes": lambda d: d.__setitem__("generation", 7),
    }
    for word, edit in cases.items():
        try:
            check_bundle(forged(edit, "f.tar.gz"))
        except ValueError as e:
            assert word in str(e), (word, str(e)[:300])
        else:
            raise AssertionError(f"forged shipped geometry accepted: {word}")
    # An edit that does not recompute the digest, and a member the manifest
    # does not name.
    bad = json.dumps({**doc, "generation": 9}).encode()
    try:
        check_bundle(_repack(b, _tmp("e.tar.gz"), {B.MULTISCALE_NAME: bad}))
    except ValueError as e:
        assert "edited after issue" in str(e), e
    else:
        raise AssertionError("an edited multiscale.json was accepted")
    old = _members(b)
    man = json.loads(old[MANIFEST_NAME])
    del man["multiscale_sha256"]
    try:
        check_bundle(_repack(b, _tmp("u.tar.gz"), {MANIFEST_NAME: json.dumps(
            man, indent=2, sort_keys=True).encode()}))
    except ValueError as e:
        assert "unsigned geometry" in str(e), e
    else:
        raise AssertionError("geometry the manifest does not cover was accepted")


def test_ir_json_is_exactly_the_bytes_the_hash_digests():
    """WP3-21: hash the stored IR bytes, not a re-dump. A reader can now check
    a bundle with sha256sum alone, and a schema change in this code can never
    make an old bundle's own bytes disagree with its own manifest."""
    pol = load_policy(DEMO)
    m = _members(_unsigned(pol))
    digest = "sha256:" + hashlib.sha256(m[IR_NAME]).hexdigest()
    man = json.loads(m[MANIFEST_NAME])
    assert digest == man["policy_hash"] == pol.policy_hash, (digest, man)
    assert len(pol.policy_hash) == len("sha256:") + 64, pol.policy_hash


def test_the_same_policy_produces_byte_identical_bundles():
    """Member mtimes are pinned to 0 so a bundle is reproducible.

    Without that, two builds of an unchanged policy differ, and a hash of the
    ARCHIVE stops being a statement about the policy. Since issued_at entered
    the manifest, "the same policy" means the same policy at the same issue
    time; ir.json never depends on it (next test).
    """
    pol = load_policy(DEMO)
    a = write_bundle(pol, _tmp("a.tar.gz"), signer=None, issued_at=ISSUED).read_bytes()
    b = write_bundle(pol, _tmp("b.tar.gz"), signer=None, issued_at=ISSUED).read_bytes()
    assert a == b


def test_a_later_issue_changes_the_manifest_but_never_the_policy_hash():
    pol = load_policy(DEMO)
    a = _members(write_bundle(pol, _tmp(), signer=None, issued_at="2026-10-06T00:00:00Z"))
    b = _members(write_bundle(pol, _tmp(), signer=None, issued_at="2026-11-30T12:00:00Z"))
    assert a[IR_NAME] == b[IR_NAME]
    ma, mb = json.loads(a[MANIFEST_NAME]), json.loads(b[MANIFEST_NAME])
    assert ma["policy_hash"] == mb["policy_hash"]
    assert (ma["issued_at"], mb["issued_at"]) == ("2026-10-06T00:00:00Z",
                                                  "2026-11-30T12:00:00Z")


def test_the_manifest_records_what_a_reviewer_needs():
    p = write_bundle(load_policy(DEMO), _tmp(), changelog="added school curfew",
                     signer=None, issued_at=ISSUED)
    man = json.loads(_members(p)[MANIFEST_NAME])
    for f in ("policy_id", "version", "generation", "policy_hash",
              "signed_by", "changelog", "issued_at", "hash_scheme",
              "ir_schema_version"):
        assert f in man, f"manifest missing {f}"
    assert man["changelog"] == "added school curfew"
    assert man["issued_at"] == ISSUED
    assert man["signed_by"] == B.UNSIGNED, (
        "an unsigned bundle must say so in the manifest, not carry a "
        "placeholder that reads like a signer")


# --------------------------------------------------------------------------- #
# bundle: refusal (integrity - no key involved)
# --------------------------------------------------------------------------- #

def test_a_tampered_IR_is_refused():
    """The test that gives the round-trip its meaning.

    Widen the no-fly zone's altitude ceiling in the shipped IR and leave the
    manifest alone. The bundle still opens, still parses, still describes a
    valid policy - and must be refused, because it is not the policy that was
    issued.
    """
    src = _unsigned(load_policy(DEMO), "ok.tar.gz")
    ir = json.loads(_members(src)[IR_NAME])
    ir["constraints"][0]["altitude_ceiling_m"] = 9999.0
    bad = _repack(src, _tmp("bad.tar.gz"),
                  {IR_NAME: json.dumps(ir, sort_keys=True,
                                       separators=(",", ":")).encode()})
    try:
        load_bundle(bad, require_signature=False)
    except ValueError as e:
        assert "policy_hash mismatch" in str(e), e
        return
    raise AssertionError("a tampered IR was accepted")


def test_a_manifest_from_another_bundle_is_refused():
    """Swap in a manifest whose hash belongs to a different policy.

    Caught by the signature line, which binds a hash to the identity that signed
    it - so the manifest and the signature must agree as well as the IR.
    """
    a = _unsigned(load_policy(DEMO), "a.tar.gz")
    b = _unsigned(load_policy(PED), "b.tar.gz")
    bad = _repack(a, _tmp("mixed.tar.gz"),
                  {MANIFEST_NAME: _members(b)[MANIFEST_NAME]})
    try:
        load_bundle(bad, require_signature=False)
    except ValueError:
        return
    raise AssertionError("a foreign manifest was accepted")


def test_a_truncated_bundle_is_refused():
    src = _unsigned(load_policy(DEMO), "ok.tar.gz")
    dst = _tmp("short.tar.gz")
    with tarfile.open(src, "r:gz") as tin, tarfile.open(dst, "w:gz") as tout:
        for m in tin.getmembers():
            if m.name == MANIFEST_NAME:
                continue
            data = tin.extractfile(m).read()
            info = tarfile.TarInfo(name=m.name)
            info.size, info.mtime = len(data), 0
            tout.addfile(info, io.BytesIO(data))
    try:
        load_bundle(dst, require_signature=False)
    except ValueError as e:
        assert "missing required member" in str(e), e
        return
    raise AssertionError("a bundle missing its manifest was accepted")


def test_an_IR_with_an_unknown_key_is_refused_even_when_its_hash_matches():
    """Pydantic ignores unknown keys. An IR carrying one hashes correctly - the
    bytes are the bytes - and then rebuilds into a DIFFERENT policy than the
    one those bytes describe. Before 2026-10-06 the loader only compared the
    rebuilt object's hash, so this case could not even be expressed; now that
    the bytes are hashed directly it must be refused explicitly."""
    pol = load_policy(DEMO)
    src = _unsigned(pol, "ok.tar.gz")
    m = _members(src)
    ir = json.loads(m[IR_NAME])
    ir["constraints"][0]["altitude_cieling_m"] = 40.0          # sic
    ir_bytes = json.dumps(ir, sort_keys=True, separators=(",", ":")).encode()
    h = "sha256:" + hashlib.sha256(ir_bytes).hexdigest()
    man = json.loads(m[MANIFEST_NAME])
    man["policy_hash"] = h
    bad = _repack(src, _tmp("extra.tar.gz"), {
        IR_NAME: ir_bytes,
        MANIFEST_NAME: json.dumps(man, indent=2, sort_keys=True).encode(),
        SIGNATURE_NAME: f"{h} {B.UNSIGNED}\n".encode()})
    try:
        check_bundle(bad)
    except ValueError as e:
        assert "cannot represent" in str(e), e
        return
    raise AssertionError("an IR the model silently truncates was accepted")


def test_a_re_serialised_IR_is_refused():
    """Same policy, prettier bytes: the artefact is no longer what was hashed."""
    src = _unsigned(load_policy(DEMO), "ok.tar.gz")
    ir = json.loads(_members(src)[IR_NAME])
    bad = _repack(src, _tmp("pretty.tar.gz"),
                  {IR_NAME: json.dumps(ir, indent=2, sort_keys=True).encode()})
    try:
        check_bundle(bad)
    except ValueError as e:
        assert "canonical encoding" in str(e), e
        return
    raise AssertionError("a re-serialised ir.json was accepted")


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
        id="nfz-dynamic", type="polygon_fence", **HARD_P0,
        vertices=[XY(x=-25, y=-25), XY(x=-15, y=-25),
                  XY(x=-15, y=-15), XY(x=-25, y=-15)], margin_m=1.0))

    assert pol.policy_hash != before_hash, "the hash must restamp"
    assert pol.generation == 1, pol.generation

    got = load_bundle(_unsigned(pol, changelog="dynamic NFZ at t+8s"),
                      require_signature=False)
    assert len(got.constraints) == before_n + 1
    assert any(c.id == "nfz-dynamic" for c in got.constraints)
    assert got.generation == 1, "the generation counter did not survive"


# --------------------------------------------------------------------------- #
# bundle: the signature (2026-10-06)
# --------------------------------------------------------------------------- #

def test_an_unsigned_bundle_is_refused_by_default_and_says_why():
    """No signature is a fact to record, not a pass. load_bundle refuses it
    unless the caller explicitly accepts an unsigned policy."""
    p = _unsigned(load_policy(DEMO))
    assert check_bundle(p).signature == B.SIG_UNSIGNED
    try:
        load_bundle(p)
    except BundleSignatureError as e:
        assert "unsigned" in str(e), e
    else:
        raise AssertionError("an unsigned bundle loaded as if signed")
    assert load_bundle(p, require_signature=False).policy_id == "fase3-sim-demo"


def test_a_signed_bundle_verifies_against_its_trust_store():
    signer, trust = _test_key()
    pol = load_policy(DEMO)
    p = write_bundle(pol, _tmp(), signer=signer, issued_at=ISSUED)
    chk = check_bundle(p, trust)
    assert chk.signature == B.SIG_VERIFIED, chk.detail
    assert chk.signed_by == signer.identity and signer.identity.startswith("test-lab:ed25519:")
    assert load_bundle(p, trust_store=trust).policy_hash == pol.policy_hash
    sig = _members(p)[SIGNATURE_NAME].decode().splitlines()
    assert sig[0] == f"{pol.policy_hash} {signer.identity}", sig
    assert sig[1].startswith("ed25519:") and len(sig[1]) == len("ed25519:") + 128


def test_signing_is_deterministic_so_signed_bundles_stay_reproducible():
    signer, _ = _test_key()
    pol = load_policy(DEMO)
    a = write_bundle(pol, _tmp("a.tar.gz"), signer=signer, issued_at=ISSUED)
    b = write_bundle(pol, _tmp("b.tar.gz"), signer=signer, issued_at=ISSUED)
    assert a.read_bytes() == b.read_bytes()


def test_an_edited_manifest_breaks_the_signature():
    """The changelog is not covered by the policy hash - only by the signature."""
    signer, trust = _test_key()
    p = write_bundle(load_policy(DEMO), _tmp(), signer=signer, issued_at=ISSUED)
    man = json.loads(_members(p)[MANIFEST_NAME])
    man["changelog"] = "approved by ITRI"
    bad = _repack(p, _tmp("edited.tar.gz"),
                  {MANIFEST_NAME: json.dumps(man, indent=2, sort_keys=True).encode()})
    try:
        check_bundle(bad, trust)
    except ValueError as e:
        assert "Ed25519 verification failed" in str(e), e
        return
    raise AssertionError("an edited manifest kept a valid signature")


def test_a_consistent_forgery_without_the_key_is_refused():
    """What the placeholder could never catch: edit the IR, recompute the hash,
    rewrite the manifest and the signature's first line consistently. Without
    the private key the Ed25519 line cannot follow."""
    signer, trust = _test_key()
    p = write_bundle(load_policy(DEMO), _tmp(), signer=signer, issued_at=ISSUED)
    m = _members(p)
    ir = json.loads(m[IR_NAME])
    ir["constraints"][0]["altitude_ceiling_m"] = 5.0
    ir_b = json.dumps(ir, sort_keys=True, separators=(",", ":")).encode()
    h = "sha256:" + hashlib.sha256(ir_b).hexdigest()
    man = json.loads(m[MANIFEST_NAME])
    man["policy_hash"] = h
    # Consistent means every record of the hash: since 2026-10-08 the
    # manifest also names it in `hashes` (and check_bundle verifies that).
    man["hashes"]["sha256-canonical-v3"] = h
    # ...and since 2026-11-02 the shipped multi-scale geometry names it too,
    # its digest in the manifest.
    ms = json.loads(m[B.MULTISCALE_NAME])
    ms["policy_hash"] = h
    ms_b = json.dumps(ms, sort_keys=True, separators=(",", ":")).encode()
    man["multiscale_sha256"] = "sha256:" + hashlib.sha256(ms_b).hexdigest()
    old_sig = m[SIGNATURE_NAME].decode().splitlines()[1]
    bad = _repack(p, _tmp("forged.tar.gz"), {
        IR_NAME: ir_b, B.MULTISCALE_NAME: ms_b,
        MANIFEST_NAME: json.dumps(man, indent=2, sort_keys=True).encode(),
        SIGNATURE_NAME: f"{h} {signer.identity}\n{old_sig}\n".encode()})
    try:
        check_bundle(bad, trust)
    except ValueError as e:
        assert "Ed25519" in str(e), e
        return
    raise AssertionError("a consistent forgery passed a keyed signature")


def test_a_key_the_trust_store_does_not_list_is_refused():
    signer, _ = _test_key()
    _, other_trust = _test_key()
    p = write_bundle(load_policy(DEMO), _tmp(), signer=signer, issued_at=ISSUED)
    assert check_bundle(p, other_trust).signature == B.SIG_UNTRUSTED
    try:
        load_bundle(p, trust_store=other_trust)
    except BundleSignatureError as e:
        assert "untrusted" in str(e), e
        return
    raise AssertionError("a bundle from an unlisted key loaded as trusted")


# RFC 8032 section 7.1, TEST 1-3: (secret seed, public key, message, signature).
# Cross-checked against `cryptography` when these lines were written.
RFC8032 = [
    ("9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60",
     "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a", "",
     "e5564300c360ac729086e2cc806e828a84877f1eb8e5d974d873e06522490155"
     "5fb8821590a33bacc61e39701cf9b46bd25bf5f0595bbe24655141438e7a100b"),
    ("4ccd089b28ff96da9db6c346ec114e0f5b8a319f35aba624da8cf6ed4fb8a6fb",
     "3d4017c3e843895a92b70aa74d1b7ebc9c982ccf2ec4968cc0cd55f12af4660c", "72",
     "92a009a9f0d4cab8720e820b5f642540a2b27b5416503f8fb3762223ebdb69da"
     "085ac1e43e15996e458f3613d0f11d8c387b2eaeb4302aeeb00d291612bb0c00"),
    ("c5aa8df43f9f837bedb7442f31dcb7b166d38535076f094b85ce3a2e0b4458f7",
     "fc51cd8e6218a1a38da47ed00230f0580816ed13ba3303ac5deb911548908025", "af82",
     "6291d657deec24024827e69c3abe01a30ce548a284743a445e3680d7db5ac3ac"
     "18ff9b538d16f290ae67f760984dc6594a7c15e9716ed28dc027beceea1ec40a"),
]


def test_the_pure_python_verifier_passes_the_rfc_8032_vectors():
    """The fallback the SITL rails depend on, pinned to the standard's own
    numbers on every interpreter - and made to say no: a changed message, a
    flipped bit in the signature, another key, and an S >= L must all be
    refused."""
    for i, (seed, pub, msg, sig) in enumerate(RFC8032):
        pub_b, msg_b, sig_b = bytes.fromhex(pub), bytes.fromhex(msg), bytes.fromhex(sig)
        assert B.py_ed25519_verify(pub_b, msg_b, sig_b), pub
        key = B._PyEd25519TestKey(bytes.fromhex(seed))
        assert key.public_raw == pub_b and key.sign(msg_b) == sig_b, pub
        assert not B.py_ed25519_verify(pub_b, msg_b + b"\x00", sig_b)
        flipped = bytearray(sig_b)
        flipped[5] ^= 1
        assert not B.py_ed25519_verify(pub_b, msg_b, bytes(flipped))
        other = bytes.fromhex(RFC8032[(i + 1) % len(RFC8032)][1])
        assert not B.py_ed25519_verify(other, msg_b, sig_b)
        s = int.from_bytes(sig_b[32:], "little") + B._Q        # same S mod L
        assert not B.py_ed25519_verify(pub_b, msg_b, sig_b[:32] + s.to_bytes(32, "little"))
    assert not B.py_ed25519_verify(b"\x00" * 31, b"", b"\x00" * 64)


def test_the_pure_python_verifier_agrees_with_cryptography():
    """Where both exist, they must give the same answer, both ways round (each
    verifies the other's signatures, each refuses the same tampering)."""
    if not B.HAVE_CRYPTO:
        return SKIP
    from cryptography.hazmat.primitives import serialization as ser
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
    for i in range(12):
        data = os.urandom(40 + i * 17)
        k = Ed25519PrivateKey.generate()
        pub = k.public_key().public_bytes(ser.Encoding.Raw, ser.PublicFormat.Raw)
        sig = k.sign(data)
        assert B.py_ed25519_verify(pub, data, sig)
        assert B._ed25519_ok("cryptography", pub, sig, data)
        bad = data[:-1] + bytes([data[-1] ^ 0x80])
        assert not B.py_ed25519_verify(pub, bad, sig)
        assert not B._ed25519_ok("cryptography", pub, sig, bad)
        py = B._PyEd25519TestKey(os.urandom(32))
        assert B._ed25519_ok("cryptography", py.public_raw, py.sign(data), data)


def test_without_cryptography_a_signature_is_still_checked():
    """The 2026-10-06 review: the SITL rails (WSL venvs without `cryptography`)
    could never verify, so every --bundle there was refused or flown as
    "unverifiable-here" - and a forged signature flew under the flag reading
    the same as a real one. With the fallback the same bundle verifies there,
    and an edited one is refused there."""
    signer, trust = _test_key()
    p = write_bundle(load_policy(DEMO), _tmp(), signer=signer, issued_at=ISSUED)
    man = json.loads(_members(p)[MANIFEST_NAME])
    man["changelog"] = "approved by ITRI"
    edited = _repack(p, _tmp("edited.tar.gz"),
                     {MANIFEST_NAME: json.dumps(man, indent=2, sort_keys=True).encode()})
    with _no_crypto():
        assert B.verifier_name() == "pure-python"
        chk = check_bundle(p, trust)
        assert chk.signature == B.SIG_VERIFIED and "pure-python" in chk.detail, chk.detail
        _, src = B.load_for_flight(p, trust_store=trust)
        assert src["signature"] == B.SIG_VERIFIED, src
        try:
            check_bundle(edited, trust)
        except ValueError as e:
            assert "Ed25519 verification failed" in str(e), e
        else:
            raise AssertionError("an edited manifest verified under the fallback")


def test_with_no_verifier_at_all_a_signature_is_UNVERIFIABLE_not_passed():
    """The failure this project keeps finding: a check that cannot run must not
    look like a check that passed. Reachable now only with the fallback off."""
    signer, trust = _test_key()
    p = write_bundle(load_policy(DEMO), _tmp(), signer=signer, issued_at=ISSUED)
    with _no_crypto(fallback=False):
        assert B.verifier_name() is None
        chk = check_bundle(p, trust)
        assert chk.signature == B.SIG_UNVERIFIABLE, chk.signature
        assert "NOT checked" in chk.detail, chk.detail
        for load in (lambda: load_bundle(p, trust_store=trust),
                     lambda: B.load_for_flight(p, trust_store=trust)):
            try:
                load()
            except BundleSignatureError as e:
                assert "unverifiable" in str(e), e
            else:
                raise AssertionError("an unverifiable signature was treated as verified")


def test_an_unknown_signer_is_named_before_any_cryptography_runs():
    """The trust lookup needs no crypto, so it must not hide behind a missing
    library: an unlisted key is "untrusted-signer" even with no verifier."""
    signer, _ = _test_key()
    _, other_trust = _test_key()
    p = write_bundle(load_policy(DEMO), _tmp(), signer=signer, issued_at=ISSUED)
    with _no_crypto(fallback=False):
        assert check_bundle(p, other_trust).signature == B.SIG_UNTRUSTED


def test_a_stripped_signature_that_keeps_its_signer_is_refused():
    """The review's forgery, verbatim: edit the IR (fence ceiling 5 m), redo
    the hash, keep the TRUSTED signer's identity, drop only the Ed25519 line.
    It used to read "unsigned" - with the trusted name attached and the false
    detail "names no key" - and flew under --allow-unverified-bundle. No tool
    writes that layout, so it is refused outright, flag or no flag.

    Shown failing on the pre-fix code: check_bundle returned signature
    'unsigned' with the trusted identity, and load_for_flight flew it."""
    signer, trust = _test_key()
    good = write_bundle(load_policy(DEMO), _tmp(), signer=signer, issued_at=ISSUED)
    for forge_ir in (True, False):
        bad = _strip_signature(good, _tmp("stripped.tar.gz"), forge_ir=forge_ir)
        for load in (lambda: check_bundle(bad, trust),
                     lambda: B.load_for_flight(bad, allow_unverified=True,
                                               trust_store=trust)):
            try:
                load()
            except BundleSignatureError:
                raise AssertionError("refused as merely unverified; it is forged")
            except ValueError as e:
                assert "stripped" in str(e), e
            else:
                raise AssertionError("a stripped signature was accepted")


def test_a_signature_under_a_keyless_identity_is_refused():
    """The mirror image of a strip: an Ed25519 line under `unsigned:...` or the
    placeholder. No writer produces it, and reading it as "unsigned" would let
    a bundle carry a signature nobody checks."""
    pol = load_policy(DEMO)
    for who in (B.UNSIGNED, B.LEGACY_PLACEHOLDER_SIGNER):
        src = _unsigned(pol)
        man = json.loads(_members(src)[MANIFEST_NAME])
        man["signed_by"] = who
        bad = _repack(src, _tmp("keyless_sig.tar.gz"), {
            MANIFEST_NAME: json.dumps(man, indent=2, sort_keys=True).encode(),
            SIGNATURE_NAME: f"{pol.policy_hash} {who}\ned25519:{'ab' * 64}\n".encode()})
        try:
            check_bundle(bad)
        except ValueError as e:
            assert "names no key" in str(e), e
        else:
            raise AssertionError(f"a signature under {who!r} was accepted")


def test_a_signature_line_naming_another_hash_is_refused():
    """signature.txt line 1 binds a hash to a signer. One naming any hash other
    than the manifest's belongs to another bundle."""
    src = _unsigned(load_policy(DEMO))
    bad = _repack(src, _tmp("otherhash.tar.gz"), {
        SIGNATURE_NAME: f"sha256:{'0' * 64} {B.UNSIGNED}\n".encode()})
    try:
        check_bundle(bad)
    except ValueError as e:
        assert "signature covers" in str(e), e
    else:
        raise AssertionError("a signature line for another hash was accepted")


def test_a_manifest_naming_another_signer_than_the_signature_is_refused():
    """The manifest's signed_by is covered by the signature only when there IS
    one; without one it must still agree with the signature line, or an
    unsigned bundle could carry the lab key's name in its manifest."""
    src = _unsigned(load_policy(DEMO))
    man = json.loads(_members(src)[MANIFEST_NAME])
    man["signed_by"] = "lab-dev:ed25519:6ec75b535ef5c533"
    bad = _repack(src, _tmp("whosigned.tar.gz"), {
        MANIFEST_NAME: json.dumps(man, indent=2, sort_keys=True).encode()})
    try:
        check_bundle(bad)
    except ValueError as e:
        assert "manifest says signed_by" in str(e), e
    else:
        raise AssertionError("a manifest naming another signer was accepted")


# --------------------------------------------------------------------------- #
# bundle: the signer under a certificate authority (WP1-04 / X-10, 2026-11-16)
# --------------------------------------------------------------------------- #
#
# Every CA below is made by the test in a temp folder
# (guardrail.harness_service.generate_pki, the function that made the lab's
# root). The lab's own CA key and trust-store entries are never read or
# written here: test_the_issue_cert_command_... checks the repository's trust
# store is byte for byte what it was.

def _ca(ca_cn: str | None = None):
    """A throw-away root CA: (its folder, its certificate as PEM text)."""
    import guardrail.harness_service as H
    d = Path(tempfile.mkdtemp()) / "pki"
    H.generate_pki(d, **({"ca_cn": ca_cn} if ca_cn else {}))
    return d, (d / "ca.cert.pem").read_text(encoding="ascii")


def _certified(authority: str = "lab-ca", **kw):
    """A throw-away key certified by a throw-away CA and pinned in a throw-away
    trust store: (signer, trust store, CA folder, CA PEM, the bare signer the
    key was before)."""
    bare, trust = _test_key()
    pki, ca_pem = _ca()
    rows = B.issue_signer_certificates(pki, trust, authority=authority, **kw)
    assert [r["issued"] for r in rows] == [True], rows
    signer = B.load_signer(trust.parent / "k.pem", trust)
    assert signer.identity == rows[0]["identity"], (signer.identity, rows)
    return signer, trust, pki, ca_pem, bare


def _x509(pem):
    from cryptography import x509
    return x509.load_pem_x509_certificate(pem.encode() if isinstance(pem, str) else pem)


def _der_sha256(pem) -> str:
    """SHA-256 of a certificate's DER by `cryptography`, not by the code under
    test."""
    from cryptography.hazmat.primitives import hashes
    return _x509(pem).fingerprint(hashes.SHA256()).hex()


def _store(trust: Path) -> dict:
    return json.loads(trust.read_text(encoding="utf-8"))


def _refused_as(load, status: str, *words: str):
    """`load` must raise BundleSignatureError naming `status` and `words`."""
    try:
        load()
    except BundleSignatureError as e:
        assert status in str(e), e
        for w in words:
            assert w in str(e), (w, str(e))
    else:
        raise AssertionError(f"accepted; expected a refusal as {status}")


def _corrupt(load, *words: str):
    """`load` must raise ValueError (integrity, not trust) naming `words`."""
    try:
        load()
    except BundleSignatureError as e:
        raise AssertionError(f"refused as merely untrusted: {e}")
    except ValueError as e:
        for w in words:
            assert w in str(e), (w, str(e))
    else:
        raise AssertionError("accepted; expected an integrity refusal")


def test_a_key_under_a_ca_signs_as_a_ca_reference_and_its_chain_verifies():
    """WP1-04: the bundle key is certified by a root CA, so `signed_by` names
    the CA, the bundle ships the certificate, and the reader chains it."""
    if not B.HAVE_CRYPTO:
        return SKIP
    from cryptography.x509.oid import ExtendedKeyUsageOID as EKU
    bare, trust = _test_key()
    pol = load_policy(DEMO)
    before = check_bundle(write_bundle(pol, _tmp(), signer=bare, issued_at=ISSUED), trust)
    assert (before.signature, before.chain) == (B.SIG_VERIFIED, B.CHAIN_NONE), before
    assert before.source_record()["chain"] == "none"

    pki, ca_pem = _ca()
    rows = B.issue_signer_certificates(pki, trust)
    fp = _der_sha256(ca_pem)
    ident = f"lab-ca:{fp[:16]}:{bare.key_id}"
    assert [(r["identity"], r["issued"], r["retired"]) for r in rows] == \
        [(ident, True, [bare.identity])], rows
    store = _store(trust)
    assert store["authorities"]["lab-ca"]["sha256"] == fp
    assert "`authorities`" in store["_doc"], "the store's own description is stale"
    assert store["signers"][ident]["issuer"] == "lab-ca"
    assert store["signers"][ident]["public_key_hex"] == bare.public_raw.hex(), \
        "the key stays pinned, so the Ed25519 check needs no certificate parser"
    assert "PRIVATE KEY" not in trust.read_text(encoding="utf-8")

    signer = B.load_signer(trust.parent / "k.pem", trust)
    assert signer.identity == ident and signer.identity.endswith(":" + signer.key_id)
    assert ident in B.load_trust_store(trust), "signed_by is still a trust-store key"
    p = write_bundle(pol, _tmp(), signer=signer, issued_at=ISSUED)
    assert write_bundle(pol, _tmp("again.tar.gz"), signer=signer,
                        issued_at=ISSUED).read_bytes() == p.read_bytes()
    m = _members(p)
    assert set(m) == {IR_NAME, MANIFEST_NAME, SIGNATURE_NAME, B.MULTISCALE_NAME,
                      B.SIGNER_CERT_NAME}, sorted(m)
    assert m[SIGNATURE_NAME].decode().splitlines()[0] == f"{pol.policy_hash} {ident}"

    chk = check_bundle(p, trust)
    assert (chk.signature, chk.chain, chk.authority) == \
        (B.SIG_VERIFIED, B.CHAIN_VERIFIED, "lab-ca"), (chk.signature, chk.detail)
    assert "chains to authority 'lab-ca'" in chk.detail, chk.detail
    assert load_bundle(p, trust_store=trust).policy_hash == pol.policy_hash
    _, src = B.load_for_flight(p, trust_store=trust, require_chain=True)
    assert (src["signature"], src["chain"], src["signed_by"]) == \
        ("verified", "verified", ident), src

    # The leaf: this key, code signing only, issued by this CA, to its notAfter.
    leaf, ca = _x509(m[B.SIGNER_CERT_NAME]), _x509(ca_pem)
    leaf.verify_directly_issued_by(ca)
    assert B._leaf_public_raw(leaf) == bare.public_raw
    from cryptography import x509
    assert list(leaf.extensions.get_extension_for_class(x509.ExtendedKeyUsage).value) \
        == [EKU.CODE_SIGNING]
    assert not leaf.extensions.get_extension_for_class(x509.BasicConstraints).value.ca
    assert B._cert_window(leaf)[1] == B._cert_window(ca)[1], \
        "the leaf must live exactly as long as its CA, not 365 days"
    assert leaf.subject.rfc4514_string().startswith(
        "CN=policy-signer-" + bare.key_id.split(":")[1])

    # Certifying again changes nothing: no second leaf, the same file.
    was = trust.read_bytes()
    again = B.issue_signer_certificates(pki, trust)
    assert [r["issued"] for r in again] == [False] and trust.read_bytes() == was, again


def test_the_manifest_carries_the_five_grant_fields_with_a_ca_reference():
    """X-10. The grant's Policy DSL page: `policy_hash` "SHA-256 over the
    canonicalised IR (post-merge, pre-sign)", `signed_by` "Lab CA or ITRI CA
    reference", `changelog` "Human-readable diff vs the previous version", and
    its worked example's `issued_at` and `layers_merged`. All five, on one
    signed bundle of a merged policy; `signed_by` was the one that failed - a
    key label, `lab-dev:ed25519:<id>`, is not a reference to a CA."""
    if not B.HAVE_CRYPTO:
        return SKIP
    from guardrail.models import load_layered
    rule = ("{{id: alt, type: altitude_envelope, layer: {layer}, constraint_type: hard, "
            "scope: mission, priority: P0, violation_action: project_fix, "
            "alt_min_m: {lo}, alt_max_m: {hi}}}")

    def layer(name, text):
        f = Path(tempfile.mkdtemp()) / name
        f.write_text(text, encoding="utf-8")
        return f

    reg = layer("reg.yaml", "policy_id: reg\nversion: 1.0.0\nconstraints:\n  - "
                + rule.format(layer="regulation", lo=5, hi=120) + "\n")
    old = load_layered([reg, layer("m1.yaml", "policy_id: m\nversion: 0.1.0\nconstraints:\n  - "
                                   + rule.format(layer="mission", lo=10, hi=40) + "\n")])
    new = load_layered([reg, layer("m2.yaml", "policy_id: m\nversion: 0.2.0\nconstraints:\n  - "
                                   + rule.format(layer="mission", lo=10, hi=50) + "\n")])
    signer, trust, _, ca_pem, _ = _certified()
    p = write_bundle(new, _tmp(), signer=signer, issued_at=ISSUED, previous=old)
    m = _members(p)
    man, ir = json.loads(m[MANIFEST_NAME]), json.loads(m[IR_NAME])

    # 1. policy_hash: the full SHA-256 of the canonical IR, post-merge.
    assert man["policy_hash"] == "sha256:" + hashlib.sha256(m[IR_NAME]).hexdigest()
    assert man["policy_hash"] == new.policy_hash and len(man["policy_hash"]) == 7 + 64
    # 2. signed_by: a reference to the CA - its name and the fingerprint of
    #    its certificate - that still ends in the key id; the record beside it
    #    gives the fingerprint in full and the certificate it issued.
    fp = _der_sha256(ca_pem)
    assert man["signed_by"] == f"lab-ca:{fp[:16]}:{signer.key_id}", man["signed_by"]
    assert not man["signed_by"].startswith("lab-dev:")
    leaf = _x509(m[B.SIGNER_CERT_NAME])
    assert man["signer"] == {
        "authority": "lab-ca", "ca_fingerprint": "sha256:" + fp,
        "leaf_serial": format(leaf.serial_number, "x"),
        "leaf_subject": leaf.subject.rfc4514_string(),
        "key_id": signer.key_id,
        "certificate_sha256": "sha256:" + _der_sha256(m[B.SIGNER_CERT_NAME])}, man["signer"]
    # 3. changelog: the diff against the previous version, readable and exact.
    assert man["changelog"].startswith(f"against m 0.1.0 ({old.policy_hash[:12]})"), man
    assert "changed alt (alt_max_m)" in man["changelog"], man["changelog"]
    assert man["changelog_diff"]["against"]["policy_hash"] == old.policy_hash
    assert man["changelog_diff"]["changed"] == {"alt": ["alt_max_m"]}
    # 4. issued_at, 5. layers_merged (in the policy the hash is over).
    assert man["issued_at"] == ISSUED
    assert ir["layers_merged"] == ["regulation", "mission"] == list(new.layers_merged), ir
    # ...and all of it is under the signature, checked through the chain.
    chk = check_bundle(p, trust)
    assert (chk.signature, chk.chain) == (B.SIG_VERIFIED, B.CHAIN_VERIFIED), chk.detail
    for field, value in (("changelog", "approved by ITRI"), ("issued_at", "2026-01-01T00:00:00Z")):
        edited = dict(man, **{field: value})
        _corrupt(lambda: check_bundle(_repack(p, _tmp("e.tar.gz"), {
            MANIFEST_NAME: json.dumps(edited, indent=2, sort_keys=True).encode()}), trust),
            "Ed25519 verification failed")
    # A bare key's manifest is what it was: no `signer`, no certificate.
    bare, _ = _test_key()
    mb = _members(write_bundle(new, _tmp(), signer=bare, issued_at=ISSUED))
    assert "signer" not in json.loads(mb[MANIFEST_NAME]) and B.SIGNER_CERT_NAME not in mb


def test_swapping_the_authority_is_one_registry_entry_and_a_re_sign():
    """The swap drill promised on 2 Nov and never written: the claim "another
    CA is one entry and a re-sign" was untested. A second CA is made here; the
    ONLY change to the trust store is its `authorities` entry, the same key
    re-signs with the leaf that CA issued, and no line of guardrail/ changes.
    Then the first authority is removed, and its bundle is refused."""
    if not B.HAVE_CRYPTO:
        return SKIP
    import guardrail.harness_service as H
    signer, trust, _, ca1_pem, _ = _certified()
    pol = load_policy(DEMO)
    b1 = write_bundle(pol, _tmp("ca1.tar.gz"), signer=signer, issued_at=ISSUED)
    assert check_bundle(b1, trust).chain == B.CHAIN_VERIFIED

    pki2, ca2_pem = _ca("ITRI Test Root CA")
    before = _store(trust)
    ref2 = B.register_authority("itri-ca", ca2_pem, trust)          # the one entry
    after = _store(trust)
    assert ref2 == "itri-ca:" + _der_sha256(ca2_pem)[:16]
    assert after["signers"] == before["signers"], "the swap touched the signers"
    assert set(after["authorities"]) == set(before["authorities"]) | {"itri-ca"}
    assert after["authorities"]["lab-ca"] == before["authorities"]["lab-ca"]

    leaf2 = H.issue_policy_signer(pki2, signer.public_raw)           # the re-sign
    signer2 = B.certified_signer(signer.private_key, leaf2, "itri-ca", ca2_pem)
    assert signer2.identity == f"{ref2}:{signer.key_id}" != signer.identity
    b2 = write_bundle(pol, _tmp("ca2.tar.gz"), signer=signer2, issued_at=ISSUED)
    chk2 = check_bundle(b2, trust)
    assert (chk2.signature, chk2.chain, chk2.authority) == \
        (B.SIG_VERIFIED, B.CHAIN_VERIFIED, "itri-ca"), chk2.detail
    assert signer2.identity not in B.load_trust_store(trust), \
        "the new signer is trusted through its CA, not through a pinned key"
    _, src = B.load_for_flight(b2, trust_store=trust, require_chain=True)
    assert src["signed_by"] == signer2.identity and src["chain"] == "verified"
    assert check_bundle(b1, trust).chain == B.CHAIN_VERIFIED, "both authorities stand"

    # The first authority goes: one entry removed, and what it certified is
    # refused - with or without `cryptography`, since no parser is needed to
    # see that an authority is not listed.
    doc = _store(trust)
    del doc["authorities"]["lab-ca"]
    trust.write_text(json.dumps(doc, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    for ctx in (contextlib.nullcontext(), _no_crypto()):
        with ctx:
            gone = check_bundle(b1, trust)
            assert (gone.signature, gone.chain) == (B.SIG_UNTRUSTED, B.CHAIN_NONE), gone
            assert "'lab-ca' is not listed" in gone.detail, gone.detail
            _refused_as(lambda: load_bundle(b1, trust_store=trust), "untrusted-signer")
            _refused_as(lambda: B.load_for_flight(b1, trust_store=trust), "untrusted-signer")
            # the same for a document that ships no certificate (a replay
            # bundle's index): the key is still pinned, its authority is gone
            idx = B.verify_signer(signer.identity, signer.sign(b"index"), b"index", trust)
            assert idx.status == B.SIG_UNTRUSTED and "is not listed" in idx.detail, idx
    assert check_bundle(b2, trust).chain == B.CHAIN_VERIFIED

    # Another CA may not take a listed name: its bundles name another
    # fingerprint, and the listed CA's would stop verifying without a word.
    try:
        B.register_authority("itri-ca", ca1_pem, trust)
    except ValueError as e:
        assert "another CA takes another name" in str(e), e
    else:
        raise AssertionError("a second CA was put under a listed authority's name")
    # ...and a bundle that names the listed authority with another CA's
    # fingerprint is not that authority's.
    stale = B.certified_signer(signer.private_key, signer.certificate_pem, "itri-ca", ca1_pem)
    worn = check_bundle(write_bundle(pol, _tmp(), signer=stale, issued_at=ISSUED), trust)
    assert worn.signature == B.SIG_UNTRUSTED and "not the CA" in worn.detail, worn.detail


def test_a_certificate_that_does_not_chain_to_a_listed_authority_is_refused():
    """What a CA adds over a pinned key is only worth having if a certificate
    it did not issue is refused. Each bundle below is consistent and carries a
    valid Ed25519 signature by the key it names; what is wrong is the
    certificate: forged (issued by a look-alike CA, or edited after issue),
    issued by a CA that is not listed, or issued for another key."""
    if not B.HAVE_CRYPTO:
        return SKIP
    import base64
    import guardrail.harness_service as H
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
    signer, trust, pki, ca_pem, _ = _certified()
    pol = load_policy(DEMO)

    def flown(sgn, name="x.tar.gz"):
        return write_bundle(pol, _tmp(name), signer=sgn, issued_at=ISSUED)

    def untrusted(sgn, *words):
        p = flown(sgn)
        chk = check_bundle(p, trust)
        assert (chk.signature, chk.chain) == (B.SIG_UNTRUSTED, B.CHAIN_NONE), \
            (chk.signature, chk.chain, chk.detail)
        for w in words:
            assert w in chk.detail, (w, chk.detail)
        _refused_as(lambda: load_bundle(p, trust_store=trust), "untrusted-signer")
        _refused_as(lambda: B.load_for_flight(p, trust_store=trust), "untrusted-signer")
        _refused_as(lambda: B.load_for_flight(p, trust_store=trust, allow_unverified=True,
                                              require_chain=True), "untrusted-signer")

    # 1. A forged leaf: an attacker's key, certified by an attacker's CA that
    #    copies the listed CA's name, presented under the listed CA's reference.
    rogue_pki, rogue_pem = _ca()
    assert _x509(rogue_pem).subject == _x509(ca_pem).subject, "same name, another key"
    thief = Ed25519PrivateKey.generate()
    thief_raw = B.Signer("x", thief).public_raw
    forged = H.issue_policy_signer(rogue_pki, thief_raw)
    untrusted(B.certified_signer(thief, forged, "lab-ca", ca_pem),
              "was not issued by this authority")
    assert B._issued_by_manual(_x509(signer.certificate_pem), _x509(ca_pem)) is True
    assert B._issued_by_manual(_x509(forged), _x509(ca_pem)) is False

    # 1b. A forged leaf: the genuine certificate with its subject edited (the
    #     CA's signature no longer covers it).
    der = B._pem_der(signer.certificate_pem)
    assert der.count(b"policy-signer-") == 1
    edited = ("-----BEGIN CERTIFICATE-----\n"
              + base64.encodebytes(der.replace(b"policy-signer-", b"policy-forger-")).decode()
              + "-----END CERTIFICATE-----\n")
    unpinned = _store(trust)
    pinned_entry = unpinned["signers"].pop(signer.identity)
    loose = trust.parent / "authority_only.json"       # the CA listed, no key pinned
    loose.write_text(json.dumps(unpinned), encoding="utf-8")
    p = flown(B.certified_signer(signer.private_key, edited, "lab-ca", ca_pem))
    got = check_bundle(p, loose)
    assert got.signature == B.SIG_UNTRUSTED and "was not issued by" in got.detail, got.detail
    #     ...and where the key IS pinned, a certificate that is not the pinned
    #     one is refused before any parsing (so without `cryptography` too).
    for ctx in (contextlib.nullcontext(), _no_crypto()):
        with ctx:
            got = check_bundle(p, trust)
            assert got.signature == B.SIG_UNTRUSTED and "is not the one the trust store" \
                in got.detail, got.detail
    #     The genuine certificate, unpinned, is accepted through the CA alone.
    ok = check_bundle(flown(signer), loose)
    assert (ok.signature, ok.chain) == (B.SIG_VERIFIED, B.CHAIN_VERIFIED), ok.detail

    # 2. A wrong-CA leaf: the same key, certified by a real CA that this
    #    trust store does not list - named honestly, and passed off as lab-ca.
    other_pki, other_pem = _ca("Somebody Else Root CA")
    foreign = H.issue_policy_signer(other_pki, signer.public_raw)
    for store in (trust, loose):
        got = check_bundle(flown(B.certified_signer(signer.private_key, foreign,
                                                    "other-ca", other_pem)), store)
        assert got.signature == B.SIG_UNTRUSTED and "'other-ca' is not listed" in got.detail, \
            got.detail
    got = check_bundle(flown(B.certified_signer(signer.private_key, foreign, "lab-ca", ca_pem)),
                       loose)
    assert got.signature == B.SIG_UNTRUSTED and "was not issued by" in got.detail, got.detail

    # 3. Leaf / key mismatch: another key signs and ships the genuine leaf of
    #    the certified key. Said honestly (the record names the signing key)...
    _corrupt(lambda: check_bundle(flown(B.certified_signer(thief, signer.certificate_pem,
                                                           "lab-ca", ca_pem)), trust),
             "does not describe the certificate", "key_id")
    #    ...or with the genuine record under the thief's identity...
    liar = B.Signer(authority=signer.authority, private_key=thief,
                    certificate_pem=signer.certificate_pem, record=signer.record)
    _corrupt(lambda: check_bundle(flown(liar), trust), "does not belong to this signer")
    #    ...or with everything naming the certified key, and the thief's
    #    signature under it.
    class _As:                                   # signs as `signer`, with another key
        public_raw = signer.public_raw
        sign = staticmethod(lambda data: thief.sign(data))
    imposter = B.Signer(authority=signer.authority, private_key=_As,
                        certificate_pem=signer.certificate_pem, record=signer.record)
    assert imposter.identity == signer.identity
    _corrupt(lambda: check_bundle(flown(imposter), trust), "Ed25519 verification failed")
    #    The verifier itself, below check_bundle's record check: a certificate
    #    for another key certifies nothing about this one.
    thief_ident = f"{signer.authority}:{B._key_id(thief_raw)}"
    res = B.verify_signer(thief_ident, thief.sign(b"doc").hex(), b"doc", loose,
                          certificate=signer.certificate_pem)
    assert res.status == B.SIG_UNTRUSTED and "not the key this identity names" in res.detail
    try:
        B.register_certificate(thief_raw, signer.certificate_pem, "lab-ca", trust)
    except ValueError as e:
        assert "not the key that signed" in str(e), e
    else:
        raise AssertionError("the store pinned a certificate issued for another key")

    # 4. A TLS identity from the same CA is not a bundle signer.
    why = B._chain_problem((pki / "harness-client.cert.pem").read_text(encoding="ascii"),
                           ca_pem, signer.public_raw)
    assert why and "code signing" in why, why
    assert B._chain_problem(signer.certificate_pem, ca_pem, signer.public_raw) is None
    #    ...and a leaf is not an authority.
    try:
        B.register_authority("leaf-ca", signer.certificate_pem, trust)
    except ValueError as e:
        assert "not a CA certificate" in str(e), e
    else:
        raise AssertionError("a leaf certificate was listed as an authority")

    # 5. Validity is judged when the bundle is verified: after the CA's (and
    #    so the leaf's) notAfter the chain no longer stands.
    good = flown(signer)
    assert check_bundle(good, trust).chain == B.CHAIN_VERIFIED
    real_now = B._utcnow
    B._utcnow = lambda: datetime(2040, 1, 1, tzinfo=real_now().tzinfo)
    try:
        late = check_bundle(good, trust)
    finally:
        B._utcnow = real_now
    assert late.signature == B.SIG_UNTRUSTED and "not valid now" in late.detail, late.detail

    # 6. The certificate is covered by the signature: dropped, swapped or
    #    added, the bundle is no longer the one that was signed.
    m = _members(good)
    stripped = _tmp("nocert.tar.gz")
    with tarfile.open(stripped, "w:gz") as tar:
        for name, data in m.items():
            if name != B.SIGNER_CERT_NAME:
                info = tarfile.TarInfo(name=name)
                info.size = len(data)
                tar.addfile(info, io.BytesIO(data))
    _corrupt(lambda: check_bundle(stripped, trust), "ships no signer.cert.pem")
    _corrupt(lambda: check_bundle(_repack(good, _tmp("swap.tar.gz"),
                                          {B.SIGNER_CERT_NAME: forged}), trust),
             "not the certificate that was signed for")
    _corrupt(lambda: check_bundle(_repack(good, _tmp("two.tar.gz"), {
        B.SIGNER_CERT_NAME: m[B.SIGNER_CERT_NAME] + forged}), trust),
        "not exactly one PEM certificate")
    bare, bare_trust = _test_key()
    _corrupt(lambda: check_bundle(_repack(flown(bare), _tmp("added.tar.gz"),
                                          {B.SIGNER_CERT_NAME: forged}), bare_trust),
             "a certificate nothing signed")
    assert pinned_entry["certificate_sha256"] == _der_sha256(signer.certificate_pem)


def test_a_retired_bare_key_is_accepted_only_for_what_was_issued_before():
    """Certifying a key retires its bare `lab-dev`-style identity, or the CA
    would add nothing: the old label would stay as good as the new one for
    ever. What was issued before the retirement still verifies - every stored
    bundle and the runs flown from them - and what says it was issued since
    is refused under the old label, on every interpreter."""
    if not B.HAVE_CRYPTO:
        return SKIP
    bare, trust = _test_key()
    pol = load_policy(DEMO)
    old = write_bundle(pol, _tmp("old.tar.gz"), signer=bare, issued_at=ISSUED)
    pki, _ = _ca()
    B.issue_signer_certificates(pki, trust, retired_on="2026-11-16T00:00:00Z")
    entry = _store(trust)["signers"][bare.identity]
    assert entry["retired_on"] == "2026-11-16T00:00:00Z" and \
        entry["superseded_by"].startswith("lab-ca:"), entry
    # The entry no longer says "put it under a CA": it says that happened,
    # and keeps what it said before.
    assert entry["note"].startswith("RETIRED 2026-11-16T00:00:00Z") and \
        entry["superseded_by"] in entry["note"], entry["note"]
    assert "issue-cert" in entry["note_before_retirement"], entry
    at = write_bundle(pol, _tmp("at.tar.gz"), signer=bare, issued_at="2026-11-16T00:00:00Z")
    late = write_bundle(pol, _tmp("late.tar.gz"), signer=bare, issued_at="2026-12-01T08:00:00Z")
    for ctx in (contextlib.nullcontext(), _no_crypto()):
        with ctx:
            chk = check_bundle(old, trust)
            assert (chk.signature, chk.chain) == (B.SIG_VERIFIED, B.CHAIN_NONE), chk.detail
            assert "before this identity was retired (2026-11-16T00:00:00Z)" in chk.detail
            assert load_bundle(old, trust_store=trust).policy_hash == pol.policy_hash
            for p in (at, late):
                got = check_bundle(p, trust)
                assert got.signature == B.SIG_UNTRUSTED and "retired on" in got.detail, got
                assert "signed as 'lab-ca:" in got.detail, got.detail
                _refused_as(lambda: load_bundle(p, trust_store=trust),
                            "untrusted-signer", "no longer accepted")
                _refused_as(lambda: B.load_for_flight(p, trust_store=trust), "untrusted-signer")
    # From now on the key signs under the CA: load_signer picks the identity
    # in force, not the retired one.
    now = B.load_signer(trust.parent / "k.pem", trust)
    assert now.identity == entry["superseded_by"] and now.record["authority"] == "lab-ca"
    assert check_bundle(write_bundle(pol, _tmp(), signer=now, issued_at="2026-12-01T08:00:00Z"),
                        trust).chain == B.CHAIN_VERIFIED
    # A document that states no issue time (a replay bundle's index): the
    # date cannot be applied, the signature still verifies, and the detail
    # says which of the two happened.
    sig = bare.sign(b"index")
    res = B.verify_signer(bare.identity, sig, b"index", trust)
    assert res.status == B.SIG_VERIFIED and "could not be applied" in res.detail, res
    res = B.verify_signer(now.identity, now.sign(b"index"), b"index", trust)
    assert (res.status, res.chain) == (B.SIG_VERIFIED, B.CHAIN_VERIFIED) and \
        "chains to authority 'lab-ca'" in res.detail, res
    # verify_signature is the pinned-key check alone, its call and its two
    # words as before: it knows nothing of authorities or retirement.
    assert B.verify_signature(bare.identity, sig, b"index", trust) == \
        (B.SIG_VERIFIED, f"Ed25519 signature by {bare.identity!r} verified (cryptography)")
    # An entry whose date cannot be read is refused, not waved through.
    doc = _store(trust)
    doc["signers"][bare.identity]["retired_on"] = "soon"
    assert B.verify_signer(bare.identity, sig, b"index", doc, issued_at=ISSUED).status \
        == B.SIG_UNTRUSTED
    # --keep-legacy: certified, and the bare identity left in force.
    kept, kept_trust = _test_key()
    B.issue_signer_certificates(pki, kept_trust, authority="lab-ca", retire_legacy=False)
    assert "retired_on" not in _store(kept_trust)["signers"][kept.identity]


def test_without_cryptography_the_chain_is_recorded_as_not_checked_never_as_verified():
    """No ASN.1 or P-256 is re-implemented for the trust path, so an
    interpreter without `cryptography` cannot chain a certificate. It must say
    so: the Ed25519 signature is still checked against the pinned key (pure
    Python), `chain` reads "not-checked-here" in the flight's record, a rail
    that requires the chain refuses, and a key that is NOT pinned - trusted
    through its certificate alone - is "unverifiable-here"."""
    if not B.HAVE_CRYPTO:
        return SKIP
    signer, trust, _, _, _ = _certified()
    pol = load_policy(DEMO)
    p = write_bundle(pol, _tmp(), signer=signer, issued_at=ISSUED)
    doc = _store(trust)
    del doc["signers"][signer.identity]
    loose = trust.parent / "authority_only.json"
    loose.write_text(json.dumps(doc), encoding="utf-8")
    bare, bare_trust = _test_key()
    plain = write_bundle(pol, _tmp(), signer=bare, issued_at=ISSUED)
    other = _certified()[0].certificate_pem.encode()     # another key's genuine leaf
    saved = os.environ.pop(B.REQUIRE_CHAIN_ENV, None)
    try:
        with _no_crypto():
            chk = check_bundle(p, trust)
            assert (chk.signature, chk.chain, chk.authority) == \
                (B.SIG_VERIFIED, B.CHAIN_NOT_CHECKED, "lab-ca"), (chk.signature, chk.detail)
            assert "pure-python" in chk.detail and "NOT checked here" in chk.detail, chk.detail
            _, src = B.load_for_flight(p, trust_store=trust)
            assert (src["signature"], src["chain"]) == ("verified", "not-checked-here"), src
            _refused_as(lambda: B.load_for_flight(p, trust_store=trust, require_chain=True),
                        "certificate chain not-checked-here")
            os.environ[B.REQUIRE_CHAIN_ENV] = "1"
            _refused_as(lambda: B.load_for_flight(p, trust_store=trust),
                        "certificate chain not-checked-here", B.REQUIRE_CHAIN_ENV)
            del os.environ[B.REQUIRE_CHAIN_ENV]
            # not pinned: only the certificate could vouch for it
            got = check_bundle(p, loose)
            assert (got.signature, got.chain) == (B.SIG_UNVERIFIABLE, B.CHAIN_NOT_CHECKED), got
            assert "NOT checked" in got.detail and "cryptography" in got.detail, got.detail
            _refused_as(lambda: load_bundle(p, trust_store=loose), "unverifiable-here")
            _refused_as(lambda: B.load_for_flight(p, trust_store=loose), "unverifiable-here")
            # what needs no parser is still checked: the certificate's digest
            _corrupt(lambda: check_bundle(_repack(p, _tmp("swap.tar.gz"),
                                                  {B.SIGNER_CERT_NAME: other}), trust),
                     "not the certificate that was signed for")
        with _no_crypto(fallback=False):
            got = check_bundle(p, trust)
            assert (got.signature, got.chain) == (B.SIG_UNVERIFIABLE, B.CHAIN_NOT_CHECKED), got
        # With `cryptography` back: the rule is opt-in, and under it a bare
        # key or an unsigned bundle does not fly, flag or no flag.
        assert B.load_for_flight(plain, trust_store=bare_trust)[1]["chain"] == "none"
        os.environ[B.REQUIRE_CHAIN_ENV] = "1"
        assert B.load_for_flight(p, trust_store=trust)[1]["chain"] == "verified"
        _refused_as(lambda: B.load_for_flight(plain, trust_store=bare_trust),
                    "certificate chain none")
        _refused_as(lambda: B.load_for_flight(_unsigned(pol), allow_unverified=True),
                    "signature unsigned")
        os.environ[B.REQUIRE_CHAIN_ENV] = "0"
        assert B.load_for_flight(plain, trust_store=bare_trust)[1]["signature"] == "verified"
    finally:
        os.environ.pop(B.REQUIRE_CHAIN_ENV, None)
        if saved is not None:
            os.environ[B.REQUIRE_CHAIN_ENV] = saved


def test_the_issue_cert_command_certifies_every_pinned_key_from_public_keys_only():
    """The integrator's step, on throw-away material: one command lists the
    CA, certifies the desktop's key AND a key whose private half is on another
    machine (the AGX Orin's, known here by its public key alone), and retires
    both bare identities. Nothing private is written, the lab's own trust
    store is not touched, and a second run is a no-op."""
    if not B.HAVE_CRYPTO:
        return SKIP
    from cryptography.hazmat.primitives import serialization
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
    lab_store = B.DEFAULT_TRUST_STORE.read_bytes()
    desktop, trust = _test_key()
    orin_key = Ed25519PrivateKey.generate()                 # "on the Orin"
    orin = B.Signer(authority="test-lab", private_key=orin_key)
    B.register_public_key(orin, trust, note="the other machine's key: public half only")
    pki, ca_pem = _ca()
    pki_before = sorted(f.name for f in pki.iterdir())

    assert B._main(["issue-cert", "--pki", str(pki), "--trust", str(trust)]) == 0
    store = _store(trust)
    fp = _der_sha256(ca_pem)
    want = {f"lab-ca:{fp[:16]}:{s.key_id}": s for s in (desktop, orin)}
    certified = {k: v for k, v in store["signers"].items() if v.get("issuer")}
    assert set(certified) == set(want), sorted(certified)
    for ident, s in want.items():
        e = certified[ident]
        assert e["public_key_hex"] == s.public_raw.hex() and e["issuer"] == "lab-ca"
        assert B._chain_problem(e["certificate_pem"], ca_pem, s.public_raw) is None
        assert store["signers"][s.identity]["superseded_by"] == ident
        assert store["signers"][s.identity]["retired_on"].endswith("Z")
    assert sorted(f.name for f in pki.iterdir()) == pki_before, "the CA folder changed"
    assert "PRIVATE" not in trust.read_text(encoding="utf-8")
    assert B.DEFAULT_TRUST_STORE.read_bytes() == lab_store, "the lab's trust store changed"

    was = trust.read_bytes()
    assert B._main(["issue-cert", "--pki", str(pki), "--trust", str(trust)]) == 0
    assert trust.read_bytes() == was, "a second run re-issued or rewrote something"

    # The other machine, with the synced trust store and its own key: what it
    # signs is under the CA, and verifies here.
    there = Path(tempfile.mkdtemp()) / "orin.pem"
    there.write_bytes(orin_key.private_bytes(serialization.Encoding.PEM,
                                             serialization.PrivateFormat.PKCS8,
                                             serialization.NoEncryption()))
    orin_ca = B.load_signer(there, trust)
    assert orin_ca.identity in want and orin_ca.identity != orin.identity
    pol = load_policy(DEMO)
    p = write_bundle(pol, _tmp("orin.tar.gz"), signer=orin_ca)
    chk = check_bundle(p, trust)
    assert (chk.signature, chk.chain) == (B.SIG_VERIFIED, B.CHAIN_VERIFIED), chk.detail
    assert B._main(["verify", str(p), "--trust", str(trust), "--require-chain"]) == 0
    # A bundle issued NOW under the retired bare label is refused...
    stale = write_bundle(pol, _tmp("stale.tar.gz"), signer=orin)
    assert check_bundle(stale, trust).signature == B.SIG_UNTRUSTED
    assert B._main(["verify", str(stale), "--trust", str(trust)]) == 1
    # ...and `verify --require-chain` says no to a bare key that verifies.
    bare, bare_trust = _test_key()
    plain = write_bundle(pol, _tmp("plain.tar.gz"), signer=bare)
    assert B._main(["verify", str(plain), "--trust", str(bare_trust)]) == 0
    assert B._main(["verify", str(plain), "--trust", str(bare_trust), "--require-chain"]) == 1

    # One key by identity, and one by its public key alone.
    lone, lone_trust = _test_key()
    extra = B.Signer("x", Ed25519PrivateKey.generate()).public_raw
    assert B._main(["issue-cert", "--pki", str(pki), "--trust", str(lone_trust),
                    "--identity", lone.identity, "--public-key-hex", extra.hex(),
                    "--keep-legacy"]) == 0
    got = _store(lone_trust)["signers"]
    assert {v["public_key_hex"] for v in got.values() if v.get("issuer")} == \
        {lone.public_raw.hex(), extra.hex()}
    assert "retired_on" not in got[lone.identity]
    # Refusals: another CA under the name already listed; a folder with no CA.
    other_pki, _ = _ca()
    assert B._main(["issue-cert", "--pki", str(other_pki), "--trust", str(trust)]) == 2
    assert B._main(["issue-cert", "--pki", str(Path(tempfile.mkdtemp())),
                    "--trust", str(trust)]) == 2
    assert trust.read_bytes() == was


def test_the_evidence_record_says_what_this_machine_established():
    """tools/bundle_ca_evidence.py, the record the integrator writes on the
    desktop and on the AGX Orin after `issue-cert`: on a throw-away store and
    a throw-away bundles folder it names the authority by the SHA-256 of its
    certificate, gives each signer its standing as checked here, and counts
    the bundles by signature and chain. Without `cryptography` it must say
    "not-checked-here", never "verified"; it reads public data and writes
    only the file it is told to."""
    if not B.HAVE_CRYPTO:
        return SKIP
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "bundle_ca_evidence", ROOT / "tools" / "bundle_ca_evidence.py")
    E = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(E)
    signer, trust, _, ca_pem, bare = _certified()
    pol = load_policy(DEMO)
    folder = Path(tempfile.mkdtemp())
    write_bundle(pol, folder / "a-ca.tar.gz", signer=signer, issued_at=ISSUED)
    write_bundle(pol, folder / "b-before.tar.gz", signer=bare, issued_at=ISSUED)
    write_bundle(pol, folder / "c-since.tar.gz", signer=bare)       # issued now
    write_bundle(pol, folder / "d-unsigned.tar.gz", signer=None, issued_at=ISSUED)
    was = trust.read_bytes()

    rec = E.collect(trust, folder)
    fp = _der_sha256(ca_pem)
    assert [(a["name"], a["sha256"], a["fingerprint_matches_record"])
            for a in rec["authorities"]] == [("lab-ca", fp, True)], rec["authorities"]
    by = {s["identity"]: s for s in rec["signers"]}
    assert by[signer.identity]["accepted_here"] and by[signer.identity]["chain"] == "verified"
    assert by[signer.identity]["issuer"] == "lab-ca" and not by[signer.identity]["retired_on"]
    assert by[bare.identity]["accepted_here"] and by[bare.identity]["chain"] == "none"
    assert by[bare.identity]["superseded_by"] == signer.identity and by[bare.identity]["retired_on"]
    assert rec["bundles"]["counts"] == {
        "policy / unsigned / chain none": 1,
        "policy / untrusted-signer / chain none": 1,
        "policy / verified / chain none": 1,
        "policy / verified / chain verified": 1}, rec["bundles"]["counts"]
    rows = {r["file"]: r for r in rec["bundles"]["policy"]}
    assert rows["a-ca.tar.gz"]["signer"]["ca_fingerprint"] == "sha256:" + fp
    assert rows["a-ca.tar.gz"]["signed_by"] == signer.identity
    assert "retired on" in rows["c-since.tar.gz"]["detail"], rows["c-since.tar.gz"]
    assert rec["summary"] == {
        "authorities": 1, "certified_signers": 1,
        "certified_signers_chain_verified_here": 1,
        "bare_signers_in_force": 0, "bare_signers_retired": 1,
        "bundles": rec["bundles"]["counts"]}, rec["summary"]
    assert rec["drill"].startswith("not run") and rec["machine"]["cryptography"]

    with _no_crypto():
        there = E.collect(trust, folder)
    assert there["machine"]["cryptography"] is None
    assert there["machine"]["ed25519_verifier"] == "pure-python"
    assert there["summary"]["certified_signers_chain_verified_here"] == 0
    assert {s["identity"]: s["chain"] for s in there["signers"]} == \
        {signer.identity: "not-checked-here", bare.identity: "none"}, there["signers"]
    assert there["bundles"]["counts"]["policy / verified / chain not-checked-here"] == 1

    # An authority that is gone is said per signer, not only per bundle.
    doc = _store(trust)
    del doc["authorities"]["lab-ca"]
    loose = trust.parent / "no_authority.json"
    loose.write_text(json.dumps(doc), encoding="utf-8")
    lost = {s["identity"]: s for s in E.collect(loose, folder)["signers"]}[signer.identity]
    assert not lost["accepted_here"] and "is not listed" in lost["why"], lost

    out = folder / "record" / "evidence.json"
    assert E.main(["--trust", str(trust), "--bundles", str(folder), "--out", str(out)]) == 0
    assert json.loads(out.read_text(encoding="utf-8"))["summary"] == rec["summary"]
    assert trust.read_bytes() == was and "PRIVATE" not in out.read_text(encoding="utf-8")


def test_one_command_issues_the_certificates_and_re_signs_the_flight_bundles():
    """`python tools/build_flight_bundles.py --issue-cert`: the integrator's
    single step. Run here against a throw-away CA, key, trust store and
    output folder (the tool's own defaults are the lab's, which this test
    never touches): the flight bundle that was signed under the bare label is
    re-signed under the CA and flies with the chain required, the other
    machine's key is certified from its public half in the same step, and a
    folder with no CA stops the command before anything is signed."""
    if not B.HAVE_CRYPTO:
        return SKIP
    import importlib.util
    from guardrail.manifest import flight_bundle_path
    spec = importlib.util.spec_from_file_location(
        "build_flight_bundles_ca", ROOT / "tools" / "build_flight_bundles.py")
    T = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(T)
    desktop, trust = _test_key()
    orin = B.Signer(authority="test-lab", private_key=B._PyEd25519TestKey(os.urandom(32)))
    B.register_public_key(orin, trust, note="the other machine's key: public half only")
    pki, ca_pem = _ca()
    out = Path(tempfile.mkdtemp())
    key = str(trust.parent / "k.pem")
    bundle = flight_bundle_path(load_policy(DEMO, runtime=False), out)
    lab_store = B.DEFAULT_TRUST_STORE.read_bytes()
    saved = os.environ.get("VLAGUARD_TRUST_STORE")
    os.environ["VLAGUARD_TRUST_STORE"] = str(trust)
    try:
        args = ["sim_demo_policy", "--key", key, "--out-dir", str(out)]
        assert T.main(args) == 0
        was = check_bundle(bundle)
        assert (was.signed_by, was.signature, was.chain) == \
            (desktop.identity, B.SIG_VERIFIED, B.CHAIN_NONE), was.detail
        old_bytes = bundle.read_bytes()

        # No CA in the folder: refused, and the bundle on disk is not touched.
        assert T.main(args + ["--issue-cert", "--pki", str(Path(tempfile.mkdtemp()))]) == 2
        assert bundle.read_bytes() == old_bytes and not _store(trust).get("authorities")

        assert T.main(args + ["--issue-cert", "--pki", str(pki)]) == 0
        ref = "lab-ca:" + _der_sha256(ca_pem)[:16]
        now = check_bundle(bundle)
        assert (now.signed_by, now.signature, now.chain, now.authority) == \
            (f"{ref}:{desktop.key_id}", B.SIG_VERIFIED, B.CHAIN_VERIFIED, "lab-ca"), now.detail
        assert now.policy.policy_hash == was.policy.policy_hash, "the re-sign changed the policy"
        assert B.load_for_flight(bundle, require_chain=True)[1]["chain"] == "verified"
        signers = _store(trust)["signers"]
        assert signers[f"{ref}:{orin.key_id}"]["issuer"] == "lab-ca", sorted(signers)
        assert signers[orin.identity]["retired_on"] and signers[desktop.identity]["retired_on"]
        # Again: nothing is re-issued, and the bundle is the same bytes when
        # the issue time is pinned.
        store_bytes = trust.read_bytes()
        pinned = args + ["--issued-at", ISSUED]
        assert T.main(pinned) == 0
        first = bundle.read_bytes()
        assert T.main(pinned + ["--issue-cert", "--pki", str(pki)]) == 0
        assert trust.read_bytes() == store_bytes and bundle.read_bytes() == first
        assert T.main(["sim_demo_policy", "--check", "--out-dir", str(out)]) == 0
    finally:
        if saved is None:
            os.environ.pop("VLAGUARD_TRUST_STORE", None)
        else:
            os.environ["VLAGUARD_TRUST_STORE"] = saved
    assert B.DEFAULT_TRUST_STORE.read_bytes() == lab_store, "the lab's trust store changed"


def _legacy_bundle(pol, ir_obj=None) -> Path:
    """The pre-2026-10-06 format: 16-hex hash over model_dump() with nulls, IR
    with nulls, placeholder signer."""
    ir = json.dumps(ir_obj if ir_obj is not None else pol.model_dump(mode="json"),
                    sort_keys=True, separators=(",", ":")).encode()
    old = pol.legacy_hashes()["legacy16-include-defaults"]
    man = {"policy_id": pol.policy_id, "version": pol.version,
           "generation": pol.generation, "policy_hash": old,
           "signed_by": B.LEGACY_PLACEHOLDER_SIGNER, "changelog": "initial"}
    p = _tmp("legacy.tar.gz")
    with tarfile.open(p, "w:gz") as tar:
        for name, data in ((IR_NAME, ir),
                           (MANIFEST_NAME, json.dumps(man, indent=2, sort_keys=True).encode()),
                           (SIGNATURE_NAME, f"{old} {B.LEGACY_PLACEHOLDER_SIGNER}\n".encode())):
            info = tarfile.TarInfo(name=name)
            info.size, info.mtime = len(data), 0
            tar.addfile(info, io.BytesIO(data))
    return p


def test_a_bundle_written_before_the_release_still_loads_as_legacy():
    """It must load (refusing it would orphan whatever it documents), report
    its form, and still refuse to pass as signed."""
    pol = load_policy(WGS84)
    p = _legacy_bundle(pol)
    chk = check_bundle(p)
    assert chk.hash_form == "legacy16-include-defaults", chk.hash_form
    assert chk.signature == B.SIG_UNSIGNED
    assert chk.policy.policy_hash == pol.policy_hash
    try:
        load_bundle(p)
    except BundleSignatureError:
        pass
    else:
        raise AssertionError("a placeholder-signed bundle loaded as signed")


def test_a_legacy_bundle_with_an_unknown_key_is_refused():
    """A legacy hash digests the REBUILT object, so a key pydantic drops never
    reaches it. The v2 check hashes bytes; the legacy branch must compare the
    archived content instead. Shown failing on the pre-fix code: this bundle
    loaded under legacy16-include-defaults."""
    pol = load_policy(WGS84)
    ir = pol.model_dump(mode="json")
    ir["constraints"][0]["altitude_cieling_m"] = 40.0          # sic
    try:
        check_bundle(_legacy_bundle(pol, ir))
    except ValueError as e:
        # Since 2026-10-06 the model itself refuses the key ("cannot
        # represent"); the legacy branch's own comparison ("does not
        # represent") stays as the second line of defence.
        assert "represent" in str(e), e
    else:
        raise AssertionError("a legacy IR with an unknown key was accepted")


def test_issued_at_in_a_policy_file_is_kept_not_dropped():
    """The grant's worked example puts issued_at in the policy document. Until
    2026-10-06 it was refused here (it had been silently dropped before that).
    Now it is part of the document as the grant has it: kept, hashed, and
    carried to the bundle manifest - never dropped, which was the original
    defect. Shown failing on the pre-fix code: the file was refused."""
    f = Path(tempfile.mkdtemp()) / "with_issued_at.yaml"
    f.write_text(DEMO.read_text(encoding="utf-8")
                 + "\nissued_at: 2026-04-28T09:00:00Z\n", encoding="utf-8")
    pol = load_policy(f)
    assert pol.issued_at == "2026-04-28T09:00:00+00:00", pol.issued_at
    assert pol.policy_hash != load_policy(DEMO).policy_hash
    man = json.loads(_members(write_bundle(pol, _tmp(), signer=None))[MANIFEST_NAME])
    assert man["issued_at"] == "2026-04-28T09:00:00Z", man


def test_the_bundles_already_on_disk_still_open():
    """bundles/*.tar.gz written before the release (gitignored, local)."""
    old = [p for p in sorted((ROOT / "bundles").glob("*.tar.gz"))
           if not p.name.endswith(".replay.tar.gz")]
    if not old:
        return SKIP
    for p in old:
        chk = check_bundle(p)
        assert chk.hash_form, p.name
        assert chk.signature in (B.SIG_UNSIGNED, B.SIG_VERIFIED,
                                 B.SIG_UNVERIFIABLE), (p.name, chk.signature)


def test_the_bundles_on_disk_still_open_after_the_lab_keys_are_certified():
    """WP1-04: certifying the keys must not strand what they signed before.
    The repository's trust store is COPIED (public data; the copy is the only
    store written) and every key it has in force is certified by a
    throw-away CA made here, listed as `drill-ca` (so this holds before the
    integrator's `issue-cert` and after it, when `lab-ca` is already taken):
    no bare identity is left in force, and every bundle and replay bundle in
    bundles/ opens with the status it had under the store as committed.
    Neither the lab's CA key nor its trust store is touched. Without
    bundles/ (a fresh worktree, CI) only the store half runs, and the test
    says SKIP."""
    if not B.HAVE_CRYPTO:
        return SKIP
    from guardrail.replay import read_replay
    lab_store = B.DEFAULT_TRUST_STORE.read_bytes()
    pinned = {e["public_key_hex"] for e in B.load_trust_store(B.DEFAULT_TRUST_STORE).values()
              if not e.get("retired_on")}
    assert pinned, "the repository's trust store pins no key in force"
    copy = Path(tempfile.mkdtemp()) / "trusted_signers.json"
    copy.write_bytes(lab_store)
    pki, ca_pem = _ca("Drill Root CA")
    rows = B.issue_signer_certificates(pki, copy, authority="drill-ca")
    ref = "drill-ca:" + _der_sha256(ca_pem)[:16]
    assert {r["public_key_hex"] for r in rows} == pinned, rows
    assert all(r["issued"] and r["identity"].startswith(ref + ":") for r in rows), rows
    after = B.load_trust_store(copy)
    in_force = sorted(k for k, v in after.items() if not v.get("issuer") and not v.get("retired_on"))
    assert not in_force, f"bare identities still in force after certifying: {in_force}"
    for r in rows:
        st = B.signer_standing(r["identity"], copy)
        assert st.ok and st.chain == B.CHAIN_VERIFIED, (r["identity"], st)
    assert B.DEFAULT_TRUST_STORE.read_bytes() == lab_store, "the lab's trust store changed"

    found = sorted((ROOT / "bundles").glob("*.tar.gz"))
    if not found:
        return SKIP
    for p in found:
        if p.name.endswith(".replay.tar.gz"):
            was = read_replay(p)["signature"]["status"]
            now = read_replay(p, copy)["signature"]["status"]
        else:
            was = check_bundle(p).signature
            now = check_bundle(p, copy).signature
        assert now == was, (p.name, was, now)


def _in_git_checkout(root: Path) -> bool | None:
    """True inside a git work tree, False outside one, None when git cannot run.

    A `git archive HEAD | tar -x` copy, the clean checkout the CI header says
    every new CI file is first run in, has no .git: there `git check-ignore`
    exits 128 ("not a git repository") and `git ls-files` prints nothing, so
    the ignore rules cannot be asked at all. Until 2026-10-19 the lab-key
    test read that 128 as "the private key path is not gitignored" and FAILED
    on every such copy (review of 2026-10-19), so the clean-copy check could
    not tell a leaked key from a missing .git."""
    try:
        r = subprocess.run(["git", "-C", str(root), "rev-parse", "--is-inside-work-tree"],
                           capture_output=True, text=True, timeout=20)
    except (OSError, subprocess.SubprocessError):
        return None
    return r.returncode == 0 and r.stdout.strip() == "true"


def _check_the_lab_key(root: Path):
    store = json.loads((root / "policies" / "keys" / "trusted_signers.json")
                       .read_text(encoding="utf-8"))
    labs = {k: v for k, v in store["signers"].items() if k.startswith("lab-dev:ed25519:")}
    assert labs, store
    for v in labs.values():
        assert len(bytes.fromhex(v["public_key_hex"])) == 32
    if not _in_git_checkout(root):
        print("      (not a git checkout, or no git: the ignore rules cannot be asked)")
        return SKIP
    try:
        ign = subprocess.run(["git", "-C", str(root), "check-ignore", "-q",
                              "policies/keys/lab-dev-ed25519.pem"],
                             capture_output=True, timeout=20)
        tracked = subprocess.run(["git", "-C", str(root), "ls-files",
                                  "policies/keys"], capture_output=True,
                                 text=True, timeout=20)
    except (OSError, subprocess.SubprocessError):
        return SKIP
    assert ign.returncode == 0, "the private key path is not gitignored"
    assert not any(x.endswith((".pem", ".key")) for x in tracked.stdout.split()), (
        f"a private key is tracked: {tracked.stdout}")


def test_the_lab_key_is_public_in_git_and_private_out_of_it():
    """The trust anchor must ship with a clone; the private key never may."""
    return _check_the_lab_key(ROOT)


def test_outside_a_git_checkout_the_lab_key_check_skips_and_does_not_fail():
    """The same check on a copy of the key store with no .git around it (as a
    `git archive` copy has) must say SKIP, never FAIL and never PASS: no
    ignore rule was read. Shown failing first: before 2026-10-19 it raised
    "the private key path is not gitignored" here."""
    copy = Path(tempfile.mkdtemp())
    if _in_git_checkout(copy) is not False:
        return SKIP                      # no git, or the temp folder sits in a repo
    keys = copy / "policies" / "keys"
    keys.mkdir(parents=True)
    (keys / "trusted_signers.json").write_bytes(
        (ROOT / "policies" / "keys" / "trusted_signers.json").read_bytes())
    assert _check_the_lab_key(copy) == SKIP


def test_the_schema_command_writes_the_published_schema():
    out = _tmp("policy_dsl.schema.json")
    assert B._main(["schema", "-o", str(out)]) == 0
    got = json.loads(out.read_text(encoding="utf-8"))
    assert got == B.policy_json_schema()
    assert got["x-hash-scheme"] == "sha256-canonical-v3"
    assert "PolygonFence" in got["$defs"] and "Corridor" in got["$defs"]


def test_a_flight_gets_its_policy_from_the_bundle_and_says_where_from():
    """ARCH-29 / WP3-21: the flight entry points load through load_for_flight.
    A YAML is recorded as unsigned; a bundle records its signature status; an
    unverified bundle does not fly unless the caller accepts that on record;
    and a run handed both is refused rather than left to pick one."""
    pol, src = B.load_for_flight(None, DEMO)
    assert src["kind"] == "yaml" and src["signature"] == "unsigned", src
    assert src["loaded_policy_hash"] == pol.policy_hash

    unsigned = _unsigned(load_policy(DEMO))
    try:
        B.load_for_flight(unsigned)
    except BundleSignatureError as e:
        assert "allow-unverified-bundle" in str(e), e
    else:
        raise AssertionError("an unsigned bundle flew as if signed")
    pol2, src2 = B.load_for_flight(unsigned, allow_unverified=True)
    assert src2["kind"] == "bundle" and src2["signature"] == B.SIG_UNSIGNED
    assert src2["accepted_unverified"] is True
    assert pol2.policy_hash == pol.policy_hash

    try:
        B.load_for_flight(unsigned, DEMO)
    except ValueError as e:
        assert "not both" in str(e), e
    else:
        raise AssertionError("a run with two candidate policies was accepted")

    signer, trust = _test_key()
    signed = write_bundle(load_policy(DEMO), _tmp(), signer=signer, issued_at=ISSUED)
    pol3, src3 = B.load_for_flight(signed, trust_store=trust)
    assert src3["signature"] == B.SIG_VERIFIED and "accepted_unverified" not in src3
    assert src3["issued_at"] == ISSUED and src3["signed_by"] == signer.identity


def test_every_flight_entry_point_can_fly_a_bundle():
    """Checked by reading the source: none of the three imports on this host
    (rclpy, pymavlink, Project AirSim)."""
    for rel in ("sitl/ros2_shield_node.py", "sitl/run_sitl_demo.py",
                "demo/follow_vlm.py"):
        src = (ROOT / rel).read_text(encoding="utf-8")
        assert '"--bundle"' in src, f"{rel} has no --bundle option"
        assert '"--allow-unverified-bundle"' in src, rel
        assert "load_for_flight(" in src, f"{rel} does not load through load_for_flight"
        assert "load_policy(args.policy)" not in src and "load_policy(policy_path" not in src, (
            f"{rel} still builds its rules straight from YAML")
        assert src.count('"policy_source"') >= 2, (
            f"{rel} must record the policy source in metrics.json AND kpi.json")
        assert "policy_source=policy_source" in src or \
            "policy_source=self.policy_source" in src, (
                f"{rel} does not hand the policy source to its replay bundle")


def test_a_refused_policy_leaves_the_previous_run_untouched():
    """follow_vlm used to delete demo/out/<tag>/flight_log.jsonl BEFORE loading
    the policy, so a refused bundle (or --bundle with --policy) on a re-used
    tag destroyed the old log and orphaned its manifest and kpi.json. The
    policy is now loaded before the output folder is touched. Checked by
    source order, since neither script imports on this host. Shown failing on
    the pre-fix code: the unlink came first in follow_vlm, mkdir in
    run_sitl_demo."""
    for rel, marker in (("demo/follow_vlm.py", "(out / f).unlink()"),
                        ("sitl/run_sitl_demo.py", "out.mkdir(")):
        src = (ROOT / rel).read_text(encoding="utf-8")
        assert src.count("load_for_flight(") == 1, rel
        assert src.index("load_for_flight(") < src.index(marker), (
            f"{rel}: the output folder is touched before the policy is loaded")


def test_the_cli_refuses_new_content_under_a_locked_version():
    """`python -m guardrail.bundle` is a release: it must not publish changed
    content under a version the lock already gives to other content."""
    src = DEMO.read_text(encoding="utf-8").replace("alt_max_m: 20", "alt_max_m: 25")
    assert src != DEMO.read_text(encoding="utf-8")
    f = Path(tempfile.mkdtemp()) / "edited.yaml"
    f.write_text(src, encoding="utf-8")
    assert B._main([str(f), "-o", str(_tmp()), "--unsigned"]) == 2


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


def test_geographic_coordinates_without_an_origin_get_the_references_frame_never_0_0():
    """An ASSUMED origin does not fail - it relocates the policy and validates,
    which is why (0, 0) is never used. The grant's form has no origin at all,
    so since 2026-10-06 the frame is DERIVED from the geometry the way the
    reference derives it (mean of the first polygon's vertices): every rule
    stays where its author put it, and the frame is near the rules.

    Shown failing on the pre-fix code: the policy was refused."""
    raw = {"policy_id": "p", "constraints": [
        {"id": "f", "type": "polygon_fence", **HARD_P0,
         "vertices": [{"lat": 25.0, "lon": 121.0}, {"lat": 25.1, "lon": 121.0},
                      {"lat": 25.1, "lon": 121.1}]}]}
    # Since IR schema 3.1.0 (2026-11-03, WP1-14) nothing is projected at load:
    # project_raw leaves the points as lat/lon, and the metres are the view the
    # Shield reads (XY.x / XY.y), computed then.
    out = project_raw(raw)
    assert all("x" not in p for p in out["constraints"][0]["vertices"]), out
    from guardrail.models import Policy
    pol = Policy.model_validate(raw)
    pts = [(v.x, v.y) for v in pol.constraints[0].vertices]
    assert all(abs(x) < 20_000 and abs(y) < 20_000 for x, y in pts), pts
    assert pol.frame_origin == ((25.0 + 25.1 + 25.1) / 3, (121.0 + 121.0 + 121.1) / 3)
    corridor_only = {"policy_id": "p", "constraints": [
        {"id": "c", "type": "corridor", **HARD_P0, "width_m": 10,
         "centerline": [{"lat": 25.0, "lon": 121.0}, {"lat": 25.0, "lon": 121.001}]}]}
    assert Policy.model_validate(corridor_only).frame_origin == (25.0, 121.0005), \
        "with no polygon the reference would use (0, 0); this uses the first geometry"


def test_mixing_frames_in_one_point_is_refused():
    raw = {"policy_id": "p", "origin": {"lat": 25.0, "lon": 121.0},
           "constraints": [
               {"id": "f", "type": "polygon_fence", **HARD_P0,
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
    got = load_bundle(_unsigned(pol), require_signature=False)
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
    # the class rule binds only the followed subject (C8, 2026-10-21; it said
    # "any pedestrian" before)
    assert "10 m away from the pedestrian you are following" in said, said
    assert "any pedestrian" not in said, said
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


def test_a_bundle_written_before_the_hashes_record_still_verifies():
    """Bundles issued before 2026-10-08 have no `hashes` in their manifest.
    They verify as before, under the one form they name - the record is
    additive."""
    pol = load_policy(DEMO)
    b = _unsigned(pol)
    man = json.loads(_members(b)[MANIFEST_NAME])
    assert set(man["hashes"]) == {B.HASH_SCHEME, B.REFERENCE_FORM}
    old = dict(man)
    for k in ("hashes", "reference_problems"):
        old.pop(k)
    before = _repack(b, _tmp("before.tar.gz"),
                     {MANIFEST_NAME: json.dumps(old, indent=2, sort_keys=True).encode()})
    chk = check_bundle(before)
    assert chk.policy.policy_hash == pol.policy_hash
    assert chk.hashes == {B.HASH_SCHEME: pol.policy_hash}, chk.hashes
    assert B.load_bundle(before, require_signature=False).policy_hash == pol.policy_hash


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
    # A test that cannot run here (no `cryptography`) prints SKIP, never PASS.
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs cryptography, git or a git checkout)")
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
