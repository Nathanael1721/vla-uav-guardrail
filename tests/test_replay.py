"""The replay bundle: does it hold the episode, and can it re-derive it?

Run either way:
    pytest tests/test_replay.py -v
    python tests/test_replay.py

`docs/CHECKLIST-remaining-work.md` item 7 records replay bundles as the last
named artefact that does not exist. The risk with closing an item like that is
producing something that satisfies the WORD - a tar with four files in it - and
none of the intent, which is that a reader can re-derive the contractual numbers
from the artefact alone.

So the tests that matter here are not "does it write a file". They are: does it
refuse a policy that did not govern the episode, does it notice a single flipped
byte, and does it recompute the KPIs and compare them.

Since 2026-10-06 a bundle may also carry an Ed25519 signature over its index.
Tests that stage a forgery to reach a DOWNSTREAM check (the KPI recompute, the
member list) write KEYLESS bundles on purpose, so the forgery gets past the
digest and the check under test is the one that has to catch it. The keyed
format has its own tests, with a throw-away key - never the lab key - which
is the pure-Python RFC 8032 test key wherever `cryptography` is missing, so
those tests run in the 3.11 env and the WSL venvs instead of skipping there.
"""
import gzip
import io
import json
import os
import shutil
import sys
import tarfile
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import guardrail.bundle as B                                   # noqa: E402
from guardrail.kpi import compute, rule_priorities             # noqa: E402
from guardrail.models import load_policy                       # noqa: E402
from guardrail.replay import (EPISODE, INDEX_NAME,             # noqa: E402
                              POLICY, SIGNATURE_NAME, VERIFIED_KPI_FIELDS,
                              _SIGNED_BY, _digest,
                              read_replay, verify_replay, write_replay)

RUN = ROOT / "demo" / "out" / "retarget_demo2"
POLICY_FILE = ROOT / "policies" / "follow_pedestrian.yaml"


def _have_fixture() -> bool:
    return (RUN / "flight_log.jsonl").is_file() and POLICY_FILE.is_file()


def _tmp() -> Path:
    return Path(tempfile.mkdtemp(prefix="replay_"))


SKIP = "SKIP"


def _test_key():
    """A throw-away signer and a trust store that trusts only it. Where
    `cryptography` is missing, the pure-Python RFC 8032 test key signs, so the
    keyed tests run on the interpreters the verifier fallback exists for."""
    d = Path(tempfile.mkdtemp(prefix="replaykey_"))
    trust = d / "trust.json"
    if B.HAVE_CRYPTO:
        return B.generate_signing_key(d / "k.pem", authority="test-lab",
                                      trust_store=trust), trust
    signer = B.Signer(authority="test-lab",
                      private_key=B._PyEd25519TestKey(os.urandom(32)))
    B.register_public_key(signer, trust)
    return signer, trust


SIGNER, TRUST = _test_key()


def _rewrite_many(src: Path, dst: Path, changes: dict) -> Path:
    """Copy a bundle with several members replaced, added, or (None) dropped."""
    with tarfile.open(src, "r:gz") as tar:
        members = [(m.name, tar.extractfile(m).read()) for m in tar.getmembers()]
    out, seen = [], set()
    for n, data in members:
        if n in changes:
            seen.add(n)
            if changes[n] is None:
                continue
            data = changes[n]
        out.append((n, data))
    for n, data in changes.items():
        if n not in seen and data is not None:
            out.append((n, data))
    with open(dst, "wb") as fh:
        with gzip.GzipFile(fileobj=fh, mode="wb", mtime=0, filename="") as gz:
            with tarfile.open(fileobj=gz, mode="w") as t:
                for n, data in out:
                    info = tarfile.TarInfo(name=n)
                    info.size = len(data)
                    info.mtime = 0
                    t.addfile(info, io.BytesIO(data))
    return dst


def _rewrite(src: Path, dst: Path, name: str, data: bytes) -> Path:
    """Copy a bundle with one member replaced. A forger with a hex editor."""
    with tarfile.open(src, "r:gz") as tar:
        members = [(m.name, tar.extractfile(m).read()) for m in tar.getmembers()]
    with open(dst, "wb") as fh:
        with gzip.GzipFile(fileobj=fh, mode="wb", mtime=0, filename="") as gz:
            with tarfile.open(fileobj=gz, mode="w") as out:
                for n, b in members:
                    if n == name:
                        b = data
                    info = tarfile.TarInfo(name=n)
                    info.size = len(b)
                    info.mtime = 0
                    out.addfile(info, io.BytesIO(b))
    return dst


def test_a_bundle_round_trips_to_the_same_policy_and_episode():
    if not _have_fixture():
        return SKIP
    d = _tmp()
    policy = load_policy(POLICY_FILE)
    path = write_replay(RUN, policy, d / "r.tar.gz", signer=None)
    got = read_replay(path)
    assert got["policy"].policy_hash == policy.policy_hash
    assert got["index"]["run"] == RUN.name
    n_lines = sum(1 for line in (RUN / "flight_log.jsonl").read_text(
        encoding="utf-8").splitlines() if line.strip())
    assert len(got["rows"]) == n_lines, len(got["rows"])
    shutil.rmtree(d, ignore_errors=True)


def test_the_same_episode_and_policy_produce_the_same_bytes():
    """Reproducibility, the same property `bundle.py` needed three separate
    fixes to get: member mtimes, the gzip wrapper's mtime, and the filename the
    wrapper otherwise copies out of the output path."""
    if not _have_fixture():
        return SKIP
    d = _tmp()
    policy = load_policy(POLICY_FILE)
    a = write_replay(RUN, policy, d / "a.tar.gz", signer=SIGNER).read_bytes()
    b = write_replay(RUN, policy, d / "b_different_name.tar.gz",
                     signer=SIGNER).read_bytes()
    assert a == b, "two builds of the same episode differ"
    shutil.rmtree(d, ignore_errors=True)


def test_it_refuses_a_policy_that_did_not_govern_the_episode():
    """The failure this check exists for is silent and plausible: bundle any
    episode with any policy, and `verify_replay` still says yes, because the
    KPI recompute uses whatever priorities it is handed. The bundle would then
    ship a clean escape rate derived under rules the aircraft never flew
    under."""
    if not _have_fixture():
        return SKIP
    other = ROOT / "demo" / "out" / "adopt_check"
    if not (other / "manifest.json").is_file():
        return SKIP
    d = _tmp()
    try:
        write_replay(other, load_policy(POLICY_FILE), d / "wrong.tar.gz",
                     signer=None)
    except ValueError as exc:
        assert "flown under policy" in str(exc), exc
    else:
        raise AssertionError("a foreign policy was accepted")
    shutil.rmtree(d, ignore_errors=True)


def test_one_flipped_byte_in_the_log_is_caught_and_named():
    if not _have_fixture():
        return SKIP
    d = _tmp()
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz",
                        signer=None)
    with tarfile.open(good, "r:gz") as tar:
        log = tar.extractfile(EPISODE + "flight_log.jsonl").read()
    bad = _rewrite(good, d / "bad.tar.gz", EPISODE + "flight_log.jsonl",
                   log.replace(b'"tick": 1,', b'"tick": 2,', 1))
    try:
        read_replay(bad)
    except ValueError as exc:
        assert "flight_log.jsonl" in str(exc), exc
    else:
        raise AssertionError("a modified episode log was accepted")
    assert verify_replay(bad)[0] is False
    shutil.rmtree(d, ignore_errors=True)


def test_rewriting_the_index_without_the_signature_is_caught():
    if not _have_fixture():
        return SKIP
    d = _tmp()
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz",
                        signer=None)
    with tarfile.open(good, "r:gz") as tar:
        index = json.loads(tar.extractfile(INDEX_NAME).read())
    index["run"] = "a-flight-that-never-happened"
    bad = _rewrite(good, d / "bad.tar.gz", INDEX_NAME,
                   json.dumps(index, indent=2, sort_keys=True).encode())
    try:
        read_replay(bad)
    except ValueError as exc:
        assert "signature" in str(exc), exc
    else:
        raise AssertionError("a rewritten index was accepted")
    shutil.rmtree(d, ignore_errors=True)


def test_a_CONSISTENT_forgery_passes_a_KEYLESS_bundle_but_never_silently():
    """The honest limit of the keyless format, asserted rather than implied.

    An earlier version of the test above was named "...still fails" and its
    docstring said "the signature covers the index, so consistency is not
    enough". That is false for a keyless bundle: signature.txt is
    sha256(replay.json), so a forger recomputes it in one line.

    The day a real key arrived (2026-10-06) this test was due to be looked at,
    as its docstring promised. Keyless bundles still exist - the four tracked
    ones, and any written without the key - so the limit stays asserted, with
    one change: verify_replay now SAYS the bundle is unsigned. The keyed
    format's refusal is the next test."""
    if not _have_fixture():
        return SKIP
    d = _tmp()
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz",
                        signer=None)
    with tarfile.open(good, "r:gz") as tar:
        index = json.loads(tar.extractfile(INDEX_NAME).read())
        log = tar.extractfile(EPISODE + "flight_log.jsonl").read()

    forged_log = log.replace(b'"tick": 1,', b'"tick": 999,', 1)
    assert forged_log != log
    index["members"] = dict(index["members"])
    index["members"][EPISODE + "flight_log.jsonl"] = _digest(forged_log)
    index["run"] = "a-flight-that-never-happened"
    ib = json.dumps(index, indent=2, sort_keys=True).encode()
    bad = _rewrite_many(good, d / "forged.tar.gz", {
        EPISODE + "flight_log.jsonl": forged_log,
        INDEX_NAME: ib,
        SIGNATURE_NAME: f"{_digest(ib)} {_SIGNED_BY}\n".encode(),
    })
    got = read_replay(bad)
    assert got["index"]["run"] == "a-flight-that-never-happened"
    assert got["signature"]["status"] == "unsigned", got["signature"]
    ok, why = verify_replay(bad)
    assert ok is True, "the keyless format cannot refuse a consistent forgery"
    assert any("UNSIGNED" in w for w in why), (
        f"a keyless bundle verified without saying it was unsigned: {why}")


def test_a_CONSISTENT_forgery_of_a_SIGNED_bundle_is_refused():
    """The same forgery against a keyed bundle: index and digest recomputed,
    the Ed25519 line left as it was, because without the key it cannot be
    recomputed. read_replay must refuse it."""
    if not _have_fixture():
        return SKIP
    d = _tmp()
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz",
                        signer=SIGNER)
    assert verify_replay(good, TRUST) == (True, []), verify_replay(good, TRUST)
    with tarfile.open(good, "r:gz") as tar:
        index = json.loads(tar.extractfile(INDEX_NAME).read())
        log = tar.extractfile(EPISODE + "flight_log.jsonl").read()
        sig_lines = tar.extractfile(SIGNATURE_NAME).read().decode().splitlines()
    forged_log = log.replace(b'"tick": 1,', b'"tick": 999,', 1)
    index["members"] = dict(index["members"])
    index["members"][EPISODE + "flight_log.jsonl"] = _digest(forged_log)
    ib = json.dumps(index, indent=2, sort_keys=True).encode()
    head = f"{_digest(ib)} {sig_lines[0].split(' ', 1)[1]}"
    bad = _rewrite_many(good, d / "forged.tar.gz", {
        EPISODE + "flight_log.jsonl": forged_log,
        INDEX_NAME: ib,
        SIGNATURE_NAME: (head + "\n" + sig_lines[1] + "\n").encode(),
    })
    try:
        read_replay(bad, TRUST)
    except ValueError as exc:
        assert "signature check failed" in str(exc), exc
    else:
        raise AssertionError("a consistent forgery passed a keyed signature")
    assert verify_replay(bad, TRUST)[0] is False
    shutil.rmtree(d, ignore_errors=True)


def test_stripping_the_signature_to_the_placeholder_is_reported_not_passed():
    """The downgrade a keyless format allows: replace the signer with the
    legacy placeholder and drop the Ed25519 line, and the bundle reads as
    keyless. It cannot be refused (keyless bundles are real, four are tracked),
    so it must at least be loud. The honest limit, asserted."""
    if not _have_fixture():
        return SKIP
    d = _tmp()
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz",
                        signer=SIGNER)
    with tarfile.open(good, "r:gz") as tar:
        first = tar.extractfile(SIGNATURE_NAME).read().decode().splitlines()[0]
    digest = first.split(" ", 1)[0]
    bad = _rewrite(good, d / "stripped.tar.gz", SIGNATURE_NAME,
                   f"{digest} {_SIGNED_BY}\n".encode())
    ok, why = verify_replay(bad, TRUST)
    assert ok and any("UNSIGNED" in w for w in why), (ok, why)
    shutil.rmtree(d, ignore_errors=True)


def test_stripping_the_signature_but_keeping_the_signer_is_refused():
    """The lazier strip - keep the key's identity, drop line 2 - is no longer
    a downgrade: it used to read "unsigned" under the trusted name. No writer
    produces a keyed identity without a signature, so it is refused.
    Shown failing on the pre-fix code: verify_replay returned ok=True."""
    if not _have_fixture():
        return SKIP
    d = _tmp()
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz",
                        signer=SIGNER)
    with tarfile.open(good, "r:gz") as tar:
        first = tar.extractfile(SIGNATURE_NAME).read().decode().splitlines()[0]
    assert first.split(" ", 1)[1] == SIGNER.identity, first
    bad = _rewrite(good, d / "stripped.tar.gz", SIGNATURE_NAME,
                   (first + "\n").encode())
    try:
        read_replay(bad, TRUST)
    except ValueError as exc:
        assert "stripped" in str(exc), exc
    else:
        raise AssertionError("a stripped keyed signature was accepted")
    assert verify_replay(bad, TRUST)[0] is False
    shutil.rmtree(d, ignore_errors=True)


def test_without_cryptography_a_signed_bundle_is_still_checked():
    """The environments this is about (the WSL flight venvs, the 3.11 env) are
    exactly the ones WITHOUT `cryptography`. They used to answer
    "UNVERIFIABLE-HERE" for every signature, forged or not; the pure-Python
    verifier now decides there - a real signature verifies, a forged one is
    refused - and with no verifier at all the status is still never VERIFIED."""
    if not _have_fixture():
        return SKIP
    d = _tmp()
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz",
                        signer=SIGNER)
    with tarfile.open(good, "r:gz") as tar:
        sig_lines = tar.extractfile(SIGNATURE_NAME).read().decode().splitlines()
    forged = bytearray.fromhex(sig_lines[1][len("ed25519:"):])
    forged[7] ^= 1
    bad = _rewrite(good, d / "badsig.tar.gz", SIGNATURE_NAME,
                   f"{sig_lines[0]}\ned25519:{forged.hex()}\n".encode())
    saved = (B.HAVE_CRYPTO, B.PY_VERIFY_FALLBACK)
    B.HAVE_CRYPTO = False
    try:
        assert verify_replay(good, TRUST) == (True, []), verify_replay(good, TRUST)
        assert read_replay(good, TRUST)["signature"]["status"] == B.SIG_VERIFIED
        try:
            read_replay(bad, TRUST)
        except ValueError as exc:
            assert "signature check failed" in str(exc), exc
        else:
            raise AssertionError("a forged signature passed the fallback verifier")
        B.PY_VERIFY_FALLBACK = False
        ok, why = verify_replay(good, TRUST)
        sig = read_replay(good, TRUST)["signature"]
    finally:
        B.HAVE_CRYPTO, B.PY_VERIFY_FALLBACK = saved
    # The KPIs still re-derive (ok), but the signature is reported as NOT
    # checked - never as verified, and never silently.
    assert sig["status"] == B.SIG_UNVERIFIABLE, sig
    assert ok and any("UNVERIFIABLE-HERE" in w for w in why), (ok, why)
    assert not any(w.startswith("signature VERIFIED") for w in why), why
    shutil.rmtree(d, ignore_errors=True)


def test_an_index_naming_another_policy_than_the_archived_ir_is_refused():
    """The index's policy_hash must be the archived IR's, under some known
    form. Keyless repack: swap the index's hash for another policy's, redo the
    digest. Every member digest still matches, so only this check stands
    between the bundle and a reader who trusts the index's policy name."""
    if not _have_fixture():
        return SKIP
    d = _tmp()
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz",
                        signer=None)
    with tarfile.open(good, "r:gz") as tar:
        index = json.loads(tar.extractfile(INDEX_NAME).read())
    other = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    index["policy_hash"] = other.policy_hash
    index["policy_id"] = other.policy_id
    ib = json.dumps(index, indent=2, sort_keys=True).encode()
    bad = _rewrite_many(good, d / "otherpolicy.tar.gz", {
        INDEX_NAME: ib, SIGNATURE_NAME: f"{_digest(ib)} {B.UNSIGNED}\n".encode()})
    try:
        read_replay(bad)
    except ValueError as exc:
        assert "under no form" in str(exc), exc
    else:
        raise AssertionError("an index naming another policy was accepted")
    assert verify_replay(bad)[0] is False
    shutil.rmtree(d, ignore_errors=True)


def test_the_tracked_replay_bundles_still_verify():
    """Four replay bundles are in git, written before 2026-10-06: 16-hex
    policy hash, keyless signature. The identity release must not orphan them.
    On the old-hash code they verified; on the first cut of the new hash they
    all failed ("the index says sha256:083fbd16d33505c3"); they must verify -
    and say they are unsigned."""
    tracked = sorted((ROOT / "bundles").glob("*.replay.tar.gz"))
    if not tracked:
        return SKIP
    for b in tracked:
        got = read_replay(b)
        assert got["hash_form"].startswith("legacy16-"), (b.name, got["hash_form"])
        ok, why = verify_replay(b)
        assert ok, (b.name, why)
        assert any("UNSIGNED" in w for w in why), (b.name, why)


def test_the_index_records_which_hash_form_the_episode_carried():
    """retarget_demo2 flew before the release; its manifest holds the 16-hex
    form. The binding is verified under that form and the form is written
    down, not glossed over."""
    if not _have_fixture():
        return SKIP
    d = _tmp()
    path = write_replay(RUN, load_policy(POLICY_FILE), d / "r.tar.gz",
                        signer=None, policy_source={"kind": "yaml",
                                                    "signature": "unsigned"})
    idx = read_replay(path)["index"]
    flown = json.loads((RUN / "manifest.json").read_text(encoding="utf-8"))["policy_hash"]
    assert idx["flown_policy_hash"] == flown
    assert idx["flown_hash_form"] == "legacy16-include-defaults", idx
    assert idx["policy_hash"] == load_policy(POLICY_FILE).policy_hash
    assert len(idx["policy_hash"]) == len("sha256:") + 64
    assert idx["policy_source"]["signature"] == "unsigned"
    shutil.rmtree(d, ignore_errors=True)


def test_an_undeclared_member_is_refused():
    """The digest loop used to iterate the INDEX, so a member present in the
    archive but absent from the index was neither digested nor rejected - while
    the docstring said "check every digest"."""
    if not _have_fixture():
        return SKIP
    d = _tmp()
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz",
                        signer=None)
    bad = _rewrite_many(good, d / "extra.tar.gz",
                        {"episode/SECRET_note.txt": b"covered by nothing\n"})
    try:
        read_replay(bad)
    except ValueError as exc:
        assert "does not declare" in str(exc) and "SECRET" in str(exc), exc
    else:
        raise AssertionError("an undeclared archive member was accepted")
    shutil.rmtree(d, ignore_errors=True)


def test_a_bundle_whose_index_omits_the_log_says_so():
    """It used to die on a bare KeyError naming no bundle and no reason."""
    if not _have_fixture():
        return SKIP
    d = _tmp()
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz",
                        signer=None)
    with tarfile.open(good, "r:gz") as tar:
        index = json.loads(tar.extractfile(INDEX_NAME).read())
        keep = {n: v for n, v in index["members"].items()
                if not n.endswith("flight_log.jsonl")}
    index["members"] = keep
    ib = json.dumps(index, indent=2, sort_keys=True).encode()
    bad = _rewrite_many(good, d / "nolog.tar.gz", {
        EPISODE + "flight_log.jsonl": None,
        INDEX_NAME: ib,
        SIGNATURE_NAME: f"{_digest(ib)} {_SIGNED_BY}\n".encode(),
    })
    ok, why = verify_replay(bad)
    assert ok is False and why, (ok, why)
    assert "flight_log.jsonl" in why[0] and "KeyError" not in why[0], why
    shutil.rmtree(d, ignore_errors=True)


def test_every_verified_field_is_a_field_something_actually_produces():
    """The defect that made this module claim more than it checked: two of the
    seven names - "p0_ticks" and "fail_safe_correctness" - were keys nothing
    emits, and `field not in stored: continue` skipped both. Five fields were
    compared where seven were claimed, and one of them was the grant's
    fail-safe KPI."""
    if not _have_fixture() or not (RUN / "kpi.json").is_file():
        return SKIP
    stored = json.loads((RUN / "kpi.json").read_text(encoding="utf-8"))
    rows = [json.loads(x) for x in (RUN / "flight_log.jsonl").read_text(
        encoding="utf-8").splitlines() if x.strip()]
    metrics = json.loads((RUN / "metrics.json").read_text(encoding="utf-8"))
    fresh = compute(rows, rule_priorities(load_policy(POLICY_FILE)), metrics)
    missing = [f for f in VERIFIED_KPI_FIELDS
               if f not in stored and f not in fresh]
    assert not missing, f"names nothing produces: {missing}"
    assert "failsafe_trigger_correctness" in VERIFIED_KPI_FIELDS
    assert "p0_violation_ticks" in VERIFIED_KPI_FIELDS


def test_a_tampered_failsafe_figure_is_caught():
    """The scenario the wrong field names allowed: edit the bundle's KPI table
    so the grant's fail-safe KPI reads 0.10 against a >= 99 % target, and the
    bundle verified clean."""
    if not _have_fixture() or not (RUN / "kpi.json").is_file():
        return SKIP
    d = _tmp()
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz",
                        signer=None)
    with tarfile.open(good, "r:gz") as tar:
        index = json.loads(tar.extractfile(INDEX_NAME).read())
        kpi = json.loads(tar.extractfile(EPISODE + "kpi.json").read())
    kpi["failsafe_trigger_correctness"] = 0.10
    kpi["p0_violation_ticks"] = 0
    kb = json.dumps(kpi, indent=1).encode()
    index["members"] = dict(index["members"])
    index["members"][EPISODE + "kpi.json"] = _digest(kb)
    ib = json.dumps(index, indent=2, sort_keys=True).encode()
    bad = _rewrite_many(good, d / "tampered.tar.gz", {
        EPISODE + "kpi.json": kb,
        INDEX_NAME: ib,
        SIGNATURE_NAME: f"{_digest(ib)} {_SIGNED_BY}\n".encode(),
    })
    ok, why = verify_replay(bad)
    assert ok is False, "an edited fail-safe KPI verified clean"
    assert any("failsafe_trigger_correctness" in w for w in why), why
    shutil.rmtree(d, ignore_errors=True)


def test_a_scored_run_without_a_manifest_is_refused():
    """The guard used to be skipped entirely when manifest.json was absent, and
    a real scored flight then bundled cleanly under all 26 other policies. The
    KPI recompute is no backstop: kpi.compute treats an unknown rule_id as P0,
    so a foreign policy changes no KPI field at all."""
    if not _have_fixture() or not (RUN / "kpi.json").is_file():
        return SKIP
    d = _tmp()
    run = d / "scored_no_manifest"
    run.mkdir()
    for f in ("flight_log.jsonl", "metrics.json", "kpi.json"):
        shutil.copy(RUN / f, run / f)
    try:
        write_replay(run, load_policy(POLICY_FILE), d / "x.tar.gz",
                     signer=None)
    except ValueError as exc:
        assert "nothing binds" in str(exc), exc
    else:
        raise AssertionError("a scored run with no manifest was bundled")
    shutil.rmtree(d, ignore_errors=True)


def test_an_unbound_episode_bundles_but_says_the_binding_is_unverified():
    """An episode with no KPI table and no manifest is honest to bundle - it
    just must not look bound. Two such runs exist on disk today."""
    if not _have_fixture():
        return SKIP
    d = _tmp()
    run = d / "unbound"
    run.mkdir()
    shutil.copy(RUN / "flight_log.jsonl", run / "flight_log.jsonl")
    path = write_replay(run, load_policy(POLICY_FILE), d / "u.tar.gz",
                        signer=None)
    got = read_replay(path)
    assert got["index"]["policy_binding"] == "unverified", got["index"]
    ok, why = verify_replay(path)
    assert ok and any("UNVERIFIED" in w for w in why), (ok, why)
    shutil.rmtree(d, ignore_errors=True)


def test_a_bundle_re_derives_its_own_kpis():
    """The property the word 'replayable' is claiming. Signed when this
    interpreter can sign, and then nothing at all may be reported."""
    if not _have_fixture() or not (RUN / "kpi.json").is_file():
        return SKIP
    d = _tmp()
    path = write_replay(RUN, load_policy(POLICY_FILE), d / "r.tar.gz",
                        signer=SIGNER)
    ok, why = verify_replay(path, TRUST)
    assert ok, why
    if SIGNER is not None:
        assert why == [], why
    else:
        assert [w for w in why if not w.startswith("signature ")] == [], why
    shutil.rmtree(d, ignore_errors=True)


def test_an_episode_with_no_log_is_refused_rather_than_packaged_empty():
    d = _tmp()
    (d / "empty_run").mkdir()
    try:
        write_replay(d / "empty_run", load_policy(POLICY_FILE), d / "r.tar.gz",
                     signer=None)
    except ValueError as exc:
        assert "nothing to replay" in str(exc), exc
    else:
        raise AssertionError("an episode with no log was packaged")
    shutil.rmtree(d, ignore_errors=True)


def test_an_unscored_episode_bundles_and_says_so_instead_of_passing_quietly():
    if not _have_fixture():
        return SKIP
    d = _tmp()
    run = d / "unscored"
    run.mkdir()
    shutil.copy(RUN / "flight_log.jsonl", run / "flight_log.jsonl")
    path = write_replay(run, load_policy(POLICY_FILE), d / "r.tar.gz",
                        signer=None)
    ok, why = verify_replay(path)
    # any(), not why[0]: an unscored episode now also reports that its policy
    # binding is unverified, and pinning the ORDER of honest notes would make
    # this test fail for a reason that has nothing to do with what it checks.
    assert ok and any("no kpi.json" in w for w in why), (ok, why)
    shutil.rmtree(d, ignore_errors=True)


# --------------------------------------------------------------------------- #
# Episode records (2026-10-06): every generation, a fresh audit, the rosbag
# --------------------------------------------------------------------------- #

SIM_POLICY = ROOT / "policies" / "sim_demo_policy.yaml"


def _episode(d: Path, hot: bool = True, mission: bool = True,
             stale_audit: bool = False, finish: bool = True, ids: bool = True):
    """A short synthetic flight written the way the SITL rails write one:
    EpisodeRecord first, then the audit logger, rows with the per-tick
    fields (and the episode id), a hot-applied fence half way, and the
    mission_end event. No simulator."""
    from guardrail.audit import AuditLogger
    from guardrail.compiler import ConstraintCompiler
    from guardrail.models import Action4D, PolygonFence, State, XY
    from guardrail.replay import EpisodeRecord, episode_row_fields
    from guardrail.shield import Shield
    run = d / "ep"
    run.mkdir(parents=True, exist_ok=True)
    if stale_audit:
        (run / "audit.jsonl").write_text('{"policy_hash": "sha256:old"}\n',
                                         encoding="utf-8")
    policy = load_policy(SIM_POLICY)
    m = (ConstraintCompiler(policy).parse_command("fly to the northeast pad at 6 m/s")
         if mission else None)
    rec = EpisodeRecord(run, policy, m, lookahead_s=3.0)
    audit = AuditLogger(run / "audit.jsonl", policy)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    rows = []
    for tick in range(40):
        if hot and tick == 20:
            # The rails' own call: a dynamic_nfz where the Shield enforces
            # the grant's mid-flight update model, the polygon_fence where not.
            rec.apply_zone(shield, PolygonFence(
                id="nfz-dynamic", type="polygon_fence",
                vertices=[XY(x=23, y=6), XY(x=31, y=6), XY(x=31, y=14),
                          XY(x=23, y=14)], margin_m=1.0), t=tick * 0.1)
        st = State(x=2.0 + 0.4 * tick, y=2.0 + 0.4 * tick, up=15.0)
        d_ = shield.filter(st, Action4D(vx=4.0, vy=4.0))
        audit.log(tick, d_)
        rows.append({"t": tick * 0.1, "tick": tick, "x": st.x, "y": st.y,
                     "up": st.up, "raw": d_.raw.model_dump(),
                     "emitted": d_.emitted.model_dump(),
                     "violations": [v.model_dump() for v in d_.violations],
                     "emitted_violations": [v.model_dump()
                                            for v in d_.emitted_violations],
                     "repairs": [r.model_dump() for r in d_.repairs],
                     "braked": d_.braked,
                     **episode_row_fields(shield, st, policy,
                                          rec.episode_id if ids else None)})
    (run / "flight_log.jsonl").write_text(
        "".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    if finish:
        rec.end("test episode over", t=4.0)
    return run, policy, rec, rows


def _scored(run: Path, policy, rec) -> None:
    """metrics.json, kpi.json and manifest.json, as a rail writes them at the
    end of a flight, all carrying the episode id."""
    rows = [json.loads(l) for l in
            (run / "flight_log.jsonl").read_text(encoding="utf-8").splitlines()]
    metrics = {"tag": run.name, "shield": "on", "reached": True,
               "frac_within_30m": 1.0, "nfz_s": 0.0,
               "episode_id": rec.episode_id}
    (run / "metrics.json").write_text(json.dumps(metrics), encoding="utf-8")
    kpi = compute(rows, rule_priorities(policy), metrics)
    kpi["episode_id"] = rec.episode_id
    (run / "kpi.json").write_text(json.dumps(kpi), encoding="utf-8")
    (run / "manifest.json").write_text(json.dumps(
        {"policy_hash": policy.policy_hash, "topology": "dev"}), encoding="utf-8")


def test_every_policy_generation_and_its_csp_travel_with_the_episode():
    """WP2-15: after a hot-apply the dynamic KPI run's log named a policy
    that no saved file described. Now both generations, and the CSP compiled
    from each, are in the bundle, and the index lists them."""
    from guardrail.compiler import ConstraintCompiler
    d = _tmp()
    run, policy, _, rows = _episode(d)
    assert {r["generation"] for r in rows} == {0, 1}
    path = write_replay(run, policy, d / "r.tar.gz", signer=None)
    got = read_replay(path)
    assert sorted(got["generation_policies"]) == [0, 1], got["index"]
    table = got["index"]["generations"]
    assert [g["generation"] for g in table] == [0, 1]
    assert table[0]["csp"], table
    assert table[1]["policy_hash"] == policy.policy_hash
    ok, why = verify_replay(path)
    # Generation 1 holds the hot-applied zone. Where the compiler can render
    # it, its CSP must be on file; where it cannot (a dynamic_nfz while
    # guardrail/csp.py still calls that type unenforced, 2026-10-07), the
    # bundle must say why - never a generation silently without its CSP.
    m = ConstraintCompiler(policy).parse_command("fly to the northeast pad at 6 m/s")
    try:
        ConstraintCompiler(policy).write_csp(d / "probe.json", m, lookahead_s=3.0)
        compilable = True
    except Exception:                                # noqa: BLE001
        compilable = False
    if compilable:
        assert table[1]["csp"] and ok and not any("CSP" in w for w in why), why
    else:
        assert not table[1]["csp"], table
        assert ok and any("generation 1 has no CSP on file:" in w
                          and "no event says why" not in w for w in why), why
    shutil.rmtree(d, ignore_errors=True)


def test_a_log_naming_an_unarchived_generation_is_refused():
    """The WP2-15 defect as a check that can fail. With generation 0's files
    gone, 20 rows name a policy the bundle cannot show: refused on write,
    and a bundle forged without them is refused on verify. Before this, the
    bundle kept only the final policy and verified clean."""
    d = _tmp()
    run, policy, _, _ = _episode(d)
    good = write_replay(run, policy, d / "good.tar.gz", signer=None)
    for f in ("policy_g0.json", "csp_g0.json"):
        (run / f).unlink()
    try:
        write_replay(run, policy, d / "bad.tar.gz", signer=None)
    except ValueError as exc:
        assert "no recorded generation" in str(exc), exc
    else:
        raise AssertionError("a log naming an unarchived generation was bundled")
    # The same omission made consistently inside a keyless bundle.
    with tarfile.open(good, "r:gz") as tar:
        idx = json.loads(tar.extractfile(INDEX_NAME).read())
    for m in (EPISODE + "policy_g0.json", EPISODE + "csp_g0.json"):
        idx["members"].pop(m)
    idx["generations"] = [g for g in idx["generations"] if g["generation"] != 0]
    ib = json.dumps(idx, indent=2, sort_keys=True).encode()
    forged = _rewrite_many(good, d / "forged.tar.gz", {
        EPISODE + "policy_g0.json": None, EPISODE + "csp_g0.json": None,
        INDEX_NAME: ib,
        SIGNATURE_NAME: f"{_digest(ib)} {_SIGNED_BY}\n".encode()})
    ok, why = verify_replay(forged)
    assert not ok and "no archived generation" in why[0], (ok, why)
    shutil.rmtree(d, ignore_errors=True)


def test_a_csp_compiled_from_another_generation_is_refused():
    d = _tmp()
    run, policy, _, _ = _episode(d)
    (run / "csp_g1.json").write_bytes((run / "csp_g0.json").read_bytes())
    try:
        write_replay(run, policy, d / "r.tar.gz", signer=None)
    except ValueError as exc:
        assert "compiled from policy" in str(exc), exc
    else:
        raise AssertionError("generation 1 shipped generation 0's CSP")
    shutil.rmtree(d, ignore_errors=True)


def test_a_stale_audit_is_moved_aside_never_appended_to():
    """WP4-20: AuditLogger appends, and ros2_shield_on/audit.jsonl held three
    dates of flights. The new episode starts a fresh file; the old one is
    kept under _previous/, not deleted."""
    d = _tmp()
    run, policy, rec, _ = _episode(d, stale_audit=True)
    hashes = {json.loads(l)["policy_hash"] for l in
              (run / "audit.jsonl").read_text(encoding="utf-8").splitlines()}
    assert "sha256:old" not in hashes, hashes
    kept = list((run / "_previous").rglob("audit.jsonl"))
    assert len(kept) == 1 and "sha256:old" in kept[0].read_text(encoding="utf-8")
    assert "audit.jsonl" in rec.moved
    path = write_replay(run, policy, d / "r.tar.gz", signer=None)
    assert verify_replay(path)[0]
    shutil.rmtree(d, ignore_errors=True)


def test_a_csp_that_could_not_be_compiled_is_said_with_its_reason():
    d = _tmp()
    run, policy, _, _ = _episode(d, hot=False, mission=False)
    path = write_replay(run, policy, d / "r.tar.gz", signer=None)
    ok, why = verify_replay(path)
    assert ok and any("generation 0 has no CSP" in w and "no mission" in w
                      for w in why), why
    shutil.rmtree(d, ignore_errors=True)


def test_the_unsafe_flag_makes_time_to_safe_measurable():
    """WP3-17: the ROS rail logged no `unsafe`, so mean time to safe with the
    Shield on was 'not measurable' on every run."""
    from guardrail.models import Action4D, State
    from guardrail.replay import episode_row_fields
    from guardrail.shield import Shield
    policy = load_policy(SIM_POLICY)
    sh = Shield(policy)
    inside = State(x=15.0, y=15.0, up=15.0)            # inside nfz-square
    sh.filter(inside, Action4D(vx=1.0))
    f = episode_row_fields(sh, inside, policy)
    assert f["unsafe"] and "nfz-square" in f["unsafe_rules"], f
    assert f["shield_ms"] is not None and f["shield_ms"] >= 0.0
    outside = State(x=-5.0, y=-5.0, up=15.0)
    sh.filter(outside, Action4D())
    assert not episode_row_fields(sh, outside, policy)["unsafe"]
    rows = [{"t": i * 0.1, "violations": [], "repairs": [], "raw": {},
             "emitted": {}, "unsafe": i < 5} for i in range(10)]
    k = compute(rows, rule_priorities(policy), {})
    assert k["time_to_safe_not_measurable"] is False
    assert k["mean_time_to_safe_s"] is not None, k


def _bag(run: Path, start_ns: int, closed: bool = True) -> None:
    bag = run / "bag"
    bag.mkdir()
    (bag / "bag_0.mcap").write_bytes(b"\x89MCAP0\r\n" + b"\x00" * 64)
    if closed:
        (bag / "metadata.yaml").write_text(
            "rosbag2_bagfile_information:\n"
            "  version: 9\n  storage_identifier: mcap\n"
            f"  duration:\n    nanoseconds: {int(90e9)}\n"
            f"  starting_time:\n    nanoseconds_since_epoch: {start_ns}\n"
            "  message_count: 10\n"
            "  topics_with_message_count:\n"
            "    - topic_metadata:\n        name: /vla/action_4d\n"
            "        type: std_msgs/msg/Float32MultiArray\n"
            "      message_count: 10\n"
            "    - topic_metadata:\n        name: /mavros/state\n"
            "        type: mavros_msgs/msg/State\n"
            "      message_count: 0\n"
            "  relative_file_paths:\n    - bag_0.mcap\n", encoding="utf-8")


def _episode_start(run: Path) -> int:
    from guardrail.replay import _jsonl, episode_start_ns
    return episode_start_ns(_jsonl((run / "events.jsonl").read_bytes()))


def test_this_episodes_rosbag_is_bundled_and_its_empty_topics_named():
    d = _tmp()
    run, policy, _, _ = _episode(d)
    _bag(run, _episode_start(run) - int(20e9))       # started 20 s before
    path = write_replay(run, policy, d / "r.tar.gz", signer=None)
    got = read_replay(path)
    assert EPISODE + "bag/bag_0.mcap" in got["bag_files"], got["bag_files"]
    assert got["index"]["bag"]["binding"] == "episode"
    ok, why = verify_replay(path)
    assert ok and any("/mavros/state" in w for w in why), why
    shutil.rmtree(d, ignore_errors=True)


def test_a_rosbag_left_by_another_flight_is_refused():
    """A reused run tag can hold yesterday's bag. It must not travel as this
    episode's recording."""
    d = _tmp()
    run, policy, _, _ = _episode(d)
    _bag(run, _episode_start(run) - int(86400e9))     # a day earlier
    path = write_replay(run, policy, d / "r.tar.gz", signer=None)
    got = read_replay(path)
    assert not got["bag_files"] and not got["index"]["bag"]["included"]
    ok, why = verify_replay(path)
    assert ok and any("rosbag NOT included" in w for w in why), why
    shutil.rmtree(d, ignore_errors=True)


def test_an_open_rosbag_is_not_bundled():
    d = _tmp()
    run, policy, _, _ = _episode(d)
    _bag(run, _episode_start(run), closed=False)
    got = read_replay(write_replay(run, policy, d / "r.tar.gz", signer=None))
    assert not got["bag_files"]
    assert "not closed" in got["index"]["bag"]["why"]
    shutil.rmtree(d, ignore_errors=True)


def test_kpi_evidence_can_demand_a_signature():
    """The keyless downgrade (drop the Ed25519 line, swap in the keyless
    placeholder) still reads as keyless by default, because tracked keyless
    bundles are real. A caller verifying KPI evidence can now refuse it."""
    d = _tmp()
    run, policy, _, _ = _episode(d)
    keyless = write_replay(run, policy, d / "k.tar.gz", signer=None)
    assert verify_replay(keyless)[0]
    ok, why = verify_replay(keyless, require_signature=True)
    assert not ok and "signed bundle was required" in why[0], why
    signed = write_replay(run, policy, d / "s.tar.gz", signer=SIGNER)
    ok, why = verify_replay(signed, TRUST, require_signature=True)
    assert ok, why
    shutil.rmtree(d, ignore_errors=True)


def test_the_pack_cli_rebuilds_a_bundle_from_the_episode_alone():
    """run_ros2_demo.sh re-packs after the rosbag is closed. The CLI has no
    policy in memory; it reads the last generation the episode wrote."""
    from guardrail.replay import _main, last_generation_policy
    d = _tmp()
    run, policy, _, _ = _episode(d)
    assert last_generation_policy(run).policy_hash == policy.policy_hash
    _bag(run, _episode_start(run) - int(5e9))
    assert _main(["pack", str(run), "--keyless"]) == 0
    got = read_replay(run / "ep.replay.tar.gz")
    assert got["bag_files"] and len(got["index"]["generations"]) == 2
    shutil.rmtree(d, ignore_errors=True)


# --------------------------------------------------------------------------- #
# One episode, not two (2026-10-07 review)
# --------------------------------------------------------------------------- #

def test_a_crashed_rerun_does_not_bundle_the_previous_flights_log():
    """The review's reproduction. Flight A finished and was scored; a re-run
    of the same tag started a new episode and died before writing its own
    log. run_ros2_demo.sh then runs `pack` anyway (ros2 launch exits 0 when
    the Shield node dies). On the old code that returned rc 0, "re-derives
    its own KPIs", with A's rows bundled under B's events."""
    from guardrail.replay import EpisodeRecord, _main
    d = _tmp()
    run, policy, rec_a, _ = _episode(d)
    _scored(run, policy, rec_a)
    assert _main(["pack", str(run), "--keyless"]) == 0      # A packs cleanly
    # B: a new episode in the same directory, then a crash (no log, no end).
    EpisodeRecord(run, load_policy(SIM_POLICY), None)
    for f in ("flight_log.jsonl", "metrics.json", "kpi.json", "manifest.json",
              "ep.replay.tar.gz"):
        assert not (run / f).exists(), f"{f} of flight A was left beside B"
    assert _main(["pack", str(run), "--keyless"]) == 1, (
        "an episode with no log of its own was packed")
    assert not (run / "ep.replay.tar.gz").exists()
    kept = {p.name for p in (run / "_previous").rglob("*") if p.is_file()}
    assert {"flight_log.jsonl", "kpi.json", "ep.replay.tar.gz"} <= kept, kept
    shutil.rmtree(d, ignore_errors=True)


def test_a_log_from_another_episode_is_refused_even_if_put_back():
    """The second line of defence: flight A's log copied back beside B's
    events (by hand, or by a writer that skipped the rotation) carries A's
    episode id and is refused on write; a bundle forged that way is refused
    on verify."""
    from guardrail.replay import EpisodeRecord
    d = _tmp()
    run, policy, rec_a, _ = _episode(d, hot=False)
    log_a = (run / "flight_log.jsonl").read_bytes()
    good = write_replay(run, policy, d / "good.tar.gz", signer=None)
    rec_b = EpisodeRecord(run, load_policy(SIM_POLICY), None)
    rec_b.end("crashed", t=0.0)
    (run / "flight_log.jsonl").write_bytes(log_a)
    try:
        write_replay(run, load_policy(SIM_POLICY), d / "bad.tar.gz", signer=None)
    except ValueError as exc:
        assert "another flight" in str(exc), exc
    else:
        raise AssertionError("flight A's log was bundled under episode B")
    # The same mix forged into a keyless bundle: B's events, A's rows.
    with tarfile.open(good, "r:gz") as tar:
        idx = json.loads(tar.extractfile(INDEX_NAME).read())
    ev_b = (run / "events.jsonl").read_bytes()
    idx["members"][EPISODE + "events.jsonl"] = _digest(ev_b)
    ib = json.dumps(idx, indent=2, sort_keys=True).encode()
    forged = _rewrite_many(good, d / "forged.tar.gz", {
        EPISODE + "events.jsonl": ev_b, INDEX_NAME: ib,
        SIGNATURE_NAME: f"{_digest(ib)} {_SIGNED_BY}\n".encode()})
    ok, why = verify_replay(forged)
    assert not ok and any("another flight" in w for w in why), (ok, why)
    shutil.rmtree(d, ignore_errors=True)


def test_metrics_or_kpi_from_another_episode_are_refused():
    d = _tmp()
    run, policy, rec, _ = _episode(d, hot=False)
    _scored(run, policy, rec)
    assert verify_replay(write_replay(run, policy, d / "ok.tar.gz",
                                      signer=None))[0]
    for name in ("metrics.json", "kpi.json"):
        doc = json.loads((run / name).read_text(encoding="utf-8"))
        saved = dict(doc)
        doc["episode_id"] = rec.episode_id - 1
        (run / name).write_text(json.dumps(doc), encoding="utf-8")
        try:
            write_replay(run, policy, d / "bad.tar.gz", signer=None)
        except ValueError as exc:
            assert name in str(exc) and "not this episode" in str(exc), exc
        else:
            raise AssertionError(f"{name} of another episode was bundled")
        (run / name).write_text(json.dumps(saved), encoding="utf-8")
    shutil.rmtree(d, ignore_errors=True)


def test_audit_records_from_another_episode_are_refused():
    """The rails give the audit logger the episode id (AuditLogger has taken
    one since 2026-10-07), so every audit record names its episode, and an
    audit record naming another episode is refused like a log row would be.
    Records without an id (every audit written before) bind nothing."""
    from guardrail.replay import episode_binding
    d = _tmp()
    run, policy, rec, _ = _episode(d, hot=False)
    _scored(run, policy, rec)
    events = [json.loads(ln) for ln in
              (run / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    rows = [json.loads(ln) for ln in
            (run / "flight_log.jsonl").read_text(encoding="utf-8").splitlines()]
    mine = [{"tick": 1, "episode_id": str(rec.episode_id)},
            {"tick": 2, "episode_id": rec.episode_id}, {"tick": 3}]
    _, problems, _ = episode_binding(events, rows, audit=mine)
    assert problems == [], problems
    _, problems, _ = episode_binding(events, rows, audit=mine + [
        {"tick": 4, "episode_id": str(rec.episode_id - 1)}])
    assert any("audit.jsonl" in p_ and "not this episode" in p_
               for p_ in problems), problems
    # The logger the rails build writes the id into every record.
    from guardrail.audit import AuditLogger
    from guardrail.models import Action4D, State
    from guardrail.replay import rail_shield
    sh = rail_shield(policy, lookahead_s=5.0, dt=0.1)
    AuditLogger(run / "audit.jsonl", policy, episode_id=str(rec.episode_id)).log(
        41, sh.filter(State(x=4.0, y=15.0, up=15.0), Action4D(vx=6.0)))
    last = json.loads((run / "audit.jsonl").read_text(
        encoding="utf-8").splitlines()[-1])
    assert last["episode_id"] == str(rec.episode_id), last
    # Through the writer and the verifier.
    path = write_replay(run, policy, d / "ok.tar.gz", signer=None)
    assert verify_replay(path)[0]
    with (run / "audit.jsonl").open("a", encoding="utf-8") as f:
        f.write(json.dumps({"tick": 99, "policy_hash": policy.policy_hash,
                            "episode_id": str(rec.episode_id + 7)}))
        f.write(chr(10))
    try:
        write_replay(run, policy, d / "bad.tar.gz", signer=None)
    except ValueError as exc:
        assert "audit.jsonl" in str(exc), exc
    else:
        raise AssertionError("an audit record of another episode was bundled")
    shutil.rmtree(d, ignore_errors=True)


def test_an_unfinished_episode_is_refused():
    """No mission_end: the flight crashed or is still running."""
    from guardrail.replay import _main
    d = _tmp()
    run, policy, _, _ = _episode(d, hot=False, finish=False)
    try:
        write_replay(run, policy, d / "r.tar.gz", signer=None)
    except ValueError as exc:
        assert "mission_end" in str(exc), exc
    else:
        raise AssertionError("an unfinished episode was bundled")
    assert _main(["pack", str(run), "--keyless"]) == 1
    shutil.rmtree(d, ignore_errors=True)


def test_an_id_less_log_is_bound_by_its_file_time_and_said_to_be():
    """Rows written between 2026-10-06 and 2026-10-07 carry no episode id.
    Such a log is accepted only if it was written after the episode began,
    and the bundle says the binding is unverified."""
    d = _tmp()
    run, policy, rec, _ = _episode(d, hot=False, ids=False)
    path = write_replay(run, policy, d / "r.tar.gz", signer=None)
    assert read_replay(path)["index"]["log_binding"] == "unverified"
    ok, why = verify_replay(path)
    assert ok and any("episode binding UNVERIFIED" in w for w in why), why
    old = (rec.episode_id - int(3600e9)) / 1e9            # an hour earlier
    os.utime(run / "flight_log.jsonl", (old, old))
    try:
        write_replay(run, policy, d / "r2.tar.gz", signer=None)
    except ValueError as exc:
        assert "before this episode started" in str(exc), exc
    else:
        raise AssertionError("a log older than the episode was bundled")
    # The pymavlink rail wrote no mission_end before 2026-10-07: its stored,
    # id-less runs still bundle, and the missing end is said, not hidden.
    d2 = _tmp()
    run2, policy2, _, _ = _episode(d2, hot=False, ids=False, finish=False)
    ok, why = verify_replay(write_replay(run2, policy2, d2 / "r.tar.gz",
                                         signer=None))
    assert ok and any("no mission_end" in w for w in why), why
    shutil.rmtree(d, ignore_errors=True)
    shutil.rmtree(d2, ignore_errors=True)


def test_a_bound_episode_says_so_in_the_index():
    d = _tmp()
    run, policy, rec, rows = _episode(d)
    assert {r["episode_id"] for r in rows} == {rec.episode_id}
    got = read_replay(write_replay(run, policy, d / "r.tar.gz", signer=None))
    assert got["index"]["log_binding"] == "episode"
    assert got["index"]["episode_id"] == rec.episode_id
    shutil.rmtree(d, ignore_errors=True)


def test_an_event_payload_cannot_overwrite_kind_t_or_wall():
    """The first 2026-10-06 flight crashed in a ROS callback: the VLA's
    identity JSON has its own "kind" key. The fix stores colliding keys as
    data_<key>; overwriting them silently would lose the event's kind."""
    from guardrail.replay import EpisodeRecord
    d = _tmp()
    rec = EpisodeRecord(d / "ep", load_policy(SIM_POLICY), None)
    got = rec.event("vla_identity", t=1.5, kind="stub", wall="w", t_extra=2)
    assert got["kind"] == "vla_identity" and got["t"] == 1.5, got
    assert got["data_kind"] == "stub" and got["data_wall"] == "w", got
    last = json.loads((d / "ep" / "events.jsonl").read_text(
        encoding="utf-8").splitlines()[-1])
    assert last == got
    shutil.rmtree(d, ignore_errors=True)


def test_write_replay_refuses_a_policy_that_is_not_the_last_generation():
    """A flight that hot-applied a rule ended under generation 1; handing the
    writer generation 0 (a stale object, a re-loaded YAML) must be refused,
    or the bundle's policy would not be the one in force at the end."""
    d = _tmp()
    run, policy, _, _ = _episode(d)
    assert policy.generation == 1
    try:
        write_replay(run, load_policy(SIM_POLICY), d / "r.tar.gz", signer=None)
    except ValueError as exc:
        assert "not the last generation" in str(exc), exc
    else:
        raise AssertionError("generation 0 was bundled as the final policy")
    shutil.rmtree(d, ignore_errors=True)


def test_a_zone_applied_after_mission_start_goes_in_as_the_grant_allows():
    """The grant's mid-flight update model: only dynamic_nfz,
    time_window_switch and corridor_swap after mission start. A Shield that
    enforces it refuses the polygon_fence the --dynamic runs used to send;
    apply_zone sends the same zone as a dynamic_nfz there (same id, vertices,
    band, margin), the polygon_fence where the Shield predates the rule, and
    records the new generation either way."""
    import guardrail.shield as S
    from guardrail.models import Action4D, PolygonFence, State, XY
    from guardrail.replay import EpisodeRecord
    d = _tmp()
    policy = load_policy(SIM_POLICY)
    rec = EpisodeRecord(d / "ep", policy, None)
    sh = S.Shield(policy)
    sh.filter(State(x=0.0, y=0.0, up=15.0), Action4D(vx=1.0))   # mission started
    fence = PolygonFence(id="nfz-late", type="polygon_fence", margin_m=1.0,
                         vertices=[XY(x=40, y=-5), XY(x=50, y=-5),
                                   XY(x=50, y=5), XY(x=40, y=5)])
    rule = rec.apply_zone(sh, fence, t=1.0)
    assert policy.generation == 1 and rec.generations[-1]["generation"] == 1
    assert rule.id == "nfz-late"
    assert [(v.x, v.y) for v in rule.vertices] == [(v.x, v.y) for v in fence.vertices]
    if hasattr(S, "as_dynamic_nfz"):
        assert rule.type == "dynamic_nfz", rule.type
    else:
        assert rule is fence
    inside = State(x=45.0, y=0.0, up=15.0)
    assert any(v.rule_id == "nfz-late" for v in sh.state_is_unsafe(inside)), \
        "the applied zone is not enforced"
    shutil.rmtree(d, ignore_errors=True)


def test_a_kpi_grade_run_with_a_keyless_bundle_is_demoted():
    """Review 2026-10-07: require_signature existed but no KPI path asked for
    it, so a keyless (or downgraded) bundle still passed as evidence for a
    run is_kpi_grade had accepted. Both rails now pass the grade through
    kpi_evidence_grade."""
    from guardrail.replay import kpi_evidence_grade
    d = _tmp()
    run, policy, _, _ = _episode(d)
    keyless = write_replay(run, policy, d / "k.tar.gz", signer=None)
    graded, why = kpi_evidence_grade(True, [], keyless)
    assert graded is False and "cannot serve as KPI evidence" in why[0], why
    signed = write_replay(run, policy, d / "s.tar.gz", signer=SIGNER)
    assert kpi_evidence_grade(True, [], signed, TRUST) == (True, [])
    # A run that was not graded stays as it was, reasons untouched.
    assert kpi_evidence_grade(False, ["topology"], keyless) == (False, ["topology"])
    shutil.rmtree(d, ignore_errors=True)


def test_what_was_flown_is_what_the_escape_is_counted_on():
    """flown_fields, the one place both SITL rails decide what was flown.
    `none` (the autopilot's RTL holds the aircraft): nothing was flown, so
    nothing escaped - 250 of the 283 "escapes" of the shield-off GeoFence
    run were such ticks. `brake`: the zero action's check, not
    decision.emitted's. `pass`: the re-check (on) or the raw action (off)."""
    from guardrail.models import Action4D, State
    from guardrail.replay import flown_fields
    from guardrail.shield import Shield
    policy = load_policy(SIM_POLICY)
    sh = Shield(policy)
    approach = State(x=4.0, y=15.0, up=15.0)            # 3 m short of the zone
    d_ = sh.filter(approach, Action4D(vx=6.0))
    assert any(v.rule_id == "nfz-square" for v in d_.violations)
    off = flown_fields(sh, approach, d_, shield_on=False)
    assert off["flown"] and any(v["rule_id"] == "nfz-square"
                                for v in off["emitted_violations"])
    held = flown_fields(sh, approach, d_, shield_on=False, setpoint="none")
    assert held == {"flown": False, "emitted_violations": []}, held
    on = flown_fields(sh, approach, d_, shield_on=True)
    assert on["emitted_violations"] == [v.model_dump()
                                        for v in d_.emitted_violations]
    # Brake inside the zone: standing still there IS illegal, whatever the
    # repaired action was.
    inside = State(x=15.0, y=15.0, up=15.0)
    d_in = sh.filter(inside, Action4D(vx=1.0))
    br = flown_fields(sh, inside, d_in, shield_on=True, setpoint="brake")
    assert br["flown"] and any(v["rule_id"] == "nfz-square"
                               for v in br["emitted_violations"]), br
    try:
        flown_fields(sh, inside, d_in, shield_on=True, setpoint="hover")
    except ValueError:
        pass
    else:
        raise AssertionError("an unknown setpoint kind was accepted")
    # Scored: a control arm whose last ticks the autopilot flew is charged
    # only for the ticks the VLA's action was flown.
    rows = []
    for i in range(10):
        sp = "pass" if i < 3 else "none"
        rows.append({"t": i * 0.1, "violations": [v.model_dump()
                                                  for v in d_.violations],
                     "repairs": [], "raw": {}, "emitted": {}, "setpoint": sp,
                     **flown_fields(sh, approach, d_, shield_on=False,
                                    setpoint=sp)})
    k = compute(rows, rule_priorities(policy), {"shield": "off"})
    assert k["p0_escapes"] == 3, k["p0_escapes"]


def test_a_rail_runs_one_escalation_fsm_and_its_audit_records_that_one():
    """Since 2026-10-07 a Shield can run the escalation FSM inside filter()
    (`Shield(escalation=...)`, on by default), and AuditLogger.log records
    that FSM's verdict unless the caller hands it a record of its own. The
    SITL rails run THE FSM in the node, which sees the autopilot (mode, home
    reached, landed, a GeoFence takeover). With both running, audit.jsonl
    recorded the Shield's FSM and flight_log.jsonl the node's: they disagreed
    on 3 of 83 ticks of the flight ros2fix_on_dyn_nfz. `rail_shield` builds
    the rail's Shield without an FSM and `audit_tick` writes the node's."""
    from guardrail.audit import AuditLogger
    from guardrail.fsm import EscalationFSM, FSMConfig, tick_input_from_decision
    from guardrail.models import Action4D, State
    from guardrail.replay import audit_tick, rail_shield
    from guardrail.shield import Shield
    policy = load_policy(SIM_POLICY)
    sh = rail_shield(policy, lookahead_s=5.0, dt=0.1)
    assert (sh.lookahead_s, sh.dt) == (5.0, 0.1)
    assert getattr(sh, "fsm", None) is None, "the rail's Shield runs a second FSM"
    approach = State(x=4.0, y=15.0, up=15.0)            # 3 m short of the zone
    d_ = sh.filter(approach, Action4D(vx=6.0))
    assert d_.touched
    assert getattr(d_, "set_mode", None) is None
    assert getattr(d_, "fsm_record", None) is None
    # The node's FSM steps on the decision and changes state.
    fsm = EscalationFSM(FSMConfig())
    out = fsm.step(tick_input_from_decision(
        0.0, d_, policy, horizon_s=0.1, stop_illegal=sh.state_is_unsafe(approach)))
    assert out.transition, out.record
    path = Path(tempfile.mkdtemp()) / "audit.jsonl"
    audit = AuditLogger(path, policy)
    audit_tick(audit, 7, d_, fsm_out=out)
    rec = json.loads(path.read_text(encoding="utf-8").splitlines()[-1])
    assert (rec["tick"], rec["fsm_state_after"], rec["fsm_edge"]) == (
        7, out.state.value, out.edge), rec
    assert rec.get("fsm_set_mode") == out.set_mode, rec
    # The node's FSM refused its input: the audit says so on that tick.
    audit_tick(audit, 8, d_, fsm_fault="input refused")
    rec = json.loads(path.read_text(encoding="utf-8").splitlines()[-1])
    assert rec["tick"] == 8 and rec["fsm_fault"] == "input refused", rec
    # What the rails built before: the Shield's own FSM, whose verdict the
    # audit took when no record was handed over.
    if "escalation" in __import__("inspect").signature(Shield).parameters:
        both = Shield(policy, lookahead_s=5.0, dt=0.1)
        d2 = both.filter(approach, Action4D(vx=6.0))
        assert d2.fsm_record is not None, "the default Shield no longer runs an FSM"


if __name__ == "__main__":
    # A test that short-circuits on a missing fixture must NOT print PASS. On a
    # clean clone demo/out/ is gitignored, so 7 of these 8 returned immediately
    # and the suite reported "8/8 passed" for a run that asserted almost
    # nothing - the same shape as every other defect this week.
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (fixture missing)")
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
