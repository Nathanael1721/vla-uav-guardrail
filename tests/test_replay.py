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
"""
import gzip
import io
import json
import shutil
import sys
import tarfile
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

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
    path = write_replay(RUN, policy, d / "r.tar.gz")
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
    a = write_replay(RUN, policy, d / "a.tar.gz").read_bytes()
    b = write_replay(RUN, policy, d / "b_different_name.tar.gz").read_bytes()
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
        write_replay(other, load_policy(POLICY_FILE), d / "wrong.tar.gz")
    except ValueError as exc:
        assert "flown under policy" in str(exc), exc
    else:
        raise AssertionError("a foreign policy was accepted")
    shutil.rmtree(d, ignore_errors=True)


def test_one_flipped_byte_in_the_log_is_caught_and_named():
    if not _have_fixture():
        return SKIP
    d = _tmp()
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz")
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
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz")
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


def test_a_CONSISTENT_forgery_passes_because_the_signature_has_no_key():
    """The honest limit, asserted rather than implied.

    An earlier version of the test above was named "...still fails" and its
    docstring said "the signature covers the index, so consistency is not
    enough". That is false: signature.txt is sha256(replay.json) with no key, so
    a forger recomputes it in one line. The old test passed only because its
    helper replaced exactly one member and forgot the signature.

    This asserts the true property, so the suite stops vouching for protection
    the format does not provide. It will start failing the day a real key is
    introduced, which is exactly when someone should look at it."""
    if not _have_fixture():
        return SKIP
    d = _tmp()
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz")
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
    assert verify_replay(bad)[0] is True, "a consistent forgery was refused"


def test_an_undeclared_member_is_refused():
    """The digest loop used to iterate the INDEX, so a member present in the
    archive but absent from the index was neither digested nor rejected - while
    the docstring said "check every digest"."""
    if not _have_fixture():
        return SKIP
    d = _tmp()
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz")
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
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz")
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
    good = write_replay(RUN, load_policy(POLICY_FILE), d / "good.tar.gz")
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
        write_replay(run, load_policy(POLICY_FILE), d / "x.tar.gz")
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
    path = write_replay(run, load_policy(POLICY_FILE), d / "u.tar.gz")
    got = read_replay(path)
    assert got["index"]["policy_binding"] == "unverified", got["index"]
    ok, why = verify_replay(path)
    assert ok and any("UNVERIFIED" in w for w in why), (ok, why)
    shutil.rmtree(d, ignore_errors=True)


def test_a_bundle_re_derives_its_own_kpis():
    """The property the word 'replayable' is claiming."""
    if not _have_fixture() or not (RUN / "kpi.json").is_file():
        return SKIP
    d = _tmp()
    path = write_replay(RUN, load_policy(POLICY_FILE), d / "r.tar.gz")
    ok, why = verify_replay(path)
    assert ok, why
    assert why == [], why
    shutil.rmtree(d, ignore_errors=True)


def test_an_episode_with_no_log_is_refused_rather_than_packaged_empty():
    d = _tmp()
    (d / "empty_run").mkdir()
    try:
        write_replay(d / "empty_run", load_policy(POLICY_FILE), d / "r.tar.gz")
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
    path = write_replay(run, load_policy(POLICY_FILE), d / "r.tar.gz")
    ok, why = verify_replay(path)
    # any(), not why[0]: an unscored episode now also reports that its policy
    # binding is unverified, and pinning the ORDER of honest notes would make
    # this test fail for a reason that has nothing to do with what it checks.
    assert ok and any("no kpi.json" in w for w in why), (ok, why)
    shutil.rmtree(d, ignore_errors=True)


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
