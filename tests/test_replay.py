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

from guardrail.models import load_policy                       # noqa: E402
from guardrail.replay import (EPISODE, INDEX_NAME,             # noqa: E402
                              read_replay, verify_replay, write_replay)

RUN = ROOT / "demo" / "out" / "retarget_demo2"
POLICY = ROOT / "policies" / "follow_pedestrian.yaml"


def _have_fixture() -> bool:
    return (RUN / "flight_log.jsonl").is_file() and POLICY.is_file()


def _tmp() -> Path:
    return Path(tempfile.mkdtemp(prefix="replay_"))


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
        return
    d = _tmp()
    policy = load_policy(POLICY)
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
        return
    d = _tmp()
    policy = load_policy(POLICY)
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
        return
    other = ROOT / "demo" / "out" / "adopt_check"
    if not (other / "manifest.json").is_file():
        return
    d = _tmp()
    try:
        write_replay(other, load_policy(POLICY), d / "wrong.tar.gz")
    except ValueError as exc:
        assert "flown under policy" in str(exc), exc
    else:
        raise AssertionError("a foreign policy was accepted")
    shutil.rmtree(d, ignore_errors=True)


def test_one_flipped_byte_in_the_log_is_caught_and_named():
    if not _have_fixture():
        return
    d = _tmp()
    good = write_replay(RUN, load_policy(POLICY), d / "good.tar.gz")
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


def test_rewriting_the_index_to_match_a_forged_member_still_fails():
    """The digests alone would let a forger who edits both through. The
    signature covers the index, so consistency is not enough."""
    if not _have_fixture():
        return
    d = _tmp()
    good = write_replay(RUN, load_policy(POLICY), d / "good.tar.gz")
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


def test_a_bundle_re_derives_its_own_kpis():
    """The property the word 'replayable' is claiming."""
    if not _have_fixture() or not (RUN / "kpi.json").is_file():
        return
    d = _tmp()
    path = write_replay(RUN, load_policy(POLICY), d / "r.tar.gz")
    ok, why = verify_replay(path)
    assert ok, why
    assert why == [], why
    shutil.rmtree(d, ignore_errors=True)


def test_an_episode_with_no_log_is_refused_rather_than_packaged_empty():
    d = _tmp()
    (d / "empty_run").mkdir()
    try:
        write_replay(d / "empty_run", load_policy(POLICY), d / "r.tar.gz")
    except ValueError as exc:
        assert "nothing to replay" in str(exc), exc
    else:
        raise AssertionError("an episode with no log was packaged")
    shutil.rmtree(d, ignore_errors=True)


def test_an_unscored_episode_bundles_and_says_so_instead_of_passing_quietly():
    if not _have_fixture():
        return
    d = _tmp()
    run = d / "unscored"
    run.mkdir()
    shutil.copy(RUN / "flight_log.jsonl", run / "flight_log.jsonl")
    path = write_replay(run, load_policy(POLICY), d / "r.tar.gz")
    ok, why = verify_replay(path)
    assert ok and why and "no kpi.json" in why[0], (ok, why)
    shutil.rmtree(d, ignore_errors=True)


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
