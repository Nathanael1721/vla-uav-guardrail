"""Resume, shards and the merge of an unattended headless sweep (WP4-06,
2026-11-17).

Run either way:
    pytest tests/test_sweep_resume.py -v
    python tests/test_sweep_resume.py

The broad profile (~15 000 episodes, hours on the desktop) can only run
unattended if a kill, a reboot or a split into shards cannot change its
result. So the tests below hold one line: a sweep assembled from checkpoints
is the sweep one uninterrupted run writes. A replay of every checkpoint must
give the same results document; a run killed part-way and resumed, and the
same episodes flown as two shards and merged, must give the same results as
one fresh run (up to the measured sim_speedup, which is wall time); each
episode is flown once; and a checkpoint folder of another sweep, a torn
checkpoint, or a merge with episodes missing must never be merged silently.
"""
import contextlib
import io
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

import sweep_scenarios as S                                       # noqa: E402

# Five cells of the smoke profile on both forecast arms (10 episodes): a
# Shield-off control, a known failure, two tracked cells and a plain pass,
# and the 3 s comparison arm, so every counter and sink of the loop replays.
SUBSET = ["--profile", "smoke", "--only", "nfz-head-on-control", "corridor-swap-shifted",
          "gps-noise-beyond-margin", "wind-gusts", "speed-cap",
          "--forecast-arms", "grant", "3s"]
SUBSET_EPISODES = 10

# Keys that are wall time or the invocation, never the result.
VOLATILE = {"_runtime_s", "_command", "_resume", "sim_speedup"}


def _norm(node):
    if isinstance(node, dict):
        return {k: _norm(v) for k, v in node.items() if k not in VOLATILE}
    if isinstance(node, list):
        return [_norm(v) for v in node]
    return node


def _main(args):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = S.main(args)
    return rc, buf.getvalue()


def _doc(p: Path) -> dict:
    return json.loads(p.read_text(encoding="utf-8"))


class _Counting:
    """S.run_episode, counted; optionally interrupted on the n-th call (a
    KeyboardInterrupt is what Ctrl+C or a console close raises, and like a
    kill it is not caught by the loop's `except Exception`)."""

    def __init__(self, interrupt_on: int | None = None):
        self.calls = 0
        self.interrupt_on = interrupt_on
        self.real = S.run_episode

    def __call__(self, *a, **k):
        self.calls += 1
        if self.interrupt_on is not None and self.calls == self.interrupt_on:
            raise KeyboardInterrupt("simulated kill")
        return self.real(*a, **k)

    def __enter__(self):
        S.run_episode = self
        return self

    def __exit__(self, *exc):
        S.run_episode = self.real
        return False


def test_shard_parsing_and_partition():
    assert S.parse_shard(None) is None and S.parse_shard("2/4") == (2, 4)
    for bad in ("0/4", "5/4", "1/0", "a/b", "1-4", "1/"):
        try:
            S.parse_shard(bad)
        except ValueError:
            continue
        raise AssertionError(f"--shard {bad!r} accepted")
    n = 4
    owners = [[i for i in range(1, n + 1) if S.in_shard(k, (i, n))] for k in range(23)]
    assert all(len(o) == 1 for o in owners), owners          # each episode in one shard


def test_a_full_replay_writes_the_run_it_replays():
    """The smoke profile flown once with checkpoints, then merged from them
    alone (--merge-only, nothing flown): the results document is the same in
    every field but the invocation and runtime, and so is the KPI report."""
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        ck, runs = td / "ck", td / "runs"
        common = ["--profile", "smoke", "--resume", str(ck), "--runs", str(runs)]
        rc1, _ = _main(common + ["--out", str(td / "a.json")])
        md1 = (runs / "kpi_report.md").read_text(encoding="utf-8")
        with _Counting() as cnt:
            rc2, log = _main(common + ["--out", str(td / "b.json"), "--merge-only"])
        md2 = (runs / "kpi_report.md").read_text(encoding="utf-8")
        a, b = _doc(td / "a.json"), _doc(td / "b.json")
    assert rc1 == rc2 and cnt.calls == 0, (rc1, rc2, cnt.calls)
    assert len(a["results"]) == 64 and a["_resume"]["episodes_flown_by_this_process"] == 64
    assert b["_resume"]["episodes_replayed"] == 64 and "64 episodes done" in log, log[:400]
    # Wall time included: the replayed records are the ones the run wrote.
    drop = {"_runtime_s", "_command", "_resume"}
    assert {k: v for k, v in a.items() if k not in drop} == \
        {k: v for k, v in b.items() if k not in drop}
    # The report, but for the command, the merge line and this process's runtime.
    def strip(md):
        return [ln for ln in md.splitlines() if "sweep_scenarios.py" not in ln
                and not ln.startswith("Merged from") and "; runtime " not in ln]
    assert strip(md1) == strip(md2) and "Merged from" in md2


def test_a_killed_run_resumes_flying_only_what_was_left():
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        rc0, _ = _main(SUBSET + ["--no-bundles", "--out", str(td / "fresh.json")])
        ck = td / "ck"
        args = SUBSET + ["--no-bundles", "--resume", str(ck), "--out", str(td / "r.json")]
        with _Counting(interrupt_on=4) as cnt:
            try:
                _main(args)
            except KeyboardInterrupt:
                pass
            else:
                raise AssertionError("the simulated kill did not stop the run")
        saved = sorted((ck / "episodes").glob("*.json"))
        assert len(saved) == 3 and not (td / "r.json").exists(), saved
        with _Counting() as cnt2:
            rc1, log = _main(args)
        fresh, resumed = _doc(td / "fresh.json"), _doc(td / "r.json")
    assert cnt.calls == 4 and cnt2.calls == SUBSET_EPISODES - 3, (cnt.calls, cnt2.calls)
    assert "3 of 10 episodes done, 7 to fly" in log, log[:400]
    assert resumed["_resume"]["episodes_replayed"] == 3
    assert resumed["_resume"]["episodes_flown_by_this_process"] == 7
    assert rc0 == rc1 and _norm(fresh) == _norm(resumed)
    assert resumed["_forecast_comparison"] is not None and resumed["_counts"]["known_failure"]


def test_two_shards_merge_into_the_one_run():
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        rc0, _ = _main(SUBSET + ["--no-bundles", "--out", str(td / "fresh.json")])
        ck = td / "ck"
        base = SUBSET + ["--no-bundles", "--resume", str(ck), "--out", str(td / "m.json")]
        rc, log = _main(base + ["--merge-only"])
        assert rc == 2 and "10 episode(s) have no checkpoint" in log, log[-400:]
        calls = 0
        for i in (1, 2):
            with _Counting() as cnt:
                rc, log = _main(base + ["--shard", f"{i}/2"])
            calls += cnt.calls
            assert rc == 0 and f"shard {i}/2" in log, log[-400:]
            assert not (td / "m.json").exists()                # a shard writes no sweep
        with _Counting() as cnt:
            rc_m, _ = _main(base + ["--merge-only"])
        fresh, merged = _doc(td / "fresh.json"), _doc(td / "m.json")
    assert calls == SUBSET_EPISODES and cnt.calls == 0, (calls, cnt.calls)
    assert rc_m == rc0 and _norm(fresh) == _norm(merged)


def test_another_sweeps_checkpoints_are_refused():
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        ck = td / "ck"
        one = ["--profile", "smoke", "--only", "speed-cap", "--no-bundles",
               "--resume", str(ck), "--out", str(td / "o.json")]
        assert _main(one)[0] == 0
        for change in (["--seed", "7"], ["--wind-model", "open_loop"],
                       ["--recovery-law", "lookahead"], ["--keep-logs"]):
            rc, log = _main(one + change)
            assert rc == 2 and "another sweep" in log, (change, log[-300:])
        # Checkpoints written at another code revision, or from another
        # library file, are another sweep too.
        hp = ck / S.CHECKPOINT_HEADER
        orig = hp.read_text(encoding="utf-8")
        for field, value in (("code_revision", "0123456789ab"), ("library_sha256", "0" * 64)):
            h = json.loads(orig)
            h["fingerprint"][field] = value
            hp.write_text(json.dumps(h), encoding="utf-8")
            rc, log = _main(one)
            assert rc == 2 and field in log, (field, log[-300:])
        hp.write_text(orig, encoding="utf-8")
        assert _main(one)[0] == 0


def test_a_torn_checkpoint_is_flown_again_never_trusted():
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        ck = td / "ck"
        one = ["--profile", "smoke", "--only", "speed-cap", "corridor-along", "--no-bundles",
               "--resume", str(ck), "--out", str(td / "o.json")]
        assert _main(one)[0] == 0
        first = _doc(td / "o.json")
        victim = sorted((ck / "episodes").glob("*.json"))[0]
        victim.write_text('{"format": 1, "key": "', encoding="utf-8")   # torn
        (ck / "episodes" / "stray.json.123.tmp").write_text("x", encoding="utf-8")
        with _Counting() as cnt:
            rc, log = _main(one)
        again = _doc(td / "o.json")
    assert rc == 0 and cnt.calls == 1 and "unreadable checkpoint" in log, (cnt.calls, log)
    assert _norm(first) == _norm(again)


def test_the_flags_are_refused_where_they_mean_nothing():
    for argv, why in ((["--shard", "1/2"], "--resume DIR"),
                      (["--merge-only"], "--resume DIR"),
                      (["--resume", "x", "--shard", "1/2", "--merge-only"], "pick one"),
                      (["--resume", "x", "--backend", "sitl"], "headless"),
                      (["--resume", "x", "--publish"], "headless"),
                      (["--resume", "x", "--shard", "3/2"], "1 <= I <= N")):
        rc, log = _main(argv)
        assert rc == 2 and why in log, (argv, log)


def test_the_broad_report_says_it_is_outside_the_kpis():
    sys.path.insert(0, str(ROOT / "tools"))
    import kpi_report
    doc = {"_not_kpi_grade": "headless", "_scope": {"grant_profile_kpi_bearing": False,
                                                    "profile": "broad"},
           "_resume": {"checkpoints": "ck", "episodes_replayed": 5,
                       "episodes_flown_by_this_process": 0, "episode_wall_s_total": 8.0,
                       "s_per_episode": 1.6}}
    md = kpi_report.render_sweep_markdown(doc)
    assert "not part of the final KPI" in md and "reproduces their scope" not in md
    assert "5 replayed" in md
    doc["_scope"]["grant_profile_kpi_bearing"] = True
    md = kpi_report.render_sweep_markdown(doc)
    assert "reproduces their scope" in md


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
