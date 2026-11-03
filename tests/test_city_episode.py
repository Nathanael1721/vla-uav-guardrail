"""
demo/follow_vlm.city_episode - the city rail's episode record and CSP.

Run either way:
    pytest tests/test_city_episode.py -v
    python tests/test_city_episode.py

Until 2026-10-28 a city follow wrote no CSP and no episode events (audit card
WP2-11), while the SITL and perception rails wrote both through
guardrail.replay.EpisodeRecord. city_episode gives the city rail the same
record: events.jsonl from episode_start to mission_end, policy_g0.json and
the typed CSP csp_g0.json compiled for the follow's mission at the Shield's
own horizon, with no rule clock (the city Shield has none). No simulator,
no GPU: the record is written from a real policy into a temporary folder.
"""
from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import load_policy                                   # noqa: E402
from guardrail.replay import verify_replay, write_replay            # noqa: E402

POLICY = ROOT / "policies" / "follow_car_citylife_nfz.yaml"


def _fv():
    import follow_vlm
    return follow_vlm


def _events(d: Path) -> list[dict]:
    return [json.loads(line) for line in (d / "events.jsonl").read_text(encoding="utf-8").splitlines()]


def test_the_record_writes_the_policy_the_csp_and_the_mission():
    fv = _fv()
    pol = load_policy(POLICY)
    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        (d / "flight_log.jsonl").write_text("{}\n", encoding="utf-8")   # a flight already writing
        rec = fv.city_episode(d, pol, 3.0, (12.5, -40.0), "a red car", 8.0, 5.0)
        assert (d / "flight_log.jsonl").is_file(), "rotate=False: the flight's own files stay"
        assert (d / "policy_g0.json").read_bytes() == pol.canonical_bytes()
        csp = json.loads((d / "csp_g0.json").read_text(encoding="utf-8"))
        assert csp["lookahead_s"] == 3.0, csp["lookahead_s"]
        assert rec.generations[0]["csp"] is True, rec.generations
        assert rec.generations[0]["policy_hash"] == pol.policy_hash
        ev = _events(d)
        assert [e["kind"] for e in ev] == ["episode_start", "policy_generation", "mission"], ev
        assert ev[0]["rule_clock"] == {"kind": "none"}, ev[0]
        assert ev[0]["policy_hash"] == pol.policy_hash
        assert ev[1]["csp"] is True and ev[1]["reason"] == "take-off", ev[1]
        m = ev[2]
        assert m["task_text"] == "follow a red car" and m["start"] == [12.5, -40.0], m
        assert m["target_proxy"] == "start" and m["speed_pref_mps"] == 5.0, m
        rec.end("the mission window closed", t=420.3)
        assert _events(d)[-1]["kind"] == "mission_end"


def test_every_file_the_record_writes_is_cleared_for_a_reused_tag():
    """fly() removes CITY_EPISODE_FILES (plus the log and detections) before
    take-off, so a re-flown tag cannot bundle the last flight's events or
    generations with its own."""
    import fnmatch
    fv = _fv()
    pol = load_policy(POLICY)
    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        fv.city_episode(d, pol, 3.0, (0.0, 0.0), "a red car", 8.0, 5.0)
        written = sorted(p.name for p in d.iterdir())
        assert written, written
        for name in written:
            assert any(fnmatch.fnmatch(name, pat) for pat in fv.CITY_EPISODE_FILES), name


def test_a_finished_episode_with_ids_on_every_row_bundles_bound_to_it():
    """The replay bundle binds the log to the episode when every row carries
    its id and the episode ended; without mission_end it is refused."""
    fv = _fv()
    pol = load_policy(POLICY)
    for ended, want_ok in ((True, True), (False, False)):
        with tempfile.TemporaryDirectory() as td:
            d = Path(td)
            rec = fv.city_episode(d, pol, 3.0, (0.0, 0.0), "a red car", 8.0, 5.0)
            rows = [{"episode_id": rec.episode_id, "t": 0.1 * i, "tick": i, "x": 0.0,
                     "y": float(i), "up": 8.0, "policy_hash": pol.policy_hash,
                     "violations": [], "repairs": []} for i in range(5)]
            with open(d / "flight_log.jsonl", "w", encoding="utf-8", newline="\n") as fh:
                for r in rows:
                    fh.write(json.dumps(r) + "\n")
            if ended:
                rec.end("the mission window closed", t=0.5)
            try:
                path = write_replay(d, pol, d / "t.replay.tar.gz", signer=None)
            except ValueError as exc:
                assert not want_ok, exc
                assert "mission_end" in str(exc), exc
                continue
            ok, why = verify_replay(path)
            if want_ok:
                assert ok, why
                assert not any("episode" in w and "not this" in w for w in why), why
                assert not any("mission_end" in w for w in why), why
            else:
                assert not ok and any("mission_end" in w for w in why), why


def test_metrics_and_kpi_carry_the_episode_id_so_the_flight_bundles_itself():
    """A city flight writes metrics.json and kpi.json beside its log. Until
    2026-11-03 only the rows carried the episode id, so write_replay refused
    every city flight ("metrics.json carries no episode id while the log
    does") and none bundled itself (agx_follow_01, the 3 Nov forecast
    flights). With episode_fields in all three the bundle binds; without it
    in the two documents it is refused as before."""
    fv = _fv()
    pol = load_policy(POLICY)
    assert fv.episode_fields(None) == {}
    for stamped in (True, False):
        with tempfile.TemporaryDirectory() as td:
            d = Path(td)
            rec = fv.city_episode(d, pol, 5.0, (0.0, 0.0), "a red car", 8.0, 5.0)
            ids = fv.episode_fields(rec)
            assert ids == {"episode_id": rec.episode_id}
            rows = [{**ids, "t": 0.1 * i, "tick": i, "x": 0.0, "y": float(i),
                     "up": 8.0, "policy_hash": pol.policy_hash,
                     "violations": [], "repairs": []} for i in range(5)]
            with open(d / "flight_log.jsonl", "w", encoding="utf-8", newline="\n") as fh:
                for r in rows:
                    fh.write(json.dumps(r) + "\n")
            rec.end("the mission window closed", t=0.5)
            (d / "manifest.json").write_text(json.dumps(fv.flight_manifest(pol.policy_hash, 1)),
                                             encoding="utf-8")
            doc_ids = ids if stamped else {}
            (d / "metrics.json").write_text(json.dumps({"tag": "t", **doc_ids, "ticks": 5}),
                                            encoding="utf-8")
            (d / "kpi.json").write_text(json.dumps({"ticks": 5, **doc_ids}), encoding="utf-8")
            try:
                path = write_replay(d, pol, d / "t.replay.tar.gz", signer=None)
            except ValueError as exc:
                assert not stamped, exc
                assert "metrics.json carries no episode id" in str(exc), exc
                assert "kpi.json carries no episode id" in str(exc), exc
                continue
            assert stamped, "a bundle without the id in metrics and kpi was written"
            ok, why = verify_replay(path)
            assert not any("carries no episode id" in w for w in why), why


# --------------------------------------------------------------------- runner

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
