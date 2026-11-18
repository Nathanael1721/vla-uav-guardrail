"""
WP1-14: the bundle census - every bundle under bundles/ with the form its
points are stored in, and the WGS84 bundle each legacy task flies from.

    python tests/test_bundle_census.py      (no pytest needed)

tools/bundle_census.py replaces a one-off listing ("13 of the 41 lab bundles
are legacy metre-only") with a check anyone can run. What is checked here,
on bundles built in a temporary folder (bundles/ is gitignored, so CI has
none but the tracked replay packs):

  * each bundle's row: kind (policy / replay), point_form, frame (stated /
    derived / none), legacy, flight_ready, signature; an unreadable file is
    listed with its error, never dropped;
  * a task (a policy_id with a bundle that is not flight-ready: a legacy
    metre bundle, refused for flight since 2026-11-18, or a point-free one
    with no frame origin, on which every spawn is refused) names the bundle a re-flight takes: the current policies/
    version's, when it is on disk and flight-ready; else the newest
    flight-ready successor, and --check fails naming the build command;
  * the CLI's --json and --check;
  * the committed census of the lab's bundles/
    (docs/data/bundle_census_20261118/) says what its rows say: 13 legacy
    metre-only bundles and 4 frameless point-free ones, every task with a
    flight-ready bundle at its current version (after the two built for this
    row) and no problem; and when the lab checkout's bundles/ is on this
    machine, today's census of it agrees with the committed rows for every
    bundle both list, and its --check problems are reported: none, or
    exactly the two current-version bundles built for this row that the lab
    checkout does not hold yet (an integrator step; then the test SKIPs and
    says so rather than passing).
"""
from __future__ import annotations

import io
import json
import shutil
import sys
import tempfile
from contextlib import redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import bundle_census as BC                                          # noqa: E402
from guardrail.bundle import as_version, write_bundle               # noqa: E402
from guardrail.models import (POINTS_LEGACY_METRES, POINTS_NONE,    # noqa: E402
                              POINTS_WGS84, Policy, load_policy)

NFZ3B = ROOT / "policies" / "follow_car_citylife_nfz3b.yaml"
EVIDENCE = ROOT / "docs" / "data" / "bundle_census_20261118"
LAB_BUNDLES = ROOT.parents[2] / "bundles" if ROOT.parent.name == "worktrees" else \
    ROOT / "bundles"
RULE = {"scope": "mission", "constraint_type": "hard", "priority": "P0",
        "violation_action": "project_fix"}


def _fixture(d: Path, with_successor: bool = True) -> dict:
    """A bundles/ folder: nfz3b's legacy metre bundle at 0.1.1, its WGS84
    bundle at the current version, a frameless point-free bundle (caps
    0.0.1) and its framed successor (0.0.2), a tracked replay pack and a
    corrupt file. Returns the `current` map it is judged against (nfz3b at
    its policies/ version, caps at 0.0.2)."""
    cur = load_policy(NFZ3B, runtime=False)
    assert cur.point_form == POINTS_WGS84
    legacy = as_version(cur.metre_form(), "0.1.1")
    assert legacy.point_form == POINTS_LEGACY_METRES and legacy.frame_origin is None
    write_bundle(legacy, d / f"{cur.policy_id}-v0.1.1.tar.gz", signer=None,
                 allow_legacy_metres=True, issued_at="2026-10-28T00:00:00Z")
    if with_successor:
        write_bundle(cur, d / f"{cur.policy_id}-v{cur.version}.tar.gz", signer=None,
                     issued_at="2026-11-18T00:00:00Z")
    caps = Policy.model_validate({"policy_id": "caps", "version": "0.0.1", "constraints": [
        {"id": "alt", "type": "altitude_envelope", **RULE, "alt_min_m": 2, "alt_max_m": 60}]})
    write_bundle(caps, d / "caps-v0.0.1.tar.gz", signer=None)
    framed = Policy.model_validate({**caps.model_dump(mode="json", exclude_none=True),
                                    "version": "0.0.2",
                                    "origin": {"lat": 25.04245, "lon": 121.5314}})
    write_bundle(framed, d / "caps-v0.0.2.tar.gz", signer=None)
    replay = sorted((ROOT / "bundles").glob("*.replay.tar.gz"))
    if replay:
        shutil.copy(replay[0], d / replay[0].name)
    (d / "broken-v0.0.1.tar.gz").write_bytes(b"not a bundle")
    return {cur.policy_id: {"version": cur.version, "file": "policies/" + NFZ3B.name,
                            "point_form": cur.point_form},
            "caps": {"version": "0.0.2", "file": "x", "point_form": POINTS_NONE}}


def test_each_bundle_is_listed_with_its_form():
    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        current = _fixture(d)
        c = BC.census(d, current)
        rows = {r["file"]: r for r in c["bundles"]}
        cur = load_policy(NFZ3B, runtime=False)
        old = rows[f"{cur.policy_id}-v0.1.1.tar.gz"]
        new = rows[f"{cur.policy_id}-v{cur.version}.tar.gz"]
        assert old["legacy"] and old["point_form"] == POINTS_LEGACY_METRES, old
        assert old["frame"] == "none" and old["frame_origin"] is None, old
        assert not new["legacy"] and new["point_form"] == POINTS_WGS84, new
        assert new["frame"] == "stated" and new["policy_hash"] == cur.policy_hash, new
        assert rows["caps-v0.0.1.tar.gz"]["frame"] == "none"
        assert rows["caps-v0.0.1.tar.gz"]["legacy"] is False
        assert rows["caps-v0.0.1.tar.gz"]["flight_ready"] is False
        assert rows["caps-v0.0.2.tar.gz"]["flight_ready"] is True
        assert not old["flight_ready"] and new["flight_ready"], (old, new)
        assert "error" in rows["broken-v0.0.1.tar.gz"]
        assert all(r["kind"] == "replay" for n, r in rows.items()
                   if n.endswith(".replay.tar.gz"))
        s = c["summary"]
        assert s["legacy_metre_bundles"] == 1 and s["unreadable"] == 1, s
        assert s["policy_bundles"] == 5 and s["frameless_bundles"] == 2, s
        assert s["not_flight_ready_bundles"] == 2, s
        tasks = {t["policy_id"]: t for t in c["legacy_tasks"]}
        assert sorted(tasks) == ["caps", cur.policy_id], sorted(tasks)
        caps = tasks["caps"]
        assert caps["legacy_bundles"] == [] and \
            caps["not_flight_ready"] == ["caps-v0.0.1.tar.gz"], caps
        assert caps["refly_from"] == "caps-v0.0.2.tar.gz" == caps["current_bundle"], caps
        task = tasks[cur.policy_id]
        assert task["not_flight_ready"] == task["legacy_bundles"] == [old["file"]], task
        assert task["refly_from"] == new["file"] == task["current_bundle"], task
        assert task["current_bundle_on_disk"] and task["successor"] == new["file"], task
        # the only problem is the unreadable file
        assert len(c["problems"]) == 1 and "broken" in c["problems"][0], c["problems"]


def test_a_legacy_task_without_its_current_bundle_is_a_problem():
    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        current = _fixture(d, with_successor=False)
        (d / "broken-v0.0.1.tar.gz").unlink()
        c = BC.census(d, current)
        task = next(t for t in c["legacy_tasks"] if t["policy_id"] != "caps")
        assert task["refly_from"] is None and not task["current_bundle_on_disk"], task
        (p,) = c["problems"]
        assert "build_flight_bundles.py follow_car_citylife_nfz3b" in p, p
        # --check exits 1 and --json writes the same census
        out = d / "census.json"
        buf = io.StringIO()
        with redirect_stdout(buf):
            rc = BC.main([str(d), "--json", str(out), "--check"])
        assert rc == 1 and "PROBLEM" in buf.getvalue(), buf.getvalue()
        back = json.loads(out.read_text(encoding="utf-8"))
        assert back["problems"] and back["summary"]["legacy_metre_bundles"] == 1
        # an older WGS84 successor is offered when the current one is missing
        cur = load_policy(NFZ3B, runtime=False)
        mid = as_version(cur, "0.1.2")
        write_bundle(mid, d / f"{cur.policy_id}-v0.1.2.tar.gz", signer=None)
        c = BC.census(d, current)
        task = next(t for t in c["legacy_tasks"] if t["policy_id"] != "caps")
        assert task["refly_from"] == f"{cur.policy_id}-v0.1.2.tar.gz", task
        assert c["problems"], "the current version's bundle is still missing"


def _evidence() -> dict:
    return json.loads((EVIDENCE / "census_after.json").read_text(encoding="utf-8"))


def test_the_committed_lab_census_says_what_its_rows_say():
    before = json.loads((EVIDENCE / "census_before.json").read_text(encoding="utf-8"))
    after = _evidence()
    for c in (before, after):
        pol = [r for r in c["bundles"] if r["kind"] == "policy" and "error" not in r]
        assert c["summary"]["legacy_metre_bundles"] == sum(r["legacy"] for r in pol)
        assert c["summary"]["legacy_metre_bundles"] == 13, c["summary"]
        # 13 legacy + 4 point-free bundles with no frame origin: none is
        # flight-ready, and every one belongs to a task.
        unready = {r["file"] for r in pol if not r["flight_ready"]}
        assert len(unready) == c["summary"]["not_flight_ready_bundles"] == 17, c["summary"]
        assert unready == {f for t in c["legacy_tasks"] for f in t["not_flight_ready"]}
        assert len(c["legacy_tasks"]) == 10, len(c["legacy_tasks"])
    # before: two legacy tasks had no bundle at their current version
    missing = sorted(p.split(":")[0] for p in before["problems"])
    assert missing == ["follow-car-citylife-nfz3", "follow-car-citylife-nfz3b"], missing
    # after: every legacy task re-flies from a WGS84 bundle at its current version
    assert after["problems"] == [], after["problems"]
    rows = {r["file"]: r for r in after["bundles"]}
    for t in after["legacy_tasks"]:
        r = rows[t["refly_from"]]
        assert r["point_form"] in (POINTS_WGS84, POINTS_NONE) and r["flight_ready"], t
        assert r["version"] == t["current_version"] and r["frame"] == "stated", t
        assert r["signature"] == "verified", (t, r["signature"])
    ids = {t["policy_id"]: t["refly_from"] for t in after["legacy_tasks"]}
    assert ids["follow-car-citylife-nfz3b"] == "follow-car-citylife-nfz3b-v0.1.3.tar.gz"
    assert ids["follow-car-citylife-nfz"] == "follow-car-citylife-nfz-v0.1.3.tar.gz"
    assert ids["follow-car-citylife"] == "follow-car-citylife-v0.1.3.tar.gz"
    built = json.loads((EVIDENCE / "built.json").read_text(encoding="utf-8"))
    for b in built["bundles"]:
        r = rows[b["file"]]
        assert r["policy_hash"] == b["policy_hash"] == \
            load_policy(ROOT / b["policy"], runtime=False).policy_hash, (b, r)


def test_todays_lab_census_agrees_with_the_committed_one():
    if not LAB_BUNDLES.is_dir() or not any(LAB_BUNDLES.glob("*-v*.tar.gz")):
        return "SKIP", "the lab checkout's bundles/ is not here"
    now = BC.census(LAB_BUNDLES)
    committed = {r["file"]: r for r in _evidence()["bundles"]}
    keys = ("kind", "policy_id", "version", "policy_hash", "point_form", "frame",
            "legacy", "flight_ready")
    n = 0
    for r in now["bundles"]:
        c = committed.get(r["file"])
        if c is None or "error" in r or "error" in c:
            continue
        assert {k: r.get(k) for k in keys} == {k: c.get(k) for k in keys}, r["file"]
        n += 1
    assert n >= 40, n
    # --check on the lab folder: no problem, or exactly the two bundles built
    # for this row (built.json) that the lab checkout does not hold yet.
    built = json.loads((EVIDENCE / "built.json").read_text(encoding="utf-8"))
    missing = {b["file"] for b in built["bundles"]
               if not (LAB_BUNDLES / b["file"]).is_file()}
    stuck = sorted(p.split(":")[0] for p in now["problems"])
    assert stuck == sorted(f.rsplit("-v", 1)[0] for f in missing), (stuck, missing)
    if missing:
        return "SKIP", (f"the lab checkout's census --check has {len(stuck)} problem(s): "
                        f"{', '.join(sorted(missing))} not built there yet (built.json "
                        f"command); every other row agrees")


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            got = fn()
            if isinstance(got, tuple) and got[0] == "SKIP":
                skipped += 1
                print(f"SKIP  {fn.__name__} ({got[1]})")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}  {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}  {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed"
          + (f", {skipped} skipped" if skipped else ""))
    sys.exit(1 if failed else 0)
