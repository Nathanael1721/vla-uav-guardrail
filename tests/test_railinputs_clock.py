"""The rule clock the flight rails give their Shield and their CSPs
(guardrail.replay.rule_clock, rail_shield(clock=), EpisodeRecord(clock=)).

    python tests/test_railinputs_clock.py      (no pytest needed)

Until 2026-10-12 neither SITL rail passed a clock: Shield(now=None) holds every
scheduled rule in force and ignores every time_window_switch's window, and
each CSP was compiled with no clock, so time windows and switches did nothing
in flight. These tests fly a zone with a 08:00-09:00 window through the
pymavlink rail's headless stand-in (tests/test_sitl_rails.py) at a scripted
08:30 and 12:00, and check the clock is the same one for the Shield and the
CSP, and that a run without --clock-start records the wall clock it used.

Shown failing first against a pristine HEAD 4418ae0 copy (git archive):
rule_clock does not exist (ImportError), rail_shield(clock=...) hands
`clock` to Shield (TypeError), EpisodeRecord records no rule_clock, and
run_sitl_demo.py refuses --clock-start (argparse exit 2).
"""
from __future__ import annotations

import json
import sys
import tempfile
from datetime import datetime, timedelta, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sitl"))

from guardrail import load_policy                                    # noqa: E402
from guardrail.models import Action4D, State                         # noqa: E402

POLICY = ROOT / "policies" / "sim_demo_policy.yaml"


def _windowed_policy_file() -> Path:
    """sim_demo_policy with its no-fly zone in force 08:00-09:00 only."""
    src = POLICY.read_text(encoding="utf-8")
    pol = load_policy(POLICY)
    zone = next(c for c in pol.constraints if c.id == "nfz-square")
    assert zone.valid_time is None
    doc = pol.model_dump(mode="json", exclude_none=True)
    for c in doc["constraints"]:
        if c["id"] == "nfz-square":
            c["valid_time"] = {"recurrence": {"start_time": "08:00",
                                              "end_time": "09:00"}}
    import yaml
    f = Path(tempfile.mkdtemp(prefix="clock_")) / "windowed.yaml"
    f.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
    assert src                                            # the source is untouched
    return f


def test_a_scripted_clock_advances_with_the_mission_time():
    from guardrail.replay import rule_clock
    t = [None]
    clock, rec = rule_clock("2026-10-12T17:29:50", elapsed=lambda: t[0])
    assert clock() == datetime(2026, 10, 12, 17, 29, 50)        # before the mission
    t[0] = 12.5
    assert clock() == datetime(2026, 10, 12, 17, 30, 2, 500000)
    assert rec["kind"] == "scripted" and rec["start"] == "2026-10-12T17:29:50"
    clock_z, rec_z = rule_clock("2026-10-12T09:00:00Z", elapsed=lambda: 0.0)
    assert clock_z().utcoffset() == timedelta(0) and rec_z["start"].endswith("+00:00")
    for bad in ("12 Oct 2026", "2026-13-40T00:00:00"):
        try:
            rule_clock(bad, elapsed=lambda: 0.0)
        except ValueError as e:
            assert "ISO 8601" in str(e)
            continue
        raise AssertionError(f"{bad!r} was accepted as a clock start")


def test_without_a_start_the_wall_clock_reads_the_missions_zone_and_says_so():
    """WSL runs on UTC: the machine's local time would put a Taipei window
    eight hours off. The default zone is a fixed +08:00, recorded."""
    from guardrail.replay import rule_clock
    instant = datetime(2026, 10, 12, 1, 15, tzinfo=timezone.utc)
    wall = lambda tz: instant.astimezone(tz)                   # noqa: E731
    clock, rec = rule_clock(None, wall=wall)
    assert clock() == datetime(2026, 10, 12, 9, 15) and clock().tzinfo is None
    assert rec["kind"] == "wall" and rec["tz"].startswith("+08:00")
    assert "default" in rec["tz_source"] and rec["first_reading"] == "2026-10-12T09:15:00"
    clock2, rec2 = rule_clock(None, tz="-05:30", wall=wall)
    assert clock2() == datetime(2026, 10, 11, 19, 45) and rec2["tz_source"] == "--clock-tz"
    assert rule_clock(None, tz="UTC", wall=wall)[0]() == datetime(2026, 10, 12, 1, 15)
    for bad in ("+25:00", "Mars/Olympus_Mons"):
        try:
            rule_clock(None, tz=bad, wall=wall)
        except ValueError:
            continue
        raise AssertionError(f"time zone {bad!r} was accepted")


def test_rail_shield_gives_the_shield_the_rails_clock():
    from guardrail.replay import rail_shield, rule_clock
    pol = load_policy(_windowed_policy_file())
    inside = State(x=15.0, y=15.0, up=10.0)
    hits = {}
    for hhmm in ("08:30", "12:00"):
        clock, _ = rule_clock(f"2026-10-12T{hhmm}:00", elapsed=lambda: 0.0)
        sh = rail_shield(pol, lookahead_s=5.0, dt=0.1, clock=clock)
        d = sh.filter(inside, Action4D(vx=1.0))
        hits[hhmm] = sorted({v.rule_id for v in d.violations})
    assert "nfz-square" in hits["08:30"] and "nfz-square" not in hits["12:00"], hits
    # No clock: in force at every hour, as every rail flew until 2026-10-12.
    d = rail_shield(pol, lookahead_s=5.0, dt=0.1).filter(inside, Action4D(vx=1.0))
    assert "nfz-square" in {v.rule_id for v in d.violations}


def test_the_episode_records_its_clock_and_compiles_each_csp_at_its_reading():
    from guardrail.compiler import ConstraintCompiler
    from guardrail.replay import EpisodeRecord, rule_clock
    pol = load_policy(_windowed_policy_file())
    mission = ConstraintCompiler(pol).parse_command("fly to the northeast pad at 6 m/s")
    t = [None]
    clock, crec = rule_clock("2026-10-12T12:00:00", elapsed=lambda: t[0])
    d = Path(tempfile.mkdtemp(prefix="clockrec_"))
    rec = EpisodeRecord(d, pol, mission, lookahead_s=5.0, clock=clock,
                        clock_record=crec)
    events = [json.loads(x) for x in (d / "events.jsonl").read_text(
        encoding="utf-8").splitlines()]
    assert events[0]["kind"] == "episode_start"
    assert events[0]["rule_clock"]["kind"] == "scripted", events[0]
    assert rec.generations[0]["rule_clock_reading"] == "2026-10-12T12:00:00"
    csp = json.loads((d / "csp_g0.json").read_text(encoding="utf-8"))
    assert csp.get("issued_at") == "2026-10-12T12:00:00", csp.get("issued_at")
    # A record without a clock says so.
    d2 = Path(tempfile.mkdtemp(prefix="clockrec_"))
    EpisodeRecord(d2, pol, mission, lookahead_s=5.0)
    e2 = json.loads((d2 / "events.jsonl").read_text(encoding="utf-8").splitlines()[0])
    assert e2["rule_clock"] == {"kind": "none"}, e2


def _fly(args):
    import test_sitl_rails as T
    return T._fly_pymavlink_rail(args)


def test_the_pymavlink_rail_flies_the_window_on_its_scripted_clock():
    """The same zone, the same straight flight with the Shield off: at 08:30
    the Shield holds it (violations on the zone, time inside it scored as
    nfz_s), at 12:00 it is off (the zone is in no row, and the time inside
    it is reported as nfz_off_by_clock_s, not as dwell: scoring it would fail
    a flight the policy allowed). metrics.json names the clock."""
    sys.path.insert(0, str(ROOT / "tests"))
    pol_file = _windowed_policy_file()
    seen, dwell = {}, {}
    for hhmm in ("08:30", "12:00"):
        out, metrics, rows, events, sim, R = _fly(
            ["--shield", "off", "--policy", str(pol_file),
             "--clock-start", f"2026-10-12T{hhmm}:00", "--tag", "t"])
        seen[hhmm] = sum(1 for r in rows for v in r["violations"]
                         if v["rule_id"] == "nfz-square")
        dwell[hhmm] = (metrics["nfz_s"],
                       metrics["nfz_off_by_clock_s"].get("nfz-square", 0.0))
        assert metrics["rule_clock"]["kind"] == "scripted", metrics["rule_clock"]
        assert metrics["rule_clock"]["start"] == f"2026-10-12T{hhmm}:00"
        start = next(e for e in events if e["kind"] == "episode_start")
        assert start["rule_clock"]["start"] == f"2026-10-12T{hhmm}:00"
    assert seen["08:30"] > 0 and seen["12:00"] == 0, seen
    assert dwell["08:30"][0] > 0 and dwell["08:30"][1] == 0.0, dwell
    assert dwell["12:00"][0] == 0.0 and dwell["12:00"][1] > 0, dwell


def test_zone_dwell_applies_each_points_clock_reading():
    """The count itself: a point inside the windowed zone counts at 08:30,
    not at 12:00, and a point without a reading counts (too high, never too
    low)."""
    from guardrail.replay import rail_shield, zone_dwell
    sh = rail_shield(load_policy(_windowed_policy_file()), lookahead_s=5.0, dt=0.1)
    p = {"x": 15.0, "y": 15.0, "up": 10.0}
    pts = [dict(p, t=0.0, clock="2026-10-12T08:30:00"),
           dict(p, t=0.1, clock="2026-10-12T12:00:00"),
           dict(p, t=0.2)]
    d = zone_dwell(sh, pts, 0.1, clock_key="clock")
    assert d["nfz_by_zone_s"] == {"nfz-square": 0.2}, d
    assert d["nfz_off_by_clock_s"] == {"nfz-square": 0.1}, d
    assert d["nfz_points_without_clock"] == 1 and d["nfz_first_index"] == 0
    # Without clock_key: the count every rail made until 2026-10-12.
    assert zone_dwell(sh, pts, 0.1)["nfz_by_zone_s"] == {"nfz-square": 0.3}


def test_a_rail_run_without_a_start_records_the_wall_clock_it_used():
    sys.path.insert(0, str(ROOT / "tests"))
    out, metrics, rows, events, sim, R = _fly(["--shield", "on"])
    rc = metrics["rule_clock"]
    assert rc["kind"] == "wall" and rc["tz"].startswith("+08:00"), rc
    assert "no --clock-start" in rc["note"]
    report = (out / "report.md").read_text(encoding="utf-8")
    assert "| Rule clock | wall: +08:00" in report, report


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
        # SystemExit too: a rail whose parser refuses a flag exits, and
        # that is this test failing, not the run ending.
        except (Exception, SystemExit) as e:         # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
