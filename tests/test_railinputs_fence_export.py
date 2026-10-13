"""guardrail/fence_export.py: the clock-proof selection of the zones an
autopilot fence may hold, moved out of sitl/mavlink_adapter_node.py.

    python tests/test_railinputs_fence_export.py      (no pytest needed)

The selection used to live inside the ROS adapter's policy_fences, beside its
deferral of a zone the aircraft is near; the perception rail needs it without
ROS. The move must change nothing the adapter does: the first test holds the
adapter to the new module's answer on every shipped policy, column and
position (a differential run against the pre-move adapter at HEAD 4418ae0
gave 609 identical cases; see the unit's report). `now` must add a reading,
never change the selection.

Shown failing first: at HEAD 4418ae0 guardrail/fence_export.py does not exist,
so every test here errors on its import.
"""
from __future__ import annotations

import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sitl"))

from guardrail import load_policy                                    # noqa: E402
from guardrail.fence_export import (defer_near, iter_zones,          # noqa: E402
                                    policy_zones, zone_clearance_m)
from guardrail.models import Policy                                  # noqa: E402

POLICY = ROOT / "policies" / "sim_demo_policy.yaml"
_SQ = [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 10}, {"x": 0, "y": 10}]


def _timed() -> Policy:
    def zone(zid, dx, **kw):
        return {"id": zid, "type": "polygon_fence", **kw,
                "vertices": [{"x": v["x"] + dx, "y": v["y"]} for v in _SQ]}
    return Policy.model_validate({"policy_id": "timed", "constraints": [
        zone("yard", 0, valid_time={"recurrence": {"start_time": "07:30",
                                                   "end_time": "17:30"}}),
        zone("held", 40),
        {"id": "sw-off", "type": "time_window_switch", "target_id": "held",
         "active": False, "valid_time": {"recurrence": {"start_time": "08:00",
                                                        "end_time": "08:30"}}},
        zone("plain", 80),
        {"id": "lane", "type": "corridor", "width_m": 5,
         "centerline": [{"x": -50, "y": 0}, {"x": 50, "y": 0}]},
    ]})


def test_the_adapter_exports_exactly_the_modules_selection_on_every_policy():
    """policy_fences = policy_zones + the hold-back near the aircraft, skip
    list in policy order, on all 29 shipped policies."""
    import mavlink_adapter_node as A
    n = 0
    for f in sorted((ROOT / "policies").glob("*.yaml")):
        try:
            pol = load_policy(f, runtime=False)
        except Exception:                                # noqa: BLE001
            continue
        for col in (None, 10.0, 30.0, 120.0):
            zones, skipped = policy_zones(pol, column_m=col)
            assert A.policy_fences(pol, column_m=col) == (zones, skipped), f.name
            for pos in ((0.0, 0.0), (15.0, 15.0), (5.5, 15.0)):
                fences, sk = A.policy_fences(pol, column_m=col, position=pos)
                up, held = defer_near(zones, pos, A.FENCE_MARGIN_M)
                assert fences == up, (f.name, col, pos)
                assert sorted(s["id"] for s in sk) == sorted(
                    [s["id"] for s in skipped] + [h["id"] for h in held]), (f.name, pos)
                n += 1
    assert n >= 29 * 4 * 3 - 12, n


def test_a_clock_never_changes_what_is_exported():
    """The autopilot fence has no clock: the export at 08:15 (switch off,
    yard window open) and at 20:00 (both closed) is the export with no
    clock."""
    pol = _timed()
    base = policy_zones(pol, column_m=30.0)
    assert [z["id"] for z in base[0]] == ["plain"], base
    for when in (datetime(2026, 10, 12, 8, 15), datetime(2026, 10, 12, 20, 0)):
        zones, skipped = policy_zones(pol, now=when, column_m=30.0)
        assert [z["id"] for z in zones] == ["plain"]
        assert [s["id"] for s in skipped] == [s["id"] for s in base[1]]


def test_with_a_clock_every_zone_says_whether_the_shield_holds_it_now():
    pol = _timed()
    at = {}
    for when in (datetime(2026, 10, 12, 8, 15), datetime(2026, 10, 12, 20, 0)):
        zones, skipped = policy_zones(pol, now=when, column_m=30.0)
        at[when.hour] = {e["id"]: e.get("in_force_now") for e in zones + skipped}
    # yard: its own window 07:30-17:30; held: switched off 08:00-08:30.
    assert at[8] == {"yard": True, "held": False, "plain": True, "lane": None}, at
    assert at[20] == {"yard": False, "held": True, "plain": True, "lane": None}, at
    # Without a clock no entry carries the key at all.
    zones, skipped = policy_zones(pol, column_m=30.0)
    assert not any("in_force_now" in e for e in zones + skipped)


def test_the_skip_reasons_are_the_adapters():
    pol = _timed()
    kinds = [(k, e["id"]) for k, e in iter_zones(pol, column_m=30.0)]
    assert kinds == [("skipped", "yard"), ("skipped", "held"), ("zone", "plain"),
                     ("skipped", "lane")], kinds
    _, skipped = policy_zones(pol, column_m=30.0)
    why = {s["id"]: s for s in skipped}
    assert why["yard"]["time_controlled"] and "time window" in why["yard"]["why"]
    assert why["held"]["time_controlled"] and "sw-off" in why["held"]["why"]
    assert "keep-in corridor" in why["lane"]["why"]
    _, none_col = policy_zones(load_policy(POLICY), column_m=None)
    assert "unknown" in none_col[0]["why"]


def test_a_zone_near_the_aircraft_is_held_back_and_uploaded_once_clear():
    zones, _ = policy_zones(load_policy(POLICY), column_m=30.0)   # nfz-square
    up, held = defer_near(zones, (15.0, 15.0), 2.0)
    assert up == [] and held[0]["deferred"] and held[0]["clearance_m"] == -8.0
    up, held = defer_near(zones, (2.0, 15.0), 2.0)
    assert [z["id"] for z in up] == ["nfz-square"] and held == []
    assert defer_near(zones, None, 2.0) == (zones, [])
    assert zone_clearance_m(zones[0], 15.0, 15.0) == -8.0


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
