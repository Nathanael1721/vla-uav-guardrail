"""THE RETREAT RAY (guardrail/holdpoint.py, since 2026-11-21): a Loiter that
begins inside the disc a spinning zone sweeps, where the hold horizon's search
spends its budget on the nearest ring, is moved out along the ray away from
the zone instead of being held in place until the zone reaches it.

The world is smfin1120_021_dyn-nfz-rotate-spin's (hil, 71aabb3, 20 Nov 2026):
the 4 x 40 m bar hot-applied at 3.076 s and spun at 3 deg/s about its centre,
the aircraft at (-188.13, -0.06, 15) when the search ran (5.278 s, 2.2 s
after the spawn), 29 s of the Loiter's 30 s left. The flight held in place
('no point within the search's budget of 600 units ... stays legal for the
30.0 s'), the point broke at 25.3 s and the Loiter's timeout flew RTL.

Run:  python tests/test_hold_retreat.py
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import holdpoint as HP  # noqa: E402
from guardrail import load_policy  # noqa: E402
from guardrail.models import Action4D, DynamicNFZ, State  # noqa: E402
from guardrail.shield import Shield  # noqa: E402

HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}
DEMO = ROOT / "policies" / "sim_demo_policy.yaml"
# The bar as the flight's events.jsonl spawned it (local metres).
BAR = [(-170.002, 20.13), (-169.763, -19.869), (-165.764, -19.845), (-166.002, 20.154)]
SEARCH_T = 5.278 - 3.076             # the search's tick, from the spawn
SPAN_S = 30.0 - (5.278 - 4.277)      # the Loiter's time left then
FLOWN = State(x=-188.134521, y=-0.063131, up=15.001364)    # smfin1120_021
REL060 = State(x=-188.6, y=-0.1, up=15.0)                  # rel060_021


def _spinning_bar() -> Shield:
    bar = DynamicNFZ(id="bar", type="dynamic_nfz", **HARD_P0,
                     vertices=[{"x": x, "y": y} for x, y in BAR],
                     altitude_floor_m=0, altitude_ceiling_m=1000, margin_m=1.0,
                     motion={"vx_mps": 0.0, "vy_mps": 0.0, "yaw_rate_dps": 3.0})
    sh = Shield(load_policy(DEMO), escalation=False)
    sh.spawn_nfz(bar, t=0.0)
    sh.filter(FLOWN, Action4D(), t=SEARCH_T)
    return sh


def _legal_through(sh: Shield, p, until_s: float) -> bool:
    q = State(x=p.x, y=p.y, up=p.up)
    t = p.travel_s
    while t <= until_s + sh.lookahead_s + 1e-9:
        with sh.looking_ahead(t):
            if sh.hard_violations(sh.state_is_unsafe(q)):
                return False
        t += sh.lookahead_s
    return True


def test_the_flown_start_is_moved_out_of_the_sweep_by_the_ray():
    sh = _spinning_bar()
    p = HP.choose_hold_point(sh, FLOWN, hold_s=SPAN_S)
    assert p.kind == "moved" and p.legal, p.reason
    assert "retreat ray away from bar" in p.reason, p.reason
    assert p.x < FLOWN.x - 1.0, (p.x, p.y)          # out, away from the bar
    assert _legal_through(sh, p, SPAN_S), (p.x, p.y)


def test_without_the_ray_the_flown_start_is_held_in_place_as_it_was():
    old = HP.RETREAT_QUERIES
    HP.RETREAT_QUERIES = 0
    try:
        sh = _spinning_bar()
        p = HP.choose_hold_point(sh, FLOWN, hold_s=SPAN_S)
    finally:
        HP.RETREAT_QUERIES = old
    assert p.kind == "in_place", p.reason
    assert "budget" in p.reason and f"{SPAN_S:.1f} s" in p.reason, p.reason


def test_a_search_that_finds_its_point_plans_exactly_as_before():
    # rel060_021's start: the ring search finds its point; the ray never runs.
    sh = _spinning_bar()
    with_ray = HP.choose_hold_point(sh, REL060, hold_s=SPAN_S)
    old = HP.RETREAT_QUERIES
    HP.RETREAT_QUERIES = 0
    try:
        sh2 = _spinning_bar()
        without = HP.choose_hold_point(sh2, REL060, hold_s=SPAN_S)
    finally:
        HP.RETREAT_QUERIES = old
    assert "retreat ray" not in with_ray.reason, with_ray.reason
    assert with_ray == without


def test_grant_keeps_spent_honest():
    sh = _spinning_bar()
    led = HP._Ledger(sh, FLOWN, 10.0)
    led.left -= 50.0
    before, left = led.spent, led.left
    led.grant(100.0)
    assert led.spent == before and led.left == left + 100.0
    led.left = -3.0
    s = led.spent
    led.grant(10.0)
    assert led.left == 10.0 and led.spent == s


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"ok   {name}")
            except AssertionError as e:
                fails += 1
                print(f"FAIL {name}: {e}")
    print(f"{fails} failed")
    sys.exit(1 if fails else 0)
