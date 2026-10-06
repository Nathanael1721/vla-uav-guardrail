"""The lamp controller shows what the traffic obeys.

tools/citylife_mcp/signals.py renders UpdateLamps (BP_SignalController) from
the plan in tools/citylife_signals.py. `lamp_code` is the Blueprint's
arithmetic written out in Python; these tests hold it equal to the plan the
cars and the pedestrians use, for every head the survey found.

Run either way:
    pytest tests/test_citylife_signal_lamps.py -v
    python tests/test_citylife_signal_lamps.py
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools import citylife_signals as S                    # noqa: E402
from tools.citylife_mcp import signals as L                # noqa: E402

PLAN = L.plan()
TIMES = [0.25 * k for k in range(0, 800)]                  # four cycles


def test_the_survey_gives_heads_of_both_kinds():
    roles = [p["role"] for p in PLAN]
    assert len(PLAN) >= 250
    assert roles.count(0) >= 20 and roles.count(1) >= 200


def test_vehicle_lamps_follow_the_traffic_state():
    names = {S.GREEN: 0, S.YELLOW: 1, S.RED: 2}
    for p in PLAN:
        if p["role"] != 0:
            continue
        jx, jy = p["junction"]
        for t in TIMES:
            assert L.lamp_code(t, p["off"], 0) == names[S.state(t, jx, jy, p["axis"])], (p, t)


def test_pedestrian_lamps_follow_the_walk_phase():
    for p in PLAN:
        if p["role"] != 1:
            continue
        jx, jy = p["junction"]
        for t in TIMES:
            code = L.lamp_code(t, p["off"], 1)
            ph = S.ped_phase(t, jx, jy, p["axis"])
            if ph == S.WALK:
                assert code == 3, (p, t)
            elif ph == S.FLASH:
                assert code in (3, 5), (p, t)
            else:
                assert code == 4, (p, t)


def test_a_leg_never_shows_green_to_cars_and_walk_to_people_at_once():
    by_leg = {}
    for p in PLAN:
        by_leg.setdefault((tuple(p["junction"]), p["axis"]), []).append(p)
    for heads in by_leg.values():
        veh = [h for h in heads if h["role"] == 0]
        ped = [h for h in heads if h["role"] == 1]
        if not veh or not ped:
            continue
        for t in TIMES:
            if L.lamp_code(t, veh[0]["off"], 0) in (0, 1):
                assert L.lamp_code(t, ped[0]["off"], 1) == 4


def test_the_rendered_graph_has_every_constant_and_no_placeholder():
    src = L.render(L.LAMPS) + L.render(L.EVENTS)
    assert "__" not in src.replace("_i", "").replace("_b.", "")
    assert "{" not in src
    for m in ("MI_jctTrafficLightBase", "MI_jctTrafficLight_Yellow", "MI_jctTrafficLight_Red_b"):
        assert m in src


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
