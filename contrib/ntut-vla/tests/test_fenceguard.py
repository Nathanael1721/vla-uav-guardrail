"""FenceGuard — does the controller pick the side the gap is actually on?

Run either way:
    pytest tests/test_fenceguard.py -v
    python tests/test_fenceguard.py

This exists because a flight got it wrong. `vlm_gapfence` (2026-08-11) flew
`follow_car_gap.yaml`, whose fence covers x 26..42 and leaves 7 m of legal road at
x 43..50. The aircraft slid WEST to x = 30.9 — the closed end — and finished the
flight 33.5 m behind the car, only 26.1% of the run within 30 m against 99.6%
unfenced. The rule held perfectly; the controller threw the mission away.

Two causes, both tested here:

  1. the old slide() scored each side by the fence distance at ONE probe point,
     which is symmetric information and cannot say which side you can get PAST on;
  2. slide() was only consulted once gate() raised `blocked`, which happens inside
     6.15 m, by which point the aircraft is at 35% speed and 5 m of lateral travel
     costs more ground than a 2 m/s target gives away.
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import load_policy                                # noqa: E402
from follow_vlm import FenceGuard                                # noqa: E402

GAP = ROOT / "policies" / "follow_car_gap.yaml"      # fence x 26..42, gap x 43..50
FULL = ROOT / "policies" / "follow_car_nfz.yaml"     # fence x 26..54, no gap


CITYMAP = ROOT / "demo" / "out" / "citymap" / "occ_day.npz"


def _guard(path, brake_m=12.0, stand_off_m=3.0, with_map=False):
    smap = None
    if with_map and CITYMAP.exists():
        import city_planner
        cm = city_planner.load_occ(str(CITYMAP))
        smap = {"occ": cm["occ"], "res": cm["res"], "ox": cm["ox"], "oy": cm["oy"]}
    return FenceGuard(load_policy(path), brake_m=brake_m, stand_off_m=stand_off_m,
                      obstacle_map=smap)


# The car drives north up lane x = 38, so the aircraft approaches heading +y.
NORTH = (0.0, 2.0)


def test_the_gap_policy_sends_the_aircraft_east_not_west():
    """The whole point. East is the 7 m of legal road; west is the closed end."""
    g = _guard(GAP)
    sx, sy, cost = g.slide(38.0, -6.0, *NORTH)
    assert (sx, sy) != (0.0, 0.0), "no side chosen at all, so it will just stop"
    assert sx > 0.5, (
        f"slide chose ({sx:+.2f}, {sy:+.2f}) — that is WEST, the closed end. "
        "This is the exact failure the vlm_gapfence flight recorded."
    )
    assert math.isfinite(cost) and cost <= 14.0, f"detour cost {cost}"


def test_the_detour_it_reports_matches_the_geometry():
    """Fence ends at x = 42 and the stand-off is 3 m, so from x = 38 the aircraft
    needs about 7 m of easting. Anything much smaller means it is aiming at a
    corner rather than at the road."""
    g = _guard(GAP)
    _, _, cost = g.slide(38.0, -6.0, *NORTH)
    assert 4.0 <= cost <= 12.0, f"detour of {cost} m does not match the 42+3 edge"


def test_a_fence_with_no_gap_reports_no_side_and_infinite_cost():
    """follow_car_nfz.yaml spans the whole corridor on purpose. Inventing a
    detour there would drive the aircraft into the boundary looking for one."""
    g = _guard(FULL)
    sx, sy, cost = g.slide(38.0, -6.0, *NORTH)
    assert (sx, sy) == (0.0, 0.0), f"found a way past a fence that has none: {sx},{sy}"
    assert cost == float("inf")


def test_the_side_choice_is_stable_along_the_whole_approach():
    """A controller that changes its mind at every tick weaves instead of
    committing, which is what makes fence behaviour look dangerous."""
    g = _guard(GAP)
    picks = []
    for y in range(-20, -2):
        sx, sy, cost = g.slide(38.0, float(y), *NORTH)
        if sx or sy:                          # NOT `or cost` — inf is truthy
            picks.append(sx > 0)
    assert picks, "never chose a side anywhere on the approach"
    assert all(picks) or not any(picks), f"side flip-flopped along the run: {picks}"
    assert all(picks), "committed to WEST along the approach"


def test_far_from_the_fence_there_is_no_side_to_choose():
    """Distant approach must return no opinion rather than a coin flip.

    With the fence out of probe range both sides scored the same minimum cost and
    the tie went to whichever the loop tried first — WEST, the closed end. The
    aircraft would have been committed to the wrong side before the fence was
    even a factor."""
    g = _guard(GAP)
    for y in (-30.0, -25.0, -20.0, -16.0):
        sx, sy, cost = g.slide(38.0, y, *NORTH)
        assert (sx, sy) == (0.0, 0.0) and cost == float("inf"), (
            f"at y={y} the path ahead is clear, yet slide picked ({sx},{sy})"
        )


def test_the_detour_begins_at_the_brake_distance_not_at_the_standoff():
    """The anticipation fix. At 12 m out the gate must already be slowing the
    aircraft, which is the condition the control loop now uses to start sliding —
    rather than waiting for `blocked`, which only fires inside 6.15 m."""
    g = _guard(GAP, brake_m=12.0, stand_off_m=3.0)
    # A point ~10 m south of the fence's y-edge (y = 2), closing on it.
    scale, dist, blocked = g.gate(38.0, -8.0, *NORTH)
    assert dist is not None and dist < g.brake_m, f"not yet inside the brake ring: {dist}"
    assert scale < 1.0, "gate is not slowing the aircraft at the brake distance"
    assert not blocked, ("premise of the fix: `blocked` is still false here, so a "
                         "controller that waits for it starts the detour far too late")
    sx, _, _ = g.slide(38.0, -8.0, *NORTH)
    assert sx > 0.5, "slide has no answer this far out, so anticipating cannot help"


def test_sliding_never_aims_somewhere_it_may_not_stand():
    """Every offset slide() returns must itself respect the stand-off, or the
    detour walks into the zone the Shield then has to repair."""
    from guardrail.geometry import fence_polygon
    from guardrail.models import PolygonFence
    from shapely.geometry import Point
    pol = load_policy(GAP)
    polys = [fence_polygon(f).buffer(f.margin_m) for f in pol.by_type(PolygonFence)]
    g = _guard(GAP)
    for y in range(-20, 0, 2):
        sx, sy, cost = g.slide(38.0, float(y), *NORTH)
        if not (sx or sy):
            continue
        px, py = 38.0 + sx * cost, float(y) + sy * cost
        d = min(p.distance(Point(px, py)) for p in polys)
        assert d >= g.stand_off_m - 1e-6, (
            f"slide target ({px:.1f},{py:.1f}) is {d:.1f} m from the fence, "
            f"inside the {g.stand_off_m} m stand-off"
        )


def test_no_fences_means_no_opinion():
    g = _guard(ROOT / "policies" / "follow_car.yaml")
    assert g.slide(38.0, -6.0, *NORTH) == (0.0, 0.0, float("inf"))
    assert g.gate(38.0, -6.0, *NORTH) == (1.0, None, False)


def test_a_stationary_command_asks_for_no_detour():
    """With no commanded motion there is no heading to sidestep relative to."""
    g = _guard(GAP)
    assert g.slide(38.0, -6.0, 0.0, 0.0) == (0.0, 0.0, float("inf"))


def test_a_detour_must_stay_on_the_road_not_merely_outside_the_fence():
    """The regression the anticipatory slide introduced.

    follow_car_nfz.yaml spans the whole corridor deliberately, so there is no way
    past. Knowing only about fences, slide() found one anyway by routing around
    the fence's eastern END at x > 55 - off the street entirely. Flown, fence_mode
    was `skirt` on 378 of 498 ticks against `hold` on 442 of 552 before, and
    Shield interventions went 0 -> 298 as the aircraft was pushed into building
    clearance. The Shield caught every one, which is the system working; the
    controller should not have been proposing them.
    """
    if not CITYMAP.exists():
        return
    g = _guard(FULL, with_map=True)
    offered = []
    for y in range(-20, 2):
        for x in (36.0, 38.0, 42.0, 46.0):
            sx, sy, cost = g.slide(x, float(y), *NORTH)
            if sx or sy:
                offered.append((x, y, round(cost, 1)))
    assert not offered, (
        f"slide offered a way past a corridor-spanning fence at {offered[:5]} - "
        "those detours leave the road"
    )


def test_the_gap_policy_still_finds_its_gap_with_the_map_on():
    """The clearance check must not be so strict that it kills the real gap."""
    if not CITYMAP.exists():
        return
    g = _guard(GAP, with_map=True)
    sx, sy, cost = g.slide(38.0, -6.0, *NORTH)
    assert sx > 0.5, f"clearance check destroyed the genuine eastward gap: {sx},{sy}"
    assert math.isfinite(cost)


def test_flying_along_the_gap_is_not_braked():
    """The reason the gap flight lost the car.

    Inside follow_car_gap.yaml's 7 m gap at x = 47, heading north, the nearest
    fence point is the corner at (43, 1) and that corner gets nearer as the
    aircraft advances — so a gate that brakes on shrinking distance throttled a
    trajectory that never enters the zone. Measured on v2_gap: the aircraft DID
    find the gap (277 of 552 ticks at x > 43) and was still held to 1.62-1.68 m/s
    against a car doing 2.0, slower than its target on 537 of 552 ticks.
    """
    g = _guard(GAP)
    for y in (-6.0, -2.0, 2.0, 6.0, 10.0):
        scale, dist, blocked = g.gate(47.0, y, *NORTH)
        assert scale == 1.0, (
            f"at (47,{y}) flying north up the gap, the gate scaled to {scale:.2f} "
            f"(fence {dist:.1f} m away) — that trajectory never enters the zone"
        )
        assert not blocked


def test_flying_into_the_fence_is_still_braked():
    """The gate must not become permissive: a heading that DOES enter the zone
    has to be slowed exactly as before."""
    g = _guard(GAP)
    scale, dist, blocked = g.gate(38.0, -6.0, *NORTH)   # straight into it
    assert scale < 1.0, f"a heading into the fence was not braked (scale {scale})"
    assert dist is not None and dist < g.brake_m


def test_a_heading_away_from_the_fence_is_never_braked():
    g = _guard(GAP)
    scale, _, blocked = g.gate(38.0, -6.0, 0.0, -2.0)   # south, away
    assert scale == 1.0 and not blocked


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
