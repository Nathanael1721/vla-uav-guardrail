"""The on-screen policy indicator: demo/policy_hud.py.

Run either way:
    pytest tests/test_policy_hud.py -v
    python tests/test_policy_hud.py

Asked for at the 2026-09-30 meeting: show on screen that the guardrail policy
is in force - the rules, how close the aircraft is to each, and a banner when
the Shield acts. These tests pin what each status means, that a one-tick
repair stays visible for the latch time, that a car flight shows the
pedestrian rule as idle rather than hiding it, that the P0-escape count reads
the EMITTED action (the grant KPI) and not the raw one, that FenceGuard says
which hazard held, and that the drawing runs on a frame without touching the
Shield.

Frames: NED metres, x = North, y = East.
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import Shield, State, load_policy                # noqa: E402
from guardrail.models import Action4D                           # noqa: E402
from guardrail.shield import Repair, ShieldDecision, Violation  # noqa: E402
from policy_hud import (PolicyIndicator, draw_policy_overlay,   # noqa: E402
                        draw_status_box, render_base_map, rule_view)

NFZ = ROOT / "policies" / "follow_car_citylife_nfz.yaml"
CITY = ROOT / "policies" / "follow_car_citylife.yaml"
A0 = Action4D(vx=0.0, vy=3.0, vz_up=0.0, yaw_rate=0.0)


def _dec(viol=(), repairs=(), braked=False, emitted_viol=(), emitted=A0):
    return ShieldDecision(
        raw=A0, emitted=emitted,
        violations=[Violation(rule_id=r, category=c, detail="", predicted_at_s=0.0)
                    for r, c in viol],
        repairs=[Repair(operator=o, detail="") for o in repairs],
        braked=braked,
        emitted_violations=[Violation(rule_id=r, category=c, detail="", predicted_at_s=0.0)
                            for r, c in emitted_viol])


def _row(snap, rid):
    return next(r for r in snap["rows"] if r["id"] == rid)


def test_every_rule_of_the_policy_gets_a_row_in_policy_order():
    pol = load_policy(NFZ)
    snap = PolicyIndicator(pol).update(0.0, _dec(), State(x=44, y=0, up=8))
    assert [r["id"] for r in snap["rows"]] == [c.id for c in pol.constraints]
    assert snap["policy"] == "follow-car-citylife-nfz v0.1.0"


def test_rule_labels_carry_the_policy_numbers():
    pol = load_policy(NFZ)
    lim = {c.id: rule_view(c).limit for c in pol.constraints}
    assert lim["standoff-any"] == ">= 5 m"
    assert lim["bld-clearance"] == ">= 3 m"
    assert lim["alt-band"] == "6-14 m"
    assert lim["kin-caps"] == "<= 5 m/s"
    assert rule_view(pol.constraints[1]).label == "Stand-off from person"


def test_a_pedestrian_rule_on_a_car_flight_is_idle_not_hidden():
    ind = PolicyIndicator(load_policy(CITY))
    snap = ind.update(0.0, _dec(), State(x=44, y=0, up=8),
                      subject_xy=(44, 20), subject_class="car")
    assert _row(snap, "standoff-pedestrian")["status"] == "idle"
    assert _row(snap, "standoff-pedestrian")["value"] == "n/a (car)"
    any_row = _row(snap, "standoff-any")
    assert any_row["status"] == "ok" and any_row["value"] == "20 m"


def test_a_standoff_with_no_subject_is_idle():
    snap = PolicyIndicator(load_policy(CITY)).update(0.0, _dec(), State(x=44, y=0, up=8))
    assert _row(snap, "standoff-any")["status"] == "idle"
    assert _row(snap, "standoff-any")["value"] == "no target"


def test_near_is_the_policys_own_soft_margin():
    ind = PolicyIndicator(load_policy(CITY))
    # standoff-any: 5 m, soft margin 1 m -> NEAR below 6 m.
    s1 = ind.update(0.0, _dec(), State(x=0, y=0, up=8), subject_xy=(0, 6.5), subject_class="car")
    assert _row(s1, "standoff-any")["status"] == "ok"
    s2 = ind.update(0.1, _dec(), State(x=0, y=0, up=8), subject_xy=(0, 5.5), subject_class="car")
    assert _row(s2, "standoff-any")["status"] == "near"
    # bld-clearance: 3 m + 2 m soft -> NEAR below 5 m.
    s3 = ind.update(0.2, _dec(), State(x=0, y=0, up=8), clearance_m=4.2)
    assert _row(s3, "bld-clearance")["status"] == "near"


def test_a_shield_repair_marks_its_rule_acting_and_raises_the_banner():
    ind = PolicyIndicator(load_policy(CITY))
    snap = ind.update(10.0, _dec(viol=[("bld-clearance", "clearance")],
                                 repairs=["ClearanceFix"]),
                      State(x=0, y=0, up=8), clearance_m=3.4)
    assert _row(snap, "bld-clearance")["status"] == "act"
    assert snap["banner"]["level"] == "act"
    assert "pushed away from obstacle" in snap["banner"]["text"]
    assert "bld-clearance" in snap["banner"]["text"]
    assert snap["counts"]["repaired"] == 1


def test_a_one_tick_repair_stays_on_screen_for_the_latch_then_clears():
    ind = PolicyIndicator(load_policy(CITY), hold_s=1.5)
    ind.update(10.0, _dec(viol=[("kin-caps", "kinematic")], repairs=["SpeedClamp"]),
               State(x=0, y=0, up=8))
    mid = ind.update(11.0, _dec(), State(x=0, y=0, up=8))
    assert _row(mid, "kin-caps")["status"] == "act"
    assert mid["banner"]["level"] == "act"
    late = ind.update(11.6, _dec(), State(x=0, y=0, up=8))
    assert _row(late, "kin-caps")["status"] == "ok"
    assert late["banner"]["level"] == "ok"
    assert late["banner"]["text"].startswith("GUARDRAIL ACTIVE")


def test_a_brake_outranks_everything_and_is_counted():
    ind = PolicyIndicator(load_policy(CITY))
    snap = ind.update(0.0, _dec(viol=[("bld-clearance", "clearance")],
                                repairs=["Brake"], braked=True),
                      State(x=0, y=0, up=8), clearance_m=2.0,
                      fence_cause="obstacle", fence_mode="skirt")
    assert snap["banner"]["level"] == "brake"
    assert _row(snap, "bld-clearance")["status"] == "brake"
    assert snap["counts"]["braked"] == 1


def test_p0_escapes_use_the_kpi_definition():
    """guardrail/kpi.py: a P0 on the RAW command that is still a P0 on the SENT
    command. The HUD's count must agree with the KPI, tick for tick."""
    ind = PolicyIndicator(load_policy(CITY))
    # A P0 seen on the raw command and repaired: not an escape.
    ind.update(0.0, _dec(viol=[("bld-clearance", "clearance")], repairs=["ClearanceFix"]),
               State(x=0, y=0, up=8))
    assert ind.counts["p0_escapes"] == 0
    # Raw P1 only, a P0 left on the sent command: kpi.py does not count it.
    ind.update(0.1, _dec(viol=[("kin-caps", "kinematic")],
                         emitted_viol=[("alt-band", "altitude")]), State(x=0, y=0, up=8))
    assert ind.counts["p0_escapes"] == 0
    # Raw P0 and still P0 on what was sent: counted.
    snap = ind.update(0.2, _dec(viol=[("alt-band", "altitude")], repairs=["AltitudeFix"],
                                emitted_viol=[("alt-band", "altitude")]),
                      State(x=0, y=0, up=8))
    assert ind.counts["p0_escapes"] == 1
    assert snap["banner"]["level"] == "brake"
    assert snap["banner"]["text"] == "P0 RULE STILL BROKEN AFTER SHIELD (alt-band)"
    assert _row(snap, "alt-band")["status"] == "breach"


def test_an_escape_is_never_reported_as_a_successful_repair():
    """The review's case: stand-off repair chain ending in a building escape
    that flies at the car. The banner must not say the Shield backed off."""
    ind = PolicyIndicator(load_policy(NFZ))
    snap = ind.update(0.0, _dec(viol=[("standoff-any", "standoff")],
                                repairs=["StandoffRecover", "ClearanceFix", "ClearanceEscape"],
                                emitted_viol=[("standoff-any", "standoff")]),
                      State(x=2.5, y=0, up=8), subject_xy=(6.0, 0.0), subject_class="car",
                      clearance_m=3.2)
    assert "STILL BROKEN" in snap["banner"]["text"]
    assert "backing off" not in snap["banner"]["text"].lower()


def test_the_banner_names_the_operator_whose_result_was_flown():
    ind = PolicyIndicator(load_policy(CITY))
    snap = ind.update(0.0, _dec(viol=[("bld-clearance", "clearance")],
                                repairs=["ClearanceFix", "ClearanceEscape"]),
                      State(x=0, y=0, up=8), clearance_m=3.4)
    assert "recovery heading away from obstacle" in snap["banner"]["text"]


def test_a_shield_repair_outranks_a_controller_hold():
    ind = PolicyIndicator(load_policy(CITY))
    for k in range(10):
        viol = [("bld-clearance", "clearance")] if k % 2 else []
        snap = ind.update(k * 0.1, _dec(viol=viol, repairs=["ClearanceFix"] if viol else []),
                          State(x=0, y=0, up=8), clearance_m=5.0,
                          fence_cause="obstacle", fence_mode="hold")
        if k >= 1:
            assert snap["banner"]["level"] == "act", (k, snap["banner"])
            assert _row(snap, "bld-clearance")["status"] == "act", k


def test_a_repair_operator_marks_its_own_rule_acting():
    """A building push made while repairing a stand-off is the clearance rule
    acting too, not just the stand-off rule."""
    ind = PolicyIndicator(load_policy(CITY))
    snap = ind.update(0.0, _dec(viol=[("standoff-any", "standoff")],
                                repairs=["ClearanceFix", "StandoffRecover"]),
                      State(x=0, y=0, up=8), subject_xy=(0, 7.0), subject_class="car",
                      clearance_m=4.0)
    assert _row(snap, "bld-clearance")["status"] == "act"
    assert _row(snap, "standoff-any")["status"] == "act"
    assert _row(snap, "standoff-pedestrian")["status"] == "idle"


def test_past_a_limit_is_a_breach_never_all_rules_satisfied():
    for kw, rid, text in (
            (dict(clearance_m=1.5), "bld-clearance", "CLOSER THAN CLEARANCE TO OBSTACLE - RECOVERING"),
            (dict(subject_xy=(0, 3.0), subject_class="car"), "standoff-any",
             "CLOSER THAN STAND-OFF TO TARGET - RECOVERING")):
        snap = PolicyIndicator(load_policy(CITY)).update(0.0, _dec(), State(x=0, y=0, up=8), **kw)
        assert _row(snap, rid)["status"] == "breach", rid
        assert snap["banner"] == {"level": "breach", "text": text}
    low = PolicyIndicator(load_policy(CITY)).update(0.0, _dec(), State(x=0, y=0, up=4.0))
    assert _row(low, "alt-band")["status"] == "breach"
    assert low["banner"]["text"] == "OUTSIDE ALTITUDE BAND - RECOVERING"


def test_inside_the_zone_margin_is_a_breach_measured_like_the_shield():
    ind = PolicyIndicator(load_policy(NFZ))
    # 0.5 m outside the polygon, inside its 1 m margin: the Shield calls this
    # inside, metrics' nfz_entered does not - so the HUD says MARGIN.
    snap = ind.update(0.0, _dec(), State(x=44.5, y=70, up=8))
    r = _row(snap, "nfz-frontage")
    assert r["status"] == "breach" and r["value"] == "IN MARGIN"
    assert snap["banner"]["text"] == "INSIDE NO-FLY ZONE MARGIN nfz-frontage - RECOVERING"


def test_the_banner_never_pairs_an_operator_with_a_rule_of_another_kind():
    ind = PolicyIndicator(load_policy(CITY))
    snap = ind.update(0.0, _dec(viol=[("kin-caps", "kinematic")],
                                repairs=["YawClamp", "ClearanceFix"]),
                      State(x=0, y=0, up=8), clearance_m=6.0)
    assert snap["banner"]["text"] == "SHIELD CORRECTED COMMAND - turn rate capped (kin-caps)"


def test_clear_aim_turns_a_carrot_in_the_zone_band_into_a_parallel_line():
    """FenceGuard.clear_aim: the carrot on the car's lane (x = 44.5) is inside
    the zone's band; the re-aimed direction must point at x = 40 (44 - 3 - 1)
    and leave an aim point that is already clear alone."""
    import follow_vlm as fv
    g = fv.FenceGuard(load_policy(NFZ), stand_off_m=3.0)
    assert g.clear_aim(44.5, 20.0, 0.0, 1.0, 10.0) is None        # aim (44.5, 30): clear
    assert g.clear_aim(40.0, 70.0, 0.0, 1.0, 10.0) is None        # already on the line
    ux, uy = g.clear_aim(44.5, 66.0, 0.0, 1.0, 10.0)              # aim (44.5, 76)
    # ...now aimed at (40, 76): 4.5 m west for every 10 m east.
    assert uy > 0 and abs(ux / uy + 0.45) < 0.03, (ux, uy)
    # No fence in the policy: never re-aims.
    assert fv.FenceGuard(load_policy(CITY)).clear_aim(44.5, 66.0, 0.0, 1.0, 10.0) is None


def test_a_zone_outside_its_altitude_band_is_idle():
    from guardrail.models import Policy
    raw = load_policy(NFZ).model_dump()
    raw["constraints"][0]["altitude_ceiling_m"] = 5.0
    pol = Policy.model_validate(raw)
    snap = PolicyIndicator(pol).update(0.0, _dec(), State(x=50, y=70, up=8))
    r = _row(snap, "nfz-frontage")
    assert r["status"] == "idle" and r["value"] == "above zone"
    assert snap["banner"]["level"] == "ok"


def test_off_the_obstacle_map_is_unknown_not_clear():
    snap = PolicyIndicator(load_policy(CITY)).update(0.0, _dec(), State(x=0, y=0, up=8),
                                                     clearance_m=115.0, off_map=True)
    r = _row(snap, "bld-clearance")
    assert r["status"] == "near" and r["value"] == "off map"
    assert snap["banner"]["text"] == "OFF THE OBSTACLE MAP - CLEARANCE UNKNOWN"


def test_a_catch_all_standoff_binds_without_a_class_as_the_shield_does():
    snap = PolicyIndicator(load_policy(CITY)).update(0.0, _dec(), State(x=0, y=0, up=8),
                                                     subject_xy=(0, 20), subject_class=None)
    assert _row(snap, "standoff-any")["status"] == "ok"
    assert _row(snap, "standoff-any")["value"] == "20 m"
    assert _row(snap, "standoff-pedestrian")["status"] == "idle"


def test_an_unlogged_cause_is_not_blamed_on_either_hazard():
    ind = PolicyIndicator(load_policy(NFZ))
    snap = ind.update(0.0, _dec(), State(x=40.9, y=59, up=8), clearance_m=6.0,
                      fence_cause=None, fence_mode="hold")
    assert snap["banner"]["text"] == "CONTROLLER HOLDING (cause not logged)"
    assert _row(snap, "bld-clearance")["status"] != "hold"
    assert _row(snap, "nfz-frontage")["status"] != "hold"


def test_the_zone_warning_distance_follows_the_controller_brake_distance():
    pol = load_policy(NFZ)
    at = State(x=44, y=45, up=8)                       # 16 m from the margin ring
    assert PolicyIndicator(pol).update(0.0, _dec(), at)["banner"]["level"] == "ok"
    snap = PolicyIndicator(pol, fence_near_m=20.0).update(0.0, _dec(), at)
    assert snap["banner"]["level"] == "near"
    assert _row(snap, "nfz-frontage")["status"] == "near"


def test_the_minimap_registers_buildings_to_the_world_frame():
    """One occupied 2 m cell centred at (10, 20): with the aircraft at the
    origin and a 400 px, 80 m map it must be drawn centred on the pixel that
    (10, 20) maps to, not half a cell off."""
    from PIL import Image
    import numpy as np
    from policy_hud import draw_minimap
    occ = np.zeros((40, 40), dtype=bool)
    occ[25, 30] = True                                 # ox = oy = -40, res 2
    base = render_base_map({"occ": occ, "res": 2.0, "ox": -40.0, "oy": -40.0})
    im = Image.new("RGB", (400, 400))
    draw_minimap(im, {"drone": (0.0, 0.0), "yaw": None, "est": None, "trail": [],
                      "fences": [], "corridors": [], "base": base}, 0, 0, 400)
    px = np.asarray(im)
    dark = np.argwhere((px[:, :, 0] == 35) & (px[:, :, 1] == 35))
    rows, cols = dark[:, 0], dark[:, 1]
    # (x=10 north, y=20 east) -> col (20+40)/80*400 = 300, row (40-10)/80*400 = 150
    assert abs(cols.mean() - 300) <= 1.0 and abs(rows.mean() - 150) <= 1.0, (cols.mean(), rows.mean())


def test_controller_avoidance_is_avoid_not_a_shield_action():
    ind = PolicyIndicator(load_policy(CITY))
    snap = ind.update(0.0, _dec(), State(x=0, y=0, up=8), clearance_m=6.0,
                      fence_cause="obstacle", fence_mode="skirt")
    assert _row(snap, "bld-clearance")["status"] == "avoid"
    assert snap["banner"]["level"] == "avoid"
    assert snap["counts"]["repaired"] == 0 and snap["counts"]["avoided"] == 1


def test_an_obstacle_hold_is_a_hold():
    ind = PolicyIndicator(load_policy(CITY))
    snap = ind.update(0.0, _dec(), State(x=0, y=0, up=8), clearance_m=5.0,
                      fence_cause="obstacle", fence_mode="hold")
    assert _row(snap, "bld-clearance")["status"] == "hold"
    assert snap["banner"] == {"level": "hold", "text": "OBSTACLE AHEAD - HOLDING"}
    assert snap["counts"]["held"] == 1


def test_the_no_fly_zone_row_counts_down_then_holds_at_the_boundary():
    ind = PolicyIndicator(load_policy(NFZ))
    # Distances are to the zone's 1 m margin ring, which is what the Shield
    # enforces: 42.0 m to the polygon from (44, 20) reads 41 m.
    far = ind.update(0.0, _dec(), State(x=44, y=20, up=8))
    r = _row(far, "nfz-frontage")
    assert r["status"] == "ok" and r["value"] == "41 m away"
    near = ind.update(2.0, _dec(), State(x=44, y=55, up=8))
    assert _row(near, "nfz-frontage")["status"] == "near"
    assert near["banner"]["level"] == "near"
    assert "APPROACHING NO-FLY ZONE nfz-frontage - 6 m" in near["banner"]["text"]
    held = ind.update(4.0, _dec(), State(x=44, y=58, up=8),
                      fence_cause="fence", fence_mode="hold")
    assert _row(held, "nfz-frontage")["status"] == "hold"
    assert held["banner"]["level"] == "hold"
    assert "HOLDING AT THE BOUNDARY (nfz-frontage)" in held["banner"]["text"]


def test_routing_around_the_zone_is_named_as_the_zone():
    ind = PolicyIndicator(load_policy(NFZ))
    snap = ind.update(0.0, _dec(), State(x=40, y=70, up=8), clearance_m=8.0,
                      fence_cause="fence", fence_mode="skirt")
    assert _row(snap, "nfz-frontage")["status"] == "avoid"
    # The obstacle row is NOT blamed for a fence detour...
    assert _row(snap, "bld-clearance")["status"] == "ok"
    assert "ROUTING AROUND IT (nfz-frontage)" in snap["banner"]["text"]
    # ...and a controller detour is AVOID, not a Shield action.
    assert snap["banner"]["level"] == "avoid"


def test_inside_the_zone_reads_inside():
    snap = PolicyIndicator(load_policy(NFZ)).update(0.0, _dec(), State(x=50, y=70, up=8))
    assert _row(snap, "nfz-frontage")["value"] == "INSIDE"


def test_the_shield_views_match_what_the_shield_enforces():
    pol = load_policy(CITY)
    sh = Shield(pol)
    assert sh.subject is None and sh.subject_class is None
    sh.set_subject(3.0, 4.0, "car")
    assert sh.subject == (3.0, 4.0) and sh.subject_class == "car"
    sh.set_subject(None)
    assert sh.subject is None
    assert math.isinf(sh.clearance_at(0.0, 0.0))      # no map


def test_fence_guard_says_which_hazard_held():
    import follow_vlm as fv
    import numpy as np
    pol = load_policy(NFZ)
    occ = np.zeros((40, 40), dtype=bool)
    g = fv.FenceGuard(pol, obstacle_map={"occ": occ, "res": 2.0, "ox": 0.0, "oy": 0.0},
                      min_clearance_m=3.0)
    assert g.last_cause is None
    # Flying north-east straight at the zone's south edge from 2 m out: fence.
    _s, _d, blocked = g.gate(44.0, 59.0, 0.0, 3.0)
    assert blocked and g.last_cause == "fence"
    # Far from the zone, empty map: nothing held.
    g.gate(10.0, 0.0, 0.0, 3.0)
    assert g.last_cause is None
    # An obstacle dead ahead, zone far away: obstacle.
    occ2 = np.zeros((40, 40), dtype=bool)
    occ2[5, 8] = True                                  # (10, 16)
    g2 = fv.FenceGuard(pol, obstacle_map={"occ": occ2, "res": 2.0, "ox": 0.0, "oy": 0.0},
                       min_clearance_m=3.0)
    sc, _d, _b = g2.gate(10.0, 8.0, 0.0, 3.0)
    assert sc < 1.0 and g2.last_cause == "obstacle"


def test_the_overlay_draws_on_a_frame_and_clips_the_map():
    from PIL import Image
    import numpy as np
    pol = load_policy(NFZ)
    occ = np.zeros((60, 60), dtype=bool)
    occ[20:25, 30:35] = True
    base = render_base_map({"occ": occ, "res": 2.0, "ox": 0.0, "oy": 0.0})
    ind = PolicyIndicator(pol, base_map=base)
    for k in range(30):                    # a trail that runs far off the map window
        snap = ind.update(k * 0.1, _dec(), State(x=44, y=-200 + 10 * k, up=8),
                          est_xy=(44, 80), yaw_rad=math.pi / 2)
    im = Image.new("RGB", (1280, 720), (0, 120, 0))
    draw_status_box(im, ["t 1.0s", "bearing", "separation", "TARGET LOCKED"])
    draw_policy_overlay(im, snap)
    px = np.asarray(im)
    # The map sits in the bottom-right 230 px; everything left of it and below
    # the rule panel must still be the untouched green frame.
    assert (px[400:700, 400:1000] == (0, 120, 0)).all()
    assert not (px[720 - 220:720 - 10, 1280 - 220:1280 - 10] == (0, 120, 0)).all()


def test_a_reused_rule_id_does_not_break_the_indicator():
    """A copy-pasted block with the same id must not pair a row with the
    wrong rule (it raised inside the control loop before)."""
    from guardrail.models import Policy
    raw = load_policy(NFZ).model_dump()
    raw["constraints"][3]["id"] = "standoff-any"          # bld-clearance, renamed
    pol = Policy.model_validate(raw)
    snap = PolicyIndicator(pol).update(0.0, _dec(), State(x=40, y=70, up=8),
                                       subject_xy=(40, 90), subject_class="car",
                                       clearance_m=6.0)
    labels = [r["label"] for r in snap["rows"]]
    assert labels[3] == "Obstacle clearance" and snap["rows"][3]["value"] == "6.0 m"


def test_after_the_mission_every_row_is_idle_and_the_banner_says_so():
    ind = PolicyIndicator(load_policy(NFZ))
    ind.update(0.0, _dec(viol=[("kin-caps", "kinematic")], repairs=["SpeedClamp"]),
               State(x=44, y=0, up=8))
    snap = ind.ended(State(x=44, y=0, up=3), "MISSION ENDED - DESCENDING TO LAND")
    assert {r["status"] for r in snap["rows"]} == {"idle"}
    assert snap["banner"] == {"level": "idle", "text": "MISSION ENDED - DESCENDING TO LAND"}
    assert snap["counts"]["repaired"] == 1
    from PIL import Image
    draw_policy_overlay(Image.new("RGB", (1280, 720)), snap)


def test_the_overlay_cache_redraws_at_its_rate_and_at_once_on_a_new_banner():
    from PIL import Image
    import numpy as np
    from policy_hud import OverlayCache
    ind = PolicyIndicator(load_policy(CITY))
    cache = OverlayCache(hz=5.0)
    base = Image.new("RGB", (1280, 720), (40, 90, 40))
    renders = []
    for k in range(10):                                   # 1 s at 10 Hz, nothing happening
        snap = ind.update(k * 0.1, _dec(), State(x=0, y=k * 0.3, up=8), clearance_m=8.0)
        cache.draw(base.copy(), snap, ["t", "b", "s", "TARGET LOCKED"])
        renders.append(cache.renders)
    assert renders[-1] == 5, renders                      # every 0.2 s of flight time
    # A Shield action changes the banner: redrawn on that very frame.
    n = cache.renders
    snap = ind.update(1.02, _dec(viol=[("kin-caps", "kinematic")], repairs=["SpeedClamp"]),
                      State(x=0, y=3, up=8), clearance_m=8.0)
    cache.draw(base.copy(), snap)
    assert cache.renders == n + 1


def test_a_cached_overlay_looks_like_a_direct_one():
    from PIL import Image
    import numpy as np
    from policy_hud import OverlayCache
    ind = PolicyIndicator(load_policy(NFZ))
    snap = ind.update(0.0, _dec(), State(x=44, y=50, up=8), subject_xy=(44, 70),
                      subject_class="car", clearance_m=5.0, fence_cause="fence",
                      fence_mode="skirt", yaw_rad=1.5)
    rng = np.random.default_rng(1)
    frame = Image.fromarray(rng.integers(0, 255, (720, 1280, 3), dtype=np.uint8))
    lines = ["t 1.0s", "bearing", "separation", "TARGET LOCKED"]
    direct = frame.copy()
    draw_status_box(direct, lines)
    draw_policy_overlay(direct, snap)
    cached = frame.copy()
    OverlayCache().draw(cached, snap, lines)
    diff = np.abs(np.asarray(direct, dtype=int) - np.asarray(cached, dtype=int))
    assert diff.mean() < 1.0 and np.percentile(diff, 99.5) < 40, (diff.mean(), np.percentile(diff, 99.5))


def test_drawing_handles_a_policy_with_no_map_and_no_fence():
    from PIL import Image
    snap = PolicyIndicator(load_policy(CITY)).update(0.0, _dec(), State(x=0, y=0, up=8))
    im = Image.new("RGB", (960, 540))
    draw_policy_overlay(im, snap)                       # must not raise at 0.75 scale


# ----------------------------------------- one rule checker (X-13 / WP3-22, 2026-10-07)

def test_no_private_rule_geometry_is_left_in_the_panel():
    """The panel used to measure zones and corridors itself (_poly_distance,
    _polyline_distance) and decide each breach on its own. It must now read
    the Shield's rule checker and nothing else."""
    src = (ROOT / "demo" / "policy_hud.py").read_text(encoding="utf-8")
    for name in ("def _poly_distance", "def _polyline_distance", "subject_xy[0]",
                 "rule.min_range_m", "rule.min_clearance_m", "rule.speed_max_mps",
                 "rule.alt_min_m", "rule.width_m"):
        assert name not in src, name
    assert "rule_status(" in src


def test_the_panel_shows_what_the_shields_checker_reports():
    """Make the Shield report a breach the geometry does not have: the panel
    must show BREACH. A panel still measuring for itself would show OK."""
    ind = PolicyIndicator(load_policy(NFZ))
    real = ind.shield.rule_status

    def lying(state, action=None, **kw):
        rows = real(state, action, **kw)
        for r in rows:
            if r["id"] == "nfz-frontage":
                r.update(breach=True, inside=True, in_margin=False, distance_m=0.0)
        return rows
    ind.shield.rule_status = lying
    snap = ind.update(0.0, _dec(), State(x=0, y=0, up=8))
    r = _row(snap, "nfz-frontage")
    assert r["status"] == "breach" and r["value"] == "INSIDE", r


def _old_poly_distance(px, py, pts):
    """demo/policy_hud.py's own zone measurement as it stood at 401305a,
    frozen here as the reference for the parity check below."""
    inside = False
    n = len(pts)
    best = float("inf")
    for i in range(n):
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % n]
        if (ay > py) != (by > py):
            xc = ax + (py - ay) * (bx - ax) / (by - ay)
            if px < xc:
                inside = not inside
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best, inside


def test_the_shields_zone_reading_matches_the_panels_old_one_off_the_rim():
    """Every zone of every shipped policy, 400 positions each: the Shield's
    distance to the margin ring, inside / in-margin and band agree with the
    panel's old geometry. The only allowed difference is AT the ring (within
    2 mm), where the old test (distance <= margin) and the Shield's shapely
    ring (polygonal arcs, boundary excluded) can disagree; the Shield's is the
    one enforced."""
    import random
    rng = random.Random(5)
    n = rim = 0
    for path in sorted((ROOT / "policies").glob("*.yaml")):
        pol = load_policy(path)
        zones = [c for c in pol.constraints if c.type in ("polygon_fence", "circle_fence")]
        if not zones:
            continue
        sh = Shield(pol, escalation=False)
        for z in zones:
            pts = [(v.x, v.y) for v in z.vertices]
            xs, ys = [p[0] for p in pts], [p[1] for p in pts]
            for _ in range(400):
                x = rng.uniform(min(xs) - 15, max(xs) + 15)
                y = rng.uniform(min(ys) - 15, max(ys) + 15)
                up = rng.uniform(z.altitude_floor_m - 5, z.altitude_floor_m + 30)
                row = next(r for r in sh.rule_status(State(x=x, y=y, up=up))
                           if r["id"] == z.id)
                d, inside = _old_poly_distance(x, y, pts)
                in_ring = inside or d <= z.margin_m
                old_d = 0.0 if in_ring else d - z.margin_m
                if abs(d - z.margin_m) < 2e-3 and not inside:
                    rim += 1
                    continue
                n += 1
                assert row["inside"] == inside, (path.name, z.id, x, y)
                assert (row["inside"] or row["in_margin"]) == in_ring, (path.name, z.id, x, y)
                assert row["in_band"] == (z.altitude_floor_m <= up <= z.altitude_ceiling_m)
                assert abs(row["distance_m"] - old_d) < 2e-3, (path.name, z.id, row["distance_m"], old_d)
    assert n > 5000, n


def test_a_circle_fence_is_drawn_and_judged_like_any_zone():
    """circle_fence is enforced (a derived 32-gon); the panel ignored it: no
    row status beyond OK, nothing on the map."""
    from guardrail.models import Policy
    pol = Policy.model_validate({"policy_id": "c", "constraints": [
        {"id": "disc", "type": "circle_fence", "center": {"x": 0, "y": 50},
         "radius_m": 10, "margin_m": 1.0}]})
    ind = PolicyIndicator(pol)
    snap = ind.update(0.0, _dec(), State(x=0, y=45, up=8))
    r = _row(snap, "disc")
    assert r["label"] == "NFZ disc" and r["status"] == "breach" and r["value"] == "INSIDE"
    assert [(f[0], len(f[1])) for f in snap["map"]["fences"]] == [("disc", 32)]
    far = PolicyIndicator(pol).update(0.0, _dec(), State(x=0, y=20, up=8))
    assert _row(far, "disc")["value"] == "19 m away"       # 50 - 10 - 1 - 20


def test_a_hot_applied_zone_gets_a_row_and_leaves_with_it():
    from guardrail.models import DynamicNFZ
    sh = Shield(load_policy(NFZ))
    ind = PolicyIndicator(sh.policy, shield=sh)
    st = State(x=0, y=0, up=8)
    n0 = len(ind.update(0.0, _dec(), st)["rows"])
    sh.filter(st, A0)
    sh.hot_apply(DynamicNFZ.model_validate({
        "id": "landslide", "type": "dynamic_nfz", "margin_m": 0,
        "vertices": [{"x": -3, "y": -3}, {"x": 3, "y": -3}, {"x": 3, "y": 3}, {"x": -3, "y": 3}]}))
    snap = ind.update(0.1, _dec(), st)
    assert len(snap["rows"]) == n0 + 1 and _row(snap, "landslide")["status"] == "breach"
    assert "landslide" in [f[0] for f in snap["map"]["fences"]]
    sh.expire_nfz("landslide")
    snap = ind.update(5.0, _dec(), st)
    assert len(snap["rows"]) == n0 and "landslide" not in [f[0] for f in snap["map"]["fences"]]


def test_an_event_landing_mid_update_never_misaligns_a_row():
    """Events arrive on the REST thread. One that lands between the panel
    reading the rule list and reading the Shield's status must not pair a row
    with another rule's status (or index past the end): the rows and their
    status come from one snapshot of the Shield's rules."""
    from guardrail.models import DynamicNFZ

    def zone(zid, cx):
        return DynamicNFZ.model_validate({
            "id": zid, "type": "dynamic_nfz", "margin_m": 0,
            "vertices": [{"x": cx - 2, "y": -2}, {"x": cx + 2, "y": -2},
                         {"x": cx + 2, "y": 2}, {"x": cx - 2, "y": 2}]})
    sh = Shield(load_policy(NFZ))
    st = State(x=0, y=0, up=8)
    sh.filter(st, A0)
    sh.hot_apply(zone("first", 0.0))
    sh.hot_apply(zone("second", 300.0))
    ind = PolicyIndicator(sh.policy, shield=sh)
    ind.update(0.0, _dec(), st)
    real = sh.rule_status
    fired = []

    def racing(*a, **k):
        if not fired:
            fired.append(1)
            sh.expire_nfz("first")                 # lands mid-update
        return real(*a, **k)
    sh.rule_status = racing
    snap = ind.update(0.1, _dec(), st)
    ids = [r["id"] for r in snap["rows"]]
    assert "first" not in ids and ids[-1] == "second", ids
    assert _row(snap, "second")["value"] == "298 m away", _row(snap, "second")


def test_a_zone_outside_its_window_is_idle():
    from datetime import datetime
    from guardrail.models import Policy
    pol = Policy.model_validate({"policy_id": "w", "constraints": [{
        "id": "yard", "type": "polygon_fence", "margin_m": 0,
        "vertices": [{"x": -5, "y": -5}, {"x": 5, "y": -5}, {"x": 5, "y": 5}, {"x": -5, "y": 5}],
        "valid_time": {"recurrence": {"start_time": "07:30", "end_time": "17:30"}}}]})
    for hhmm, status, drawn in (("12:00", "breach", ["yard"]), ("18:00", "idle", [])):
        when = datetime(2026, 10, 7, int(hhmm[:2]), int(hhmm[3:]))
        sh = Shield(pol, now=lambda w=when: w)
        snap = PolicyIndicator(pol, shield=sh).update(0.0, _dec(), State(x=0, y=0, up=8))
        assert _row(snap, "yard")["status"] == status, (hhmm, _row(snap, "yard"))
        # ...and the map draws only the zones in force (2026-10-07 review: a
        # map that drew every zone passed every test).
        assert [f[0] for f in snap["map"]["fences"]] == drawn, (hhmm, snap["map"]["fences"])


# ---------------------------------------------------------------------------
# Every on-screen fact must be true (2026-10-07 review). A violated rule the
# Shield did not correct - monitor_only by design, or every rule in the log
# of a flight flown with the Shield off - was shown as ACTING under "SHIELD
# CORRECTED COMMAND", and counted as a correction. Both tests fail on the
# panel of that review.

def _monitor_only_policy():
    from guardrail.models import Policy
    return Policy.model_validate({"policy_id": "m", "version": "0.1.0", "constraints": [
        {"id": "nfz-watch", "type": "polygon_fence", "margin_m": 1.0,
         "violation_action": "monitor_only", "priority": "P1",
         "vertices": [{"x": 7, "y": 7}, {"x": 23, "y": 7}, {"x": 23, "y": 23},
                      {"x": 7, "y": 23}]},
        {"id": "kin", "type": "kinematic_envelope", "priority": "P1", "speed_max_mps": 4.0,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}]})


def test_a_monitor_only_rule_is_watched_never_shown_as_corrected():
    sh = Shield(_monitor_only_policy(), escalation=False)
    ind = PolicyIndicator(sh.policy, shield=sh)
    st = State(x=0, y=15, up=4)
    d = sh.filter(st, Action4D(vx=3.0))
    assert d.repairs == [] and d.emitted == d.raw          # flown as given, by design
    snap = ind.update(0.0, d, st)
    assert _row(snap, "nfz-watch")["status"] == "watch", _row(snap, "nfz-watch")
    assert snap["banner"] == {"level": "watch",
                              "text": "RULE BROKEN - MONITORED, NOT CORRECTED (nfz-watch)"}
    assert snap["counts"]["repaired"] == 0
    # A tick that DOES correct the speed cap: the cap is ACTING and counted,
    # the zone stays WATCH, and the banner names only what was corrected.
    d = sh.filter(st, Action4D(vx=6.0))
    assert [r.operator for r in d.repairs] == ["SpeedClamp"]
    snap = ind.update(5.0, d, st)
    assert _row(snap, "kin")["status"] == "act"
    assert _row(snap, "nfz-watch")["status"] == "watch"
    assert "SHIELD CORRECTED COMMAND" in snap["banner"]["text"]
    assert "nfz-watch" not in snap["banner"]["text"], snap["banner"]
    assert snap["counts"]["repaired"] == 1


def test_a_violation_with_no_repair_is_not_a_correction():
    """A Shield-off log: the raw command breaks a P1 rule and is flown as is
    (no repair, no brake). The panel must not claim a correction."""
    ind = PolicyIndicator(load_policy(CITY))
    snap = ind.update(0.0, _dec(viol=[("kin-caps", "kinematic")]), State(x=0, y=0, up=8))
    assert _row(snap, "kin-caps")["status"] == "watch", _row(snap, "kin-caps")
    assert snap["banner"]["level"] == "watch"
    assert "CORRECTED COMMAND" not in snap["banner"]["text"]
    assert snap["counts"]["repaired"] == 0


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
