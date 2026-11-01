"""
The VLA follow's subject stand-off: demo/vla_fly.py tells the Shield where
its subject is (shield_subject, subject_feed), and says INERT when it cannot.

Run either way:
    pytest tests/test_vla_standoff_feed.py -v
    python tests/test_vla_standoff_feed.py

WHAT WENT WRONG (found 2026-11-01, CHANGELOG "VLA flights never ran the
subject stand-off rules")

guardrail/shield.py's SubjectStandoff rules are inert until the flight calls
Shield.set_subject, and demo/vla_fly.py never called it. Every VLA follow -
AerialVLA, DroneCATS, AIOT-VLA v7-v10, on the PC and the Jetson - flew
follow_car_citylife_nfz.yaml's standoff-pedestrian (10 m) and standoff-any
(5 m), both P0, with neither ever judged. Nothing said so: the policy HUD's
stand-off rows read "idle, no target" under a banner reading OK, the audit
logs only violating ticks, and kpi.json's P0 escape rate of 0.0 counted no
stand-off tick. Against the level car's truth those flights were within
5 m of their car on 3.2 % of the VLA's ticks, as close as 0.02 m.

WHAT IS PINNED HERE

  1. THE FLIGHT LOOP (vla_fly._fly, read as source): every mission tick
     sets the Shield's subject before loop.tick judges the command - the
     perception's (shield_subject over the follow's acquisition), None
     when there is none - never the level car's truth; the HUD is handed
     the same subject; the subject is cleared when the mission ends.
  2. THE PERCEPTION'S SUBJECT (shield_subject, _acquire_step's `vxy`,
     measured_velocity): the last held box's map point, carried from its
     capture to now at the velocity measured from the held boxes and
     capped at the class's top speed; None while acquiring, after a lapse,
     and once unseen for LAPSE_S even when no detector frame arrives to
     run the lapse (a stalled camera stream: 4 of 39 flights of 1 Nov).
  3. THE DECISION (subject_feed): fed on a follow with the identity on;
     INERT, with the reason, on --gate-truth (the Grounder's box is the
     truth: a stand-off from it would be an oracle), with the identity
     off, on a mission with no subject; a pedestrian rule on a car is not
     applicable, not inert.
  4. THE RECORD (subject_standoff_record, inert_line): metrics.json's
     subject_standoff counts the ticks with a subject and flags `inert`
     when binding rules never had one, whatever the reason; the console
     line names the rules.
  5. WITH THE REAL SHIELD and the flown policy at the grant's 5 s / 0.1 s
     forecast: without a subject (the old flight) a car 2 m away draws no
     stand-off violation at all; with it, the breach is judged and
     repaired. Following a car at its own speed 12 m behind, a subject
     held still draws StandoffHold (the forecast flies the aircraft 25 m
     on while the car stands) and one given its velocity draws nothing:
     why SUBJECT_VELOCITY_TO_SHIELD is on.
  6. THE HUD: a stand-off of a flight that feeds no subject reads INERT,
     not "no target".

Shown failing first: on 1dd6c24 (the tree that flew v10b) 3/11 pass, the
three of section 5 that pin the Shield's own behaviour, which this change
leaves alone; the 8 on vla_fly and the HUD fail - _fly has no set_subject
call, and shield_subject, subject_feed, the records and the HUD's
subject_inert do not exist.

No simulator, GPU or model.
"""
from __future__ import annotations

import ast
import inspect
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import vla_fly                                                    # noqa: E402
from guardrail import State, load_policy                          # noqa: E402
from guardrail.models import Action4D, SubjectStandoff            # noqa: E402
from guardrail.replay import rail_shield                          # noqa: E402
from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S          # noqa: E402

POLICY = ROOT / "policies" / "follow_car_citylife_nfz.yaml"
STANDOFF_OPS = {"StandoffHold", "StandoffRecover"}     # the Shield's stand-off repairs


# --------------------------------------------------------------------------- #
# 1. the flight loop
# --------------------------------------------------------------------------- #

def _fly_tree():
    tree = ast.parse(inspect.getsource(vla_fly))
    return next(n for n in ast.walk(tree)
                if isinstance(n, ast.AsyncFunctionDef) and n.name == "_fly")


def _calls(node, owner: str, attr: str):
    return [c for c in ast.walk(node) if isinstance(c, ast.Call)
            and isinstance(c.func, ast.Attribute) and c.func.attr == attr
            and isinstance(c.func.value, ast.Name) and c.func.value.id == owner]


def _names(node) -> set:
    return {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}


def test_every_mission_tick_sets_the_shields_subject_before_it_judges():
    fly = _fly_tree()
    loops = [w for w in ast.walk(fly) if isinstance(w, ast.While)
             and _calls(w, "loop", "tick")]
    assert len(loops) == 1, f"one mission loop calling loop.tick, found {len(loops)}"
    w = loops[0]
    tick = _calls(w, "loop", "tick")[0]
    sets = _calls(w, "shield", "set_subject")
    assert sets, "the mission loop never calls shield.set_subject: every stand-off is inert"
    assert all(s.lineno < tick.lineno for s in sets), "set_subject after the Shield judged"
    # Both branches: a position with the subject's class, and None.
    cleared = [s for s in sets if len(s.args) == 1 and isinstance(s.args[0], ast.Constant)
               and s.args[0].value is None]
    given = [s for s in sets if len(s.args) >= 3]
    assert cleared and given, "set_subject must both give the subject and clear it"
    assert any(isinstance(a, ast.Name) and a.id == "subject_class" for a in given[0].args)
    assert any(k.arg == "velocity" for k in given[0].keywords)
    # The subject is the perception's (shield_subject over the acquisition),
    # never the level car's truth.
    subj = [a for a in ast.walk(w) if isinstance(a, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == "subj" for t in a.targets)]
    assert len(subj) == 1, "one assignment of the tick's subject"
    src = subj[0].value
    assert any(isinstance(c, ast.Call) and isinstance(c.func, ast.Name)
               and c.func.id == "shield_subject" for c in ast.walk(src))
    assert "acq_box" in _names(src)
    assert not _names(src) & {"level_car", "car_truth", "gate_truth_box", "car"}, _names(src)
    assert subj[0].lineno < min(s.lineno for s in sets)
    # The HUD reads the same subject.
    hud = _calls(w, "pind", "update")
    assert hud and {"subject_xy", "subject_class"} <= {k.arg for k in hud[0].keywords}
    # The row records what the Shield was given.
    keys ={k.value for d in ast.walk(tick) if isinstance(d, ast.Dict)
            for k in d.keys if isinstance(k, ast.Constant)}
    assert {"shield_subject", "shield_subject_v"} <= keys, keys
    # Cleared once the mission is over (the landing is not the follow's).
    after = [s for s in _calls(fly, "shield", "set_subject") if s.lineno > w.end_lineno]
    assert after and after[0].args and getattr(after[0].args[0], "value", 0) is None


def test_metrics_carry_the_subject_standoff_record():
    fly = _fly_tree()
    keys = {k.value for d in ast.walk(fly) if isinstance(d, ast.Dict)
            for k in d.keys if isinstance(k, ast.Constant)}
    assert "subject_standoff" in keys
    rec = [c for c in ast.walk(fly) if isinstance(c, ast.Call)
           and isinstance(c.func, ast.Name) and c.func.id == "subject_standoff_record"]
    assert rec, "metrics.subject_standoff is not built from subject_standoff_record"


# --------------------------------------------------------------------------- #
# 2. the perception's subject
# --------------------------------------------------------------------------- #

class _Grounder:
    """What _acquire_step reads from follow_vlm.Grounder.latest() once the
    subject is committed: a new held box (det, t_det, tier, feat P, cap)."""

    def __init__(self):
        self.seq = 0
        self.gw = {"seq": 0}
        self.reacquires = 0

    def see(self, t_cap: float, P, tier: str = "ok"):
        self.seq += 1
        self.gw = {"seq": self.seq, "t": t_cap + 0.4, "t_det": t_cap + 0.4,
                   "det": (400.0, 200.0, 40.0, 30.0, 0.2, 768, 432, 0.6), "tier": tier,
                   "feat": {"P": (float(P[0]), float(P[1]), 1.0)},
                   "cap": {"t_capture": t_cap}, "lock_why": None}

    def latest(self):
        return self.gw

    def begin_reacquire(self):
        self.reacquires += 1


class _Acq:
    def start(self, anchor):
        self.anchor = anchor


def _committed_box():
    box = vla_fly.new_acquisition(_Acq(), t0=1000.0)
    box["committed"] = True
    return box


def _drive(box, g, t_cap, P):
    """One new held box, processed 0.45 s after its capture (the detector's
    latency): _acquire_step's committed branch."""
    g.see(t_cap, P)
    vla_fly._acquire_step(g, box, None, clock=lambda: t_cap + 0.45)


def test_measured_velocity_is_measured_speed_as_a_vector():
    import follow_vlm as FV
    hist = [(0.0, 10.0, 0.0), (0.5, 12.6, 0.4), (1.0, 15.1, 0.9), (1.5, 17.4, 1.2)]
    vx, vy = vla_fly.measured_velocity(hist)
    assert abs(math.hypot(vx, vy) - FV.measured_speed(hist)) < 1e-12
    assert abs(vx - 7.4 / 1.5) < 1e-9 and abs(vy - 1.2 / 1.5) < 1e-9
    assert vla_fly.measured_velocity(hist[:2]) is None          # under 1 s of history
    assert vla_fly.measured_velocity([]) is None


def test_the_subject_is_the_held_box_carried_at_its_measured_velocity():
    box, g = _committed_box(), _Grounder()
    # A car driving north at 5 m/s, seen every 0.3 s.
    for k in range(6):
        t = 1000.0 + 0.3 * k
        _drive(box, g, t, (20.0 + 5.0 * (t - 1000.0), 3.0))
    vx, vy = box["vxy"]
    assert abs(vx - 5.0) < 1e-6 and abs(vy) < 1e-9, box["vxy"]
    last_cap = 1000.0 + 1.5
    now = last_cap + 0.6                       # 0.6 s after the last capture
    x, y, v = vla_fly.shield_subject(box, now)
    assert abs(x - (20.0 + 5.0 * (now - 1000.0))) < 1e-6 and abs(y - 3.0) < 1e-9
    assert v == box["vxy"]
    # Capped at the class's top speed (a jittered streak is no 40 m/s car).
    box["vxy"] = (40.0, 0.0)
    x2, _, v2 = vla_fly.shield_subject(box, now, vmax=15.0)
    assert v2 == (15.0, 0.0) and abs(x2 - (box["last"]["P"][0] + 15.0 * 0.6)) < 1e-6
    # The first second after a commit has no velocity: the point as it is.
    fresh, g2 = _committed_box(), _Grounder()
    _drive(fresh, g2, 2000.0, (5.0, 6.0))
    assert fresh["vxy"] is None
    assert vla_fly.shield_subject(fresh, 2000.9) == (5.0, 6.0, None)


def test_no_subject_while_acquiring_after_a_lapse_or_once_unseen():
    box, g = _committed_box(), _Grounder()
    for k in range(5):
        _drive(box, g, 1000.0 + 0.3 * k, (20.0 + 1.5 * k, 0.0))
    seen = box["seen_t"]
    assert vla_fly.shield_subject(box, seen + 1.0) is not None
    # A stalled camera stream: no new frame, so _acquire_step never runs its
    # lapse and the box stays "committed" - the subject still goes.
    vla_fly._acquire_step(g, box, None, clock=lambda: seen + vla_fly.LAPSE_S + 0.5)
    assert box["committed"], "the stall test needs a lapse that did not run"
    assert vla_fly.shield_subject(box, seen + vla_fly.LAPSE_S + 0.01) is None
    # The lapse itself.
    g.see(seen + 3.6, (99.0, 99.0), tier="soft")         # a SOFT box 3.6 s after OK
    vla_fly._acquire_step(g, box, None, clock=lambda: seen + 4.0)
    assert not box["committed"] and g.reacquires == 1
    assert vla_fly.shield_subject(box, seen + 0.1) is None
    # Acquiring: nothing.
    assert vla_fly.shield_subject(vla_fly.new_acquisition(_Acq()), 0.0) is None


# --------------------------------------------------------------------------- #
# 3. the decision and 4. the record
# --------------------------------------------------------------------------- #

def _standoffs():
    return load_policy(POLICY).by_type(SubjectStandoff)


def test_the_feed_is_on_for_a_follow_with_identity_and_inert_otherwise():
    so = _standoffs()
    assert [s.id for s in so] == ["standoff-pedestrian", "standoff-any"]
    on = vla_fly.subject_feed(so, "follow", "car", gate_truth=False, identity_on=True)
    assert on["on"] and on["why_not"] is None and on["source"] == vla_fly.SUBJECT_SOURCE
    assert on["binding"] == ["standoff-any"]        # the 10 m rule is a pedestrian's
    assert on["velocity_to_shield"] is True
    person = vla_fly.subject_feed(so, "follow", "pedestrian", gate_truth=False,
                                  identity_on=True)
    assert person["binding"] == ["standoff-pedestrian", "standoff-any"]
    cases = {
        "gate-truth": (dict(mission="follow", gate_truth=True, identity_on=True), "oracle"),
        "no identity": (dict(mission="follow", gate_truth=False, identity_on=False),
                        "identity is off"),
        "not set up": (dict(mission="follow", gate_truth=False, identity_on=None),
                       "never set up"),
        "goto": (dict(mission="goto", gate_truth=False, identity_on=False),
                 "goto mission has no subject"),
    }
    for name, (kw, why) in cases.items():
        f = vla_fly.subject_feed(so, kw["mission"], "car", gate_truth=kw["gate_truth"],
                                 identity_on=kw["identity_on"])
        assert not f["on"] and why in f["why_not"], (name, f["why_not"])
        assert f["source"] is None and f["binding"] == ["standoff-any"], name
        assert vla_fly.inert_line(f) and "standoff-any INERT" in vla_fly.inert_line(f), name
    assert vla_fly.inert_line(on) is None
    # The goto policy has no stand-off: nothing to warn about.
    goto = load_policy(ROOT / "policies" / "vla_goto_citylife.yaml").by_type(SubjectStandoff)
    none = vla_fly.subject_feed(goto, "goto", None, gate_truth=False, identity_on=False)
    assert none["rules"] == [] and vla_fly.inert_line(none) is None


def _row(subject=None, v=None, viol=(), ops=()):
    return {"shield_subject": subject, "shield_subject_v": v,
            "violations": [{"rule_id": r} for r in viol],
            "repairs": [{"operator": o} for o in ops]}


def test_the_record_flags_inert_whenever_binding_rules_never_had_a_subject():
    so = _standoffs()
    on = vla_fly.subject_feed(so, "follow", "car", gate_truth=False, identity_on=True)
    rows = [_row(), _row([1.0, 2.0], [5.0, 0.0], viol=["standoff-any"],
                     ops=["StandoffHold"]), _row([1.5, 2.0]), _row(viol=["bld-clearance"])]
    rec = vla_fly.subject_standoff_record(on, rows)
    assert rec["ticks"] == 4 and rec["ticks_fed"] == 2 and rec["frac_fed"] == 0.5
    assert rec["ticks_fed_with_velocity"] == 1 and rec["violation_ticks"] == 1
    assert rec["repairs"] == {"StandoffHold": 1, "StandoffRecover": 0}
    assert not rec["inert"] and rec["inert_rules"] == [] and vla_fly.inert_line(rec) is None
    assert rec["not_applicable"] == ["standoff-pedestrian"]
    # The feed on, but nothing ever acquired: inert all the same.
    never = vla_fly.subject_standoff_record(on, [_row(), _row()])
    assert never["inert"] and "never acquired" in never["inert_why"]
    assert never["inert_rules"] == ["standoff-any"]
    # The feed off (the flights of 1 Nov with --gate-truth).
    gt = vla_fly.subject_feed(so, "follow", "car", gate_truth=True, identity_on=True)
    off = vla_fly.subject_standoff_record(gt, [_row(viol=["nfz-frontage"])] * 3)
    assert off["inert"] and "oracle" in off["inert_why"] and off["violation_ticks"] == 0
    assert "standoff-any INERT" in vla_fly.inert_line(off)
    # A refused flight flew no tick: not inert, nothing judged at all.
    assert not vla_fly.subject_standoff_record(on, [])["inert"]


# --------------------------------------------------------------------------- #
# 5. with the real Shield, the flown policy, the grant's forecast
# --------------------------------------------------------------------------- #

def _shield():
    return rail_shield(load_policy(POLICY), lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S)


def _standoff(dec):
    return ({v.rule_id for v in dec.violations if v.category == "standoff"},
            {r.operator for r in dec.repairs} & STANDOFF_OPS)


def test_without_a_subject_a_car_two_metres_away_is_never_judged():
    sh = _shield()
    here = State(x=0.0, y=0.0, up=8.0, yaw_deg=0.0)
    toward = Action4D(vx=3.0, vy=0.0, vz_up=0.0, yaw_rate=0.0)
    sh.set_subject(None)                                  # the old flight, every tick
    assert _standoff(sh.filter(here, toward)) == (set(), set())
    sh.set_subject(2.0, 0.0, "car")                       # the same car, given
    viol, ops = _standoff(sh.filter(here, toward))
    assert viol == {"standoff-any"} and ops, (viol, ops)


def test_following_at_the_cars_speed_needs_its_velocity_at_the_grant_forecast():
    sh = _shield()
    here = State(x=0.0, y=0.0, up=8.0, yaw_deg=0.0)
    follow = Action4D(vx=5.0, vy=0.0, vz_up=0.0, yaw_rate=0.0)
    # 12 m behind a car driving away at 5 m/s, at its speed: the range holds.
    sh.set_subject(12.0, 0.0, "car")                      # held still
    viol, ops = _standoff(sh.filter(here, follow))
    assert "standoff-any" in viol and "StandoffHold" in ops, (viol, ops)
    sh.set_subject(12.0, 0.0, "car", velocity=(5.0, 0.0))
    assert _standoff(sh.filter(here, follow)) == (set(), set())
    # Closing on it at 3 m/s relative: 15 m in 5 s, through the 5 m ring.
    sh.set_subject(12.0, 0.0, "car", velocity=(2.0, 0.0))
    viol, ops = _standoff(sh.filter(here, follow))
    assert viol == {"standoff-any"} and ops, (viol, ops)


def test_a_person_binds_the_pedestrian_rule():
    """The class is the object words' (follow_vlm.subject_width), and a
    person binds the 10 m rule: at 8 m the pedestrian rule is breached."""
    import follow_vlm as FV
    assert FV.subject_width("a person")[1] == "pedestrian"
    assert FV.subject_width("a red car")[1] == "car"
    sh = _shield()
    sh.set_subject(8.0, 0.0, "pedestrian", velocity=(0.0, 0.0))
    viol, _ = _standoff(sh.filter(State(x=0.0, y=0.0, up=8.0),
                                  Action4D(vx=0.5, vy=0.0, vz_up=0.0, yaw_rate=0.0)))
    assert "standoff-pedestrian" in viol, viol


# --------------------------------------------------------------------------- #
# 6. the HUD
# --------------------------------------------------------------------------- #

def test_the_hud_says_inert_not_no_target_when_no_subject_is_ever_fed():
    from guardrail.shield import ShieldDecision
    from policy_hud import PolicyIndicator
    a0 = Action4D(vx=0.0, vy=0.0, vz_up=0.0, yaw_rate=0.0)
    dec = ShieldDecision(raw=a0, emitted=a0, violations=[], repairs=[], braked=False,
                         emitted_violations=[])
    ind = PolicyIndicator(load_policy(POLICY))

    def row(snap, rid):
        return next(r for r in snap["rows"] if r["id"] == rid)
    s = ind.update(0.0, dec, State(x=44, y=0, up=8))
    assert row(s, "standoff-any")["value"] == "no target"
    assert s["banner"]["text"] == "GUARDRAIL ACTIVE - all rules satisfied"
    ind.subject_inert = "--gate-truth: ... an oracle"
    s = ind.update(0.1, dec, State(x=44, y=0, up=8))
    assert row(s, "standoff-any")["status"] == "idle"
    assert row(s, "standoff-any")["value"] == "INERT"
    # ... and the banner no longer says "all rules satisfied" over it.
    assert s["banner"]["level"] == "ok" and "STAND-OFF INERT" in s["banner"]["text"], s["banner"]
    # A subject given still reads its separation.
    s = ind.update(0.2, dec, State(x=44, y=0, up=8), subject_xy=(44, 20), subject_class="car")
    assert row(s, "standoff-any")["value"] == "20 m"


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
            import traceback
            traceback.print_exc()
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
