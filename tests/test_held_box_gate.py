"""
demo/vla_fly.take_held_box - a held box the subject could not have reached.

Run either way:
    pytest tests/test_held_box_gate.py -v
    python tests/test_held_box_gate.py

agx_e2e_aerovla_01 (2026-11-03) scored its one P0 escape against a phantom:
a held box scored 0.0093 (threshold 0.008) put the subject 20.1 m from the
previous estimate and 3.4 m from the aircraft, the car 29.9 m away. The
committed follow took every held box; it now takes one only within
follow_vlm.Reacquirer's reach from the last measured position. No
simulator, no GPU.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import vla_fly as VF                                                # noqa: E402


def _committed(t0=100.0, P=(40.0, 0.0)):
    box = VF.new_acquisition(None, t0=t0)
    box.update(committed=True, hist=[(t0, P[0], P[1])], last={"t": t0, "P": P},
               seen_t=t0, t_ok=t0, v=None, vxy=None)
    return box


def _drive(box, t0, speed, n, dt=0.4, y0=0.0):
    """n held "ok" boxes of a car driving +y at `speed`."""
    for k in range(1, n + 1):
        t = t0 + k * dt
        assert VF.take_held_box(box, t, t, "ok", (40.0, y0 + speed * k * dt)), k
    return t0 + n * dt


def test_a_car_at_speed_is_taken_box_after_box():
    box = _committed()
    t = _drive(box, 100.0, 10.0, 20)
    assert box["jumps"] == 0 and len(box["hist"]) == 21
    assert abs(box["v"] - 10.0) < 0.5, box["v"]
    vx, vy = box["vxy"]
    assert abs(vy - 10.0) < 0.5 and abs(vx) < 0.1, box["vxy"]
    assert box["seen_t"] == t


def test_the_phantom_jump_is_refused_and_not_a_sighting():
    """20.1 m from the last point 0.5 s later, as at tick 3824."""
    box = _committed()
    t = _drive(box, 100.0, 3.0, 10)
    last, seen, v = dict(box["last"]), box["seen_t"], box["vxy"]
    P = (box["hist"][-1][1] + 20.1 * math.cos(1.0), box["hist"][-1][2] + 20.1 * math.sin(1.0))
    assert not VF.take_held_box(box, t + 0.5, t + 0.5, "ok", P)
    assert box["jumps"] == 1 and box["last"] == last and box["seen_t"] == seen
    assert box["vxy"] == v, "a refused box must not move the measured velocity"
    j = box["jumps_first"][0]
    assert j["jump_m"] == 20.1 and j["reach_m"] < 20.1 and j["t"] == round(t + 0.5 - 100.0, 3), j


def test_the_reach_grows_with_the_time_since_the_last_good_point():
    """A box refused at 0.5 s is taken when it is as far 3 s later: the gate
    is a reach, not a blacklist - the car may simply have been unseen
    (at 3 m/s, v_r = 4.5 m/s: 12.25 m of reach after 0.5 s, 23.5 m after 3)."""
    box = _committed()
    t = _drive(box, 100.0, 3.0, 10)
    x, y = box["hist"][-1][1], box["hist"][-1][2]
    assert not VF.take_held_box(box, t + 0.5, t + 0.5, "ok", (x, y + 15.0))
    assert VF.take_held_box(box, t + 3.0, t + 3.0, "ok", (x, y + 15.0))
    assert box["jumps"] == 1


def test_a_slow_cars_map_point_jitter_is_not_a_jump():
    """The reason for reach_base_m, not the streak's 3 m step_m: a car at a
    red light whose map point jitters 6 m between frames (1 m depth steps,
    road behind the box) is the same car. With step_m the gate refused 460
    boxes within 5 m of the car over the replayed follows."""
    box = _committed()
    t = 100.0
    for k in range(1, 30):
        t += 0.2
        assert VF.take_held_box(box, t, t, "ok", (40.0 + (3.0 if k % 2 else -3.0), 0.0)), k
    assert box["jumps"] == 0


def test_a_soft_box_must_continue_the_track_an_ok_box_may_move_it():
    """The phantom of tick 3824 was SOFT, 12.4 m from the last point 0.91 s
    later: inside an ok box's reach (10 m + 4.5 m/s x 0.91 s = 14.1 m at
    3 m/s), outside a soft box's (3 m + 4.1 m = 7.1 m)."""
    box = _committed()
    t = _drive(box, 100.0, 3.0, 10)
    x, y = box["hist"][-1][1], box["hist"][-1][2]
    P = (x - 11.79, y - 3.93)                               # 12.4 m
    import copy
    soft = copy.deepcopy(box)
    assert not VF.take_held_box(soft, t + 0.91, t + 0.91, "soft", P)
    assert VF.take_held_box(box, t + 0.91, t + 0.91, "ok", P)


def test_the_reach_is_the_reacquirers():
    """reach_base_m + clip(1.5 v, v_min, v_max) x dt, with follow_vlm.
    Reacquirer's own numbers: 10 m, 3 and 12 m/s."""
    import follow_vlm as FV
    import inspect
    sig = inspect.signature(FV.Reacquirer.__init__).parameters
    assert VF.JUMP_REACH_BASE_M == sig["reach_base_m"].default
    assert VF.JUMP_STEP_M == sig["step_m"].default
    assert (VF.JUMP_V_MIN, VF.JUMP_V_MAX) == (sig["v_min"].default, sig["v_max"].default)
    box = _committed()
    # No speed yet: v_r = v_min = 3 m/s; 1 s later the reach is 13 m.
    assert VF.take_held_box(box, 101.0, 101.0, "ok", (40.0, 12.9))
    box = _committed()
    assert not VF.take_held_box(box, 101.0, 101.0, "ok", (40.0, 13.1))


def test_soft_boxes_and_boxes_without_a_map_point_keep_their_old_rules():
    box = _committed()
    # A soft box within 3 s of an ok one is a sighting; past 3 s it is not.
    assert VF.take_held_box(box, 101.0, 101.0, "soft", (40.0, 1.0))
    assert not VF.take_held_box(box, 104.5, 104.5, "soft", (40.0, 1.0))
    # No map point: a sighting, nothing measured, never a jump.
    n = len(box["hist"])
    assert VF.take_held_box(box, 105.0, 105.0, "ok", None)
    assert len(box["hist"]) == n and box["jumps"] == 0 and box["seen_t"] == 105.0


def test_refused_long_enough_the_subject_lapses_from_its_last_good_place():
    """Nothing but phantoms for LAPSE_S: shield_subject gives the Shield no
    subject rather than the phantom, and the last good place is what a
    re-acquisition would be anchored at."""
    box = _committed()
    t = _drive(box, 100.0, 3.0, 5)
    good = box["last"]["P"]
    k = 0
    while (t + 0.4 * (k + 1)) - box["seen_t"] <= VF.LAPSE_S + 0.5:
        k += 1
        assert not VF.take_held_box(box, t + 0.4 * k, t + 0.4 * k, "ok", (good[0] + 40.0, good[1]))
    now = t + 0.4 * k
    assert VF.shield_subject(box, now) is None
    assert box["last"]["P"] == good and box["jumps"] == k


def test_acquire_step_and_the_replay_take_boxes_through_the_same_function():
    src = (ROOT / "demo" / "vla_fly.py").read_text(encoding="utf-8")
    body = src.split("def _acquire_step", 1)[1].split("\ndef ", 1)[0]
    assert "take_held_box(acq_box" in body and "hist.append" not in body
    tool = (ROOT / "tools" / "replay_standoff_feed.py").read_text(encoding="utf-8")
    assert "VF.take_held_box(box" in tool


def test_the_record_carries_the_refusals():
    box = _committed()
    _drive(box, 100.0, 3.0, 3)
    VF.take_held_box(box, 102.0, 102.0, "ok", (90.0, 0.0))
    import inspect
    src = inspect.getsource(VF)
    assert '"held_jumps_refused": acq_box.get("jumps", 0)' in src
    assert box["jumps"] == 1 and box["jumps_first"][0]["jump_m"] > 40


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
