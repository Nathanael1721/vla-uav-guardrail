"""Target identity for "follow a red car": demo/identity.py.

Run either way:
    pytest tests/test_identity.py -v
    python tests/test_identity.py

The follow locked on red pedestrian signals, signs and cones because nothing
asked whether a red box could physically BE a car. These tests build boxes
by projecting real 3-D objects through the FrontCamera (768x432, HFOV 90,
20 deg nose-down, 8 m up). Each one then checks the verdict: a car on the
road is OK, a thing whose bottom edge is 2.5 m up is HARD, a car hidden
behind a truck (a sliver) is only SOFT, and a missing feature never makes
anything HARD.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "demo"))

import camera_model as cam                                  # noqa: E402
from identity import (DEFAULTS, OFF_GRID_M, RULE_KEYS,      # noqa: E402
                      StreetDistance, classify, classify_split, features_for,
                      load_thresholds, noun_of, thresholds_hash)

W, H, HFOV = 768, 432, 90.0
POSE = {"x": 0.0, "y": 0.0, "up": 8.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}


def _T(m):
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


def project(pose, n, e, up):
    """World point (north, east, up) -> pixel (u, v): ray_world inverted."""
    d = (n - pose["x"], e - pose["y"], -(up - pose["up"]))
    b = cam._mat_mul_vec(_T(cam._rot_z(pose["yaw"])), d)
    b = cam._mat_mul_vec(_T(cam._rot_y(pose.get("pitch") or 0.0)), b)
    b = cam._mat_mul_vec(_T(cam._rot_x(pose.get("roll") or 0.0)), b)
    c = cam._mat_mul_vec(_T(cam._rot_y(math.radians(cam.MOUNT_PITCH_DEG))), b)
    f = cam.focal_px(W, HFOV)
    return W / 2 + f * c[1] / c[0], H / 2 + f * c[2] / c[0]


def box_of(pose, n, e, width_m, bottom_up, top_up, score=0.2, colour=0.5):
    """(box dict, slant range to its centre) for an upright object of this
    width standing (or hanging) at (n, e) between bottom_up and top_up."""
    mid = (bottom_up + top_up) / 2
    brg = math.atan2(e - pose["y"], n - pose["x"])
    pn, pe = -math.sin(brg), math.cos(brg)            # horizontal, rightward
    ul, _ = project(pose, n - pn * width_m / 2, e - pe * width_m / 2, mid)
    ur, _ = project(pose, n + pn * width_m / 2, e + pe * width_m / 2, mid)
    _, vt = project(pose, n, e, top_up)
    _, vb = project(pose, n, e, bottom_up)
    cx, cy = project(pose, n, e, mid)
    r = math.sqrt((n - pose["x"]) ** 2 + (e - pose["y"]) ** 2
                  + (mid - pose["up"]) ** 2)
    return ({"cx": cx, "cy": (vt + vb) / 2, "w": abs(ur - ul), "h": vb - vt,
             "score": score, "colour": colour}, r)


def road():
    """A north-south road |east| <= 4 m, 2 m cells, x -20..120, y -20..20."""
    xs = np.arange(-20.0, 121.0, 2.0)
    ys = np.arange(-20.0, 21.0, 2.0)
    street = np.zeros((len(xs), len(ys)), dtype=np.uint8)
    street[:, np.abs(ys) <= 4.0] = 1
    return StreetDistance(street, 2.0, -20.0, -20.0)


STREET = road()
TUNED = load_thresholds()
BOTH = (("defaults", dict(DEFAULTS)), ("tuned", TUNED))


def test_projection_round_trips_through_ray_world():
    for pose in (POSE, {**POSE, "yaw": 0.7, "pitch": -0.15, "roll": 0.1}):
        u, v = project(pose, 20.0, 3.0, 0.5)
        ray = cam.ray_world(u, v, W, H, HFOV, pose["yaw"], pose["pitch"],
                            pose["roll"])
        d = (20.0 - pose["x"], 3.0 - pose["y"], pose["up"] - 0.5)
        nd = math.sqrt(sum(c * c for c in d))
        assert all(abs(ray[i] - d[i] / nd) < 1e-9 for i in range(3)), (ray, d)


def test_a_car_at_20_m_on_the_road_is_ok():
    box, r = box_of(POSE, 20.0, 0.0, 1.8, 0.0, 1.5)
    f = features_for(box, r, POSE, W, H, HFOV, STREET)
    assert abs(f["rng_h"] - 20.0) < 0.1, f["rng_h"]
    assert abs(f["width_m"] - 1.8) < 0.05, f["width_m"]
    assert abs(f["bottom_h"]) < 0.5, f["bottom_h"]
    assert f["off_street_m"] == 0.0
    for name, th in BOTH:
        tier, why = classify(f, "car", th)
        assert tier == "ok", (name, why)
    assert classify(f, "a red car", TUNED)[0] == "ok"


def test_a_narrow_red_box_whose_bottom_is_2_5_m_up_is_hard():
    """A pedestrian signal head at the kerb: 0.5 m wide, 2.5-3.3 m up."""
    box, r = box_of(POSE, 15.0, 5.0, 0.5, 2.5, 3.3)
    f = features_for(box, r, POSE, W, H, HFOV, STREET)
    assert abs(f["bottom_h"] - 2.5) < 0.3, f["bottom_h"]
    for name, th in BOTH:
        tier, why = classify(f, "car", th)
        assert tier == "hard", (name, tier, why)
        assert any("bottom" in w for w in why), why


def test_a_car_sliver_behind_a_truck_is_soft_not_hard():
    """The case that forces two tiers: 0.7 m of a car beside a truck looks
    like a signal (narrow, tall) but stands on the road, on the street."""
    box, r = box_of(POSE, 25.0, 1.0, 0.7, 0.0, 1.2)
    f = features_for(box, r, POSE, W, H, HFOV, STREET)
    assert 0.6 < f["width_m"] < 0.8 and f["aspect"] < 0.8, (f["width_m"], f["aspect"])
    assert 10 <= f["w_px"] <= 15, f["w_px"]
    for name, th in BOTH:
        tier, why = classify(f, "car", th)
        assert tier == "soft", (name, tier, why)


def test_a_far_box_with_aspect_1_at_90_m_is_soft():
    box, r = box_of(POSE, 90.0, 0.0, 1.8, 0.0, 1.8)
    f = features_for(box, r, POSE, W, H, HFOV, STREET)
    assert f["rng_h"] > 60 and abs(f["aspect"] - 1.0) < 0.15, (f["rng_h"], f["aspect"])
    for name, th in BOTH:
        tier, why = classify(f, "car", th)
        assert tier == "soft", (name, tier, why)
        assert any(w.startswith("far:") for w in why), why


def test_a_car_8_m_off_the_street_is_hard():
    box, r = box_of(POSE, 30.0, 12.0, 1.8, 0.0, 1.5)
    f = features_for(box, r, POSE, W, H, HFOV, STREET)
    assert f["off_street_m"] > 6.0, f["off_street_m"]
    assert classify(f, "car", TUNED)[0] == "hard"


def test_person_rules_are_hard_only():
    pers, r = box_of(POSE, 15.0, 6.0, 0.5, 0.0, 1.8)       # on the pavement
    f = features_for(pers, r, POSE, W, H, HFOV, STREET)
    assert f["aspect"] < 0.5 and f["off_street_m"] <= 3.0
    assert classify(f, "person", TUNED) == ("ok", [])      # narrow is fine
    assert classify(f, "a pedestrian", TUNED) == ("ok", [])
    up, r = box_of(POSE, 15.0, 3.0, 0.5, 3.0, 4.8)        # on a balcony
    assert classify(features_for(up, r, POSE, W, H, HFOV, STREET),
                    "person", TUNED)[0] == "hard"
    far, r = box_of(POSE, 20.0, 12.0, 0.5, 0.0, 1.8)      # 8 m off the road
    assert classify(features_for(far, r, POSE, W, H, HFOV, STREET),
                    "person", TUNED)[0] == "hard"
    # a person whose bottom is 1.2 m up is still OK (limit 1.5)
    low, r = box_of(POSE, 15.0, 3.0, 0.5, 1.2, 3.0)
    assert classify(features_for(low, r, POSE, W, H, HFOV, STREET),
                    "person", TUNED)[0] == "ok"


def test_missing_features_never_hard_reject():
    box, r = box_of(POSE, 15.0, 5.0, 0.5, 2.5, 3.3)        # the signal again
    f = features_for(box, None, POSE, W, H, HFOV, STREET)
    assert f["r"] is None and f["bottom_h"] is None and f["P"] is None
    assert f["off_street_m"] is None and f["width_m"] is None
    tier, why = classify(f, "car", TUNED)
    assert tier == "soft" and "no range" in why, (tier, why)
    assert classify(f, "person", TUNED) == ("ok", [])
    # pitch not measured: no bottom height, so the bottom rule cannot fire
    f = features_for(box, r, {**POSE, "pitch": None}, W, H, HFOV)
    assert f["bottom_h"] is None and not f["pitch_measured"]
    assert classify(f, "car", TUNED)[0] == "soft"


def test_a_pose_that_is_not_finite_never_hard_rejects():
    """A NaN position used to land "off the grid" (1000 m off the street) and
    HARD-reject the box; an unknown place is SOFT, not impossible."""
    box, r = box_of(POSE, 20.0, 0.0, 1.8, 0.0, 1.5)       # a real car
    for bad in ({"x": math.nan}, {"up": math.inf}, {"yaw": math.nan}):
        f = features_for(box, r, {**POSE, **bad}, W, H, HFOV, STREET)
        assert f["P"] is None and f["rng_h"] is None and f["off_street_m"] is None
        assert f["bottom_h"] is None and f["bearing"] is None
        assert classify(f, "car", TUNED)[0] == "soft"
    f = features_for(box, r, {**POSE, "pitch": math.nan}, W, H, HFOV, STREET)
    assert not f["pitch_measured"] and f["bottom_h"] is None
    assert STREET((math.nan, 0.0)) != STREET((math.nan, 0.0))   # NaN, not 1000
    # a null rule value in a thresholds dict keeps the default (it used to
    # raise TypeError comparing a float with None)
    sig, rs = box_of(POSE, 15.0, 5.0, 0.5, 2.5, 3.3)       # the signal head
    fs = features_for(sig, rs, POSE, W, H, HFOV, STREET)
    tier, why = classify(fs, "car", {**TUNED, "bottom_max": None})
    assert tier == "hard" and any(f"> {DEFAULTS['bottom_max']:g}" in w
                                  for w in why), why


def test_a_frame_filling_box_is_hard_even_without_range():
    f = features_for({"cx": 384, "cy": 216, "w": 700, "h": 300}, None, POSE,
                     W, H, HFOV)
    assert classify(f, "car", TUNED)[0] == "hard"


def test_widths_are_pinhole_not_linear():
    """A 40 px box at the centre at 20 m subtends 2*atan(20/384): 2.08 m,
    where the old linear map said (pi/4)*(40/384)*20 = 1.64 m."""
    f = features_for({"cx": 384, "cy": 216, "w": 40, "h": 20}, 20.0, POSE,
                     W, H, HFOV)
    assert abs(f["width_m"] - 20.0 * 2 * math.atan(20.0 / 384.0)) < 1e-9


def test_every_box_shape_gives_the_same_features():
    d = {"cx": 300.0, "cy": 250.0, "w": 30.0, "h": 20.0, "score": 0.1,
         "colour": 0.4}
    a = features_for(d, 20.0, POSE, W, H, HFOV)
    b = features_for((300.0, 250.0, 30.0, 20.0, 0.1, W, H, 0.4), 20.0, POSE,
                     W, H, HFOV)
    c = features_for([300.0, 250.0, 30.0, 20.0, 0.1, 0.4], 20.0, POSE,
                     W, H, HFOV)
    assert a == b == c


def test_nouns():
    assert noun_of("a red car") == "car" and noun_of("red cars") == "car"
    assert noun_of("the white van") == "van" and noun_of("two buses") == "bus"
    assert noun_of("the pedestrian in red") == "person"
    assert noun_of("a building") is None and noun_of(None) is None
    # the head noun comes first: a car query past a zebra crossing stays a car
    assert noun_of("the red car past the pedestrian crossing") == "car"
    assert noun_of("the person next to the red car") == "person"
    box, r = box_of(POSE, 15.0, 5.0, 0.5, 2.5, 3.3)
    assert classify(features_for(box, r, POSE, W, H, HFOV), "a building") == ("ok", [])


def test_thresholds_hash_is_stable():
    th = dict(DEFAULTS)
    h = thresholds_hash(th)
    assert len(h) == 12 and h == thresholds_hash(dict(reversed(list(th.items()))))
    assert thresholds_hash({**th, "note": "x", "tuned_on": ["a"]}) == h
    assert thresholds_hash({**th, "width_min": th["width_min"] + 0.1}) != h
    assert thresholds_hash({k: int(v) if float(v).is_integer() else v
                            for k, v in th.items()}) == h


def test_the_committed_thresholds_file_is_self_consistent():
    p = ROOT / "demo" / "identity_thresholds.json"
    doc = json.loads(p.read_text(encoding="utf-8"))
    assert set(RULE_KEYS) <= set(doc), set(RULE_KEYS) - set(doc)
    assert doc["note"] and doc["tuned_on"]
    assert thresholds_hash(load_thresholds(p)) == doc["hash"]
    assert load_thresholds(ROOT / "no" / "such.json") == DEFAULTS


def test_street_distance_on_a_tiny_grid():
    s = np.zeros((5, 5), dtype=np.uint8)
    s[:, 2] = 1                                       # street along east = 4 m
    sd = StreetDistance(s, 2.0, 0.0, 0.0)
    assert sd((4.0, 4.0)) == 0.0                      # on a street cell
    assert abs(sd((4.0, 6.0)) - 2.0) < 1e-9           # one cell east
    assert abs(sd((4.0, 8.0)) - 4.0) < 1e-9           # two cells east
    assert abs(sd((4.0, 5.0)) - 1.0) < 1e-9           # the kerb, between
    assert abs(sd((4.0, 0.0)) - 4.0) < 1e-9
    assert sd((100.0, 4.0)) == OFF_GRID_M             # off the grid
    assert sd((8.9, 4.0)) == 0.0                      # half a cell past the edge
    assert sd((4.0, 4.0, 7.5)) == 0.0                 # the up component is ignored
    none = StreetDistance(np.zeros((3, 3)), 2.0, 0.0, 0.0)
    assert none((2.0, 2.0)) == OFF_GRID_M


def test_street_distance_loads_the_citylife_mask():
    p = ROOT / "demo" / "out" / "citymap_citylife" / "street.npz"
    if not p.is_file():
        return                                         # map not built here
    sd = StreetDistance.load(p)
    d = np.load(p)
    i, j = np.argwhere(d["street"] > 0)[0]
    n = float(d["origin_x"]) + i * float(d["res"])
    e = float(d["origin_y"]) + j * float(d["res"])
    assert sd((n, e)) == 0.0


def test_a_box_cut_off_by_the_frame_bottom_has_no_bottom_edge():
    """final2 seq 55-67: the car 6.5-7.7 m out, its box running off the
    bottom of the frame, read 1.8-2.0 m 'above the road' and went HARD."""
    box = {"cx": 384.0, "cy": 400.0, "w": 95.0, "h": 60.0, "score": 0.2, "colour": 0.5}
    f = features_for(box, 7.5, POSE, W, H, HFOV, STREET)
    assert f["bottom_clipped"] and f["bottom_h"] is None
    assert classify(f, "car", TUNED)[0] != "hard"
    box2 = dict(box, cy=380.0, h=40.0)          # bottom at 400: inside the frame
    assert not features_for(box2, 7.5, POSE, W, H, HFOV, STREET)["bottom_clipped"]


def test_classify_split_keeps_each_reason_with_its_rule():
    """A signal head is HARD by its bottom and also SOFT by width/aspect;
    the split lets a counter file only the bottom rule as HARD."""
    box, r = box_of(POSE, 15.0, 5.0, 0.5, 2.5, 3.3)
    f = features_for(box, r, POSE, W, H, HFOV, STREET)
    tier, hard, soft = classify_split(f, "car", TUNED)
    assert tier == "hard" and hard and all("bottom" in w or "street" in w for w in hard)
    assert classify(f, "car", TUNED) == (tier, hard + soft)
    ok = classify_split({"r": 20.0, "rng_h": 19.0, "width_m": 2.0, "aspect": 1.5,
                         "score": 0.2}, "car", TUNED)
    assert ok == ("ok", [], [])


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
