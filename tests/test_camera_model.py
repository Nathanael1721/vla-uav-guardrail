"""The FrontCamera pinhole model: demo/camera_model.py.

Run either way:
    pytest tests/test_camera_model.py -v
    python tests/test_camera_model.py
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "demo"))

from camera_model import (angular_width, bearing_to_cx, box_angular_width,  # noqa: E402
                          depression, focal_px, ground_hit, horizontal_range,
                          pixel_bearing, point_at, ray_world, world_to_pixel)

W, H, HFOV = 768, 432, 90.0


def test_centre_and_edges_agree_with_the_field_of_view():
    assert abs(pixel_bearing(W / 2, W, HFOV)) < 1e-12
    assert abs(math.degrees(pixel_bearing(W, W, HFOV)) - 45.0) < 1e-9
    assert abs(math.degrees(pixel_bearing(0, W, HFOV)) + 45.0) < 1e-9


def test_a_quarter_of_the_way_in_is_where_the_linear_map_was_wrong():
    """The linear map said 22.5 deg at cx = 3W/4; a pinhole says 26.57."""
    b = math.degrees(pixel_bearing(0.75 * W, W, HFOV))
    assert abs(b - math.degrees(math.atan(0.5))) < 1e-9
    assert b - 22.5 > 4.0


def test_bearing_and_column_are_inverses():
    for cx in (0.0, 100.0, 384.0, 500.0, 767.0):
        assert abs(bearing_to_cx(pixel_bearing(cx, W, HFOV), W, HFOV) - cx) < 1e-9
    assert bearing_to_cx(math.radians(100), W, HFOV) == math.inf


def test_a_small_centred_box_subtends_twice_its_width_over_the_frame():
    """At 90 deg HFOV f = W/2, so a small centred box subtends ~2*bw/W rad -
    the linear map gave (pi/4)*bw/(W/2) = 0.785*bw/(W/2), ~21 % less."""
    a = box_angular_width(W / 2, 20.0, W, HFOV)
    assert abs(a - 2 * 20.0 / W) < 1e-4
    lin = math.radians(HFOV / 2) * 20.0 / (W / 2)
    assert 0.20 < 1 - lin / a < 0.22


def test_the_centre_ray_of_a_level_body_points_twenty_degrees_down():
    r = ray_world(W / 2, H / 2, W, H, HFOV, yaw=0.0)
    assert abs(math.degrees(depression(r)) - 20.0) < 1e-9
    assert r[0] > 0.9 and abs(r[1]) < 1e-12              # north, dead ahead


def test_yaw_turns_the_ray_clockwise_from_north():
    r = ray_world(W / 2, H / 2, W, H, HFOV, yaw=math.pi / 2)   # facing east
    assert r[1] > 0.9 and abs(r[0]) < 1e-9


def test_nose_down_pitch_steepens_the_view():
    level = depression(ray_world(W / 2, H / 2, W, H, HFOV, 0.0, body_pitch=0.0))
    down = depression(ray_world(W / 2, H / 2, W, H, HFOV, 0.0, body_pitch=math.radians(-5)))
    assert abs(math.degrees(down - level) - 5.0) < 1e-9


def test_the_centre_ray_meets_the_ground_where_trigonometry_says():
    r = ray_world(W / 2, H / 2, W, H, HFOV, yaw=0.0)
    n, e, slant = ground_hit(r, 8.0)
    assert abs(n - 8.0 / math.tan(math.radians(20))) < 1e-9 and abs(e) < 1e-9
    assert abs(slant - 8.0 / math.sin(math.radians(20))) < 1e-9
    assert ground_hit(ray_world(W / 2, 0.0, W, H, HFOV, 0.0), 8.0) is None   # above horizon


def test_a_point_at_range_has_the_height_the_ray_implies():
    r = ray_world(W / 2, H / 2, W, H, HFOV, yaw=0.0)
    x, y, up = point_at(r, 10.0, 0.0, 0.0, 8.0)
    assert abs(up - (8.0 - 10.0 * math.sin(math.radians(20)))) < 1e-9
    assert abs(horizontal_range(r, 10.0) - 10.0 * math.cos(math.radians(20))) < 1e-9


def test_focal_length():
    assert abs(focal_px(W, HFOV) - W / 2) < 1e-9
    assert abs(angular_width(0, W, W, HFOV) - math.pi / 2) < 1e-12



def test_world_to_pixel_inverts_ray_world():
    for (uu, vv, yaw, pitch, roll) in ((384, 216, 0.0, 0.0, 0.0), (600, 300, 1.2, -0.1, 0.05),
                                      (50, 400, -2.5, 0.08, -0.03), (700, 30, 3.0, 0.0, 0.1)):
        r = ray_world(uu, vv, W, H, HFOV, yaw, pitch, roll)
        pn, pe, pup = point_at(r, 23.0, 5.0, -7.0, 8.0)
        back = world_to_pixel(pn, pe, pup, 5.0, -7.0, 8.0, W, H, HFOV, yaw, pitch, roll)
        assert abs(back[0] - uu) < 1e-6 and abs(back[1] - vv) < 1e-6, (uu, vv, back)
    # behind the camera: no pixel
    assert world_to_pixel(-10.0, 0.0, 0.0, 0.0, 0.0, 8.0, W, H, HFOV, 0.0) is None

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
