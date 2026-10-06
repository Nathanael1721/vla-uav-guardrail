"""The FrontCamera as a pinhole: pixels to angles, rays, and ground points.

WHY

Every bearing in this project was computed as `hfov/2 * (cx - W/2)/(W/2)` - a
LINEAR map from pixel to angle. A camera is a pinhole: the angle of a pixel is
atan((cx - W/2) / f) with f = (W/2)/tan(hfov/2). At 90 deg HFOV the two agree
at the centre and at the edges and differ by up to ~4 deg in between (a quarter
of the way in: 22.5 deg linear, 26.6 deg true), and the angle a SMALL box
subtends at the centre is 2*bw/W rad, not (pi/4)*bw/(W/2): widths from the
linear map were ~21 % small. The estimator, the presence width check and the
tracking scorer all inherited that (found 2026-09-29, while designing the
identity check that needs physical widths to be right).

Frames: NED. The body frame is x forward, y right, z down; the camera frame is
the same axes rotated by the mount's pitch (-20 deg: nose down). Yaw is from
North, clockwise; pitch nose-UP positive; roll right-wing-DOWN positive.

Pure geometry, no simulator: tests/test_camera_model.py.
"""
from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]

MOUNT_PITCH_DEG = -20.0          # robot_semantic_quad.jsonc FrontCamera rpy 0 -20 0


def focal_px(img_w: float, hfov_deg: float) -> float:
    """Focal length in pixels for a square-pixel pinhole of this width/HFOV."""
    return (img_w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


def pixel_bearing(cx: float, img_w: float, hfov_deg: float) -> float:
    """Horizontal angle (rad) of image column `cx` off the optical axis;
    positive to the right. Exact for a level camera; with the -20 deg mount it
    is the angle in the camera's own frame, which is what a yaw servo turns."""
    return math.atan((cx - img_w / 2.0) / focal_px(img_w, hfov_deg))


def bearing_to_cx(bearing_rad: float, img_w: float, hfov_deg: float) -> float:
    """Inverse of pixel_bearing: the column a bearing lands on. +-inf at or
    beyond +-90 deg (behind the camera it has no column)."""
    if abs(bearing_rad) >= math.pi / 2:
        return math.copysign(math.inf, bearing_rad)
    return img_w / 2.0 + focal_px(img_w, hfov_deg) * math.tan(bearing_rad)


def angular_width(x0: float, x1: float, img_w: float, hfov_deg: float) -> float:
    """Angle (rad) between columns x0 and x1 - what a box of those edges
    subtends horizontally."""
    return abs(pixel_bearing(x1, img_w, hfov_deg) - pixel_bearing(x0, img_w, hfov_deg))


def box_angular_width(cx: float, bw: float, img_w: float, hfov_deg: float) -> float:
    return angular_width(cx - bw / 2.0, cx + bw / 2.0, img_w, hfov_deg)


def _mat_mul_vec(m, v) -> Vec3:
    return (m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2])


def _rot_y(theta: float):
    """Rotation of the FRAME nose-up by theta (aerospace pitch)."""
    c, s = math.cos(theta), math.sin(theta)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rot_x(phi: float):
    c, s = math.cos(phi), math.sin(phi)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _rot_z(psi: float):
    c, s = math.cos(psi), math.sin(psi)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def ray_world(u: float, v: float, img_w: float, img_h: float, hfov_deg: float,
              yaw: float, body_pitch: float = 0.0, body_roll: float = 0.0,
              mount_pitch_deg: float = MOUNT_PITCH_DEG) -> Vec3:
    """Unit NED direction (north, east, down) of the ray through pixel (u, v),
    for a camera on a body with this yaw/pitch/roll (rad)."""
    f = focal_px(img_w, hfov_deg)
    c = (1.0, (u - img_w / 2.0) / f, (v - img_h / 2.0) / f)
    n = math.sqrt(c[0] ** 2 + c[1] ** 2 + c[2] ** 2)
    c = (c[0] / n, c[1] / n, c[2] / n)
    b = _mat_mul_vec(_rot_y(math.radians(mount_pitch_deg)), c)
    b = _mat_mul_vec(_rot_x(body_roll), b)
    b = _mat_mul_vec(_rot_y(body_pitch), b)
    return _mat_mul_vec(_rot_z(yaw), b)



def _transpose(m):
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


def world_to_pixel(pn: float, pe: float, pup: float, cam_n: float, cam_e: float,
                   cam_up: float, img_w: float, img_h: float, hfov_deg: float,
                   yaw: float, body_pitch: float = 0.0, body_roll: float = 0.0,
                   mount_pitch_deg: float = MOUNT_PITCH_DEG) -> Optional[Tuple[float, float]]:
    """Pixel (u, v) of the world point (north, east, up) seen from a camera at
    (cam_n, cam_e, cam_up) on a body with this yaw/pitch/roll: the exact
    inverse of ray_world. None when the point is behind the camera. The
    level-camera shortcut bearing_to_cx(azimuth - yaw) ignores the 20 deg
    mount and the body's attitude, and puts a near, off-axis point 20-45 px
    too far from the centre (review, 2026-09-29)."""
    d = (pn - cam_n, pe - cam_e, cam_up - pup)          # NED: down positive
    b = _mat_mul_vec(_transpose(_rot_z(yaw)), d)
    b = _mat_mul_vec(_transpose(_rot_y(body_pitch)), b)
    b = _mat_mul_vec(_transpose(_rot_x(body_roll)), b)
    c = _mat_mul_vec(_transpose(_rot_y(math.radians(mount_pitch_deg))), b)
    if c[0] <= 1e-9:
        return None
    f = focal_px(img_w, hfov_deg)
    return img_w / 2.0 + f * c[1] / c[0], img_h / 2.0 + f * c[2] / c[0]

def depression(ray: Vec3) -> float:
    """Angle (rad) of a NED ray below the horizon; negative above it."""
    horiz = math.hypot(ray[0], ray[1])
    return math.atan2(ray[2], horiz)


def ground_hit(ray: Vec3, cam_up_m: float,
               min_depression_deg: float = 2.0) -> Optional[Tuple[float, float, float]]:
    """Where a ray from a camera `cam_up_m` above flat ground meets it:
    (north offset, east offset, slant range) relative to the camera. None when
    the ray is within `min_depression_deg` of the horizon or above it."""
    if cam_up_m <= 0 or depression(ray) < math.radians(min_depression_deg):
        return None
    t = cam_up_m / ray[2]
    return (ray[0] * t, ray[1] * t, t)


def point_at(ray: Vec3, r: float, cam_n: float, cam_e: float, cam_up: float) -> Vec3:
    """The world point (north, east, up) at slant range r along a ray."""
    return (cam_n + ray[0] * r, cam_e + ray[1] * r, cam_up - ray[2] * r)


def horizontal_range(ray: Vec3, r: float) -> float:
    """Horizontal distance to the point at slant range r along a ray."""
    return r * math.hypot(ray[0], ray[1])
