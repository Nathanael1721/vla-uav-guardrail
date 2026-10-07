# Facing East, "forward" flew North

**Date:** 2026-10-06
**Found by:** the contract audit (card WP3-04: the grant locks body-frame
velocities, ours are world frame) and the regression tests written for the CSP
work in `tests/test_real_vla_prompt.py`.
**Status:** fixed in `demo/real_vla_demo.py`, with regression tests. Not yet
flown with the fix.

## What was claimed

`demo/real_vla_demo.py` puts OpenVLA-7B in the action slot. Until 6 Oct the
mapping from the model's output to the Shield's input read:

```python
# OpenVLA emits 7-DoF arm deltas; map first 3 to body velocities,
# scaled into the drone's envelope.
act = Action4D(vx=float(np.clip(r[0] * self.scale, -4, 4)),
               vy=float(np.clip(r[1] * self.scale, -4, 4)), ...)
```

The comment calls the result "body velocities". The code writes them into
`Action4D.vx` / `vy`, which are **North** and **East** (`guardrail/models.py`),
and the control loop sends them to `moveByVelocityAsync`, which is world frame.
Nothing rotated one into the other.

Two defects, both silent:

1. **No rotation.** The model sees a camera image, so its "forward" is the
   aircraft's nose. Written straight into North/East, "forward" means North
   whatever the heading.
2. **No heading.** The control loop built its state as
   `State(x=pos.x_val, y=pos.y_val, up=-pos.z_val)`. `State.yaw_deg` defaulted
   to 0, so everything downstream saw an aircraft that always faced North.

Neither raised an error. At heading 0 the two frames coincide, so every run that
never turned behaved correctly by accident.

## The evidence

At scale 150, one forward delta of 0.01 is 1.5 m/s along the nose:

| Heading | Delta | Old Action4D | Correct Action4D |
|---|---|---|---|
| 0 (North) | forward 0.01 | vx 1.5, vy 0.0 | vx 1.5, vy 0.0 |
| 90 (East) | forward 0.01 | **vx 1.5, vy 0.0** (North) | vx 0.0, vy 1.5 (East) |
| 90 (East) | right 0.01 | **vx 0.0, vy 1.5** (East) | vx -1.5, vy 0.0 (South) |

`test_yaw_90_forward_flies_east` and `test_yaw_90_right_flies_south` failed on
the old mapping with exactly the bold values. The end-to-end flight test (the
real `main()` against a stand-in AirSim at heading 90) recorded a trajectory
whose `yaw_deg` was 0.0 on every tick before the fix and 90.0 after it.

## The fix

- The model's three deltas are read as (forward, right, up) and cross into the
  world frame once, through `guardrail.frames.from_body`, at the heading the
  frame was **captured** with (`openvla_raw_to_action(raw, scale, yaw_deg)`;
  `yaw_deg` has no default, so a caller cannot forget it).
- The control loop and the inference worker both build their state with
  `state_from_pose(pose)`, which reads the heading from the quaternion.
- Every logged step carries the state at capture, the raw output, the body
  action and the world action, and `replay_step()` recomputes the world action
  from the log alone.

Still unverified, and logged on every step as `lateral_sign`: whether the
Bridge arm's +y delta is the drone's right or its left.

## What still stands

The 14 July run (OpenVLA-7B, 4-bit, AirSimNH) kept the aircraft out of the
no-fly zone. That result stands: the Shield checked the world-frame action that
was actually sent, so what it kept out of the zone is what flew, whatever the
model meant by "forward". The demo commanded a yaw rate of 0. That run logged no
heading, so whether its "forward" pointed where the model intended cannot be
re-derived.

## The general lesson

A frame is part of a contract, and a comment is not a conversion. The grant
locks body frame; this project's Shield is world frame for a stated reason
(`guardrail/frames.py`). That deviation is safe only if every backend crosses
the boundary exactly once, through one function, at the right heading.
`guardrail/vla_backends.py` now says so where the next backend will be written.
