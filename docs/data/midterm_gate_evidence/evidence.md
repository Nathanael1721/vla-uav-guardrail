# Mid-term gate (2026-07-20): what the reference repository's git dates

Read on 16 Nov 2026 from the PI's reference repository as cloned beside this
one (`kuanting-vla-uav-guardrail/`, a separate git history, not part of this
repository). Dates are git author dates. Nothing here says the mid-term gate
was met: it records what existed, where and by whom, so the PI can rule.

Commands (run inside that clone):

    git log --all --format='%h | %an | %ad | %s' --date=short
    git branch -a --contains <commit>
    git ls-tree -r --name-only 9e039b0
    git show 9e039b0:packages/safety-shield/src/safety_shield/repair.py
    git show 9e039b0:ros2_ws/src/safety_shield_node/safety_shield_node/node.py
    git show 9e039b0:sim/README.md

| Commit | Date | Author | On | What it holds |
|---|---|---|---|---|
| 9e039b0 "1st commit" | 2026-07-05 | the PI | origin/main and every branch | packages/policy-dsl (policy_dsl.schema.json, models.py, ingest.py, ir.py of 158 lines, projection.py, tests); packages/safety-shield (repair.py with `AltitudeClamp` and `LateralProjection`, checker.py, audit.py); ros2_ws/src/safety_shield_node/safety_shield_node/node.py (class `ShieldNode`: subscribes to /vla/action_4d and two MAVROS position topics, publishes /shield/setpoint and /shield/intercept); ros2_ws/src/mavlink_adapter; demo/mid_term_demo.py; sim/README.md, which says the perception rail "(Project AirSim + HIL bridge) is deferred per the 2026-06-30 fallback gate" |
| 2ad3250 | 2026-07-08 | the developer | fork branches | fix(shield): trend-aware recovery for vehicles already inside an NFZ |
| ecf5f31 | 2026-07-08 | the developer | fork branches | demo: kinematic harness for the functional rail (no ROS needed) |
| 29a96d2 | 2026-07-08 | the developer | fork branches | sim: SITL+ROS bring-up (sim/run_sitl_ros_demo.sh, sim/README-sitl-demo.md) and a finding report |
| c0214e2 | 2026-07-09 | the developer | fork branches | sim: flight_controller node (arm and take-off via MAVROS services) |
| b3112ad | 2026-07-09 | the developer | fork branches | demo(sitl): flown A/B over ArduPilot SITL via pymavlink (Shield off FAIL / on PASS) |
| b0b0f22 | 2026-07-10 | the developer | fork branches | demo(airsim): the Shield flies Project AirSim (perception rail), A/B PASS |

Limits of this record:

- 9e039b0 is the PI's work (the reference implementation). The delivered
  code in this repository (guardrail/, sitl/) was written separately; its
  own first commit is 2026-08-24 and its typed IR is of 2026-10-06
  (docs/FINAL-REPORT.md section 7.1).
- The developer's July commits are on fork branches (`fix/trend-aware-recovery`,
  `experiment/dynamic-nfz-spawn-on-top`, `demo/projectairsim`,
  `contrib/ntut-vla-fine-tuning`), not on origin/main.
- The flown A/B of 9 July went over pymavlink, not through the ROS 2 node.
- The Gazebo rail the gate also names was not flown before 2026-11-02 (ARCH-16).
- When 9e039b0 was pushed to its origin was not checked.
