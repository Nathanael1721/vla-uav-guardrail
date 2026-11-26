# ROS 2 rail report - shield ON

| Item | Value |
|------|-------|
| Path | vla node -> /vla/action_4d (body) -> safety_shield -> /shield/setpoint (body) -> mavlink_adapter -> /mavros/setpoint_raw/local -> MAVROS 2 -> ArduPilot |
| Autopilot | ArduCopter V4.5.7 (2a3dc4b7) (AUTOPILOT_VERSION via /mavros/vehicle_info_get) |
| Pilot | `guardrail.vla_stub.StubVLA` |
| Policy | `fase3-sim-demo` `sha256:b665adbaac29186240d5a74a8fa36a94b69adae2db6035ca69d45c8e93e7f358` |
| Policy source | bundle `bundles/fase3-sim-demo-v0.1.4.tar.gz` - signature **verified** |
| Rule clock | wall: +08:00 (Asia/Taipei (UTC+08:00, no daylight saving)) |
| Hot-apply API | not served |
| Action stream | 463 received; 0 stale spell(s) (timeout 0.5 s) |
| Pose stream | 8.71 Hz; 0 stale spell(s) (timeout 1 s), 0 s of ticks on a stale pose |
| GPS fix (`fix_ok`, /mavros/global_position/raw/fix) | 475 message(s); 0 tick(s) without a fix; X8 input on 0 tick(s) |
| Autopilot estimator (`estimator_ok`, EKF_STATUS_REPORT on /uas1/mavlink_source) | 475 report(s), 0 unhealthy; 0 tick(s) declared lost (threshold 0.8, FS_EKF_THRESH (read from the autopilot)) |
| Simulator parameters | at take-off {'SIM_WIND_SPD': 0.0, 'SIM_WIND_DIR': 180.0, 'SIM_WIND_T': 0.0, 'SIM_WIND_TURB': 0.0, 'SIM_GPS_DISABLE': 0.0, 'SIM_BATT_VOLTAGE': 12.600000381469727}; 0 change(s) in flight |
| Tick latency (e2e p50 / p99 / max) | 1.481 / 2.019 / 2.357 ms, budget 100 ms |
| Topology | `dev` |
| Target reached | yes |
| Scenario | none |
| Ticks | 113 (113 flown, 0 held by the autopilot or a fault) |
| Shield lookahead | 5 s at 0.1 s (50 poses) |
| Shield interventions | 109 |
| Escalation FSM | Brake (max state) |
| Autopilot took over | no |
| Autopilot fence parameters | {'FENCE_ENABLE': 0.0, 'FENCE_TYPE': 7.0, 'FENCE_ACTION': 1.0, 'FENCE_ALT_MAX': 100.0, 'FENCE_MARGIN': 2.0, 'AVOID_ENABLE': 3.0, 'source': '/mavros/param (read from the autopilot)', 'parm_file': {'FENCE_ENABLE': 1.0, 'FENCE_TYPE': 5.0, 'FENCE_ACTION': 1.0, 'FENCE_ALT_MAX': 30.0, 'FENCE_MARGIN': 2.0, 'AVOID_ENABLE': 0.0}, 'policy_ceiling_m': 20.0, 'policy_ceiling_source': 'altitude_envelope', 'fence_alt_max_raised_from': None, 'raise_allowed': True, 'sitl': True, 'sim_speedup_wait': {'sim_speedup': 1.0, 'autopilot_kind': 'sitl', 'waited_s': 0.0, 'wait_s': 40.0}, 'set_by': 'sitl defaults file (start_sitl.sh GUARDRAIL_FENCE) or this node', 'gcs': None, 'backstop_active': False, 'stricter_than_policy': False, 'plan_why': None} |
| Autopilot fence upload | no upload reported |
| Dynamic NFZ | not used |
| **Time inside NFZ** | **0.0 s** |
| **P0 KPI** | **PASS** |
