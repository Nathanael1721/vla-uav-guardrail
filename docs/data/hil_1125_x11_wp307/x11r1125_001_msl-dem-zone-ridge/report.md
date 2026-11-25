# ROS 2 rail report - shield ON

| Item | Value |
|------|-------|
| Path | vla node -> /vla/action_4d (body) -> safety_shield -> /shield/setpoint (body) -> mavlink_adapter -> /mavros/setpoint_raw/local -> MAVROS 2 -> ArduPilot |
| Autopilot | ArduCopter V4.5.7 (2a3dc4b7) (AUTOPILOT_VERSION via /mavros/vehicle_info_get) |
| Pilot | `guardrail.vla_stub.GotoPilot` |
| Policy | `msl-dem-zone` `sha256:b80ee30f89eaaf99c9354418f3939fd8a11c29c2206875f162332df5fd3f7a35` |
| Policy source | bundle `bundles/msl-dem-zone-v0.1.0.tar.gz` - signature **verified** |
| Rule clock | wall: +08:00 (Asia/Taipei (UTC+08:00, no daylight saving)) |
| Hot-apply API | not served |
| Action stream | 845 received; 0 stale spell(s) (timeout 0.5 s) |
| Pose stream | 9.97 Hz; 0 stale spell(s) (timeout 1 s), 0 s of ticks on a stale pose |
| GPS fix (`fix_ok`, /mavros/global_position/raw/fix) | 851 message(s); 0 tick(s) without a fix; X8 input on 0 tick(s) |
| Autopilot estimator (`estimator_ok`, EKF_STATUS_REPORT on /uas1/mavlink_source) | 850 report(s), 12 unhealthy; 0 tick(s) declared lost (threshold 0.8, FS_EKF_THRESH (read from the autopilot)) |
| Simulator parameters | at take-off {'SIM_WIND_SPD': 0.0, 'SIM_WIND_DIR': 180.0, 'SIM_WIND_T': 0.0, 'SIM_WIND_TURB': 0.0, 'SIM_GPS_DISABLE': 0.0, 'SIM_BATT_VOLTAGE': 12.600000381469727}; 0 change(s) in flight |
| Tick latency (e2e p50 / p99 / max) | 4.7 / 6.519 / 7.833 ms, budget 100 ms |
| Topology | `hil` |
| Target reached | yes |
| Scenario | x11r1125_001_msl-dem-zone-ridge |
| Ticks | 667 (667 flown, 0 held by the autopilot or a fault) |
| Shield lookahead | 5 s at 0.1 s (50 poses) |
| Shield interventions | 0 |
| Escalation FSM | Normal (max state) |
| Autopilot took over | no |
| Autopilot fence parameters | {'FENCE_ENABLE': 1.0, 'FENCE_TYPE': 5.0, 'FENCE_ACTION': 1.0, 'FENCE_ALT_MAX': 86.7249984741211, 'FENCE_MARGIN': 2.0, 'AVOID_ENABLE': 0.0, 'source': '/mavros/param (read from the autopilot)', 'parm_file': {'FENCE_ENABLE': 1.0, 'FENCE_TYPE': 5.0, 'FENCE_ACTION': 1.0, 'FENCE_ALT_MAX': 30.0, 'FENCE_MARGIN': 2.0, 'AVOID_ENABLE': 0.0}, 'policy_ceiling_m': 81.72500000000002, 'policy_ceiling_source': "altitude_envelope, above home (583.775 m MSL): MSL ceilings less home, AGL ceilings plus the ground's highest 605.5 m MSL less home", 'fence_alt_max_raised_from': 30.0, 'raise_allowed': True, 'sitl': True, 'sim_speedup_wait': {'sim_speedup': 1.0, 'autopilot_kind': 'sitl', 'waited_s': 5.62, 'wait_s': 40.0}, 'set_by': 'sitl defaults file (start_sitl.sh GUARDRAIL_FENCE) or this node', 'gcs': None, 'backstop_active': True, 'stricter_than_policy': False, 'plan_why': 'FENCE_ALT_MAX 30 m is below the policy ceiling 81.725 m + 5 m: the backstop would fire on a flight the Shield considers legal'} |
| Autopilot fence upload | {'kind': 'fence_upload', 'generation': 0, 'policy_hash': 'sha256:b80ee30f89eaaf99c9354418f3939fd8a11c29c2206875f162332df5fd3f7a35', 'fence_ids': [], 'items': 0, 'skipped': [{'id': 'msl-ridge-nfz', 'why': 'altitude band 22.225-116.225 m above home does not cover the fence column 0-86.725 m; a 2-D autopilot fence would be stricter than the policy'}], 'reason': 'take-off', 'backstop_active': True, 'home': [-35.3636652, 149.1652299], 'home_local_ne': [-45.0, 0.0], 'origin_offset_m': [-45.0, -0.01], 'success': True, 'transferred': 1, 'read_back': 0, 'push_success': False} |
| Dynamic NFZ | not used |
| **Time inside NFZ** | **0.0 s** |
| **P0 KPI** | **PASS** |
