# ROS 2 rail report - shield ON

| Item | Value |
|------|-------|
| Path | vla node -> /vla/action_4d (body) -> safety_shield -> /shield/setpoint (body) -> mavlink_adapter -> /mavros/setpoint_raw/local -> MAVROS 2 -> ArduPilot |
| Autopilot | ArduCopter V4.5.7 (2a3dc4b7) (AUTOPILOT_VERSION via /mavros/vehicle_info_get) |
| Pilot | `guardrail.vla_stub.GotoPilot` |
| Policy | `fase3-sim-demo` `sha256:591674cc3f1506dfa1e082b9ae8da7435e2ed19a534c3a104fedaefdf576f726` |
| Policy source | bundle `bundles/fase3-sim-demo-v0.1.4.tar.gz` - signature **verified** |
| Rule clock | wall: +08:00 (Asia/Taipei (UTC+08:00, no daylight saving)) |
| Hot-apply API | not served |
| Action stream | 581 received; 0 stale spell(s) (timeout 0.5 s) |
| Pose stream | 9.94 Hz; 0 stale spell(s) (timeout 1 s), 0 s of ticks on a stale pose |
| GPS fix (`fix_ok`, /mavros/global_position/raw/fix) | 576 message(s); 0 tick(s) without a fix; X8 input on 0 tick(s) |
| Autopilot estimator (`estimator_ok`, EKF_STATUS_REPORT on /uas1/mavlink_source) | 575 report(s), 0 unhealthy; 0 tick(s) declared lost (threshold 0.8, FS_EKF_THRESH (read from the autopilot)) |
| Simulator parameters | at take-off {'SIM_WIND_SPD': 0.0, 'SIM_WIND_DIR': 180.0, 'SIM_WIND_T': 0.0, 'SIM_WIND_TURB': 0.0, 'SIM_GPS_DISABLE': 0.0, 'SIM_BATT_VOLTAGE': 12.600000381469727}; 1 change(s) in flight |
| Tick latency (e2e p50 / p99 / max) | 3.083 / 7.548 / 8.584 ms, budget 100 ms |
| Topology | `hil` |
| Target reached | NO |
| Scenario | bat1125_001_failsafe-low-battery, 1 of 1 event(s) applied |
| Ticks | 413 (101 flown, 312 held by the autopilot or a fault) |
| Shield lookahead | 5 s at 0.1 s (50 poses) |
| Shield interventions | 313 |
| Escalation FSM | Land (max state) |
| Autopilot took over | no |
| Autopilot fence parameters | {'FENCE_ENABLE': 1.0, 'FENCE_TYPE': 5.0, 'FENCE_ACTION': 1.0, 'FENCE_ALT_MAX': 30.0, 'FENCE_MARGIN': 2.0, 'AVOID_ENABLE': 0.0, 'source': '/mavros/param (read from the autopilot)', 'parm_file': {'FENCE_ENABLE': 1.0, 'FENCE_TYPE': 5.0, 'FENCE_ACTION': 1.0, 'FENCE_ALT_MAX': 30.0, 'FENCE_MARGIN': 2.0, 'AVOID_ENABLE': 0.0}, 'policy_ceiling_m': 20.0, 'policy_ceiling_source': 'altitude_envelope', 'fence_alt_max_raised_from': None, 'raise_allowed': True, 'sitl': True, 'sim_speedup_wait': {'sim_speedup': 1.0, 'autopilot_kind': 'sitl', 'waited_s': 3.52, 'wait_s': 40.0}, 'set_by': 'sitl defaults file (start_sitl.sh GUARDRAIL_FENCE) or this node', 'gcs': None, 'backstop_active': True, 'stricter_than_policy': False, 'plan_why': None} |
| Autopilot fence upload | {'kind': 'fence_upload', 'generation': 1, 'policy_hash': 'sha256:591674cc3f1506dfa1e082b9ae8da7435e2ed19a534c3a104fedaefdf576f726', 'fence_ids': ['nfz-square', 'rtl-zone'], 'items': 8, 'skipped': [], 'reason': 'scenario event(s): spawn_polygon_fence rtl-zone', 'backstop_active': True, 'home': [-35.3650576, 149.16523], 'home_local_ne': [-200.0, 0.009], 'origin_offset_m': [-200.0, 0.0], 'success': True, 'transferred': 8, 'read_back': 8} |
| Dynamic NFZ | not used |
| **Time inside NFZ** | **0.0 s** |
| **P0 KPI** | **PASS** |
