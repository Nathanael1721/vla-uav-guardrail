# SITL Guardrail report — shield ON

| Item | Value |
|------|-------|
| Autopilot | ArduPilot SITL (real flight code, GUIDED mode) |
| Adapter | MAVLink SET_POSITION_TARGET_LOCAL_NED @ 10 Hz |
| Command | `fly to the northeast pad at 6 m/s` |
| Policy | `fase3-sim-demo` `sha256:b665adbaac29186240d5a74a8fa36a94b69adae2db6035ca69d45c8e93e7f358` |
| Policy source | bundle `bundles/fase3-sim-demo-v0.1.4.tar.gz` - signature **verified** |
| Rule clock | wall: +08:00 (Asia/Taipei (UTC+08:00, no daylight saving)) |
| Hot-apply API | not served |
| Target reached | NO |
| Ticks | 271 |
| Shield interventions | 271 |
| Brakes | 0 |
| Dynamic NFZ | not used |
| Topology | `ardupilot-sitl-pymavlink` |
| Code revision | `28a594dd0c43` |
| Sim speedup | `1.0` |
| **Time inside NFZ** | **0.0 s** |
| Time outside altitude envelope | 0.0 s |
| **P0 violation escape rate** | **0.0** |
| Repairs | 4 |
| Mission outcome | RTL_triggered |
| **KPI-grade** | **no** |

Not KPI-grade because:

- topology is 'ardupilot-sitl-pymavlink', not the grant's 'hil' (nor a waivable 'dev'): functional-rail evidence, not a contractual KPI number
