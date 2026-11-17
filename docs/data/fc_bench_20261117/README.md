# Flight-controller bench, 17 Nov 2026

A bench test of the real flight controller, not a flight. The aircraft was
never armed.

- **Flight controller:** CUAV V5 Nano (board fmuv5), ArduCopter 4.7.1,
  on the desk over USB (COM8), no GPS fix.
- **Bundle:** bundles/fase3-sim-demo-v0.1.4.tar.gz.

## What was run

1. Every parameter was backed up (`params_before.param`). The FC held no
   fence items before the test.
2. The EKF origin and home were set to the bundle's frame origin with
   SET_GPS_GLOBAL_ORIGIN and DO_SET_HOME (ack 0). The FC has no GPS fix, so
   this was needed for the fence upload.
3. `sitl/gcs_backstop.py` ran its own procedure on the FC (`gcs_backstop.json`):
   - the FENCE_* parameters were set and read back;
   - the bundle's keep-out zone `nfz-square` was uploaded as 4 exclusion
     items and read back, and the read-back matched.
4. Each FSM mode was requested by MAVLink with MAV_CMD_DO_SET_MODE. Every
   request was accepted (COMMAND_ACK 0), and the next heartbeat reported the
   requested mode:

   | Mode | custom_mode |
   |---|---|
   | GUIDED | 4 |
   | BRAKE | 17 |
   | LOITER | 5 |
   | RTL | 6 |
   | LAND | 9 |
   | STABILIZE | 0 |

5. Restore. Changed parameters were restored from the backup, the fence was
   cleared (0 items), and the mission was cleared (MIS_TOTAL 0; DO_SET_HOME
   had stored a home item). The FC was then rebooted. `fc_verify.json` lists
   the parameters that still differ from the backup:
   - the boot-time gyro and baro calibration values and STAT_BOOTCNT, which
     change on every boot;
   - the MAV1_* stream rates, which were 4 in the backup and 0 after the
     reboot. These look like live values that were never saved, but that has
     not been checked.

The run was split across processes because COM8 stayed held after step 3:
`fc_bench.py` covers steps 1-3, and `fc_restore.py` and `fc_verify.py` cover
steps 4-5 and the final check. `fc_bench_summary.json` is the hand-written
summary of all three.

## What this shows and what it does not

**Shows:** the GCS backstop procedure (WP3-10) and the FSM's mode requests
(WP3-07) are accepted by a real ArduCopter flight controller.

**Does not show:**
- a flight;
- the companion computer driving the FC (the mode requests came from a bench
  script, not from the FSM over MAVROS 2);
- the flight topology (ARCH-14).
