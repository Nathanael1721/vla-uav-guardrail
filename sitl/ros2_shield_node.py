"""
ROS 2 node: the Safety Shield, wired the way the grant's interface locks it.

    [vla node] --/vla/action_4d--> [THIS NODE] --/shield/setpoint--> [mavlink_adapter] --/mavros/setpoint_raw/local--> MAVROS 2 --> ArduPilot
                Float32MultiArray                Float32MultiArray                      mavros_msgs/PositionTarget
                BODY [vx_fwd, vy_right,          BODY, post-Shield                      FRAME_LOCAL_NED
                vz_up, yaw_rate] 10 Hz

THE FRAME (WP3-04, ARCH-04, WP2-17). The grant locks the VLA action as
"a = (vx, vy, vz, yaw_rate) # body-frame velocities + yaw rate" and puts the
NED conversion in the MAVLink adapter after the Shield. This node hands the
body-frame action to the Shield as it arrives (Shield.filter_body, with the
heading of the same pose the adapter reads) and publishes the body-frame
action it returns on /shield/setpoint. Since 2026-11-04 the Shield takes the
body frame at its own boundary; until then this node rotated the action into
the world frame (guardrail.frames.from_body) before Shield.filter() and back
(frames.to_body) after - the same exact rotation, now inside the Shield.
With --arc-forecast on (2026-11-17; off by default) the Shield forecasts a
body action with a yaw rate along the arc it flies, not along its world
direction at t = 0; a zero yaw rate (the stub pilot) is forecast the same.
sitl/mavlink_adapter_node.py turns /shield/setpoint into NED - the single
body -> local-NED crossing - and owns every MAVROS command after take-off
(SET_MODE, fence upload). Until 2026-10-06 this node published a Twist on
/mavros/setpoint_velocity/cmd_vel_unstamped and did the ENU conversion itself
(audit cards WP3-03, WP3-09, ARCH-05).

WHAT EVERY RUN RECORDS (WP2-11, WP2-15, WP1-24, WP4-13, WP4-20, WP3-11, WP3-17)

  flight_log.jsonl  one row per tick: position, heading, the action in both
                    frames, violations, repairs, `unsafe` + `unsafe_rules`
                    (is the POSITION illegal), policy_hash + generation,
                    shield_ms (the Shield's own time), the declared subject
                    (tgt_x/tgt_y), the FSM state, the autopilot's mode, the
                    episode id, and `flown`: whether anything was sent on
                    that tick (false while the autopilot holds the aircraft,
                    e.g. a GeoFence RTL - such a tick cannot be an escape).
  policy_g<N>.json, csp_g<N>.json
                    the policy and the typed CSP of every generation, written
                    at take-off and after each hot-apply (guardrail.replay.
                    EpisodeRecord).
  shield_audit.jsonl
                    this episode's Shield audit only (the grant's file name;
                    audit.jsonl until 2026-11-02); a stale one is moved to
                    _previous/, never appended to.
  events.jsonl      episode start, generations, hot-applies, mode requests and
                    changes, fence uploads, FSM transitions, autopilot text.
  metrics.json      incl. hil_evidence (the MAVROS chain, the autopilot's
                    own version, the host, the link) and the pilot record.
  manifest.json     the grant's six fields; topology from the evidence.
  <tag>.replay.tar.gz  written here, and re-packed by run_ros2_demo.sh once
                    the rosbag is closed so it carries bag/ as well.

TOPOLOGY. One desktop running ArduPilot SITL + MAVROS 2 + this node is the
grant's `dev`. On a Jetson Orin, with ArduPilot SITL on the desktop, the same
node gathers the Orin's evidence (device tree, L4T / JetPack, the processes
running on the Orin, the VLA node's own host from /vla/identity, MAVROS's
fcu_url) and its manifest says `hil`, whether MAVROS runs on the desktop (the
grant's table) or on the Orin (docs/RUNBOOK-orin-hil.md); with a hardware
autopilot it says `flight`. The label comes from that evidence
(guardrail.manifest.detect_topology), never from a flag, so moving the stack
from the desktop to the Orin to the drone changes no code. The evidence is
gathered by sitl/mavros_evidence.py, which the mission-style node
(sitl/ros2_mission_node.py) uses too (2026-11-06).

GEOFENCE BACKSTOP. At take-off the node reads FENCE_ENABLE, FENCE_ALT_MAX,
AVOID_ENABLE and the rest back from the autopilot, raises FENCE_ALT_MAX above
the policy's ceiling when it sits below it, and publishes each generation's
keep-out zones to the adapter - except a zone the aircraft is in or near,
which is held back until the aircraft is clear, so a zone hot-applied on top
of the aircraft is the Shield's to resolve, not an instant autopilot RTL.
When the ground station set the fence (2026-11-03, WP3-10: sitl/
gcs_backstop.py wrote <out>/gcs_backstop.json before this node started,
run_ros2_demo.sh GCS_BACKSTOP=1), this node changes no fence parameter -
the GCS owns them, as the grant has it - and records the GCS's own read-back
beside its own (hil_evidence.autopilot_fence.gcs, `matches_autopilot`).

SHIELD HORIZON. 5 s at 0.1 s steps, 50 future poses, as the grant's Safety
Shield page locks it (3 s at 0.5 s until 2026-10-07).

ESCALATION (WP3-07 wiring). With the Shield on, every decision goes through
guardrail.fsm.EscalationFSM (docs/DESIGN-escalation-fsm.md, "Integration").
Its mode requests go to the adapter on /shield/mode_request. Theta is applied
to the Shield's per-operator Repair.magnitude_m (the default basis,
guardrail.fsm.DEFAULT_THETA_HORIZON_S); `--theta-horizon-s 0.1` selects the
one-period reading (the setpoint change over one 0.1 s control period, a PI
question, docs/DESIGN-escalation-fsm.md). The source is recorded in
metrics.json as `fsm.theta_basis`. Flown with magnitude_m on
2026-10-07, the default mission left Normal at the first tick (a forecast
penetration of 8.1 m against theta 2.0 m), and ArduCopter's LOITER, with no
RC input on SITL, descended to the ground (docs/DESIGN-ros2-interface.md,
"Escalation FSM wiring"); the FSM's LOITER is now flown as ArduCopter's
BRAKE (the adapter's hold mode, mavlink_adapter_node.DEFAULT_HOLD_MODE). The
FSM also gets the position's illegal depth (Shield.unsafe_depth) for X6's
progress and X7's losing-ground tests, logged as `unsafe_depth_m`, and the
caps the Shield clamped against (Shield.applied_caps). Mode requests are
recorded in the FSM's words and ArduCopter's (`mode_requests`,
`autopilot_mode_requests`; mavlink_adapter_node.ModeRequests). This node
runs the one FSM: its Shield is built without an FSM of its own
(guardrail.replay.rail_shield), because only the node sees the autopilot, and
each audit record carries the node's FSM verdict and its log row's tick
number (guardrail.replay.audit_tick).

A STRESS SCENARIO ON THIS RAIL (2026-11-06). The stress harness
(experiments/sweep_scenarios.py --backend hil / sitl) flies the scenario
library here, one scenario per mission. Until this date the node took a
target and a seed but no start, no pilot and no events
(docs/RUNBOOK-orin-hil.md, step 14):
  * --start X,Y[,UP]: the policy-frame position of the autopilot's home. The
    harness boots SITL there (SITL_HOME, the policy frame origin's projection
    of (X, Y)); every pose is offset by it, the home's position goes into
    the fences document (`home_policy_ne`, which the adapter projects the
    zones about), and the node refuses to arm unless the autopilot's own home
    lies within HOME_TOL_M of it (check_home, metrics.json `home_check`).
    UP is the take-off altitude, held before the mission starts.
  * --target X,Y[,UP], --no-target, --reach-m: the scenario's goal (none for
    a constant pilot: `reached` null) and its tolerance.
  * --events FILE: the scenario's ScenarioEvents, applied in the tick once
    the mission time reaches each, with the harness's own code
    (guardrail.scenario_events), the Shield given t=None as REST gives it;
    each is a `scenario_event` in events.jsonl and writes its generation.
  * --scenario-meta FILE: the harness's record, in metrics.json `scenario`
    and `scenario_family` (tools/kpi_report.py's row).
  * Each row carries the rule clock's reading (`clock`), the zones holding
    the position (`zones_in`, `zone_d_m`), the same judgements under the
    previous tick's rules and clock (`unsafe_prev_rules`, `zones_in_prev`:
    _prev_judgement) and the rule changes the position met (`rule_events`),
    which the harness reads to tell a breach from a rule that switched on
    under the aircraft; since 2026-11-07 also the decision's trace words the
    harness counts (`shield_trace`: window_opens_on / window_opened_on,
    cap_refused / floor_refused; shield_trace_fields).
The pilot laws are the VLA stub's (sitl/ros2_vla_stub_node.py --pilot,
--action), which reads the same arguments.

THE GPS FIX AND THE SIMULATOR'S STRESSORS (2026-11-07, X-14).
  * State.fix_ok from /mavros/global_position/raw/fix: MAVROS 2's
    global_position plugin (core MAVROS, loaded on both rails without
    mavros_extras) publishes it from the autopilot's GPS_RAW_INT, with
    status STATUS_FIX when fix_type > 2 and STATUS_NO_FIX otherwise ("GP: No
    GPS fix"). fix_ok_from: True for STATUS_FIX or better, False for
    STATUS_NO_FIX, None before the first message and when the last is older
    than FIX_STALE_S - a rail without the topic flies exactly as before. The
    Shield judges X8's input from it (ShieldDecision.position_lost, its dead
    reckoning on what this node sent: Shield.note_flown) and this node's FSM
    takes X8 (RTL) - since 2026-11-12 only while the autopilot's own
    estimator verdict is not reported ("THE AUTOPILOT'S ESTIMATOR"). pos_sigma_m is NOT fed: with 1 m margins X8's sigma
    branch needs sigma <= 0.33 m (SITL reports 0.3 m; a field receiver
    reports metres, and every mission passing near a zone would return);
    sizing margins to the rail's accuracy is a policy decision left open.
    Rows carry fix_ok, fix_status, fix_age_ms and the Shield's
    position_lost; metrics.json `fix_quality`.
  * The simulator's stressors are set on the SIMULATOR'S side
    (sitl/sitl_stressors.py, on the desktop beside SITL), never here. This
    node only (a) announces its mission clock: on a scenario mission
    (--scenario-meta with a run nonce) a STATUSTEXT `GRDL T0 <nonce8> t=<s>`
    through /mavros/statustext/send at t = 0 and each of MARKER_TIMES_S, the
    t = 0 the simulator side schedules against; (b) reads SITL's stressor
    parameters back at take-off (SIM_SPEEDUP answering: SITL only) and
    records every change the autopilot broadcasts (/mavros/param/event) on
    its own mission clock - the measured alignment the stress harness checks
    (metrics.json `sim_params`).

THE AUTOPILOT'S ESTIMATOR (2026-11-12). The pose this node hands the Shield
is the autopilot's own navigation estimate (EKF3, through MAVROS), so whether
it is valid is the autopilot's call, not the GPS receiver's:
  * State.estimator_ok from ArduPilot's EKF_STATUS_REPORT (MAVLink 193),
    read from MAVLINK_SOURCE_TOPIC (MAVROS's raw MAVLink feed to the UAS: no
    MAVROS 2 plugin publishes the message). EstimatorHealth: healthy while
    EKF_POS_HORIZ_ABS is set, EKF_CONST_POS_MODE and EKF_UNINITIALIZED are
    clear and the horizontal-position and velocity test ratios are below
    FS_EKF_THRESH (read from the autopilot at take-off, 0.8 when unread);
    False once unhealthy reports have outweighed healthy ones for 1 s, the
    autopilot's own EKF failsafe rule (ArduCopter ekf_check); None with no
    report, or none for longer than the pose timeout.
  * The Shield (Shield._position_verdict): True certifies the position even
    with the GPS fix lost - EKF3 dead-reckons on its IMU, and the Shield
    holds no command dead reckoning of its own against it; False is X8 (the
    position certifies no hard P0 zone); None falls back on fix_ok and the
    dead reckoning, as before.
  * Measured on the rig (731 flights of nt1111, sm1111, sm1110 and rh03):
    the 701 flights without a GPS denial never reported an unhealthy
    estimate while armed (test ratios at most 0.05 / 0.08); in the 30 GPS
    denials EKF3 kept its absolute position for 6.8-6.9 s, then dropped it
    for 0.1-0.2 s until the GPS returned at 7 s (23 of the 24 7 s denials),
    and was up to 1.47 m off SITL's truth meanwhile: a hard P0 margin
    covers a denial only if it covers that error (a policy question).
  * Rows carry estimator_ok, ekf_flags, ekf_pos_var, ekf_vel_var,
    ekf_age_ms once a report has arrived; metrics.json
    `fix_quality.estimator`; events.jsonl `ekf_failsafe` (FS_EKF_THRESH and
    FS_EKF_ACTION as read). SROS 2: sitl/sros2_policy.xml grants the
    subscription; a keystore made before it refuses it, recorded in
    metrics.json (`fix_quality.estimator.refused`, `sim_params.inputs_refused`),
    and the node flies on fix_ok as before.

THE RTL WATCH (2026-11-14, WP3-07 edge G6). The grant's RTL -> Land edge,
"RTL fails (battery / blocked path)" (Safety Shield p4), takes `rtl_failed`
from the autopilot's side, and until this date this node passed False on
every tick: G6 could not fire on a rail, an RTL that made no headway home
into a wind beyond authority stayed an RTL, and the hil campaigns refused
the two library cells that take G6 (failsafe-rtl-blocked,
failsafe-low-battery). Now RTLFailureWatch judges every tick the FSM is in
RTL (and is reset on every other tick); the RTL fails on the first tick on
which any of these holds:
  * blocked path - guardrail.rtl_path.check_leg on the Shield's own hard,
    enforced P0 zones and band in force now (zones_from_shield,
    band_from_shield): the climb to the return altitude or the straight leg
    home crosses a zone's ring or leaves the band. The stress harness's own
    test, from the FSM's RTL on, as the harness judges it.
  * no progress home - while the autopilot REPORTS RTL (/mavros/state) and
    the aircraft is farther from home than RTL_ALT / RTL_CONE_SLOPE (5 m by
    default; RTL_HOME_RADIUS_M at least): the climb plus the distance home
    (rtl_path.remaining_to_home) fell by less than 0.5 m over 5 s
    (rtl_path.ProgressMonitor: the grant's T, X6's 0.1 m/s). Inside that
    radius ArduCopter's cone cuts the climb and the RTL loiters and lands
    over home: an RTL that is ending, never judged.
  * battery - ArduPilot's own battery failsafe at CRITICAL or worse:
    BATTERY_STATUS (MAVLink 147) charge_state, read from
    MAVLINK_SOURCE_TOPIC beside EKF_STATUS_REPORT (BatteryHealth; a report
    older than BATTERY_STALE_S is not a report). LOW does not fail an RTL:
    it is ArduPilot's reason to start one. With BATT_CRT_VOLT and
    BATT_CRT_MAH at 0 (the copter default) ArduPilot never declares
    CRITICAL and this reason cannot fire; since 2026-11-17 the battery cell
    (failsafe-low-battery) gives the SIMULATED vehicle a battery fail-safe
    setup and drops SIM_BATT_VOLTAGE from the simulator's side
    (sitl/sitl_stressors.py; experiments/sweep_scenarios.py RAIL_BATTERY),
    never from this node.
  * RTL not entered - the autopilot has not reported RTL
    RTL_ENTRY_TIMEOUT_S (5 s, five heartbeats) after the FSM entered it.
Every threshold errs toward NOT failing a return: the progress test waits
for the autopilot's own RTL and a full window and leaves the end of the
return alone; nothing but these four reasons fails one; an error while
reading the rules is recorded and fails nothing. RTL_ALT, RTL_CONE_SLOPE,
BATT_CRT_VOLT, BATT_CRT_MAH and BATT_FS_CRT_ACT are read at take-off
(read_rtl_params; ArduCopter's defaults when unread) and recorded
(events.jsonl `rtl_watch`). Each judged row carries rtl_failed and
rtl_confirmed (and the reasons when it failed, also an `rtl_failed`
event); metrics.json `rtl_watch` has the configuration, the failures and
the battery reports. G6's own reason in the FSM record names them ("RTL
failed: ...").

THE PILOT'S DECISION PERIOD (2026-11-17, WP3-07). guardrail/fsm.py's
FSMConfig.for_decision_period (decided 2026-11-02, best practice) scales
N-in-T's window for a pilot that decides less often than 10 Hz: T = max(T,
N x period x 1.5), so a pilot deciding every 3 s can still reach N onsets in
T. Until this date no rail applied it. `--decision-period-s S` declares the
pilot's period; the FSM is built with for_decision_period(S) over the
--fsm-config thresholds whenever that widens T (S > T / (1.5 x N): 1.11 s
for the grant's N = 3, T = 5 s, 0.83 s for N = 4), and otherwise, or with
the option left out (the default: the 10 Hz stub every KPI campaign flies),
with the thresholds and their hash exactly as before (fsm_config_for_pilot).
metrics.json `fsm.decision_period` says which. Only a hand-launched node
takes it (sitl/run_ros2_demo.sh EXTRA): no campaign path passes it - the
harness's rail_node_args never adds it and rail_extra_args accepts only the
hot-apply flags.

ZONE DWELL AND COST MAP (2026-10-11). `nfz_s` is the count every rail shares
(guardrail.replay.zone_dwell, read from the Shield's rule checker): polygon
and circle fences and every dynamic_nfz that does not move, the hot-applied
zone from when it went live; a moving zone is named in
`nfz_moving_not_counted` instead of being scored where it was declared.
`--cost-map` gives each generation's CSP its cost map (EpisodeRecord).

Run (after `source /opt/ros/jazzy/setup.bash`; normally via run_ros2_demo.sh
or the launch file in sitl/ros2_ws):

    ~/venv-ros/bin/python sitl/ros2_shield_node.py --shield on [--dynamic]
    ~/venv-ros/bin/python sitl/ros2_shield_node.py --bundle bundles/fase3-sim-demo-v0.1.3.tar.gz
    ~/venv-ros/bin/python sitl/ros2_shield_node.py --policy P.yaml --dev-yaml

POLICY. `--bundle` flies the signed bundle (guardrail/bundle.py): the policy
the run used is then the issued artefact, not whatever the YAML says today.
Since 2026-10-12 a YAML policy flies only with `--dev-yaml`, recorded in the
policy source (`dev_yaml: true`, in metrics.json, kpi.json and the replay
index) and as a reason the run is not KPI-grade; with neither option the node
flies the signed bundle of policies/sim_demo_policy.yaml when it is in
bundles/ (tools/build_flight_bundles.py), and refuses otherwise
(guardrail.manifest.flight_policy_choice). The VLA node announces its own
identity and policy hash on /vla/identity; a mismatch with this node's policy
is logged and recorded.

RULE CLOCK (2026-10-12). Until this date the node gave neither its Shield nor
its CSPs a clock, so every scheduled rule was in force and every
time_window_switch inert in flight. Now one clock (guardrail.replay.
rule_clock) goes to both: `--clock-start ISO8601` scripts it (it advances
with the mission time, from the start); without it the wall clock in the
mission's time zone (`--clock-tz`, default Taiwan's +08:00), read as the
policy's naive local time. Which clock a run used is in the episode_start
event and metrics.json `rule_clock`.

HOT-APPLY ON THE FLYING SHIELD (2026-10-12). `--hotapply-port` serves the REST
hot-apply API (guardrail.api.FlightHotApply), started on the pad so a listener
that cannot start stops the run before arming: 127.0.0.1 by default, mutual
TLS with the harness CA (`--hotapply-pki`, the folder
`python -m guardrail.harness_service gen-pki` writes) for any other address.
The Shield's rules are swapped under the lock the control tick holds (the
REST server runs on its own thread beside the rclpy executor); each accepted
event writes its generation's policy and CSP, re-publishes the fences
(holding back a zone the aircraft is near) and records its receipt-to-swap
time against the grant's 50 ms target. After the episode ends it answers 503.

STALE ACTIONS (2026-10-12). Until this date a pilot whose action stream
stopped left the node flying its last action until the time cap. If no
/vla/action_4d arrives for `--action-timeout-s` (default 0.5 s: five of the
grant's 10 Hz ticks), the node holds the aircraft the way the escalation FSM
holds it: LOITER through the adapter, flown as ArduCopter BRAKE, after one
tick of the Shield's own answer to a stop; the FSM does not step while held,
and the pilot's next action returns the aircraft to GUIDED. Every spell is in
events.jsonl and metrics.json `action_watchdog`; rows carry `action_stale`.

STALE POSE (2026-10-19). Until this date nothing watched
/mavros/local_position/pose: a position that stopped while the setpoints
still went through was re-checked every tick and repeated in the rows and
the trajectory, so zone dwell could read 0 for a flight that crossed a zone.
If no pose arrives for `--pose-timeout-s` (default 1.0 s), the node holds
the aircraft as for a quiet action stream, asks for the streams again at
once, and the next pose releases the hold (GUIDED only once no hold reason
is left, and never after a takeover). The FSM does not step on a stale pose
in any state - with the FSM in Loiter or a fail-safe there is no hold, and it
went on stepping until the review of 2026-10-19 - and its clean streaks
restart (EscalationFSM.unobserved), so a recovery (G7 / G8) needs T_recover
of fresh clean ticks. Rows carry `pose_age_ms` and `pose_stale`;
metrics.json `pose_watchdog`; a run with a stale spell is not KPI-grade, and
report.md says how long the checks did not see the aircraft.

TAKEOVER (2026-10-19). A takeover is any change to a mode this node did not
ask for, out of GUIDED or out of a hold it asked for (BRAKE -> LAND), as on
the perception rail. Once the autopilot or a pilot has the aircraft the node
sends no mode request at all: no GUIDED when actions resume, and no
mission-end LAND whatever the mode.

SETPOINT DELIVERY (2026-11-17). Until this date nothing compared what this
node published on /shield/setpoint with what the MAVLink adapter sent to
MAVROS: in hil run ntFIN2_175_dyn-nfz-spawn-ahead the adapter sent none of
245 setpoints (25.6 s) and the run was graded on them. The adapter now
reports every send and beats once a second on /mavlink_adapter/sent; every
publication and report goes into a guardrail.delivery.DeliveryMonitor. Rows
carry `delivery_wait_ms` (how long the oldest publication no send has
followed has waited at this tick; None when none waits) and
`adapter_n_sent`; metrics.json `setpoint_delivery` lists every publication
no adapter send followed for over DELIVERY_GAP_S (0.5 s) as a gap with its
span, the setpoints published in it and the adapter beats that arrived in
it, and counts the publications no send report answered (`shortfall`: drops
while other sends continued). A run with a gap or a shortfall over
DELIVERY_SHORTFALL_MAX is not KPI-grade (guardrail.manifest.is_kpi_grade).
What is checked is the hop to the adapter's publish() on
/mavros/setpoint_raw/local, not MAVROS-to-autopilot (guardrail/delivery.py,
WHAT IS CHECKED). Each publication's monitor time is read just before its
publish(). Under SROS 2 the topic needs a keystore signed since this date
(sitl/sros2_env.sh refuses an older one; under Enforce this node would die
at construction). The node does
not react in flight: when setpoints stop, ArduCopter's own GUIDED setpoint
timeout stops the aircraft (ntFIN2_175: the speed held about 2 m/s for about
3 s into the gap, the GUIDED timeout, then fell, to about zero roughly 5 s
into it), and this node's mode requests go through the same
adapter, so a BRAKE asked for there would wait on the stalled node; asking
MAVROS directly would bypass the adapter's single mode path (WP3-09) on
evidence from one run. The record makes the next occurrence measurable.

TICK LATENCY (2026-10-12). Each row records `e2e_ms` (an action's receipt to
the publication of the first setpoint flown from it), `action_age_ms` (the
age of the action a published setpoint came from) and `tick_ms` (the tick
callback's own time to publication); metrics.json `tick_latency` gives p50 /
p95 / p99 / max against the grant's 100 ms monitor tick budget. The receipt
is the /vla/action_4d callback's entry on this node's monotonic clock; the
DDS hop before it and the adapter and MAVROS after the publication are not
in e2e. Until 2026-11-11 the node ticked on its own 10 Hz timer, so e2e
included the wait for the next tick, up to one period on top of the
processing (sm1111: tick max 95.98 ms, e2e p99 106.1 ms).

THE TICK ON THE ACTION (2026-11-11; --tick-trigger action, the default). A
valid pilot message runs the tick at once and a 40 Hz poll keeps the 10 Hz
watchdog for a quiet stream (TickScheduler, THE TICK SCHEDULE), so e2e is
the tick's own work; the tick itself, the FSM's time-based rules, the hold
planner's per-tick slices and the stale-action rule are unchanged, at the
pilot's 10 Hz. Work that does not decide the setpoint now follows its
publication (the audit record and events.jsonl appends, the intercept
topic, a held-back fence's re-check, the publisher count, the stream-rate
re-assert), and Python's full garbage collection - the ~110 ms spike near
t = 130 s of every long mission of ge1110 - is run there too, paced, never
inside a tick (GcPacer, THE GARBAGE COLLECTOR; --gc-pacing). Rows carry
`tick_trigger`; metrics.json `tick_schedule` and `gc`. --tick-trigger timer
and --gc-pacing off fly the old behaviour (an A/B on the rig).

THE START-UP TICK (2026-11-13). On the final nightly ntFIN2 (116214f) the
only ticks over 100 ms end to end were 13 of 171,393, every one row 2 of its
mission (t 0.101 s, "deferred", e2e 100.6-121.9 ms). start_mission does
~90 ms of work without spinning (the Shield warm-up, the collector's start),
so the pilot message published meanwhile waited in the DDS queue and was
delivered just after t = 0 (+2-3 ms), ticked as if live, and the pilot's
next message, 12-18 ms later, was taken for a pilot faster than 1 / min_gap
and deferred to the poll 80-86 ms on (row 2 of 418 of the 583 missions; the
13 are those whose deferred tick took 15-36 ms of work). Now that backlog is
delivered BEFORE the mission clock starts (START_BACKLOG_S, a bounded drain
of the executor), a message received before the start is held and never
ticked, the first message after it ticks at once whatever its gap to the held
one, and the first watchdog tick gives a live stream the slack it gives one
after a message's tick (TickScheduler). The held action is flown only by that
watchdog, for a quiet stream, under the stale-action rule as every tick.

CAMERA TO SETPOINT (2026-11-11, WP3-13). With the VLA bridge, each row whose
setpoint was flown from a camera-backed action carries `camera_age_ms`: the
age of the camera frame behind it at the setpoint's publication, the
bridge's decision_age_s plus this node's receipt-to-publication, two
durations on one monotonic clock each (THE CAMERA'S AGE AT THE SETPOINT);
metrics.json `camera_to_setpoint` gives p50 / p95 / p99 / max, and with the
stub says there is no camera behind the actions instead of a zero.

A REAL VLA IN THE SLOT (2026-10-23, to-do VLA-08). sitl/ros2_vla_bridge_node.py
puts AerialVLA where the stub was. Its /vla/action_4d carries the grant's four
values and then the decision behind them (VLA_ACTION_FIELDS: the decision's
age and seq, who authored the tick, the LAND / END flags); the four values
are read as before, so the stub's messages change nothing here. With the
extended message (the review of 339f23a tightened every point):
  * a message that is not a whole valid tail is REFUSED - not flown, not a
    receipt for the 0.5 s watchdog - and held: a non-finite value anywhere,
    an out-of-set field, another length, a layout label that is not
    VLA_LAYOUT_LABEL (or none, once the bridge's identity announced its
    layout), a decision seq going backwards, motion no decision authored
    (pilot_fields: only a vla_step moves horizontally or turns), and, once
    the bridge has shown itself (an extended message, or an identity with
    its action_layout), the grant's four values alone - a stub left running
    beside it. The publishers on /vla/action_4d are counted with their
    names; more than one on ACTION_PUBLISHERS_CHECKS consecutive checks is
    recorded and the run is not KPI-grade.
  * a vla_step tick whose decision is older than --decision-timeout-s
    (default 10 s, the bridge's step-tracker timeout) plus a 1 s grace
    (DECISION_GRACE_S), or a hold older than twice the timeout plus the
    grace, is not flown: the node holds the aircraft as for a quiet stream
    (reason "decision") until the next valid message that is not itself
    stale (DecisionWatchdog). Silence ages a decision only up to the
    receipt timeout: a pilot gone quiet is the receipt watchdog's (re-review
    of 23c88d3, N1: a sound bridge's 2 s stall near the end of a step landed
    the mission). A hold of this node's that lasts --decision-hold-max-s
    (10 s), or DECISION_SPELLS_MAX holds within DECISION_SPELLS_WINDOW_S,
    end the mission through the mission-end LAND, with where and in which
    state recorded; the FSM's own Loiter or fail-safe is never overridden.
    The bridge zeroes such a step
    itself; this is the check that does not trust it (docs/DESIGN-vla-
    pilot.md section 5: a republished held action keeps the 0.5 s receipt
    watchdog green while the model is dead). Rows carry pilot_author,
    decision_seq, decision_age_s, decision_stale; metrics.json `vla_pilot`.
  * the LAND flag (the model answered LAND) or the END flag (the bridge's
    mission ended) ends the mission through the node's own mission-end LAND
    (_finish: never after a takeover, never over an FSM fail-safe); the VLA
    never sends a mode request. It lands in place - this rail has no
    landing-site transfer - and metrics.json `vla_pilot.land_request` says
    where, in which FSM state and autopilot mode, and which keep-out rules
    the position breaks.
  * a kind "vla" identity with the server's weights digest is filed as
    pilot_record kind vla with that `reported_digest`, and the manifest's
    vla_model_hash says sha256 of it (pilot_from_identity): the companion has
    no weights to hash, and "<id>@unresolved" was refused for KPI grade. A
    fake: id is filed as a stub, and a bridge that never became ready loses
    its digest ("<id>@unresolved").
  * --vla-ready-timeout-s S (the launch file adds it for vla:=bridge, 3300 s
    by default; 0, the default here, keeps the stub's start-up): before
    GUIDED and arming, wait up to S for a /vla/identity saying `ready` (the
    bridge's warm-up passed) whose layout and decision timeout are this
    node's, and refuse to arm otherwise or on the timeout (vla_ready,
    identity_contract_problems). An identity without `ready` (a stub) does
    not pass. GUIDED, arming and the climb get their whole BRING_UP_S budget
    after the wait.
"""
from __future__ import annotations

import argparse
import gc
import json
import math
import struct
import sys
import threading
import time
from pathlib import Path
from typing import NamedTuple

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "sitl"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import Action4D, AuditLogger, Shield, State             # noqa: E402
from guardrail import scenario_events as SE                               # noqa: E402
from guardrail.audit import AUDIT_FILE                                    # noqa: E402
from guardrail.bundle import load_for_flight                              # noqa: E402
from guardrail.compiler import ConstraintCompiler                         # noqa: E402
from guardrail.delivery import (DELIVERY_SHORTFALL_MAX, DeliveryMonitor,  # noqa: E402
                                SENT_QOS_DEPTH, parse_report)
from guardrail import holdpoint                                           # noqa: E402
from guardrail.fsm import (DEFAULT_THETA_HORIZON_S, FAILSAFE_STATES,     # noqa: E402
                           MODE_FOR, THETA_HORIZON_S, EscalationFSM, FSMConfig,
                           FSMState, tick_input_from_decision)
from guardrail.geometry import fence_polygon                              # noqa: E402
from guardrail.hold_flight import HoldFlight, HoldWatch, body_command     # noqa: E402
from guardrail.response import HOLD_STREAM_GAP_S                          # noqa: E402
from guardrail.kpi import compute_from_dir                                # noqa: E402
from guardrail.manifest import (DEV_YAML_FLAG, UNRESOLVED,               # noqa: E402
                                flight_bundle_path, flight_policy_choice,
                                is_kpi_grade, pilot_record)
from guardrail.models import AltitudeEnvelope, PolygonFence, XY          # noqa: E402
from guardrail.projection import LocalProjection                          # noqa: E402
from guardrail.shield import (altitude_out_of_band, standoff_breached,       # noqa: E402
                              standoff_ring_m, standoff_rules)
from guardrail.world_map import load_world, missing_maps                   # noqa: E402
from guardrail.people import declared_subject_source                     # noqa: E402
from guardrail.terrain import (TerrainReference, ground_from_args,       # noqa: E402
                               is_dem, is_msl)
from guardrail.rtl_path import (MIN_PROGRESS_MPS, PROGRESS_WINDOW_S,     # noqa: E402
                                RTL_ALT_M, ProgressMonitor, band_from_shield,
                                check_leg, home_reached, remaining_to_home,
                                leg_rules_from_shield, zones_from_shield)
from guardrail.replay import (EpisodeRecord, audit_tick,                  # noqa: E402
                              episode_row_fields, flown_fields,
                              kpi_evidence_grade, rail_shield, rule_clock,
                              verify_replay, write_replay, zone_dwell)
from mavros_evidence import (get_params, initial_evidence,                # noqa: E402
                             note_mavros_state, param_value, poll_autopilot_version,
                             poll_sim_speedup, record_link, topology_manifest,
                             write_stack_revisions)
from sitl_stressors import (MARKER_TIMES_S, WATCHED_SIM_PARAMS,          # noqa: E402
                            marker_text)
from mavlink_adapter_node import (ADAPTER_NODE_NAME, DEFAULT_HOLD_MODE,   # noqa: E402
                                  FENCE_MARGIN_M, ORIGIN_TOLERANCE_M,
                                  HOLD_MODES, TOPIC_FENCES, TOPIC_MODE_REQUEST,
                                  TOPIC_SENT, TOPIC_SETPOINT, TOPIC_STATUS,
                                  ModeRequests,
                                  XYArgumentParser, fence_alt_plan,
                                  heading_deg_from_enu_quaternion,
                                  policy_ceiling_m, policy_fences, read_parm,
                                  zones_near)

TICK = 0.1
MAX_S = 120
REACH_M = 2.0
DYNAMIC_AT_S = 8.0
DYNAMIC_FENCE = PolygonFence(
    id="nfz-dynamic", type="polygon_fence",
    constraint_type="hard", priority="P0", violation_action="project_fix", scope="mission",
    vertices=[XY(x=23, y=6), XY(x=31, y=6), XY(x=31, y=14), XY(x=23, y=14)],
    margin_m=1.0,
)
# `--dynamic-zone on-aircraft`: the grant's spawn_polygon_fence perturbation at
# its hardest, a zone hot-applied ON TOP of the aircraft, so the run starts
# unsafe and time to safe is measured with the Shield on (WP3-17).
ON_AIRCRAFT_HALF_M = 5.0


def gcs_backstop_record(out: Path, autopilot_vals: dict) -> dict | None:
    """The ground station's GeoFence step (sitl/gcs_backstop.py) as this run
    recorded it, or None when no GCS set the fence: its verdict, its link,
    each parameter as the GCS read it back, its fence upload, and whether
    the values this node read through MAVROS (`autopilot_vals`) are the
    GCS's (`matches_autopilot`; None for a parameter either side lacks)."""
    p = Path(out) / "gcs_backstop.json"
    if not p.is_file():
        return None
    try:
        doc = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        return {"file": p.name, "ok": False, "error": f"unreadable: {e}"}
    params = doc.get("params") or {}
    gcs_vals = {k: (v or {}).get("read_back") for k, v in params.items()}
    diffs = {k: {"gcs": g, "autopilot": autopilot_vals.get(k)}
             for k, g in gcs_vals.items()
             if g is None or autopilot_vals.get(k) is None
             or abs(float(g) - float(autopilot_vals[k])) > 1e-3}
    fence = doc.get("fence") or {}
    return {"file": p.name, "ok": bool(doc.get("ok")), "error": doc.get("error"),
            "link": doc.get("link"), "router_endpoint": doc.get("router_endpoint"),
            "gcs_sysid": doc.get("gcs_sysid"), "pid": doc.get("pid"),
            "bundle_policy_hash": (doc.get("bundle") or {}).get("policy_hash"),
            "params_read_back": gcs_vals,
            "params_before": {k: (v or {}).get("before") for k, v in params.items()},
            "fence_zones": fence.get("zones"), "fence_items_sent": fence.get("items_sent"),
            "fence_items_read_back": fence.get("items_read_back"),
            "fence_match": fence.get("match"),
            "matches_autopilot": not diffs, "differences": diffs}


def keepout_zones(policy) -> list:
    """Every keep-out zone with vertices: polygon and circle fences, and the
    dynamic_nfz a mid-flight zone becomes where the Shield enforces the
    grant's update model. (A moving dynamic_nfz is drawn where it was
    spawned; zone dwell is guardrail.replay.zone_dwell, which names a moving
    zone instead of scoring it.)"""
    return [c for c in getattr(policy, "constraints", [])
            if getattr(c, "type", "") in ("polygon_fence", "circle_fence",
                                          "dynamic_nfz")
            and getattr(c, "vertices", None)]


def zone_around(x: float, y: float, half_m: float = ON_AIRCRAFT_HALF_M,
                zone_id: str = "nfz-on-aircraft") -> PolygonFence:
    """A square keep-out zone centred on (x North, y East)."""
    return PolygonFence(
        id=zone_id, type="polygon_fence", margin_m=1.0,
        constraint_type="hard", priority="P0", violation_action="project_fix", scope="mission",
        vertices=[XY(x=x - half_m, y=y - half_m), XY(x=x + half_m, y=y - half_m),
                  XY(x=x + half_m, y=y + half_m), XY(x=x - half_m, y=y + half_m)])
FENCE_PARM = ROOT / "sitl" / "fence" / "guardrail_fence.parm"
# The grant's Safety Shield horizon: "5 s of predicted trajectory at 10 Hz =
# 50 future poses". Until 2026-10-07 both SITL rails ran 3 s at 0.5 s steps
# (6 poses).
LOOKAHEAD_S = 5.0
LOOKAHEAD_DT_S = 0.1
# The autopilot parameters that make the GeoFence a backstop, read back at
# take-off and recorded (hil_evidence.autopilot_fence).
FENCE_PARAMS = ("FENCE_ENABLE", "FENCE_TYPE", "FENCE_ACTION", "FENCE_ALT_MAX",
                "FENCE_MARGIN", "AVOID_ENABLE")
# The one-period reading of theta (guardrail.fsm.THETA_HORIZON_S, 0.1 s):
# selectable with --theta-horizon-s 0.1, not the default (the per-operator
# magnitude_m is). The name is kept for the tests and tools that read it; the
# basis a run used is written into metrics.json.
PROXY_HORIZON_S = THETA_HORIZON_S
# After the autopilot takes the aircraft away from the Shield (a GeoFence
# breach RTL, a pilot on the GCS), how long to keep recording before ending
# the episode if it has not landed.
TAKEOVER_GRACE_S = 25.0
# How long after the FSM or the autopilot entered RTL / Land to wait for the
# landing before the episode is ended anyway.
FAILSAFE_GRACE_S = 40.0
# The stale-action watchdog's default: this many missed ticks of the grant's
# 10 Hz action rate (Safety Shield page: "Monitor rate: 10 Hz, matching the
# VLA action rate"). A pilot slower than 10 Hz (OpenVLA-7B makes about one
# decision per second) needs a longer --action-timeout-s, which is recorded.
ACTION_TIMEOUT_TICKS = 5
ACTION_TIMEOUT_S = ACTION_TIMEOUT_TICKS * TICK
# The stale-pose watchdog's default (--pose-timeout-s). Well under the
# Shield's 5 s lookahead (LOOKAHEAD_S), so the hold comes while the forecast
# made from the last pose still covers the path; above what harmless flights
# produce: a 0.6-0.7 s whole-simulator stall on ros2if_on_dyn_yaw (6 rows on
# one pose), and the 2-4 Hz stream rates Mission Planner sets on the shared
# link until the node's next 5 s re-assert (sitl/README.md).
POSE_TIMEOUT_S = 1.0
# The grant's monitor tick budget, end to end (Safety Shield page,
# "Performance targets": <= 100 ms (10 Hz) end-to-end).
TICK_BUDGET_MS = 100.0
# The policy flown when neither --bundle nor --policy is given: its signed
# bundle when bundles/ holds it, this YAML only under --dev-yaml.
DEFAULT_POLICY_YAML = ROOT / "policies" / "sim_demo_policy.yaml"
DEFAULT_HOTAPPLY_HOST = "127.0.0.1"
# The VLA bridge's extended /vla/action_4d (sitl/ros2_vla_bridge_node.py
# ACTION_FIELDS, AUTHOR_CODES, FLAG_*). This node's own copy, as tests/
# vla_fake_server.py keeps its own copy of the protocol: a bridge that changed
# its layout would otherwise be read with its own constants;
# tests/test_ros2_vla_bridge.py pins the two equal.
VLA_ACTION_FIELDS = ("vx_fwd", "vy_right", "vz_up", "yaw_rate",
                     "decision_age_s", "decision_seq", "author", "flags")
VLA_LAYOUT_LABEL = ",".join(VLA_ACTION_FIELDS)
VLA_AUTHORS = {0: "none", 1: "vla_step", 2: "hold", 3: "hover"}
VLA_FLAG_LAND = 1
VLA_FLAG_END = 2
VLA_ACTION_TOPIC = "/vla/action_4d"
# Refused /vla/action_4d messages written to events.jsonl one by one; later
# ones are counted only (metrics.json vla_pilot).
REFUSAL_EVENTS_MAX = 20
# A value this small is zero on the wire (float32 carries the bridge's exact
# zeros; anything larger is motion).
ZERO_EPS = 1e-6
# A vla_step tick whose decision is older than this plus DECISION_GRACE_S is
# not flown (--decision-timeout-s): the bridge's step tracker stops flying a
# step at its own decision timeout, guardrail.vla_link.TrackerConfig.
# decision_timeout_s (10 s; the test pins them equal), so only a broken bridge
# sends one. The grace: a sound bridge flies a slow step until its age
# reaches the timeout exactly (5 m at 1 m/s plus a few seconds of latency
# gets there), and that last tick is aged here by up to one receipt period
# more; without it the last tick of every such step held the aircraft.
DECISION_TIMEOUT_S = 10.0
DECISION_GRACE_S = 1.0
# How long a decision hold may last before the mission ends through the
# mission-end LAND (--decision-hold-max-s; review of 339f23a, P3-1). A hold
# is a pilot fault the Shield cannot repair; a sound bridge that this hold
# put in BRAKE suspends itself and sends no fresh step until GUIDED comes
# back, so waiting for one longer only hovers. 10 s is the city rail's
# dead-server timeout (vla_mission.MonitorConfig.dead_timeout_s), ours.
DECISION_HOLD_MAX_S = 10.0
# This many decision holds within DECISION_SPELLS_WINDOW_S end the mission
# the same way (re-review of 23c88d3, N3): a hold ends on the next valid
# message (N1), so a pilot that keeps sending stale or refused messages - or
# a stub left running beside the bridge - flapped LOITER / GUIDED instead.
# Ours, not measured: a sound bridge starts no decision hold at all.
DECISION_SPELLS_MAX = 3
DECISION_SPELLS_WINDOW_S = 30.0
# Consecutive once-a-second checks that must see more than one publisher on
# /vla/action_4d before the run is flagged (N7): a node killed with -9 stays
# in the graph until its lease expires, a ghost the first check can see.
ACTION_PUBLISHERS_CHECKS = 3
# GUIDED, arming, take-off and the climb share this budget (bring_up). With
# --vla-ready-timeout-s it starts after the wait for the VLA, which can last
# far longer (review of 339f23a, P1-B).
BRING_UP_S = 180.0
# A SCENARIO'S START (--start X,Y[,UP], 2026-11-06). The autopilot's home,
# projected about the policy's frame origin, must lie this close to (X, Y)
# before arming (the adapter's own tolerance for its fence upload): a SITL
# booted elsewhere flies the scenario somewhere else, and is refused on the
# pad. The node asks MAVROS for the home (/mavros/home_position/req_update)
# every HOME_REQUEST_EVERY_S while it waits, up to HOME_WAIT_S.
HOME_TOL_M = ORIGIN_TOLERANCE_M
HOME_WAIT_S = 90.0
HOME_REQUEST_EVERY_S = 3.0
# With a start altitude the mission starts once the aircraft has held it
# within START_UP_TOL_M for START_SETTLE_S (the climb's overshoot is not the
# scenario's start); without one, at cruise altitude - 1 m, as before.
START_UP_TOL_M = 0.5
START_SETTLE_S = 1.0
# A heading for the scenario's events (spawn ahead, move left) is read from
# the measured velocity above this speed; slower, the last one is kept.
EVENT_HEADING_MIN_MPS = 0.1
# The GPS fix behind State.fix_ok (2026-11-07; module docstring, "THE GPS FIX
# AND THE SIMULATOR'S STRESSORS"). sensor_msgs/NavSatStatus: STATUS_NO_FIX
# -1, STATUS_FIX 0 (SBAS 1, GBAS 2). A status older than the pose timeout is
# not a report about the pose this tick checks.
FIX_TOPIC = "/mavros/global_position/raw/fix"
NAVSAT_STATUS_FIX = 0
FIX_STALE_S = POSE_TIMEOUT_S
# The autopilot's own verdict on its horizontal position behind
# State.estimator_ok (2026-11-12; module docstring, "THE AUTOPILOT'S
# ESTIMATOR"). ArduPilot reports it in EKF_STATUS_REPORT (MAVLink 193,
# ardupilotmega dialect), which no MAVROS 2 plugin publishes (sys_status
# publishes ESTIMATOR_STATUS, 230, which ArduPilot does not send), so it is
# read from MAVROS's raw MAVLink feed to the UAS - every message the autopilot
# sent, as mavros_msgs/Mavlink - keeping only the autopilot's own reports
# (MAVROS's default target, system 1 component 1). Flags: MAVLink's
# EKF_STATUS_FLAGS (EKF_PRED_POS_HORIZ_ABS is 512, not 256).
MAVLINK_SOURCE_TOPIC = "/uas1/mavlink_source"
MAVLINK_SOURCE_DEPTH = 100
EKF_STATUS_REPORT_ID = 193
AUTOPILOT_IDS = (1, 1)
EKF_POS_HORIZ_ABS = 16
EKF_CONST_POS_MODE = 128
EKF_UNINITIALIZED = 1024
EKF_GPS_GLITCHING = 32768
# ArduCopter's own EKF failsafe (ArduCopter/ekf_check.cpp, 4.5.7): FS_EKF_THRESH
# on the test ratios (0.8 by default and on the rig), and 10 failing checks
# at 10 Hz (EKF_CHECK_ITERATIONS_MAX: 1 s) before it is declared, each passing
# check taking one back. A report older than the pose timeout is not a
# report about this tick's pose.
EKF_THRESH_DEFAULT = 0.8
EKF_FAILSAFE_S = 1.0
EKF_STALE_S = POSE_TIMEOUT_S
# The simulator side's inputs and this node's announcement of its clock.
PARAM_EVENT_TOPIC = "/mavros/param/event"
STATUSTEXT_SEND_TOPIC = "/mavros/statustext/send"
STATUSTEXT_INFO = 6                   # MAV_SEVERITY_INFO
# Parameter events kept in metrics.json (MAVROS's pull at connect sends one
# per parameter; only the watched SIM_* ones are kept, and at most these).
SIM_PARAM_EVENTS_MAX = 200
# THE RTL WATCH (2026-11-14; module docstring): the FSM's input for G6, "RTL
# fails (battery / blocked path)" (Safety Shield p4), from what the autopilot
# reports. Until this date the node passed rtl_failed=False on every tick.
#
# The battery, as ArduPilot itself judges it: BATTERY_STATUS (MAVLink 147)
# `charge_state` is AP_BattMonitor::get_mavlink_charge_state - LOW for its
# Failsafe::Low, CRITICAL for Failsafe::Critical (the BATT_LOW_* / BATT_CRT_*
# thresholds, a voltage only after BATT_LOW_TIMER). Read from
# MAVLINK_SOURCE_TOPIC like EKF_STATUS_REPORT: the topic is already granted
# (SROS 2), and MAVROS 2's sensor_msgs/BatteryState has no charge state.
BATTERY_STATUS_ID = 147
# The base payload (36 bytes) and the two extension fields ArduPilot fills
# first, time_remaining (int32) and charge_state (uint8); MAVLink 2 truncates
# trailing zero bytes, restored here.
BATTERY_STATUS_LEN = 41
BATTERY_CHARGE_STATES = {0: "UNDEFINED", 1: "OK", 2: "LOW", 3: "CRITICAL",
                         4: "EMERGENCY", 5: "FAILED", 6: "UNHEALTHY", 7: "CHARGING"}
# The charge states that fail an RTL: CRITICAL and worse, the level at which
# ArduPilot's own critical action (BATT_FS_CRT_ACT) lands. LOW is ArduPilot's
# reason to START an RTL (BATT_FS_LOW_ACT), not to abandon one; UNHEALTHY is
# a monitor fault, not a charge.
BATTERY_FAILED_STATES = (3, 4, 5)
# A report older than this is not about the battery now (BATTERY_STATUS rides
# the extended-status stream, which this node asks for at 10 Hz).
BATTERY_STALE_S = 5.0
# No progress home: guardrail.rtl_path.ProgressMonitor, the harness's own test
# (the grant's T = 5 s, X6's 0.1 m/s: under 0.5 m of the climb plus the
# distance home made good in 5 s), judged only while the autopilot reports
# RTL, and not within RTL_HOME_RADIUS_M of home, where ArduCopter's RTL no
# longer flies what remaining_to_home counts: its return-altitude cone
# (RTL_CONE_SLOPE, mode_rtl.cpp) cuts the climb within RTL_ALT / RTL_CONE_SLOPE
# of home (15 m / 3 = 5 m by default), and over home it loiters RTL_LOIT_TIME
# and descends - an RTL that is ending, not failing (such a return ends by X5:
# landed, home_reached never read at the full return altitude).
RTL_PROGRESS_WINDOW_S = PROGRESS_WINDOW_S
RTL_MIN_PROGRESS_MPS = MIN_PROGRESS_MPS
RTL_HOME_RADIUS_M = 5.0
RTL_CONE_SLOPE_DEFAULT = 3.0
# RTL not entered: the FSM asked for RTL and the autopilot still reports
# another mode this long after. /mavros/state follows ArduPilot's 1 Hz
# HEARTBEAT, so this is five heartbeats - a request that went through is
# confirmed within about one; one ArduCopter refused (RTL needs a position
# estimate and a home) never is.
RTL_ENTRY_TIMEOUT_S = 5.0
# Read from the autopilot at take-off and recorded (read_rtl_params): RTL_ALT
# (cm) and RTL_CONE_SLOPE size the climb and the home radius; the BATT_*
# say whether ArduPilot can declare CRITICAL at all (BATT_CRT_VOLT and
# BATT_CRT_MAH both 0 disable it, the copter default) and what it then does.
RTL_WATCH_PARAMS = ("RTL_ALT", "RTL_CONE_SLOPE", "BATT_FS_CRT_ACT", "BATT_CRT_VOLT",
                    "BATT_CRT_MAH")
# RTL failures kept with their reasons in metrics.json (the rest counted).
RTL_FAILURES_KEPT = 20
def slow_pilot_above_s(cfg: FSMConfig) -> float:
    """THE PILOT'S DECISION PERIOD (module docstring): the period above which
    FSMConfig.for_decision_period widens `cfg`'s N-in-T window, T / (1.5 x
    N) - 1.11 s for the grant's N = 3, T = 5 s, 0.83 s for N = 4."""
    return cfg.window_s / (1.5 * cfg.n_violations)


def fsm_config_for_pilot(cfg: FSMConfig, period_s: float | None) -> tuple[FSMConfig, dict]:
    """(the FSM's config, the record for metrics.json `fsm.decision_period`)
    for a pilot deciding every `period_s` seconds (--decision-period-s; None:
    not declared). FSMConfig.for_decision_period(period_s, cfg) whenever it
    changes `cfg` (a pilot slower than slow_pilot_above_s(cfg), whatever N
    and T the --fsm-config sets); otherwise `cfg` itself, the same object,
    so its thresholds and digest are unchanged. Raises ValueError for a
    period that is not a finite number > 0."""
    rec = {"declared_s": period_s, "slow_above_s": round(slow_pilot_above_s(cfg), 4),
           "window_s_base": cfg.window_s, "n_violations": cfg.n_violations}
    if period_s is None:
        return cfg, {**rec, "applied": False, "window_s": cfg.window_s,
                     "why": "no decision period declared (--decision-period-s): the "
                            "thresholds as configured"}
    if isinstance(period_s, bool) or not isinstance(period_s, (int, float)) \
            or not math.isfinite(period_s) or period_s <= 0:
        raise ValueError(f"--decision-period-s must be a finite number of seconds > 0, "
                         f"got {period_s!r}")
    slow = FSMConfig.for_decision_period(float(period_s), cfg)
    if slow == cfg:
        return cfg, {**rec, "applied": False, "window_s": cfg.window_s,
                     "why": f"a pilot deciding every {period_s:g} s (at most "
                            f"{slow_pilot_above_s(cfg):.4g} s for N = {cfg.n_violations}, "
                            f"T = {cfg.window_s:g} s) keeps the thresholds as configured"}
    return slow, {**rec, "applied": True, "window_s": slow.window_s,
                  "why": f"a pilot deciding every {period_s:g} s: N-in-T's T = max(T, N x "
                         f"period x 1.5) (FSMConfig.for_decision_period)"}


# --------------------------------------------------------------------------- #
# Pure helpers (tests/test_sitl_rails.py imports these without ROS)
# --------------------------------------------------------------------------- #

def parse_xy(text: str | None) -> tuple[float, float] | None:
    if not text:
        return None
    parts = [float(v) for v in text.split(",")]
    if len(parts) != 2 or not all(math.isfinite(v) for v in parts):
        raise ValueError(f"expected X,Y, got {text!r}")
    return parts[0], parts[1]


def parse_point(text: str | None, what: str = "point"
                ) -> tuple[float, float, float | None] | None:
    """`X,Y` or `X,Y,UP` (finite metres) as (x, y, up or None); None for no
    text. Raises ValueError otherwise, before anything flies: --target and
    --start (2026-11-06; --target took X,Y only until then)."""
    if not text:
        return None
    try:
        parts = [float(v) for v in str(text).split(",")]
    except ValueError:
        raise ValueError(f"{what}: expected X,Y or X,Y,UP, got {text!r}") from None
    if len(parts) not in (2, 3) or not all(math.isfinite(v) for v in parts):
        raise ValueError(f"{what}: expected X,Y or X,Y,UP, got {text!r}")
    return parts[0], parts[1], (parts[2] if len(parts) == 3 else None)


def repo_path(p: str | Path) -> Path:
    """A file named on the command line: as given when it exists (absolute, or
    relative to the working directory), else relative to the repository - the
    container's working directory is the checkout, the desktop rail's need
    not be."""
    path = Path(p)
    if path.is_absolute() or path.exists():
        return path
    return ROOT / path


def load_scenario_meta(path) -> dict:
    """--scenario-meta: the stress harness's record of the scenario this
    mission flies (experiments/sweep_scenarios.py rail_meta): a JSON object,
    written into metrics.json as `scenario`, its `family` as
    `scenario_family` (the KPI report's row)."""
    doc = json.loads(repo_path(path).read_text(encoding="utf-8"))
    if not isinstance(doc, dict):
        raise ValueError(f"--scenario-meta {path}: not a JSON object")
    return doc


def load_events(path) -> list:
    """--events: a JSON list of the library's ScenarioEvents (guardrail.
    scenario_spec.ScenarioEvent, validated here, before anything flies),
    returned as (index in the file, event) in the order they fire: by
    at_sim_t, ties in file order, as the stress harness schedules them."""
    from guardrail.scenario_spec import ScenarioEvent
    raw = json.loads(repo_path(path).read_text(encoding="utf-8"))
    if isinstance(raw, dict):
        raw = raw.get("events")
    if not isinstance(raw, list):
        raise ValueError(f"--events {path}: expected a JSON list of ScenarioEvents")
    evs = [(i, ScenarioEvent.model_validate(e)) for i, e in enumerate(raw)]
    return sorted(evs, key=lambda p: (p[1].at_sim_t, p[0]))


def home_check(home, origin, start_xy, tol_m: float = HOME_TOL_M) -> dict:
    """Is the autopilot's home where --start says the scenario starts?

    `home` is (lat, lon, (north, east) of home from the EKF origin) from
    /mavros/home_position/home, or None if none arrived; `origin` the
    policy's frame origin (lat, lon), None for a metre policy; `start_xy` the
    policy-frame position the node flies the EKF origin at. The node offsets
    every pose by `start_xy`, so home must sit at start_xy + its EKF offset
    in the policy's frame, within `tol_m`. A metre policy has no geography to
    check: its frame is the one --start declares."""
    sx, sy = float(start_xy[0]), float(start_xy[1])
    rec = {"start": [sx, sy], "tolerance_m": tol_m}
    if origin is None:
        return dict(rec, checked=False, ok=True,
                    why="the policy is written in metres (no frame origin): the "
                        "frame is the one --start declares")
    rec["frame_origin"] = [float(origin[0]), float(origin[1])]
    if home is None:
        return dict(rec, checked=True, ok=False,
                    why="no home position from the autopilot "
                        "(/mavros/home_position/home): SITL's home cannot be checked "
                        "against --start")
    lat, lon = float(home[0]), float(home[1])
    en = home[2] if len(home) > 2 and home[2] is not None else (0.0, 0.0)
    n, e = LocalProjection(rec["frame_origin"][0], rec["frame_origin"][1]).to_local(lat, lon)
    want_n, want_e = sx + float(en[0]), sy + float(en[1])
    miss = math.hypot(n - want_n, e - want_e)
    ok = miss <= tol_m
    return dict(rec, checked=True, ok=ok, home=[lat, lon],
                home_policy_ne=[round(n, 3), round(e, 3)],
                home_from_ekf_origin_ne=[round(float(en[0]), 3), round(float(en[1]), 3)],
                miss_m=round(miss, 3),
                why=None if ok else
                (f"the autopilot's home is at ({n:.1f}, {e:.1f}) m in the policy's "
                 f"frame, {miss:.1f} m from the scenario's start ({want_n:.1f}, "
                 f"{want_e:.1f}) m (tolerance {tol_m:g} m): SITL booted at another "
                 f"home (SITL_HOME) than the scenario starts from"))


def alt_dwell_ticks(traj: list[dict], envelopes) -> int:
    """(point, envelope) pairs outside the envelope: metrics.json's
    alt_violation_s is this times TICK. Each envelope is judged in its own
    reference (X-11): an MSL envelope against the point's `msl_m` (written
    only with --ground), every other envelope against `up` (above home, or
    above the DEM under the aircraft with a --ground grid). Until 2026-11-18
    every envelope was judged against `up`, so a flight of
    experiments/policies/msl_dem_sitl.yaml counted every tick as an altitude
    violation (604 m MSL read as 604 m above ground). A point without
    `msl_m` (every flight without --ground, where an MSL rule is refused
    before take-off) is judged as before."""
    def height(p: dict, e) -> float:
        return p["msl_m"] if is_msl(e) and "msl_m" in p else p["up"]
    return sum(1 for p in traj for e in envelopes
               if altitude_out_of_band(e, height(p, e)))


def zones_at(shield, st: State) -> tuple[list[str], float | None]:
    """(ids of the keep-out zones in force at the aircraft's altitude whose
    POLYGON holds the position, metres from the nearest zone's polygon -
    negative inside, None with no zone in force): the stress harness's
    polygon measures (sweep_scenarios._signed_dist), on the zones as the
    Shield places them now (Shield.zones_now: moving zones where they are,
    a zone a clock has switched off left out)."""
    from guardrail.geometry import ring_contains, signed_edge_distance
    ids: list[str] = []
    best = None
    for zr in shield.zones_now(st.up):
        poly = zr.polygon
        inside = ring_contains(poly, st.x, st.y)
        sd = signed_edge_distance(poly, st.x, st.y)
        best = sd if best is None else min(best, sd)
        if inside:
            ids.append(zr.rule.id)
    return sorted(ids), (None if best is None else round(best, 4))


# The decision's trace words the stress harness counts per tick (2026-11-07,
# experiments/sweep_scenarios.py rail_rule_measurements, as run_episode counts
# them): the zones whose window opens on the aircraft sooner than it can leave
# (`window_opens_on`, the anticipated count) or opened on it (`window_opened_on`),
# and an escape whose cap or floor was refused (`cap_refused`, `floor_refused`).
SHIELD_TRACE_KEYS = ("window_opens_on", "window_opened_on", "cap_refused", "floor_refused")


def shield_trace_fields(decision) -> dict:
    """The row's `shield_trace`: SHIELD_TRACE_KEYS that the decision's trace
    holds (zone lists sorted), {} when it holds none. Written on every row,
    so an empty one is a measured nothing, never a field the log lacks."""
    tr = getattr(decision, "trace", None) or {}
    out = {}
    for k in SHIELD_TRACE_KEYS:
        v = tr.get(k)
        if v:
            out[k] = sorted(v) if isinstance(v, (list, tuple, set)) else v
    return out


def adapter_listening(node_names) -> bool:
    """Is the MAVLink adapter among these subscribers of /shield/mode_request?

    Until 2026-10-19 any subscriber counted (count_subscribers > 0), and
    rosbag2 subscribes to that topic in every recorded run (run_ros2_demo.sh
    BAG=1, the Docker mission role): with the adapter down, LOITER, RTL and
    the mission-end LAND went to the bag alone while events.jsonl said
    `via: adapter`, and the direct-MAVROS fallback could never run."""
    return ADAPTER_NODE_NAME in node_names


def default_tag(shield: str, dynamic: bool) -> str:
    """Kept identical to run_ros2_demo.sh's rule, which records the bag there."""
    return f"ros2_shield_{shield}" + ("_dynamic" if dynamic else "")


# The own-velocity forecast (docs/DESIGN-own-velocity-forecast.md) on this
# rail: OFF by default. Its response model (guardrail.response.
# ARDUCOPTER_GUIDED) was measured on the Project AirSim quad behind
# ArduCopter, and this rail flies ArduPilot's own SITL physics, whose response
# nobody has measured: on its recorded flights the aircraft reached 0.69-0.96
# of the commanded speed in steady flight, against 0.96 on the Project AirSim
# quad. Turn it on (`--own-velocity-forecast on`) to fly it and log the data
# to measure it with (state_v); the velocity is logged either way.
OWN_VELOCITY_DEFAULT = "off"
# A velocity older than this is not the aircraft's velocity now (MAVROS
# publishes velocity_local at the 10 Hz stream rate the node asks for).
VELOCITY_MAX_AGE_S = 0.3


def velocity_from_enu(linear) -> tuple[float, float, float]:
    """/mavros/local_position/velocity_local's twist.linear (ENU: x East,
    y North, z up) in the Shield's frame (North, East, up)."""
    return float(linear.y), float(linear.x), float(linear.z)


def shield_state(state: State, velocity, age_s: float | None, enabled: bool) -> State:
    """The State the Shield is given: `state` with the measured velocity when
    the own-velocity forecast is on and the velocity is fresh and finite,
    else `state` itself (the commanded forecast, as before)."""
    if (not enabled or velocity is None or age_s is None or age_s > VELOCITY_MAX_AGE_S
            or not all(math.isfinite(c) for c in velocity)):
        return state
    return state.model_copy(update={"velocity": tuple(velocity)})


def fix_ok_from(status, age_s: float | None, stale_s: float = FIX_STALE_S) -> bool | None:
    """State.fix_ok from the latest /mavros/global_position/raw/fix
    (2026-11-07): its NavSatStatus `status` and its age on the node's clock.
    None - not reported, the position trusted exactly as before - with no
    message yet (a rail without the topic) or one older than `stale_s`; True
    for STATUS_FIX or better (MAVROS: GPS_RAW_INT.fix_type > 2, a 3D fix);
    False for STATUS_NO_FIX (fix_type 0-2: SITL's GPS with SIM_GPS_DISABLE
    reports 0, libraries/SITL/SIM_GPS_UBLOX.cpp)."""
    if status is None or age_s is None:
        return None
    age = float(age_s)
    if not (math.isfinite(age) and 0.0 <= age <= float(stale_s)):
        return None
    return int(status) >= NAVSAT_STATUS_FIX


def fix_spells(rows: list[dict]) -> list[dict]:
    """The spells of rows whose fix_ok is False: [{t_start, t_end, ticks}],
    t_end None for a spell still running at the last row."""
    out: list[dict] = []
    cur = None
    for r in rows:
        if r.get("fix_ok") is False:
            if cur is None:
                cur = {"t_start": r.get("t"), "t_end": None, "ticks": 0}
                out.append(cur)
            cur["ticks"] += 1
        elif cur is not None:
            cur["t_end"] = r.get("t")
            cur = None
    return out


def ekf_status_from_mavlink(msgid, sysid, compid, length, payload64) -> dict | None:
    """EKF_STATUS_REPORT's fields from one mavros_msgs/Mavlink (2026-11-12),
    or None for any other message, or one not from the autopilot
    (AUTOPILOT_IDS). MAVLink orders the fields by size, floats first:
    velocity, pos_horiz, pos_vert, compass and terrain_alt variances (the
    square roots of EKF3's innovation test ratios), then flags (uint16);
    MAVLink 2 truncates trailing zero bytes, restored here. payload64 is the
    payload as little-endian 64-bit words, `length` its byte count."""
    try:
        if int(msgid) != EKF_STATUS_REPORT_ID or (int(sysid), int(compid)) != AUTOPILOT_IDS:
            return None
        raw = b"".join(struct.pack("<Q", int(w) & 0xFFFFFFFFFFFFFFFF) for w in payload64)
    except (TypeError, ValueError, OverflowError):
        return None
    raw = raw[:max(0, int(length))].ljust(22, b"\0")
    vv, ph, pv, cv, tv, flags = struct.unpack_from("<5fH", raw, 0)
    return {"flags": int(flags), "velocity_variance": vv, "pos_horiz_variance": ph,
            "pos_vert_variance": pv, "compass_variance": cv, "terrain_alt_variance": tv}


def ekf_position_healthy(report: dict, thresh: float = EKF_THRESH_DEFAULT) -> tuple[bool, str]:
    """(healthy, why): is the autopilot's absolute horizontal position valid by
    its own test (2026-11-12)? ArduCopter navigates on it while
    EKF_POS_HORIZ_ABS is set and EKF_CONST_POS_MODE clear
    (ekf_has_absolute_position, armed). This also wants EKF_UNINITIALIZED
    clear and the horizontal-position and velocity test ratios finite and
    below `thresh` (FS_EKF_THRESH) - stricter than ArduCopter's
    ekf_over_threshold, which needs two of compass / velocity / position (or
    the velocity at twice it). EKF_GPS_GLITCHING is not read: the autopilot
    keeps navigating through it, and its own failsafe does not count it."""
    flags = int(report["flags"])
    if not flags & EKF_POS_HORIZ_ABS:
        return False, "no absolute horizontal position (EKF_POS_HORIZ_ABS clear)"
    if flags & EKF_CONST_POS_MODE:
        return False, "constant-position mode (EKF_CONST_POS_MODE)"
    if flags & EKF_UNINITIALIZED:
        return False, "not initialised (EKF_UNINITIALIZED)"
    for key in ("pos_horiz_variance", "velocity_variance"):
        v = float(report[key])
        if not (math.isfinite(v) and v < thresh):
            return False, f"{key} {v:.3g} not below {thresh:g}"
    return True, "absolute horizontal position valid"


def _round_or_none(v, nd: int):
    """round(v, nd) for a finite number, None otherwise (a row's JSON)."""
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return round(v, nd) if math.isfinite(v) else None


def ekf_thresh_from(value) -> tuple[float, str]:
    """The test-ratio threshold the estimator check uses, and why: the
    autopilot's FS_EKF_THRESH when read and positive, else ArduCopter's
    default (EKF_THRESH_DEFAULT) - a disabled variance check (0) on the
    autopilot does not disable this one."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return EKF_THRESH_DEFAULT, "FS_EKF_THRESH not read: ArduCopter's default"
    if not (math.isfinite(v) and v > 0.0):
        return EKF_THRESH_DEFAULT, f"FS_EKF_THRESH {value!r} disables the autopilot's check: ArduCopter's default"
    return v, "FS_EKF_THRESH (read from the autopilot)"


class EstimatorHealth:
    """State.estimator_ok from the autopilot's EKF_STATUS_REPORT stream, as the
    autopilot's own EKF failsafe judges it (2026-11-12; module docstring,
    "THE AUTOPILOT'S ESTIMATOR"). Pure: `report(t, fields)` per report
    (receipt on one monotonic clock), `verdict(t)` per tick.

    An unhealthy report (ekf_position_healthy) adds the time since the
    previous report to a leaky account (at most the stale bound) and a
    healthy one takes it back, as ArduCopter's ekf_check counts failing 10 Hz
    checks up and passing ones down. The verdict is False from the report
    that brings the account to `failsafe_s` until it is back at 0 (ekf_check
    clears its failsafe at a zero count), True otherwise - the autopilot
    still navigates on its estimate - and None with no report yet or the
    latest older than `stale_s`: not a report, and the Shield falls back on
    fix_ok. On the hil rig EKF3 dropped its absolute position 6.8-6.9 s into
    23 of 24 7 s GPS denials, for 0.1-0.2 s, until the GPS came back: an
    account of 0.2 s, never a verdict."""

    def __init__(self, thresh: float = EKF_THRESH_DEFAULT,
                 failsafe_s: float = EKF_FAILSAFE_S, stale_s: float = EKF_STALE_S) -> None:
        for name, v in (("thresh", thresh), ("failsafe_s", failsafe_s), ("stale_s", stale_s)):
            if not (isinstance(v, (int, float)) and math.isfinite(v) and v > 0):
                raise ValueError(f"{name} must be a positive number, got {v!r}")
        self.thresh = float(thresh)
        self.thresh_source = "ArduCopter's default"
        self.failsafe_s = float(failsafe_s)
        self.stale_s = float(stale_s)
        self.last: dict | None = None
        self.last_why = ""
        self.rx: float | None = None
        self.n = 0
        self.n_unhealthy = 0
        self.bad_s = 0.0
        self.declared = False
        self.max_gap_s = 0.0
        # Spells of unhealthy reports, and of a declared verdict (False):
        # [{t_start, t_end, reports, why}], t_end None while running.
        self.spells: list[dict] = []
        self.declared_spells: list[dict] = []

    def set_thresh(self, value) -> None:
        self.thresh, self.thresh_source = ekf_thresh_from(value)

    def report(self, t: float, fields: dict) -> None:
        t = float(t)
        healthy, why = ekf_position_healthy(fields, self.thresh)
        dt = 0.0
        if self.rx is not None:
            gap = max(0.0, t - self.rx)
            self.max_gap_s = max(self.max_gap_s, gap)
            dt = min(gap, self.stale_s)
        self.last, self.last_why, self.rx = dict(fields), why, t
        self.n += 1
        if healthy:
            self.bad_s = max(0.0, self.bad_s - dt)
            if self.spells and self.spells[-1]["t_end"] is None:
                self.spells[-1]["t_end"] = t
            if self.declared and self.bad_s <= 1e-9:
                self.declared = False
                self.declared_spells[-1]["t_end"] = t
            return
        self.n_unhealthy += 1
        self.bad_s = min(self.failsafe_s, self.bad_s + dt)
        if not self.spells or self.spells[-1]["t_end"] is not None:
            self.spells.append({"t_start": t, "t_end": None, "reports": 0, "why": why})
        self.spells[-1]["reports"] += 1
        if not self.declared and self.bad_s >= self.failsafe_s - 1e-9:
            self.declared = True
            self.declared_spells.append({"t_start": t, "t_end": None, "why": why})

    def age(self, t: float) -> float | None:
        return None if self.rx is None else float(t) - self.rx

    def verdict(self, t: float) -> bool | None:
        age = self.age(t)
        if age is None or not (math.isfinite(age) and 0.0 <= age <= self.stale_s):
            return None
        return not self.declared

    def summary(self, to_mission_t=None) -> dict:
        conv = to_mission_t or (lambda v: v)

        def spell(s):
            return {**s, "t_start": round(conv(s["t_start"]), 3),
                    "t_end": None if s["t_end"] is None else round(conv(s["t_end"]), 3)}
        return {"messages": self.n, "unhealthy_reports": self.n_unhealthy,
                "thresh": self.thresh, "thresh_source": self.thresh_source,
                "failsafe_s": self.failsafe_s, "stale_after_s": self.stale_s,
                "max_gap_s": round(self.max_gap_s, 3),
                "unhealthy_spells": [spell(s) for s in self.spells],
                "declared_spells": [spell(s) for s in self.declared_spells],
                "declared_at_end": self.declared}


def battery_status_from_mavlink(msgid, sysid, compid, length, payload64) -> dict | None:
    """BATTERY_STATUS's fields from one mavros_msgs/Mavlink (2026-11-14), or
    None for any other message, or one not from the autopilot
    (AUTOPILOT_IDS). MAVLink orders the base fields by size -
    current_consumed and energy_consumed (int32), temperature (int16),
    voltages[10] (uint16), current_battery (int16), id, battery_function,
    type (uint8), battery_remaining (int8) - and appends the extensions as
    declared: time_remaining (int32), then charge_state (uint8). MAVLink 2
    truncates trailing zero bytes, restored here, so a charge_state the
    message does not carry reads 0, UNDEFINED. payload64 is the payload as
    little-endian 64-bit words, `length` its byte count."""
    try:
        if int(msgid) != BATTERY_STATUS_ID or (int(sysid), int(compid)) != AUTOPILOT_IDS:
            return None
        raw = b"".join(struct.pack("<Q", int(w) & 0xFFFFFFFFFFFFFFFF) for w in payload64)
    except (TypeError, ValueError, OverflowError):
        return None
    raw = raw[:max(0, int(length))].ljust(BATTERY_STATUS_LEN, b"\0")
    consumed, _energy, _temp = struct.unpack_from("<iih", raw, 0)
    cells = struct.unpack_from("<10H", raw, 10)
    current_ca, = struct.unpack_from("<h", raw, 30)
    bid, _function, _type, remaining = struct.unpack_from("<BBBb", raw, 32)
    _time_remaining, charge = struct.unpack_from("<iB", raw, 36)
    mv = [c for c in cells if c != 0xFFFF]
    return {"id": int(bid), "charge_state": int(charge),
            "remaining_pct": None if remaining < 0 else int(remaining),
            "voltage_v": round(sum(mv) / 1000.0, 3) if mv else None,
            "current_a": None if current_ca == -1 else round(current_ca / 100.0, 2),
            "consumed_mah": None if consumed == -1 else int(consumed)}


def charge_state_name(state) -> str:
    """MAV_BATTERY_CHARGE_STATE's name for `state` (its number for one
    MAVLink does not define)."""
    return BATTERY_CHARGE_STATES.get(int(state), str(int(state)))


class BatteryHealth:
    """Is the battery the autopilot's own reason to give up a return
    (2026-11-14; module docstring, THE RTL WATCH)? Pure: `report(t, fields)`
    per BATTERY_STATUS (battery_status_from_mavlink; receipt on one
    monotonic clock), `verdict(t)` per tick: (True, why) while the latest
    report of any battery instance fresher than `stale_s` is in
    BATTERY_FAILED_STATES (ArduPilot's own battery failsafe at CRITICAL or
    worse), (False, why) while every fresh one is not, and (None, why) with
    no fresh report: not reported, and the RTL watch judges no battery."""

    def __init__(self, stale_s: float = BATTERY_STALE_S) -> None:
        if isinstance(stale_s, bool) or not isinstance(stale_s, (int, float)) \
                or not math.isfinite(stale_s) or stale_s <= 0:
            raise ValueError(f"stale_s must be a positive number, got {stale_s!r}")
        self.stale_s = float(stale_s)
        self.latest: dict[int, tuple[float, dict]] = {}
        self.n = 0
        self.states_seen: dict[str, int] = {}
        # Spells of a failed state as reported: [{t_start, t_end, id, state}].
        self.spells: list[dict] = []

    def report(self, t: float, fields: dict) -> None:
        t, bid = float(t), int(fields["id"])
        state = int(fields["charge_state"])
        self.latest[bid] = (t, dict(fields))
        self.n += 1
        name = charge_state_name(state)
        self.states_seen[name] = self.states_seen.get(name, 0) + 1
        bad = state in BATTERY_FAILED_STATES
        open_ = [s for s in self.spells if s["id"] == bid and s["t_end"] is None]
        if bad and not open_:
            self.spells.append({"t_start": t, "t_end": None, "id": bid, "state": name})
        elif not bad and open_:
            open_[0]["t_end"] = t

    def verdict(self, t: float) -> tuple[bool | None, str]:
        t = float(t)
        fresh = [f for rx, f in self.latest.values()
                 if math.isfinite(t - rx) and 0.0 <= t - rx <= self.stale_s]
        if not fresh:
            return None, (f"no BATTERY_STATUS from the autopilot within {self.stale_s:g} s"
                          if self.n else "no BATTERY_STATUS from the autopilot")
        bad = [f for f in fresh if f["charge_state"] in BATTERY_FAILED_STATES]
        f = bad[0] if bad else fresh[0]
        why = (f"ArduPilot reports battery {f['id']} {charge_state_name(f['charge_state'])} "
               f"(BATTERY_STATUS charge_state; {f.get('voltage_v')} V, "
               f"{f.get('remaining_pct')} % remaining)")
        return bool(bad), why

    def summary(self, to_mission_t=None) -> dict:
        conv = to_mission_t or (lambda v: v)
        return {"messages": self.n, "stale_after_s": self.stale_s,
                "fails_rtl_at": [BATTERY_CHARGE_STATES[s] for s in BATTERY_FAILED_STATES],
                "charge_states_seen": dict(self.states_seen),
                "failed_spells": [{**s, "t_start": round(conv(s["t_start"]), 3),
                                   "t_end": (None if s["t_end"] is None
                                             else round(conv(s["t_end"]), 3))}
                                  for s in self.spells],
                "last": {str(k): {**f, "rx_t": round(conv(rx), 3)}
                         for k, (rx, f) in sorted(self.latest.items())}}


class RTLCheck(NamedTuple):
    """One RTL tick's verdict (RTLFailureWatch.assess). `failed` is the FSM's
    rtl_failed; `kinds` names each reason's test (blocked_path, no_progress,
    battery, not_entered); `confirmed` whether the autopilot has reported
    RTL since the FSM entered it; `judged_progress` whether this tick fed the
    progress test. (A NamedTuple, not a dataclass: the tests load this file
    under another module name.)"""
    failed: bool
    reasons: tuple = ()
    kinds: tuple = ()
    confirmed: bool = False
    judged_progress: bool = False

    @property
    def note(self) -> str:
        return "; ".join(self.reasons)


class RTLFailureWatch:
    """rtl_failed for this node's FSM, edge G6 (RTL -> Land, "RTL fails
    (battery / blocked path)", Safety Shield p4; 2026-11-14, module
    docstring THE RTL WATCH). Pure: `assess` on every tick the FSM is in RTL,
    before it steps; `reset` on every other tick. An RTL fails when, on that
    tick:
      blocked_path  the straight leg ArduCopter's RTL flies home - the climb
                    to the return altitude, then straight home at it -
                    crosses a hard, enforced P0 zone's ring or leaves the P0
                    band: guardrail.rtl_path.check_leg on the Shield's own
                    rules, the stress harness's test, judged from the FSM's
                    RTL on (the leg does not depend on the mode);
      no_progress   the autopilot reports RTL, the aircraft is farther than
                    `home_radius_m` from home, and the climb plus the
                    distance home (remaining_to_home) fell by less than
                    min_rate_mps per second over window_s
                    (rtl_path.ProgressMonitor, the harness's test; the window
                    restarts whenever either condition lapses);
      battery       the battery verdict handed in is True (BatteryHealth:
                    ArduPilot's own battery failsafe at CRITICAL or worse);
      not_entered   the autopilot has not reported RTL entry_timeout_s after
                    the FSM entered it.
    Nothing else fails one: an exception inside the leg check is recorded
    (`errors`) and fails nothing - a watch that breaks must not land a
    return that is working."""

    def __init__(self, home, rtl_alt_m: float = RTL_ALT_M, *,
                 cone_slope: float = RTL_CONE_SLOPE_DEFAULT,
                 window_s: float = RTL_PROGRESS_WINDOW_S,
                 min_rate_mps: float = RTL_MIN_PROGRESS_MPS,
                 entry_timeout_s: float = RTL_ENTRY_TIMEOUT_S) -> None:
        hx, hy = float(home[0]), float(home[1])
        for name, v in (("home x", hx), ("home y", hy)):
            if not math.isfinite(v):
                raise ValueError(f"{name} must be finite, got {v!r}")
        for name, v in (("window_s", window_s), ("entry_timeout_s", entry_timeout_s)):
            if isinstance(v, bool) or not isinstance(v, (int, float)) \
                    or not math.isfinite(v) or v <= 0:
                raise ValueError(f"{name} must be a positive number, got {v!r}")
        self.home = (hx, hy)
        self.window_s = float(window_s)
        self.min_rate_mps = float(min_rate_mps)
        self.entry_timeout_s = float(entry_timeout_s)
        self._progress = ProgressMonitor(self.window_s, self.min_rate_mps)
        self.configure(rtl_alt_m, cone_slope, "ArduCopter's defaults (not read)")
        self._entered: float | None = None
        self._confirmed: float | None = None
        self.n_rtls = self.n_judged = self.n_failed = self.n_errors = 0
        self.failures: list[dict] = []
        self.errors: list[str] = []

    def configure(self, rtl_alt_m=None, cone_slope=None, source: str = "") -> None:
        """The return altitude (metres) and RTL_CONE_SLOPE the autopilot flies:
        a value that is missing, not finite or negative keeps ArduCopter's
        default (RTL_ALT_M, RTL_CONE_SLOPE_DEFAULT); a slope of 0 is no cone.
        The home radius is the cone's, RTL_ALT / RTL_CONE_SLOPE, never under
        RTL_HOME_RADIUS_M."""
        def num(v, lo):
            try:
                v = float(v)
            except (TypeError, ValueError):
                return None
            return v if math.isfinite(v) and v >= lo else None
        alt = num(rtl_alt_m, 0.0)            # 0: ArduCopter returns at the current altitude
        slope = num(cone_slope, 0.0)
        self.rtl_alt_m = RTL_ALT_M if alt is None else alt
        self.cone_slope = RTL_CONE_SLOPE_DEFAULT if slope is None else slope
        self.home_radius_m = max(RTL_HOME_RADIUS_M,
                                 self.rtl_alt_m / self.cone_slope if self.cone_slope > 0
                                 else 0.0)
        self.source = source or "given"

    def reset(self) -> None:
        self._progress.reset()
        self._entered = None
        self._confirmed = None

    def note_error(self, t: float, e: Exception) -> None:
        """An error while judging the leg: recorded (the first
        RTL_FAILURES_KEPT), and it fails nothing."""
        self.n_errors += 1
        if len(self.errors) < RTL_FAILURES_KEPT:
            self.errors.append(f"t={float(t):.3f}: {type(e).__name__}: {e}")

    def assess(self, t: float, x: float, y: float, up: float, mode: str | None,
               zones, band=None, battery: tuple = (None, ""), *,
               up_home: float | None = None, over=None) -> RTLCheck:
        """`up`: the height the Shield's rules (`zones`, `band`) are in.
        `up_home`: the height above HOME, the reference RTL_ALT is in, when
        the two differ (X-11: with a --ground grid the Shield is told the
        height above the DEM, terrain.TerrainReference); None, every flight
        without one, when they are the same. With it the climb's target is
        RTL_ALT above home expressed in the rules' reference AT THE AIRCRAFT
        (rtl_alt_m + up - up_home) and the progress test counts the climb
        above home. `over` (2026-11-18): the DEM the leg is judged along
        (rtl_path.check_leg, THE LEG OVER A DEM: the leg level in MSL, each
        sample's height above the ground under it, an MSL rule in MSL);
        None, every flight without a --ground grid, judges the leg at the
        one height, as before."""
        t = float(t)
        rtl_alt = self.rtl_alt_m
        up_rtl = up
        if up_home is not None and math.isfinite(float(up_home)):
            rtl_alt = self.rtl_alt_m + (float(up) - float(up_home))
            up_rtl = float(up_home)
        if self._entered is None:
            self._entered = t
            self.n_rtls += 1
        reasons: list[str] = []
        kinds: list[str] = []
        leg = None
        if zones is not None:          # None: the rules could not be read (note_error)
            try:
                leg = (check_leg(x, y, up, self.home, zones, band, rtl_alt)
                       if over is None else
                       check_leg(x, y, up, self.home, zones, band, rtl_alt, over=over))
            except Exception as e:                           # noqa: BLE001
                self.note_error(t, e)
        if leg is not None and not leg.clear:
            kinds.append("blocked_path")
            reasons.extend(f"blocked path: {r}" for r in leg.reasons)
        in_rtl = mode == "RTL"
        if in_rtl and self._confirmed is None:
            self._confirmed = t
        judged = in_rtl and math.hypot(x - self.home[0], y - self.home[1]) > self.home_radius_m
        if judged:
            stuck, why = self._progress.update(
                t, remaining_to_home(x, y, up_rtl, self.home, self.rtl_alt_m))
            if stuck:
                kinds.append("no_progress")
                reasons.append(f"{why}, the autopilot in RTL")
        else:
            self._progress.reset()
        if self._confirmed is None and t - self._entered >= self.entry_timeout_s - 1e-9:
            kinds.append("not_entered")
            reasons.append(f"RTL not entered: the autopilot still reports "
                           f"{mode or 'no mode'} {t - self._entered:.1f} s after the FSM "
                           f"asked for RTL")
        crit, why = battery if battery else (None, "")
        if crit:
            kinds.append("battery")
            reasons.append(f"battery critical: {why}")
        self.n_judged += 1
        if reasons:
            self.n_failed += 1
            if len(self.failures) < RTL_FAILURES_KEPT:
                self.failures.append({"t": round(t, 3), "kinds": list(kinds),
                                      "reasons": list(reasons)})
        return RTLCheck(failed=bool(reasons), reasons=tuple(reasons), kinds=tuple(kinds),
                        confirmed=self._confirmed is not None, judged_progress=judged)

    def summary(self, to_mission_t=None) -> dict:
        conv = to_mission_t or (lambda v: v)
        return {"rtl_alt_m": self.rtl_alt_m, "cone_slope": self.cone_slope,
                "source": self.source, "home": list(self.home),
                "home_radius_m": round(self.home_radius_m, 3),
                "progress_window_s": self.window_s, "min_progress_mps": self.min_rate_mps,
                "entry_timeout_s": self.entry_timeout_s,
                "rtls_judged": self.n_rtls, "ticks_judged": self.n_judged,
                "ticks_failed": self.n_failed,
                "failures": [{**f, "t": round(conv(f["t"]), 3)} for f in self.failures],
                "errors_n": self.n_errors, "errors": list(self.errors)}


# ArduCopter ignores a cone slope below this (mode_rtl.cpp RTL_MIN_CONE_SLOPE).
RTL_MIN_CONE_SLOPE = 0.5
# BATT_FS_CRT_ACT's values on ArduCopter 4.5 (AP_BattMonitor, Copter's list).
BATT_FS_ACTIONS = {0: "none", 1: "Land", 2: "RTL", 3: "SmartRTL or RTL",
                   4: "SmartRTL or Land", 5: "Terminate", 6: "Auto DO_LAND_START or RTL",
                   7: "Brake or Land"}


def _param_num(vals: dict, name: str) -> float | None:
    try:
        v = float(vals.get(name))
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def rtl_watch_config(vals: dict) -> tuple[float | None, float | None, str]:
    """(return altitude in metres, cone slope, source) for RTLFailureWatch.
    configure from the autopilot's RTL_ALT (centimetres; 0 returns at the
    current altitude) and RTL_CONE_SLOPE (below RTL_MIN_CONE_SLOPE ArduCopter
    flies no cone: 0 here); None for one not read, which keeps ArduCopter's
    default."""
    alt = _param_num(vals, "RTL_ALT")
    slope = _param_num(vals, "RTL_CONE_SLOPE")
    alt_m = None if alt is None or alt < 0 else alt / 100.0
    if slope is not None and slope < RTL_MIN_CONE_SLOPE:
        slope = 0.0
    read = [k for k, v in (("RTL_ALT", alt_m), ("RTL_CONE_SLOPE", slope)) if v is not None]
    missing = [k for k in ("RTL_ALT", "RTL_CONE_SLOPE") if k not in read]
    parts = ((["read from the autopilot: " + ", ".join(read)] if read else [])
             + (["ArduCopter's default for " + ", ".join(missing)] if missing else []))
    return alt_m, slope, "; ".join(parts)


def battery_failsafe_note(vals: dict) -> str:
    """What the autopilot's BATT_CRT_* / BATT_FS_CRT_ACT mean for the RTL
    watch's battery reason, in words for the record: whether ArduPilot can
    declare CRITICAL at all, and what it then does on its own (a mode it
    switches to itself is a takeover here, after which the FSM no longer
    steps)."""
    volt, mah = _param_num(vals, "BATT_CRT_VOLT"), _param_num(vals, "BATT_CRT_MAH")
    act = _param_num(vals, "BATT_FS_CRT_ACT")
    if volt is None and mah is None:
        return ("BATT_CRT_VOLT / BATT_CRT_MAH not read: whether ArduPilot can declare its "
                "battery CRITICAL is not known")
    if not (volt or 0.0) > 0 and not (mah or 0.0) > 0:
        return ("BATT_CRT_VOLT and BATT_CRT_MAH are 0: ArduPilot never declares its battery "
                "CRITICAL, so the battery reason cannot fire")
    what = " / ".join(s for s in ((f"{volt:g} V" if volt else ""),
                                  (f"{mah:g} mAh" if mah else "")) if s)
    name = BATT_FS_ACTIONS.get(int(act)) if act is not None else None
    own = ("its own critical action is none: the FSM's G6 lands the RTL"
           if act == 0 else
           f"its own critical action is {name or act} (BATT_FS_CRT_ACT): a mode it "
           f"switches to itself is a takeover here" if act is not None else
           "BATT_FS_CRT_ACT not read")
    return f"ArduPilot declares its battery CRITICAL below {what}; {own}"


def shield_step(shield: Shield, state: State, body: Action4D, shield_on: bool):
    """One tick through the Shield in the grant's body frame.

    Returns `(decision, raw_world, emitted_world, emitted_body)`. The Shield
    always evaluates (the control arm needs its violations); with it off the
    body action passes through untouched, not rotated there and back.

    The Shield takes the body frame itself (`Shield.filter_body`, since
    2026-11-04); this node turns nothing. The body -> local-NED conversion
    is the MAVLink adapter's (mavlink_adapter_node.py).
    """
    decision, emitted_body = shield.filter_body(state, body)
    if not shield_on:
        return decision, decision.raw, decision.raw, body
    return decision, decision.raw, decision.emitted, emitted_body


def build_row(*, now: float, tick: int, state: State, body: Action4D,
              decision, raw_world: Action4D, emitted_world: Action4D,
              emitted_body: Action4D | None, shield_on: bool, extra: dict,
              subject: tuple | None, fsm_out=None, stop_illegal=None,
              mode: str | None = None, setpoint: str = "pass",
              shield: Shield | None = None,
              unsafe_depth: float | None = None) -> dict:
    """The per-tick row, in the shape guardrail/kpi.py reads.

    raw / emitted stay in the world frame the Shield's checks use (kpi.py's
    repair magnitudes are frame-invariant anyway); raw_body / emitted_body are
    what the grant's interface carried. `flown` and `emitted_violations` say
    what was FLOWN and the check on it (guardrail.replay.flown_fields): the
    re-check with the Shield on, the violations of `raw` with it off - which
    is what earns the control arm its escapes - the zero action's on a Brake
    tick, and nothing (`flown: false`) on a tick where the rail sent nothing
    because the autopilot or a fault held the aircraft. `shield` is needed
    for a Brake tick, whose check is the Shield's own.
    """
    if setpoint == "brake" and shield is None:
        raise ValueError("a Brake row needs the Shield to check the zero action")
    flown = flown_fields(shield, state, decision, shield_on=shield_on,
                         setpoint=setpoint)
    row = {
        "t": round(now, 3), "tick": tick,
        "x": round(state.x, 3), "y": round(state.y, 3), "up": round(state.up, 3),
        "yaw_deg": round(state.yaw_deg, 2),
        "frame": "body",
        "raw_body": body.model_dump(),
        "emitted_body": None if emitted_body is None else emitted_body.model_dump(),
        "raw": raw_world.model_dump(),
        "emitted": emitted_world.model_dump(),
        "violations": [v.model_dump() for v in decision.violations],
        **flown,
        "repairs": ([r.model_dump() for r in decision.repairs]
                    if shield_on else []),
        "braked": bool(shield_on and decision.braked),
        "touched": bool(shield_on and decision.touched),
        "setpoint": setpoint,
        "mode": mode,
        **extra,
    }
    if subject is not None:
        row["tgt_x"], row["tgt_y"] = subject[0], subject[1]
    if fsm_out is not None:
        row.update({"fsm_state_before": fsm_out.before.value,
                    "fsm_state_after": fsm_out.state.value,
                    "fsm_edge": fsm_out.edge, "set_mode": fsm_out.set_mode,
                    "stop_illegal": bool(stop_illegal),
                    "unsafe_depth_m": (None if unsafe_depth is None
                                       else round(unsafe_depth, 4)),
                    "failsafe": bool(fsm_out.transition
                                     and fsm_out.state in FAILSAFE_STATES)})
    elif shield_on is False:
        row["fsm_state_after"] = None
    return row


def theta_basis(horizon_s: float | None) -> dict:
    """How theta's repair magnitude is measured on this run."""
    if horizon_s is None:
        return {"source": "per-operator magnitude_m", "horizon_s": None}
    one = abs(horizon_s - THETA_HORIZON_S) < 1e-9
    return {"source": ("setpoint change |dv| over one control period, the one-period "
                       "reading (guardrail.fsm.THETA_HORIZON_S; selectable, pending "
                       "the PI)" if one else
                       "velocity proxy |dv| * h (guardrail.fsm.magnitude_from_actions)"),
            "horizon_s": horizon_s,
            "note": ("under a 4-5 m/s / 2 m/s envelope a clamped command changes by "
                     "at most 1.0 / 0.4 m per period, so the theta cap cannot fire "
                     "on these flights (docs/DESIGN-escalation-fsm.md); 'no operator "
                     "converged', N-in-T, X6, X7 and direct RTL / Land / Loiter "
                     "actions are live")}


def default_theta_horizon() -> float | None:
    """The basis a run uses unless --theta-horizon-s says otherwise:
    guardrail.fsm.DEFAULT_THETA_HORIZON_S, None - each repair's own
    magnitude_m, which shield.py reports since 2026-10-07. (On 2026-10-08 a
    builder made the one-period reading the default and called it the
    grant's; reverted the next day: the grant's p5 audit example does not
    support it, and under the shipped caps it can never exceed theta.)"""
    return DEFAULT_THETA_HORIZON_S


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    s = sorted(values)
    k = min(len(s) - 1, max(0, int(math.ceil(q / 100.0 * len(s))) - 1))
    return round(s[k], 3)


def check_expected_hash(policy, expected: str | None) -> dict:
    """The `--expected-hash` pin: `{}` when none was given, else the record
    for the policy source. Raises ValueError, before take-off, when the
    policy that loaded is not the one the caller pinned - the stress
    harness passes the hash it expects with every bundle it sends
    (experiments/sweep_scenarios.py build_sitl_cmd), and until 2026-10-13
    this node did not declare the flag, so parse_known_args dropped it and a
    different bundle would have flown unchallenged."""
    if not expected:
        return {}
    got = policy.policy_hash
    want = expected if expected.startswith("sha256:") else f"sha256:{expected}"
    if got != want:
        raise ValueError(f"--expected-hash {want} but the policy that loaded is {got}: "
                         f"refusing to fly a policy other than the one pinned")
    return {"expected_hash": want, "expected_hash_ok": True}


def resolve_policy(args) -> tuple[str | None, str | None, dict]:
    """`(bundle, yaml, record)` for load_for_flight: --bundle, or --policy
    with --dev-yaml, or the default policy's signed bundle
    (guardrail.manifest.flight_policy_choice). Raises ValueError, before
    anything flies, for a YAML policy without --dev-yaml."""
    return flight_policy_choice(
        args.bundle, args.policy, dev_yaml=getattr(args, "dev_yaml", False),
        default_yaml=DEFAULT_POLICY_YAML,
        default_bundle=flight_bundle_path(DEFAULT_POLICY_YAML))


class ActionWatchdog:
    """Is the pilot's action stream alive?

    `received(t)` on every valid action; `check(t)` once per tick returns
    "stale" on the tick the stream is declared quiet (no action for longer
    than `timeout_s`, counted from the last action, or from `start(t)` when
    none has arrived since), "resumed" on the first tick after an action
    arrives during a stale spell, and None otherwise. `t` is seconds on one
    monotonic clock. Each spell is kept in `spells`. The node watches the
    pose stream with one too (`what="pose"`, since 2026-10-19)."""

    def __init__(self, timeout_s: float = ACTION_TIMEOUT_S,
                 what: str = "action") -> None:
        timeout_s = float(timeout_s)
        if not (timeout_s > 0.0 and math.isfinite(timeout_s)):
            raise ValueError(f"the {what} timeout must be a positive number of "
                             f"seconds, got {timeout_s!r}")
        self.timeout_s = timeout_s
        self.t0: float | None = None
        self.last_rx: float | None = None
        self.stale = False
        self._rx_in_spell = False
        self.spells: list[dict] = []

    def start(self, t: float) -> None:
        self.t0 = float(t)

    def received(self, t: float) -> None:
        self.last_rx = float(t)
        if self.stale:
            self._rx_in_spell = True

    def age(self, t: float) -> float | None:
        ref = self.last_rx if self.last_rx is not None else self.t0
        return None if ref is None else float(t) - ref

    def check(self, t: float) -> str | None:
        if self.stale:
            if self._rx_in_spell:
                self.stale = self._rx_in_spell = False
                self.spells[-1]["t_end"] = float(t)
                return "resumed"
            return None
        age = self.age(t)
        if age is not None and age > self.timeout_s:
            self.stale = True
            self.spells.append({"t_start": float(t), "age_s": round(age, 3),
                                "t_end": None})
            return "stale"
        return None

    def summary(self, to_mission_t=None) -> dict:
        conv = to_mission_t or (lambda v: v)
        spells = [{"t_start": round(conv(s["t_start"]), 3),
                   "t_end": (None if s["t_end"] is None
                             else round(conv(s["t_end"]), 3)),
                   "quiet_s_at_onset": s["age_s"]} for s in self.spells]
        return {"timeout_s": self.timeout_s, "spells": spells,
                "n_spells": len(spells),
                "stale_at_end": self.stale}


def pilot_fields(data) -> tuple[dict | None, str | None]:
    """`(fields, error)` for the tail of one /vla/action_4d message.

    The grant's four values alone (the stub): `(None, None)`. The VLA
    bridge's eight (VLA_ACTION_FIELDS): `decision_age_s` (None for -1, no
    decision behind the action), `decision_seq` (int or None), `author`
    (VLA_AUTHORS) and the `land` / `end` flags. Anything else is `(None,
    why)`, and the node refuses the message (not flown, not a receipt):
    another length, a non-finite or out-of-set field, an age or seq of -1
    without the other, and - the step tracker's own contract
    (guardrail/vla_link.StepTracker; review of 339f23a, P2-1) - motion that
    no decision authored: only a vla_step moves horizontally or turns, a
    vla_step always has its decision, a hold may only climb or descend (its
    step's altitude) and only with a decision behind it, a hover or a none
    is all zero. Never raises (it runs in a subscription callback)."""
    try:
        d = [float(v) for v in data]
    except (TypeError, ValueError):
        return None, "not numbers"
    if len(d) == 4:
        return None, None
    if len(d) != len(VLA_ACTION_FIELDS):
        return None, f"{len(d)} values, not 4 or {len(VLA_ACTION_FIELDS)}"
    if not all(math.isfinite(v) for v in d):
        return None, "a non-finite field"
    age, seq, author, flags = d[4:]
    if round(author) != author or int(author) not in VLA_AUTHORS:
        return None, f"author code {author!r}"
    if round(flags) != flags or not 0 <= int(flags) <= VLA_FLAG_LAND | VLA_FLAG_END:
        return None, f"flags {flags!r}"
    none = age == -1.0 and seq == -1.0
    if not none and (age < 0.0 or seq < 0.0 or round(seq) != seq):
        return None, f"decision age {age!r} / seq {seq!r}"
    who = VLA_AUTHORS[int(author)]
    moves = [abs(d[i]) > ZERO_EPS for i in range(4)]       # vx, vy, vz, yaw_rate
    if who == "vla_step" and none:
        return None, "a vla_step with no decision behind it"
    if who in ("none", "hover") and any(moves):
        return None, f"a {who} tick that moves"
    if who == "hold" and (moves[0] or moves[1] or moves[3]):
        return None, "a hold that moves horizontally or turns"
    if who == "hold" and none and moves[2]:
        return None, "a hold with no decision that climbs or descends"
    return {"decision_age_s": None if none else age,
            "decision_seq": None if none else int(seq),
            "author": who,
            "land": bool(int(flags) & VLA_FLAG_LAND),
            "end": bool(int(flags) & VLA_FLAG_END)}, None


class DecisionWatchdog:
    """Is the VLA's MOTION backed by a decision recent enough? (2026-10-23)

    `check(t, fields, rx_t, refused)` once per tick, with the latest valid
    message's pilot_fields and receipt time, and `refused` (why the latest
    message was refused, or None). A spell starts when the latest message was
    refused, when a vla_step's decision, aged by the time since its receipt,
    is older than `timeout_s` + `grace_s` (DECISION_GRACE_S says why), or
    when a hold's is older than 2 x `timeout_s` + `grace_s` - the tracker's
    longest sound hold: a step completes within the timeout of its capture,
    then holds at most the timeout. A hover and a none tick carry no
    decision and no motion (pilot_fields refuses one that moves). Returns
    "stale" on the tick a spell starts, "resumed" on the tick it ends, else
    None; each spell is kept.

    SILENCE IS NOT AGE (re-review of 23c88d3, N1). The age is the one the
    message carried plus the time since its receipt, the latter capped at
    `max_silence_s` (the node's receipt timeout): a pilot that has gone quiet
    is the receipt watchdog's to hold, and it releases the hold when messages
    come back. Uncapped, a 2 s stall near the end of a sound 10 s step aged
    its last message past the timeout here, and the mission landed.

    A SPELL ENDS ON THE NEXT VALID MESSAGE THAT IS NOT ITSELF STALE (N1).
    23c88d3 waited for a fresh vla_step, which the hold itself prevents: this
    node's LOITER puts a sound bridge in BRAKE, it suspends and sends none
    ticks, and the wait ended in the escalation's LAND. A pilot that keeps
    starting spells is counted instead (`recent(t, window)`; the node ends
    the mission at DECISION_SPELLS_MAX within DECISION_SPELLS_WINDOW_S), and
    one spell that lasts is ended past --decision-hold-max-s. With the
    grant's four values only (the stub) nothing is judged and `applies`
    stays False."""

    def __init__(self, timeout_s: float = DECISION_TIMEOUT_S,
                 grace_s: float = DECISION_GRACE_S,
                 max_silence_s: float = ACTION_TIMEOUT_S) -> None:
        timeout_s = float(timeout_s)
        if not (timeout_s > 0.0 and math.isfinite(timeout_s)):
            raise ValueError(f"the decision timeout must be a positive number of "
                             f"seconds, got {timeout_s!r}")
        self.timeout_s = timeout_s
        self.grace_s = float(grace_s)
        self.max_silence_s = float(max_silence_s)
        self.stale = False
        self.applies = False
        self.spells: list[dict] = []

    @property
    def hold_limit_s(self) -> float:
        return 2.0 * self.timeout_s + self.grace_s

    def age(self, t: float, fields: dict | None, rx_t: float | None) -> float | None:
        if not fields or fields.get("decision_age_s") is None or rx_t is None:
            return None
        quiet = min(max(0.0, float(t) - float(rx_t)), self.max_silence_s)
        return float(fields["decision_age_s"]) + quiet

    def verdict(self, t: float, fields: dict | None, rx_t: float | None,
                refused: str | None = None) -> str | None:
        """Why this tick's latest message must not fly (None: it may)."""
        if refused is not None:
            return f"refused: {refused}"
        if not fields:
            return None
        age = self.age(t, fields, rx_t)
        if fields.get("author") == "vla_step" and age is not None \
                and age > self.timeout_s + self.grace_s:
            return "stale step"
        if fields.get("author") == "hold" and age is not None and age > self.hold_limit_s:
            return "stale hold"
        return None

    def check(self, t: float, fields: dict | None, rx_t: float | None,
              refused: str | None = None) -> str | None:
        if fields is not None or refused is not None:
            self.applies = True
        why = self.verdict(t, fields, rx_t, refused)
        if why is not None and not self.stale:
            age = self.age(t, fields, rx_t)
            self.stale = True
            self.spells.append({"t_start": float(t), "why": why,
                                "age_s": None if age is None else round(age, 3),
                                "decision_seq": (fields or {}).get("decision_seq"),
                                "t_end": None})
            return "stale"
        if self.stale and why is None:
            self.stale = False
            self.spells[-1]["t_end"] = float(t)
            return "resumed"
        return None

    def held_for(self, t: float) -> float:
        return float(t) - self.spells[-1]["t_start"] if self.stale else 0.0

    def recent(self, t: float, window_s: float) -> int:
        """Spells that started within the last `window_s` seconds."""
        return sum(1 for s in self.spells if float(t) - s["t_start"] <= window_s)

    def summary(self, to_mission_t=None) -> dict:
        conv = to_mission_t or (lambda v: v)
        return {"timeout_s": self.timeout_s, "grace_s": self.grace_s,
                "hold_limit_s": self.hold_limit_s, "applies": self.applies,
                "spells": [{"t_start": round(conv(s["t_start"]), 3),
                            "t_end": (None if s["t_end"] is None
                                      else round(conv(s["t_end"]), 3)),
                            "why": s["why"],
                            "decision_age_s_at_onset": s["age_s"],
                            "decision_seq": s["decision_seq"]} for s in self.spells],
                "n_spells": len(self.spells), "stale_at_end": self.stale,
                "max_silence_s": self.max_silence_s,
                "rule": ("held like a quiet stream: a refused message, a vla_step whose "
                         "decision (its age at publication plus the time since its "
                         "receipt, at most max_silence_s) is older than timeout_s + "
                         "grace_s, a hold older than hold_limit_s; released by the next "
                         "valid message that is not itself stale")}


def pilot_from_identity(ident) -> tuple[str, dict | None, str | None]:
    """`(model_id, pilot record or None, reported digest or None)` from the
    VLA node's /vla/identity, for metrics.json and the manifest. Never raises:
    it runs in _report, where an exception loses every record of the flight.

    A kind "vla" identity carrying the server's `reported_digest` (the VLA
    bridge's) is pilot_record kind vla with that digest, and the digest goes
    to build_manifest, so vla_model_hash reads "<id>@sha256:<digest>" instead
    of "<id>@unresolved" (the companion holds no weights). A digest the
    manifest refuses (not 64 lowercase hex) is dropped, and the record says
    why. Every other identity is filed as before (the stub: kind stub, no
    digest).

    Never a VLA (review of 339f23a, P3; the same rule as vla_mission.
    identity_is_vla): a `fake:` id is a stand-in, kind stub, whatever kind
    it announces; and a bridge that never became ready (`ready` false: no
    hello, a refused warm-up) keeps kind vla but loses its digest, so
    vla_model_hash reads "<id>@unresolved" (the KPI gate refuses it,
    tools/todo.py says vla_unverified) and the record says why."""
    ident = ident if isinstance(ident, dict) else {}
    model_id = ident.get("model_id") or "unresolved:no-vla-identity"
    if not (ident.get("model_id") and ident.get("kind") in ("vla", "stub", "none")):
        return model_id, None, None
    kind = ident["kind"]
    extra = {"yaw_rate": ident.get("yaw_rate"), "frame": ident.get("frame")}
    if kind == "vla" and str(model_id).startswith("fake:"):
        kind = "stub"
        extra["filed_as_stub_because"] = "a fake: id is a stand-in, never a VLA"
    digest = ident.get("reported_digest") if kind == "vla" else None
    if kind == "vla" and ident.get("ready") is False:
        digest = None
        extra["not_ready_because"] = str(ident.get("not_ready_because")
                                         or "the VLA node never became ready")
    if digest is not None:
        try:
            pilot = pilot_record(model_id, kind, reported_digest=digest,
                                 reported_by=ident.get("reported_by"), **extra)
            return model_id, pilot, digest
        except ValueError as exc:
            refused = str(exc)
        try:
            pilot = pilot_record(model_id, kind, **extra)
        except ValueError:
            return model_id, None, None
        pilot["reported_digest_refused"] = refused
        return model_id, pilot, None
    try:
        return model_id, pilot_record(model_id, kind, **extra), None
    except ValueError:
        return model_id, None, None


def vla_ready(ident) -> tuple[bool | None, str]:
    """`(ready, why)` from /vla/identity for --vla-ready-timeout-s: None
    while there is none; True for `ready` true; False, with the node's
    reason, for `ready` false - and for a node that declares no readiness
    at all: the gate is the VLA bridge's, and until the review of 339f23a
    (P3) a stub left running passed it."""
    if not isinstance(ident, dict):
        return None, "no /vla/identity yet"
    if "ready" not in ident:
        return False, ("the VLA node declares no readiness (not the VLA bridge: "
                       "--vla-ready-timeout-s waits for sitl/ros2_vla_bridge_node.py)")
    if ident["ready"] is True:
        w = ident.get("warmup") if isinstance(ident.get("warmup"), dict) else {}
        return True, (f"ready (warm-up last round trip {w.get('last_rtt_s')} s, gate "
                      f"{w.get('max_latency_s')} s)")
    return False, str(ident.get("not_ready_because") or "the VLA node says it is not ready")


def identity_contract_problems(ident, decision_timeout_s: float) -> list[str]:
    """What the VLA node's /vla/identity announces that this node would read
    differently (review of 339f23a, P3): its `action_layout` against
    VLA_ACTION_FIELDS, its `decision_timeout_s` (the bridge's step tracker)
    against this node's --decision-timeout-s. Empty for an identity that
    announces neither (the stub). The ready gate refuses to arm on any;
    without the gate they are recorded. Never raises: it runs in
    _on_identity, a subscription callback (re-review of 23c88d3, N2: a
    layout of 5 or true raised TypeError there)."""
    if not isinstance(ident, dict):
        return []
    out = []
    layout = ident.get("action_layout")
    if layout is not None and not (isinstance(layout, (list, tuple))
                                   and list(layout) == list(VLA_ACTION_FIELDS)):
        out.append(f"action layout {layout!r} is not this node's {list(VLA_ACTION_FIELDS)}")
    dt = ident.get("decision_timeout_s")
    if dt is not None:
        try:
            same = abs(float(dt) - float(decision_timeout_s)) < 1e-9
        except (TypeError, ValueError):
            same = False
        if not same:
            out.append(f"the VLA's decision timeout {dt!r} s is not this node's "
                       f"--decision-timeout-s {decision_timeout_s:g} s: give both nodes "
                       f"the same value")
    return out


def latency_fields(tick_ns: int, pub_ns: int | None, rx_ns: int | None,
                   fresh: bool) -> dict:
    """One row's latency, in ms on time.monotonic_ns(): `tick_ms` (the tick
    callback's entry to the setpoint's publication), `action_age_ms` (the
    receipt of the action the setpoint came from to its publication) and
    `e2e_ms` (the same, on the first tick that flies a newly received
    action; None on a tick that re-flies one, publishes nothing, or holds)."""
    if pub_ns is None:
        return {"tick_ms": None, "action_age_ms": None, "e2e_ms": None}
    age = None if rx_ns is None else round((pub_ns - rx_ns) / 1e6, 3)
    return {"tick_ms": round((pub_ns - tick_ns) / 1e6, 3),
            "action_age_ms": age, "e2e_ms": age if fresh else None}


E2E_NOTES = {
    "action": ("e2e_ms: from this node's receipt of a pilot message (the /vla/action_4d "
               "callback's entry, time.monotonic_ns) to the publication on "
               "/shield/setpoint of the first setpoint flown from it. With --tick-trigger "
               "action the tick runs on the message's arrival, so e2e is the tick's own "
               "work up to the publication, plus a deferral only for a message that "
               "arrived within min_gap_s of the previous tick (tick_schedule). Not in it: "
               "the DDS hop from the pilot's publish to the callback (the message carries "
               "no stamp) and what follows the publication (the adapter's NED "
               "conversion, MAVROS, the autopilot)"),
    "timer": ("e2e_ms: from this node's receipt of a pilot message (the /vla/action_4d "
              "callback's entry) to the first setpoint flown from it; with "
              "--tick-trigger timer the node ticks on its own 10 Hz timer, so up to one "
              "tick period of it is the wait for the next tick"),
}


def latency_summary(rows: list[dict], budget_ms: float = TICK_BUDGET_MS,
                    trigger: str = "action") -> dict:
    """p50 / p95 / p99 / max of the rows' e2e_ms, action_age_ms and tick_ms,
    and the ticks over the grant's budget. The null is a node doing nothing
    per tick: on a 10 Hz timer with a pilot on its own 10 Hz clock, e2e then
    sits anywhere in 0-100 ms, the phase between the two clocks; ticking on
    the message's arrival (`trigger` "action", THE TICK SCHEDULE) e2e is the
    tick's own work."""
    out: dict = {"budget_ms": budget_ms}
    for key in ("e2e_ms", "action_age_ms", "tick_ms"):
        vals = [r[key] for r in rows if isinstance(r.get(key), (int, float))]
        out[key] = {"n": len(vals), "p50": percentile(vals, 50),
                    "p95": percentile(vals, 95), "p99": percentile(vals, 99),
                    "max": round(max(vals), 3) if vals else None,
                    "over_budget": sum(1 for v in vals if v > budget_ms)}
    out["tick_trigger"] = trigger
    out["note"] = E2E_NOTES.get(trigger, E2E_NOTES["action"])
    return out


# --------------------------------------------------------------------------- #
# The take-off read of SITL's parameters, the tick schedule, the garbage
# collector, the camera's age at the setpoint (2026-11-11). Pure: the node
# hands each its clock, its service call and its spin.
# --------------------------------------------------------------------------- #

# THE TAKE-OFF WAIT FOR SIM_SPEEDUP (2026-11-11). SIM_SPEEDUP is the node's
# only sign that the autopilot is SITL: the fence read raises a SITL
# FENCE_ALT_MAX below the policy's ceiling only then, and the stressor read
# records SITL's boot values only then. Both now take the answer of ONE
# bounded poll (takeoff_speedup), run once in bring_up before either read,
# and fail closed only after it: not SITL, no fence raise, no stressor values.
# Until then the fence read gave SIM_SPEEDUP a 5 s grace after the FENCE_*
# values (SIM_SPEEDUP_GRACE_S) and the stressor read one read. Sized from the
# rosbags of sm1111 (48 missions at 22d434b) and hs06b (9): the autopilot sent
# SIM_SPEEDUP (index 521 of 1381) in its list 1.3-5.4 s after FCU connect,
# ~0.5 s BEFORE FENCE_ALT_MAX, on every run; on 9 of the 57 MAVROS's
# parameter set did not take it from the list, and it became readable only
# when MAVROS asked for it again, 11.3-23.0 s after FCU connect (p50 20.4 s),
# past the 5 s grace on all 9 and past a 20 s wait on 5. The wait below is
# that worst case and then some; on SITL it ends at the first answer, and
# only a hardware autopilot (no SIM_* at all) waits it out on the pad.
SIM_SPEEDUP_WAIT_S = 40.0
# The two-reads-agree settle of the stressor read, after SIM_SPEEDUP answered.
SIM_PARAMS_SETTLE_S = 10.0


def takeoff_speedup(get, spin, *, now=time.time, wait_s: float = SIM_SPEEDUP_WAIT_S,
                    warn=None) -> dict:
    """THE TAKE-OFF WAIT FOR SIM_SPEEDUP: one bounded poll, through
    mavros_evidence.poll_sim_speedup (the first positive answer; a hardware
    autopilot is SYSID_THISMAV answering and SIM_SPEEDUP not, after the
    whole wait). `get(names)` returns {name: value} (the node's
    GetParameters call), `spin()` lets MAVROS fill its parameter set, `now()`
    is the clock. Returns {"sim_speedup": float or None, "autopilot_kind":
    "sitl" / "hardware" / None, "waited_s", "wait_s"}."""
    seen: dict = {}
    t0 = now()
    sp = poll_sim_speedup(get, spin, seen, now=now, wait_s=wait_s, warn=warn)
    return {"sim_speedup": sp, "autopilot_kind": seen.get("autopilot_kind"),
            "waited_s": round(now() - t0, 2), "wait_s": wait_s}


def speedup_is_sitl(*answers) -> bool:
    """Does any of these SIM_SPEEDUP answers say SITL (a positive float)?"""
    return any(isinstance(v, float) and v > 0.0 for v in answers)


def sim_params_record(get, spin, speedup: dict | None, *, now=time.time,
                      settle_s: float = SIM_PARAMS_SETTLE_S) -> dict:
    """metrics.json `sim_params.at_takeoff`: SITL's stressor parameters
    (sitl_stressors.WATCHED_SIM_PARAMS) as the autopilot holds them, through
    MAVROS, given THE TAKE-OFF WAIT's answer `speedup` (takeoff_speedup).
    `get`, `spin` and `now` as there.

    Until 2026-11-11 one read decided: on sm1111, 8 of 48 missions read
    before MAVROS had SIM_SPEEDUP - the manifest's later read found 1.0 on
    every one - and two stressed missions (gps-dropout, wind) were refused
    by the harness although their stressors flew. With SIM_SPEEDUP answered
    the values are accepted, as before, once two reads in a row agree with
    every value present, for up to `settle_s`.

    FAIL-CLOSED: no SIM_SPEEDUP after the wait leaves `values` None (`sitl`
    False for a hardware autopilot, else None, unknown), and the harness
    then refuses a stressed cell (experiments/sweep_scenarios.py
    rail_stress_check)."""
    speedup = speedup or {}
    sp = speedup.get("sim_speedup")
    waited = speedup.get("waited_s")
    if not speedup_is_sitl(sp):
        hardware = speedup.get("autopilot_kind") == "hardware"
        wait_s = speedup.get("wait_s")
        return {"sitl": False if hardware else None, "values": None, "sim_speedup": None,
                "speedup_wait_s": waited,
                "why": ("SIM_SPEEDUP absent where SYSID_THISMAV answers: a hardware "
                        "autopilot, which has no SIM_* parameter" if hardware else
                        f"SIM_SPEEDUP did not answer within the take-off wait "
                        f"({wait_s if wait_s is not None else '?'} s): MAVROS never "
                        f"had it, or the read failed")}
    names = list(WATCHED_SIM_PARAMS) + ["SIM_SPEEDUP"]
    deadline = now() + settle_s
    vals = get(names)
    cur = {k: vals.get(k) for k in WATCHED_SIM_PARAMS}
    prev, reads, stable = None, 1, False
    while True:
        if cur == prev and all(v is not None for v in cur.values()):
            stable = True
            break
        if now() >= deadline:
            break
        spin()
        prev = cur
        vals = get(names)
        cur = {k: vals.get(k) for k in WATCHED_SIM_PARAMS}
        reads += 1
    return {"sitl": True, "source": "/mavros/param (read from the autopilot)",
            "values": cur, "reads": reads, "stable": stable, "sim_speedup": sp,
            "speedup_wait_s": waited}


# THE TICK SCHEDULE (--tick-trigger action, the default since 2026-11-11).
# Until then the control tick ran on the node's own 10 Hz timer, at a phase
# unrelated to the pilot's own 10 Hz stream, so a fresh action waited up to
# one period for the next tick before the Shield saw it: on the hil smoke
# sm1111 (48 missions at 22d434b) no tick took 100 ms (max 95.98 ms) and yet
# e2e reached p99 106.1 ms - the wait plus the work. Now:
#   * a valid /vla/action_4d message ticks at once ("action"), unless the
#     previous tick started less than ACTION_TICK_MIN_GAP_S ago; then it is
#     run at the first poll past that gap ("deferred"). The tick follows a
#     10 Hz pilot one for one and never runs faster than 1 /
#     ACTION_TICK_MIN_GAP_S (12.5 Hz) for a faster one. On sm1111 (15,002
#     receipts) the interval between messages was 90.6 ms at p1 and under
#     80 ms 7 times;
#   * with no message, the TICK_POLL_S poll ticks ("timer") one TICK after
#     the previous timer tick - the 10 Hz the node always ran at with a quiet
#     stream - or TICK + TIMER_SLACK_S after a message's tick, so a message a
#     little late (p99.9 114 ms, max 172 ms on sm1111) is not overtaken by a
#     timer tick that re-flies the previous action and then defers the late
#     one.
# Each tick is the same tick as before: the FSM, the watchdogs and the rule
# clock read mission time, the hold planner spends holdpoint.TICK_QUERIES a
# tick, the stale-action rule is unchanged, and at ~10 Hz the tick counts
# metrics.json turns into seconds (x TICK) keep their meaning. Rows carry
# `tick_trigger`; metrics.json `tick_schedule`. --tick-trigger timer flies the
# old 10 Hz timer unchanged (the A/B on the rig).
#
# THE START-UP TICK (2026-11-13, module docstring). A message received before
# the mission clock starts is HELD: it ticks nothing, and the rate limit does
# not count it, so the first message after the start ticks at once. The first
# watchdog tick is due one period after the start, plus TIMER_SLACK_S when a
# message came within one period before it (the stream is live and its next
# message is due; as after a message's tick). The node delivers what queued
# during its own start-up work before the clock starts (start_mission's
# bounded drain: START_BACKLOG_SPINS spins of no wait, START_BACKLOG_S at
# most), so that backlog is held, not taken for a burst after t = 0.
TICK_POLL_S = TICK / 4.0
ACTION_TICK_MIN_GAP_S = 0.8 * TICK
TIMER_SLACK_S = 0.5 * TICK
TICK_TRIGGERS = ("action", "deferred", "timer")
_SCHED_EPS = 1e-9
START_BACKLOG_SPINS = 200
START_BACKLOG_S = 0.03


class TickScheduler:
    """When the control tick runs (THE TICK SCHEDULE). No ROS: the node calls
    `on_message(t)` from its /vla/action_4d callback for a valid message and
    `on_poll(t)` from its TICK_POLL_S timer, `t` on its monotonic clock in
    seconds; each returns the trigger to tick with now, or None. `ran(t,
    trigger)` is called as each tick starts. Nothing ticks before `start(t)`
    (bring-up's spins deliver messages too): a message before it is held
    (`held_t`, THE START-UP TICK) and never counted by the rate limit."""

    def __init__(self, period_s: float = TICK, min_gap_s: float = ACTION_TICK_MIN_GAP_S,
                 slack_s: float = TIMER_SLACK_S, poll_s: float = TICK_POLL_S) -> None:
        for name, v in (("period_s", period_s), ("min_gap_s", min_gap_s),
                        ("slack_s", slack_s), ("poll_s", poll_s)):
            if isinstance(v, bool) or not isinstance(v, (int, float)) \
                    or not math.isfinite(v) or v < 0 or (v == 0 and name != "slack_s"):
                raise ValueError(f"tick schedule: {name} must be a positive number of "
                                 f"seconds, got {v!r}")
        if min_gap_s > period_s:
            raise ValueError(f"tick schedule: min_gap_s {min_gap_s:g} s over the period "
                             f"{period_s:g} s would slow a pilot at the grant's rate")
        if poll_s >= period_s:
            raise ValueError(f"tick schedule: the poll ({poll_s:g} s) must be shorter than "
                             f"the period ({period_s:g} s)")
        self.period_s, self.min_gap_s = float(period_s), float(min_gap_s)
        self.slack_s, self.poll_s = float(slack_s), float(poll_s)
        self.t0: float | None = None
        self.last_t: float | None = None
        self.last_msg_t: float | None = None
        self.last_trigger: str | None = None
        self.pending = False
        # The last message received before start(t): held, never ticked.
        self.held_t: float | None = None
        self.held_n = 0
        self.counts = {k: 0 for k in TICK_TRIGGERS}

    def start(self, t: float) -> None:
        """The mission clock starts at `t`: the first message after it ticks
        at once (a held one is not counted by the rate limit), the first timer
        tick is due one period later, plus the slack for a live stream
        (`held_live`)."""
        self.t0 = float(t)

    def held_live(self) -> bool:
        """Did a message come within one period before the start? Then the
        stream is live and its next message is due within the period."""
        return (self.t0 is not None and self.held_t is not None
                and self.t0 - self.held_t < self.period_s - _SCHED_EPS)

    def watchdog_due_at(self) -> float | None:
        """When the poll ticks if no message comes first."""
        base = self.last_t if self.last_t is not None else self.t0
        if base is None:
            return None
        led = (self.last_trigger in ("action", "deferred") if self.last_t is not None
               else self.held_live())
        return base + self.period_s + (self.slack_s if led else 0.0)

    def start_record(self) -> dict:
        """metrics.json `tick_schedule.start`: the messages held before the
        start, the last one's age at it, and when the first watchdog was due."""
        due = None
        if self.t0 is not None:
            led = self.held_live()
            due = round(self.period_s + (self.slack_s if led else 0.0), 6)
        return {"held_before_start": self.held_n,
                "held_age_ms": (None if self.held_t is None or self.t0 is None
                                else round((self.t0 - self.held_t) * 1000.0, 3)),
                "first_watchdog_after_s": due}

    def on_message(self, t: float) -> str | None:
        # The rate limit is judged on the gap between MESSAGES, not on the
        # gap since the last tick: judged against a deferred tick's start, a
        # pilot and a poll that are both exact (the rig's stub at 10 Hz, the
        # 40 Hz poll) lock at one phase and every later message is deferred
        # by the same 20-35 ms (hs07, 2026-11-12: 22 of 30 missions, e2e to
        # 110.7 ms). A pilot faster than 1 / min_gap is still held to it.
        if self.t0 is None:
            # Before the start: held, never ticked, and not the previous
            # message of the first one after it (THE START-UP TICK).
            self.held_t, self.held_n = float(t), self.held_n + 1
            return None
        prev, self.last_msg_t = self.last_msg_t, float(t)
        if self.last_t is None or prev is None or t - prev >= self.min_gap_s - _SCHED_EPS:
            return "action"
        self.pending = True
        return None

    def on_poll(self, t: float) -> str | None:
        if self.t0 is None:
            return None
        if self.pending and self.last_t is not None \
                and t - self.last_t >= self.min_gap_s - _SCHED_EPS:
            return "deferred"
        # Half a poll early: the poll fires on its own grid, so the tick one
        # period after a timer tick lands on the poll at that period.
        if t >= self.watchdog_due_at() - self.poll_s / 2.0:
            return "timer"
        return None

    def ran(self, t: float, trigger: str) -> None:
        if trigger not in TICK_TRIGGERS:
            raise ValueError(f"unknown tick trigger {trigger!r}")
        self.last_t, self.last_trigger, self.pending = float(t), trigger, False
        self.counts[trigger] += 1

    def config(self) -> dict:
        return {"period_s": self.period_s, "poll_s": self.poll_s,
                "min_gap_s": self.min_gap_s, "slack_s": self.slack_s}


def tick_schedule_summary(rows: list[dict], mode: str,
                          sched: TickScheduler | None = None,
                          start_backlog: dict | None = None) -> dict:
    """metrics.json `tick_schedule`: which trigger ran each tick, the rate,
    and the interval between tick starts (from the rows' mission time `t`,
    ms resolution); with the action trigger, THE START-UP TICK's record
    (`start`: the messages held before the start and the backlog the node
    delivered before it)."""
    ts = [float(r["t"]) for r in rows if isinstance(r.get("t"), (int, float))]
    gaps = [round((b - a) * 1000.0, 3) for a, b in zip(ts, ts[1:])]
    span = ts[-1] - ts[0] if len(ts) > 1 else 0.0
    by: dict = {}
    for r in rows:
        k = r.get("tick_trigger") or "unrecorded"
        by[k] = by.get(k, 0) + 1
    out = {"mode": mode, "ticks_by_trigger": by,
           "tick_hz": round((len(ts) - 1) / span, 3) if span > 0 else None,
           "interval_ms": {"n": len(gaps), "p1": percentile(gaps, 1),
                           "p50": percentile(gaps, 50), "p99": percentile(gaps, 99),
                           "min": min(gaps) if gaps else None,
                           "max": max(gaps) if gaps else None}}
    if mode == "action":
        out.update(sched.config() if sched is not None else TickScheduler().config())
        out["rule"] = ("a valid /vla/action_4d message ticks at once unless the previous "
                       "tick started under min_gap_s ago (then at the first poll past it: "
                       "deferred); with no message the poll ticks period_s after a timer "
                       "tick, period_s + slack_s after a message's tick; a message "
                       "received before the mission clock starts is held (the backlog of "
                       "the node's start-up work is delivered before it), the first one "
                       "after the start ticks at once, and the first watchdog waits "
                       "period_s + slack_s when one came within period_s before the start")
        out["start"] = dict(sched.start_record() if sched is not None else {},
                            backlog=start_backlog)
    else:
        out.update(period_s=TICK, rule="the node's own 10 Hz timer (--tick-trigger timer)")
    return out


# THE GARBAGE COLLECTOR (--gc-pacing on, the default since 2026-11-11): the
# pacer, its measurements and its constants live in guardrail/gc_pacing.py
# since 2026-11-15 (the PAS perception node paces its collector with it too).
from guardrail.gc_pacing import (GC_FULL_EVERY_S, GC_THRESHOLD_NEVER,     # noqa: E402,F401
                                 GcPacer)


# THE CAMERA'S AGE AT THE SETPOINT (2026-11-11; WP3-13, the meeting's "did you
# measure the latency from camera input to action output?"). Per published
# setpoint that was flown from a camera-backed action, `camera_age_ms`: how
# old the camera frame behind that action was when the setpoint went out.
# It adds two durations, each read on ONE monotonic clock, and never
# subtracts readings of two clocks, so no offset between hosts enters:
#   1. decision_age_s, carried by the message: the VLA bridge's monotonic
#      now - t_capture at its publication (sitl/ros2_vla_bridge_node.py).
#      t_capture is the front frame's header stamp mapped onto the bridge's
#      monotonic clock when that stamp is on the bridge host's wall clock,
#      else the frame's arrival at the bridge (_RingObs.capture_time);
#   2. this node's monotonic clock from the message's receipt (the callback's
#      entry) to the setpoint's publication: the row's action_age_ms.
# Not in it: the DDS hop from the bridge's publish to this node's callback
# (the message carries no stamp; sub-ms on one host), and the camera
# driver's own delay before its header stamp. So it is a lower bound by that
# hop. Only `pass` setpoints from a valid message whose decision_age_s is
# set (a vla_step or a hold); a stop for a stale input, the FSM's own stop,
# a hold transit and the stub's four values carry none. The row's
# `decision_age_s` (the decision watchdog's view at row time) is unchanged.
CAMERA_AGE_CLOCKS = (
    "camera_age_ms = decision_age_s (the VLA bridge's monotonic clock: its now - "
    "t_capture at publication; t_capture the front frame's header stamp mapped onto "
    "that clock when the stamp is on the bridge host's wall clock, else the frame's "
    "arrival) + this node's monotonic clock from the message's receipt to the "
    "setpoint's publication. Two durations on one clock each, no cross-clock "
    "subtraction. Not counted: the DDS hop bridge -> this node, and the camera "
    "driver's delay before its stamp: a lower bound by that hop")


def camera_age_ms(decision_age_s, rx_ns: int | None, pub_ns: int | None) -> float | None:
    """The age of the camera frame behind a published setpoint, ms
    (THE CAMERA'S AGE AT THE SETPOINT), or None when there is none to give:
    no publication, no receipt, no decision age (the stub, a hover / none
    tick)."""
    if pub_ns is None or rx_ns is None or decision_age_s is None:
        return None
    age = float(decision_age_s)
    if not math.isfinite(age) or age < 0:
        return None
    return round(age * 1000.0 + (pub_ns - rx_ns) / 1e6, 3)


def camera_latency_summary(rows: list[dict], extended_messages: int) -> dict:
    """metrics.json `camera_to_setpoint`, next to tick_latency: p50 / p95 /
    p99 / max of the rows' camera_age_ms. With no camera-backed setpoint the
    figures are None and `applies` False with the reason - never a zero."""
    vals = [r["camera_age_ms"] for r in rows
            if isinstance(r.get("camera_age_ms"), (int, float))]
    out = {"applies": bool(vals), "n": len(vals),
           "p50": percentile(vals, 50), "p95": percentile(vals, 95),
           "p99": percentile(vals, 99),
           "max": round(max(vals), 3) if vals else None,
           "clocks": CAMERA_AGE_CLOCKS}
    if not vals:
        out["why"] = ("no camera behind the actions: the pilot sent the grant's four "
                      "values only (the stub, or no pilot), so there is no frame age to "
                      "report - not a zero" if not extended_messages else
                      "no setpoint was flown from a camera-backed decision: every "
                      "published setpoint came from a hover / none tick, a stop for a "
                      "stale input, the FSM's own stop or a hold transit")
    else:
        bridge = [r["camera_age_ms"] - r["action_age_ms"] for r in rows
                  if isinstance(r.get("camera_age_ms"), (int, float))
                  and isinstance(r.get("action_age_ms"), (int, float))]
        out["bridge_part_ms_p50"] = percentile([round(b, 3) for b in bridge], 50)
        out["node_part_ms_p50"] = percentile(
            [r["action_age_ms"] for r in rows
             if isinstance(r.get("camera_age_ms"), (int, float))
             and isinstance(r.get("action_age_ms"), (int, float))], 50)
    return out


# --------------------------------------------------------------------------- #
# The node (needs rclpy + mavros_msgs)
# --------------------------------------------------------------------------- #

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from geometry_msgs.msg import PoseStamped
    try:
        # The own-velocity forecast's input; a stand-in ROS stack without it
        # (tests/test_railinputs_node.py) runs the node without the velocity.
        from geometry_msgs.msg import TwistStamped
    except ImportError:
        TwistStamped = None
    from std_msgs.msg import Float32MultiArray, String
    from mavros_msgs.msg import ExtendedState, State as MavState, StatusText
    from mavros_msgs.srv import (CommandBool, CommandTOL, SetMode, StreamRate,
                                 VehicleInfoGet)
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters
    try:
        # The autopilot's home, checked against --start before arming
        # (2026-11-06); a stand-in ROS stack without them runs without it.
        from mavros_msgs.msg import HomePosition
    except ImportError:
        HomePosition = None
    try:
        from std_srvs.srv import Trigger            # /mavros/home_position/req_update
    except ImportError:
        Trigger = None
    try:
        # The GPS fix behind State.fix_ok (2026-11-07); a stand-in ROS stack
        # without it runs the node with fix_ok None, as before.
        from sensor_msgs.msg import NavSatFix
    except ImportError:
        NavSatFix = None
    try:
        # The autopilot's raw MAVLink, for its EKF_STATUS_REPORT behind
        # State.estimator_ok (2026-11-12); a stand-in ROS stack without it
        # runs the node with estimator_ok None, as before.
        from mavros_msgs.msg import Mavlink
    except ImportError:
        Mavlink = None
    try:
        # SITL's stressor parameters as the autopilot broadcasts them.
        from mavros_msgs.msg import ParamEvent
    except ImportError:
        ParamEvent = None
    try:
        # EXTENDED_SYS_STATE asked of the autopilot by its message id
        # (2026-11-07); a stand-in ROS stack without it runs as before.
        from mavros_msgs.srv import MessageInterval
    except ImportError:
        MessageInterval = None
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

# After the GeoFence parameters are all read, how long the fence read still
# waits for SIM_SPEEDUP (2026-11-11). MAVROS fills /mavros/param while it
# pulls the autopilot's list, and SIM_SPEEDUP - the node's only sign that the
# autopilot is SITL, whose FENCE_ALT_MAX it may raise - can arrive after the
# FENCE_* values. Four of the 36 missions of the grant example sweep on hil
# (ge1110, 10 Nov: 012, 017, 030, 031) read the fence complete and SIM_SPEEDUP
# still absent, so the raise was not allowed, FENCE_ALT_MAX stayed 30 m under
# a policy ceiling of 80 m, and the climb to the 50 m start breached the
# autopilot's own fence: RTL before the mission began, nothing flown. A
# hardware autopilot never answers SIM_SPEEDUP; there the read goes on after
# this grace, as before. Since the evening of 2026-11-11 bring_up waits for
# SIM_SPEEDUP once, before this read (THE TAKE-OFF WAIT FOR SIM_SPEEDUP,
# SIM_SPEEDUP_WAIT_S): the grace was too short for 8 of the 48 missions of
# sm1111 and 1 of 9 of hs06b, and applies now only to a read made without
# that wait.
SIM_SPEEDUP_GRACE_S = 5.0


def fence_read_done(vals: dict, fence_complete_for_s: float,
                    grace_s: float = SIM_SPEEDUP_GRACE_S,
                    speedup_polled: bool = False) -> bool:
    """Is the take-off fence read finished? Every FENCE_PARAMS value read,
    and SIM_SPEEDUP too - or `grace_s` passed since the fence values were
    complete (a hardware autopilot has no SIM_SPEEDUP) - or, with
    `speedup_polled`, THE TAKE-OFF WAIT has already given its answer, so
    the FENCE_* values are all this read still needs."""
    if not all(vals.get(k) is not None for k in FENCE_PARAMS):
        return False
    if speedup_polled:
        return True
    return vals.get("SIM_SPEEDUP") is not None or fence_complete_for_s >= grace_s


def takeoff_fence_refusal(alt_m: float | None, fence_alt_max: float | None,
                          margin_m: float | None,
                          backstop_active: bool | None) -> str | None:
    """Why a take-off to `alt_m` must not start, or None (2026-11-11). The
    autopilot's GeoFence acts `margin_m` below FENCE_ALT_MAX, so a climb to
    an altitude at or above that line ends in the autopilot's own RTL before
    the mission begins - the stress harness refuses such a start for the same
    reason (sweep_scenarios.rail_refusal). Nothing is refused while the fence
    is off or unread: there is no backstop to trip, or nothing known to judge."""
    if not backstop_active or alt_m is None or fence_alt_max is None:
        return None
    line = float(fence_alt_max) - float(margin_m or 0.0)
    if float(alt_m) < line:
        return None
    return (f"take-off to {float(alt_m):g} m refused: the autopilot's GeoFence acts at "
            f"FENCE_ALT_MAX {float(fence_alt_max):g} m less its {float(margin_m or 0.0):g} m "
            f"margin ({line:g} m), so the backstop would end the climb before the "
            f"mission starts")


def landed_from(mode: str | None, ext_landed: bool | None, armed: bool | None,
                flew_armed: bool) -> bool:
    """The FSM's "landed" input (G10, X5), only in RTL or LAND as before
    (2026-11-07): /mavros/extended_state's landed_state once the autopilot
    sends it (`ext_landed` not None), otherwise ArduCopter's own end of a
    landing - LAND and the end of RTL disarm on touchdown - read as
    /mavros/state's armed flag falling after this run armed. A disarm before
    the run armed, or in any other mode, is not a landing."""
    if mode not in ("RTL", "LAND"):
        return False
    if ext_landed is not None:
        return bool(ext_landed)
    return armed is False and bool(flew_armed)


if HAVE_ROS:
    LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    LANDED_STATE_ON_GROUND = 1
    # ArduCopter puts EXTENDED_SYS_STATE (MAVLink common, id 245) in none of
    # the stream groups REQUEST_DATA_STREAM switches on, so with the streams
    # alone /mavros/extended_state stays empty: every hil run of 6-7 Nov
    # recorded 0 messages on it (rh01_032's bag), and the FSM never read
    # "landed" - G12 took rh01_032 to Land and G10 never fired (WP3-07,
    # "G10 never fired after a landing"). Asked for by id instead
    # (MAV_CMD_SET_MESSAGE_INTERVAL through MAVROS's sys_status plugin).
    MSG_EXTENDED_SYS_STATE = 245
    EXT_STATE_RATE_HZ = 2.0

    class ShieldNode(Node):
        def __init__(self, args, out: Path, subject=None) -> None:
            super().__init__("safety_shield")
            self.args = args
            self.shield_on = args.shield == "on"
            self.dynamic = args.dynamic
            self.out = out
            self.max_s = float(args.max_s)

            # The signed bundle (given, or the default policy's); a YAML only
            # under --dev-yaml, recorded so. Refuses before take-off, never
            # mid-air.
            bundle, yaml_path, choice = resolve_policy(args)
            # The declared ground (--ground, X-11; guardrail/terrain.py
            # load_ground), placed in the frame of the policy about to fly:
            # with it an MSL rule passes the flight gate and the Shield
            # converts it. Without --ground (the default) nothing below
            # differs from every flight before 2026-11-18: no ground, the
            # pose's height above home handed to the Shield as it was.
            # --ground-sha256 (2026-11-18): the file must hash to it, checked
            # before it is read (terrain.ground_file); either way its path
            # and hash are recorded (metrics.json ground.file), which is what
            # the replay bundle packs and checks.
            self.ground, self.ground_file = ground_from_args(args, bundle, yaml_path,
                                                             root=ROOT)
            self.policy, self.policy_source = load_for_flight(
                bundle, yaml_path,
                allow_unverified=args.allow_unverified_bundle,
                **({} if self.ground is None else {"ground": self.ground}))
            self.policy_source.update(choice)
            self.policy_source.update(check_expected_hash(
                self.policy, getattr(args, "expected_hash", None)))
            self.get_logger().info(
                f"policy {self.policy.policy_id} v{self.policy.version} "
                f"{self.policy.policy_hash} from {self.policy_source['kind']} "
                f"{self.policy_source['path']} - signature "
                f"{self.policy_source['signature']}")
            compiler = (ConstraintCompiler(self.policy) if self.ground is None
                        else ConstraintCompiler(self.policy, ground=self.ground))
            # The default mission is written out literally: the stress
            # harness (experiments/sweep_scenarios.py SITL_RAIL_COMMAND) and
            # its test compare scenarios against exactly this text.
            self.mission = (compiler.parse_command(args.command) if args.command
                            else compiler.parse_command("fly to the northeast pad at 6 m/s"))
            tgt = parse_point(args.target, "--target")
            # A scenario's start (2026-11-06): the policy-frame position of
            # the autopilot's home (every pose is offset by it), and the
            # altitude the mission starts at.
            start = parse_point(getattr(args, "start", None), "--start")
            self.start_given = start is not None
            self.start_xy = (start[0], start[1]) if start else (0.0, 0.0)
            self.start_up = start[2] if start else None
            # Over a DEM (--ground a grid) the Shield is told the height above
            # the DEM under the aircraft, from the pose's height above home
            # and home's elevation (--ground-home-msl: the autopilot's own
            # home altitude, the SITL rails' 584 m; checked against the DEM's
            # elevation at home, terrain.TerrainReference). A flat --ground
            # is the home's own ground: the height above home is the height
            # above it, so nothing is converted. None without --ground.
            self.terrain = None
            self.ground_kw: dict = {}
            if self.ground is not None:
                hmsl = getattr(args, "ground_home_msl", None)
                if is_dem(self.ground):
                    self.terrain = TerrainReference(self.ground, hmsl, home_xy=self.start_xy,
                                                    source="--ground-home-msl")
                    self.ground_kw = {"ground": self.ground, "up_ref": "terrain"}
                else:
                    e = float(self.ground.elevation_at())
                    if hmsl is not None and abs(float(hmsl) - e) > 1e-6:
                        raise ValueError(f"REFUSED before take-off: --ground-home-msl {hmsl:g} "
                                         f"and the flat ground {self.ground!r} differ")
                    self.ground_kw = {"ground": self.ground}
            self.has_target = not getattr(args, "no_target", False)
            self.reach_m = float(getattr(args, "reach_m", REACH_M))
            upd: dict = {}
            if tgt is not None:
                upd.update(target_x=tgt[0], target_y=tgt[1])
            if tgt is not None and tgt[2] is not None:
                upd["cruise_alt_m"] = tgt[2]
            elif self.start_up is not None:
                upd["cruise_alt_m"] = self.start_up
            if start is not None:
                upd.update(start_x=start[0], start_y=start[1])
            if not self.has_target:
                # No target: the mission's (its CSP's) target is the start,
                # as the stress harness writes a target-less mission's.
                upd.update(target_x=self.start_xy[0], target_y=self.start_xy[1])
            if upd:
                self.mission = self.mission.model_copy(update=upd)
            if args.speed is not None:
                self.mission = self.mission.model_copy(
                    update={"speed_pref_mps": float(args.speed)})
            # The stress harness's scenario (--scenario-meta) and its
            # mid-flight events (--events), applied in the tick once the
            # mission time reaches each (_apply_scenario_events).
            meta = getattr(args, "scenario_meta", None)
            self.scenario_meta = load_scenario_meta(meta) if meta else None
            ev_path = getattr(args, "events", None)
            self._ev_pending = load_events(ev_path) if ev_path else []
            self._ev_file = str(ev_path) if ev_path else None
            self._ev_declared = len(self._ev_pending)
            self._ev_state = None               # SE.EventState, once the Shield exists
            self._ev_records: list[dict] = []
            self._ev_unexpected: list[dict] = []
            self._ev_live_from: dict[str, float] = {}
            self._ev_heading = (1.0, 0.0)
            self._ev_heading_source = "default (no start heading yet)"
            # The autopilot's home (/mavros/home_position/home) and its check
            # against --start before arming (check_home).
            self.home_geo = None
            self.home_check: dict | None = None
            self.n_home_requests = 0

            # The rule clock, one callable for the Shield and every
            # generation's CSP (guardrail.replay.rule_clock): scripted from
            # --clock-start with the mission time, else the wall clock in the
            # mission's time zone. Mission time is None until mission start.
            # It can be PINNED for one query (`_clock_pin`, under the tick's
            # lock): a tick judges the position under the previous tick's
            # rules at the previous tick's reading (_prev_judgement), which
            # the stress harness reads to tell a breach the aircraft made from
            # a rule that switched on under it.
            self.mission_t0: float | None = None
            base_clock, self.clock_record = rule_clock(
                getattr(args, "clock_start", None), elapsed=self._t,
                tz=getattr(args, "clock_tz", None))
            self._clock_pin = None

            def clock():
                pin = self._clock_pin
                return pin if pin is not None else base_clock()
            self.clock = clock
            self._last_clock = None             # the previous row's reading
            self._gen_after_tick: int | None = None
            # The declared world (--world, guardrail/world_map.py; WP1-07):
            # the buildings and roads maps a distance_envelope is measured
            # on, given to every Shield this node builds. Without --world no
            # map is passed, as every flight before 2026-11-17 flew. A policy
            # whose distance_envelope has no map is refused here, before the
            # episode record moves anything aside and before take-off.
            wpath = getattr(args, "world", None)
            self.world = load_world(wpath, ROOT) if wpath else None
            self.world_kw = self.world.shield_kwargs() if self.world else {}
            gaps = missing_maps(self.policy, self.world)
            # A distance_envelope over people (2026-11-18, guardrail/
            # people.py): SITL's world has no pedestrians, so its one person
            # is a --subject declared as one; refused without it. Nothing
            # for a policy without such a rule.
            ppl_kw, ppl_gaps = declared_subject_source(
                self.policy, subject, getattr(args, "subject_class", None),
                rail="the ArduPilot SITL rail")
            self.world_kw.update(ppl_kw)
            gaps = gaps + ppl_gaps
            if gaps:
                raise ValueError("REFUSED before take-off: " + "; ".join(gaps))
            # Episode record FIRST: it moves an earlier flight's audit and
            # generation files aside, so the logger below starts a fresh file.
            self.rec = EpisodeRecord(out, self.policy, self.mission,
                                     lookahead_s=LOOKAHEAD_S,
                                     cost_map=getattr(args, "cost_map", False),
                                     clock=self.clock,
                                     clock_record=self.clock_record,
                                     **({} if self.ground is None
                                        else {"ground": self.ground}))
            (out / "prompt.yaml").write_text(compiler.build_prompt(self.mission),
                                             encoding="utf-8")
            # No escalation FSM inside the Shield: this node runs THE FSM,
            # because only it sees the autopilot (guardrail.replay.rail_shield).
            # pipe_clock: the setpoints sent (note_flown) are aged on the
            # node's real clock, not on filter()'s counted 0.1 s ticks (read
            # only with the own-velocity forecast on; docs/DESIGN-own-
            # velocity-forecast.md, "The pipe's clock"). The world's maps
            # (--world, read above) go to it; none without --world.
            # --arc-forecast on (2026-11-17, WP3-04 / ARCH-04; off by
            # default, so every flight flies as before): filter_body
            # forecasts a body action with a yaw rate along its arc.
            self.arc = getattr(args, "arc_forecast", "off") == "on"
            # --path-repair (2026-11-17, WP3-06): on (default) the Shield's
            # default chain with PathRepair after LateralProjection (in chain
            # mode it detours round a --subject's stand-off ring only); off is
            # the chain every run before 2026-11-17 flew.
            self.path_repair = getattr(args, "path_repair", "on") == "on"
            self.shield = rail_shield(self.policy, lookahead_s=LOOKAHEAD_S,
                                      dt=LOOKAHEAD_DT_S, clock=self.clock,
                                      pipe_clock=time.monotonic, **self.world_kw,
                                      **self.ground_kw,
                                      path_repair=self.path_repair,
                                      **({"arc_forecast": True} if self.arc else {}))
            self._ev_state = SE.EventState(shields=[self.shield])
            # The own-velocity forecast (OWN_VELOCITY_DEFAULT: off here), the
            # Shield-on arm only; the latest velocity_local and its receipt.
            self.own_v = (getattr(args, "own_velocity_forecast", OWN_VELOCITY_DEFAULT)
                          == "on" and self.shield_on)
            self.vel: tuple | None = None
            self.vel_rx: float | None = None
            self.n_vel_used = 0
            self.n_vel_seen = 0
            # Held by every control tick and by the REST hot-apply thread
            # while it swaps the Shield's rules and records the generation.
            self._lock = threading.RLock()
            # What runs the tick (THE TICK SCHEDULE: a pilot message, or the
            # 10 Hz timer with --tick-trigger timer); the scheduler exists
            # from here, so a message bring-up's spins deliver is held (THE
            # START-UP TICK), and ticks nothing until the mission clock
            # starts. And the collector's pacing (THE GARBAGE COLLECTOR),
            # started with the mission clock.
            self.tick_trigger = getattr(args, "tick_trigger", "action")
            self._sched: TickScheduler | None = (
                None if self.tick_trigger == "timer" else TickScheduler())
            # THE START-UP TICK: the backlog start_mission delivered before
            # the mission clock started (None with --tick-trigger timer).
            self.start_backlog: dict | None = None
            self._in_tick = False
            self.gc_pacer = GcPacer(pace=getattr(args, "gc_pacing", "on") == "on")
            self.hot = None             # guardrail.api.FlightHotApply, if served
            self.audit = AuditLogger(out / AUDIT_FILE, self.policy,
                                     episode_id=str(self.rec.episode_id))

            # Escalation FSM, Shield-on arm only (the control arm records
            # fsm_state_after: null).
            self.theta_horizon_s = (default_theta_horizon()
                                    if args.theta_horizon_s == "auto"
                                    else (None if args.theta_horizon_s == "none"
                                          else float(args.theta_horizon_s)))
            cfg = FSMConfig.from_yaml(args.fsm_config) if args.fsm_config else FSMConfig()
            # A slow pilot's N-in-T window (THE PILOT'S DECISION PERIOD).
            cfg, self.decision_period = fsm_config_for_pilot(
                cfg, getattr(args, "decision_period_s", None))
            self.fsm = EscalationFSM(cfg) if self.shield_on else None
            # PathRepair (default chain since 2026-11-17) stands aside where
            # the projection is over THIS FSM's theta; the Shield has no FSM.
            self.shield.theta_config = cfg
            self.fsm_fault: str | None = None
            self.hold_mode = getattr(args, "hold_mode", DEFAULT_HOLD_MODE)
            # What was asked of the autopilot, in the FSM's words and
            # ArduCopter's, and which request explains a mode change.
            self._asks = ModeRequests(self.hold_mode)
            # Where the FSM's Loiter holds and its Land lands (--hold-points,
            # since 2026-11-07): the hold-point search's plan, a moved point
            # flown as a GUIDED transit and then the hold (guardrail/
            # hold_flight.py, the mode mapping); "in_place" holds where the
            # aircraft is, as this node did until then.
            self.moved_holds = getattr(args, "hold_points", "moved") == "moved"
            # How a Loiter is flown (--hold-flight, since 2026-11-07 night):
            # "guided", held in GUIDED by this node on the plan's point, or
            # "brake", ArduCopter's BRAKE on arrival / at once (the flow until
            # then, whose hand-over carried hs04_005/006 into the sweep).
            # --hold-points in_place keeps ArduCopter's own hold, as before.
            self.hold_flight = HoldFlight(
                guided_hold=(self.moved_holds
                             and getattr(args, "hold_flight", "guided") == "guided"))
            # Whether the Loiter's hold is kept (the FSM's X9, 2026-11-11):
            # the hold's distance and reach, judged before each FSM step.
            self.hold_watch = HoldWatch()
            self._last_pub_ns: int | None = None   # the last setpoint published
            self.hold_planner = holdpoint.HoldPlanner()
            self._hold_work: dict | None = None    # this tick's, for its row

            # A DECLARED subject, so subject_standoff rules bind on a rail with
            # no camera. Ground truth, not perception.
            self.subject = subject
            self.subject_class = args.subject_class
            if subject is not None:
                self.shield.set_subject(subject[0], subject[1], self.subject_class)
                rings = [(c.id, c.min_range_m)
                         for c in standoff_rules(self.policy.constraints,
                                                 self.subject_class)]
                self.get_logger().info(
                    f"{self.subject_class} DECLARED at ({subject[0]:.1f}, "
                    f"{subject[1]:.1f}); "
                    + (", ".join(f"{i} {r:.0f}m" for i, r in rings) if rings
                       else "WARNING no standoff rule binds - the ring is inert"))

            self.state: State | None = None
            self.raw_body = Action4D()
            self.n_actions = 0
            # The action stream: its watchdog, and the receipt of the latest
            # action (sequence number and monotonic ns) for the tick latency.
            self.watchdog = ActionWatchdog(getattr(args, "action_timeout_s",
                                                   ACTION_TIMEOUT_S))
            # The pose stream's (/mavros/local_position/pose). Until
            # 2026-10-19 nothing watched it: a pose that stopped while the
            # setpoints still went through was re-checked every tick, the
            # rows and the trajectory repeated it, and zone dwell over that
            # trajectory read 0 for a flight that may have crossed a zone.
            self.pose_watchdog = ActionWatchdog(
                getattr(args, "pose_timeout_s", POSE_TIMEOUT_S), what="pose")
            # Why this node holds the aircraft, if it does: "action" (the
            # pilot went quiet) and/or "pose" (the position stopped). One
            # hold; GUIDED again only once no reason is left, so an action
            # stream that resumes cannot release a hold the pose still needs.
            self._holds: set[str] = set()
            self._action_seq = 0
            self._action_rx_ns: int | None = None
            self._flown_seq = 0
            # The decision behind the latest action (pilot_fields; None for
            # the grant's four values alone, the stub), its receipt time,
            # and the watchdog on its age (2026-10-23, the VLA bridge).
            self.pilot: dict | None = None
            self._pilot_rx: float | None = None
            self.n_pilot_msgs = 0
            self.n_pilot_malformed = 0
            self.pilot_error: str | None = None
            # Silence ages a decision only up to the receipt timeout (N1).
            self.decision_watchdog = DecisionWatchdog(
                getattr(args, "decision_timeout_s", DECISION_TIMEOUT_S),
                max_silence_s=self.watchdog.timeout_s)
            # When each hold reason was placed (the escalation counts the
            # time of THIS node's decision hold, never the FSM's own: N4).
            self._hold_t: dict[str, float] = {}
            self.decision_escalation: dict | None = None
            self._multi_pub_streak = 0
            self.action_publishers_flagged = False
            self.action_publisher_names: set[str] = set()
            self.decision_hold_max_s = float(getattr(args, "decision_hold_max_s",
                                                     DECISION_HOLD_MAX_S))
            # Review of 339f23a (P1-A): why the latest message was refused
            # (None when it was flown), what it carried, the last decision
            # seq flown, and the most publishers seen on /vla/action_4d.
            self.pilot_refused: str | None = None
            self.pilot_sent: list[float] | None = None
            self._last_seq: int | None = None
            self.action_publishers_max: int | None = None
            self.contract_problems: list[str] = []
            self.vla_land: dict | None = None
            self.vla_ready: dict | None = None
            self.mav_connected = False
            self.mode: str | None = None
            self.landed: bool | None = None
            # /mavros/state's armed flag, and the mission's end after the FSM
            # ended it (its terminal edge; logging went on to the landing).
            self.armed: bool | None = None
            # Whether /mavros/state has read armed during this run: a disarm
            # after it, in RTL or LAND, is ArduCopter's own end of a landing
            # (landed_input).
            self._flew_armed = False
            self._terminal_t: float | None = None
            self.after_terminal: dict | None = None
            self.traj: list[dict] = []
            self.rows: list[dict] = []
            # Evidence of the topology, gathered from the live system rather
            # than asserted (guardrail.manifest.check_topology_evidence), the
            # same way as the mission-style node (sitl/mavros_evidence.py).
            # vla_node: the VLA node's own host, from /vla/identity, is None
            # until (and unless) it reports one; the grant puts it on the Orin.
            self.hil_evidence: dict = initial_evidence(vla_node=True)
            # SROS 2 (sitl/run_ros2_demo.sh SROS2=1): recorded only when it is
            # on, so a run without it records what it always did.
            security = self.hil_evidence.get("ros2_security")
            if security is not None:
                self.get_logger().info(
                    f"SROS 2 {security['strategy']}: enclave {security['enclave']}, "
                    f"identity CA {security['identity_ca_sha256']}")
            self.vla_identity: dict | None = None
            self.fence_status: list[dict] = []
            # The autopilot's fence parameters as read back at take-off, and
            # the zones held back because the aircraft was in or near them.
            self.autopilot_fence: dict = {}
            self.fence_column_m: float | None = None
            self.fence_margin_m: float = FENCE_MARGIN_M
            self.deferred_fences: list[str] = []
            self.zones_over_home: list[str] = []
            # The zones the autopilot holds (the adapter confirmed the
            # upload): never held back again, since each upload replaces the
            # list (guardrail.fence_export.defer_near). And the last list
            # sent, which the adapter's confirmation is matched against.
            self.fences_enforced: dict = {}
            self._fences_sent: dict | None = None
            self.sim_speedup: float | None = None
            # THE TAKE-OFF WAIT FOR SIM_SPEEDUP's answer (takeoff_speedup),
            # shared by the fence read and the stressor read; None until run.
            self.takeoff_speedup: dict | None = None
            self.n_touched = self.n_braked = 0
            self.spawn_t = self.spawn_pos = None
            self.dynamic_fence_id: str | None = None
            self.done = False
            self.reached = False
            self.takeover: dict | None = None
            self.failsafe_t: float | None = None
            self.n_pose = 0
            self._last_rate_req = 0.0
            # The GPS fix (FIX_TOPIC): the latest NavSatStatus and its
            # receipt on the node's monotonic clock; fix_ok_from reads them
            # each tick. And the X8 verdicts the Shield handed this node's FSM.
            self.fix_status: int | None = None
            self.fix_rx: float | None = None
            self.n_fix = 0
            self.n_position_lost = 0
            # The autopilot's own verdict on its estimate (EKF_STATUS_REPORT
            # from MAVLINK_SOURCE_TOPIC, 2026-11-12): fed per report, read per
            # tick (State.estimator_ok).
            self.estimator = EstimatorHealth()
            # THE RTL WATCH (2026-11-14): the autopilot's battery verdict
            # (BATTERY_STATUS from the same feed) and the FSM's rtl_failed
            # (G6), judged on every tick the FSM is in RTL. Home is --start's
            # position in the policy frame, as for home_reached.
            self.battery = BatteryHealth()
            self.rtl_watch = RTLFailureWatch(self.start_xy)
            self.rtl_params: dict | None = None
            # SITL's stressor parameters: as read back at take-off, every
            # change the autopilot broadcast (on the mission clock), and the
            # t = 0 markers this node sent (module docstring).
            self.sim_params_takeoff: dict | None = None
            self.sim_param_events: list[dict] = []
            self.n_sim_param_events = 0
            self._run_nonce = str((self.scenario_meta or {}).get("run_nonce") or "")
            self.markers_sent: list[float] = []
            self._marker_times = list(MARKER_TIMES_S) if self._run_nonce else []

            self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                     self._on_pose, qos_profile_sensor_data)
            if TwistStamped is not None:
                self.create_subscription(TwistStamped,
                                         "/mavros/local_position/velocity_local",
                                         self._on_velocity, qos_profile_sensor_data)
            self.create_subscription(MavState, "/mavros/state", self._on_state, 10)
            self.create_subscription(ExtendedState, "/mavros/extended_state",
                                     self._on_ext, qos_profile_sensor_data)
            self.create_subscription(StatusText, "/mavros/statustext/recv",
                                     self._on_text, qos_profile_sensor_data)
            self.create_subscription(Float32MultiArray, "/vla/action_4d",
                                     self._on_action, 10)
            self.create_subscription(String, "/vla/identity", self._on_identity,
                                     LATCHED)
            self.create_subscription(String, TOPIC_STATUS, self._on_adapter,
                                     LATCHED)
            # What the adapter sent (SETPOINT DELIVERY).
            self.delivery = DeliveryMonitor()
            self.create_subscription(String, TOPIC_SENT, self._on_adapter_sent,
                                     SENT_QOS_DEPTH)
            self.pub = self.create_publisher(Float32MultiArray, TOPIC_SETPOINT, 10)
            self.pub_mode = self.create_publisher(String, TOPIC_MODE_REQUEST, 10)
            self.pub_fences = self.create_publisher(String, TOPIC_FENCES, LATCHED)
            self.pub_intercept = self.create_publisher(String, "/shield/intercept", 10)

            self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
            self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
            self.cli_tol = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
            self.cli_rate = self.create_client(StreamRate, "/mavros/set_stream_rate")
            self.cli_msg_interval = (
                self.create_client(MessageInterval, "/mavros/set_message_interval")
                if MessageInterval is not None else None)
            self.cli_param = self.create_client(GetParameters,
                                                "/mavros/param/get_parameters")
            self.cli_mavros_param = self.create_client(GetParameters,
                                                       "/mavros/get_parameters")
            self.cli_info = self.create_client(VehicleInfoGet,
                                               "/mavros/vehicle_info_get")
            self.cli_param_set = self.create_client(
                SetParameters, "/mavros/param/set_parameters")
            # The autopilot's home, for check_home (--start). The QoS the
            # adapter reads it with (sensor data), which MAVROS's publisher
            # serves; MAVROS re-sends it when asked (req_update).
            if HomePosition is not None:
                self.create_subscription(HomePosition, "/mavros/home_position/home",
                                         self._on_home, qos_profile_sensor_data)
            self.cli_home = (self.create_client(Trigger, "/mavros/home_position/req_update")
                             if Trigger is not None else None)
            # The GPS fix (fix_ok) and SITL's parameter changes, both with the
            # sensor-data QoS: best effort matches MAVROS's publishers
            # whatever their reliability. The t = 0 marker goes out RELIABLE,
            # which MAVROS's subscription accepts whatever its own. Each may be
            # refused by SROS 2 under a keystore made before sitl/
            # sros2_policy.xml granted it (2026-11-07): recorded, and the node
            # flies without it - fix_ok None as before, no marker (a stressed
            # cell then errors on the harness's check, never flies as scored).
            self.inputs_refused: dict[str, str] = {}
            self.pub_text = None
            try:
                if NavSatFix is not None:
                    self.create_subscription(NavSatFix, FIX_TOPIC, self._on_fix,
                                             qos_profile_sensor_data)
            except Exception as e:                               # noqa: BLE001
                self._input_refused("fix", e)
            try:
                # Every MAVLink message MAVROS hands the UAS (about 300 a
                # second on the rig, 1 in 30 an EKF_STATUS_REPORT); the
                # callback keeps EKF_STATUS_REPORT. Best effort, with a queue
                # deeper than the sensor-data profile's 5, so a tick that
                # holds the executor does not cost the report.
                if Mavlink is not None:
                    best_effort = getattr(ReliabilityPolicy, "BEST_EFFORT", None)
                    qos = (QoSProfile(depth=MAVLINK_SOURCE_DEPTH, reliability=best_effort)
                           if best_effort is not None else qos_profile_sensor_data)
                    self.create_subscription(Mavlink, MAVLINK_SOURCE_TOPIC,
                                             self._on_mavlink, qos)
            except Exception as e:                               # noqa: BLE001
                self._input_refused("estimator", e)
            try:
                if ParamEvent is not None:
                    self.create_subscription(ParamEvent, PARAM_EVENT_TOPIC,
                                             self._on_param_event, qos_profile_sensor_data)
            except Exception as e:                               # noqa: BLE001
                self._input_refused("param_event", e)
            try:
                if self._run_nonce:
                    self.pub_text = self.create_publisher(StatusText,
                                                          STATUSTEXT_SEND_TOPIC, 10)
            except Exception as e:                               # noqa: BLE001
                self._input_refused("marker", e)
            # The fences are published in bring_up(), once the autopilot's
            # fence parameters and the aircraft's position are known.

        # ---------------- subscriptions ---------------- #

        def _t(self) -> float | None:
            return None if self.mission_t0 is None else time.monotonic() - self.mission_t0

        def _shield_pose(self, st: State) -> State:
            """The pose as the Shield is told it (X-11): over a DEM
            (--ground a grid) the height above the DEM under the aircraft
            (terrain.TerrainReference: the pose's height above home, plus
            home's elevation, less the DEM's); otherwise `st` itself, the
            same object, as every flight before 2026-11-18."""
            return st if self.terrain is None else self.terrain.state(st)

        def _msl_of(self, st: State) -> float:
            """The MSL height of `st` as the Shield was told it (with
            --ground only): over a DEM its height above the DEM plus the
            DEM there; over a flat ground its height above home (= above that
            ground) plus the ground's elevation."""
            if self.terrain is not None:
                return self.terrain.msl(st.up, st.x, st.y)
            return float(st.up) + float(self.ground.elevation_at())

        def _on_pose(self, msg: PoseStamped) -> None:
            q = msg.pose.orientation
            # MAVROS ENU -> our up-positive frame (x = N, y = E), heading
            # clockwise from North. The adapter reads the same topic. The
            # pose is metres about the EKF origin, which SITL puts at home;
            # --start says where home is in the policy's frame (0, 0 when
            # not given: the frame origin IS home, as every run before
            # 2026-11-06 flew it).
            self.state = State(x=msg.pose.position.y + self.start_xy[0],
                               y=msg.pose.position.x + self.start_xy[1],
                               up=msg.pose.position.z,
                               yaw_deg=heading_deg_from_enu_quaternion(
                                   q.x, q.y, q.z, q.w))
            # Receipt on the node's own monotonic clock, not header.stamp:
            # MAVROS stamps through FCU timesync, which can be offset or jump.
            self.pose_watchdog.received(time.monotonic())
            if self.mission_t0 is not None:
                self.n_pose += 1

        def _on_velocity(self, msg) -> None:
            # The EKF's local velocity, for the own-velocity forecast and the
            # record (state_v); receipt on the node's monotonic clock.
            self.vel = velocity_from_enu(msg.twist.linear)
            self.vel_rx = time.monotonic()

        def _on_home(self, msg) -> None:
            # HomePosition: geo is WGS84; position is ENU metres from the EKF
            # origin (x East, y North) - kept as (north, east).
            first = self.home_geo is None
            self.home_geo = (float(msg.geo.latitude), float(msg.geo.longitude),
                             (float(msg.position.y), float(msg.position.x)))
            if first:
                self.rec.event("autopilot_home", t=self._t(), lat=self.home_geo[0],
                               lon=self.home_geo[1],
                               from_ekf_origin_ne=list(self.home_geo[2]))

        def _input_refused(self, what: str, e: Exception) -> None:
            self.inputs_refused[what] = f"{type(e).__name__}: {e}"
            self.get_logger().warn(f"{what} not available: {type(e).__name__}: {e}")

        def _on_fix(self, msg) -> None:
            # NavSatFix from the autopilot's GPS_RAW_INT (FIX_TOPIC); receipt
            # on the node's monotonic clock, as the pose's.
            try:
                self.fix_status = int(msg.status.status)
            except (AttributeError, TypeError, ValueError):
                return
            self.fix_rx = time.monotonic()
            self.n_fix += 1

        def fix_ok_now(self) -> bool | None:
            """State.fix_ok for this tick (fix_ok_from)."""
            age = None if self.fix_rx is None else time.monotonic() - self.fix_rx
            return fix_ok_from(self.fix_status, age)

        def _on_mavlink(self, msg) -> None:
            # MAVLINK_SOURCE_TOPIC: every message the autopilot sent; only
            # its EKF_STATUS_REPORT and (2026-11-14, THE RTL WATCH) its
            # BATTERY_STATUS are read (the id first: the rest return at
            # once). Receipt on the node's monotonic clock, as the pose's.
            try:
                mid = int(msg.msgid)
                if mid == EKF_STATUS_REPORT_ID:
                    fields, sink = ekf_status_from_mavlink(
                        msg.msgid, msg.sysid, msg.compid, msg.len,
                        msg.payload64), self.estimator
                elif mid == BATTERY_STATUS_ID:
                    fields, sink = battery_status_from_mavlink(
                        msg.msgid, msg.sysid, msg.compid, msg.len,
                        msg.payload64), self.battery
                else:
                    return
            except (AttributeError, TypeError, ValueError):
                return
            if fields is not None:
                sink.report(time.monotonic(), fields)

        def estimator_ok_now(self) -> bool | None:
            """State.estimator_ok for this tick (EstimatorHealth.verdict)."""
            return self.estimator.verdict(time.monotonic())

        def _on_param_event(self, msg) -> None:
            # A PARAM_VALUE the autopilot sent (MAVROS's pull at connect, or
            # the broadcast after any PARAM_SET, the simulator side's
            # included): only SITL's stressor parameters are kept, with the
            # mission time this node received it at (None before mission
            # start). The simulator side's alignment is measured here.
            pid = str(getattr(msg, "param_id", "") or "")
            if pid not in WATCHED_SIM_PARAMS:
                return
            try:
                val = param_value(msg.value)
            except (AttributeError, TypeError, ValueError):
                val = None
            self.n_sim_param_events += 1
            t = self._t()
            if len(self.sim_param_events) < SIM_PARAM_EVENTS_MAX:
                self.sim_param_events.append(
                    {"t": None if t is None else round(t, 4), "param": pid, "value": val})
            if t is not None:
                self.rec.event("sim_param", t=t, param=pid, value=val)
                self.get_logger().info(f"simulator parameter {pid} = {val} at t={t:.2f}s")

        def _send_marker(self, t: float) -> None:
            """The episode's t = 0 for the simulator side: a STATUSTEXT
            through MAVROS (sitl_stressors.marker_text), on a scenario
            mission only; the mission time it carries is this node's."""
            if self.pub_text is None:
                return
            try:
                self.pub_text.publish(StatusText(severity=STATUSTEXT_INFO,
                                                 text=marker_text(self._run_nonce, t)))
                self.markers_sent.append(round(float(t), 4))
            except Exception as e:                           # noqa: BLE001
                self.get_logger().warn(f"t=0 marker not sent: {type(e).__name__}: {e}")

        def _on_state(self, msg: MavState) -> None:
            self.mav_connected = msg.connected
            self.armed = bool(msg.armed)
            if msg.armed:
                self._flew_armed = True
            note_mavros_state(self.hil_evidence, msg.connected)
            if msg.mode != self.mode:
                self.rec.event("mode", t=self._t(), mode=msg.mode,
                               previous=self.mode, armed=bool(msg.armed))
                if msg.mode == "GUIDED":
                    self._asks.entered_guided()
                if (self.mission_t0 is not None and not self.done
                        and msg.mode != "GUIDED" and self.takeover is None
                        and (self.mode == "GUIDED" or self._fsm_asked(self.mode))
                        and not self._fsm_asked(msg.mode)):
                    # The autopilot left GUIDED - or a hold this node asked
                    # for - for a mode the node did not ask for: a GeoFence
                    # breach, a failsafe, or a pilot on the GCS. Until
                    # 2026-10-19 only a change out of GUIDED counted (the
                    # perception rail's rule until 2026-10-13), so a pilot
                    # taking the aircraft out of the node's hold (BRAKE ->
                    # LAND) went unseen, and when the actions resumed, or the
                    # FSM's G8 cleared its Loiter, the node sent GUIDED and
                    # pulled the aircraft out of the pilot's LAND.
                    self.takeover = {"t": self._t(), "mode": msg.mode,
                                     "previous": self.mode}
                    self.get_logger().warn(
                        f"autopilot took the aircraft: {self.mode} -> {msg.mode}")
            self.mode = msg.mode

        def _fsm_asked(self, mode: str) -> bool:
            """Did this node ask for the mode the autopilot just entered, since
            the aircraft last came back to GUIDED? A LOITER request sent through
            the adapter is flown as the adapter's hold mode (BRAKE by default,
            LOITER with its --hold-mode LOITER), so either counts for it
            (ModeRequests.explains)."""
            return self._asks.explains(mode)

        def _on_ext(self, msg: ExtendedState) -> None:
            self.landed = msg.landed_state == LANDED_STATE_ON_GROUND

        def _on_text(self, msg: StatusText) -> None:
            self.rec.event("autopilot_text", t=self._t(), severity=int(msg.severity),
                           text=msg.text)

        def _expects_extended(self) -> bool:
            """Has the VLA bridge shown itself - an extended message flown,
            or an identity announcing its action layout? From then on the
            grant's four values alone are a refusal (a second publisher)."""
            ident = self.vla_identity if isinstance(self.vla_identity, dict) else {}
            return self.n_pilot_msgs > 0 or ident.get("action_layout") is not None

        def _on_action(self, msg: Float32MultiArray) -> None:
            # The receipt: the callback's entry, on the node's monotonic clock
            # (e2e_ms and the camera's age start here).
            rx_ns = time.monotonic_ns()
            try:
                d = [float(v) for v in msg.data]
            except (TypeError, ValueError):
                d = []
            if len(d) < 4:
                err = f"{len(d)} values, fewer than the grant's four"
                pf = None
            elif not all(math.isfinite(v) for v in d[:4]):
                # Until the re-review of 23c88d3 (N8) dropped without a word.
                err, pf = "a non-finite action value", None
            else:
                # The VLA bridge's decision fields, if the message has them,
                # and every reason to refuse it (review of 339f23a, P1-A):
                # until then a broken tail - or a stub left running beside
                # the bridge - was flown as the stub's four values, counted
                # as a receipt, and released a decision hold.
                pf, err = pilot_fields(d)
            dims = getattr(getattr(msg, "layout", None), "dim", None) or []
            label = getattr(dims[0], "label", None) if dims else None
            ident = self.vla_identity if isinstance(self.vla_identity, dict) else {}
            if err is None and len(d) == len(VLA_ACTION_FIELDS) and label \
                    and label != VLA_LAYOUT_LABEL:
                err = f"layout label {label!r} is not {VLA_LAYOUT_LABEL!r}"
            if err is None and len(d) == len(VLA_ACTION_FIELDS) and not label \
                    and ident.get("action_layout") is not None:
                # The bridge announced its layout and labels every message
                # (N6): an unlabelled eight is not its message.
                err = "no layout label, where the VLA bridge announced one"
            if err is None and pf is None and self._expects_extended():
                err = "the grant's four values alone, where the VLA bridge sends eight"
            seq = pf["decision_seq"] if pf is not None else None
            if err is None and seq is not None and self._last_seq is not None \
                    and seq < self._last_seq:
                err = f"decision seq went backwards ({self._last_seq} -> {seq})"
            self.pilot_sent = [round(v, 4) if math.isfinite(v) else None for v in d[:4]]
            if err is not None:
                # Not flown, not a receipt: the decision watchdog holds now,
                # the receipt watchdog after its timeout if nothing valid
                # follows.
                self.n_pilot_malformed += 1
                self.pilot_error = self.pilot_refused = err
                if self.n_pilot_malformed <= REFUSAL_EVENTS_MAX:
                    self.rec.event("vla_action_refused", t=self._t(), why=err,
                                   values=[round(v, 4) if math.isfinite(v) else None
                                           for v in d[:len(VLA_ACTION_FIELDS) + 1]],
                                   n=self.n_pilot_malformed)
                return
            self.pilot_refused = None
            self.raw_body = Action4D(vx=d[0], vy=d[1], vz_up=d[2], yaw_rate=d[3])
            self.n_actions += 1
            self._action_rx_ns = rx_ns
            self._action_seq += 1
            self.watchdog.received(self._action_rx_ns / 1e9)
            self.pilot = pf
            self._pilot_rx = self._action_rx_ns / 1e9
            if pf is not None:
                self.n_pilot_msgs += 1
                if seq is not None:
                    self._last_seq = seq
            # THE TICK SCHEDULE: the fresh action is judged and flown now,
            # not at the next timer tick (None before the mission clock
            # starts, and with --tick-trigger timer).
            if self._sched is not None:
                trigger = self._sched.on_message(rx_ns / 1e9)
                if trigger is not None:
                    self._run_tick(trigger)

        def _on_identity(self, msg: String) -> None:
            try:
                ident = json.loads(msg.data)
            except ValueError:
                ident = None
            if not isinstance(ident, dict):
                # Not a JSON object: recorded and ignored. Until 2026-10-19 a
                # valid JSON non-object ('null', a list, a string) raised
                # AttributeError on .get() inside this callback - which ended
                # spin and the node, or, swallowed by a _call() wrapper, left
                # the non-dict here for _report to crash on, losing every
                # record of the flight. An identity accepted earlier is kept.
                self.get_logger().warn("/vla/identity is not a JSON object - ignored")
                self.rec.event("vla_identity_refused", t=self._t(),
                               data=str(msg.data)[:500])
                return
            self.vla_identity = ident
            self.rec.event("vla_identity", identity=ident)
            # A bridge that would read the contract differently (its layout,
            # its decision timeout): refused at the ready gate, recorded here.
            self.contract_problems = identity_contract_problems(
                ident, self.decision_watchdog.timeout_s)
            if self.contract_problems:
                self.get_logger().warn("VLA identity: " + "; ".join(self.contract_problems))
                self.rec.event("vla_contract_mismatch", problems=self.contract_problems)
            host = ident.get("host")
            # None stays None: a VLA node that does not say where it runs
            # cannot be shown to be on the Orin (manifest._vla_host_missing).
            self.hil_evidence["vla_host"] = host if isinstance(host, dict) else None
            vh = ident.get("policy_hash")
            with self._lock:            # not mid-way through a REST swap
                same = (not vh) or self.policy.matches_hash(vh)
            if not same:
                # In the ros2_ped_* runs the two nodes read different files and
                # nothing showed it (audit card WP1-23).
                self.get_logger().warn(
                    f"the VLA node flies policy {vh}, this node "
                    f"{self.policy.policy_hash}")
                self.rec.event("policy_mismatch", vla=vh,
                               shield=self.policy.policy_hash)

        def _on_adapter(self, msg: String) -> None:
            try:
                rec = json.loads(msg.data)
            except ValueError:
                return
            if not isinstance(rec, dict):     # as _on_identity: never raise here
                self.rec.event("adapter_status_refused", t=self._t(),
                               data=str(msg.data)[:500])
                return
            if rec.get("kind") == "fence_upload":
                self.fence_status.append(rec)
                with self._lock:
                    sent = self._fences_sent
                    if (rec.get("success") is True and sent is not None
                            and rec.get("generation") == sent["generation"]
                            and sorted(rec.get("fence_ids") or []) == sent["ids"]):
                        self.fences_enforced = {f["id"]: f for f in sent["fences"]}
            self.rec.event(f"adapter_{rec.get('kind', 'status')}", t=self._t(),
                           **{k: v for k, v in rec.items() if k not in ("kind", "t")})

        def _on_adapter_sent(self, msg: String) -> None:
            """/mavlink_adapter/sent: a setpoint the adapter handed to MAVROS,
            or its beat (SETPOINT DELIVERY). Counted, never raises."""
            self.delivery.report(parse_report(msg.data), time.monotonic())

        # ---------------- outputs ---------------- #

        def _publish_fences(self, reason: str) -> None:
            """This generation's keep-out zones to the adapter, under the lock
            the REST hot-apply thread also publishes them under."""
            with self._lock:
                self._publish_fences_locked(reason)

        def _publish_fences_locked(self, reason: str) -> None:
            """This generation's keep-out zones to the adapter, minus any the
            aircraft is in or near (`deferred_fences`, re-sent from _tick once
            clear). The column is the autopilot's own FENCE_ALT_MAX as read
            back at take-off; unknown, nothing is exported."""
            st = self.state
            # No pose yet means the aircraft is still on the pad, at home:
            # the policy-frame position --start gives it (the local origin
            # without one). "Unknown, so upload everything" armed a zone that
            # covers home under the aircraft on a 2026-10-07 flight: the
            # GeoFence held it on the pad and take-off never completed.
            if st is not None:
                pos, pos_src = (st.x, st.y), "pose"
            else:
                pos, pos_src = self.start_xy, "assumed home: no pose yet (on the pad)"
            fences, skipped = policy_fences(
                self.policy, column_m=self.fence_column_m, position=pos,
                margin_m=self.fence_margin_m, enforced=self.fences_enforced,
                **self._fence_ground_kw())
            self.deferred_fences = sorted(s["id"] for s in skipped
                                          if s.get("deferred"))
            # Zones over home (the local origin in SITL, --start's position
            # in the policy's frame): an RTL flies back into them. Recorded
            # and warned, not withheld.
            every, _ = policy_fences(self.policy, column_m=self.fence_column_m,
                                     **self._fence_ground_kw())
            over_home = zones_near(every, self.start_xy[0], self.start_xy[1],
                                   self.fence_margin_m)
            if over_home and over_home != self.zones_over_home:
                self.get_logger().warn(
                    f"zone(s) {', '.join(over_home)} cover home: an RTL lands "
                    f"in them and the GeoFence will report a breach there")
            self.zones_over_home = over_home
            # A geographic policy's frame origin (lat, lon), or None for local
            # metres about home; the adapter refuses one far from home.
            origin = getattr(self.policy, "frame_origin", None)
            doc = {"generation": self.policy.generation,
                   "policy_hash": self.policy.policy_hash,
                   "fences": fences, "skipped": skipped, "reason": reason,
                   "fence_alt_max_m": self.fence_column_m,
                   "position": [round(pos[0], 2), round(pos[1], 2)],
                   "position_source": pos_src,
                   "origin": (None if origin is None else
                              {"lat": origin[0], "lon": origin[1]}),
                   # Where this node flies the autopilot's home in the
                   # policy's frame (--start; 0, 0 without): the adapter
                   # projects the zones about home from there and refuses a
                   # home the frame origin puts elsewhere (fence_frame).
                   "home_policy_ne": [self.start_xy[0], self.start_xy[1]],
                   "backstop_active": self.autopilot_fence.get("backstop_active")}
            self._fences_sent = {"generation": doc["generation"],
                                 "ids": sorted(f["id"] for f in fences),
                                 "fences": fences}
            self.pub_fences.publish(String(data=json.dumps(doc)))
            self.rec.event("fences_published", t=self._t(), reason=reason,
                           generation=doc["generation"],
                           fence_ids=[f["id"] for f in fences], skipped=skipped,
                           deferred=self.deferred_fences,
                           zones_over_home=self.zones_over_home)

        def _recheck_deferred(self, st: State, unsafe_rules: list[str]) -> None:
            """Upload a held-back zone once the aircraft is clear of it: out
            of the zone and its margin, and the Shield no longer finds the
            position unsafe under that rule."""
            if not self.deferred_fences:
                return
            _, skipped = policy_fences(self.policy, column_m=self.fence_column_m,
                                       position=(st.x, st.y),
                                       margin_m=self.fence_margin_m,
                                       **self._fence_ground_kw())
            still = {s["id"] for s in skipped if s.get("deferred")}
            cleared = [i for i in self.deferred_fences
                       if i not in still and i not in unsafe_rules]
            if cleared:
                self._publish_fences(f"deferred zone(s) {', '.join(cleared)} "
                                     f"now clear")

        def _request_mode(self, mode: str, why: str) -> None:
            """Through the adapter (WP3-09); straight to MAVROS only when no
            adapter is listening, and the record says which."""
            via = "adapter"
            # The adapter by name, not any subscriber (adapter_listening).
            subs = [e.node_name for e in
                    self.get_subscriptions_info_by_topic(TOPIC_MODE_REQUEST)]
            to_adapter = adapter_listening(subs)
            ap = self._asks.ask(mode, via_adapter=to_adapter)
            if to_adapter:
                # The FSM's vocabulary: the adapter maps LOITER to its hold mode.
                self.pub_mode.publish(String(data=mode))
            elif self.cli_mode.service_is_ready():
                self.cli_mode.call_async(SetMode.Request(custom_mode=ap))
                via = (f"mavros (no adapter subscribed; others: {sorted(subs)}) "
                       f"as {ap}")
            else:
                via = "nobody: set_mode not ready"
            self.rec.event("mode_request", t=self._t(), mode=mode, autopilot_mode=ap,
                           why=why, via=via)
            self.get_logger().info(f"{mode} requested ({why}) via {via}")

        def _publish_body(self, a: Action4D) -> None:
            self.pub.publish(Float32MultiArray(
                data=[float(a.vx), float(a.vy), float(a.vz_up), float(a.yaw_rate)]))

        # ---------------- helpers ---------------- #

        def request_extended_state(self) -> bool | None:
            """Ask the autopilot for EXTENDED_SYS_STATE at EXT_STATE_RATE_HZ
            (MAV_CMD_SET_MESSAGE_INTERVAL through MAVROS's sys_status plugin)
            and record the answer in events.jsonl: True / False as the
            autopilot acknowledged, None when the service or the message type
            is missing. Best effort: without it "landed" is the disarm
            (landed_input)."""
            ok = None
            if self.cli_msg_interval is not None:
                res = self._call(self.cli_msg_interval, MessageInterval.Request(
                    message_id=MSG_EXTENDED_SYS_STATE,
                    message_rate=EXT_STATE_RATE_HZ))
                ok = None if res is None else bool(res.success)
            self.rec.event("extended_sys_state_requested", t=self._t(),
                           message_id=MSG_EXTENDED_SYS_STATE,
                           rate_hz=EXT_STATE_RATE_HZ, acknowledged=ok)
            self.get_logger().info(
                f"EXTENDED_SYS_STATE at {EXT_STATE_RATE_HZ} Hz requested: {ok}")
            return ok

        def landed_input(self) -> bool:
            """The FSM's "landed" (G10, X5): landed_from on this node's view."""
            return landed_from(self.mode, self.landed, self.armed, self._flew_armed)

        def _rtl_check(self, now: float, st: State, pose: State | None = None
                       ) -> "RTLCheck | None":
            """This tick's RTLCheck while the FSM is in RTL (THE RTL WATCH),
            None on every other tick (the watch reset). The zones and band are
            the Shield's hard, enforced P0 rules in force now (rtl_path.
            zones_from_shield / band_from_shield, as the harness reads them).
            Reading them is guarded: the watch never faults the FSM - an
            error is recorded and the leg goes unjudged on that tick.

            `pose`: the pose above HOME when `st` is not (a --ground grid:
            `st` holds the height above the DEM, X-11). RTL_ALT is measured
            above home, so the watch is handed both (RTLFailureWatch.assess,
            up_home); the zones and band stay in the Shield's reference."""
            if self.fsm is None or self.fsm.terminal or self.fsm.state is not FSMState.RTL:
                self.rtl_watch.reset()
                return None
            over = None
            try:
                if self.terrain is None:
                    zones, band = (zones_from_shield(self.shield),
                                   band_from_shield(self.shield, st))
                else:
                    # Over a --ground grid (X-11) the leg home is judged
                    # along the DEM under it, each rule in its own reference
                    # (rtl_path, THE LEG OVER A DEM).
                    zones, band, over = leg_rules_from_shield(self.shield, st)
            except Exception as e:                               # noqa: BLE001
                zones = band = over = None
                self.rtl_watch.note_error(now, e)
            chk = self.rtl_watch.assess(
                now, st.x, st.y, st.up, self.mode, zones, band,
                self.battery.verdict(time.monotonic()),
                up_home=(pose.up if pose is not None and pose is not st else None),
                **({} if over is None else {"over": over}))
            if chk.failed:
                self.rec.event("rtl_failed", t=now, kinds=list(chk.kinds),
                               reasons=list(chk.reasons), mode=self.mode)
            return chk

        def _call(self, client, req, timeout: float = 5.0):
            if not client.wait_for_service(timeout_sec=timeout):
                return None
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            return fut.result()

        # The MAVROS chain's evidence: the readers are sitl/mavros_evidence.py's,
        # shared with the mission-style node; these wrappers hand them this
        # node's service calls, its spin and its clock.

        def _get_params(self, client, names: list[str]) -> dict:
            return get_params(self._call, client, GetParameters.Request, names)

        def _spin_evidence(self) -> None:
            rclpy.spin_once(self, timeout_sec=0.5)

        def read_sim_speedup(self) -> float | None:
            """SIM_SPEEDUP from the autopilot, or None - and whether the
            autopilot is SITL at all (mavros_evidence.poll_sim_speedup).
            Never assumed."""
            return poll_sim_speedup(
                lambda names: self._get_params(self.cli_param, names),
                self._spin_evidence, self.hil_evidence, now=time.time,
                warn=self.get_logger().warn)

        def wait_sim_speedup(self) -> dict:
            """THE TAKE-OFF WAIT FOR SIM_SPEEDUP: one bounded poll
            (takeoff_speedup, SIM_SPEEDUP_WAIT_S) before the fence read and
            the stressor read, which both take its answer; recorded as the
            `sim_speedup_wait` event. Never raises."""
            try:
                ans = takeoff_speedup(
                    lambda names: self._get_params(self.cli_param, names),
                    self._spin_evidence, now=time.time, warn=self.get_logger().warn)
            except Exception as e:                               # noqa: BLE001
                ans = {"sim_speedup": None, "autopilot_kind": None, "waited_s": None,
                       "wait_s": SIM_SPEEDUP_WAIT_S,
                       "error": f"{type(e).__name__}: {e}"}
            self.takeoff_speedup = ans
            self.rec.event("sim_speedup_wait", t=None, **ans)
            self.get_logger().info(
                f"SIM_SPEEDUP at take-off: {ans.get('sim_speedup')} after "
                f"{ans.get('waited_s')} s ({ans.get('autopilot_kind') or 'unknown kind'})")
            return ans

        def read_autopilot_version(self) -> None:
            """AUTOPILOT_VERSION as MAVROS received it (ARCH-31); the setup
            script's banner only as a labelled fallback
            (mavros_evidence.poll_autopilot_version)."""
            poll_autopilot_version(
                lambda: self._call(self.cli_info, VehicleInfoGet.Request(
                    sysid=0, compid=0, get_all=False), 5.0),
                self._spin_evidence, self.hil_evidence, now=time.time,
                warn=self.get_logger().warn)

        def read_link(self) -> None:
            """The MAVLink link MAVROS flies over: its own fcu_url parameter,
            else the URL the launch script started it with (labelled); and
            which MAVLink actors run on this host (mavros_evidence.record_link)."""
            record_link(self._get_params(self.cli_mavros_param, ["fcu_url"]).get("fcu_url"),
                        self.hil_evidence)

        def _set_param(self, name: str, value: float) -> bool:
            """One autopilot parameter through MAVROS 2 (/mavros/param is a
            node whose parameters are the autopilot's)."""
            p = Parameter(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))
            try:
                res = self._call(self.cli_param_set,
                                 SetParameters.Request(parameters=[p]), 5.0)
            except Exception:                                 # noqa: BLE001
                return False
            return bool(res and res.results and res.results[0].successful)

        def _fence_ground_kw(self) -> dict:
            """What policy_ceiling_m needs to give FENCE_ALT_MAX's ceiling
            above HOME (X-11): the --ground and home's elevation (over a DEM
            --ground-home-msl as TerrainReference checked it; over a flat
            ground its one elevation). {} without --ground: the call every
            flight before 2026-11-18 made."""
            if self.ground is None:
                return {}
            home = (self.terrain.home_msl_m if self.terrain is not None
                    else float(self.ground.elevation_at()))
            return {"ground": self.ground, "home_msl_m": home}

        def read_autopilot_fence(self) -> None:
            """The GeoFence parameters as the AUTOPILOT holds them, not as the
            .parm file says (GUARDRAIL_FENCE=0 starts SITL without it, and the
            node flies any --policy). FENCE_ALT_MAX is raised above the
            policy's ceiling when it sits below it, so the backstop is never
            stricter than the policy; if that fails the record says so."""
            vals: dict = {}
            deadline = time.time() + 20.0
            fence_since: float | None = None
            while time.time() < deadline:
                vals = self._get_params(self.cli_param,
                                        list(FENCE_PARAMS) + ["SIM_SPEEDUP"])
                # ALL of them: MAVROS fills /mavros/param while it pulls the
                # parameter list, and a 2026-10-07 flight read FENCE_ENABLE
                # before FENCE_ALT_MAX existed, so no zone was exported. And
                # SIM_SPEEDUP with them (fence_read_done, 2026-11-11): read
                # without it, a SITL autopilot looked like hardware and its
                # fence was left below the policy's ceiling. Since THE
                # TAKE-OFF WAIT FOR SIM_SPEEDUP runs before this read, its
                # answer stands in for the grace (speedup_polled).
                polled = self.takeoff_speedup is not None
                if all(vals.get(k) is not None for k in FENCE_PARAMS):
                    if fence_since is None:
                        fence_since = time.time()
                    if fence_read_done(vals, time.time() - fence_since,
                                       speedup_polled=polled):
                        break
                rclpy.spin_once(self, timeout_sec=0.5)
            ceiling, ceiling_src = policy_ceiling_m(self.policy,
                                                    self.mission.cruise_alt_m,
                                                    **self._fence_ground_kw())
            alt_max = vals.get("FENCE_ALT_MAX")
            plan = fence_alt_plan(ceiling, alt_max)
            raised_from = None
            # Only SITL's parameters are changed by this node. On a hardware
            # autopilot the grant has the GeoFence parameters "GCS-set", and a
            # change would persist in its EEPROM: there a fence below the
            # policy's ceiling is recorded (stricter_than_policy), not fixed.
            # SITL: this read's SIM_SPEEDUP, or THE TAKE-OFF WAIT's (one
            # answer for both reads; none after the wait is not SITL).
            waited = (self.takeoff_speedup or {}).get("sim_speedup")
            is_sitl = speedup_is_sitl(vals.get("SIM_SPEEDUP"), waited)
            # A GCS-set fence (gcs_backstop.json in the run folder, written by
            # the ground station's own process before this node started): the
            # GCS owns the parameters, so this node changes none of them.
            gcs = gcs_backstop_record(self.out, vals)
            raise_ok = is_sitl and gcs is None
            if plan["raise_to"] is not None and alt_max is not None and raise_ok:
                if self._set_param("FENCE_ALT_MAX", plan["raise_to"]):
                    raised_from = alt_max
                    alt_max = self._get_params(
                        self.cli_param, ["FENCE_ALT_MAX"]).get("FENCE_ALT_MAX")
            enabled = vals.get("FENCE_ENABLE")
            self.autopilot_fence = {
                **{k: vals.get(k) for k in FENCE_PARAMS},
                "FENCE_ALT_MAX": alt_max,
                "source": ("/mavros/param (read from the autopilot)"
                           if enabled is not None else None),
                "parm_file": {k: v for k, v in read_parm(FENCE_PARM).items()
                              if k in FENCE_PARAMS},
                "policy_ceiling_m": ceiling, "policy_ceiling_source": ceiling_src,
                "fence_alt_max_raised_from": raised_from,
                "raise_allowed": raise_ok,
                # Whether the autopilot is SITL, and the take-off wait's
                # answer it was judged with besides this read's own.
                "sitl": is_sitl,
                "sim_speedup_wait": self.takeoff_speedup,
                "set_by": ("gcs" if gcs is not None else
                           "sitl defaults file (start_sitl.sh GUARDRAIL_FENCE) or "
                           "this node"),
                "gcs": gcs,
                "backstop_active": (None if enabled is None else enabled >= 1.0),
                "stricter_than_policy": (
                    None if alt_max is None or ceiling is None
                    else not fence_alt_plan(ceiling, alt_max)["ok"]),
                "plan_why": plan["why"],
            }
            self.hil_evidence["autopilot_fence"] = self.autopilot_fence
            self.fence_column_m = alt_max
            if vals.get("FENCE_MARGIN") is not None:
                self.fence_margin_m = float(vals["FENCE_MARGIN"])
            self.rec.event("autopilot_fence", **self.autopilot_fence)
            self.get_logger().info(
                f"autopilot fence: ENABLE {enabled}, ALT_MAX {alt_max}"
                + (f" (raised from {raised_from})" if raised_from is not None else "")
                + f", AVOID_ENABLE {vals.get('AVOID_ENABLE')}, policy ceiling "
                f"{ceiling} m")
            if self.autopilot_fence["backstop_active"] is False:
                self.get_logger().warn("FENCE_ENABLE is 0: no GeoFence backstop "
                                       "behind the Shield on this run")
            if self.autopilot_fence["stricter_than_policy"]:
                self.get_logger().warn(f"backstop stricter than the policy: "
                                       f"{plan['why']}")

        def read_sim_params(self) -> None:
            """SITL's stressor parameters (sitl_stressors.WATCHED_SIM_PARAMS)
            as the autopilot holds them before take-off, through MAVROS
            (metrics.json `sim_params.at_takeoff`): the boot values the
            simulator side set, read back by a second process on a second
            link. Only on SITL (SIM_SPEEDUP answers); a hardware autopilot has
            no SIM_* parameter, and the record says so.

            SIM_SPEEDUP is THE TAKE-OFF WAIT's answer (wait_sim_speedup, run
            first if it has not been; sim_params_record, 2026-11-11: one read
            used to decide, and 8 of the 48 missions of sm1111 read it before
            MAVROS had it); then the values are accepted once two reads in a
            row agree with every value present (a forced pull once read 0.0
            mid-refill: mavros_evidence.poll_sim_speedup), for up to
            SIM_PARAMS_SETTLE_S. Evidence only, and fail-closed: a read that
            fails, or a SIM_SPEEDUP that never answered, is recorded with why
            (`values` None) and bring-up goes on - the harness then refuses a
            stressed cell (experiments/sweep_scenarios.py rail_stress_check)."""
            if self.takeoff_speedup is None:
                self.wait_sim_speedup()
            try:
                self.sim_params_takeoff = sim_params_record(
                    lambda names: self._get_params(self.cli_param, names),
                    lambda: rclpy.spin_once(self, timeout_sec=0.2),
                    self.takeoff_speedup, now=time.time)
            except Exception as e:                               # noqa: BLE001
                self.sim_params_takeoff = {"sitl": None, "values": None,
                                           "why": f"not read: {type(e).__name__}: {e}"}
            self.rec.event("sim_params", **self.sim_params_takeoff)
            vals = self.sim_params_takeoff.get("values")
            if vals is None:
                self.get_logger().warn("simulator parameters not read: "
                                       + str(self.sim_params_takeoff.get("why")))
                return
            self.get_logger().info(
                f"simulator parameters at take-off (SIM_SPEEDUP after "
                f"{self.sim_params_takeoff['speedup_wait_s']} s): " + ", ".join(
                    f"{k}={vals.get(k)}" for k in WATCHED_SIM_PARAMS))

        def read_ekf_failsafe(self) -> None:
            """FS_EKF_THRESH and FS_EKF_ACTION as the autopilot holds them
            (2026-11-12): the threshold the estimator check uses
            (EstimatorHealth.set_thresh; ArduCopter's default when unread),
            and the action its own EKF failsafe takes, recorded. Read after
            the fence read, by which time MAVROS holds the parameter list."""
            try:
                vals = self._get_params(self.cli_param, ["FS_EKF_THRESH", "FS_EKF_ACTION"])
            except Exception as e:                               # noqa: BLE001
                vals = {}
                self.get_logger().warn(f"FS_EKF_* not read: {type(e).__name__}: {e}")
            self.estimator.set_thresh(vals.get("FS_EKF_THRESH"))
            self.rec.event("ekf_failsafe", t=None, FS_EKF_THRESH=vals.get("FS_EKF_THRESH"),
                           FS_EKF_ACTION=vals.get("FS_EKF_ACTION"),
                           thresh=self.estimator.thresh,
                           thresh_source=self.estimator.thresh_source)

        def read_rtl_params(self) -> None:
            """RTL_WATCH_PARAMS as the autopilot holds them (2026-11-14, THE
            RTL WATCH): RTL_ALT and RTL_CONE_SLOPE configure the watch's
            climb, leg altitude and home radius (rtl_watch_config;
            ArduCopter's defaults when unread), and the BATT_* are recorded
            with what they mean for the battery reason. Read after the
            EKF failsafe's, by which time MAVROS holds the parameter list."""
            try:
                vals = self._get_params(self.cli_param, list(RTL_WATCH_PARAMS))
            except Exception as e:                               # noqa: BLE001
                vals = {}
                self.get_logger().warn(f"RTL_* / BATT_* not read: {type(e).__name__}: {e}")
            alt_m, slope, source = rtl_watch_config(vals)
            self.rtl_watch.configure(alt_m, slope, source)
            self.rtl_params = {k: vals.get(k) for k in RTL_WATCH_PARAMS}
            self.rec.event("rtl_watch", t=None, **self.rtl_params,
                           rtl_alt_m=self.rtl_watch.rtl_alt_m,
                           cone_slope=self.rtl_watch.cone_slope,
                           home_radius_m=self.rtl_watch.home_radius_m, source=source,
                           battery=battery_failsafe_note(vals))
            self.get_logger().info(
                f"RTL watch: return altitude {self.rtl_watch.rtl_alt_m:g} m, home radius "
                f"{self.rtl_watch.home_radius_m:g} m ({source}); "
                f"{battery_failsafe_note(vals)}")

        def bring_up(self) -> None:
            log = self.get_logger()
            self.serve_hot_apply()
            t_end = time.time() + BRING_UP_S
            while time.time() < t_end and not self.mav_connected:
                rclpy.spin_once(self, timeout_sec=0.5)
            if not self.mav_connected:
                raise RuntimeError("mavros never connected to FCU")
            log.info("FCU connected")
            self.read_autopilot_version()
            log.info(f"autopilot: {self.hil_evidence.get('ardupilot_version')} "
                     f"({self.hil_evidence.get('ardupilot_version_source')})")
            self.read_link()

            # ArduPilot streams nothing until asked.
            self._call(self.cli_rate,
                       StreamRate.Request(stream_id=0, message_rate=10, on_off=True))
            log.info("stream rate 10 Hz requested")
            # ...and EXTENDED_SYS_STATE by id: no stream group carries it on
            # ArduCopter, and without it the FSM never learns "landed".
            self.request_extended_state()

            # THE TAKE-OFF WAIT FOR SIM_SPEEDUP (2026-11-11): one bounded poll
            # whose answer both the fence read and the stressor read below
            # take - what tells SITL (whose fence may be raised) from
            # hardware. Without an answer by its end both fail closed.
            self.wait_sim_speedup()
            # The backstop as the autopilot holds it, then this generation's
            # zones - once the aircraft's position is known, so a zone over
            # the pad is held back rather than armed under the aircraft.
            self.read_autopilot_fence()
            # A take-off the backstop would end is refused here, before
            # arming (takeoff_fence_refusal, 2026-11-11), with the reason in
            # events.jsonl; until then the node armed and climbed into the
            # autopilot's own fence.
            refusal = takeoff_fence_refusal(
                self.start_up if self.start_up is not None else self.mission.cruise_alt_m,
                self.fence_column_m, self.fence_margin_m,
                (self.autopilot_fence or {}).get("backstop_active"))
            if refusal:
                self.rec.event("takeoff_refused", t=None, reason=refusal,
                               autopilot_fence=self.autopilot_fence)
                raise RuntimeError(refusal)
            # SITL's stressor parameters as they stand before take-off (the
            # simulator side set its boot values before this node started).
            self.read_sim_params()
            # The autopilot's own EKF failsafe threshold, for the estimator
            # check behind State.estimator_ok (2026-11-12).
            self.read_ekf_failsafe()
            # The return the RTL watch judges, and the battery failsafe it
            # reads (2026-11-14).
            self.read_rtl_params()
            pose_deadline = time.time() + 10.0
            while self.state is None and time.time() < pose_deadline:
                rclpy.spin_once(self, timeout_sec=0.5)
            self._publish_fences("take-off")

            # Give the adapter a moment to upload the fences before arming, so
            # the autopilot's backstop is in place from take-off.
            fence_deadline = time.time() + 15.0
            while time.time() < fence_deadline and not self.fence_status:
                rclpy.spin_once(self, timeout_sec=0.5)
            log.info("autopilot fence: " + (json.dumps(self.fence_status[-1])
                                            if self.fence_status else
                                            "no upload reported by the adapter"))

            # A scenario's start (--start): refused on the pad unless the
            # autopilot's home is where the scenario starts.
            self.check_home()

            # A VLA that must warm up first (--vla-ready-timeout-s): refused
            # here, before GUIDED and arming, never in the air. The wait is
            # the VLA's, not the autopilot's: GUIDED, arming and the climb get
            # their whole budget after it (review of 339f23a, P1-B: a 900 s
            # cold load spent the shared 180 s, and arming "timed out (EKF)"
            # without one attempt).
            if self.wait_for_vla():
                t_end = time.time() + BRING_UP_S

            while time.time() < t_end:
                r = self._call(self.cli_mode, SetMode.Request(custom_mode="GUIDED"))
                if r and r.mode_sent:
                    break
                time.sleep(1)
            log.info("GUIDED requested")

            attempts = 0
            while time.time() < t_end:
                attempts += 1
                r = self._call(self.cli_arm, CommandBool.Request(value=True))
                if r and r.success:
                    log.info("armed")
                    break
                time.sleep(2)
                rclpy.spin_once(self, timeout_sec=0.1)
            else:
                raise RuntimeError(
                    f"arming refused {attempts} time(s) within the {BRING_UP_S:g} s bring-up "
                    f"budget (the autopilot's pre-arm checks did not pass)" if attempts else
                    f"arming never tried: the {BRING_UP_S:g} s bring-up budget ran out first")

            # The scenario's start altitude (--start X,Y,UP), else the
            # mission's cruise altitude, as before.
            alt = self.start_up if self.start_up is not None else self.mission.cruise_alt_m
            while time.time() < t_end:
                r = self._call(self.cli_tol, CommandTOL.Request(altitude=float(alt)))
                if r and r.success:
                    log.info(f"takeoff accepted -> {alt} m")
                    break
                time.sleep(2)
            else:
                raise RuntimeError("takeoff never accepted")

            settled_since: float | None = None
            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.5)
                if self.start_up is None:
                    ready = bool(self.state and self.state.up >= alt - 1.0)
                else:
                    # Settled at the start altitude: within START_UP_TOL_M
                    # for START_SETTLE_S, so the climb's overshoot is not
                    # where the scenario starts.
                    near = (self.state is not None
                            and abs(self.state.up - alt) <= START_UP_TOL_M)
                    if not near:
                        settled_since = None
                    elif settled_since is None:
                        settled_since = time.time()
                    ready = near and time.time() - settled_since >= START_SETTLE_S
                if ready:
                    # ArduPilot may set home only when the EKF is ready (at
                    # arming), and the adapter uploads the fence once home is
                    # known: give it a bounded wait before the mission.
                    wait_end = time.time() + 10.0
                    while not self.fence_status and time.time() < wait_end:
                        rclpy.spin_once(self, timeout_sec=0.5)
                    # FENCE_ALT_MAX still unread (MAVROS slow to fill its
                    # parameters): read again now, and re-send the zones,
                    # which were all withheld without a column.
                    if self.fence_column_m is None:
                        self.read_autopilot_fence()
                        if self.fence_column_m is not None:
                            self._publish_fences("fence parameters read late")
                            late_end = time.time() + 5.0
                            while time.time() < late_end:
                                rclpy.spin_once(self, timeout_sec=0.5)
                    self.rec.event("fence_before_mission",
                                   uploaded=bool(self.fence_status and
                                                 self.fence_status[-1].get("success")),
                                   fence_alt_max_m=self.fence_column_m)
                    log.info(f"at {self.state.up:.1f} m - mission start")
                    # Read here, not in _finish(): a service call from a timer
                    # callback is a nested spin and never completes.
                    self.sim_speedup = self.read_sim_speedup()
                    log.info(f"SIM_SPEEDUP read from autopilot: {self.sim_speedup}")
                    return
            raise RuntimeError("never reached takeoff altitude")

        def check_home(self) -> None:
            """--start X,Y (2026-11-06): refuse to arm - RuntimeError, before
            GUIDED - unless the autopilot's home, projected about the
            policy's frame origin, lies within HOME_TOL_M of (X, Y)
            (home_check). This is what catches a SITL booted at another home
            than the scenario starts from: every pose is offset by --start,
            so such a flight would put the Shield's zones where they are not.
            MAVROS publishes the home when ArduPilot sends it; the node asks
            for it (/mavros/home_position/req_update) every
            HOME_REQUEST_EVERY_S, up to HOME_WAIT_S. Without --start nothing
            is checked, as before (the adapter still refuses a fence upload
            whose frame origin is far from home). The verdict is an event
            (`home_check`) and metrics.json `home_check`."""
            if not self.start_given:
                return
            origin = getattr(self.policy, "frame_origin", None)
            t0 = time.time()
            last_req = None
            while origin is not None and self.home_geo is None \
                    and time.time() - t0 < HOME_WAIT_S:
                if (self.cli_home is not None and self.cli_home.service_is_ready()
                        and (last_req is None
                             or time.time() - last_req >= HOME_REQUEST_EVERY_S)):
                    self.cli_home.call_async(Trigger.Request())
                    self.n_home_requests += 1
                    last_req = time.time()
                rclpy.spin_once(self, timeout_sec=0.5)
            self.home_check = dict(home_check(self.home_geo, origin, self.start_xy),
                                   waited_s=round(time.time() - t0, 1),
                                   home_requests=self.n_home_requests)
            self.rec.event("home_check", **self.home_check)
            if self.home_check["ok"]:
                self.get_logger().info(
                    "home check: " + ("not checkable, metre policy"
                                      if not self.home_check["checked"] else
                                      f"home {self.home_check['miss_m']} m from --start "
                                      f"{list(self.start_xy)}"))
                return
            msg = f"refusing to arm: {self.home_check['why']}"
            self.get_logger().error(msg)
            raise RuntimeError(msg)

        def wait_for_vla(self) -> bool:
            """--vla-ready-timeout-s S (2026-10-23): wait up to S for a
            /vla/identity whose `ready` is true (the VLA bridge's warm-up gate
            passed; vla_ready) and that reads the contract as this node does
            (identity_contract_problems: its layout, its decision timeout),
            and refuse to arm - RuntimeError, before GUIDED - otherwise, or on
            the timeout. The wait and its verdict are an event (`vla_ready`)
            and metrics.json `vla_ready`. True when it waited (bring_up then
            gives arming its whole budget); 0 (the default) does not wait, as
            before: the stub needs none."""
            timeout = float(getattr(self.args, "vla_ready_timeout_s", 0.0) or 0.0)
            if timeout <= 0.0:
                return False
            log = self.get_logger()
            log.info(f"waiting up to {timeout:g} s for a ready VLA on /vla/identity")
            t0 = time.time()
            while True:
                ready, why = vla_ready(self.vla_identity)
                if ready is not None or time.time() - t0 >= timeout:
                    break
                rclpy.spin_once(self, timeout_sec=0.5)
            problems = identity_contract_problems(self.vla_identity,
                                                  self.decision_watchdog.timeout_s)
            if ready is True and problems:
                ready, why = False, "; ".join(problems)
            self.vla_ready = {"timeout_s": timeout, "waited_s": round(time.time() - t0, 1),
                              "ready": ready, "why": why}
            self.rec.event("vla_ready", **self.vla_ready)
            if ready is True:
                log.info(f"VLA ready: {why}")
                return True
            msg = (f"the VLA node is not ready, refusing to arm: {why}" if ready is False
                   else f"no ready VLA on /vla/identity within {timeout:g} s: refusing to arm")
            log.error(msg)
            raise RuntimeError(msg)

        # ---------------- mission tick ---------------- #

        def start_mission(self) -> None:
            # The first mission tick took 116 ms on a 2026-10-07 flight (33 ms
            # on the pymavlink rail; every later tick <= 3.2 ms), over the
            # grant's 100 ms tick budget. One Shield call before the mission,
            # on a throwaway Shield (the real one's history stays the
            # flight's), and its time recorded: on the two flights since, the
            # first mission tick took 1.2 ms. The spike's cause is not
            # established; the record keeps it visible if it recurs.
            if self.state is not None:
                warm = rail_shield(self.policy, lookahead_s=LOOKAHEAD_S,
                                   dt=LOOKAHEAD_DT_S, clock=self.clock, **self.world_kw,
                                   **self.ground_kw)
                warm.filter_body(self._shield_pose(self.state), self.raw_body)
                self.rec.event("shield_warmup", t=None,
                               ms=round(warm.history[-1].elapsed_ms, 3))
            # THE GARBAGE COLLECTOR: everything alive now is collected once
            # and frozen, before the mission clock starts (a full walk of the
            # whole heap, which must not land in a tick); its mission-time
            # schedule counts from t = 0.
            self.gc_pacer.start(0.0)
            self.rec.event("gc_pacing", t=None, pace=self.gc_pacer.pace,
                           start_collect_ms=self.gc_pacer.start_collect_ms,
                           frozen=self.gc_pacer.frozen_at_start,
                           every_s=self.gc_pacer.every_s)
            if self._sched is not None:
                # THE START-UP TICK: what queued while this node did not
                # spin is delivered now, held (never ticked, not counted by
                # the rate limit), not as a burst after t = 0.
                self.start_backlog = self._deliver_backlog()
            self.mission_t0 = time.monotonic()
            # The episode's t = 0 for the simulator side (a scenario mission
            # only), sent the moment the mission clock starts.
            if self._marker_times and self._marker_times[0] <= 0.0:
                self._marker_times.pop(0)
                self._send_marker(0.0)
            self.watchdog.start(self.mission_t0)
            self.pose_watchdog.start(self.mission_t0)
            # The heading the scenario's first events see (spawn ahead, move
            # left): start to target, else the start yaw (SITL boots facing
            # the scenario's start yaw), as the stress harness does; then the
            # measured velocity's direction once the aircraft moves.
            here = (self.start_xy if self.start_given else
                    (self.state.x, self.state.y) if self.state is not None else (0.0, 0.0))
            self._ev_heading = SE.start_heading(
                here, ((self.mission.target_x, self.mission.target_y)
                       if self.has_target else None),
                self.state.yaw_deg if self.state is not None else 0.0)
            self._ev_heading_source = ("start to target" if self.has_target
                                       else "start yaw")
            self.rec.event("mission_start", t=0.0,
                           target=([self.mission.target_x, self.mission.target_y]
                                   if self.has_target else None),
                           speed_pref_mps=self.mission.speed_pref_mps,
                           shield=self.args.shield,
                           rule_clock_reading=self.clock().isoformat(
                               timespec="seconds"),
                           action_timeout_s=self.watchdog.timeout_s,
                           pose_timeout_s=self.pose_watchdog.timeout_s,
                           start=(None if not self.start_given else
                                  [self.start_xy[0], self.start_xy[1], self.start_up]),
                           reach_m=self.reach_m if self.has_target else None,
                           scenario=((self.scenario_meta or {}).get("run_id")
                                     if self.scenario_meta else None),
                           scenario_events_declared=self._ev_declared,
                           tick_trigger=self.tick_trigger,
                           start_backlog=self.start_backlog)
            if self.tick_trigger == "timer":
                # The node's own 10 Hz timer, as every run until 2026-11-11.
                self.create_timer(TICK, self._tick)
                return
            # THE TICK SCHEDULE: a pilot message ticks (_on_action), and this
            # poll keeps the 10 Hz watchdog for a quiet stream.
            self._sched.start(time.monotonic())
            self.create_timer(TICK_POLL_S, self._on_tick_poll)

        def _deliver_backlog(self) -> dict:
            """THE START-UP TICK: deliver what queued while this node did not
            spin - bring-up's last service call, the Shield warm-up, the
            collector's start - before the mission clock starts, so a pilot
            message published meanwhile is received (and held) before t = 0
            instead of arriving as a burst after it. Spins of no wait, at most
            START_BACKLOG_SPINS of them and START_BACKLOG_S in all: a stream
            that never runs dry (MAVROS's MAVLink source, ~300 a second) ends
            it by the time bound. Returns its record for the mission_start
            event and metrics.json `tick_schedule.start.backlog`."""
            t_start = time.monotonic()
            n0 = self.n_actions
            spins = 0
            while spins < START_BACKLOG_SPINS:
                rclpy.spin_once(self, timeout_sec=0.0)
                spins += 1
                if time.monotonic() - t_start >= START_BACKLOG_S:
                    break
            return {"spins": spins, "ms": round((time.monotonic() - t_start) * 1000.0, 3),
                    "actions": self.n_actions - n0,
                    "max_spins": START_BACKLOG_SPINS, "max_s": START_BACKLOG_S}

        def serve_hot_apply(self) -> None:
            """The REST hot-apply API on this Shield (--hotapply-port), started
            on the pad by bring_up(): a listener that cannot start (FastAPI
            missing from the venv, the port taken, the PKI absent) stops the
            run before arming, never mid-air. Events before mission start
            change the mission-start policy (Shield.hot_apply)."""
            port = getattr(self.args, "hotapply_port", None)
            if not port:
                return
            from guardrail.api import FlightHotApply
            self.hot = FlightHotApply(self.shield, self.rec, self._lock,
                                      mission_t=self._t,
                                      after=self._after_hot_apply,
                                      log_fn=self.get_logger().info)
            th = self.hot.start(self.args.hotapply_host, int(port),
                                pki_dir=self.args.hotapply_pki,
                                server_name=self.args.hotapply_server_cert,
                                allowed_clients=self.args.hotapply_client)
            self.get_logger().info(f"hot-apply API at {th.url}")

        def _after_hot_apply(self, result: dict, t: float | None) -> None:
            """A REST hot-apply was accepted (under the tick's lock): the new
            generation's zones go to the adapter, a zone the aircraft is in
            or near held back as for every other generation."""
            self._publish_fences(f"hot_apply {result['applied']} "
                                 f"({result['op']} {result['type']}) over REST")

        def _prev_judgement(self, st: State, changing: bool):
            """THIS tick's position under the PREVIOUS tick's rules, zone
            positions and rule-clock reading: (unsafe rule ids, ids of the
            zones whose polygon holds it), or None on the first tick. The
            stress harness's breach and polygon-entry tests (it judges the
            new position under the rules of the instant before; sweep_
            scenarios.rail_measurements reads these), so a zone spawned onto
            the aircraft, a window opening over it or a zone moving onto it
            is not counted as a step the aircraft made.

            Called first in the tick, before any event of this tick and
            before filter(): the Shield then still holds the previous
            generation and places moving zones at its last tick, and the
            rule clock is pinned to the previous row's reading for the one
            query (under the tick's lock, so the REST thread never sees the
            pin). Only computed where it can differ from this tick's own
            judgement - an event due, a rule that switches with the clock, a
            moving zone - else "same", and the row copies its own."""
            if self._last_clock is None:
                return None
            if not (changing or SE.rules_switch_with_clock(self.policy)
                    or SE.has_moving_zone(self.policy)):
                return "same"
            self._clock_pin = self._last_clock
            try:
                unsafe = sorted({v.rule_id for v in self.shield.state_is_unsafe(st)})
                zin, _ = zones_at(self.shield, st)
            finally:
                self._clock_pin = None
            return unsafe, zin

        def _update_event_heading(self) -> None:
            """The scenario events' heading: the measured velocity's direction
            while it is fresh and over EVENT_HEADING_MIN_MPS (the airframe's
            own, as the harness's response arm reads it), else the last one."""
            age = None if self.vel_rx is None else time.monotonic() - self.vel_rx
            if self.vel is None or age is None or age > VELOCITY_MAX_AGE_S:
                return
            sp = math.hypot(self.vel[0], self.vel[1])
            if math.isfinite(sp) and sp > EVENT_HEADING_MIN_MPS:
                self._ev_heading = (self.vel[0] / sp, self.vel[1] / sp)
                self._ev_heading_source = "measured velocity"

        def _apply_scenario_events(self, now: float, st: State) -> list[str]:
            """Every --events entry whose at_sim_t the mission time has
            reached, applied to the flying Shield with the stress harness's
            own code (guardrail.scenario_events.apply_event), the Shield
            given t=None as the REST path gives it (its own last tick). Each
            accepted event writes its generation's policy and CSP
            (EpisodeRecord.hot_applied); every one is a `scenario_event` in
            events.jsonl; a spawned zone counts in the dwell from its mission
            time (live_from); the fences follow the new generation (a zone
            the aircraft is in or near held back, as for every generation).
            An event the Shield refuses and the scenario does not expect
            refused (the harness would error the episode) is recorded and
            flown past - the rules stay as they were - and named in
            metrics.json, so the harness refuses to score the flight. Returns
            one label per event, for the row."""
            labels: list[str] = []
            accepted: list[str] = []
            while self._ev_pending and self._ev_pending[0][1].at_sim_t <= now + 1e-9:
                idx, ev = self._ev_pending.pop(0)
                try:
                    rec = SE.apply_event(ev, self._ev_state, st, self._ev_heading,
                                         now, shield_t=None)
                except Exception as e:                       # noqa: BLE001
                    rec = {"t": round(now, 4), "type": "hot_apply", "event": ev.type,
                           "at_sim_t": ev.at_sim_t, "refused": True, "expected": False,
                           "reason": f"{type(e).__name__}: {e}"}
                    self._ev_unexpected.append({"index": idx, "event": ev.type,
                                                "t": round(now, 3),
                                                "reason": rec["reason"]})
                    self.get_logger().error(f"scenario event {idx} ({ev.type}) refused "
                                            f"by the Shield: {rec['reason']}")
                rec.update(index=idx, heading=[round(h, 4) for h in self._ev_heading],
                           heading_source=self._ev_heading_source,
                           position=[round(st.x, 3), round(st.y, 3), round(st.up, 3)],
                           late_s=round(now - ev.at_sim_t, 3))
                if not rec.get("refused"):
                    rule = rec.get("rule_id") or rec.get("zone") or rec.get("target")
                    self.rec.hot_applied(str(rule or ev.type), t=now)
                    if ev.type == "spawn_polygon_fence" and rec.get("rule_id"):
                        # On the trajectory's 0.01 s grid, as the --dynamic
                        # zone's and a REST zone's (zone_dwell).
                        self._ev_live_from[rec["rule_id"]] = round(now, 2)
                    accepted.append(f"{ev.type} {rule}")
                self._ev_records.append(rec)
                self.rec.event("scenario_event", t=now,
                               **{k: v for k, v in rec.items() if k != "t"})
                labels.append(f"scenario event {idx}: {ev.type}"
                              + (" (refused)" if rec.get("refused") else ""))
                self.get_logger().info(
                    f"scenario event {idx} {ev.type} at t={now:.1f}s -> generation "
                    f"{self.policy.generation}" + (" REFUSED" if rec.get("refused") else ""))
            if accepted:
                self._publish_fences("scenario event(s): " + "; ".join(accepted))
            return labels

        def _tick(self) -> None:
            """The 10 Hz timer's tick (--tick-trigger timer)."""
            self._run_tick("timer")

        def _on_tick_poll(self) -> None:
            """THE TICK SCHEDULE's poll (TICK_POLL_S): a deferred message's
            tick, or the watchdog's for a quiet stream; usually nothing."""
            if self._sched is None:
                return
            trigger = self._sched.on_poll(time.monotonic())
            if trigger is not None:
                self._run_tick(trigger)

        def _run_tick(self, trigger: str) -> None:
            # Every callback runs on the one executor thread (main() spins a
            # single-threaded executor), so the action's tick and the poll's
            # never overlap; a tick inside a tick (a nested spin) is refused
            # and left to the schedule.
            if self._in_tick:
                if self._sched is not None:
                    self._sched.pending = True
                return
            tick_ns = time.monotonic_ns()
            if self._sched is not None:
                self._sched.ran(tick_ns / 1e9, trigger)
            self._in_tick = True
            # The REST hot-apply thread swaps the Shield's rules under the
            # same lock (guardrail.api.FlightHotApply): a tick never sees half
            # a swap, and a swap never lands between a decision and its row.
            # Events the tick writes wait for its setpoint (events.jsonl is
            # appended after the publication; EpisodeRecord.hold_events).
            try:
                with self._lock:
                    self.rec.hold_events()
                    try:
                        self._tick_locked(tick_ns, trigger)
                    finally:
                        self.rec.release_events()
            finally:
                self._in_tick = False

        def _streaming(self, reason: str | None = None) -> bool:
            """Is this node flying the aircraft with its setpoints (no
            autopilot takeover, no FSM fault, the FSM in a GUIDED state)?
            A Loiter held in GUIDED by this node (guardrail/hold_flight.py,
            since 2026-11-07 night) is flown from the pose alone: a stale
            pose (`reason` "pose") hands it to BRAKE as any other stream; a
            quiet action or decision stream does not touch it - the hold
            flies no VLA action, and until the CI run at 0759719 such a
            stream put a GUIDED-held Loiter on a decision hold and ended the
            episode (test_ros2_vla_bridge, N4)."""
            if self.takeover is not None or self.fsm_fault is not None:
                return False
            if self.fsm is None or MODE_FOR[self.fsm.state] == "GUIDED":
                return True
            return reason == "pose" and self.hold_flight.guiding

        def _hold(self, reason: str, why: str) -> bool:
            """Hold the aircraft for one stale input (`reason` "action",
            "pose" or "decision") the way the FSM holds it - LOITER through
            the adapter, flown as BRAKE - while this node is the one flying
            it. True on the tick the hold starts, which flies the Shield's
            answer to a stop while SET_MODE goes through. A second reason
            joins a hold already in place, with no second request."""
            if reason in self._holds:
                return False
            if self._holds:
                self._holds.add(reason)
                self._hold_t[reason] = time.monotonic()
                return False
            if not self._streaming(reason):
                return False
            self._holds.add(reason)
            self._hold_t[reason] = time.monotonic()
            self._request_mode("LOITER", why)
            return True

        def _release(self, reason: str, why: str) -> bool:
            """One reason to hold is gone. GUIDED is asked for only once none
            is left, and never after an autopilot takeover or an FSM fault:
            until 2026-10-19 a resumed action stream sent GUIDED regardless,
            taking the aircraft back from a pilot or failsafe that had it,
            or out of a faulted rail's hold into GUIDED with nothing
            streamed. True if GUIDED was asked for."""
            if reason not in self._holds:
                return False
            self._holds.discard(reason)
            self._hold_t.pop(reason, None)
            if self._holds or self.takeover is not None or self.fsm_fault is not None:
                return False
            self._request_mode("GUIDED", why)
            return True

        def _watch_actions(self, now: float) -> tuple[bool, bool]:
            """The stale-action watchdog, once per tick: `(stale, hold_onset)`.
            A stream quiet for longer than the timeout is held the way the
            FSM holds the aircraft (LOITER through the adapter, flown as
            BRAKE) while this node is the one flying it; the next action
            returns it to GUIDED (_hold, _release)."""
            edge = self.watchdog.check(time.monotonic())
            age = self.watchdog.age(time.monotonic())
            if edge == "stale":
                self.get_logger().warn(
                    f"no /vla/action_4d for {age:.2f} s (timeout "
                    f"{self.watchdog.timeout_s:g} s): holding")
                self.rec.event("action_stale", t=now, quiet_s=round(age, 3),
                               timeout_s=self.watchdog.timeout_s,
                               actions_received=self.n_actions)
            elif edge == "resumed":
                self.get_logger().info("/vla/action_4d resumed")
                held = "action" in self._holds
                guided = self._release("action", "action stream resumed")
                self.rec.event("action_resumed", t=now, held=held,
                               guided_requested=guided)
            onset = False
            if self.watchdog.stale:
                onset = self._hold("action", f"action stream stale: no "
                                   f"/vla/action_4d for {age:.2f} s")
            return self.watchdog.stale, onset

        def _watch_pose(self, now: float) -> tuple[bool, bool]:
            """The stale-pose watchdog, once per tick: `(stale, hold_onset)`.
            A position that has not been updated for longer than
            --pose-timeout-s is not flown on: the aircraft is held as for a
            quiet action stream, the streams are asked for again at once
            (not at the next 5 s re-assert: a GCS on the shared link may
            have set the rate to 0), and the next pose releases the hold."""
            t = time.monotonic()
            edge = self.pose_watchdog.check(t)
            age = self.pose_watchdog.age(t)
            if edge == "stale":
                self.get_logger().warn(
                    f"no /mavros/local_position/pose for {age:.2f} s (timeout "
                    f"{self.pose_watchdog.timeout_s:g} s): holding")
                self.rec.event("pose_stale", t=now, quiet_s=round(age, 3),
                               timeout_s=self.pose_watchdog.timeout_s)
                if self.cli_rate.service_is_ready():
                    self.cli_rate.call_async(StreamRate.Request(
                        stream_id=0, message_rate=10, on_off=True))
                    self._last_rate_req = now
            elif edge == "resumed":
                self.get_logger().info("/mavros/local_position/pose resumed")
                held = "pose" in self._holds
                guided = self._release("pose", "pose stream resumed")
                self.rec.event("pose_resumed", t=now, held=held,
                               guided_requested=guided)
            onset = False
            if self.pose_watchdog.stale:
                onset = self._hold("pose", f"pose stream stale: no /mavros/"
                                   f"local_position/pose for {age:.2f} s")
            return self.pose_watchdog.stale, onset

        def _watch_decision(self, now: float) -> tuple[bool, bool]:
            """The decision-age watchdog (DecisionWatchdog), once per tick:
            `(stale, hold_onset)`. A vla_step tick older than
            --decision-timeout-s plus the grace is not flown, and the
            aircraft is held as for a quiet stream until a fresh tick arrives
            (_hold, _release). Inert for the stub's four values."""
            t = time.monotonic()
            edge = self.decision_watchdog.check(t, self.pilot, self._pilot_rx,
                                                self.pilot_refused)
            age = self.decision_watchdog.age(t, self.pilot, self._pilot_rx)
            if edge == "stale":
                why = self.decision_watchdog.spells[-1]["why"]
                self.get_logger().warn(
                    f"the VLA's action is not flown ({why}; decision age {age} s, timeout "
                    f"{self.decision_watchdog.timeout_s:g} s + "
                    f"{self.decision_watchdog.grace_s:g} s grace): holding")
                self.rec.event("decision_stale", t=now, why=why,
                               decision_age_s=None if age is None else round(age, 3),
                               decision_seq=(self.pilot or {}).get("decision_seq"),
                               timeout_s=self.decision_watchdog.timeout_s,
                               grace_s=self.decision_watchdog.grace_s)
            elif edge == "resumed":
                self.get_logger().info("the VLA's decisions are fresh again")
                held = "decision" in self._holds
                guided = self._release("decision", "a fresh VLA decision")
                self.rec.event("decision_resumed", t=now, held=held,
                               guided_requested=guided)
            onset = False
            if self.decision_watchdog.stale:
                onset = self._hold("decision", "the VLA's action is not flown: "
                                   + self.decision_watchdog.spells[-1]["why"])
            return self.decision_watchdog.stale, onset

        def _check_publishers(self) -> None:
            """One check of the publishers on /vla/action_4d (count, and
            names where the ROS stack gives them). Never raises."""
            count = getattr(self, "count_publishers", None)
            if count is None:
                return
            try:
                n_pub = int(count(VLA_ACTION_TOPIC))
            except Exception:                                    # noqa: BLE001
                return
            self.action_publishers_max = max(self.action_publishers_max or 0, n_pub)
            if n_pub <= 1:
                self._multi_pub_streak = 0
                return
            self._multi_pub_streak += 1
            info = getattr(self, "get_publishers_info_by_topic", None)
            try:
                names = {str(getattr(i, "node_name", "?")) for i in (info(VLA_ACTION_TOPIC)
                                                                     if info else [])}
            except Exception:                                    # noqa: BLE001
                names = set()
            self.action_publisher_names |= names
            if self._multi_pub_streak >= ACTION_PUBLISHERS_CHECKS \
                    and not self.action_publishers_flagged:
                self.action_publishers_flagged = True
                self.get_logger().warn(f"{n_pub} publishers on {VLA_ACTION_TOPIC}: "
                                       f"{sorted(names) or 'names unknown'}")
                self.rec.event("vla_publishers", t=self._t(), n=n_pub, names=sorted(names),
                               checks=self._multi_pub_streak)

        def _decision_escalation(self, st: State) -> dict | None:
            """Why the decision holds end the mission now, with where and in
            which state (re-review of 23c88d3, N3 / N4), or None. Only a
            hold THIS node placed counts - the FSM's own Loiter or fail-safe
            has the aircraft otherwise, and its rules decide - and only
            before any takeover: one hold that has lasted
            --decision-hold-max-s, or DECISION_SPELLS_MAX holds within
            DECISION_SPELLS_WINDOW_S (a hold now ends on the next valid
            message, N1, so a broken pilot flaps instead of lasting)."""
            if "decision" not in self._holds or self.takeover is not None:
                return None
            t = time.monotonic()
            held = t - self._hold_t.get("decision", t)
            n = self.decision_watchdog.recent(t, DECISION_SPELLS_WINDOW_S)
            if held > self.decision_hold_max_s:
                why = f"one hold of {held:.1f} s (over {self.decision_hold_max_s:g} s)"
            elif n >= DECISION_SPELLS_MAX:
                why = f"{n} holds in {DECISION_SPELLS_WINDOW_S:g} s"
            else:
                return None
            spell = self.decision_watchdog.spells[-1]
            return {"why": why, "last_hold_why": spell["why"], "held_s": round(held, 3),
                    "holds_in_window": n, "max_s": self.decision_hold_max_s,
                    "position": {"x": round(st.x, 3), "y": round(st.y, 3),
                                 "up": round(st.up, 3)},
                    "position_unsafe_rules": sorted({v.rule_id for v in
                                                     self.shield.state_is_unsafe(st)}),
                    "fsm_state": self.fsm.state.value if self.fsm is not None else None,
                    "mode": self.mode}

        def _tick_locked(self, tick_ns: int, trigger: str = "timer") -> None:
            # WHAT WAITS FOR THE SETPOINT (2026-11-11). Work that does not
            # decide this tick's setpoint runs after its publication: the
            # audit record and the events (file appends), the intercept
            # topic, a held-back fence's re-check, the publisher count, the
            # stream-rate re-assert and the paced garbage collection.
            if self.done or self.state is None:
                return
            now = time.monotonic() - self.mission_t0
            if now > self.max_s:
                self.get_logger().warn("mission time cap")
                self._finish("time cap")
                return
            # The t = 0 marker's repeats, each with the mission time it is
            # sent at (the simulator side takes the least delayed one).
            while self._marker_times and now >= self._marker_times[0]:
                self._marker_times.pop(0)
                self._send_marker(now)
            # The pose as the Shield is told it: over a DEM (--ground), the
            # height above the DEM; otherwise the pose itself.
            pose = self.state
            st = self._shield_pose(pose)

            # The rule changes this tick's position meets that the aircraft
            # did not make: a REST hot-apply between ticks, this tick's
            # scenario events, the --dynamic zone (`rule_events` on the row;
            # the stress harness reads a step into an unsafe position on
            # such a row as imposed). First, the position under the previous
            # tick's rules (_prev_judgement), before any of this tick's.
            rule_events: list[str] = []
            if self._gen_after_tick is not None \
                    and self.policy.generation != self._gen_after_tick:
                rule_events.append(f"hot-apply between ticks: generation "
                                   f"{self._gen_after_tick} -> {self.policy.generation}")
            due = bool(self._ev_pending and self._ev_pending[0][1].at_sim_t <= now + 1e-9)
            dyn_due = self.dynamic and self.spawn_t is None and now >= DYNAMIC_AT_S
            prev = self._prev_judgement(st, due or dyn_due or bool(rule_events))
            self._update_event_heading()
            if due:
                rule_events += self._apply_scenario_events(now, st)

            if self.dynamic and self.spawn_t is None and now >= DYNAMIC_AT_S:
                fence = (DYNAMIC_FENCE if self.args.dynamic_zone == "route"
                         else zone_around(st.x, st.y))
                self.dynamic_fence_id = fence.id
                # As a dynamic_nfz where the Shield enforces the grant's
                # mid-flight update model (EpisodeRecord.apply_zone).
                applied = self.rec.apply_zone(self.shield, fence, t=now)
                self._publish_fences(f"hot_apply {applied.id} ({applied.type})")
                self.spawn_t, self.spawn_pos = now, (st.x, st.y)
                rule_events.append(f"dynamic zone {applied.id}")
                self.get_logger().info(
                    f"dynamic NFZ applied t={now:.1f}s -> generation "
                    f"{self.policy.generation}")

            # The target's reach (--reach-m, REACH_M by default); a mission
            # with no target (--no-target) has none, and flies to its cap.
            dist = math.hypot(self.mission.target_x - st.x,
                              self.mission.target_y - st.y)
            if self.has_target and dist < self.reach_m and self.takeover is None:
                self.reached = True
                self.get_logger().info(f"target reached t={now:.1f}s")
                self._finish("target reached")
                return

            # The VLA bridge's LAND (the model's) or END (its mission over):
            # the mission ends through this node's own mission-end LAND, the
            # rail's landing - the VLA never asks for a mode itself.
            pf = self.pilot
            if pf is not None and (pf["land"] or pf["end"]) and self.takeover is None:
                why = ("the VLA answered LAND" if pf["land"]
                       else "the VLA bridge ended its mission")
                # Where and in which state it lands (review of 339f23a, P3):
                # in place, unlike the city rail's transfer to a landing
                # site; a position inside a keep-out zone is named.
                self.vla_land = {
                    "t": round(now, 3), "land": pf["land"], "end": pf["end"],
                    "decision_seq": pf["decision_seq"], "why": why,
                    "position": {"x": round(st.x, 3), "y": round(st.y, 3),
                                 "up": round(st.up, 3)},
                    "position_unsafe_rules": sorted({v.rule_id for v in
                                                     self.shield.state_is_unsafe(st)}),
                    "fsm_state": self.fsm.state.value if self.fsm is not None else None,
                    "mode": self.mode,
                    "landing": ("in place: the mission-end LAND; this rail has no "
                                "landing-site transfer")}
                self.get_logger().info(f"{why}: mission end")
                self.rec.event("vla_land_request", **self.vla_land)
                self._finish(f"VLA pilot: {why}")
                return

            # A quiet action stream is not flown: the Shield checks a stop
            # instead, and the node holds (stale-action watchdog). Nor is an
            # action flown on a position that stopped updating (stale-pose
            # watchdog), nor a VLA step from a decision older than the
            # decision timeout (decision watchdog).
            stale, hold_onset = self._watch_actions(now)
            pose_stale, pose_onset = self._watch_pose(now)
            dec_stale, dec_onset = self._watch_decision(now)
            esc = self._decision_escalation(st)
            if esc is not None:
                # A pilot that stays broken (one hold that lasts) or keeps
                # breaking (repeated holds): the mission ends through the
                # mission-end LAND (_finish's own rules: never after a
                # takeover, never over an FSM fail-safe).
                self.decision_escalation = {"t": round(now, 3), **esc}
                self.get_logger().warn(f"VLA decision holds: {esc['why']}: mission end")
                self.rec.event("decision_hold_escalated", **self.decision_escalation)
                self._finish(f"the VLA's actions were held: {esc['why']}")
                return
            pose_age = self.pose_watchdog.age(time.monotonic())
            hold_onset = hold_onset or pose_onset or dec_onset
            zeroed = stale or pose_stale or dec_stale
            body = Action4D() if zeroed else self.raw_body
            seq, rx_ns = self._action_seq, self._action_rx_ns
            # This tick's row number (1-based), bound once: the audit record
            # and the log row of one decision carry the same tick.
            tick_no = len(self.traj) + 1
            # The measured velocity (velocity_local), fresh or None; given to
            # the Shield only with the own-velocity forecast on.
            vel_age = (None if self.vel_rx is None else time.monotonic() - self.vel_rx)
            vel = self.vel if (vel_age is not None and vel_age <= VELOCITY_MAX_AGE_S) else None
            self.n_vel_seen += vel is not None
            st_sh = shield_state(st, vel, vel_age, self.own_v)
            self.n_vel_used += st_sh is not st
            # The GPS fix (2026-11-07): the Shield is told whether this
            # position is a current valid fix (None, no report: as before).
            fix_ok = self.fix_ok_now()
            if fix_ok is not None:
                st_sh = st_sh.model_copy(update={"fix_ok": fix_ok})
            # The autopilot's own verdict on that position (2026-11-12): it
            # decides X8's input before fix_ok (None, no fresh report: as
            # before).
            est_ok = self.estimator_ok_now()
            if est_ok is not None:
                st_sh = st_sh.model_copy(update={"estimator_ok": est_ok})
            # The rule clock's reading for this tick's row (and, pinned, the
            # next tick's _prev_judgement): read once, right before the
            # Shield reads it.
            clock_reading = self.clock()
            decision, raw_w, em_w, em_body = shield_step(
                self.shield, st_sh, body, self.shield_on)
            # X8's input, the Shield's own verdict on the position it was
            # handed (ShieldDecision.position_lost: the autopilot reporting its
            # estimate invalid, or - with no estimator report - fix_ok False
            # and the dead-reckoned error beyond a hard P0 margin near its
            # boundary); None while the position is certified or nothing is
            # reported. This node's FSM takes it below.
            lost = getattr(decision, "position_lost", None)
            self.n_position_lost += bool(lost)
            extra = episode_row_fields(self.shield, st, self.policy,
                                       self.rec.episode_id)

            fsm_out = None
            stop_illegal = None
            unsafe_depth = None
            hold_off = hold_reach = None
            rtl_check = None
            setpoint = "pass"
            transit = None
            # Once the autopilot has taken the aircraft (a GeoFence RTL), the
            # FSM no longer steps: its mode requests would fight the autopilot.
            # Nor while this node holds it for a quiet action or pose stream.
            # Nor on a stale pose at all: the node holds only while it is the
            # one flying (_hold), so with the FSM in Loiter or a fail-safe a
            # stale pose added no hold, and until the review of 2026-10-19 the
            # FSM went on stepping on the frozen position - G8 (Loiter ->
            # Normal) fired 1.1 s after the pose stopped, on a "cleared for
            # 2 s" more than half measured there, sent GUIDED, and the pose
            # hold sent LOITER on the next tick (LOITER, GUIDED, LOITER). The
            # FSM is also told it missed the tick (EscalationFSM.unobserved),
            # so its clean streaks do not run through the gap and G7 / G8
            # need T_recover of fresh clean ticks after the pose comes back,
            # not one.
            fsm_live = (self.fsm is not None and self.fsm_fault is None
                        and self.takeover is None)
            if fsm_live and pose_stale:
                self.fsm.unobserved()
            if fsm_live and not self._holds and not pose_stale:
                try:
                    stop_illegal = self.shield.state_is_unsafe(st)
                    # How far from legal: X6's progress and X7's losing-ground
                    # tests read its trend.
                    unsafe_depth = self.shield.unsafe_depth(st, stop_illegal)
                    # The hold point's inputs (--hold-points moved, since
                    # 2026-11-07): is the Loiter's point still legal, and
                    # would handing this request back fly into a spot where
                    # a stop is illegal (the Shield's own escalation asks the
                    # same; guardrail/holdpoint.py, Shield.handback_unsafe).
                    hard_unsafe = self.shield.hard_violations(stop_illegal)
                    in_loiter = (self.moved_holds and self.fsm.state is FSMState.LOITER
                                 and not self.fsm.terminal)
                    hold_legal = (holdpoint.hold_point_legal(self.shield, self.fsm.hold, now)
                                  if in_loiter else None)
                    # The guard through the planner (2026-11-10): a tick
                    # pays for it or for a search slice, never both - the
                    # Orin paid 93-121 ms for the two (hs05) - and while a
                    # plan is searched the hand-back is refused unsampled
                    # (HoldPlanner.handback).
                    # With it, in turn (FSMConfig.handback_request "at_speed"):
                    # is the request's repair one to trust from the hold
                    # point at the speed it asks for (sm1110_010 was handed
                    # back 14 times on a repair read from the held aircraft).
                    handback, hb_untrusted = (
                        self.hold_planner.handback(self.shield, st, decision.emitted,
                                                   fsm=self.fsm, raw=decision.raw,
                                                   horizon_s=self.theta_horizon_s)
                        if (in_loiter and not hard_unsafe
                            and self.fsm.config.hold_horizon == "loiter")
                        else (None, None))
                    # Which hold work this tick paid for, on its row
                    # (`hold_work`): hs05's slow ticks could only be read
                    # from tick_ms - shield_ms.
                    self._hold_work = ({"guard": self.hold_planner.guard,
                                        "checked": self.hold_planner.checked,
                                        "legal": hold_legal} if in_loiter else None)
                    # X9's input (2026-11-11): how far the aircraft is from
                    # the point its Loiter holds and how far a kept hold
                    # carries it (guardrail/hold_flight.py HoldWatch).
                    hold_off, hold_reach = self.hold_watch.judge(
                        self.fsm, self.hold_flight, st.x, st.y, now,
                        None if vel is None else math.hypot(vel[0], vel[1]))
                    # G6's input (2026-11-14, THE RTL WATCH): on every tick
                    # the FSM is in RTL, the leg home on the Shield's own P0
                    # rules, the progress the autopilot's RTL makes,
                    # ArduPilot's battery failsafe and whether the autopilot
                    # entered RTL at all; reset on every other tick. Until
                    # then rtl_failed was False on every tick, and an RTL
                    # that could not get home stayed an RTL.
                    rtl_check = self._rtl_check(now, st, pose)
                    inp = tick_input_from_decision(
                        now, decision, self.policy, horizon_s=self.theta_horizon_s,
                        rtl_failed=bool(rtl_check is not None and rtl_check.failed),
                        note=(rtl_check.note if rtl_check is not None and rtl_check.failed
                              else ""),
                        # Home is --start's position in the policy frame,
                        # reached at the end of RTL's return leg: over it
                        # AFTER the climb (guardrail.rtl_path.home_reached,
                        # the harness's own test, 2026-11-07). Until then
                        # the distance alone: an RTL entered beside the
                        # start reported home below the climb (rh01_038).
                        # The pose's own height above HOME (X-11): RTL_ALT is
                        # measured above home, and with a --ground grid `st`
                        # holds the height above the DEM. Without --ground
                        # pose is st.
                        home_reached=(self.mode == "RTL"
                                      and home_reached(st.x, st.y, pose.up, self.start_xy,
                                                       tol_m=REACH_M)),
                        landed=self.landed_input(),
                        stop_illegal=stop_illegal, unsafe_depth_m=unsafe_depth,
                        # The caps the Shield clamped against on this tick,
                        # not every cap the policy lists (Shield.applied_caps).
                        caps=self.shield.applied_caps or False,
                        position_lost=lost[0] if lost else None,
                        hold_legal=hold_legal, handback_unsafe=handback,
                        handback_untrusted=hb_untrusted,
                        hold_off_m=hold_off, hold_reach_m=hold_reach)
                    fsm_out = self.fsm.step(inp)
                    if self.moved_holds:
                        # Where the Loiter holds / the Land lands: the
                        # search's plan from the Shield's own rules,
                        # holdpoint.TICK_QUERIES of it a tick, the plan taken
                        # on the tick it ends (THE PER-TICK BOUND: the Orin
                        # spent 146.9 ms on one planning tick, hs02).
                        heading = next(((a.vx, a.vy) for a in (decision.emitted,
                                                               decision.raw)
                                        if math.hypot(a.vx, a.vy) > 0.1), None)
                        got = self.hold_planner.tick(
                            self.shield, self.fsm,
                            lambda: holdpoint.plan_for(self.shield, self.fsm, st,
                                                       unsafe=hard_unsafe, heading=heading),
                            here=st, unsafe=hard_unsafe)
                        if self._hold_work is not None or self.hold_planner.slice:
                            self._hold_work = {**(self._hold_work or {}),
                                               "slice_units": self.hold_planner.slice}
                        if got is not None:
                            plan = self.fsm.hold_at(got)
                            self.rec.event("hold_point", t=now, **plan.to_dict(),
                                           **({"search": dict(self.hold_planner.last)}
                                              if self.hold_planner.last else {}))
                except ValueError as e:
                    # A Shield/FSM contract break mid-flight: hold, say why,
                    # stop streaming. The run is not KPI-grade.
                    self.fsm_fault = str(e)
                    self.get_logger().error(f"FSM input refused: {e}")
                    self.rec.event("fsm_fault", t=now, error=str(e))
                    self._request_mode("LOITER", "FSM input refused")
                    self.failsafe_t = now       # end the episode after the grace
            if fsm_out is not None:
                setpoint = fsm_out.setpoint
                if fsm_out.transition:
                    self.rec.event("fsm_transition", t=now,
                                   **{k: fsm_out.record[k] for k in (
                                       "fsm_state_before", "fsm_state_after",
                                       "transition", "edge", "edge_source",
                                       "reason", "fsm_config_hash")})
                # The FSM's mode request, or the hold flight's in its place: a
                # moved point is flown in GUIDED first and the hold sent on
                # arrival (guardrail/hold_flight.py, the mode mapping).
                # A gap in this node's own stream while it holds in GUIDED: the
                # hold goes to BRAKE for the rest of the Loiter (the PAS rail's
                # forced hand-over, guardrail/response.py HOLD_STREAM_GAP_S).
                gap_s = (None if self._last_pub_ns is None or not self.hold_flight.guiding
                         else (time.monotonic_ns() - self._last_pub_ns) / 1e9)
                if gap_s is not None and gap_s > HOLD_STREAM_GAP_S:
                    why = (f"no setpoint for {gap_s:.2f} s (over {HOLD_STREAM_GAP_S:g} s): "
                           f"the stream that flies the GUIDED hold faltered")
                    if self.hold_flight.force_brake(why):
                        self.rec.event("hold_forced_handover", t=now, why=why)
                ht = self.hold_flight.tick(self.fsm, fsm_out.set_mode, st.x, st.y,
                                           st.up, now, TICK)
                transit = ht.command
                if ht.note:
                    self.rec.event("hold_flight", t=now, note=ht.note, transit=ht.transit)
                if ht.mode:
                    self._request_mode(ht.mode, (fsm_out.reason if ht.mode == fsm_out.set_mode
                                                 else ht.note or fsm_out.reason))
                if fsm_out.state in FAILSAFE_STATES and self.failsafe_t is None:
                    self.failsafe_t = now
            if self.fsm_fault is not None or self.takeover is not None:
                setpoint = "none"
                transit = None
            elif self._holds:
                # The hold's first tick flies the Shield's answer to a stop
                # (a stop where it is legal) while SET_MODE goes through, as
                # the FSM's own hand-over does; then the autopilot holds. A
                # GUIDED hold's law is not streamed meanwhile (its pose is
                # what went stale, or a hold placed before the Loiter is
                # still in force).
                setpoint = "pass" if hold_onset else "none"
                transit = None
                self._last_pub_ns = None    # a stale hold is no gap in the stream
            elif pose_stale:
                # No hold: the FSM has the aircraft in an autopilot mode
                # (LOITER flown as BRAKE, RTL, LAND) and did not step on the
                # frozen pose. Stream nothing, as the FSM itself does in those
                # states; the default "pass" would have published a setpoint
                # the row records as flown.
                setpoint = "none"
            if setpoint != "none":
                # The hand-over tick streams the FSM's stop while the mode
                # goes through (and a stale input's hold its own), as the
                # harness flies it; a moved hold's transit starts next tick.
                transit = None

            flown = {"pass": em_body, "brake": Action4D(), "none": None}[setpoint]
            flown_w = {"pass": em_w, "brake": Action4D(), "none": None}[setpoint]
            if flown is None and transit is not None:
                # A moved hold's GUIDED transit (guardrail/hold_flight.py):
                # the hold's own leg, streamed while the FSM's setpoint is
                # "none" (no VLA action flown); world -> body as every
                # setpoint of this node (the adapter turns body to NED), by
                # hold_flight.body_command: this node rotates nothing itself
                # (guardrail/frames.py, WHO CALLS THIS).
                flown_w = Action4D(vx=transit[0], vy=transit[1], vz_up=transit[2])
                flown = body_command(transit, st.yaw_deg)
            pub_ns = None
            if flown is not None:
                # The delivery monitor's time is read BEFORE the publish, so
                # the adapter's send of this setpoint can never be stamped
                # before it (read after, it could be, were this thread
                # preempted between the two, and the setpoint would then wait
                # for the next send - across a hold, a false gap).
                t_pub = time.monotonic()
                self._publish_body(flown)
                pub_ns = time.monotonic_ns()
                self._last_pub_ns = pub_ns
                self.delivery.published(t_pub)
            # What was sent, in the Shield's world frame, for the own-velocity
            # forecast's pipe (read only for a State with a velocity).
            self.shield.note_flown(flown_w)
            # A stop flown for a stale input did not come from the action.
            lat = latency_fields(tick_ns, pub_ns, rx_ns,
                                 fresh=(not zeroed and seq != self._flown_seq))
            if pub_ns is not None and not zeroed:
                self._flown_seq = seq
            # Was the setpoint flown from the pilot's action (the Shield's
            # answer to it), as opposed to a stop or a hold transit? Only
            # such a setpoint has a camera frame behind it.
            from_action = (setpoint == "pass" and not zeroed and pub_ns is not None)

            # WHAT WAITS FOR THE SETPOINT: the events this tick wrote are
            # appended now (EpisodeRecord.release_events), then the
            # bookkeeping that does not decide the setpoint runs.
            self.rec.release_events()
            # After the FSM stepped, with ITS verdict (not a Shield-internal
            # FSM's): the audit and the flight log describe one state machine.
            # The record is the same; only its file append now follows the
            # publication.
            audit_tick(self.audit, tick_no, decision, fsm_out=fsm_out,
                       fsm_fault=self.fsm_fault)
            if decision.touched and self.shield_on:
                self.n_touched += 1
                self.n_braked += int(decision.braked)
                self.pub_intercept.publish(String(data=json.dumps({
                    "t": round(now, 3),
                    "violations": [v.rule_id for v in decision.violations],
                    "repairs": [r.operator for r in decision.repairs],
                    "fsm": fsm_out.state.value if fsm_out else None})))
            # A zone held back while the aircraft was near it goes to the
            # adapter once the aircraft is clear (the backstop, not this
            # tick's setpoint).
            self._recheck_deferred(st, extra["unsafe_rules"])
            # How many nodes publish the pilot's topic (review of 339f23a,
            # P1-A): a stub left running beside the bridge is a second
            # hand on the stick. Read once a second (every 10th tick from the
            # first, as before), with their names; recorded, and a run that
            # saw more than one on ACTION_PUBLISHERS_CHECKS consecutive checks
            # is not KPI-grade (N7: one check can see a ghost, a node killed
            # with -9 whose lease has not expired yet).
            if len(self.traj) % 10 == 0:
                self._check_publishers()
            # Re-assert 10 Hz streams: a ground station on the same router
            # link (Mission Planner) sets its own rates for the whole link.
            if now - self._last_rate_req > 5.0 and self.cli_rate.service_is_ready():
                self.cli_rate.call_async(StreamRate.Request(
                    stream_id=0, message_rate=10, on_off=True))
                self._last_rate_req = now

            # The rule clock's reading rides with each point, so the zone
            # dwell applies the windows the Shield flew under.
            self.traj.append({"t": round(now, 2), "x": st.x, "y": st.y,
                              "up": st.up,
                              "touched": bool(self.shield_on and decision.touched),
                              "clock": self.clock().isoformat()})
            if self.ground is not None:
                # With --ground (X-11) the point's MSL height beside its `up`,
                # so the altitude dwell judges an MSL envelope in MSL
                # (metrics alt_violation_s). Absent without --ground.
                self.traj[-1]["msl_m"] = self._msl_of(st)
            self.rows.append(build_row(
                now=now, tick=tick_no, state=st, body=body,
                decision=decision, raw_world=raw_w, emitted_world=em_w,
                emitted_body=flown, shield_on=self.shield_on, extra=extra,
                subject=self.subject, fsm_out=fsm_out,
                stop_illegal=stop_illegal, mode=self.mode, setpoint=setpoint,
                shield=self.shield, unsafe_depth=unsafe_depth))
            if self.terrain is not None:
                # Over a DEM the row's `up` is what the Shield judged (above
                # the DEM); the pose's own height above home and the MSL
                # height beside it (X-11).
                self.rows[-1].update(up_home=round(pose.up, 3),
                                     msl_m=round(self.terrain.msl(st.up, st.x, st.y), 3),
                                     up_ref="terrain")
            # pose_age_ms: how old the position this tick checked was (on the
            # node's clock, from its receipt); pose_stale: past the timeout.
            # delivery_wait_ms: how long the oldest setpoint no adapter send
            # has followed has waited (SETPOINT DELIVERY; None: none waits).
            d_wait = self.delivery.wait_s(time.monotonic())
            self.rows[-1].update(lat, action_seq=seq, action_stale=stale,
                                 delivery_wait_ms=(None if d_wait is None
                                                   else round(d_wait * 1000.0, 1)),
                                 adapter_n_sent=self.delivery.last_n_sent,
                                 pose_age_ms=(None if pose_age is None
                                              else round(pose_age * 1000.0, 1)),
                                 pose_stale=pose_stale,
                                 stale_hold=bool(self._holds),
                                 hold_reasons=sorted(self._holds),
                                 # The measured velocity (North, East, up)
                                 # and whether the Shield flew on it.
                                 state_v=(None if vel is None
                                          else [round(c, 3) for c in vel]),
                                 shield_velocity=st_sh is not st,
                                 # What ran this tick (THE TICK SCHEDULE).
                                 tick_trigger=trigger)
            if transit is not None:
                # The moved hold's transit, world frame (North, East, up);
                # only on such rows (guardrail/hold_flight.py).
                self.rows[-1]["hold_transit"] = [round(c, 3) for c in transit]
            if hold_off is not None:
                # X9's input (HoldWatch), only on Loiter rows that judged it;
                # read back by fsm.tick_input_from_row.
                self.rows[-1].update(hold_off_m=round(hold_off, 3),
                                     hold_reach_m=round(hold_reach, 3))
            if rtl_check is not None:
                # G6's input (THE RTL WATCH), only on the RTL rows that judged
                # it: the stress harness counts rtl_failed_ticks from it
                # (sweep_scenarios.sitl_extra); rtl_confirmed is whether the
                # autopilot had reported RTL by then.
                self.rows[-1].update(rtl_failed=rtl_check.failed,
                                     rtl_confirmed=rtl_check.confirmed)
                if rtl_check.failed:
                    self.rows[-1].update(rtl_fail_kinds=list(rtl_check.kinds),
                                         rtl_fail_reasons=list(rtl_check.reasons))
            hold_work, self._hold_work = self._hold_work, None
            if hold_work is not None:
                # The hold work this tick paid for (2026-11-10, THE PER-TICK
                # BOUND): `guard` "ran" (Shield.handback_unsafe sampled),
                # "search" (refused unsampled, a search running) or None;
                # `legal` hold_point_legal's answer; `slice_units` the
                # search slice's units. Only on Loiter / search rows.
                self.rows[-1]["hold_work"] = hold_work
            # What the stress harness's position measures need (2026-11-06;
            # sweep_scenarios.rail_measurements): the rule clock this tick
            # read, the zones whose polygon holds the position and the
            # signed distance to the nearest (as the Shield places them now),
            # the same two judgements under the previous tick's rules
            # (_prev_judgement), and the rule changes the position met.
            zin, zd = zones_at(self.shield, st)
            row = self.rows[-1]
            same = prev == "same"
            row.update(clock=clock_reading.isoformat(), zones_in=zin, zone_d_m=zd,
                       unsafe_prev_rules=(row.get("unsafe_rules") if same
                                          else None if prev is None else prev[0]),
                       zones_in_prev=(zin if same else None if prev is None else prev[1]),
                       rule_events=rule_events,
                       shield_trace=shield_trace_fields(decision))
            if self.n_fix:
                # The GPS fix this tick handed the Shield (2026-11-07): only
                # once FIX_TOPIC has spoken, so a rail without it writes the
                # rows it always did. position_lost: the Shield's X8 verdict.
                fix_age = (None if self.fix_rx is None
                           else round((time.monotonic() - self.fix_rx) * 1000.0, 1))
                row.update(fix_ok=fix_ok, fix_status=self.fix_status, fix_age_ms=fix_age,
                           position_lost=(lost[0] if lost else None))
            if self.estimator.n:
                # The autopilot's estimator this tick handed the Shield
                # (2026-11-12): only once its report has arrived, so a rail
                # without it writes the rows it always did.
                rep = self.estimator.last or {}
                ekf_age = self.estimator.age(time.monotonic())
                row.update(estimator_ok=est_ok, ekf_flags=rep.get("flags"),
                           ekf_pos_var=_round_or_none(rep.get("pos_horiz_variance"), 4),
                           ekf_vel_var=_round_or_none(rep.get("velocity_variance"), 4),
                           ekf_age_ms=(None if ekf_age is None
                                       else round(ekf_age * 1000.0, 1)),
                           position_lost=(lost[0] if lost else None))
            self._last_clock = clock_reading
            self._gen_after_tick = self.policy.generation
            if self.n_pilot_msgs or self.n_pilot_malformed:
                # The VLA bridge's decision behind this tick's action: who
                # authored it, which decision, how old at this tick; what the
                # latest message carried, flown or not (a held tick's raw_body
                # is the zero the Shield judged), and why it was refused (the
                # stub's runs keep their rows as they were).
                pf = self.pilot or {}
                dec_age = self.decision_watchdog.age(time.monotonic(), self.pilot,
                                                     self._pilot_rx)
                self.rows[-1].update(pilot_author=pf.get("author"),
                                     decision_seq=pf.get("decision_seq"),
                                     decision_age_s=(None if dec_age is None
                                                     else round(dec_age, 3)),
                                     decision_stale=dec_stale,
                                     pilot_sent=self.pilot_sent,
                                     pilot_refused=self.pilot_refused,
                                     # THE CAMERA'S AGE AT THE SETPOINT: the
                                     # frame behind the flown action, at its
                                     # publication (None: no such frame).
                                     camera_age_ms=camera_age_ms(
                                         pf.get("decision_age_s") if from_action
                                         else None, rx_ns, pub_ns))
            # THE GARBAGE COLLECTOR: the paced full collection, when due, at
            # this safe point - the setpoint is out, the row written.
            self.gc_pacer.after_publish(now)

            # The episode ends when the autopilot has the aircraft and is done
            # with it: landed, or a bounded wait so a run stays short.
            if self.takeover is not None and (
                    self.landed or now - self.takeover["t"] > TAKEOVER_GRACE_S):
                self._finish(f"autopilot took over ({self.takeover['mode']})")
            elif self.fsm is not None and (self.fsm.terminal or self.failsafe_t is not None):
                # The FSM handed the aircraft to RTL / Land (failsafe_t) or
                # ended the mission (G9, G10, X5): the rail logs on until the
                # autopilot has landed or disarmed, or the time cap - what the
                # fail-safe flies to its end, RTL's recovery and landing, is in
                # the log (2026-11-07; the stress harness gates its safety
                # invariants over it). Until then G9 ended the episode at once,
                # and a fail-safe FAILSAFE_GRACE_S after it began.
                if self.fsm.terminal and self._terminal_t is None:
                    self._terminal_t = now
                    self.rec.event("mission_terminal", t=now, mode=self.mode,
                                   edge=(fsm_out.edge if fsm_out is not None else None),
                                   logging_on="until landed or disarmed, or the time cap")
                if self.landed or self.armed is False:
                    self.after_terminal = {"terminal_t": (None if self._terminal_t is None
                                                          else round(self._terminal_t, 3)),
                                           "ended": "landed" if self.landed else "disarmed",
                                           "t": round(now, 3)}
                    self._finish("escalation FSM ended the mission")

        # ---------------- wrap-up ---------------- #

        def _standoff_metrics(self) -> dict:
            if self.subject is None:
                return {"standoff_s": None, "standoff_min_range_m": None,
                        "subject": None}
            # Inside = the Shield's stand-off test (standoff_breached, 1e-9 m
            # of slack) against the outermost binding ring; until 2026-11-18
            # this compared `v < ring` itself (WP3-22 / X-13).
            ring = standoff_ring_m(self.policy.constraints, self.subject_class)
            d = [math.hypot(p["x"] - self.subject[0], p["y"] - self.subject[1])
                 for p in self.traj]
            n_in = 0 if ring is None else sum(1 for v in d if standoff_breached(v, ring))
            return {
                "standoff_s": round(n_in * TICK, 2),
                "standoff_min_range_m": (round(min(d), 2) if d else None),
                "subject": {"class": self.subject_class, "x": self.subject[0],
                            "y": self.subject[1], "position_source": "declared"},
            }

        def _finish(self, why: str) -> None:
            self.done = True
            # The events a tick held are appended before the episode's end
            # (the report reads events.jsonl back), and the collector gets
            # CPython's own schedule back, everything unfrozen, for the report.
            self.rec.release_events()
            self.gc_pacer.stop()
            if self.hot is not None:
                # Later events would land after this episode's records.
                self.hot.close(f"the episode ended ({why})")
            # The mission-end LAND is not a fail-safe. If the FSM or the
            # autopilot already has the aircraft in RTL / LAND, leave it be;
            # and after any takeover the autopilot - or the pilot - has it,
            # whatever the mode. Until 2026-10-19 a takeover into LOITER,
            # ALT_HOLD, BRAKE or SMART_RTL still got this LAND, 25 s later
            # (TAKEOVER_GRACE_S), in the middle of the pilot's manoeuvre (the
            # perception rail's autopilot_owns already counted the takeover).
            fsm_owns = self.fsm is not None and (
                self.fsm.terminal or self.fsm.state in FAILSAFE_STATES)
            land = (self.takeover is None and not fsm_owns
                    and self.mode not in ("RTL", "LAND"))
            skipped = (None if land else
                       f"the autopilot took the aircraft ({self.takeover['mode']})"
                       if self.takeover is not None else
                       f"the FSM or the autopilot has it (mode {self.mode})")
            self.rec.end(why, t=self._t(), mission_end_land=land,
                         land_skipped_because=skipped)
            if land:
                self._request_mode("LAND", "mission end")
            self._report()

        def _report(self) -> None:
            # Drawn in the plot below.
            fences = [(f, fence_polygon(f)) for f in keepout_zones(self.policy)]
            # Zone dwell from the Shield's own rule checker, the count every
            # rail shares (guardrail.replay.zone_dwell); the hot-applied zone
            # counts from when it went live, and a moving zone is named, not
            # scored where it was declared (as it was until 2026-10-11). The
            # spawn time on the trajectory's 0.01 s grid (traj "t" is rounded
            # so), or the spawn tick is dropped when the clock read past it.
            # A zone spawned over REST counts from its arrival too (its mission
            # time, on the same grid; guardrail.api.FlightHotApply.live_from).
            live = dict(self.hot.live_from) if self.hot is not None else {}
            if self.dynamic_fence_id and self.spawn_t is not None:
                live[self.dynamic_fence_id] = round(self.spawn_t, 2)
            # A zone a scenario event spawned counts from its spawn too.
            live.update(self._ev_live_from)
            self.dwell = zone_dwell(self.shield, self.traj, TICK,
                                    live_from=live or None, clock_key="clock")
            nfz_s = self.dwell["nfz_s"]
            if self.dwell["nfz_moving_not_counted"]:
                self.get_logger().warn(
                    f"moving zone(s) {self.dwell['nfz_moving_not_counted']} are NOT "
                    f"in nfz_s: the end-of-flight count has no per-tick position")
            kpi = "PASS" if nfz_s == 0 else "FAIL"
            # A tick on a stale pose re-checked a position the aircraft had
            # left, so zone dwell over it can read 0 for a flight that
            # crossed a zone: the verdict says how much went unobserved.
            n_pose_stale = sum(1 for r in self.rows if r.get("pose_stale"))
            pose_stale_s = round(n_pose_stale * TICK, 2)
            # A setpoint still unsent at the end is an open delivery gap.
            self.delivery.close(time.monotonic())
            if self.delivery.n_gaps:
                self.get_logger().error(
                    f"setpoint delivery: {self.delivery.n_gaps} gap(s), "
                    f"{self.delivery.gap_s_total:.1f} s in all, in which the "
                    f"adapter sent none of the Shield's setpoints (metrics.json "
                    f"setpoint_delivery)")
            d_short = self.delivery.n_published - self.delivery.n_reports
            if d_short > DELIVERY_SHORTFALL_MAX:
                self.get_logger().error(
                    f"setpoint delivery: {d_short} of {self.delivery.n_published} "
                    f"published setpoints had no adapter send report (metrics.json "
                    f"setpoint_delivery.shortfall)")
            if n_pose_stale:
                kpi += (f" over fresh-pose ticks only ({pose_stale_s:.1f} s on a "
                        f"stale pose, not observed)")

            try:
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.pyplot as plt
                fig, ax = plt.subplots(figsize=(7, 7))
                for f, poly in fences:
                    dyn = f.id == self.dynamic_fence_id
                    xs, ys = poly.exterior.xy
                    ax.fill(ys, xs, alpha=0.25, color="purple" if dyn else "red",
                            label="dynamic NFZ" if dyn else f"NFZ {f.id}")
                if self.spawn_pos:
                    ax.plot(self.spawn_pos[1], self.spawn_pos[0], "X",
                            color="purple", markersize=12)
                if self.traj:
                    ax.plot([p["y"] for p in self.traj], [p["x"] for p in self.traj],
                            "-", color="tab:blue", linewidth=2, label="flight path")
                    tx = [p["y"] for p in self.traj if p["touched"]]
                    if tx:
                        ax.plot(tx, [p["x"] for p in self.traj if p["touched"]], ".",
                                color="orange", markersize=4, label="shield active")
                    ax.plot(self.traj[0]["y"], self.traj[0]["x"], "go",
                            markersize=10, label="start")
                if self.has_target:
                    ax.plot(self.mission.target_y, self.mission.target_x, "k*",
                            markersize=16, label="target")
                ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
                ax.set_title(f"ROS 2 + MAVROS rail - shield "
                             f"{'ON' if self.shield_on else 'OFF'}\n"
                             f"NFZ time: {nfz_s:.1f}s | "
                             f"{'no target' if not self.has_target else 'reached' if self.reached else 'NOT reached'}")
                ax.legend(loc="upper left", fontsize=9)
                ax.set_aspect("equal"); ax.grid(alpha=0.3)
                fig.tight_layout(); fig.savefig(self.out / "trajectory.png", dpi=130)
            except ImportError:
                pass

            (self.out / "trajectory.json").write_text(json.dumps(self.traj),
                                                      encoding="utf-8")

            envelopes = self.policy.by_type(AltitudeEnvelope)
            alt_bad = alt_dwell_ticks(self.traj, envelopes)
            alt_s = alt_bad * TICK
            shield_ms = [r["shield_ms"] for r in self.rows
                         if isinstance(r.get("shield_ms"), (int, float))]
            dur = self._t() or 0.0
            # A VLA bridge's identity carries the server's weights digest
            # (pilot_from_identity); the stub's is filed as before.
            model_id, pilot, reported_digest = pilot_from_identity(self.vla_identity)

            metrics = {
                "tag": self.out.name,
                "episode_id": self.rec.episode_id,
                "ticks": len(self.rows),
                # Ticks on which the VLA's (or the Shield's) action was sent,
                # and those on which the autopilot or a fault held the
                # aircraft and nothing was (flown: false in the row).
                "ticks_flown": sum(1 for r in self.rows if r.get("flown")),
                "ticks_not_flown": sum(1 for r in self.rows
                                       if r.get("flown") is False),
                "shield": "on" if self.shield_on else "off",
                "shield_lookahead": {"horizon_s": LOOKAHEAD_S,
                                     "dt_s": LOOKAHEAD_DT_S,
                                     "poses": int(round(LOOKAHEAD_S
                                                        / LOOKAHEAD_DT_S))},
                # Which forecast the Shield flew (docs/DESIGN-own-velocity-
                # forecast.md; OWN_VELOCITY_DEFAULT is off on this rail).
                "shield_forecast": {
                    "mode": "own_velocity" if self.own_v else "commanded",
                    "flag": ("--own-velocity-forecast "
                             + ("on" if self.own_v else "off")),
                    "response": (self.shield.velocity_response.to_dict()
                                 if self.shield.velocity_response else None),
                    "velocity_source": "/mavros/local_position/velocity_local",
                    "velocity_ticks_seen": self.n_vel_seen,
                    "velocity_ticks": self.n_vel_used,
                    "shield_velocity_ticks": self.shield.n_velocity_ticks,
                    "velocity_nonfinite": self.shield.n_velocity_nonfinite,
                    # --arc-forecast: a body action with a yaw rate
                    # forecast along its arc (Shield.filter_body).
                    "arc": bool(getattr(self.shield, "arc_forecast", False)),
                    # --path-repair (WP3-06, 2026-11-17).
                    "path_repair": bool(getattr(self.shield, "path_repair", False))},
                # None with no target (--no-target): no goal was declared,
                # so none was missed either.
                "reached": self.reached if self.has_target else None,
                # compute() reads this as "did the mission happen"; a waypoint
                # mission expresses reaching the target in its units.
                "frac_within_30m": ((1.0 if self.reached else 0.0)
                                    if self.has_target else None),
                "reach_m": self.reach_m if self.has_target else None,
                "nfz_s": round(nfz_s, 2),
                "nfz_by_zone_s": self.dwell["nfz_by_zone_s"],
                "nfz_moving_not_counted": self.dwell["nfz_moving_not_counted"],
                "nfz_off_by_clock_s": self.dwell["nfz_off_by_clock_s"],
                **self._standoff_metrics(),
                "alt_violation_s": round(alt_s, 2),
                "interventions": self.n_touched,
                "brakes": self.n_braked,
                "frame": "body (grant Architecture constraints p2)",
                "seed": self.args.seed,
                "target": ([self.mission.target_x, self.mission.target_y]
                           if self.has_target else None),
                "speed_pref_mps": self.mission.speed_pref_mps,
                # A scenario's start (--start): where the autopilot's home is
                # in the policy's frame, the altitude the mission started at,
                # and the check of the autopilot's own home against it.
                "start": (None if not self.start_given else
                          {"policy_ne": [self.start_xy[0], self.start_xy[1]],
                           "up": self.start_up, "source": "--start"}),
                "home_check": self.home_check,
                # The stress harness's scenario (--scenario-meta), its KPI
                # report row, and its mid-flight events (--events).
                "scenario": self.scenario_meta,
                "scenario_family": (self.scenario_meta or {}).get("family"),
                # The declared world the Shield judged a distance_envelope in
                # (--world; None: no map was given).
                "world": self.world.record() if self.world else None,
                # Where the Shield was told about people (a distance_envelope
                # over people only; guardrail/people.py).
                **({} if self.shield.people_source is None
                   else {"people_source": self.shield.people_source}),
                "scenario_events": (None if not self._ev_file else {
                    "file": self._ev_file, "declared": self._ev_declared,
                    "applied": [r for r in self._ev_records if not r.get("refused")],
                    "refused": [r for r in self._ev_records if r.get("refused")],
                    "unexpected_refusals": self._ev_unexpected,
                    "not_applied": [{"index": i, "event": e.type, "at_sim_t": e.at_sim_t}
                                    for i, e in self._ev_pending],
                    # The harness's gate: None when nothing was refused, else
                    # whether every refusal was expected and nothing expected
                    # refused was accepted (sweep_scenarios event_refusals_
                    # as_expected).
                    "event_refusals_as_expected": (
                        None if not (self._ev_state.refused or self._ev_unexpected)
                        else (not self._ev_unexpected
                              and all(exp for _, _, exp in self._ev_state.refused))),
                    "live_from": dict(self._ev_live_from),
                    "shield_time": "t=None (the Shield's own last tick), as REST"}),
                "pilot": pilot,
                "vla_identity": self.vla_identity,
                "vla_actions_received": self.n_actions,
                # The decision behind the actions (the VLA bridge's extended
                # /vla/action_4d; nothing to judge for the stub's four values).
                "vla_pilot": {
                    "extended_messages": self.n_pilot_msgs,
                    # Refused messages: not flown, not receipts (P1-A).
                    "malformed_tails": self.n_pilot_malformed,
                    "last_malformed": self.pilot_error,
                    "layout": list(VLA_ACTION_FIELDS) if self.n_pilot_msgs else None,
                    "decision_watchdog": self.decision_watchdog.summary(
                        lambda v: v - self.mission_t0),
                    "decision_hold_max_s": self.decision_hold_max_s,
                    # None: the ROS stack could not say (no count_publishers).
                    "action_publishers_max": self.action_publishers_max,
                    "action_publishers_flagged": self.action_publishers_flagged,
                    "action_publishers_checks_to_flag": ACTION_PUBLISHERS_CHECKS,
                    "action_publisher_names": sorted(self.action_publisher_names),
                    "contract_problems": self.contract_problems,
                    "decision_spells_max": DECISION_SPELLS_MAX,
                    "decision_spells_window_s": DECISION_SPELLS_WINDOW_S,
                    "decision_escalation": self.decision_escalation,
                    "land_request": self.vla_land,
                },
                "vla_ready": self.vla_ready,
                "pose_hz": (round(self.n_pose / dur, 2) if dur > 0 else None),
                "shield_ms_p50": percentile(shield_ms, 50),
                "shield_ms_p99": percentile(shield_ms, 99),
                "shield_ms_max": (round(max(shield_ms), 3) if shield_ms else None),
                "fsm": ({**self.fsm.summary(),
                         "theta_basis": theta_basis(self.theta_horizon_s),
                         "decision_period": self.decision_period,
                         "fault": self.fsm_fault,
                         # Where the holds were flown (--hold-points) and how
                         # many moved points were flown as a GUIDED transit.
                         "hold_points": {"choice": ("moved" if self.moved_holds
                                                    else "in_place"),
                                         "moved_transits": self.hold_flight.transits}}
                        if self.fsm else None),
                # G6's input (2026-11-14; module docstring, THE RTL WATCH):
                # how the watch was configured, what it judged and why each
                # RTL failed; the autopilot's battery reports behind it.
                # Present on every run of this node, so the stress harness
                # reads rtl_failed_ticks as measured (0 included).
                "rtl_watch": ({**self.rtl_watch.summary(),
                               "params": self.rtl_params,
                               "battery": {
                                   "topic": MAVLINK_SOURCE_TOPIC,
                                   "source": ("BATTERY_STATUS (MAVLink 147) charge_state "
                                              "from the autopilot (system 1, component 1) "
                                              "in MAVROS's raw feed"),
                                   "note": battery_failsafe_note(self.rtl_params or {}),
                                   "refused": self.inputs_refused.get("estimator"),
                                   **self.battery.summary(
                                       to_mission_t=lambda v: v - (self.mission_t0 or 0.0))},
                               "ticks_rtl_failed": sum(1 for r in self.rows
                                                       if r.get("rtl_failed") is True)}
                              if self.fsm else None),
                "autopilot_took_over": self.takeover,
                # After the FSM's terminal edge the rail logged on to the
                # landing (2026-11-07): when the edge came, and how it ended
                # (landed / disarmed; None: the time cap, or no terminal edge).
                "after_terminal": (self.after_terminal if self.after_terminal is not None
                                   else ({"terminal_t": round(self._terminal_t, 3),
                                          "ended": None}
                                         if self._terminal_t is not None else None)),
                # Both vocabularies (mavlink_adapter_node.ModeRequests): the
                # FSM's words, and what ArduCopter was asked for (a request
                # through the adapter is flown as the adapter's hold mode).
                "mode_requests": list(self._asks.fsm),
                "autopilot_mode_requests": list(self._asks.autopilot),
                "fence_upload": (self.fence_status[-1] if self.fence_status else None),
                "fence_uploads": self.fence_status,
                "fences_still_deferred": self.deferred_fences,
                "zones_over_home": self.zones_over_home,
                "autopilot_fence": self.autopilot_fence or None,
                "generations": self.rec.generations,
                # Which clock the rules were evaluated on (scripted or wall).
                "rule_clock": self.clock_record,
                "hot_apply": (self.hot.summary() if self.hot is not None
                              else None),
                "action_watchdog": self.watchdog.summary(
                    lambda v: v - self.mission_t0),
                # Ticks whose checks ran on a position past the pose timeout:
                # zone dwell over them describes where the aircraft WAS.
                "pose_watchdog": {**self.pose_watchdog.summary(
                    lambda v: v - self.mission_t0),
                    "stale_ticks": n_pose_stale,
                    "stale_s": pose_stale_s},
                # Setpoints published against setpoints the adapter sent
                # (SETPOINT DELIVERY): a gap here keeps the run from KPI grade
                # (guardrail.manifest.is_kpi_grade reads this key).
                "setpoint_delivery": {
                    "topic": TOPIC_SENT,
                    **self.delivery.summary(lambda v: v - (self.mission_t0 or 0.0))},
                "tick_latency": latency_summary(self.rows, trigger=self.tick_trigger),
                # What ran the ticks and how often (THE TICK SCHEDULE).
                "tick_schedule": tick_schedule_summary(self.rows, self.tick_trigger,
                                                       self._sched, self.start_backlog),
                # The camera frame's age behind each setpoint flown from a
                # camera-backed action (THE CAMERA'S AGE AT THE SETPOINT).
                "camera_to_setpoint": camera_latency_summary(self.rows,
                                                             self.n_pilot_msgs),
                # The collector's pacing and every collection it saw (THE
                # GARBAGE COLLECTOR).
                "gc": self.gc_pacer.summary(),
                # The GPS fix behind State.fix_ok (2026-11-07; module
                # docstring, "THE GPS FIX AND THE SIMULATOR'S STRESSORS").
                "fix_quality": {
                    "topic": FIX_TOPIC,
                    "source": ("sensor_msgs/NavSatFix status MAVROS 2's global_position "
                               "plugin sets from GPS_RAW_INT (fix_type > 2: STATUS_FIX)"),
                    "messages": self.n_fix,
                    "stale_after_s": FIX_STALE_S,
                    "ticks_fix_ok": sum(1 for r in self.rows if r.get("fix_ok") is True),
                    "ticks_no_fix": sum(1 for r in self.rows if r.get("fix_ok") is False),
                    "ticks_not_reported": sum(1 for r in self.rows
                                              if r.get("fix_ok") is None),
                    "no_fix_spells": fix_spells(self.rows),
                    "position_lost_ticks": self.n_position_lost,
                    # The autopilot's own verdict on its estimate (2026-11-12;
                    # module docstring, "THE AUTOPILOT'S ESTIMATOR").
                    "estimator": {
                        "topic": MAVLINK_SOURCE_TOPIC,
                        "source": ("EKF_STATUS_REPORT (MAVLink 193) from the autopilot "
                                   "(system 1, component 1) in MAVROS's raw feed"),
                        "healthy_when": ("EKF_POS_HORIZ_ABS set, EKF_CONST_POS_MODE and "
                                         "EKF_UNINITIALIZED clear, pos_horiz and velocity "
                                         "test ratios below the threshold"),
                        # Why no report arrived, when the graph refused the
                        # subscription (an SROS 2 keystore signed before the
                        # policy granted it); None otherwise.
                        "refused": self.inputs_refused.get("estimator"),
                        **self.estimator.summary(
                            to_mission_t=lambda v: v - (self.mission_t0 or 0.0)),
                        "ticks_ok": sum(1 for r in self.rows
                                        if r.get("estimator_ok") is True),
                        "ticks_lost": sum(1 for r in self.rows
                                          if r.get("estimator_ok") is False),
                        "ticks_not_reported": sum(1 for r in self.rows
                                                  if r.get("estimator_ok") is None),
                    },
                    "pos_sigma_m": None,
                    "pos_sigma_m_why": ("not fed: with 1 m margins X8's sigma branch "
                                        "needs sigma <= 0.33 m (SITL reports 0.3 m, a "
                                        "field receiver metres, which would return "
                                        "every mission passing near a zone); sizing "
                                        "margins to the rail's accuracy is a policy "
                                        "decision left open")},
                # SITL's stressor parameters (set by the simulator side,
                # sitl/sitl_stressors.py): read back at take-off, every change
                # the autopilot broadcast on this node's mission clock, and
                # the t = 0 markers this node announced its clock with.
                "sim_params": {
                    "watched": list(WATCHED_SIM_PARAMS),
                    "at_takeoff": self.sim_params_takeoff,
                    "events": self.sim_param_events,
                    "events_total": self.n_sim_param_events,
                    "events_topic": PARAM_EVENT_TOPIC,
                    "markers_sent": self.markers_sent,
                    "marker_topic": STATUSTEXT_SEND_TOPIC if self.pub_text else None,
                    # Inputs the ROS graph refused this node (an SROS 2
                    # keystore made before they were granted): {} normally.
                    "inputs_refused": self.inputs_refused},
                # Beside the manifest, which is exactly the grant's six fields.
                "hil_evidence": self.hil_evidence,
                "policy_source": self.policy_source,
            }
            (self.out / "flight_log.jsonl").write_text(
                "".join(json.dumps(r) + "\n" for r in self.rows), encoding="utf-8")

            # The topology the evidence supports, or the refusal recorded and
            # the rail's own label (mavros_evidence.topology_manifest).
            speedup = self.sim_speedup
            manifest, refused = topology_manifest(
                policy_hash=self.policy.policy_hash, model_id=model_id,
                seed=self.args.seed, sim_speedup=speedup, evidence=self.hil_evidence,
                reported_digest=reported_digest, warn=self.get_logger().warn)
            if refused is not None:
                metrics["topology_refused"] = refused
            metrics["topology"] = manifest["topology"]
            if self.ground is not None:
                # The ground the Shield converted MSL rules over (--ground,
                # X-11): its file and hash, and over a DEM the home elevation
                # the heights were taken from. Absent without --ground.
                metrics["ground"] = (self.terrain.record() if self.terrain is not None
                                     else {"ground": self.ground.record(),
                                           "up_ref": "home"})
                metrics["ground"]["file"] = dict(self.ground_file)
            (self.out / "metrics.json").write_text(json.dumps(metrics, indent=2),
                                                   encoding="utf-8")
            (self.out / "manifest.json").write_text(json.dumps(manifest, indent=2),
                                                    encoding="utf-8")
            # Beside the six fields: this host's revision, the autopilot's
            # firmware and the simulator build (ARCH-31; the hil harness adds
            # the desktop's after it fetches the run).
            write_stack_revisions(self.out, manifest, self.hil_evidence,
                                  warn=self.get_logger().warn)

            kpi_res = compute_from_dir(self.out, self.policy)
            graded, why = is_kpi_grade(manifest, {**metrics, "det_hz": None,
                                                  "start_heading_err_deg": None})
            if self.fsm_fault:
                graded, why = False, why + [f"FSM fault: {self.fsm_fault}"]
            if self.pose_watchdog.spells:
                graded, why = False, why + [
                    f"the pose stream went stale ({len(self.pose_watchdog.spells)} "
                    f"spell(s) over {self.pose_watchdog.timeout_s:g} s, "
                    f"{pose_stale_s:g} s of ticks): the checks on those ticks did "
                    f"not see the aircraft"]
            if self.policy_source.get("dev_yaml"):
                graded, why = False, why + [
                    f"the policy was flown from YAML under {DEV_YAML_FLAG}: "
                    f"unsigned, not its issued bundle"]
            if self.action_publishers_flagged:
                # Two hands on the stick: whatever flew is not one pilot's.
                graded, why = False, why + [
                    f"{self.action_publishers_max} publishers on {VLA_ACTION_TOPIC} "
                    f"({', '.join(sorted(self.action_publisher_names)) or 'names unknown'}) "
                    f"on {ACTION_PUBLISHERS_CHECKS} consecutive checks: the actions flown "
                    f"are not one pilot's"]
            kpi_res["kpi_grade"] = graded
            kpi_res["kpi_grade_reasons"] = why
            kpi_res["manifest"] = manifest
            kpi_res["policy_source"] = self.policy_source
            kpi_res["episode_id"] = self.rec.episode_id
            kpi_path = self.out / "kpi.json"
            kpi_path.write_text(json.dumps(kpi_res, indent=2), encoding="utf-8")

            # The bundle, while the policy that governed the flight is still in
            # memory. The rosbag is still open here, so it is left out;
            # run_ros2_demo.sh re-packs (python -m guardrail.replay pack) once
            # the bag is closed.
            rb_path = self.out / f"{self.out.name}.replay.tar.gz"
            try:
                rb = write_replay(self.out, self.policy, rb_path,
                                  changelog=f"ros2 {self.out.name}",
                                  policy_source=self.policy_source)
                ok, notes = verify_replay(rb)
                self.get_logger().info(
                    f"[replay] {rb.name} "
                    f"{'re-derives its own KPIs' if ok else 'FAILED verification'}")
                for n_ in notes:
                    self.get_logger().info(f"[replay]   - {n_}")
                # KPI evidence needs a SIGNED bundle: a graded run whose
                # bundle is keyless is demoted, and kpi.json and the bundle
                # are rewritten to say so.
                graded2, why2 = kpi_evidence_grade(graded, why, rb)
                if graded2 != graded:
                    graded, why = graded2, why2
                    kpi_res["kpi_grade"], kpi_res["kpi_grade_reasons"] = graded, why
                    kpi_path.write_text(json.dumps(kpi_res, indent=2),
                                        encoding="utf-8")
                    write_replay(self.out, self.policy, rb_path,
                                 changelog=f"ros2 {self.out.name}",
                                 policy_source=self.policy_source)
            except Exception as exc:                            # noqa: BLE001
                self.get_logger().warn(
                    f"[replay] not written: {type(exc).__name__}: {exc}")
                if graded:
                    graded, why = False, why + [f"no replay bundle: {exc}"]
                    kpi_res["kpi_grade"], kpi_res["kpi_grade_reasons"] = graded, why
                    kpi_path.write_text(json.dumps(kpi_res, indent=2),
                                        encoding="utf-8")
            self.get_logger().info(
                f"P0 escape rate {kpi_res['p0_violation_escape_rate']} | "
                f"NFZ {nfz_s:.1f}s | alt {alt_s:.1f}s | interventions "
                f"{self.n_touched} | topology {manifest['topology']}")
            self.get_logger().info(
                f"KPI-GRADE: {'YES' if graded else 'no'}"
                + ("" if graded else "  (" + "; ".join(why) + ")"))
            fsm_line = (f"{self.fsm.summary()['max_state']} (max state)"
                        if self.fsm else "not run (control arm)")
            (self.out / "report.md").write_text(
                f"""# ROS 2 rail report - shield {'ON' if self.shield_on else 'OFF'}

| Item | Value |
|------|-------|
| Path | vla node -> /vla/action_4d (body) -> safety_shield -> /shield/setpoint (body) -> mavlink_adapter -> /mavros/setpoint_raw/local -> MAVROS 2 -> ArduPilot |
| Autopilot | {self.hil_evidence.get('ardupilot_version')} ({self.hil_evidence.get('ardupilot_version_source')}) |
| Pilot | `{model_id}` |
| Policy | `{self.policy.policy_id}` `{self.policy.policy_hash}` |
| Policy source | {self.policy_source['kind']} `{self.policy_source['path']}` - signature **{self.policy_source['signature']}**{' (--dev-yaml)' if self.policy_source.get('dev_yaml') else ''} |
| Rule clock | {self.clock_record.get('kind')}: {self.clock_record.get('start') or self.clock_record.get('tz')} |
| Hot-apply API | {f"{metrics['hot_apply']['n_applied']} event(s) at {metrics['hot_apply']['endpoint']['url']}, receipt to swap max {metrics['hot_apply']['receipt_to_swap_ms_max']} ms" if self.hot is not None else 'not served'} |
| Action stream | {self.n_actions} received; {self.watchdog.summary()['n_spells']} stale spell(s) (timeout {self.watchdog.timeout_s:g} s) |
| Pose stream | {metrics['pose_hz']} Hz; {len(self.pose_watchdog.spells)} stale spell(s) (timeout {self.pose_watchdog.timeout_s:g} s), {pose_stale_s:g} s of ticks on a stale pose |
| GPS fix (`fix_ok`, {FIX_TOPIC}) | {f"{self.n_fix} message(s); {metrics['fix_quality']['ticks_no_fix']} tick(s) without a fix; X8 input on {self.n_position_lost} tick(s)" if self.n_fix else 'not reported (no message): fix_ok None, as before'} |
| Autopilot estimator (`estimator_ok`, EKF_STATUS_REPORT on {MAVLINK_SOURCE_TOPIC}) | {f"{self.estimator.n} report(s), {self.estimator.n_unhealthy} unhealthy; {metrics['fix_quality']['estimator']['ticks_lost']} tick(s) declared lost (threshold {self.estimator.thresh:g}, {self.estimator.thresh_source})" if self.estimator.n else 'not reported (no report): estimator_ok None, fix_ok alone decides'} |
| Simulator parameters | {f"at take-off {self.sim_params_takeoff.get('values')}; {sum(1 for e in self.sim_param_events if e['t'] is not None)} change(s) in flight" if (self.sim_params_takeoff or {}).get('sitl') else 'not SITL, or not read'} |
| Tick latency (e2e p50 / p99 / max) | {metrics['tick_latency']['e2e_ms']['p50']} / {metrics['tick_latency']['e2e_ms']['p99']} / {metrics['tick_latency']['e2e_ms']['max']} ms, budget {TICK_BUDGET_MS:g} ms |
| Topology | `{manifest['topology']}` |
| Target reached | {'no target (--no-target)' if not self.has_target else 'yes' if self.reached else 'NO'} |
| Scenario | {(self.scenario_meta or {}).get('run_id') or 'none'}{f", {sum(1 for r in self._ev_records if not r.get('refused'))} of {self._ev_declared} event(s) applied" if self._ev_file else ''} |
| Ticks | {len(self.traj)} ({metrics['ticks_flown']} flown, {metrics['ticks_not_flown']} held by the autopilot or a fault) |
| Shield lookahead | {LOOKAHEAD_S:g} s at {LOOKAHEAD_DT_S:g} s ({metrics['shield_lookahead']['poses']} poses) |
| Shield interventions | {self.n_touched} |
| Escalation FSM | {fsm_line} |
| Autopilot took over | {self.takeover or 'no'} |
| Autopilot fence parameters | {self.autopilot_fence or 'not read'} |
| Autopilot fence upload | {(self.fence_status[-1] if self.fence_status else 'no upload reported')} |
| Dynamic NFZ | {f'hot-applied t={self.spawn_t:.1f}s, generation {self.policy.generation}' if self.spawn_t else 'not used'} |
| **Time inside NFZ** | **{nfz_s:.1f} s** |
| **P0 KPI** | **{kpi}** |
""", encoding="utf-8")
            print(f"[report] NFZ {nfz_s:.1f}s -> {kpi} | touched {self.n_touched} "
                  f"| braked {self.n_braked} | ticks {len(self.traj)}")


def make_parser() -> argparse.ArgumentParser:
    # XYArgumentParser: `--target -20,15` (negative X) parses as a value
    # instead of exiting with status 2 (mavlink_adapter_node.join_negative_xy).
    # allow_abbrev off (2026-11-06), as the VLA bridge's: the rail hands
    # every argument to this node and the VLA node alike, and the stub's
    # `--action=-3,0,0,0` was read here as an abbreviation of
    # --action-timeout-s - an argparse error, the mission never started.
    ap = XYArgumentParser(allow_abbrev=False)
    ap.add_argument("--shield", choices=["on", "off"], default="on")
    ap.add_argument("--dynamic", action="store_true")
    ap.add_argument("--dynamic-zone", choices=["route", "on-aircraft"],
                    default="route",
                    help="with --dynamic: 'route' hot-applies the 8 x 8 m zone "
                         "across the route at 8 s; 'on-aircraft' a 10 x 10 m "
                         "zone centred on the aircraft, so the run starts "
                         "unsafe and time to safe is measured")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--policy", default=None,
                    help="policy YAML; flown only with --dev-yaml, recorded as "
                         "UNSIGNED. Without --policy and --bundle the signed "
                         "bundle of policies/sim_demo_policy.yaml flies")
    ap.add_argument("--bundle", default=None,
                    help="signed policy bundle. Refused unless its signature "
                         "verifies.")
    ap.add_argument("--allow-unverified-bundle", action="store_true")
    ap.add_argument("--expected-hash", default=None, metavar="SHA256",
                    help="refuse to fly unless the loaded policy has this hash "
                         "(the stress harness pins every bundle it sends)")
    ap.add_argument(DEV_YAML_FLAG, dest="dev_yaml", action="store_true",
                    help="fly a YAML policy (--policy, or the default YAML): a "
                         "development run, recorded as such and never "
                         "KPI-grade")
    ap.add_argument("--clock-start", default=None, metavar="ISO8601",
                    help="script the rule clock: it reads this at mission start "
                         "and advances with the mission time. Without it the "
                         "wall clock in the mission's time zone (--clock-tz)")
    ap.add_argument("--clock-tz", default=None,
                    help="the mission's time zone for the wall rule clock: a UTC "
                         "offset (+08:00, the default: Taiwan) or an IANA name")
    ap.add_argument("--hotapply-port", type=int, default=None,
                    help="serve the REST hot-apply API (guardrail/api.py) on "
                         "this port, started on the pad before arming")
    ap.add_argument("--hotapply-host", default=DEFAULT_HOTAPPLY_HOST,
                    help="address for --hotapply-port (default 127.0.0.1; any "
                         "other needs --hotapply-pki)")
    ap.add_argument("--hotapply-pki", default=None, metavar="DIR",
                    help="mutual TLS with the harness CA: a folder written by "
                         "python -m guardrail.harness_service gen-pki")
    ap.add_argument("--hotapply-server-cert", default=None, metavar="NAME",
                    help="the server certificate's name in --hotapply-pki "
                         "(default harness-server)")
    ap.add_argument("--hotapply-client", action="append", default=None,
                    metavar="CN",
                    help="client certificate common name allowed in (repeat; "
                         "default harness-client)")
    ap.add_argument("--action-timeout-s", type=float, default=ACTION_TIMEOUT_S,
                    help="hold the aircraft once /vla/action_4d has been quiet "
                         f"this long (default {ACTION_TIMEOUT_S:g} s: "
                         f"{ACTION_TIMEOUT_TICKS} ticks at 10 Hz)")
    ap.add_argument("--decision-timeout-s", type=float, default=DECISION_TIMEOUT_S,
                    help="hold the aircraft while the VLA bridge flies a step whose "
                         f"decision is older than this plus {DECISION_GRACE_S:g} s "
                         f"(default {DECISION_TIMEOUT_S:g} s, its step tracker's own "
                         "timeout; give both nodes the same value). Inert for a pilot "
                         "that sends the grant's four values")
    ap.add_argument("--decision-hold-max-s", type=float, default=DECISION_HOLD_MAX_S,
                    help="end the mission (mission-end LAND) once the VLA bridge's "
                         f"actions have been held this long (default "
                         f"{DECISION_HOLD_MAX_S:g} s): a broken pilot is not waited for")
    ap.add_argument("--vla-ready-timeout-s", type=float, default=0.0,
                    help="before arming, wait this long for a /vla/identity saying "
                         "ready (the VLA bridge's warm-up passed) and refuse to arm "
                         "without one (default 0: no wait, the stub's start-up)")
    ap.add_argument("--pose-timeout-s", type=float, default=POSE_TIMEOUT_S,
                    help="hold the aircraft once /mavros/local_position/pose "
                         f"has not been updated this long (default "
                         f"{POSE_TIMEOUT_S:g} s; keep it well under the Shield's "
                         f"{LOOKAHEAD_S:g} s lookahead)")
    ap.add_argument("--tick-trigger", choices=["action", "timer"], default="action",
                    help="what runs the control tick: 'action' (default) a pilot "
                         "message as it arrives, with a 10 Hz watchdog for a quiet "
                         "stream; 'timer' the node's own 10 Hz timer, as every run "
                         "until 2026-11-11 (module docstring, TICK LATENCY)")
    ap.add_argument("--gc-pacing", choices=["on", "off"], default="on",
                    help="'on' (default): freeze the heap at mission start and run "
                         "Python's full garbage collection only right after a "
                         f"setpoint is published, every {GC_FULL_EVERY_S:g} s; 'off': "
                         "CPython's own schedule (collections are counted either way)")
    ap.add_argument("--own-velocity-forecast", choices=["on", "off"],
                    default=OWN_VELOCITY_DEFAULT,
                    help="give the Shield /mavros/local_position/velocity_local, so "
                         "its forecast starts from the measured velocity through "
                         "ArduCopter's measured response (default off on this rail: "
                         "that response was measured on the Project AirSim quad, not "
                         "on ArduPilot's own SITL physics; the velocity is logged "
                         "either way)")
    ap.add_argument("--path-repair", choices=["on", "off"], default="on",
                    help="on (default since 2026-11-17, WP3-06): the Shield's chain "
                         "runs PathRepair after LateralProjection, the grant's order; "
                         "in chain mode it detours round a --subject's stand-off ring "
                         "only. off: the chain every run before 2026-11-17 flew")
    ap.add_argument("--arc-forecast", choices=["on", "off"], default="off",
                    help="on: the Shield forecasts a body action with a yaw rate along "
                         "the arc it flies (velocity held in the body frame while the "
                         "heading turns; Shield.filter_body, WP3-04 / ARCH-04); off "
                         "(default, every flight before 2026-11-17): along its world "
                         "direction at t = 0. A zero yaw rate (the stub pilot) is "
                         "forecast identically either way")
    ap.add_argument("--command", default=None,
                    help="mission text (default: fly to the northeast pad at 6 m/s)")
    ap.add_argument("--target", default=None, metavar="X,Y[,UP]",
                    help="override the mission target (policy-frame metres North, "
                         "East; UP its altitude, the mission's cruise altitude), so "
                         "the rail can fly a stress scenario's mission; a "
                         "negative X parses either way (-20,15 or =-20,15)")
    ap.add_argument("--no-target", action="store_true",
                    help="the mission has no target (a stress scenario's constant "
                         "pilot): no reach test, `reached` null; it flies to --max-s")
    ap.add_argument("--reach-m", type=float, default=REACH_M,
                    help=f"the target counts as reached within this (default "
                         f"{REACH_M:g} m; the stress harness passes its goal "
                         f"tolerance, 3 m)")
    ap.add_argument("--start", default=None, metavar="X,Y[,UP]",
                    help="a stress scenario's start: the policy-frame position of the "
                         "autopilot's home (every pose is offset by it; the node "
                         "refuses to arm unless the autopilot's home lies within "
                         f"{HOME_TOL_M:g} m of it), and UP the altitude the mission "
                         "starts at (take-off climbs to it and settles)")
    ap.add_argument("--scenario-meta", default=None, metavar="FILE",
                    help="the stress harness's record of the scenario flown (JSON), "
                         "written into metrics.json as `scenario` and its `family` "
                         "as `scenario_family`")
    ap.add_argument("--events", default=None, metavar="FILE",
                    help="the scenario's mid-flight events (a JSON list of "
                         "guardrail.scenario_spec.ScenarioEvent), each applied to "
                         "the flying Shield once the mission time reaches its "
                         "at_sim_t (guardrail.scenario_events)")
    ap.add_argument("--world", default=None, metavar="FILE",
                    help="a declared world (guardrail/world_map.py, repo-relative): the "
                         "buildings and roads maps the Shield measures a "
                         "distance_envelope on (WP1-07). Default: no map, as before "
                         "2026-11-17; a policy whose distance_envelope needs one is "
                         "refused before take-off")
    ap.add_argument("--ground", default=None, metavar="FILE",
                    help="a declared ground (guardrail/terrain.py load_ground, "
                         "repo-relative): a WGS84 elevation grid (JSON kind "
                         "wgs84_grid, or .npz), a metre grid or a flat ground. With "
                         "it an MSL rule is flyable: the Shield converts it over this "
                         "ground and, over a grid, is told the height above the grid "
                         "(X-11). Default: no ground, as before 2026-11-18; an MSL "
                         "rule is then refused before take-off")
    ap.add_argument("--ground-home-msl", type=float, default=None, metavar="M",
                    help="with a --ground grid: the elevation (m MSL) of the "
                         "autopilot's home / EKF origin (the SITL rails: 584). Must "
                         "agree with the grid's elevation at home within "
                         "terrain.GROUND_HOME_TOL_M. Default: the grid's own "
                         "elevation at home")
    ap.add_argument("--ground-sha256", default=None, metavar="HEX",
                    help="the sha256 of the --ground file (64 hex digits): the file "
                         "must hash to it or the flight is refused before take-off "
                         "(terrain.ground_file; the stress harness always sends it). "
                         "Default: unpinned; the file's hash is recorded either way")
    ap.add_argument("--speed", type=float, default=None,
                    help="override the pilot's preferred speed, m/s")
    ap.add_argument("--seed", type=int, default=0,
                    help="recorded in the manifest. StubVLA itself draws no "
                         "random numbers.")
    ap.add_argument("--max-s", type=float, default=MAX_S,
                    help="mission time cap, seconds")
    ap.add_argument("--subject", default=None, metavar="X,Y",
                    help="declare a subject at this NED position so "
                         "subject_standoff rules bind (declared, not perceived)")
    ap.add_argument("--subject-class", default="pedestrian")
    ap.add_argument("--fsm-config", default=None,
                    help="escalation FSM thresholds YAML (guardrail.fsm.FSMConfig)")
    ap.add_argument("--decision-period-s", type=float, default=None,
                    help="the pilot's decision period, seconds: above T / (1.5 x N) "
                         "(1.11 s for the grant's N = 3, T = 5 s) the FSM's N-in-T window "
                         "is scaled to it (FSMConfig.for_decision_period); left out (the "
                         "default, the 10 Hz stub) or at most that, the thresholds are "
                         "unchanged")
    ap.add_argument("--theta-horizon-s", default="auto",
                    help="'auto' (the default basis, guardrail.fsm."
                         "DEFAULT_THETA_HORIZON_S: per-operator magnitude_m, the "
                         "forecast depth), 'none' (the same, explicitly), or seconds "
                         "(0.1 = the one-period reading: the setpoint change over one "
                         "control period; pending the PI)")
    ap.add_argument("--hold-mode", choices=list(HOLD_MODES), default=DEFAULT_HOLD_MODE,
                    help="ArduCopter mode the FSM's Loiter is flown as when no adapter "
                         "is subscribed (the adapter's own --hold-mode otherwise)")
    ap.add_argument("--hold-points", choices=["moved", "in_place"], default="moved",
                    help="where the FSM's Loiter holds and its Land lands: moved (the "
                         "hold-point search's legal point, flown as a GUIDED transit "
                         "and then the hold, guardrail/hold_flight.py; the default "
                         "since 2026-11-07) or in_place (ArduCopter's own hold where "
                         "the aircraft is, as before)")
    ap.add_argument("--hold-flight", choices=["guided", "brake"], default="guided",
                    help="how a Loiter at the search's point is flown (guardrail/"
                         "hold_flight.py): guided (held in GUIDED by this node on the "
                         "point, BRAKE only on a stale input or a stream gap; the "
                         "default since 2026-11-07 night) or brake (ArduCopter's "
                         "BRAKE on arrival, as before)")
    ap.add_argument("--cost-map", action="store_true",
                    help="give every generation's CSP its cost map (costmap_csp_g<N>"
                         ".npz/.json, named in cost_map_ref; guardrail.replay."
                         "EpisodeRecord)")
    return ap


def main(argv: list[str] | None = None) -> None:
    args, ros_args = make_parser().parse_known_args(argv)
    # Refused before ROS starts and before anything flies: a YAML policy
    # without --dev-yaml, no policy and no signed default bundle, a clock
    # start or zone that cannot be read, a timeout that is not one.
    try:
        resolve_policy(args)
        rule_clock(args.clock_start, elapsed=lambda: None, tz=args.clock_tz)
        ActionWatchdog(args.action_timeout_s)
        ActionWatchdog(args.pose_timeout_s, what="pose")
        DecisionWatchdog(args.decision_timeout_s)
        fsm_config_for_pilot(FSMConfig(), args.decision_period_s)
        if not (math.isfinite(args.decision_hold_max_s) and args.decision_hold_max_s > 0):
            raise ValueError(f"--decision-hold-max-s must be a positive number of seconds, "
                             f"got {args.decision_hold_max_s!r}")
        if not (math.isfinite(args.vla_ready_timeout_s) and args.vla_ready_timeout_s >= 0):
            raise ValueError(f"--vla-ready-timeout-s must be 0 or more seconds, "
                             f"got {args.vla_ready_timeout_s!r}")
        # A stress scenario's mission (2026-11-06): its start, target, reach,
        # record and events, each refused here rather than mid-air.
        parse_point(args.target, "--target")
        start = parse_point(args.start, "--start")
        if start is not None and start[2] is not None and not start[2] > 0:
            raise ValueError(f"--start altitude must be above home, got {start[2]!r}")
        if not (math.isfinite(args.reach_m) and args.reach_m > 0):
            raise ValueError(f"--reach-m must be a positive number of metres, "
                             f"got {args.reach_m!r}")
        if args.scenario_meta:
            load_scenario_meta(args.scenario_meta)
        if args.events:
            load_events(args.events)
        if args.world:
            load_world(args.world, ROOT)
        if args.ground_home_msl is not None and not args.ground:
            raise ValueError("--ground-home-msl is the home elevation of a --ground")
        if args.ground or args.ground_sha256:
            # Read and placed in the policy's frame before ROS starts; the
            # node reads it again with the policy (a bad file, a file that
            # is not the --ground-sha256 named, or a ground in another
            # frame, is refused with no node built).
            b, y, _ = resolve_policy(args)
            ground_from_args(args, b, y, root=ROOT)
    except (ValueError, OSError) as exc:
        raise SystemExit(f"[safety_shield] REFUSED: {exc}")
    if not HAVE_ROS:
        raise SystemExit("ros2_shield_node needs rclpy and mavros_msgs: source "
                         "/opt/ros/jazzy/setup.bash and use ~/venv-ros/bin/python")
    tag = args.tag or default_tag(args.shield, args.dynamic)
    out = ROOT / "demo" / "out" / tag
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = ShieldNode(args, out, subject=parse_xy(args.subject))
    try:
        node.bring_up()
        node.start_mission()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
        # keep spinning briefly so the LAND request goes out
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 3:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
