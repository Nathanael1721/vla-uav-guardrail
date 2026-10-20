# The Shield's own-velocity forecast, and a hold that stops clear

**Date:** 2026-10-19
**Status:** built and tested offline. `guardrail/response.py` (the response
model), `guardrail/shield.py` (the forecast, `note_flown`, `advise`,
`stop_breaks`), `guardrail/models.py` (`State.velocity`), the three ArduPilot
rails, `tools/fit_velocity_response.py` (the fit) and
`tools/own_velocity_closed_loop.py` (the closed loop);
`tests/test_own_velocity_forecast.py` at 16/16. **Not flown yet**: the
re-flight command is at the end.
**Revised 2026-10-20:** the measured velocity now only ADDS to the commanded
forecast (section 2, "The measured velocity only adds"). The first build
judged the own-velocity forecast alone, and in the closed loop it streamed
full-speed commands at the 14 Oct building from the edge of its margin;
re-checked on the commanded forecast, the basis of every stored P0 escape
count, those were 6 P0 escape ticks on the 14 Oct approach and 1 on the 13
Oct one, against the old Shield's 0. Sections 5, 6 and 8 carry the numbers
of the revised build.
**Grant terms kept:** the 5 s / 0.1 s forecast grid, the FSM's states and
edges G1-G8 (and X1-X7) as written, the P0 escape KPI's definition (the
Shield's re-check of what was flown). What changes is where the forecast
starts, and how the ArduPilot rail flies the FSM's Loiter.

## 1. The problem

`guardrail.models.State` had four fields (x, y, up, yaw). `Shield._predict`
advanced each candidate action at constant velocity from t = 0, so every
forecast pose assumed the aircraft took up the commanded velocity instantly.

The 14 Oct ArduPilot close approach (`demo/out/pas_ap_video_on_20261014`,
Shield on, BRAKE hold) shows what that costs. The position was inside the P0
building margin (`bld-clearance`, 3 m) on 71 of 517 ticks, 19-20 of them
while the Shield flew and the rest during the RTL climb, at most 1.0 m deep.
The flight log:

| tick | t (s) | clearance (m) | measured vy (m/s) | flown vy (m/s) | what happened |
|---|---|---|---|---|---|
| 363 | 40.73 | 4.92 | -3.2 | -0.01 | StandoffHold; FSM Normal -> Brake (G1) |
| 364 | 40.84 | 4.61 | -3.1 | +0.35 | first `bld-clearance` flag, ClearanceFix pushes away |
| 366 | 41.06 | 3.82 | -3.2 (noisy) | +0.38 | still at near full speed, ~0.35 s after the decision |
| 369 | 41.38 | 2.98 | -2.8 | +0.42 | enters the margin |
| 375 | 42.03 | 2.16 | -0.5 | +0.49 | decelerated at ~2-2.7 m/s^2 |
| 381 | 42.69 | 1.99 | -0.3 | +0.49 | deepest, 1.0 m in: ~0.7 s of creeping deeper against +0.49 commanded |
| 388 | 43.44 | 1.99 | 0.0 | - | X7 (losing ground) -> RTL; inside for 5.65 s more |

The Shield's forecast of its own push said "leaving the margin" from tick 364
on. The aircraft kept flying the setpoints already sent for about a third of
a second, then braked at about 2 m/s^2. The 13 Oct close run
(`pas_ap_close_on`) is the same approach with a second failure on top: the
FSM went Brake -> Loiter (G3) at tick 387 with the position still legal, the
rail asked ArduCopter for BRAKE, and BRAKE carried the aircraft 2.9 m on and
3.4 m into the margin before X7 sent it home; it was still moving at 1 m/s.

## 2. What changed

**`State.velocity`** (optional, `(vx North, vy East, vz_up)` m/s, the
autopilot's measured velocity; default None). A State without it dumps
exactly as before (a wrap serializer drops the None, as
`Policy.canonical_ir` drops absent fields). A non-finite velocity is not
refused by the model; the Shield ignores it for that tick and counts it
(`n_velocity_nonfinite`).

**The forecast** (`Shield._predict`, `velocity_response`, default
`response.ARDUCOPTER_GUIDED`). With a measured velocity, each pose flies,
for the measured reaction time (0.35 s), the setpoints the rail already sent
(`Shield.note_flown(action)`, called once per tick after the send; the
"pipe", `response.pipe_segments`), and then turns towards the candidate at
the measured acceleration limit (2.0 m/s^2), exactly integrated. A caller
that never calls `note_flown` gets the measured velocity kept for the
reaction time instead. Horizontal only: the climb rate stays the commanded
one (section 3). Same grid, same 50 poses.

**The measured velocity only adds** (2026-10-20). Every rule is judged on
the own-velocity forecast AND, as before the change, on the commanded
forecast from the same position; a rule is broken if either breaks it, at
the earlier time. `Shield._predict` yields both poses at each grid time (the
fence, stand-off, corridor and altitude tests take the first pose that
breaks them; the depths, boxes and closest approaches take the extreme over
all), the clearance rule judges its two forecasts separately (its test reads
a forecast in order: receding, forgiving) and its repair steers by the one
that breaks, and a rule judged from the stop point (below) is also judged
from the position on the commanded forecast. `Shield.forecast` stays the
own-velocity forecast alone. Why: the own-velocity forecast is the more
accurate one, so on its own it is LESS cautious than the commanded one
whenever the aircraft is slower than its command. 3.7 m off the 14 Oct wall,
drifting away at 0.5 m/s, a 3.2 m/s command at the wall passes the
own-velocity "stop after one control period" test - the aircraft cannot take
it up within a period - and the first build flew such commands one tick at a
time, each followed by a stop. That is sound only while the next tick comes,
and a re-check on the commanded forecast counts each as a P0 escape. With
both judged, nothing the old Shield refused is emitted clean (a seeded
property test over 1,200 decisions around the 14 Oct building and on the six
identity policies, `test_the_measured_velocity_only_adds_...`: 1,188 re-check
clean with the velocity and every one re-checks clean on the commanded
forecast; on the first build 16 of 394 near-building decisions did not), and
the momentum the old forecast could not see is still seen. The cost: one
`filter()` on the 14 Oct log takes median 0.94 ms, p95 2.5 ms, max 4.5 ms
with the velocity against 0.23 / 0.63 / 0.89 ms without (desktop, the
logged ticks replayed), far inside the rail's 100 ms tick.

**Rules the momentum already decides** (`Shield._committed`). If a GUIDED
stop commanded now (the pipe, then zero at the limit) would still break a
rule, no candidate action can avoid that part of the forecast. Judging such
a rule on the own-velocity forecast would refuse every action and the P0
escape count would charge the Shield with an incursion it could not
prevent. So:

* a rule the stop path breaks is judged as if the aircraft were already at
  the stop point (no velocity): its trend-aware "already inside" test asks
  whether the command leaves it, and its repair (ClearanceFix's push,
  GeofenceEscape, StandoffRecover, CorridorReturn) acts from there - the
  recovery starts while the aircraft is still outside;
* a rule the POSITION already breaks is judged exactly as before the change
  (from where the aircraft is, commanded forecast). On the own-velocity
  forecast the pipe was credited to the command: a zero command 0.8 m
  inside the ring "receded" on the tick after a 0.5 m/s push and passed, and
  the escape crawled out at half the floor (found in the closed loop);
* every other rule is judged on the own-velocity forecast (and the
  commanded one beside it); a stop (zero) is clean for each of those by
  construction - on the own-velocity forecast it is the stop path that did
  not break them, on the commanded one standing still at a legal position -
  so the decision always has a clean fallback where the old one did.

The stand-off and corridor repairs judged "breach ahead" on a straight line
along the command; with a measured velocity they ask the check itself, so
they act on the violation the check raised.

**The clearance rule** asked "can I still stop before the ring?" as the
command at constant velocity over 0.1 s + v/(2 x 3 m/s^2). With a measured
velocity it asks it of the response: the pipe, the candidate for one control
period, then a stop, to the standstill. From 3.2 m/s the old form's stop was
1.5 m shorter than the 14 Oct aircraft's.

**Position questions stay position questions.** `state_is_unsafe`,
`unsafe_depth`, the new `stop_breaks` and the Shield's own FSM input drop the
velocity first (`_positional`). The rails' `unsafe` column, the FSM's
`stop_illegal` (X6/X7) and the time-to-safe KPI mean what they meant.

**`Shield.advise(state, raw)`**: the decision `filter()` would make, without
recording it (no sliding-window entry, no FSM step, no clock tick). The rail
uses it for a stop: the Shield's answer to a zero action from a state with a
measured velocity is the zero itself where the stop ends legal, and the
recovery the stop point needs where the momentum carries the aircraft into
a rule.

## 3. The fit

`tools/fit_velocity_response.py` over the six recorded `pas_ap_*` flights
(`tests/fixtures/pas_ap_response_id.jsonl.gz` is the slim copy the test
re-fits). Positions are the EKF's (GLOBAL_POSITION_INT); no run logged a
velocity, so the measured velocity at a tick is the centred least-squares
slope of five ticks. A window starts at every GUIDED tick with setpoints
sent from 1 s before to 2.5 s after (4417 windows); a step window has the
command more than 1 m/s from the measured velocity (550), a decelerating one
has the command more than 1 m/s below the current speed along the direction
of travel (197).

The physical form - each setpoint actually sent acts from `reaction_s` after
it went out, the velocity moves towards it at most `accel_mps2` - was fitted
on a 0.05 s x 0.25 m/s^2 grid, objective the mean RMS horizontal position
error at 1.0, 1.5, 2.0 s on the step windows. Minimum: **0.35 s, 2.0
m/s^2** (the only grid point within 1 % of it). The selection rule written
in the tool (the longest stop from 5 m/s among the 1 % ridge with a
non-negative braking bias, else among the ridge) is moot on these flights.
RMS on the 550 step windows:

| forecast | 0.5 s | 1.0 s | 1.5 s | 2.0 s | 2.5 s | braking bias at 1.5 s (median, m) |
|---|---|---|---|---|---|---|
| commanded (the old forecast) | 0.88 | 1.41 | 1.81 | 2.34 | 3.06 | -1.49 (short) |
| measured velocity held | 0.22 | 0.78 | 1.53 | 2.33 | 3.12 | +1.40 (long) |
| Shield form, no pipe (velocity kept 0.35 s) | 0.20 | 0.46 | 0.71 | 1.16 | 1.86 | +0.19 |
| **Shield form with the pipe** (what the rails fly) | 0.12 | 0.34 | 0.70 | 1.29 | 2.07 | -0.25 |
| physical form (knows the later commands) | 0.12 | 0.31 | 0.49 | 0.64 | 0.75 | -0.11 |

The Shield's forms cannot know the next command, so their error past 1.5 s
is mostly the pilot changing its mind; inside the 1-1.5 s that decides a
wall approach the pipe form is 4x (1 s) and 2.6x (1.5 s) better than the old
forecast. A pipe-free "Shield form" fit (choosing the parameters to score
the constant-command forecast directly) lands on a flat ridge (0.40 s / 2.75
to 0.55 s / 5.0 within 1 %) because it fits the pilot's later changes too;
the physical fit has a single minimum and is the aircraft's, which is why it
is the one used.

**Steady state.** Holding a command for 1.5 s, the aircraft reaches a median
0.956 of the commanded speed (p10 0.886, p90 1.073): converging to the
command exactly is a slight over-forecast, the conservative side.

**Vertical.** On the 73 windows whose commanded climb differed from the
measured by more than 0.3 m/s the response model forecast altitude worse
than the commanded forecast (RMS 0.28 against 0.15 m at 1 s, 0.45 against
0.31 m at 1.5 s), and 95 % of ticks command under 0.24 m/s of climb. The
vertical forecast is unchanged.

**BRAKE.** Two recorded entries (`pas_ap_close_on` ticks 280 and 387: 1.4 and
2.7 m/s measured, 2.47 and 2.91 m travelled, both still moving 2.3-2.4 s
later). After the GUIDED pipe, the envelope whose travel at each entry's last
sample and whose full stop are never short of the recorded travel (0.05 m
slack), smallest RMS: **0.05 s, then at most 0.75 m/s^2** (RMS 0.28 m). Under
half of a GUIDED zero setpoint's 2.0. Two entries make an envelope, not a
fit; entry 387 was still moving at 1 m/s when RTL took over, so its full stop
is longer than anything recorded.

## 4. Without a velocity nothing changes

`tests/test_own_velocity_forecast.py` pins the sha256 of 18,000 seeded
`filter()` decisions (six policies - the 14 Oct one, its NFZ variant, the
corridor survey, the SITL pedestrian, the gap follow, the 21-rule hard
policy - each with and without the Shield's own escalation FSM, the CityLife
flight-band map where the policy has a clearance rule), computed on c77c77d
before a line of this change existed. They are byte-identical with no
velocity, with `velocity=None` written out, and with a velocity given to a
Shield built with `velocity_response=None` (6000 per run in CI, the 18,000
with `--full`). The sample repairs on more than 30 % of its ticks and raises
clearance, stand-off, geofence, corridor and altitude violations; the same
sample with a velocity does not reproduce the digest. `test_check_contract`
(28,800 `_check` answers) and every existing Shield, FSM, rail and replay
test pass unchanged.

## 5. The replay: when the wall is first seen

Every logged tick of the 14 Oct and 13 Oct flights through the Shield (open
loop), the logged flown action fed back as the pipe, the velocity a causal
three-tick slope of the logged positions (the stand-in for the EKF velocity;
a centred five-tick slope gives the same ticks):

| flight | old first `bld-clearance` flag | new first flag | earlier by | clearance there (old / new) |
|---|---|---|---|---|
| 14 Oct `pas_ap_video_on_20261014` | tick 364 (= the flight's) | tick 356 | 8 ticks, 0.87 s | 4.61 m / 7.17 m |
| 13 Oct `pas_ap_close_on` | tick 382 (= the flight's) | tick 375 | 7 ticks, 0.76 s | 4.98 m / 7.29 m |

**The stop point.** A GUIDED stop commanded at the old tick, through the
measured response and the pipe, ends 0.66 m from the wall on 14 Oct (2.3 m
inside the margin) and 0.61 m on 13 Oct; commanded at the new tick it ends
3.27 m and 3.25 m away, outside the 3 m margin. (The model's stop from tick
363 is 0.4 m longer than the flown one - the flight decelerated at up to
2.7 m/s^2 against the fit's 2.0, and pushed away rather than stopped - the
conservative side.)

**The hold.** At `pas_ap_close_on` tick 387, where the FSM went Brake ->
Loiter and the rail asked for BRAKE, the hold plan with the measured velocity
and the pipe predicts that BRAKE's stop path breaks `bld-clearance` and keeps
the Loiter in GUIDED; its predicted BRAKE stop point lies past where the
aircraft actually was when X7 fired. Without a velocity it plans BRAKE, as
the rail flew.

**P0 escapes, open loop, all six recorded flights** (the Shield's own
re-check of its output, P0 rules; `tools/own_velocity_closed_loop.py
--all-runs`): commanded / own-velocity 0 / 0, 0 / 0, 1 / 1 (the same tick,
`pas_ap_close_off` 407), 0 / 0, 0 / 0, 0 / 0 - the same on the old basis
(the own-velocity Shield's output re-checked on the commanded forecast from
the position). The own-velocity Shield touches more ticks (81 -> 91,
43 -> 53, 749 -> 755, 173 -> 180, 755 -> 767, 155 -> 171): it intervenes
earlier.

## 6. The closed loop

A replay cannot show where the aircraft would have gone. `tools/
own_velocity_closed_loop.py` flies the recorded pilot's raw actions through
the Shield, the rail's FSM and its hold, with the measured response as the
aircraft (the physical form, integrated at 5 ms, with the setpoints the loop
actually sends; 0.05 m/s of noise on the measured velocity), BRAKE at the
envelope's 0.75 m/s^2, and an RTL held 6 s (both recorded RTLs from inside the
margin stayed inside it 5.65 and 6.44 s while climbing). Validation: flown
with the commanded forecast it is inside the margin on 73 ticks of each
recorded approach, where the flights logged 71 and 78.

The two approaches (the 111 ticks around each). P0 escapes on both bases:
the rail's (the Shield's re-check of what was flown, with the velocity when
it was given) / the old one (the flown action re-checked on the commanded
forecast from the position, the basis of every stored run):

| approach | aircraft | forecast | ticks inside the margin | min clearance | first flag | FSM | P0 escapes |
|---|---|---|---|---|---|---|---|
| 14 Oct | fitted (0.35 s, 2.0) | commanded | 73 | inside the footprint | 362 | G1, G3, X7 -> RTL | 0 / 0 |
| 14 Oct | fitted | **own-velocity** | **0** | **3.21 m** | 355 | G1 x2, G7, G3 | 0 / 0 |
| 14 Oct | slower (0.45 s, 1.75) | commanded | 73 | inside the footprint | 362 | G1, G3, X7 -> RTL | 0 / 0 |
| 14 Oct | slower | own-velocity | 26 | 2.21 m | 356 | G1, G3 | 0 / 0 |
| 13 Oct | fitted | commanded | 73 | inside the footprint | 382 | G1, G3, X7 -> RTL | 0 / 0 |
| 13 Oct | fitted | **own-velocity** | **0** | **3.29 m** | 375 | G1 x2, G7, G3 | 0 / 0 |
| 13 Oct | slower | commanded | 74 | inside the footprint | 381 | G1, G3, X7 -> RTL | 0 / 0 |
| 13 Oct | slower | own-velocity | 19 | 2.50 m | 375 | G1, G3 | 0 / 0 |

On the fitted aircraft both approaches stay outside the margin, by 0.21 and
0.29 m. On an aircraft slower than the model the forecast is optimistic and
the aircraft enters, 0.5-0.8 m deep instead of into the building, on fewer
ticks, with no fail-safe. The first build (own-velocity forecast alone) kept
the fitted aircraft out by 1 and 4 cm, with 6 and 1 P0 escapes on the old
basis (full-speed commands at the wall from 3.0-3.4 m off it, one tick at a
time) and 34 ticks inside on the slower 14 Oct aircraft; `test_the_closed_
loop_keeps_both_approaches_outside_the_margin` fails on that build.

Every recorded flight, whole (the same pilot raw actions open loop, so after
the first divergence the pilot no longer "sees" the aircraft; ticks inside
any hard rule while the Shield/FSM or the RTL climb had the aircraft; P0
escapes 0 on both bases in every run):

| flight | aircraft | commanded: inside / fail-safe / min clearance | own-velocity: inside / fail-safe / min clearance |
|---|---|---|---|
| 8 Oct follow | fitted | 101 / RTL / 2.62 | 6 / - / 2.85 |
| 8 Oct follow | slower | 99 / RTL / 2.65 | 5 / - / 3.05 |
| 12 Oct follow | fitted | 13 / - / 2.79 | 0 / - / 3.03 |
| 12 Oct follow | slower | 107 / RTL / 2.94 | 5 / - / 2.96 |
| 13 Oct close, Shield-off log | fitted | 76 / RTL / footprint | 0 / - / 3.10 |
| 13 Oct close, Shield-off log | slower | 76 / RTL / footprint | 25 / - / 2.25 |
| 13 Oct close | fitted | 75 / RTL / footprint | 3 / - / 2.97 |
| 13 Oct close | slower | 75 / RTL / 0.99 | 15 / - / 2.56 |
| 14 Oct, Shield-off log | fitted | 75 / RTL / 1.71 | 0 / - / 3.08 |
| 14 Oct, Shield-off log | slower | 20 / - / 2.46 | 6 / - / 2.93 |
| 14 Oct | fitted | 90 / RTL / footprint | 1 / - / 3.04 |
| 14 Oct | slower | 90 / RTL / footprint | 35 / - / 2.15 |

Fewer ticks inside a hard rule in all 12, no P0 escape in any on either
basis, and no fail-safe in any own-velocity run (the first build had two
N-in-T RTLs, G4). Not all of it is free: the earlier interventions add
Brake/Loiter spells - on the 8 Oct follow the FSM goes to Loiter seven times
(G3) against once, and back (G8) six times - expected of a Shield that sees
the wall 0.8 s earlier, and the number to watch in a flight. (`inside`
counts every hard rule, so a run can count a tick inside and still keep
3 m from the building.)

## 7. The hold that stops clear

On this rail the FSM's Loiter is flown as ArduCopter BRAKE (ArduCopter's
LOITER sinks with no pilot). `guardrail.response.hold_plan` predicts BRAKE's
stop path from the measured velocity and the pipe (the envelope above) and
asks the Shield, pose by pose, whether a stop there breaks a hard, enforced
rule (`stop_breaks`, the FSM's `stop_illegal` reading). If it does - or the
aircraft is still faster than 0.5 m/s (`HOLD_HANDOVER_MPS`: in the closed
loop a stop path predicted clear at 1.4 m/s ended 1.6 cm inside the margin,
where BRAKE then held it) - the rail does not ask for BRAKE: it stays in
GUIDED and streams the Shield's answer to a stop (`advise` of a zero
action), asks again every tick, and hands over to BRAKE once BRAKE's stop
would end legal and slow. The FSM's "brake" setpoint is flown the same way
(the Shield's answer to a stop instead of a raw zero). The FSM's states,
edges and inputs are untouched: this is how the rail flies its Loiter, as
`--hold-mode` already chooses between BRAKE and LOITER. Rows record
`hold_deferred`, `stop_advised`, `fsm_setpoint` (the FSM's own) and
`autopilot_set_mode` (what was actually asked: None while deferred, BRAKE
at the hand-over); `metrics.hold.stop_clear` lists every plan.

## 8. The rails

| rail | velocity | default | what it records |
|---|---|---|---|
| `demo/pas_ardupilot` (pymavlink) | LOCAL_POSITION_NED vx/vy/-vz, else GLOBAL_POSITION_INT's, fresh within 0.3 s (`autopilot.snapshot_velocity`) | **on** (`--own-velocity-forecast on`), Shield-on arm only | rows `state_v`, `state_v_source`, `shield_velocity`, `hold_deferred`, `stop_advised`, `fsm_setpoint`; `metrics.shield_forecast`, `metrics.hold.stop_clear` |
| `demo/pas_ardupilot` (MAVROS link) | `/mavros/local_position/velocity_local` (ENU swapped) | on, as above | as above |
| `sitl/ros2_shield_node.py` | `/mavros/local_position/velocity_local` | **off** | rows `state_v`, `shield_velocity`; `metrics.shield_forecast` |
| `sitl/run_sitl_demo.py` | LOCAL_POSITION_NED of the pose it flies on | **off** | as above |

On for the Project AirSim + ArduPilot rail because the evidence the task set
holds there, on both P0 escape bases: replayed open loop on all six recorded
flights the Shield's P0 escapes are the same ticks with and without it
(re-checked with the velocity or on the commanded forecast alike), and in
closed loop the position was illegal on fewer ticks in all 12 runs, with no
P0 escape and no fail-safe in any. The harness (experiments/sweep_scenarios.py)
passes no velocity: the smoke profile's summary is identical to c77c77d's
field for field, timings and paths aside. Off on the two SITL rails because the response was measured on the
Project AirSim quad and they fly ArduPilot's own SITL physics, whose
recorded flights reach 0.69-0.96 of the commanded speed in steady flight
(against 0.96): nothing measured says the model fits them. They log the
velocity so it can be measured (re-run `tools/fit_velocity_response.py` on
their logs once they carry `state_v`). The hold that stops clear is on the
ArduPilot rail only; the SITL rails' holds go through
`sitl/mavlink_adapter_node.py`. `demo/follow_vlm.py` is not wired (a later
task). With the Shield off, no rail gives the Shield the velocity, so the
control arm's escapes stay judged on the commanded forecast.

## 9. What this does not do, and the open questions

* **The P0 escape count's basis.** Its definition is unchanged - the
  Shield's re-check of the action that was flown, P0 rules - and with the
  flag on that re-check is made with the velocity, as the decision was. It
  is never laxer than the old basis (the commanded forecast from the
  position): by the rule in section 2 an action that re-checks clean with
  the velocity re-checks clean without it (the property test). It can be
  stricter: a momentum breach the old forecast misses is counted. A flown
  row carries `state_v` and `flown_action`, so the old basis can be
  recomputed from any flight log.
* A rule the stop path breaks is judged from the stop point (and from the
  position on the commanded forecast); a thin zone the stop path crosses and
  leaves is then judged from beyond it. No shipped policy has a zone thinner
  than a stop from the speed cap (8 m from 5 m/s).

* The committed-rule judgement places a rule's check at the stop point at
  forecast time 0; a time window or a moving zone is read at the forecast's
  own times from there, not at the time the aircraft reaches the stop point.
* The stand-off forecast uses the aircraft's response; the subject's own
  velocity is the separate `set_subject(velocity=)` (no rail passes it).
* While the position is already inside a rule, recovery is exactly the old
  one: its push is sized from the depth (e.g. ClearanceFix's 0.5-0.67 m/s),
  not from the momentum still carrying the aircraft deeper. The 14 Oct creep
  was in that regime; the change is that the Shield should no longer get
  there on the fitted aircraft.
* **For the PI:** whether `stop_illegal` should ever mean "where a stop
  would end" for the FSM (X6/X7 and the "stop withheld" rule read it); it
  does not here, by design. And whether the earlier interventions' extra
  Brake/Loiter spells (section 6) are acceptable or call for N-in-T to be
  revisited for momentum-aware forecasts.

## 10. Re-flying the 14 Oct close approach

The 14 Oct run's parameters (its `metrics.json` `params`: object "a red
car", cruise 8 m, yaw gain 1.2, want width 0.5, speed max 4.0, alt gain 0.6,
climb max 1.8, det thresh 0.02, colour min 0.1, seed 20261006, pymavlink,
Project AirSim camera, scene load, hold BRAKE, subject actor auto; the signed
`bundles/follow-car-citylife-v0.1.0.tar.gz`; recorded with the view). Only
the want width differs from the node's defaults, so the 14 Oct command
(docs/DESIGN-projectairsim-ardupilot.md) re-flies it with the new default:

```powershell
.\scriptsun_pas_ardupilot.ps1 -Step node -Seconds 120 -SaveView -Tag pas_ap_ovf_on_<date> -NodeArgs "--want-width","0.5"
# the same build with the old forecast and BRAKE hold, for an A/B:
.\scriptsun_pas_ardupilot.ps1 -Step node -Seconds 120 -SaveView -Tag pas_ap_ovf_off_<date> -NodeArgs "--want-width","0.5","--own-velocity-forecast","off"
```

Compare with `pas_ap_video_on_20261014`: the rows with `unsafe` true while
the Shield or the FSM flew (71 there, 20 of them before the RTL) and the
deepest `unsafe_depth_m` (1.0 m); `metrics.fsm.transitions` (there: G1 at
tick 363, X7 -> RTL at 388; expected: no X7); `kpi.json` `p0_escapes`
(0 there); the first row whose `violations` cite `bld-clearance` against the
tick the aircraft is 4.6 m from the wall (expected ~0.8 s earlier);
`metrics.shield_forecast` (`mode` own_velocity, `velocity_ticks` close to
`ticks`, `velocity_sources` mostly `local_velocity`); and
`metrics.hold.stop_clear` (each plan, `guided_hold_ticks`,
`stop_advised_ticks`). The approach depends on where the car leads the
follow, so a run that never comes near the building says nothing either way.
