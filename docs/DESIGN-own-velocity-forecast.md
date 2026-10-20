# The Shield's own-velocity forecast, and a hold that stops clear

**Date:** 2026-10-19
**Status:** built and tested offline. `guardrail/response.py` (the response
model), `guardrail/shield.py` (the forecast, `note_flown`, `advise`,
`stop_breaks`), `guardrail/models.py` (`State.velocity`), the three ArduPilot
rails, `tools/fit_velocity_response.py` (the fit) and
`tools/own_velocity_closed_loop.py` (the closed loop);
`tests/test_own_velocity_forecast.py` at 27/27. **Not flown yet**: the
re-flight command is at the end.
**Revised 2026-10-20:** the measured velocity now only ADDS to the commanded
forecast (section 2, "The measured velocity only adds"). The first build
judged the own-velocity forecast alone, and in the closed loop it streamed
full-speed commands at the 14 Oct building from the edge of its margin;
re-checked on the commanded forecast, the basis of every stored P0 escape
count, those were 6 P0 escape ticks on the 14 Oct approach and 1 on the 13
Oct one, against the old Shield's 0.
**Revised again 2026-10-20, after review:** the response numbers the
Shield forecasts with were short on braking in the form the Shield flies
(median 0.25 m at 1.5 s), and the fit's own safety rule had fallen back to
them without saying so. They are now chosen by that rule in the Shield's
form (0.45 s, 1.75 m/s^2; section 3); the closed loop's aircraft is the
identified one, no longer the Shield's own model (section 6); a measured
velocity is set aside while the position already breaks a hard rule
(section 2); the segment check walks each forecast path; the stop-advised
ticks have their own audit record; a deferred hold is handed to BRAKE when
the stream cannot be trusted (section 7); the pipe is aged on the rail's
real clock; and the velocity mode's query cost is measured at 50 rules and
brought under the 5 ms budget. Every number below is the revised build's.
**Re-review 2026-10-20:** the episode exporter split a run's audit at every
stop-advised tick (its second record has the same tick) and joined a
fragment - fixed, section 7 "Read back"; the braking-bias rule is worded
as what it is, a median at one horizon (section 3: p25 0.33 m short at
1.5 s, the median 0.16 m short at 2.0 s); the set-aside's reach across
rules is stated (section 2) and a per-rule version left open (section 9);
the cost figures are one measurement each (section 2, "The cost").
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
for the reaction time (0.45 s), the setpoints the rail already sent
(`Shield.note_flown(action)`, called once per tick after the send; the
"pipe", `response.pipe_segments`), and then turns towards the candidate at
the acceleration limit (1.75 m/s^2), exactly integrated. Both numbers are
measured, and chosen so that, in this form, the MEDIAN braking error at
1.5 s is not short (+0.03 m): a median at one horizon, not a bound - a
quarter of the braking windows are 0.33 m short or more at 1.5 s, and at
2.0 s the median is 0.16 m short (p25 0.51 m; section 3). A
caller that never calls `note_flown` gets the measured velocity kept for
the reaction time instead. Horizontal only: the climb rate stays the
commanded one (section 3). Same grid, same 50 poses.

**The pipe's clock** (`Shield(pipe_clock=...)`). A setpoint's age is read
on the rail's real clock, not on the Shield's monitor clock: the ArduPilot
rail lets `filter()` count its ticks at the nominal 0.1 s, while its real
ticks are 0.109 s median in every recorded log and once 0.73 s (14 Oct), so
on the monitor clock each setpoint was aged about 9 % too young per tick and
a delayed tick re-applied a setpoint already in the measured velocity. The
rail passes its loop clock; the clock is latched once per tick, and read
only for a state with a velocity (the rule clock, the sliding window and
N-in-T are untouched).

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
identity policies, `test_the_measured_velocity_only_adds_...`: 1,190 re-check
clean with the velocity and every one re-checks clean on the commanded
forecast; on the first build 16 of 394 near-building decisions did not), and
the momentum the old forecast could not see is still seen.

**The cost** (desktop i7-14700, pinned to its performance cores). One
`filter()` on the 14 Oct log takes median 0.49 ms, p95 1.86 ms, max 3.5 ms
with the velocity against 0.17 / 0.55 / 1.06 ms without (the logged ticks
replayed), far inside the rail's 100 ms tick. The grant's other budget is
the monitor query at 50 rules, <= 5 ms, which the first build did not time
in this mode (review): `tools/profile_shield_tick.py profile --arms
design_load design_load_velocity --horizons grant` (the packed real flights
with 50 rules, 1301 ticks) measured `_check` at median 4.8 ms, p99 18.4 ms,
max 31 ms with the velocity - every rule asked `_committed` afresh outside a
tick, and a pose test built a shapely Point for poses nowhere near the zone.
(Re-measured at the re-review on the same desktop, `--cpus
0,2,4,6,8,10,12,14`, the first build's Shield (6987366) under the same
tool: median 4.19 ms, p99 11.8 ms, max 17.7 ms, 359 of the 1301 ticks over
5 ms - lower on a quieter host, the same verdict. The `_check` docstring's
"p99 13.9 ms" matched neither run and now cites these two.)
Now `_check` with a velocity opens a tick view of its own (one `_committed`,
one pipe and one set of poses per action, cached), the altitude rule walks
one of the two paths (they share every altitude), and the fence tests skip a
pose outside the ring's envelope before building the Point - the same
answers, pose for pose (a seeded digest of 3,600 velocity decisions and
`_check` answers on the six identity policies, segment check on and off, is
unchanged by it; the 18,000-decision identity proof and the 28,800 answers of
`test_check_contract` pass). At 50 rules `_check` is now median 0.87 ms,
p99 1.23 ms, max 1.51 ms with the velocity (0.22 / 0.41 / 0.54 ms without;
`filter()` p99 9.0 ms, max 40 ms on one tick, against 100 ms; re-measured
pinned at the re-review: 0.87 / 1.21 / 1.48 ms, `filter()` p99 9.2 ms,
max 39 ms), and on the
synthetic bench of `experiments/bench_shield_50rules.py` (400 seeded states
per scenario) at most 1.92 ms (p99 1.18 ms) where the first build reached
4.93 ms (6.1 ms in the review's run).

**From an illegal position the velocity is set aside**
(`Shield._decision_state`, review). Replayed open loop on the Shield-off
close run (`pas_ap_close_off` ticks 733-739), the logged aircraft was
0.8-1.0 m inside the building footprint, still flying deeper at 1.2-1.8 m/s,
3.8-4.9 m from the car. The building rule, broken by the position, was
judged as before, but the car's stand-off ring was committed (a stop would
end inside it) and judged from the stop point, where only an action opening
the range passes. No action both left the building and opened the range
from there, the chain fell to the ClearanceEscape rescue, and the rescue
broke the stand-off on the commanded forecast too: 7 P0 escapes where the
Shield without the velocity pushed slowly out and broke nothing. The
momentum adds constraints; from an illegal position, where an escape must be
found, they can leave none. So while the position breaks a hard, enforced
rule (the FSM's `stop_illegal`), the decision is made from the position
without the velocity - byte for byte the old Shield's - and the velocity
returns the tick the position is legal; counted (`n_velocity_set_aside`,
`metrics.shield_forecast.velocity_set_aside_ticks`). It makes true of every
rule what section 9 said of the broken one: while the position is already
inside a rule, recovery is exactly the old one.

The set-aside is all or nothing ACROSS RULES (re-review): any broken hard,
enforced rule drops the momentum for every rule, so an unrelated breach -
the follow dipping under `alt-band`'s 6 m floor - also drops it for the
building. Replayed open loop on the six recorded flights (causal velocity,
the logged pipe), the velocity is set aside on 310 of the 1101 ticks of
`pas_ap_20261008_224440`, 279 of them for `alt-band` alone (seven spans
below 6 m between ticks 218 and 1045, mid-follow, not take-off or landing;
at most
0.66 m/s horizontally there), 25 for the building, 6 for the stand-off; on
the other five flights every set-aside tick is the building rule itself
(12, 720, 78, 739, 71 ticks). During such an alt-band span a fast approach
to a building would be judged on the commanded forecast only. No harm
measured (the alt-band spans are slow; the re-review found the closed-loop
results unchanged with the set-aside disabled, and the open-loop P0 escapes
the same ticks on all six flights). Setting aside only the broken rule
would not have prevented the 7 escapes above - the rule that left no escape
there was the stand-off ring, unbroken but committed - so a per-rule
version has to choose the rules whose committed judgement conflicts with
the escape, which needs evidence of its own: an open item (section 9).

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
  so where no rule is committed the decision has the clean fallback the old
  one had. Where the momentum commits two rules whose recoveries conflict
  (a building and a subject's ring at once) there may be none: the aircraft
  is then about to break one whatever is commanded, and the best effort's
  violations are counted (section 9, the P0 basis). From an illegal position
  that case cost 7 P0 escapes on the Shield-off close run before the velocity
  was set aside there (above).

The stand-off and corridor repairs judged "breach ahead" on a straight line
along the command; with a measured velocity they ask the check itself, so
they act on the violation the check raised.

**The segment check walks each path** (`Shield(segment_check=True)`, the
grant's "trajectory segment crosses polygon"; `_segment_paths`). With a
velocity `_predict` yields the two forecasts interleaved, and the first
build tested consecutive poses of that stream - the connectors BETWEEN the
two forecasts, which neither path flies (review). Moving North at 4 m/s,
commanded East at 4 m/s, it refused a 0.1 m square lying between the two
paths ("predicted segment (0.0,1.2)->(1.2,0.0) crosses NFZ") and missed a
0.02 x 0.2 m strip the own-velocity path crosses between its t + 1.5 s and
t + 1.6 s poses. The fence check, the corridor check and the geofence
repair now walk the own-velocity path and the commanded path separately
(the earlier crossing; the repair takes the deeper); both cases are tests.
A zone crossed WITHIN the reaction time (the review's own probe, between the
t + 0.2 s and t + 0.3 s poses) is crossed by the stop path as well: it is
committed and judged from the stop point, so it is not flagged - by the
rule for what the momentum already decides, section 9, not by the segment
test. No rail turns the segment check on.

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
a rule (from an illegal position, the old Shield's recovery: the velocity is
set aside there too).

**The record.** A velocity tick's sliding-window entry (`TickRecord`) also
carries the tick's pipe, so `Shield.forecast(rec.state, rec.decision.emitted,
pipe=rec.pipe)` rebuilds the forecast the tick judged exactly, whatever was
sent since (the review found the docstring's promise broken: the forecast
also depends on the pipe and the tick's time, neither on the State). A
record whose velocity was set aside holds the state without it, as decided.

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
error at 1.0, 1.5, 2.0 s on the step windows. Its minimum, the IDENTIFIED
aircraft: **0.35 s, 2.0 m/s^2** (the only grid point within 1 % of it). It
is the closed loop's aircraft (section 6); it is not what the Shield
forecasts with.

**What happened to the safety rule, and the numbers the Shield uses.** The
tool was written to pick, among the 1 % ridge, a point whose median braking
error is not negative (not short, in the median, of where a braking
aircraft went - short is the unsafe direction for a keep-out rule), "else
among the ridge". On these
flights no ridge point met it: the ridge holds one point, whose braking
error is -0.11 m in the physical form, and the fallback took it without
saying so - the first version of this note called the rule "moot". Worse,
the rule read the wrong form: in the form the Shield actually flies (the
pipe of setpoints already sent, then the candidate held) those numbers are a
median **0.25 m short at 1.5 s** (p25 0.51 m short), and in the closed loop
an aircraft a little slower than the fit then still entered the margin. The
rule is now applied in the Shield's form, over the whole grid: the most
accurate point (physical objective) whose median braking error in the
Shield's form is >= 0. It chooses **0.45 s, 1.75 m/s^2** (median +0.03 m;
physical objective 13.6 % above the minimum: it describes a slightly slower
aircraft than the one identified). When nothing qualifies the report says
`fallback` and the tool prints a WARNING; the fixture test pins the sign of
the bias, the absence of a fallback, and that a stiff one-point grid does
fall back loudly. A slower model is only optimistic about an aircraft
speeding UP towards a hazard, which the commanded forecast judged beside it
already covers (section 2).

**What the rule does not say** (re-review). It is a MEDIAN at ONE horizon,
1.5 s - not "never short", which the commit introducing it claimed in its
title ("Never be optimistic about a stop"). With the chosen numbers, in the
Shield's form, the braking error by horizon is (p25 / median, m; the fit's
`decel_along_by_h_m`, pinned by the fixture test):

| horizon | 0.5 s | 1.0 s | 1.5 s | 2.0 s | 2.5 s |
|---|---|---|---|---|---|
| Shield form, the Shield's numbers | -0.03 / +0.06 | -0.16 / +0.13 | **-0.33 / +0.03** | **-0.51 / -0.16** | -0.71 / -0.31 |
| physical form, the Shield's numbers | -0.03 / +0.06 | -0.16 / +0.13 | -0.23 / +0.09 | -0.29 / +0.06 | -0.33 / +0.04 |

A quarter of the braking windows are 0.33 m short or more at 1.5 s, and at
2.0 s the median itself is short. The physical form with the same numbers
stays >= 0 in the median at every horizon, so the later shortfall is mostly
the pilot changing its command, which no Shield form can know. And on an
aircraft slower than these numbers (0.55 s, 1.5 m/s^2) the closed loop's
approach still entered the 3 m margin, to 2.01 m on 33 ticks (section 6).

RMS on the 550 step windows, and the along-track error on the 197
decelerating windows at 1.5 s (forecast minus actual; negative = short):

| forecast | 0.5 s | 1.0 s | 1.5 s | 2.0 s | 2.5 s | braking error p25 / median / p75 (m) |
|---|---|---|---|---|---|---|
| commanded (the old forecast) | 0.88 | 1.41 | 1.81 | 2.34 | 3.06 | -2.05 / -1.49 / -1.01 |
| measured velocity held | 0.22 | 0.78 | 1.53 | 2.33 | 3.12 | +1.01 / +1.40 / +1.84 |
| **Shield form with the pipe, the Shield's numbers** (what the rails fly) | 0.13 | 0.36 | 0.71 | 1.25 | 1.98 | -0.33 / **+0.03** / +0.42 |
| Shield form, no pipe (velocity kept 0.45 s) | 0.22 | 0.57 | 0.84 | 1.20 | 1.80 | +0.10 / +0.46 / +0.89 |
| Shield form with the pipe, identified numbers (the first build) | 0.12 | 0.34 | 0.70 | 1.29 | 2.07 | -0.51 / -0.25 / +0.11 |
| physical form, identified numbers (knows the later commands) | 0.12 | 0.31 | 0.49 | 0.64 | 0.75 | -0.36 / -0.11 / +0.16 |
| physical form, the Shield's numbers | 0.13 | 0.35 | 0.56 | 0.74 | 0.87 | -0.23 / +0.09 / +0.48 |

In the form the Shield flies, its numbers are as accurate as the identified
ones (within 0.02 m at every horizon up to 1.5 s) and no longer short on
braking in the median at 1.5 s (p25 still -0.33 m; the median is -0.16 m at
2.0 s, above): the choice costs nothing where the forecast is read. The Shield's
forms cannot know the next command, so their error past 1.5 s is mostly the
pilot changing its mind; inside the 1-1.5 s that decides a wall approach the
pipe form is 3.9x (1 s) and 2.5x (1.5 s) better than the old forecast. A
pipe-free "Shield form" fit (choosing the parameters to score the
constant-command forecast directly) lands on a flat ridge (0.40 s / 2.75 to
0.55 s / 5.0 within 1 %) because it fits the pilot's later changes too,
which is why the objective stays the physical one and the Shield's form
enters through the bias rule.

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
slack), smallest RMS. Behind the Shield's GUIDED numbers (what the hold plan
predicts with): **no further coast, then at most 0.95 m/s^2** (RMS 0.17 m;
`ARDUCOPTER_BRAKE`). Behind the identified GUIDED numbers (the closed loop's
BRAKE plant): 0.05 s, then 0.75 m/s^2 (RMS 0.28 m). Each envelope is
consistent with its own pipe; both are about half of a GUIDED zero
setpoint's limit. Two entries make an envelope, not a fit, and neither is a
clean BRAKE measurement: entry 387 was still moving at 1 m/s when RTL took
over (its full stop is longer than anything recorded), and inside BRAKE its
speed rose again from 0.57 to 1.06 m/s (ticks 400-407) while the aircraft
was 2.2-3.2 m into the 3 m margin - at the building's face, most likely in
contact with it (the closed loop puts the old run 1.5-2.0 m inside the
footprint). ArduCopter's own BRAKE decelerates far harder in the source
(`BRAKE_MODE_DECEL_RATE`, 750 cm/s/s), so the envelope errs towards
GUIDED - the long side for a stop - but it is not a measurement of BRAKE. A
clean SITL BRAKE step test is an open item (section 9).

## 4. Without a velocity nothing changes

`tests/test_own_velocity_forecast.py` pins the sha256 of 18,000 seeded
`filter()` decisions (six policies - the 14 Oct one, its NFZ variant, the
corridor survey, the SITL pedestrian, the gap follow, the 21-rule hard
policy - each with and without the Shield's own escalation FSM, the CityLife
flight-band map where the policy has a clearance rule), computed on c77c77d
before a line of this change existed. `--regenerate` recomputes them on a
`git archive` of c77c77d, never on the code checked out, and the fixture
records the base's full hash (`computed_on`), which the identity tests check
(review: the first version hashed whatever was checked out under the base's
name); regenerated so, the digests came out unchanged. They are byte-identical with no
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
| 14 Oct `pas_ap_video_on_20261014` | tick 364 (= the flight's) | tick 354 | 10 ticks, 1.09 s | 4.61 m / 7.83 m |
| 13 Oct `pas_ap_close_on` | tick 382 (= the flight's) | tick 373 | 9 ticks, 0.98 s | 4.98 m / 7.97 m |

(The first build, with the identified numbers, flagged at 356 and 375: the
Shield's numbers now forecast a longer stop, so the wall is seen two ticks
sooner.)

**The stop point**, on the identified aircraft (the best estimate of where
it would have gone): a GUIDED stop commanded at the old tick, through the
response and the pipe of what was sent, ends 0.66 m from the wall on 14 Oct
(2.3 m inside the margin) and 0.61 m on 13 Oct; commanded at the new tick it
ends 3.71 m and 3.82 m away, outside the 3 m margin. The Shield's own
numbers forecast every one of those stops longer (into the footprint from
the old ticks; 2.94 and 3.05 m from the new ones): it acts on the longer
stop. (The flight itself decelerated at up to 2.7 m/s^2 from tick 363 and
pushed away rather than stopped, so both models' stops are longer than the
flown one - the conservative side.)

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
the position). Without the set-aside of section 2 the Shield-off close run
read 1 / 8 (ticks 733-739, inside the building footprint; the Shield-on
aircraft never gets there - 0 P0 escapes and 0 ticks inside in its closed
loop). The own-velocity Shield touches more ticks (81 -> 92, 43 -> 54,
749 -> 757, 173 -> 182, 755 -> 774, 155 -> 176): it intervenes earlier.

## 6. The closed loop

A replay cannot show where the aircraft would have gone. `tools/
own_velocity_closed_loop.py` flies the recorded pilot's raw actions through
the Shield, the rail's FSM and its hold, with a simulated aircraft (the
physical form, integrated at 5 ms, with the setpoints the loop actually
sends; 0.05 m/s of noise on the measured velocity), its BRAKE as the
envelope behind its own pipe, and an RTL held 6 s (both recorded RTLs from
inside the margin stayed inside it 5.65 and 6.44 s while climbing).
Validation: flown with the commanded forecast it is inside the margin on 73
ticks of each recorded approach, where the flights logged 71 and 78.

**The aircraft is not the Shield's model.** The first build flew the
Shield's own response numbers as the aircraft and its own BRAKE envelope as
BRAKE, so its main result was partly circular (review). The aircraft is now
the IDENTIFIED one (0.35 s, 2.0 m/s^2, with the BRAKE envelope fitted behind
its pipe: 0.05 s, 0.75 m/s^2 - `PLANT_*`, pinned to the fit by the fixture
test), while the Shield forecasts with its own, slower numbers. `--robust`
also flies an aircraft exactly as slow as the Shield's model (the matched,
circular case, for reference) and one slower than it (0.55 s, 1.5 m/s^2), on
which the Shield's forecast is optimistic. It remains a model of the same
form as the Shield's, fitted to the same six flights; what the closed loop
cannot show is an aircraft whose response has another FORM, which only a
flight can.

The two approaches (the 111 ticks around each). P0 escapes on both bases:
the rail's (the Shield's re-check of what was flown, with the velocity when
it was given) / the old one (the flown action re-checked on the commanded
forecast from the position, the basis of every stored run):

| approach | aircraft | forecast | ticks inside the margin | min clearance | first flag | FSM | P0 escapes |
|---|---|---|---|---|---|---|---|
| 14 Oct | identified (0.35 s, 2.0) | commanded | 73 | inside the footprint | 362 | G1, G3, X7 -> RTL | 0 / 0 |
| 14 Oct | identified | **own-velocity** | **0** | **3.28 m** | 351 | G1 x2, G7, G3 | 0 / 0 |
| 14 Oct | as slow as the model (0.45 s, 1.75) | commanded | 73 | inside the footprint | 362 | G1, G3, X7 -> RTL | 0 / 0 |
| 14 Oct | as slow as the model | own-velocity | 0 | 3.12 m | 351 | G1 x2, G3 x2, G8 | 0 / 0 |
| 14 Oct | slower than the model (0.55 s, 1.5) | commanded | 73 | inside the footprint | 362 | G1, G3, X7 -> RTL | 0 / 0 |
| 14 Oct | slower than the model | own-velocity | 33 | 2.01 m | 353 | G1, G3 | 0 / 0 |
| 13 Oct | identified | commanded | 73 | inside the footprint | 382 | G1, G3, X7 -> RTL | 0 / 0 |
| 13 Oct | identified | **own-velocity** | **0** | **3.48 m** | 373 | G1 x2, G7, G3 | 0 / 0 |
| 13 Oct | as slow as the model | commanded | 74 | inside the footprint | 381 | G1, G3, X7 -> RTL | 0 / 0 |
| 13 Oct | as slow as the model | own-velocity | 0 | 3.16 m | 373 | G1 x2, G3 x2, G8 | 0 / 0 |
| 13 Oct | slower than the model | commanded | 74 | inside the footprint | 381 | G1, G3, X7 -> RTL | 0 / 0 |
| 13 Oct | slower than the model | own-velocity | 26 | 2.22 m | 373 | G1, G3 | 0 / 0 |

On the identified aircraft both approaches stay outside the margin, by 0.28
and 0.48 m (the first build: 0.21 and 0.29 m), and so they do on an
aircraft as slow as the Shield's model (the first build let that aircraft
into the margin on 26 and 19 ticks, to 2.21 and 2.50 m). On an aircraft
slower than the model the forecast is optimistic and the aircraft enters,
0.8-1.0 m deep instead of into the building, on 33 and 26 ticks against the
commanded forecast's 73 and 74 on that same aircraft, with no fail-safe.
`test_the_closed_loop_keeps_both_approaches_outside_the_margin` compares it
with the commanded forecast on the SAME slower aircraft (the first version
compared it with the fitted one). The first build (own-velocity forecast
alone) kept the fitted aircraft out by 1 and 4 cm, with 6 and 1 P0 escapes on
the old basis (full-speed commands at the wall from 3.0-3.4 m off it, one
tick at a time).

Every recorded flight, whole (the same pilot raw actions open loop, so after
the first divergence the pilot no longer "sees" the aircraft; ticks inside
any hard rule while the Shield/FSM or the RTL climb had the aircraft; P0
escapes 0 on both bases in every run; no forced hand-over of a deferred hold
in any):

| flight | aircraft | commanded: inside / fail-safe / min clearance | own-velocity: inside / fail-safe / min clearance |
|---|---|---|---|
| 8 Oct follow | identified | 102 / RTL / 2.56 | 2 / - / 3.07 |
| 8 Oct follow | as slow as the model | 99 / RTL / 2.60 | 5 / - / 3.30 |
| 8 Oct follow | slower than the model | 17 / - / 2.88 | 1 / - / 3.12 |
| 12 Oct follow | identified | 13 / - / 2.79 | 0 / - / 3.07 |
| 12 Oct follow | as slow as the model | 107 / RTL / 2.94 | 0 / - / 3.04 |
| 12 Oct follow | slower than the model | 108 / RTL / 2.81 | 8 / - / 2.92 |
| 13 Oct close, Shield-off log | identified | 76 / RTL / footprint | 0 / - / 3.18 |
| 13 Oct close, Shield-off log | as slow as the model | 76 / RTL / footprint | 0 / - / 3.17 |
| 13 Oct close, Shield-off log | slower than the model | 76 / RTL / footprint | 31 / - / 2.09 |
| 13 Oct close | identified | 75 / RTL / footprint | 0 / - / 3.50 |
| 13 Oct close | as slow as the model | 75 / RTL / 0.94 | 0 / - / 3.02 |
| 13 Oct close | slower than the model | 75 / RTL / 0.14 | 20 / - / 2.46 |
| 14 Oct, Shield-off log | identified | 75 / RTL / 1.58 | 0 / - / 3.72 |
| 14 Oct, Shield-off log | as slow as the model | 21 / - / 2.40 | 0 / - / 3.25 |
| 14 Oct, Shield-off log | slower than the model | 0 / - / 3.08 | 0 / - / 3.43 |
| 14 Oct | identified | 90 / RTL / footprint | 0 / - / 3.29 |
| 14 Oct | as slow as the model | 90 / RTL / footprint | 0 / - / 3.03 |
| 14 Oct | slower than the model | 87 / RTL / footprint | 33 / - / 2.27 |

Fewer ticks inside a hard rule in 17 of the 18 runs and none in either arm
of the other (the 14 Oct Shield-off log on the slowest aircraft), no P0
escape in any on either basis, no fail-safe in any own-velocity run (the
first build had two N-in-T RTLs, G4), and on the identified aircraft and
the matched one the building's 3 m kept in every flight. Not all of it is
free: the earlier interventions add Brake/Loiter spells. On the 8 Oct follow
the own-velocity FSM goes to Loiter (G3) six times and back (G8) seven
times on the identified aircraft, where the commanded run went once - but
that run ended in an RTL a third of the way through; on the slowest
aircraft, where both arms fly the whole flight, it is 7 Loiter entries
against 6. Expected of a Shield that sees the wall a second earlier, and the
number to watch in a flight. (`inside` counts every hard rule, so a run can
count a tick inside and still keep 3 m from the building.)

## 7. The hold that stops clear

On this rail the FSM's Loiter is flown as ArduCopter BRAKE (ArduCopter's
LOITER sinks with no pilot). `guardrail.response.hold_plan` predicts BRAKE's
stop path from the measured velocity and the pipe (the envelope above) and
asks the Shield, pose by pose, whether a stop there breaks a hard, enforced
rule (`stop_breaks`, the FSM's `stop_illegal` reading). If it does - or the
aircraft is still faster than 0.5 m/s (`HOLD_HANDOVER_MPS`: in the first
build's closed loop a stop path predicted clear at 1.4 m/s ended 1.6 cm
inside the margin, where BRAKE then held it; with the current numbers the
gate changes nothing on the two approaches, and it is kept as a margin on
an envelope of two samples) - the rail does not ask for BRAKE: it stays in
GUIDED and streams the Shield's answer to a stop (`advise` of a zero
action), asks again every tick, and hands over to BRAKE once BRAKE's stop
would end legal and slow. The FSM's "brake" setpoint is flown the same way
(the Shield's answer to a stop instead of a raw zero). The FSM's states,
edges and inputs are untouched: this is how the rail flies its Loiter, as
`--hold-mode` already chooses between BRAKE and LOITER. Rows record
`hold_deferred`, `stop_advised`, `fsm_setpoint` (the FSM's own) and
`autopilot_set_mode` (what was actually asked: None while deferred, BRAKE
at the hand-over); `metrics.hold.stop_clear` lists every plan.

**A deferred hold is flown by the stream** (review). While the Loiter is
held in GUIDED, BRAKE has not been asked for, and if the node stops
streaming ArduCopter keeps flying the last GUIDED setpoint until its own
GUID_TIMEOUT (3 s); under the old flow BRAKE held the aircraft with no
companion involvement. So the hold goes to BRAKE whatever the plan says on
a stale link (the stale branch asks for it at once), on a gap in the stream
over `HOLD_STREAM_GAP_S` (0.3 s, three periods; the 14 Oct log has a 0.73 s
gap), and after `HOLD_DEFER_MAX_S` (3 s: GUID_TIMEOUT itself, and longer
than any deferral the closed loop needed - 1.84 s at most over every
recorded flight on the three aircraft). Each such hand-over is a plan with
`forced` (link_stale, stream_gap, deferral_cap) and is counted in
`metrics.hold.stop_clear.forced_handovers`. None happened in any closed-loop
run.

**On record.** On a stop-advised tick the action sent is the advised
decision's, not the decision on the pilot's action, and the first build
wrote only the latter to `audit.jsonl` - the decision behind the action
ArduCopter flew had no record, and a row mixed the two (review). Now that
decision has its own audit record on the same tick, always written, tagged
`decision_kind: "stop_advised"` (a new optional field of `AuditRecord`,
absent from every other record, so they are written exactly as before), with
its repairs and repair passes; the row carries it as `stop_advice` (emitted,
violations, repairs, emitted_violations, braked, touched), and its
`emitted_violations` re-checks `flown_action`, which on those ticks is
`stop_advice.emitted`. Rows also carry `p0_escape_commanded_basis` (the
flown action re-checked from the position on the commanded forecast,
`Shield.recheck_commanded`), and `metrics.shield_forecast` counts it, so a
flight with the velocity is comparable like for like with every stored run.
`tests/test_own_velocity_forecast.py` re-derives that field row by row with
a Shield that never saw a velocity, on a tick that escapes on both bases
(True) and on one that escapes on the rail's basis only, through momentum
(False).

**Read back** (re-review). A tick with two records was new to every reader.
`tools/export_episodes.py` split an audit into flights wherever the tick
did not advance, so each stop-advised tick started a new "flight" and the
WP4 export joined the best single fragment: on the hold rail run of the
test file 22 "flights" and 40 of 60 rows joined, on its FSM-brake run 61
and 2 of 60, the per-step policy state and the priority / category labels
lost with no trace but a lower `joined`. Now a record with a
`decision_kind` right after its tick's untagged record is that tick's
second decision: the tick's filter() record is joined (the stop's raw
action, a zero, is never compared with the row), and the stop's decision
rides on the step as `stop_advice`; both runs join 60 of 60 in one flight.
`demo/track_truth.score_standoff_firings`, scored from an audit, skips such
a record (its violations are the stop's, not those the row holds). The
replay bundle's checks (episode id, policy hashes) read every record and
need no change; the KPI rescoring reads the flight log, not the audit.

**On screen.** The HUD (an ITRI deliverable) said "FSM Loiter(BRAKE) -
AUTOPILOT HOLDING POSITION" while the aircraft was still in GUIDED and the
node streamed the Shield's stop, and on a stop-advised Brake it described
the repair of the raw action that was not flown (review). A deferred Loiter
now reads "FSM Loiter(GUIDED) - STOP THROUGH SHIELD (BRAKE WOULD END IN
bld-clearance)" (or the speed BRAKE waits for), the mode label says
Loiter(GUIDED), and a stop-advised Brake shows the advised decision: "STOP
THROUGH SHIELD (A ZERO WOULD END IN <rules>)", or the zero-setpoint banner
when the advice was the zero itself.

## 8. The rails

| rail | velocity | default | what it records |
|---|---|---|---|
| `demo/pas_ardupilot` (pymavlink) | LOCAL_POSITION_NED vx/vy/-vz, else GLOBAL_POSITION_INT's, fresh within 0.3 s (`autopilot.snapshot_velocity`) | **on** (`--own-velocity-forecast on`), Shield-on arm only | rows `state_v`, `state_v_source`, `shield_velocity`, `hold_deferred`, `stop_advised`, `stop_advice`, `fsm_setpoint`, `p0_escape_commanded_basis`; audit records `decision_kind: "stop_advised"`; `metrics.shield_forecast` (incl. `noted_setpoints`, `velocity_set_aside_ticks`, `p0_escape_ticks_commanded_basis`), `metrics.hold.stop_clear` (incl. `forced_handovers`) |
| `demo/pas_ardupilot` (MAVROS link) | `/mavros/local_position/velocity_local` (ENU swapped) | on, as above | as above |
| `sitl/ros2_shield_node.py` | `/mavros/local_position/velocity_local` | **off** | rows `state_v`, `shield_velocity`; `metrics.shield_forecast` |
| `sitl/run_sitl_demo.py` | LOCAL_POSITION_NED of the pose it flies on | **off** | as above |

On for the Project AirSim + ArduPilot rail because the evidence the task set
holds there, on both P0 escape bases: replayed open loop on all six recorded
flights the Shield's P0 escapes are the same ticks with and without it
(re-checked with the velocity or on the commanded forecast alike; this holds
because the velocity is set aside from an illegal position, section 2), and
in closed loop the position was illegal on fewer ticks in 17 of 18 runs and
equally (never) in the other, with no P0 escape and no fail-safe in any. The
harness (experiments/sweep_scenarios.py) passes no velocity: the smoke
profile's summary is identical to c77c77d's field for field, timings and
paths aside. The rail ages its pipe on its loop clock (`pipe_clock`). Off on the two SITL rails because the response was measured on the
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
  row carries `p0_escape_commanded_basis` (and `state_v`, `flown_action`),
  and `metrics.shield_forecast.p0_escape_ticks_commanded_basis` counts it,
  so a non-zero result is comparable like for like with the stored runs.
* **The response is a model of one form, fitted to six flights of one
  airframe in simulation.** The closed loop's aircraft is the identified
  one, not the Shield's model, but it has the same form; an aircraft whose
  response has another form (a different controller, wind, a real airframe)
  is not tested here. On an aircraft slower than the Shield's model
  (0.55 s, 1.5 m/s^2) the forecast is optimistic and the aircraft entered
  the margin 0.8-1.0 m deep on 26-33 ticks of the approaches.
* **BRAKE is an envelope of two entries, neither clean** (section 3: one is
  at the building's face, speed rising inside BRAKE). It errs towards
  GUIDED, the long side. **Open item for the lead / PI:** a clean SITL BRAKE
  step test (GUIDED at 1, 2, 3 and 4 m/s in open air, then BRAKE; log the
  velocity) to replace it with a fit.
* A rule the stop path breaks is judged from the stop point (and from the
  position on the commanded forecast); a thin zone the stop path crosses and
  leaves is then judged from beyond it. No shipped policy has a zone thinner
  than a stop from the speed cap (8 m from 5 m/s).

* The committed-rule judgement places a rule's check at the stop point at
  forecast time 0; a time window or a moving zone is read at the forecast's
  own times from there, not at the time the aircraft reaches the stop point.
* The stand-off forecast uses the aircraft's response; the subject's own
  velocity is the separate `set_subject(velocity=)` (no rail passes it).
* While the position already breaks a hard rule, the whole decision is the
  old one (the velocity set aside, section 2): the push is sized from the
  depth (e.g. ClearanceFix's 0.5-0.67 m/s), not from the momentum still
  carrying the aircraft deeper. The 14 Oct creep was in that regime; the
  change is that the Shield should no longer get there on the identified
  aircraft, nor on one as slow as its model.
* **The set-aside is all or nothing across rules** (section 2): any broken
  hard rule - `alt-band`, seven mid-follow dips below 6 m on the 8 Oct
  flight, 279 of its 310 set-aside ticks - drops the momentum for every
  other rule. **Open item:** set aside only the rules whose committed
  judgement conflicts with the escape. Not done: setting aside only the
  broken rule would have flown the 7 escapes of section 2 again (the
  conflicting rule there was the unbroken stand-off ring), and choosing the
  conflicting rules needs its own replay and closed-loop evidence.
* **The braking-bias rule is a median at 1.5 s** (section 3), not a bound:
  p25 -0.33 m there, median -0.16 m at 2.0 s.
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
.\scripts\run_pas_ardupilot.ps1 -Step node -Seconds 120 -SaveView -Tag pas_ap_ovf_on_<date> -NodeArgs "--want-width","0.5"
# the same build with the old forecast and BRAKE hold, for an A/B:
.\scripts\run_pas_ardupilot.ps1 -Step node -Seconds 120 -SaveView -Tag pas_ap_ovf_off_<date> -NodeArgs "--want-width","0.5","--own-velocity-forecast","off"
```

Compare with `pas_ap_video_on_20261014`: the rows with `unsafe` true while
the Shield or the FSM flew (71 there, 20 of them before the RTL) and the
deepest `unsafe_depth_m` (1.0 m); `metrics.fsm.transitions` (there: G1 at
tick 363, X7 -> RTL at 388; expected: no X7); `kpi.json` `p0_escapes`
(0 there) beside `metrics.shield_forecast.p0_escape_ticks_commanded_basis`
(the same count on the stored runs' basis); the first row whose
`violations` cite `bld-clearance` against the tick the aircraft is 4.6 m
from the wall (expected ~1 s earlier); `metrics.shield_forecast` (`mode`
own_velocity, `velocity_ticks` close to `ticks`, `velocity_sources` mostly
`local_velocity`, `noted_setpoints` equal to the ticks that sent one,
`velocity_set_aside_ticks` small); and `metrics.hold.stop_clear` (each plan,
`guided_hold_ticks`, `stop_advised_ticks`, `forced_handovers` - expected
empty); every `stop_advised` row has its `decision_kind: "stop_advised"`
record in `audit.jsonl`; and `python tools/export_episodes.py
demo/out/pas_ap_ovf_on_<date> --out <dir>` reports `audit_join` with
`flights_in_audit` 1 and `stop_advice_records` equal to the stop-advised
rows (the exporter before the re-review split such a run). The approach
depends on where the car leads the follow, so a run that never comes near
the building says nothing either way.
