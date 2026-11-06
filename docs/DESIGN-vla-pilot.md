# The VLA pilot: AerialVLA behind the Shield

Status, 2026-10-21: the pilot side is built and tested offline
(`guardrail/vla_link.py`, `guardrail/vla_backends.RemoteVLABackend`,
`tests/test_vla_link.py`, `tests/test_vla_backends.py`). Nothing here has
flown, and no model was loaded for it. The server side (`demo/vla_server.py`,
protocol v2) is a separate change. Wiring the backend into a rail
(`perception_node --pilot vla`, the ROS 2 `/vla/action_4d` bridge), the HUD
panel and the manifest's `pilot_record(kind="vla")` come after both.

2026-10-22, wave V2: the CityLife flight that puts this pilot in the air is
built (`demo/vla_fly.py`, `demo/vla_mission.py`, section 13), with its
policy, cameras, launcher and offline replay gate. It is tested without the
simulator (`tests/test_vla_fly.py`, `tests/test_vla_replay_gate.py`) and was
driven end to end against a fake `projectairsim` module with the server's
scripted stand-in; it has NOT flown in Unreal and has not flown the model.

2026-10-23, wave V2b: the same pilot on the Project AirSim + ArduPilot rail
(`demo/pas_ardupilot/perception_node.py --pilot vla`, section 14), with
AerialVLA's two cameras added to that rail's robot config. Tested with the
rail's stand-ins against the scripted server (`tests/test_pas_vla.py`); it
has NOT flown ArduPilot, Unreal or the model.

2026-10-23, wave FT: the follow's step-tracker smoothing (section 13,
"Smoothing the follow") and a format-exact fine-tune of the adapter for the
follow, its evaluation tool and the server's adapter identity (section 15).
Tested offline (`tests/test_vla_follow_smooth.py`,
`tests/test_finetune_follow.py`); neither has flown, and no training run has
been made.

2026-10-22: the review of 7c7cc50 found nine problems; all are fixed, each
with a failing-first test. The ones that change behaviour: a server error is
no answer (section 5), the client re-reads `raw_text` and pins the model
identity (section 10), the log is written off the loop thread and nothing
from the link, the log or the caller's callbacks escapes `act()` (section
9), the goal phrase is never a default (section 1), and a policy never
widens or drops a configured altitude band (section 3).

## 1. Who flies

| Layer | What it decides | Code |
|---|---|---|
| AerialVLA (OpenVLA-7B + the original `aero_vla` LoRA, own process) | every step: which way to turn, how far forward, how far up or down, and when it has arrived (LAND) | `demo/vla_server.py` |
| The hint | one of eight training phrases, from the GOTO goal (`hint_source: goal`) or the OWL-ViT box bearing (`detector`, `demo/vla_bridge.py`), or none | `vla_link.goal_hint`, `vla_bridge` |
| The step tracker | how each step is flown at 10 Hz: a deterministic interpolator, not a planner | `vla_link.StepTracker` |
| The Shield | whether each 10 Hz action is legal, and its repair | `guardrail/shield.py` |
| The escalation FSM and the rail | holds, Loiter, RTL, Land; the autopilot link | `guardrail/fsm.py`, the rails |

The hint's source is always named by the caller: `make_backend("aerovla",
..., get_hint=...)` for a detector phrase, or `hint="goal"` for a GOTO
mission. Until 2026-10-22 the goal phrase toward `mission.target_x/target_y`
was the silent default for any mission, so a follow or approach mission
wired without its detector hint would have flown on an oracle direction
labelled as a GOTO goal. The factory now refuses a call that names neither.

The model has no language grounding. Its `{object}` slot is inert and only
the compass phrase steers it (`docs/FINDING-what-drives-aerialvla.md`). So:

| Say | Do not say |
|---|---|
| "AerialVLA chose every step at X Hz from the front+down camera mosaic and a compass phrase computed from the goal (or from the OWL-ViT box); a deterministic step tracker flew each step at 10 Hz; the Shield filtered every tick." | "The VLA follows the red car by language." "The VLA plans the route." "The VLA runs at 10 Hz." |
| "Y % of ticks were VLA-step ticks" (from `Tick.author`) | "VLA-flown" for a run with no step ticks, or one where the tracker hovered |

## 2. What an answer means (upstream step semantics)

Each answer is three numbers in 99 bins, plus an optional LAND token
(`"FF DD YY"` or `"FF DD YY LAND"`, then EOS):

| Field | Range | Meaning | One bin |
|---|---|---|---|
| `fwd_m` | 0 .. 5 m | forward displacement | 0.051 m |
| `down_m` | -5 .. 5 m | vertical displacement, positive DOWN | 0.102 m |
| `dpsi_rad` | -1.1 .. 1.1 rad | heading change, positive clockwise | 0.0224 rad |

Upstream executes it as: new heading = heading + dpsi; if |dpsi| >= 0.25 rad,
rotate only (and change altitude); otherwise move `fwd_m` along the new
heading and `down_m` vertically at 1.0 m/s; then hover 0.5 s (upstream
`airsim_plugin/AirVLNSimulatorClientTool_AeroVLA.py:300-385`; arXiv
2603.14363, action space).

Our code before this change read the same numbers as forward m/s, down m/s and
yaw rad/s and held them for 7-12 s (`demo/aerialvla_demo.py:384-405`,
`demo/semantic_seek.py:172-224`). That misreading, not only the decision rate,
explains the spinning, the stale yaw and the 0-1 of 5 pure-VLA reach rate of
the earlier runs (`deploy/evidence/vla_backend_table.md:84-91`). Nothing in `vla_link` reads them as velocities, and
`tests/test_vla_link.py` fails if the tracker flies `fwd_m` as a speed.

## 3. How the 10 Hz contract is met: the step tracker

The grant's contract is a 4-D action at 10 Hz into the Shield
(`kuanting-vla-uav-guardrail/docs/02-implementation/safety-shield.md:13`:
"Monitor rate: 10 Hz, matching the VLA action rate"). A 7B model does not
decide at 10 Hz: 0.35 s per decision on an RTX 4090 in the paper, 2.5-12 s
measured here on the old RTX 4080 SUPER. The tracker fills the gap, and this
must be disclosed wherever the rate is reported: **the action stream is 10 Hz;
the decisions behind it are not.**

Each accepted step gets fixed world-frame targets, at the pose its image was
CAPTURED at:

    heading   psi_t = psi_capture + dpsi                        (wrapped to [-pi, pi))
    position  p_t   = p_capture + fwd_m * (cos psi_t, sin psi_t)   (none if |dpsi| >= 0.25)
    altitude  up_t  = up_capture - down_m, clamped into the policy's hard altitude band

Per tick (any rate; the rails run 10 Hz):

    yaw_rate = clip(k_yaw * wrap(psi_t - psi), +-yaw_rate_cap)
    v_h      = toward p_t at min(cruise, k_pos * distance), zero once p_t is
               within pos_tol or behind the aircraft along psi_t
    vz_up    = clip(k_alt * (up_t - up), +-climb_cap)

| Constant | Default | Source |
|---|---|---|
| `cruise_mps` | 1.0 | upstream's `CRUISE_SPEED` |
| `speed_cap_mps` | 2.0 | ours; cruise may not exceed it |
| `climb_cap_mps` | 1.0 | upstream moves vertically at 1.0 m/s |
| `yaw_rate_cap_rad_s` | 0.5 (28.6 deg/s) | ours; a full 1.1 rad turn takes about 2.2 s |
| `k_yaw`, `k_pos`, `k_alt` | 1.5, 1.0, 1.0 per s | ours |
| `pos_tol_m`, `alt_tol_m`, `yaw_tol_rad` | 0.3, 0.3, 0.05 | ours |
| step time cap | 1.5 x expected duration + 2 s, at most 15 s | ours; a 5 m step gets 9.5 s |
| `decision_timeout_s` | 10 s | ours; see section 6 |
| `alt_band_inset_m` | 0.5 (or a quarter of a thin band) | the hold-point planner's inset rule (`guardrail/holdpoint.py`) |

Why anchor at the capture pose: with pipelining an answer can arrive after the
aircraft has turned or moved. Anchoring there keeps psi_t correct;
re-anchoring at arrival would add every turn made since the capture a second
time (a target 40 deg to the right, seen twice during one turn, would become
80 deg).

Why never backwards: AerialVLA's forward axis is 0 .. 5 m. A step whose
endpoint is already behind the aircraft (the image was taken a moment earlier)
counts as reached; the tracker does not invent a reverse the model cannot
command.

Why clamp the altitude target: the pilot should not ask for a height the
Shield would refuse. The Shield still enforces the band. The unclamped target
is recorded beside the clamped one (`target.up_raw`, `target.alt_clamped`, and
the `alt_clamped` count), so a model that keeps diving shows in the record
even though the Shield's altitude rule then rarely fires on VLA ticks.

Which band: the intersection of the policy's hard altitude envelopes measured
above ground (`vla_link.policy_alt_band`), combined with any band the caller
set in `TrackerConfig.alt_band` (`vla_link.combine_alt_bands`): the tighter
of the two, so a policy never widens a configured band and a hot-applied
policy without an envelope never removes it. An envelope with `altitude_ref:
MSL` is skipped and named (`skipped_msl`): the Shield refuses MSL envelopes
(no ground-elevation source), and clamping its 120-300 m as heights above
ground would be wrong. A configured band that misses the policy band entirely
is refused when the backend is built; on a hot-apply the policy band wins
(`source: policy_disjoint`, counted), because the Shield enforces the policy.
`stats()["alt_band"]` records the band, its source, both inputs and the
skipped envelopes. (Until 2026-10-22 the policy band replaced the configured
one, MSL envelopes were clamped as above-ground heights, and a policy without
an envelope dropped a configured clamp.)

What the tracker does NOT do: choose a direction the model did not choose,
avoid anything, look at the policy's zones, blend in a planner or a servo.
There is no `goal_blend`. The only policy input is the altitude band.

## 4. Cadence: pipelined and stop-and-go

One request is outstanding at a time, by construction (`RemoteVLA.submit`
returns None while one is in flight or its result is still unread; the fake
server counts overlaps, and they stay at 0). The unread-result rule closed a
loss: until 2026-10-22 a second request could go out between `act()`'s poll
and its submit, and when that request came back within one tick (an instant
refusal, a server that died) the worker overwrote the first result, a step or
a LAND that never reached the tracker.

- **pipelined** (`RemoteVLABackend`'s default; a labelled arm of the CityLife
  flight): the next image is captured and sent as soon as the previous answer
  is back, while that answer's step is being flown. The newer step supersedes
  the older on arrival. With 0.35 s answers the aircraft flies continuously
  toward a target a few metres ahead; with 3 s answers most steps complete
  before they are superseded. Each answer's turn and climb are applied at the
  DECISION rate, not once per step as in training (section 13 has the
  measured difference).
- **stop_and_go**: upstream's own cadence, and the CityLife flight's default
  since 2026-10-23 (`demo/vla_fly.py --cadence`, `-Cadence` in the launcher).
  The next image is taken only after the step is complete and the tracker has
  held for 0.5 s. No step is cut short.

Both arms reach a goal 36 m away and 6 m up in the closed-loop tests, at 0.4 s
and 3 s of latency, from steps quantized to the model's 99 bins, and end on
the model's own LAND (`test_a_point_vehicle_reaches_the_goal_from_scripted_steps`;
end to end through TCP in `test_the_backend_flies_a_goto_through_the_link_and_lands`).
The scripted model in those tests is perfect by construction; they test the
pilot plumbing, not AerialVLA.

## 5. Hold policy and authorship

Every tick says who authored it (`Tick.author`, and `Tick.as_row()` for the
per-tick rows):

| Author | When | Action |
|---|---|---|
| `vla_step` | flying toward an accepted step's targets (`reason`: rotate / translate; a follow's executor flying a big turn while moving: `turn_moving`) | the tracker's law above |
| `hold` | the step is complete (`step_complete`); the answer was malformed (`parse_fail`); the model said LAND (`land`; a follow holds it as arrival instead: `land_as_arrival`) | zero, except a completed step keeps its altitude target; parse_fail and LAND are all-zero |
| `hover` | no usable decision: the step's image is too old (`stale`), a hold lasted past the timeout (`no_new_decision`), the link failed (`link_lost`), the server refused the request (`server_error`), or a rail hold invalidated it (`awaiting_fresh_decision`) | all-zero |
| `none` | no decision accepted yet (`no_decision`, or `link_lost` / `server_error` when that is why), or the rail has the tracker suspended (`suspended`) | all-zero |

A malformed answer HOLDS ZERO and is counted: it replaces the step being
flown, because a misread answer is no licence to keep flying the previous one.
(The integration discovery report proposed letting the previous decision keep
fading; the brief's rule, adopted here, is the stricter one. Its yaw-fade and
"forward only while the box is fresh" rules were for velocity semantics; a
step is a bounded displacement, so there is nothing to fade.)

A server error (`ok: false`) is NOT an answer: the model never saw the
request. It is refused (`refused: server_error`, counted `server_error`),
ends the step being flown like a link failure (`end: server_error`), and the
tracker hovers; with `hover_on_link_loss` off it leaves the step alone, as a
link failure does. The client backs off `reconnect_backoff_s` before the next
request, on the same connection. Until 2026-10-22 a server error was accepted
as a `hold` and reset the hold timer, with no backoff: a fake server answering
every request with "CUDA out of memory" for 30 s gave 299 `hold` ticks, 0
`hover`, 300 requests and a reported decision rate of 9.97 Hz. A dead model
looked like a pilot holding on purpose, and the first field mismatch between
the server and the client (the server refusing `unknown field` on every
request) would have looked the same. Now that run gives 300 `none` ticks
(reason `server_error`), 30 requests and a decision rate of 0
(`test_a_server_that_refuses_every_request_never_looks_like_a_pilot`).

## 6. Timeouts

| Timeout | Default | What it bounds |
|---|---|---|
| `decision_timeout_s`, step | 10 s | the age of the image behind a step being flown (capture to now). Past it the step is dropped (`end: stale`) and the tracker hovers. An answer already older on arrival is refused (`stale_on_arrival`). A 5 m step at 1 m/s plus about 3 s of latency fits. |
| `decision_timeout_s`, hold | 10 s | how long a hold lasts waiting for the next answer, measured from the hold's start. Then hover (`end: hold_timeout`). |
| client response timeout | = `decision_timeout_s` (factory) | one request's round trip; a later answer would be refused as stale anyway. On timeout the connection is dropped - unless `late_reply_wait_s` is set (the CityLife flight sets 120 s): then the timeout is still posted at once, but the client keeps the connection and sends nothing until the abandoned request's late answer arrives, because the server cannot cancel a `generate()` and a new request would queue behind it on the model lock (`RemoteVLA`, `submit_draining`, `late_replies`). |
| connect timeout | 2 s | opening the TCP connection |
| reconnect backoff | 1 s | no new request this long after a failure or a server error |
| step time cap | per step (section 3) | a step that cannot reach tolerance (wind, a Shield repair holding it back) ends and holds |

The hold's timeout is measured from the hold's start, not from the capture.
The first version measured both from the capture; the stop-and-go closed loop
at 3 s of latency then turned 8 of 9 holds into hovers while the next answer
was on its way, because in that cadence the next image is taken only after the
step completes. `test_a_decision_past_the_timeout_is_never_flown` pins both
rules.

These are starting values. They must be re-set from the measured p95 latency
on the RTX 4090 (experiment E0 in the integration plan), with the renderer
running.

## 7. Rail holds and LAND

`suspend()` when the rail takes the aircraft (an escalation hold, Loiter, a
Shield hold); `resume()` when it gives it back. The step being flown ends
(`suspended`), and every answer to an image captured before the resume is
refused (`invalidated`), including the one in flight: it describes a scene
from before the hold. The tracker hovers until an answer to a fresh image
arrives.

LAND is a flag, never a mode request. The tracker stops (all-zero, `hold` /
`land`), latches `land_requested`, refuses later answers (`after_land`) and
the backend stops sending. The rail hands the flag to the escalation FSM's
Land or its own landing. `test_the_module_can_never_request_a_flight_mode`
reads the source of `vla_link` and `RemoteVLABackend`: no identifier that
talks to an autopilot or names a mode, no mode name as a string constant, no
MAVLink or ROS import.

## 8. What the Shield and the FSM do with a slow pilot

- **The Shield** judges every 10 Hz tracker action from the current state with
  its 5 s / 0.1 s forecast. A step that heads into a no-fly zone or a building
  margin is repaired on every tick it would breach, whatever the decision rate.
  The tracker's actions are bounded (cruise 1 m/s, climb 1 m/s, yaw 0.5 rad/s),
  so the kinematic caps rarely fire on them.
- **Unmapped obstacles** (trees, street furniture) are invisible to the Shield
  (`demo/vla_bridge.py:132-137`). The tracker's speed cap is the only defence.
- **The FSM's N-in-T** (N=3 onsets in T=5 s, `guardrail/fsm.py` defaults)
  counts violation onsets. A pilot that sends one step every few seconds into
  a fence produces one sustained violation (one onset) or one onset per step,
  so N=3 in 5 s rarely trips. A proposed extension, not built: escalate on K
  consecutive repaired decisions, or scale T by the measured decision period
  through the FSM's YAML. That is a PI decision.
- **Silence that reads as success**: on the ROS 2 rail a republished held
  action keeps the Shield node's 0.5 s receipt watchdog green while the model
  is dead. The decision's age and sequence number must travel with the action
  (`Tick.seq`, `Tick.age_s`); the bridge that publishes them is not built.

## 9. Records

`vla_steps.jsonl` (schema `vla_steps/1`), one LF-terminated line per request,
written when the decision stops being flown so the line carries what became of
it:

- request: `seq`, `t_capture`, `t_sent`, `t_received`, `rtt_s`,
  `latency_client_s` (capture to answer), `pose_capture`, `phrase`,
  `hint_source`, `object`, `csp_mode`, `csp_text`, `csp_sha256`, the JPEG
  hashes and sizes (not the bytes);
- answer: `kind` (step / land / parse_fail / server_error / link_failure,
  or unanswered for a request still in flight when the run closed),
  `ok`, `error`, `raw_text`, `bins`, `step`, `land`, `parse_ok`,
  `latency_server_s`, `prompt_sha256`, `image_sha256`, `model`, and
  `failure` for a link failure;
- fate: `accepted`, `refused` (server_error / suspended / invalidated /
  after_land / stale_on_arrival / future_capture / link_failure / closed /
  drain_error, the last with the error's text in `drain_error`),
  `target` (psi, x, y, up, up_raw, alt_clamped, rotate_only, time_cap_s),
  `outcome` (t_start, completed, completion tolerance / time_cap,
  t_complete, end superseded / stale / hold_timeout / link_lost /
  server_error / suspended / land / closed, t_end, ticks by author).

`RemoteVLABackend.stats()` gives, for `metrics.json`: `decisions_accepted` and
`decision_hz`, which count ONLY answers the model parsed (a step or LAND) that
the tracker took; `answers` (every reply to a request sent, by kind, accepted
or not); `parse_fail`, `server_error` and `link_failure` with their own rates;
the error counts below; the altitude band and its source; refusals, ends,
ticks by author and the link's counters. (Until 2026-10-22 the decision rate
counted every accepted reply, server errors and malformed answers included.)

The log's counts are `log_written`, `log_errors`, `last_log_error` and
`log_pending` (records queued and not yet written or failed). With no
`steps_log` all four are None: nothing was measured, and until the second
review of 2026-10-22 a 0 said the same as a log written without a fault.
stats() reads them without waiting for the writer thread (it may run on the
loop), so `log_errors` is final only when `log_pending` is 0: after `close()`.

Nothing here may stop the 10 Hz loop:

- `StepLog.write()` only queues the record; a writer thread serializes and
  writes it, and a failure (an `OSError`, a record that is not JSON) is
  counted (`log_errors`, `last_log_error`), never raised. `close()` drains
  the queue first. `RemoteVLABackend.records` keeps every record in memory as
  well. Until 2026-10-22 the line was written and flushed on the loop thread
  inside `act()`, and a full disk (D: runs near full) raised out of `act()` at
  the second tick. The rails keep file IO off the loop the same way
  (`demo/follow_vlm.py` writes detections on its worker thread and
  `flight_log.jsonl` in its `finally`).
- An exception from the caller's `get_capture` or `get_hint`, or from building
  the request (`Hint` and `build_infer_fields` refuse bad values by raising),
  is counted (`request_errors`, `last_request_error`) and the tick still
  returns the tracker's action. A reply that cannot be handed to the tracker
  is counted (`drain_errors`, `last_drain_error`) and treated as a lost link
  (`none` before any decision, `hover` after one), and its request keeps its
  line (`refused: drain_error`; the tracker owns the record once it has the
  decision). A reply to a seq never sent is counted the same way and is not
  an answer. Until the second review of 2026-10-22 the guard had no test and
  the request's line was lost.
- The client's worker thread posts a result for every request whatever goes
  wrong in it (`failure: internal` for anything unexpected), so it never dies
  with a request outstanding and every request keeps its line.

## 10. Protocol v2, the client's half

TCP (default 127.0.0.1:8790), one JSON object per line, UTF-8, one request to
one response, many requests per connection; `hello` on every new connection.

The first hello's `model` is pinned as `RemoteVLA.identity`. It must be the
brief's identity: `id` and `base` non-empty strings, `weights_sha256` 64
lowercase hex digits, `precision` nf4 or bf16, `attn` eager or sdpa, `merged`
true or false (serving facts beside these are kept). Every answer's `model`,
and the hello of every reconnect, must name the same six fields; otherwise the
reply is a protocol failure counted `model_changed`, so a server restarted
with other weights stops the run's decisions instead of mixing two models in
one record (the dashboard classifies runs by `weights_sha256`). A scripted
stand-in with no weights is accepted only with `RemoteVLA(...,
strict_identity=False)`, said out loud by the caller.

The client refuses to send a request with any field the protocol does not name,
and refuses `target`, `goal`, `object_position`, `xy`, `lat` and `lon` by name.
The phrase must be one of the eight training strings, exactly (trailing space
included); `hint_source: none` sends only `""`; a detector hint is never `""`
(no detection, no request: `demo/vla_bridge.py`). `csp_mode: off` sends no CSP
text; `csp` and `neutral` need it.

A reply is refused (a `protocol` failure: the connection is dropped and the
answer is never a decision) when the version or sequence number is wrong, a
field is missing, a step is outside AerialVLA's range, `parse_ok` is false but
a step or LAND came with it, the bins are not three integers 0..98, the echoed
`t_capture` or pose is not what was sent, a hash is not 64 hex digits, the
model is not the pinned identity, or the line is not JSON (NaN included, and
nesting too deep for the JSON reader).

The client does not take `parse_ok: true` on trust. The brief's headline bug
was a server-side parser (`ints[-3:]`) that turned malformed LAND outputs into
motion; a server still running one would otherwise be flown. An answer marked
`parse_ok` is refused unless `raw_text` IS that answer: exactly three 2-digit
ASCII bins separated by single spaces, `" LAND"` exactly when `land` is true,
optionally the EOS marker `</s>` (the server decodes with special tokens),
nothing else; the three bins equal `bins`; and each step value lies within
half a bin of its bin's value (`b / 98 * (hi - lo) + lo`, the lower edge of
the bin's cell, upstream's decoder). So `"00 49 49 49"` (four numbers) with
`parse_ok: true` and `"40 49 49 LAND"` with `land: false` are both refused,
where until 2026-10-22 the first flew as a 2.5 m forward step and the second
as motion instead of LAND.

`ok: false` with a reason is a server error: no answer (section 5), counted,
backed off, not a link failure.

## 11. Open questions for the PI

1. **A decision rate below 10 Hz.** The grant pairs the Shield's 10 Hz monitor
   rate with "the VLA action rate" (`safety-shield.md:13`), and gate G3 would
   swap a 7B backend that cannot produce actions at 10 Hz for a smaller one
   (`kuanting-vla-uav-guardrail/docs/04-risks-and-fallbacks/fallback-gates.md:61-63`).
   We asked on 12 Oct (Q3 of that day's question list) whether a VLA deciding
   at about 1 Hz may carry the language KPIs with its rate stated; no answer is
   recorded. Our position here: a 10 Hz action stream from a disclosed
   deterministic interpolator of 0.2-3 Hz decisions, with the decision rate
   reported beside every result.
2. **The FSM for a slow pilot** (section 8).
3. Any lock-step run (simulation paused during inference) is not real time and
   must be labelled so.

## 12. Not in this change

- the VLA server, protocol v2 (`demo/vla_server.py`), and the parser fix (three
  2-digit bins plus optional LAND; anything else parse_fail);
- `perception_node --pilot vla`, the ROS 2 `/vla/action_4d` bridge with the
  decision age, the HUD's VLA panel, `pilot_record(kind="vla")`;
- the measured decision rate on the RTX 4090 (E0) and timeouts re-set from it;
- a camera pair matching training (256x256, 90 deg, front pitch 0, down pitch
  -90).

(Since then: the server is `demo/vla_server_v2.py` with the strict reader
(wave V1a); the HUD's VLA panel, `pilot_record(kind="vla")` and the camera
pair are in the CityLife flight, section 13; `perception_node --pilot vla`
and the ArduPilot rail's camera pair in section 14, built and not flown. The
ROS 2 bridge and the in-flight E0 measurement are still open.)

## 13. Flying it: `demo/vla_fly.py` (CityLife_Day, simple_flight)

### Why a new script

`follow_vlm.fly()` puts a hand-written follower's layers (FenceGuard's gate
and slide, the trail, the guard smoother, the slew limiter) between its
controller and the Shield; a VLA arm there would have to switch each off or
call the run hybrid (integration report, section 3.4). In `vla_fly` the step
tracker's action IS the raw action the Shield judges
(`vla_mission.MissionLoop`): `backend.act(state)` -> `shield.filter` ->
`move_by_velocity` (the repaired `emitted`; with `--shield off`, the control
arm, the step tracker's action unfiltered - see "The control arm" below).
Infrastructure is imported from
`follow_vlm` (heartbeat, collision topic, landing, Grounder, recorder), so a
fix there reaches here. The Shield is the grant's 5 s / 0.1 s forecast
without an FSM inside it (`guardrail/replay.rail_shield`): this rail flies
`emitted`. follow_vlm keeps 3 s / 0.5 s because its stand-off subject is
held still; since 2026-11-01 the follow here gives the Shield its subject
WITH the subject's velocity (`vla_fly.shield_subject`), which on a replay of
65 logged VLA follows took the grant forecast's stand-off repairs from 9.1 %
of ticks to 1.1 % (`tools/replay_standoff_feed.py`), so the grant's forecast
stays. Before that date no VLA flight gave the Shield a subject at all: the
follow policy's stand-offs were inert (CHANGELOG 2026-11-01 "late").

### The missions

| | M1 `goto` | M2 `approach` |
|---|---|---|
| hint | the goal's bearing from the capture pose, relative to the heading (`vla_link.goal_phrase`), `""` inside `--arrive-radius-m` (3 m): hint_source `goal`, the trained task | the OWL-ViT box on the detector's own camera through `vla_bridge.VLABridge` (gain 1: three reachable phrases), its bearing rotated by the turn between the detector's frame and the VLA's capture (`DetectorHint`): hint_source `detector`; no box fresher than `--det-fresh-s` (1 s), no request |
| ends on | LAND; inside the arrive radius 8 s without LAND (`arrived_no_land`); flown past the goal - it was inside the radius, and the goal has stayed behind and receding for `--overshoot-s` (3 s): `arrived_overshoot`, with `overshoot_m`; time | LAND; the box at 0.15 of the frame width (`close_box`); no fresh box for 15 s (`target_lost`); time |
| default | goal (43, 40), object words `vla_mission.GOTO_OBJECT_TEXT` | the scenario car parked at (48, 12), object `"an orange car"` |

Object words. Every one of upstream's 410,450 training prompts describes the
target in a sentence ("The man is walking on a road near some bushes and tall
evergreen trees, ..."); none has an empty slot. So a GOTO sends one fixed,
neutral street description in that style, `"The target is on a paved city
street lined with tall buildings, street trees and parked cars."`, never
`""`; it names nothing that is there to be found and no position
(`aerialvla_codec.validate_object`, checked before take-off), it is inert
like any object words, it is recorded in metrics.json (`mission.object`) and
in every vla_steps.jsonl line, and `--object` overrides it.

The turn since the box (M2). The box can be up to `--det-fresh-s` older than
the VLA's capture; at the tracker's 0.5 rad/s yaw cap that is up to 28.6 deg
of turn, enough to move a box from "straight ahead " to "forward-left ".
`DetectorHint` subtracts (heading at the VLA capture - heading at the
detector's frame, from the Grounder's capture pose) from the box's bearing
before the band lookup, and records the box's age, the correction and the
detector's own latency (rows `box_age_at_hint_s`,
`yaw_correction_deg_at_hint`, `detector_latency_s`; metrics
`mission.detector_hint`).

Both: take off at the usual spawn (35, -20), climb to 10 m, a hand-written
transfer through the Shield to the mission start (42, -18), face the goal
(M1) or down the street (90 deg, M2), then the mission clock starts. Why the
transfer: two street trees in the flight band stand at (32-34, -16) and
(32-34, -12); every straight line east from the spawn passes them within
0.8-1.3 m. From the start, the line to the goal never drops below 3.97 m of
clearance on the Shield's own distance field over the 6-14 m map, the goal
has 13.5 m, both are on the street mask (`vla_mission.py`, CityLife geometry;
`tests/test_vla_fly.py` re-reads it from the map where the map is present).

The transfer must end within `--start-tolerance-m` (2 m) of the mission
start, or the mission is refused and the aircraft lands
(`start_not_reached`, exit 4); the start error and the pose the mission
really began at are recorded (`pre_mission.start_error_m`,
`start_pose_actual`).

Every mission also ends, and lands, on a VLA failure: hovering on link_lost
for `--dead-timeout-s` (10 s) - `server_slow` when the requests timed out
(the model is still computing), `server_dead` when the link failed (refused,
closed, a broken reply) - or on server_error that long, malformed answers
held that long (`parse_fail_streak`), or no parsed answer within
`--first-decision-timeout-s` (60 s, `no_first_decision`). A slow model's
first answer gets those 60 s: before its first parsed answer, timeouts do not
end the mission on the 10 s rule (until 2026-10-23 a model slower than the
response timeout ended `server_dead` after about 20 s, and the 60 s never
applied). Rows say what each link failure was (`link_failure`: timeout /
link / protocol), the pilot summary counts them (`link_failure_by_kind`,
`late_replies`). Those ends exit with code 3; an abort with 1; Ctrl+C lands
where the aircraft is, writes every record marked `interrupted` and exits
130. LAND is the tracker's flag; the landing is
`follow_vlm._fly_to_landing_site` (a landable pavement cell, through the
Shield) and its descent, never a mode request.

### Cadence: why stop-and-go is the default

The reviewer's point-vehicle probe (the real `StepTracker`, 10 Hz, 0.3 s
velocity lag, answers back 0.5 s after capture, 30 s, the same answer every
time; re-run 2026-10-23):

| constant answer | pipelined | stop_and_go |
|---|---|---|
| `96 48 49` (fwd 4.90 m, climb 0.102 m) | 59 decisions, climb +1.77 m, 29.3 m travelled | 5 decisions, climb +0.51 m, 23.5 m travelled |
| `96 49 47` (fwd 4.90 m, dpsi -0.045 rad = 2 bins) | heading -54.6 deg | heading -12.9 deg |

Pipelined applies each answer's turn and climb every time an answer arrives
(12 answers per step at 0.5 s), so a one-bin bias the model would apply once
per 4.9 m step in training is applied a dozen times: 4.2x the heading drift
and 3.5x the climb in 30 s. Stop-and-go applies it once per step, as upstream
does (step, hover 0.5 s, capture). It is the flight's default
(`--cadence stop_and_go`); pipelined stays a labelled arm. The cadence is in
metrics.json (`params.cadence`, `pilot.cadence`), kpi.json and on the HUD.
Fewer decisions follow: a stop-and-go GOTO decides about once every 6 s (a
4.9 m step, the 0.5 s hold, the answer), so the dashboard's 10-decision
floor for a VLA flight (tools/todo.py) needs about 50 m of flying - the
default goal is 58 m away.

### The control arm (`--shield off`)

The Shield still judges every tick, in shadow; nothing it decides flies. Two
things keep that arm honest (review of 2026-10-23):

- The pilot side is not held inside the policy's band either. With the
  Shield on, the tracker clamps altitude targets into the policy's hard band
  (6-14 m here); with it off the backend is built WITHOUT the policy and
  clamps only into `vla_mission.SIM_SAFETY_BAND_M` (2-40 m), recorded as
  `pilot_alt_band` in metrics.json and kpi.json (`clamp`
  `sim_safety_band`, the band the backend applied and its source). So the
  off arm flies the step tracker's action, not "the raw VLA action":
  the tracker's interpolation, its speed caps and that safety band stand
  between the model and the airframe. (The manifest keeps the grant's six
  fields.)
- The HUD and its counts read `vla_mission.hud_decision` (perception_node's
  rule): no repair and no brake, the flown action as emitted, the shadow's
  violations as escapes - never "SHIELD CORRECTED COMMAND" or an ACTING row
  for a Shield that was not in the loop. `interventions` counts corrections
  that flew (0 off); `shadow_interventions` the shadow's would-be ones.

### Cameras

`demo/pas_config/robot_vla_quad.jsonc` (M1) and `robot_vla_quad_det.jsonc`
(M2) keep the follow rail's airframe and controller and add `VLAFront`
(256x256, 90 deg, pitch 0) and `VLADown` (256x256, 90 deg, pitch -90),
AerialVLA's training cameras, captured every 0.1 s; M2 adds the detector's
768x432, -20 deg `FrontCamera`, the camera every OWL-ViT number here was
measured on (a 256 px frame would be upscaled 3x into OWL-ViT's 768 input).
Only `Chase` streams. `vla_mission.VLAFrames` pairs the newest front and
down frames by sim stamp (a lone front frame is no request: the server runs
`--missing-down refuse`), takes the capture pose from the loop's pose ring at
the frame's stamp, and logs the camera sizes (`vla_captures.jsonl`).

### The server

Started by the flight unless `--vla-server HOST:PORT` attaches to one:
`demo/vla_server_v2.py serve --precision bf16 --attn sdpa --no-merge` (the
fidelity option, 96 % agreement with the reference on the 4090 probe) or,
with `--vla-low-vram` / `-Precision nf4`, `--precision nf4 --attn sdpa
--merge` (25-33 %). The flight refuses an occupied port, waits for hello
(up to 45 min, `--vla-hello-timeout-s`: a bf16 cold load under RAM pressure
took over 15 min; it prints how long it has waited every minute), refuses a
non-VLA identity unless it started the scripted stand-in itself
(`--vla-fake-output`) or `--allow-stand-in` is given, and pins the
identity. A server the flight started is stopped on every way out (op
shutdown, then terminate), including an exception before the simulator; an
attached one never is. A stand-in run is recorded as kind `stub` and
`fake:scripted@unresolved`, never as a VLA; every `fake:` id is a stand-in
to tools/todo.py and the dashboard, whatever model name follows it.

Before arming, one WARM-UP inference on a live camera pair (the mission's
own phrase for M1, "straight ahead " labelled `goal` for M2): no answer, a
server error or a round trip over `--warmup-max-latency-s` (5 s) refuses the
flight before take-off (`warmup_refused`, exit 4), with the measurement in
metrics.json (`warmup`). Its frames are kept like any request's
(vla_captures.jsonl marks it `warmup`).

### Records (demo/out/<tag>/)

`vla_steps.jsonl` (section 9), `vla_frames/<sha256>.jpg` and
`vla_captures.jsonl` (only the captures SENT, with their seq and phrase:
a capture with no hint, M2 without a fresh box, is neither saved nor
logged), `flight_log.jsonl` (per tick: `author` /
`author_reason`, `vla_seq`, `vla_age_s`, the flown decision's `phrase`,
`hint_source`, `step`, `bins`, `raw_text`, the Shield's `raw` / `emitted` /
`flown_action` / `violations` / `repairs` / `emitted_violations`, `flown`,
`shield`, `goal_dist_m` or the box), `metrics.json` (`pilot` = pilot_record
kind `vla` with the server's `reported_digest`, `server_identity` with
precision and agreement, the backend stats under `pilot.vla` - what
tools/todo.py and the dashboard read - and `pilot.summary`: decision rate,
latency p50/p95, parse_fail, server_error, LAND, ticks by author,
`vla_authored_tick_share`), `manifest.json`, `kpi.json`, `audit.jsonl`, the
replay bundle, `view/` with the HUD panel (time, altitude, goal distance or
box; the decision number, its image's age and the cadence; the phrase and
its source; the step; who authored the tick) and `vla_server_stdout.log`.
A reused tag first loses the earlier run's records - with its replay
bundle, replay_gate.json and the server's logs, which are opened for
append - but never `vla_frames/` (content-addressed; this run's
vla_steps.jsonl names only its own) or `view/` (the recorder's, usually a
junction to C:); a record that is a link is left alone. When writing a
record fails, metrics.json is still written with `records_errors` (and the
manifest if it was built), and the flight exits 1.

### Commands

Build the policy's signed bundle once, where the lab key is (else the
launcher flies the YAML under `--dev-yaml`, recorded as a development run):

    python tools\build_flight_bundles.py vla_goto_citylife

1. CityLife frames for the gate without the model (the scripted stand-in
   steps straight ahead; the Shield is on; stop-and-go asks about every
   6 s, so 180 s gives the gate its 10 frames):

       .\scripts\run_vla_fly.ps1 -Mission goto -Seconds 180 -Tag vla_dry -FakeOutput "96 48 49</s>"

2. The offline replay gate, with the GPU, on those frames - `--frames-only`,
   because a stand-in flew them and its answers are no reference for the
   model (or on the committed protocol mosaics with `--frames-dir
   deploy\evidence\vla_protocol_frames`):

       C:\Users\natha\.conda\envs\vla-real\python.exe tools\vla_replay_gate.py --run demo\out\vla_dry --start-server --frames-only
       C:\Users\natha\.conda\envs\vla-real\python.exe tools\vla_replay_gate.py --run demo\out\vla_dry --vla-server 127.0.0.1:8790 --frames-only

   Exit 0 GATE PASSED, 1 a check failed, 2 the gate could not run (no input,
   no hello within `--hello-timeout-s`, 45 min by default), 3 NOT
   COMPARABLE (replaying a flight the server is not the model of; the
   flight's `pilot.server_identity` is compared), 4 STAND-IN (never a
   pass). The checks: at least 10 frames, the seven phrases monotone from
   left rear to right rear, parse failures, LAND only near the goal or
   without a hint, no large descents or steps toward the ground away from
   the goal, every request answered.

3. M1 GOTO (bf16 fidelity by default; `-Precision nf4` for low VRAM; `-Goal 43,100` runs beside the NFZ; `-Cadence pipelined` for that arm; `-PrintArgs` shows the flight's arguments and starts nothing):

       .\scripts\run_vla_fly.ps1 -Mission goto -Seconds 180 -Tag vla_goto_01

   or, with the simulator already on CityLife_Day:

       C:\Users\natha\.conda\envs\vla-real\python.exe demo\vla_fly.py --mission goto --tag vla_goto_01 --max-s 180 --vla-precision bf16 --vla-attn sdpa --no-vla-merge

4. M2 APPROACH the parked orange car:

       .\scripts\run_vla_fly.ps1 -Mission approach -Seconds 180 -Tag vla_appr_01

5. The control arm: `-Shield off` (recorded `shield: off`; the step
   tracker's action flies unfiltered, clamped only to the 2-40 m simulator
   safety band - "The control arm" above).

Exit codes of `demo\vla_fly.py` (and the launcher): 0 the mission ended on
its own terms; 1 aborted, no rows, or the records incomplete; 3 a VLA
failure end; 4 refused before the mission (warm-up, start not reached); 130
interrupted. The launcher checks `-Goal` / `-ParkAt` (X,Y, written 43,100 or
"43,100") and runs `demo\vla_fly.py --check-args` BEFORE it starts the
simulator.

### Not verified, and the risks

- Nothing here has run in Unreal. The fake-simulator runs checked the
  plumbing, not Project AirSim's camera topics, the 256x256 render or the
  rotate/land calls. Ctrl+C was checked as a KeyboardInterrupt raised inside
  the flight loop and handled through the same path asyncio's cancellation
  takes; a real console Ctrl+C has not been pressed on a flight.
- The 5 s warm-up gate, the 120 s late-answer wait and the 2 m start
  tolerance are ours, not measured on the RTX 4090 with the renderer up.
- Whether bf16 (about 17 GB) fits beside CityLife on the 24 GB card, and the
  decision rate with the renderer running, are unmeasured (E0).
- AerialVLA's competence in CityLife is unknown; the default goal is chosen
  so a VLA that flies its phrase needs no repair, not so it will succeed.
- M2's close rule (0.15 of the frame) is a guess at 10 m: the car leaves the
  -20 deg camera's frame below about 8.6 m horizontally.
- The policy has no subject stand-off: the VLA rail estimates no subject
  position, and a declared or truth position would be an oracle.
- The FSM's N-in-T for a slow pilot (section 8) is still a PI question; this
  rail runs no escalation FSM.

### Smoothing the follow (2026-10-23)

The user found the VLA follow "very not smooth". On the hand-written
follow's own definitions (`tools/compare_motion.py`: the Shield's emitted
velocity and yaw rate differenced over max(dt, 0.05 s)), follow #9 commanded
p95 / p99 / max 3.75 / 32.68 / 54.32 m/s^2 and a yaw acceleration p99 / max of
6.87 / 18.41 rad/s^2, against 2.60 / 3.90 / 5.56 and 0.50 / 2.09 for the
hand-written follow of 19 Oct. The largest steps were translate-to-translate:
a new answer re-pointing and re-speeding the velocity in one tick, the speed
having decayed into the old endpoint at k_pos x d.

`TrackerConfig` gains three opt-in limits (all off by default; `vla_fly`
turns them on for the follow executor unless `--no-smooth`, and refuses
`--smooth` for goto and approach):

| field | follow default | what it does |
|---|---|---|
| `smooth_accel_mps2` | 2.5 (follow_vlm's `--accel-max`) | the commanded horizontal velocity changes by at most this x dt (dt capped at one tick), as a vector; the endpoint is approached at min(cruise, sqrt(2 a d)) instead of k_pos x d |
| `smooth_yaw_accel_rps2` | 0.6 | the yaw rate changes by at most this x dt; the heading is approached at min(cap, k_yaw e, sqrt(2 alpha e)), so the slew cannot overshoot |
| `carry_s` | 1.0 | a step flown to its end while a request is in flight keeps its velocity, decaying linearly to zero over carry_s (author vla_step, reason `carry`, counted) instead of stopping between steps |

Every output passes the limits last, whoever authored it: a hold, a hover
and the follow's close hold ramp down instead of stopping in one tick
(`smooth_decay_ticks`). The close hold is therefore the tracker's input
(`StepTracker.tick(hold_horizontal=True)`, read before `act` in
`MissionLoop`) when the tracker smooths. The Shield still judges and repairs
every tick after the tracker. vla_fly's rows now carry `pitch_deg` /
`roll_deg` (follow_vlm's quaternion reading) - until now compare_motion read
their absence as 0 tilt - and metrics.json carries `motion` on the same
definitions. Closed loop on a point vehicle (0.8 s answers, pipelined, a car
turning a corner; `tests/test_vla_follow_smooth.py`): commanded acceleration
p99 22.1 -> 2.5 m/s^2, yaw acceleration p99 1.73 -> 0.60 rad/s^2, median
distance to the car 17.4 -> 16.3 m. Not flown.

## 14. Flying it on the ArduPilot rail: `perception_node.py --pilot vla`

Wave V2b, 2026-10-23 (to-do VLA-07, and VLA-03 for the cameras). The same
pilot as section 13, from the same parts (`guardrail/vla_link`,
`RemoteVLABackend`, `demo/vla_mission`), in the perception node that flies
Project AirSim through ArduPilot SITL. `--pilot servo` (follow_vlm's servo
law on the detector's box) stays the default.

### Who flies, and what stands between

| Step | Detector pilot (`servo`) | VLA pilot (`vla`) |
|---|---|---|
| raw action | `FollowPilot.step` body action, rotated by `body_to_world` | `VLAPilot.step`: the step tracker's world-frame action, nothing between (its body copy, `guardrail.frames.to_body`, is only recorded) |
| Shield | `shield.filter` (5 s / 0.1 s, own-velocity forecast) | the same call |
| escalation FSM | Brake / Loiter (BRAKE) / RTL / Land, the hold that stops clear | the same code |
| sent | the Shield's `emitted`, the FSM's brake, or nothing | the same |
| subject for stand-offs | the Grounder's box, placed from its own capture | approach and follow: the same box, unchanged; goto: none (no subject) |

A tick on which the pilot's action did not fly (the FSM's brake setpoint or
a stop the Shield advised, Loiter / RTL / Land, a fault, a takeover) suspends
the tracker (`backend.suspend`) and stops the mission's timers
(`MissionMonitor.hold`), and so does a stale-link span (`VLAPilot.hold`); the
next tick that flies resumes both, and every answer to an image captured
before is refused (`invalidated`). So no VLA end (no first decision, arrived
without LAND, target lost, a failure streak) can end, or name, a mission the
rail was holding (review of 4a0da6d, P3-9, P3-13). The rows say so
(`pilot_held`, author `none` / `suspended`, `raw_zeroed_by_hold`); metrics.vla
has `held_ticks`, `stale_link_holds` and `fsm_ticks_on_zeroed_raw` (the FSM
stepping on the suspended tracker's zero, its recovery timer running on a
zero the pilot never chose: recorded, not changed), the monitor's summary
`held_s`. Held ticks never count as VLA-flown (`pilot.summary`:
`ticks_held_by_rail`, excluded from `vla_authored_tick_share`), although the
first tick of a hold is the tracker's `vla_step` (its raw action, not
flown).

### Missions

- `--mission goto` (the default for `--pilot vla`): `--goal X,Y` in the
  Shield's frame, required. There is no default goal: this rail takes off at
  the CityLife spawn (35, -20), where every straight line east passes two
  street trees in the flight band within about a metre (`vla_mission`'s
  geometry notes), and it has no hand-written transfer to a clear start. No
  detector runs (`NoDetector`: OWL-ViT would share the GPU with the model for
  nothing), `det_hz` is None, and no subject or stand-off binds. Its score is
  `reached_goal`; `follow_scored` and `frac_within_30m` are None (not
  applicable), so kpi.json does not call a GOTO an unscored follow.
- `--mission approach`: the OWL-ViT box's phrase through `VLABridge` and
  `DetectorHint` (the turn since the box corrected), the box still placed for
  the Shield's stand-offs as on the servo path. Its `reached_goal` is the
  subject's true range at the end within `--approach-radius-m` (10 m),
  evaluation only, None with why when the subject truth did not score.
  bf16 (~17 GB) beside OWL-ViT and Unreal on the 24 GB card is unmeasured:
  until it is, fly the approach with the low-VRAM option (nf4: `--vla-precision
  nf4 --vla-attn sdpa --vla-merge`, agreeing with the reference on 25-33 % of
  the probe inputs against bf16's 96 %, so say which); metrics.vla
  `gpu_memory` has nvidia-smi's free / used memory at the hello and after the
  warm-up.
- Ends: the model's LAND (`vla_land`), landed by the node's own mission-end
  LAND (`link.land()`), never a mode request from the pilot; the other
  `MissionMonitor` ends (a dead, slow or erroring server, malformed answers,
  no first decision, arrived without LAND, overshoot, close box, target lost);
  and this rail's own (max_s, a lost link, a takeover, the FSM's end). A VLA
  failure end exits 3.

### The start gate

It is a detector-subject gate (`StartGate`: N fresh inferences with a
plausible box). A GOTO has no detected subject, so the gate is skipped and
`start_gate` says why (`enabled: false`); the approach keeps it
(`--start-when-seen`), so the VLA is asked to approach only a subject the
detector has confirmed. metrics.vla `start_gate` says which ran.

### Cadence on this rail

Stop-and-go only, unless `--allow-unstamped-pipelined` (recorded): this
rail's pose ring carries EKF poses with no sim stamps, so a capture's time is
its frame's arrival and its pose the EKF interpolated there (`VLAFrames`
`t_capture_basis` "arrival", `pose_src` counts in metrics.vla.frames).
Stop-and-go captures while hovering (centimetres, under a degree); pipelined
captures in motion, 0.1-0.4 m and 3-9 deg off (review of 4a0da6d, P2-5).

### The server, the warm-up, the link

`--vla-server HOST:PORT` attaches; otherwise the node starts
`demo/vla_server_v2.py` with `--vla-precision`, `--vla-attn` and
`--vla-merge` / `--no-vla-merge`, all three named (refused otherwise: the
node has no default configuration), or the scripted stand-in
(`--vla-fake-output`). The process starts before the scene loads; its hello
is waited for (up to `--vla-hello-timeout-s`) before ArduPilot starts, and a
started server is stopped on every way out. A non-VLA identity is refused
unless it is the started stand-in or `--allow-stand-in`; it is then recorded
as kind `stub` and `fake:` in the manifest, and the KPI grade says so.

The warm-up gate (`vla_mission.run_warmup`, three requests on live frames,
the last judged) runs in the pre-arm step, disarmed. A refusal arms nothing,
exits 4 and is on record as the city rail records one: metrics.json
(`end_reason` `warmup_refused`, `refused`, the warm-up, the pilot and server
records), manifest.json (so the Dashboard lists it), vla_captures.jsonl, the
episode's mission_end, a Dashboard row ending "failed" with why. The VLA
pilot needs a link with a pre-arm step; both real links have one, and a link
without is refused.

Each warm-up request carries its own answer timeout
(`RemoteVLA.submit(response_timeout_s=...)`): the first, the model's first
inference ever (a fresh server does not warm itself), gets
`--first-decision-timeout-s` (60 s), the others the decision timeout, and the
record's `timeout_s` is the one the client enforced. Before each, the client
is waited for (`vla_mission.wait_client_ready`: a request in flight, a
timed-out request's late answer, an unpolled result, the backoff after a
server error), bounded, with what it waited on recorded (`client_ready`).
Until the review of 4a0da6d (P1-1, P2-8) the record said 60 s while the
socket gave up at the client's 10 s, the late answer was then drained for
up to 120 s and the next two requests were "not sent": a cold first answer
over 10 s, or one refused first request, refused a healthy server. The city
rail (`demo/vla_fly._warmup`) had the same gap and uses the same two pieces
now. While an answer is awaited, and while the client is waited for, the
node pumps its autopilot link (`idle=link.pump`), as its detector wait does.

### Commands

The simulator up on CityLife (the rail's own steps, `-Step scene` /
`-Step sitl`), a server already serving AerialVLA (`demo/vla_server_v2.py
serve --precision bf16 --attn sdpa --no-merge` in vla-real), then:

    python demo\pas_ardupilot\perception_node.py --network nat --windows-ip <win> --wsl-ip <wsl> --pilot vla --mission goto --goal 46,-52 --vla-server 127.0.0.1:8790 --tag pas_vla_goto_01 --max-s 180 --save-view

or let the node start the server, naming its configuration:
`--vla-precision bf16 --vla-attn sdpa --no-vla-merge` in place of
`--vla-server`. `--dry-run` prints the server command it would start. An
attaching node (`--scene-mode attach`) needs the scene loaded with the VLA
cameras: `run_pas_ardupilot.ps1 -Step scene -VlaCameras`.

The example goal (46, -52) is 33.8 m from the spawn (35, -20), WNW of it
(the spawn faces north, so the first phrase is "to your left "). Checked on
the Shield's own distance field over the 6-14 m band map, as section 13's
goal was: the straight line from the spawn never comes within 4.08 m of an
occupied cell (the spawn's own clearance; the obstacle rule's margin is
3 m), the goal has 10.0 m and lies on the street mask (`tests/test_pas_vla.py`
`test_the_documented_example_goal_is_clear_from_the_spawn`, which needs the
gitignored `demo/out/citymap_citylife` and SKIPs without it). An earlier
draft of this section gave (60, -20), an occupied cell (review of 4a0da6d,
P2-6). Until 2026-11-04 it gave (50, -78), 17 m south of the map of that
day (y -61 to 139 m): its "17.0 m at the goal" was the Shield's estimate
off the map, which assumes every obstacle lies on it. On the map of the
whole level the same line passes through a building (-1.86 m) and the goal
has 3.70 m. The goto flights pas_vla_goto_01 and _02 stayed on that map.

### Cameras (VLA-03)

The robot template `robot_citylife_ardupilot.jsonc` carries `VLAFront` and
`VLADown`, byte for byte the CityLife VLA configs' (256x256, 90 deg, pitch 0
/ -90, every 0.1 s, not streamed), beside OWL-ViT's FrontCamera. The run's
rendered robot config keeps them only for `--pilot vla` or `--vla-cameras`
(`rail.select_vla_cameras`; recorded as `sim_config.vla_cameras`): in every
run they would add two 256x256 renders per 0.1 s beside FrontCamera and Chase,
unmeasured (review of 4a0da6d, P2-4). So the detector and VLA arms' robot
configs differ by exactly these two sensors. The camera subscribes the pair
into `vla_mission.VLAFrames` and refuses a robot config without them; an
attached node also refuses a scene whose live topics lack them.

### Records

As section 13's: `vla_steps.jsonl`, `vla_frames/` and `vla_captures.jsonl`,
rows with `pilot`, `author` / `author_reason`, `vla_seq`, `vla_age_s`, the
decision's phrase, hint source, step and raw text, `pilot_held`, the goal or
box fields; `metrics.pilot` (`vla_mission.pilot_entry`: kind `vla` only with
the server's real identity, its digest as `reported_digest`, the backend's
stats under `vla`, the summary); `metrics.vla` (mission, hint, monitor,
warm-up, altitude clamp, held ticks, frames, GPU memory); `reached_goal` and
`land`; the manifest's `vla_model_hash`; the HUD panel (`vla_mission.
annotate_vla`, moved there from `vla_fly`, with the rail's own label - the
FSM's state, an autopilot takeover, SHIELD OFF - as its last line,
`panel_lines`) beside the rail's banner. vla_captures.jsonl is written on
the way out of a run that crashed too. `tools/todo.py classify_run`
reads such a run as a VLA flight from these records (tested on the scripted
server with a real-looking identity).

### The follow, and AIOT-VLA (2026-11-06)

`--mission follow` flies section 13's M3 on this rail, from `demo/vla_fly.py`
itself:

- the follow's `DetectorHint` (the sighting's memory, the search turn, the
  close rule);
- its step tracker, built by `vla_fly.tracker_config` on vla_fly's own
  parse of the flight (`perception_node.vla_fly_follow`);
- the close hold;
- the request count for the stop rules, so a follow that never asked ends
  `subject_not_acquired`.

`--vla-front-source detector` sends the detector's FrontCamera frame, which
AIOT-VLA reads from v3 on; the server's hello must declare that camera. The
run also records:

- the server's host as its `vla_host` (hil only on an Orin);
- the object words as what they are to the model (AIOT-VLA's grounding
  phrase);
- `vla_observation`, the distinct front frames that reached the VLA.

The hil commands are in `docs/DESIGN-projectairsim-ardupilot.md`. Not flown.

### Not verified

- Nothing has run against ArduPilot SITL, Unreal or the model. The camera
  subscription was checked on a stand-in `projectairsim`; the rendered pair,
  the pose ring's pairing with EKF poses (no sim stamps on this rail: capture
  time falls back to arrival) and the decision rate with the renderer, OWL-ViT
  (approach) and the model on one GPU are not measured.
- `scripts/run_pas_ardupilot.ps1` has no `-Pilot vla` switch yet: the flags
  reach the node through `-NodeArgs` (e.g. `-NodeArgs "--pilot","vla",
  "--goal","46,-52","--vla-server","127.0.0.1:8790"`); the launcher does not
  check them before the simulator starts, as `run_vla_fly.ps1` does.
- The free-VRAM record and the nf4 advice for the approach stand in for a
  measurement of bf16 + OWL-ViT + Unreal on one 24 GB card, which has not
  been made.
- The FSM's N-in-T for a slow pilot (section 8) applies here unchanged; how
  often a stop-and-go VLA trips it is unmeasured.

## 15. Fine-tuning AerialVLA to follow (wave FT, 2026-10-23)

The approved next step after the ten follow flights: expert demonstrations
from the hand-written follow (recorded and labelled by branch FT-D,
`tools/build_follow_dataset.py`), and a fine-tune of the original adapter on
them.

### Format-exact training

`training/finetune_aerialvla_follow.py` trains on exactly what the server
feeds and reads: the prompt is `aerialvla_codec.build_prompt`'s (a sample is
kept only when its prompt re-builds from its phrase and object), the image is
the dataset's 224x448 mosaic through the Prismatic processor as
`vla_server_v2.generate_text` calls it, the target is the truncated two-digit
"FF DD YY" the strict reader accepts, and input_ids = tok(prompt + target) +
[EOS] with the server's tok(prompt) as a checked prefix (labels -100 over
it; loss on the answer and EOS only). On the real tokenizer the prefix holds
on all eight phrases, upstream's text EOS is the same token, and one encoded
sample equals the server's own preprocessing (prompt bytes, mosaic sha256,
prompt ids, pixel_values: `--dry-run`'s `server_parity`). The July trainer
(`training/finetune_aerialvla.py`) rounded and did not pad its bins,
tokenised the answer apart (a second dummy-prefix space), used one image and
its own prompt; it is kept, not reused.

bf16 LoRA continuing `aero_vla` (its own r 64 / alpha 128 / targets /
projector), the LoRA and projector in fp32 for AdamW under bf16 autocast,
gradient checkpointing; nf4 QLoRA as the fallback. Cosine LR with warm-up,
per-epoch checkpoints `D:/models/aerialvla-ft-follow/<run>/epoch<N>`, a
held-out evaluation before training and after every epoch (parse-ok rate,
exact bin per axis, step MAE in m / rad, turn-sign agreement), early
stopping on step_mae_norm, loss and evaluation CSVs and `run.json`.
Held-out runs never train (split by run, refused otherwise).

### Which adapter flew

Each checkpoint carries `aerialvla_ft_provenance.json`; `vla_server_v2
--adapter` announces it as `aerialvla-ft-follow/<run>/<epoch>@<adapter
sha8>` with an `adapter` block (path, sha256 over its files, variant,
provenance), the pilot side records the model id
`aerialvla:openvla-7b+aerialvla-ft-follow/<run>/<epoch>` (never upstream's
name with the fine-tune's digest), and `tools/todo.py` allows the prefix as a
VLA and records the variant (`adapter`: original / fine-tune); a run
configured as one adapter and served by the other is `vla_unverified`.
`vla_fly --vla-adapter DIR` starts the server with a checkpoint.

### Evaluating

`tools/eval_vla_adapter.py` sends every held-out sample as a flight sends it
(the mosaic's tiles as PNG, which the server rebuilds pixel for pixel - each
answer's image and prompt sha256 are checked) to a running server (refused
unless its adapter digest is `--adapter`'s) or in process through
`V2Server`'s core, and the same for the original adapter as the baseline.
That, not the training run's own evaluation (bf16 autocast over fp32 LoRA),
is the number to quote; and offline step accuracy is not the follow: the
closed-loop measure is a flight scored by `tools/compare_motion.py` and
`frac_within_30m`.

### Not verified

- No training run has been made: nothing here loaded the 7B model. The loop
  ran end to end only on a CPU stand-in model; whether bf16 LoRA with
  gradient checkpointing fits 24 GB beside nothing else is unmeasured (the
  estimate is about 20 GB at batch 4).
- The expert's decision cadence must match the model's (the July lesson:
  run2 overshot because the expert decided at 10 Hz and the model at
  0.8 Hz); that is the dataset's labels (FT-D's horizon and stride), not this
  trainer's.
