# The VLA pilot: AerialVLA behind the Shield

Status, 2026-10-21: the pilot side is built and tested offline
(`guardrail/vla_link.py`, `guardrail/vla_backends.RemoteVLABackend`,
`tests/test_vla_link.py`, `tests/test_vla_backends.py`). Nothing here has
flown, and no model was loaded for it. The server side (`demo/vla_server.py`,
protocol v2) is a separate change. Wiring the backend into a rail
(`perception_node --pilot vla`, the ROS 2 `/vla/action_4d` bridge), the HUD
panel and the manifest's `pilot_record(kind="vla")` come after both.

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

- **pipelined** (default): the next image is captured and sent as soon as the
  previous answer is back, while that answer's step is being flown. The newer
  step supersedes the older on arrival. With 0.35 s answers the aircraft flies
  continuously toward a target a few metres ahead; with 3 s answers most steps
  complete before they are superseded.
- **stop_and_go**: upstream's own cadence, as a labelled arm. The next image is
  taken only after the step is complete and the tracker has held for 0.5 s.
  No step is cut short.

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
| `vla_step` | flying toward an accepted step's targets (`reason`: rotate / translate) | the tracker's law above |
| `hold` | the step is complete (`step_complete`); the answer was malformed (`parse_fail`); the model said LAND (`land`) | zero, except a completed step keeps its altitude target; parse_fail and LAND are all-zero |
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
| client response timeout | = `decision_timeout_s` (factory) | one request's round trip; a later answer would be refused as stale anyway. On timeout the connection is dropped. |
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
