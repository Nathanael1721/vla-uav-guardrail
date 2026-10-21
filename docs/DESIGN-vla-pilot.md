# The VLA pilot: AerialVLA behind the Shield

Status, 2026-10-21: the pilot side is built and tested offline
(`guardrail/vla_link.py`, `guardrail/vla_backends.RemoteVLABackend`,
`tests/test_vla_link.py`, `tests/test_vla_backends.py`). Nothing here has
flown, and no model was loaded for it. The server side (`demo/vla_server.py`,
protocol v2) is a separate change. Wiring the backend into a rail
(`perception_node --pilot vla`, the ROS 2 `/vla/action_4d` bridge), the HUD
panel and the manifest's `pilot_record(kind="vla")` come after both.

## 1. Who flies

| Layer | What it decides | Code |
|---|---|---|
| AerialVLA (OpenVLA-7B + the original `aero_vla` LoRA, own process) | every step: which way to turn, how far forward, how far up or down, and when it has arrived (LAND) | `demo/vla_server.py` |
| The hint | one of eight training phrases, from the GOTO goal (`hint_source: goal`) or the OWL-ViT box bearing (`detector`, `demo/vla_bridge.py`), or none | `vla_link.goal_hint`, `vla_bridge` |
| The step tracker | how each step is flown at 10 Hz: a deterministic interpolator, not a planner | `vla_link.StepTracker` |
| The Shield | whether each 10 Hz action is legal, and its repair | `guardrail/shield.py` |
| The escalation FSM and the rail | holds, Loiter, RTL, Land; the autopilot link | `guardrail/fsm.py`, the rails |

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

What the tracker does NOT do: choose a direction the model did not choose,
avoid anything, look at the policy's zones, blend in a planner or a servo.
There is no `goal_blend`. The only policy input is the altitude band.

## 4. Cadence: pipelined and stop-and-go

One request is outstanding at a time, by construction (`RemoteVLA.submit`
returns None while one is in flight; the fake server counts overlaps, and they
stay at 0).

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
| `hold` | the step is complete (`step_complete`); the answer was malformed (`parse_fail`) or a server error (`server_error`); the model said LAND (`land`) | zero, except a completed step keeps its altitude target; parse_fail, server_error and LAND are all-zero |
| `hover` | no usable decision: the step's image is too old (`stale`), a hold lasted past the timeout (`no_new_decision`), the link failed (`link_lost`), or a rail hold invalidated it (`awaiting_fresh_decision`) | all-zero |
| `none` | no decision accepted yet (`no_decision`), or the rail has the tracker suspended (`suspended`) | all-zero |

A malformed answer HOLDS ZERO and is counted: it replaces the step being
flown, because a misread answer is no licence to keep flying the previous one.
(The integration discovery report proposed letting the previous decision keep
fading; the brief's rule, adopted here, is the stricter one. Its yaw-fade and
"forward only while the box is fresh" rules were for velocity semantics; a
step is a bounded displacement, so there is nothing to fade.)

## 6. Timeouts

| Timeout | Default | What it bounds |
|---|---|---|
| `decision_timeout_s`, step | 10 s | the age of the image behind a step being flown (capture to now). Past it the step is dropped (`end: stale`) and the tracker hovers. An answer already older on arrival is refused (`stale_on_arrival`). A 5 m step at 1 m/s plus about 3 s of latency fits. |
| `decision_timeout_s`, hold | 10 s | how long a hold lasts waiting for the next answer, measured from the hold's start. Then hover (`end: hold_timeout`). |
| client response timeout | = `decision_timeout_s` (factory) | one request's round trip; a later answer would be refused as stale anyway. On timeout the connection is dropped. |
| connect timeout | 2 s | opening the TCP connection |
| reconnect backoff | 1 s | no new request this long after a failure |
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
- fate: `accepted`, `refused` (suspended / invalidated / after_land /
  stale_on_arrival / future_capture / link_failure / closed), `target` (psi, x, y, up,
  up_raw, alt_clamped, rotate_only, time_cap_s), `outcome` (t_start,
  completed, completion tolerance / time_cap, t_complete, end superseded /
  stale / hold_timeout / link_lost / suspended / land / closed, t_end, ticks
  by author).

`RemoteVLABackend.stats()` gives the run's decision count and rate, refusals,
ends, ticks by author and the link's counters, for `metrics.json`.

## 10. Protocol v2, the client's half

TCP (default 127.0.0.1:8790), one JSON object per line, UTF-8, one request to
one response, many requests per connection; `hello` on every new connection
(its `model` is kept as `RemoteVLA.identity`).

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
`t_capture` or pose is not what was sent, a hash is not 64 hex digits, or the
line is not JSON (NaN included). `ok: false` with a reason is a server error:
held at zero and counted, not a link failure.

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
