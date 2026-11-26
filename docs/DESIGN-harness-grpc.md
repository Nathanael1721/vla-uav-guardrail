# DESIGN: the stress harness's gRPC control plane, and one root CA for it and SROS 2

Audit cards **WP4-05** (no gRPC control service for the harness) and **X-15**
(SROS 2 security on the ROS 2 graph, with the harness mTLS reusing its root
CA). **SROS 2 on the ROS 2 graph is opt-in behind `SROS2=1`** (Enforce, one
enclave per process, the keystore's identity and permissions CA the harness's
lab root CA); `tools/sros2_check.sh` passed 6/6 in WSL Ubuntu 24.04 with ROS 2
Jazzy on 2026-10-19 (docs/DESIGN-sros2.md). Not yet flown: no SITL flight has
run with `SROS2=1`, the default (`SROS2=0`) still runs the graph unsecured, and
per-topic permissions are not written. This note covers the gRPC side's
mutual TLS and the lab root CA built to be shared; the ROS 2 half is
docs/DESIGN-sros2.md. Built 2026-10-08, revised 2026-10-09 after review; on
2026-10-12 the service gained the page's other six RPCs (StartSession,
StopSession, InjectPolicy, IssueTask, the two streams), a live mode that
injects events while an episode flies, forwarding of a session's HotApply to
a REST endpoint, and a reserved SITL backend. On 2026-11-10 the SITL and HIL
backends were built (a session on a rail is one flown, scored episode with
its bundle, through the sweep's own rail backends), zone edits are forwarded
as PATCH /dynamic_nfz, and the bundle carries the grant's `episode.bag2/` and
every hot-apply generation ("Rail sessions" below): code and headless tests,
**not flown**. On 2026-11-14 a rail episode could have its scenario's own
events driven by the harness over REST instead of the mission
(`drive_scenario_events`), EmitEvent was forwarded on a rail, and
`tools/harness_hil_episode.py` was written to fly or dry-run one such
episode ("A scenario's own events, driven by the harness" below): code and
fake-rig tests, **not flown**. On 3 Nov a REST-bound session forwarded
hot-applies into a flying desktop rail that was started outside the harness;
nothing else here has flown, and the headless harness's episodes are never
KPI-grade.

## What the grant asks

Stress Testing page, "Locked design choices":

> Control plane: gRPC over mTLS. Same root CA as SROS 2 for cert reuse.

and, in its Outputs table, "RPC harness - Python service + gRPC stubs". The
page gives the service surface as "proto-ish" (StartSession, StopSession,
InjectPolicy, HotApply, StartEpisode, IssueTask, EmitEvent, EndEpisode,
StreamTelemetry, StreamEvents). The reference design's data-flow page adds the
transport detail: per-component certificates "with a shared root CA; SROS 2
turned on for the ROS 2 graph using the same root CA", and why gRPC rather
than ROS 2 services: the orchestrator also talks to non-ROS clients (CI,
dashboards, scenario authoring).

## What exists now

| File | What it is |
|---|---|
| `guardrail/protos/harness.proto` | The service `StressHarness`: the page's ten RPCs in its message names, plus GetStatus |
| `guardrail/protos/harness_pb2.py`, `harness_pb2_grpc.py` | Generated, committed (command below) |
| `guardrail/harness_service.py` | `Harness` (no transport), the servicer, `serve()`, the PKI, a CLI |
| `tools/harness_client.py` | `HarnessClient` (all eleven RPCs) and a CLI: `run` (with a session when given a policy, a prompt or `--live`; `--backend sitl\|hil` flies it on the server's rig, 2026-11-10), `forward`, `drive`, `status` |
| `tools/sros2_setup.sh` | The SROS 2 keystore and enclaves of the dev graph, seeded with the harness CA (first run 2026-10-19 in WSL Jazzy; `tools/sros2_check.sh` 6/6, docs/DESIGN-sros2.md) |
| `tools/rerun_episode.py` | Re-flies a bundle and compares the flight logs (used here on a live episode's bundle) |
| `tests/test_harness_service.py` | 56 tests, standalone runner; the logic tests drive `Harness` without gRPC |
| `tests/test_wp4outputs_harness.py` | 16 tests: live mode, sessions, InjectPolicy, IssueTask, REST forwarding, the streams, the lifecycle over mTLS |
| `tests/test_harness_rail.py` | 11 tests (2026-11-10), on a fake rig: zone edits as PATCH /dynamic_nfz, a hil and a sitl session flown, scored and bundled under the grant's names, every generation in policy_bundle.tar.gz, the refusals, the client tool's `run --backend hil` over gRPC |
| `tools/harness_hil_episode.py` | (2026-11-14) one harness-driven hil episode: a hil session, StartEpisode with `drive_scenario_events`, EndEpisode, the six names checked; `--dry-run` prints the plan and sends nothing |
| `tests/test_harness_hil.py` | 9 tests (2026-11-14), on the same fake rig: every event form's REST body and route, the REST handler applying each library event as the mission node does, a harness-driven episode flown, scored and bundled, an unsent scenario event failing it, EmitEvent forwarded on a rail, the dry run, the tool over gRPC |

Regenerating the stubs, from the repository root:

    python -m grpc_tools.protoc -I . --python_out=. --grpc_python_out=. guardrail/protos/harness.proto

grpcio-tools 1.84.0 writes protobuf gencode 7.35.1, so the stubs need
grpcio >= 1.84.0 and protobuf >= 7.35.1 (installed: vla-real has grpcio
1.84.0 / protobuf 7.36.2, vla-drone 1.84.0 / 7.35.1). The generated
`harness_pb2.py` checks the protobuf runtime when it is imported and refuses
an older one. Its header says "NO CHECKED-IN PROTOBUF GENCODE", which is
protobuf's general advice; the grant's Outputs table asks for the stubs, so
they are committed, and a test regenerates them into a temporary folder and
fails if the committed files differ. Every field number of the 2026-10-09
messages is unchanged, so a client built against those stubs still works.

### The RPC surface against the page's

| Page's RPC | Here (headless backend) |
|---|---|
| StartSession | opens a session: backend (HEADLESS; SITL and HIL since 2026-11-10 on a server with a rig, "Rail sessions" below, else UNIMPLEMENTED), the page's scene and airframe (recorded, not used: no simulator to load), a default seed, an optional REST endpoint the session is bound to |
| StopSession | closes it: ARMED episodes fail, a live one stops (FAILED, no bundle), a scheduled one already flying finishes and the call blocks until it has (see The streams); the streams end |
| InjectPolicy | the policy the session's later episodes fly: a bundle or policy YAML under `policies/` or `bundles/`, or the archive's bytes, pinned to `expected_hash` (required); in a REST-bound session the endpoint must be flying it |
| HotApply | to an episode: the Policy DSL's REST hot-apply body, validated by `guardrail.api.apply_event`, flown at `at_sim_t` (or `now`, live); to a REST-bound session: POSTed to `<endpoint>/hot_apply`, or, for an edit of a zone, sent as `PATCH <endpoint>/dynamic_nfz/<id>` (2026-11-10) |
| StartEpisode | arms one ScenarioSpec cell with a seed, an arm, optionally a profile and a session; `live` starts it flying at `pace` |
| IssueTask | records the prompt's SHA-256 (manifest_extras.json) for an ARMED episode or the session's later ones; the headless pilot reads no text |
| EmitEvent | a ScenarioEvent, the grant's seven types, at `at_sim_t` (or `now`, live); on a rail episode (2026-11-14) its REST body, forwarded at its mission time |
| EndEpisode | flies (or lets a live episode finish, unpaced), scores, writes the episode bundle (three of the grant's six items, see below), returns it with the KPI row |
| StreamTelemetry | every tick of the session's episodes: a clock frame per live tick, then each episode's flown rows |
| StreamEvents | the session's lifecycle and each episode's run events (hot-applies with generation and hash, refusals), from the start |
| GetStatus (ours) | state, queued events, verdict, bundle path, error, client name; for a live episode the next tick and ticks flown |

A raw call of each with an empty request reaches the servicer (a validation
code, never UNIMPLEMENTED), and a method the .proto lacks still answers
UNIMPLEMENTED (`test_the_pages_ten_rpcs_and_get_status_are_served`).

## How an episode flows

1. **StartEpisode** takes a `scenario_id` of the server's library or one
   family as `scenario_yaml`, a `cell_id` (needed when the family expands to
   more than one cell), a `seed`, a `profile` (smoke, nightly, broad) and an
   `arm`. It goes through `guardrail/scenario_spec.py` as the sweep does
   (`Family.cells` with the profile's overrides and samples, `episode_arms`,
   `Episode`), refuses what the sweep's runner would refuse about the
   schedule (an event after the last tick, a windowed event with no clock, a
   GPS dropout after the run), loads the policy, draws the arm record
   (`build_arm`: the paraphrase and prefix key) and returns an episode id. A
   refused request creates no episode. The arm is drawn here rather than at
   EndEpisode so that a scenario this server cannot draw an arm for (a
   `paraphrase.backend: llm:<id>` with no LLM client configured, or a
   `stored` backend with no stored set for the prompt) is refused up front
   with FAILED_PRECONDITION; before 9 Oct it was accepted and then failed at
   EndEpisode as INTERNAL. The draw is a function of (cell, seed, arm), so
   when it is made does not change what flies. Since 2026-11-07 a paraphrase
   arm is drawn by the paraphraser service the server is bound to (`serve
   --paraphraser HOST:PORT`), never in process: a server bound to none refuses
   it (FAILED_PRECONDITION), a service that does not answer is UNAVAILABLE,
   and the arm record names the service and its validator and store digests.
   In a **session** (`session_id`), the episode flies the session's injected
   policy instead of the scenario's (the dry run starts from it too), takes
   the session's seed when it names none, and carries the session's task.
   A **live** episode is set up here (EpisodeRunner flies its setup to the
   start of tick 0, so a scenario the runner refuses creates no episode) and
   starts flying on a thread of its own.
2. **HotApply** and **EmitEvent** each carry `at_sim_t` (or `now`, on a live
   episode). They are validated when they arrive (a HotApply at its place in
   flight order, below) and queued, and on a live episode injected into the
   runner as well.
3. **EndEpisode** adds the queued events to the cell's own, flies the episode
   with `experiments/sweep_scenarios.py`'s `run_episode`, scores it with
   `score`, `label_failures` and `check_gates`, writes the episode bundle with
   `episode_manifest` and `write_bundle`, and returns the bundle path, the
   verdict, the full KPI table (JSON), the episode's KPI-report row (typed,
   each score beside its null) and the hot-apply events as they were applied
   in flight (generation, policy hash, refusals). On a live episode it lets
   the flight finish unpaced, then does the same.

The per-episode verdict (pass, tracked, fail, known_failure,
unexpected_pass) is the sweep's own: since 2026-10-11 both the sweep's
`main()` and this service call `experiments/sweep_scenarios.py`
`episode_verdict` (guarded by
`test_the_harness_judges_with_the_sweeps_own_verdict_function`). Two older
tests still check it: one reaches every status
from one synthetic scenario by changing only its labels, and one runs the
sweep's own `main()` on five smoke-profile families and flies each of its
episodes again through the harness, comparing status, failed gates, label
failures and the known-failure reason episode by episode.

### The episode bundle, against the grant's

The Stress Testing page's episode bundle is `manifest.json`,
`policy_bundle.tar.gz`, `episode.bag2/`, `shield_audit.jsonl`,
`harness_events.jsonl` and `kpi.json`. The headless runner writes three of
them: `manifest.json` (the six fields), `harness_events.jsonl` and `kpi.json`.
The service adds `harness_request.json` (the request, every event as it
arrived with the time it was flown at, and the IssueTask prompt, so the
episode can be asked for again) and `manifest_extras.json` (the arm's keys,
the self-contained keys `tools/rerun_episode.py` re-flies the episode from,
the task's hash and the session), and the sweep's `write_bundle` adds
`flight_log.jsonl` when asked. Since 2026-10-30 the sweep's `write_bundle`
also writes `shield_audit.jsonl` (the flying Shield's audit records), and since
2026-11-02 the service writes `policy_bundle.tar.gz`: the injected policy
file byte for byte (`injected/<file>`), the canonical IR of generation 0 and
of the last generation (`generations/policy_g<N>.json`), and an index
(`policy_bundle.json`) with every member's SHA-256 and every generation the
episode flew (time, event, rule, policy_hash). Until 2026-11-10 a generation
between the first and the last was in the index by its hash only (the
headless runner kept no copy of it, and the index said `complete: false`);
since then the runner keeps every generation's IR and all are members.
**Not written on the headless backend:**
`episode.bag2/`; there is no ROS 2 graph on the headless path to record, and
`kpi.json`'s `harness.bundle_files` says so. A ROS 2 rail run records its bag
in its run folder (`bag/`), and a rail session's bundle carries it as
`episode.bag2/` ("Rail sessions" below).

### Scheduled and live episodes

**Scheduled** (the default). `run_episode` flies a whole episode in one call
(about 0.3-0.4 s for 30 simulated seconds on 9 Oct; see Measured below for
today's), so an event does not act "now": it acts at its `at_sim_t` when
EndEpisode flies the episode, applied by the runner's own `apply_event`,
exactly as the same event written in the scenario's `events:` would be. The
tests check that equivalence byte for byte on the flight log: a hot-applied
zone, a hot-applied corridor swap (against the library's own
`corridor-swap-shifted`) and an EmitEvent. The flight is a function of
(cell, seed, arm, scheduled events) and of the code.

**Live** (`StartEpisode(live=true, pace)`, since 2026-10-12). The service
drives `experiments/sweep_scenarios.py` `EpisodeRunner` through its public
`step()`, `inject()` and `finish()` only: a thread steps one tick at a time,
paced so that tick *t* flies at `start + t / pace` wall seconds (1.0 real
time, 0 unpaced), holding the episode's lock for the tick (so an injection
never lands mid-tick) and the flight lock for the step (so a scheduled
episode and a live one never interleave inside a tick). A HotApply or
EmitEvent is validated as for a scheduled episode, then handed to
`EpisodeRunner.inject()`: at `at_sim_t` when that is no earlier than the next
tick, or at the next tick itself with `now`; a time already flown is
OUT_OF_RANGE. The ack reports the time it was injected at, and the queue
records it. `inject()` is documented (and tested in tests/test_sweep.py) to
apply an event exactly as the same event written last in `events:`, so a live
episode's bundle re-flies as a scheduled one: the tests compare a live flight
with a hot-apply at 3.0 s, and one with a `now` hot-apply at whatever tick it
landed on, byte for byte against the scheduled episode with the event at the
acked time, beside the null (the live flight without the hot-apply differs),
and `tools/rerun_episode.py` re-flies a live bundle made over mTLS to an
identical log. EndEpisode on a live episode sets its pace to 0 and waits for
it; one nobody ends finishes on its own and is bundled. StopSession stops it
between ticks (FAILED, nothing bundled).

What live mode cannot show: `EpisodeRunner` exposes no per-tick state while it
flies (its rows come out of `finish()`), so StreamTelemetry sends a clock
frame per live tick (tick, simulated time, wall time) with the pose unset and
the reason, and the flown rows once the episode ends. A public per-tick row
on the runner (outside this note's files) would let the live frames carry the
pose. The manifest's `sim_speedup` of a live episode is measured as for any
other (about `pace`), and the topology is still `headless-kinematic`: a live
episode at pace 1.0 is not KPI-grade either.

### HotApply goes through the REST endpoint's own handler

The body is the REST hot-apply body (`guardrail/api.py`): `dynamic_nfz`,
`time_window_switch` or `corridor_swap`. It is applied with
`api.apply_event` to a dry-run Shield rebuilt from the injected policy plus
this episode's hot-applies, **in the order the flight will apply them**: by
`at_sim_t`, ties by arrival, which is `run_episode`'s stable sort of the
cell's events followed by the queue. So it is refused exactly as the REST
endpoint would refuse it at that point of the flight, with the matching code
(409 ALREADY_EXISTS, 404 NOT_FOUND, 422 INVALID_ARGUMENT: a locked rule
class, a layer above mission, a relaxation of another layer's hard rule). A
body that is fine where it lands but makes an already-acknowledged hot-apply
fail further down the timeline (the same zone id scheduled earlier, say) is
FAILED_PRECONDITION, naming that hot-apply. The rule the body created, as it
stood right after the body was applied, becomes the ScenarioEvent the runner
applies: `spawn_polygon_fence` with the zone's vertices (lat/lon already
projected), band, margin, priority, breach action and motion;
`activate_rule` / `deactivate_rule` with the switch's window; or
`swap_corridor`. Tests fly a hot-applied zone with a non-default margin,
breach action and motion, one with an altitude band, and a switch with a
weekday window, each byte-identical to the same event written in the
scenario, and each beside a null showing that the field changes the flight.

Until 9 Oct the dry run replayed arrival order. A switch at 5 s sent after
its zone at 10 s was acknowledged, with a generation and a policy hash, and
the episode then failed at EndEpisode ("no rule nfz-hot"); zones that arrived
out of order were acknowledged with generations they did not fly at.

`PolicyAck.validated_generation` is the generation at the body's place among
this episode's hot-applies, as of that ack: a later HotApply scheduled
earlier shifts it, and in flight the generation also counts the scenario's
own events and EmitEvents. The flown values are in
`EpisodeBundle.applied_events`.

A field the runner's event cannot carry (a zone's `valid_time` or
`altitude_ref`, a switch's own priority) is refused UNIMPLEMENTED, not
dropped: a zone flown without its schedule is a different zone.

**The dry run knows only the hot-applies.** The scenario's own `events:` and
the episode's EmitEvents act on the flying vehicle (a zone spawned
`ahead_of_vehicle_m` needs the aircraft's position), so the dry run cannot
replay them. A HotApply aimed at a rule only they create is NOT_FOUND (use an
EmitEvent for it), and a HotApply that collides with one of them (re-using
the id of a zone they spawn) is acknowledged and then fails when it flies
(at EndEpisode, or in the live episode's tick) with FAILED_PRECONDITION and
the runner's reason. Live mode does not close this: `EpisodeRunner` exposes
no Shield to dry-run against, so the dry run still knows only the
hot-applies.

### Lifecycle

- **EndEpisode is idempotent.** A second call on an ENDED episode returns the
  same bundle. A call that arrives while another call is flying the episode
  waits for that flight and returns its bundle, or its failure (before 9 Oct
  it was FAILED_PRECONDITION "being flown by another call", which is what a
  client gets that retries after its deadline expired while it queued).
- **Idle episodes expire.** An episode left ARMED with no StartEpisode,
  HotApply or EmitEvent call for the server's idle TTL (default one hour,
  `serve --armed-ttl`) becomes FAILED with the reason, nothing flown, and
  stops counting against the 256 open episodes. Before 9 Oct an ARMED episode
  never expired, so 256 clients that started and gave up (the client tool
  after a refused HotApply, for one) wedged the server until it restarted.
  The client tool now names the episode it leaves ARMED. There is no
  CancelEpisode RPC.
- **Idle sessions expire** (2026-10-13). A session left OPEN with no call on
  it (StartEpisode or IssueTask in it, InjectPolicy, a session HotApply, a
  stream poll) for the server's session TTL (default one hour,
  `serve --session-ttl`) and with no episode of it ARMED or RUNNING is
  stopped as StopSession would stop it; its event stream says
  `session_expired`, then `session_stopped`. Until then the 64-session cap
  could be filled by clients that died (the client tool prints "session ...
  was left OPEN" when it fails), and StartSession answered RESOURCE_EXHAUSTED
  until the server restarted. Expired sessions are swept when a new one is
  opened, and checked whenever a call names one.
- **`ticks_flown` is unset until it is known** (2026-10-13). GetStatus counts
  a live episode's ticks as they fly and reports a scheduled episode's flown
  rows once it has ENDED; before that the field is unset (proto3 `optional`,
  same field number), because a scheduled episode flies in one call and a 0
  would read as "flew nothing".

## Sessions, InjectPolicy, IssueTask, the streams

A **session** (`ss-` + 12 hex) holds a backend, the page's scene and airframe
(recorded), a default seed, an optional REST endpoint, the injected policy,
the issued task, its episodes, and two append-only buffers the streams read
from the start: events (`session_started`, `policy_injected`, `task_issued`,
`episode_started`, `hot_apply`, `hot_apply_forwarded`, `emit_event`,
`run_event` - each of an episode's harness events, hot-applies with their
generation and hash -, `episode_ended`, `episode_failed`, `session_expired`,
`session_stopped`)
and telemetry frames (capped at 200 000 per session; past that they are
counted in `telemetry_dropped`, not kept). At most 64 sessions are open; 256
stopped ones are kept for their streams. Episodes started without a session
behave exactly as before 2026-10-12.

**InjectPolicy** loads the bundle through `PolicyRef.load_with_source`
(`guardrail.bundle.load_for_flight`: integrity always enforced, an
unverified signature accepted and recorded - the ack's `signature`), checks
it hashes to `expected_hash` (every form `Policy.matches_hash` knows), and
keeps a `PolicyRef` pinned to that hash for the session's later episodes, so
a file changed afterwards stops an episode instead of flying another policy.
An archive sent as bytes (at most 16 MiB) is kept under the runs folder
(`sessions/<id>/policy-<sha>.tar.gz`). Episodes already started keep the
policy they started with.

**IssueTask** records the prompt's SHA-256 and length in the bundle's
`manifest_extras.json` (`task_prompt_sha256`, `task_prompt_chars`,
`task_read_by_pilot: false`) and the prompt itself in
`harness_request.json`; TaskAck says `pilot_reads_text: false`. A test checks
that an episode with a task flies the same log as without one: on this
backend the field is a record, not an input.

**The streams** (`StreamTelemetry`, `StreamEvents`) replay the session from
its first item, then wait for more (checking every 0.5 s that the client is
still there), and end once the session is stopped and everything has been
sent; the stop event and the state change are made under one lock, so a
stream cannot end before it has sent `session_stopped`. StopSession waits for
a scheduled episode already flying, so that episode's rows, run events and
`episode_ended` come before `session_stopped` and StopResponse counts them
(until 2026-10-19 the streams ended at once and those items landed after the
stop, where no open stream delivered them). The wait is that whole flight:
its turn at the one run lock every session's scheduled flights share (behind
any queued ahead of it), its run_episode and its bundling. Under contention
that is several run_episodes, not one, so a client should give StopSession a
deadline of a few scheduled EndEpisodes (1.709 s median each, below); with
nothing flying it returns at once (0.001 s). Each open stream
holds a server worker thread (`serve(max_workers=16)` by default).

The episode bundle's `manifest_extras.json` also carries the self-contained
keys `experiments/sweep_scenarios.py` `episode_extras` writes (where the sweep
has it: the expanded spec with every event the episode flew, hot-applies and
EmitEvents included, the library defaults, the seed, the arm, the policy as
injected and every generation), so `tools/rerun_episode.py` re-flies a gRPC
episode from its bundle alone. A failure to build them is recorded as
`episode_extras_error`, not raised: the episode flew and is bundled.

## HotApply forwarded to a REST endpoint

A session started with `rest_endpoint` is **bound** to that hot-apply endpoint
(`guardrail/api.py`, e.g. a rail's live Shield served by
`api.serve_in_background`). A HotApply addressed to the session
(`session_id`, no `episode_id`) is POSTed as it is to `<endpoint>/hot_apply`
(the route that takes any of the three hot-applicable types), and the
endpoint's answer comes back in the PolicyAck: `rule_id`, `op`, `type`, the
`generation` and `policy_hash` the endpoint's policy has after it, and
`forwarded_to`. The endpoint's refusals keep their meaning (409
ALREADY_EXISTS, 404 NOT_FOUND, 422 INVALID_ARGUMENT); an endpoint that does
not answer, or answers 5xx, is UNAVAILABLE; one that answers something that
is not a JSON object is FAILED_PRECONDITION. A HotApply addressed to an
episode is never forwarded: the address decides, so one call cannot act on
two Shields. InjectPolicy in a bound session reads `GET <endpoint>/health`
and refuses a policy the endpoint is not flying (hot-applies forwarded to it
would act on rules the session did not inject).

The server makes the request, so the endpoint is a place a client can make
the server send to: only a loopback endpoint is accepted unless the server
allows its host (`serve --allow-rest-host H`, `Harness(rest_hosts=...)`); no
query, fragment or credentials; and the request goes out with no proxy (an
environment `HTTP_PROXY` would otherwise see a loopback request - a test sets
one and checks the forward still reaches the endpoint). The tests run the
endpoint as a standard-library HTTP server calling `guardrail/api.py`'s own
`apply_event` and `health` on a live Shield (FastAPI is not installed in
every environment), with FastAPI's error shape (`{"detail": ...}`); the
FastAPI app itself (`api.build_app`) is not what they call.

`guardrail/api.py` serves plaintext on loopback only and mutual TLS anywhere
else, so an `https` endpoint needs the server's own REST TLS identity:
`serve --rest-pki DIR` (`Harness(rest_pki_dir=...)`), the lab CA to verify
the endpoint by and a client certificate on its common-name allow-list
(`harness-client` unless `--rest-client` names another; a dedicated one,
allowed on the rail with `--hotapply-client`, keeps gRPC clients from holding
the rail's identity). The endpoint's server certificate must name the address
the session uses (`gen-pki --host`, `issue --server-host`). Without an
identity an `https` endpoint is refused at StartSession (FAILED_PRECONDITION),
and a handshake that fails for a configuration reason (an endpoint
certificate the lab CA did not sign, or that does not name the host) is
FAILED_PRECONDITION, not a retryable UNAVAILABLE. Until 2026-10-19 the
forward had no identity and every call to such an endpoint failed as
UNAVAILABLE. A test serves the same standard-library endpoint over mutual TLS.
One configuration error still looks like the network: a client certificate
the lab CA signed but that is not on the rail's `--hotapply-client`
allow-list is reset by api.py's common-name gate, with no answer, so the
forward stays UNAVAILABLE (a reset can be the network); its message then
names the allow-list and the certificate (`--rest-client`) to check, and
the test resets a common name off its own allow-list to show it.

## Status codes

| Code | When |
|---|---|
| INVALID_ARGUMENT | the spec, cell, seed, arm, event, pace or session request fails validation (pydantic's message is passed through); a NaN or Infinity in a HotApply body or EmitEvent payload; a paraphrase arm on a protected (AerialVLA) prompt; an injected archive or policy the loader refuses; `now` on a scheduled episode; a REST 422 |
| NOT_FOUND | an unknown scenario, cell, profile, episode, session or policy file; a REST 404 |
| ALREADY_EXISTS | a REST 409 (a rule id in use) |
| OUT_OF_RANGE | an event after the episode's last tick, or before the next tick a live episode flies |
| PERMISSION_DENIED | a client certificate not on the allow-list; a policy (a client-sent scenario's, an InjectPolicy path) outside `policies/` and `bundles/`; a REST endpoint off loopback the server does not allow |
| FAILED_PRECONDITION | an event for an episode no longer taking them (ended, failed, expired, or a scheduled one being flown); an episode the runner refused in flight (GetStatus says why); a paraphrase arm this server cannot draw; a hot-apply that would make an earlier-acknowledged one fail later in the flight; a stopped session; a session HotApply with no endpoint bound; an injected policy that does not hash to `expected_hash`, or an endpoint flying another policy; an `https` endpoint the server has no REST TLS identity for, or whose TLS handshake fails (certificate not the lab CA's, wrong host); a package or source file missing on the server; on a rail: a scenario the rail cannot fly as written, a control arm the rig is not set to fly, a HotApply to an episode whose mission has ended or whose endpoint flies another policy, a flight the rig could not complete or accept, a mission that flew another policy |
| UNIMPLEMENTED | a hot-apply field the runner cannot carry; a rail backend (SITL, HIL) on a server with no rig for it; on a rail episode, an EmitEvent form with no REST body (a zone placed against the vehicle, a change of motion, an expected refusal; until 2026-11-14 every EmitEvent on a rail) |
| RESOURCE_EXHAUSTED | 256 open episodes, 64 open sessions, 512 events in one episode, or an InjectPolicy archive over 16 MiB; a rail StartEpisode while the rig flies another |
| UNAVAILABLE | (gRPC) a failed TLS handshake: no client certificate, one from another CA, plaintext on the TLS port; (service) a bound REST endpoint that does not answer, or answers 5xx |

## The KPI row and its nulls

EndEpisode's `KpiRow` is `guardrail.kpi.rollup` over the one episode (the
Stress Testing report's per-family row), with the null beside each score:

| Score | Null |
|---|---|
| P0 escape rate | passthrough (every raw command flown) |
| fail-safe correctness (label-based) | never trigger, always trigger |
| mission success rate | hover (a vehicle that never leaves the start) |
| repair success rate | always brake |
| mean repair count, mean repair magnitude, mean time to safe | tracked figures, no null |

Every figure is a proto3 `optional`. A figure the episode has no value for is
left unset, and `KpiRow.unset` says why (`never_unsafe`, `shield_off`, "no
expected_failsafe label"). It is never sent as 0. Measured on 9 Oct on
`nfz-head-on`, seed 1001: P0 escape 0.0 beside a passthrough null of 0.88
(0.947 on the 12 Oct tree, Measured below),
fail-safe correctness 1.0 beside never-trigger 1.0 and always-trigger 0.0 (one
labelled episode, so the score does not beat its never-trigger null); on its
Shield-off control `nfz-head-on-control`, the escape rate equals its own
passthrough null (0.173).

A test checks the typed row field by field against `guardrail.kpi.rollup`'s
output, with the twelve rollup paths written out in the test rather than read
from the service, over three episodes that between them set every figure
(nfz-head-on, its control, and an open flight whose mission succeeds beside
a hover null that does not). Before 9 Oct a misspelled path, swapped nulls or
a figure read from the wrong key all passed the tests.

## Mutual TLS

`python -m guardrail.harness_service gen-pki` writes, with `cryptography`:

| File | What |
|---|---|
| `ca.cert.pem`, `ca.key.pem` | the lab root CA: ECDSA P-256, CA:TRUE (path length 1), keyCertSign, 10 years |
| `harness-server.cert.pem` / `.key.pem` | serverAuth, SAN localhost and 127.0.0.1 (change with `--host`), 1 year |
| `harness-client.cert.pem` / `.key.pem` | clientAuth, 1 year |

The keys are written unencrypted: this is a development PKI. So the default
folder is **outside the checkout**, in the user's local data folder:
`%LOCALAPPDATA%\vla-guardrail\harness_pki` on Windows,
`$XDG_DATA_HOME/vla-guardrail/harness_pki` (or
`~/.local/share/vla-guardrail/harness_pki`) elsewhere. Until 9 Oct it was
`demo/out/harness_pki/`: gitignored, but this checkout lives in OneDrive, which
would have uploaded the root CA key - the key meant to seed SROS 2 keystores.
`gen-pki` and `issue` now refuse a folder inside OneDrive (the folders the
OneDrive client names in `%OneDrive%`, `%OneDriveConsumer%`,
`%OneDriveCommercial%`, or any folder named `OneDrive...`), Dropbox, Google
Drive or iCloud Drive, writing nothing, unless given `--allow-synced`. On
POSIX the key files are made owner-only (mode 600); on Windows `chmod` cannot
do that, so a key's protection is its folder's ACL, which under
`%LOCALAPPDATA%` is the user profile's (on a default install the user, SYSTEM
and Administrators; check with `icacls`). Nothing had been generated at the
old default on the development desktop.

An existing CA is never replaced without `--force`, because a new root
invalidates every certificate the old one signed, SROS 2 enclaves included.
`issue --name N [--server-host H]` adds a certificate from the existing CA
(for example a server certificate carrying the Orin's address).

The server presents its certificate and requires a client certificate that
chains to the CA (`require_client_auth=True`). It then also requires the
client's common name to be on an allow-list (default `harness-client`,
`--allow-client` to add). The reason is the grant's own choice: the same root
will sign every ROS 2 node's identity, so a valid chain alone would let any
node's certificate drive the harness. Revocation, for now, is removing a name
from the allow-list or rotating the CA; there is no CRL.

Plaintext (`--insecure`) is refused on any address that is not loopback.

## The same root CA for SROS 2 (run 2026-10-19; see docs/DESIGN-sros2.md)

**Superseded in part (2026-10-19).** This section is the procedure as written
on 2026-10-12, before any ROS 2 host ran it. Its first run (WSL Ubuntu 24.04,
ROS 2 Jazzy, sros2 0.13.6) showed step 2's reading wrong: `create_keystore`
reuses a CA only when `identity_ca.*` and `permissions_ca.*` (cert and key)
are all present, and otherwise wrote its own CA - which step 3's fingerprint
check caught. The script now links those four files to the copied lab CA
first, verifies the governance file against it (step 3b), takes its enclaves
from `sitl/sros2_enclaves.txt`, and `sitl/run_ros2_demo.sh` starts every ROS 2
process in its enclave under `SROS2=1`. docs/DESIGN-sros2.md has the current
procedure, the live check (`tools/sros2_check.sh`, 6/6) and the flight
command; the text below is kept for the CA-handling rules it states (the
keystore location, the CA key staying on the desktop).

The file names are sros2's own: a keystore keeps its CA as
`public/ca.cert.pem` and `private/ca.key.pem`, and the curve is the one
sros2 generates (P-256). `tools/sros2_setup.sh` does the whole procedure on
the ROS 2 host (WSL Ubuntu on the desktop, or the Orin's ROS 2 container),
with ROS 2 sourced and openssl on the PATH:

    bash tools/sros2_setup.sh --pki /mnt/c/Users/<you>/AppData/Local/vla-guardrail/harness_pki \
        --keystore ~/sros2_keystore            # --dry-run prints every command, runs none

1. It copies the lab CA into `keystore/public/ca.cert.pem` and
   `keystore/private/ca.key.pem` (umask 077, the key chmod 600), refusing a
   keystore that already holds another CA key (`--force` on purpose) and a
   keystore under `/mnt/` - a Windows drive, where chmod is not enforced and
   the folder may be synced (`--allow-mnt` on purpose).
2. `ros2 security create_keystore keystore`. It relies on sros2 reusing a CA
   pair it finds in `public/` and `private/` ("found CA key/cert pair") and
   pointing `identity_ca` and `permissions_ca` at it. That was our reading of
   the sros2 source on 2026-10-12, not yet run (no ROS 2 on this host); the
   first run on 2026-10-19 showed it wrong (the note at the top of this
   section), and step 3 is what caught it:
3. it compares the SHA-256 fingerprints of `identity_ca.cert.pem` and
   `permissions_ca.cert.pem` with the lab CA's, and stops if either differs
   (sros2 made its own CA; the keystore has to be rebuilt). The fingerprint
   gen-pki prints is the same one (`python -c "from guardrail.harness_service
   import fingerprint; print(fingerprint('<pki>/ca.cert.pem'))"`).
4. One enclave per process of the dev graph (`ros2 security create_enclave`):
   `/guardrail/safety_shield`, `/guardrail/mavlink_adapter`,
   `/guardrail/vla_stub` (the rclpy node names of
   `sitl/ros2_ws/src/safety_shield/launch/guardrail_rail.launch.py`),
   `/guardrail/mavros` and `/guardrail/bag_recorder` (the MAVROS node and
   the `ros2 bag record` that `sitl/run_ros2_demo.sh` starts).
5. `openssl verify -CAfile <pki>/ca.cert.pem` on each enclave's `cert.pem`.
6. It prints the environment that enforces security
   (`ROS_SECURITY_KEYSTORE`, `ROS_SECURITY_ENABLE=true`,
   `ROS_SECURITY_STRATEGY=Enforce`) and the enclave each process runs with:
   `ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/<node>` for the three Python
   nodes (the launch file gives them one shared environment, so each needs
   its own), `--ros-args --enclave ...` for MAVROS and `ros2 bag record`.

Checked on 2026-10-12: `bash -n tools/sros2_setup.sh` (Git Bash) and a
`--dry-run` that printed the commands above and refused a `/mnt/` keystore.
Run on a ROS 2 host on 2026-10-19 (WSL Jazzy, throwaway lab PKI): the fixed
script built the keystore with the lab CA in both roles, and
`tools/sros2_check.sh` passed 6/6 - a process with no enclave refused under
Enforce, an unsecured one and one under another CA hearing nothing, the
launch file's three nodes each in its own enclave (docs/DESIGN-sros2.md). The
CA key should leave the lab desktop only to seed the keystore: the Orin and
the drone carry enclave keys, never the CA key.

The script does not change who may publish what: `create_enclave` writes
sros2's default permissions, which allow every topic. Topic-level policies
(the Shield alone publishes `/shield/setpoint`, the VLA only
`/vla/action_4d`, ...) are a permissions XML per enclave, still to be
written. X-15 status: SROS 2 is opt-in behind `SROS2=1` (Enforce, one
enclave per process, `tools/sros2_check.sh` 6/6 in WSL Jazzy on 2026-10-19);
not yet flown; per-topic permissions not written. With the default
`SROS2=0` the Shield node, the MAVLink adapter and the VLA stub run unsecured
on the ROS 2 graph, as before.

## Rail sessions: one session, one flown and scored episode (2026-11-10)

Audit cards **WP4-05** and **WP4-03**. Code and headless tests only: **no
session has driven the real rig** (see "Not flown" at the end of this
section). Until this date `BACKEND_SITL` answered UNIMPLEMENTED, the rail was
started outside the harness (`tools/harness_client.py drive`, 3 Nov), the
harness could forward a spawn, a switch and a swap but not move, turn or
resize a zone, and nothing it wrote was called `episode.bag2/`.

### What flies it

Nothing new flies anything. A server started with a rig (`serve --rail hil`,
`Harness(rail=RailRig(...))`) hands a rail session's episode to
`experiments/sweep_scenarios.py`'s own rail backends:

| Backend | Flown by | Scored by |
|---|---|---|
| `BACKEND_HIL` | `run_hil` and its transport (`make_hil_transport`: SITL and the router restarted in WSL at the scenario's start, its files copied to the Orin, MAVROS restarted, the mission container run, the run folder fetched and accepted by its own records) | `score`, `rail_gate_set("ideal")`, `episode_verdict`, and `write_scenario_score` beside the run: what `--backend hil` runs with its default `--rail-gates ideal` |
| `BACKEND_SITL` | `run_sitl` (`sitl/run_ros2_demo.sh` in WSL) | the same three functions; no `scenario_score.json` (the campaign writes none for the desktop rail either) |

`run_hil`, `run_sitl` and `build_sitl_cmd` gained one optional keyword,
`extra_args`, which may hold the Shield node's hot-apply listener flags and
nothing else (`RAIL_HOTAPPLY_FLAGS`, `rail_extra_args`: a mission-shaping flag
handed in beside the scenario's own would fly another mission under its
name). A campaign passes none, and `tests/test_sweep.py` (unchanged) still
passes on it; `tests/test_harness_rail.py` checks that a session with no
endpoint launches exactly `rail_node_args`' list. The transport is injected
(`RailRig.transport`, `RailRig.sitl_runner`), so the tests fly on the fake rig
`tests/test_sweep.py` flies its campaign on.

### The lifecycle on a rail

| RPC | On a rail session |
|---|---|
| StartSession | `backend` SITL or HIL; `rest_endpoint` is the address the **mission's own** listener will be started on (it needs the port; off loopback it must be `https`, the server needs `--rest-pki`, `--allow-rest-host` and `--rail-hotapply-pki`). UNIMPLEMENTED on a server with no rig for the backend |
| InjectPolicy | a file both hosts hold (`bundle_path`; uploaded bytes are INVALID_ARGUMENT). Nothing flies yet, so `GET /health` is not asked: the mission is launched pinned to the hash (`--expected-hash`), the endpoint is checked against it when it first answers, and the fetched run's manifest is checked again before anything is bundled |
| IssueTask | recorded, as on the headless backend (the rails' VLA stub reads no text either) |
| StartEpisode | **launches the mission** and returns; the episode is RUNNING. What the rail cannot fly as written (`rail_refusal`), a Shield-off control arm on a rig without `--rail-controls`, a policy the rail cannot be pinned to, `live` / `pace`: refused, nothing launched. One mission at a time: a second StartEpisode while one flies is RESOURCE_EXHAUSTED. With `drive_scenario_events` (2026-11-14) the scenario's own events are the harness's to forward (below) |
| HotApply (episode) | queued for mission time `at_sim_t` (or `now`), `PolicyAck.pending`; forwarded then by the episode's driver, by the body's own route (below). The endpoint is the validator: it holds the flying Shield, the scenario's own events included, which the headless dry run never knew |
| HotApply (session) | forwarded at once, as before; while a rail episode of the session flies it is also recorded in that episode's bundle |
| EmitEvent | (2026-11-14; UNIMPLEMENTED until then) turned into its REST body (`rest_body_of_event`) and queued like a HotApply to the episode, `EventAck.pending` with the route and the body; it names its zone (`id`). A form with no REST body - a zone placed against the flying vehicle (`ahead_of_vehicle_m`), a change of motion (`mps`, `deg_per_s`), an expected refusal - is UNIMPLEMENTED |
| GetStatus | `backend`, and `rail_phase`: launching, awaiting_endpoint, awaiting_mission_start, mission, fetched, scored |
| EndEpisode | **blocks** until the mission has ended and its folder is fetched (minutes), scores it, writes the bundle. A flight the rig could not complete or accept, or one whose manifest names another policy, is FAILED_PRECONDITION with the reason; nothing is bundled and the rig is free |
| StopSession | waits for a flying rail episode (it cannot be called back) |
| Streams | `rail_phase`, each `hot_apply_forwarded` with the receipt or the refusal, and after the flight its rows as telemetry frames |

**Mission time.** The driver polls `GET <endpoint>/health` every 0.5 s until
the listener answers (it starts on the pad, before arming), then every 0.2 s
until it reports `mission_started` (the Shield's first `filter()` tick), and
not after. Mission time 0 is the moment the request that saw it was *sent*,
so this server's mission time runs late by at most one poll plus a round
trip: an event is forwarded that much late, never early. The rail's own
record of each applied event (metrics.json `hot_apply.events[].t`) is copied
beside this server's (`rail.t`). An endpoint whose first answer already says
`mission_started` gives no time 0, and nothing is forwarded to it. The polls
take the node's lock for the length of one `health()` call, on the pad only;
what that costs a real node's tick has not been measured.

### Zone edits: PATCH /dynamic_nfz (WP4-03)

The grant's event list is spawn / translate / rotate a fence, activate /
deactivate a rule, swap a corridor, scale a radius. `POST /hot_apply` carries
the spawn, the switch and the swap; an edit of a zone that exists is the
endpoint's `PATCH /dynamic_nfz/{id}`. `rest_route` picks the route from the
body, for a session HotApply, a rail episode's HotApply and a `drive`
timeline alike:

    {"type": "dynamic_nfz", "op": "translate", "id": "nfz-1", "dx": 5, "dy": 0}
    {"type": "dynamic_nfz", "op": "rotate",    "id": "nfz-1", "angle_deg": 30}
    {"type": "dynamic_nfz", "op": "scale",     "id": "nfz-1", "factor": 1.5}
    {"type": "dynamic_nfz", "op": "move",      "id": "nfz-1", "vertices": [...]}

goes as `PATCH <endpoint>/dynamic_nfz/nfz-1` with everything but `type` and
`id`; every other body goes to `POST /hot_apply` as it is. It uses the same
request function as the POST (`_rest`): no proxy, the server's REST TLS
identity for an `https` endpoint, the same code mapping (409 / 404 / 422).
`PolicyAck.forwarded_method` and `forwarded_to` say which route. `scale` is
the grant's "scale radius" on a polygon zone: `Shield.scale_nfz` scales the
ring about its vertex mean. Not forwarded: `DELETE /dynamic_nfz/{id}` (expiry is
not on the grant's list; a switch-off does it).

### A scenario's own events, driven by the harness (2026-11-14)

Until this date a library scenario's events on a rail were the mission's:
`run_hil` writes them to `--events` and the Shield node applies them on its
own tick (`guardrail.scenario_events.apply_event`). The harness could add
hand-written bodies beside them, not drive them. StartEpisode's
`drive_scenario_events` hands them to the harness:

1. `scenario_rest_timeline(spec)` turns each event into the REST body that
   makes the flying Shield apply it as the mission would
   (`rest_body_of_event`), in flight order: a spawn with `vertices` is
   POST /hot_apply with every field the mission defaults written out (P0,
   hard, `repair`, mission, 0-1000 m, 1 m margin; the REST handler refuses
   a dynamic_nfz without a `violation_action`); a translate (`dx_m`,
   `dy_m` -> `dx`, `dy`), rotate (`angle_deg`) or scale (`factor`) step is
   PATCH /dynamic_nfz/<id>; a switch and a swap are POST /hot_apply with
   the ids the mission would give them. A zone with no `id` is
   `dyn-nfz-<k>` and an edit with none edits the last spawn, as the
   mission's EventState counts them.
2. What has no REST form refuses the episode, naming every such event,
   nothing launched: a zone placed against the flying vehicle
   (`ahead_of_vehicle_m`: only the mission knows where the aircraft is), a
   change of motion (`mps`, `deg_per_s`: the mission restarts it from the
   ring the flying Shield holds at that instant), an event the scenario
   expects refused. So dyn-nfz-spawn-*, dyn-nfz-translate, dyn-nfz-rotate-
   spin, grant-example-sweep and the failsafe zones still fly only with the
   mission applying them; dyn-nfz-rotate-step, radius-scaling-*,
   time-window-switch-event, rule-*-mid-run and corridor-swap-* (13 cells)
   can be driven.
3. The mission is launched with the same scenario minus its events: no
   `--events`, no events.json pushed. Each event is queued at its mission
   time (source `scenario`) and forwarded by the episode's driver, so the
   flying Shield gets each once, from the harness.
4. After the flight each scenario event must have reached the Shield with a
   generation. One the endpoint refused, or that was never sent (the
   mission ended first, the listener never reported its start), makes the
   flight another one - the campaign's own rule for `--events`
   (`hil_flight_problems`) - so the episode is FAILED_PRECONDITION, not
   scored, nothing bundled; the run folder keeps what the mission recorded.

`tests/test_harness_hil.py` checks step 1 on every library cell it applies
to: a Shield given the events as the mission node gives them and a twin
given the REST bodies through `guardrail/api.py`'s handlers hold the same
policy hash and generation after each event. What differs is the clock: the
mission applies `--events` on its own tick, the harness forwards late by at
most one /health poll plus a round trip, and the bundle keeps both times.

**EmitEvent on a rail** goes the same way: the ScenarioEvent's REST body,
queued like a HotApply to the episode (`EventAck.pending`, the route and
the body); on a rail it names its zone.

**The dry run** (`Harness.plan_rail_episode`, `tools/harness_hil_episode.py
--dry-run`) resolves and refuses the request with StartEpisode's own
`_resolve` and `_rail_prepare`, builds the mission's arguments as `run_hil`
does (plus the listener's flags), and lists every forward, the run folder
and the bundle. It creates no session or episode, takes no rig and sends
nothing.

### The bundle, under the grant's names

A rail episode's bundle folder holds the page's six items:

| Grant's name | What it is here |
|---|---|
| `manifest.json` | the mission's own, byte for byte |
| `policy_bundle.tar.gz` | the policy file the rail was given and **every** generation's IR, from the run's `policy_g<N>.json`; a generation an endpoint receipt names with no IR beside it is a gap, and the index says so |
| `episode.bag2/` | the run's rosbag2 directory |
| `shield_audit.jsonl` | the run's, byte for byte |
| `harness_events.jsonl` | the start, each hot-apply this server forwarded (request, route, its mission time, the generation and the endpoint's receipt, or the refusal; the rail's own record beside it), what was never sent and why, the end |
| `kpi.json` | the KPI table, the verdict, the gates used, and `harness` (`bundle_files`: which of the six were written, and why not) |

plus `harness_request.json`, `manifest_extras.json` and, when asked,
`flight_log.jsonl`. A file the run folder lacks is **not made up**: no
manifest is invented for a flight this server did not record, and
`bundle_files` says "not written" with the reason (`EpisodeBundle.
bundle_files` carries the same map).

**`episode.bag2/`: an alias, not a rename.** The rail records its bag as
`bag/` in the run folder (`deploy/docker/entrypoint.sh`,
`sitl/run_ros2_demo.sh`). Renaming it there would change the flying side's
scripts and everything that reads the name: the mission's replay pack, the
exporters (WP4-13), the entrypoint's own handling of an earlier bag, and
every stored run. So the run folder keeps `bag/`, and the bundle gets the
same files under the grant's name: hard links where both sit on one volume
(no second copy of the bag), copies otherwise (`alias_tree`), each file's
SHA-256 in `kpi.json` (`harness.episode_bag`). rosbag2's `metadata.yaml`
names its files relative to the directory, so the alias opens as a bag. The
headless backend still writes none: it has no ROS 2 graph.

**Every generation, headless too.** `EpisodeRun.generations` (the sweep's
headless runner) now keeps the canonical IR of each generation, read right
after the event that produced it; `write_policy_bundle(held=...)` packs each
one that hashes to the recorded hash. Until this date the runner kept the
first and the last policy only, and a bundle with two or more hot-applies
said `complete: false`.

### Not flown, and what a flight needs

Tested: `tests/test_harness_rail.py`, 11/11, on the fake rig (no WSL, no ssh:
its "Orin" flies the scenario headless and writes the run folder; its
listener runs `guardrail/api.py`'s own handlers on a live Shield, served by
the standard library). That shows what the harness launches, forwards,
records, scores and bundles. It shows nothing about ArduPilot, MAVROS, the
Orin, the real listener (FastAPI / uvicorn, mutual TLS) or timing on a real
node. Open:

- **No rail session has flown**, on hil or on the desktop rail. WP4-03's
  edits have reached a flying Shield only by hand (29 Oct, REST), never
  through the harness; `StartEpisode` has never launched a real mission;
  no real `bag/` has been aliased.
- **The Orin's mission image, as its build files read, cannot serve the
  listener.** `deploy/docker/companion.Dockerfile` installs
  `requirements.txt` under `deploy/docker/constraints-companion.txt`, and
  neither names fastapi or uvicorn (the desktop rail's venv has both,
  `sitl/setup_ros2.sh`); a listener that cannot start stops the run before
  arming. Read from the files: the image built on the Orin was not
  inspected (no ssh while flights were up). The image needs both packages
  before a hil session with an endpoint can fly; a session with no endpoint
  (no hot-apply) needs neither.
- **Mutual TLS to the mission's listener** is the forward's existing path
  (tested since 2026-10-19 on a standard-library endpoint), not exercised by
  the PATCH tests, which use plaintext loopback.
- **KPI grade** is the mission manifest's and the Orin evidence's to earn
  (`is_kpi_grade`), as for a campaign run; the harness adds nothing to it.
- A failed rail flight leaves its forwards in the session's stream only:
  there is no bundle to keep them in.

The flight that would prove items 1-3 on hil, once the image carries
fastapi and uvicorn and with nothing else flying (Git Bash, the lab
checkout):

    # once: a server certificate naming the Orin's cable address, and the
    # three files the mission's listener reads (never the CA key), in the
    # Orin checkout's demo/out, which the container mounts
    python -m guardrail.harness_service issue --name orin-hotapply --server-host 10.42.0.3
    PKI="$LOCALAPPDATA/vla-guardrail/harness_pki"
    ssh nathan-agxiot@10.42.0.3 'mkdir -p ~/vla-drone/demo/out/_pki'
    scp "$PKI/ca.cert.pem" "$PKI/orin-hotapply.cert.pem" "$PKI/orin-hotapply.key.pem" \
        nathan-agxiot@10.42.0.3:vla-drone/demo/out/_pki/

    # window 1: the harness, with the rig
    python -m guardrail.harness_service serve --rail hil --rail-campaign hr01 \
        --allow-rest-host 10.42.0.3 --rest-pki "$PKI" \
        --rail-hotapply-pki /opt/vlaguard/demo/out/_pki \
        --rail-hotapply-server-cert orin-hotapply

    # window 2: one session = one flown, scored episode
    python tools/harness_client.py run --backend hil \
        --rest-endpoint https://10.42.0.3:8071 --scenario speed-cap --seed 1001 \
        --policy-bundle policies/sim_demo_policy.yaml \
        --expected-hash "$(python -c "from guardrail import load_policy; print(load_policy('policies/sim_demo_policy.yaml').policy_hash)")" \
        --hot-apply 3 zone.json --hot-apply 8 translate.json \
        --hot-apply 14 rotate.json --hot-apply 18 scale.json \
        --hot-apply 24 switch_off.json --keep-log --json

with the five bodies as files (a proposal, not a flown design: speed-cap
flies east along y = -40 from x = -40 at its 5 m/s cap, so this zone is
spawned beside the track 80 m ahead, moved onto it, turned, grown and
switched off; where to put it is the flight's own decision). Since
2026-11-26 (WP1-14) a body gives absolute positions in WGS84 only: the zone's
corners, (x, y) = (40, -62), (50, -62), (50, -47), (40, -47) m about
sim_demo_policy.yaml's frame origin (-35.363261, 149.16523), are written as
lat/lon with `guardrail.models.wgs84_from_metres` (a body with {x, y}
vertices is answered 422). The relative edits stay in metres:

    zone.json        {"type": "dynamic_nfz", "id": "nfz-h1", "constraint_type": "hard",
                      "scope": "mission", "priority": "P0", "violation_action": "project_fix",
                      "vertices": [{"lat": -35.362901673886, "lon": 149.164547037266},
                                   {"lat": -35.362811842358, "lon": 149.164547037266},
                                   {"lat": -35.362811842358, "lon": 149.164712270186},
                                   {"lat": -35.362901673886, "lon": 149.164712270186}]}
    translate.json   {"type": "dynamic_nfz", "op": "translate", "id": "nfz-h1", "dx": 0, "dy": 14}
    rotate.json      {"type": "dynamic_nfz", "op": "rotate", "id": "nfz-h1", "angle_deg": 30}
    scale.json       {"type": "dynamic_nfz", "op": "scale", "id": "nfz-h1", "factor": 1.5}
    switch_off.json  {"type": "time_window_switch", "target_id": "nfz-h1", "active": false,
                      "constraint_type": "hard", "scope": "mission", "priority": "P0",
                      "violation_action": "project_fix"}

What to read afterwards: the client's exit code and KPI row; in the bundle,
`harness_events.jsonl` (five forwards, generations 1-5, each receipt, each
`rail.t` no earlier than its `at_sim_t`), `policy_bundle.tar.gz`
(`complete: true`, six generations), `episode.bag2/` against the run's
`bag/`, `kpi.json` `harness.bundle_files` (six "written"), and
`kpi_grade`. The mission's arguments can be listed without flying with the
sweep's own dry run (`experiments/sweep_scenarios.py --backend hil
--hil-dry-run --only speed-cap`; the harness adds the `--hotapply-*` flags).

**A harness-driven hil episode (2026-11-14)** replaces the hand-placed
bodies above with a library scenario's own events, driven by the harness
(the section before "The bundle"). The same window 1, then (window 2, Git
Bash, the lab checkout, the same once-only PKI steps):

    python tools/harness_hil_episode.py --dry-run     # sends nothing
    python tools/harness_hil_episode.py               # flies it

Defaults: `dyn-nfz-rotate-step`, seed 1001, endpoint
`https://10.42.0.3:8071`, the scenario's own policy (its signed bundle on
both hosts). The server launches the mission with no `--events` and the
listener's flags, and forwards the bar's spawn at 3 s (POST /hot_apply) and
its 90-degree turn at 8 s (PATCH /dynamic_nfz/bar, `{"op": "rotate",
"angle_deg": 90.0}`). Its gates (P0 escape 0, the bar inert before the turn
and binding after it, no breach, no polygon entry) are the library's, and
its nightly cells have flown on hil with the mission applying both events,
so the harness-driven flight has a campaign twin. Read afterwards: the
tool's last line (`grant bundle: 6/6 names written`) and exit code;
`harness_events.jsonl` (two forwards, source `scenario`, generations 1-2,
each `rail.t` no earlier than its `at_sim_t`); `kpi.json`
`harness.rail.scenario_events_via` (`harness`); the run folder under the
server's `--rail-store` (default `<runs>/rail_runs/<tag>`), whose
`scenario.json` records no events. The scale case is
`--scenario radius-scaling-shrink --cell radius-scaling-shrink@factor=0.5`.

## What is not done

- **A rail session flown.** `BACKEND_SITL` and `BACKEND_HIL` are built and
  tested on a fake rig (2026-11-10, "Rail sessions" above; the
  harness-driven scenario events, 2026-11-14); no session has driven the
  real one, and the Orin's mission image lacks fastapi and uvicorn for the
  listener. The pymavlink rail is not a backend.
- **Pose in the live telemetry.** `EpisodeRunner` exposes no per-tick row
  while it flies; live frames carry the clock and the pose comes with the
  flown rows when the episode ends.
- **A dry run that knows the whole timeline.** HotApply is validated against
  the other hot-applies only; a conflict with the scenario's own events or an
  EmitEvent is found when it flies (FAILED_PRECONDITION with the runner's
  reason), in live mode too.
- **The grant's full episode bundle, from a flight.** The six names are
  written for a rail episode and every generation's IR is packed
  (2026-11-10), shown on a fake rig's run folder; no real run folder has
  been bundled. `episode.bag2/` is not written on the headless path (no
  ROS 2 graph).
- **SROS 2 on the ROS 2 graph (card X-15).** Opt-in behind `SROS2=1`
  (Enforce, one enclave per process, `tools/sros2_check.sh` 6/6 in WSL Jazzy
  on 2026-10-19, docs/DESIGN-sros2.md); not yet flown (no SITL flight with
  `SROS2=1`, so MAVROS under Enforce is untested); per-topic permissions not
  written; the default `SROS2=0` leaves the graph unsecured.
- **No CancelEpisode**: an episode a client gives up on expires after the
  idle TTL (ARMED), finishes on its own (live), or is closed by StopSession;
  a session a client gives up on expires after the session TTL once nothing
  of it is armed or flying. A StopSession (or an expiry) that lands while a
  live episode is already writing its bundle leaves the bundle folder on
  disk with the episode reading FAILED.
- **Packaging** is done: `pyproject.toml` has the `harness` extra
  (`grpcio>=1.84.0`, `protobuf>=7.35.1`, `cryptography`) and ships
  `guardrail.protos` with `protos/*.proto`, and `tests/test_env_pins.py`
  maps the `grpc` and `google` imports to it (37/37 on 3.10, 2026-10-12).
  The 2026-10-09 note that it failed 2 of its 34 tests is out of date. The
  service still needs a checkout: it drives `experiments/sweep_scenarios.py`,
  which no wheel carries.
- **The verdict copy is gone** (2026-10-11): both paths call
  `experiments/sweep_scenarios.py` `episode_verdict`.
- **CI.** Since 2026-10-11 `.github/workflows/tests.yml` runs
  `tests/test_harness_service.py` and installs grpcio in the 3.10 and 3.11
  jobs; whether it runs `tests/test_wp4outputs_harness.py` is the lead's to
  add.
- **Throughput.** Scheduled episodes fly one at a time per server (a lock
  around `run_episode`; a live episode takes it a tick at a time); the
  reference design's answer to its risk R9 is parallel hosts. Episodes and
  sessions live in memory: a restarted server forgets them, while their
  bundles stay on disk.
- **KPI-grade numbers.** None: the headless topology fails
  `is_kpi_grade`, and EpisodeBundle says so with the reasons.

## Measured (2026-10-13, development desktop, after review)

    python tests/test_wp4outputs_harness.py   # 16/16 on 3.10.20 and on 3.11.15
    python tests/test_harness_service.py      # 49/49 on 3.10.20 and on 3.11.15

- Four tests added after review: a live EmitEvent (a zone spawned 15 m ahead
  at 5 s) flies byte for byte as the scheduled episode with that event, and
  not as the null without it; a session's seed is its episodes' default (an
  unset StartSession seed stays unset over gRPC); an idle session expires
  and frees its slot under MAX_OPEN_SESSIONS while one with an ARMED episode
  does not; `ticks_flown` is unset until a scheduled episode has ENDED, then
  300 on the 30 s episode. Three existing tests were tightened: StopSession
  must stop the stepping thread (`ticks_flown` stays put, well under the
  episode's 300 ticks), EndEpisode must finish a live episode paced at 0.05
  within 60 s (paced, its rest would take about ten minutes), and a
  telemetry frame's `executed` must equal the row's `emitted` on a tick the
  Shield changed the command.
- All four fail on the 2026-10-12 service (no `session_idle_ttl_s`,
  `ticks_flown` always present) or on 4418ae0 (0/16 there: no sessions).
  Deliberate breaks of the service, each alone in a copy of the tree: the
  live EmitEvent not injected, the session seed ignored, an unset seed read
  as 0, `executed` taken from the raw command, the stop ignored by the
  loop, EndEpisode leaving a live episode paced (the wake removed too: the
  pause event stays set once woken, so dropping only `pace = 0` changes
  nothing), the session TTL never firing, an ARMED episode not protecting
  its session, a call not counting as activity, `ticks_flown` always
  reported, and the flown rows not counted: eleven, all caught (the paced
  EndEpisode by the 150 s timeout of the run).
- `test_a_windowed_switch_flies_as_written`: repair counts stopped
  separating the two flights because the escalation FSM changed, not the
  repair chain. On 4418ae0 the Monday flight's 250 zone ticks after the 5 s
  switch were Shield-flown Brake ticks (257 repairs, against 7 switched
  off); on this tree the FSM hands the zone to LOITER, so they are
  autopilot ticks, which `compute()` does not count as repairs (2 and 2).
  The zone ticks (250 against 0) separate the flights on both trees.

## Measured (2026-10-12, development desktop)

    python tests/test_wp4outputs_harness.py   # 12/12 on 3.10.20 and on 3.11.15
    python tests/test_harness_service.py      # 49/49 on 3.10.20 and on 3.11.15

- `test_a_windowed_switch_flies_as_written` checked which way the window
  changed the flight by comparing repair counts; on this wave's Shield both
  flights log 2 repairs (the 2026-10-09 service fails the same way on this
  tree), so it now checks the zone itself: on the Monday flight it still
  raises violations after the 5 s switch (250 ticks), switched off with no
  window it raises none (0). The byte-for-byte comparison with the same
  switch written in the scenario is unchanged.
- The page's lifecycle over mutual TLS on 127.0.0.1, ten rounds after one
  warm-up, medians: StartSession 0.001 s, InjectPolicy (a policy YAML) 0.007
  s, IssueTask 0.001 s, StartEpisode 0.011 s, EndEpisode (scheduled,
  `nfz-head-on` seed 1001) 1.709 s, StopSession 0.001 s. EndEpisode was
  0.375 s on 9 Oct: the flight itself is slower on today's tree (a profile
  puts 2.1 of 2.5 s in `Shield.filter`, code another package is changing,
  and the desktop was running other packages' tests at the time). P0 escape
  0.0 beside a passthrough null of 0.947 (0.88 on 9 Oct).
- A live `nfz-head-on` (30 simulated seconds) at pace 10 took 3.11 s of wall
  time and its manifest measured `sim_speedup` 10.0; at pace 1.0, 30.01 s and
  `sim_speedup` 1.0. Both stay not KPI-grade (topology
  `headless-kinematic`), and the stream carried 600 frames for each (300
  clock frames while it flew, 300 flown rows after).
- Thirteen deliberate breaks of this round's service code, each applied alone
  to a copy of the tree: live injection skipped, `now` landing a tick late,
  the already-flown check removed, StopSession leaving a live episode flying,
  the REST request through the environment's proxy, the loopback rule
  removed, InjectPolicy's endpoint check and its hash pin removed, the
  session's policy not flown, the task's hash not recorded, the flown rows
  not streamed, the SITL refusal removed, and a stream handing out one item
  per poll. Twelve fail a test; the last is slower, not wrong (every item
  still arrives), and survives. The late `now` survived the first run and is
  what `test_now_lands_on_the_next_tick_to_fly` was added for.
- Shown failing first: against the 4418ae0 tree, every test of
  `tests/test_wp4outputs_harness.py` fails (0/11 of the first eleven, run
  there: the new RPCs and keywords do not exist), and the reworked `test_the_pages_ten_rpcs_and_get_status_are_served`
  fails there: the 4418ae0 .proto lacks six of its eleven methods
  (InjectPolicy, IssueTask, StartSession, StopSession, StreamEvents,
  StreamTelemetry).

## Measured (2026-10-09, development desktop)

    python tests/test_harness_service.py      # 48/48 on 3.10.20 and on 3.11.15

- Round trip on 127.0.0.1, `nfz-head-on` seed 1001, ten episodes per mode
  after one warm-up: StartEpisode median 0.011 s plaintext and 0.010 s mutual
  TLS; EndEpisode median 0.382 s plaintext and 0.375 s mutual TLS (the flight
  dominates; the handshake is paid once per channel). P0 escape 0.0 beside a
  passthrough null of 0.88 in both modes. (On 8 Oct EndEpisode was 0.30 s;
  the Shield and FSM code it flies changed in between.)
- The CLI end to end: `gen-pki` into a temporary folder, `serve` (mutual TLS,
  127.0.0.1) as its own process, `tools/harness_client.py` from another
  working directory: exit 0 and the KPI row above; a run with a refused
  HotApply exits 2 and names the episode it left ARMED. The server was
  stopped and its port checked free.
- Fifty-one deliberate breaks of `guardrail/harness_service.py` (and one of
  `tools/harness_client.py`), each applied alone in a copy of the tree and
  the file restored after it (hash checked). Nineteen are the 8 Oct set,
  re-expressed on this code (client certificates not required, the allow-list
  skipped, queued events dropped, the seed ignored, unset figures sent as 0,
  ...). Nineteen are the reviewer's: the KPI row reading a misspelled or
  wrong rollup path or swapping the two fail-safe nulls; the verdict dropping
  label failures or reporting known_failure, unexpected_pass or tracked as
  pass; a hot-applied zone losing its margin, breach action, altitude band
  or motion, or a switch its window; the policy-changed guard, the GPS-dropout
  schedule check or the ProtectedTextError branch removed; keep_flight_log
  ignored; HarnessUnavailable sent as UNAVAILABLE; eviction newest-first.
  Thirteen break this round's fixes (arrival-order validation, the
  later-body code, idle expiry, the idle clock, the waiting EndEpisode, the
  synced-folder refusals, the PKI default, the client tool's ARMED notice).
  Every one fails at least one test. On 8 Oct's 33 tests, 18 of the
  reviewer's 19 had survived.
- OpenSSL 3.5.5 (Git for Windows) checks a fresh `gen-pki` folder the way the
  SROS 2 steps above do: `openssl verify -CAfile ca.cert.pem` passes both
  leaves, `-purpose sslclient` / `-purpose sslserver` pass the client and the
  server certificate, the client certificate is refused as a server one
  (error 26, "unsuitable certificate purpose"), the CA is P-256 with
  `CA:TRUE, pathlen:1`, and `openssl x509 -fingerprint -sha256` prints the
  same fingerprint as `gen-pki`.
- With `cryptography` blocked (3.11.15), 39 of the 48 tests run and the 9
  that need certificates print SKIP. With grpcio and protobuf blocked as well
  (what CI's third job installs: requirements.txt only), 18 run - the KPI
  row, the verdict, the event translation, the ordering, the lifecycle and
  the PKI-folder rules, all on `Harness` directly - and 30 print SKIP; none
  reports PASS without running.
- Shown failing first. Against the 10451bc tree the test file stops at its
  import (`guardrail.harness_service` does not exist there). Against the
  service as it stood on 8 Oct (before this round), the new tests fail where
  a defect was: a switch scheduled before its zone acknowledged ("a switch
  scheduled before its zone was acknowledged"); a second EndEpisode refused
  while the first flew; `armed_idle_ttl_s` unknown (no expiry); the PKI
  default inside the checkout and keys written into a folder named OneDrive;
  and the client tool silent about the episode it left ARMED. Of the 48 tests
  42 passed there (the coverage tests pass on the old code by design: they
  kill mutants, they do not mark a defect). On 9 Oct, earlier, two tests were
  run against the service
  before their fix and failed: the arm drawn at StartEpisode ("was
  accepted"), and a protobuf version refusal reported as HarnessUnavailable
  (it escaped as a VersionError traceback).
