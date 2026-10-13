# DESIGN: the stress harness's gRPC control plane, and one root CA for it and SROS 2

Audit cards **WP4-05** (no gRPC control service for the harness) and **X-15**
(SROS 2 security on the ROS 2 graph, with the harness mTLS reusing its root
CA). **SROS 2 is not enabled: the ROS 2 graph is still unsecured.** What this
note covers for X-15 is preparation only: the gRPC side's mutual TLS, a lab
root CA built to be shared, and a script that seeds a SROS 2 keystore with it
(`tools/sros2_setup.sh`), checked with `bash -n` and `--dry-run` but not run
on a ROS 2 host. Built 2026-10-08, revised 2026-10-09 after review; on
2026-10-12 the service gained the page's other six RPCs (StartSession,
StopSession, InjectPolicy, IssueTask, the two streams), a live mode that
injects events while an episode flies, forwarding of a session's HotApply to
a REST endpoint, and a reserved SITL backend. Nothing here was flown on a rail: the
service drives the headless harness, whose episodes are never KPI-grade.

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
| `tools/harness_client.py` | `HarnessClient` (all eleven RPCs) and a CLI: `run` (with a session when given a policy, a prompt or `--live`), `forward`, `status` |
| `tools/sros2_setup.sh` | The SROS 2 keystore and enclaves of the dev graph, seeded with the harness CA (not run here) |
| `tools/rerun_episode.py` | Re-flies a bundle and compares the flight logs (used here on a live episode's bundle) |
| `tests/test_harness_service.py` | 49 tests, standalone runner; the logic tests drive `Harness` without gRPC |
| `tests/test_wp4outputs_harness.py` | 12 tests: live mode, sessions, InjectPolicy, IssueTask, REST forwarding, the streams, the lifecycle over mTLS |

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
| StartSession | opens a session: backend (HEADLESS; SITL is reserved and answers UNIMPLEMENTED), the page's scene and airframe (recorded, not used: no simulator to load), a default seed, an optional REST endpoint the session is bound to |
| StopSession | closes it: ARMED episodes fail, a live one stops (FAILED, no bundle), a scheduled one already flying finishes; the streams end |
| InjectPolicy | the policy the session's later episodes fly: a bundle or policy YAML under `policies/` or `bundles/`, or the archive's bytes, pinned to `expected_hash` (required); in a REST-bound session the endpoint must be flying it |
| HotApply | to an episode: the Policy DSL's REST hot-apply body, validated by `guardrail.api.apply_event`, flown at `at_sim_t` (or `now`, live); to a REST-bound session: POSTed to `<endpoint>/hot_apply` |
| StartEpisode | arms one ScenarioSpec cell with a seed, an arm, optionally a profile and a session; `live` starts it flying at `pace` |
| IssueTask | records the prompt's SHA-256 (manifest_extras.json) for an ARMED episode or the session's later ones; the headless pilot reads no text |
| EmitEvent | a ScenarioEvent, the grant's seven types, at `at_sim_t` (or `now`, live) |
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
   when it is made does not change what flies.
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
`flight_log.jsonl` when asked. **Not written:** `policy_bundle.tar.gz`
(the episode records the policy's hash, not a copy), `episode.bag2/` (there is
no ROS 2 graph on the headless path to record) and `shield_audit.jsonl` (no
audit writer runs on this path). Those three belong to a rail episode.

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
stream cannot end before it has sent `session_stopped`. Each open stream
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

## Status codes

| Code | When |
|---|---|
| INVALID_ARGUMENT | the spec, cell, seed, arm, event, pace or session request fails validation (pydantic's message is passed through); a paraphrase arm on a protected (AerialVLA) prompt; an injected archive or policy the loader refuses; `now` on a scheduled episode; a REST 422 |
| NOT_FOUND | an unknown scenario, cell, profile, episode, session or policy file; a REST 404 |
| ALREADY_EXISTS | a REST 409 (a rule id in use) |
| OUT_OF_RANGE | an event after the episode's last tick, or before the next tick a live episode flies |
| PERMISSION_DENIED | a client certificate not on the allow-list; a policy (a client-sent scenario's, an InjectPolicy path) outside `policies/` and `bundles/`; a REST endpoint off loopback the server does not allow |
| FAILED_PRECONDITION | an event for an episode no longer taking them (ended, failed, expired, or a scheduled one being flown); an episode the runner refused in flight (GetStatus says why); a paraphrase arm this server cannot draw; a hot-apply that would make an earlier-acknowledged one fail later in the flight; a stopped session; a session HotApply with no endpoint bound; an injected policy that does not hash to `expected_hash`, or an endpoint flying another policy; a package or source file missing on the server |
| UNIMPLEMENTED | a hot-apply field the runner cannot carry; the reserved SITL backend |
| RESOURCE_EXHAUSTED | 256 open episodes, 64 open sessions, 512 events in one episode, or an InjectPolicy archive over 16 MiB |
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

## The same root CA for SROS 2 (a script; not run here)

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
   pointing `identity_ca` and `permissions_ca` at it. That is our reading of
   the sros2 source, not something run here (no ROS 2 on this host), so:
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

Checked here: `bash -n tools/sros2_setup.sh` (Git Bash) and a `--dry-run`
that prints the commands above with the given paths and refuses a `/mnt/`
keystore. Not checked: any of it on a ROS 2 host. The CA key should leave
the lab desktop only to seed the keystore: the Orin and the drone carry
enclave keys, never the CA key.

The script does not change who may publish what: `create_enclave` writes
sros2's default permissions, which allow every topic. Topic-level policies
(the Shield alone publishes `/shield/setpoint`, the VLA only
`/vla/action_4d`, ...) are a permissions XML per enclave, still to be
written. Until the script runs on the ROS 2 host and the launch file starts
each node with its enclave, X-15 is open: there is no keystore and no
enclave, and the Shield node, the MAVLink adapter and the VLA stub still run
unsecured on the ROS 2 graph.

## What is not done

- **The SITL backend.** `BACKEND_SITL` is a reserved value that answers
  UNIMPLEMENTED; the rails (`sitl/run_ros2_demo.sh`, the pymavlink rail) fly
  their own episodes. A REST-bound session can already forward hot-applies to
  a rail's live endpoint, which is the part of the hil lifecycle that works
  today.
- **Pose in the live telemetry.** `EpisodeRunner` exposes no per-tick row
  while it flies; live frames carry the clock and the pose comes with the
  flown rows when the episode ends.
- **A dry run that knows the whole timeline.** HotApply is validated against
  the other hot-applies only; a conflict with the scenario's own events or an
  EmitEvent is found when it flies (FAILED_PRECONDITION with the runner's
  reason), in live mode too.
- **The grant's full episode bundle.** `policy_bundle.tar.gz`,
  `episode.bag2/` and `shield_audit.jsonl` are not written on the headless
  path (above).
- **SROS 2 on the ROS 2 graph (card X-15).** Not enabled: the script exists,
  has not been run on a ROS 2 host, and the launch file does not yet start
  each node with its enclave. Topic-level permissions are not written.
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
