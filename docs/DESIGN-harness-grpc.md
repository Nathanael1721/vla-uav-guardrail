# DESIGN: the stress harness's gRPC control plane, and one root CA for it and SROS 2

Audit cards **WP4-05** (no gRPC control service for the harness) and **X-15**
(SROS 2 security on the ROS 2 graph, with the harness mTLS reusing its root
CA). **SROS 2 is not enabled: the ROS 2 graph is still unsecured.** What this
note covers for X-15 is preparation only: the gRPC side's mutual TLS, a lab
root CA built to be shared, and a procedure for seeding a SROS 2 keystore with
it that has not been run. Built 2026-10-08, revised 2026-10-09 after review.
Nothing here was flown on a rail: the service drives the headless harness,
whose episodes are never KPI-grade.

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
| `guardrail/protos/harness.proto` | The service `StressHarness`, in the page's message names |
| `guardrail/protos/harness_pb2.py`, `harness_pb2_grpc.py` | Generated, committed (command below) |
| `guardrail/harness_service.py` | `Harness` (no transport), the servicer, `serve()`, the PKI, a CLI |
| `tools/harness_client.py` | `HarnessClient` and a CLI that prints the KPI row with its nulls |
| `tests/test_harness_service.py` | 47 tests, standalone runner; the logic tests drive `Harness` without gRPC |

Regenerating the stubs, from the repository root:

    python -m grpc_tools.protoc -I . --python_out=. --grpc_python_out=. guardrail/protos/harness.proto

grpcio-tools 1.84.0 writes protobuf gencode 7.35.1, so the stubs need
grpcio >= 1.84.0 and protobuf >= 7.35.1 (installed: vla-real has grpcio
1.84.0 / protobuf 7.36.2, vla-drone 1.84.0 / 7.35.1). The generated
`harness_pb2.py` checks the protobuf runtime when it is imported and refuses
an older one. Its header says "NO CHECKED-IN PROTOBUF GENCODE", which is
protobuf's general advice; the grant's Outputs table asks for the stubs, so
they are committed, and a test regenerates them into a temporary folder and
fails if the committed files differ.

### The RPC surface against the page's

| Page's RPC | Here | Why |
|---|---|---|
| StartEpisode | built | arms one ScenarioSpec cell with a seed, an arm and optionally a profile |
| HotApply | built | the Policy DSL's REST hot-apply body, validated by `guardrail.api.apply_event` |
| EmitEvent | built | a ScenarioEvent, the grant's seven types |
| EndEpisode | built | flies, scores, writes the episode bundle (three of the grant's six items, see below), returns it with the KPI row |
| GetStatus | ours | state, queued events, verdict, bundle path, error, client name |
| StartSession / StopSession | not built | the headless runner has no simulator session to open; needed once the service drives the hil rail |
| InjectPolicy | not built | the policy is part of the ScenarioSpec (`policy: {path}` or `{bundle_path, expected_hash}`) |
| IssueTask | not built | the headless pilot reads no text; meaningful once a VLA flies |
| StreamTelemetry / StreamEvents | not built | an episode is flown inside EndEpisode, so there is nothing live to stream yet |

A client calling an RPC that is not in the .proto gets UNIMPLEMENTED from gRPC
(a test calls all six by name and checks the code).

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
2. **HotApply** and **EmitEvent** each carry `at_sim_t`. They are validated
   when they arrive (a HotApply at its place in flight order, below) and
   queued.
3. **EndEpisode** adds the queued events to the cell's own, flies the episode
   with `experiments/sweep_scenarios.py`'s `run_episode`, scores it with
   `score`, `label_failures` and `check_gates`, writes the episode bundle with
   `episode_manifest` and `write_bundle`, and returns the bundle path, the
   verdict, the full KPI table (JSON), the episode's KPI-report row (typed,
   each score beside its null) and the hot-apply events as they were applied
   in flight (generation, policy hash, refusals).

The per-episode verdict (pass, tracked, fail, known_failure,
unexpected_pass) is the sweep's own: since 2026-10-11 both the sweep's
`main()` and this service call `experiments/sweep_scenarios.py`
`episode_verdict` (guarded by
`test_the_harness_judges_with_the_sweeps_own_verdict_function`). Two older
tests still check it: one reaches every status
from one synthetic scenario by changing only its labels, and one runs the
sweep's own `main()` on five smoke-profile families and flies each of its
episodes again through the harness, comparing status, failed gates, label
failures and the known-failure reason episode by episode. Moving the verdict
into a public function that both call is listed below as a change outside
this note's files.

### The episode bundle, against the grant's

The Stress Testing page's episode bundle is `manifest.json`,
`policy_bundle.tar.gz`, `episode.bag2/`, `shield_audit.jsonl`,
`harness_events.jsonl` and `kpi.json`. The headless runner writes three of
them: `manifest.json` (the six fields), `harness_events.jsonl` and `kpi.json`.
The service adds `harness_request.json` (the request and every event as it
arrived, so the episode can be asked for again), and the sweep's
`write_bundle` adds `manifest_extras.json` when the episode has an arm record
and `flight_log.jsonl` when asked. **Not written:** `policy_bundle.tar.gz`
(the episode records the policy's hash, not a copy), `episode.bag2/` (there is
no ROS 2 graph on the headless path to record) and `shield_audit.jsonl` (no
audit writer runs on this path). Those three belong to a rail episode.

### Events are scheduled, not live

`run_episode` flies a whole episode in one call (about 0.3-0.4 s for 30
simulated seconds) and has no hook to pause between ticks. So an event does
not act "now": it acts at its `at_sim_t` when EndEpisode flies the episode,
applied by the runner's own `apply_event`, exactly as the same event written
in the scenario's `events:` would be. The tests check that equivalence byte
for byte on the flight log: a hot-applied zone, a hot-applied corridor swap
(against the library's own `corridor-swap-shifted`) and an EmitEvent.

This is also what makes an episode reproducible: the flight is a function of
(cell, seed, arm, scheduled events) and of the code. A live mode, where an
event lands at whatever tick the episode has reached, needs a steppable runner.
Since 2026-10-11 `experiments/sweep_scenarios.py` has one, `EpisodeRunner`
(`step()`, `inject(event)`, `finish()`; since 2026-10-12 it fails fast after
an error instead of hanging), but this service does not use it yet; it is the first thing the
hil rail will need, because a real autopilot cannot be paused.

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
the id of a zone they spawn) is acknowledged and then fails at EndEpisode
with FAILED_PRECONDITION and the runner's reason. Closing that needs the
service to drive `EpisodeRunner` (above), which it does not yet.

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

## Status codes

| Code | When |
|---|---|
| INVALID_ARGUMENT | the spec, cell, seed, arm or event fails validation (pydantic's message is passed through); a paraphrase arm on a protected (AerialVLA) prompt; a REST 422 |
| NOT_FOUND | an unknown scenario, cell, profile, episode or policy file; a REST 404 |
| ALREADY_EXISTS | a REST 409 (a rule id in use) |
| OUT_OF_RANGE | an event after the episode's last tick |
| PERMISSION_DENIED | a client certificate not on the allow-list; a client-sent scenario whose policy is outside `policies/` and `bundles/` |
| FAILED_PRECONDITION | an event for an episode no longer ARMED (ended, failed or expired); an episode the runner refused in flight (GetStatus says why); a paraphrase arm this server cannot draw; a hot-apply that would make an earlier-acknowledged one fail later in the flight; a package or source file missing on the server |
| UNIMPLEMENTED | a hot-apply field the runner cannot carry |
| RESOURCE_EXHAUSTED | 256 open episodes, or 512 events in one |
| UNAVAILABLE | (gRPC) a failed TLS handshake: no client certificate, one from another CA, plaintext on the TLS port |

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
expected_failsafe label"). It is never sent as 0. Measured on
`nfz-head-on`, seed 1001: P0 escape 0.0 beside a passthrough null of 0.88,
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

## The same root CA for SROS 2 (a procedure; not run here)

The file names are sros2's own: a keystore keeps its CA as
`public/ca.cert.pem` and `private/ca.key.pem`, and the curve is the one
sros2 generates (P-256). The steps, on the ROS 2 host (Humble):

    # 1. Seed a keystore with the lab CA.
    mkdir -p keystore/public keystore/private
    cp <pki>/ca.cert.pem keystore/public/ca.cert.pem
    cp <pki>/ca.key.pem  keystore/private/ca.key.pem
    ros2 security create_keystore keystore

    # 2. Check that sros2 used it rather than making its own.
    openssl x509 -noout -fingerprint -sha256 -in keystore/public/identity_ca.cert.pem
    openssl x509 -noout -fingerprint -sha256 -in keystore/public/permissions_ca.cert.pem
    #    both must equal the fingerprint gen-pki printed (also: python -c
    #    "from guardrail.harness_service import fingerprint; print(fingerprint('<pki>/ca.cert.pem'))")

    # 3. One enclave per node of sitl/ros2_ws (node names safety_shield,
    #    mavlink_adapter, vla_stub), each checked against the same CA.
    ros2 security create_enclave keystore /guardrail/safety_shield
    ros2 security create_enclave keystore /guardrail/mavlink_adapter
    ros2 security create_enclave keystore /guardrail/vla_stub
    openssl verify -CAfile <pki>/ca.cert.pem keystore/enclaves/guardrail/safety_shield/cert.pem

    # 4. Run with security enforced; each node is started with its enclave
    #    (--ros-args --enclave /guardrail/safety_shield).
    export ROS_SECURITY_KEYSTORE=$PWD/keystore
    export ROS_SECURITY_ENABLE=true
    export ROS_SECURITY_STRATEGY=Enforce

Step 1 relies on `create_keystore` reusing a CA pair it finds in
`public/` and `private/` instead of generating one. That is our reading of
the sros2 source, not something run on this host (no ROS 2 here): step 2 is
the check, and if a fingerprint differs, sros2 made its own CA and the
keystore has to be rebuilt. The CA key should leave the lab desktop only to
seed the keystore: the Orin and the drone carry enclave keys, never the CA
key.

Until those steps run on the ROS 2 host, X-15 is open: there is no keystore
and no enclave, and the Shield node, the MAVLink adapter and the VLA stub
still run unsecured on the ROS 2 graph. The CA and this procedure are
preparation for it, not part of it.

## What is not done

- **Live events.** Events are scheduled at `at_sim_t`; a live mode needs the
  service to drive `EpisodeRunner` (it exists since 2026-10-11; the service
  does not use it yet).
- **A dry run that knows the whole timeline.** HotApply is validated against
  the other hot-applies only; a conflict with the scenario's own events or an
  EmitEvent is found at EndEpisode (FAILED_PRECONDITION with the runner's
  reason), not when the HotApply arrives. Needs the service on
  `EpisodeRunner`.
- **The session RPCs, InjectPolicy, IssueTask and the streams** (table above),
  and no CancelEpisode: an episode a client gives up on expires after the
  idle TTL instead.
- **The grant's full episode bundle.** `policy_bundle.tar.gz`,
  `episode.bag2/` and `shield_audit.jsonl` are not written on the headless
  path (above).
- **SROS 2 on the ROS 2 graph (card X-15).** Not enabled: no keystore, no
  enclaves, the ROS 2 nodes run unsecured. The CA and the procedure above are
  preparation only.
- **Packaging.** `pyproject.toml` has no `harness` extra and does not list
  `guardrail.protos`; until it does, `tests/test_env_pins.py` fails 2 of its
  34 tests: the `grpc` / `google` imports are declared nowhere, and
  `protos/harness.proto` is not shipped as package data. Those two failures
  are that test doing its job. The fix is outside this note's files:
  `pyproject.toml` gains `harness = ["grpcio>=1.84.0", "protobuf>=7.35.1",
  "cryptography"]`, `packages = ["guardrail", "guardrail.protos"]` and
  `"protos/*.proto"` in package-data; `tests/test_env_pins.py` maps the
  import names `grpc` and `google` to the `harness` extra (and to their
  distribution names, grpcio and protobuf, which differ from the import
  names). The service needs a checkout anyway: it drives
  `experiments/sweep_scenarios.py`, which no wheel carries.
- **The verdict copy is gone** (2026-10-11): both paths call
  `experiments/sweep_scenarios.py` `episode_verdict`.
- **CI.** Since 2026-10-11 `.github/workflows/tests.yml` runs this test file
  and installs grpcio in the 3.10 and 3.11 jobs.
- **Throughput.** One episode is flown at a time per server (a lock around
  `run_episode`); the reference design's answer to its risk R9 is parallel
  hosts. Episodes live in memory: a restarted server forgets them, while
  their bundles stay on disk.
- **KPI-grade numbers.** None: the headless topology fails
  `is_kpi_grade`, and EpisodeBundle says so with the reasons.

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
