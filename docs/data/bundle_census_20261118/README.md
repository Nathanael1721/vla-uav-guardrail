# Bundle census, 18 Nov 2026 (WP1-14)

What `tools/bundle_census.py` says about the lab checkout's `bundles/`: every
bundle, the form its points are stored in, whether it flies with every grant
event available (`flight_ready`: WGS84 or point-free, with a frame origin),
and the WGS84 bundle each task with a bundle that is not is to be flown
from. Checked by `tests/test_bundle_census.py`.

| File | What it is |
|---|---|
| `census_before.json` | The lab checkout's `bundles/` on 18 Nov 2026 (42 policy bundles, 8 replay packs). 13 policy bundles store their points in local metres (the legacy read form) and 4 more carry no point and no frame origin: 17 bundles that are not flight-ready, over 10 tasks. Two problems: `follow-car-citylife-nfz3` and `follow-car-citylife-nfz3b` had no bundle at their current version (0.1.3); their WGS84 bundles at 0.1.2 were on disk. |
| `built.json` | The two bundles built for this row (signed with the lab-dev key, issue time pinned, rebuilt byte for byte), with their hashes. |
| `census_after.json` | The same folder plus those two bundles: no problem; every task's `refly_from` is the flight-ready WGS84 bundle at its current version. No checkout holds this folder yet: the two bundles were built in the branch's worktree (`bundles/` is gitignored), so the lab checkout still needs the build step below. |

Commands:

    python tools/bundle_census.py <lab>/bundles --label "..." --json census_before.json
    python tools/build_flight_bundles.py follow_car_citylife_nfz3 follow_car_citylife_nfz3b --key <lab>/policies/keys/lab-dev-ed25519.pem --issued-at 2026-11-18T00:00:00Z
    python tools/bundle_census.py <folder with both> --label "..." --json census_after.json --check

The census reads bundles only. It does not say which tasks were flown or when.

Since 18 Nov the flight loader (`guardrail/bundle.py` `load_for_flight`)
refuses the 13 legacy metre bundles at load, naming the flight-ready WGS84
bundle beside it, unless the load is a declared read of a recorded flight
(`VLAGUARD_LEGACY_READ=1`); before, it flew them and a spawned zone was
refused mid-flight. The 4 point-free ones (follow-car-citylife 0.1.0 and
0.1.1, follow-car 0.1.1, follow-pedestrian 0.1.1) carry no projected
coordinate and still fly, but no zone can be written into a policy with no
frame (every spawn on them is refused in flight); their current versions
(0.1.3) state an origin and are on disk.

Rig steps (not done here; in this order):

1. In the lab checkout, build the two bundles and check them against
   `built.json` (sha256 `64153e45...` for nfz3, `7c458986...` for nfz3b):

       python tools/build_flight_bundles.py follow_car_citylife_nfz3 follow_car_citylife_nfz3b --key policies/keys/lab-dev-ed25519.pem --issued-at 2026-11-18T00:00:00Z
       sha256sum bundles/follow-car-citylife-nfz3-v0.1.3.tar.gz bundles/follow-car-citylife-nfz3b-v0.1.3.tar.gz
       python tools/bundle_census.py bundles --check          # exit 0, no PROBLEM line
       python tests/test_bundle_census.py                     # 4/4, no SKIP

2. Copy them to the AGX Orin (`deploy/orin/sync_follow_controller.ps1`
   copies `bundles/follow-car-citylife*.tar.gz`).

3. Re-fly the follow tasks last flown from a legacy bundle (the 3-4 Nov AGX
   matrix flew `follow-car-citylife-nfz-v0.1.1` and
   `follow-car-citylife-nfz3b-v0.1.1`), with the recorded arguments
   (`console.log` of `agx_follow_01`, `agx_hier_aerovla_20261103` and
   `nfz3b_agx_hier_aerovla_20261104`). `--policy` flies that YAML's signed
   bundle at its current version, so these fly the v0.1.3 WGS84 bundles.
   Start the simulator on the desktop first
   (`deploy/orin/start_sim_for_jetson.ps1`); for the `hier_aerovla` flights,
   the AerialVLA hierarchy server on the board at 127.0.0.1:8790, as on 4 Nov:

       SIM=10.42.0.1 TAG=agx_follow_wgs84_<date> POLICY=policies/follow_car_citylife_nfz.yaml bash deploy/orin/run_follow_controller.sh
       SIM=10.42.0.1 TAG=agx_hier_aerovla_wgs84_<date> POLICY=policies/follow_car_citylife_nfz.yaml EXTRA="--vla-hierarchy 127.0.0.1:8790" bash deploy/orin/run_follow_controller.sh
       SIM=10.42.0.1 TAG=nfz3b_agx_hier_aerovla_wgs84_<date> POLICY=policies/follow_car_citylife_nfz3b.yaml EXTRA="--vla-hierarchy 127.0.0.1:8790" bash deploy/orin/run_follow_controller.sh

   Each run's `metrics.json` `policy_source.path` must name the `-v0.1.3`
   bundle, with `signature: verified` and no `legacy_read` key.
