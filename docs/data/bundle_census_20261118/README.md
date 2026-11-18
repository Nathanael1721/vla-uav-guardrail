# Bundle census, 18 Nov 2026 (WP1-14)

What `tools/bundle_census.py` says about the lab checkout's `bundles/`: every
bundle, the form its points are stored in, and the WGS84 bundle each legacy
metre-only task is to be flown from. Checked by `tests/test_bundle_census.py`.

| File | What it is |
|---|---|
| `census_before.json` | The lab checkout's `bundles/` on 18 Nov 2026 (42 policy bundles, 8 replay packs). 13 policy bundles store their points in local metres (the legacy read form), over 7 tasks. Two problems: `follow-car-citylife-nfz3` and `follow-car-citylife-nfz3b` had no bundle at their current version (0.1.3). |
| `built.json` | The two bundles built for this row (signed with the lab-dev key, issue time pinned, rebuilt byte for byte), with their hashes. |
| `census_after.json` | The same folder plus those two bundles: no problem; every legacy task's `refly_from` is the WGS84 bundle at its current version. |

Commands:

    python tools/bundle_census.py <lab>/bundles --label "..." --json census_before.json
    python tools/build_flight_bundles.py follow_car_citylife_nfz3 follow_car_citylife_nfz3b --key <lab>/policies/keys/lab-dev-ed25519.pem --issued-at 2026-11-18T00:00:00Z
    python tools/bundle_census.py <folder with both> --label "..." --json census_after.json --check

The census reads bundles only. It does not say which tasks were flown or when:
the legacy bundles were flown up to 4 Nov 2026, and a re-flight from
`refly_from` is a rig step, not done here.

Of the 17 bundles without a frame origin, 13 are the legacy metre ones and 4
carry no point at all (follow-car-citylife 0.1.0 and 0.1.1, follow-car 0.1.1,
follow-pedestrian 0.1.1). These 4 carry no projected coordinate, but since 18
Nov an event with points (a spawned zone) is refused on them, because there
is no frame to write it in. Their current versions state an origin.
