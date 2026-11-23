"""Build and sign the policy bundles the flight rails fly, into bundles/.

    python tools/build_flight_bundles.py            # build + sign + verify all
    python tools/build_flight_bundles.py --check    # verify what is on disk
    python tools/build_flight_bundles.py corridor_survey wgs84_taipei

WHY

Until 2026-10-12 both SITL rails flew policies/sim_demo_policy.yaml, unsigned,
whenever no --bundle was given, and the perception rail's CityLife policies
existed only as YAML. A run's policy was then "whatever the YAML said that
day": identified by its hash, attested by nobody (audit cards ARCH-29,
WP3-21). The rails now refuse a YAML policy without --dev-yaml
(guardrail.manifest.flight_policy_choice) and fly the signed bundle of their
default policy when it is on disk; this tool puts it there.

WHAT IT DOES, PER POLICY

  1. loads the YAML and refuses it if the version lock gives its id@version
     to different content (the release check `python -m guardrail.bundle`
     runs), or if the Shield would not enforce every rule as written
     (`Policy.flight_problems`): a flight bundle that no rail can fly is not
     written;
  2. writes bundles/<policy_id>-v<version>.tar.gz
     (guardrail.manifest.flight_bundle_path), signed with the lab key the
     repository already uses (policies/keys/lab-dev-ed25519.pem, or
     $VLAGUARD_SIGNING_KEY; its public half is in
     policies/keys/trusted_signers.json). Without the key, or without
     `cryptography`, it stops: it never writes an unsigned flight bundle;
  3. reads the bundle back through the flight loader
     (guardrail.bundle.load_for_flight, signature required) and checks that
     it flies the policy hash the YAML has.

`--check` does step 3 only, for every bundle the list names, and reports a
bundle that is missing, unsigned, untrusted, or no longer the YAML's policy.

Bundles are build products (bundles/*.tar.gz is gitignored), byte-derivable
from the policy and the issue time: `--issued-at` (or SOURCE_DATE_EPOCH)
pins it; by default the policy's own `issued_at`, else now.

The signing key is the lab's bundle key. Since 2026-11-16 it can be certified
by the lab root CA (`python -m guardrail.bundle issue-cert`, run once where the
CA's key is): from then on `load_signer` signs as `lab-ca:<CA>:ed25519:<key>`
and each bundle ships the signer's certificate, so re-running this tool is the
re-sign; until then it signs under the bare `lab-dev` label.

    python tools/build_flight_bundles.py --issue-cert

does both in one step, on the machine that holds the CA's key: it certifies
every key the trust store pins (this machine's and, from its public key
alone, the AGX Orin's), then builds, signs and verifies the bundles under the
new identity. On a machine without the CA's key (the Orin), sync the trust
store and run the tool without the flag. Which authority
signs the delivered bundles (the lab or ITRI) is a question put to the PI
(docs/PI-DECISIONS-2026-10-06.md, Q4.8) and not answered; swapping the
authority changes the trust store, not this tool: `python -m guardrail.bundle
register-cert` pins the leaf the other CA issued, and a key certified by two
authorities is re-signed here with `--authority <name>` (or
$VLAGUARD_SIGNING_AUTHORITY), never under one picked by file order.

THE CHANGELOG IS A DIFF AGAINST THE PREVIOUS VERSION (2026-11-21, X-10b)

The grant's Policy DSL page ("Versioning and signing") has every published
bundle carry a changelog that is a "Human-readable diff vs the previous
version". guardrail.bundle.write_bundle can record one since 2026-10-29
(`previous=`: the manifest's `changelog_diff`, signed with the rest), but
until 2026-11-21 this tool never passed it, so every flight bundle carried
the free text "flight bundle of <file>" and no diff. Now step 2 first finds
the policy's previous published version (`previous_version`): the highest
version below this one among the bundles of the same policy_id in the output
folder (named as flight_bundle_path names them; replay packs are not
policies), read with check_bundle and taken only when its signature verifies
under the trust store. An unreadable or untrusted predecessor stops the build
rather than being skipped, since skipping it would write "first version" on
a bundle that has one. `--previous NAME=FILE` (repeatable; NAME is the
policy as listed or its policy_id; FILE a bundle or a policy YAML) names the
predecessor instead, for one that is not in the folder. The bundle then
carries `changelog_diff` and, as its `changelog`, the diff's one-line reading
(guardrail.bundle.diff_text); with no predecessor its changelog says so
("first version: ..."). `--check` reports a bundle as BAD when a predecessor
exists and its manifest has no `changelog_diff`, or a diff against another
version than that one.

The bundles delivered in bundles/ were built before this and are not rebuilt
by the change (which authority signs them is the PI's open question above).
Once it is settled, rebuild them in place, where the signing key is, so each
is diffed against its predecessor already in bundles/, and check them:

    python tools/build_flight_bundles.py
    python tools/build_flight_bundles.py --check
"""
from __future__ import annotations

import argparse
import glob
import sys
import tarfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail import bundle as B                                     # noqa: E402
from guardrail.manifest import flight_bundle_path                     # noqa: E402
from guardrail.models import Policy, load_policy                      # noqa: E402

POLICIES_DIR = ROOT / "policies"
# The policies the rails fly: the SITL rails' default mission policy, the
# stress harness's corridor and geographic policies, the perception rail's two
# CityLife follow policies (which the city rail's launcher flies too), and,
# since the city rail flies signed bundles as well (demo/follow_vlm.py
# city_policy_choice, ARCH-29, 2026-10-21), the policies its other launchers
# name: follow_vlm's default follow_car, follow_car_nfz (run_follow_vlm.ps1)
# and follow_pedestrian (run_citylife_follow.ps1's person mission,
# run_retarget_demo.ps1); and the AerialVLA pilot's CityLife policy
# (demo/vla_fly.py flies its signed bundle by default, as the SITL rails do).
FLIGHT_POLICIES = ("sim_demo_policy", "corridor_survey", "follow_car_citylife",
                   "follow_car_citylife_nfz", "wgs84_taipei", "follow_car",
                   "follow_car_nfz", "follow_pedestrian", "vla_goto_citylife",
                   # The distance_envelope family on the SITL rails (WP1-07,
                   # 2026-11-17; experiments/profiles/distance_envelope.yaml).
                   "distance_envelope_sitl",
                   # The three-zone CityLife follow tasks (28 Oct - 4 Nov,
                   # flown with --bundle up to v0.1.1, the legacy metre form):
                   # their WGS84 bundles at the current version, which a
                   # re-flight takes (WP1-14, 2026-11-18; tools/bundle_census.py).
                   "follow_car_citylife_nfz3", "follow_car_citylife_nfz3b")


def _policy_file(name: str) -> Path:
    p = Path(name)
    if p.suffix in (".yaml", ".yml", ".json") and p.is_file():
        return p
    return POLICIES_DIR / f"{name}.yaml"


def _ground_kw(ground: str | None, src: Path) -> dict:
    """{"ground": the ground file `ground` in the frame of policy `src`}, or
    {} without one (X-11: an MSL rule is flyable only over a ground, so its
    bundle is built and verified over the ground it will fly over)."""
    if not ground:
        return {}
    from guardrail.terrain import ground_for_flight
    return {"ground": ground_for_flight(ground, yaml_path=src, root=ROOT)}


# The changelog of a bundle with no previous version (X-10b): said in words,
# not left as free text that reads either way.
FIRST_VERSION = "first version"


def _verified_previous(path: Path, trust_store=None) -> Policy:
    """The policy in bundle `path`, which must open (check_bundle) and carry a
    signature that verifies under `trust_store`. ValueError otherwise: a diff
    against a predecessor nobody attests is not a changelog."""
    try:
        chk = B.check_bundle(path, trust_store)
    except (ValueError, OSError, tarfile.TarError) as exc:
        raise ValueError(f"{path.name} cannot be read: {exc}") from exc
    if not chk.verified:
        raise ValueError(f"{path.name}: signature {chk.signature}: {chk.detail}")
    return chk.flight_policy or chk.policy


def previous_version(name: str, pol: Policy, out_dir: Path | None = None,
                     trust_store=None, previous: dict | None = None
                     ) -> tuple[str, Policy] | None:
    """(file name, Policy) of the version policy `pol` replaces, or None when
    it has none: the grant's changelog is a "diff vs the previous version"
    (X-10b, 2026-11-21).

    `previous` maps a policy - by `name` as listed, the file's stem, or its
    policy_id - to that version's bundle or policy file, and wins. Otherwise
    the previous version is the highest version below `pol.version` among
    the bundles <policy_id>-v<version>.tar.gz in the output folder
    (flight_bundle_path's names; an equal or higher version is no
    predecessor, and the bundle's own manifest must agree with its name).
    ValueError when that bundle (or a mapped one) does not open or is not
    signed by a trusted key, when a mapped version of the same policy_id is
    not lower, and for a version that is not plain semver."""
    want = B._semver_key(pol.version)
    if not want:
        raise ValueError(f"{pol.policy_id}: version {pol.version!r} is not plain "
                         f"semver, so no previous version can be ordered below it")
    mapping = previous or {}
    for key in (name, Path(name).stem, pol.policy_id):
        if key in mapping:
            src = Path(mapping[key])
            prev = (_verified_previous(src, trust_store) if src.name.endswith(".tar.gz")
                    else load_policy(src, runtime=False))
            if prev.policy_id == pol.policy_id and not B._semver_key(prev.version) < want:
                raise ValueError(f"--previous {key}={src}: {prev.policy_id} "
                                 f"{prev.version} is not below {pol.version}")
            return src.name, prev
    folder = flight_bundle_path(pol, out_dir).parent
    prefix = f"{pol.policy_id}-v"
    found = []
    for cand in folder.glob(glob.escape(prefix) + "*.tar.gz"):
        if cand.name.endswith(".replay.tar.gz"):
            continue
        key = B._semver_key(cand.name[len(prefix):-len(".tar.gz")])
        if key and key < want:
            found.append((key, cand.name, cand))
    if not found:
        return None
    key, _, cand = max(found)
    prev = _verified_previous(cand, trust_store)
    if (prev.policy_id, B._semver_key(prev.version)) != (pol.policy_id, key):
        raise ValueError(f"{cand.name} holds {prev.policy_id} {prev.version}, not "
                         f"the version its name says")
    return cand.name, prev


def check_one(name: str, out_dir: Path | None = None,
              trust_store=None, ground: str | None = None,
              previous: dict | None = None) -> dict:
    """Is the flight bundle of policy `name` on disk, signed by a trusted key,
    flyable, and still the YAML's policy? `{"policy", "bundle", "ok", "why"}`.
    `ground`: the ground file an MSL policy flies over (--ground).

    Since 2026-11-21 (X-10b) also: is its changelog the diff against the
    previous version (previous_version, `previous` as there)? A bundle with
    a predecessor whose manifest has no `changelog_diff`, or a diff against
    another version, is not ok. The row's `previous_version` names the
    predecessor (None: a first version) and `changelog` quotes the manifest."""
    src = _policy_file(name)
    gkw = _ground_kw(ground, src)
    pol = load_policy(src, runtime=False)
    out = flight_bundle_path(pol, out_dir)
    shown = src.resolve()
    row = {"policy": (shown.relative_to(ROOT).as_posix()
                      if shown.is_relative_to(ROOT) else shown.as_posix()),
           "policy_id": pol.policy_id, "version": pol.version,
           "policy_hash": pol.policy_hash, "bundle": str(out), "ok": False}
    if not out.is_file():
        row["why"] = "no bundle on disk"
        return row
    try:
        flown, source = B.load_for_flight(out, trust_store=trust_store, **gkw)
    except (ValueError, OSError) as exc:
        row["why"] = f"refused by the flight loader: {exc}"
        return row
    if flown.policy_hash != pol.policy_hash:
        row["why"] = (f"the bundle flies {flown.policy_hash}, the YAML is now "
                      f"{pol.policy_hash}: rebuild it")
        return row
    try:
        prev = previous_version(name, pol, out_dir, trust_store, previous)
    except ValueError as exc:
        row["why"] = f"its previous version cannot be judged: {exc}"
        return row
    manifest = B.check_bundle(out, trust_store).manifest
    row.update(previous_version=None if prev is None else prev[0],
               changelog=manifest.get("changelog"))
    if prev is not None:
        if "changelog_diff" not in manifest:
            row["why"] = (f"its manifest has no changelog_diff, but {prev[0]} "
                          f"({prev[1].version}) came before it: rebuild it")
            return row
        against = manifest["changelog_diff"].get("against") or {}
        if against.get("policy_hash") != prev[1].policy_hash:
            row["why"] = (f"its changelog_diff is against {against.get('version')} "
                          f"({str(against.get('policy_hash'))[:19]}...), the previous "
                          f"version is {prev[0]} ({prev[1].version}, "
                          f"{prev[1].policy_hash[:19]}...): rebuild it")
            return row
    row.update(ok=True, signature=source.get("signature"),
               signed_by=source.get("signed_by"),
               issued_at=source.get("issued_at"), why="")
    return row


def build_one(name: str, signer: B.Signer, out_dir: Path | None = None, *,
              issued_at=None, trust_store=None, ground: str | None = None,
              previous: dict | None = None) -> dict:
    """Write, sign and verify the flight bundle of policy `name`. Raises
    ValueError for a policy that may not be released or flown. `ground`:
    the ground file an MSL policy flies over (--ground; X-11).

    The changelog (X-10b, 2026-11-21): the diff against the previous version
    (previous_version; `previous` as there), recorded as the manifest's
    `changelog_diff` with its one-line reading as `changelog`; with no
    previous version, a changelog that begins FIRST_VERSION. Until then
    every bundle said "flight bundle of <file>" and carried no diff."""
    src = _policy_file(name)
    gkw = _ground_kw(ground, src)
    pol = load_policy(src, runtime=False)
    why = B._release_check(pol, str(src))     # the CLI's own release check
    if why:
        raise ValueError(why)
    gaps = pol.flight_problems(**gkw)
    if gaps:
        raise ValueError(f"{src.name} is not flyable as written, so no flight "
                         f"bundle is written: " + "; ".join(gaps))
    out = flight_bundle_path(pol, out_dir)
    prev = previous_version(name, pol, out_dir, trust_store, previous)
    if prev is None:
        changelog = (f"{FIRST_VERSION}: no earlier published version of "
                     f"{pol.policy_id} (flight bundle of {src.name})")
        B.write_bundle(pol, out, changelog, issued_at=issued_at, signer=signer)
    else:
        B.write_bundle(pol, out, B.diff_text(B.policy_diff(prev[1], pol)),
                       issued_at=issued_at, signer=signer, previous=prev[1])
    row = check_one(name, out_dir, trust_store, ground=ground, previous=previous)
    if not row["ok"]:
        raise ValueError(f"{out.name} was written but does not verify: "
                         f"{row['why']}")
    return row


def build(names=FLIGHT_POLICIES, out_dir: Path | None = None, *,
          signer: B.Signer | None = None, issued_at=None,
          trust_store=None, ground: str | None = None,
          previous: dict | None = None) -> list[dict]:
    """Build every bundle in `names`; `signer` defaults to the lab key
    (guardrail.bundle.load_signer), and its absence stops the build.
    `previous`: explicit previous versions (previous_version; X-10b)."""
    if signer is None:
        signer = B.load_signer(trust_store=trust_store)
    if signer.authority == "unregistered":
        raise ValueError(f"the signing key {signer.key_id} is not in the trust "
                         f"store, so every bundle it signs would be refused; "
                         f"register it (python -m guardrail.bundle keygen) or "
                         f"use the lab key")
    return [build_one(n, signer, out_dir, issued_at=issued_at,
                      trust_store=trust_store, previous=previous,
                      **({} if ground is None else {"ground": ground})) for n in names]


def _previous_map(items: list[str], parser) -> dict[str, str]:
    """--previous NAME=FILE, repeated, as {NAME: FILE}."""
    out = {}
    for item in items:
        name, sep, path = item.partition("=")
        if not sep or not name or not path:
            parser.error(f"--previous takes NAME=FILE, not {item!r}")
        out[name] = path
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("policies", nargs="*", default=list(FLIGHT_POLICIES),
                    help="policy names under policies/ (or YAML paths); "
                         "default: " + ", ".join(FLIGHT_POLICIES))
    ap.add_argument("--check", action="store_true",
                    help="verify the bundles on disk; write nothing")
    ap.add_argument("--ground", default=None, metavar="FILE",
                    help="the ground the listed policies fly over (guardrail/terrain.py "
                         "load_ground; X-11): a policy with an MSL rule is flyable, so "
                         "bundled, only over one. Give it only with such policies "
                         "(e.g. experiments/policies/msl_dem_sitl.yaml)")
    ap.add_argument("--out-dir", default=None,
                    help="default bundles/ (gitignored)")
    ap.add_argument("--previous", action="append", default=[], metavar="NAME=FILE",
                    help="the version policy NAME (as listed, or its policy_id) "
                         "replaces: a bundle or a policy YAML. Default: the "
                         "highest lower version of its policy_id in the output "
                         "folder; the changelog is the diff against it (X-10b). "
                         "Repeatable")
    ap.add_argument("--key", default=None,
                    help="signing key (default: the lab key, or "
                         "$VLAGUARD_SIGNING_KEY)")
    ap.add_argument("--issued-at", default=None,
                    help="ISO 8601; default the policy's own issued_at, else "
                         "SOURCE_DATE_EPOCH, else now")
    ap.add_argument("--issue-cert", action="store_true",
                    help="first put every key the trust store pins under the "
                         "lab root CA (python -m guardrail.bundle issue-cert; "
                         "needs the CA's key, so the desktop), then build and "
                         "sign under the new identity")
    ap.add_argument("--pki", default=None,
                    help="with --issue-cert: the CA's folder (default: the lab "
                         "PKI folder of guardrail.harness_service)")
    ap.add_argument("--authority", default=None,
                    help="sign under this listed authority's certificate when "
                         "the key is certified by more than one (default "
                         "$VLAGUARD_SIGNING_AUTHORITY; with neither, such a key "
                         "is refused rather than one picked by file order)")
    args = ap.parse_args(argv)
    out_dir = Path(args.out_dir) if args.out_dir else None
    previous = _previous_map(args.previous, ap)
    if args.pki and not args.issue_cert:
        ap.error("--pki is the CA folder of --issue-cert")
    if args.issue_cert:
        if args.check:
            ap.error("--issue-cert issues and signs; --check writes nothing")
        # The same step as the command of its own; a refusal there (no CA key
        # here, another CA under the listed name) stops before anything is
        # signed.
        rc = B._main(["issue-cert"] + (["--pki", args.pki] if args.pki else []))
        if rc:
            return rc

    if args.check:
        rows = [check_one(n, out_dir, ground=args.ground, previous=previous)
                for n in args.policies]
        for r in rows:
            print(f"{'OK     ' if r['ok'] else 'MISSING' if 'no bundle' in r['why'] else 'BAD    '}"
                  f" {Path(r['bundle']).name:44s} {r['policy']}"
                  + (f"  ({r['why']})" if r["why"] else
                     f"  {r['signature']} by {r['signed_by']}, "
                     + (f"diff against {r['previous_version']}"
                        if r["previous_version"] else FIRST_VERSION)))
        return 0 if all(r["ok"] for r in rows) else 1

    try:
        signer = B.load_signer(args.key, authority=args.authority)
        rows = build(args.policies, out_dir, signer=signer,
                     issued_at=args.issued_at, previous=previous,
                     **({} if args.ground is None else {"ground": args.ground}))
    except (ValueError, RuntimeError, OSError) as exc:
        print(f"REFUSED  {type(exc).__name__}: {exc}")
        return 2
    for r in rows:
        print(f"SIGNED   {Path(r['bundle']).name:44s} {r['policy']}  "
              f"{r['policy_hash'][:23]}...  by {r['signed_by']}, "
              + (f"diff against {r['previous_version']}"
                 if r["previous_version"] else FIRST_VERSION))
    print(f"{len(rows)} flight bundle(s) in {out_dir or B.ROOT / 'bundles'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
