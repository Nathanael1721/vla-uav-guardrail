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

The signing key is the lab's DEVELOPMENT key. Which authority signs the
delivered bundles (the lab or ITRI) is a question put to the PI
(docs/PI-DECISIONS-2026-10-06.md, Q4.8) and not answered; swapping the key
changes the trust store, not this tool.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail import bundle as B                                     # noqa: E402
from guardrail.manifest import flight_bundle_path                     # noqa: E402
from guardrail.models import load_policy                              # noqa: E402

POLICIES_DIR = ROOT / "policies"
# The policies the rails fly: the SITL rails' default mission policy, the
# stress harness's corridor and geographic policies, and the perception
# rail's two CityLife follow policies.
FLIGHT_POLICIES = ("sim_demo_policy", "corridor_survey", "follow_car_citylife",
                   "follow_car_citylife_nfz", "wgs84_taipei")


def _policy_file(name: str) -> Path:
    p = Path(name)
    if p.suffix in (".yaml", ".yml", ".json") and p.is_file():
        return p
    return POLICIES_DIR / f"{name}.yaml"


def check_one(name: str, out_dir: Path | None = None,
              trust_store=None) -> dict:
    """Is the flight bundle of policy `name` on disk, signed by a trusted key,
    flyable, and still the YAML's policy? `{"policy", "bundle", "ok", "why"}`."""
    src = _policy_file(name)
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
        flown, source = B.load_for_flight(out, trust_store=trust_store)
    except (ValueError, OSError) as exc:
        row["why"] = f"refused by the flight loader: {exc}"
        return row
    if flown.policy_hash != pol.policy_hash:
        row["why"] = (f"the bundle flies {flown.policy_hash}, the YAML is now "
                      f"{pol.policy_hash}: rebuild it")
        return row
    row.update(ok=True, signature=source.get("signature"),
               signed_by=source.get("signed_by"),
               issued_at=source.get("issued_at"), why="")
    return row


def build_one(name: str, signer: B.Signer, out_dir: Path | None = None, *,
              issued_at=None, trust_store=None) -> dict:
    """Write, sign and verify the flight bundle of policy `name`. Raises
    ValueError for a policy that may not be released or flown."""
    src = _policy_file(name)
    pol = load_policy(src, runtime=False)
    why = B._release_check(pol, str(src))     # the CLI's own release check
    if why:
        raise ValueError(why)
    gaps = pol.flight_problems()
    if gaps:
        raise ValueError(f"{src.name} is not flyable as written, so no flight "
                         f"bundle is written: " + "; ".join(gaps))
    out = flight_bundle_path(pol, out_dir)
    B.write_bundle(pol, out, f"flight bundle of {src.name}",
                   issued_at=issued_at, signer=signer)
    row = check_one(name, out_dir, trust_store)
    if not row["ok"]:
        raise ValueError(f"{out.name} was written but does not verify: "
                         f"{row['why']}")
    return row


def build(names=FLIGHT_POLICIES, out_dir: Path | None = None, *,
          signer: B.Signer | None = None, issued_at=None,
          trust_store=None) -> list[dict]:
    """Build every bundle in `names`; `signer` defaults to the lab key
    (guardrail.bundle.load_signer), and its absence stops the build."""
    if signer is None:
        signer = B.load_signer(trust_store=trust_store)
    if signer.authority == "unregistered":
        raise ValueError(f"the signing key {signer.key_id} is not in the trust "
                         f"store, so every bundle it signs would be refused; "
                         f"register it (python -m guardrail.bundle keygen) or "
                         f"use the lab key")
    return [build_one(n, signer, out_dir, issued_at=issued_at,
                      trust_store=trust_store) for n in names]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("policies", nargs="*", default=list(FLIGHT_POLICIES),
                    help="policy names under policies/ (or YAML paths); "
                         "default: " + ", ".join(FLIGHT_POLICIES))
    ap.add_argument("--check", action="store_true",
                    help="verify the bundles on disk; write nothing")
    ap.add_argument("--out-dir", default=None,
                    help="default bundles/ (gitignored)")
    ap.add_argument("--key", default=None,
                    help="signing key (default: the lab key, or "
                         "$VLAGUARD_SIGNING_KEY)")
    ap.add_argument("--issued-at", default=None,
                    help="ISO 8601; default the policy's own issued_at, else "
                         "SOURCE_DATE_EPOCH, else now")
    args = ap.parse_args(argv)
    out_dir = Path(args.out_dir) if args.out_dir else None

    if args.check:
        rows = [check_one(n, out_dir) for n in args.policies]
        for r in rows:
            print(f"{'OK     ' if r['ok'] else 'MISSING' if 'no bundle' in r['why'] else 'BAD    '}"
                  f" {Path(r['bundle']).name:44s} {r['policy']}"
                  + (f"  ({r['why']})" if r["why"] else
                     f"  {r['signature']} by {r['signed_by']}"))
        return 0 if all(r["ok"] for r in rows) else 1

    try:
        signer = B.load_signer(args.key)
        rows = build(args.policies, out_dir, signer=signer,
                     issued_at=args.issued_at)
    except (ValueError, RuntimeError, OSError) as exc:
        print(f"REFUSED  {type(exc).__name__}: {exc}")
        return 2
    for r in rows:
        print(f"SIGNED   {Path(r['bundle']).name:44s} {r['policy']}  "
              f"{r['policy_hash'][:23]}...  by {r['signed_by']}")
    print(f"{len(rows)} flight bundle(s) in {out_dir or B.ROOT / 'bundles'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
