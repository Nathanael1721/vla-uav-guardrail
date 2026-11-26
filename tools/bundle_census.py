"""List every bundle under bundles/ with the form its policy's points are stored
in, and name the WGS84 bundle each task is to be flown from whose bundle the
flight loader refuses (legacy metre-only) or that can take no zone in flight
(point-free with no frame origin).

    python tools/bundle_census.py                      # bundles/ of this checkout
    python tools/bundle_census.py D:/lab/bundles --json docs/data/census.json
    python tools/bundle_census.py --check              # exit 1 on a gap

WHY (WP1-14, 2026-11-18)

The grant's Policy DSL page: "WGS84 latitude / longitude is canonical ... the
DSL itself never carries projected coordinates". Every policies/*.yaml is
WGS84 since 2026-11-03 and every writer converts metre input at ingest (or
refuses it where the policy has no frame origin; models.NoFrameOrigin), but
the signed bundles built before that date are still on disk: a policy whose
points are local metres (Policy.point_form, the legacy READ form) still loads,
for the runs flown under it and their replays, but since 2026-11-18 the
flight loader refuses it unless the load is a declared read
(bundle.load_for_flight legacy_read). A point-free policy with no frame
origin still flies, but every zone event on it is refused.
Which bundles are which was
known only from a one-off listing (13 of the 41 lab bundles on 18 Nov). This
tool makes it a check anyone can run on any bundles/ folder.

WHAT IT REPORTS, PER BUNDLE

  file, kind ("policy" or "replay": a *.replay.tar.gz flight-replay pack is
  not a policy bundle and is listed, not judged), policy_id, version,
  policy_hash, point_form (models.POINTS_*), frame ("stated" origin,
  "derived" from the geometry, or "none"), legacy (point_form is one of the
  legacy metre forms), flight_ready (bundle.flight_ready: not legacy and a
  WGS84 frame origin, so it flies with every grant event available),
  signature status and signer, hash_form, issued_at; or
  `error` for a bundle check_bundle refuses.

PER TASK (`legacy_tasks`: a policy_id with at least one bundle that is not
flight_ready)

  the legacy metre versions on disk (`legacy_bundles`), every version that is
  not flight_ready (`not_flight_ready`: the legacy ones, refused for flight,
  and the point-free ones with no frame origin, on which every spawned zone
  is refused), the
  newest flight_ready bundle of the same id above them (`successor`), the
  version policies/ (or experiments/policies/) holds now (`current_version`)
  and whether that version's bundle is on disk and flight_ready
  (`current_bundle`). `refly_from` is the bundle a re-flight of the task
  takes: the current version's when it is on disk, else the newest
  successor.

--check exits 1 when a bundle cannot be read, or a task has no flight_ready
bundle at its current version on disk (build it with
tools/build_flight_bundles.py <policy name>). It never writes a bundle.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from guardrail.bundle import check_bundle, flight_ready               # noqa: E402
from guardrail.models import (LEGACY_POINT_FORMS,                     # noqa: E402
                              load_legacy_metre_policy)

REPLAY_SUFFIX = ".replay.tar.gz"
POLICY_DIRS = (ROOT / "policies", ROOT / "experiments" / "policies")


def _version_key(v: str) -> tuple:
    out = []
    for part in str(v).split("."):
        out.append(int(part) if part.isdigit() else -1)
    return tuple(out)


def _frame(policy) -> str:
    if policy.origin is not None:
        return "stated"
    return "derived" if policy.frame_origin is not None else "none"


def bundle_row(path: Path) -> dict:
    """One bundle's census row (see the module docstring)."""
    row = {"file": path.name}
    if path.name.endswith(REPLAY_SUFFIX):
        row["kind"] = "replay"
        return row
    row["kind"] = "policy"
    try:
        chk = check_bundle(path)
    except Exception as exc:                                  # noqa: BLE001
        row["error"] = f"{type(exc).__name__}: {exc}"
        return row
    pol = chk.policy
    form = pol.point_form
    fo = pol.frame_origin
    row.update({
        "policy_id": pol.policy_id, "version": pol.version,
        "policy_hash": pol.policy_hash, "point_form": form,
        "frame": _frame(pol),
        "frame_origin": None if fo is None else [fo[0], fo[1]],
        "legacy": form in LEGACY_POINT_FORMS,
        # x/y beside lat/lon in the archived ir.json (the dump of a bundle
        # hashed before 2026-10-06): refused for flight by load_for_flight
        # since 2026-11-26 (BundleCheck.projected_points, WP1-14).
        "projected_points": chk.projected_points,
        "flight_ready": flight_ready(pol) and not chk.projected_points,
        "signature": chk.signature, "signed_by": chk.signed_by,
        "hash_form": chk.hash_form,
        "issued_at": chk.manifest.get("issued_at"),
    })
    return row


def current_versions(dirs=POLICY_DIRS) -> dict[str, dict]:
    """policy_id -> {"version", "file", "point_form"} of the policy files the
    repository holds now (policies/ and experiments/policies/)."""
    out: dict[str, dict] = {}
    for d in dirs:
        for f in sorted(Path(d).glob("*.yaml")):
            try:
                # A census READS every form, the legacy metre one included
                # (a declared re-read, WP1-14): it reports point_form.
                pol = load_legacy_metre_policy(f, runtime=False)
            except Exception:                                 # noqa: BLE001
                continue
            shown = f.resolve()
            name = (shown.relative_to(ROOT).as_posix() if shown.is_relative_to(ROOT)
                    else shown.as_posix())
            prev = out.get(pol.policy_id)
            if prev is None or _version_key(pol.version) > _version_key(prev["version"]):
                out[pol.policy_id] = {"version": pol.version, "file": name,
                                      "point_form": pol.point_form}
    return out


def census(bundles_dir: Path, current: dict[str, dict] | None = None) -> dict:
    """The census of one bundles/ folder: {"dir", "bundles", "legacy_tasks",
    "summary", "problems"}."""
    bundles_dir = Path(bundles_dir)
    rows = [bundle_row(p) for p in sorted(bundles_dir.glob("*.tar.gz"))]
    current = current_versions() if current is None else current
    policy_rows = [r for r in rows if r["kind"] == "policy" and "error" not in r]
    by_id: dict[str, list[dict]] = {}
    for r in policy_rows:
        by_id.setdefault(r["policy_id"], []).append(r)

    tasks, problems = [], []
    for r in rows:
        if "error" in r:
            problems.append(f"{r['file']}: unreadable ({r['error']})")
    for pid in sorted(by_id):
        mine = sorted(by_id[pid], key=lambda r: _version_key(r["version"]))
        legacy = [r for r in mine if r["legacy"]]
        unready = [r for r in mine if not r["flight_ready"]]
        if not unready:
            continue
        newest_unready = _version_key(unready[-1]["version"])
        wgs = [r for r in mine if r["flight_ready"]
               and _version_key(r["version"]) > newest_unready]
        cur = current.get(pid)
        cur_row = None
        if cur is not None:
            cur_row = next((r for r in mine if r["version"] == cur["version"]), None)
        cur_ok = cur_row is not None and cur_row["flight_ready"]
        succ = wgs[-1] if wgs else None
        refly = cur_row if cur_ok else succ
        task = {
            "policy_id": pid,
            "legacy_bundles": [r["file"] for r in legacy],
            "not_flight_ready": [r["file"] for r in unready],
            "successor": None if succ is None else succ["file"],
            "current_version": None if cur is None else cur["version"],
            "current_file": None if cur is None else cur["file"],
            "current_point_form": None if cur is None else cur["point_form"],
            "current_bundle": None if cur_row is None else cur_row["file"],
            "current_bundle_on_disk": cur_row is not None,
            "refly_from": None if refly is None else refly["file"],
        }
        tasks.append(task)
        if cur is None:
            problems.append(f"{pid}: bundle(s) not flight-ready "
                            f"{task['not_flight_ready']} and no policy file holds the "
                            f"id any more")
        elif not cur_ok:
            problems.append(
                f"{pid}: bundle(s) not flight-ready "
                f"{task['not_flight_ready']}; the current version {cur['version']} "
                f"({cur['file']}) has no WGS84 bundle in {bundles_dir}: build it "
                f"(python tools/build_flight_bundles.py {Path(cur['file']).stem})")
    n_policy = len([r for r in rows if r["kind"] == "policy"])
    forms: dict[str, int] = {}
    for r in policy_rows:
        forms[r["point_form"]] = forms.get(r["point_form"], 0) + 1
    summary = {
        "bundles": len(rows), "policy_bundles": n_policy,
        "replay_bundles": len([r for r in rows if r["kind"] == "replay"]),
        "unreadable": len([r for r in rows if "error" in r]),
        "legacy_metre_bundles": len([r for r in policy_rows if r["legacy"]]),
        "point_forms": dict(sorted(forms.items())),
        "frameless_bundles": len([r for r in policy_rows if r["frame"] == "none"]),
        "not_flight_ready_bundles": len([r for r in policy_rows if not r["flight_ready"]]),
        "legacy_tasks": len(tasks),
    }
    return {"dir": str(bundles_dir), "bundles": rows, "legacy_tasks": tasks,
            "summary": summary, "problems": problems}


def _print(c: dict) -> None:
    for r in c["bundles"]:
        if r["kind"] == "replay":
            print(f"  replay  {r['file']}")
        elif "error" in r:
            print(f"  ERROR   {r['file']}: {r['error']}")
        else:
            tag = ("LEGACY" if r["legacy"] else
                   "XYSIDE" if r.get("projected_points") else
                   "NOFRAM" if not r["flight_ready"] else "ok    ")
            print(f"  {tag}  {r['file']:44s} {r['point_form']:38s} frame={r['frame']:7s} "
                  f"{r['signature']}")
    s = c["summary"]
    print(f"{s['policy_bundles']} policy bundles ({s['legacy_metre_bundles']} legacy "
          f"metre-only, {s['frameless_bundles']} without a frame origin), "
          f"{s['replay_bundles']} replay packs, {s['unreadable']} unreadable")
    for t in c["legacy_tasks"]:
        print(f"  re-fly {t['policy_id']}: from {t['refly_from']} "
              f"(not flight-ready: {', '.join(t['not_flight_ready'])})")
    for p in c["problems"]:
        print(f"PROBLEM  {p}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("dir", nargs="?", default=str(ROOT / "bundles"),
                    help="the bundles folder (default: bundles/ of this checkout)")
    ap.add_argument("--json", default=None, help="write the census here as JSON")
    ap.add_argument("--label", default=None,
                    help="what the JSON records as `dir` (default: the folder's path), "
                         "for evidence that must not carry a machine's path")
    ap.add_argument("--check", action="store_true",
                    help="exit 1 on an unreadable bundle or a task with no "
                         "flight-ready WGS84 bundle at its current version")
    args = ap.parse_args(argv)
    c = census(Path(args.dir))
    if args.label:
        c["dir"] = args.label
        c["problems"] = [p.replace(str(Path(args.dir)), args.label) for p in c["problems"]]
    _print(c)
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        with open(args.json, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(json.dumps(c, indent=2, sort_keys=True) + "\n")
    return 1 if (args.check and c["problems"]) else 0


if __name__ == "__main__":
    sys.exit(main())
