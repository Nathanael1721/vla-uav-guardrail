"""Re-fly a headless episode from its bundle and compare the flight logs.

    python tools/rerun_episode.py demo/out/harness/ep-0123/episode-...--seed1001
    python tools/rerun_episode.py <bundle> --seed 1002          # the null: must differ
    python tools/rerun_episode.py <bundle> --write-log rerun.jsonl --json

WHAT THE GRANT ASKS FOR

Stress Testing, "Episode bundle": "Every episode produces a self-contained
bundle that can be replayed bit-for-bit", and "Determinism: every episode
emits a six-field manifest". guardrail/replay.py re-derives a flown episode's
KPIs from its log; it cannot re-FLY it. For the headless harness it can be
re-flown, and this tool does it: it rebuilds the episode from its bundle,
flies it again through experiments/sweep_scenarios.py, serialises the rows
exactly as `write_bundle` writes flight_log.jsonl, and compares SHA-256.

HOW THE EPISODE IS REBUILT (first that applies)

  1. manifest_extras.json carrying the self-contained keys
     (`extras_schema`, the expanded `scenario_spec` with its events, the
     `library_defaults`, `random_seed`, the `arm`, the `policy` as injected
     and its `generations`; experiments/sweep_scenarios.py episode_extras,
     2026-10-12): flown by that module's `rerun_from_extras`.
  2. Otherwise from the bundle's own records through the sweep's public
     functions: the family from the server's library (or the client's
     `scenario_yaml` in harness_request.json), the cell by its id under the
     family's own expansion or any profile's overrides, the seed from the
     manifest, the arm record from kpi.json (`arm_record`, which run_episode
     only records), and the events a gRPC client queued
     (`harness.events_injected`), appended after the cell's own as the
     harness flies them.

WHAT IS COMPARED

  * flight_log.jsonl, byte for byte (SHA-256) once both sides have LF line
    endings (write_bundle writes the platform's), when the bundle kept it;
  * else the KPI table's headline figures and the final position
    (harness_events.jsonl `episode_end`), which is weaker, and said so;
  * the policy hash at t = 0 against the manifest's, always: a policy file
    edited since the flight makes the re-run another experiment.

`--seed N` re-flies with another seed: the null. It must DIFFER, or the seed
is not what makes the episode reproducible; the report says whether the run
consumed the seed at all (`seed_consumed`), because a scenario that never
draws from it flies the same under every seed. The code revision the bundle
was flown at is printed beside today's: a mismatch on another revision may be
the code, not the bundle.

Exit codes: 0 the re-run is identical, 1 it differs, 2 the episode could not
be rebuilt or compared (a missing key, an unknown scenario or cell, a policy
file that is gone or moved, a scenario the runner refuses), 3 the tool itself
failed on something it did not expect (the traceback is printed). Only 1
means "the re-run flew differently".
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# Headline fields compared when the bundle kept no flight log.
KPI_FIELDS = ("ticks", "p0_escapes", "p0_violation_ticks", "repair_count",
              "mean_repair_magnitude_mps", "mean_time_to_safe_s", "outcome",
              "mission_success")


class RerunError(Exception):
    """The episode cannot be rebuilt from this bundle."""


def sweep():
    """experiments/sweep_scenarios.py, loaded by path (it is not a package),
    shared with any copy already imported from the same file."""
    path = ROOT / "experiments" / "sweep_scenarios.py"
    mod = sys.modules.get("sweep_scenarios")
    if mod is not None and Path(getattr(mod, "__file__", "")).resolve() == path.resolve():
        return mod
    spec = importlib.util.spec_from_file_location("sweep_scenarios", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["sweep_scenarios"] = mod
    spec.loader.exec_module(mod)
    return mod


def _json(p: Path) -> dict:
    try:
        return json.loads(p.read_text(encoding="utf-8")) if p.is_file() else {}
    except (OSError, json.JSONDecodeError):
        return {}


def log_bytes(rows: list[dict]) -> bytes:
    """flight_log.jsonl as experiments/sweep_scenarios.py write_bundle writes it."""
    S = sweep()
    return "".join(json.dumps(S._json_safe(r), allow_nan=False) + "\n"
                   for r in rows).encode("utf-8")


def _sha(data: bytes) -> str:
    return "sha256:" + hashlib.sha256(data).hexdigest()


def _cell_from_library(sid: str, cell_id: str, yaml_text: str | None):
    """(family's cell, library defaults, where it was found)."""
    import yaml
    from guardrail.scenario_spec import Family, Library, load_profile
    S = sweep()
    lib = Library(S.LIB)
    if yaml_text:
        fam = Family(yaml.safe_load(yaml_text), source="rerun")
        where = "harness_request.json scenario_yaml"
    else:
        fam = lib.by_id().get(sid)
        if fam is None:
            raise RerunError(f"no scenario {sid!r} in {Path(S.LIB).name}")
        where = Path(S.LIB).name
    tries = [(None, 1, where)]
    for p in sorted(Path(S.PROFILES).glob("*.yaml")):
        try:
            prof = load_profile(p)
        except Exception:                                        # noqa: BLE001
            continue
        tries.append((prof.sweep_overrides.get(fam.scenario_id),
                      prof.samples_per_family, f"{where} + profile {prof.profile}"))
    for overrides, samples, label in tries:
        try:
            cells = fam.cells(overrides, samples)
        except Exception:                                        # noqa: BLE001
            continue
        hit = [c for c in cells if c.episode_id == cell_id]
        if hit:
            return hit[0], lib.defaults, label
    raise RerunError(f"{fam.scenario_id} has no cell {cell_id!r} under its own "
                     f"expansion or any profile's")


def _refusals(S) -> tuple:
    """What rebuilding an episode raises when the bundle cannot be rebuilt,
    as opposed to a defect of this tool: a key the extras lack, a value that
    fails validation (pydantic's errors are ValueErrors), a policy file that
    is gone (OSError), a scenario the runner does not support."""
    return (ValueError, KeyError, TypeError, OSError,
            getattr(S, "UnsupportedScenario", ValueError))


def rebuild_and_fly(bundle: Path, seed: int | None = None) -> dict:
    """Fly the episode again. Returns {run, spec, seed, rebuilt_from, ...}."""
    S = sweep()
    manifest = _json(bundle / "manifest.json")
    doc = _json(bundle / "kpi.json")
    extras = _json(bundle / "manifest_extras.json")
    request = _json(bundle / "harness_request.json")
    if not manifest:
        raise RerunError(f"{bundle} has no manifest.json")
    want_seed = int(manifest.get("random_seed", 0)) if seed is None else int(seed)

    refused = _refusals(S)
    if extras.get("extras_schema") and hasattr(S, "rerun_from_extras"):
        ex = copy.deepcopy(extras)
        ex["random_seed"] = want_seed
        try:
            run = S.rerun_from_extras(ex, ROOT)
        except refused as e:
            raise RerunError(f"manifest_extras: {type(e).__name__}: {e}") from None
        return {"run": run, "seed": want_seed, "rebuilt_from": "manifest_extras",
                "spec": None, "doc": doc, "manifest": manifest}

    sid, cell_id = doc.get("scenario_id"), doc.get("id_cell") or doc.get("id")
    if not sid or not cell_id:
        raise RerunError("kpi.json names no scenario_id / id_cell, and manifest_extras "
                         "carries no self-contained episode")
    yaml_text = (request.get("request") or {}).get("scenario_yaml")
    events = [x["event"] for x in ((doc.get("harness") or {}).get("events_injected") or [])
              if isinstance(x, dict) and isinstance(x.get("event"), dict)]
    from guardrail.scenario_spec import ScenarioEvent
    try:
        cell, defaults, where = _cell_from_library(sid, cell_id, yaml_text)
        spec = cell.spec.model_copy(update={"events": list(cell.spec.events)
                                            + [ScenarioEvent.model_validate(e)
                                               for e in events]})
    except refused as e:
        raise RerunError(f"the episode's cell cannot be rebuilt: {type(e).__name__}: "
                         f"{e}") from None
    try:
        run = S.run_episode(spec, defaults, want_seed, arm=doc.get("arm_record"))
    except refused as e:
        raise RerunError(f"the runner refused the rebuilt episode: {type(e).__name__}: "
                         f"{e}") from None
    return {"run": run, "seed": want_seed, "spec": spec, "doc": doc, "manifest": manifest,
            "rebuilt_from": f"library ({where})" + (f", {len(events)} queued event(s)"
                                                     if events else "")}


def compare(bundle: Path, seed: int | None = None) -> dict:
    S = sweep()
    got = rebuild_and_fly(bundle, seed)
    run, manifest, doc = got["run"], got["manifest"], got["doc"]
    out = {"bundle": bundle.as_posix(), "rebuilt_from": got["rebuilt_from"],
           "seed_recorded": manifest.get("random_seed"), "seed_flown": got["seed"],
           "seed_consumed": bool(getattr(run, "seed_consumed", False)),
           "code_revision_flown": manifest.get("code_revision"),
           "policy_hash_recorded": manifest.get("policy_hash"),
           "policy_hash_rerun": run.policy_hash_start}
    if manifest.get("policy_hash") and run.policy_hash_start != manifest.get("policy_hash"):
        out.update(verdict="cannot_compare", match=None,
                   why=f"the policy at t = 0 hashes to {run.policy_hash_start}, the "
                       f"bundle flew {manifest.get('policy_hash')}: the policy changed")
        return out
    data = log_bytes(run.rows)
    out["rerun_log_sha256"] = _sha(data)
    out["_rows"] = run.rows
    stored_log = bundle / "flight_log.jsonl"
    if stored_log.is_file():
        # write_bundle writes with Path.write_text, so the file has the line
        # endings of the platform that flew it (CRLF on Windows): compared,
        # and hashed, with LF endings on both sides.
        stored = stored_log.read_bytes().replace(b"\r\n", b"\n")
        out.update(compared="flight_log.jsonl (line endings normalised to LF)",
                   stored_log_sha256=_sha(stored), match=stored == data)
        if not out["match"]:
            a, b = stored.decode("utf-8").splitlines(), data.decode("utf-8").splitlines()
            first = next((i for i, (x, y) in enumerate(zip(a, b)) if x != y),
                         min(len(a), len(b)))
            out.update(first_differing_tick=first, stored_ticks=len(a), rerun_ticks=len(b))
    else:
        spec = got["spec"]
        if spec is None:
            from guardrail.scenario_spec import ScenarioSpec
            try:
                spec = ScenarioSpec.model_validate(_json(bundle / "manifest_extras.json")
                                                   .get("scenario_spec") or {})
            except ValueError as e:
                raise RerunError(f"manifest_extras scenario_spec: {e}") from None
        res = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
        table = doc.get("kpi") if isinstance(doc.get("kpi"), dict) else {}
        diffs = {f: {"stored": table.get(f), "rerun": res.get(f)} for f in KPI_FIELDS
                 if f in table and table.get(f) != res.get(f)}
        end = next((e for e in reversed(run.events) if e.get("type") == "episode_end"), {})
        ev = bundle / "harness_events.jsonl"
        stored_end = {}
        if ev.is_file():
            for line in ev.read_text(encoding="utf-8").splitlines():
                try:
                    e = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if e.get("type") == "episode_end":
                    stored_end = e
        if stored_end.get("final") != end.get("final"):
            diffs["final"] = {"stored": stored_end.get("final"), "rerun": end.get("final")}
        out.update(compared="kpi.json headline figures and the final position (the "
                            "bundle kept no flight_log.jsonl: weaker than a log hash)",
                   kpi_differences=diffs, match=not diffs)
    out["verdict"] = "identical" if out["match"] else "differs"
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("bundle", help="a headless episode bundle folder")
    ap.add_argument("--seed", type=int, default=None,
                    help="fly another seed (the null: it must differ)")
    ap.add_argument("--write-log", default=None, help="write the re-run's flight_log.jsonl")
    ap.add_argument("--json", action="store_true", help="print the report as JSON")
    args = ap.parse_args(argv)
    try:
        rep = compare(Path(args.bundle), args.seed)
    except RerunError as e:
        print(f"cannot rebuild: {e}", file=sys.stderr)
        return 2
    except Exception:                                            # noqa: BLE001
        # Not "differs" (1) and not "cannot rebuild" (2): a script reading
        # the exit code must not mistake this tool's own failure for either.
        import traceback
        traceback.print_exc()
        return 3
    rows = rep.pop("_rows", None)
    if args.write_log and rows is not None:
        with open(args.write_log, "wb") as fh:
            fh.write(log_bytes(rows))
    try:
        from guardrail.manifest import code_revision
        rep["code_revision_now"] = code_revision(ROOT)
    except Exception:                                            # noqa: BLE001
        rep["code_revision_now"] = None
    if args.json:
        print(json.dumps(rep, indent=1, sort_keys=True, default=str))
    else:
        print(f"{rep['bundle']}: rebuilt from {rep['rebuilt_from']}, seed "
              f"{rep['seed_flown']} (recorded {rep['seed_recorded']}, consumed "
              f"{rep['seed_consumed']})")
        print(f"  compared {rep.get('compared', '-')}: {rep['verdict'].upper()}")
        if rep.get("stored_log_sha256"):
            print(f"  stored {rep['stored_log_sha256']}\n  re-run {rep['rerun_log_sha256']}")
        if rep.get("first_differing_tick") is not None:
            print(f"  first differing line {rep['first_differing_tick']} "
                  f"({rep['stored_ticks']} stored, {rep['rerun_ticks']} re-run)")
        for f, d in (rep.get("kpi_differences") or {}).items():
            print(f"  {f}: stored {d['stored']}, re-run {d['rerun']}")
        if rep.get("why"):
            print(f"  {rep['why']}")
        print(f"  code revision flown {rep['code_revision_flown']}, now "
              f"{rep['code_revision_now']}")
    if rep["verdict"] == "cannot_compare":
        return 2
    return 0 if rep["match"] else 1


if __name__ == "__main__":
    sys.exit(main())
