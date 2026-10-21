"""Build tracker/data/dashboard.json: every number on the tracker's Dashboard.

    python tools/build_dashboard.py            # full: ffprobe the videos, make posters
    python tools/build_dashboard.py --quick    # no ffprobe, no ffmpeg (what the server runs)
    python tools/build_dashboard.py --check    # exit 1 when dashboard.json or the cards are stale
    python tools/build_dashboard.py --demo-out D:/path/to/demo/out   # e.g. from a git worktree

Standard library only (ffprobe and ffmpeg are optional), so it runs under
either conda env and inside the tracker server (tracker/serve.py rebuilds
with --quick when an input is newer than the file).

WHAT IT READS, and nothing else:
  demo/out/<tag>/{manifest,kpi,metrics}.json and the videos beside them
      (folders starting with "_" are set aside and skipped);
  docs/data/*.json (the published KPI sources) and
      deploy/evidence/dev/shield_tick_profile.json (the tick profile);
  tracker/data/cards/batch_*.json, content_overview.json, updates.json,
      todo.json;
  CHANGELOG.md (its section headings and dates).
It never opens tracker/data/state.json (the user's board): the board bar is
computed in the browser from /api/state, as before.

THE RULES, each because the opposite already happened somewhere:
  * Every number carries `source` ("path#/json/pointer") and `as_of` (the
    source's own `generated` date, else its last commit date, else its file
    date). No number is typed in here; targets are constants cited to the
    grant's locked KPI tables (kuanting-vla-uav-guardrail/docs/
    02-implementation/*.md, line numbers in KPI_SPECS).
  * A KPI with no source is `value: null` and status "not_measured". The
    page prints "not measured", never 0: a zero that only means "never ran"
    reads as success (memory note silence-reads-as-success). A self-test
    feeds the builder nothing and checks exactly that.
  * Every KPI row carries its null baseline when the source has one, and is
    green on the page only when it meets the target AND is KPI-grade AND,
    for a KPI that has a null (NULL_REQUIRED: P0 escape, fail-safe, mission
    success, repair success, P0 coverage), beats it - a missing null is not
    beaten (the published fail-safe value, 0.838, is below its
    never-trigger null, 0.940). A flight's P0 null is its paired Shield-off
    arm. Timing budgets and tracked counts have no null by definition.
  * KPI-grade is RECOMPUTED from the manifest: topology "hil" (the grant's
    Jetson Orin topology; a stored "canonical-hil" reads as "dev"), sim
    speedup 1.0, a clean tree and resolved hashes. A stored kpi_grade is
    shown beside it and never trusted: 5 stored kpi.json files from the
    August MAVROS runs still say true, while the 6 Oct rollup counts 0.
  * A flight is a VLA flight only when its vla_model_hash names an allow-
    listed VLA (tools/todo.py classify_pilot) AND its own records show the
    VLA SERVER answered and its steps were flown (tools/todo.py
    vla_evidence / classify_run: accepted answers the model parsed, each
    with the server's weights_sha256, and ticks authored by vla_step, read
    from vla_steps.jsonl or metrics.json's pilot block). 65 of the 99
    manifests of 21 Oct name the OWL-ViT DETECTOR there; with the 24
    StubVLA and 10 hand-written ones, 0 are VLA flights. The vla_model_hash
    is computed on the pilot side from the configured id and the local
    weight files, so a run configured with AerialVLA whose server never
    answered counted as one, and closed the first-VLA-flight milestone
    (review of 8035d0a, P2): it is now "vla_unverified", shown apart.
  * Only browser-playable videos are linked: the per-run `_demo.mp4` files
    are MPEG-4 Part 2, which browsers do not play. With ffprobe the codec is
    read from the file (cached by size and mtime in tracker/data/dash/
    media_cache.json); without it (--quick) a cached answer or the naming
    convention (`_phone` / `_share` = H.264, `_demo` = MPEG-4 Part 2) is
    used, and the basis is recorded.

`--check` exits 1 when dashboard.json is older than any input, or when the
newest CHANGELOG section is newer than the newest card review (the cards then
describe a project that has moved on), mirroring build_public_site.py --check.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from datetime import date, datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))

import add_update as AU                                     # noqa: E402
import todo as TODO                                         # noqa: E402

SCHEMA = 1
LATEST_MEDIA = 16                # flights whose videos are probed and given posters
FINAL = "2026-11-30"             # final delivery (reference/Grant overview p.2)
MIDTERM = "2026-07-20"           # mid-term hard gate (same page)
AREAS = ("WP1", "WP2", "WP3", "WP4", "ARCH", "CROSS", "CLAIMS")
VERDICTS = ("DONE", "PARTIAL", "MISSING", "DRIFT")
PLAYABLE = {"h264", "vp8", "vp9", "av1"}
VIDEO_EXT = (".mp4", ".webm")
# stored topology -> how the grant reads it (guardrail/manifest.py
# normalize_topology: a stored "canonical-hil" is the desktop dev rail)
TOPO_NORMAL = {"canonical-hil": "dev"}
TOPO_RAIL = {"projectairsim-ardupilot-pymavlink": "pas_ap", "projectairsim-ardupilot-mavros": "pas_ap",
             "projectairsim-single-host": "airsim", "ardupilot-sitl-pymavlink": "sitl",
             "dev": "sitl", "canonical-hil": "sitl", "hil": "hil", "headless-kinematic": "headless"}
GRANT = "kuanting-vla-uav-guardrail/docs/02-implementation/"


def paths(root: Path, demo_out: Path | None = None, data_dir: Path | None = None) -> dict:
    data = Path(data_dir) if data_dir else root / "tracker" / "data"
    return {
        "root": root, "data": data, "demo_out": Path(demo_out) if demo_out else root / "demo" / "out",
        "docs_data": root / "docs" / "data", "changelog": root / "CHANGELOG.md",
        "out": data / "dashboard.json", "dash": data / "dash",
    }


# ------------------------------------------------------------------ small helpers
def read_json(p: Path):
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def ptr(doc, pointer: str):
    """JSON-pointer lookup ("/a/b/0"); None when any step is missing."""
    cur = doc
    for part in [x for x in pointer.split("/") if x != ""]:
        part = part.replace("~1", "/").replace("~0", "~")
        if isinstance(cur, dict) and part in cur:
            cur = cur[part]
        elif isinstance(cur, list) and part.isdigit() and int(part) < len(cur):
            cur = cur[int(part)]
        else:
            return None
    return cur


def num(x):
    return float(x) if isinstance(x, (int, float)) and not isinstance(x, bool) else None


def git(root: Path, *args: str) -> str | None:
    """A read-only git query that never takes an optional lock. The server's
    background --quick rebuild runs this up to every 30 s in the user's own
    checkout, and a plain `git status` may take .git/index.lock to refresh
    the index, so a `git commit` or `git checkout` the user runs at that
    moment fails with "index.lock exists" (review of 8035d0a, P7)."""
    try:
        r = subprocess.run(["git", "--no-optional-locks", *args], cwd=root, capture_output=True,
                           text=True, encoding="utf-8", timeout=20,
                           env={**os.environ, "GIT_OPTIONAL_LOCKS": "0"})
        return r.stdout.strip() if r.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        return None


def mtime_date(p: Path) -> str | None:
    try:
        return datetime.fromtimestamp(p.stat().st_mtime).date().isoformat()
    except OSError:
        return None


_AS_OF: dict = {}


def as_of(root: Path, p: Path, doc=None) -> tuple[str | None, str]:
    """(YYYY-MM-DD, basis) for a source file: its own `generated`, else its
    last commit, else its file date (a fresh git worktree gives every file
    today's mtime, so the commit date comes before the mtime)."""
    for k in ("generated", "created_utc", "created"):
        g = doc.get(k) if isinstance(doc, dict) else None
        if isinstance(g, str) and re.match(r"\d{4}-\d{2}-\d{2}", g):
            return g[:10], k
    key = str(p)
    if key not in _AS_OF:
        try:
            rel = p.resolve().relative_to(root.resolve()).as_posix()
            d = git(root, "log", "-1", "--format=%cs", "--", rel)
        except ValueError:
            d = None
        _AS_OF[key] = (d, "commit") if d else (mtime_date(p), "mtime")
    return _AS_OF[key]


def rel(root: Path, p: Path) -> str:
    """p relative to the repository: as spelled first (tracker/data may be a
    junction into another checkout), then resolved."""
    for a, b in ((p, root), (p.resolve(), root.resolve())):
        try:
            return a.relative_to(b).as_posix()
        except ValueError:
            pass
    return p.as_posix()


# ------------------------------------------------------------------ media
def codec_by_name(name: str) -> str | None:
    n = name.lower()
    if n.endswith("_demo.mp4"):
        return "mpeg4"
    if re.search(r"_(phone|share)\.(mp4|webm)$", n):
        return "h264"
    return None


class Media:
    """Codec answers (ffprobe, cached by size + mtime) and posters (ffmpeg)."""

    def __init__(self, dash: Path, quick: bool):
        self.dash = dash
        self.quick = quick
        self.cache_path = dash / "media_cache.json"
        self.cache = read_json(self.cache_path) or {}
        self.ffprobe = None if quick else shutil.which("ffprobe")
        self.ffmpeg = None if quick else shutil.which("ffmpeg")
        self.changed = False

    def codec(self, f: Path, key: str) -> tuple[str | None, str]:
        try:
            st = f.stat()
        except OSError:
            return None, "missing"
        c = self.cache.get(key)
        if isinstance(c, dict) and c.get("size") == st.st_size and c.get("mtime_ns") == st.st_mtime_ns:
            return c.get("codec"), "ffprobe"
        if self.ffprobe:
            try:
                r = subprocess.run([self.ffprobe, "-v", "error", "-select_streams", "v:0",
                                    "-show_entries", "stream=codec_name", "-of",
                                    "default=noprint_wrappers=1:nokey=1", str(f)],
                                   capture_output=True, text=True, timeout=30)
                codec = (r.stdout or "").strip().splitlines()[0] if r.returncode == 0 and r.stdout.strip() else None
            except (OSError, subprocess.SubprocessError, IndexError):
                codec = None
            if codec:
                self.cache[key] = {"size": st.st_size, "mtime_ns": st.st_mtime_ns, "codec": codec}
                self.changed = True
                return codec, "ffprobe"
        return codec_by_name(f.name), "name"

    def poster(self, video: Path, name: str) -> str | None:
        """data/dash/posters/<name>.jpg when it exists (made here unless
        --quick); None otherwise."""
        out = self.dash / "posters" / f"{name}.jpg"
        try:
            fresh = out.is_file() and out.stat().st_mtime >= video.stat().st_mtime
        except OSError:
            fresh = False
        if not fresh and self.ffmpeg:
            out.parent.mkdir(parents=True, exist_ok=True)
            tmp = out.with_name(f".{out.name}.tmp.jpg")
            try:
                subprocess.run([self.ffmpeg, "-y", "-v", "error", "-ss", "4", "-i", str(video),
                                "-frames:v", "1", "-vf", "scale=480:-2", "-q:v", "5", str(tmp)],
                               capture_output=True, timeout=60)
                if tmp.is_file() and tmp.stat().st_size > 0:
                    os.replace(tmp, out)
            except (OSError, subprocess.SubprocessError):
                pass
            finally:
                if tmp.exists():
                    try:
                        tmp.unlink()
                    except OSError:
                        pass
        return f"data/dash/posters/{name}.jpg" if out.is_file() else None

    def save(self):
        if self.changed:
            try:
                write_atomic(self.cache_path, self.cache)
            except OSError:
                pass


# ------------------------------------------------------------------ flights
def _date_of(tag: str, manifest: Path) -> tuple[str | None, str]:
    d = mtime_date(manifest)
    if d:
        return d, "manifest_mtime"
    m = re.search(r"(20\d{2})(\d{2})(\d{2})", tag)
    return (f"{m.group(1)}-{m.group(2)}-{m.group(3)}", "tag") if m else (None, "unknown")


def recompute_kpi_grade(manifest: dict) -> tuple[bool, list[str]]:
    """The manifest's own facts, judged the way guardrail/manifest.py
    is_kpi_grade judges them (without its imports): reason codes, not text."""
    reasons = []
    stored = manifest.get("topology")
    topo = TOPO_NORMAL.get(stored, stored)
    if topo != "hil":
        reasons.append(f"topology:{stored}")
    if manifest.get("sim_speedup") != 1.0:
        reasons.append("sim_speedup")
    rev = str(manifest.get("code_revision") or "")
    if not rev or rev.endswith(("-dirty", "-unknown")) or rev == "unversioned":
        reasons.append("code_revision")
    for f in ("vla_model_hash", "policy_hash"):
        v = str(manifest.get(f) or "")
        if not v or "unresolved" in v:
            reasons.append(f)
    return not reasons, reasons


def _variant(tag: str) -> str | None:
    """The on/off token of a paired run's tag (pas_ap_ovf_on_20261021). It
    names what the pair compares, which is not always the Shield: both
    pas_ap_ovf runs fly the Shield and compare its forecast."""
    toks = tag.split("_")
    return "on" if "on" in toks else "off" if "off" in toks else None


def _shield(tag: str, metrics: dict | None) -> str | None:
    """metrics.json "shield" when the rail writes it, else the tag's token."""
    s = (metrics or {}).get("shield")
    return s if s in ("on", "off") else _variant(tag)


def _pair_key(tag: str) -> str | None:
    toks = tag.split("_")
    if "on" not in toks and "off" not in toks:
        return None
    return "_".join("*" if t in ("on", "off") else t for t in toks)


def _evidence_summary(tag: str, ev: dict) -> dict:
    """The counts the page may show of what the VLA server returned: no raw
    text, no prompt, no image hash; the source as a logical demo/out path."""
    src = ev.get("source")
    return {
        "source": None if src is None else f"demo/out/{tag}/{src}",
        "decisions": ev.get("decisions") or 0, "vla_step_ticks": ev.get("vla_step_ticks") or 0,
        "weights_sha256": ev.get("weights_sha256"),
        "model_id": None if ev.get("model_id") is None else str(ev["model_id"])[:80],
        "digests_seen": len(ev.get("digests") or []),
        **{k: ev.get(k) or 0 for k in ("parse_fail", "server_error", "link_failure", "no_digest")},
        "min_decisions": TODO.MIN_VLA_DECISIONS, "min_vla_step_ticks": TODO.MIN_VLA_STEP_TICKS,
    }


def flight_record(root: Path, run: Path) -> dict | None:
    tag = run.name
    mp = run / "manifest.json"
    man = read_json(mp)
    if not isinstance(man, dict):
        return None
    kpi = read_json(run / "kpi.json")
    kpi = kpi if isinstance(kpi, dict) else None
    met = read_json(run / "metrics.json")
    met = met if isinstance(met, dict) else None
    d, basis = _date_of(tag, mp)
    try:
        mt = mp.stat().st_mtime
    except OSError:
        mt = 0.0
    h = man.get("vla_model_hash")
    # The class needs the SERVER's word for a VLA (tools/todo.py classify_run):
    # a manifest that only names AerialVLA is "vla_unverified" (review of
    # 8035d0a, P2).
    ev = TODO.vla_evidence(run, met)
    cls, unverified = TODO.classify_run(h, ev)
    configured = TODO.classify_pilot(h)
    grade, reasons = recompute_kpi_grade(man)
    topo = man.get("topology")
    p0 = None
    if kpi is not None:
        p0 = {"escapes": kpi.get("p0_escapes"), "rate": num(kpi.get("p0_violation_escape_rate")),
              "status": kpi.get("p0_violation_escape_rate_status"),
              "violation_ticks": kpi.get("p0_violation_ticks")}
    tw = (met or {}).get("tick_work_ms") if isinstance((met or {}).get("tick_work_ms"), dict) else None
    pilot_kind = ((met or {}).get("pilot") or {}).get("kind") if isinstance((met or {}).get("pilot"), dict) else None
    stored_reasons = (kpi or {}).get("kpi_grade_reasons")
    return {
        "tag": tag, "date": d, "date_basis": basis, "t": round(mt, 1),
        "rail": TOPO_RAIL.get(topo) or ("pas_ap" if tag.startswith(("pas_ap", "g1_")) else "airsim"),
        "topology": topo,
        "pilot": {"class": cls, "configured": configured, "unverified_reason": unverified,
                  "id": str(h).split("@", 1)[0] if h else None,
                  "hash": str(h)[:80] if h else None, "metrics_kind": pilot_kind,
                  "evidence": _evidence_summary(tag, ev) if (configured == "vla" or ev.get("source")) else None},
        "policy_hash": str(man.get("policy_hash") or "")[:23] or None,
        "seed": man.get("random_seed"), "code_revision": man.get("code_revision"),
        "sim_speedup": man.get("sim_speedup"),
        "kpi_grade": grade, "kpi_grade_reasons": reasons,
        "stored_kpi_grade": kpi.get("kpi_grade") if kpi else None,
        "stored_kpi_grade_reasons": [str(r)[:300] for r in stored_reasons][:4]
        if isinstance(stored_reasons, list) else [],
        "shield": _shield(tag, met), "variant": _variant(tag), "pair": _pair_key(tag),
        "ticks": (kpi or {}).get("ticks", (met or {}).get("ticks")),
        "loop_hz": num((met or {}).get("loop_hz")),
        "end_reason": str((met or {}).get("end_reason"))[:120] if (met or {}).get("end_reason") else None,
        "mission_s": num((met or {}).get("mission_s")),
        "p0": p0, "unsafe_p0_ticks": (kpi or {}).get("unsafe_p0_ticks"),
        "frac_within_30m": num((met or {}).get("frac_within_30m")),
        "nfz_s": num((met or {}).get("nfz_s")), "det_hz": num((met or {}).get("det_hz")),
        "tick_work_ms": {k: num(tw.get(k)) for k in ("median", "p95", "max")} if tw else None,
        "videos": [], "poster": None,
        # logical paths: demo/out is what /media/run/ serves, wherever it lives
        "sources": {"manifest": f"demo/out/{tag}/manifest.json",
                    "kpi": f"demo/out/{tag}/kpi.json" if kpi is not None else None,
                    "metrics": f"demo/out/{tag}/metrics.json" if met is not None else None},
    }


def _video_entries(media: Media, demo_out: Path, files: list[Path]) -> list[dict]:
    out = []
    for f in sorted(files, key=lambda p: p.name):
        key = f.relative_to(demo_out).as_posix()
        codec, basis = media.codec(f, key)
        if codec in PLAYABLE:
            try:
                size = f.stat().st_size
            except OSError:
                continue
            out.append({"path": f"run/{key}", "codec": codec, "basis": basis, "bytes": size,
                        "phone": bool(re.search(r"_phone\.", f.name))})
    # the small phone copy first, then the rest
    return sorted(out, key=lambda v: (not v["phone"], v["path"]))


def scan_flights(P: dict, media: Media) -> list[dict]:
    root, demo_out = P["root"], P["demo_out"]
    runs = []
    if demo_out.is_dir():
        for mp in demo_out.glob("*/manifest.json"):
            if mp.parent.name.startswith("_"):
                continue
            r = flight_record(root, mp.parent)
            if r:
                runs.append(r)
    runs.sort(key=lambda r: (r["date"] or "", r["t"]), reverse=True)
    for r in runs[:LATEST_MEDIA]:
        run = demo_out / r["tag"]
        vids = [f for f in run.iterdir() if f.is_file() and f.suffix.lower() in VIDEO_EXT] if run.is_dir() else []
        r["videos"] = _video_entries(media, demo_out, vids)
        src = next((f for f in sorted(vids, key=lambda p: ("_phone" not in p.name, p.name))), None)
        if src is not None:
            r["poster"] = media.poster(src, r["tag"])
    return runs


def scan_compares(P: dict, media: Media, flights: list[dict]) -> list[dict]:
    """Top-level demo/out/<prefix>_compare_<date>[_phone].mp4 videos. The runs
    a video stacks are named only by a sidecar demo/out/<prefix>_compare_
    <date>.json ({"a": tag, "b": tag}); without one `runs` is empty. A guess
    from the file name was tried and was wrong on the first real case:
    citylife_motion_compare_20261019 stacks the 13 Oct flight beside the
    second 19 Oct one (its title bars), while the name matches both 19 Oct
    runs and neither 13 Oct one."""
    demo_out = P["demo_out"]
    known = {r["tag"] for r in flights}
    groups: dict[str, list[Path]] = {}
    if demo_out.is_dir():
        for f in demo_out.glob("*_compare_*"):
            if f.is_file() and f.suffix.lower() in VIDEO_EXT:
                key = re.sub(r"_(phone|share)$", "", f.stem)
                groups.setdefault(key, []).append(f)
    out = []
    for key, files in groups.items():
        m = re.match(r"^(.*)_compare_(20\d{6})$", key)
        if not m:
            continue
        dtok = m.group(2)
        side = read_json(demo_out / f"{key}.json")
        runs = [side.get(k) for k in ("a", "b")] if isinstance(side, dict) else []
        runs = [r for r in runs if isinstance(r, str) and r in known]
        vids = _video_entries(media, demo_out, files)
        try:
            mt = max(f.stat().st_mtime for f in files)
        except (OSError, ValueError):
            mt = 0.0
        src = next((f for f in sorted(files, key=lambda p: ("_phone" not in p.name, p.name))), None)
        out.append({"key": key, "date": f"{dtok[:4]}-{dtok[4:6]}-{dtok[6:]}", "t": round(mt, 1),
                    "runs": runs, "runs_source": f"demo/out/{key}.json" if runs else None, "videos": vids,
                    "poster": media.poster(src, key) if src is not None else None})
    return sorted(out, key=lambda c: (c["date"], c["t"]), reverse=True)


# ------------------------------------------------------------------ VLA
def vla_section(flights: list[dict], todo_items: list[dict]) -> dict:
    counts = {k: 0 for k in ("vla", "vla_unverified", "stub", "detector", "none", "unknown", "missing")}
    for r in flights:
        counts[r["pilot"]["class"]] = counts.get(r["pilot"]["class"], 0) + 1
    vla = [r for r in flights if r["pilot"]["class"] == "vla"]
    first = min(vla, key=lambda r: (r["date"] or "", r["t"])) if vla else None
    last = max(vla, key=lambda r: (r["date"] or "", r["t"])) if vla else None
    # Configured as a VLA, no server evidence: shown apart, never counted.
    unv = [r for r in flights if r["pilot"]["class"] == "vla_unverified"]
    last_unv = max(unv, key=lambda r: (r["date"] or "", r["t"])) if unv else None
    latest_by_rail = {}
    for r in flights:                                       # newest first already
        latest_by_rail.setdefault(r["rail"], r)
    return {
        "counts": counts, "total": len(flights),
        "first_vla_flight": {"tag": first["tag"], "date": first["date"], "id": first["pilot"]["id"]} if first else None,
        "last_vla_flight": {"tag": last["tag"], "date": last["date"], "id": last["pilot"]["id"],
                            "decisions": last["pilot"]["evidence"]["decisions"],
                            "weights_sha256": last["pilot"]["evidence"]["weights_sha256"]} if last else None,
        "last_vla_unverified": {"tag": last_unv["tag"], "date": last_unv["date"], "id": last_unv["pilot"]["id"],
                                "reason": last_unv["pilot"]["unverified_reason"]} if last_unv else None,
        "evidence_rule": {"min_decisions": TODO.MIN_VLA_DECISIONS, "min_vla_step_ticks": TODO.MIN_VLA_STEP_TICKS},
        "latest_by_rail": [{"rail": k, "tag": r["tag"], "date": r["date"], "topology": r["topology"],
                            "pilot_class": r["pilot"]["class"], "pilot_id": r["pilot"]["id"]}
                           for k, r in latest_by_rail.items()],
        "milestones": [i["id"] for i in todo_items if i.get("milestone") == "vla"],
    }


# ------------------------------------------------------------------ WP progress
def load_cards(data: Path) -> list[dict]:
    cards = []
    for f in sorted((data / "cards").glob("batch_*.json")):
        d = read_json(f)
        if isinstance(d, list):
            cards += [c for c in d if isinstance(c, dict) and c.get("id")]
    return cards


def cited_cards(entry: dict) -> set[str]:
    ids = set(entry.get("cards") or [])
    for c in entry.get("changes") or []:
        ids |= set(c.get("cards") or [])
    return ids


def wp_section(cards: list[dict], updates: list[dict]) -> dict:
    by_id = {c["id"]: c for c in cards}
    stale = {}
    for e in updates:
        d = e.get("date") or ""
        for cid in cited_cards(e):
            c = by_id.get(cid)
            if c is not None and (c.get("updated") or "") < d:
                stale[cid] = max(stale.get(cid, ""), d)
    areas = []
    for a in AREAS:
        cs = [c for c in cards if c.get("area") == a]
        if not cs:
            continue
        counts = {v: sum(1 for c in cs if c.get("status") == v) for v in VERDICTS}
        areas.append({
            "area": a, "total": len(cs), "counts": counts,
            "gate_open": sorted(c["id"] for c in cs if c.get("priority") == "gate" and c.get("status") != "DONE"),
            "newest_review": max((c.get("updated") or "" for c in cs), default="") or None,
            "needs_review": sorted(cid for cid in stale if by_id[cid].get("area") == a),
        })
    return {"areas": areas, "needs_review_total": len(stale),
            "newest_review": max((c.get("updated") or "" for c in cards), default="") or None,
            "cards_total": len(cards)}


# ------------------------------------------------------------------ KPIs
def _src(root, p, doc, pointer, **extra) -> dict | None:
    v = ptr(doc, pointer) if doc is not None else None
    if v is None:
        return None
    d, basis = as_of(root, p, doc)
    out = {"value": v, "source": f"{rel(root, p)}#{pointer}", "as_of": d, "as_of_basis": basis}
    out.update({k: x for k, x in extra.items() if x is not None})
    return out


def _frac(s) -> tuple[float, int, int] | None:
    m = re.fullmatch(r"\s*(\d+)\s*/\s*(\d+)\s*", str(s or ""))
    if not m or int(m.group(2)) == 0:
        return None
    return int(m.group(1)) / int(m.group(2)), int(m.group(1)), int(m.group(2))


def _latest_flight(flights, need) -> dict | None:
    """The newest Shield-on flight that has the value, preferring a run whose
    A/B variant is not "off". Skipping only shield == "off" took
    pas_ap_ovf_off_20261021 - the Shield on, the own-velocity forecast OFF,
    the deliberately degraded control arm that gave 79 unsafe margin ticks -
    as the "latest Shield-on flight" (review of 8035d0a, P5). An off arm is
    used only when no other Shield-on run has the value; the row names the
    arm either way (kpi_rows `flight`)."""
    on = [r for r in flights if r.get("shield") != "off" and need(r)]
    return next((r for r in on if r.get("variant") != "off"), on[0] if on else None)


# Which KPIs are judged against a null, and against which one. Green needs
# the null BEATEN for these; a missing null keeps the row from going green.
# It used to be `beats_null is not False`, so a row with no null passed: the
# first clean hil flight with zero P0 escapes would have gone green with no
# null comparison, although its Shield-off pair is the natural null (review
# of 8035d0a, P4). Every other KPI has no null by definition: a timing
# budget (tick, monitor query, cold start, hot apply), a pass rate checked
# against 100 % (round trip, replayability) or a tracked count.
NULL_REQUIRED = {
    "p0_escape_rate": "Shield off (the harness passthrough, or a flight's paired Shield-off arm)",
    "failsafe_trigger_correctness": "never trigger",
    "mission_success_rate": "hover",
    "repair_success_rate": "always brake",
    "p0_coverage": "no CSP",
}


def kpi_rows(P: dict, flights: list[dict]) -> list[dict]:
    root, dd = P["root"], P["docs_data"]
    sweep_p, wp1_p = dd / "scenario_sweep.json", dd / "wp1_roundtrip.json"
    csp_p, prog_p = dd / "csp_kpis.json", dd / "progress_oct.json"
    prof_p = root / "deploy" / "evidence" / "dev" / "shield_tick_profile.json"
    sweep, wp1, csp, prog, prof = (read_json(p) for p in (sweep_p, wp1_p, csp_p, prog_p, prof_p))
    allon = "/_rollup/_all_shield_on"

    def harness(field, null=None, n=None, **kw):
        if sweep is None:
            return None
        s = _src(root, sweep_p, sweep, f"{allon}/{field}", grade="headless", **kw)
        if s is None:
            return None
        if null:
            s["null"] = ptr(sweep, f"{allon}/{null}")
            s["null_source"] = f"{rel(root, sweep_p)}#{allon}/{null}"
        if n:
            s["n"] = ptr(sweep, f"{allon}/{n}")
        return s

    def flight(field_fn, label, null_fn=None):
        r = _latest_flight(flights, lambda x: field_fn(x) is not None)
        if r is None:
            return None
        v, pointer, src_key = field_fn(r)
        what = label if not r.get("pair") else f"{label} ({r.get('variant')} arm of {r['pair']})"
        out = {"value": v, "source": f"{r['sources'][src_key]}#{pointer}", "as_of": r["date"],
               "as_of_basis": "flight", "grade": "kpi" if r["kpi_grade"] else "functional",
               "run": r["tag"], "arm": r.get("variant"), "what": what}
        nul = null_fn(r) if null_fn else None
        if nul is not None:
            out["null"], out["null_source"], out["null_run"] = nul
        return out

    def p0_flight(r):
        if r.get("p0") and r["p0"].get("rate") is not None and r["p0"].get("status") == "measured":
            return r["p0"]["rate"], "/p0_violation_escape_rate", "kpi"
        return None

    def p0_null(r):
        """A flight's P0 null: its paired arm (same pair key) flown with the
        Shield OFF, measured. A pair that compares something else (both
        pas_ap_ovf arms fly the Shield) has none."""
        if not r.get("pair"):
            return None
        for o in flights:
            if (o is not r and o.get("pair") == r["pair"] and o.get("shield") == "off"
                    and p0_flight(o) is not None and o["sources"].get("kpi")):
                return o["p0"]["rate"], f"{o['sources']['kpi']}#/p0_violation_escape_rate", o["tag"]
        return None

    def tick_flight(r):
        tw = r.get("tick_work_ms")
        if tw and tw.get("p95") is not None:
            return tw["p95"], "/tick_work_ms/p95", "metrics"
        return None

    def bench_check_p99():
        cells = ptr(prog, "/wp3_bench/grant_load") if prog else None
        if not isinstance(cells, dict):
            return None
        best = None
        for k, v in cells.items():
            if k.endswith("/_check") and num(ptr(v, "/current/p99_ms")) is not None:
                if best is None or v["current"]["p99_ms"] > best[1]:
                    best = (k, v["current"]["p99_ms"])
        if best is None:
            return None
        return _src(root, prog_p, prog, f"/wp3_bench/grant_load/{best[0].replace('/', '~1')}/current/p99_ms",
                    grade="bench", what="worst cell p99")

    def bench_filter_p99():
        cells = ptr(prog, "/wp3_bench/grant_load") if prog else None
        if not isinstance(cells, dict):
            return None
        best = None
        for k, v in cells.items():
            if k.endswith("/filter") and num(ptr(v, "/current/p99_ms")) is not None:
                if best is None or v["current"]["p99_ms"] > best[1]:
                    best = (k, v["current"]["p99_ms"])
        if best is None:
            return None
        return _src(root, prog_p, prog, f"/wp3_bench/grant_load/{best[0].replace('/', '~1')}/current/p99_ms",
                    grade="bench", what="worst cell filter() p99")

    def fs():
        if sweep is None:
            return None
        s = _src(root, sweep_p, sweep, "/_failsafe_labels/fail_safe_correctness", grade="headless",
                 n=ptr(sweep, "/_failsafe_labels/labelled"),
                 correct=ptr(sweep, "/_failsafe_labels/correct"),
                 false_triggers=ptr(sweep, "/_failsafe_labels/false_triggers"),
                 missed=ptr(sweep, "/_failsafe_labels/missed_triggers"))
        if s:
            s["null"] = ptr(sweep, "/_failsafe_labels/null_never_trigger")
            s["null_source"] = f"{rel(root, sweep_p)}#/_failsafe_labels/null_never_trigger"
        return s

    def cold():
        s = _src(root, wp1_p, wp1, "/cold_start/verdict/p95_ms", grade="bench",
                 whole_process_p95_ms=ptr(wp1, "/cold_start/verdict/whole_process_wall_p95_ms"))
        return s

    def cov():
        s = _src(root, csp_p, csp, "/default/p0_coverage", grade="offline",
                 discriminates=ptr(csp, "/default/kpi_discriminates"),
                 baseline=ptr(csp, "/default/p0_coverage_baseline"))
        if s:
            s["null"] = ptr(csp, "/default/p0_coverage_null")
            s["null_source"] = f"{rel(root, csp_p)}#/default/p0_coverage_null"
        return s

    def round_trip():
        s = _src(root, wp1_p, wp1, "/kpi/round_trip", grade="offline")
        f = _frac(s["value"]) if s else None
        if not f:
            return None
        s["value"], s["k"], s["n"] = f[0], f[1], f[2]
        neg = _frac(ptr(wp1, "/kpi/negative_refusal"))
        if neg:
            s["negative_refused"], s["negative_n"] = neg[1], neg[2]
        return s

    def replay():
        if wp1 is None or num(ptr(wp1, "/stored_runs/n")) is None:
            return None
        s = _src(root, wp1_p, wp1, "/stored_runs/matched_after", grade="offline")
        n = ptr(wp1, "/stored_runs/n")
        if s is None or num(s["value"]) is None:
            return None
        if not n:
            return None
        aside = [r for r in (ptr(wp1, "/stored_runs/set_aside") or []) if not r.get("resolves")]
        # A ratio of two pointers: `derived` names both, so the page (and the
        # test that resolves every source) can recompute it.
        base = rel(root, wp1_p)
        s["value"], s["k"], s["n"] = s["value"] / n, s["value"], n
        s["source"] = f"{base}#/stored_runs"
        s["derived"] = {"k": f"{base}#/stored_runs/matched_after", "n": f"{base}#/stored_runs/n"}
        s["set_aside_unresolved"] = len(aside)
        return s

    def prof_check():
        return _src(root, prof_p, prof, "/headline/check_p99_ms", grade="bench",
                    what="dev tick profile, real flight states") if prof else None

    def prof_filter():
        return _src(root, prof_p, prof, "/headline/filter_p99_ms", grade="bench",
                    what="dev tick profile, real flight states") if prof else None

    specs = [
        ("p0_escape_rate", "acceptance", ("<=", 0.0), "ratio", GRANT + "stress-testing.md:213; safety-shield.md:163",
         [lambda: harness("p0_escape_rate", null="null_passthrough_p0_escape_rate", n="p0_violation_ticks",
                          escapes=ptr(sweep, f"{allon}/p0_escapes") if sweep else None),
          lambda: flight(p0_flight, "latest Shield-on flight", null_fn=p0_null)]),
        ("failsafe_trigger_correctness", "acceptance", (">=", 0.99), "ratio", GRANT + "stress-testing.md:214", [fs]),
        ("mission_success_rate", "acceptance", None, "ratio", GRANT + "stress-testing.md:212",
         [lambda: harness("mission_success_rate", null="null_hover_mission_success_rate", n="episodes_mission_scored")]),
        ("average_repair_count", "acceptance", None, "count", GRANT + "stress-testing.md:215",
         [lambda: harness("mean_repair_count_per_episode", n="episodes")]),
        ("mean_repair_magnitude", "acceptance", None, "mps", GRANT + "stress-testing.md:216",
         [lambda: harness("mean_repair_magnitude_mps")]),
        ("repair_success_rate", "wp3", None, "ratio", GRANT + "safety-shield.md:164",
         [lambda: harness("repair_success_rate", null="null_always_brake_repair_success_rate")]),
        ("mean_time_to_safe", "wp3", None, "s", GRANT + "safety-shield.md:165",
         [lambda: harness("mean_time_to_safe_s", n="time_to_safe_episodes",
                          censored=ptr(sweep, f"{allon}/time_to_safe_censored") if sweep else None)]),
        ("shield_tick", "wp3", ("<=", 100.0), "ms", GRANT + "safety-shield.md:166",
         [lambda: flight(tick_flight, "loop tick work p95, latest Shield-on flight"), prof_filter, bench_filter_p99]),
        ("monitor_query_latency", "wp1", ("<=", 5.0), "ms", GRANT + "policy-dsl.md:182",
         [bench_check_p99, prof_check]),
        ("cold_start", "wp1", ("<=", 1000.0), "ms", GRANT + "policy-dsl.md:180", [cold]),
        ("hot_apply", "wp1", ("<=", 50.0), "ms", GRANT + "policy-dsl.md:181", []),
        ("policy_round_trip", "wp1", (">=", 1.0), "ratio", GRANT + "policy-dsl.md:178-179", [round_trip]),
        ("bundle_replayability", "wp1", None, "ratio", GRANT + "policy-dsl.md:172", [replay]),
        ("p0_coverage", "wp2", (">=", 1.0), "ratio", GRANT + "prefix-compiler.md:177", [cov]),
        ("per_paraphrase_robustness", "wp4", None, "ratio", "reference/Grant overview (WP4 row)", []),
    ]
    rows = []
    for key, contract, target, unit, grant_src, readers in specs:
        vals = []
        for fn in readers:
            try:
                v = fn()
            except Exception:                              # noqa: BLE001 - a broken source is "not measured"
                v = None
            if v and num(v.get("value")) is not None:
                v["value"] = float(v["value"])
                vals.append(v)
        primary = vals[0] if vals else None
        row = {"key": key, "contract": contract, "unit": unit, "grant_source": grant_src,
               "target": {"op": target[0], "value": target[1]} if target else None,
               "status": "measured" if primary else "not_measured",
               "value": primary["value"] if primary else None, "primary": primary, "also": vals[1:]}
        row["meets_target"] = _meets(target, primary["value"]) if (primary and target) else None
        nul = num(primary.get("null")) if primary else None
        row["beats_null"] = None if nul is None else _beats(target, key, primary["value"], nul)
        row["kpi_grade"] = bool(primary and primary.get("grade") == "kpi")
        # NULL_REQUIRED: the null must be beaten, not merely "not lost to"
        row["null_rule"] = "required" if key in NULL_REQUIRED else "none"
        row["null_kind"] = NULL_REQUIRED.get(key)
        null_ok = row["beats_null"] is True if row["null_rule"] == "required" else True
        row["green"] = bool(row["meets_target"] and row["kpi_grade"] and null_ok)
        rows.append(row)
    return rows


LOWER_IS_BETTER = {"p0_escape_rate", "mean_repair_magnitude", "mean_time_to_safe", "shield_tick",
                   "monitor_query_latency", "cold_start", "hot_apply", "average_repair_count"}


def _meets(target, v) -> bool | None:
    if target is None or v is None:
        return None
    op, t = target
    return v <= t if op == "<=" else v >= t if op == ">=" else None


def _beats(target, key, v, null) -> bool:
    return v < null if key in LOWER_IS_BETTER else v > null


# ------------------------------------------------------------------ timeline, changelog
def changelog_info(P: dict) -> dict:
    try:
        secs = AU.parse_changelog(P["changelog"].read_text(encoding="utf-8"))
    except OSError:
        secs = []
    dated = sorted(secs, key=lambda s: s.date, reverse=True)
    newest = dated[0] if dated else None
    return {"newest_date": newest.date if newest else None,
            "newest_heading": newest.heading[:160] if newest else None,
            "sections": len(secs)}


def timeline_section(overview: dict | None, updates: list[dict], flights: list[dict],
                     todo_items: list[dict], today: str) -> dict:
    plan = []
    for i, g in enumerate((overview or {}).get("timeline") or []):
        if not isinstance(g, dict):
            continue
        plan.append({"index": i, "start": g.get("start"), "end": g.get("end"), "status": g.get("status"),
                     "overdue": bool(g.get("end") and g["end"] < today and g.get("status") != "DONE")})
    per_day = {}
    for r in flights:
        if r["date"]:
            d = per_day.setdefault(r["date"], {"date": r["date"], "n": 0, "vla": 0})
            d["n"] += 1
            d["vla"] += r["pilot"]["class"] == "vla"
    return {
        "start": "2026-02-01", "end": "2026-12-01", "today": today,
        "gates": [{"key": "midterm", "date": MIDTERM}, {"key": "final", "date": FINAL}],
        "plan": plan,
        "updates": [{"seq": e.get("seq"), "date": e.get("date"), "kind": e.get("kind")} for e in updates],
        "flights": sorted(per_day.values(), key=lambda x: x["date"]),
        "todo_due": [{"id": i["id"], "due": i["due"], "status": i["status_effective"]}
                     for i in todo_items if i.get("due")],
    }


def todo_section(P: dict, flights: list[dict], today: str) -> tuple[list[dict], dict]:
    doc = read_json(P["data"] / "todo.json")
    items = doc.get("items") if isinstance(doc, dict) and isinstance(doc.get("items"), list) else []
    vla = [(r["tag"], r["pilot"]["hash"]) for r in sorted(flights, key=lambda r: (r["date"] or "", r["t"]))
           if r["pilot"]["class"] == "vla"]
    out = []
    for it in items:
        if not isinstance(it, dict) or not it.get("id"):
            continue
        st, ev = TODO.effective(it, vla)
        probe = it.get("probe")
        probe_ok = None
        if isinstance(probe, dict) and TODO._safe_rel(probe.get("file")):
            f = P["root"] / probe["file"]
            try:
                probe_ok = f.is_file() and (probe.get("contains", "") in f.read_text(encoding="utf-8", errors="replace"))
            except OSError:
                probe_ok = False
        out.append({
            "id": it["id"], "kind": it.get("kind"), "wp": it.get("wp"), "owner": it.get("owner"),
            "status": it.get("status"), "status_effective": st, "auto_evidence": ev,
            "cards": [c for c in it.get("cards") or [] if isinstance(c, str)],
            "due": it.get("due"), "overdue": bool(it.get("due") and it["due"] < today and st != "done"),
            "branch": it.get("branch"), "closed_by": it.get("closed_by"), "milestone": it.get("milestone"),
            "needs": it.get("needs") or [], "effort": it.get("effort"),
            "probe": {"file": probe.get("file"), "contains": probe.get("contains")} if isinstance(probe, dict) else None,
            "probe_ok": probe_ok, "i18n": {k: {"title": str(((it.get("i18n") or {}).get(k) or {}).get("title", ""))}
                                            for k in ("en", "id", "zh")},
        })
    counts = {s: sum(1 for i in out if i["status_effective"] == s) for s in TODO.STATUSES}
    return out, {"counts": counts, "overdue": sum(1 for i in out if i["overdue"]), "present": doc is not None}


# ------------------------------------------------------------------ inputs, staleness
def inputs(P: dict) -> list[Path]:
    """Every file the build reads (for --check and the server's stale flag)."""
    out = [P["changelog"], P["data"] / "content_overview.json", P["data"] / "updates.json",
           P["data"] / "todo.json", P["root"] / "VERSION"]
    out += sorted((P["data"] / "cards").glob("batch_*.json"))
    out += sorted(P["docs_data"].glob("*.json"))
    out.append(P["root"] / "deploy" / "evidence" / "dev" / "shield_tick_profile.json")
    if P["demo_out"].is_dir():
        for run in P["demo_out"].iterdir():
            if run.is_dir() and not run.name.startswith("_"):
                out += [run / n for n in ("manifest.json", "kpi.json", "metrics.json", "vla_steps.jsonl")]
        out += [f for f in P["demo_out"].glob("*_compare_*") if f.is_file()]
    return [p for p in out if p.name != "state.json"]


def newest_input(P: dict) -> tuple[float, str | None]:
    best, name = 0.0, None
    for p in inputs(P):
        try:
            m = p.stat().st_mtime
        except OSError:
            continue
        if m > best:
            best, name = m, rel(P["root"], p)
    return best, name


def stale_reasons(P: dict, doc: dict | None = None) -> list[str]:
    out = []
    try:
        built = P["out"].stat().st_mtime
    except OSError:
        return ["tracker/data/dashboard.json does not exist: run python tools/build_dashboard.py"]
    newest, name = newest_input(P)
    if newest > built + 1.0:
        out.append(f"{name} is newer than dashboard.json")
    doc = doc if doc is not None else read_json(P["out"])
    meta = (doc or {}).get("meta") or {}
    cl, cr = meta.get("changelog_newest"), meta.get("cards_newest_review")
    if cl and cr and cl > cr:
        out.append(f"the newest CHANGELOG section ({cl}) is newer than the newest card review ({cr})")
    return out


# ------------------------------------------------------------------ build
def write_atomic(path: Path, obj) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix="." + path.name + ".", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as fh:
            json.dump(obj, fh, ensure_ascii=False, indent=1)
            fh.write("\n")
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def build(P: dict, quick: bool = False, today: str | None = None) -> dict:
    today = today or date.today().isoformat()
    _AS_OF.clear()
    media = Media(P["dash"], quick)
    flights = scan_flights(P, media)
    compares = scan_compares(P, media, flights)
    media.save()
    cards = load_cards(P["data"])
    upd = read_json(P["data"] / "updates.json")
    updates = upd.get("entries") if isinstance(upd, dict) and isinstance(upd.get("entries"), list) else []
    overview = read_json(P["data"] / "content_overview.json")
    todo_items, todo_meta = todo_section(P, flights, today)
    wps = wp_section(cards, updates)
    cl = changelog_info(P)
    try:
        version = (P["root"] / "VERSION").read_text(encoding="utf-8").strip()
    except OSError:
        version = None
    newest, _ = newest_input(P)
    status = git(P["root"], "status", "--porcelain", "--untracked-files=no")
    return {
        "meta": {
            "schema": SCHEMA, "generator": "tools/build_dashboard.py",
            "generated": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "mode": "quick" if quick else "full", "today": today,
            "git_head": git(P["root"], "rev-parse", "--short=12", "HEAD"),
            "git_dirty": bool(status) if status is not None else None,
            "version": version, "final": FINAL, "midterm": MIDTERM,
            "changelog_newest": cl["newest_date"], "changelog_newest_heading": cl["newest_heading"],
            "cards_newest_review": wps["newest_review"],
            "content_stale": bool(cl["newest_date"] and wps["newest_review"]
                                  and cl["newest_date"] > wps["newest_review"]),
            "inputs_newest": datetime.fromtimestamp(newest, timezone.utc).isoformat(timespec="seconds") if newest else None,
            "ffprobe": bool(media.ffprobe), "ffmpeg": bool(media.ffmpeg),
            "demo_out_present": P["demo_out"].is_dir(), "flights_total": len(flights),
        },
        "vla": vla_section(flights, todo_items),
        "wps": wps,
        "kpis": kpi_rows(P, flights),
        "flights": flights,
        "compares": compares,
        "todo": {"items": todo_items, **todo_meta},
        "timeline": timeline_section(overview, updates, flights, todo_items, today),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Build the tracker Dashboard's data file.")
    ap.add_argument("--quick", action="store_true", help="no ffprobe, no posters")
    ap.add_argument("--check", action="store_true", help="exit 1 when dashboard.json or the cards are stale")
    ap.add_argument("--demo-out", default=None, help="the recorded runs (default <repo>/demo/out)")
    ap.add_argument("--data-dir", default=None, help="the tracker data folder (default tracker/data)")
    ap.add_argument("--out", default=None, help="write here instead of <data-dir>/dashboard.json")
    ap.add_argument("--root", default=None, help=argparse.SUPPRESS)      # tests: another repository
    a = ap.parse_args(argv)
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass
    root = Path(a.root) if a.root else ROOT
    P = paths(root, a.demo_out, a.data_dir)
    if a.out:
        P["out"] = Path(a.out)
    if a.check:
        why = stale_reasons(P)
        for w in why:
            print("STALE", w)
        print("dashboard.json is up to date" if not why else f"{len(why)} reason(s)")
        return 1 if why else 0
    doc = build(P, quick=a.quick)
    write_atomic(P["out"], doc)
    m, v = doc["meta"], doc["vla"]["counts"]
    measured = sum(1 for k in doc["kpis"] if k["status"] == "measured")
    print(f"{rel(root, P['out'])}: {m['flights_total']} flights (VLA {v['vla']}, VLA configured but "
          f"unverified {v['vla_unverified']}, stub {v['stub']}, "
          f"detector {v['detector']}, none {v['none']}), {len(doc['compares'])} compare videos, "
          f"{measured}/{len(doc['kpis'])} KPIs measured, {len(doc['todo']['items'])} to-do items, "
          f"{doc['wps']['needs_review_total']} cards need review ({m['mode']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
