"""The grant tracker's update log: tracker/data/updates.json.

    python tools/add_update.py list
    python tools/add_update.py check
    python tools/add_update.py new --heading 2026-10-12          # a dated CHANGELOG section
    python tools/add_update.py new --release 0.6.0              # a released CHANGELOG version
    python tools/add_update.py new --commits 2457987 fbb0c84 --date 2026-09-16
    python tools/add_update.py add                              # validate the draft, append it
    python tools/add_update.py extract --out <dir>              # one skeleton per CHANGELOG section

Every change the project makes gets one entry, in English, Indonesian and
Traditional Chinese, so the tracker site can show what changed and when.

VERSIONS. Two numbers, and neither invents a release:
  seq      the site's update number, 1, 2, 3 ... with no gaps, append-only.
  version  a released CHANGELOG heading copied exactly ("0.5.1"), or, for
           unreleased work, "<last release on or before that date>+<date>",
           with ".2", ".3" for a second or third entry on the same date
           ("0.5.1+2026-09-29.2"). The part after "+" is semver build metadata:
           it says "no new release".

WHAT `check` ENFORCES (each rule has a test that breaks it on purpose,
tests/test_add_update.py):
  - seq contiguous from 1, ids unique, dates in seq order;
  - version rule above, against the CHANGELOG's own headings;
  - en, id and zh present for every title, summary, change, highlight, proof;
  - every number in the English text appears in the entry's sources, and the
    Indonesian and Chinese texts carry the same numbers as the English;
  - one `retracted` change per top-level bullet under the source section's
    Retracted headings: a retraction is never merged away;
  - no Simplified-only characters in zh;
  - files, card ids, media and site pages that an entry names exist.

The rules exist because an update log that quietly rounds a number, drops a
retraction or slips into Simplified Chinese would be worse than none.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
UPDATES = ROOT / "tracker" / "data" / "updates.json"
DRAFT = ROOT / "tracker" / "data" / "_update_draft.json"
CHANGELOG = ROOT / "CHANGELOG.md"
VERSION_FILE = ROOT / "VERSION"
CARDS_DIR = ROOT / "tracker" / "data" / "cards"
APP_JS = ROOT / "tracker" / "assets" / "app.js"

LANGS = ("en", "id", "zh")
KINDS = ("release", "work", "site")
CATS = ("added", "changed", "fixed", "retracted", "flown", "found", "limits", "confirmed",
        "tests", "docs", "share")
PAGES = ("overview", "flow", "grant", "board", "evidence", "glossary", "updates")
PROOF_TYPES = ("tests", "flight", "command", "review", "api", "screenshot", "replay")
# Characters that exist only in Simplified Chinese (their Traditional forms
# differ). Not exhaustive; it catches the common slips.
SIMPLIFIED = set("这们个为发说时实现过来对没还会进动问题关样经点种从开间无飞机区则检测图数据视频录试验证"
                 "错误变显应该报设计节约线条结构认识让处当务网页优软长头标级档维护层传输载达态况适轨规")


# ------------------------------------------------------------------ CHANGELOG
@dataclass
class Section:
    heading: str
    level: int                   # 2 for "## [x]", 3 for "### date"
    version: str | None          # released version, or None
    date: str                    # end date, YYYY-MM-DD
    date_start: str | None
    start_line: int
    text: str
    retracted: int = 0           # top-level bullets under Retracted headings
    subsections: list = field(default_factory=list)

    @property
    def sha(self) -> str:
        return hashlib.sha256(self.text.encode("utf-8")).hexdigest()[:12]


_DATE = re.compile(r"(\d{4}-\d{2}-\d{2})")


def parse_changelog(text: str) -> list[Section]:
    """Every '## [x.y.z] - dates' release and every '### YYYY-MM-DD ...'
    section under [Unreleased], in file order."""
    lines = text.splitlines()
    heads = []
    for i, ln in enumerate(lines):
        m = re.match(r"^## \[(\d+\.\d+\.\d+)\]\s*[—-]\s*(.+)$", ln)
        if m:
            dates = _DATE.findall(m.group(2))
            heads.append((i, 2, m.group(1), dates))
            continue
        if re.match(r"^## \[", ln):
            heads.append((i, 2, None, []))          # [Unreleased]: a boundary only
            continue
        m = re.match(r"^### (\d{4}-\d{2}-\d{2})", ln)
        if m:
            heads.append((i, 3, None, [m.group(1)]))
    out = []
    for k, (i, level, ver, dates) in enumerate(heads):
        if not dates:
            continue
        end = len(lines)
        for j, lv, _v, _d in heads[k + 1:]:
            if lv <= level or level == 3:
                end = j
                break
        body = lines[i:end]
        sub_prefix = "#### " if level == 3 else "### "
        subs, cur, retracted = [], None, 0
        for ln in body[1:]:
            if ln.startswith(sub_prefix):
                cur = ln[len(sub_prefix):].strip()
                subs.append(cur)
            elif cur and cur.lower().startswith("retract") and ln.startswith("- "):
                retracted += 1
        out.append(Section(heading=lines[i].strip(), level=level, version=ver,
                           date=dates[-1], date_start=dates[0] if len(dates) > 1 else None,
                           start_line=i + 1, text="\n".join(body), retracted=retracted,
                           subsections=subs))
    return out


def releases(sections: list[Section]) -> list[tuple[str, str]]:
    return sorted(((s.version, s.date) for s in sections if s.version), key=lambda r: r[1])


def base_version(date: str, rels: list[tuple[str, str]]) -> str | None:
    best = None
    for v, d in rels:
        if d <= date:
            best = v
    return best


def find_section(sections: list[Section], heading: str) -> Section | None:
    for s in sections:
        if s.heading == heading:
            return s
    return None


# ------------------------------------------------------------------ numbers
_NUM = re.compile(r"(?<![\w.])(\d[\d,]*(?:\.\d+)?)")


def numbers(text: str) -> set[str]:
    """Numeric tokens, normalised: thousands commas dropped, leading zeros of
    integers dropped, trailing '.0' kept as written."""
    out = set()
    for raw in _NUM.findall(text or ""):
        t = raw.rstrip(",.")
        if re.fullmatch(r"\d{1,3}(,\d{3})+(\.\d+)?", t):
            t = t.replace(",", "")
        elif "," in t:                        # "3,4" style or a list: split
            for part in t.split(","):
                if part:
                    out.add(part.lstrip("0") or "0")
            continue
        if "." in t:
            out.add(t)
        else:
            out.add(t.lstrip("0") or "0")
    return out


def entry_texts(e: dict, lang: str) -> list[str]:
    """Every user-visible string of one entry in one language."""
    out = []
    t = (e.get("i18n") or {}).get(lang) or {}
    out += [t.get("title", ""), t.get("summary", "")]
    for c in e.get("changes", []):
        out.append(((c.get("i18n") or {}).get(lang)) or "")
    for h in e.get("highlights", []):
        out.append(str(h.get("value", "")))
        out.append(((h.get("i18n") or {}).get(lang)) or "")
    for p in e.get("proof", []):
        out.append(str(p.get("value", "")))
        out.append(((p.get("i18n") or {}).get(lang)) or "")
    return out


def strip_code(s: str) -> str:
    """Numbers inside `code` (paths, ids, versions) are identifiers, not
    claims; they are checked as references instead."""
    return re.sub(r"`[^`]*`", " ", s)


# ------------------------------------------------------------------ context
@dataclass
class Ctx:
    root: Path
    sections: list[Section]
    cards: set[str]
    media_ids: set[str]
    version_file: str | None
    git_text: dict = field(default_factory=dict)       # sha -> message + stat


def load_ctx(root: Path = ROOT) -> Ctx:
    sections = parse_changelog((root / "CHANGELOG.md").read_text(encoding="utf-8"))
    cards = set()
    for f in sorted((root / "tracker" / "data" / "cards").glob("batch_*.json")):
        try:
            cards |= {c["id"] for c in json.loads(f.read_text(encoding="utf-8"))}
        except (OSError, ValueError, KeyError):
            pass
    media = set()
    app = root / "tracker" / "assets" / "app.js"
    if app.is_file():
        media = set(re.findall(r'(\w+): \["(?:img|video)"', app.read_text(encoding="utf-8")))
    try:
        ver = (root / "VERSION").read_text(encoding="utf-8").strip()
    except OSError:
        ver = None
    return Ctx(root=root, sections=sections, cards=cards, media_ids=media, version_file=ver)


def git_message(ctx: Ctx, sha: str) -> str:
    if sha not in ctx.git_text:
        try:
            r = subprocess.run(["git", "show", "-s", "--format=%B", sha], cwd=ctx.root,
                               capture_output=True, text=True, encoding="utf-8", timeout=20)
            st = subprocess.run(["git", "show", "--stat", "--format=", sha], cwd=ctx.root,
                                capture_output=True, text=True, encoding="utf-8", timeout=20)
            ctx.git_text[sha] = (r.stdout or "") + "\n" + (st.stdout or "")
        except (OSError, subprocess.SubprocessError):
            ctx.git_text[sha] = ""
    return ctx.git_text[sha]


def source_text(e: dict, ctx: Ctx) -> tuple[str, Section | None]:
    """All the text an entry may take numbers from."""
    src = e.get("source") or {}
    sec = find_section(ctx.sections, src.get("heading", "")) if src.get("heading") else None
    parts = [sec.text if sec else ""]
    for f in src.get("extra", []):
        p = ctx.root / f
        if p.is_file():
            parts.append(p.read_text(encoding="utf-8", errors="replace"))
    for sha in src.get("commits", []):
        parts.append(git_message(ctx, sha))
    parts.append(e.get("version", "") + " " + e.get("date", "") + " " + (e.get("date_start") or ""))
    return "\n".join(parts), sec


# ------------------------------------------------------------------ check
def _vt(v: str) -> tuple:
    try:
        return tuple(int(x) for x in v.split("."))
    except ValueError:
        return (0,)


def check(doc: dict, ctx: Ctx, partial: bool = False) -> list[str]:
    errs = []
    entries = doc.get("entries")
    if doc.get("schema") != 1 or not isinstance(entries, list):
        return ["updates.json must be {\"schema\": 1, \"entries\": [...]}"]
    rels = releases(ctx.sections)
    rel_dates = dict(rels)
    ids, prev_date = set(), ""
    for i, e in enumerate(entries):
        tag = f"#{e.get('seq', '?')}"
        if not partial and e.get("seq") != i + 1:
            errs.append(f"{tag}: seq must be {i + 1} (contiguous from 1, in file order)")
        if e.get("id") in ids or not e.get("id"):
            errs.append(f"{tag}: id missing or duplicate ({e.get('id')})")
        ids.add(e.get("id"))
        date = e.get("date", "")
        if not re.fullmatch(r"\d{4}-\d{2}-\d{2}", date):
            errs.append(f"{tag}: date must be YYYY-MM-DD")
        elif date < prev_date and not partial:
            errs.append(f"{tag}: date {date} is before the previous entry's {prev_date}")
        prev_date = max(prev_date, date)
        kind = e.get("kind")
        if kind not in KINDS:
            errs.append(f"{tag}: kind must be one of {KINDS}")
        ver = e.get("version", "")
        if kind == "release":
            if ver not in rel_dates:
                errs.append(f"{tag}: release version {ver} is not a CHANGELOG release heading")
            elif rel_dates[ver] != date:
                errs.append(f"{tag}: release {ver} is dated {rel_dates[ver]} in CHANGELOG, not {date}")
        else:
            base = base_version(date, rels)
            m = re.fullmatch(r"(\d+\.\d+\.\d+)\+(\d{4}-\d{2}-\d{2})(?:\.(\d+))?", ver)
            if not m:
                errs.append(f"{tag}: version {ver} must be <release>+<date>[.N] for unreleased work")
            else:
                if m.group(1) != base:
                    errs.append(f"{tag}: base version {m.group(1)} should be {base} (last release on or before {date})")
                if m.group(2) != date:
                    errs.append(f"{tag}: version date {m.group(2)} differs from entry date {date}")
            if e.get("base_version") not in (None, base):
                errs.append(f"{tag}: base_version {e.get('base_version')} should be {base}")
        if kind == "release" and ctx.version_file and _vt(ver) > _vt(ctx.version_file):
            errs.append(f"{tag}: release {ver} is newer than VERSION ({ctx.version_file})")

        # three languages everywhere
        t = e.get("i18n") or {}
        for lang in LANGS:
            x = t.get(lang) or {}
            if not (x.get("title") or "").strip() or not (x.get("summary") or "").strip():
                errs.append(f"{tag}: {lang} title/summary missing")
            if lang != "en" and ("TODO" in (x.get("title", "") + x.get("summary", ""))):
                errs.append(f"{tag}: {lang} still has TODO text")
        for n, c in enumerate(e.get("changes", [])):
            if c.get("cat") not in CATS:
                errs.append(f"{tag}: change {n + 1} has unknown category {c.get('cat')}")
            for lang in LANGS:
                if not ((c.get("i18n") or {}).get(lang) or "").strip():
                    errs.append(f"{tag}: change {n + 1} has no {lang} text")
            for cid in c.get("cards", []):
                if cid not in ctx.cards:
                    errs.append(f"{tag}: change {n + 1} names unknown card {cid}")
        for group in ("highlights", "proof"):
            for n, h in enumerate(e.get(group, [])):
                for lang in LANGS:
                    if not ((h.get("i18n") or {}).get(lang) or "").strip():
                        errs.append(f"{tag}: {group} {n + 1} has no {lang} text")
                if group == "proof" and h.get("type") not in PROOF_TYPES:
                    errs.append(f"{tag}: proof {n + 1} has unknown type {h.get('type')}")

        # Traditional Chinese only
        zh = " ".join(entry_texts(e, "zh"))
        bad = sorted({ch for ch in zh if ch in SIMPLIFIED})
        if bad:
            errs.append(f"{tag}: zh contains Simplified characters: {''.join(bad)}")

        # numbers: English against the sources, other languages against English
        src, sec = source_text(e, ctx)
        if e.get("source", {}).get("heading") and sec is None:
            errs.append(f"{tag}: source heading not found in CHANGELOG: {e['source']['heading'][:60]}")
        src_nums = numbers(src)
        en_nums = set()
        for s in entry_texts(e, "en"):
            en_nums |= numbers(strip_code(s))
        missing = sorted(en_nums - src_nums, key=lambda x: (len(x), x))
        if missing and not e.get("numbers_ok"):
            errs.append(f"{tag}: English numbers not found in the sources: {', '.join(missing[:12])}")
        for lang in ("id", "zh"):
            ln = set()
            for s in entry_texts(e, lang):
                ln |= numbers(strip_code(s))
            if ln != en_nums:
                diff = sorted(ln ^ en_nums)[:10]
                errs.append(f"{tag}: {lang} numbers differ from English: {', '.join(diff)}")

        # retractions are never merged away
        if sec is not None:
            n_ret = sum(1 for c in e.get("changes", []) if c.get("cat") == "retracted")
            if n_ret != sec.retracted:
                errs.append(f"{tag}: {n_ret} retracted change(s), but the source has {sec.retracted} retracted bullet(s)")

        # references
        for f in e.get("files", []):
            if not (ctx.root / f).exists():
                errs.append(f"{tag}: file does not exist: {f}")
        for cid in e.get("cards", []):
            if cid not in ctx.cards:
                errs.append(f"{tag}: unknown card {cid}")
        for m in e.get("media", []):
            if "/" in m:
                kind_dir, _, rest = m.partition("/")
                if kind_dir not in ("img", "video", "share") or not (ctx.root / "docs" / kind_dir / rest).is_file():
                    errs.append(f"{tag}: media not found: {m}")
            elif m not in ctx.media_ids:
                errs.append(f"{tag}: unknown media id {m}")
        for pg in e.get("pages", []):
            if pg not in PAGES:
                errs.append(f"{tag}: unknown page {pg}")
    return errs


# ------------------------------------------------------------------ io
def load_doc(path: Path = UPDATES) -> dict:
    if not path.is_file():
        return {"schema": 1, "entries": []}
    return json.loads(path.read_text(encoding="utf-8"))


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


def next_version(doc: dict, date: str, ctx: Ctx, kind: str, release: str | None = None) -> str:
    if kind == "release":
        return release or ""
    base = base_version(date, releases(ctx.sections)) or "0.0.0"
    same = [e for e in doc.get("entries", []) if e.get("kind") != "release" and e.get("date") == date]
    return f"{base}+{date}" + (f".{len(same) + 1}" if same else "")


def skeleton(doc: dict, ctx: Ctx, *, heading: str | None = None, release: str | None = None,
             commits: list[str] | None = None, date: str | None = None, kind: str | None = None) -> dict:
    """A draft entry with everything mechanical filled in and TODO text."""
    sec = None
    if release:
        sec = next((s for s in ctx.sections if s.version == release), None)
    elif heading:
        sec = next((s for s in ctx.sections if s.heading.startswith("### " + heading) or s.heading == heading), None)
    if (release or heading) and sec is None:
        raise SystemExit(f"no CHANGELOG section for {release or heading}")
    kind = kind or ("release" if release else "work")
    date = date or (sec.date if sec else None)
    if not date:
        raise SystemExit("--date is required when there is no CHANGELOG section")
    seq = len(doc.get("entries", [])) + 1
    ver = next_version(doc, date, ctx, kind, release)
    src = {"file": "CHANGELOG.md", "heading": sec.heading, "sha256": sec.sha} if sec else {}
    if commits:
        src["commits"] = commits
    text = sec.text if sec else "\n".join(git_message(ctx, c) for c in commits or [])
    paths = sorted({p for p in re.findall(r"`([\w./-]+\.(?:py|md|yaml|json|js|ps1|sh|css|html))`", text)
                    if (ctx.root / p).exists()})
    cards = sorted({c for c in ctx.cards if any(p in json.dumps(card_evidence(ctx, c)) for p in paths)})[:12]
    return {
        "seq": seq, "id": f"u{seq}-{date}", "kind": kind, "version": ver,
        "base_version": None if kind == "release" else base_version(date, releases(ctx.sections)),
        "date": date, "date_start": sec.date_start if sec else None, "backfilled": False,
        "source": src, "tags": [], "highlights": [], "chart": None,
        "changes": [], "files": paths[:20], "cards": cards, "pages": [], "media": [], "proof": [],
        "i18n": {lang: {"title": "TODO", "summary": "TODO"} for lang in LANGS},
        "_hints": {"subsections": sec.subsections if sec else [], "retracted_bullets": sec.retracted if sec else 0,
                   "numbers": sorted(numbers(text), key=lambda x: (len(x), x))[:200]},
    }


_EVID: dict = {}


def card_evidence(ctx: Ctx, cid: str) -> list:
    if not _EVID:
        for f in sorted((ctx.root / "tracker" / "data" / "cards").glob("batch_*.json")):
            for c in json.loads(f.read_text(encoding="utf-8")):
                _EVID[c["id"]] = c.get("evidence", [])
    return _EVID.get(cid, [])


# ------------------------------------------------------------------ cli
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Maintain the tracker's update log.")
    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("list")
    c = sp.add_parser("check")
    c.add_argument("--file", default=None, help="check this file instead of updates.json")
    c.add_argument("--partial", action="store_true", help="a slice of the log: skip the seq/date-order rules")
    n = sp.add_parser("new")
    g = n.add_mutually_exclusive_group(required=True)
    g.add_argument("--heading", help="date of a '### YYYY-MM-DD' CHANGELOG section")
    g.add_argument("--release", help="a released CHANGELOG version, e.g. 0.6.0")
    g.add_argument("--commits", nargs="+", help="commit shas, for work with no CHANGELOG section")
    n.add_argument("--date")
    n.add_argument("--kind", choices=KINDS)
    a = sp.add_parser("add")
    a.add_argument("--from", dest="src", default=str(DRAFT))
    x = sp.add_parser("extract")
    x.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")   # Windows consoles
    except (AttributeError, ValueError):
        pass

    ctx = load_ctx()
    doc = load_doc(Path(args.file)) if getattr(args, "file", None) else load_doc()
    if args.cmd == "list":
        for e in doc["entries"]:
            print(f"#{e['seq']:<3} {e['date']}  {e['kind']:<7} {e['version']:<22} {e['i18n']['en']['title']}")
        return 0
    if args.cmd == "check":
        errs = check(doc, ctx, partial=getattr(args, "partial", False))
        for m in errs:
            print("ERROR", m)
        print(f"{len(doc['entries'])} entries, {len(errs)} problem(s)")
        return 1 if errs else 0
    if args.cmd == "new":
        d = skeleton(doc, ctx, heading=args.heading, release=args.release, commits=args.commits,
                     date=args.date, kind=args.kind)
        write_atomic(DRAFT, d)
        print(f"draft #{d['seq']} ({d['version']}) -> {DRAFT.relative_to(ROOT)}; fill in en/id/zh, then: add")
        return 0
    if args.cmd == "add":
        d = json.loads(Path(args.src).read_text(encoding="utf-8"))
        d.pop("_hints", None)
        trial = {"schema": 1, "entries": doc["entries"] + [d]}
        errs = [m for m in check(trial, ctx) if m.startswith(f"#{d.get('seq')}:") or m.startswith("#?")]
        if errs:
            for m in errs:
                print("ERROR", m)
            return 1
        write_atomic(UPDATES, trial)
        print(f"Added update #{d['seq']} ({d['version']})")
        return 0
    if args.cmd == "extract":
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        for s in ctx.sections:
            d = skeleton({"entries": []}, ctx, release=s.version) if s.version else \
                skeleton({"entries": []}, ctx, heading=s.heading)
            name = (s.version or s.heading[4:].split(" ")[0] + ("-evening" if "evening" in s.heading else "")).replace(" ", "_")
            write_atomic(out / f"{name}.json", dict(d, _source_text=s.text))
        print(f"{len(ctx.sections)} skeletons -> {out}")
        return 0
    return 2


if __name__ == "__main__":
    sys.exit(main())
