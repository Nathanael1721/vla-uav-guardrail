# Grant tracker (local website)

A local site that explains the project, shows how it works, and tracks every grant item on a kanban board. It is in English, Indonesian and Traditional Chinese, and it also carries the project's versioned update log.

## Run
- Double-click `tracker\start_tracker.bat`, or
- run `python tracker/serve.py` and open http://127.0.0.1:8765/

The server uses the Python standard library only and listens on 127.0.0.1, so it is reachable from this PC only. Add `?lang=en`, `?lang=id` or `?lang=zh` to the address to open the site in that language.

## Pages
| Page | Content |
|---|---|
| Overview | What the project is, days left to the 30 Nov gate, progress by audit verdict and by your board, mental model, work packages, delivery gates, open PI decisions, latest update. |
| How it works | Clickable diagrams: the city pipeline as built; the grant's architecture (green built, amber partly built, red not built); one Shield tick, ours against the grant's; an annotated policy HUD. |
| Grant | Each work package as the grant states it, the KPI table, the dev / hil / flight topologies, and the grant timeline with a line for today. |
| Checklist | 148 cards from the 2026-10-05 grant audit, in five columns: Needs PI decision, To do, In progress, Done, Waived. Drag a card between columns, or open it. An open card shows what exists, what is missing, why it matters, next steps, the grant quote, evidence and examples, and your notes. |
| Examples | Images and videos, each with what it shows and what it does not show, plus a flight-results table with a note on where each number comes from. |
| Glossary | Terms used on the site. |
| Updates | **Version log:** every update to the project, newest first, with a timeline, what changed (retractions in red), related cards, proof and source. **Board activity:** your card moves and note edits, with times. |

## Versions
- `seq` (Update #N) is the site's update number: 1, 2, 3… with no gaps.
- `version` copies a CHANGELOG release, such as `0.5.1`. Unreleased work is written `<release>+<date>`, for example `0.5.1+2026-10-03`, or `0.5.1+2026-10-05.2` for a second entry on the same day. The `+` part means "no new release".

## How to post an update
1. Write the change in `CHANGELOG.md`, under a `### YYYY-MM-DD — title` section (or `## [x.y.z]` for a release).
2. Run `python tools/add_update.py new --heading YYYY-MM-DD`. This writes a draft, `tracker/data/_update_draft.json`, with the number, version and source filled in.
3. Fill in the title, summary and changes in `en`, `id` and `zh` (Traditional Chinese). Copy numbers exactly as the CHANGELOG has them.
4. Run `python tools/add_update.py add` (it validates, then appends), then `python tools/add_update.py check`.
5. Reload the site. The header dot shows that there is something new.

`check` refuses:
- a gap in the numbering;
- an invented release;
- a missing language;
- Simplified Chinese;
- a number the source does not contain, or one that differs between languages;
- a retraction merged into another item;
- links to files, cards, media or pages that do not exist.

The rules are tested in `tests/test_add_update.py`.

## Files
| Path | What it is |
|---|---|
| `tracker/serve.py` | The server. Tested in `tests/test_tracker_serve.py`. |
| `tracker/data/cards/batch_*.json` | Card content in three languages, generated from `docs/data/grant_audit_2026-10-05.json` and then reviewed. |
| `tracker/data/content_overview.json`, `content_flow.json` | Page content. |
| `tracker/data/updates.json` | The update log. |
| `tracker/data/state.json` | Your board (columns, notes, history). Personal and gitignored. Back it up with **Export** on the Checklist page. If two tabs are open, the last one to save wins. |
| `/media/...` | Served read-only from `docs/img`, `docs/video` and `docs/share`. |
