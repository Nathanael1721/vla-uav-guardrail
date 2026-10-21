# Grant tracker (local website)

A local site that explains the project, shows how it works, and tracks every grant item on a kanban board. It is in English, Indonesian and Traditional Chinese, and it also carries the project's versioned update log.

## Run
- Double-click `tracker\start_tracker.bat`. If the tracker already runs, it only opens the browser; otherwise it refreshes the Dashboard data (`tools/build_dashboard.py --quick`) and starts the server with `--open`, which opens the browser once the port is bound. Or
- run `python tracker/serve.py --open` (or without `--open`, then open http://127.0.0.1:8765/).
- `--demo-out <folder>` serves recorded runs from another folder (for example from a git worktree, whose `demo/out` is empty), `--no-rebuild` stops the background rebuild, `--live-dir` reads heartbeats from another folder.
- Starting it with Windows (a shortcut in the Startup folder, or an "At log on" scheduled task) is a persistent system change: set it up yourself if you want it.

The server uses the Python standard library only and listens on 127.0.0.1, so it is reachable from this PC only. That does not stop a web page open in your browser from sending your browser to it, so every request must also name the server itself as its Host (`127.0.0.1:<port>` or `localhost:<port>`), any Origin must be the server's own, and saving the board must send `application/json`; anything else gets a 403 (the module docstring of `tracker/serve.py` says why). Open the site at `127.0.0.1` or `localhost`: another name for this PC is refused. Add `?lang=en`, `?lang=id` or `?lang=zh` to the address to open the site in that language.

## Pages
| Page | Content |
|---|---|
| Dashboard (the default page) | The progress monitor. Days to 30 Nov, git head, when the data was built, and a red "data stale" badge when an input is newer than the data (an amber one when the newest CHANGELOG section is newer than the newest card review). **VLA in the loop:** the last flight the VLA flew (none yet), what flew every recorded run (VLA / VLA configured but unverified / StubVLA / OWL-ViT recorded as the VLA / hand-written controller), the latest pilot per rail, and the VLA-pilot milestones with a "code found" probe. **Running now:** live heartbeats of flights, harness runs, training and workflows, a GPU row, and a red banner when a flight and a workflow run at the same time. **Work-package progress:** verdict bars, your board, open gate items, cards to re-review. **Grant KPIs:** target, value, null baseline, grade, date and source file; green only when KPI-grade, on target and better than its null; "not measured" is never shown as 0. **Latest flights:** posters and the H.264 videos (the MPEG-4 Part 2 originals are never linked), KPI grade recomputed from the topology. **Timeline** and the **open to-do list**. The live panel refreshes every 5 s and the data every 60 s while the page is visible. |
| Overview | What the project is, days left to the 30 Nov gate, progress by audit verdict and by your board, mental model, work packages, delivery gates, open PI decisions, latest update. |
| How it works | Clickable diagrams: the city pipeline as built; the grant's architecture (green built, amber partly built, red not built); the five system figures drawn from code on 14 Oct (end-to-end flow, components, topologies, one Shield tick, evidence chain; click to enlarge); one Shield tick, ours against the grant's; an annotated policy HUD. The overview shows the end-to-end figure. |
| Grant | Each work package as the grant states it, the KPI table, the dev / hil / flight topologies, and the grant timeline with a line for today. |
| Checklist | 148 cards from the 2026-10-05 grant audit, in five columns: Needs PI decision, To do, In progress, Done, Waived. Drag a card between columns, or open it. An open card shows what exists, what is missing, why it matters, next steps, the grant quote, evidence and examples, and your notes. |
| Examples | Images and videos, each with what it shows and what it does not show (the recorded runs of 13-14 Oct first), the system figures, and a flight-results table with a note on where each number comes from. |
| Glossary | Terms used on the site. |
| Updates | **Version log:** every update to the project, newest first, with a timeline, what changed (retractions in red), related cards, proof and source. **Board activity:** your card moves and note edits, with times. |

## Versions
- `seq` (Update #N) is the site's update number: 1, 2, 3… with no gaps.
- `version` copies a CHANGELOG release, such as `0.5.1`. Unreleased work is written `<release>+<date>`, for example `0.5.1+2026-10-03`, or `0.5.1+2026-10-05.2` for a second entry on the same day. The `+` part means "no new release".

## How to post an update
1. Write the change in `CHANGELOG.md`, under a `### YYYY-MM-DD — title` section (or `## [x.y.z]` for a release).
2. Run `python tools/add_update.py new --heading YYYY-MM-DD`. This writes a draft, `tracker/data/_update_draft.json`, with the number, version and source filled in.
3. Fill in the title, summary and changes in `en`, `id` and `zh` (Traditional Chinese). Copy numbers exactly as the CHANGELOG has them.
4. Run `python tools/add_update.py add` (it validates, then appends), then `python tools/add_update.py check`. An update can show a recorded flight's video or image: add `run/<tag>/<file>.mp4` (or a top-level compare video, `run/<name>_phone.mp4`) to its `media`; only media files under `demo/out` are accepted, never a log. `pages` may name `dashboard`.
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

## The Dashboard's data
- `python tools/build_dashboard.py` writes `tracker/data/dashboard.json` (with ffprobe and ffmpeg: codecs read from the files, posters in `tracker/data/dash/posters/`). `--quick` skips both (what the launcher and the server's background rebuild run; a flight is never slowed by posters). `--check` exits 1 when the file is older than an input or the cards are older than the CHANGELOG.
- Every number is read from `demo/out/*/{manifest,kpi,metrics}.json`, `docs/data/*.json`, the cards, `updates.json`, `todo.json` and `CHANGELOG.md`, and carries its source (`path#/json/pointer`) and date. Targets are cited to the grant's locked KPI tables. The builder never opens `state.json`.
- The server answers `GET /api/dashboard` with the file plus a staleness block, and rebuilds it (`--quick`) in the background when an input is newer, at most every 30 s, never while a flight runs (its heartbeat is fresh, or its process is alive between beats). If the server stops answering, the page keeps the last data under a red "server unreachable" badge, never "Data current". Tested in `tests/test_build_dashboard.py`.

## Live status ("Running now")
- Flights (`demo/follow_vlm.py`, `demo/pas_ardupilot/perception_node.py`) and harness runs (`experiments/sweep_scenarios.py` from the command line) write a heartbeat by themselves through `tools/live_status.py`. Its `beat()` only stores numbers in memory; a background thread writes at most once a second to `%LOCALAPPDATA%\vla-guardrail\live` (outside OneDrive; `VLAGUARD_LIVE_DIR` moves it, `VLAGUARD_LIVE_OFF=1` turns it off), and every failure is swallowed, so a flight is never slowed or stopped by it. A flight beats at each slow step outside its loop too, with a phase (starting, airborne, landing) and a short detail, and its row ends when the run ends (follow_vlm with its exit code; a Ctrl+C reads "stopped").
- A row stays "running" while its process is alive, also between beats (shown as "alive, no beat" after 30 s, for at most an hour, because Windows reuses process ids); a gone process reads "stopped unexpectedly". So the rebuild deferral and the flight + workflow banner hold for the whole flight, scene load and landing included.
- A workflow or any other job reports with `python tools/live_status.py start <name> --kind workflow`, optionally `beat <name> --progress 0.5`, and `stop <name>`. `python tools/live_status.py list` prints what runs.
- `GET /api/live` returns only whitelisted fields (no log content, no paths: a run folder shows as its `demo/out` tag) and one `nvidia-smi` reading, cached 5 s (while a flight runs: every 30 s, without the process list). A failed read says only that it failed, never "Nothing is running". Tested in `tests/test_live_status.py` and `tests/test_tracker_serve.py`.

## To-do list
- `tracker/data/todo.json` (local, gitignored): id, title in en / id / zh, owner (`user`, `agent` or `pi`: a role, no names), status (`open`, `doing`, `blocked`, `done`), area, cards, due date, and an optional code probe. Seeded on 21 Oct with the VLA-pilot milestones and the open contract items and bugs.
- `python tools/todo.py list` / `check` / `set <id> --status doing` / `close <id> --commit <sha>` (or `--changelog "<heading>"`) / `add ...`. `check` refuses a missing language, Simplified Chinese, an owner that is not a role, an unknown card, and an item marked done with no commit, CHANGELOG heading or run behind it.
- The "first VLA flight" milestone (VLA-05) shows as done on its own as soon as a run is a VLA flight (`python tools/todo.py sync` writes that into the file). Tested in `tests/test_todo.py`.
- A run is a VLA flight only when its manifest names a real VLA **and** its own records show the VLA server answered: in `vla_steps.jsonl` (or `metrics.json`'s pilot block) at least 10 answers the model parsed and the tracker accepted, each with the server's `weights_sha256` (one digest, not a placeholder), for an allow-listed model, and at least 10 loop ticks flown toward those steps. The manifest's `vla_model_hash` is computed on the pilot side from the configured model id, so on its own it only says what was configured: such a run shows as "VLA configured, no server answers recorded", with the reason, and closes nothing.

## Files
| Path | What it is |
|---|---|
| `tracker/serve.py` | The server. Tested in `tests/test_tracker_serve.py`. |
| `tracker/data/cards/batch_*.json` | Card content in three languages, generated from `docs/data/grant_audit_2026-10-05.json` and then reviewed. |
| `tracker/data/content_overview.json`, `content_flow.json` | Page content. |
| `tracker/data/updates.json` | The update log. |
| `tracker/data/state.json` | Your board (columns, notes, history). Personal and gitignored. Back it up with **Export** on the Checklist page. If two tabs are open, the last one to save wins. |
| `tracker/data/dashboard.json`, `tracker/data/dash/` | The Dashboard's data, its posters and its codec cache: generated by `tools/build_dashboard.py`, safe to delete and rebuild. |
| `tracker/data/todo.json` | The to-do list (`tools/todo.py`). Gitignored like the rest of `tracker/data/`, so it has no history: back it up with the folder. |
| `tools/build_dashboard.py`, `tools/live_status.py`, `tools/todo.py` | The Dashboard's builder, heartbeat writer/reader and to-do tool (standard library only). |
| `/media/...` | Served read-only from `docs/img`, `docs/video` and `docs/share`, and `/media/run/...` from `demo/out` (videos and images only, so the logs beside them stay unserved). Videos seek (HTTP Range). A media key in `tracker/assets/app.js` `MEDIA` is `[folder, file]`, with folder `img`, `video`, `share` or `run`. The recorded runs' full-resolution `_demo.mp4` files are MPEG-4 Part 2, which browsers do not play, so the site links their H.264 `_share` / `_phone` copies and the compare video. |
