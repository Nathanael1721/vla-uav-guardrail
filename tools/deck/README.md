# Midterm deck build

Builds `docs/VLA-Guardrail-Midterm-Aug2026.pptx` on the nathan-deck design
system (Poppins, teal `#249DB2`, rounded title pills, page badges).

## Run

```bash
npm install          # pptxgenjs, react, react-dom, react-icons, sharp
node build_midterm_deck.js
```

The build reads `demo/out/<tag>/metrics.json`, `flight_log.jsonl`,
`view/recorder.json` and `docs/data/chase_resolution_study.json`, then repairs
and audits the package before it will report success.

## Two rules the script enforces

**Every number is read from the artefacts at build time.** Nothing is typed into
a slide. `tools/build_report_results.py` generates section 6 of the written
report from the same files, so the deck and the report cannot disagree with each
other or with the flight logs.

**No video is embedded.** The previous deck reached **486 MB** because six raw
`.mp4` files were placed inside it, which is unusable as an email attachment.
Videos ship alongside and are referenced by filename; the build refuses any deck
over 10 MB.

## Why there is a repair step

pptxgenjs 4.0.1 emits a package PowerPoint will not open, and PowerPoint reports
this only as *"PowerPoint could not open the file"* — no part name, no slide
number, no reason. Three separate defects were found by bisecting the deck one
slide at a time:

| Defect | Effect |
|---|---|
| `[Content_Types].xml` declares overrides for `slideMaster2..7` and layouts that were never written | Package refused |
| ZIP directory entries (`ppt/`, `ppt/media/`, …) which are not OPC parts | Non-conformant |
| A `card()` shorter than its own header produced `<a:ext cy="-100584">` | Package refused |

The first two are repaired by `tools/fix_pptx_package.py`. The third is now
impossible: `card()` throws at build time with the card title and the height it
needs, and `tools/audit_pptx.py` fails the build if any negative extent, dangling
override, dangling relationship or directory entry survives.

This is easy to misdiagnose, because a deck PowerPoint has ever opened and
re-saved comes back normalised. An older deck in the same folder therefore opens
while a freshly generated one does not, which points suspicion at the slide
content rather than the packaging.

## Export to PDF

```bash
powershell -File ../office_to_pdf.ps1 -Path ../../docs/VLA-Guardrail-Midterm-Aug2026.pptx
```

## The explainer pack (14 Oct 2026)

`docs/share/2026-10-14-Explainer/` (gitignored) is written by two builders here.
Both check every number they print against its source file and refuse a build
whose number has drifted.

```bash
# the five figures (end-to-end flowchart, components, topologies, the Shield
# per tick, the evidence chain) as SVG into docs/img/explainer_*
python tools/deck/build_explainer_figures.py
python tools/deck/build_explainer_figures.py --png      # also the PNGs, through Edge headless
python tools/deck/build_explainer_figures.py --out DIR  # somewhere else

# the deck and the pack
node tools/deck/build_explainer_deck.js                 # the pack, fonts embedded through PowerPoint
node tools/deck/build_explainer_deck.js --render --pdf  # + every slide as PNG (_build/render/), + the PDF
node tools/deck/build_explainer_deck.js --check-facts-only   # check the facts and exit
node tools/deck/build_explainer_deck.js --out DIR --no-office  # a test build: no PowerPoint
```

| Flag of `build_explainer_deck.js` | What it does |
|---|---|
| `--render` | Every slide to `_build/render/slide_NN.png` through PowerPoint, to review by eye. |
| `--pdf` | The PDF beside the deck (`tools/office_to_pdf.ps1`). |
| `--out DIR` | Write the deck and `_build/` to DIR; only the pack build writes into the pack. |
| `--no-office` | No PowerPoint: no font embedding, render or PDF (what the test uses). |
| `--check-facts-only` | Check every fact against its file, print the count, exit 1 on a mismatch. |
| `--allow-unverified` | Let the pack build through when a fact's source file is absent (flight output under `demo/out` on another machine); test builds allow it anyway. |
| `--still PATH` | The city-video still (default: the pack's `figures/` copy, extracted with ffmpeg from the video when it is on disk). |
| `--onoff-on PATH --onoff-off PATH` | The ArduPilot Shield-on and Shield-off stills (default: `figures/ardupilot_shield_on.png` and `_off.png`). |

A missing optional image drops its slide and nothing else. Every slide's text
and notes go through `tools/check_claims.py` and a hedge-word scan; a hit stops
the build. The pack build also copies the figures into the pack, writes the
ten-number tables of `CHEATSHEET-EN.md` and `CHEATSHEET-ID.md` (between their
`numbers` markers) and writes the pack's `README.md`, which lists every file and
the videos with their length and size. `EXPLAINER_REPO` points the fact check at
another tree and `EXPLAINER_PY` names the Python that runs `check_claims` and
the package audit; the test uses both. Tests: `tests/test_explainer_figures.py`,
`tests/test_explainer_deck.py`. Where pandoc and Edge are installed, the deck
test also prints both cheatsheets with a fixed recipe (`SHEET_CSS` in the test:
pandoc's HTML, Edge headless, A4) and checks that each is one page.

## Videos that go beside a deck

No video is embedded in a deck (see above). Two tools write the videos:

- `python tools/make_demo_video.py --tag <tag> --height 720` turns a run recorded
  with `--save-view` (`demo/follow_vlm.py`, or `scripts/run_pas_ardupilot.ps1
  -Step node -SaveView` on the ArduPilot rail) into `demo/out/<tag>/<tag>_demo.mp4`,
  the drone's camera with the policy HUD beside the chase camera, at the rate
  the recorder measured (`view/recorder.json`).
- `python tools/make_compare_video.py --a <tag A> --b <tag B>` puts two recorded
  runs in one video, by default "SHIELD ON" stacked above "SHIELD OFF - control",
  both from take-off, each half at its own run's measured rate; one that ends
  first holds its last frame, marked ENDED.

| Flag of `make_compare_video.py` | What it does |
|---|---|
| `--a`, `--b` | The two runs: a tag under `demo/out/`, a run folder, or its `view/` folder. |
| `--label-a`, `--label-b` | The title bars (default `SHIELD ON`, `SHIELD OFF - control`). |
| `--layout stacked\|side` | A above B (default) or side by side. |
| `--panel both\|fpv\|tps` | Each half shows both views (default), the HUD view, or the chase view. |
| `--height N` | Height of each panel in pixels (default 540). |
| `--start-a`, `--start-b` | Mission time each half starts at (default 0, the first tick after take-off). |
| `--fps-a`, `--fps-b`, `--fps` | Capture rates for a run without one in `recorder.json`; the output rate. |
| `--end longest\|shortest`, `--max-s S` | Stop at the longer run's end (default) or the shorter's; cut at S seconds. |
| `--crf N`, `--ffmpeg PATH` | libx264 quality (default 20); the ffmpeg to use (default: on PATH). |
| `--out FILE`, `--force` | Output (default `demo/out/<A>/<A>_vs_<B>_compare.mp4`); an existing file is refused without `--force`. |

`demo/out/pas_ap_video_compare_20261014.mp4` is the 14 Oct pair stacked this way.
`make_compare_video.py` encodes H.264 (libx264); `make_demo_video.py` writes
MPEG-4 Part 2 through OpenCV (`mp4v`), which browsers do not play. The smaller H.264
`_share` and `_phone` files beside the 13-14 Oct videos were made with ffmpeg by
hand; no tool here writes them, and their settings are not recorded.

The explainer's ArduPilot on/off slide takes one still from each run's
`_demo.mp4`, the left (drone camera + HUD) panel:

```bash
ffmpeg -ss 41.15 -i demo/out/pas_ap_video_on_20261014/pas_ap_video_on_20261014_demo.mp4 \
  -frames:v 1 -vf crop=1280:720:0:0 docs/share/2026-10-14-Explainer/figures/ardupilot_shield_on.png
ffmpeg -ss 60.7 -i demo/out/pas_ap_video_off_20261014/pas_ap_video_off_20261014_demo.mp4 \
  -frames:v 1 -vf crop=1280:720:0:0 docs/share/2026-10-14-Explainer/figures/ardupilot_shield_off.png
```
