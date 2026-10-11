# The public progress site (`docs/progress/`)

**Date:** 2026-10-06 (revised 2026-10-07; re-curated 2026-10-08)
**Where it is served:** `docs/` is the GitHub Pages source (`docs/_config.yml`),
so the site is served at `<pages-url>/progress/`. It has been live since
2026-10-08, so every push of `docs/progress/` publishes.
**Code:** `tools/build_public_site.py`, curated text in
`docs/progress/_src/site.json`, styles in `docs/progress/assets/site.css`,
tests in `tests/test_build_public_site.py`.

## Two sites, two audiences

| | Local tracker (`tracker/`) | Public site (`docs/progress/`) |
|---|---|---|
| Reader | the project team | readers outside the lab, first of all ITRI |
| Content | the full working record of the project | progress: what is built, what is shown, how it works, what changed, what comes next |
| Languages | English, Indonesian, Traditional Chinese | English, Traditional Chinese |
| Data | `tracker/data/` (gitignored, local only) | static HTML committed under `docs/progress/` |
| Runs as | `python tracker/serve.py` on 127.0.0.1 | plain files: GitHub Pages, `file://`, or any static server |

The tracker stays the working tool. The public site is generated from a
reviewed subset, never by copying the tracker wholesale.

## Pages

Every page exists in English (`en/`) and Traditional Chinese (`zh/`), with a
language switch on each page and a chooser at `docs/progress/index.html`.

| Page | What it shows |
|---|---|
| Overview | the project in one paragraph, the grant facts, headline figures (scores with their baseline, and counts), the five roles, the four work packages, the latest changes and the next steps |
| How it works | the grant's architecture as a diagram, one Shield tick in six steps, the city demonstration pipeline, an annotated frame of the policy indicator, and the three topologies (dev, hil, flight) |
| WP1-WP4 | per work package: the grant's own words, what is built (with file paths), what is shown (figures, numbers and the command that reproduces them), and what is next |
| Evidence | figures and videos, each with what it shows |
| Changes | a dated timeline taken from `CHANGELOG.md` |
| Roadmap | a February-November timeline, planned work by window, the final-delivery list, and the milestones reached |
| Glossary | the terms the site uses |

## Content policy

1. **Progress, stated exactly.** The site reports what is built and what has
   been shown. Work that is not finished appears only as a **Next** item, on the
   roadmap and on its work-package page. A work package's description states
   its scope ("WP3 delivers ...", "in the grant's design ..."), so it does not
   read as a claim that everything in it is built. Open defects stay in the
   engineering documents, but the site never says or implies they are fixed:
   a run with an open defect is shown only for what it demonstrates (what ran,
   for how long, what the ground station received), none of its safety scores
   is stated without the defect beside it, and a sentence about earlier runs
   is scoped to those runs ("on each of these runs", not "on every run").
   The generator cannot tell a defect from a result, so this part is checked
   by reading at each re-curation, not by the build.
2. **Every number is the current, corrected number.** A number in curated text
   must appear in a source the text cites, and the English and Chinese text
   must carry the same numbers. Quantities are written as digits, so that both
   checks see them. A data file is cited by the path of the value
   (`docs/data/eval_sep2026.json#kpi.runs.ros2_shield_on.nfz_s`), and the
   number must match that value. A CHANGELOG section counts only outside its
   Retracted, Found, Known limitations and Not measured subsections, so a
   figure that survives only in a retraction list supports nothing. A score is
   shown beside its baseline (the Shield-off run beside the Shield-on run, the
   earlier flights beside the new ones, a flight without the no-fly zone beside
   the one with it). A timing is shown with the configuration it was measured
   in. Counts that change daily, such as test totals, are not published.
3. **KPI figures come from the hil topology.** The grant measures every
   reported KPI on the hil topology (VLA and Shield on the Jetson Orin). A
   headline tile on the overview may name an acceptance KPI only when it comes
   from a hil run. Development (dev) runs appear on the work-package pages,
   labelled with their topology, beside the sentence that the grant measures its
   KPIs on hil. Grant quotes are given whole; where the grant names a topology
   in other words than the site's three names, the site's name is given in
   brackets (`[hil]`).
4. **Superseded wording never returns.** The generator refuses a list of
   phrasings and figures the project no longer uses (`FORBIDDEN_TEXT` in the
   generator); add to that list whenever a published statement is updated.
5. **Only public material.** The site carries no internal review outcomes,
   task-tracking identifiers, decision records, meeting records, correction
   lists or working logs.
6. **No personal names except the PI**, as the grant lists him. The build
   refuses the names of commit authors and of the repository owner, and any name
   in an optional local deny list (below), in Latin or Chinese characters.
7. **Simulation is labelled as simulation, and the topology is named.** Runs on
   one desktop with MAVROS 2 are the grant's *dev* topology and are labelled
   so; the city demonstrations up to 3 Oct say that a hand-written test
   controller is the pilot and that Project AirSim's own flight controller
   flies the drone. Since 8 Oct ArduPilot SITL can fly the Project AirSim
   vehicle; such a run names its MAVLink link, and a run that commanded
   ArduPilot over pymavlink says that it is not one of the grant's topologies
   (they all use MAVROS 2). The stress harness's episodes are headless (no
   simulator and no ArduPilot; Loiter, RTL and Land are modelled as ideal
   position control), and a sentence about them says so: they are episodes,
   not flights. The simulator is called Project AirSim, or classic
   AirSim for the July OpenVLA run; bare "AirSim" is refused as ambiguous.
8. **Every image has been looked at.** The text checks cannot read an image, so
   each image is listed in `site.json` `reviewed_images` with the digest of the
   file a person checked against the content policy and against its caption
   (the run it shows, the wording inside it).

## How the policy is enforced

The generator checks everything before it writes anything, and a refusal stops
the build. "Refused" below means `PolicyViolation` or `BuildError`.

| Rule | Mechanism | Test |
|---|---|---|
| Only reviewed tracker fields | `ALLOW_LIST` names every field that may be copied, per section and per language. Anything else, including the Indonesian text, is not copied. | `test_the_extract_copies_allowed_fields_and_nothing_else` |
| Never an internal field | `DISALLOWED_KEYS` (`verdict`, `needs_pi`, `evidence`, `notes`, `retracted`, `decisions`, `status`, ...). An allow-list that names one is refused; so is such a key anywhere in the extract, in `site.json`, or in the page model. | `test_each_named_disallowed_field_in_the_allow_list_is_refused`, `test_a_disallowed_key_anywhere_in_the_build_inputs_is_refused` |
| Public wording replaces tracker wording | `overrides` in `site.json` are applied by `build()` before anything is rendered, and the committed snapshot is written with them applied | `test_overrides_replace_one_field_and_refuse_unknown_paths`, `test_overrides_reach_every_page_and_the_snapshot` |
| No corrected wording, no internal markers | `FORBIDDEN_TEXT`, applied to every string and again, by `build()`, to every rendered file. The meeting rule matches a meeting as an event or record ("at the meeting", "meeting notes"), not the verb ("meeting the 5 ms budget"). | `test_withdrawn_claims_and_internal_markers_are_refused`, `test_neutral_project_wording_is_not_refused`, `test_the_rendered_page_is_scanned_too`, `test_build_scans_every_page_it_renders` |
| No names | commit authors (from `git log`), the owner in the remote URL, an optional deny list; each name as a whole (any whitespace between its words), its Latin words of 3+ letters and its runs of 2+ Chinese characters; only a name made entirely of the PI's name words is exempt | `test_person_names_are_refused_but_the_pi_is_not`, `test_deny_list_names_in_chinese_characters_and_short_names_are_refused`, `test_a_commit_author_from_git_is_refused_in_the_build`, `test_the_repository_owner_named_in_the_remote_is_refused`, `test_the_pi_exemption_needs_every_word_of_his_name` |
| The name check cannot switch itself off | no git is an error; `--no-git` builds without it for a local look only | `test_git_is_required_unless_the_build_is_told_otherwise` |
| Traditional Chinese | the Simplified-only character list from `tools/add_update.py`, on every page | `test_simplified_characters_are_refused_in_chinese_text_only` |
| Numbers have a source | `check_numbers`: each number must appear in a cited source (a percentage may match its fraction, 98.5 % against 0.985); en and zh must agree | `test_a_number_must_appear_in_the_section_it_cites` |
| A data file supports only the value cited | a JSON source must be cited as `file.json#path`; the number must match that value, to the precision written (0.6265 for 0.626506) | `test_a_json_source_is_cited_by_the_path_of_its_value` |
| Counts are digits | `NUMBER_WORDS`: "three runs" or "三次" in curated text is refused, so a count cannot change unseen | `test_quantities_in_curated_text_are_written_as_digits` |
| A retraction certifies nothing | CHANGELOG numbers are read from the section without its Retracted, Found, Known limitations and Not measured subsections (and headings nested under them) | `test_a_number_only_in_a_retracted_subsection_supports_nothing` |
| Tracker text agrees across languages | `check_parity`: the same numbers in the en and zh tracker text | `test_tracker_text_must_carry_the_same_numbers_in_both_languages` |
| No dev run as a headline KPI | `check_highlights`: a tile naming an acceptance KPI needs `"topology": "hil"` | `test_a_headline_tile_may_not_present_a_dev_run_as_a_kpi` |
| Overrides fit their field | a text field takes `{"en", "zh"}`, a plain field (`grant_quote`) one string; unknown items and fields are refused | `test_overrides_replace_one_field_and_refuse_unknown_paths`, `test_the_wp4_grant_quote_is_complete_and_names_hil` |
| Paths are real | every file path a page names, and the script of every Reproduce command, must exist | `test_a_code_path_that_does_not_exist_is_refused`, `test_a_reproduce_command_must_name_a_script_that_exists` |
| Images are on Pages, and reviewed | an image under `docs/img/` must be tracked by git; a copy under `docs/progress/assets/img/` is committed with the pages; every image must be in `reviewed_images` with its current digest, and every entry there must be used | `test_videos_link_only_when_committed_and_small`, `test_every_image_is_reviewed_and_unchanged` |
| Next is labelled Next | every planned item on a work-package page carries the Next label; no heading names limitations or risks | `test_no_heading_lists_obstacles_and_open_work_is_labelled_next` |
| Pages are generated, not edited | the committed pages equal a fresh build; a page the generator no longer makes is deleted | `test_the_committed_pages_match_a_fresh_build`, `test_a_page_the_generator_no_longer_makes_is_removed` |
| The harness is labelled headless | a curated sentence saying the grant's event types run in the middle of a flight or episode must say "headless" in English and 無模擬器 in Chinese; the wording published on 2026-10-08 is the null | `test_the_harness_events_are_labelled_headless` |
| Unfinished harness work stays Next | while `sitl_refusal()` in `experiments/sweep_scenarios.py` refuses a scenario with events, the roadmap must carry a Next item for flying such scenarios on ArduPilot SITL (the item was dropped on 2026-10-08 as if done) | `test_the_roadmap_shows_event_scenarios_on_sitl_while_the_harness_refuses_them` |
| The site is not older than the CHANGELOG | every build records a digest of each CHANGELOG section it watches (every cited section, every undated section, every section dated on or after `as_of`) in `_src/build_manifest.json`. `--check` exits 1 when a page differs, when a watched section changed since the last build (new work is often written under an existing dated heading), or when a section is dated after `as_of` | `test_check_fails_on_an_edited_page_and_on_a_grown_changelog_section`, `test_changelog_sections_newer_than_as_of_are_reported` |

Each refusal test has a matching case that must pass (neutral wording, the
PI's name, a correct number, a rounded value, a reviewed image, a hil tile), so
a generator that refused everything would fail too.

Mutation check (2026-10-07): each guard below was switched off in a scratch copy
of the generator, and the full test file was run against it. Every mutant made
at least one test fail: the override step in `build()`, the overrides in the
written snapshot, `--check` returning 1 (and its CHANGELOG-digest part), the
watched-section selection, stale-page deletion, the remote-owner name, the
PI exemption needing every word, `inline()` escaping, the Simplified check on
every page, tracker en/zh parity, embedding only committed videos,
`sections_after`, `number_in`, the whole-name and per-word name forms,
`check_highlights`, `check_files` and its command scripts, the image-tracked
check and its exemption for the site's own copies, the image review and its
unused-entry check, the claims-only CHANGELOG text, the e-mail rule, the
disallowed-key walk in the extract, the number-word check, JSON path citation
and its rounding tolerance, and the narrowed meeting and AirSim rules.

## Re-curation of 2026-10-08

The site went live on 8 Oct showing "as of 6 Oct" and listing the work of
7 Oct as Next. `--check` named both new CHANGELOG sections. What changed:

- `as_of` 2026-10-08; change entries for 2026-10-07 and 2026-10-08.
- WP1: the grant's rule format, policy layers, rule changes during a flight
  and the GeoJSON/KML converter move to Built; two Shown items with both
  nulls from `docs/data/wp1_roundtrip.json` (35/35 malformed policies refused
  against 20/35 on 5 Oct and 0/35 for a loader that checks nothing; 3/3
  reference documents loaded here, against 0/3). The second Shown item says
  the reference loader reads "the 3 bundles we write from" the reference's
  documents, not "ours": the cited key comes from
  `reference_loader_reads_our_reference_form_bundle`, and our own 29 policies
  use rule types the reference lacks (`reference_hash_representable` 0/29), so
  "ours load there" claimed interoperability nobody has shown. The 7 Oct
  change entry gives the same baseline as the Shown item (20/35 on 5 Oct,
  14/25 of the 25 earlier cases), not 14/25 alone.
- WP3: the grant's ROS 2 interface, escalation inside the Shield, ArduPilot's
  GeoFence as backstop, and Project AirSim flown by ArduPilot with a stream to
  Mission Planner's UDP port 14550 move to Built. Mission Planner itself was
  never run (a listener stood in on its port), so neither the item nor the
  8 Oct change entry says Mission Planner was attached, and the glossary's
  mavlink-router entry separates the grant's design (Mission Planner and
  MAVROS 2) from what the router has fed on this rail (Mission Planner's port
  and the perception node). The Reproduce command of the stream item shows
  both steps, `listen` and `check`; the capture behind 19.0 Hz is not stored
  with the run (`docs/DESIGN-projectairsim-ardupilot.md` says where it is).
  Shown gains Gate G1 (with
  `docs/img/g1_pas_ardupilot_airborne.png`, reviewed: byte-identical to run 1's
  chase frame, no text inside it) and the Mission Planner stream of the 8 Oct
  follow, labelled as a pymavlink run. That run's P0 escapes and Loiter
  escalations are open defects: the site states none of its safety scores
  (`docs/DESIGN-projectairsim-ardupilot.md`, "Measured 2026-10-08", has them).
  The SITL escape-rate sentence is scoped to its own runs. The live site had
  said "the P0 violation escape rate is 0.00 on every run" (and the Chinese
  equivalent); under content policy 4 that wording is now in
  `FORBIDDEN_TEXT`, so an unscoped zero cannot come back. The first version
  of that rule matched the published wording only; it now matches the rate,
  a zero and every/all runs or flights in either order within one clause, in
  both languages, and lets "all of these runs" and a non-zero rate such as
  0.0053 pass. The published `site.json` of 10451bc is refused by it; the
  generator of 10451bc accepted all 13 `WITHDRAWN` escape-rate phrases, and
  the first version of the rule accepted 10 of them.
- WP4: scenario templates and events during an episode, and the Orin image
  definitions and profiler, move to Built. The templates item and the 7 Oct
  change entry say the events run in the middle of a headless episode (no
  simulator; Loiter, RTL and Land modelled as ideal position control): every
  stored sweep and nightly manifest names the topology `headless-kinematic`,
  and the harness's ArduPilot path refuses every scenario with events. The
  first re-curation said "in the middle of a flight", which left the
  simulation unlabelled and read as RTL and Land seen on ArduPilot. The Orin item says "image
  definitions" and a profiler "written to run on the Orin unchanged": no
  image has been built and no Orin has run it yet, so "ready for the Orin"
  and "runs on the Orin" would overstate it.
- A G1 headline tile (p95 EKF error 0.81-0.82 m against a 1.0 m limit and a
  frozen-position null of 18.9-19.0 m) replaces the paraphrase count, which
  was a count rather than a measurement.
- Roadmap: the done items leave Next; Next is now the Orin (hil) setup and
  timing, the perception node through MAVROS 2, sentence templates for the
  mid-flight rule types (at `as_of` the flight loaders still refuse a policy
  that declares one, `guardrail.models.RUNTIME_TYPES`), OpenVLA with the rule
  summary, the broad stress profile, then the November KPI campaign on the
  Orin and, after the hil runs, the drone. Two Next items were added on
  review. Flying the scenarios with mid-flight rule changes on ArduPilot SITL
  through MAVROS 2 (WP4): the first re-curation dropped the old item "a stress
  harness that starts ArduPilot SITL sessions, changes rules mid-flight ..."
  as if it were done, but `sitl_refusal()` still refuses every scenario with
  events, the rule change flown mid-flight on SITL so far is the ROS 2 node's
  own no-fly-zone spawn (`--dynamic`, outside the harness), and the November
  campaign of dynamic scenarios depends on it. And keeping the aircraft in its altitude band while the escalation holds
  it on ArduPilot, with fail-safe trigger correctness brought to the grant's
  99 % (WP3): no current figure is stated, only the target.
- Topologies: dev keeps the Shield on/off runs; Gate G1 is said to have run
  on the same desktop, not to be a dev-topology run, because it had neither
  the Shield nor MAVROS 2.
- The architecture's ground-station box links to WP3; the glossary gains
  Gate G1 and mavlink-router, and the Project AirSim entry says who flies the
  vehicle before and after 8 Oct.
- The tracker's demo-pipeline autopilot text changed on 8 Oct to "the older
  demo path still uses AirSim's controller", which the build refuses (bare
  "AirSim"). `overrides` keep the pipeline's autopilot box consistent with the
  pipeline's lead, which describes the chain as flown up to 3 Oct (identity
  check, tracker and policy indicator, none of which the 8 Oct ArduPilot
  follow used), and point to WP3 for ArduPilot flying the vehicle.
- `assets/site.css`: on a 375 px screen the roadmap's "Today" label sat
  inside the final-gate date label (overlap 552 px² in English, 394 px² in
  Chinese, measured in the browser); the gate label now sits one line higher
  (overlap 0 at 375 px and at the pane's 626 px).
  `test_the_roadmap_gate_label_sits_above_the_today_label` reads a 0 px gap
  from the stylesheet of 10451bc (it fails there) and 20 px from this one.
- `docs/index.md` (the Pages root) titles its escape-rate section "on the
  mid-evaluation flights" and states the 8 Oct follow's 5 P0 stand-off escapes
  (0.0053, no Shield-off control on that rail yet) beside the 41 flights'
  0.0.
- Checked after the rebuild: `--check` 21/21 pages up to date; every page,
  link, anchor and image served with HTTP 200 from `python -m http.server
  -d docs`; no page wider than a 375 px viewport.

## Sources

- **`docs/progress/_src/site.json`** holds all curated wording: highlights,
  work-package items, gallery captions, change entries, roadmap, topologies, the
  extra glossary terms, `overrides` that replace a copied tracker field with
  public wording, and `reviewed_images`. Each item that states a number has a
  `src` list. A change entry names its CHANGELOG section with `changelog`
  (`2026-10-06`, `2026-09-29 (evening)`, `0.5.0`), which is its source
  automatically; a point that adds its own `src` lists the section too.
- **`tracker/data/content_overview.json`, `content_flow.json`** supply the
  explanation and diagram text through `ALLOW_LIST`. Because `tracker/data/` is
  gitignored, every build that reads it also writes the copied fields, with the
  overrides applied, to `docs/progress/_src/tracker_extract.json`. A clone
  without the tracker data builds from that snapshot, and the tests do.
- **`CHANGELOG.md`** anchors the timeline. A change entry whose section does not
  exist is refused. Retraction subsections are never published, and never
  used as the source of a number.
- **`docs/img/`** images are referenced in place (`../../img/...`). Where a
  file's name would mislabel what it shows on a public URL, the site keeps a
  byte-identical copy under a neutral name in `docs/progress/assets/img/`
  (`sitl_nfz_off_on.png`, a dev-topology run).

`_src/` starts with an underscore, so Jekyll does not serve it. The generated
HTML files have no front matter, so Jekyll copies them unchanged.

## Videos

`docs/video/` is gitignored, so no video is committed today. A video is
embedded only when it is tracked by git and at most `MAX_VIDEO_BYTES` (10 MB).
Otherwise the page shows a poster image and a note that names the file and its
size, as recorded in `site.json` (the build refuses a recorded size that
differs from the file on disk). To publish a video, either commit a small
re-encode under `docs/` or upload it as a GitHub release asset; do not commit
the large originals.

## How to update the site

1. **Re-curate before every publish.** Other work lands in the repository
   between publishes, and the build can only compare the pages with their own
   sources, not with the code. `python tools/build_public_site.py --check`
   names the CHANGELOG sections that changed since the last build. Read each
   work package's Built, Shown and Next items in `docs/progress/_src/site.json`
   against them and against the code: move finished Next items to Built, add
   the new change entry, update counts and lists that the new code changed
   (rule types, profiles, file paths), and set `as_of` to the date of the
   newest CHANGELOG section shown. A rebuild records the new section digests,
   so do this reading before the rebuild, not after.
2. Edit `docs/progress/_src/site.json`. Keep each English and Chinese pair
   saying the same thing with the same numbers, write quantities as digits, and
   give every number a `src`. Prefer a value in a data file under `docs/data/`
   (`file.json#path.to.value`) or a result document; a CHANGELOG section counts
   only outside its Retracted, Found, Known limitations and Not measured
   subsections. For a new change entry, add an item to `changes` with the
   CHANGELOG section's anchor.
3. For a new or changed image, look at it: does it show the run its caption
   names, and does any text inside it use wording the project has corrected?
   Then add or update its line in `reviewed_images` with the digest the build
   prints.
4. Optionally refresh the tracker content (`tracker/data/content_*.json`); the
   next build copies the allow-listed fields again.
5. Run `python tools/build_public_site.py`. If it prints `REFUSED`, fix what it
   names; it never writes a partial site.
6. Run `python tests/test_build_public_site.py`.
7. Preview: `python -m http.server -d docs 8000`, then open
   `http://127.0.0.1:8000/progress/`. Opening `docs/progress/index.html`
   directly from disk also works.
8. Commit `docs/progress/` together with the sources that changed.

`--check` is what a CI step or a pre-publish check would run. The build needs
git (for the author-name check and the committed-image check); `--no-git`
builds without them, for a local look only, never for publishing.

To add names that must never appear (colleagues, for example), list them one
per line in `tracker/data/_public_deny_names.txt`. That folder is gitignored,
so the list itself is never published.
